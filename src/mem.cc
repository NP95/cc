//========================================================================== //
// Copyright (c) 2020, Stephen Henry
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//========================================================================== //

#include "mem.h"

#include "noc.h"
#include "utility.h"

namespace cc {

//
//
const char* to_string(MemCmdOpcode opcode) {
  switch (opcode) {
    case MemCmdOpcode::Read:
      return "Read";
    case MemCmdOpcode::Write:
      return "Write";
    default:
      return "Invalid";
  }
}

//
//
const char* to_string(MemRspOpcode opcode) {
  switch (opcode) {
    case MemRspOpcode::ReadOkay:
      return "ReadOkay";
    case MemRspOpcode::WriteOkay:
      return "WriteOkay";
    default:
      return "Invalid";
  }
}

//
//
MemCmdMsg::MemCmdMsg() : Message(MessageClass::MemCmd) {}

//
//
std::string MemCmdMsg::to_string() const {
  using cc::to_string;

  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("opcode", to_string(opcode()));
  r.add_field("dest", dest()->path());
  return r.to_string();
}

//
//
MemRspMsg::MemRspMsg() : Message(MessageClass::MemRsp) {}

//
//
std::string MemRspMsg::to_string() const {
  using cc::to_string;

  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("cls", to_string(cls()));
  r.add_field("opcode", to_string(opcode()));
  return r.to_string();
}

//
//
std::string DtMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

//
//
std::string DtRspMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

//
//
class MemCntrlModel::RequestDispatcherProcess : public AgentProcess {
 public:
  RequestDispatcherProcess(kernel::Kernel* k, const std::string& name,
                           MemCntrlModel* model)
      : AgentProcess(k, name), model_(model) {}

 private:
  // Initialization
  void init() override {
    MQArb* rdis_arb = model_->rdis_arb();
    wait_on(rdis_arb->request_arrival_event());
  }

  // Evaluation
  void eval() override {
    MQArb* rdis_arb = model_->rdis_arb();
    MQArbTmt t = rdis_arb->tournament();

    if (!t.has_requester()) {
      // If no requesters, block until a requester has arrived.
      wait_on(rdis_arb->request_arrival_event());
      return;
    }

    const MemCmdMsg* cmdmsg =
        static_cast<const MemCmdMsg*>(t.winner()->dequeue());

    LogMessage lm("Execute message: ");
    lm.append(cmdmsg->to_string());
    lm.level(Level::Debug);
    log(lm);

    MemRspMsg* rspmsg = new MemRspMsg;
    rspmsg->set_t(cmdmsg->t());
    switch (cmdmsg->opcode()) {
      case MemCmdOpcode::Read: {
        rspmsg->set_opcode(MemRspOpcode::ReadOkay);
      } break;
      case MemCmdOpcode::Write: {
        rspmsg->set_opcode(MemRspOpcode::WriteOkay);
      } break;
      default: {
        LogMessage lmsg("Invalid message opcode received: ");
        lmsg.append(to_string(cmdmsg->opcode()));
        lmsg.level(Level::Fatal);
        log(lmsg);
      } break;
    }
    issue_emit_to_noc(cmdmsg->origin(), rspmsg);
    // Discard message
    cmdmsg->release();
    t.advance();

    t = rdis_arb->tournament();
    if (t.has_requester()) {
      wait_for(kernel::Time{10, 0});
    } else {
      wait_on(rdis_arb->request_arrival_event());
    }
  }

  void issue_emit_to_noc(Agent* dest, const Message* msg) {
    NocMsg* nocmsg = new NocMsg();
    nocmsg->set_payload(msg);
    nocmsg->set_origin(model_);
    nocmsg->set_dest(dest);
    // Issue to NOC
    MessageQueueProxy* mq = model_->mem_noc__msg_q();
    mq->issue(nocmsg);
  }

  //
  MemCntrlModel* model_ = nullptr;
};

//
//
class MemNocEndpoint : public NocEndpoint {
 public:
  //
  MemNocEndpoint(kernel::Kernel* k, const std::string& name)
      : NocEndpoint(k, name)
  {}
  //
  void register_agent(Agent* agent, MessageQueueProxy* proxy) {
    endpoints_.insert(std::make_pair(agent, proxy));
  }
  //
  MessageQueueProxy* lookup_mq(const Message* msg) const override {
    if (auto it = endpoints_.find(msg->origin()); it != endpoints_.end()) {
      return it->second;
    } else {
      LogMessage lm("End point not register for origin: ");
      lm.append(msg->origin()->path());
      lm.level(Level::Fatal);
      log(lm);
    }
    return nullptr;
  }

 private:
  //
  std::map<Agent*, MessageQueueProxy*> endpoints_;
};

MemCntrlModel::MemCntrlModel(kernel::Kernel* k) : Agent(k, "mem") { build(); }

MemCntrlModel::~MemCntrlModel() {
  // Cleanup request dispatcher thread.
  delete rdis_proc_;
  delete rdis_arb_;
  for (const std::pair<Agent*, MessageQueue*>& pp : rdis_mq_) {
    MessageQueue* mq = pp.second;
    delete mq;
  }
  delete noc_endpoint_;
  for (MessageQueueProxy* p : endpoints_) { delete p; }
  delete mem_noc__msg_q_;
}

void MemCntrlModel::build() {
  // NOC endpoint
  noc_endpoint_ = new MemNocEndpoint(k(), "noc_ep");
  add_child_module(noc_endpoint_);
  // Request dispatcher process:
  rdis_proc_ = new RequestDispatcherProcess(k(), "rdis", this);
  add_child_process(rdis_proc_);
  // Construct arbiter
  rdis_arb_ = new MQArb(k(), "arb");
  add_child_module(rdis_arb_);
}

void MemCntrlModel::register_agent(Agent* agent) {
  const std::string mq_name = agent->name() + "_mq";
  MessageQueue* mq = new MessageQueue(k(), mq_name, 3);
  add_child_module(mq);
  rdis_mq_.insert(std::make_pair(agent, mq));
}

void MemCntrlModel::elab() {
  for (const std::pair<Agent*, MessageQueue*>& pp : rdis_mq_) {
    MessageQueue* mq = pp.second;
    rdis_arb_->add_requester(mq);
  }

  for (const std::pair<Agent*, MessageQueue*> pp : rdis_mq_) {
    MessageQueueProxy* proxy = pp.second->construct_proxy();
    noc_endpoint_->register_agent(pp.first, proxy);
    endpoints_.push_back(proxy);
  }
}

void MemCntrlModel::set_mem_noc__msg_q(MessageQueueProxy* mq) {
  mem_noc__msg_q_ = mq;
  add_child_module(mem_noc__msg_q_);
}

void MemCntrlModel::drc() {
  if (mem_noc__msg_q_ == nullptr) {
    LogMessage msg("NOC egress message queue has not been bound");
    msg.level(Level::Fatal);
    log(msg);
  }
}

MessageQueue* MemCntrlModel::endpoint() const {
  return noc_endpoint_->ingress_mq();
}

}  // namespace cc
