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
class MemCntrlModel::RequestDispatcherProcess : public kernel::Process {
 public:
  RequestDispatcherProcess(kernel::Kernel* k, const std::string& name,
                           MemCntrlModel* model)
      : Process(k, name), model_(model) {}

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
    if (t.has_requester()) {
      MessageQueue* mq = t.winner();

      const MemCmdMsg* cmdmsg = static_cast<const MemCmdMsg*>(mq->dequeue());
      switch (cmdmsg->opcode()) {
        case MemCmdOpcode::Write:
        case MemCmdOpcode::Read: {
          const bool is_read = (cmdmsg->opcode() == MemCmdOpcode::Read);
          MemRspMsg* rspmsg = new MemRspMsg;
          rspmsg->set_opcode(is_read ? MemRspOpcode::ReadOkay
                                     : MemRspOpcode::WriteOkay);
          rspmsg->set_t(cmdmsg->t());

          // Memory read command
          NocMsg* nocmsg = new NocMsg();
          nocmsg->set_payload(rspmsg);
          nocmsg->set_origin(model_);
          nocmsg->set_dest(cmdmsg->dest());
          nocmsg->set_t(cmdmsg->t());
          // Issue to NOC
          MessageQueueProxy* mem_noc__msg_q = model_->mem_noc__msg_q();
          mem_noc__msg_q->issue(nocmsg);

          cmdmsg->release();
        } break;
        default: {
          LogMessage lmsg("Invalid message opcode received: ");
          lmsg.append(to_string(cmdmsg->opcode()));
          lmsg.level(Level::Fatal);
          log(lmsg);
        } break;
      }

      LogMessage lm("Execute message: ");
      lm.append(cmdmsg->to_string());
      lm.level(Level::Debug);
      log(lm);
    } else {
      wait_on(rdis_arb->request_arrival_event());
    }
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
