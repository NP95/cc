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

  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
    r.add_field("opcode", to_string(opcode()));
    r.add_field("dest", dest()->path());
  }
  return ss.str();
}

//
//
MemRspMsg::MemRspMsg() : Message(MessageClass::MemRsp) {}

//
//
std::string MemRspMsg::to_string() const {
  using cc::to_string;

  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
    r.add_field("cls", to_string(cls()));
    r.add_field("opcode", to_string(opcode()));
  }
  return ss.str();
}

//
//
class MemCntrlModel::NocIngressProcess : public kernel::Process {
 public:
  NocIngressProcess(kernel::Kernel* k, const std::string& name,
                    MemCntrlModel* model)
      : kernel::Process(k, name), model_(model) {}

 private:
  // Initialization
  void init() override {
    MessageQueue* mq = model_->noc_mem__msg_q();
    wait_on(mq->request_arrival_event());
  }

  // Elaboration
  void eval() override {
    // Upon reception of a NOC message, remove transport layer
    // encapsulation and issue to the appropriate ingress queue.
    MessageQueue* noc_mq = model_->noc_mem__msg_q();
    const NocMsg* msg = static_cast<const NocMsg*>(noc_mq->dequeue());

    // Validate message
    if (msg->cls() != MessageClass::Noc) {
      LogMessage lmsg("Received invalid message class: ");
      lmsg.append(to_string(msg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    MessageQueue* iss_mq = model_->lookup_rdis_mq(msg->origin());
    if (iss_mq == nullptr) {
      LogMessage lmsg("Message queue not found for agent: ");
      lmsg.append(msg->dest()->path());
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    // Forward message message to destination queue and discard
    // encapsulation/transport message.
    iss_mq->push(msg->payload());
    msg->release();

    // Set conditions for subsequent re-evaluations.
    if (!noc_mq->empty()) {
      // TODO:Cleanup
      // Wait some delay
      wait_for(kernel::Time{10, 0});
    } else {
      // Not further work; await until noc ingress queue becomes non-full.
      wait_on(noc_mq->request_arrival_event());
    }
  }

  // Finalization
  void fini() override {}

  // Pointer to owning Mem instance.
  MemCntrlModel* model_ = nullptr;
};

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
    MessageQueueArbiter* rdis_arb = model_->rdis_arb();
    wait_on(rdis_arb->request_arrival_event());
  }

  // Evaluation
  void eval() override {
    using Tournament = MessageQueueArbiter::Tournament;
    using Interface = MessageQueueArbiter::Interface;

    MessageQueueArbiter* rdis_arb = model_->rdis_arb();
    Tournament t = rdis_arb->tournament();
    if (t.has_requester()) {
      Interface* intf = t.intf();

      const MemCmdMsg* cmdmsg = static_cast<const MemCmdMsg*>(intf->dequeue());
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
          MessageQueue* mem_noc__msg_q = model_->mem_noc__msg_q();
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
      lm.level(Level::Info);
      log(lm);
    } else {
      wait_on(rdis_arb->request_arrival_event());
    }
  }

  // Finalization
  void fini() override {}

  //
  MemCntrlModel* model_ = nullptr;
};

MemCntrlModel::MemCntrlModel(kernel::Kernel* k) : Agent(k, "mem") { build(); }

MemCntrlModel::~MemCntrlModel() {
  // Delete queues.
  delete noc_mem__msg_q_;
  // Cleanup NOC ingress thread.
  delete noci_proc_;
  // Cleanup request dispatcher thread.
  delete rdis_proc_;
  delete rdis_arb_;
  for (const std::pair<Agent*, MessageQueue*>& pp : rdis_mq_) {
    MessageQueue* mq = pp.second;
    delete mq;
  }
}

void MemCntrlModel::build() {
  // NOC -> Mem message queue:
  noc_mem__msg_q_ = new MessageQueue(k(), "noc_mem__msg_q", 3);
  add_child_module(noc_mem__msg_q_);

  // NOC ingress process:
  noci_proc_ = new NocIngressProcess(k(), "noci", this);
  add_child_process(noci_proc_);

  // Request dispatcher process:
  rdis_proc_ = new RequestDispatcherProcess(k(), "rdis", this);
  add_child_process(rdis_proc_);
  // Construct arbiter
  rdis_arb_ = new MessageQueueArbiter(k(), "arb");
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
}

void MemCntrlModel::drc() {
  if (mem_noc__msg_q_ == nullptr) {
    LogMessage msg("NOC egress message queue has not been bound");
    msg.level(Level::Fatal);
    log(msg);
  }
}

MessageQueue* MemCntrlModel::lookup_rdis_mq(Agent* agent) {
  MessageQueue* mq = nullptr;
  std::map<Agent*, MessageQueue*>::iterator it = rdis_mq_.find(agent);
  if (it != rdis_mq_.end()) {
    mq = it->second;
  }
  return mq;
}

}  // namespace cc
