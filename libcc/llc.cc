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

#include "llc.h"
#include "msg.h"
#include "noc.h"
#include "mem.h"
#include "dir.h"
#include "utility.h"

namespace cc {

const char* to_string(LLCCmdOpcode opcode) {
  switch (opcode) {
    case LLCCmdOpcode::Fill: return "Fill";
    case LLCCmdOpcode::Evict: return "Evict";
    case LLCCmdOpcode::PutLine: return "PutLine";
    default: return "Invalid";
  }
}

//
//
LLCCmdMsg::LLCCmdMsg() : Message(MessageClass::LLCCmd) {}


std::string LLCCmdMsg::to_string() const {
  using cc::to_string;
  
  std::stringstream ss;
  {
    Hexer h;
    KVListRenderer r(ss);
    render_msg_fields(r);
    r.add_field("cls", to_string(cls()));
    r.add_field("opcode", to_string(opcode()));
    r.add_field("addr", h.to_hex(addr()));
  }
  return ss.str();
}

const char* to_string(LLCRspOpcode opcode) {
  switch (opcode) {
    case LLCRspOpcode::Okay: return "Okay";
    default: return "Invalid";
  }
}


//
//
LLCCmdRspMsg::LLCCmdRspMsg() : Message(MessageClass::LLCCmdRsp) {}


std::string LLCCmdRspMsg::to_string() const {
  using cc::to_string;
  
  std::stringstream ss;
  {
    Hexer h;
    KVListRenderer r(ss);
    render_msg_fields(r);
    r.add_field("cls", to_string(cls()));
    r.add_field("opcode", to_string(opcode()));
  }
  return ss.str();
}

//
//
class LLCModel::NocIngressProcess : public kernel::Process {
 public:
  NocIngressProcess(kernel::Kernel* k, const std::string& name, LLCModel* model)
      : Process(k, name), model_(model) {
  }

 private:
  // Initialization
  void init() override {
    MessageQueue* mq = model_->noc_llc__msg_q();
    wait_on(mq->request_arrival_event());
  }

  // Elaboration
  void eval() override {
    using cc::to_string;
    
    // Upon reception of a NOC message, remove transport layer
    // encapsulation and issue to the appropriate ingress queue.
    MessageQueue* noc_mq = model_->noc_llc__msg_q();
    const NocMsg* nocmsg = static_cast<const NocMsg*>(noc_mq->dequeue());

    // Validate message
    if (nocmsg->cls() != MessageClass::Noc) {
      LogMessage lmsg("Received invalid message class: ");
      lmsg.append(to_string(nocmsg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    const Message* msg = nocmsg->payload();
    MessageQueue* iss_mq = model_->lookup_rdis_mq(msg->cls());
    if (iss_mq == nullptr) {
      LogMessage lmsg("Message queue not found for class: ");
      lmsg.append(to_string(msg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    // Forward message message to destination queue and discard
    // encapsulation/transport message.
    iss_mq->push(msg);
    nocmsg->release();

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
  void fini() override {
  }
  
  LLCModel* model_ = nullptr;
};

//
//
class LLCModel::RdisProcess : public kernel::Process {
  using Tournament = MessageQueueArbiter::Tournament;

  enum class State {
    Idle, ProcessMessage, ExecuteActions
  };

  static const char* to_string(State s) {
    switch (s) {
      case State::Idle: return "Idle";
      case State::ProcessMessage: return "ProcessMessage";
      case State::ExecuteActions: return "ExecuteActions";
      default: return "Invalid";
    }
  }

 public:
  RdisProcess(kernel::Kernel* k, const std::string& name, LLCModel* model)
      : kernel::Process(k, name), model_(model) {
    set_state(State::Idle);
  }

  State state() const { return state_; }
  void set_state(State state) {
    if (state_ != state) {
      LogMessage msg("State transition: ");
      msg.level(Level::Debug);
      msg.append(to_string(state_));
      msg.append(" -> ");
      msg.append(to_string(state));
      log(msg);
    }
    state_ = state;
  }

 private:

  // Initialization
  void init() override {
    set_state(State::Idle);

    MessageQueueArbiter* arb = model_->arb();
    wait_on(arb->request_arrival_event());
  }

  // Elaboration
  void eval() override {
    MessageQueueArbiter* arb = model_->arb();
    Tournament t;
    bool commit = false;

    t = arb->tournament();
    if (t.has_requester()) {
      const Message* msg = t.intf()->peek();
      switch (msg->cls()) {
        case MessageClass::LLCCmd: {
          const LLCCmdMsg* llccmd = static_cast<const LLCCmdMsg*>(msg);
          commit = eval_handle_llc_cmd(llccmd);
        } break;
        case MessageClass::MemRsp: {
          const MemRspMsg* memrsp = static_cast<const MemRspMsg*>(msg);
          commit = eval_handle_mem_rsp(memrsp);
        } break;
        default: {
          using cc::to_string;
          // Unknown message class
          LogMessage lm("Unknown message received: ");
          lm.append(to_string(msg->cls()));
          lm.level(Level::Fatal);
          log(lm);
        } break;
      }
      if (commit) {
        // Discard message and advance arbitration state.
        const Message* msg = t.intf()->dequeue();
        t.advance();
        msg->release();
      }
    }

    if (commit) {
      t = arb->tournament();
      if (t.has_requester()) {
      } else {
        wait_on(arb->request_arrival_event());
      }
    }
  }

  bool eval_handle_llc_cmd(const LLCCmdMsg* msg) {
    bool commit = true;
    switch (msg->opcode()) {
      case LLCCmdOpcode::Fill: {
        // Message to LLC
        MemCmdMsg* memcmd = new MemCmdMsg;
        memcmd->set_opcode(MemCmdOpcode::Read);
        memcmd->set_dest(model_);
        memcmd->set_t(msg->t());
    
        NocMsg* nocmsg = new NocMsg;
        nocmsg->set_origin(model_);
        nocmsg->set_dest(model_->mc());
        nocmsg->set_payload(memcmd);

        MessageQueue* mq = model_->llc_noc__msg_q();
        mq->issue(nocmsg);
      } break;
      case LLCCmdOpcode::Evict: {
      } break;
      case LLCCmdOpcode::PutLine: {
      } break;
      default: {
      } break;
    }
    return commit;
  }

  bool eval_handle_mem_rsp(const MemRspMsg* msg) {
    bool commit = true;
    switch (msg->opcode()) {
      case MemRspOpcode::ReadOkay: {
        LLCCmdRspMsg* llcrsp = new LLCCmdRspMsg;
        llcrsp->set_t(msg->t());
        llcrsp->set_opcode(LLCRspOpcode::Okay);

        NocMsg* nocmsg = new NocMsg;
        nocmsg->set_origin(model_);
        nocmsg->set_dest(model_->dir());
        nocmsg->set_payload(llcrsp);

        MessageQueue* mq = model_->llc_noc__msg_q();
        mq->issue(nocmsg);
      } break;
      case MemRspOpcode::WriteOkay: {
      } break;
      default: {
      } break;
    }
    return commit;
  }

  // Finalization
  void fini() override {
  }

  // Current machine state
  State state_;
  // Pointer to owning LLC instance.
  LLCModel* model_ = nullptr;
};

LLCModel::LLCModel(kernel::Kernel* k, const LLCModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

LLCModel::~LLCModel() {
  delete noc_llc__msg_q_;
  delete dir_llc__cmd_q_;
  delete mem_llc__rsp_q_;
  delete arb_;
  delete rdis_proc_;
  delete noci_proc_;
}

void LLCModel::build() {
  // NOC -> LLC message queue:
  noc_llc__msg_q_ = new MessageQueue(k(), "noc_llc__msg_q", 3);
  add_child_module(noc_llc__msg_q_);
  // DIR -> LLC command queue
  dir_llc__cmd_q_ = new MessageQueue(k(), "dir_llc__cmd_q", 3);
  add_child_module(dir_llc__cmd_q_);
  // MEM -> LLC response queue
  mem_llc__rsp_q_ = new MessageQueue(k(), "mem_llc__rsp_q", 3);
  add_child_module(mem_llc__rsp_q_);
  // Construct arbiter
  arb_ = new MessageQueueArbiter(k(), "arb");
  add_child_module(arb_);
  // Construct main thread
  rdis_proc_ = new RdisProcess(k(), "main", this);
  add_child_process(rdis_proc_);
  // NOC Ingress process.
  noci_proc_ = new NocIngressProcess(k(), "noci", this);
  add_child_process(noci_proc_);
}

void LLCModel::elab() {
  arb_->add_requester(dir_llc__cmd_q_);
  arb_->add_requester(mem_llc__rsp_q_);
}

void LLCModel::drc() {
  if (llc_noc__msg_q_ == nullptr) {
    LogMessage msg("LLC to NOC egress queue has not been bound.");
    msg.level(Level::Fatal);
    log(msg);
  }
}

MessageQueue* LLCModel::lookup_rdis_mq(MessageClass cls) const {
  switch (cls) {
    case MessageClass::LLCCmd: return dir_llc__cmd_q_;
    case MessageClass::MemRsp: return mem_llc__rsp_q_;
    default: return nullptr;
  }
}

} // namespace cc
