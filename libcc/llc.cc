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

#include "dir.h"
#include "mem.h"
#include "msg.h"
#include "noc.h"
#include "utility.h"

namespace cc {

const char* to_string(LLCCmdOpcode opcode) {
  switch (opcode) {
    case LLCCmdOpcode::Fill:
      return "Fill";
    case LLCCmdOpcode::Evict:
      return "Evict";
    case LLCCmdOpcode::PutLine:
      return "PutLine";
    default:
      return "Invalid";
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
    case LLCRspOpcode::Okay:
      return "Okay";
    default:
      return "Invalid";
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
class LLCModel::RdisProcess : public kernel::Process {
  enum class State { Idle, ProcessMessage, ExecuteActions };

  static const char* to_string(State s) {
    switch (s) {
      case State::Idle:
        return "Idle";
      case State::ProcessMessage:
        return "ProcessMessage";
      case State::ExecuteActions:
        return "ExecuteActions";
      default:
        return "Invalid";
    }
  }

 public:
  RdisProcess(kernel::Kernel* k, const std::string& name, LLCModel* model)
      : kernel::Process(k, name), model_(model) {}

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

    MQArb* arb = model_->arb();
    wait_on(arb->request_arrival_event());
  }

  // Elaboration
  void eval() override {
    MQArb* arb = model_->arb();
    MQArbTmt t;
    bool commit = false;

    t = arb->tournament();
    if (t.has_requester()) {
      const Message* msg = t.winner()->peek();
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
        const Message* msg = t.winner()->dequeue();

        LogMessage lm("Execute message: ");
        lm.append(msg->to_string());
        lm.level(Level::Info);
        log(lm);

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
        memcmd->set_origin(model_);
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
  void fini() override {}

  // Current machine state
  State state_ = State::Idle;
  // Pointer to owning LLC instance.
  LLCModel* model_ = nullptr;
};

//
//
class LLCNocEndpoint : public NocEndpoint {
 public:
  LLCNocEndpoint(kernel::Kernel* k, const std::string& name)
      : NocEndpoint(k, name)
  {}
  //
  void register_endpoint(MessageClass cls, MessageQueueProxy* p) {
    endpoints_.insert(std::make_pair(cls, p));
  }
  //
  MessageQueueProxy* lookup_mq(const Message* msg) const override {
    if (auto it = endpoints_.find(msg->cls()); it != endpoints_.end()) {
      return it->second;
    } else {
      LogMessage lm("End point not register for class: ");
      lm.append(cc::to_string(msg->cls()));
      lm.level(Level::Fatal);
      log(lm);
    }
    return nullptr;
  }

 private:
  //
  std::map<MessageClass, MessageQueueProxy*> endpoints_;
};

LLCModel::LLCModel(kernel::Kernel* k, const LLCModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

LLCModel::~LLCModel() {
  delete dir_llc__cmd_q_;
  delete mem_llc__rsp_q_;
  delete arb_;
  delete rdis_proc_;
  delete noc_endpoint_;
  for (MessageQueueProxy* p : endpoints_) { delete p; }
}

void LLCModel::build() {
  // DIR -> LLC command queue
  dir_llc__cmd_q_ = new MessageQueue(k(), "dir_llc__cmd_q", 3);
  add_child_module(dir_llc__cmd_q_);
  // MEM -> LLC response queue
  mem_llc__rsp_q_ = new MessageQueue(k(), "mem_llc__rsp_q", 3);
  add_child_module(mem_llc__rsp_q_);
  // Construct arbiter
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Construct main thread
  rdis_proc_ = new RdisProcess(k(), "main", this);
  add_child_process(rdis_proc_);
  // NOC endpoint
  noc_endpoint_ = new LLCNocEndpoint(k(), "noc_ep");
  add_child_module(noc_endpoint_);
}

void LLCModel::elab() {
  arb_->add_requester(dir_llc__cmd_q_);
  arb_->add_requester(mem_llc__rsp_q_);

  MessageQueueProxy* p = nullptr;

  //
  p = dir_llc__cmd_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::LLCCmd, p);
  endpoints_.push_back(p);
  //
  p = mem_llc__rsp_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::MemRsp, p);
  endpoints_.push_back(p);
}

void LLCModel::drc() {
  if (llc_noc__msg_q_ == nullptr) {
    LogMessage msg("LLC to NOC egress queue has not been bound.");
    msg.level(Level::Fatal);
    log(msg);
  }
}

MessageQueue* LLCModel::endpoint() const {
  return noc_endpoint_->ingress_mq();
}

}  // namespace cc
