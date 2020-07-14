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
#include "utility.h"

namespace cc {

//
//
LLCCmdMsg::LLCCmdMsg() : Message(MessageClass::LLCCmd) {}

//
//
LLCRspMsg::LLCRspMsg() : Message(MessageClass::LLCRsp) {}

//
//
class LLCModel::MainProcess : public kernel::Process {

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
  MainProcess(kernel::Kernel* k, const std::string& name, LLCModel* model)
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

    Arbiter<const Message*>* arb = model_->arb();
    wait_on(arb->request_arrival_event());
  }

  // Elaboration
  void eval() override {

    // TODO: simply forward to memory

    // Message to LLC
    MemCmdMsg* memcmd = new MemCmdMsg;
    memcmd->set_opcode(MemCmdOpcode::Read);
    memcmd->set_dest(model_);
    
    NocMsg* nocmsg = new NocMsg;
    nocmsg->set_origin(model_);
    nocmsg->set_dest(model_->mc());
    nocmsg->set_payload(memcmd);
    model_->issue(model_->llc_noc__msg_q(), kernel::Time{10, 0}, nocmsg);
    
    Arbiter<const Message*>* arb = model_->arb();
    wait_on(arb->request_arrival_event());
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
  delete arb_;
  delete main_;
}

void LLCModel::build() {
  // NOC -> LLC message queue:
  noc_llc__msg_q_ = new MessageQueue(k(), "noc_llc__msg_q", 3);
  add_child_module(noc_llc__msg_q_);
  // Construct arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  add_child_module(arb_);
  // Construct main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void LLCModel::elab() {
  arb_->add_requester(noc_llc__msg_q_);
}

void LLCModel::drc() {
  if (llc_noc__msg_q_ == nullptr) {
    LogMessage msg("LLC to NOC egress queue has not been bound.");
    msg.level(Level::Fatal);
    log(msg);
  }
}

} // namespace cc
