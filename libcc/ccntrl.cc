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

#include "ccntrl.h"
#include "primitives.h"

namespace cc {

class CacheController::MainProcess : public kernel::Process {

#define STATES(__func)                          \
  __func(AwaitingMessage)                       \
  __func(ProcessMessage)

  enum class State {
#define __declare_state(__name) __name,
    STATES(__declare_state)
#undef __declare_state
  };
  std::string to_string(State state) {
    switch (state) {
      default:
        return "Unknown";
#define __declare_to_string(__name)             \
        case State::__name:                     \
          return #__name;                       \
          break;
        STATES(__declare_to_string)
#undef __declare_to_string
    }
    return "Invalid";
  }
#undef STATES
 
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, CacheController* cc)
      : Process(k, name), cc_(cc) {
  }
  State state() const { return state_; }

 private:

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

  // Initialization
  void init() override {
    Arbiter<const Message*>* arb = cc_->arb();
    set_state(State::AwaitingMessage);
    wait_on(arb->request_arrival_event());
  }    

  // Evaluation
  void eval() override {
    switch (state()) {
      case State::AwaitingMessage: {
        handle_awaiting_message();
      } break;
      case State::ProcessMessage: {
        handle_process_message();
      } break;
      default: {
        // Unknown state
      } break;
    }
  }

  // Finalization
  void fini() override {
  }

  void handle_awaiting_message() {
    set_state(State::ProcessMessage);
    next_delta();
  }

  void handle_process_message() {
    const MsgRequesterIntf* intf = ctxt_.t.intf();
    const Message* msg = intf->peek();
    switch (msg->cls()) {
      case MessageClass::L2CC_AceCmd: {
        // L2 -> CC bus tranasaction.
      } break;
      default: {
        // Unknown message class.
      } break;
    }
  }

  // Current processing state
  State state_;
  // Cache controller instance.
  CacheController* cc_ = nullptr;
};

CacheController::CacheController(
    kernel::Kernel* k, const CacheControllerCfg& config)
    : Agent(k, config.name), config_(config) {
  build();
}

void CacheController::build() {
  // Construct L2 to CC command queue
  l2_cc__cmd_q_ = new MessageQueue(k(), "l2_cc__cmd_q", 3);
  add_child_module(l2_cc__cmd_q_);
  // NOC -> CC msg queue.
  noc_cc__msg_q_ = new MessageQueue(k(), "noc_cc__msg_q_", 3);
  add_child_module(noc_cc__msg_q_);
  // Arbiteer
  arb_ = new Arbiter<const Message*>(k(), "arb");
  add_child_module(arb_);
  // Main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void CacheController::elab() {
  // Add ingress queues to arbitrator.
  arb_->add_requester(l2_cc__cmd_q_);
  arb_->add_requester(noc_cc__msg_q_);
}

void CacheController::drc() {

  if (dm() == nullptr) {
    // The Directory Mapper object computes the host directory for a
    // given address. In a single directory system, this is a basic
    // mapping to a single directory instance, but in more performant
    // systems this may some non-trivial mapping to multiple home
    // directories.
    LogMessage msg("Directory mapper is not defined.", Level::Warning);
    log(msg);
  }
}


} // namespace cc
