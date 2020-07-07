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

#include "cc/dir.h"
#include "amba.h"
#include "primitives.h"

namespace cc {

class DirectoryModel::MainProcess : public kernel::Process {

  struct Context {
    // Current arbiter tournament; retained such that the
    Arbiter<const Message*>::Tournament t;
  };

  // clang-format off
#define STATES(__func)				\
  __func(AwaitingMessage)			\
  __func(ProcessMessage)			\
  __func(ExecuteActions)
  // clang-format on

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
  MainProcess(kernel::Kernel* k, const std::string& name, DirectoryModel* model)
      : kernel::Process(k, name), model_(model) {
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
  void init() {
    set_state(State::AwaitingMessage);
    Arbiter<const Message*>* arb = model_->arb();
    // Await the arrival of a new message at the ingress message
    // queues.
    wait_on(arb->request_arrival_event());
  }

  // Evaluation
  void eval() {
    switch (state()) {
      case State::AwaitingMessage: {
        handle_awaiting_message();
      } break;
      case State::ProcessMessage: {
        handle_process_message();
      } break;
      case State::ExecuteActions: {
        handle_execute_actions();
      } break;
      default: {
        LogMessage msg("Unknown state: ");
        msg.append(to_string(state()));
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  // Finalization
  void fini() {
  }

  void handle_awaiting_message() {
    // Idle state, awaiting more work.
    Arbiter<const Message*>* arb = model_->arb();
    Arbiter<const Message*>::Tournament t = arb->tournament();

    // Detect deadlock at L1Cache front-end. This occurs only in the
    // presence of a protocol violation and is therefore by definition
    // unrecoverable.
    if (t.deadlock()) {
      const LogMessage msg{"A protocol deadlock has been detected.",
            Level::Fatal};
      log(msg);
    }

    // Check for the presence of issue-able messages at the
    // pipeline front-end.
    if (t.has_requester()) {
      // A message is available; begin processing in the next
      // delta cycle.
      ctxt_ = Context();
      ctxt_.t = t;
      set_state(State::ProcessMessage);
      next_delta();
    } else {
      // Otherwise, block awaiting the arrival of a message at on
      // the of the message queues.
      wait_on(arb->request_arrival_event());
    }
  }

  void handle_process_message() {
    const MsgRequesterIntf* intf = ctxt_.t.intf();
    const Message* msg = intf->peek();
    switch (msg->cls()) {
      case Message::Ace: {
        log(LogMessage("Got message"));
      } break;
      default: {
      } break;
    }
  }

  void handle_execute_actions() {
  }

  // Current execution context
  Context ctxt_;
  // Current machine state
  State state_ = State::AwaitingMessage;
  // Pointer to main process.
  MainProcess* main_ = nullptr;
  // Pointer to parent directory instance.
  DirectoryModel* model_ = nullptr;
};

DirectoryModel::DirectoryModel(
    kernel::Kernel* k, const DirectoryModelConfig& config)
    : kernel::Agent<const Message*>(k, config.name), config_(config) {
  build();
}

void DirectoryModel::build() {
  // Construct command queue
  cmdq_ = new MessageQueue(k(), "cmdq", config_.cmd_queue_n);
  add_child_module(cmdq_);

  // Construct response queue
  rspq_ = new MessageQueue(k(), "rspq", config_.rsp_queue_n);
  add_child_module(rspq_);

  // Construct arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  arb_->add_requester(cmdq_);
  arb_->add_requester(rspq_);
  add_child_module(arb_);

  // Construct main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void DirectoryModel::elab() {
  add_end_point(EndPoint::CmdQ, cmdq_);
  add_end_point(EndPoint::RspQ, rspq_);
}

void DirectoryModel::drc() {
}

} // namespace cc
