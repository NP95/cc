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

#include "cc/llc.h"
#include "primitives.h"
#include "utility.h"

namespace cc {

const char* to_string(LLCEp d) {
  switch (d) {
    default: return "Invalid";
#define __declare_string(__name) case LLCEp::__name: return #__name;
      LLC_MESSAGE_QUEUES(__declare_string)
#undef __declare_string
  }
}

class LLCModel::MainProcess : public kernel::Process {

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
  MainProcess(kernel::Kernel* k, const std::string& name, LLCModel* model)
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
  void init() override {
    set_state(State::AwaitingMessage);
    Arbiter<const Message*>* arb = model_->arb();
    // Await the arrival of a new message at the ingress message
    // queues.
    wait_on(arb->request_arrival_event());
  }

  // Elaboration
  void eval() override {
  }

  // Finalization
  void fini() override {
  }

  // Current execution context
  Context ctxt_;
  // Current machine state
  State state_ = State::AwaitingMessage;
  // Pointer to owning LLC instance.
  LLCModel* model_ = nullptr;
};

LLCModel::LLCModel(kernel::Kernel* k, const LLCModelConfig& config)
    : kernel::Agent<const Message*>(k, config.name), config_(config) {
  build();
}

void LLCModel::build() {
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

void LLCModel::elab() {
  // Register message queue end-points
  add_end_point(ut(LLCEp::CmdQ), cmdq_);
  add_end_point(ut(LLCEp::RspQ), rspq_);
}

void LLCModel::drc() {
}

} // namespace cc
