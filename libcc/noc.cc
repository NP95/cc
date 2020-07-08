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

#include "cc/noc.h"
#include "primitives.h"

namespace cc {

class NocModel::MainProcess : public kernel::Process {

  // clang-format off
#define STATES(__func)				\
  __func(AwaitingMessage)			\
  __func(ChooseQueue)                           \
  __func(IssueMessage)
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

  struct Context {
    Arbiter<const Message*>::Tournament t;
  };
  
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, NocModel* model)
      : kernel::Process(k, name), model_(model)
  {}

  State state() const { return state_; }
  void set_state(State state) { state_ = state; }

 private:

  void init() override {
    // Await the arrival of requesters
    Arbiter<const Message*>* arb = model_->arb();
    set_state(State::AwaitingMessage);
    wait_on(arb->request_arrival_event());
  }

  void eval() override {
    switch (state()) {
      case State::AwaitingMessage: {
        handle_awaiting_message();
      } break;
      case State::ChooseQueue: {
        handle_choose_queue();
      } break;
      case State::IssueMessage: {
        handle_issue_message();
      } break;
      default: {
      } break;
    }
  }

  void fini() override {
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

    if (t.has_requester()) {
      ctxt_ = Context();
      set_state(State::ChooseQueue);
      next_delta();
    } else {
      // Otherwise, block awaiting the arrival of a message at on
      // the of the message queues.
      wait_on(arb->request_arrival_event());
    }
  }

  void handle_choose_queue() {
    // Idle state, awaiting more work.
    Arbiter<const Message*>* arb = model_->arb();

    ctxt_.t = arb->tournament();
    if (ctxt_.t.has_requester()) {
      set_state(State::IssueMessage);
      next_delta();
    }
  }

  void handle_issue_message() {
    //kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
    //issue_message(intf->dequeue());
    set_state(State::AwaitingMessage);
    wait_for(kernel::Time{10, 0});
  }

  void issue_message(const Message* msg) {
    const NocMessage* nocmsg = static_cast<const NocMessage*>(msg);
    if (nocmsg->ep() == nullptr) {
      LogMessage lmsg("Invalid end-point in transport header.", Level::Fatal);
      log(lmsg);
    }
    //model_->issue(nocmsg->ep(), kernel::Time{1000, 0}, nocmsg->payload());
  }

  // Current context
  Context ctxt_;
  // Current state
  State state_;
  // Pointer to parent NocModel instance.
  NocModel* model_ = nullptr;
};

NocModel::NocModel(kernel::Kernel* k, const NocModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

kernel::EndPointIntf<const Message*>* NocModel::get_input(std::size_t n) {
  return imqs_[n];
}

void NocModel::build() {
  // Construct input message queues.
  for (std::size_t i = 0; i < config_.ingress_ports_n; i++) {
    const std::string mqname = "mq" + std::to_string(imqs_.size());
    MessageQueue* mq = new MessageQueue(k(), mqname, config_.ingress_q_n);
    add_child_module(mq);
    imqs_.push_back(mq);
  }

  // Construct ingress selection aribter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  for (MessageQueue* mq : imqs_) {
    arb_->add_requester(mq);
  }
  add_child_module(arb_);

  // Construct main process
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void NocModel::elab() {
}

void NocModel::drc() {
  if (imqs_.empty()) {
    LogMessage lmsg("No ingress interfaces are defined!.", Level::Fatal);
    log(lmsg);
  }
}

} // namespace cc
