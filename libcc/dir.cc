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

#include "dir.h"
#include "dir_gen.h"
#include "noc.h"
#include "llc.h"
#include "llc_gen.h"
#include "amba.h"
#include "primitives.h"
#include "utility.h"

namespace cc {

class DirectoryModel::MainProcess : public kernel::Process {

  struct Context {
    // Current arbiter tournament; retained such that the
    Arbiter<const Message*>::Tournament t;
  };

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, DirectoryModel* model)
      : kernel::Process(k, name), model_(model) {
  }

  DirState state() const { return state_; }
  void set_state(DirState state) {
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
    set_state(DirState::AwaitingMessage);
    Arbiter<const Message*>* arb = model_->arb();
    // Await the arrival of a new message at the ingress message
    // queues.
    wait_on(arb->request_arrival_event());
  }

  // Evaluation
  void eval() {
    switch (state()) {
      case DirState::AwaitingMessage: {
        handle_awaiting_message();
      } break;
      case DirState::ProcessMessage: {
        handle_process_message();
      } break;
      case DirState::ExecuteActions: {
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
      set_state(DirState::ProcessMessage);
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
      case MessageClass::Noc:
      case MessageClass::AceCmd: {

        LLCCmdMessage* cmdmsg = new LLCCmdMessage;
        cmdmsg->set_opcode(LLCCmdOpcode::Fill);
        cmdmsg->set_home(model_);

        NocMessage* nocmsg = new NocMessage;
        nocmsg->set_origin(model_);
        nocmsg->set_dest(model_->llc());
        nocmsg->set_payload(cmdmsg);
        model_->issue(model_->dir_noc__msg_q(), kernel::Time{10, 0}, nocmsg);

        msg->release();
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
  DirState state_ = DirState::AwaitingMessage;
  // Pointer to parent directory instance.
  DirectoryModel* model_ = nullptr;
};

DirectoryModel::DirectoryModel(
    kernel::Kernel* k, const DirectoryModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

DirectoryModel::~DirectoryModel() {
  delete noc_dir__msg_q_;
}

void DirectoryModel::build() {
  // NOC -> DIR message queue
  noc_dir__msg_q_ = new MessageQueue(k(), "noc_dir__msg_q", 3);
  add_child_module(noc_dir__msg_q_);
  // Construct arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  add_child_module(arb_);
  // Construct main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void DirectoryModel::elab() {
  // Register message queue end-points
  arb_->add_requester(noc_dir__msg_q_);
}

void DirectoryModel::drc() {
}

} // namespace cc
