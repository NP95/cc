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
#include "ccntrl_gen.h"
#include "primitives.h"
#include "msg.h"
#include "protocol.h"
#include "amba.h"
#include "noc.h"
#include "dir.h"

namespace cc {

class CacheController::MainProcess : public kernel::Process {

  struct Context {
    // Current arbiter tournament; retained such that the
    Arbiter<const Message*>::Tournament t;
    //
    CCModelApplyResult ar;

    Table<CacheControllerLineState*>::Iterator it;
  };

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, CacheController* cc)
      : Process(k, name), cc_(cc) {
  }
  CCMainState state() const { return state_; }

 private:

  void set_state(CCMainState state) {
#ifdef VERBOSE_LOGGING
    if (state_ != state) {
      LogMessage msg("State transition: ");
      msg.level(Level::Debug);
      msg.append(to_string(state_));
      msg.append(" -> ");
      msg.append(to_string(state));
      log(msg);
    }
#endif
    state_ = state;
  }

  // Initialization
  void init() override {
    Arbiter<const Message*>* arb = cc_->arb();
    set_state(CCMainState::AwaitingMessage);
    wait_on(arb->request_arrival_event());
  }    

  // Evaluation
  void eval() override {
    switch (state()) {
      case CCMainState::AwaitingMessage: {
        handle_awaiting_message();
      } break;

      case CCMainState::ProcessMessage: {
        handle_process_message();
      } break;
        
      case CCMainState::ExecuteActions: {
        handle_execute_actions();
      } break;

      default: {
        // Unknown state
        const LogMessage lmsg("Transition into invalid state.", Level::Fatal);
        log(lmsg);
      } break;
    }
  }

  // Finalization
  void fini() override {
  }

  void handle_awaiting_message() {
    // Idle state, awaiting more work.
    Arbiter<const Message*>* arb = cc_->arb();
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
      context_ = Context();
      context_.t = t;
      set_state(CCMainState::ProcessMessage);
      next_delta();
    } else {
      // Otherwise, block awaiting the arrival of a message at on
      // the of the message queues.
      wait_on(arb->request_arrival_event());
    }
  }

  void handle_process_message() {
    const kernel::RequesterIntf<const Message*>* intf = context_.t.intf();
    const Message* msg = intf->peek();
    switch (msg->cls()) {
      case MessageClass::AceCmd: {
        // L2 -> CC bus tranasaction.
        const AceCmdMsg* acemsg = static_cast<const AceCmdMsg*>(msg);
        Table<CacheControllerLineState*>* table = cc_->table();
        const Table<CacheControllerLineState*>::Manager th(
            table->begin(), table->end());
        context_.it = th.first_invalid();

        //const bool has_invalid_entry = (it != table->end());
        // const bool has_active_entry = false; // TODO
        const bool has_invalid_entry = true;
        const bool has_active_entry = false;
        if (has_invalid_entry && !has_active_entry) {
          // There are no prior transactions to this line in flight
          // and available slots in the transaction table.
          const CacheControllerProtocol* protocol = cc_->protocol();
          CacheControllerLineState* line = protocol->construct_line();

          // Install line; (move to commit).
          table->install(context_.it, line);
          protocol->apply(context_.ar, line, acemsg);
          set_state(CCMainState::ExecuteActions);
          next_delta();
        } else {
          // Transaction cannot proceed because the table is either
          // full, or there is another entry for this line which
          // is currently active.
          if (!has_invalid_entry) {
            // LOG
          } else if (has_active_entry) {
            // LOG
          }
        }
      } break;
      default: {
        // Unknown message
      } break;
    }
  }

  void handle_execute_actions() {
    const MsgRequesterIntf* intf = context_.t.intf();
    const Message* msg = intf->peek();

    CCModelApplyResult& ar = context_.ar;
    while (!ar.empty()) {
      const CCUpdateAction action = ar.next();
      switch (action) {
        case CCUpdateAction::UpdateState: {
          // Apply line state update.
          const CacheControllerProtocol* protocol = cc_->protocol();
          CacheControllerLineState* line = *context_.it;
          protocol->update_line_state(line, ar.state());
          ar.pop();
        } break;
        case CCUpdateAction::Commit: {
          ar.pop();
        } break;
        case CCUpdateAction::Block: {
          ar.pop();
        } break;
        case CCUpdateAction::EmitAceCmdToDir: {
          const AceCmdMsg* acemsg = static_cast<const AceCmdMsg*>(msg);
          MessageQueue* mq = cc_->cc_noc__msg_q();

          // Encapsulate AceCmd in the NOC transport message.
          NocMessage* nocmsg = new NocMessage;
          nocmsg->set_origin(cc_);
          const DirectoryMapper* dm = cc_->dm();
          nocmsg->set_dest(dm->lookup(acemsg->addr()));
          nocmsg->set_payload(msg);
          cc_->issue(mq, kernel::Time{10, 0}, nocmsg);
          ar.pop();
        } break;
        default: {
          // Invalid action
        } break;
      }
    }
  }
  
  // Current processing context.
  Context context_;
  // Current processing state
  CCMainState state_;
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
  // Transaction table
  table_ = new Table<CacheControllerLineState*>(k(), "ttable", 16);
  add_child_module(table_);
  // Create protocol instance
  protocol_ = config_.pbuilder->create_cc();
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
