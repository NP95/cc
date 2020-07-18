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

#include "amba.h"
#include "dir.h"
#include "msg.h"
#include "noc.h"
#include "primitives.h"
#include "protocol.h"

namespace cc {

//
//
class CC::NocIngressProcess : public kernel::Process {
 public:
  NocIngressProcess(kernel::Kernel* k, const std::string& name, CC* cc)
      : Process(k, name), cc_(cc) {}

  // Initialization
  void init() override {
    MessageQueue* mq = cc_->noc_cc__msg_q();
    wait_on(mq->request_arrival_event());
  }

  // Evaluation
  void eval() override {
    using cc::to_string;

    // Upon reception of a NOC message, remove transport layer
    // encapsulation and issue to the appropriate ingress queue.
    MessageQueue* noc_mq = cc_->noc_cc__msg_q();
    const NocMsg* nocmsg = static_cast<const NocMsg*>(noc_mq->dequeue());

    // Validate message
    if (nocmsg->cls() != MessageClass::Noc) {
      LogMessage lmsg("Received invalid message class: ");
      lmsg.append(to_string(nocmsg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    const Message* msg = nocmsg->payload();
    MessageQueue* iss_mq = cc_->lookup_rdis_mq(msg->cls());
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

 private:
  CC* cc_ = nullptr;
};

//
//
class CC::RdisProcess : public kernel::Process {
  using Tournament = MessageQueueArbiter::Tournament;
  using TableIt = Table<CCLineState*>::Iterator;

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
  RdisProcess(kernel::Kernel* k, const std::string& name, CC* cc)
      : Process(k, name), cc_(cc) {}
  State state() const { return state_; }

 private:
  void set_state(State state) {
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
    set_state(State::Idle);
    wait_on(arb->request_arrival_event());
  }

  // Evaluation
  void eval() override {
    switch (state()) {
      case State::Idle: {
        handle_awaiting_message();
      } break;
      case State::ProcessMessage: {
        handle_process_message();
      } break;
      case State::ExecuteActions: {
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
  void fini() override {}

  void handle_awaiting_message() {
    // Idle state, awaiting more work.
    MessageQueueArbiter* arb = cc_->arb();
    t_ = arb->tournament();

    // Detect deadlock at L1Cache front-end. This occurs only in the
    // presence of a protocol violation and is therefore by definition
    // unrecoverable.
    if (t_.deadlock()) {
      const LogMessage msg{"A protocol deadlock has been detected.",
                           Level::Fatal};
      log(msg);
    }

    // Check for the presence of issue-able messages at the
    // pipeline front-end.
    if (t_.has_requester()) {
      // A message is available; begin processing in the next
      // delta cycle.
      set_state(State::ProcessMessage);
      next_delta();
    } else {
      // Otherwise, block awaiting the arrival of a message at on
      // the of the message queues.
      wait_on(arb->request_arrival_event());
    }
  }

  void handle_process_message() {
    const MsgRequesterIntf* intf = t_.intf();
    const Message* msg = intf->peek();
    bool commits = false;

    // Dispatch on message class:
    switch (msg->cls()) {
      case MessageClass::AceCmd: {
        // L2 -> CC bus tranasaction.
        const AceCmdMsg* acemsg = static_cast<const AceCmdMsg*>(msg);
        Table<CCLineState*>* table = cc_->table();
        const Table<CCLineState*>::Manager th(table->begin(), table->end());

        TableIt it = th.first_invalid();

        // const bool has_invalid_entry = (it != table->end());
        // const bool has_active_entry = false; // TODO
        const bool has_invalid_entry = true;
        const bool has_active_entry = false;
        if (has_invalid_entry && !has_active_entry) {
          // There are no prior transactions to this line in flight
          // and available slots in the transaction table.
          const CCProtocol* protocol = cc_->protocol();
          CCLineState* line = protocol->construct_line();

          // Install line; (move to commit).
          table->install(it, line);
          // TODO!
          it_ = it;

          context_ = CCContext();
          context_.set_line(line);
          context_.set_msg(msg);
          std::tie(commits, action_list_) = protocol->apply(context_);
          set_state(State::ExecuteActions);
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
      case MessageClass::Dt:
      case MessageClass::CohEnd:
      case MessageClass::CohCmdRsp: {
        Table<CCLineState*>* table = cc_->table();
        if (it_ != table->end()) {
          context_ = CCContext();
          CCLineState* line = *it_;
          context_.set_line(line);
          context_.set_msg(msg);
          const CCProtocol* protocol = cc_->protocol();
          std::tie(commits, action_list_) = protocol->apply(context_);
          set_state(State::ExecuteActions);
          next_delta();
        } else {
          LogMessage lm("Cannot find table context for message: ");
          lm.append(msg->to_string());
          lm.level(Level::Fatal);
          log(lm);
        }
      } break;
      default: {
        using cc::to_string;

        LogMessage lmsg("Invalid message class: ");
        lmsg.append(to_string(msg->cls()));
        lmsg.level(Level::Fatal);
        log(lmsg);
      } break;
    }

    if (commits) {
      // Current message commits and is applied to the machine state.
      LogMessage lm("Execute message: ");
      lm.append(msg->to_string());
      lm.level(Level::Info);
      log(lm);

      set_state(State::ExecuteActions);
      next_delta();
    }
  }

  void handle_execute_actions() {
    while (!action_list_.empty()) {
      CoherenceAction* action = action_list_.back();
      if (!action->execute()) break;

      action->release();
      action_list_.pop_back();
    }
    if (action_list_.empty()) {
      using Interface = MessageQueueArbiter::Interface;

      // Complete: discard message and advance arbitration state.
      Interface* intf = t_.intf();
      const Message* msg = intf->dequeue();
      msg->release();
      t_.advance();

      // Return to idle state.
      set_state(State::Idle);
      next_delta();
    }
  }

  // TODO: save table entry for reuse.
  TableIt it_;

  // Current arbitration tournament.
  Tournament t_;
  // Current processing context.
  CCContext context_;
  // Current processing state
  State state_;
  // Coherence action list.
  CCActionList action_list_;
  // Cache controller instance.
  CC* cc_ = nullptr;
};

CC::CC(kernel::Kernel* k, const CCConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

CC::~CC() {
  delete l2_cc__cmd_q_;
  delete noc_cc__msg_q_;
  delete dir_cc__rsp_q_;
  delete cc__dt_q_;
  delete arb_;
  delete rdis_proc_;
  delete noci_proc_;
  delete table_;
  delete protocol_;
}

void CC::build() {
  // Construct L2 to CC command queue
  l2_cc__cmd_q_ = new MessageQueue(k(), "l2_cc__cmd_q", 3);
  add_child_module(l2_cc__cmd_q_);
  // NOC -> CC msg queue.
  noc_cc__msg_q_ = new MessageQueue(k(), "noc_cc__msg_q", 3);
  add_child_module(noc_cc__msg_q_);
  // DIR -> CC response queue
  dir_cc__rsp_q_ = new MessageQueue(k(), "dir_cc__rsp_q", 3);
  add_child_module(dir_cc__rsp_q_);
  //
  cc__dt_q_ = new MessageQueue(k(), "cc__dt_q", 3);
  add_child_module(cc__dt_q_);
  // Arbiteer
  arb_ = new MessageQueueArbiter(k(), "arb");
  add_child_module(arb_);
  // Dispatcher process
  rdis_proc_ = new RdisProcess(k(), "rdis_proc", this);
  add_child_process(rdis_proc_);
  //
  noci_proc_ = new NocIngressProcess(k(), "noci_proc", this);
  add_child_process(noci_proc_);
  // Transaction table
  table_ = new Table<CCLineState*>(k(), "ttable", 16);
  add_child_module(table_);
  // Create protocol instance
  protocol_ = config_.pbuilder->create_cc();
}

//
//
void CC::elab() {
  // Add ingress queues to arbitrator.
  arb_->add_requester(l2_cc__cmd_q_);
  arb_->add_requester(dir_cc__rsp_q_);
  arb_->add_requester(cc__dt_q_);
  protocol_->set_cc(this);
}

//
//
void CC::drc() {
  if (dm() == nullptr) {
    // The Dir Mapper object computes the host directory for a
    // given address. In a single directory system, this is a basic
    // mapping to a single directory instance, but in more performant
    // systems this may some non-trivial mapping to multiple home
    // directories.
    LogMessage msg("Directory mapper is not defined.", Level::Warning);
    log(msg);
  }
}

//
//
MessageQueue* CC::lookup_rdis_mq(MessageClass cls) const {
  switch (cls) {
    case MessageClass::Dt:
      return cc__dt_q_;
    case MessageClass::L2Cmd:
      return l2_cc__cmd_q_;
    case MessageClass::CohEnd:
      return dir_cc__rsp_q_;
    case MessageClass::CohCmdRsp:
      return dir_cc__rsp_q_;
    default:
      return nullptr;
  }
}

}  // namespace cc
