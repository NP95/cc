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
#include "cache.h"
#include "utility.h"

namespace cc {

DirCmdMsg::DirCmdMsg() : Message(MessageClass::DirCmd) {}


class DirectoryModel::NocIngressProcess : public kernel::Process {
 public:
  NocIngressProcess(kernel::Kernel* k, const std::string& name,
                    DirectoryModel* model)
      : kernel::Process(k, name), model_(model) {
  }

  // Initialization
  void init() override {
    MessageQueue* mq = model_->noc_dir__msg_q();
    wait_on(mq->request_arrival_event());
  }

  // Evaluation
  void eval() override {
    using cc::to_string;
    
    // Upon reception of a NOC message, remove transport layer
    // encapsulation and issue to the appropriate ingress queue.
    MessageQueue* noc_mq = model_->noc_dir__msg_q();
    const NocMessage* nocmsg = static_cast<const NocMessage*>(noc_mq->dequeue());

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

 private:
  DirectoryModel* model_ = nullptr;
};

class DirectoryModel::RdisProcess : public kernel::Process {
  using Tournament = MessageQueueArbiter::Tournament;
  using CacheLineIt = CacheModel<DirLineState*>::LineIterator;

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
  RdisProcess(kernel::Kernel* k, const std::string& name, DirectoryModel* model)
      : kernel::Process(k, name), model_(model) {
    state_ = State::Idle;
  }

  State state() const { return state_; }
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

 private:

  // Initialization
  void init() override {
    set_state(State::Idle);
    MessageQueueArbiter* arb = model_->arb();
    // Await the arrival of a new message at the ingress message
    // queues.
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
        LogMessage msg("Unknown state: ");
        msg.append(to_string(state()));
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  // Finalization
  void fini() override {
  }

  void handle_awaiting_message() {
    // Idle state, awaiting more work.
    MessageQueueArbiter* arb = model_->arb();
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

    switch (msg->cls()) {
      case MessageClass::DirCmd: {
        const DirCmdMsg* cmdmsg = static_cast<const DirCmdMsg*>(msg);
        CacheModel<DirLineState*>* cache = model_->cache();
        const CacheAddressHelper ah = cache->ah();
        // Check cache occupancy status:
        CacheModel<DirLineState*>::Set set = cache->set(ah.set(cmdmsg->addr()));
        if (set.hit(ah.tag(cmdmsg->addr()))) {
          // TODO: Hit.
        } else {
          // Cache miss; either service current command by installing
          CacheModel<DirLineState*>::Evictor evictor;
          const std::pair<CacheModel<DirLineState*>::LineIterator, bool> line_lookup =
              evictor.nominate(set.begin(), set.end());

          CacheLineIt it = line_lookup.first;
          if (it != set.end()) {
            // A nominated line has been found; now consider whether
            // the currently selected line must first be evicted
            // (i.e. written-back) before the fill operation can
            // proceed.
            const bool eviction_required = line_lookup.second;
            if (eviction_required) {
              // TODO
            } else {
              // Construct new line; initialized to the invalid state
              // (or the equivalent, as determined by the protocol).
              const DirectoryProtocol* protocol = model_->protocol();
              DirLineState* dirline = protocol->construct_line();
              // Install newly constructed line in the cache set.
              set.install(it, ah.tag(cmdmsg->addr()), dirline);
              // Apply state update to the line.
              context_ = DirCoherenceContext();
              context_.set_line(dirline);
              context_.set_msg(msg);
              bool commits = false;
              std::tie(commits, action_list_) = protocol->apply(context_);
              // Advance to execute state.
              set_state(State::ExecuteActions);
              next_delta();
            }
          } else {
            // TODO:
          }
        }
      } break;
      default: {
      } break;
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

  // Current execution context
  Tournament t_;
  // Current machine state
  State state_;
  //
  DirCoherenceContext context_;
  // Coherence action list.
  DirectoryActionList action_list_;
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
  delete noc_dir__cmd_q_;
  delete arb_;
  delete noci_proc_;
  delete rdis_proc_;
  delete protocol_;
}

void DirectoryModel::build() {
  // NOC -> DIR message queue
  noc_dir__msg_q_ = new MessageQueue(k(), "noc_dir__msg_q", 3);
  add_child_module(noc_dir__msg_q_);
  // NOC -> DIR command queue
  noc_dir__cmd_q_ = new MessageQueue(k(), "noc_dir__cmd_q", 3);
  add_child_module(noc_dir__cmd_q_);
  // Construct arbiter
  arb_ = new MessageQueueArbiter(k(), "arb");
  add_child_module(arb_);
  // Directory state cache
  CacheModelConfig cfg;
  cache_ = new CacheModel<DirLineState*>(cfg);
  // Construct NOC ingress thread.
  noci_proc_ = new NocIngressProcess(k(), "noci", this);
  add_child_process(noci_proc_);
  // Construct request dispatcher thread
  rdis_proc_ = new RdisProcess(k(), "rdis", this);
  add_child_process(rdis_proc_);
  // Setup protocol
  protocol_ = config_.pbuilder->create_dir();
}

void DirectoryModel::elab() {
  // Register message queue end-points
  arb_->add_requester(noc_dir__cmd_q_);
  protocol_->set_dir(this);
}

void DirectoryModel::drc() {
  if (dir_noc__msg_q_ == nullptr) {
    LogMessage lmsg("Directory to NOC message queue is unbound.", Level::Fatal);
    log(lmsg);
  }
}

MessageQueue* DirectoryModel::lookup_rdis_mq(MessageClass cls) const {
  MessageQueue* mq = nullptr;
  switch (cls) {
    case MessageClass::DirCmd: {
      mq = noc_dir__cmd_q_;
    } break;
    default: {
      mq = nullptr;
    } break;
  }
  return mq;
}

} // namespace cc
