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

#include "l1cache.h"
#include "primitives.h"
#include "cpu_msg.h"
#include "cache.h"
#include "cc/cpu.h"
#include "cc/sim.h"
#include "cc/protocol.h"

namespace cc {

class L1CacheModel::MainProcess : public kernel::Process {

  enum class State {
    AwaitingMessage, ProcessMessage, IssueMessage, ComputeDelay, IncurFlush
  };

  enum class L1CacheMessageType {
    CpuRsp, GetS, GetE
  };

  class ProtocolResult {
   public:
    ProtocolResult() {}

    bool commit() const { return commit_; }
    void commit(bool commit) { commit_ = commit; }

    std::vector<L1UpdateAction>& actions() { return actions_; }
    const std::vector<L1UpdateAction>& actions() const { return actions_; }

   private:
    std::size_t penalty_cycles_n_;
    
    // Flag indicating that the message has committed.
    bool commit_ = false;

    // Set of actions to be applyed sbusequent to the committment of the
    // current message.
    std::vector<L1UpdateAction> actions_;
  };
  
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model)
  {}

  // Cache protocol
  L1CacheModelProtocol* protocol() const { return model_->config_.protocol; }
  // Current process state
  State state() const { return state_; }
  void set_state(State state) { state_ = state; }

 private:
  // Initialization
  void init() override {
    // Await the arrival of requesters
    Arbiter<const Message*>* arb = model_->arb();
    wait_on(arb->request_arrival_event());
    set_state(State::AwaitingMessage);
  }

  // Finalization
  void fini() override {
  }

    // Evaluation
  void eval() override {
    Arbiter<const Message*>* arb = model_->arb();
    switch (state()) {
      case State::AwaitingMessage: {
        // Idle state, awaiting more work.
        Arbiter<const Message*>::Tournament t = arb->tournament();

        // Check for the presence of issue-able messages at the
        // pipeline front-end.
        if (t.has_requester()) {
          // A message is available; begin processing in the next
          // delta cycle.
          set_state(State::ProcessMessage);
          next_delta();
        } else {
          // Otherwise, block awaiting the arrival of a message at on
          // the of the message queues.
          wait_on(arb->request_arrival_event());
        }
      } break;
      case State::ProcessMessage: {
        Arbiter<const Message*>::Tournament t = arb->tournament();

        if (!t.has_requester()) {
          // Expect work at this point, if not we have entered this
          // state erroneously.
          const LogMessage msg(
              "Expected work, but no messages found", Level::Fatal);
          log(msg);
        }

        // Detect deadlock at L1Cache front-end. This occurs only in the
        // presence of a protocol violation and is therefore by definition
        // unrecoverable.
        if (t.deadlock()) {
          const LogMessage msg{
            "A protocol deadlock has been detected.", Level::Fatal};
          log(msg);
        }

        set_state(handle_nominated(t.intf()));
        next_delta();
      } break;
      case State::IssueMessage: {
        const L1CacheMessageType msg_type = msgs_.front();
        // Try to issue head-of-line message
        if (issue_message(msg_type)) {
          // Message was successfully emitted:
          msgs_.pop_front();
          if (!msgs_.empty()) {
            // Message can now be release back to the owning pool.
            msg_->release();

            // Return to Idle/AwaitingMessage state after the message
            // issue cost has been incurred.
            set_state(State::AwaitingMessage);

            // message_cost(msg_type)
            wait_for(cc::kernel::Time{10, 0});
          } else {
            // Wait for some delay
            wait_for(cc::kernel::Time{10, 0});
          }
        } else {
          // Could not emit message, try again in the next cycle.
          wait_for(cc::kernel::Time{10, 0});
        }
      } break;
      case State::ComputeDelay: {
        set_state(State::AwaitingMessage);
        // Wait for some delay
        wait_for(cc::kernel::Time{10, 0});
      } break;
      case State::IncurFlush: {
        // Penalty state which blocks the execution of the cache for
        // some number of cycles.
        set_state(State::AwaitingMessage);
        // Wait for some delay
        wait_for(cc::kernel::Time{50, 0});
      } break;
      default: {
        const LogMessage msg("Unknown state entered", Level::Fatal);
        log(msg);
      } break;
    }
  }

  bool issue_message(L1CacheMessageType type) {
    bool did_issue = false;
    switch (type) {
      case L1CacheMessageType::CpuRsp: {
        did_issue = true;
        if (did_issue) {
          // Form message and emit.
          CpuResponseMessage* rsp = new CpuResponseMessage(msg_->t());
          kernel::EndPointIntf<const Message*>* cpu_end_point = model_->cpu();
          model_->issue(cpu_end_point, kernel::Time{10, 0}, rsp);
        }
      } break;
      case L1CacheMessageType::GetS: {
        did_issue = true;
      } break;
      case L1CacheMessageType::GetE: {
        did_issue = true;
      } break;
      default: {
      } break;
    }
    return did_issue;
  }

  std::size_t message_cost(L1CacheMessageType type) {
    return 10;
  }

  State handle_nominated(kernel::RequesterIntf<const Message*>* intf) {
    State ret;
    msg_ = intf->peek();
    switch (msg_->cls()) {
      case Message::CpuCmd: {
        using CacheModel = CacheModel<L1LineState*>;
        
        CacheModel* cache = model_->cache();
        L1CacheModelProtocol* p = protocol();;
        CacheAddressHelper ah = cache->ah();
        const CpuCommandMessage* cpucmd =
            static_cast<const CpuCommandMessage*>(msg_);

        // Check cache occupancy status:
        CacheModel::Set set = cache->set(ah.set(cpucmd->addr()));
        if (set.hit(ah.tag(cpucmd->addr()))) {
          // Although the address is presently in the cache, we
          // do not know at present what we can do with it as this
          // is some unknown function of the cache line state, the
          // command opcode and the protocol.
          CacheModel::LineIterator it = set.find(ah.tag(cpucmd->addr()));
          CacheModel::Line& line = it.line();

          const L1CacheModelApplyResult result = p->apply(line.t(), cpucmd);

          switch (result.status()) {
            case L1UpdateStatus::IsBlocked: {
              // The protocol disallows the current message from
              // completing (the reason why this has happeneded is
              // unclear within the context of the cache and is known
              // only to the protocol manager itself).

              // Block the interface, so that it cannot be rescheduled
              // in subsequent timesteps.
              intf->set_blocked(true);

              // Incur the penatly associated with the failed attempt;
              // approximately equivalent to the penalty associated
              // with flushing the cache pipeline.
              ret = State::IncurFlush;
            } break;
            case L1UpdateStatus::CanCommit: {
              // Process actions
              const bool has_messages = handle_actions(msg_, result.actions());
              // Apply result to line state.
              p->commit(result, line.t());
              // Remove message from owning Queue instance.
              intf->dequeue();
              // Consume message; return to pool or destruct.
              if (!has_messages) { msg_->release(); }
              // Compute next state.
              ret = has_messages ? State::IssueMessage : State::ComputeDelay;
            } break;
          }
        } else {
          // Cache miss; either service current command by installing

        }
      } break;
      default: {
        LogMessage lmsg("Invalid message received: ");
        lmsg.append(msg_->to_string());
        lmsg.level(Level::Error);
        log(lmsg);
      } break;
    }
    return ret;
  }

  bool handle_actions(
      const Message* msg, const std::vector<L1UpdateAction>& actions) {
    bool had_message = false;
    for (L1UpdateAction action : actions) {
      switch (action) {
        case L1UpdateAction::EmitCpuRsp: {
          // Emit response to requesting CPU
          msgs_.push_back(L1CacheMessageType::CpuRsp);
          had_message = true;
        } break;
        case L1UpdateAction::EmitGetS: {
          // Emit GetS (Get Shared) to owning L2 cache.
          msgs_.push_back(L1CacheMessageType::GetS);
          had_message = true;
        } break;
        case L1UpdateAction::EmitGetE: {
          // Emit GetE (Get Exclusive) to owning L2 cache.
          msgs_.push_back(L1CacheMessageType::GetE);
          had_message = true;
        } break;
        case L1UpdateAction::UnblockCmdQueue: {
        } break;
        default: {
        } break;
      }
    }
    return had_message;
  }

  // Pointer to parent module.
  L1CacheModel* model_ = nullptr;
  // Current process state
  State state_;
  // Set of outstanding messages to be issued by the cache.
  std::deque<L1CacheMessageType> msgs_;
  // Current message being processed
  const Message* msg_;
};

L1CacheModel::L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config)
    : kernel::Agent<const Message*>(k, config.name), config_(config) {
  build();
}

L1CacheModel::~L1CacheModel() {
}

void L1CacheModel::build() {
  // Capture stimulus;
  cpu_ = new Cpu(k(), "cpu");
  cpu_->set_stimulus(config_.stim);
  add_child_module(cpu_);

  // Construct queues: Command Request Queue
  msgreqq_ = new MessageQueue(k(), "cmdreqq", 16);
  add_child_module(msgreqq_);

  // Construct queues: Command Response Queue
  msgrspq_ = new MessageQueue(k(), "cmdrspq", 16);
  add_child_module(msgrspq_);

  // Arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  arb_->add_requester(cpu_);
  arb_->add_requester(msgreqq_);
  arb_->add_requester(msgrspq_);
  add_child_module(arb_);

  // Main thread of execution
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void L1CacheModel::elab() {
  add_end_point(EndPoints::CpuRsp, cpu_);
  add_end_point(EndPoints::L1CmdReq, msgreqq_);
  add_end_point(EndPoints::L1CmdRsp, msgrspq_);
}

void L1CacheModel::drc() {
}

} // namespace cc
