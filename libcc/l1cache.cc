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

  enum class L1CacheMessageType {
    CpuRsp, GetS, GetE
  };

  // Cleanup
  struct Context {
    Context(Arbiter<const Message*>::Tournament t) : t(t)
    {}

    //
    const Message* msg() const { return intf->peek(); }

    void consume() {
      CacheModel<L1LineState*>::Line& line = line_it.line();
      // Apply protocol update to cache line.
      protocol->commit(apply_result, line.t());
      // Remove head-of-line message from nominated Message Queue
      const Message* msg = intf->dequeue();
      // Release message back to pool
      msg->release();
      // Advance arbitration state.
      t.advance();
    }

    void discard() {
      // NOP: state is not owned by context.
    }

    // Originating interface from which the current message was
    // sourced.
    kernel::RequesterIntf<const Message*>* intf = nullptr;

    // Current arbiter tournament; retained such that the
    Arbiter<const Message*>::Tournament t;

    // Set of actions to be carried out on the state of the
    // cache in response to the application of the message to
    // the protocol.
    std::vector<L1UpdateAction> l1_update_actions;

    // Set of messages to be emitted by the cache.
    std::deque<L1CacheMessageType> l1_cache_messages;

    // State containing the result of the protocol message
    // application.
    L1CacheModelApplyResult apply_result;

    // Pointer to the protocol logic object.
    L1CacheModelProtocol* protocol = nullptr;

    // Iterator to the line into which the current operation is taking
    // place.
    CacheModel<L1LineState*>::LineIterator line_it;
  };

#define STATES(__func)                          \
  __func(AwaitingMessage)                       \
  __func(ProcessMessage)                        \
  __func(ExecuteActions)                        \
  __func(EmitMessages)                          \
  __func(CommitContext)                         \
  __func(DiscardContext)
  
  enum class State {
#define __declare_state(__name) __name,
    STATES(__declare_state)
#undef __declare_state
  };

  std::string to_string(State state) {
    switch (state) {
      default: return "Unknown";
#define __declare_to_string(__name) case State::__name: return #__name; break;
      STATES(__declare_to_string)
#undef __declare_to_string
    }
    return "Invalid";
  }
  
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model)
  {}

  // Cache protocol
  L1CacheModelProtocol* protocol() const { return model_->config_.protocol; }
  // Current process state
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
    switch (state()) {
      case State::AwaitingMessage: {
        // Idle state, awaiting more work.
        Arbiter<const Message*>* arb = model_->arb();
        Arbiter<const Message*>::Tournament t = arb->tournament();

        // Detect deadlock at L1Cache front-end. This occurs only in the
        // presence of a protocol violation and is therefore by definition
        // unrecoverable.
        if (t.deadlock()) {
          const LogMessage msg{
            "A protocol deadlock has been detected.", Level::Fatal};
          log(msg);
        }

        // Check for the presence of issue-able messages at the
        // pipeline front-end.
        if (t.has_requester()) {
          // A message is available; begin processing in the next
          // delta cycle.
          ctxt_ = new Context(t);
          ctxt_->protocol = protocol();
          set_state(State::ProcessMessage);
          next_delta();
        } else {
          // Otherwise, block awaiting the arrival of a message at on
          // the of the message queues.
          wait_on(arb->request_arrival_event());
        }
      } break;
        
      case State::ProcessMessage: {
        handle_nominated_message();
      } break;

      case State::ExecuteActions: {
        handle_execute_actions();
      } break;

      case State::EmitMessages: {
        handle_emit_messages();
      } break;

      case State::CommitContext: {
        ctxt_->consume();
        delete ctxt_;
        set_state(State::AwaitingMessage);
        next_delta();
      } break;

      case State::DiscardContext: {
        ctxt_->discard();
        delete ctxt_;
        set_state(State::AwaitingMessage);
        next_delta();
      } break;
      default: {
        log(LogMessage{"Unknown state entered", Level::Fatal});
      } break;
    }
  }

  void handle_nominated_message() {
    const kernel::RequesterIntf<const Message*>* intf = ctxt_->intf;
    const Message* msg = intf->peek();
    switch (msg->cls()) {
      case Message::CpuCmd: {
        using CacheModel = CacheModel<L1LineState*>;
        
        CacheModel* cache = model_->cache();
        CacheAddressHelper ah = cache->ah();
        const CpuCommandMessage* cpucmd =
            static_cast<const CpuCommandMessage*>(msg);

        // Check cache occupancy status:
        CacheModel::Set set = cache->set(ah.set(cpucmd->addr()));
        if (set.hit(ah.tag(cpucmd->addr()))) {
          // Although the address is presently in the cache, we
          // do not know at present what we can do with it as this
          // is some unknown function of the cache line state, the
          // command opcode and the protocol.
          ctxt_->line_it = set.find(ah.tag(cpucmd->addr()));
          CacheModel::Line& line = ctxt_->line_it.line();


          L1CacheModelApplyResult& apply_result = ctxt_->apply_result;
          const L1CacheModelProtocol* p = ctxt_->protocol;
          p->apply(apply_result, line.t(), cpucmd);

          switch (apply_result.status()) {
            case L1UpdateStatus::IsBlocked: {
              // The protocol disallows the current message from
              // completing (the reason why this has happeneded is
              // unclear within the context of the cache and is known
              // only to the protocol manager itself).

              // Block the interface, so that it cannot be rescheduled
              // in subsequent timesteps.
              kernel::RequesterIntf<const Message*>* intf = ctxt_->intf;
              intf->set_blocked(true);

              // Incur the penatly associated with the failed attempt;
              // approximately equivalent to the penalty associated
              // with flushing the cache pipeline.
              set_state(State::DiscardContext);
              next_delta();
            } break;
            case L1UpdateStatus::CanCommit: {
              set_state(State::ExecuteActions);
              next_delta();
            } break;
          }
        } else {
          // Cache miss; either service current command by installing
          CacheModel::LineIterator it = set.begin(), nominated_line = set.end();
          while (it != set.end()) {
            const L1LineState* line = it.line().t();
            if (line->is_evictable()) {
              nominated_line = it;
              break;
            }
            ++it;
          }
          if (nominated_line != set.end()) {
            // Eviction can take place to the current line;
            // TODO
          } else {
            // Eviction cannot take place therefore the current
            // message is blocked
            kernel::RequesterIntf<const Message*>* intf = ctxt_->intf;
            intf->set_blocked(true);
          }
        }
      } break;
      default: {
        LogMessage lmsg("Invalid message received: ");
        lmsg.append(msg->to_string());
        lmsg.level(Level::Error);
        log(lmsg);
      } break;
    }
  }

  void handle_execute_actions() {
    std::deque<L1CacheMessageType>& msgs = ctxt_->l1_cache_messages;
    for (L1UpdateAction action : ctxt_->l1_update_actions) {
      switch (action) {
        case L1UpdateAction::EmitCpuRsp: {
          // Emit response to requesting CPU
          msgs.push_back(L1CacheMessageType::CpuRsp);
        } break;
        case L1UpdateAction::EmitGetS: {
          // Emit GetS (Get Shared) to owning L2 cache.
          msgs.push_back(L1CacheMessageType::GetS);
        } break;
        case L1UpdateAction::EmitGetE: {
          // Emit GetE (Get Exclusive) to owning L2 cache.
          msgs.push_back(L1CacheMessageType::GetE);
        } break;
        case L1UpdateAction::UnblockCmdQueue: {
          
        } break;
        default: {
        } break;
      }
    }
    set_state(msgs.empty() ? State::CommitContext : State::EmitMessages);
    next_delta();
  }

  void handle_emit_messages() {
    std::deque<L1CacheMessageType>& msgs = ctxt_->l1_cache_messages;
    switch (msgs.front()) {
      case L1CacheMessageType::CpuRsp: {
        const bool did_issue = true;
        if (did_issue) {
          // Form message and emit.
          const kernel::RequesterIntf<const Message*>* intf = ctxt_->intf;
          const Message* msg = intf->peek();
          // Construct message:
          CpuResponseMessage* rsp = new CpuResponseMessage(msg->t());
          // Issue to CPU:
          kernel::EndPointIntf<const Message*>* cpu_end_point = model_->cpu();
          model_->issue(cpu_end_point, kernel::Time{10, 0}, rsp);
          // As now issue, discard current message request.
          msgs.pop_front();
        }
      } break;
      case L1CacheMessageType::GetS: {
        const bool did_issue = true;
      } break;
      case L1CacheMessageType::GetE: {
        const bool did_issue = true;
      } break;
      default: {
        LogMessage msg("Unknown message type: ", Level::Error);
        log(msg);
      } break;
    }

    if (msgs.empty()) { set_state(State::CommitContext); }
    next_delta();
  }

  // Pointer to parent module.
  L1CacheModel* model_ = nullptr;
  // Current process state
  State state_;
  // Set of outstanding messages to be issued by the cache.
  std::deque<L1CacheMessageType> msgs_;
  // Context of current operation.
  Context* ctxt_ = nullptr;
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
