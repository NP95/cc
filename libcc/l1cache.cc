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

#include "cache.h"
#include "cc/cpu.h"
#include "cc/protocol.h"
#include "cc/sim.h"
#include "cpu_msg.h"
#include "l2cache.h"
#include "primitives.h"

namespace cc {

class L1CacheModel::MainProcess : public kernel::Process {
  using CacheModel = CacheModel<L1LineState*>;

  enum class L1CacheMessageType { CpuRsp, GetS, GetE };

  struct Context {
    // Current arbiter tournament; retained such that the
    Arbiter<const Message*>::Tournament t;

    // Set of messages to be emitted by the cache.
    std::deque<L1CacheMessageType> l1_cache_messages;

    // State containing the result of the protocol message
    // application.
    L1CacheModelApplyResult apply_result;

    // Pointer to the protocol logic object.
    L1CacheModelProtocol* protocol = nullptr;

    // Iterator to the line into which the current operation is taking
    // place.
    CacheModel::LineIterator line_it;
  };

#define STATES(__func)                                                  \
  __func(AwaitingMessage) __func(ProcessMessage) __func(ExecuteActions) \
      __func(EmitMessages) __func(CommitContext) __func(DiscardContext)

  enum class State {
#define __declare_state(__name) __name,
    STATES(__declare_state)
#undef __declare_state
  };

  std::string to_string(State state) {
    switch (state) {
      default:
        return "Unknown";
#define __declare_to_string(__name) \
  case State::__name:               \
    return #__name;                 \
    break;
        STATES(__declare_to_string)
#undef __declare_to_string
    }
    return "Invalid";
  }

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model) {}

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
  void fini() override {}

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
          const L1CacheModelConfig& config = model_->config();
          ctxt_.protocol = config.protocol;
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
        handle_commit_context();
      } break;

      case State::DiscardContext: {
        handle_discard_context();
      } break;

      default: {
        log(LogMessage{"Unknown state entered", Level::Fatal});
      } break;
    }
  }

  void handle_nominated_message() {
    const kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
    const Message* msg = intf->peek();
    switch (msg->cls()) {
      case Message::CpuCmd: {
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
          ctxt_.line_it = set.find(ah.tag(cpucmd->addr()));
          CacheModel::Line& line = ctxt_.line_it.line();

          L1CacheModelApplyResult& apply_result = ctxt_.apply_result;
          const L1CacheModelProtocol* p = ctxt_.protocol;
          p->apply(apply_result, line.t(), cpucmd);

          switch (apply_result.status()) {
            case L1UpdateStatus::IsBlocked: {
              // The protocol disallows the current message from
              // completing (the reason why this has happeneded is
              // unclear within the context of the cache and is known
              // only to the protocol manager itself).

              // Block the interface, so that it cannot be rescheduled
              // in subsequent timesteps.
              kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
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
          const std::pair<CacheModel::LineIterator, bool> line_lookup =
              nominate_line(set.begin(), set.end());

          // Context iterator to point to nominated line.
          ctxt_.line_it = line_lookup.first;
          if (ctxt_.line_it != set.end()) {
            // A nominated line has been found; now consider whether
            // the currently selected line must first be evicted
            // (i.e. written-back) before the fill operation can
            // proceed.
            const bool eviction_required = line_lookup.second;
            if (eviction_required) {
              // TODO
            } else {
              const L1CacheModelProtocol* protocol = ctxt_.protocol;
              // Construct new line; initialized to the invalid state
              // (or the equivalent, as determined by the protocol).
              L1LineState* l1line = protocol->construct_line();
              // Install newly constructed line in the cache set.
              set.install(ctxt_.line_it, ah.tag(cpucmd->addr()), l1line);
              // Apply state update to the line.
              L1CacheModelApplyResult& apply_result = ctxt_.apply_result;
              protocol->apply(apply_result, l1line, cpucmd);
              // Advance to execute state.
              set_state(State::ExecuteActions);
              next_delta();
            }
          } else {
            // Eviction cannot take place therefore the current
            // message is blocked
            kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
            intf->set_blocked(true);
            //
            set_state(State::DiscardContext);
            next_delta();
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

  std::pair<CacheModel::LineIterator, bool> nominate_line(
      CacheModel::LineIterator begin, CacheModel::LineIterator end) {
    CacheModel::LineIterator it;
    // Firstly consider empty lines within the set.
    it = begin;
    while (it != end) {
      const CacheModel::Line& line = it.line();
      if (!line.valid()) return std::make_pair(it, false);
      ++it;
    }
    // Otherwise, consider lines which can be evicted.
    it = begin;
    while (it != end) {
      const CacheModel::Line& line = it.line();
      if (line.t()->is_evictable()) return std::make_pair(it, true);
    }
    // Otherwise, all lines are busy.
    return std::make_pair(end, false);
  }

  void handle_execute_actions() {
    // Incoming actions:
    L1CacheModelApplyResult& apply_result = ctxt_.apply_result;
    // Outgoing message requests.
    std::deque<L1CacheMessageType>& msgs = ctxt_.l1_cache_messages;
    // Dispatch for all actions
    for (L1UpdateAction action : apply_result.actions()) {
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
        case L1UpdateAction::UnblockCmdReqQueue: {
          MessageQueue* mq = model_->msgreqq();
          mq->set_blocked(false);
        } break;
        default: {
        } break;
      }
    }
    set_state(msgs.empty() ? State::CommitContext : State::EmitMessages);
    next_delta();
  }

  void handle_emit_messages() {
    std::deque<L1CacheMessageType>& msgs = ctxt_.l1_cache_messages;
    switch (msgs.front()) {
      case L1CacheMessageType::CpuRsp: {
        const bool did_issue = true;
        if (did_issue) {
          // Form message and emit.
          const kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
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
        if (did_issue) {
          L1L2Message* msg = new L1L2Message(msg->t());
          msg->opcode(L1L2Message::GetS);
          L2CacheModel* l2cache = model_->l2cache();
          kernel::EndPointIntf<const Message*>* l2_ep =
              l2cache->end_point(L2CacheModel::L1CmdReq);
          model_->issue(l2_ep, kernel::Time{10, 0}, msg);
          msgs.pop_front();
        }
      } break;
      case L1CacheMessageType::GetE: {
        const bool did_issue = true;
        if (did_issue) {
          L1L2Message* msg = new L1L2Message(msg->t());
          msg->opcode(L1L2Message::GetE);
          L2CacheModel* l2cache = model_->l2cache();
          kernel::EndPointIntf<const Message*>* l2_ep =
              l2cache->end_point(L2CacheModel::L1CmdReq);
          model_->issue(l2_ep, kernel::Time{10, 0}, msg);
          msgs.pop_front();
        }
      } break;
      default: {
        LogMessage msg("Unknown message type: ", Level::Error);
        log(msg);
      } break;
    }

    if (msgs.empty()) {
      set_state(State::CommitContext);
    }
    next_delta();
  }

  void handle_commit_context() {
    CacheModel::LineIterator line_it = ctxt_.line_it;
    CacheModel::Line& line = line_it.line();
    // Apply protocol update to cache line.
    const L1CacheModelProtocol* protocol = ctxt_.protocol;
    protocol->commit(ctxt_.apply_result, line.t());
    // Remove head-of-line message from nominated Message Queue and
    // return to associated pool.
    kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
    const Message* msg = intf->dequeue();
    msg->release();
    // Advance arbitration state.
    ctxt_.t.advance();
    set_state(State::AwaitingMessage);
    next_delta();
  }

  void handle_discard_context() {
    set_state(State::AwaitingMessage);
    next_delta();
  }

  // Current process state
  State state_;
  // Context of current operation.
  Context ctxt_;
  // Pointer to parent module.
  L1CacheModel* model_ = nullptr;
};

L1CacheModel::L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config)
    : kernel::Agent<const Message*>(k, config.name), config_(config) {
  build();
}

L1CacheModel::~L1CacheModel() {}

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

  cache_ = new CacheModel<L1LineState*>(config_.cconfig);
}

void L1CacheModel::elab() {
  add_end_point(EndPoints::CpuRsp, cpu_);
  add_end_point(EndPoints::L1CmdReq, msgreqq_);
  add_end_point(EndPoints::L1CmdRsp, msgrspq_);
}

void L1CacheModel::drc() {}

}  // namespace cc
