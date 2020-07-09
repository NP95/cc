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

#include "l2cache.h"

#include "cc/sim.h"
#include "l1cache.h"
#include "primitives.h"
#include "protocol.h"
#include "cache.h"
#include "amba.h"
#include "noc.h"
#include "dir.h"
#include "ccntrl.h"
#include "utility.h"

namespace cc {

std::string L1L2__CmdMsg::to_string_short() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
  }
  return ss.str();
}

std::string L1L2__CmdMsg::to_string() const {
  std::stringstream ss;
  {
    using cc::to_string;
    using std::to_string;
    
    KVListRenderer r(ss);
    r.add_field("opcode", to_string(opcode()));
    r.add_field("addr", to_string(addr()));
  }
  return ss.str();
}

class L2CacheModel::MainProcess : public kernel::Process {

  struct Context {
    // Current arbiter tournament; retained such that the
    Arbiter<const Message*>::Tournament t;

    // State containing the result of the protocol message
    // application.
    L2CacheModelApplyResult apply_result;

    // Iterator to the line into which the current operation is taking
    // place.
    CacheModel<L2LineState*>::LineIterator line_it;
  };

  // clang-format off
#define STATES(__func)				\
  __func(AwaitingMessage)			\
  __func(ProcessMessage)			\
  __func(ExecuteActions)			\
  __func(CommitContext)				\
  __func(DiscardContext)
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
  MainProcess(kernel::Kernel* k, const std::string& name, L2CacheModel* model)
      : kernel::Process(k, name), model_(model) {}

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
  
  // Initialization:
  void init() override {
    set_state(State::AwaitingMessage);
    Arbiter<const Message*>* arb = model_->arb();
    wait_on(arb->request_arrival_event());
  }

  // Evaluation:
  void eval() override {
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
        log(LogMessage{"Unknown state entered", Level::Fatal});
      } break;
    }
  }

  // Finalization:
  void fini() override {}

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
      const L2CacheModelConfig& config = model_->config();
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
      case MessageClass::L1L2__CmdMsg: {
        const L1L2__CmdMsg* cmdmsg = static_cast<const L1L2__CmdMsg*>(msg);
        CacheModel<L2LineState*>* cache = model_->cache();
        const CacheAddressHelper ah = cache->ah();
        // Check cache occupancy status:
        CacheModel<L2LineState*>::Set set = cache->set(ah.set(cmdmsg->addr()));
        if (set.hit(ah.tag(cmdmsg->addr()))) {
          // TODO: Hit.
        } else {
          // Cache miss; either service current command by installing
	  CacheModel<L2LineState*>::Evictor evictor;
          const std::pair<CacheModel<L2LineState*>::LineIterator, bool> line_lookup =
              evictor.nominate(set.begin(), set.end());
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
              // Construct new line; initialized to the invalid state
              // (or the equivalent, as determined by the protocol).
              const L2CacheModelProtocol* protocol = model_->protocol();
              L2LineState* l1line = protocol->construct_line();
              // Install newly constructed line in the cache set.
              set.install(ctxt_.line_it, ah.tag(cmdmsg->addr()), l1line);
              // Apply state update to the line.
              L2CacheModelApplyResult& apply_result = ctxt_.apply_result;
              protocol->apply(apply_result, l1line, cmdmsg);
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
    // TODO: to prevent deadlock, the resources for the apply result
    // must be available (and reserved before it can complete.
    
    L2CacheModelApplyResult& ar = ctxt_.apply_result;
    while (!ar.empty()) {
      const L2UpdateAction action = ar.next();
      switch (action) {
        case L2UpdateAction::UpdateState: {
          // Apply line state update.
          const L2CacheModelProtocol* protocol = model_->protocol();
          L2LineState* line = ctxt_.line_it->t();
          protocol->update_line_state(line, ar.state());
          // Action completeed; discard.
          ar.pop();
        } break;

        case L2UpdateAction::Commit: {
          kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
          intf->dequeue();
          ctxt_.t.advance();
          ar.pop();
        } break;

        case L2UpdateAction::Block: {
          kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
          intf->set_blocked(true);
          ar.pop();
        } break;

        case L2UpdateAction::EmitReadNoSnoop:
        case L2UpdateAction::EmitReadOnce:
        case L2UpdateAction::EmitReadClean:
        case L2UpdateAction::EmitReadSharedNotDirty:
        case L2UpdateAction::EmitReadShared:
        case L2UpdateAction::EmitReadUnique:
        case L2UpdateAction::EmitCleanUnique:
        case L2UpdateAction::EmitCleanShared:
        case L2UpdateAction::EmitCleanInvalid:
        case L2UpdateAction::EmitMakeUnique:
        case L2UpdateAction::EmitMakeInvalid:
        case L2UpdateAction::EmitWriteNoSnoop:
        case L2UpdateAction::EmitWriteLineUnique:
        case L2UpdateAction::EmitWriteBack:
        case L2UpdateAction::EmitWriteClean:
        case L2UpdateAction::EmitEvict: {
          const bool can_issue = true;
          if (can_issue) {
            const MsgRequesterIntf* intf = ctxt_.t.intf();
            const Message* msg = intf->peek();
            // Issue message to cache controller.
            AceCmdMsg* acemsg = new AceCmdMsg(msg->t());
            acemsg->opcode(update_to_opcode(action));
            // Issue message into cache controller.
            model_->issue(model_->l2_cc__cmd_q(), kernel::Time{2000, 0}, acemsg);
            // Action completeed; discard.
            ar.pop();
          } else {
            // Message blocks on flow-control to interconnect.

            // FATAL
          }
        } break;
        default: {
        } break;
      }
    }
    set_state(State::AwaitingMessage);
    next_delta();
  }

  // Pointer to parent L2.
  L2CacheModel* model_ = nullptr;

  State state_;

  Context ctxt_;
};

L2CacheModel::L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

L2CacheModel::~L2CacheModel() {}

void L2CacheModel::add_l1c(L1CacheModel* l1c) {
  // Called during build phase.
  
  // Construct associated message queues
  MessageQueue* l1c_cmdq = new MessageQueue(k(), "l1_l2__cmd_q", 3);
  l1_l2__cmd_qs_.push_back(l1c_cmdq);
  add_child_module(l1c_cmdq);
  // Add L1 cache
  l1cs_.push_back(l1c);
}

void L2CacheModel::build() {
  // CC -> L2 response queue.
  cc_l2__rsp_q_ = new MessageQueue(k(), "cc_l2__rsp_q", 16);
  add_child_module(cc_l2__rsp_q_);
  // Arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  add_child_module(arb_);
  // Main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
  // Cache model
  cache_ = new CacheModel<L2LineState*>(config_.cconfig);
  // Setup protocol
  protocol_ = config_.pbuilder->create_l2();
}

void L2CacheModel::elab() {
  // Add command queues to arbiter
  arb_->add_requester(cc_l2__rsp_q_);
  for (MessageQueue* msgq : l1_l2__cmd_qs_) {
    arb_->add_requester(msgq);
  }
}

void L2CacheModel::drc() {
  if (protocol_ == nullptr) {
    // No protocol is defined.
  }
  
  if (l1cs_.empty()) {
    // Typically, a nominal configuration would expect some number of
    // L1 caches to belong to the L2. This is not strictly necessary,
    // as the L2 cache can essentially remain ccompletley inert and
    // idle, but otherwise points to some misconfiguration in the
    // system somoewhere.
    LogMessage msg{"L2 has no child L1 cache(s).", Level::Warning};
    log(msg);
  }
}

}  // namespace cc
