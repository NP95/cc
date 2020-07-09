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
#include "cpu.h"
#include "protocol.h"
#include "cc/sim.h"
#include "msg.h"
#include "l2cache.h"
#include "primitives.h"
#include "utility.h"

// #define VERBOSE_LOGGING

namespace cc {

std::string CpuL1__CmdMsg::to_string_short() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
  }
  return ss.str();
}

std::string CpuL1__CmdMsg::to_string() const {
  std::stringstream ss;
  {
    using cc::to_string;
    using std::to_string;
    
    KVListRenderer r(ss);
    r.add_field("opcode", to_string(opcode()));
    const Hexer hexer;
    r.add_field("addr", hexer.to_hex(addr()));
  }
  return ss.str();
}

class L1CacheModel::MainProcess : public kernel::Process {

  enum class L1CacheMessageType { CpuRsp, GetS, GetE };

  struct Context {
    // Current arbiter tournament; retained such that the
    Arbiter<const Message*>::Tournament t;

    // State containing the result of the protocol message
    // application.
    L1CacheModelApplyResult apply_result;

    // Iterator to the line into which the current operation is taking
    // place.
    CacheModel<L1LineState*>::LineIterator line_it;
  };

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model) {}

  // Current process state
  L1MainState state() const { return state_; }
  void set_state(L1MainState state) {
#ifdef VERBOSE_LOGGING
    if (state_ != state) {
      LogMessage msg("L1MainState transition: ");
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
    // Await the arrival of requesters
    Arbiter<const Message*>* arb = model_->arb();
    wait_on(arb->request_arrival_event());
    set_state(L1MainState::AwaitingMessage);
  }

  // Finalization
  void fini() override {}

  // Evaluation
  void eval() override {
    switch (state()) {
      case L1MainState::AwaitingMessage: {
	    handle_awaiting_message();
      } break;

      case L1MainState::ProcessMessage: {
        handle_nominated_message();
      } break;

      case L1MainState::ExecuteActions: {
        handle_execute_actions();
      } break;

      default: {
        log(LogMessage{"Unknown state entered", Level::Fatal});
      } break;
    }
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
      const L1CacheModelConfig& config = model_->config();
      set_state(L1MainState::ProcessMessage);
      next_delta();
    } else {
      // Otherwise, block awaiting the arrival of a message at on
      // the of the message queues.
      wait_on(arb->request_arrival_event());
    }
  }

  void handle_nominated_message() {
    const MsgRequesterIntf* intf = ctxt_.t.intf();
    const Message* msg = intf->peek();
    switch (msg->cls()) {
      case MessageClass::CpuL1__CmdMsg: {
        CacheModel<L1LineState*>* cache = model_->cache();
        const CacheAddressHelper ah = cache->ah();
        const CpuL1__CmdMsg* cpucmd = static_cast<const CpuL1__CmdMsg*>(msg);
        // Check cache occupancy status:
        CacheModel<L1LineState*>::Set set = cache->set(ah.set(cpucmd->addr()));
        if (set.hit(ah.tag(cpucmd->addr()))) {
          // Although the address is presently in the cache, we
          // do not know at present what we can do with it as this
          // is some unknown function of the cache line state, the
          // command opcode and the protocol.
          ctxt_.line_it = set.find(ah.tag(cpucmd->addr()));
          CacheModel<L1LineState*>::Line& line = ctxt_.line_it.line();

          L1CacheModelApplyResult& apply_result = ctxt_.apply_result;
          const L1CacheModelProtocol* p = model_->protocol();
          p->apply(apply_result, line.t(), cpucmd);
          set_state(L1MainState::ExecuteActions);
          next_delta();
        } else {
          // Cache miss; either service current command by installing
	  CacheModel<L1LineState*>::Evictor evictor;
          const std::pair<CacheModel<L1LineState*>::LineIterator, bool> line_lookup =
              evictor.nominate(set.begin(), set.end());

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
              const L1CacheModelProtocol* protocol = model_->protocol();
              // Construct new line; initialized to the invalid state
              // (or the equivalent, as determined by the protocol).
              L1LineState* l1line = protocol->construct_line();
              // Install newly constructed line in the cache set.
              set.install(ctxt_.line_it, ah.tag(cpucmd->addr()), l1line);
              // Apply state update to the line.
              L1CacheModelApplyResult& apply_result = ctxt_.apply_result;
              protocol->apply(apply_result, l1line, cpucmd);
              // Advance to execute state.
              set_state(L1MainState::ExecuteActions);
              next_delta();
            }
          } else {
            // Eviction cannot take place therefore the current
            // message is blocked
            MsgRequesterIntf* intf = ctxt_.t.intf();
            intf->set_blocked(true);
            //
            set_state(L1MainState::AwaitingMessage);
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

  void handle_execute_actions() {
    kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
    const Message* msg = intf->peek();

    LogMessage lmsg("Execute message: ");
    lmsg.append(msg->to_string());
    lmsg.level(Level::Info);
    log(lmsg);

    L1CacheModelApplyResult& ar = ctxt_.apply_result;
    while (!ar.empty()) {
      const L1UpdateAction action = ar.next();
      switch (action) {
        case L1UpdateAction::UpdateState: {
          // Apply line state update.
          const L1CacheModelProtocol* protocol = model_->protocol();
          L1LineState* line = ctxt_.line_it->t();
          protocol->update_line_state(line, ar.state());
          // Action completeed; discard.
          ar.pop();
        } break;
        case L1UpdateAction::Commit: {
          intf->dequeue();
          ctxt_.t.advance();
          ar.pop();
        } break;
        case L1UpdateAction::Block: {
          kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
          intf->set_blocked(true);
          ar.pop();
        } break;
        case L1UpdateAction::EmitCpuRsp: {
          ar.pop();
        } break;
        case L1UpdateAction::EmitGetS: {
          L1L2__CmdMsg* msg = new L1L2__CmdMsg(nullptr);
          msg->set_opcode(L2L1Opcode::GetS);
          msg->set_addr(0);
          model_->issue(model_->l1_l2__cmd_q(), kernel::Time{1000, 0}, msg);
          ar.pop();
        } break;
        default: {
          // Error, unknown action
        } break;
      }
    }
    set_state(L1MainState::AwaitingMessage);
    next_delta();
  }

  // Current process state
  L1MainState state_;
  // Context of current operation.
  Context ctxt_;
  // Pointer to parent module.
  L1CacheModel* model_ = nullptr;
};

L1CacheModel::L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

L1CacheModel::~L1CacheModel() {}

void L1CacheModel::build() {
  // Construct command request queue
  cpu_l1__cmd_q_ = new MessageQueue(k(), "cpu_l1__cmd_q", 16);
  add_child_module(cpu_l1__cmd_q_);

  // Construct L2 -> L1 response queue
  l2_l1__rsp_q_ = new MessageQueue(k(), "l2_l1__rsp_q", 16);
  add_child_module(l2_l1__rsp_q_);

  // Arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  add_child_module(arb_);

  // Main thread of execution
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);

  cache_ = new CacheModel<L1LineState*>(config_.cconfig);

  // Set up protocol
  protocol_ = config_.pbuilder->create_l1();
}

void L1CacheModel::elab() {
  arb_->add_requester(cpu_l1__cmd_q_);
  arb_->add_requester(l2_l1__rsp_q_);
}

void L1CacheModel::drc() {
  if (cpu_ == nullptr) {
    // Error: CPU has not been bound.
  }

  if (l2c_ == nullptr) {
    // Error: L2C has not been bound.
  }
}

}  // namespace cc
