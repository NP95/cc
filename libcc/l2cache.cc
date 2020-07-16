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

#include "sim.h"
#include "l1cache.h"
#include "l2cache.h"
#include "primitives.h"
#include "protocol.h"
#include "cache.h"
#include "amba.h"
#include "noc.h"
#include "dir.h"
#include "ccntrl.h"
#include "utility.h"

namespace cc {

const char* to_string(L2CmdOpcode opcode) {
  switch (opcode) {
    case L2CmdOpcode::L1GetS: return "L1GetS";
    case L2CmdOpcode::L1GetE: return "L1GetE";
    default: return "Invalid";
  }
}

L2CmdMsg::L2CmdMsg() : Message(MessageClass::L2Cmd) {}

std::string L2CmdMsg::to_string() const {
  using cc::to_string;

  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
    r.add_field("opcode", to_string(opcode()));
    Hexer h;
    r.add_field("addr", h.to_hex(addr()));
  }
  return ss.str();
}

//
//
L2CmdRspMsg::L2CmdRspMsg() : Message(MessageClass::L2CmdRsp) {}

//
//
const char* to_string(L2RspOpcode opcode) {
  switch (opcode) {
    case L2RspOpcode::L1InstallS: return "L1InstallS";
    case L2RspOpcode::L1InstallE: return "L1InstallE";
    default: return "Invalid";
  }
}

//
//
std::string L2CmdRspMsg::to_string() const {
  using cc::to_string;

  std::stringstream ss;
  {
    KVListRenderer r(ss);
    r.add_field("cls", to_string(cls()));
    r.add_field("opcode", to_string(opcode()));
  }
  return ss.str();
}


class L2CacheModel::MainProcess : public kernel::Process {
  using Tournament = MessageQueueArbiter::Tournament;
  using CacheLineIt = CacheModel<L2LineState*>::LineIterator;

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
    set_state(State::Idle);
    Arbiter<const Message*>* arb = model_->arb();
    wait_on(arb->request_arrival_event());
  }

  // Evaluation:
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
        log(LogMessage{"Unknown state entered", Level::Fatal});
      } break;
    }
  }

  // Finalization:
  void fini() override {}

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
    bool commits = false;
    switch (msg->cls()) {
      case MessageClass::L2Cmd: {
        const L2CmdMsg* cmdmsg = static_cast<const L2CmdMsg*>(msg);
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
              const L2CacheModelProtocol* protocol = model_->protocol();
              L2LineState* l2line = protocol->construct_line();
              // Install newly constructed line in the cache set.
              set.install(it, ah.tag(cmdmsg->addr()), l2line);
              it_ = it;
              // Apply state update to the line.
              context_ = L2CoherenceContext();
              context_.set_line(l2line);
              context_.set_msg(msg);
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
      case MessageClass::AceCmdRspB:
      case MessageClass::AceCmdRspR: {
        if (true) {
          context_ = L2CoherenceContext();
          context_.set_line(it_->t());
          context_.set_msg(msg);
          const L2CacheModelProtocol* protocol = model_->protocol();
          std::tie(commits, action_list_) = protocol->apply(context_);
          // Advance to execute state.
          set_state(State::ExecuteActions);
          next_delta();
        } else {
          // Cache line not installed for current line.
        }
      } break;
      default: {
        using cc::to_string;

        LogMessage lmsg("Invalid message class received: ");
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

  CacheLineIt it_;

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

  // Pointer to parent L2.
  L2CacheModel* model_ = nullptr;

  State state_;
  // Current arbitration tournament.
  Tournament t_;
  // Coherence context for current operation
  L2CoherenceContext context_;
  // Coherence action list.
  L2CoherenceActionList action_list_;
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
  arb_ = new MessageQueueArbiter(k(), "arb");
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
  protocol_->set_l2cache(this);
}

void L2CacheModel::set_l1cache_n(std::size_t n) {
  l2_l1__rsp_qs_.resize(n);
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
