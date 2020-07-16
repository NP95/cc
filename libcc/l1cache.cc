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
#include "msg.h"
#include "l2cache.h"
#include "primitives.h"
#include "utility.h"

// #define VERBOSE_LOGGING

namespace cc {

const char* to_string(L1CacheOpcode opcode) {
  switch (opcode) {
    case L1CacheOpcode::CpuLoad: return "CpuLoad";
    case L1CacheOpcode::CpuStore: return "CpuStore";
    default: return "Invalid";
  }
}

L1CmdMsg::L1CmdMsg() : Message(MessageClass::L1Cmd) {}


std::string L1CmdMsg::to_string() const {
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
L1CmdRspMsg::L1CmdRspMsg() : Message(MessageClass::L1CmdRsp) {}

//
//
std::string L1CmdRspMsg::to_string() const {
  using cc::to_string;
  
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    r.add_field("cls", to_string(cls()));
  }
  return ss.str();
}


class L1CacheModel::MainProcess : public kernel::Process {
  using Tournament = MessageQueueArbiter::Tournament;
  using CacheLineIt = CacheModel<L1LineState*>::LineIterator;

  enum class State {
    Idle, ProcessMessage, ExecuteActions
  };

  static const char* to_string(State state) {
    switch (state) {
      case State::Idle: return "Idle";
      case State::ProcessMessage: return "ProcessMessage";
      case State::ExecuteActions: return "ExecuteActions";
      default: return "Invalid";
    }
  }

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model) {}

  // Current process state
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
    // Await the arrival of requesters
    MessageQueueArbiter* arb = model_->arb();
    wait_on(arb->request_arrival_event());
    set_state(State::Idle);
  }

  // Finalization
  void fini() override {}

  // Evaluation
  void eval() override {
    switch (state()) {
      case State::Idle: {
        handle_await_message();
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

  void handle_await_message() {
    // Idle state, awaiting more work.
    MessageQueueArbiter* arb = model_->arb();
    t_  = arb->tournament();

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
      case MessageClass::L1Cmd: {
        CacheModel<L1LineState*>* cache = model_->cache();
        const CacheAddressHelper ah = cache->ah();
        const L1CmdMsg* cpucmd = static_cast<const L1CmdMsg*>(msg);
        // Check cache occupancy status:
        CacheModel<L1LineState*>::Set set = cache->set(ah.set(cpucmd->addr()));
        if (set.hit(ah.tag(cpucmd->addr()))) {
          // Although the address is presently in the cache, we
          // do not know at present what we can do with it as this
          // is some unknown function of the cache line state, the
          // command opcode and the protocol.
          CacheLineIt it = set.find(ah.tag(cpucmd->addr()));
          context_ = L1CoherenceContext();
          context_.set_line(it.line().t());
          context_.set_msg(msg);
          const L1CacheModelProtocol* p = model_->protocol();
          std::tie(commits, action_list_) = p->apply(context_);
          // TODO: assume commits = true
          set_state(State::ExecuteActions);
          next_delta();
        } else {
          // Cache miss; either service current command by installing
	  CacheModel<L1LineState*>::Evictor evictor;
          const std::pair<CacheModel<L1LineState*>::LineIterator, bool> line_lookup =
              evictor.nominate(set.begin(), set.end());

          // Context iterator to point to nominated line.
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
              const L1CacheModelProtocol* protocol = model_->protocol();
              // Construct new line; initialized to the invalid state
              // (or the equivalent, as determined by the protocol).
              L1LineState* l1line = protocol->construct_line();
              // Install newly constructed line in the cache set.
              set.install(it, ah.tag(cpucmd->addr()), l1line);
              // Apply state update to the line.
              context_ = L1CoherenceContext();
              context_.set_line(l1line);
              context_.set_msg(msg);
              std::tie(commits, action_list_) = protocol->apply(context_);
              // Advance to execute state.
              set_state(State::ExecuteActions);
              next_delta();
            }
          } else {
            // Eviction cannot take place therefore the current
            // message is blocked
            //MsgRequesterIntf* intf = ctxt_.t.intf();
            // intf->set_blocked(true);
            //
            set_state(State::Idle);
            next_delta();
          }
        }
      } break;
      case MessageClass::L2CmdRsp: {
        CacheModel<L1LineState*>* cache = model_->cache();
        const CacheAddressHelper ah = cache->ah();
        const L2CmdRspMsg* l2rsp = static_cast<const L2CmdRspMsg*>(msg);
        // Check cache occupancy status:
        // add_t addr = l2rsp->addr();
        // TODO: need to recover state information from a table.
        addr_t addr = 0;
        CacheModel<L1LineState*>::Set set = cache->set(ah.set(addr));
        if (set.hit(ah.tag(addr))) {
          CacheLineIt it = set.find(ah.tag(addr));
          context_ = L1CoherenceContext();
          context_.set_line(it.line().t());
          context_.set_msg(msg);
          const L1CacheModelProtocol* p = model_->protocol();
          std::tie(commits, action_list_) = p->apply(context_);
          // TODO: assume commits = true
          set_state(State::ExecuteActions);
          next_delta();
        } else {
          LogMessage lmsg("L2 response message received, but address is not "
                          "present in the cache.", Level::Fatal);
          log(lmsg);
        }
      } break;
      default: {
        LogMessage lmsg("Invalid message received: ");
        lmsg.append(msg->to_string());
        lmsg.level(Level::Error);
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

  // Current process state
  State state_;
  // Current arbitration tournament.
  Tournament t_;
  // Coherence context for current operation
  L1CoherenceContext context_;
  //
  L1CoherenceActionList action_list_;
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
  arb_ = new MessageQueueArbiter(k(), "arb");
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

  protocol_->set_l1cache(this);
}

void L1CacheModel::drc() {
  if (l1_l2__cmd_q_ == nullptr) {
    LogMessage msg("L2 message queue has not been bound.");
    msg.level(Level::Fatal);
    log(msg);
  }
  if (l1_cpu__rsp_q_ == nullptr) {
    LogMessage msg("L1 to CPU response message queue.");
    msg.level(Level::Fatal);
    log(msg);
  }
  if (cpu_ == nullptr) {
    // Error: CPU has not been bound.
  }
  if (l2c_ == nullptr) {
    // Error: L2C has not been bound.
  }
}

}  // namespace cc
