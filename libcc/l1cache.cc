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
#include "l2cache.h"
#include "msg.h"
#include "primitives.h"
#include "protocol.h"
#include "utility.h"

// #define VERBOSE_LOGGING

namespace cc {

const char* to_string(L1CacheOpcode opcode) {
  switch (opcode) {
    case L1CacheOpcode::CpuLoad:
      return "CpuLoad";
    case L1CacheOpcode::CpuStore:
      return "CpuStore";
    default:
      return "Invalid";
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

const char* to_string(L1Wait w) {
  switch (w) {
    case L1Wait::Invalid:
      return "Invalid";
    case L1Wait::MsgArrival:
      return "MsgArrival";
    case L1Wait::NextEpochIfHasRequestOrWait:
      return "NextEpochIfHasRequestOrWait";
    default:
      return "Unknown";
  }
}

L1CacheContext::~L1CacheContext() {
  // Destroy all actions.
  for (CoherenceAction* a : al_) {
    a->release();
  }
  if (owns_line()) {
    line_->release();
  }
}

//
//
class L1CacheModel::MainProcess : public kernel::Process {
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model) {}

 private:
  // Initialization
  void init() override {
    L1CacheContext c;
    c.set_wait(L1Wait::MsgArrival);
    wait_on_context(c);
  }

  // Evaluation
  void eval() override {
    MessageQueueArbiter* arb = model_->arb();
    L1Tournament t = arb->tournament();

    // Construct and initialize current processing context.
    L1CacheContext c;
    c.set_l1cache(model_);
    c.set_t(t);

    // Check if requests are present, if not block until a new message
    // arrives at the arbiter. Process should ideally not wake in the
    // absence of requesters.
    if (!t.has_requester()) {
      c.set_wait(L1Wait::MsgArrival);
      wait_on_context(c);
      return;
    }

    // Fetch nominated message.
    const Message* msg = t.intf()->peek();
    c.set_msg(msg);

    // Dispatch to appropriate handler based upon message class.
    switch (c.msg()->cls()) {
      case MessageClass::L1Cmd: {
        // L1 command originating from either the CPU or the parent
        // L2 cache instance.
        eval_msg(c, static_cast<const L1CmdMsg*>(c.msg()));
      } break;
      case MessageClass::L2CmdRsp: {
        eval_msg(c, static_cast<const L2CmdRspMsg*>(c.msg()));
      } break;
      default: {
        using cc::to_string;
        LogMessage lmsg("Invalid message class received: ");
        lmsg.append(to_string(c.msg()->cls()));
        lmsg.level(Level::Error);
        log(lmsg);
      } break;
    }
    // If command commits, execute compute action list.
    if (c.commits()) {
      execute(c);
    }
    // Apply context's wait condition to processes wait-state.
    wait_on_context(c);
  }

 private:
  void eval_msg(L1CacheContext& c, const L1CmdMsg* cmd) const {
    L1Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const L1CacheModelProtocol* protocol = model_->protocol();

    L1CacheSet set = cache->set(ah.set(cmd->addr()));
    L1CacheLineIt it = set.find(ah.tag(cmd->addr()));
    if (it == set.end()) {
      // Line for current address has not been found in the set,
      // therefore a new line must either be installed or, if the set
      // is currently full, another line must be nominated and
      // evicted.
      L1Cache::Evictor evictor;
      if (const std::pair<L1CacheLineIt, bool> p =
              evictor.nominate(set.begin(), set.end());
          p.second) {
        // Eviction required before command can complete.
        c.set_it(p.first);
        c.set_dequeue(false);
        c.set_stalled(true);
        protocol->evict(c);
        check_resources(c);
      } else {
        // Eviction not required for the command to complete.
        c.set_it(p.first);
        c.set_dequeue(true);
        protocol->install(c);
        check_resources(c);
        if (c.commits()) {
          set.install(c.it(), ah.tag(cmd->addr()), c.line());
          // Line ownership passed to cache.
          c.set_owns_line(false);
        }
      }
    } else {
      // Line is present in the cache, apply state update.
      c.set_line(it->t());
      c.set_dequeue(true);
      protocol->apply(c);
      check_resources(c);
    }
  }

  void eval_msg(L1CacheContext& c, const L2CmdRspMsg* rsp) const {
    L1TTable* tt = model_->tt();

    L1TTable::iterator it = tt->find(rsp->t());
    if (it == tt->end()) {
      LogMessage msg("Cannot find transaction table entry for ");
      msg.append(to_string(rsp->t()));
      msg.level(Level::Fatal);
      log(msg);
    }

    L1TState* st = it->second;
    c.set_line(st->line());
    const L1CacheModelProtocol* protocol = model_->protocol();
    protocol->apply(c);
  }

  void check_resources(L1CacheContext& c) const {
    if (!c.commits()) return;

    // Check the set of L1 resources used by this context to determine
    // if it can commit to the machine state in one single atomic
    // action. If not, commit cannot take place and the message is
    // blocked until sufficient resources become available.

    // TODO
  }

  void execute(const L1CacheContext& c) const {
    // Current action (command issue to L2 or consequent eviction)
    // commits, therefore install entry in the transaction table.
    for (CoherenceAction* a : c.actions()) {
      a->execute();
    }
    update_ttable(c);
    L1Tournament t = c.t();
    if (c.stalled()) {
      t.intf()->set_blocked(true);
    } else if (c.dequeue()) {
      t.intf()->dequeue();
      t.advance();
    }
  }

  void update_ttable(const L1CacheContext& c) const {
    L1TTable* tt = model_->tt();
    Transaction* trn = c.msg()->t();
    if (c.ttadd()) {
      // Create new state.
      L1TState* st = new L1TState;
      st->set_line(c.line());
      // Install context into table.
      tt->install(trn, st);
    } else if (c.ttdel()) {
      // Delete entry from table.
      L1TTable::iterator it = tt->find(trn);
      if (it == tt->end()) {
        LogMessage msg("Transaction not present in transaction table!");
        msg.level(Level::Fatal);
        log(msg);
      }
      L1TState* st = it->second;
      st->release();
      tt->erase(it);
    }
  }

  void wait_on_context(const L1CacheContext& c) {
    switch (c.wait()) {
      case L1Wait::MsgArrival: {
        MessageQueueArbiter* arb = model_->arb();
        wait_on(arb->request_arrival_event());
      } break;
      case L1Wait::NextEpochIfHasRequestOrWait: {
        MessageQueueArbiter* arb = model_->arb();
        L1Tournament t = arb->tournament();
        if (t.has_requester()) {
          // Wait some delay
          wait_for(kernel::Time{10, 0});
        } else {
          // No further commands, block process until something
          // arrives.
          wait_on(arb->request_arrival_event());
        }
      } break;
      default: {
        LogMessage msg("Invalid wait condition: ");
        msg.append(to_string(c.wait()));
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  // Pointer to parent module.
  L1CacheModel* model_ = nullptr;
};

L1CacheModel::L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

L1CacheModel::~L1CacheModel() {
  delete cpu_l1__cmd_q_;
  delete l2_l1__rsp_q_;
  delete arb_;
  delete tt_;
  delete main_;
  delete cache_;
}

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
  // Transaction table.
  tt_ = new L1TTable(k(), "tt", 16);
  add_child_module(tt_);
  // Main thread of execution
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
  cache_ = new L1Cache(config_.cconfig);
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
