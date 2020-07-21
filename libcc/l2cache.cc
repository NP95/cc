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

#include "utility.h"

namespace cc {

const char* to_string(L2CmdOpcode opcode) {
  switch (opcode) {
    case L2CmdOpcode::L1GetS:
      return "L1GetS";
    case L2CmdOpcode::L1GetE:
      return "L1GetE";
    default:
      return "Invalid";
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
    case L2RspOpcode::L1InstallS:
      return "L1InstallS";
    case L2RspOpcode::L1InstallE:
      return "L1InstallE";
    default:
      return "Invalid";
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

const char* to_string(L2Wait w) {
  switch (w) {
    case L2Wait::Invalid:
      return "Invalid";
    case L2Wait::MsgArrival:
      return "MsgArrival";
    case L2Wait::NextEpochIfHasRequestOrWait:
      return "NextEpochIfHasRequestOrWait";
    default:
      return "Unknown";
  }
}

L2CacheContext::~L2CacheContext() {
  for (CoherenceAction* a : al_) {
    a->release();
  }
}

//
//
class L2CacheModel::MainProcess : public kernel::Process {
  using CacheLineIt = CacheModel<L2LineState*>::LineIterator;

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L2CacheModel* model)
      : kernel::Process(k, name), model_(model) {}

 private:
  // Initialization:
  void init() override {
    L2CacheContext c;
    c.set_wait(L2Wait::MsgArrival);
    wait_on_context(c);
  }

  // Evaluation:
  void eval() override {
    MQArb* arb = model_->arb();
    MQArbTmt t = arb->tournament();

    // Construct and initialize current processing context.
    L2CacheContext c;
    c.set_l2cache(model_);
    c.set_t(t);

    // Fetch nominated message.
    const Message* msg = t.winner()->peek();
    c.set_mq(t.winner());
    c.set_msg(msg);

    // Dispatch to appropriate message class
    switch (c.msg()->cls()) {
      case MessageClass::L2Cmd: {
        eval_msg(c, static_cast<const L2CmdMsg*>(c.msg()));
      } break;
      case MessageClass::AceCmdRspR: {
        L2TState* st = lookup_tt(c.msg()->t());
        c.set_st(st);
        c.set_line(st->line());
        eval_msg(c, static_cast<const AceCmdRspRMsg*>(c.msg()));
      } break;
      default: {
      } break;
    }
    // If context commits, apply set of state updates.
    if (c.commits()) {
      execute(c);
    }
    // Apply context's wait condition to processes wait-state.
    wait_on_context(c);
  }

  void eval_msg(L2CacheContext& c, const L2CmdMsg* cmd) const {
    L2Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const L2CacheModelProtocol* protocol = model_->protocol();

    L2CacheSet set = cache->set(ah.set(cmd->addr()));
    L2CacheLineIt it = set.find(ah.tag(cmd->addr()));
    if (it == set.end()) {
      // Line for current address has not been found in the set,
      // therefore a new line must either be installed or, if the set
      // is currently full, another line must be nominated and
      // evicted.
      L2Cache::Evictor evictor;
      if (const std::pair<L2CacheLineIt, bool> p =
              evictor.nominate(set.begin(), set.end());
          p.second) {
        // Eviction required before command can complete.
        c.set_it(p.first);
        protocol->evict(c);
        check_resources(c);
      } else {
        // Eviction not required for the command to complete.
        c.set_it(p.first);
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
      protocol->apply(c);
      check_resources(c);
    }
  }

  void eval_msg(L2CacheContext& c, const AceCmdRspRMsg* rsp) const {
    const L2CacheModelProtocol* protocol = model_->protocol();
    protocol->apply(c);
  }

  
  L2TState* lookup_tt(Transaction* t) {
    L2TTable* tt = model_->tt();
    L2TTable::iterator it = tt->find(t);
    if (it == tt->end()) {
      LogMessage msg("Cannot find entry in transaction table.");
      msg.level(Level::Fatal);
      log(msg);
      return nullptr;
    }
    return it->second;
  }

  void check_resources(L2CacheContext& c) const {}

  void execute(const L2CacheContext& c) const {
    for (CoherenceAction* a : c.actions()) {
      a->execute();
    }
    update_ttable(c);
    MQArbTmt t = c.t();
    if (c.dequeue()) {
      t.winner()->dequeue();
      t.advance();
    }
  }

  void update_ttable(const L2CacheContext& c) const {
    L2TTable* tt = model_->tt();
    Transaction* trn = c.msg()->t();
    if (c.ttadd()) {
      // Create new state.
      L2TState* st = new L2TState;
      st->set_line(c.line());
      
      if (c.stalled()) {
        c.mq()->set_blocked(true);
        st->add_bmq(c.mq());
      }
      // Install context into table.
      tt->install(trn, st);
    } else if (c.ttdel()) {
      // Delete entry from table.
      L2TTable::iterator it = tt->find(trn);
      if (it == tt->end()) {
        LogMessage msg("Transaction not present in transaction table!");
        msg.level(Level::Fatal);
        log(msg);
      }
      L2TState* st = it->second;
      for (MessageQueue* mq : st->bmqs())
        mq->set_blocked(false);
      st->release();
      tt->remove(it);
    }
  }
  
  void wait_on_context(const L2CacheContext& c) {
    switch (c.wait()) {
      case L2Wait::MsgArrival: {
        MQArb* arb = model_->arb();
        wait_on(arb->request_arrival_event());
      } break;
      case L2Wait::NextEpochIfHasRequestOrWait: {
        MQArb* arb = model_->arb();
        MQArbTmt t = arb->tournament();
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
        LogMessage msg("Invalid wait condition.");
        msg.append(to_string(c.wait()));
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }
  // Pointer to parent L2.
  L2CacheModel* model_ = nullptr;
};

L2CacheModel::L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

L2CacheModel::~L2CacheModel() {
  delete cc_l2__rsp_q_;
  delete arb_;
  delete main_;
  delete cache_;
  delete protocol_;
}

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
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Transaction table.
  tt_ = new L2TTable(k(), "tt", 16);
  add_child_module(tt_);
  // Main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
  // Cache model
  cache_ = new L2Cache(config_.cconfig);
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

void L2CacheModel::set_l1cache_n(std::size_t n) { l2_l1__rsp_qs_.resize(n); }

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
