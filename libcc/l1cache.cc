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

  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("opcode", to_string(opcode()));
  Hexer h;
  r.add_field("addr", h.to_hex(addr()));
  return r.to_string();
}

//
//
L1CmdRspMsg::L1CmdRspMsg() : Message(MessageClass::L1CmdRsp) {}

//
//
std::string L1CmdRspMsg::to_string() const {
  using cc::to_string;

  KVListRenderer r;
  r.add_field("cls", to_string(cls()));
  return r.to_string();
}

const char* to_string(L1Opcode opcode) {
  switch (opcode) {
#define __declare_to_string(__name) \
  case L1Opcode::__name:            \
    return #__name;
    L1OPCODE_LIST(__declare_to_string)
#undef __declare_to_string
    default:
      return "Invalid";
  }
}

L1CacheContext::~L1CacheContext() {
  if (owns_line()) {
    // line_->release();
    delete line_;
  }
}

std::string L1Command::to_string() const {
  KVListRenderer r;
  r.add_field("opcode", cc::to_string(opcode()));
  switch (opcode()) {
    case L1Opcode::InvokeCoherenceAction: {
      r.add_field("action", oprands.coh.action->to_string());
    } break;
    default: {
    } break;
  }
  return r.to_string();
}


L1Command::~L1Command() {
  switch (opcode()) {
    case L1Opcode::InvokeCoherenceAction: {
      oprands.coh.action->release();
    } break;
    default: {
    } break;
  }
}

L1Command* L1CommandBuilder::from_opcode(L1Opcode opcode) {
  return new L1Command(opcode);
}

L1Command* L1CommandBuilder::from_action(CoherenceAction* action) {
  L1Command* cmd = new L1Command(L1Opcode::InvokeCoherenceAction);
  cmd->oprands.coh.action = action;
  return cmd;
}

L1CommandList::~L1CommandList() {
  for (L1Command* cmd : cmds_) {
    cmd->release();
  }
}

void L1CommandList::push_back(L1Command* cmd) { cmds_.push_back(cmd); }

class L1CommandInterpreter {
  struct State {
    // Current transaction
    const Transaction* t;
    // Transaction state
    L1TState* ts;
    // Address of current command.
    addr_t addr;
  };

 public:
  L1CommandInterpreter() = default;

  void set_l1cache(L1CacheModel* model) { model_ = model; }
  void set_process(AgentProcess* process) { process_ = process; }

  void execute(L1CacheContext& ctxt, const L1Command* c) {
    switch (c->opcode()) {
      default: {
      } break;
#define __declare_dispatcher(__name) \
  case L1Opcode::__name:             \
    execute##__name(ctxt, c);        \
    break;
        L1OPCODE_LIST(__declare_dispatcher)
#undef __declare_dispatcher
    }
  }

 private:
  void executeTableInstall(L1CacheContext& ctxt, const L1Command* cmd) {
    L1TTable* tt = model_->tt();
    state_.t = ctxt.msg()->t();
    state_.ts = new L1TState();
    state_.ts->set_line(ctxt.line());
    tt->install(state_.t, state_.ts);
  }

  void executeTableGetCurrentState(L1CacheContext& ctxt, const L1Command* cmd) {
    // Lookup the Transaction Table for the current transaction
    // and set the table pointer for subsequent operations.
    L1TTable* tt = model_->tt();
    auto it = tt->find(ctxt.msg()->t());
    if (it == tt->end()) {
      throw std::runtime_error(
          "Transaction not present in the transaction table");
    }
    state_.t = it->first;
    state_.ts = it->second;
  }

  void executeTableRemove(L1CacheContext& ctxt, const L1Command* cmd) const {
    // Remove the current Transsaction context from the
    // Transaction Table.
    state_.ts->release();
    L1TTable* tt = model_->tt();
    tt->remove(state_.t);
  }

  void executeTableMqUnblockAll(L1CacheContext& ctxt,
                                const L1Command* cmd) const {
    // For the current table context, unblock all messages
    // queues that are currently blocked awaiting completion of
    // the current transaction.
    for (MessageQueue* mq : state_.ts->mq()) {
      mq->set_blocked(false);
    }
  }

  void executeTableMqAddToBlockedList(L1CacheContext& ctxt,
                                      const L1Command* cmd) const {
    // Add the currently address Message Queue to the set of
    // queues blocked on the current transaction.
    state_.ts->add_blocked_mq(ctxt.mq());
  }

  void executeWaitOnMsg(L1CacheContext& ctxt, const L1Command* cmd) const {
    // Set wait state of current process; await the arrival of a
    // new message.
    MQArb* arb = model_->arb();
    process_->wait_on(arb->request_arrival_event());
  }

  void executeWaitNextEpochOrWait(L1CacheContext& ctxt,
                                  const L1Command* cmd) const {
    MQArb* arb = model_->arb();
    MQArbTmt t = arb->tournament();
    if (t.has_requester()) {
      // Wait some delay
      process_->wait_for(kernel::Time{10, 0});
    } else {
      // No further commands, block process until something
      // arrives.
      process_->wait_on(arb->request_arrival_event());
    }
  }

  void executeMqSetBlocked(L1CacheContext& ctxt, const L1Command* cmd) const {
    // Set the blocked status of the current Message Queue.
    ctxt.mq()->set_blocked(true);
  }

  void executeMsgConsume(L1CacheContext& ctxt, const L1Command* cmd) const {
    // Dequeue and release the head message of the currently
    // addressed Message Queue.
    const Message* msg = ctxt.mq()->dequeue();
    msg->release();
    ctxt.t().advance();
  }

  void executeMsgL1CmdExtractAddr(L1CacheContext& ctxt, const L1Command* cmd) {
    // Extract address field from command message.
    state_.addr = static_cast<const L1CmdMsg*>(ctxt.msg())->addr();
  }

  void executeInstallLine(L1CacheContext& ctxt, const L1Command* cmd) const {
    // Install line within the current context into the cache at
    // an appropriate location. Expects that any prior evictions
    // to thg destination set have already taken place.
    L1Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    L1CacheSet set = cache->set(ah.set(state_.addr));
    L1Cache::Evictor evictor;
    if (const std::pair<L1CacheLineIt, bool> p =
        evictor.nominate(set.begin(), set.end());
        !p.second) {
      set.install(p.first, ah.tag(state_.addr), ctxt.line());
      ctxt.set_owns_line(false);
    } else {
      throw std::runtime_error("Cannot install line as set is full.");
    }
  }

  void executeSetL2LineDirty(L1CacheContext& ctxt, const L1Command* cmd) const {
    L2CacheModel* l2cache = ctxt.l1cache()->l2cache();
    l2cache->set_cache_line_modified(ctxt.addr());
  }

  void executeInvokeCoherenceAction(L1CacheContext& ctxt,
                                    const L1Command* cmd) const {
    CoherenceAction* action = cmd->action();
    action->execute();
  }

  //
  State state_;
  //
  AgentProcess* process_ = nullptr;
  //
  L1CacheModel* model_ = nullptr;
};

//
//
class L1CacheModel::MainProcess : public AgentProcess {
  using cb = L1CommandBuilder;

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : AgentProcess(k, name), model_(model) {}

  // Initialization
  void init() override {
    L1CacheContext c;
    L1CommandList cl;
    cl.push_back(cb::from_opcode(L1Opcode::WaitOnMsg));
    execute(c, cl);
  }

  // Evaluation
  void eval() override {
    MQArb* arb = model_->arb();

    // Construct and initialize current processing context.
    L1CommandList cl;
    L1CacheContext ctxt;
    ctxt.set_t(arb->tournament());
    ctxt.set_l1cache(model_);

    // Check if requests are present, if not block until a new message
    // arrives at the arbiter. Process should ideally not wake in the
    // absence of requesters.
    if (!ctxt.t().has_requester()) {
      cl.push_back(cb::from_opcode(L1Opcode::WaitOnMsg));
      execute(ctxt, cl);
      return;
    }

    // Fetch nominated message queue
    ctxt.set_mq(ctxt.t().winner());

    // Dispatch to appropriate handler based upon message class.
    switch (ctxt.msg()->cls()) {
      case MessageClass::L1Cmd:
        process_l1cmd(ctxt, cl);
        break;
      case MessageClass::L2CmdRsp:
        process_l2cmdrsp(ctxt, cl);
        break;
      default: {
        LogMessage lmsg("Invalid message class received: ");
        lmsg.append(cc::to_string(ctxt.msg()->cls()));
        lmsg.level(Level::Error);
        log(lmsg);
      } break;
    }

    if (can_execute(cl)) {
      LogMessage lm("Execute message: ");
      lm.append(ctxt.msg()->to_string());
      lm.level(Level::Debug);
      log(lm);

      execute(ctxt, cl);
    }
  }

  void set_cache_line_shared_or_invalid(addr_t addr, bool shared) {
    L1CommandList cl;
    L1CacheContext ctxt;
    ctxt.set_l1cache(model_);
    L1Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    L1CacheSet set = cache->set(ah.set(addr));
    L1CacheLineIt it = set.find(ah.tag(addr));
    if (it != set.end()) {
      ctxt.set_line(it->t());
      const L1CacheModelProtocol* protocol = model_->protocol();
      protocol->set_line_shared_or_invalid(ctxt, cl, shared);
    } else {
      LogMessage msg("L2 sets cache line status to ");
      msg.append(shared ? "shared" : "Invalid");
      msg.append(" but line is not present in the cache.");
      msg.level(Level::Fatal);
      log(msg);
    }
    execute(ctxt, cl);
  }

 private:
  void process_l1cmd(L1CacheContext& c, L1CommandList& cl) const {
    const L1CmdMsg* cmd = static_cast<const L1CmdMsg*>(c.msg());
    c.set_addr(cmd->addr());
    L1Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const L1CacheModelProtocol* protocol = model_->protocol();

    // Otherwise, no transactions to current line in flight, therefore
    // query the cache to determine hit/miss status of line_id.
    L1CacheSet set = cache->set(ah.set(cmd->addr()));
    if (L1CacheLineIt it = set.find(ah.tag(cmd->addr())); it == set.end()) {
      // Line for current address has not been found in the set,
      // therefore a new line must either be installed or, if the set
      // is currently full, another line must be nominated and
      // evicted.
      L1Cache::Evictor evictor;
      if (const std::pair<L1CacheLineIt, bool> p =
              evictor.nominate(set.begin(), set.end());
          p.second) {
        // TODO
      } else {
        c.set_line(protocol->construct_line());
        c.set_owns_line(true);
        protocol->apply(c, cl);
      }
    } else {
      // Line is present in the cache, apply state update.
      c.set_line(it->t());
      protocol->apply(c, cl);
    }
  }

  void process_l2cmdrsp(L1CacheContext& c, L1CommandList& cl) const {
    Transaction* t = c.msg()->t();
    L1TTable* tt = model_->tt();
    if (L1TTable::iterator it = tt->find(t); it != tt->end()) {
      const L1CacheModelProtocol* protocol = model_->protocol();
      const L1TState* st = it->second;
      c.set_line(st->line());
      protocol->apply(c, cl);
    } else {
      LogMessage lm("Cannot find transaction table entry for ");
      lm.append(to_string(t));
      lm.level(Level::Fatal);
      log(lm);
    }
  }

  bool can_execute(L1CommandList& cl) const { return true; }

  void execute(L1CacheContext& ctxt, const L1CommandList& cl) {
    try {
      L1CommandInterpreter interpreter;
      interpreter.set_l1cache(model_);
      interpreter.set_process(this);
      for (const L1Command* cmd : cl) {
#if 0
        LogMessage lm("Executing cmd: ");
        lm.append(cmd->to_string());
        lm.level(Level::Debug);
        log(lm);
#endif
        interpreter.execute(ctxt, cmd);
      }
    } catch (const std::runtime_error& ex) {
      LogMessage lm("Interpreter encountered an error: ");
      lm.append(ex.what());
      lm.level(Level::Fatal);
      log(lm);
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
  delete l1_l2__cmd_q_;
  delete l1_cpu__rsp_q_;
  delete arb_;
  delete tt_;
  delete main_;
  delete cache_;
  delete protocol_;
}

void L1CacheModel::build() {
  // Construct command request queue
  cpu_l1__cmd_q_ = new MessageQueue(k(), "cpu_l1__cmd_q", 16);
  add_child_module(cpu_l1__cmd_q_);
  // Construct L2 -> L1 response queue
  l2_l1__rsp_q_ = new MessageQueue(k(), "l2_l1__rsp_q", 16);
  add_child_module(l2_l1__rsp_q_);
  // Arbiter
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Transaction table.
  tt_ = new L1TTable(k(), "tt", 16);
  add_child_module(tt_);
  // Main thread of execution
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
  //
  cache_ = new L1Cache(config_.cconfig);
  // Set up protocol
  protocol_ = config_.pbuilder->create_l1(k());
  add_child_module(protocol_);
}

void L1CacheModel::set_l1_l2__cmd_q(MessageQueueProxy* mq) {
  l1_l2__cmd_q_ = mq;
  add_child_module(l1_l2__cmd_q_);
}

void L1CacheModel::set_l1_cpu__rsp_q(MessageQueueProxy* mq) {
  l1_cpu__rsp_q_ = mq;
  add_child_module(l1_cpu__rsp_q_);
}

void L1CacheModel::elab() {
  arb_->add_requester(cpu_l1__cmd_q_);
  arb_->add_requester(l2_l1__rsp_q_);
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
    LogMessage msg("CPU has not been bound.", Level::Fatal);
    log(msg);
  }
  if (l2cache_ == nullptr) {
    LogMessage msg("L2 has not been bound.", Level::Fatal);
    log(msg);
  }
}

void L1CacheModel::set_cache_line_shared_or_invalid(addr_t addr, bool shared) {
  main_->set_cache_line_shared_or_invalid(addr, shared);
}

}  // namespace cc
