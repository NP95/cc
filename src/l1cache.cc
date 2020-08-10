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

const char* to_string(L1CmdOpcode opcode) {
  switch (opcode) {
    case L1CmdOpcode::CpuLoad:
      return "CpuLoad";
    case L1CmdOpcode::CpuStore:
      return "CpuStore";
    case L1CmdOpcode::Invalid:
      return "Invalid";
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
    case L1Opcode::StartTransaction:
      return "StartTransaction";
    case L1Opcode::EndTransaction:
      return "EndTransaction";
    case L1Opcode::MqSetBlockedOnEvent:
      return "MqSetBlockedOnEvent";
    case L1Opcode::MqSetBlockedOnTransaction:
      return "MqSetBlockedOnTransaction";
    case L1Opcode::MqSetBlockedOnTable:
      return "MqSetBlockedOnTable";
    case L1Opcode::MsgDequeue:
      return "MsgDequeue";
    case L1Opcode::MsgConsume:
      return "MsgConsume";
    case L1Opcode::MsgReissue:
      return "MsgReissue";
    case L1Opcode::RemoveLine:
      return "RemoveLine";
    case L1Opcode::InvokeCoherenceAction:
      return "InvokeCoherenceAction";
    case L1Opcode::WaitOnMsg:
      return "WaitOnMsg";
    case L1Opcode::WaitNextEpoch:
      return "WaitNextEpoch";
    case L1Opcode::SetL2LineModified:
      return "SetL2LineModified";
    case L1Opcode::ReserveReplaySlot:
      return "ReserveReplaySlot";
    case L1Opcode::Invalid:
      return "Invalid";
    default:
      return "Invalid";
  }
}

L1CacheContext::~L1CacheContext() {
  if (owns_line()) {
    line_->release();
  }
  if (owns_tstate()) {
    tstate_->release();
  }
}

std::string L1Command::to_string() const {
  using cc::to_string;
  using std::to_string;
  Hexer h;
  KVListRenderer r;
  r.add_field("opcode", cc::to_string(opcode()));
  switch (opcode()) {
    case L1Opcode::InvokeCoherenceAction: {
      r.add_field("action", oprands.action->to_string());
    } break;
    case L1Opcode::RemoveLine: {
      r.add_field("addr", h.to_hex(addr()));
    } break;
    default: {
    } break;
  }
  return r.to_string();
}

L1Command::~L1Command() {
  switch (opcode()) {
    case L1Opcode::InvokeCoherenceAction: {
      oprands.action->release();
    } break;
    default: {
    } break;
  }
}

L1Command* L1CommandBuilder::from_opcode(L1Opcode opcode) {
  return new L1Command(opcode);
}

L1Command* L1CommandBuilder::from_action(L1CoherenceAction* action) {
  L1Command* cmd = new L1Command(L1Opcode::InvokeCoherenceAction);
  cmd->oprands.action = action;
  return cmd;
}

L1Command* L1CommandBuilder::build_remove_line(addr_t addr) {
  L1Command* cmd = new L1Command(L1Opcode::RemoveLine);
  cmd->set_addr(addr);
  return cmd;
}

L1Command* L1CommandBuilder::build_blocked_on_event(MessageQueue* mq,
                                                    kernel::Event* e) {
  L1Command* cmd = new L1Command(L1Opcode::MqSetBlockedOnEvent);
  cmd->set_event(e);
  return cmd;
}

L1Command* L1CommandBuilder::build_start_transaction(Transaction* t) {
  L1Command* cmd = new L1Command(L1Opcode::StartTransaction);
  cmd->set_t(t);
  return cmd;
}

L1Command* L1CommandBuilder::build_end_transaction(Transaction* t) {
  L1Command* cmd = new L1Command(L1Opcode::EndTransaction);
  cmd->set_t(t);
  return cmd;
}

L1CommandList::~L1CommandList() {
  clear();
}

void L1CommandList::clear() {
  for (L1Command* cmd : cmds_) {
    cmd->release();
  }
}

void L1CommandList::push_back(L1Command* cmd) { cmds_.push_back(cmd); }

void L1CommandList::transaction_start(Transaction* t, bool is_blocking) {
  if (is_blocking) {
    // Blocking cache implementation:

    // MQ is blocked until the completion of the current command.
    push_back(cb::from_opcode(L1Opcode::MqSetBlockedOnTransaction));

    // Advance agent state to next epoch, but do not consume message
    // has this sits blocked at the head of the issue queue.
    next_and_do_consume(false);
  } else {
    // Reserve replay queue slot.
    push_back(cb::from_opcode(L1Opcode::ReserveReplaySlot));
    // Dequeue message, but do not release.
    push_back(cb::from_opcode(L1Opcode::MsgDequeue));
    // Advance to next epoch
    push_back(cb::from_opcode(L1Opcode::WaitNextEpoch));
  }

  // Transaction starts; install associated state in transaction
  // table.
  push_back(cb::build_start_transaction(t));
}

void L1CommandList::transaction_end(Transaction* t, bool was_blocking) {
  if (!was_blocking) {
    // If non-blocking command, re-issue command now that the
    // dependent transaction has completed.
    push_back(cb::from_opcode(L1Opcode::MsgReissue));
  }
  push_back(cb::build_end_transaction(t));
  // Advance to next epoch
  push_back(cb::from_opcode(L1Opcode::WaitNextEpoch));
}

void L1CommandList::next_and_do_consume(bool do_consume) {
  if (do_consume) {
    push_back(L1CommandBuilder::from_opcode(L1Opcode::MsgConsume));
  }
  push_back(L1CommandBuilder::from_opcode(L1Opcode::WaitNextEpoch));
}

L1Resources::L1Resources(const L1CommandList& list) {
  build(list);
}

void L1Resources::build(const L1CommandList& list) {
  for (L1Command* cmd : list) {
    const L1Opcode opcode = cmd->opcode();
    switch (opcode) {
      case L1Opcode::StartTransaction: {
        ++tt_entry_n_;
      } break;
      case L1Opcode::InvokeCoherenceAction: {
        L1CoherenceAction* action = cmd->action();
        action->set_resources(*this);
      } break;
      default:
        // No resources required.
        break;
    }
  }
}

L1TState::L1TState(kernel::Kernel* k) {
  transaction_start_ = new kernel::Event(k, "transaction_start");
  transaction_end_ = new kernel::Event(k, "transaction_end");
}

L1TState::~L1TState() {
  delete transaction_start_;
  delete transaction_end_;
}

class L1CommandInterpreter {
 public:
  L1CommandInterpreter() = default;

  void execute(L1CacheContext& ctxt, const L1Command* cmd) {
    const L1Opcode opcode = cmd->opcode();
    switch (opcode) {
      case L1Opcode::StartTransaction: {
        execute_start_transaction(ctxt, cmd);
      } break;
      case L1Opcode::EndTransaction: {
        execute_end_transaction(ctxt, cmd);
      } break;
      case L1Opcode::MqSetBlockedOnEvent: {
        execute_mq_set_blocked_on_event(ctxt, cmd);
      } break;
      case L1Opcode::MqSetBlockedOnTransaction: {
        execute_mq_set_blocked_on_transaction(ctxt, cmd);
      } break;
      case L1Opcode::MqSetBlockedOnTable: {
        execute_mq_set_blocked_on_table(ctxt, cmd);
      } break;
      case L1Opcode::MsgDequeue: {
        execute_msg_dequeue(ctxt, cmd, false);
      } break;
      case L1Opcode::MsgConsume: {
        execute_msg_dequeue(ctxt, cmd, true);
      } break;
      case L1Opcode::MsgReissue: {
        execute_msg_reissue(ctxt, cmd);
      } break;
      case L1Opcode::RemoveLine: {
        execute_remove_line(ctxt, cmd);
      } break;
      case L1Opcode::InvokeCoherenceAction: {
        execute_invoke_coherence_action(ctxt, cmd);
      } break;
      case L1Opcode::WaitOnMsg: {
        execute_wait_on_msg(ctxt, cmd);
      } break;
      case L1Opcode::WaitNextEpoch: {
        execute_wait_next_epoch(ctxt, cmd);
      } break;
      case L1Opcode::SetL2LineModified: {
        execute_set_l2_line_modified(ctxt, cmd);
      } break;
      case L1Opcode::ReserveReplaySlot: {
        execute_reserve_replay_slot(ctxt, cmd);
      } break;
      default: {
        throw std::runtime_error("Invalid opcode");
      } break;
    }
  }

 private:
  void execute_start_transaction(L1CacheContext& ctxt, const L1Command* cmd) {
    L1TState* tstate = ctxt.tstate();
    L1TTable* tt = ctxt.l1cache()->tt();
    tt->install(cmd->t(), tstate);
    ctxt.set_owns_tstate(false);

    if (ctxt.owns_line()) {
      // Install line in the dache.
      CacheModel<L1LineState*>* cache = ctxt.l1cache()->cache();
      const CacheAddressHelper ah = cache->ah();
      const addr_t addr = tstate->addr();
      L1CacheModelSet set = cache->set(ah.set(addr));
      if (L1CacheModelLineIt it = set.find(ah.tag(addr)); it == set.end()) {
        L1CacheModel::Evictor evictor;
        if (auto p = evictor.nominate(set.begin(), set.end()); !p.second) {
          // A way in the set has been nominated, install cache line.
          set.install(p.first, ah.tag(addr), tstate->line());
        } else {
          throw std::runtime_error(
              "Cannot install line in the directory cache; no free cache line "
              "ways are available.");
        }
      } else {
        throw std::runtime_error(
            "Cannot install line in directory cache; cache line is already "
            "present in the cache.");
      }
      ctxt.set_owns_line(false);
    }
    // Transaction starts; notify.
    tstate->transaction_start()->notify();
  }

  void execute_end_transaction(L1CacheContext& ctxt, const L1Command* cmd) {
    // Notify transaction event event; unblocks message queues
    // awaiting completion of current transaction.
    L1TState* tstate = ctxt.tstate();
    tstate->transaction_end()->notify();
    // Delete transaction from transaction table.
    L1TTable* tt = ctxt.l1cache()->tt();
    tt->remove(cmd->t());
    tstate->release();
  }

  void execute_mq_set_blocked_on_event(L1CacheContext& ctxt, const L1Command* cmd) {
    // Message Queue is blocked until event is notified.
    ctxt.mq()->set_blocked_until(cmd->event());
  }

  void execute_mq_set_blocked_on_transaction(L1CacheContext& ctxt,
                                             const L1Command* cmd) {
    // TODO: implement in terms of blocked on event
    L1TState* tstate = ctxt.tstate();
    ctxt.mq()->set_blocked_until(tstate->transaction_end());
  }

  void execute_mq_set_blocked_on_table(L1CacheContext& ctxt,
                                       const L1Command* cmd) {
    // TODO: implement in terms of blocked on event
    L1TTable* tt = ctxt.l1cache()->tt();
    ctxt.mq()->set_blocked_until(tt->non_full_event());
  }

  void execute_msg_dequeue(L1CacheContext& ctxt, const L1Command* cmd,
                           bool do_release = false) {
    const Message* msg = ctxt.mq()->dequeue();
    if (do_release) { msg->release(); }
    ctxt.t().advance();
  }

  void execute_msg_reissue(L1CacheContext& ctxt, const L1Command* cmd) {
    // Issue message contained within transaction object to message queue.
    const Message* msg = ctxt.tstate()->msg();
    MessageQueue* mq = ctxt.l1cache()->replay__cmd_q();
    if (!mq->issue(msg)) {
      throw std::runtime_error(
          "Attempt to replay message, but message queue is full.");
    }
  }
  
  // Derive addr from command opcode.
  void execute_remove_line(L1CacheContext& ctxt, const L1Command* cmd) {
    L1CacheModel* cache = ctxt.l1cache()->cache();
    const CacheAddressHelper ah = cache->ah();
    const addr_t addr = cmd->addr();
    L1CacheModelSet set = cache->set(ah.set(addr));
    if (auto it = set.find(addr); it != set.end()) {
      set.evict(it);
    } else {
      throw std::runtime_error("Cannot remove line, line is not present.");
    }
  }

  void execute_invoke_coherence_action(L1CacheContext& ctxt,
                                       const L1Command* cmd) {
    L1CoherenceAction* action = cmd->action();
    action->execute();
  }

  void execute_wait_on_msg(L1CacheContext& ctxt, const L1Command* cmd) {
    MQArb* arb = ctxt.l1cache()->arb();
    ctxt.process()->wait_on(arb->request_arrival_event());
  }

  void execute_wait_next_epoch(L1CacheContext& ctxt, const L1Command* cmd) {
    ctxt.process()->wait_for(kernel::Time{10, 0});
  }

  void execute_set_l2_line_modified(L1CacheContext& ctxt,
                                    const L1Command* cmd) {
    L2CacheAgent* l2cache = ctxt.l1cache()->l2cache();
    l2cache->set_cache_line_modified(ctxt.tstate()->addr());
  }

  void execute_reserve_replay_slot(L1CacheContext& ctxt,
                                   const L1Command* cmd) {
    // TODO
  }
};

//
//
class L1CacheAgent::MainProcess : public AgentProcess {
  using cb = L1CommandBuilder;

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheAgent* model)
      : AgentProcess(k, name), model_(model) {}

  // Initialization
  void init() override {
    L1CacheContext ctxt;
    ctxt.set_process(this);
    ctxt.set_l1cache(model_);
    L1CommandList cl;
    cl.push_back(cb::from_opcode(L1Opcode::WaitOnMsg));
    execute(ctxt, cl);
  }

  // Evaluation
  void eval() override {
    MQArb* arb = model_->arb();

    // Construct and initialize current processing context.
    L1CacheContext ctxt;
    ctxt.set_process(this);
    ctxt.set_l1cache(model_);
    ctxt.set_t(arb->tournament());
    ctxt.set_l1cache(model_);
    L1CommandList cl;

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

    LogMessage lm("Execute message: ");
    lm.append(ctxt.msg()->to_string());
    lm.level(Level::Debug);
    log(lm);

    check_resources(ctxt, cl);
    execute(ctxt, cl);
  }

  void set_cache_line_shared_or_invalid(addr_t addr, bool shared) {
    L1CommandList cl;
    L1CacheContext ctxt;
    ctxt.set_l1cache(model_);
    ctxt.set_addr(addr);
    L1Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    L1CacheSet set = cache->set(ah.set(addr));
    L1CacheLineIt it = set.find(ah.tag(addr));
    if (it != set.end()) {
      ctxt.set_line(it->t());
      const L1CacheAgentProtocol* protocol = model_->protocol();
      protocol->set_line_shared_or_invalid(ctxt, cl, shared);
    } else {
      LogMessage msg("L2 sets cache line status to ");
      msg.append(shared ? "Shared" : "Invalid");
      msg.append(" but line is not present in the cache.");
      msg.level(Level::Fatal);
      log(msg);
    }
    execute(ctxt, cl);
  }

 private:
  void process_l1cmd(L1CacheContext& ctxt, L1CommandList& cl) const {
    const L1CmdMsg* cmd = static_cast<const L1CmdMsg*>(ctxt.msg());
    L1Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const L1CacheAgentProtocol* protocol = model_->protocol();

    // Construct new instance of Transaction State as command (likely)
    // starts a new transaction round.
    L1TState* tstate = new L1TState(k());
    ctxt.set_tstate(tstate);
    ctxt.set_owns_tstate(true);

    // Otherwise, no transactions to current line in flight, therefore
    // query the cache to determine hit/miss status of line_id.
    const addr_t set_id = ah.set(cmd->addr());
    L1CacheSet set = cache->set(set_id);
    if (L1CacheLineIt it = set.find(ah.tag(cmd->addr())); it == set.end()) {
      // Line for current address has not been found in the set,
      // therefore a new line must either be installed or, if the set
      // is currently full, another line must be nominated and
      // evicted.
      L1Cache::Evictor evictor;
      if (const std::pair<L1CacheLineIt, bool> p =
              evictor.nominate(set.begin(), set.end());
          p.second) {
        ctxt.set_addr(ah.addr_from_set_tag(set_id, p.first->tag()));
        ctxt.set_line(p.first->t());
        protocol->evict(ctxt, cl);
      } else {
        ctxt.set_line(protocol->construct_line());
        ctxt.set_owns_line(true);
        protocol->apply(ctxt, cl);
      }
    } else {
      // Line is present in the cache, apply state update.
      ctxt.set_line(it->t());
      protocol->apply(ctxt, cl);
    }
  }

  void process_l2cmdrsp(L1CacheContext& ctxt, L1CommandList& cl) const {
    Transaction* t = ctxt.msg()->t();
    L1TTable* tt = model_->tt();
    if (L1TTable::iterator it = tt->find(t); it != tt->end()) {
      const L1CacheAgentProtocol* protocol = model_->protocol();
      const L1TState* st = it->second;
      ctxt.set_line(st->line());
      ctxt.set_tstate(it->second);
      protocol->apply(ctxt, cl);
    } else {
      LogMessage lm("Cannot find transaction table entry for ");
      lm.append(to_string(t));
      lm.level(Level::Fatal);
      log(lm);
    }
  }

  void check_resources(L1CacheContext& ctxt, L1CommandList& cl) const {
    // Compute the Agent resources required to execute the command
    // list given by 'cl'. If the agent has insufficient resources,
    // the ENTIRE command list must be killed and the agent blocked
    // awaiting the arrival of sufficient resources. The command list
    // must execute atomically, otherwise if it was to become blocked
    // after a partial application and deadlock could occur.
    
    const L1Resources res(cl);

    L1TTable* tt = model_->tt();
    if (!tt->has_at_least(res.tt_entry_n())) {
      // No transaction table entries available. Block Process until
      // sufficient space has been attained.

      // Destroy prior CommandList and issue new command to block
      // current process.
      cl.clear();

      // Current Message(Queue) becomes belocked on the Transaction
      // Table.
      cl.push_back(cb::from_opcode(L1Opcode::MqSetBlockedOnTable));
      return;
    }

    auto check_mq_credits = [&](const MessageQueueProxy* mq, std::size_t n) -> bool {
      bool ret = true;
      if (!mq->has_at_least(n)) {
        // Insufficient space in L2 command queue.
      
        // Destory old command list.
        cl.clear();

        // Message Queue becomes blocked awaiting credit to destination
        // queue.
        cl.push_back(cb::build_blocked_on_event(ctxt.mq(), mq->add_credit_event()));
        ret = false;
      }
      return ret;
    };

    if (!check_mq_credits(model_->l1_l2__cmd_q(), res.l2_cmd_n())) return;
    if (!check_mq_credits(model_->l1_cpu__rsp_q(), res.cpu_rsp_n())) return;

    // Resources have been attained; command list is ready to be
    // executed and committed to the agent's state.
  }

  void execute(L1CacheContext& ctxt, const L1CommandList& cl) {
    try {
      L1CommandInterpreter interpreter;
      for (const L1Command* cmd : cl) {
        LogMessage lm("Executing cmd: ");
        lm.append(cmd->to_string());
        lm.level(Level::Debug);
        log(lm);
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
  L1CacheAgent* model_ = nullptr;
};

L1CacheAgent::L1CacheAgent(kernel::Kernel* k, const L1CacheAgentConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

L1CacheAgent::~L1CacheAgent() {
  delete cpu_l1__cmd_q_;
  delete replay__cmd_q_;
  delete l2_l1__rsp_q_;
  delete l1_l2__cmd_q_;
  delete l1_cpu__rsp_q_;
  delete arb_;
  delete tt_;
  delete main_;
  delete cache_;
  delete protocol_;
}

void L1CacheAgent::build() {
  // Construct command request queue
  cpu_l1__cmd_q_ = new MessageQueue(k(), "cpu_l1__cmd_q", config_.cpu_l1__cmd_n);
  add_child_module(cpu_l1__cmd_q_);
  // Construct replay queue
  replay__cmd_q_ = new MessageQueue(k(), "replay__cmd_q", config_.replay__cmd_n);
  add_child_module(replay__cmd_q_);
  // Construct L2 -> L1 response queue
  l2_l1__rsp_q_ = new MessageQueue(k(), "l2_l1__rsp_q", config_.l2_l1__rsp_n);
  add_child_module(l2_l1__rsp_q_);
  // Arbiter
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Transaction table.
  tt_ = new L1TTable(k(), "tt", config_.tt_entries_n);
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

void L1CacheAgent::set_l1_l2__cmd_q(MessageQueueProxy* mq) {
  l1_l2__cmd_q_ = mq;
  add_child_module(l1_l2__cmd_q_);
}

void L1CacheAgent::set_l1_cpu__rsp_q(MessageQueueProxy* mq) {
  l1_cpu__rsp_q_ = mq;
  add_child_module(l1_cpu__rsp_q_);
}

void L1CacheAgent::elab() {
  arb_->add_requester(cpu_l1__cmd_q_);
  arb_->add_requester(replay__cmd_q_);
  arb_->add_requester(l2_l1__rsp_q_);
}

void L1CacheAgent::drc() {
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

void L1CacheAgent::set_cache_line_shared_or_invalid(addr_t addr, bool shared) {
  main_->set_cache_line_shared_or_invalid(addr, shared);
}

}  // namespace cc
