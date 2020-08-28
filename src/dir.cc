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

#include "dir.h"

#include <sstream>

#include "amba.h"
#include "cache.h"
#include "llc.h"
#include "msg.h"
#include "noc.h"
#include "primitives.h"
#include "protocol.h"
#include "utility.h"
#include "verif.h"

namespace cc {

const char* to_string(DirOpcode opcode) {
  switch (opcode) {
    case DirOpcode::StartTransaction:
      return "StartTransaction";
    case DirOpcode::EndTransaction:
      return "EndTransaction";
    case DirOpcode::MsgConsume:
      return "MsgConsume";
    case DirOpcode::RemoveLine:
      return "RemoveLine";
    case DirOpcode::MqSetBlockedOnTransaction:
      return "MqSetBlockedOnTransaction";
    case DirOpcode::MqSetBlockedOnTable:
      return "MqSetBlockedOnTable";
    case DirOpcode::MqSetBlockedOnEvt:
      return "MqSetBlockedOnEvt";
    case DirOpcode::InvokeCoherenceAction:
      return "InvokeCoherenceAction";
    case DirOpcode::WaitOnMsg:
      return "WaitOnMsg";
    case DirOpcode::WaitNextEpoch:
      return "WaitNextEpoch";
    case DirOpcode::Invalid:
      return "Invalid";
    default:
      return "Invalid";
  }
}

DirCommand::~DirCommand() {
  switch (opcode()) {
    case DirOpcode::InvokeCoherenceAction:
      oprands.action->release();
      break;
    default:
      break;
  }
}

std::string DirCommand::to_string() const {
  using cc::to_string;

  KVListRenderer r;
  r.add_field("opcode", to_string(opcode()));
  switch (opcode()) {
    case DirOpcode::InvokeCoherenceAction: {
      r.add_field("action", oprands.action->to_string());
    } break;
    default: {
    } break;
  }
  return r.to_string();
}

DirCommand* DirCommandBuilder::from_opcode(DirOpcode opcode) {
  return new DirCommand(opcode);
}

DirCommand* DirCommandBuilder::from_action(DirCoherenceAction* action) {
  DirCommand* cmd = new DirCommand(DirOpcode::InvokeCoherenceAction);
  cmd->oprands.action = action;
  return cmd;
}

DirCommand* DirCommandBuilder::build_blocked_on_event(MessageQueue* mq,
                                                      kernel::Event* e) {
  DirCommand* cmd = new DirCommand(DirOpcode::MqSetBlockedOnEvt);
  cmd->set_event(e);
  return cmd;
}

DirCommand* DirCommandBuilder::build_error(const std::string& reason) {
  DirCommand* cmd = new DirCommand(DirOpcode::Error);
  cmd->set_reason(reason);
  return cmd;
}


enum class TStateUpdateOpcode {
  // Set expected snoop count
  SetSnoopN,

  // Increment received snoop count
  IncSnoopI,

  // Increment Data Transfer (DT) count
  IncDt,

  // Increment Pass Dirty (PD) count
  IncPd,

  // Increment Is Shared (IS) count
  IncIs,

  // Set LLC opcode
  SetLLC,

  // Invalid; placeholder.
  Invalid
};

const char* to_string(TStateUpdateOpcode action) {
  switch (action) {
    case TStateUpdateOpcode::SetSnoopN:
      return "SetSnoopN"; 
    case TStateUpdateOpcode::IncSnoopI:
      return "IncSnoopI";
    case TStateUpdateOpcode::IncDt:
      return "IncDt";
    case TStateUpdateOpcode::IncPd:
      return "IncPd";
    case TStateUpdateOpcode::IncIs:
      return "IncIs";
    case TStateUpdateOpcode::SetLLC:
      return "SetLLC";
    case TStateUpdateOpcode::Invalid:
      [[fallthrough]];
    default:
      return "Invalid";
  }
}

// Coherence action which applies the state updates to the directory's
// transaction state
//
struct TStateUpdateAction : public DirCoherenceAction {
  TStateUpdateAction(DirTState* tstate, TStateUpdateOpcode action)
      : tstate_(tstate), action_(action) {}
  std::string to_string() const override {
    using cc::to_string;
    KVListRenderer r;
    r.add_field("action", to_string(action_));
    switch (action()) {
      case TStateUpdateOpcode::SetSnoopN: {
        r.add_field("snoop_n", to_string(snoop_n_));
        r.add_field("snoop_i", to_string(0));
      } break;
      case TStateUpdateOpcode::SetLLC: {
        r.add_field("llc_cmd_opcode", to_string(llc_cmd_opcode_));
      } break;
      default: {
      } break;
    }
    return r.to_string();
  }

  // Accessors:

  // Current update action
  TStateUpdateOpcode action() const { return action_; }

  // Current snoop count.
  std::size_t snoop_n() const { return snoop_n_; }


  // Setters:

  // Set snoop count.
  void set_snoop_n(std::size_t snoop_n) { snoop_n_ = snoop_n; }

  // Set LLC Command opcode.
  void set_llc_cmd_opcode(LLCCmdOpcode opcode) { llc_cmd_opcode_ = opcode; }

  bool execute() override {
    switch (action()) {
      case TStateUpdateOpcode::SetSnoopN: {
        tstate_->set_snoop_n(snoop_n_);
        tstate_->set_snoop_i(0);
      } break;
      case TStateUpdateOpcode::IncSnoopI: {
        tstate_->set_snoop_i(tstate_->snoop_n() + 1);
      } break;
      case TStateUpdateOpcode::IncDt: {
        tstate_->set_dt_i(tstate_->dt_i() + 1);
      } break;
      case TStateUpdateOpcode::IncPd: {
        tstate_->set_pd_i(tstate_->pd_i() + 1);
      } break;
      case TStateUpdateOpcode::IncIs: {
        tstate_->set_is_i(tstate_->is_i() + 1);
      } break;
      case TStateUpdateOpcode::SetLLC: {
        tstate_->set_llc_cmd_opcode(llc_cmd_opcode_);
      }
      default: {
      } break;
    }
    return true;
  }

 private:
  // Pending LLC command
  LLCCmdOpcode llc_cmd_opcode_ = LLCCmdOpcode::Invalid;
  // Pointer to transaction state instance.
  DirTState* tstate_ = nullptr;
  // Snoop count
  std::size_t snoop_n_ = 0;
  // Requests state update action.
  TStateUpdateOpcode action_ = TStateUpdateOpcode::Invalid;
};


DirCommand* DirTState::build_set_llc_cmd_opcode(LLCCmdOpcode opcode) {
  TStateUpdateAction* action =
      new TStateUpdateAction(this, TStateUpdateOpcode::SetLLC);
  action->set_llc_cmd_opcode(opcode);
  return DirCommandBuilder::from_action(action);
}

// Build action to Increment DT
DirCommand* DirTState::build_inc_dt() {
  return DirCommandBuilder::from_action(
      new TStateUpdateAction(this, TStateUpdateOpcode::IncDt));
}

// Build action to Increment PD
DirCommand* DirTState::build_inc_pd() {
  return DirCommandBuilder::from_action(
      new TStateUpdateAction(this, TStateUpdateOpcode::IncPd));
}

// Build action to Increment IS
DirCommand* DirTState::build_inc_is() {
  return DirCommandBuilder::from_action(
      new TStateUpdateAction(this, TStateUpdateOpcode::IncIs));
}

DirCommand* DirTState::build_set_snoop_n(std::size_t n) {
  TStateUpdateAction* action =
      new TStateUpdateAction(this, TStateUpdateOpcode::SetSnoopN);
  action->set_snoop_n(n);
  return DirCommandBuilder::from_action(action);
}

// Build action to increment snoop response count.
DirCommand* DirTState::build_inc_snoop_i() {
  return DirCommandBuilder::from_action(
      new TStateUpdateAction(this, TStateUpdateOpcode::IncSnoopI));
}

void DirTState::release() { delete this; }

bool DirTState::is_final_snoop(bool is_snoop_rsp) const {
  // Not expecting any snoops.
  if (snoop_n() == 0) { return true; }

  // Otherwise, if when called in snoop response
  return snoop_n() == (is_snoop_rsp ? 1 : 0) + snoop_i();
}

std::size_t DirResources::coh_snp_n(const Agent* agent) const {
  std::size_t n = 0;
  if (auto it = coh_snp_n_.find(agent); it != coh_snp_n_.end()) {
    n = it->second;
  }
  return n;
}

void DirResources::set_coh_snp_n(const Agent* agent, std::size_t n) {
  coh_snp_n_[agent] = n;
}

DirContext::~DirContext() {
  if (owns_line()) {
    tstate_->line()->release();
  }
  if (owns_tstate()) {
    tstate_->release();
  }
}

DirCommandList::~DirCommandList() { clear(); }

void DirCommandList::clear() {
  for (DirCommand* cmd : cmds_) {
    cmd->release();
  }
  cmds_.clear();
}

void DirCommandList::push_back(DirOpcode opcode) {
  push_back(DirCommandBuilder::from_opcode(opcode));
}

void DirCommandList::push_back(DirCoherenceAction* action) {
  push_back(DirCommandBuilder::from_action(action));
}

void DirCommandList::push_back(DirCommand* cmd) { cmds_.push_back(cmd); }

void DirCommandList::raise_error(const std::string& reason) {
  clear();
  push_back(DirCommandBuilder::build_error(reason));
}

void DirCommandList::next_and_do_consume(bool do_consume) {
  if (do_consume) {
    push_back(DirCommandBuilder::from_opcode(DirOpcode::MsgConsume));
  }
  push_back(DirCommandBuilder::from_opcode(DirOpcode::WaitNextEpoch));
}

DirTState::DirTState(kernel::Kernel* k) {
  transaction_start_ = new kernel::Event(k, "transaction_start");
  transaction_end_ = new kernel::Event(k, "transaction_end");
}

DirTState::~DirTState() {
  delete transaction_start_;
  delete transaction_end_;
}

//
//
class DirCommandInterpreter {
 public:
  DirCommandInterpreter(kernel::Kernel* k) : k_(k) {}

  void set_dir(DirAgent* model) { model_ = model; }
  void set_process(AgentProcess* process) { process_ = process; }

  void execute(DirContext& ctxt, const DirCommand* cmd) {
    const DirOpcode opcode = cmd->opcode();
    switch (cmd->opcode()) {
      case DirOpcode::StartTransaction: {
        execute_start_transaction(ctxt, cmd);
      } break;
      case DirOpcode::EndTransaction: {
        execute_end_transaction(ctxt, cmd);
      } break;
      case DirOpcode::MsgConsume: {
        execute_msg_consume(ctxt, cmd);
      } break;
      case DirOpcode::RemoveLine: {
        execute_remove_line(ctxt, cmd);
      } break;
      case DirOpcode::MqSetBlockedOnTransaction: {
        execute_mq_set_blocked_on_transaction(ctxt, cmd);
      } break;
      case DirOpcode::MqSetBlockedOnTable: {
        execute_mq_set_blocked_on_table(ctxt, cmd);
      } break;
      case DirOpcode::MqSetBlockedOnEvt: {
        execute_mq_set_blocked_on_event(ctxt, cmd);
      } break;
      case DirOpcode::InvokeCoherenceAction: {
        execute_invoke_coherence_action(ctxt, cmd);
      } break;
      case DirOpcode::WaitOnMsg: {
        execute_wait_on_msg(ctxt, cmd);
      } break;
      case DirOpcode::WaitNextEpoch: {
        execute_wait_next_epoch(ctxt, cmd);
      } break;
      default: {
      } break;
    }
  }

 private:
  void execute_start_transaction(DirContext& ctxt, const DirCommand* cmd) {
    DirTState* tstate = ctxt.tstate();

    // Install in the transaction table.
    Table<Transaction*, DirTState*>* tt = model_->tt();
    tt->install(ctxt.msg()->t(), tstate);
    ctxt.set_owns_tstate(false);

    if (ctxt.owns_line()) {
      // Install line in the dache.
      CacheModel<DirLineState*>* cache = model_->cache();
      const CacheAddressHelper ah = cache->ah();
      auto set = cache->set(ah.set(tstate->addr()));
      if (auto it = set.find(ah.tag(tstate->addr())); it == set.end()) {
        CacheModel<DirLineState*>::Evictor evictor;
        if (auto p = evictor.nominate(set.begin(), set.end()); !p.second) {
          // A way in the set has been nominated, install cache line.
          set.install(p.first, ah.tag(tstate->addr()), tstate->line());
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

  void execute_end_transaction(DirContext& ctxt, const DirCommand* cmd) {
    // Notify transaction event event; unblocks message queues
    // awaiting completion of current transaction.
    ctxt.tstate()->transaction_end()->notify();
    // Delete transaction from transaction table.
    Table<Transaction*, DirTState*>* tt = model_->tt();
    tt->remove(ctxt.msg()->t());
    ctxt.tstate()->release();
  }

  void execute_msg_consume(DirContext& ctxt, const DirCommand* cmd) {
    // Dequeue and release the head message of the currently
    // addressed Message Queue.
    const Message* msg = ctxt.mq()->dequeue();
    msg->release();
    ctxt.t().advance();
  }

  void execute_remove_line(DirContext& ctxt, const DirCommand* cmd) {
    CacheModel<DirLineState*>* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    auto set = cache->set(ah.set(ctxt.addr()));
    if (auto it = set.find(ah.tag(ctxt.addr())); it != set.end()) {
      set.evict(it);
    } else {
      throw std::runtime_error("Cannot remove line, line is not present.");
    }
  }

  void execute_invoke_coherence_action(DirContext& ctxt,
                                       const DirCommand* cmd) {
    DirCoherenceAction* action = cmd->action();
    action->execute();
  }

  void execute_mq_set_blocked_on_transaction(DirContext& ctxt,
                                             const DirCommand* cmd) {
    ctxt.mq()->set_blocked_until(ctxt.tstate()->transaction_end());
  }

  void execute_mq_set_blocked_on_table(DirContext& ctxt,
                                       const DirCommand* cmd) {
    ctxt.mq()->set_blocked_until(model_->tt()->non_full_event());
  }

  void execute_mq_set_blocked_on_event(DirContext& ctxt,
                                       const DirCommand* cmd) {
    ctxt.mq()->set_blocked_until(cmd->event());
  }

  void execute_wait_on_msg(DirContext& ctxt, const DirCommand* cmd) const {
    // Set wait state of current process; await the arrival of a
    // new message.
    MQArb* arb = model_->arb();
    process_->wait_on(arb->request_arrival_event());
  }

  void execute_wait_next_epoch(DirContext& ctxt, const DirCommand* cmd) const {
    process_->wait_epoch();
  }

  // Simulation kernel instance
  kernel::Kernel* k_ = nullptr;

  // Invoke process context
  AgentProcess* process_ = nullptr;

  // Directory agent instance.
  DirAgent* model_ = nullptr;
};

//
//
DirResources::DirResources(const DirCommandList& cl) { build(cl); }

void DirResources::build(const DirCommandList& cl) {
  for (DirCommand* cmd : cl) {
    const DirOpcode opcode = cmd->opcode();
    switch (opcode) {
      case DirOpcode::StartTransaction: {
        tt_entry_n_++;
      } break;
      default: {
        // No resources required.
      } break;
    }
  }
}

//
//
class DirAgent::RdisProcess : public AgentProcess {
  using cb = DirCommandBuilder;

 public:
  RdisProcess(kernel::Kernel* k, const std::string& name, DirAgent* model)
      : AgentProcess(k, name), model_(model) {}

 private:
  // Initialization
  void init() override {
    DirContext ctxt;
    DirCommandList cl;
    cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsg));
    execute(ctxt, cl);
  }

  // Evaluation
  void eval() override {
    MQArb* arb = model_->arb();

    // Construct and initialize current processing context.
    DirCommandList cl;
    DirContext ctxt;
    ctxt.set_t(arb->tournament());

    // Check if requests are present, if not block until a new message
    // arrives at the arbiter. Process should ideally not wake in the
    // absence of requesters.
    if (!ctxt.t().has_requester()) {
      cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsg));
      execute(ctxt, cl);
      return;
    }

    // Fetch nominated message queue
    ctxt.set_mq(ctxt.t().winner());
    ctxt.set_dir(model_);

    // Dispatch to appropriate message class
    const Message* msg = ctxt.msg();
    switch (msg->cls()) {
      case MessageClass::CohSrt: {
        process(ctxt, cl, static_cast<const CohSrtMsg*>(msg));
      } break;
      case MessageClass::CohCmd: {
        process_in_flight(ctxt, cl);
      } break;
      case MessageClass::LLCCmdRsp: {
        process_in_flight(ctxt, cl);
      } break;
      case MessageClass::CohSnpRsp: {
        process_in_flight(ctxt, cl);
      } break;
      default: {
        using cc::to_string;

        LogMessage lmsg("Invalid message class received: ");
        lmsg.append(to_string(ctxt.msg()->cls()));
        lmsg.set_level(Level::Error);
        log(lmsg);
      } break;
    }

    check_resources(ctxt, cl);

    LogMessage lm("Execute message: ");
    lm.append(ctxt.msg()->to_string());
    lm.set_level(Level::Debug);
    log(lm);

    execute(ctxt, cl);
  }

  void process(DirContext& ctxt, DirCommandList& cl, const CohSrtMsg* msg) {
    DirTState* tstate = lookup_state_or_fatal(msg->t(), false);
    if (tstate == nullptr) {
      // Transaction state is not already installed in the table
      // (otherwise error). Now, need to consider if the line is
      // already installed in the directory.
      tstate = lookup_state_by_addr(msg->addr());
      if (tstate != nullptr) {
        // Transaction already in progress for the current line,
        // command must be blocked until the completion of the current
        // command.
        ctxt.set_tstate(tstate);
        cl.push_back(DirOpcode::MqSetBlockedOnTransaction);
        // Advance
        cl.next_and_do_consume(false);
      } else if (Table<Transaction*, DirTState*>* tt = model_->tt(); !tt->full()) {
        // Transaction has not already been initiated, there is no
        // pending transaction to this line, AND, there are free
        // entries in the transaction table: the command can proceed
        // with execution.
        process_new_transaction(ctxt, cl, msg);
      } else {
        // Otherwise, a transaction is not in progress and the
        // transaction is full, therefore block Message Queue until
        // the transaction table becomes non-full.
        cl.push_back(DirOpcode::MqSetBlockedOnTable);
        // Advance
        cl.next_and_do_consume(false);
      }
    } else {
      // Transaction is already present in the transaction table;
      // attempting to reinstall the transaction.
      LogMessage msg("Transation is already present in table.");
      msg.set_level(Level::Fatal);
      log(msg);
    }
  }

  void process_new_transaction(DirContext& ctxt, DirCommandList& cl,
                               const CohSrtMsg* msg) const {
    // Otherwise, if there are free entries in the transaction table,
    // the transaction can proceed. Issue.  Search for the line in the
    // directory cache; if present, proceed, if not install new line,
    // if set is full, need to consider either evicting a line
    // presently in an evictable state, or blocking until one of the
    // transactions in the set completes.
    CacheModel<DirLineState*>* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const DirProtocol* protocol = model_->protocol();
    // Construct new transactions state object.
    DirTState* tstate = new DirTState(k());
    tstate->set_addr(msg->addr());
    tstate->set_origin(msg->origin());
    ctxt.set_owns_tstate(true);
    ctxt.set_tstate(tstate);
    const addr_t set_id = ah.set(msg->addr());
    auto set = cache->set(set_id);
    if (auto it = set.find(ah.tag(tstate->addr())); it == set.end()) {
      // Line is not present in the cache.
      CacheModel<DirLineState*>::Evictor evictor;
      if (auto p = evictor.nominate(set.begin(), set.end()); p.second) {
        // Eviction required.
        tstate->set_addr(ah.addr_from_set_tag(set_id, p.first->tag()));
        tstate->set_line(p.first->t());
        protocol->recall(ctxt, cl);
      } else {
        // Free line, or able to be evicted.
        tstate->set_line(protocol->construct_line());
        ctxt.set_owns_line(true);
        // Execute protocol update.
        protocol->apply(ctxt, cl);
      }
    } else {
      // Otherwise, lookup the line and assign to the new tstate.
      tstate->set_line(it->t());
      // Execute protocol update.
      protocol->apply(ctxt, cl);
    }
  }

  void process_in_flight(DirContext& ctxt, DirCommandList& cl) const {
    // Lookup transaction table or bail if not found.
    const DirProtocol* protocol = model_->protocol();
    DirTState* tstate = lookup_state_or_fatal(ctxt.msg()->t());
    if (const Message* msg = ctxt.mq()->peek();
        msg->cls() == MessageClass::CohCmd) {
      // Grab anciliary state on the Coherence Command message.
      const CohCmdMsg* coh = static_cast<const CohCmdMsg*>(msg);
      tstate->set_addr(coh->addr());
      tstate->set_opcode(coh->opcode());
    }
    ctxt.set_tstate(tstate);
    protocol->apply(ctxt, cl);
  }

  void check_resources(const DirContext& ctxt, DirCommandList& cl) const {
    const DirResources res(cl);

    bool has_resources = true;

    // Check table resources
    Table<Transaction*, DirTState*>* tt = model_->tt();
    if (!tt->has_at_least(res.tt_entry_n())) {
      cl.clear();
      // Blocks on table occupancy; wait until table becomes free.
      cl.push_back(DirOpcode::MqSetBlockedOnTable);
      has_resources = false;
    }

    // Check NOC credits
    NocPort* port = model_->dir_noc__port();
    if (CreditCounter* cc = port->ingress_cc(); cc->empty()) {
      // No NOC credits, block.
      cl.push_back(cb::build_blocked_on_event(ctxt.mq(), cc->credit_event()));
      has_resources = false;
    }

    // Check if the credit counter for agent 'agent' has at least 'n'
    // credits.
    auto check_credits = [&](const auto& ccntrs, const Agent* agent,
                             std::size_t n) -> bool {
      bool success = true;
      // If a credit exists for the destination agent, credit count
      // requirement must be attained, otherwise the requirement is ignored.
      if (auto it = ccntrs.find(agent); it != ccntrs.end()) {
        CreditCounter* cc = it->second;
        if (cc->i() < n) {
          cl.clear();
          cl.push_back(
              cb::build_blocked_on_event(ctxt.mq(), cc->credit_event()));
          success = false;
        }
      }
      return success;
    };

    auto check_credit_counter = [&](MessageClass cls, const auto& res) {
      if (!has_resources) return;

      auto& ccntrs_map = model_->ccntrs_map();
      if (auto it = ccntrs_map.find(cls); it != ccntrs_map.end()) {
        for (const auto& resp : res) {
          if (!check_credits(it->second, resp.first, resp.second)) {
            has_resources = false;
            break;
          }
        }
      }
    };

    // Check Coherence Snoop credits
    check_credit_counter(MessageClass::CohSnp, res.coh_snp_n());

    if (!has_resources) {
      cl.push_back(DirOpcode::WaitNextEpoch);
    }
  }

  void execute(DirContext& ctxt, const DirCommandList& cl) {
    try {
      DirCommandInterpreter interpreter(k());
      interpreter.set_dir(model_);
      interpreter.set_process(this);
      for (const DirCommand* cmd : cl) {
        LogMessage lm("Executing command: ");
        lm.append(cmd->to_string());
        lm.set_level(Level::Debug);
        log(lm);

        interpreter.execute(ctxt, cmd);
      }
    } catch (const std::runtime_error& ex) {
      LogMessage lm("Interpreter encountered an error: ");
      lm.append(ex.what());
      lm.set_level(Level::Fatal);
      log(lm);
    }
  }

  DirTState* lookup_state_or_fatal(Transaction* t,
                                   bool allow_fatal = true) const {
    Table<Transaction*, DirTState*>* tt = model_->tt();
    DirTState* st = nullptr;
    if (auto it = tt->find(t); it != tt->end()) {
      st = it->second;
    } else if (allow_fatal) {
      // Expect to find a entry in the transaction table. If an entry
      // is not present bail.
      LogMessage msg("Transaction not found in table.");
      msg.set_level(Level::Fatal);
      log(msg);
    }
    return st;
  }

  DirTState* lookup_state_by_addr(addr_t addr) const {
    for (auto p : *model_->tt()) {
      DirTState* entry = p.second;
      if (entry->addr() == addr) return entry;
    }
    return nullptr;
  }

  // Pointer to parent directory instance.
  DirAgent* model_ = nullptr;
};

// NOC Endpoint class to accept a message from the NOC and forward the
// message to the appropriate message queue. Nominally, this is done
// solely on the MessageClass associated with the message, but may
// also consider the originator agent.
//
class DirNocEndpoint : public NocEndpoint {
 public:
  //
  DirNocEndpoint(kernel::Kernel* k, const std::string& name)
      : NocEndpoint(k, name) {}

  //
  void register_endpoint(MessageClass cls, MessageQueue* mq) {
    // Register/install {cls} -> Message Queue mapping
    cls_eps_.insert(std::make_pair(cls, mq));
  }

  void register_endpoint(MessageClass cls, const Agent* origin,
                         MessageQueue* mq) {
    // Register/install {cls, origin} -> Message Queue mapping
    origin_eps_[cls][origin] = mq;
  }

  // Lookup canonical MessageClass to MessageQueue mapping.
  MessageQueue* lookup_endpoint(MessageClass cls, const Agent* origin) const {
    MessageQueue* mq = nullptr;
    if (const auto it = cls_eps_.find(cls); it != cls_eps_.end()) {
      mq = it->second;
    } else if (const auto jt = origin_eps_.find(cls); jt != origin_eps_.end()) {
      if (const auto kt = jt->second.find(origin); kt != jt->second.end()) {
        mq = kt->second;
      }
    }
    return mq;
  }

  MessageQueue* lookup_mq(const Message* msg) const override {
    MessageQueue* ret = lookup_endpoint(msg->cls(), msg->origin());
    if (ret == nullptr) {
      using cc::to_string;

      LogMessage lm("End point not register for class: ");
      lm.append(to_string(msg->cls()));
      lm.set_level(Level::Fatal);
      log(lm);
    }
    return ret;
  }

 private:
  // {Cls} -> Message Queue mapping
  std::map<MessageClass, MessageQueue*> cls_eps_;
  // {Cls, Origin} -> Message Queue mapping
  std::map<MessageClass, std::map<const Agent*, MessageQueue*> > origin_eps_;
};

DirAgent::DirAgent(kernel::Kernel* k, const DirAgentConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

DirAgent::~DirAgent() {
  for (const auto& p : cc_dir__cmd_q_) {
    delete p.second;
  }
  delete llc_dir__rsp_q_;
  delete cc_dir__snprsp_q_;
  delete arb_;
  delete cache_;
  delete noc_endpoint_;
  delete rdis_proc_;
  delete protocol_;
  delete tt_;
  // Destroy credit counters.
  for (const auto& cls_dest_cc : ccntrs_map_) {
    for (const auto& dest_cc : cls_dest_cc.second) {
      delete dest_cc.second;
    }
  }
}

void DirAgent::build() {
  // LLC -> DIR command queue
  llc_dir__rsp_q_ = new MessageQueue(k(), "llc_dir__rsp_q", 30);
  add_child_module(llc_dir__rsp_q_);
  // CC -> DIR snoop response queue.
  cc_dir__snprsp_q_ = new MessageQueue(k(), "cc_dir__snprsp_q", 30);
  add_child_module(cc_dir__snprsp_q_);
  // Construct arbiter
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Dir state cache
  CacheModelConfig cfg;
  cache_ = new CacheModel<DirLineState*>(cfg);
  // Construct transaction table.
  tt_ = new Table<Transaction*, DirTState*>(k(), "tt", 16);
  add_child_module(tt_);
  // Construct NOC ingress module.
  noc_endpoint_ = new DirNocEndpoint(k(), "noc_ep");
  noc_endpoint_->set_epoch(config_.epoch);
  add_child_module(noc_endpoint_);
  // Construct request dispatcher thread
  rdis_proc_ = new RdisProcess(k(), "rdis", this);
  rdis_proc_->set_epoch(config_.epoch);
  add_child_process(rdis_proc_);
  // Setup protocol
  protocol_ = config_.pbuilder->create_dir(k());
  add_child_module(protocol_);
}

// Register (add) a child command queue.
//
void DirAgent::register_command_queue(const Agent* origin) {
  const std::string name = "cmdq" + std::to_string(cc_dir__cmd_q_.size());
  MessageQueue* mq = new MessageQueue(k(), name, config_.cmd_queue_n);
  add_child_module(mq);
  cc_dir__cmd_q_.insert(std::make_pair(origin, mq));
}

// Register a verification monitor instance.
//
void DirAgent::register_monitor(Monitor* monitor) {
  if (monitor == nullptr) return;

  monitor->register_client(this);
}

// Elaborate Directory model
//
bool DirAgent::elab() {
  // Add message queues to arbiter
  for (auto& p : cc_dir__cmd_q_) {
    MessageQueue* mq = p.second;
    arb_->add_requester(mq);
  }
  arb_->add_requester(llc_dir__rsp_q_);
  arb_->add_requester(cc_dir__snprsp_q_);

  // Register message queue end-points to NOC ingress
  for (auto& p : cc_dir__cmd_q_) {
    const Agent* origin = p.first;
    MessageQueue* mq = p.second;

    noc_endpoint_->register_endpoint(MessageClass::CohSrt, origin, mq);
    noc_endpoint_->register_endpoint(MessageClass::CohCmd, origin, mq);
  }
  noc_endpoint_->register_endpoint(MessageClass::LLCCmdRsp, llc_dir__rsp_q_);
  noc_endpoint_->register_endpoint(MessageClass::CohSnpRsp, cc_dir__snprsp_q_);

  return false;
}

void DirAgent::set_dir_noc__port(NocPort* port) {
  dir_noc__port_ = port;
  add_child_module(dir_noc__port_);
}

//
//
void DirAgent::drc() {
  if (dir_noc__port_ == nullptr) {
    LogMessage lmsg("Dir to NOC message queue is unbound.", Level::Fatal);
    log(lmsg);
  }
}

void DirAgent::register_credit_counter(MessageClass cls, Agent* dest,
                                       std::size_t n) {
  const std::string name =
      join_path(flatten_path(dest->path()), to_string(cls));
  // Construct new credit counter.
  CreditCounter* cc = new CreditCounter(k(), name);
  cc->set_n(n);
  add_child_module(cc);
  // Install credit counter.
  ccntrs_map_[cls].insert(std::make_pair(dest, cc));
}

MessageQueue* DirAgent::endpoint() { return noc_endpoint_->ingress_mq(); }

CreditCounter* DirAgent::cc_by_cls_agent(MessageClass cls,
                                         const Agent* agent) const {
  CreditCounter* ret = nullptr;
  if (cls == MessageClass::Noc) {
    ret = dir_noc__port_->ingress_cc();
  } else if (auto i = ccntrs_map_.find(cls); i != ccntrs_map_.end()) {
    const auto& agent_cc_map = i->second;
    if (auto j = agent_cc_map.find(agent); j != agent_cc_map.end()) {
      ret = j->second;
    }
  }
  return ret;
}

MessageQueue* DirAgent::mq_by_msg_cls(MessageClass cls,
                                      const Agent* agent) const {
  return noc_endpoint_->lookup_endpoint(cls, agent);
}

}  // namespace cc
