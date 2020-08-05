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
#include "l1cache.h"

#include "utility.h"

namespace cc {

const char* to_string(L2CmdOpcode opcode) {
  switch (opcode) {
    case L2CmdOpcode::L1GetS:
      return "L1GetS";
    case L2CmdOpcode::L1GetE:
      return "L1GetE";
    case L2CmdOpcode::L1Put:
      return "L1Put";
    case L2CmdOpcode::Invalid:
      return "Invalid";
    default:
      return "Invalid";
  }
}

L2CmdMsg::L2CmdMsg() : Message(MessageClass::L2Cmd) {}

std::string L2CmdMsg::to_string() const {
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
  KVListRenderer r;
  r.add_field("cls", to_string(cls()));
  return r.to_string();
}

L2CacheContext::~L2CacheContext() {
  if (owns_line_) {
    // line_->release();
    delete line_;
  }

  if (owns_tstate()) {
    tstate_->release();
  }
}

const char* to_string(L2Opcode opcode) {
  switch (opcode) {
    case L2Opcode::StartTransaction:
      return "StartTransaction";
    case L2Opcode::EndTransaction:
      return "EndTransaction";
    case L2Opcode::MqSetBlockedOnTransaction:
      return "MqSetBlockedOnTransaction";
    case L2Opcode::MqSetBlockedOnTable:
      return "MqSetBlockedOnTable";
    case L2Opcode::MsgConsume:
      return "MsgConsume";
    case L2Opcode::InvokeCoherenceAction:
      return "InvokeCoherenceAction";
    case L2Opcode::SetL1LinesShared:
      return "SetL1LinesShared";
    case L2Opcode::SetL1LinesInvalid:
      return "SetL1LinesInvalid";
    case L2Opcode::WaitOnMsg:
      return "WaitOnMsg";
    case L2Opcode::WaitNextEpoch:
      return "WaitNextEpoch";
    default:
      return "Invalid";
  }
}

L2Command::~L2Command() {
  switch (opcode()) {
    case L2Opcode::InvokeCoherenceAction:
      oprands.coh.action->release();
      break;
    default:
      break;
  }
}

std::string L2Command::to_string() const {
  KVListRenderer r;
  r.add_field("opcode", cc::to_string(opcode()));
  switch (opcode()) {
    case L2Opcode::InvokeCoherenceAction: {
      r.add_field("action", oprands.coh.action->to_string());
    } break;
    default: {
    } break;
  }
  return r.to_string();
}

L2Command* L2CommandBuilder::from_opcode(L2Opcode opcode) {
  return new L2Command(opcode);
}

L2Command* L2CommandBuilder::from_action(CoherenceAction* action) {
  L2Command* cmd = new L2Command(L2Opcode::InvokeCoherenceAction);
  cmd->oprands.coh.action = action;
  return cmd;
}

L2CommandList::~L2CommandList() {
  for (L2Command* cmd : cmds_) {
    cmd->release();
  }
}

void L2CommandList::push_back(L2Command* cmd) { cmds_.push_back(cmd); }

L2TState::L2TState(kernel::Kernel* k) {
  transaction_start_ = new kernel::Event(k, "transaction_start");
  transaction_end_ = new kernel::Event(k, "transaction_end");
}

L2TState::~L2TState() {
  delete transaction_start_;
  delete transaction_end_;
}


class L2CommandInterpreter {
 public:
  L2CommandInterpreter() = default;

  void set_l2cache(L2CacheAgent* model) { model_ = model; }
  void set_process(AgentProcess* process) { process_ = process; }

  void execute(L2CacheContext& ctxt, const L2Command* cmd) {
    switch (cmd->opcode()) {
      case L2Opcode::StartTransaction: {
        execute_start_transaction(ctxt, cmd);
      } break;
      case L2Opcode::EndTransaction: {
        execute_end_transaction(ctxt, cmd);
      } break;
      case L2Opcode::MqSetBlockedOnTransaction: {
        execute_mq_set_blocked_on_transaction(ctxt, cmd);
      } break;
      case L2Opcode::MsgConsume: {
        execute_msg_consume(ctxt, cmd);
      } break;
      case L2Opcode::RemoveLine: {
        execute_remove_line(ctxt, cmd);
      } break;
      case L2Opcode::InvokeCoherenceAction: {
        execute_invoke_coherence_action(ctxt, cmd);
      } break;
      case L2Opcode::SetL1LinesShared: {
        execute_set_l1_lines_shared(ctxt, cmd);
      } break;
      case L2Opcode::SetL1LinesInvalid: {
        execute_set_l1_lines_invalid(ctxt, cmd);
      } break;
      case L2Opcode::WaitOnMsg: {
        execute_wait_on_msg(ctxt, cmd);
      } break;
      case L2Opcode::WaitNextEpoch: {
        execute_wait_next_epoch_or_wait(ctxt, cmd);
      } break;
      default: {
      } break;
    }
  }
 private:

  void execute_start_transaction(L2CacheContext& ctxt, const L2Command* cmd) {
    L2TState* tstate = ctxt.tstate();
    L2TTable* tt = model_->tt();
    tt->install(ctxt.msg()->t(), tstate);
    ctxt.set_owns_tstate(false);

    if (ctxt.owns_line()) {
      // Install line in the dache.
      L2CacheModel* cache = model_->cache();
      const CacheAddressHelper ah = cache->ah();
      L2CacheModelSet set = cache->set(ah.set(tstate->addr()));
      if (L2CacheModelLineIt it = set.find(ah.tag(tstate->addr())); it == set.end()) {
        L2CacheModel::Evictor evictor;
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

  void execute_end_transaction(L2CacheContext& ctxt, const L2Command* cmd) {
    // Notify transaction event event; unblocks message queues
    // awaiting completion of current transaction.
    ctxt.tstate()->transaction_end()->notify();
    // Delete transaction from transaction table.
    L2TTable* tt = model_->tt();
    tt->remove(ctxt.msg()->t());
    ctxt.tstate()->release();
  }
  
  void execute_mq_set_blocked_on_transaction(
      L2CacheContext& ctxt, const L2Command* cmd) const {
    // Set the blocked status of the current Message Queue.
    ctxt.mq()->set_blocked_until(ctxt.tstate()->transaction_end());
  }

  void execute_mq_set_blocked_on_table(
      L2CacheContext& ctxt, const L2Command* cmd) const {
    // Set the blocked status of the current Message Queue.
    ctxt.mq()->set_blocked_until(model_->tt()->non_full_event());
  }

  void execute_msg_consume(L2CacheContext& ctxt, const L2Command* cmd) const {
    // Dequeue and release the head message of the currently
    // addressed Message Queue.
    const Message* msg = ctxt.mq()->dequeue();
    msg->release();
    ctxt.t().advance();
  }  

  void execute_remove_line(L2CacheContext& ctxt, const L2Command* cmd) const {
    L2CacheModel* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    L2CacheModelSet set = cache->set(ah.set(ctxt.addr()));
    if (auto it = set.find(ah.tag(ctxt.addr()));  it != set.end()) {
      set.evict(it);
    } else {
      throw std::runtime_error("Cannot remove line, line is not present.");
    }
  }
  
  void execute_invoke_coherence_action(L2CacheContext& ctxt,
                                       const L2Command* cmd) const {
    CoherenceAction* action = cmd->action();
    action->execute();
  }

  void execute_set_l1_lines_shared(L2CacheContext& ctxt,
                                   const L2Command* cmd) const {
    const std::vector<L1CacheAgent*>& agents = cmd->agents();
    for (L1CacheAgent* l1cache : ctxt.l2cache()->l1cs_) {
      if (std::find(agents.begin(), agents.end(), l1cache) != agents.end()) {
        // Agent in keep-out set.
        continue;
      }
      l1cache->set_cache_line_shared_or_invalid(ctxt.addr());
    }
  }
  

  void execute_set_l1_lines_invalid(L2CacheContext& ctxt,
                                   const L2Command* cmd) const {
    const std::vector<L1CacheAgent*>& agents = cmd->agents();
    for (L1CacheAgent* l1cache : ctxt.l2cache()->l1cs_) {
      // Search agent 'keep-out' list such that we do not invalidate
      // agents that we wish to retain, typically L1 which is about to
      // receive exclusive ownership of the line.
      if (std::find(agents.begin(), agents.end(), l1cache) != agents.end()) {
        continue;
      }
      l1cache->set_cache_line_shared_or_invalid(ctxt.addr(), false);
    }
  }
  
  void execute_wait_on_msg(L2CacheContext& ctxt, const L2Command* cmd) const {
    // Set wait state of current process; await the arrival of a
    // new message.
    MQArb* arb = model_->arb();
    process_->wait_on(arb->request_arrival_event());
  }
  
  void execute_wait_next_epoch_or_wait(L2CacheContext& ctxt,
                                       const L2Command* cmd) const {
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
  
  //
  AgentProcess* process_ = nullptr;
  //
  L2CacheAgent* model_ = nullptr;
};

//
//
class L2CacheAgent::MainProcess : public AgentProcess {
  using cb = L2CommandBuilder;

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L2CacheAgent* model)
      : AgentProcess(k, name), model_(model) {}

  // Initialization:
  void init() override {
    L2CacheContext c;
    L2CommandList cl;
    cl.push_back(cb::from_opcode(L2Opcode::WaitOnMsg));
    execute(c, cl);
  }

  // Evaluation:
  void eval() override {
    MQArb* arb = model_->arb();

    // Construct and initialize current processing context.
    L2CommandList cl;
    L2CacheContext ctxt;
    ctxt.set_t(arb->tournament());
    ctxt.set_l2cache(model_);

    // Check if requests are present, if not block until a new message
    // arrives at the arbiter. Process should ideally not wake in the
    // absence of requesters.
    if (!ctxt.t().has_requester()) {
      cl.push_back(cb::from_opcode(L2Opcode::WaitOnMsg));
      execute(ctxt, cl);
      return;
    }

    // Fetch nominated message queue
    ctxt.set_mq(ctxt.t().winner());

    // Dispatch to appropriate message class
    const MessageClass cls = ctxt.msg()->cls();
    switch (cls) {
      case MessageClass::L2Cmd: {
        process_l2cmd(ctxt, cl);
      } break;
      case MessageClass::AceCmdRsp: {
        process_acecmdrsp(ctxt, cl);
      } break;
      case MessageClass::AceSnoop: {
        process_acesnoop(ctxt, cl);
      } break;
      default: {
        LogMessage lmsg("Invalid message class received: ");
        lmsg.append(cc::to_string(ctxt.msg()->cls()));
        lmsg.level(Level::Fatal);
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

  void set_cache_line_modified(addr_t addr) {
    L2CommandList cl;
    L2CacheContext ctxt;
    ctxt.set_l2cache(model_);
    const L2CacheAgentProtocol* protocol = model_->protocol();
    L2CacheModel* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();

    L2CacheModelSet set = cache->set(ah.set(addr));
    L2CacheModelLineIt it = set.find(ah.tag(addr));
    if (it != set.end()) {
      ctxt.set_line(it->t());
      protocol->set_modified_status(ctxt, cl);
    } else {
      LogMessage msg("L1 attempts to set modified state of cache line but "
                     "cache line is not resident in L2.");
      msg.level(Level::Fatal);
      log(msg);
    }
    execute(ctxt, cl);
  }

 private:
  void process_l2cmd(L2CacheContext& ctxt, L2CommandList& cl) const {
    const L2CmdMsg* cmd = static_cast<const L2CmdMsg*>(ctxt.msg());
    L2CacheModel* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const L2CacheAgentProtocol* protocol = model_->protocol();

    // Command starts new transaction, therefore construct new
    // transaction table entry; unclear at this point whether
    // transaction will start, therefore context owns tstate upon
    // destruction.
    L2TState* tstate = new L2TState(k());
    ctxt.set_tstate(tstate);
    ctxt.set_owns_tstate(true);

    // Cache line state.
    L2LineState* line = nullptr;

    L2CacheModelSet set = cache->set(ah.set(cmd->addr()));
    L2CacheModelLineIt it = set.find(ah.tag(cmd->addr()));
    const bool has_cache_line = (it != set.end());
    const L2CmdOpcode opcode = cmd->opcode();
    switch (opcode) {
      case L2CmdOpcode::L1GetS:
      case L2CmdOpcode::L1GetE: {
        if (!has_cache_line) {
          // Line for current address has not been found in the set,
          // therefore a new line must either be installed or, if the set
          // is currently full, another line must be nominated and
          // evicted.
          L2CacheModel::Evictor evictor;
          if (const std::pair<L2CacheModelLineIt, bool> p =
              evictor.nominate(set.begin(), set.end());
              p.second) {
            // Eviction required before command can complete.
            // TODO
          } else {
            // Eviction not required for the command to complete.
            line = protocol->construct_line();
            ctxt.set_line(line);
            ctxt.set_owns_line(true);
            tstate->set_line(line);
            protocol->apply(ctxt, cl);
          }
        } else {
          // Line is present in the cache, apply state update.
          line = it->t();
          ctxt.set_line(line);
          tstate->set_line(line);
          protocol->apply(ctxt, cl);
        }
      } break;
      case L2CmdOpcode::L1Put: {
        if (has_cache_line) {
          line = it->t();
          ctxt.set_line(line);
          tstate->set_line(line);
          protocol->apply(ctxt, cl);
        } else {
          LogMessage msg("L1 requests eviction of line which is not present "
                         "in L2; simulation assumes inclusive cache model.");
          msg.level(Level::Fatal);
          log(msg);
        }
      } break;
      default: {
        // Unknown opcode.
      } break;
    }
  }

  void process_acecmdrsp(L2CacheContext& ctxt, L2CommandList& cl) const {
    Transaction* t = ctxt.msg()->t();
    L2TTable* tt = model_->tt();
    if (auto it = tt->find(t); it != tt->end()) {
      const L2CacheAgentProtocol* protocol = model_->protocol();
      ctxt.set_tstate(it->second);
      ctxt.set_line(ctxt.tstate()->line());
      protocol->apply(ctxt, cl);
    } else {
      LogMessage lm("Cannot find transaction table entry for ");
      lm.append(to_string(t));
      lm.level(Level::Fatal);
      log(lm);
    }
  }

  void process_acesnoop(L2CacheContext& ctxt, L2CommandList& cl) const {
    const bool opt_allow_silent_evictions = false;
    // Lookup cache line of interest
    L2CacheModel* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const AceSnpMsg* msg = static_cast<const AceSnpMsg*>(ctxt.msg());
    ctxt.set_addr(msg->addr());
    L2CacheModelSet set = cache->set(ah.set(msg->addr()));
    L2CacheModelLineIt it = set.find(ah.tag(msg->addr()));
    if (it != set.end()) {
      // Found line in cache; set constext
      ctxt.set_line(it->t());
    } else if (opt_allow_silent_evictions) {
      //
      ctxt.set_silently_evicted(true);
    } else {
      LogMessage msg(
          "Expect line to be installed in cache for snoops "
          "when silent evictions have not be enabled.");
      msg.level(Level::Fatal);
      log(msg);
    }
    ctxt.set_owns_line(false);
    const L2CacheAgentProtocol* protocol = model_->protocol();
    protocol->apply(ctxt, cl);
  }

  bool can_execute(const L2CommandList& cl) const {
    return true;
  }

  void execute(L2CacheContext& ctxt, const L2CommandList& cl) {
    try {
      L2CommandInterpreter interpreter;
      interpreter.set_l2cache(model_);
      interpreter.set_process(this);
      for (const L2Command* cmd : cl) {
        LogMessage lm("Executing command: ");
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

  // Pointer to parent L2.
  L2CacheAgent* model_ = nullptr;
};

L2CacheAgent::L2CacheAgent(kernel::Kernel* k, const L2CacheAgentConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

L2CacheAgent::~L2CacheAgent() {
  delete arb_;
  delete main_;
  delete cache_;
  delete protocol_;
  delete cc_l2__cmd_q_;
  delete cc_l2__rsp_q_;
  for (MessageQueue* mq : l1_l2__cmd_qs_) {
    delete mq;
  }
  delete l2_cc__cmd_q_;
  for (auto it : l2_l1__rsp_qs_) {
    delete it.second;
  }
  delete l2_cc__snprsp_q_;
  delete tt_;
}

MessageQueueProxy* L2CacheAgent::l2_l1__rsp_q(L1CacheAgent* l1cache) const {
  MessageQueueProxy* proxy = nullptr;
  if (auto it = l2_l1__rsp_qs_.find(l1cache); it != l2_l1__rsp_qs_.end()) {
    proxy = it->second;
  } else {
    LogMessage msg("Cannot find l2_lq__rsp_qs proxy instance for l1cache.");
    msg.level(Level::Fatal);
    log(msg);
  }
  return proxy;
}

void L2CacheAgent::add_l1c(L1CacheAgent* l1c) {
  using std::to_string;
  // Assert build phase.
  
  // Construct associated message queues
  std::string name = "l1_l2__cmd_q";
  name += to_string(l1cs_.size());
  MessageQueue* l1c_cmdq = new MessageQueue(k(), name, 3);
  l1_l2__cmd_qs_.push_back(l1c_cmdq);
  add_child_module(l1c_cmdq);
  // Add L1 cache
  l1cs_.push_back(l1c);
}

void L2CacheAgent::build() {
  // CC -> L2 command queue.
  cc_l2__cmd_q_ = new MessageQueue(k(), "cc_l2__cmd_q", 16);
  add_child_module(cc_l2__cmd_q_);
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
  cache_ = new L2CacheModel(config_.cconfig);
  // Setup protocol
  protocol_ = config_.pbuilder->create_l2(k());
  add_child_module(protocol_);
}

void L2CacheAgent::elab() {
  // Add command queues to arbiter
  arb_->add_requester(cc_l2__cmd_q_);
  arb_->add_requester(cc_l2__rsp_q_);
  for (MessageQueue* msgq : l1_l2__cmd_qs_) {
    arb_->add_requester(msgq);
  }
}

void L2CacheAgent::set_l2_cc__cmd_q(MessageQueueProxy* mq) {
  l2_cc__cmd_q_ = mq;
  add_child_module(l2_cc__cmd_q_);
}

void L2CacheAgent::set_l2_l1__rsp_q(
    L1CacheAgent* l1cache, MessageQueueProxy* mq) {
  l2_l1__rsp_qs_[l1cache] = mq;
  add_child_module(l2_l1__rsp_qs_[l1cache]);
}

void L2CacheAgent::set_l2_cc__snprsp_q(MessageQueueProxy* mq) {
  l2_cc__snprsp_q_ = mq;
  add_child_module(l2_cc__snprsp_q_);
}

void L2CacheAgent::drc() {
  if (protocol_ == nullptr) {
    LogMessage msg("Protocol has not been bound.", Level::Fatal);
    log(msg);
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

void L2CacheAgent::set_cache_line_modified(addr_t addr) {
  main_->set_cache_line_modified(addr);
}


}  // namespace cc
