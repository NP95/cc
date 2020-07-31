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
}

const char* to_string(L2Opcode opcode) {
  switch (opcode) {
    case L2Opcode::TableInstall:
      return "TableInstall";
    case L2Opcode::TableGetCurrentState:
      return "TableGetCurrentState";
    case L2Opcode::TableMqAddToBlockedList:
      return "TableMqAddToBlockedList";
    case L2Opcode::MqSetBlocked:
      return "MqSetBlocked";
    case L2Opcode::MsgL1CmdExtractAddr:
      return "MsgL1CmdExtractAddr";
    case L2Opcode::MsgConsume:
      return "MsgConsume";
    case L2Opcode::InstallLine:
      return "InstallLine";
    case L2Opcode::InvokeCoherenceAction:
      return "InvokeCoherenceAction";
    case L2Opcode::SetL1LinesShared:
      return "SetL1LinesShared";
    case L2Opcode::SetL1LinesInvalid:
      return "SetL1LinesInvalid";
    case L2Opcode::WaitOnMsg:
      return "WaitOnMsg";
    case L2Opcode::WaitNextEpochOrWait:
      return "WaitNextEpochOrWait";
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

class L2CommandInterpreter {

  struct State {
    // Iterator into current transaction table entry.
    L2TTable::iterator table_it;
    // Address of current command.
    addr_t addr;
  };
  
 public:
  L2CommandInterpreter() = default;

  void set_l2cache(L2CacheModel* model) { model_ = model; }
  void set_process(AgentProcess* process) { process_ = process; }

  void execute(L2CacheContext& ctxt, const L2Command* cmd) {
    switch (cmd->opcode()) {
      case L2Opcode::TableInstall: {
        execute_table_install(ctxt, cmd);
      } break;
      case L2Opcode::TableGetCurrentState: {
        execute_table_get_current_state(ctxt, cmd);
      } break;
      case L2Opcode::TableMqAddToBlockedList: {
        execute_table_mq_add_to_blocked_list(ctxt, cmd);
      } break;
      case L2Opcode::MqSetBlocked: {
        execute_mq_set_blocked(ctxt, cmd);
      } break;
      case L2Opcode::MsgL1CmdExtractAddr: {
        execute_msg_l1_cmd_extract_addr(ctxt, cmd);
      } break;
      case L2Opcode::MsgConsume: {
        execute_msg_consume(ctxt, cmd);
      } break;
      case L2Opcode::InstallLine: {
        execute_install_line(ctxt, cmd);
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
      case L2Opcode::WaitNextEpochOrWait: {
        execute_wait_next_epoch_or_wait(ctxt, cmd);
      } break;
      default: {
      } break;
    }
  }
 private:

  void execute_table_install(L2CacheContext& ctxt, const L2Command* cmd) const {
    L2TTable* tt = model_->tt();
    L2TState* st = new L2TState();
    st->set_line(ctxt.line());
    tt->install(ctxt.msg()->t(), st);
  }
  
  void execute_table_get_current_state(L2CacheContext& ctxt, const L2Command* cmd) {
    // Lookup the Transaction Table for the current transaction
    // and set the table pointer for subsequent operations.
    L2TTable* tt = model_->tt();
    state_.table_it = tt->find(ctxt.msg()->t());
    if (state_.table_it == tt->end()) {
      throw std::runtime_error(
          "Transaction not present in the transaction table");
    }
  }
  
  void execute_table_mq_add_to_blocked_list(L2CacheContext& ctxt, const L2Command* cmd) const {
    // Add the currently address Message Queue to the set of
    // queues blocked on the current transaction.
    L2TState* st = state_.table_it->second;
    st->add_blocked_mq(ctxt.mq());
  }
  
  void execute_mq_set_blocked(L2CacheContext& ctxt, const L2Command* cmd) const {
    // Set the blocked status of the current Message Queue.
    ctxt.mq()->set_blocked(true);
  }
  
  void execute_msg_l1_cmd_extract_addr(L2CacheContext& ctxt, const L2Command* cmd) {
    // Extract address field from command message.
    state_.addr = static_cast<const L2CmdMsg*>(ctxt.msg())->addr();
  }

  void execute_msg_consume(L2CacheContext& ctxt, const L2Command* cmd) const {
    // Dequeue and release the head message of the currently
    // addressed Message Queue.
    const Message* msg = ctxt.mq()->dequeue();
    msg->release();
    ctxt.t().advance();
  }  
  
  void execute_install_line(L2CacheContext& ctxt, const L2Command* cmd) const {
    // Install line within the current context into the cache at
    // an appropriate location. Expects that any prior evictions
    // to thg destination set have already taken place.
    L2Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    L2CacheSet set = cache->set(ah.set(state_.addr));
    L2Cache::Evictor evictor;
    if (const std::pair<L2CacheLineIt, bool> p =
        evictor.nominate(set.begin(), set.end());
        !p.second) {
      set.install(p.first, ah.tag(state_.addr), ctxt.line());
      ctxt.set_owns_line(false);
    } else {
      throw std::runtime_error("Cannot install line as set is full.");
    }
  }

  void execute_remove_line(L2CacheContext& ctxt, const L2Command* cmd) const {
    L2Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    L2CacheSet set = cache->set(ah.set(ctxt.addr()));
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
    for (L1CacheModel* l1cache : ctxt.l2cache()->l1cs_) {
      l1cache->set_cache_line_shared_or_invalid(ctxt.addr());
    }
  }
  

  void execute_set_l1_lines_invalid(L2CacheContext& ctxt,
                                   const L2Command* cmd) const {
    for (L1CacheModel* l1cache : ctxt.l2cache()->l1cs_) {
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
  State state_;
  //
  AgentProcess* process_ = nullptr;
  //
  L2CacheModel* model_ = nullptr;
};

//
//
class L2CacheModel::MainProcess : public AgentProcess {
  using cb = L2CommandBuilder;

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L2CacheModel* model)
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
    switch (ctxt.msg()->cls()) {
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
    const L2CacheModelProtocol* protocol = model_->protocol();
    L2Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();

    L2CacheSet set = cache->set(ah.set(addr));
    L2CacheLineIt it = set.find(ah.tag(addr));
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
        // TODO
      } else {
        // Eviction not required for the command to complete.
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

  void process_acecmdrsp(L2CacheContext& ctxt, L2CommandList& cl) const {
    Transaction* t = ctxt.msg()->t();
    L2TTable* tt = model_->tt();
    if (auto it = tt->find(t); it != tt->end()) {
      const L2CacheModelProtocol* protocol = model_->protocol();
      const L2TState* st = it->second;
      ctxt.set_line(st->line());
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
    L2Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const AceSnpMsg* msg = static_cast<const AceSnpMsg*>(ctxt.msg());
    ctxt.set_addr(msg->addr());
    L2CacheSet set = cache->set(ah.set(msg->addr()));
    L2CacheLineIt it = set.find(ah.tag(msg->addr()));
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
    const L2CacheModelProtocol* protocol = model_->protocol();
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
#if 0
        LogMessage lm("Executing command: ");
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

  // Pointer to parent L2.
  L2CacheModel* model_ = nullptr;
};

L2CacheModel::L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

L2CacheModel::~L2CacheModel() {
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
  for (MessageQueueProxy* mq : l2_l1__rsp_qs_) {
    delete mq;
  }
  delete l2_cc__snprsp_q_;
  delete tt_;
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
  cache_ = new L2Cache(config_.cconfig);
  // Setup protocol
  protocol_ = config_.pbuilder->create_l2(k());
  add_child_module(protocol_);
}

void L2CacheModel::elab() {
  // Add command queues to arbiter
  arb_->add_requester(cc_l2__cmd_q_);
  arb_->add_requester(cc_l2__rsp_q_);
  for (MessageQueue* msgq : l1_l2__cmd_qs_) {
    arb_->add_requester(msgq);
  }
}

void L2CacheModel::set_l1cache_n(std::size_t n) { l2_l1__rsp_qs_.resize(n); }

void L2CacheModel::set_l2_cc__cmd_q(MessageQueueProxy* mq) {
  l2_cc__cmd_q_ = mq;
  add_child_module(l2_cc__cmd_q_);
}

void L2CacheModel::set_l2_l1__rsp_q(std::size_t n, MessageQueueProxy* mq) {
  l2_l1__rsp_qs_[n] = mq;
  add_child_module(l2_l1__rsp_qs_[n]);
}

void L2CacheModel::set_l2_cc__snprsp_q(MessageQueueProxy* mq) {
  l2_cc__snprsp_q_ = mq;
  add_child_module(l2_cc__snprsp_q_);
}

void L2CacheModel::drc() {
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

void L2CacheModel::set_cache_line_modified(addr_t addr) {
  main_->set_cache_line_modified(addr);
}


}  // namespace cc
