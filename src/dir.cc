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
      oprands.coh.action->release();
      break;
    default:
      break;
  }
}

std::string DirCommand::to_string() const {
  KVListRenderer r;
  r.add_field("opcode", cc::to_string(opcode()));
  switch (opcode()) {
    case DirOpcode::InvokeCoherenceAction: {
      r.add_field("action", oprands.coh.action->to_string());
    } break;
    default: {
    } break;
  }
  return r.to_string();
}

DirCommand* DirCommandBuilder::from_opcode(DirOpcode opcode) {
  return new DirCommand(opcode);
}

DirCommand* DirCommandBuilder::from_action(CoherenceAction* action) {
  DirCommand* cmd = new DirCommand(DirOpcode::InvokeCoherenceAction);
  cmd->oprands.coh.action = action;
  return cmd;
}

void DirTState::release() { delete this; }

DirContext::~DirContext() {
  if (owns_line()) {
    tstate_->line()->release();
  }
  if (owns_tstate()) {
    tstate_->release();
  }
}

DirCommandList::~DirCommandList() {
  for (DirCommand* cmd : cmds_) {
    cmd->release();
  }
}

void DirCommandList::push_back(DirCommand* cmd) { cmds_.push_back(cmd); }

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

  void set_dir(DirModel* model) { model_ = model; }
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
    DirTTable* tt = model_->tt();
    tt->install(ctxt.msg()->t(), tstate);
    ctxt.set_owns_tstate(false);

    if (ctxt.owns_line()) {
      // Install line in the dache.
      DirCacheModel* cache = model_->cache();
      const CacheAddressHelper ah = cache->ah();
      DirCacheModelSet set = cache->set(ah.set(tstate->addr()));
      if (DirCacheModelLineIt it = set.find(ah.tag(tstate->addr()));
          it == set.end()) {
        DirCacheModel::Evictor evictor;
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
    DirTTable* tt = model_->tt();
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
    DirCacheModel* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    DirCacheModelSet set = cache->set(ah.set(ctxt.addr()));
    if (auto it = set.find(ah.tag(ctxt.addr())); it != set.end()) {
      set.evict(it);
    } else {
      throw std::runtime_error("Cannot remove line, line is not present.");
    }
  }

  void execute_invoke_coherence_action(DirContext& ctxt,
                                       const DirCommand* cmd) {
    CoherenceAction* action = cmd->action();
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

  void execute_wait_on_msg(DirContext& ctxt, const DirCommand* cmd) const {
    // Set wait state of current process; await the arrival of a
    // new message.
    MQArb* arb = model_->arb();
    process_->wait_on(arb->request_arrival_event());
  }

  void execute_wait_next_epoch(DirContext& ctxt, const DirCommand* cmd) const {
    process_->wait_for(kernel::Time{10, 0});
  }

  //
  kernel::Kernel* k_ = nullptr;
  //
  AgentProcess* process_ = nullptr;
  //
  DirModel* model_ = nullptr;
};

//
//
class DirModel::RdisProcess : public AgentProcess {
  using cb = DirCommandBuilder;

 public:
  RdisProcess(kernel::Kernel* k, const std::string& name, DirModel* model)
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
        cl.push_back(cb::from_opcode(DirOpcode::MqSetBlockedOnTransaction));
        // Advance
        cl.next_and_do_consume(false);
        // cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
      } else if (DirTTable* tt = model_->tt(); !tt->full()) {
        // Transaction has not already been initiated, there is no
        // pending transaction to this line, AND, there are free
        // entries in the transaction table: the command can proceed
        // with execution.
        process_new_transaction(ctxt, cl, msg);
      } else {
        // Otherwise, a transaction is not in progress and the
        // transaction is full, therefore block Message Queue until
        // the transaction table becomes non-full.
        cl.push_back(cb::from_opcode(DirOpcode::MqSetBlockedOnTable));
        // Advance
        cl.next_and_do_consume(false);
        // cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
      }
    } else {
      // Transaction is already present in the transaction table;
      // attempting to reinstall the transaction.
      LogMessage msg("Transation is already present in table.");
      msg.level(Level::Fatal);
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
    DirCacheModel* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const DirProtocol* protocol = model_->protocol();
    // Construct new transactions state object.
    DirTState* tstate = new DirTState(k());
    tstate->set_addr(msg->addr());
    tstate->set_origin(msg->origin());
    DirCacheModelSet set = cache->set(ah.set(tstate->addr()));
    if (DirCacheModelLineIt it = set.find(ah.tag(tstate->addr()));
        it == set.end()) {
      // Line is not present in the cache.
      DirCacheModel::Evictor evictor;
      if (auto p = evictor.nominate(set.begin(), set.end()); p.second) {
        // Eviction required.
        // TODO:
      } else {
        // Free line, or able to be evicted.
        tstate->set_line(protocol->construct_line());
        ctxt.set_owns_line(true);
      }
    } else {
      // Otherwise, lookup the line and assign to the new tstate.
      tstate->set_line(it->t());
    }
    ctxt.set_tstate(tstate);
    ctxt.set_owns_tstate(true);

    // Execute protocol update.
    protocol->apply(ctxt, cl);
  }

  void process_in_flight(DirContext& ctxt, DirCommandList& cl) const {
    // Lookup transaction table or bail if not found.
    const DirProtocol* protocol = model_->protocol();
    DirTState* tstate = lookup_state_or_fatal(ctxt.msg()->t());
    const Message* msg = ctxt.mq()->peek();
    if (msg->cls() == MessageClass::CohCmd) {
      // Grab anciliary state on the Coherence Command message.
      const CohCmdMsg* coh = static_cast<const CohCmdMsg*>(msg);
      tstate->set_addr(coh->addr());
      tstate->set_opcode(coh->opcode());
    }
    ctxt.set_tstate(tstate);
    protocol->apply(ctxt, cl);
  }

  bool can_execute(const DirCommandList& cl) const { return true; }

  void execute(DirContext& ctxt, const DirCommandList& cl) {
    try {
      DirCommandInterpreter interpreter(k());
      interpreter.set_dir(model_);
      interpreter.set_process(this);
      for (const DirCommand* cmd : cl) {
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

  DirTState* lookup_state_or_fatal(Transaction* t,
                                   bool allow_fatal = true) const {
    DirTTable* tt = model_->tt();
    DirTState* st = nullptr;
    if (auto it = tt->find(t); it != tt->end()) {
      st = it->second;
    } else if (allow_fatal) {
      // Expect to find a entry in the transaction table. If an entry
      // is not present bail.
      LogMessage msg("Transaction not found in table.");
      msg.level(Level::Fatal);
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
  DirModel* model_ = nullptr;
};

//
//
class DirNocEndpoint : public NocEndpoint {
 public:
  //
  DirNocEndpoint(kernel::Kernel* k, const std::string& name)
      : NocEndpoint(k, name) {}
  //
  void register_endpoint(MessageClass cls, MessageQueueProxy* p) {
    endpoints_.insert(std::make_pair(cls, p));
  }

  // Lookup canonical MessageClass to MessageQueue mapping.
  MessageQueueProxy* lookup_endpoint(MessageClass cls) const {
    MessageQueueProxy* mq = nullptr;
    if (auto it = endpoints_.find(cls); it != endpoints_.end()) {
      mq = it->second;
    }
    return mq;
  }

  MessageQueueProxy* lookup_mq(const Message* msg) const override {
    if (auto it = endpoints_.find(msg->cls()); it != endpoints_.end()) {
      return it->second;
    } else {
      LogMessage lm("End point not register for class: ");
      lm.append(cc::to_string(msg->cls()));
      lm.level(Level::Fatal);
      log(lm);
    }
    return nullptr;
  }

 private:
  //
  std::map<MessageClass, MessageQueueProxy*> endpoints_;
};

DirModel::DirModel(kernel::Kernel* k, const DirModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

DirModel::~DirModel() {
  delete cpu_dir__cmd_q_;
  delete llc_dir__rsp_q_;
  delete cc_dir__snprsp_q_;
  delete arb_;
  delete cache_;
  delete noc_endpoint_;
  delete rdis_proc_;
  delete protocol_;
  for (MessageQueueProxy* p : endpoints_) {
    delete p;
  }
  delete dir_noc__msg_q_;
  delete tt_;
}

void DirModel::build() {
  // CPU -> DIR command queue
  cpu_dir__cmd_q_ = new MessageQueue(k(), "cpu_dir__cmd_q", 30);
  add_child_module(cpu_dir__cmd_q_);
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
  cache_ = new DirCacheModel(cfg);
  // Construct transaction table.
  tt_ = new DirTTable(k(), "tt", 16);
  add_child_module(tt_);
  // Construct NOC ingress module.
  noc_endpoint_ = new DirNocEndpoint(k(), "noc_ep");
  add_child_module(noc_endpoint_);
  // Construct request dispatcher thread
  rdis_proc_ = new RdisProcess(k(), "rdis", this);
  add_child_process(rdis_proc_);
  // Setup protocol
  protocol_ = config_.pbuilder->create_dir(k());
  add_child_module(protocol_);
}

//
//
void DirModel::elab() {
  // Register message queue end-points
  arb_->add_requester(cpu_dir__cmd_q_);
  arb_->add_requester(llc_dir__rsp_q_);
  arb_->add_requester(cc_dir__snprsp_q_);

  MessageQueueProxy* p = nullptr;

  p = cpu_dir__cmd_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::CohSrt, p);
  noc_endpoint_->register_endpoint(MessageClass::CohCmd, p);
  endpoints_.push_back(p);

  p = llc_dir__rsp_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::LLCCmdRsp, p);
  endpoints_.push_back(p);

  p = cc_dir__snprsp_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::CohSnpRsp, p);
  endpoints_.push_back(p);
}

void DirModel::set_dir_noc__msg_q(MessageQueueProxy* mq) {
  dir_noc__msg_q_ = mq;
  add_child_module(dir_noc__msg_q_);
}

//
//
void DirModel::drc() {
  if (dir_noc__msg_q_ == nullptr) {
    LogMessage lmsg("Dir to NOC message queue is unbound.", Level::Fatal);
    log(lmsg);
  }
}

MessageQueue* DirModel::endpoint() { return noc_endpoint_->ingress_mq(); }

MessageQueueProxy* DirModel::mq_by_msg_cls(MessageClass cls) const {
  return noc_endpoint_->lookup_endpoint(cls);
}

}  // namespace cc
