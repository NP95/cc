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
#include "noc.h"
#include "primitives.h"
#include "protocol.h"
#include "msg.h"
#include "utility.h"

namespace cc {

const char* to_string(DirOpcode opcode) {
  switch (opcode) {
#define __declare_to_string(__name)              \
    case DirOpcode::__name:                      \
      return #__name;
    DIROPCODE_LIST(__declare_to_string)
#undef __declare_to_string
    default: return "Invalid";
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
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    r.add_field("opcode", cc::to_string(opcode()));
    switch (opcode()) {
      case DirOpcode::InvokeCoherenceAction: {
        r.add_field("action", oprands.coh.action->to_string());
      } break;
      default: {
      } break;
    }
  }
  return ss.str();
}

DirCommand* DirCommandBuilder::from_opcode(DirOpcode opcode) {
  return new DirCommand(opcode);
}

DirCommand* DirCommandBuilder::from_action(CoherenceAction* action) {
  DirCommand* cmd = new DirCommand(DirOpcode::InvokeCoherenceAction);
  cmd->oprands.coh.action = action;
  return cmd;
}

DirContext::~DirContext() {
  if (owns_line_) {
    line_->release();
  }
}

DirCommandList::~DirCommandList() {
  for (DirCommand* cmd : cmds_) {
    cmd->release();
  }
}

void DirCommandList::push_back(DirCommand* cmd) { cmds_.push_back(cmd); }

//
//
class DirModel::NocIngressProcess : public AgentProcess {
 public:
  NocIngressProcess(kernel::Kernel* k, const std::string& name, DirModel* model)
      : AgentProcess(k, name), model_(model) {}

  // Initialization
  void init() override {
    MessageQueue* mq = model_->noc_dir__msg_q();
    wait_on(mq->non_empty_event());
  }

  // Evaluation
  void eval() override {
    using cc::to_string;

    // Upon reception of a NOC message, remove transport layer
    // encapsulation and issue to the appropriate ingress queue.
    MessageQueue* noc_mq = model_->noc_dir__msg_q();
    const NocMsg* nocmsg = static_cast<const NocMsg*>(noc_mq->dequeue());

    // Validate message
    if (nocmsg->cls() != MessageClass::Noc) {
      LogMessage lmsg("Received invalid message class: ");
      lmsg.append(to_string(nocmsg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    const Message* msg = nocmsg->payload();
    MessageQueue* iss_mq = model_->lookup_rdis_mq(msg->cls());
    if (iss_mq == nullptr) {
      LogMessage lmsg("Message queue not found for class: ");
      lmsg.append(to_string(msg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    // Forward message message to destination queue and discard
    // encapsulation/transport message.
    iss_mq->issue(msg);
    nocmsg->release();

    // Set conditions for subsequent re-evaluations.
    if (!noc_mq->empty()) {
      // TODO:Cleanup
      // Wait some delay
      wait_for(kernel::Time{10, 0});
    } else {
      // Not further work; await until noc ingress queue becomes non-full.
      wait_on(noc_mq->non_empty_event());
    }
  }

 private:
  DirModel* model_ = nullptr;
};

//
//
class DirCommandInterpreter {
  struct State {
    // Transaction Table state
    DirTState* st;
  };
 public:
  DirCommandInterpreter() = default;

  void set_dir(DirModel* model) { model_ = model; }
  void set_process(AgentProcess* process) { process_ = process; }

  void execute(DirContext& ctxt, const DirCommand* cmd) {
    switch (cmd->opcode()) {
      default: {
      } break;
#define __declare_dispatcher(__name)            \
        case DirOpcode::__name:                 \
          execute##__name(ctxt, cmd);           \
          break;
        DIROPCODE_LIST(__declare_dispatcher)
#undef __declare_dispatcher
    }
  }
 private:
  void executeTableInstall(DirContext& ctxt, const DirCommand* cmd) {
    state_.st = new DirTState;
    DirTTable* tt = model_->tt();
    tt->install(ctxt.msg()->t(), state_.st);
  }
  
  void executeTableLookup(DirContext& ctxt, const DirCommand* cmd) {
    DirTTable* tt = model_->tt();
    if (auto it = tt->find(ctxt.msg()->t()); it != tt->end()) {
      state_.st = it->second;
    } else {
      throw std::runtime_error("Cannot find transaction table entry.");
    }
  }

  void executeTableInstallLine(DirContext& ctxt, const DirCommand* cmd) {
    state_.st->set_line(ctxt.line());
    ctxt.set_owns_line(false);
  }

  void executeMsgConsume(DirContext& ctxt, const DirCommand* cmd) {
    // Dequeue and release the head message of the currently
    // addressed Message Queue.
    const Message* msg = ctxt.mq()->dequeue();
    msg->release();
    ctxt.t().advance();
  }

  void executeInvokeCoherenceAction(DirContext& ctxt, const DirCommand* cmd) {
    CoherenceAction* action = cmd->action();
    action->execute();
  }

  void executeMqSetBlockedOnTable(DirContext& ctxt, const DirCommand* cmd) {
    DirTTable* tt = model_->tt();
    ctxt.mq()->set_blocked_until(tt->non_empty_event());
  }  
  
  void executeWaitOnMsg(DirContext& ctxt, const DirCommand* cmd) const {
    // Set wait state of current process; await the arrival of a
    // new message.
    MQArb* arb = model_->arb();
    process_->wait_on(arb->request_arrival_event());
  }

  void executeWaitOnMsgOrNextEpoch(DirContext& ctxt, const DirCommand* cmd) const {
    MQArb* arb = model_->arb();
    MQArbTmt t = arb->tournament();
    if (t.has_requester()) {
      process_->wait_for(kernel::Time{10, 0});
    } else {
      // Otherwise, block process until a new message arrives.
      executeWaitOnMsg(ctxt, cmd);
    }
  }
  
  //
  State state_;
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
    switch (ctxt.msg()->cls()) {
      case MessageClass::CohSrt: {
        process_cohsrt(ctxt, cl);
      } break;
      case MessageClass::CohCmd: {
        process_cohcmd(ctxt, cl);
      } break;
      case MessageClass::LLCCmdRsp: {
        process_llccmdrsp(ctxt, cl);
      } break;
      default: {
        LogMessage lmsg("Invalid message class received: ");
        lmsg.append(cc::to_string(ctxt.msg()->cls()));
        lmsg.level(Level::Error);
        log(lmsg);
      } break;
    }

    if (can_execute(cl)) {
      execute(ctxt, cl);
    }
  }

  void process_cohsrt(DirContext& ctxt, DirCommandList& cl) {
    Transaction* t = ctxt.msg()->t();
    DirTTable* tt = model_->tt();
    DirTState* st = lookup_state_or_fatal(t, false);
    if (st == nullptr) {
      if (!tt->full()) {
        // Free entries exist in the transaction table, therefore the
        // transaction can begin.
        const DirProtocol* protocol = model_->protocol();
        protocol->apply(ctxt, cl);
      } else {
        // The transaction table is full, therefore the new inbound
        // command is blocked.
        cl.push_back(cb::from_opcode(DirOpcode::MqSetBlockedOnTable));
        cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
      }
    } else {
      // Transaction is already present in the transaction table;
      // attempting to reinstall the transaction.
      LogMessage msg("Transation is already present in table.");
      msg.level(Level::Fatal);
      log(msg);
    }
  }

  void process_cohcmd(DirContext& ctxt, DirCommandList& cl) const {
    // Lookup transaction table or bail if not found.
    DirTState* st = lookup_state_or_fatal(ctxt.msg()->t());
    const DirProtocol* protocol = model_->protocol();
    ctxt.set_line(protocol->construct_line());
    ctxt.set_owns_line(true);
    protocol->apply(ctxt, cl);
  }

  void process_llccmdrsp(DirContext& ctxt, DirCommandList& cl) const {
    // Lookup transaction table or bail if not found.
    DirTState* st = lookup_state_or_fatal(ctxt.msg()->t());
    const DirProtocol* protocol = model_->protocol();
    ctxt.set_line(st->line());
    protocol->apply(ctxt, cl);
  }

  bool can_execute(const DirCommandList& cl) const {
    return true;
  }

  void execute(DirContext& ctxt, const DirCommandList& cl) {
    try {
      DirCommandInterpreter interpreter;
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


  DirTState* lookup_state_or_fatal(
      Transaction* t, bool allow_fatal = true) const {
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

  // Pointer to parent directory instance.
  DirModel* model_ = nullptr;
};

DirModel::DirModel(kernel::Kernel* k, const DirModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

DirModel::~DirModel() {
  delete noc_dir__msg_q_;
  delete cpu_dir__cmd_q_;
  delete llc_dir__rsp_q_;
  delete arb_;
  delete cache_;
  delete noci_proc_;
  delete rdis_proc_;
  delete protocol_;
}

void DirModel::build() {
  // NOC -> DIR message queue
  noc_dir__msg_q_ = new MessageQueue(k(), "noc_dir__msg_q", 3);
  add_child_module(noc_dir__msg_q_);
  // NOC -> DIR command queue
  cpu_dir__cmd_q_ = new MessageQueue(k(), "cpu_dir__cmd_q", 3);
  add_child_module(cpu_dir__cmd_q_);
  // LLC -> DIR command queue
  llc_dir__rsp_q_ = new MessageQueue(k(), "llc_dir__rsp_q", 3);
  add_child_module(llc_dir__rsp_q_);
  // Construct arbiter
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Dir state cache
  CacheModelConfig cfg;
  cache_ = new CacheModel<DirLineState*>(cfg);
  // Construct transaction table.
  tt_ = new DirTTable(k(), "tt", 16);
  add_child_module(tt_);
  // Construct NOC ingress thread.
  noci_proc_ = new NocIngressProcess(k(), "noci", this);
  add_child_process(noci_proc_);
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
}

//
//
void DirModel::drc() {
  if (dir_noc__msg_q_ == nullptr) {
    LogMessage lmsg("Dir to NOC message queue is unbound.", Level::Fatal);
    log(lmsg);
  }
}

//
//
MessageQueue* DirModel::lookup_rdis_mq(MessageClass cls) const {
  switch (cls) {
    case MessageClass::CohSrt:
      return cpu_dir__cmd_q_;
    case MessageClass::CohCmd:
      return cpu_dir__cmd_q_;
    case MessageClass::LLCCmdRsp:
      return llc_dir__rsp_q_;
    default:
      return nullptr;
  }
}

}  // namespace cc
