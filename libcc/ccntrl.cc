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

#include "ccntrl.h"

#include "amba.h"
#include "dir.h"
#include "msg.h"
#include "noc.h"
#include "primitives.h"
#include "protocol.h"
#include "utility.h"

namespace cc {

const char* to_string(CCOpcode opcode) {
  switch (opcode) {
#define __declare_to_string(__name)             \
    case CCOpcode::__name:                      \
      return #__name;
    CCOPCODE_LIST(__declare_to_string)
#undef __declare_to_string
    default: return "Invalid";
  }
}

CCCommand::~CCCommand() {
  switch (opcode()) {
    case CCOpcode::InvokeCoherenceAction:
      oprands.coh.action->release();
      break;
    default:
      break;
  }
}

std::string CCCommand::to_string() const {
  KVListRenderer r;
  r.add_field("opcode", cc::to_string(opcode()));
  switch (opcode()) {
    case CCOpcode::InvokeCoherenceAction: {
      r.add_field("action", oprands.coh.action->to_string());
    } break;
    default: {
    } break;
  }
  return r.to_string();
}

CCCommand* CCCommandBuilder::from_opcode(CCOpcode opcode) {
  return new CCCommand(opcode);
}

CCCommand* CCCommandBuilder::from_action(CoherenceAction* action) {
  CCCommand* cmd = new CCCommand(CCOpcode::InvokeCoherenceAction);
  cmd->oprands.coh.action = action;
  return cmd;
}

CCContext::~CCContext() {
  if (owns_line_) {
    line_->release();
  }
}

CCCommandList::~CCCommandList() {
  for (CCCommand* cmd : cmds_) {
    cmd->release();
  }
}

void CCCommandList::push_back(CCCommand* cmd) { cmds_.push_back(cmd); }

//
//
class CCCommandInterpreter {
  struct State {
  };
 public:
  CCCommandInterpreter() = default;

  void set_cc(CCModel* model) { model_ = model; }
  void set_process(AgentProcess* process) { process_ = process; }

  void execute(CCContext& ctxt, const CCCommand* c) {
    switch (c->opcode()) {
      default: {
      } break;
#define __declare_dispatcher(__name)            \
        case CCOpcode::__name:                  \
          execute##__name(ctxt, c);             \
          break;
        CCOPCODE_LIST(__declare_dispatcher)
#undef __declare_dispatcher
      }
  }
 private:

  void executeStartTransaction(CCContext& ctxt, const CCCommand* cmd) {
    CCTTable* tt = model_->tt();
    CCTState* st = new CCTState();
    st->set_line(ctxt.line());
    tt->install(ctxt.msg()->t(), st);
    ctxt.set_owns_line(false);
    
  }

  void executeEndTransaction(CCContext& ctxt, const CCCommand* cmd) {
    CCTTable* tt = model_->tt();
    Transaction* t = ctxt.msg()->t();
    if (auto it = tt->find(t); it != tt->end()) {
      tt->remove(t);
    } else {
      throw std::runtime_error("Table entry for transaction does not exist.");
    }
  }
  
  void executeInvokeCoherenceAction(CCContext& ctxt, const CCCommand* cmd) {
    CoherenceAction* action = cmd->action();
    action->execute();
  }

  void executeMsgConsume(CCContext& ctxt, const CCCommand* cmd) {
    // Dequeue and release the head message of the currently
    // addressed Message Queue.
    const Message* msg = ctxt.mq()->dequeue();
    msg->release();
    ctxt.t().advance();
  }

  void executeWaitOnMsg(CCContext& ctxt, const CCCommand* cmd) {
    // Set wait state of current process; await the arrival of a
    // new message.
    MQArb* arb = model_->arb();
    process_->wait_on(arb->request_arrival_event());
  }

  void executeWaitNextEpochOrWait(CCContext& ctxt, const CCCommand* cmd) {
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
  CCModel* model_ = nullptr;
};

//
//
class CCModel::RdisProcess : public AgentProcess {
  using cb = CCCommandBuilder;
 public:
  RdisProcess(kernel::Kernel* k, const std::string& name, CCModel* model)
      : AgentProcess(k, name), model_(model) {}

 private:
  // Initialization
  void init() override {
    CCContext ctxt;
    CCCommandList cl;
    cl.push_back(cb::from_opcode(CCOpcode::WaitOnMsg));
    execute(ctxt, cl);
  }

  // Evaluation
  void eval() override {
    CCCommandList cl;
    CCContext ctxt;
    ctxt.set_cc(model_);
    MQArb* arb = model_->arb();
    ctxt.set_t(arb->tournament());

    // Check if requests are present, if not block until a new message
    // arrives at the arbiter. Process should ideally not wake in the
    // absence of requesters.
    if (!ctxt.t().has_requester()) {
      cl.push_back(cb::from_opcode(CCOpcode::WaitOnMsg));
      execute(ctxt, cl);
      return;
    }
    // Fetch nominated message queue
    ctxt.set_mq(ctxt.t().winner());

    // Dispatch to appropriate message class
    switch (ctxt.msg()->cls()) {
      case MessageClass::AceCmd: {
        process_acecmd(ctxt, cl);
      } break;
      case MessageClass::CohEnd: {
        process_cohend(ctxt, cl);
      } break;
      case MessageClass::CohCmdRsp: {
        process_cohcmdrsp(ctxt, cl);
      } break;
      case MessageClass::Dt: {
        process_dt(ctxt, cl);
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

  void process_acecmd(CCContext& ctxt, CCCommandList& cl) const {
    const CCProtocol* protocol = model_->protocol();

    ctxt.set_line(protocol->construct_line());
    ctxt.set_owns_line(true);
    protocol->apply(ctxt, cl);
  }

  void process_cohend(CCContext& ctxt, CCCommandList& cl) const {
    const CCTState* st = lookup_state_or_fail(ctxt.msg()->t()); 
    ctxt.set_line(st->line());
    const CCProtocol* protocol = model_->protocol();
    protocol->apply(ctxt, cl);
  }

  void process_cohcmdrsp(CCContext& ctxt, CCCommandList& cl) const {
    const CCTState* st = lookup_state_or_fail(ctxt.msg()->t()); 
    ctxt.set_line(st->line());
    const CCProtocol* protocol = model_->protocol();
    protocol->apply(ctxt, cl);
  }

  void process_dt(CCContext& ctxt, CCCommandList& cl) const {
    const CCTState* st = lookup_state_or_fail(ctxt.msg()->t()); 
    const CCProtocol* protocol = model_->protocol();
    protocol->apply(ctxt, cl);
  }  

  bool can_execute(const CCCommandList& cl) const {
    return true;
  }

  void execute(CCContext& ctxt, const CCCommandList& cl) {
    try {
      CCCommandInterpreter interpreter;
      interpreter.set_cc(model_);
      interpreter.set_process(this);
      for (const CCCommand* cmd : cl) {
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

  CCTState* lookup_state_or_fail(Transaction* t) const {
    CCTTable* tt = model_->tt();
    CCTTable::iterator it;
    if (it = tt->find(t); it == tt->end()) {
      // Expect to find a entry in the transaction table. If an entry
      // is not present bail.
      LogMessage msg("Transaction not found in table.");
      msg.level(Level::Fatal);
      log(msg);
    }
    return it->second;
  }

  // Cache controller instance.
  CCModel* model_ = nullptr;
};

//
//
class CCNocEndpoint : public NocEndpoint {
 public:
  //
  CCNocEndpoint(kernel::Kernel* k, const std::string& name)
      : NocEndpoint(k, name)
  {}
  //
  void register_endpoint(MessageClass cls, MessageQueueProxy* p) {
    endpoints_.insert(std::make_pair(cls, p));
  }
  //
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

CCModel::CCModel(kernel::Kernel* k, const CCConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

CCModel::~CCModel() {
  delete l2_cc__cmd_q_;
  delete dir_cc__rsp_q_;
  delete cc__dt_q_;
  delete arb_;
  delete rdis_proc_;
  delete noc_endpoint_;
  delete tt_;
  delete protocol_;
  delete cc_l2__rsp_q_;
  delete cc_noc__msg_q_;
  for (MessageQueueProxy* p : endpoints_) { delete p; }
}

void CCModel::build() {
  // Construct L2 to CC command queue
  l2_cc__cmd_q_ = new MessageQueue(k(), "l2_cc__cmd_q", 3);
  add_child_module(l2_cc__cmd_q_);
  // DIR -> CC response queue
  dir_cc__rsp_q_ = new MessageQueue(k(), "dir_cc__rsp_q", 3);
  add_child_module(dir_cc__rsp_q_);
  //
  cc__dt_q_ = new MessageQueue(k(), "cc__dt_q", 3);
  add_child_module(cc__dt_q_);
  // Arbiteer
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Dispatcher process
  rdis_proc_ = new RdisProcess(k(), "rdis_proc", this);
  add_child_process(rdis_proc_);
  // NOC endpoint
  noc_endpoint_ = new CCNocEndpoint(k(), "noc_ep");
  add_child_module(noc_endpoint_);
  // Transaction table
  tt_ = new CCTTable(k(), "tt", 16);
  add_child_module(tt_);
  // Create protocol instance
  protocol_ = config_.pbuilder->create_cc(k());
  add_child_module(protocol_);
}

//
//
void CCModel::elab() {
  // Add ingress queues to arbitrator.
  arb_->add_requester(l2_cc__cmd_q_);
  arb_->add_requester(dir_cc__rsp_q_);
  arb_->add_requester(cc__dt_q_);

  MessageQueueProxy* p = nullptr;

  // Fix up ingress queues for the NOC ingress process.
  p = cc__dt_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::Dt, p);
  endpoints_.push_back(p);

  p = l2_cc__cmd_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::L2Cmd, p);
  endpoints_.push_back(p);
      
  p = dir_cc__rsp_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::CohCmdRsp, p);
  noc_endpoint_->register_endpoint(MessageClass::CohEnd, p);
  endpoints_.push_back(p);
}

// Set CC -> NOC message queue
void CCModel::set_cc_noc__msg_q(MessageQueueProxy* mq) {
  cc_noc__msg_q_ = mq;
  add_child_module(cc_noc__msg_q_);
}

// Set CC -> L2 response queue
void CCModel::set_cc_l2__rsp_q(MessageQueueProxy* mq) {
  cc_l2__rsp_q_ = mq;
  add_child_module(cc_l2__rsp_q_);
}

//
//
void CCModel::drc() {
  if (dm() == nullptr) {
    // The Dir Mapper object computes the host directory for a
    // given address. In a single directory system, this is a basic
    // mapping to a single directory instance, but in more performant
    // systems this may some non-trivial mapping to multiple home
    // directories.
    LogMessage msg("Directory mapper is not defined.", Level::Warning);
    log(msg);
  }
}

MessageQueue* CCModel::endpoint() const {
  return noc_endpoint_->ingress_mq();
}

}  // namespace cc
