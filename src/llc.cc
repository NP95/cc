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

#include "llc.h"

#include "cpucluster.h"
#include "dir.h"
#include "mem.h"
#include "msg.h"
#include "noc.h"
#include "utility.h"

namespace cc {

const char* to_string(LLCCmdOpcode opcode) {
  switch (opcode) {
    case LLCCmdOpcode::Fill:
      return "Fill";
    case LLCCmdOpcode::Evict:
      return "Evict";
    case LLCCmdOpcode::PutLine:
      return "PutLine";
    default:
      return "Invalid";
  }
}

//
//
LLCCmdMsg::LLCCmdMsg() : Message(MessageClass::LLCCmd) {}

std::string LLCCmdMsg::to_string() const {
  using cc::to_string;

  Hexer h;
  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("cls", to_string(cls()));
  r.add_field("opcode", to_string(opcode()));
  r.add_field("addr", h.to_hex(addr()));
  return r.to_string();
}

const char* to_string(LLCRspOpcode opcode) {
  switch (opcode) {
    case LLCRspOpcode::Okay:
      return "Okay";
    default:
      return "Invalid";
  }
}

//
//
LLCCmdRspMsg::LLCCmdRspMsg() : Message(MessageClass::LLCCmdRsp) {}

std::string LLCCmdRspMsg::to_string() const {
  using cc::to_string;

  Hexer h;
  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("cls", to_string(cls()));
  r.add_field("opcode", to_string(opcode()));
  return r.to_string();
}

enum class State { FillAwaitMemRsp, PutAwaitCCDtRsp, PutAwaitMemDtRsp, Idle };

const char* to_string(State state) {
  switch (state) {
    case State::FillAwaitMemRsp:
      return "FillAwaitMemRsp";
    case State::PutAwaitCCDtRsp:
      return "PutAwaitCCDtRsp";
    case State::PutAwaitMemDtRsp:
      return "PutAwaitMemDtRsp";
    default:
      return "Idle";
  }
}

//
//
class LLCTState {
 public:
  LLCTState() = default;

  //
  State state() const { return state_; }
  Agent* origin() const { return origin_; }

  //
  void set_state(State state) { state_ = state; }
  void set_origin(Agent* origin) { origin_ = origin; }

 private:
  State state_;
  Agent* origin_ = nullptr;
};

//
//
class LLCModel::RdisProcess : public AgentProcess {
 public:
  RdisProcess(kernel::Kernel* k, const std::string& name, LLCModel* model)
      : AgentProcess(k, name), model_(model) {}

 private:
  // Initialization
  void init() override {
    MQArb* arb = model_->arb();
    wait_on(arb->request_arrival_event());
  }

  // Elaboration
  void eval() override {
    MQArb* arb = model_->arb();
    MQArbTmt t;

    t = arb->tournament();
    if (t.has_requester()) {
      const Message* msg = t.winner()->dequeue();

      LogMessage lm;
      lm.append("Execute message: ");
      lm.append(msg->to_string());
      lm.level(Level::Debug);
      log(lm);

      switch (msg->cls()) {
        case MessageClass::LLCCmd: {
          process(static_cast<const LLCCmdMsg*>(msg));
        } break;
        case MessageClass::MemRsp: {
          process(static_cast<const MemRspMsg*>(msg));
        } break;
        case MessageClass::DtRsp: {
          process(static_cast<const DtRspMsg*>(msg));
        } break;
        default: {
          using cc::to_string;
          // Unknown message class
          LogMessage lm("Unknown message received: ");
          lm.append(to_string(msg->cls()));
          lm.level(Level::Fatal);
          log(lm);
        } break;
      }
      msg->release();
      t.advance();

      wait_for(kernel::Time{10, 0});
    } else {
      wait_on(arb->request_arrival_event());
    }
  }

  void process(const LLCCmdMsg* msg) {
    switch (msg->opcode()) {
      case LLCCmdOpcode::Fill: {
        // Message to LLC
        MemCmdMsg* memcmd = new MemCmdMsg;
        memcmd->set_origin(model_);
        memcmd->set_opcode(MemCmdOpcode::Read);
        memcmd->set_dest(model_);
        memcmd->set_t(msg->t());
        issue_emit_to_noc(model_->mc(), memcmd);

        LLCTState* tstate = new LLCTState;
        tstate->set_state(State::FillAwaitMemRsp);
        install_state_or_fatal(msg->t(), tstate);
      } break;
      case LLCCmdOpcode::Evict: {
      } break;
      case LLCCmdOpcode::PutLine: {
        // Send data to requesting agent. Assumes prior fill operation
        // initiated by associated directory.
        DtMsg* dt = new DtMsg;
        dt->set_origin(model_);
        dt->set_t(msg->t());
        issue_emit_to_noc(msg->agent(), dt);

        LLCTState* tstate = new LLCTState;
        tstate->set_state(State::PutAwaitCCDtRsp);
        tstate->set_origin(msg->origin());
        install_state_or_fatal(msg->t(), tstate);
      } break;
      default: {
      } break;
    }
  }

  void process(const MemRspMsg* msg) {
    LLCTState* tstate = lookup_state_or_fatal(msg->t());
    switch (tstate->state()) {
      case State::FillAwaitMemRsp: {
        LLCCmdRspMsg* llcrsp = new LLCCmdRspMsg;
        llcrsp->set_t(msg->t());
        llcrsp->set_opcode(LLCRspOpcode::Okay);
        issue_emit_to_noc(model_->dir(), llcrsp);
        // Command complete: Delete transaction table entry.
        erase_state_or_fatal(msg->t());
      } break;
      default: {
      } break;
    }
  }

  void process(const DtRspMsg* msg) {
    LLCTState* tstate = lookup_state_or_fatal(msg->t());
    switch (tstate->state()) {
      case State::PutAwaitCCDtRsp: {
        // Issue Put response to originator directory
        LLCCmdRspMsg* llcrsp = new LLCCmdRspMsg;
        llcrsp->set_t(msg->t());
        llcrsp->set_opcode(LLCRspOpcode::Okay);
        issue_emit_to_noc(model_->dir(), llcrsp);
        // Command complete: Delete transaction table entry.
        erase_state_or_fatal(msg->t());
      } break;
      case State::PutAwaitMemDtRsp: {
        // Issue Put response to originator directory
        LLCCmdRspMsg* llcrsp = new LLCCmdRspMsg;
        llcrsp->set_t(msg->t());
        llcrsp->set_opcode(LLCRspOpcode::Okay);
        issue_emit_to_noc(model_->dir(), llcrsp);
        // Command complete: Delete transaction table entry.
        erase_state_or_fatal(msg->t());
      } break;
      default: {
      } break;
    }
  }

  void install_state_or_fatal(Transaction* t, LLCTState* tstate) {
    LLCTTable* tt = model_->tt();
    if (auto pp = tt->insert(std::make_pair(t, tstate)); !pp.second) {
      LogMessage msg("Could not install transaction state.");
      msg.level(Level::Fatal);
      log(msg);
    }
  }
  LLCTState* lookup_state_or_fatal(Transaction* t) const {
    LLCTTable* tt = model_->tt();
    LLCTState* st = nullptr;
    if (auto it = tt->find(t); it != tt->end()) {
      st = it->second;
    } else {
      // Expect to find a entry in the transaction table. If an entry
      // is not present bail.
      LogMessage msg("Transaction not found in table.");
      msg.level(Level::Fatal);
      log(msg);
    }
    return st;
  }
  void erase_state_or_fatal(Transaction* t) {
    LLCTTable* tt = model_->tt();
    if (auto it = tt->find(t); it != tt->end()) {
      delete it->second;
      tt->erase(it);
    } else {
      LogMessage msg("Transaction not found in table.");
      msg.level(Level::Fatal);
      log(msg);
    }
  }

  void issue_emit_to_noc(Agent* dest, const Message* msg) {
    NocMsg* nocmsg = new NocMsg;
    nocmsg->set_origin(model_);
    nocmsg->set_dest(dest);
    nocmsg->set_payload(msg);

    MessageQueueProxy* mq = model_->llc_noc__msg_q();
    mq->issue(nocmsg);
  }

  // Pointer to owning LLC instance.
  LLCModel* model_ = nullptr;
};

//
//
class LLCNocEndpoint : public NocEndpoint {
 public:
  LLCNocEndpoint(kernel::Kernel* k, const std::string& name)
      : NocEndpoint(k, name) {}
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

LLCModel::LLCModel(kernel::Kernel* k, const LLCModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

LLCModel::~LLCModel() {
  delete dir_llc__cmd_q_;
  delete mem_llc__rsp_q_;
  delete arb_;
  delete rdis_proc_;
  delete noc_endpoint_;
  for (MessageQueue* mq : cc_llc__rsp_qs_) {
    delete mq;
  }
  for (MessageQueueProxy* p : endpoints_) {
    delete p;
  }
  delete llc_noc__msg_q_;
  delete tt_;
}

void LLCModel::build() {
  // DIR -> LLC command queue
  dir_llc__cmd_q_ = new MessageQueue(k(), "dir_llc__cmd_q", 30);
  add_child_module(dir_llc__cmd_q_);
  // MEM -> LLC response queue
  mem_llc__rsp_q_ = new MessageQueue(k(), "mem_llc__rsp_q", 30);
  add_child_module(mem_llc__rsp_q_);
  // Construct arbiter
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Construct main thread
  rdis_proc_ = new RdisProcess(k(), "main", this);
  add_child_process(rdis_proc_);
  // NOC endpoint
  noc_endpoint_ = new LLCNocEndpoint(k(), "noc_ep");
  add_child_module(noc_endpoint_);
  // Construct transaction table.
  tt_ = new LLCTTable;
}

void LLCModel::register_cc(CpuCluster* cc) {
  const std::string mq_name = cc->name() + "_mq";
  MessageQueue* mq = new MessageQueue(k(), mq_name, 30);
  add_child_module(mq);
  cc_llc__rsp_qs_.push_back(mq);
}

void LLCModel::elab() {
  arb_->add_requester(dir_llc__cmd_q_);
  arb_->add_requester(mem_llc__rsp_q_);
  for (MessageQueue* mq : cc_llc__rsp_qs_) {
    arb_->add_requester(mq);
  }

  MessageQueueProxy* p = nullptr;

  //
  p = dir_llc__cmd_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::LLCCmd, p);
  endpoints_.push_back(p);
  //
  p = mem_llc__rsp_q_->construct_proxy();
  noc_endpoint_->register_endpoint(MessageClass::MemRsp, p);
  endpoints_.push_back(p);

  for (MessageQueue* mq : cc_llc__rsp_qs_) {
    p = mq->construct_proxy();
    noc_endpoint_->register_endpoint(MessageClass::DtRsp, p);
    endpoints_.push_back(p);
  }
}

void LLCModel::drc() {
  if (llc_noc__msg_q_ == nullptr) {
    LogMessage msg("LLC to NOC egress queue has not been bound.");
    msg.level(Level::Fatal);
    log(msg);
  }
}

MessageQueue* LLCModel::endpoint() const { return noc_endpoint_->ingress_mq(); }

void LLCModel::set_llc_noc__msg_q(MessageQueueProxy* mq) {
  llc_noc__msg_q_ = mq;
  add_child_module(llc_noc__msg_q_);
}

}  // namespace cc
