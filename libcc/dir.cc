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
#include "utility.h"

namespace cc {

//
//
class DirModel::NocIngressProcess : public kernel::Process {
 public:
  NocIngressProcess(kernel::Kernel* k, const std::string& name, DirModel* model)
      : kernel::Process(k, name), model_(model) {}

  // Initialization
  void init() override {
    MessageQueue* mq = model_->noc_dir__msg_q();
    wait_on(mq->request_arrival_event());
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
    iss_mq->push(msg);
    nocmsg->release();

    // Set conditions for subsequent re-evaluations.
    if (!noc_mq->empty()) {
      // TODO:Cleanup
      // Wait some delay
      wait_for(kernel::Time{10, 0});
    } else {
      // Not further work; await until noc ingress queue becomes non-full.
      wait_on(noc_mq->request_arrival_event());
    }
  }

 private:
  DirModel* model_ = nullptr;
};

class DirModel::RdisProcess : public kernel::Process {
  using CacheLineIt = CacheModel<DirLineState*>::LineIterator;

  enum class State { Idle, ProcessMessage, ExecuteActions };

  static const char* to_string(State s) {
    switch (s) {
      case State::Idle:
        return "Idle";
      case State::ProcessMessage:
        return "ProcessMessage";
      case State::ExecuteActions:
        return "ExecuteActions";
      default:
        return "Invalid";
    }
  }

 public:
  RdisProcess(kernel::Kernel* k, const std::string& name, DirModel* model)
      : kernel::Process(k, name), model_(model) {
    state_ = State::Idle;
  }

  State state() const { return state_; }
  void set_state(State state) {
#ifdef VERBOSE_LOGGING
    if (state_ != state) {
      LogMessage msg("State transition: ");
      msg.level(Level::Debug);
      msg.append(to_string(state_));
      msg.append(" -> ");
      msg.append(to_string(state));
      log(msg);
    }
#endif
    state_ = state;
  }

 private:
  // Initialization
  void init() override {
    set_state(State::Idle);
    MQArb* arb = model_->arb();
    // Await the arrival of a new message at the ingress message
    // queues.
    wait_on(arb->request_arrival_event());
  }

  // Evaluation
  void eval() override {
    // Idle state, awaiting more work.
    MQArb* arb = model_->arb();
    t_ = arb->tournament();

    // Check for the presence of issue-able messages at the
    // pipeline front-end.
    if (t_.has_requester()) {
      const Message* msg = t_.winner()->dequeue();

      switch (msg->cls()) {
        case MessageClass::CohCmd: {
          const CohCmdMsg* cmd = static_cast<const CohCmdMsg*>(msg);

          CohEndMsg* cohend = new CohEndMsg;
          cohend->set_t(cmd->t());

          NocMsg* nocmsg = new NocMsg;
          nocmsg->set_payload(cohend);
          nocmsg->set_origin(model_);
          nocmsg->set_dest(cmd->origin());

          model_->dir_noc__msg_q()->issue(nocmsg);
        } break;
        default: {
        } break;
      }
    }
    wait_on_context();
  }

  void wait_on_context() {
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
  }

  // Current execution context
  MQArbTmt t_;
  // Current machine state
  State state_;
  // Coherence context
  DirCoherenceContext context_;
  // Coherence action list.
  DirActionList action_list_;
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
  // Construct NOC ingress thread.
  noci_proc_ = new NocIngressProcess(k(), "noci", this);
  add_child_process(noci_proc_);
  // Construct request dispatcher thread
  rdis_proc_ = new RdisProcess(k(), "rdis", this);
  add_child_process(rdis_proc_);
  // Setup protocol
  protocol_ = config_.pbuilder->create_dir();
}

//
//
void DirModel::elab() {
  // Register message queue end-points
  arb_->add_requester(cpu_dir__cmd_q_);
  arb_->add_requester(llc_dir__rsp_q_);
  protocol_->set_dir(this);
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
