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

#include "cpu.h"

#include <exception>
#include <sstream>

#include "l1cache.h"
#include "msg.h"
#include "stimulus.h"
#include "utility.h"

namespace cc {

L1CmdOpcode to_l1cache_opcode(CpuOpcode opcode) {
  switch (opcode) {
    case CpuOpcode::Load:
      return L1CmdOpcode::CpuLoad;
    case CpuOpcode::Store:
      return L1CmdOpcode::CpuStore;
    default:
      throw std::invalid_argument("Unknown CpuOpcode");
  }
}

//
//
class Cpu::ProducerProcess : public kernel::Process {
 public:
  ProducerProcess(kernel::Kernel* k, const std::string& name, Cpu* cpu)
      : kernel::Process(k, name), cpu_(cpu) {}

 private:
  // Initialization
  virtual void init() override {
    StimulusContext* stimulus = cpu_->stimulus();
    Frontier f;
    if (stimulus->front(f)) {
      wait_until(f.time);
    }
  }

  // Finalization
  virtual void fini() override {}

  // Elaboration
  virtual void eval() override {
    StimulusContext* stimulus = cpu_->stimulus();
    Frontier f;
    if (!stimulus->front(f)) return;

    MessageQueue* mq = cpu_->cpu_l1__cmd_q();
    if (mq->full()) {
      // Cpu issue queue has backpressured, therefore await notification
      // that the queue has become non-full.
      wait_on(mq->non_full_event());
      return;
    }

    // A new transaction starts.
    Transaction* t = cpu_->start_transaction();
    // Free space in the issue queue, form a message and issue.
    // L1CmdMsg* msg = new L1CmdMsg;
    L1CmdMsg* msg = Pool<L1CmdMsg>::construct();
    const Command& cmd = f.cmd;
    msg->set_opcode(to_l1cache_opcode(cmd.opcode()));
    msg->set_addr(cmd.addr());
    msg->set_t(t);
    // Issue message
    mq->issue(msg);
    // Consume stimulus.
    stimulus->issue();
    if (stimulus->done()) {
      // Stimulus has been exhausted, wait until further stimulus has
      // arrived.
      wait_on(stimulus->non_empty_event());
      // Await next command
    } else if (stimulus->front(f)) {
      wait_until(f.time);
    }
  }

  // Point to process owner module.
  Cpu* cpu_ = nullptr;
};

//
//
class Cpu::ConsumerProcess : public kernel::Process {
 public:
  ConsumerProcess(kernel::Kernel* k, const std::string& name, Cpu* cpu)
      : kernel::Process(k, name), cpu_(cpu) {}

 private:
  // Initialization
  virtual void init() override {
    MessageQueue* mq = cpu_->l1_cpu__rsp_q();
    wait_on(mq->non_empty_event());
  }

  // Elaboration
  virtual void eval() override {
    MessageQueue* mq = cpu_->l1_cpu__rsp_q();
    if (!mq->empty()) {
      const Message* msg = mq->dequeue();
      switch (msg->cls()) {
        case MessageClass::L1CmdRsp: {
          StimulusContext* stimulus = cpu_->stimulus();
          const L1CmdRspMsg* l1rspmsg = static_cast<const L1CmdRspMsg*>(msg);

          Transaction* t = l1rspmsg->t();
          // Transaction is complete.
          cpu_->end_transaction(t);
          l1rspmsg->release();
          stimulus->retire();
        } break;
        default: {
        } break;
      }
    }
    if (mq->empty()) {
      wait_on(mq->non_empty_event());
    } else {
      next_delta();
    }
  }

  // Finalization
  virtual void fini() override {}

  // Point to process owner module.
  Cpu* cpu_ = nullptr;
};

//
//
Cpu::Cpu(kernel::Kernel* k, const CpuConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

Cpu::~Cpu() {
  if (stimulus_ != nullptr) {
    delete pp_;
    delete cp_;
  }
  delete stimulus_;
  delete l1_cpu__rsp_q_;
}

void Cpu::build() {
  // Response queue
  l1_cpu__rsp_q_ = new MessageQueue(k(), "l1_cpu__rsp_q", 3);
  add_child_module(l1_cpu__rsp_q_);
}

void Cpu::set_stimulus(StimulusContext* stimulus) {
  if (stimulus_ != nullptr) {
    LogMessage msg("Stimulus has already been defined: ");
    msg.append(stimulus->path());
    msg.level(Level::Fatal);
    log(msg);
  }

  stimulus_ = stimulus;
  // Construct producer thread
  pp_ = new ProducerProcess(k(), "producer", this);
  add_child_process(pp_);
  // Construct consumer thread
  cp_ = new ConsumerProcess(k(), "consumer", this);
  add_child_process(cp_);
}

void Cpu::set_cpu_l1__cmd_q(MessageQueue* mq) {
  cpu_l1__cmd_q_ = mq;
  add_child_module(cpu_l1__cmd_q_);
}

void Cpu::drc() {
  // Do DRC
  if (stimulus_ == nullptr) {
    // No transaction source associated with L1; raise warning.
    const LogMessage msg("L1Cache has no associated stimulus.", Level::Warning);
    log(msg);
  }
}

Transaction* Cpu::start_transaction() {
  Transaction* t = new Transaction;
  t->set_start_time(k()->time());
  ts_.insert(t);

  LogMessage msg("Transaction starts: ");
  msg.append(t->to_string());
  msg.level(Level::Info);
  log(msg);

  return t;
}

void Cpu::end_transaction(Transaction* t) {
  std::set<Transaction*>::iterator it = ts_.find(t);
  if (it != ts_.end()) {
    // TODO:
    LogMessage msg("Transaction ends: ");
    msg.append(t->to_string());
    msg.level(Level::Info);
    log(msg);

    (*it)->release();
    ts_.erase(it);
  } else {
    LogMessage lmsg("Unknown transaction consumed!");
    lmsg.level(Level::Fatal);
    log(lmsg);
  }
}

}  // namespace cc
