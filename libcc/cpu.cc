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
#include "cpu_gen.h"
#include "utility.h"
#include "msg.h"
#include "stimulus.h"
#include "l1cache.h"
#include <sstream>

namespace cc {

//
//
class Cpu::ProducerProcess : public kernel::Process {
  using Frontier = Stimulus::Frontier;
 public:
  ProducerProcess(kernel::Kernel* k, const std::string& name, Cpu* cpu)
      : kernel::Process(k, name), cpu_(cpu) {}
 private:
  // Initialization
  virtual void init() override {
    Stimulus* stimulus = cpu_->stimulus();
    if (!stimulus->done()) {
      const Frontier& f = stimulus->front();
      wait_for(f.time);
    }
  }

  // Finalization
  virtual void fini() override {
  }

  // Elaboration
  virtual void eval() override {
    Stimulus* stimulus = cpu_->stimulus();
    if (stimulus->done()) return;
    
    MessageQueue* mq = cpu_->cpu_l1__cmd_q();
    if (!mq->full()) {
      // A new transaction starts.
      Transaction* t = cpu_->start_transaction();
      // Free space in the issue queue, form a message and issue.
      CpuL1__CmdMsg* msg = new CpuL1__CmdMsg;
      const Command& cmd = stimulus->front().cmd;
      msg->set_opcode(cmd.opcode());
      msg->set_addr(cmd.addr());
      msg->set_t(t);
      msg->set_issue_time(k()->time());
      // Issue message
      mq->issue(msg);
      // Consume stimulus.
      stimulus->consume();
      // Await next command
      const Frontier& f = stimulus->front();
      wait_for(f.time);
    } else {
      // Cpu issue queue has backpressured, therefore await notification
      // that the queue has become non-full.
      wait_on(mq->non_full_event());
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
        case MessageClass::CpuRsp: {
          msg = static_cast<const CpuRspMsg*>(msg);

          Transaction* t = msg->t();
          // Transaction is complete.
          cpu_->end_transaction(t);

          msg->release();
        } break;
        default: {
          
        } break;
      }
    } else {
      // No respones, wait until soething arrives.
      wait_on(mq->non_empty_event());
    }
  }

  // Finalization
  virtual void fini() override {
  }

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
  delete l1_cpu__rsp_q_;
}

void Cpu::build() {
  // Capture stimulus
  stimulus_ = config_.stimulus;

  if (stimulus_ != nullptr) {
    // Construct producer thread
    pp_ = new ProducerProcess(k(), "producer", this);
    add_child_process(pp_);
    // Construct consumer thread
    cp_ = new ConsumerProcess(k(), "consumer", this);
    add_child_process(cp_);
  }

  // Response queue
  l1_cpu__rsp_q_ = new MessageQueue(k(), "l1_cpu__rsp_q", 3);
  add_child_module(l1_cpu__rsp_q_);
}

void Cpu::elab() {
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
  if (it == ts_.end()) {
    LogMessage lmsg("Unknown transaction consumed: ");
    lmsg.append(t->to_string());
    lmsg.level(Level::Fatal);
    log(lmsg);
  }
  // TODO:

  LogMessage msg("Transaction ends: ");
  msg.append(t->to_string());
  msg.level(Level::Info);
  log(msg);

  ts_.erase(it);
  (*it)->release();
}

}  // namespace cc
