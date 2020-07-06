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

#include "cc/cpu.h"

#include "cpu_msg.h"

namespace cc {

class Cpu::MainProcess : public kernel::Process {
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, Cpu* parent)
      : kernel::Process(k, name), parent_(parent) {}
  void dequeued() {
    Stimulus* stim = parent_->stim_;
    stim->consume();
    if (!stim->done()) {
      const Stimulus::Frontier& f{stim->front()};
      wait_for(f.time);
    }
  }

 private:
  // Initialization
  virtual void init() override {
    Stimulus* stim = parent_->stim_;
    if (!stim->done()) {
      const Stimulus::Frontier& f{stim->front()};
      wait_for(f.time);
    }
  }

  // Finalization
  virtual void fini() override {}

  // Elaboration
  virtual void eval() override { parent_->mature_event_.notify(); }

  // Point to process owner module.
  Cpu* parent_ = nullptr;
};

Cpu::Cpu(kernel::Kernel* k, const std::string& name)
    : kernel::Module(k, name), mature_event_(k, "mature_event") {
  build();
}

void Cpu::push(const Message* msg) {
  Transaction* t = msg->t();
  std::set<Transaction*>::iterator it = ts_.find(t);
  if (it == ts_.end()) {
    const LogMessage lmsg{
        "Message received for transaction which does not exist.", Level::Error};
    log(lmsg);
  }

  // TODO: do something with the message.

  // Log Message:
  LogMessage lmsg("Message received: ");
  lmsg.append(msg->to_string());
  log(lmsg);
  // Log Transaction:
  LogMessage ltrn("Transaction complete: ");
  lmsg.append(t->to_string());
  log(ltrn);
  // Transaction is now complete remove from table.
  ts_.erase(it);
  delete msg;
  delete t;
}

bool Cpu::has_req() const {
  if (stim_->done()) return false;

  const Stimulus::Frontier& f = stim_->front();
  return f.time <= k()->time();
}

const Message* Cpu::peek() const {
  if (!has_req()) return nullptr;

  if (msg_ == nullptr) {
    construct_new_message();
  }
  return msg_;
}

void Cpu::construct_new_message() const {
  const Stimulus::Frontier& f = stim_->front();
  Command cmd = f.cmd;
  // Transaction starts. Create transction and form message.
  Transaction* t = new Transaction;
  CpuCommandMessage* m = new CpuCommandMessage(t);
  m->set_addr(cmd.addr());
  switch (cmd.opcode()) {
    case Command::Load: {
      // Construct Load instrduction message.
      m->set_opcode(CpuCommandMessage::Load);
    } break;
    case Command::Store: {
      // Construct Store instrduction message.
      m->set_opcode(CpuCommandMessage::Store);
    } break;
    default: {
      const LogMessage msg{"Unknown command opcode.", Level::Fatal};
      log(msg);
    };
  }
  // Message placeholder.
  msg_ = m;
}

const Message* Cpu::dequeue() {
  if (msg_ == nullptr) {
    construct_new_message();
  }

  Transaction* t = msg_->t();

  // Log Transaction:
  LogMessage ltrn("Transaction issued: ");
  ltrn.append(t->to_string());
  log(ltrn);
  // Log Message:
  LogMessage lmsg("Message issued: ");
  lmsg.append(msg_->to_string());
  log(lmsg);

  // Add transaction to processor transaction table.
  ts_.insert(t);
  // Consume stimulus command, from which this command had been constructed.
  main_->dequeued();
  // Nullify preconstructed and return old.
  return std::exchange(msg_, nullptr);
}

kernel::Event& Cpu::request_arrival_event() { return mature_event_; }

void Cpu::build() {
  // Construct main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void Cpu::elab() {}

void Cpu::drc() {
  // Do DRC
  if (stim_ == nullptr) {
    // No transaction source associated with L1; raise warning.
    const LogMessage msg("L1Cache has no associated stimulus.", Level::Warning);
    log(msg);
  }
}

}  // namespace cc
