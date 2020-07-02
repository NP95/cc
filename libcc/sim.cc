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

#include "cc/sim.h"

namespace cc {

MessageQueue::MessageQueue(kernel::Kernel* k, const std::string& name, std::size_t n)
    : kernel::Module(k, name) {
  build(n);
}

bool MessageQueue::has_req() const {
  return !q_->empty();
}

const Message* MessageQueue::peek() const {
  return nullptr;
}

const Message* MessageQueue::dequeue() {
  return nullptr;
  /*
  const Message* msg;
  if (!q_->dequeue(msg)) {
    const LogMessage msg{"Attempt to dequeue Message failed.", Level::Fatal};
    log(msg);
    return nullptr;
  }
  return msg;
  */
}

kernel::Event& MessageQueue::request_arrival_event() {
  return q_->non_empty_event();
}

void MessageQueue::build(std::size_t n) {
  q_ = new Queue<const Message*>(k(), "queue", n);
  add_child(q_);
}

class ProcessorModel::MainProcess : kernel::Process {
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, ProcessorModel* parent)
      : kernel::Process(k, name), parent_(parent) {
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
  virtual void fini() override {
  }

  // Elaboration
  virtual void eval() override {
    parent_->mature_event_.notify();
  }

  ProcessorModel* parent_ = nullptr;
};

ProcessorModel::ProcessorModel(kernel::Kernel* k, const std::string& name)
    : kernel::Module(k, name), mature_event_(k, "mature_event") {
  build();
}

bool ProcessorModel::has_req() const {
  if (stim_->done()) return false;

  const Stimulus::Frontier& f = stim_->front();
  return f.time <= k()->time();
}

const Message* ProcessorModel::peek() const {
  if (!has_req()) return nullptr;
  
  if (msg_ == nullptr) { construct_new_message(); }
  return msg_;
}

void ProcessorModel::construct_new_message() const {
  const Stimulus::Frontier& f = stim_->front();
  Command cmd = f.cmd;
  // Transaction starts. Create transction and form message.
  Transaction* t = new Transaction;
  CpuMessage* m = new CpuMessage(t);
  m->set_addr(cmd.addr());
  switch (cmd.opcode()) {
    case Command::Load: {
      // Construct Load instrduction message.
      m->set_opcode(CpuMessage::Load);
    } break;
    case Command::Store: {
      // Construct Store instrduction message.
      m->set_opcode(CpuMessage::Store);
    } break;
    default: {
      const LogMessage msg{"Unknown command opcode.", Level::Fatal};
      log(msg);
    };
  }
  // Message placeholder.
  msg_ = m;
}

const Message* ProcessorModel::dequeue() {
  if (msg_ == nullptr) { construct_new_message(); }
  
  // Add transaction to processor transaction table.
  ts_.insert(msg_->t());
  // Consume stimulus command, from which this command had been constructed.
  stim_->consume();
  // Nullify preconstructed and return old.
  return std::exchange(msg_, nullptr);
}

kernel::Event& ProcessorModel::request_arrival_event() {
  return mature_event_;
}

void ProcessorModel::build() {
}

void ProcessorModel::elab() {
}

void ProcessorModel::drc() {
  // Do DRC
  if (stim_ == nullptr) {
    // No transaction source associated with L1; raise warning.
    const LogMessage msg(
        "L1Cache has no associated stimulus.", Level::Warning);
    log(msg);
  }
}

} // namespace cc
