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

#include "mem.h"
//#include "mem_enum.h"

namespace cc {

class MemModel::MainProcess : public kernel::Process {

  struct Context {
  };
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, MemModel* model)
      : kernel::Process(k, name), model_(model) {
  }
 private:

  // Initialization
  void init() override {
    Arbiter<const Message*>* arb = model_->arb();
    wait_on(arb->request_arrival_event());
  }

  // Elaboration
  void eval() override {
    LogMessage lmsg("Got message");
    log(lmsg);

    Arbiter<const Message*>* arb = model_->arb();
    wait_on(arb->request_arrival_event());
  }

  // Finalization
  void fini() override {
  }

  // Current execution context
  Context ctxt_;
  // Current machine state
  //LlcState state_ = LlcState::AwaitingMessage;
  // Pointer to owning Mem instance.
  MemModel* model_ = nullptr;
};

MemModel::MemModel(kernel::Kernel* k)
    : Agent(k, "mem") {
  build();
}

MemModel::~MemModel() {
  delete noc_mem__msg_q_;
  delete arb_;
  delete main_;
}

void MemModel::build() {
  // NOC -> Mem message queue:
  noc_mem__msg_q_ = new MessageQueue(k(), "noc_mem__msg_q", 3);
  add_child_module(noc_mem__msg_q_);
  // Construct arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  add_child_module(arb_);
  // Construct main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void MemModel::register_agent(Agent* agent) {
  clients_.push_back(agent);
}

void MemModel::elab() {
  arb_->add_requester(noc_mem__msg_q_);
}

void MemModel::drc() {
}

} // namespace cc

