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

#include "cc/l1cache.h"
#include "cc/sim.h"

namespace cc {

class L1CacheModel::MainProcess : public kernel::Process {
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model)
  {}

  // Initialization
  void init() override {
  }

  // Finalization
  void fini() override {
  }

  // Evaluation
  void eval() override {
  }
 private:
  // Pointer to parent module.
  L1CacheModel* model_ = nullptr;
};

L1CacheModel::L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config)
    : kernel::Module(k, config.name), config_(config) {
  build();
}

L1CacheModel::~L1CacheModel() {
}

void L1CacheModel::build() {
  // Capture stimulus;
  proc_ = new ProcessorModel(k(), "cpu");
  proc_->set_stimulus(config_.stim);
  add_child(proc_);

  // Construct queues: TBD
  MessageQueue* mq = new MessageQueue(k(), "cmdq", 16);
  add_child(mq);
  mqs_.push_back(mq);

  // Arbiter
  arb_ = new Arbiter<Message*>(k(), "arb");
  arb_->add_requester(proc_);
  for (MessageQueue* mq : mqs_) {
    arb_->add_requester(mq);
  }
  add_child(arb_);

  // Main thread of execution
  main_ = new MainProcess(k(), "main", this);
  add_child(main_);
}

void L1CacheModel::elab() {
  // Do elaborate
}
 
void L1CacheModel::drc() {
  if (mqs_.empty()) {
    const LogMessage msg{"L1Cache has no message queues.", Level::Fatal};
    log(msg);
  }
}

} // namespace cc
