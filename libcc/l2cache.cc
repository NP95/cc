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

#include "cc/l2cache.h"
#include "cc/sim.h"

namespace cc {

class L2CacheModel::MainProcess : public kernel::Process {
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L2CacheModel* model)
      : kernel::Process(k, name), model_(model)
  {}
 private:
  // Initialization:
  void init() override {
    Arbiter<const Message*>* arb = model_->arb_;
    wait_on(arb->request_arrival_event());
  }

  // Evaluation:
  void eval() override {
  }

  // Finalization:
  void fini() override {
  }

  // Pointer to parent L2.
  L2CacheModel* model_ = nullptr;
};

L2CacheModel::L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config)
    : kernel::Module(k, config.name), config_(config) {
  build();
}

L2CacheModel::~L2CacheModel() {
}


void L2CacheModel::build() {

  // Counter to compute the number of slots in the L2 command queue to
  // support the maximum number of commands from all associated child
  // L1. 
  std::size_t l2_cmd_queue_depth_n = 0;

  // Counter to compute the number of slots required in the L2
  // response queue to support the maximum number of commands to all
  // associated child L1.
  std::size_t l2_rsp_queue_depth_n = 0;
  
  // Construct child instances.
  for (const L1CacheModelConfig& l1cfg : config_.l1configs) {
    // Compute running count of queue sized.
    l2_cmd_queue_depth_n += l1cfg.l2_cmdq_credits_n;
    l2_rsp_queue_depth_n += l1cfg.l1_cmdq_slots_n;
    // COnstruct child L1 instance.
    L1CacheModel* l1c = new L1CacheModel(k(), l1cfg);
    add_child_module(l1c);
    l1cs_.push_back(l1c);
  }

  // Construct L1 Command Request Queue
  l1cmdreqq_ = new MessageQueue(k(), "l1cmdreqq", l2_cmd_queue_depth_n);
  add_child_module(l1cmdreqq_);

  // Construct L1 Command Response Queue
  l1cmdrspq_ = new MessageQueue(k(), "l1cmdrspq", l2_rsp_queue_depth_n);
  add_child_module(l1cmdrspq_);

  // Arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  arb_->add_requester(l1cmdreqq_);
  arb_->add_requester(l1cmdrspq_);
  add_child_module(arb_);

  // Main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void L2CacheModel::elab() {
  // Fix up parent cache.
  for (L1CacheModel* l1c : l1cs_) {
    l1c->set_parent(this);
  }
}
 
void L2CacheModel::drc() {
  if (l1cs_.empty()) {
    const LogMessage msg{"L2 has no child L1 cache(s).", Level::Fatal};
    log(msg);
  }
}

} // namespace cc
