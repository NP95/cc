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

#include "cc/cpucluster.h"
#include "l2cache.h"
#include "l1cache.h"
#include "cpu.h"
#include "ccntrl.h"

namespace cc {

CpuCluster::CpuCluster(kernel::Kernel* k, const CpuClusterCfg& config)
    : Agent(k, config.name), config_(config) {
  build();
}

//
void CpuCluster::build() {
  // Construct cache controller.
  cc_= new CacheController(k(), config_.cc_config);
  add_child_module(cc_);

  // Construct L2 cache instance.
  l2c_ = new L2CacheModel(k(), config_.l2c_config);
  add_child_module(l2c_);

  // Construct L1 cache instance(s).
  for (std::size_t i = 0; i < config_.l1c_configs.size(); i++) {
    const L1CacheModelConfig& l1c_cfg = config_.l1c_configs[i];

    // L1 Cache Model
    L1CacheModel* l1c = new L1CacheModel(k(), l1c_cfg);
    l1cs_.push_back(l1c);
    add_child_module(l1c);

    // Create instance on L2 for new L1 cache.
    l2c_->add_l1c(l1c);

    // Construct associated CPU instance.
    Cpu* cpu = new Cpu(k(), config_.cpu_configs[i]);
    cpus_.push_back(cpu);
    add_child_module(cpu);
  }
}

//
void CpuCluster::pre_elab_drc() {
  // Validate that baseline agents are present before elaboration
  // starts.
  
  if (cc_ == nullptr) {
  }

  if (l2c_ == nullptr) {
  }
}

//
void CpuCluster::elab() {
  // Elabration occurs top-down; therefore ensure that all agents are
  // Bind Cache Controller to L2 instanceappropriately bound before an
  // attempt is made to elaborate down the hierarchy.
  
  // (L2 -> CC) Bind L2 cache to parent Cache Controller
  l2c_->set_cc(cc_);
  l2c_->set_l2_cc__cmd_q(cc_->l2_cc__cmd_q());
  // (CC -> L2) Bind Cache Controller to L2 instance.
  cc_->set_l2c(l2c_);
  
  // Bind L1 caches to parent L2.
  for (std::size_t i = 0; i < l1cs_.size(); i++) {
    L1CacheModel* l1c = l1cs_[i];
    // (L1C -> L2C)
    l1c->set_l2c(l2c_);
    l1c->set_l1_l2__cmd_q(l2c_->l1_l2__cmd_q(i));
  }

  // Bind CPU to parent L1.
  for (std::size_t i = 0; i < cpus_.size(); i++) {
    Cpu* cpu = cpus_[i];
    L1CacheModel* l1c = l1cs_[i];
    // (CPU -> L1)
    cpu->set_l1c(l1c);
    cpu->set_cpu_l1__cmd_q(l1c->cpu_l1__cmd_q());
    // (L1 -> CPU)
    l1c->set_cpu(cpu);
  }
}

//
void CpuCluster::drc() {
  if (l1cs_.empty()) {
    // Warning: no L1 instance.
  }

  if (cpus_.empty()) {
    // Warning: no CPU, therefore no stimulus.
  }
  
  if (cpus_.size() != l1cs_.size()) {
    // Fatal: Size mismatch on CPU to L1 Cache instances.
  }
}

} // namespace cc
