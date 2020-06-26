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

#include "cc.h"
#include <vector>

struct SimConfig {
  // Toplevel name
  std::string name;

  std::vector<cc::L2CacheModelConfig> l2cfgs;
};

class SimTop : cc::kernel::Module {
 public:
  SimTop(cc::kernel::Kernel* k, const SimConfig& config)
      : cc::kernel::Module(k, config.name), config_(config) {
    build();
  }
  virtual ~SimTop() {
    for (cc::L2CacheModel* l2c : l2cs_) delete l2c;
  }

  // Return simulation configuration.
  SimConfig config() const { return config_; }

  // Invoke simulation.
  void run() {
    // Build and elaborate simulation environment.
    elaborate();
    // Run Design Rule Check to validate environment correctness.
    drc();
    // Run simulation.
    k()->run();
  }
  
 protected:
  // Construct top-level simulation enviornment and associated
  // collateral.
  void build() {
    for (const cc::L2CacheModelConfig& l2cfg : config_.l2cfgs) {
      cc::L2CacheModel* l2c = new cc::L2CacheModel(k(), l2cfg);
      add_child(l2c);
      l2cs_.push_back(l2c);
    }
  }
  void elaborate() {
    Module::elaborate();
  }
  void drc() {
    Module::drc();
  }
 private:
  SimConfig config_;
  std::vector<cc::L2CacheModel*> l2cs_;
};

int main(int argc, char** argv) {
  SimConfig cfg;
  cfg.name = "top";
  cc::kernel::Kernel k;
  SimTop top(&k, cfg);
  top.run();
  return 0;
}
