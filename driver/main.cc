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
  // Level 2 Cache configuration.
  std::vector<cc::L2CacheModelConfig> l2configs;
  // NOC configuration
  cc::NocModelConfig nocconfig;
  // Directory configurations
  std::vector<cc::DirectoryModelConfig> dconfigs;
};

struct SimContext {
  // Simulation configuration.
  SimConfig simconfig;
  // Kernel associated with current simulation instance.
  cc::kernel::Kernel* k = nullptr;
};

class SimTop : cc::kernel::TopModule {
 public:
  SimTop(const SimContext& simcontext)
      : cc::kernel::TopModule(simcontext.k, simcontext.simconfig.name)
      , simcontext_(simcontext) {
    build();
  }

  // Return simulation configuration.
  SimContext simcontext() const { return simcontext_; }

  // Invoke simulation.
  void run() { k()->run(); }

 protected:
  // Construct top-level simulation enviornment and associated
  // collateral.
  void build() {
    const SimConfig& simconfig = simcontext_.simconfig;

    // Construct NOC instance
    noc_ = new cc::NocModel(k(), simconfig.nocconfig);
    add_child_module(noc_);

    // Construct L2 instance
    for (const cc::L2CacheModelConfig& l2cfg : simconfig.l2configs) {
      cc::L2CacheModel* l2c = new cc::L2CacheModel(k(), l2cfg);
      l2c->set_noc_ep(noc_->get_input(0));
      add_child_module(l2c);
      l2cs_.push_back(l2c);
    }

    // Construct directory
    for (const cc::DirectoryModelConfig& dconfig : simconfig.dconfigs) {
      cc::DirectoryModel* dir = new cc::DirectoryModel(k(), dconfig);
      add_child_module(dir);
      dirs_.push_back(dir);
    }
  }

  void elab() override {
    // Construct directory mapper object;
    cc::DirectoryModel* dir = dirs_.front();
    cc::DirectoryMapper* dm = new cc::SingleDirectoryMapper(dir);
    for (cc::L2CacheModel* l2c : l2cs_) {
      l2c->set_dm(dm);
    }
  }

  void drc() override {
  }
 private:
  SimContext simcontext_;

  // Model L2 Cache instances
  std::vector<cc::L2CacheModel*> l2cs_;
  // NOC Model instance.
  cc::NocModel* noc_;
  // Directories(s)
  std::vector<cc::DirectoryModel*> dirs_;
};

int main(int argc, char** argv) {
  // Simulation configuration.
  SimConfig simconfig;
  simconfig.name = "top";
  simconfig.nocconfig = cc::NocModelConfig();
  

  cc::ProtocolBuilder* p = cc::ProtocolBuilderRegistry::build("moesi");
  
  cc::L2CacheModelConfig l2config;
  l2config.name = "l2cache";
  l2config.cconfig = cc::CacheModelConfig();
  l2config.protocol = p->create_l2();

  cc::L1CacheModelConfig l1config;
  l1config.name = "l1cache";
  l1config.protocol = p->create_l1();
  l1config.cconfig = cc::CacheModelConfig();

  cc::DirectoryModelConfig dconfig;
  dconfig.name = "directory";
  dconfig.protocol = p->create_dir();
  dconfig.cconfig = cc::CacheModelConfig();

  cc::ProgrammaticStimulus* stim = new cc::ProgrammaticStimulus;

  stim->push_back(cc::kernel::Time{100}, cc::Command{cc::Command::Load, 0});
  stim->push_back(cc::kernel::Time{100}, cc::Command{cc::Command::Load, 0});
  l1config.stim = stim;
  l2config.l1configs.push_back(l1config);
  simconfig.l2configs.push_back(l2config);
  simconfig.dconfigs.push_back(dconfig);

  SimContext simcontext;
  simcontext.k = new cc::kernel::Kernel;
  simcontext.simconfig = simconfig;

  // TODO: pass object ownership to constructing agent.
  
  SimTop* top = new SimTop(simcontext);
  top->run();

  delete simcontext.k;

  return 0;
}
