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

#include "cc/soc.h"
#include "cc/cfgs.h"
#include "cc/kernel.h"
#include "noc.h"
#include "cpucluster.h"
#include "dir.h"
#include "llc.h"
#include "mem.h"

namespace cc {

class SocTop : public kernel::TopModule {
 public:
  SocTop(kernel::Kernel* k, const SocCfg& cfg)
      : TopModule(k, cfg.name), config_(cfg) {
    build();
  }

  ~SocTop() {
    // Destroy child instances
    for (CpuCluster* cc : ccs_) {
      delete cc;
    }
    //
    for (DirectoryModel* dm : dms_) {
      delete dm;
    }
    delete noc_;
  }

  const SocCfg& config() const { return config_; }

 private:
  void build() {
    // Construct interconnect:
    noc_ = new NocModel(k(), config_.noccfg);
    add_child_module(noc_);

    // Construct memory controller (s)
    MemCntrlModel* mm = new MemCntrlModel(k());
    noc_->register_agent(mm);
    add_child_module(mm);
    mms_.push_back(mm);
    
    // Construct child CPU clusters
    for (const CpuClusterCfg& cccfg : config_.ccls) {
      CpuCluster* cc = new CpuCluster(k(), cccfg);
      noc_->register_agent(cc);
      add_child_module(cc);
      ccs_.push_back(cc);
    }

    // Construct child directories
    for (const DirectoryModelConfig& dcfg : config_.dcfgs) {
      DirectoryModel* dm = new DirectoryModel(k(), dcfg);
      noc_->register_agent(dm);
      add_child_module(dm);
      dms_.push_back(dm);

      if (!dcfg.is_null_filter) {
        // Construct corresponding LLC
        LLCModel* llc = new LLCModel(k(), dcfg.llcconfig);
        mm->register_agent(llc);
        noc_->register_agent(llc);
        add_child_module(llc);
        llcs_.push_back(llc);
        // Bind DIR instance to associated LLC.
        dm->set_llc(llc);
      } else {
        // Directory is Null filter, it must therefore interact
        // directly with the memory controller to initiate
        // lookups/writebacks to main memory.
        // TODO
      }
    }
  }

  void elab() override {
    // Bind interconnect:
    for (CpuCluster* cc : ccs_) {
      NocPort* port = noc_->get_agent_port(cc);
      // NOC -> CC message queue
      port->set_egress(cc->noc_cc__msg_q());
      // CC -> NOC message queue
      cc->set_cc_noc__msg_q(port->ingress());
    }
    for (DirectoryModel* dm : dms_) {
      NocPort* port = noc_->get_agent_port(dm);
      // NOC -> DIR message queue
      port->set_egress(dm->noc_dir__msg_q());
      // DIR -> NOC message queue
      dm->set_dir_noc__msg_q(port->ingress());

      const DirectoryModelConfig& cfg = dm->config();
      if (!cfg.is_null_filter) {
        LLCModel* llc = dm->llc();
        // Bind memory controller
        llc->set_mc(mms_.front());
        // Bind associated LLC to NOC
        NocPort* port = noc_->get_agent_port(llc);
        // NOC -> LLC
        port->set_egress(llc->noc_llc__msg_q());
        // LLC -> NOC
        llc->set_llc_noc__msg_q(port->ingress());
      }
    }
    for (MemCntrlModel* mm : mms_) {
      NocPort* port = noc_->get_agent_port(mm);
      // NOC -> MEM
      port->set_egress(mm->noc_mem__msg_q());
      // MEM -> NOC
      mm->set_mem_noc__msg_q(port->ingress());
    }
    // Construct directory mapper
    dm_ = new SingleDirectoryMapper(*dms_.begin());
    for (CpuCluster* cc : ccs_) {
      cc->set_dm(dm_);
    }
  }

  void drc() override {
    if (dms_.empty()) {
      LogMessage msg("No directories have been defined.", Level::Fatal);
      log(msg);
    }

    if (ccs_.empty()) {
      LogMessage msg("No CPU clusters have been defined.", Level::Fatal);
      log(msg);
    }
  }

  // Directory Mapper instance
  DirectoryMapper* dm_;
  // NOC/Interconnect instance
  NocModel* noc_ = nullptr;
  // Directory model instance
  std::vector<DirectoryModel*> dms_;
  // LLC Models 1-to-1 relationship with non-Null Filter directories.
  std::vector<LLCModel*> llcs_;
  // CPU Cluster instances
  std::vector<CpuCluster*> ccs_;
  // Memory Controller instances.
  std::vector<MemCntrlModel*> mms_;
  // Soc configuration
  SocCfg config_;
};

Soc::Soc(const SocCfg& cfg) {
  build(cfg);
}

Soc::~Soc() {
  delete kernel_;
}

void Soc::initialize() {
  kernel_->invoke_elab();
  kernel_->invoke_drc();
  kernel_->invoke_init();
}

void Soc::run() {
  const kernel::RunMode r = kernel::RunMode::ToExhaustion;
  const kernel::Time t;
  kernel_->invoke_run(r, t);
}

void Soc::finalize() {
  kernel_->invoke_fini();
}

void Soc::build(const SocCfg& cfg) {
  // Construct simulation kernel
  kernel_ = new kernel::Kernel(1);
  // Construct top-level instance.
  top_ = new SocTop(kernel_, cfg);
}

Soc* construct_soc(const SocCfg& soccfg) {
  return new Soc(soccfg);
}

} // namespace
