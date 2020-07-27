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
#include "ccntrl.h"
#include "cpucluster.h"
#include "dir.h"
#include "llc.h"
#include "mem.h"
#include "noc.h"
#include "protocol.h"
#include "stimulus.h"

namespace cc {

class SocTop : public kernel::TopModule {
 public:
  SocTop(kernel::Kernel* k, const SocConfig& cfg)
      : TopModule(k, cfg.name) {
    build(cfg);
  }

  ~SocTop() {
    // Destroy child instances
    for (CpuCluster* cc : ccs_) {
      delete cc;
    }
    //
    for (DirModel* dm : dms_) {
      delete dm;
    }
    for (LLCModel* llc : llcs_) {
	delete llc;
    }
    for (MemCntrlModel* mm : mms_) {
      delete mm;
    }
    delete dm_;
    delete noc_;
    delete stimulus_;
  }

 private:
  void build(const SocConfig& cfg) {
    // Construct stimulus (as module)
    stimulus_ = stimulus_builder(k(), cfg.scfg);
    add_child_module(stimulus_);

    // Construct interconnect:
    noc_ = new NocModel(k(), cfg.noccfg);
    add_child_module(noc_);

    // Construct memory controller (s)
    MemCntrlModel* mm = new MemCntrlModel(k());
    noc_->register_agent(mm);
    add_child_module(mm);
    mms_.push_back(mm);

    // Construct child CPU clusters
    for (const CpuClusterConfig& cccfg : cfg.ccls) {
      CpuCluster* cpuc = new CpuCluster(k(), cccfg, stimulus_);
      // NOC end point is the coherence controller within the CPU
      // cluster; not the CPU cluster itself.
      noc_->register_agent(cpuc->cc());
      add_child_module(cpuc);
      ccs_.push_back(cpuc);
    }

    // Construct child directories
    for (const DirModelConfig& dcfg : cfg.dcfgs) {
      DirModel* dm = new DirModel(k(), dcfg);
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
        // Dir is Null filter, it must therefore interact
        // directly with the memory controller to initiate
        // lookups/writebacks to main memory.
        // TODO
      }
    }
  }

  void elab() override {
    // Bind interconnect:
    for (CpuCluster* cpuc : ccs_) {
      NocPort* port = noc_->get_agent_port(cpuc->cc());
      if (port == nullptr) {
        // Cannot find port for agent; must call register_agent on the
        // NOC instance before attempting to bind to the port.
        LogMessage msg("Cannot bind to port on: ");
        msg.append(cpuc->cc()->path());
        msg.level(Level::Fatal);
        log(msg);
      }
      // NOC -> CC message queue
      MessageQueue* noc_cc__msg_q = cpuc->noc_cc__msg_q();
      port->set_egress(noc_cc__msg_q->construct_proxy());
      // CC -> NOC message queue
      MessageQueue* ingress_mq = port->ingress();
      cpuc->set_cc_noc__msg_q(ingress_mq->construct_proxy());
    }
    for (DirModel* dm : dms_) {
      NocPort* dm_port = noc_->get_agent_port(dm);
      // NOC -> DIR message queue
      dm_port->set_egress(dm->endpoint()->construct_proxy());
      // DIR -> NOC message queue
      MessageQueue* dm_port_ingress = dm_port->ingress();
      dm->set_dir_noc__msg_q(dm_port_ingress->construct_proxy());

      const DirModelConfig& cfg = dm->config();
      if (!cfg.is_null_filter) {
        LLCModel* llc = dm->llc();
        // Bind memory controller
        llc->set_mc(mms_.front());
        // Bind directory
        llc->set_dir(dm);
        //
        // Bind associated LLC to NOC
        NocPort* llc_port = noc_->get_agent_port(llc);
        // NOC -> LLC
        llc_port->set_egress(llc->endpoint()->construct_proxy());
        // LLC -> NOC
        MessageQueue* llc_port_ingress = llc_port->ingress();
        llc->set_llc_noc__msg_q(llc_port_ingress->construct_proxy());
      }
    }
    for (MemCntrlModel* mm : mms_) {
      NocPort* port = noc_->get_agent_port(mm);
      // NOC -> MEM
      port->set_egress(mm->endpoint()->construct_proxy());
      // MEM -> NOC
      mm->set_mem_noc__msg_q(port->ingress()->construct_proxy());
    }
    // Construct directory mapper
    dm_ = new SingleDirMapper(*dms_.begin());
    for (CpuCluster* cc : ccs_) {
      cc->set_dm(dm_);
    }
    // Register CC <-> LLC ports
    for (DirModel* dm : dms_) {
      LLCModel* llc = dm->llc();
      for (CpuCluster* cc : ccs_) {
        llc->register_cc(cc);
      }
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
  DirMapper* dm_;
  // NOC/Interconnect instance
  NocModel* noc_ = nullptr;
  // Directory model instance
  std::vector<DirModel*> dms_;
  // LLC Models 1-to-1 relationship with non-Null Filter directories.
  std::vector<LLCModel*> llcs_;
  // CPU Cluster instances
  std::vector<CpuCluster*> ccs_;
  // Memory Controller instances.
  std::vector<MemCntrlModel*> mms_;
  // Stimulus "module" instance.
  Stimulus* stimulus_;
};

Soc::Soc(const SocConfig& cfg) { build(cfg); }

Soc::~Soc() {
  delete kernel_;
  delete top_;
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

void Soc::finalize() { kernel_->invoke_fini(); }

void Soc::build(const SocConfig& cfg) {
  // Construct simulation kernel
  kernel_ = new kernel::Kernel(1);
  // Construct top-level instance.
  top_ = new SocTop(kernel_, cfg);
}

Soc* construct_soc(const SocConfig& soccfg) { return new Soc(soccfg); }

ProtocolBuilder* construct_protocol_builder(const std::string& name) {
  return ProtocolBuilderRegistry::build(name);
}

}  // namespace cc
