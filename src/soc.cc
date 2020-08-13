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

SocTop::SocTop(kernel::Kernel* k, const SocConfig& cfg)
    : TopModule(k, cfg.name), cfg_(cfg) {
  build(cfg);
}

SocTop::~SocTop() {
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

void SocTop::build(const SocConfig& cfg) {
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

bool SocTop::elab() {
  // Two-pass elaboration discussion:
  //
  // First pass:
  //
  // Perform port-binding between Agents and NOC. This from this,
  // various NOC related Message Queues may/are constructed and
  // registered as end-points within their owning agents.
  //
  // Second pass:
  //
  // Traverse NOC clients in the SOC and resize associated Message
  // Queues so that they have sufficient space to handle all credits
  // previously allocated to them. For this to occur, the mapping
  // between end-point to agent Message Queue in the endpoint
  // ("register_endpoint") must have taken place.
  //
  // Notes: Multiple passes is perhaps undesirable, however it is a
  // simplification overall since this prevents the uncessary
  // proliferation of elab-states and/or the enforcement of
  // strict-ordering on the overall, global elaobration process.
  
  bool do_retry = false;
  switch (elab_pass_++) {
    case 0: {
      // First pass; bind ports
      elab_bind_ports();
      do_retry = true;
    } break;
    case 1: {
      // Second pass; update credit counters.
      elab_credit_counts();
    } break;
    default: {
      // Never reached.
      do_retry = false;
    } break;
  }
  return do_retry;
}

void SocTop::elab_bind_ports() {
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
    port->set_egress(cpuc->noc_cc__msg_q());
    // CC -> NOC message queue
    cpuc->set_cc_noc__msg_q(port->ingress());
  }
  for (DirModel* dm : dms_) {
    NocPort* dm_port = noc_->get_agent_port(dm);
    // NOC -> DIR message queue
    dm_port->set_egress(dm->endpoint());
    // DIR -> NOC message queue
    dm->set_dir_noc__msg_q(dm_port->ingress());

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
      llc_port->set_egress(llc->endpoint());
      // LLC -> NOC
      llc->set_llc_noc__msg_q(llc_port->ingress());
    }
  }
  for (MemCntrlModel* mm : mms_) {
    NocPort* port = noc_->get_agent_port(mm);
    // NOC -> MEM
    port->set_egress(mm->endpoint());
    // MEM -> NOC
    mm->set_mem_noc__msg_q(port->ingress());
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

void SocTop::elab_credit_counts() {
  MessageQueue* mq = nullptr;
  std::size_t credits_n = 16;

  // Map of credits allocated to a given Message Queue.
  std::map<MessageQueue*, std::size_t> mqcredits;

  // Update CPU Cluster to Directory credit paths.
  for (CpuCluster* cpuc : ccs_) {
    CCModel* cc = cpuc->cc();

    // Register edge from Cpu Cluster to directories
    for (DirModel* dm : dms_) {
      // Coherence start message
      cc->register_credit_counter(MessageClass::CohSrt, dm, credits_n);
      mq = dm->mq_by_msg_cls(MessageClass::CohSrt);
      if (mq != nullptr) { mqcredits[mq] += credits_n; }

      // Coherence command message
      cc->register_credit_counter(MessageClass::CohCmd, dm, credits_n);
      mq = dm->mq_by_msg_cls(MessageClass::CohCmd);
      if (mq != nullptr) { mqcredits[mq] += credits_n; }
    }

    // Register edge from Cpu Cluster to all other Cpu clusters (Dt
    // transfers).
    for (CpuCluster* cpuc_dest : ccs_) {
      // No edge from self to self. (A Cpu Cluster will never send
      // a Dt to itself).
      if (cpuc_dest == cpuc) continue;

      cc->register_credit_counter(MessageClass::Dt, cpuc_dest, credits_n);
      mq = cpuc_dest->mq_by_msg_cls(MessageClass::Dt);
      if (mq != nullptr) { mqcredits[mq] += credits_n; }
    }
  }

  // Set Directory to CPU Cluster (Snoops) credit paths
  for (DirModel* dm : dms_) {

    for (CpuCluster* cpuc : ccs_) {
      CCModel* cc = cpuc->cc();
      
      dm->register_credit_counter(MessageClass::CohSnp, cc, credits_n);
      mq = cc->mq_by_msg_cls(MessageClass::CohSnp);
      if (mq != nullptr) { mqcredits[mq] += credits_n; }
    }

  }

  // Now that credit counters have been set, update the capacity of
  // the destination Message Queues.
  for (const auto& mqcredit : mqcredits) {
    mq = mqcredit.first;
    const std::size_t credits = mqcredit.second;
    mq->resize(credits);
  }
}

void SocTop::drc() {
  if (dms_.empty()) {
    LogMessage msg("No directories have been defined.", Level::Fatal);
    log(msg);
  }

  if (ccs_.empty()) {
    LogMessage msg("No CPU clusters have been defined.", Level::Fatal);
    log(msg);
  }
}

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
