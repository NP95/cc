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

#ifndef CC_LIBCC_CPUCLUSTER_H
#define CC_LIBCC_CPUCLUSTER_H

#include "kernel.h"
#include "cfgs.h"
#include "primitives.h"

namespace cc {

class Message;
class CacheController;
class L2CacheModel;
class L1CacheModel;
class Cpu;

class CpuCluster : public Agent {
  friend class SocTop;
 public:
  CpuCluster(kernel::Kernel* k, const CpuClusterCfg& cfg);

  //
  const CpuClusterCfg& config() const { return config_; }

  // Get NOC -> CC message queue instance (CC owned)
  MessageQueue* noc_cc__msg_q() const;

 private:
  // Construction
  void build();
  // Elaboration
  void elab();
  // Set CC -> NOC message queue instance (NOC owned)
  void set_cc_noc__msg_q(MessageQueue* mq);
  // Set directory mapper
  void set_dm(DirectoryMapper* dm);
  
  // Design Rule Check (DRC)
  void drc();
  
  //
  CacheController* cc_ = nullptr;
  //
  L2CacheModel* l2c_ = nullptr;
  //
  std::vector<L1CacheModel*> l1cs_;
  //
  std::vector<Cpu*> cpus_;
  //
  CpuClusterCfg config_;
};

} // namespace cc

#endif
