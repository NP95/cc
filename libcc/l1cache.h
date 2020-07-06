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

#ifndef CC_INCLUDE_CC_L1CACHE_H
#define CC_INCLUDE_CC_L1CACHE_H

#include <string>
#include "cc/cfgs.h"
#include "kernel.h"

namespace cc {

// Forwards:
class Stimulus;
class Message;
class MessageQueue;
template<typename> class Arbiter;
class Cpu;
class L2CacheModel;
template<typename> class CacheModel;
class L1LineState;

//
//
class L1CacheModel : public kernel::Agent<const Message*> {
  class MainProcess;

  friend class MainProcess;
  friend class L2CacheModel;
  
 public:
  // End-points
  enum EndPoints : kernel::end_point_id_t {
    // CPU Response: return path back to originating CPU.
    CpuRsp,
    // Command Request: ingress commands from parent L2Cache.
    L1CmdReq,
    // Command Response: ingress command response from parent L2Cache.
    L1CmdRsp
  };
  
  L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config);
  virtual ~L1CacheModel();

  // Return current L1 configuration.
  const L1CacheModelConfig& config() const { return config_; }
 protected:
  // Pointer to current arbiter child instance.
  Arbiter<const Message*>* arb() const { return arb_; }
  // Pointer to current CPU child instance.
  Cpu* cpu() const { return cpu_; }
  // Pointer to owning L2Cache
  L2CacheModel* l2cache() const { return l2cache_; }
  // Pointer to command request message queue.
  MessageQueue* msgreqq() const { return msgreqq_; }

  // Set parent L2Cache (Elaboration-Phase)
  void set_parent(L2CacheModel* l2cache) { l2cache_ = l2cache; }
  
  virtual void elab() override;
  virtual void drc() override;
 private:
  // Construct l1cache instance
  void build();
  // Accessors:
  CacheModel<L1LineState*>* cache() const { return cache_; }
  
  // L1 Cache stimulus (models the concept of a processor data path
  // emitting instructions into the cache as part of a programs
  // execution).
  Cpu* cpu_;
  // Coherence message request queue 
  MessageQueue* msgreqq_;
  // Coherency message response queue
  MessageQueue* msgrspq_;
  // Message servicing arbiter.
  Arbiter<const Message*>* arb_;
  // Main process of execution.
  MainProcess* main_;
  // Cache Instance
  CacheModel<L1LineState*>* cache_;
  // Pointer to parent L2.
  L2CacheModel* l2cache_;
  // Cache configuration.
  L1CacheModelConfig config_;
  
};

} // namespace cc

#endif
