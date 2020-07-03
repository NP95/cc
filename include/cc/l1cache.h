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
#include "kernel.h"

namespace cc {

class Stimulus;
class Message;
class MessageQueue;
template<typename> class Arbiter;
class Cpu;
class L2CacheModel;

//
//
struct L1CacheModelConfig {
  // L1 Cache Model name
  std::string name = "l1cache";

  // Pointer to the transaction source instance for the current l1
  // cache instance (models the notion of a microprocessor
  // periodically emitting load/store instructions to memory).
  Stimulus* stim = nullptr;

  // Number of slots in the command queue.
  std::size_t l1_cmdq_slots_n = 3;

  // Number of credits/slots reserved to the L2 cache command queue.
  std::size_t l2_cmdq_credits_n = 3;

  // LD/ST pipe flush penalty (cycles)
  std::size_t ldst_flush_penalty_n = 3;
};

//
//
class L1CacheModel : public kernel::Agent<const Message*> {
  class MainProcess;
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
  L1CacheModelConfig config() const { return config_; }

  // Set parent L2Cache (Elaboration-Phase)
  void set_parent(L2CacheModel* l2cache) { l2cache_ = l2cache; }
  
 protected:
  virtual void elab() override;
  virtual void drc() override;
 private:
  void build();
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
  // TODO
  // Pointer to parent L2.
  L2CacheModel* l2cache_;
  // Cache configuration.
  L1CacheModelConfig config_;
  
};

} // namespace cc

#endif
