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

#ifndef CC_LIBCC_L1CACHE_H
#define CC_LIBCC_L1CACHE_H

#include <string>

#include "cc/cfgs.h"
#include "primitives.h"
#include "l2cache.h"

namespace cc {

// Forwards:
class Stimulus;
class Message;
class MessageQueue;
template <typename>
class Arbiter;
class Cpu;
class L2CacheModel;
template <typename> class CacheModel;
class L1LineState;


enum class L1CacheOpcode {
  CpuLoad,
  CpuStore
};

//
//
class L1CmdMsg : public Message {
 public:
  L1CmdMsg();

  //
  std::string to_string() const override;

  //
  L1CacheOpcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }

  //
  void set_opcode(L1CacheOpcode opcode) { opcode_ = opcode; }
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  L1CacheOpcode opcode_;
  addr_t addr_;
};

//
//
class L1RspMsg : public Message {
 public:
  L1RspMsg();
};

//
//
class L1CacheModel : public Agent {
  class MainProcess;
  friend class CpuCluster;
 public:
  L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config);
  virtual ~L1CacheModel();

  // Return current L1 configuration.
  const L1CacheModelConfig& config() const { return config_; }
  // CPU -> l1 command queue
  MessageQueue* cpu_l1__cmd_q() const { return cpu_l1__cmd_q_; }
  // L1 -> CPU response queue
  MessageQueue* l1_cpu__rsp_q() const { return l1_cpu__rsp_q_; }
  // L1 -> L2 command queue
  MessageQueue* l1_l2__cmd_q() const { return l1_l2__cmd_q_; }
  // L2 -> L1 response queeu
  MessageQueue* l2_l1__rsp_q() const { return l2_l1__rsp_q_; }

 protected:
  // Accessors:
  // Pointer to current arbiter child instance.
  MessageQueueArbiter* arb() const { return arb_; }
  // Pointer to current CPU child instance.
  Cpu* cpu() const { return cpu_; }
  // Pointer to owning L2Cache
  L2CacheModel* l2c() const { return l2c_; }
  // Accessors:
  CacheModel<L1LineState*>* cache() const { return cache_; }
  // Protocol
  L1CacheModelProtocol* protocol() const { return protocol_; }
  
  // Build Phase:
  void build();

  // Elaboration Phase:
  virtual void elab() override;
  // Set parent L2Cache (Elaboration-Phase)
  void set_l2c(L2CacheModel* l2c) { l2c_ = l2c; }
  // Set CPU (Elaboration-Phase)
  void set_cpu(Cpu* cpu) { cpu_ = cpu; }
  // Set L1 -> L2 Command Queue
  void set_l1_l2__cmd_q(MessageQueue* mq) { l1_l2__cmd_q_ = mq; }
  // Set L1 -> CPU Response Queue
  void set_l1_cpu__rsp_q(MessageQueue* mq) { l1_cpu__rsp_q_ = mq; }
  
  // Design Rule Check (DRC) Phase
  virtual void drc() override;

 private:
  // L1 Cache stimulus (models the concept of a processor data path
  // emitting instructions into the cache as part of a programs
  // execution).
  Cpu* cpu_ = nullptr;
  // CPU -> L1 Command Queue (L1 owned)
  MessageQueue* cpu_l1__cmd_q_ = nullptr;
  // L1 -> L2 Command Queue (L2 owned)
  MessageQueue* l1_l2__cmd_q_ = nullptr;
  // L2 -> L1 Response Queue (L1 owned)
  MessageQueue* l2_l1__rsp_q_ = nullptr;
  // L2 -> L1 Command Queue (L1 owned)
  MessageQueue* l2_l1__cmd_q_ = nullptr;
  // L1 -> CPU Response Queue (CPU owned)
  MessageQueue* l1_cpu__rsp_q_ = nullptr;
  // Message servicing arbiter.
  MessageQueueArbiter* arb_ = nullptr;
  // Main process of execution.
  MainProcess* main_ = nullptr;
  // Cache Instance
  CacheModel<L1LineState*>* cache_ = nullptr;
  // Pointer to parent L2.
  L2CacheModel* l2c_ = nullptr;
  // L1 cache protocol
  L1CacheModelProtocol* protocol_ = nullptr;
  // Cache configuration.
  L1CacheModelConfig config_;
};

}  // namespace cc

#endif
