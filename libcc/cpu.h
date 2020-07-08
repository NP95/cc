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

#ifndef CC_LIBCC_CPU_H
#define CC_LIBCC_CPU_H

#include <set>

#include "kernel.h"
#include "sim.h"
#include "cc/cfgs.h"
#include "cc/msg.h"
#include "cc/primitives.h"

namespace cc {

// Forwards
class Message;
class L1CacheModel;
class MessageQueue;

//
//
class CpuL1__CmdMsg : public Message {
 public:
  enum Opcode { Load, Store };

  CpuL1__CmdMsg(Transaction* t) : Message(t, MessageClass::CpuL1__CmdMsg) {}

  Opcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }

  void set_addr(addr_t addr) { addr_ = addr; }
  void set_opcode(Opcode opcode) { opcode_ = opcode; }

 private:
  addr_t addr_;
  Opcode opcode_;
};

//
//
class L1Cpu__RspMsg : public Message {
 public:
  L1Cpu__RspMsg(Transaction* t) : Message(t, MessageClass::L1Cpu__RspMsg) {}
};

//
//
class Cpu : public Agent {
  class ProducerProcess;
  class ConsumerProcess;
  
  friend class CpuCluster;
 public:
  Cpu(kernel::Kernel* k, const CpuConfig& config);

  // Cache configuration
  const CpuConfig& config() const { return config_; }

 protected:

  // Accessors;
  MessageQueue* cpu_l1__cmd_q() const { return cpu_l1__cmd_q_; }
  

  // Construction:
  void build();

  // Elaboration:
  void elab() override;
  // Set parent L1 cache instance.
  void set_l1c(L1CacheModel* l1c) { l1c_ = l1c; }
  // Set CPU -> L1 command queue
  void set_cpu_l1__cmd_q(MessageQueue* mq) { cpu_l1__cmd_q_ = mq; }

  // Design Rule Check (DRC):
  void drc() override;

 private:
  // 
  Stimulus* stimulus_ = nullptr;
  // CPU -> L1 message queue.
  MessageQueue* cpu_l1__cmd_q_ = nullptr;
  // Producer thread of execution.
  ProducerProcess* pp_ = nullptr;
  // Consumer thread of execution.
  ConsumerProcess* cp_ = nullptr;
  // L1Cache instance.
  L1CacheModel* l1c_ = nullptr;
  // CPU Configuration.
  CpuConfig config_;
};

}  // namespace cc

#endif
