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

#ifndef CC_LIBCC_L2CACHE_H
#define CC_LIBCC_L2CACHE_H

#include <vector>

#include "cfgs.h"
#include "kernel.h"
#include "primitives.h"
#include "msg.h"
#include "l2cache_enum.h"

namespace cc {

// Forwards:
class L1CacheModel;
class MessageQueue;
template <typename>
class Arbiter;
class Message;
class L2LineState;
template <typename>
class CacheModel;
class CacheController;

enum class L2Opcode {
  GetS
};

//
//
class L1L2__CmdMsg : public Message {
 public:
  L1L2__CmdMsg(Transaction* t) : Message(t, MessageClass::L1L2__CmdMsg) {}

  L2L1Opcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }
  std::string to_string_short() const override;
  std::string to_string() const override;

  void set_addr(addr_t addr) { addr_ = addr; }
  void set_opcode(L2L1Opcode opcode) { opcode_ = opcode; }
  
 private:
  addr_t addr_;
  L2L1Opcode opcode_;
};

//
//
class L2CacheModel : public Agent {
  class MainProcess;
  friend class CpuCluster;

 public:
  L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config);
  virtual ~L2CacheModel();

  L2CacheModelConfig config() const { return config_; }

  // L1 Cache Command Queue (n)
  MessageQueue* l1_l2__cmd_q(std::size_t n) const { return l1_l2__cmd_qs_[n]; }

 protected:

  // Accessors:
  // Pointer to module arbiter instance.
  Arbiter<const Message*>* arb() const { return arb_; }
  // Point to module cache instance.
  CacheModel<L2LineState*>* cache() const { return cache_; }
  // Protocol instance
  L2CacheModelProtocol* protocol() const { return protocol_; }
  // L2 -> CC command queue
  MessageQueue* l2_cc__cmd_q() const { return l2_cc__cmd_q_; }
  // CC -> L2 response queue
  MessageQueue* cc_l2__rsp_q() const { return cc_l2__rsp_q_; }

  // Construction:
  void build();
  // Add L1 cache child.
  void add_l1c(L1CacheModel* l1c);

  // Elaboration:
  virtual void elab() override;
  // Set parent cache controlle
  void set_cc(CacheController* cc) { cc_ = cc; }
  // Set L2 -> CC command queue.
  void set_l2_cc__cmd_q(MessageQueue* mq) { l2_cc__cmd_q_ = mq; }
  
  // Design Rule Check (DRC) callback
  virtual void drc() override;

 private:

  // L2 Cache Configuration.
  L2CacheModelConfig config_;
  // Child L1 Caches
  std::vector<L1CacheModel*> l1cs_;
  // L1 -> L2 Command Request
  std::vector<MessageQueue*> l1_l2__cmd_qs_;
  // L2 -> CC Command Queue
  MessageQueue* l2_cc__cmd_q_ = nullptr;
  // CC -> L2 Response Queue
  MessageQueue* cc_l2__rsp_q_ = nullptr;
  // Queue selection arbiter
  Arbiter<const Message*>* arb_ = nullptr;
  // Cache Instance
  CacheModel<L2LineState*>* cache_ = nullptr;
  // Cache Controller instance
  CacheController* cc_ = nullptr;
  // L1 cache protocol
  L2CacheModelProtocol* protocol_ = nullptr;
  // Main process of execution.
  MainProcess* main_;
};

}  // namespace cc

#endif
