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

#include "cache.h"
#include "cc/cfgs.h"
#include "msg.h"
#include "primitives.h"
#include "sim.h"

namespace cc {

// Forwards:
class Cpu;
class L1CacheModel;
class L2CacheModel;
class L1LineState;
class CoherenceAction;

enum class L1CacheOpcode { CpuLoad, CpuStore };

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
class L1CmdRspMsg : public Message {
 public:
  L1CmdRspMsg();

  //
  std::string to_string() const override;
};

// Cache data type
using L1Cache = CacheModel<L1LineState*>;
// Cache Set data type
using L1CacheSet = L1Cache::Set;
// Cache Line Iterator type.
using L1CacheLineIt = L1Cache::LineIterator;

// clang-format off
#define L1OPCODE_LIST(__func)                   \
  __func(TableInstall)                          \
  __func(TableGetCurrentState)                  \
  __func(TableRemove)                           \
  __func(TableMqUnblockAll)                     \
  __func(TableMqAddToBlockedList)               \
  __func(WaitOnMsg)                             \
  __func(WaitNextEpochOrWait)                   \
  __func(MqSetBlocked)                          \
  __func(MsgConsume)                            \
  __func(MsgL1CmdExtractAddr)                   \
  __func(InstallLine)                           \
  __func(InvokeCoherenceAction)
// clang-format on

enum class L1Opcode {
#define __declare_opcode(__name) __name,
  L1OPCODE_LIST(__declare_opcode)
#undef __declare_opcode
};

//
//
const char* to_string(L1Opcode opcode);

//
//
class L1Command {
  friend class L1CommandBuilder;

 public:
  L1Command(L1Opcode opcode) : opcode_(opcode) {}
  virtual void release() const { delete this; }

  std::string to_string() const;

  L1Opcode opcode() const { return opcode_; }
  CoherenceAction* action() const { return oprands.coh.action; }

 private:
  virtual ~L1Command();
  //
  union {
    struct {
      CoherenceAction* action;
    } coh;
  } oprands;
  //
  L1Opcode opcode_;
};

//
//
class L1CommandBuilder {
 public:
  static L1Command* from_opcode(L1Opcode opcode);

  static L1Command* from_action(CoherenceAction* action);
};

//
//
class L1CommandList {
  using vector_type = std::vector<L1Command*>;

 public:
  using const_iterator = vector_type::const_iterator;

  L1CommandList() = default;
  ~L1CommandList();

  

  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  void push_back(L1Command* cmd);

 private:
  std::vector<L1Command*> cmds_;
};

//

//
class L1TState {
 public:
  L1TState() = default;

  // Destruct/Return to pool
  void release() { delete this; }

  // Get current cache line
  L1LineState* line() const { return line_; }

  // Set current cache line
  void set_line(L1LineState* line) { line_ = line; }

  void add_blocked_mq(MessageQueue* mq) { mqs_.push_back(mq); }

  const std::vector<MessageQueue*>& mq() const { return mqs_; }

 private:
  // Cache line on which current transaction is executing. (Can
  // otherwise be recovered from the address, but this simply saves
  // the lookup into the cache structure).
  L1LineState* line_ = nullptr;
  //
  std::vector<MessageQueue*> mqs_;
};

//
//
using L1TTable = TransactionTable<L1TState*>;

//
//
class L1CacheContext {
 public:
  L1CacheContext() = default;
  ~L1CacheContext();

  //
  MQArbTmt t() const { return t_; }
  const Message* msg() const { return mq_->peek(); }
  MessageQueue* mq() const { return mq_; }
  L1CacheModel* l1cache() const { return l1cache_; }
  bool owns_line() const { return owns_line_; }
  L1LineState* line() const { return line_; }

  //
  void set_t(MQArbTmt t) { t_ = t; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_l1cache(L1CacheModel* l1cache) { l1cache_ = l1cache; }
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }
  void set_line(L1LineState* line) { line_ = line; }

 private:
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  //
  bool owns_line_ = false;
  //
  L1LineState* line_ = nullptr;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // L1 cache instance
  L1CacheModel* l1cache_ = nullptr;
};

//
//
class L1CacheModel : public Agent {
  class MainProcess;

  friend class CpuCluster;
  friend class L1CommandInterpreter;

 public:
  L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config);
  virtual ~L1CacheModel();

  // Return current L1 configuration.
  const L1CacheModelConfig& config() const { return config_; }
  // Accessors:
  CacheModel<L1LineState*>* cache() const { return cache_; }
  // CPU -> l1 command queue
  MessageQueue* cpu_l1__cmd_q() const { return cpu_l1__cmd_q_; }
  // L1 -> CPU response queue
  MessageQueueProxy* l1_cpu__rsp_q() const { return l1_cpu__rsp_q_; }
  // L1 -> L2 command queue
  MessageQueueProxy* l1_l2__cmd_q() const { return l1_l2__cmd_q_; }
  // L2 -> L1 response queeu
  MessageQueue* l2_l1__rsp_q() const { return l2_l1__rsp_q_; }

 protected:
  // Accessors:
  // Pointer to current arbiter child instance.
  MQArb* arb() const { return arb_; }
  // Pointer to current CPU child instance.
  Cpu* cpu() const { return cpu_; }
  // Pointer to owning L2Cache
  L2CacheModel* l2c() const { return l2c_; }
  // Protocol
  L1CacheModelProtocol* protocol() const { return protocol_; }
  // Transaction table.
  L1TTable* tt() const { return tt_; }

  // Build Phase:
  void build();

  // Elaboration Phase:
  virtual void elab() override;
  // Set parent L2Cache (Elaboration-Phase)
  void set_l2c(L2CacheModel* l2c) { l2c_ = l2c; }
  // Set CPU (Elaboration-Phase)
  void set_cpu(Cpu* cpu) { cpu_ = cpu; }
  // Set L1 -> L2 Command Queue
  void set_l1_l2__cmd_q(MessageQueueProxy* mq);
  //
  void set_l2_l1__cmd_q(MessageQueue* mq) { l2_l1__cmd_q_ = mq; }
  // Set L1 -> CPU Response Queue
  void set_l1_cpu__rsp_q(MessageQueueProxy* mq);

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
  MessageQueueProxy* l1_l2__cmd_q_ = nullptr;
  // L2 -> L1 Response Queue (L1 owned)
  MessageQueue* l2_l1__rsp_q_ = nullptr;
  // L2 -> L1 Command Queue (L1 owned)
  MessageQueue* l2_l1__cmd_q_ = nullptr;
  // L1 -> CPU Response Queue (CPU owned)
  MessageQueueProxy* l1_cpu__rsp_q_ = nullptr;
  // Message servicing arbiter.
  MQArb* arb_ = nullptr;
  // Transaction table.
  L1TTable* tt_ = nullptr;
  // Main process of execution.
  MainProcess* main_ = nullptr;
  // Cache Instance
  L1Cache* cache_ = nullptr;
  // Pointer to parent L2.
  L2CacheModel* l2c_ = nullptr;
  // L1 cache protocol
  L1CacheModelProtocol* protocol_ = nullptr;
  // Cache configuration.
  L1CacheModelConfig config_;
};

}  // namespace cc

#endif
