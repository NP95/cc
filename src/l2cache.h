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

#include "cache.h"
#include "cfgs.h"
#include "kernel.h"
#include "msg.h"
#include "protocol.h"
#include "sim.h"

namespace cc {

// Forwards:
class L1CacheAgent;
class L2CacheAgent;
class MessageQueue;
class L2LineState;
template <typename>
class CacheModel;
class CCModel;
class L2TState;

//
//
enum class L2CmdOpcode {
  // Obtain line in Shared State
  L1GetS,
  // Obtain line in Exclusive State
  L1GetE,
  // Evict line
  L1Put,
  // Invalid Opcode
  Invalid
};

const char* to_string(L2CmdOpcode opcode);

//
//
class L2CmdMsg : public Message {
  template <typename>
  friend class PooledItem;
  L2CmdMsg();

 public:
  //
  std::string to_string() const override;

  //
  L2CmdOpcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }
  L1CacheAgent* l1cache() const { return l1cache_; }

  void set_opcode(L2CmdOpcode opcode) { opcode_ = opcode; }
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_l1cache(L1CacheAgent* l1cache) { l1cache_ = l1cache; }

 private:
  // Originator L1 cache instance (for response message).
  L1CacheAgent* l1cache_ = nullptr;
  L2CmdOpcode opcode_;
  addr_t addr_;
};

//
//
enum class L2RspOpcode { L1InstallS, L1InstallE };

const char* to_string(L2RspOpcode opcode);

//
//
class L2CmdRspMsg : public Message {
  template <typename>
  friend class PooledItem;
  L2CmdRspMsg();

 public:
  //
  std::string to_string() const override;

  //
  L2RspOpcode opcode() const { return opcode_; }
  bool is() const { return is_; }

  //
  void set_opcode(L2RspOpcode opcode) { opcode_ = opcode; }
  void set_is(bool is) { is_ = is; }

 private:
  L2RspOpcode opcode_;
  bool is_ = false;
};

// Cache data type
using L2CacheModel = CacheModel<L2LineState*>;
// Cache Set data type
using L2CacheModelSet = L2CacheModel::Set;
// Cache Line Iterator type.
using L2CacheModelLineIt = L2CacheModel::LineIterator;

enum class L2Opcode {
  StartTransaction,
  EndTransaction,
  MqSetBlockedOnTransaction,
  MqSetBlockedOnTable,
  MsgConsume,
  RemoveLine,
  InvokeCoherenceAction,
  SetL1LinesShared,
  SetL1LinesInvalid,
  WaitOnMsg,
  WaitNextEpoch
};

//
//
const char* to_string(L2Opcode opcode);

//
//
class L2Command {
  friend class L2CommandBuilder;

 public:
  L2Command(L2Opcode opcode) : opcode_(opcode) {}
  virtual void release() const { delete this; }

  std::string to_string() const;

  // Command opcode
  L2Opcode opcode() const { return opcode_; }
  // Command Coherence action
  CoherenceAction* action() const { return oprands.action; }
  // "Agent" keep out list.
  std::vector<L1CacheAgent*>& agents() { return oprands.agents; }
  const std::vector<L1CacheAgent*>& agents() const { return oprands.agents; }
  addr_t addr() const { return oprands.addr; }

  // Setters
  void set_addr(addr_t addr) { oprands.addr = addr; }

 private:
  virtual ~L2Command();
  //
  struct {
    addr_t addr;
    CoherenceAction* action;
    std::vector<L1CacheAgent*> agents;
  } oprands;
  //
  L2Opcode opcode_;
};

//
//
class L2CommandBuilder {
 public:
  static L2Command* from_opcode(L2Opcode opcode);

  static L2Command* from_action(CoherenceAction* action);
};

//
//
class L2CommandList {
  using vector_type = std::vector<L2Command*>;

 public:
  using const_iterator = vector_type::const_iterator;

  L2CommandList() = default;
  ~L2CommandList();

  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  //
  void push_back(L2Command* cmd);

  // Consume current message and advance agent to next simulation
  // epoch.
  void next_and_do_consume(bool do_consume = false);

 private:
  // Command List
  std::vector<L2Command*> cmds_;
};

//
//
class L2TState {
 public:
  L2TState(kernel::Kernel* k);
  virtual ~L2TState();

  // Destruct/Return to pool
  void release() { delete this; }

  //
  kernel::Event* transaction_start() const { return transaction_start_; }
  kernel::Event* transaction_end() const { return transaction_end_; }

  // Get current cache line
  L2LineState* line() const { return line_; }
  // Address of current transaction.
  addr_t addr() const { return addr_; }
  // Transaction opcode.
  L2CmdOpcode opcode() const { return opcode_; }
  // L1 cache instance
  L1CacheAgent* l1cache() const { return l1cache_; }

  // Set current cache line
  void set_line(L2LineState* line) { line_ = line; }
  // Current transaction address.
  void set_addr(addr_t addr) { addr_ = addr; }
  // Current transaction opcode
  void set_opcode(L2CmdOpcode opcode) { opcode_ = opcode; }
  // Set L1 cache instance.
  void set_l1cache(L1CacheAgent* l1cache) { l1cache_ = l1cache; }

 private:
  //
  kernel::Event* transaction_start_;
  kernel::Event* transaction_end_;
  // Cache line on which current transaction is executing. (Can
  // otherwise be recovered from the address, but this simply saves
  // the lookup into the cache structure).
  L2LineState* line_ = nullptr;
  // Current transaction addres
  addr_t addr_;
  // Current transaction opcode
  L2CmdOpcode opcode_;
  // Originating L1Cache instance.
  L1CacheAgent* l1cache_ = nullptr;
};

//
//
using L2TTable = Table<Transaction*, L2TState*>;

//
//
class L2CacheContext {
 public:
  L2CacheContext() = default;
  ~L2CacheContext();

  //
  addr_t addr() const { return addr_; }
  MQArbTmt t() const { return t_; }
  const Message* msg() const { return mq_->peek(); }
  MessageQueue* mq() const { return mq_; }
  L2CacheAgent* l2cache() const { return l2cache_; }
  bool owns_line() const { return owns_line_; }
  L2LineState* line() const { return line_; }
  bool silently_evicted() const { return silently_evicted_; }

  bool owns_tstate() const { return owns_tstate_; }
  L2TState* tstate() const { return tstate_; }

  //
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_t(MQArbTmt t) { t_ = t; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_l2cache(L2CacheAgent* l2cache) { l2cache_ = l2cache; }
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }
  void set_line(L2LineState* line) { line_ = line; }
  void set_silently_evicted(bool silently_evicted) {
    silently_evicted_ = silently_evicted;
  }

  void set_tstate(L2TState* tstate) { tstate_ = tstate; }
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }

 private:
  // Current address of interest.
  addr_t addr_ = 0;
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  //
  bool owns_line_ = false;
  //
  L2LineState* line_ = nullptr;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // Line is not present (silently evicted).
  bool silently_evicted_ = false;
  // Owns transaction state object.
  bool owns_tstate_ = false;
  // Current transaction state
  L2TState* tstate_ = nullptr;
  // L2 cache instance
  L2CacheAgent* l2cache_ = nullptr;
  // L1 cache instance (originating requestor).
  L1CacheAgent* l1cache_ = nullptr;
};

//
//
class L2CacheAgent : public Agent {
  class MainProcess;

  friend class CpuCluster;
  friend class L1CommandInterpreter;
  friend class L2CommandInterpreter;

 public:
  L2CacheAgent(kernel::Kernel* k, const L2CacheAgentConfig& config);
  virtual ~L2CacheAgent();

  L2CacheAgentConfig config() const { return config_; }

  // L1 Cache Command Queue (n)
  MessageQueue* l1_l2__cmd_q(std::size_t n) const { return l1_l2__cmd_qs_[n]; }
  // L2 -> L1 response queue
  MessageQueueProxy* l2_l1__rsp_q(L1CacheAgent* l1cache) const;
  // L2 -> CC command queue
  MessageQueueProxy* l2_cc__cmd_q() const { return l2_cc__cmd_q_; }
  // CC -> L2 (Snoop) command queue
  MessageQueue* cc_l2__cmd_q() const { return cc_l2__cmd_q_; }
  // CC -> L2 response queue
  MessageQueue* cc_l2__rsp_q() const { return cc_l2__rsp_q_; }
  // L2 -> CC snoop response queue
  MessageQueueProxy* l2_cc__snprsp_q() const { return l2_cc__snprsp_q_; }

 protected:
  // Accessors:
  // Pointer to module arbiter instance.
  MQArb* arb() const { return arb_; }
  // Point to module cache instance.
  L2CacheModel* cache() const { return cache_; }
  // Protocol instance
  L2CacheAgentProtocol* protocol() const { return protocol_; }
  // Transaction table.
  L2TTable* tt() const { return tt_; }

  // Construction:
  void build();
  // Add L1 cache child.
  void add_l1c(L1CacheAgent* l1c);

  // Elaboration:
  virtual void elab() override;
  //
  // Set parent cache controlle
  void set_cc(CCModel* cc) { cc_ = cc; }
  // Set L2 -> CC command queue.
  void set_l2_cc__cmd_q(MessageQueueProxy* mq);
  // L2 -> L1 response queue.
  void set_l2_l1__rsp_q(L1CacheAgent* l1cache, MessageQueueProxy* mq);
  // L2 -> CC snoop response queue.
  void set_l2_cc__snprsp_q(MessageQueueProxy* mq);

  // Design Rule Check (DRC) callback
  virtual void drc() override;

  // "Back-door" write-through cache related method(s):

  // Set cache line 'addr' to Modified state. Expects line to be
  // resident in the cache and in a state where promotion to the
  // modified state can take place. Called by L1 in response to a
  // Store command committing to a line presently in the E state. This
  // 'back-door' mechanism emulates the behavior or a write-through
  // cache.
  void set_cache_line_modified(addr_t addr);

 private:
  // L2 Cache Configuration.
  L2CacheAgentConfig config_;
  // Child L1 Caches
  std::vector<L1CacheAgent*> l1cs_;
  // L1 -> L2 Command Request
  std::vector<MessageQueue*> l1_l2__cmd_qs_;
  // L2 -> L1 Response queue
  std::map<L1CacheAgent*, MessageQueueProxy*> l2_l1__rsp_qs_;
  // L2 -> CC Command Queue
  MessageQueueProxy* l2_cc__cmd_q_ = nullptr;
  // CC -> L2 Command Queue
  MessageQueue* cc_l2__cmd_q_ = nullptr;
  // CC -> L2 Response Queue
  MessageQueue* cc_l2__rsp_q_ = nullptr;
  // L2 -> CC Response Queue
  MessageQueueProxy* l2_cc__snprsp_q_ = nullptr;
  // Queue selection arbiter
  MQArb* arb_ = nullptr;
  // Transaction table.
  L2TTable* tt_ = nullptr;
  // Cache Instance
  L2CacheModel* cache_ = nullptr;
  // Cache Controller instance
  CCModel* cc_ = nullptr;
  // L1 cache protocol
  L2CacheAgentProtocol* protocol_ = nullptr;
  // Main process of execution.
  MainProcess* main_;
};

}  // namespace cc

#endif
