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
class L1CacheAgent;
class L2CacheAgent;
class L1LineState;
class CoherenceAction;

enum class L1CmdOpcode { CpuLoad, CpuStore };

//
//
class L1CmdMsg : public Message {
 public:
  L1CmdMsg();

  //
  std::string to_string() const override;

  //
  L1CmdOpcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }

  //
  void set_opcode(L1CmdOpcode opcode) { opcode_ = opcode; }
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  L1CmdOpcode opcode_;
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

// Opcode
enum class L1Opcode {

  // Raise notification that a new transaction has begun.
  StartTransaction,

  // Raise notification that the current transactio has completed.
  EndTransaction,

  // Set blocked status of the currently selected message queue on
  // a prior transaction to the same line.
  MqSetBlockedOnTransaction,

  // Set blocked status of the currently selected message queue on
  // the availability of free entries within the agents transaction
  // table.
  MqSetBlockedOnTable,

  // Consume message at the head of the currently selected message
  // queue.
  MsgConsume,

  // Remove a line given by the current line address in the
  // Transaction State object.
  RemoveLine,

  // Invoke a coherence protocol defined action.
  InvokeCoherenceAction,

  // Wait on the arrival of a message from one of the agents ingress
  // message queues.
  WaitOnMsg,

  // Re-evaluate agent after an 'Epoch' has elapsed.
  WaitNextEpoch,

  // Set corresponding line in L2 to Modified state. Invoked
  // specifically on the transition from the E to the M states and
  // emulates the speculative nature of the promotion process that
  // would typically be carried out by the store queue hardware in a
  // hardware implementation.
  SetL2LineModified,

  // Invalid opcode; placeholder for default bad state.
  Invalid
};

// Convert Opcode to string
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

  // List accessors.
  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  // Add item.
  void push_back(L1Command* cmd);

  // Transaction starts
  void transaction_start();

  // Transaction ends
  void transaction_end();
  
  // Consume current message and advance agent to next simulation
  // epoch.
  void next_and_do_consume(bool do_consume = false);

 private:
  std::vector<L1Command*> cmds_;
};

//

//
class L1TState {
 public:
  L1TState(kernel::Kernel* k);

  // Destruct/Return to pool
  void release() { delete this; }

  // Transaction "Start" Event
  kernel::Event* transaction_start() const { return transaction_start_; }
  // Transaction "End" Event
  kernel::Event* transaction_end() const { return transaction_end_; }

  // Get current cache line
  L1LineState* line() const { return line_; }
  // Current transaction address
  addr_t addr() const { return addr_; }
  // Command opcode
  L1CmdOpcode opcode() const { return opcode_; }

  // Set current cache line
  void set_line(L1LineState* line) { line_ = line; }
  // Set current address
  void set_addr(addr_t addr) { addr_ = addr; }
  // Command opcode.
  void set_opcode(L1CmdOpcode opcode) { opcode_ = opcode; }

  void add_blocked_mq(MessageQueue* mq) { mqs_.push_back(mq); }

  const std::vector<MessageQueue*>& mq() const { return mqs_; }

 private:
  virtual ~L1TState();
  
  // Transaction event instances.
  kernel::Event* transaction_start_;
  kernel::Event* transaction_end_;
  // Transaction address
  addr_t addr_;
  // Initiator command opcode
  L1CmdOpcode opcode_;
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
  AgentProcess* process() const { return process_; }
  addr_t addr() const { return addr_; }
  MQArbTmt t() const { return t_; }
  const Message* msg() const { return mq_->peek(); }
  MessageQueue* mq() const { return mq_; }
  L1CacheAgent* l1cache() const { return l1cache_; }
  // REMOVE
  L1LineState* line() const { return line_; }
  bool owns_line() const { return owns_line_; }
  L1TState* tstate() const { return tstate_; }
  bool owns_tstate() const { return owns_tstate_; }

  //
  void set_process(AgentProcess* process) { process_ = process; }
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_t(MQArbTmt t) { t_ = t; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_l1cache(L1CacheAgent* l1cache) { l1cache_ = l1cache; }
  void set_line(L1LineState* line) { line_ = line; }
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }
  void set_tstate(L1TState* tstate) { tstate_ = tstate; }
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }

 private:
  // Current invoke process instance.
  AgentProcess* process_ = nullptr;
  addr_t addr_;
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  // Cacheline instance.
  L1LineState* line_ = nullptr;
  // Context owns cache line instance (and is therefore responsbile for
  // its destruction).
  bool owns_line_ = false;
  // Transaction state instance.
  L1TState* tstate_ = nullptr;
  // Context owns transaction state instance
  bool owns_tstate_ = false;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // L1 cache instance
  L1CacheAgent* l1cache_ = nullptr;
};

// Cache data type
using L1CacheModel = CacheModel<L1LineState*>;
// Cache Set data type
using L1CacheModelSet = L1CacheModel::Set;
// Cache Line Iterator type.
using L1CacheModelLineIt = L1CacheModel::LineIterator;

//
//
class L1CacheAgent : public Agent {
  class MainProcess;

  friend class CpuCluster;
  friend class L2CommandInterpreter;
  friend class L1CommandInterpreter;

 public:
  L1CacheAgent(kernel::Kernel* k, const L1CacheAgentConfig& config);
  virtual ~L1CacheAgent();

  // Return current L1 configuration.
  const L1CacheAgentConfig& config() const { return config_; }
  // Accessors:
  // Cache model instance
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
  L2CacheAgent* l2cache() const { return l2cache_; }
  // Protocol
  L1CacheAgentProtocol* protocol() const { return protocol_; }
  // Transaction table.
  L1TTable* tt() const { return tt_; }

  // Build Phase:
  void build();

  // Elaboration Phase:
  virtual void elab() override;
  // Set parent L2Cache (Elaboration-Phase)
  void set_l2cache(L2CacheAgent* l2cache) { l2cache_ = l2cache; }
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

  // "Back-door" write-through cache related method(s):

  // Set cache line 'addr' to either Shared or Invalid state. Method
  // expects line to reside in cache. Called upon L2 initiated
  // demotion in response to some inbound snoop command.
  //
  void set_cache_line_shared_or_invalid(addr_t addr, bool shared = true);

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
  L2CacheAgent* l2cache_ = nullptr;
  // L1 cache protocol
  L1CacheAgentProtocol* protocol_ = nullptr;
  // Cache configuration.
  L1CacheAgentConfig config_;
};

}  // namespace cc

#endif
