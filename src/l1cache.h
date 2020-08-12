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
#include "utility.h"

namespace cc {

// Forwards:
class Cpu;
class L1CacheAgent;
class L2CacheAgent;
class L1LineState;
class L1CoherenceAction;

enum class L1CmdOpcode {
  // CPU initiates a Load to a region of memory of some unspecified
  // length, but entirely encapsulated within a single cache line.
  CpuLoad,

  // CPU instiates a Store to a region of memory of some unspecified
  // length, but entirely encapsulated within a single cache line.
  CpuStore,

  // Invalid CPU command; default, placeholder state.
  Invalid
};

//
//
class L1CmdMsg : public Message {
  template <typename>
  friend class PooledItem;

  L1CmdMsg();

 public:
  //
  std::string to_string() const override;

  //
  L1CmdOpcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }

  //
  void set_opcode(L1CmdOpcode opcode) { opcode_ = opcode; }
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  // Current command opcode
  L1CmdOpcode opcode_ = L1CmdOpcode::Invalid;

  // Current command address
  addr_t addr_;
};

//
//
class L1CmdRspMsg : public Message {
  template <typename>
  friend class PooledItem;

  L1CmdRspMsg();

 public:
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

  // Set blocked status of the currently selected message queue, to
  // be subsequently cleared upon notification of the event given as
  // an argument to the command.
  //
  MqSetBlockedOnEvent,

  // Set blocked status of the currently selected message queue on
  // a prior transaction to the same line.
  MqSetBlockedOnTransaction,

  // Set blocked status of the currently selected message queue on
  // the availability of free entries within the agents transaction
  // table.
  MqSetBlockedOnTable,

  // Dequeue message from associated Message Queue, but do not release
  // as it has now been installed in the transaction table.
  MsgDequeue,
  
  // Consume message at the head of the currently selected message
  // queue.
  MsgConsume,

  // Reissue message contained within transaction state object.
  MsgReissue,

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

  // Reserve location in replay queue for use upon transaction end.
  //
  ReserveReplaySlot,

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
  L1CoherenceAction* action() const { return oprands.action; }
  addr_t addr() const { return oprands.addr; }
  kernel::Event* event() const { return oprands.event; }
  Transaction* t() const { return oprands.t; }

  // Setters
  void set_addr(addr_t addr) { oprands.addr = addr; }
  void set_event(kernel::Event* event) { oprands.event = event; }
  void set_t(Transaction* t) { oprands.t = t; }

 private:
  virtual ~L1Command();
  //
  struct {
    L1CoherenceAction* action;
    addr_t addr;
    kernel::Event* event;
    Transaction* t;
  } oprands;
  //
  L1Opcode opcode_;
};

//
//
class L1CommandBuilder {
 public:
  // Build command object instance from opcode.
  static L1Command* from_opcode(L1Opcode opcode);
  // Build protocol defined command from action instance.
  static L1Command* from_action(L1CoherenceAction* action);
  // Build remove line command from address addr.
  static L1Command* build_remove_line(addr_t addr);
  //
  static L1Command* build_blocked_on_event(
      MessageQueue* mq, kernel::Event* e);
  //
  static L1Command* build_start_transaction(Transaction* t);
  //
  static L1Command* build_end_transaction(Transaction* t);
};

//
//
class L1CommandList {
  using vector_type = std::vector<L1Command*>;
  using cb = L1CommandBuilder;

 public:
  using const_iterator = vector_type::const_iterator;

  L1CommandList() = default;
  ~L1CommandList();

  // List accessors.
  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  void clear();

  // Add item.
  void push_back(L1Command* cmd);

  // Transaction starts
  void transaction_start(Transaction* t, bool is_blocking = true);

  // Transaction ends
  void transaction_end(Transaction* t, bool was_blocking = true);

  // Consume current message and advance agent to next simulation
  // epoch.
  void next_and_do_consume(bool do_consume = false);

 private:
  std::vector<L1Command*> cmds_;
};

//
//
class L1Resources {
 public:
  L1Resources(const L1CommandList& l);

  // Getters
  std::size_t tt_entry_n() const { return tt_entry_n_; }
  std::size_t l2_cmd_n() const { return l2_cmd_n_; }
  std::size_t cpu_rsp_n() const { return cpu_rsp_n_; }

  // Setters
  void set_tt_entry_n(std::size_t tt_entry_n) { tt_entry_n_ = tt_entry_n; }
  void set_l2_cmd_n(std::size_t l2_cmd_n) { l2_cmd_n_ = l2_cmd_n; }
  void set_cpu_rsp_n(std::size_t cpu_rsp_n) { cpu_rsp_n_ = cpu_rsp_n; }

 private:
  void build(const L1CommandList& l);

  // Transaction Table entry
  std::size_t tt_entry_n_ = 0;
  // L2 Command Queue entry
  std::size_t l2_cmd_n_ = 0;
  // Cpu Response Queue entry
  std::size_t cpu_rsp_n_ = 0;
};

//
//
class L1CoherenceAction {
 public:
  virtual std::string to_string() const = 0;

  // Set Resources object for current action.
  virtual void set_resources(L1Resources& r) const {}
  
  // Invoke/Execute coherence action
  virtual bool execute() = 0;

  virtual void release() { delete this; }

 protected:
  virtual ~L1CoherenceAction() = default;
};

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
  // Message should be replayed on end
  bool do_replay() const { return do_replay_; }
  // Current message
  const Message* msg() const { return msg_; }

  // Set current cache line
  void set_line(L1LineState* line) { line_ = line; }
  // Set current address
  void set_addr(addr_t addr) { addr_ = addr; }
  // Command opcode.
  void set_opcode(L1CmdOpcode opcode) { opcode_ = opcode; }
  // Set do replay flag
  void set_do_replay(bool do_replay) { do_replay_ = do_replay; }
  // Set current message
  void set_msg(const Message* msg) { msg_ = msg; }

 private:
  virtual ~L1TState();

  // Transaction event instances.
  kernel::Event* transaction_start_ = nullptr;
  kernel::Event* transaction_end_ = nullptr;
  // Transaction address
  addr_t addr_;
  // Initiator command opcode
  L1CmdOpcode opcode_ = L1CmdOpcode::Invalid;
  // Cache line on which current transaction is executing. (Can
  // otherwise be recovered from the address, but this simply saves
  // the lookup into the cache structure).
  L1LineState* line_ = nullptr;
  // Flag indicating that upon completion of the transaction, the
  // message should be issued to the replay queue.
  bool do_replay_ = false;
  // Current initiating message.
  const Message* msg_ = nullptr;
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
  MessageQueue* l1_cpu__rsp_q() const { return l1_cpu__rsp_q_; }
  // L1 -> L2 command queue
  MessageQueue* l1_l2__cmd_q() const { return l1_l2__cmd_q_; }
  // L2 -> L1 response queue
  MessageQueue* l2_l1__rsp_q() const { return l2_l1__rsp_q_; }
  // Message replay queue.
  MessageQueue* replay__cmd_q() const { return replay__cmd_q_; }

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
  bool elab() override;
  // Set parent L2Cache (Elaboration-Phase)
  void set_l2cache(L2CacheAgent* l2cache) { l2cache_ = l2cache; }
  // Set CPU (Elaboration-Phase)
  void set_cpu(Cpu* cpu) { cpu_ = cpu; }
  // Set L1 -> L2 Command Queue
  void set_l1_l2__cmd_q(MessageQueue* mq);
  // Set L1 -> CPU Response Queue
  void set_l1_cpu__rsp_q(MessageQueue* mq);

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
  // Replay Queue 
  MessageQueue* replay__cmd_q_ = nullptr;
  // L1 -> L2 Command Queue (L2 owned)
  MessageQueue* l1_l2__cmd_q_ = nullptr;
  // L2 -> L1 Response Queue (L1 owned)
  MessageQueue* l2_l1__rsp_q_ = nullptr;
  // L1 -> CPU Response Queue (CPU owned)
  MessageQueue* l1_cpu__rsp_q_ = nullptr;
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
