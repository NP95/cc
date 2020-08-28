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

#ifndef CC_SRC_L2CACHE_H
#define CC_SRC_L2CACHE_H

#include <vector>

#include "cache.h"
#include "cc/cfgs.h"
#include "cc/kernel.h"
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
class CCAgent;
class L2TState;
class L2CoherenceAction;
class Monitor;

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

// Convert to human-readable string
const char* to_string(L2CmdOpcode opcode);

//
//
class L2CmdMsg : public Message {
  template <typename>
  friend class PooledItem;
  L2CmdMsg();

 public:
  // Convert to a humand-readable string
  std::string to_string() const override;

  // Command opcode
  L2CmdOpcode opcode() const { return opcode_; }

  // Command address
  addr_t addr() const { return addr_; }

  // Originator L1cache instance.
  L1CacheAgent* l1cache() const { return l1cache_; }


  // Set message opcode
  void set_opcode(L2CmdOpcode opcode) { opcode_ = opcode; }

  // Set address
  void set_addr(addr_t addr) { addr_ = addr; }

  // Set originator L1 cache instance
  void set_l1cache(L1CacheAgent* l1cache) { l1cache_ = l1cache; }

 private:
  // Originator L1 cache instance (for response message).
  L1CacheAgent* l1cache_ = nullptr;

  // Opcode
  L2CmdOpcode opcode_ = L2CmdOpcode::Invalid;

  // Address
  addr_t addr_;
};

//
//
enum class L2RspOpcode { L1InstallS, L1InstallE };

// Command opcode.
const char* to_string(L2RspOpcode opcode);

//
//
class L2CmdRspMsg : public Message {
  template <typename>
  friend class PooledItem;
  L2CmdRspMsg();

 public:
  // Convert to a humand-readable string
  std::string to_string() const override;

  // Message opcode
  L2RspOpcode opcode() const { return opcode_; }

  // Message IsShared flag
  bool is() const { return is_; }

  // Set message opcode
  void set_opcode(L2RspOpcode opcode) { opcode_ = opcode; }

  // Set message IsShared
  void set_is(bool is) { is_ = is; }

 private:
  // Response opcode
  L2RspOpcode opcode_;

  // IsShared flag
  bool is_ = false;
};

// Cache data type
using L2CacheModel = CacheModel<L2LineState*>;
// Cache Set data type
using L2CacheModelSet = L2CacheModel::Set;
// Cache Line Iterator type.
using L2CacheModelLineIt = L2CacheModel::LineIterator;


// Enumeration definition the opcodes which can be executed by the L2
// cache interpreter.
//
enum class L2Opcode {

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

  // Set corresponding line in L1 to the Shared state
  SetL1LinesShared,

  // Set corresponding line in L1 to the Invalid state
  SetL1LinesInvalid,

  // Wait on the arrival of a message from one of the agents ingress
  // message queues.
  WaitOnMsg,

  // Re-evaluate agent after an 'Epoch' has elapsed.
  WaitNextEpoch
};


// Convert Opcode to humand readable string
const char* to_string(L2Opcode opcode);

// Command class which encapsulates the notion of some 'action' to be
// performed on the L2 caches architectural state.
//
class L2Command {
  friend class L2CommandBuilder;

  virtual ~L2Command();
 public:
  L2Command(L2Opcode opcode) : opcode_(opcode) {}
  virtual void release() const { delete this; }

  // Convert to a human-readable string.
  std::string to_string() const;

  // Accessors:
  
  // Command opcode
  L2Opcode opcode() const { return opcode_; }

  // Command Coherence action
  L2CoherenceAction* action() const { return oprands.action; }

  // "Agent" keep out list.
  const std::vector<L1CacheAgent*>& agents() const { return oprands.agents; }

  // Command address
  addr_t addr() const { return oprands.addr; }

  // Setters:

  // Set command address
  void set_addr(addr_t addr) { oprands.addr = addr; }

  // Agent keep out set.
  std::vector<L1CacheAgent*>& agents() { return oprands.agents; }

 private:

  // Oprands associated with current opcode
  struct {
    addr_t addr;
    L2CoherenceAction* action;
    std::vector<L1CacheAgent*> agents;
  } oprands;

  // Command opcode.
  L2Opcode opcode_;
};

// Builder utility to construct instances of L2 commands.
//
class L2CommandBuilder {
 public:

  // Construct an L2 command instance from an opcode.
  static L2Command* from_opcode(L2Opcode opcode);

  // Construct an L2 command from some coherency-defined action.
  static L2Command* from_action(L2CoherenceAction* action);
};

//
//
class L2CommandList {
  using vector_type = std::vector<L2Command*>;

 public:
  using const_iterator = vector_type::const_iterator;

  L2CommandList() = default;
  ~L2CommandList();

  // Iterators ober command list:

  // Iterator to beginning of the command list.
  const_iterator begin() const { return cmds_.begin(); }

  // Iterator to one-past-the-end of the command list.
  const_iterator end() const { return cmds_.end(); }

  // Clear (and destroy) all commands contained in the list.
  void clear();

  // Push opcode (also construct the associated command object).
  void push_back(L2Opcode opcode);

  // Push command object.
  void push_back(L2Command* cmd);

  // Consume current message and advance agent to next simulation
  // epoch.
  void next_and_do_consume(bool do_consume = false);

 private:
  // Command List
  std::vector<L2Command*> cmds_;
};

// Class to encapsulate the resources required by a pre-defined
// CommandList.
//
class L2Resources {
 public:
  L2Resources(const L2CommandList& cl) { build(cl); }

  // Accessors:

  // Number of transaction table entries required.
  std::size_t tt_entry_n() const { return tt_entry_n_; }

  // Number of command credits required.
  std::size_t cc_cmd_n() const { return cc_cmd_n_; }

  // Number of snoop responses required.
  std::size_t cc_snp_rsp_n() const { return cc_snp_rsp_n_; }

  // Number of L1 responses required.
  std::size_t l1_rsp_n() const { return l1_rsp_n_; }


  // Setters:

  // Set required transaction table entry count.
  void set_tt_entry_n(std::size_t tt_entry_n) { tt_entry_n_ = tt_entry_n; }

  // Set command credit count
  void set_cc_cmd_n(std::size_t cc_cmd_n) { cc_cmd_n_ = cc_cmd_n; }

  // Set snoop response credit count.
  void set_cc_snp_rsp_n(std::size_t cc_snp_rsp_n) {
    cc_snp_rsp_n_ = cc_snp_rsp_n;
  }

  // Set L1 response queue credit count.
  void set_l1_rsp_n(std::size_t l1_rsp_n) { l1_rsp_n_ = l1_rsp_n; }

 private:
  void build(const L2CommandList& cl);

  // Transaction Table entry.
  std::size_t tt_entry_n_ = 0;
  // Cache Controller Command Queue
  std::size_t cc_cmd_n_ = 0;
  // Cache Controller Snoop Response Queue
  std::size_t cc_snp_rsp_n_ = 0;
  // L1 Response Queue.
  std::size_t l1_rsp_n_ = 0;
};


//
//
class L2CoherenceAction {
 public:
  virtual std::string to_string() const = 0;

  // Set Resources object for current action.
  virtual void set_resources(L2Resources& r) const {}

  // Invoke/Execute coherence action
  virtual bool execute() = 0;

  virtual void release() { delete this; }

 protected:
  virtual ~L2CoherenceAction() = default;
};


// Class to encapsulate the Transaction state.
//
class L2TState {
 public:
  L2TState(kernel::Kernel* k);
  virtual ~L2TState();

  // Destruct/Return to pool
  void release() { delete this; }

  // Event indicating the start of a transaction.
  kernel::Event* transaction_start() const { return transaction_start_; }

  // Event indiciating the end of a transaction.
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

  // Event notified upon the installation of the transaction in the
  // transaction table.
  kernel::Event* transaction_start_ = nullptr;

  // Event notified upon the removall of the transaction from the
  // transaction table.
  kernel::Event* transaction_end_ = nullptr;

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

// Context class which encapsulates all of the state which persists
// through a coherence action. 
//
class L2CacheContext {
 public:
  L2CacheContext() = default;
  ~L2CacheContext();

  // Accesors:
  
  // Current transaction address
  addr_t addr() const { return addr_; }

  // Current arbitration tournament
  MQArbTmt t() const { return t_; }

  // Current message being processed.
  const Message* msg() const { return mq_->peek(); }

  // Currently nomianted message queue
  MessageQueue* mq() const { return mq_; }

  // Owning L2 cache instance.
  L2CacheAgent* l2cache() const { return l2cache_; }

  // Flag indicating the context owns the current cache line.
  bool owns_line() const { return owns_line_; }

  // L2 line state.
  L2LineState* line() const { return line_; }

  // Flag indicating that the line has been silently evicted.
  bool silently_evicted() const { return silently_evicted_; }

  // Flag indciating that the context owns the current transaction state
  // object.
  bool owns_tstate() const { return owns_tstate_; }

  // Current transaction state object.
  L2TState* tstate() const { return tstate_; }


  // Setters:

  // Set current address
  void set_addr(addr_t addr) { addr_ = addr; }

  // Set current arbitration tournament.
  void set_t(MQArbTmt t) { t_ = t; }

  // Set current message queue
  void set_mq(MessageQueue* mq) { mq_ = mq; }

  // Set current L2 cache instance.
  void set_l2cache(L2CacheAgent* l2cache) { l2cache_ = l2cache; }

  // Set flag indicating that context owns cache line.
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }

  // Set cache line
  void set_line(L2LineState* line) { line_ = line; }

  // Set silently evicted line.
  void set_silently_evicted(bool silently_evicted) {
    silently_evicted_ = silently_evicted;
  }

  // Set transaction state.
  void set_tstate(L2TState* tstate) { tstate_ = tstate; }

  // Set flag indicating that context owns current transaction state.
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }

 private:
  // Current address of interest.
  addr_t addr_ = 0;
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  // Flag indicating that the context owns the line (and is therefore
  // responsible for its destruction).
  bool owns_line_ = false;
  // Cache line state
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
  MessageQueue* l2_l1__rsp_q(L1CacheAgent* l1cache) const;
  // L2 -> CC command queue
  MessageQueue* l2_cc__cmd_q() const { return l2_cc__cmd_q_; }
  // CC -> L2 (Snoop) command queue
  MessageQueue* cc_l2__cmd_q() const { return cc_l2__cmd_q_; }
  // CC -> L2 response queue
  MessageQueue* cc_l2__rsp_q() const { return cc_l2__rsp_q_; }
  // L2 -> CC snoop response queue
  MessageQueue* l2_cc__snprsp_q() const { return l2_cc__snprsp_q_; }

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
  // Register verification monitor instance.
  void register_monitor(Monitor* monitor);

  // Elaboration:
  bool elab() override;
  //
  // Set parent cache controlle
  void set_cc(CCAgent* cc) { cc_ = cc; }
  // Set L2 -> CC command queue.
  void set_l2_cc__cmd_q(MessageQueue* mq);
  // L2 -> L1 response queue.
  void set_l2_l1__rsp_q(L1CacheAgent* l1cache, MessageQueue* mq);
  // L2 -> CC snoop response queue.
  void set_l2_cc__snprsp_q(MessageQueue* mq);

  // Design Rule Check (DRC) callback
  void drc() override;

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
  std::map<L1CacheAgent*, MessageQueue*> l2_l1__rsp_qs_;
  // L2 -> CC Command Queue (CC owned)
  MessageQueue* l2_cc__cmd_q_ = nullptr;
  // CC -> L2 Command Queue (L2 owned)
  MessageQueue* cc_l2__cmd_q_ = nullptr;
  // CC -> L2 Response Queue (L2 owned)
  MessageQueue* cc_l2__rsp_q_ = nullptr;
  // L2 -> CC Response Queue (CC owned)
  MessageQueue* l2_cc__snprsp_q_ = nullptr;
  // Queue selection arbiter
  MQArb* arb_ = nullptr;
  // Transaction table.
  L2TTable* tt_ = nullptr;
  // Cache Instance
  L2CacheModel* cache_ = nullptr;
  // Cache Controller instance
  CCAgent* cc_ = nullptr;
  // L1 cache protocol
  L2CacheAgentProtocol* protocol_ = nullptr;
  // Verification monitor instance.
  Monitor* monitor_ = nullptr;
  // Main process of execution.
  MainProcess* main_;
};

}  // namespace cc

#endif
