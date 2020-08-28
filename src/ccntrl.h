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

#ifndef CC_SRC_CCNTRL_H
#define CC_SRC_CCNTRL_H

#include "cc/cfgs.h"
#include "cc/kernel.h"
#include "msg.h"
#include "protocol.h"
#include "sim.h"

namespace cc {

class DirMapper;
class MessageQueue;
class NocPort;
class L2CacheAgent;
class CCLineState;
class CCProtocol;
class CCAgent;
class CCNocEndpoint;
class CCCoherenceAction;
class Monitor;

enum class CCOpcode {
  // Raise notification that a new transaction has begun.
  TransactionStart,

  // Raise notification that the current transactio has completed.
  TransactionEnd,

  // Consume message at the head of the currently selected message
  // queue.
  MsgConsume,

  // Invoke a coherence protocol defined action.
  InvokeCoherenceAction,

  // Set blocked status of Message Queue awaiting notification of
  // event.
  MqSetBlockedOnEvt,

  // Wait on the arrival of a message from one of the agents ingress
  // message queues.
  WaitOnMsg,

  // Re-evaluate agent after an 'Epoch' has elapsed.
  WaitNextEpoch
};

// Convert to humand-readable string
const char* to_string(CCOpcode opcode);


// Command class which encapsulates the notion of some 'action' to be
// performed on the cache controller's architectural state.
//
class CCCommand {
  friend class CCCommandBuilder;

 public:
  CCCommand(CCOpcode opcode) : opcode_(opcode) {}
  virtual void release() const { delete this; }

  // Convert to a humand readable string.
  std::string to_string() const;

  // Accessors

  // Opcode
  CCOpcode opcode() const { return opcode_; }

  // Coherence action instance.
  CCCoherenceAction* action() const { return oprands_.action; }

  // Transaction instance.
  Transaction* t() const { return oprands_.t; }

  // Associated event
  kernel::Event* event() const { return oprands_.e; }


  // Setters:

  // Set transaction instance
  void set_t(Transaction* t) { oprands_.t = t; }

  // Set event isntance.
  void set_event(kernel::Event* e) { oprands_.e = e; }

 private:
  virtual ~CCCommand();

  // Oprand associated with current opcode.
  struct {
    CCCoherenceAction* action;
    Transaction* t;
    kernel::Event* e;
  } oprands_;

  // Opcode
  CCOpcode opcode_;
};


// Helper class to construct/build CCCommand instances.
//
class CCCommandBuilder {
 public:

  // Construct somple command from an opcode.
  static CCCommand* from_opcode(CCOpcode opcode);

  // Construct command from a coherency-defined action.
  static CCCommand* from_action(CCCoherenceAction* action);

  // Construct transaction end command.
  static CCCommand* build_transaction_end(Transaction* t);

  // Construct "block on event" action.
  static CCCommand* build_blocked_on_event(MessageQueue* mq,
                                           kernel::Event* evt);
};


// Class with encapsulates the notion of a sequence of commands to be
// applied to the cache controller's architectural state.
//
class CCCommandList {
  using cb = CCCommandBuilder;
  using vector_type = std::vector<CCCommand*>;

 public:
  using const_iterator = vector_type::const_iterator;

  CCCommandList() = default;
  ~CCCommandList();

  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  // Destry contents of command list.
  void clear();

  // Push back from opcode.
  void push_back(CCOpcode opcode);

  // Push back from coherence action.
  void push_back(CCCoherenceAction* action);

  // Push back from command instance.
  void push_back(CCCommand* cmd) { cmds_.push_back(cmd); }

  // Transaction starts
  void push_transaction_start();

  // Transaction ends
  void push_transaction_end(Transaction* t);

  // Consume current message and advance agent to next simulation
  // epoch.
  void next_and_do_consume(bool do_consume = false);

 private:
  std::vector<CCCommand*> cmds_;
};

// Class to encapsulate the cache controller resources required to
// perform a given command list.
//
class CCResources {
  using key_map = std::map<const Agent*, std::size_t>;

 public:
  CCResources(const CCCommandList& cl) { build(cl); }


  // Accessors:

  // NOC credits requried for cl
  std::size_t noc_credit_n() const { return noc_credit_n_; }

  //
  std::size_t cmd_q_n() const { return cmd_q_n_; }

  //
  std::size_t rsp_q_n() const { return rsp_q_n_; }

  // CohSrt credits required for 'agent'
  std::size_t coh_srt_n(const Agent* agent) const;

  // CohCmd credits require for 'agent'
  std::size_t coh_cmd_n(const Agent* agent) const;

  // Dt credits required for 'agent'
  std::size_t dt_n(const Agent* agent) const;

  // CohSrt keymap
  const key_map& coh_srt() const { return coh_srt_; }

  // CohCmd keymap
  const key_map& coh_cmd() const { return coh_cmd_; }

  // Dt keymap
  const key_map& dt() const { return dt_; }


  // Setters:

  // Set NOC credits required
  void set_noc_credit_n(std::size_t noc_credit_n) {
    noc_credit_n_ = noc_credit_n;
  }

  // Set command queue entries required
  void set_cmd_q_n(std::size_t cmd_q_n) { cmd_q_n_ = cmd_q_n; }

  // Set response queue entries required
  void set_rsp_q_n(std::size_t rsp_q_n) { rsp_q_n_ = rsp_q_n; }

  // Set CohSrt credits required per agent
  void set_coh_srt_n(const Agent* agent, std::size_t coh_srt_n);

  // Set CohCmd credits required per agent.
  void set_coh_cmd_n(const Agent* agent, std::size_t coh_cmd_n);

  // Set Dt credits required per agent.
  void set_dt_n(const Agent* agent, std::size_t dt_n);

 private:
  void build(const CCCommandList& cl);

  // NOC credits required (Messages Emitted).
  std::size_t noc_credit_n_ = 0;

  // Command queue entries required
  std::size_t cmd_q_n_ = 0;

  // Response queue entries required
  std::size_t rsp_q_n_ = 0;

  // CohSrt agent to credit mapping
  key_map coh_srt_;

  // CohCmd agent to credit mapping
  key_map coh_cmd_;

  // Dt agent to credit mapping
  key_map dt_;
};

//
//
class CCCoherenceAction {
 public:
  virtual std::string to_string() const = 0;

  // Set Resources object for current action.
  virtual void set_resources(CCResources& r) const {}

  // Invoke/Execute coherence action
  virtual bool execute() = 0;

  virtual void release() { delete this; }

 protected:
  virtual ~CCCoherenceAction() = default;
};

//
//
class CCTState {
 public:
  CCTState() = default;
  // Destruct/Return to pool
  void release() { delete this; }


  // Protocol line
  CCLineState* line() const { return line_; }

  // Current line address
  addr_t addr() const { return addr_; }


  // Set protocol line
  void set_line(CCLineState* line) { line_ = line; }

  // Set cache line
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  // Address of current operation
  addr_t addr_ = 0;
  // 
  CCLineState* line_ = nullptr;
};

using CCTTable = Table<Transaction*, CCTState*>;


// Class with encapsulates the state associate with some cache
// controller action.
//
class CCContext {
 public:
  CCContext() = default;
  ~CCContext();

  //
  MQArbTmt t() const { return t_; }

  // Current Message at head of Message Queue
  const Message* msg() const { return mq_->peek(); }

  // Current Message Queue
  MessageQueue* mq() const { return mq_; }

  // Cache Controller model instance
  CCAgent* cc() const { return cc_; }

  // Context owns line
  bool owns_line() const { return owns_line_; }

  // Transaction line
  CCLineState* line() const { return line_; }

  // Invoking process
  AgentProcess* process() const { return process_; }

  // Current time cursor.
  time_t cursor() const { return cursor_; }

  // Set current arbitration tournament
  void set_t(MQArbTmt t) { t_ = t; }

  // Set current Message Queue
  void set_mq(MessageQueue* mq) { mq_ = mq; }

  // Set cache controller instance
  void set_cc(CCAgent* cc) { cc_ = cc; }

  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }
  void set_line(CCLineState* line) { line_ = line; }
  void set_process(AgentProcess* process) { process_ = process; }

  // Advance cursor by delta units.
  void advance_cursor(time_t delta) { cursor_ += delta; }

 private:
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  // Context owns transaction line; is reponsible for its destruction.
  bool owns_line_ = false;
  // Transaction Line
  CCLineState* line_ = nullptr;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // L2 cache instance
  CCAgent* cc_ = nullptr;
  // Invoking process
  AgentProcess* process_ = nullptr;
  // Cursor
  time_t cursor_ = 0;
};

//
//
enum class CCSnpOpcode {
  // Raise notification that a new transaction has begun.
  TransactionStart,

  // Raise notification that the current transactio has completed.
  TransactionEnd,

  // Invoke a coherence protocol defined action.
  InvokeCoherenceAction,

  // Consume message from nominated message queue.
  MsgConsume,

  // Re-evaluate agent after an 'Epoch' has elapsed.
  WaitNextEpoch,

  // Wait on the arrival of a message from one of the agents ingress
  // message queues.
  WaitOnMsg,

  // Invalid, placeholder.
  Invalid
};

// Convert Snoop Opcode to string.
const char* to_string(CCSnpOpcode opcode);

//
//
class CCSnpCommand {
  friend class CCSnpCommandBuilder;

 public:
  CCSnpCommand() = default;
  virtual void release() const { delete this; }

  std::string to_string() const;

  //
  CCSnpOpcode opcode() const { return opcode_; }
  CCCoherenceAction* action() const { return oprands_.action; }

  //
  void set_opcode(CCSnpOpcode opcode) { opcode_ = opcode; }
  void set_action(CCCoherenceAction* action) { oprands_.action = action; }

 private:
  virtual ~CCSnpCommand();

  //
  struct {
    CCCoherenceAction* action;
  } oprands_;
  //
  CCSnpOpcode opcode_;
};

//
//
class CCSnpCommandBuilder {
 public:
  // Construct new command from opcode.
  static CCSnpCommand* from_opcode(CCSnpOpcode opcode);

  // Construct new command from a protocol-defined action.
  static CCSnpCommand* from_action(CCCoherenceAction* action);
};

//
//
class CCSnpCommandList {
  using cb = CCSnpCommandBuilder;
  using vector_type = std::vector<CCSnpCommand*>;

 public:
  using const_iterator = vector_type::const_iterator;

  CCSnpCommandList() = default;
  ~CCSnpCommandList();

  // Command list iterators:
  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  // Push back from opcode.
  void push_back(CCSnpOpcode opcode);

  // Push back from coherence action instance.
  void push_back(CCCoherenceAction* action);

  // Push back from command instance
  void push_back(CCSnpCommand* cmd) { cmds_.push_back(cmd); }

  // Consume current message and advance agent to next simulation
  // epoch.
  void next_and_do_consume(bool do_consume = false);

 private:
  // Command list.
  std::vector<CCSnpCommand*> cmds_;
};

//
//
class CCSnpTState {
 public:
  CCSnpTState() = default;

  // Reclaim state object.
  void release() { delete this; }

  // Snoop line
  CCSnpLineState* line() const { return line_; }

  // TState owns line
  bool owns_line() const { return owns_line_; }

  // Current line address
  addr_t addr() const { return addr_; }


  // Set snoop line
  void set_line(CCSnpLineState* line) { line_ = line; }

  // Set owns line
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }

  // Set addr
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  // Destruct object using 'release' method
  virtual ~CCSnpTState() = default;

  // Snoop line state.
  CCSnpLineState* line_ = nullptr;

  // Flag denoting that the TState object has ownership of the line
  // and is therefore responsible for destructing the line.
  bool owns_line_ = false;

  // Current line address
  addr_t addr_ = 0;
};

//
//
using CCSnpTTable = Table<Transaction*, CCSnpTState*>;

//
//
class CCSnpContext {
 public:
  CCSnpContext() = default;

  // Accesors:
  
  // Current snoop arbitration tournament
  MQArbTmt t() const { return t_; }

  // Current cache controller instance.
  CCAgent* cc() const { return cc_; }

  // Currently nominated Message Queue
  MessageQueue* mq() const { return mq_; }

  // Currently message at the head of the nomianted message queue.
  const Message* msg() const { return mq_->peek(); }

  // Currenty transaction state
  CCSnpTState* tstate() const { return tstate_; }

  // Flag indiating that context owns the current transaction state.
  bool owns_tstate() const { return owns_tstate_; }

  // Invoking process
  AgentProcess* process() const { return process_; }

  // Current time cursor.
  time_t cursor() const { return cursor_; }

  // Setters:
  
  // Set current arbitration tournament
  void set_t(MQArbTmt t) { t_ = t; }

  // Set current cache controller instance.
  void set_cc(CCAgent* cc) { cc_ = cc; }

  // Set currently nomianted message queue
  void set_mq(MessageQueue* mq) { mq_ = mq; }

  // Set current transaction state
  void set_tstate(CCSnpTState* tstate) { tstate_ = tstate; }

  // Set flag indicating that context owns transaction state
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }

  // Set currently executing procsss handle.
  void set_process(AgentProcess* process) { process_ = process; }

  // Advance cursor by delta units.
  void advance_cursor(time_t delta) { cursor_ += delta; }

 private:
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // L2 cache instance
  CCAgent* cc_ = nullptr;
  // Snp transaction state.
  CCSnpTState* tstate_ = nullptr;
  // Flag indiciating that current context owns the tstate and must
  // release the state upon destruction.
  bool owns_tstate_ = false;
  // Invoking process
  AgentProcess* process_ = nullptr;
  // Cursor
  time_t cursor_ = 0;
};

//
//
class CCAgent : public Agent {
  friend class SocTop;
  friend class CpuCluster;
  friend class CCCommandInterpreter;
  friend class CCSnpCommandInterpreter;

  class RdisProcess;
  class SnpProcess;

 public:
  using ccntr_map = std::map<const Agent*, CreditCounter*>;

  CCAgent(kernel::Kernel* k, const CCConfig& config);
  ~CCAgent();

  // Obtain cache controller configuration.
  const CCConfig& config() const { return config_; }

  // L2 -> Controller (Transaction) Command Queue (owning)
  MessageQueue* l2_cc__cmd_q() const { return l2_cc__cmd_q_; }

  // CC -> L2 Command Queue (Snoops)
  MessageQueue* cc_l2__cmd_q() const { return cc_l2__cmd_q_; }

  // CC -> L2 Response Queue
  MessageQueue* cc_l2__rsp_q() const { return cc_l2__rsp_q_; }

  // L2 -> CC Snoop Response
  MessageQueue* l2_cc__snprsp_q() const { return l2_cc__snprsp_q_; }

  // NOC -> CC Ingress Queue
  MessageQueue* endpoint() const;

  // CC -> NOC Egress Queue
  NocPort* cc_noc__port() const { return cc_noc__port_; }

  // Directory Mapper instance.
  DirMapper* dm() const { return dm_; }

  // L2 cache model
  L2CacheAgent* l2c() const { return l2c_; }

  // Credit Counters
  const std::map<MessageClass, ccntr_map>& ccntrs_map() const {
    return ccntrs_map_;
  }

  // Lookup Credit Counter by Message Class and agent
  CreditCounter* cc_by_cls_agent(MessageClass cls, const Agent* agent) const;

  // Lookup Message Queue by Message Class
  MessageQueue* mq_by_msg_cls(MessageClass cls) const;

 protected:
  // Accessors:

  // Pointer to module arbiter instance:
  MQArb* arb() const { return arb_; }

  // Snoop arbitrator instance.
  MQArb* snp_arb() const { return snp_arb_; }

  // Protocol instance
  CCProtocol* protocol() const { return protocol_; }

  // Transaction table.
  CCTTable* table() const { return tt_; }

  // Snoop transactiont table.
  CCSnpTTable* snp_table() const { return snp_tt_; }

  // Construction
  void build();

  // Register Verification Monitor
  void register_monitor(Monitor* monitor);

  // Elaboration
  bool elab() override;

  // Set slave L2C instance.
  void set_l2c(L2CacheAgent* l2c) { l2c_ = l2c; }

  // Set directory mapper.
  void set_dm(DirMapper* dm) { dm_ = dm; }

  // Set CC -> NOC message queue
  void set_cc_noc__port(NocPort* port);

  // Set CC -> L2 message
  void set_cc_l2__cmd_q(MessageQueue* mq);

  // Set CC -> L2 response queue
  void set_cc_l2__rsp_q(MessageQueue* mq);

  // Register credit counter for MessageClass 'cls' in Agent 'agent' with
  // an initial value of 'n' credits.
  void register_credit_counter(MessageClass cls, Agent* dest, std::size_t n);

  // Transaction table
  CCTTable* tt() const { return tt_; }

  // Snoop Transaction Table
  CCSnpTTable* snp_tt() const { return snp_tt_; }

  // Design Rule Check (DRC)
  void drc() override;

 private:
  // L2 Cache Model to which this controller is bound.
  L2CacheAgent* l2c_ = nullptr;

  // L2 -> Controller (Transaction) Command Queue (CC owned)
  MessageQueue* l2_cc__cmd_q_ = nullptr;

  // CC -> L2 command queue (L2 owned)
  MessageQueue* cc_l2__cmd_q_ = nullptr;

  // CC -> L2 response queue (L2 owned)
  MessageQueue* cc_l2__rsp_q_ = nullptr;

  // CC -> NOC Egress Queue (noc owned)
  NocPort* cc_noc__port_ = nullptr;

  // DIR -> CC Ingress Queue (cc owned)
  MessageQueue* dir_cc__rsp_q_ = nullptr;

  // DIR -> CC Command Queue (snoops) (cc owned)
  MessageQueue* dir_cc__snpcmd_q_ = nullptr;

  // L2 -> CC snoop response queue (cc owned)
  MessageQueue* l2_cc__snprsp_q_ = nullptr;

  // CC -> CC response queue. (cc owned)
  MessageQueue* cc_cc__rsp_q_ = nullptr;

  // {LLC, CC} -> CC Data (dt) queue (cc owned)
  MessageQueue* cc__dt_q_ = nullptr;

  // Agent credit counters (keyed on Message class)
  std::map<MessageClass, ccntr_map> ccntrs_map_;

  // Queue selection arbiter
  MQArb* arb_ = nullptr;

  // Snoop Queue Selection Arbiter instance
  MQArb* snp_arb_ = nullptr;

  // Directory Mapper instance
  DirMapper* dm_ = nullptr;

  // Disatpcher process
  RdisProcess* rdis_proc_ = nullptr;

  // Snoop process
  SnpProcess* snp_proc_ = nullptr;

  // NOC endpoint
  CCNocEndpoint* noc_endpoint_ = nullptr;

  // Transaction table instance.
  CCTTable* tt_ = nullptr;

  // Snoop transaction table instance.
  CCSnpTTable* snp_tt_ = nullptr;

  // Cache controller protocol instance.
  CCProtocol* protocol_ = nullptr;

  // Verification monitor instance.
  Monitor* monitor_ = nullptr;

  // Cache controller configuration.
  CCConfig config_;
};

}  // namespace cc

#endif
