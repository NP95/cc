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

#ifndef CC_SRC_DIR_H
#define CC_SRC_DIR_H

#include "amba.h"
#include "cache.h"
#include "llc.h"
#include "cc/cfgs.h"
#include "cc/kernel.h"
#include "cc/types.h"
#include "msg.h"
#include "sim.h"

namespace cc {

// Forwards
class LLCAgent;
class DirAgent;
class DirLineState;
class NocPort;
class MessageQueue;
class CoherenceList;
class DirCoherenceAction;
class DirNocEndpoint;
class DirResources;
class DirProtocol;
class Monitor;

enum class DirOpcode {

  // Start new transaction.
  StartTransaction,

  // Terminate current transaction
  EndTransaction,

  // Consume Message at the head of currently selected Message Queue.
  MsgConsume,

  // Remove line from cache.
  RemoveLine,

  // Set Message Queue blocked on prior Transaction completion.
  MqSetBlockedOnTransaction,

  // Set Message Queue blocked on Transaction Table entry
  MqSetBlockedOnTable,

  // Set Message Queue blocked on Event.
  MqSetBlockedOnEvt,

  // Invoke some coherency sub-system defined action.
  InvokeCoherenceAction,

  // Wait on the arrival of another message.
  WaitOnMsg,

  // Re-invoke process after another Epoch has elapsed.
  WaitNextEpoch,

  // Raise error; invalid machine state
  Error,

  // Invalid opcode; place-holder
  Invalid
};

// Convert Opcode to human readable string.
const char* to_string(DirOpcode opcode);


//
//
class DirCommand {
  friend class DirCommandBuilder;

 public:
  explicit DirCommand(DirOpcode opcode) : opcode_(opcode) {}
  virtual void release() const { delete this; }

  std::string to_string() const;

  // Accessors:

  // Command opcode
  DirOpcode opcode() const { return opcode_; }

  // Coherence action oprand associated with command.
  DirCoherenceAction* action() const { return oprands.action; }

  // Event oprand associated with command.
  kernel::Event* event() const { return oprands.e; }

  // Setters:

  // Set (blocked on) event instance
  void set_event(kernel::Event* e) { oprands.e = e; }

  // Set (error) reason message
  void set_reason(const std::string& reason) { oprands.reason = reason; }

 private:
  virtual ~DirCommand();

  // Commands oprands, where applicable
  struct {
    // Action oprand
    DirCoherenceAction* action = nullptr;

    // Event oprand
    kernel::Event* e = nullptr;

    // Error reason message
    std::string reason;
  } oprands;

  // Current command opcode
  DirOpcode opcode_ = DirOpcode::Invalid;
};


// Command builder helper class.
//
class DirCommandBuilder {
 public:
  // Construct command instance from opcode
  static DirCommand* from_opcode(DirOpcode opcode);

  // Construct command instance from coherence action
  static DirCommand* from_action(DirCoherenceAction* action);

  // Construct blocked on event instance.
  static DirCommand* build_blocked_on_event(MessageQueue* mq, kernel::Event* e);

  // Construct error command instance.
  static DirCommand* build_error(const std::string& reason);
};


// Directory command list.
//
class DirCommandList {
  using vector_type = std::vector<DirCommand*>;

 public:
  using const_iterator = vector_type::const_iterator;

  DirCommandList() = default;
  ~DirCommandList();

  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  // Remove all commands from list.
  void clear();

  // Push back from opcode.
  void push_back(DirOpcode opcode);

  // Push back from coherence action
  void push_back(DirCoherenceAction* action);

  // Push back from command instance
  void push_back(DirCommand* cmd);

  // Raise error
  void raise_error(const std::string& reason);

  // Consume current message and advance agent to next simulation
  // epoch.
  void next_and_do_consume(bool do_consume = false);

 private:
  // Command list.
  std::vector<DirCommand*> cmds_;
};

//
//
class DirCoherenceAction {
 public:
  virtual std::string to_string() const = 0;

  // Set Resources object for current action.
  virtual void set_resources(DirResources& r) const {}

  // Invoke/Execute coherence action
  virtual bool execute() = 0;

  virtual void release() { delete this; }

 protected:
  virtual ~DirCoherenceAction() = default;
};

//
//
class DirTState {
 public:
  explicit DirTState(kernel::Kernel* k);
  // Destruct/Return to pool
  void release();


  // Builder methods:

  // Build action to set the LLC comand opcode.
  DirCommand* build_set_llc_cmd_opcode(LLCCmdOpcode opcode);

  // Build action to increment DT
  DirCommand* build_inc_dt();

  // Build action to increment PD
  DirCommand* build_inc_pd();

  // Build action to increment IS
  DirCommand* build_inc_is();

  // Build action to increment snoop expectation count.
  DirCommand* build_set_snoop_n(std::size_t n);

  // Build action to increment snoop response count.
  DirCommand* build_inc_snoop_i();
  


  // Events denoting the initiation and completion of the transaction.

  // Transaction start event; notified on start of transaction.
  kernel::Event* transaction_start() const { return transaction_start_; }

  // Transaction end event; notified on end of transaction.
  kernel::Event* transaction_end() const { return transaction_end_; }



  // Accessors:

  // Flag indicating that this is the final snoop
  bool is_final_snoop(bool is_snoop_rsp) const;

  // The total number of snoops outstanding.
  std::size_t snoop_n() const { return snoop_n_; }

  // The total number of snoop responses received.
  std::size_t snoop_i() const { return snoop_i_; }

  // Data Transfer count associated with current transaction.
  std::size_t dt_i() const { return dt_i_; }

  // Pass Dirty count
  std::size_t pd_i() const { return pd_i_; }

  // Is Shared count
  std::size_t is_i() const { return is_i_; }

  // Directory line associated with current transaction.
  DirLineState* line() const { return line_; }

  // Address of current transaction.
  addr_t addr() const { return addr_; }

  // Originator agent of current transaction.
  Agent* origin() const { return origin_; }

  // Current transaction opcode.
  AceCmdOpcode opcode() const { return opcode_; }

  // Current opcode issued to LLC (or invalid)
  LLCCmdOpcode llc_cmd_opcode() const { return llc_cmd_opcode_; }


  // Setters:

  // Set number of snoop commands.
  void set_snoop_n(std::size_t snoop_n) { snoop_n_ = snoop_n; }

  // Set number of snoop commands (received so far).
  void set_snoop_i(std::size_t snoop_i) { snoop_i_ = snoop_i; }

  // Set Data Transfer flag count.
  void set_dt_i(std::size_t dt_i) { dt_i_ = dt_i; }

  // Set Pass Dirty flag count.
  void set_pd_i(std::size_t pd_i) { pd_i_ = pd_i; }

  // Set Is Shared flag count.
  void set_is_i(std::size_t is_i) { is_i_ = is_i; }

  // Set line instance.
  void set_line(DirLineState* line) { line_ = line; }

  // Set transaction address.
  void set_addr(addr_t addr) { addr_ = addr; }

  // Set originator agent.
  void set_origin(Agent* origin) { origin_ = origin; }

  // Set current ACE command opcode
  void set_opcode(AceCmdOpcode opcode) { opcode_ = opcode; }

  // Set current LLC command opcode (where applicable).
  void set_llc_cmd_opcode(LLCCmdOpcode opcode) { llc_cmd_opcode_ = opcode; }

 protected:
  virtual ~DirTState();

 private:
  // Event notified on transation start.
  kernel::Event* transaction_start_ = nullptr;
  // Event notified on transaction end.
  kernel::Event* transaction_end_ = nullptr;
  // Current directory line.
  DirLineState* line_ = nullptr;
  // Current line address
  addr_t addr_;
  // Originating cache controller origin.
  Agent* origin_ = nullptr;
  // Current Ace comamnd begin processed.
  AceCmdOpcode opcode_ = AceCmdOpcode::Invalid;
  // The total expected number of snoop responses
  std::size_t snoop_n_ = 0;
  // The total number of snoop responses received.
  std::size_t snoop_i_ = 0;
  // The total number of data transfers
  std::size_t dt_i_ = 0;
  // Snoop was Passed Dirty flag
  std::size_t pd_i_ = 0;
  // Snoop was Is Shared flag
  std::size_t is_i_ = 0;
  // Awaiting LLC Command.
  LLCCmdOpcode llc_cmd_opcode_ = LLCCmdOpcode::Invalid;
};


// Class to encapsulate the notion of the resources associated with a
// given CommandList
//
class DirResources {
  using key_map = std::map<const Agent*, std::size_t>;

 public:
  // Construct (compute) resource set required by CommandList.
  DirResources(const DirCommandList& cl);

  // Accessors:

  // Required number of Transaction Table entries.
  std::size_t tt_entry_n() const { return tt_entry_n_; }

  // Required number of NOC credits (messages) emitted to the
  // interconnect.
  std::size_t noc_credit_n() const { return noc_credit_n_; }

  // Required number of snoops (for a given destination agent).
  std::size_t coh_snp_n(const Agent* agent) const;

  // Agent to credit count map.
  const key_map& coh_snp_n() const { return coh_snp_n_; }

  // Setters:

  // Set NOC credit count.
  void set_noc_credit_n(std::size_t n) { noc_credit_n_ = n; }

  // Set Snoop count.
  void set_coh_snp_n(const Agent* agent, std::size_t n);

 private:
  // Compute resource set.
  void build(const DirCommandList& cl);

  // Transaction table entries required.
  std::size_t tt_entry_n_ = 0;
  // NOC credits required
  std::size_t noc_credit_n_ = 0;
  // Coherence snoops requires to agent.
  key_map coh_snp_n_;
};

//
//
class DirContext {
 public:
  explicit DirContext() = default;
  ~DirContext();

  // Accessors:

  // Current address
  addr_t addr() const { return addr_; }

  // Current Arbiter tournament
  MQArbTmt t() const { return t_; }

  // Message at the head of the nominated message queue.
  const Message* msg() const { return mq_->peek(); }

  // Currently nominated message queue.
  MessageQueue* mq() const { return mq_; }

  // Directory agent instance.
  DirAgent* dir() const { return dir_; }

  // Current transaction state instance.
  DirTState* tstate() const { return tstate_; }

  // Flag indicating that context owns transaction state instance.
  bool owns_tstate() const { return owns_tstate_; }

  // Flag indicating that context owns line within transaction state
  // instance.
  bool owns_line() const { return owns_line_; }

  // Consensus: data was transfers.
  bool dt() const { return dt_n() > 0; }

  // Consensus: was passed dirty/
  bool pd() const { return pd_n() > 0; }

  // Consensus: data was retained as shared.
  bool is() const { return is_n() > 0; }

  // Current "Data Transfer" (DT) count
  std::size_t dt_n() const { return dt_n_; }

  // Current "Pass Dirty" (PD) count
  std::size_t pd_n() const { return pd_n_; }

  // Current "Is Shared" (IS) count.
  std::size_t is_n() const { return is_n_; }


  // Setters:

  // Set current address
  void set_addr(addr_t addr) { addr_ = addr; }

  // Set current arbitration tournament
  void set_t(MQArbTmt t) { t_ = t; }

  // Set current Message Queue
  void set_mq(MessageQueue* mq) { mq_ = mq; }

  // Set directory instance
  void set_dir(DirAgent* dir) { dir_ = dir; }

  // Set Transaction State instance.
  void set_tstate(DirTState* tstate) { tstate_ = tstate; }

  // Set flag indicating that context owns the Transaction state
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }

  // Set flag indicating that context owns line
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }

  // Set Data Transfer count
  void set_dt_n(std::size_t dt_n) { dt_n_ = dt_n; }

  // Set Pass Dirty count
  void set_pd_n(std::size_t pd_n) { pd_n_ = pd_n; }

  // Set Is Shared count
  void set_is_n(std::size_t is_n) { is_n_ = is_n; }

 private:
  // Current address of interest.
  addr_t addr_ = 0;
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // L2 cache instance
  DirAgent* dir_ = nullptr;
  // Transaction state
  DirTState* tstate_ = nullptr;
  // Context owns transaction state
  bool owns_tstate_ = false;
  // Context owns lint
  bool owns_line_ = false;
  // Data Transfer count
  std::size_t dt_n_ = 0;
  // Pass Dirty count
  std::size_t pd_n_ = 0;
  // Is Shared count
  std::size_t is_n_ = 0;
};


// Directory Agent class.
//
class DirAgent : public Agent {
  friend class SocTop;
  friend class DirCommandInterpreter;

  class RdisProcess;

 public:
  using ccntr_map = std::map<const Agent*, CreditCounter*>;

  DirAgent(kernel::Kernel* k, const DirAgentConfig& config);
  ~DirAgent();

  // Current directory configuration.
  const DirAgentConfig& config() const { return config_; }

  // Accessors:

  // LLC owned by current directory
  LLCAgent* llc() const { return llc_; }
  // NOC -> DIR message queue
  MessageQueue* endpoint();
  // DIR -> NOC message queue
  NocPort* dir_noc__port() const { return dir_noc__port_; }
  // Coherence protocol
  DirProtocol* protocol() const { return protocol_; }
  // Transaction table.
  Table<Transaction*, DirTState*>* tt() const { return tt_; }
  // Credit Counters
  const std::map<MessageClass, ccntr_map>& ccntrs_map() const {
    return ccntrs_map_;
  }
  // Lookup Credit Counter by Message Class and agent
  CreditCounter* cc_by_cls_agent(MessageClass cls, const Agent* agent) const;
  // Lookup destination/ingress Messag Queue based upon MessageClass
  // and, additionally, origin agent where applicable.
  MessageQueue* mq_by_msg_cls(MessageClass cls, const Agent* origin) const;
  // Point to module cache instance.
  const CacheModel<DirLineState*>* cache() const { return cache_; }

 protected:
  // Build
  void build();
  // Build phase; register new command queue (belonging to
  // a distinct cache controller instance.
  void register_command_queue(const Agent* origin);
  // Register verification monitor.
  void register_monitor(Monitor* monitor);
  // Set asscoiated LLC instance.
  void set_llc(LLCAgent* llc) { llc_ = llc; }
  // Elaboration
  bool elab() override;
  // Set Dir -> NOC message queue (NOC owned).
  void set_dir_noc__port(NocPort* port);
  // Register credit counter for MessageClass 'cls' in Agent 'agent' with
  // an initial value of 'n' credits.
  void register_credit_counter(MessageClass cls, Agent* dest, std::size_t n);

  // Design Rule Check (DRC)
  void drc() override;

  // accessor(s)

  // Directory arbiter instance.
  MQArb* arb() const { return arb_; }
  // Point to module cache instance.
  CacheModel<DirLineState*>* cache() { return cache_; }

 private:
  // Queue selection arbiter
  MQArb* arb_ = nullptr;
  // DIR -> NOC message queue (owned by NOC)
  NocPort* dir_noc__port_ = nullptr;
  // CPU -> DIR command queue (owned by DIR)
  std::map<const Agent*, MessageQueue*> cc_dir__cmd_q_;
  // LLC -> DIR response queue (owned by DIR)
  MessageQueue* llc_dir__rsp_q_ = nullptr;
  // CC -> DIR snoop response queue.
  MessageQueue* cc_dir__snprsp_q_ = nullptr;
  // Agent credit counters (keyed on Message class)
  std::map<MessageClass, ccntr_map> ccntrs_map_;
  // NOC endpoint
  DirNocEndpoint* noc_endpoint_ = nullptr;
  // request dispatcher process
  RdisProcess* rdis_proc_ = nullptr;
  // Last Level Cache instance (where applicable).
  LLCAgent* llc_ = nullptr;
  // Transaction table
  Table<Transaction*, DirTState*>* tt_ = nullptr;
  // Cache Instance
  CacheModel<DirLineState*>* cache_ = nullptr;
  // Coherence protocol
  DirProtocol* protocol_ = nullptr;
  // Verification monitor instance, where applicable.
  Monitor* monitor_ = nullptr;
  // Current directory configuration.
  DirAgentConfig config_;
};

//
//
class DirMapper {
 public:
  DirMapper() = default;
  virtual ~DirMapper() = default;

  virtual DirAgent* lookup(addr_t addr) const = 0;
};

//
//
class SingleDirMapper : public DirMapper {
 public:
  SingleDirMapper(DirAgent* dm) : dm_(dm) {}

  DirAgent* lookup(addr_t addr) const override { return dm_; }

 private:
  // Dir end-point definition.
  DirAgent* dm_ = nullptr;
};

}  // namespace cc

#endif
