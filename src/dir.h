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

#ifndef CC_LIBCC_DIR_H
#define CC_LIBCC_DIR_H

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
class LLCModel;
class DirAgent;
class DirLineState;
class NocPort;
class MessageQueue;
class CoherenceList;
class DirCoherenceAction;
class DirNocEndpoint;
class DirResources;

enum class DirOpcode {
  StartTransaction,
  EndTransaction,
  MsgConsume,
  RemoveLine,
  MqSetBlockedOnTransaction,
  MqSetBlockedOnTable,
  MqSetBlockedOnEvt,
  InvokeCoherenceAction,

  //
  WaitOnMsg,

  //
  WaitNextEpoch,

  //
  Error,

  // Invalid opcode; place-holder
  Invalid
};

const char* to_string(DirOpcode opcode);

//
//
class DirCommand {
  friend class DirCommandBuilder;

 public:
  DirCommand(DirOpcode opcode) : opcode_(opcode) {}
  virtual void release() const { delete this; }

  std::string to_string() const;

  // Accessors:
  DirOpcode opcode() const { return opcode_; }
  DirCoherenceAction* action() const { return oprands.action; }
  kernel::Event* event() const { return oprands.e; }

  // Setters

  // Set (blocked on) event instance
  void set_event(kernel::Event* e) { oprands.e = e; }

  // Set (error) reason message
  void set_reason(const std::string& reason) { oprands.reason = reason; }

 private:
  virtual ~DirCommand();
  //
  struct {
    DirCoherenceAction* action;
    kernel::Event* e;
    std::string reason;
  } oprands;
  //
  DirOpcode opcode_;
};

//
//
class DirCommandBuilder {
 public:
  static DirCommand* from_opcode(DirOpcode opcode);

  static DirCommand* from_action(DirCoherenceAction* action);

  static DirCommand* build_blocked_on_event(MessageQueue* mq, kernel::Event* e);

  static DirCommand* build_error(const std::string& reason);
};

//
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
  DirTState(kernel::Kernel* k);
  // Destruct/Return to pool
  void release();


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
  kernel::Event* transaction_start() const { return transaction_start_; }
  kernel::Event* transaction_end() const { return transaction_end_; }

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

  //
  LLCCmdOpcode llc_cmd_opcode() const { return llc_cmd_opcode_; }


  void set_snoop_n(std::size_t snoop_n) { snoop_n_ = snoop_n; }
  void set_snoop_i(std::size_t snoop_i) { snoop_i_ = snoop_i; }
  void set_dt_i(std::size_t dt_i) { dt_i_ = dt_i; }
  void set_pd_i(std::size_t pd_i) { pd_i_ = pd_i; }
  void set_is_i(std::size_t is_i) { is_i_ = is_i; }
  void set_line(DirLineState* line) { line_ = line; }
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_origin(Agent* origin) { origin_ = origin; }
  void set_opcode(AceCmdOpcode opcode) { opcode_ = opcode; }
  void set_llc_cmd_opcode(LLCCmdOpcode opcode) { llc_cmd_opcode_ = opcode; }

 protected:
  virtual ~DirTState();

 private:
  //
  kernel::Event* transaction_start_ = nullptr;
  kernel::Event* transaction_end_ = nullptr;
  //
  DirLineState* line_ = nullptr;
  addr_t addr_;
  // Originating cache controller origin.
  Agent* origin_ = nullptr;
  //
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

//
//
class DirResources {
  using key_map = std::map<const Agent*, std::size_t>;

 public:
  DirResources(const DirCommandList& cl);

  // Accessors:
  std::size_t tt_entry_n() const { return tt_entry_n_; }
  std::size_t noc_credit_n() const { return noc_credit_n_; }
  std::size_t coh_snp_n(const Agent* agent) const;

  const key_map& coh_snp_n() const { return coh_snp_n_; }

  // Setters
  void set_noc_credit_n(std::size_t n) { noc_credit_n_ = n; }
  void set_coh_snp_n(const Agent* agent, std::size_t n);

 private:
  void build(const DirCommandList& cl);

  // Transaction table entries required.
  std::size_t tt_entry_n_ = 0;
  // NOC credits required
  std::size_t noc_credit_n_ = 0;
  // Coherence snoops requires to agent.
  key_map coh_snp_n_;
};

// Cache data type
using DirCacheModel = CacheModel<DirLineState*>;
// Cache Set data type
using DirCacheModelSet = DirCacheModel::Set;
// Cache Line Iterator type.
using DirCacheModelLineIt = DirCacheModel::LineIterator;
//
using DirTTable = Table<Transaction*, DirTState*>;

//
//
class DirContext {
 public:
  DirContext() = default;
  ~DirContext();

  //
  addr_t addr() const { return addr_; }
  MQArbTmt t() const { return t_; }
  const Message* msg() const { return mq_->peek(); }
  MessageQueue* mq() const { return mq_; }
  DirAgent* dir() const { return dir_; }
  DirTState* tstate() const { return tstate_; }
  bool owns_tstate() const { return owns_tstate_; }
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

  //
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_t(MQArbTmt t) { t_ = t; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_dir(DirAgent* dir) { dir_ = dir; }
  void set_tstate(DirTState* tstate) { tstate_ = tstate; }
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }
  void set_dt_n(std::size_t dt_n) { dt_n_ = dt_n; }
  void set_pd_n(std::size_t pd_n) { pd_n_ = pd_n; }
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
  //
  DirTState* tstate_ = nullptr;
  //
  bool owns_tstate_ = false;
  //
  bool owns_line_ = false;

  //
  std::size_t dt_n_ = 0;
  //
  std::size_t pd_n_ = 0;
  //
  std::size_t is_n_ = 0;
};

//
//
class DirAgent : public Agent {
  friend class SocTop;
  friend class DirCommandInterpreter;

  class RdisProcess;

 public:
  using ccntr_map = std::map<const Agent*, CreditCounter*>;

  DirAgent(kernel::Kernel* k, const DirModelConfig& config);
  ~DirAgent();

  const DirModelConfig& config() const { return config_; }

  // accessors:
  // LLC owned by current directory
  LLCModel* llc() const { return llc_; }
  // NOC -> DIR message queue
  MessageQueue* endpoint();
  // DIR -> NOC message queue
  NocPort* dir_noc__port() const { return dir_noc__port_; }
  // Coherence protocol
  DirProtocol* protocol() const { return protocol_; }
  // Transaction table.
  DirTTable* tt() const { return tt_; }
  // Credit Counters
  const std::map<MessageClass, ccntr_map>& ccntrs_map() const {
    return ccntrs_map_;
  }
  // Lookup Credit Counter by Message Class and agent
  CreditCounter* cc_by_cls_agent(MessageClass cls, const Agent* agent) const;
  // Lookup destination/ingress Messag Queue based upon MessageClass
  // and, additionally, origin agent where applicable.
  MessageQueue* mq_by_msg_cls(MessageClass cls, const Agent* origin) const;

 protected:
  // Build
  void build();
  // Build phase; register new command queue (belonging to
  // a distinct cache controller instance.
  void register_command_queue(const Agent* origin);
  // Set asscoiated LLC instance.
  void set_llc(LLCModel* llc) { llc_ = llc; }
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
  DirCacheModel* cache() const { return cache_; }

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
  LLCModel* llc_ = nullptr;
  // Transaction table
  DirTTable* tt_ = nullptr;
  // Cache Instance
  DirCacheModel* cache_ = nullptr;
  // Coherence protocol
  DirProtocol* protocol_ = nullptr;
  // Current directory configuration.
  DirModelConfig config_;
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
