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
#include "cfgs.h"
#include "kernel.h"
#include "msg.h"
#include "sim.h"
#include "types.h"

namespace cc {

// Forwards
class LLCModel;
class DirModel;
class DirLineState;
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
  WaitOnMsg,
  WaitNextEpoch,
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
  void set_event(kernel::Event* e) { oprands.e = e; }

 private:
  virtual ~DirCommand();
  //
  struct {
    DirCoherenceAction* action;
    kernel::Event* e;
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

  static DirCommand* build_blocked_on_event(MessageQueue* mq,
                                            kernel::Event* e);
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

  //
  kernel::Event* transaction_start() const { return transaction_start_; }
  kernel::Event* transaction_end() const { return transaction_end_; }

  std::size_t snoop_n() const { return snoop_n_; }
  std::size_t snoop_i() const { return snoop_i_; }
  std::size_t dt_i() const { return dt_i_; }

  DirLineState* line() const { return line_; }
  addr_t addr() const { return addr_; }
  Agent* origin() const { return origin_; }
  AceCmdOpcode opcode() const { return opcode_; }

  void set_snoop_n(std::size_t snoop_n) { snoop_n_ = snoop_n; }
  void set_snoop_i(std::size_t snoop_i) { snoop_i_ = snoop_i; }
  void set_dt_i(std::size_t dt_i) { dt_i_ = dt_i; }
  void set_line(DirLineState* line) { line_ = line; }
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_origin(Agent* origin) { origin_ = origin; }
  void set_opcode(AceCmdOpcode opcode) { opcode_ = opcode; }

 protected:
  virtual ~DirTState();

 private:
  //
  kernel::Event* transaction_start_;
  kernel::Event* transaction_end_;
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
  void set_noc_credit_n(std::size_t n ) { noc_credit_n_ = n; }
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
  DirModel* dir() const { return dir_; }
  DirTState* tstate() const { return tstate_; }
  bool owns_tstate() const { return owns_tstate_; }
  bool owns_line() const { return owns_line_; }

  //
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_t(MQArbTmt t) { t_ = t; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_dir(DirModel* dir) { dir_ = dir; }
  void set_tstate(DirTState* tstate) { tstate_ = tstate; }
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }

 private:
  // Current address of interest.
  addr_t addr_ = 0;
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // L2 cache instance
  DirModel* dir_ = nullptr;
  //
  DirTState* tstate_ = nullptr;
  //
  bool owns_tstate_ = false;
  //
  bool owns_line_ = false;
};

//
//
class DirModel : public Agent {
  friend class SocTop;
  friend class DirCommandInterpreter;

  class RdisProcess;

 public:
  using ccntr_map = std::map<const Agent*, CreditCounter*>;

  DirModel(kernel::Kernel* k, const DirModelConfig& config);
  ~DirModel();

  const DirModelConfig& config() const { return config_; }

  // accessors:
  // LLC owned by current directory
  LLCModel* llc() const { return llc_; }
  // NOC -> DIR message queue
  MessageQueue* endpoint();
  // DIR -> NOC message queue
  MessageQueue* dir_noc__msg_q() const { return dir_noc__msg_q_; }
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
  void set_dir_noc__msg_q(MessageQueue* mq);
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
  MessageQueue* dir_noc__msg_q_ = nullptr;
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

  virtual DirModel* lookup(addr_t addr) const = 0;
};

//
//
class SingleDirMapper : public DirMapper {
 public:
  SingleDirMapper(DirModel* dm) : dm_(dm) {}

  DirModel* lookup(addr_t addr) const override { return dm_; }

 private:
  // Dir end-point definition.
  DirModel* dm_ = nullptr;
};

}  // namespace cc

#endif
