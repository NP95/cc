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

#ifndef CC_LIBCC_CCNTRL_H
#define CC_LIBCC_CCNTRL_H

#include "cc/cfgs.h"
#include "cc/kernel.h"
#include "msg.h"
#include "protocol.h"
#include "sim.h"

namespace cc {

class MessageQueue;
class L2CacheModel;
class CCLineState;
class CCProtocol;
class CCModel;
class CCNocEndpoint;

#define CCOPCODE_LIST(__func)                   \
  __func(StartTransaction)                      \
  __func(EndTransaction)                        \
  __func(InvokeCoherenceAction)                 \
  __func(MsgConsume)                            \
  __func(WaitOnMsg)                             \
  __func(WaitNextEpochOrWait)

enum class CCOpcode {
  StartTransaction,
  EndTransaction,
  InvokeCoherenceAction,
  MsgConsume,
  WaitOnMsg,
  WaitNextEpochOrWait
};

const char* to_string(CCOpcode opcode);

//
//
class CCCommand {
  friend class CCCommandBuilder;

 public:
  CCCommand(CCOpcode opcode) : opcode_(opcode) {}
  virtual void release() const { delete this; }

  std::string to_string() const;

  CCOpcode opcode() const { return opcode_; }
  CoherenceAction* action() const { return oprands.coh.action; }

 private:
  virtual ~CCCommand();
  //
  union {
    struct {
      CoherenceAction* action;
    } coh;
  } oprands;
  //
  CCOpcode opcode_;
};

//
//
class CCCommandBuilder {
 public:
  static CCCommand* from_opcode(CCOpcode opcode);

  static CCCommand* from_action(CoherenceAction* action);
};


//
//
class CCCommandList {
  using vector_type = std::vector<CCCommand*>;

 public:
  using const_iterator = vector_type::const_iterator;

  CCCommandList() = default;
  ~CCCommandList();

  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  void push_back(CCCommand* cmd) { cmds_.push_back(cmd); }

 private:
  std::vector<CCCommand*> cmds_;
};

//
//
class CCTState {
 public:
  CCTState() = default;

  // Destruct/Return to pool
  void release() { delete this; }

  //
  CCLineState* line() const { return line_; }
  //
  void set_line(CCLineState* line) { line_ = line; }

 private:
  CCLineState* line_ = nullptr;
};
using CCTTable = Table<Transaction*, CCTState*>;

//
//
class CCContext {
 public:
  CCContext() = default;
  ~CCContext();

  //
  MQArbTmt t() const { return t_; }
  const Message* msg() const { return mq_->peek(); }
  MessageQueue* mq() const { return mq_; }
  CCModel* cc() const { return cc_; }
  bool owns_line() const { return owns_line_; }
  CCLineState* line() const { return line_; }

  //
  void set_t(MQArbTmt t) { t_ = t; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_cc(CCModel* cc) { cc_ = cc; }
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }
  void set_line(CCLineState* line) { line_ = line; }

 private:
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  //
  bool owns_line_ = false;
  //
  CCLineState* line_ = nullptr;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // L2 cache instance
  CCModel* cc_ = nullptr;
};

//
//
enum class CCSnpOpcode {
  TransactionStart,
  TransactionEnd,
  InvokeCoherenceAction,
  ConsumeMsg,
  NextEpoch,
  WaitOnMsg,
  Invalid
};

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
  CoherenceAction* action() const { return oprands.coh.action; }

  //
  void set_opcode(CCSnpOpcode opcode) { opcode_ = opcode; }
  void set_action(CoherenceAction* action) { oprands.coh.action = action; }

 private:
  virtual ~CCSnpCommand();
  
  //
  union {
    struct {
      CoherenceAction* action;
    } coh;
  } oprands;
  //
  CCSnpOpcode opcode_;
};

//
//
class CCSnpCommandList {
  using vector_type = std::vector<CCSnpCommand*>;

 public:
  using const_iterator = vector_type::const_iterator;

  CCSnpCommandList() = default;
  ~CCSnpCommandList();

  const_iterator begin() const { return cmds_.begin(); }
  const_iterator end() const { return cmds_.end(); }

  void push_back(CCSnpCommand* cmd) { cmds_.push_back(cmd); }

 private:
  std::vector<CCSnpCommand*> cmds_;
};

//
//
class CCSnpCommandBuilder {
 public:
  static CCSnpCommand* from_opcode(CCSnpOpcode opcode);

  static CCSnpCommand* from_action(CoherenceAction* action);
};

//
//
class CCSnpTState {
 public:
  CCSnpTState() = default;
  //
  void release() { delete this; }

  //
  CCSnpLineState* line() const { return line_; }
  bool owns_line() const { return owns_line_; }
  
  //
  void set_line(CCSnpLineState* line) { line_ = line; }
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }

 private:
  CCSnpLineState* line_ = nullptr;
  bool owns_line_ = false;
};

//
//
using CCSnpTTable = Table<Transaction*, CCSnpTState*>;

//
//
class CCSnpContext {
 public:
  CCSnpContext() = default;

  //
  MQArbTmt t() const { return t_; }
  CCModel* cc() const { return cc_; }
  MessageQueue* mq() const { return mq_; }
  const Message* msg() const { return mq_->peek(); }
  CCSnpTState* tstate() const { return tstate_; }
  bool owns_tstate() const { return owns_tstate_; }
  
  //
  void set_t(MQArbTmt t) { t_ = t; }
  void set_cc(CCModel* cc) { cc_ = cc; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_tstate(CCSnpTState* tstate) { tstate_ = tstate; }
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }
 private:
  // Current Message Queue arbiter tournament.
  MQArbTmt t_;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  // L2 cache instance
  CCModel* cc_ = nullptr;
  //
  CCSnpTState* tstate_ = nullptr;
  // Flag indiciating that current context owns the tstate and must
  // release the state upon destruction.
  bool owns_tstate_ = false;
};


//
//
class CCModel : public Agent {
  friend class CpuCluster;
  friend class CCCommandInterpreter;
  friend class CCSnpCommandInterpreter;

  class RdisProcess;
  class SnpProcess;
 public:
  CCModel(kernel::Kernel* k, const CCConfig& config);
  ~CCModel();

  // Obtain cache controller configuration.
  const CCConfig& config() const { return config_; }
  // L2 -> Controller (Transaction) Command Queue (owning)
  MessageQueue* l2_cc__cmd_q() const { return l2_cc__cmd_q_; }
  // CC -> L2 Command Queue (Snoops)
  MessageQueueProxy* cc_l2__cmd_q() const { return cc_l2__cmd_q_; }
  // CC -> L2 Response Queue
  MessageQueueProxy* cc_l2__rsp_q() const { return cc_l2__rsp_q_; }
  // L2 -> CC Snoop Response
  MessageQueue* l2_cc__snprsp_q() const { return l2_cc__snprsp_q_; }
  // NOC -> CC Ingress Queue
  MessageQueue* endpoint() const;
  // CC -> NOC Egress Queue
  MessageQueueProxy* cc_noc__msg_q() const { return cc_noc__msg_q_; }
  // Directory Mapper instance.
  DirMapper* dm() const { return dm_; }
  // L2 cache model
  L2CacheModel* l2c() const { return l2c_; }

 protected:
  // Accessors:
  // Pointer to module arbiter instance:
  MQArb* arb() const { return arb_; }
  //
  MQArb* snp_arb() const { return snp_arb_; }
  // Protocol instance
  CCProtocol* protocol() const { return protocol_; }
  // Transaction table.
  CCTTable* table() const { return tt_; }
  // Snoop transactiont table.
  CCSnpTTable* snp_table() const { return snp_tt_; }

  // Construction
  void build();

  // Elaboration
  void elab();
  // Set slave L2C instance.
  void set_l2c(L2CacheModel* l2c) { l2c_ = l2c; }
  // Set directory mapper.
  void set_dm(DirMapper* dm) { dm_ = dm; }
  // Set CC -> NOC message queue
  void set_cc_noc__msg_q(MessageQueueProxy* mq);
  // Set CC -> L2 message
  void set_cc_l2__cmd_q(MessageQueueProxy* mq);
  // Set CC -> L2 response queue
  void set_cc_l2__rsp_q(MessageQueueProxy* mq);
  // Transaction table
  CCTTable* tt() const { return tt_; }
  //
  CCSnpTTable* snp_tt() const { return snp_tt_; }

  // Design Rule Check (DRC)
  void drc();

 private:
  // L2 Cache Model to which this controller is bound.
  L2CacheModel* l2c_ = nullptr;
  // L2 -> Controller (Transaction) Command Queue (owning)
  MessageQueue* l2_cc__cmd_q_ = nullptr;
  // CC -> L2 command queue
  MessageQueueProxy* cc_l2__cmd_q_ = nullptr;
  // CC -> L2 response queue
  MessageQueueProxy* cc_l2__rsp_q_ = nullptr;
  // CC -> NOC Egress Queue
  MessageQueueProxy* cc_noc__msg_q_ = nullptr;
  // DIR -> CC Ingress Queue
  MessageQueue* dir_cc__rsp_q_ = nullptr;
  // DIR -> CC Command Queue (snoops)
  MessageQueue* dir_cc__snpcmd_q_ = nullptr;
  // L2 -> CC snoop response queue
  MessageQueue* l2_cc__snprsp_q_ = nullptr;
  // CC -> CC response queue.
  MessageQueue* cc_cc__rsp_q_ = nullptr;
  // {LLC, CC} -> CC Data (dt) queue
  MessageQueue* cc__dt_q_ = nullptr;
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
  // NOC endpoint proxies.
  std::vector<MessageQueueProxy*> endpoints_;
  // Transaction table instance.
  CCTTable* tt_ = nullptr;
  // Snoop transaction table instance.
  CCSnpTTable* snp_tt_ = nullptr;
  // Cache controller protocol instance.
  CCProtocol* protocol_ = nullptr;
  // Cache controller configuration.
  CCConfig config_;
};

}  // namespace cc

#endif
