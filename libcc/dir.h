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
class CoherenceAction;
class DirNocEndpoint;

#define DIROPCODE_LIST(__func)                  \
  __func(StartTransaction)                      \
  __func(EndTransaction)                        \
  __func(MsgConsume)                            \
  __func(MqSetBlockedOnTransaction)             \
  __func(MqSetBlockedOnTable)                   \
  __func(InvokeCoherenceAction)                 \
  __func(WaitOnMsg)                             \
  __func(WaitOnMsgOrNextEpoch)


enum class DirOpcode {
#define __declare_opcode(__name) __name,
  DIROPCODE_LIST(__declare_opcode)
#undef __declare_opcode
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

  DirOpcode opcode() const { return opcode_; }
  CoherenceAction* action() const { return oprands.coh.action; }

 private:
  virtual ~DirCommand();
  //
  union {
    struct {
      CoherenceAction* action;
    } coh;
  } oprands;
  //
  DirOpcode opcode_;
};

//
//
class DirCommandBuilder {
 public:
  static DirCommand* from_opcode(DirOpcode opcode);

  static DirCommand* from_action(CoherenceAction* action);
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

  void push_back(DirCommand* cmd);

 private:
  std::vector<DirCommand*> cmds_;
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

  DirLineState* line() const { return line_; }
  addr_t addr() const { return addr_; }
  Agent* origin() const { return origin_; }
  AceCmdOpcode opcode() const { return opcode_; }

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
};

// Cache data type
using DirCacheModel = CacheModel<DirLineState*>;
// Cache Set data type
using DirCacheSet = DirCacheModel::Set;
// Cache Line Iterator type.
using DirCacheLineIt = DirCacheModel::LineIterator;
//
using DirTTable = Table<Transaction*, DirTState*>;

//
//
class DirContext {
 public:
  DirContext() = default;
  ~DirContext();

  //
  MQArbTmt t() const { return t_; }
  const Message* msg() const { return mq_->peek(); }
  MessageQueue* mq() const { return mq_; }
  DirModel* dir() const { return dir_; }
  DirTState* tstate() const { return tstate_; }
  bool owns_tstate() const { return owns_tstate_; }
  bool owns_line() const { return owns_line_; }

  //
  void set_t(MQArbTmt t) { t_ = t; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_dir(DirModel* dir) { dir_ = dir; }
  void set_tstate(DirTState* tstate) { tstate_ = tstate; }
  void set_owns_tstate(bool owns_tstate) { owns_tstate_ = owns_tstate; }
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }

 private:
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
  DirModel(kernel::Kernel* k, const DirModelConfig& config);
  ~DirModel();

  const DirModelConfig& config() const { return config_; }

  // accessors:
  // LLC owned by current directory
  LLCModel* llc() const { return llc_; }
  // NOC -> DIR message queue
  MessageQueue* endpoint();
  // DIR -> NOC message queue
  MessageQueueProxy* dir_noc__msg_q() const { return dir_noc__msg_q_; }
  // Coherence protocol
  DirProtocol* protocol() const { return protocol_; }
  // Transaction table.
  DirTTable* tt() const { return tt_; }

 protected:
  // Build
  void build();
  //
  void set_llc(LLCModel* llc) { llc_ = llc; }
  // Elaboration
  void elab() override;
  //
  void set_dir_noc__msg_q(MessageQueueProxy* mq);

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
  MessageQueueProxy* dir_noc__msg_q_ = nullptr;
  // CPU -> DIR command queue (owned by DIR)
  MessageQueue* cpu_dir__cmd_q_ = nullptr;
  // LLC -> DIR response queue (owned by DIR)
  MessageQueue* llc_dir__rsp_q_ = nullptr;
  // CC -> DIR snoop response queue.
  MessageQueue* cc_dir__snprsp_q_ = nullptr;
  // NOC endpoint
  DirNocEndpoint* noc_endpoint_ = nullptr;
  // NOC endpoint proxies.
  std::vector<MessageQueueProxy*> endpoints_;
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
