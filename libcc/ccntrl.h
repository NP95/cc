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
class CC;

//
//
class CCTState {
 public:
  CCTState() = default;

  // Destruct/Return to pool
  void release() { delete this; }

  //
  CCLineState* line() const { return line_; }
  // Command responsible for the current transaction.
  const Message* msg() const { return msg_; }
  // Set of Message Queues blocked on the current transaction.
  const std::vector<MessageQueue*>& bmqs() const { return bmqs_; }

  //
  void set_line(CCLineState* line) { line_ = line; }
  //
  void set_msg(const Message* msg) { msg_ = msg; }
  // Add Blocked Message Queue
  void add_bmq(MessageQueue* mq) { bmqs_.push_back(mq); }

 private:
  CCLineState* line_ = nullptr;
  // Set of message queues block on the current transaction.
  std::vector<MessageQueue*> bmqs_;
  //
  const Message* msg_ = nullptr;
};

//
//
using CCTTable = Table<Transaction*, CCTState*>;

//
//
enum class CCWait {
  Invalid,
  //
  MsgArrival,
  //
  NextEpochIfHasRequestOrWait
};

//
const char* to_string(CCWait w);

//
//
class CCContext {
 public:
  CCContext() = default;

  //
  bool owns_line() const { return owns_line_; }
  bool stalled() const { return stalled_; }
  bool dequeue() const { return dequeue_; }
  bool commits() const { return commits_; }
  bool ttadd() const { return ttadd_; }
  bool ttdel() const { return ttdel_; }
  const Message* msg() const { return msg_; }
  MessageQueue* mq() const { return mq_; }
  MQArbTmt t() const { return t_; }
  CoherenceActionList& actions() { return al_; }
  const CoherenceActionList& actions() const { return al_; }
  CC* cc() const { return cc_; }
  CCWait wait() const { return wait_; }
  CCLineState* line() const { return line_; }
  CCTState* st() const { return st_; }

  //
  void set_owns_line(bool owns_line) { owns_line_ = owns_line; }
  void set_stalled(bool stalled) { stalled_ = stalled; }
  void set_dequeue(bool dequeue) { dequeue_ = dequeue; }
  void set_commits(bool commits) { commits_ = commits; }
  void set_ttadd(bool ttadd) { ttadd_ = ttadd; }
  void set_ttdel(bool ttdel) { ttdel_ = ttdel; }
  void set_msg(const Message* msg) { msg_ = msg; }
  void set_mq(MessageQueue* mq) { mq_ = mq; }
  void set_t(MQArbTmt t) { t_ = t; }
  void set_wait(CCWait wait) { wait_ = wait; }
  void set_line(CCLineState* line) { line_ = line; }
  void set_cc(CC* cc) { cc_ = cc; }
  void set_st(CCTState* st) { st_ = st; }

 private:
  // Current message is stalled (cannot execute) for some protocol or
  // flow control related issue.
  bool stalled_ = false;
  // Dequeue message from associated message queue upon execution.
  bool dequeue_ = false;
  // Do commit context to L1 state and issue actions.
  bool commits_ = false;
  // Context owns the current line (on an install line) and therefore
  // must delete the line upon destruction.
  bool owns_line_ = false;
  // Transaction table ADD
  bool ttadd_ = false;
  // Transaction table DELete
  bool ttdel_ = false;
  // Current arbitration tournament.
  MQArbTmt t_;
  //
  CC* cc_ = nullptr;
  //
  CCLineState* line_ = nullptr;
  //
  CCTState* st_ = nullptr;
  // Message being processed.
  const Message* msg_ = nullptr;
  // Current Message Queue
  MessageQueue* mq_ = nullptr;
  //
  CoherenceActionList al_;
  // Wait condition.
  CCWait wait_ = CCWait::Invalid;
};

//
//
class CC : public Agent {
  friend class CpuCluster;

  class RdisProcess;
  class NocIngressProcess;

 public:
  CC(kernel::Kernel* k, const CCConfig& config);
  ~CC();

  // Obtain cache controller configuration.
  const CCConfig& config() const { return config_; }
  // L2 -> Controller (Transaction) Command Queue (owning)
  MessageQueue* l2_cc__cmd_q() const { return l2_cc__cmd_q_; }
  // CC -> L2 Queue
  MessageQueue* cc_l2__rsp_q() const { return cc_l2__rsp_q_; }
  // NOC -> CC Ingress Queue
  MessageQueue* noc_cc__msg_q() const { return noc_cc__msg_q_; }
  // CC -> NOC Egress Queue
  MessageQueue* cc_noc__msg_q() const { return cc_noc__msg_q_; }
  // Directory Mapper instance.
  DirMapper* dm() const { return dm_; }
  // L2 cache model
  L2CacheModel* l2c() const { return l2c_; }

 protected:
  // Accessors:
  // Pointer to module arbiter instance:
  MQArb* arb() const { return arb_; }
  // Protocol instance
  CCProtocol* protocol() const { return protocol_; }
  // Transaction table.
  CCTTable* table() const { return tt_; }

  // Construction
  void build();

  // Elaboration
  void elab();
  // Set slave L2C instance.
  void set_l2c(L2CacheModel* l2c) { l2c_ = l2c; }
  // Set directory mapper.
  void set_dm(DirMapper* dm) { dm_ = dm; }
  // Set CC -> NOC message queue
  void set_cc_noc__msg_q(MessageQueue* mq) { cc_noc__msg_q_ = mq; }
  /// Set CC -> L2 response queue
  void set_cc_l2__rsp_q(MessageQueue* mq) { cc_l2__rsp_q_ = mq; }

  // Design Rule Check (DRC)
  void drc();

  // lookup rdis process message queue for traffic class.
  MessageQueue* lookup_rdis_mq(MessageClass cls) const;

 private:
  // L2 Cache Model to which this controller is bound.
  L2CacheModel* l2c_ = nullptr;
  // L2 -> Controller (Transaction) Command Queue (owning)
  MessageQueue* l2_cc__cmd_q_ = nullptr;
  // CC -> L2 response queue
  MessageQueue* cc_l2__rsp_q_ = nullptr;
  // NOC -> CC Ingress Queue
  MessageQueue* noc_cc__msg_q_ = nullptr;
  // CC -> NOC Egress Queue
  MessageQueue* cc_noc__msg_q_ = nullptr;
  // DIR -> CC Ingress Queue
  MessageQueue* dir_cc__rsp_q_ = nullptr;
  // {LLC, CC} -> CC Data (dt) queue
  MessageQueue* cc__dt_q_ = nullptr;
  // Queue selection arbiter
  MQArb* arb_ = nullptr;
  // Directory Mapper instance
  DirMapper* dm_ = nullptr;
  // Disatpcher process
  RdisProcess* rdis_proc_ = nullptr;
  // NOC ingress process
  NocIngressProcess* noci_proc_ = nullptr;
  // Transaction table instance.
  CCTTable* tt_ = nullptr;
  // Cache controller protocol instance.
  CCProtocol* protocol_ = nullptr;
  // Cache controller configuration.
  CCConfig config_;
};

}  // namespace cc

#endif
