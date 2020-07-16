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

#include "cc/kernel.h"
#include "cc/cfgs.h"
#include "sim.h"
#include "msg.h"

namespace cc {

class MessageQueue;
class L2CacheModel;
class CCLineState;
class CCProtocol;

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

 protected:

  // Accessors:
  // Pointer to module arbiter instance:
  MessageQueueArbiter* arb() const { return arb_; }
  // Protocol instance
  CCProtocol* protocol() const { return protocol_; }
  // Transaction table.
  Table<CCLineState*>* table() const { return table_; }
  
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
  // Queue selection arbiter
  MessageQueueArbiter* arb_ = nullptr;
  // Directory Mapper instance
  DirMapper* dm_ = nullptr;
  // Disatpcher process
  RdisProcess* rdis_proc_ = nullptr;
  // NOC ingress process
  NocIngressProcess* noci_proc_ = nullptr;
  // Transaction table instance.
  Table<CCLineState*>* table_ = nullptr;
  // Cache controller protocol instance.
  CCProtocol* protocol_ = nullptr;
  // Cache controller configuration.
  CCConfig config_;
};

} // namespace cc

#endif
