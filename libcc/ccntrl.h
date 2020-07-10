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

namespace cc {

class Message;
class MessageQueue;
class L2CacheModel;
class CacheControllerLineState;
class CacheControllerProtocol;

class CacheController : public Agent {
  friend class CpuCluster;
  class MainProcess;
 public:
  CacheController(kernel::Kernel* k, const CacheControllerCfg& config);

  // Obtain cache controller configuration.
  const CacheControllerCfg& config() const { return config_; }

 protected:

  // Accessors:
  // Pointer to module arbiter instance:
  Arbiter<const Message*>* arb() const { return arb_; }
  // Directory Mapper instance.
  DirectoryMapper* dm() const { return dm_; }
  // Protocol instance
  CacheControllerProtocol* protocol() const { return protocol_; }
  // Transaction table.
  Table<CacheControllerLineState*>* table() const { return table_; }
  // L2 -> Controller (Transaction) Command Queue (owning)
  MessageQueue* l2_cc__cmd_q() const { return l2_cc__cmd_q_; }
  // NOC -> CC Ingress Queue
  MessageQueue* noc_cc__msg_q() const { return noc_cc__msg_q_; }
  // CC -> NOC Egress Queue
  MessageQueue* cc_noc__msg_q() const { return cc_noc__msg_q_; }
  
  // Construction
  void build();

  // Elaboration
  void elab();
  // Set slave L2C instance.
  void set_l2c(L2CacheModel* l2c) { l2c_ = l2c; }
  // Set directory mapper.
  void set_dm(DirectoryMapper* dm) { dm_ = dm; }
  // Set CC -> NOC message queue
  void set_cc_noc__msg_q(MessageQueue* mq) { cc_noc__msg_q_ = mq; }
  
  // Design Rule Check (DRC)
  void drc();

  
 private:
  // L2 Cache Model to which this controller is bound.
  L2CacheModel* l2c_ = nullptr;
  // L2 -> Controller (Transaction) Command Queue (owning)
  MessageQueue* l2_cc__cmd_q_ = nullptr;
  // NOC -> CC Ingress Queue
  MessageQueue* noc_cc__msg_q_ = nullptr;
  // CC -> NOC Egress Queue
  MessageQueue* cc_noc__msg_q_ = nullptr;
  // Queue selection arbiter
  Arbiter<const Message*>* arb_ = nullptr;
  // Directory Mapper instance
  DirectoryMapper* dm_ = nullptr;
  // Main process instance
  MainProcess* main_ = nullptr;
  // Transaction table instance.
  Table<CacheControllerLineState*>* table_ = nullptr;
  // Cache controller protocol instance.
  CacheControllerProtocol* protocol_ = nullptr;
  // Cache controller configuration.
  CacheControllerCfg config_;
};

} // namespace cc

#endif
