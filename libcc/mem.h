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

#ifndef CC_LIBC_DIR_H
#define CC_LIBC_DIR_H

#include "msg.h"
#include "sim.h"
#include <map>

namespace cc {


// Memory Controller Model
//
class MemCntrlModel : public Agent {
  class NocIngressProcess;
  class RequestDispatcherProcess;
  
  friend class SocTop;
 public:
  MemCntrlModel(kernel::Kernel* k);
  ~MemCntrlModel();

  // Accessors:

  // NOC -> MEM
  MessageQueue* noc_mem__msg_q() const { return noc_mem__msg_q_; }
  // MEM -> NOC
  MessageQueue* mem_noc__msg_q() const { return mem_noc__msg_q_; }

 protected:
  // Build
  void build();
  // Add a memory controller client to the current instance;
  // constructs the necessary ingress queues beyond the NOC interface
  // front-end.
  void register_agent(Agent* agent);

  // Elaboration
  void elab() override;
  //
  void set_mem_noc__msg_q(MessageQueue* mq) { mem_noc__msg_q_ = mq; }

  // Design Rule Check (DRC)
  void drc() override;

  // Accessors(s)

  // RDIS aribter
  MessageQueueArbiter* rdis_arb() const { return rdis_arb_; }

  MessageQueue* lookup_rdis_mq(Agent* agent);

 private:
  // NOC -> MEM message queue (owned by directory)
  MessageQueue* noc_mem__msg_q_ = nullptr;
  // MEM -> NOC message queue (owned by noc)
  MessageQueue* mem_noc__msg_q_ = nullptr;
  
  // NOC Ingress Queue thread
  NocIngressProcess* noci_proc_ = nullptr;
  
  // Request Dispatcher process
  RequestDispatcherProcess* rdis_proc_ = nullptr;
  // Request Dispatcher arbitrator
  MessageQueueArbiter* rdis_arb_ = nullptr;
  // Request Dispatcher memory queues
  std::map<Agent*, MessageQueue*> rdis_mq_;
};

} // namespace cc

#endif
