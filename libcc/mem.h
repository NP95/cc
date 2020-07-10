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

#include "primitives.h"
#include <vector>

namespace cc {

class MemModel : public Agent {
  class MainProcess;

  friend class SocTop;
 public:
  MemModel(kernel::Kernel* k);
  ~MemModel();

  // Accessors:

  // NOC -> MEM
  MessageQueue* noc_mem__msg_q() const { return noc_mem__msg_q_; }

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
  Arbiter<const Message*>* arb() const { return arb_; }

 private:
  // NOC -> MEM message queue (owned by directory)
  MessageQueue* noc_mem__msg_q_ = nullptr;
  // MEM -> NOC message queue (owned by noc)
  MessageQueue* mem_noc__msg_q_ = nullptr;
  // Queue selector arbiter
  Arbiter<const Message*>* arb_ = nullptr;
  // Main trhead
  MainProcess* main_ = nullptr;
  // Client agents (agents that interact with the memory controller).
  std::vector<Agent*> clients_;
};

} // namespace cc

#endif
