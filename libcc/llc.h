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

#ifndef CC_LIBCC_LLC_H
#define CC_LIBCC_LLC_H

#include "kernel.h"
#include "primitives.h"
#include "cfgs.h"

namespace cc {

// Forwards:
class MemModel;
class Message;
class MessageQueue;
template<typename T> class Arbiter;

//
//
class LLCModel : public Agent {
  class MainProcess;

  friend class SocTop;
 public:
  LLCModel(kernel::Kernel* k, const LLCModelConfig& config);
  ~LLCModel();

  // Return model configuration.
  const LLCModelConfig& config() const { return config_; }

  // Accessors:
  // NOC -> LLC message queue
  MessageQueue* noc_llc__msg_q() const { return noc_llc__msg_q_; }
  // LLC -> NOC message queue
  MessageQueue* llc_noc__msg_q() const { return llc_noc__msg_q_; }
  // Home memory controller
  MemModel* mc() const { return mc_; }
  
 protected:
  // Construction/Build
  void build();

  // Elaboration
  void elab() override;
  // NOC -> LLC message queue
  void set_llc_noc__msg_q(MessageQueue* mq) { llc_noc__msg_q_ = mq; }
  // Set memory controller.
  void set_mc(MemModel* mc) { mc_ = mc; }

  // Design Rule Check
  void drc() override;

  // Accessors:

  // Queue arbiter:
  Arbiter<const Message*>* arb() const { return arb_; }
  
 private:
  // LLC -> NOC command queue (NOC owned)
  MessageQueue* llc_noc__msg_q_ = nullptr;
  // NOC -> LLC response queue (LLC owned)
  MessageQueue* noc_llc__msg_q_ = nullptr;
  // Queue selector arbiter
  Arbiter<const Message*>* arb_ = nullptr;
  // Home memory controller
  MemModel* mc_ = nullptr;
  // Main thread 
  MainProcess* main_ = nullptr;
  // LLC Cache configuration
  LLCModelConfig config_;
};

} // namespace cc

#endif
