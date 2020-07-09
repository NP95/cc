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

#ifndef CC_INCLUDE_CC_LLC_H
#define CC_INCLUDE_CC_LLC_H

#include "kernel.h"
#include "cfgs.h"
//#include "primitives.h"

namespace cc {
/*
// Forwards:
class Message;
class MessageQueue;
template<typename T> class Arbiter;

#define LLC_MESSAGE_QUEUES(__func)              \
  __func(CmdQ)                                  \
  __func(RspQ)

enum class LLCEp : kernel::end_point_id_t {
#define __declare_enum(__name) __name,
  LLC_MESSAGE_QUEUES(__declare_enum)
#undef __declare_enum
};

const char* to_string(LLCEp d);

//
//
class LLCModel : public Agent {
  class MainProcess;
 public:
  LLCModel(kernel::Kernel* k, const LLCModelConfig& config);

  // Return model configuration.
  const LLCModelConfig& config() const { return config_; }

 protected:
  // Construction/Build
  void build();

  // Elaboration
  void elab() override;

  // Design Rule Check
  void drc() override;

  // Accessors:

  // Queue arbiter:
  Arbiter<const Message*>* arb() const { return arb_; }
  
 private:
  // Command message queue
  MessageQueue* cmdq_ = nullptr;
  // Response message queue
  MessageQueue* rspq_ = nullptr;
  // Queue selector arbiter
  Arbiter<const Message*>* arb_ = nullptr;
  // Main thread 
  MainProcess* main_ = nullptr;
  // LLC Cache configuration
  LLCModelConfig config_;
};

*/
} // namespace cc

#endif
