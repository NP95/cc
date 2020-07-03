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

#ifndef CC_INCLUDE_CC_L1CACHE_H
#define CC_INCLUDE_CC_L1CACHE_H

#include <string>
#include "kernel.h"

namespace cc {

class Stimulus;
class Message;
class MessageQueue;
template<typename> class Arbiter;
class ProcessorModel;

//
//
struct L1CacheModelConfig {
  // L1 Cache Model name
  std::string name = "l1cache";

  // Pointer to the transaction source instance for the current l1
  // cache instance (models the notion of a microprocessor
  // periodically emitting load/store instructions to memory).
  Stimulus* stim = nullptr;

  // LD/ST pipe flush penalty (cycles)
  std::size_t ldst_flush_penalty_n = 3;
};

//
//
class L1CacheModel : public kernel::Module {
  class MainProcess;
 public:
  L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config);
  virtual ~L1CacheModel();

  L1CacheModelConfig config() const { return config_; }
 protected:
  virtual void elab() override;
  virtual void drc() override;
 private:
  void build();
  // L1 Cache stimulus (models the concept of a processor data path
  // emitting instructions into the cache as part of a programs
  // execution).
  ProcessorModel* proc_;
  // Associated message queues for coherency messages.
  std::vector<MessageQueue*> mqs_;
  // Message servicing arbiter.
  Arbiter<const Message*>* arb_;
  // Main process of execution.
  MainProcess* main_;
  // Cache Instance
  // TODO
  // Cache configuration.
  L1CacheModelConfig config_;
};

} // namespace cc

#endif
