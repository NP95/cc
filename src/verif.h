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

#ifndef CC_LIBCC_SRC_VERIF_H
#define CC_LIBCC_SRC_VERIF_H

#include "cc/kernel.h"
#include <string>

namespace cc {

class Cpu;
class L1CacheAgent;
class L2CacheAgent;
class NocModel;
class DirAgent;
class CCAgent;
class Transaction;

struct CpuMonitor {

  virtual void start_transaction_event(Transaction* t) = 0;

  virtual void end_transaction_event(Transaction* t) = 0;
};


// Verification Monitor.
//
// Equivalent to a testbench monitor which continuouslty probes events
// and state within the simulation model to validate simulation
// "invariants" (conditions that must be true for correctness to be
// maintained).
//
class Monitor : public kernel::Module,
                public CpuMonitor {
 public:
  Monitor(kernel::Kernel* k, const std::string& name);
  virtual ~Monitor() = default;

  // Registration methods:
  
  // Register CPU instance.
  void register_client(Cpu* cpu);

  // Register L1 cache instance.
  void register_client(L1CacheAgent* l1c);

  // Register L2 cache instance.
  void register_client(L2CacheAgent* l2c);

  // Register NOC agent instance.
  void register_client(NocModel* noc);

  // Register Directory agent instance.
  void register_client(DirAgent* dir);

  // Register Cache Controller agent instance.
  void register_client(CCAgent* cc);


  // CPU Interface:
  //

  // Notify start of transaction.
  void start_transaction_event(Transaction* t) override;

  // Notify end of transaction.
  void end_transaction_event(Transaction* t) override;


  // L1Cache Interface:
  //

  // L2Cache Interface:
  //
  
  // Directory Interface:
  //


 private:
  // Current inflight transaction set.
  std::set<Transaction*> ts_;
};


} // namespace cc

#endif
