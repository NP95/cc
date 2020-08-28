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

#ifndef CC_SRC_VERIF_H
#define CC_SRC_VERIF_H

#include <string>
#include <set>

#include "cc/kernel.h"
#include "cc/types.h"

namespace cc {

class Cpu;
class L1CacheAgent;
class L2CacheAgent;
class NocModel;
class DirAgent;
class CCAgent;
class Transaction;

class LineState;

// CPU Monitor Interface:
//
struct CpuMonitor {
  // Event denoting start of a new transaction.
  virtual void start_transaction_event(Cpu* cpu, Transaction* t) = 0;

  // Event denoting the end of a in-flight transaction.
  virtual void end_transaction_event(Cpu* cpu, Transaction* t) = 0;
};


// L1 Cache Monitor Interface:
//
struct L1CacheMonitor {
  // Read hit to address
  virtual void read_hit(L1CacheAgent* l1c, addr_t addr) = 0;

  // Write hit to address
  virtual void write_hit(L1CacheAgent* l1c, addr_t addr) = 0;
  
  // Install line in L1
  virtual void install_line(L1CacheAgent* l1c, addr_t addr,
                            bool is_writeable = false) = 0;

  // Remove line from L1; line is presently consistent with L2 by
  // definition, therefore no prior writeback is requried.
  virtual void remove_line(L1CacheAgent* l1c, addr_t addr) = 0;
};

// Verification Monitor.
//
// Equivalent to a testbench monitor which continuouslty probes events
// and state within the simulation model to validate simulation
// "invariants" (conditions that must be true for correctness to be
// maintained).
//
class Monitor : public kernel::Module,
                public CpuMonitor,
                public L1CacheMonitor {
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
  void start_transaction_event(Cpu* cpu, Transaction* t) override;

  // Notify end of transaction.
  void end_transaction_event(Cpu* cpu, Transaction* t) override;


  // L1Cache Interface:
  //

  // Read hit to address
  void read_hit(L1CacheAgent* l1c, addr_t addr) override;

  // Write hit to address
  void write_hit(L1CacheAgent* l1c, addr_t addr) override;

  // Install line in L1
  void install_line(L1CacheAgent* l1c, addr_t addr,
                    bool is_writeable = false) override;
                    
  // Remove line from L1; line is presently consistent with L2 by
  // definition, therefore no prior writeback is requried.
  void remove_line(L1CacheAgent* l1c, addr_t addr) override;
  

  // L2Cache Interface:
  //
  
  // Directory Interface:
  //


 private:
  // Current inflight transaction set.
  std::set<Transaction*> ts_;

  // Set of registered CPU
  std::set<Cpu*> cpus_;

  // Set of registered L1 caches
  std::set<L1CacheAgent*> l1cs_;

  // Set of registered L2 caches
  std::set<L2CacheAgent*> l2cs_;

  // Set of registered NOCs (interconnect.), nominally 1.
  std::set<NocModel*> nocs_;

  // Set of registered Directories
  std::set<DirAgent*> dirs_;

  // Set of registered Cache Controllers
  std::set<CCAgent*> ccs_;

  // Cache line registry.
  std::map<addr_t, LineState*> line_registry_;
};


} // namespace cc

#endif
