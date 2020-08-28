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

#ifndef CC_SRC_STATS_H
#define CC_SRC_STATS_H

#include "cc/kernel.h"
#include <set>
#include <map>

namespace cc {

class Cpu;
class L1CacheAgent;
class L2CacheAgent;
class NocModel;
class DirAgent;
class CCAgent;


struct CpuStatistics {
  // Start transaction
  virtual void start_transaction_event(Cpu* cpu) = 0;

  // End transaction
  virtual void end_transaction_event(Cpu* cpu) = 0;
};


struct CpuStatisticsState {
  // Convert to humand-readable format.
  std::string to_string() const;
  
  // Total number of transaction that have completed.
  std::size_t transaction_count = 0;
};



struct L1CacheStatistics {
  enum Event {
    StoreHit, LoadHit, StoreMiss, LoadMiss
  };

  virtual void event(Event event, L1CacheAgent* l1c) = 0;
};

struct L1CacheStatisticsState {
  // Convert to humand-readable format.
  std::string to_string() const;
  
  // Total number of store hits
  std::size_t store_hit_n = 0;

  // Total number of store misses.
  std::size_t store_miss_n = 0;

  // Total number of load hits
  std::size_t load_hit_n = 0;

  // total number of load misses.
  std::size_t load_miss_n = 0;
};


class Statistics : public kernel::Module,
                   public CpuStatistics,
                   public L1CacheStatistics {
 public:
  Statistics(kernel::Kernel* k, const std::string& name);
  virtual ~Statistics();

  // Accumulated statistics:

  // CPU statistics:
  const CpuStatisticsState* cpu_state(Cpu* cpu) const;

  // L1 statistics
  const L1CacheStatisticsState* l1c_state(L1CacheAgent* l1c) const;

  
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


  // CPU statistics:
  //

  // Start transaction
  void start_transaction_event(Cpu* cpu) override;

  // End transaction
  void end_transaction_event(Cpu* cpu) override;

  // L1 Cache statistics:
  //
  void event(L1CacheStatistics::Event event, L1CacheAgent* agent) override;

  
  // Write hit
  // void write_hit_event(L1CacheAgent* l1c) override;

  // Read hit
  //void read_hit_event(L1CacheAgent* l1c) override;
  

  // L2 Cache statistics:
  //
  // TBD

  // NOC statistics:
  //
  // TBD

  // Directory statistics:
  //
  // TBD

  // Cache Controller statistics:
  //
  // TBD


 private:
  void report_statistics() const;
  
  // Set of register CPU instances.
  std::set<Cpu*> cpus_;
  
  // CPU instance -> statistics map
  std::map<Cpu*, CpuStatisticsState*> cpu_stats_;

  // Set of registered L1 cache instances
  std::set<L1CacheAgent*> l1cs_;

  // L1 Cache -> statistic map
  std::map<L1CacheAgent*, L1CacheStatisticsState*> l1c_stats_;

  // Reporter process
  kernel::Process* reporter_ = nullptr;
};

} // namespace cc

#endif
