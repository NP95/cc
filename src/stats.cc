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

#include "stats.h"
#include "l1cache.h"
#include "l2cache.h"
#include "cpu.h"
#include "dir.h"
#include "ccntrl.h"

namespace cc {

std::string CpuStatisticsState::to_string() const {
  using std::to_string;
  KVListRenderer r;
  r.add_field("transaction_count", to_string(transaction_count));
  return r.to_string();
}

std::string L1CacheStatisticsState::to_string() const {
  using std::to_string;
  KVListRenderer r;
  r.add_field("store_hit_n", to_string(store_hit_n));
  r.add_field("store_miss_n", to_string(store_miss_n));
  r.add_field("load_hit_n", to_string(load_hit_n));
  r.add_field("load_miss_n", to_string(load_miss_n));
  return r.to_string();
}

Statistics::Statistics(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {
  struct ReporterProcess : kernel::Process {
    ReporterProcess(kernel::Kernel* k, const std::string& name)
        : Process(k, name) {}
    // 
    void set_statistics(Statistics* s) { s_ = s; }

    // Callbacks
    void fini() override {
      s_->report_statistics();
    }
    
   private:
    Statistics* s_ = nullptr;
  };
  
  ReporterProcess* r = new ReporterProcess(k, "reporter");
  r->set_statistics(this);
  reporter_ = r;
  add_child_process(r);
}

Statistics::~Statistics() {
  // Destruct CPU statistics
  for (const std::pair<Cpu*, CpuStatisticsState*>& p : cpu_stats_) {
    delete p.second;
  }
  // Destruct L1 cache statistics
  for (const std::pair<L1CacheAgent*, L1CacheStatisticsState*>& p : l1c_stats_) {
    delete p.second;
  }
  delete reporter_;
}

const CpuStatisticsState* Statistics::cpu_state(Cpu* cpu) const {
  const CpuStatisticsState* cstate = nullptr;
  if (auto it = cpu_stats_.find(cpu); it != cpu_stats_.end()) {
    cstate = it->second;
  }
  return cstate;
}

const L1CacheStatisticsState* Statistics::l1c_state(L1CacheAgent* l1c) const {
  const L1CacheStatisticsState* lstate = nullptr;
  if (auto it = l1c_stats_.find(l1c); it != l1c_stats_.end()) {
    lstate = it->second;
  }
  return lstate;
}

// Register CPU instance
void Statistics::register_client(Cpu* cpu) {
  cpus_.insert(cpu);

  cpu_stats_[cpu] = new CpuStatisticsState;
}

// Register L1 cache instance
void Statistics::register_client(L1CacheAgent* l1c) {
  l1cs_.insert(l1c);

  l1c_stats_[l1c] = new L1CacheStatisticsState;
}

// Register L2 cache instance
void Statistics::register_client(L2CacheAgent* l2c) {
}

// Register NOC agent instance.
void Statistics::register_client(NocModel* noc) {
}

// Register Directory agent instance.
void Statistics::register_client(DirAgent* dir) {
}

// Register Cache Controller agent instance.
void Statistics::register_client(CCAgent* cc) {
}

void Statistics::start_transaction_event(Cpu* cpu) {
}

void Statistics::end_transaction_event(Cpu* cpu) {
  if (auto it = cpu_stats_.find(cpu); it != cpu_stats_.end()) {
    CpuStatisticsState* s = it->second;
    s->transaction_count++;
  } else {
    // CPU not found; error out.
  }
}

void Statistics::event(L1CacheStatistics::Event event, L1CacheAgent* l1c) {
  if (auto it = l1c_stats_.find(l1c); it != l1c_stats_.end()) {
    L1CacheStatisticsState* s = it->second;
    switch (event) {
      case L1CacheStatistics::StoreHit: {
        s->store_hit_n++;
      } break;
      case L1CacheStatistics::LoadHit: {
        s->load_hit_n++;
      } break;
      case L1CacheStatistics::StoreMiss: {
        s->store_miss_n++;
      } break;
      case L1CacheStatistics::LoadMiss: {
        s->load_miss_n++;
      } break;
    }
  } else {
    // 
  }
}

void Statistics::report_statistics() const {
  // Report CPu stastics
  for (const auto& p : cpu_stats_) {
    LogMessage msg("CPU statistics path ");
    msg.append(p.first->path());
    msg.append(": ");
    msg.append(p.second->to_string());
    log(msg);
  }
  // Report L1 Cache statistics
  for (const auto& p : l1c_stats_) {
    LogMessage msg("L1 Cache statistics ");
    msg.append(p.first->path());
    msg.append(": ");
    msg.append(p.second->to_string());
    log(msg);
  }
}

} // namespace cc
