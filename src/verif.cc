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

#include "verif.h"
#include "l1cache.h"
#include "l2cache.h"
#include "cpu.h"
#include "log.h"

namespace cc {

class LineState {
  template <typename>
  friend class PooledItem;

  LineState() = default;
 public:
  virtual ~LineState() = default;
  virtual void release() const = 0;

  // Accessors:

  // Cache line address
  addr_t addr() const { return addr_; }

  // Owning agent.
  L1CacheAgent* owner_l1c() const { return owning_l1c_; }

  // Sharer set.
  const std::set<L1CacheAgent*>& sharer_l1cs() const { return sharing_l1c_; }

  // Setters:

  // Set address:
  void set_addr(addr_t addr) { addr_ = addr; }

  // Set owning L1 instance.
  void set_owner(L1CacheAgent* owning_l1c) { owning_l1c_ = owning_l1c; }

  // Sharer set
  std::set<L1CacheAgent*>& sharer_l1c() { return sharing_l1c_; }
  

 private:

  // Current owning L1 cache; sharing set must be empty.
  L1CacheAgent* owning_l1c_ = nullptr;

  // The set of sharing L1 caches.
  std::set<L1CacheAgent*> sharing_l1c_;

  // Line address;
  addr_t addr_;
};

Monitor::Monitor(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {
}

// Register CPU instance
void Monitor::register_client(Cpu* cpu) {
  if (auto it = cpus_.insert(cpu); !it.second) {
    LOG_FATAL("Attempt to re-register CPU instance.");
  }
}

// Register L1 cache instance
void Monitor::register_client(L1CacheAgent* l1c) {
  if (auto it = l1cs_.insert(l1c); !it.second) {
    LOG_FATAL("Attempt to re-register L1 cache instance.");
  }
}

// Register L2 cache instance
void Monitor::register_client(L2CacheAgent* l2c) {
  if (auto it = l2cs_.insert(l2c); !it.second) {
    LOG_FATAL("Attempt to re-register L2 cache instance.");
  }
}

// Register NOC agent instance.
void Monitor::register_client(NocModel* noc) {
  if (auto it = nocs_.insert(noc); !it.second) {
    LOG_FATAL("Attempt to re-register NOC instance.");
  }
}

// Register Directory agent instance.
void Monitor::register_client(DirAgent* dir) {
  if (auto it = dirs_.insert(dir); !it.second) {
    LOG_FATAL("Attempt to re-register Directory instance.");
  }
}

// Register Cache Controller agent instance.
void Monitor::register_client(CCAgent* cc) {
  if (auto it = ccs_.insert(cc); !it.second) {
    LOG_FATAL("Attempt to re-register Cache Controller instance.");
  }
}

// Notify start of transaction.
void Monitor::start_transaction_event(Cpu* cpu, Transaction* t) {
  if (auto it = cpus_.find(cpu); it == cpus_.end()) {
    LOG_FATAL("Unknown CPU instance.");
  }
  if (auto it = ts_.insert(t); !it.second) {
    // Transaction could not be inserted, currently already present in
    // the transaction set. Attempt to reissuing transaction which
    // is already in flight.
    LOG_FATAL("Attempt to register transaction which is already in flight.");
  }
}

// Notify end of transaction.
void Monitor::end_transaction_event(Cpu* cpu, Transaction* t) {
  if (auto it = cpus_.find(cpu); it == cpus_.end()) {
    LOG_FATAL("Unknown CPU instance.");
  }
  if (auto it = ts_.find(t); it != ts_.end()) {
    ts_.erase(it);
  } else {
    // Transaction is not present; perhaps perviously consumed?
    LOG_FATAL("Attempt to deregister invalid transaction.");
  }
}

void Monitor::read_hit(L1CacheAgent* l1c, addr_t addr) {
  // TODO: Verify that line is aligned.
  if (auto line_it = line_registry_.find(addr); line_it != line_registry_.end()) {
    LineState* lstate = line_it->second;
    if (lstate->owner_l1c() != l1c && (lstate->sharer_l1cs().count(l1c) == 0)) {
      // Cache is neither the owner, nor in the sharer set; it should
      // therefore not have the line installed in the cache.
      LOG_FATAL("Read hit to line; but line should not be present in "
                "indicate cache instance.");
    }
  } else {
    // Line is not presently registered, cannot therefore hit to
    // the line in the cache.
    LOG_FATAL("Line is not present in behavorial model.");
  }
}

void Monitor::write_hit(L1CacheAgent* l1c, addr_t addr) {
  // TODO: Verify that line is aligned.
  if (auto line_it = line_registry_.find(addr); line_it != line_registry_.end()) {
    LineState* lstate = line_it->second;
    if (lstate->owner_l1c() != l1c) {
      LOG_FATAL("Write hit to line; but line is not in a writeable state "
                "in the indicated cache instance.");
    }
  } else {
    // Line is not presently registered, cannot therefore hit to
    // the line in the cache.
    LOG_FATAL("Line is not present in behavorial model.");
  }
}

void Monitor::install_line(L1CacheAgent* l1c, addr_t addr,
                           bool is_writeable) {
  // TODO: Verify that line is aligned.
  LineState* lstate = nullptr;
  if (auto line_it = line_registry_.find(addr); line_it == line_registry_.end()) {
    lstate = Pool<LineState>::construct();
    lstate->set_addr(addr);
    line_registry_[addr] = lstate;
  } else {
    lstate = line_it->second;
  }
  if (is_writeable) {
    // Current L1 becomes owner of the line.
    lstate->set_owner(l1c);
  } else {
    // Otherwise, L1 becomes a sharer of the line.
    auto& sharer_set = lstate->sharer_l1c();
    if (auto set_it = sharer_set.insert(l1c); !set_it.second) {
      // L1 Cache is alreayd present in sharer set; cannot re-install.
      LOG_FATAL("L1C is already present in associated sharer set.");
    }
  }
  
}
                    
void Monitor::remove_line(L1CacheAgent* l1c, addr_t addr) {
  // TODO: Verify that line is aligned.
  LineState* lstate = nullptr;
  if (auto line_it = line_registry_.find(addr); line_it != line_registry_.end()) {
    lstate = line_it->second;
    if (lstate->owner_l1c() == l1c) {
      // L1 is current designated owner. Delete,
      lstate->set_owner(nullptr);
    } else {
      // Otherwise, expect to be present in the sharer set.
      auto& sharer_set = lstate->sharer_l1c();
      if (auto set_it = sharer_set.find(l1c); set_it != sharer_set.end()) {
        sharer_set.erase(set_it);
      } else {
        // Line is not present in sharer set;
        LOG_FATAL("Expect to find L1C in line's sharer set, but cache has "
                  "not been registered.");
      }
    }
  } else {
    // Line does not appear to be installed in Monitor's state.
    LOG_FATAL("Attempt to uninstall a line which is not present in "
              "Monitors state table (has line already been removed?).");
  }

  // Check line state
  if (lstate != nullptr) {
    const bool has_no_owner = lstate->owner_l1c() == nullptr;
    const bool has_no_sharers = lstate->sharer_l1c().empty();
    if (has_no_owner && has_no_sharers) {
      // Fault on this; debug.
      // lstate->release();
    }
  }
}

} // namespace cc

