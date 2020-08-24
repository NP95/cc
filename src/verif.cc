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

namespace cc {

Monitor::Monitor(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {
}

// Register CPU instance
void Monitor::register_client(Cpu* cpu) {
}

// Register L1 cache instance
void Monitor::register_client(L1CacheAgent* l1c) {
}

// Register L2 cache instance
void Monitor::register_client(L2CacheAgent* l2c) {
}

// Register NOC agent instance.
void Monitor::register_client(NocModel* noc) {
}

// Register Directory agent instance.
void Monitor::register_client(DirAgent* dir) {
}

// Register Cache Controller agent instance.
void Monitor::register_client(CCAgent* cc) {
}

// Notify start of transaction.
void Monitor::start_transaction_event(Transaction* t) {
  if (auto it = ts_.insert(t); !it.second) {
    // Transaction could not be inserted, currently already present in
    // the transaction set. Attempt to reissuing transaction which
    // is already in flight.
    LOG_FATAL("Attempt to register transaction which is already in flight.");
  }
}

// Notify end of transaction.
void Monitor::end_transaction_event(Transaction* t) {
  if (auto it = ts_.find(t); it != ts_.end()) {
    ts_.erase(it);
  } else {
    // Transaction is not present; perhaps perviously consumed?
    LOG_FATAL("Attempt to deregister invalid transaction.");
  }
}

} // namespace cc

