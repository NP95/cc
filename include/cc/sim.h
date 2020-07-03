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

#ifndef CC_INCLUDE_CC_SIM_H
#define CC_INCLUDE_CC_SIM_H

#include "kernel.h"
#include "types.h"
#include <vector>
#include <deque>
#include <set>
#include <string>

namespace cc {

// Basic (load/store) command class.
//
class Command {
 public:
  enum Opcode { Load, Store };

  Command(Opcode opcode, addr_t addr)
      : opcode_(opcode), addr_(addr)
  {}

  addr_t addr() const { return addr_; }
  Opcode opcode() const { return opcode_; }

 private:
  addr_t addr_;
  Opcode opcode_;
};

// Abstract base class encapsulating the concept of a transaction
// source; more specifically, a block response to model the issue of
// of load or store instructions to a cache.
//
class Stimulus {
 public:

  // Transaction frontier record.
  struct Frontier {
    kernel::Time time;
    Command cmd;
  };
  
  Stimulus() = default;
  virtual ~Stimulus() = default;

  // Flag denoting whether the transaction source has been exhausted.
  virtual bool done() const { return true; }
  
  // Transaction at the head of the source queue; nullptr if source
  // has been exhausted.
  virtual Frontier& front() = 0;
  virtual const Frontier& front() const = 0;

  // Consume transaction at the head of the source queue.
  virtual void consume() {}
};


// Elementary realization of a transaction source. Transactions are
// programmatically constructed and issued to the source before the
// start of the simulation. Upon exhaustion of the transactions
// the source remained exhausted for the duration of the simulation.
//
class ProgrammaticStimulus : public Stimulus {
 public:
  ProgrammaticStimulus() {}

  bool done() const override { return cs_.empty(); }
  Frontier& front() override { return cs_.front(); }
  const Frontier& front() const override { return cs_.front(); }

  void consume() override { cs_.pop_front(); }
  void push_back(kernel::Time t, const Command& c) { cs_.push_back({t, c}); }
 private:
  std::deque<Frontier> cs_;
};

} // namespace cc

#endif
