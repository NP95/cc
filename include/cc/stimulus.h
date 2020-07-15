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

#ifndef CC_INCLUDE_CC_STIMULUS_H
#define CC_INCLUDE_CC_STIMULUS_H

#include "kernel.h"
#include "types.h"
#include "cfgs.h"
#include <deque>
#include <set>
#include <string>
#include <map>

namespace cc {

// Forwards:
class Cpu;
class TraceStimulusContext;

enum class CpuOpcode {
  Invalid, Load, Store
};

std::string to_string(CpuOpcode opcode);

// Basic (load/store) command class.
//
class Command {
 public:
  Command() {}
  Command(CpuOpcode opcode, addr_t addr) : opcode_(opcode), addr_(addr) {}

  addr_t addr() const { return addr_; }
  CpuOpcode opcode() const { return opcode_; }

 private:
  addr_t addr_ = 0;
  CpuOpcode opcode_ = CpuOpcode::Invalid;
};

// TODO: cleanup.
// Transaction frontier record.
struct Frontier {
  kernel::Time time;
  Command cmd;
};

//
//
class StimulusContext : public kernel::Module {
 public:
  StimulusContext(kernel::Kernel* k, const std::string& name);
  virtual ~StimulusContext() = default;

  // Flag denoting whether the transaction source has been exhausted.
  virtual bool done() const { return true; }

  // Transaction at the head of the source queue; nullptr if source
  // has been exhausted.
  virtual bool front(Frontier& f) const = 0;

  // Consume transaction at the head of the source queue.
  virtual void consume() {}
  
};

// Abstract base class encapsulating the concept of a transaction
// source; more specifically, a block response to model the issue of
// of load or store instructions to a cache.
//
class Stimulus : public kernel::Module {
 public:
  Stimulus(kernel::Kernel* k, const StimulusCfg& config);
  virtual ~Stimulus() = default;

  // Accessors:
  const StimulusCfg& config() const { return config_; }

  // Register a new CPU instance.
  virtual StimulusContext* register_cpu(Cpu* cpu) { return nullptr; }

 private:
  // Configuration
  StimulusCfg config_;
};

// Trace file specification:
//
// Directives:
//
//  +(A)INTEGER
//
//  Advance the current time cursor by INTEGER "A" such that the next
//  set of commands shall be scheduled for issue at new time.
//
//  For example:
//
//    +100 // advance the current time cursor by 100 time-units.
//
//  C:(A)INTEGER:{LD,ST}:(B)INTEGER
//
//  Enqueue a Load (LD)/Store (ST) command to address (B) in CPU "A"'s
//  work-queue.
//
//  For example:
//
//    C:1:LD:1000 // Cpu 1 issued a Load to 0x1000 at current time.
//
//    C:0:ST:1000 // CPu 0 issues a Store to 0x1000 at current time.
//
class TraceStimulus : public Stimulus {
 public:
  TraceStimulus(kernel::Kernel* k, const StimulusCfg& config);
  ~TraceStimulus();
 private:

  // Phases
  void build();
  void elab() override;
  void drc() override;

  // Stimulus: 
  StimulusContext* register_cpu(Cpu* cpu) override;

  void parse_tracefile();

  std::vector<TraceStimulusContext*> compute_index_table();

  // Registered CPU
  std::map<Cpu*, TraceStimulusContext*> cpumap_;
  // Trace input
  std::istream* is_ = nullptr;
};

//
//
Stimulus* stimulus_builder(kernel::Kernel* k, const StimulusCfg& cfg);

}  // namespace cc

#endif
