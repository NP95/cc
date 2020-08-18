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
#include <exception>

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

// Stimulus Exception class
//
class StimulusException : public std::runtime_error {
 public:
  StimulusException(const std::string& reason);
};

//
//
class StimulusContext : public kernel::Module {
 public:
  //
  StimulusContext(Stimulus* parent, kernel::Kernel* k, const std::string& name);
  //
  virtual ~StimulusContext();

  // Pointer to parent Stimulus instance.
  Stimulus* parent() const { return parent_; }

  // Further stimulus has arrived at context.
  kernel::Event* non_empty_event() const {
    return non_empty_event_;
  }

  // Partner CPU instance.
  void set_cpu(const Cpu* cpu) { cpu_ = cpu; }

  // Flag denoting whether the transaction source has been exhausted.
  virtual bool done() const { return true; }

  // Transaction at the head of the source queue; nullptr if source
  // has been exhausted.
  virtual bool front(Frontier& f) const = 0;

  // Consume transaction at the head of the source queue.
  virtual void issue();

  // Retire transaction
  virtual void retire();

 private:
  // Stimulus parent.
  Stimulus* parent_ = nullptr;

  // Partner Cpu instance.
  const Cpu* cpu_ = nullptr;

  // Event indiciating that additional stimulus queue is non-empty.
  kernel::Event* non_empty_event_ = nullptr;
};

// Forward of Dequee-based context
class DequeueContext;

// Abstract base class encapsulating the concept of a transaction
// source; more specifically, a block response to model the issue of
// of load or store instructions to a cache.
//
class Stimulus : public kernel::Module {
  friend class StimulusContext;
 public:
  Stimulus(kernel::Kernel* k, const StimulusConfig& config);
  virtual ~Stimulus() = default;

  // Accessors:
  const StimulusConfig& config() const { return config_; }

  // Register a new CPU instance.
  virtual StimulusContext* register_cpu(Cpu* cpu) { return nullptr; }

  // Total issue count
  std::size_t issue_n() const { return issue_n_; }

  // Total retire count
  std::size_t retire_n() const { return retire_n_; }

 private:
  // 'context' issues a transaction.
  virtual void issue(StimulusContext* context);

  // 'context' retires a transaction.
  virtual void retire(StimulusContext* context);

  // Total issue count
  std::size_t issue_n_ = 0;
  // Total retire count
  std::size_t retire_n_ = 0;
  // Configuration
  StimulusConfig config_;
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

  // Construct appropriate trace configuration-file from some string.
  static StimulusConfig from_string(const std::string& s);
  
  TraceStimulus(kernel::Kernel* k, const StimulusConfig& config);
  ~TraceStimulus();

 private:

  // Phases
  void build();
  bool elab() override;
  void drc() override;

  // Stimulus: 
  StimulusContext* register_cpu(Cpu* cpu) override;

  void parse_tracefile();

  std::vector<DequeueContext*> compute_index_table();

  // Registered CPU
  std::map<Cpu*, DequeueContext*> cpumap_;
  // Trace input
  std::istream* is_ = nullptr;
  //
  std::size_t line_ = 0;
  //
  std::size_t col_ = 0;
};

// Basic programmatic stimulus source where stimulus is generated
// explicitly according to method call.
//
// advance_cursor(time_t)
//
// Advance stimulus cursor (current time) to value. Subsequent
// calls to push_stimulus are performed relative to this value.
//
// push_stimulus(ID, opcode, addr)
//
// Assign new {opcode, addr} instruction to the CPU given by ID.
//
class ProgrammaticStimulus : public Stimulus {
 public:
  ProgrammaticStimulus(kernel::Kernel* k, const StimulusConfig& config);
  ~ProgrammaticStimulus();

  
  StimulusContext* register_cpu(Cpu* cpu) override;

  //
  std::uint64_t get_cpu_id(const Cpu* cpu);

  //
  void push_stimulus(std::uint64_t cpu_id, CpuOpcode opcode, addr_t addr);

  // Advance the current stimulus cursor to the current simulation
  // time such that consequent stimulus definitions are relative to
  // the true stimulation time as seen by the kernel.
  void advance_cursor(time_t c) { cursor_ += c; }

 private:
  // Current simulation time cursor in terms of stimulus generation.
  // This potentially lags the true simuluation time if the end-user
  // decides to inject some stimulus, run the simulation and then
  // repeat the process. To maintain correctness, the end-user should
  // advance time to the current simulation time.
  cursor_t cursor_ = 0;

  // ID to Context instance map.
  std::map<std::uint64_t, DequeueContext*> context_map_;

  // CPU to ID reverse map.
  std::map<const Cpu*, std::uint64_t> id_map_;
};

//
//
Stimulus* stimulus_builder(kernel::Kernel* k, const StimulusConfig& cfg);

}  // namespace cc

#endif
