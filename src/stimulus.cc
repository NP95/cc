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

#include "cc/stimulus.h"

#include <algorithm>
#include <deque>
#include <fstream>

#include "cpu.h"

namespace cc {

std::string to_string(CpuOpcode opcode) {
  switch (opcode) {
    case CpuOpcode::Load:
      return "Load";
    case CpuOpcode::Store:
      return "Store";
    default:
      return "Invalid";
  }
}

StimulusException::StimulusException(const std::string& reason)
    : std::runtime_error(reason) {}


StimulusContext::StimulusContext(Stimulus* parent, kernel::Kernel* k,
                                 const std::string& name)
    : parent_(parent), Module(k, name) {}

void StimulusContext::issue() { parent_->issue(this); }

// Retire transaction
void StimulusContext::retire() { parent_->retire(this); }

Stimulus::Stimulus(kernel::Kernel* k, const StimulusConfig& config)
    : Module(k, config.name), config_(config) {}

void Stimulus::issue(StimulusContext* context) { ++issue_n_; }

void Stimulus::retire(StimulusContext* context) { ++retire_n_; }

class TraceStimulus::Context : public StimulusContext {
  friend class TraceStimulus;

 public:
  Context(Stimulus* parent, kernel::Kernel* k, const std::string& name)
      : StimulusContext(parent, k, name) {}

  // Stimulus: Flag indicate that stimulus is complete
  bool done() const override { return fs_.empty(); }

  // Stimulus: Return the next "Frontier" object to be processed,
  // returning false if all stimulus has been consumed.
  bool front(Frontier& f) const override {
    if (done()) return false;

    f = fs_.front();
    return true;
  }

  // Stimulus: Consume current head of queue, or NOP if already
  // exhausted.
  void issue() override {
    if (!done()) { fs_.pop_front(); }
    StimulusContext::issue();
  }

 private:
  // Push new stimulus to the back of the work queue.
  void push_back(const Frontier& f) { fs_.push_back(f); }

  // Pending frontiers
  std::deque<Frontier> fs_;
};

StimulusConfig TraceStimulus::from_string(const std::string& s) {
  StimulusConfig cfg;
  cfg.type = StimulusType::Trace;
  cfg.is = new std::istringstream(s);
  return cfg;
}

TraceStimulus::TraceStimulus(kernel::Kernel* k, const StimulusConfig& config)
    : Stimulus(k, config) {
  build();
}

TraceStimulus::~TraceStimulus() { delete is_; }

void TraceStimulus::build() { is_ = config().is; }

bool TraceStimulus::elab() {
  parse_tracefile();
  return false;
}

void TraceStimulus::drc() {}

void TraceStimulus::parse_tracefile() {
  using time_type = kernel::Time::time_type;

  // Scanner states
  enum class State {
    Scanning,
    InLineComment,
    InTime,
    InCmdIndex,
    InCmdOpcode,
    InCmdAddr
  };
  // Current scanner state.
  State state = State::Scanning;
  // Current location in tracefile.
  line_ = 0;
  col_ = -1;
  // Current scanner context (some accumulated string).
  std::string ctxt;
  // Current scanner time cursor.
  time_type current_time = 0;
  // Command context
  struct {
    std::size_t cpu_index;
    CpuOpcode opcode;
    addr_t addr;
  } cmd_ctxt;

  // Resolve index to context* mapping.
  const std::vector<Context*> ctxt_table = compute_index_table();
  // Start scanning file.
  while (!is_->eof()) {
    // Advance column
    col_++;

    char c = is_->get();

    // Skip whitespace
    if (c == ' ') continue;

    // Check for comment
    if (c == '/' && is_->peek() == '/') {
      is_->get();
      state = State::InLineComment;
      continue;
    }

    if (c == '\n') {
      // New line terminates current directive.
      switch (state) {
        case State::InLineComment: {
          // Line comment ends
        } break;
        case State::Scanning: {
          // Skip new lines.
        } break;
        case State::InTime: {
          // Advance scanner time cursor.
          time_type time_delta = std::stoi(ctxt);
          ctxt.clear();
          current_time += time_delta;
          // Discard ctxt.
          ctxt.clear();
        } break;
        case State::InCmdAddr: {
          std::size_t num_chars = 0;
          // Auto-detect radix.
          cmd_ctxt.addr = std::stoi(ctxt, &num_chars, 0);
          ctxt.clear();
          // Issue completed command to CPU stimulus context.
          Frontier f;
          f.time = kernel::Time{current_time, 0};
          f.cmd = Command(cmd_ctxt.opcode, cmd_ctxt.addr);
          Context* ctxt = ctxt_table[cmd_ctxt.cpu_index];
          if (ctxt != nullptr) {
            ctxt->push_back(f);
          } else {
            LogMessage msg("Invalid CPU index detected at (");
            msg.append(std::to_string(line_));
            msg.append(", ");
            msg.append(std::to_string(col_));
            msg.append(")");
            msg.level(Level::Fatal);
            log(msg);
          }
        } break;
        default: {
          // Otherwise, malformed trace file.
          LogMessage msg("Malformed trace file detected at (");
          msg.append(std::to_string(line_));
          msg.append(", ");
          msg.append(std::to_string(col_));
          msg.append(")");
          msg.level(Level::Fatal);
          log(msg);
        } break;
      }
      // Return to idle scanning state.
      state = State::Scanning;
      // Advance line count.
      col_ = -1;
      line_++;
    } else if (state != State::InLineComment) {
      // Otherwise, currently accumulating current directive.
      switch (state) {
        case State::InTime: {
          ctxt += c;
        } break;
        case State::InCmdIndex: {
          if (c == ',') {
            cmd_ctxt.cpu_index = std::stoi(ctxt);
            if (cmd_ctxt.cpu_index >= ctxt_table.size()) {
              LogMessage msg("Invalid cpu index is out of range ");
              msg.append(ctxt);
              msg.append(" at (");
              msg.append(std::to_string(line_));
              msg.append(", ");
              msg.append(std::to_string(col_));
              msg.append(")");
              msg.level(Level::Fatal);
              log(msg);
            }
            // Advance to opcode.
            ctxt.clear();
            state = State::InCmdOpcode;
          } else {
            // Sill accumulating.
            ctxt += c;
          }
        } break;
        case State::InCmdOpcode: {
          if (c == ',') {
            if (ctxt == "LD") {
              cmd_ctxt.opcode = CpuOpcode::Load;
            } else if (ctxt == "ST") {
              cmd_ctxt.opcode = CpuOpcode::Store;
            } else {
              LogMessage msg("Invalid opcode \'");
              msg.append(ctxt);
              msg.append("\' at (");
              msg.append(std::to_string(line_));
              msg.append(", ");
              msg.append(std::to_string(col_));
              msg.append(")");
              msg.level(Level::Fatal);
              log(msg);
            }
            ctxt.clear();
            state = State::InCmdAddr;
          } else {
            // Still accumulating.
            ctxt += c;
          }
        } break;
        case State::InCmdAddr: {
          // Still accumulating (delimited by newline.)
          ctxt += c;
        } break;
        default: {
          switch (c) {
            case '+': {
              state = State::InTime;
            } break;
            case 'C': {
              state = State::InCmdIndex;
              // Discard ':' prefix.
              c = is_->get();
              if (c != ':') {
                LogMessage msg("Expecting \':\' at (");
                msg.append(std::to_string(line_));
                msg.append(", ");
                msg.append(std::to_string(col_));
                msg.append(")");
                msg.level(Level::Fatal);
                log(msg);
              }
            } break;
          }
        } break;
      }
    }
  }
  if (state != State::Scanning) {
    // Expect to be back in the idle state upon EOF; if not the file
    // appears to have been truncated.
    LogMessage msg("Tracefile has been truncated at (");
    msg.append(std::to_string(line_));
    msg.append(", ");
    msg.append(std::to_string(col_));
    msg.append(")");
    msg.level(Level::Fatal);
    log(msg);
  }
}

std::vector<TraceStimulus::Context*> TraceStimulus::compute_index_table() {
  std::map<std::size_t, std::string> index_table;
  enum class ScanState {
    Idle,
    ExpectColon,
    GetIndex,
    GetPath
  };
  
  // To zeroth column
  col_ = -1;
  ScanState state = ScanState::Idle;
  
  // Parse Map header from trace file.
  std::string ctxt;
  struct {
    std::size_t index;
    std::string path;
  } item;
  bool done = false;
  while (!done && !is_->eof()) {
    const char c = is_->peek();
    ++col_;

    switch (state) {
      case ScanState::Idle: {
        if (c == 'M') {
          state = ScanState::ExpectColon;
        } else {
          done = true;
        }
      } break;
      case ScanState::ExpectColon: {
        if (c == ':') {
          state = ScanState::GetIndex;
        } else {
          // Error
        }
      } break;
      case ScanState::GetIndex: {
        if (std::isdigit(c)) {
          ctxt.push_back(c);
        } else if (c == ',') {
          item.index = std::stoi(ctxt);
          ctxt.clear();
          state = ScanState::GetPath;
        } else {
          // Error
        } 
      } break;
      case ScanState::GetPath: {
        if (c == '\n') {
          item.path = ctxt;

          auto it = index_table.find(item.index);
          if (it != index_table.end()) {
            LogMessage msg("Index is already present in mapping table: ");
            msg.append(std::to_string(item.index));
            msg.level(Level::Warning);
            log(msg);
          }

          // Insert item in table.
          index_table[item.index] = item.path;

          // Update indices
          line_++;
          col_ = -1;
          
          ctxt.clear();
          state = ScanState::Idle;
        } else {
          // Accumulate path
          ctxt.push_back(c);
        }
      } break;
      default: {
        // Error: unknown state entered.
      } break;
    }

    if (!done) { is_->get(); }
  }
  
  std::vector<Context*> t;
  for (const auto& index_cpu_it : index_table) {
    const std::string& cpu_path = index_cpu_it.second;

    // Utility class to find CPU in the instance set.
    struct CpuFinder {
      CpuFinder(const std::string& path) : path_(path) {}
      bool operator()(const std::pair<Cpu*, Context*> p) const {
        return p.first->path() == path_;
      }
     private:
      std::string path_;
    };
    // Search for CPU corresponding to path in set of registered
    // instance; if not found, bail.
    if (auto it = std::find_if(cpumap_.begin(), cpumap_.end(), CpuFinder{cpu_path});
        it != cpumap_.end()) {
      const std::size_t index = index_cpu_it.first;
      // Grow mapping table to correct length.
      if (index >= t.size()) { t.resize(index + 1); }
      // Insert entry at appropriate location.
      t[index] = it->second;
    } else {
      // CPU with path was not found.
      LogMessage msg("Cannot find cpu path: ");
      msg.append(cpu_path);
      msg.level(Level::Fatal);
      log(msg);
    }
  }
  return t;
}

StimulusContext* TraceStimulus::register_cpu(Cpu* cpu) {
  Context* ctxt = new Context(this, k(), "stimulus_context");
  cpumap_.insert(std::make_pair(cpu, ctxt));
  return ctxt;
}

class ProgrammaticStimulus::Context : public StimulusContext {
  friend class ProgrammaticStimulus;
 public:
  Context(ProgrammaticStimulus* parent, kernel::Kernel* k,
                      const std::string& name)
      : StimulusContext(parent, k, name)
  {}

  // Stimulus: Flag indicate that stimulus is complete
  bool done() const override { return fs_.empty(); }

  // Stimulus: Return the next "Frontier" object to be processed,
  // returning false if all stimulus has been consumed.
  bool front(Frontier& f) const override {
    if (done()) return false;

    f = fs_.front();
    return true;
  }

  // Stimulus: Consume current head of queue, or NOP if already
  // exhausted.
  void issue() override {
    if (!done()) { fs_.pop_front(); }
    StimulusContext::issue();
  }

 private:
  // Push new stimulus to the back of the work queue.
  void push_back(const Frontier& f) { fs_.push_back(f); }

  // Pending frontiers
  std::deque<Frontier> fs_;

  // Pointer to parent stimulus instance.
  ProgrammaticStimulus* parent_ = nullptr;
};


ProgrammaticStimulus::ProgrammaticStimulus(
    kernel::Kernel* k, const StimulusConfig& config) : Stimulus(k, config) {
}

ProgrammaticStimulus::~ProgrammaticStimulus() {
}

StimulusContext* ProgrammaticStimulus::register_cpu(Cpu* cpu) {
  Context* context = new Context(this, k(), "stimulus");
  context->set_cpu(cpu);
  const std::uint64_t id = context_map_.size();
  // Create mapping ID -> Context
  context_map_.insert(std::make_pair(id, context));
  // Create mappping CPU -> ID
  id_map_.insert(std::make_pair(cpu, id));
  return context;
}

//
std::uint64_t ProgrammaticStimulus::get_cpu_id(const Cpu* cpu) {
  std::uint64_t r = -1;
  if (auto it = id_map_.find(cpu); it != id_map_.end()) {
    r = it->second;
  }
  return r;
}

void ProgrammaticStimulus::push_stimulus(std::uint64_t cpu_id,
                                         CpuOpcode opcode, addr_t addr) {
  if (auto it = context_map_.find(cpu_id); it != context_map_.end()) {
    Frontier f;
    f.time = kernel::Time{cursor_};
    f.cmd = Command{opcode, addr};
    Context* context = it->second;
    context->push_back(f);
  } else {
    // Fail, cannot find CPU for provided ID.
    throw StimulusException("Invalid CPU ID: " + std::to_string(cpu_id));
  }
}

Stimulus* stimulus_builder(kernel::Kernel* k, const StimulusConfig& cfg) {
  Stimulus* s = nullptr;
  switch (cfg.type) {
    case StimulusType::Programmatic: {
      s = new ProgrammaticStimulus(k, cfg);
    } break;
    case StimulusType::Trace: {
      s = new TraceStimulus(k, cfg);
    } break;
    default: {
      // Unknown Stimulus type.
    } break;
  }
  return s;
}

}  // namespace cc
