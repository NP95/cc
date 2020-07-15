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
#include "cpu.h"
#include <fstream>
#include <deque>

namespace cc {

std::string to_string(CpuOpcode opcode) {
  switch (opcode){
    case CpuOpcode::Load: return "Load";
    case CpuOpcode::Store: return "Store";
    default: return "Invalid";
  }
}

Stimulus::Stimulus(kernel::Kernel* k, const StimulusCfg& config)
    : Module(k, config.name), config_(config) {
}

StimulusContext::StimulusContext(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {}

class TraceStimulusContext : public StimulusContext {
  friend class TraceStimulus;
 public:
  TraceStimulusContext(kernel::Kernel* k, const std::string& name)
      : StimulusContext(k, name)
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
  void consume() override {
    if (!done()) { fs_.pop_front(); }
  }

 private:
  // Push new stimulus to the back of the work queue.
  void push_back(const Frontier& f) { fs_.push_back(f); }

  // Pending frontiers
  std::deque<Frontier> fs_;
};



TraceStimulus::TraceStimulus(kernel::Kernel* k, const StimulusCfg& config)
    : Stimulus(k, config) {
  build();
}

TraceStimulus::~TraceStimulus() {
  delete is_;
}

void TraceStimulus::build() {
  const StimulusCfg& c = config();
  std::cout << c.filename << "\n";
  is_ = new std::ifstream(c.filename);
}

void TraceStimulus::elab() {
  parse_tracefile();
}

void TraceStimulus::drc() {
}

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
  std::size_t line = 0, col = -1;
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
  const std::vector<TraceStimulusContext*> ctxt_table = compute_index_table();
  // Start scanning file.
  while (!is_->eof()) {
    // Advance column
    col++;

    char c = is_->get();

    // Skip whitespace
    if (c == ' ') continue;

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
          current_time += time_delta;
          // Discard ctxt.
          ctxt.clear();
        } break;
        case State::InCmdAddr: {
          cmd_ctxt.addr = std::stoi(ctxt);
          // Issue completed command to CPU stimulus contexgt.
          Frontier f;
          f.time = kernel::Time{current_time, 0};
          f.cmd = Command(cmd_ctxt.opcode, cmd_ctxt.addr);
          ctxt_table[cmd_ctxt.cpu_index]->push_back(f);
        } break;
        default: {
          // Otherwise, malformed trace file.
          LogMessage msg("Malformed trace file detected at (");
          msg.append(std::to_string(line));
          msg.append(", ");
          msg.append(std::to_string(col));
          msg.append(")");
          msg.level(Level::Fatal);
          log(msg);
        } break;
      }
      // Return to idle scanning state.
      state = State::Scanning;
      // Advance line count.
      line++;
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
              msg.append(std::to_string(line));
              msg.append(", ");
              msg.append(std::to_string(col));
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
            if (ctxt == "LD") { cmd_ctxt.opcode = CpuOpcode::Load; }
            else if (ctxt == "ST") { cmd_ctxt.opcode = CpuOpcode::Store; }
            else {
              LogMessage msg("Invalid opcode \'");
              msg.append(ctxt);
              msg.append("\' at (");
              msg.append(std::to_string(line));
              msg.append(", ");
              msg.append(std::to_string(col));
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
                msg.append(std::to_string(line));
                msg.append(", ");
                msg.append(std::to_string(col));
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
    msg.append(std::to_string(line));
    msg.append(", ");
    msg.append(std::to_string(col));
    msg.append(")");
    msg.level(Level::Fatal);
    log(msg);
  }
}

std::vector<TraceStimulusContext*> TraceStimulus::compute_index_table() {
  std::vector<TraceStimulusContext*> t;
  const StimulusCfg& cfg = config();
  for (const std::string& path : cfg.cpath) {
    // Utility class to find CPU in the instance set.
    struct CpuFinder {
      CpuFinder(const std::string& path) : path_(path) {}
      bool operator()(const std::pair<Cpu*, TraceStimulusContext*> p) const {
        return p.first->path() == path_;
      }
     private:
      std::string path_;
    };
    // Search for CPU corresponding to path in set of registered
    // instance; if not found, bail.
    std::map<Cpu*, TraceStimulusContext*>::iterator it =
        std::find_if(cpumap_.begin(), cpumap_.end(), CpuFinder{path});
    if (it == cpumap_.end()) {
      // CPU with path was not found.
      LogMessage msg("Cannot find cpu path: ");
      msg.append(path);
      msg.level(Level::Fatal);
      log(msg);
    }
    // Otherwise, install context in table at next index.
    t.push_back(it->second);
  }
  return t;
}

StimulusContext* TraceStimulus::register_cpu(Cpu* cpu) {
  TraceStimulusContext* ctxt =
      new TraceStimulusContext(k(), "stimulus_context");
  cpumap_.insert(std::make_pair(cpu, ctxt));
  return ctxt;
}

Stimulus* stimulus_builder(kernel::Kernel* k, const StimulusCfg& cfg) {
  return new TraceStimulus(k, cfg);
}

} // namespace cc
