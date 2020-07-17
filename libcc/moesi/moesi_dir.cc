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

#include "moesi.h"
#include "protocol.h"
#include "sim.h"
#include "dir.h"
#include "llc.h"
#include "noc.h"
#include "mem.h"
#include <set>

namespace {

using namespace cc;

//
//
enum class State {

  // Invalid State
  //
  I,

  //
  //
  IE,

  //
  //
  S,

  //
  //
  M,

  //
  //
  E,

  //
  //
  O
};

//
//
const char* to_string(State state) {
  switch (state) {
    case State::I: return "I";
    case State::IE: return "IE";
    case State::S: return "S";
    case State::E: return "E";
    case State::M: return "M";
    case State::O: return "O";
    default: return "Invalid";
  }
}

//
//
bool is_stable(State state) {
  switch (state) {
    case State::I:
    case State::S:
    case State::E:
    case State::M:
      return true;
    default:
      return false;
  }
}

//
//
class LineState : public DirLineState {
  using sharer_set = std::set<Agent*>;
  
 public:
  LineState() = default;

  // Current line state.
  State state() const { return state_; }
  // Current owning agent.
  Agent* owner() const { return owner_; }
  // Line resides in a stable state.
  bool is_stable() const { return true; }
  // Flag denoting if 'agent' is in the sharer set.
  bool is_sharer(Agent* agent) const {
    std::set<Agent*>::iterator it = sharers_.find(agent);
    return it != sharers_.end();
  }
  // Remove sharer
  bool del_sharer(Agent* agent) {
    bool removed = false;
    std::set<Agent*>::iterator it = sharers_.find(agent);
    if (it != sharers_.end()) {
      sharers_.erase(it);
      removed = true;
    }
    return removed;
  }
  // Add agent to the set of sharers, return false if
  // already in the sharing set.
  bool add_sharer(Agent* agent) {
    const std::pair<sharer_set::iterator, bool> it =
        sharers_.insert(agent);
    return it.second;
  }
  // Set state of line.
  void set_state(State state) { state_ = state; }
  // Set owner of line.
  void set_owner(Agent* owner) { owner_ = owner; }
  
 private:
  // Coherence State.
  State state_ = State::I;
  // Owning agent.
  Agent* owner_ = nullptr;
  // Current set of sharers
  sharer_set sharers_;
};


//
//
struct UpdateStateAction : public CoherenceAction {
  UpdateStateAction(LineState* line, State state)
      : line_(line), state_(state)
  {}
  bool execute() override {
    line_->set_state(state_);
    return true;
  }
 private:
  LineState* line_ = nullptr;
  State state_;
};
  

//
//
class MOESIDirProtocol : public DirProtocol {
 public:
  MOESIDirProtocol() {}

  //
  //
  DirLineState* construct_line() const override {
    LineState* line = new LineState;
    line->set_state(State::I);
    return line;
  }

  //
  //
  std::pair<bool, DirActionList> apply(
      const DirCoherenceContext& context) const override{ 
    bool commits = true;
    DirActionList al;
    LineState* line = static_cast<LineState*>(context.line());
    switch (context.msg()->cls()) {
      case MessageClass::CohSrt: {
        const CohSrtMsg* msg = static_cast<const CohSrtMsg*>(context.msg());
        commits = apply(al, line, msg);
      } break;
      case MessageClass::CohCmd: {
        const CohCmdMsg* msg = static_cast<const CohCmdMsg*>(context.msg());
        commits = apply(al, line, msg);
      } break;
      case MessageClass::LLCCmdRsp: {
        const LLCCmdRspMsg* msg = static_cast<const LLCCmdRspMsg*>(context.msg());
        commits = apply(al, line, msg);
      } break;
      default: {
      } break;
    }
    return std::make_pair(commits, al);
  }

  //
  //
  bool apply(DirActionList& al, LineState* line, const CohSrtMsg* srt) const {
    bool commits = true;
    switch (line->state()) {
      case State::I: {
      } break;
      default: {
      } break;
    }
    return commits;
  }

  //
  //
  bool apply(DirActionList& al, LineState* line, const CohCmdMsg* cmd) const {
    bool commits = true;
    switch (line->state()) {
      case State::I: {
        switch (cmd->opcode()){
          case AceCmdOpcode::ReadShared: {
            // Line is not present in the directory
            LLCCmdMsg* msg = new LLCCmdMsg;
            msg->set_t(cmd->t());
            msg->set_opcode(LLCCmdOpcode::Fill);
            msg->set_addr(msg->addr());
            DirModel* d = dir();
            issue_emit_to_noc(al, msg, d->llc());
            // Originator becomes owner.
            issue_set_owner_action(al, line, cmd->origin());
            // Update state
            issue_update_state(al, line, State::IE);
          } break;
          default: {
            // TODO
          } break;
        }
      } break;
      default: {
      } break;
    }
    return commits;
  }

  //
  //
  bool apply(DirActionList& al, LineState* line, const LLCCmdRspMsg* cmd) const {
    bool commits = true;
    switch (line->state()) {
      case State::IE: {
        {
          // Send data
          DtMsg* msg = new DtMsg;
          msg->set_t(cmd->t());
          issue_emit_to_noc(al, msg, line->owner());
        }

        {
          // Send CohEnd
          CohEndMsg* msg = new CohEndMsg;
          msg->set_t(cmd->t());
          issue_emit_to_noc(al, msg, line->owner());
        }
        
        // Update state:
        issue_update_state(al, line, State::E);
      } break;
      default: {
      } break;
    }
    return commits;
  }

  //
  //
  void issue_update_state(DirActionList& al, LineState* line, State state) const {
    al.push_back(new UpdateStateAction(line, state));
  }

  //
  //
  void issue_set_owner_action(
      DirActionList& al, LineState* line, Agent* agent) const {
    struct SetOwnerAction : public CoherenceAction {
      SetOwnerAction(LineState* line, Agent* owner)
          : line_(line), owner_(owner)
      {}
      bool execute() override {
        line_->set_owner(owner_);
        return true;
      }
     private:
      LineState* line_ = nullptr;
      Agent* owner_ = nullptr;
    };
    al.push_back(new SetOwnerAction(line, agent));
  }

};

}

namespace cc::moesi {

DirProtocol* build_dir_protocol() { return new MOESIDirProtocol{}; }

} // namespace cc::moesi
