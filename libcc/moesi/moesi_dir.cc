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
#include "utility.h"
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
    auto it = sharers_.insert(agent);
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
  std::set<Agent*> sharers_;
};
  

//
//
class MOESIDirProtocol : public DirProtocol {
  using cb = DirCommandBuilder;
 public:
  MOESIDirProtocol(kernel::Kernel* k) : DirProtocol(k, "moesidir") {}

  //
  //
  DirLineState* construct_line() const override {
    LineState* line = new LineState;
    return line;
  }

  //
  //
  void apply(DirContext& ctxt, DirCommandList& cl) const override {
    switch (ctxt.msg()->cls()) {
      case MessageClass::CohSrt: {
        apply(ctxt, cl, static_cast<const CohSrtMsg*>(ctxt.msg()));
      } break;
      case MessageClass::CohCmd: {
        apply(ctxt, cl, static_cast<const CohCmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::LLCCmdRsp: {
        apply(ctxt, cl, static_cast<const LLCCmdRspMsg*>(ctxt.msg()));
      } break;
      default: {
        LogMessage msg("Invalid message class received!");
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  void apply(DirContext& ctxt, DirCommandList& cl, const CohSrtMsg* msg) const {
    cl.push_back(cb::from_opcode(DirOpcode::TableInstall));
    cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
    cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
  }

  void apply(DirContext& ctxt, DirCommandList& cl, const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.line());
    switch (line->state()) {
      case State::I: {
        // Line is not present in the directory
        LLCCmdMsg* cmd = new LLCCmdMsg;
        cmd->set_t(msg->t());
        cmd->set_opcode(LLCCmdOpcode::Fill);
        cmd->set_addr(msg->addr());
        //
        issue_emit_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
        // Originator becomes owner.
        issue_set_owner_action(ctxt, cl, msg->origin());
        // Update state
        issue_update_state(ctxt, cl, State::IE);

        cl.push_back(cb::from_opcode(DirOpcode::TableLookup));
        cl.push_back(cb::from_opcode(DirOpcode::TableInstallLine));
        cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
        cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
      } break;
      default: {
      } break;
    }
  }

  //
  //
  void apply(DirContext& ctxt, DirCommandList& cl, const LLCCmdRspMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.line());
    switch (line->state()) {
      case State::IE: {
        // Send data
        DtMsg* dt = new DtMsg;
        dt->set_t(msg->t());
        issue_emit_to_noc(ctxt, cl, dt, line->owner());
        // Send CohEnd
        CohEndMsg* end = new CohEndMsg;
        end->set_t(msg->t());
        issue_emit_to_noc(ctxt, cl, end, line->owner());
        // Update state:
        issue_update_state(ctxt, cl, State::E);

        cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
        cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
      } break;
      default: {
      } break;
    }
  }

  //
  //
  void issue_update_state(
      DirContext& ctxt, DirCommandList& cl, State state) const {
    struct UpdateStateAction : public CoherenceAction {
      UpdateStateAction(LineState* line, State state)
          : line_(line), state_(state) {}
      std::string to_string() const override {
        using cc::to_string;
        
        std::stringstream ss;
        {
          KVListRenderer r(ss);
          r.add_field("action", "update state");
          r.add_field("current", to_string(line_->state()));
          r.add_field("next", to_string(state_));
        }
        return ss.str();
      }
      bool execute() override {
        line_->set_state(state_);
        return true;
      }
     private:
      LineState* line_ = nullptr;
      State state_;
    };
    LineState* line = static_cast<LineState*>(ctxt.line());
    cl.push_back(cb::from_action(new UpdateStateAction(line, state)));
  }

  //
  //
  void issue_set_owner_action(
      DirContext& ctxt, DirCommandList& cl, Agent* agent) const { 
    struct SetOwnerAction : public CoherenceAction {
      SetOwnerAction(LineState* line, Agent* owner)
          : line_(line), owner_(owner)
      {}
      std::string to_string() const override {
        using cc::to_string;
        
        std::stringstream ss;
        {
          KVListRenderer r(ss);
          r.add_field("action", "set owner");
          r.add_field("owner", owner_->path());
        }
        return ss.str();
      }
      bool execute() override {
        line_->set_owner(owner_);
        return true;
      }
     private:
      LineState* line_ = nullptr;
      Agent* owner_ = nullptr;
    };
    LineState* line = static_cast<LineState*>(ctxt.line());
    cl.push_back(cb::from_action(new SetOwnerAction(line, agent)));
  }
};

}

namespace cc::moesi {

DirProtocol* build_dir_protocol(kernel::Kernel* k) {
  return new MOESIDirProtocol(k);
}

} // namespace cc::moesi
