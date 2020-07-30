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
  IS,

  //
  //
  S,

  //
  //
  M,

  //
  //
  E,

  // E -> S
  //
  // Transition on the demotion of a line from exclusive to shared:
  //
  ES,

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
    case State::IS: return "IS";
    case State::S: return "S";
    case State::E: return "E";
    case State::ES: return "ES";
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

enum class SubState {
  AwaitingLLCFillRsp,
  AwaitingLLCFwdRsp,
  AwaitingCohSnpRsp,
  None
};

const char* to_string(SubState state) {
  switch (state) {
    case SubState::AwaitingLLCFillRsp:
      return "AwaitingLLCFillRsp";
    case SubState::AwaitingLLCFwdRsp:
      return "AwaitingLLCFwdRsp";
    case SubState::AwaitingCohSnpRsp:
      return "AwaitingCohSnpRsp";
    case SubState::None:
      return "None";
    default:
      return "Invalid";
  }
}

//
//
class LineState : public DirLineState {
 public:
  LineState() = default;

  // Current line state.
  State state() const { return state_; }
  //
  SubState substate() const { return substate_; }
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
  //
  void set_substate(SubState substate) { substate_ = substate; }
  
 private:
  // Coherence State.
  State state_ = State::I;
  // Trandition state.
  SubState substate_;
  // Owning agent.
  Agent* owner_ = nullptr;
  // Current set of sharers
  std::set<Agent*> sharers_;
};

enum class LineUpdate {
  State,
  SubState,
  SetOwner,
  AddSharer,
  Invalid
};

const char* to_string(LineUpdate update) {
  switch (update) {
    case LineUpdate::State:
      return "State";
    case LineUpdate::SubState:
      return "SubState";
    case LineUpdate::SetOwner:
      return "SetOwner";
    case LineUpdate::AddSharer:
      return "AddSharer";
    case LineUpdate::Invalid:
    default:
      return "Invalid";
  }
}

struct LineUpdateAction : public CoherenceAction {
  LineUpdateAction(LineState* line, LineUpdate update)
      : line_(line), update_(update) {}
  void set_state(State state) { state_ = state; }
  void set_substate(SubState substate) { substate_ = substate; }
  void set_agent(Agent* agent) { agent_ = agent; }
  std::string to_string() const override {
    KVListRenderer r;
    r.add_field("update", ::to_string(update_));
    switch (update_) {
      case LineUpdate::State: {
        r.add_field("state", ::to_string(state_));
      } break;
      case LineUpdate::SubState: {
        r.add_field("state", ::to_string(substate_));
      } break;
      case LineUpdate::AddSharer: {
        r.add_field("sharer", agent_->path());
      } break;
      case LineUpdate::SetOwner: {
        r.add_field("owner", agent_->path());
      } break;
      case LineUpdate::Invalid: {
      } break;
      default: {
      } break;
    }
    return r.to_string();
  }
  bool execute() override {
    switch (update_) {
      case LineUpdate::State: {
        line_->set_state(state_);
      } break;
      case LineUpdate::SubState: {
        line_->set_substate(substate_);
      } break;
      case LineUpdate::SetOwner: {
        line_->set_owner(agent_);
      } break;
      case LineUpdate::AddSharer: {
        line_->add_sharer(agent_);
      } break;
      default: {
      } break;
    }
    return true;
  }
 private:
  //
  State state_;
  //
  SubState substate_;
  //
  Agent* agent_ = nullptr;
  //
  LineState* line_ = nullptr;
  //
  LineUpdate update_ = LineUpdate::Invalid;
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
      case MessageClass::CohSnpRsp: {
        apply(ctxt, cl, static_cast<const CohSnpRspMsg*>(ctxt.msg()));
      } break;
      default: {
        LogMessage msg("Invalid message class received!");
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  void apply(DirContext& ctxt, DirCommandList& cl, const CohSrtMsg* msg) const {
    cl.push_back(cb::from_opcode(DirOpcode::StartTransaction));
    cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
    cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
  }

  void apply(DirContext& ctxt, DirCommandList& cl, const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::I: {
        // Line is not present in the directory
        LLCCmdMsg* cmd = new LLCCmdMsg;
        cmd->set_t(msg->t());
        cmd->set_opcode(LLCCmdOpcode::Fill);
        cmd->set_addr(msg->addr());

        // Issue Fill command to LLC.
        issue_emit_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
        // Originator becomes owner.
        issue_set_owner(ctxt, cl, msg->origin());
        // Update state
        issue_update_state(ctxt, cl, State::IE);
        issue_update_substate(ctxt, cl, SubState::AwaitingLLCFillRsp);

        cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
        cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
      } break;
      case State::S: {
        switch (msg->opcode()) {
          case AceCmdOpcode::ReadShared: {
            // Add requester to sharer list; forward data to requester
            // from either LLC or nominated sharing cache
            // (complication in the presence of silent evictions).

            // Instruct LLC to forward line (presently in the S-state)
            // to the requesting agent.
            LLCCmdMsg* cmd = new LLCCmdMsg;
            cmd->set_t(msg->t());
            cmd->set_opcode(LLCCmdOpcode::PutLine);
            DirTState* tstate = ctxt.tstate();
            cmd->set_addr(tstate->addr());
            cmd->set_agent(tstate->origin());
            issue_emit_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
            // Requester now becomes a sharer.
            issue_add_sharer(ctxt, cl, msg->origin());
            issue_update_substate(ctxt, cl, SubState::AwaitingLLCFwdRsp);
            cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
            cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
          } break;
          case AceCmdOpcode::ReadUnique: {
            // Issue invalidation commands to sharing caches, if unique
            // promote line to exclusive state.
          } break;
          default: {
          } break;
        }
      } break;
      case State::E: {
        switch (msg->opcode()) {
          case AceCmdOpcode::ReadShared: {
            // Line resides in the exclusive state, meaning that a
            // single agent has it in either the E or M state. We
            // therefore wish for the agent to relinquish control of
            // the line by either updating LLC and demoting the line
            // to S, or in the ReadShared case, transfering ownership
            // to the requesting agent.
            //
            CohSnpMsg* snp = new CohSnpMsg;
            snp->set_t(msg->t());
            snp->set_addr(msg->addr());
            snp->set_origin(ctxt.dir());
            snp->set_agent(msg->origin());
            // ReadShared; owning agent can relinquish the line or
            // retain it in the shared state.
            snp->set_opcode(AceSnpOpcode::ReadShared);
            issue_emit_to_noc(ctxt, cl, snp, line->owner());

            // Transition to exclusive state, owning agent can transition
            // to I or S state after completion of transaction.
            issue_update_state(ctxt, cl, State::ES);
            issue_update_substate(ctxt, cl, SubState::AwaitingCohSnpRsp);
            cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
            cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
          } break;
          case AceCmdOpcode::ReadUnique: {
          } break;
          default: {
          } break;
        }
      } break;
      default: {
        LogMessage msg("Unexpected state transition.", Level::Fatal);
        log(msg);
      } break;
    }

    // Issue command message response (returns credit).
    CohCmdRspMsg* rsp = new CohCmdRspMsg;
    rsp->set_t(msg->t());
    issue_emit_to_noc(ctxt, cl, rsp, msg->origin());
  }

  //
  //
  void apply(DirContext& ctxt, DirCommandList& cl, const LLCCmdRspMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::IS:
      case State::IE: {
        switch (line->substate()) {
          case SubState::AwaitingLLCFillRsp: {
            // LLC line fill has completed. Now, instruct the LLC to
            // forward the desired line to the requesting (owning)
            // agent.
            LLCCmdMsg* cmd = new LLCCmdMsg;
            cmd->set_t(msg->t());
            cmd->set_opcode(LLCCmdOpcode::PutLine);
            DirTState* tstate = ctxt.tstate();
            cmd->set_addr(tstate->addr());
            cmd->set_agent(tstate->origin());
            issue_emit_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
            cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));

            issue_update_substate(ctxt, cl, SubState::AwaitingLLCFwdRsp);
            cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
          } break;
          case SubState::AwaitingLLCFwdRsp: {
            // Requesting cache now has the line, issue notification
            // of the line status and terminate the transction.
            // Send CohEnd
            CohEndMsg* end = new CohEndMsg;
            end->set_t(msg->t());
            end->set_origin(ctxt.dir());
            issue_emit_to_noc(ctxt, cl, end, line->owner());
            const State next_state =
                (line->state() == State::IS) ? State::S : State::E;
            issue_update_state(ctxt, cl, next_state);
            // Return to stable/non-transient state.
            issue_update_substate(ctxt, cl, SubState::None);

            cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
            cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
            cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
          } break;
          default: {
          }
        }
      } break;
      case State::S: {
        switch (line->substate()) {
          case SubState::AwaitingLLCFwdRsp: {
            CohEndMsg* end = new CohEndMsg;
            end->set_t(msg->t());
            end->set_origin(ctxt.dir());
            issue_emit_to_noc(ctxt, cl, end, ctxt.tstate()->origin());
            issue_update_state(ctxt, cl, State::S);
            // Return to stable/non-transient state.
            issue_update_substate(ctxt, cl, SubState::None);

            cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
            cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
            cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
          } break;
          default: {
          } break;
        }
      } break;
      default: {
      } break;
    }
  }

  //
  //
  void apply(DirContext& ctxt, DirCommandList& cl, const CohSnpRspMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::ES: {

        CohEndMsg* end = new CohEndMsg;
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        issue_emit_to_noc(ctxt, cl, end, ctxt.tstate()->origin());
        
        cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
        cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));
        cl.push_back(cb::from_opcode(DirOpcode::WaitOnMsgOrNextEpoch));
      } break;
      default: {
      } break;
    }
  }

  //
  //
  void issue_update_state(DirContext& ctxt, DirCommandList& cl, State state) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::State);
    update->set_state(state);
    cl.push_back(cb::from_action(update)); 
  }

  //
  //
  void issue_update_substate(DirContext& ctxt, DirCommandList& cl, SubState state) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::SubState);
    update->set_substate(state);
    cl.push_back(cb::from_action(update)); 
  }

  //
  //
  void issue_set_owner(DirContext& ctxt, DirCommandList& cl, Agent* owner) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::SetOwner);
    update->set_agent(owner);
    cl.push_back(cb::from_action(update)); 
  }

  void issue_add_sharer(DirContext& ctxt, DirCommandList& cl, Agent* owner) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::AddSharer);
    update->set_agent(owner);
    cl.push_back(cb::from_action(update)); 
  }
};

}

namespace cc::moesi {

DirProtocol* build_dir_protocol(kernel::Kernel* k) {
  return new MOESIDirProtocol(k);
}

} // namespace cc::moesi
