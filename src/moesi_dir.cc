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

#include <set>

#include "dir.h"
#include "llc.h"
#include "mem.h"
#include "moesi.h"
#include "noc.h"
#include "protocol.h"
#include "sim.h"
#include "utility.h"

namespace {

using namespace cc;

//
//
enum class State {
  // Bad state (placeholder)
  X,

  // Invalid
  I,

  // Invalid -> Exclusive
  I_E,

  // Invalid -> Shared
  I_S,

  // Shared (stable)
  S,

  // Shared -> Invalid
  S_I,

  // Shared -> Exclusive
  S_E,

  // Shared -> Shared or Exclusive
  S_SE,

  // Shared -> Modified or Exclusive
  S_ME, 

  // Modified (stable)
  M,

  // Modified -> Invalid
  M_I,

  // Modified -> Owned
  M_O,

  // Modified -> Exclusive
  M_EO,

  // Modified -> Shared or Exclusive
  M_SE,

  // Modified -> Modified or Exclusive
  M_ME,

  // Exclusive (stable)
  E,

  // Exclusive -> Owned
  E_O,

  // Exclusive -> Exclusive
  E_E,

  // Exclusive -> Invalid
  E_I,

  // Exclusive -> Shared or Exclusive
  E_SE,

  // Owned (stable)
  O, 

  // Owned -> Owned
  O_O,

  // Owned -> Modified or Exclusive
  O_ME,

  // Owned -> Exclusive
  O_E,

  // Owned -> Invalid
  O_I,

  // Owned -> Shared or Exclusive
  O_SE
};

//
//
const char* to_string(State state) {
  switch (state) {
    case State::X:
      return "X";
    case State::I:
      return "I";
    case State::I_E:
      return "I_E";
    case State::I_S:
      return "I_S";
    case State::S:
      return "S";
    case State::E:
      return "E";
    case State::E_O:
      return "E_O";
    case State::E_E:
      return "E_E";
    case State::M:
      return "M";
    case State::O:
      return "O";
    case State::O_O:
      return "O_O";
    case State::O_E:
      return "O_E";
    default:
      return "Invalid";
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

  // Factory functions to construct Command objects to permute the
  // current line instance.
  
  // Build Update state command:
  DirCommand* build_update_state(State state);

  // Build Set owner command:
  DirCommand* build_set_owner(Agent* agent);

  // Build Delete owner command:
  DirCommand* build_del_owner();

  // Build add sharer command:
  DirCommand* build_add_sharer(Agent* agent);

  // Build delete sharer command:
  DirCommand* build_del_sharer(Agent* agent);
  

  // Convert line to human-readable format.
  std::string to_string() const {
    using cc::to_string;
    KVListRenderer r;
    r.add_field("state", to_string(state()));
    r.add_field("owner", owner() == nullptr ? "<NONE>" : owner()->path());
    ArrayRenderer a;
    for (const Agent* agent : sharers()) {
      a.add_item(agent->path());
    }
    r.add_field("sharers", a.to_string());
    return r.to_string();
  }

  // Current line state.
  State state() const { return state_; }
  // Current sharer set (full, non-sparse set).
  const std::set<Agent*>& sharers() const { return sharers_; }
  // Current owning agent.
  Agent* owner() const { return owner_; }
  // Line resides in a stable state.
  bool is_stable() const { return true; }
  // Flag denoting if 'agent' is in the sharer set.
  bool is_sharer(Agent* agent) const {
    return sharers_.find(agent) != sharers_.end();
  }

  // Delete sharer, from sharer set
  bool del_sharer(Agent* agent) {
    bool removed = false;
    if (auto it = sharers_.find(agent); it != sharers_.end()) {
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

enum class LineUpdateOpcode {
  // Set Line State
  SetState,

  // Set Agent as Owner
  SetOwner,

  // Delete current Owner
  DelOwner,

  // Add Agent to Sharer set
  AddSharer,

  // Del Agent from Sharer set
  DelSharer,

  // Invalid; placeholder.
  Invalid
};

const char* to_string(LineUpdateOpcode update) {
  switch (update) {
    case LineUpdateOpcode::SetState:
      return "SetState";
    case LineUpdateOpcode::SetOwner:
      return "SetOwner";
    case LineUpdateOpcode::AddSharer:
      return "AddSharer";
    case LineUpdateOpcode::DelSharer:
      return "DelSharer";
    case LineUpdateOpcode::Invalid:
    default:
      return "Invalid";
  }
}

//
//
struct LineUpdateAction : public DirCoherenceAction {
  LineUpdateAction(LineState* line, LineUpdateOpcode update)
      : line_(line), update_(update) {}

  void set_state(State state) { state_ = state; }
  void set_agent(Agent* agent) { agent_ = agent; }

  std::string to_string() const override {
    KVListRenderer r;
    r.add_field("update", ::to_string(update_));
    switch (update_) {
      case LineUpdateOpcode::SetState: {
        r.add_field("state", ::to_string(state_));
      } break;
      case LineUpdateOpcode::SetOwner: {
        r.add_field("owner", agent_->path());
      } break;
      case LineUpdateOpcode::AddSharer: {
        r.add_field("sharer", agent_->path());
      } break;
      case LineUpdateOpcode::DelSharer: {
        r.add_field("sharer", agent_->path());
      } break;
      case LineUpdateOpcode::Invalid: {
      } break;
      default: {
      } break;
    }
    return r.to_string();
  }

  bool execute() override {
    switch (update_) {
      case LineUpdateOpcode::SetState: {
        line_->set_state(state_);
      } break;
      case LineUpdateOpcode::SetOwner: {
        line_->set_owner(agent_);
      } break;
      case LineUpdateOpcode::DelOwner: {
        line_->set_owner(nullptr);
      } break;
      case LineUpdateOpcode::AddSharer: {
        line_->add_sharer(agent_);
      } break;
      case LineUpdateOpcode::DelSharer: {
        line_->del_sharer(agent_);
      } break;
      default: {
      } break;
    }
    return true;
  }

 private:
  // State to which the line transitions
  State state_ = State::X;
  // New Sharing/Owning agent
  Agent* agent_ = nullptr;
  // Directory line of interest
  LineState* line_ = nullptr;
  // Update opcode
  LineUpdateOpcode update_ = LineUpdateOpcode::Invalid;
};

DirCommand* LineState::build_update_state(State state) {
  LineUpdateAction* action =
      new LineUpdateAction(this, LineUpdateOpcode::SetState);
  action->set_state(state);
  return DirCommandBuilder::from_action(action);
}

// Build Set owner command:
DirCommand* LineState::build_set_owner(Agent* agent) {
  LineUpdateAction* action =
      new LineUpdateAction(this, LineUpdateOpcode::SetOwner);
  action->set_agent(agent);
  return DirCommandBuilder::from_action(action);
}

// Build Delete owner command:
DirCommand* LineState::build_del_owner() {
  LineUpdateAction* action =
      new LineUpdateAction(this, LineUpdateOpcode::DelOwner);
  return DirCommandBuilder::from_action(action);
}

// Build add sharer command:
DirCommand* LineState::build_add_sharer(Agent* agent) {
  LineUpdateAction* action =
      new LineUpdateAction(this, LineUpdateOpcode::AddSharer);
  action->set_agent(agent);
  return DirCommandBuilder::from_action(action);
}

// Build delete sharer command:
DirCommand* LineState::build_del_sharer(Agent* agent) {
  LineUpdateAction* action =
      new LineUpdateAction(this, LineUpdateOpcode::DelSharer);
  action->set_agent(agent);
  return DirCommandBuilder::from_action(action);
}

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
    cl.push_back(DirOpcode::StartTransaction);
    // Consume and advance
    cl.next_and_do_consume(true);
  }

  void apply(DirContext& ctxt, DirCommandList& cl, const CohCmdMsg* msg) const {
    // Update Transaction State with appropriate state contained
    // within the Command Message (also present in the context).
    DirTState* tstate = ctxt.tstate();
    tstate->set_opcode(msg->opcode());
    tstate->set_addr(msg->addr());
    tstate->set_origin(msg->origin());
    ctxt.set_tstate(tstate);

    // Issue command message response (returns credit).
    CohCmdRspMsg* rsp = Pool<CohCmdRspMsg>::construct();
    rsp->set_t(msg->t());
    rsp->set_origin(ctxt.dir());
    issue_msg_to_noc(ctxt, cl, rsp, msg->origin());

    //
    const AceCmdOpcode opcode = tstate->opcode();
    switch (opcode) {
      case AceCmdOpcode::ReadNoSnoop: {
        handle_cmd_read_no_snoop(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::ReadOnce: {
        handle_cmd_read_once(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::ReadClean: {
        handle_cmd_read_clean(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::ReadNotSharedDirty: {
        handle_cmd_read_not_shared_dirty(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::ReadShared: {
        handle_cmd_read_shared(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::ReadUnique: {
        handle_cmd_read_unique(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::CleanUnique: {
        handle_cmd_clean_unique(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::CleanShared: {
        handle_cmd_clean_shared(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::CleanInvalid: {
        handle_cmd_clean_invalid(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::MakeUnique: {
        handle_cmd_make_unique(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::MakeInvalid: {
        handle_cmd_make_invalid(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::WriteNoSnoop: {
        handle_cmd_write_no_snoop(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::WriteUnique: {
        handle_cmd_write_unique(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::WriteLineUnique: {
        handle_cmd_write_line_unique(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::WriteBack: {
        handle_cmd_write_back(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::WriteClean: {
        handle_cmd_write_clean(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::Evict: {
        handle_cmd_evict(ctxt, cl, msg);
      } break;
      default: {
        // Unknown opcode; raise error.
        std::string reason = "Unsupported opcode received: ";
        reason += to_string(opcode);
        cl.raise_error(reason);
      } break;
    }
  }

  // C4.5.1 ReadNoSnoop
  void handle_cmd_read_no_snoop(DirContext& ctxt, DirCommandList& cl,
                                const CohCmdMsg* msg) const {
    // The directory should never see non-cached commands. Instead,
    // the requesting agent should issue to the command directly into
    // the interconnect where it is directed to either a memory
    // controller or a peripheral/accelerator.
    cl.raise_error("Directory has received a non-cached command: "
                   "ReadNoSnoop");
  }

  // C4.5.2 ReadOnce
  //
  // Requesting agent receives a "snapshot" of the cache line; the
  // cache line is fetched from memory, however nominally it is not
  // retained by the agent for reuse (received in Shared state),
  // however it may be retained by the agent, if already present in
  // the cache.
  //
  void handle_cmd_read_once(DirContext& ctxt, DirCommandList& cl,
                            const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::I:
      case State::S:
      case State::E:
      case State::M:
      case State::O:
      default: {
        cl.raise_error("Not implemented");
      } break;
    }
  }

  // C4.5.3 ReadClean
  //
  //
  //
  void handle_cmd_read_clean(DirContext& ctxt, DirCommandList& cl,
                             const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::I:
      case State::S:
      case State::E:
      case State::M:
      case State::O:
      default: {
        cl.raise_error("Not implemented");
      } break;
    }
  }

  // C4.5.4 ReadNotSharedDirty
  //
  //
  //
  void handle_cmd_read_not_shared_dirty(DirContext& ctxt,
                                        DirCommandList& cl,
                                        const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::I:
      case State::S:
      case State::E:
      case State::M:
      case State::O:
      default: {
        cl.raise_error("Not implemented");
      } break;
    }
  }

  // C4.5.5 ReadShared
  //
  //
  void handle_cmd_read_shared(DirContext& ctxt, DirCommandList& cl,
                              const CohCmdMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    const State state = line->state();
    switch (state) {
      case State::I: {
        // Line is not present in the directory
        LLCCmdMsg* cmd = Pool<LLCCmdMsg>::construct();
        cmd->set_t(msg->t());
        cmd->set_opcode(LLCCmdOpcode::Fill);
        cmd->set_addr(msg->addr());

        // Issue Fill command to LLC.
        issue_msg_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
        // Originator becomes owner.
        cl.push_back(line->build_set_owner(msg->origin()));
        //issue_set_owner(ctxt, cl, msg->origin());
        // Update state
        cl.push_back(line->build_update_state(State::I_E));

        // Set flag awaiting LLC response
        cl.push_back(tstate->build_set_llc_cmd_opcode(cmd->opcode()));
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::S: {
        // Add requester to sharer list; forward data to requester
        // from either LLC or nominated sharing cache
        // (complication in the presence of silent evictions).

        // Instruct LLC to forward line (presently in the S-state)
        // to the requesting agent.
        LLCCmdMsg* cmd = Pool<LLCCmdMsg>::construct();
        cmd->set_t(msg->t());
        cmd->set_opcode(LLCCmdOpcode::PutLine);
        cmd->set_addr(tstate->addr());
        cmd->set_agent(tstate->origin());
        issue_msg_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
        // Requester now becomes a sharer.
        cl.push_back(line->build_add_sharer(msg->origin()));
        //issue_add_sharer(ctxt, cl, msg->origin());
        // Set flag awaiting LLC response
        cl.push_back(tstate->build_set_llc_cmd_opcode(cmd->opcode()));
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::M:
      case State::O:
      case State::E: {
        // Line resides in the exclusive state, meaning that a
        // single agent has it in either the E or M state. We
        // therefore wish for the agent to relinquish control of
        // the line by either updating LLC and demoting the line
        // to S, or in the ReadShared case, transfering ownership
        // to the requesting agent.
        //
        CohSnpMsg* snp = Pool<CohSnpMsg>::construct();
        snp->set_t(msg->t());
        snp->set_addr(msg->addr());
        snp->set_origin(ctxt.dir());
        snp->set_agent(msg->origin());
        // ReadShared; owning agent can relinquish the line or
        // retain it in the shared state.
        snp->set_opcode(to_snp_opcode(msg->opcode()));
        issue_msg_to_noc(ctxt, cl, snp, line->owner());

        // Issue one snoop; therefore await one snoop response
        cl.push_back(tstate->build_set_snoop_n(1));
        //issue_set_snoop_n(ctxt, cl, 1);

        // Directory has no notion of local modified status,
        // therefore when a line has been installed in the
        // Exclusive state, the directory asssumes that the cache
        // line has been modified by the owning agent.
        State next_state = State::X;
        switch (state) {
          case State::M: { next_state = State::M_O; } break;
          case State::O: { next_state = State::O_O; } break;
          case State::E: { next_state = State::E_O; } break;
          default:         next_state = State::X;
        }
        cl.push_back(line->build_update_state(next_state));
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        cl.raise_error("Not implemented");
      } break;
    }
  }

  // C4.5.6 ReadUnique
  //
  void handle_cmd_read_unique(DirContext& ctxt, DirCommandList& cl,
                              const CohCmdMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::I: {
        // Line is not present in the directory
        LLCCmdMsg* cmd = Pool<LLCCmdMsg>::construct();
        cmd->set_t(msg->t());
        cmd->set_opcode(LLCCmdOpcode::Fill);
        cmd->set_addr(msg->addr());

        // Issue Fill command to LLC.
        issue_msg_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
        // Originator becomes owner.
        cl.push_back(line->build_set_owner(msg->origin()));
        // Set flag awaiting LLC response.
        cl.push_back(tstate->build_set_llc_cmd_opcode(cmd->opcode()));
        // Update state
        cl.push_back(line->build_update_state(State::I_E));
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::M:
      case State::E: {
        // Owning cache may have line in either the E or the M
        // state; would nominally expected for the line to have
        // been promoted to the M state by the time some other
        // cache requests the line, but this is necessarily the
        // case.
        CohSnpMsg* snp = Pool<CohSnpMsg>::construct();
        snp->set_t(msg->t());
        snp->set_addr(msg->addr());
        snp->set_origin(ctxt.dir());
        snp->set_agent(msg->origin());
        snp->set_opcode(AceSnpOpcode::ReadUnique);
        issue_msg_to_noc(ctxt, cl, snp, line->owner());

        // Transition to Exclusive state from current Shared/Exclusive
        // state.
        State next_state = State::X;
        switch (state) {
          case State::M: { next_state = State::M_ME; } break;
          case State::E: { next_state = State::E_E; } break;
          default:         next_state = State::X;
        }
        // Update state
        cl.push_back(line->build_update_state(next_state));
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::S:
      case State::O: {
        // Requesting cache does not have line installed and
        // requests line in a Unique state (presumably to carry
        // out a Store command). Some other cache has line in
        // Ownership state, and there may be some number of other
        // caches with the line in a Shared state.
        //
        // Options:
        //
        // 1. Owning cache transfers ownership to requesting cache;
        //    all caches except for requester is invalidated.
        //
        // 2. Owning cache writes line back to memory and signals
        //    that it no longer has the line installed. Line
        //    must then be sourced from memory.
        //
        CohSnpMsg* snp = nullptr;
        std::size_t snoop_n = 0;

        // Issue snoop to owner. Owner should forwarded line to
        // requesting agent and relinquish ownership of the line.
        if (Agent* owner = line->owner(); owner != nullptr) {
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(msg->origin());
          snp->set_opcode(to_snp_opcode(ctxt.tstate()->opcode()));
          issue_msg_to_noc(ctxt, cl, snp, owner);

          ++snoop_n;
        }

        for (Agent* sharer : line->sharers()) {
          // Do not snoop to self.
          if (sharer == ctxt.tstate()->origin()) continue;
          
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(msg->origin());
          snp->set_opcode(to_snp_opcode(ctxt.tstate()->opcode()));
          issue_msg_to_noc(ctxt, cl, snp, sharer);

          ++snoop_n;
        }

        // Set expected snoop response count in transaction state
        // object.
        cl.push_back(tstate->build_set_snoop_n(snoop_n));
        //issue_set_snoop_n(ctxt, cl, snoop_n);
        // Update state:
        //
        // Owner -> Modified or Exclusive.
        //
        // Modified state is entered if upon concensus of the
        // coherence operation, owner ship has been passed by the
        // owning agent to the requester. The Exclusive state is
        // entered if ownership is not passed.
        const State next_state =
            (state == State::S) ? State::S_ME : State::O_ME;
        cl.push_back(line->build_update_state(next_state));
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  // C4.6.1 CleanUnique
  //
  // Requesting cache has line installed in its cache, but not in a
  // state where a write can be performed (Shared, SC or Owned, SD).
  // The CleanUnique command is issued such that all other copies of
  // lines in the system are invalidated and that line in the
  // originator is promoted to the an Exclusive state (Exclusive, UC or
  // Modified, UD).
  //
  void handle_cmd_clean_unique(DirContext& ctxt, DirCommandList& cl,
                               const CohCmdMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (line->state()) {
      case State::I: {
        // CleanUnique can be issued by agent in Invalid state
        // although this is not typical. In this case, the transaction
        // ends immediately as no snoops are required.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        // "the {IsShared/PassDirty} response must be deasserted"
        end->set_is(false);
        end->set_pd(false);
        // Issue response back to originator.
        issue_msg_to_noc(ctxt, cl, end, ctxt.tstate()->origin());
        // Transaction is now complete.
        cl.push_back(DirOpcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::S:
      case State::E:
      case State::M:
      case State::O: {
        // Issue invalidation snoops to sharing agents
        std::size_t snoop_n = 0;
        CohSnpMsg* snp = nullptr;

        // Issue invalidation snoop (CleanInvalid) to owning agent (if
        // applicable).
        if (Agent* owner = line->owner(); owner != nullptr) {
          // If command originator is currently the owner, skip the
          // invalidation snoop.
          if (owner == msg->origin()) break;
          
          // Issue snoop.
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(msg->origin());
          snp->set_opcode(to_snp_opcode(msg->opcode()));
          // Issue snoop to interconnect
          issue_msg_to_noc(ctxt, cl, snp, owner);
          ++snoop_n;
        }

        // Issue invalidation snoops (CleanInvalid) to sharing agents.
        for (Agent* agent : line->sharers()) {
          // Do not sent message to originator as agent must retain
          // line, as per. requirements.
          if (agent == msg->origin()) continue;

          // Issue snoop.
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(msg->origin());
          snp->set_opcode(to_snp_opcode(msg->opcode()));
          issue_msg_to_noc(ctxt, cl, snp, agent);
          ++snoop_n;
        }

        // Set expected snoop response count in transaction state
        // object.
        cl.push_back(tstate->build_set_snoop_n(snoop_n));
        // issue_set_snoop_n(ctxt, cl, snoop_n);

        State next_state = State::X;
        switch (state) {
          // Transition from clean state to exclusive
          case State::S: { next_state = State::S_E; } break;
          case State::E: { next_state = State::E_E; } break;

          // Transition to Exclusive of Owned state depending upon
          // whether the owning agent performs a writeback to memory
          // before forwarding the line. Owning agent can potentially
          // opt. to transfer ownership a the dirty line to the
          // requesting agent.
          case State::M: { next_state = State::M_EO; } break;
          case State::O: { next_state = State::M_EO; } break;
          default:         next_state = State::X;
        }
        cl.push_back(line->build_update_state(next_state));

        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  // C4.6.2 CleanShared
  //
  // Command instructs all agents holding the line to clean the
  // line. Agents may choose to retain the line is a clean state
  // (Shared, SC or Exclusive, UC) or may choose to invalidate (after
  // a prior writeback operation when applicable). As the transaction
  // does not snoop the requesting agent, the requestor must the line
  // in a clean state before issuing the CleanShared command.
  //
  void handle_cmd_clean_shared(DirContext& ctxt, DirCommandList& cl,
                               const CohCmdMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (line->state()) {
      case State::I: {
        // CleanUnique can be issued by agent in Invalid state
        // although this is not typical. In this case, the transaction
        // ends immediately as no snoops are required.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        // "the {IsShared/PassDirty} response must be deasserted"
        end->set_is(false);
        end->set_pd(false);
        // Issue response back to originator.
        issue_msg_to_noc(ctxt, cl, end, ctxt.tstate()->origin());
        // Transaction is now complete.
        cl.push_back(DirOpcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::S:
      case State::E:
      case State::M:
      case State::O: {
        // Issue invalidation snoops to sharing agents
        std::size_t snoop_n = 0;
        CohSnpMsg* snp = nullptr;

        // Issue invalidation snoop (CleanInvalid) to owning agent (if
        // applicable).
        if (Agent* owner = line->owner(); owner != nullptr) {
          // If command originator is currently the owner, skip the
          // invalidation snoop.
          if (owner == msg->origin()) break;
          
          // Issue snoop.
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(msg->origin());
          snp->set_opcode(to_snp_opcode(msg->opcode()));
          // Issue snoop to interconnect
          issue_msg_to_noc(ctxt, cl, snp, owner);
          ++snoop_n;
        }

        // Issue invalidation snoops (CleanInvalid) to sharing agents.
        for (Agent* agent : line->sharers()) {
          // Do not sent message to originator as agent must retain
          // line, as per. requirements.
          if (agent == msg->origin()) continue;

          // Issue snoop.
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(msg->origin());
          snp->set_opcode(to_snp_opcode(msg->opcode()));
          issue_msg_to_noc(ctxt, cl, snp, agent);
          ++snoop_n;
        }

        // Set expected snoop response count in transaction state
        // object.
        cl.push_back(tstate->build_set_snoop_n(snoop_n));
        // issue_set_snoop_n(ctxt, cl, snoop_n);

        State next_state = State::X;
        switch (state) {
          // Transition from clean state to Shared or Exclusive (where
          // requester is the cache in the Exclusive state).
          case State::S: { next_state = State::S_SE; } break;
          case State::E: { next_state = State::E_SE; } break;

          // Transition from Dirty state to Shared to Exclusve state
          // depending upon whether the Owning agent decides to
          // retain the line after its writeback).
          case State::M: { next_state = State::M_SE; } break;
          case State::O: { next_state = State::O_SE; } break;
          default:         next_state = State::X;
        }
        cl.push_back(line->build_update_state(next_state));

        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  // C4.6.3 CleanInvalid
  //
  // For all agents holding the line, invalidate the line such that
  // upon completion of the command no cache is holding the line and
  // main memory is holding a valid and upto date copy.
  //
  void handle_cmd_clean_invalid(DirContext& ctxt, DirCommandList& cl,
                                const CohCmdMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::I: {
        // No lines have the cache, command is effectively a nop
        // and therefore completes immediately.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        // "the {IsShared/PassDirty} response must be deasserted"
        end->set_is(false);
        end->set_pd(false);
        // Issue response back to originator.
        issue_msg_to_noc(ctxt, cl, end, ctxt.tstate()->origin());
        // Transaction is now complete.
        cl.push_back(DirOpcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::S:
      case State::E:
      case State::M:
      case State::O: {
        // Otherwise, issue invalidation snoops (MakeInvalid) to the
        // Owner (if present), and sharers (if present).
        std::size_t snoop_n = 0;
        CohSnpMsg* snp = nullptr;
        if (Agent* owner = line->owner(); owner != nullptr) {
          // Issue invalidation to owner
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(ctxt.tstate()->origin());
          snp->set_opcode(to_snp_opcode(msg->opcode()));
          // Issue to interconnect
          issue_msg_to_noc(ctxt, cl, snp, owner);
          ++snoop_n;
        }
        for (Agent* sharer : line->sharers()) {
          // Issue invalidation to sharer
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(ctxt.tstate()->origin());
          snp->set_opcode(to_snp_opcode(msg->opcode()));
          // Issue to interconnect
          issue_msg_to_noc(ctxt, cl, snp, sharer);
          ++snoop_n;
        }
        // Set expected number of snoop responses.
        cl.push_back(tstate->build_set_snoop_n(snoop_n));
        // Compute next state (transition to invalid)
        State next_state = State::X;
        switch (state) {
          case State::S: { next_state = State::S_I; } break;
          case State::E: { next_state = State::E_I; } break;
          case State::M: { next_state = State::M_I; } break;
          case State::O: { next_state = State::O_I; } break;
          default:         next_state = State::X;
        }
        cl.push_back(line->build_update_state(next_state));
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  // C4.7.1 Make Unique
  //
  // Invalidate all copies of the line in all agents aside from the
  // command initiator. From the perspective of the directory, this
  // command behaves as the MakeInvalid command. The major difference
  // between the two commands is at the L2, where the line remains
  // installed in the originator, which then performs a full-cache
  // line write. We therefore piggy-back the MakeInvalid
  // implementation.
  //
  void handle_cmd_make_unique(DirContext& ctxt, DirCommandList& cl,
                              const CohCmdMsg* msg) const {
    handle_cmd_make_invalid(ctxt, cl, msg);
  }

  // C4.7.2 Make Invalid
  //
  // Invalidate all agents currently holding the line regardless of
  // state (owning/sharing). Command assumes that the initiating
  // agent had previously invalidate the line held in its cache,
  // therefore a snoop is not issued to the originator.
  // 
  void handle_cmd_make_invalid(DirContext& ctxt, DirCommandList& cl,
                               const CohCmdMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    const State state = line->state();
    switch (state) {
      case State::I: {
        // No lines have the cache, command is effectively a nop
        // and therefore completes immediately.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        // "the {IsShared/PassDirty} response must be deasserted"
        end->set_is(false);
        end->set_pd(false);
        // Issue response back to originator.
        issue_msg_to_noc(ctxt, cl, end, ctxt.tstate()->origin());
        // Transaction is now complete.
        cl.push_back(DirOpcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::S:
      case State::E:
      case State::M:
      case State::O: {
        // Otherwise, issue invalidation snoops (MakeInvalid) to the
        // Owner (if present), and sharers (if present).
        std::size_t snoop_n = 0;
        CohSnpMsg* snp = nullptr;
        if (Agent* owner = line->owner(); owner != nullptr) {
          // Issue invalidation to owner
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(ctxt.tstate()->origin());
          snp->set_opcode(to_snp_opcode(msg->opcode()));
          // Issue to interconnect
          issue_msg_to_noc(ctxt, cl, snp, owner);
          ++snoop_n;
        }
        for (Agent* sharer : line->sharers()) {
          // Issue invalidation to sharer
          snp = Pool<CohSnpMsg>::construct();
          snp->set_t(msg->t());
          snp->set_addr(msg->addr());
          snp->set_origin(ctxt.dir());
          snp->set_agent(ctxt.tstate()->origin());
          snp->set_opcode(to_snp_opcode(msg->opcode()));
          // Issue to interconnect
          issue_msg_to_noc(ctxt, cl, snp, sharer);
          ++snoop_n;
        }
        // Set expected number of snoop responses.
        cl.push_back(tstate->build_set_snoop_n(snoop_n));
        // Compute next state (transition to invalid)
        State next_state = State::X;
        switch (state) {
          case State::S: { next_state = State::S_I; } break;
          case State::E: { next_state = State::E_I; } break;
          case State::M: { next_state = State::M_I; } break;
          case State::O: { next_state = State::O_I; } break;
          default:         next_state = State::X;
        }
        cl.push_back(line->build_update_state(next_state));
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        cl.raise_error("Invalid state");
      } break;
    }
  }

  // C4.8.1
  //
  // Agent issues a write to a non-cacheable region of memory. As the
  // line is non-cachable, this command should never reach the
  // directory and should be handled externally by the system
  // interconnect.
  //
  void handle_cmd_write_no_snoop(DirContext& ctxt, DirCommandList& cl,
                                 const CohCmdMsg* msg) const {
    // The directory should never see non-cached commands. Instead,
    // the requesting agent should issue to the command directly into
    // the interconnect where it is directed to either a memory
    // controller or a peripheral/accelerator.
    cl.raise_error("Directory has received a non-cached command: "
                   "ReadNoSnoop");
  }

  // C4.8.2
  //
  // Write line from the clean S or E states. Typically used by
  // non-cachable agents who are writing to a cacheable region of
  // memory.
  //
  // Salient point:
  // 
  // "In the case of master holding a line in a Clean state while
  // performing a WriteUnique transaction, the cache line must be
  // updated to the new value when the WriteUnique transaction
  // response is received."
  //
  // This point requires that the WriteUnique be held in a speculative
  // state until the response has been received. The command can only
  // write from a Clean state, which means that the line itself is the
  // most recent value in the system, the Write operation therefore
  // only removes the Uniqueness of the line. Which is why, I guess,
  // the line update is held of until the response. The agent must
  // response to snoops to the line when the WriteUnique is in
  // progress.
  //
  void handle_cmd_write_unique(DirContext& ctxt, DirCommandList& cl,
                               const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::I:
      case State::S:
      case State::E:
      case State::M:
      case State::O:
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  // C4.8.3
  void handle_cmd_write_line_unique(DirContext& ctxt, DirCommandList& cl,
                                    const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::I:
      case State::S:
      case State::E:
      case State::M:
      case State::O:
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  // C4.8.4 Writeback
  //
  // Agent issues a WriteBack to main memory such that the line
  // contained by the LLC is up to date. Upon completion of the
  // command, the line is invalidated and is no longer resident in the
  // originators cache.
  //
  // Command presupposes that requesting agent has already written
  // line to appropriate location in LLC before issuing the command to
  // the directory. The directory itself is not responsible for
  // forwarding the data to the LLC.
  //
  void handle_cmd_write_back(DirContext& ctxt, DirCommandList& cl,
                             const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    const State state = line->state();
    switch (line->state()) {
      case State::I:
      case State::S: {
        // Writeback cannot be issued by agent when line is in the
        // Invalid or Shared states; line must be unique in a cache.
        using cc::to_string;
        std::string reason;
        reason += "Writeback issued by agent: ";
        reason += msg->origin()->path();
        reason += " but directory indicates that line is presently in the: ";
        reason += to_string(state);
        reason += " state.";
        cl.raise_error(reason);
      } break;
      case State::M:
      case State::O:
      case State::E: {
        // Form coherence result; line is now invalid.  Agent cache
        // controller must have first issued to the Dt to LLC and have
        // received its response before issuing the WriteBack to the
        // directory.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_dt_n(0);
        issue_msg_to_noc(ctxt, cl, end, msg->origin());
        // Line becomes idle.
        cl.push_back(line->build_update_state(State::I));
        // Perhaps the line can be retained by the LLC but be simply
        // non-resident in any of the child caches. 
        cl.push_back(DirOpcode::RemoveLine);
        cl.push_back(DirOpcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  // C4.8.5
  //
  // Agent issues a WriteClean to such that the corresponding line in
  // LLC is up to date. The agent retains its own copy of the cache
  // line, but in a Clean (E, S) state.
  //
  // As in the Writeback case, except that the line is retained upon
  // completion of the command.
  //
  void handle_cmd_write_clean(DirContext& ctxt, DirCommandList& cl,
                              const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    const State state = line->state();
    switch (state) {
      case State::I:
      case State::S: {
        // Writeback cannot be issued by agent when line is in the
        // Invalid or Shared states; line must be unique in a cache.
        using cc::to_string;
        std::string reason;
        reason += "Writeclean issued by agent: ";
        reason += msg->origin()->path();
        reason += " but directory indicates that line is presently in the: ";
        reason += to_string(state);
        reason += " state.";
        cl.raise_error(reason);
      } break;
      case State::E:
      case State::M:
      case State::O: {
        // Form coherence result; line is now invalid.  Agent cache
        // controller must have first issued to the Dt to LLC and have
        // received its response before issuing the WriteBack to the
        // directory.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_dt_n(0);
        // Issue coherence end message back to originator.
        issue_msg_to_noc(ctxt, cl, end, ctxt.tstate()->origin());
        // Line becomes idle.
        State next_state = State::X;
        switch (state) {
          // Line returns to Clean state from Unique, which is
          // Exclusive.
          case State::E: { next_state = State::E; } break;
          case State::M: { next_state = State::E; } break;
          // WriteClean does not invalidate lines present in other caches,
          // therefore line returns back to Shared state.
          case State::O: { next_state = State::S; } break;
          default:       { next_state = State::X; } break;
        }
        cl.push_back(line->build_update_state(next_state));
        // Transaction is complete
        cl.push_back(DirOpcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  // C4.9.1 Evict
  void handle_cmd_evict(DirContext& ctxt, DirCommandList& cl,
                        const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    switch (line->state()) {
      case State::I:
        // The spec does not specifically mention what happens in this
        // case as one cannot explicitly evict a line which has not
        // been installed in the cache. In this case, we simply ignore
        // the command and consider it to be spurious.
        [[fallthrough]];
      case State::S:
      case State::E: {
        // Line is installed in the Exclusive state within the
        // requester's cache. As the line is exclusive, remove the
        // from the directory cache and issue a response.

        // Issue coherence response, line has now been removed from cache.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        // No data transfer associated with Evict transaction.
        end->set_dt_n(0);
        issue_msg_to_noc(ctxt, cl, end, msg->origin());

        // TODO; del sharer/owner

        // Line becomes idle.
        cl.push_back(line->build_update_state(State::I));
        cl.push_back(DirOpcode::RemoveLine);
        cl.push_back(DirOpcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::M:
      case State::O: {
        using cc::to_string;

        // Cannot evict a line which is current in a modified state.
        std::string reason = "Agent: ";
        reason += msg->origin()->path();
        reason += " evicts cache line: ";
        reason += to_string(msg->addr());
        reason += " but line is currently in modified state: ";
        reason += to_string(line->state());
        cl.raise_error(reason);
      } break;
      default: {
        cl.raise_error("Not implemented");
      } break;        
    }
  }

  void apply(DirContext& ctxt, DirCommandList& cl, const LLCCmdRspMsg* msg) const {
    const AceCmdOpcode opcode = ctxt.tstate()->opcode();
    switch (opcode) {
      case AceCmdOpcode::ReadShared: {
        handle_llc_read_shared(ctxt, cl, msg);
      } break;
      case AceCmdOpcode::ReadUnique: {
        handle_llc_read_unique(ctxt, cl, msg);
      } break;
      default: {
        // LLC Resp is not expected.
      } break;
    }
    cl.next_and_do_consume(true);
  }

  // TODO; rationalize
  void handle_llc_read_shared(DirContext& ctxt, DirCommandList& cl,
                              const LLCCmdRspMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    DirTState* tstate = ctxt.tstate();
    const LLCCmdOpcode opcode = tstate->llc_cmd_opcode();
    switch (opcode) {
      case LLCCmdOpcode::Fill: {
        // Initial line installation in LLC; followed by PutLine to
        // requesting agent.

        // Issue Forward command to LLC to send newly installed line
        // to requesting agent.
        LLCCmdMsg* cmd = Pool<LLCCmdMsg>::construct();
        cmd->set_t(msg->t());
        cmd->set_opcode(LLCCmdOpcode::PutLine);
        cmd->set_addr(tstate->addr());
        cmd->set_agent(tstate->origin());
        issue_msg_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
        // Set flag indicating that we are waiting Put response.
        cl.push_back(tstate->build_set_llc_cmd_opcode(cmd->opcode()));
      } break;
      case LLCCmdOpcode::PutLine: {
        // On Put completion, transaction is now complete.

        // Issue coherence result to command originator.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        // May be retained by other cache which decided simply not to
        // relinquish the line upon a prior snoop.
        end->set_is(ctxt.is());
        // Cannot be dirty (by definition) if sourced from LLC.
        end->set_pd(false);
        // Only one DT since LLC would not have been queried if
        // an intervention had taken place.
        end->set_dt_n(1);
        issue_msg_to_noc(ctxt, cl, end, tstate->origin());
        // Update state
        const State next_state = end->is() ? State::S : State::E;
        cl.push_back(line->build_update_state(next_state));
        // Transaction ends
        cl.push_back(DirOpcode::EndTransaction);
      } break;
      default: {
      } break;
    }
  }

  void handle_llc_read_unique(DirContext& ctxt, DirCommandList& cl,
                              const LLCCmdRspMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    DirTState* tstate = ctxt.tstate();
    const LLCCmdOpcode opcode = tstate->llc_cmd_opcode();
    switch (opcode) {
      case LLCCmdOpcode::Fill: {
        // Initial line installation in LLC; followed by PutLine to
        // requesting agent.

        // Issue Forward command to LLC to send newly installed line
        // to requesting agent.
        LLCCmdMsg* cmd = Pool<LLCCmdMsg>::construct();
        cmd->set_t(msg->t());
        cmd->set_opcode(LLCCmdOpcode::PutLine);
        cmd->set_addr(tstate->addr());
        cmd->set_agent(tstate->origin());
        issue_msg_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
        // Set flag indicating that we are waiting Put response.
        cl.push_back(tstate->build_set_llc_cmd_opcode(cmd->opcode()));
      } break;
      case LLCCmdOpcode::PutLine: {
        // On Put completion, transaction is now complete.

        // Issue coherence result to command originator.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        // Cannot be Shared as ReadUnique
        end->set_is(false);
        // May have been passes as dirty
        end->set_pd(ctxt.pd());
        // Only one DT since LLC would not have been queried if
        // an intervention had taken place.
        end->set_dt_n(1);
        issue_msg_to_noc(ctxt, cl, end, tstate->origin());
        // Update state
        const State next_state = end->pd() ? State::M : State::E;
        cl.push_back(line->build_update_state(next_state));
        // Transaction ends
        cl.push_back(DirOpcode::EndTransaction);
      } break;
      default: {
        // Error out.
      } break;
    }
  }

  //
  //
  void apply(DirContext& ctxt, DirCommandList& cl, const CohSnpRspMsg* msg) const {
    DirTState* tstate = ctxt.tstate();

    // Update snoop response concensus.
    if (msg->dt()) {
      // DT in current snoop; increment total DT count.
      cl.push_back(tstate->build_inc_dt());
    }

    // Recall that the PD, IS bits are not themselves predicated on
    // PD, as the naming would imply.
    if (msg->pd()) {
      // Line was dirty before snoop;
      cl.push_back(tstate->build_inc_pd());
    }
    if (msg->is()) {
      // Line is retained as shared.
      cl.push_back(tstate->build_inc_is());
    }

    // Upon receipt of new snoop response, update transaction
    // state. If all snoop responses have been received by the agent, form
    // the final coherence concensus response.
    if (tstate->is_final_snoop(true)) { 
      // Compute final counts (tstate has not yet been updated at this
      // point).
      ctxt.set_dt_n(tstate->dt_i() + (msg->dt() ? 1 : 0));
      ctxt.set_pd_n(tstate->pd_i() + (msg->pd() ? 1 : 0));
      ctxt.set_is_n(tstate->is_i() + (msg->is() ? 1 : 0));
    
      const AceCmdOpcode opcode = ctxt.tstate()->opcode();
      switch (opcode) {
        case AceCmdOpcode::ReadNoSnoop: {
          handle_snp_read_no_snoop(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::ReadOnce: {
          handle_snp_read_once(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::ReadClean: {
          handle_snp_read_clean(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::ReadNotSharedDirty: {
          handle_snp_read_not_shared_dirty(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::ReadShared: {
          handle_snp_read_shared(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::ReadUnique: {
          handle_snp_read_unique(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::CleanUnique: {
          handle_snp_clean_unique(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::CleanShared: {
          handle_snp_clean_shared(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::CleanInvalid: {
          handle_snp_clean_invalid(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::MakeUnique: {
          handle_snp_make_unique(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::MakeInvalid: {
          handle_snp_make_invalid(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::WriteNoSnoop: {
          handle_snp_write_no_snoop(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::WriteUnique: {
          handle_snp_write_unique(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::WriteLineUnique: {
          handle_snp_write_line_unique(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::WriteBack: {
          handle_snp_write_back(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::WriteClean: {
          handle_snp_write_clean(ctxt, cl, msg);
        } break;
        case AceCmdOpcode::Evict: {
          handle_snp_evict(ctxt, cl, msg);
        } break;
        default: {
          // Unknown opcode; raise error.
          std::string reason = "Unsupported opcode received: ";
          reason += to_string(opcode);
          cl.raise_error(reason);
        } break;
      }
    }

    // Update (Snoop) Credit Counter.
    issue_add_credit(ctxt, cl, to_cmd_type(msg->cls()));
    // Consume and advance
    cl.next_and_do_consume(true);
  }

  void handle_snp_read_no_snoop(DirContext& ctxt, DirCommandList& cl,
                                const CohSnpRspMsg* msg) const {
    // No snoops associated with ReadNoSnoop
    cl.raise_error("Unexpected snoop received.");
  }

  void handle_snp_read_once(DirContext& ctxt, DirCommandList& cl,
                            const CohSnpRspMsg* msg) const {
  }

  void handle_snp_read_clean(DirContext& ctxt, DirCommandList& cl,
                             const CohSnpRspMsg* msg) const {
  }

  void handle_snp_read_not_shared_dirty(DirContext& ctxt, DirCommandList& cl,
                                        const CohSnpRspMsg* msg) const {
  }

  void handle_snp_read_shared(DirContext& ctxt, DirCommandList& cl,
                              const CohSnpRspMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::M_O:
      case State::O_O:
      case State::E_O: {
        // Compute next state
        State next_state = State::X;
        if      (!ctxt.is() && !ctxt.pd()) { next_state = State::E; }
        else if (!ctxt.is() &&  ctxt.pd()) { next_state = State::M; }
        else if ( ctxt.is() && !ctxt.pd()) { next_state = State::S; }
        else                               { next_state = State::O; }
        
        if (ctxt.dt()) {
          // Data has been transfered; the originator is now in
          // receipt of some number of lines. Now compute the final
          // coherence result and issue final response.

          CohEndMsg* end = Pool<CohEndMsg>::construct();
          end->set_t(msg->t());
          end->set_origin(ctxt.dir());
          end->set_is(ctxt.is());
          end->set_pd(ctxt.pd());
          end->set_dt_n(ctxt.dt_n());

          switch (next_state) {
            case State::E: {
              // Requesting agent becomes owner.
              cl.push_back(line->build_set_owner(tstate->origin()));
            } break;
            case State::M: {
              // Requesting agent becomes owner.
              cl.push_back(line->build_set_owner(tstate->origin()));
            } break;
            case State::S: {
              // Requester becomes Sharer.
              cl.push_back(line->build_add_sharer(tstate->origin()));
            } break;
            case State::O: {
              // Requester becomes Owner; responder retains Shared.
              cl.push_back(line->build_set_owner(tstate->origin()));
            }
            default: {
            } break;
          }
          // Issue response to originator.
          issue_msg_to_noc(ctxt, cl, end, tstate->origin());
          // Transaction ends
          cl.push_back(DirOpcode::EndTransaction);
        } else {
          // No data transfer, therefore DIR must issue fill to
          // LLC.
          LLCCmdMsg* cmd = Pool<LLCCmdMsg>::construct();
          cmd->set_t(msg->t());
          cmd->set_opcode(LLCCmdOpcode::PutLine);
          cmd->set_addr(tstate->addr());
          cmd->set_agent(tstate->origin());
          // Issue LLC request; now awaiting LLC response before
          // final coherence result can be issued to originator.
          issue_msg_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
          // Now awaiting an LLC response, set flag
          cl.push_back(tstate->build_set_llc_cmd_opcode(cmd->opcode()));
        }
        // Update state
        cl.push_back(line->build_update_state(next_state));
      } break;
      default: {
        cl.raise_error("Invalid state transition.");
      } break;
    }
  }

  void handle_snp_read_unique(DirContext& ctxt, DirCommandList& cl,
                              const CohSnpRspMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::S_ME:
      case State::O_ME:
      case State::M_ME:
      case State::E_E: {
        const State next_state = ctxt.pd() ? State::M : State::E;
        if (ctxt.dt()) {
          CohEndMsg* end = Pool<CohEndMsg>::construct();
          end->set_t(msg->t());
          end->set_origin(ctxt.dir());
          end->set_is(false);
          end->set_dt_n(ctxt.dt_n());
          end->set_pd(next_state == State::O);

          // Requesting agent becomes owner.
          cl.push_back(line->build_set_owner(tstate->origin()));

          // Issue completed response to the requester.
          issue_msg_to_noc(ctxt, cl, end, tstate->origin());

          // Transaction ends at this point, as the final state of
          // the line is now known. The requesters cache controller
          // now awaits 1 DT either from responding agent or the
          // LLC.
          cl.push_back(DirOpcode::EndTransaction);
        } else {
          // Data was not transfers to the requesting agent. In this
          // case, the owning agent must have explicitly written back
          // back its line before forming the snoop response. Ideally,
          // the responding agent would have relinquished the line
          // before responding, but the ACE specification does not
          // require this behavior.

          // Instruct the LLC to forward the cache line to the
          // requesting agent.
          LLCCmdMsg* llc = Pool<LLCCmdMsg>::construct();
          llc->set_t(msg->t());
          llc->set_opcode(LLCCmdOpcode::PutLine);
          llc->set_addr(tstate->addr());
          llc->set_agent(tstate->origin());
          issue_msg_to_noc(ctxt, cl, llc, ctxt.dir()->llc());

          // Set flag indicating that we are now awaiting a LLC
          // response.
          cl.push_back(tstate->build_set_llc_cmd_opcode(llc->opcode()));
        }
        cl.push_back(line->build_update_state(next_state));
      } break;
      default: {
        cl.raise_error("Invalid state transition.");
      } break;
    }
  }

  void handle_snp_clean_unique(DirContext& ctxt, DirCommandList& cl,
                               const CohSnpRspMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::S_E:
      case State::E_E:
      case State::M_EO: {
        // Final response, therefore send completed coherence
        // response back to requester to terminate the
        // transaction.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        // "Clean" instruction, therefore as no data is transferred
        // these fields are cleared as per. C4.6.1
        end->set_is(false);
        end->set_pd(false);
        issue_msg_to_noc(ctxt, cl, end, tstate->origin());

        // Transaction is complete once overall transaction
        // response has been computed.
        cl.push_back(DirOpcode::EndTransaction);
      } break;
      default: {
        cl.raise_error("Invalid state transition.");
      } break;
    }
  }

  void handle_snp_clean_shared(DirContext& ctxt, DirCommandList& cl,
                               const CohSnpRspMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::S_SE:
      case State::E_SE:
      case State::M_SE:
      case State::O_SE: {
        // Final snoop response received; issue final coherence
        // concensus.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        end->set_pd(false);
        end->set_is(ctxt.is());
        issue_msg_to_noc(ctxt, cl, end, tstate->origin());
        // On completion, line is clean.
        const State next_state = ctxt.is() ? State::S : State::E;
        cl.push_back(line->build_update_state(next_state));
        // Transaction is complete
        cl.push_back(DirOpcode::EndTransaction);
      } break;
      default: {
        cl.raise_error("Invalid state transition.");
      } break;
    }
  }

  void handle_snp_clean_invalid(DirContext& ctxt, DirCommandList& cl,
                                const CohSnpRspMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::S_I:
      case State::E_I:
      case State::M_I:
      case State::O_I: {
        // Final snoop response received; issue final coherence
        // concensus.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        // "the {IsShared/PassDirty} response must be deasserted"
        end->set_pd(false);
        end->set_is(false);
        issue_msg_to_noc(ctxt, cl, end, tstate->origin());
        // Update final line state
        cl.push_back(line->build_update_state(State::I));
        // Delete line
        cl.push_back(DirOpcode::RemoveLine);
        // Transaction is complete
        cl.push_back(DirOpcode::EndTransaction);
      } break;
      default: {
        cl.raise_error("Invalid state transition.");
      } break;
    }
  }

  void handle_snp_make_unique(DirContext& ctxt, DirCommandList& cl,
                              const CohSnpRspMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::S_I:
      case State::E_I:
      case State::M_I:
      case State::O_I: {
        // Final snoop response received; issue final coherence
        // concensus.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        end->set_pd(false);
        end->set_is(false);
        issue_msg_to_noc(ctxt, cl, end, tstate->origin());
        // Update final line state; originator now has Exclusive
        // access to the line and will, presumably, perform a full
        // line write. From the perspective of the directory,
        // Exclusive is synonymous with Modified.
        cl.push_back(line->build_update_state(State::E));
        // Originator is now the owner.
        cl.push_back(line->build_set_owner(line->owner()));
        //issue_set_owner(ctxt, cl, line->owner());
        // Delete line
        cl.push_back(DirOpcode::RemoveLine);
        // Transaction is complete
        cl.push_back(DirOpcode::EndTransaction);
      } break;
      default: {
        cl.raise_error("Invalid state transition.");
      } break;
    }
  }

  void handle_snp_make_invalid(DirContext& ctxt, DirCommandList& cl,
                               const CohSnpRspMsg* msg) const {
    DirTState* tstate = ctxt.tstate();
    LineState* line = static_cast<LineState*>(tstate->line());
    const State state = line->state();
    switch (state) {
      case State::S_I:
      case State::E_I:
      case State::M_I:
      case State::O_I: {
        // Final snoop response received; issue final coherence
        // concensus.
        CohEndMsg* end = Pool<CohEndMsg>::construct();
        end->set_t(msg->t());
        end->set_origin(ctxt.dir());
        end->set_pd(false);
        end->set_is(false);
        issue_msg_to_noc(ctxt, cl, end, tstate->origin());
        // Update final line state
        cl.push_back(line->build_update_state(State::I));
        // Delete line
        cl.push_back(DirOpcode::RemoveLine);
        // Transaction is complete
        cl.push_back(DirOpcode::EndTransaction);
      } break;
      default: {
        cl.raise_error("Invalid state transition.");
      } break;
    }
  }

  void handle_snp_write_no_snoop(DirContext& ctxt, DirCommandList& cl,
                                 const CohSnpRspMsg* msg) const {
    // Command should not issue a Snoop therefore we so not expect
    // a snoop response.
    cl.raise_error("Snoop response raised for command which does "
                   "not issue a snoop command");
  }

  void handle_snp_write_unique(DirContext& ctxt, DirCommandList& cl,
                               const CohSnpRspMsg* msg) const {
    // Command should not issue a Snoop therefore we so not expect
    // a snoop response.
    cl.raise_error("Snoop response raised for command which does "
                   "not issue a snoop command");
  }

  void handle_snp_write_line_unique(DirContext& ctxt, DirCommandList& cl,
                                    const CohSnpRspMsg* msg) const {
    // Command should not issue a Snoop therefore we so not expect
    // a snoop response.
    cl.raise_error("Snoop response raised for command which does "
                   "not issue a snoop command");
  }

  void handle_snp_write_back(DirContext& ctxt, DirCommandList& cl,
                             const CohSnpRspMsg* msg) const {
    // Command should not issue a Snoop therefore we so not expect
    // a snoop response.
    cl.raise_error("Snoop response raised for command which does "
                   "not issue a snoop command");
  }

  void handle_snp_write_clean(DirContext& ctxt, DirCommandList& cl,
                              const CohSnpRspMsg* msg) const {
    // Command should not issue a Snoop therefore we so not expect
    // a snoop response.
    cl.raise_error("Snoop response raised for command which does "
                   "not issue a snoop command");
  }

  void handle_snp_evict(DirContext& ctxt, DirCommandList& cl,
                        const CohSnpRspMsg* msg) const {
    // Command should not issue a Snoop therefore we so not expect
    // a snoop response.
    cl.raise_error("Snoop response raised for command which does "
                   "not issue a snoop command");
  }

  void issue_add_credit(DirContext& ctxt, DirCommandList& cl,
                        MessageClass cls) const {
    struct AddCreditAction : DirCoherenceAction {
      AddCreditAction() = default;
      std::string to_string() const override {
        KVListRenderer r;
        r.add_field("action", "add credit");
        r.add_field("cc", cc_->path());
        return r.to_string();
      }
      void set_cc(CreditCounter* cc) { cc_ = cc; }
      // No resources required (should always make "forward progress")
      bool execute() override {
        cc_->credit();
        return true;
      }

     private:
      CreditCounter* cc_ = nullptr;
    };
    const Agent* origin = ctxt.msg()->origin();
    if (CreditCounter* cc = ctxt.dir()->cc_by_cls_agent(cls, origin);
        cc != nullptr) {
      // Counter exists for this edge. Issue credit update aciton.
      AddCreditAction* action = new AddCreditAction;
      action->set_cc(cc);
      cl.push_back(action);
    }
  }

  void issue_msg_to_noc(DirContext& ctxt, DirCommandList& cl,
                        const Message* msg, Agent* dest) const {
    struct EmitMessageToNocAction : DirCoherenceAction {
      EmitMessageToNocAction() = default;

      std::string to_string() const override {
        KVListRenderer r;
        r.add_field("action", "emit message to noc");
        r.add_field("mq", port_->ingress()->path());
        r.add_field("msg", msg_->to_string());
        return r.to_string();
      }

      void set_port(NocPort* port) { port_ = port; }
      void set_msg(const NocMsg* msg) { msg_ = msg; }
      void set_dir(const DirAgent* dir) { dir_ = dir; }

      void set_resources(DirResources& r) const override {
        // Always require a NOC credit.
        r.set_noc_credit_n(r.noc_credit_n() + 1);

        const Agent* dest = msg_->dest();
        const Message* payload = msg_->payload();
        switch (payload->cls()) {
          case MessageClass::CohSnp: {
            r.set_coh_snp_n(dest, r.coh_snp_n(dest) + 1);
          } break;
          default: {
            // No resources required;
            //
            // DtRsp, CohSnpRsp are assumed to either have resources
            // reserved upon issue of their originator commands (Dt,
            // CohSnp), or are otherwise guarenteed to make forward
            // progress.
          } break;
        }
      }

      bool execute() override {
        // If a credit counter exists for at the destination for the current
        // MessageClass, deduct one credit, otherwise ignore.
        const Message* payload = msg_->payload();
        // Lookup counter map for current message class.
        if (CreditCounter* cc =
                dir_->cc_by_cls_agent(payload->cls(), msg_->dest());
            cc != nullptr) {
          // Credit counter is present, therefore deduct a credit before
          // message is issued.
          cc->debit();
        }
        // Deduct NOC credit
        CreditCounter* cc = port_->ingress_cc();
        cc->debit();

        // Issue message to queue.
        MessageQueue* mq = port_->ingress();
        return mq->issue(msg_);
      }

     private:
      // Message to issue to NOC.
      const NocMsg* msg_ = nullptr;
      // Destination Message Queue
      NocPort* port_ = nullptr;
      // Cache controller model
      const DirAgent* dir_ = nullptr;
    };
    // Encapsulate message in NOC transport protocol.
    NocMsg* nocmsg = Pool<NocMsg>::construct();
    nocmsg->set_t(msg->t());
    nocmsg->set_payload(msg);
    nocmsg->set_origin(ctxt.dir());
    nocmsg->set_dest(dest);
    // Issue Message Emit action.
    EmitMessageToNocAction* action = new EmitMessageToNocAction;
    action->set_port(ctxt.dir()->dir_noc__port());
    action->set_msg(nocmsg);
    action->set_dir(ctxt.dir());
    cl.push_back(action);
  }
};

}  // namespace

namespace cc::moesi {

DirProtocol* build_dir_protocol(kernel::Kernel* k) {
  return new MOESIDirProtocol(k);
}

}  // namespace cc::moesi
