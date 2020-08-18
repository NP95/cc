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
enum class State { X, I, IE, IS, S, M, E, EO, EE, O, OO, OE };

//
//
const char* to_string(State state) {
  switch (state) {
    case State::X:
      return "X";
    case State::I:
      return "I";
    case State::IE:
      return "IE";
    case State::IS:
      return "IS";
    case State::S:
      return "S";
    case State::E:
      return "E";
    case State::EO:
      return "EO";
    case State::EE:
      return "EE";
    case State::M:
      return "M";
    case State::O:
      return "O";
    case State::OO:
      return "OO";
    case State::OE:
      return "OE";
    default:
      return "Invalid";
  }
}

State compute_final_state(bool is, bool pd) {
  State next = State::X;
  if (!is && !pd) next = State::E;
  if (!is && pd) next = State::M;
  if (is && !pd) next = State::S;
  if (is && pd) next = State::O;
  return next;
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
  //
  SubState substate() const { return substate_; }
  const std::set<Agent*>& sharers() const { return sharers_; }
  //
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

enum class LineUpdate { State, SubState, SetOwner, AddSharer, Invalid };

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

enum class TStateAction { Invalid, SetSnoopN, IncSnoopN, IncDt };

//
//
struct TStateUpdateAction : public DirCoherenceAction {
  TStateUpdateAction(DirTState* tstate, TStateAction action)
      : tstate_(tstate), action_(action) {}
  std::string to_string() const override {
    using std::to_string;
    KVListRenderer r;
    switch (action()) {
      case TStateAction::SetSnoopN: {
        r.add_field("action", "set_snoop_n_action");
        r.add_field("snoop_n", to_string(snoop_n_));
        r.add_field("snoop_i", to_string(0));
      } break;
      case TStateAction::IncSnoopN: {
        r.add_field("action", "inc_snoop_i_action");
      } break;
      case TStateAction::IncDt: {
        r.add_field("action", "inc_dt");
      } break;
      default: {
      } break;
    }
    return r.to_string();
  }

  // Getters
  TStateAction action() const { return action_; }
  std::size_t snoop_n() const { return snoop_n_; }

  // Setters
  void set_snoop_n(std::size_t snoop_n) { snoop_n_ = snoop_n; }

  bool execute() override {
    switch (action()) {
      case TStateAction::SetSnoopN: {
        tstate_->set_snoop_n(snoop_n_);
        tstate_->set_snoop_i(0);
      } break;
      case TStateAction::IncSnoopN: {
        tstate_->set_snoop_i(tstate_->snoop_n() + 1);
      } break;
      case TStateAction::IncDt: {
        tstate_->set_dt_i(tstate_->dt_i() + 1);
      } break;
      default: {
      } break;
    }
    return true;
  }

 private:
  DirTState* tstate_ = nullptr;
  std::size_t snoop_n_ = 0;
  ;
  TStateAction action_ = TStateAction::Invalid;
};

//
//
struct LineUpdateAction : public DirCoherenceAction {
  LineUpdateAction(LineState* line, LineUpdate update)
      : line_(line), update_(update) {}

  void set_state(State state) { state_ = state; }
  void set_substate(SubState substate) { substate_ = substate; }
  void set_agent(Agent* agent) { agent_ = agent; }
  void set_snoop_n(std::size_t snoop_n) { snoop_n_ = snoop_n; }

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
        if (state_ != State::X) {
          line_->set_state(state_);
        } else {
          // Assign invalid state; something has gone wrong.
        }
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
  //
  std::size_t snoop_n_ = 0;
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
    // Consume and advance
    cl.next_and_do_consume(true);
  }

  void apply(DirContext& ctxt, DirCommandList& cl, const CohCmdMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());

    // Update Transaction State with appropriate state contained
    // within the Command Message (also present in the context).
    DirTState* tstate = ctxt.tstate();
    tstate->set_opcode(msg->opcode());
    tstate->set_addr(msg->addr());

    // Issue command message response (returns credit).
    CohCmdRspMsg* rsp = Pool<CohCmdRspMsg>::construct();
    rsp->set_t(msg->t());
    rsp->set_origin(ctxt.dir());
    issue_msg_to_noc(ctxt, cl, rsp, msg->origin());

    //
    const State state = line->state();
    const AceCmdOpcode opcode = msg->opcode();
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
        issue_set_owner(ctxt, cl, msg->origin());
        // Update state
        issue_update_state(ctxt, cl, State::IE);
        issue_update_substate(ctxt, cl, SubState::AwaitingLLCFillRsp);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::S: {
        switch (opcode) {
          case AceCmdOpcode::ReadShared: {
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
            issue_add_sharer(ctxt, cl, msg->origin());
            issue_update_substate(ctxt, cl, SubState::AwaitingLLCFwdRsp);
            // Consume and advance
            cl.next_and_do_consume(true);
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
        switch (opcode) {
          case AceCmdOpcode::ReadShared: {
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
            snp->set_opcode(AceSnpOpcode::ReadShared);
            issue_msg_to_noc(ctxt, cl, snp, line->owner());

            // Issue one snoop; therefore await one snoop response
            issue_set_snoop_n(ctxt, cl, 1);

            // Directory has no notion of local modified status,
            // therefore when a line has been installed in the
            // Exclusive state, the directory asssumes that the cache
            // line has been modified by the owning agent.
            issue_update_state(ctxt, cl, State::EO);
            issue_update_substate(ctxt, cl, SubState::AwaitingCohSnpRsp);
            // Consume and advance
            cl.next_and_do_consume(true);
          } break;
          case AceCmdOpcode::ReadUnique: {
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

            issue_update_state(ctxt, cl, State::EE);
            issue_update_substate(ctxt, cl, SubState::AwaitingCohSnpRsp);
            // Consume and advance
            cl.next_and_do_consume(true);
          } break;
          case AceCmdOpcode::Evict: {
            // Line is installed in the Exclusive state within the
            // requester's cache. As the line is exclusive, remove the
            // from the directory cache and issue a response.

            // Issue coherence response, line has now been removed from cache.
            CohEndMsg* end = Pool<CohEndMsg>::construct();
            end->set_t(msg->t());
            // No data transfer associated with Evict transaction.
            end->set_dt_n(0);
            issue_msg_to_noc(ctxt, cl, end, msg->origin());

            // Line becomes idle.
            issue_update_state(ctxt, cl, State::I);
            cl.push_back(cb::from_opcode(DirOpcode::RemoveLine));
            cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
            // Consume and advance
            cl.next_and_do_consume(true);
          } break;
          case AceCmdOpcode::WriteBack: {
            // TOOD: form message to evict from LLC and write to
            // memory.

            // Form coherence result; should be gated on LLC response
            // message.
            CohEndMsg* end = Pool<CohEndMsg>::construct();
            end->set_t(msg->t());
            end->set_dt_n(0);
            issue_msg_to_noc(ctxt, cl, end, msg->origin());
            // Line becomes idle.
            issue_update_state(ctxt, cl, State::I);
            cl.push_back(cb::from_opcode(DirOpcode::RemoveLine));
            cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
            // Consume and advance
            cl.next_and_do_consume(true);
          } break;
          default: {
          } break;
        }
      } break;
      case State::O: {
        switch (opcode) {
          case AceCmdOpcode::ReadUnique: {
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
            std::size_t snoop_n = 0;

            // Issue snoop to owner. Owner should forwarded line to
            // requesting agent and relinquish ownership of the line.
            CohSnpMsg* snp = Pool<CohSnpMsg>::construct();
            snp->set_t(msg->t());
            snp->set_addr(msg->addr());
            snp->set_origin(ctxt.dir());
            snp->set_agent(msg->origin());
            snp->set_opcode(to_snp_opcode(opcode));
            issue_msg_to_noc(ctxt, cl, snp, line->owner());
            snoop_n++;

            // TODO:
            // Any sharers should be invalidated.

            // Set expected snoop response count in transaction state
            // object.
            issue_set_snoop_n(ctxt, cl, snoop_n);

            // Update state: O -> OO -> O; current owner reliquishes
            // ownership of the line and forwards ownership to the
            // requesting agent.
            issue_update_state(ctxt, cl, State::OO);
            // Consume and advance
            cl.next_and_do_consume(true);
          } break;
          case AceCmdOpcode::CleanUnique: {
            // Requesting cache has line presently installed in its
            // cache but requests ownership of the line so that it can
            // (presumably) complete a write operation to the line.
            // Issue invalidate request to all sharers of the line and
            // then forward the completed response back to the
            // originating agent.

            // Issue invalidation snoops to sharing agents
            std::size_t snoop_n = 0;
            for (Agent* agent : line->sharers()) {
              // Do not sent message to originator as redundant.
              if (agent == msg->origin()) continue;

              // Issue snoop.
              CohSnpMsg* snp = Pool<CohSnpMsg>::construct();
              snp->set_t(msg->t());
              snp->set_addr(msg->addr());
              snp->set_origin(ctxt.dir());
              snp->set_agent(msg->origin());
              snp->set_opcode(to_snp_opcode(opcode));
              issue_msg_to_noc(ctxt, cl, snp, agent);
              snoop_n++;
            }

            // Set expected snoop response count in transaction state
            // object.
            issue_set_snoop_n(ctxt, cl, snoop_n);

            // Update state: transition Owner -> Exclusive.
            issue_update_state(ctxt, cl, State::OE);

            // Consume and advance
            cl.next_and_do_consume(true);
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
  }

  //
  //
  void apply(DirContext& ctxt, DirCommandList& cl,
             const LLCCmdRspMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    const State state = line->state();
    switch (state) {
      case State::IS:
      case State::IE: {
        switch (line->substate()) {
          case SubState::AwaitingLLCFillRsp: {
            // LLC line fill has completed. Now, instruct the LLC to
            // forward the desired line to the requesting (owning)
            // agent.
            LLCCmdMsg* cmd = Pool<LLCCmdMsg>::construct();
            cmd->set_t(msg->t());
            cmd->set_opcode(LLCCmdOpcode::PutLine);
            DirTState* tstate = ctxt.tstate();
            cmd->set_addr(tstate->addr());
            cmd->set_agent(tstate->origin());
            issue_msg_to_noc(ctxt, cl, cmd, ctxt.dir()->llc());
            cl.push_back(cb::from_opcode(DirOpcode::MsgConsume));

            issue_update_substate(ctxt, cl, SubState::AwaitingLLCFwdRsp);
            // Advance
            cl.next_and_do_consume(false);
          } break;
          case SubState::AwaitingLLCFwdRsp: {
            // Requesting cache now has the line, issue notification
            // of the line status and terminate the transction.
            // Send CohEnd
            CohEndMsg* end = Pool<CohEndMsg>::construct();
            end->set_t(msg->t());
            end->set_origin(ctxt.dir());
            end->set_dt_n(1);
            issue_msg_to_noc(ctxt, cl, end, line->owner());
            const State next_state =
                (line->state() == State::IS) ? State::S : State::E;
            issue_update_state(ctxt, cl, next_state);
            // Return to stable/non-transient state.
            issue_update_substate(ctxt, cl, SubState::None);

            cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
            // Consume and advance
            cl.next_and_do_consume(true);
          } break;
          default: {
          }
        }
      } break;
      case State::S: {
        switch (line->substate()) {
          case SubState::AwaitingLLCFwdRsp: {
            CohEndMsg* end = Pool<CohEndMsg>::construct();
            end->set_t(msg->t());
            end->set_origin(ctxt.dir());
            end->set_dt_n(1);
            issue_msg_to_noc(ctxt, cl, end, ctxt.tstate()->origin());
            issue_update_state(ctxt, cl, State::S);
            // Return to stable/non-transient state.
            issue_update_substate(ctxt, cl, SubState::None);

            cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
            // Consume and advance
            cl.next_and_do_consume(true);
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
  void apply(DirContext& ctxt, DirCommandList& cl,
             const CohSnpRspMsg* msg) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());

    // Obtain transaction object.
    DirTState* tstate = ctxt.tstate();

    // Transition from Owned (with some number of sharers) to
    // the exclusive state. Directory is awaiting response(s)
    // from all of the sharing agents in the system. Snooped
    // agents may or may not transfer data to the requester
    // (preferrably not), however since they may do so, send
    // the final DT count to the requester so that it may
    // await all DT which may be in flight.
    const bool is_final_snprsp = (tstate->snoop_n() == 1 + tstate->snoop_i());

    const AceCmdOpcode opcode = tstate->opcode();
    const State state = line->state();
    switch (opcode) {
      case AceCmdOpcode::CleanUnique: {
        switch (state) {
          case State::OE: {
            // Issue snoop count increment.
            issue_inc_snoop_i(ctxt, cl);

            // If data has been transfered, increment DT count.
            if (msg->dt()) {
              issue_inc_dt(ctxt, cl);
            }

            if (is_final_snprsp) {
              // Final response, therefore send completed coherence
              // response back to requester to terminate the
              // transaction.
              CohEndMsg* end = Pool<CohEndMsg>::construct();
              end->set_t(msg->t());
              end->set_origin(ctxt.dir());
              // "Clean" instruction, therefore as data is transferred
              // these fields are cleared as per. C4.6.1
              end->set_is(false);
              end->set_pd(false);
              // Set expected transfer count; nominally zero.
              const std::size_t final_dt_n =
                  tstate->dt_i() + (msg->dt() ? 1 : 0);
              end->set_dt_n(final_dt_n);

              issue_msg_to_noc(ctxt, cl, end, tstate->origin());

              // Transaction is complete once overall transaction
              // response has been computed.
              cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
            }
          } break;
          default: {
            // TODO
          } break;
        }
      } break;
      case AceCmdOpcode::ReadShared: {
        //
        switch (state) {
          case State::EO: {
            // CHECK: expect this to be the final snoop response; only
            // one snoop should have been issued.

            // Line was in Exclusive state in owning cache. Upon
            // completion of the ReadShared the owning cache may have
            // either invalidated the line, retained the line as the
            // owner, or it may have passed ownership to the
            // requesting agent.  Update state of line based upon
            // response.
            CohEndMsg* end = Pool<CohEndMsg>::construct();
            end->set_t(msg->t());
            end->set_origin(ctxt.dir());

            const bool is = msg->is(), pd = msg->pd(), dt = msg->dt();
            State next_state = State::X;
            if (dt && !is && !pd) {
              // Requesting agent becomes owner.
              end->set_is(false);
              end->set_pd(false);
              end->set_dt_n(1);
              issue_set_owner(ctxt, cl, tstate->origin());
              // Line is no longer present in response cache.
              next_state = State::E;
            } else if (dt && !is && pd) {
              // Requesting agent becomes owner.
              end->set_is(false);
              end->set_pd(true);
              end->set_dt_n(1);
              issue_set_owner(ctxt, cl, tstate->origin());
              // Line is no longer present in responder cache.
              next_state = State::M;
            } else if (dt && is && !pd) {
              // Requester becomes Sharer.
              end->set_is(true);
              end->set_pd(false);
              end->set_dt_n(1);
              issue_add_sharer(ctxt, cl, tstate->origin());
              // Responder becomes owner of assumed dirty line.
              next_state = State::O;
            } else if (dt && is && pd) {
              // Requester becomes Owner; responder retains Shared.
              end->set_is(true);
              end->set_pd(true);
              end->set_dt_n(1);
              issue_set_owner(ctxt, cl, tstate->origin());
              // Responder retains line in Shared state, line is dirty
              // with respect to memory therefore requester gains line
              // in ownership state.
              next_state = State::O;
            } else {
              // No data transfer, therefore DIR must issue fill to
              // LLC.

              // TODO: Cannot handle case !dt
            }
            issue_update_state(ctxt, cl, next_state);

            issue_msg_to_noc(ctxt, cl, end, tstate->origin());

            cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
          } break;
          default: {
          } break;
        }
      } break;
      case AceCmdOpcode::ReadUnique: {
        switch (line->state()) {
          case State::OO: {
            // Line is presently in the Owned state; owning cache
            // has been snooped and we *expect* for it to forward
            // the line to the requesting agent (TODO: agent may
            // instead choose to writeback line to main memory.
            // in which case the line may need to be sourced from
            // the LLC).
            const bool is = msg->is(), pd = msg->pd(), dt = msg->dt();
            if (is) {
              // Cannot obtain line with IsShared property set. Ownership
              // must have been relinqusihed.
              throw std::runtime_error("Cannot receive with IS.");
            }

            CohEndMsg* end = Pool<CohEndMsg>::construct();
            end->set_t(msg->t());
            end->set_origin(ctxt.dir());
            end->set_is(false);

            State next_state = State::X;
            if (dt) {
              // Responder was in Owned state, therefore we expect
              // the line to be dirty (but presently unclear if this
              // is strictly always the case as we cannot discount
              // edges to the Owned state from a Clean/Unmodified
              // line).
              end->set_pd(pd);
              end->set_dt_n(1);
              // Requester now has ownership of line.
              next_state = State::O;
            } else {
              // Else, data has not been transferred, line must be
              // sourced from the LLC as the responding agent has
              // written back the line.

              // TODO
            }
            issue_update_state(ctxt, cl, next_state);
            // Ownership is transferred to requester.
            issue_set_owner(ctxt, cl, tstate->origin());
            // Issue coherence response to the requester
            issue_msg_to_noc(ctxt, cl, end, tstate->origin());
            // Transaction is complete.
            cl.push_back(DirOpcode::EndTransaction);
          } break;
          case State::EE: {
            // Line is presently in the Exclusive state in the owning
            // cache (possibly modified). Exclusive ownership is to be
            // passed to the requesting agent. Therefore the state
            // transition is: E -> EE -> E.
            const bool is = msg->is(), pd = msg->pd(), dt = msg->dt();
            if (is) {
              // Cannot obtain line with IsShared property set.
              throw std::runtime_error("Cannot receive with IS.");
            }

            CohEndMsg* end = Pool<CohEndMsg>::construct();
            end->set_t(msg->t());
            end->set_origin(ctxt.dir());
            end->set_is(false);

            State next_state = State::X;
            if (dt && !pd) {
              issue_set_owner(ctxt, cl, tstate->origin());
              // Line is clean. The requesting agent can evict the
              // line without having first written it back.
              end->set_pd(false);
              end->set_dt_n(1);
              next_state = State::E;
            } else if (dt && pd) {
              issue_set_owner(ctxt, cl, tstate->origin());
              // Line is dirty, therefore cache obtains line in
              // ownership state and must writeback the line before
              // eviction.
              end->set_pd(true);
              end->set_dt_n(1);
              next_state = State::O;
            } else {
              //
              // TODO: agent does not forward data
            }
            issue_update_state(ctxt, cl, next_state);

            // Issue completed response to the requester.
            issue_msg_to_noc(ctxt, cl, end, tstate->origin());

            // Transaction ends at this point, as the final state of
            // the line is now known. The requesters cache controller
            // now awaits 1 DT either from responding agent or the
            // LLC.
            cl.push_back(cb::from_opcode(DirOpcode::EndTransaction));
          } break;
          default: {
            // TODO
          } break;
        }
      } break;
      default: {
        // Unhandled AMBA command
      } break;
    }

    // Update (Snoop) Credit Counter.
    issue_add_credit(ctxt, cl, to_cmd_type(msg->cls()));
    // Consume and advance
    cl.next_and_do_consume(true);
  }

  //
  //
  void issue_update_state(DirContext& ctxt, DirCommandList& cl,
                          State state) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::State);
    update->set_state(state);
    cl.push_back(cb::from_action(update));
  }

  //
  //
  void issue_update_substate(DirContext& ctxt, DirCommandList& cl,
                             SubState state) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::SubState);
    update->set_substate(state);
    cl.push_back(cb::from_action(update));
  }

  //
  //
  void issue_set_owner(DirContext& ctxt, DirCommandList& cl,
                       Agent* owner) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::SetOwner);
    update->set_agent(owner);
    cl.push_back(cb::from_action(update));
  }

  void issue_add_sharer(DirContext& ctxt, DirCommandList& cl,
                        Agent* owner) const {
    LineState* line = static_cast<LineState*>(ctxt.tstate()->line());
    LineUpdateAction* update =
        new LineUpdateAction(line, LineUpdate::AddSharer);
    update->set_agent(owner);
    cl.push_back(cb::from_action(update));
  }

  void issue_set_snoop_n(DirContext& ctxt, DirCommandList& cl,
                         std::size_t n) const {
    TStateUpdateAction* action =
        new TStateUpdateAction(ctxt.tstate(), TStateAction::SetSnoopN);
    action->set_snoop_n(n);
    cl.push_back(cb::from_action(action));
  }

  void issue_inc_snoop_i(DirContext& ctxt, DirCommandList& cl) const {
    TStateUpdateAction* action =
        new TStateUpdateAction(ctxt.tstate(), TStateAction::IncSnoopN);
    cl.push_back(cb::from_action(action));
  }

  void issue_inc_dt(DirContext& ctxt, DirCommandList& cl) const {
    TStateUpdateAction* action =
        new TStateUpdateAction(ctxt.tstate(), TStateAction::IncDt);
    cl.push_back(cb::from_action(action));
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
      cl.push_back(cb::from_action(action));
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
