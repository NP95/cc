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

#include "amba.h"
#include "l1cache.h"
#include "l2cache.h"
#include "moesi.h"
#include "protocol.h"
#include "utility.h"

namespace {

using namespace cc;

//
//
enum class State { X, I, IS, IE, S, E, EI, M, MI, O, OE };

//
//
const char* to_string(State state) {
  switch (state) {
    case State::X:
      return "X";
    case State::I:
      return "I";
    case State::IS:
      return "IS";
    case State::IE:
      return "IE";
    case State::S:
      return "S";
    case State::E:
      return "E";
    case State::EI:
      return "EI";
    case State::M:
      return "M";
    case State::MI:
      return "MI";
    case State::O:
      return "O";
    case State::OE:
      return "OE";
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
    case State::O:
      return true;
    default:
      return false;
  }
}

//
//
enum class L2EgressQueue {
  CCCmdQ,
  CCSnpRspQ,
  L1RspQ,
  Invalid
};

const char* to_string(L2EgressQueue q) {
  switch (q) {
    case L2EgressQueue::Invalid:
    default:
      return "Invalid";
  }
}

//
//
class LineState : public L2LineState {
 public:
  // Stable state status. deprecate
  bool is_stable() const { return true; }

  LineState() {}

  // Current line state.
  State state() const { return state_; }
  Agent* owner() const { return owner_; }
  const std::set<Agent*>& sharers() const { return sharers_; }

  // Set current line state
  void set_state(State state) { state_ = state; }

  // Line has an owner L1. Line may be present in L2, but not present
  // in any L1. This may occur after an eviction of a line from L1,
  // but where the L2 has chosen to retain the line and forgo an
  // immediate writeback.
  bool has_owner() const { return owner_ != nullptr; }

  // Set owning agent
  void set_owner(Agent* owner) { owner_ = owner; }

  // Add agent to sharer list
  bool add_sharer(Agent* agent) {
    auto p = sharers_.insert(agent);
    return p.second;
  }

  // Delete agent from sharer list
  bool del_sharer(Agent* agent) {
    bool success = false;
    if (auto it = sharers_.find(agent); it != sharers_.end()) {
      sharers_.erase(it);
      success = true;
    }
    return success;
  }

  // Clear sharer set
  void clr_sharer() { sharers_.clear(); }

 private:
  // Current line state
  State state_ = State::I;
  // Current owning agent
  Agent* owner_ = nullptr;
  // Current sharers
  std::set<Agent*> sharers_;
};

enum class LineUpdate {
  SetState,
  SetOwner,
  DelOwner,
  AddSharer,
  DelSharer,
  ClrSharer,
  Invalid
};

const char* to_string(LineUpdate update) {
  switch (update) {
    case LineUpdate::SetState:
      return "SetState";
    case LineUpdate::SetOwner:
      return "SetOwner";
    case LineUpdate::DelOwner:
      return "DelOwner";
    case LineUpdate::AddSharer:
      return "AddSharer";
    case LineUpdate::DelSharer:
      return "DelSharer";
    case LineUpdate::ClrSharer:
      return "ClrSharer";
    case LineUpdate::Invalid:
    default:
      return "Invalid";
  }
}

//
//
struct LineUpdateAction : public L2CoherenceAction {
  LineUpdateAction(LineState* line, LineUpdate update)
      : line_(line), update_(update) {}

  std::string to_string() const override {
    using cc::to_string;

    KVListRenderer r;
    switch (update_) {
      case LineUpdate::SetState: {
        r.add_field("action", "set_state");
        r.add_field("state", to_string(line_->state()));
        r.add_field("next_state", to_string(state_));
      } break;
      case LineUpdate::SetOwner: {
        r.add_field("action", "set_owner");
      } break;
      case LineUpdate::DelOwner: {
        r.add_field("action", "del_owner");
      } break;
      case LineUpdate::AddSharer: {
        r.add_field("action", "add_sharer");
      } break;
      case LineUpdate::DelSharer: {
        r.add_field("action", "del_sharer");
      } break;
      case LineUpdate::ClrSharer: {
        r.add_field("action", "clr_sharer");
      } break;
      default: {
      } break;
    }
    return r.to_string();
  }

  // Setters
  void set_state(State state) { state_ = state; }
  void set_agent(Agent* agent) { agent_ = agent; }

  bool execute() override {
    switch (update_) {
      case LineUpdate::SetState: {
        line_->set_state(state_);
      } break;
      case LineUpdate::SetOwner: {
        line_->set_owner(agent_);
      } break;
      case LineUpdate::DelOwner: {
        line_->set_owner(nullptr);
      } break;
      case LineUpdate::AddSharer: {
        line_->add_sharer(agent_);
      } break;
      case LineUpdate::DelSharer: {
        line_->del_sharer(agent_);
      } break;
      case LineUpdate::ClrSharer: {
        line_->clr_sharer();
      } break;
      default: {
      } break;
    }
    return true;
  }

 private:
  // Line state
  LineState* line_ = nullptr;
  // Line update opcode
  LineUpdate update_ = LineUpdate::Invalid;
  // Set update
  State state_ = State::X;
  // Agent of interest
  Agent* agent_ = nullptr;
};

//
//
class MOESIL2CacheProtocol : public L2CacheAgentProtocol {
  using cb = L2CommandBuilder;

 public:
  MOESIL2CacheProtocol(kernel::Kernel* k)
      : L2CacheAgentProtocol(k, "moesil2") {}

  //
  //
  L2LineState* construct_line() const override {
    LineState* line = new LineState();
    return line;
  }

  //
  //
  void apply(L2CacheContext& ctxt, L2CommandList& cl) const override {
    LineState* line = static_cast<LineState*>(ctxt.line());
    const Message* msg = ctxt.msg();
    const MessageClass cls = msg->cls();
    switch (cls) {
      case MessageClass::L2Cmd: {
        // CPU -> L1 command:
        apply(ctxt, cl, line, static_cast<const L2CmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::AceCmdRsp: {
        apply(ctxt, cl, line, static_cast<const AceCmdRspMsg*>(ctxt.msg()));
      } break;
      case MessageClass::AceSnoop: {
        apply(ctxt, cl, line, static_cast<const AceSnpMsg*>(ctxt.msg()));
      } break;
      default: {
        // Unknown message class; error
      } break;
    }
  }

  //
  //
  void evict(L2CacheContext& ctxt, L2CommandList& cl) const override {
    // TODO
  }

  void set_modified_status(L2CacheContext& ctxt,
                           L2CommandList& cl) const override {
    LineState* line = static_cast<LineState*>(ctxt.line());
    switch (line->state()) {
      case State::M: {
        LogMessage msg(
            "Attempt to set modified status of line already in M state.");
        msg.level(Level::Warning);
        log(msg);
      }
        [[fallthrough]];
      case State::O:
      case State::E: {
        // Set modified status of line; should really be in the E
        // state.  Still valid if performed from the M state but
        // redundant and suggests that something has gone awry.
        issue_update_state(ctxt, cl, line, State::M);
      } break;
      default: {
        LogMessage msg("Unable to set modified state; line is not owned.");
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

 private:
  void apply(L2CacheContext& ctxt, L2CommandList& cl, LineState* line,
             const L2CmdMsg* cmd) const {
    // Update Transaction State with data snooped from command message.
    L2TState* tstate = ctxt.tstate();
    tstate->set_addr(cmd->addr());
    tstate->set_l1cache(cmd->l1cache());
    tstate->set_opcode(cmd->opcode());

    // Lookup L2 to L1 response queue keyed on origin agent.
    MessageQueue* l2_l1__rsp_q =
        ctxt.l2cache()->l2_l1__rsp_q(tstate->l1cache());

    const L2CmdOpcode opcode = cmd->opcode();
    const State state = line->state();
    switch (state) {
      case State::I: {
        AceCmdMsg* msg = Pool<AceCmdMsg>::construct();
        msg->set_t(cmd->t());
        msg->set_addr(cmd->addr());
        switch (opcode) {
          case L2CmdOpcode::L1GetS: {
            // State I; requesting GetS (ie. Shared); issue ReadShared
            msg->set_opcode(AceCmdOpcode::ReadShared);
            // Update state
            issue_update_state(ctxt, cl, line, State::IS);
          } break;
          case L2CmdOpcode::L1GetE: {
            // State I; requesting GetE (i.e. Exclusive); issue ReadShared
            msg->set_opcode(AceCmdOpcode::ReadUnique);
            // Update state
            issue_update_state(ctxt, cl, line, State::IE);
          } break;
          default: {
          } break;
        }
        // Issue ACE command to the cache controller.
        issue_msg_to_queue(L2EgressQueue::CCCmdQ, cl, ctxt, msg, tstate);
        // Message is stalled on lookup transaction.
        // Install new entry in transaction table as the transaction
        // has now started and commands are inflight. The transaction
        // itself is not complete at this point.
        cl.push_back(L2Opcode::StartTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::S: {
        L2CmdRspMsg* msg = Pool<L2CmdRspMsg>::construct();
        msg->set_t(cmd->t());
        switch (opcode) {
          case L2CmdOpcode::L1GetS: {
            // Add agent to set of sharers
            issue_add_sharer(cl, line, tstate->l1cache());
            // L2 line remains in Shared state
          } break;
          case L2CmdOpcode::L1GetE: {
            // Line is presently Shared in multiple L1. Requester
            // requests line in Exclusive state. Requester may or may
            // not already have the line in the Shared state. (TODO:
            // might be some advantage to model the relative cost of
            // transport delay in both the have and have-not line
            // cases).
            // Requester becomes owner
            issue_set_owner(cl, line, tstate->l1cache());
            // Invalid all other copies of line.
            issue_set_l1_invalid_except(cl, ctxt.addr(), tstate->l1cache());
            // Line becomes Exclusive
            issue_update_state(ctxt, cl, line, State::E);
          } break;
          default: {
          } break;
        }
        // Issue response back to requester.
        issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, msg, tstate);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::O: {
        switch (opcode) {
          case L2CmdOpcode::L1GetS: {
            // L2 currently has a dirty copy of the line in its cache
            // and can therefore immediately service a request for a
            // line in the S state. Issue response and add requester
            // to set of sharers.

            // TODO: sharer state.
            // TODO: L1 should be in the context state at this point.

          } break;
          case L2CmdOpcode::L1GetE: {
            // L2 has line in Owning state, but must first promote the
            // line to the exclusive state. L2 already has the data it
            // requires

            // L2 already has the line, therefore simply issue a
            // CleanUnique command to invalidate other copies within
            // the system.
            AceCmdMsg* msg = Pool<AceCmdMsg>::construct();
            msg->set_t(cmd->t());
            msg->set_addr(cmd->addr());
            msg->set_opcode(AceCmdOpcode::CleanUnique);
            issue_msg_to_queue(L2EgressQueue::CCCmdQ, cl, ctxt, msg);
            // Update state: transitional Owner to Exclusive.
            issue_update_state(ctxt, cl, line, State::OE);
            // Command initiates a transaction, therefore consume the
            // message and install a new transaction object in the
            // transaction table.
            cl.push_back(L2Opcode::StartTransaction);
            // Consume and advance
            cl.next_and_do_consume(true);
          } break;
          default: {
          } break;
        }
      } break;
      case State::E: {
        const bool requester_is_owner = false;
        if (!requester_is_owner) {
          switch (opcode) {
            case L2CmdOpcode::L1GetS: {
              // Demote line in owner to Shared state, add requester to
              // set of sharers
              L2CmdRspMsg* msg = Pool<L2CmdRspMsg>::construct();
              msg->set_t(cmd->t());
              // L1 lines become sharers
              issue_set_l1_shared_except(cl, ctxt.addr(), tstate->l1cache());
              // cl.push_back(cb::from_opcode(L2Opcode::SetL1LinesShared));
              // Requester becomes sharer.
              msg->set_is(true);
              // No longer owning, therefore delete owner pointer.
              issue_del_owner(cl, line);
              issue_add_sharer(cl, line, tstate->l1cache());
              // Line becomes Shared
              issue_update_state(ctxt, cl, line, State::S);
              // Issue response to L1.
              issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, msg, tstate);
            } break;
            case L2CmdOpcode::L1GetE: {
              L2CmdRspMsg* msg = Pool<L2CmdRspMsg>::construct();
              msg->set_t(cmd->t());
              // Requester becomes owner
              msg->set_is(false);
              // L1 lines become sharers
              issue_set_l1_invalid_except(cl, ctxt.addr(), tstate->l1cache());
              // Requester becomes owner
              issue_set_owner(cl, line, tstate->l1cache());
              // Line remains in Exclusive state
              // Issue response to L1.
              issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, msg, tstate);
            } break;
            case L2CmdOpcode::L1Put: {
              const bool opt_silently_evict = false;
              if (!opt_silently_evict) {
                // Issue eviction notification to home directory. Line
                // is not modified therefore no Write command is
                // required.
                // Issue Evict command to home directory.
                AceCmdMsg* msg = Pool<AceCmdMsg>::construct();
                msg->set_t(cmd->t());
                msg->set_addr(cmd->addr());
                msg->set_opcode(AceCmdOpcode::Evict);
                issue_msg_to_queue(L2EgressQueue::CCCmdQ, cl, ctxt, msg);
                // Transition to Exclusive -> Invalid state; awaiting
                // receipt of AceCmdRspMsg for the Evict.
                issue_update_state(ctxt, cl, line, State::EI);
                // Current command commits:
                cl.push_back(L2Opcode::StartTransaction);
              } else {
                // Silently evict the cache line; line is clean with
                // respect to memory therefore no writeback required.
              }
            } break;
            default: {
              // Unknown command
            } break;
          }
          // Consume and advance
          cl.next_and_do_consume(true);
        } else {
          // Otherwise, somehow received a message from the owner
          // which (although unexpected) probably okay if not
          // redundant. Command completes immediately as it is simply
          // a NOP.
          L2CmdRspMsg* msg = Pool<L2CmdRspMsg>::construct();
          msg->set_t(cmd->t());
          issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, msg, tstate);
          // Consume and advance
          cl.next_and_do_consume(true);
        }
      } break;
      case State::M: {
        const bool requester_is_owner = false;
        if (!requester_is_owner) {
          switch (opcode) {
            case L2CmdOpcode::L1GetS: {
              // Requester requests line in the Shared state. Owning
              // agent can retain line, but must demote the line from
              // the Modified to Shared state. As the cache is
              // write-through, L2 already has an up-to date copy of
              // the modified data therefore we can simply modify the
              // state in the owners L1.

              // Issue response to requester.
              L2CmdRspMsg* msg = Pool<L2CmdRspMsg>::construct();
              msg->set_t(cmd->t());
              msg->set_is(true);
              issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, msg, tstate);
              // Current owner L1 relinqushes ownership.
              issue_del_owner(cl, line);
              // Add current requester to set of sharers for this
              // line.
              issue_add_sharer(cl, line, tstate->l1cache());
              // State becomes Owned (still dirty with respect to
              // memory).
              issue_update_state(ctxt, cl, line, State::O);
            } break;
            case L2CmdOpcode::L1GetE: {
              // Requester requests line in the Exclusive
              // state. Owning cache must relinquish line and invalid
              // its copy. Again, as the cache is write-through, L2
              // already has the most recent state in its memory.
              // Line is owned by requester

              // Issue response to requester.
              L2CmdRspMsg* msg = Pool<L2CmdRspMsg>::construct();
              msg->set_t(cmd->t());
              msg->set_is(false);
              issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, msg, tstate);
              // Invalidate L1 copies.
              issue_set_l1_invalid_except(cl, ctxt.addr(), tstate->l1cache());
              // Requester becomes owner
              issue_set_owner(cl, line, tstate->l1cache());
              // State remains modified.
              issue_clr_sharer(cl, line);
            } break;
            case L2CmdOpcode::L1Put: {
              // The line is Modified and therefore resides in only
              // one L1.
              //
              // Discussion:
              //
              // The Put operation simply informs L2 that L1 is
              // removing its line, it does not necessarily invoke a
              // writeback operation as hardware may choose to retain
              // the line in the modified state, but without a
              // designated L1 owner. Only once the line is itself
              // evicted from L2 will the line be written back. For
              // simplicity, we choose to evict the line on tht Put.
              //
              const bool writeback_on_put = true;
              if (writeback_on_put) {
                // Issue Writeback transaction to home directory.
                AceCmdMsg* msg = Pool<AceCmdMsg>::construct();
                msg->set_t(cmd->t());
                msg->set_addr(cmd->addr());
                msg->set_opcode(AceCmdOpcode::WriteBack);
                issue_msg_to_queue(L2EgressQueue::CCCmdQ, cl, ctxt, msg, tstate);
                // Update state Modified -> Invalid
                issue_update_state(ctxt, cl, line, State::MI);
                // Current command commits:
                cl.push_back(L2Opcode::StartTransaction);
              } else {
                // Retain line, but L1 is no longer an owner. Line is
                // now orphaned and owned by the L2.
                issue_del_owner(cl, line);
              }
            } break;
            default: {
            } break;
          }
          // Consume and advance
          cl.next_and_do_consume(true);
        } else {
          // Otherwise, requester is currently owner.
          L2CmdRspMsg* msg = Pool<L2CmdRspMsg>::construct();
          msg->set_t(cmd->t());
          issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, msg, tstate);
          // Consume and advance
          cl.next_and_do_consume(true);
        }
      } break;
      default: {
      } break;
    }
  }

  void apply(L2CacheContext& ctxt, L2CommandList& cl, LineState* line,
             const AceCmdRspMsg* msg) const {
    L2TState* tstate = ctxt.tstate();
    // Lookup L2 to L1 response queue keyed on origin agent.
    MessageQueue* l2_l1__rsp_q =
        ctxt.l2cache()->l2_l1__rsp_q(tstate->l1cache());

    const L2CmdOpcode opcode = tstate->opcode();
    const State state = line->state();
    switch (state) {
      case State::IS: {
        L2CmdRspMsg* rsp = Pool<L2CmdRspMsg>::construct();
        rsp->set_t(msg->t());
        // Always sending to the zeroth L1
        issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, rsp, tstate);
        // Compute final line state
        const bool is = msg->is(), pd = msg->pd();
        if (is && pd) {
          rsp->set_is(false);
          issue_update_state(ctxt, cl, line, State::O);
        } else if (!is && !pd) {
          rsp->set_is(false);
          issue_update_state(ctxt, cl, line, State::E);
        } else {
          rsp->set_is(true);
          issue_update_state(ctxt, cl, line, State::S);
        }
        // Transaction complete
        cl.push_back(L2Opcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::IE: {
        // Transition to Exclusive state
        L2CmdRspMsg* rsp = Pool<L2CmdRspMsg>::construct();
        rsp->set_t(msg->t());
        rsp->set_is(false);
        // Always sending to the zeroth L1
        issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, rsp, tstate);
        // Compute final line state
        if (msg->is()) {
          // Throw error cannot receive in the shared state.
        }
        // Compute next state; expect !is_shared, Ownership if recieving
        // dirty data otherwise Exclusive.
        const State next_state = msg->pd() ? State::O : State::E;
        issue_update_state(ctxt, cl, line, next_state);
        // Transaction complete
        cl.push_back(L2Opcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::OE: {
        // TODO: should perform some additional qualificiation on the command
        // type.

        L2CmdRspMsg* rsp = Pool<L2CmdRspMsg>::construct();
        rsp->set_t(msg->t());
        // Always sending to the zeroth L1
        issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, rsp, tstate);

        issue_update_state(ctxt, cl, line, State::E);
        // Transaction complete
        cl.push_back(L2Opcode::EndTransaction);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case State::EI: {
        // Exclusive -> Invalid:
        //
        // Edge from Exclusive to Invalid state; in response from an
        // eviction request from a line in the Exclusive state. Edge
        // cannot be entered from a Write{Back,Clean} transaction as
        // this would imply that the line was dirty.
        //
        // Response received from home directory indicating that line
        // has now been evicted.
        switch (opcode) {
          case L2CmdOpcode::L1Put: {
            // Inform L2 requestor (as per. AMBA spec. a empty "dummy"
            // response message is returned."
            L2CmdRspMsg* rsp = Pool<L2CmdRspMsg>::construct();
            rsp->set_t(msg->t());
            issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, rsp, tstate);

            // Line becomes Invalid in the cache. The line is also
            // subsequently removed from the cache.
            issue_update_state(ctxt, cl, line, State::I);
            cl.push_back(L2Opcode::RemoveLine);

            // Transaction ends
            cl.push_back(L2Opcode::EndTransaction);
            // Consume and advance
            cl.next_and_do_consume(true);
          } break;
          default: {
          } break;
        }
      } break;
      case State::MI: {
        // Modified -> Invalid
        //
        // In response to a WriteBack transaction.
        switch (opcode) {
          case L2CmdOpcode::L1Put: {
            // Inform L2 requestor (as per. AMBA spec. a empty "dummy"
            // response message is returned."
            L2CmdRspMsg* rsp = Pool<L2CmdRspMsg>::construct();
            rsp->set_t(msg->t());
            issue_msg_to_queue(L2EgressQueue::L1RspQ, cl, ctxt, rsp, tstate);

            // Line becomes Invalid in the cache. The line is also
            // subsequently removed from the cache.
            issue_update_state(ctxt, cl, line, State::I);
            cl.push_back(L2Opcode::RemoveLine);

            // Transaction ends
            cl.push_back(L2Opcode::EndTransaction);
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

  void apply(L2CacheContext& ctxt, L2CommandList& cl, LineState* line,
             const AceSnpMsg* msg) const {
    ctxt.set_addr(msg->addr());
    const AceSnpOpcode opcode = msg->opcode();
    switch (opcode) {
      case AceSnpOpcode::ReadShared: {
        AceSnpRspMsg* rsp = Pool<AceSnpRspMsg>::construct();
        rsp->set_t(msg->t());
        // In the siliently evicted case, there is no line.
        const State state = ctxt.silently_evicted() ? State::I : line->state();
        switch (state) {
          case State::I: {
            // Dir thinks cache has the line, but it doesn't. Line
            // must have been siliently evicted.
            rsp->set_dt(false);
          } break;
          case State::E: {
            // Demote to S or evict (I); dt or not dt.
            const bool retain = true;
            const bool dt = true;
            if (retain) {
              rsp->set_dt(dt);
              rsp->set_pd(false);
              rsp->set_is(true);
              rsp->set_wu(true);
              // Demote line to Shared state.
              issue_update_state(ctxt, cl, line, State::S);
              // Denote child L1 caches to Shared State, too.
              issue_set_l1_shared_except(cl, msg->addr());
            } else {
              // Relinquish line.
              rsp->set_dt(true);
              rsp->set_pd(false);
              rsp->set_is(false);
              rsp->set_wu(true);
              // Demote line to S state.
              issue_update_state(ctxt, cl, line, State::I);
              // Invalidate L1 child caches
              issue_set_l1_invalid_except(cl, msg->addr());
              // Delete line from cache
              cl.push_back(L2Opcode::RemoveLine);
            }
          } break;
          case State::S: {
            // Retain S or evict (I); dt or not dt.

            // Line remains in S state.
          } break;
          case State::M: {
            // Options:
            //
            //  1. Retain as owner
            //
            //  2. Evict and pass ownership.

            const bool retain_as_owner = true;

            if (retain_as_owner) {
              rsp->set_dt(true);
              rsp->set_pd(false);
              rsp->set_is(true);
              rsp->set_wu(true);

              issue_update_state(ctxt, cl, line, State::O);

              // Write-through cache, demote lines back to shared
              // state.
              issue_set_l1_shared_except(cl, msg->addr());
            } else {
              rsp->set_dt(true);
              rsp->set_pd(true);
              rsp->set_is(false);
              rsp->set_wu(true);

              issue_update_state(ctxt, cl, line, State::I);
              cl.push_back(L2Opcode::RemoveLine);

              // Write-through cache, therefore immediately evict
              // lines from child L1 cache.
              issue_set_l1_invalid_except(cl, msg->addr());
            }
          } break;
          default: {
            // TODO: decide what to do here.
          } break;
        }
        // Issue response to CC.
        L2CacheAgent* l2cache = ctxt.l2cache();
        issue_msg_to_queue(L2EgressQueue::CCSnpRspQ, cl, ctxt, rsp);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case AceSnpOpcode::ReadUnique: {
        AceSnpRspMsg* rsp = Pool<AceSnpRspMsg>::construct();
        rsp->set_t(msg->t());
        // C5.3.3 ReadUnique
        switch (line->state()) {
          case State::I:
          case State::S:
          case State::E: {
            rsp->set_dt(true);
            rsp->set_pd(false);
            rsp->set_is(false);
            rsp->set_wu(true);
          } break;
          case State::O:
          case State::M: {
            rsp->set_dt(true);
            rsp->set_pd(true);
            rsp->set_is(false);
            rsp->set_wu(true);
            // Transition back to invalid state, line is gone.
          } break;
          default: {
            // TODO: figure out transient states.
          } break;
        }
        // Issue response to CC.
        L2CacheAgent* l2cache = ctxt.l2cache();
        issue_msg_to_queue(L2EgressQueue::CCSnpRspQ, cl, ctxt, rsp);
        // Final state is Invalid
        issue_update_state(ctxt, cl, line, State::I);
        cl.push_back(L2Opcode::RemoveLine);
        issue_set_l1_invalid_except(cl, msg->addr());
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case AceSnpOpcode::MakeInvalid:
      case AceSnpOpcode::CleanInvalid: {
        AceSnpRspMsg* rsp = Pool<AceSnpRspMsg>::construct();
        rsp->set_t(msg->t());

        // C5.3.4 CleanInvalid
        //
        // Specificiation recommends that data is transferred only if
        // present in the dirty state. (The cache would typically not
        // be snooped in the dirty case as this would be the
        // initiating agent in the system for the command).
        //
        // C5.3.5 MakeInvalid
        //
        // Specification recommands that data is NOT transferred.
        switch (line->state()) {
          case State::O:
          case State::M:
            if (opcode != AceSnpOpcode::MakeInvalid) {
              // Transfer data in the CleanInvalid case.
              rsp->set_dt(true);
              rsp->set_pd(true);
            }
            [[fallthrough]];
          case State::I:
          case State::S:
          case State::E: {
            issue_update_state(ctxt, cl, line, State::I);
          } break;
          default: {
            // TODO
          } break;
        }
        // Issue response to CC.
        issue_msg_to_queue(L2EgressQueue::CCSnpRspQ, cl, ctxt, rsp);
        // Final state is Invalid
        issue_update_state(ctxt, cl, line, State::I);
        cl.push_back(L2Opcode::RemoveLine);
        issue_set_l1_invalid_except(cl, msg->addr());
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        LogMessage lm("Unknown opcode received: ");
        lm.append(cc::to_string(msg->opcode()));
        lm.level(Level::Fatal);
        log(lm);
      } break;
    }
  }

  void issue_update_state(L2CacheContext& ctxt, L2CommandList& cl,
                          LineState* line, State state) const {
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::SetState);
    update->set_state(state);
    cl.push_back(cb::from_action(update));
  }

  void issue_del_owner(L2CommandList& cl, LineState* line) const {
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::DelOwner);
    cl.push_back(cb::from_action(update));
  }

  void issue_add_sharer(L2CommandList& cl, LineState* line,
                        Agent* agent) const {
    LineUpdateAction* update =
        new LineUpdateAction(line, LineUpdate::AddSharer);
    update->set_agent(agent);
    cl.push_back(cb::from_action(update));
  }

  void issue_set_owner(L2CommandList& cl, LineState* line, Agent* agent) const {
    LineUpdateAction* update = new LineUpdateAction(line, LineUpdate::SetOwner);
    update->set_agent(agent);
    cl.push_back(cb::from_action(update));
  }

  void issue_clr_sharer(L2CommandList& cl, LineState* line) const {
    LineUpdateAction* update =
        new LineUpdateAction(line, LineUpdate::ClrSharer);
    cl.push_back(cb::from_action(update));
  }

  template <typename... AGENT>
  void issue_set_l1_invalid_except(L2CommandList& cl, addr_t addr,
                                   AGENT... excluded) const {
    // Issue L1 invalidate of current line, but add "agent" to set of
    // keep out agents. The command therefore allows agent to retain
    // the line whereas all other will be invalidated.
    L2Command* cmd = cb::from_opcode(L2Opcode::SetL1LinesInvalid);
    cmd->set_addr(addr);
    if constexpr (sizeof...(excluded) != 0) {
      std::vector<L1CacheAgent*>& agents = cmd->agents();
      for (const auto& agent : {excluded...}) {
        agents.push_back(agent);
      }
    }
    cl.push_back(cmd);
  }

  template <typename... AGENT>
  void issue_set_l1_shared_except(L2CommandList& cl, addr_t addr,
                                  AGENT... excluded) const {
    // Demote L1 lines to Shared except Agents contains with in the
    // excluded set.
    L2Command* cmd = cb::from_opcode(L2Opcode::SetL1LinesShared);
    cmd->set_addr(addr);
    if constexpr (sizeof...(excluded) != 0) {
      std::vector<L1CacheAgent*>& agents = cmd->agents();
      for (const auto& agent : {excluded...}) {
        agents.push_back(agent);
      }
    }
    cl.push_back(cmd);
  }

  void issue_msg_to_queue(L2EgressQueue eq, L2CommandList& cl,
                          L2CacheContext& ctxt, const Message* msg,
                          L2TState* tstate = nullptr) const {
    struct EmitMessageActionProxy : L2CoherenceAction {
      EmitMessageActionProxy() = default;

      std::string to_string() const override {
        KVListRenderer r;
        r.add_field("action", "emit message");
        r.add_field("mq", mq_->path());
        r.add_field("msg", msg_->to_string());
        return r.to_string();
      }

      // Setters:
      void set_eq(L2EgressQueue eq) { eq_ = eq; }
      void set_mq(MessageQueue* mq) { mq_ = mq; }
      void set_msg(const Message* msg) { msg_ = msg; }

      void set_resources(L2Resources& r) const override {
        switch (eq_) {
          case L2EgressQueue::CCCmdQ: {
            r.set_cc_cmd_n(r.cc_cmd_n() + 1);
          } break;
          case L2EgressQueue::CCSnpRspQ: {
            r.set_cc_snp_rsp_n(r.cc_snp_rsp_n() + 1);
          } break;
          case L2EgressQueue::L1RspQ: {
            r.set_l1_rsp_n(r.l1_rsp_n() + 1);
          } break;
          default: {
            // No resource requirement.
          } break;
        }
      }
      //
      bool execute() override { return mq_->issue(msg_); }

     private:
      //
      L2EgressQueue eq_ = L2EgressQueue::Invalid;
      //
      MessageQueue* mq_ = nullptr;
      //
      const Message* msg_ = nullptr;
    };
    EmitMessageActionProxy* action = new EmitMessageActionProxy;
    action->set_eq(eq);
    action->set_msg(msg);
    MessageQueue* mq = nullptr;
    switch (eq) {
      case L2EgressQueue::CCCmdQ: {
        mq = ctxt.l2cache()->l2_cc__cmd_q();
      } break;
      case L2EgressQueue::CCSnpRspQ: {
        mq = ctxt.l2cache()->l2_cc__snprsp_q();
      } break;
      case L2EgressQueue::L1RspQ: {
        mq = ctxt.l2cache()->l2_l1__rsp_q(tstate->l1cache());
      } break;
      default: {
        LogMessage lm("Unknown Egress Queue: ");
        lm.append(to_string(eq));
        log(lm);
      } break;
    }
    action->set_mq(mq);
    cl.push_back(L2CommandBuilder::from_action(action));
  }
};

}  // namespace

namespace cc::moesi {

L2CacheAgentProtocol* build_l2_protocol(kernel::Kernel* k) {
  return new MOESIL2CacheProtocol(k);
}

}  // namespace cc::moesi
