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

#include "protocol.h"
#include "l1cache.h"
#include "l2cache.h"
#include "kernel.h"
#include "utility.h"

namespace {

using namespace cc;

enum class State {
  // Invalid
  I,
  IS,
  IE,
  // Shared
  S,
  SI,
  SE,
  // Exclusive
  E,
  EI,
  // Modified
  M,
  MI
};

//
//
const char* to_string(State state) {
  switch (state) {
    case State::I: return "I";
    case State::IS: return "IS";
    case State::IE: return "IE";
    case State::S: return "S";
    case State::SI: return "SI";
    case State::SE: return "SE";
    case State::E: return "E";
    case State::EI: return "EI";
    case State::M: return "M";
    case State::MI: return "MI";
    default: return "Invalid";
  }
};

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
class MOESIL1LineState : public L1LineState {
 public:
  MOESIL1LineState() {}

  // Current line state.
  State state() const { return state_; }
  void set_state(State state) { state_ = state; }

  // Stable state status.
  bool is_stable() const override {
    switch (state()) {
      case State::I:
      case State::S:
      case State::E:
      case State::M:
        return true;
      default:
        return false;
    }
  }

  // Return true if line is in a state which allows reading.
  bool is_readable() const override {
    switch (state()) {
      case State::S:
      case State::E:
      case State::M:
        return true;
      default:
        return false;
    }
  }

  // Return true if the line is in a state which allows writing.
  bool is_writeable() const override {
    switch (state()) {
      case State::E:
      case State::M:
        return true;
      default:
        return false;
    }
  }

 private:
  State state_ = State::I;
};

//
//
class MOESIL1CacheProtocol : public L1CacheAgentProtocol {
  using cb = L1CommandBuilder;
  
 public:
  MOESIL1CacheProtocol(kernel::Kernel* k) :
      L1CacheAgentProtocol(k, "moesil2")
  {}

  //
  //

  L1LineState* construct_line() const override {
    return new MOESIL1LineState();
  }

  //
  //
  void apply(L1CacheContext& ctxt, L1CommandList& cl) const override {
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(ctxt.line());
    const MessageClass cls = ctxt.msg()->cls();
    switch(cls) {
      case MessageClass::L1Cmd: {
        // CPU -> L1 command:
        apply(ctxt, cl, line, static_cast<const L1CmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::L2CmdRsp: {
        apply(ctxt, cl, line, static_cast<const L2CmdRspMsg*>(ctxt.msg()));
      } break;
      default: {
        // Unknown message class; error
      } break;
    }
  }

  //
  //
  void evict(L1CacheContext& ctxt, L1CommandList& cl) const override {

    // Update Transaction State with data snooped from command message.
    L1TState* tstate = ctxt.tstate();
    // Address becomes eviction address
    tstate->set_addr(ctxt.addr());
    // Line becomes evictee.
    tstate->set_line(ctxt.line());
    
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(ctxt.line());
    const State state = line->state();
    switch (state) {
      case State::M:
        // As in the Exclusive state, L2 already has the most recent
        // upto date copy of the line therefore we just need to inform
        // L2 that the line is being removed from the cache, and it
        // will itself perform the writeback.
      case State::E: {
        // Evict line present in the Exclusive state. L2 should
        // already have a recent upto date copy of the cache line,
        // therefore no actual data needs to be written-back, however
        // we inform L2 so that it may (conditionally) inform the home
        // directory that the line is no longer present in the cache
        // (when silent evictions are not allowed).

        // Issue command notification to L2.
        L2CmdMsg* cmd = new L2CmdMsg;
        cmd->set_t(ctxt.msg()->t());
        cmd->set_addr(ctxt.addr());
        cmd->set_opcode(L2CmdOpcode::L1Put);
        cmd->set_l1cache(ctxt.l1cache());
        issue_msg(cl, ctxt.l1cache()->l1_l2__cmd_q(), cmd);
        // Line transition to Invalid state.
        const State next_state = (state == State::M) ? State::MI : State::EI;
        issue_update_state(cl, line, next_state);
        // Start new transaction as we are awaiting the response from
        // L2 before the line is evicted from the cache.
        cl.transaction_start();
        // Source Message Queue is blocked until the current
        // transaction (lookup to L2) has completed.
        cl.push_back(cb::from_opcode(L1Opcode::MqSetBlockedOnTransaction));
        // Advance to next
        cl.next_and_do_consume(false);
      } break;
      default: {
      } break;
    }
  }

  //
  //
  void set_line_shared_or_invalid(
      L1CacheContext& ctxt, L1CommandList& cl, bool shared) const override {
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(ctxt.line());
    issue_update_state(cl, line, shared ? State::S : State::I);
    if (!shared) {
      cl.push_back(cb::build_remove_line(ctxt.addr()));
    }
  }

 private:

  void apply(L1CacheContext& ctxt, L1CommandList& cl, MOESIL1LineState* line,
             const L1CmdMsg* msg) const {

    // Update Transaction State with data snooped from command message.
    L1TState* tstate = ctxt.tstate();
    tstate->set_line(ctxt.line());
    tstate->set_addr(msg->addr());
    tstate->set_opcode(msg->opcode());
    
    const State state = line->state();
    switch (state) {
      case State::I: {
        // Emit request to L2.
        L2CmdMsg* l2cmdmsg = new L2CmdMsg();
        l2cmdmsg->set_t(msg->t());
        l2cmdmsg->set_addr(msg->addr());
        switch (msg->opcode()) {
          case L1CmdOpcode::CpuLoad: {
            // Update state I -> IS
            issue_update_state(cl, line, State::IS);
            // Issue GetS
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetS);
          } break;
          case L1CmdOpcode::CpuStore: {
            // Update state I -> IE
            issue_update_state(cl, line, State::IE);
            // Issue GetS
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetE);
          } break;
          default: {
          } break;
        }
        l2cmdmsg->set_l1cache(ctxt.l1cache());
        // Issue L2 command
        issue_msg(cl, ctxt.l1cache()->l1_l2__cmd_q(), l2cmdmsg);
        // Message is stalled on lookup transaction.
        // Install new entry in transaction table as the transaction
        // has now started and commands are inflight. The transaction
        // itself is not complete at this point.
        cl.transaction_start();
        // cl.push_back(cb::from_opcode(L1Opcode::StartTransaction));
        // Source Message Queue is blocked until the current
        // transaction (lookup to L2) has completed.
        cl.push_back(cb::from_opcode(L1Opcode::MqSetBlockedOnTransaction));
        // Advance to next
        cl.next_and_do_consume(false);
      } break;
      case State::S: {
        // Line is present in the cache. 
        switch (msg->opcode()) {
          case L1CmdOpcode::CpuLoad: {
            // LD to line in S-state can complete immediately. Forward
            // the response to the CPU.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            issue_msg(cl, ctxt.l1cache()->l1_cpu__rsp_q(), rsp);
            // Advance to next
            cl.next_and_do_consume(true);
          } break;
          case L1CmdOpcode::CpuStore: {
            // Store instruction to line in S-state. Line must be promoted to
            // the E state before the transaction can complete.

            // Issue GetE message to owning L2 cache.
            L2CmdMsg* l2cmdmsg = new L2CmdMsg();
            l2cmdmsg->set_t(msg->t());
            l2cmdmsg->set_addr(msg->addr());
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetE);
            l2cmdmsg->set_l1cache(ctxt.l1cache());
            // Issue L2 command
            issue_msg(cl, ctxt.l1cache()->l1_l2__cmd_q(), l2cmdmsg);
            // Start new transaction
            cl.transaction_start();
            // cl.push_back(cb::from_opcode(L1Opcode::StartTransaction));
            // Source Message Queue is blocked until the current
            // transaction (lookup to L2) has completed.
            // Set blocked status in Message Queue to rescind requestor
            // status.
            cl.push_back(cb::from_opcode(L1Opcode::MqSetBlockedOnTransaction));
            // Advance to next
            cl.next_and_do_consume(false);
            // State transitions to SE state.
            issue_update_state(cl, line, State::SE);
          } break;
          default: {
            // Invalid command.
          } break;
        }
      } break;
      case State::M: {
        switch (msg->opcode()) {
          case L1CmdOpcode::CpuLoad: {
            // Ld from M-state; commits immediately
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            // Issue response to CPU.
            issue_msg(cl, ctxt.l1cache()->l1_cpu__rsp_q(), rsp);
            // Advance to next and consume
            cl.next_and_do_consume(true);
          } break;
          case L1CmdOpcode::CpuStore: {
            // St to M-state; commits immediate.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            // Issue response to CPU.
            issue_msg(cl, ctxt.l1cache()->l1_cpu__rsp_q(), rsp);
            // Advance to next and consume
            cl.next_and_do_consume(true);
          } break;
          default: {

          } break;
        }
      } break;
      case State::E: {
        switch (msg->opcode()) {
          case L1CmdOpcode::CpuLoad: {
            // Load to line in the exclusive state; instruction can complete
            // successfully.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            issue_msg(cl, ctxt.l1cache()->l1_cpu__rsp_q(), rsp);
            // Advance to next and consume
            cl.next_and_do_consume(true);
          } break;
          case L1CmdOpcode::CpuStore: {
            // Store to line the exclusive state; instruction can complete
            // but line must transition to the modified state.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            issue_msg(cl, ctxt.l1cache()->l1_cpu__rsp_q(), rsp);
            issue_update_state(cl, line, State::M);
            // Write through to L2. such that L2 sees the transition
            // to M immediately.
            cl.push_back(cb::from_opcode(L1Opcode::SetL2LineModified));
            // Advance to next and consume
            cl.next_and_do_consume(true);
          } break;
          default: {
            // Unknown CPU command.
          } break;
        }
      } break;
      default: {
        // Invalid state
      } break;
    }
  }

  void apply(L1CacheContext& ctxt, L1CommandList& cl, MOESIL1LineState* line,
             const L2CmdRspMsg* msg) const {
    const State state = line->state();
    switch (state) {
      case State::IS: {
        // Update state
        issue_update_state(cl, line, msg->is() ? State::S : State::E);
          
        // Update transaction table; wake all blocked Message Queues
        // and delete context.
        cl.transaction_end();
        // Advance to next and consume
        cl.next_and_do_consume(true);
      } break;
      case State::IE: {
        // Update state
        issue_update_state(cl, line, State::E);
        // Update transaction table; wake all blocked Message Queues
        // and delete context.
        cl.transaction_end();
        // Advance to next and consume
        cl.next_and_do_consume(true);
      } break;
      case State::SE: {
        // Line is installed in 'E' state.
        issue_update_state(cl, line, State::E);
        // Update transaction table; wake all blocked Message Queues
        // and delete context.
        cl.transaction_end();
        // Advance to next and consume
        cl.next_and_do_consume(true);
      } break;
      case State::SI:
      case State::MI:
      case State::EI: {
        // Shared/Modifed/Exclusive -> Invalid
        //
        // Edge entered upon an eviction from L1 cache. As L1 is
        // write-through to L2, L2 maintains a recent up to date copy
        // of the line and this transaction therefore serves as
        // notification that the line has been evicted from L1. In the
        // case of SI, L1 may optionally silently evict the line
        // without L2 notification, in which the message is never
        // sent.

        // Transition back to Invalid state and evict line.
        issue_update_state(cl, line, State::I);
        // Remove line from cache as now invalid.
        const addr_t addr = ctxt.tstate()->addr();
        cl.push_back(cb::build_remove_line(addr));
        // Transaction ends
        cl.transaction_end();
        // Advance to next and consume
        cl.next_and_do_consume(true);
      } break;
      default: {
        // Invalid state
      } break;
    }
  }

  void issue_update_state(
      L1CommandList& cl, MOESIL1LineState* line, State state) const {
    struct UpdateStateAction : public CoherenceAction {
      UpdateStateAction(MOESIL1LineState* line, State state)
          : line_(line), state_(state)
      {}
      std::string to_string() const override {
        using cc::to_string;
        
        KVListRenderer r;
        r.add_field("action", "update state");
        r.add_field("current", to_string(line_->state()));
        r.add_field("next", to_string(state_));
        return r.to_string();
      }
      bool execute() override {
        line_->set_state(state_);
        return true;
      }
     private:
      MOESIL1LineState* line_ = nullptr;
      State state_;
    };
    CoherenceAction* action = new UpdateStateAction(line, state);
    cl.push_back(cb::from_action(action));
  }

};

} // namespace


namespace cc::moesi {

//
//
L1CacheAgentProtocol* build_l1_protocol(kernel::Kernel* k) {
  return new MOESIL1CacheProtocol(k);
}

} // namespace cc::moesi

