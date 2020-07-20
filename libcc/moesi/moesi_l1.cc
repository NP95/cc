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

namespace {

using namespace cc;

enum class State {
  // Invalid
  I,
  IS,
  // Shared
  S,
  IE,
  SE,
  // Exclusive
  E,
  EM,
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
    case State::S: return "S";
    case State::IE: return "IE";
    case State::E: return "E";
    case State::EM: return "EM";
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
  bool is_stable() const {
    return true;
  }

 private:
  State state_ = State::I;
};

//
//
struct UpdateStateAction : public CoherenceAction {
  UpdateStateAction(MOESIL1LineState* line, State state)
      : line_(line), state_(state)
  {}
  bool execute() override {
    line_->set_state(state_);
    return true;
  }
 private:
  MOESIL1LineState* line_ = nullptr;
  State state_;
};

//
//
class MOESIL1CacheProtocol : public L1CacheModelProtocol {
  
 public:
  MOESIL1CacheProtocol() = default;

  //
  //
  void install(L1CacheContext& c, L1CacheOutcome& o) const override {
    MOESIL1LineState* line = new MOESIL1LineState();
    // Leak on line if operation is not committed.
    c.set_line(line);
    o.set_line(line);
    o.set_owns_line(true);
    apply(c, o);
  }

  //
  //
  void apply(L1CacheContext& c, L1CacheOutcome& o) const override {
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(c.line());
    switch(c.msg()->cls()) {
      case MessageClass::L1Cmd: {
        // CPU -> L1 command:
        apply(c, o, line, static_cast<const L1CmdMsg*>(c.msg()));
      } break;
      case MessageClass::L2CmdRsp: {
        apply(c, o, line, static_cast<const L2CmdRspMsg*>(c.msg()));
      } break;
      default: {
        // Unknown message class; error
      } break;
    }
  }

  //
  //
  void evict(L1CacheContext& c, L1CacheOutcome& o) const override {
    // TODO
  }

 private:

  void apply(L1CacheContext& c, L1CacheOutcome& o, MOESIL1LineState* line,
             const L1CmdMsg* msg) const {
    switch (line->state()) {
      case State::I: {
        // Line is not in the cache; issue the appropriate request to
        // L2 to initiate the fill operation.

        // Emit request to L2.
        L2CmdMsg* l2cmdmsg = new L2CmdMsg();
        l2cmdmsg->set_t(msg->t());
        l2cmdmsg->set_addr(msg->addr());
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetS);
          } break;
          case L1CacheOpcode::CpuStore: {
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetE);
          } break;
        }
        l2cmdmsg->set_l1cache(c.l1cache());
        issue_msg(o.actions(), c.l1cache()->l1_l2__cmd_q(), l2cmdmsg);
        // Update state
        issue_update_state(o, line, State::IS);
        // Update context
        // Message is stalled on lookup transaction.
        o.set_stalled(true);
        o.set_commits(true);
        // Install new entry in transaction table as the transaction
        // has now started and commands are inflight. The transaction
        // itself is not complete at this point.
        o.set_ttadd(true);
        o.set_wait(L1Wait::NextEpochIfHasRequestOrWait);
      } break;
      case State::S: {
        // Line is present in the cache. 
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            // LD to line in S-state can complete immediately. Forward
            // the response to the CPU.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            issue_msg(o.actions(), c.l1cache()->l1_cpu__rsp_q(), rsp);
            // Update context.
            o.set_commits(true);
            o.set_dequeue(true);
            o.set_wait(L1Wait::NextEpochIfHasRequestOrWait);
          } break;
          case L1CacheOpcode::CpuStore: {
          } break;
        }
      } break;
      default: {
        // Invalid state
      } break;
    }
  }

  void apply(L1CacheContext& c, L1CacheOutcome& o, MOESIL1LineState* line,
             const L2CmdRspMsg* msg) const {
    switch (line->state()) {
      case State::IS: {
        // Response has arrived back at L1 from L2. Forward response
        // to CPU.
        // No message to CPU; simply unblock message queue.
        // L1CmdRspMsg* rsp = new L1CmdRspMsg;
        // rsp->set_t(msg->t());
        // issue_msg(o.actions(), c.l1cache()->l1_cpu__rsp_q(), rsp);
        // Update state
        issue_update_state(o, line, State::S);
        // Update context
        o.set_commits(true);
        o.set_dequeue(true);
        o.set_ttdel(true);
        o.set_wait(L1Wait::NextEpochIfHasRequestOrWait);
      } break;
      default: {
        // Invalid state
      } break;
    }
  }

  void issue_update_state(
      L1CacheOutcome& o, MOESIL1LineState* line, State state) const {
    o.actions().push_back(new UpdateStateAction(line, state));
  }

};

} // namespace


namespace cc::moesi {

//
//
L1CacheModelProtocol* build_l1_protocol() { return new MOESIL1CacheProtocol{}; }

} // namespace cc::moesi

