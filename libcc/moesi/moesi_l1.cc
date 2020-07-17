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
  I_S,
  // Shared
  S,
  I_E,
  S_E,
  // Exclusive
  E,
  E_M,
  // Modified
  M,
  M_I
};

//
//
const char* to_string(State state) {
  switch (state) {
    case State::I: return "I";
    case State::I_S: return "I_S";
    case State::S: return "S";
    case State::I_E: return "I_E";
    case State::E: return "E";
    case State::E_M: return "E_M";
    case State::M: return "M";
    case State::M_I: return "M_I";
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
  void install(L1CacheContext& c) const override {
    MOESIL1LineState* line = new MOESIL1LineState();
    // Leak on line if operation is not committed.
    c.set_line(line);
    c.set_owns_line(true);
    apply(c);
  }

  //
  //
  void apply(L1CacheContext& c) const override {
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(c.line());
    switch(c.msg()->cls()) {
      case MessageClass::L1Cmd: {
        // CPU -> L1 command:
        apply(c, line, static_cast<const L1CmdMsg*>(c.msg()));
      } break;
      case MessageClass::L2CmdRsp: {
        // L2 -> L1 command response:
        apply(c, line, static_cast<const L1CmdRspMsg*>(c.msg()));
      } break;
      default: {
        // Unknown message class; error
      } break;
    }
  }

  //
  //
  void evict(L1CacheContext& c) const override {
    // TODO
  }

 private:

  void apply(L1CacheContext& c, MOESIL1LineState* line, const L1CmdMsg* msg) const {
    switch (line->state()) {
      case State::I: {
        // Line is not in the cache; issue the appropriate request to
        // L2 to initiate the fill operation.
        {
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
          l2cmdmsg->set_l1cache(l1cache());
          issue_msg(c.actions(), l1cache()->l1_l2__cmd_q(), l2cmdmsg);
        }
        // Update state
        issue_update_state(c, line, State::I_S);
        // Update context
        // Wrong, should not commit!
        // c.set_stalled(true);
        c.set_commits(true);
        c.set_dequeue(true);
        // Install new entry in transaction table as the transaction
        // has now started and commands are inflight. The transaction
        // itself is not complete at this point.
        c.set_ttadd(true);
        c.set_wait(L1Wait::NextEpochIfHasRequestOrWait);
      } break;
      case State::S: {
        // Line is present in the cache. 
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            // LD to line in S-state can complete immediately. Forward
            // the response to the CPU.
            {
              L1CmdRspMsg* rsp = new L1CmdRspMsg;
              rsp->set_t(msg->t());
              issue_msg(c.actions(), c.l1cache()->l1_cpu__rsp_q(), rsp);
            }
            // Update context.
            c.set_commits(true);
            c.set_dequeue(true);
            c.set_wait(L1Wait::NextEpochIfHasRequestOrWait);
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

  void apply(L1CacheContext& c, MOESIL1LineState* line, const L1CmdRspMsg* msg) const {
    switch (line->state()) {
      case State::I_S: {
        // Response has arrived back at L1 from L2. Forward response
        // to CPU.
        {
          L1CmdRspMsg* rsp = new L1CmdRspMsg;
          rsp->set_t(msg->t());
          issue_msg(c.actions(), c.l1cache()->l1_cpu__rsp_q(), rsp);
        }
        // Update state
        issue_update_state(c, line, State::S);
        // Update context
        c.set_commits(true);
        c.set_dequeue(true);
        c.set_ttdel(true);
        c.set_wait(L1Wait::NextEpochIfHasRequestOrWait);
      } break;
      default: {
        // Invalid state
      } break;
    }
  }

  void issue_update_state(
      L1CacheContext& c, MOESIL1LineState* line, State state) const {
    std::vector<CoherenceAction*>& a = c.actions();
    a.push_back(new UpdateStateAction(line, state));
  }

};

} // namespace


namespace cc::moesi {

//
//
L1CacheModelProtocol* build_l1_protocol() { return new MOESIL1CacheProtocol{}; }

} // namespace cc::moesi

