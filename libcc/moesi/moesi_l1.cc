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

enum class MOESIL1State {
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
const char* to_string(MOESIL1State state) {
  switch (state) {
    case MOESIL1State::I: return "I";
    case MOESIL1State::I_S: return "I_S";
    case MOESIL1State::S: return "S";
    case MOESIL1State::I_E: return "I_E";
    case MOESIL1State::E: return "E";
    case MOESIL1State::E_M: return "E_M";
    case MOESIL1State::M: return "M";
    case MOESIL1State::M_I: return "M_I";
    default: return "Invalid";
  }
};

//
//
bool is_stable(MOESIL1State state) {
  switch (state) {
    case MOESIL1State::I:
    case MOESIL1State::S:
    case MOESIL1State::E:
    case MOESIL1State::M:
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
  MOESIL1State state() const { return state_; }
  void set_state(MOESIL1State state) { state_ = state; }

  // Stable state status.
  bool is_stable() const {
    return true;
  }

 private:
  MOESIL1State state_ = MOESIL1State::I;
};

//
//
struct UpdateStateAction : public CoherenceAction {
  UpdateStateAction(MOESIL1LineState* line, MOESIL1State state)
      : line_(line), state_(state)
  {}
  bool execute() override {
    line_->set_state(state_);
    return true;
  }
 private:
  MOESIL1LineState* line_ = nullptr;
  MOESIL1State state_;
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
        const L1CmdMsg* msg = static_cast<const L1CmdMsg*>(c.msg());
        apply(c, line, msg);
      } break;
      case MessageClass::L2CmdRsp: {
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
      case MOESIL1State::I: {
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
        issue_update_state(c, line, MOESIL1State::I_S);
        // Update context
        c.set_commits(true);
        c.set_dequeue(true);
        c.set_wait(L1Wait::NextEpochIfHasRequestOrWait);
      } break;
      default: {
      } break;
    }
  }

  void emit_to_l2(const Message* msg) {
  }

  void issue_update_state(
      L1CacheContext& c, MOESIL1LineState* line, MOESIL1State state) const {
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

