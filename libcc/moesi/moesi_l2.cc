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
#include "l1cache.h"
#include "l2cache.h"
#include "amba.h"

namespace {

using namespace cc;

//
//
enum class State {
  I,
  I_S,
  S,
  E,
  M,
  O
};

//
//
const char* to_string(State state) {
  switch (state) {
    case State::I: return "I";
    case State::I_S: return "I_S";
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
    case State::O:
      return true;
    default:
      return false;
  }
}

//
//
class MOESIL2LineState : public L2LineState {
 public:
  MOESIL2LineState() {}

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
  UpdateStateAction(MOESIL2LineState* line, State state)
      : line_(line), state_(state)
  {}
  bool execute() override {
    line_->set_state(state_);
    return true;
  }
 private:
  MOESIL2LineState* line_ = nullptr;
  State state_;
};

//
//
struct EmitMessageAction : public CoherenceAction {
  EmitMessageAction(MessageQueue* mq, const Message* msg)
      : mq_(mq), msg_(msg)
  {}
  bool execute() override {
    return mq_->issue(msg_);
  }
 private:
  MessageQueue* mq_ = nullptr;
  const Message* msg_ = nullptr;
};

//
//
class MOESIL2CacheProtocol : public L2CacheModelProtocol {
 public:
  MOESIL2CacheProtocol() = default;

  //
  //
  void install(L2CacheContext& c) const {
    MOESIL2LineState* line = new MOESIL2LineState;
    line->set_state(State::S);
    c.set_line(line);
    c.set_owns_line(true);
    apply(c);
  }

  //
  //
  void apply(L2CacheContext& c) const {
    MOESIL2LineState* line = static_cast<MOESIL2LineState*>(c.line());
    switch (c.msg()->cls()) {
      case MessageClass::L2Cmd: {
        apply(c, line, static_cast<const L2CmdMsg*>(c.msg()));
      } break;
      default: {
        // Invalid message class.
      } break;
    }
  }

  //
  //
  void evict(L2CacheContext& c) const {
    // TODO
  }


 private:
  void apply(L2CacheContext& c, const MOESIL2LineState* line, const L2CmdMsg* cmd) const {
    switch (line->state()) {
      case State::I: {
      } break;
      case State::S: {
        switch (cmd->opcode()) {
          case L2CmdOpcode::L1GetS: {
            // L1 -> L2 GetS command when presently in the S state; command
            // succeeds; forward data to requesting L1 in the S state.
            {
              L2CmdRspMsg* msg = new L2CmdRspMsg;
              msg->set_t(cmd->t());
              issue_msg(c.actions(), cmd->l1cache()->l2_l1__rsp_q(), msg);
            }
            c.set_commits(true);
            c.set_dequeue(true);
            // Transaction completes; delete transaction table entry.
            // c.set_ttdel(true);
            c.set_wait(L2Wait::NextEpochIfHasRequestOrWait);
          } break;
          case L2CmdOpcode::L1GetE: {
          } break;
          default: {
          } break;
        }
      } break;
      default: {
      } break;
    }
  }

};

} // namespace

namespace cc::moesi {

L2CacheModelProtocol* build_l2_protocol() { return new MOESIL2CacheProtocol{}; }

} // namespace cc::moesi;
