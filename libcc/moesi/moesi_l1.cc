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
#include "l1cache.h"

namespace cc::moesi {

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
class MOESIL1CacheProtocol : public L1CacheModelProtocol {
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
  
 public:
  MOESIL1CacheProtocol() {}

  L1LineState* construct_line() const override {
    MOESIL1LineState* l1 = new MOESIL1LineState();
    return l1;
  }

  std::pair<bool, L1CoherenceActionList> apply(
      const L1CoherenceContext& context) const override {
    bool commits;
    L1CoherenceActionList al;

    const Message* msg = context.msg();
    switch(msg->cls()) {
      case MessageClass::L1Cmd: {
        const L1CmdMsg* l1cmdmsg = static_cast<const L1CmdMsg*>(msg);
        commits = apply(al, context, l1cmdmsg);
      } break;
      case MessageClass::L2CmdRsp: {
        const L2CmdRspMsg* l2rspmsg = static_cast<const L2CmdRspMsg*>(msg);
        commits = apply(al, context, l2rspmsg);
      } break;
      default: {
        // Unknown message class; error
        commits = false;
      } break;
    }
    return std::make_pair(commits, al);
  }

  bool apply(L1CoherenceActionList& al, const L1CoherenceContext& context,
             const L1CmdMsg* msg) const {
    bool commits = true;
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(context.line());
    switch (line->state()) {
      case MOESIL1State::I: {
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
        al.push_back(new EmitMessageAction(l1cache()->l1_l2__cmd_q(), l2cmdmsg));
        // Update state
        al.push_back(new UpdateStateAction(line, MOESIL1State::I_S));
      } break;
      case MOESIL1State::E: {
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            commits = true;
          } break;
          case L1CacheOpcode::CpuStore: {
            commits = true;
            al.push_back(new UpdateStateAction(line, MOESIL1State::M));
          } break;
        }
      } break;
      case MOESIL1State::M: {
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            commits = true;
          } break;
          case L1CacheOpcode::CpuStore: {
            commits = true;
          } break;
        }
      } break;
      case MOESIL1State::S: {
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            commits = true;

            L1CmdRspMsg* l1rspmsg = new L1CmdRspMsg();
            l1rspmsg->set_t(msg->t());
            al.push_back(new EmitMessageAction(l1cache()->l1_cpu__rsp_q(), l1rspmsg));
          } break;
          case L1CacheOpcode::CpuStore: {
            // Cannot store to a line in the Shared-state; the line
            // must first be promoted to the exclusive state.
            commits = false;
            L2CmdMsg* l2cmdmsg = new L2CmdMsg();
            l2cmdmsg->set_t(msg->t());
            l2cmdmsg->set_addr(msg->addr());
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetE);
            al.push_back(new EmitMessageAction(l1cache()->l1_l2__cmd_q(), l2cmdmsg));

            al.push_back(new UpdateStateAction(line, MOESIL1State::S_E));
          } break;
        }
      } break;
      default: {
        al.push_back(new ProtocolViolation("Invalid line state"));
      } break;
    }
    return commits;
  }

  bool apply(L1CoherenceActionList& al, const L1CoherenceContext& context,
             const L2CmdRspMsg* msg) const {
    bool commits = true;
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(context.line());
    switch (line->state()) {
      case MOESIL1State::I_S: {
        L1CmdRspMsg* l1rsp = new L1CmdRspMsg();
        l1rsp->set_t(msg->t());
        al.push_back(new EmitMessageAction(l1cache()->l1_cpu__rsp_q(), l1rsp));

        switch (msg->opcode()) {
          case L2RspOpcode::L1InstallS: {
            al.push_back(new UpdateStateAction(line, MOESIL1State::S));
          } break;
          case L2RspOpcode::L1InstallE: {
            // Requested a line in the Shared state; line arrives in
            // the exclusive state.
            al.push_back(new UpdateStateAction(line, MOESIL1State::E));
          } break;
          default: {
            // Unknown opcode.
          } break;
        }
      } break;
      default: {
        al.push_back(new ProtocolViolation("Invalid line state"));
      } break;
    }
    return commits;
  }
};

L1CacheModelProtocol* build_l1_protocol() {
  return new MOESIL1CacheProtocol{};
}

} // namespace cc::moesi
