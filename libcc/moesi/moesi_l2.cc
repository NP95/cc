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

namespace cc::moesi {

//
//
enum class MOESIL2State {
  I,
  I_S,
  S,
  E,
  M,
  O
};

//
//
const char* to_string(MOESIL2State state) {
  switch (state) {
    case MOESIL2State::I: return "I";
    case MOESIL2State::I_S: return "I_S";
    case MOESIL2State::S: return "S";
    case MOESIL2State::E: return "E";
    case MOESIL2State::M: return "M";
    case MOESIL2State::O: return "O";
    default: return "Invalid";
  }
}

//
//
bool is_stable(MOESIL2State state) {
  switch (state) {
    case MOESIL2State::I:
    case MOESIL2State::S:
    case MOESIL2State::E:
    case MOESIL2State::M:
    case MOESIL2State::O:
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
  MOESIL2State state() const { return state_; }
  void set_state(MOESIL2State state) { state_ = state; }

  // Stable state status.
  bool is_stable() const {
    return true;
  }

 private:
  MOESIL2State state_ = MOESIL2State::I;
};

class MOESIL2CacheProtocol : public L2CacheModelProtocol {
  struct UpdateStateAction : public CoherenceAction {
    UpdateStateAction(MOESIL2LineState* line, MOESIL2State state)
        : line_(line), state_(state)
    {}
    bool execute() override {
      line_->set_state(state_);
      return true;
    }
   private:
    MOESIL2LineState* line_ = nullptr;
    MOESIL2State state_;
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
  MOESIL2CacheProtocol() {}


  //
  L2LineState* construct_line() const override {
    MOESIL2LineState* l = new MOESIL2LineState;
    l->set_state(MOESIL2State::I);
    return l;
  }

  std::pair<bool, L2CoherenceActionList> apply(
      const L2CoherenceContext& context) const override {
    bool commits = true;
    L2CoherenceActionList al;
    const Message* msg = context.msg();
    switch(msg->cls()) {
      case MessageClass::L2Cmd: {
        const L2CmdMsg* l2cmdmsg = static_cast<const L2CmdMsg*>(msg);
        commits = apply(al, context, l2cmdmsg);
      } break;
      case MessageClass::AceCmdRspR: {
        const AceCmdRspRMsg* acersp = static_cast<const AceCmdRspRMsg*>(msg);
        commits = apply(al, context, acersp);
      } break;
      case MessageClass::AceCmdRspB: {
        const AceCmdRspBMsg* acersp = static_cast<const AceCmdRspBMsg*>(msg);
        commits = apply(al, context, acersp);
      } break;
      default: {
        al.push_back(new ProtocolViolation("Invalid message class"));
      } break;
    }
    return std::make_pair(commits, al);
  }
 private:

  bool apply(L2CoherenceActionList& al, const L2CoherenceContext& context,
             const L2CmdMsg* msg) const {
    bool commits = true;
    MOESIL2LineState* line = static_cast<MOESIL2LineState*>(context.line());
    switch (line->state()) {
      case MOESIL2State::I: {
        AceCmdMsg* acecmd = new AceCmdMsg();
        switch (msg->opcode()) {
          case L2CmdOpcode::L1GetS: {
            acecmd->set_opcode(AceCmdOpcode::ReadShared);
          } break;
          case L2CmdOpcode::L1GetE: {
            acecmd->set_opcode(AceCmdOpcode::ReadUnique);
          } break;
        }
        acecmd->set_addr(msg->addr());
        acecmd->set_t(msg->t());
        al.push_back(new EmitMessageAction(l2cache()->l2_cc__cmd_q(), acecmd));

        // Update state
        al.push_back(new UpdateStateAction(line, MOESIL2State::I_S));
      } break;
      case MOESIL2State::S: {
        switch (msg->opcode()) {
          case L2CmdOpcode::L1GetS: {
            commits = true;
            L2CmdRspMsg* l2rsp = new L2CmdRspMsg();
            l2rsp->set_t(msg->t());
            l2rsp->set_opcode(L2RspOpcode::L1InstallS);
            MessageQueue* l2_l1__rsp_q = msg->l1cache()->l2_l1__rsp_q();
            al.push_back(new EmitMessageAction(l2_l1__rsp_q, l2rsp));
          } break;
          case L2CmdOpcode::L1GetE: {
            AceCmdMsg* acecmd = new AceCmdMsg();
            acecmd->set_t(msg->t());
            acecmd->set_opcode(AceCmdOpcode::ReadUnique);
            acecmd->set_addr(msg->addr());
            al.push_back(new EmitMessageAction(l2cache()->l2_cc__cmd_q(), acecmd));
            // Update state
            al.push_back(new UpdateStateAction(line, MOESIL2State::E));
          } break;
        }
      } break;
      case MOESIL2State::M: {
      } break;
      case MOESIL2State::E: {
      } break;
      case MOESIL2State::O: {
      } break;
      default: {
        al.push_back(new ProtocolViolation("Invalid line state"));
      } break;
    }
    return commits;
  }

  bool apply(L2CoherenceActionList& al, const L2CoherenceContext& context,
             const AceCmdRspRMsg* msg) const {
    bool commits = true;
    MOESIL2LineState* line = static_cast<MOESIL2LineState*>(context.line());

    // Emit response to L1
    L2CmdRspMsg* rsp = new L2CmdRspMsg;
    rsp->set_t(msg->t());
    const L2RspOpcode opcode =
        msg->is_shared() ? L2RspOpcode::L1InstallS : L2RspOpcode::L1InstallE;
    rsp->set_opcode(opcode);

    MessageQueue* l2_l1__rsp_q = l2cache()->l2_l1__rsp_q(0);
    al.push_back(new EmitMessageAction(l2_l1__rsp_q, rsp));

    // Compute final line state based upon state in the R-channel.
    const MOESIL2State state =
        (msg->pass_dirty() ? MOESIL2State::O : (
            msg->is_shared() ? MOESIL2State::S : MOESIL2State::E));
    al.push_back(new UpdateStateAction(line, state));
    
    return commits;
  }

  bool apply(L2CoherenceActionList& al, const L2CoherenceContext& context,
             const AceCmdRspBMsg* msg) const {
    bool commits = true;
    return commits;
  }

};

L2CacheModelProtocol* build_l2_protocol() {
  return new MOESIL2CacheProtocol{};
}

} // namespace cc::moesi;
