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
#include "amba.h"
#include "ccntrl.h"
#include "dir.h"
#include "noc.h"

namespace cc::moesi {

//
//
enum class MOESICCState {
  I, I_S, S
};

//
//
const char* to_string(MOESICCState state) {
  switch (state) {
    case MOESICCState::I: return "I";
    case MOESICCState::I_S: return "I_S";
    case MOESICCState::S: return "S";
    default: return "Invalid";
  }
}

//
//
bool is_stable(MOESICCState state) {
  switch (state) {
    case MOESICCState::I:
    case MOESICCState::S:
      return true;
    default:
      return false;
  }
}

class MOESICCLineState : public CCLineState {
 public:
  MOESICCLineState() = default;

  //
  MOESICCState state() const { return state_; }

  void set_state(MOESICCState state) { state_ = state; }

 private:
  MOESICCState state_;
};

class MOESICCProtocol : public CCProtocol {
  struct UpdateStateAction : public CoherenceAction {
    UpdateStateAction(MOESICCLineState* line,
                      MOESICCState state)
        : line_(line), state_(state)
    {}
    bool execute() override {
      line_->set_state(state_);
      return true;
    }
   private:
    MOESICCLineState* line_ = nullptr;
    MOESICCState state_;
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
  MOESICCProtocol() {}

  CCLineState* construct_line() const override {
    CCLineState* line = new MOESICCLineState;
    return line;
  }

  std::pair<bool, CCActionList> apply(
      const CCContext& context) const override {
    bool commits = true;
    CCActionList al;
    const Message *msg = context.msg();
    switch (msg->cls()) {
      case MessageClass::AceCmd: {
        const AceCmdMsg* acecmd = static_cast<const AceCmdMsg*>(msg);
        commits = apply(al, context, acecmd);
      } break;
      case MessageClass::DirRsp: {
        const DirCmdRspMsg* dirrsp = static_cast<const DirCmdRspMsg*>(msg);
        commits = apply(al, context, dirrsp);
      } break;
      default: {
        al.push_back(new ProtocolViolation("Invalid message class"));
      } break;
    }
    return std::make_pair(commits, al);
  }

  bool apply(CCActionList& al, const CCContext& context,
             const AceCmdMsg* msg) const {
    bool commits = true;
    MOESICCLineState* line =
        static_cast<MOESICCLineState*>(context.line());
    switch (msg->opcode()) {
      case AceCmdOpcode::ReadShared: {
        // Pass command to associated directory.
        DirCmdMsg* dirmsg = new DirCmdMsg;
        dirmsg->set_opcode(msg->opcode());
        dirmsg->set_addr(msg->addr());
        dirmsg->set_t(msg->t());
        dirmsg->set_origin(cc());
        
        NocMsg* nocmsg = new NocMsg;
        nocmsg->set_t(msg->t());
        nocmsg->set_payload(dirmsg);
        nocmsg->set_origin(cc());
        DirMapper* dm = cc()->dm();
        nocmsg->set_dest(dm->lookup(msg->addr()));

        MessageQueue* cc_noc__msg_q = cc()->cc_noc__msg_q();
        al.push_back(new EmitMessageAction(cc_noc__msg_q, nocmsg));
        
        // Update state
        al.push_back(new UpdateStateAction(line, MOESICCState::I_S));
      } break;
      default: {
        al.push_back(new ProtocolViolation("Invalid opcode"));
      } break;
    }
    return commits;
  }

  bool apply(CCActionList& al, const CCContext& context,
             const DirCmdRspMsg* msg) const {
    bool commits = true;
    MOESICCLineState* line =
        static_cast<MOESICCLineState*>(context.line());

    AceCmdRspRMsg* acersp = new AceCmdRspRMsg;
    acersp->set_t(msg->t());
    acersp->set_pass_dirty(false);
    acersp->set_is_shared(false);

    // Emit message to L2
    MessageQueue* cc_l2__rsp_q = cc()->cc_l2__rsp_q();
    al.push_back(new EmitMessageAction(cc_l2__rsp_q, acersp));
    
    return commits;
  }  
};

CCProtocol* build_cc_protocol() {
  return new MOESICCProtocol{};
}

} // namespace cc::moesi
