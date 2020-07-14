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

//#include "moesi.h"

#include <string>

#include "l1cache.h"
#include "utility.h"
#include "msg.h"
#include "protocol.h"
#include "cpu.h"
#include "amba.h"
#include "ccntrl.h"
#include "noc.h"
#include "dir.h"
#include "llc.h"

namespace cc {

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

//
//
enum class MOESIDirState {
  I,
  I_S,
  S,
  I_E,
  M,
  E,
  O
};

//
//
const char* to_string(MOESIDirState state) {
  switch (state) {
    case MOESIDirState::I: return "I";
    case MOESIDirState::I_S: return "I_S";
    case MOESIDirState::S: return "S";
    case MOESIDirState::I_E: return "I_E";
    case MOESIDirState::E: return "E";
    case MOESIDirState::M: return "M";
    case MOESIDirState::O: return "O";
    default: return "Invalid";
  }
}

//
//
bool is_stable(MOESIDirState state) {
  switch (state) {
    case MOESIDirState::I:
    case MOESIDirState::S:
    case MOESIDirState::E:
    case MOESIDirState::M:
      return true;
    default:
      return false;
  }
}

//
//
struct ProtocolViolation : public CoherenceAction {
  ProtocolViolation(const std::string& desc)
      : desc_(desc)
  {}
  bool execute() override {
    throw std::runtime_error("Protocol violation!");
    return true;
  }
 private:
  std::string desc_;
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
    using cc::is_stable;
    return is_stable(state());
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
      case MessageClass::L2Rsp: {
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
    using cc::is_stable;
    return is_stable(state());
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

//
//
class MOESIDirLineState : public DirLineState {
 public:
  MOESIDirLineState() {}

  MOESIDirState state() const { return state_; }
  Agent* owner() const { return owner_; }
  bool is_stable() const { return true; }

  //
  void set_state(MOESIDirState state) { state_ = state; }
  void set_owner(Agent* owner) { owner_ = owner; }
  
 private:
  MOESIDirState state_;
  Agent* owner_;
};

//
//
class MOESIDirectoryProtocol : public DirectoryProtocol {

  //
  struct UpdateStateAction : public CoherenceAction {
    UpdateStateAction(MOESIDirLineState* line, MOESIDirState state)
        : line_(line), state_(state)
    {}
    bool execute() override {
      line_->set_state(state_);
      return true;
    }
   private:
    MOESIDirLineState* line_ = nullptr;
    MOESIDirState state_;
  };

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
  struct SetOwnerAction : public CoherenceAction {
    SetOwnerAction(MOESIDirLineState* line, Agent* owner)
        : line_(line), owner_(owner)
    {}
    bool execute() override {
      line_->set_owner(owner_);
      return true;
    }
   private:
    MOESIDirLineState* line_ = nullptr;
    Agent* owner_ = nullptr;
  };
  
 public:
  MOESIDirectoryProtocol() {}

  //
  //
  DirLineState* construct_line() const override {
    MOESIDirLineState* line = new MOESIDirLineState;
    line->set_state(MOESIDirState::I);
    return line;
  }

  //
  //
  std::pair<bool, DirectoryActionList> apply(
      const DirCoherenceContext& context) const override{
    bool commits = true;
    DirectoryActionList al;
    const Message* msg = context.msg();
    switch (msg->cls()) {
      case MessageClass::DirCmd: {
        const DirCmdMsg* dircmd = static_cast<const DirCmdMsg*>(msg);
        commits = apply(al, context, dircmd);
      } break;
      case MessageClass::LLCCmdRsp: {
        const LLCCmdRspMsg* llcrsp = static_cast<const LLCCmdRspMsg*>(msg);
        commits = apply(al, context, llcrsp);
      } break;
      default: {
        al.push_back(new ProtocolViolation("Invalid line state"));
      } break;
    }
    return std::make_pair(commits, al);
  }

  bool apply(DirectoryActionList& al, const DirCoherenceContext& context,
             const DirCmdMsg* msg) const {
    bool commits = true;
    MOESIDirLineState* line = static_cast<MOESIDirLineState*>(context.line());
    switch (line->state()) {
      case MOESIDirState::I: {
        // Line is not present in the directory
        LLCCmdMsg* llcmsg = new LLCCmdMsg;
        llcmsg->set_t(msg->t());
        llcmsg->set_opcode(LLCCmdOpcode::Fill);
        llcmsg->set_addr(msg->addr());

        DirectoryModel* d = dir();
        NocMsg* nocmsg = new NocMsg;
        nocmsg->set_payload(llcmsg);
        nocmsg->set_origin(d);
        nocmsg->set_dest(d->llc());

        MessageQueue* dir_noc__msg_q = d->dir_noc__msg_q();
        al.push_back(new EmitMessageAction(dir_noc__msg_q, nocmsg));

        // Set owner to requester:
        al.push_back(new SetOwnerAction(line, msg->origin()));
        
        // Update State:
        al.push_back(new UpdateStateAction(line, MOESIDirState::I_E));
      } break;
      default: {
      } break;
    }
    return commits;
  }

  bool apply(DirectoryActionList& al, const DirCoherenceContext& context,
             const LLCCmdRspMsg* msg) const {
    bool commits = true;
    MOESIDirLineState* line = static_cast<MOESIDirLineState*>(context.line());
    switch (line->state()) {
      case MOESIDirState::I_E: {
        DirCmdRspMsg* dirrsp = new DirCmdRspMsg;
        dirrsp->set_t(msg->t());

        DirectoryModel* d = dir();
        NocMsg* nocmsg = new NocMsg;
        nocmsg->set_payload(dirrsp);
        nocmsg->set_origin(d);
        nocmsg->set_dest(line->owner());

        MessageQueue* dir_noc__msg_q = d->dir_noc__msg_q();
        al.push_back(new EmitMessageAction(dir_noc__msg_q, nocmsg));

        // Update state:
        al.push_back(new UpdateStateAction(line, MOESIDirState::E));
      } break;
      default: {
      } break;
    }
    return commits;
  }
};


class MOESICacheControllerLineState : public CacheControllerLineState {
 public:
  MOESICacheControllerLineState() = default;

  //
  MOESICCState state() const { return state_; }

  void set_state(MOESICCState state) { state_ = state; }

 private:
  MOESICCState state_;
};

class MOESICacheControllerProtocol : public CacheControllerProtocol {
  struct UpdateStateAction : public CoherenceAction {
    UpdateStateAction(MOESICacheControllerLineState* line,
                      MOESICCState state)
        : line_(line), state_(state)
    {}
    bool execute() override {
      line_->set_state(state_);
      return true;
    }
   private:
    MOESICacheControllerLineState* line_ = nullptr;
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
  MOESICacheControllerProtocol() {}

  CacheControllerLineState* construct_line() const override {
    CacheControllerLineState* line = new MOESICacheControllerLineState;
    return line;
  }

  std::pair<bool, CacheControllerActionList> apply(
      const CacheControllerContext& context) const override {
    bool commits = true;
    CacheControllerActionList al;
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

  bool apply(CacheControllerActionList& al, const CacheControllerContext& context,
             const AceCmdMsg* msg) const {
    bool commits = true;
    MOESICacheControllerLineState* line =
        static_cast<MOESICacheControllerLineState*>(context.line());
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
        DirectoryMapper* dm = cc()->dm();
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

  bool apply(CacheControllerActionList& al, const CacheControllerContext& context,
             const DirCmdRspMsg* msg) const {
    bool commits = true;
    MOESICacheControllerLineState* line =
        static_cast<MOESICacheControllerLineState*>(context.line());

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

class MOESIProtocolBuilder : public ProtocolBuilder {
 public:
  // Create an instance of the L1 protocol
  L1CacheModelProtocol* create_l1() override {
    return new MOESIL1CacheProtocol{};
  }

  // Create an instance of the L2 protocol
  L2CacheModelProtocol* create_l2() override {
    return new MOESIL2CacheProtocol{};
  }

  // Create and instance of the Directory protocol
  DirectoryProtocol* create_dir() override {
    return new MOESIDirectoryProtocol{};
  }

  CacheControllerProtocol* create_cc() override {
    return new MOESICacheControllerProtocol{};
  }
};

CC_DECLARE_PROTOCOL_BUILDER("moesi", MOESIProtocolBuilder);

}  // namespace cc
