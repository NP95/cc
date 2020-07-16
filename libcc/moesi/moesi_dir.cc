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
#include "dir.h"
#include "llc.h"
#include "noc.h"

namespace cc::moesi {

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
class MOESIDirProtocol : public DirProtocol {

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
  MOESIDirProtocol() {}

  //
  //
  DirLineState* construct_line() const override {
    MOESIDirLineState* line = new MOESIDirLineState;
    line->set_state(MOESIDirState::I);
    return line;
  }

  //
  //
  std::pair<bool, DirActionList> apply(
      const DirCoherenceContext& context) const override{
    bool commits = true;
    DirActionList al;
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

  bool apply(DirActionList& al, const DirCoherenceContext& context,
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

        DirModel* d = dir();
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

  bool apply(DirActionList& al, const DirCoherenceContext& context,
             const LLCCmdRspMsg* msg) const {
    bool commits = true;
    MOESIDirLineState* line = static_cast<MOESIDirLineState*>(context.line());
    switch (line->state()) {
      case MOESIDirState::I_E: {
        DirCmdRspMsg* dirrsp = new DirCmdRspMsg;
        dirrsp->set_t(msg->t());

        DirModel* d = dir();
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

DirProtocol* build_dir_protocol() {
  return new MOESIDirProtocol{};
}

} // namespace cc::moesi
