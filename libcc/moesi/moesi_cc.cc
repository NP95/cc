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
#include "l2cache.h"
#include "noc.h"
#include "mem.h"
#include "utility.h"

namespace {

using namespace cc;

//
//
enum class State {

  // Invalid State
  // 
  I,


  // State Transition: I -> S; Awaiting data from owner.
  //
  // Next states:
  //
  //    S:   On receipt of data (non-owning).
  //
  //    E:   On receipt of data (owning).
  //
  IS,

  
  IE,


  // Shared State
  //
  S,


  // Exclusive State
  //
  // Next state:
  //
  //    M:   On commit of CPU store instruction.
  //
  E,


  // Owned State
  //
  O,


  // Modified State
  //
  M
};


//
//
const char* to_string(State state) {
  switch (state) {
    case State::I: return "I";
    case State::IS: return "IS";
    case State::IE: return "IE";
    case State::S: return "S";
    case State::E: return "E";
    case State::O: return "O";
    case State::M: return "M";
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
    case State::O:
    case State::M:
      return true;
    default:
      return false;
  }
}


//
//
class Line : public CCLineState {
 public:
  Line() = default;
  ~Line() {}

  //
  State state() const { return state_; }
  bool awaiting_cmdmsg_rsp() const { return awaiting_cmdmsg_rsp_; }

  //
  void set_state(State state) { state_ = state; }
  void set_awaiting_cmdmsg_rsp(bool b) { awaiting_cmdmsg_rsp_ = b; }

 private:
  //
  bool awaiting_cmdmsg_rsp_ = false;
  // Current line state
  State state_ = State::I;
};

class SnpLine : public CCSnpLineState {
 public:
  SnpLine() = default;

  // Set fields; Set Aciton handler.
};

//
//
enum class LineUpdate {
  SetAwaitingCmdMsgRsp,
  ClrAwaitingCmdMsgRsp,
  State,
  Invalid
};

const char* to_string(LineUpdate update) {
  switch (update) {
    case LineUpdate::SetAwaitingCmdMsgRsp:
      return "SetAwaitingCmdMsgRsp";
    case LineUpdate::ClrAwaitingCmdMsgRsp:
      return "ClrAwaitingCmdMsgRsp";
    case LineUpdate::State:
      return "State";
    case LineUpdate::Invalid:
    default:
      return "Invalid";
  }
}

struct LineUpdateAction : public CoherenceAction {
  LineUpdateAction(Line* line, LineUpdate update)
      : line_(line), update_(update)
  {}
  std::string to_string() const override {
    using cc::to_string;
    KVListRenderer r;
    r.add_field("update", to_string(update_));
    switch (update_) {
      case LineUpdate::State: {
        r.add_field("action", "update state");
        r.add_field("current", to_string(line_->state()));
        r.add_field("next", to_string(state_));
      } break;
      default: {
      } break;
    }
    return r.to_string();
  }
  void set_state(State state) { state_ = state; }
  bool execute() override {
    switch (update_) {
      case LineUpdate::SetAwaitingCmdMsgRsp: {
        line_->set_awaiting_cmdmsg_rsp(true);
      } break;
      case LineUpdate::ClrAwaitingCmdMsgRsp: {
        line_->set_awaiting_cmdmsg_rsp(false);
      } break;
      case LineUpdate::State: {
        line_->set_state(state_);
      } break;
      default: {
      } break;
    }
    return true;
  }
 private:
  State state_;
  Line* line_ = nullptr;
  LineUpdate update_ = LineUpdate::Invalid;
};

//
//
class MOESICCProtocol : public CCProtocol {
  using cb = CCCommandBuilder;
  
 public:
  MOESICCProtocol(kernel::Kernel* k) : CCProtocol(k, "moesicc") {}

  //
  //
  CCLineState* construct_line() const override {
    return new Line;
  }

  //
  //
  CCSnpLineState* construct_snp_line() const override {
    return new SnpLine;
  }

  //
  //
  void apply(CCContext& ctxt, CCCommandList& cl) const override {
    const MessageClass cls = ctxt.msg()->cls();
    switch (cls) {
      case MessageClass::AceCmd: {
        eval_msg(ctxt, cl, static_cast<const AceCmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::CohEnd: {
        eval_msg(ctxt, cl, static_cast<const CohCmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::CohCmdRsp: {
        eval_msg(ctxt, cl, static_cast<const CohCmdRspMsg*>(ctxt.msg()));
      } break;
      case MessageClass::Dt: {
        eval_msg(ctxt, cl, static_cast<const DtMsg*>(ctxt.msg()));
      } break;
      default: {
        LogMessage msg("Invalid message class received: ");
        msg.append(cc::to_string(cls));
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  //
  //
  void apply(CCSnpContext& ctxt, CCSnpCommandList& cl) const override {
    const MessageClass cls = ctxt.msg()->cls();
    switch (cls) {
      case MessageClass::CohSnp: {
        eval_msg(ctxt, cl, static_cast<const CohSnpMsg*>(ctxt.msg()));
      } break;
      case MessageClass::AceSnoopRsp: {
        eval_msg(ctxt, cl, static_cast<const AceSnpMsg*>(ctxt.msg()));
      } break;
      default: {
        LogMessage msg("Invalid message class received: ");
        msg.append(cc::to_string(cls));
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl, const CohCmdRspMsg* msg) const {
    // Clear waiting bit.
    issue_field_update(ctxt, cl, LineUpdate::ClrAwaitingCmdMsgRsp);
    cl.push_back(cb::from_opcode(CCOpcode::MsgConsume));
    // Advance to next
    cl.push_back(cb::from_opcode(CCOpcode::WaitNextEpochOrWait));
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl, const AceCmdMsg* msg) const {
    switch (msg->opcode()) {
      case AceCmdOpcode::ReadShared: {
        const DirMapper* dm = ctxt.cc()->dm();
        
        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(ctxt.cc());
        cohsrt->set_addr(msg->addr());
        issue_emit_to_noc(ctxt, cl, cohsrt, dm->lookup(msg->addr()));

        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(ctxt.cc());
        cohcmd->set_addr(msg->addr());
        issue_emit_to_noc(ctxt, cl, cohcmd, dm->lookup(msg->addr()));
        issue_field_update(ctxt, cl, LineUpdate::SetAwaitingCmdMsgRsp);

        // ACE command advances to active state; install entry within
        // transaction table.
        cl.push_back(cb::from_opcode(CCOpcode::StartTransaction));
        // Consume ACE command
        cl.push_back(cb::from_opcode(CCOpcode::MsgConsume));
        // Advance to next
        cl.push_back(cb::from_opcode(CCOpcode::WaitNextEpochOrWait));
        // Update state
        issue_update_state(ctxt, cl, State::IS);
      } break;
      case AceCmdOpcode::ReadUnique: {
        const DirMapper* dm = ctxt.cc()->dm();
        
        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(ctxt.cc());
        cohsrt->set_addr(msg->addr());
        issue_emit_to_noc(ctxt, cl, cohsrt, dm->lookup(msg->addr()));

        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(ctxt.cc());
        cohcmd->set_addr(msg->addr());
        issue_emit_to_noc(ctxt, cl, cohcmd, dm->lookup(msg->addr()));
        issue_field_update(ctxt, cl, LineUpdate::SetAwaitingCmdMsgRsp);

        // ACE command advances to active state; install entry within
        // transaction table.
        cl.push_back(cb::from_opcode(CCOpcode::StartTransaction));
        // Consume ACE command
        cl.push_back(cb::from_opcode(CCOpcode::MsgConsume));
        // Advance to next
        cl.push_back(cb::from_opcode(CCOpcode::WaitNextEpochOrWait));
        // Update state
        issue_update_state(ctxt, cl, State::IE);
      } break;
      default: {
        std::string name = "Unable to handle ACE command: ";
        name += to_string(msg->opcode());
        issue_invalid_state_transition(cl, name);
      } break;
    }
  }

  void eval_msg(CCSnpContext& ctxt, CCSnpCommandList& cl, const AceSnpMsg* msg) const {
    
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl, const CohCmdMsg* msg) const {
    AceCmdRspMsg* rmsg = new AceCmdRspMsg;
    rmsg->set_t(msg->t());
    issue_msg(cl, ctxt.cc()->cc_l2__rsp_q(), rmsg);

    // Transaction is now complete; delete entry from transaction table.
    cl.push_back(cb::from_opcode(CCOpcode::EndTransaction));
    // Consume message
    cl.push_back(cb::from_opcode(CCOpcode::MsgConsume));
    // Advance to next
    cl.push_back(cb::from_opcode(CCOpcode::WaitNextEpochOrWait));
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl, const DtMsg* msg) const {
    // Issue Dt response to LLC
    DtRspMsg* rsp = new DtRspMsg;
    rsp->set_t(msg->t());
    issue_emit_to_noc(ctxt, cl, rsp, msg->origin());
    // Consume message
    cl.push_back(cb::from_opcode(CCOpcode::MsgConsume));
    // Advance to next
    cl.push_back(cb::from_opcode(CCOpcode::WaitNextEpochOrWait));
  }

  void eval_msg(CCSnpContext& ctxt, CCSnpCommandList& cl, const CohSnpMsg* msg) const {
    using snpcb = CCSnpCommandBuilder;

    // Forward snoop request to L2.
    AceSnpMsg* acesnp = new AceSnpMsg;
    acesnp->set_t(msg->t());
    acesnp->set_opcode(msg->opcode());
    acesnp->set_addr(msg->addr());
    issue_msg(cl, ctxt.cc()->cc_l2__cmd_q(), acesnp);
    // Consume message
    cl.push_back(snpcb::from_opcode(CCSnpOpcode::ConsumeMsg));
    // Advance to next
    cl.push_back(snpcb::from_opcode(CCSnpOpcode::NextEpoch));
  }

  void issue_field_update(
      CCContext& ctxt,  CCCommandList& cl, LineUpdate update) const {
    Line* line = static_cast<Line*>(ctxt.line());
    LineUpdateAction* action = new LineUpdateAction(line, update);
    cl.push_back(cb::from_action(action));
  }

  void issue_update_state(CCContext& ctxt, CCCommandList& cl, State state) const {
    Line* line = static_cast<Line*>(ctxt.line());
    LineUpdateAction* action = new LineUpdateAction(line, LineUpdate::State);
    action->set_state(state);
    cl.push_back(cb::from_action(action));
  }
};

} // namespace

namespace cc::moesi {

//
//
CCProtocol* build_cc_protocol(kernel::Kernel* k) { return new MOESICCProtocol(k); }

} // namespace cc::moesi
