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
  IS_D,


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
    case State::IS_D: return "IS_D";
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

  //
  State state() const { return state_; }

  //
  void set_state(State state) { state_ = state; }

 private:
  // Current line state
  State state_ = State::I;
};

//
//
struct DtToL2Action : public CoherenceAction {
  DtToL2Action(L2CacheModel* l2c, const DtMsg* dt)
      : l2c_(l2c), dt_(dt)
  {}

  bool execute() override {
    // TBD: No data message queue on L2.
    return true;
  }

 private:
  L2CacheModel* l2c_ = nullptr;
  const DtMsg* dt_ = nullptr;
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
  void apply(CCContext& ctxt, CCCommandList& cl) const override {
    const MessageClass cls = ctxt.msg()->cls();
    switch (cls) {
      case MessageClass::AceCmd: {
        eval_msg(ctxt, cl, static_cast<const AceCmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::CohEnd: {
        eval_msg(ctxt, cl, static_cast<const CohCmdMsg*>(ctxt.msg()));
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

  void eval_msg(CCContext& ctxt, CCCommandList& cl, const AceCmdMsg* msg) const {
    switch (msg->opcode()) {
      case AceCmdOpcode::ReadShared: {
        const DirMapper* dm = ctxt.cc()->dm();
        
        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(ctxt.cc());
        issue_emit_to_noc(ctxt, cl, cohsrt, dm->lookup(msg->addr()));

        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(ctxt.cc());
        cohcmd->set_addr(msg->addr());
        issue_emit_to_noc(ctxt, cl, cohcmd, dm->lookup(msg->addr()));

        // ACE command advances to active state; install entry within
        // transaction table.
        cl.push_back(cb::from_opcode(CCOpcode::TableInstall));
        // Consume ACE command
        cl.push_back(cb::from_opcode(CCOpcode::MsgConsume));
        // Advance to next
        cl.push_back(cb::from_opcode(CCOpcode::WaitNextEpochOrWait));
        // Update state
        issue_update_state(ctxt, cl, State::IS_D);
      } break;
      default: {
      } break;
    }
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl, const CohCmdMsg* msg) const {
    AceCmdRspMsg* rmsg = new AceCmdRspMsg;
    rmsg->set_t(msg->t());
    issue_msg(cl, ctxt.cc()->cc_l2__rsp_q(), rmsg);

    // Transaction is now complete; delete entry from transaction table.
    cl.push_back(cb::from_opcode(CCOpcode::TableUninstall));
    // Consume message
    cl.push_back(cb::from_opcode(CCOpcode::MsgConsume));
    // Advance to next
    cl.push_back(cb::from_opcode(CCOpcode::WaitNextEpochOrWait));
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl, const DtMsg* msg) const {
    // Consume message
    cl.push_back(cb::from_opcode(CCOpcode::MsgConsume));
    // Advance to next
    cl.push_back(cb::from_opcode(CCOpcode::WaitNextEpochOrWait));
  }

  void issue_update_state(CCContext& ctxt, CCCommandList& cl, State state) const {
    struct UpdateStateAction : public CoherenceAction {
      UpdateStateAction(Line* line, State state)
          : line_(line), state_(state)
      {}
      std::string to_string() const override {
        using cc::to_string;

        std::stringstream ss;
        {
          KVListRenderer r(ss);
          r.add_field("action", "update state");
          r.add_field("current", to_string(line_->state()));
          r.add_field("next", to_string(state_));
        }
        return ss.str();
      }
      bool execute() override {
        line_->set_state(state_);
        return true;
      }
     private:
      Line* line_ = nullptr;
      State state_;
    };
    Line* line = static_cast<Line*>(ctxt.line());
    CoherenceAction* action = new UpdateStateAction(line, state);
    cl.push_back(cb::from_action(action));
  }
  
};

} // namespace

namespace cc::moesi {

//
//
CCProtocol* build_cc_protocol(kernel::Kernel* k) {
  return new MOESICCProtocol(k);
}

} // namespace cc::moesi
