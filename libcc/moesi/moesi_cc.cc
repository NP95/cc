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
class LineState : public CCLineState {
 public:
  LineState() = default;

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
struct UpdateStateAction : public CoherenceAction {
  UpdateStateAction(LineState* line, State state)
      : line_(line), state_(state)
  {}
  bool execute() override {
    line_->set_state(state_);
    return true;
  }
 private:
  LineState* line_ = nullptr;
  State state_;
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
 public:
  MOESICCProtocol() = default;

  //
  //
  void install(CCContext& c) const override {
    LineState* line = new LineState;
    c.set_line(line);
    c.set_owns_line(true);
    apply(c);
  }

  //
  //
  void apply(CCContext& c) const override {
    switch (c.msg()->cls()) {
      case MessageClass::AceCmd: {
        eval_msg(c, static_cast<const AceCmdMsg*>(c.msg()));
      } break;
      case MessageClass::CohEnd: {
        eval_msg(c, static_cast<const CohCmdMsg*>(c.msg()));
      } break;
      default: {
      } break;
    }
  }

  void eval_msg(CCContext& c, const AceCmdMsg* msg) const {
    switch (msg->opcode()) {
      case AceCmdOpcode::ReadShared: {
        const DirMapper* dm = c.cc()->dm();
        
        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(c.cc());
        issue_emit_to_noc(c, cohsrt, dm->lookup(msg->addr()));

        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(c.cc());
        cohcmd->set_addr(msg->addr());
        issue_emit_to_noc(c, cohcmd, dm->lookup(msg->addr()));

        // Message is inserted into TT
        c.set_dequeue(true);
        c.set_commits(true);
        c.set_ttadd(true);
        c.set_wait(CCWait::NextEpochIfHasRequestOrWait);
      } break;
      default: {
      } break;
    }
  }

  void eval_msg(CCContext& c, const CohCmdMsg* msg) const {
    AceCmdRspMsg* rmsg = new AceCmdRspMsg;
    rmsg->set_t(msg->t());

    L2CacheModel* l2cache = c.cc()->l2c();
    issue_msg(c, l2cache->cc_l2__rsp_q(), rmsg);

    c.set_dequeue(true);
    c.set_commits(true);
    c.set_ttdel(true);
    c.set_wait(CCWait::NextEpochIfHasRequestOrWait);
  }
  
};

} // namespace

namespace cc::moesi {

//
//
CCProtocol* build_cc_protocol() { return new MOESICCProtocol{}; }

} // namespace cc::moesi
