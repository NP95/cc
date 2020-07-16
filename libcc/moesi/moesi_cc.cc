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
  State state_;
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
  MOESICCProtocol() {}

  //
  //
  CCLineState* construct_line() const override {
    LineState* line = new LineState;
    line->set_state(State::I);
    return line;
  }

  //
  //
  std::pair<bool, CCActionList> apply(const CCContext& context) const override {
    bool commits = true;
    CCActionList al;
    LineState* line = static_cast<LineState*>(context.line());
    switch (context.msg()->cls()) {
      case MessageClass::AceCmd: {
        // L2 -> CC bus command
        const AceCmdMsg* msg = static_cast<const AceCmdMsg*>(context.msg());
        commits = apply(al, line, msg);
      } break;
      case MessageClass::CohEnd: {
        // DIR -> CC coherence "end" message
        const CohEndMsg* msg = static_cast<const CohEndMsg*>(context.msg());
        commits = apply(al, line, msg);
      } break;
      case MessageClass::CohCmdRsp: {
        // DIR -> CC command acknowledge.
        const CohCmdRspMsg* msg = static_cast<const CohCmdRspMsg*>(context.msg());
        commits = apply(al, line, msg);
      } break;
      case MessageClass::Dt: {
        // {LLC, CC} -> CC cache line.
        const DtMsg* msg = static_cast<const DtMsg*>(context.msg());
        commits = apply(al, line, msg);
      } break;
      default: {
        issue_protocol_violation(al);
      } break;
    }
    return std::make_pair(commits, al);
  }

  //
  //
  bool apply(CCActionList& al, LineState* line, const AceCmdMsg* ace) const {
    bool commits = true;
    switch (line->state()) {
      case State::I: {
        // Lookup home directory.
        const DirMapper* dm = cc()->dm();
        DirModel* dir = dm->lookup(ace->addr());
        {
          // Transaction starts; issue CohStrMsg
          CohSrtMsg* msg = new CohSrtMsg;
          msg->set_t(ace->t());
          msg->set_origin(cc());
          issue_emit_to_noc(al, msg, dir);
        }

        {
          // Issue Corresponding CohCmdMsg indicating the actual
          // coherence command to be executed.
          CohCmdMsg* msg = new CohCmdMsg;
          msg->set_t(ace->t());
          msg->set_origin(cc());
          msg->set_opcode(ace->opcode());
          msg->set_addr(ace->addr());
          issue_emit_to_noc(al, msg, dir);
        }
        // Update coherence state
        issue_update_state(al, line, State::IS_D);
        commits = true;
      } break;

      default: {
        // The L2 should not issue coherence transactions to the CC
        // until prior operations to the line have completed (the L2
        // has sufficient state to understand which transaction are
        // taking place). Therefore, it a command is issued to the bus
        // for a command already taking place, a fatal error has
        // occured.
        issue_protocol_violation(al);
      } break;
    }
    return commits;
  }

  //
  //
  bool apply(CCActionList& al, LineState* line, const CohEndMsg* dt) const {
    bool commits = true;
    switch (line->state()) {
      case State::IS_D: {
      } break;
      default: {
      } break;
    }
    return true;
  }

  //
  //
  bool apply(CCActionList& al, LineState* line, const CohCmdRspMsg* dt) const {
    // No-Operation
    return true;
  }

  //
  //
  bool apply(CCActionList& al, LineState* line, const DtMsg* dt) const {
    bool commits = true;
    switch (line->state()) {
      case State::IS_D: {
        issue_emit_dt_to_l2(al, dt);
      } break;
      default: {
        // TODO
        issue_protocol_violation(al);
      } break;
    }
    return true;
  }

  //
  //
  void issue_emit_dt_to_l2(CCActionList& al, const DtMsg* dt) const {
    L2CacheModel* l2c = cc()->l2c();
    al.push_back(new DtToL2Action(l2c, dt));
  }

  //
  //
  void issue_update_state(DirActionList& al, LineState* line, State state) const {
    al.push_back(new UpdateStateAction(line, state));
  }
};

} // namespace

namespace cc::moesi {

//
//
CCProtocol* build_cc_protocol() { return new MOESICCProtocol{}; }

} // namespace cc::moesi
