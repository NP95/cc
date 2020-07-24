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
#include "utility.h"

namespace {

using namespace cc;

//
//
enum class State {
  I,
  IS,
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
    case State::IS: return "IS";
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
  using cb = L2CommandBuilder;
 public:
  MOESIL2CacheProtocol(kernel::Kernel* k)
      : L2CacheModelProtocol(k, "moesil2")
  {}

  //
  //
  L2LineState* construct_line() const override {
    MOESIL2LineState* line = new MOESIL2LineState();
    return line;
  }

  //
  //
  void apply(L2CacheContext& ctxt, L2CommandList& cl) const override {
    MOESIL2LineState* line = static_cast<MOESIL2LineState*>(ctxt.line());
    switch(ctxt.msg()->cls()) {
      case MessageClass::L2Cmd: {
        // CPU -> L1 command:
        apply(ctxt, cl, line, static_cast<const L2CmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::AceCmdRsp: {
        apply(ctxt, cl, line, static_cast<const AceCmdRspMsg*>(ctxt.msg()));
      } break;
      default: {
        // Unknown message class; error
      } break;
    }
  }

  //
  //
  void evict(L2CacheContext& ctxt, L2CommandList& cl) const override {
    // TODO
  }


 private:
  void apply(L2CacheContext& ctxt, L2CommandList& cl, MOESIL2LineState* line,
             const L2CmdMsg* cmd) const {
    switch (line->state()) {
      case State::I: {
        AceCmdMsg* msg = new AceCmdMsg;
        msg->set_t(cmd->t());
        msg->set_addr(cmd->addr());
        switch (cmd->opcode()) {
          case L2CmdOpcode::L1GetS: {
            // State I; requesting GetS (ie. Shared); issue ReadShared
            msg->set_opcode(AceCmdOpcode::ReadShared);
          } break;
          case L2CmdOpcode::L1GetE: {
            // State I; requesting GetE (i.e. Exclusive); issue ReadShared
            msg->set_opcode(AceCmdOpcode::ReadUnique);
          } break;
        }
        // Issue ACE command to the cache controller.
        issue_msg(cl, ctxt.l2cache()->l2_cc__cmd_q(), msg);
        // Update state
        issue_update_state(cl, line, State::IS);
        // Message is stalled on lookup transaction.
        // Install new entry in transaction table as the transaction
        // has now started and commands are inflight. The transaction
        // itself is not complete at this point.
        cl.push_back(cb::from_opcode(L2Opcode::TableInstall));
        cl.push_back(cb::from_opcode(L2Opcode::TableGetCurrentState));
        // Source Message Queue is blocked until the current
        // transaction (lookup to L2) has completed.
        cl.push_back(cb::from_opcode(L2Opcode::TableMqAddToBlockedList));
        // Set blocked status in Message Queue to rescind requestor
        // status.
        cl.push_back(cb::from_opcode(L2Opcode::MqSetBlocked));
        cl.push_back(cb::from_opcode(L2Opcode::MsgL1CmdExtractAddr));
        // Install new cache line.
        cl.push_back(cb::from_opcode(L2Opcode::InstallLine));
        // Advance to next
        cl.push_back(cb::from_opcode(L2Opcode::WaitNextEpochOrWait));
      } break;
      case State::S: {
        switch (cmd->opcode()) {
          case L2CmdOpcode::L1GetS: {
            // L1 requests a line which is already in the S-state.
            L2CmdRspMsg* msg = new L2CmdRspMsg;
            msg->set_t(cmd->t());
            L2CacheModel* l2cache = ctxt.l2cache();
            issue_msg(cl, l2cache->l2_l1__rsp_q(0), msg);
            // Consume L1Cmd as it can complete successfully.
            cl.push_back(cb::from_opcode(L2Opcode::MsgConsume));
            // Advance to next
            cl.push_back(cb::from_opcode(L2Opcode::WaitNextEpochOrWait));
          } break;
          default: {
          } break;
        }
      } break;
      default: {
      } break;
    }
  }

  void apply(L2CacheContext& ctxt, L2CommandList& cl,
             MOESIL2LineState* line, const AceCmdRspMsg* msg) const {
    switch (line->state()) {
      case State::IS: {
        L2CmdRspMsg* rsp = new L2CmdRspMsg;
        rsp->set_t(msg->t());
        // Always sending to the zeroth L1
        L2CacheModel* l2cache = ctxt.l2cache();
        issue_msg(cl, l2cache->l2_l1__rsp_q(0), rsp);
        issue_update_state(cl, line, State::S);
        // Consume L1Cmd as it can complete successfully.
        cl.push_back(cb::from_opcode(L2Opcode::MsgConsume));
        // Advance to next
        cl.push_back(cb::from_opcode(L2Opcode::WaitNextEpochOrWait));
      } break;
      default: {
      } break;
    }
  }

  void issue_update_state(
      L2CommandList& cl, MOESIL2LineState* line, State state) const {
    struct UpdateStateAction : public CoherenceAction {
      UpdateStateAction(MOESIL2LineState* line, State state)
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
      MOESIL2LineState* line_ = nullptr;
      State state_;
    };
    CoherenceAction* action = new UpdateStateAction(line, state);
    cl.push_back(cb::from_action(action));
  }
};

} // namespace

namespace cc::moesi {

L2CacheModelProtocol* build_l2_protocol(kernel::Kernel* k) {
  return new MOESIL2CacheProtocol(k);
}

} // namespace cc::moesi;
