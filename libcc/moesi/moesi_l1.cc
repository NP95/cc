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

#include "protocol.h"
#include "l1cache.h"
#include "l2cache.h"
#include "kernel.h"
#include "utility.h"

namespace {

using namespace cc;

enum class State {
  // Invalid
  I,
  IS,
  // Shared
  S,
  IE,
  SE,
  // Exclusive
  E,
  // Modified
  M,
  MI
};

//
//
const char* to_string(State state) {
  switch (state) {
    case State::I: return "I";
    case State::IS: return "IS";
    case State::S: return "S";
    case State::SE: return "SE";
    case State::IE: return "IE";
    case State::E: return "E";
    case State::M: return "M";
    case State::MI: return "MI";
    default: return "Invalid";
  }
};

//
//
bool is_stable(State state) {
  switch (state) {
    case State::I:
    case State::S:
    case State::E:
    case State::M:
      return true;
    default:
      return false;
  }
}

//
//
class MOESIL1LineState : public L1LineState {
 public:
  MOESIL1LineState() {}

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
class MOESIL1CacheProtocol : public L1CacheModelProtocol {
  using cb = L1CommandBuilder;
  
 public:
  MOESIL1CacheProtocol(kernel::Kernel* k) :
      L1CacheModelProtocol(k, "moesil2")
  {}

  //
  //

  L1LineState* construct_line() const override {
    return new MOESIL1LineState();
  }

  //
  //
  void apply(L1CacheContext& c, L1CommandList& cl) const override {
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(c.line());
    switch(c.msg()->cls()) {
      case MessageClass::L1Cmd: {
        // CPU -> L1 command:
        apply(c, cl, line, static_cast<const L1CmdMsg*>(c.msg()));
      } break;
      case MessageClass::L2CmdRsp: {
        apply(c, cl, line, static_cast<const L2CmdRspMsg*>(c.msg()));
      } break;
      default: {
        // Unknown message class; error
      } break;
    }
  }

  //
  //p
  void evict(L1CacheContext& c, L1CommandList& cl) const override {
    // TODO
  }

 private:

  void apply(L1CacheContext& c, L1CommandList& cl, MOESIL1LineState* line,
             const L1CmdMsg* msg) const {
        
    switch (line->state()) {
      case State::I: {
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
        l2cmdmsg->set_l1cache(c.l1cache());
        // Issue L2 command
        issue_msg(cl, c.l1cache()->l1_l2__cmd_q(), l2cmdmsg);
        // Update state I -> IS
        issue_update_state(cl, line, State::IS);
        // Message is stalled on lookup transaction.
        // Install new entry in transaction table as the transaction
        // has now started and commands are inflight. The transaction
        // itself is not complete at this point.
        cl.push_back(cb::from_opcode(L1Opcode::TableInstall));
        cl.push_back(cb::from_opcode(L1Opcode::TableGetCurrentState));
        // Source Message Queue is blocked until the current
        // transaction (lookup to L2) has completed.
        cl.push_back(cb::from_opcode(L1Opcode::TableMqAddToBlockedList));
        // Set blocked status in Message Queue to rescind requestor
        // status.
        cl.push_back(cb::from_opcode(L1Opcode::MqSetBlocked));
        cl.push_back(cb::from_opcode(L1Opcode::MsgL1CmdExtractAddr));
        // Install new cache line.
        cl.push_back(cb::from_opcode(L1Opcode::InstallLine));
        // Advance to next
        cl.push_back(cb::from_opcode(L1Opcode::WaitNextEpochOrWait));
      } break;
      case State::S: {
        // Line is present in the cache. 
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            // LD to line in S-state can complete immediately. Forward
            // the response to the CPU.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            issue_msg(cl, c.l1cache()->l1_cpu__rsp_q(), rsp);
            // Consume L1Cmd as it can complete successfully.
            cl.push_back(cb::from_opcode(L1Opcode::MsgConsume));
            // Advance to next
            cl.push_back(cb::from_opcode(L1Opcode::WaitNextEpochOrWait));
          } break;
          case L1CacheOpcode::CpuStore: {
            // Store instruction to line in S-state. Line must be promoted to
            // the E state before the transaction can complete.
            issue_update_state(cl, line, State::SE);
          } break;
        }
      } break;
      case State::M: {
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            // Ld from M-state; commits immediately
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            // Issue response to CPU.
            issue_msg(cl, c.l1cache()->l1_cpu__rsp_q(), rsp);
            // Consume message
            cl.push_back(cb::from_opcode(L1Opcode::MsgConsume));
            // Advance to next
            cl.push_back(cb::from_opcode(L1Opcode::WaitNextEpochOrWait));
          } break;
          case L1CacheOpcode::CpuStore: {
            // St to M-state; commits immediate.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            // Issue response to CPU.
            issue_msg(cl, c.l1cache()->l1_cpu__rsp_q(), rsp);
            // Consume message
            cl.push_back(cb::from_opcode(L1Opcode::MsgConsume));
            // Advance to next
            cl.push_back(cb::from_opcode(L1Opcode::WaitNextEpochOrWait));
          } break;
          default: {

          } break;
        }
      } break;
      case State::E: {
        switch (msg->opcode()) {
          case L1CacheOpcode::CpuLoad: {
            // Load to line in the exclusive state; instruction can complete
            // successfully.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            issue_msg(cl, c.l1cache()->l1_cpu__rsp_q(), rsp);
            // Instruction commits.
            cl.push_back(cb::from_opcode(L1Opcode::MsgConsume));
            // Advance to next
            cl.push_back(cb::from_opcode(L1Opcode::WaitNextEpochOrWait));
          } break;
          case L1CacheOpcode::CpuStore: {
            // Store to line the exclusive state; instruction can complete
            // but line must transition to the modified state.
            L1CmdRspMsg* rsp = new L1CmdRspMsg;
            rsp->set_t(msg->t());
            issue_msg(cl, c.l1cache()->l1_cpu__rsp_q(), rsp);
            issue_update_state(cl, line, State::M);
            // Instruction commits
            cl.push_back(cb::from_opcode(L1Opcode::MsgConsume));
            // Advance to next
            cl.push_back(cb::from_opcode(L1Opcode::WaitNextEpochOrWait));
          } break;
          default: {
            // Unknown CPU command.
          } break;
        }
      } break;
      default: {
        // Invalid state
      } break;
    }
  }

  void apply(L1CacheContext& c, L1CommandList& cl, MOESIL1LineState* line,
             const L2CmdRspMsg* msg) const {
    switch (line->state()) {
      case State::IS: {
        // Update state
        issue_update_state(cl, line, State::S);
        // Update transaction table; wake all blocked Message Queues
        // and delete context.
        cl.push_back(cb::from_opcode(L1Opcode::TableGetCurrentState));
        cl.push_back(cb::from_opcode(L1Opcode::TableMqUnblockAll));
        // Consume committed message.
        cl.push_back(cb::from_opcode(L1Opcode::MsgConsume));
        // Advance to next
        cl.push_back(cb::from_opcode(L1Opcode::WaitNextEpochOrWait));
      } break;
      case State::SE: {
        //
      } break;
      default: {
        // Invalid state
      } break;
    }
  }

  void issue_update_state(
      L1CommandList& cl, MOESIL1LineState* line, State state) const {
    struct UpdateStateAction : public CoherenceAction {
      UpdateStateAction(MOESIL1LineState* line, State state)
          : line_(line), state_(state)
      {}
      std::string to_string() const override {
        using cc::to_string;
        
        KVListRenderer r;
        r.add_field("action", "update state");
        r.add_field("current", to_string(line_->state()));
        r.add_field("next", to_string(state_));
        return r.to_string();
      }
      bool execute() override {
        line_->set_state(state_);
        return true;
      }
     private:
      MOESIL1LineState* line_ = nullptr;
      State state_;
    };
    CoherenceAction* action = new UpdateStateAction(line, state);
    cl.push_back(cb::from_action(action));
  }

};

} // namespace


namespace cc::moesi {

//
//
L1CacheModelProtocol* build_l1_protocol(kernel::Kernel* k) {
  return new MOESIL1CacheProtocol(k);
}

} // namespace cc::moesi

