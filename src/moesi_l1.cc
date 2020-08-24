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

#include "kernel.h"
#include "l1cache.h"
#include "l2cache.h"
#include "protocol.h"
#include "utility.h"

namespace {

using namespace cc;

enum class State {
  // Invalid
  I,
  IS,
  IE,
  // Shared
  S,
  SI,
  SE,
  // Exclusive
  E,
  EI,
  // Modified
  M,
  MI,

  // Invalid; placeholder
  X 
};

//
//
const char* to_string(State state) {
  switch (state) {
    case State::I:
      return "I";
    case State::IS:
      return "IS";
    case State::IE:
      return "IE";
    case State::S:
      return "S";
    case State::SI:
      return "SI";
    case State::SE:
      return "SE";
    case State::E:
      return "E";
    case State::EI:
      return "EI";
    case State::M:
      return "M";
    case State::MI:
      return "MI";
    default:
      return "Invalid";
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
  bool is_stable() const override {
    switch (state()) {
      case State::I:
      case State::S:
      case State::E:
      case State::M:
        return true;
      default:
        return false;
    }
  }

  // Return true if line is in a state which allows reading.
  bool is_readable() const override {
    switch (state()) {
      case State::S:
      case State::E:
      case State::M:
        return true;
      default:
        return false;
    }
  }

  // Return true if the line is in a state which allows writing.
  bool is_writeable() const override {
    switch (state()) {
      case State::E:
      case State::M:
        return true;
      default:
        return false;
    }
  }

 private:
  State state_ = State::I;
};

enum class L1EgressQueue { L2CmdQ, CpuRspQ, Invalid };

const char* to_string(L1EgressQueue q) {
  switch (q) {
    case L1EgressQueue::L2CmdQ:
      return "L2CmdQ";
    case L1EgressQueue::CpuRspQ:
      return "CpuRspQ";
    case L1EgressQueue::Invalid:
      return "Invalid";
    default:
      return "Invalid";
  }
}

//
//
class MOESIL1CacheProtocol : public L1CacheAgentProtocol {
  using cb = L1CommandBuilder;

 public:
  MOESIL1CacheProtocol(kernel::Kernel* k)
      : L1CacheAgentProtocol(k, "moesil2") {}

  //
  //

  L1LineState* construct_line() const override {
    return new MOESIL1LineState();
  }

  //
  //
  void apply(L1CacheContext& ctxt, L1CommandList& cl) const override {
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(ctxt.line());
    const MessageClass cls = ctxt.msg()->cls();
    switch (cls) {
      case MessageClass::L1Cmd: {
        // CPU -> L1 command:
        apply(ctxt, cl, line, static_cast<const L1CmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::L2CmdRsp: {
        apply(ctxt, cl, line, static_cast<const L2CmdRspMsg*>(ctxt.msg()));
      } break;
      default: {
        // Unknown message class; error
      } break;
    }
  }

  //
  //
  void evict(L1CacheContext& ctxt, L1CommandList& cl) const override {
    // Current invoke L1 cache instance configuration.
    const L1CacheAgentConfig& config = ctxt.l1cache()->config();

    // Update Transaction State with data snooped from command message.
    L1TState* tstate = ctxt.tstate();
    // Address becomes eviction address
    tstate->set_addr(ctxt.addr());
    // Line becomes evictee.
    tstate->set_line(ctxt.line());

    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(ctxt.line());
    const State state = line->state();
    switch (state) {
      case State::M:
        // As in the Exclusive state, L2 already has the most recent
        // upto date copy of the line therefore we just need to inform
        // L2 that the line is being removed from the cache, and it
        // will itself perform the writeback.
      case State::E: {
        // Evict line present in the Exclusive state. L2 should
        // already have a recent upto date copy of the cache line,
        // therefore no actual data needs to be written-back, however
        // we inform L2 so that it may (conditionally) inform the home
        // directory that the line is no longer present in the cache
        // (when silent evictions are not allowed).

        // Issue command notification to L2.
        Transaction* t = ctxt.msg()->t();
        L2CmdMsg* cmd = Pool<L2CmdMsg>::construct();
        cmd->set_t(t);
        cmd->set_addr(ctxt.addr());
        cmd->set_opcode(L2CmdOpcode::L1Put);
        cmd->set_l1cache(ctxt.l1cache());
        issue_msg_to_queue(L1EgressQueue::L2CmdQ, cl, ctxt, cmd);
        // Line transition to Invalid state.
        const State next_state = (state == State::M) ? State::MI : State::EI;
        issue_update_state(cl, line, next_state);
        // Start new transaction as we are awaiting the response from
        // L2 before the line is evicted from the cache.
        cl.transaction_start(t, config.is_blocking_cache);
      } break;
      default: {
      } break;
    }
  }

  //
  //
  void set_line_shared_or_invalid(L1CacheContext& ctxt, L1CommandList& cl,
                                  bool shared) const override {
    MOESIL1LineState* line = static_cast<MOESIL1LineState*>(ctxt.line());
    issue_update_state(cl, line, shared ? State::S : State::I);
    if (!shared) {
      cl.push_back(cb::build_remove_line(ctxt.addr()));
    }
  }

 private:
  void apply(L1CacheContext& ctxt, L1CommandList& cl, MOESIL1LineState* line,
             const L1CmdMsg* msg) const {
    // Current invoke L1 cache instance.
    const L1CacheAgentConfig& config = ctxt.l1cache()->config();

    // Update Transaction State with data snooped from command message.
    L1TState* tstate = ctxt.tstate();
    tstate->set_line(ctxt.line());
    tstate->set_addr(msg->addr());
    tstate->set_opcode(msg->opcode());

    const State state = line->state();
    switch (state) {
      case State::I: {
        // Emit request to L2.
        L2CmdMsg* l2cmdmsg = Pool<L2CmdMsg>::construct();
        l2cmdmsg->set_t(msg->t());
        l2cmdmsg->set_addr(msg->addr());
        switch (msg->opcode()) {
          case L1CmdOpcode::CpuLoad: {
            // Update state I -> IS
            issue_update_state(cl, line, State::IS);
            // Issue GetS
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetS);
          } break;
          case L1CmdOpcode::CpuStore: {
            // Update state I -> IE
            issue_update_state(cl, line, State::IE);
            // Issue GetS
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetE);
          } break;
          default: {
          } break;
        }
        l2cmdmsg->set_l1cache(ctxt.l1cache());
        // Issue L2 command
        issue_msg_to_queue(L1EgressQueue::L2CmdQ, cl, ctxt, l2cmdmsg);

        // In the non-blocking cache, the transaction state takes
        // ownership of the message at the head of the command queue
        // as this is subsequently dequeued to prevent head-of-line
        // blocking.
        if (!config.is_blocking_cache) {
          tstate->set_do_replay(true);
          tstate->set_msg(msg);
        }

        // Message is stalled on lookup transaction.
        // Install new entry in transaction table as the transaction
        // has now started and commands are inflight. The transaction
        // itself is not complete at this point.
        cl.transaction_start(msg->t(), config.is_blocking_cache);
      } break;
      case State::S: {
        // Line is present in the cache.
        switch (msg->opcode()) {
          case L1CmdOpcode::CpuLoad: {
            // LD to line in S-state can complete immediately. Forward
            // the response to the CPU.
            L1CmdRspMsg* rsp = Pool<L1CmdRspMsg>::construct();
            rsp->set_t(msg->t());
            issue_msg_to_queue(L1EgressQueue::CpuRspQ, cl, ctxt, rsp);
            // Raise event
            cl.push_back(cb::build_cache_event(L1CacheEvent::ReadHit, msg->addr()));
            // Advance to next
            cl.next_and_do_consume(true);
          } break;
          case L1CmdOpcode::CpuStore: {
            // Store instruction to line in S-state. Line must be promoted to
            // the E state before the transaction can complete.

            // Issue GetE message to owning L2 cache.
            L2CmdMsg* l2cmdmsg = Pool<L2CmdMsg>::construct();
            l2cmdmsg->set_t(msg->t());
            l2cmdmsg->set_addr(msg->addr());
            l2cmdmsg->set_opcode(L2CmdOpcode::L1GetE);
            l2cmdmsg->set_l1cache(ctxt.l1cache());
            // Issue L2 command
            // issue_msg(cl, ctxt.l1cache()->l1_l2__cmd_q(), l2cmdmsg);
            issue_msg_to_queue(L1EgressQueue::L2CmdQ, cl, ctxt, l2cmdmsg);
            // Start new transaction
            cl.transaction_start(msg->t(), config.is_blocking_cache);
            // State transitions to SE state.
            issue_update_state(cl, line, State::SE);
          } break;
          default: {
            // Invalid command.
          } break;
        }
      } break;
      case State::M: {
        switch (msg->opcode()) {
          case L1CmdOpcode::CpuLoad: {
            // Ld from M-state; commits immediately
            L1CmdRspMsg* rsp = Pool<L1CmdRspMsg>::construct();
            rsp->set_t(msg->t());
            // Issue response to CPU.
            issue_msg_to_queue(L1EgressQueue::CpuRspQ, cl, ctxt, rsp);
            // Raise event
            cl.push_back(cb::build_cache_event(L1CacheEvent::ReadHit, msg->addr()));
            // Advance to next and consume
            cl.next_and_do_consume(true);
          } break;
          case L1CmdOpcode::CpuStore: {
            // St to M-state; commits immediate.
            L1CmdRspMsg* rsp = Pool<L1CmdRspMsg>::construct();
            rsp->set_t(msg->t());
            // Issue response to CPU.
            issue_msg_to_queue(L1EgressQueue::CpuRspQ, cl, ctxt, rsp);
            // Raise event
            cl.push_back(cb::build_cache_event(L1CacheEvent::WriteHit, msg->addr()));
            // Advance to next and consume
            cl.next_and_do_consume(true);
          } break;
          default: {
          } break;
        }
      } break;
      case State::E: {
        switch (msg->opcode()) {
          case L1CmdOpcode::CpuLoad: {
            // Load to line in the exclusive state; instruction can complete
            // successfully.
            L1CmdRspMsg* rsp = Pool<L1CmdRspMsg>::construct();
            rsp->set_t(msg->t());
            issue_msg_to_queue(L1EgressQueue::CpuRspQ, cl, ctxt, rsp);
            // Raise event
            cl.push_back(cb::build_cache_event(L1CacheEvent::ReadHit, msg->addr()));
            // Advance to next and consume
            cl.next_and_do_consume(true);
          } break;
          case L1CmdOpcode::CpuStore: {
            // Store to line the exclusive state; instruction can complete
            // but line must transition to the modified state.
            L1CmdRspMsg* rsp = Pool<L1CmdRspMsg>::construct();
            rsp->set_t(msg->t());
            issue_msg_to_queue(L1EgressQueue::CpuRspQ, cl, ctxt, rsp);
            issue_update_state(cl, line, State::M);
            // Write through to L2. such that L2 sees the transition
            // to M immediately.
            cl.push_back(L1Opcode::SetL2LineModified);
            // Raise event
            cl.push_back(cb::build_cache_event(L1CacheEvent::WriteHit, msg->addr()));
            // Advance to next and consume
            cl.next_and_do_consume(true);
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

  void apply(L1CacheContext& ctxt, L1CommandList& cl, MOESIL1LineState* line,
             const L2CmdRspMsg* msg) const {
    Transaction* t = msg->t();
    const addr_t addr = ctxt.tstate()->addr();
    const State state = line->state();
    switch (state) {
      case State::IS: {
        // Update state
        const State next_state = msg->is() ? State::S : State::E;
        issue_update_state(cl, line, next_state);

        // Update transaction table; wake all blocked Message Queues
        // and delete context.
        const L1CacheEvent l1event = (next_state == State::S) ?
                                     L1CacheEvent::InstallShareable :
                                     L1CacheEvent::InstallWriteable;
        cl.push_back(cb::build_cache_event(l1event, addr));
        cl.transaction_end(t);
        // Advance to next and consume
        cl.next_and_do_consume(true);
      } break;
      case State::IE: {
        // Update state
        issue_update_state(cl, line, State::E);
        // Raise event
        cl.push_back(cb::build_cache_event(L1CacheEvent::InstallWriteable, addr));
        // Update transaction table; wake all blocked Message Queues
        // and delete context.
        cl.transaction_end(t);
        // Advance to next and consume
        cl.next_and_do_consume(true);
      } break;
      case State::SE: {
        // Line is installed in 'E' state.
        issue_update_state(cl, line, State::E);
        // Update transaction table; wake all blocked Message Queues
        // and delete context.
        cl.push_back(cb::build_cache_event(L1CacheEvent::InstallWriteable, addr));
        cl.transaction_end(t);
        // Advance to next and consume
        cl.next_and_do_consume(true);
      } break;
      case State::SI:
      case State::MI:
      case State::EI: {
        // Shared/Modifed/Exclusive -> Invalid
        //
        // Edge entered upon an eviction from L1 cache. As L1 is
        // write-through to L2, L2 maintains a recent up to date copy
        // of the line and this transaction therefore serves as
        // notification that the line has been evicted from L1. In the
        // case of SI, L1 may optionally silently evict the line
        // without L2 notification, in which the message is never
        // sent.

        // Transition back to Invalid state and evict line.
        issue_update_state(cl, line, State::I);
        // Remove line from cache as now invalid.
        cl.push_back(cb::build_remove_line(addr));
        // Transaction ends
        cl.transaction_end(t);
        // Raise line is invalidate line.
        cl.push_back(cb::build_cache_event(L1CacheEvent::InvalidateLine, addr));
        // Advance to next and consume
        cl.next_and_do_consume(true);
      } break;
      default: {
        // Invalid state
      } break;
    }
  }

  void issue_msg_to_queue(L1EgressQueue eq, L1CommandList& cl,
                          L1CacheContext& ctxt, const Message* msg) const {
    struct EmitMessageActionProxy : public L1CoherenceAction {
      EmitMessageActionProxy() = default;

      std::string to_string() const override {
        KVListRenderer r;
        r.add_field("action", "emit message");
        r.add_field("mq", mq_->path());
        r.add_field("msg", msg_->to_string());
        return r.to_string();
      }

      void set_eq(L1EgressQueue eq) { eq_ = eq; }
      void set_mq(MessageQueue* mq) { mq_ = mq; }
      void set_msg(const Message* msg) { msg_ = msg; }

      void set_resources(L1Resources& r) const override {
        switch (eq_) {
          case L1EgressQueue::L2CmdQ: {
            r.set_l2_cmd_n(1 + r.l2_cmd_n());
          } break;
          case L1EgressQueue::CpuRspQ: {
            r.set_cpu_rsp_n(1 + r.cpu_rsp_n());
          } break;
          default: {
            // No cost
          } break;
        }
      }

      //
      bool execute() override { return mq_->issue(msg_); }

     private:
      //
      L1EgressQueue eq_ = L1EgressQueue::Invalid;
      //
      MessageQueue* mq_ = nullptr;
      //
      const Message* msg_ = nullptr;
    };

    EmitMessageActionProxy* action = new EmitMessageActionProxy;
    action->set_eq(eq);
    action->set_msg(msg);
    switch (eq) {
      case L1EgressQueue::L2CmdQ: {
        action->set_mq(ctxt.l1cache()->l1_l2__cmd_q());
      } break;
      case L1EgressQueue::CpuRspQ: {
        action->set_mq(ctxt.l1cache()->l1_cpu__rsp_q());
      } break;
      default: {
      } break;
    }
    cl.push_back(action);
  }

  void issue_update_state(L1CommandList& cl, MOESIL1LineState* line,
                          State state) const {
    struct UpdateStateAction : public L1CoherenceAction {
      UpdateStateAction(MOESIL1LineState* line, State state)
          : line_(line), state_(state) {}
      std::string to_string() const override {
        using cc::to_string;

        KVListRenderer r;
        r.add_field("action", "update state");
        r.add_field("current", to_string(line_->state()));
        r.add_field("next", to_string(state_));
        return r.to_string();
      }
      void set_resources(L1Resources& r) const override {
        // No resources required for state update.
      }
      bool execute() override {
        line_->set_state(state_);
        return true;
      }

     private:
      MOESIL1LineState* line_ = nullptr;
      State state_;
    };
    L1CoherenceAction* action = new UpdateStateAction(line, state);
    cl.push_back(action);
  }
};

}  // namespace

namespace cc::moesi {

//
//
L1CacheAgentProtocol* build_l1_protocol(kernel::Kernel* k) {
  return new MOESIL1CacheProtocol(k);
}

}  // namespace cc::moesi
