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

#include "amba.h"
#include "ccntrl.h"
#include "dir.h"
#include "l2cache.h"
#include "mem.h"
#include "moesi.h"
#include "noc.h"
#include "protocol.h"
#include "utility.h"

namespace {

using namespace cc;

//
//
class Line : public CCLineState {
 public:
  Line() = default;

  // Accessors:
  Transaction* t() const { return t_; }
  bool is() const { return is_; }
  bool pd() const { return pd_; }
  std::size_t dt_n() const { return dt_n_; }
  std::size_t dt_i() const { return dt_i_; }
  bool awaiting_cohend() const { return awaiting_cohend_; }
  bool awaiting_cohcmdrsp() const { return awaiting_cohcmdrsp_; }

  // Setters:
  void set_t(Transaction* t) { t_ = t; }
  void set_is(bool is) { is_ = is; }
  void set_pd(bool pd) { pd_ = pd; }
  void set_dt_n(std::size_t dt_n) { dt_n_ = dt_n; }
  void set_dt_i(std::size_t dt_i) { dt_i_ = dt_i; }
  void set_awaiting_cohend(bool awaiting_cohend) {
    awaiting_cohend_ = awaiting_cohend;
  }
  void set_awaiting_cohcmdrsp(bool awaiting_cohcmdrsp) {
    awaiting_cohcmdrsp_ = awaiting_cohcmdrsp;
  }

  //
  bool is_complete() const {
    // If awaiting coherence end; not complete
    if (awaiting_cohend()) return false;
    // If awaiting coherence command response; not complete
    if (awaiting_cohcmdrsp()) return false;

    // Otherwise, if the expected number of data transfers have
    // arrived, we are free to compute the final response.
    return dt_n() == dt_i();
  }

 private:
  // Current Transaction (retained as the message which would
  // otherwise retain it has been dequeued by the type the final
  // CohEnd message has been emitted.
  Transaction* t_ = nullptr;
  // CohEnd indicates response is shared.
  bool is_ = false;
  // CohEnd indicates response was passed as dirty.
  bool pd_ = false;
  // The total number of expected data transfers.
  std::size_t dt_n_ = 0;
  // The total number of received data transfers.
  std::size_t dt_i_ = 0;
  // Flag indicating that CohEnd msg has been received/awaiting.
  bool awaiting_cohend_ = false;
  // Flag indicating that CohCmdRsp msg has been received/awaiting.
  bool awaiting_cohcmdrsp_ = false;
};

//
//
class ApplyMsgAction : public CCCoherenceAction {
 public:
  ApplyMsgAction(const Message* msg, Line* line) : msg_(msg), line_(line) {}

  std::string to_string() const override { return "TODO"; }

  bool execute() override {
    const MessageClass cls = msg_->cls();

    bool ret = false;
    switch (cls) {
      case MessageClass::AceCmd: {
        ret = execute_apply(static_cast<const AceCmdMsg*>(msg_));
      } break;
      case MessageClass::CohEnd: {
        ret = execute_apply(static_cast<const CohEndMsg*>(msg_));
      } break;
      case MessageClass::CohCmdRsp: {
        ret = execute_apply(static_cast<const CohCmdRspMsg*>(msg_));
      } break;
      case MessageClass::Dt: {
        ret = execute_apply(static_cast<const DtMsg*>(msg_));
      } break;
      default: {
        // Other.
      } break;
    }
    return ret;
  }

 private:
  bool execute_apply(const AceCmdMsg* msg) const {
    line_->set_t(msg->t());
    return true;
  }

  bool execute_apply(const CohEndMsg* msg) const {
    line_->set_is(msg->is());
    line_->set_pd(msg->pd());
    line_->set_dt_n(msg->dt_n());
    line_->set_awaiting_cohend(false);
    return true;
  }

  bool execute_apply(const CohCmdRspMsg* msg) const {
    line_->set_awaiting_cohcmdrsp(false);
    return true;
  }

  bool execute_apply(const DtMsg* msg) const {
    line_->set_dt_i(line_->dt_i() + 1);
    return true;
  }

  //
  const Message* msg_ = nullptr;
  //
  Line* line_ = nullptr;
};

//
//
enum class LineUpdate { SetAwaitingCohEnd, SetAwaitingCohCmdRsp, Invalid };

const char* to_string(LineUpdate update) {
  switch (update) {
    case LineUpdate::SetAwaitingCohEnd:
      return "SetAwaitingCohEnd";
    case LineUpdate::SetAwaitingCohCmdRsp:
      return "SetAwaitingCohCmdRsp";
    case LineUpdate::Invalid:
      return "Invalid";
    default:
      return "Invalid";
  }
}

struct LineUpdateAction : public CCCoherenceAction {
  LineUpdateAction(Line* line, LineUpdate update)
      : line_(line), update_(update) {}
  std::string to_string() const override {
    using cc::to_string;
    KVListRenderer r;
    r.add_field("update", to_string(update_));
    return r.to_string();
  }
  bool execute() override {
    switch (update_) {
      case LineUpdate::SetAwaitingCohEnd: {
        line_->set_awaiting_cohend(true);
      } break;
      case LineUpdate::SetAwaitingCohCmdRsp: {
        line_->set_awaiting_cohcmdrsp(true);
      } break;
      default: {
      } break;
    }
    return true;
  }

 private:
  Line* line_ = nullptr;
  LineUpdate update_ = LineUpdate::Invalid;
};

//
//
class SnpLine : public CCSnpLineState {
 public:
  SnpLine() = default;

  //
  Agent* origin() const { return origin_; }
  Agent* agent() const { return agent_; }

  //
  void set_origin(Agent* origin) { origin_ = origin; }
  void set_agent(Agent* agent) { agent_ = agent; }

 private:
  // Originating directory.
  Agent* origin_ = nullptr;
  // Requesting agent.
  Agent* agent_ = nullptr;
};

// Destination egress queue definition
enum class CCEgressQueue {
  L2RspQ,
  L2CmdQ,
  Invalid
};

const char* to_string(CCEgressQueue q) {
  switch (q) {
    case CCEgressQueue::L2RspQ:
      return "L2RspQ";
    case CCEgressQueue::L2CmdQ:
      return "L2CmdQ";
    case CCEgressQueue::Invalid:
      [[fallthrough]];
    default:
      return "Invalid";
  }
}

//
//
class MOESICCProtocol : public CCProtocol {
  using cb = CCCommandBuilder;

 public:
  MOESICCProtocol(kernel::Kernel* k) : CCProtocol(k, "moesicc") {}

  //
  //
  CCLineState* construct_line() const override { return new Line; }

  //
  //
  CCSnpLineState* construct_snp_line() const override { return new SnpLine; }

  //
  //
  void apply(CCContext& ctxt, CCCommandList& cl) const override {
    const MessageClass cls = ctxt.msg()->cls();
    switch (cls) {
      case MessageClass::AceCmd: {
        eval_msg(ctxt, cl, static_cast<const AceCmdMsg*>(ctxt.msg()));
      } break;
      case MessageClass::CohEnd: {
        eval_msg(ctxt, cl, static_cast<const CohEndMsg*>(ctxt.msg()));
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
  bool is_complete(CCContext& ctxt, CCCommandList& cl) const override {
    Line* line = static_cast<Line*>(ctxt.line());

    // Check line is complete, if not bail.
    if (!line->is_complete()) return false;

    AceCmdRspMsg* rsp = Pool<AceCmdRspMsg>::construct();
    rsp->set_t(line->t());
    rsp->set_origin(ctxt.cc());
    rsp->set_pd(line->pd());
    rsp->set_is(line->is());
    issue_msg_to_queue(CCEgressQueue::L2RspQ, ctxt, cl, rsp);

    // Transaction is now complete; delete entry from transaction table.
    cl.push_transaction_end(line->t());

    return true;
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
        eval_msg(ctxt, cl, static_cast<const AceSnpRspMsg*>(ctxt.msg()));
      } break;
      case MessageClass::DtRsp: {
        eval_msg(ctxt, cl, static_cast<const DtRspMsg*>(ctxt.msg()));
      } break;
      default: {
        LogMessage msg("Invalid message class received: ");
        msg.append(cc::to_string(cls));
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl,
                const CohCmdRspMsg* msg) const {
    // Apply message to transaction state.
    issue_apply_msg(ctxt, cl, msg);
    // Consume and advance
    cl.next_and_do_consume(true);
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl,
                const AceCmdMsg* msg) const {
    // Apply message to transaction state.
    issue_apply_msg(ctxt, cl, msg);

    // TODO: a lot of this is redundant. Cleanup.

    // Defaults
    bool set_awaiting_cohend = false;
    bool set_awaiting_cmdrsp = false;

    const AceCmdOpcode opcode = msg->opcode();
    switch (opcode) {
      case AceCmdOpcode::ReadShared: {
        const DirMapper* dm = ctxt.cc()->dm();

        // Issue coherence start message
        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(ctxt.cc());
        cohsrt->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohsrt, dm->lookup(msg->addr()));

        // Issue coherence command message
        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(ctxt.cc());
        cohcmd->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohcmd, dm->lookup(msg->addr()));

        // Set flags
        set_awaiting_cohend = true;
        set_awaiting_cmdrsp = true;

        // Set flag indicating we are awaiting the command response.

        // ACE command advances to active state; install entry within
        // transaction table.
        cl.push_back(CCOpcode::TransactionStart);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case AceCmdOpcode::ReadUnique: {
        const DirMapper* dm = ctxt.cc()->dm();

        // Issue coherence start message
        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(ctxt.cc());
        cohsrt->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohsrt, dm->lookup(msg->addr()));

        // Issue coherence command message
        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(ctxt.cc());
        cohcmd->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohcmd, dm->lookup(msg->addr()));

        // Set flags
        set_awaiting_cohend = true;
        set_awaiting_cmdrsp = true;

        // ACE command advances to active state; install entry within
        // transaction table.
        cl.push_back(CCOpcode::TransactionStart);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case AceCmdOpcode::CleanUnique: {
        // Agent has copy of cache line and requests promotion of the
        // line to an owning state.
        const DirMapper* dm = ctxt.cc()->dm();

        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(ctxt.cc());
        cohsrt->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohsrt, dm->lookup(msg->addr()));

        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(ctxt.cc());
        cohcmd->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohcmd, dm->lookup(msg->addr()));

        // Set flags
        set_awaiting_cohend = true;
        set_awaiting_cmdrsp = true;

        // ACE command advances to active state; install entry within
        // transaction table.
        cl.push_back(CCOpcode::TransactionStart);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case AceCmdOpcode::Evict: {
        const DirMapper* dm = ctxt.cc()->dm();

        // Issue cohernce start message
        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(ctxt.cc());
        cohsrt->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohsrt, dm->lookup(msg->addr()));

        // Issue cohernece command
        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(ctxt.cc());
        cohcmd->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohcmd, dm->lookup(msg->addr()));

        // Set flags
        set_awaiting_cohend = true;
        set_awaiting_cmdrsp = true;

        // ACE command advances to active state; install entry within
        // transaction table.
        cl.push_back(CCOpcode::TransactionStart);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      case AceCmdOpcode::WriteBack: {
        // L2 evicts a line and initiates a write to memory.
        const DirMapper* dm = ctxt.cc()->dm();

        // Issue coherence start message
        CohSrtMsg* cohsrt = new CohSrtMsg;
        cohsrt->set_t(msg->t());
        cohsrt->set_origin(ctxt.cc());
        cohsrt->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohsrt, dm->lookup(msg->addr()));

        // Issue coherence command message
        CohCmdMsg* cohcmd = new CohCmdMsg;
        cohcmd->set_t(msg->t());
        cohcmd->set_opcode(msg->opcode());
        cohcmd->set_origin(ctxt.cc());
        cohcmd->set_addr(msg->addr());
        issue_msg_to_noc(ctxt, cl, cohcmd, dm->lookup(msg->addr()));

        // Set flags
        set_awaiting_cohend = true;
        set_awaiting_cmdrsp = true;

        // Set flag indicating we are awaiting the command response.

        // ACE command advances to active state; install entry within
        // transaction table.
        cl.push_back(CCOpcode::TransactionStart);
        // Consume and advance
        cl.next_and_do_consume(true);
      } break;
      default: {
        std::string name = "Unable to handle ACE command: ";
        name += to_string(msg->opcode());
        //        issue_invalid_state_transition(cl, name);
      } break;
    }

    if (set_awaiting_cohend) {
      // Set "Awaiting CohEnd" flag
      issue_line_update(ctxt, cl, LineUpdate::SetAwaitingCohEnd);
    }

    if (set_awaiting_cmdrsp) {
      // Set "Awaiting CohCmdRsp" flag
      issue_line_update(ctxt, cl, LineUpdate::SetAwaitingCohCmdRsp);
    }
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl,
                const CohEndMsg* msg) const {
    // Apply message to transaction state.
    issue_apply_msg(ctxt, cl, msg);
    // Consume and advance
    cl.next_and_do_consume(true);
  }

  void eval_msg(CCContext& ctxt, CCCommandList& cl, const DtMsg* msg) const {
    // Apply message to transaction state.
    issue_apply_msg(ctxt, cl, msg);

    // Issue Dt response to LLC/CC
    DtRspMsg* rsp = new DtRspMsg;
    rsp->set_t(msg->t());
    rsp->set_origin(ctxt.cc());
    issue_msg_to_noc(ctxt, cl, rsp, msg->origin());
    // Consume and advance
    cl.next_and_do_consume(true);
  }

  void eval_msg(CCSnpContext& ctxt, CCSnpCommandList& cl,
                const CohSnpMsg* msg) const {
    using snpcb = CCSnpCommandBuilder;

    // Forward snoop request to L2.
    AceSnpMsg* acesnp = Pool<AceSnpMsg>::construct();
    acesnp->set_t(msg->t());
    acesnp->set_opcode(msg->opcode());
    acesnp->set_addr(msg->addr());
    issue_msg_to_queue(CCEgressQueue::L2CmdQ, ctxt, cl, acesnp);

    SnpLine* snpline = static_cast<SnpLine*>(ctxt.tstate()->line());
    snpline->set_origin(msg->origin());
    snpline->set_agent(msg->agent());

    // Consume message
    cl.push_back(CCSnpOpcode::TransactionStart);
    cl.push_back(CCSnpOpcode::ConsumeMsg);
    cl.push_back(CCSnpOpcode::NextEpoch);
  }

  void eval_msg(CCSnpContext& ctxt, CCSnpCommandList& cl,
                const AceSnpRspMsg* msg) const {
    using snpcb = CCSnpCommandBuilder;
    // Snoop line
    SnpLine* snpline = static_cast<SnpLine*>(ctxt.tstate()->line());

    // Forward response back to originating directory.
    CohSnpRspMsg* rsp = new CohSnpRspMsg;
    rsp->set_t(msg->t());
    rsp->set_origin(ctxt.cc());
    rsp->set_dt(msg->dt());
    rsp->set_pd(msg->pd());
    rsp->set_is(msg->is());
    rsp->set_wu(msg->wu());
    issue_msg_to_noc(ctxt, cl, rsp, snpline->origin());

    if (msg->dt()) {
      // Data transfer, send data to requester.
      DtMsg* dt = new DtMsg;
      dt->set_t(msg->t());
      dt->set_origin(ctxt.cc());

      issue_msg_to_noc(ctxt, cl, dt, snpline->agent());
    }
    cl.push_back(CCSnpOpcode::ConsumeMsg);
    cl.push_back(CCSnpOpcode::NextEpoch);
  }

  void eval_msg(CCSnpContext& ctxt, CCSnpCommandList& cl,
                const DtRspMsg* msg) const {
    using snpcb = CCSnpCommandBuilder;
    cl.push_back(CCSnpOpcode::TransactionEnd);
    cl.push_back(CCSnpOpcode::ConsumeMsg);
    cl.push_back(CCSnpOpcode::NextEpoch);
  }

  void issue_apply_msg(CCContext& ctxt, CCCommandList& cl,
                       const Message* msg) const {
    Line* line = static_cast<Line*>(ctxt.line());
    ApplyMsgAction* action = new ApplyMsgAction(msg, line);
    cl.push_back(cb::from_action(action));
  }

  void issue_line_update(CCContext& ctxt, CCCommandList& cl,
                         LineUpdate update) const {
    Line* line = static_cast<Line*>(ctxt.line());
    LineUpdateAction* action = new LineUpdateAction(line, update);
    cl.push_back(cb::from_action(action));
  }

  template<typename CONTEXT, typename LIST>
  void issue_msg_to_queue(CCEgressQueue eq, CONTEXT& ctxt, LIST& cl,
                          const Message* msg) const {
    struct EmitMessageActionProxy : CCCoherenceAction {
      EmitMessageActionProxy() = default;

      std::string to_string() const override {
        KVListRenderer r;
        r.add_field("action", "emit message");
        r.add_field("mq", mq_->path());
        r.add_field("msg", msg_->to_string());
        return r.to_string();
      }
      
      // Setters
      void set_eq(CCEgressQueue eq) { eq_ = eq; }
      void set_mq(MessageQueueProxy* mq) { mq_ = mq; }
      void set_msg(const Message* msg) { msg_ = msg; }

      void set_resources(CCResources& r) const override {
        switch (eq_) {
          case CCEgressQueue::L2CmdQ: {
            r.set_cmd_q_n(r.cmd_q_n() + 1);
          } break;
          case CCEgressQueue::L2RspQ: {
            r.set_rsp_q_n(r.rsp_q_n() + 1);
          } break;
          default: {
            // No resource requirement
          } break;
        }
      }

      bool execute() override { return mq_->issue(msg_); };
      
     private:
      // Desintation Egress Queue
      CCEgressQueue eq_ = CCEgressQueue::Invalid;
      // Destination message queue
      MessageQueueProxy* mq_ = nullptr;
      // Message to issue
      const Message* msg_ = nullptr;
    };
    EmitMessageActionProxy* action = new EmitMessageActionProxy;
    action->set_eq(eq);
    action->set_msg(msg);
    MessageQueueProxy* mq = nullptr;
    switch (eq) {
      case CCEgressQueue::L2CmdQ: {
        mq = ctxt.cc()->cc_l2__cmd_q();
      } break;
      case CCEgressQueue::L2RspQ: {
        mq = ctxt.cc()->cc_l2__rsp_q();
      } break;
      default: {
        LogMessage lm("Unknown destination message queue: ");
        lm.append(to_string(eq));
        log(lm);
      } break;
    }
    action->set_mq(mq);
    cl.push_back(action);
  }

  template<typename CONTEXT, typename LIST>
  void issue_msg_to_noc(CONTEXT& ctxt, LIST& cl, const Message* msg,
                        Agent* dest) const {
    struct EmitMessageToNocAction : CCCoherenceAction {
      EmitMessageToNocAction() = default;

      std::string to_string() const override {
        KVListRenderer r;
        r.add_field("action", "emit message to noc");
        r.add_field("mq", mq_->path());
        r.add_field("msg", msg_->to_string());
        return r.to_string();
      }

      void set_mq(MessageQueueProxy* mq) { mq_ = mq; }
      void set_msg(const NocMsg* msg) { msg_ = msg; }
      void set_cc(const CCModel* cc) { cc_ = cc; }

      void set_resources(CCResources& r) const override {
        // Always require a NOC credit.
        r.set_noc_credit_n(r.noc_credit_n() + 1);

        const Agent* dest = msg_->dest();
        const Message* payload = msg_->payload();
        switch (payload->cls()) {
          case MessageClass::CohSrt: {
            // Require Coherence Start message resource.
            r.set_coh_srt_n(dest, r.coh_srt_n(dest) + 1);
          } break;
          case MessageClass::CohCmd: {
            // Require Coherence Command message resource.
            r.set_coh_cmd_n(dest, r.coh_cmd_n(dest) + 1);
          } break;
          case MessageClass::Dt: {
            // Require Data message resource.
            r.set_dt_n(dest, r.dt_n(dest) + 1);
          } break;
          default: {
            // No resources required;
            //
            // DtRsp, CohSnpRsp are assumed to either have resources
            // reserved upon issue of their originator commands (Dt,
            // CohSnp), or are otherwise guarenteed to make forward
            // progress.
          } break;
        }
      }

      bool execute() override {
        // If a credit counter exists for at the destination for the current
        // MessageClass, deduct one credit, otherwise ignore.
        const Message* payload = msg_->payload();
        // Lookup counter map for current message class.
        const auto& cntrs_map = cc_->ccntrs_map();
        if (auto ccntr_map_it = cntrs_map.find(payload->cls());
            ccntr_map_it != cntrs_map.end()) {
          // Lookup map to determine if a credit counter for the
          // destination agent is present.
          const CCModel::ccntr_map& agent_counter = ccntr_map_it->second;
          if (auto it = agent_counter.find(msg_->dest());
              it != agent_counter.end()) {
            // Credit counter is present, therefore deduct a credit before
            // message is issued.
            CreditCounter* cc = it->second;
            cc->debit();
          }
          // Otherwise, no credit counter can be found for the current
          // { MessageClass, Agent* } pair, therefore disregard.
        }
        // Issue message to queue.
        return mq_->issue(msg_);
      }

     private:
      // Message to issue to NOC.
      const NocMsg* msg_ = nullptr;
      // Destination Message Queue
      MessageQueueProxy* mq_ = nullptr;
      // Cache controller model
      const CCModel* cc_ = nullptr;
    };
    // Encapsulate message in NOC transport protocol.
    NocMsg* nocmsg = new NocMsg;
    nocmsg->set_t(msg->t());
    nocmsg->set_payload(msg);
    nocmsg->set_origin(ctxt.cc());
    nocmsg->set_dest(dest);
    // Issue Message Emit action.
    EmitMessageToNocAction* action =
        new EmitMessageToNocAction;
    action->set_mq(ctxt.cc()->cc_noc__msg_q());
    action->set_msg(nocmsg);
    action->set_cc(ctxt.cc());
    cl.push_back(action);
  }
};

}  // namespace

namespace cc::moesi {

//
//
CCProtocol* build_cc_protocol(kernel::Kernel* k) {
  return new MOESICCProtocol(k);
}

}  // namespace cc::moesi
