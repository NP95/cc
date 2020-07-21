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

#include "ccntrl.h"

#include "amba.h"
#include "dir.h"
#include "msg.h"
#include "noc.h"
#include "primitives.h"
#include "protocol.h"

namespace cc {

const char* to_string(CCWait w) {
  switch (w) {
    case CCWait::Invalid:
      return "Invalid";
    case CCWait::MsgArrival:
      return "MsgArrival";
    case CCWait::NextEpochIfHasRequestOrWait:
      return "NextEpochIfHasRequestOrWait";
    default:
      return "Unknown";
  }
}

//
//
class CC::NocIngressProcess : public kernel::Process {
 public:
  NocIngressProcess(kernel::Kernel* k, const std::string& name, CC* cc)
      : Process(k, name), cc_(cc) {}

  // Initialization
  void init() override {
    MessageQueue* mq = cc_->noc_cc__msg_q();
    wait_on(mq->request_arrival_event());
  }

  // Evaluation
  void eval() override {
    using cc::to_string;

    // Upon reception of a NOC message, remove transport layer
    // encapsulation and issue to the appropriate ingress queue.
    MessageQueue* noc_mq = cc_->noc_cc__msg_q();
    const NocMsg* nocmsg = static_cast<const NocMsg*>(noc_mq->dequeue());

    // Validate message
    if (nocmsg->cls() != MessageClass::Noc) {
      LogMessage lmsg("Received invalid message class: ");
      lmsg.append(to_string(nocmsg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    const Message* msg = nocmsg->payload();
    MessageQueue* iss_mq = cc_->lookup_rdis_mq(msg->cls());
    if (iss_mq == nullptr) {
      LogMessage lmsg("Message queue not found for class: ");
      lmsg.append(to_string(msg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    // Forward message message to destination queue and discard
    // encapsulation/transport message.
    iss_mq->push(msg);
    nocmsg->release();

    // Set conditions for subsequent re-evaluations.
    if (!noc_mq->empty()) {
      // TODO:Cleanup
      // Wait some delay
      wait_for(kernel::Time{10, 0});
    } else {
      // Not further work; await until noc ingress queue becomes non-full.
      wait_on(noc_mq->request_arrival_event());
    }
  }

 private:
  CC* cc_ = nullptr;
};

//
//
class TAddrFinder {
 public:
  TAddrFinder(CCTTable::iterator begin, CCTTable::iterator end)
      : begin_(begin), end_(end) {}

  // Search the table for a transaction to the current line. Return true
  // if a corresponding transaction has been found.
  bool has_transaction(line_id_t line_id) {
    // TODO
    return false;
  }

 private:
  CCTTable::iterator begin_, end_;
};

//
//
class CC::RdisProcess : public kernel::Process {
 public:
  RdisProcess(kernel::Kernel* k, const std::string& name, CC* cc)
      : Process(k, name), cc_(cc) {}

 private:
  // Initialization
  void init() override {
    CCContext c;
    c.set_wait(CCWait::MsgArrival);
    wait_on_context(c);
  }

  // Evaluation
  void eval() override {
    MQArb* arb = cc_->arb();
    MQArbTmt t = arb->tournament();

    CCContext c;
    c.set_cc(cc_);
    c.set_t(t);

    // Check if requests are present, if not block until a new message
    // arrives at the arbiter. Process should ideally not wake in the
    // absence of requesters.
    if (!t.has_requester()) {
      c.set_wait(CCWait::MsgArrival);
      wait_on_context(c);
      return;
    }

    // Fetch nominated message.
    const Message* msg = t.winner()->peek();
    c.set_mq(t.winner());
    c.set_msg(msg);

    switch (c.msg()->cls()) {
      case MessageClass::AceCmd: {
        eval_msg(c, static_cast<const AceCmdMsg*>(c.msg()));
      } break;
      case MessageClass::CohEnd: {
        CCTState* st = lookup_tt(c.msg()->t());
        c.set_st(st);
        eval_msg(c, static_cast<const CohEndMsg*>(c.msg()));
      } break;
      default: {
        using cc::to_string;
        LogMessage lmsg("Invalid message class received: ");
        lmsg.append(to_string(c.msg()->cls()));
        lmsg.level(Level::Error);
        log(lmsg);
      } break;
    }
    // If command commits, execute compute action list.
    if (c.commits()) {
      execute(c);
    }
    // Apply context's wait condition to processes wait-state.
    wait_on_context(c);
  }

  void eval_msg(CCContext& c, const AceCmdMsg* msg) {
    CCTTable* tt = cc_->table();
    TAddrFinder af(tt->begin(), tt->end());
    if (tt->full() || af.has_transaction(msg->addr())) {
      // Transaction table is either full, or there is already
      // a transaction in flight for the current line.
      c.mq()->set_blocked(true);
      return;
    }
    // Otherwise, install a new transaction entry table and proceed
    const CCProtocol* protocol = cc_->protocol();
    protocol->install(c);
  }

  CCTState* lookup_tt(Transaction* t) {
    CCTTable* tt = cc_->table();
    CCTTable::iterator it = tt->find(t);
    if (it == tt->end()) {
      LogMessage msg("Cannot find entry in transaction table.");
      msg.level(Level::Fatal);
      log(msg);
      return nullptr;
    }
    return it->second;
  }

  void eval_msg(CCContext& c, const CohEndMsg* msg) {
    const CCProtocol* protocol = cc_->protocol();
    protocol->apply(c);
  }

  void execute(CCContext& c) {
    // Current action (command issue to L2 or consequent eviction)
    // commits, therefore install entry in the transaction table.
    for (CoherenceAction* a : c.actions()) {
      a->execute();
    }
    update_ttable(c);
    MQArbTmt t = c.t();
    if (c.dequeue()) {
      t.winner()->dequeue();
      t.advance();
    }
  }

  void update_ttable(CCContext& c) {
    CCTTable* tt = cc_->table();
    Transaction* t = c.msg()->t();
    if (c.ttadd()) {
      // Install new entry in transaction table.
      CCTState* st = new CCTState;
      st->set_msg(c.msg());
      st->set_line(c.line());
      c.set_owns_line(false);
      tt->install(t, st);
    } else if (c.ttdel()) {
      // Delete entry from table.
      CCTTable::iterator it = tt->find(t);
      if (it == tt->end()) {
        LogMessage msg("Transaction not present in transaction table!");
        msg.level(Level::Fatal);
        log(msg);
      }
      CCTState* st = it->second;
      for (MessageQueue* mq : st->bmqs()) mq->set_blocked(false);
      st->release();
      tt->remove(it);
    }
  }

  void wait_on_context(const CCContext& c) {
    switch (c.wait()) {
      case CCWait::MsgArrival: {
        MQArb* arb = cc_->arb();
        wait_on(arb->request_arrival_event());
      } break;
      case CCWait::NextEpochIfHasRequestOrWait: {
        MQArb* arb = cc_->arb();
        MQArbTmt t = arb->tournament();
        if (t.has_requester()) {
          // Wait some delay
          wait_for(kernel::Time{10, 0});
        } else {
          // No further commands, block process until something
          // arrives.
          wait_on(arb->request_arrival_event());
        }
      } break;
      default: {
        LogMessage msg("Invalid wait condition: ");
        msg.append(to_string(c.wait()));
        msg.level(Level::Fatal);
        log(msg);
      } break;
    }
  }

  // Cache controller instance.
  CC* cc_ = nullptr;
};

CC::CC(kernel::Kernel* k, const CCConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

CC::~CC() {
  delete l2_cc__cmd_q_;
  delete noc_cc__msg_q_;
  delete dir_cc__rsp_q_;
  delete cc__dt_q_;
  delete arb_;
  delete rdis_proc_;
  delete noci_proc_;
  delete tt_;
  delete protocol_;
}

void CC::build() {
  // Construct L2 to CC command queue
  l2_cc__cmd_q_ = new MessageQueue(k(), "l2_cc__cmd_q", 3);
  add_child_module(l2_cc__cmd_q_);
  // NOC -> CC msg queue.
  noc_cc__msg_q_ = new MessageQueue(k(), "noc_cc__msg_q", 3);
  add_child_module(noc_cc__msg_q_);
  // DIR -> CC response queue
  dir_cc__rsp_q_ = new MessageQueue(k(), "dir_cc__rsp_q", 3);
  add_child_module(dir_cc__rsp_q_);
  //
  cc__dt_q_ = new MessageQueue(k(), "cc__dt_q", 3);
  add_child_module(cc__dt_q_);
  // Arbiteer
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Dispatcher process
  rdis_proc_ = new RdisProcess(k(), "rdis_proc", this);
  add_child_process(rdis_proc_);
  //
  noci_proc_ = new NocIngressProcess(k(), "noci_proc", this);
  add_child_process(noci_proc_);
  // Transaction table
  tt_ = new CCTTable(k(), "tt", 16);
  add_child_module(tt_);
  // Create protocol instance
  protocol_ = config_.pbuilder->create_cc();
}

//
//
void CC::elab() {
  // Add ingress queues to arbitrator.
  arb_->add_requester(l2_cc__cmd_q_);
  arb_->add_requester(dir_cc__rsp_q_);
  arb_->add_requester(cc__dt_q_);
}

//
//
void CC::drc() {
  if (dm() == nullptr) {
    // The Dir Mapper object computes the host directory for a
    // given address. In a single directory system, this is a basic
    // mapping to a single directory instance, but in more performant
    // systems this may some non-trivial mapping to multiple home
    // directories.
    LogMessage msg("Directory mapper is not defined.", Level::Warning);
    log(msg);
  }
}

//
//
MessageQueue* CC::lookup_rdis_mq(MessageClass cls) const {
  switch (cls) {
    case MessageClass::Dt:
      return cc__dt_q_;
    case MessageClass::L2Cmd:
      return l2_cc__cmd_q_;
    case MessageClass::CohEnd:
      return dir_cc__rsp_q_;
    case MessageClass::CohCmdRsp:
      return dir_cc__rsp_q_;
    default:
      return nullptr;
  }
}

}  // namespace cc
