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

#include "l1cache.h"

#include "cache.h"
#include "cpu.h"
#include "l2cache.h"
#include "msg.h"
#include "primitives.h"
#include "protocol.h"
#include "utility.h"

// #define VERBOSE_LOGGING

namespace cc {

const char* to_string(L1CacheOpcode opcode) {
  switch (opcode) {
    case L1CacheOpcode::CpuLoad:
      return "CpuLoad";
    case L1CacheOpcode::CpuStore:
      return "CpuStore";
    default:
      return "Invalid";
  }
}

L1CmdMsg::L1CmdMsg() : Message(MessageClass::L1Cmd) {}

std::string L1CmdMsg::to_string() const {
  using cc::to_string;

  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
    r.add_field("opcode", to_string(opcode()));
    Hexer h;
    r.add_field("addr", h.to_hex(addr()));
  }
  return ss.str();
}

//
//
L1CmdRspMsg::L1CmdRspMsg() : Message(MessageClass::L1CmdRsp) {}

//
//
std::string L1CmdRspMsg::to_string() const {
  using cc::to_string;

  std::stringstream ss;
  {
    KVListRenderer r(ss);
    r.add_field("cls", to_string(cls()));
  }
  return ss.str();
}

const char* to_string(L1Opcode opcode) {
  switch (opcode) {
#define __declare_to_string(__name)             \
    case L1Opcode::__name: return #__name;
    L1OPCODE_LIST(__declare_to_string)
#undef __declare_to_string
    default: return "Invalid";
  }
}

L1CacheContext::~L1CacheContext() {
  if (owns_line()) { delete line_; }
}

L1Command::~L1Command() {
  switch (opcode()){
    case L1Opcode::InvokeCoherenceAction: {
      oprands.coh.action->release();
    } break;
    default: {
    } break;
  }
}

L1Command* L1CommandBuilder::from_opcode(L1Opcode opcode) {
  return new L1Command(opcode);
}

L1Command* L1CommandBuilder::from_action(CoherenceAction* action) {
  L1Command* cmd = new L1Command(L1Opcode::InvokeCoherenceAction);
  cmd->oprands.coh.action = action;
  return cmd;
}

L1CommandList::~L1CommandList() {
  for (L1Command* cmd : cmds_) {
    cmd->release();
  }

}

void L1CommandList::push_back(L1Command *cmd) {
  cmds_.push_back(cmd);
}

class L1CommandInterpreter {
 public:
  L1CommandInterpreter(L1CacheModel* model, kernel::Process* process)
      : model_(model), process_(process) {}

  void execute(const L1Command* c) {
    switch (c->opcode()) {
      default: {
      } break;
#define __declare_dispatcher(__name)                    \
      case L1Opcode::__name: execute##__name(c); break;
      L1OPCODE_LIST(__declare_dispatcher)
#undef __declare_dispatcher
    }
  }

 private:
  void executeTableInstall(const L1Command* cmd) const {
    //    L1TTable* tt = model_->tt();
    //    L1TState* st = new L1TState();
    //    st->set_line(c.line());
    //    tt->install(c.msg()->t(), st);
  }

  void executeTableGetCurrentState(const L1Command* cmd) const {
    // Lookup the Transaction Table for the current transaction
    // and set the table pointer for subsequent operations.
    /*
    L1TTable* tt = model_->tt();
    table_it = tt->find(c.msg()->t());
    if (table_it == tt->end()) {
      LogMessage msg("Transaction not present in transaction table!");
      msg.level(Level::Fatal);
      log(msg);
    }
    */
  }

  void executeTableRemove(const L1Command* cmd) const {
    // Remove the current Transsaction context from the
    // Transaction Table.
    /*
    L1TState *st = table_it->second;
    st->release();
    L1TTable* tt = model_->tt();
    tt->remove(table_it);
    */
  }

  void executeTableMqUnblockAll(const L1Command* cmd) const {
    // For the current table context, unblock all messages
    // queues that are currently blocked awaiting completion of
    // the current transaction.
    /*
    L1TState *st = table_it->second;
    for (MessageQueue* mq : st->mq()) {
      mq->set_blocked(false);
    }
    */
  }

  void executeTableMqAddToBlockedList(const L1Command* cmd) const {
    // Add the currently address Message Queue to the set of
    // queues blocked on the current transaction.
    /*
    L1TState *st = table_it->second;
    st->add_blocked_mq(c.mq());
    */
  }

  void executeWaitOnMsg(const L1Command* cmd) const {
    // Set wait state of current process; await the arrival of a
    // new message.
    /*
    MQArb* arb = model_->arb();
    wait_on(arb->request_arrival_event());
    */
  }

  void executeWaitNextEpochOrWait(const L1Command* cmd) const {
  /*
    MQArb* arb = model_->arb();
    MQArbTmt t = arb->tournament();
    if (t.has_requester()) {
      // Wait some delay
      wait_for(kernel::Time{10, 0});
    } else {
      // No further commands, block process until something
      // arrives.
      wait_on(arb->request_arrival_event());
    }
  */
  }

  void executeMqSetBlocked(const L1Command* cmd) const {
  /*
    // Set the blocked status of the current Message Queue.
    c.mq()->set_blocked(true);
  */
  }

  void executeMsgConsume(const L1Command* cmd) const {
  /*
    // Dequeue and release the head message of the currently
    // addressed Message Queue.
    const Message* msg = c.mq()->dequeue();
    msg->release();
    t_.advance();
  */
  }

  void executeMsgL1CmdExtractAddr(const L1Command* cmd) const {
  /*
    // Extract address field from command message.
    const L1CmdMsg* cmd = static_cast<const L1CmdMsg*>(c.msg());
    addr = cmd->addr();
  */
  }

  void executeInstallLine(const L1Command* cmd) const {
  /*
    // Install line within the current context into the cache at
    // an appropriate location. Expects that any prior evictions
    // to thg destination set have already taken place.
    L1Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    L1CacheSet set = cache->set(ah.set(addr));
    L1Cache::Evictor evictor;
    if (const std::pair<L1CacheLineIt, bool> p = evictor.nominate(
            set.begin(), set.end()); !p.second) {
      set.install(p.first, ah.tag(addr), c.line());
      c.set_owns_line(false);
    } else {
      LogMessage msg("Cannot install line as set is full.");
      msg.level(Level::Fatal);
      log(msg);
    }
  */
  }

  void executeInvokeCoherenceAction(const L1Command* cmd) const {
  /*
    CoherenceActions* actions = c.oprands.coh.actions;
    actions->execute();
  */
  }


  struct {
    addr_t addr;
    
  } context_;

  //
  kernel::Process* process_ = nullptr;
  //
  L1CacheModel* model_ = nullptr;
};

//
//
class L1CacheModel::MainProcess : public kernel::Process {
  using cb = L1CommandBuilder;
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model) {}

 private:
  // Initialization
  void init() override {
    L1CacheContext c;
    L1CommandList cl;
    cl.push_back(cb::from_opcode(L1Opcode::WaitOnMsg));
    execute(c, cl);
  }

  // Evaluation
  void eval() override {
    MQArb* arb = model_->arb();
    t_ = arb->tournament();

    // Construct and initialize current processing context.
    L1CommandList cl;
    L1CacheContext c;
    c.set_l1cache(model_);

    // Check if requests are present, if not block until a new message
    // arrives at the arbiter. Process should ideally not wake in the
    // absence of requesters.
    if (!t_.has_requester()) {
      cl.push_back(cb::from_opcode(L1Opcode::WaitOnMsg));
      execute(c, cl);
      return;
    }

    // Fetch nominated message queue
    c.set_mq(t_.winner());

    // Dispatch to appropriate handler based upon message class.
    switch (c.msg()->cls()) {
      case MessageClass::L1Cmd: process_l1cmd(c, cl); break;
      case MessageClass::L2CmdRsp: process_l2cmdrsp(c, cl); break;
      default: {
        LogMessage lmsg("Invalid message class received: ");
        lmsg.append(cc::to_string(c.msg()->cls()));
        lmsg.level(Level::Error);
        log(lmsg);
      } break;
    }

    if (can_execute(cl)) { execute(c, cl); }
  }

 private:
  void process_l1cmd(L1CacheContext& c, L1CommandList& cl) const {
    const L1CmdMsg* cmd = static_cast<const L1CmdMsg*>(c.msg());
    L1Cache* cache = model_->cache();
    const CacheAddressHelper ah = cache->ah();
    const L1CacheModelProtocol* protocol = model_->protocol();

    // Otherwise, no transactions to current line in flight, therefore
    // query the cache to determine hit/miss status of line_id.
    L1CacheSet set = cache->set(ah.set(cmd->addr()));
    if (L1CacheLineIt it = set.find(ah.tag(cmd->addr())); it == set.end()) {
      // Line for current address has not been found in the set,
      // therefore a new line must either be installed or, if the set
      // is currently full, another line must be nominated and
      // evicted.
      L1Cache::Evictor evictor;
      if (const std::pair<L1CacheLineIt, bool> p = evictor.nominate(
              set.begin(), set.end()); p.second) {
        // TODO
      } else {
        c.set_line(protocol->construct_line());
        c.set_owns_line(true);
        protocol->apply(c, cl);
      }
    } else {
      // Line is present in the cache, apply state update.
      c.set_line(it->t());
      protocol->apply(c, cl);
    }
  }

  void process_l2cmdrsp(L1CacheContext& c, L1CommandList& cl) const {
    Transaction* t = c.msg()->t();
    L1TTable* tt = model_->tt();
    if (L1TTable::iterator it = tt->find(t); it != tt->end()) {
      const L1CacheModelProtocol* protocol = model_->protocol();
      const L1TState* st = it->second;
      c.set_line(st->line());
      protocol->apply(c, cl);
    } else {
      LogMessage lm("Cannot find transaction table entry for ");
      lm.append(to_string(t));
      lm.level(Level::Fatal);
      log(lm);
    }
  }

  bool can_execute(L1CommandList& cl) const {
    return true;
  }

  void execute(L1CacheContext& c, const L1CommandList& cl) {
    L1TTable::iterator table_it;
    addr_t addr;

    L1CommandInterpreter interpreter(model_, this);
    for (const L1Command* cmd : cl) {
      LogMessage lm("Executing opcode: ");
      //lm.append(to_string(opcode));
      lm.level(Level::Debug);
      log(lm);
      
      interpreter.execute(cmd);
      cmd->release();
    }
  }

  // Current arbiter Tournament
  MQArbTmt t_;
  // Pointer to parent module.
  L1CacheModel* model_ = nullptr;
};

L1CacheModel::L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

L1CacheModel::~L1CacheModel() {
  delete cpu_l1__cmd_q_;
  delete l2_l1__rsp_q_;
  delete arb_;
  delete tt_;
  delete main_;
  delete cache_;
}

void L1CacheModel::build() {
  // Construct command request queue
  cpu_l1__cmd_q_ = new MessageQueue(k(), "cpu_l1__cmd_q", 16);
  add_child_module(cpu_l1__cmd_q_);
  // Construct L2 -> L1 response queue
  l2_l1__rsp_q_ = new MessageQueue(k(), "l2_l1__rsp_q", 16);
  add_child_module(l2_l1__rsp_q_);
  // Arbiter
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);
  // Transaction table.
  tt_ = new L1TTable(k(), "tt", 16);
  add_child_module(tt_);
  // Main thread of execution
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
  //
  cache_ = new L1Cache(config_.cconfig);
  // Set up protocol
  protocol_ = config_.pbuilder->create_l1(k());
  add_child_module(protocol_);
}

void L1CacheModel::elab() {
  arb_->add_requester(cpu_l1__cmd_q_);
  arb_->add_requester(l2_l1__rsp_q_);
}

void L1CacheModel::drc() {
  if (l1_l2__cmd_q_ == nullptr) {
    LogMessage msg("L2 message queue has not been bound.");
    msg.level(Level::Fatal);
    log(msg);
  }
  if (l1_cpu__rsp_q_ == nullptr) {
    LogMessage msg("L1 to CPU response message queue.");
    msg.level(Level::Fatal);
    log(msg);
  }
  if (cpu_ == nullptr) {
    // Error: CPU has not been bound.
  }
  if (l2c_ == nullptr) {
    // Error: L2C has not been bound.
  }
}

}  // namespace cc
