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

#include "noc.h"

#include <sstream>

#include "primitives.h"
#include "utility.h"

namespace cc {

//
//
NocMsg::NocMsg() : Message(MessageClass::Noc) {}

std::string NocMsg::to_string() const {
  using cc::to_string;

  std::stringstream ss;
  {
    KVListRenderer r(ss);
    r.add_field("cls", to_string(cls()));
    r.add_field("payload", payload()->to_string());
  }
  return ss.str();
}

//
//
class NocModel::MainProcess : public kernel::Process {
  enum class State { Idle, ChooseQueue, IssueMessage };

  static const char* to_string(State state) {
    switch (state) {
      case State::Idle:
        return "Idle";
      case State::ChooseQueue:
        return "ChooseQueue";
      case State::IssueMessage:
        return "IssueMessage";
      default:
        return "Invalid";
    }
  }

  struct Context {
    MQArbTmt t;
  };

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, NocModel* model)
      : kernel::Process(k, name), model_(model) {}

  State state() const { return state_; }
  void set_state(State state) { state_ = state; }

 private:
  void init() override {
    // Await the arrival of requesters
    MQArb* arb = model_->arb();
    set_state(State::Idle);
    wait_on(arb->request_arrival_event());
  }

  void eval() override {
    switch (state()) {
      case State::Idle: {
        handle_awaiting_message();
      } break;
      case State::ChooseQueue: {
        handle_choose_queue();
      } break;
      case State::IssueMessage: {
        handle_issue_message();
      } break;
      default: {
      } break;
    }
  }

  void fini() override {}

  void handle_awaiting_message() {
    // Idle state, awaiting more work.
    MQArb* arb = model_->arb();
    MQArbTmt t = arb->tournament();

    // Detect deadlock at L1Cache front-end. This occurs only in the
    // presence of a protocol violation and is therefore by definition
    // unrecoverable.
    if (t.deadlock()) {
      const LogMessage msg{"A protocol deadlock has been detected.",
                           Level::Fatal};
      log(msg);
    }

    if (t.has_requester()) {
      ctxt_ = Context();
      ctxt_.t = t;
      set_state(State::ChooseQueue);
      next_delta();
    } else {
      // Otherwise, block awaiting the arrival of a message at on
      // the of the message queues.
      wait_on(arb->request_arrival_event());
    }
  }

  void handle_choose_queue() {
    // Idle state, awaiting more work.
    MQArb* arb = model_->arb();

    ctxt_.t = arb->tournament();
    if (ctxt_.t.has_requester()) {
      set_state(State::IssueMessage);
      next_delta();
    }
  }

  void handle_issue_message() {
    MessageQueue* mq = ctxt_.t.winner();
    const Message* msg = mq->peek();
    switch (msg->cls()) {
      case MessageClass::Noc: {
        // Forward message to destination message queue after some
        // fixed delay.
        const NocMsg* nocmsg = static_cast<const NocMsg*>(msg);

        // Lookup destination port ingress queue.
        NocPort* port = model_->get_agent_port(nocmsg->dest());
        if (port == nullptr) {
          LogMessage lmsg("Unable to resolve destination port.");
          lmsg.level(Level::Fatal);
          log(lmsg);
        }

        // Issue message to destination agent ingress queue.
        // const Message* payload = nocmsg->payload();
        MessageQueue* egress = port->egress();
        egress->issue(nocmsg);

        // Message has now been issued to destination. Destroy
        // transport message and update arbitration.
        // nocmsg->release();
        mq->dequeue();

        // Advance arbitration state.
        ctxt_.t.advance();
      } break;
      default: {
        using cc::to_string;
        // The NOC can process only NOC-class commands as it is only a
        // conduit through which messages are passed. If some other
        // message arrives, this is a fatal error.
        LogMessage lmsg("Invalid message class recieved: ");
        lmsg.append(to_string(msg->cls()));
        lmsg.level(Level::Fatal);
        log(lmsg);
      } break;
    }

    // kernel::RequesterIntf<const Message*>* intf = ctxt_.t.intf();
    // issue_message(intf->dequeue());
    set_state(State::Idle);
    wait_for(kernel::Time{10, 0});
  }

  // Current context
  Context ctxt_;
  // Current state
  State state_;
  // Pointer to parent NocModel instance.
  NocModel* model_ = nullptr;
};

NocPort::NocPort(kernel::Kernel* k, const std::string& name) : Module(k, name) {
  build();
}

NocPort::~NocPort() { delete ingress_; }

void NocPort::build() {
  // Construct owned ingress queue; egress is owned by the agent itself.
  ingress_ = new MessageQueue(k(), "ingress", 3);
  add_child_module(ingress_);
}


class NocEndpoint::MainProcess : public AgentProcess {
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, NocEndpoint* ep)
      : AgentProcess(k, name), ep_(ep)
  {}

 private:

  // Initialization
  void init() override {
    MessageQueue* mq = ep_->ingress_mq();
    wait_on(mq->non_empty_event());
  }

  // Evaluation
  void eval() override {
    // Upon reception of a NOC message, remove transport layer
    // encapsulation and issue to the appropriate ingress queue.
    MessageQueue* mq = ep_->ingress_mq();
    const NocMsg* nocmsg = static_cast<const NocMsg*>(mq->dequeue());

    // Validate message
    if (nocmsg->cls() != MessageClass::Noc) {
      LogMessage lmsg("Received invalid message class: ");
      lmsg.append(to_string(nocmsg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    const Message* msg = nocmsg->payload();

    MessageQueueProxy* proxy = ep_->lookup_mq(msg);
    if (proxy == nullptr) {
      LogMessage lmsg("Message queue not found for class: ");
      lmsg.append(cc::to_string(msg->cls()));
      lmsg.level(Level::Fatal);
      log(lmsg);
    }

    // Forward message message to destination queue and discard
    // encapsulation/transport message.
    proxy->issue(msg);
    nocmsg->release();

    // Set conditions for subsequent re-evaluations.
    if (!mq->empty()) {
      // TODO:Cleanup
      // Wait some delay
      wait_for(kernel::Time{10, 0});
    } else {
      // Not further work; await until noc ingress queue becomes non-full.
      wait_on(mq->non_empty_event());
    }
  }

  NocEndpoint* ep_ = nullptr;
};


NocEndpoint::NocEndpoint(kernel::Kernel* k, const std::string& name)
    : Agent(k, name) {
  build();
}

NocEndpoint::~NocEndpoint() {
  delete main_;
  // proxies are not owned by the end-point as they may be duplicated.
}

void NocEndpoint::build() {
  ingress_mq_ = new MessageQueue(k(), "ingress_mq", 3);
  add_child_module(ingress_mq_);
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

NocModel::NocModel(kernel::Kernel* k, const NocModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

NocPort* NocModel::get_agent_port(Agent* agent) {
  NocPort* port = nullptr;
  std::map<Agent*, NocPort*>::iterator it = ports_.find(agent);
  if (it != ports_.end()) {
    port = it->second;
  }
  return port;
}

void NocModel::build() {
  // Construct ingress selection aribter
  arb_ = new MQArb(k(), "arb");
  add_child_module(arb_);

  // Construct main process
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

void NocModel::register_agent(Agent* agent) {
  const std::string port_name = agent->name();
  NocPort* port = new NocPort(k(), port_name);
  add_child_module(port);
  // Install in port table.
  ports_.insert(std::make_pair(agent, port));
}

void NocModel::elab() {
  for (std::pair<Agent*, NocPort*> pp : ports_) {
    NocPort* port = pp.second;
    arb_->add_requester(port->ingress());
  }
}

void NocModel::drc() {
  for (std::pair<Agent*, NocPort*> pp : ports_) {
    NocPort* port = pp.second;
    if (port->egress() == nullptr) {
      Agent* agent = pp.first;
      LogMessage msg("Egress port has not been bound for agent: ");
      msg.append(agent->path());
      msg.level(Level::Fatal);
      log(msg);
    }
  }
}

}  // namespace cc
