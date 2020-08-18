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

  KVListRenderer r;
  r.add_field("cls", to_string(cls()));
  r.add_field("payload", payload()->to_string());
  return r.to_string();
}

//
//
class NocModel::MainProcess : public AgentProcess {
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, NocModel* model)
      : AgentProcess(k, name), model_(model) {}

 private:
  void init() override {
    // Await the arrival of requesters
    wait_on(model_->arb()->request_arrival_event());
  }

  void eval() override {
    MQArb* arb = model_->arb();
    MQArbTmt t = arb->tournament();

    if (!t.has_requester()) {
      wait_on(arb->request_arrival_event());
      return;
    }

    // Detect deadlock at L1Cache front-end. This occurs only in the
    // presence of a protocol violation and is therefore by definition
    // unrecoverable.
    if (t.deadlock()) {
      LogMessage msg("A protocol deadlock has been detected.");
      msg.level(Level::Fatal);
      log(msg);
    }
    MessageQueue* mq = t.winner();
    const Message* msg = mq->peek();
    switch (msg->cls()) {
      case MessageClass::Noc: {
        // Forward message to destination message queue after some
        // fixed delay.
        const NocMsg* nocmsg = static_cast<const NocMsg*>(msg);

        // Lookup origin port for message
        NocPort* origin_port = model_->get_agent_port(nocmsg->origin());

        // Lookup destination port ingress queue.
        NocPort* dest_port = model_->get_agent_port(nocmsg->dest());
        if (dest_port == nullptr) {
          LogMessage lmsg("Unable to resolve destination port.");
          lmsg.level(Level::Fatal);
          log(lmsg);
        }

        // Issue message to destination agent ingress queue.
        MessageQueue* egress = dest_port->egress();

        const NocTimingModel* tm = model_->tm();
        const time_t cost = tm->cost(nocmsg->origin(), nocmsg->dest());
        egress->issue(nocmsg, cost);

        // Return credit back to Ingress port
        CreditCounter* cc = origin_port->ingress_cc();
        cc->credit();

        // Message has now been issued to destination. Destroy
        // transport message and update arbitration.
        mq->dequeue();

        // Advance arbitration state.
        t.advance();
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
    wait_epoch();
  }

  // Pointer to parent NocModel instance.
  NocModel* model_ = nullptr;
};

NocPort::NocPort(kernel::Kernel* k, const std::string& name) : Module(k, name) {
  build();
}

NocPort::~NocPort() {
  delete ingress_;
  delete ingress_cc_;
}

void NocPort::build() {
  // Construct owned ingress queue; egress is owned by the agent itself.
  ingress_ = new MessageQueue(k(), "ingress", 10);
  add_child_module(ingress_);
  // Credit counter denoting Ingress Queue capacity.
  ingress_cc_ = new CreditCounter(k(), "ingress_cc");
  ingress_cc_->set_n(ingress_->n());
  add_child_module(ingress_cc_);
}

class NocEndpoint::MainProcess : public AgentProcess {
 public:
  MainProcess(kernel::Kernel* k, const std::string& name, NocEndpoint* ep)
      : AgentProcess(k, name), ep_(ep) {}

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

    MessageQueue* proxy = ep_->lookup_mq(msg);
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
      // Wait for an epoch (cycle)/
      wait_epoch();
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
  delete ingress_mq_;
  delete main_;
}

void NocEndpoint::build() {
  ingress_mq_ = new MessageQueue(k(), "ingress_mq", 10);
  add_child_module(ingress_mq_);
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);
}

bool NocEndpoint::elab() {
  main_->set_epoch(epoch_);
  return false;
}

// Lookup cost for origin -> dest edge
time_t NocTimingModel::cost(const Agent* origin, const Agent* dest) const {
  time_t delay = base();
  if (auto it = timing_.find(origin); it != timing_.end()) {
    if (auto jt = it->second.find(dest); jt != it->second.end()) {
      delay = jt->second;
    }
  }
  return delay;
}

void NocTimingModel::register_edge(const Agent* origin, const Agent* dest,
                                   time_t delay) {
  timing_[origin][dest] = delay;
}

NocModel::NocModel(kernel::Kernel* k, const NocModelConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

NocModel::~NocModel() {
  delete arb_;
  delete main_;
  for (auto pp : ports_) {
    delete pp.second;
  }
  delete tm_;
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
  // Name port as the 'flatten' path to the agent; otherwise,
  // duplicates may exist when agents with the same name, but
  // different paths, are registers with the NOC.
  const std::string port_name = flatten_path(agent->path());
  NocPort* port = new NocPort(k(), port_name);
  add_child_module(port);
  // Install in port table.
  ports_.insert(std::make_pair(agent, port));
}

bool NocModel::elab() {
  for (std::pair<Agent*, NocPort*> pp : ports_) {
    NocPort* port = pp.second;
    arb_->add_requester(port->ingress());
  }
  return false;
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
