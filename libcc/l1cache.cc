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

#include "cc/l1cache.h"
#include "cc/sim.h"

namespace cc {

class L1CacheModel::MainProcess : public kernel::Process {
  enum class Consequence {
    Invalid,
    Blocked,
    Consume,
    Discard
  };

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L1CacheModel* model)
      : kernel::Process(k, name), model_(model)
  {}

  Arbiter<const Message*>* arb() const { return model_->arb_; }

  // Initialization
  void init() override {
    // Await the arrival of requesters
    wait_on(arb()->wake_event());
  }

  // Finalization
  void fini() override {
  }

  // Evaluation
  void eval() override {
    if (!pmsgs_.empty()) {
      try_emit_message();
    } else {
      handle_new_message();
    }
  }
 private:
  void try_emit_message() {
  }
  void handle_new_message() {
    Arbiter<const Message*>::Tournament t = arb()->tournament();

    // Detect deadlock at L1Cache front-end. This occurs only in the
    // presence of a protocol violation and is therefore by definition
    // unrecoverable.
    if (t.deadlock()) {
      const LogMessage msg{"A protocol deadlock has been detected.", Level::Fatal};
      log(msg);
    }

    // Detect whether requesters are available. Ideally, this block
    // should never have been invoked if there are no requesters, but
    // incase it has block and await new events.
    if (!t.has_requester()) {
      wait_on(arb()->wake_event());
      return;
    }
    const Message* msg = intf->peek();
    bool do_destruct = false;

    LogMessage lmsg{"Process "};
    lmsg.append(msg->to_string_short());
    switch (handle_message(msg, lmsg)) {
      case Consequence::Blocked: {
        // Block message; cannot advance against current state.
        intf->set_blocked(true);
        // Incur pipeline flush penalty.
        // TODO:
        const L1CacheModelConfig& config = model_->config();
        // wait(clk.rising_edge_event(), config.ldst_flush_penalty_n);
      } break;
      case Consequence::Consume: {
        // Consume message; action is complete.
        intf->dequeue();
        do_destruct = true;
        // wait(clk.rising_edge_event());
      } break;
      case Consequence::Discard: {
        // Throw message away.
        intf->dequeue();
        do_destruct = true;
        // wait(clk.rising_edge_event());
      } break;
      default: {
      } break;
    }
    // Emit status information.
    log(lmsg);
    // Destroy message if complete.
    if (do_destruct) { delete msg; }
    // Advance arbitration state.
    t.advance();
  }
  Consequence handle_message(const Message* msg, LogMessage& lmsg) {
    Consequence c{Consequence::Invalid};
    switch (msg->cls()) {
      case Message::Cpu: {


        // Consume message.
        intf->dequeue();
      } break;
      default: {
        // Invalid message has been received; cannot proceed. Error out.
        lmsg.append("; Invalid message received: ");
        lmsg.level(Level::Error);
        lmsg.append(msg->to_string());
        c = Consequence::Discard;
      } break;
    }
    return c;
  }

  // Pointer to parent module.
  L1CacheModel* model_ = nullptr;
  // Pending message queue
  std::vector<const Message*> pmsgs_;
};

L1CacheModel::L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config)
    : kernel::Module(k, config.name), config_(config) {
  build();
}

L1CacheModel::~L1CacheModel() {
}

void L1CacheModel::build() {
  // Capture stimulus;
  proc_ = new ProcessorModel(k(), "cpu");
  proc_->set_stimulus(config_.stim);
  add_child(proc_);

  // Construct queues: TBD
  MessageQueue* mq = new MessageQueue(k(), "cmdq", 16);
  add_child(mq);
  mqs_.push_back(mq);

  // Arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  arb_->add_requester(proc_);
  for (MessageQueue* mq : mqs_) {
    arb_->add_requester(mq);
  }
  add_child(arb_);

  // Main thread of execution
  main_ = new MainProcess(k(), "main", this);
  add_child(main_);
}

void L1CacheModel::elab() {
  // Do elaborate
}

void L1CacheModel::drc() {
  if (mqs_.empty()) {
    const LogMessage msg{"L1Cache has no message queues.", Level::Fatal};
    log(msg);
  }
}

} // namespace cc
