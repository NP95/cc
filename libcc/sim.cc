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

#include "sim.h"

#include "msg.h"
#include "protocol.h"

namespace cc {

MessageQueue::MessageQueue(kernel::Kernel* k, const std::string& name,
                           std::size_t n)
    : kernel::Module(k, name) {
  build(n);
}

bool MessageQueue::issue(const Message* msg, kernel::Time t) {
  struct EnqueueAction : kernel::Action {
    EnqueueAction(kernel::Kernel* k, MessageQueue* mq, const Message* msg)
        : Action(k, "enqueue_action"), mq_(mq), msg_(msg) {}
    bool eval() override {
      mq_->push(msg_);
      return true;
    }

   private:
    MessageQueue* mq_ = nullptr;
    const Message* msg_;
  };

  if (full()) return false;

    // Log message issue:
#if 0
  LogMessage lmsg("Issue Message (dst: ");
  lmsg.append(path());
  lmsg.append("): ");
  lmsg.append(msg->to_string());
  lmsg.level(Level::Debug);
  log(lmsg);
#endif
  // Issue action:
  const kernel::Time execute_time = k()->time() + t;
  k()->add_action(execute_time, new EnqueueAction(k(), this, msg));

  return true;
}

void MessageQueue::push(const Message* msg) {
  if (!q_->enqueue(msg)) {
    LogMessage lmsg("Attempt to push new message to full queue.", Level::Error);
    log(lmsg);
  }
}

bool MessageQueue::has_req() const { return !q_->empty(); }

const Message* MessageQueue::peek() const {
  const Message* msg = nullptr;
  if (!q_->peek(msg)) {
    const LogMessage lmsg("Attempt to access empty queue.", Level::Fatal);
    log(lmsg);
  }
  return msg;
}

const Message* MessageQueue::dequeue() {
  const Message* msg = nullptr;
  if (!q_->dequeue(msg)) {
    const LogMessage lmsg("Attempt to dequeue Message failed.", Level::Fatal);
    log(lmsg);
    return nullptr;
  }
  return msg;
}

kernel::Event& MessageQueue::request_arrival_event() {
  return q_->non_empty_event();
}

void MessageQueue::build(std::size_t n) {
  q_ = new Queue<const Message*>(k(), "queue", n);
  add_child_module(q_);
}

Agent::Agent(kernel::Kernel* k, const std::string& name) : Module(k, name) {}

std::string to_string(const Agent* agent) { return agent->path(); }

}  // namespace cc
