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
#include "utility.h"

namespace cc {

MessageQueue::MessageQueue(kernel::Kernel* k, const std::string& name,
                           std::size_t n)
    : Agent(k, name) {
  // TEMPORARY override N
  build(100);
}

MessageQueue::~MessageQueue() { delete q_; }

std::string MessageQueue::to_string() const {
  using cc::to_string;
  KVListRenderer r;
  r.add_field("empty", to_string(empty()));
  r.add_field("full", to_string(full()));
  return r.to_string();
}

bool MessageQueue::has_at_least(std::size_t n) const {
  return q_->free() >= n;
}

bool MessageQueue::issue(const Message* msg, cursor_t cursor) {
  struct EnqueueAction : kernel::Action {
    EnqueueAction(kernel::Kernel* k, Queue<const Message*>* q,
                  const Message* msg)
        : Action(k, "enqueue_action"), q_(q), msg_(msg) {}
    bool eval() override {
      if (!q_->enqueue(msg_)) {
        LogMessage lm("Attempt to push new message to full queue.");
        lm.level(Level::Fatal);
        log(lm);
      }
      return true;
    }

   private:
    Queue<const Message*>* q_ = nullptr;
    const Message* msg_;
  };

  if (full()) return false;
  // Issue action:
  const kernel::Time execute_time = k()->time() + kernel::Time{cursor, 0};
  k()->add_action(execute_time, new EnqueueAction(k(), q_, msg));

  return true;
}

void MessageQueue::resize(std::size_t n) {
  if (!empty()) {
    // Cannot call resize when there are outstanding entries within
    // the queue as this method should only be called before the start
    // of the simulation.
    LogMessage msg("Attempt to resize Message Queue but is currently "
                   "non-empty.");
    msg.level(Level::Fatal);
    log(msg);
  }
  q_->resize(n);
}

void MessageQueue::set_blocked_until(kernel::Event* event) {
  struct UnblockAction : kernel::Action {
    UnblockAction(kernel::Kernel* k, MessageQueue* mq)
        : Action(k, "unblock_action"), mq_(mq) {}
    bool eval() override {
      mq_->set_blocked(false);
      return true;
    }

   private:
    MessageQueue* mq_ = nullptr;
  };
  // Message queue is now blocked
  set_blocked(true);
  // Add notify action to 'awaking' event; which then rescinds the
  // blocked state.
  event->add_notify_action(new UnblockAction(k(), this));
}

bool MessageQueue::has_req() const { return !blocked() && !q_->empty(); }

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
    const LogMessage lm("Attempt to dequeue Message failed.", Level::Fatal);
    log(lm);
    return nullptr;
  } else {
    LogMessage lm("Dequeue message: ");
    lm.append(msg->to_string());
    lm.append(" queue state: ");
    lm.append(to_string());
    lm.level(Level::Debug);
    log(lm);
  }
  return msg;
}

void MessageQueue::build(std::size_t n) {
  q_ = new Queue<const Message*>(k(), "queue", n);
  add_child_module(q_);
}

AgentProcess::AgentProcess(kernel::Kernel* k, const std::string& name)
    : Process(k, name) {}

void AgentProcess::wait_epoch() {
  wait_for(kernel::Time{epoch_, 0});
}

// Suspend/Re-evaulate process after delay.
void AgentProcess::wait_for(kernel::Time t) {
  wait_set_ = true;
  base_type::wait_for(t);
}

// Suspend/Re-evaulate process at time.
void AgentProcess::wait_until(kernel::Time t) {
  wait_set_ = true;
  base_type::wait_until(t);
}

// Suspend/Re-evaluate process upon the notification of event.
void AgentProcess::wait_on(kernel::Event* event) {
  wait_set_ = true;
  base_type::wait_on(event);
}

void AgentProcess::invoke_init() {
  wait_set_ = false;
  init();
  if (!wait_set_) {
    LogMessage msg("Wait condition not set; process terminates.");
    msg.level(Level::Fatal);
    log(msg);
  }
}

void AgentProcess::invoke_eval() {
  wait_set_ = false;
  eval();
  if (!wait_set_) {
    LogMessage msg("Wait condition not set; process terminates.");
    msg.level(Level::Fatal);
    log(msg);
  }
}

Agent::Agent(kernel::Kernel* k, const std::string& name) : Module(k, name) {}

CreditCounter::CreditCounter(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {
  credit_event_ = new kernel::Event(k, "credit_event");
  add_child(credit_event_);
}

CreditCounter::~CreditCounter() {
  delete credit_event_;
}

void CreditCounter::credit() {
  using std::to_string;

  if (i_ >= n_) {
    LogMessage msg("Credit overflow: ");
    KVListRenderer r;
    r.add_field("i", to_string(i()));
    r.add_field("n", to_string(n()));
    msg.append(r.to_string());
    msg.level(Level::Fatal);
    log(msg);
  }
  // Add credit
  ++i_;
  // Notify awaitees
  credit_event_->notify();
}

void CreditCounter::debit() {
  if (i_ == 0) {
    LogMessage msg("Credit underflow: ");
    KVListRenderer r;
    r.add_field("i", to_string(i()));
    r.add_field("n", to_string(n()));
    msg.append(r.to_string());
    msg.level(Level::Fatal);
    log(msg);
  }
  // Deduct credit
  --i_;
}

}  // namespace cc
