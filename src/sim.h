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

#ifndef CC_LIBCC_SIM_H
#define CC_LIBCC_SIM_H

#include "primitives.h"
#include "msg.h"

namespace cc {

class Message;
class Transaction;

//
//
class AgentProcess : public kernel::Process {
  using base_type = kernel::Process;

 public:
  AgentProcess(kernel::Kernel* k, const std::string& name);

  // Suspend/Re-evaulate process after delay.
  void wait_for(kernel::Time t) override;

  // Suspend/Re-evaulate process at time.
  void wait_until(kernel::Time t) override;

  // Suspend/Re-evaluate process upon the notification of event.
  void wait_on(kernel::Event* event) override;

 private:
  void invoke_init() final;

  void invoke_eval() final;

  bool wait_set_ = false;
};

class MessageQueue;

//
//
class Agent : public kernel::Module {
 public:
  Agent(kernel::Kernel* k, const std::string& name);
};


//
//
class MessageQueue : public Agent {
  friend class UnblockAction;

 public:
  MessageQueue(kernel::Kernel* k, const std::string& name, std::size_t n);
  ~MessageQueue();

  std::string to_string() const;

  // Total number of entries in the queue.
  std::size_t n() const { return q_->n(); }
  // Queue has at least 'n' available entries.
  bool has_at_least(std::size_t n) const;
  // Queue is empty.
  bool empty() const { return q_->empty(); }
  // Queue is full.
  bool full() const { return q_->full(); }
  // Flag indicating that the current agent is blocked.
  bool blocked() const { return blocked_; }

  // Event notified when Message Queue transitions from empty to
  // non-empty state.
  kernel::Event* non_empty_event() const { return q_->non_empty_event(); }
  // Event notified when Message Queue transitions from full to
  // non-full state.
  kernel::Event* non_full_event() const { return q_->non_full_event(); }
  // Event notified when a Message has been dequeued from the Message
  // Queue.
  kernel::Event* dequeue_event() const { return q_->dequeue_event(); }

  // Message Queue is currently asserting its request status for
  // arbitration.
  bool has_req() const;
  // Peek head message, nullptr on empty.
  const Message* peek() const;
  // Dequeue head message from queue.
  const Message* dequeue();
  // Set blocked status of Message Queue until notified by event.
  void set_blocked_until(kernel::Event* event);
  // Issue message to queue after 'epoch' agent epochs.
  bool issue(const Message* msg, time_t time = 0);
  // Resize queue (build/elab only)
  void resize(std::size_t n);

 private:
  // Construct module
  void build(std::size_t n);
  // Queue primitive.
  Queue<const Message*>* q_ = nullptr;
  // Set blocked status of requestor.
  void set_blocked(bool blocked) { blocked_ = blocked; }
  // Flag indicating that the current requestor is blocked.
  bool blocked_ = false;
};

//
//
class MQArb : public Arbiter<MessageQueue> {
 public:
  MQArb(kernel::Kernel* k, const std::string& name) : Arbiter(k, name) {}
};

//
//
using MQArbTmt = MQArb::Tournament;

//
//
template <typename STATE>
class TransactionTable : public Table<const Transaction*, STATE> {
  using base_type = Table<const Transaction*, STATE>;

 public:
  using key_type = const Transaction*;
  using value_type = STATE;
  using iterator = typename base_type::iterator;
  using const_iterator = typename base_type::const_iterator;

  TransactionTable(kernel::Kernel* k, const std::string& name, std::size_t n)
      : Table<const Transaction*, STATE>(k, name, n) {}
};

//
//
class CreditCounter : public kernel::Module {
 public:
  CreditCounter(kernel::Kernel* k, const std::string& name);
  ~CreditCounter();

  // Credit capacity
  std::size_t n() const { return n_; }
  // Credit count
  std::size_t i() const { return i_; }
  // Credit "credited" event.
  kernel::Event* credit_event() const { return credit_event_; }
  // Empty status (no credits available)
  bool empty() const { return i() == 0; }
  // Full status (no outstanding credits).
  bool full() const { return i() == n(); }

  // Setters:
  // Set credit capacity
  void set_n(std::size_t n) { n_ = n; set_i(n); }
  // Set credit count
  void set_i(std::size_t i) { i_ = i; }

  // Credit counter
  void credit();
  // Debit counter
  void debit();

 private:
  // Credit "credited" event.
  kernel::Event* credit_event_ = nullptr;
  // Credit counter.
  std::size_t n_ = 0, i_ = 0;
};

}  // namespace cc

#endif
