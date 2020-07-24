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

  void invoke_init() override final;

  void invoke_eval() override final;
  
  bool wait_set_ = false;
};

//
//
class Agent : public kernel::Module {
 public:
  Agent(kernel::Kernel* k, const std::string& name);
};

class MessageQueueProxy;

//
//
class MessageQueue : public Agent {
  friend class MessageQueueProxy;
 public:
  MessageQueue(kernel::Kernel* k, const std::string& name, std::size_t n);
  ~MessageQueue();

  MessageQueueProxy* construct_proxy();
  
  // Queue depth.
  std::size_t n() const { return q_->n(); }

  std::size_t credits() { return credits_; }

  bool empty() const { return q_->empty(); }
  bool full() const { return credits_ == 0; }
  // Flag indicating that the current agent is blocked.
  bool blocked() const { return blocked_; }
  bool has_req() const;
  

  // Event notified when Message Queue transitions from empty to
  // non-empty state.
  kernel::Event* non_empty_event() const { return q_->non_empty_event(); }

  // Event notified when Message Queue transitions from full to
  // non-full state.
  kernel::Event* non_full_event() const { return q_->non_full_event(); }

  // Peek head message, nullptr on empty.
  const Message* peek() const;
  // Dequeue head message from queue.
  const Message* dequeue();
  
  // Set blocked status of Message Queue until notified by event.
  void set_blocked_until(kernel::Event* event);

  // Deprecate
  // Set blocked status of requestor.
  void set_blocked(bool blocked) { blocked_ = blocked; }

 private:
  // Construct module
  void build(std::size_t n);
  // Issue message to queue after 'epoch' agent epochs.
  bool issue(const Message* msg, epoch_t epoch = 0);
  // Queue primitive.
  Queue<const Message*>* q_ = nullptr;
  // Flag indicating that the current requestor is blocked.
  bool blocked_ = false;
  // Queue credit count.
  std::size_t credits_;
};

//
//
class MessageQueueProxy : public Agent {
  friend class MessageQueue;

  MessageQueueProxy(MessageQueue* mq);
 public:
  // Target Message Queue is full.
  bool full() const { return credits_ == 0; }
  // Total credits available in Message Queue
  std::size_t credits() const { return credits_; }
  // Return credit to proxy.
  void add_credit() { credits_++; }
  // Issue message to target
  bool issue(const Message* msg, epoch_t epoch = 0);
 private:
  // Associated Message Queue
  MessageQueue* mq_ = nullptr;
  // Credit counter.
  std::size_t credits_;
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

}  // namespace cc

#endif
