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

//
//
class MessageQueue : public kernel::Module {
 public:
  MessageQueue(kernel::Kernel* k, const std::string& name, std::size_t n);

  // Queue depth.
  std::size_t n() const { return q_->n(); }

  bool empty() const { return q_->empty(); }
  bool full() const { return q_->full(); }

  bool issue(const Message* msg, kernel::Time t = kernel::Time{});

  // Set blocked status of requestor.
  void set_blocked(bool blocked) { blocked_ = blocked; }

  // Flag indicating that the current agent is blocked.
  bool blocked() const { return blocked_; }

  // Endpoint Interface:
  void push(const Message* msg);

  // Requester Interface:
  bool has_req() const;
  const Message* peek() const;
  const Message* dequeue();

  kernel::Event& request_arrival_event();
  kernel::Event& non_empty_event() { return q_->non_empty_event(); }
  kernel::Event& non_full_event() { return q_->non_full_event(); }

 private:
  // Construct module
  void build(std::size_t n);
  // Queue primitive.
  Queue<const Message*>* q_ = nullptr;
  // Flag indicating that the current requestor is blocked.
  bool blocked_ = false;
};

//
//
class MQArb : public Arbiter<MessageQueue> {
 public:
  MQArb(kernel::Kernel* k, const std::string& name)
      : Arbiter(k, name) {}
};

//
//
using MQArbTmt = MQArb::Tournament;

//
//
class Agent : public kernel::Module {
 public:
  Agent(kernel::Kernel* k, const std::string& name);
};

std::string to_string(const Agent* agent);

}  // namespace cc

#endif
