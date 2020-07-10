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
class MessageQueue : public kernel::Module,
                     public kernel::RequesterIntf<const Message*> {
 public:
  MessageQueue(kernel::Kernel* k, const std::string& name, std::size_t n);

  // Queue depth.
  std::size_t n() const { return q_->n(); }

  bool empty() const { return q_->empty(); }
  bool full() const { return q_->full(); }

  // Endpoint Interface:
  void push(const Message* msg);

  // Requester Interface:
  bool has_req() const override;
  const Message* peek() const override;
  const Message* dequeue() override;
  kernel::Event& request_arrival_event() override;

 private:
  // Construct module
  void build(std::size_t n);
  // Queue primitive.
  Queue<const Message*>* q_ = nullptr;
};

//
//
class MessageQueueArbiter : public Arbiter<const Message*> {
 public:
  MessageQueueArbiter(kernel::Kernel* k, const std::string& name)
      : Arbiter(k, name)
  {}
};

//
//
class Agent : public kernel::Module {
 public:
  Agent(kernel::Kernel* k, const std::string& name);

 protected:
  // (Run-Phase only)
  void issue(MessageQueue* mq, const kernel::Time& time, const Message* msg);
};

std::string to_string(const Agent* agent);

} // namespace cc

#endif
