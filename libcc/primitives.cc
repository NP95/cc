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

#include "primitives.h"

namespace cc {

Clock::Clock(kernel::Kernel* k, const std::string& name, int ticks, int period)
    : Module(k, name),
      ticks_(ticks),
      period_(period),
      rising_edge_event_(k, "rising_edge_event") {
  struct ClockProcess : kernel::Process {
    ClockProcess(kernel::Kernel* k, Clock* clk)
        : Process(k, "ClockProcess"), clk_(clk) {
      ticks_ = clk->ticks();
    }
    void init() override {
      if (ticks_ == 0) return;
      kernel::Time time = k()->time();
      time.time += clk_->period();
      wait_until(time);
    }
    void eval() override {
      // Notify
      clk_->rising_edge_event().notify();
      if (--ticks_ != 0) {
        // Schedule next
        kernel::Time time = k()->time();
        time.time += clk_->period();
        wait_until(time);
      }
    }
    Clock* clk_;
    int ticks_;
  };
  p_ = new ClockProcess(k, this);
  add_child_process(p_);
}

MessageQueue::MessageQueue(kernel::Kernel* k, const std::string& name,
                           std::size_t n)
    : kernel::Module(k, name) {
  build(n);
}

void MessageQueue::push(const Message* msg) {
  if (!q_->enqueue(msg)) {
    LogMessage lmsg("Attempt to push new message to full queue.", Level::Error);
    log(lmsg);
  }
}

bool MessageQueue::has_req() const { return !q_->empty(); }

const Message* MessageQueue::peek() const { return nullptr; }

const Message* MessageQueue::dequeue() {
  return nullptr;
  /*
    const Message* msg;
    if (!q_->dequeue(msg)) {
    const LogMessage msg{"Attempt to dequeue Message failed.", Level::Fatal};
    log(msg);
    return nullptr;
    }
    return msg;
  */
}

kernel::Event& MessageQueue::request_arrival_event() {
  return q_->non_empty_event();
}

void MessageQueue::build(std::size_t n) {
  q_ = new Queue<const Message*>(k(), "queue", n);
  add_child(q_);
}

}  // namespace cc
