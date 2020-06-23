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

#include "kernel.h"
#include <algorithm>

namespace cc {

Object::Object() {}

Loggable::Loggable() {}

Action::Action() {}

void Event::notify(Context& context) {
  struct EvalProcessAction : Action {
    EvalProcessAction(Process* p) : p_(p) {}
    void eval(Context& context) { p_->eval(context); }
    Process* p_;
  };
  const Time time{context.time().cycle, context.time().delta + 1};
  for (Process* p : ps_) {
    Kernel* kernel = context.kernel();
    kernel->add_action(time, new EvalProcessAction(p));
  }
  ps_.clear();
}

Process::Process() {}

void Process::wait_until(Context& context, Time t) {
  struct WaitUntilAction : Action {
    WaitUntilAction(Process* p) : p_(p) {}
    void eval(Context& context) override { p_->eval(context); }
    Process* p_ = nullptr;
  };
  Kernel* kernel = context.kernel();
  kernel->add_action(t, new WaitUntilAction(this));
}

void Process::wait_on(Event& event) {
  event.add_waitee(this);
}

Module::Module() {}

Clock::Clock(int n, int period) : n_(n), period_(period) {}

void Clock::elaborate() {
  struct ClockProcess : Process {
    ClockProcess(Clock* clk) : clk_(clk) {}
    void init(Context& context) override {
      Time time = context.time();
      time.cycle += clk_->period();
      wait_until(context, time);
    }
    void eval(Context& context) override {
      // Notify
      clk_->rising_edge_event().notify(context);
      // Schedule next
      Time time = context.time();
      time.cycle += clk_->period();
      wait_until(context, time);
    }
    Clock* clk_;
  };
  p_ = create_process<ClockProcess>(this);
}

Kernel::Kernel() {}

void Kernel::run() {
  while (!eq_.empty()) {
    const Event e = eq_.front();
    std::pop_heap(eq_.begin(), eq_.end(), EventComparer{});
    eq_.pop_back();
    Context context{this, e.time};
    e.action->eval(context);
  }
}

void Kernel::add_action(Time t, Action* a) {
  eq_.push_back(Event{t, a});
  std::push_heap(eq_.begin(), eq_.end(), EventComparer{});
}

}
