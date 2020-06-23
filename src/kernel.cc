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

bool operator<(const Time& lhs, const Time& rhs) {
  if (lhs.cycle < rhs.cycle) return true;
  if (lhs.cycle > rhs.cycle) return false;
  return (lhs.delta < rhs.delta);
}

Object::Object() {}

Loggable::Loggable() {}

Action::Action() {}

Event::Event(Kernel* k) : k_(k) {}

void Event::notify() {
  struct EvalProcessAction : Action {
    EvalProcessAction(Process* p) : p_(p) {}
    void eval(Kernel* k) override { p_->eval(); }
    Process* p_;
  };
  const Time current_time{k()->time()};
  const Time time{current_time.cycle, current_time.delta + 1};
  for (Process* p : ps_) {
    k()->add_action(time, new EvalProcessAction(p));
  }
  ps_.clear();
}

Process::Process(Kernel* k) : k_(k) {}

void Process::wait_until(Time t) {
  struct WaitUntilAction : Action {
    WaitUntilAction(Process* p) : p_(p) {}
    void eval(Kernel* k) override { p_->eval(); }
    Process* p_ = nullptr;
  };
  k()->add_action(t, new WaitUntilAction(this));
}

void Process::wait_on(Event& event) {
  event.add_waitee(this);
}

Module::Module(Kernel* k) : k_(k) {}

Module::~Module() {
  for (Module* m : ms_) delete m;
  for (Process* p : ps_) delete p;
}

void Module::init() {
  for (Module* m : ms_) m->init();
  for (Process* p : ps_) p->init();
}

void Module::fini() {
}

Clock::Clock(Kernel* k, int n, int period)
    : Module(k), n_(n), period_(period), rising_edge_event_(k) {
  struct ClockProcess : Process {
    ClockProcess(Kernel* k, Clock* clk) : Process(k), clk_(clk) {}
    void init() override {
      Time time = k()->time();
      time.cycle += clk_->period();
      wait_until(time);
    }
    void eval() override {
      // Notify
      clk_->rising_edge_event().notify();
      // Schedule next
      Time time = k()->time();
      time.cycle += clk_->period();
      wait_until(time);
    }
    Clock* clk_;
  };
  p_ = new ClockProcess(k, this);
  add_process(p_);
}

Kernel::Kernel() {}

void Kernel::run(RunMode r, Time t) {
  while (!eq_.empty()) {
    const Event e = eq_.front();
    if (r == RunMode::ForTime && t < e.time) break;
    std::pop_heap(eq_.begin(), eq_.end(), EventComparer{});
    eq_.pop_back();
    time_ = e.time;
    e.action->eval(this);
  }
}

void Kernel::add_action(Time t, Action* a) {
  eq_.push_back(Event{t, a});
  std::push_heap(eq_.begin(), eq_.end(), EventComparer{});
}

}
