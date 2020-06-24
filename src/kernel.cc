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
#include <iostream>

namespace cc {

Time operator+(const Time& lhs, const Time& rhs) {
  return Time{lhs.time + rhs.time, lhs.delta + rhs.delta};
}

bool operator<(const Time& lhs, const Time& rhs) {
  if (lhs.time < rhs.time) return true;
  if (lhs.time > rhs.time) return false;
  return (lhs.delta < rhs.delta);
}

std::ostream& operator<<(std::ostream& os, const Time& t) {
  return os << t.time << ":" << t.delta;
}

RandomSource::RandomSource(seed_type seed) : seed_(seed), mt_(seed) {}

bool RandomSource::random_bool(float true_probability) {
  std::bernoulli_distribution dist(true_probability);
  return dist(mt_);
}

Kernel::Kernel(seed_type seed) : random_source_(seed) {}

void Kernel::run(RunMode r, Time t) {
  while (!eq_.empty()) {
    // IF a fatal error has occurred, terminate the simulation immediately.
    if (fatal()) break;

    const Event e = eq_.front();
    // If simulation time has elapsed, terminate.
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

void Kernel::set_seed(seed_type seed) {
  random_source_ = RandomSource(seed);
}

Object::Object(Kernel* k, const std::string& name) : k_(k), name_(name) {}

Object::~Object() {
  for (Object* o : children_) delete o;
}

std::string Object::path() const {
  if (path_.empty() && !is_top()) {
    // Construct path;
    std::vector<std::string> vs;
    const Object* current = this;
    while (current != nullptr) {
      vs.push_back(current->name());
      current = current->parent_;
    }
    for (int i = vs.size() - 1; i >= 0; i--) {
      path_ += vs[i];
      if (i != 0) path_ += '.';
    }
  }
  return path_;
}

void Object::add_child(Object* c) {
  children_.push_back(c);
  c->set_parent(this);
}

Loggable::Message& Loggable::Message::append(const std::string& str) {
  msg_ += str;
  return *this;
}

Loggable::Loggable(Kernel* k, const std::string& name) : Object(k, name) {}

void Loggable::log_prefix(Level l, std::ostream& os) const {
  os << "[" << l.to_char() << ";" << path() << "@" << k()->time() << "]: ";
}

void Loggable::log(const Message& m) const {
  if (level() >= m.level()) {
    log_prefix(m.level(), std::cout);
    std::cout << m.msg() << "\n";
  }
  if (m.level() == Level::Fatal) {
    k()->raise_fatal();
  }
}

Action::Action() {}

Event::Event(Kernel* k) : k_(k) {}

void Event::notify() {
  struct EvalProcessAction : Action {
    EvalProcessAction(Process* p) : p_(p) {}
    void eval(Kernel* k) override { p_->eval(); }
    Process* p_;
  };
  const Time current_time{k()->time()};
  const Time time{current_time.time, current_time.delta + 1};
  for (Process* p : ps_) {
    k()->add_action(time, new EvalProcessAction(p));
  }
  ps_.clear();
}

Process::Process(Kernel* k, const std::string& name) : Loggable(k, name) {}

void Process::wait_for(Time t) {
  wait_until(k()->time() + t);
}

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

Module::Module(Kernel* k, const std::string& name) : Loggable(k, name) {}

Module::~Module() {}

void Module::init() {
  for (Module* m : ms_) m->init();
  for (Process* p : ps_) p->init();
}

void Module::fini() {
}

void Module::add_child(Module* m) {
  ms_.push_back(m);
  Object::add_child(m);
}

void Module::add_child(Process* p) {
  ps_.push_back(p);
  Object::add_child(p);
}

}
