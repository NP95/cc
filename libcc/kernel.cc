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
#include <exception>
#include <iostream>

namespace cc::kernel {

const char* to_string(Phase phases) {
  switch (phases) {
#define __declare_to_string(__name) \
  case Phase::__name:               \
    return #__name;                 \
    break;
    PHASES(__declare_to_string)
  }
#undef __declare_to_string
  return "Invalid";
}

Time operator+(const Time& lhs, const Time& rhs) {
  return Time{lhs.time + rhs.time, lhs.delta + rhs.delta};
}

bool operator==(const Time& lhs, const Time& rhs) {
  if (lhs.time != rhs.time) return false;
  if (lhs.delta != rhs.delta) return false;
  return true;
}

bool operator<(const Time& lhs, const Time& rhs) {
  if (lhs.time < rhs.time) return true;
  if (lhs.time > rhs.time) return false;
  return (lhs.delta < rhs.delta);
}

bool operator<=(const Time& lhs, const Time& rhs) {
  if (operator==(lhs, rhs)) return true;
  return operator<(lhs, rhs);
}

std::string Time::to_string() const {
  return std::string{std::to_string(time) + "." + std::to_string(delta)};
}

std::string to_string(const Time& t) { return t.to_string(); }

std::ostream& operator<<(std::ostream& os, const Time& t) {
  return os << t.to_string();
}

RandomSource::RandomSource(seed_type seed) : seed_(seed), mt_(seed) {}

bool RandomSource::random_bool(float true_probability) {
  std::bernoulli_distribution dist(true_probability);
  return dist(mt_);
}

void ObjectVisitor::iterate(Object* root) { root->accept(this); }

void Object::set_top() {
  parent_ = nullptr;
  k_->set_top(this);
}

void Object::iterate_children(ObjectVisitor* visitor) {
  for (Object* o : children_) {
    o->accept(visitor);
  }
}

LogContext::LogContext(std::ostream* os) : os_(os) {}

LogContext::~LogContext() {
  if (os_ != std::addressof(std::cout)) {
    os_->flush();
    delete os_;
  }
}

void LogContext::info(const std::string& name, bool nl) {
  if (os_) {
    *os_ << name;
    if (nl) *os_ << "\n";
  }
}

Kernel::Kernel(seed_type seed) : random_source_(seed), Module(this, "kernel") {}

void Kernel::run(RunMode r, Time t) {
  // Build phase is already complete by this stage and it is assume
  // that the object heirarchy has been correct constructed at this point.

  // Build and elaborate simulation environment.
  invoke_elab();
  // Run Design Rule Check to validate environment correctness.
  invoke_drc();
  // Invoke initialization phase.
  invoke_init();
  // Run simulation.
  invoke_run(r, t);
  // Run finalization.
  invoke_fini();
}

void Kernel::add_action(Time t, Action* a) {
  eq_.push_back(Event{t, a});
  if (eq_.size() > 1) {
    // EventComparer has undefined behavior unless event-queue has at
    // least two entries.
    std::push_heap(eq_.begin(), eq_.end(), EventComparer{});
  }
}

void Kernel::set_seed(seed_type seed) { random_source_ = RandomSource(seed); }

void Kernel::invoke_elab() {
  set_phase(Phase::Elab);
  struct InvokeElabVisitor : ObjectVisitor {
    void visit(Module* o) override { o->elab(); }
  };
  InvokeElabVisitor visitor;
  visitor.iterate(top());
}

void Kernel::invoke_drc() {
  set_phase(Phase::Drc);
  struct InvokeDrcVisitor : ObjectVisitor {
    void visit(Module* o) override { o->drc(); }
  };
  InvokeDrcVisitor visitor;
  visitor.iterate(top());
}

void Kernel::invoke_init() {
  set_phase(Phase::Init);
  struct InvokeInitVisitor : ObjectVisitor {
    void visit(Process* o) override { o->invoke_init(); }
  };
  InvokeInitVisitor visitor;
  visitor.iterate(top());
}

void Kernel::invoke_run(RunMode r, Time t) {
  set_phase(Phase::Run);
  //
  Time current_time = time();
  try {
    while (!eq_.empty()) {
      // IF a fatal error has occurred, terminate the simulation immediately.
      if (fatal()) break;

      const Event e = eq_.front();
      // If simulation time has elapsed, terminate.
      if (r == RunMode::ForTime && t < e.time) break;

      if (eq_.size() > 1) {
        // EventComparer has undefined behavior unless greater than or
        // equal to two entries.
        std::pop_heap(eq_.begin(), eq_.end(), EventComparer{});
      }
      eq_.pop_back();
      if (e.time < current_time) {
        // TODO: Kernel should eventually become the top-level module.

        // LogMessage msg("Attempt to schedule an action in the past",
        // Level::Fatal); log(msg);
        throw std::runtime_error("Fatal error occurred.");
      }
      time_ = e.time;
      if (e.action->eval()) {
        e.action->release();
      }
    }
  } catch (...) {
    // LogMessage msg("Simulation terminated unexpectedly.", Level::Fatal);
    // msg.suppress_except(true);
    // log(msg);
  }
}

void Kernel::invoke_fini() {
  set_phase(Phase::Fini);
  struct InvokeFiniVisitor : ObjectVisitor {
    void visit(Process* o) override { o->fini(); }
  };
  InvokeFiniVisitor visitor;
  visitor.iterate(top());
}

Object::Object(Kernel* k, const std::string& name) : k_(k), name_(name) {}

Object::~Object() {}

std::string Object::path() const {
  if (path_.empty()) {
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

Loggable::LogMessage& Loggable::LogMessage::append(const std::string& str) {
  msg_ += str;
  return *this;
}

Loggable::Loggable(Kernel* k, const std::string& name) : Object(k, name) {}

void Loggable::log_prefix(Level l, std::ostream& os) const {
  const char* t = type_str();
  os << l.to_char() << "[" << to_string(k()->phase())[0] << ";" << path() << ";"
     << t[0] << "@" << k()->time() << "]: ";
}

void Loggable::log(const LogMessage& m) const {
  if (level() >= m.level()) {
    LogContext& log_context = k()->log_context();
    log_prefix(m.level(), log_context.os());
    log_context.os() << m.msg() << "\n";
  }
  if (m.level() == Level::Fatal) {
    k()->raise_fatal();
    if (!m.suppress_except()) {
      throw std::runtime_error("Fatal error occurred.");
    }
  }
}

Action::Action(Kernel* k, const std::string& name) : Loggable(k, name) {}

void Action::release() { delete this; }

ProcessHost::ProcessHost(Kernel* k, const std::string& name)
    : Loggable(k, name) {}

void ProcessHost::add_child_process(Process* p) {
  Object::add_child(p);
  ps_.push_back(p);
}

Event::Event(Kernel* k, const std::string& name) : Loggable(k, name) {}

void Event::add_waitee(Process* p) {
  struct EvalProcessAction : Action {
    EvalProcessAction(Kernel* k, Process* p)
        : Action(k, "EvalProcessAction"), p_(p) {}
    bool eval() override {
      p_->invoke_eval();
      // Discard after evaluation.
      return true;
    }
    Process* p_ = nullptr;
  };
  as_.push_back(new EvalProcessAction(k(), p));
}

void Event::notify() {
  const Time current_time{k()->time()};
  const Time time{current_time.time, current_time.delta + 1};
  for (Action* a : as_) {
    k()->add_action(time, a);
  }
  as_.clear();
}

EventOr::EventOr(Kernel* k, const std::string& name) : Event(k, name) {}

EventOr::~EventOr() {
  for (const Action* a : fwdas_) {
    delete a;
  }
}

void EventOr::finalize() {
  // Forwarding process awaits the notification of one of the
  // associated child events and then forwards the notification to the
  // associated parent event (to be scheduled in the subsequent delta
  // cycle).
  struct EventNotifyForwardAction : Action {
    EventNotifyForwardAction(Kernel* k, const std::string& name, Event* parent,
                             Event* child)
        : Action(k, name), parent_(parent), child_(child) {}
    bool eval() override {
      parent_->notify();
      // Must register EventOr with child as once notified the waitee
      // action is flushed from the childs event list.
      child_->add_notify_action(this);
      // Do not discard after evaluation as action is reused.
      return false;
    }

   private:
    Kernel* k_ = nullptr;
    Event* parent_ = nullptr;
    Event* child_ = nullptr;
  };

  for (Event* e : childs_) {
    const std::string process_name = name() + ".fwd." + e->name();
    EventNotifyForwardAction* fwda =
        new EventNotifyForwardAction(k(), process_name, this, e);
    e->add_notify_action(fwda);
    fwdas_.push_back(fwda);
  }
}

Process::Process(Kernel* k, const std::string& name) : Loggable(k, name) {}

void Process::next_delta() {
  // Suspend the current process to be again reinvoked in the delta cycle.
  const Time current_time = k()->time();
  wait_until(Time{current_time.time, current_time.delta + 1});
}

void Process::wait_for(Time t) { wait_until(k()->time() + t); }

void Process::wait_until(Time t) {
  struct WaitUntilAction : Action {
    WaitUntilAction(Kernel* k, Process* p)
        : Action(k, "WaitUntilAction"), p_(p) {}
    bool eval() override {
      p_->eval();
      // Discard after evaluation.
      return true;
    }
    Process* p_ = nullptr;
  };
  k()->add_action(t, new WaitUntilAction(k(), this));
}

void Process::wait_on(Event* event) { event->add_waitee(this); }

//
void Process::invoke_init() {
  init();
}

//
void Process::invoke_eval() {
  eval();
}

Module::Module(Kernel* k, const std::string& name) : ProcessHost(k, name) {}

Module::~Module() {}

void Module::add_child_module(Module* m) {
  Object::add_child(m);
  ms_.push_back(m);
}

TopModule::TopModule(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {
  set_top();
}

}  // namespace cc::kernel
