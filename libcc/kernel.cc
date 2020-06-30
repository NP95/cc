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

namespace cc::kernel {

Time operator+(const Time& lhs, const Time& rhs) {
  return Time{lhs.time + rhs.time, lhs.delta + rhs.delta};
}

bool operator<(const Time& lhs, const Time& rhs) {
  if (lhs.time < rhs.time) return true;
  if (lhs.time > rhs.time) return false;
  return (lhs.delta < rhs.delta);
}

std::string Time::to_string() const {
  return std::string{std::to_string(time) + ":" + std::to_string(delta)};
}

std::ostream& operator<<(std::ostream& os, const Time& t) {
  return os << t.to_string();
}

RandomSource::RandomSource(seed_type seed) : seed_(seed), mt_(seed) {}

bool RandomSource::random_bool(float true_probability) {
  std::bernoulli_distribution dist(true_probability);
  return dist(mt_);
}

void ObjectVisitor::iterate(Object* root) {
  root->accept(this);
}

void Object::iterate_children(ObjectVisitor* visitor) {
  for (Object* o : children_) { o->accept(visitor); }
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

Kernel::Kernel(seed_type seed) : random_source_(seed) {}

void Kernel::run(RunMode r, Time t) {
  // Build phase is already complete by this stage and it is assume
  // that the object heirarchy has been correct constructed at this point.

  // Check top is set.
  
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
  std::push_heap(eq_.begin(), eq_.end(), EventComparer{});
}

void Kernel::set_seed(seed_type seed) { random_source_ = RandomSource(seed); }

void Kernel::invoke_elab() {
  set_phase(Phase::Elab);
  log_context().info("Elaborating simulation.");

  struct InvokeElabVisitor : ObjectVisitor {
    void visit(Module* o) override { o->elab(); }
  };
  InvokeElabVisitor visitor;
  visitor.iterate(top());
}

void Kernel::invoke_drc() {
  set_phase(Phase::Drc);
  log_context().info("Running Design Rule Check.");

  struct InvokeDrcVisitor : ObjectVisitor {
    void visit(Module* o) override { o->drc(); }
  };
  InvokeDrcVisitor visitor;
  visitor.iterate(top());
}

void Kernel::invoke_init() {
  set_phase(Phase::Init);
  log_context().info("Invoking initialization");

  struct InvokeInitVisitor : ObjectVisitor {
    void visit(Process* o) override { o->init(); }
  };
  InvokeInitVisitor visitor;
  visitor.iterate(top());
}

void Kernel::invoke_run(RunMode r, Time t) {
  set_phase(Phase::Run);
  log_context().info("Starting simulation.");
  while (!eq_.empty()) {
    // IF a fatal error has occurred, terminate the simulation immediately.
    if (fatal()) break;

    const Event e = eq_.front();
    // If simulation time has elapsed, terminate.
    if (r == RunMode::ForTime && t < e.time) break;

    std::pop_heap(eq_.begin(), eq_.end(), EventComparer{});
    eq_.pop_back();
    time_ = e.time;
    if (e.action->eval()) {
      e.action->release();
    }
  }
}

void Kernel::invoke_fini() {
  set_phase(Phase::Fini);
  log_context().info("Finalizing.");

  struct InvokeFiniVisitor : ObjectVisitor {
    void visit(Process* o) override { o->fini(); }
  };
  InvokeFiniVisitor visitor;
  visitor.iterate(top());
}

Object::Object(Kernel* k, const std::string& name) : k_(k), name_(name) {}

Object::~Object() {
  for (Object* o : children_) delete o;
}

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
    LogContext& log_context = k()->log_context();
    log_prefix(m.level(), log_context.os());
    log_context.os() << m.msg() << "\n";
  }
  if (m.level() == Level::Fatal) {
    k()->raise_fatal();
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

Event::Event(Kernel* k, const std::string& name) : ProcessHost(k, name) {}

void Event::notify() {
  struct EvalProcessAction : Action {
    EvalProcessAction(Kernel* k, Process* p)
        : Action(k, "EvalProcessAction"), p_(p) {}
    bool eval() override {
      p_->eval();
      // Discard after evaluation.
      return true;
    }
    Process* p_;
  };
  const Time current_time{k()->time()};
  const Time time{current_time.time, current_time.delta + 1};
  for (Process* p : ps_) {
    k()->add_action(time, new EvalProcessAction(k(), p));
  }
  ps_.clear();
}

EventOr::EventOr(Kernel* k, const std::string& name)
    : Event(k, name) {}

void EventOr::finalize() {
  // Forwarding process awaits the notification of one of the
  // associated child events and then forwards the notification to the
  // associated parent event (to be scheduled in the subsequent delta
  // cycle).
  struct EventNotifyForwardProcess : Process {
    EventNotifyForwardProcess(Kernel* k, Event* parent, Event* e, const std::string& name)
        : Process(k, name), parent_(parent), e_(e) {}

    // Pointer to parent event object.
    Event* parent() const { return e_; }
    // Pointer to underlying event object.
    Event* e() const { return e_; }

    void init() override {
      wait_on(*e());
    }
    void eval() override {
      parent_->notify();
      wait_on(*e());
    }
   private:
    Kernel* k_;
    Event* e_;
    Event* parent_;
  };
  for (Event* e : childs_) {
    const std::string process_name = name() + ".fwd." + e->name();
    EventNotifyForwardProcess* fwdp =
        new EventNotifyForwardProcess(k(), this, e, process_name);
    fwdps_.push_back(fwdp);
    add_child_process(fwdp);
  }
}

Process::Process(Kernel* k, const std::string& name) : Loggable(k, name) {}

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

void Process::wait_on(Event& event) { event.add_waitee(this); }

Module::Module(Kernel* k, const std::string& name) : ProcessHost(k, name) {}

Module::~Module() {}

void Module::elab() {
  for (Module* m : ms_) m->elab();
}

void Module::drc() {
  for (Module* m : ms_) m->drc();
}

void Module::add_child_module(Module* m) {
  Object::add_child(m);
  ms_.push_back(m);
}

TopModule::TopModule(Kernel* k, const std::string& name) : Module(k, name) {
  set_top();
}


}  // namespace cc::kernel
