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

#ifndef CC_KERNEL_H
#define CC_KERNEL_H

#include <vector>
#include <ostream>

namespace cc {

// Forwards:
class Kernel;
class Process;

struct Time {
  using cycle_type = std::uint32_t;
  using delta_type = std::uint32_t;

  Time() = default;
  
  cycle_type cycle = 0;
  delta_type delta = 0;
};

bool operator<(const Time& lhs, const Time& rhs);

class Object {
 public:
  Object();

  void parent(Object* o) {}
};

class Loggable : public Object {
 public:
  Loggable();
};

class Action {
  friend class Process;

 public:
  Action();
  virtual ~Action() = default;

  virtual void eval(Kernel* k) = 0;
};

class Event {
  friend class Process;
 public:
  Event(Kernel* k);

  Kernel* k() const { return k_; }

  void notify();

 private:
  void add_waitee(Process* p) { ps_.push_back(p); }
  
  std::vector<Process*> ps_;
  Kernel* k_;
};

class Process : public Loggable {
  friend class Module;

 public:
  Process(Kernel* k);
  virtual ~Process() = default;

  Kernel* k() const { return k_; }

  //
  virtual void init() {}
  virtual void eval() {}

  //
  virtual void wait_until(Time t);
  virtual void wait_on(Event& event);

 private:
  Kernel* k_;
};

class Module : public Loggable {
  friend class Kernel;

 protected:
  Module(Kernel* k);
 public:
  //
  virtual ~Module();

  //
  Kernel* k() const { return k_; }

  //
  virtual void init();
  virtual void fini();

  //
  void add_process(Process* p) { ps_.push_back(p); }
  void add_child(Module* m) { ms_.push_back(m); }

 private:
  Kernel* k_;
  std::vector<Process*> ps_;
  std::vector<Module*> ms_;
};

class Clock : public Module {
 public:
  Clock(Kernel* k, int n, int period = 10);

  int n() const { return n_; }
  int period() const { return period_; }

  Event& rising_edge_event() { return rising_edge_event_; }
  const Event& rising_edge_event() const { return rising_edge_event_; }

 private:
  Event rising_edge_event_;
  Process* p_;
  int n_, period_;
};

enum class RunMode { ToExhaustion, ForTime };

class Kernel {
  friend class Process;

  struct Event {
    Time time;
    Action* action;
  };

  struct EventComparer {
    bool operator()(const Event& lhs, const Event& rhs) {
      if (lhs.time.cycle > rhs.time.cycle) return true;
      if (lhs.time.cycle < rhs.time.cycle) return false;
      return lhs.time.delta < rhs.time.delta;
    }
  };
  
 public:
  Kernel();

  // Observers
  Time time() const { return time_; }

  void run(RunMode r = RunMode::ToExhaustion, Time t = Time{});

  void add_action(Time t, Action* a);
 private:
  std::vector<Event> eq_;
  Time time_;
};

}

#endif //CC_KERNEL_H
