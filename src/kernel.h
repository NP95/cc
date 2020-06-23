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

class Object {
 public:
  Object();

  void parent(Object* o) {}
};

class Loggable : public Object {
 public:
  Loggable();
};

class Agent {
 public:
  Kernel* k() const { return k_; }
  void k(Kernel* k) { k_ = k; }
  
 private:
  Kernel* k_ = nullptr;
};

class Context {
 public:
  Context(Kernel* kernel, Time time) : kernel_(kernel), time_(time) {}

  Kernel* kernel() const { return kernel_; }
  Time time() const { return time_; }

 private:
  Time time_;
  Kernel* kernel_;
};

class Action {
  friend class Process;

 public:
  Action();
  virtual ~Action() = default;

  virtual void eval(Context& context) = 0;
};

class Event {
  friend class Process;
 public:
  Event() {}

  void notify(Context& context);

 private:
  void add_waitee(Process* p) { ps_.push_back(p); }
  
  std::vector<Process*> ps_;
};

class Process : public Agent, public Loggable {
  friend class Module;

 public:
  Process();
  virtual ~Process() = default;

  //
  virtual void init(Context& context) {}
  
  //
  virtual void eval(Context& context) {}

  //
  virtual void wait_until(Context& context, Time t);

  //
  virtual void wait_on(Event& event);
};

class Module : public Agent, public Loggable {
  friend class Kernel;

 protected:
  Module();
 public:
  //
  virtual ~Module() {}

  //
  virtual void elaborate() {}

  //
  template<typename T, typename ...ARGS>
  T* create_process(ARGS&& ...args) {
    // ASSERT: only in build phase
    T* p = new T(std::forward<ARGS>(args)...);
    p->k(k());
    p->parent(this);
    Context context(k(), Time{});
    p->init(context);
    return p;
  }

  //
  template<typename T, typename ...ARGS>
  T* create_child(ARGS&& ...args) {
    // ASSERT: only in build phase
    T* m = new T(std::forward<ARGS>(args)...);
    m->k(k());
    m->parent(this);
    m->elaborate();
    return m;
  }
};

class Clock : public Module {
  
 public:
  Clock(int n, int period = 10);

  int n() const { return n_; }
  int period() const { return period_; }

  void elaborate() override;

  Event& rising_edge_event() { return rising_edge_event_; }
  const Event& rising_edge_event() const { return rising_edge_event_; }

 private:
  Event rising_edge_event_;
  Process* p_;
  int n_, period_;
};

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

  template<typename T, typename ...ARGS>
  Module* create_top(ARGS&& ...args) {
    top_ = new T(std::forward<ARGS>(args)...);
    top_->k(this);
    top_->elaborate();
    return top_;
  }

  void run();

  void add_action(Time t, Action* a);
 private:
  
  std::vector<Event> eq_;

  Module* top_ = nullptr;
};

}

#endif //CC_KERNEL_H
