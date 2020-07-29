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

#ifndef CC_INCLUDE_CC_KERNEL_H
#define CC_INCLUDE_CC_KERNEL_H

#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <vector>

namespace cc::kernel {

// Forwards
class Kernel;

// clang-format off
#define KERNEL_TYPES(__func)			\
  __func(Module)				\
  __func(ProcessHost)				\
  __func(Process)				\
  __func(Action)				\
  __func(Loggable)				\
  __func(Object)
// clang-format on

#define __declare_forwards(__type) class __type;
KERNEL_TYPES(__declare_forwards)
#undef __declare_forwards

struct ObjectVisitor {
  virtual ~ObjectVisitor() = default;
  void iterate(Object* root);

#define __declare_visit_method(__type) \
  virtual void visit(__type* o) {}
  KERNEL_TYPES(__declare_visit_method)
#undef __declare_visit_method
};

#define DECLARE_VISITEE(__name)                 \
  friend class ObjectVisitor;                   \
  virtual void accept(ObjectVisitor* visitor) { \
    visitor->visit(this);                       \
    Object::iterate_children(visitor);          \
  }                                             \
  virtual const char* type_str() const {	\
    return #__name;				\
  }

struct Time {
  using time_type = std::uint32_t;
  using delta_type = std::uint32_t;

  std::string to_string() const;

  // TODO: consider changing to a tuple type.
  time_type time = 0;
  delta_type delta = 0;
};

std::string to_string(const Time& t);

Time operator+(const Time& lhs, const Time& rhs);
bool operator<(const Time& lhs, const Time& rhs);
bool operator==(const Time& lhs, const Time& rhs);
bool operator<=(const Time& lhs, const Time& rhs);
std::ostream& operator<<(std::ostream& os, const Time& t);

// Class which encapsulates randomization functionality associated
// with a particular seed value.
class RandomSource {
  using mt_type = std::mt19937_64;
  using seed_type = std::mt19937_64::result_type;

 public:
  RandomSource(seed_type seed = 1);

  // Initial random seed against which state was initiailization. Not
  // necessarily representative of the randomization state after
  // random values have been created.
  seed_type seed() const { return seed_; }

  // Emit a random boolean value with true probabiliy according to a
  // Bernoulli distribution.
  bool random_bool(float true_probability = 0.5f);

  // Emit a random integral type in the closed-interval [min, max].
  template <typename T>
  T uniform(const T min = std::numeric_limits<T>::min(),
            const T max = std::numeric_limits<T>::max()) {
    std::uniform_int_distribution<T> dist(min, max);
    return dist(mt_);
  };

 private:
  seed_type seed_;
  mt_type mt_;
};

class LogContext {
 public:
  LogContext(std::ostream* os = std::addressof(std::cout));
  ~LogContext();

  std::ostream& os() { return *os_; }

  void info(const std::string& name, bool nl = true);

 private:
  std::ostream* os_ = nullptr;
};

enum class RunMode { ToExhaustion, ForTime };

// clang-format off
#define PHASES(__func)				\
  __func(Build)					\
  __func(PreElabDrc)		                \
  __func(Elab)					\
  __func(Drc)					\
  __func(Init)					\
  __func(Run)					\
  __func(Fini)
// clang-format on

enum class Phase {
#define __declare_enum(__name) __name,
  PHASES(__declare_enum)
#undef __declare_enum
};

const char* to_string(Phase phases);

class Object {
  friend class ObjectVisitor;
  DECLARE_VISITEE(Object);

 public:
  Object(Kernel* k, const std::string& name);
  virtual ~Object();

  // Current kernel instance.
  Kernel* k() const { return k_; }
  // Flag indicating is current object is the root of the object tree.
  bool is_top() const { return parent_ == nullptr; }
  // Object path in object model
  std::string path() const;
  // Object name
  std::string name() const { return name_; }

  // Set current object as object top.
  void set_top();
  // Set parent object.
  void set_parent(Object* o) { parent_ = o; }
  // Add child object.
  bool add_child(Object* c);
  //
  Object* find_path(const std::string& path);

 protected:
  void iterate_children(ObjectVisitor* visitor);

 private:
  Object* find_path(std::vector<std::string>& path);

  // Kernel
  Kernel* k_ = nullptr;
  // Parent object
  Object* parent_ = nullptr;
  // Children objects
  std::vector<Object*> children_;
  // Path (lazily constructed)
  mutable std::string path_;
  // Objec name
  std::string name_;
};

class Loggable : public Object {
  DECLARE_VISITEE(Loggable);

 protected:
  struct Level {
    enum : int { Fatal = 0, Error, Warning, Info, Debug };
    Level() : level_(Debug) {}
    Level(int level) : level_(level) {}
    operator int() const { return level_; }
    int level() const { return level_; }
    char to_char() const { return str()[0]; }
    const char* str() const {
      switch (level_) {
        case Debug:
          return "DEBUG";
          break;
        case Info:
          return "INFO";
          break;
        case Warning:
          return "WARNING";
          break;
        case Error:
          return "ERROR";
          break;
        case Fatal:
          return "FATAL";
          break;
        default:
          return "X";
          break;
      }
    }

   private:
    int level_;
  };

  struct LogMessage {
    LogMessage(const std::string& msg, Level level = Level::Info)
        : msg_(msg), level_(level) {}
    Level level() const { return level_; }
    std::string msg() const { return msg_; }

    bool suppress_except() const { return suppress_except_; }
    void suppress_except(bool s = true) { suppress_except_ = s; }
    void level(Level level) { level_ = level; }
    LogMessage& append(const std::string& str);
    void clear() { msg_.clear(); }

   private:
    bool suppress_except_ = false;
    std::string msg_;
    Level level_;
  };

 public:
  //
  Loggable(Kernel* k, const std::string& name);

  Level level() const { return level_; }
  void level(const Level& level) { level_ = level; }

  //
  void log(const LogMessage& msg) const;

 private:
  void log_prefix(Level l, std::ostream& os) const;

  Level level_;
};

//
//
class Action : public Loggable {
  friend class Process;
  DECLARE_VISITEE(Action);

 public:
  Action(Kernel* k, const std::string& name);
  virtual ~Action() = default;

  virtual bool eval() = 0;
  virtual void release();
};

//
//
class Event : public Loggable {
  friend class Process;
 public:
  Event(Kernel* k, const std::string& name);
  ~Event();

  // Has awaiting processes.x
  bool has_awaitees() const { return !as_.empty(); }

  // Wait awaiteeseventor
  void notify();

  // Add 'action' to be evaluated upon event notification.
  void add_notify_action(Action* a) { as_.push_back(a); }

 private:
  void add_waitee(Process* p);
  //
  std::vector<Action*> as_;
};

// Event subtype to model the composition of an "or-list" of events.
//
class EventOr : public Event {
 public:
  EventOr(Kernel* k, const std::string& name);

  // Add child event.
  void add_child_event(Event* child) { childs_.push_back(child); }
  void finalize();

 private:
  std::vector<Event*> childs_;
};

//
//
class Process : public Loggable {
  friend class Module;
  DECLARE_VISITEE(Process);

 public:
  Process(Kernel* k, const std::string& name);
  virtual ~Process() = default;

  // Initialization routine (called after elaboration).
  virtual void init() {}

  // Finalization routine (classed at end of simulation).
  virtual void fini() {}

  // Evaluation routine (called upon satisfaction of 'wait' condition).
  virtual void eval() {}

  // Re-evaluate on the next delta cycle.
  virtual void next_delta();

  // Suspend/Re-evaulate process after delay.
  virtual void wait_for(Time t);

  // Suspend/Re-evaulate process at time.
  virtual void wait_until(Time t);

  // Suspend/Re-evaluate process upon the notification of event.
  virtual void wait_on(Event* event);

  //
  virtual void invoke_init();

  //
  virtual void invoke_eval();
};

//
//
class ProcessHost : public Loggable {
  DECLARE_VISITEE(ProcessHost);

 public:
  ProcessHost(Kernel* k, const std::string& name);

  // Add evaluate-able process
  virtual void add_child_process(Process* p);

 private:
  std::vector<Process*> ps_;
};

//
//
class Module : public ProcessHost {
  friend class Kernel;
  DECLARE_VISITEE(Module);

 protected:
  Module(Kernel* k, const std::string& name);

 public:
  virtual ~Module();

  // Invoke elaboration.
  virtual void elab() {}
  // Run Design Rule Check.
  virtual void drc() {}

  // A child objects.
  void add_child_module(Module* m);

 private:
  std::vector<Module*> ms_;
};

//
//
class Kernel : public Module {
  friend class Process;
  friend class Module;
  friend class Object;

  using seed_type = std::mt19937_64::result_type;

  struct Event {
    Time time;
    Action* action;
  };

  struct EventComparer {
    bool operator()(const Event& lhs, const Event& rhs) {
      // TODO: create operators
      if (lhs.time.time > rhs.time.time) return true;
      if (lhs.time.time < rhs.time.time) return false;
      return lhs.time.delta < rhs.time.delta;
    }
  };

 public:
  Kernel(seed_type seed = 1);

  // Observers
  Time time() const { return time_; }
  std::size_t events_n() const { return eq_.size(); }
  bool fatal() const { return fatal_; }
  RandomSource& random_source() { return random_source_; }
  LogContext& log_context() { return log_context_; }
  Phase phase() const { return phase_; }
  Object* top() const { return top_; }

  // Deprecate this method;
  void run(RunMode r = RunMode::ToExhaustion, Time t = Time{});
  void add_action(Time t, Action* a);
  void raise_fatal() { fatal_ = true; }
  void set_seed(seed_type seed);

  //private:
  // Set current simulation phase.
  void set_phase(Phase phase) { phase_ = phase; }
  void set_top(Object* top) { top_ = top; }

  // Simulation phases invocations.
  void invoke_elab();
  void invoke_drc();
  void invoke_init();
  void invoke_run(RunMode r, Time t = Time{});
  void invoke_fini();

  // Simulation event queue.
  std::vector<Event> eq_;
  // Current simulation time.
  Time time_;
  // Flag denoting that a fatal error has occurred.
  bool fatal_{false};
  // Current random state.
  RandomSource random_source_;
  //
  LogContext log_context_;
  // Current simulation phase.
  Phase phase_{Phase::Build};
  // Simulation top-level.
  Object* top_{nullptr};
};

//
//
class TopModule : public Module {
  DECLARE_VISITEE(TopModule);

 public:
  TopModule(Kernel* k, const std::string& name);
};

}  // namespace cc::kernel

#endif  // CC_KERNEL_H
