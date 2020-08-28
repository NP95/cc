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

// Forwards:
namespace cc {
class Soc;
class MessageQueue;
}

namespace cc::kernel {

// Forwards
class Kernel;
class Module;
class Process;
class ProcessHost;
class Action;
class Loggable;
class Object;

struct ObjectVisitor {
  virtual ~ObjectVisitor() = default;
  void iterate(Object* root);

  virtual void visit(Module* o) {}
  virtual void visit(ProcessHost* o) {}
  virtual void visit(Process* o) {}
  virtual void visit(Action* o) {}
  virtual void visit(Loggable* o) {}
  virtual void visit(Object* o) {}
};

#define DECLARE_VISITEE(__name)                 \
  friend class ObjectVisitor;                   \
  virtual void accept(ObjectVisitor* visitor) { \
    visitor->visit(this);                       \
    Object::iterate_children(visitor);          \
  }                                             \
  virtual const char* type_str() const { return #__name; }

struct Time {
  using time_type = std::uint64_t;
  using delta_type = std::uint32_t;

  std::string to_string() const;

  // TODO: consider changing to a tuple type.
  time_type time = 0;
  delta_type delta = 0;
};

// Convert time to human readable string.
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

// Stimulation run-mode (halting condition).
enum class RunMode { ToExhaustion, ForTime };

// Simulation phasea
enum class Phase { Build, PreElabDrc, Elab, Drc, Init, Run, Fini };

// Phase to human readable string
const char* to_string(Phase phases);

// Basic Object class denoting an addressable item in the simulations
// object heirarchy.
//
class Object {
  friend class ObjectVisitor;
  DECLARE_VISITEE(Object);

 public:
  Object(Kernel* k, const std::string& name);
  virtual ~Object();

  // Accessors:

  // Current kernel instance.
  Kernel* k() const { return k_; }

  // Flag indicating is current object is the root of the object tree.
  bool is_top() const { return parent_ == nullptr; }

  // Object path in object model
  std::string path() const;

  // Object name
  std::string name() const { return name_; }


  // Setters:

  // Set current object as object top.
  void set_top();

  // Set parent object.
  void set_parent(Object* o) { parent_ = o; }

  // Add child object.
  bool add_child(Object* c);

  // Search for an object relative to the current item and return a
  // pointer to the item, or nullptr if not found.
  Object* find_path(const std::string& path);

 protected:
  void iterate_children(ObjectVisitor* visitor);

 private:
  // Find item in object heirarchy from split list of paths.
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

// Object from which log messages can be issued.
//
class Loggable : public Object {
  DECLARE_VISITEE(Loggable);

 protected:

  
  struct Level {
    enum : std::uint32_t { Fatal = 0, Error, Warning, Info, Debug };

    // Default level
    Level() : level_(Debug) {}

    // From level as integer
    Level(std::uint32_t level) : level_(level) {}

    // Implicit conversion to integer.
    operator std::uint32_t() const { return level_; }

    // Accessors:

    // Level as integer.
    std::uint32_t level() const { return level_; }

    // To single character.
    char to_char() const { return "FEWID"[level_]; }

    // To human readable string
    const char* str() const;

   private:
    // Current level
    std::uint32_t level_;
  };


  // Log Message
  //
  struct LogMessage {

    // Construct empty message
    LogMessage() = default;

    // Construct message with 'msg' at level 'level'
    LogMessage(const std::string& msg, Level level = Level::Info)
        : msg_(msg), level_(level) {}


    // Accessors:

    // Message level.
    Level level() const { return level_; }

    // Message string.
    std::string msg() const { return msg_; }

    // Flag indiciating that exceptions have been surpressed for
    // current message.
    bool suppress_except() const { return suppress_except_; }


    // Setters:

    // Set current message level.
    void set_level(Level level) { level_ = level; }

    // Set suppress exceptions.
    void set_suppress_except(bool s = true) { suppress_except_ = s; }


    // Append string to current message
    LogMessage& append(const std::string& str);

    // Clear message
    void clear() { msg_.clear(); }

   private:
    // Suppress exception associated with level (if applicable)
    bool suppress_except_ = false;

    // Message string
    std::string msg_;

    // Message level
    Level level_;
  };

 public:
  //
  Loggable(Kernel* k, const std::string& name);

  // Accessors:

  // Current log level.
  Level level() const { return level_; }


  // Setters:

  // Set current log level.
  void set_level(const Level& level) { level_ = level; }


  // Issue log message to logger.
  void log(const LogMessage& msg) const;

 private:
  // Write standard log prefix.
  void log_prefix(Level l, std::ostream& os) const;

  // Current log level.
  Level level_ = Level::Debug;
};

// Base schedulable object class. Derive 'actions' to be scheduled and
// executed at some future time by the simulation kernel.
//
class Action : public Loggable {
  friend class Process;
  DECLARE_VISITEE(Action);

 public:
  Action(Kernel* k, const std::string& name);
  virtual ~Action() = default;

  // Evaluate action; override in derived class.
  virtual bool eval() = 0;

  // Release (deallocate) object.
  virtual void release();
};


// Event; Observer class from which processes can be sensitive. When
// notified, processes registered with the class are scheduled to be
// evaluated in the following simulation delta-cycle,
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
  // Add process to set of entites awaiting notification.
  void add_waitee(Process* p);

  // Set of actions to be evaluated on notification.
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
  // Child events which when triggered cause the current EventOr to
  // itself be triggered.
  std::vector<Event*> childs_;
};


// Basic schedulable item base class; derive from this some class
// which defines some independent thread of execution.
//
class Process : public Loggable {
  friend class Module;
  friend class InvokeInitVisitor;
  friend class EvalProcessAction;
  
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

 private:
  // Invoke initialization phase; detect whether a wait condition has
  // been set upon completion.
  virtual void invoke_init();

  // Invoke evaluation phase; detect whether a wait condition has
  // been set upon completion.
  virtual void invoke_eval();
};


// Object which is capable of hosting some number of execution
// contexts (Processes).
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

  // Invoke elaboration. Returns true if requests a repeated
  // invocation.
  virtual bool elab() { return false; }
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
  friend class SimPhaseRunner;
  friend class ActionAdder;

  using seed_type = std::mt19937_64::result_type;

  struct FrontierItem {
    Time time;
    Action* action;
  };

  struct FrontierItemComparer;

 public:
  Kernel(seed_type seed = 1);

  // Accessors:

  // Current simulation time.
  Time time() const { return time_; }

  // Number of actions presently in the event queue.
  std::size_t events_n() const { return eq_.size(); }

  // Flag indicating that a fatal error has occurred.
  bool fatal() const { return fatal_; }

  // Reference to current randomization state.
  RandomSource& random_source() { return random_source_; }

  // Reference to current logging context.
  LogContext& log_context() { return log_context_; }

  // Current simulation phase.
  Phase phase() const { return phase_; }

  // Top-level module instance.
  Object* top() const { return top_; }

  void raise_fatal() { fatal_ = true; }

  // Set random ssed.
  void set_seed(seed_type seed);

 private:
  // Add action 'a' to be invoked at time 't'.
  void add_action(Time t, Action* a);

  // Set current simulation phase.
  void set_phase(Phase phase) { phase_ = phase; }

  // Set simulation top-level
  void set_top(Object* top) { top_ = top; }

  // Simulation phases invocations:

  // Invoke elaboration
  void invoke_elab();

  // Invoke Design Rule Check (DRC)
  void invoke_drc();

  // Invoke initialization
  void invoke_init();

  // Invoke run
  void invoke_run(RunMode r, Time t = Time{});

  // Invoke finalization.
  void invoke_fini();


  // Simulation event queue.
  std::vector<FrontierItem> eq_;
  // Current simulation time.
  Time time_;
  // Flag denoting that a fatal error has occurred.
  bool fatal_{false};
  // Current random state.
  RandomSource random_source_;
  // Log context
  LogContext log_context_;
  // Current simulation phase.
  Phase phase_{Phase::Build};
  // Simulation top-level.
  Object* top_{nullptr};
};

// Class to represent, top-level module in the object-hierarchy (an
// object with no parent).
//
class TopModule : public Module {
  DECLARE_VISITEE(TopModule);

 public:
  TopModule(Kernel* k, const std::string& name);
};


// Helper class to invoke various simulation phases within the
// simulation kernel instance.
//
class SimPhaseRunner {
 public:
  SimPhaseRunner(Kernel* k) : k_(k) {}

  // Invoke elaboration
  void elab() const;

  // Invoke Design Rule Check
  void drc() const;

  // Invoke initialization.
  void init() const;

  // Invoke run.
  void run(RunMode r = RunMode::ToExhaustion, Time time = Time{}) const;

  // Invoke initialization.
  void fini() const;

 private:
  // Kernel instance
  Kernel* k_ = nullptr;
};

// Invoke simulation phases on kernel instance. Helper class to all
// relevant state to be hidden from the top-level Kernel class.
//
class SimSequencer {
 public:
  SimSequencer(Kernel* k) : k_(k) {}
  
  // Run/Invoke simulation.
  void run(RunMode r = RunMode::ToExhaustion, Time time = Time{}) const;

 private:
  // Kernel instance.
  Kernel* k_ = nullptr;
};

// Helper class to hide the add_action method from the top-level
// kernel instance.
class ActionAdder {
 public:
  ActionAdder(Kernel* k) : k_(k) {}

  // Add action to kernel instance.
  void add_action(Time t, Action* a) const;

 private:
  Kernel* k_ = nullptr;
};

}  // namespace cc::kernel

#endif  // CC_KERNEL_H
