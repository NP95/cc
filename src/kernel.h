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
#include <random>
#include <limits>
#include <sstream>

namespace cc::kernel {

// Forwards:
class Process;
class Action;

struct Time {
  using time_type = std::uint32_t;
  using delta_type = std::uint32_t;

  Time() = default;

  // TODO: consider changing to a tuple type.
  time_type time = 0;
  delta_type delta = 0;
};

Time operator+(const Time& lhs, const Time& rhs);
bool operator<(const Time& lhs, const Time& rhs);
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
  template<typename T>
  T uniform(const T min = std::numeric_limits<T>::min(),
            const T max = std::numeric_limits<T>::max()) {
    std::uniform_int_distribution<T> dist(min, max);
    return dist(mt_);
  };

 private:
  seed_type seed_;
  mt_type mt_;
};

enum class RunMode { ToExhaustion, ForTime };

class Kernel {
  friend class Process;

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

  void run(RunMode r = RunMode::ToExhaustion, Time t = Time{});
  void add_action(Time t, Action* a);
  void raise_fatal() { fatal_ = true; }
  void set_seed(seed_type seed);
 private:
  // Simulation event queue.
  std::vector<Event> eq_;
  // Current simulation time.
  Time time_;
  // Flag denoting that a fatal error has occurred.
  bool fatal_;
  // Current random state.
  RandomSource random_source_;
};

class Object {
 public:
  Object(Kernel* k, const std::string& name);
  virtual ~Object();

  //
  Kernel* k() const { return k_; }
  bool is_top() const { return parent_ == nullptr; }
  std::string path() const;
  std::string name() const { return name_; }

  //
  void set_top() { parent_ = nullptr; }
  void set_parent(Object* o) { parent_ = o; }
  void add_child(Object* c);

 private:
  Kernel* k_ = nullptr;
  Object* parent_ = nullptr;
  std::vector<Object*> children_;
  mutable std::string path_;
  std::string name_;
};

class Loggable : public Object {
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
        case Debug: return "DEBUG"; break;
        case Info: return "INFO"; break;
        case Warning: return "WARNING"; break;
        case Error: return "ERROR"; break;
        case Fatal: return "FATAL"; break;
        default: return "X"; break;
      }
    }
   private:
    int level_;
  };

  struct Message {
    Message(const std::string& msg, Level level = Level::Info)
        : msg_(msg), level_(level) {}
    Level level() const { return level_; }
    std::string msg() const { return msg_; }

    void level(Level level) { level_ = level; }
    Message& append(const std::string& str);
   private:
    std::string msg_;
    Level level_;
  };

 public:
  //
  Loggable(Kernel* k, const std::string& name);

  Level level() const { return level_; }
  void level(const Level& level) { level_ = level; }

  //
  void log(const Message& msg) const;
 private:
  void log_prefix(Level l, std::ostream& os) const;

  Level level_;
};

class Action : public Loggable {
  friend class Process;

 public:
  Action(Kernel* k, const std::string& name);
  virtual ~Action() = default;

  virtual bool eval() = 0;
  virtual void release();
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
  Process(Kernel* k, const std::string& name);
  virtual ~Process() = default;

  // Initialization routine (called after elaboration).
  virtual void init() {}

  // Evaluation routine (called upon satisfaction of 'wait' condition).
  virtual void eval() {}

  // Suspend/Re-evaulate process after delay.
  virtual void wait_for(Time t);

  // Suspend/Re-evaulate process at time.
  virtual void wait_until(Time t);

  // Suspend/Re-evaluate process upon the notification of event.
  virtual void wait_on(Event& event);
};

class Module : public Loggable {
  friend class Kernel;

 protected:
  Module(Kernel* k, const std::string& name);
 public:
  virtual ~Module();

  // Initialization routine.
  virtual void init();
  // Finalization routine.
  virtual void fini();

  // A child objects.
  void add_child(Module* m);
  void add_child(Process* p);

 private:
  std::vector<Module*> ms_;
  std::vector<Process*> ps_;
};

} // namespace cc::kernel

#endif //CC_KERNEL_H
