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
#include <string>

namespace cc {

// Forwards:
class Process;
class Action;

struct Time {
  using cycle_type = std::uint32_t;
  using delta_type = std::uint32_t;

  Time() = default;
  
  cycle_type cycle = 0;
  delta_type delta = 0;
};

bool operator<(const Time& lhs, const Time& rhs);
std::ostream& operator<<(std::ostream& os, const Time& t);

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
  std::size_t events_n() const { return eq_.size(); }

  void run(RunMode r = RunMode::ToExhaustion, Time t = Time{});

  void add_action(Time t, Action* a);
 private:
  std::vector<Event> eq_;
  Time time_;
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

struct LogMessage {
  LogMessage(const std::string& msg) : msg_(msg) {}
  std::string msg() const { return msg_; }
 private:
  std::string msg_;
};

class Loggable : public Object {
  enum class Level { Debug, Info, Warning, Error, Fatal };
 public:
  //
  Loggable(Kernel* k, const std::string& name);

  //
  void report_debug(const LogMessage& msg) const;
  void report_info(const LogMessage& msg) const;
  void report_warning(const LogMessage& msg) const;
  void report_error(const LogMessage& msg) const;
  void report_fatal(const LogMessage& msg) const;
 private:
  void report_prefix(Level l, std::ostream& os) const;
  char level_to_char(Level l) const;
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
  Process(Kernel* k, const std::string& name);
  virtual ~Process() = default;

  //
  virtual void init() {}
  virtual void eval() {}

  //
  virtual void wait_until(Time t);
  virtual void wait_on(Event& event);
};

class Module : public Loggable {
  friend class Kernel;

 protected:
  Module(Kernel* k, const std::string& name);
 public:
  //
  virtual ~Module();

  //
  virtual void init();
  virtual void fini();

  //
  void add_child(Module* m);
  void add_process(Process* p);

 private:
  std::vector<Module*> ms_;
  std::vector<Process*> ps_;
};

}

#endif //CC_KERNEL_H
