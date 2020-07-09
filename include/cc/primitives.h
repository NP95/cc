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

#ifndef CC_INCLUDE_CC_PRIMITIVES_H
#define CC_INCLUDE_CC_PRIMITIVES_H

#include <vector>

#include "cc/kernel.h"
#include "cc/types.h"

namespace cc {

class Message;

// Class type used to model a generic clock source; a periodic and
// deterministic tick from which to initiate other actions.
//
class Clock : public kernel::Module {
 public:
  Clock(kernel::Kernel* k, const std::string& name, int ticks, int period = 10);

  int ticks() const { return ticks_; }
  int period() const { return period_; }

  kernel::Event& rising_edge_event() { return rising_edge_event_; }
  const kernel::Event& rising_edge_event() const { return rising_edge_event_; }

 private:
  kernel::Event rising_edge_event_;
  kernel::Process* p_;
  int ticks_, period_;
};

// Class type used to model the behavior of a queue type data
// structure with events corresponding to enqueue and dequeue events
// where necessary.
//
template <typename T>
class Queue : public kernel::Module {
 public:
  Queue(kernel::Kernel* k, const std::string& name, std::size_t n)
      : kernel::Module(k, name),
        n_(n),
        enqueue_event_(k, "enqueue_event"),
        dequeue_event_(k, "dequeue_event"),
        non_empty_event_(k, "non_empty_event"),
        non_full_event_(k, "non_full_event") {
    empty_ = true;
    full_ = false;
    wr_ptr_ = 0;
    rd_ptr_ = 0;
    size_ = 0;
    ts_.resize(n);
  }

  // The capacity of the queue.
  std::size_t n() const { return n_; }
  // The number of free entries in the queue.
  std::size_t free() const { return n() - size(); }
  // The occupancy of the queue.
  std::size_t size() const { return size_; }
  // Flag denoting full status of the queue.
  bool full() const { return full_; }
  // Flag denoting empty status of the queue.
  bool empty() const { return empty_; }
  // kernel::Event notified on the enqueue of an entry into the queue.
  kernel::Event& enqueue_event() { return enqueue_event_; }
  // kernel::Event notified on the dequeue of an entry into the queue.
  kernel::Event& dequeue_event() { return dequeue_event_; }
  // kernel::Event notified on the transition to non-empty state.
  kernel::Event& non_empty_event() { return non_empty_event_; }
  // kernel::Event notified on the transition out of the full state.
  kernel::Event& non_full_event() { return non_full_event_; }

  // Enqueue entry into queue; returns true on success.
  bool enqueue(const T& t) {
    if (full()) return false;

    ts_[wr_ptr_] = t;
    if (++wr_ptr_ == n()) wr_ptr_ = 0;

    // If was empty, not empty after an enqueue therefore notify,
    // awaitees waiting for the queue become non-empty.
    if (empty()) non_empty_event_.notify();

    empty_ = false;
    full_ = (++size_ == n_);

    enqueue_event_.notify();
    return true;
  }

  bool peek(T& t) const {
    if (empty()) return false;

    t = ts_[rd_ptr_];
    return true;
  }

  // Dequeue entry from queue; returns false on success.
  bool dequeue(T& t) {
    if (empty()) return false;

    t = ts_[rd_ptr_];

    // If was full, not full after dequeue therefore notify non-full
    // event to indicate transition away from full state.
    if (full()) non_full_event_.notify();

    if (++rd_ptr_ == n()) rd_ptr_ = 0;
    empty_ = (--size_ == 0);
    full_ = false;

    dequeue_event_.notify();
    return true;
  }

 private:
  bool full_, empty_;
  std::size_t wr_ptr_, rd_ptr_;
  std::size_t n_, size_;
  std::vector<T> ts_;

  kernel::Event enqueue_event_;
  kernel::Event dequeue_event_;
  kernel::Event non_empty_event_;
  kernel::Event non_full_event_;
};

//
//
template <typename T>
class Arbiter : public kernel::Module {
 public:
  // Helper class which encapsulates the concept of a single
  // arbitration round.
  class Tournament {
    friend class Arbiter;
    Tournament(Arbiter* parent) : parent_(parent) { execute(); }

   public:
    Tournament() : parent_(nullptr) {}
    // Return the winning requester interface.
    kernel::RequesterIntf<T>* intf() const { return intf_; }
    bool has_requester() const { return intf_ != nullptr; }
    bool deadlock() const { return deadlock_; }

    // Advance arbitration state to the next index if prior
    // arbitration has succeeded.
    void advance() const {
      if (intf_ != nullptr) {
        parent_->idx_ = (idx_ + 1) % parent_->n();
      }
    }

   private:
    void execute() {
      std::size_t requesters = 0;
      intf_ = nullptr;
      for (std::size_t i = 0; i < parent_->n(); i++) {
        // Compute index of next requester interface in roundrobin order.
        idx_ = (parent_->idx_ + i) % parent_->n();
        kernel::RequesterIntf<T>* cur = parent_->intfs_[idx_];

        if (!cur->has_req()) continue;
        // Current agent is requesting, proceed.

        requesters++;
        if (!cur->blocked()) {
          // Current agent is requesting and is not blocked by some
          // protocol condition.
          intf_ = cur;
          return;
        }
      }
      // A deadlock has occurred iff there are pending work items in the
      // child queues, but all of these queues are currently blocked
      // awaiting the completion of some other action.
      deadlock_ = (requesters == parent_->n());
    }
    bool deadlock_ = false;
    kernel::RequesterIntf<T>* intf_ = nullptr;
    Arbiter* parent_ = nullptr;
    std::size_t idx_ = 0;
  };

  Arbiter(kernel::Kernel* k, const std::string& name)
      : kernel::Module(k, name) {
    build();
  }
  virtual ~Arbiter() = default;

  // The number of requesting agents.
  std::size_t n() const { return intfs_.size(); }
  // Event denoting rising edge to the ready to grant state.
  kernel::Event& request_arrival_event() { return *request_arrival_event_; }
  // Initiate an arbitration tournament.
  Tournament tournament() { return Tournament(this); }
  // Add a requester to the current arbiter (Build-/Elaboration-Phases only).
  void add_requester(kernel::RequesterIntf<T>* intf) { intfs_.push_back(intf); }

 private:
  void build() {
    request_arrival_event_ = new kernel::EventOr(k(), "request_arrival_event");
    add_child(request_arrival_event_);
  }

  void elab() override {
    if (!intfs_.empty()) {
      // Construct EventOr denoting the event which is notified when the
      // arbiter goes from having no requestors to having non-zero
      // requestors.
      for (kernel::RequesterIntf<T>* r : intfs_) {
        request_arrival_event_->add_child_event(&r->request_arrival_event());
      }
      request_arrival_event_->finalize();
    } else {
      const LogMessage msg{"Arbiter has no associated requestors.",
                           Level::Error};
      log(msg);
    }
  }

  void drc() override {}

  //
  kernel::EventOr* request_arrival_event_ = nullptr;
  // Current arbitration index.
  std::size_t idx_ = 0;
  //
  std::vector<kernel::RequesterIntf<T>*> intfs_;
};

//
//
template<typename T>
class Table : public kernel::Module {

  struct Entry {
    // Flag denoting validity of table.
    bool is_valid = false;
    // Table entry state.
    T t;
  };

  using raw_iterator = typename std::vector<Entry>::iterator;
  
 public:

  class Iterator {
   public:
    Iterator() : it_() {}
    Iterator(raw_iterator it) : it_(it) {}

    // Iterator operators
    T& operator*() { return it_->t; }
    const T& operator*() const { return it_->t; }
    Iterator& operator++() { ++it_; return *this; }
    Iterator operator++(int) const { return Iterator(it_ + 1); }
    raw_iterator operator->() const { return it_; }
    bool operator==(const Iterator& rhs) const{ return it_ == rhs.it_; }
    bool operator!=(const Iterator& rhs) const { return !operator==(rhs.it_); }

   private:
    // Pointer to underlying state in parent structure.
    raw_iterator it_;
  };


  // Utility class to perform a verify of operation on a transaction
  // table.
  struct Manager {
    Manager(Iterator begin, Iterator end)
        : begin_(begin), end_(end)
    {}

    Iterator begin() const { return begin_; }
    Iterator end() const { return end_; }

    Iterator first_invalid() const {
      Iterator it = begin_;
      while (it != end_) {
        if (!it->is_valid) break;
        ++it;
      }
      return it;
    }

    Iterator find_transaction(addr_t addr) {
      // TODO
      return end_;
    }
  
    bool is_full() const {
      return first_invalid() == end();
    }

   private:
    Iterator begin_, end_;
  };
  
  Table(kernel::Kernel* k, const std::string& name, std::size_t n)
      : Module(k, name), n_(n) {
    t_.resize(n);
  }

  // Accessors:
  // Table size.
  std::size_t n() const { return n_; }
  // Number of unused slots
  std::size_t free_n() const { return free_n_; }
  // Flag denoting whether table is empty.
  bool is_empty() const { return n() == free_n(); }
  // Flag denoting whether table is fully utilized.
  bool is_full() const { return free_n() == 0; }

  Iterator begin() { return Iterator(t_.begin()); }
  Iterator end() { return Iterator(t_.end()); }

  bool install(Iterator it, const T& t) {
    return false;
  }

 private:
  // Count of unused locations in table.
  std::size_t free_n_;
  // Table state.
  std::vector<Entry> t_;
  // Table size
  std::size_t n_;
};

//
//
class MessageQueue : public kernel::Module,
                     public kernel::EndPointIntf<const Message*>,
                     public kernel::RequesterIntf<const Message*> {
 public:
  MessageQueue(kernel::Kernel* k, const std::string& name, std::size_t n);

  // Queue depth.
  std::size_t n() const { return q_->n(); }

  // Endpoint Interface:
  void push(const Message* msg) override;

  // Requester Interface:
  bool has_req() const override;
  const Message* peek() const override;
  const Message* dequeue() override;
  kernel::Event& request_arrival_event() override;

 private:
  // Construct module
  void build(std::size_t n);
  // Queue primitive.
  Queue<const Message*>* q_ = nullptr;
};

//
//
class Agent : public kernel::Module {
 public:
  Agent(kernel::Kernel* k, const std::string& name);

 protected:
  // (Run-Phase only)
  void issue(MessageQueue* mq, const kernel::Time& time, const Message* msg);
};

}  // namespace cc

#endif
