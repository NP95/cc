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

#include "kernel.h"
#include "protocol.h"

namespace cc {

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

template<typename T>
class RequesterIntf {
 public:
  virtual ~RequesterIntf() = default;

  // Return flag indicating whether the current agent is currently
  // requesting.
  virtual bool is_requesting() const = 0;

  // Acknowledge success of agent's participation in current
  // tournament.
  virtual void grant() = 0;

  // Reference to underlying state being arbitrated.
  virtual T& data() = 0;
  virtual const T& data() const = 0;

  // Event notified on rising-edge of requester event.
  virtual kernel::Event& request_arrival_event() = 0;
};


template<typename T>
class Arbiter : public kernel::Module {
 public:
  Arbiter(kernel::Kernel* k, const std::string& name);
  virtual ~Arbiter() = default;

  // Event denoting rising edge to the ready to grant state.
  kernel::Event& grant_event() { return grant_event_; }

  T& data();
  const T& data() const;

  void add_requester(RequesterIntf<T>* intf) { intfs_.push_back(intf); }
 private:

  void elaborate() override {
    // Construct EventOr denoting the event which is notified when the
    // arbiter goes from having no requestors to having non-zero
    // requestors.
    for (RequesterIntf<T>* r : intfs_) {
      request_or_event_.add_child(&r->request_arrival_event());
    }
    request_or_event_.finalize();
  }

  void drc() override {
  }

  void init() override {
  }

  void fini() override {
  }
  
  kernel::Event grant_event_;
  kernel::EventOr request_or_event_;
  std::vector<RequesterIntf<T>*> intfs_;
};


// Abstract base class encapsulating the concept of a transaction
// source; more specifically, a block response to model the issue of
// of load or store instructions to a cache.
//
class TransactionSource : public kernel::Module {
 protected:

  // Transaction frontier record.
  struct Frontier {
    Transaction* t;
    kernel::Time time;
  };
  
 public:
  TransactionSource(kernel::Kernel* k, const std::string& name);
  virtual ~TransactionSource() = default;

  // Flag denoting whether the transaction source has been exhausted.
  virtual bool done() const { return done_; }
  // Event notified whenever a transaction has become available.
  kernel::Event& matured_event() { return matured_event_; }
  // Event notified whenever the source has been exhausted.
  kernel::Event& exhausted_event() { return exhausted_event_; }

  // Transaction at the head of the source queue; nullptr if source
  // has been exhausted.
  Transaction* head();

  // Consume transaction at the head of the source queue.
  virtual void consume();

 protected:

  virtual bool cb_replenish() = 0;

  // Child indicates that all transaction have been issued.
  virtual void set_done() { done_ = true; }

 private:
  kernel::Event matured_event_;
  kernel::Event exhausted_event_;
  bool done_ = false;
  std::vector<Frontier> ts_;
};


// Elementary realization of a transaction source. Transactions are
// programmatically constructed and issued to the source before the
// start of the simulation. Upon exhaustion of the transactions
// the source remained exhausted for the duration of the simulation.
//
class ProgrammaticTransactionSource : public TransactionSource {
 public:
  ProgrammaticTransactionSource(kernel::Kernel* k, const std::string& name);

  void add_command(Opcode opcode, kernel::Time t);

 private:
  bool cb_replenish() override;
};

}  // namespace cc

#endif
