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

#ifndef CC_SIM_H
#define CC_SIM_H

#include "kernel.h"
#include <vector>

namespace cc {

// Class type used to model a generic clock source; a periodic and
// deterministic tick from which to initiate other actions.
//
class Clock : public Module {
 public:
  Clock(Kernel* k, const std::string& name, int ticks, int period = 10);

  int ticks() const { return ticks_; }
  int period() const { return period_; }

  Event& rising_edge_event() { return rising_edge_event_; }
  const Event& rising_edge_event() const { return rising_edge_event_; }

 private:
  Event rising_edge_event_;
  Process* p_;
  int ticks_, period_;
};

// Class type used to model the behavior of a queue type data
// structure with events corresponding to enqueue and dequeue events
// where necessary.
//
template<typename T>
class Queue : public Module {
 public:
  Queue(Kernel* k, const std::string& name, std::size_t n)
      : Module(k, name), n_(n),
        enqueue_event_(k), dequeue_event_(k), non_empty_event_(k), non_full_event_(k) {
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
  // Event notified on the enqueue of an entry into the queue.
  Event& enqueue_event() { return enqueue_event_; }
  // Event notified on the dequeue of an entry into the queue.
  Event& dequeue_event() { return dequeue_event_; }
  // Event notified on the transition to non-empty state.
  Event& non_empty_event() { return non_empty_event_; }
  // Event notified on the transition out of the full state.
  Event& non_full_event() { return non_full_event_; }

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

  Event enqueue_event_;
  Event dequeue_event_;
  Event non_empty_event_;
  Event non_full_event_;
};

} // namespace cc


#endif
