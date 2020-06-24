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

#include "gtest/gtest.h"
#include "sim.h"

TEST(Sim, BasicClock) {
  struct OnRisingEdgeProcess : public cc::Process {
    OnRisingEdgeProcess(cc::Kernel* k, cc::Clock* clk)
        : cc::Process(k, "OnRisingEdgeProcess"), clk_(clk) {}
    int n() const { return n_; }
    void init() override {
      wait_on(clk_->rising_edge_event());
    }
    void eval() override {
      const Message msg("On rising edge");
      log(msg);
      wait_on(clk_->rising_edge_event());
      n_++;
    }
   private:
    int n_ = 0;
    cc::Clock* clk_;
  };

  struct TopLevel : public cc::Module {
    TopLevel(cc::Kernel* k, int n) : cc::Module(k, "top"), n_(n) {
      set_top();
      clk_ = new cc::Clock(k, "Clock", n);
      add_child(clk_);
      p_ = new OnRisingEdgeProcess(k, clk_);
      add_child(p_);
    }
    int n() const { return n_; }
    void fini() override {
      // Check that process has been invoked N times.
      EXPECT_EQ(p_->n(), n());
    }
   private:
    cc::Clock* clk_;
    OnRisingEdgeProcess* p_;
    int n_;
  };

  for (int n : {10, 20, 30}) {
    cc::Kernel* k = new cc::Kernel;
    TopLevel* top = new TopLevel(k, n);
    top->init();
    k->run();
    top->fini();
    // Check that outstanding event queue has been exhausted.
    EXPECT_EQ(k->events_n(), 0);
    delete top;
    delete k;
  }
}

TEST(Sim, QueueDequeueImmediately) {

  // Enqueue process; perodically enqueues entries into the queue
  // ensuring that it is non-full (if so error out, as the consuemr
  // process should be removing the entries immediately after they are
  // enqueued)
  struct EnqueueProcess : cc::Process {
    EnqueueProcess(cc::Kernel* k, cc::Queue<cc::Time>* q, std::size_t n)
        : cc::Process(k, "EnqueueProcess"), q_(q), n_(n) {
    }
    std::size_t n() const { return n_; }
    void init() override {
      wait_for(cc::Time{10});
    }
    void eval() override {
      EXPECT_TRUE(q_->empty());
      EXPECT_TRUE(q_->enqueue(k()->time()));

      if (--n_ != 0) wait_for(cc::Time{10});
      log(Message{"Enqueued entry"});
    }
   private:
    cc::Queue<cc::Time>* q_;
    std::size_t n_;
  };

  // Dequeue process; dequeue entry from the queue whenever the
  // process received notification that an entry has been placed into
  // the queue. The dequeue should occur in the delta cycle
  // immediately preceeding the current dequeue cycle.
  struct DequeueProcess : cc::Process {
    DequeueProcess(cc::Kernel* k, cc::Queue<cc::Time>* q, std::size_t n)
        : cc::Process(k, "DequeueProcess"), q_(q), n_(n) {
    }
    std::size_t n() const { return n_; }
    void init() override {
      wait_on(q_->non_empty_event());
    }
    void eval() override {
      // Upon invokation, validate than queue has pending entries.
      EXPECT_FALSE(q_->empty());
      // Dequeue entry and validate
      cc::Time t;
      EXPECT_TRUE(q_->dequeue(t));
      const cc::Time current_time = k()->time();
      EXPECT_EQ(current_time.time, t.time);
      EXPECT_EQ(current_time.delta, t.delta + 1);
      // Validate that we haven't received more entries than have been
      // originally enqueued.
      EXPECT_NE(n_, 0);
      n_--;
      log(Message{"Dequeued entry"});
      wait_on(q_->enqueue_event());
    }
   private:
    cc::Queue<cc::Time>* q_;
    std::size_t n_;
  };

  // Top-level module for simulation.
  struct Top : public cc::Module {
    Top(cc::Kernel* k) : cc::Module(k, "top") {
      // Queue channel.
      q_ = new cc::Queue<cc::Time>(k, "Queue", 16);
      add_child(q_);
      // Enqueue Process
      ep_ = new EnqueueProcess(k, q_, 20);
      add_child(ep_);
      // Dequeu process
      dp_ = new DequeueProcess(k, q_, 20);
      add_child(dp_);
    }
    void validate() {
      EXPECT_TRUE(q_->empty());
      EXPECT_FALSE(q_->full());
    }
    cc::Queue<cc::Time>* q_;
    EnqueueProcess* ep_;
    DequeueProcess* dp_;
  };

  auto k = std::make_unique<cc::Kernel>();
  auto top = std::make_unique<Top>(k.get());
  top->init();
  k->run();
  top->validate();
}



int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}