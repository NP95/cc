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

#include "cc/kernel.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

TEST(Kernel, BasicScheduling) {
  struct TickAction : public cc::kernel::Action {
    TickAction(cc::kernel::Kernel* k, cc::kernel::Time::time_type expected_time)
        : cc::kernel::Action(k, "TickAction"), expected_time_(expected_time) {}
    bool eval() override {
      const cc::kernel::Time time = k()->time();
      EXPECT_EQ(time.time, expected_time_);
      EXPECT_EQ(time.delta, 0);
      // Discard after evaluation.
      return true;
    }
    cc::kernel::Time::time_type expected_time_;
  };

  cc::kernel::Kernel k;
  k.add_action(cc::kernel::Time{0, 0}, new TickAction(&k, 0));
  k.add_action(cc::kernel::Time{10, 0}, new TickAction(&k, 10));
  k.add_action(cc::kernel::Time{20, 0}, new TickAction(&k, 20));
  k.add_action(cc::kernel::Time{30, 0}, new TickAction(&k, 30));
  k.add_action(cc::kernel::Time{40, 0}, new TickAction(&k, 40));
  k.add_action(cc::kernel::Time{50, 0}, new TickAction(&k, 50));
  k.run();
}

TEST(Kernel, RandomEventScheduling) {
  struct CheckTime : public cc::kernel::Action {
    CheckTime(cc::kernel::Kernel* k, cc::kernel::Time* time)
        : cc::kernel::Action(k, "CheckTimeAction"), time_(time) {}
    bool eval() override {
      EXPECT_TRUE(later_or_coincident(*time_, k()->time()));
      *time_ = k()->time();
      // Discard after evaluation.
      return true;
    }
    bool later_or_coincident(const cc::kernel::Time& lhs,
                             const cc::kernel::Time& rhs) {
      if (lhs.time == rhs.time && lhs.delta == rhs.delta) return true;
      return lhs < rhs;
    }
    cc::kernel::Time* time_ = nullptr;
  };

  cc::kernel::Kernel k;
  cc::kernel::RandomSource& r = k.random_source();
  std::vector<cc::kernel::Time> random_times;
  std::generate_n(std::back_inserter(random_times), 10000, [&r]() {
    return cc::kernel::Time{r.uniform<cc::kernel::Time::time_type>(),
                            r.uniform<cc::kernel::Time::delta_type>()};
  });

  cc::kernel::Time prior_time{0, 0};
  for (const cc::kernel::Time& time : random_times)
    k.add_action(time, new CheckTime(&k, &prior_time));
  k.run();
}

TEST(Kernel, FatalError) {
  struct TopModule : cc::kernel::Module {
    struct RaiseErrorProcess : cc::kernel::Process {
      RaiseErrorProcess(cc::kernel::Kernel* k)
          : cc::kernel::Process(k, "RaiseErrorProcess") {}
      void init() override { wait_until(cc::kernel::Time{100}); }
      void eval() override {
        log(Message("Some error condition", Level::Fatal));
      }
    };
    TopModule(cc::kernel::Kernel* k) : cc::kernel::Module(k, "top") {
      set_top();
      p_ = new RaiseErrorProcess(k);
      add_child_process(p_);
    }
    RaiseErrorProcess* p_;
  };
  auto k = std::make_unique<cc::kernel::Kernel>();
  auto top = std::make_unique<TopModule>(k.get());
  top->init();
  k->run();
  ASSERT_TRUE(k->fatal());
  const cc::kernel::Time time = k->time();
  ASSERT_EQ(time.time, 100);
}

TEST(Kernel, EventOr) {
  // Test to validate correctness of the EventOr notification mechanism.
  struct Top : cc::kernel::Module {

    // Producer Process; invoked on random intervals;
    struct ProducerProcess : cc::kernel::Process {
      ProducerProcess(cc::kernel::Kernel* k, const std::string& name, std::size_t n)
          : cc::kernel::Process(k, name), e_(k, "e"), n_(n) {
      }
      std::size_t n() const { return n_; }
      cc::kernel::Event& e() { return e_; }
     private:
      void init() override {
        if (n_ != 0) { wait_for(random_interval()); }
      }
      void eval() override {
        const Message msg("Raise notify.", Level::Debug);
        log(msg);
        e_.notify();
        if (--n_ != 0) { wait_for(random_interval()); }
      }
      cc::kernel::Time random_interval() {
        cc::kernel::RandomSource& r = k()->random_source();
        const cc::kernel::Time time{r.uniform<cc::kernel::Time::time_type>(10, 100)};
        return time;
      }
      cc::kernel::Event e_;
      std::size_t n_;
    };

    // Consumer Process; invoked by the child (producer) events.
    struct ConsumerProcess : cc::kernel::Process {
      ConsumerProcess(cc::kernel::Kernel* k, std::size_t n)
          : cc::kernel::Process(k, "ConsumerProcess"), eor_(k, "eor"), n_(n) {
      }
      ~ConsumerProcess() {
        EXPECT_EQ(n(), 0);
      }
      int n() const { return n_; }
      void add_child_event(cc::kernel::Event& e) { eor_.add_child_event(&e); }
      void finalize() { eor_.finalize(); }
     private:
      void init() override {
        eor_.init();
        wait_on(eor_);
      }
      void eval() override {
        // Check for extraneous notifications.
        EXPECT_NE(n_, 0);
        const Message msg{"Notified!", Level::Debug};
        log(msg);
        n_--;
        wait_on(eor_);
      }
      cc::kernel::EventOr eor_;
      int n_;
    };

    Top(cc::kernel::Kernel* k, std::size_t n, std::size_t event_n)
        : cc::kernel::Module(k, "top"), n_(n), event_n_(event_n) {
      set_top();
      cps_ = new ConsumerProcess(k, n * event_n);
      add_child_process(cps_);
      for (std::size_t i = 0; i < n; i++) {
        std::string process_name{"ProducerProcess"};
        process_name += std::to_string(i);
        ProducerProcess* pp = new ProducerProcess(k, process_name, event_n);
        cps_->add_child_event(pp->e());
        pps_.push_back(pp);
        add_child_process(pp);
      }
      cps_->finalize();
    }
    std::size_t n() const { return n_; }
    std::size_t event_n() const { return event_n_; }
   private:
    ConsumerProcess* cps_;
    std::vector<ProducerProcess*> pps_;
    std::size_t n_;
    std::size_t event_n_;
  };
  cc::kernel::Kernel k;
  Top top(&k, 3, 10);
  top.init();
  k.run();
};

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
