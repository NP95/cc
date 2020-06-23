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
#include "kernel.h"

TEST(Kernel, BasicScheduling) {
  cc::Kernel k;

  struct TickAction : public cc::Action {
    TickAction(cc::Time::cycle_type expected_cycle)
        : expected_cycle_(expected_cycle) {}
    void eval(cc::Kernel* k) override {
      const cc::Time time = k->time();
      EXPECT_EQ(time.cycle, expected_cycle_);
      EXPECT_EQ(time.delta, 0);
    }
    cc::Time::cycle_type expected_cycle_;
  };

  k.add_action(cc::Time{ 0, 0}, new TickAction( 0));
  k.add_action(cc::Time{10, 0}, new TickAction(10));
  k.add_action(cc::Time{20, 0}, new TickAction(20));
  k.add_action(cc::Time{30, 0}, new TickAction(30));
  k.add_action(cc::Time{40, 0}, new TickAction(40));
  k.add_action(cc::Time{50, 0}, new TickAction(50));
  k.run();
}

TEST(Kernel, BasicClock) {
  cc::Kernel* k = new cc::Kernel;

  struct OnRisingEdgeProcess : public cc::Process {
    OnRisingEdgeProcess(cc::Kernel* k, cc::Clock* clk)
        : cc::Process(k), clk_(clk) {}
    void init() override {
      wait_on(clk_->rising_edge_event());
    }
    void eval() override {
      std::cout << "On rising edge: " << i++ << "\n";
      wait_on(clk_->rising_edge_event());
    }
    int i = 0;
    cc::Clock* clk_;
  };

  struct TopLevel : public cc::Module {
    TopLevel(cc::Kernel* k) : cc::Module(k) {
      clk_ = new cc::Clock(k, 100);
      add_child(clk_);
      p_ = new OnRisingEdgeProcess(k, clk_);
      add_process(p_);
      //      p_->init();
    }
    cc::Clock* clk_;
    cc::Process* p_;
  };

  TopLevel* top = new TopLevel(k);
  top->init();
  k->run(cc::RunMode::ForTime, cc::Time{100, 0});
  top->fini();
  delete top;
  delete k;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
