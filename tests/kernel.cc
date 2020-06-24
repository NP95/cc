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
#include <memory>

TEST(Kernel, BasicScheduling) {
  cc::Kernel k;

  struct TickAction : public cc::Action {
    TickAction(cc::Time::time_type expected_time)
        : expected_time_(expected_time) {}
    void eval(cc::Kernel* k) override {
      const cc::Time time = k->time();
      EXPECT_EQ(time.time, expected_time_);
      EXPECT_EQ(time.delta, 0);
    }
    cc::Time::time_type expected_time_;
  };

  k.add_action(cc::Time{ 0, 0}, new TickAction( 0));
  k.add_action(cc::Time{10, 0}, new TickAction(10));
  k.add_action(cc::Time{20, 0}, new TickAction(20));
  k.add_action(cc::Time{30, 0}, new TickAction(30));
  k.add_action(cc::Time{40, 0}, new TickAction(40));
  k.add_action(cc::Time{50, 0}, new TickAction(50));
  k.run();
}

TEST(Kernel, FatalError) {
  struct TopModule : cc::Module {
    struct RaiseErrorProcess : cc::Process {
      RaiseErrorProcess(cc::Kernel* k) : cc::Process(k, "RaiseErrorProcess") {}
      void init() override {
        wait_until(cc::Time{100});
      }
      void eval() override {
        log(Message("Some error condition", Level::Fatal));
      }
    };
    TopModule(cc::Kernel* k) : cc::Module(k, "top") {
      set_top();
      p_ = new RaiseErrorProcess(k);
      add_child(p_);
    }
    RaiseErrorProcess* p_;
  };
  auto k = std::make_unique<cc::Kernel>();
  auto top = std::make_unique<TopModule>(k.get());
  top->init();
  k->run();
  ASSERT_TRUE(k->fatal());
  const cc::Time time = k->time();
  ASSERT_EQ(time.time, 100);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
