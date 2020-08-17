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

#include "test/builder.h"
#include "test/top.h"
#include "test/checker.h"
#include "cc/stimulus.h"
#include "src/l1cache.h"
#include "gtest/gtest.h"

TEST(Cfg112, Read1) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(2);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  test::TbTop top(cfg);

  // Address of interest
  const cc::addr_t addr = 0;

  // Stimulus: single load instruction to some address.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  stimulus->advance_cursor(200);
  stimulus->push_stimulus(0, cc::CpuOpcode::Load, addr);

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const cc::L1CacheAgent* l1c =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0));

  // Check line status.
  const test::L1Checker checker(l1c);
  EXPECT_TRUE(checker.is_hit(addr));
  EXPECT_TRUE(checker.is_readable(addr));

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 1);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

TEST(Cfg111, ReadN) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(2);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  test::TbTop top(cfg);

  const std::size_t N = 100;
  // Address of interest
  const cc::addr_t addr = 0;
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  // Issue some number of Load instructions to the same address at some
  // periodic interval.
  for (std::size_t i = 0; i < N; i++) {
    stimulus->advance_cursor(200 + (i * 10));
    stimulus->push_stimulus(0, cc::CpuOpcode::Load, addr);
  }

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const cc::L1CacheAgent* l1c =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0));

  // Check line status.
  const test::L1Checker checker(l1c);
  EXPECT_TRUE(checker.is_hit(addr));
  EXPECT_TRUE(checker.is_readable(addr));

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), N);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

TEST(Cfg112, Write1) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(2);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  test::TbTop top(cfg);

  // Address of interest
  const cc::addr_t addr = 0;
  // Stimulus: single load instruction to some address.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  stimulus->advance_cursor(200);
  stimulus->push_stimulus(0, cc::CpuOpcode::Store, addr);

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const cc::L1CacheAgent* l1c =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0));

  // Check line status.
  const test::L1Checker checker(l1c);
  EXPECT_TRUE(checker.is_hit(addr));
  EXPECT_TRUE(checker.is_readable(addr));

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 1);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

TEST(Cfg112, WriteN) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(2);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  test::TbTop top(cfg);

  // Address of interest
  const cc::addr_t addr = 0;
  // Stimulus: single load instruction to some address.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  const std::size_t N = 100;
  for (std::size_t i = 0; i < N; i++) {
    stimulus->advance_cursor(200 + (i * 10));
    stimulus->push_stimulus(0, cc::CpuOpcode::Store, addr);
  }

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const cc::L1CacheAgent* l1c =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0));

  // Check line status.
  const test::L1Checker checker(l1c);
  EXPECT_TRUE(checker.is_hit(addr));
  EXPECT_TRUE(checker.is_readable(addr));
 
 // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), N);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
