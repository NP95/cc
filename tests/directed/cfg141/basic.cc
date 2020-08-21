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

/*
// Load4Unique
// ===========
//
// Description
// -----------
//
// Issue a Load instruct to some unique and non-conflicting address to
// each CPU in sequence.
//
// Expected Behavior
// -----------------
//
// Each CPU issues a load to its L1. The line misses in the cache and
// is fetched from the LLC and eventually installed in the Exclusive
// state.
TEST(Cfg141, Load4Unique) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(4);
  cb.set_cpu_n(1);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  test::TbTop top(cfg);

  std::size_t N = 4;
  std::vector<cc::addr_t> addrs;
  
  // Issue a load to each CPU.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  for (std::size_t i = 0; i < N; i++) {
    stimulus->advance_cursor(200);
    addrs.push_back(1000 * i);
    stimulus->push_stimulus(i, cc::CpuOpcode::Load, addrs.back());
  }

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const std::array<const cc::L1CacheAgent*, 4> l1cs = {
    // L1 Cache ID 0
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0)),
    // L1 Cache ID 1
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 1)),
    // L1 Cache ID 2
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 2)),
    // L1 Cache ID 3
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 3))
  };

  for (std::size_t i = 0; i < N; i++) {
    const test::L1Checker checker(l1cs[i]);
    const cc::addr_t addr = addrs[i];
    // Expect line to be present in cache.
    EXPECT_TRUE(checker.is_hit(addr));
    // Expect line to be readable
    EXPECT_TRUE(checker.is_readable(addr));
    // Expect not to be writeable (shared between caches)/
    EXPECT_FALSE(checker.is_writeable(addr));
  }

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), N);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}
*/

// Load4Same
// ===========
//
// Description
// -----------
//
// Each CPU issues a Load to the same address at the same instant.
//
// Expected Behavior
// -----------------
//
// Each CPU issues a Load instruction and the line returns from LLC.
// For the first CPU (the winner of the simultaneous requests),
// initially receives the line in a Unique state. Upon completion of
// the subsequent commands, the line is installed in the Shared state.
//
TEST(Cfg141, Load4Same) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(4);
  cb.set_cpu_n(1);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  test::TbTop top(cfg);

  // Address of interest
  const cc::addr_t addr = 0;

  // Issue a load to each CPU.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  stimulus->advance_cursor(200);
  stimulus->push_stimulus(0, cc::CpuOpcode::Load, addr);
  stimulus->push_stimulus(1, cc::CpuOpcode::Load, addr);
  stimulus->push_stimulus(2, cc::CpuOpcode::Load, addr);
  stimulus->push_stimulus(3, cc::CpuOpcode::Load, addr);


  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const std::array<const cc::L1CacheAgent*, 4> l1cs = {
    // L1 Cache ID 0
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0)),
    // L1 Cache ID 1
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 1)),
    // L1 Cache ID 2
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 2)),
    // L1 Cache ID 3
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 3))
  };

  for (const cc::L1CacheAgent* l1c : l1cs) {
    const test::L1Checker checker(l1c);
    // Expect line to be present in cache.
    EXPECT_TRUE(checker.is_hit(addr));
    // Expect line to be readable
    EXPECT_TRUE(checker.is_readable(addr));
    // Expect not to be writeable (shared between caches)/
    EXPECT_FALSE(checker.is_writeable(addr));
  }

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 4);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
