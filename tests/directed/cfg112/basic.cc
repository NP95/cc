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
#include <array>

// Load1
// =====
//
// Description
// -----------
//
// CPU0 issues a Load instruction to some address.
//
// Expected Behavior
// -----------------
//
// Directory installs line in either the Shared or the Exclusive
// State.
//
TEST(Cfg112, Load1) {
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
  stimulus->push_stimulus(1, cc::CpuOpcode::Load, addr);

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const cc::L1CacheAgent* l1c =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 1));

  // Check line status.
  const test::L1Checker checker(l1c);
  EXPECT_TRUE(checker.is_hit(addr));
  EXPECT_TRUE(checker.is_readable(addr));

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 1);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}


// LoadN
// =====
//
// Description
// -----------
//
// CPU0 issues some number of Load instructions to the same address.
//
// Expected Behavior
// -----------------
//
// On the initial compulsary miss, the directory installs line in
// either the Shared or the Exclusive State. Subsequent Load
// instructions should commit immediate as ths line is now installed
// in the cache.
//
TEST(Cfg112, LoadN) {
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
    stimulus->push_stimulus(1, cc::CpuOpcode::Load, addr);
  }

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const cc::L1CacheAgent* l1c =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 1));

  // Check line status.
  const test::L1Checker checker(l1c);
  EXPECT_TRUE(checker.is_hit(addr));
  EXPECT_TRUE(checker.is_readable(addr));

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), N);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}


// Store1
// ======
//
// Description
// -----------
//
// CPU0 issues a Store instruction to some address.
//
// Expected Behavior
// -----------------
//
// Directory installs the line the Exclusive state. Upon commit of the
// initial store instruction, the line is immediately promoted to the
// Modified state.
//
TEST(Cfg112, Store1) {
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


// StoreN
// ======
//
// Description
// -----------
//
// CPU0 issues some number of Store instructions to the same address.
//
// Expected Behavior
// -----------------
//
// Directory installs the line the Exclusive state. Upon commit of the
// initial store instruction, the line is immediately promoted to the
// Modified state. All subsequent commands commit immediately as the
// line is already present in the cache and is already in a writeable
// state.
//
TEST(Cfg112, StoreN) {
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


// AlternatingLoadsN
// =================
//
// Description
// -----------
//
// CPU0 and CPU1 alternate reads to the same line in L2.
//
// Expected Behavior
// -----------------
//
// Upon a Load from CPU A, the line is installed in either the Shared
// or Exclusive state. The line remains in the same state after the
// instruction has committed to the machine state. Upon a Load from
// CPU B, the line is installed in the Shared state or the Exclusive
// state (nominally Shared). In the Exclusive state, the line cannot
// be in the CPU B. Nominally, the line in CPU A retained by the cache
// is demoted to the Shared state.
//
TEST(Cfg112, AlternatingLoadsN) {
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

  // Stimulus: single load instruction to some address.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  bool store0 = false;
  for (std::size_t i = 0; i < N; i++) {
    stimulus->advance_cursor(200 + (i * 10));
    store0 = !store0;
    const std::uint64_t cpu_id = store0 ? 0 : 1;
    stimulus->push_stimulus(cpu_id, cc::CpuOpcode::Load, addr);
  }

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const std::array<const cc::L1CacheAgent*, 2> l1cs = {
    // L1 Cache ID 0
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0)),
    // L1 Cache ID 1
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 1))
  };

  // Validate
  for (const cc::L1CacheAgent* l1c : l1cs) {
    const test::L1Checker checker(l1c);

    // Line should be present in cache.
    EXPECT_TRUE(checker.is_hit(addr));
    // Line should be readable in cache.
    EXPECT_TRUE(checker.is_readable(addr));
    if (N > 1) {
      // Cache line *may* have been installed in the Exclusive state
      // on the first Load however if subsequent Loads have taken
      // place on other CPU, the line must have been demoted to the
      // Shared state.
      EXPECT_FALSE(checker.is_writeable(addr));
    }
  }

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), N);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
  
}

// AlternatingStoresN
// ==================
//
// Description
// -----------
//
// CPU0 and CPU1 alterate writes to the same line in L2. 
//
// Expected Behavior
// -----------------
//
// Upon a write from CPU A, the appropriate line is installed in the
// Exclusive state and promoted to the Modified state upon commit of
// the write instruction. Upon a write from CPU B, the line in CPU A
// should be invalidated at the line sourced from L2 in the Exclusive
// state. The line is then promoted to the Modified state upon commit
// of the instruction. Process repeats with the roles of CPU A and CPU
// B being inverted.
//
TEST(Cfg112, AlternatingStoresN) {
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

  // Stimulus: single load instruction to some address.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  bool store0 = false;
  for (std::size_t i = 0; i < N; i++) {
    stimulus->advance_cursor(200 + (i * 10));
    store0 = !store0;
    const std::uint64_t cpu_id = store0 ? 0 : 1;
    stimulus->push_stimulus(cpu_id, cc::CpuOpcode::Store, addr);
  }

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const std::array<const cc::L1CacheAgent*, 2> l1cs = {
    // L1 Cache ID 0
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0)),
    // L1 Cache ID 1
    top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 1))
  };

  // Validate

  // Last written cache instance.
  const cc::L1CacheAgent* l1c_wr = store0 ? l1cs[0] : l1cs[1];
  // Other cache instance (expected to be invalid).
  const cc::L1CacheAgent* l1c_inv = store0 ? l1cs[1] : l1cs[0];

  const test::L1Checker checker_wr(l1c_wr), checker_inv(l1c_inv);

  // Validate that line is writeable in last written cache.
  EXPECT_TRUE(checker_wr.is_hit(addr));
  EXPECT_TRUE(checker_wr.is_readable(addr));
  EXPECT_TRUE(checker_wr.is_writeable(addr));

  // Validate that line is not present in other cache.
  EXPECT_FALSE(checker_inv.is_hit(addr));
  EXPECT_FALSE(checker_inv.is_writeable(addr));

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), N);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
  
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
