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
#include "src/protocol.h"
#include "src/l1cache.h"
#include "gtest/gtest.h"


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
/*
TEST(Cfg111, Load1) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

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
TEST(Cfg111, LoadN) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

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
*/

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
TEST(Cfg111, Store1) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

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

/*
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
TEST(Cfg111, StoreN) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

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


// StoreAddrN
// ==========
//
// Description
// -----------
//
// CPU0 issues some number of Store commands to distinct
// lines. Address stimulus is constructed such that Store commands do
// not require evict of prior modified state.
//
// Expected behavior
// -----------------
//
// Lines are installed in the Exclusive state and immediately promoted
// to the Modified state upon commit of the Store
// instruction. Addresses do not conflcit and also do not cause
// capacity evictions therefore protocol activity is limited to the
// initial line installation only.
//
TEST(Cfg111, StoreAddrN) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  test::TbTop top(cfg);

  const cc::L1CacheAgent* l1agent =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0));
  cc::CacheModel<cc::L1LineState*>* l1cache = l1agent->cache();
  const cc::CacheAddressHelper& ah = l1cache->ah();
  
  // Stimulus: single load instruction to some address.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  std::vector<cc::addr_t> va;
  // Address of interest
  cc::addr_t addr = 0;
  // Fill entire cache; all sets, all ways. All should be present at EOS.
  // TODO: takes too long to complete; need to debug.
  //const std::size_t N = ah.sets_n() * ah.ways_n();
  const std::size_t N = 100;
  for (std::size_t i = 0; i < N; i++) {
    // Commands arrive at some periodic interval.
    stimulus->advance_cursor(200 + (i * 10));
    stimulus->push_stimulus(0, cc::CpuOpcode::Load, addr);
    va.push_back(addr);
    addr += ah.line_span();
  }

  // Run to exhaustion
  top.run_all();

  // Lookup CPU instance.
  const cc::L1CacheAgent* l1c =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0));

  // Check line status.
  const test::L1Checker checker(l1c);
  for (cc::addr_t addr : va) {
    EXPECT_TRUE(checker.is_hit(addr));
    EXPECT_TRUE(checker.is_readable(addr));
  }

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), va.size());

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}


// StoreThenLoad
// =============
//
// Description
// -----------
//
// CPU0 issues a Store instruction to some address and then issues a
// subsequent Load to the same line.
//
// Expected behavior
// -----------------
//
// CPU0 installs line in Exclusive state which is then immediately
// promoted to the Modified state upon commit of the Store
// instruction. The subsequent Load instruction then completes without
// further coherence traffic as the line is already in a readable
// state.
//
TEST(Cfg111, StoreThenLoad) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

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
  stimulus->advance_cursor(10);
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
  EXPECT_TRUE(checker.is_writeable(addr));

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 2);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}


// LoadThenStore
// =============
//
// Description
// -----------
//
// CPU0 issues a Load instruction to some address and then issues a
// subsequent Store to the same line.
//
// Expected behavior
// -----------------
//
// On the initial Load instruction, CPU0 installs line in either the
// Shared or Exclusive state. On the subsequent Store instruction,
// CPU0 requests promotion of the line to the Exclusive state (if line
// was initially installed in the Shared state), otherwise (if line
// was initially installed in the Exclusive state), line is
// immediately promoted to the Modified state.
//
TEST(Cfg111, LoadThenStore) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

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
  stimulus->advance_cursor(10);
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
  EXPECT_TRUE(checker.is_writeable(addr));

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 2);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}


// EvictLineUnmodifiedLine
// =======================
//
// Description
// -----------
//
// Issue N Load instructions such that each address belongs to the
// same cache set (where N is the number of ways in the cache). Issue
// a further Load instruction to the same set such that a prior line
// is evicted.
//
// Expected Behavior
// -----------------
//
// The final value of the set will be equal to all lines loaded by
// stimulus, minus some line which has been evicted. The line which
// has been evicted is indeterminate and some function of the current
// eviction policy which is not validated by this test.
//
TEST(Cfg111, EvictLineUnmodifiedLine) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  test::TbTop top(cfg);

  const cc::L1CacheAgent* l1agent =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0));
  cc::CacheModel<cc::L1LineState*>* l1cache = l1agent->cache();
  const cc::CacheAddressHelper& ah = l1cache->ah();
  
  // Stimulus: single load instruction to some address.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());
  std::vector<cc::addr_t> va;
  // Address of interest
  cc::addr_t addr = 0, addr_stride = (ah.line_span() * ah.sets_n());
  // Fill all ways in the Set.
  const std::size_t N = ah.ways_n();
  for (std::size_t i = 0; i < N; i++) {
    // Commands arrive at some periodic interval.
    stimulus->advance_cursor(200 + (i * 10));
    stimulus->push_stimulus(0, cc::CpuOpcode::Load, addr);
    va.push_back(addr);
    addr += addr_stride;
  }

  // Run to exhaustion
  top.initialize();
  top.run();

  // Lookup CPU instance.
  const cc::L1CacheAgent* l1c =
      top.lookup_by_path<cc::L1CacheAgent>(test::path_l1c_by_cpu_id(cfg, 0));
  const test::L1Checker checker(l1c);

  // WAYS_N Loads have been performed by CPU0; we expect all 
  
  // Check line status.
  for (cc::addr_t addr : va) {
    EXPECT_TRUE(checker.is_hit(addr));
    EXPECT_TRUE(checker.is_readable(addr));
  }

  // Install conflicting line which causes the eviction of a line which
  // is already present in the cache.
  stimulus->advance_cursor(top.time() + 20);
  stimulus->push_stimulus(0, cc::CpuOpcode::Load, addr);
  va.push_back(addr);

  // Advance simulation state.
  top.run();
  top.finalize();

  // One address should now have been evicted.
  std::size_t count_hit = 0;
  for (cc::addr_t addr : va) {
    if (checker.is_hit(addr)) { count_hit++; }
  }
  EXPECT_EQ(count_hit, ah.ways_n());

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), va.size());

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}
*/
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
