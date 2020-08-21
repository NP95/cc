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
#include "src/dir.h"
#include "src/l1cache.h"
#include "gtest/gtest.h"

TEST(Cfg141, RecallUnmodified) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(4);
  cb.set_cpu_n(1);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();
  test::TbTop top(cfg);

  const cc::DirAgent* dir = top.lookup_by_path<cc::DirAgent>("top.dir0");
  ASSERT_TRUE(dir != nullptr);

  // Look up L1 cache instances.
  std::vector<const cc::L1CacheAgent*> l1cs;
  for (std::size_t i = 0; i< cfg.ccls.size(); i++) {
    const std::string path = test::path_l1c_by_cpu_id(cfg, i);
    l1cs.push_back(top.lookup_by_path<cc::L1CacheAgent>(path));
  }

  // Lookup the numbe of ways in the directory cache structure (N). We
  // require N + 1 loads to lines belonging to the same line for a
  // recall to occur.
  const cc::CacheAddressHelper dir_ah = dir->cache()->ah();

  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());

  std::size_t transactions_n = 0;
  cc::addr_t addr = 0;
  std::vector<cc::addr_t> addrs;
  for (std::size_t i = 0; i < dir_ah.ways_n(); i++) {
    stimulus->advance_cursor(200);
    stimulus->push_stimulus(i, cc::CpuOpcode::Load, addr);
    addrs.push_back(addr);
    addr += (dir_ah.line_span() * dir_ah.sets_n());
    ++transactions_n;
  }

  // Initialize simulation.
  top.initialize();

  // Run to exhaustion
  top.run();

  // Check status of all lines.
  for (std::size_t cpu_id = 0; cpu_id < addrs.size(); cpu_id++) {
    const test::L1Checker checker(l1cs[cpu_id]);
    EXPECT_TRUE(checker.is_hit(addrs[cpu_id]));
  }

  // The following transaction should cause recall of one of the prior lines.
  stimulus->advance_cursor(200);
  stimulus->push_stimulus(0, cc::CpuOpcode::Load, addr);
  ++transactions_n;

  top.run();

  // Validate that line has been installed in the requester cache.
  const test::L1Checker checker(l1cs[0]);
  EXPECT_TRUE(checker.is_hit(addr));

  // Validate that one address has now been evicted.
  std::size_t seen_n = 0;
  for (std::size_t cpu_id = 0; cpu_id < cb.cc_n(); cpu_id++) {
    const test::L1Checker checker(l1cs[cpu_id]);
    if (checker.is_hit(addrs[cpu_id])) { ++seen_n; }
  }
  EXPECT_EQ(seen_n, cb.cc_n() - 1);

  // Finalize, terminate simulation.
  top.finalize();

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), transactions_n);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}


TEST(Cfg141, RecallModified) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(4);
  cb.set_cpu_n(1);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();
  test::TbTop top(cfg);

  const cc::DirAgent* dir = top.lookup_by_path<cc::DirAgent>("top.dir0");
  ASSERT_TRUE(dir != nullptr);

  // Look up L1 cache instances.
  std::vector<const cc::L1CacheAgent*> l1cs;
  for (std::size_t i = 0; i< cfg.ccls.size(); i++) {
    const std::string path = test::path_l1c_by_cpu_id(cfg, i);
    l1cs.push_back(top.lookup_by_path<cc::L1CacheAgent>(path));
  }

  // Lookup the numbe of ways in the directory cache structure (N). We
  // require N + 1 loads to lines belonging to the same line for a
  // recall to occur.
  const cc::CacheAddressHelper dir_ah = dir->cache()->ah();

  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());

  std::size_t transactions_n = 0;
  cc::addr_t addr = 0;
  std::vector<cc::addr_t> addrs;
  for (std::size_t i = 0; i < dir_ah.ways_n(); i++) {
    stimulus->advance_cursor(200);
    stimulus->push_stimulus(i, cc::CpuOpcode::Store, addr);
    addrs.push_back(addr);
    addr += (dir_ah.line_span() * dir_ah.sets_n());
    ++transactions_n;
  }

  // Initialize simulation.
  top.initialize();

  // Run to exhaustion
  top.run();

  // Check status of all lines.
  for (std::size_t cpu_id = 0; cpu_id < addrs.size(); cpu_id++) {
    const test::L1Checker checker(l1cs[cpu_id]);
    EXPECT_TRUE(checker.is_hit(addrs[cpu_id]));
    EXPECT_TRUE(checker.is_writeable(addrs[cpu_id]));
  }

  // The following transaction should cause recall of one of the prior lines.
  stimulus->advance_cursor(200);
  stimulus->push_stimulus(0, cc::CpuOpcode::Store, addr);
  ++transactions_n;

  top.run();

  // Validate that line has been installed in the requester cache.
  const test::L1Checker checker(l1cs[0]);
  EXPECT_TRUE(checker.is_hit(addr));

  // Validate that one address has now been evicted.
  std::size_t seen_n = 0;
  for (std::size_t cpu_id = 0; cpu_id < cb.cc_n(); cpu_id++) {
    const test::L1Checker checker(l1cs[cpu_id]);
    if (checker.is_hit(addrs[cpu_id])) { ++seen_n; }
  }
  EXPECT_EQ(seen_n, cb.cc_n() - 1);

  // Finalize, terminate simulation.
  top.finalize();

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), transactions_n);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
