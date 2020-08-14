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
#include "cc/kernel.h"
#include "cc/stimulus.h"
#include "src/l1cache.h"
#include "gtest/gtest.h"
#include <array>

TEST(Coincident, Cfg121_SimpleRead) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(2);
  cb.set_cpu_n(1);

  cc::StimulusConfig stimulus_config;
  stimulus_config.type = cc::StimulusType::Programmatic;
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  cc::kernel::Kernel k;
  test::TbTop top(&k, cfg);

  // Stimulus: single load instruction to some address.
  cc::ProgrammaticStimulus* stimulus =
      static_cast<cc::ProgrammaticStimulus*>(top.stimulus());

  // Address of interest
  const cc::addr_t addr = 0;

  // CPU 0 issues Load to 0x0 @ 200; note that loads are coincident at
  // the same simulation time and corresponding to difference CPU in
  // different clusters.
  stimulus->advance_cursor(200);
  stimulus->push_stimulus(0, cc::CpuOpcode::Load, addr);
  stimulus->push_stimulus(1, cc::CpuOpcode::Load, addr);

  // Run to exhaustion
  k.run();

  const std::array<cc::L1CacheAgent*, 2> l1cs = {
    // L1 Cache ID 0
    top.lookup_by_path<cc::L1CacheAgent>(test::l1c_by_cpu_id(cfg, 0)),
    // L1 Cache ID 1
    top.lookup_by_path<cc::L1CacheAgent>(test::l1c_by_cpu_id(cfg, 1))
  };

  // Validate final line state.
  for (cc::L1CacheAgent* l1c : l1cs) {
    const test::L1Checker checker(l1c);
    EXPECT_TRUE(checker.is_hit(addr));
    EXPECT_TRUE(checker.is_readable(addr));
  }

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 2);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
