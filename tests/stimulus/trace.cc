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
#include "cc/kernel.h"
#include "cc/soc.h"
#include "cc/stimulus.h"
#include "gtest/gtest.h"

TEST(Trace, Cfg111_SimpleRead) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(1);
  cb.set_cpu_n(1);

  // Define stimulus:
  const char* trace =
      // Map CPU ID to location in object hierarchy.
      "M:0,top.cluster0.cpu0\n"
      // Advance 200 time-units
      "+200\n"
      // CPU 0 issues Load instruction to address 0x0.
      "C:0,LD,0\n"
      ;

  cc::StimulusConfig stimulus_config = cc::TraceStimulus::from_string(trace);
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  cc::kernel::Kernel k;
  cc::SocTop top(&k, cfg);

  // Run to exhaustion
  k.run();

  // Validation.

  cc::Stimulus* stimulus = top.stimulus();

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 1);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

TEST(Trace, Cfg121_SimpleRead) {
  test::ConfigBuilder cb;
  cb.set_dir_n(1);
  cb.set_cc_n(2);
  cb.set_cpu_n(1);

  // Define stimulus:
  const char* trace =
      // Map CPU ID to location in object hierarchy.
      "M:0,top.cluster0.cpu0\n"
      "M:1,top.cluster1.cpu0\n"
      // Advance 200 time-units
      "+200\n"
      // CPU 0 issues Load instruction to address 0x0.
      "C:0,LD,0\n"
      // Advance 200 time-units
      "+200\n"
      // CPU 1 issues Load instruction to address 0x0.
      "C:1,LD,0\n"
      ;

  cc::StimulusConfig stimulus_config = cc::TraceStimulus::from_string(trace);
  cb.set_stimulus(stimulus_config);

  const cc::SocConfig cfg = cb.construct();

  cc::kernel::Kernel k;
  cc::SocTop top(&k, cfg);

  // Run to exhaustion
  k.run();

  // Validation.

  cc::Stimulus* stimulus = top.stimulus();

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 2);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
