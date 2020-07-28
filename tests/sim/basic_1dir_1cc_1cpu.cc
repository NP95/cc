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

#include "cache.h"
#include "l1cache.h"
#include "cc/cfgs.h"
#include "cc/soc.h"
#include "cc/stimulus.h"
#include "test_utilities.h"
#include "gtest/gtest.h"

class SocModel {
 public:
  SocModel(cc::kernel::Kernel* k, const cc::SocConfig& cfg)
      : k_(k) {
    top_ = new cc::SocTop(k_, cfg);
  }

  ~SocModel() {
    delete top_;
  }

  void set_stimulus(cc::Stimulus* stimulus) { stimulus_ = stimulus; }

  template<typename T>
  T get_object_as(const std::string& name) {
    return static_cast<T>(top_->find_path(name));
  }
  
  void run() {
    k_->invoke_elab();
    k_->invoke_drc();
    k_->invoke_init();
    const cc::kernel::RunMode r = cc::kernel::RunMode::ToExhaustion;
    const cc::kernel::Time t;
    k_->invoke_run(r, t);
  }

 private:
  cc::kernel::Kernel* k_ = nullptr;
  cc::SocTop* top_ = nullptr;
  cc::Stimulus* stimulus_ = nullptr;
};

TEST(Basic, SimpleRead) {
  cc::kernel::Kernel k;
  cc::SocConfig cfg;

  // Build simple test configuration.
  test::build_config(cfg, 1, 1, 1);

  // Test program:
  //
  //  1) Advance 200 time-units.
  //
  //  2) Issue a load instruction from CPU 0 to address 0x0.
  //
  const std::vector<const char*> trace{
    "+200",
    "C:0,LD,0"
  };
  cfg.scfg = test::build_stimulus(trace);
  SocModel soc(&k, cfg);
  soc.run();

  // Lookup L1 cache model instance where we expect to find the line.
  const cc::L1CacheModel* l1cache =
      soc.get_object_as<cc::L1CacheModel*>("top.cluster.l1cache");

  test::LineChecker checker(l1cache->cache(), 0);

  // Validate that cache has line installed.
  EXPECT_TRUE(checker.has_line());

  // Validate that cache line is in a readable state.
  EXPECT_TRUE(checker.line_is_readable());

  // Line may or may not be in a writeable state at this point
  // (ideally the line should be in a writeable state as it should be
  // exclusive at thsi point, but this is not something that we
  // specifically enforce).
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
