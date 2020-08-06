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

  const cc::SocConfig& cfg() const { return top_->cfg(); }

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

/*
TEST(Evict111, LoadEvictOneLine) {
  cc::kernel::Kernel k;
  cc::SocConfig cfg;

  const std::vector<const char*> trace = {
    "+200",
    "C:0,LD,0x00000",
    "+200",
    "C:0,LD,0x10000",
    "+200",
    "C:0,LD,0x20000",
    "+200",
    "C:0,LD,0x30000",
    "+200",
    "C:0,LD,0x40000"
  };
  // Build simple test configuration.
  test::build_config(cfg, 1, 1, 1, trace);
  SocModel soc(&k, cfg);
  soc.run();

  // Lookup L1 cache model instance where we expect to find the line.
  const cc::L1CacheAgent* l1cache =
      soc.get_object_as<cc::L1CacheAgent*>("top.cluster0.l1cache0");
  EXPECT_NE(l1cache, nullptr);


  // Presently assumes that this first address is evicted, which may
  // not be the case in other eviction policies.
  for (std::size_t addr : {0x10000, 0x20000, 0x30000, 0x40000}) {
    test::LineChecker checker(l1cache->cache(), addr);
    // Validate that cache has line installed.
    EXPECT_TRUE(checker.has_line());
    // Validate that cache line is in a readable state.
    EXPECT_TRUE(checker.line_is_readable());
  }
}
*/

TEST(Evict111, StoreEvictOneLine) {
  cc::kernel::Kernel k;
  cc::SocConfig cfg;

  const std::vector<const char*> trace = {
    "+200",
    "C:0,ST,0x00000",
    "+200",
    "C:0,ST,0x10000",
    "+200",
    "C:0,ST,0x20000",
    "+200",
    "C:0,ST,0x30000",
    "+200",
    "C:0,ST,0x40000"
  };
  // Build simple test configuration.
  test::build_config(cfg, 1, 1, 1, trace);
  SocModel soc(&k, cfg);
  soc.run();

  // Lookup L1 cache model instance where we expect to find the line.
  const cc::L1CacheAgent* l1cache =
      soc.get_object_as<cc::L1CacheAgent*>("top.cluster0.l1cache0");
  EXPECT_NE(l1cache, nullptr);


  // Presently assumes that this first address is evicted, which may
  // not be the case in other eviction policies.
  for (std::size_t addr : {0x10000, 0x20000, 0x30000, 0x40000}) {
    test::LineChecker checker(l1cache->cache(), addr);
    EXPECT_TRUE(checker.has_line());
    EXPECT_TRUE(checker.line_is_readable());
    EXPECT_TRUE(checker.line_is_writeable());
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
