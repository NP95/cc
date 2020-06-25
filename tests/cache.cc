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
#include "kernel.h"
#include <set>

#include "gtest/gtest.h"

TEST(Cache, AddressHelper) {
  cc::CacheModelConfig cfg;
  cfg.sets_n = 1024;
  cfg.ways_n = 4;
  cfg.line_bytes_n = 64;
  cc::CacheAddressHelper helper(cfg);
  EXPECT_EQ(helper.offset_bits(), 6);
  EXPECT_EQ(helper.line_bits(), 10);
  EXPECT_EQ(helper.offset(0xFFFFF), 0x3F);
  EXPECT_EQ(helper.set(0xFFFFFFF), 0x3FF);
}

TEST(Cache, Basic) {
  struct State {
    std::uint64_t token;
  };

  cc::CacheModelConfig cfg;
  cfg.sets_n = 1024;
  cfg.ways_n = 4;
  cfg.line_bytes_n = 64;
  cc::CacheModel<State> cache(cfg);

  cc::kernel::Kernel k;
  cc::kernel::RandomSource& rnd = k.random_source();
  
  std::map<cc::addr_t, State> cache_predict;
  const cc::CacheAddressHelper& ah = cache.ah();

  // Install some random state into all lines belonging to a set.
  for (int way = 0; way < cfg.ways_n; way++) {
    const cc::addr_t a = way << (ah.line_bits() + ah.offset_bits());
    cc::CacheModel<State>::Set set = cache.set(ah.set(a));
    State state;
    state.token = rnd.uniform<std::uint64_t>();
    EXPECT_FALSE(set.requires_eviction(ah.tag(a)));
    bool did_install = false;
    for (cc::CacheModel<State>::LineIterator it = set.begin(); it != set.end(); ++it) {
      const cc::CacheModel<State>::Line& line = it.line();
      if (!line.valid) {
        did_install = true;
        EXPECT_TRUE(set.install(it, ah.tag(a), state));
        EXPECT_EQ(cache_predict.count(a), 0);
        cache_predict[a] = state;
        break;
      }
    }
    EXPECT_TRUE(did_install);
  }

  // Read back all previously installed entries and validate that
  // 1) they are indeed present in the cache, 2) the values contained
  // in by the line are what was expeted.
  for (const std::pair<cc::addr_t, State> & p: cache_predict) {
    const cc::addr_t a = p.first;
    const State state_expected = p.second;

    cc::CacheModel<State>::Set set = cache.set(ah.set(a));
    State state_actual;
    // Validate that the address is present in the cache.
    EXPECT_TRUE(set.hit(ah.tag(a), state_actual));
    // Validate that the state recovered from the cache model is equal
    // to that which was originally installed.
    EXPECT_EQ(state_expected.token, state_actual.token);
    // Find line in cache.
    cc::CacheModel<State>::LineIterator it = set.find(ah.tag(a));
    EXPECT_NE(it, set.end());
    // Validate state of line.
    EXPECT_TRUE(it.line().valid);
    // Evict line.
    set.evict(it);
    it = set.find(a);
    // Expect to no longer be in the set.
    EXPECT_EQ(it, set.end());
    // Expect to no longer be in the cache.
    EXPECT_FALSE(cache.hit(a));
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

