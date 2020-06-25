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
  EXPECT_EQ(helper.line(0xFFFFFFF), 0x3FF);
}

TEST(Cache, Basic) {
  struct State {
    bool dirty = false;
  };

  cc::CacheModelConfig cfg;
  cfg.sets_n = 1024;
  cfg.ways_n = 4;
  cfg.line_bytes_n = 64;
  cc::CacheModel<State> cache(cfg);
  EXPECT_FALSE(cache.hit(0));

  const cc::CacheModel<State>::Set s{cache.set(0)};
  EXPECT_FALSE(s.requires_eviction(0));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

