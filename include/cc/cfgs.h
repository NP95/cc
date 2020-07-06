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

#ifndef CC_INCLUDE_CC_CFGS_H
#define CC_INCLUDE_CC_CFGS_H

#include <cstddef>
#include <string>
#include <vector>

namespace cc {

// Forwards
class Stimulus;
class L1CacheModelProtocol;
class L2CacheModelProtocol;

//
//
struct CacheModelConfig {
  // The number of sets
  std::uint16_t sets_n = 1024;

  // The number of ways per set (degree of cache associativity).
  std::uint8_t ways_n = 4;

  // The length of a line in bytes.
  std::uint8_t line_bytes_n = 64;

  // The total number of cache lines.
  std::size_t lines() const;

  // The total cache capacity in bytes.
  std::size_t bytes() const;
};

//
//
struct L1CacheModelConfig {
  // L1 Cache Model name
  std::string name = "l1cache";

  // Pointer to the transaction source instance for the current l1
  // cache instance (models the notion of a microprocessor
  // periodically emitting load/store instructions to memory).
  Stimulus* stim = nullptr;

  // Protocol to the cache protocol logic.
  L1CacheModelProtocol* protocol = nullptr;

  // Number of slots in the command queue.
  std::size_t l1_cmdq_slots_n = 3;

  // Number of credits/slots reserved to the L2 cache command queue.
  std::size_t l2_cmdq_credits_n = 3;

  // LD/ST pipe flush penalty (cycles)
  std::size_t ldst_flush_penalty_n = 3;

  // Cache configuration.
  CacheModelConfig cconfig;
};

//
//
struct L2CacheModelConfig {
  // L2 Cache Model name
  std::string name = "l2cache";

  // Pointer to the cache protocol logic
  L2CacheModelProtocol* protocol = nullptr;
  
  // Child L1 client configurations.
  std::vector<L1CacheModelConfig> l1configs;
};

} // namespace cc

#endif
