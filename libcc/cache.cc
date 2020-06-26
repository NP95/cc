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

namespace cc {

CacheAddressHelper::CacheAddressHelper(const CacheModelConfig& config)
    : config_(config) {
  offset_bits_ = log2ceil(config.line_bytes_n - 1);
  line_bits_ = log2ceil(config.sets_n - 1);
}

addr_t CacheAddressHelper::offset(const addr_t& a) const {
  return a & mask<addr_t>(offset_bits_);
}

addr_t CacheAddressHelper::set(const addr_t& a) const {
  return (a >> offset_bits_) & mask<addr_t>(line_bits_);
}

addr_t CacheAddressHelper::tag(const addr_t& a) const {
  return (a >> (offset_bits_ + line_bits_));
}

std::size_t CacheModelConfig::lines() const { return ways_n * sets_n; }

std::size_t CacheModelConfig::bytes() const { return line_bytes_n * lines(); }

}  // namespace cc
