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

#include "utility.h"
#include  <algorithm>

namespace cc {

KVListRenderer::KVListRenderer(std::ostream& os) : os_(os) {}

KVListRenderer::~KVListRenderer() {
  initialize();
  for (std::size_t i = 0; i < kvs_.size(); i++) {
    const kv_type& kv = kvs_[i];
    if (i != 0) os_ << ", ";
    render(kv);
  }
  finalize();
}

void KVListRenderer::initialize() {
  os_ << "'{";
}

void KVListRenderer::finalize() {
  os_ << "}";
}

void KVListRenderer::render(const kv_type& kv) {
  os_ << kv.first << ":" << kv.second;
}

void KVListRenderer::add_field(
    const std::string& key, const std::string& value){
  kvs_.push_back(std::make_pair(key, value));
}

std::string Hexer::to_hex(std::uint64_t x, std::size_t bits) const {
  return to_hex(reinterpret_cast<const char *>(std::addressof(x)), bits);
}

std::string Hexer::to_hex(const char * c, std::size_t bits) const {
  if (bits == 0) return "";

  // Predecrement character point to simply control logic in main
  // loop.
  --c;
  std::string result;
  while (bits > 0) {
    char x = *++c;
    // Lower nibble
    std::size_t bits_mask = std::min(4ul, bits);
    result.push_back(hex_char(x & mask<char>(bits_mask)));
    bits -= bits_mask;
    if (bits > 0) {
      // Upper nibble
      x >>= 4;
      bits_mask = std::min(4ul, bits);
      result.push_back(hex_char(x & mask<char>(bits_mask)));
      bits -= bits_mask;
    }    
  }

  if (truncate_) {
    // Remove leading zeros such that: 0x00000000 becomes 0x0.
    while ((result.size() > 1) && (result.back() == '0'))
      result.pop_back();
  }
  
  if (prefix_) {
    // Append hex prefix if applicable.
    result.push_back('x');
    result.push_back('0');
  }

  std::reverse(result.begin(), result.end());
  return result;
}

char Hexer::hex_char(char x) const {
  return (x < 10) ? ('0' + x) : ((upper_case_ ? 'A' : 'a') + (x - 10));
}

const char* to_string(bool b) {
  return b ? "true" : "false";
}

} // namespace cc
