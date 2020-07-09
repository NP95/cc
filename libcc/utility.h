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

#ifndef CC_INCLUDE_CC_COMMON_H
#define CC_INCLUDE_CC_COMMON_H

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <string>
#include <ostream>
#include <utility>
#include <vector>

namespace cc {

using addr_t = std::uint64_t;

template <typename T>
T log2ceil(T t) {
  int n = 0;
  while (t) {
    t >>= 1;
    n++;
  }
  return n;
}

template <typename T>
T mask(std::size_t bits) {
  if (bits == 0) return 0;

  T t = 1;
  return (t << bits) - 1;
}

template <typename T>
typename std::underlying_type<T>::type ut(T e) {
  return static_cast<typename std::underlying_type<T>::type>(e);
}

class KVListRenderer {
  using kv_type = std::pair<std::string, std::string>;
 public:
  KVListRenderer(std::ostream& os);
  ~KVListRenderer();

  //
  void add_field(const std::string& key, const std::string& value);
  
 private:
  void initialize();
  void finalize();
  void render(const kv_type& kv);

  // Key/Value pairs
  std::vector<kv_type> kvs_;
  // Output stream
  std::ostream& os_;
};

}  // namespace cc

#endif
