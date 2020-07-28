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

#ifndef CC_TESTS_SIM_TEST_UTILITIES_H
#define CC_TESTS_SIM_TEST_UTILITIES_H

#include "cc/cfgs.h"
#include "cc/soc.h"
#include "cache.h"
#include "protocol.h"
#include <vector>
#include <string>
#include <fstream>

namespace test {

//
cc::StimulusConfig build_stimulus(const std::vector<const char*>& trace);

//
void build_config(cc::SocConfig& cfg, std::size_t dir_n, std::size_t cc_n,
                  std::size_t cpu_n);

//
template<typename LINE>
class LineChecker {
 public:
  LineChecker(const cc::CacheModel<LINE>* cache, cc::addr_t addr)
      : cache_(cache), addr_(addr) {
  }

  bool has_line() const {
    auto ah = cache_->ah();
    auto set = cache_->set(ah.set(addr_));
    return (set.find(ah.tag(addr_)) != set.end());
  }

  bool line_is_readable() const {
      auto ah = cache_->ah();
      auto set = cache_->set(ah.set(addr_));
      auto it = set.find(ah.tag(addr_));
      if (it == set.end()) return false;
      const cc::L1LineState* line = it->t();
      return line->is_readable();
  }

 private:
  
  cc::addr_t addr_;
  const cc::CacheModel<LINE>* cache_ = nullptr;
};
    

} // namespace test


#endif
