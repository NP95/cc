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

#ifndef CC_TESTS_INCLUDE_TEST_TOP_H
#define CC_TESTS_INCLUDE_TEST_TOP_H

#include "cc/soc.h"
#include "cc/kernel.h"

namespace test {


class TbTop {
 public:
  TbTop(cc::kernel::Kernel* k, const cc::SocConfig& config);
  ~TbTop();

  // Accessors:

  // Kernel instance
  cc::kernel::Kernel* k() const { return k_; }

  // Stimulus instance.
  cc::Stimulus* stimulus() const { return top_->stimulus(); }


  // Lookup object of type 'T' and return instance pointer.
  template<typename T>
  T* lookup_by_path(const std::string& path) const {
    return static_cast<T*>(top_->find_path(path));
  }

 private:
  // SOC top instance.
  cc::SocTop* top_ = nullptr;
  // Kernel instance
  cc::kernel::Kernel* k_ = nullptr;
};

} // namespace test

#endif
