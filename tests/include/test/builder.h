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

#ifndef CC_TESTS_INCLUDE_TEST_BUILDER_H
#define CC_TESTS_INCLUDE_TEST_BUILDER_H

#include "cc/cfgs.h"

namespace test {

class ConfigBuilder {
 public:
  ConfigBuilder() = default;

  // Set CPU count (replicating per Cluster).
  void set_cpu_n(std::size_t n) { cpu_n_ = n; }

  // Set Directory count
  void set_dir_n(std::size_t n) { dir_n_ = n; }

  // Set Cpu Cluster count
  void set_cc_n(std::size_t n) { cc_n_ = n; }

  // Set stimulus configuration.
  void set_stimulus(const cc::StimulusConfig& s) { stimulus_config_ = s; }

  // Construct SOC configuration.
  cc::SocConfig construct() const;

 private:
  // CPU count
  std::size_t cpu_n_ = 1;

  // Directory count
  std::size_t dir_n_ = 1;

  // CPU cluster count.
  std::size_t cc_n_ = 1;

  // Stimulus Configuration
  cc::StimulusConfig stimulus_config_;
};


} // namespace test

#endif
