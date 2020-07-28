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

#include "test_utilities.h"

namespace test {

cc::StimulusConfig build_stimulus(const std::vector<const char*>& trace) {
  cc::StimulusConfig cfg;
  std::string s;
  for (const char* line : trace) {
    s += line;
    s += "\n";
  }
  cfg.is = new std::istringstream(s);
  cfg.cpaths = {
    "top.cluster.cpu"
  };
  return cfg;
}

void build_config(cc::SocConfig& cfg, std::size_t dir_n,
                  std::size_t cc_n, std::size_t cpu_n) {
  cc::ProtocolBuilder* pb = cc::construct_protocol_builder("moesi");

  for (std::size_t i = 0; i < cc_n; i++) {
    cc::CpuClusterConfig cpuc_cfg;

    cc::CCConfig cc_cfg;
    cc_cfg.pbuilder = pb;
    cpuc_cfg.cc_config = cc_cfg;

    cc::L2CacheModelConfig l2c_config;
    l2c_config.pbuilder = pb;
    cpuc_cfg.l2c_config = l2c_config;

    cc::L1CacheModelConfig l1c_config;
    l1c_config.pbuilder = pb;
    cpuc_cfg.l1c_configs.push_back(l1c_config);

    cc::CpuConfig cpu_config;
    cpuc_cfg.cpu_configs.push_back(cpu_config);

    cfg.ccls.push_back(cpuc_cfg);
  }

  for (std::size_t i = 0; i < dir_n; i++) {
    cc::DirModelConfig dcfg;
    dcfg.pbuilder = pb;

    cfg.dcfgs.push_back(dcfg);
  }
}


} // namespace test
