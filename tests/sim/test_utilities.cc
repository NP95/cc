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
#include "utility.h"

namespace test {

void build_config(cc::SocConfig& cfg, std::size_t dir_n,
                  std::size_t cc_n, std::size_t cpu_n,
                  const std::vector<const char*>& trace) {
  cc::ProtocolBuilder* pb = cc::construct_protocol_builder("moesi");

  for (std::size_t cc = 0; cc < cc_n; cc++) {
    cc::CpuClusterConfig cpuc_cfg;
    cpuc_cfg.name += std::to_string(cc);

    cc::CCConfig cc_cfg;
    cc_cfg.pbuilder = pb;
    cpuc_cfg.cc_config = cc_cfg;

    cc::L2CacheAgentConfig l2c_config;
    l2c_config.pbuilder = pb;
    cpuc_cfg.l2c_config = l2c_config;

    for (std::size_t cpu = 0; cpu < cpu_n; cpu++) {
      cc::L1CacheModelConfig l1c_config;
      l1c_config.name += std::to_string(cpu);
      l1c_config.pbuilder = pb;
      cpuc_cfg.l1c_configs.push_back(l1c_config);

      cc::CpuConfig cpu_config;
      cpu_config.name += std::to_string(cpu);
      cpuc_cfg.cpu_configs.push_back(cpu_config);
    }

    cfg.ccls.push_back(cpuc_cfg);
  }

  for (std::size_t d = 0; d < dir_n; d++) {
    cc::DirModelConfig dcfg;
    dcfg.name += std::to_string(d);
    dcfg.pbuilder = pb;

    cfg.dcfgs.push_back(dcfg);
  }

  cc::StimulusConfig scfg;
  std::string s;
  for (const char* line : trace) {
    s += line;
    s += "\n";
  }
  scfg.is = new std::istringstream(s);
  std::vector<std::string> cpu_path;
  cpu_path.push_back("top");
  for (std::size_t cc = 0; cc < cc_n; cc++) {
    cpu_path.push_back("cluster" + std::to_string(cc));
    for (std::size_t cpu = 0; cpu < cpu_n; cpu++) {
      cpu_path.push_back("cpu" + std::to_string(cpu));
      scfg.cpaths.push_back(cc::join(cpu_path.begin(), cpu_path.end()));
      cpu_path.pop_back();
    }
    cpu_path.pop_back();
  }
  cfg.scfg = scfg;
}


} // namespace test
