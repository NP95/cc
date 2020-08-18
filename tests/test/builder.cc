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

#include "test/builder.h"
#include "test/utility.h"
#include "cc/soc.h"
#include "cc/types.h"

namespace test {

cc::SocConfig ConfigBuilder::construct() const {
  cc::SocConfig cfg;
  cc::ProtocolBuilder* pb = cc::construct_protocol_builder("moesi");

  // Define CPU cluster configuration
  for (std::size_t cc = 0; cc < cc_n_; cc++) {
    cc::CpuClusterConfig cpuc_cfg;
    cpuc_cfg.name += std::to_string(cc);

    cc::CCConfig cc_cfg;
    cc_cfg.pbuilder = pb;
    cpuc_cfg.cc_config = cc_cfg;

    cc::L2CacheAgentConfig l2c_config;
    l2c_config.pbuilder = pb;
    cpuc_cfg.l2c_config = l2c_config;

    for (std::size_t cpu = 0; cpu < cpu_n_; cpu++) {
      cc::L1CacheAgentConfig l1c_config;
      l1c_config.name += std::to_string(cpu);
      l1c_config.pbuilder = pb;
      cpuc_cfg.l1c_configs.push_back(l1c_config);

      cc::CpuConfig cpu_config;
      cpu_config.name += std::to_string(cpu);
      cpuc_cfg.cpu_configs.push_back(cpu_config);
    }

    cfg.ccls.push_back(cpuc_cfg);
  }

  // Defined Memory controller
  cc::MemModelConfig mcfg;
  cfg.mcfgs.push_back(mcfg);

  // Define Directory model
  for (std::size_t d = 0; d < dir_n_; d++) {
    cc::DirModelConfig dcfg;
    dcfg.name += std::to_string(d);
    dcfg.pbuilder = pb;

    cfg.dcfgs.push_back(dcfg);
  }

  // Define NOC configuration.
  cfg.noccfg = construct_noc(cfg);

  // Set stimulus
  cfg.scfg = stimulus_config_;

  return cfg;
}

cc::NocModelConfig ConfigBuilder::construct_noc(const cc::SocConfig& cfg) const {
  // Default cost for all edges
  const cc::epoch_t cost = 10;

  std::vector<std::string> vs;
  cc::NocModelConfig noccfg;
  // Top:
  vs.push_back(cfg.name);
  // CPU -> DIR
  for (const cc::CpuClusterConfig& cluster_cfg : cfg.ccls) {
    // Cluster
    vs.push_back(cluster_cfg.name);
    // Cache Controller
    vs.push_back(cluster_cfg.cc_config.name);
    // Compute origin path
    const std::string origin = test::join(vs.begin(), vs.end());

    vs.pop_back();
    vs.pop_back();
    for (const cc::DirModelConfig& dir_config : cfg.dcfgs) {
      // Directory name
      vs.push_back(dir_config.name);
      // Compute destination path
      const std::string dest = test::join(vs.begin(), vs.end());
      noccfg.edges[origin][dest] = cost;
      vs.pop_back();
    }
  }

  // DIR -> CPU
  for (const cc::DirModelConfig& dir_config : cfg.dcfgs) {
    // Directory name
    vs.push_back(dir_config.name);
    // Compute destination path
    const std::string origin = test::join(vs.begin(), vs.end());
    vs.pop_back();
    for (const cc::CpuClusterConfig& cluster_cfg : cfg.ccls) {
      // Cluster
      vs.push_back(cluster_cfg.name);
      // Cache Controller
      vs.push_back(cluster_cfg.cc_config.name);
      // Compute origin path
      const std::string dest = test::join(vs.begin(), vs.end());
      noccfg.edges[origin][dest] = cost;

      vs.pop_back();
      vs.pop_back();
    }
  }
  return noccfg;
}

std::string path_l1c_by_cpu_id(const cc::SocConfig& cfg, std::size_t id) {
  std::size_t cluster_id = 0, cpu_offset = 0;
  for (const cc::CpuClusterConfig& ccfg : cfg.ccls) {
    const std::size_t l1_count = ccfg.l1c_configs.size();
    if (id < cpu_offset + l1_count) {
      const std::size_t cpu_idx = id - cpu_offset;

      std::vector<std::string> vs;
      // Top
      vs.push_back(cfg.name);
      // Cluster
      vs.push_back(ccfg.name);
      // Cpu
      vs.push_back(ccfg.l1c_configs[cpu_idx].name);
      // Complete!
      return test::join(vs.begin(), vs.end());
    } else {
      cluster_id++;
      cpu_offset += l1_count;
    }
  }
  // CPU ID is invalid; too large.
  const std::string reason = "Cannot find cpu id = " + std::to_string(id);
  throw std::invalid_argument(reason);
}

std::string path_dir_by_dir_id(const cc::SocConfig& cfg, std::size_t id) {
  std::vector<std::string> vs;
  // Top
  vs.push_back(cfg.name);
  // Directory
  vs.push_back(cfg.dcfgs[id].name);
  return test::join(vs.begin(), vs.end());
}

} // namespace test
