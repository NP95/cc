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
#include <memory>

namespace cc {

// Forwards
class Stimulus;
class L1CacheModelProtocol;
class L2CacheAgentProtocol;
class DirProtocol;
class DirMapper;
class ProtocolBuilder;

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
struct CpuConfig {
  // Instance name
  std::string name = "cpu";
};

//
//
struct L1CacheModelConfig {
  // L1 Cache Model name
  std::string name = "l1cache";

  // Number of slots in the command queue.
  std::size_t l1_cmdq_slots_n = 3;

  // Number of credits/slots reserved to the L2 cache command queue.
  std::size_t l2_cmdq_credits_n = 3;

  // LD/ST pipe flush penalty (cycles)
  std::size_t ldst_flush_penalty_n = 3;

  // Cache configuration.
  CacheModelConfig cconfig;

  // Protocol builder instance.
  ProtocolBuilder* pbuilder = nullptr;
};

//
//
struct L2CacheAgentConfig {
  // L2 Cache Model name
  std::string name = "l2cache";

  // Cache configuration.
  CacheModelConfig cconfig;

  // Protocol builder instance.
  ProtocolBuilder* pbuilder = nullptr;

};

//
//
struct NocModelConfig {
  // NOC model name
  std::string name = "noc";
  // Ingress Message Queue capacity in messages.
  std::size_t ingress_q_n = 16;
};

//
//
struct LLCModelConfig {
  // LLC name
  std::string name = "llc";

  // Command Queue size
  std::size_t cmd_queue_n = 4;

  // Response Queue size
  std::size_t rsp_queue_n = 4;
};

//
//
struct DirModelConfig {
  // Directory name
  std::string name = "dir";

  // Command Queue size
  std::size_t cmd_queue_n = 4;

  // Response Queue size
  std::size_t rsp_queue_n = 4;

  // Flag indicatig whether the directory is a null filter (does not
  // have and associated LLC).
  bool is_null_filter = false;

  // Cache configuratioh (non-Null Filter case).
  CacheModelConfig cconfig;

  // In the non-Null filter case, the directory maintains the LLC. The
  // LLC cache configuration matches the directory one-to-one.
  LLCModelConfig llcconfig;

  // Protocol builder instance.
  ProtocolBuilder* pbuilder = nullptr;
};

//
//
struct CCConfig {
  // Controller name
  std::string name = "ccntrl";

  // Protocol builder instance.
  ProtocolBuilder* pbuilder = nullptr;
};

//
//
struct CpuClusterConfig {
  // Cluster name;
  std::string name = "cluster";

  // Cache Controller config;
  CCConfig cc_config;

  // L2 cache configuration.
  L2CacheAgentConfig l2c_config;

  // L1 cache configuration(s)
  std::vector<L1CacheModelConfig> l1c_configs;

  // CPU configurations
  std::vector<CpuConfig> cpu_configs;
};

//
//
struct StimulusConfig {
  // Module name
  std::string name = "stimulus";
  // Type
  std::string type = "trace";
  // Trace file.
  std::string filename;
  // Index to cpu instance mapping.
  std::vector<std::string> cpaths;
  //
  std::istream* is = nullptr;
};

//
//
struct SocConfig {
  ~SocConfig();
  // Toplevel name
  std::string name = "top";
  // Coherence protocol
  std::string protocol = "moesi";
  // Cpu Cluster configuration.
  std::vector<CpuClusterConfig> ccls;
  // Directory configuration.
  std::vector<DirModelConfig> dcfgs;
  // Stimulus configuration.
  StimulusConfig scfg;
  // NOC/Interconnect configuration.
  NocModelConfig noccfg;
  // Protocol Builder
  ProtocolBuilder* pbuilder;
};

}  // namespace cc

#endif
