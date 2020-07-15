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

namespace cc {

// Forwards
class Stimulus;
class L1CacheModelProtocol;
class L2CacheModelProtocol;
class DirectoryProtocol;
class DirectoryMapper;
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
  // CPU stimulus driver
  Stimulus* stimulus = nullptr;
};

//
//
struct L1CacheModelConfig {
  // L1 Cache Model name
  std::string name = "l1cache";

  // Pointer to the transaction source instance for the current l1
  // cache instance (models the notion of a microprocessor
  // periodically emitting load/store instructions to memory).
  Stimulus* stim = nullptr;

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
struct L2CacheModelConfig {
  // L2 Cache Model name
  std::string name = "l2cache";

  // Child L1 client configurations.
  std::vector<L1CacheModelConfig> l1configs;

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

  // The number of ingress ports;
  std::size_t ingress_ports_n = 1;

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
struct DirectoryModelConfig {
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
struct CacheControllerCfg {
  // Controller name
  std::string name = "ccntrl";

  // Protocol builder instance.
  ProtocolBuilder* pbuilder = nullptr;
};

//
//
struct CpuClusterCfg {
  // Cluster name;
  std::string name = "cluster";

  // Cache Controller config;
  CacheControllerCfg cc_config;

  // L2 cache configuration.
  L2CacheModelConfig l2c_config;

  // L1 cache configuration(s)
  std::vector<L1CacheModelConfig> l1c_configs;

  // CPU configurations
  std::vector<CpuConfig> cpu_configs;
};

//
//
struct StimulusCfg {
  // Module name
  std::string name = "stimulus";
  // Trace file.
  std::string filename;
  // Index to cpu instance mapping.
  std::vector<std::string> cpath;
};

//
//
struct SocCfg {
  // Toplevel name
  std::string name = "top";
  // Cpu Cluster configuration.
  std::vector<CpuClusterCfg> ccls;
  // Directory configuration.
  std::vector<DirectoryModelConfig> dcfgs;
  // Stimulus configuration.
  StimulusCfg scfg;
  // NOC/Interconnect configuration.
  NocModelConfig noccfg;
};

}  // namespace cc

#endif
