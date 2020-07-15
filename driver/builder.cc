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

#include "driver.h"
#include "builder.h"
#include "cc/stimulus.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace cc {

class SocConfigBuilderJson {
#define CHECK(__name)                                                   \
  MACRO_BEGIN                                                           \
  if (!j.contains(#__name)) {                                           \
    BuilderException ex("Required argument not found: " #__name);       \
    ex.set_line(__LINE__);                                              \
    ex.set_file(__FILE__);                                              \
    throw ex;                                                           \
  }                                                                     \
  MACRO_END
  
  
#define CHECK_AND_SET(__name)                                           \
  MACRO_BEGIN                                                           \
  if (j.contains(#__name)) {                                            \
    c.__name = j[#__name];                                              \
  } else  {                                                             \
    BuilderException ex("Required argument not found: " #__name);       \
    ex.set_line(__LINE__);                                              \
    ex.set_file(__FILE__);                                              \
    throw ex;                                                           \
  }                                                                     \
  MACRO_END

#define THROW_EX(__desc)                        \
  MACRO_BEGIN                                   \
  BuilderException ex(__desc);                  \
  ex.set_line(__LINE__);                        \
  ex.set_file(__FILE__);                        \
  throw ex;                                     \
  MACRO_END
  

#define CHECK_AND_SET_OPTIONAL(__name)                  \
  if (j.contains(#__name)) c.__name = j[#__name]

 public:
  SocConfigBuilderJson(std::istream& is) : is_(is) {
    is >> jtop_;
  }

  SocConfig build() {
    SocConfig soc;
    build(soc, jtop_);
    post(soc);
    return soc;
  }

 private:
  void build(CacheModelConfig& c, json j) {
    // Set .sets_n
    CHECK_AND_SET_OPTIONAL(sets_n);
    // Set .ways_n
    CHECK_AND_SET_OPTIONAL(ways_n);
    // Set .line_bytes_n
    CHECK_AND_SET_OPTIONAL(line_bytes_n);
  }

  void build(CpuConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
  }

  void build(L1CacheModelConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
    // Set .l1_cmdq_slots_n
    CHECK_AND_SET_OPTIONAL(l1_cmdq_slots_n);
    // Set .l2_cmdq_credits_n
    CHECK_AND_SET_OPTIONAL(l2_cmdq_credits_n);
    // Set .ldst_flush_penalty_n
    CHECK_AND_SET_OPTIONAL(ldst_flush_penalty_n);
    // Set .cconfig
    CHECK(cconfig);
    build(c.cconfig, j["cconfig"]);
    
  }

  void build(L2CacheModelConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
    // Set .cconfig
    CHECK(cconfig);
    build(c.cconfig, j["cconfig"]);
  }

  void build(NocModelConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
    // Set .ingress_q_n
    CHECK_AND_SET_OPTIONAL(ingress_q_n);
  }

  void build(LLCModelConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
    // Set .cmd_queue_n
    CHECK_AND_SET_OPTIONAL(cmd_queue_n);
    // Set .rsp_queue_n
    CHECK_AND_SET_OPTIONAL(rsp_queue_n);
  }

  void build(DirectoryModelConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
    // Set .cmd_queue_n
    CHECK_AND_SET_OPTIONAL(cmd_queue_n);
    // Set .rsp_queue_n
    CHECK_AND_SET_OPTIONAL(rsp_queue_n);
    // Set .is_null_filter
    CHECK_AND_SET_OPTIONAL(is_null_filter);
    // Set .cconfig
    CHECK(cconfig);
    build(c.cconfig, j["cconfig"]);
    // Set .llcconfig
    CHECK(llcconfig);
    build(c.llcconfig, j["llcconfig"]);
  }

  void build(CacheControllerConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
  }

  void build(CpuClusterConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
    // Set .cc_config
    CHECK(cc_config);
    build(c.cc_config, j["cc_config"]);
    // Set .l2c_config
    CHECK(l2c_config);
    build(c.l2c_config, j["l2c_config"]);
    // Set .l1c_config
    CHECK(l1c_config);
    for (const auto& item : j["l1c_config"]) {
      L1CacheModelConfig cmc;
      build(cmc, item);
      c.l1c_configs.push_back(cmc);
    }
    if (c.l1c_configs.empty())
      THROW_EX("No L1 are defined");
    // Set .cpu_configs
    CHECK(cpu_configs);
    for (const auto& item : j["cpu_configs"]) {
      CpuConfig cc;
      build(cc, item);
      c.cpu_configs.push_back(cc);
    }
    if (c.cpu_configs.empty())
      THROW_EX("No CPU are defined");
    if (c.cpu_configs.size() != c.l1c_configs.size()) {
      THROW_EX("CPU count does not equal L1 count.");
    }
  }

  void build(StimulusConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
    // Set .type
    CHECK_AND_SET(type);
    // Parse type related options
    if (c.type == "trace"){
      // Set .filename
      CHECK_AND_SET(filename);
      // Set .cpath
      for (const auto& path : j["cpath"]) {
        c.cpaths.push_back(path);
      }
    } else {
      throw BuilderException("Unknown stimulus type" + c.type);
    }
  }
  
  void build(SocConfig& c, json j) {
    // Set .name
    CHECK_AND_SET(name);
    // Set .protocol
    CHECK(protocol);
    // Construct protocol definition.
    const std::string protocol = jtop_["protocol"];
    pb_ = construct_protocol_builder(protocol);
    if (pb_ == nullptr) {
      const std::string msg = "Invalid protocol: " + protocol;
      throw BuilderException(msg);
    }
    // Set .ccls (CpuClusterConfig)
    CHECK(ccls);
    for (const auto& item : j["ccls"]) {
      CpuClusterConfig ccc;
      build(ccc, item);
      c.ccls.push_back(ccc);
    }
    if (c.ccls.empty())
      throw BuilderException("No CPU clusters configured.");
    // Set .dcfgs (DirectoryModelConfig)
    CHECK(dcfgs);
    for (const auto& item : j["dcfgs"]) {
      DirectoryModelConfig dmc;
      build(dmc, item);
      c.dcfgs.push_back(dmc);
    }
    if (c.dcfgs.empty())
      throw BuilderException("No directories configured");
    // Set .scfg (StimulusConfig)
    CHECK(scfg);
    build(c.scfg, j["scfg"]);
    // Set .noccfg (NocModelConfig)
    CHECK(noccfg);
    build(c.noccfg, j["noccfg"]);
  }

  void post(SocConfig& cfg) {
    for (CpuClusterConfig& c : cfg.ccls) {
      post(c);
    }
    for (DirectoryModelConfig& c : cfg.dcfgs) {
      post(c);
    }
  }

  void post(CpuClusterConfig& cfg) {
    post(cfg.cc_config);
    post(cfg.l2c_config);
    for (L1CacheModelConfig& l1c : cfg.l1c_configs) {
      post(l1c);
    }
  }

  void post(CacheControllerConfig& cfg) {
    cfg.pbuilder = pb_;
  }

  void post(L1CacheModelConfig& cfg) {
    cfg.pbuilder = pb_;
  }

  void post(L2CacheModelConfig& cfg) {
    cfg.pbuilder = pb_;
  }

  void post(DirectoryModelConfig& c) {
    c.pbuilder = pb_;
  }
  
  //
  ProtocolBuilder* pb_ = nullptr;
  // Input configuration stream.
  std::istream& is_;
  // Top json object.
  json jtop_;

#undef CHECK_AND_SET_OPTIONAL
#undef CHECK_AND_SET
#undef CHECK  
};


SocConfig build_soc_config(std::istream& is) {
  SocConfigBuilderJson builder(is);
  return builder.build();
}

} // namespace cc
