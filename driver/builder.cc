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

#include "builder.h"
#include "cc/stimulus.h"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace cc {

class SocCfgBuilderJson {
 public:
  SocCfgBuilderJson(std::istream& is) : is_(is) {
    is >> jtop_;
  }

  SocCfg build() {
    SocCfg soc;
    build_top(soc, jtop_);
    
    CpuClusterCfg cfg;
    cfg.cc_config.pbuilder = pb_;
    cfg.l2c_config.pbuilder = pb_;
    for (int i = 0; i < 1; i++) {
      L1CacheModelConfig l1c;
      l1c.pbuilder = pb_;
      cfg.l1c_configs.push_back(l1c);

      CpuConfig cpu;
    
      //  ProgrammaticStimulus* s = new ProgrammaticStimulus;
      //s->push_back(kernel::Time{100}, Command{CpuOpcode::Load, 0});
      //cpu.stimulus = s;
    
      cfg.cpu_configs.push_back(cpu);
    }

    // Construct a directory
    DirectoryModelConfig dir;
    dir.pbuilder = pb_;

    soc.ccls.push_back(cfg);
    soc.dcfgs.push_back(dir);

    soc.scfg.filename = jtop_["filename"];
    soc.scfg.cpath.push_back("top.cluster.cpu");
    
    return soc;
  }

 private:
  void build_top(SocCfg& cfg, json j) {
    if (j.contains("protocol")) {
      // Construct protocol definition.
      const std::string protocol = jtop_["protocol"];
      pb_ = construct_protocol_builder(protocol);
      if (pb_ == nullptr) {
        const std::string msg = "Invalid protocol: " + protocol;
        throw BuilderException(msg);
      }
    } else {
      throw BuilderException("Protocol is not specified.");
    }
  }
  //
  ProtocolBuilder* pb_ = nullptr;
  // Input configuration stream.
  std::istream& is_;
  // Top json object.
  json jtop_;
};


SocCfg build_soc_config(std::istream& is) {
  SocCfgBuilderJson builder(is);
  return builder.build();
}

} // namespace cc
