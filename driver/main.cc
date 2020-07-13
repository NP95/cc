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

#include "cc.h"
#include <vector>


cc::SocCfg generate_cfg() {
  cc::ProtocolBuilder* pbuilder = cc::construct_protocol_builder("moesi");

  cc::CpuClusterCfg cfg;

  cfg.cc_config.pbuilder = pbuilder;
  cfg.l2c_config.pbuilder = pbuilder;
  for (int i = 0; i < 1; i++) {
    cc::L1CacheModelConfig l1c;
    l1c.pbuilder = pbuilder;
    cfg.l1c_configs.push_back(l1c);

    cc::CpuConfig cpu;
    
    cc::ProgrammaticStimulus* s = new cc::ProgrammaticStimulus;
    s->push_back(cc::kernel::Time{100}, cc::Command{cc::CpuOpcode::Load, 0});
    // s->push_back(cc::kernel::Time{100}, cc::Command{cc::CpuOpcode::Load, 0});
    cpu.stimulus = s;
    
    cfg.cpu_configs.push_back(cpu);
  }

  // Construct a directory
  cc::DirectoryModelConfig dir;
  dir.pbuilder = pbuilder;
  
  cc::SocCfg soc;
  soc.ccls.push_back(cfg);
  soc.dcfgs.push_back(dir);
  return soc;
}

int main(int argc, char** argv) {
  // Simulation configuration.

  cc::Soc* soc = cc::construct_soc(generate_cfg());
  soc->initialize();
  soc->run();
  soc->finalize();
  delete soc;

  return 0;
}
