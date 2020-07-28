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

#ifndef CC_INCLUDE_CC_SOC_H
#define CC_INCLUDE_CC_SOC_H

#include "kernel.h"
#include <string>

namespace cc {

class SocConfig;
class SocTop;
class ProtocolBuilder;
class DirMapper;
class NocModel;
class DirModel;
class LLCModel;
class CpuCluster;
class MemCntrlModel;
class Stimulus;

//
//
class SocTop : public kernel::TopModule {
 public:
  SocTop(kernel::Kernel* k, const SocConfig& cfg);

  ~SocTop();

 private:
  void build(const SocConfig& cfg);

  void elab() override;

  void drc() override;

  // Directory Mapper instance
  DirMapper* dm_;
  // NOC/Interconnect instance
  NocModel* noc_ = nullptr;
  // Directory model instance
  std::vector<DirModel*> dms_;
  // LLC Models 1-to-1 relationship with non-Null Filter directories.
  std::vector<LLCModel*> llcs_;
  // CPU Cluster instances
  std::vector<CpuCluster*> ccs_;
  // Memory Controller instances.
  std::vector<MemCntrlModel*> mms_;
  // Stimulus "module" instance.
  Stimulus* stimulus_;
};

class Soc {
 public:
  Soc(const SocConfig& soccfg);
  ~Soc();

  void initialize();

  void run();

  void finalize();
  
 private:
  void build(const SocConfig& config);

  // Simulation kernel instance
  kernel::Kernel* kernel_;
  // Top module instance.
  SocTop* top_;
};

//
//
Soc* construct_soc(const SocConfig& soccfg);

//
//
ProtocolBuilder* construct_protocol_builder(const std::string& name);

} // namespace cc

#endif
