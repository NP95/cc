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

#ifndef CC_INCLUDE_CC_L2CACHE_H
#define CC_INCLUDE_CC_L2CACHE_H

#include "kernel.h"
#include "l1cache.h"
#include <vector>

namespace cc {

//
//
struct L2CacheModelConfig {
  // L2 Cache Model name
  std::string name = "l2cache";
  // Child L1 client configurations.
  std::vector<L1CacheModelConfig> l1configs;
};

//
//
class L2CacheModel : public kernel::Module {
  class MainProcess;
 public:
  enum EndPoints : kernel::end_point_id_t {
    L1CmdReq,
    L1CmdRsp    
  };
  
  L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config);
  virtual ~L2CacheModel();

  L2CacheModelConfig config() const { return config_; }
 protected:
  virtual void elab() override;
  virtual void drc() override;
 private:
  void build();

  // L2 Cache Configuration.
  L2CacheModelConfig config_;
  // Child L1 Caches
  std::vector<L1CacheModel*> l1cs_;
  // L1 Command Request
  MessageQueue* l1cmdreqq_;
  // L1 Command Response
  MessageQueue* l1cmdrspq_;
  // Queue selection arbiter
  Arbiter<const Message*>* arb_;
  //
  MainProcess* main_;
};

} // namespace cc

#endif
