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

#ifndef CC_INCLUDE_CC_SIM_H
#define CC_INCLUDE_CC_SIM_H

#include "kernel.h"
#include "primitives.h"
#include <vector>

namespace cc {

struct L1CacheModelConfig {
  // L1 Cache Model name
  std::string name = "l1cache";
  // Pointer to the transaction source instance for the current l1
  // cache instance (models the notion of a microprocessor
  // periodically emitting load/store instructions to memory).
  TransactionSource* ts = nullptr;
};

class L1CacheModel : public kernel::Module {
 public:
  L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config);
  virtual ~L1CacheModel();

  L1CacheModelConfig config() const { return config_; }
 protected:
  virtual void elab() override;
  virtual void drc() override;
 private:
  TransactionSource* ts_;
  L1CacheModelConfig config_;
};

struct L2CacheModelConfig {
  // L2 Cache Model name
  std::string name = "l2cache";
  // Child L1 client configurations.
  std::vector<L1CacheModelConfig> l1configs;
};

class L2CacheModel : public kernel::Module {
 public:
  L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config);
  virtual ~L2CacheModel();

  L2CacheModelConfig config() const { return config_; }
 protected:
  virtual void elab() override;
  virtual void drc() override;
 private:
  void build();
  L2CacheModelConfig config_;
  std::vector<L1CacheModel*> l1cs_;
};

} // namespace cc

#endif
