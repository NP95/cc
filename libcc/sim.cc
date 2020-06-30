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

#include "cc/sim.h"

namespace cc {

L1CacheModel::L1CacheModel(kernel::Kernel* k, const L1CacheModelConfig& config)
    : kernel::Module(k, config.name), config_(config), ts_(config.ts) {
}

L1CacheModel::~L1CacheModel() {
}

void L1CacheModel::elab() {
  // Do elaborate
}

void L1CacheModel::drc() {
  // Do DRC
  if (ts_ == nullptr) {
    // No transaction source associated with L1; raise warning.
    const Message msg(
        "L1Cache has no associated transaction source.", Level::Warning);
    log(msg);
  }
}

L2CacheModel::L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config)
    : kernel::Module(k, config.name), config_(config) {
  build();
}

L2CacheModel::~L2CacheModel() {
}

void L2CacheModel::build() {
  for (const L1CacheModelConfig& l1cfg : config_.l1configs) {
    L1CacheModel* l1c = new L1CacheModel(k(), l1cfg);
    add_child(l1c);
    l1cs_.push_back(l1c);
  }
}

void L2CacheModel::elab() {
  // Do elaborate
}

void L2CacheModel::drc() {
  Module::drc();
  // Do DRC
}

} // namespace cc
