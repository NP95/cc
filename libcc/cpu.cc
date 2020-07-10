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

#include "cpu.h"
#include "utility.h"
#include "msg.h"
#include "l1cache.h"
#include <sstream>

namespace cc {

class Cpu::ProducerProcess : public kernel::Process {
 public:
  ProducerProcess(kernel::Kernel* k, const std::string& name, Cpu* parent)
      : kernel::Process(k, name), parent_(parent) {}
 private:
  // Initialization
  virtual void init() override {
    wait_for(kernel::Time{100, 0});
  }

  // Finalization
  virtual void fini() override {
  }

  // Elaboration
  virtual void eval() override {
    CpuL1__CmdMsg* msg = new CpuL1__CmdMsg;
    msg->set_opcode(L1CpuOpcode::Load);
    msg->set_addr(0);
    parent_->issue(parent_->cpu_l1__cmd_q(), kernel::Time{10, 0}, msg);
  }

  // Point to process owner module.
  Cpu* parent_ = nullptr;
};

class Cpu::ConsumerProcess : public kernel::Process {
 public:
  ConsumerProcess(kernel::Kernel* k, const std::string& name, Cpu* parent)
      : kernel::Process(k, name), parent_(parent) {}
 private:
  // Initialization
  virtual void init() override {
  }

  // Finalization
  virtual void fini() override {
  }

  // Elaboration
  virtual void eval() override {
  }

  // Point to process owner module.
  Cpu* parent_ = nullptr;
};

Cpu::Cpu(kernel::Kernel* k, const CpuConfig& config)
    : Agent(k, config.name), config_(config) {
  build();
}

void Cpu::build() {
  // Capture stimulus
  stimulus_ = config_.stimulus;

  if (stimulus_ != nullptr) {
    // Construct producer thread
    pp_ = new ProducerProcess(k(), "producer", this);
    add_child_process(pp_);
    // Construct consumer thread
    cp_ = new ConsumerProcess(k(), "consumer", this);
    add_child_process(cp_);
  }
}

void Cpu::elab() {
}

void Cpu::drc() {
  // Do DRC
  if (stimulus_ == nullptr) {
    // No transaction source associated with L1; raise warning.
    const LogMessage msg("L1Cache has no associated stimulus.", Level::Warning);
    log(msg);
  }
}

}  // namespace cc
