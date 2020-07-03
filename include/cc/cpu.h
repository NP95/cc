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

#ifndef CC_INCLUDE_CC_CPU_H
#define CC_INCLUDE_CC_CPU_H

#include "kernel.h"
#include "sim.h"
#include <set>

namespace cc {

// Forwards
class Transaction;
class Message;


//
//
class Cpu : public kernel::Module
          , public kernel::EndPointIntf<const Message*>
          , public kernel::RequesterIntf<const Message*> {

  class MainProcess;
 public:
  Cpu(kernel::Kernel* k, const std::string& name);

  // Set stimulus object for processor.
  void set_stimulus(Stimulus* stim) { stim_ = stim; }

  // End-Point interface:
  void push(const Message* msg) override;

  // Requester Interface:
  bool has_req() const override;
  const Message* peek() const override;
  const Message* dequeue() override;
  kernel::Event& request_arrival_event() override;
 private:
  void build();
  void elab() override;
  void drc() override;
  void construct_new_message() const;
  //
  kernel::Event mature_event_;
  //
  Stimulus* stim_ = nullptr;
  //
  MainProcess* main_ = nullptr;
  // Set of outstanding transactions.
  std::set<Transaction*> ts_;
  // Current head-of-queue message to be emitted.
  mutable const Message* msg_ = nullptr;
};

} // namespace cc

#endif
