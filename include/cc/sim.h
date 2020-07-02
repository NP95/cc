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


//
//

//
//
struct Transaction {
  
};

//
//
struct Message {
  // Pointer to the transaction to which this message is a party.
  Transaction* t;
};


//
//
class MessageQueue : public kernel::Module, public RequesterIntf<Message*> {
 public:
  MessageQueue(kernel::Kernel* k, const std::string& name, std::size_t n);

  // Queue depth.
  std::size_t n() const { return q_->n(); }
  
  // Requester Interface:
  bool is_requesting() const override;
  void grant() override;
  Message* data() const override;
  kernel::Event& request_arrival_event() override;
  
 private:
  // Construct module
  void build(std::size_t n);
  // Queue primitive.
  Queue<Message*>* q_ = nullptr;
};

//
//
class ProcessorModel : public kernel::Module, public RequesterIntf<Message*> {
 public:
  ProcessorModel(kernel::Kernel* k, const std::string& name);

  // Set stimulus object for processor.
  void set_stimulus(Stimulus* stim) { stim_ = stim; }
  
  // Requester Interface:
  bool is_requesting() const override;
  void grant() override;
  Message* data() const override;
  kernel::Event& request_arrival_event() override;
 private:
  void build();
  void elab() override;
  void drc() override;
  Stimulus* stim_ = nullptr;
};

} // namespace cc

#endif
