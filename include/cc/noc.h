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

#ifndef CC_INCLUDE_CC_NOC_H
#define CC_INCLUDE_CC_NOC_H

#include "kernel.h"
#include "cc/primitives.h"
#include "cc/cfgs.h"
#include "cc/msg.h"
#include <vector>

namespace cc {

class Message;
class MessageQueue;
template<typename> class Arbiter;

//
//
class NocMessage : public Message {
 public:
  NocMessage(Transaction* t) : Message(t, MessageClass::Noc) {}

  //
  const Message* payload() const { return payload_; }
  void set_payload(const Message* payload) { payload_ = payload; }

  //
  kernel::EndPointIntf<const Message*>* ep() const { return ep_; }
  void set_ep(kernel::EndPointIntf<const Message*>* ep) { ep_ = ep; }

  //
  kernel::Agent<const Message*>* origin() const { return origin_; }
  void set_origin(kernel::Agent<const Message*>* origin) { origin_ = origin; }

 private:
  // Message Payload
  const Message* payload_ = nullptr;
  // Destination end-point
  kernel::EndPointIntf<const Message*>* ep_ = nullptr;
  // Originating agent.
  kernel::Agent<const Message*>* origin_ = nullptr;
};


//
//
class NocModel : public Agent {
  class MainProcess;
 public:
  NocModel(kernel::Kernel* k, const NocModelConfig& config);

  const NocModelConfig& config() const { return config_; }

  //
  kernel::EndPointIntf<const Message*>* get_input(std::size_t n);

 protected:

  // Build Phase
  void build();

  // Elaboration Phase
  virtual void elab() override;

  // Design Rule Check Phase
  virtual void drc() override;

  // Accessors;

  // Arbiter
  Arbiter<const Message*>* arb() const { return arb_; }

  // Ingress Message Queue(s)
  const std::vector<MessageQueue*>& imqs() const { return imqs_; }
  
 private:
  // Queue selection arbiter
  Arbiter<const Message*>* arb_ = nullptr;
  // Set of ingress Message Queues
  std::vector<MessageQueue*> imqs_;
  // Main thread of execution.
  MainProcess* main_ = nullptr;
  // NOC Configuration
  NocModelConfig config_;
};

} // namespace 

#endif
