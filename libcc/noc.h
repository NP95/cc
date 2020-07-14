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

#ifndef CC_LIBCC_NOC_H
#define CC_LIBCC_NOC_H

#include "kernel.h"
#include "sim.h"
#include "cc/cfgs.h"
#include "msg.h"
#include <vector>
#include <map>

namespace cc {


class MessageQueue;
template<typename> class Arbiter;


class NocMsg : public Message {
 public:
  NocMsg();

  const Message* payload() const { return payload_; }
  Agent* origin() const { return origin_; }
  Agent* dest() const { return dest_; }

  void set_payload(const Message* payload) { payload_ = payload; }
  void set_origin(Agent* origin) { origin_ = origin; }
  void set_dest(Agent* dest) { dest_ = dest; }
  
 private:
  const Message* payload_ = nullptr;
  Agent* origin_ = nullptr;
  Agent* dest_ = nullptr;
};

class NocPort : public kernel::Module {
  friend class SocTop;
 public:
  NocPort(kernel::Kernel* k, const std::string& name);
  ~NocPort();

  // Owned by port
  MessageQueue* ingress() const { return ingress_; }

  // Owned by agent
  MessageQueue* egress() const { return egress_; }
 private:
  // Build phase:
  void build();

  // Elaboration phase:
  void set_egress(MessageQueue* egress) { egress_ = egress; }
  
  MessageQueue* ingress_ = nullptr;
  MessageQueue* egress_ = nullptr;
};

//
//
class NocModel : public Agent {
  class MainProcess;

  friend class SocTop;
 public:
  
  NocModel(kernel::Kernel* k, const NocModelConfig& config);

  const NocModelConfig& config() const { return config_; }

  NocPort* get_agent_port(Agent* agent);

 protected:

  // Build Phase
  void build();
  // Construct new agent interface.
  void register_agent(Agent* agent);

  // Elaboration Phase
  virtual void elab() override;

  // Design Rule Check Phase
  virtual void drc() override;

  // Accessors;

  // Arbiter
  Arbiter<const Message*>* arb() const { return arb_; }
  
 private:
  // Queue selection arbiter
  Arbiter<const Message*>* arb_ = nullptr;
  // Set of ingress Message Queues
  std::map<Agent*, NocPort*> ports_;
  // Main thread of execution.
  MainProcess* main_ = nullptr;
  // NOC Configuration
  NocModelConfig config_;
};

} // namespace 

#endif
