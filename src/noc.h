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

#include <map>
#include <vector>

#include "cc/cfgs.h"
#include "kernel.h"
#include "msg.h"
#include "sim.h"

namespace cc {

class MessageQueue;
class MessageQueueArbiter;
class Monitor;

class NocMsg : public Message {
  template <typename>
  friend class PooledItem;

  NocMsg();
 public:

  std::string to_string() const override;

  //
  const Message* payload() const { return payload_; }
  Agent* origin() const { return origin_; }
  Agent* dest() const { return dest_; }

  //
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

  // Ingress Message Queue (Owned by port)
  MessageQueue* ingress() const { return ingress_; }

  // Ingress Credit Counter (Owned by port)
  CreditCounter* ingress_cc() const { return ingress_cc_; }

  // Egress Message Queue (Owned by agent)
  MessageQueue* egress() const { return egress_; }

 private:
  // Build phase:
  void build();

  // Elaboration phase:
  void set_egress(MessageQueue* egress) { egress_ = egress; }

  // Ingress (Agent -> NOC) Message Queue
  MessageQueue* ingress_ = nullptr;

  // Ingress Credit Counter.
  CreditCounter* ingress_cc_ = nullptr;

  // Egress (NOC -> Agent) Message Queue
  MessageQueue* egress_ = nullptr;
};

//
//
class NocEndpoint : public Agent {
  class MainProcess;

 public:
  NocEndpoint(kernel::Kernel* k, const std::string& name);
  virtual ~NocEndpoint();

  // Agent period
  time_t epoch() const { return epoch_; }

  // Set EndPoint epoch (period)
  void set_epoch(time_t epoch) { epoch_ = epoch; }

  //
  MessageQueue* ingress_mq() const { return ingress_mq_; }

 protected:
  //
  virtual MessageQueue* lookup_mq(const Message* msg) const = 0;

 private:
  // Construct
  void build();

  //
  virtual bool elab() override;

  // Agent
  time_t epoch_ = 0;

  //
  MainProcess* main_ = nullptr;

  //
  MessageQueue* ingress_mq_ = nullptr;
};

//
//
class NocTimingModel {
 public:
  NocTimingModel() = default;

  // Accessors
  time_t base() const { return base_; }

  // Set base edge cost.
  void set_base(time_t d) { base_ = d; }

  // Lookup cost for origin -> dest edge
  time_t cost(const Agent* origin, const Agent* dest) const;

  // Register edge {origin, dest} -> cost;
  void register_edge(const Agent* origin, const Agent* dest, time_t delay);

 private:
  // Base edge delay
  time_t base_ = 0;
  // Timing information {Agent*, Agent*} -> time_t
  std::map<const Agent*, std::map<const Agent*, time_t> > timing_;
};

//
//
class NocModel : public Agent {
  class MainProcess;

  friend class SocTop;

 public:
  NocModel(kernel::Kernel* k, const NocModelConfig& config);
  ~NocModel();

  const NocModelConfig& config() const { return config_; }

  NocPort* get_agent_port(Agent* agent);

 protected:
  // Build Phase
  void build();
  // Construct new agent interface.
  void register_agent(Agent* agent);
  // Register verification monitor.
  void register_monitor(Monitor* monitor);

  // Elaboration Phase
  bool elab() override;
  // Register timing model
  void register_timing_model(const NocTimingModel* tm) { tm_ = tm; }

  // Design Rule Check Phase
  virtual void drc() override;

  // Accessors;

  // Arbiter
  MQArb* arb() const { return arb_; }
  // Timing Model
  const NocTimingModel* tm() const { return tm_; }

 private:
  // Queue selection arbiter
  MQArb* arb_ = nullptr;
  // Set of ingress Message Queues
  std::map<Agent*, NocPort*> ports_;
  // Main thread of execution.
  MainProcess* main_ = nullptr;
  // Verification monitor
  Monitor* monitor_ = nullptr;
  // Timing model
  const NocTimingModel* tm_ = nullptr;
  // NOC Configuration
  NocModelConfig config_;
};

}  // namespace cc

#endif
