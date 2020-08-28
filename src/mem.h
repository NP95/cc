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

#ifndef CC_SRC_MEM_H
#define CC_SRC_MEM_H

#include <map>

#include "cc/cfgs.h"
#include "msg.h"
#include "sim.h"

namespace cc {

class NocPort;
class MemNocEndpoint;

// Memory command opcodes:
enum class MemCmdOpcode {
  // Read some data from memory.
  Read,

  // Write some data to memory.
  Write,

  // Invalid; place-holder.
  Invalid
};

// Convert opcode to human readable string.
const char* to_string(MemCmdOpcode opcode);

//
//
class MemCmdMsg : public Message {
  template <typename>
  friend class PooledItem;

  MemCmdMsg();
 public:
  // Produce human-readable string of message.
  std::string to_string() const override;

  // Accessors:
  
  // Command opcode
  MemCmdOpcode opcode() const { return opcode_; }

  // Command destination agent (of Dt, where applicable)
  Agent* dest() const { return dest_; }


  // Setters:

  // Set message opcode
  void set_opcode(MemCmdOpcode opcode) { opcode_ = opcode; }

  // Set message destination Agent.
  void set_dest(Agent* dest) { dest_ = dest; }

 private:
  // Command opcode
  MemCmdOpcode opcode_ = MemCmdOpcode::Invalid;

  // Destination agent.
  Agent* dest_ = nullptr;
};


// Memory response opcodes.
//
enum class MemRspOpcode {
  // Read completed okay
  ReadOkay,

  // Write completed okay
  WriteOkay,

  // Invalid; place-holder.
  Invalid
};

// Convert to a humand-readable string.
const char* to_string(MemRspOpcode opcode);


//
//
class MemRspMsg : public Message {
  template <typename>
  friend class PooledItem;
  
  MemRspMsg();
 public:

  // Produce humand-readable string of message. 
  std::string to_string() const override;

  // Accessors:
  
  // Response opcode.
  MemRspOpcode opcode() const { return opcode_; }


  // Setters:
  
  // Set opcode
  void set_opcode(MemRspOpcode opcode) { opcode_ = opcode; }

 private:
  // Response opcode.
  MemRspOpcode opcode_ = MemRspOpcode::Invalid;
};


// Message encapsulating the notion of the transfer of one cache-line
// of memory between agents.
//
class DtMsg : public Message {
  template <typename>
  friend class PooledItem;

  DtMsg();
 public:
  // Produce human-readable string of message.
  std::string to_string() const override;
};


// Message encapsulating the notion of a response to a prior Data
// Transfer message.
//
class DtRspMsg : public Message {
  template <typename>
  friend class PooledItem;
  
  DtRspMsg();
 public:

  // Produce human-readable string of message.
  std::string to_string() const override;
};

// Memory Controller Model
//
class MemCntrlAgent : public Agent {
  class RequestDispatcherProcess;

  friend class SocTop;

 public:
  MemCntrlAgent(kernel::Kernel* k, const MemModelConfig& config);
  ~MemCntrlAgent();

  // Accessors:
  MessageQueue* endpoint() const;

  // Configuration
  const MemModelConfig& config() const { return config_; }

  // MEM -> NOC
  NocPort* mem_noc__port() const { return mem_noc__port_; }

 protected:
  // Build
  void build();
  // Add a memory controller client to the current instance;
  // constructs the necessary ingress queues beyond the NOC interface
  // front-end.
  void register_agent(Agent* agent);

  // Elaboration
  bool elab() override;
  //
  void set_mem_noc__port(NocPort* port);

  // Design Rule Check (DRC)
  void drc() override;

  // Accessors(s)

  // RDIS aribter
  MQArb* rdis_arb() const { return rdis_arb_; }

 private:
  // NOC -> MEM message queue (owned by directory)
  MessageQueue* noc_mem__msg_q_ = nullptr;
  // MEM -> NOC message queue (owned by noc)
  NocPort* mem_noc__port_ = nullptr;
  // NOC endpoint
  MemNocEndpoint* noc_endpoint_ = nullptr;
  // Request Dispatcher process
  RequestDispatcherProcess* rdis_proc_ = nullptr;
  // Request Dispatcher arbitrator
  MQArb* rdis_arb_ = nullptr;
  // Request Dispatcher memory queues
  std::map<Agent*, MessageQueue*> rdis_mq_;
  // Configuration
  MemModelConfig config_;
};

}  // namespace cc

#endif
