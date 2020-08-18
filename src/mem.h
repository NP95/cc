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

#ifndef CC_LIBC_DIR_H
#define CC_LIBC_DIR_H

#include <map>

#include "msg.h"
#include "sim.h"

namespace cc {

class NocPort;
class MemNocEndpoint;

//
//
enum class MemCmdOpcode { Read, Write };

const char* to_string(MemCmdOpcode opcode);

//
//
class MemCmdMsg : public Message {
 public:
  MemCmdMsg();

  //
  std::string to_string() const override;

  //
  MemCmdOpcode opcode() const { return opcode_; }
  Agent* dest() const { return dest_; }

  //
  void set_opcode(MemCmdOpcode opcode) { opcode_ = opcode; }
  void set_dest(Agent* dest) { dest_ = dest; }

 private:
  MemCmdOpcode opcode_;
  Agent* dest_ = nullptr;
};

//
//
enum class MemRspOpcode { ReadOkay, WriteOkay };

const char* to_string(MemRspOpcode opcode);

//
//
class MemRspMsg : public Message {
 public:
  MemRspMsg();

  //
  std::string to_string() const override;

  //
  MemRspOpcode opcode() const { return opcode_; }

  //
  void set_opcode(MemRspOpcode opcode) { opcode_ = opcode; }

 private:
  MemRspOpcode opcode_;
};

//
//
class DtMsg : public Message {
 public:
  DtMsg() : Message(MessageClass::Dt) {}

  //
  std::string to_string() const override;
};

//
//
class DtRspMsg : public Message {
 public:
  DtRspMsg() : Message(MessageClass::DtRsp) {}

  //
  std::string to_string() const override;
};

// Memory Controller Model
//
class MemCntrlModel : public Agent {
  class RequestDispatcherProcess;

  friend class SocTop;

 public:
  MemCntrlModel(kernel::Kernel* k);
  ~MemCntrlModel();

  // Accessors:
  MessageQueue* endpoint() const;

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
};

}  // namespace cc

#endif
