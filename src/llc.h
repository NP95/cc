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

#ifndef CC_LIBCC_LLC_H
#define CC_LIBCC_LLC_H

#include <map>

#include "cfgs.h"
#include "kernel.h"
#include "msg.h"
#include "sim.h"

namespace cc {

// Forwards:
class MemCntrlModel;
class DirModel;
class Message;
class MessageQueue;
class LLCNocEndpoint;
class CpuCluster;

//
//
enum class LLCCmdOpcode {
  // Fill: transfer a line from main memory to the cache.
  Fill,
  // Evict: remove the contents of line (and conditionally fill).
  Evict,
  // Put: transfer line present in cache to some agent.
  PutLine
};

const char* to_string(LLCCmdOpcode opcode);

//
//
class LLCCmdMsg : public Message {
 public:
  LLCCmdMsg();

  //
  std::string to_string() const override;

  //
  LLCCmdOpcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }
  Agent* agent() const { return agent_; }

  //
  void set_opcode(LLCCmdOpcode opcode) { opcode_ = opcode; }
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_agent(Agent* agent) { agent_ = agent; }

 private:
  addr_t addr_;
  LLCCmdOpcode opcode_;
  Agent* agent_ = nullptr;
};

//
//
enum class LLCRspOpcode { Okay };

const char* to_string(LLCRspOpcode opcode);

//
//
class LLCCmdRspMsg : public Message {
 public:
  LLCCmdRspMsg();

  //
  std::string to_string() const override;

  //
  LLCRspOpcode opcode() const { return opcode_; }

  //
  void set_opcode(LLCRspOpcode opcode) { opcode_ = opcode; }

 private:
  LLCRspOpcode opcode_;
};

class LLCTState;

//
using LLCTTable = std::map<Transaction*, LLCTState*>;

//
//
class LLCModel : public Agent {
  class RdisProcess;

  friend class SocTop;

 public:
  LLCModel(kernel::Kernel* k, const LLCModelConfig& config);
  ~LLCModel();

  // Return model configuration.
  const LLCModelConfig& config() const { return config_; }

  // Accessors:
  // NOC -> LLC message queue
  MessageQueue* endpoint() const;
  // LLC -> NOC message queue
  MessageQueue* llc_noc__msg_q() const { return llc_noc__msg_q_; }
  // Home memory controller
  MemCntrlModel* mc() const { return mc_; }
  // Directory model instance.
  DirModel* dir() const { return dir_; }

 protected:
  // Construction/Build
  void build();
  //
  void register_cc(CpuCluster* cc);

  // Elaboration
  bool elab() override;
  // NOC -> LLC message queue
  void set_llc_noc__msg_q(MessageQueue* mq);
  // Set memory controller.
  void set_mc(MemCntrlModel* mc) { mc_ = mc; }
  // Set owner directory.
  void set_dir(DirModel* dir) { dir_ = dir; }

  // Design Rule Check
  void drc() override;
  // Accessors:

  // Queue arbiter:
  MQArb* arb() const { return arb_; }
  //
  LLCTTable* tt() const { return tt_; }

 private:
  // LLC -> NOC command queue (NOC owned)
  MessageQueue* llc_noc__msg_q_ = nullptr;
  // DIR -> LLC command queue (LLC owned)
  MessageQueue* dir_llc__cmd_q_ = nullptr;
  // MEM -> LLC response queue (LLC owned)
  MessageQueue* mem_llc__rsp_q_ = nullptr;
  //
  std::vector<MessageQueue*> cc_llc__rsp_qs_;
  // Queue selector arbiter
  MQArb* arb_ = nullptr;
  // Home memory controller
  MemCntrlModel* mc_ = nullptr;
  // Home directory.
  DirModel* dir_ = nullptr;
  // Transaction table.
  LLCTTable* tt_ = nullptr;
  // Request distruction process.
  RdisProcess* rdis_proc_ = nullptr;
  // NOC endpoint
  LLCNocEndpoint* noc_endpoint_ = nullptr;
  // LLC Cache configuration
  LLCModelConfig config_;
};

}  // namespace cc

#endif
