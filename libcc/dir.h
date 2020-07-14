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

#ifndef CC_LIBCC_DIR_H
#define CC_LIBCC_DIR_H

#include "kernel.h"
#include "cfgs.h"
#include "types.h"
#include "sim.h"
#include "msg.h"
#include "cache.h"
#include "amba.h"

namespace cc {

// Forwards
class LLCModel;
class DirLineState;
class MessageQueue;

//
//
class DirCmdMsg : public Message {
 public:
  DirCmdMsg();

  //
  std::string to_string() const override;

  //
  AceCmdOpcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }
  Agent* origin() const { return origin_; }

  //
  void set_opcode(AceCmdOpcode opcode) { opcode_ = opcode; }
  void set_addr(addr_t addr) { addr_ = addr; }
  void set_origin(Agent* origin) { origin_ = origin; }

 private:
  AceCmdOpcode opcode_;
  addr_t addr_;
  Agent* origin_;
};

//
//
class DirCmdRspMsg : public Message {
 public:
  DirCmdRspMsg();
};

//
//
class DirectoryModel : public Agent {
  friend class SocTop;

  class RdisProcess;
  class NocIngressProcess;
 public:
  DirectoryModel(kernel::Kernel* k, const DirectoryModelConfig& config);
  ~DirectoryModel();

  const DirectoryModelConfig& config() const { return config_; }

  // accessors:
  // LLC owned by current directory
  LLCModel* llc() const { return llc_; }
  // noc -> dir message queue
  MessageQueue* noc_dir__msg_q() const { return noc_dir__msg_q_; }
  // dir -> noc message queue
  MessageQueue* dir_noc__msg_q() const { return dir_noc__msg_q_; }
  // coherence protocol
  DirectoryProtocol* protocol() const { return protocol_; }

 protected:
  // build
  void build();
  //
  void set_llc(LLCModel* llc) { llc_ = llc; }
  // elaboration
  void elab() override;
  //
  void set_dir_noc__msg_q(MessageQueue* mq) { dir_noc__msg_q_ = mq; }
  
  // design rule check (drc)
  void drc() override;

  // accessor(s)

  // directory arbiter instance.
  MessageQueueArbiter* arb() const { return arb_; }
  // point to module cache instance.
  CacheModel<DirLineState*>* cache() const { return cache_; }


  // lookup rdis process message queue for traffic class.
  MessageQueue* lookup_rdis_mq(MessageClass cls) const;
  
 private:
  // queue selection arbiter
  MessageQueueArbiter* arb_ = nullptr;
  // NOC -> DIR message queue (owned by directory)
  MessageQueue* noc_dir__msg_q_ = nullptr;
  // DIR -> NOC message queue (owned by NOC)
  MessageQueue* dir_noc__msg_q_ = nullptr;
  // CPU -> DIR command queue (owned by DIR)
  MessageQueue* cpu_dir__cmd_q_ = nullptr;
  // LLC -> DIR response queue (owned by DIR)
  MessageQueue* llc_dir__rsp_q_ = nullptr;
  // NOC ingress process
  NocIngressProcess* noci_proc_ = nullptr;
  // request dispatcher process
  RdisProcess* rdis_proc_ = nullptr;
  // Last Level Cache instance (where applicable).
  LLCModel* llc_ = nullptr;
  // Cache Instance
  CacheModel<DirLineState*>* cache_ = nullptr;
  // Coherence protocol
  DirectoryProtocol* protocol_ = nullptr;
  // Current directory configuration.
  DirectoryModelConfig config_;
};

//
//
class DirectoryMapper {
 public:
  DirectoryMapper() = default;
  virtual ~DirectoryMapper() = default;

  virtual DirectoryModel* lookup(addr_t addr) const = 0;
};

//
//
class SingleDirectoryMapper : public DirectoryMapper {
 public:
  SingleDirectoryMapper(DirectoryModel* dm)
      : dm_(dm) {}

  DirectoryModel* lookup(addr_t addr) const override {
    return dm_;
  }

 private:
  // Directory end-point definition.
  DirectoryModel* dm_ = nullptr;
};

} // namespace cc

#endif
