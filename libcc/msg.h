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

#ifndef CC_LIBCC_MSG_H
#define CC_LIBCC_MSG_H

#include "cc/kernel.h"
#include "cc/types.h"
#include <string>

namespace cc {

class KVListRenderer;

// Forwards:
namespace kernel {

template<typename> class Agent;
template<typename> class RequesterIntf;
}
  
//
//
using trans_id_t = std::size_t;


//
//
class Transaction {
 public:
  Transaction();
  virtual ~Transaction() = default;

  //
  virtual std::string to_string() const;


  // Accessors:
  kernel::Time start_time() const { return start_time_; }

  trans_id_t tid() const { return tid_; }
  

  // Setters:
  void set_start_time(kernel::Time time) { start_time_ = time; }

  virtual void release() const { delete this; }

 private:
  kernel::Time start_time_;
  trans_id_t tid_;
};

//
//
enum class MessageClass {
  // ------------------------------------------------------------------------ //
  // Generic place holder denoting "Invalid"/"Bad" command (should never be
  // seen during a simulation.
  Invalid,

  
  // ------------------------------------------------------------------------ //
  // Cpu Comand/Response Message:

  // CPU -> L1
  CpuCmd,

  // L1 -> CPU
  CpuCmdRsp,

  
  // ------------------------------------------------------------------------ //
  // L1 Cache Command/Response Message:

  // CPU -> L1
  // L1  -> CPU
  L1Cmd,

  // L1 -> CPU
  // L1 -> L@
  L1CmdRsp,

  
  // ------------------------------------------------------------------------ //
  // L2 Cache Command/Response Messages:

  // CC -> L2
  // L1 -> L2
  L2Cmd,

  // L2 -> CC
  // L2 -> CC
  L2CmdRsp,

  
  // ------------------------------------------------------------------------ //
  // AXI command message from L2 to the coherence subsystem.

  // L2 -> CC
  AceCmd,

  // CC -> L2
  AceCmdRspR,

  // CC -> L2
  AceCmdRspB,
  
  // ------------------------------------------------------------------------ //
  // Ace "Snoop" message encapsulating snoop requests and respones
  // from a AMBA compliant agent (the L2).

  // CC -> L2
  AceSnoop,

  // L2 -> CC
  AceSnoopRsp,

  
  // ------------------------------------------------------------------------ //
  // Network-On-Chip/Interconnect:

  // Generic transport message allowing two NOC registers agents to
  // communicate.
  Noc,

  
  // ------------------------------------------------------------------------ //
  // Deprecate?
  DirCmd,
  DirRsp,

  
  // ------------------------------------------------------------------------ //
  // Coherence Messages:

  // Coherence transactional command(s):

  // Coherence transaction demarcation messages:
  //
  // CC -> DIR
  CohSrt,

  // End a coherence action:
  //
  // DIR -> CC
  CohEnd,


  // Coherence commands 
  //
  
  // CC -> DIR
  CohCmd,

  // DIR -> CC
  CohCmdRsp,

  // Coherence "Forwarding"/"Snoop" messages:
  //

  // DIR -> CC
  CohFwd,

  // CC -> DIR
  CohFwdRsp,

  // Coherence agent line invalidation messages:
  //

  // DIR -> CC
  CohInv,

  // CC - DIR
  CohInvRsp,

  
  // ------------------------------------------------------------------------ //
  // Last-Level Cache Message Classes:

  // DIR -> LLC command {Fill, Writeback}:
  LLCCmd,

  // LLC -> DIR command response:
  LLCCmdRsp,

  // DIR -> LLC forwarding message:
  LLCFwd,

  // LLC -> DIR forwarding response message:
  LLCFwdRsp,

  
  // ------------------------------------------------------------------------ //
  // Memory Controller Message Classes:
  
  // Memory Command; agent -> memory (read/write)
  MemCmd,
  // Memory Response; memory -> agent (read data/write acknowledgement)
  MemRsp,

  
  // ------------------------------------------------------------------------ //
  // Payload Message Classes:

  // Permissible transactions:
  // MEM -> LLC
  // LLC -> MEM
  // L2 -> LLC
  // LLC -> L2
  // L2 -> L2
  // CC -> L2
  // L2 -> CC

  // Message denoting the transfer of a complete L2 cache line of data.
  Dt,
  //
  DtRsp
};

const char* to_string(MessageClass cls);


// Message unimque ID type
using msg_id_t = std::size_t;

//
//
class Message {
 public:

  Message(MessageClass cls);
  virtual ~Message() = default;

  Transaction* t() const { return t_; }
  MessageClass cls() const { return cls_; }
  kernel::Agent<const Message*>* agent() const { return origin_; }
  msg_id_t mid() const { return mid_; }

  virtual std::string to_string_short() const { return "Some message"; } // TO abstract
  virtual std::string to_string() const { return "Some message"; } // TO abstract

  void set_origin(kernel::Agent<const Message*>* origin) { origin_ = origin; }
  void set_t(Transaction* t) { t_ = t; }
  void set_cls(MessageClass cls) { cls_ = cls; }

  // Release message; return to owning message pool or destruct where
  // applicable.
  virtual void release() const { delete this; }

 protected:
  void render_msg_fields(KVListRenderer& r) const;

 private:
  // Message ID (globally unique)
  msg_id_t mid_;
  // Parent transaction;
  Transaction* t_ = nullptr;
  // Message type
  MessageClass cls_ = MessageClass::Invalid;
  // Originating agent.
  kernel::Agent<const Message*>* origin_;
};

using MsgRequesterIntf = kernel::RequesterIntf<const Message*>;

} // namespace cc

#endif
