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

#ifndef CC_SRC_AMBA_H
#define CC_SRC_AMBA_H

#include "msg.h"

namespace cc {

// Command opcode enumeration
//
enum class AceCmdOpcode {
  ReadNoSnoop,
  ReadOnce,
  ReadClean,
  ReadNotSharedDirty,
  ReadShared,
  ReadUnique,
  CleanUnique,
  CleanShared,
  CleanInvalid,
  MakeUnique,
  MakeInvalid,
  WriteNoSnoop,
  WriteUnique,
  WriteLineUnique,
  WriteBack,
  WriteClean,
  Evict,

  // Recall command (not a ACE opcode, but piggybacked to avoid the
  // need to create a protocol specific opcode definition).
  Recall,

  // Invalid; placeholder
  Invalid
};

//
//
const char* to_string(AceCmdOpcode opcode);

// Ace Command which encapsulates the issue of a message from the
// initiators AR/AW channels.
//
class AceCmdMsg : public Message {
  template <typename>
  friend class PooledItem;

  AceCmdMsg();
 public:

  // Convert to a human readable string
  std::string to_string() const override;

  // Accessors:

  // Current command opcode
  AceCmdOpcode opcode() const { return opcode_; }

  // Current comamnd address
  addr_t addr() const { return addr_; }


  // Setters:

  // Set current opcode
  void set_opcode(AceCmdOpcode opcode) { opcode_ = opcode; }

  // Set current address
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  // Opcode
  AceCmdOpcode opcode_;

  // Address
  addr_t addr_;
};


// Ace Response message which encapsulates the response message from
// the interconnect back into the initiating agent. Message
// corresponds to a B/R channel transaction.
//
class AceCmdRspMsg : public Message {
  template <typename>
  friend class PooledItem;
  AceCmdRspMsg();

 public:

  // Convert to a human readable string
  std::string to_string() const override;

  // Accessors:
  
  // Pass-Dirty flag
  bool pd() const { return pd_; }

  // Is-Shared flag
  bool is() const { return is_; }


  // Accessors:

  // Set Pass-Dirty flag
  void set_pd(bool pd) { pd_ = pd; }

  // Set Is-Shared falg
  void set_is(bool is) { is_ = is; }

 private:
  // PassDirty
  bool pd_ = false;

  // IsShared
  bool is_ = false;
};

// Snoop opcode enumeration
//
enum class AceSnpOpcode {
  ReadOnce,
  ReadClean,
  ReadNotSharedDirty,
  ReadShared,
  ReadUnique,
  CleanInvalid,
  MakeInvalid,
  CleanShared,
  // Invalid; place-holder
  Invalid,
};


// Convert opcode to humand readable string
const char* to_string(AceSnpOpcode opcode);

// Convert command opcode to corresponding (recommanded) snoop opcode.
AceSnpOpcode to_snp_opcode(AceCmdOpcode opcode);

// Ace Snoop message which encapsulates a response into some cache
// coherent agent. Message corresponds to a transaction on the AC
// channel.
//
class AceSnpMsg : public Message {
  template <typename>
  friend class PooledItem;

  AceSnpMsg();
 public:

  // Convert to a humand readable string
  std::string to_string() const override;

  // Accessors:
  
  // Snoop command opcode
  AceSnpOpcode opcode() const { return opcode_; }

  // Snoop command address
  addr_t addr() const { return addr_; }


  // Setters:
  
  // Set command opcode
  void set_opcode(AceSnpOpcode opcode) { opcode_ = opcode; }

  // Set command address
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  // Snoop opcode
  AceSnpOpcode opcode_ = AceSnpOpcode::Invalid;

  // Address
  addr_t addr_ = 0;
};


// Ace snoop response message which encapsulates the response to the
// interconnect after some prior snoop command. Message corresponds to
// a transaction of the CR/CD channels.
//
class AceSnpRspMsg : public Message {
  template <typename>
  friend class PooledItem;
  AceSnpRspMsg();

 public:

  // Convert to a humand readable string.
  std::string to_string() const override;

  // Accessors:

  // Data Transfer flag
  bool dt() const { return dt_; }

  // Pass Dirty flag
  bool pd() const { return pd_; }

  // Is Shared flag
  bool is() const { return is_; }

  // Was Unique flag
  bool wu() const { return wu_; }


  // Setters:

  // Set Data Transfer flag
  void set_dt(bool dt) { dt_ = dt; }

  // Set Pass Dirty flag
  void set_pd(bool pd) { pd_ = pd; }

  // Set Is Shared flag
  void set_is(bool is) { is_ = is; }

  // Set Was Unique flag
  void set_wu(bool wu) { wu_ = wu; }

 private:
  // Data Transfer flag
  bool dt_ = false;

  // Pass Dirty flag
  bool pd_ = false;

  // Is Shared flag
  bool is_ = false;

  // Was Unique flag
  bool wu_ = false;
};

}  // namespace cc

#endif
