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

#ifndef CC_LIBCC_AMBA_H
#define CC_LIBCC_AMBA_H

#include "cc/msg.h"
#include "protocol.h"
#include "amba_enum.h"

namespace cc {

AceCmdOpcode update_to_opcode(L2UpdateAction action);

//
//
class AceCmdMsg : public Message {
 public:
  AceCmdMsg(Transaction* t) : Message(t, MessageClass::AceCmd) {}

  // Tracing methods:
  std::string to_string_short() const override;
  std::string to_string() const override;

  // Opcode:
  AceCmdOpcode opcode() const { return opcode_; }
  void opcode(AceCmdOpcode opcode) { opcode_ = opcode; }

  // Address:
  addr_t addr() const { return addr_; }
  void set_addr(addr_t addr) { addr_ = addr; }
  
 private:
  addr_t addr_;
  AceCmdOpcode opcode_;
};


//
//
class AceCmdRspMsg : public Message {
 public:
  AceCmdRspMsg(Transaction* t) : Message(t, MessageClass::AceCmd) {}

  bool pass_dirty() const { return pass_dirty_; }
  bool is_shared() const { return is_shared_; }
  
 private:
  // Pass Dirty flag
  bool pass_dirty_ = false;
  // Is Shared flag
  bool is_shared_ = false;
};

//
//
class AceSnpMsg : public Message {
 public:
  AceSnpMsg(Transaction* t) : Message(t, MessageClass::AceSnoop) {}

  AceSnpOpcode opcode() const { return opcode_; }
 private:
  AceSnpOpcode opcode_;
};

//
//
class AceSnpRspMsg : public Message {
 public:
  AceSnpRspMsg(Transaction* t) : Message(t, MessageClass::AceSnoop) {}

  // Observers:
  bool data_transfer() const { return data_transfer_; }
  bool error() const { return error_; }
  bool pass_dirty() const { return pass_dirty_; }
  bool is_shared() const { return is_shared_; }
  bool was_unique() const { return was_unique_; }
  
 private:
  // Data transfer flag
  bool data_transfer_ = false;
  // Error flag;
  bool error_ = false;
  // Pass dirty flag
  bool pass_dirty_ = false;
  // Is Shared flag
  bool is_shared_= false;
  // Was Unique flag
  bool was_unique_ = false;
};

} // namespace cc

#endif
