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
#include "cc/protocol.h"

namespace cc {

// clang-format off
#define ACE_COMMAND_OPCODES(__func)             \
  __func(ReadNoSnoop)                           \
  __func(ReadOnce)                              \
  __func(ReadClean)                             \
  __func(ReadSharedNotDirty)                    \
  __func(ReadShared)                            \
  __func(ReadUnique)                            \
  __func(CleanUnique)                           \
  __func(CleanShared)                           \
  __func(CleanInvalid)                          \
  __func(MakeUnique)                            \
  __func(MakeInvalid)                           \
  __func(WriteNoSnoop)                          \
  __func(WriteLineUnique)                       \
  __func(WriteBack)                             \
  __func(WriteClean)                            \
  __func(Evict)
// clang-format on

enum class AceCmdOpcode {
#define __declare_enum(__name) __name,
ACE_COMMAND_OPCODES(__declare_enum)
#undef __declare_enum
  Invalid
};

const char* to_string(AceCmdOpcode opcode);
AceCmdOpcode update_to_opcode(L2UpdateAction action);

//
//
class AceCmdMsg : public Message {
 public:
  AceCmdMsg(Transaction* t) : Message(t, MessageClass::AceCmd) {}

  AceCmdOpcode opcode() const { return opcode_; }
  void opcode(AceCmdOpcode opcode) { opcode_ = opcode; }
  
 private:
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

#define ACE_SNOOP_OPCODES(__func)               \
  __func(ReadOnce)                              \
  __func(ReadClean)                             \
  __func(ReadNotSharedDirty)                    \
  __func(ReadShared)                            \
  __func(ReadUnique)                            \
  __func(CleanInvalid)                          \
  __func(MakeInvalid)                           \
  __func(CleanShared)

enum class AceSnpOpcode {
#define __declare_enum(__name) __name,
  ACE_SNOOP_OPCODES(__declare_enum)
#undef __declare_enum
};

const char* to_string(AceSnpOpcode cmd);

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
