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

#include "msg.h"
#include "protocol.h"

namespace cc {

//
//
enum class AceCmdOpcode {
  ReadNoSnoop,
  ReadOnce,
  ReadClean,
  ReadSharedNotDirty,
  ReadShared,
  ReadUnique,
  CleanUnique,
  CleanShared,
  CleanInvalid,
  MakeUnique,
  MakeInvalid,
  WriteNoSnoop,
  WriteLineUnique,
  WriteBack,
  WriteClean,
  Evict,
  Invalid
};

//
//
const char* to_string(AceCmdOpcode opcode);

//
//
class AceCmdMsg : public Message {
 public:
  AceCmdMsg();

  //
  //
  AceCmdOpcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }

  //
  //
  void set_opcode(AceCmdOpcode opcode) { opcode_ = opcode; }
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  AceCmdOpcode opcode_;
  addr_t addr_;
};


//
//
class AceCmdRspRMsg : public Message {
 public:
  AceCmdRspRMsg();

  //
  //
  bool pass_dirty() const { return pass_dirty_; }
  bool is_shared() const { return is_shared_; }

  //
  //
  void set_pass_dirty(bool pass_dirty) { pass_dirty_ = pass_dirty; }
  void set_is_shared(bool is_shared) { is_shared_ = is_shared; }

 private:
  bool pass_dirty_ = false;
  bool is_shared_ = false;
};

//
//
class AceCmdRspBMsg : public Message {
 public:
  AceCmdRspBMsg();

 private:
};

//
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
  Invalid,
};

//
//
const char* to_string(AceSnpOpcode opcode);

//
//
class AceSnpMsg : public Message {
 public:
  AceSnpMsg();

  AceSnpOpcode opcode() const { return opcode_; }

  void set_opcode(AceSnpOpcode opcode) { opcode_ = opcode; }
  
 private:
  AceSnpOpcode opcode_;
};

//
//
class AceSnpRspMsg : public Message {
 public:
  AceSnpRspMsg();

  //
  bool data_transfer() const { return data_transfer_; }
  bool error() const { return error_; }
  bool pass_dirty() const { return pass_dirty_; }
  bool is_shared() const { return is_shared_; }
  bool was_unique() const { return was_unique_; }

  //
  void set_data_transfer(bool data_transfer) { data_transfer_ = data_transfer; }
  void set_error(bool error) { error_ = error; }
  void set_pass_dirty(bool pass_dirty) { pass_dirty_ = pass_dirty; }
  void set_is_shared(bool is_shared) { is_shared_ = is_shared; }
  void set_was_unique(bool was_unique) { was_unique_ = was_unique; }

 private:
  bool data_transfer_ = false;
  bool error_ = false;
  bool pass_dirty_ = false;
  bool is_shared_ = false;
  bool was_unique_ = false;
};

} // namespace cc

#endif
