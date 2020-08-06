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

namespace cc {

//
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
  Invalid
};

//
//
const char* to_string(AceCmdOpcode opcode);

//
//
class AceCmdMsg : public Message {
  template<typename> friend class PooledItem;
  AceCmdMsg();
 public:

  //
  std::string to_string() const override;

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
class AceCmdRspMsg : public Message {
  template<typename> friend class PooledItem;
  AceCmdRspMsg();
 public:

  //
  std::string to_string() const override;

  //
  //
  bool pd() const { return pd_; }
  bool is() const { return is_; }

  //
  //
  void set_pd(bool pd) { pd_ = pd; }
  void set_is(bool is) { is_ = is; }

 private:
  bool pd_ = false;
  bool is_ = false;
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
AceSnpOpcode to_snp_opcode(AceCmdOpcode opcode);

//
//
class AceSnpMsg : public Message {
 public:
  AceSnpMsg();

  //
  std::string to_string() const override;

  //
  AceSnpOpcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }

  //
  void set_opcode(AceSnpOpcode opcode) { opcode_ = opcode; }
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  AceSnpOpcode opcode_ = AceSnpOpcode::Invalid;
  addr_t addr_ = 0;
};

//
//
class AceSnpRspMsg : public Message {
  template<typename> friend class PooledItem;
  AceSnpRspMsg();
 public:

  //
  std::string to_string() const override;

  //
  bool dt() const { return dt_; }
  bool pd() const { return pd_; }
  bool is() const { return is_; }
  bool wu() const { return wu_; }

  //
  void set_dt(bool dt) { dt_ = dt; }
  void set_pd(bool pd) { pd_ = pd; }
  void set_is(bool is) { is_ = is; }
  void set_wu(bool wu) { wu_ = wu; }

 private:
  bool dt_ = false;
  bool pd_ = false;
  bool is_ = false;
  bool wu_ = false;
};

}  // namespace cc

#endif
