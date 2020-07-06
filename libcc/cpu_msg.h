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

#ifndef CC_LIBCC_CPU_MSG_H
#define CC_LIBCC_CPU_MSG_H

#include "cc/cpu.h"
#include "protocol.h"


namespace cc {


//
//
class CpuCommandMessage : public Message {
 public:
  enum Opcode { Load, Store };
  
  CpuCommandMessage(Transaction* t) : Message(t, CpuCmd) {}

  Opcode opcode() const { return opcode_; }
  addr_t addr() const { return addr_; }

  void set_addr(addr_t addr) { addr_ = addr; }
  void set_opcode(Opcode opcode) { opcode_ = opcode; }
 private:
  addr_t addr_;
  Opcode opcode_;
};


//
//
class CpuResponseMessage : public Message {
 public:
  enum Opcode { Load, Store };

  CpuResponseMessage(Transaction* t) : Message(t, CpuRsp) {}

  Opcode opcode() const { return opcode_; }

  void set_opcode(Opcode opcode) { opcode_ = opcode; }
 private:
  Opcode opcode_;
};


} // namespace cc

#endif
