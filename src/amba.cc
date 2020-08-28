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

#include "amba.h"

#include <sstream>

#include "utility.h"

namespace cc {

const char* to_string(AceCmdOpcode opcode) {
  switch (opcode) {
    case AceCmdOpcode::ReadNoSnoop:
      return "ReadNoSnoop";
    case AceCmdOpcode::ReadOnce:
      return "ReadOnce";
    case AceCmdOpcode::ReadClean:
      return "ReadClean";
    case AceCmdOpcode::ReadNotSharedDirty:
      return "ReadNotSharedDirty";
    case AceCmdOpcode::ReadShared:
      return "ReadShared";
    case AceCmdOpcode::ReadUnique:
      return "ReadUnique";
    case AceCmdOpcode::CleanUnique:
      return "CleanUnique";
    case AceCmdOpcode::CleanShared:
      return "CleanShared";
    case AceCmdOpcode::CleanInvalid:
      return "CleanInvalid";
    case AceCmdOpcode::MakeUnique:
      return "MakeUnique";
    case AceCmdOpcode::MakeInvalid:
      return "MakeInvalid";
    case AceCmdOpcode::WriteNoSnoop:
      return "WriteNoSnoop";
    case AceCmdOpcode::WriteUnique:
      return "WriteUnique";
    case AceCmdOpcode::WriteLineUnique:
      return "WriteLineUnique";
    case AceCmdOpcode::WriteBack:
      return "WriteBack";
    case AceCmdOpcode::WriteClean:
      return "WriteClean";
    case AceCmdOpcode::Evict:
      return "Evict";
    case AceCmdOpcode::Recall:
      return "Recall";
    case AceCmdOpcode::Invalid:
      return "Invalid";
    default:
      return "Invalid";
  }
}

//
//
AceCmdMsg::AceCmdMsg() : Message(MessageClass::AceCmd) {}

std::string AceCmdMsg::to_string() const {
  using cc::to_string;

  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("opcode", to_string(opcode()));
  Hexer h;
  r.add_field("addr", h.to_hex(addr()));
  return r.to_string();
}

//
//
AceCmdRspMsg::AceCmdRspMsg() : Message(MessageClass::AceCmdRsp) {}

//
//
std::string AceCmdRspMsg::to_string() const {
  using cc::to_string;

  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("is", to_string(is()));
  r.add_field("pd", to_string(pd()));
  return r.to_string();
}

//
//
const char* to_string(AceSnpOpcode opcode) {
  switch (opcode) {
    case AceSnpOpcode::ReadOnce:
      return "ReadOnce";
    case AceSnpOpcode::ReadClean:
      return "ReadClean";
    case AceSnpOpcode::ReadNotSharedDirty:
      return "ReadNotSharedDirty";
    case AceSnpOpcode::ReadShared:
      return "ReadShared";
    case AceSnpOpcode::ReadUnique:
      return "ReadUnique";
    case AceSnpOpcode::CleanInvalid:
      return "CleanInvalid";
    case AceSnpOpcode::MakeInvalid:
      return "MakeInvalid";
    case AceSnpOpcode::CleanShared:
      return "CleanShared";
    case AceSnpOpcode::Invalid:
      return "Invalid";
    default:
      return "Invalid";
  }
}

//
//
AceSnpOpcode to_snp_opcode(AceCmdOpcode opcode) {
  // AMBA reference C5-1
  switch (opcode) {
    case AceCmdOpcode::ReadOnce:
      return AceSnpOpcode::ReadOnce;
    case AceCmdOpcode::ReadClean:
      return AceSnpOpcode::ReadClean;
    case AceCmdOpcode::ReadNotSharedDirty:
      return AceSnpOpcode::ReadNotSharedDirty;
    case AceCmdOpcode::ReadShared:
      return AceSnpOpcode::ReadShared;
    case AceCmdOpcode::ReadUnique:
      return AceSnpOpcode::ReadUnique;
    case AceCmdOpcode::CleanUnique:
      return AceSnpOpcode::CleanInvalid;
    case AceCmdOpcode::CleanShared:
      return AceSnpOpcode::CleanShared;
    case AceCmdOpcode::CleanInvalid:
      return AceSnpOpcode::CleanInvalid;
    case AceCmdOpcode::MakeUnique:
      return AceSnpOpcode::MakeInvalid;
    case AceCmdOpcode::MakeInvalid:
      return AceSnpOpcode::MakeInvalid;
    case AceCmdOpcode::WriteUnique:
      return AceSnpOpcode::CleanInvalid;
    case AceCmdOpcode::WriteLineUnique:
      return AceSnpOpcode::MakeInvalid;
    default:
      // Either original command is "Not Snooped", or bad opcode.
      return AceSnpOpcode::Invalid;
  }
}

//
//
AceSnpMsg::AceSnpMsg() : Message(MessageClass::AceSnoop) {}

//
//
std::string AceSnpMsg::to_string() const {
  using cc::to_string;

  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("opcode", to_string(opcode()));
  Hexer h;
  r.add_field("addr", h.to_hex(addr()));
  return r.to_string();
}

//
//
AceSnpRspMsg::AceSnpRspMsg() : Message(MessageClass::AceSnoopRsp) {}

//
//
std::string AceSnpRspMsg::to_string() const {
  using cc::to_string;

  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("dt", to_string(dt()));
  r.add_field("pd", to_string(pd()));
  r.add_field("is", to_string(is()));
  r.add_field("wu", to_string(wu()));
  return r.to_string();
}

}  // namespace cc
