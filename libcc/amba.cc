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
#include "utility.h"
#include <sstream>

namespace cc {

const char* to_string(AceCmdOpcode opcode) {
  switch (opcode) {
    case AceCmdOpcode::ReadNoSnoop: return "ReadNoSnoop";
    case AceCmdOpcode::ReadOnce: return "ReadOnce";
    case AceCmdOpcode::ReadClean: return "ReadClean";
    case AceCmdOpcode::ReadSharedNotDirty: return "ReadSharedNotDirty";
    case AceCmdOpcode::ReadShared: return "ReadShared";
    case AceCmdOpcode::ReadUnique: return "ReadUnique";
    case AceCmdOpcode::CleanUnique: return "CleanUnique";
    case AceCmdOpcode::CleanShared: return "CleanShared";
    case AceCmdOpcode::CleanInvalid: return "CleanInvalid";
    case AceCmdOpcode::MakeUnique: return "MakeUnique";
    case AceCmdOpcode::MakeInvalid: return "MakeInvalid";
    case AceCmdOpcode::WriteNoSnoop: return "WriteNoSnoop";
    case AceCmdOpcode::WriteLineUnique: return "WriteLineUnique";
    case AceCmdOpcode::WriteBack: return "WriteBack";
    case AceCmdOpcode::WriteClean: return "WriteClean";
    case AceCmdOpcode::Evict: return "Evict";
    case AceCmdOpcode::Invalid: return "Invalid";
    default: return "Invalid";
  }
}

//
//
AceCmdMsg::AceCmdMsg() : Message(MessageClass::AceCmd) {}

//
//
AceCmdRspRMsg::AceCmdRspRMsg() : Message(MessageClass::AceCmdRspR) {}

//
//
AceCmdRspBMsg::AceCmdRspBMsg() : Message(MessageClass::AceCmdRspB) {}


//
//
const char* to_string(AceSnpOpcode opcode) {
  switch (opcode) {
    case AceSnpOpcode::ReadOnce: return "ReadOnce";
    case AceSnpOpcode::ReadClean: return "ReadClean";
    case AceSnpOpcode::ReadNotSharedDirty: return "ReadNotSharedDirty";
    case AceSnpOpcode::ReadShared: return "ReadShared";
    case AceSnpOpcode::ReadUnique: return "ReadUnique";
    case AceSnpOpcode::CleanInvalid: return "CleanInvalid";
    case AceSnpOpcode::MakeInvalid: return "MakeInvalid";
    case AceSnpOpcode::CleanShared: return "CleanShared";
    case AceSnpOpcode::Invalid: return "Invalid";
    default: return "Invalid";
  }
}


//
//
AceSnpMsg::AceSnpMsg() : Message(MessageClass::AceSnoop) {}

//
//
AceSnpRspMsg::AceSnpRspMsg() : Message(MessageClass::AceSnoopRsp) {}

} // namespace cc
