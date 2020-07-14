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

#include "msg.h"

namespace cc {

const char* to_string(MessageClass cls) {
  switch (cls) {
    case MessageClass::Invalid: return "Invalid";
    case MessageClass::CpuCmd: return "CpuCmd";
    case MessageClass::CpuRsp: return "CpuRsp";
    case MessageClass::L1Cmd: return "L1Cmd";
    case MessageClass::L1Rsp: return "L1Rsp";
    case MessageClass::AceCmd: return "AceCmd";
    case MessageClass::AceCmdRsp: return "AceCmdRsp";
    case MessageClass::AceSnoop: return "AceSnoop";
    case MessageClass::AceSnoopRsp: return "AceSnoopRsp";
    case MessageClass::Noc: return "Noc";
    case MessageClass::DirCmd: return "DirCmd";
    case MessageClass::DirRsp: return "DirRsp";
    case MessageClass::LLCCmd: return "LLCCmd";
    case MessageClass::LLCRsp: return "LLCRsp";
    case MessageClass::MemCmd: return "MemCmd";
    case MessageClass::MemRsp: return "MemRsp";
    case MessageClass::DataLine: return "DataLine";
    default: return "Invalid";
  }
}


} // namespace cc
