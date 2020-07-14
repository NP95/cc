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
#include "utility.h"
#include <sstream>

namespace cc {

const char* to_string(MessageClass cls) {
  switch (cls) {
    case MessageClass::Invalid: return "Invalid";
    case MessageClass::CpuCmd: return "CpuCmd";
    case MessageClass::CpuRsp: return "CpuRsp";
    case MessageClass::L1Cmd: return "L1Cmd";
    case MessageClass::L1Rsp: return "L1Rsp";
    case MessageClass::L2Cmd: return "L2Cmd";
    case MessageClass::L2Rsp: return "L2Rsp";
    case MessageClass::AceCmd: return "AceCmd";
    case MessageClass::AceCmdRspR: return "AceCmdRspR";
    case MessageClass::AceCmdRspB: return "AceCmdRspB";
    case MessageClass::AceSnoop: return "AceSnoop";
    case MessageClass::AceSnoopRsp: return "AceSnoopRsp";
    case MessageClass::Noc: return "Noc";
    case MessageClass::DirCmd: return "DirCmd";
    case MessageClass::DirRsp: return "DirRsp";
    case MessageClass::LLCCmd: return "LLCCmd";
    case MessageClass::LLCCmdRsp: return "LLCCmdRsp";
    case MessageClass::MemCmd: return "MemCmd";
    case MessageClass::MemRsp: return "MemRsp";
    case MessageClass::DataLine: return "DataLine";
    default: return "Invalid";
  }
}

Transaction::Transaction() {
  static trans_id_t tid_counter = 0;
  tid_ = tid_counter++;
}

std::string Transaction::to_string() const {
  using std::to_string;
  
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    r.add_field("tid", to_string(tid()));
  }
  return ss.str();
}

Message::Message(MessageClass cls) : cls_(cls) {
  static msg_id_t mid_counter = 0;
  mid_ = mid_counter++;
}


void Message::render_msg_fields(KVListRenderer& r) const {
  using std::to_string;
  
  Hexer h;
  r.add_field("mid", to_string(mid()));
  r.add_field("cls", to_string(cls()));
  if (t() == nullptr) {
    r.add_field("tid", "Invalid");
  } else {
    r.add_field("tid", to_string(t()->tid()));
  }
}


} // namespace cc
