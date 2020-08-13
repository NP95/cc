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

#include <sstream>

#include "utility.h"

namespace cc {

const char* to_string(MessageClass cls) {
  switch (cls) {
    case MessageClass::Invalid:
      return "Invalid";
    case MessageClass::CpuCmd:
      return "CpuCmd";
    case MessageClass::CpuCmdRsp:
      return "CpuCmdRsp";
    case MessageClass::L1Cmd:
      return "L1Cmd";
    case MessageClass::L1CmdRsp:
      return "L1CmdRsp";
    case MessageClass::L2Cmd:
      return "L2Cmd";
    case MessageClass::L2CmdRsp:
      return "L2CmdRsp";
    case MessageClass::AceCmd:
      return "AceCmd";
    case MessageClass::AceCmdRsp:
      return "AceCmdRsp";
    case MessageClass::AceSnoop:
      return "AceSnoop";
    case MessageClass::AceSnoopRsp:
      return "AceSnoopRsp";
    case MessageClass::Noc:
      return "Noc";
    case MessageClass::CohSrt:
      return "CohSrt";
    case MessageClass::CohEnd:
      return "CohEnd";
    case MessageClass::CohCmd:
      return "CohCmd";
    case MessageClass::CohCmdRsp:
      return "CohCmdRsp";
    case MessageClass::CohSnp:
      return "CohSnp";
    case MessageClass::CohSnpRsp:
      return "CohSnpRsp";
    case MessageClass::LLCCmd:
      return "LLCCmd";
    case MessageClass::LLCCmdRsp:
      return "LLCCmdRsp";
    case MessageClass::LLCFwd:
      return "LLCFwd";
    case MessageClass::LLCFwdRsp:
      return "LLCFwdRsp";
    case MessageClass::MemCmd:
      return "MemCmd";
    case MessageClass::MemRsp:
      return "MemRsp";
    case MessageClass::Dt:
      return "Dt";
    case MessageClass::DtRsp:
      return "DtRsp";
    default:
      return "Invalid";
  }
}

MessageClass to_cmd_type(MessageClass cls) {
  switch (cls) {
    case MessageClass::DtRsp:
      return MessageClass::Dt;
    case MessageClass::MemRsp:
      return MessageClass::MemCmd;
    case MessageClass::LLCFwdRsp:
      return MessageClass::LLCFwd;
    case MessageClass::LLCCmdRsp:
      return MessageClass::LLCCmd;
    case MessageClass::CohSnpRsp:
      return MessageClass::CohSnp;
    case MessageClass::CohCmdRsp:
      return MessageClass::CohCmd;
    case MessageClass::CohEnd:
      return MessageClass::CohSrt;
    case MessageClass::AceSnoopRsp:
      return MessageClass::AceSnoop;
    case MessageClass::AceCmdRsp:
      return MessageClass::AceCmd;
    case MessageClass::L2CmdRsp:
      return MessageClass::L2Cmd;
    case MessageClass::L1CmdRsp:
      return MessageClass::L1Cmd;
    case MessageClass::CpuCmdRsp:
      return MessageClass::CpuCmd;
    default:
      return MessageClass::Invalid;
  }
}

Transaction::Transaction() {
  static trans_id_t tid_counter = 0;
  tid_ = tid_counter++;
}

std::string Transaction::to_string() const {
  using std::to_string;

  KVListRenderer r;
  r.add_field("tid", to_string(tid()));
  return r.to_string();
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

}  // namespace cc
