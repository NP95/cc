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

namespace cc {


const char* to_string(AceCmdOpcode opcode) {
  switch (opcode) {
#define __declare_to_string(__name)             \
    case AceCmdOpcode::__name: return #__name;
    ACE_COMMAND_OPCODES(__declare_to_string)
#undef __declare_to_string
    default: return "Invalid";
  }
}

AceCmdOpcode update_to_opcode(L2UpdateAction action) {
  switch (action) {
#define __declare_mapping(__opcode)             \
    case L2UpdateAction::Emit##__opcode:        \
      return AceCmdOpcode::__opcode; break;
    ACE_COMMAND_OPCODES(__declare_mapping)
    default: return AceCmdOpcode::Invalid;
  }
#undef DECLARE_MAPPING
}

const char* to_string(AceSnpOpcode cmd) {
  switch (cmd) {
#define __declare_to_string(__name)             \
    case AceSnpOpcode::__name: return #__name;
    ACE_SNOOP_OPCODES(__declare_to_string)
#undef __declare_to_string
  }
  return "Invalid";
}

} // namespace cc
