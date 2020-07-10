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

AceCmdOpcode update_to_opcode(L2UpdateAction action) {
  switch (action) {
    case L2UpdateAction::EmitReadNoSnoop:
      return AceCmdOpcode::ReadNoSnoop;
      break;
    case L2UpdateAction::EmitReadOnce:
      return AceCmdOpcode::ReadOnce;
      break;
    case L2UpdateAction::EmitReadClean:
      return AceCmdOpcode::ReadClean;
      break;
    case L2UpdateAction::EmitReadSharedNotDirty:
      return AceCmdOpcode::ReadSharedNotDirty;
      break;
    case L2UpdateAction::EmitReadShared:
      return AceCmdOpcode::ReadShared;
      break;
    case L2UpdateAction::EmitReadUnique:
      return AceCmdOpcode::ReadUnique;
      break;
    case L2UpdateAction::EmitCleanUnique:
      return AceCmdOpcode::CleanUnique;
      break;
    case L2UpdateAction::EmitCleanShared:
      return AceCmdOpcode::CleanShared;
      break;
    case L2UpdateAction::EmitCleanInvalid:
      return AceCmdOpcode::CleanInvalid;
      break;
    case L2UpdateAction::EmitMakeUnique:
      return AceCmdOpcode::MakeUnique;
      break;
    case L2UpdateAction::EmitMakeInvalid:
      return AceCmdOpcode::MakeInvalid;
      break;
    case L2UpdateAction::EmitWriteNoSnoop:
      return AceCmdOpcode::WriteNoSnoop;
      break;
    case L2UpdateAction::EmitWriteLineUnique:
      return AceCmdOpcode::WriteLineUnique;
      break;
    case L2UpdateAction::EmitWriteBack:
      return AceCmdOpcode::WriteBack;
      break;
    case L2UpdateAction::EmitWriteClean:
      return AceCmdOpcode::WriteClean;
      break;
    case L2UpdateAction::EmitEvict:
      return AceCmdOpcode::Evict;
      break;
    default:
      return AceCmdOpcode::Invalid;
      break;
  }
}

} // namespace cc
