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

//#include "moesi.h"

#include <string>

#include "l1cache.h"
#include "utility.h"
#include "msg.h"
#include "protocol.h"
#include "cpu.h"
#include "amba.h"
#include "moesi_gen.h"

namespace cc {

//
//
class MOESIL1LineState : public L1LineState {
 public:
  MOESIL1LineState() {}

  // Current line state.
  MOESIL1State state() const { return state_; }
  void set_state(MOESIL1State state) { state_ = state; }

  // Stable state status.
  bool is_stable() const {
    using cc::is_stable;
    return is_stable(state());
  }

 private:
  MOESIL1State state_ = MOESIL1State::I;
};

//
//
class MOESIL1CacheProtocol : public L1CacheModelProtocol {
 public:
  MOESIL1CacheProtocol() {}

  L1LineState* construct_line() const override {
    MOESIL1LineState* l1 = new MOESIL1LineState();
    return l1;
  }

  void apply(L1CacheModelApplyResult& r, L1LineState* line,
             const CpuL1__CmdMsg* msg) const override {
    const MOESIL1LineState* mline = static_cast<const MOESIL1LineState*>(line);
    switch (msg->opcode()) {
      case CpuOpcode::Load: {
        switch (mline->state()) {
          case MOESIL1State::I: {
            r.push(L1UpdateAction::EmitGetS);
            r.push(L1UpdateAction::UpdateState);
            r.state(ut(MOESIL1State::I_S));
            r.push(L1UpdateAction::Block);
          } break;
          case MOESIL1State::E:
          case MOESIL1State::M:
          case MOESIL1State::S: {
            // Load instruction hits in the cache, therefore commit
            // and emit response back to the cache.
            r.push(L1UpdateAction::EmitCpuRsp);
            r.push(L1UpdateAction::Commit);
          } break;
          default: {
            // Otherwise,
            r.push(L1UpdateAction::Block);
          } break;
        }
      } break;
      case CpuOpcode::Store: {
        switch (mline->state()) {
          case MOESIL1State::I: {
            r.push(L1UpdateAction::UpdateState);
            r.state(ut(MOESIL1State::I_E));
            r.push(L1UpdateAction::Block);
          } break;
          case MOESIL1State::E: {
            r.state(ut(MOESIL1State::M));
            r.push(L1UpdateAction::Commit);
          } break;
          case MOESIL1State::M: {
            r.push(L1UpdateAction::Commit);
          } break;
          case MOESIL1State::S: {
            r.state(ut(MOESIL1State::S_E));
            r.push(L1UpdateAction::Block);
          } break;
          default: {
          } break;
        }
      } break;
      default: {
        // Invalid command.
      } break;
    }
  }

  void update_line_state(L1LineState* line, state_t state) const override {
    MOESIL1LineState* mline = static_cast<MOESIL1LineState*>(line);
    mline->set_state(static_cast<MOESIL1State>(state));
  }
};


//
//
class MOESIL2LineState : public L2LineState {
 public:
  MOESIL2LineState() {}

  // Current line state.
  MOESIL2State state() const { return state_; }
  void set_state(MOESIL2State state) { state_ = state; }

  // Stable state status.
  bool is_stable() const {
    using cc::is_stable;
    return is_stable(state());
  }

 private:
  MOESIL2State state_ = MOESIL2State::I;
};

class MOESIL2CacheProtocol : public L2CacheModelProtocol {
 public:
  MOESIL2CacheProtocol() {}


  //
  L2LineState* construct_line() const override {
    MOESIL2LineState* l = new MOESIL2LineState;
    l->set_state(MOESIL2State::I);
    return l;
  }

  //
  void apply(L2CacheModelApplyResult& r, L2LineState* line,
             const L1L2__CmdMsg* msg) const override {
    const MOESIL2LineState* mline = static_cast<MOESIL2LineState*>(line);
    switch (msg->opcode()) {
      case L2L1Opcode::GetS: {
        handle_gets(r, mline);
      } break;
    }
  }

  //
  void commit(const L2CacheModelApplyResult& r,
              L2LineState* state) const override {
  }

  //
  void update_line_state(L2LineState* line, state_t state) const override {
    MOESIL2LineState* mline = static_cast<MOESIL2LineState*>(line);
    mline->set_state(static_cast<MOESIL2State>(state));
  }

 private:

  void handle_gets(L2CacheModelApplyResult& r, const MOESIL2LineState* line) const {
    switch (line->state()){
      case MOESIL2State::I: {
        r.push(L2UpdateAction::UpdateState);
        r.state(ut(MOESIL2State::I_S));
        r.push(L2UpdateAction::EmitReadShared);
        r.push(L2UpdateAction::Block);
      } break;
      default: {
      } break;
    }
  }
};

class MOESIDirectoryProtocol : public DirectoryProtocol {
 public:
  MOESIDirectoryProtocol() {}
};


class MOESICacheControllerLineState : public CacheControllerLineState {
 public:
  MOESICacheControllerLineState() = default;
};

class MOESICacheControllerProtocol : public CacheControllerProtocol {
 public:
  MOESICacheControllerProtocol() {}

  CacheControllerLineState* construct_line() const override {
    CacheControllerLineState* line = new MOESICacheControllerLineState;
    return line;
  }

  void apply(CCModelApplyResult& r, CacheControllerLineState* line,
             const AceCmdMsg* msg) const override {
    switch (msg->opcode()) {
      case AceCmdOpcode::ReadNoSnoop: {
        // TODO
      } break;
      case AceCmdOpcode::ReadOnce: {
        // TODO
      } break;
      case AceCmdOpcode::ReadClean: {
        // TODO
      } break;
      case AceCmdOpcode::ReadSharedNotDirty: {
        // TODO
      } break;
      case AceCmdOpcode::ReadShared: {
        // TODO
        r.push(CCUpdateAction::EmitAceCmdToDir);
        r.push(CCUpdateAction::UpdateState);
        r.state(ut(MOESICCState::I_S));
      } break;
      case AceCmdOpcode::ReadUnique: {
        // TODO
      } break;
      case AceCmdOpcode::CleanUnique: {
        // TODO
      } break;
      case AceCmdOpcode::CleanShared: {
        // TODO
      } break;
      case AceCmdOpcode::CleanInvalid: {
        // TODO
      } break;
      case AceCmdOpcode::MakeUnique: {
        // TODO
      } break;
      case AceCmdOpcode::MakeInvalid: {
        // TODO
      } break;
      case AceCmdOpcode::WriteNoSnoop: {
        // TODO
      } break;
      case AceCmdOpcode::WriteLineUnique: {
        // TODO
      } break;
      case AceCmdOpcode::WriteBack: {
        // TODO
      } break;
      case AceCmdOpcode::WriteClean: {
        // TODO
      } break;
      case AceCmdOpcode::Evict: {
        // TODO
      } break;
      default: {
        // TODO
      } break;
    }
  }

  void update_line_state(
      CacheControllerLineState* line, state_t state) const override {
    //MOESICacheControllerLineState* mline =
    //static_cast<MOESICacheControllerLineState*>(line);
    //mline->set_state(static_cast<MOESIL1State>(state));
  }
};

class MOESIProtocolBuilder : public ProtocolBuilder {
 public:
  // Create an instance of the L1 protocol
  L1CacheModelProtocol* create_l1() override {
    return new MOESIL1CacheProtocol{};
  }

  // Create an instance of the L2 protocol
  L2CacheModelProtocol* create_l2() override {
    return new MOESIL2CacheProtocol{};
  }

  // Create and instance of the Directory protocol
  DirectoryProtocol* create_dir() override {
    return new MOESIDirectoryProtocol{};
  }

  CacheControllerProtocol* create_cc() override {
    return new MOESICacheControllerProtocol{};
  }
};

CC_DECLARE_PROTOCOL_BUILDER("moesi", MOESIProtocolBuilder);

}  // namespace cc
