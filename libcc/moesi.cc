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

#include "cpu_msg.h"
#include "l1cache.h"
#include "utility.h"
#include "cc/protocol.h"

namespace cc {

// clang-format off  
#define DECLARE_L1_STATES(__func)		\
  __func(I, true)				\
  __func(I_S, false)				\
  __func(S, true)				\
  __func(I_E, false)				\
  __func(S_E, false)				\
  __func(E, true)				\
  __func(E_M, false)				\
  __func(M, true)				\
  __func(M_I, false)
// clang-format on

#define __declare_states(__state, __stable) __state,
enum class L1State : state_t { DECLARE_L1_STATES(__declare_states) };
#undef __declare_states

std::string to_string(L1State s) {
#define __declare_to_string(__state, __stable) \
  case L1State::__state:                       \
    return #__state;
  switch (s) {
    DECLARE_L1_STATES(__declare_to_string)
    default:
      return "Bad State";
  }
#undef __declare_to_string
}

bool is_stable(L1State s) {
#define __declare_is_stable(__state, __stable) \
  case L1State::__state:                       \
    return __stable;
  switch (s) {
    DECLARE_L1_STATES(__declare_is_stable)
    default:
      return false;
  }
#undef __declare_is_stable
}
#undef DECLARE_L1_STATES

//
//
class MOESIL1LineState : public L1LineState {
 public:
  MOESIL1LineState() {}

  // Current line state.
  L1State state() const { return state_; }
  void set_state(L1State state) { state_ = state; }

  // Stable state status.
  bool is_stable() const {
    using cc::is_stable;
    return is_stable(state());
  }

 private:
  L1State state_ = L1State::I;
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
             const CpuCommandMessage* msg) const override {
    const MOESIL1LineState* mline = static_cast<const MOESIL1LineState*>(line);
    std::vector<L1UpdateAction>& actions = r.actions();
    switch (msg->opcode()) {
      case CpuCommandMessage::Load: {
        switch (mline->state()) {
          case L1State::I: {
            actions.push_back(L1UpdateAction::EmitGetS);
            r.set_state(ut(L1State::I_S));
            r.set_status(L1UpdateStatus::IsBlocked);
          } break;
          case L1State::E:
          case L1State::M:
          case L1State::S: {
            // Load instruction hits in the cache, therefore commit
            // and emit response back to the cache.
            actions.push_back(L1UpdateAction::EmitCpuRsp);
            r.set_status(L1UpdateStatus::CanCommit);
          } break;
          default: {
            // Otherwise,
            r.set_status(L1UpdateStatus::IsBlocked);
          } break;
        }
      } break;
      case CpuCommandMessage::Store: {
        switch (mline->state()) {
          case L1State::I: {
            r.set_state(ut(L1State::I_E));
            r.set_status(L1UpdateStatus::IsBlocked);
          } break;
          case L1State::E: {
            r.set_state(ut(L1State::M));
            r.set_status(L1UpdateStatus::CanCommit);
          } break;
          case L1State::M: {
            r.set_status(L1UpdateStatus::CanCommit);
          } break;
          case L1State::S: {
            r.set_state(ut(L1State::S_E));
            r.set_status(L1UpdateStatus::IsBlocked);
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

  void commit(const L1CacheModelApplyResult& r,
              L1LineState* line) const override {
    MOESIL1LineState* mline = static_cast<MOESIL1LineState*>(line);
    if (ut(mline->state()) != r.state()) {
      // Apply requested state transition:
      mline->set_state(static_cast<L1State>(r.state()));
    }
  }
};

class MOESIL2CacheProtocol : public L2CacheModelProtocol {
 public:
  MOESIL2CacheProtocol() {}
};

class MOESIDirectoryProtocol : public DirectoryProtocol {
 public:
  MOESIDirectoryProtocol() {}
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
};

CC_DECLARE_PROTOCOL_BUILDER("moesi", MOESIProtocolBuilder);

}  // namespace cc
