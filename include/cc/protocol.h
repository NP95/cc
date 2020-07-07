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

#ifndef CC_INCLUDE_CC_PROTOCOL_H
#define CC_INCLUDE_CC_PROTOCOL_H

#include <map>
#include <string>
#include <vector>
#include <deque>
#include "types.h"

namespace cc {

// Message Forwards:
class CpuCommandMessage;
class L1L2Message;

namespace kernel {

// Forwards
template <typename T>
class Agent;
template <typename T>
class RequesterIntf;
template <typename T>
class EndPointIntf;
}  // namespace kernel

//
//
class L1LineState {
 public:
  L1LineState() {}
  virtual ~L1LineState() = default;

  // Flag indiciating if the line is currently residing in a stable
  // state.
  virtual bool is_stable() const = 0;

  // Flag indiciating if the line is currently evictable (not in a
  // transient state).
  virtual bool is_evictable() const { return is_stable(); }
};

//
enum class L1UpdateStatus { CanCommit, IsBlocked };

//
enum class L1UpdateAction {
  EmitCpuRsp,
  EmitGetS,
  EmitGetE,
  UnblockCmdReqQueue
};

//
//
class L1CacheModelApplyResult {
 public:
  L1CacheModelApplyResult() = default;

  // Penalty cycles incurred on
  std::size_t penalty_cycles_n() const { return penalty_cycles_n_; }
  void penalty_cycles_n(std::size_t n) { penalty_cycles_n_ = n; }

  // Action result (issue or blocked).
  L1UpdateStatus status() const { return status_; }
  void set_status(L1UpdateStatus status) { status_ = status; }

  // Final state of line upon commitment of result.
  state_t state() const { return state_; }
  void set_state(state_t state) { state_ = state; }

  // Actions to be completed by hosting L1 cache.
  std::vector<L1UpdateAction>& actions() { return actions_; }
  const std::vector<L1UpdateAction>& actions() const { return actions_; }

 private:
  //
  L1UpdateStatus status_;
  //
  std::size_t penalty_cycles_n_;
  // State to which line shall transition if result is committed.
  state_t state_;
  //
  std::vector<L1UpdateAction> actions_;
};

//
//
class L1CacheModelProtocol {
 public:
  L1CacheModelProtocol() = default;
  virtual ~L1CacheModelProtocol() = default;

  //
  virtual L1LineState* construct_line() const = 0;

  //
  virtual void apply(L1CacheModelApplyResult& r, L1LineState* line,
                     const CpuCommandMessage* msg) const = 0;

  //
  virtual void commit(const L1CacheModelApplyResult& r,
                      L1LineState* state) const = 0;
};

//
//
class L2LineState {
 public:
  L2LineState() {}
  virtual ~L2LineState() = default;

  // Flag indiciating if the line is currently residing in a stable
  // state.
  virtual bool is_stable() const = 0;

  // Flag indiciating if the line is currently evictable (not in a
  // transient state).
  virtual bool is_evictable() const { return is_stable(); }
};

// clang-format off
#define L2_UPDATE_ACTIONS(__func)                   \
  __func(UpdateState)                               \
  __func(EmitReadNoSnoop)                           \
  __func(EmitReadOnce)                              \
  __func(EmitReadClean)                             \
  __func(EmitReadSharedNotDirty)                    \
  __func(EmitReadShared)                            \
  __func(EmitReadUnique)                            \
  __func(EmitCleanUnique)                           \
  __func(EmitCleanShared)                           \
  __func(EmitCleanInvalid)                          \
  __func(EmitMakeUnique)                            \
  __func(EmitMakeInvalid)                           \
  __func(EmitWriteNoSnoop)                          \
  __func(EmitWriteLineUnique)                       \
  __func(EmitWriteBack)                             \
  __func(EmitWriteClean)                            \
  __func(EmitEvict)
// clang-format on

enum class L2UpdateAction {
#define __declare_enum(__name) __name,
L2_UPDATE_ACTIONS(__declare_enum)
#undef __declare_enum
};

const char* to_string(L2UpdateAction opcode);


//
//
class L2CacheModelApplyResult {
 public:
  L2CacheModelApplyResult() = default;

  bool empty() const { return actions_.empty(); }
  state_t state() const { return state_; }
  void state(state_t state) { state_ = state; }
  L2UpdateAction next() const { return actions_.front(); }
  void push(L2UpdateAction action) { actions_.push_back(action); }
  void pop() { actions_.pop_front(); }

 private:
  state_t state_;
  std::deque<L2UpdateAction> actions_;
};

//
//
class L2CacheModelProtocol {
 public:
  L2CacheModelProtocol() = default;
  virtual ~L2CacheModelProtocol() = default;

  //
  virtual L2LineState* construct_line() const = 0;

  //
  virtual void apply(L2CacheModelApplyResult& r, L2LineState* line,
                     const L1L2Message* msg) const = 0;

  //
  virtual void commit(const L2CacheModelApplyResult& r,
                      L2LineState* state) const = 0;

  //
  virtual void update_line_state(L2LineState* line, state_t state) const = 0;
};

//
//
class DirectoryLineState {
 public:
  DirectoryLineState() {}
  virtual ~DirectoryLineState() = default;

  // Flag indiciating if the line is currently residing in a stable
  // state.
  virtual bool is_stable() const = 0;

  // Flag indiciating if the line is currently evictable (not in a
  // transient state).
  virtual bool is_evictable() const { return is_stable(); }
};

//
//
class DirectoryProtocol {
 public:
  DirectoryProtocol() = default;
  virtual ~DirectoryProtocol() = default;
};

//
//
class ProtocolBuilder {
 public:
  virtual ~ProtocolBuilder() = default;

  // Create an instance of the L1 protocol
  virtual L1CacheModelProtocol* create_l1() = 0;

  // Create an instance of the L2 protocol
  virtual L2CacheModelProtocol* create_l2() = 0;

  // Create and instance of the Directory protocol
  virtual DirectoryProtocol* create_dir() = 0;
};

//
//
class ProtocolBuilderRegistry {
 public:
  static ProtocolBuilder* build(const std::string& name);

 protected:
  struct ProtocolBuilderFactory {
    virtual ~ProtocolBuilderFactory() = default;
    virtual ProtocolBuilder* construct() = 0;
  };
  void register_protocol(const std::string& name, ProtocolBuilderFactory* f) {
    m_[name] = f;
  }

 private:
  static std::map<std::string, ProtocolBuilderFactory*> m_;
};

//
//
#define CC_DECLARE_PROTOCOL_BUILDER(__name, __builder)          \
  static struct Register##__builder : ProtocolBuilderRegistry { \
    Register##__builder() {                                     \
      factory_ = new __Builder##Factory{};                      \
      register_protocol(__name, factory_);                      \
    }                                                           \
    ~Register##__builder() { delete factory_; }                 \
                                                                \
   private:                                                     \
    struct __Builder##Factory : ProtocolBuilderFactory {        \
      ProtocolBuilder* construct() { return new __builder{}; }  \
    };                                                          \
    ProtocolBuilderFactory* factory_;                           \
  } __register

}  // namespace cc

#endif
