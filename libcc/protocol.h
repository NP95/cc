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

#ifndef CC_LIBCC_PROTOCOL_H
#define CC_LIBCC_PROTOCOL_H

#include <map>
#include <string>
#include <vector>
#include <deque>
#include "types.h"

namespace cc {

// Message Forwards:
class Message;
class L1CacheModel;
class L2CacheModel;
class CacheController;
class DirectoryModel;

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
//
class CoherenceAction {
 public:
  virtual ~CoherenceAction() = default;

  virtual bool execute() = 0;
  virtual void release() { delete this; }
};

using L1CoherenceActionList = std::vector<CoherenceAction*>;


//
//
class L1CoherenceContext {
 public:
  L1CoherenceContext() = default;

  L1LineState* line() const { return line_; }
  const Message* msg() const { return msg_; }

  void set_line(L1LineState* line) { line_ = line; }
  void set_msg(const Message* msg) { msg_ = msg; }

 private:
  L1LineState* line_ = nullptr;
  const Message* msg_ = nullptr;
};

//
//
class L1CacheModelProtocol {
 public:
  L1CacheModelProtocol() = default;
  virtual ~L1CacheModelProtocol() = default;

  //
  L1CacheModel* l1cache() const { return l1cache_; }

  //
  void set_l1cache(L1CacheModel* l1cache) { l1cache_ = l1cache; }

  //
  //
  virtual L1LineState* construct_line() const = 0;

  //
  //
  virtual std::pair<bool, L1CoherenceActionList> apply(
      const L1CoherenceContext& context) const = 0;

 private:
  L1CacheModel* l1cache_ = nullptr;
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

//
//
using L2CoherenceActionList = std::vector<CoherenceAction*>;


//
//
class L2CoherenceContext {
 public:
  L2CoherenceContext() = default;

  L2LineState* line() const { return line_; }
  const Message* msg() const { return msg_; }

  void set_line(L2LineState* line) { line_ = line; }
  void set_msg(const Message* msg) { msg_ = msg; }

 private:
  L2LineState* line_ = nullptr;
  const Message* msg_ = nullptr;
};

//
//
class L2CacheModelProtocol {
 public:
  L2CacheModelProtocol() = default;
  virtual ~L2CacheModelProtocol() = default;

  //
  L2CacheModel* l2cache() const { return l2cache_; }

  //
  void set_l2cache(L2CacheModel* l2cache) { l2cache_ = l2cache; }

  //
  //
  virtual L2LineState* construct_line() const = 0;

  //
  //
  virtual std::pair<bool, L2CoherenceActionList> apply(
      const L2CoherenceContext& context) const = 0;

 private:
  L2CacheModel* l2cache_ = nullptr;
};

//
//
class DirLineState {
 public:
  DirLineState() {}
  virtual ~DirLineState() = default;

  // Flag indiciating if the line is currently residing in a stable
  // state.
  virtual bool is_stable() const = 0;

  // Flag indiciating if the line is currently evictable (not in a
  // transient state).
  virtual bool is_evictable() const { return is_stable(); }
};

using DirectoryActionList = std::vector<CoherenceAction*>;

//
//
class DirCoherenceContext {
 public:
  DirCoherenceContext() = default;

  DirLineState* line() const { return line_; }
  const Message* msg() const { return msg_; }

  void set_line(DirLineState* line) { line_ = line; }
  void set_msg(const Message* msg) { msg_ = msg; }

 private:
  DirLineState* line_ = nullptr;
  const Message* msg_ = nullptr;
};

//
//
class DirectoryProtocol {
 public:
  DirectoryProtocol() = default;
  virtual ~DirectoryProtocol() = default;

  //
  DirectoryModel* dir() const { return dir_; }

  //
  void set_dir(DirectoryModel* dir) { dir_ = dir; }

  //
  //
  virtual DirLineState* construct_line() const = 0;

  //
  //
  virtual std::pair<bool, DirectoryActionList> apply(
      const DirCoherenceContext& context) const = 0;

 private:
  DirectoryModel* dir_ = nullptr;
};

//
//
class CacheControllerLineState {
 public:
  CacheControllerLineState() = default;
  virtual ~CacheControllerLineState() = default;
};

using CacheControllerActionList = std::vector<CoherenceAction*>;

//
//
class CacheControllerContext {
 public:
  CacheControllerContext() = default;

  const Message* msg() const { return msg_; }
  CacheControllerLineState* line() const { return line_; }

  void set_msg(const Message* msg) { msg_ = msg; }
  void set_line(CacheControllerLineState* line) { line_ = line; }

 private:
  const Message* msg_ = nullptr;
  CacheControllerLineState* line_ = nullptr;
};

//
//
class CacheControllerProtocol {
 public:
  CacheControllerProtocol() = default;
  virtual ~CacheControllerProtocol() = default;

  //
  CacheController* cc() const { return cc_; }

  //
  void set_cc(CacheController* cc) { cc_ = cc; }

  //
  virtual CacheControllerLineState* construct_line() const = 0;

  //
  //
  virtual std::pair<bool, CacheControllerActionList> apply(
      const CacheControllerContext& context) const = 0;

 private:
  CacheController* cc_ = nullptr;
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

  // Create an instance of the Directory protocol
  virtual DirectoryProtocol* create_dir() = 0;

  // Create an instance of a Cache Controller protocol.
  virtual CacheControllerProtocol* create_cc() = 0;
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
