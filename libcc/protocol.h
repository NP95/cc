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

#include <deque>
#include <map>
#include <string>
#include <vector>

#include "amba.h"
#include "msg.h"
#include "types.h"

namespace cc {

// Message Forwards:
class L1CacheContext;

class L1CacheModel;
class L2CacheContext;
class L2CacheModel;
class CC;
class CCContext;
class DirModel;
class MessageQueue;
class Agent;

//
//
class CohSrtMsg : public Message {
 public:
  CohSrtMsg() : Message(MessageClass::CohSrt) {}

  //
  std::string to_string() const override;

  //
  Agent* origin() const { return origin_; }

  //
  void set_origin(Agent* origin) { origin_ = origin; }

 private:
  // Originator agent of coherence transaction.
  Agent* origin_;
};

//
//
class CohEndMsg : public Message {
 public:
  CohEndMsg() : Message(MessageClass::CohEnd) {}

  //
  std::string to_string() const override;
};

//
//
class CohCmdMsg : public Message {
 public:
  CohCmdMsg() : Message(MessageClass::CohCmd) {}

  //
  std::string to_string() const override;

  //
  AceCmdOpcode opcode() const { return opcode_; }
  Agent* origin() const { return origin_; }
  addr_t addr() const { return addr_; }

  //
  void set_opcode(AceCmdOpcode opcode) { opcode_ = opcode; }
  void set_origin(Agent* origin) { origin_ = origin; }
  void set_addr(addr_t addr) { addr_ = addr; }

 private:
  AceCmdOpcode opcode_;
  Agent* origin_ = nullptr;
  addr_t addr_;
};

//
//
class CohCmdRspMsg : public Message {
 public:
  CohCmdRspMsg() : Message(MessageClass::CohCmdRsp) {}

  //
  std::string to_string() const override;
};

//
//
class CohFwdMsg : public Message {
 public:
  CohFwdMsg() : Message(MessageClass::CohFwd) {}

  //
  std::string to_string() const override;
};

//
//
class CohFwdRspMsg : public Message {
 public:
  CohFwdRspMsg() : Message(MessageClass::CohFwdRsp) {}

  //
  std::string to_string() const override;
};

//
//
class CohInvMsg : public Message {
 public:
  CohInvMsg() : Message(MessageClass::CohInv) {}

  //
  std::string to_string() const override;
};

//
//
class CohInvRspMsg : public Message {
 public:
  CohInvRspMsg() : Message(MessageClass::CohInvRsp) {}

  //
  std::string to_string() const override;
};

//
//
class L1LineState {
 public:
  L1LineState() {}
  virtual ~L1LineState() = default;

  // Release line back to pool, or destruct.
  virtual void release() { delete this; }

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

using CoherenceActionList = std::vector<CoherenceAction*>;

//
//
class L1CacheModelProtocol : public kernel::Module {
 public:
  L1CacheModelProtocol(kernel::Kernel* k);
  virtual ~L1CacheModelProtocol() = default;

  //
  //
  virtual L1LineState* construct_line() const = 0;

  //
  //
  virtual void apply(L1CacheContext& c, L1CommandList& cl) const = 0;

  //
  //
  virtual void evict(L1CacheContext& c, L1CommandList& cl) const = 0;

 protected:
  virtual void issue_msg(L1CommandList& cl, MessageQueue* mq,
                         const Message* msg) const;
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
  virtual void install(L2CacheContext& c) const = 0;

  //
  //
  virtual void apply(L2CacheContext& c) const = 0;

  //
  //
  virtual void evict(L2CacheContext& c) const = 0;

 protected:
  virtual void issue_msg(CoherenceActionList& al, MessageQueue* mq,
                         const Message* msg) const;

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

using DirActionList = std::vector<CoherenceAction*>;

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
class DirProtocol {
 public:
  DirProtocol() = default;
  virtual ~DirProtocol() = default;

  //
  DirModel* dir() const { return dir_; }

  //
  void set_dir(DirModel* dir) { dir_ = dir; }

  //
  //
  virtual DirLineState* construct_line() const = 0;

  //
  //
  virtual std::pair<bool, DirActionList> apply(
      const DirCoherenceContext& context) const = 0;

 protected:
  //
  virtual void issue_emit_to_noc(DirActionList& al, const Message* msg,
                                 Agent* dest) const;

  //
  virtual void issue_protocol_violation(DirActionList& al) const;

 private:
  DirModel* dir_ = nullptr;
};

//
//
class CCLineState {
 public:
  CCLineState() = default;
  virtual ~CCLineState() = default;
};

enum class CCMessageID {};

using CCMessageIDList = std::vector<CCMessageID>;

using CCActionList = std::vector<CoherenceAction*>;

//
//
class CCProtocol {
 public:
  CCProtocol() = default;
  virtual ~CCProtocol() = default;

  //
  //
  virtual void install(CCContext& c) const = 0;

  //
  //
  virtual void apply(CCContext& c) const = 0;

 protected:
  //
  virtual void issue_msg(CCContext& c, MessageQueue* mq,
                         const Message* msg) const;

  //
  virtual void issue_emit_to_noc(CCContext& c, const Message* msg,
                                 Agent* dest) const;
};

//
//
class ProtocolBuilder {
 public:
  virtual ~ProtocolBuilder() = default;

  // Create an instance of the L1 protocol
  virtual L1CacheModelProtocol* create_l1(kernel::Kernel*) = 0;

  // Create an instance of the L2 protocol
  virtual L2CacheModelProtocol* create_l2() = 0;

  // Create an instance of the Dir protocol
  virtual DirProtocol* create_dir() = 0;

  // Create an instance of a Cache Controller protocol.
  virtual CCProtocol* create_cc() = 0;
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
