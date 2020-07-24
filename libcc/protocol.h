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
class L1CommandList;
class L1CacheContext;

class L2CommandList;
class L2CacheContext;

class CCCommandList;
class CCContext;

class DirCommandList;
class DirContext;

class MessageQueueProxy;
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
  virtual std::string to_string() const = 0;
  virtual bool execute() = 0;
  virtual void release() { delete this; }
 protected:
  virtual ~CoherenceAction() = default;
};

using CoherenceActionList = std::vector<CoherenceAction*>;

//
//
class L1CacheModelProtocol : public kernel::Module {
 public:
  L1CacheModelProtocol(kernel::Kernel* k, const std::string& name);
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
  virtual void issue_msg(L1CommandList& cl, MessageQueueProxy* mq,
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
class L2CacheModelProtocol : public kernel::Module {
 public:
  L2CacheModelProtocol(kernel::Kernel* k, const std::string& name);
  virtual ~L2CacheModelProtocol() = default;

  //
  //
  virtual L2LineState* construct_line() const = 0;

  //
  //
  virtual void apply(L2CacheContext& ctxt, L2CommandList& cl) const = 0;

  //
  //
  virtual void evict(L2CacheContext& ctxt, L2CommandList& cl) const = 0;

 protected:
  virtual void issue_msg(L2CommandList& cl, MessageQueueProxy* mq,
                         const Message* msg) const;
};

//
//
class DirLineState {
 public:
  DirLineState() {}
  virtual void release() { delete this; }

  // Flag indiciating if the line is currently residing in a stable
  // state.
  virtual bool is_stable() const = 0;

  // Flag indiciating if the line is currently evictable (not in a
  // transient state).
  virtual bool is_evictable() const { return is_stable(); }

 protected:
  virtual ~DirLineState() = default;
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
class DirProtocol : public kernel::Module {
 public:
  DirProtocol(kernel::Kernel* k, const std::string& name);
  virtual ~DirProtocol() = default;

  //
  //
  virtual DirLineState* construct_line() const  = 0;

  //
  //
  virtual void apply(DirContext& ctxt, DirCommandList& cl) const = 0;

 protected:
  //
  virtual void issue_msg(DirCommandList& lc, MessageQueue* mq,
                         const Message* msg) const;

  //
  virtual void issue_emit_to_noc(DirContext& ctxt, DirCommandList& lc,
                                 const Message* msg, Agent* dest) const;
};

//
//
class CCLineState {
 public:
  CCLineState() = default;
  virtual void release() const { delete this; }

 protected:
  virtual ~CCLineState() = default;
};

enum class CCMessageID {};

using CCMessageIDList = std::vector<CCMessageID>;

using CCActionList = std::vector<CoherenceAction*>;

//
//
class CCProtocol : public kernel::Module {
 public:
  CCProtocol(kernel::Kernel* k, const std::string& name);
  virtual ~CCProtocol() = default;

  //
  //
  virtual CCLineState* construct_line() const = 0;

  //
  //
  virtual void apply(CCContext& ctxt, CCCommandList& cl) const = 0;

 protected:
  //
  virtual void issue_msg(CCCommandList& lc, MessageQueueProxy* mq,
                         const Message* msg) const;

  //
  virtual void issue_emit_to_noc(CCContext& ctxt, CCCommandList& lc,
                                 const Message* msg, Agent* dest) const;
};

//
//
class ProtocolBuilder {
 public:
  virtual ~ProtocolBuilder() = default;

  // Create an instance of the L1 protocol
  virtual L1CacheModelProtocol* create_l1(kernel::Kernel*) = 0;

  // Create an instance of the L2 protocol
  virtual L2CacheModelProtocol* create_l2(kernel::Kernel*) = 0;

  // Create an instance of the Dir protocol
  virtual DirProtocol* create_dir(kernel::Kernel*) = 0;

  // Create an instance of a Cache Controller protocol.
  virtual CCProtocol* create_cc(kernel::Kernel*) = 0;
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
