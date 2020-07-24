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

#include "protocol.h"

#include "ccntrl.h"
#include "dir.h"
#include "l1cache.h"
#include "l2cache.h"
#include "ccntrl.h"
#include "noc.h"
#include "sim.h"
#include "utility.h"

namespace cc {

std::string CohSrtMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

std::string CohEndMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

std::string CohCmdMsg::to_string() const {
  using cc::to_string;
  Hexer h;
  KVListRenderer r;
  render_msg_fields(r);
  r.add_field("opcode", to_string(opcode()));
  r.add_field("addr", h.to_hex(addr()));
  r.add_field("origin", origin()->path());
  return r.to_string();
}

std::string CohCmdRspMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

std::string CohFwdMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

std::string CohFwdRspMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

std::string CohInvMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

std::string CohInvRspMsg::to_string() const {
  KVListRenderer r;
  render_msg_fields(r);
  return r.to_string();
}

using pbr = ProtocolBuilderRegistry;

// Coherence protocol registry
std::map<std::string, pbr::ProtocolBuilderFactory*> pbr::m_;

ProtocolBuilder* ProtocolBuilderRegistry::build(const std::string& name) {
  ProtocolBuilder* r = nullptr;
  auto it = m_.find(name);
  if (it != m_.end()) {
    ProtocolBuilderFactory* factory = it->second;
    r = factory->construct();
  }
  return r;
}

//
//
struct EmitMessageActionProxy : public CoherenceAction {
  EmitMessageActionProxy(MessageQueueProxy* mq, const Message* msg)
      : mq_(mq), msg_(msg) {}
  std::string to_string() const override {
    KVListRenderer r;
    r.add_field("action", "emit message");
    r.add_field("mq", mq_->path());
    r.add_field("msg", msg_->to_string());
    return r.to_string();
  }
  bool execute() override { return mq_->issue(msg_); }
 private:
  MessageQueueProxy* mq_ = nullptr;
  const Message* msg_ = nullptr;
};
/*
struct EmitMessageAction : public CoherenceAction {
  EmitMessageAction(MessageQueue* mq, const Message* msg)
      : mq_(mq), msg_(msg) {}
  std::string to_string() const override {
    std::stringstream ss;
    {
      KVListRenderer r(ss);
      r.add_field("action", "emit message");
      r.add_field("mq", mq_->path());
      r.add_field("msg", msg_->to_string());
    }
    return ss.str();
  }
  bool execute() override { return mq_->issue(msg_); }
 private:
  MessageQueue* mq_ = nullptr;
  const Message* msg_ = nullptr;
};
*/
L1CacheModelProtocol::L1CacheModelProtocol(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {}

void L1CacheModelProtocol::issue_msg(L1CommandList& cl, MessageQueueProxy* mq,
                                     const Message* msg) const {


  CoherenceAction* action = new EmitMessageActionProxy(mq, msg);
  cl.push_back(L1CommandBuilder::from_action(action));
}

L2CacheModelProtocol::L2CacheModelProtocol(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {}

void L2CacheModelProtocol::issue_msg(L2CommandList& cl, MessageQueueProxy* mq,
                                     const Message* msg) const {
  CoherenceAction* action = new EmitMessageActionProxy(mq, msg);
  cl.push_back(L2CommandBuilder::from_action(action));
}

CCProtocol::CCProtocol(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {}

void CCProtocol::issue_msg(CCCommandList& cl, MessageQueueProxy* mq,
                           const Message* msg) const {
  CoherenceAction* action = new EmitMessageActionProxy(mq, msg);
  cl.push_back(CCCommandBuilder::from_action(action));
}

void CCProtocol::issue_emit_to_noc(CCContext& ctxt, CCCommandList& cl,
                                   const Message* msg, Agent* dest) const {
  CCModel* cc = ctxt.cc();
  // Encapsulate message in NOC transport protocol.
  NocMsg* nocmsg = new NocMsg;
  nocmsg->set_t(msg->t());
  nocmsg->set_payload(msg);
  nocmsg->set_origin(cc);
  nocmsg->set_dest(dest);
  // Issue Message Emit action.
  CoherenceAction* action = new EmitMessageActionProxy(cc->cc_noc__msg_q(), nocmsg);
  cl.push_back(CCCommandBuilder::from_action(action));
}

DirProtocol::DirProtocol(kernel::Kernel* k, const std::string& name)
    : Module(k, name) {}

void DirProtocol::issue_msg(DirCommandList& cl, MessageQueueProxy* mq,
                           const Message* msg) const {
  CoherenceAction* action = new EmitMessageActionProxy(mq, msg);
  cl.push_back(DirCommandBuilder::from_action(action));
}

void DirProtocol::issue_emit_to_noc(DirContext& ctxt, DirCommandList& cl,
                                   const Message* msg, Agent* dest) const {
  // Encapsulate message in NOC transport protocol.
  NocMsg* nocmsg = new NocMsg;
  nocmsg->set_t(msg->t());
  nocmsg->set_payload(msg);
  nocmsg->set_origin(ctxt.dir());
  nocmsg->set_dest(dest);
  // Issue Message Emit action.
  CoherenceAction* action =
      new EmitMessageActionProxy(ctxt.dir()->dir_noc__msg_q(), nocmsg);
  cl.push_back(DirCommandBuilder::from_action(action));
}

}  // namespace cc
