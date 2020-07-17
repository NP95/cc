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
#include "sim.h"
#include "noc.h"
#include "dir.h"
#include "ccntrl.h"
#include "utility.h"

namespace cc {

std::string CohSrtMsg::to_string() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
  }
  return ss.str();
}

std::string CohEndMsg::to_string() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
  }
  return ss.str();
}

std::string CohCmdMsg::to_string() const {
  using cc::to_string;
  
  std::stringstream ss;
  {
    Hexer h;
    KVListRenderer r(ss);
    render_msg_fields(r);
    r.add_field("opcode", to_string(opcode()));
    r.add_field("addr", h.to_hex(addr()));
    r.add_field("origin", origin()->path());
  }
  return ss.str();
}


std::string CohCmdRspMsg::to_string() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
  }
  return ss.str();
}

std::string CohFwdMsg::to_string() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
  }
  return ss.str();
}

std::string CohFwdRspMsg::to_string() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
  }
  return ss.str();
}

std::string CohInvMsg::to_string() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
  }
  return ss.str();
}

std::string CohInvRspMsg::to_string() const {
  std::stringstream ss;
  {
    KVListRenderer r(ss);
    render_msg_fields(r);
  }
  return ss.str();
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
struct EmitMessageAction : public CoherenceAction {
  EmitMessageAction(MessageQueue* mq, const Message* msg)
      : mq_(mq), msg_(msg)
  {}
  bool execute() override {
    return mq_->issue(msg_);
  }
 private:
  MessageQueue* mq_ = nullptr;
  const Message* msg_ = nullptr;
};


void L1CacheModelProtocol::issue_msg(
    L1CoherenceActionList& al, MessageQueue* mq, const Message* msg) const {
  al.push_back(new EmitMessageAction(mq, msg));
}

void L2CacheModelProtocol::issue_msg(
    L1CoherenceActionList& al, MessageQueue* mq, const Message* msg) const {
  al.push_back(new EmitMessageAction(mq, msg));
}

void DirProtocol::issue_emit_to_noc(
    DirActionList& al, const Message* msg, Agent* dest) const {
  // Encapsulate message in NOC transport protocol.
  NocMsg* nocmsg = new NocMsg;
  nocmsg->set_t(msg->t());
  nocmsg->set_payload(msg);
  nocmsg->set_origin(dir_);
  nocmsg->set_dest(dest);
  // Issue Message Emit action.
  al.push_back(new EmitMessageAction(dir_->dir_noc__msg_q(), nocmsg));
}

void DirProtocol::issue_protocol_violation(DirActionList& al) const {
  // TODO  
}

void CCProtocol::issue_emit_to_noc(
    CCActionList& al, const Message* msg, Agent* dest) const {
  // Encapsulate message in NOC transport protocol.
  NocMsg* nocmsg = new NocMsg;
  nocmsg->set_t(msg->t());
  nocmsg->set_payload(msg);
  nocmsg->set_origin(cc_);
  nocmsg->set_dest(dest);
  // Issue Message Emit action.
  al.push_back(new EmitMessageAction(cc_->cc_noc__msg_q(), nocmsg));
}

void CCProtocol::issue_protocol_violation(CCActionList& al) const {
  // TODO  
}

}  // namespace cc
