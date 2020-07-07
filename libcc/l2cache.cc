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

#include "cc/l2cache.h"

#include "cc/sim.h"
#include "l1cache.h"
#include "primitives.h"
#include "protocol.h"
#include "cache.h"
#include "amba.h"
#include "noc.h"
#include "dir.h"
#include "utility.h"

namespace cc {

class L2CacheModel::MainProcess : public kernel::Process {

  struct Context {
    // Current arbiter tournament; retained such that the
    Arbiter<const Message*>::Tournament t;

    // Pointer to the protocol logic object.
    L2CacheModelProtocol* protocol = nullptr;

    // State containing the result of the protocol message
    // application.
    L2CacheModelApplyResult apply_result;

    // Iterator to the line into which the current operation is taking
    // place.
    CacheModel<L2LineState*>::LineIterator line_it;
  };

  // clang-format off
#define STATES(__func)				\
  __func(AwaitingMessage)			\
  __func(ProcessMessage)			\
  __func(ExecuteActions)			\
  __func(CommitContext)				\
  __func(DiscardContext)
  // clang-format on

  enum class State {
#define __declare_state(__name) __name,
    STATES(__declare_state)
#undef __declare_state
  };

  std::string to_string(State state) {
    switch (state) {
      default:
        return "Unknown";
#define __declare_to_string(__name)             \
        case State::__name:                     \
          return #__name;                       \
          break;
        STATES(__declare_to_string)
#undef __declare_to_string
    }
    return "Invalid";
  }
#undef STATES

 public:
  MainProcess(kernel::Kernel* k, const std::string& name, L2CacheModel* model)
      : kernel::Process(k, name), model_(model) {}

 private:

  State state() const { return state_; }
  void set_state(State state) {
    if (state_ != state) {
      LogMessage msg("State transition: ");
      msg.level(Level::Debug);
      msg.append(to_string(state_));
      msg.append(" -> ");
      msg.append(to_string(state));
      log(msg);
    }
    state_ = state;
  }
  
  // Initialization:
  void init() override {
    set_state(State::AwaitingMessage);
    Arbiter<const Message*>* arb = model_->arb_;
    wait_on(arb->request_arrival_event());
  }

  // Evaluation:
  void eval() override {
    switch (state()) {
      case State::AwaitingMessage: {
	handle_awaiting_message();
      } break;

      case State::ProcessMessage: {
        handle_nominated_message();
      } break;

      case State::ExecuteActions: {
        handle_execute_actions();
      } break;

      case State::CommitContext: {
        handle_commit_context();
      } break;

      case State::DiscardContext: {
        handle_discard_context();
      } break;

      default: {
        log(LogMessage{"Unknown state entered", Level::Fatal});
      } break;
    }
  }

  // Finalization:
  void fini() override {}

  void handle_awaiting_message() {
    // Idle state, awaiting more work.
    Arbiter<const Message*>* arb = model_->arb();
    Arbiter<const Message*>::Tournament t = arb->tournament();

    // Detect deadlock at L1Cache front-end. This occurs only in the
    // presence of a protocol violation and is therefore by definition
    // unrecoverable.
    if (t.deadlock()) {
      const LogMessage msg{"A protocol deadlock has been detected.",
            Level::Fatal};
      log(msg);
    }

    // Check for the presence of issue-able messages at the
    // pipeline front-end.
    if (t.has_requester()) {
      // A message is available; begin processing in the next
      // delta cycle.
      ctxt_ = Context();
      ctxt_.t = t;
      const L2CacheModelConfig& config = model_->config();
      ctxt_.protocol = config.protocol;
      set_state(State::ProcessMessage);
      next_delta();
    } else {
      // Otherwise, block awaiting the arrival of a message at on
      // the of the message queues.
      wait_on(arb->request_arrival_event());
    }
  }

  void handle_nominated_message() {
    const MsgRequesterIntf* intf = ctxt_.t.intf();
    const Message* msg = intf->peek();
    switch (msg->cls()) {
      case Message::L1L2Cmd: {
        CacheModel<L2LineState*>* cache = model_->cache();
        CacheAddressHelper ah = cache->ah();
        const L1L2Message* l1l2msg = static_cast<const L1L2Message*>(msg);
        CacheModel<L2LineState*>::Set set = cache->set(ah.set(l1l2msg->addr()));
        if (set.hit(ah.tag(l1l2msg->addr()))) {
          // Cache hit; 
        } else {
          // Cache miss; either service current command by installing
	  CacheModel<L2LineState*>::Evictor evictor;
          const std::pair<CacheModel<L2LineState*>::LineIterator, bool> line_lookup =
              evictor.nominate(set.begin(), set.end());
          // Context iterator to point to nominated line.
          ctxt_.line_it = line_lookup.first;
          if (ctxt_.line_it != set.end()) {
            const bool eviction_required = line_lookup.second;
            if (eviction_required) {
            } else {
              const L2CacheModelProtocol* protocol = ctxt_.protocol;
              L2LineState* l2line = protocol->construct_line();
              // Install newly constructed line in the cache set.
              set.install(ctxt_.line_it, ah.tag(l1l2msg->addr()), l2line);
              // Apply state update to the line.
              L2CacheModelApplyResult& apply_result = ctxt_.apply_result;
              protocol->apply(apply_result, l2line, l1l2msg);
              // Advance to execute state.
              set_state(State::ExecuteActions);
              next_delta();
            }
          } else {
          }
        }
        
      } break;
      default: {
      } break;
        case Message::Invalid:
            break;
        case Message::CpuCmd:
            break;
        case Message::CpuRsp:
            break;
    }
  }

  void handle_execute_actions() {
    L2CacheModelApplyResult& ar = ctxt_.apply_result;
    while (!ar.empty()) {
      const L2UpdateAction action = ar.next();
      switch (action) {
        case L2UpdateAction::UpdateState: {
          // Apply line state update.
          const L2CacheModelProtocol* protocol = ctxt_.protocol;
          L2LineState* line = ctxt_.line_it->t();
          protocol->update_line_state(line, ar.state());
          // Action completeed; discard.
          ar.pop();
        } break;

        case L2UpdateAction::EmitReadNoSnoop:
        case L2UpdateAction::EmitReadOnce:
        case L2UpdateAction::EmitReadClean:
        case L2UpdateAction::EmitReadSharedNotDirty:
        case L2UpdateAction::EmitReadShared:
        case L2UpdateAction::EmitReadUnique:
        case L2UpdateAction::EmitCleanUnique:
        case L2UpdateAction::EmitCleanShared:
        case L2UpdateAction::EmitCleanInvalid:
        case L2UpdateAction::EmitMakeUnique:
        case L2UpdateAction::EmitMakeInvalid:
        case L2UpdateAction::EmitWriteNoSnoop:
        case L2UpdateAction::EmitWriteLineUnique:
        case L2UpdateAction::EmitWriteBack:
        case L2UpdateAction::EmitWriteClean:
        case L2UpdateAction::EmitEvict: {
          // Emit AMBA message to directory (through interconnect).
          const bool can_issue = true;
          if (can_issue) {
            // Issue message to interconnect.
            AceCmdMsg* msg = new AceCmdMsg(/* TODO */ nullptr);
            msg->opcode(update_to_opcode(action));
            // Issue message into interconnect
            issue_to_noc(msg);
            // Action completeed; discard.
            ar.pop();
          } else {
            // Message blockes on flow-control to interconnect.
          }
        } break;
        default: {
        } break;
      }
    }
  }

  void handle_commit_context() {
  }

  void handle_discard_context() {
  }

  void issue_to_noc(const Message* msg) {
    // Encapsulate application message in NOC trasnport and issue
    // to the NOC for forwarding to the destination.
    NocMessage* nocmsg = new NocMessage(msg->t());
    nocmsg->set_payload(msg);
    nocmsg->set_origin(model_);

    // Compute home directory for the current address.
    const DirectoryMapper* dmap = model_->dm();
    DirectoryModel* dm = dmap->lookup(0);
    kernel::EndPointIntf<const Message*>* ep = dm->end_point(ut(DirEp::CmdQ));
    if (ep == nullptr) {
      LogMessage lm("Cannot find end-point for current address");
      lm.level(Level::Fatal);
      log(lm);
    }
    nocmsg->set_ep(ep);

    // Issue encapsulated message to interconnect.
    model_->issue(model_->noc_ep(), kernel::Time{10, 0}, nocmsg);
  }

  // Pointer to parent L2.
  L2CacheModel* model_ = nullptr;

  State state_;

  Context ctxt_;
};

L2CacheModel::L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config)
    : kernel::Agent<const Message*>(k, config.name), config_(config) {
  build();
}

L2CacheModel::~L2CacheModel() {}

void L2CacheModel::build() {
  // Counter to compute the number of slots in the L2 command queue to
  // support the maximum number of commands from all associated child
  // L1.
  std::size_t l2_cmd_queue_depth_n = 0;

  // Counter to compute the number of slots required in the L2
  // response queue to support the maximum number of commands to all
  // associated child L1.
  std::size_t l2_rsp_queue_depth_n = 0;

  // Construct child instances.
  for (const L1CacheModelConfig& l1cfg : config_.l1configs) {
    // Compute running count of queue sized.
    l2_cmd_queue_depth_n += l1cfg.l2_cmdq_credits_n;
    l2_rsp_queue_depth_n += l1cfg.l1_cmdq_slots_n;
    // COnstruct child L1 instance.
    L1CacheModel* l1c = new L1CacheModel(k(), l1cfg);
    add_child_module(l1c);
    l1cs_.push_back(l1c);
  }

  // Construct L1 Command Request Queue
  l1cmdreqq_ = new MessageQueue(k(), "l1cmdreqq", l2_cmd_queue_depth_n);
  add_child_module(l1cmdreqq_);

  // Construct L1 Command Response Queue
  l1cmdrspq_ = new MessageQueue(k(), "l1cmdrspq", l2_rsp_queue_depth_n);
  add_child_module(l1cmdrspq_);

  // Arbiter
  arb_ = new Arbiter<const Message*>(k(), "arb");
  arb_->add_requester(l1cmdreqq_);
  arb_->add_requester(l1cmdrspq_);
  add_child_module(arb_);

  // Main thread
  main_ = new MainProcess(k(), "main", this);
  add_child_process(main_);

  // Cache model
  cache_ = new CacheModel<L2LineState*>(config_.cconfig);
}

void L2CacheModel::elab() {
  // Fix up parent cache.
  for (L1CacheModel* l1c : l1cs_) {
    l1c->set_parent(this);
  }
  add_end_point(EndPoints::L1CmdReq, l1cmdreqq_);
  add_end_point(EndPoints::L1CmdRsp, l1cmdrspq_);
}

void L2CacheModel::drc() {
  if (l1cs_.empty()) {
    // Typically, a nominal configuration would expect some number of
    // L1 caches to belong to the L2. This is not strictly necessary,
    // as the L2 cache can essentially remain ccompletley inert and
    // idle, but otherwise points to some misconfiguration in the
    // system somoewhere.
    LogMessage msg{"L2 has no child L1 cache(s).", Level::Warning};
    log(msg);
  }

  if (noc_ep() == nullptr) {
    // We expect the NOC end-point to to have been defined in some
    // prior set. If this has not been defined, the L2 instance is
    // essentially independent of any interconnect and therefore
    // cannot interact with other agents in the simulation.
    LogMessage msg("NOC End-Point is not defined.", Level::Fatal);
    log(msg);
  }

  if (dm() == nullptr) {
    // The Directory Mapper object computes the host directory for a
    // given address. In a single directory system, this is a basic
    // mapping to a single directory instance, but in more performant
    // systems this may some non-trivial mapping to multiple home
    // directories.
    LogMessage msg("Directory mapper is not defined.", Level::Fatal);
    log(msg);
  }
}

}  // namespace cc
