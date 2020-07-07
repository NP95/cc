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

#ifndef CC_INCLUDE_CC_L2CACHE_H
#define CC_INCLUDE_CC_L2CACHE_H

#include <vector>

#include "cfgs.h"
#include "kernel.h"

namespace cc {

// Forwards:
class L1CacheModel;
class MessageQueue;
template <typename>
class Arbiter;
class Message;
class L2LineState;
template <typename>
class CacheModel;

//
//
class L2CacheModel : public kernel::Agent<const Message*> {
  class MainProcess;

 public:
  enum EndPoints : kernel::end_point_id_t { L1CmdReq, L1CmdRsp };

  L2CacheModel(kernel::Kernel* k, const L2CacheModelConfig& config);
  virtual ~L2CacheModel();

  L2CacheModelConfig config() const { return config_; }

  // Elaboration-Time fix-up(s)
  
  // NOC ingress end-point
  void set_noc_ep(kernel::EndPointIntf<const Message*>* noc_ep) { noc_ep_ = noc_ep; }

  void set_dm(cc::DirectoryMapper* dm) { dm_ = dm; }

 protected:
  // Elaboration callback
  virtual void elab() override;
  // Design Rule Check (DRC) callback
  virtual void drc() override;
  // Pointer to module arbiter instance.
  Arbiter<const Message*>* arb() const { return arb_; }
  // Point to module cache instance.
  CacheModel<L2LineState*>* cache() const { return cache_; }
  // Registers NOC End-Point interface.
  kernel::EndPointIntf<const Message*>* noc_ep() const { return noc_ep_; }
  // Pointer to directory mapper (address to directory mapper).
  cc::DirectoryMapper* dm() const { return dm_; }
 private:
  void build();

  // L2 Cache Configuration.
  L2CacheModelConfig config_;
  // Child L1 Caches
  std::vector<L1CacheModel*> l1cs_;
  // L1 Command Request
  MessageQueue* l1cmdreqq_ = nullptr;
  // L1 Command Response
  MessageQueue* l1cmdrspq_ = nullptr;
  // Queue selection arbiter
  Arbiter<const Message*>* arb_ = nullptr;
  // Cache Instance
  CacheModel<L2LineState*>* cache_ = nullptr;
  // End-Point into NOC instance.
  kernel::EndPointIntf<const Message*>* noc_ep_ = nullptr;
  // Directory mapper instance.
  cc::DirectoryMapper* dm_ = nullptr;
  // Main process of execution.
  MainProcess* main_;
};

}  // namespace cc

#endif
