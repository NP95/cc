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

#include "cc/sim.h"

namespace cc {

MessageQueue::MessageQueue(kernel::Kernel* k, const std::string& name, std::size_t n)
    : kernel::Module(k, name) {
  build(n);
}

bool MessageQueue::is_requesting() const {
  return false;
}

void MessageQueue::grant() {
}

Message* MessageQueue::data() const {
  return nullptr;
}

kernel::Event& MessageQueue::request_arrival_event() {
  return q_->non_empty_event();
}

void MessageQueue::build(std::size_t n) {
  q_ = new Queue<Message*>(k(), "queue", n);
  add_child(q_);
}

ProcessorModel::ProcessorModel(kernel::Kernel* k, const std::string& name)
    : kernel::Module(k, name) {
  build();
}

bool ProcessorModel::is_requesting() const {
  return false;
}

void ProcessorModel::grant() {
}

Message* ProcessorModel::data() const {
  return nullptr;
}

kernel::Event& ProcessorModel::request_arrival_event() {
  return stim_->matured_event();
}

void ProcessorModel::build() {
}

void ProcessorModel::elab() {
}

void ProcessorModel::drc() {
  // Do DRC
  if (stim_ == nullptr) {
    // No transaction source associated with L1; raise warning.
    const LogMessage msg(
        "L1Cache has no associated stimulus.", Level::Warning);
    log(msg);
  }
}

} // namespace cc
