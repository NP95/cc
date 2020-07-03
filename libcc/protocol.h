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

#include <string>

namespace cc {

namespace kernel {

// Forwards
template<typename T> class Agent;
}

//
//
class Transaction {
 public:
  Transaction() {}

  std::string to_string_short() const {
    return "Some transaction";
  }
  std::string to_string() const {
    return "Some transaction.";
  };
};


#define MESSAGE_CLASSES(__func)                 \
  __func(Invalid)                               \
  __func(CpuCmd)                                \
  __func(CpuRsp)

//
//
class Message {
 public:
#define __declare_enum(__t) __t,
  enum Cls {
    MESSAGE_CLASSES(__declare_enum)
  };
#undef __declare_enum

  Message() {}
  Message(Transaction* t, Cls cls) : t_(t), cls_(cls) {}
  virtual ~Message() = default;

  Transaction* t() const { return t_; }
  Cls cls() const { return cls_; }
  kernel::Agent<const Message*>* agent() const { return origin_; }

  std::string to_string_short() const {
    return "Some message";
  }
  std::string to_string() const {
    return "Some message";
  }

  void set_origin(kernel::Agent<const Message*>* origin) { origin_ = origin; }
  void set_t(Transaction* t) { t_ = t; }
  void set_cls(Cls cls) { cls_ = cls; }
  
 private:
  // Parent transaction;
  Transaction* t_;
  // Message type
  Cls cls_;
  // Originating agent.
  kernel::Agent<const Message*>* origin_;
};

} // namespace cc

#endif
