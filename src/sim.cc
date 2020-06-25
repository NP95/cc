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

#include "sim.h"

namespace cc {

Clock::Clock(kernel::Kernel* k, const std::string& name, int ticks, int period)
    : Module(k, name), ticks_(ticks), period_(period), rising_edge_event_(k) {
  struct ClockProcess : kernel::Process {
    ClockProcess(kernel::Kernel* k, Clock* clk)
        : Process(k, "ClockProcess"), clk_(clk) {
      ticks_ = clk->ticks();
    }
    void init() override {
      if (ticks_ == 0) return;
      kernel::Time time = k()->time();
      time.time += clk_->period();
      wait_until(time);
    }
    void eval() override {
      // Notify
      clk_->rising_edge_event().notify();
      if (--ticks_ != 0) {
        // Schedule next
        kernel::Time time = k()->time();
        time.time += clk_->period();
        wait_until(time);
      }
    }
    Clock* clk_;
    int ticks_;
  };
  p_ = new ClockProcess(k, this);
  add_child(p_);
}

}  // namespace cc
