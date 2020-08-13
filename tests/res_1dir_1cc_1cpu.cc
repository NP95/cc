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

#include "cache.h"
#include "l1cache.h"
#include "cc/cfgs.h"
#include "cc/soc.h"
#include "cc/stimulus.h"
#include "test_utilities.h"
#include "gtest/gtest.h"

class SocModel {
 public:
  SocModel(cc::kernel::Kernel* k, const cc::SocConfig& cfg)
      : k_(k) {
    top_ = new cc::SocTop(k_, cfg);
  }

  ~SocModel() { delete top_; }

  //
  const cc::SocConfig& config() const { return top_->config(); }

  // Stimulus instance.
  cc::Stimulus* stimulus() const { return top_->stimulus(); }

  template<typename T>
  T get_object_as(const std::string& name) {
    return static_cast<T>(top_->find_path(name));
  }
  
  void run() {
    k_->invoke_elab();
    k_->invoke_drc();
    k_->invoke_init();
    const cc::kernel::RunMode r = cc::kernel::RunMode::ToExhaustion;
    const cc::kernel::Time t;
    k_->invoke_run(r, t);
  }

 private:
  // Simulation Kernel
  cc::kernel::Kernel* k_ = nullptr;
  // SOC top instance
  cc::SocTop* top_ = nullptr;
};

TEST(Res111, BlockedOnTT) {
  cc::kernel::Kernel k;

  const std::vector<const char*> trace = {
    "+200",
    "C:0,LD,0x000", // 1
    "C:0,LD,0x100", // 2
    "C:0,LD,0x200", // 3
    "C:0,LD,0x300", // 4
    "C:0,LD,0x400", // 5
    "C:0,LD,0x500", // 6
    "C:0,LD,0x600", // 7
    "C:0,LD,0x700"  // 8
  };
  // Build simple test configuration.
  cc::SocConfig cfg;
  test::build_config(cfg, 1, 1, 1, trace);
  for (cc::CpuClusterConfig& cpu_cfg : cfg.ccls) {
    for (cc::L1CacheAgentConfig& l1_cfg : cpu_cfg.l1c_configs) {
      l1_cfg.tt_entries_n = 1;
      //l1_cfg.is_blocking_cache = false;
    }
  }
  SocModel soc(&k, cfg);
  soc.run();

  const cc::Stimulus* stimulus = soc.stimulus();

  // Validate expected transaction count.
  EXPECT_EQ(stimulus->issue_n(), 8);

  // Validate that all transactions have retired at end-of-sim.
  EXPECT_EQ(stimulus->issue_n(), stimulus->retire_n());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
