## ========================================================================= ##
## Copyright (c) 2020, Stephen Henry
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright notice, this
##   list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright notice,
##   this list of conditions and the following disclaimer in the documentation
##   and/or other materials provided with the distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
## ========================================================================= ##

cc_generate(moesi.gen)
cc_generate(protocol.gen)
cc_generate(amba.gen)
cc_generate(cpu.gen)
cc_generate(l1cache.gen)
cc_generate(l2cache.gen)
cc_generate(ccntrl.gen)
cc_generate(msg.gen)
cc_generate(dir.gen)
cc_generate(llc.gen)
cc_generate(mem.gen)

add_library(cc SHARED
  kernel.cc
  primitives.cc
  cache.cc
  protocol.cc
  protocol_gen.cc
  sim.cc
  l1cache.cc
  l1cache_gen.cc
  l2cache.cc
  l2cache_gen.cc
  cpu.cc
  cpu_gen.cc
  moesi.cc
  moesi_gen.cc
  amba.cc
  amba_gen.cc
  noc.cc
  dir.cc
  dir_gen.cc
  llc.cc
  llc_gen.cc
  mem.cc
  mem_gen.cc
  cpucluster.cc
  ccntrl.cc
  ccntrl_gen.cc
  utility.cc
  soc.cc
  msg_gen.cc
  builder.cc
  )

target_include_directories(cc PRIVATE ${CMAKE_SOURCE_DIR}/include/cc)
target_include_directories(cc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(cc PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
