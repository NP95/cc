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

#cc_generate_enum(moesi.enum)
#cc_generate_enum(protocol.enum)
#cc_generate_enum(amba.enum)
#cc_generate_enum(cpu.enum)
#cc_generate_enum(l1cache.enum)
#cc_generate_enum(l2cache.enum)
#cc_generate_enum(ccntrl.enum)
#cc_generate_enum(msg.enum)
#cc_generate_enum(dir.enum)
#cc_generate_enum(llc.enum)
#cc_generate_enum(mem.enum)

cc_generate_msg(llc.msg)

add_library(cc SHARED
  kernel.cc
  primitives.cc
  cache.cc
  protocol.cc
  protocol_enum.cc
  sim.cc
  l1cache.cc
  l1cache_enum.cc
  l2cache.cc
  l2cache_enum.cc
  cpu.cc
  cpu_enum.cc
  moesi.cc
  moesi_enum.cc
  amba.cc
  amba_enum.cc
  noc.cc
  dir.cc
  dir_enum.cc
  llc.cc
  llc_enum.cc
  llc_msg.cc
  mem.cc
  mem_enum.cc
  cpucluster.cc
  ccntrl.cc
  ccntrl_enum.cc
  utility.cc
  soc.cc
  msg_enum.cc
  builder.cc
  )

target_include_directories(cc PRIVATE ${CMAKE_SOURCE_DIR}/include/cc)
target_include_directories(cc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(cc PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
