##========================================================================== //
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
##========================================================================== //


macro (create_test testfile)
  get_filename_component(testname ${testfile} NAME_WE)
  
  add_executable(${testname} ${testfile})
  target_include_directories(${testname} PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/tests/include
    )
  target_link_libraries(${testname}
    cc
    gtest_main
    gtest
    pthread
    cctest
    )
  add_test(NAME ${testname} COMMAND ${testname})
endmacro ()

add_subdirectory(test)
add_subdirectory(general)
add_subdirectory(stimulus)
add_subdirectory(directed)

#add_library(cctest SHARED test_utilities.cc)
#target_include_directories(cctest PRIVATE
#  ${CMAKE_SOURCE_DIR}/src
#  ${CMAKE_SOURCE_DIR}/include
#  )
#target_link_libraries(cctest
#  cc
#  )

#create_test(utility utility.cc)
#create_test(kernel kernel.cc)
#create_test(primitives primitives.cc)
#create_test(cache cache.cc)

# Basic protocol/sanity tests
#create_test(basic_1dir_1cc_1cpu basic_1dir_1cc_1cpu.cc)
#create_test(basic_1dir_1cc_2cpu basic_1dir_1cc_2cpu.cc)
#create_test(basic_1dir_2cc_1cpu basic_1dir_2cc_1cpu.cc)

# Basic eviction tests
#create_test(evict_1dir_1cc_1cpu evict_1dir_1cc_1cpu.cc)

# Basic resource allocation/reservation tests
#create_test(res_1dir_1cc_1cpu res_1dir_1cc_1cpu.cc)

# Programmatic stimulus tests
#create_test(pro_1dir_1cc_1cpu pro_1dir_1cc_1cpu.cc)
