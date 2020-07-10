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

## Utility macro to generate an header and object file from an
## enumerate definition file.
macro (cc_generate gen_file)
  get_filename_component(gen_prefix ${gen_file} NAME_WE)

  set(header_file "${gen_prefix}_gen.h")
  set(source_file "${gen_prefix}_gen.cc")

  set(script_py ${CMAKE_SOURCE_DIR}/scripts/generate_deps.py)

  add_custom_command(
    OUTPUT ${header_file} ${source_file}
    COMMAND python3 ${script_py}
      ${CMAKE_CURRENT_SOURCE_DIR}/${gen_file}
#    COMMAND ${Python_EXECUTABLE} ${script_py}
#      ${CMAKE_CURRENT_SOURCE_DIR}/${gen_file}
    DEPENDS ${gen_file} ${script_py}
    COMMENT "Generating ${gen_prefix}.gen..."
    )
  add_custom_target(
    run_gen_${gen_prefix} DEPENDS ${header_file} ${source_file})
endmacro ()
