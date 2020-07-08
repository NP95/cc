##========================================================================== ##
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
##========================================================================== ##

import sys
import re

COPYRIGHT_BANNER='''\
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
'''

# Enum initial line
ENUM_PREFIX_RE=re.compile('^@enum_begin\((?P<name>.*)\)')

# Enum terminal line
ENUM_SUFFIX_RE=re.compile('^@enum_end')

class EnumWriter:
    def __init__(self, ofn):
        self.ofn = ofn
        self.ofn_h = open(ofn + '_enum.h', 'w')
        self.ofn_cc = open(ofn + '_enum.cc', 'w')
    def __enter__(self):
        # Header
        self._emit_copyright(self.ofn_h)
        self._emit_header_guard_begin(self.ofn_h)
        # Source
        self._emit_copyright(self.ofn_cc)
        self._emit_header_include(self.ofn_cc)
        return self
    def __exit__(self, type, value, traceback):
        self._emit_header_guard_end(self.ofn_h)
    def append(self, e):
        self._append_header(e)
        self._append_source(e)
    def _append_header(self, e):
        self._emit_definition(e)
        self._emit_to_string_def(e)
        pass
    def _append_source(self, e):
        self._emit_to_string(e)
    def _emit_copyright(self, of):
        of.write(COPYRIGHT_BANNER)
        self._emit_nl(of, 1)
    def _emit_header_guard_begin(self, of):
        of.write('#ifndef CC_LIBCC_{}\n'.format(self.ofn.upper()))
        of.write('#define CC_LIBCC_{}\n'.format(self.ofn.upper()))
        self._emit_nl(of, 1)
    def _emit_header_guard_end(self, of):
        of.write('#endif\n')
    def _emit_header_include(self, of):
        of.write('#include "{}.h"'.format(self.ofn))
        self._emit_nl(of, 2)
    def _emit_definition(self, e):
        self.ofn_h.write('// {}:{}\n'.format(e.fn, e.l))
        self.ofn_h.write('enum class {0} : state_t {{\n  '.format(e.name))
        self.ofn_h.write(',\n  '.join(e.items))
        self.ofn_h.write('\n}\n')
        self._emit_nl(self.ofn_h, 1)
        pass
    def _emit_to_string_def(self, e):
        self.ofn_h.write('const char* to_string({} s);\n'.format(e.name))
        self._emit_nl(self.ofn_h, 1)
        pass
    def _emit_to_string(self, e):
        self.ofn_cc.write('// {}:{}\n'.format(e.fn, e.l))
        self.ofn_cc.write('const char* to_string({} s) {{\n'.format(e.name))
        self.ofn_cc.write('  switch(s) {\n')
        for item in e.items:
            self.ofn_cc.write(
                '    case {0}::{1}: return "{1}";\n'.format(e.name, item))
        self.ofn_cc.write('    default: return "Invalid";\n')
        self.ofn_cc.write('  }\n')
        self.ofn_cc.write('}\n')
        self._emit_nl(self.ofn_cc, 1)
    def _emit_nl(self, of, n):
        for i in range(n):
            of.write('\n')

class EnumDefinition:
    def __init__(self, name, fn, l):
        self.name = name
        self.fn = fn
        self.l = l
        self.items = []
    def add_item(self, item):
        self.items.append(item)

def process_file(fn):
    enum_defs = []
    line_no = 0
    for l in open(fn, 'r').readlines():
        line_no += 1
        l = l.rstrip('\n')
        if not l or l.startswith('//'): continue

        m = ENUM_PREFIX_RE.match(l)
        if m:
            enum_defs.append(EnumDefinition(m.group('name'), fn, line_no))
            continue

        m = ENUM_SUFFIX_RE.match(l)
        if m:
            continue

        enum_defs[-1].add_item(l)

    ofn = fn.rstrip('.enum')

    with EnumWriter(ofn) as ew:
        for e in enum_defs:
            ew.append(e)

def main(args):
    for fn in args[1:]:
        process_file(fn)
        

if __name__ == '__main__':
    main(sys.argv)
