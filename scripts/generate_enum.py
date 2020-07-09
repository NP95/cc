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
import os

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

# Emit directive
EMIT_DIRECTIVE_RE=re.compile('^#emit:(?P<command>.*)')

class EnumWriter:
    def __init__(self, ofn):
        self.directives = []
        self.ofn = os.path.basename(ofn)
        self.ofn_h = open(self.ofn + '_enum.h', 'w')
        self.ofn_cc = open(self.ofn + '_enum.cc', 'w')
    def __enter__(self):
        # Header
        self._emit_copyright(self.ofn_h)
        self._emit_header_guard_begin(self.ofn_h)
        self._emit_namespace_begin(self.ofn_h)
        # Source
        self._emit_copyright(self.ofn_cc)
        self._emit_header_include(self.ofn_cc)
        self._emit_namespace_begin(self.ofn_cc)
        return self
    def __exit__(self, type, value, traceback):
        self._emit_namespace_end(self.ofn_h)
        self._emit_header_guard_end(self.ofn_h)
        self._emit_namespace_end(self.ofn_cc)
    def add_directive(self, dir):
        self.directives.append(dir)
    def append(self, e):
        self._append_header(e)
        self._append_source(e)
    def _append_header(self, e):
        self._emit_definition(e)
        if 'to_string' in self.directives:
            self._emit_to_string_def(e)
        if 'is_stable' in self.directives:
            self._emit_is_stable_def(e)
    def _append_source(self, e):
        if 'to_string' in self.directives:
            self._emit_to_string(e)
        if 'is_stable' in self.directives:
            self._emit_is_stable(e)
    def _emit_copyright(self, of):
        of.write(COPYRIGHT_BANNER)
        self._emit_nl(of, 1)
    def _emit_namespace_begin(self, of):
        of.write('namespace cc {\n')
        self._emit_nl(of, 1)
    def _emit_namespace_end(self, of):
        of.write('} // namespace cc\n')
        self._emit_nl(of, 1)
    def _emit_header_guard_begin(self, of):
        of.write('#ifndef CC_LIBCC_{}_ENUM_H\n'.format(self.ofn.upper()))
        of.write('#define CC_LIBCC_{}_ENUM_H\n'.format(self.ofn.upper()))
        self._emit_nl(of, 1)
        of.write('#include "cc/types.h"\n')
        self._emit_nl(of, 1)
    def _emit_header_guard_end(self, of):
        of.write('#endif\n')
    def _emit_header_include(self, of):
        of.write('#include "{}_enum.h"'.format(self.ofn))
        self._emit_nl(of, 2)
    def _emit_definition(self, e):
        self.ofn_h.write('// {}:{}\n'.format(e.fn, e.l))
        self.ofn_h.write('enum class {0} : cc::state_t {{\n  '.format(e.name))
        self.ofn_h.write(',\n  '.join([state.split(';')[0] for state in e.items]))
        self.ofn_h.write('\n};\n')
        self._emit_nl(self.ofn_h, 1)
        pass
    def _emit_to_string_def(self, e):
        self.ofn_h.write('const char* to_string({} s);\n'.format(e.name))
        self._emit_nl(self.ofn_h, 1)
    def _emit_is_stable_def(self, e):
        self.ofn_h.write('bool is_stable({} s);\n'.format(e.name))
        self._emit_nl(self.ofn_h, 1)
    def _emit_to_string(self, e):
        self.ofn_cc.write('// {}:{}\n'.format(e.fn, e.l))
        self.ofn_cc.write('const char* to_string({} s) {{\n'.format(e.name))
        self.ofn_cc.write('  switch(s) {\n')
        for item in e.items:
            state_split = item.split(';')
            self.ofn_cc.write(
                '    case {0}::{1}: return "{1}";\n'.format(e.name, state_split[0]))
        self.ofn_cc.write('    default: return "Invalid";\n')
        self.ofn_cc.write('  }\n')
        self.ofn_cc.write('}\n')
        self._emit_nl(self.ofn_cc, 1)
    def _emit_is_stable(self, e):
        self.ofn_cc.write('// {}:{}\n'.format(e.fn, e.l))
        self.ofn_cc.write('bool is_stable({} s) {{\n'.format(e.name))
        self.ofn_cc.write('  switch(s) {\n')
        for item in e.items:
            state_split = item.split(';')
            if len(state_split) > 1:
                self.ofn_cc.write('    case {0}::{1}: return true;\n'.format(e.name, state_split[0]))
        self.ofn_cc.write('    default: return false;\n')
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
    (ofn, ext) = os.path.splitext(os.path.basename(fn))
    with EnumWriter(ofn) as ew:
        enum_defs = []
        for (line_no, l) in enumerate(open(fn, 'r').readlines()):
            l = l.rstrip('\n')
            if not l or l.startswith('//'): continue
            m = ENUM_PREFIX_RE.match(l)
            if m:
                enum_defs.append(EnumDefinition(m.group('name'), fn, line_no))
                continue
            m = ENUM_SUFFIX_RE.match(l)
            if m:
                continue
            m = EMIT_DIRECTIVE_RE.match(l)
            if m:
                ew.add_directive(m.group('command'))
                continue
            enum_defs[-1].add_item(l)
        for e in enum_defs:
            ew.append(e)

def main(args):
    for fn in args[1:]:
        process_file(fn)

if __name__ == '__main__':
    main(sys.argv)
