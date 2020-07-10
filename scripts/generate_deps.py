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
CMD_DIRECTIVE_RE=re.compile('^#cmd:(?P<command>.*)')

# Enum initial line
MSG_PREFIX_RE=re.compile('^@msg_begin\((?P<header>.*)\)')

# Enum terminal line
MSG_SUFFIX_RE=re.compile('^@msg_end')

# Emit directive
INCLUDE_DIRECTIVE_RE=re.compile('^#include:(?P<command>.*)')

class GenWriter:
    def __init__(self, ofn):
        self.directives = []
        self.includes = []
        self.ofn = os.path.basename(ofn)
        self.ofn_h = open(self.ofn + '_gen.h', 'w')
        self.ofn_cc = open(self.ofn + '_gen.cc', 'w')
    def __enter__(self):
        # Header
        self._emit_copyright(self.ofn_h)
        self._emit_header_guard_begin(self.ofn_h)
        # Source
        self._emit_copyright(self.ofn_cc)
        self._emit_header_include(self.ofn_cc)
        self._emit_namespace_begin(self.ofn_cc)
        return self
    def __exit__(self, type, value, traceback):
        self._emit_namespace_end(self.ofn_h)
        self._emit_header_guard_end(self.ofn_h)
        self._emit_namespace_end(self.ofn_cc)
    def append_directive(self, directive):
        self.directives.append(directive)
    def append_msg(self, msg):
        self._emit_msg_header(msg)
        self._emit_msg_source(msg)
    def append_enum(self, e):
        self._emit_enum_header(e)
        self._emit_to_string_def(e)
        if 'emit_is_stable' in self.directives:
            self._emit_is_stable_def(e)
        self._emit_to_string(e)
        if 'emit_is_stable' in self.directives:
            self._emit_is_stable(e)
    def add_directive(self, dir):
        (opcode, oprand) = dir.split(":")
        if opcode == "#include":
            self.includes.append(oprand)
        elif opcode == "#emit":
            self.directive.append(oprand)
    def emit_preamble(self, msgs):
        self._emit_includes(self.ofn_h, msgs)
        self._emit_namespace_begin(self.ofn_h)
    def _emit_includes(self, of, msgs):
        self._emit_nl(of, 1)
        of.write('#include "cc/types.h"\n')
        if msgs:
            # Suppress generation of msg.h whenever control file has
            # no message definitions (i.e in the msg.h case)
            of.write('#include "msg.h"\n')
        for include in self.includes:
            of.write('#include "{}"\n'.format(include))
        self._emit_nl(of, 1)
    def _emit_enum_header(self, e):
        self.ofn_h.write('// {}:{}\n'.format(e.fn, e.l))
        self.ofn_h.write('enum class {0} : cc::state_t {{\n  '.format(e.name))
        self.ofn_h.write(',\n  '.join([state.split(';')[0] for state in e.items]))
        self.ofn_h.write('\n};\n')
        self._emit_nl(self.ofn_h, 1)
    def _emit_to_string_def(self, e):
        self.ofn_h.write('const char* to_string({} s);\n'.format(e.name))
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
    def _emit_is_stable_def(self, e):
        self.ofn_h.write('bool is_stable({} s);\n'.format(e.name))
        self._emit_nl(self.ofn_h, 1)
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
    def _emit_msg_header(self, msg):
        (classname, cls) = msg.header.split(';')
        self.ofn_h.write('class {} : public Message {{\n'.format(classname))
        self.ofn_h.write('public:\n')
        self.ofn_h.write('  {}() : Message(MessageClass::{}) {{}}\n'.format(classname, cls))
        self._emit_nl(self.ofn_h, 1)
        self.ofn_h.write('  std::string to_string() const;\n')
        self._emit_nl(self.ofn_h, 1)
        for item in msg.items:
            (n, t) = item.split(';')
            self.ofn_h.write("  {0} {1}() const {{ return {1}_; }}\n".format(t, n))
            self.ofn_h.write("  void set_{0}({1} {0}) {{ {0}_ = {0}; }}\n".format(n, t))
            self._emit_nl(self.ofn_h, 1)
        self.ofn_h.write('private:\n')
        for item in msg.items:
            (n, t) = item.split(';')
            self.ofn_h.write("  {} {}_;\n".format(t, n))
        self.ofn_h.write('};\n')
        self._emit_nl(self.ofn_h, 1)
    def _emit_msg_source(self, msg):
        of = self.ofn_cc
        (classname, cls) = msg.header.split(';')
        of.write('std::string {}::to_string() const {{\n'.format(classname))
        of.write('  using cc::to_string;\n')
        of.write('  using std::to_string;\n')
        of.write('  std::stringstream ss;\n')
        of.write('  {\n')
        of.write('    KVListRenderer r(ss);\n')
        for item in msg.items:
            (n, t) = item.split(';')
            of.write('    r.add_field("{0}", to_string({0}()));\n'.format(n))
        of.write('  }\n')
        of.write('  return ss.str();\n');
        of.write('}\n')
        self._emit_nl(of, 1)
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
        of.write('#ifndef CC_LIBCC_{}_GEN_H\n'.format(self.ofn.upper()))
        of.write('#define CC_LIBCC_{}_GEN_H\n'.format(self.ofn.upper()))
    def _emit_header_guard_end(self, of):
        of.write('#endif\n')
    def _emit_header_include(self, of):
        of.write('#include "{}_gen.h"\n'.format(self.ofn))
        of.write('#include "utility.h"\n')
        of.write('#include <sstream>\n')
        self._emit_nl(of, 2)
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

class MsgDefinition:
    def __init__(self, header, fn, l):
        self.header = header
        self.fn = fn
        self.l = l
        self.items = []
    def add_item(self, item):
        self.items.append(item)

def process_file(fn):
    (ofn, ext) = os.path.splitext(os.path.basename(fn))
    with GenWriter(ofn) as genw:
        msg_defs = []
        enum_defs = []

        in_msg = False
        in_enum = False

        lines = open(fn, 'r').readlines()
        # First pass to obtain directives
        for (line_no, l) in enumerate(lines):
            l = l.rstrip('\n')
            if not l or l.startswith('//'): continue
            m = CMD_DIRECTIVE_RE.match(l)
            if m:
                genw.add_directive(l)
                continue
            m = INCLUDE_DIRECTIVE_RE.match(l)
            if m:
                genw.add_directive(l)
                continue

        for (line_no, l) in enumerate(lines):
            l = l.rstrip('\n')
            if not l or l.startswith('//'): continue

            ## @msg_begin:
            m = MSG_PREFIX_RE.match(l)
            if m:
                in_msg = True
                msg_defs.append(MsgDefinition(m.group('header'), fn, line_no))
                continue

            ## @msg_end:
            m = MSG_SUFFIX_RE.match(l)
            if m:
                in_msg = False
                continue

            ## @emit_begin:
            m = ENUM_PREFIX_RE.match(l)
            if m:
                in_enum = True
                enum_defs.append(EnumDefinition(m.group('name'), fn, line_no))
                continue

            ## @emit_end:
            m = ENUM_SUFFIX_RE.match(l)
            if m:
                in_enum = False
                continue

            ##
            m = CMD_DIRECTIVE_RE.match(l)
            if m:
                genw.append_directive(m.group('command'))
                continue

            if in_msg:
                msg_defs[-1].add_item(l)
            if in_enum:
                enum_defs[-1].add_item(l)

        genw.emit_preamble(msg_defs)
        for enum in enum_defs:
            genw.append_enum(enum)
        for msg in msg_defs:
            genw.append_msg(msg)

def main(args):
    for fn in args[1:]:
        process_file(fn)

if __name__ == '__main__':
    main(sys.argv)
