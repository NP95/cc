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
MSG_PREFIX_RE=re.compile('^@msg_begin\((?P<header>.*)\)')

# Enum terminal line
MSG_SUFFIX_RE=re.compile('^@msg_end')

# Emit directive
EMIT_DIRECTIVE_RE=re.compile('^#include:(?P<command>.*)')

class MsgWriter:
    def __init__(self, ofn):
        self.includes = []
        self.ofn = os.path.basename(ofn)
        self.ofn_h = open(self.ofn + '_msg.h', 'w')
        self.ofn_cc = open(self.ofn + '_msg.cc', 'w')
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
    def append(self, e):
        self._append_header(e)
        self._append_source(e)
    def add_directive(self, dir):
        (opcode, oprand) = dir.split(":")
        if opcode == "#include":
            self.includes.append(oprand)
    def emit_preamble(self):
        self._emit_includes(self.ofn_h)
        self._emit_namespace_begin(self.ofn_h)
    def _append_header(self, msg):
        self._emit_header(msg, self.ofn_h)
    def _append_source(self, msg):
        self._emit_source(msg, self.ofn_cc)
    def _emit_includes(self, of):
        self._emit_nl(of, 1)
        of.write('#include "msg.h"\n')
        for include in self.includes:
            of.write('#include "{}"\n'.format(include))
        self._emit_nl(of, 1)
    def _emit_header(self, msg, of):
        (classname, cls) = msg.header.split(';')
        of.write('class {} : public Message {{\n'.format(classname))
        of.write('public:\n')
        of.write('  {}() : Message(MessageClass::{}) {{}}\n'.format(classname, cls))
        self._emit_nl(of, 1)
        of.write('  std::string to_string() const;\n')
        self._emit_nl(of, 1)
        for item in msg.items:
            (n, t) = item.split(';')
            of.write("  {0} {1}() const {{ return {1}_; }}\n".format(t, n))
            of.write("  void set_{0}({1} {0}) {{ {0}_ = {0}; }}\n".format(n, t))
            self._emit_nl(of, 1)
        of.write('private:\n')
        for item in msg.items:
            (n, t) = item.split(';')
            of.write("  {} {}_;\n".format(t, n))
        of.write('};\n')
        self._emit_nl(of, 1)
    def _emit_source(self, msg, of):
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
        of.write('#ifndef CC_LIBCC_{}_MSG_H\n'.format(self.ofn.upper()))
        of.write('#define CC_LIBCC_{}_MSG_H\n'.format(self.ofn.upper()))
    def _emit_header_guard_end(self, of):
        of.write('#endif\n')
    def _emit_header_include(self, of):
        of.write('#include "{}_msg.h"\n'.format(self.ofn))
        of.write('#include "utility.h"\n')
        of.write('#include <sstream>\n')
        self._emit_nl(of, 2)
    def _emit_nl(self, of, n):
        for i in range(n):
            of.write('\n')

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
    with MsgWriter(ofn) as msgw:
        msg_defs = []
        lines = open(fn, 'r').readlines()

        # First pass to obtain directives
        for (line_no, l) in enumerate(lines):
            l = l.rstrip('\n')
            if not l or l.startswith('//'): continue
            m = EMIT_DIRECTIVE_RE.match(l)
            if m:
                msgw.add_directive(l)

        for (line_no, l) in enumerate(lines):
            l = l.rstrip('\n')
            if not l or l.startswith('//'): continue
            m = MSG_PREFIX_RE.match(l)
            if m:
                msg_defs.append(MsgDefinition(m.group('header'), fn, line_no))
                continue
            m = MSG_SUFFIX_RE.match(l)
            if m:
                continue
            m = EMIT_DIRECTIVE_RE.match(l)
            if m:
                continue
            msg_defs[-1].add_item(l)

        msgw.emit_preamble()
        for msg in msg_defs:
            msgw.append(msg)

def main(args):
    for fn in args[1:]:
        process_file(fn)

if __name__ == '__main__':
    main(sys.argv)
