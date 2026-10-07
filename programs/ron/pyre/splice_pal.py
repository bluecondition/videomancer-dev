#!/usr/bin/env python3
"""Regenerate C_PAL in pyre.vhd from genpal.py."""
import subprocess, sys, os
d = os.path.dirname(os.path.abspath(__file__))
out = subprocess.run([sys.executable, os.path.join(d, 'genpal.py')], capture_output=True, text=True, check=True).stdout
const = out[out.index('  constant C_PAL'):].rstrip() + '\n'
p = os.path.join(d, 'pyre.vhd')
s = open(p).read()
a = s.index('  constant C_PAL')
b = s.index('  );\n', a) + len('  );\n')
open(p, 'w').write(s[:a] + const + s[b:])
print('C_PAL replaced')
