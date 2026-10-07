#!/usr/bin/env python3
"""Generate/refresh the constant tables in pyre.vhd: C_SD (sim smoothstep S 0..256
and derivative D = 128*6f(1-f) for 7-bit fractions), C_SS6 (display smoothstep
0..64 for 6-bit fractions), C_MOD3, and C_PAL (via genpal.py)."""
import numpy as np, subprocess, sys, os
d = os.path.dirname(os.path.abspath(__file__))
f = np.arange(128) / 128.0
S = np.round(256 * (3 * f**2 - 2 * f**3)).astype(int)
D = np.round(128 * 6 * f * (1 - f)).astype(int)
f6 = np.arange(64) / 64.0
S6 = np.round(64 * (3 * f6**2 - 2 * f6**3)).astype(int)
out = []
out.append('  -- @@TABLES_BEGIN@@')
out.append('  type t_sd is array(0 to 127) of unsigned(16 downto 0);')
out.append('  constant C_SD : t_sd := (')
rows = []
for i in range(0, 128, 4):
    rows.append('    ' + ', '.join(f'{i+j} => "{(S[i+j] << 8) | D[i+j]:017b}"' for j in range(4)))
out.append(',\n'.join(rows) + '\n  );')
out.append('  type t_ss6 is array(0 to 63) of unsigned(6 downto 0);')
out.append('  constant C_SS6 : t_ss6 := (')
rows = []
for i in range(0, 64, 8):
    rows.append('    ' + ', '.join(f'{i+j} => "{S6[i+j]:07b}"' for j in range(8)))
out.append(',\n'.join(rows) + '\n  );')
out.append('  type t_mod3 is array(0 to 127) of unsigned(1 downto 0);')
out.append('  constant C_MOD3 : t_mod3 := (')
rows = []
for i in range(0, 128, 16):
    rows.append('    ' + ', '.join(f'{i+j} => "{(i+j) % 3:02b}"' for j in range(16)))
out.append(',\n'.join(rows) + '\n  );')
pal = subprocess.run([sys.executable, os.path.join(d, 'genpal.py')], capture_output=True, text=True, check=True).stdout
out.append(pal.rstrip())
out.append('  -- @@TABLES_END@@')
block = '\n'.join(out) + '\n'
p = os.path.join(d, 'pyre.vhd')
s = open(p).read()
if '-- @@TABLES@@' in s:
    s = s.replace('  -- @@TABLES@@\n', block)
else:
    a = s.index('  -- @@TABLES_BEGIN@@'); b = s.index('  -- @@TABLES_END@@\n') + len('  -- @@TABLES_END@@\n')
    s = s[:a] + block + s[b:]
open(p, 'w').write(s)
print('tables written')
