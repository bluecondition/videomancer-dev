#!/usr/bin/env python3
"""Regenerate the C_UCODE constant inside cubist.vhd from frame_ucode.py."""
import subprocess, sys

rom = subprocess.run([sys.executable, 'frame_ucode.py'], capture_output=True,
                     text=True, check=True)
new = rom.stdout.rstrip('\n')
src = open('cubist.vhd').read().split('\n')
a = next(i for i, l in enumerate(src) if 'type t_ucode is array' in l)
b = next(i for i in range(a, len(src))
         if src[i].rstrip().endswith(');') and 'x"' in src[i])
src[a:b + 1] = new.split('\n')
open('cubist.vhd', 'w').write('\n'.join(src))
print(rom.stderr.strip())
