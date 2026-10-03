#!/usr/bin/env python3
"""Regenerate the generated constants inside cubist.vhd:

  C_UCODE   microcode ROM (frame_ucode.py)
  C_TABLE   sticker state + turn tables (turn_model.table_rom)
  C_CHROMA  sticker chroma lit by a 5-bit light level: word (level*8 + colour)
            = cv8 << 8 | cu8, both centred on 128 (x4 on use)
"""
import subprocess, sys
sys.path.insert(0, '.')
import turn_model as tm

rom = subprocess.run([sys.executable, 'frame_ucode.py'], capture_output=True,
                     text=True, check=True)


def const16(name, words):
    rows, row = [], []
    for i, w in enumerate(words):
        row.append(f'x"{w:04X}"')
        if len(row) == 12 or i == len(words) - 1:
            rows.append('        ' + ', '.join(row))
            row = []
    return (f'    constant {name} : t_sd := (\n' + ',\n'.join(rows) + ' );')


def chroma():
    out = []
    for lvl in range(32):
        # stickers stay vivid in shade: the light only takes chroma down to
        # ~2/3 (the luma carries the shading), in steps too fine to see
        g = 256 - (31 - lvl) * 3
        for c in range(8):
            if c < 6:
                u8, v8 = tm.C_STU[c] >> 2, tm.C_STV[c] >> 2
            else:
                u8 = v8 = 128
            cu = max(0, min(255, (((u8 - 128) * g) >> 8) + 128))
            cv = max(0, min(255, (((v8 - 128) * g) >> 8) + 128))
            out.append(cv << 8 | cu)
    return out


def replace_block(src, start_pat, new):
    lines = src.split('\n')
    a = next(i for i, l in enumerate(lines) if start_pat in l)
    b = next(i for i in range(a, len(lines))
             if lines[i].rstrip().endswith(');') and 'x"' in lines[i])
    lines[a:b + 1] = new.split('\n')
    return '\n'.join(lines)


src = open('cubist.vhd').read()
src = replace_block(src, 'type t_ucode is array', rom.stdout.rstrip('\n'))
src = replace_block(src, 'constant C_TABLE : t_sd', const16('C_TABLE', tm.table_rom()))
src = replace_block(src, 'constant C_CHROMA : t_sd', const16('C_CHROMA', chroma()))
open('cubist.vhd', 'w').write(src)
print(rom.stderr.strip())
