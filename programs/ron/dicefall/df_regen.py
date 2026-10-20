#!/usr/bin/env python3
"""Regenerate the generated constants inside dicefall.vhd (between the
@@BEGIN/@@END GENERATED markers):

  C_UCODE   microcode ROM (df_ucode.py)
  C_TABLE   die table RAM image, 512 x 16 (df_tables.py)
  C_EROM    engine ROM: quarter sine, numeral constants, inks, face gamma
            (df_ucode.py)
  C_GLY0..3 numeral distance fields, the four 2 x 2-interleaved banks
            (df_glyphs.py)
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import df_tables as T  # noqa: E402
import df_ucode as U   # noqa: E402
import df_glyphs as GL  # noqa: E402
from df_uasm import vhdl_ucode, vhdl_rom16  # noqa: E402


def vhdl_rom4(name, words):
    rows, row = [], []
    for i, w in enumerate(words):
        row.append(f'x"{w & 15:X}"')
        if len(row) == 24 or i == len(words) - 1:
            rows.append('        ' + ', '.join(row))
            row = []
    return f'    constant {name} : t_gly := (\n' + ',\n'.join(rows) + ' );'


def main():
    words, rom = U.build()
    table, meta = T.build()
    depth = 1024 if len(words) > 512 else 512
    body = '\n'.join([
        f'    -- {len(words)} microcode words; table {meta["used"]}/512 words',
        vhdl_ucode(words, depth),
        '    type t_tbl is array (0 to 511) of std_logic_vector(15 downto 0);',
        vhdl_rom16('C_TABLE', table, 't_tbl'),
        '    type t_erom is array (0 to 255) of std_logic_vector(15 downto 0);',
        vhdl_rom16('C_EROM', rom, 't_erom'),
        '    type t_gly is array (0 to 1023) of std_logic_vector(3 downto 0);',
    ] + [vhdl_rom4(f'C_GLY{b}', bank) for b, bank in enumerate(GL.banks())])
    path = os.path.join(HERE, 'dicefall.vhd')
    src = open(path).read()
    new = re.sub(r'(-- @@BEGIN GENERATED[^\n]*\n).*?(\s*-- @@END GENERATED)',
                 lambda m: m.group(1) + body + '\n' + m.group(2).lstrip('\n'), src, flags=re.S)
    assert new != src or body in src
    open(path, 'w').write(new)
    print(f'{len(words)} words (ROM depth {depth}), table {meta["used"]} words')


if __name__ == '__main__':
    main()
