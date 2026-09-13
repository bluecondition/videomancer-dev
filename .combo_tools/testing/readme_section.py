#!/usr/bin/env python3
"""Print README sections (same shape as the kept programs' sections) from the header comment of
programs/ron/catalogue-combo/testing/<name>/<name>.vhd for each name given."""
import re, sys, pathlib
ROOT = pathlib.Path('/home/ron/videomancer-dev/programs/ron/catalogue-combo/testing')
for name in sys.argv[1:]:
    lines = []
    for l in (ROOT / name / f'{name}.vhd').read_text().splitlines():
        if not l.startswith('--'): break
        lines.append(l[2:].rstrip())
    # paragraphs split on blank comment lines
    paras, cur = [], []
    for l in lines:
        if l.strip() == '':
            if cur: paras.append(cur); cur = []
        else: cur.append(l)
    if cur: paras.append(cur)
    intro = ' '.join(p.strip() for p in paras[0])
    ctrl = []
    for p in paras[1:-1]:
        for l in p:
            m = re.match(r'\s*(K\d|S\d+|P12)\s+(\S+)\s*(.*)', l)
            if m: ctrl.append(f'- {m.group(1):<3} {m.group(2):<8} {m.group(3)}'.rstrip())
            elif ctrl and l.startswith('   '): ctrl[-1] += ' ' + l.strip()
    outro = ' '.join(p.strip() for p in paras[-1]) if len(paras) > 2 else ''
    print(f'## {name}\n{intro}\n')
    print('\n'.join(ctrl))
    if outro: print(f'\n{outro}')
    print()
