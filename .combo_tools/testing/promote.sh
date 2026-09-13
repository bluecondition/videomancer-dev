#!/bin/bash
# usage: promote.sh <name> "<quote>" "<blurb>" -- move a passing testing program into the main catalogue-combo set
set -e; cd /home/ron/videomancer-dev
n=$1; q=${2:-"$n is good"}; b=${3:-""}
mv programs/ron/catalogue-combo/testing/$n programs/ron/catalogue-combo/$n
mv out/rev_b/ron/catalogue-combo/testing/$n.vmprog out/rev_b/ron/catalogue-combo/$n.vmprog
python3 - "$n" "$q" "$b" <<'PY'
import pathlib, sys
n, q, b = sys.argv[1:4]
t = pathlib.Path('programs/ron/catalogue-combo/testing/README.md'); m = pathlib.Path('programs/ron/catalogue-combo/README.md')
s = t.read_text(); i = s.index(f'## {n}\n'); j = s.find('\n## ', i + 1); j = len(s) if j < 0 else j + 1
sec = s[i:j]; t.write_text(s[:i] + s[j:])
m.write_text(m.read_text().rstrip('\n') + '\n\n' + sec.rstrip('\n') + '\n')
e = pathlib.Path('programs/ron/EXPLORE_LIST.md')
e.write_text(e.read_text().rstrip('\n') + f'\n| 2026-09-13 | {n} | programs/ron/catalogue-combo/{n}/ | "{q}" ({b}; moved out of testing/) |\n')
PY
echo "promoted $n"; ls programs/ron/catalogue-combo/$n; tail -1 programs/ron/EXPLORE_LIST.md
