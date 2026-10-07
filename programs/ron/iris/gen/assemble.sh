#!/bin/bash
# Regenerates ../iris.vhd from the four RTL parts in src/ plus the ROM / sequencer
# program emitted by gen_roms.py and useq.py.  iris.vhd is GENERATED -- edit the parts
# here, never iris.vhd itself.
#   IRIS_DEBUG=1  splices src/iris_dbg.vhd (report-only process) in for sims
#   IRIS_DEST=<dir> writes <dir>/iris.vhd instead (e.g. a throwaway sim copy)
set -e
G=$(cd "$(dirname "$0")" && pwd)
D=${IRIS_DEST:-$(dirname "$G")}
S=$G/src
P=iris_r
TMP=$(mktemp -d "${TMPDIR:-$HOME}/.iris_gen.XXXXXX")
trap 'rm -rf "$TMP"' EXIT
python3 "$G/gen_roms.py" > "$TMP/roms.vhdinc"
python3 "$G/useq.py" vhdl >> "$TMP/roms.vhdinc"
python3 - "$S" "$P" "$TMP/roms.vhdinc" "$D/iris.vhd" <<'PY'
import os, sys
S, P, roms_path, out_path = sys.argv[1:]
rd = lambda n: open(os.path.join(S, n)).read()
p4 = rd(P + "4.vhd")
if os.environ.get("IRIS_DEBUG"): p4 = rd("iris_dbg.vhd") + p4
out = rd(P + "1.vhd").replace("-- @@ROMS@@", open(roms_path).read()) + rd(P + "2.vhd") + rd(P + "3.vhd") + p4
open(out_path, "w").write(out)
PY
wc -l "$D/iris.vhd"
