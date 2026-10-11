#!/usr/bin/env bash
# Golden testbench: motion_key RTL vs model/wake_model.py, bit-exact on the
# grid (cur, M) and per-update energy. Usage: tb/run_golden.sh [workdir]
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WAKE="$(dirname "$HERE")"
ROOT="$(cd "$WAKE/../../.." && pwd)"
WORK="${1:-$HOME/.cache/wake_golden}"
PY="${WAKE_PY:-$HOME/.venvs/wake/bin/python}"
mkdir -p "$WORK"
source "$ROOT/videomancer-sdk/build/oss-cad-suite/environment"
# GHDL links with -lz but the box only has libz.so.1.3: point the linker at a symlink
mkdir -p "$WORK/lib" && ln -sf /usr/lib/x86_64-linux-gnu/libz.so.1.3 "$WORK/lib/libz.so"
export LIBRARY_PATH="$WORK/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
"$PY" "$HERE/golden.py" stim "$WORK"
cd "$WORK"
ghdl -a --std=08 "$ROOT/videomancer-sdk/fpga/common/rtl/video_stream/video_stream_pkg.vhd" \
    "$ROOT/videomancer-sdk/fpga/common/rtl/video_sync/video_field_detector.vhd" \
    "$WAKE/motion_key.vhd" "$HERE/tb_motion_key.vhd"
ghdl -e --std=08 tb_motion_key
ghdl -r --std=08 tb_motion_key ${GOLDEN_FMT:+-gG_TID=0010} -gG_STEADY=${GOLDEN_STEADY:-0}
"$PY" "$HERE/golden.py" check "$WORK"
