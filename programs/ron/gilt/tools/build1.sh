#!/bin/bash
# Single-config GILT build into programs/ron/gilt/.synth/<cfg>_s<seed> with a short report.
# usage: tools/build1.sh <config> [seed] [router]   e.g. tools/build1.sh hd_analog 1 router2
CFG=${1:-hd_analog}; SEED=${2:-1}; RT=${3:-router2}
ROOT=/home/ron/videomancer-dev
OUT=$ROOT/programs/ron/gilt/.synth/${CFG}_s${SEED}_${RT}
rm -rf "$OUT"; mkdir -p "$OUT"
source $ROOT/videomancer-sdk/build/oss-cad-suite/environment 2>/dev/null
timeout 2400 make -C $ROOT/videomancer-sdk/fpga PROGRAM=gilt CONFIG=$CFG \
  PROJECT_ROOT=$ROOT/programs/ron/gilt BUILD_ROOT=$OUT ROUTER="--router $RT" SEED=$SEED > $OUT/build.log 2>&1
echo "== $CFG seed $SEED $RT (exit $?)"
grep -E "ICESTORM_LC:|ICESTORM_RAM:" $OUT/build.log | tail -2
grep -E "Max frequency for clock" $OUT/build.log | tail -1
L=$(grep -n "Critical path report for clock" $OUT/build.log | tail -1 | cut -d: -f1)
[ -n "$L" ] && sed -n "$L,$((L+400))p" $OUT/build.log | grep -E "Source|ns logic" | sed 's/.*Source //' | sed 's/u_core_top.yuv444_30b_top_inst.//' | cut -c1-90 | awk 'NR==1||/ns logic/{print; next} {print}' | head -40
