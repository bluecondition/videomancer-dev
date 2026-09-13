#!/bin/bash
# usage: prof.sh <name> <cfg> <seed>  -- synth + nextpnr with --timing-allow-fail, print critical-path summary
cd /home/ron/videomancer-dev
source videomancer-sdk/build/oss-cad-suite/environment
p=$1; cfg=${2:-hd_analog}; seed=${3:-1}
B=/home/ron/videomancer-dev/.combo_tools/prof/$p; mkdir -p $B
make -C videomancer-sdk/fpga PROGRAM=$p PROJECT_ROOT=/home/ron/videomancer-dev/programs/ron/catalogue-combo/testing/$p BUILD_ROOT=$B CONFIG=$cfg VIDEOMANCER_SDK_ROOT=/home/ron/videomancer-dev/videomancer-sdk $B/bitstreams/$cfg.json > $B/synth_$cfg.log 2>&1; echo "synth exit $?"
nextpnr-ice40 --hx4k --package tq144 --pcf videomancer-sdk/fpga/hardware/rev_b/constraints/pin_map.pcf --freq 74.25 --json $B/bitstreams/$cfg.json --asc $B/x_$cfg.asc --seed $seed --router router2 --timing-allow-fail -l $B/prof_${cfg}_$seed.log > /dev/null 2>&1; echo "pnr exit $?"
grep -a "Max frequency" $B/prof_${cfg}_$seed.log | tail -1
awk '/Critical path report/,0' $B/prof_${cfg}_$seed.log | grep -ao "s_[a-z0-9_]*" | sed 's/[0-9]*$//' | sort | uniq -c | sort -rn | head -8
awk '/Critical path report/,0' $B/prof_${cfg}_$seed.log | grep -a "logic.*routing\|Source\|Net " | head -30
