#!/bin/bash
# usage: elab.sh <name>   -- elaborate programs/ron/<name>/<name>.vhd (plus any extra .vhd in the dir)
cd /home/ron/videomancer-dev
source videomancer-sdk/build/oss-cad-suite/environment
R=videomancer-sdk/fpga
PK="$R/common/rtl/video_timing/video_timing_pkg.vhd $R/common/rtl/video_stream/video_stream_pkg.vhd $R/common/rtl/video_sync/video_sync_pkg.vhd $(ls $R/common/rtl/*/*_pkg.vhd | grep -v 'video_timing_pkg\|video_stream_pkg\|video_sync_pkg' | tr '\n' ' ') $R/core/yuv444_30b/rtl/core_pkg.vhd $R/core/yuv444_30b/rtl/program_top.vhd"
D=programs/ron/catalogue-combo/$1; [ -d "$D" ] || D=programs/ron/catalogue-combo/testing/$1
EXTRA=$(ls $D/*.vhd | grep -v "/$1.vhd$" | tr '\n' ' ')
yosys -q -m ghdl -p "ghdl --std=08 $PK $EXTRA $D/$1.vhd -e program_top" 2>&1 | grep -v "^$" | head -20
echo "elab exit=${PIPESTATUS[0]}"
