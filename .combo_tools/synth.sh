#!/bin/bash
# usage: synth.sh <vhd> <log>
cd /home/ron/videomancer-dev
source videomancer-sdk/build/oss-cad-suite/environment
R=videomancer-sdk/fpga
PK="$R/common/rtl/video_timing/video_timing_pkg.vhd $R/common/rtl/video_stream/video_stream_pkg.vhd $R/common/rtl/video_sync/video_sync_pkg.vhd $(ls $R/common/rtl/*/*_pkg.vhd | grep -v 'video_timing_pkg\|video_stream_pkg\|video_sync_pkg' | tr '\n' ' ') $R/core/yuv444_30b/rtl/core_pkg.vhd $R/core/yuv444_30b/rtl/program_top.vhd"
timeout 600 yosys -q -l "$2" -m ghdl -p "ghdl --std=08 $PK $1 -e program_top; synth_ice40; stat" > /dev/null 2>&1
echo "exit=$?" >> "$2"
