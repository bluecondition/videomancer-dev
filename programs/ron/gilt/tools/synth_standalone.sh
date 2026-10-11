#!/bin/bash
# Standalone yosys synthesis of GILT (program only, no core) for quick size ranking.
# Prints LUT4 / carry / FF / RAM counts. Full fit = tools/build1.sh (core adds ~600 LUT;
# placed LC was ~1.5-1.6x the total LUT4 count for this design).
ROOT=/home/ron/videomancer-dev; S=$ROOT/videomancer-sdk/fpga; G=$ROOT/programs/ron/gilt
W=$G/.synth/standalone; mkdir -p $W && cd $W && rm -f *.cf
source $ROOT/videomancer-sdk/build/oss-cad-suite/environment 2>/dev/null
F="$S/common/rtl/core_config/hd_analog_pkg.vhd $S/common/rtl/video_timing/video_timing_pkg.vhd $S/common/rtl/video_sync/video_sync_pkg.vhd $S/common/rtl/video_stream/video_stream_pkg.vhd $S/core/yuv444_30b/rtl/core_pkg.vhd $S/core/yuv444_30b/rtl/program_top.vhd $G/gilt_pkg.vhd $G/gilt.vhd"
timeout 1200 yosys -m ghdl -p "ghdl --std=08 $F -e program_top; synth_ice40 -top program_top; stat" > syn.log 2>&1
echo "exit $?"; grep -n "error" syn.log | head -5
sed -n '/Printing statistics/,$p' syn.log | grep -E "SB_LUT4|SB_CARRY|SB_DFF|SB_RAM" | head -9
echo "emulate_transparency: $(grep -c emulate_transparency syn.log)"
