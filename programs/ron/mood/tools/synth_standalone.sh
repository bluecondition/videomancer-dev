#!/bin/bash
# Standalone yosys synthesis of MOOD (program only, no core) for a quick
# syntax / size check.  Usage: tools/synth_standalone.sh [clean]
# Prints LUT4 / carry / FF / RAM counts and the emulate_transparency count.
ROOT=/home/ron/videomancer-dev; S=$ROOT/videomancer-sdk/fpga; P=$ROOT/programs/ron/mood
GEN=false; TAG=vintage
if [ "$1" = clean ]; then GEN=true; TAG=clean; fi
W=$HOME/.mood_work/standalone_$TAG; mkdir -p $W && cd $W && rm -f *.cf
source $ROOT/videomancer-sdk/build/oss-cad-suite/environment 2>/dev/null
sed "s/G_CLEAN => false/G_CLEAN => $GEN/" $P/mood.vhd > $W/top.vhd
F="$S/common/rtl/core_config/hd_hdmi_pkg.vhd $S/common/rtl/video_timing/video_timing_pkg.vhd $S/common/rtl/video_sync/video_sync_pkg.vhd $S/common/rtl/video_stream/video_stream_pkg.vhd $S/core/yuv444_30b/rtl/core_pkg.vhd $S/core/yuv444_30b/rtl/program_top.vhd $P/mood_pkg.vhd $P/mood_core.vhd $W/top.vhd"
timeout 1200 yosys -m ghdl -p "ghdl --std=08 $F -e program_top; synth_ice40 -top program_top; stat" > syn.log 2>&1
echo "exit $?"; grep -n -i "error" syn.log | head -8
sed -n '/Printing statistics/,$p' syn.log | grep -E "SB_LUT4|SB_CARRY|SB_DFF|SB_RAM" | head -12
echo "emulate_transparency: $(grep -c emulate_transparency syn.log)"
