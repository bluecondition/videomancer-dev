#!/bin/bash
# After the edge-fill chain: rebuild trapquant (timing rework v0.1.3) with the same logging scheme.
cd /home/ron/videomancer-dev
L=.combo_tools/edgefill
until grep -q "chain done" $L/chain.log 2>/dev/null; do sleep 30; done
for p in halation; do
  echo "$(date +%H:%M:%S) start rebuild $p" >> $L/chain.log
  ./build_combo.sh $p > $L/rebuild_$p.log 2>&1
  echo "$(date +%H:%M:%S) rebuild exit $? $p" >> $L/chain.log
done
echo "$(date +%H:%M:%S) after done" >> $L/chain.log
