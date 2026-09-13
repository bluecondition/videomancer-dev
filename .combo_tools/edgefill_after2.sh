#!/bin/bash
# After edgefill_after.sh: rebuild impasto (relief-product split v0.1.2).
cd /home/ron/videomancer-dev
L=.combo_tools/edgefill
until grep -q "after done" $L/chain.log 2>/dev/null; do sleep 30; done
for p in impasto; do
  echo "$(date +%H:%M:%S) start rebuild $p" >> $L/chain.log
  ./build_combo.sh $p > $L/rebuild_$p.log 2>&1
  echo "$(date +%H:%M:%S) rebuild exit $? $p" >> $L/chain.log
done
echo "$(date +%H:%M:%S) after2 done" >> $L/chain.log
