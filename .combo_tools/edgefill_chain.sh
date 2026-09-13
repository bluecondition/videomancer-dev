#!/bin/bash
# Serial rebuild of the nine edge-fill programs; logs under .combo_tools/ (survives /tmp wipes).
cd /home/ron/videomancer-dev
L=.combo_tools/edgefill
mkdir -p $L
for p in halation parallax etchwarp mirage tremor hueflutter flutter trapquant; do
  echo "$(date +%H:%M:%S) start $p" >> $L/chain.log
  ./build_combo.sh $p > $L/build_$p.log 2>&1
  echo "$(date +%H:%M:%S) build exit $? $p" >> $L/chain.log
done
echo "$(date +%H:%M:%S) chain done" >> $L/chain.log
