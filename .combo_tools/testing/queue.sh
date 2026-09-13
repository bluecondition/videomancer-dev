#!/bin/bash
# Serial build runner for programs under test. Appends to queue.txt add work; each name is built once
# (./build_combo.sh <name>, log build_<name>.log) in file order. Exits when queue.txt contains "END" and all are built.
cd /home/ron/videomancer-dev
Q=.combo_tools/testing/queue.txt; D=.combo_tools/testing
touch $Q
while true; do
  next=""
  while read -r n; do
    [ -z "$n" ] && continue
    [ "$n" = "END" ] && { [ -z "$next" ] && { echo "$(date +%T) queue END" >> $D/chain.log; exit 0; }; break; }
    [ -e "$D/build_$n.log" ] && continue
    next="$n"; break
  done < $Q
  if [ -z "$next" ]; then sleep 20; continue; fi
  echo "$(date +%T) start $next" >> $D/chain.log
  ./build_combo.sh "$next" > "$D/build_$next.log" 2>&1
  echo "$(date +%T) done  $next rc=$? vmprog=$(ls out/rev_b/ron/catalogue-combo/testing/$next.vmprog 2>/dev/null | wc -l)" >> $D/chain.log
done
