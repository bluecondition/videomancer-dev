#!/bin/bash
# Summarise the latest build log per program: HD Fmax (analog/hdmi/dual), LC, seeds, vmprog presence
cd /home/ron/videomancer-dev
for p in "$@"; do
  f=$(ls -t .${p}_work/build*.log 2>/dev/null | head -1)
  [ -z "$f" ] && { echo "$p: no log"; continue; }
  # config order: HD Analog, SD Analog, HD HDMI, SD HDMI, HD Dual, SD Dual
  mapfile -t L < <(sed 's/\x1b\[[0-9;]*m//g' "$f" | grep -a "Completed" | sed -E 's/.*\(seed ([0-9]+)\) - Fmax: ([0-9.]+) MHz, LCs: ([0-9]+).*/\2@\1 \3/')
  hd=""; for i in 0 2 4; do v=${L[$i]:-"-"}; hd="$hd ${v%% *}"; done
  lc=$(echo "${L[0]}" | awk '{print $2}')
  vm=$(ls out/rev_b/ron/catalogue-combo/$p.vmprog 2>/dev/null | wc -l)
  status="ok"; for i in 0 2 4; do v=${L[$i]:-""}; fm=${v%%@*}; sd=${v##*@}; sd=${sd%% *}; if [ -z "$v" ]; then status="incomplete"; elif [ "$sd" = "6" ] && awk "BEGIN{exit !($fm < 74.25)}"; then status="MISS"; fi; done
  printf "%-11s HD(an/hdmi/dual)=%-32s LC=%-5s vmprog=%s %s\n" "$p" "$hd" "$lc" "$vm" "$status"
done
