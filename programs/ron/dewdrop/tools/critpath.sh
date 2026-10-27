#!/bin/bash
# Summarize the critical path of the newest nextpnr make log (builder's mktemp in /tmp),
# or of the log given as $1.  Prints Fmax lines, the top signal names on the path, and
# the logic/routing split.
f=${1:-$(ls -t /tmp/tmp.* 2>/dev/null | head -1)}
echo "log: $f"
grep -a "ICESTORM_LC:\|Max frequency for clock" "$f" | tail -4
awk '/Critical path report/,0' "$f" | grep -a "ns logic" | head -4
echo "--- top names on the critical path ---"
awk '/Critical path report/,0' "$f" | grep -a "Source\|Sink" | head -80 | grep -ao "inst\.[a-zA-Z0-9_]*" | sed 's/inst\.//; s/_SB_.*//; s/\[.*//' | sort | uniq -c | sort -rn | head -12
