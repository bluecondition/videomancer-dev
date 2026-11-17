#!/bin/bash
# Rebuild one program of a sub-folder set: programs/ron/<set>/<name> -> out/rev_b/ron/<set>/<name>.vmprog
#   ./build_set.sh kinetic riley        ./build_set.sh patchbay <name>        ./build_set.sh hardline <name>        ./build_set.sh vellum <name>        ./build_set.sh straightedge <name>
# build_programs.sh only knows programs/<vendor>/<program>, so link the dir in temporarily (as build_heirloom.sh does).
set -u
cd "$(dirname "$0")"
s="${1:?usage: build_set.sh <set> <name>}"; n="${2:?usage: build_set.sh <set> <name>}"
case "$s" in kinetic|patchbay|hardline|vellum|straightedge) ;; *) echo "unknown set: $s (kinetic | patchbay | hardline | vellum | straightedge)"; exit 1;; esac
src="programs/ron/$s/$n"; dst="out/rev_b/ron/$s"
[ -f "$src/$n.vhd" ] || { echo "no such $s program: $n"; exit 1; }
[ -e "programs/ron/$n" ] && { echo "programs/ron/$n already exists; refusing"; exit 1; }
ln -s "$s/$n" "programs/ron/$n"
./build_programs.sh ron "$n"; rc=$?
rm "programs/ron/$n"
if [ -f "out/rev_b/ron/$n.vmprog" ]; then mkdir -p "$dst"; mv "out/rev_b/ron/$n.vmprog" "$dst/"; fi
exit $rc
