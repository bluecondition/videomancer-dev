#!/bin/bash
# Rebuild one catalogue-combo program: programs/ron/catalogue-combo/<name> -> out/rev_b/ron/catalogue-combo/<name>.vmprog
# Programs under test live in programs/ron/catalogue-combo/testing/<name> and pack to out/rev_b/ron/catalogue-combo/testing/.
# build_programs.sh only knows programs/<vendor>/<program>, so link the dir in temporarily.
set -u
cd "$(dirname "$0")"
n="$1"; src="programs/ron/catalogue-combo/$n"; dst="out/rev_b/ron/catalogue-combo"
if [ ! -d "$src" ] && [ -d "programs/ron/catalogue-combo/testing/$n" ]; then src="programs/ron/catalogue-combo/testing/$n"; dst="$dst/testing"; fi
[ -d "$src" ] || { echo "no such combo program: $n"; exit 1; }
[ -e "programs/ron/$n" ] && { echo "programs/ron/$n already exists; refusing"; exit 1; }
ln -s "${src#programs/ron/}" "programs/ron/$n"
./build_programs.sh ron "$n"; rc=$?
rm "programs/ron/$n"
if [ -f "out/rev_b/ron/$n.vmprog" ]; then mkdir -p "$dst"; mv "out/rev_b/ron/$n.vmprog" "$dst/"; fi
exit $rc
