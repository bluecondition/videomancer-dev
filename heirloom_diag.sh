#!/bin/bash
# Heirloom phase-2 preview copies (Ron 2026-11-01): first-round changes go into a diag copy so the
# production program stays intact until Ron previews and approves it.
#
#   ./heirloom_diag.sh new   <name>   copy programs/ron/heirloom/<name> to .heirloom_work/rb_<name>/diag/<name>_diag
#                                     (own program_id "...<name>_diag" and name "<Name> Diag", so it loads beside the original)
#   ./heirloom_diag.sh build <name>   build it -> out/rev_b/ron/_diag/<name>_diag_v<version>.vmprog
#   ./heirloom_diag.sh merge <name>   after approval: copy it back over programs/ron/heirloom/<name>
#                                     (id and name restored; the diag copy is kept)
set -u
cd "$(dirname "$0")"
cmd="${1:-}"; n="${2:-}"
[ -n "$cmd" ] && [ -n "$n" ] || { sed -n 2,9p "$0"; exit 1; }
prod="programs/ron/heirloom/$n"; diag=".heirloom_work/rb_$n/diag/${n}_diag"
[ -d "$prod" ] && [ "$n" != "photos" ] || { echo "no such heirloom program: $n"; exit 1; }
name=$(sed -n 's/^program_name = "\(.*\)"/\1/p' "$prod/$n.toml" | sed 's/ Diag$//')

case "$cmd" in
new)
    [ -e "$diag" ] && { echo "$diag already exists; refusing"; exit 1; }
    mkdir -p "$diag"
    cp -p "$prod"/* "$diag/"
    mv "$diag/$n.toml" "$diag/${n}_diag.toml"
    [ -f "$diag/$n.py" ] && mv "$diag/$n.py" "$diag/${n}_diag.py"
    sed -i -e "s/^program_id = \"\(.*\)\.$n\"/program_id = \"\1.${n}_diag\"/" \
           -e "s/^program_name = \"$name\"/program_name = \"$name Diag\"/" "$diag/${n}_diag.toml"
    grep -n "^program_id\|^program_name" "$diag/${n}_diag.toml"
    echo "diag copy: $diag"
    ;;
build)
    [ -d "$diag" ] || { echo "no diag copy at $diag (run: $0 new $n)"; exit 1; }
    [ -e "programs/ron/${n}_diag" ] && { echo "programs/ron/${n}_diag already exists; refusing"; exit 1; }
    ver=$(sed -n 's/^program_version = "\(.*\)"/\1/p' "$diag/${n}_diag.toml")
    ln -s "../../$diag" "programs/ron/${n}_diag"
    ./build_programs.sh ron "${n}_diag"; rc=$?
    rm "programs/ron/${n}_diag"
    if [ -f "out/rev_b/ron/${n}_diag.vmprog" ]; then
        mkdir -p out/rev_b/ron/_diag
        mv "out/rev_b/ron/${n}_diag.vmprog" "out/rev_b/ron/_diag/${n}_diag_v$ver.vmprog"
        echo "preview: out/rev_b/ron/_diag/${n}_diag_v$ver.vmprog"
    fi
    exit $rc
    ;;
merge)
    [ -d "$diag" ] || { echo "no diag copy at $diag"; exit 1; }
    for f in "$diag"/*; do
        [ -f "$f" ] || continue
        b=$(basename "$f")
        case "$b" in
            "${n}_diag.toml") cp -p "$f" "$prod/$n.toml" ;;
            "${n}_diag.py")   cp -p "$f" "$prod/$n.py" ;;
            *)                cp -p "$f" "$prod/$b" ;;
        esac
    done
    sed -i -e "s/^program_id = \"\(.*\)\.${n}_diag\"/program_id = \"\1.$n\"/" \
           -e "s/^program_name = \"$name Diag\"/program_name = \"$name\"/" "$prod/$n.toml"
    rm -f "build/programs/ron/$n/program_config.bin" "build/programs/ron/$n/rev_b/program_config.bin"
    grep -n "^program_id\|^program_name\|^program_version" "$prod/$n.toml"
    echo "merged into $prod; now ./build_heirloom.sh $n, update SUMMARY/README/PHASE2 and commit"
    ;;
*)
    sed -n 2,9p "$0"; exit 1 ;;
esac
