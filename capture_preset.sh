#!/bin/bash
# Capture the Videomancer's live knob/switch/slider values into a program TOML.
#
#   ./capture_preset.sh --preset N [--name "Name"] [--dry-run]
#   ./capture_preset.sh --defaults [--dry-run]
#
# Wraps tools/capture_preset.py -- a patched local copy of LZX's
# videomancer-sdk/scripts/capture_preset.py (upstream 606b668), kept out of
# the SDK so the submodule stays stock:
#   - runs it with .tools-venv (pyserial installed there)
#   - searches programs/ron (including catalogue-combo, skipping _archive)
#     for the TOML whose program_id matches the program loaded on the device
#   - after a real write, deletes that program's cached program_config.bin
#     so the next build actually carries the new presets/defaults to hardware
#
# USB: if the Videomancer is not already attached to WSL, this attaches it
# (usbipd-win), captures, then detaches it again so Windows gets its USB
# drive / MIDI back.  Set KEEP_ATTACHED=1 to leave it attached.
# Any other capture_preset.py option (--toml, --port, --timeout) passes through.

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="$ROOT/.tools-venv/bin/python"
SCRIPT="$ROOT/tools/capture_preset.py"   # patched local copy; SDK stays stock

if [ ! -x "$PY" ]; then
    echo "error: $PY missing. Recreate it with:" >&2
    echo "  /usr/bin/python3 -m venv $ROOT/.tools-venv && $ROOT/.tools-venv/bin/pip install pyserial" >&2
    exit 1
fi

USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"
HWID="16d0:14db"
attached_here=0
[ -n "${KEEP_ATTACHED_PARENT:-}" ] && attached_here=1
detach() {
    if [ "$attached_here" = 1 ] && [ "${KEEP_ATTACHED:-0}" != 1 ]; then
        (cd /mnt/c && "$USBIPD" detach --hardware-id "$HWID" >/dev/null 2>&1) \
            && echo "Videomancer handed back to Windows"
    fi
}
trap detach EXIT
if ! ls /dev/serial/by-id/*Videomancer* >/dev/null 2>&1 && ! ls /dev/ttyACM* >/dev/null 2>&1; then
    echo "attaching Videomancer to WSL..."
    (cd /mnt/c && "$USBIPD" attach --wsl --hardware-id "$HWID" 2>&1 | tr -d '\r' | grep -v '^usbipd: info') || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do ls /dev/ttyACM* >/dev/null 2>&1 && break; sleep 0.5; done
    attached_here=1
fi
for t in /dev/ttyACM*; do
    if [ -e "$t" ] && [ ! -r "$t" -o ! -w "$t" ]; then
        # Already added to dialout but this shell predates it (e.g. an open
        # Claude session): re-run through sg, which reads /etc/group fresh.
        if [ -z "${CAPTURE_SG:-}" ] && getent group dialout | grep -qw "$(id -un)"; then
            trap - EXIT
            [ "$attached_here" = 1 ] && export KEEP_ATTACHED_PARENT=1
            CAPTURE_SG=1 exec sg dialout -c "$(printf '%q ' "$0" "$@")"
        fi
        echo "error: no permission on $t. One-time fix (in a WSL terminal), then open a new terminal:" >&2
        echo "  sudo usermod -aG dialout \$USER" >&2
        exit 1
    fi
done

dirs=()
for d in "$ROOT"/programs/ron/*/; do
    case "$(basename "$d")" in _archive) continue ;; esac
    dirs+=(--programs-dir "$d")
done

out="$("$PY" "$SCRIPT" "${dirs[@]}" "$@" 2>&1)" && rc=0 || rc=$?
echo "$out"
[ "$rc" -eq 0 ] || exit "$rc"

written="$(printf '%s\n' "$out" | sed -n 's/^wrote //p')"
if [ -n "$written" ]; then
    prog="$(basename "$(dirname "$written")")"
    for f in "$ROOT/build/programs/ron/$prog/program_config.bin" \
             "$ROOT/build/programs/ron/$prog/rev_b/program_config.bin"; do
        [ -f "$f" ] && rm -f "$f" && echo "cleared stale ${f#$ROOT/}"
    done
    if [[ "$written" == */catalogue-combo/* ]]; then
        echo "rebuild:  ./build_combo.sh $prog"
    else
        echo "rebuild:  ./build_programs.sh ron $prog"
    fi
fi
