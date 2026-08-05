#!/usr/bin/env python3
"""
MORSEL -- per-frame control maths check.

preview_morsel.py models the PIXEL path, but it computes the per-frame
constants in clean Python ints, so it cannot see width bugs in the vblank
sequencer -- and a still render cannot show them either, because each frame
looks perfectly fine on its own.  That blind spot shipped three real faults:

    v_m20 := resize(shift_left(s_k6, 3) - s_k6, 20);

`shift_left` on an UNSIGNED returns the operand's own width, so the top bits
are thrown away and the whole expression silently became mod 1024.  Chip Size
ramped and wrapped four times across the knob, K2 Shape was pinned at 0 (a
dead control), and K4's specular cycled.  The fix is to resize BEFORE shifting.

This script re-implements each sequencer slot with the VHDL's exact widths and
asserts the panel behaviour a knob is supposed to have: monotonic, full range,
no wrap.  Run it after touching p_frame.
"""

import sys

M10 = 0x3FF
M20 = 0xFFFFF


def u(val, bits):
    """Truncate to `bits`, the way a numeric_std assignment of that width does."""
    return val & ((1 << bits) - 1)


def sl(val, n, bits):
    """shift_left on an unsigned of `bits` -- keeps the width, drops the top."""
    return u(val << n, bits)


# --- the sequencer slots, transcribed with their real widths ---------------
def s_vlo(k2):                                   # slot 63
    v_m20 = u(sl(k2, 2, 20) + k2, 20)            # k2 * 5
    return (v_m20 >> 10) & 0x7


def s_szb(k6):                                   # slot 61
    v_m20 = u(sl(k6, 3, 20) - k6, 20)            # k6 * 7
    return u(20 + ((v_m20 >> 7) & 0x3F), 7)


def s_spec(k4):                                  # slot 3
    v_m20 = u(sl(k4, 2, 20) + k4, 20)            # k4 * 5
    return (v_m20 >> 4) & 0x1FF


C_JMAX = [15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 13, 11, 10, 8, 6, 4,
          3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


def s_jmax(k6):                                  # slot 60
    return C_JMAX[s_szb(k6) >> 2]


def s_dens(p12):                                 # slot 33 (deadband ignored)
    return (p12 >> 2) & 0xFF


def s_scl(k1):                                   # slot 63
    return 0 if k1 < 205 else 1 if k1 < 410 else 2 if k1 < 640 else \
           3 if k1 < 900 else 4


# --- checks ----------------------------------------------------------------
FAILS = []


def check(name, fn, lo_want, hi_want, monotonic=True):
    vals = [fn(k) for k in range(1024)]
    ok = True
    if monotonic:
        for a, b in zip(vals, vals[1:]):
            if b < a:
                FAILS.append("%-12s NOT MONOTONIC: drops %d -> %d" % (name, a, b))
                ok = False
                break
    if vals[0] != lo_want or vals[-1] != hi_want:
        FAILS.append("%-12s RANGE %d..%d, expected %d..%d"
                     % (name, vals[0], vals[-1], lo_want, hi_want))
        ok = False
    span = len(set(vals))
    print("%-12s %4d..%-4d  %3d distinct  %s"
          % (name, vals[0], vals[-1], span, "ok" if ok else "FAIL"))


print("control        range          steps")
print("-" * 46)
check("Chip Size", s_szb, 20, 75)
check("Shape", s_vlo, 0, 4)
check("Specular", s_spec, 0, 319)
check("Chips", s_dens, 0, 255)
check("Scale", s_scl, 0, 4)
# jitter room legitimately falls as chips grow
check("Jitter room", s_jmax, 15, 0, monotonic=False)
vals = [s_jmax(k) for k in range(1024)]
for a, b in zip(vals, vals[1:]):
    if b > a:
        FAILS.append("Jitter room  NOT MONOTONIC DECREASING: rises %d -> %d" % (a, b))
        break

print("-" * 46)
if FAILS:
    for f in FAILS:
        print("FAIL:", f)
    sys.exit(1)
print("all per-frame controls sweep cleanly")
