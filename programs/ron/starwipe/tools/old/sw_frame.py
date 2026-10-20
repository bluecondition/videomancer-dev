#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE frame model: the microcode engine (emulated) computes each field's
ports, then the RTL's raster DDA and sw_pixel's pipeline render every pixel
(both fields interleaved under interlace), through the core's 4:2:2 packer.
"""
import math
import numpy as np
import sw_pixel as px

# ---------------------------------------------------------------- video modes
# sy8 = height of one FRAME line in u-units, Q.8 (u-unit = 1/16 h-pixel):
# square pixels 16; NTSC lines are 11/10 h-px tall, PAL 11/12 (Rec.601 PAR).
MODES = {
    '1080p': dict(W=1920, HF=1080, ilace=False, sy8=4096, tid=13),
    '720p':  dict(W=1280, HF=720,  ilace=False, sy8=4096, tid=14),
    '480i':  dict(W=720,  HF=486,  ilace=True,  sy8=4506, tid=0),
    '576i':  dict(W=720,  HF=576,  ilace=True,  sy8=3755, tid=8),
}

# ---------------------------------------------------------------- render
def raster_uv(mode, P, field):
    """the RTL's raster DDA for one field: arrays u, v (lines x W)"""
    M = MODES[mode]
    W, HF = M['W'], M['HF']
    nl = HF // 2 if M['ilace'] else HF
    su, sv = P['SEED'][field]
    UL = np.int64(su) << 8
    VL = np.int64(sv) << 8
    x = np.arange(W, dtype=np.int64)
    lines = np.arange(nl, dtype=np.int64)
    Ul = UL + lines * P['DUY']
    Vl = VL + lines * P['DVY']
    U = Ul[:, None] + x[None, :] * P['DUX']
    V = Vl[:, None] + x[None, :] * P['DVX']
    return U >> 10, V >> 10                      # key units (accumulators Q.10)


# ---------------------------------------------------------------- microcode-driven
import sw_ucode as uc
import sw_uemu as ue

CLASSIC, CHEESE, STAR_RHO = uc.CLASSIC, uc.CHEESE, uc.STAR_RHO

_WORDS = None


def words():
    global _WORDS
    if _WORDS is None:
        _WORDS = uc.build().words()
    return _WORDS


def ctl_inputs(mode, ctl, field=0):
    M = MODES[mode]
    sw = ctl['s7'] | ctl['s8'] << 1 | ctl['s9'] << 2 | ctl['s10'] << 3 | ctl['s11'] << 4
    return dict(k1=ctl['k1'], k2=ctl['k2'], k3=ctl['k3'], k4=ctl['k4'], k5=ctl['k5'],
                k6=ctl['k6'], sw=sw, p12=ctl['p12'], tid=M['tid'], W=M['W'],
                H=M['HF'] // 2 if M['ilace'] else M['HF'], fld=field)


def emu_params(mode, ctl, eng=None, settle=True, field=0):
    """run the microcode for this frame (settled: until the T glide has
    landed and the slot hysteresis is stable); returns (ports dict, clocks)"""
    from sw_uasm import Asm
    eng = eng or ue.Engine(words())
    inp = ctl_inputs(mode, ctl, field)
    clk = eng.run_frame(inp)
    if settle:
        A = uc.build()
        ts, th = A.r('Ts'), A.r('Th')
        for _ in range(80):
            if eng.rf[ts] == eng.rf[th] and eng.rf[th] == (inp['p12'] << 6):
                break
            clk = eng.run_frame(inp)
        clk = eng.run_frame(inp)
    return ue.port_dict(eng.ports), clk


def render_emu(mode, ctl, A_img, eng=None):
    M = MODES[mode]
    out = np.zeros_like(A_img)
    fields = (0, 1) if M['ilace'] else (0,)
    for f in fields:
        P, clk = emu_params(mode, ctl, eng, field=f)
        u, v = raster_uv(mode, P, 0)
        D = px.field(u, v, P)
        ki, ko = px.keys(D, P)
        rows = slice(f, None, 2) if M['ilace'] else slice(None)
        out[rows] = px.compose(A_img[rows], ki, ko, P)
    return px.pack422(out), P, clk
