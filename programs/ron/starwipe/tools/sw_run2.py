#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Run the emulated frame program and render through the scanline model."""
import numpy as np
import sw_ucode2 as uc, sw_uemu as ue, sw_scan as sc, sw_frame2 as fr
from sw_uasm import Asm

_W = None


def words():
    global _W
    if _W is None:
        _W = uc.build().words()
    return _W


def ctl_inputs(mode, ctl, field=0):
    M = fr.MODES[mode]
    sw = ctl['s7'] | ctl['s8'] << 1 | ctl['s9'] << 2 | ctl['s10'] << 3 | ctl['s11'] << 4
    return dict(k1=ctl['k1'], k2=ctl['k2'], k3=ctl['k3'], k4=ctl['k4'], k5=ctl['k5'], k6=ctl['k6'], sw=sw,
                p12=ctl['p12'], tid=M['tid'], W=M['W'], H=M['HF'] // 2 if M['ilace'] else M['HF'], fld=field, set=0)


def settle(eng, inp, A):
    ts, th = A.r('Ts'), A.r('Th')
    for _ in range(80):
        clk = eng.run_frame(inp)
        if eng.rf[ts] == eng.rf[th] and eng.rf[th] == (inp['p12'] << 6):
            break
    return eng.run_frame(inp)


def bands_from(eng, P, acc, rset=0):
    """records in DATA -> sw_scan Band for accumulator acc (0/1)"""
    pre = 'LA_' if acc == 0 else 'LB_'
    base0 = uc.REC0 + uc.REC_STRIDE * rset
    up, dn, ay = P.get(pre + 'UP', 0), P.get(pre + 'DN', 0), P.get(pre + 'AY', 0)
    q = ue.s32(P.get('QA' if acc == 0 else 'QB', 0))
    def lst(desc):
        base, cnt = desc >> 8, desc & 255             # base = record ADDRESS
        out = []
        for i in range(cnt):
            w = [ue.s32(eng.data[base + 7 * i + j]) for j in range(7)]
            out.append(sc.Ray(w[0], w[1], w[2], w[3], w[4], w[5], w[6] & 0xFFFFFFFF, i == 0))
        return out
    return sc.Band(ay, lst(up), lst(dn), 1 << 30), q


def render(mode, ctl, A_img, eng=None, Aasm=None):
    M = fr.MODES[mode]
    Aasm = Aasm or uc.build()
    eng = eng or ue.Engine(words())
    out = np.zeros_like(A_img)
    fields = (0, 1) if M['ilace'] else (0,)
    nl = M['HF'] // 2 if M['ilace'] else M['HF']
    info = {}
    for f in fields:
        clk = settle(eng, ctl_inputs(mode, ctl, f), Aasm)
        P = eng.ports
        mode_ = P.get('MODE', 0)
        rows = slice(f, None, 2) if M['ilace'] else slice(None)
        BC = (P['BCY'] & 1023, (P['BCUV'] >> 10) & 1023, P['BCUV'] & 1023)
        RC = (P['RCY'] & 1023, (P['RCUV'] >> 10) & 1023, P['RCUV'] & 1023)
        if mode_ & 4:
            k = P['TK']
            B = np.array(BC)
            out[rows] = A_img[rows] + (((B - A_img[rows]) * k + 128) >> 8)
        else:
            bA, qa = bands_from(eng, P, 0)
            DA, _ = sc.render_fields(sc.Frame([bA], qa=qa), M['W'], nl)
            op = mode_ & 3
            if op:
                bB, qb = bands_from(eng, P, 1)
                DB, _ = sc.render_fields(sc.Frame([bB], qa=qb), M['W'], nl)
                DA = np.minimum(DA, DB) if op == 1 else np.maximum(DA, DB)
            out[rows] = sc.compose(A_img[rows], DA, P['DK'], BC, RC, rev=bool(mode_ & 8))
        info[f] = dict(clk=clk, P=dict(P))
    return sc.pack422(out), info
