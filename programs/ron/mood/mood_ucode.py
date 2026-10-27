#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of Videomancer Community Programs.
# See LICENSE file in the repository root for full license text.
#
"""MOOD microcode: the virtual-time sample program.

Runs once per virtual line (15.734 kHz, one sample tick), from PC 0 to END:

  1. SOURCE  INTERNAL: the phantom stereo record (kick, two bass walks, a
             panned mid partial, hats, snare noise) as real waveforms, mixed
             at half scale (no wrap); the detector gain makes it up.
             EXTERNAL (Path B): audio(t) = level(t) x carrier(t), level = the
             modulated K1/K2 register, carrier pitch = level activity.
  2. DETECT  per channel: x' = x * G >> 12, G = VOLUME x GAIN >> 7 (0..8176),
             squaring circuit with
             hysteresis -> rising zero crossings, rectifier -> one-pole
             integrator (CONTOUR) = ENERGY -> E1/E2 ports (one ring entry).
  3. VIRTUAL FRAME (every 262 samples): the alternating zero-crossing
             counter latches into its channel's 6-bit register, then a
             chain of frame tasks (one per following sample) recomputes
             mode, gains, contours, both colours and the CLEAN AA constants.
             AUTO's check runs mid-frame (the live counter is half full).
  4. ROTATION: one of eight slow tasks per sample (envelopes, sequencer
             steps, Path B level-reversal detectors), so each runs at ~2 kHz.

Formats: audio samples signed 16-bit; ENERGY e in sample units, displayed as
Q5.8 reference units (e * 2.5 / 256, clamped to 32.0); angles 65536 per turn;
sines Q15.  The register file powers up with the assembler's init values.
"""
from mood_uasm import Asm, DEPTH, raw
import mood_tables as T

B = T.BANK

# hue: bass (DL16) red at centre COLOR; hue(n) = PHI0 - n * theta in the
# U/V-plane angle (a longer tap = more phase lag)
PHI0 = round((108.7 + 16 * 18.0) / 360 * 65536) & 0xFFFF
assert PHI0 < 8192
THETA_MIN = round(6 / 360 * 65536)      # 1092: 6 deg per stage at CCW
THETA_SPAN = round(24 / 360 * 65536)    # +24 deg at CW (30 deg per stage)
HYS = 64                                # squaring-circuit hysteresis (x' units)
E_MAX = 8191                            # 31.99 reference units, Q5.8
REV_DB = 8                              # Path B reversal deadband (register LSBs)
REV_K = 256                             # activity per reversal
ACT_SH = 5                              # count = act >> 5 (~8 counts per rev/s)
CARRIER_INC = 250                       # 60 Hz per count, Q16 phase / sample
KF0 = T.inc(150)                        # kick pitch start
KFL = T.inc(48)                         # kick pitch floor


def build():
    A = Asm()
    e = A.emit
    C = A.const
    R = A.r

    # ---- persistent state (power-up values) ----
    R('zero', 0)
    R('vfc', 261)                 # virtual-frame countdown
    R('par', 0)                   # 0: this virtual frame counts ch1, 1: ch2
    R('cm1', 1); R('cm2', 0)      # count masks
    R('slen', 2100)               # samples per 16th note (~112 BPM)
    R('stgt', 2100)
    R('sc', 0)
    R('stp', 15); R('bar', 0); R('plen', 8); R('arr', 0); R('arr16', 0)
    R('lk', T.LEVELS[0][0]); R('lb', T.LEVELS[0][1])
    R('lm', T.LEVELS[0][2]); R('lh', T.LEVELS[0][3])
    R('nL', 4); R('nR', 6)
    R('bfL', T.inc(T.BASS_HZ[4])); R('bfR', T.inc(T.BASS_HZ[6]))
    R('mf', T.inc(T.MID_HZ[5]))
    R('kf', KFL)
    R('kzone', 0)
    R('cf1', CARRIER_INC); R('cf2', CARRIER_INC)
    R('ex1', 0); R('ex2', 0); R('dir1', 0); R('dir2', 0)
    R('g1', 3600); R('g2', 3600)
    R('a1', 1000); R('a2', 1000)
    R('pinc', 9)                  # mid pan LFO (per frame)

    def ldt(dst, v):
        """a rarely used constant: LDI into a temporary (saves a register)"""
        e('LDI', dst, imm=v)

    # =================================================================
    # 1. SOURCE
    # =================================================================
    e('CTL', 'sw', imm='sw')
    e('AND', 't', 'sw', imm=16)
    e('JNZ', a='t', imm='ext')

    # ---- INTERNAL: phantom voices (multiplies interleaved with adds) ----
    e('ADD', 'kph', 'kph', 'kf')
    e('SIN', 'u1', 'kph')
    e('MUL', a='u1', b='ka')
    e('ADD', 'bphL', 'bphL', 'bfL')
    e('ADD', 'bphR', 'bphR', 'bfR')
    e('SIN', 'u2', 'bphL')
    e('MRD', 'u3', imm=16)                          # kick
    e('MUL', a='u2', b='baL')
    e('SIN', 'u1', 'bphR')
    e('ADD', 'mph', 'mph', 'mf')
    e('MRD', 'xL', imm=16)
    e('MUL', a='u1', b='baR')
    e('SIN', 'u2', 'mph')
    e('ADD', 'xL', 'xL', 'u3')
    e('MRD', 'xR', imm=16)
    e('MUL', a='u2', b='maL')
    e('ADD', 'xR', 'xR', 'u3')
    e('CTL', 'nz', imm='rnd')
    e('MRD', 't', imm=16)
    e('MUL', a='u2', b='maR')
    e('ADD', 'xL', 'xL', 't')
    e('MRD', 't', imm=16)
    e('MUL', a='nz', b='haL')
    e('ADD', 'xR', 'xR', 't')
    e('MRD', 't', imm=16)
    e('MUL', a='nz', b='haR')
    e('ADD', 'xL', 'xL', 't')
    e('CTL', 'nz', imm='rnd')
    e('MRD', 't', imm=16)
    e('MUL', a='nz', b='sn')
    e('ADD', 'xR', 'xR', 't')
    e('MRD', 't', imm=16)
    e('ADD', 'xL', 'xL', 't')
    e('ADD', 'xR', 'xR', 't')
    e('JMP', imm='detect')

    # ---- EXTERNAL (Path B): level x carrier, level = modulated K1/K2 ----
    A.label('ext')
    e('CTL', 'l1', imm='k1')
    e('ADD', 'cph1', 'cph1', 'cf1')
    e('SIN', 'u1', 'cph1')
    e('MUL', a='u1', b='l1')
    e('CTL', 'l2', imm='k2')
    e('ADD', 'cph2', 'cph2', 'cf2')
    e('SIN', 'u2', 'cph2')
    e('MRD', 'xL', imm=11)
    e('MUL', a='u2', b='l2')
    e('MRD', 'xR', imm=11)

    # =================================================================
    # 2. DETECT (both channels)
    # =================================================================
    A.label('detect')
    for ch, x, g, st, en, am, port, cm in (
            (1, 'xL', 'g1', 'st1', 'e1', 'a1', 'E1', 'cm1'),
            (2, 'xR', 'g2', 'st2', 'e2', 'a2', 'E2', 'cm2')):
        zd, hi, rnd_skip, eok, elo = (A.lbl() for _ in range(5))
        e('MUL', a=x, b=g)
        e('MRD', 'u1', imm=12)                      # x' = x * G >> 12
        # squaring circuit with hysteresis; count rising crossings
        e('JNZ', a=st, imm=hi)
        e('JLT', a='u1', b=C(HYS), imm=zd)
        e('LDI', st, imm=1)
        e('ADD', 'cz', 'cz', cm)
        e('JMP', imm=zd)
        A.label(hi)
        e('JGE', a='u1', b=C(-HYS), imm=zd)
        e('MOV', st, 'zero')
        A.label(zd)
        # detector + integrator: e += (|x'| - e) * a, stochastic rounding
        e('ABS', 'u2', 'u1')
        e('SUB', 'u2', 'u2', en)
        e('MUL', a='u2', b=am)
        e('CTL', 'nz', imm='rnd')
        e('MRD', 'u3', imm=16)
        e('MRD', 'u4', imm=0)
        e('ADD', en, en, 'u3')
        e('ADD', 'u4', 'u4', C(-32768))             # unsigned fraction -> signed
        e('JLT', a='u4', b='nz', imm=rnd_skip)
        e('ADD', en, en, C(1))
        A.label(rnd_skip)
        # ENERGY out, Q5.8 = e * 2.5 / 256 reference units, 0..31.99
        e('JLT', a=en, b='zero', imm=elo)
        e('JLT', a=en, b=C(E_MAX * 2 // 5), imm=eok)
        e('PRT', a=C(E_MAX), imm=port)
        e('JMP', imm=f'det{ch}_end')
        A.label(elo)
        e('PRT', a='zero', imm=port)
        e('JMP', imm=f'det{ch}_end')
        A.label(eok)
        e('SHR', 'u1', en, imm=1)
        e('ADD', 'u1', 'u1', en)
        e('ADD', 'u1', 'u1', en)
        e('PRT', a='u1', imm=port)
        A.label(f'det{ch}_end')

    # =================================================================
    # 3. VIRTUAL FRAME bookkeeping
    # =================================================================
    e('SUB', 'vfc', 'vfc', C(1))
    e('JLT', a='vfc', b='zero', imm='vwrap')
    ldt('t', 131)
    e('SUB', 't', 'vfc', 't')
    e('JNZ', a='t', imm='rot')                      # mid-frame: AUTO check
    e('LDI', 'tptr', imm='t_auto')
    e('JMP', imm='rot')
    A.label('vwrap')
    e('LDI', 'vfc', imm=T.VFRAME - 1)
    e('LDI', 'tptr', imm='f_latch')

    # =================================================================
    # 4. ROTATION: one slow task per sample, then the frame task, END
    # =================================================================
    A.label('rot')
    e('JR', a='rptr')

    A.label('t_next')                               # every task ends here
    e('JR', a='tptr')
    A.label('t_idle')
    e('END')

    def rot_end(nxt):
        e('LDI', 'rptr', imm=nxt)
        e('JMP', imm='t_next')

    def decay(reg, sh):
        """reg -= (reg >> sh) + 1, floored at 0 (per 8 samples)"""
        ok = A.lbl()
        e('SHR', 't', reg, imm=sh)
        e('SUB', reg, reg, 't')
        e('SUB', reg, reg, C(1))
        e('JGE', a=reg, b='zero', imm=ok)
        e('MOV', reg, 'zero')
        A.label(ok)

    # R0: kick amplitude decay (~65 ms) and pitch drop (~16 ms)
    A.label('r0')
    decay('ka', 7)
    ldt('t2', KFL)
    e('SUB', 't', 'kf', 't2')
    e('SHR', 't', 't', imm=5)
    e('SUB', 'kf', 'kf', 't')
    rot_end('r1')

    # R1: bass decays (~130 ms)
    A.label('r1')
    decay('baL', 8)
    decay('baR', 8)
    rot_end('r2')

    # R2: mid envelopes glide to their targets
    A.label('r2')
    for a_, t_ in (('maL', 'mtL'), ('maR', 'mtR')):
        e('SUB', 't', t_, a_)
        e('SHR', 't', 't', imm=6)
        e('ADD', a_, a_, 't')
    rot_end('r3')

    # R3: hats decay (~16 ms), snare noise (~65 ms)
    A.label('r3')
    decay('haL', 5)
    decay('haR', 5)
    decay('sn', 7)
    rot_end('r4')

    # R4: sequencer
    A.label('r4')
    e('SUB', 'sc', 'sc', C(8))
    e('JGE', a='sc', b='zero', imm='r4_end')
    e('ADD', 'sc', 'sc', 'slen')
    e('ADD', 'stp', 'stp', C(1))
    e('AND', 'stp', 'stp', imm=15)
    e('JNZ', a='stp', imm='step')
    # ---- new bar ----
    e('ADD', 'bar', 'bar', C(1))
    e('SUB', 't', 'stgt', 'slen')                   # tempo glides per bar
    e('SHR', 't', 't', imm=2)
    e('ADD', 'slen', 'slen', 't')
    e('JLT', a='bar', b='plen', imm='bar_mid')
    # ---- new phrase: next arrangement, tempo target, length ----
    e('MOV', 'bar', 'zero')
    e('ADD', 'arr', 'arr', C(1))
    e('AND', 'arr', 'arr', imm=3)
    e('CTL', 'nz', imm='rnd')
    e('AND', 't', 'nz', imm=7)                      # 1 in 8: skip the breakdown
    e('JNZ', a='t', imm='ph_keep')
    e('SUB', 't', 'arr', C(1))
    e('JNZ', a='t', imm='ph_keep')
    e('LDI', 'arr', imm=2)
    A.label('ph_keep')
    e('SHL', 'arr16', 'arr', imm=4)
    e('SHL', 't', 'arr', imm=2)
    for i, lv in enumerate(('lk', 'lb', 'lm', 'lh')):
        e('SIN', lv, 't', imm=raw(B['levels']))
        if i < 3:
            e('ADD', 't', 't', C(1))
    e('AND', 't', 'nz', imm=511)                    # 96..132 BPM
    e('AND', 't2', 'nz', imm=127)
    e('ADD', 't', 't', 't2')
    ldt('t2', 1788)
    e('ADD', 'stgt', 't', 't2')
    e('LDI', 'plen', imm=8)
    e('AND', 't', 'nz', imm=1024)
    e('JNZ', a='t', imm='bar_mid')
    e('LDI', 'plen', imm=16)
    A.label('bar_mid')
    # ---- every bar: a new mid partial ----
    e('CTL', 'nz', imm='rnd')
    e('AND', 't', 'nz', imm=15)
    ldt('t2', 14)
    e('JLT', a='t', b='t2', imm='mid_ok')
    e('AND', 't', 't', imm=7)
    A.label('mid_ok')
    e('SIN', 'mf', 't', imm=raw(B['mid']))
    A.label('step')
    # ---- every 16th: pattern flags -> triggers ----
    e('ADD', 't', 'arr16', 'stp')
    e('SIN', 'fl', 't', imm=raw(B['pattern']))
    # build: the last bar of a build rolls the kick on every 16th and the
    # snare on every 8th (the noise climax: counter rollover flashes)
    ldt('t2', 2)
    e('SUB', 't', 'arr', 't2')
    e('JNZ', a='t', imm='no_roll')
    e('SUB', 't', 'plen', 'bar')
    e('SUB', 't', 't', C(1))
    e('JNZ', a='t', imm='no_roll')
    e('AND', 't', 'fl', imm=T.F_KICK)
    e('JNZ', a='t', imm='roll_snr')
    e('ADD', 'fl', 'fl', C(T.F_KICK))
    A.label('roll_snr')
    e('AND', 't', 'fl', imm=T.F_SNR)
    e('JNZ', a='t', imm='no_roll')
    e('AND', 't', 'stp', imm=1)
    e('JNZ', a='t', imm='no_roll')
    e('ADD', 'fl', 'fl', C(T.F_SNR))
    A.label('no_roll')
    e('AND', 't', 'fl', imm=T.F_KICK)
    e('JNZ', a='t', imm='trig_kick')
    A.label('trig_bass')
    e('CTL', 'nz', imm='rnd')
    for side, flag in (('L', T.F_BASSL), ('R', T.F_BASSR)):
        skip, lo, hi = A.lbl(), A.lbl(), A.lbl()
        e('AND', 't', 'fl', imm=flag)
        e('JNZ', a='t', imm=lo + 'go')
        e('JMP', imm=skip)
        A.label(lo + 'go')
        n = 'n' + side
        if side == 'R':                             # 1 in 4: unison with L
            e('AND', 't', 'nz', imm=0x300)
            e('JNZ', a='t', imm=lo + 'walk')
            e('MOV', n, 'nL')
            e('JMP', imm=hi)
            A.label(lo + 'walk')
            e('SHR', 'nz', 'nz', imm=4)
        e('AND', 't', 'nz', imm=3)                  # walk -3..+3, triangular
        e('AND', 't2', 'nz', imm=12)
        e('SHR', 't2', 't2', imm=2)
        e('SUB', 't', 't', 't2')
        e('ADD', n, n, 't')
        e('JGE', a=n, b='zero', imm=lo)
        e('MOV', n, 'zero')
        A.label(lo)
        ldt('t2', 14)
        e('JLT', a=n, b='t2', imm=hi)
        e('LDI', n, imm=13)
        A.label(hi)
        e('SIN', 'bf' + side, n, imm=raw(B['bass']))
        e('MOV', 'ba' + side, 'lb')
        A.label(skip)
    for side, flag in (('L', T.F_HATL), ('R', T.F_HATR)):
        skip = A.lbl()
        e('AND', 't', 'fl', imm=flag)
        e('JNZ', a='t', imm=skip + 'go')
        e('JMP', imm=skip)
        A.label(skip + 'go')
        e('MOV', 'ha' + side, 'lh')
        e('AND', 't', 'fl', imm=T.F_ACC)
        e('JNZ', a='t', imm=skip)
        e('SHR', 'ha' + side, 'lh', imm=1)
        A.label(skip)
    e('AND', 't', 'fl', imm=T.F_SNR)                # snare: a noise burst on both
    e('JNZ', a='t', imm='snr_go')
    e('JMP', imm='mid_trig')
    A.label('snr_go')
    e('SHL', 'sn', 'lh', imm=1)
    e('ADD', 'sn', 'sn', 'lh')
    A.label('mid_trig')
    e('AND', 't', 'fl', imm=T.F_MID)
    e('JNZ', a='t', imm='mid_go')
    e('JMP', imm='r4_end')
    A.label('mid_go')
    e('MOV', 'mlev', 'lm')
    A.label('r4_end')
    rot_end('r5')
    A.label('trig_kick')
    e('LDI', 'kf', imm=KF0)
    e('MOV', 'ka', 'lk')
    e('JMP', imm='trig_bass')

    # R5 / R6: Path B level-reversal detectors (deadband peak follower)
    for ch, l, ex, dr, act, nxt in ((1, 'l1', 'ex1', 'dir1', 'act1', 'r6'),
                                    (2, 'l2', 'ex2', 'dir2', 'act2', 'r7')):
        down, rev, done = A.lbl(), A.lbl(), A.lbl()
        A.label(f'r{4 + ch}')
        ldt('t2', REV_DB)
        e('JNZ', a=dr, imm=down)
        # rising: track the peak; a fall of REV_DB below it is a reversal
        e('JLT', a=ex, b=l, imm=done + 'mx')
        e('SUB', 't', ex, l)
        e('JLT', a='t', b='t2', imm=done)
        e('LDI', dr, imm=1)
        e('JMP', imm=rev)
        A.label(done + 'mx')
        e('MOV', ex, l)
        e('JMP', imm=done)
        A.label(down)
        e('JLT', a=l, b=ex, imm=done + 'mn')
        e('SUB', 't', l, ex)
        e('JLT', a='t', b='t2', imm=done)
        e('MOV', dr, 'zero')
        A.label(rev)
        e('MOV', ex, l)
        e('ADD', act, act, C(REV_K))
        ldt('t2', 7500)
        e('SHL', 't2', 't2', imm=2)                 # 30000
        e('JLT', a=act, b='t2', imm=done)
        e('MOV', act, 't2')
        e('JMP', imm=done)
        A.label(done + 'mn')
        e('MOV', ex, l)
        A.label(done)
        rot_end(nxt)

    # R7: mid level: slow swell toward the phrase level, then fade
    A.label('r7')
    decay('mlev', 11)
    e('SUB', 't', 'mlev', 'mlv')
    e('SHR', 't', 't', imm=4)
    e('ADD', 'mlv', 'mlv', 't')
    rot_end('r0')

    # =================================================================
    # FRAME TASKS (chained, one per sample after the virtual-frame wrap)
    # =================================================================
    def task_end(nxt):
        e('LDI', 'tptr', imm=nxt)
        e('END')

    # F0: latch the alternating zero-crossing counter (6-bit, wraps)
    A.label('f_latch')
    e('AND', 't', 'cz', imm=63)
    e('JNZ', a='par', imm='lat2')
    e('MOV', 'reg1', 't')
    e('JMP', imm='lat_done')
    A.label('lat2')
    e('MOV', 'reg2', 't')
    A.label('lat_done')
    e('MOV', 'cz', 'zero')
    e('MOV', 'cm2', 'cm1')                          # swap the count masks
    e('SUB', 'cm1', C(1), 'cm1')
    e('SUB', 'par', C(1), 'par')
    e('PRT', a='par', imm='FRM')
    task_end('f_mode')

    # F1: SHAPE zone (K6, 4 zones with hysteresis), switches, AUTO -> MODE
    A.label('f_mode')
    e('CTL', 't', imm='k6')
    e('SHL', 't2', 'kzone', imm=8)
    e('SUB', 't2', 't', 't2')
    ldt('u1', 128)
    e('SUB', 't2', 't2', 'u1')
    e('ABS', 't2', 't2')
    ldt('u1', 144)
    e('JLT', a='t2', b='u1', imm='zone_keep')
    e('SHR', 'kzone', 't', imm=8)
    A.label('zone_keep')
    e('AND', 'hsel', 'sw', imm=3)
    e('AND', 'vsel', 'sw', imm=12)
    e('SHR', 'vsel', 'vsel', imm=2)
    e('MOV', 'shp', 'kzone')
    ldt('u1', 3)
    e('JLT', a='kzone', b='u1', imm='mode_out')
    # AUTO: arrays from the random draw; shape locked by S3/S4 unless AUTO ALL
    e('MOV', 'shp', 'vsel')
    e('MOV', 'hsel', 'ahs')
    e('MOV', 'vsel', 'avs')
    e('JLT', a='shp', b='u1', imm='mode_out')
    e('MOV', 'shp', 'ash')
    A.label('mode_out')
    e('SHL', 't', 'vsel', imm=2)
    e('ADD', 't', 't', 'hsel')
    e('SHL', 't2', 'shp', imm=4)
    e('ADD', 't', 't', 't2')
    e('PRT', a='t', imm='MODE')
    task_end('f_gain')

    # F2: gains (VOLUME x GAIN, or VOLUME alone on EXTERNAL) and contours
    A.label('f_gain')
    e('CTL', 'u1', imm='p12')
    e('AND', 't', 'sw', imm=16)
    e('JNZ', a='t', imm='gain_ext')
    e('CTL', 't', imm='k1')
    e('MUL', a='u1', b='t')
    e('CTL', 't', imm='k2')
    e('MRD', 'g1', imm=7)
    e('MUL', a='u1', b='t')
    e('MRD', 'g2', imm=7)
    e('JMP', imm='gain_done')
    A.label('gain_ext')
    e('SHL', 'g1', 'u1', imm=3)
    e('MOV', 'g2', 'g1')
    A.label('gain_done')
    e('CTL', 't', imm='k3')
    e('SHR', 't', 't', imm=3)
    e('SIN', 'a1', 't', imm=raw(B['contour']))
    e('CTL', 't', imm='k4')
    e('SHR', 't', 't', imm=3)
    e('SIN', 'a2', 't', imm=raw(B['contour']))
    task_end('f_hue1')

    # F3 / F4: colour tap (Table I) -> hue = PHI0 - n * theta -> legal U/V
    for ch, reg, pu, pv, nxt in ((1, 'reg1', 'U1', 'V1', 'f_hue2'),
                                 (2, 'reg2', 'U2', 'V2', 'f_slow')):
        A.label(f'f_hue{ch}')
        e('SIN', 'u1', reg, imm=raw(B['tablei']))   # tap 1..16
        e('CTL', 't', imm='k5')
        ldt('t2', THETA_SPAN)
        e('MUL', a='t', b='t2')
        ldt('t2', THETA_MIN)
        e('MRD', 'u2', imm=10)
        e('ADD', 'u2', 'u2', 't2')                  # theta per stage
        e('MUL', a='u1', b='u2')
        ldt('t2', PHI0)
        e('MRD', 't', imm=0)                        # n * theta, mod one turn
        e('SUB', 'u3', 't2', 't')                   # hue angle
        e('SHR', 't', 'u3', imm=10)
        e('AND', 't', 't', imm=63)
        e('SIN', 'u4', 't', imm=raw(B['radius']))   # legal radius at this hue
        e('SIN', 't2', 'u3', imm=1)
        e('MUL', a='t2', b='u4')
        e('SIN', 'u1', 'u3')
        e('MRD', 't2', imm=15)
        e('MUL', a='u1', b='u4')
        e('PRT', a='t2', imm=pu)
        e('MRD', 'u1', imm=15)
        e('PRT', a='u1', imm=pv)
        task_end(nxt)

    # F5: slow per-frame work: Path B activity -> carrier pitch; mid pan
    A.label('f_slow')
    for act, cf in (('act1', 'cf1'), ('act2', 'cf2')):
        e('SHR', 't', act, imm=5)                   # act leaks: tau ~0.5 s
        e('SUB', act, act, 't')
        e('SHR', 't', act, imm=ACT_SH)              # count (crossings/frame)
        e('ADD', 't', 't', C(1))
        ldt('t2', 120)
        e('JLT', a='t', b='t2', imm=cf + 'ok')
        e('MOV', 't', 't2')
        A.label(cf + 'ok')
        ldt('t2', CARRIER_INC)
        e('MUL', a='t', b='t2')
        e('MRD', cf, imm=0)
    e('ADD', 'pph', 'pph', 'pinc')
    e('SIN', 'u1', 'pph')
    e('SHR', 'u1', 'u1', imm=1)                     # pan -0.5 .. +0.5
    ldt('t2', 16384 // 4)
    e('SHL', 't2', 't2', imm=2)
    e('ADD', 't', 'u1', 't2')
    e('MUL', a='t', b='mlv')
    e('ADD', 't2', 't2', 't2')                      # 32768 (= -32768: wraps)
    e('SUB', 't2', 't2', 't')                       # 32768 - t, as unsigned
    e('SUB', 't2', 't2', C(1))
    e('MRD', 'mtL', imm=15)
    e('MUL', a='t2', b='mlv')
    e('MRD', 'mtR', imm=15)
    task_end('f_aa')

    # F6: CLEAN anti-aliasing constants: pixels per reference unit
    A.label('f_aa')
    e('SHR', 'u1', 'sw', imm=5)
    e('AND', 'u1', 'u1', imm=7)
    e('SIN', 'u2', 'u1', imm=raw(B['wpx']))         # Wpx / 32, Q8.8
    e('SIN', 'u3', 'u1', imm=raw(B['lines']))       # lines / 32, Q8.8
    e('SIN', 't', 'hsel', imm=raw(B['recipn']))
    e('MUL', a='u2', b='t')
    e('SIN', 't', 'vsel', imm=raw(B['recipm']))
    e('MRD', 'u2', imm=15)
    e('MUL', a='u3', b='t')
    e('MRD', 'u3', imm=15)
    e('JLT', a='u2', b='u3', imm='aa_min')
    e('MOV', 'u2', 'u3')
    A.label('aa_min')
    ldt('t', 26214 // 4)
    e('SHL', 't', 't', imm=2)
    e('MUL', a='u2', b='t')                         # x 0.8: a 45-degree edge
    e('MRD', 'u2', imm=15)
    e('PRT', a='u2', imm='AAS')
    e('PRT', a='u2', imm='AAW')                     # ring band: 1.0 unit
    task_end('t_idle')

    # =================================================================
    # AUTO (mid-frame): 6-bit random register from the colour registers'
    # and the live counter's two LSBs; fire on N ones; draw one frame later
    # =================================================================
    A.label('t_auto')
    e('AND', 'rr', 'reg1', imm=3)
    e('AND', 't', 'reg2', imm=3)
    e('SHL', 't', 't', imm=2)
    e('ADD', 'rr', 'rr', 't')
    e('AND', 't', 'cz', imm=3)
    e('SHL', 't', 't', imm=4)
    e('ADD', 'rr', 'rr', 't')
    e('JNZ', a='apend', imm='a_draw')
    ldt('t2', 3)
    e('JLT', a='kzone', b='t2', imm='a_done')
    e('SUB', 'adiv', 'adiv', C(1))
    e('JGE', a='adiv', b='zero', imm='a_done')
    # rate (S1/S2): frantic (5 ones, every frame), lively (6, every frame),
    # relaxed (6, every 4th frame), rare (6, every 16th)
    e('AND', 't', 'sw', imm=3)
    e('LDI', 'need', imm=6)
    e('MOV', 'adiv', 'zero')
    e('JNZ', a='t', imm='a_r1')
    e('LDI', 'need', imm=5)
    e('JMP', imm='a_chk')
    A.label('a_r1')
    ldt('t2', 2)
    e('SUB', 't', 't', 't2')
    e('JLT', a='t', b='zero', imm='a_chk')
    e('LDI', 'adiv', imm=3)
    e('JLT', a='zero', b='t', imm='a_r3')
    e('JMP', imm='a_chk')
    A.label('a_r3')
    e('LDI', 'adiv', imm=15)
    A.label('a_chk')
    e('SIN', 't', 'rr', imm=raw(B['popcount']))
    e('JLT', a='t', b='need', imm='a_done')
    e('LDI', 'apend', imm=1)
    e('JMP', imm='a_done')
    A.label('a_draw')
    e('MOV', 'apend', 'zero')
    e('AND', 'ahs', 'rr', imm=3)
    e('AND', 'avs', 'rr', imm=12)
    e('SHR', 'avs', 'avs', imm=2)
    e('SHR', 'ash', 'rr', imm=4)
    ldt('t2', 3)
    e('JLT', a='ash', b='t2', imm='a_done')
    e('MOV', 'ash', 'zero')
    A.label('a_done')
    task_end('t_idle')

    A.resolve()
    A.r('rptr', A.labels['r0'])
    A.r('tptr', A.labels['t_idle'])
    assert len(A.code) <= DEPTH, len(A.code)
    return A


if __name__ == '__main__':
    a = build()
    print(f'{len(a.code)} words, {len(a.regs)} registers')
