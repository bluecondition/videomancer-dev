#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
WOODGRAIN - reference Atari 2600 model (py65 CPU + a TIA/RIOT written to mirror
woodgrain.vhd).  Used to check the TIA rules on real ROMs and to produce frames
to diff against GHDL renders.

TIA rules follow Stella's counter model (object counters 0..159 that tick on
visible clocks and HMOVE pulses; draw-start decodes at 156 / 12 / 28 / 60; a
player's first pixel 5 clocks after its decode, missile / ball 4), minus the
exotic quirks (starfield, late-RESPx, NUSIZ changes mid-draw).  A CPU write in
cycle n lands just before color clock 3n+3+d, d = 0 (most), 1 (GRPx, ENAxx,
REFPx, VBLANK), 2 (PFx, HMxx, HMCLR), 6 (HMOVE).

  python3 vcs_model.py rom.bin out.png [--frames N] [--joy R|L|U|D...]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from isa6502 import OPS  # noqa: E402

DECODES = {0: (156,), 1: (156, 12), 2: (156, 28), 3: (156, 12, 28), 4: (156, 60),
           5: (156,), 6: (156, 28, 60), 7: (156,)}
DIVIDER = {5: 2, 7: 4}
WIDTHS = (1, 2, 4, 8)
DELAY = {0x1B: 1, 0x1C: 1, 0x1D: 1, 0x1E: 1, 0x1F: 1, 0x0B: 1, 0x0C: 1, 0x01: 1,
         0x0D: 2, 0x0E: 2, 0x0F: 2, 0x20: 2, 0x21: 2, 0x22: 2, 0x23: 2, 0x24: 2,
         0x2B: 2, 0x2A: 6}
W0, WH = 25, 216          # display window: lines 25..240 after the VSYNC-off line


def hm_clocks(v):
    return ((v >> 4) & 0x0F) ^ 0x08


class Obj:
    def __init__(self, kind):
        self.kind = kind          # 'P', 'M', 'B'
        self.counter = 0
        self.rendering = False
        self.rc = 0
        self.sample = 0
        self.copy = 1
        self.nusiz = 0
        self.width = 1
        self.hm = 0
        self.moving = False
        self.on = False

    def decodes(self):
        return (156,) if self.kind == 'B' else DECODES[self.nusiz & 7]

    def divider(self):
        return DIVIDER.get(self.nusiz & 7, 1) if self.kind == 'P' else 1


class TIA:
    def __init__(self):
        self.hctr = 0
        self.frame_state = False      # False = HBLANK
        self.ext_hblank = False
        self.p = [Obj('P'), Obj('P')]
        self.m = [Obj('M'), Obj('M')]
        self.bl = Obj('B')
        self.grp_new = [0, 0]
        self.grp_old = [0, 0]
        self.refp = [False, False]
        self.vdelp = [False, False]
        self.enam = [False, False]
        self.resmp = [False, False]
        self.enabl_new = False
        self.enabl_old = False
        self.vdelbl = False
        self.pf = 0                   # 20-bit pattern, bit i = PF pixel i
        self.pf_ref = False           # CTRLPF D0
        self.pf_refl = False          # latched at x = 0 / 79
        self.ctrlpf = 0
        self.col = [0, 0, 0, 0]       # P0 P1 PF BK
        self.cx = 0                   # 15 collision latches
        self.vblank = 0
        self.vsync = 0
        self.mv_on = False
        self.mv_clock = 0
        self.queue = []               # [ticks_left, reg, value]
        self.wsync = False
        self.inpt4 = 0x80
        self.line_px = [0] * 160
        self.on_line = None           # callback(line_pixels)
        self.on_vsync = None          # callback(new_state)

    # ---------------- CPU interface ----------------
    def write(self, a, v):
        a &= 0x3F
        d = DELAY.get(a)
        if d is not None:
            self.queue.append([d, a, v])
        else:
            self.apply(a, v)

    def read(self, a):
        a &= 0x0F
        cx = self.cx
        bits = {0: ((cx >> 0) & 1, (cx >> 1) & 1), 1: ((cx >> 2) & 1, (cx >> 3) & 1),
                2: ((cx >> 4) & 1, (cx >> 5) & 1), 3: ((cx >> 6) & 1, (cx >> 7) & 1),
                4: ((cx >> 8) & 1, (cx >> 9) & 1), 5: ((cx >> 10) & 1, (cx >> 11) & 1),
                6: ((cx >> 12) & 1, 0), 7: ((cx >> 13) & 1, (cx >> 14) & 1)}
        if a < 8:
            b7, b6 = bits[a]
            return (b7 << 7) | (b6 << 6)
        if a == 0x0C:
            return self.inpt4
        if a in (0x0D, 0x08, 0x09, 0x0A, 0x0B):
            return 0x80
        return 0

    def resx(self):
        if not self.frame_state:
            return 158 if self.hctr >= 73 else 159   # late HBLANK (Stella resxLateHblankThreshold)
        return 157

    def apply(self, a, v):
        p, m = self.p, self.m
        if a == 0x00:
            new = v & 2
            if new != self.vsync and self.on_vsync:
                self.on_vsync(new)
            self.vsync = new
        elif a == 0x01:
            self.vblank = v & 2
        elif a == 0x02:
            self.wsync = True
        elif a in (0x04, 0x05):
            i = a - 4
            p[i].nusiz = v & 7
            m[i].nusiz = v & 7
            m[i].width = WIDTHS[(v >> 4) & 3]
        elif 0x06 <= a <= 0x09:
            self.col[a - 6] = v & 0xFE
        elif a == 0x0A:
            self.ctrlpf = v
            self.pf_ref = bool(v & 1)
            self.bl.width = WIDTHS[(v >> 4) & 3]
        elif a in (0x0B, 0x0C):
            self.refp[a - 0x0B] = bool(v & 8)
        elif a == 0x0D:
            self.pf = (self.pf & 0xFFFF0) | (v >> 4)
        elif a == 0x0E:
            r = 0                                   # PF1: bit 7 is the leftmost pixel
            for b in range(8):
                if v & (0x80 >> b):
                    r |= 1 << b
            self.pf = (self.pf & 0xFF00F) | (r << 4)
        elif a == 0x0F:
            self.pf = (self.pf & 0x00FFF) | (v << 12)
        elif a in (0x10, 0x11):
            self.p[a - 0x10].counter = self.resx()
        elif a in (0x12, 0x13):
            self.m[a - 0x12].counter = self.resx()
        elif a == 0x14:
            self.bl.counter = self.resx()
        elif a == 0x1B:
            self.grp_new[0] = v
            self.grp_old[1] = self.grp_new[1]
        elif a == 0x1C:
            self.grp_new[1] = v
            self.grp_old[0] = self.grp_new[0]
            self.enabl_old = self.enabl_new
        elif a in (0x1D, 0x1E):
            self.enam[a - 0x1D] = bool(v & 2)
        elif a == 0x1F:
            self.enabl_new = bool(v & 2)
        elif a in (0x20, 0x21):
            self.p[a - 0x20].hm = hm_clocks(v)
        elif a in (0x22, 0x23):
            self.m[a - 0x22].hm = hm_clocks(v)
        elif a == 0x24:
            self.bl.hm = hm_clocks(v)
        elif a in (0x25, 0x26):
            self.vdelp[a - 0x25] = bool(v & 1)
        elif a == 0x27:
            self.vdelbl = bool(v & 1)
        elif a in (0x28, 0x29):
            self.resmp[a - 0x28] = bool(v & 2)
        elif a == 0x2A:
            self.mv_on = True
            self.mv_clock = 0
            self.ext_hblank = True
            for o in self.objs():
                o.moving = True
        elif a == 0x2B:
            for o in self.objs():
                o.hm = 8
        elif a == 0x2C:
            self.cx = 0

    def objs(self):
        return (self.p[0], self.p[1], self.m[0], self.m[1], self.bl)

    # ---------------- per-object clocks ----------------
    def tick_obj(self, o, i=None):
        """One counter clock: visibility from the state BEFORE the clock."""
        if o.kind == 'P':
            div = o.divider()
            trip = 0 if div == 1 else 1
            pat = self.grp_old[i] if self.vdelp[i] else self.grp_new[i]
            s = o.sample if self.refp[i] else 7 - o.sample
            o.on = o.rendering and o.rc >= trip and bool(pat & (1 << s))
            if o.counter in o.decodes():
                o.rendering, o.sample, o.rc = True, 0, -5
                o.copy = 1 if o.counter == 156 else 2
            elif o.rendering:
                o.rc += 1
                if div == 1:
                    if o.rc > 0:
                        o.sample += 1
                elif o.rc > 1 and ((o.rc - 1) & (div - 1)) == 0:
                    o.sample += 1
                if o.sample > 7:
                    o.rendering = False
        else:
            if o.kind == 'M':
                en = self.enam[i] and not self.resmp[i]
                width = o.width
            else:
                en = self.enabl_old if self.vdelbl else self.enabl_new
                width = o.width
            o.on = o.rendering and o.rc >= 0 and en
            if o.counter in o.decodes() and not (o.kind == 'M' and self.resmp[i]):
                o.rendering, o.rc = True, -4
            elif o.rendering:
                o.rc += 1
                if o.rc >= width:
                    o.rendering = False
        o.counter = 0 if o.counter >= 159 else o.counter + 1

    def tick(self):
        # 1. delayed writes due now
        if self.queue:
            due = [q for q in self.queue if q[0] == 0]
            self.queue = [q for q in self.queue if q[0] > 0]
            for _, a, v in due:
                self.apply(a, v)
            for q in self.queue:
                q[0] -= 1
        # 2. HMOVE pulses every 4th clock
        if self.mv_on and (self.hctr & 3) == 0:
            clk = 0 if self.mv_clock > 15 else self.mv_clock
            for idx, o in enumerate(self.objs()):
                if o.moving:
                    if clk == o.hm:
                        o.moving = False
                    elif not self.frame_state:
                        self.tick_obj(o, idx % 2 if idx < 4 else None)
                        o.on = False
            self.mv_on = any(o.moving for o in self.objs())
            self.mv_clock += 1
        # 3. HBLANK / visible
        if not self.frame_state:
            for o in self.objs():
                o.on = False
            if self.hctr == 0:
                self.ext_hblank = False
            elif self.hctr == 67 and not self.ext_hblank:
                self.frame_state = True
            elif self.hctr == 75 and self.ext_hblank:
                self.frame_state = True
            pix = None
        else:
            x = self.hctr - 68
            if x == 0 or x == 79:
                self.pf_refl = self.pf_ref
            if (x & 3) == 0:
                col = x >> 2
                if col < 20:
                    b = col
                elif self.pf_refl:
                    b = 39 - col
                else:
                    b = col - 20
                self.pf_on = bool(self.pf & (1 << b))
            self.tick_obj(self.p[0], 0)
            self.tick_obj(self.p[1], 1)
            self.tick_obj(self.m[0], 0)
            self.tick_obj(self.m[1], 1)
            self.tick_obj(self.bl)
            for i in (0, 1):
                pl = self.p[i]
                if self.resmp[i] and pl.rendering and pl.sample == 4 and pl.copy == 1:
                    div = pl.divider()
                    back = {1: 5, 2: 8, 4: 12}[div]
                    self.m[i].counter = (pl.counter - back) % 160
            pix = self.render(x)
        # 4. collisions (not during VBLANK)
        if self.frame_state and not self.vblank:
            P0, P1 = self.p[0].on, self.p[1].on
            M0, M1 = self.m[0].on, self.m[1].on
            BL, PF = self.bl.on, self.pf_on
            pairs = [(M0, P1), (M0, P0), (M1, P0), (M1, P1), (P0, PF), (P0, BL), (P1, PF),
                     (P1, BL), (M0, PF), (M0, BL), (M1, PF), (M1, BL), (BL, PF), (P0, P1),
                     (M0, M1)]
            for k, (a, b) in enumerate(pairs):
                if a and b:
                    self.cx |= 1 << k
        if pix is not None:
            self.line_px[self.hctr - 68] = pix
        # 5. advance
        self.hctr += 1
        if self.hctr >= 228:
            self.hctr = 0
            self.frame_state = False
            if self.on_line:
                self.on_line(self.line_px)
            self.line_px = [0] * 160

    pf_on = False

    def render(self, x):
        if self.vblank:
            return 0
        P0, P1 = self.p[0].on, self.p[1].on
        M0, M1 = self.m[0].on, self.m[1].on
        BL, PF = self.bl.on, self.pf_on
        c0, c1, cpf, cbk = self.col
        score = (self.ctrlpf & 6) == 2
        pfp = bool(self.ctrlpf & 4)
        pfc = (c0 if x < 80 else c1) if score else cpf
        if pfp:
            order = ((PF, cpf), (BL, cpf), (P0, c0), (M0, c0), (P1, c1), (M1, c1))
        elif score:
            order = ((P0, c0), (M0, c0), (PF, pfc), (P1, c1), (M1, c1), (BL, cpf))
        else:
            order = ((P0, c0), (M0, c0), (P1, c1), (M1, c1), (PF, cpf), (BL, cpf))
        for on, c in order:
            if on:
                return c
        return cbk


class RIOT:
    def __init__(self):
        self.ram = bytearray(128)
        self.timer = 0
        self.div = 1024
        self.sub = 0
        self.flag = 0
        self.swcha = 0xFF
        self.swchb = 0x3F

    def cycle(self):
        self.sub = (self.sub + 1) & (self.div - 1)
        if self.flag:
            self.timer = (self.timer - 1) & 0xFF
        elif self.sub == 0:
            if self.timer == 0:
                self.timer = 0xFF
                self.flag = 1
            else:
                self.timer -= 1

    def read(self, a):
        if not a & 0x200:
            return self.ram[a & 0x7F]
        r = a & 7
        if r == 0:
            return self.swcha
        if r == 2:
            return self.swchb
        if r in (4, 6):
            self.flag = 0
            return self.timer
        if r in (5, 7):
            return self.flag << 7
        return 0

    def write(self, a, v):
        if not a & 0x200:
            self.ram[a & 0x7F] = v
            return
        if a & 0x14 == 0x14:                       # timer writes $294-$297
            self.div = (1, 8, 64, 1024)[a & 3]
            self.timer = v
            self.sub = self.div - 1
            self.flag = 0


class Bus:
    """py65 memory object: 13-bit 6507 address decode."""
    def __init__(self, rom, tia, riot):
        self.rom, self.tia, self.riot = rom, tia, riot
        self.mask = len(rom) - 1

    def __getitem__(self, a):
        if isinstance(a, slice):
            return [self[i] for i in range(*a.indices(0x10000))]
        a &= 0x1FFF
        if a & 0x1000:
            return self.rom[a & self.mask]
        if not a & 0x80:
            return (self.tia.read(a) & 0xC0) | (a & 0x3F)
        return self.riot.read(a)

    def peek(self, a):
        a &= 0x1FFF
        if a & 0x1000:
            return self.rom[a & self.mask]
        if a & 0x80 and not a & 0x200:
            return self.riot.ram[a & 0x7F]
        return 0

    def __setitem__(self, a, v):
        if isinstance(a, slice):
            return
        a &= 0x1FFF
        v &= 0xFF
        if a & 0x1000:
            return
        if not a & 0x80:
            self.tia.write(a, v)
        else:
            self.riot.write(a, v)

    def __len__(self):
        return 0x10000


def make_mpu(bus):
    from py65.devices.mpu6502 import MPU

    class MPU2600(MPU):
        pass

    MPU2600.instruct = list(MPU.instruct)
    MPU2600.cycletime = list(MPU.cycletime)
    MPU2600.extracycles = list(MPU.extracycles)
    mpu = MPU2600(memory=bus)
    add_illegals(MPU2600)
    return mpu


def add_illegals(cls):
    """Functional semantics of the stable illegal opcodes (timing comes from OPS)."""
    def ea(self, mode):
        if mode == 'zp':
            return self.ZeroPageAddr()
        if mode == 'zpx':
            return self.ZeroPageXAddr()
        if mode == 'zpy':
            return self.ZeroPageYAddr()
        if mode == 'abs':
            return self.AbsoluteAddr()
        if mode == 'abx':
            return self.AbsoluteXAddr()
        if mode == 'aby':
            return self.AbsoluteYAddr()
        if mode == 'izx':
            return self.IndirectXAddr()
        if mode == 'izy':
            return self.IndirectYAddr()
        raise ValueError(mode)

    def setnz(self, v):
        self.p &= ~(self.NEGATIVE | self.ZERO)
        if v & 0x80:
            self.p |= self.NEGATIVE
        if v == 0:
            self.p |= self.ZERO

    def make(mn, mode, pen):
        n = {'imm': 1, 'zp': 1, 'zpx': 1, 'zpy': 1, 'izx': 1, 'izy': 1,
             'abs': 2, 'abx': 2, 'aby': 2, 'imp': 0}[mode]

        def f(self):
            if mode == 'imm':
                addr = self.pc
            elif mode != 'imp':
                self.addcycles = pen
                addr = ea(self, mode)
            if mn in ('SLO', 'RLA', 'SRE', 'RRA', 'DCP', 'ISB'):
                v = self.ByteAt(addr)
                c = self.p & self.CARRY
                if mn == 'SLO':
                    self.p = (self.p & ~self.CARRY) | (1 if v & 0x80 else 0)
                    v = (v << 1) & 0xFF
                    self.memory[addr] = v
                    self.a |= v
                    setnz(self, self.a)
                elif mn == 'RLA':
                    self.p = (self.p & ~self.CARRY) | (1 if v & 0x80 else 0)
                    v = ((v << 1) | c) & 0xFF
                    self.memory[addr] = v
                    self.a &= v
                    setnz(self, self.a)
                elif mn == 'SRE':
                    self.p = (self.p & ~self.CARRY) | (v & 1)
                    v >>= 1
                    self.memory[addr] = v
                    self.a ^= v
                    setnz(self, self.a)
                elif mn == 'RRA':
                    self.p = (self.p & ~self.CARRY) | (v & 1)
                    v = (v >> 1) | (0x80 if c else 0)
                    self.memory[addr] = v
                    self.opADC(lambda: addr)
                elif mn == 'DCP':
                    v = (v - 1) & 0xFF
                    self.memory[addr] = v
                    self.opCMPR(lambda: addr, self.a)
                elif mn == 'ISB':
                    v = (v + 1) & 0xFF
                    self.memory[addr] = v
                    self.opSBC(lambda: addr)
            elif mn == 'SAX':
                self.memory[addr] = self.a & self.x
            elif mn == 'LAX':
                self.a = self.x = self.ByteAt(addr)
                setnz(self, self.a)
            elif mn == 'ANC':
                self.a &= self.ByteAt(addr)
                setnz(self, self.a)
                self.p = (self.p & ~self.CARRY) | (1 if self.a & 0x80 else 0)
            elif mn == 'ALR':
                self.a &= self.ByteAt(addr)
                self.p = (self.p & ~self.CARRY) | (self.a & 1)
                self.a >>= 1
                setnz(self, self.a)
            elif mn == 'ARR':
                self.a &= self.ByteAt(addr)
                self.a = (self.a >> 1) | (0x80 if self.p & self.CARRY else 0)
                setnz(self, self.a)
                b6, b5 = (self.a >> 6) & 1, (self.a >> 5) & 1
                self.p = (self.p & ~(self.CARRY | self.OVERFLOW)) | b6 | ((b6 ^ b5) << 6)
            elif mn == 'SBX':
                t = (self.a & self.x) - self.ByteAt(addr)
                self.p = (self.p & ~self.CARRY) | (1 if t >= 0 else 0)
                self.x = t & 0xFF
                setnz(self, self.x)
            elif mn == 'LXA':
                self.a = self.x = self.ByteAt(addr)
                setnz(self, self.a)
            elif mn == 'SBC':
                self.opSBC(lambda: addr)
            # NOP variants: nothing
            self.pc += n
        return f

    from py65.devices.mpu6502 import MPU
    cls.cycletime[0xCE] = 6               # py65 lists DEC abs as 3 cycles
    for op, (mn, mode, cyc, pen) in OPS.items():
        if MPU.instruct[op] is not MPU.instruct[0x02] and MPU.instruct[op].__name__ != 'inst_not_implemented':
            continue
        cls.instruct[op] = make(mn, mode, pen)
        cls.cycletime[op] = cyc
        cls.extracycles[op] = 0


def instr_cycles(bus, mpu):
    """Cycle count of the instruction at PC (incl. page-cross / branch extras)."""
    pc = mpu.pc
    op = bus.peek(pc)
    mn, mode, cyc, pen = OPS[op]
    lo, hi = bus.peek(pc + 1), bus.peek(pc + 2)
    if pen:
        if mode == 'abx':
            cyc += ((lo + mpu.x) >> 8) & 1
        elif mode == 'aby':
            cyc += ((lo + mpu.y) >> 8) & 1
        elif mode == 'izy':
            base = bus.peek(lo) | (bus.peek((lo + 1) & 0xFF) << 8)
            cyc += ((base & 0xFF) + mpu.y) >> 8 & 1
    if mode == 'rel':
        p = mpu.p
        cond = {'BPL': not p & 0x80, 'BMI': p & 0x80, 'BVC': not p & 0x40, 'BVS': p & 0x40,
                'BCC': not p & 1, 'BCS': p & 1, 'BNE': not p & 2, 'BEQ': p & 2}[mn]
        if cond:
            nxt = (pc + 2) & 0xFFFF
            tgt = (nxt + (lo - 256 if lo & 0x80 else lo)) & 0xFFFF
            cyc += 1 + ((tgt ^ nxt) >> 8 != 0)
    return cyc


class VCS:
    def __init__(self, rom):
        if len(rom) == 2048:
            rom = rom + rom
        self.tia = TIA()
        self.riot = RIOT()
        self.bus = Bus(rom, self.tia, self.riot)
        self.mpu = make_mpu(self.bus)
        self.mpu.pc = self.bus.peek(0xFFFC) | (self.bus.peek(0xFFFD) << 8)
        self.mpu.sp = 0xFD
        self.mpu.p = 0x24
        self.line = 0                 # lines since the VSYNC-off line
        self.frame = {}
        self.frames = []
        self.tia.on_line = self._line
        self.tia.on_vsync = self._vsync
        self.cycles = 0
        self.trace = None

    def _line(self, px):
        if W0 <= self.line < W0 + WH:
            self.frame[self.line - W0] = list(px)
        self.line += 1
        if self.line >= 400:            # no VSYNC: free-run frames of 400 lines
            self._vsync(2)
            self._vsync(0)

    def _vsync(self, on):
        if on:
            self.frames.append(self.frame)
            self.frame = {}
        else:
            self.line = 0

    def step(self):
        mpu, tia, riot = self.mpu, self.tia, self.riot
        if self.trace is not None:
            self.trace.append((mpu.pc, mpu.a, mpu.x, mpu.y, mpu.sp, mpu.p | 0x30, self.cycles))
        cyc = instr_cycles(self.bus, mpu)
        for _ in range(cyc):
            for _ in range(3):
                tia.tick()
            riot.cycle()
        self.cycles += cyc
        mpu.step()
        if tia.wsync:
            tia.wsync = False
            while tia.hctr != 0:
                for _ in range(3):
                    tia.tick()
                riot.cycle()
                self.cycles += 1

    def run_frames(self, n, max_cycles=10_000_000):
        start = len(self.frames)
        c0 = self.cycles
        while len(self.frames) < start + n and self.cycles - c0 < max_cycles:
            self.step()
        return self.frames[-1] if self.frames else self.frame


def ntsc_palette():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'ntsc_palette.txt')) as f:
        vals = [int(t, 16) for t in f.read().split()]
    return [((v >> 16) & 255, (v >> 8) & 255, v & 255) for v in vals]


def save_png(frame, path, scale=(4, 2)):
    from PIL import Image
    pal = ntsc_palette()
    sx, sy = scale
    img = Image.new('RGB', (160 * sx, WH * sy))
    px = img.load()
    for y in range(WH):
        row = frame.get(y, [0] * 160)
        for x in range(160):
            c = pal[(row[x] >> 1) & 0x7F]
            for dy in range(sy):
                for dx in range(sx):
                    px[x * sx + dx, y * sy + dy] = c
    img.save(path)


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('rom')
    ap.add_argument('out')
    ap.add_argument('--frames', type=int, default=4)
    ap.add_argument('--joy', default='', help='hold directions, e.g. RU (right+up)')
    ap.add_argument('--fire', action='store_true')
    ap.add_argument('--reset-frames', type=int, default=0, help='hold Game Reset for N frames first')
    ap.add_argument('--dump', help='write every frame as text (216 lines x 160 hex colors)')
    a = ap.parse_args()
    vcs = VCS(open(a.rom, 'rb').read())
    if a.reset_frames:
        vcs.riot.swchb &= ~1
        vcs.run_frames(a.reset_frames)
        vcs.riot.swchb |= 1
    bits = {'R': 0x80, 'L': 0x40, 'D': 0x20, 'U': 0x10}
    for ch in a.joy.upper():
        vcs.riot.swcha &= ~bits[ch]
    if a.fire:
        vcs.tia.inpt4 = 0
    f = vcs.run_frames(a.frames)
    save_png(f, a.out)
    if a.dump:
        with open(a.dump, 'w') as fo:
            for i, fr in enumerate(vcs.frames):
                fo.write('frame %d\n' % i)
                for y in range(WH):
                    row = fr.get(y, [0] * 160)
                    fo.write(''.join('%02X' % (c & 0xFE) for c in row) + '\n')
    print('%s: %d frames, %d cycles, %d lines in last frame' % (
        a.out, len(vcs.frames), vcs.cycles, len(f)))
