#!/usr/bin/env python3
"""RTL-faithful CYCLE model of the NACRE knot engine's seed emission.

This is a mechanical translation of nacre.vhd's p_engine (E_SPAN + E_HERM),
p_norm, p_spst, p_seedram, p_ram and the p_fd replay, with EXACT VHDL signal
semantics: every process reads the PRE-edge value of every signal and writes
a POST-edge value.  A one-cycle read skew therefore shows up here exactly as
it does on the die -- in seconds, instead of a 3.5-minute image sim.

Usage:
    python3 nacre_model.py            # single-line trace + diagnostics
    python3 nacre_model.py --image    # full frame vs the exact field
"""
import sys

# ---------------------------------------------------------------- constants
C_NC     = 8        # chained discs (HD)
C_NT     = 12       # total rings (HD)
C_RCL    = 4095
NO_SUBPIX = True
TWO_CENTRES = False   # outer centre never wins and costs 2 comparators   # test: drop the sub-pixel distance correction

C_RROM = [(2 ** 18 + (256 + i)) // (2 * (256 + i)) for i in range(256)]


def sgn(v, w):
    """wrap to w-bit two's complement (VHDL signed truncation)"""
    v &= (1 << w) - 1
    return v - (1 << w) if v >> (w - 1) else v


def uns(v, w):
    return v & ((1 << w) - 1)


def msb18(v):
    for i in range(17, 0, -1):
        if v & (1 << i):
            return i
    return 0


# ------------------------------------------------------------------- engine
class Engine:
    """One scanline pass: E_SPAN then E_HERM.  Returns the seed word list."""

    def __init__(self, W, H, cx, cy, spac, ex, ey, kstep=None, rmax=None):
        self.W, self.H = W, H
        self.s_cx, self.s_cy = cx, cy
        self.s_spac = spac
        self.ram_ex = list(ex)          # C_NC entries
        self.ram_ey = list(ey)
        if kstep is None:
            kstep = 255 if spac >= 128 else (16 if spac < 8 else uns(spac << 1, 8))
        self.s_kstep = kstep
        self.s_rmax = min(spac * C_NT, C_RCL) if rmax is None else rmax

    # -- signal state -------------------------------------------------------
    def reset(self, dy):
        z = dict(
            e_st='E_SPAN', e_cnt=0, e_ph=0,
            m_a=0, m_b=0, m_ph=0, m_pl=0, m_p=0,
            sp_dy=dy, sp_ey=0, sp_rr=self.s_spac, sp_ady=0, sp_emp=0,
            sp_ex=0, sp_exc=0, sp_excq1=0, sp_empq1=0, sp_empq2=0,
            sp_wl=0, sp_wr1=0,
            sqa_num=0, sqa_res=0, sqa_one=0, sqa_run=0,
            sqb_num=0, sqb_res=0, sqb_one=0, sqb_run=0,
            spst_wa=0, spst_we=0, spst_wd=0, spst_ra=0, spst_q=0, sp_cl=0,
            sd_wa=0, sd_we=0, sd_wd=0,
            eng_bank=0,
            hm_slot=0, hm_side=0, hm_ph=0, hm_ean=0, hm_k=0, hm_lowp=C_NT,
            hm_xae=0, hm_xbe=0, hm_icx=0, hm_ocx=0, hm_idy2=0, hm_ody2=0,
            hm_iR=0, hm_oR=0, hm_icx0=0, hm_dy_n=0,
            hk_x=0, hk_ti=0, hk_to=0, hk_f=0,
            hw_di=0, hq_l=0, hq_df=0,
            hw_xn=0, hw_f0=0, hw_pend=0,
            hp_x=0, hp_f=0, hp_v=0,
            ks_g=0, ks_n=0, ks_n2=0,
            nrm_in=0, nrm_nm=0, hn=0, rrom_a=0, rrom_q=0,
            nrm_sh=0, nrm_in_q=0, nrm_nm_q=0,
            doa_ex=0, doa_ey=0,
        )
        self.s = z
        self.spst = [0] * 256
        self.seeds = []          # (slot, x_next, f0, d0) in write order
        self.trace = []          # per-knot internals (debug)
        self.cycles = 0
        self.slot_overflow = False

    # -- per-cycle processes ------------------------------------------------
    def step(self):
        s = self.s
        n = dict(s)

        # ---- ea_addr (combinational) + p_ram
        if s['e_st'] == 'E_HERM':
            ea = s['hm_ean']
        else:
            ea = s['e_cnt'] if s['e_cnt'] < C_NC else 0
        n['doa_ex'] = self.ram_ex[ea]
        n['doa_ey'] = self.ram_ey[ea]

        # ---- p_spst (read pre-edge, then write)
        n['spst_q'] = self.spst[s['spst_ra']]
        q = s['spst_q']
        n['sp_cl'] = self.W if q > self.W else (0 if q < 0 else q)
        if s['spst_we']:
            self.spst[s['spst_wa']] = s['spst_wd']

        # ---- p_seedram (write only; the replay is modelled separately)
        if s['sd_we']:
            slot = s['sd_wa'] & 0xFF
            xn = (s['sd_wd'] >> 46) & 0x7FF
            f0 = sgn((s['sd_wd'] >> 28) & 0x3FFFF, 18)
            d0 = sgn(s['sd_wd'] & 0xFFFFFFF, 28)
            if any(sl == slot for sl, _, _, _ in self.seeds):
                self.slot_overflow = True
            self.seeds.append((slot, xn, f0, d0))

        # ---- p_norm, 3 pipeline stages: encode / barrel / ROM.
        # hn is valid at T+3 and rrom_q at T+4 (rrom_q ALWAYS trails hn by one
        # cycle -- the ROM read indexes the PRE-edge rrom_a).
        n['nrm_sh'] = msb18(s['nrm_in'])
        n['nrm_in_q'] = s['nrm_in']
        n['nrm_nm_q'] = s['nrm_nm']
        nb = s['nrm_sh']
        if nb >= 8:
            v_tp = s['nrm_in_q'] >> (nb - 8)
            v_nu = s['nrm_nm_q'] >> (nb - 8)
        else:
            v_tp = s['nrm_in_q'] << (8 - nb)
            v_nu = s['nrm_nm_q'] << (8 - nb)
        n['rrom_a'] = v_tp & 0xFF
        n['hn'] = max(-16384, min(16383, v_nu))
        n['rrom_q'] = C_RROM[s['rrom_a']]

        # ---- p_engine
        self.engine(s, n)

        self.s = n
        self.cycles += 1

    def engine(self, s, n):
        W = self.W
        spac = self.s_spac
        kstep = self.s_kstep

        # 2-stage multiply
        mb = uns(s['m_b'], 15)
        n['m_ph'] = sgn(s['m_a'] * sgn(mb >> 7, 8), 23)
        n['m_pl'] = sgn(s['m_a'] * (mb & 0x7F), 23)
        n['m_p'] = sgn((s['m_ph'] << 7) + s['m_pl'], 30)

        # one-shot write strobes
        n['spst_we'] = 0
        n['sd_we'] = 0

        # sqrt units (radix-4: two radix-2 steps per cycle, 7 cycles)
        for u in ('sqa', 'sqb'):
            if s[u + '_run']:
                num, res, one = s[u + '_num'], s[u + '_res'], s[u + '_one']
                for _ in range(1):          # radix-2: one step per cycle
                    tr = res | one
                    if num >= tr:
                        num -= tr
                        res = (res >> 1) | one
                    else:
                        res >>= 1
                    one >>= 2
                n[u + '_num'], n[u + '_res'], n[u + '_one'] = num, res, one
                if one == 0:
                    n[u + '_run'] = 0

        if s['e_st'] == 'E_SPAN':
            self.e_span(s, n)
        elif s['e_st'] == 'E_HERM':
            self.e_herm(s, n)

    # ------------------------------------------------------------- E_SPAN --
    def e_span(self, s, n):
        ph, cnt = s['e_ph'], s['e_cnt']
        spac = self.s_spac

        if ph == 1:
            if cnt < C_NC:
                n['sp_ey'] = s['doa_ey']
                n['sp_ex'] = s['doa_ex']
        elif ph == 2:
            if cnt < C_NT:
                v_dy = s['sp_dy'] - s['sp_ey']
                n['sp_ady'] = uns(abs(v_dy), 15)
                n['sp_exc'] = sgn(s['sp_ex'] + self.s_cx, 16)
        elif ph == 3:
            if cnt < C_NT:
                if s['sp_ady'] > C_RCL:
                    n['sp_emp'] = 1
                    n['sp_ady'] = C_RCL
                else:
                    n['sp_emp'] = 0
        elif ph == 4:
            if cnt < C_NT:
                ady = s['sp_ady'] & 0x1FFF          # (12 downto 0)
                n['m_a'] = sgn(s['sp_rr'] - ady, 15)
                n['m_b'] = sgn(s['sp_rr'] + ady, 15)
        elif ph == 7:
            v_w = sgn(s['sqa_res'] & 0x1FFF, 16)
            n['sp_wl'] = sgn(s['sp_excq1'] - v_w, 16)
            n['sp_wr1'] = sgn(s['sp_excq1'] + v_w + 1, 16)
            n['sp_empq2'] = s['sp_empq1']
            if cnt < C_NT:
                n['sqa_num'] = 0 if s['m_p'] < 0 else (s['m_p'] & 0x7FFFFFF)
                n['sqa_res'] = 0
                n['sqa_one'] = 1 << 26
                n['sqa_run'] = 1
                n['sp_rr'] = C_RCL if s['sp_rr'] + spac > C_RCL else s['sp_rr'] + spac
            n['sp_excq1'] = s['sp_exc']
            n['sp_empq1'] = 1 if s['m_p'] < 0 else s['sp_emp']
        elif ph == 8:
            if cnt >= 1:
                n['spst_wa'] = cnt - 1
                n['spst_wd'] = s['sp_wl']
                n['spst_we'] = 1
                if s['sp_empq2'] == 0 and s['hm_lowp'] == C_NT:
                    n['hm_lowp'] = cnt - 1
        elif ph == 9:
            if cnt >= 1:
                n['spst_wa'] = C_NT + cnt - 1
                n['spst_wd'] = s['sp_wr1']
                n['spst_we'] = 1
            if cnt == C_NT:
                n['hm_ph'] = 48
                n['e_st'] = 'E_HERM'

        if ph == 14:
            n['e_ph'] = 0
            n['e_cnt'] = cnt + 1
        else:
            n['e_ph'] = ph + 1

    # ------------------------------------------------------------- E_HERM --
    # 23-cycle knot frame (0..22), run roll 23..29, finish 30..38, init 40..46.
    # The piece-slope divide is PIPELINED into the next knot's sqrt shadow
    # (states 2..9) and the sub-pixel distance correction is gone -- measured
    # worth 1.4/255 of mean error and it cost 8 cycles a knot.
    def e_herm(self, s, n):
        W = self.W
        spac = self.s_spac
        kstep = self.s_kstep
        ph = s['hm_ph']

        def clamp_edge(q):
            if q > W:
                return W
            if q < 0:
                return 0
            return q

        # ---------------- line init ----------------
        if ph == 48:
            n['hm_slot'] = 0
            n['hp_v'] = 0
            n['hw_pend'] = 0
            if s['hm_lowp'] == C_NT:
                n['hm_ph'] = 45
            else:
                n['hm_k'] = C_NT - 1
                n['hm_side'] = 0
                n['hm_ean'] = C_NC - 1
                n['spst_ra'] = C_NT - 1
                n['hm_oR'] = self.s_rmax
                n['hm_ph'] = 49
        elif ph == 49:
            n['hm_ean'] = C_NT - 2 if C_NT - 2 < C_NC else C_NC - 1
            n['hm_ph'] = 50
        elif ph == 50:
            n['hm_ocx'] = sgn(s['doa_ex'] + self.s_cx, 16)
            n['hm_dy_n'] = sgn(s['sp_dy'] - s['doa_ey'], 15)
            n['spst_ra'] = C_NT - 2 if C_NT - 1 > s['hm_lowp'] else C_NT + C_NT - 1
            n['hm_ph'] = 51
        elif ph == 51:
            n['m_a'] = s['hm_dy_n']          # OUTER dy^2 -> m_p at 54
            n['m_b'] = s['hm_dy_n']
            n['hm_xae'] = s['sp_cl']
            n['hm_ph'] = 52
        elif ph == 52:
            n['hm_icx'] = sgn(s['doa_ex'] + self.s_cx, 16)
            n['hm_dy_n'] = sgn(s['sp_dy'] - s['doa_ey'], 15)
            n['hm_iR'] = uns(self.s_rmax - spac, 13)
            n['hm_ph'] = 53
        elif ph == 53:
            n['m_a'] = s['hm_dy_n']          # INNER dy^2 -> m_p at 56
            n['m_b'] = s['hm_dy_n']
            n['hm_xbe'] = s['sp_cl']
            n['hm_ph'] = 54
        elif ph == 54:
            # m_p is the multiply issued at 51.  Reading it at 53 (and the
            # inner one at 54) was a ONE-CYCLE-EARLY read that left hm_ody2
            # holding E_SPAN's last w^2 -- every line's first run had a bogus
            # outer circle, masked only while the nest is concentric.
            n['hm_ody2'] = s['m_p'] & 0x7FFFFFF
            n['hm_ph'] = 55
        elif ph == 55:
            n['hm_ph'] = 56
        elif ph == 56:
            n['hm_idy2'] = s['m_p'] & 0x7FFFFFF
            # LEADING GROUND piece: flat black up to the first edge
            n['hw_xn'] = uns(s['hm_xae'], 11)
            n['hw_f0'] = 0
            n['hq_l'] = 0
            n['hq_df'] = 0
            n['hw_pend'] = 1
            n['ks_g'] = sgn(s['hm_xae'] + kstep, 16)
            n['hk_x'] = s['hm_xae']
            n['hm_ph'] = 29 if s['hm_xbe'] <= s['hm_xae'] + 1 else 0

        # --------------- knot frame (23 cycles) ---------------
        elif ph == 0:
            # hk_ti/hk_to registers are gone: the subtract feeds the multiply
            # operand register directly (FF-to-FF, no extra logic level)
            n['m_a'] = sgn(s['hk_x'] - s['hm_icx'], 15)
            n['m_b'] = sgn(s['hk_x'] - s['hm_icx'], 15)
            n['hm_ph'] = 1
        elif ph == 1:
            n['m_a'] = sgn(s['hk_x'] - s['hm_ocx'], 15)
            n['m_b'] = sgn(s['hk_x'] - s['hm_ocx'], 15)
            # previous piece's slope divide rides this knot's sqrt shadow
            n['nrm_in'] = s['hq_l'] & 0xFFF
            n['nrm_nm'] = s['hq_df']
            n['hm_ph'] = 2
        elif ph == 2:
            n['hm_ph'] = 3
        elif ph == 3:
            n['sqa_num'] = uns((s['m_p'] & 0x7FFFFFF) + s['hm_idy2'], 27)
            n['sqa_res'] = 0
            n['sqa_one'] = 1 << 26
            n['sqa_run'] = 1
            n['hm_ph'] = 4
        elif ph == 4:
            n['sqb_num'] = uns((s['m_p'] & 0x7FFFFFF) + s['hm_ody2'], 27)
            n['sqb_res'] = 0
            n['sqb_one'] = 1 << 26
            n['sqb_run'] = 1
            n['hm_ph'] = 5
        elif ph == 5:
            n['m_a'] = s['hn']          # previous piece's slope divide
            n['m_b'] = s['rrom_q']
            n['hm_ph'] = 6
        elif ph in (6, 7):
            n['hm_ph'] = ph + 1
        elif ph == 8:
            if s['hw_pend']:
                v_dp = s['m_p'] >> 3
                v_dp = max(-134217728, min(134217727, v_dp))
                n['sd_wd'] = (s['sd_wd'] & ~0xFFFFFFF) | uns(v_dp, 28)
            n['hm_ph'] = 9
        elif ph == 9:
            if s['hw_pend']:
                n['sd_wa'] = (s['eng_bank'] << 8) | s['hm_slot']
                w = s['sd_wd'] & 0xFFFFFFF
                w |= (uns(s['hw_xn'], 11) << 46) | (uns(s['hw_f0'], 18) << 28)
                n['sd_wd'] = w
                n['sd_we'] = 1
                n['hm_slot'] = uns(s['hm_slot'] + 1, 8)
                n['hw_pend'] = 0
            n['hm_ph'] = 10
        elif ph == 10:
            n['hm_ph'] = 11
        # The next knot's x does not depend on this knot's f, so the whole
        # candidate chain runs here in the sqrt's idle window.  As one cycle at
        # the end of the frame it was an 18 ns compare/mux/add chain and the
        # design's critical path.
        elif ph == 11:
            v_c = s['hm_xbe']
            if s['ks_g'] + 1 < s['hm_xbe']:
                v_c = s['ks_g']
            n['ks_n'] = v_c
            n['hm_ph'] = 12
        elif ph == 12:
            if s['hm_icx'] > s['hk_x'] + 1 and s['hm_icx'] < s['ks_n']:
                n['ks_n'] = s['hm_icx']
            n['hm_ph'] = 13
        elif ph == 13:
            n['ks_n2'] = sgn(s['ks_n'] + kstep, 16)
            n['hm_ph'] = 14
        elif ph in (14, 15, 16, 17):
            n['hm_ph'] = ph + 1
        elif ph == 18:
            iR6 = uns(s['hm_iR'] << 6, 20)
            r6 = (s['sqa_res'] & 0x3FFF) << 6
            n['hw_di'] = uns(r6 - iR6, 19) if r6 > iR6 else 0
            n['hm_ph'] = 19
        elif ph == 19:
            # d_out and the gap fold into ONE cycle so the frame stays 23
            oR6 = uns(s['hm_oR'] << 6, 20)
            r6 = (s['sqb_res'] & 0x3FFF) << 6
            v_do = uns(oR6 - r6, 19) if oR6 > r6 else 0
            v_rv = s['hw_di'] + v_do
            if v_rv > 262143:
                v_rv = 262143
            if v_rv < 64:
                v_rv = 64
            n['nrm_in'] = v_rv
            n['nrm_nm'] = s['hw_di']
            n['hm_ph'] = 20
        elif ph in (20, 21, 22):
            n['hm_ph'] = ph + 1
        elif ph == 23:
            n['m_a'] = s['hn']
            n['m_b'] = s['rrom_q']
            n['hm_ph'] = 24
        elif ph in (24, 25):
            n['hm_ph'] = ph + 1
        elif ph == 26:
            v_fp = s['m_p'] >> 1
            n['hk_f'] = 0 if v_fp < 0 else (65536 if v_fp > 65536 else v_fp)
            self.trace.append(dict(
                x=s['hk_x'], q6a=s['hw_di'], q6b=0, di=s['hw_di'],
                iR=s['hm_iR'], oR=s['hm_oR'], icx=s['hm_icx'], ocx=s['hm_ocx'],
                idy2=s['hm_idy2'], ody2=s['hm_ody2'], k=s['hm_k'],
                side=s['hm_side'], f=n['hk_f']))
            n['hm_ph'] = 27
        elif ph == 27:
            if s['hp_v'] and s['hk_x'] > s['hp_x']:
                n['hw_xn'] = uns(s['hk_x'], 11)
                n['hw_f0'] = s['hp_f']
                n['hq_l'] = s['hk_x'] - s['hp_x']
                # >>8, not >>3: p_norm scales the numerator by the SAME
                # exponent as L, so a short piece shifts it UP -- >>3
                # saturated hn for every piece under 128 px
                n['hq_df'] = (s['hk_f'] - s['hp_f']) >> 8
                n['hw_pend'] = 1
            n['hp_x'] = s['hk_x']
            n['hp_f'] = s['hk_f']
            n['hp_v'] = 1
            n['hm_ph'] = 28
        elif ph == 28:
            if s['hk_x'] >= s['hm_xbe']:
                n['hm_ph'] = 29
            else:
                n['hk_x'] = s['ks_n']          # chosen back in states 11..13
                n['ks_g'] = s['ks_n2']         # grid re-anchors on every knot
                n['hm_ph'] = 0

        # ---------------- run roll ---------------
        elif ph == 29:
            if s['hm_side'] == 0 and s['hm_k'] > s['hm_lowp']:
                n['hm_k'] = s['hm_k'] - 1
                n['hm_ean'] = 0 if s['hm_k'] < 2 else (s['hm_k'] - 2 if s['hm_k'] - 2 < C_NC else C_NC - 1)
                n['hm_ph'] = 30
            elif s['hm_side'] == 0:
                n['hm_side'] = 1
                if s['hm_lowp'] < C_NT - 1:
                    n['hm_k'] = s['hm_lowp'] + 1
                    n['hm_ean'] = s['hm_lowp'] + 1 if s['hm_lowp'] + 1 < C_NC else C_NC - 1
                    n['hm_ph'] = 30
                else:
                    n['hm_ph'] = 36
            elif s['hm_k'] < C_NT - 1:
                n['hm_k'] = s['hm_k'] + 1
                n['hm_ean'] = s['hm_k'] + 1 if s['hm_k'] + 1 < C_NC else C_NC - 1
                n['hm_ph'] = 30
            else:
                n['hm_ph'] = 36
        elif ph == 30:
            if s['hm_side'] == 0 and s['hm_k'] > s['hm_lowp']:
                n['spst_ra'] = s['hm_k'] - 1
            else:
                n['spst_ra'] = C_NT + s['hm_k']
            n['hm_ph'] = 31
        elif ph == 31:
            if s['hm_side'] == 0:
                n['hm_ocx'] = s['hm_icx']
                n['hm_ody2'] = s['hm_idy2']
                n['hm_oR'] = s['hm_iR']
                n['hm_iR'] = 0 if s['hm_k'] == 0 else uns(s['hm_iR'] - spac, 13)
            else:
                n['hm_icx'] = s['hm_ocx']
                n['hm_idy2'] = s['hm_ody2']
                n['hm_iR'] = s['hm_oR']
                n['hm_oR'] = uns(s['hm_oR'] + spac, 13)
            n['hm_dy_n'] = sgn(s['sp_dy'] - s['doa_ey'], 15)
            n['hm_ph'] = 32
        elif ph == 32:
            if s['hm_side'] == 0:
                n['hm_icx'] = sgn(s['doa_ex'] + self.s_cx, 16)
            else:
                n['hm_ocx'] = sgn(s['doa_ex'] + self.s_cx, 16)
            n['m_a'] = s['hm_dy_n']
            n['m_b'] = s['hm_dy_n']
            n['hm_xae'] = s['hm_xbe']
            n['ks_g'] = sgn(s['hm_xbe'] + kstep, 16)
            n['hm_ph'] = 33
        elif ph == 33:
            n['hm_xbe'] = s['sp_cl']
            n['hm_ph'] = 34
        elif ph == 34:
            n['hm_ph'] = 35
        elif ph == 35:
            if s['hm_side'] == 0:
                n['hm_idy2'] = s['m_p'] & 0x7FFFFFF
            else:
                n['hm_ody2'] = s['m_p'] & 0x7FFFFFF
            # the shared edge belongs to BOTH runs but with different values
            # (f jumps 1 -> 0 across a ring edge), so it re-opens the piece
            n['hp_v'] = 0
            if s['hm_xbe'] <= s['hm_xae'] + 1:
                n['hm_ph'] = 29
            else:
                n['hk_x'] = s['hm_xae']
                n['hm_ph'] = 0

        # ---------------- finish ---------------
        elif ph == 36:
            n['nrm_in'] = s['hq_l'] & 0xFFF
            n['nrm_nm'] = s['hq_df']
            n['hm_ph'] = 37
        elif ph in (37, 38, 39):
            n['hm_ph'] = ph + 1
        elif ph == 40:
            n['m_a'] = s['hn']
            n['m_b'] = s['rrom_q']
            n['hm_ph'] = 41
        elif ph in (41, 42):
            n['hm_ph'] = ph + 1
        elif ph == 43:
            if s['hw_pend']:
                v_dp = s['m_p'] >> 3
                v_dp = max(-134217728, min(134217727, v_dp))
                n['sd_wd'] = (s['sd_wd'] & ~0xFFFFFFF) | uns(v_dp, 28)
            n['hm_ph'] = 44
        elif ph == 44:
            if s['hw_pend']:
                n['sd_wa'] = (s['eng_bank'] << 8) | s['hm_slot']
                w = s['sd_wd'] & 0xFFFFFFF
                w |= (uns(s['hw_xn'], 11) << 46) | (uns(s['hw_f0'], 18) << 28)
                n['sd_wd'] = w
                n['sd_we'] = 1
                n['hm_slot'] = uns(s['hm_slot'] + 1, 8)
                n['hw_pend'] = 0
            n['hm_ph'] = 45
        elif ph == 45:
            # TRAILING GROUND sentinel: x_next = 2047 never matches
            n['sd_wa'] = (s['eng_bank'] << 8) | s['hm_slot']
            n['sd_wd'] = 0x7FF << 46
            n['sd_we'] = 1
            n['hm_ph'] = 57
        else:
            n['e_st'] = 'E_IDLE'

    # -- run one line -------------------------------------------------------
    def line(self, y, limit=200000):
        dy = y - self.s_cy
        self.reset(dy)
        while self.s['e_st'] != 'E_IDLE' and self.cycles < limit:
            self.step()
        # one more cycle to flush the last sd_we
        self.step()
        return self.seeds


# -------------------------------------------------------------- pixel path
def replay(seeds, W, stale_word=0):
    """Cycle-accurate p_fd + p_seedram.  Returns luma per fd_px."""
    ram = [(0x7FF, 0, 0)] * 256
    for slot, xn, f0, d0 in seeds:
        ram[slot] = (xn, f0, d0)

    sd_ra, sd_q = 0, (0x7FF, 0, 0)
    fd_f = fd_d = 0
    fd_xn = 0x7FF
    fd_ptr = 0
    fd_px = 0
    fd_ld = 0
    out = []
    # cycles -2,-1 = blanking (sd_ra parked on word 0); 0 = avid rise
    for c in range(-2, W + 4):
        nsd_ra, nsd_q = sd_ra, ram[sd_ra & 0xFF]
        nf, nd, nxn, nptr, npx, nld = fd_f, fd_d, fd_xn, fd_ptr, fd_px, fd_ld
        if c < 0:
            nsd_ra = 0                       # blanking park: prefetch word 0
        elif c == 0:
            nsd_ra = 1                       # word 0 is already in sd_q
            nptr, npx, nld = 0, 0, 1
        elif fd_ld == 1:
            nf = sd_q[1] << 6
            nd = sd_q[2]
            nxn = sd_q[0]
            nptr = 1
            nsd_ra = 1
            nld = 0
        else:
            if fd_px == fd_xn:
                nf = sd_q[1] << 6
                nd = sd_q[2]
                nxn = sd_q[0]
                nptr = uns(fd_ptr + 1, 8)
                nsd_ra = fd_ptr + 1
            else:
                nf = fd_f + fd_d
            npx = uns(fd_px + 1, 11)
        # fold
        v_f = fd_f >> 6
        v_t = 0 if v_f < 0 else (65536 if v_f > 65536 else v_f)
        v_tr = v_t if v_t < 32768 else 65536 - v_t
        out.append(min(255, v_tr >> 7))
        sd_ra, sd_q = nsd_ra, nsd_q
        fd_f, fd_d, fd_xn, fd_ptr, fd_px, fd_ld = nf, nd, nxn, nptr, npx, nld
    return out


# ------------------------------------------------------------------ oracle
def ref_line(y, W, cx, cy, spac, ex, ey, nt=C_NT):
    """exact float field, same geometry"""
    import math
    cs = [(cx + (ex[min(k, len(ex) - 1)]), cy + ey[min(k, len(ey) - 1)]) for k in range(nt)]
    R = [(k + 1) * spac for k in range(nt)]
    out = []
    for x in range(W):
        r = [math.hypot(x - cs[k][0], y - cs[k][1]) for k in range(nt)]
        j = next((k for k in range(nt) if r[k] < R[k]), None)
        if j is None:
            out.append(0)
            continue
        if j == 0:
            f = r[0] / R[0]
        else:
            di = r[j - 1] - R[j - 1]
            do = R[j] - r[j]
            f = di / max(di + do, 1e-9)
        f = min(max(f, 0.0), 1.0)
        out.append(int(round((1 - abs(2 * f - 1)) * 255)))
    return out


# -------------------------------------------------------------------- main
def main():
    W, H = 480, 270
    cx, cy = W // 2, H // 2
    spac = 14
    ex = [0] * C_NC
    ey = [0] * C_NC

    eng = Engine(W, H, cx, cy, spac, ex, ey)

    if '--image' in sys.argv:
        import numpy as np
        from PIL import Image
        got = np.zeros((H, W), np.uint8)
        want = np.zeros((H, W), np.uint8)
        maxcyc = 0
        maxslot = 0
        for y in range(H):
            seeds = eng.line(y)
            maxcyc = max(maxcyc, eng.cycles)
            maxslot = max(maxslot, len(seeds))
            lum = replay(seeds, W)
            got[y] = lum[5:W + 5]
            want[y] = ref_line(y, W, cx, cy, spac, ex, ey)
        err = np.abs(got.astype(int) - want.astype(int))
        print(f'cycles/line max {maxcyc}   seeds/line max {maxslot}')
        print(f'mean err {err.mean():.2f}  p99 {np.percentile(err,99):.0f}  max {err.max()}')
        Image.fromarray(np.concatenate([got, want,
                        (err * 4).clip(0, 255).astype(np.uint8)], 0)).save(
            '/home/ron/videomancer-dev/programs/ron/nacre/model_check.png')
        print('wrote model_check.png (MODEL / REF / err x4)')
        return

    if '--knots' in sys.argv:
        import math
        y = cy + (int(sys.argv[sys.argv.index('--knots') + 1])
                  if len(sys.argv) > sys.argv.index('--knots') + 1 else 0)
        eng.line(y)
        print(f'knot audit, line y={y}  (dy={y-cy})')
        print(f'{"x":>5} {"k":>3} {"sd":>2} '
              f'{"q6a":>8} {"true_ri*64":>10} '
              f'{"q6b":>8} {"true_ro*64":>10} '
              f'{"f":>7} {"true_f":>7}  err')
        for t in eng.trace[:60]:
            ri = math.hypot(t['x'] - t['icx'], math.sqrt(t['idy2']))
            ro = math.hypot(t['x'] - t['ocx'], math.sqrt(t['ody2']))
            di = max(ri - t['iR'], 0.0)
            do = max(t['oR'] - ro, 0.0)
            tf = di / max(di + do, 1e-9)
            tf = min(max(tf, 0.0), 1.0) * 65536
            print(f'{t["x"]:5d} {t["k"]:3d} {t["side"]:2d} '
                  f'{t["q6a"]:8d} {ri*64:10.1f} '
                  f'{t["q6b"]:8d} {ro*64:10.1f} '
                  f'{t["f"]:7d} {tf:7.0f}  {t["f"]-tf:+8.0f}')
        return

    y = cy                      # the centre scanline: every ring is crossed
    seeds = eng.line(y)
    print(f'line y={y}  cycles={eng.cycles}  seeds={len(seeds)}  lowp={eng.s["hm_lowp"]}')
    print(f'kstep={eng.s_kstep} rmax={eng.s_rmax} spac={spac}')
    print('\nspan store (L / R edges):')
    for k in range(C_NT):
        print(f'  ring {k:2d}: L={eng.spst[k]:5d}  R={eng.spst[C_NT+k]:5d}')
    print('\nfirst 24 seed words (slot, x_next, f0, d0):')
    for w in seeds[:24]:
        print(f'  slot {w[0]:3d}  x_next {w[1]:5d}  f0 {w[2]:7d}  d0 {w[3]:9d}')
    xs = [w[1] for w in seeds]
    bad = [i for i in range(1, len(xs) - 1) if xs[i] <= xs[i - 1]]
    print(f'\nnon-monotonic x_next at seed indices: {bad[:20]}')
    lum = replay(seeds, W)
    ref = ref_line(y, W, cx, cy, spac, ex, ey)
    got = lum[5:W + 5]
    err = [abs(a - b) for a, b in zip(got, ref)]
    print(f'replay mean err {sum(err)/len(err):.1f}  max {max(err)}')
    print('x    model  ref')
    for x in range(cx, min(cx + 60, W), 2):
        print(f'{x:4d}  {got[x]:5d}  {ref[x]:4d}')


if __name__ == '__main__':
    main()
