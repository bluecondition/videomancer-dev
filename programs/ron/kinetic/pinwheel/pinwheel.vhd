-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Pinwheel v0.1.0 -- Kinetic.
--
-- A field of rotors, one per square cell: N blade pairs as alternately coloured sectors, a hub dot in the
-- accent colour (pinwheel quilts, op-art fans).  Twist bends the blades into spirals.
--
--   element  the pinwheel of one cell (pitch P); state = its rotation angle alpha (13 bits per turn)
--   theta    = Spread * index - phase;  index: Linear = i + j, Radial = octagonal distance (|i|, |j|)
--   alpha    = Depth * wave(theta) + source          (wave: sine / snap; source: the cell's held luma, or Osc)
--   size     disc radius = (0.75 + 0.25 * source) * 0.46 P: the source also swells the rotor (a per-field table)
--   polar    cell-local (dx, dy) folded into the first octant: ratio = min * recip(max) (block-RAM reciprocal
--            and one multiply), angle = ratio + D(ratio) (atan table), radius = max * sec (second word of the
--            same table); unfolded by octant, mirrored in odd cells (Hand = Alternate)
--   blades   sp = N * (angle - alpha - Twist * r)  mod one turn;  colour A where sp < 1/2, B elsewhere;
--            blade pair 0's A blade is drawn in the accent colour (the marker that makes the rotation readable)
--   edges    blade coverage from the distance to sp = 0 or 1/2 times the radius (the radius capped where the
--            twist's radial gradient takes over), disc edge and hub from the radius; 9 steps each
--   colour   one read of a per-field table T(rim / hub, edge coverage, blade coverage) holding the blends of
--            A, B with the ground (rim) or the accent (hub); luma 10 bits, chroma 8 bits until the output filter
--
--   K1  Scale    cell pitch 64 .. 477 virtual lines (exponential, glided)
--   K2  Shape    blade pairs 2, 3, 4, 5, 6, 8, 10, 12
--   K3  Depth    rotation swing 0 .. 1.25 turns (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Spread   phase step per cell 0 .. 1/2 cycle (square law)
--   K6  Colour   the 16 shared palettes (ground, A and B = blades, accent = hub)
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Hand     Alternate (neighbours are mirror images) / Same
--   S10 Tile     Disc (round rotors on the ground) / Square (the sectors fill the cell)
--   S11 Source   Video / Osc
--   P12 TWIST    0 .. 2 turns of blade bend at the cell edge (square law, glided)
--
-- Chassis sections lifted from louver.vhd:
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-cell hold
--   [D] wave engine: cell accumulators -> theta -> wave ROM -> state
--   [E] the look: cell-local polar, blades, coverage, the blend table
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 36) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 409 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.pinwheel_pkg.all;
use work.all;

architecture pinwheel of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(409, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 36;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob

    -- [A] virtual-1080 steps from the timing ID: y step per field line (Q8), x step in half virtual px
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- field lines from the top to the screen centre (540 virtual lines)
    function f_half(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(121, 11);
            when 4          => return to_unsigned(240, 11);
            when 8          => return to_unsigned(144, 11);
            when 12         => return to_unsigned(288, 11);
            when 5 | 6 | 14 => return to_unsigned(360, 11);
            when 1 | 2 | 10 => return to_unsigned(270, 11);
            when others     => return to_unsigned(540, 11);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- gated operands: every multiply in this file is a sum of these (one adder per two operand bits per stage)
    function f_gu(x : unsigned; b : std_logic; w : natural) return unsigned is
        variable r : unsigned(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    function f_gs(x : signed; b : std_logic; w : natural) return signed is
        variable r : signed(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (one read port each, registered address and data, then a fabric register)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_rc_a : unsigned(8 downto 0) := (others => '0');
    signal r_rc_q : unsigned(15 downto 0) := (others => '0');
    signal r_at_a : unsigned(9 downto 0) := (others => '0');
    signal r_at_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [C] source hold: one smeared luma sample per cell column, written on the row before each cell row
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal sl : t_r16 := (others => (others => '0'));
    attribute no_rw_check of sl : signal is true;
    signal sl_ra, sl_wa : unsigned(7 downto 0) := (others => '0');
    signal sl_q, sl_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal sl_we : std_logic := '0';
    -- disc radius by source value, refilled every field: (192 + s / 4) / 256 of the full radius (Q2 half px)
    signal rt : t_r16 := (others => (others => '0'));
    attribute no_rw_check of rt : signal is true;
    signal rt_ra, rt_wa : unsigned(7 downto 0) := (others => '0');
    signal rt_q, rt_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal rt_we, rt_run : std_logic := '0';
    signal rt_acc : unsigned(22 downto 0) := (others => '0');
    signal rt_k   : unsigned(7 downto 0) := (others => '0');
    signal q13_rd, q14_rd, q15_rd, q16_rd : unsigned(11 downto 0) := (others => '0');
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    signal tr : t_r16 := (others => (others => '0'));          -- {sx, swap, max}: stage 5 -> 13
    attribute no_rw_check of tr : signal is true;
    signal tr_q, tr_f : std_logic_vector(15 downto 0) := (others => '0');
    signal ar : t_r16 := (others => (others => '0'));          -- alpha: stage 12 -> 17
    attribute no_rw_check of ar : signal is true;
    signal ar_q, ar_f : std_logic_vector(15 downto 0) := (others => '0');
    signal er : t_r16 := (others => (others => '0'));          -- {rim, edge coverage, AA mantissa, AA shift}: stage 18 -> 24
    attribute no_rw_check of er : signal is true;
    signal er_d, er_q, er_f : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: T(rim, cx, ca) = [(8 - cx)(8 B + ca (A - B)) + 8 cx K] / 64 with
    -- K = accent (hub) or ground (rim): {Y 10 bits, U 8, V 8}; address = rim & cx & ca
    type t_tt is array (0 to 511) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => x"00408080");               -- black with neutral chroma until the first fill
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(8 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_fs  : unsigned(1 downto 0) := "00";
    signal t_sel : std_logic := '0';
    signal t_cx, t_ca : unsigned(3 downto 0) := (others => '0');
    type t_s18 is array (0 to 2) of signed(17 downto 0);
    type t_s12 is array (0 to 2) of signed(11 downto 0);
    signal t_rs, t_stp, t_e : t_s18 := (others => (others => '0'));
    signal t_ab, t_kb : t_s12 := (others => (others => '0'));
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_r4 := (others => (others => '1'));           -- syncs: C_LAT
    attribute no_rw_check of sr : signal is true;
    signal sr_q, sr_f : std_logic_vector(3 downto 0) := (others => '1');
    signal rc   : unsigned(7 downto 0) := (others => '0');

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_sd   : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw rate, switches, palette, blade count
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(618 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(380 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_same, s_sq, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal s_ni  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    -- K6 Camera: knob (deadband +-4), glided gain k/256 in Q3 (k = 256: the shipped 1x; 512: 2x; 1023: 4x)
    signal s_kr  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_kt  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_kgd : signed(13 downto 0) := (others => '0');
    signal s_kg  : unsigned(12 downto 0) := to_unsigned(4096, 13);

    -- field serial multiplier (24 iterations), serial shifter, sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(47 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(23 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_nz  : std_logic := '0';
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : std_logic := '0';
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_mi  : unsigned(16 downto 0) := to_unsigned(65536, 17);
    signal z_tn  : unsigned(11 downto 0) := (others => '0');
    signal z_loc : signed(11 downto 0) := (others => '0');
    signal z_kp  : std_logic := '0';
    signal z_nrun, z_first : std_logic := '0';
    signal z_rrun : std_logic := '0';
    signal z_rn   : unsigned(9 downto 0) := (others => '0');
    signal z_racc, z_rthr : unsigned(15 downto 0) := (others => '0');

    -- per-field constants
    signal c_pw   : unsigned(19 downto 0) := to_unsigned(432 * 512, 20);   -- pitch in half virtual px, Q9
    signal c_p2   : unsigned(9 downto 0);                               -- pitch, half virtual px
    signal c_p2h  : unsigned(8 downto 0);                               -- to the cell centre
    signal c_sh   : unsigned(1 downto 0) := "10";                       -- the pitch's octave
    signal c_r    : unsigned(11 downto 0) := to_unsigned(796, 12);      -- disc radius, Q2 half px
    signal c_d    : unsigned(7 downto 0) := (others => '0');            -- depth, Q7 turns
    signal c_s    : unsigned(22 downto 0) := (others => '0');           -- spread per cell, Q24 cycles
    signal c_twm  : unsigned(9 downto 0) := (others => '0');            -- twist per half px (mantissa)
    signal c_rc   : unsigned(9 downto 0) := (others => '1');            -- radius where the twist gradient takes over
    signal c_n    : unsigned(3 downto 0) := to_unsigned(4, 4);          -- blade pairs
    signal c_mn   : unsigned(1 downto 0) := "00";                       -- AA gain mantissa: 0.75 / 1 / 1.25
    signal c_shb  : unsigned(3 downto 0) := to_unsigned(10, 4);         -- AA shift base
    signal c_ymp  : signed(18 downto 0) := (others => '0');             -- y step - pitch (Q7)
    signal c_x0   : unsigned(9 downto 0) := (others => '0');            -- pixel 0: offset in its cell, cell parity, Spread * i
    signal c_ip0  : std_logic := '0';
    signal c_si0  : signed(27 downto 0) := (others => '0');
    signal c_ph   : unsigned(25 downto 0) := (others => '0');           -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    -- palette, raw: luma 10 bits, chroma 8 bits
    signal c_gy, c_ay, c_by, c_ky : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_gc, c_ac, c_bc, c_kc : t_u8 := (others => to_unsigned(128, 8));

    -- [B] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- [C] source
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_sv  : unsigned(7 downto 0) := (others => '0');
    signal s_sav, s_savd, s_srow : std_logic := '0';
    signal s_y1 : unsigned(13 downto 0) := (others => '0');
    -- K6 Camera: smeared sample x gain, three stages of gated sums (8 x 10 bits) -> 10 bits (1.0 = 256)
    signal m1_p0, m1_p1, m1_p2, m1_p3 : unsigned(11 downto 0) := (others => '0');
    signal m2_a, m2_b : unsigned(13 downto 0) := (others => '0');
    signal s_yg : unsigned(9 downto 0) := (others => '0');

    -- [D] wave engine: line registers (the NEXT line's start values) and pixel accumulators (stage 1)
    signal ls_sj  : signed(27 downto 0) := (others => '0');
    signal ls_y   : unsigned(16 downto 0) := (others => '0');
    signal ls_n1  : unsigned(16 downto 0) := (others => '0');
    signal ls_n2  : signed(18 downto 0) := (others => '0');
    signal ls_dy  : signed(10 downto 0) := (others => '0');
    signal ls_ay  : unsigned(8 downto 0) := (others => '0');
    signal ls_jp  : std_logic := '0';
    signal l_sj   : signed(27 downto 0) := (others => '0');
    signal l_ay   : unsigned(8 downto 0) := (others => '0');
    signal l_sy   : std_logic := '0';
    signal a_si   : signed(27 downto 0) := (others => '0');
    signal a_x    : unsigned(9 downto 0) := (others => '0');
    signal a_ip, a_nw : std_logic := '0';
    signal s_nw   : std_logic_vector(2 to 9) := (others => '0');
    signal q2_ask, q2_asu : unsigned(18 downto 0) := (others => '0');
    signal q2_lin : unsigned(13 downto 0) := (others => '0');
    signal q3_ge  : std_logic := '0';
    signal q3_ask, q3_asu : unsigned(14 downto 0) := (others => '0');
    signal q3_lin, q4_xa, q4_xb, q5_th : unsigned(13 downto 0) := (others => '0');
    signal s_neg  : std_logic_vector(6 to 11) := (others => '0');
    signal q8_w   : unsigned(9 downto 0) := (others => '0');
    signal q9_p0, q9_p1, q9_p2, q9_p3 : unsigned(11 downto 0) := (others => '0');
    signal q10_qa, q10_qb : unsigned(13 downto 0) := (others => '0');
    signal q10_src : unsigned(9 downto 0) := (others => '0');
    signal q11_aw  : unsigned(13 downto 0) := (others => '0');
    signal q11_st  : unsigned(12 downto 0) := (others => '0');
    signal q12_al  : unsigned(12 downto 0) := (others => '0');
    signal k8, k9, k10 : unsigned(7 downto 0) := (others => '0');

    -- [E] the look
    signal q2_dx  : signed(10 downto 0) := (others => '0');
    signal q2_ip, q3_sx, q4_sx, q4_swap, q5_sx, q5_swap : std_logic := '0';
    signal q3_ax, q4_ax, q5_mn, q6_mn, q7_mn : unsigned(8 downto 0) := (others => '0');
    signal q7_rm  : unsigned(10 downto 0) := (others => '0');
    signal q7_l, q8_l, q9_l, q10_l : unsigned(3 downto 0) := (others => '0');
    signal q8_r0, q8_r1, q8_r2, q8_r3 : unsigned(12 downto 0) := (others => '0');
    signal q8_r4, q9_rc : unsigned(10 downto 0) := (others => '0');
    signal q9_ra, q9_rb : unsigned(14 downto 0) := (others => '0');
    signal q10_pr : unsigned(19 downto 0) := (others => '0');
    signal q12_rt, q13_rt : unsigned(9 downto 0) := (others => '0');
    signal q13_d, q13_s : unsigned(7 downto 0) := (others => '0');
    signal q14_t  : unsigned(10 downto 0) := (others => '0');
    signal q14_m0, q14_m1, q14_m2, q14_m3 : unsigned(10 downto 0) := (others => '0');
    signal q14_mx, q15_mx : unsigned(8 downto 0) := (others => '0');
    signal q14_swap, q14_sx, q15_sx : std_logic := '0';
    signal q15_t2 : unsigned(11 downto 0) := (others => '0');
    signal q15_ma, q15_mb : unsigned(12 downto 0) := (others => '0');
    signal q16_t3, q17_phi, q17_al, q18_pa, q19_pa, q19_tw, q20_a : unsigned(12 downto 0) := (others => '0');
    signal q16_r2 : unsigned(11 downto 0) := (others => '0');
    signal q17_w0, q17_w1, q17_w2, q17_w3, q17_w4, q18_wc : unsigned(11 downto 0) := (others => '0');
    signal q18_wa, q18_wb : unsigned(13 downto 0) := (others => '0');
    signal q17_da, q17_db : signed(12 downto 0) := (others => '0');
    signal q17_sel : std_logic := '0';
    signal q17_re : unsigned(9 downto 0) := (others => '0');
    signal q21_n0, q21_n1 : unsigned(14 downto 0) := (others => '0');
    signal q22_w  : unsigned(16 downto 0) := (others => '0');
    signal q23_mk, q24_mk, q25_mk, q26_mk : std_logic := '0';
    signal q23_d  : unsigned(10 downto 0) := (others => '0');
    signal q23_ina, q24_ina, q25_ina, q26_ina : std_logic := '0';
    signal q24_v1 : unsigned(11 downto 0) := (others => '0');
    signal q25_v2 : unsigned(12 downto 0) := (others => '0');
    signal q25_shq : unsigned(3 downto 0) := (others => '0');
    signal q25_sx, q26_sx : std_logic_vector(4 downto 0) := (others => '0');
    signal q26_off : unsigned(2 downto 0) := (others => '0');
    signal q29_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q29_c  : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    c_p2  <= c_pw(18 downto 9);
    c_p2h <= c_pw(18 downto 10);

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, hold, delay rings, blend table (one write statement each;
    -- none ever reads the address it is writing)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_PW_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_PW_SIN(to_integer(r_w_a));
            r_rc_q <= C_PW_RCP(to_integer(r_rc_a));
            r_at_q <= C_PW_AT(to_integer(r_at_a));
            r_pl_q <= C_PW_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_sl : process(clk)
    begin
        if rising_edge(clk) then
            sl_q <= sl(to_integer(sl_ra));
            if sl_we = '1' then sl(to_integer(sl_wa)) <= sl_wd; end if;
        end if;
    end process p_sl;

    p_rt : process(clk)
    begin
        if rising_edge(clk) then
            rt_q <= rt(to_integer(rt_ra));
            if rt_we = '1' then rt(to_integer(rt_wa)) <= rt_wd; end if;
            -- fill (zone, zc 90 .. 346): one accumulator
            rt_we <= '0';
            if s_zone = '1' and s_zc = 90 then
                rt_run <= '1'; rt_k <= (others => '0');
                rt_acc <= shift_left(resize(c_r, 23), 11) + shift_left(resize(c_r, 23), 10);
            elsif rt_run = '1' then
                rt_we <= '1'; rt_wa <= rt_k; rt_wd <= "00000" & std_logic_vector(rt_acc(22 downto 12));
                rt_acc <= rt_acc + shift_left(resize(c_r, 23), 2);
                rt_k <= rt_k + 1;
                if rt_k = 255 then rt_run <= '0'; end if;
            end if;
        end if;
    end process p_rt;

    p_tt : process(clk)
    begin
        if rising_edge(clk) then
            tt_q <= tt(to_integer(tt_ra));
            if tt_we = '1' then tt(to_integer(tt_wa)) <= tt_wd; end if;
        end if;
    end process p_tt;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            tr(to_integer(rc + 6)) <= "00000" & q5_sx & q5_swap & std_logic_vector(r_rc_a);
            tr_q <= tr(to_integer(rc));
            tr_f <= tr_q;
            ar(to_integer(rc + 3)) <= "000" & std_logic_vector(q12_al);
            ar_q <= ar(to_integer(rc));
            ar_f <= ar_q;
            er(to_integer(rc + 4)) <= er_d;
            er_q <= er(to_integer(rc));
            er_f <= er_q;
            cr(to_integer(rc + 6)) <= std_logic_vector(q29_c(1)) & std_logic_vector(q29_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, per-field sequencer (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_t  : unsigned(28 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_en : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 2047 then
                s_zc <= s_zc + 1;
            end if;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- Rate: bipolar with a +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_snap <= registers_in(6)(0);
                s_rad  <= registers_in(6)(1);
                s_same <= registers_in(6)(2);
                s_sq   <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); s_sd <= '1';
                    when 3      => s_oxs <= to_unsigned(153, 9); s_sd <= '0';
                    when others => s_oxs <= to_unsigned(102, 9); s_sd <= '0';
                end case;
                -- Colour: 16 palettes with hysteresis; Shape: 8 blade counts with hysteresis
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
                v_d := signed(resize(unsigned(registers_in(1)), 11)) - signed(resize(s_ni & "1000000", 11));
                if v_d > 76 or v_d < -76 then s_ni <= unsigned(registers_in(1)(9 downto 7)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale and Twist (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 1 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k (no zc decode on the
            -- multiplier's path).
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) /= 10
               and s_zc(8 downto 5) /= 11 and s_zc(8 downto 5) /= 13 and s_zc(8 downto 5) /= 15 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(5950, 24);      -- pitch exponent
                    when 3  =>                                                                                -- depth^2
                        z_fa(12 downto 3) <= unsigned(registers_in(2)); z_fb(12 downto 3) <= unsigned(registers_in(2));
                    when 4  => z_fa(12 downto 0) <= s_gp;                 z_fb <= resize(s_gp, 24);           -- twist^2
                    when 5  => z_fa(16 downto 0) <= z_mi;                 z_fb <= resize(z_tn, 24);           -- twist / pitch
                    when 7  =>                                                                                -- spread^2
                        z_fa(12 downto 3) <= unsigned(C_KFIX_5); z_fb(12 downto 3) <= unsigned(C_KFIX_5);
                    when 12 => z_fa(8 downto 0)  <= s_vm;                 z_fb <= to_unsigned(900, 24);       -- rate exponent
                    when others => z_fa(15 downto 0) <= z_mt;             z_fb <= to_unsigned(1193046, 24);   -- rate (14)
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(46 downto 0) & '0';
                f_b <= '0' & f_b(23 downto 1);
            end if;

            -- serial shifter for the 2^n scalings (no barrels)
            if z_shn /= 0 then
                z_shn <= z_shn - 1;
                if z_sht = '0' then c_pw <= c_pw(18 downto 0) & '0';
                else c_dph <= c_dph(20 downto 0) & '0'; end if;
            end if;

            -- the cell origin: cell (0, 0) is centred on the screen centre; step back to the first pixel / line
            if z_nrun = '1' then
                if z_loc(11) = '1' then
                    z_loc <= z_loc + signed(resize(c_p2, 12));
                    z_kp  <= not z_kp;
                else
                    z_nrun <= '0';
                end if;
            end if;

            -- the radius where the twist has turned one radian (the AA's radial limit): count the steps
            if z_rrun = '1' then
                if z_racc >= z_rthr or z_rn = 1023 then
                    z_rrun <= '0'; c_rc <= z_rn;
                else
                    z_racc <= z_racc + c_twm; z_rn <= z_rn + 1;
                end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- wave phase (last field's step): one adder, add or subtract
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    -- palette: ground, A, B, accent (chroma kept to 8 bits)
                    when 22 => r_pl_a <= "00" & s_pal & "00";
                    when 25 =>
                        c_gy <= unsigned(r_pl_q(31 downto 22)); c_gc(1) <= unsigned(r_pl_q(21 downto 14)); c_gc(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "01";
                    when 28 =>
                        c_ay <= unsigned(r_pl_q(31 downto 22)); c_ac(1) <= unsigned(r_pl_q(21 downto 14)); c_ac(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "10";
                    when 31 =>
                        c_by <= unsigned(r_pl_q(31 downto 22)); c_bc(1) <= unsigned(r_pl_q(21 downto 14)); c_bc(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "11";
                    when 34 =>
                        c_ky <= unsigned(r_pl_q(31 downto 22)); c_kc(1) <= unsigned(r_pl_q(21 downto 14)); c_kc(2) <= unsigned(r_pl_q(11 downto 4));

                    -- pitch: e = 1 + k1 * 2.9 octaves (Q8), P = 64 * 2^e half px, and 2^-frac(e)
                    when 60 => z_e <= f_acc(26 downto 16) + 256;
                    when 61 =>
                        r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                        if z_e(7 downto 0) = 0 then z_nz <= '0'; else z_nz <= '1'; end if;
                    when 63 => z_mt <= r_ex_q;
                    when 64 =>
                        c_pw <= "0000" & z_mt; z_shn <= z_sh; z_sht <= '0';
                        c_sh <= z_sh(1 downto 0);
                        r_ex_a <= 0 - z_e(7 downto 0);
                    when 66 => z_mt <= r_ex_q;
                    when 72 =>
                        if z_nz = '1' then z_mi <= '0' & z_mt; else z_mi <= z_mt & '0'; end if;
                    when 80 =>
                        c_r <= resize(c_p2 & '0', 12) - resize(c_p2(9 downto 3), 12) - resize(c_p2(9 downto 5), 12);
                        c_ymp <= signed(resize(s_ys, 19)) - signed(resize(c_p2 & "0000000", 19));
                    when 82 =>
                        case to_integer(s_ni) is
                            when 0      => c_n <= to_unsigned(2, 4);  c_mn <= "00"; v_en := to_unsigned(1, 3);
                            when 1      => c_n <= to_unsigned(3, 4);  c_mn <= "01"; v_en := to_unsigned(2, 3);
                            when 2      => c_n <= to_unsigned(4, 4);  c_mn <= "00"; v_en := to_unsigned(2, 3);
                            when 3      => c_n <= to_unsigned(5, 4);  c_mn <= "10"; v_en := to_unsigned(3, 3);
                            when 4      => c_n <= to_unsigned(6, 4);  c_mn <= "01"; v_en := to_unsigned(3, 3);
                            when 5      => c_n <= to_unsigned(8, 4);  c_mn <= "00"; v_en := to_unsigned(3, 3);
                            when 6      => c_n <= to_unsigned(10, 4); c_mn <= "10"; v_en := to_unsigned(4, 3);
                            when others => c_n <= to_unsigned(12, 4); c_mn <= "01"; v_en := to_unsigned(4, 3);
                        end case;
                        if s_sd = '1' then c_shb <= to_unsigned(9, 4) + v_en; else c_shb <= to_unsigned(8, 4) + v_en; end if;
                    -- K6 Camera: knob with a +-4 deadband (ends snap to 0 / 1023), then ease 1/8 of the gap
                    when 96 => s_kr <= unsigned(registers_in(5));
                    when 97 =>
                        v_d := signed(resize(s_kr, 11)) - signed(resize(s_kt, 11));
                        if s_kr < 4 then s_kt <= (others => '0');
                        elsif s_kr > 1019 then s_kt <= (others => '1');
                        elsif v_d > 4 or v_d < -4 then s_kt <= s_kr; end if;
                    when 98 => s_kgd <= signed(resize(s_kt & "000", 14)) - signed(resize(s_kg, 14));
                    when 99 =>
                        if C_SIM_SNAP or (s_kgd < 8 and s_kgd > -8) then s_kg <= s_kt & "000";
                        else s_kg <= unsigned(signed(s_kg) + resize(shift_right(s_kgd, 3), 13)); end if;
                    when 156 =>                      -- depth = (k3 / 1023)^2 * 1.25 turns
                        v_t := shift_left(resize(f_acc(25 downto 0), 29), 2) + resize(f_acc(25 downto 0), 29);
                        c_d <= v_t(28 downto 21);
                    when 188 => z_tn <= f_acc(25 downto 14);
                    when 220 => c_twm <= f_acc(27 downto 18);
                    when 284 => c_s <= f_acc(25 downto 3);
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= '1';
                    -- cell origin along x (960 or 720 virtual px to the centre), then along y (540)
                    when 550 =>
                        if s_sd = '1' then z_loc <= signed(resize(c_p2h, 12)) - 1440;
                        else z_loc <= signed(resize(c_p2h, 12)) - 1920; end if;
                        z_kp <= '0'; z_nrun <= '1';
                    when 620 =>
                        c_x0 <= unsigned(z_loc(9 downto 0)); c_ip0 <= z_kp; c_si0 <= ls_sj;
                    when 630 =>
                        z_loc <= signed(resize(c_p2h, 12)) - 1080;
                        z_kp <= '0'; z_nrun <= '1';
                    when 700 => z_first <= '1';
                    when 710 =>
                        z_rrun <= '1'; z_rn <= (others => '0'); z_racc <= (others => '0');
                        case c_sh is
                            when "00"   => z_rthr <= to_unsigned(2608, 16);
                            when "01"   => z_rthr <= to_unsigned(5216, 16);
                            when "10"   => z_rthr <= to_unsigned(10432, 16);
                            when others => z_rthr <= to_unsigned(20864, 16);
                        end case;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            if g_start = '1' then z_first <= '0'; end if;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Blend table fill (zone, from zc 40; adders only).  For rim = 0 / 1 (K =
    -- accent / ground), cx = 0 .. 8, ca = 0 .. 8:
    --   T = [(8 - cx)(8 B + ca (A - B)) + 8 cx K + 32] / 64
    -- row start rs = 64 B + 32 + 8 cx (K - B), step = (8 - cx)(A - B).
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_a, v_b, v_k : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            if s_zone = '1' and s_zc = 40 then
                t_fs <= "01"; t_sel <= '0';
            else
                case t_fs is
                    when "01" =>                 -- a new K: differences, first row
                        for i in 0 to 2 loop
                            if i = 0 then
                                v_a := signed(resize(c_ay, 12)); v_b := signed(resize(c_by, 12));
                                if t_sel = '1' then v_k := signed(resize(c_gy, 12)); else v_k := signed(resize(c_ky, 12)); end if;
                            else
                                v_a := signed(resize(c_ac(i), 12)); v_b := signed(resize(c_bc(i), 12));
                                if t_sel = '1' then v_k := signed(resize(c_gc(i), 12)); else v_k := signed(resize(c_kc(i), 12)); end if;
                            end if;
                            t_ab(i) <= v_a - v_b;
                            t_kb(i) <= v_k - v_b;
                            t_rs(i) <= shift_left(resize(v_b, 18), 6) + 32;
                        end loop;
                        t_cx <= (others => '0');
                        t_fs <= "10";
                    when "10" =>                 -- row start
                        for i in 0 to 2 loop
                            t_e(i) <= t_rs(i);
                            if t_cx = 0 then t_stp(i) <= shift_left(resize(t_ab(i), 18), 3);
                            else t_stp(i) <= t_stp(i) - resize(t_ab(i), 18); end if;
                            t_rs(i) <= t_rs(i) + shift_left(resize(t_kb(i), 18), 3);
                        end loop;
                        t_ca <= (others => '0');
                        t_fs <= "11";
                    when "11" =>                 -- cells
                        tt_we <= '1'; tt_wa <= t_sel & t_cx & t_ca;
                        tt_wd <= "000000" & std_logic_vector(t_e(0)(15 downto 6)) & std_logic_vector(t_e(1)(13 downto 6))
                                 & std_logic_vector(t_e(2)(13 downto 6));
                        for i in 0 to 2 loop t_e(i) <= t_e(i) + t_stp(i); end loop;
                        t_ca <= t_ca + 1;
                        if t_ca = 8 then
                            t_cx <= t_cx + 1;
                            if t_cx /= 8 then t_fs <= "10";
                            elsif t_sel = '0' then t_sel <= '1'; t_fs <= "01";
                            else t_fs <= "00"; end if;
                        end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [B] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
    ---------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;

            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if g_avr = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;

            if g_start = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- [C] Source field.  Osc: a two-axis triangle lattice (640 x 540 virtual
    -- px) drifting diagonally.  The luma (or the lattice) is smeared along x by
    -- a 1/64 filter; p_wave samples it once per cell column on the row before
    -- each cell row and holds it for the whole cell, so every rotor is rigid.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
        variable v_m18 : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_tx := f_tri7(s_ox(15 downto 8)); v_ty := f_tri7(s_oy(15 downto 8));
            v_s := resize(v_tx, 8) + resize(v_ty, 8);
            v_o := v_s - shift_right(v_s, 3);

            -- input sample: source value (8 bits), held through blanking
            s_sav <= data_in.avid; s_savd <= s_sav;
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_o;
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                end if;
            end if;

            -- smear along the line
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
                s_srow <= z_first or not ls_n2(18);     -- line 0, or the last row before a new cell row
            elsif s_sav = '1' then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 6), 14));
            end if;

            -- K6 Camera: the smeared sample x the glided gain (k / 256), one multiply over three stages
            -- (3 px later than the smear: invisible against its 64 px)
            m1_p0 <= f_gu(s_kg(12 downto 3), s_y1(6), 12)  + shift_left(f_gu(s_kg(12 downto 3), s_y1(7), 12), 1);
            m1_p1 <= f_gu(s_kg(12 downto 3), s_y1(8), 12)  + shift_left(f_gu(s_kg(12 downto 3), s_y1(9), 12), 1);
            m1_p2 <= f_gu(s_kg(12 downto 3), s_y1(10), 12) + shift_left(f_gu(s_kg(12 downto 3), s_y1(11), 12), 1);
            m1_p3 <= f_gu(s_kg(12 downto 3), s_y1(12), 12) + shift_left(f_gu(s_kg(12 downto 3), s_y1(13), 12), 1);
            m2_a <= resize(m1_p0, 14) + shift_left(resize(m1_p1, 14), 2);
            m2_b <= resize(m1_p2, 14) + shift_left(resize(m1_p3, 14), 2);
            v_m18 := resize(m2_a, 18) + shift_left(resize(m2_b, 18), 4);
            s_yg <= v_m18(17 downto 8);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Wave engine.  Cell accumulators: the offset inside the cell (x per
    -- pixel, y per line; half virtual px), the cell parities, and Spread * i,
    -- Spread * j stepped at each cell wrap.  ls_* hold the NEXT line's values.
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_b   : signed(27 downto 0);
        variable v_a31 : signed(28 downto 0);
        variable v_sub, v_en : std_logic;
        variable v_n   : unsigned(10 downto 0);
        variable v_w   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- line registers: wrap candidates are pre-registered (ls_y is stable between line starts)
            ls_n1 <= resize(ls_y + s_ys, 17);
            ls_n2 <= signed(resize(ls_y, 19)) + c_ymp;
            ls_dy <= signed(resize(ls_y(16 downto 7), 11)) - signed(resize(c_p2h, 11));
            v_w := resize(abs(ls_dy), 12);
            ls_ay <= unsigned(v_w(8 downto 0));
            -- Spread * cell number: - S per step back in the zone's origin search, + S at each cell-row wrap (one adder)
            v_sub := z_nrun;
            v_en := (z_nrun and z_loc(11)) or (g_start and not ls_n2(18));
            v_b := signed(resize(c_s(22 downto 2), 28));
            if v_sub = '1' then v_b := not v_b; end if;
            v_a31 := (ls_sj & '1') + (v_b & v_sub);
            if s_zone = '1' and (s_zc = 550 or s_zc = 630) then
                ls_sj <= (others => '0');
            elsif v_en = '1' then
                ls_sj <= v_a31(28 downto 1);
            end if;
            if s_zone = '1' and s_zc = 700 then
                ls_y <= unsigned(z_loc(9 downto 0)) & "0000000"; ls_jp <= z_kp;
            elsif g_start = '1' then
                if ls_n2(18) = '0' then
                    ls_y <= unsigned(ls_n2(16 downto 0)); ls_jp <= not ls_jp;
                else
                    ls_y <= ls_n1;
                end if;
            end if;

            -- pixel accumulators (stage 1)
            if g_start = '1' then
                a_x <= c_x0; a_si <= c_si0; a_ip <= c_ip0; a_nw <= '0';
                l_sj <= ls_sj; l_ay <= ls_ay;
                l_sy <= ls_dy(10) xor (ls_jp and not s_same);
            elsif g_avid = '1' then
                v_n := resize(a_x, 11) + s_xs;
                v_w := signed(resize(v_n, 12)) - signed(resize(c_p2, 12));
                if v_w(11) = '0' then
                    a_x <= unsigned(v_w(9 downto 0));
                    a_si <= a_si + signed(resize(c_s(22 downto 2), 28));
                    a_ip <= not a_ip;
                    a_nw <= '1';
                else
                    a_x <= v_n(9 downto 0);
                    a_nw <= '0';
                end if;
            else
                a_nw <= '0';
            end if;
        end if;
    end process p_acc;

    p_wave : process(clk)
        variable v_a   : unsigned(19 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_al  : signed(14 downto 0);
        variable v_p   : unsigned(17 downto 0);
        variable v_k   : unsigned(7 downto 0);
        variable v_sel : unsigned(13 downto 0);
        variable v_st  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            s_nw(2) <= a_nw;
            for i in 3 to 9 loop s_nw(i) <= s_nw(i - 1); end loop;

            -- 2: magnitudes for the radial metric and the linear sum (14 fraction bits from here on)
            v_a := unsigned(abs(a_si(27 downto 8))); q2_ask <= v_a(18 downto 0);
            v_a := unsigned(abs(l_sj(27 downto 8))); q2_asu <= v_a(18 downto 0);
            q2_lin <= unsigned(a_si(21 downto 8)) + unsigned(l_sj(21 downto 8));
            -- 3: compare
            if q2_ask >= q2_asu then q3_ge <= '1'; else q3_ge <= '0'; end if;
            q3_ask <= q2_ask(14 downto 0); q3_asu <= q2_asu(14 downto 0); q3_lin <= q2_lin;
            -- 4: octagonal metric max + min / 2 (Radial) or i + j (Linear), less the phase
            if s_rad = '0' then
                v_sel := q3_lin; q4_xb <= (others => '0');
            elsif q3_ge = '1' then
                v_sel := q3_ask(13 downto 0); q4_xb <= q3_asu(14 downto 1);
            else
                v_sel := q3_asu(13 downto 0); q4_xb <= q3_ask(14 downto 1);
            end if;
            q4_xa <= v_sel - c_ph(25 downto 12);
            -- 5: theta (14 bits per cycle)
            q5_th <= q4_xa + q4_xb;
            -- 6: wave ROM address: quarter-wave fold; Snap = the first sixteenth stretched over the quarter
            v_pos := q5_th(11 downto 0);
            if q5_th(12) = '1' then v_pos := not v_pos; end if;
            if s_snap = '0' then r_w_a <= v_pos(11 downto 2);
            elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
            else r_w_a <= v_pos(9 downto 0); end if;
            s_neg(6) <= q5_th(13);
            for i in 7 to 11 loop s_neg(i) <= s_neg(i - 1); end loop;
            -- 7: ROM; 8: fabric copy
            q8_w <= r_w_q(9 downto 0);
            -- 9 .. 11: Depth * wave as gated sums (turns, 13 bits per turn)
            q9_p0 <= f_gu(q8_w, c_d(0), 12) + shift_left(f_gu(q8_w, c_d(1), 12), 1);
            q9_p1 <= f_gu(q8_w, c_d(2), 12) + shift_left(f_gu(q8_w, c_d(3), 12), 1);
            q9_p2 <= f_gu(q8_w, c_d(4), 12) + shift_left(f_gu(q8_w, c_d(5), 12), 1);
            q9_p3 <= f_gu(q8_w, c_d(6), 12) + shift_left(f_gu(q8_w, c_d(7), 12), 1);
            q10_qa <= resize(q9_p0, 14) + shift_left(resize(q9_p1, 14), 2);
            q10_qb <= resize(q9_p2, 14) + shift_left(resize(q9_p3, 14), 2);
            v_p := resize(q10_qa, 18) + shift_left(resize(q10_qb, 18), 4);
            q11_aw <= v_p(17 downto 4);

            -- source hold: read address at 8, RAM at 9, fabric at 10; the cell column counts from 0 at each line
            v_k := k8;
            if s_gav_sr(5) = '1' and s_gav_sr(6) = '0' then v_k := (others => '0');
            elsif s_nw(7) = '1' then v_k := k8 + 1; end if;
            k8 <= v_k;
            if s_gav_sr(5) = '1' then sl_ra <= v_k; else sl_ra <= (not v_k(7)) & v_k(6 downto 0); end if;
            k9 <= k8; k10 <= k9;
            q10_src <= unsigned(sl_q(9 downto 0));
            -- on the sample row, at the cell column's last pixel: store the smeared sample x the camera gain
            sl_wd <= "000000" & std_logic_vector(s_yg);
            sl_wa <= k10;
            if s_srow = '1' and s_gav_sr(8) = '1' and (s_nw(9) = '1' or s_gav_sr(7) = '0') then sl_we <= '1'; else sl_we <= '0'; end if;

            -- 11: source term (Video x10: white = 0.27 turn; Osc x20)
            -- (the held source is 10 bits since the K6 camera gain: 256 = the shipped 1x; rotation wraps mod a turn)
            v_st := shift_left(resize(q10_src, 13), 3) + shift_left(resize(q10_src, 13), 1);
            if s_src = '1' then q11_st <= v_st(11 downto 0) & '0'; else q11_st <= v_st; end if;
            if s_src = '0' then
                if q10_src(9 downto 8) /= "00" then rt_ra <= (others => '1'); else rt_ra <= q10_src(7 downto 0); end if;
            elsif q10_src(9 downto 7) /= "000" then rt_ra <= (others => '1');
            else rt_ra <= q10_src(6 downto 0) & '0'; end if;
            -- 12: the state: alpha = +-Depth * wave + source (13 bits per turn)
            v_al := signed(resize(q11_aw, 15));
            if s_neg(11) = '1' then v_al := -v_al; end if;
            v_al := v_al + signed(resize(q11_st, 15));
            q12_al <= unsigned(v_al(12 downto 0));
        end if;
    end process p_wave;

    ---------------------------------------------------------------------------
    -- [E] The look: cell-local polar coordinates, blades, coverage, blend table
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_ab  : signed(10 downto 0);
        variable v_r11 : unsigned(10 downto 0);
        variable v_m   : unsigned(16 downto 0);
        variable v_tw  : unsigned(19 downto 0);
        variable v_rh  : unsigned(9 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_c   : signed(12 downto 0);
        variable v_cx  : unsigned(3 downto 0);
        variable v_e   : unsigned(3 downto 0);
        variable v_m1, v_m0 : std_logic;
        variable v_dq  : unsigned(11 downto 0);
        variable v_sh  : unsigned(12 downto 0);
        variable v_ca  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: dx from the cell centre (half virtual px); 3: |dx|, its sign (mirrored in odd cells: Hand)
            q2_dx <= signed(resize(a_x, 11)) - signed(resize(c_p2h, 11));
            q2_ip <= a_ip;
            v_ab := abs(q2_dx);
            q3_ax <= unsigned(v_ab(8 downto 0));
            q3_sx <= q2_dx(10) xor (q2_ip and not s_same);
            -- 4: octant: which of |dx|, |dy| is the larger
            if l_ay > q3_ax then q4_swap <= '1'; else q4_swap <= '0'; end if;
            q4_ax <= q3_ax; q4_sx <= q3_sx;
            -- 5: max -> reciprocal ROM address; min
            if q4_swap = '1' then r_rc_a <= l_ay; q5_mn <= q4_ax;
            else r_rc_a <= q4_ax; q5_mn <= l_ay; end if;
            q5_swap <= q4_swap; q5_sx <= q4_sx;
            -- 6: ROM; 7: fabric copies {mantissa 11 bits, exponent}
            q6_mn <= q5_mn; q7_mn <= q6_mn;
            q7_rm <= r_rc_q(15 downto 5); q7_l <= r_rc_q(4 downto 1);
            -- 8 .. 10: ratio = min * mantissa as gated sums
            q8_r0 <= f_gu(q7_rm, q7_mn(0), 13) + shift_left(f_gu(q7_rm, q7_mn(1), 13), 1);
            q8_r1 <= f_gu(q7_rm, q7_mn(2), 13) + shift_left(f_gu(q7_rm, q7_mn(3), 13), 1);
            q8_r2 <= f_gu(q7_rm, q7_mn(4), 13) + shift_left(f_gu(q7_rm, q7_mn(5), 13), 1);
            q8_r3 <= f_gu(q7_rm, q7_mn(6), 13) + shift_left(f_gu(q7_rm, q7_mn(7), 13), 1);
            q8_r4 <= f_gu(q7_rm, q7_mn(8), 11);
            q8_l <= q7_l; q9_l <= q8_l; q10_l <= q9_l;
            q9_ra <= resize(q8_r0, 15) + shift_left(resize(q8_r1, 15), 2);
            q9_rb <= resize(q8_r2, 15) + shift_left(resize(q8_r3, 15), 2);
            q9_rc <= q8_r4;
            q10_pr <= resize(q9_ra, 20) + shift_left(resize(q9_rb, 20), 4) + shift_left(resize(q9_rc, 20), 8);
            -- 11: >> exponent, clamp: the atan / sec ROM address (10 bits: min / max)
            case to_integer(q10_l) is
                when 0      => v_r11 := q10_pr(10 downto 0);
                when 1      => v_r11 := q10_pr(11 downto 1);
                when 2      => v_r11 := q10_pr(12 downto 2);
                when 3      => v_r11 := q10_pr(13 downto 3);
                when 4      => v_r11 := q10_pr(14 downto 4);
                when 5      => v_r11 := q10_pr(15 downto 5);
                when 6      => v_r11 := q10_pr(16 downto 6);
                when 7      => v_r11 := q10_pr(17 downto 7);
                when others => v_r11 := q10_pr(18 downto 8);
            end case;
            if v_r11(10) = '1' then r_at_a <= (others => '1'); else r_at_a <= v_r11(9 downto 0); end if;
            -- 12: ROM; 13: fabric copies; {sx, swap, max} come back from their delay ring (tr_f)
            q12_rt <= r_at_a; q13_rt <= q12_rt;
            q13_d <= r_at_q(15 downto 8); q13_s <= r_at_q(7 downto 0);
            q13_rd <= unsigned(rt_q(11 downto 0)); q14_rd <= q13_rd; q15_rd <= q14_rd; q16_rd <= q15_rd;   -- this cell's disc radius
            -- 14: angle inside the octant (1024 per octant); radius = max * sec: gated sums
            q14_t <= resize(q13_rt, 11) + resize(q13_d, 11);
            q14_mx <= unsigned(tr_f(8 downto 0)); q14_swap <= tr_f(9); q14_sx <= tr_f(10);
            q14_m0 <= f_gu(unsigned(tr_f(8 downto 0)), q13_s(0), 11) + shift_left(f_gu(unsigned(tr_f(8 downto 0)), q13_s(1), 11), 1);
            q14_m1 <= f_gu(unsigned(tr_f(8 downto 0)), q13_s(2), 11) + shift_left(f_gu(unsigned(tr_f(8 downto 0)), q13_s(3), 11), 1);
            q14_m2 <= f_gu(unsigned(tr_f(8 downto 0)), q13_s(4), 11) + shift_left(f_gu(unsigned(tr_f(8 downto 0)), q13_s(5), 11), 1);
            q14_m3 <= f_gu(unsigned(tr_f(8 downto 0)), q13_s(6), 11) + shift_left(f_gu(unsigned(tr_f(8 downto 0)), q13_s(7), 11), 1);
            -- 15: unfold the octant
            if q14_swap = '1' then q15_t2 <= to_unsigned(2048, 12) - resize(q14_t, 12); else q15_t2 <= resize(q14_t, 12); end if;
            q15_ma <= resize(q14_m0, 13) + shift_left(resize(q14_m1, 13), 2);
            q15_mb <= resize(q14_m2, 13) + shift_left(resize(q14_m3, 13), 2);
            q15_mx <= q14_mx; q15_sx <= q14_sx;
            -- 16: unfold the half plane; the radius (Q2 half px)
            if q15_sx = '1' then q16_t3 <= to_unsigned(4096, 13) - resize(q15_t2, 13); else q16_t3 <= resize(q15_t2, 13); end if;
            v_m := resize(q15_ma, 17) + shift_left(resize(q15_mb, 17), 4);
            q16_r2 <= (q15_mx & "00") + resize(v_m(16 downto 7), 12);
            -- 17: the angle (13 bits per turn); Twist * radius as gated sums; radius tests
            if l_sy = '1' then q17_phi <= 0 - q16_t3; else q17_phi <= q16_t3; end if;
            v_rh := q16_r2(11 downto 2);
            q17_w0 <= f_gu(v_rh, c_twm(0), 12) + shift_left(f_gu(v_rh, c_twm(1), 12), 1);
            q17_w1 <= f_gu(v_rh, c_twm(2), 12) + shift_left(f_gu(v_rh, c_twm(3), 12), 1);
            q17_w2 <= f_gu(v_rh, c_twm(4), 12) + shift_left(f_gu(v_rh, c_twm(5), 12), 1);
            q17_w3 <= f_gu(v_rh, c_twm(6), 12) + shift_left(f_gu(v_rh, c_twm(7), 12), 1);
            q17_w4 <= f_gu(v_rh, c_twm(8), 12) + shift_left(f_gu(v_rh, c_twm(9), 12), 1);
            q17_da <= signed(resize(q16_r2, 13)) - signed(resize(q16_rd, 13));                  -- past the disc edge
            q17_db <= signed(resize(c_r(11 downto 3), 13)) - signed(resize(q16_r2, 13));     -- inside the hub
            if q16_r2 > c_r(11 downto 2) then q17_sel <= '1'; else q17_sel <= '0'; end if;
            if v_rh > c_rc then q17_re <= c_rc; else q17_re <= v_rh; end if;
            -- 18: angle - state; edge coverage (rim: ground beyond the disc; hub: accent); AA gain of the radius
            q18_pa <= q17_phi - unsigned(ar_f(12 downto 0));
            q18_wa <= resize(q17_w0, 14) + shift_left(resize(q17_w1, 14), 2);
            q18_wb <= resize(q17_w2, 14) + shift_left(resize(q17_w3, 14), 2);
            q18_wc <= q17_w4;
            if q17_sel = '1' then v_d := q17_da; else v_d := q17_db; end if;
            if s_sd = '1' then v_d := shift_right(v_d, 1); end if;
            v_c := v_d + 4;
            if v_c < 0 then v_cx := (others => '0'); elsif v_c > 8 then v_cx := to_unsigned(8, 4); else v_cx := unsigned(v_c(3 downto 0)); end if;
            if q17_sel = '1' and s_sq = '1' then v_cx := (others => '0'); end if;
            v_e := (others => '0'); v_m1 := '0'; v_m0 := '0';
            for i in 1 to 9 loop
                if q17_re(i) = '1' then
                    v_e := to_unsigned(i, 4);
                    v_m1 := q17_re(i - 1);
                    if i >= 2 then v_m0 := q17_re(i - 2); else v_m0 := '0'; end if;
                end if;
            end loop;
            er_d <= "00000" & q17_sel & std_logic_vector(v_cx) & v_m1 & v_m0 & std_logic_vector(c_shb - v_e);
            -- 19: Twist * radius, scaled by the pitch's octave
            v_tw := resize(q18_wa, 20) + shift_left(resize(q18_wb, 20), 4) + shift_left(resize(q18_wc, 20), 8);
            case c_sh is
                when "00"   => q19_tw <= v_tw(13 downto 1);
                when "01"   => q19_tw <= v_tw(14 downto 2);
                when "10"   => q19_tw <= v_tw(15 downto 3);
                when others => q19_tw <= v_tw(16 downto 4);
            end case;
            q19_pa <= q18_pa;
            -- 20: the blade angle
            q20_a <= q19_pa - q19_tw;
            -- 21, 22: * blade pairs, + a quarter blade pair (so the bits above one turn number the blade pairs
            -- from edge centre to edge centre)
            q21_n0 <= f_gu(q20_a, c_n(0), 15) + shift_left(f_gu(q20_a, c_n(1), 15), 1);
            q21_n1 <= f_gu(q20_a, c_n(2), 15) + shift_left(f_gu(q20_a, c_n(3), 15), 1) + 512;
            q22_w <= resize(q21_n0, 17) + shift_left(resize(q21_n1, 17), 2);
            -- 23: distance to the nearest blade edge; which colour; the marker blade (pair 0, drawn in the accent)
            if q22_w(11) = '0' then q23_d <= not q22_w(10 downto 0); else q23_d <= q22_w(10 downto 0); end if;
            q23_ina <= q22_w(12) xor q22_w(11);
            if q22_w(16 downto 13) = 0 or q22_w(16 downto 13) = c_n then q23_mk <= '1'; else q23_mk <= '0'; end if;
            q24_mk <= q23_mk; q25_mk <= q24_mk; q26_mk <= q25_mk;
            -- 24: * the blade count's AA gain (0.75, 1, 1.25)
            v_dq := resize(q23_d, 12);
            case c_mn is
                when "00"   => q24_v1 <= v_dq - shift_right(v_dq, 2);
                when "01"   => q24_v1 <= v_dq;
                when others => q24_v1 <= v_dq + shift_right(v_dq, 2);
            end case;
            q24_ina <= q23_ina;
            -- 25: * the radius's mantissa (1.mm, from the ring: er_f)
            q25_v2 <= resize(q24_v1, 13) + f_gu(shift_right(q24_v1, 1), er_f(5), 13) + f_gu(shift_right(q24_v1, 2), er_f(4), 13);
            q25_shq <= unsigned(er_f(3 downto 0));
            q25_sx <= er_f(10 downto 6);
            q25_ina <= q24_ina;
            -- 26: >> the radius's exponent: distance in eighths of a pixel, capped at 4
            v_sh := shift_right(q25_v2, to_integer(q25_shq));
            if v_sh > 4 then q26_off <= to_unsigned(4, 3); else q26_off <= v_sh(2 downto 0); end if;
            q26_ina <= q25_ina; q26_sx <= q25_sx;
            -- 27: blade coverage (eighths of colour A) -> the blend table's address
            if q26_ina = '1' then v_ca := to_unsigned(4, 4) + q26_off; else v_ca := to_unsigned(4, 4) - q26_off; end if;
            if q26_mk = '1' and (q26_sx(4) = '0' or q26_sx(3 downto 0) = "0000") then
                -- marker blade: B blended toward the accent by the blade coverage (or the hub's, if larger)
                if unsigned(q26_sx(3 downto 0)) > v_ca then tt_ra <= '0' & unsigned(q26_sx(3 downto 0)) & "0000";
                else tt_ra <= '0' & v_ca & "0000"; end if;
            else
                tt_ra <= unsigned(q26_sx) & v_ca;
            end if;
            -- 28: RAM; 29: fabric copy; the ground colour outside the generated avid
            if s_gav_sr(26) = '0' then
                q29_y <= c_gy; q29_c <= c_gc;
            else
                q29_y <= unsigned(tt_q(25 downto 16));
                q29_c(1) <= unsigned(tt_q(15 downto 8)); q29_c(2) <= unsigned(tt_q(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(26);
        end if;
    end process p_look;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 37 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 30: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x ground once 8 blank samples are in, so a glitched sample can't offset the colours for good
            if s_bah = "00000000" then
                s_bu <= c_gc(1) & "000"; s_bv <= c_gc(2) & "000";
            else
                s_bu <= s_bu + q29_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q29_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q29_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- 31: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 32: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 33: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y33 <= s_y32; s_u33 <= s_u32; s_v33 <= s_v32;
            else
                s_y33 <= to_unsigned(64, 10); s_u33 <= to_unsigned(512, 10); s_v33 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y33);
    data_out.u       <= std_logic_vector(s_u33);
    data_out.v       <= std_logic_vector(s_v33);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture pinwheel;
