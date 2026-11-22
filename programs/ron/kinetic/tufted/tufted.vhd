-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tufted v0.1.0 -- Kinetic wave 2.
--
-- A diamond-tufted surface (Chesterfield deep-button tufting, capitonne walls, quilted leather): a lattice of
-- 45-degree diamond pillows meeting at buttons, every pillow a soft dome lit from the upper left, dark pleats
-- along the seams.  A travelling wave inflates and deflates the pillows in turn; past zero a pillow becomes a
-- dimple and its shading inverts.
--
--   lattice  A = (X + Y) / P + 1/2, B = (X - Y) / P + 1/2 (two diagonal DDAs, Q20, virtual-1080 coordinates);
--            cell (i, j) = integer parts, p, q = 2 frac - 1 in [-1, 1) (10 bits, 512 = 1)
--   element  a pillow; state d = Puff + Depth * wave(Spread * n - phase) + camera   (signed, Q6, 64 = 1)
--            n = i + j (Linear) or |i| + |j| (Radial).  The wave part is a per-field 128-entry table by n.
--   relief   s = Tp(p) + Tq(q): per-field tables of the light term (light from the upper left) on a profile
--            F = sat(k p) (k = 1: paraboloid dome .. k large: four flat faces with hard, anti-aliased creases)
--            plus a zero-mean height term; tone = 32 + 16 d s (ONE per-pixel multiply), 2x2 ordered dither
--   pleats   weight toward the ground colour from the distance to the seam (and to the button rim), width
--            growing with |d| (a 2048-entry ROM), with a solid core of 1.5 px at every scale
--   buttons  anti-aliased octagon-metric discs in palette B at the corners, shaded, with a highlight dot
--   colour   a 1024-word table refilled every field: {tone 6, pleat 3} -> black .. A .. highlight blended toward
--            the ground; {button tone 5, coverage 3} -> dark B .. B .. light B over the ground
--
--   K1  Scale    diamond size 96 .. 640 virtual px (exponential, glided)
--   K2  Shape    pillow profile: soft dome .. faceted pyramid
--   K3  Depth    wave depth 0 .. 1 (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour   the 16 shared palettes (Spread fixed: 0.09 cycle per step Linear, 0.13 Radial)
--   K6  Camera   camera (and Osc) strength: the held luma per pillow, less the field mean, adds to d
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Buttons  Off / On
--   S10 Finish   Matte / Gloss (blown specular spot, deeper pleats)
--   S11 Source   Video / Osc
--   P12 PUFF     the mean inflation 0.1 .. 1.5 (glided): nearly flat quilt .. over-stuffed
--
-- Sections (the Kinetic chassis, lifted from louver):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source: Osc lattice, luma smear, field mean, per-pillow map (two banks, previous field)
--   [D] wave engine: D table fill (per field), diagonal DDAs, index
--   [E] the look: relief table fill, colour table fill, pixel path
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 24) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tufted_pkg.all;
use work.all;

architecture tufted of program_top is
    constant C_LAT      : integer := 24;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob
    constant C_RB       : integer := 108;     -- button radius, p10 units
    constant C_SLIN     : integer := 1475;    -- spread per index step, 14 bits per cycle
    constant C_SRAD     : integer := 2130;

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
    -- one pixel in p10 units x P = round(1448 x the larger step)
    function f_kpx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(6436, 13);
            when 4          => return to_unsigned(3258, 13);
            when 8          => return to_unsigned(5430, 13);
            when 12         => return to_unsigned(2896, 13);
            when 5 | 6 | 14 => return to_unsigned(2172, 13);
            when 1 | 2 | 10 => return to_unsigned(2896, 13);
            when others     => return to_unsigned(1448, 13);
        end case;
    end function;
    -- the sharpest crease the shape may ask for (log2, Q8, relative to the pitch exponent)
    function f_cv(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(600, 11);
            when 4          => return to_unsigned(852, 11);
            when 8          => return to_unsigned(663, 11);
            when 12         => return to_unsigned(896, 11);
            when 5 | 6 | 14 => return to_unsigned(1002, 11);
            when 1 | 2 | 10 => return to_unsigned(896, 11);
            when others     => return to_unsigned(1152, 11);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_gs(x : signed; b : std_logic; w : natural) return signed is
        variable r : signed(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    function f_umin(a, b : unsigned) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;
    function f_umax(a, b : unsigned) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(7 downto 0) := (others => '0');
    signal r_sn_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_pt_a : unsigned(10 downto 0) := (others => '0');
    signal r_pt_q : unsigned(3 downto 0) := (others => '0');

    -- [C] per-pillow source map: 2 banks x 1024 cells (previous field's bank is read)
    type t_mp is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal mp : t_mp := (others => x"00");
    attribute no_rw_check of mp : signal is true;
    signal mr_a, mw_a : unsigned(10 downto 0) := (others => '0');
    signal mr_q, mw_d : std_logic_vector(7 downto 0) := x"00";
    signal mw_we : std_logic := '0';
    -- [D] wave + puff by index (per field)
    type t_d is array (0 to 127) of std_logic_vector(7 downto 0);
    signal dt : t_d := (others => x"00");
    attribute no_rw_check of dt : signal is true;
    signal dt_ra, dt_wa : unsigned(6 downto 0) := (others => '0');
    signal dt_q, dt_wd : std_logic_vector(7 downto 0) := x"00";
    signal dt_we : std_logic := '0';
    -- [E] relief tables (per field)
    type t_tt is array (0 to 255) of std_logic_vector(15 downto 0);
    signal tp, tq : t_tt := (others => (others => '0'));
    attribute no_rw_check of tp : signal is true;
    attribute no_rw_check of tq : signal is true;
    signal tp_ra, tq_ra, tp_wa, tq_wa : unsigned(7 downto 0) := (others => '0');
    signal tp_q, tq_q, tp_wd, tq_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal tp_we : std_logic := '0';
    -- [E] colour table (per field): {Y 10, U 8, V 8}
    type t_ct is array (0 to 1023) of std_logic_vector(25 downto 0);
    signal ct : t_ct := (others => "0001000000" & x"8080");
    attribute no_rw_check of ct : signal is true;
    signal ct_ra, ct_wa : unsigned(9 downto 0) := (others => '0');
    signal ct_q, ct_wd : std_logic_vector(25 downto 0) := (others => '0');
    signal ct_we : std_logic := '0';
    -- delay rings
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
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
    signal s_kpx  : unsigned(12 downto 0) := to_unsigned(1448, 13);
    signal s_cv   : unsigned(10 downto 0) := to_unsigned(1152, 11);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(500 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(450 * 8, 13);
    signal s_gk : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_src, s_gloss : std_logic := '0';
    signal s_btn : std_logic := '1';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (24 iterations), sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(23 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_nz  : std_logic := '0';
    signal z_m   : unsigned(16 downto 0) := to_unsigned(65536, 17);
    signal z_kx  : unsigned(10 downto 0) := (others => '0');
    signal z_ksh : unsigned(2 downto 0) := (others => '0');
    signal z_er  : unsigned(10 downto 0) := (others => '0');
    signal z_rsh : unsigned(2 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    -- mean divider
    signal z_dv  : unsigned(29 downto 0) := (others => '0');
    signal z_dc  : unsigned(28 downto 0) := (others => '0');
    signal z_dq  : unsigned(7 downto 0) := (others => '0');
    signal z_dk  : unsigned(3 downto 0) := (others => '0');
    signal z_go4, z_go70, z_go260, z_go400 : std_logic := '0';   -- registered fill start pulses (zc 4 / 70 / 260 / 400)

    -- per-field constants
    signal c_ip   : unsigned(13 downto 0) := to_unsigned(4326, 14);    -- 1 / pitch, Q20
    signal c_kx   : unsigned(15 downto 0) := to_unsigned(6489, 16);    -- A, B step per pixel
    signal c_ky   : unsigned(15 downto 0) := to_unsigned(6489, 16);    -- A step per line (B: minus)
    signal c_a0, c_b0 : signed(27 downto 0) := (others => '0');
    signal c_pxe  : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal c_core : unsigned(7 downto 0) := to_unsigned(12, 8);
    signal c_shc  : unsigned(2 downto 0) := (others => '0');          -- 7 = shift left 1, else shift right
    signal c_df   : unsigned(10 downto 0) := to_unsigned(18, 11);
    signal c_amp  : unsigned(7 downto 0) := (others => '0');
    signal c_puff : unsigned(9 downto 0) := to_unsigned(184, 10);
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    signal c_cam  : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal c_lbar : unsigned(7 downto 0) := (others => '0');
    signal c_hm1  : unsigned(10 downto 0) := (others => '1');          -- active lines - 1 (previous field)
    signal c_gy   : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    signal c_gc   : t_u8 := (others => to_unsigned(128, 8));

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
    signal s_sav, s_savd : std_logic := '0';
    signal s_y1  : unsigned(13 downto 0) := (others => '0');
    signal s_lsum : unsigned(29 downto 0) := (others => '0');
    signal s_lcnt : unsigned(21 downto 0) := (others => '0');
    signal s_bank : std_logic := '0';

    -- fills
    signal cf_s   : unsigned(2 downto 0) := (others => '0');   -- 0 idle 1 key 2 curve 3 mload 4 mul 5 cstore 6 wsetup 7 write
    signal cf_btn : std_logic := '0';
    signal cf_i   : unsigned(5 downto 0) := (others => '0');
    signal cf_n   : unsigned(2 downto 0) := (others => '0');
    signal cf_ch  : unsigned(1 downto 0) := (others => '0');
    signal cf_k   : unsigned(2 downto 0) := (others => '0');
    signal cf_seg : unsigned(1 downto 0) := (others => '0');
    signal cf_wt, cf_wq : unsigned(7 downto 0) := (others => '0');
    type t_s11 is array (0 to 2) of signed(10 downto 0);
    type t_s14 is array (0 to 2) of signed(13 downto 0);
    type t_u10 is array (0 to 2) of unsigned(9 downto 0);
    signal cf_c   : t_u10 := (others => (others => '0'));
    signal cf_md, cf_acc : signed(19 downto 0) := (others => '0');
    signal cf_a2  : t_s14 := (others => (others => '0'));
    signal cf_st  : t_s11 := (others => (others => '0'));
    -- keys: 0 G, 1 A, 2 H, 3 B, 4 B dark, 5 B light (Y 10, U 8, V 8 in a 10-bit field)
    type t_key is array (0 to 5) of t_u10;
    signal kk : t_key := (others => (to_unsigned(64, 10), to_unsigned(128, 10), to_unsigned(128, 10)));
    signal tf_run, tf_pass : std_logic := '0';
    signal tf_t   : unsigned(6 downto 0) := (others => '0');
    signal tf_cs  : unsigned(2 downto 0) := (others => '0');
    signal tf_f, tf_fv, tf_g, tf_gm : unsigned(10 downto 0) := (others => '0');
    signal tf_gs, tf_sg : unsigned(17 downto 0) := (others => '0');
    signal tf_x   : signed(12 downto 0) := (others => '0');
    signal tf_ao  : signed(13 downto 0) := (others => '0');
    signal tf_lp, tf_lq : signed(11 downto 0) := (others => '0');
    signal df_run, df_sg : std_logic := '0';
    signal df_n   : unsigned(6 downto 0) := (others => '0');
    signal df_cs  : unsigned(3 downto 0) := (others => '0');
    signal df_th  : unsigned(13 downto 0) := (others => '0');
    signal df_mc, df_acc : unsigned(17 downto 0) := (others => '0');
    signal df_mp  : unsigned(7 downto 0) := (others => '0');
    signal df_d8  : signed(11 downto 0) := (others => '0');

    -- [D] DDAs (stage 1)
    signal l_a, l_b, a_a, a_b : signed(27 downto 0) := (others => '0');
    signal l_ln : unsigned(10 downto 0) := (others => '0');
    signal l_yp, a_xp, a_yp, a_first, a_last : std_logic := '0';
    signal a_x  : unsigned(11 downto 0) := (others => '0');
    -- [E] pixel path
    signal q2_i, q2_j : signed(7 downto 0) := (others => '0');
    signal q2_p, q2_q : signed(9 downto 0) := (others => '0');
    signal q2_bd : std_logic := '0';
    signal s_xp, s_yp : std_logic_vector(2 to 14) := (others => '0');
    signal q3_adp, q3_adq : unsigned(9 downto 0) := (others => '0');
    signal q3_dps, q3_dqs : signed(9 downto 0) := (others => '0');
    signal q4_mx, q4_mn, q5_mx, q5_mn : unsigned(9 downto 0) := (others => '0');
    signal q4_lin : signed(10 downto 0) := (others => '0');
    signal q4_ddx, q4_ddy, q5_dmx, q5_dmn, q6_dmx : unsigned(10 downto 0) := (others => '0');
    signal q5_t1, q6_rc : unsigned(10 downto 0) := (others => '0');
    signal q5_bt0, q6_bt0, q7_bt0, q8_bt0 : signed(7 downto 0) := (others => '0');
    signal q6_dt, q7_rd : unsigned(11 downto 0) := (others => '0');
    signal q5_d, q6_d, q7_d, q8_d : signed(7 downto 0) := (others => '0');
    signal q5_l : unsigned(7 downto 0) := (others => '0');
    signal q5_tp, q5_tq : signed(11 downto 0) := (others => '0');
    signal q6_s, q7_s, q8_s, q9_s : signed(9 downto 0) := (others => '0');
    signal q6_src : signed(8 downto 0) := (others => '0');
    signal q6_ee, q7_ee, q8_ee : unsigned(9 downto 0) := (others => '0');
    signal q7_pa  : signed(12 downto 0) := (others => '0');
    signal q7_pb  : signed(11 downto 0) := (others => '0');
    signal q7_ec  : unsigned(9 downto 0) := (others => '0');
    signal q7_x   : signed(10 downto 0) := (others => '0');
    signal q8_term : signed(11 downto 0) := (others => '0');
    signal q8_ec  : unsigned(9 downto 0) := (others => '0');
    signal q8_cv  : signed(10 downto 0) := (others => '0');
    signal q8_btd : signed(10 downto 0) := (others => '0');
    signal q9_d   : signed(7 downto 0) := (others => '0');
    signal q9_e   : unsigned(9 downto 0) := (others => '0');
    signal q9_cov, q10_cov, q11_cov, q12_cov, q13_cov : unsigned(3 downto 0) := (others => '0');
    signal q9_bt, q10_bt, q11_bt, q12_bt, q13_bt : unsigned(4 downto 0) := (others => '0');
    signal q10_p01, q10_p23, q10_p45, q10_p67 : signed(11 downto 0) := (others => '0');
    signal q10_e1 : unsigned(9 downto 0) := (others => '0');
    signal q10_ad : unsigned(8 downto 0) := (others => '0');
    signal q11_q0, q11_q1 : signed(14 downto 0) := (others => '0');
    signal q12_pr : signed(18 downto 0) := (others => '0');
    signal q13_tone : unsigned(5 downto 0) := (others => '0');
    signal q13_w4 : unsigned(3 downto 0) := (others => '0');
    signal q16_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q16_c  : t_u8 := (others => to_unsigned(128, 8));
    signal q17_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q17_c  : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    type t_y6 is array (0 to 5) of unsigned(9 downto 0);
    signal s_ysr  : t_y6 := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, tables, map, rings (one write statement each; none ever
    -- reads the address it is writing while the read is used)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_TF_EXP2(to_integer(r_ex_a));
            r_sn_q <= C_TF_SIN(to_integer(r_sn_a));
            r_pl_q <= C_TF_PAL(to_integer(r_pl_a));
            r_pt_q <= C_TF_PLT(to_integer(r_pt_a));
        end if;
    end process p_rom;

    p_mp : process(clk)
    begin
        if rising_edge(clk) then
            mr_q <= mp(to_integer(mr_a));
            if mw_we = '1' then mp(to_integer(mw_a)) <= mw_d; end if;
        end if;
    end process p_mp;

    p_dt : process(clk)
    begin
        if rising_edge(clk) then
            dt_q <= dt(to_integer(dt_ra));
            if dt_we = '1' then dt(to_integer(dt_wa)) <= dt_wd; end if;
        end if;
    end process p_dt;

    p_tp : process(clk)
    begin
        if rising_edge(clk) then
            tp_q <= tp(to_integer(tp_ra));
            if tp_we = '1' then tp(to_integer(tp_wa)) <= tp_wd; end if;
        end if;
    end process p_tp;

    p_tq : process(clk)
    begin
        if rising_edge(clk) then
            tq_q <= tq(to_integer(tq_ra));
            if tp_we = '1' then tq(to_integer(tq_wa)) <= tq_wd; end if;
        end if;
    end process p_tq;

    p_ct : process(clk)
    begin
        if rising_edge(clk) then
            ct_q <= ct(to_integer(ct_ra));
            if ct_we = '1' then ct(to_integer(ct_wa)) <= ct_wd; end if;
        end if;
    end process p_ct;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            cr(to_integer(rc + 6)) <= std_logic_vector(q17_c(1)) & std_logic_vector(q17_c(2));
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
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_e2 : unsigned(10 downto 0);
        variable v_ek : unsigned(11 downto 0);
        variable v_dc : unsigned(29 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_kpx <= f_kpx(s_tid);
            s_cv  <= f_cv(s_tid);

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
            v_s := signed(resize(s_k4, 11)) - to_signed(512, 11);
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
                s_snap  <= registers_in(6)(0);
                s_rad   <= registers_in(6)(1);
                s_btn   <= registers_in(6)(2);
                s_gloss <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
                c_hm1 <= l_ln - 1;
                s_bank <= not s_bank;
                -- the field mean of the source: sum / count (restoring division, zc 2 .. 9)
                z_dv <= s_lsum; z_dc <= s_lcnt & "0000000"; z_dq <= (others => '0'); z_dk <= to_unsigned(8, 4);
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            if z_dk /= 0 and s_zgo = '0' then
                v_dc := resize(z_dc, 30);
                if z_dv >= v_dc then
                    z_dv <= z_dv - v_dc; z_dq <= z_dq(6 downto 0) & '1';
                else
                    z_dq <= z_dq(6 downto 0) & '0';
                end if;
                z_dc <= '0' & z_dc(28 downto 1);
                z_dk <= z_dk - 1;
            end if;

            -- shared glide for Scale, Puff and Camera (1/8 of the gap per field, +-2 / +-4 deadband)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(1) = '1' then s_gm <= s_gk; s_gr <= unsigned(registers_in(5));
                        elsif z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif (s_gd > 16 or s_gd < -16) and (z_gc(1) = '0' or s_gd > 32 or s_gd < -32) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(1) = '1' then s_gk <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        elsif z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) /= 9
               and s_zc(8 downto 5) < 11 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                    z_fb <= to_unsigned(5606, 24);  -- pitch exponent
                    when 1  =>                                                                              -- depth^2
                        z_fa(9 downto 0) <= unsigned(registers_in(2)); z_fb(9 downto 0) <= unsigned(registers_in(2));
                    when 2  => z_fa(12 downto 0) <= s_gp;                    z_fb <= to_unsigned(2867, 24);  -- puff
                    when 3  => z_fa(16 downto 0) <= z_m;                     z_fb <= to_unsigned(10923, 24); -- 1 / pitch
                    when 4  => z_fa(13 downto 0) <= c_ip;                    z_fb <= resize(s_ys, 24);       -- step per line
                    when 5  =>                                                                              -- origin A
                        z_fa(13 downto 0) <= c_ip;
                        if s_xs = 4 then z_fb <= to_unsigned(1260, 24); else z_fb <= to_unsigned(1500, 24); end if;
                    when 6  =>                                                                              -- origin B
                        z_fa(13 downto 0) <= c_ip;
                        if s_xs = 4 then z_fb <= to_unsigned(180, 24); else z_fb <= to_unsigned(420, 24); end if;
                    when 7  => z_fa(13 downto 0) <= c_ip;                    z_fb <= resize(s_kpx, 24);      -- pixel size
                    when 8  => z_fa(8 downto 0)  <= s_vm;                    z_fb <= to_unsigned(900, 24);   -- rate exponent
                    when others => z_fa(15 downto 0) <= z_mt;                z_fb <= to_unsigned(1193046, 24); -- rate (10)
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(23 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- wave phase (last field's step): one adder, add or subtract
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    when 21 => c_cam <= s_gk(12 downto 6);
                    when 22 => c_lbar <= z_dq;
                    -- pitch: e = k1 * 2.74 octaves (Q8), 1 / P = 2^(-e/256) / 96
                    when 60 => z_e <= f_acc(26 downto 16);
                    when 61 =>
                        r_ex_a <= 0 - z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                        if z_e(7 downto 0) = 0 then z_nz <= '0'; else z_nz <= '1'; end if;
                    when 63 =>
                        if z_nz = '1' then z_m <= '0' & r_ex_q; else z_m <= to_unsigned(65536, 17); end if;
                    -- shape gain: 2^(min(1.5 k2, e + cv) / 256)
                    when 64 =>
                        v_e2 := unsigned(registers_in(1)) + resize(unsigned(registers_in(1)(9 downto 1)), 11);
                        v_ek := resize(z_e, 12) + resize(s_cv, 12);
                        if resize(v_e2, 12) < v_ek then z_kx <= v_e2; else z_kx <= v_ek(10 downto 0); end if;
                    when 65 => r_ex_a <= z_kx(7 downto 0); z_ksh <= z_kx(10 downto 8);
                    when 67 =>
                        case to_integer(z_ksh) is
                            when 0      => c_df <= resize(r_ex_q(15 downto 12), 11);
                            when 1      => c_df <= resize(r_ex_q(15 downto 11), 11);
                            when 2      => c_df <= resize(r_ex_q(15 downto 10), 11);
                            when 3      => c_df <= resize(r_ex_q(15 downto 9), 11);
                            when 4      => c_df <= resize(r_ex_q(15 downto 8), 11);
                            when 5      => c_df <= resize(r_ex_q(15 downto 7), 11);
                            when others => c_df <= resize(r_ex_q(15 downto 6), 11);
                        end case;
                    when 92  => c_amp <= f_acc(19 downto 12);
                    when 124 => c_puff <= to_unsigned(26, 10) + resize(f_acc(24 downto 16), 10);
                    when 156 =>
                        case to_integer(z_sh) is
                            when 0      => c_ip <= f_acc(29 downto 16);
                            when 1      => c_ip <= f_acc(30 downto 17);
                            when others => c_ip <= f_acc(31 downto 18);
                        end case;
                    when 157 =>
                        case to_integer(s_xs) is
                            when 4      => c_kx <= resize(c_ip & '0', 16);
                            when 3      => c_kx <= resize(c_ip, 16) + resize(c_ip(13 downto 1), 16);
                            when others => c_kx <= resize(c_ip, 16);
                        end case;
                    when 188 => c_ky <= f_acc(23 downto 8);
                    when 220 => c_a0 <= to_signed(524288, 28) - signed(f_acc(27 downto 0));
                    when 252 => c_b0 <= to_signed(524288, 28) - signed(f_acc(27 downto 0));
                    when 284 => c_pxe <= f_acc(26 downto 20);
                    when 285 =>
                        c_core <= resize(c_pxe, 8) + resize(c_pxe(6 downto 1), 8);
                        if    c_pxe <= 4  then c_shc <= "111";
                        elsif c_pxe <= 8  then c_shc <= "000";
                        elsif c_pxe <= 16 then c_shc <= "001";
                        elsif c_pxe <= 32 then c_shc <= "010";
                        elsif c_pxe <= 64 then c_shc <= "011";
                        else c_shc <= "100"; end if;
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 316 => z_er <= f_acc(18 downto 8);
                    when 317 => r_ex_a <= z_er(7 downto 0); z_rsh <= z_er(10 downto 8);
                    when 319 => z_mt <= r_ex_q;
                    when 380 =>
                        if s_vm = 0 then
                            c_dph <= (others => '0');
                        else
                            c_dph <= shift_left(resize(f_acc(36 downto 21), 22), to_integer(z_rsh));
                        end if;
                    when others => null;
                end case;
            end if;

            if s_zone = '1' and s_zc = 3 then z_go4 <= '1'; else z_go4 <= '0'; end if;
            if s_zone = '1' and s_zc = 69 then z_go70 <= '1'; else z_go70 <= '0'; end if;
            if s_zone = '1' and s_zc = 259 then z_go260 <= '1'; else z_go260 <= '0'; end if;
            if s_zone = '1' and s_zc = 399 then z_go400 <= '1'; else z_go400 <= '0'; end if;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [E] Colour table fill (zone, zc 4 ..): the keys, then 64 pillow tones and
    -- 32 button tones; each: curve word -> lo / hi keys -> C = lo + (hi - lo) wt / 256
    -- (three serial 8-step products) -> 8 entries blended toward the ground (pillows)
    -- or from the ground (buttons) in eighths, the last one exact.
    ---------------------------------------------------------------------------
    p_cfill : process(clk)
        variable v_lo, v_hi : unsigned(9 downto 0);
        variable v_o : unsigned(25 downto 0);
        variable v_c : signed(13 downto 0);
        variable v_c12 : signed(11 downto 0);
        variable v_ch : integer range 0 to 2;
    begin
        if rising_edge(clk) then
            ct_we <= '0';
            if cf_ch = 2 then v_ch := 2; elsif cf_ch = 1 then v_ch := 1; else v_ch := 0; end if;
            -- the segment's end keys for the channel being multiplied
            if cf_btn = '1' then
                if cf_seg = 0 then v_lo := kk(4)(v_ch); v_hi := kk(3)(v_ch); else v_lo := kk(3)(v_ch); v_hi := kk(5)(v_ch); end if;
            else
                case to_integer(cf_seg) is
                    when 0 =>
                        if v_ch = 0 then v_lo := to_unsigned(64, 10); else v_lo := to_unsigned(128, 10); end if;
                        v_hi := kk(1)(v_ch);
                    when 1 => v_lo := kk(1)(v_ch); v_hi := kk(2)(v_ch);
                    when others =>
                        v_lo := kk(2)(v_ch);
                        if v_ch = 0 then v_hi := to_unsigned(940, 10); else v_hi := to_unsigned(128, 10); end if;
                end case;
            end if;
            if z_go4 = '1' then
                cf_s <= "001"; cf_btn <= '0'; cf_i <= (others => '0'); cf_n <= (others => '0'); cf_k <= (others => '0');
                r_pl_a <= "0" & s_pal & "000";
            else
                case to_integer(cf_s) is
                    when 1 =>                                   -- keys: address at n 0, data at n 2
                        cf_n <= cf_n + 1;
                        if cf_n = 2 then
                            kk(to_integer(cf_k))(0) <= unsigned(r_pl_q(31 downto 22));
                            kk(to_integer(cf_k))(1) <= resize(unsigned(r_pl_q(21 downto 14)), 10);
                            kk(to_integer(cf_k))(2) <= resize(unsigned(r_pl_q(13 downto 6)), 10);
                            cf_n <= (others => '0');
                            if cf_k = 5 then
                                cf_s <= "010"; r_pl_a <= to_unsigned(128, 8);
                            else
                                cf_k <= cf_k + 1; r_pl_a <= "0" & s_pal & (cf_k + 1);
                            end if;
                        end if;
                    when 2 =>                                   -- curve word: address at n 0, data at n 2
                        cf_n <= cf_n + 1;
                        if cf_n = 2 then
                            if cf_btn = '0' and s_gloss = '1' then
                                cf_seg <= unsigned(r_pl_q(9 downto 8)); cf_wt <= unsigned(r_pl_q(7 downto 0));
                            else
                                cf_seg <= unsigned(r_pl_q(19 downto 18)); cf_wt <= unsigned(r_pl_q(17 downto 10));
                            end if;
                            cf_ch <= "00"; cf_s <= "011";
                        end if;
                    when 3 =>                                   -- C(ch) = lo + (hi - lo) wt / 256: load
                        cf_md <= resize(signed(resize(v_hi, 11)) - signed(resize(v_lo, 11)), 20);
                        cf_acc <= (others => '0'); cf_wq <= cf_wt; cf_n <= (others => '0');
                        cf_s <= "100";
                    when 4 =>                                   -- 8 shift-add steps
                        if cf_wq(0) = '1' then cf_acc <= cf_acc + cf_md; end if;
                        cf_md <= shift_left(cf_md, 1);
                        cf_wq <= '0' & cf_wq(7 downto 1);
                        cf_n <= cf_n + 1;
                        if cf_n = 7 then cf_s <= "101"; end if;
                    when 5 =>
                        v_c12 := signed(resize(v_lo, 12)) + resize(shift_right(cf_acc, 8), 12);
                        cf_c(v_ch) <= unsigned(v_c12(9 downto 0));
                        if cf_ch = 2 then cf_s <= "110"; else cf_ch <= cf_ch + 1; cf_s <= "011"; end if;
                    when 6 =>                                   -- blend toward (pillows) / from (buttons) the ground
                        for c in 0 to 2 loop
                            if cf_btn = '1' then
                                cf_a2(c) <= signed(resize(kk(0)(c) & "000", 14));
                                cf_st(c) <= signed(resize(cf_c(c), 11)) - signed(resize(kk(0)(c), 11));
                            else
                                cf_a2(c) <= signed(resize(cf_c(c) & "000", 14));
                                cf_st(c) <= signed(resize(kk(0)(c), 11)) - signed(resize(cf_c(c), 11));
                            end if;
                        end loop;
                        cf_k <= (others => '0'); cf_s <= "111";
                    when 7 =>                                   -- 8 entries in eighths, the last exact
                        for c in 0 to 2 loop
                            v_c := shift_right(cf_a2(c), 3);
                            if cf_k = 7 then
                                if cf_btn = '1' then v_c := signed(resize(cf_c(c), 14)); else v_c := signed(resize(kk(0)(c), 14)); end if;
                            end if;
                            if c = 0 then v_o(25 downto 16) := unsigned(v_c(9 downto 0));
                            elsif c = 1 then v_o(15 downto 8) := unsigned(v_c(7 downto 0));
                            else v_o(7 downto 0) := unsigned(v_c(7 downto 0)); end if;
                            cf_a2(c) <= cf_a2(c) + resize(cf_st(c), 14);
                        end loop;
                        ct_we <= '1';
                        ct_wd <= std_logic_vector(v_o);
                        if cf_btn = '1' then ct_wa <= "10" & cf_i(4 downto 0) & cf_k;
                        else ct_wa <= "0" & cf_i & cf_k; end if;
                        cf_k <= cf_k + 1;
                        if cf_k = 7 then
                            cf_n <= (others => '0'); cf_s <= "010";
                            if cf_btn = '0' and cf_i = 63 then
                                cf_btn <= '1'; cf_i <= (others => '0'); r_pl_a <= to_unsigned(192, 8);
                            elsif cf_btn = '1' and cf_i = 31 then
                                cf_s <= "000";
                            else
                                cf_i <= cf_i + 1;
                                if cf_btn = '1' then r_pl_a <= to_unsigned(192, 8) + resize(cf_i + 1, 8);
                                else r_pl_a <= to_unsigned(128, 8) + resize(cf_i + 1, 8); end if;
                            end if;
                        end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_cfill;

    ---------------------------------------------------------------------------
    -- [E] Relief table fill (zone, zc 70 ..): F = sat(k p) at p = (t + 1/2)/128,
    -- G = its running integral; pass 0 sums G for the mean, pass 1 writes
    -- Tp = -+ (F - F/32) + 1.625 (Gmean - G), Tq = +- F/4 + 1.625 (Gmean - G), Q8.
    ---------------------------------------------------------------------------
    p_tfill : process(clk)
        variable v_t : signed(13 downto 0);
        variable v_sg : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            tp_we <= '0';
            if z_go70 = '1' then
                tf_run <= '1'; tf_pass <= '0'; tf_t <= (others => '0'); tf_cs <= (others => '0');
                tf_f <= resize(c_df(10 downto 1), 11); tf_gs <= (others => '0'); tf_sg <= (others => '0');
            elsif tf_run = '1' then
                tf_cs <= tf_cs + 1;
                case to_integer(tf_cs) is
                    when 0 =>
                        if tf_f > 1024 then tf_fv <= to_unsigned(1024, 11); else tf_fv <= tf_f; end if;
                    when 1 => tf_gs <= tf_gs + tf_fv;
                    when 2 =>
                        tf_g <= resize(shift_right(tf_gs - resize(tf_fv(10 downto 1), 18), 7), 11);
                    when 3 =>
                        if resize(tf_fv, 12) + resize(c_df, 12) > 2047 then tf_f <= to_unsigned(2047, 11);
                        else tf_f <= tf_fv + c_df; end if;
                        if tf_pass = '0' then
                            v_sg := tf_sg + tf_g;
                            tf_sg <= v_sg;
                            tf_cs <= (others => '0');
                            tf_t <= tf_t + 1;
                            if tf_t = 127 then
                                tf_pass <= '1'; tf_gm <= v_sg(17 downto 7);
                                tf_f <= resize(c_df(10 downto 1), 11); tf_gs <= (others => '0');
                            end if;
                        else
                            tf_x  <= signed(resize(tf_gm, 13)) - signed(resize(tf_g, 13));
                            tf_lp <= signed(resize(tf_fv, 12)) - signed(resize(tf_fv(10 downto 5), 12));
                            tf_lq <= signed(resize(tf_fv(10 downto 2), 12));
                        end if;
                    when 4 =>
                        tf_ao <= resize(tf_x, 14) + resize(shift_right(tf_x, 1), 14) + resize(shift_right(tf_x, 3), 14);
                    when 5 =>
                        tp_we <= '1';
                        tp_wa <= '1' & tf_t; tq_wa <= '1' & tf_t;
                        v_t := tf_ao - resize(tf_lp, 14);
                        tp_wd <= std_logic_vector(resize(shift_right(v_t, 2), 16));
                        v_t := tf_ao + resize(tf_lq, 14);
                        tq_wd <= std_logic_vector(resize(shift_right(v_t, 2), 16));
                    when others =>
                        tp_we <= '1';
                        tp_wa <= '0' & (not tf_t); tq_wa <= '0' & (not tf_t);
                        v_t := tf_ao + resize(tf_lp, 14);
                        tp_wd <= std_logic_vector(resize(shift_right(v_t, 2), 16));
                        v_t := tf_ao - resize(tf_lq, 14);
                        tq_wd <= std_logic_vector(resize(shift_right(v_t, 2), 16));
                        tf_cs <= (others => '0');
                        tf_t <= tf_t + 1;
                        if tf_t = 127 then tf_run <= '0'; end if;
                end case;
            end if;
        end if;
    end process p_tfill;

    ---------------------------------------------------------------------------
    -- [D] Wave table fill (zone, zc 400 ..): for n = -64 .. 63,
    -- theta = Spread n - phase, wave = quarter-sine ROM (Snap: the first quarter
    -- of the quarter, then dwell), D = (puff + Depth * wave / 1024) / 4, clamped (Q6)
    ---------------------------------------------------------------------------
    p_dfill : process(clk)
        variable v_pos : unsigned(11 downto 0);
        variable v_s   : unsigned(13 downto 0);
        variable v_d6  : signed(11 downto 0);
        variable v_t14 : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            dt_we <= '0';
            if s_rad = '1' then v_s := to_unsigned(C_SRAD, 14); else v_s := to_unsigned(C_SLIN, 14); end if;
            if z_go400 = '1' then
                df_run <= '1'; df_n <= to_unsigned(64, 7); df_cs <= (others => '0');
                v_t14 := shift_left(v_s, 6) + c_ph(25 downto 12);
                df_th <= (not v_t14) + 1;          -- -(Spread x 64 + phase)
            elsif df_run = '1' then
                df_cs <= df_cs + 1;
                case to_integer(df_cs) is
                    when 0 =>
                        v_pos := df_th(11 downto 0);
                        if df_th(12) = '1' then v_pos := not v_pos; end if;
                        if s_snap = '0' then r_sn_a <= v_pos(11 downto 4);
                        elsif v_pos(11 downto 10) /= "00" then r_sn_a <= (others => '1');
                        else r_sn_a <= v_pos(9 downto 2); end if;
                        df_sg <= df_th(13);
                    when 2 =>
                        df_mc <= resize(r_sn_q(9 downto 0), 18); df_mp <= c_amp; df_acc <= (others => '0');
                    when 3 to 10 =>
                        if df_mp(0) = '1' then df_acc <= df_acc + df_mc; end if;
                        df_mc <= shift_left(df_mc, 1);
                        df_mp <= '0' & df_mp(7 downto 1);
                    when 11 =>
                        if df_sg = '1' then
                            df_d8 <= signed(resize(c_puff, 12)) - signed(resize(df_acc(17 downto 10), 12));
                        else
                            df_d8 <= signed(resize(c_puff, 12)) + signed(resize(df_acc(17 downto 10), 12));
                        end if;
                    when 12 =>
                        v_d6 := shift_right(df_d8, 2);
                        if v_d6 > 127 then dt_wd <= x"7F";
                        elsif v_d6 < -128 then dt_wd <= x"80";
                        else dt_wd <= std_logic_vector(v_d6(7 downto 0)); end if;
                        dt_we <= '1'; dt_wa <= df_n;
                        df_th <= df_th + v_s;
                        df_n <= df_n + 1;
                        df_cs <= (others => '0');
                        if df_n = 63 then df_run <= '0'; end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_dfill;

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
    -- [C] Source.  Osc: a two-axis triangle lattice (640 x 540 virtual px)
    -- drifting diagonally.  The luma (or the lattice) is summed for the field
    -- mean and smeared along x by a 1/16 one-pole filter; the pixel path writes
    -- the smeared value into the pillow's map cell (inner half, or the border).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
    begin
        if rising_edge(clk) then
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

            s_sav <= data_in.avid; s_savd <= s_sav;
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_o;
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                end if;
            end if;
            -- field sum and count of the source (for the mean)
            if s_zgo = '1' then
                s_lsum <= (others => '0'); s_lcnt <= (others => '0');
            elsif s_sav = '1' then
                s_lsum <= s_lsum + s_sv; s_lcnt <= s_lcnt + 1;
            end if;
            -- smear (1/16 one-pole, Q6), restarted at each line
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
            elsif s_sav = '1' then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 4), 14));
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Diagonal DDAs: A, B step +kx per pixel; per active line A += ky,
    -- B -= ky; the field starts at (A0, B0) (zone, zc 260)
    ---------------------------------------------------------------------------
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            if z_go260 = '1' then
                l_a <= c_a0; l_b <= c_b0; l_ln <= (others => '0'); l_yp <= '0';
            elsif g_start = '1' then
                l_a <= l_a + signed(resize(c_ky, 28));
                l_b <= l_b - signed(resize(c_ky, 28));
                l_ln <= l_ln + 1; l_yp <= not l_yp;
            end if;
            if g_start = '1' then
                a_a <= l_a; a_b <= l_b;
                a_x <= (others => '0'); a_xp <= '0'; a_yp <= l_yp;
                if l_ln = 0 then a_first <= '1'; else a_first <= '0'; end if;
                if l_ln = c_hm1 then a_last <= '1'; else a_last <= '0'; end if;
            elsif g_avid = '1' then
                a_a <= a_a + signed(resize(c_kx, 28));
                a_b <= a_b + signed(resize(c_kx, 28));
                a_x <= a_x + 1; a_xp <= not a_xp;
            end if;
        end if;
    end process p_acc;

    ---------------------------------------------------------------------------
    -- [E] Pixel path (stage k = the k-th register after the DDAs)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_i, v_j : signed(7 downto 0);
        variable v_n, v_c, v_r : signed(7 downto 0);
        variable v_ap, v_aq : unsigned(9 downto 0);
        variable v_a11 : signed(10 downto 0);
        variable v_lin : signed(10 downto 0);
        variable v_dx, v_dy : signed(10 downto 0);
        variable v_t1 : unsigned(10 downto 0);
        variable v_dt : unsigned(11 downto 0);
        variable v_s12 : signed(12 downto 0);
        variable v_tm : signed(16 downto 0);
        variable v_d  : signed(12 downto 0);
        variable v_x  : signed(10 downto 0);
        variable v_bt : signed(10 downto 0);
        variable v_e  : unsigned(9 downto 0);
        variable v_e1 : signed(10 downto 0);
        variable v_ad : unsigned(8 downto 0);
        variable v_ix : signed(19 downto 0);
        variable v_dith : unsigned(1 downto 0);
        variable v_w  : unsigned(4 downto 0);
        variable v_cv : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_xp(2) <= a_xp; s_yp(2) <= a_yp;
            for k in 3 to 14 loop s_xp(k) <= s_xp(k - 1); s_yp(k) <= s_yp(k - 1); end loop;

            -- 2: cell ids, cell coordinates (p, q in [-1, 1), 512 = 1), border flag
            q2_i <= a_a(27 downto 20); q2_j <= a_b(27 downto 20);
            q2_p <= signed((not a_a(19)) & a_a(18 downto 10));
            q2_q <= signed((not a_b(19)) & a_b(18 downto 10));
            if a_x = 0 or a_x = g_wm1 or a_first = '1' or a_last = '1' then q2_bd <= '1'; else q2_bd <= '0'; end if;

            -- 3: table addresses: index, map cell (read the previous field's bank, write this one), relief
            v_i := q2_i; v_j := q2_j;
            if s_rad = '1' then v_n := abs(v_i) + abs(v_j); else v_n := v_i + v_j; end if;
            dt_ra <= unsigned(v_n(6 downto 0));
            v_c := v_i + v_j; v_r := v_i - v_j;
            mr_a <= (not s_bank) & unsigned(v_c(5 downto 0)) & unsigned(v_r(4 downto 1));
            mw_a <= s_bank & unsigned(v_c(5 downto 0)) & unsigned(v_r(4 downto 1));
            mw_d <= std_logic_vector(s_y1(13 downto 6));
            if s_gav_sr(0) = '1' and ((q2_p > -256 and q2_p < 256 and q2_q > -256 and q2_q < 256) or q2_bd = '1') then
                mw_we <= '1';
            else
                mw_we <= '0';
            end if;
            tp_ra <= unsigned((not q2_p(9)) & q2_p(8 downto 2));
            tq_ra <= unsigned((not q2_q(9)) & q2_q(8 downto 2));
            v_a11 := abs(resize(q2_p, 11)); v_ap := unsigned(v_a11(9 downto 0));
            v_a11 := abs(resize(q2_q, 11)); v_aq := unsigned(v_a11(9 downto 0));
            q3_adp <= to_unsigned(512, 10) - v_ap;
            q3_adq <= to_unsigned(512, 10) - v_aq;
            q3_dps <= signed((not q2_p(9)) & q2_p(8 downto 0));     -- offset from the nearest corner
            q3_dqs <= signed((not q2_q(9)) & q2_q(8 downto 0));

            -- 4: (RAM) max / min for the octagon distances; button light term, highlight-dot offsets
            q4_mx <= f_umax(q3_adp, q3_adq); q4_mn <= f_umin(q3_adp, q3_adq);
            q4_lin <= -resize(q3_dps, 11) + resize(shift_right(q3_dps, 5), 11) + resize(shift_right(q3_dqs, 2), 11);
            v_dx := resize(q3_dps, 11) + to_signed(37, 11);
            v_dy := resize(q3_dqs, 11) - to_signed(10, 11);
            q4_ddx <= unsigned(abs(v_dx)); q4_ddy <= unsigned(abs(v_dy));

            -- 5: fabric copies of the table reads; octagon partial sums; button base tone
            q5_d  <= signed(dt_q);
            q5_l  <= unsigned(mr_q);
            q5_tp <= signed(tp_q(11 downto 0));
            q5_tq <= signed(tq_q(11 downto 0));
            q5_t1 <= resize(q4_mx, 11) - resize(q4_mx(9 downto 3), 11) + resize(q4_mn(9 downto 1), 11);
            q5_mx <= q4_mx; q5_mn <= q4_mn;
            v_lin := shift_right(q4_lin, 3);
            q5_bt0 <= resize(v_lin, 8) + to_signed(16, 8);
            q5_dmx <= f_umax(q4_ddx, q4_ddy); q5_dmn <= f_umin(q4_ddx, q4_ddy);

            -- 6: s = Tp + Tq (Q6); the held source less the field mean; rc = octagon distance to the corner
            v_s12 := resize(q5_tp, 13) + resize(q5_tq, 13);
            q6_s <= resize(shift_right(v_s12, 2), 10);
            q6_src <= signed(resize(q5_l, 9)) - signed(resize(c_lbar, 9));
            q6_d <= q5_d;
            q6_rc <= f_umax(resize(q5_mx, 11), q5_t1);
            q6_ee <= q5_mn;
            q6_dt <= resize(q5_dmx, 12) - resize(q5_dmx(10 downto 3), 12) + resize(q5_dmn(10 downto 1), 12);
            q6_dmx <= q5_dmx;
            q6_bt0 <= q5_bt0;

            -- 7: source x camera as two gated partial sums; the button rim terms
            q7_pa <= (f_gs(q6_src, c_cam(0), 13) + shift_left(f_gs(q6_src, c_cam(1), 13), 1))
                   + (shift_left(f_gs(q6_src, c_cam(2), 13), 2) + shift_left(f_gs(q6_src, c_cam(3), 13), 3));
            q7_pb <= f_gs(q6_src, c_cam(4), 12) + shift_left(f_gs(q6_src, c_cam(5), 12), 1)
                   + shift_left(f_gs(q6_src, c_cam(6), 12), 2);
            q7_d <= q6_d; q7_s <= q6_s; q7_ee <= q6_ee;
            if q6_rc > C_RB then q7_ec <= resize(q6_rc - C_RB, 10); else q7_ec <= (others => '0'); end if;
            q7_x <= to_signed(C_RB, 11) - signed(q6_rc);
            q7_rd <= f_umax(resize(q6_dmx, 12), q6_dt);
            q7_bt0 <= q6_bt0;

            -- 8: the camera term; dimple distance 0.625 (rc - R); coverage ramp; highlight-dot tone
            v_tm := resize(q7_pa, 17) + shift_left(resize(q7_pb, 17), 4);
            q8_term <= resize(shift_right(v_tm, 5), 12);
            q8_d <= q7_d; q8_s <= q7_s; q8_ee <= q7_ee;
            q8_ec <= resize(q7_ec(9 downto 1), 10) + resize(q7_ec(9 downto 3), 10);
            case to_integer(c_shc) is
                when 7      =>                                   -- x 2, saturated (far pixels stay outside)
                    if q7_x < -256 then q8_cv <= to_signed(-512, 11); else q8_cv <= shift_left(q7_x, 1); end if;
                when 0      => q8_cv <= q7_x;
                when 1      => q8_cv <= shift_right(q7_x, 1);
                when 2      => q8_cv <= shift_right(q7_x, 2);
                when 3      => q8_cv <= shift_right(q7_x, 3);
                when others => q8_cv <= shift_right(q7_x, 4);
            end case;
            q8_btd <= to_signed(31, 11) - signed(resize(q7_rd(11 downto 2), 11));
            q8_bt0 <= q7_bt0;

            -- 9: the state d (clamped, Q6); seam distance (with the button rim); coverage; button tone
            v_d := resize(q8_d, 13) + resize(q8_term, 13);
            if v_d > 127 then q9_d <= to_signed(127, 8);
            elsif v_d < -128 then q9_d <= to_signed(-128, 8);
            else q9_d <= v_d(7 downto 0); end if;
            q9_s <= q8_s;
            if s_btn = '1' and q8_ec < q8_ee then q9_e <= q8_ec; else q9_e <= q8_ee; end if;
            v_x := q8_cv + to_signed(4, 11);
            if v_x < 0 then q9_cov <= (others => '0');
            elsif v_x > 8 then q9_cov <= to_unsigned(8, 4);
            else q9_cov <= unsigned(v_x(3 downto 0)); end if;
            if q8_btd > resize(q8_bt0, 11) then v_bt := q8_btd; else v_bt := resize(q8_bt0, 11); end if;
            if v_bt < 0 then q9_bt <= (others => '0');
            elsif v_bt > 31 then q9_bt <= to_unsigned(31, 5);
            else q9_bt <= unsigned(v_bt(4 downto 0)); end if;

            -- 10 .. 12: d x s as gated sums (two's complement d); pleat distance past the core; |d|
            q10_p01 <= f_gs(q9_s, q9_d(0), 12) + shift_left(f_gs(q9_s, q9_d(1), 12), 1);
            q10_p23 <= f_gs(q9_s, q9_d(2), 12) + shift_left(f_gs(q9_s, q9_d(3), 12), 1);
            q10_p45 <= f_gs(q9_s, q9_d(4), 12) + shift_left(f_gs(q9_s, q9_d(5), 12), 1);
            q10_p67 <= f_gs(q9_s, q9_d(6), 12) - shift_left(f_gs(q9_s, q9_d(7), 12), 1);
            v_e1 := signed(resize(q9_e, 11)) - signed(resize(c_core, 11));
            if v_e1 < 0 then q10_e1 <= (others => '0'); else q10_e1 <= unsigned(v_e1(9 downto 0)); end if;
            v_ad := unsigned(abs(resize(q9_d, 9)));
            if s_gloss = '1' then q10_ad <= v_ad + resize(v_ad(8 downto 1), 9); else q10_ad <= v_ad; end if;
            q10_cov <= q9_cov; q10_bt <= q9_bt;
            q11_q0 <= resize(q10_p01, 15) + shift_left(resize(q10_p23, 15), 2);
            q11_q1 <= resize(q10_p45, 15) + shift_left(resize(q10_p67, 15), 2);
            if q10_ad(8 downto 2) > 31 then r_pt_a(10 downto 6) <= (others => '1');
            else r_pt_a(10 downto 6) <= q10_ad(6 downto 2); end if;
            if q10_e1(9 downto 2) > 63 then r_pt_a(5 downto 0) <= (others => '1');
            else r_pt_a(5 downto 0) <= q10_e1(7 downto 2); end if;
            q11_cov <= q10_cov; q11_bt <= q10_bt;
            q12_pr <= resize(q11_q0, 19) + shift_left(resize(q11_q1, 19), 4);
            q12_cov <= q11_cov; q12_bt <= q11_bt;

            -- 13: tone = 32 + d s 16 with the 2x2 ordered dither; pleat weight (fabric copy)
            v_dith := (s_yp(12) xor s_xp(12)) & s_yp(12);           -- [[0 2] [3 1]] by (y, x) parity
            v_ix := resize(shift_right(q12_pr, 6), 20) + to_signed(128, 20) + signed(resize(v_dith, 20));
            if v_ix < 0 then q13_tone <= (others => '0');
            elsif v_ix > 255 then q13_tone <= (others => '1');
            else q13_tone <= unsigned(v_ix(7 downto 2)); end if;
            q13_w4 <= r_pt_q;
            q13_cov <= q12_cov; q13_bt <= q12_bt;

            -- 14: the colour table address: button, or {tone, pleat weight (dithered)}
            v_w := resize(q13_w4, 5) + ("0000" & (s_xp(13) xor s_yp(13)));
            if s_btn = '1' and q13_cov /= 0 then
                if q13_cov(3) = '1' then ct_ra <= "10" & q13_bt & "111";
                else ct_ra <= "10" & q13_bt & q13_cov(2 downto 0); end if;
            elsif v_w(4 downto 1) > 7 then
                ct_ra <= "0" & q13_tone & "111";
            else
                ct_ra <= "0" & q13_tone & v_w(3 downto 1);
            end if;

            -- 15: (RAM) 16: fabric copy 17: ground outside the generated avid
            q16_y <= unsigned(ct_q(25 downto 16));
            q16_c(1) <= unsigned(ct_q(15 downto 8));
            q16_c(2) <= unsigned(ct_q(7 downto 0));
            if s_gav_sr(14) = '0' then
                q17_y <= c_gy; q17_c <= c_gc;
            else
                q17_y <= q16_y; q17_c <= q16_c;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;
    c_gy <= kk(0)(0);
    c_gc(1) <= kk(0)(1)(7 downto 0);
    c_gc(2) <= kk(0)(2)(7 downto 0);

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 25 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_bah = "00000000" then
                s_bu <= c_gc(1) & "000"; s_bv <= c_gc(2) & "000";
            else
                s_bu <= s_bu + q17_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q17_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q17_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
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

end architecture tufted;
