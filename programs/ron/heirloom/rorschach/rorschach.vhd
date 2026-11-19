-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Rorschach v0.6.0 (phase-2 diag) -- Heirloom #4, rebuilt from the photo WIN_20221124_22_54_05.
--
-- The photo: camera video folded top-to-bottom about row 525.5 (a frame-store
-- mirror; there is no left/right mirror), through an 8-colour cyclic
-- colourizer -- the dark band, then cream, peach, pink, orchid, lavender and
-- pale cyan, wrapping back to the dark band -- whose dark band takes the
-- camera's hue (blue, or red / magenta / green) sample-and-held in cells of
-- 42.25 px (16 SD px), so red/blue boundaries stair-step on a fixed grid while
-- every other contour is smooth.  Busy strips either side of the fold carry
-- small fragments and frame-store line glitches (rows displaced and smeared).
--
-- The program: a 64 x 128 wrapped-phase field of the top half (Osc: a ROM
-- fitted to the photo; Video: the camera, written live), read back folded
-- about the fold line with a wrap-aware bilinear interpolation (four bank
-- reads per pixel), times the colourizer gain, plus hashed fragments per
-- sample-and-hold cell where the field is busy, plus the colour-cycle offset,
-- into the 7-band palette; the dark band's hue sample-and-held per 42.25-px
-- cell.  A per-line engine with a background tear-group unit draws the
-- frame-store tears (row jumps, sideways shifts, colour smears).
--
--   K1  Bands    colourizer gain 0.5 .. 4 cycles (1 = the photo; Video: 2x)
--   K2  Ink      the dark band's width (the blue / red shapes), square law: 0 = gone,
--                50 % = the photo (37/256 of a cycle), 100 % = 4x (149/256);
--                the six pastel bands share the rest equally
--   K3  Palette  colourways (was K6, K6 camera pass; Detail, the busy fragments, is fixed
--                at the photo's 60 %)
--   K4  Speed    bipolar colour cycling, still at the centre (+-8); beyond the
--                default speed (|K4 - 512| > 92) the inkblot also slides sideways
--                in the cycle's direction (square law, ~1 width / 3 s at the ends)
--   K5  Fold     fold line 0.125 .. 1.0 frame heights, linear (0.487 = the photo
--                at 41 %; 0 = eight folded copies)
--   K6  Camera   camera strength (K6 camera pass).  Below 50 % the source (camera, or the
--                Osc scene) shows only where the photo's own inkblot phase is under a level,
--                the rest is the flat ground a black input gives (0 % = no camera); 50 % =
--                the shipped look; above it the chroma is boosted to x1.875 at 100 %.
--                (K3 Palette: Rorschach / Ink (mono) / Swap / Comp / Turn / Twist (chroma
--                swaps and turns) / Night (luma negative) / Vivid (double chroma))
--   S7  Mirror   Mirror / Repeat (the top half reflected, or repeated)
--   S8  Stairs   Hue / All: only the dark band's hue stair-steps (the photo), or
--                the whole picture sample-and-held into 84-px blocks
--   S9  Quad     Off / On: the scene squeezed into the left half and mirrored
--                into the right (a four-way inkblot)
--   S10 Negative Off / On: the scalar inverted (band order reversed)
--   S11 Source   Video / Osc
--   P12 TEAR     frame-store tears: 0 = clean fold .. 100 % = torn into slabs;
--                above the photo's 35 % the mirrored copies also tear away
--                sideways (to half the width at 100 %)
--
-- The store is read with mirrored edges (period two widths), so sideways drift,
-- slip, tears and Quad never show a seam.
--
-- Generated avid; output conditioning (luma [1 3 4 4 3 1]/16 smear, chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the blanking gate.  Latency 24
-- clocks in every mode.  Colours in the hardware convention (U = Cr, V = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K3 = Palette (was K6); K3 Detail fixed at 614 (C_KFIX_3).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.rorschach_pkg.all;
use work.all;

architecture rorschach of program_top is
    constant C_KFIX_3 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(614, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 24;   -- matches processing_delay_clks
    constant C_LIM : integer := 8388608;   -- 128 store rows, Q16
    constant C_HC  : std_logic_vector(15 downto 0) := x"3C6B";

    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;

    -- 16-bit hash stages (see hashes.py)
    function f_h1(k : unsigned(15 downto 0)) return unsigned is
    begin
        return k + shift_left(k, 5);
    end function;
    function f_h2(v1 : unsigned(15 downto 0)) return unsigned is
        variable v2 : unsigned(15 downto 0);
    begin
        v2 := v1 xor shift_right(v1, 7);
        return (v2 + shift_left(v2, 3)) xor unsigned(C_HC);
    end function;
    function f_h3(v3 : unsigned(15 downto 0)) return unsigned is
        variable v4 : unsigned(15 downto 0);
    begin
        v4 := v3 xor shift_right(v3, 9);
        return v4 + shift_left(v4, 11);
    end function;
    function f_h4(v5 : unsigned(15 downto 0)) return unsigned is
    begin
        return v5 xor shift_right(v5, 6);
    end function;
    function v_mul_fy(d : signed(5 downto 0); f : unsigned(4 downto 0)) return signed is
    begin
        return resize(d * signed(resize(f, 6)), 11);
    end function;
    function bidx(rp, cp : std_logic) return integer is
    begin
        if rp = '1' then
            if cp = '1' then return 3; else return 2; end if;
        else
            if cp = '1' then return 1; else return 0; end if;
        end if;
    end function;
    function f_xs(x : unsigned(31 downto 0)) return unsigned is
        variable v : unsigned(31 downto 0);
    begin
        v := x xor shift_left(x, 13);
        v := v xor shift_right(v, 17);
        v := v xor shift_left(v, 5);
        return v;
    end function;


    -- timing (computed per field from the measured raster)
    signal t_cx  : unsigned(12 downto 0) := to_unsigned(2184, 13);
    signal t_cx2 : unsigned(11 downto 0) := to_unsigned(1092, 12);
    signal t_rx  : unsigned(13 downto 0) := to_unsigned(2184, 14);   -- read step (Quad: 2x)
    signal t_cc  : unsigned(12 downto 0) := to_unsigned(1551, 13);
    signal t_ys  : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_hs  : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal t_hd  : std_logic := '1';
    signal t_la, t_lb : unsigned(20 downto 0) := (others => '0');

    -- zone / width / lines
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lcnt, s_hl : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_wd : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_hd : unsigned(10 downto 0) := to_unsigned(1080, 11);

    -- glides (Q.3): 0 K1 Bands, 1 K2 Ink, 2 K3 Detail (fixed), 3 K5 Fold, 4 P12 Tear, 5 K6 Camera
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(387 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(614 * 8, 13),
                         to_unsigned(423 * 8, 13), to_unsigned(358 * 8, 13), to_unsigned(512 * 8, 13));
    -- K6 Camera: reveal level (scene phase, 64 = all) and chroma boost (1 + n/8), per field
    signal c_klv : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal c_bst : unsigned(2 downto 0) := (others => '0');
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w, q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal s_m4 : unsigned(8 downto 0) := (others => '0');
    signal s_s4 : std_logic := '0';
    signal s_k3x : unsigned(8 downto 0) := (others => '0');

    -- serial multiplier (9 slots) and divider (5 slots)
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 13) := (others => '0');
    signal m_a : unsigned(19 downto 0) := (others => '0');
    signal m_b, m_l : unsigned(17 downto 0) := (others => '0');
    signal m_h : unsigned(19 downto 0) := (others => '0');
    signal m_an : unsigned(19 downto 0) := (others => '0');
    signal m_bn : unsigned(17 downto 0) := (others => '0');
    signal d_n : unsigned(25 downto 0) := (others => '0');
    signal d_r : unsigned(12 downto 0) := (others => '0');
    signal d_v : unsigned(11 downto 0) := (others => '0');

    -- per-field values
    signal f_g     : unsigned(9 downto 0) := to_unsigned(255, 10);
    signal f_gw    : unsigned(9 downto 0) := to_unsigned(149, 10);
    signal f_drm   : unsigned(10 downto 0) := (others => '0');
    signal f_av    : unsigned(10 downto 0) := to_unsigned(526, 11);
    signal f_aq8   : unsigned(18 downto 0) := to_unsigned(526 * 256, 19);
    signal f_t, f_t2, f_t4 : unsigned(9 downto 0) := (others => '0');
    signal f_pz, f_po, f_pe : unsigned(7 downto 0) := (others => '0');
    signal f_dym   : unsigned(8 downto 0) := to_unsigned(106, 9);
    signal f_dxm   : unsigned(12 downto 0) := to_unsigned(575, 13);
    signal f_hmax  : unsigned(5 downto 0) := to_unsigned(6, 6);
    signal f_dxc   : unsigned(11 downto 0) := to_unsigned(413, 12);
    signal f_sp    : unsigned(2 downto 0) := (others => '0');
    signal f_cs    : unsigned(13 downto 0) := (others => '0');
    signal f_kstep : unsigned(17 downto 0) := to_unsigned(15947, 18);
    signal f_amp   : unsigned(8 downto 0) := to_unsigned(307, 9);
    signal f_rot   : unsigned(15 downto 0) := (others => '0');
    signal f_rac   : unsigned(15 downto 0) := (others => '0');
    signal f_fc    : unsigned(1 downto 0) := (others => '0');
    signal f_gseed : unsigned(31 downto 0) := x"9E3779B9";
    signal f_nseed : unsigned(15 downto 0) := x"C4E3";
    signal f_pl    : unsigned(2 downto 0) := (others => '0');
    signal f_rep, f_stair, f_quad, f_neg, f_osc : std_logic := '0';
    -- phase 2: sideways drift (K4), slip of the mirrored copies (P12), Ink thresholds (K2)
    signal s_mdr  : unsigned(8 downto 0) := (others => '0');
    signal f_xd   : unsigned(22 downto 0) := (others => '0');
    signal f_sd   : unsigned(9 downto 0) := (others => '0');
    signal f_slip : unsigned(21 downto 0) := (others => '0');
    signal f_b0, f_b1 : signed(23 downto 0) := to_signed(1092, 24);
    signal f_dk   : unsigned(7 downto 0) := to_unsigned(37, 8);
    signal f_w6   : unsigned(8 downto 0) := to_unsigned(219, 9);
    signal f_s6   : unsigned(13 downto 0) := to_unsigned(9344, 14);
    signal f_tacc : unsigned(15 downto 0) := (others => '0');
    -- band lookup (K2 Ink): phase -> {dark, band - 1}, refilled every field from the thresholds
    type t_bram is array (0 to 255) of std_logic_vector(3 downto 0);
    signal bram : t_bram := (others => "1000");
    signal bf_we : std_logic := '0';
    signal bf_a  : unsigned(7 downto 0) := (others => '0');
    signal bf_d  : std_logic_vector(3 downto 0) := "1000";
    signal bf_b  : unsigned(2 downto 0) := (others => '0');
    signal bf_tn : unsigned(7 downto 0) := (others => '0');
    signal r_bd  : std_logic_vector(3 downto 0) := "1000";

    -- per-line engine; it runs 24 clocks after the source avid fall (the
    -- pixel path has drained) and writes the line registers (l_*) directly
    signal le_st : unsigned(3 downto 0) := (others => '0');
    signal le_dl : unsigned(4 downto 0) := (others => '0');
    signal n_vy  : unsigned(19 downto 0) := (others => '0');
    signal n_ym, n_ymt, n_yr, n_yb : signed(25 downto 0) := (others => '0');
    signal n_up, n_adv, n_new, n_odd : std_logic := '0';
    signal n_rpw : unsigned(25 downto 0) := (others => '0');
    signal n_d   : signed(20 downto 0) := (others => '0');
    signal n_dab : unsigned(19 downto 0) := (others => '0');
    signal n_p   : unsigned(7 downto 0) := (others => '0');
    signal n_grem : unsigned(6 downto 0) := (others => '0');
    signal n_vv  : unsigned(11 downto 0) := (others => '0');
    signal g_on  : std_logic := '0';   -- the current tear group is torn (its draw is read from tg_* directly)
    signal l_ra0, l_ra1, l_hrow : unsigned(5 downto 0) := (others => '0');
    signal l_r0p, l_hon, l_wok : std_logic := '0';
    signal l_fy  : unsigned(4 downto 0) := (others => '0');
    signal l_u0, l_cp0 : signed(23 downto 0) := (others => '0');
    signal l_hc  : unsigned(3 downto 0) := (others => '0');
    signal l_grp : unsigned(7 downto 0) := (others => '0');
    signal l_wrow : unsigned(6 downto 0) := (others => '0');

    -- tear-group unit: draws and scales the next group in the background (one serial multiplier)
    signal tg_go, tg_m, tg_hf : std_logic := '0';
    signal tg_st : unsigned(3 downto 0) := (others => '0');
    signal tgs, tg_r2 : unsigned(31 downto 0) := x"9E3779B9";
    signal tg_r1 : unsigned(15 downto 0) := (others => '0');
    signal tg_a, tg_acc : unsigned(21 downto 0) := (others => '0');
    signal tg_b : unsigned(7 downto 0) := (others => '0');
    signal tg_n : unsigned(3 downto 0) := (others => '0');
    signal tg_lines : unsigned(6 downto 0) := (others => '0');
    signal tg_act : unsigned(7 downto 0) := (others => '1');
    signal tg_dy, tg_dx, tg_cpo : signed(23 downto 0) := (others => '0');
    signal tg_hc : unsigned(3 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- beam (stage 0)
    signal b_u, b_cp : signed(23 downto 0) := (others => '0');
    signal b_t : unsigned(21 downto 0) := (others => '0');   -- b_u folded: mirrored edges
    signal b_cst, b_first : std_logic := '0';
    signal b_cell : unsigned(7 downto 0) := (others => '0');

    -- stores
    signal a_b0, a_b1, a_b2, a_b3 : unsigned(10 downto 0) := (others => '0');
    signal a_hue : unsigned(11 downto 0) := (others => '0');
    signal r_s0h, r_s1h, r_s2h, r_s3h : std_logic_vector(3 downto 0) := (others => '0');
    signal r_s0l, r_s1l, r_s2l, r_s3l : std_logic_vector(1 downto 0) := (others => '0');
    signal r_shue : std_logic_vector(1 downto 0) := (others => '0');
    type t_vram is array (0 to 2047) of std_logic_vector(3 downto 0);
    type t_hram is array (0 to 4095) of std_logic_vector(1 downto 0);
    signal vr0, vr1, vr2, vr3 : t_vram := (others => (others => '0'));
    signal vh : t_hram := (others => (others => '0'));
    signal r_v0, r_v1, r_v2, r_v3 : std_logic_vector(3 downto 0) := (others => '0');
    signal r_vh : std_logic_vector(1 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of vr0 : signal is true;
    attribute no_rw_check of vr1 : signal is true;
    attribute no_rw_check of vr2 : signal is true;
    attribute no_rw_check of vr3 : signal is true;
    attribute no_rw_check of vh  : signal is true;
    attribute no_rw_check of bram : signal is true;
    -- video write side
    signal w_yf : unsigned(13 downto 0) := (others => '0');
    signal w_uw : unsigned(21 downto 0) := (others => '0');
    signal w_ev, w_evb : std_logic := '0';
    signal w_y0 : unsigned(9 downto 0) := (others => '0');
    signal w_h0, w_hb : std_logic_vector(1 downto 0) := (others => '0');
    signal w_colb : unsigned(5 downto 0) := (others => '0');
    signal w_yn : unsigned(9 downto 0) := (others => '0');
    signal w_col, w_col2 : unsigned(5 downto 0) := (others => '0');
    signal w_ev2 : std_logic := '0';
    signal w_pr : unsigned(19 downto 0) := (others => '0');
    signal w_hc2, w_hc3 : std_logic_vector(1 downto 0) := (others => '0');
    signal w_we : std_logic_vector(3 downto 0) := (others => '0');
    signal w_hwe : std_logic := '0';
    signal w_a : unsigned(10 downto 0) := (others => '0');
    signal w_ha : unsigned(11 downto 0) := (others => '0');
    signal w_d : std_logic_vector(3 downto 0) := (others => '0');
    signal w_hd : std_logic_vector(1 downto 0) := (others => '0');

    -- pixel path
    signal p1_cp, p2_cp : std_logic := '0';
    signal p1_fx, p2_fx, p3_fx, p4_fx, p5_fx, p6_fx, p7_fx : unsigned(4 downto 0) := (others => '0');
    signal ha_1, ha_2, ha_3 : unsigned(15 downto 0) := (others => '0');
    signal p_cst : t_sl(1 to 2) := (others => '0');
    signal p_ast : t_sl(1 to 5) := (others => '0');   -- start of a cell pair (Stairs All)
    signal p_tk : t_sl(2 to 5) := (others => '0');
    signal p3_q00, p3_q01, p3_q10, p3_q11 : unsigned(5 downto 0) := (others => '0');
    signal p3_hh : unsigned(1 downto 0) := (others => '0');
    signal p4_na : signed(8 downto 0) := (others => '0');
    signal p5_na : signed(8 downto 0) := (others => '0');
    type t_s9a is array (natural range <>) of signed(8 downto 0);
    signal p_n : t_s9a(6 to 8) := (others => (others => '0'));
    type t_u3a is array (natural range <>) of unsigned(2 downto 0);
    signal p4_hz, p4_hm : unsigned(1 downto 0) := (others => '0');
    signal p_hue : t_u3a(5 to 5) := (others => (others => '0'));   -- dark-band hue (to stage 13 via ringp)
    signal p_kz : std_logic_vector(4 to 10) := (others => '0');       -- K6: pixel shows the flat ground
    signal p3_sk : unsigned(5 downto 0) := (others => '0');           -- K6: the photo scene's phase at the corner
    signal s_tu23, s_tv23 : signed(13 downto 0) := (others => '0');   -- K6: chroma boost d n
    signal p4_d0, p4_d1 : signed(5 downto 0) := (others => '0');
    signal p4_s0, p4_s1 : std_logic := '0';
    signal p4_q00, p4_q01, p4_q10, p4_q11 : unsigned(5 downto 0) := (others => '0');
    signal p4_mx : unsigned(5 downto 0) := (others => '0');
    signal p5_p0, p5_p1 : signed(10 downto 0) := (others => '0');
    signal p5_s0, p5_s1 : std_logic := '0';
    signal p5_q00, p5_q01, p5_q10, p5_q11 : unsigned(5 downto 0) := (others => '0');
    signal p5_b4 : unsigned(2 downto 0) := (others => '0');
    signal p6_a, p6_b : unsigned(10 downto 0) := (others => '0');
    signal p6_w4 : unsigned(2 downto 0) := (others => '0');
    signal p7_a, p7_b : unsigned(10 downto 0) := (others => '0');
    signal p7_dd : signed(10 downto 0) := (others => '0');
    signal p7_sx : std_logic := '0';
    signal p7_amp : unsigned(9 downto 0) := (others => '0');
    signal p8_a, p8_b : unsigned(10 downto 0) := (others => '0');
    signal p8_px : signed(15 downto 0) := (others => '0');
    signal p8_sx : std_logic := '0';
    signal p8_fx : unsigned(4 downto 0) := (others => '0');
    signal p8_ae : unsigned(6 downto 0) := (others => '0');
    signal p9_phi : unsigned(10 downto 0) := (others => '0');
    signal p9_np : signed(16 downto 0) := (others => '0');
    signal p10_gp : unsigned(20 downto 0) := (others => '0');
    signal p10_phi : unsigned(10 downto 0) := (others => '0');
    signal p10_no : signed(7 downto 0) := (others => '0');
    signal p11_pg : unsigned(7 downto 0) := (others => '0');
    signal p11_no : signed(7 downto 0) := (others => '0');
    signal p12_ph : unsigned(7 downto 0) := (others => '0');
    signal p14_sel, p14_hsel, p14_ah : unsigned(3 downto 0) := (others => '0');
    signal p15_x : std_logic_vector(63 downto 0) := x"10EB808080808080";   -- f_palx: Y, 251-Y, U, V, -U, -V, 2U, 2V
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p1_c, p2_cl : unsigned(5 downto 0) := (others => '0');
    signal p1_first : std_logic := '0';
    signal p2_cn : unsigned(3 downto 0) := (others => '0');

    -- output conditioning
    signal s_wp : unsigned(7 downto 0) := (others => '0');
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_ring8 is array (0 to 255) of std_logic_vector(7 downto 0);
    signal ringc : t_ring := (others => x"8080");
    signal ringp : t_ring8 := (others => (others => '0'));
    attribute no_rw_check of ringc : signal is true;
    attribute no_rw_check of ringp : signal is true;
    signal r_cb, r_cbf, r_cw : std_logic_vector(15 downto 0) := x"8080";
    signal r_pb, r_pbf : std_logic_vector(7 downto 0) := (others => '0');
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 box inputs
    signal s_y17, s_y18, s_y19, s_y20, s_y21, s_y22x, s_yf22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pa, s_pb, s_pc : unsigned(10 downto 0) := to_unsigned(128, 11);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin


    ---------------------------------------------------------------------------
    -- Scene ROMs (Osc) and the camera store (Video): four banks by (row
    -- parity, column parity) so the four bilinear corners come out together.
    ---------------------------------------------------------------------------
    p_srom : process(clk)
    begin
        if rising_edge(clk) then
            r_s0h <= C_S0H(to_integer(a_b0)); r_s0l <= C_S0L(to_integer(a_b0));
            r_s1h <= C_S1H(to_integer(a_b1)); r_s1l <= C_S1L(to_integer(a_b1));
            r_s2h <= C_S2H(to_integer(a_b2)); r_s2l <= C_S2L(to_integer(a_b2));
            r_s3h <= C_S3H(to_integer(a_b3)); r_s3l <= C_S3L(to_integer(a_b3));
            r_shue <= C_SHUE(to_integer(a_hue));
        end if;
    end process p_srom;

    p_vr0 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(0) = '1' then vr0(to_integer(w_a)) <= w_d; end if;
            r_v0 <= vr0(to_integer(a_b0));
        end if;
    end process p_vr0;
    p_vr1 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(1) = '1' then vr1(to_integer(w_a)) <= w_d; end if;
            r_v1 <= vr1(to_integer(a_b1));
        end if;
    end process p_vr1;
    p_vr2 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(2) = '1' then vr2(to_integer(w_a)) <= w_d; end if;
            r_v2 <= vr2(to_integer(a_b2));
        end if;
    end process p_vr2;
    p_vr3 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(3) = '1' then vr3(to_integer(w_a)) <= w_d; end if;
            r_v3 <= vr3(to_integer(a_b3));
        end if;
    end process p_vr3;
    p_bram : process(clk)
    begin
        if rising_edge(clk) then
            if bf_we = '1' then bram(to_integer(bf_a)) <= bf_d; end if;
            r_bd <= bram(to_integer(p12_ph));
        end if;
    end process p_bram;
    p_vh : process(clk)
    begin
        if rising_edge(clk) then
            if w_hwe = '1' then vh(to_integer(w_ha)) <= w_hd; end if;
            r_vh <= vh(to_integer(a_hue));
        end if;
    end process p_vh;


    -- delay rings (phase-2 fit, were FF chains): written at s_wp, read 6 back, plus one fabric
    -- register = the sample 8 clocks back.  The chroma ring is written from its own copy register
    -- (so p16 stays beside the colourway logic) and read 5 back: still p16 8 clocks back.
    p_ringc : process(clk)
    begin
        if rising_edge(clk) then
            s_wp <= s_wp + 1;
            r_cw <= std_logic_vector(p16_u(9 downto 2)) & std_logic_vector(p16_v(9 downto 2));
            ringc(to_integer(s_wp)) <= r_cw;
            r_cb <= ringc(to_integer(s_wp - 5));
            r_cbf <= r_cb;
        end if;
    end process p_ringc;
    p_ringp : process(clk)
    begin
        if rising_edge(clk) then
            ringp(to_integer(s_wp)) <= "000" & p_tk(5) & p_ast(5) & std_logic_vector(p_hue(5));
            r_pb <= ringp(to_integer(s_wp - 6));
            r_pbf <= r_pb;
        end if;
    end process p_ringp;


    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field sequencer (serial multiplier,
    -- serial divider, the P12 tear curves by shift-add), field counters
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_pr : unsigned(37 downto 0);
        variable v_rm : unsigned(12 downto 0);
        variable v_t  : unsigned(9 downto 0);
        variable v_10 : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then

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
            elsif s_zc /= 1023 then
                s_zc <= s_zc + 1;
            end if;

            -- width (source avid pixels per line, max over the field) and active lines
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
                if s_lcnt /= 2047 then s_lcnt <= s_lcnt + 1; end if;
            end if;
            g_wm1 <= s_w - 1;
            if s_w < 512 then s_wd <= to_unsigned(1920, 12); else s_wd <= s_w; end if;
            if s_hl < 200 then s_hd <= to_unsigned(1080, 11); else s_hd <= s_hl; end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_hl <= s_lcnt; s_lcnt <= (others => '0');
                -- K4 Speed: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 7 then s_s4 <= '0'; v_u := unsigned(v_x) - 8; s_m4 <= v_u(8 downto 0);
                elsif v_x < -7 then s_s4 <= '1'; v_u := unsigned(-v_x) - 8; s_m4 <= v_u(8 downto 0);
                else s_s4 <= '0'; s_m4 <= (others => '0'); end if;
                -- labelled knob: 0..7 at program load is the label index
                if unsigned(registers_in(2)) < 8 then f_pl <= unsigned(registers_in(2)(2 downto 0));
                else f_pl <= unsigned(registers_in(2)(9 downto 7)); end if;
                f_rep   <= registers_in(6)(0);
                f_stair <= registers_in(6)(1);
                f_quad  <= registers_in(6)(2);
                f_neg   <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- colour cycle; reseed the tears and fragments every 4 fields
                if s_s4 = '1' then f_rac <= f_rac - resize(f_drm, 16);
                else f_rac <= f_rac + resize(f_drm, 16); end if;
                f_fc <= f_fc + 1;
                if f_fc = 3 then f_gseed <= f_gseed + x"9E3779B9"; end if;
            end if;
            f_rot <= f_rac;
            -- drift depth: beyond the default speed (s_m4 = 80 at K4 600), 0 there (+4 for pot noise)
            if s_m4 > 84 then s_mdr <= s_m4 - 84; else s_mdr <= (others => '0'); end if;
            -- read step: Quad reads two store widths per line (mirrored about the centre)
            if f_quad = '1' then t_rx <= shift_left(resize(t_cx, 14), 1); else t_rx <= resize(t_cx, 14); end if;
            -- line start bases (store column + 0.5, Q16; half the read step = the first pixel's
            -- centre): even copies, and odd copies slipped by P12
            f_b0 <= signed(resize(t_rx(13 downto 1), 24)) + signed(resize(f_xd, 24));
            f_b1 <= f_b0 + signed(resize(f_slip, 24));
            f_nseed <= f_gseed(15 downto 0) xor x"5A5A";
            f_amp <= s_g(2)(12 downto 4);
            if s_g(2)(12 downto 3) > 614 then s_k3x <= resize(s_g(2)(12 downto 3) - 614, 9); else s_k3x <= (others => '0'); end if;
            f_aq8 <= f_av & x"00";

            -- glides: K1, K2, K3, K5, P12 ease 1/8 of the gap per field (deadband 4)
            if s_zone = '1' and s_zc >= 4 and s_zc < 28 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(C_KFIX_3) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when 4 => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
                            when others => gl_k <= unsigned(registers_in(5)) & "000"; gl_o <= s_g(5);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when 4 => s_g(4) <= gl_n;
                                when others => s_g(5) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- K6 Camera (after the glides): below 50 % a reveal level picked so the revealed share
            -- of the photo's scene grows linearly with the knob (the scene's phase histogram);
            -- above, the chroma boost n = (k - 512) / 64
            if s_zone = '1' and s_zc = 30 then
                if s_g(5)(12) = '1' then
                    c_klv <= to_unsigned(64, 7); c_bst <= s_g(5)(11 downto 9);
                else
                    c_bst <= (others => '0');
                    case to_integer(s_g(5)(11 downto 7)) is
                        when 0 => c_klv <= to_unsigned(0, 7);   when 1 => c_klv <= to_unsigned(3, 7);
                        when 2 => c_klv <= to_unsigned(5, 7);   when 3 => c_klv <= to_unsigned(6, 7);
                        when 4 => c_klv <= to_unsigned(8, 7);   when 5 => c_klv <= to_unsigned(10, 7);
                        when 6 => c_klv <= to_unsigned(12, 7);  when 7 => c_klv <= to_unsigned(13, 7);
                        when 8 => c_klv <= to_unsigned(15, 7);  when 9 => c_klv <= to_unsigned(16, 7);
                        when 10 => c_klv <= to_unsigned(17, 7); when 11 => c_klv <= to_unsigned(17, 7);
                        when 12 => c_klv <= to_unsigned(18, 7); when 13 => c_klv <= to_unsigned(20, 7);
                        when 14 => c_klv <= to_unsigned(21, 7); when 15 => c_klv <= to_unsigned(22, 7);
                        when 16 => c_klv <= to_unsigned(24, 7); when 17 => c_klv <= to_unsigned(26, 7);
                        when 18 => c_klv <= to_unsigned(26, 7); when 19 => c_klv <= to_unsigned(27, 7);
                        when 20 => c_klv <= to_unsigned(28, 7); when 21 => c_klv <= to_unsigned(29, 7);
                        when 22 => c_klv <= to_unsigned(29, 7); when 23 => c_klv <= to_unsigned(30, 7);
                        when 24 => c_klv <= to_unsigned(33, 7); when 25 => c_klv <= to_unsigned(36, 7);
                        when 26 => c_klv <= to_unsigned(39, 7); when 27 => c_klv <= to_unsigned(45, 7);
                        when 28 => c_klv <= to_unsigned(48, 7); when 29 => c_klv <= to_unsigned(52, 7);
                        when 30 => c_klv <= to_unsigned(56, 7); when others => c_klv <= to_unsigned(60, 7);
                    end case;
                end if;
            end if;

            -- serial products: slot p = zc/32 (1..9): phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            -- operands are registered from the slot number (phase 0..1), loaded at phase 2,
            -- shifted at phases 3..20, stored at phase 22
            if s_zone = '1' and v_p >= 1 and v_p <= 13 and v_ph = 1 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 13 and v_ph >= 2 and v_ph <= 19 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 13 loop
                if s_zone = '1' and v_p = k and v_ph = 21 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            v_t := s_g(4)(12 downto 3);
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                m_a <= m_an; m_b <= m_bn;
            end if;
            if true then
                case to_integer(q_p) is
                    when 1 => m_an <= resize(s_g(0)(12 downto 3), 20); m_bn <= resize(s_g(0)(12 downto 3), 18);
                    when 2 => m_an <= m_h(1 downto 0) & m_l;  m_bn <= to_unsigned(896, 18);   -- k1^2 (slot 1)
                    when 3 => m_an <= resize(f_g, 20);        m_bn <= to_unsigned(599, 18);
                    when 4 => m_an <= resize(s_m4, 20);       m_bn <= resize(s_m4, 18);
                    when 5 => m_an <= resize(m_l, 20);       m_bn <= to_unsigned(338, 18);   -- m4^2 (slot 4)
                    when 6 => m_an <= resize(s_g(3)(12 downto 3), 20); m_bn <= to_unsigned(947, 18);
                    when 7 => m_an <= resize(v_t, 20);        m_bn <= resize(v_t, 18);
                    when 8 => m_an <= resize(f_t2, 20);       m_bn <= resize(f_t2, 18);
                    when 10 => m_an <= resize(s_mdr, 20);     m_bn <= resize(s_mdr, 18);   -- drift: depth^2
                    when 11 => m_an <= resize(s_g(1)(12 downto 3), 20); m_bn <= resize(s_g(1)(12 downto 3), 18);   -- K2 Ink: k2^2
                    when 12 => m_an <= m_h(1 downto 0) & m_l; m_bn <= to_unsigned(150, 18);   -- d = k2^2 * 150 / 2^20
                    when 13 => m_an <= resize(f_w6, 20);      m_bn <= to_unsigned(10923, 18);   -- pastel band: w6 / 6, Q8
                    when others => m_an <= resize(s_k3x, 20); m_bn <= to_unsigned(3, 18);   -- spill: 0..1 quarter
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(2) = '1' then f_g <= to_unsigned(128, 10) + v_pr(29 downto 20); end if;
            if q_wr(3) = '1' then f_gw <= v_pr(19 downto 10); end if;
            if q_wr(5) = '1' then f_drm <= v_pr(26 downto 16); end if;
            if q_wr(6) = '1' then f_av <= to_unsigned(135, 11) + resize(v_pr(19 downto 10), 11); end if;   -- 135 .. 1081
            if q_wr(7) = '1' then f_t2 <= v_pr(19 downto 10); end if;
            if q_wr(8) = '1' then f_t4 <= v_pr(19 downto 10); end if;
            if q_wr(9) = '1' then f_sp <= v_pr(12 downto 10); end if;   -- spill, quarters (0..1)
            -- sideways drift: depth^2 / 8 Q16 columns per field (mirrored edges: wraps at two widths)
            if q_wr(10) = '1' then
                if s_s4 = '1' then f_xd <= f_xd - resize(v_pr(17 downto 3), 23);
                else f_xd <= f_xd + resize(v_pr(17 downto 3), 23); end if;
            end if;
            if q_wr(12) = '1' then f_dk <= v_pr(27 downto 20); end if;   -- 0 .. 37 (512) .. 149
            if q_wr(13) = '1' then f_s6 <= v_pr(21 downto 8); end if;

            -- P12 slip (zc 326..327): the odd copies' sideways offset, 0 up to the photo's 35 %,
            -- then 3072 Q16 per step (half the width at 100 %)
            if s_zone = '1' and s_zc = 326 then
                if v_t > 358 then f_sd <= v_t - 358; else f_sd <= (others => '0'); end if;
            elsif s_zone = '1' and s_zc = 327 then
                f_slip <= shift_left(resize(f_sd, 22), 11) + shift_left(resize(f_sd, 22), 10);
            end if;
            -- K2 Ink: the dark band d (slot 12); the six pastel bands split the rest, thresholds
            -- T_1 = d, T_k = d + ceil((k - 1) * (256 - d) / 6) (= the photo's 37, 74, 110, 147, 183,
            -- 220 at d = 37); the band lookup is refilled at zc 512..767, one phase per clock
            bf_we <= '0';
            if s_zone = '1' and s_zc = 408 then
                f_w6 <= to_unsigned(256, 9) - resize(f_dk, 9);
            elsif s_zone = '1' and s_zc = 511 then
                bf_tn <= f_dk; bf_b <= (others => '0'); f_tacc <= resize(f_s6, 16) + 255;
            elsif s_zone = '1' and s_zc(9 downto 8) = "10" then
                v_10 := resize(bf_b, 11);
                if s_zc(7 downto 0) >= bf_tn and bf_b /= 6 then
                    v_10 := resize(bf_b, 11) + 1;
                    bf_tn <= f_dk + f_tacc(15 downto 8); f_tacc <= f_tacc + f_s6;
                end if;
                bf_b <= v_10(2 downto 0);
                bf_we <= '1'; bf_a <= s_zc(7 downto 0);
                if v_10(2 downto 0) = "000" then bf_d <= "1000";
                else bf_d <= '0' & std_logic_vector(v_10(2 downto 0) - 1); end if;
            end if;

            -- P12 tear curves by shift-add (zc 320..324)
            if s_zone = '1' and s_zc = 320 then
                f_t <= v_t;
                v_10 := resize(v_t(9 downto 2), 11) - resize(v_t(9 downto 5), 11) + resize(f_t4(9 downto 4), 11);
                if v_10 > 255 then f_pz <= (others => '1'); else f_pz <= v_10(7 downto 0); end if;
                f_po <= resize(v_t(9 downto 5), 8) + resize(f_t4(9 downto 3), 8);
                f_pe <= resize(v_t(9 downto 3), 8) + resize(f_t4(9 downto 4), 8);
                f_dym <= to_unsigned(78, 9) + resize(f_t2(9 downto 2), 9) - resize(f_t2(9 downto 5), 9);
                f_hmax <= to_unsigned(6, 6) + resize(f_t4(9 downto 5), 6);
            elsif s_zone = '1' and s_zc = 321 then
                f_cs <= to_unsigned(85, 14) + resize(f_t, 14) + resize(f_t(9 downto 2), 14);
                f_dxm <= resize(shift_left(resize(f_t4, 13), 1) + resize(f_t4, 13), 13);
            elsif s_zone = '1' and s_zc = 322 then
                f_dxm <= resize(f_cs, 13) + f_dxm - resize(f_t4(9 downto 3), 13);
            elsif s_zone = '1' and s_zc = 323 then
                f_cs <= resize(f_dxm(12 downto 1), 14) + resize(f_dxm(12 downto 3), 14);
                f_dxc <= resize(f_dxm(12 downto 4), 12) + resize(f_dxm(12 downto 5), 12);
            elsif s_zone = '1' and s_zc = 324 then
                f_dxc <= resize(f_cs, 12) + f_dxc;
            end if;

            -- serial divider: slots at zc = 32 * k, k = 23..27: phase 0 load, 1..26 steps, 27 store
            if s_zone = '1' and v_p >= 23 and v_p <= 27 then
                if v_ph = 0 then
                    d_r <= (others => '0');
                    case v_p is
                        when 23 => d_n <= to_unsigned(4194304, 26); d_v <= s_wd;
                        when 24 => d_n <= to_unsigned(2978325, 26); d_v <= s_wd;
                        when 26 => d_n <= to_unsigned(276480, 26);  d_v <= resize(s_hd, 12);
                        when others => d_n <= t_ys & "000000000000000"; d_v <= resize(f_av, 12);
                    end case;
                elsif v_ph <= 26 then
                    v_rm := d_r(11 downto 0) & d_n(25);
                    if v_rm >= resize(d_v, 13) then
                        d_r <= v_rm - resize(d_v, 13); d_n <= d_n(24 downto 0) & '1';
                    else
                        d_r <= v_rm; d_n <= d_n(24 downto 0) & '0';
                    end if;
                elsif v_ph = 27 then
                    case v_p is
                        when 23 => t_cx <= d_n(12 downto 0); t_cx2 <= d_n(12 downto 1);
                        when 24 => t_cc <= d_n(12 downto 0);
                        when 26 => t_ys <= d_n(10 downto 0);
                        when 27 => f_kstep <= d_n(17 downto 0);
                        when others => null;
                    end case;
                end if;
            end if;
            -- derived per-field values (zc 896..898): T_HS = H * 243 / 1024
            if s_zone = '1' and s_zc = 896 then
                t_la <= shift_left(resize(s_hd, 21), 8) - shift_left(resize(s_hd, 21), 3);
                t_lb <= shift_left(resize(s_hd, 21), 2) + resize(s_hd, 21);
                if s_wd >= 1280 then t_hd <= '1'; else t_hd <= '0'; end if;
            elsif s_zone = '1' and s_zc = 897 then
                t_la <= t_la - t_lb;
            elsif s_zone = '1' and s_zc = 898 then
                t_hs <= resize(t_la(18 downto 10), 9);
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;


    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start, field-latched width)
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
    -- Per-line engine: 24 clocks after the source avid fall (the pixel path
    -- has drained) it computes the next line straight into the line
    -- registers; line 0 runs late in the vblank zone.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y : signed(25 downto 0);
        variable v_v : signed(25 downto 0);
        variable v_gdy, v_gdx, v_gcp : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                n_vy <= resize(t_ys(10 downto 1), 20);
                n_ym <= signed(resize(f_kstep(17 downto 1), 26)); n_up <= '1';
                n_rpw <= resize(f_kstep(17 downto 1), 26);
                n_grem <= (others => '0');
                n_odd <= '0';
            end if;
            if data_in.avid = '0' and s_avq = '1' then
                le_dl <= to_unsigned(24, 5);
            elsif le_dl /= 0 then
                le_dl <= le_dl - 1;
            end if;
            if s_zone = '1' and s_zc = 904 then
                le_st <= to_unsigned(2, 4);
            elsif le_dl = 1 then
                le_st <= to_unsigned(1, 4);
            elsif le_st /= 0 then
                if le_st = 9 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
            end if;

            tg_go <= '0';
            case to_integer(le_st) is
                when 1 =>
                    -- advance one line
                    n_vy <= n_vy + t_ys;
                    if n_up = '1' or f_rep = '1' then n_ymt <= n_ym + signed(resize(f_kstep, 26));
                    else n_ymt <= n_ym - signed(resize(f_kstep, 26)); end if;
                    n_rpw <= n_rpw + f_kstep;
                    n_adv <= '1';
                    if n_grem = 0 then n_new <= '1'; else n_new <= '0'; n_grem <= n_grem - 1; end if;
                when 2 =>
                    -- fold: bounce (Mirror) or wrap (Repeat) at 128 store rows
                    n_adv <= '0';
                    if n_adv = '1' then
                        -- n_odd: copy parity (flips at every bounce or wrap), for the P12 slip
                        if f_rep = '1' then
                            if n_ymt >= C_LIM then n_ym <= n_ymt - C_LIM; n_odd <= not n_odd; else n_ym <= n_ymt; end if;
                        elsif n_up = '1' then
                            if n_ymt >= C_LIM then n_ym <= 2 * C_LIM - n_ymt; n_up <= '0'; n_odd <= not n_odd; else n_ym <= n_ymt; end if;
                        else
                            if n_ymt < 0 then n_ym <= -n_ymt; n_up <= '1'; n_odd <= not n_odd; else n_ym <= n_ymt; end if;
                        end if;
                    else
                        n_new <= '1';   -- line 0 (zone run)
                    end if;
                    n_d <= signed(resize(n_vy, 21)) - signed(resize(f_aq8, 21));
                when 3 =>
                    if n_d < 0 then n_dab <= unsigned(-n_d(19 downto 0)); else n_dab <= unsigned(n_d(19 downto 0)); end if;
                when 4 =>
                    -- tear probability by distance from the fold
                    if n_dab >= 70 * 256 and n_dab <= 235 * 256 then n_p <= f_pz;
                    elsif n_dab > 470 * 256 then n_p <= f_pe;
                    else n_p <= f_po; end if;
                when 5 =>
                    -- a new tear group: take the precomputed draw (it stays in tg_* for the whole group;
                    -- the next one is drawn on the group's last line, state 9)
                    if n_new = '1' then
                        if tg_lines = 0 then n_grem <= (others => '0'); else n_grem <= tg_lines - 1; end if;
                        if tg_act < n_p then g_on <= '1'; else g_on <= '0'; end if;
                    end if;
                when 6 =>
                    if g_on = '1' then v_gdy := tg_dy; v_gdx := tg_dx; v_gcp := tg_cpo;
                    else v_gdy := (others => '0'); v_gdx := (others => '0'); v_gcp := (others => '0'); end if;
                    v_y := n_ym + resize(v_gdy, 26);
                    if v_y < 0 then n_yr <= (others => '0');
                    elsif v_y >= C_LIM then n_yr <= to_signed(C_LIM - 1, 26);
                    else n_yr <= v_y; end if;
                    if n_odd = '1' then l_u0 <= f_b1 + v_gdx; else l_u0 <= f_b0 + v_gdx; end if;
                    l_cp0 <= v_gcp - 16288;
                    l_hon <= g_on and tg_hf; l_hc <= tg_hc;
                when 7 =>
                    n_yb <= n_yr - 32768;
                    l_hrow <= unsigned(n_yr(22 downto 17));
                    l_grp <= n_vy(17 downto 10);
                    if n_rpw(25 downto 23) = "000" then l_wok <= '1'; else l_wok <= '0'; end if;
                    l_wrow <= n_rpw(22 downto 16);
                when 8 =>
                    v_v := n_yb;
                    if v_v < 0 then v_v := (others => '0');
                    elsif v_v >= 127 * 65536 then v_v := to_signed(127 * 65536 - 1, 26); end if;
                    n_vv <= unsigned(v_v(22 downto 11));
                when 9 =>
                    if n_grem = 0 then tg_go <= '1'; end if;   -- last line of the group: draw the next
                    l_ra0 <= resize(shift_right(resize(n_vv(11 downto 5), 8) + 1, 1), 6);
                    l_ra1 <= n_vv(11 downto 6);
                    l_r0p <= n_vv(5);
                    l_fy <= n_vv(4 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Tear-group unit: draws the next group (height, row jump, sideways shift,
    -- cell offset, smear) with one serial shift-add multiplier, ~45 clocks;
    -- started by the line engine when it takes a group, and in the vblank zone
    ---------------------------------------------------------------------------
    p_tg : process(clk)
        variable v_hv : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then tgs <= f_gseed or x"00000001"; end if;
            if tg_m = '1' then
                if tg_b(0) = '1' then tg_acc <= tg_acc + tg_a; end if;
                tg_a <= shift_left(tg_a, 1); tg_b <= '0' & tg_b(7 downto 1);
                if tg_n = 1 then tg_m <= '0'; end if;
                tg_n <= tg_n - 1;
            else
                case to_integer(tg_st) is
                    when 0 =>
                        if tg_go = '1' or (s_zone = '1' and s_zc = 800) then tg_st <= to_unsigned(1, 4); end if;
                    when 1 =>
                        tgs <= f_xs(tgs); tg_st <= to_unsigned(2, 4);
                    when 2 =>
                        tg_r1 <= tgs(15 downto 0); tgs <= f_xs(tgs); tg_st <= to_unsigned(3, 4);
                    when 3 =>
                        tg_r2 <= tgs;
                        tg_acc <= (others => '0'); tg_a <= resize(f_hmax, 22); tg_b <= resize(tg_r1(4 downto 0), 8);
                        tg_n <= to_unsigned(5, 4); tg_m <= '1'; tg_st <= to_unsigned(4, 4);
                    when 4 =>
                        v_hv := resize(tg_acc(10 downto 5), 7) + 1;
                        tg_acc <= (others => '0'); tg_a <= resize(t_hs, 22); tg_b <= resize(v_hv, 8);
                        tg_n <= to_unsigned(6, 4); tg_m <= '1'; tg_st <= to_unsigned(5, 4);
                    when 5 =>
                        tg_lines <= tg_acc(14 downto 8);
                        tg_acc <= (others => '0'); tg_a <= resize(f_dym, 22); tg_b <= tg_r2(7 downto 0);
                        tg_n <= to_unsigned(8, 4); tg_m <= '1'; tg_st <= to_unsigned(6, 4);
                    when 6 =>
                        tg_dy <= shift_left(signed(resize(tg_acc, 24)) - signed(resize(shift_left(resize(f_dym, 16), 7), 24)), 5);
                        tg_acc <= (others => '0'); tg_a <= resize(f_dxm, 22); tg_b <= tg_r2(15 downto 8);
                        tg_n <= to_unsigned(8, 4); tg_m <= '1'; tg_st <= to_unsigned(7, 4);
                    when 7 =>
                        tg_dx <= shift_left(signed(resize(tg_acc, 24)) - signed(resize(shift_left(resize(f_dxm, 20), 7), 24)), 1);
                        tg_acc <= (others => '0'); tg_a <= resize(f_dxc, 22); tg_b <= tg_r2(15 downto 8);
                        tg_n <= to_unsigned(8, 4); tg_m <= '1'; tg_st <= to_unsigned(8, 4);
                    when 8 =>
                        tg_cpo <= shift_left(signed(resize(tg_acc, 24)) - signed(resize(shift_left(resize(f_dxc, 19), 7), 24)), 1);
                        tg_hc <= to_unsigned(2, 4) + resize(tg_r2(31 downto 29), 4);
                        if tg_r2(23 downto 16) < 77 then tg_hf <= '1'; else tg_hf <= '0'; end if;
                        tg_act <= tg_r1(15 downto 8);
                        tg_st <= (others => '0');
                    when others =>
                        tg_st <= (others => '0');
                end case;
            end if;
        end if;
    end process p_tg;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): column and sample-and-hold cell accumulators
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_cn : signed(23 downto 0);
        variable v_nx : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            -- b_u = store column + 0.5 (Q16); b_t = b_u modulo 128 columns, folded back from 64,
            -- so the store reads mirrored about columns -0.5 and 63.5 (seamless edges)
            if g_start = '1' then v_nx := l_u0;
            elsif g_avid = '1' then v_nx := b_u + signed(resize(t_rx, 24));
            else v_nx := b_u; end if;
            if v_nx(22) = '1' then b_t <= not unsigned(v_nx(21 downto 0)); else b_t <= unsigned(v_nx(21 downto 0)); end if;
            if g_start = '1' then
                b_u <= l_u0; b_cp <= l_cp0; b_first <= '1';
                b_cst <= '1'; b_cell <= unsigned(l_cp0(23 downto 16));
            elsif g_avid = '1' then
                b_u <= b_u + signed(resize(t_rx, 24));
                v_cn := b_cp + signed(resize(t_cc, 24));
                b_cp <= v_cn; b_first <= '0';
                if v_cn(23 downto 16) /= b_cp(23 downto 16) then
                    b_cst <= '1'; b_cell <= unsigned(v_cn(23 downto 16));
                else
                    b_cst <= '0';
                end if;
            end if;
        end if;
    end process p_beam;


    ---------------------------------------------------------------------------
    -- Video write side: the camera's top part (store rows 0..127) into the
    -- store, one 4-bit phase per column (IIR-smoothed luma x gain), and a
    -- 2-bit hue class per column of every other row
    ---------------------------------------------------------------------------
    p_vw : process(clk)
        variable v_by : signed(14 downto 0);
        variable v_y  : signed(14 downto 0);
        variable v_uw : unsigned(21 downto 0);
        variable v_col : unsigned(5 downto 0);
        variable v_yi : signed(10 downto 0);
        variable v_cr, v_cb : unsigned(9 downto 0);
        variable v_hc : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            w_ev <= '0';
            -- hue class of this pixel (U = Cr, V = Cb): red, magenta, green, else blue
            v_cr := unsigned(data_in.u); v_cb := unsigned(data_in.v);
            if v_cr > 528 then
                if v_cb > 528 then v_hc := "10"; else v_hc := "01"; end if;
            elsif v_cr < 496 and v_cb < 496 then v_hc := "11";
            else v_hc := "00"; end if;
            if data_in.avid = '1' then
                if s_avq = '0' then
                    v_by := to_signed(64 * 16, 15); v_uw := (others => '0');
                else
                    v_by := signed('0' & w_yf); v_uw := w_uw;
                end if;
                v_y := signed(shift_left(resize(unsigned(data_in.y), 15), 4)) - v_by;
                if t_hd = '1' then v_y := v_by + shift_right(v_y, 4); else v_y := v_by + shift_right(v_y, 3); end if;
                w_yf <= unsigned(v_y(13 downto 0));
                w_hb <= v_hc;
                v_col := v_uw(21 downto 16);
                v_uw := v_uw + resize(t_cx, 22);
                w_uw <= v_uw;
                if v_uw(21 downto 16) /= v_col then
                    w_ev <= '1'; w_col <= v_col;
                    w_y0 <= unsigned(v_y(13 downto 4)); w_h0 <= v_hc;
                end if;
            elsif s_avq = '1' then
                -- avid fall: the last column
                w_ev <= '1'; w_col <= w_uw(21 downto 16);
                w_y0 <= w_yf(13 downto 4); w_h0 <= w_hb;
            end if;
            -- E0b: luma offset
            w_evb <= w_ev; w_colb <= w_col;
            v_yi := signed(resize(w_y0, 11)) - 64;
            if v_yi < 0 then w_yn <= (others => '0'); else w_yn <= unsigned(v_yi(9 downto 0)); end if;
            w_hc2 <= w_h0;
            w_hc3 <= w_hc2;
            -- E1: phase product
            w_ev2 <= w_evb; w_col2 <= w_colb;
            w_pr <= w_yn * f_gw;
            -- E2: write strobes (one bank)
            w_d <= std_logic_vector(w_pr(15 downto 12));
            w_a <= l_wrow(6 downto 1) & w_col2(5 downto 1);
            w_ha <= l_wrow(6 downto 1) & w_col2;
            w_hd <= w_hc3;
            for b in 0 to 3 loop
                if w_ev2 = '1' and l_wok = '1' and bidx(l_wrow(0), w_col2(0)) = b then w_we(b) <= '1'; else w_we(b) <= '0'; end if;
            end loop;
            w_hwe <= w_ev2 and l_wok;
        end if;
    end process p_vw;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ut : unsigned(21 downto 0);
        variable v_ux : unsigned(4 downto 0);
        variable v_ca, v_cb, v_mc : unsigned(4 downto 0);
        variable v_cpar : std_logic;
        variable v_cn4 : unsigned(3 downto 0);
        variable v_c  : unsigned(5 downto 0);
        type t_q4 is array (0 to 3) of unsigned(5 downto 0);
        variable v_b  : t_q4;
        variable v_hu : std_logic_vector(1 downto 0);
        variable v_hh : unsigned(1 downto 0);
        variable v_d0, v_d1, v_dh : signed(5 downto 0);
        variable v_a0, v_a1, v_ah : unsigned(6 downto 0);
        variable v_m  : unsigned(6 downto 0);
        variable v_w  : unsigned(3 downto 0);
        variable v_dd : signed(10 downto 0);
        variable v_ad : unsigned(10 downto 0);
        variable v_ha : unsigned(15 downto 0);
        variable v_na : signed(9 downto 0);
        variable v_hz : unsigned(1 downto 0);
        variable v_pg : unsigned(7 downto 0);
        variable v_h  : unsigned(1 downto 0);
        variable v_hv : unsigned(9 downto 0);
        variable v_sel : unsigned(3 downto 0);
        variable v_sy : unsigned(13 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ks : unsigned(15 downto 0);
        variable v_y8 : unsigned(7 downto 0);
        variable v_u8, v_v8 : unsigned(7 downto 0);
        variable v_sk : unsigned(5 downto 0);
        variable v_cu, v_cv : signed(13 downto 0);
        variable v_du11, v_dv11 : signed(10 downto 0);
        variable v_tu, v_tv : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: column, fraction, bank and hue addresses; fragment hashes (this cell, next cell)
            -- u = t - 0.5 column, clamped to [0, 63) as in v0.5; the bank columns (c + 1) / 2 and
            -- c / 2 straight from t's bits (m = t / 2 columns), so no decrement on the address path
            v_ut := b_t;
            v_mc := v_ut(21 downto 17);
            if v_ut(21 downto 15) = "0000000" then
                v_c := (others => '0'); v_ux := (others => '0'); v_ca := (others => '0'); v_cb := (others => '0'); v_cpar := '0';
            elsif v_ut(21 downto 15) = "1111111" then
                v_c := to_unsigned(62, 6); v_ux := (others => '1'); v_ca := to_unsigned(31, 5); v_cb := to_unsigned(31, 5); v_cpar := '0';
            else
                if v_ut(15) = '1' then v_c := v_ut(21 downto 16); else v_c := v_ut(21 downto 16) - 1; end if;
                v_ux := (not v_ut(15)) & v_ut(14 downto 11);
                if v_ut(16) = '1' and v_ut(15) = '1' then v_ca := v_mc + 1; else v_ca := v_mc; end if;
                if v_ut(16) = '0' and v_ut(15) = '0' then v_cb := v_mc - 1; else v_cb := v_mc; end if;
                v_cpar := not (v_ut(16) xor v_ut(15));
            end if;
            a_b0 <= l_ra0 & v_ca;
            a_b1 <= l_ra0 & v_cb;
            a_b2 <= l_ra1 & v_ca;
            a_b3 <= l_ra1 & v_cb;
            a_hue <= l_hrow & v_c;
            p1_cp <= v_cpar; p1_fx <= v_ux;
            v_ks := f_nseed xor x"7777";
            ha_1 <= f_h1((b_cell & l_grp) xor v_ks);
            p_cst(1) <= b_cst;
            p_ast(1) <= b_cst and (b_first or not b_cell(0));
            p1_c <= v_c; p1_first <= b_first;

            -- 2: (store reads) hashes; the tear's smear: take a new colour once the column has
            -- changed l_hc times (either way) since the last take
            p2_cp <= p1_cp; p2_fx <= p1_fx;
            p2_cl <= p1_c;
            if p1_c /= p2_cl then v_cn4 := p2_cn + 1; else v_cn4 := p2_cn; end if;
            if p1_first = '1' or v_cn4 >= l_hc then
                p_tk(2) <= '1'; p2_cn <= (others => '0');
            else
                p_tk(2) <= '0'; p2_cn <= v_cn4;
            end if;
            ha_2 <= f_h2(ha_1);
            p_cst(2) <= p_cst(1);

            -- 3: fabric copies, Osc / Video select, corners; dark-band hue held from the cell start
            if f_osc = '1' then
                v_b(0) := unsigned(std_logic_vector'(r_s0h & r_s0l)); v_b(1) := unsigned(std_logic_vector'(r_s1h & r_s1l));
                v_b(2) := unsigned(std_logic_vector'(r_s2h & r_s2l)); v_b(3) := unsigned(std_logic_vector'(r_s3h & r_s3l));
                v_hu := r_shue;
            else
                v_b(0) := unsigned(std_logic_vector'(r_v0 & "00")); v_b(1) := unsigned(std_logic_vector'(r_v1 & "00"));
                v_b(2) := unsigned(std_logic_vector'(r_v2 & "00")); v_b(3) := unsigned(std_logic_vector'(r_v3 & "00"));
                v_hu := r_vh;
            end if;
            p3_q00 <= v_b(bidx(l_r0p, p2_cp));
            p3_q01 <= v_b(bidx(l_r0p, not p2_cp));
            p3_q10 <= v_b(bidx(not l_r0p, p2_cp));
            p3_q11 <= v_b(bidx(not l_r0p, not p2_cp));
            -- K6: the photo scene's phase at this corner keys the reveal (>= level: flat ground)
            case bidx(l_r0p, p2_cp) is
                when 0 => v_sk := unsigned(std_logic_vector'(r_s0h & r_s0l));
                when 1 => v_sk := unsigned(std_logic_vector'(r_s1h & r_s1l));
                when 2 => v_sk := unsigned(std_logic_vector'(r_s2h & r_s2l));
                when others => v_sk := unsigned(std_logic_vector'(r_s3h & r_s3l));
            end case;
            p3_sk <= v_sk;
            if p_cst(2) = '1' then v_hh := unsigned(v_hu); else v_hh := p3_hh; end if;
            p3_hh <= v_hh;
            p3_fx <= p2_fx;
            ha_3 <= f_h3(ha_2);

            -- 4: wrapped differences, skip flags, busyness; fragment values and hue class
            v_d0 := signed(p3_q10 - p3_q00); v_d1 := signed(p3_q11 - p3_q01); v_dh := signed(p3_q01 - p3_q00);
            v_a0 := unsigned(abs(resize(v_d0, 7))); v_a1 := unsigned(abs(resize(v_d1, 7))); v_ah := unsigned(abs(resize(v_dh, 7)));
            p4_d0 <= v_d0; p4_d1 <= v_d1;
            if v_a0 > 13 then p4_s0 <= '1'; else p4_s0 <= '0'; end if;
            if v_a1 > 13 then p4_s1 <= '1'; else p4_s1 <= '0'; end if;
            if v_ah > v_a0 then p4_mx <= v_ah(5 downto 0); else p4_mx <= v_a0(5 downto 0); end if;
            -- y-lerp bases: the top corner, or (step across a jump) the nearer corner
            if v_a0 > 13 and l_fy(4) = '1' then p4_q00 <= p3_q10; else p4_q00 <= p3_q00; end if;
            if v_a1 > 13 and l_fy(4) = '1' then p4_q01 <= p3_q11; else p4_q01 <= p3_q01; end if;
            p4_fx <= p3_fx;
            v_ha := f_h4(ha_3);
            v_na := signed(resize(v_ha(15 downto 8), 10)) - 128;
            p4_na <= v_na(8 downto 0);
            if v_ha(7 downto 0) < 115 then v_hz := "00";
            elsif v_ha(7 downto 0) < 184 then v_hz := "01";
            elsif v_ha(7 downto 0) < 230 then v_hz := "10";
            else v_hz := "11"; end if;
            p4_hz <= v_hz; p4_hm <= p3_hh;

            -- 5: y products, busy level (quarters), fragment slope
            if p4_s0 = '1' then p5_p0 <= (others => '0'); else p5_p0 <= v_mul_fy(p4_d0, l_fy); end if;
            if p4_s1 = '1' then p5_p1 <= (others => '0'); else p5_p1 <= v_mul_fy(p4_d1, l_fy); end if;
            p5_q00 <= p4_q00; p5_q01 <= p4_q01;
            p5_fx <= p4_fx;
            if p4_mx > 12 then v_m := resize(p4_mx - 12, 7); else v_m := (others => '0'); end if;
            if v_m(6 downto 1) > 4 then p5_b4 <= to_unsigned(4, 3); else p5_b4 <= v_m(3 downto 1); end if;
            -- dark-band hue: the hashed hue in busy cells (busy >= 2 quarters), else the map
            if p_kz(4) = '1' then p_hue(5) <= "000";
            elsif p4_mx >= 16 then p_hue(5) <= resize(p4_hz, 3);
            else p_hue(5) <= resize(p4_hm, 3); end if;
            p5_na <= p4_na;

            -- 6: y lerps (wrap-aware, step across > 13 levels); fragment value
            p6_a <= (p5_q00 & "00000") + unsigned(p5_p0);
            p6_b <= (p5_q01 & "00000") + unsigned(p5_p1);
            v_w := resize(p5_b4, 4) + resize(f_sp, 4);
            if v_w > 4 then p6_w4 <= to_unsigned(4, 3); else p6_w4 <= v_w(2 downto 0); end if;
            p6_fx <= p5_fx;
            p_n(6) <= p5_na;

            -- 7: x difference, skip, fragment amplitude = f_amp * w / 4
            v_dd := signed(p6_b - p6_a);
            p7_dd <= v_dd;
            v_ad := unsigned(abs(v_dd));
            if v_ad > 416 then
                p7_sx <= '1';
                if p6_fx(4) = '0' then p7_a <= p6_a; else p7_a <= p6_b; end if;
            else
                p7_sx <= '0'; p7_a <= p6_a;
            end if;
            p7_fx <= p6_fx;
            case to_integer(p6_w4) is
                when 0 => p7_amp <= (others => '0');
                when 1 => p7_amp <= resize(f_amp(8 downto 2), 10);
                when 2 => p7_amp <= resize(f_amp(8 downto 1), 10);
                when 3 => p7_amp <= resize(shift_right(resize(f_amp, 11) + shift_left(resize(f_amp, 11), 1), 2), 10);
                when others => p7_amp <= resize(f_amp, 10);
            end case;

            -- 8: x product
            if p7_sx = '1' then p8_px <= (others => '0'); else p8_px <= resize(p7_dd * signed(resize(p7_fx, 6)), 16); end if;
            p8_a <= p7_a;
            p8_ae <= p7_amp(9 downto 3);

            -- 9: phase of the field (11 bits, 2048 = one cycle); fragment product
            p9_phi <= p8_a + unsigned(resize(shift_right(p8_px, 5), 11));
            p9_np <= p_n(8) * signed(resize(p8_ae, 8));

            -- 10: colourizer gain (Osc), fragment offset
            p10_gp <= p9_phi * f_g;
            p10_phi <= p9_phi;
            p10_no <= p9_np(11 downto 4);

            -- 11: 8-bit phase, Negative
            if f_osc = '1' then v_pg := p10_gp(18 downto 11); else v_pg := p10_phi(10 downto 3); end if;
            if f_neg = '1' then v_pg := 0 - v_pg; end if;
            if p_kz(10) = '1' then              -- K6: the flat ground (a black input's phase 0)
                p11_pg <= (others => '0'); p11_no <= (others => '0');
            else
                p11_pg <= v_pg; p11_no <= p10_no;
            end if;

            -- 12: + fragments + colour-cycle offset
            p12_ph <= p11_pg + unsigned(p11_no) + f_rot(15 downto 8);

            -- 13: band (0 = the dark band): the block-RAM lookup r_bd = {dark, band - 1} of p12_ph
            -- (K2 Ink: the photo's thresholds 37, 74, 110, 147, 183, 220 at 50 %)

            -- 14: dark-band hue, palette index; Stairs All (the whole index held per cell pair); the tear's smear
            v_h := unsigned(r_pbf(1 downto 0));
            if r_bd(3) = '1' then v_sel := "10" & v_h; else v_sel := '0' & unsigned(r_bd(2 downto 0)); end if;
            if r_pbf(3) = '1' then p14_ah <= v_sel; elsif f_stair = '1' then v_sel := p14_ah; end if;
            if l_hon = '1' and r_pbf(4) = '0' then
                p14_sel <= p14_hsel;
            else
                p14_sel <= v_sel; p14_hsel <= v_sel;
            end if;

            -- 15: palette, with its colourway variants precomputed per entry (f_palx)
            p15_x <= f_palx(p14_sel);

            -- 16: 10-bit
            -- colourway (K6): chroma swaps / turns / complement, mono, luma negative, vivid, as a mux
            -- of the precomputed variants (offsets clamped to +-127, + 128, exactly as v0.5 computed them)
            if f_pl = 6 then v_y8 := unsigned(p15_x(55 downto 48)); else v_y8 := unsigned(p15_x(63 downto 56)); end if;
            case to_integer(f_pl) is
                when 1 =>      v_u8 := x"80";                          v_v8 := x"80";                           -- Ink
                when 2 =>      v_u8 := unsigned(p15_x(39 downto 32)); v_v8 := unsigned(p15_x(47 downto 40));   -- Swap
                when 3 =>      v_u8 := unsigned(p15_x(31 downto 24)); v_v8 := unsigned(p15_x(23 downto 16));   -- Comp
                when 4 =>      v_u8 := unsigned(p15_x(23 downto 16)); v_v8 := unsigned(p15_x(47 downto 40));   -- Turn
                when 5 =>      v_u8 := unsigned(p15_x(39 downto 32)); v_v8 := unsigned(p15_x(31 downto 24));   -- Twist
                when 7 =>      v_u8 := unsigned(p15_x(15 downto 8));  v_v8 := unsigned(p15_x(7 downto 0));     -- Vivid
                when others => v_u8 := unsigned(p15_x(47 downto 40)); v_v8 := unsigned(p15_x(39 downto 32));   -- Rorschach, Night
            end case;
            -- chroma neutral off the generated avid, so the 8-sums below resync in every blanking interval
            p16_y <= v_y8 & "00";
            if s_gav_sr(14) = '1' then
                p16_u <= v_u8 & "00";
                p16_v <= v_v8 & "00";
            else
                p16_u <= to_unsigned(512, 10); p16_v <= to_unsigned(512, 10);
            end if;
            s_bxh <= s_bxh(6 downto 0) & s_gav_sr(14);

            -- 17..22: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums; luma [1 3 4 4 3 1]/16 = box4 smear * [1 2 1]).  The running
            -- 8-sums snap back to neutral once 8 blank samples are in, so a glitched sample
            -- can't offset the chroma for good.
            if s_bxh = "00000000" then
                s_boxu <= to_unsigned(4096, 14); s_boxv <= to_unsigned(4096, 14);
            else
                s_boxu <= s_boxu + p16_u - (unsigned(r_cbf(15 downto 8)) & "00");
                s_boxv <= s_boxv + p16_v - (unsigned(r_cbf(7 downto 0)) & "00");
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v22 <= v_sc(13 downto 4);
            s_y17 <= p16_y; s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19;
            s_y21 <= s_y20; s_y22x <= s_y21;
            s_pa <= resize(s_y17, 11) + resize(s_y22x, 11);
            s_pb <= resize(s_y18, 11) + resize(s_y21, 11);
            s_pc <= resize(s_y19, 11) + resize(s_y20, 11);
            v_sy := resize(s_pa, 14) + resize(s_pb, 14) + shift_left(resize(s_pb, 14), 1) + shift_left(resize(s_pc, 14), 2);
            s_yf22 <= v_sy(13 downto 4);
            -- 23: luma toward black (white 940 -> 777); blanking gate
            if s_yf22 > 64 then v_dl := s_yf22 - 64; else v_dl := (others => '0'); end if;
            -- K6 above 50 %: chroma 512 + d (1 + n/8), d = c - 512: d n here (two adds), the rest in 24
            v_du11 := signed(resize(s_u22, 11)) - 512; v_dv11 := signed(resize(s_v22, 11)) - 512;
            v_tu := (others => '0'); v_tv := (others => '0');
            if c_bst(2) = '1' then v_tu := shift_left(resize(v_du11, 14), 2); v_tv := shift_left(resize(v_dv11, 14), 2); end if;
            if c_bst(1) = '1' then v_tu := v_tu + shift_left(resize(v_du11, 14), 1); v_tv := v_tv + shift_left(resize(v_dv11, 14), 1); end if;
            if c_bst(0) = '1' then v_tu := v_tu + resize(v_du11, 14); v_tv := v_tv + resize(v_dv11, 14); end if;
            if s_gav_sr(21) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
                s_tu23 <= v_tu; s_tv23 <= v_tv;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
                s_tu23 <= (others => '0'); s_tv23 <= (others => '0');
            end if;
            -- 24: + d n / 8, clamped to 512 +- 508 (the palette's own limit; a no-op at n = 0)
            v_cu := signed(resize(s_u23, 14)) + shift_right(s_tu23, 3);
            v_cv := signed(resize(s_v23, 14)) + shift_right(s_tv23, 3);
            if v_cu > 1020 then v_cu := to_signed(1020, 14); elsif v_cu < 4 then v_cu := to_signed(4, 14); end if;
            if v_cv > 1020 then v_cv := to_signed(1020, 14); elsif v_cv < 4 then v_cv := to_signed(4, 14); end if;
            s_y24 <= s_y23; s_u24 <= unsigned(v_cu(9 downto 0)); s_v24 <= unsigned(v_cv(9 downto 0));

            -- delays

            if resize(p3_sk, 7) >= c_klv then p_kz(4) <= '1'; else p_kz(4) <= '0'; end if;   -- K6 reveal, stage 4
            for i in 5 to 10 loop p_kz(i) <= p_kz(i - 1); end loop;
            for i in 2 to 5 loop p_ast(i) <= p_ast(i - 1); end loop;
            for i in 3 to 5 loop p_tk(i) <= p_tk(i - 1); end loop;
            for i in 7 to 8 loop p_n(i) <= p_n(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture rorschach;
