-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lotus v0.5.0 -- Heirloom #33, rebuilt from the photo WIN_20230515_23_23_36.
--
-- The photo: two copies (period 943 px, 2.04 across) of an orange/green contour
-- "eye" -- an olive core diamond in nested green / orange rings, a pointed ring
-- tip to the right, an X where the rings above the eye cross -- over a yellow /
-- green band of zigzag contours, with a pink / white soft cushion ("lotus bowl")
-- keyed into the bright green bottom of each copy.  The left screen edge holds a
-- smooth yellow-green gradient up to a stepped staircase of line starts, each
-- start marked by a red / pink / white flash, and the zone between the eyes is cut
-- into vertical orange / green stripes and frozen blocks by a stuttering sync.
--
-- Reading (measured): the main field is a horizontal triangle oscillator through a
-- per-line VCA plus a per-line offset, F = a(y) T(u) + b(y) (per-row fits rms 6-15
-- in the top), plus a low-rank correction G_k(y) h_k(u) (the eye's tight left side; rank 3
-- fitted, the first two evaluated), an
-- 8x triangle (zigzag band, seam stripes) and a seam-zone gate; the colourizer is
-- R = R0 + RA cos 2 pi F, G = G0 + GA cos 2 pi (F + PG) with per-line parameters, B
-- = a soft key on the cushion (R rises 15 px before B).  lotus.py carries the fitted
-- tables (256 entries over 1080 virtual lines, interpolated per line).
--
-- The program: a per-line engine (one multiplier) interpolates the table, folds the
-- Rings gain into the coefficients, then fills two ping-pong block-RAM tables for
-- the next line: the field over one period (64 entries of F and slope, flow folded
-- in) and the colour palette (128 phases of R, G).  The pixel path reads F at the stuttered / torn / mirrored phase,
-- interpolates, adds the video etch, looks up the palette, keys the cushion and
-- composes the line-start hold gradient and flash.
--
--   K1  Lobes    copies across: 0.7 .. 5.8 (centre = the photo's 2.04)
--   K2  Stutter  the sync stutter: hold staircase, frozen blocks, sweeps, tears (0 = clean)
--   K3  Bloom    the cushions' size (0 = none)
--   K4  Speed    bipolar contour flow (centre +-8 = still)
--   K5  Ripple   the 8x zigzag in the middle band and the seam stripes
--   K6  Etch     how far the video (or Osc) bends the contours
--   S7  Palette  Photo / Night (red and blue exchanged)
--   S8  Mirror   the eyes face the other way
--   S9  Contour  Soft (the photo) / Hard two-level bands
--   S10 Fold     the top half mirrored into the bottom: four eyes, no cushions
--   S11 Source   Video / Osc
--   P12 RINGS    contour density 0.25x .. 3x about the eye core (534 = the photo)
--
-- Virtual 1080-line geometry (720p and 1080 identical; SD shows the centre 1440 px).
-- Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1],
-- luma to ~777) before the blanking gate.  Latency 20 clocks in every mode.
-- Generated colours in the hardware convention (U carries Cr, V carries Cb).
-- Build: ROUTER1 (router2 misses, 71 MHz); all seed 1 except HD Dual seed 2.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lotus_pkg.all;
use work.all;

architecture lotus of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_UPK : unsigned(19 downto 0) := to_unsigned(527068, 20);   -- triangle peak (474 / 943)
    constant C_UK  : unsigned(19 downto 0) := to_unsigned(533740, 20);   -- cushion axis (480 / 943)
    constant C_UC  : unsigned(19 downto 0) := to_unsigned(1009657, 20);  -- phase at the screen centre
    constant C_FR  : signed(20 downto 0) := to_signed(C_FREF, 21);       -- F at the eye core, Q12

    -- per-line table: words 0..7 and 8..11 (256 entries each), see lotus.py
    type t_lta is array(0 to 2047) of std_logic_vector(15 downto 0);
    type t_ltb is array(0 to 1023) of std_logic_vector(15 downto 0);
    function f_lta return t_lta is
        variable r : t_lta;
    begin
        for j in 0 to 255 loop
            r(j) := C_LT0(j); r(256 + j) := C_LT1(j); r(512 + j) := C_LT2(j); r(768 + j) := C_LT3(j);
            r(1024 + j) := C_LT4(j); r(1280 + j) := C_LT5(j); r(1536 + j) := C_LT6(j); r(1792 + j) := C_LT7(j);
        end loop;
        return r;
    end function;
    function f_ltb return t_ltb is
        variable r : t_ltb;
    begin
        for j in 0 to 255 loop
            r(j) := C_LT8(j); r(256 + j) := C_LT9(j); r(512 + j) := C_LT10(j); r(768 + j) := C_LT11(j);
        end loop;
        return r;
    end function;
    constant C_LTA : t_lta := f_lta;
    constant C_LTB : t_ltb := f_ltb;
    -- horizontal basis, padded to a block-RAM depth
    type t_hb2 is array(0 to 255) of std_logic_vector(47 downto 0);
    function f_hb2 return t_hb2 is
        variable r : t_hb2 := (others => (others => '0'));
    begin
        for j in 0 to 63 loop r(j) := C_HB(j); end loop;
        return r;
    end function;
    constant C_HB2 : t_hb2 := f_hb2;

    -- K1 Lobes: 256 * 2^((16 i - 512) / 342)
    type t_m8 is array(0 to 63) of integer range 0 to 1023;
    constant C_M8 : t_m8 := (91, 94, 97, 100, 103, 107, 110, 114, 118, 121, 125, 130, 134, 138, 143, 148, 152,
        157, 163, 168, 173, 179, 185, 191, 198, 204, 211, 218, 225, 232, 240, 248, 256, 264, 273, 282, 291, 301,
        311, 321, 332, 343, 354, 366, 378, 390, 403, 416, 430, 444, 459, 474, 490, 506, 522, 540, 557, 576, 595,
        614, 635, 656, 677, 700);

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_tri8(v : unsigned(15 downto 0)) return unsigned is
    begin
        if v(15) = '1' then return not v(14 downto 7); else return v(14 downto 7); end if;
    end function;
    function f_clamp16(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 5);
        elsif v > 16 then return to_unsigned(16, 5);
        else return unsigned(v(4 downto 0)); end if;
    end function;

    -- ROM ports (registered address and data)
    signal r_ta_a : unsigned(10 downto 0) := (others => '0');
    signal r_tb_a : unsigned(9 downto 0) := (others => '0');
    signal r_ta_q, r_tb_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_hb_a : unsigned(7 downto 0) := (others => '0');
    signal r_hb_q : std_logic_vector(47 downto 0) := (others => '0');
    signal r_c_a  : unsigned(7 downto 0) := (others => '0');
    signal r_c_q  : std_logic_vector(8 downto 0) := (others => '0');

    -- per-line ping-pong tables: field {F 16 | slope / 16 12 | 0} (2 x 64), palette {R 8 | G 8} (2 x 128)
    type t_fram is array(0 to 127) of std_logic_vector(31 downto 0);
    type t_pram is array(0 to 255) of std_logic_vector(15 downto 0);
    signal fr_ram : t_fram := (others => (others => '0'));
    signal pl_ram : t_pram := (others => (others => '0'));
    -- rings: chroma box window {U V}, syncs
    type t_uring is array(0 to 255) of std_logic_vector(19 downto 0);
    type t_sring is array(0 to 255) of std_logic_vector(2 downto 0);
    signal ru_ram : t_uring := (others => "1000000000" & "1000000000");
    signal rs_ram : t_sring := (others => "111");
    attribute no_rw_check : boolean;
    attribute no_rw_check of fr_ram : signal is true;
    attribute no_rw_check of pl_ram : signal is true;
    attribute no_rw_check of ru_ram : signal is true;
    attribute no_rw_check of rs_ram : signal is true;
    signal fr_we, pl_we : std_logic := '0';
    signal fr_wa, fr_ra : unsigned(6 downto 0) := (others => '0');
    signal pl_wa, pl_ra : unsigned(7 downto 0) := (others => '0');
    signal fr_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal pl_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal fr_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal pl_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal s_wp, ru_ra, rs_ra : unsigned(7 downto 0) := (others => '0');
    signal ru_q, ru_q2 : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal rs_q, rs_q2 : std_logic_vector(2 downto 0) := "111";

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_k254 : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_fc   : unsigned(11 downto 0) := (others => '0');

    -- glided knobs (Q3): K1 K2 K3 K5 K6 P12
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(534 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    -- switches (field-latched)
    signal s_night, s_mir, s_hard, s_fold, s_src : std_logic := '0';
    -- speed
    signal s_vneg, s_vstill : std_logic := '1';
    signal s_vm : unsigned(9 downto 0) := (others => '0');

    -- per-field serial products
    signal f_acc : unsigned(27 downto 0) := (others => '0');
    signal f_a   : unsigned(27 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_m8  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_dub : unsigned(11 downto 0) := to_unsigned(1112, 12);
    signal f_du  : unsigned(13 downto 0) := to_unsigned(1112, 14);      -- phase per pixel (2^20 per period)
    signal f_duv : unsigned(11 downto 0) := to_unsigned(1112, 12);      -- phase per virtual px
    signal f_u0  : unsigned(19 downto 0) := (others => '0');
    signal f_p2  : unsigned(15 downto 0) := (others => '0');
    signal f_gr  : unsigned(9 downto 0) := to_unsigned(256, 10);        -- Rings gain, Q8
    signal f_e8g : unsigned(7 downto 0) := to_unsigned(75, 8);          -- etch x gain
    signal f_spd : unsigned(10 downto 0) := (others => '0');
    signal f_hh  : unsigned(8 downto 0) := to_unsigned(203, 9);
    signal f_flow : unsigned(11 downto 0) := (others => '0');
    signal f_cfr : signed(20 downto 0) := (others => '0');              -- Fref + flow
    signal f_roll : unsigned(7 downto 0) := (others => '0');
    signal f_seed : unsigned(15 downto 0) := x"ACE1";
    signal f_s8, f_b8, f_r8 : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal f_e8 : unsigned(7 downto 0) := to_unsigned(75, 8);

    -- Osc source
    signal s_ox, s_oy, s_oxl : unsigned(15 downto 0) := (others => '0');
    signal s_oys : unsigned(11 downto 0) := to_unsigned(320, 12);

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- per-line engine: phase A (table interpolation, gain), F row, palette, phase C (scalars)
    signal e_st   : unsigned(5 downto 0) := (others => '0');
    signal e_yv   : unsigned(19 downto 0) := (others => '0');          -- virtual line, Q8
    signal e_yf   : unsigned(10 downto 0) := (others => '0');
    signal e_tq   : unsigned(18 downto 0) := (others => '0');
    signal e_j    : unsigned(7 downto 0) := (others => '0');
    signal e_f    : unsigned(7 downto 0) := (others => '0');
    signal e_j1   : unsigned(7 downto 0) := (others => '0');
    signal e_hc, e_hp : signed(16 downto 0) := (others => '0');        -- hi fields of the word at j + 1 / j
    signal e_lc, e_lp : unsigned(7 downto 0) := (others => '0');       -- lo fields
    signal e_md   : unsigned(10 downto 0) := (others => '0');          -- (yf + roll) mod 162
    signal e_typ  : unsigned(1 downto 0) := (others => '0');
    signal e_wb, r_bank : std_logic := '0';
    signal e_zch  : std_logic := '0';
    signal ia_dh  : signed(16 downto 0) := (others => '0');
    signal ia_dl  : signed(9 downto 0) := (others => '0');
    -- the multiplier: shift-add over three pipeline stages (product of operands set in state s
    -- readable in state s + 4)
    signal m_a, m_a1, m_a2 : signed(12 downto 0) := (others => '0');
    signal m_b, m_b1, m_b2 : signed(9 downto 0) := (others => '0');
    signal m_s1, m_s2 : signed(22 downto 0) := (others => '0');
    signal m_p    : signed(22 downto 0) := (others => '0');
    -- interpolated coefficients of the line being prepared
    signal c_b : signed(16 downto 0) := (others => '0');
    signal c_a : unsigned(8 downto 0) := (others => '0');
    signal c_ga, c_z1r, c_z2r : unsigned(6 downto 0) := (others => '0');
    signal c_g1, c_g2, c_z3, c_r0 : signed(9 downto 0) := (others => '0');
    signal c_gb : unsigned(4 downto 0) := (others => '0');
    signal c_ra : signed(8 downto 0) := (others => '0');
    signal c_g0, c_pg, c_rs, c_gs, c_re, c_ge : unsigned(7 downto 0) := (others => '0');
    signal c_kw : unsigned(8 downto 0) := (others => '0');
    signal c_zs : unsigned(9 downto 0) := (others => '0');
    signal c_mid : std_logic := '0';
    signal e_bd   : signed(16 downto 0) := (others => '0');
    signal c_a2, c_g1s, c_g2s, c_z3s, c_zs2 : signed(12 downto 0) := (others => '0');
    signal e_kt   : signed(20 downto 0) := (others => '0');
    signal c_k    : signed(20 downto 0) := (others => '0');
    -- F row
    signal e_fon  : std_logic := '0';
    signal e_fe   : unsigned(6 downto 0) := (others => '0');
    signal e_fs   : unsigned(2 downto 0) := (others => '0');
    type t_u7a is array (natural range <>) of unsigned(6 downto 0);
    type t_u3a is array (natural range <>) of unsigned(2 downto 0);
    signal t_v : std_logic_vector(1 to 4) := (others => '0');
    signal t_e : t_u7a(1 to 4) := (others => (others => '0'));
    signal t_k : t_u3a(1 to 4) := (others => (others => '0'));
    signal e_acc  : signed(20 downto 0) := (others => '0');
    signal e_fp, e_fpp, e_fp0 : signed(15 downto 0) := (others => '0');
    signal e_fpv, e_last : std_logic := '0';
    signal e_fpe  : unsigned(6 downto 0) := (others => '0');
    -- palette
    signal e_pon  : std_logic := '0';
    signal e_pq   : unsigned(8 downto 0) := (others => '0');
    signal e_c    : signed(8 downto 0) := (others => '0');
    signal e_pr   : unsigned(7 downto 0) := (others => '0');
    -- phase C
    signal c_st   : unsigned(4 downto 0) := (others => '0');
    signal e_kick : std_logic := '0';
    signal e_kwp  : unsigned(9 downto 0) := (others => '0');
    signal e_sp, e_l1p, e_jp : unsigned(8 downto 0) := (others => '0');
    signal e_dyt, e_dyb, e_dy : signed(11 downto 0) := (others => '0');
    signal e_vb, e_vr : signed(12 downto 0) := (others => '0');        -- 2 dy + 6, 2 dy + 22
    signal e_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal e_tl   : unsigned(4 downto 0) := (others => '0');
    signal e_ton  : std_logic := '0';
    signal e_amp  : unsigned(5 downto 0) := (others => '0');
    signal e_amps : unsigned(6 downto 0) := (others => '0');
    signal e_dpx  : signed(9 downto 0) := (others => '0');
    signal e_new  : std_logic := '0';
    -- current-line scalars (written in phase C after the previous line has left the stages)
    signal l_kwu : unsigned(13 downto 0) := (others => '0');
    signal l_abv, l_arv : unsigned(4 downto 0) := (others => '0');
    signal l_act : std_logic := '0';
    signal l_xhx, l_2l1x : signed(12 downto 0) := (others => '0');     -- plateau / freeze lengths - xs
    signal l_s : unsigned(8 downto 0) := (others => '0');
    signal l_2s, l_2sm, l_2l1 : unsigned(10 downto 0) := (others => '0');
    signal l_jump : signed(20 downto 0) := (others => '0');
    signal l_dut : signed(20 downto 0) := (others => '0');
    signal l_gb : unsigned(4 downto 0) := (others => '0');
    signal l_rs, l_gs : unsigned(7 downto 0) := (others => '0');
    signal l_drx, l_dgx : signed(10 downto 0) := (others => '0');

    -- beam / stutter
    signal b_ut  : unsigned(19 downto 0) := (others => '0');
    signal b_e   : signed(20 downto 0) := (others => '0');
    signal b_xh  : unsigned(11 downto 0) := (others => '0');
    signal b_md  : unsigned(2 downto 0) := (others => '0');
    signal b_cnt : unsigned(11 downto 0) := (others => '0');
    signal b_ft, b_sc : std_logic := '0';                              -- freeze window / seam next pixel

    -- video tap
    signal s_vy1, s_vy2 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_lum : unsigned(8 downto 0) := (others => '0');

    -- pixel path
    signal p1_u  : unsigned(11 downto 0) := (others => '0');
    signal p1_ut : unsigned(13 downto 0) := (others => '0');
    signal p1_osc : unsigned(7 downto 0) := (others => '0');
    signal p2_fr, p3_fr, p4_fr : unsigned(2 downto 0) := (others => '0');
    signal p2_kd : signed(13 downto 0) := (others => '0');
    signal p3_ak : unsigned(13 downto 0) := (others => '0');
    signal p3_lum, p4_lum : unsigned(6 downto 0) := (others => '0');
    signal p4_f, p5_f : unsigned(15 downto 0) := (others => '0');
    signal p4_sl : signed(11 downto 0) := (others => '0');
    signal p4_dx : signed(14 downto 0) := (others => '0');
    signal p5_sp : signed(15 downto 0) := (others => '0');
    signal p4_et, p5_et : unsigned(10 downto 0) := (others => '0');
    signal p_e4  : unsigned(3 downto 0) := (others => '0');
    signal p5_abh, p5_arh : unsigned(4 downto 0) := (others => '0');
    type t_u5a is array (natural range <>) of unsigned(4 downto 0);
    signal p_ab, p_ar : t_u5a(6 to 8) := (others => (others => '0'));
    signal p8_r, p8_g : unsigned(7 downto 0) := (others => '0');
    signal p9_r, p9_g, p9_b : unsigned(7 downto 0) := (others => '0');
    signal p9_kp : unsigned(7 downto 0) := (others => '0');
    signal p9_bo : unsigned(4 downto 0) := (others => '0');
    signal p10_r, p10_g, p10_b : unsigned(7 downto 0) := (others => '0');
    -- hold gradient / flash, aligned to stage 9 (advanced on the stage-9 avid)
    signal h_xh  : unsigned(11 downto 0) := (others => '0');
    signal h_cnt : unsigned(6 downto 0) := (others => '0');
    signal h_ar, h_ag : signed(17 downto 0) := (others => '0');
    signal p11_y5r, p11_u7r, p11_v14b : unsigned(11 downto 0) := (others => '0');
    signal p11_y10g, p11_u6g, p11_v9g : unsigned(11 downto 0) := (others => '0');
    signal p11_y2b, p11_v5r : unsigned(10 downto 0) := (others => '0');
    signal p11_ub : unsigned(7 downto 0) := (others => '0');
    signal p12_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p12_u, p12_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y13, s_y14, s_y15, s_y16, s_y17, s_y18x, s_yf18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs, the ping-pong line tables, the rings
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ta_q <= C_LTA(to_integer(r_ta_a));
            r_tb_q <= C_LTB(to_integer(r_tb_a));
            r_hb_q <= C_HB2(to_integer(r_hb_a));
            r_c_q  <= C_COS(to_integer(r_c_a));
        end if;
    end process p_rom;

    p_fram : process(clk)
    begin
        if rising_edge(clk) then
            if fr_we = '1' then fr_ram(to_integer(fr_wa)) <= fr_wd; end if;
            fr_q <= fr_ram(to_integer(fr_ra));
        end if;
    end process p_fram;

    p_pram : process(clk)
    begin
        if rising_edge(clk) then
            if pl_we = '1' then pl_ram(to_integer(pl_wa)) <= pl_wd; end if;
            pl_q <= pl_ram(to_integer(pl_ra));
        end if;
    end process p_pram;

    p_uring : process(clk)
    begin
        if rising_edge(clk) then
            ru_ram(to_integer(s_wp)) <= std_logic_vector(p12_u) & std_logic_vector(p12_v);
            ru_q <= ru_ram(to_integer(ru_ra));
            ru_q2 <= ru_q;
        end if;
    end process p_uring;

    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            rs_ram(to_integer(s_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            rs_q <= rs_ram(to_integer(rs_ra));
            rs_q2 <= rs_q;
        end if;
    end process p_sring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field products (one serial multiplier),
    -- ring pointers, the generated avid delay
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_sd : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);

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

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            g_wm1 <= s_w - 1;
            f_cfr <= C_FR + signed(resize(f_flow, 21));

            -- K4 Speed: bipolar, deadband 8
            v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;
            if v_s > -9 and v_s < 9 then s_vstill <= '1'; else s_vstill <= '0'; end if;
            s_vneg <= v_s(10);
            if v_s(10) = '1' then v_m := unsigned(not v_s(9 downto 0)) - 7; else v_m := unsigned(v_s(9 downto 0)) - 8; end if;
            s_vm <= v_m;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_xs    <= f_vstep_x(s_tid);
                s_night <= registers_in(6)(0);
                s_mir   <= registers_in(6)(1);
                s_hard  <= registers_in(6)(2);
                s_fold  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_fc    <= s_fc + 1;
                -- motion from last field's rates: + flow streams the rings out of the eyes
                if s_vstill = '0' then
                    if s_vneg = '1' then f_flow <= f_flow + resize(f_spd, 12);
                    else f_flow <= f_flow - resize(f_spd, 12); end if;
                end if;
                -- the stutter staircase crawls down one line every two fields
                if s_fc(0) = '1' then
                    if f_roll >= 161 then f_roll <= (others => '0'); else f_roll <= f_roll + 1; end if;
                end if;
                -- tear seed: new every 16 fields
                v_sd := shift_left(resize(s_fc(11 downto 4), 18), 10) - shift_left(resize(s_fc(11 downto 4), 18), 6)
                        - shift_left(resize(s_fc(11 downto 4), 18), 4) + resize(s_fc(11 downto 4), 18);
                f_seed <= x"ACE1" xor v_sd(15 downto 0);
                case to_integer(s_xs) is
                    when 4      => s_k254 <= to_unsigned(63, 7);
                    when 3      => s_k254 <= to_unsigned(84, 7);
                    when others => s_k254 <= to_unsigned(127, 7);
                end case;
                s_ox <= s_ox + 151; s_oy <= s_oy - 113;
            end if;
            s_oys <= resize(shift_right(resize(s_ys, 14) + shift_left(resize(s_ys, 14), 2), 2), 12);
            f_dub <= resize(shift_left(resize(s_xs, 12), 9) + shift_left(resize(s_xs, 12), 5)
                     + shift_left(resize(s_xs, 12), 3) + shift_left(resize(s_xs, 12), 2), 12);    -- 556 xs

            -- shared glide (1/8 of the gap per field, +-2 deadband): K1 K2 K3 K5 K6 P12
            if s_zone = '1' and s_zc >= 1 and s_zc <= 18 then
                case to_integer(s_zc) is
                    when 1  => s_gm <= s_g1; s_gd <= signed(resize(unsigned(registers_in(0)) & "000", 15)) - signed(resize(s_g1, 15));
                    when 4  => s_gm <= s_g2; s_gd <= signed(resize(unsigned(registers_in(1)) & "000", 15)) - signed(resize(s_g2, 15));
                    when 7  => s_gm <= s_g3; s_gd <= signed(resize(unsigned(registers_in(2)) & "000", 15)) - signed(resize(s_g3, 15));
                    when 10 => s_gm <= s_g5; s_gd <= signed(resize(unsigned(registers_in(4)) & "000", 15)) - signed(resize(s_g5, 15));
                    when 13 => s_gm <= s_g6; s_gd <= signed(resize(unsigned(registers_in(5)) & "000", 15)) - signed(resize(s_g6, 15));
                    when 16 => s_gm <= s_gp; s_gd <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(s_gp, 15));
                    when 2 | 5 | 8 | 11 | 14 | 17 =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                    when 3  => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 6  => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 9  => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 12 => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 15 => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                end case;
            end if;
            if s_zone = '1' and s_zc = 20 then
                f_m8 <= to_unsigned(C_M8(to_integer(s_g1(12 downto 7))), 10);
                f_s8 <= s_g2(12 downto 4);
                f_b8 <= s_g3(12 downto 4);
                f_r8 <= s_g5(12 downto 4);
                f_e8 <= s_g6(12 downto 5);
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result
            -- stable from load + 17 (stored at load + 18)
            if s_zone = '1' and (s_zc = 32 or s_zc = 56 or s_zc = 80 or
               s_zc = 176 or s_zc = 200 or s_zc = 224 or s_zc = 248 or s_zc = 272) then
                f_acc <= (others => '0');
                case to_integer(s_zc) is
                    when 32  => f_a <= resize(f_dub, 28); f_b <= resize(f_m8, 16);
                    when 56  => f_a <= to_unsigned(1112, 28); f_b <= resize(f_m8, 16);
                    when 80  => f_a <= resize(f_du, 28); f_b <= resize(s_w(11 downto 1), 16);
                    when 176 => f_a <= resize(s_gp(12 downto 3), 28); f_b <= resize(s_gp(12 downto 3), 16);
                    when 200 => f_a <= resize(f_p2, 28); f_b <= to_unsigned(704, 16);
                    when 224 => f_a <= resize(s_vm, 28); f_b <= resize(s_vm, 16);
                    when 248 => f_a <= to_unsigned(407, 28); f_b <= resize(f_b8, 16);
                    when others => f_a <= resize(f_e8, 28); f_b <= resize(f_gr, 16);
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(26 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 50  => f_du  <= f_acc(21 downto 8);
                    when 74  => f_duv <= f_acc(19 downto 8);
                    when 98  => f_u0  <= C_UC - f_acc(19 downto 0);
                    when 194 => f_p2  <= f_acc(19 downto 4);
                    when 218 => f_gr  <= to_unsigned(64, 10) + resize(f_acc(25 downto 16), 10);
                    when 242 => if s_vstill = '1' then f_spd <= (others => '0'); else f_spd <= f_acc(17 downto 7); end if;
                    when 266 => f_hh  <= f_acc(17 downto 9);
                    when 290 => f_e8g <= f_acc(15 downto 8);
                    when others => null;
                end case;
            end if;

            -- rings: write every clock; chroma window read 8 back, syncs C_LAT back
            s_wp <= s_wp + 1;
            ru_ra <= s_wp - 5;
            rs_ra <= s_wp - (C_LAT - 3);
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
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
    -- Per-line engine, one multiplier (13 x 10 shift-add over three pipeline
    -- stages: the product of operands set in state s is read in state s + 4).
    -- Kicked at the source avid rise of line n (line 0 in the vblank zone), it
    -- prepares line n+1:
    --   phase A (states 1..49): row, table position, the 12 words at entries j and
    --     j+1 interpolated (stage A writes v0 into the coefficient, the hi product
    --     of word w loaded in state 10 + 2w and the lo product in 11 + 2w are added
    --     4 states later), then the Rings gain folded into the coefficients and the
    --     constant term K;
    --   F row (64 entries x 5 products): F = K + (a' T + G1' h1 + G2' h2) / 16
    --     + (z' X + z3' h4) / 32, written with its slope into the next line's bank;
    --   palette (128 phases x 2 products): R = R0 + RA cos, G = G0 + GA cos(+ PG);
    --   phase C (kicked when line n has passed stage 12, ~20 clocks): the cushion,
    --     stutter, tear and hold scalars, written straight into l_*.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_yi  : unsigned(11 downto 0);
        variable v_tq  : unsigned(18 downto 0);
        variable v_w   : integer range 0 to 15;
        variable v_c   : std_logic_vector(15 downto 0);
        variable v_hc  : signed(16 downto 0);
        variable v_lc8 : unsigned(7 downto 0);
        variable v_dl  : signed(9 downto 0);
        variable v_rw, v_kh, v_kl, v_aw : integer range 0 to 15;
        variable v_rv, v_cv, v_sb, v_av : std_logic;
        variable v_rwu : unsigned(3 downto 0);
        variable v_ja  : unsigned(7 downto 0);
        variable v_x   : signed(13 downto 0);
        variable v_r   : unsigned(15 downto 0);
        variable v_bit : std_logic;
        variable v_st  : integer range 0 to 63;
        variable v_ph  : integer range 0 to 511;
        variable v_rr  : signed(11 downto 0);
        variable v_gg  : signed(11 downto 0);
        variable v_cs  : integer range 0 to 31;
        variable v_fp  : signed(20 downto 0);
        variable v_df  : signed(15 downto 0);
        variable v_acc : signed(22 downto 0);
        variable v_q   : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- kick
            if s_zone = '1' and s_zc = 300 then
                e_yv <= (others => '0'); e_st <= to_unsigned(1, 6); e_wb <= '0'; e_zch <= '1';
                e_lfsr <= f_seed; e_tl <= (others => '0');
            elsif data_in.avid = '1' and s_avq = '0' then
                e_yv <= e_yv + resize(s_ys, 20); e_st <= to_unsigned(1, 6);
                r_bank <= e_wb; e_wb <= not e_wb; e_zch <= '0';
            elsif e_st /= 0 then
                if e_st = 49 then e_st <= (others => '0'); else e_st <= e_st + 1; end if;
            end if;
            v_st := to_integer(e_st);

            -- row, table position, stutter block type
            if v_st = 1 then
                v_yi := e_yv(19 downto 8);
                if v_yi > 1079 then v_yi := to_unsigned(1079, 12); end if;
                if s_fold = '1' and v_yi >= 560 then
                    e_yf <= resize(to_unsigned(1119, 12) - v_yi, 11);
                else
                    e_yf <= v_yi(10 downto 0);
                end if;
            end if;
            if v_st = 2 then
                v_tq := shift_left(resize(e_yf, 19), 8) - shift_left(resize(e_yf, 19), 3)
                        - shift_left(resize(e_yf, 19), 2) - resize(e_yf, 19);              -- yf * 243
                e_tq <= v_tq;
                e_md <= e_yf + resize(f_roll, 11);
            end if;
            if v_st = 3 then
                if e_tq(18 downto 10) > 254 then e_j <= to_unsigned(254, 8); e_f <= to_unsigned(255, 8);
                else e_j <= e_tq(17 downto 10); e_f <= e_tq(9 downto 2); end if;
            end if;
            -- (yf + roll) mod 162: one conditional subtract per state
            if v_st >= 3 and v_st <= 11 then
                if e_md >= 162 then e_md <= e_md - 162; end if;
            end if;
            if v_st = 12 then
                if e_md < 85 then e_typ <= "00"; elsif e_md < 146 then e_typ <= "01"; else e_typ <= "10"; end if;
            end if;

            -- table reads: word w at entry j in state 5 + 2w, j + 1 in state 6 + 2w
            -- (all state decodes are constant case arms: no arithmetic on the state)
            if v_st = 4 then e_j1 <= e_j + 1; end if;
            v_rv := '1';
            case v_st is
                when 5 | 6 => v_rw := 0;
                when 7 | 8 => v_rw := 1;
                when 9 | 10 => v_rw := 2;
                when 11 | 12 => v_rw := 3;
                when 13 | 14 => v_rw := 4;
                when 15 | 16 => v_rw := 5;
                when 17 | 18 => v_rw := 6;
                when 19 | 20 => v_rw := 7;
                when 21 | 22 => v_rw := 8;
                when 23 | 24 => v_rw := 9;
                when 25 | 26 => v_rw := 10;
                when 27 | 28 => v_rw := 11;
                when others => v_rw := 0; v_rv := '0';
            end case;
            v_rwu := to_unsigned(v_rw, 4);
            if e_st(0) = '1' then v_ja := e_j; else v_ja := e_j1; end if;
            if v_rv = '1' then
                if v_rwu(3) = '0' then r_ta_a <= v_rwu(2 downto 0) & v_ja;
                else r_tb_a <= v_rwu(1 downto 0) & v_ja; end if;
            end if;
            -- fabric copy of the word read 2 states earlier, cut to its hi / lo fields
            v_cv := '1'; v_sb := '0';
            case v_st is
                when 7 | 8              => v_kh := 0; v_kl := 2;
                when 9 | 10             => v_kh := 1; v_kl := 1;
                when 11 | 12            => v_kh := 3; v_kl := 0;
                when 13 to 20           => v_kh := 3; v_kl := 2;
                when 21 | 22            => v_kh := 2; v_kl := 1;
                when 25 | 26            => v_kh := 1; v_kl := 1; v_sb := '1';
                when 23 | 24 | 27 to 30 => v_kh := 4; v_kl := 2; v_sb := '1';
                when others             => v_kh := 4; v_kl := 2; v_cv := '0';
            end case;
            if v_sb = '0' then v_c := r_ta_q; else v_c := r_tb_q; end if;
            case v_kh is
                when 0 =>      v_hc := resize(signed(v_c), 17);
                when 1 =>      v_hc := signed(resize(unsigned(v_c(15 downto 7)), 17));
                when 2 =>      v_hc := resize(signed(v_c(15 downto 7)), 17);
                when 3 =>      v_hc := resize(signed(v_c(15 downto 6)), 17);
                when others => v_hc := signed(resize(unsigned(v_c(15 downto 8)), 17));
            end case;
            case v_kl is
                when 0 =>      v_lc8 := resize(unsigned(v_c(5 downto 1)), 8);
                when 1 =>      v_lc8 := resize(unsigned(v_c(6 downto 0)), 8);
                when others => v_lc8 := unsigned(v_c(7 downto 0));
            end case;
            if v_cv = '1' then
                e_hc <= v_hc; e_hp <= e_hc; e_lc <= v_lc8; e_lp <= e_lc;
            end if;

            -- interpolation stage A (state 9 + 2w): v0 into the coefficients, d for the multiplier
            v_av := '1';
            case v_st is
                when 9 => v_aw := 0;
                when 11 => v_aw := 1;
                when 13 => v_aw := 2;
                when 15 => v_aw := 3;
                when 17 => v_aw := 4;
                when 19 => v_aw := 5;
                when 21 => v_aw := 6;
                when 23 => v_aw := 7;
                when 25 => v_aw := 8;
                when 27 => v_aw := 9;
                when 29 => v_aw := 10;
                when 31 => v_aw := 11;
                when others => v_aw := 15; v_av := '0';
            end case;
            if v_av = '1' then
                ia_dh <= e_hc - e_hp;
                v_dl := signed(resize(e_lc, 10)) - signed(resize(e_lp, 10));
                if v_aw = 8 then v_dl := resize(v_dl(7 downto 0), 10); end if;     -- the colour phase wraps
                ia_dl <= v_dl;
                case v_aw is
                    when 0  => c_b <= e_hp;
                    when 1  => c_a <= unsigned(e_hp(8 downto 0)); c_ga <= e_lp(6 downto 0);
                    when 2  => c_g1 <= e_hp(9 downto 0); c_gb <= e_lp(4 downto 0);
                    when 3  => c_g2 <= e_hp(9 downto 0);
                    when 5  => c_z3 <= e_hp(9 downto 0);
                    when 6  => c_r0 <= e_hp(9 downto 0);
                    when 7  => c_ra <= e_hp(8 downto 0); c_z1r <= e_lp(6 downto 0);
                    when 8  => c_g0 <= unsigned(e_hp(7 downto 0)); c_pg <= e_lp;
                    when 9  => c_kw <= unsigned(e_hp(8 downto 0)); c_z2r <= e_lp(6 downto 0);
                    when 10 => c_rs <= unsigned(e_hp(7 downto 0)); c_gs <= e_lp;
                    when 11 => c_re <= unsigned(e_hp(7 downto 0)); c_ge <= e_lp;
                    when others => null;
                end case;
            end if;

            -- F-row and palette loop counters
            if v_st = 48 then
                e_fon <= '1'; e_fe <= (others => '0'); e_fs <= (others => '0');
            elsif e_fon = '1' then
                if e_fe = 65 and e_fs = 1 then
                    e_fon <= '0'; e_pon <= '1'; e_pq <= (others => '0');
                elsif e_fs = 4 then
                    e_fs <= (others => '0'); e_fe <= e_fe + 1;
                else
                    e_fs <= e_fs + 1;
                end if;
            elsif e_pon = '1' then
                if e_pq = 263 then e_pon <= '0'; else e_pq <= e_pq + 1; end if;
            end if;
            v_ph := to_integer(e_pq);
            -- basis ROM: entry 0 in state 47, entry e + 1 at slot 3 of entry e
            if v_st = 47 then
                r_hb_a <= (others => '0');
            elsif e_fon = '1' and e_fs = 3 and e_fe <= 62 then
                r_hb_a <= resize(e_fe + 1, 8);
            end if;
            -- cosine ROM: phase 2 phi (even pq), 2 phi + PG (odd pq); fabric copy, Hard two-level
            if e_pon = '1' and v_ph <= 255 then
                if e_pq(0) = '0' then r_c_a <= e_pq(7 downto 1) & '0';
                else r_c_a <= (e_pq(7 downto 1) & '0') + c_pg; end if;
            end if;
            if s_hard = '1' then
                if r_c_q(8) = '1' then e_c <= to_signed(-256, 9); else e_c <= to_signed(255, 9); end if;
            else
                e_c <= signed(r_c_q);
            end if;

            -- multiplier operands (one product per clock)
            v_cs := to_integer(c_st);
            if v_st >= 10 and v_st <= 33 then
                if v_st = 10 then m_a <= resize(shift_right(ia_dh, 8), 13);
                elsif v_st = 11 then m_a <= signed(resize(unsigned(ia_dh(7 downto 0)), 13));
                elsif e_st(0) = '0' then m_a <= resize(ia_dh, 13);
                else m_a <= resize(ia_dl, 13); end if;
                m_b <= signed(resize(e_f, 10));
            elsif v_st = 38 then
                if c_z1r /= 0 then m_a <= signed(resize(c_z1r, 13)); else m_a <= signed(resize(c_z2r, 13)); end if;
                m_b <= signed(resize(f_r8, 10));
            elsif v_st >= 39 and v_st <= 45 then
                case v_st is
                    when 39 => m_a <= signed(resize(c_a, 13));
                    when 40 => m_a <= resize(c_g1, 13);
                    when 41 => m_a <= resize(c_g2, 13);
                    when 42 => m_a <= resize(c_z3, 13);
                    when 43 => m_a <= signed(resize(c_zs, 13));
                    when 44 => m_a <= resize(shift_right(e_bd, 8), 13);
                    when others => m_a <= signed(resize(unsigned(e_bd(7 downto 0)), 13));
                end case;
                m_b <= signed(resize(f_gr(9 downto 1), 10));
            elsif e_fon = '1' then
                case to_integer(e_fs) is
                    when 0 => m_a <= c_a2;  m_b <= signed(resize(unsigned(r_hb_q(47 downto 40)), 10));
                    when 1 => m_a <= c_g1s; m_b <= resize(signed(r_hb_q(39 downto 32)), 10);
                    when 2 => m_a <= c_g2s; m_b <= resize(signed(r_hb_q(31 downto 24)), 10);
                    when 3 =>
                        m_a <= c_zs2;
                        if c_mid = '1' then
                            case to_integer(e_fe(2 downto 0)) is      -- X = the 8x triangle at entry e
                                when 0 => m_b <= to_signed(0, 10);
                                when 1 | 7 => m_b <= to_signed(64, 10);
                                when 2 | 6 => m_b <= to_signed(128, 10);
                                when 3 | 5 => m_b <= to_signed(192, 10);
                                when others => m_b <= to_signed(255, 10);
                            end case;
                        else
                            m_b <= signed(resize(unsigned(r_hb_q(7 downto 0)), 10));
                        end if;
                    when others => m_a <= c_z3s; m_b <= signed(resize(unsigned(r_hb_q(15 downto 8)), 10));
                end case;
            elsif e_pon = '1' then
                if e_pq(0) = '1' then m_a <= resize(c_ra, 13); else m_a <= signed(resize(c_ga, 13)); end if;
                m_b <= resize(e_c, 10);
            else
                case v_cs is
                    when 1 => m_a <= signed(resize(c_kw, 13)); m_b <= signed(resize(f_b8, 10));
                    when 2 =>
                        case e_typ is
                            when "00" => m_a <= to_signed(172, 13);
                            when "01" => m_a <= to_signed(117, 13);
                            when others => m_a <= to_signed(222, 13);
                        end case;
                        m_b <= signed(resize(f_s8, 10));
                    when 3 =>
                        case e_typ is
                            when "00" => m_a <= to_signed(120, 13);
                            when "01" => m_a <= to_signed(170, 13);
                            when others => m_a <= to_signed(150, 13);
                        end case;
                        m_b <= signed(resize(f_s8, 10));
                    when 4 =>
                        case e_typ is
                            when "00" => m_a <= to_signed(140, 13);
                            when "01" => m_a <= to_signed(110, 13);
                            when others => m_a <= to_signed(160, 13);
                        end case;
                        m_b <= signed(resize(f_s8, 10));
                    when 5 => m_a <= signed(resize(e_amp, 13)); m_b <= signed(resize(f_s8, 10));
                    when 6 => m_a <= e_vb; m_b <= to_signed(73, 10);
                    when 7 => m_a <= e_vr; m_b <= to_signed(205, 10);
                    when 8 => m_a <= signed(resize(e_kwp, 13)); m_b <= to_signed(278, 10);
                    when 9 => m_a <= signed(resize(f_duv, 13)); m_b <= signed(resize(e_jp, 10));
                    when 10 => m_a <= signed(resize(e_lfsr(14 downto 7), 13)) - 128; m_b <= signed(resize(e_amps, 10));
                    when 15 => m_a <= signed(resize(f_duv, 13)); m_b <= e_dpx;
                    when others => null;
                end case;
            end if;
            -- the multiplier pipeline: b bits 0..3, 4..6, 7..9 (bit 9 = the sign)
            v_acc := (others => '0');
            for k in 0 to 3 loop
                if m_b(k) = '1' then v_acc := v_acc + shift_left(resize(m_a, 23), k); end if;
            end loop;
            m_s1 <= v_acc; m_a1 <= m_a; m_b1 <= m_b;
            v_acc := m_s1;
            for k in 4 to 6 loop
                if m_b1(k) = '1' then v_acc := v_acc + shift_left(resize(m_a1, 23), k); end if;
            end loop;
            m_s2 <= v_acc; m_a2 <= m_a1; m_b2 <= m_b1;
            v_acc := m_s2;
            for k in 7 to 8 loop
                if m_b2(k) = '1' then v_acc := v_acc + shift_left(resize(m_a2, 23), k); end if;
            end loop;
            if m_b2(9) = '1' then v_acc := v_acc - shift_left(resize(m_a2, 23), 9); end if;
            m_p <= v_acc;

            -- interpolation: the hi product of word w arrives in state 14 + 2w, the lo one in 15 + 2w
            v_q := resize(shift_right(m_p, 8), 10);
            case v_st is
                when 14 => c_b  <= c_b + resize(m_p, 17);
                when 15 => c_b  <= c_b + resize(shift_right(m_p, 8), 17);
                when 16 => c_a  <= c_a + unsigned(v_q(8 downto 0));
                when 17 => c_ga <= c_ga + unsigned(v_q(6 downto 0));
                when 18 => c_g1 <= c_g1 + v_q;
                when 19 => c_gb <= c_gb + unsigned(v_q(4 downto 0));
                when 20 => c_g2 <= c_g2 + v_q;
                when 24 => c_z3 <= c_z3 + v_q;
                when 26 => c_r0 <= c_r0 + v_q;
                when 28 => c_ra <= c_ra + v_q(8 downto 0);
                when 29 => c_z1r <= c_z1r + unsigned(v_q(6 downto 0));
                when 30 => c_g0 <= c_g0 + unsigned(v_q(7 downto 0));
                when 31 => c_pg <= c_pg + unsigned(v_q(7 downto 0));
                when 32 => c_kw <= c_kw + unsigned(v_q(8 downto 0));
                when 33 => c_z2r <= c_z2r + unsigned(v_q(6 downto 0));
                when 34 => c_rs <= c_rs + unsigned(v_q(7 downto 0));
                when 35 => c_gs <= c_gs + unsigned(v_q(7 downto 0));
                when 36 => c_re <= c_re + unsigned(v_q(7 downto 0));
                when 37 => c_ge <= c_ge + unsigned(v_q(7 downto 0));
                when others => null;
            end case;
            -- Rings gain into the coefficients (products of state s read in state s + 4)
            if v_st = 38 then e_bd <= c_b - resize(C_FR(16 downto 0), 17); end if;
            case v_st is
                when 42 =>
                    if c_z1r /= 0 then c_mid <= '1'; c_zs <= resize(unsigned(m_p(15 downto 8)), 10);
                    else c_mid <= '0'; c_zs <= shift_left(resize(unsigned(m_p(15 downto 8)), 10), 1); end if;
                when 43 => c_a2  <= resize(shift_right(m_p, 6), 13);
                when 44 => c_g1s <= resize(shift_right(m_p, 6), 13);
                when 45 => c_g2s <= resize(shift_right(m_p, 6), 13);
                when 46 => c_z3s <= resize(shift_right(m_p, 6), 13);
                when 47 => c_zs2 <= resize(shift_right(m_p, 6), 13);
                when 48 => e_kt  <= shift_left(resize(m_p, 21), 1);
                when 49 => c_k   <= e_kt + resize(shift_right(m_p, 7), 21) + f_cfr;
                when others => null;
            end case;

            -- F row: product tags (loads at slot k of entry e arrive 4 clocks later)
            if e_fon = '1' and e_fe <= 63 then t_v(1) <= '1'; else t_v(1) <= '0'; end if;
            t_e(1) <= e_fe; t_k(1) <= e_fs;
            for i in 2 to 4 loop t_v(i) <= t_v(i - 1); t_e(i) <= t_e(i - 1); t_k(i) <= t_k(i - 1); end loop;
            e_fpv <= '0';
            if t_v(4) = '1' then
                case to_integer(t_k(4)) is
                    when 0 => e_acc <= resize(shift_right(m_p, 4), 21);
                    when 1 | 2 => e_acc <= e_acc + resize(shift_right(m_p, 4), 21);
                    when 3 => e_acc <= e_acc + resize(shift_right(m_p, 5), 21);
                    when others =>
                        v_fp := c_k + e_acc + resize(shift_right(m_p, 5), 21);
                        e_fp <= v_fp(15 downto 0); e_fpv <= '1'; e_fpe <= t_e(4);
                end case;
            end if;
            -- write entry e - 1 when F(e) is ready; entry 63 one clock after F(63)
            fr_we <= '0'; e_last <= '0';
            if e_fpv = '1' then
                if e_fpe = 0 then
                    e_fp0 <= e_fp;
                else
                    v_df := e_fp - e_fpp;
                    fr_we <= '1'; fr_wa <= e_wb & resize(e_fpe - 1, 6);
                    fr_wd <= std_logic_vector(e_fpp) & std_logic_vector(v_df(15 downto 4)) & "0000";
                end if;
                e_fpp <= e_fp;
                if e_fpe = 63 then e_last <= '1'; end if;
            elsif e_last = '1' then
                v_df := e_fp0 - e_fpp;
                fr_we <= '1'; fr_wa <= e_wb & to_unsigned(63, 6);
                fr_wd <= std_logic_vector(e_fpp) & std_logic_vector(v_df(15 downto 4)) & "0000";
            end if;

            -- palette: R of phase phi arrives at pq = 2 phi + 7, G at 2 phi + 8 -> write
            pl_we <= '0';
            if e_pon = '1' and v_ph >= 7 then
                if e_pq(0) = '1' then
                    v_rr := resize(c_r0, 12) + resize(shift_right(m_p, 8), 12);
                    if v_rr < 0 then e_pr <= (others => '0'); elsif v_rr > 250 then e_pr <= to_unsigned(250, 8);
                    else e_pr <= unsigned(v_rr(7 downto 0)); end if;
                elsif v_ph <= 262 then
                    v_gg := signed(resize(c_g0, 12)) + resize(shift_right(m_p, 8), 12);
                    if v_gg < 0 then v_gg := (others => '0'); elsif v_gg > 255 then v_gg := to_signed(255, 12); end if;
                    pl_we <= '1'; pl_wa <= e_wb & to_unsigned((v_ph - 8) / 2, 7);
                    pl_wd <= std_logic_vector(e_pr) & std_logic_vector(v_gg(7 downto 0));
                end if;
            end if;

            ---------------------------------------------------------------------
            -- phase C: kicked when the previous line has left stage 12 (or after the
            -- zone's palette); products of state s are read in state s + 4
            ---------------------------------------------------------------------
            e_kick <= '0';
            if (s_gav_sr(11) = '1' and s_gav_sr(10) = '0') or (e_zch = '1' and e_pon = '1' and e_pq = 263) then
                e_kick <= '1';
            end if;
            if e_kick = '1' then
                c_st <= to_unsigned(1, 5);
            elsif c_st /= 0 then
                if c_st = 20 then c_st <= (others => '0'); else c_st <= c_st + 1; end if;
            end if;
            case v_cs is
                when 1 =>
                    e_dyt <= signed(resize(e_yf, 12)) - (833 - signed(resize(f_hh, 12)));
                    e_dyb <= (833 + signed(resize(f_hh, 12))) - signed(resize(e_yf, 12));
                    l_gb <= c_gb; l_rs <= c_rs; l_gs <= c_gs;
                    -- tears: a new block of 6..28 lines
                    if e_tl = 0 then
                        v_r := e_lfsr;
                        v_bit := v_r(0) xor v_r(2) xor v_r(3) xor v_r(5);
                        v_r := v_bit & v_r(15 downto 1);
                        e_lfsr <= v_r;
                        e_tl <= resize(v_r(3 downto 0), 5) + resize(v_r(6 downto 4), 5) + 5;
                        if v_r(6 downto 5) /= "00" then e_ton <= '1'; else e_ton <= '0'; end if;
                        if e_yf < 460 then e_amp <= (others => '0');
                        elsif e_yf < 640 then e_amp <= to_unsigned(30, 6);
                        else e_amp <= to_unsigned(12, 6); end if;
                        e_new <= '1';
                    else
                        e_tl <= e_tl - 1;
                        e_new <= '0';
                    end if;
                when 2 =>
                    if e_dyt < e_dyb then e_dy <= e_dyt; else e_dy <= e_dyb; end if;
                    -- hold gradient steps (per half-px step)
                    v_x := signed(resize(c_re, 14)) - signed(resize(c_rs, 14));
                    case to_integer(s_xs) is
                        when 4 => l_drx <= resize(shift_left(v_x, 2), 11);
                        when 3 => l_drx <= resize(shift_left(v_x, 1) + v_x, 11);
                        when others => l_drx <= resize(shift_left(v_x, 1), 11);
                    end case;
                    v_x := signed(resize(c_ge, 14)) - signed(resize(c_gs, 14));
                    case to_integer(s_xs) is
                        when 4 => l_dgx <= resize(shift_left(v_x, 2), 11);
                        when 3 => l_dgx <= resize(shift_left(v_x, 1) + v_x, 11);
                        when others => l_dgx <= resize(shift_left(v_x, 1), 11);
                    end case;
                when 3 =>
                    e_vb <= shift_left(resize(e_dy, 13), 1) + 6;
                    e_vr <= shift_left(resize(e_dy, 13), 1) + 22;
                when 5 => e_kwp <= unsigned(m_p(17 downto 8));
                when 6 => e_sp <= unsigned(m_p(16 downto 8));
                    -- 1112 kw in units of 64
                when 7 => e_l1p <= unsigned(m_p(16 downto 8));
                    l_s <= e_sp;
                    l_2s <= shift_left(resize(e_sp, 11), 1);
                    l_2sm <= shift_left(resize(e_sp, 11), 1) - resize(s_xs, 11);
                when 8 => e_jp <= unsigned(m_p(16 downto 8));
                    l_2l1x <= signed(shift_left(resize(e_l1p, 13), 1)) - signed(resize(s_xs, 13));
                    case e_typ is
                        when "00" =>   l_xhx <= to_signed(350, 13) - signed(resize(s_xs, 13));
                        when "01" =>   l_xhx <= to_signed(310, 13) - signed(resize(s_xs, 13));
                        when others => l_xhx <= to_signed(530, 13) - signed(resize(s_xs, 13));
                    end case;
                    if e_yf < 470 and f_s8 /= 0 then l_act <= '1'; else l_act <= '0'; end if;
                when 9 => e_amps <= unsigned(m_p(14 downto 8));
                when 10 => l_abv <= f_clamp16(shift_right(m_p, 6));            -- (2 dy + 6) x 73 / 64
                when 11 => l_arv <= f_clamp16(shift_right(m_p, 9));            -- (2 dy + 22) x 205 / 512
                when 12 => l_kwu <= unsigned(m_p(17 downto 4));                -- 1112 kw / 64
                when 13 => l_jump <= resize(m_p, 21);
                when 14 =>
                    if e_ton = '1' then e_dpx <= resize(shift_right(m_p, 7), 10); else e_dpx <= (others => '0'); end if;
                when 19 =>
                    if e_new = '1' then l_dut <= resize(m_p, 21); end if;
                when others => null;
            end case;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Beam: the horizontal phase u (2^20 per period), the stutter phase error
    -- (freeze at the line-start release then snap back; after each seam a jump, a
    -- plateau and a catch-up), the virtual x
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_un  : unsigned(20 downto 0);
        variable v_xn  : unsigned(11 downto 0);
        variable v_e   : signed(20 downto 0);
        variable v_md  : unsigned(2 downto 0);
        variable v_cnt, v_cn : unsigned(11 downto 0);
        variable v_em, v_ej : signed(20 downto 0);
        variable v_u2  : unsigned(20 downto 0);
        variable v_cb  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ut <= f_u0; b_e <= (others => '0'); b_xh <= (others => '0'); b_md <= (others => '0');
                b_cnt <= (others => '0');
                v_u2 := resize(f_u0, 21) + resize(f_du, 21);
                b_sc <= v_u2(20);
                if l_2sm = 0 and l_2s /= 0 then b_ft <= '1'; else b_ft <= '0'; end if;
            elsif g_avid = '1' then
                -- the flags of the next pixel are registered here, so the mode logic
                -- below only sees registered conditions
                v_un := resize(b_ut, 21) + resize(f_du, 21);
                v_u2 := resize(b_ut, 21) + shift_left(resize(f_du, 21), 1);
                b_sc <= v_u2(20) xor v_un(20);
                v_xn := b_xh + resize(s_xs, 12);
                if v_xn >= l_2sm and v_xn < l_2s then b_ft <= '1'; else b_ft <= '0'; end if;
                v_cn := b_cnt + resize(s_xs, 12);
                v_em := b_e - signed(resize(f_du, 21));
                v_ej := b_e + l_jump;
                v_e := b_e; v_md := b_md; v_cnt := b_cnt; v_cb := signed(resize(b_cnt, 13));
                if l_act = '1' then
                    if b_ft = '1' then
                        v_md := "001"; v_cnt := (others => '0');
                        v_cn := resize(s_xs, 12); v_cb := (others => '0');
                    elsif v_md = "000" and b_sc = '1' then
                        v_md := "100"; v_cnt := (others => '0');
                        v_cn := resize(s_xs, 12); v_cb := (others => '0'); v_e := v_ej;
                    end if;
                    case v_md is
                        when "001" =>
                            v_e := v_em; v_cnt := v_cn;
                            if v_cb >= l_2l1x then v_md := "010"; end if;
                        when "010" =>
                            v_e := (others => '0'); v_md := "000";
                        when "100" =>
                            v_cnt := v_cn;
                            if v_cb >= l_xhx then v_md := "101"; end if;
                        when "101" =>
                            if b_e <= signed(resize(f_du, 21)) then v_e := (others => '0'); v_md := "000";
                            else v_e := v_em; end if;
                        when others => null;
                    end case;
                end if;
                b_ut <= v_un(19 downto 0); b_xh <= v_xn; b_e <= v_e; b_md <= v_md; b_cnt <= v_cnt;
            end if;
            if s_zgo = '1' then
                s_oxl <= s_oy;
            elsif data_in.avid = '1' and s_avq = '0' then
                s_oxl <= s_oxl + resize(s_oys, 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage k = k registers after the beam)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : signed(20 downto 0);
        variable v_u, v_ut : unsigned(19 downto 0);
        variable v_lm  : signed(11 downto 0);
        variable v_f   : unsigned(15 downto 0);
        variable v_ak  : signed(14 downto 0);
        variable v_xv  : unsigned(10 downto 0);
        variable v_ti  : signed(11 downto 0);
        variable v_r, v_gr, v_b : unsigned(15 downto 0);
        variable v_y   : unsigned(16 downto 0);
        variable v_cs  : signed(17 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_rf, v_gf, v_bf : unsigned(7 downto 0);
        variable v_g9  : unsigned(8 downto 0);
        variable v_h   : signed(17 downto 0);
        variable v_o   : unsigned(15 downto 0);
        variable v_et  : unsigned(10 downto 0);
        variable v_sp  : signed(15 downto 0);
        variable v_kd  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap (a few px early: invisible in a bend): (Y - 64) / 2, 2-px average
            s_vy1 <= unsigned(data_in.y); s_vy2 <= s_vy1;
            v_lm := signed(resize(s_vy1, 12)) + signed(resize(s_vy2, 12)) - 128;
            if v_lm < 0 then s_lum <= (others => '0');
            elsif v_lm > 876 then s_lum <= to_unsigned(438, 9);
            else s_lum <= unsigned(v_lm(10 downto 2)); end if;

            -- 1: field phase = true phase + stutter error + tear offset; Mirror; the true phase
            --    for the cushion key; Osc
            v_d := signed(resize(b_ut, 21)) + b_e + l_dut;
            v_u := unsigned(v_d(19 downto 0)); v_ut := b_ut;
            if s_mir = '1' then
                v_u := resize(shift_left(resize(C_UPK, 21), 1) - resize(v_u, 21), 20);
            end if;
            p1_u <= v_u(19 downto 8);
            p1_ut <= b_ut(19 downto 6);
            v_o := b_ut(19 downto 4) + s_ox;
            p1_osc <= f_tri8(v_o);
            -- 2: field-row address (64 entries per period) and fraction (1/8); offset from the cushion axis
            fr_ra <= r_bank & p1_u(11 downto 6);
            p2_fr <= p1_u(5 downto 3);
            p2_kd <= signed(p1_ut - C_UK(19 downto 6));
            -- 3: field-row read; |offset|; luma / Osc (7 bit)
            v_ak := abs(resize(p2_kd, 15)); p3_ak <= unsigned(v_ak(13 downto 0));
            p3_fr <= p2_fr;
            if s_src = '1' then p3_lum <= p1_osc(7 downto 1); else p3_lum <= s_lum(8 downto 2); end if;
            -- 4: fabric copy; cushion horizontal distance; etch x gain (bits 3, 2)
            p4_f <= unsigned(fr_q(31 downto 16)); p4_sl <= signed(fr_q(15 downto 4));
            p4_fr <= p3_fr; p4_lum <= p3_lum;
            p4_dx <= signed(resize(l_kwu, 15)) - signed(resize(p3_ak, 15));
            p_e4 <= f_e8g(7 downto 4);
            v_et := (others => '0');
            if p_e4(3) = '1' then v_et := shift_left(resize(p3_lum, 11), 3); end if;
            if p_e4(2) = '1' then v_et := v_et + shift_left(resize(p3_lum, 11), 2); end if;
            p4_et <= v_et;
            -- 5: slope x fraction and the etch (bits 1, 0) by conditional adds; cushion alphas
            p5_f <= p4_f;
            v_sp := (others => '0');
            if p4_fr(2) = '1' then v_sp := shift_left(resize(p4_sl, 16), 2); end if;
            if p4_fr(1) = '1' then v_sp := v_sp + shift_left(resize(p4_sl, 16), 1); end if;
            if p4_fr(0) = '1' then v_sp := v_sp + resize(p4_sl, 16); end if;
            p5_sp <= v_sp;
            v_et := p4_et;
            if p_e4(1) = '1' then v_et := v_et + shift_left(resize(p4_lum, 11), 1); end if;
            if p_e4(0) = '1' then v_et := v_et + resize(p4_lum, 11); end if;
            p5_et <= v_et;
            p5_abh <= f_clamp16(shift_right(p4_dx + 104, 4));
            p5_arh <= f_clamp16(shift_right(p4_dx + 382, 5));
            -- 6: F -> palette address (1/128 cycle)
            v_f := p5_f + unsigned(shift_left(p5_sp, 1)) + shift_left(resize(p5_et, 16), 2);
            pl_ra <= r_bank & v_f(11 downto 5);
            if p5_abh < l_abv then p_ab(6) <= p5_abh; else p_ab(6) <= l_abv; end if;
            if p5_arh < l_arv then p_ar(6) <= p5_arh; else p_ar(6) <= l_arv; end if;
            -- 7: palette read
            -- 8: fabric copy
            p8_r <= unsigned(pl_q(15 downto 8)); p8_g <= unsigned(pl_q(7 downto 0));
            -- 9: cushion key product; the green ground's boost; B
            v_kd := to_unsigned(241, 8) - p8_r;
            case to_integer(p_ar(8)(4 downto 2)) is
                when 0 => p9_kp <= (others => '0');
                when 1 => p9_kp <= shift_right(v_kd, 2);
                when 2 => p9_kp <= shift_right(v_kd, 1);
                when 3 => p9_kp <= shift_right(v_kd, 1) + shift_right(v_kd, 2);
                when others => p9_kp <= v_kd;
            end case;
            p9_r <= p8_r; p9_g <= p8_g;
            if p_ab(8) < 6 then p9_bo <= l_gb;
            elsif p_ab(8) < 11 then p9_bo <= shift_right(l_gb, 1);
            else p9_bo <= (others => '0'); end if;
            p9_b <= resize(shift_right(shift_left(resize(p_ab(8), 13), 8) - shift_left(resize(p_ab(8), 13), 5)
                    - shift_left(resize(p_ab(8), 13), 1), 4), 8);
            -- 10: compose: hold gradient / flash / field; Palette
            v_xv := h_xh(11 downto 1);
            v_ti := signed(resize(v_xv, 12)) - signed(resize(l_s, 12));
            v_g9 := resize(p9_g, 9) + resize(p9_bo, 9);
            if l_act = '1' and v_xv < l_s then
                v_h := signed(resize(l_rs, 18)) + shift_right(h_ar, 8); v_rf := unsigned(v_h(7 downto 0));
                v_h := signed(resize(l_gs, 18)) + shift_right(h_ag, 8); v_gf := unsigned(v_h(7 downto 0));
                v_bf := (others => '0');
            elsif l_act = '1' and v_ti >= 0 and v_ti < 16 then
                case to_integer(v_ti(3 downto 2)) is
                    when 0 => v_rf := to_unsigned(245, 8); v_gf := to_unsigned(70, 8); v_bf := to_unsigned(30, 8);
                    when 1 => v_rf := to_unsigned(244, 8); v_gf := to_unsigned(120, 8); v_bf := to_unsigned(180, 8);
                    when 2 => v_rf := to_unsigned(246, 8); v_gf := to_unsigned(190, 8); v_bf := to_unsigned(120, 8);
                    when others => v_rf := to_unsigned(250, 8); v_gf := to_unsigned(200, 8); v_bf := to_unsigned(20, 8);
                end case;
            else
                v_rf := p9_r + p9_kp;
                if v_g9 > 255 then v_gf := (others => '1'); else v_gf := v_g9(7 downto 0); end if;
                v_bf := p9_b;
            end if;
            if s_night = '1' then
                p10_r <= v_bf; p10_g <= v_gf; p10_b <= v_rf;
            else
                p10_r <= v_rf; p10_g <= v_gf; p10_b <= v_bf;
            end if;
            -- 11: RGB -> YUV terms (hardware convention: U = Cr, V = Cb), luma pre-scaled 16/13:
            --     Y = 64 + (5 R + 10 G + 2 B) / 4;  U = 512 + (7 R - 6 G - B) / 4;  V = 512 + (14 B - 5 R - 9 G) / 8
            v_r := resize(p10_r, 16); v_gr := resize(p10_g, 16); v_b := resize(p10_b, 16);
            p11_y5r  <= resize(shift_left(v_r, 2) + v_r, 12);
            p11_y10g <= resize(shift_left(v_gr, 3) + shift_left(v_gr, 1), 12);
            p11_y2b  <= resize(shift_left(v_b, 1), 11);
            p11_u7r  <= resize(shift_left(v_r, 3) - v_r, 12);
            p11_u6g  <= resize(shift_left(v_gr, 2) + shift_left(v_gr, 1), 12);
            p11_ub   <= p10_b;
            p11_v14b <= resize(shift_left(v_b, 4) - shift_left(v_b, 1), 12);
            p11_v5r  <= resize(shift_left(v_r, 2) + v_r, 11);
            p11_v9g  <= resize(shift_left(v_gr, 3) + v_gr, 12);
            -- 12: Y (capped 940), U, V (10 bit)
            v_y := resize(shift_right(resize(p11_y5r, 14) + resize(p11_y10g, 14) + resize(p11_y2b, 14), 2), 17) + 64;
            if v_y > 940 then p12_y <= to_unsigned(940, 10); else p12_y <= v_y(9 downto 0); end if;
            v_cs := resize(shift_right(signed(resize(p11_u7r, 15)) - signed(resize(p11_u6g, 15)) - signed(resize(p11_ub, 15)), 2), 18) + 512;
            if v_cs < 0 then p12_u <= (others => '0'); elsif v_cs > 1023 then p12_u <= (others => '1');
            else p12_u <= unsigned(v_cs(9 downto 0)); end if;
            v_cs := resize(shift_right(signed(resize(p11_v14b, 15)) - signed(resize(p11_v5r, 15)) - signed(resize(p11_v9g, 15)), 3), 18) + 512;
            if v_cs < 0 then p12_v <= (others => '0'); elsif v_cs > 1023 then p12_v <= (others => '1');
            else p12_v <= unsigned(v_cs(9 downto 0)); end if;

            -- 13..18: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent
            -- 8-sums over the ring window, luma [1 2 1]; centre at stage 17 -> 18)
            s_boxu <= s_boxu + p12_u - unsigned(ru_q2(19 downto 10));
            s_boxv <= s_boxv + p12_v - unsigned(ru_q2(9 downto 0));
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v18 <= v_sc(13 downto 4);
            s_y13 <= p12_y; s_y14 <= s_y13; s_y15 <= s_y14; s_y16 <= s_y15;
            s_y17 <= s_y16; s_y18x <= s_y17;
            v_sy := resize(s_y16, 12) + shift_left(resize(s_y17, 12), 1) + resize(s_y18x, 12);
            s_yf18 <= v_sy(11 downto 2);
            -- 19: luma toward black (white 940 -> 777); blanking gate
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            -- 20: output register
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            -- hold gradient: x and the two ramps of the stage-9 pixel, advanced on its avid
            if s_gav_sr(8) = '0' then
                h_xh <= (others => '0'); h_cnt <= (others => '0');
                h_ar <= (others => '0'); h_ag <= (others => '0');
            else
                h_xh <= h_xh + resize(s_xs, 12);
                if h_cnt < s_k254 then
                    h_cnt <= h_cnt + 1;
                    h_ar <= h_ar + resize(l_drx, 18); h_ag <= h_ag + resize(l_dgx, 18);
                end if;
            end if;

            -- delays
            p_ab(7) <= p_ab(6); p_ar(7) <= p_ar(6);
            p_ab(8) <= p_ab(7); p_ar(8) <= p_ar(7);
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= rs_q2(2);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lotus;
