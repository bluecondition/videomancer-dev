-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Plumage v0.1.0 -- Heirloom phase 3 (video clips), from the clip WIN_20230710_00_15_14 at 1.5 s.
--
-- The still: a fan of curved feathers sweeping up and right from a root at the lower left (near-vertical at
-- the left, ~30 deg at the right), each with a crisp dark leading edge and banded violet / purple / olive /
-- magenta / pink shading that fades into the ground; a row of short spikes along the bottom; a warm ground
-- (yellow -> orange -> crimson, left to right) inside a band that scrolls down through a cool one (green ->
-- teal -> navy); a green -> black root strip at the far left; torn streaks at the top left.
--
-- The patch, as fitted (plumage.py has every constant):
--   KL   = 930 ln(x + 140)                               (normalize + a 512-entry ROM of 930 ln(.), x in h)
--   Z    = PI - s y - KL + bend * luma                   parallel log curves; s = the sweep (1 = the still)
--   u    = lane0 - G ln(Z)                              re-spaced so the lanes crowd toward the lower right;
--          G ln(.) from two per-field tables (no pixel multiplier)
--   feather = s = frac(u) < w (1 + j/8) tap(a): crisp edge at s = 0, palette at t = s / (w tap),
--          a = x + stream + OFFA(lane) on a 15000 px segment tapered at both ends, streaming inward
--   teeth = rows of y + 0.375 K L1 (170.7 lines apart), a tooth every 520 px (rounded nose, pointed tip),
--          only in the lowest `rows` rows; teeth over feathers over the ground
--   ground = warm / cool by x inside / outside a 1600-line band that scrolls down, its top boundary lumpy,
--          torn by per-line vertical jitter; root strip at x < 190
--   tear = per-line horizontal offsets of x, top-left region (grows with P12)
--
--   K1  Feathers  number of feathers (G, x0.33 .. x3)            K2  Sweep  curvature (s x2.8 .. x0.35)
--   K3  Width     feather width (quills .. overlapping plumes)   K4  Speed  bipolar stream / wipe rate
--   K5  Teeth     rows of teeth (none .. the still .. the frame) K6  Bend   camera (or Osc) bend / key
--   S7  Colours   Plumage / Rainbow    S8 Mirror  One / Two fans    S9 Sky  Day / Night (warm and cool swap)
--   S10 Root      Below / Above (the fan hangs from the top left)     S11 Source  Video / Osc
--   P12 TEAR      pristine fan .. the still (380) .. the whole picture shredded into streaks
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared with the line engine; shared glide), a per-line
-- engine (tear draws, band
-- phase, line terms of the two fields), a per-field table fill, a 27-stage pixel path, five block-RAM delay
-- rings, then the shell's
-- output conditioning (U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum resync, luma [1 2 1]
-- and 13/16) and the blanking gate.  Generated avid; one latency (32) in every mode; virtual 1080-line
-- coordinates (SD shows the centre 4:3); colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.plumage_pkg.all;
use work.all;

architecture plumage of program_top is
    constant C_LAT : integer := 32;      -- = processing_delay_clks
    constant C_LB  : integer := 30000;   -- feather segment length (h), period 32768

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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- position of the leading one (0 if none)
    function f_msb13(v : unsigned(12 downto 0)) return unsigned is
        variable r : unsigned(3 downto 0) := (others => '0');
    begin
        for i in 0 to 12 loop
            if v(i) = '1' then r := to_unsigned(i, 4); end if;
        end loop;
        return r;
    end function;
    function f_msb19(v : unsigned(18 downto 0)) return unsigned is
        variable r : unsigned(4 downto 0) := (others => '0');
    begin
        for i in 0 to 18 loop
            if v(i) = '1' then r := to_unsigned(i, 5); end if;
        end loop;
        return r;
    end function;
    -- 4x4 Bayer threshold
    function f_bayer(x : unsigned(1 downto 0); y : unsigned(1 downto 0)) return unsigned is
        type t_b is array (0 to 15) of integer range 0 to 15;
        constant C_B : t_b := (0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5);
    begin
        return to_unsigned(C_B(to_integer(y & x)), 4);
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data; each read once per clock)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_kln_a : unsigned(8 downto 0) := (others => '0');
    signal r_kln_q : unsigned(15 downto 0) := (others => '0');
    signal r_top_a : unsigned(7 downto 0) := (others => '0');
    signal r_top_q : unsigned(15 downto 0) := (others => '0');
    signal r_qb_a  : unsigned(7 downto 0) := (others => '0');
    signal r_qb_q  : unsigned(15 downto 0) := (others => '0');
    signal r_pal_a, r_bg_a : unsigned(7 downto 0) := (others => '0');
    signal r_pal_q, r_bg_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_fs_a  : unsigned(8 downto 0) := (others => '0');
    signal r_fs_q  : unsigned(15 downto 0) := (others => '0');

    -- per-field tables (block RAM, written by the fill engine in the vblank zone only, read by the pixel
    -- path during active video only): G ln(1 + m/256), G p ln 2 (mod 16 lanes), the feather taper / width
    signal gln : t_c_pl_gln0 := C_PL_GLN0;
    signal ge  : t_c_pl_ge0 := C_PL_GE0;
    signal qap : t_c_pl_qap0 := C_PL_QAP0;
    attribute no_rw_check : boolean;
    attribute no_rw_check of gln : signal is true;
    attribute no_rw_check of ge  : signal is true;
    attribute no_rw_check of qap : signal is true;
    signal r_gln_a : unsigned(7 downto 0) := (others => '0');
    signal r_ge_a  : unsigned(4 downto 0) := (others => '0');
    signal r_qa_a  : unsigned(6 downto 0) := (others => '0');
    signal r_gln_q, r_ge_q, r_qa_q : unsigned(15 downto 0) := (others => '0');
    signal f_wa  : unsigned(7 downto 0) := (others => '0');
    signal f_wd  : unsigned(15 downto 0) := (others => '0');
    signal f_we1, f_we2, f_we3 : std_logic := '0';
    signal f_run : std_logic := '0';
    signal f_i   : unsigned(8 downto 0) := (others => '0');
    signal f_c   : unsigned(3 downto 0) := (others => '0');
    signal f_tab : unsigned(1 downto 0) := (others => '0');
    signal fm_a, fm_acc : unsigned(25 downto 0) := (others => '0');
    signal fm_b  : unsigned(11 downto 0) := (others => '0');

    -- delay rings (block RAM): ring 1 carries the feather segment base from stage 4 to 16, ring 2 the tooth
    -- phase base from stage 4 to 13, ring 3 the ground {mode, index} from stage 8 to 23, ring 4 the filtered
    -- luma 8 pixels back (to sit with the chroma boxes' delay), ring 5 the syncs and the generated avid for the
    -- first 24 clocks of the latency.  Reads never meet the write address.
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ring1, ring2, ring3, ring4, ring5 : t_r16 := (others => (others => '0'));
    attribute no_rw_check of ring1 : signal is true;
    attribute no_rw_check of ring2 : signal is true;
    attribute no_rw_check of ring3 : signal is true;
    attribute no_rw_check of ring4 : signal is true;
    attribute no_rw_check of ring5 : signal is true;
    signal w_wa  : unsigned(7 downto 0) := (others => '0');
    signal w_r1d, w_r2d, w_r3d : std_logic_vector(15 downto 0) := (others => '0');
    signal w_r1q, w_r2q, w_r3q : std_logic_vector(15 downto 0) := (others => '0');
    signal w_r4q : std_logic_vector(15 downto 0) := x"0040";
    signal w_r5q : std_logic_vector(15 downto 0) := x"0007";

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(380 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(772, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_bank, s_mir, s_night, s_flip, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(12 downto 0) := (others => '0');
    signal z_shg, z_shw, z_shs : signed(2 downto 0) := (others => '0');
    signal z_shr  : signed(3 downto 0) := (others => '0');
    signal z_mg, z_mw, z_mr : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_vsq  : unsigned(17 downto 0) := (others => '0');
    signal z_dt   : unsigned(9 downto 0) := (others => '0');
    signal c_s    : unsigned(11 downto 0) := to_unsigned(1024, 12);     -- sweep: y scale, Q10
    signal c_g    : unsigned(11 downto 0) := to_unsigned(914, 12);      -- lanes per nat, Q8
    signal c_rw   : unsigned(9 downto 0) := to_unsigned(123, 10);       -- 1 / width, Q6
    signal c_rows : unsigned(11 downto 0) := to_unsigned(128, 12);      -- teeth rows, Q8
    signal c_rint : unsigned(3 downto 0) := (others => '0');
    signal c_rsh  : unsigned(1 downto 0) := "01";
    signal c_rpart, c_teeth : std_logic := '1';
    signal c_pik  : unsigned(19 downto 0) := to_unsigned(147024, 20);   -- 16 (PI + K ln 2)
    signal c_b0   : unsigned(17 downto 0) := to_unsigned(53824, 18);    -- 16 (B0 + 0.375 K ln 2)
    signal c_lane0 : unsigned(13 downto 0) := to_unsigned(10647, 14);   -- G ln 16 + 0.5, Q10 lanes
    signal z_inc  : unsigned(14 downto 0) := (others => '0');           -- stream / wipe per field, Q8
    signal z_incb : unsigned(11 downto 0) := (others => '0');           -- tooth phase per field, Q16
    signal c_stra : unsigned(22 downto 0) := (others => '0');
    signal c_strb : unsigned(15 downto 0) := (others => '0');
    signal c_scr  : unsigned(18 downto 0) := (others => '0');
    signal c_tg   : unsigned(7 downto 0) := to_unsigned(64, 8);          -- tear gain, Q6
    signal c_xt   : unsigned(12 downto 0) := to_unsigned(1800, 13);     -- tear region x limit (h)
    signal c_ylim : unsigned(10 downto 0) := to_unsigned(800, 11);      -- tear region y limit (lines)
    signal c_bigth : unsigned(7 downto 0) := to_unsigned(111, 8);
    signal c_vg   : unsigned(7 downto 0) := to_unsigned(39, 8);          -- camera gain
    signal c_xb0  : unsigned(11 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (computes the NEXT line)
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ln   : unsigned(1 downto 0) := (others => '0');
    signal e_yf   : unsigned(18 downto 0) := (others => '0');
    signal e_ys   : unsigned(10 downto 0) := (others => '0');
    signal e_r1   : unsigned(31 downto 0) := x"2545F491";
    signal e_r2   : unsigned(31 downto 0) := x"9E3779B9";
    signal e_ey, e_eyv : unsigned(4 downto 0) := (others => '0');
    signal e_sm   : signed(7 downto 0) := (others => '0');
    signal e_bg   : signed(9 downto 0) := (others => '0');
    signal e_fl   : std_logic := '0';
    signal e_raw  : signed(9 downto 0) := (others => '0');
    signal e_jr   : signed(8 downto 0) := (others => '0');
    signal e_jvf  : std_logic := '0';
    signal e_g    : unsigned(11 downto 0) := (others => '0');
    signal e_jv   : signed(10 downto 0) := (others => '0');
    signal e_yb   : unsigned(18 downto 0) := (others => '0');
    signal e_ysc  : unsigned(17 downto 0) := (others => '0');
    signal e_z1   : signed(18 downto 0) := (others => '0');
    signal e_fa   : unsigned(31 downto 0) := (others => '0');
    signal e_fb   : unsigned(15 downto 0) := (others => '0');
    signal e_ld   : std_logic := '0';
    signal n_toff : signed(10 downto 0) := (others => '0');
    signal n_zy   : unsigned(19 downto 0) := (others => '0');
    signal n_uby  : signed(17 downto 0) := (others => '0');
    signal n_bl   : unsigned(10 downto 0) := (others => '0');
    signal n_ln   : unsigned(1 downto 0) := (others => '0');

    -- beam and this line's constants
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_toff : signed(10 downto 0) := (others => '0');
    signal l_zy   : unsigned(19 downto 0) := (others => '0');
    signal l_uby  : signed(17 downto 0) := (others => '0');
    signal l_bl   : unsigned(10 downto 0) := (others => '0');
    signal l_ln   : unsigned(1 downto 0) := (others => '0');

    -- pixel path (qN = stage N; the beam registers are stage 1)
    signal s_tog  : std_logic := '0';
    signal q2_xm  : unsigned(11 downto 0) := (others => '0');
    signal q2_ex  : unsigned(1 downto 0) := (others => '0');
    signal q2_px, q3_px, q4_px, q5_px, q6_px : unsigned(1 downto 0) := (others => '0');
    signal q3_xt, q4_xt, q5_xt, q6_xt, q7_xt : signed(13 downto 0) := (others => '0');
    signal q4_dx, q5_dx : unsigned(12 downto 0) := (others => '0');
    signal q4_a1  : unsigned(14 downto 0) := (others => '0');
    signal q4_vb  : unsigned(15 downto 0) := (others => '0');
    signal q4_va, q4_vbl : unsigned(11 downto 0) := (others => '0');
    signal q5_p1, q6_p1 : unsigned(3 downto 0) := (others => '0');
    signal q5_vp  : unsigned(15 downto 0) := (others => '0');
    signal q6_top : unsigned(9 downto 0) := (others => '0');
    signal q6_vz, q7_vz : unsigned(13 downto 0) := (others => '0');
    signal q6_vd  : unsigned(10 downto 0) := (others => '0');
    signal q7_ke, q8_ke : unsigned(16 downto 0) := (others => '0');
    signal q7_dd  : unsigned(10 downto 0) := (others => '0');
    signal q7_thr : unsigned(10 downto 0) := (others => '0');
    signal q7_dth3 : unsigned(5 downto 0) := (others => '0');
    signal q7_dth5 : unsigned(6 downto 0) := (others => '0');
    signal q7_xr  : signed(13 downto 0) := (others => '0');
    signal q8_kln : unsigned(13 downto 0) := (others => '0');
    signal q8_zv, q9_zv : unsigned(20 downto 0) := (others => '0');
    signal q8_gm  : unsigned(7 downto 0) := (others => '0');
    signal q9_kl  : unsigned(17 downto 0) := (others => '0');
    signal q10_z, q11_z : unsigned(18 downto 0) := (others => '0');
    signal q10_ubk : unsigned(16 downto 0) := (others => '0');
    signal q11_p2 : unsigned(4 downto 0) := (others => '0');
    signal q11_ub : signed(17 downto 0) := (others => '0');
    signal q12_kb, q13_kb, q14_kb : unsigned(3 downto 0) := (others => '0');
    signal q12_sb, q13_sb, q14_sb, q15_sb, q16_sb, q17_sb, q18_sb : unsigned(7 downto 0) := (others => '0');
    signal q12_win, q13_win, q14_win, q15_win, q16_win, q17_win, q18_win, q19_win : std_logic := '0';
    signal q12_pt, q13_pt, q14_pt, q15_pt, q16_pt, q17_pt, q18_pt, q19_pt : std_logic := '0';
    signal q14_gln : unsigned(12 downto 0) := (others => '0');
    signal q14_ge  : unsigned(13 downto 0) := (others => '0');
    signal q14_vb0 : unsigned(15 downto 0) := (others => '0');
    signal q15_ua : unsigned(13 downto 0) := (others => '0');
    signal q15_vb : unsigned(15 downto 0) := (others => '0');
    signal q16_sa, q17_sa, q18_sa, q19_sa, q20_sa, q21_sa, q22_sa : unsigned(7 downto 0) := (others => '0');
    signal q16_off, q17_off : unsigned(14 downto 0) := (others => '0');
    signal q17_a1 : unsigned(14 downto 0) := (others => '0');
    signal q18_a  : unsigned(14 downto 0) := (others => '0');
    signal q18_qb : unsigned(11 downto 0) := (others => '0');
    signal q19_in, q20_in, q21_in, q22_in, q23_in : std_logic := '0';
    signal q19_d1, q19_d2 : unsigned(14 downto 0) := (others => '0');
    signal q19_ba, q19_bb : unsigned(13 downto 0) := (others => '0');
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    signal q_tb   : t_u11(20 to 24) := (others => (others => '1'));
    signal q22_qa : unsigned(11 downto 0) := (others => '0');
    signal q23_ta, q23_tb : unsigned(13 downto 0) := (others => '0');
    signal q24_ta : unsigned(10 downto 0) := (others => '1');
    signal q24_gm : unsigned(7 downto 0) := (others => '0');
    signal q25_p, q26_p : std_logic := '0';
    signal s_y27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma boxes, luma [1 2 1]
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cau, s_cav, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_yd1, s_yd2, s_y29, s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29, s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs, the
    -- per-field tables and the rings
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q  <= C_PL_EXP2(to_integer(r_ex_a));
            r_kln_q <= C_PL_KLN(to_integer(r_kln_a));
            r_top_q <= C_PL_TOP(to_integer(r_top_a));
            r_qb_q  <= C_PL_QB(to_integer(r_qb_a));
            r_pal_q <= C_PL_PAL(to_integer(r_pal_a));
            r_bg_q  <= C_PL_BG(to_integer(r_bg_a));
            r_fs_q  <= C_PL_FSRC(to_integer(r_fs_a));
        end if;
    end process p_rom;

    p_gln : process(clk)
    begin
        if rising_edge(clk) then
            r_gln_q <= gln(to_integer(r_gln_a));
            if f_we1 = '1' then gln(to_integer(f_wa)) <= f_wd; end if;
        end if;
    end process p_gln;
    p_ge : process(clk)
    begin
        if rising_edge(clk) then
            r_ge_q <= ge(to_integer(r_ge_a));
            if f_we2 = '1' then ge(to_integer(f_wa(4 downto 0))) <= f_wd; end if;
        end if;
    end process p_ge;
    p_qap : process(clk)
    begin
        if rising_edge(clk) then
            r_qa_q <= qap(to_integer(r_qa_a));
            if f_we3 = '1' then qap(to_integer(f_wa(6 downto 0))) <= f_wd; end if;
        end if;
    end process p_qap;

    -- ring 1: written from stage 4, read data at 16 (write address - 11); ring 2: from 4, data at 13 (- 8);
    -- ring 3: from 8, data at 23 (- 14); ring 4: luma, 10 clocks; ring 5: syncs / avid, 24 clocks.  Each read
    -- data register gets a fabric copy before use.
    p_ring1 : process(clk)
    begin
        if rising_edge(clk) then
            w_r1q <= ring1(to_integer(w_wa - 11));
            ring1(to_integer(w_wa)) <= w_r1d;
        end if;
    end process p_ring1;
    p_ring2 : process(clk)
    begin
        if rising_edge(clk) then
            w_r2q <= ring2(to_integer(w_wa - 8));
            ring2(to_integer(w_wa)) <= w_r2d;
        end if;
    end process p_ring2;
    p_ring4 : process(clk)
    begin
        if rising_edge(clk) then
            w_r4q <= ring4(to_integer(w_wa - 8));
            ring4(to_integer(w_wa)) <= "000000" & std_logic_vector(s_y29);
        end if;
    end process p_ring4;
    p_ring5 : process(clk)
    begin
        if rising_edge(clk) then
            w_r5q <= ring5(to_integer(w_wa - 23));
            ring5(to_integer(w_wa)) <= x"000" & g_avid & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_ring5;
    p_ring3 : process(clk)
    begin
        if rising_edge(clk) then
            w_r3q <= ring3(to_integer(w_wa - 14));
            ring3(to_integer(w_wa)) <= w_r3d;
        end if;
    end process p_ring3;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(12 downto 0);
        variable v_d  : signed(11 downto 0);
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
            -- speed: bipolar, still within +-8 of the centre
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
                s_bank  <= registers_in(6)(0);
                s_mir   <= registers_in(6)(1);
                s_night <= registers_in(6)(2);
                s_flip  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 40 + 24 k, 16 iterations, result stable from load + 17
            -- (stores at load + 18).  The slot is decoded at load - 2, the operands pre-muxed at load - 1.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 38  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 62  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(914, 36);   z_fb <= z_mg;                    -- lanes G
                    when 1  => z_fa <= to_unsigned(123, 36);   z_fb <= z_mw;                    -- 1 / width
                    when 2  => z_fa <= to_unsigned(128, 36);   z_fb <= z_mr;                    -- rows
                    when 3  => z_fa <= resize(c_s, 36);        z_fb <= to_unsigned(17280, 16);  -- 16 * 1080 s
                    when 4  => z_fa <= resize(c_g, 36);        z_fb <= to_unsigned(2839, 16);   -- lane0
                    when 5  => z_fa <= resize(s_vm, 36);       z_fb <= resize(s_vm, 16);        -- speed^2
                    when others => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= resize(s_g6(12 downto 3), 16);  -- bend
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            elsif e_ld = '1' then                      -- the line engine (outside the slots' window)
                f_acc <= (others => '0');
                f_a <= resize(e_fa, 36); f_b <= e_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps)
                    when 20 =>
                        if s_vneg = '1' then
                            c_stra <= c_stra - resize(z_inc, 23); c_strb <= c_strb - resize(z_incb, 16);
                            c_scr <= c_scr - resize(z_inc, 19);
                        else
                            c_stra <= c_stra + resize(z_inc, 23); c_strb <= c_strb + resize(z_incb, 16);
                            c_scr <= c_scr + resize(z_inc, 19);
                        end if;
                        c_xb0 <= resize(s_x0, 12);
                    -- exponents (Q8 octaves) and their mantissas
                    when 21 =>                         -- G: +-1.625 octaves (13/16)
                        v_d := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        v_e := shift_left(resize(v_d, 13), 3) + shift_left(resize(v_d, 13), 2) + resize(v_d, 13);
                        z_e <= shift_right(v_e, 4);
                    when 22 => r_ex_a <= unsigned(z_e(7 downto 0)); z_shg <= z_e(10 downto 8);
                    when 24 => z_mg <= r_ex_q;
                    when 25 =>                         -- 1 / width: 2.5 octaves up (thin), 0.94 down (wide)
                        v_d := 512 - signed(resize(s_g3(12 downto 3), 12));
                        if v_d >= 0 then
                            z_e <= shift_right(shift_left(resize(v_d, 13), 2) + resize(v_d, 13), 2);
                        else
                            v_e := shift_left(resize(v_d, 13), 4) - resize(v_d, 13);
                            z_e <= shift_right(v_e, 5);
                        end if;
                    when 26 => r_ex_a <= unsigned(z_e(7 downto 0)); z_shw <= z_e(10 downto 8);
                    when 28 => z_mw <= r_ex_q;
                    when 29 =>                         -- rows: +-4.6 octaves around 0.5 rows (37/16)
                        v_d := signed(resize(s_g5(12 downto 3), 12)) - 512;
                        v_e := shift_left(resize(v_d, 13), 5) + shift_left(resize(v_d, 13), 2) + resize(v_d, 13);
                        z_e <= shift_right(v_e, 4);
                        if s_g5(12 downto 3) < 8 then c_teeth <= '0'; else c_teeth <= '1'; end if;
                    when 30 => r_ex_a <= unsigned(z_e(7 downto 0)); z_shr <= z_e(11 downto 8);
                    when 32 => z_mr <= r_ex_q;
                    when 33 =>                         -- sweep: y scale 2^(-0.75 (k2 - 512) / 256)
                        v_d := 512 - signed(resize(s_g2(12 downto 3), 12));
                        z_e <= resize(shift_right(resize(v_d, 13) + shift_left(resize(v_d, 13), 1), 2), 13);
                    when 34 => r_ex_a <= unsigned(z_e(7 downto 0)); z_shs <= z_e(10 downto 8);
                    when 36 =>
                        case to_integer(z_shs) is
                            when -2     => c_s <= resize(r_ex_q(15 downto 7), 12);
                            when -1     => c_s <= resize(r_ex_q(15 downto 6), 12);
                            when 0      => c_s <= resize(r_ex_q(15 downto 5), 12);
                            when others => c_s <= r_ex_q(15 downto 4);
                        end case;
                    -- products
                    when 58 =>
                        case to_integer(z_shg) is
                            when -2     => c_g <= f_acc(28 downto 17);
                            when -1     => c_g <= f_acc(27 downto 16);
                            when 0      => c_g <= f_acc(26 downto 15);
                            when others => c_g <= f_acc(25 downto 14);
                        end case;
                    when 82 =>
                        case to_integer(z_shw) is
                            when -1     => c_rw <= f_acc(25 downto 16);
                            when 0      => c_rw <= f_acc(24 downto 15);
                            when 1      => c_rw <= f_acc(23 downto 14);
                            when others => c_rw <= f_acc(22 downto 13);
                        end case;
                    when 106 =>
                        case to_integer(z_shr) is
                            when -5     => c_rows <= f_acc(31 downto 20);
                            when -4     => c_rows <= f_acc(30 downto 19);
                            when -3     => c_rows <= f_acc(29 downto 18);
                            when -2     => c_rows <= f_acc(28 downto 17);
                            when -1     => c_rows <= f_acc(27 downto 16);
                            when 0      => c_rows <= f_acc(26 downto 15);
                            when 1      => c_rows <= f_acc(25 downto 14);
                            when 2      => c_rows <= f_acc(24 downto 13);
                            when 3      => c_rows <= f_acc(23 downto 12);
                            when others => c_rows <= f_acc(22 downto 11);
                        end case;
                    when 107 =>                        -- full rows and the partial row's width shift
                        c_rint <= c_rows(11 downto 8);
                        if c_rows(7 downto 6) = "11" then c_rsh <= "00"; c_rpart <= '1';
                        elsif c_rows(7 downto 5) >= 3 then c_rsh <= "01"; c_rpart <= '1';
                        elsif c_rows(7 downto 4) >= 3 then c_rsh <= "10"; c_rpart <= '1';
                        elsif c_rows(7 downto 3) >= 3 then c_rsh <= "11"; c_rpart <= '1';
                        else c_rsh <= "11"; c_rpart <= '0'; end if;
                    when 130 =>
                        c_pik <= to_unsigned(129744, 20) + resize(f_acc(26 downto 10), 20);
                        c_b0  <= to_unsigned(36544, 18) + resize(f_acc(26 downto 10), 18);
                    when 154 => c_lane0 <= f_acc(21 downto 8) + 512;
                    when 178 => z_vsq <= f_acc(17 downto 0);
                    -- stream / wipe (0.0254 vsq: 6.4 h per field at the clip's speed) and tooth phase steps
                    when 179 => z_inc  <= resize(z_vsq(17 downto 6), 15) + resize(z_vsq(17 downto 7), 15) + resize(z_vsq(17 downto 9), 15);
                    when 180 => z_incb <= resize(z_vsq(17 downto 8), 12) + resize(z_vsq(17 downto 9), 12) + resize(z_vsq(17 downto 12), 12);
                    -- tear gain 0.164 p12 (Q6), region x and y limits past the still's setting
                    when 181 => c_tg <= resize(s_gp(12 downto 6), 8) + resize(s_gp(12 downto 8), 8) + resize(s_gp(12 downto 10), 8);
                    when 182 =>
                        if s_gp(12 downto 3) > 378 then z_dt <= s_gp(12 downto 3) - 378; else z_dt <= (others => '0'); end if;
                    when 183 =>
                        c_xt   <= to_unsigned(1800, 13) + shift_left(resize(z_dt, 13), 3) - resize(z_dt(9 downto 1), 13);
                        c_ylim <= to_unsigned(800, 11) + shift_left(resize(z_dt, 11), 1) - resize(z_dt(9 downto 3), 11);
                    when 184 => c_bigth <= to_unsigned(64, 8) + resize(s_gp(12 downto 6), 8);
                    when 210 =>                        -- bend (k6^2); the Osc drives it twice as hard
                        if s_src = '0' then c_vg <= f_acc(19 downto 12);
                        elsif f_acc(19) = '1' then c_vg <= (others => '1');
                        else c_vg <= f_acc(18 downto 11); end if;
                    when others => null;
                end case;
            end if;

            -- syncs and the generated avid: 24 clocks through ring 5, then FFs for the taps
            s_hsync_sr(24) <= w_r5q(0); s_vsync_sr(24) <= w_r5q(1);
            s_field_sr(24) <= w_r5q(2); s_gav_sr(24) <= w_r5q(3);
            for i in 25 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Per-field table fill (vblank zone, from zc 330, 16 clocks per entry):
    -- GLN(m) = G ln(1 + m/256) >> 8, GE(p) = G p ln 2 >> 8 (mod 16 lanes),
    -- QAP(t) = min(4095, rW taper(t) >> 6).  One serial shift-add multiplier;
    -- the write enables are one-clock pulses.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_i : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            f_we1 <= '0'; f_we2 <= '0'; f_we3 <= '0';
            if s_zone = '1' and s_zc = 330 then
                f_run <= '1'; f_i <= (others => '0'); f_c <= (others => '0');
            elsif f_run = '1' then
                f_c <= f_c + 1;
                case to_integer(f_c) is
                    when 0 =>
                        if f_i < 256 then f_tab <= "00"; r_fs_a <= f_i;
                        elsif f_i < 275 then f_tab <= "01";
                        else f_tab <= "10"; v_i := f_i - 19; r_fs_a <= v_i; end if;
                    when 2 =>
                        fm_acc <= (others => '0');
                        case f_tab is
                            when "00"   => fm_a <= resize(r_fs_q(9 downto 0), 26);  fm_b <= c_g;
                            when "01"   => fm_a <= to_unsigned(C_PL_E(to_integer(f_i(4 downto 0))), 26); fm_b <= c_g;
                            when others => fm_a <= resize(r_fs_q(11 downto 0), 26); fm_b <= resize(c_rw, 12);
                        end case;
                    when 15 =>
                        f_wa <= f_i(7 downto 0);
                        case f_tab is
                            when "00"   => f_wd <= resize(fm_acc(20 downto 8), 16); f_we1 <= '1';
                            when "01"   => f_wd <= resize(fm_acc(21 downto 8), 16);
                                           f_wa <= resize(f_i(4 downto 0), 8); f_we2 <= '1';
                            when others =>
                                if fm_acc(25 downto 18) /= 0 then f_wd <= to_unsigned(4095, 16);
                                else f_wd <= resize(fm_acc(17 downto 6), 16); end if;
                                f_wa <= resize(f_i - 275, 8); f_we3 <= '1';
                        end case;
                        if f_i = 402 then f_run <= '0'; end if;
                        f_i <= f_i + 1;
                    when others =>
                        if f_c >= 3 then
                            if fm_b(0) = '1' then fm_acc <= fm_acc + fm_a; end if;
                            fm_a <= fm_a(24 downto 0) & '0';
                            fm_b <= '0' & fm_b(11 downto 1);
                        end if;
                end case;
            end if;
        end if;
    end process p_fill;

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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on the sequencer's serial multiplier (idle
    -- outside the zone; a request at lc L gives the product at L + 18): the
    -- sweep, the tear draw and gain, the band jitter and phase.  Two 32-bit
    -- LFSRs are stepped 32 times every other line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : signed(19 downto 0);
        variable v_s3 : unsigned(5 downto 0);
        variable v_s2 : unsigned(4 downto 0);
        variable v_a  : signed(7 downto 0);
        variable v_b  : signed(9 downto 0);
        variable v_bl : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 220 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0'); e_ln <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19); e_ln <= e_ln + 1;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            e_ld <= '0';

            -- fresh draws: 32 LFSR steps (lc 120 .. 151) on even lines
            if e_lc >= 120 and e_lc < 152 and e_ln(0) = '0' then
                if e_r1(0) = '1' then e_r1 <= ('0' & e_r1(31 downto 1)) xor x"80200003";
                else e_r1 <= '0' & e_r1(31 downto 1); end if;
                if e_r2(0) = '1' then e_r2 <= ('0' & e_r2(31 downto 1)) xor x"80000057";
                else e_r2 <= '0' & e_r2(31 downto 1); end if;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    if s_flip = '1' then
                        if e_vy < 276480 then e_yf <= to_unsigned(276480, 19) - e_vy; else e_yf <= (others => '0'); end if;
                    else
                        e_yf <= e_vy;
                    end if;
                    if e_vy(18 downto 8) > 1100 then e_ys <= to_unsigned(1100, 11); else e_ys <= e_vy(18 downto 8); end if;
                    n_ln <= e_ln;
                when 2 =>
                    -- sweep: s y (Q4) on the multiplier; y envelopes (Q4) of the tear region and band jitter
                    e_fa <= resize(e_yf(18 downto 4), 32); e_fb <= resize(c_s, 16); e_ld <= '1';
                    v_y := signed(resize(c_ylim, 20)) - signed(resize(e_ys, 20));
                    if v_y < 0 then e_ey <= (others => '0');
                    elsif v_y >= 512 then e_ey <= to_unsigned(16, 5);
                    else e_ey <= resize(unsigned(v_y(8 downto 5)), 5); end if;
                    v_y := to_signed(800, 20) - signed(resize(e_ys, 20));
                    if v_y < 64 then e_eyv <= to_unsigned(2, 5);
                    elsif v_y >= 512 then e_eyv <= to_unsigned(16, 5);
                    else e_eyv <= resize(unsigned(v_y(8 downto 5)), 5); end if;
                    -- draws: small ~ N(0, 12 h), big ~ N(0, 80 h) on a fraction of the line pairs
                    v_s3 := resize(e_r1(3 downto 0), 6) + resize(e_r1(7 downto 4), 6) + resize(e_r1(11 downto 8), 6);
                    v_a := signed(resize(v_s3, 8)) - 22;
                    e_sm <= resize(shift_right(v_a + shift_left(v_a, 1), 1), 8);
                    v_s2 := resize(e_r1(27 downto 24), 5) + resize(e_r1(31 downto 28), 5);
                    v_b := signed(resize(v_s2, 10)) - 15;
                    e_bg <= shift_left(v_b, 3) + shift_left(v_b, 2);
                    if e_r1(23 downto 16) < c_bigth then e_fl <= '1'; else e_fl <= '0'; end if;
                    v_s3 := resize(e_r2(3 downto 0), 6) + resize(e_r2(7 downto 4), 6) + resize(e_r2(11 downto 8), 6);
                    e_jr <= signed(resize(v_s3, 9)) - 22;
                    if e_r2(19 downto 12) < 90 then e_jvf <= '1'; else e_jvf <= '0'; end if;
                when 3 =>
                    if e_fl = '1' then e_raw <= resize(e_sm, 10) + e_bg; else e_raw <= resize(e_sm, 10); end if;
                    if e_jvf = '1' then e_jr <= shift_left(e_jr, 3) + shift_left(e_jr, 1);   -- x10
                    else e_jr <= shift_left(e_jr, 1); end if;                                  -- x2
                when 20 =>
                    e_ysc <= f_acc(27 downto 10);
                    e_fa <= resize(c_tg, 32); e_fb <= resize(e_ey, 16); e_ld <= '1';
                when 21 =>
                    n_zy <= c_pik - resize(e_ysc, 20);
                    e_z1 <= signed(resize(e_ysc, 19)) - signed(resize(c_b0, 19));
                when 22 =>
                    n_uby <= resize(shift_right(resize(e_z1, 21) + shift_left(resize(e_z1, 21), 1), 3), 18);
                when 38 =>
                    e_g <= f_acc(11 downto 0);
                when 39 =>
                    e_fa <= resize(unsigned(abs(e_raw)), 32); e_fb <= resize(e_g, 16); e_ld <= '1';
                when 57 =>
                    if e_raw < 0 then n_toff <= -signed(resize(f_acc(20 downto 10), 11));
                    else n_toff <= signed(resize(f_acc(20 downto 10), 11)); end if;
                    e_fa <= resize(c_tg, 32); e_fb <= resize(e_eyv, 16); e_ld <= '1';
                when 75 =>
                    e_g <= f_acc(11 downto 0);
                when 76 =>
                    e_fa <= resize(unsigned(abs(e_jr)), 32); e_fb <= resize(e_g, 16); e_ld <= '1';
                when 94 =>
                    if e_jr < 0 then e_jv <= -signed(resize(f_acc(19 downto 10), 11));
                    else e_jv <= signed(resize(f_acc(19 downto 10), 11)); end if;
                    e_fa <= resize(e_yf, 32); e_fb <= to_unsigned(328, 16); e_ld <= '1';
                when 112 =>
                    e_yb <= f_acc(26 downto 8);
                when 113 =>
                    v_bl := e_yb - c_scr;
                    n_bl <= v_bl(18 downto 8) + unsigned(e_jv);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x in h units (half virtual px), the line's constants loaded at
    -- the line start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_px <= (others => '0');
                l_toff <= n_toff; l_zy <= n_zy; l_uby <= n_uby; l_bl <= n_bl; l_ln <= n_ln;
            elsif g_avid = '1' then
                b_x  <= b_x + s_xs;
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam registers)
    ---------------------------------------------------------------------------
    w_r1d <= '0' & std_logic_vector(q4_a1);
    w_r2d <= std_logic_vector(q4_vb);
    w_r3d <= x"00" & std_logic_vector(q8_gm);

    p_pix : process(clk)
        variable v_xm  : unsigned(11 downto 0);
        variable v_dxe : signed(13 downto 0);
        variable v_tof : signed(10 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_z   : signed(21 downto 0);
        variable v_p   : unsigned(20 downto 0);
        variable v_n1  : unsigned(12 downto 0);
        variable v_n2  : unsigned(18 downto 0);
        variable v_w   : unsigned(11 downto 0);
        variable v_d   : unsigned(14 downto 0);
        variable v_u   : unsigned(19 downto 0);
        variable v_ri  : signed(7 downto 0);
        variable v_tb  : unsigned(10 downto 0);
        variable v_ta  : unsigned(10 downto 0);
        variable v_ix  : unsigned(6 downto 0);
        variable v_root, v_warm : std_logic;
        variable v_dth : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            w_wa <= w_wa + 1;
            s_tog <= not s_tog;

            -- 2: mirror; tear x envelope (full, half over the last 512 h before the region's limit, none)
            if s_mir = '1' and b_x >= 1920 then v_xm := to_unsigned(3839, 12) - b_x; else v_xm := b_x; end if;
            q2_xm <= v_xm;
            v_dxe := signed(resize(c_xt, 14)) - signed(resize(v_xm, 14));
            if v_dxe < 0 then q2_ex <= "00";
            elsif v_dxe < 512 then q2_ex <= "01";
            else q2_ex <= "10"; end if;
            q2_px <= b_px(1 downto 0);
            -- 3: torn x (h)
            case q2_ex is
                when "10"   => v_tof := l_toff;
                when "01"   => v_tof := shift_right(l_toff, 1);
                when others => v_tof := (others => '0');
            end case;
            q3_xt <= signed(resize(q2_xm, 14)) + resize(v_tof, 14);
            q3_px <= q2_px;
            -- 4: dx = x + 280 h (the root at -140 px), clamped; ground boundary address; feather segment and
            -- tooth phase bases (rings 1, 2); camera luma x gain (nibble partials)
            v_t := q3_xt + 280;
            if v_t < 64 then q4_dx <= to_unsigned(64, 13);
            else q4_dx <= unsigned(v_t(12 downto 0)); end if;
            if q3_xt < 0 then r_top_a <= (others => '0');
            elsif q3_xt >= 4096 then r_top_a <= (others => '1');
            else r_top_a <= unsigned(q3_xt(11 downto 4)); end if;
            q4_xt <= q3_xt;
            q4_a1 <= unsigned(resize(q3_xt, 15)) + c_stra(22 downto 8);
            q4_vb <= unsigned(resize(shift_left(resize(q3_xt, 16), 6) - resize(q3_xt, 16), 16)) + c_strb;
            q4_va <= unsigned(s_lb_q) * c_vg(3 downto 0);
            q4_vbl <= unsigned(s_lb_q) * c_vg(7 downto 4);
            q4_px <= q3_px;
            -- 5: leading one of dx; camera product
            q5_p1 <= f_msb13(q4_dx);
            q5_dx <= q4_dx; q5_xt <= q4_xt; q5_px <= q4_px;
            q5_vp <= resize(q4_va, 16) + shift_left(resize(q4_vbl, 16), 4);
            -- 6: mantissa -> 930 ln ROM; boundary (fabric copy of its ROM); camera terms
            v_n1 := shift_left(q5_dx, to_integer(12 - q5_p1));
            r_kln_a <= v_n1(11 downto 3);
            q6_p1 <= q5_p1;
            q6_top <= r_top_q(9 downto 0);
            q6_xt <= q5_xt; q6_px <= q5_px;
            q6_vz <= q5_vp(15 downto 2);
            q6_vd <= q5_vp(15 downto 5);
            -- 7: exponent part; band phase of this pixel (the camera keys the band), threshold, dither
            q7_ke  <= to_unsigned(C_PL_KE(to_integer(q6_p1)), 17);
            q7_dd  <= l_bl - q6_vd;
            q7_thr <= resize(q6_top, 11) + 45;
            v_dth := f_bayer(q6_px, l_ln);
            q7_dth3 <= resize(v_dth, 6) + shift_left(resize(v_dth, 6), 1);
            q7_dth5 <= resize(v_dth, 7) + shift_left(resize(v_dth, 7), 2);
            q7_xr <= q6_xt - 300;
            q7_xt <= q6_xt; q7_vz <= q6_vz;
            -- 8: ROM mantissa (fabric copy); Z's line + camera term; warm / cool / root (ring 3)
            q8_kln <= r_kln_q(13 downto 0);
            q8_ke  <= q7_ke;
            q8_zv  <= resize(l_zy, 21) + resize(q7_vz, 21);
            v_w := resize(q7_dd, 12) + resize(q7_dth3, 12);
            if v_w >= resize(q7_thr, 12) and v_w < 1216 then v_warm := '1'; else v_warm := '0'; end if;
            v_warm := v_warm xor s_night;
            if q7_xr < 0 then v_root := '1';
            elsif q7_xr < 80 and q7_xr < signed(resize(q7_dth5, 14)) then v_root := '1';
            else v_root := '0'; end if;
            if v_root = '1' then
                if q7_xt < 0 then q8_gm <= "10000000"; else q8_gm <= "10" & unsigned(q7_xt(8 downto 3)); end if;
            else
                if q7_xt < 0 then q8_gm <= '0' & v_warm & "000000";
                elsif q7_xt >= 4096 then q8_gm <= '0' & v_warm & "111111";
                else q8_gm <= '0' & v_warm & unsigned(q7_xt(11 downto 6)); end if;
            end if;
            -- 9: KL = 930 ln(dx) (Q4 lines)
            q9_kl <= resize(q8_ke, 18) + resize(q8_kln, 18);
            q9_zv <= q8_zv;
            -- 10: Z = PI - s y - KL + camera, clamped; the teeth field's x term (KL 9/64)
            v_z := signed(resize(q9_zv, 22)) - signed(resize(q9_kl, 22));
            if v_z < 64 then q10_z <= to_unsigned(64, 19);
            elsif v_z >= 524288 then q10_z <= (others => '1');
            else q10_z <= unsigned(v_z(18 downto 0)); end if;
            v_p := resize(q9_kl, 21) + shift_left(resize(q9_kl, 21), 3);
            q10_ubk <= v_p(20 downto 4) srl 2;
            -- 11: leading one of Z; teeth row coordinate (Q10)
            q11_p2 <= f_msb19(q10_z);
            q11_z  <= q10_z;
            q11_ub <= l_uby + signed(resize(q10_ubk, 18));
            -- 12: mantissa / exponent -> the per-field lane tables; row index / fraction / window
            v_n2 := shift_left(q11_z, to_integer(18 - q11_p2));
            r_gln_a <= v_n2(17 downto 10);
            r_ge_a  <= q11_p2;
            q12_kb <= unsigned(q11_ub(13 downto 10));
            q12_sb <= unsigned(q11_ub(9 downto 2));
            v_ri := -signed(resize(c_rint, 8));
            q12_win <= '0'; q12_pt <= '0';
            if c_teeth = '1' then
                if q11_ub(17 downto 10) >= v_ri then q12_win <= '1';
                elsif q11_ub(17 downto 10) = v_ri - 1 and c_rpart = '1' then q12_win <= '1'; q12_pt <= '1'; end if;
            end if;
            q13_kb <= q12_kb; q13_sb <= q12_sb; q13_win <= q12_win; q13_pt <= q12_pt;
            -- 14: fabric copies of the tables and of ring 2
            q14_gln <= r_gln_q(12 downto 0);
            q14_ge  <= r_ge_q(13 downto 0);
            q14_vb0 <= unsigned(w_r2q);
            q14_kb <= q13_kb; q14_sb <= q13_sb; q14_win <= q13_win; q14_pt <= q13_pt;
            -- 15: lane coordinate u = lane0 - G ln Z (Q10, mod 16 lanes); tooth phase
            q15_ua <= c_lane0 - q14_ge - resize(q14_gln, 14);
            q15_vb <= q14_vb0 + shift_left(to_unsigned(C_PL_OFFB(to_integer(q14_kb)), 16), 8);
            q15_sb <= q14_sb; q15_win <= q14_win; q15_pt <= q14_pt;
            -- 16: tooth ROM address; lane table
            r_qb_a <= q15_vb(15 downto 8);
            q16_sa  <= q15_ua(9 downto 2);
            q16_off <= to_unsigned(C_PL_OFFA(to_integer(q15_ua(13 downto 10))), 15);
            q16_sb <= q15_sb; q16_win <= q15_win; q16_pt <= q15_pt;
            -- 17: fabric copy of ring 1
            q17_a1 <= unsigned(w_r1q(14 downto 0));
            q17_off <= q16_off; q17_sa <= q16_sa;
            q17_sb <= q16_sb; q17_win <= q16_win; q17_pt <= q16_pt;
            -- 18: feather segment coordinate; tooth ROM (fabric copy)
            q18_a  <= q17_a1 + q17_off;
            q18_qb <= r_qb_q(11 downto 0);
            q18_sa <= q17_sa; q18_sb <= q17_sb; q18_win <= q17_win; q18_pt <= q17_pt;
            -- 19: inside the segment; tooth partials
            if q18_a < C_LB then q19_in <= '1'; else q19_in <= '0'; end if;
            q19_d1 <= q18_a;
            q19_d2 <= to_unsigned(C_LB, 15) - q18_a;
            q19_ba <= q18_sb * q18_qb(5 downto 0);
            q19_bb <= q18_sb * q18_qb(11 downto 6);
            q19_sa <= q18_sa; q19_win <= q18_win; q19_pt <= q18_pt;
            -- 20: taper table address (distance to the nearer end, 8 h per step); tooth t (Q10, saturated;
            -- the partial row's teeth narrowed by 1, 2 or 3 octaves)
            if q19_d1 < q19_d2 then v_d := q19_d1; else v_d := q19_d2; end if;
            if q19_in = '0' then r_qa_a <= (others => '0');
            elsif v_d(14 downto 10) /= 0 then r_qa_a <= (others => '1');
            else r_qa_a <= v_d(9 downto 3); end if;
            v_u := resize(q19_ba, 20) + shift_left(resize(q19_bb, 20), 6);
            if q19_pt = '1' then
                case to_integer(c_rsh) is
                    when 0      => null;
                    when 1      => v_u := v_u sll 1;
                    when 2      => v_u := v_u sll 2;
                    when others => v_u := v_u sll 3;
                end case;
            end if;
            if q19_win = '0' or v_u(19 downto 15) /= 0 then q_tb(20) <= (others => '1');
            else q_tb(20) <= v_u(14 downto 4); end if;
            q20_sa <= q19_sa; q20_in <= q19_in;
            q21_sa <= q20_sa; q21_in <= q20_in;
            -- 22: taper / width (fabric copy)
            q22_qa <= r_qa_q(11 downto 0);
            q22_sa <= q21_sa; q22_in <= q21_in;
            -- 23: feather partials
            q23_ta <= q22_sa * q22_qa(5 downto 0);
            q23_tb <= q22_sa * q22_qa(11 downto 6);
            q23_in <= q22_in;
            -- 24: feather t (Q10, saturated); ring 3 (fabric copy)
            v_u := resize(q23_ta, 20) + shift_left(resize(q23_tb, 20), 6);
            if q23_in = '0' or v_u(19 downto 15) /= 0 then q24_ta <= (others => '1');
            else q24_ta <= v_u(14 downto 4); end if;
            q24_gm <= unsigned(w_r3q(7 downto 0));
            for i in 21 to 24 loop q_tb(i) <= q_tb(i - 1); end loop;
            -- 25: teeth (solid) over feathers over teeth (fringe) over the ground; the root strip on top.  The
            -- soft trailing fringe alternates palette / ground by column (the luma [1 2 1] and the chroma boxes
            -- average it to the half blend).
            v_tb := q_tb(24); v_ta := q24_ta;
            q25_p <= '1';
            if q24_gm(7 downto 6) = "10" then
                q25_p <= '0'; v_ix := (others => '0');
            elsif v_tb < 1024 then v_ix := v_tb(10 downto 4);
            elsif v_ta < 1024 then v_ix := v_ta(10 downto 4);
            elsif v_ta < 1106 then v_ix := v_ta(10 downto 4); q25_p <= s_tog;
            elsif v_tb < 1106 then v_ix := v_tb(10 downto 4); q25_p <= s_tog;
            else q25_p <= '0'; v_ix := (others => '0'); end if;
            r_pal_a <= s_bank & v_ix;
            r_bg_a  <= q24_gm;
            -- 26: ROMs; 27: colour (palette or ground); black outside the generated avid
            q26_p <= q25_p;
            if s_gav_sr(24) = '0' then
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            elsif q26_p = '1' then
                s_y27 <= unsigned(r_pal_q(31 downto 22)); s_u27 <= unsigned(r_pal_q(21 downto 12)); s_v27 <= unsigned(r_pal_q(11 downto 2));
            else
                s_y27 <= unsigned(r_bg_q(31 downto 22)); s_u27 <= unsigned(r_bg_q(21 downto 12)); s_v27 <= unsigned(r_bg_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(24);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums), luma [1 2 1] (the HD HDMI bar fix), luma
    -- 13/16 and the blanking gate.  The running 8-sums snap back to neutral
    -- once 8 blank samples are in (a glitched sample can't offset the colours
    -- for good).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_l  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- luma [1 2 1] (stage 29), then 8 pixels back through ring 4 (+ a fabric copy): stage 31 holds the
            -- luma of the pixel the two chroma boxes are centred on
            s_yd1 <= s_y27; s_yd2 <= s_yd1;
            v_l := resize(s_y27, 12) + shift_left(resize(s_yd1, 12), 1) + resize(s_yd2, 12);
            s_y29 <= v_l(11 downto 2);
            s_y31 <= unsigned(w_r4q(9 downto 0));
            -- chroma box A: stage 28 sums, stage 29 output
            s_cau(0) <= s_u27; s_cav(0) <= s_v27;
            for i in 1 to 7 loop s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u27 - s_cau(7);
                s_bav <= s_bav + s_v27 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(26) = '1' then
                v_sc := resize(s_bau, 14) + s_baud; s_u29 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v29 <= v_sc(13 downto 4);
            else
                s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(26);
            -- chroma box B: stage 30 sums, stage 31 output
            s_cbu(0) <= s_u29; s_cbv(0) <= s_v29;
            for i in 1 to 7 loop s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bbu <= s_bbu + s_u29 - s_cbu(7);
                s_bbv <= s_bbv + s_v29 - s_cbv(7);
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u31 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v31 <= v_sc(13 downto 4);
            -- 32: luma 13/16
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
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture plumage;
