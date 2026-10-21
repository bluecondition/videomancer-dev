-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Rorschach v0.5.0 -- Heirloom #4, rebuilt from the photo WIN_20221124_22_54_05.
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
--   K2  Level    colourizer offset: the bands crawl through the cycle
--   K3  Detail   busy fragments 0 .. 2 cycles; above 60 % they spill into the bands
--   K4  Speed    bipolar colour cycling, still at the centre (+-8)
--   K5  Fold     fold line 0.25 .. 1.0 frame heights (0.487 = the photo)
--   K6  Palette  colourways of the photo palette: Rorschach / Ink (mono) / Swap /
--                Comp / Turn / Twist (chroma swaps and turns) / Night (luma
--                negative) / Vivid (double chroma)
--   S7  Mirror   Mirror / Repeat (the top half reflected, or repeated)
--   S8  Stairs   Off / On: the whole dark band sample-and-held into blocks
--   S9  Split    Hue / Blue: the dark band's hue, or all blue
--   S10 Negative Off / On: the scalar inverted (band order reversed)
--   S11 Source   Video / Osc
--   P12 TEAR     frame-store tears: 0 = clean fold .. 100 % = torn into slabs
--
-- Generated avid; output conditioning (luma [1 3 4 4 3 1]/16 smear, chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the blanking gate.  Latency 24
-- clocks in every mode.  Colours in the hardware convention (U = Cr, V = Cb).

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
    signal t_cx2m : signed(23 downto 0) := to_signed(1092 - 32768, 24);
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

    -- glides (Q.3): 0 K1 Bands, 1 K2 Level, 2 K3 Detail, 3 K5 Fold, 4 P12 Tear
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(387 * 8, 13), to_unsigned(0, 13), to_unsigned(614 * 8, 13),
                         to_unsigned(323 * 8, 13), to_unsigned(358 * 8, 13));
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
    signal q_wr : std_logic_vector(1 to 9) := (others => '0');
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
    signal f_rep, f_stair, f_blue, f_neg, f_osc : std_logic := '0';

    -- per-line engine; it runs 24 clocks after the source avid fall (the
    -- pixel path has drained) and writes the line registers (l_*) directly
    signal le_st : unsigned(3 downto 0) := (others => '0');
    signal le_dl : unsigned(4 downto 0) := (others => '0');
    signal n_vy  : unsigned(19 downto 0) := (others => '0');
    signal n_ym, n_ymt, n_yr : signed(25 downto 0) := (others => '0');
    signal n_up, n_adv, n_new : std_logic := '0';
    signal n_rpw : unsigned(25 downto 0) := (others => '0');
    signal n_d   : signed(20 downto 0) := (others => '0');
    signal n_dab : unsigned(19 downto 0) := (others => '0');
    signal n_p   : unsigned(7 downto 0) := (others => '0');
    signal n_grem : unsigned(6 downto 0) := (others => '0');
    signal n_vv  : unsigned(11 downto 0) := (others => '0');
    signal g_dy, g_dx, g_cpo : signed(23 downto 0) := (others => '0');
    signal g_hon : std_logic := '0';
    signal g_hc  : unsigned(3 downto 0) := (others => '0');
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
    signal p_cst : t_sl(1 to 13) := (others => '0');
    signal p_tk : t_sl(1 to 14) := (others => '0');
    signal p3_q00, p3_q01, p3_q10, p3_q11 : unsigned(5 downto 0) := (others => '0');
    signal p3_hh : unsigned(1 downto 0) := (others => '0');
    signal p4_na : signed(8 downto 0) := (others => '0');
    signal p5_na : signed(8 downto 0) := (others => '0');
    type t_s9a is array (natural range <>) of signed(8 downto 0);
    signal p_n : t_s9a(6 to 8) := (others => (others => '0'));
    type t_u3a is array (natural range <>) of unsigned(2 downto 0);
    signal p4_hz, p4_hm : unsigned(1 downto 0) := (others => '0');
    signal p_hue : t_u3a(5 to 13) := (others => (others => '0'));   -- dark-band hue
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
    signal p13_bd : unsigned(2 downto 0) := (others => '0');
    signal p13_dk : std_logic := '0';
    signal p14_dh : std_logic := '0';
    signal p14_sel, p14_hsel : unsigned(3 downto 0) := (others => '0');
    signal p15_c : std_logic_vector(23 downto 0) := x"108080";
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p1_cl : unsigned(5 downto 0) := (others => '0');

    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
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
    p_vh : process(clk)
    begin
        if rising_edge(clk) then
            if w_hwe = '1' then vh(to_integer(w_ha)) <= w_hd; end if;
            r_vh <= vh(to_integer(a_hue));
        end if;
    end process p_vh;


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
                if unsigned(registers_in(5)) < 8 then f_pl <= unsigned(registers_in(5)(2 downto 0));
                else f_pl <= unsigned(registers_in(5)(9 downto 7)); end if;
                f_rep   <= registers_in(6)(0);
                f_stair <= registers_in(6)(1);
                f_blue  <= registers_in(6)(2);
                f_neg   <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- colour cycle; reseed the tears and fragments every 4 fields
                if s_s4 = '1' then f_rac <= f_rac - resize(f_drm, 16);
                else f_rac <= f_rac + resize(f_drm, 16); end if;
                f_fc <= f_fc + 1;
                if f_fc = 3 then f_gseed <= f_gseed + x"9E3779B9"; end if;
            end if;
            f_rot <= f_rac + shift_left(resize(s_g(1)(12 downto 3), 16), 6);
            f_nseed <= f_gseed(15 downto 0) xor x"5A5A";
            f_amp <= s_g(2)(12 downto 4);
            if s_g(2)(12 downto 3) > 614 then s_k3x <= resize(s_g(2)(12 downto 3) - 614, 9); else s_k3x <= (others => '0'); end if;
            f_aq8 <= f_av & x"00";

            -- glides: K1, K2, K3, K5, P12 ease 1/8 of the gap per field (deadband 4)
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
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
                                when others => s_g(4) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- serial products: slot p = zc/32 (1..9): phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            -- operands are registered from the slot number (phase 0..1), loaded at phase 2,
            -- shifted at phases 3..20, stored at phase 22
            if s_zone = '1' and v_p >= 1 and v_p <= 9 and v_ph = 1 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 9 and v_ph >= 2 and v_ph <= 19 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 9 loop
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
                    when 6 => m_an <= resize(s_g(3)(12 downto 3), 20); m_bn <= to_unsigned(809, 18);
                    when 7 => m_an <= resize(v_t, 20);        m_bn <= resize(v_t, 18);
                    when 8 => m_an <= resize(f_t2, 20);       m_bn <= resize(f_t2, 18);
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
            if q_wr(6) = '1' then f_av <= to_unsigned(271, 11) + resize(v_pr(19 downto 10), 11); end if;
            if q_wr(7) = '1' then f_t2 <= v_pr(19 downto 10); end if;
            if q_wr(8) = '1' then f_t4 <= v_pr(19 downto 10); end if;
            if q_wr(9) = '1' then f_sp <= v_pr(12 downto 10); end if;   -- spill, quarters (0..1)

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
                t_cx2m <= signed(resize(t_cx2, 24)) - 32768;
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
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                n_vy <= resize(t_ys(10 downto 1), 20);
                n_ym <= signed(resize(f_kstep(17 downto 1), 26)); n_up <= '1';
                n_rpw <= resize(f_kstep(17 downto 1), 26);
                n_grem <= (others => '0');
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
                if le_st = 8 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
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
                        if f_rep = '1' then
                            if n_ymt >= C_LIM then n_ym <= n_ymt - C_LIM; else n_ym <= n_ymt; end if;
                        elsif n_up = '1' then
                            if n_ymt >= C_LIM then n_ym <= 2 * C_LIM - n_ymt; n_up <= '0'; else n_ym <= n_ymt; end if;
                        else
                            if n_ymt < 0 then n_ym <= -n_ymt; n_up <= '1'; else n_ym <= n_ymt; end if;
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
                    -- a new tear group: take the precomputed draw, then draw the next one
                    if n_new = '1' then
                        if tg_lines = 0 then n_grem <= (others => '0'); else n_grem <= tg_lines - 1; end if;
                        if tg_act < n_p then
                            g_dy <= tg_dy; g_dx <= tg_dx; g_cpo <= tg_cpo; g_hon <= tg_hf; g_hc <= tg_hc;
                        else
                            g_dy <= (others => '0'); g_dx <= (others => '0'); g_cpo <= (others => '0'); g_hon <= '0';
                        end if;
                        tg_go <= '1';
                    end if;
                when 6 =>
                    v_y := n_ym + resize(g_dy, 26);
                    if v_y < 0 then n_yr <= (others => '0');
                    elsif v_y >= C_LIM then n_yr <= to_signed(C_LIM - 1, 26);
                    else n_yr <= v_y; end if;
                    l_u0 <= t_cx2m + g_dx;
                    l_cp0 <= g_cpo - 16288;
                    l_hon <= g_hon; l_hc <= g_hc;
                when 7 =>
                    v_v := n_yr - 32768;
                    if v_v < 0 then v_v := (others => '0');
                    elsif v_v >= 127 * 65536 then v_v := to_signed(127 * 65536 - 1, 26); end if;
                    n_vv <= unsigned(v_v(22 downto 11));
                    l_hrow <= unsigned(n_yr(22 downto 17));
                    l_grp <= n_vy(17 downto 10);
                    if n_rpw(25 downto 23) = "000" then l_wok <= '1'; else l_wok <= '0'; end if;
                    l_wrow <= n_rpw(22 downto 16);
                when 8 =>
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
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_u <= l_u0; b_cp <= l_cp0; b_first <= '1';
                b_cst <= '1'; b_cell <= unsigned(l_cp0(23 downto 16));
            elsif g_avid = '1' then
                b_u <= b_u + signed(resize(t_cx, 24));
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
        variable v_uq : signed(23 downto 0);
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
        variable v_bd : unsigned(2 downto 0);
        variable v_dk : std_logic;
        variable v_h  : unsigned(1 downto 0);
        variable v_hv : unsigned(9 downto 0);
        variable v_sel : unsigned(3 downto 0);
        variable v_sy : unsigned(13 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ks : unsigned(15 downto 0);
        variable v_y8 : unsigned(7 downto 0);
        variable v_du, v_dv, v_t9 : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: column, fraction, bank and hue addresses; fragment hashes (this cell, next cell)
            v_uq := b_u;
            if v_uq < 0 then v_uq := (others => '0');
            elsif v_uq >= 63 * 65536 then v_uq := to_signed(63 * 65536 - 1, 24); end if;
            v_c := unsigned(v_uq(21 downto 16));
            a_b0 <= l_ra0 & resize(shift_right(resize(v_c, 7) + 1, 1), 5);
            a_b1 <= l_ra0 & v_c(5 downto 1);
            a_b2 <= l_ra1 & resize(shift_right(resize(v_c, 7) + 1, 1), 5);
            a_b3 <= l_ra1 & v_c(5 downto 1);
            a_hue <= l_hrow & v_c;
            p1_cp <= v_c(0); p1_fx <= unsigned(v_uq(15 downto 11));
            v_ks := f_nseed xor x"7777";
            ha_1 <= f_h1((b_cell & l_grp) xor v_ks);
            p_cst(1) <= b_cst;
            -- the tear's smear: take a new colour when the column has moved by l_hc
            if b_first = '1' or v_c - p1_cl >= l_hc then
                p_tk(1) <= '1'; p1_cl <= v_c;
            else
                p_tk(1) <= '0';
            end if;

            -- 2: (store reads) hashes
            p2_cp <= p1_cp; p2_fx <= p1_fx;
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
            -- dark-band hue: the hashed hue in busy cells (busy >= 2 quarters), else the map; Split = blue
            if f_blue = '1' then p_hue(5) <= "000";
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
            p11_pg <= v_pg; p11_no <= p10_no;

            -- 12: + fragments + colour-cycle offset
            p12_ph <= p11_pg + unsigned(p11_no) + f_rot(15 downto 8);

            -- 13: band (0 = the dark band)
            v_bd := (others => '0');
            if p12_ph >= 37 then v_bd := to_unsigned(1, 3); end if;
            if p12_ph >= 74 then v_bd := to_unsigned(2, 3); end if;
            if p12_ph >= 110 then v_bd := to_unsigned(3, 3); end if;
            if p12_ph >= 147 then v_bd := to_unsigned(4, 3); end if;
            if p12_ph >= 183 then v_bd := to_unsigned(5, 3); end if;
            if p12_ph >= 220 then v_bd := to_unsigned(6, 3); end if;
            p13_bd <= v_bd;
            if v_bd = 0 then p13_dk <= '1'; else p13_dk <= '0'; end if;

            -- 14: Stairs (dark band held per cell), dark-band hue, palette index, the tear's smear
            if p_cst(13) = '1' then p14_dh <= p13_dk; end if;
            v_dk := p13_dk or (f_stair and p14_dh and not p_cst(13));
            v_h := p_hue(13)(1 downto 0);
            if v_dk = '1' then v_sel := "10" & v_h; else v_sel := '0' & (p13_bd - 1); end if;
            if l_hon = '1' and p_tk(13) = '0' then
                p14_sel <= p14_hsel;
            else
                p14_sel <= v_sel; p14_hsel <= v_sel;
            end if;

            -- 15: palette
            p15_c <= f_pal(p14_sel);

            -- 16: 10-bit
            -- colourway (K6): chroma swaps / turns / complement, mono, luma negative, vivid
            v_y8 := unsigned(p15_c(23 downto 16));
            v_du := signed(resize(unsigned(p15_c(15 downto 8)), 10)) - 128;
            v_dv := signed(resize(unsigned(p15_c(7 downto 0)), 10)) - 128;
            case to_integer(f_pl) is
                when 1 => v_du := (others => '0'); v_dv := (others => '0');
                when 2 => v_t9 := v_du; v_du := v_dv; v_dv := v_t9;
                when 3 => v_du := -v_du; v_dv := -v_dv;
                when 4 => v_t9 := v_du; v_du := -v_dv; v_dv := v_t9;
                when 5 => v_t9 := v_du; v_du := v_dv; v_dv := -v_t9;
                when 6 => v_y8 := 251 - v_y8;
                when 7 => v_du := shift_left(v_du, 1); v_dv := shift_left(v_dv, 1);
                when others => null;
            end case;
            if v_du > 127 then v_du := to_signed(127, 10); elsif v_du < -127 then v_du := to_signed(-127, 10); end if;
            if v_dv > 127 then v_dv := to_signed(127, 10); elsif v_dv < -127 then v_dv := to_signed(-127, 10); end if;
            v_du := v_du + 128; v_dv := v_dv + 128;
            p16_y <= v_y8 & "00";
            p16_u <= unsigned(v_du(7 downto 0)) & "00";
            p16_v <= unsigned(v_dv(7 downto 0)) & "00";

            -- 17..22: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums; luma [1 3 4 4 3 1]/16 = box4 smear * [1 2 1])
            s_csu(0) <= p16_u; s_csv(0) <= p16_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + p16_u - s_csu(7);
            s_boxv <= s_boxv + p16_v - s_csv(7);
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
            if s_gav_sr(21) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
            -- 24
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;

            -- delays
            for i in 3 to 13 loop p_cst(i) <= p_cst(i - 1); end loop;
            for i in 2 to 13 loop p_tk(i) <= p_tk(i - 1); end loop;
            for i in 7 to 8 loop p_n(i) <= p_n(i - 1); end loop;
            for i in 6 to 13 loop p_hue(i) <= p_hue(i - 1); end loop;
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
