-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Idol v0.5.0 -- Heirloom #31, rebuilt from the photo WIN_20230515_23_14_37.
--
-- The photo: a camera on a small idol figure (hat, eyes, an open "O" mouth -- it shows plainly in
-- the blue channel, twice, 324 px apart), folded top-to-bottom about row 518.5 by a frame-store
-- mirror (no left/right mirror), through a cyclic colourizer -- yellow, orange, peach, cream, pale,
-- grey, light blue, blue, navy, dark brown, dark red, red, orange, back to yellow -- with the dark
-- camera background keyed to a flat saturated yellow ground.  The picture is smeared sideways
-- (chroma wider than luma: whitish fringes where the yellow meets blue), columns between the
-- figures are torn into per-line streaks (an interlace comb of the noisy camera, every other line
-- anti-correlated), and a few horizontal slices are displaced (frame-store slips).
--
-- The program: a 96 x 128 store of the photo's top half (Osc: a ROM of 4-bit colourizer phases fitted
-- to the photo, code 15 = the yellow key; Video: the camera's luma, 64 x 128 x 4 bits, written live),
-- read back folded about the mirror line with a wrap-aware bilinear interpolation (four banks by row
-- and column parity), plus per-line streak noise (knots every 16 px, a fresh draw every 2 virtual
-- rows, half shared with the line pair at opposite sign) where the busy map says so, times the
-- colourizer gain about a pivot, plus the hue offset, into the 256-entry palette; the key window
-- flattens the ground to yellow.  Then box smears (luma N/4, chroma N, powers of two) from 256-entry
-- rings.  Geometry comes from the measured raster (serial divisions in the vblank zone), so every mode
-- shows the whole picture across the width.
--
--   K1  Feather  the streaks: none .. the photo (40 %) .. the whole idol torn into feathers
--   K2  Hue      colourizer phase (0 = the photo); the ground stays yellow
--   K3  Axis     mirror line 30 .. 70 % of the height (48 % = the photo)
--   K4  Speed    bipolar colour cycling, still at the centre (+-8)
--   K5  Smear    horizontal smear, chroma box 1 .. 128 px (64 = the photo), luma a quarter of it
--   K6  Ground   the yellow key window: none .. the photo .. yellow floods the light bands
--   S7  Mirror   Fold / Off (Osc: the top half repeats; Video: the plain camera)
--   S8  Twin     left/right mirror too
--   S9  Slips    frame-store slips on / off
--   S10 Negative colourizer input inverted (band order reversed; Video: bright -> yellow)
--   S11 Source   Video / Osc
--   P12 BANDS    colourizer gain 0.5 .. 1 (the photo, 50 %) .. 4 cycles: dense rings at 100 %
--
-- Generated avid; output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777)
-- before the blanking gate.  Latency 28 clocks in every mode.  Colours in the hardware convention
-- (U = Cr, V = Cb), see idol.py.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.idol_pkg.all;
use work.all;

architecture idol of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks
    constant C_PIV : integer := 1782; -- Osc gain pivot (red / orange step), 2048 = one cycle

    type t_sl is array (natural range <>) of std_logic;
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    type t_u11a is array (natural range <>) of unsigned(10 downto 0);
    type t_s13a is array (natural range <>) of signed(12 downto 0);

    function f_glide(g : unsigned(9 downto 0); d : signed(10 downto 0)) return unsigned is
        variable r : signed(10 downto 0);
    begin
        if d >= 8 or d <= -8 then
            r := signed('0' & g) + shift_right(d, 3);
        elsif d >= 4 then
            r := signed('0' & g) + 1;
        elsif d <= -4 then
            r := signed('0' & g) - 1;
        else
            r := signed('0' & g);
        end if;
        return unsigned(r(9 downto 0));
    end function;
    -- 16-bit xorshift (7, 9, 8)
    function f_x16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x xor shift_left(x, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        return v;
    end function;
    function f_x32(x : unsigned(31 downto 0)) return unsigned is
        variable v : unsigned(31 downto 0);
    begin
        v := x xor shift_left(x, 13);
        v := v xor shift_right(v, 17);
        v := v xor shift_left(v, 5);
        return v;
    end function;
    -- knot value: (own + / - shared) / 2, both signed 8-bit
    function f_knot(o, s : unsigned(7 downto 0); neg : std_logic) return signed is
        variable v : signed(8 downto 0);
    begin
        if neg = '1' then
            v := resize(signed(o), 9) - resize(signed(s), 9);
        else
            v := resize(signed(o), 9) + resize(signed(s), 9);
        end if;
        return v(8 downto 1);
    end function;
    ---------------------------------------------------------------------------
    -- timing, zone, controls
    ---------------------------------------------------------------------------
    signal t_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal t_cs, t_cv, t_kn : unsigned(15 downto 0) := (others => '0');
    signal t_wv : unsigned(16 downto 0) := (others => '0');
    signal t_sm : std_logic := '0';
    signal t_hd : std_logic := '1';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone : std_logic := '0';
    signal s_zc : unsigned(10 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w2 : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_hl : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_lcnt : unsigned(10 downto 0) := (others => '0');
    signal d_n : unsigned(22 downto 0) := (others => '0');
    signal d_r : unsigned(12 downto 0) := (others => '0');
    signal d_v : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal t_q : unsigned(14 downto 0) := to_unsigned(2185, 15);
    signal s_brst : std_logic := '0';
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    -- glides: K1 Feather, K2 Hue, K3 Axis, K6 Ground, P12 Bands
    signal s_g1 : unsigned(9 downto 0) := to_unsigned(409, 10);
    signal s_g2 : unsigned(9 downto 0) := (others => '0');
    signal s_g3 : unsigned(9 downto 0) := to_unsigned(463, 10);
    signal s_g6 : unsigned(9 downto 0) := to_unsigned(307, 10);
    signal s_gp : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_gl_g, s_gl_r : unsigned(9 downto 0) := (others => '0');
    signal s_gl_t : std_logic_vector(9 downto 0) := (others => '0');
    signal s_gl_d : signed(10 downto 0) := (others => '0');
    -- per-field values
    signal f_osc, f_mir, f_twin, f_slips, f_neg : std_logic := '0';
    signal f_sp : signed(10 downto 0) := (others => '0');
    signal f_acc : unsigned(15 downto 0) := (others => '0');
    signal f_ph : unsigned(10 downto 0) := (others => '0');
    signal f_g : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal f_w : unsigned(9 downto 0) := to_unsigned(61, 10);
    signal f_axis, f_ax2 : unsigned(19 downto 0) := to_unsigned(520 * 256, 20);
    signal f_ta : unsigned(19 downto 0) := (others => '0');
    signal f_F : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_Fm : unsigned(9 downto 0) := (others => '0');
    signal m_a : unsigned(15 downto 0) := (others => '0');
    signal m_b : unsigned(7 downto 0) := (others => '0');
    signal m_p : unsigned(23 downto 0) := (others => '0');
    signal m_add : unsigned(9 downto 0) := (others => '0');
    signal f_amp1, f_amp2, f_amp3, f_amp0 : unsigned(9 downto 0) := (others => '0');
    signal f_kc, f_ky : unsigned(2 downto 0) := (others => '0');
    signal f_nc, f_ny : unsigned(7 downto 0) := to_unsigned(1, 8);
    signal s_bfc, s_bfy : std_logic := '0';
    signal f_cstep : unsigned(15 downto 0) := (others => '0');
    signal f_cmax : unsigned(23 downto 0) := to_unsigned(95 * 65536 - 1, 24);
    signal f_c0 : signed(23 downto 0) := to_signed(-32768, 24);
    signal f_hoff : unsigned(22 downto 0) := (others => '0');
    signal f_lo, f_hi, f_2lo, f_2hi : signed(20 downto 0) := (others => '0');
    signal f_K : unsigned(10 downto 0) := to_unsigned(1872, 11);
    signal f_fseed : unsigned(15 downto 0) := x"ACE1";
    signal f_sseed : unsigned(15 downto 0) := x"9E37";
    signal f_fc : unsigned(2 downto 0) := (others => '0');

    -- line engine (computes the next line's l_* registers in the horizontal blanking)
    signal le_st : unsigned(5 downto 0) := (others => '0');
    signal le_dl : unsigned(5 downto 0) := (others => '0');
    signal n_vy : unsigned(19 downto 0) := (others => '0');
    signal n_v : signed(20 downto 0) := (others => '0');
    signal n_m : unsigned(18 downto 0) := (others => '0');
    signal n_kb : unsigned(10 downto 0) := (others => '0');
    signal n_pr : unsigned(29 downto 0) := (others => '0');
    signal n_fy : signed(14 downto 0) := (others => '0');
    signal n_wacc : unsigned(22 downto 0) := (others => '0');
    signal n_wrow : unsigned(6 downto 0) := (others => '0');
    signal n_ho, n_hs : unsigned(15 downto 0) := (others => '0');
    signal n_slr : unsigned(15 downto 0) := x"9E37";
    signal n_srem : unsigned(4 downto 0) := (others => '0');
    signal n_sdy : signed(20 downto 0) := (others => '0');
    signal n_sdx : signed(23 downto 0) := (others => '0');
    signal l_j0 : unsigned(6 downto 0) := (others => '0');
    signal l_ty : unsigned(4 downto 0) := (others => '0');
    signal l_jp : std_logic := '0';
    signal l_rb0, l_rb1 : unsigned(5 downto 0) := (others => '0');
    signal l_bbase : unsigned(10 downto 0) := (others => '0');
    signal l_c0 : signed(23 downto 0) := to_signed(-32768, 24);
    signal l_so, l_ss : unsigned(15 downto 0) := x"ACE1";
    signal l_kp, l_kn : signed(7 downto 0) := (others => '0');
    signal l_sg, l_go : std_logic := '0';
    signal l_wrow : unsigned(6 downto 0) := (others => '0');
    signal l_wen : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- beam
    signal b_col : signed(23 downto 0) := to_signed(-32768, 24);
    signal b_kn : unsigned(22 downto 0) := (others => '0');
    signal b_kev, b_rev : std_logic := '0';
    signal b_lfo, b_lfs : unsigned(15 downto 0) := x"ACE1";
    signal b_kp, b_kq : signed(7 downto 0) := (others => '0');
    signal b_t : unsigned(3 downto 0) := (others => '0');
    signal b_pd : unsigned(1 downto 0) := (others => '0');

    -- stores
    signal a_o00, a_o01, a_o10, a_o11 : unsigned(9 downto 0) := (others => '0');
    signal a_v00, a_v01, a_v10, a_v11 : unsigned(10 downto 0) := (others => '0');
    signal a_bz : unsigned(10 downto 0) := (others => '0');
    signal r_s000, r_s001, r_s002, r_s010, r_s011, r_s012 : std_logic_vector(3 downto 0) := (others => '0');
    signal r_s100, r_s101, r_s102, r_s110, r_s111, r_s112 : std_logic_vector(3 downto 0) := (others => '0');
    signal r_bz : std_logic_vector(1 downto 0) := (others => '0');
    type t_vram is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal vr00, vr01, vr10, vr11 : t_vram := (others => (others => '0'));
    signal r_v00, r_v01, r_v10, r_v11 : std_logic_vector(3 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of vr00 : signal is true;
    attribute no_rw_check of vr01 : signal is true;
    attribute no_rw_check of vr10 : signal is true;
    attribute no_rw_check of vr11 : signal is true;
    signal a_pal : unsigned(7 downto 0) := (others => '0');
    signal r_pyu : std_logic_vector(15 downto 0) := (others => '0');
    signal r_pv : std_logic_vector(7 downto 0) := (others => '0');
    -- smear ring
    signal s_wp : unsigned(7 downto 0) := (others => '0');
    type t_ringc is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_ringy is array (0 to 255) of std_logic_vector(7 downto 0);
    signal ringc : t_ringc := (others => (others => '0'));
    signal ringy : t_ringy := (others => (others => '0'));
    attribute no_rw_check of ringc : signal is true;
    attribute no_rw_check of ringy : signal is true;
    signal r_rg, r_rgf : std_logic_vector(15 downto 0) := (others => '0');
    signal r_ry, r_ryf : std_logic_vector(7 downto 0) := (others => '0');

    -- video write side
    signal w_yf : unsigned(11 downto 0) := (others => '0');
    signal w_d1 : unsigned(10 downto 0) := (others => '0');
    signal w_l2 : unsigned(7 downto 0) := (others => '0');
    signal w_a1, w_a2, w_a3 : std_logic := '0';
    signal w_col : signed(23 downto 0) := (others => '0');
    signal w_ev : std_logic := '0';
    signal w_k : unsigned(5 downto 0) := (others => '0');
    signal w_d : std_logic_vector(3 downto 0) := (others => '0');
    signal w_a : unsigned(10 downto 0) := (others => '0');
    signal w_we : std_logic_vector(3 downto 0) := (others => '0');

    -- pixel path
    signal p1_tx : unsigned(4 downto 0) := (others => '0');
    signal p1_ip : std_logic := '0';
    signal p1_be, p1_bo : unsigned(1 downto 0) := (others => '0');
    signal p2_be, p2_bo, p3_be, p3_bo : unsigned(1 downto 0) := (others => '0');
    signal p_tx : t_u8a(2 to 5) := (others => (others => '0'));
    signal p_ip : t_sl(2 to 4) := (others => '0');
    signal p3_x00, p3_x01, p3_x10, p3_x11 : unsigned(3 downto 0) := (others => '0');
    signal p3_bz : unsigned(1 downto 0) := (others => '0');
    signal p4_a, p4_b, p4_c, p4_d : unsigned(7 downto 0) := (others => '0');
    signal p4_ca, p4_cb, p4_cc : unsigned(3 downto 0) := (others => '0');
    signal p4_ka, p4_kb, p4_kc, p4_kd : std_logic := '0';
    signal p4_bz : unsigned(1 downto 0) := (others => '0');
    signal p5_a : unsigned(7 downto 0) := (others => '0');
    signal p5_db, p5_dc : signed(8 downto 0) := (others => '0');
    signal p5_ddc : signed(9 downto 0) := (others => '0');
    signal p5_nk : std_logic := '0';
    signal p5_vb : unsigned(1 downto 0) := (others => '0');
    signal p5_bz : unsigned(1 downto 0) := (others => '0');
    signal p6_a32 : signed(13 downto 0) := (others => '0');
    signal p6_dc32 : signed(13 downto 0) := (others => '0');
    signal p6_m1 : signed(13 downto 0) := (others => '0');
    signal p6_m2 : signed(14 downto 0) := (others => '0');
    signal p7_top, p7_bot : signed(16 downto 0) := (others => '0');
    signal p8_top : signed(16 downto 0) := (others => '0');
    signal p8_dtb : signed(17 downto 0) := (others => '0');
    signal p9_top : signed(16 downto 0) := (others => '0');
    signal p9_m3 : signed(22 downto 0) := (others => '0');
    signal p10_ph : unsigned(10 downto 0) := (others => '0');
    signal p_nk : t_sl(6 to 16) := (others => '0');
    -- noise
    signal q2_kp : signed(7 downto 0) := (others => '0');
    signal q2_dk : signed(8 downto 0) := (others => '0');
    signal q2_t : unsigned(3 downto 0) := (others => '0');
    signal q3_kp : signed(7 downto 0) := (others => '0');
    signal q3_m : signed(13 downto 0) := (others => '0');
    signal q4_n : signed(8 downto 0) := (others => '0');
    signal q5_n, q6_n : signed(8 downto 0) := (others => '0');
    signal q6_amp : unsigned(9 downto 0) := (others => '0');
    signal q7_no : signed(19 downto 0) := (others => '0');
    signal p_no : t_s13a(8 to 10) := (others => (others => '0'));
    -- phase, key, gain
    signal p11_sn : unsigned(10 downto 0) := (others => '0');
    signal p12_ab : unsigned(10 downto 0) := (others => '0');
    signal p12_D : signed(11 downto 0) := (others => '0');
    signal p13_pr : signed(21 downto 0) := (others => '0');
    signal p13_key : std_logic := '0';
    signal p14_idx : unsigned(10 downto 0) := (others => '0');
    signal p_key : t_sl(14 to 17) := (others => '0');
    signal p17_y, p17_u, p17_v : unsigned(7 downto 0) := (others => '0');
    signal c18_y, c18_u, c18_v : unsigned(7 downto 0) := (others => '0');
    signal c19_y, c19_u, c19_v : unsigned(7 downto 0) := (others => '0');
    signal c20_y, c20_u, c20_v : unsigned(7 downto 0) := (others => '0');
    signal s_sy, s_su, s_sv : unsigned(14 downto 0) := (others => '0');
    signal s20_y, s20_u, s20_v : unsigned(7 downto 0) := (others => '0');
    signal s21_y, s21_u, s21_v : unsigned(7 downto 0) := (others => '0');
    -- output conditioning
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u8a(0 to 8) := (others => to_unsigned(128, 8));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y22, s_y23, s_y24 : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_yf, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y26, s_y27, s_u26, s_v26, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yd : t_u10a(0 to 1) := (others => to_unsigned(64, 10));
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Stores: the scene ROM (Osc; 4 banks x 3 blocks of 1024 x 4), the busy map, the camera store
    -- (Video; 4 banks of 2048 x 4, written by the source side), the palette, the smear ring
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_s000 <= C_SC000(to_integer(a_o00)); r_s001 <= C_SC001(to_integer(a_o00)); r_s002 <= C_SC002(to_integer(a_o00));
            r_s010 <= C_SC010(to_integer(a_o01)); r_s011 <= C_SC011(to_integer(a_o01)); r_s012 <= C_SC012(to_integer(a_o01));
            r_s100 <= C_SC100(to_integer(a_o10)); r_s101 <= C_SC101(to_integer(a_o10)); r_s102 <= C_SC102(to_integer(a_o10));
            r_s110 <= C_SC110(to_integer(a_o11)); r_s111 <= C_SC111(to_integer(a_o11)); r_s112 <= C_SC112(to_integer(a_o11));
            r_bz <= C_BUSY(to_integer(a_bz));
            r_pyu <= C_PYU(to_integer(a_pal));
            r_pv <= C_PV(to_integer(a_pal));
        end if;
    end process p_rom;

    p_vr00 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(0) = '1' then vr00(to_integer(w_a)) <= w_d; end if;
            r_v00 <= vr00(to_integer(a_v00));
        end if;
    end process p_vr00;
    p_vr01 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(1) = '1' then vr01(to_integer(w_a)) <= w_d; end if;
            r_v01 <= vr01(to_integer(a_v01));
        end if;
    end process p_vr01;
    p_vr10 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(2) = '1' then vr10(to_integer(w_a)) <= w_d; end if;
            r_v10 <= vr10(to_integer(a_v10));
        end if;
    end process p_vr10;
    p_vr11 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(3) = '1' then vr11(to_integer(w_a)) <= w_d; end if;
            r_v11 <= vr11(to_integer(a_v11));
        end if;
    end process p_vr11;

    -- smear ring: written every clock with the composed colour; read N - 2 ahead so x[t - N] arrives at t
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_wp <= s_wp + 1;
            ringc(to_integer(s_wp)) <= std_logic_vector(c18_u) & std_logic_vector(c18_v);
            r_rg <= ringc(to_integer(s_wp + 2 - f_nc));
            r_rgf <= r_rg;
        end if;
    end process p_ring;
    p_ringy : process(clk)
    begin
        if rising_edge(clk) then
            ringy(to_integer(s_wp)) <= std_logic_vector(c18_y);
            r_ry <= ringy(to_integer(s_wp + 2 - f_ny));
            r_ryf <= r_ry;
        end if;
    end process p_ringy;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field values (one small op per zone clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_r : unsigned(12 downto 0);
        variable v_sl : integer range -1 to 7;
        variable v_ph : integer range 0 to 15;
        variable v_i : unsigned(4 downto 0);
        variable v_h : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
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
                if s_lcnt /= 2047 then s_lcnt <= s_lcnt + 1; end if;
            end if;
            g_wm1 <= s_w - 1;

            -- geometry from the measured raster (works at every mode and in decimated sims):
            -- q = 2^22 / W (zc 64 .. 87), ys = 1080 * 256 / lines (zc 96 .. 119)
            if s_zone = '1' and (s_zc = 64 or s_zc = 96) then
                d_r <= (others => '0');
                if s_zc = 64 then
                    d_n <= to_unsigned(4194304, 23);
                    if s_w < 360 then d_v <= to_unsigned(1920, 12); else d_v <= s_w; end if;
                else
                    d_n <= to_unsigned(276480, 23);
                    if s_hl < 200 then d_v <= to_unsigned(1080, 12); else d_v <= resize(s_hl, 12); end if;
                end if;
            elsif s_zone = '1' and ((s_zc > 64 and s_zc <= 87) or (s_zc > 96 and s_zc <= 119)) then
                v_r := d_r(11 downto 0) & d_n(22);
                if v_r >= resize(d_v, 13) then
                    d_r <= v_r - resize(d_v, 13); d_n <= d_n(21 downto 0) & '1';
                else
                    d_r <= v_r; d_n <= d_n(21 downto 0) & '0';
                end if;
            elsif s_zone = '1' and s_zc = 90 then
                t_q <= d_n(14 downto 0);
            elsif s_zone = '1' and s_zc = 122 then
                t_ys <= d_n(11 downto 0);
                t_cs <= resize(t_q, 16) + resize(t_q(14 downto 1), 16);       -- 96 / W
                t_cv <= resize(t_q, 16);                                       -- 64 / W
                t_kn <= shift_left(resize(t_q, 16), 1) - resize(t_q(14 downto 3), 16);   -- 120 / W
                if s_w >= 1600 then t_sm <= '0'; else t_sm <= '1'; end if;
                if s_w >= 1100 then t_hd <= '1'; else t_hd <= '0'; end if;
            elsif s_zone = '1' and s_zc = 123 then
                -- video-store rows per line (Q16): ys x 30.34 (65536 / 8.4375 / 256)
                t_wv <= shift_left(resize(t_ys, 17), 5) - shift_left(resize(t_ys, 17), 1)
                        + resize(t_ys(11 downto 2), 17) + resize(t_ys(11 downto 4), 17) + resize(t_ys(11 downto 5), 17);
            elsif s_zone = '1' and s_zc = 124 then
                -- K5 Smear: chroma box 2^kc (1 .. 128; the photo 64), luma 2^(kc - 2); half at 720p / SD
                v_i := resize(unsigned(registers_in(4)(9 downto 7)), 5);
                if t_sm = '1' and v_i /= 0 then v_i := v_i - 1; end if;
                f_kc <= v_i(2 downto 0);
                if v_i >= 2 then v_i := v_i - 2; else v_i := (others => '0'); end if;
                f_ky <= v_i(2 downto 0);
            elsif s_zone = '1' and s_zc = 125 then
                f_nc <= shift_left(to_unsigned(1, 8), to_integer(f_kc));
                f_ny <= shift_left(to_unsigned(1, 8), to_integer(f_ky));
            end if;
            -- smear integrators: zeroed at zc 600, then N clocks with the leaving sample forced to 0
            -- (the pipeline input is constant in the zone), so each sum restarts at exactly N x
            if s_zone = '1' and s_zc = 600 then s_brst <= '1'; else s_brst <= '0'; end if;
            if s_zone = '1' and s_zc > 600 and s_zc <= 600 + resize(f_nc, 11) then s_bfc <= '1'; else s_bfc <= '0'; end if;
            if s_zone = '1' and s_zc > 600 and s_zc <= 600 + resize(f_ny, 11) then s_bfy <= '1'; else s_bfy <= '0'; end if;

            if s_zone = '1' and s_zc = 1 then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_hl <= s_lcnt; s_lcnt <= (others => '0');
                f_mir   <= not registers_in(6)(0);
                f_twin  <= registers_in(6)(1);
                f_slips <= not registers_in(6)(2);
                f_neg   <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- K4 Speed: bipolar, deadband 8
                v_s := signed('0' & unsigned(registers_in(3))) - 512;
                if v_s > 8 then f_sp <= v_s - 8;
                elsif v_s < -8 then f_sp <= v_s + 8;
                else f_sp <= (others => '0'); end if;
                f_fc <= f_fc + 1;
                f_fseed <= f_x16(f_fseed);
                if f_fc = 7 then f_sseed <= f_fseed or x"0001"; end if;
            end if;
            -- zone clocks 4 .. 23: glides (load, difference, ease, store) on one shared datapath
            if s_zone = '1' and s_zc(10 downto 5) = 0 and s_zc(4 downto 2) /= 0 and s_zc(4 downto 2) /= 6
               and s_zc(4 downto 2) /= 7 then
                case to_integer(s_zc(1 downto 0)) is
                    when 0 =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_gl_g <= s_g1; s_gl_t <= registers_in(0);
                            when 2      => s_gl_g <= s_g2; s_gl_t <= registers_in(1);
                            when 3      => s_gl_g <= s_g3; s_gl_t <= registers_in(2);
                            when 4      => s_gl_g <= s_g6; s_gl_t <= registers_in(5);
                            when others => s_gl_g <= s_gp; s_gl_t <= registers_in(7);
                        end case;
                    when 1 =>
                        s_gl_d <= signed('0' & unsigned(s_gl_t)) - signed('0' & s_gl_g);
                    when 2 =>
                        if s_warm = 1 then s_gl_r <= unsigned(s_gl_t); else s_gl_r <= f_glide(s_gl_g, s_gl_d); end if;
                    when others =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_g1 <= s_gl_r;
                            when 2      => s_g2 <= s_gl_r;
                            when 3      => s_g3 <= s_gl_r;
                            when 4      => s_g6 <= s_gl_r;
                            when others => s_gp <= s_gl_r;
                        end case;
                end case;
            end if;

            -- zone clocks 132 .. 139: derived values
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 132 =>
                        -- feather F = 2.5 K1 (Q8: 256 = the photo); axis = 324 + 432 K3 (virtual rows, Q8)
                        f_F <= resize(shift_right(s_g1, 1) + shift_right(s_g1, 3), 10);
                        f_ta <= shift_left(resize(s_g3, 20), 6) + shift_left(resize(s_g3, 20), 5);
                        f_acc <= unsigned(signed(f_acc) + resize(f_sp, 16));
                        if f_osc = '1' then
                            f_cstep <= t_cs; f_cmax <= to_unsigned(95 * 65536 - 1, 24);
                            f_lo <= to_signed(-40 * 256, 21); f_hi <= to_signed(520 * 256, 21);
                            f_K <= to_unsigned(1872, 11);
                        else
                            f_cstep <= t_cv; f_cmax <= to_unsigned(63 * 65536 - 1, 24);
                            f_lo <= (others => '0'); f_hi <= to_signed(1080 * 256, 21);
                            f_K <= to_unsigned(971, 11);
                        end if;
                    when 133 =>
                        f_ta <= f_ta + shift_left(resize(s_g3, 20), 3) + shift_left(resize(s_g3, 20), 2);
                        if f_F > 256 then f_Fm <= f_F - 256; else f_Fm <= (others => '0'); end if;
                        f_2lo <= shift_left(f_lo, 1); f_2hi <= shift_left(f_hi, 1);
                    when 134 =>
                        f_axis <= f_ta + to_unsigned(324 * 256, 20);
                        -- P12 Bands: gain (Q6) 32 .. 64 .. 256
                        if s_gp <= 512 then
                            f_g <= to_unsigned(32, 9) + resize(s_gp(9 downto 4), 9);
                        else
                            f_g <= to_unsigned(64, 9) + resize(shift_right(resize(s_gp - 512, 12) + shift_left(resize(s_gp - 512, 12), 1), 3), 9);
                        end if;
                    when 135 =>
                        f_ax2 <= shift_left(f_axis, 1);
                        -- K6 Ground: key window (1/2048 cycle)
                        if s_g6 <= 307 then
                            f_w <= resize(shift_right(s_g6, 3) + shift_right(s_g6, 4) + shift_right(s_g6, 6), 10);
                        else
                            f_w <= 61 + resize(shift_right(s_g6 - 307, 1) + shift_right(s_g6 - 307, 3)
                                     + shift_right(s_g6 - 307, 4) + shift_right(s_g6 - 307, 5), 10);
                        end if;
                    when 136 =>
                        -- hue: K2 (two units per knob step) + the cycle, plus the gain pivot
                        v_h := shift_left(resize(s_g2, 11), 1) + f_acc(15 downto 5);
                        if f_osc = '1' then f_ph <= v_h + to_unsigned(C_PIV, 11); else f_ph <= v_h; end if;
                        s_w2 <= shift_right(s_w, 1);
                    when others => null;
                end case;
            end if;
            -- zone clocks 144 .. 223: one serial multiplier, 16 clocks per product
            --   slots 0..3: streak amplitude per busy level (F <= 256: A F / 256; above: A + M (F - 256) / 256,
            --   A = 0 61 133 225, M = 205 205 205 225); slot 4: the luma centring offset (N_y / 2) x step
            if s_zone = '1' and s_zc >= 144 and s_zc < 224 then
                v_sl := to_integer(s_zc(6 downto 4)) - 1;      -- 144 = slot 0 (zc(6..4) = 1 .. 5)
                v_ph := to_integer(s_zc(3 downto 0));
                if v_ph = 0 then
                    m_p <= (others => '0');
                    if v_sl = 4 then
                        m_a <= f_cstep; m_b <= shift_right(f_ny, 1); m_add <= (others => '0');
                    elsif f_F <= 256 then
                        m_a <= resize(f_F, 16); m_add <= (others => '0');
                        case v_sl is
                            when 0 => m_b <= to_unsigned(0, 8);
                            when 1 => m_b <= to_unsigned(61, 8);
                            when 2 => m_b <= to_unsigned(133, 8);
                            when others => m_b <= to_unsigned(225, 8);
                        end case;
                    else
                        m_a <= resize(f_Fm, 16);
                        case v_sl is
                            when 0 => m_b <= to_unsigned(205, 8); m_add <= to_unsigned(0, 10);
                            when 1 => m_b <= to_unsigned(205, 8); m_add <= to_unsigned(61, 10);
                            when 2 => m_b <= to_unsigned(205, 8); m_add <= to_unsigned(133, 10);
                            when others => m_b <= to_unsigned(225, 8); m_add <= to_unsigned(225, 10);
                        end case;
                    end if;
                elsif v_ph <= 8 then
                    if m_b(7) = '1' then m_p <= shift_left(m_p, 1) + resize(m_a, 24); else m_p <= shift_left(m_p, 1); end if;
                    m_b <= shift_left(m_b, 1);
                elsif v_ph = 9 then
                    case v_sl is
                        when 0 => f_amp0 <= m_add + resize(m_p(17 downto 8), 10);
                        when 1 => f_amp1 <= m_add + resize(m_p(17 downto 8), 10);
                        when 2 => f_amp2 <= m_add + resize(m_p(17 downto 8), 10);
                        when 3 => f_amp3 <= m_add + resize(m_p(17 downto 8), 10);
                        when others => f_c0 <= signed(resize(m_p, 24)) - 32768;
                    end case;
                end if;
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
    -- Line engine: 32 clocks after the source avid fall (the pixel path has drained) it computes the
    -- next line straight into the l_* registers; line 0 runs in the vblank zone.
    -- v = the virtual row folded about the mirror line, slipped, folded into the store's range;
    -- fy = (v - lo) K (store rows, Q5); the streak seeds; the camera-store write row; slips.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_v : signed(20 downto 0);
        variable v_f : signed(14 downto 0);
        variable v_j : unsigned(6 downto 0);
        variable v_br : unsigned(4 downto 0);
        variable v_h : unsigned(15 downto 0);
        variable v_n : unsigned(4 downto 0);
        variable v_dy : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 140 then
                n_vy <= (others => '0'); n_wacc <= (others => '0'); n_wrow <= to_unsigned(127, 7);
                n_slr <= f_sseed; n_srem <= (others => '0');
            end if;
            if data_in.avid = '0' and s_avq = '1' then
                le_dl <= to_unsigned(32, 6);
            elsif le_dl /= 0 then
                le_dl <= le_dl - 1;
            end if;
            if s_zone = '1' and s_zc = 164 then
                le_st <= to_unsigned(1, 6);
            elsif le_dl = 1 then
                le_st <= to_unsigned(1, 6);
            elsif le_st /= 0 then
                if le_st = 40 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
            end if;

            l_go <= '0';
            case to_integer(le_st) is
                when 1 =>
                    if f_mir = '1' and n_vy > f_axis then
                        n_v <= signed(resize(f_ax2, 21)) - signed(resize(n_vy, 21));
                    else
                        n_v <= signed(resize(n_vy, 21));
                    end if;
                    n_slr <= f_x16(n_slr);
                when 2 =>
                    if n_srem /= 0 then
                        n_v <= n_v + n_sdy; n_srem <= n_srem - 1;
                    elsif f_slips = '1' and n_slr(7 downto 0) = 0 then
                        -- a new slip: 8 .. 23 lines (fewer at 720p / SD), +-(8..38) rows, +-(0..1.5) cells
                        v_n := ('0' & n_slr(11 downto 8)) + 8;
                        if t_ys >= 768 then v_n := shift_right(v_n, 2);
                        elsif t_ys >= 384 then v_n := shift_right(v_n, 1); end if;
                        n_srem <= v_n;
                        v_dy := shift_left(resize(n_slr(15 downto 12), 13), 1) + 8;
                        if n_slr(9) = '1' then n_sdy <= -signed(resize(v_dy & "00000000", 21));
                        else n_sdy <= signed(resize(v_dy & "00000000", 21)); end if;
                        if n_slr(8) = '1' then n_sdx <= -signed(resize(n_slr(15 downto 10) & "00000000000", 24));
                        else n_sdx <= signed(resize(n_slr(15 downto 10) & "00000000000", 24)); end if;
                    end if;
                when 3 to 6 =>
                    -- fold into the store's range by reflection (twice)
                    if n_v < f_lo then n_v <= f_2lo - n_v;
                    elsif n_v > f_hi then n_v <= f_2hi - n_v; end if;
                when 7 =>
                    v_v := n_v - f_lo;
                    if v_v < 0 then n_m <= (others => '0'); else n_m <= unsigned(v_v(18 downto 0)); end if;
                    n_kb <= f_K; n_pr <= (others => '0');
                when 8 to 18 =>
                    -- serial multiply n_m x f_K (11 steps, MSB first)
                    if n_kb(10) = '1' then
                        n_pr <= shift_left(n_pr, 1) + resize(n_m, 30);
                    else
                        n_pr <= shift_left(n_pr, 1);
                    end if;
                    n_kb <= shift_left(n_kb, 1);
                when 19 =>
                    n_fy <= signed(resize(n_pr(29 downto 16), 15)) - 16;
                when 20 =>
                    v_f := n_fy;
                    if v_f < 0 then v_f := (others => '0');
                    elsif v_f > 4063 then v_f := to_signed(4063, 15); end if;
                    v_j := unsigned(v_f(11 downto 5));
                    l_j0 <= v_j; l_ty <= unsigned(v_f(4 downto 0)); l_jp <= v_j(0);
                    l_rb0 <= resize(shift_right(resize(v_j, 8) + 1, 1), 6);
                    l_rb1 <= v_j(6 downto 1);
                    v_br := resize(shift_right(unsigned(v_f(11 downto 0)) + 16, 7), 5);
                    l_bbase <= shift_left(resize(v_br, 11), 5) + shift_left(resize(v_br, 11), 4);
                    -- streak seeds: one per 2 virtual rows, one per pair
                    v_h := resize(n_vy(19 downto 9), 16) xor f_fseed;
                    n_ho <= v_h + shift_left(v_h, 5);
                    v_h := resize(n_vy(19 downto 10), 16) xor (f_fseed(7 downto 0) & f_fseed(15 downto 8));
                    n_hs <= v_h + shift_left(v_h, 5);
                    l_sg <= n_vy(9);
                when 21 =>
                    n_ho <= n_ho xor shift_right(n_ho, 7);
                    n_hs <= n_hs xor shift_right(n_hs, 7);
                when 22 =>
                    n_ho <= (n_ho + shift_left(n_ho, 3)) or x"0001";
                    n_hs <= (n_hs + shift_left(n_hs, 3)) or x"0001";
                when 23 =>
                    l_so <= n_ho; l_ss <= n_hs; l_go <= '1';
                    -- the line's DDA start (centring offset + slip)
                    if n_srem /= 0 then l_c0 <= f_c0 + n_sdx; else l_c0 <= f_c0; end if;
                when 26 =>
                    -- camera-store write row (the camera's own, unmirrored row)
                    v_j := n_wacc(22 downto 16);
                    if v_j /= n_wrow and n_vy < 1080 * 256 then l_wen <= '1'; else l_wen <= '0'; end if;
                    l_wrow <= v_j; n_wrow <= v_j;
                when 27 =>
                    n_vy <= n_vy + t_ys;
                    n_wacc <= n_wacc + t_wv;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Video write side: camera luma (8 bit, IIR-smoothed) sampled into 64 columns per stored row
    ---------------------------------------------------------------------------
    p_vw : process(clk)
        variable v_d : signed(10 downto 0);
        variable v_x : unsigned(10 downto 0);
        variable v_y : signed(13 downto 0);
        variable v_c : signed(23 downto 0);
        variable v_z : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- W1: luma above black; W2: x 0.291 (8 bit, 0..255); W3: IIR and the column events
            v_d := signed(resize(unsigned(data_in.y), 11)) - 64;
            if v_d < 0 then w_d1 <= (others => '0'); else w_d1 <= unsigned(v_d); end if;
            w_a1 <= data_in.avid;
            v_x := shift_right(w_d1, 2) + shift_right(w_d1, 5) + shift_right(w_d1, 7) + shift_right(w_d1, 9);
            if v_x > 255 then w_l2 <= (others => '1'); else w_l2 <= v_x(7 downto 0); end if;
            w_a2 <= w_a1; w_a3 <= w_a2;
            w_ev <= '0';
            if w_a2 = '1' then
                if w_a3 = '0' then
                    w_yf <= w_l2 & "0000";
                    w_col <= to_signed(-98304, 24);
                else
                    v_y := signed(resize(w_l2 & "0000", 14)) - signed(resize(w_yf, 14));
                    if t_hd = '1' then v_y := shift_right(v_y, 3); else v_y := shift_right(v_y, 2); end if;
                    v_z := signed(resize(w_yf, 14)) + v_y;
                    w_yf <= unsigned(v_z(11 downto 0));
                    v_c := w_col + signed(resize(t_cv, 24));
                    w_col <= v_c;
                    if v_c(23 downto 16) /= w_col(23 downto 16) and v_c >= 0 then
                        w_ev <= '1'; w_k <= unsigned(v_c(21 downto 16));
                    end if;
                end if;
            elsif w_a3 = '1' then
                w_ev <= '1'; w_k <= to_unsigned(63, 6);
            end if;
            -- write strobes
            w_d <= std_logic_vector(w_yf(11 downto 8));
            w_a <= l_wrow(6 downto 1) & w_k(5 downto 1);
            for b in 0 to 3 loop
                if w_ev = '1' and l_wen = '1' and to_integer(unsigned'(l_wrow(0) & w_k(0))) = b then
                    w_we(b) <= '1';
                else
                    w_we(b) <= '0';
                end if;
            end loop;
        end if;
    end process p_vw;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): column DDA (cells, Q16; reverses at the centre for Twin), knot DDA and the
    -- streak knots.  Outside the generated avid everything holds the line's start values, so the
    -- pipeline (and the smear ring) carries the line's first colour through the blanking.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_k : unsigned(22 downto 0);
        variable v_o, v_s : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if g_avid = '0' then
                b_col <= l_c0; b_kn <= (others => '0'); b_rev <= '0'; b_t <= (others => '0');
                if l_go = '1' then
                    b_lfo <= l_so; b_lfs <= l_ss; b_pd <= "10";
                elsif b_pd /= 0 then
                    -- two draws: kp, kq of the line's first knot interval
                    v_o := f_x16(b_lfo); v_s := f_x16(b_lfs);
                    b_kp <= b_kq; b_kq <= f_knot(v_o(15 downto 8), v_s(15 downto 8), l_sg);
                    b_lfo <= v_o; b_lfs <= v_s; b_pd <= b_pd - 1;
                end if;
            else
                if f_twin = '1' and g_cnt >= s_w2 then b_rev <= '1'; end if;
                if b_rev = '1' then
                    b_col <= b_col - signed(resize(f_cstep, 24));
                else
                    b_col <= b_col + signed(resize(f_cstep, 24));
                end if;
                v_k := b_kn + resize(t_kn, 23);
                b_kn <= v_k; b_t <= v_k(15 downto 12);
                if v_k(22 downto 16) /= b_kn(22 downto 16) then
                    v_o := f_x16(b_lfo); v_s := f_x16(b_lfs);
                    b_kp <= b_kq;
                    b_kq <= f_knot(v_o(15 downto 8), v_s(15 downto 8), l_sg);
                    b_lfo <= v_o; b_lfs <= v_s;
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_fx : unsigned(23 downto 0);
        variable v_i0 : unsigned(6 downto 0);
        variable v_ce, v_co : unsigned(5 downto 0);
        variable v_bc : unsigned(6 downto 0);
        variable v_q : unsigned(3 downto 0);
        type t_q4 is array (0 to 3) of unsigned(3 downto 0);
        variable v_c : t_q4;
        variable v_ca, v_cb, v_cc, v_cd : unsigned(3 downto 0);
        variable v_a, v_b, v_cx, v_d : unsigned(7 downto 0);
        variable v_ka, v_kb, v_kc, v_kd : std_logic;
        variable v_r : unsigned(7 downto 0);
        variable v_db, v_dc, v_dd : signed(9 downto 0);
        variable v_g : unsigned(4 downto 0);
        variable v_s : signed(16 downto 0);
        variable v_sn : unsigned(10 downto 0);
        variable v_sd : signed(10 downto 0);
        variable v_ab : signed(11 downto 0);
        variable v_fb : unsigned(23 downto 0);
        variable v_ix : unsigned(10 downto 0);
        variable v_yo, v_uo, v_vo : unsigned(7 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_cu : unsigned(11 downto 0);
        variable v_t : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: clamp, cell, bank addresses
            if b_col < 0 then v_fx := (others => '0');
            elsif unsigned(b_col) > f_cmax then v_fx := f_cmax;
            else v_fx := unsigned(b_col); end if;
            v_i0 := v_fx(22 downto 16);
            p1_tx <= v_fx(15 downto 11); p1_ip <= v_i0(0);
            v_ce := resize(shift_right(resize(v_i0, 8) + 1, 1), 6);    -- even-column bank
            v_co := v_i0(6 downto 1);                                   -- odd-column bank
            a_o00 <= v_ce(3 downto 0) & l_rb0; a_o01 <= v_co(3 downto 0) & l_rb0;
            a_o10 <= v_ce(3 downto 0) & l_rb1; a_o11 <= v_co(3 downto 0) & l_rb1;
            p1_be <= v_ce(5 downto 4); p1_bo <= v_co(5 downto 4);
            a_v00 <= l_rb0 & v_ce(4 downto 0); a_v01 <= l_rb0 & v_co(4 downto 0);
            a_v10 <= l_rb1 & v_ce(4 downto 0); a_v11 <= l_rb1 & v_co(4 downto 0);
            v_fb := v_fx + 32768;
            v_bc := v_fb(23 downto 17);                                   -- busy column (40 px)
            if v_bc > 47 then v_bc := to_unsigned(47, 7); end if;
            a_bz <= l_bbase + resize(v_bc, 11);
            -- noise knots
            q2_kp <= b_kp; q2_dk <= resize(b_kq, 9) - resize(b_kp, 9); q2_t <= b_t;

            -- 2: store reads
            p2_be <= p1_be; p2_bo <= p1_bo; p_tx(2) <= resize(p1_tx, 8); p_ip(2) <= p1_ip;
            q3_kp <= q2_kp; q3_m <= q2_dk * signed(resize(q2_t, 5));

            -- 3: fabric register (Osc: the block select off the ROMs; Video: the camera store)
            p3_be <= p2_be; p3_bo <= p2_bo; p_tx(3) <= p_tx(2); p_ip(3) <= p_ip(2);
            if f_osc = '1' then
                case to_integer(p2_be) is
                    when 0 => p3_x00 <= unsigned(r_s000); p3_x10 <= unsigned(r_s100);
                    when 1 => p3_x00 <= unsigned(r_s001); p3_x10 <= unsigned(r_s101);
                    when others => p3_x00 <= unsigned(r_s002); p3_x10 <= unsigned(r_s102);
                end case;
                case to_integer(p2_bo) is
                    when 0 => p3_x01 <= unsigned(r_s010); p3_x11 <= unsigned(r_s110);
                    when 1 => p3_x01 <= unsigned(r_s011); p3_x11 <= unsigned(r_s111);
                    when others => p3_x01 <= unsigned(r_s012); p3_x11 <= unsigned(r_s112);
                end case;
            else
                p3_x00 <= unsigned(r_v00); p3_x01 <= unsigned(r_v01);
                p3_x10 <= unsigned(r_v10); p3_x11 <= unsigned(r_v11);
            end if;
            p3_bz <= unsigned(r_bz);
            q4_n <= resize(q3_kp, 9) + resize(shift_right(q3_m, 4), 9);

            -- 4: corners (a = j0 i0, b = j0 i1, c = j1 i0, d = j1 i1); bank index = row parity * 2 + column parity
            v_c(0) := p3_x00; v_c(1) := p3_x01; v_c(2) := p3_x10; v_c(3) := p3_x11;
            if l_jp = '0' then
                if p_ip(3) = '0' then v_ca := v_c(0); v_cb := v_c(1); v_cc := v_c(2); v_cd := v_c(3);
                else v_ca := v_c(1); v_cb := v_c(0); v_cc := v_c(3); v_cd := v_c(2); end if;
            else
                if p_ip(3) = '0' then v_ca := v_c(2); v_cb := v_c(3); v_cc := v_c(0); v_cd := v_c(1);
                else v_ca := v_c(3); v_cb := v_c(2); v_cc := v_c(1); v_cd := v_c(0); end if;
            end if;
            p4_ca <= v_ca; p4_cb <= v_cb; p4_cc <= v_cc;
            if f_osc = '1' then
                p4_a <= unsigned(C_LEV(to_integer(v_ca))); p4_b <= unsigned(C_LEV(to_integer(v_cb)));
                p4_c <= unsigned(C_LEV(to_integer(v_cc))); p4_d <= unsigned(C_LEV(to_integer(v_cd)));
                if v_ca = 15 then p4_ka <= '1'; else p4_ka <= '0'; end if;
                if v_cb = 15 then p4_kb <= '1'; else p4_kb <= '0'; end if;
                if v_cc = 15 then p4_kc <= '1'; else p4_kc <= '0'; end if;
                if v_cd = 15 then p4_kd <= '1'; else p4_kd <= '0'; end if;
            else
                p4_a <= v_ca & "0000"; p4_b <= v_cb & "0000"; p4_c <= v_cc & "0000"; p4_d <= v_cd & "0000";
                p4_ka <= '0'; p4_kb <= '0'; p4_kc <= '0'; p4_kd <= '0';
            end if;
            p4_bz <= p3_bz; p_tx(4) <= p_tx(3); p_ip(4) <= p_ip(3);
            q5_n <= q4_n;

            -- 5: key substitution, wrap-aware differences, nearest-corner key, camera busy level
            v_ka := p4_ka; v_kb := p4_kb; v_kc := p4_kc; v_kd := p4_kd;
            if v_ka = '0' then v_r := p4_a; elsif v_kb = '0' then v_r := p4_b;
            elsif v_kc = '0' then v_r := p4_c; else v_r := p4_d; end if;
            if v_ka = '1' then v_a := v_r; else v_a := p4_a; end if;
            if v_kb = '1' then v_b := v_r; else v_b := p4_b; end if;
            if v_kc = '1' then v_cx := v_r; else v_cx := p4_c; end if;
            if v_kd = '1' then v_d := v_r; else v_d := p4_d; end if;
            if f_osc = '1' then
                v_db := resize(signed(v_b - v_a), 10); v_dc := resize(signed(v_cx - v_a), 10);
                v_dd := resize(signed(v_d - v_a), 10);
            else
                v_db := signed(resize(v_b, 10)) - signed(resize(v_a, 10));
                v_dc := signed(resize(v_cx, 10)) - signed(resize(v_a, 10));
                v_dd := signed(resize(v_d, 10)) - signed(resize(v_a, 10));
            end if;
            p5_a <= v_a; p5_db <= v_db(8 downto 0); p5_dc <= v_dc(8 downto 0); p5_ddc <= v_dd - v_dc;
            if l_ty(4) = '0' then
                if p_tx(4)(4) = '0' then p5_nk <= v_ka; else p5_nk <= v_kb; end if;
            else
                if p_tx(4)(4) = '0' then p5_nk <= v_kc; else p5_nk <= v_kd; end if;
            end if;
            -- camera busy level: |c - a| + |b - a| (levels) / 2, max 3
            v_g := resize(unsigned(abs(signed(resize(p4_cc, 5)) - signed(resize(p4_ca, 5)))), 5)
                 + resize(unsigned(abs(signed(resize(p4_cb, 5)) - signed(resize(p4_ca, 5)))), 5);
            if v_g >= 6 then p5_vb <= "11"; else p5_vb <= v_g(2 downto 1); end if;
            p5_bz <= p4_bz; p_tx(5) <= p_tx(4);
            q6_n <= q5_n;

            -- 6: x products; streak amplitude by busy level
            p6_a32 <= signed(resize(p5_a & "00000", 14));
            p6_dc32 <= shift_left(resize(p5_dc, 14), 5);
            p6_m1 <= resize(p5_db * signed(resize(p_tx(5)(4 downto 0), 6)), 14);
            p6_m2 <= resize(p5_ddc * signed(resize(p_tx(5)(4 downto 0), 6)), 15);
            p_nk(6) <= p5_nk;
            if f_osc = '1' then v_q := "00" & p5_bz; else v_q := "00" & p5_vb; end if;
            case to_integer(v_q) is
                when 0 => q6_amp <= f_amp0;
                when 1 => q6_amp <= f_amp1;
                when 2 => q6_amp <= f_amp2;
                when others => q6_amp <= f_amp3;
            end case;

            -- 7: top / bottom rows; streak offset product
            p7_top <= resize(p6_a32, 17) + resize(p6_m1, 17);
            p7_bot <= resize(p6_a32, 17) + resize(p6_dc32, 17) + resize(p6_m2, 17);
            q7_no <= resize(q6_n * signed('0' & q6_amp), 20);

            -- 8: vertical difference
            p8_top <= p7_top;
            p8_dtb <= resize(p7_bot, 18) - resize(p7_top, 18);
            p_no(8) <= resize(shift_right(q7_no, 6), 13);

            -- 9: y product
            p9_top <= p8_top;
            p9_m3 <= resize(p8_dtb * signed(resize(l_ty, 6)), 23);

            -- 10: field phase (2048 = one cycle)
            v_s := p9_top + resize(shift_right(p9_m3, 5), 17);
            p10_ph <= unsigned(v_s(12 downto 2));

            -- 11: + streaks; Negative
            v_sn := p10_ph + unsigned(p_no(10)(10 downto 0));
            if f_neg = '1' then v_sn := 0 - v_sn; end if;
            p11_sn <= v_sn;

            -- 12: key distance; gain operand (Osc about the pivot, Video from black)
            v_sd := signed(p11_sn);
            v_ab := abs(resize(v_sd, 12));
            p12_ab <= unsigned(v_ab(10 downto 0));
            if f_osc = '1' then
                p12_D <= resize(signed(p11_sn - to_unsigned(C_PIV, 11)), 12);
            else
                p12_D <= signed(resize(p11_sn, 12));
            end if;

            -- 13: gain product; key test
            p13_pr <= p12_D * signed('0' & f_g);
            if f_w /= 0 and (p12_ab < resize(f_w, 11) or p_nk(12) = '1') then p13_key <= '1'; else p13_key <= '0'; end if;

            -- 14: palette index (+ pivot + hue)
            v_ix := unsigned(p13_pr(16 downto 6)) + f_ph;
            p14_idx <= v_ix;
            p_key(14) <= p13_key;

            -- 15: palette address
            a_pal <= p14_idx(10 downto 3);
            p_key(15) <= p_key(14);
            -- 16: palette read
            p_key(16) <= p_key(15);
            -- 17: fabric copy
            p17_y <= unsigned(r_pyu(15 downto 8)); p17_u <= unsigned(r_pyu(7 downto 0)); p17_v <= unsigned(r_pv);
            p_key(17) <= p_key(16);
            -- 18: key fill
            if p_key(17) = '1' then
                c18_y <= to_unsigned(C_YEL_Y, 8); c18_u <= to_unsigned(C_YEL_U, 8); c18_v <= to_unsigned(C_YEL_V, 8);
            else
                c18_y <= p17_y; c18_u <= p17_u; c18_v <= p17_v;
            end if;
            c19_y <= c18_y; c19_u <= c18_u; c19_v <= c18_v;
            c20_y <= c19_y; c20_u <= c19_u; c20_v <= c19_v;

            -- 19: box sums (chroma N_c from the ring or short taps; luma N_y), exact running sums
            case to_integer(f_kc) is
                when 0 => v_uo := c19_u; v_vo := c19_v;
                when 1 => v_uo := c20_u; v_vo := c20_v;
                when others => v_uo := unsigned(r_rgf(15 downto 8)); v_vo := unsigned(r_rgf(7 downto 0));
            end case;
            case to_integer(f_ky) is
                when 0 => v_yo := c19_y;
                when 1 => v_yo := c20_y;
                when others => v_yo := unsigned(r_ryf);
            end case;
            if s_bfc = '1' then v_uo := (others => '0'); v_vo := (others => '0'); end if;
            if s_bfy = '1' then v_yo := (others => '0'); end if;
            if s_brst = '1' then
                s_sy <= (others => '0'); s_su <= (others => '0'); s_sv <= (others => '0');
            else
                s_sy <= s_sy + c18_y - v_yo;
                s_su <= s_su + c18_u - v_uo;
                s_sv <= s_sv + c18_v - v_vo;
            end if;
            -- 20: normalise (N = 2^k)
            v_sc := shift_right(s_sy, to_integer(f_ky)); s20_y <= v_sc(7 downto 0);
            v_sc := shift_right(s_su, to_integer(f_kc)); s20_u <= v_sc(7 downto 0);
            v_sc := shift_right(s_sv, to_integer(f_kc)); s20_v <= v_sc(7 downto 0);
            -- 21
            s21_y <= s20_y; s21_u <= s20_u; s21_v <= s20_v;

            -- 22..25: output conditioning (8-bit samples; a sample s is 4 s + 2 in 10 bits):
            -- chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums), luma [1 2 1]/4
            s_csu(0) <= s21_u; s_csv(0) <= s21_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s21_u - s_csu(7);
            s_boxv <= s_boxv + s21_v - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_cu := resize(s_boxu, 12) + s_boxud; s_u25 <= resize(v_cu(11 downto 2), 10) + 2;
            v_cu := resize(s_boxv, 12) + s_boxvd; s_v25 <= resize(v_cu(11 downto 2), 10) + 2;
            s_y22 <= s21_y; s_y23 <= s_y22; s_y24 <= s_y23;
            v_t := resize(s_y22, 12) + shift_left(resize(s_y23, 12), 1) + resize(s_y24, 12);
            s_yd(0) <= v_t(9 downto 0) + 2;
            s_yf <= s_yd(0);
            -- 26: luma scale and gate
            if s_yf > 64 then v_dl := s_yf - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(25) = '1' then
                s_y26 <= s_yf - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u26 <= s_u25; s_v26 <= s_v25;
            else
                s_y26 <= to_unsigned(64, 10); s_u26 <= to_unsigned(512, 10); s_v26 <= to_unsigned(512, 10);
            end if;
            -- 27
            s_y27 <= s_y26; s_u27 <= s_u26; s_v27 <= s_v26;

            -- delays
            p_no(9) <= p_no(8); p_no(10) <= p_no(9);
            for i in 7 to 16 loop p_nk(i) <= p_nk(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y27);
    data_out.u       <= std_logic_vector(s_u27);
    data_out.v       <= std_logic_vector(s_v27);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture idol;
