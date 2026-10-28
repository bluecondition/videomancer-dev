-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Fern v0.5.0 -- Heirloom #46, rebuilt from the photo WIN_20230713_00_17_11.
--
-- The photo: a blue ground of fine horizontal saw scanlines (13.86 lines) on the left; to the right a
-- big curved ">" (apex (856, 515), mirrored about row 515.5) of nested pink / red / yellow frond arms
-- crossed by vertical stems (period 206 px), the cells filled with a green / cyan maze; a small arrowhead
-- at the far left whose first 82 px are a line-start hold (horizontal bars).
--
-- The patch, per channel:
--   * an FM'd line ramp c = (x' + g (1 - 866 k)) / (1 - k g), x' = max(x, 82), g = G(|y - cy|) the
--     fitted chevron law; the arm oscillator theta = T(c) (spacing 108 -> 200 px), the stem oscillator
--     psi = x' / 206;
--   * R = a key on a level field L(theta, psi, height) (a fitted 16 x 16 x 8 table, blended between the
--     height bands per line, bilinear per pixel);  G = a vertical maze oscillator phase-modulated by
--     theta and psi in the cells, a saw-phase colour ramp + contour lines in the fronds, dark outlines on
--     both sides of the key edge;  B = the scanline saw outside the fronds, the stem / arm ramp inside;
--   * a window on c: the blue ground is 282 < c < 856 (rims on both edges).
--
--   K1  Fronds  arm density 0.5x .. 2x (the ramp c is scaled; centre = the photo)
--   K2  Stems   stem frequency 0.5x .. 2x (centre = the photo)
--   K3  Speed   bipolar, still at the centre +-8: the fronds stream outward (default) / inward
--   K4  Flesh   frond thickness: thin wireframe .. fat flames (centre = the photo)
--   K5  Maze    maze modulation depth: straight scan stripes .. tight curly maze (27 % = the photo)
--   K6  Bend    how far the source (camera luma / Osc) bends the fronds, the window edge and the maze
--   S7  Mirror  Photo (mirrored chevron) / Sweep (one-way sweep: the top half bends the other way)
--   S8  Palette Pink (the photo) / Green (Cr negated: green fronds on a magenta maze, blue ground)
--   S9  Dart    the small arrowhead at the left: On / Off
--   S10 Stems   On / Off (the level table read between the stems: pure nested chevrons)
--   S11 Source  Video / Osc
--   P12 ARROW   chevron depth 0.25x (shallow, near-vertical stems) .. 1x (the photo, 43 %) .. 2x (deep
--               converging arrowheads)
--
-- Virtual 1080-line coordinates; SD shows virtual x 0 .. 1440 (left-aligned 4:3).  Generated avid
-- (parity-locked start, field-latched width); output conditioning (luma [1 2 1], chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma x 13/16, pre-boosted 16/13) before the blanking gate.  Latency 32 clocks in
-- every mode (syncs through a block-RAM ring).  Generated colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.fern_pkg.all;
use work.all;

architecture fern of program_top is
    constant C_LAT : integer := 32;   -- matches processing_delay_clks
    constant C_CY256 : integer := 131968;   -- mirror row 515.5, Q.8

    function f_tri8(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;

    -- quarter weights: fill outline at 0..10 px from the key edge, frond rim at 0..13 px
    type t_q is array(0 to 15) of unsigned(2 downto 0);
    constant C_ENVQ : t_q := ("001", "001", "010", "010", "011", "011", "100", "100", "100", "100", "100",
                              "100", "100", "100", "100", "100");
    constant C_ENVMQ : t_q := ("001", "010", "010", "010", "011", "011", "011", "100", "100", "100", "100",
                               "100", "100", "100", "100", "100");
    -- R per key quarter (0..4) and the ground (5), pre-multiplied for Y / U / V (Q.6)
    type t_r6 is array(0 to 7) of signed(15 downto 0);
    constant C_YR : t_r6 := (to_signed(162, 16), to_signed(5022, 16), to_signed(9963, 16), to_signed(14904, 16),
                             to_signed(19845, 16), to_signed(243, 16), to_signed(0, 16), to_signed(0, 16));
    constant C_UR : t_r6 := (to_signed(224, 16), to_signed(6944, 16), to_signed(13776, 16), to_signed(20608, 16),
                             to_signed(27440, 16), to_signed(336, 16), to_signed(0, 16), to_signed(0, 16));
    constant C_VR : t_r6 := (to_signed(-76, 16), to_signed(-2356, 16), to_signed(-4674, 16), to_signed(-6992, 16),
                             to_signed(-9310, 16), to_signed(-114, 16), to_signed(0, 16), to_signed(0, 16));

    -- timing
    signal t_sx2 : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal t_ys  : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_bsl, t_mzl, t_mzq : unsigned(11 downto 0) := (others => '0');
    signal t_kax, t_kay : unsigned(7 downto 0) := (others => '0');
    signal t_psk : unsigned(9 downto 0) := to_unsigned(318, 10);

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls: glided K1, K2, K4, K5, K6, P12 (Q.3)
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(512 * 8, 13), to_unsigned(256 * 8, 13), to_unsigned(438 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w, q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal s_m3 : unsigned(8 downto 0) := (others => '0');
    signal s_s3 : std_logic := '0';
    signal f_sw, f_pal, f_dart0, f_nost, f_osc : std_logic := '0';

    -- per-field values
    signal f_f1, f_f2 : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal f_gs : unsigned(8 downto 0) := to_unsigned(255, 9);
    signal f_lofs : signed(8 downto 0) := to_signed(3, 9);
    signal f_mza5 : unsigned(4 downto 0) := to_unsigned(13, 5);
    signal f_k6s : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal f_sxf : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal f_ps : unsigned(10 downto 0) := to_unsigned(318, 11);
    signal f_pb : unsigned(10 downto 0) := to_unsigned(119, 11);
    signal f_p0 : unsigned(15 downto 0) := to_unsigned(53448, 16);
    signal f_bk : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal f_tlo, f_thi, f_thi2, f_tlo2 : signed(16 downto 0) := (others => '0');
    signal f_m3sq : unsigned(17 downto 0) := (others => '0');
    signal f_spd : unsigned(15 downto 0) := (others => '0');
    signal f_thoff, f_mzoff, f_broll : unsigned(15 downto 0) := (others => '0');
    signal f_ta : unsigned(15 downto 0) := (others => '0');

    -- engine (zone products per field; the NEXT line per line), committed at the source avid rise
    signal n_vy : unsigned(19 downto 0) := (others => '0');
    signal n_bph, n_mzs, n_mzq : unsigned(15 downto 0) := (others => '0');
    signal n_pa : unsigned(15 downto 0) := (others => '0');
    signal n_pp : std_logic := '0';
    signal e_st : unsigned(5 downto 0) := (others => '0');
    signal e_dy : signed(20 downto 0) := (others => '0');
    signal e_neg : std_logic := '0';
    signal e_ga : unsigned(9 downto 0) := (others => '0');
    signal r_egq : std_logic_vector(9 downto 0) := (others => '0');
    signal e_g0 : unsigned(9 downto 0) := (others => '0');
    signal e_gp : unsigned(10 downto 0) := (others => '0');
    signal e_gpn : std_logic := '0';
    signal e_den : unsigned(21 downto 0) := (others => '0');
    signal e_m : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal e_xa : unsigned(10 downto 0) := (others => '0');
    signal e_xan : std_logic := '0';
    signal e_c0m : unsigned(14 downto 0) := (others => '0');
    signal e_j : unsigned(2 downto 0) := (others => '0');
    signal e_fg : unsigned(3 downto 0) := (others => '0');
    -- serial multiplier (18 x 16) and divider
    signal m_a : unsigned(17 downto 0) := (others => '0');
    signal m_b : unsigned(15 downto 0) := (others => '0');
    signal m_h : unsigned(18 downto 0) := (others => '0');
    signal m_l : unsigned(15 downto 0) := (others => '0');
    signal m_cnt : unsigned(4 downto 0) := (others => '0');
    signal d_r : unsigned(22 downto 0) := (others => '0');
    signal d_q : unsigned(15 downto 0) := (others => '0');
    signal d_cnt : unsigned(4 downto 0) := (others => '0');
    -- next-line results / committed line values
    signal n_c0 : signed(18 downto 0) := (others => '0');
    signal n_dc : unsigned(14 downto 0) := to_unsigned(4096, 15);
    signal n_bs, n_q8 : unsigned(7 downto 0) := (others => '0');
    signal l_c0 : signed(18 downto 0) := (others => '0');
    signal l_dc : unsigned(14 downto 0) := to_unsigned(4096, 15);
    signal l_bs, l_q8 : unsigned(7 downto 0) := (others => '0');
    signal l_sl : unsigned(9 downto 0) := (others => '0');
    signal l_pa : unsigned(15 downto 0) := (others => '0');
    signal l_pp : std_logic := '0';

    -- slice builder: 2048 x 16 level ROM -> 4 parity banks x (2 x 64) x 16
    signal sb_go, sb_run : std_logic := '0';
    signal sb_i : unsigned(8 downto 0) := (others => '0');
    signal sb_a : unsigned(10 downto 0) := (others => '0');
    signal r_lutq, sb_q : std_logic_vector(15 downto 0) := (others => '0');
    signal sb_v1, sb_v2, sb_v3, sb_v4 : std_logic := '0';
    signal sb_o1, sb_o2, sb_o3 : std_logic := '0';
    signal sb_e1, sb_e2, sb_e3, sb_e4 : unsigned(7 downto 0) := (others => '0');
    signal sb_w0 : std_logic_vector(15 downto 0) := (others => '0');
    signal sb_pl : signed(11 downto 0) := (others => '0');
    signal sb_pg, sb_pb : signed(9 downto 0) := (others => '0');
    signal sb_l0 : signed(6 downto 0) := (others => '0');
    signal sb_g0 : unsigned(4 downto 0) := (others => '0');
    signal sb_b0 : unsigned(3 downto 0) := (others => '0');
    signal sb_we : std_logic_vector(3 downto 0) := (others => '0');
    signal sb_wa : unsigned(6 downto 0) := (others => '0');
    signal sb_wd : std_logic_vector(15 downto 0) := (others => '0');
    type t_bank is array(0 to 127) of std_logic_vector(15 downto 0);
    signal r_bk0, r_bk1, r_bk2, r_bk3 : t_bank := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_bk0 : signal is true;
    attribute no_rw_check of r_bk1 : signal is true;
    attribute no_rw_check of r_bk2 : signal is true;
    attribute no_rw_check of r_bk3 : signal is true;
    type t_a7 is array(0 to 3) of unsigned(6 downto 0);
    signal r_ba : t_a7 := (others => (others => '0'));
    signal r_bq0, r_bq1, r_bq2, r_bq3 : std_logic_vector(15 downto 0) := (others => '0');
    type t_w16 is array(0 to 3) of std_logic_vector(15 downto 0);

    -- sync ring (hsync, vsync, field): written every clock, read 30 back, two registers -> 32 clocks
    type t_ring is array(0 to 255) of std_logic_vector(3 downto 0);
    signal r_ring : t_ring := (others => (others => '1'));
    attribute no_rw_check of r_ring : signal is true;
    signal r_wp : unsigned(7 downto 0) := (others => '0');
    signal r_rq, r_rq2 : std_logic_vector(3 downto 0) := (others => '1');
    -- chroma ring (8-bit U, V): read 6 back, two registers -> the 8-clock box delay
    type t_cring is array(0 to 255) of std_logic_vector(15 downto 0);
    signal r_cring : t_cring := (others => x"8080");
    attribute no_rw_check of r_cring : signal is true;
    signal r_cq, r_cq2 : std_logic_vector(15 downto 0) := x"8080";

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';
    signal q_gst : std_logic_vector(7 downto 0) := (others => '0');

    -- video tap
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vd2 : signed(7 downto 0) := (others => '0');

    -- beam (stage 0) and the late stem beam (stage 8)
    signal b_vx : unsigned(11 downto 0) := (others => '0');
    signal b_c  : signed(26 downto 0) := (others => '0');
    signal b_pa : unsigned(15 downto 0) := (others => '0');
    signal b_lvx : unsigned(11 downto 0) := (others => '0');
    signal b_psi : unsigned(15 downto 0) := (others => '0');

    -- pixel pipeline
    type t_sl is array (natural range <>) of std_logic;
    type t_u3a is array (natural range <>) of unsigned(2 downto 0);
    type t_u4a is array (natural range <>) of unsigned(3 downto 0);
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    type t_s9a is array (natural range <>) of signed(8 downto 0);
    type t_s10a is array (natural range <>) of signed(9 downto 0);
    signal p1_c4 : signed(18 downto 0) := (others => '0');
    signal p1_ta : signed(8 downto 0) := (others => '0');
    signal p2_c4 : signed(18 downto 0) := (others => '0');
    signal p2_src : signed(7 downto 0) := (others => '0');
    signal p3_c4 : signed(18 downto 0) := (others => '0');
    signal p3_bp : signed(15 downto 0) := (others => '0');
    signal p4_cp : signed(18 downto 0) := (others => '0');
    type t_s6a is array (natural range <>) of signed(5 downto 0);
    signal p_bp5 : t_s6a(4 to 12) := (others => (others => '0'));
    signal r_ta : unsigned(8 downto 0) := (others => '0');
    signal r_thq, p7_th : std_logic_vector(23 downto 0) := (others => '0');
    signal p_f4 : t_u4a(5 to 7) := (others => (others => '0'));
    signal p5_win, p5_left, p5_rim : std_logic := '0';
    signal p_win : t_sl(6 to 21) := (others => '0');
    signal p_left : t_sl(6 to 16) := (others => '0');
    signal p_rim : t_sl(6 to 17) := (others => '0');
    signal p8_tf : unsigned(9 downto 0) := (others => '0');
    -- 9: addresses
    signal r_cta, r_cpa, r_lna : unsigned(7 downto 0) := (others => '0');
    signal r_ctq, r_cpq, r_lnq : std_logic_vector(7 downto 0) := (others => '0');
    signal p_ft : t_u4a(9 to 12) := (others => (others => '0'));
    signal p_fp : t_u4a(9 to 14) := (others => (others => '0'));
    signal p_tpar, p_ppar : t_sl(9 to 11) := (others => '0');
    signal p_wa : t_u3a(9 to 19) := (others => (others => '0'));
    signal p11_bq : t_w16 := (others => (others => '0'));
    signal p11_ct, p11_cp : unsigned(7 downto 0) := (others => '0');
    signal p_ln : t_u4a(11 to 19) := (others => (others => '0'));
    -- 12: corners
    type t_c4l is array(0 to 3) of signed(6 downto 0);
    type t_c4g is array(0 to 3) of unsigned(4 downto 0);
    type t_c4b is array(0 to 3) of unsigned(3 downto 0);
    signal p12_l : t_c4l := (others => (others => '0'));
    signal p12_g : t_c4g := (others => (others => '0'));
    signal p12_b : t_c4b := (others => (others => '0'));
    signal p12_mz : unsigned(12 downto 0) := (others => '0');
    signal p12_cp : unsigned(7 downto 0) := (others => '0');
    -- 13/14: theta lerp
    signal p13_pl0, p13_pl1 : signed(11 downto 0) := (others => '0');
    signal p13_pg0, p13_pg1 : signed(7 downto 0) := (others => '0');
    signal p13_pb0, p13_pb1 : signed(6 downto 0) := (others => '0');
    signal p13_l0, p13_l1 : signed(6 downto 0) := (others => '0');
    signal p13_g0, p13_g1 : unsigned(4 downto 0) := (others => '0');
    signal p13_b0, p13_b1 : unsigned(3 downto 0) := (others => '0');
    signal p13_s : unsigned(9 downto 0) := (others => '0');
    signal p14_la, p14_lb : signed(12 downto 0) := (others => '0');
    signal p14_ga, p14_gb : signed(8 downto 0) := (others => '0');
    signal p14_ba, p14_bb : signed(7 downto 0) := (others => '0');
    signal r_mza : unsigned(7 downto 0) := (others => '0');
    signal r_mzq : std_logic_vector(7 downto 0) := (others => '0');
    -- 15/16: psi lerp
    signal p15_pl : signed(17 downto 0) := (others => '0');
    signal p15_pg : signed(10 downto 0) := (others => '0');
    signal p15_pb : signed(9 downto 0) := (others => '0');
    signal p15_la : signed(12 downto 0) := (others => '0');
    signal p15_ga : signed(8 downto 0) := (others => '0');
    signal p15_ba : signed(7 downto 0) := (others => '0');
    signal p16_l : signed(17 downto 0) := (others => '0');
    signal p_g8 : t_u8a(16 to 19) := (others => (others => '0'));
    signal p_bm : t_u4a(16 to 19) := (others => (others => '0'));
    signal p_mz : t_u8a(16 to 19) := (others => (others => '0'));
    signal p17_lq : signed(9 downto 0) := (others => '0');
    signal p18_lf : signed(9 downto 0) := (others => '0');
    signal p19_kq : unsigned(2 downto 0) := (others => '0');
    signal p19_dout, p19_din : unsigned(3 downto 0) := (others => '0');
    signal p20_gf : unsigned(7 downto 0) := (others => '0');
    signal p20_gm : signed(10 downto 0) := (others => '0');
    signal p20_lt : signed(8 downto 0) := (others => '0');
    signal p20_bm : unsigned(7 downto 0) := (others => '0');
    signal p20_kq : unsigned(2 downto 0) := (others => '0');
    signal p20_din : unsigned(3 downto 0) := (others => '0');
    signal p21_gm : unsigned(7 downto 0) := (others => '0');
    signal p21_gf, p21_bm : unsigned(7 downto 0) := (others => '0');
    signal p21_kq : unsigned(2 downto 0) := (others => '0');
    signal p22_rs : unsigned(2 downto 0) := (others => '0');
    signal p22_g, p22_b : unsigned(7 downto 0) := (others => '0');
    signal p23_rs : unsigned(2 downto 0) := (others => '0');
    signal p23_yg, p23_ug, p23_vg : signed(16 downto 0) := (others => '0');
    signal p23_yb, p23_ub, p23_vb : signed(16 downto 0) := (others => '0');
    signal p24_y, p24_u, p24_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y25, s_y26, s_y27, s_y28, s_y29, s_y30x, s_yf30 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered address and output; a fabric register after each)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_lutq <= C_LUT(to_integer(sb_a));
            r_thq  <= C_TH(to_integer(r_ta));
            r_egq  <= C_EG(to_integer(e_ga));
            r_ctq  <= C_CS(to_integer(r_cta));
            r_lnq  <= C_LN(to_integer(r_lna));
            r_mzq  <= C_MZ(to_integer(r_mza));
        end if;
    end process p_rom;
    p_rom2 : process(clk)
    begin
        if rising_edge(clk) then
            r_cpq <= C_CS(to_integer(r_cpa));
        end if;
    end process p_rom2;

    -- slice banks: one write port (the builder, next line's half), one read port (this line's half)
    p_bk0 : process(clk) begin
        if rising_edge(clk) then
            if sb_we(0) = '1' then r_bk0(to_integer(sb_wa)) <= sb_wd; end if;
            r_bq0 <= r_bk0(to_integer(r_ba(0)));
        end if;
    end process p_bk0;
    p_bk1 : process(clk) begin
        if rising_edge(clk) then
            if sb_we(1) = '1' then r_bk1(to_integer(sb_wa)) <= sb_wd; end if;
            r_bq1 <= r_bk1(to_integer(r_ba(1)));
        end if;
    end process p_bk1;
    p_bk2 : process(clk) begin
        if rising_edge(clk) then
            if sb_we(2) = '1' then r_bk2(to_integer(sb_wa)) <= sb_wd; end if;
            r_bq2 <= r_bk2(to_integer(r_ba(2)));
        end if;
    end process p_bk2;
    p_bk3 : process(clk) begin
        if rising_edge(clk) then
            if sb_we(3) = '1' then r_bk3(to_integer(sb_wa)) <= sb_wd; end if;
            r_bq3 <= r_bk3(to_integer(r_ba(3)));
        end if;
    end process p_bk3;

    -- sync ring
    p_ring : process(clk) begin
        if rising_edge(clk) then
            r_ring(to_integer(r_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_rq <= r_ring(to_integer(r_wp - 30));
            r_rq2 <= r_rq;
            r_wp <= r_wp + 1;
        end if;
    end process p_ring;
    p_cring : process(clk) begin
        if rising_edge(clk) then
            r_cring(to_integer(r_wp)) <= std_logic_vector(p24_u(9 downto 2)) & std_logic_vector(p24_v(9 downto 2));
            r_cq <= r_cring(to_integer(r_wp - 6));
            r_cq2 <= r_cq;
        end if;
    end process p_cring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field values
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_id : integer range 0 to 15;
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_s  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            v_id := to_integer(unsigned(registers_in(8)(3 downto 0)));
            t_sx2 <= unsigned(C_T_SX2(v_id)); t_ys <= unsigned(C_T_YS(v_id));
            t_bsl <= unsigned(C_T_BSL(v_id)); t_mzl <= unsigned(C_T_MZL(v_id)); t_mzq <= unsigned(C_T_MZQ(v_id));
            t_kax <= unsigned(C_T_KAX(v_id)); t_kay <= unsigned(C_T_KAY(v_id));
            t_psk <= unsigned(C_T_PSK(v_id));

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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K3 Speed (bipolar, deadband 8)
                v_x := signed(resize(unsigned(registers_in(2)), 11)) - 512;
                if v_x > 7 then s_s3 <= '0'; v_u := unsigned(v_x) - 8; s_m3 <= v_u(8 downto 0);
                elsif v_x < -7 then s_s3 <= '1'; v_u := unsigned(-v_x) - 8; s_m3 <= v_u(8 downto 0);
                else s_s3 <= '0'; s_m3 <= (others => '0'); end if;
                f_sw    <= registers_in(6)(0);
                f_pal   <= registers_in(6)(1);
                f_dart0 <= registers_in(6)(2);
                f_nost  <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- field phases
                if s_s3 = '1' then f_thoff <= f_thoff - f_spd; else f_thoff <= f_thoff + f_spd; end if;
                f_mzoff <= f_mzoff + 262;
                f_broll <= f_broll + 118;
                f_ta <= f_ta + 262;
            end if;
            -- glides: K1, K2, K4, K5, K6, P12 ease 1/8 of the gap per field (deadband 4); one channel per
            -- four zone clocks (4..27): select, difference, ease, write
            if s_zone = '1' and s_zc >= 4 and s_zc < 28 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(3)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when 4 => gl_k <= unsigned(registers_in(5)) & "000"; gl_o <= s_g(4);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(5);
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

            -- knob curves (registered; settle long before the zone products)
            v_k := s_g(0)(12 downto 3);
            if v_k(9) = '0' then f_f1 <= to_unsigned(128, 9) + resize(v_k(8 downto 2), 9);
            else f_f1 <= to_unsigned(256, 9) + resize(v_k(8 downto 1), 9); end if;
            v_k := s_g(1)(12 downto 3);
            if v_k(9) = '0' then f_f2 <= to_unsigned(128, 9) + resize(v_k(8 downto 2), 9);
            else f_f2 <= to_unsigned(256, 9) + resize(v_k(8 downto 1), 9); end if;
            v_s := signed(resize(s_g(2)(12 downto 3), 13)) - 512;                       -- K4 Flesh
            v_s := shift_right(shift_left(v_s, 2) + v_s, 7);
            f_lofs <= resize(v_s, 9) + 3;
            v_k := s_g(3)(12 downto 3);                                                 -- K5 Maze
            v_k := v_k - resize(v_k(9 downto 3), 10) - resize(v_k(9 downto 5), 10);
            f_mza5 <= v_k(9 downto 5);
            f_k6s <= s_g(4)(12 downto 7);                                               -- K6 Bend
            v_k := s_g(5)(12 downto 3);                                                 -- P12 Arrow
            v_t := shift_left(resize(v_k, 14), 3) - resize(v_k, 14);
            f_gs <= to_unsigned(64, 9) + resize(v_t(13 downto 4), 9);
            case to_integer(t_sx2) is
                when 2 => f_sxf <= resize(f_f1 & '0', 11);
                when 3 => f_sxf <= resize(f_f1, 11) + resize(f_f1 & '0', 11);
                when others => f_sxf <= resize(f_f1 & "00", 11);
            end case;
            f_pb <= resize(shift_right(resize(f_ps, 13) + shift_left(resize(f_ps, 13), 1), 3), 11);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 2/3 clocks after the source's avid rise, field-latched width)
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
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
            q_gst <= q_gst(6 downto 0) & g_start;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Engine: one serial 18 x 16 multiplier and a serial divider.
    --   zone (once per field, zc 40): speed m3^2 13, stem step f2 psk, stem start f2 12089, bend k6 f1,
    --     the four window thresholds x f1;
    --   per line (kicked at the commit = source avid rise of line n; line 0 in the vblank zone, zc 300),
    --     line n + 1: g0 = G(|dy|), band j / fg, g' = g0 gs, m = 2^35 / (2^20 -+ 409 g' / 2),
    --     c0 = (164 +- 0.662 g') m f1, dc = m sx2 f1; the scanline saw and the maze modulator; starts the
    --     slice builder.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_s   : unsigned(18 downto 0);
        variable v_pr  : unsigned(34 downto 0);
        variable v_r   : unsigned(22 downto 0);
        variable v_kick : std_logic;
        variable v_ad  : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            sb_go <= '0';
            v_kick := '0';
            if s_zgo = '1' then
                n_vy <= (others => '0');
                n_bph <= to_unsigned(50665, 16) + f_broll;
                n_mzs <= f_mzoff; n_mzq <= (others => '0');
                n_pa <= f_ta;
            end if;
            if s_zone = '1' and s_zc = 300 then
                v_kick := '1';
            end if;
            if data_in.avid = '1' and s_avq = '0' then
                l_c0 <= n_c0; l_dc <= n_dc; l_bs <= n_bs; l_q8 <= n_q8;
                l_sl <= n_mzs(15 downto 6);
                l_pa <= n_pa;
                l_pp <= n_pp; n_pp <= not n_pp;
                n_vy <= n_vy + t_ys;
                n_bph <= n_bph + t_bsl; n_mzs <= n_mzs + t_mzl; n_mzq <= n_mzq + t_mzq;
                n_pa <= n_pa + t_kay;
                v_kick := '1';
            end if;

            -- serial multiplier: m_h:m_l = m_a * m_b in 16 steps
            if m_cnt /= 0 then
                if m_b(0) = '1' then v_s := m_h + resize(m_a, 19); else v_s := m_h; end if;
                m_h <= '0' & v_s(18 downto 1);
                m_l <= v_s(0) & m_l(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                m_cnt <= m_cnt - 1;
            end if;
            v_pr := m_h(18 downto 0) & m_l;
            -- serial divider: 2^35 / den, 16 quotient bits
            if d_cnt /= 0 then
                v_r := d_r(21 downto 0) & '0';
                if v_r >= resize(e_den, 23) then
                    d_r <= v_r - resize(e_den, 23); d_q <= d_q(14 downto 0) & '1';
                else
                    d_r <= v_r; d_q <= d_q(14 downto 0) & '0';
                end if;
                d_cnt <= d_cnt - 1;
            end if;

            if s_zone = '1' and s_zc = 40 then
                e_st <= to_unsigned(32, 6);
            elsif v_kick = '1' then
                e_st <= to_unsigned(1, 6);
            else
                case to_integer(e_st) is
                    -- per line
                    when 1 =>
                        e_dy <= signed(resize(n_vy, 21)) - C_CY256;
                        e_st <= e_st + 1;
                    when 2 =>
                        if e_dy < 0 then e_neg <= '1'; v_ad := -e_dy; else e_neg <= '0'; v_ad := e_dy; end if;
                        if v_ad >= 576 * 256 then e_ga <= to_unsigned(575, 10);
                        else e_ga <= unsigned(v_ad(17 downto 8)); end if;
                        e_st <= e_st + 1;
                    when 3 =>
                        e_ga <= "1010000000" + resize(n_bph(15 downto 9), 10);    -- 640 + saw phase
                        e_st <= e_st + 1;
                    when 4 =>
                        e_g0 <= unsigned(r_egq);
                        e_ga <= "1100000000" + resize(n_mzq(15 downto 9), 10);    -- 768 + modulator phase
                        e_st <= e_st + 1;
                    when 5 =>
                        n_bs <= unsigned(r_egq(7 downto 0));
                        m_a <= resize(e_g0, 18); m_b <= to_unsigned(1865, 16);
                        m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                        e_st <= e_st + 1;
                    when 6 =>
                        n_q8 <= unsigned(r_egq(7 downto 0));
                        e_st <= e_st + 1;
                    when 7 =>
                        if m_cnt = 0 then
                            -- band position (Q.8, 7 bands over 492 px)
                            if v_pr(20 downto 18) = "111" then e_j <= to_unsigned(6, 3); e_fg <= to_unsigned(15, 4);
                            else e_j <= v_pr(20 downto 18); e_fg <= v_pr(17 downto 14); end if;
                            e_st <= e_st + 1;
                        end if;
                    when 8 =>
                        sb_go <= '1';
                        m_a <= resize(e_g0, 18); m_b <= resize(f_gs, 16);
                        m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                        e_st <= e_st + 1;
                    when 9 =>
                        if m_cnt = 0 then
                            e_gp <= v_pr(18 downto 8);
                            e_gpn <= f_sw and e_neg;                          -- Sweep: signed above the axis
                            e_st <= e_st + 1;
                        end if;
                    when 10 =>
                        m_a <= resize(e_gp, 18); m_b <= to_unsigned(409, 16);
                        m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                        e_st <= e_st + 1;
                    when 11 =>
                        if m_cnt = 0 then
                            if e_gpn = '1' then e_den <= to_unsigned(1048576, 22) + resize(v_pr(21 downto 1), 22);
                            else e_den <= to_unsigned(1048576, 22) - resize(v_pr(21 downto 1), 22); end if;
                            e_st <= e_st + 1;
                        end if;
                    when 12 =>
                        d_r <= to_unsigned(524288, 23); d_q <= (others => '0'); d_cnt <= to_unsigned(16, 5);
                        m_a <= resize(e_gp, 18); m_b <= to_unsigned(43405, 16);
                        m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                        e_st <= e_st + 1;
                    when 13 =>
                        if d_cnt = 0 then
                            e_m <= d_q;
                            -- xa = 164 +- a (Q.1)
                            if e_gpn = '1' then
                                if v_pr(26 downto 16) > 164 then e_xan <= '1'; e_xa <= v_pr(26 downto 16) - 164;
                                else e_xan <= '0'; e_xa <= 164 - v_pr(26 downto 16); end if;
                            else
                                e_xan <= '0'; e_xa <= v_pr(26 downto 16) + 164;
                            end if;
                            e_st <= e_st + 1;
                        end if;
                    when 14 =>
                        m_a <= resize(e_xa, 18); m_b <= e_m;
                        m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                        e_st <= e_st + 1;
                    when 15 =>
                        if m_cnt = 0 then
                            e_c0m <= v_pr(26 downto 12);
                            e_st <= e_st + 1;
                        end if;
                    when 16 =>
                        m_a <= resize(e_c0m, 18); m_b <= resize(f_f1, 16);
                        m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                        e_st <= e_st + 1;
                    when 17 =>
                        if m_cnt = 0 then
                            if e_xan = '1' then n_c0 <= -signed(resize(v_pr(25 downto 8), 19));
                            else n_c0 <= signed(resize(v_pr(25 downto 8), 19)); end if;
                            e_st <= e_st + 1;
                        end if;
                    when 18 =>
                        m_a <= resize(e_m, 18); m_b <= resize(f_sxf, 16);
                        m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                        e_st <= e_st + 1;
                    when 19 =>
                        if m_cnt = 0 then
                            n_dc <= v_pr(26 downto 12);
                            e_st <= (others => '0');
                        end if;
                    -- zone products
                    when 32 =>
                        m_a <= resize(s_m3, 18); m_b <= resize(s_m3, 16);
                        m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                        e_st <= e_st + 1;
                    when 33 =>
                        if m_cnt = 0 then
                            f_m3sq <= v_pr(17 downto 0);
                            m_a <= v_pr(17 downto 0); m_b <= to_unsigned(13, 16);
                            m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                            e_st <= e_st + 1;
                        end if;
                    when 34 =>
                        if m_cnt = 0 then
                            f_spd <= v_pr(25 downto 10);
                            m_a <= resize(f_f2, 18); m_b <= resize(t_psk, 16);
                            m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                            e_st <= e_st + 1;
                        end if;
                    when 35 =>
                        if m_cnt = 0 then
                            f_ps <= v_pr(18 downto 8);
                            m_a <= resize(f_f2, 18); m_b <= to_unsigned(12089, 16);
                            m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                            e_st <= e_st + 1;
                        end if;
                    when 36 =>
                        if m_cnt = 0 then
                            f_p0 <= unsigned(-signed(v_pr(23 downto 8)));
                            m_a <= resize(f_k6s, 18); m_b <= resize(f_f1, 16);
                            m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                            e_st <= e_st + 1;
                        end if;
                    when 37 =>
                        if m_cnt = 0 then
                            f_bk <= v_pr(14 downto 8);
                            m_a <= resize(f_f1, 18); m_b <= to_unsigned(282 * 16, 16);
                            m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                            e_st <= e_st + 1;
                        end if;
                    when 38 =>
                        if m_cnt = 0 then
                            f_tlo <= signed(resize(v_pr(23 downto 8), 17));
                            m_a <= resize(f_f1, 18); m_b <= to_unsigned(856 * 16, 16);
                            m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                            e_st <= e_st + 1;
                        end if;
                    when 39 =>
                        if m_cnt = 0 then
                            f_thi <= signed(resize(v_pr(23 downto 8), 17));
                            m_a <= resize(f_f1, 18); m_b <= to_unsigned(876 * 16, 16);
                            m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                            e_st <= e_st + 1;
                        end if;
                    when 40 =>
                        if m_cnt = 0 then
                            f_thi2 <= signed(resize(v_pr(23 downto 8), 17));
                            m_a <= resize(f_f1, 18); m_b <= to_unsigned(270 * 16, 16);
                            m_h <= (others => '0'); m_l <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                            e_st <= e_st + 1;
                        end if;
                    when 41 =>
                        if m_cnt = 0 then
                            f_tlo2 <= signed(resize(v_pr(23 downto 8), 17));
                            e_st <= (others => '0');
                        end if;
                    when others =>
                        e_st <= (others => '0');
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Slice builder: for e = 0..255 read band j and band j+1, blend by fg (1/16), write bank (e(4), e(0)) at
    -- {next half, e(7 downto 5), e(3 downto 1)}
    ---------------------------------------------------------------------------
    p_sb : process(clk)
        variable v_dl : signed(7 downto 0);
        variable v_dg : signed(5 downto 0);
        variable v_db : signed(4 downto 0);
        variable v_lr : signed(7 downto 0);
        variable v_gr : signed(6 downto 0);
        variable v_br : signed(5 downto 0);
    begin
        if rising_edge(clk) then
            if sb_go = '1' then
                sb_run <= '1'; sb_i <= (others => '0');
            elsif sb_run = '1' then
                if sb_i = 511 then sb_run <= '0'; end if;
                sb_i <= sb_i + 1;
            end if;
            if sb_i(0) = '0' then sb_a <= e_j & sb_i(8 downto 1); else sb_a <= (e_j + 1) & sb_i(8 downto 1); end if;
            sb_v1 <= sb_run; sb_o1 <= sb_i(0); sb_e1 <= sb_i(8 downto 1);
            sb_v2 <= sb_v1; sb_o2 <= sb_o1; sb_e2 <= sb_e1;
            sb_q <= r_lutq;
            sb_v3 <= sb_v2; sb_o3 <= sb_o2; sb_e3 <= sb_e2;
            sb_v4 <= '0';
            if sb_v3 = '1' then
                if sb_o3 = '0' then
                    sb_w0 <= sb_q;
                else
                    sb_l0 <= signed(sb_w0(15 downto 9)); sb_g0 <= unsigned(sb_w0(8 downto 4)); sb_b0 <= unsigned(sb_w0(3 downto 0));
                    v_dl := resize(signed(sb_q(15 downto 9)), 8) - resize(signed(sb_w0(15 downto 9)), 8);
                    v_dg := signed(resize(unsigned(sb_q(8 downto 4)), 6)) - signed(resize(unsigned(sb_w0(8 downto 4)), 6));
                    v_db := signed(resize(unsigned(sb_q(3 downto 0)), 5)) - signed(resize(unsigned(sb_w0(3 downto 0)), 5));
                    sb_pl <= resize(v_dl * signed('0' & e_fg), 12);
                    sb_pg <= resize(v_dg * signed('0' & e_fg), 10);
                    sb_pb <= resize(v_db * signed('0' & e_fg), 10);
                    sb_v4 <= '1'; sb_e4 <= sb_e3;
                end if;
            end if;
            sb_we <= (others => '0');
            if sb_v4 = '1' then
                v_lr := resize(sb_l0, 8) + resize(shift_right(sb_pl, 4), 8);
                v_gr := signed(resize(sb_g0, 7)) + resize(shift_right(sb_pg, 4), 7);
                v_br := signed(resize(sb_b0, 6)) + resize(shift_right(sb_pb, 4), 6);
                sb_wd <= std_logic_vector(v_lr(6 downto 0)) & std_logic_vector(v_gr(4 downto 0)) & std_logic_vector(v_br(3 downto 0));
                sb_wa <= n_pp & sb_e4(7 downto 5) & sb_e4(3 downto 1);
                sb_we(to_integer(unsigned'(sb_e4(4) & sb_e4(0)))) <= '1';
            end if;
        end if;
    end process p_sb;

    ---------------------------------------------------------------------------
    -- Beam: virtual x (half px), FM'd ramp c_T (Q.12, x Fronds) with the line-start hold, Osc phases;
    -- the stem phase psi (Q.16) on a beam 8 clocks late (it is first needed at stage 9)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_vx <= (others => '0');
                b_c <= shift_left(resize(l_c0, 27), 8);
                b_pa <= l_pa;
            elsif g_avid = '1' then
                b_vx <= b_vx + t_sx2;
                if b_vx < 164 then b_c <= shift_left(resize(l_c0, 27), 8);
                else b_c <= b_c + signed(resize(l_dc, 27)); end if;
                b_pa <= b_pa + t_kax;
            end if;
            if q_gst(7) = '1' then
                b_lvx <= (others => '0');
                b_psi <= f_p0;
            elsif s_gav_sr(7) = '1' then
                b_lvx <= b_lvx + t_sx2;
                if b_lvx < 164 then b_psi <= f_p0;
                elsif b_lvx >= 400 and b_lvx < 560 then b_psi <= b_psi + f_ps + f_pb;
                else b_psi <= b_psi + f_ps; end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x   : signed(10 downto 0);
        variable v_s8  : signed(7 downto 0);
        variable v_cp  : signed(18 downto 0);
        variable v_lo  : signed(18 downto 0);
        variable v_e   : signed(19 downto 0);
        variable v_tf  : unsigned(9 downto 0);
        variable v_ta0, v_ta1, v_pa0, v_pa1 : unsigned(2 downto 0);
        variable v_ix  : integer range 0 to 3;
        variable v_s   : unsigned(11 downto 0);
        variable v_l   : signed(9 downto 0);
        variable v_m   : signed(9 downto 0);
        variable v_dp  : unsigned(15 downto 0);
        variable v_w   : signed(16 downto 0);
        variable v_lt  : signed(9 downto 0);
        variable v_g   : signed(10 downto 0);
        variable v_gq  : unsigned(7 downto 0);
        variable v_bq  : unsigned(9 downto 0);
        variable v_dg  : signed(5 downto 0);
        variable v_db  : signed(4 downto 0);
        variable v_d9  : signed(8 downto 0);
        variable v_d8  : signed(7 downto 0);
        variable v_gg, v_bb : unsigned(7 downto 0);
        variable v_y, v_u, v_v : signed(17 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_q   : unsigned(2 downto 0);
        variable v_ab  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: source luma -> signed bend value
            v_y1 <= unsigned(data_in.y);
            v_x := signed(resize(v_y1, 11)) - 502;
            v_x := shift_right(v_x, 2);
            if v_x > 127 then v_s8 := to_signed(127, 8);
            elsif v_x < -128 then v_s8 := to_signed(-128, 8);
            else v_s8 := v_x(7 downto 0); end if;
            s_vd2 <= v_s8;

            -- 1: ramp to Q.4, Osc triangles
            p1_c4 <= resize(shift_right(b_c, 8), 19);
            p1_ta <= f_tri8(b_pa);
            -- 2: source
            p2_c4 <= p1_c4;
            if f_osc = '1' then
                if p1_ta < -128 then p2_src <= to_signed(-128, 8); else p2_src <= p1_ta(7 downto 0); end if;
            else
                p2_src <= s_vd2;
            end if;
            -- 3: bend product (Bend x Fronds)
            p3_c4 <= p2_c4;
            p3_bp <= resize(p2_src * signed('0' & f_bk), 16);
            -- 4: bent ramp
            p4_cp <= p3_c4 + resize(shift_right(p3_bp, 1), 19);
            p_bp5(4) <= resize(shift_right(p3_bp, 9), 6);
            -- 5: T ROM address (16-px knots), window, rim distances (Q.4, c_T units)
            v_cp := p4_cp;
            if v_cp < 0 then r_ta <= (others => '0'); p_f4(5) <= (others => '0');
            elsif v_cp(18 downto 17) /= 0 then r_ta <= (others => '1'); p_f4(5) <= (others => '1');
            else r_ta <= unsigned(v_cp(16 downto 8)); p_f4(5) <= unsigned(v_cp(7 downto 4)); end if;
            if f_dart0 = '1' then v_lo := to_signed(-262144, 19); else v_lo := resize(f_tlo, 19); end if;
            if v_cp < v_lo or v_cp > resize(f_thi, 19) then p5_win <= '1'; else p5_win <= '0'; end if;
            if v_cp < v_lo then p5_left <= '1'; else p5_left <= '0'; end if;
            if (v_cp > resize(f_thi, 19) and v_cp < resize(f_thi2, 19)) or
               (f_dart0 = '0' and v_cp < v_lo and v_cp > resize(f_tlo2, 19)) then p5_rim <= '1'; else p5_rim <= '0'; end if;
            -- 6: (ROM) rims to Q.1, the larger of the two
            p_win(6) <= p5_win; p_left(6) <= p5_left; p_rim(6) <= p5_rim; p_f4(6) <= p_f4(5);
            -- 7: fabric copy of theta / step
            p7_th <= r_thq; p_f4(7) <= p_f4(6);
            -- 8: theta = T + step f4 / 16 (Q.10, fraction) minus the stream phase
            p8_tf <= unsigned(p7_th(17 downto 8)) + resize(shift_right(unsigned(p7_th(7 downto 0)) * p_f4(7), 4), 10)
                     - f_thoff(15 downto 6);
            -- 9: addresses: slice banks, cos(theta), lines, cos(psi + q); arm weight from psi
            v_tf := p8_tf;
            if f_nost = '1' then v_dp := x"8000"; else v_dp := b_psi; end if;
            v_ta1 := v_tf(9 downto 7);
            v_ta0 := v_tf(9 downto 7) + ("00" & v_tf(6));
            v_pa1 := v_dp(15 downto 13);
            v_pa0 := v_dp(15 downto 13) + ("00" & v_dp(12));
            r_ba(0) <= l_pp & v_ta0 & v_pa0;      -- bank (theta even, psi even)
            r_ba(1) <= l_pp & v_ta0 & v_pa1;      -- bank (theta even, psi odd)
            r_ba(2) <= l_pp & v_ta1 & v_pa0;      -- bank (theta odd, psi even)
            r_ba(3) <= l_pp & v_ta1 & v_pa1;      -- bank (theta odd, psi odd)
            p_tpar(9) <= v_tf(6); p_ppar(9) <= v_dp(12);
            p_ft(9) <= v_tf(5 downto 2); p_fp(9) <= v_dp(11 downto 8);
            r_cta <= v_tf(9 downto 2);
            r_lna <= v_tf(7 downto 0);
            r_cpa <= v_dp(15 downto 8) + l_q8;
            v_dp := v_dp + x"8000";
            if v_dp(15) = '1' then v_dp := v_dp - x"8000"; else v_dp := x"8000" - v_dp; end if;
            v_w := signed(resize(v_dp, 17)) - 7930;
            if v_w < 0 then p_wa(9) <= (others => '0'); elsif v_w(16 downto 11) > 4 then p_wa(9) <= to_unsigned(4, 3);
            else p_wa(9) <= unsigned(v_w(13 downto 11)); end if;
            -- 10: (RAM / ROM)
            -- 11: fabric copies
            p11_bq <= (r_bq0, r_bq1, r_bq2, r_bq3);
            p11_ct <= unsigned(r_ctq); p11_cp <= unsigned(r_cpq); p_ln(11) <= unsigned(r_lnq(7 downto 4));
            -- 12: corners by parity; maze product
            for k in 0 to 3 loop
                case k is
                    when 0 => v_ix := to_integer(unsigned'(p_tpar(11) & p_ppar(11)));
                    when 1 => v_ix := to_integer(unsigned'(p_tpar(11) & not p_ppar(11)));
                    when 2 => v_ix := to_integer(unsigned'(not p_tpar(11) & p_ppar(11)));
                    when others => v_ix := to_integer(unsigned'(not p_tpar(11) & not p_ppar(11)));
                end case;
                p12_l(k) <= signed(p11_bq(v_ix)(15 downto 9));
                p12_g(k) <= unsigned(p11_bq(v_ix)(8 downto 4));
                p12_b(k) <= unsigned(p11_bq(v_ix)(3 downto 0));
            end loop;
            p12_mz <= f_mza5 * p11_ct;
            p12_cp <= p11_cp;
            -- 13: theta lerp products (L 1/16, G / B 1/4); maze phase
            p13_l0 <= p12_l(0); p13_l1 <= p12_l(1);
            p13_g0 <= p12_g(0); p13_g1 <= p12_g(1);
            p13_b0 <= p12_b(0); p13_b1 <= p12_b(1);
            p13_pl0 <= resize((resize(p12_l(2), 8) - resize(p12_l(0), 8)) * signed('0' & p_ft(12)), 12);
            p13_pl1 <= resize((resize(p12_l(3), 8) - resize(p12_l(1), 8)) * signed('0' & p_ft(12)), 12);
            v_dg := signed(resize(p12_g(2), 6)) - signed(resize(p12_g(0), 6));
            p13_pg0 <= resize(v_dg * signed('0' & p_ft(12)(3 downto 2)), 8);
            v_dg := signed(resize(p12_g(3), 6)) - signed(resize(p12_g(1), 6));
            p13_pg1 <= resize(v_dg * signed('0' & p_ft(12)(3 downto 2)), 8);
            v_db := signed(resize(p12_b(2), 5)) - signed(resize(p12_b(0), 5));
            p13_pb0 <= resize(v_db * signed('0' & p_ft(12)(3 downto 2)), 7);
            v_db := signed(resize(p12_b(3), 5)) - signed(resize(p12_b(1), 5));
            p13_pb1 <= resize(v_db * signed('0' & p_ft(12)(3 downto 2)), 7);
            v_s := resize(l_sl, 12) + resize(p12_mz(12 downto 1), 12) + resize(p12_cp & '0', 12)
                   + unsigned(shift_left(resize(p_bp5(12), 12), 4));
            p13_s <= v_s(9 downto 0);
            -- 14: xa, xb; maze ROM address
            p14_la <= shift_left(resize(p13_l0, 13), 4) + resize(p13_pl0, 13);
            p14_lb <= shift_left(resize(p13_l1, 13), 4) + resize(p13_pl1, 13);
            p14_ga <= signed(shift_left(resize(p13_g0, 9), 2)) + resize(p13_pg0, 9);
            p14_gb <= signed(shift_left(resize(p13_g1, 9), 2)) + resize(p13_pg1, 9);
            p14_ba <= signed(shift_left(resize(p13_b0, 8), 2)) + resize(p13_pb0, 8);
            p14_bb <= signed(shift_left(resize(p13_b1, 8), 2)) + resize(p13_pb1, 8);
            r_mza <= p13_s(9 downto 2);
            -- 15: psi lerp products; (MZ ROM)
            p15_la <= p14_la; p15_ga <= p14_ga; p15_ba <= p14_ba;
            p15_pl <= resize((resize(p14_lb, 14) - resize(p14_la, 14)) * signed('0' & p_fp(14)), 18);
            v_d9 := p14_gb - p14_ga;
            p15_pg <= resize(v_d9 * signed('0' & p_fp(14)(3 downto 2)), 11);
            v_d8 := p14_bb - p14_ba;
            p15_pb <= resize(v_d8 * signed('0' & p_fp(14)(3 downto 2)), 10);
            -- 16: L (Q.1 << 8), G mean (0..255), B level; MZ fabric copy
            p16_l <= shift_left(resize(p15_la, 18), 4) + p15_pl;
            v_g := shift_left(resize(p15_ga, 11), 2) + p15_pg;                         -- G/8 << 4
            if v_g < 0 then p_g8(16) <= (others => '0'); else p_g8(16) <= unsigned(v_g(8 downto 1)); end if;
            v_e := resize(shift_left(resize(p15_ba, 11), 2) + resize(p15_pb, 11), 20);   -- B4 << 4
            if v_e < 0 then p_bm(16) <= (others => '0'); else p_bm(16) <= unsigned(v_e(7 downto 4)); end if;
            p_mz(16) <= unsigned(r_mzq);
            -- 17: L + Flesh (+ Dart thinning)
            v_l := resize(shift_right(p16_l, 8), 10) + resize(f_lofs, 10);
            if p_left(16) = '1' then v_l := v_l - 14; end if;
            p17_lq <= v_l;
            -- 18: max with the window rims, clamp
            if p_rim(17) = '1' and p17_lq < 6 then v_l := to_signed(6, 10); else v_l := p17_lq; end if;
            if v_l < -64 then p18_lf <= to_signed(-64, 10); elsif v_l > 63 then p18_lf <= to_signed(63, 10);
            else p18_lf <= v_l; end if;
            -- 19: key quarters, outline / rim indices
            v_l := p18_lf;
            if v_l < -3 then p19_kq <= to_unsigned(0, 3); elsif v_l < -1 then p19_kq <= to_unsigned(1, 3);
            elsif v_l < 1 then p19_kq <= to_unsigned(2, 3); elsif v_l < 3 then p19_kq <= to_unsigned(3, 3);
            else p19_kq <= to_unsigned(4, 3); end if;
            v_m := -v_l;
            if v_m < 0 then p19_dout <= (others => '0'); elsif v_m > 21 then p19_dout <= to_unsigned(10, 4);
            else p19_dout <= unsigned(v_m(4 downto 1)); end if;
            if v_l < 0 then p19_din <= (others => '0'); elsif v_l > 27 then p19_din <= to_unsigned(13, 4);
            else p19_din <= unsigned(v_l(4 downto 1)); end if;
            -- 20: fill green (maze x outline quarter), frond green (x2, lines x arm quarter), frond blue
            v_q := C_ENVQ(to_integer(p19_dout)); v_gq := p_mz(19);
            case to_integer(v_q) is
                when 1 => p20_gf <= shift_right(v_gq, 2);
                when 2 => p20_gf <= shift_right(v_gq, 1);
                when 3 => v_ab := resize(v_gq, 10) + shift_left(resize(v_gq, 10), 1); p20_gf <= v_ab(9 downto 2);
                when others => p20_gf <= v_gq;
            end case;
            v_lt := signed(resize(shift_right(shift_left(resize(p_ln(19) & "0000", 10), 2) + resize(p_ln(19) & "0000", 10), 3), 10)) - 48;
            case to_integer(p_wa(19)) is
                when 0 => p20_lt <= (others => '0');
                when 1 => p20_lt <= resize(shift_right(v_lt, 2), 9);
                when 2 => p20_lt <= resize(shift_right(v_lt, 1), 9);
                when 3 => p20_lt <= resize(shift_right(v_lt + shift_left(v_lt, 1), 2), 9);
                when others => p20_lt <= resize(v_lt, 9);
            end case;
            p20_gm <= signed(shift_left(resize(p_g8(19), 11), 1)) - 112;
            p20_bm <= to_unsigned(64, 8) + shift_left(resize(p_bm(19), 8), 3) + resize(p_bm(19), 8);
            p20_kq <= p19_kq; p20_din <= p19_din;
            -- 21: frond green with lines, clamp 0..200, times the rim quarter
            v_g := p20_gm + resize(p20_lt, 11);
            if v_g < 0 then v_gq := (others => '0'); elsif v_g > 200 then v_gq := to_unsigned(200, 8);
            else v_gq := unsigned(v_g(7 downto 0)); end if;
            v_q := C_ENVMQ(to_integer(p20_din));
            case to_integer(v_q) is
                when 1 => p21_gm <= shift_right(v_gq, 2);
                when 2 => p21_gm <= shift_right(v_gq, 1);
                when 3 => v_ab := resize(v_gq, 10) + shift_left(resize(v_gq, 10), 1); p21_gm <= v_ab(9 downto 2);
                when others => p21_gm <= v_gq;
            end case;
            p21_gf <= p20_gf; p21_bm <= p20_bm; p21_kq <= p20_kq;
            -- 22: compose in quarters (ground: R 3, G 0, B saw)
            v_gg := p21_gm; v_bb := p21_bm;
            v_bq := resize(l_bs, 10) - resize(l_bs(7 downto 4), 10);           -- 15/16 saw in the fill
            if p_win(21) = '1' then
                p22_rs <= p21_kq;
                case to_integer(p21_kq) is
                    when 0 => p22_g <= p21_gf; p22_b <= v_bq(7 downto 0);
                    when 1 =>
                        v_ab := resize(v_gg, 10) + shift_left(resize(p21_gf, 10), 1) + resize(p21_gf, 10); p22_g <= v_ab(9 downto 2);
                        v_ab := resize(v_bb, 10) + shift_left(v_bq, 1) + v_bq; p22_b <= v_ab(9 downto 2);
                    when 2 =>
                        v_ab := resize(v_gg, 10) + resize(p21_gf, 10); p22_g <= v_ab(8 downto 1);
                        v_ab := resize(v_bb, 10) + v_bq; p22_b <= v_ab(8 downto 1);
                    when 3 =>
                        v_ab := shift_left(resize(v_gg, 10), 1) + resize(v_gg, 10) + resize(p21_gf, 10); p22_g <= v_ab(9 downto 2);
                        v_ab := shift_left(resize(v_bb, 10), 1) + resize(v_bb, 10) + v_bq; p22_b <= v_ab(9 downto 2);
                    when others => p22_g <= v_gg; p22_b <= v_bb;
                end case;
            else
                p22_rs <= to_unsigned(5, 3); p22_g <= (others => '0'); p22_b <= l_bs;
            end if;
            -- 23: RGB -> YUV terms (Q.6, luma pre-boosted 16/13): R from a table, G / B in signed-digit form
            p23_rs <= p22_rs;
            v_gg := p22_g; v_bb := p22_b;
            p23_yg <= signed(shift_left(resize(v_gg, 17), 7) + shift_left(resize(v_gg, 17), 5) - resize(v_gg, 17));   -- 159 G
            p23_yb <= signed(shift_left(resize(v_bb, 17), 5) - resize(v_bb, 17));                                    -- 31 B
            p23_ug <= signed(shift_left(resize(v_gg, 17), 7) - shift_left(resize(v_gg, 17), 5) - shift_left(resize(v_gg, 17), 1));   -- 94 G
            p23_ub <= signed(shift_left(resize(v_bb, 17), 4) + shift_left(resize(v_bb, 17), 1));                    -- 18 B
            p23_vg <= signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 3) + shift_left(resize(v_gg, 17), 2) - resize(v_gg, 17));   -- 75 G
            p23_vb <= signed(shift_left(resize(v_bb, 17), 7) - shift_left(resize(v_bb, 17), 4));                    -- 112 B
            -- 24: sums to 10 bits; Palette Green: Cr negated
            v_y := shift_right(resize(C_YR(to_integer(p23_rs)), 18) + resize(p23_yg, 18) + resize(p23_yb, 18), 6) + 64;
            v_u := shift_right(resize(C_UR(to_integer(p23_rs)), 18) - resize(p23_ug, 18) - resize(p23_ub, 18), 6) + 512;
            v_v := shift_right(resize(C_VR(to_integer(p23_rs)), 18) - resize(p23_vg, 18) + resize(p23_vb, 18), 6) + 512;
            if f_pal = '1' then v_u := 1024 - v_u; end if;
            if v_y > 1023 then p24_y <= to_unsigned(1023, 10); else p24_y <= unsigned(v_y(9 downto 0)); end if;
            if v_u < 0 then p24_u <= (others => '0'); elsif v_u > 1023 then p24_u <= to_unsigned(1023, 10); else p24_u <= unsigned(v_u(9 downto 0)); end if;
            if v_v < 0 then p24_v <= (others => '0'); elsif v_v > 1023 then p24_v <= to_unsigned(1023, 10); else p24_v <= unsigned(v_v(9 downto 0)); end if;

            -- 25..30: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums, luma [1 2 1])
            s_boxu <= s_boxu + (p24_u(9 downto 2) & "00") - (unsigned(r_cq2(15 downto 8)) & "00");
            s_boxv <= s_boxv + (p24_v(9 downto 2) & "00") - (unsigned(r_cq2(7 downto 0)) & "00");
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u30 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v30 <= v_sc(13 downto 4);
            s_y25 <= p24_y; s_y26 <= s_y25; s_y27 <= s_y26; s_y28 <= s_y27;
            s_y29 <= s_y28; s_y30x <= s_y29;
            v_sy := resize(s_y28, 12) + shift_left(resize(s_y29, 12), 1) + resize(s_y30x, 12);
            s_yf30 <= v_sy(11 downto 2);
            -- 31: luma toward black (x 13/16); blanking gate
            if s_yf30 > 64 then v_dl := s_yf30 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(29) = '1' then
                s_y31 <= s_yf30 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u31 <= s_u30; s_v31 <= s_v30;
            else
                s_y31 <= to_unsigned(64, 10); s_u31 <= to_unsigned(512, 10); s_v31 <= to_unsigned(512, 10);
            end if;
            -- 32: pad to the 32-clock latency
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;

            -- delays
            for i in 5 to 12 loop p_bp5(i) <= p_bp5(i - 1); end loop;
            for i in 7 to 21 loop p_win(i) <= p_win(i - 1); end loop;
            for i in 7 to 16 loop p_left(i) <= p_left(i - 1); end loop;
            for i in 7 to 17 loop p_rim(i) <= p_rim(i - 1); end loop;
            for i in 10 to 11 loop p_tpar(i) <= p_tpar(i - 1); p_ppar(i) <= p_ppar(i - 1); end loop;
            for i in 10 to 12 loop p_ft(i) <= p_ft(i - 1); end loop;
            for i in 10 to 14 loop p_fp(i) <= p_fp(i - 1); end loop;
            for i in 10 to 19 loop p_wa(i) <= p_wa(i - 1); end loop;
            for i in 12 to 19 loop p_ln(i) <= p_ln(i - 1); end loop;
            for i in 17 to 19 loop p_mz(i) <= p_mz(i - 1); p_g8(i) <= p_g8(i - 1); p_bm(i) <= p_bm(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y32);
    data_out.u       <= std_logic_vector(s_u32);
    data_out.v       <= std_logic_vector(s_v32);
    data_out.hsync_n <= r_rq2(0);
    data_out.vsync_n <= r_rq2(1);
    data_out.field_n <= r_rq2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture fern;
