-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Crossbeam v0.5.0 -- Heirloom #37, rebuilt 2026-10-27 from WIN_20230527_01_43_26.
--
-- The photo: a glowing cyan cross beam (two arms crossing at the centre, slope ~0.23, a bright core
-- fading to dark teal at the frame edges) luma-keyed over a grid of colour bars (rows of 39 lines,
-- a 612 px period of three columns 305 / 230 / 77, the magenta / yellow / white / blue / olive /
-- maroon / navy / black table measured cell by cell, rows 6-9 and 22-25 torn 376 px sideways).
-- Brighter bars push into the beam (the bullet-shaped wedges left and right, the yellow sliver in
-- the core); a dark keyer outline runs along every cut.  Cells that show "through" carry a
-- mirrored contour field: red / black zebra stripes (a wavefolder on a smooth field) in the corners
-- and along the bottom, torn row by row, and green -> ochre -> red ramps near the centre.  The
-- zebra and the beam meet in a non-additive mix (salmon lines on teal).
--
--   K1  Angle     the beam's arm slope 0.05 .. 0.55 (0.23 = the photo)
--   K2  Rows      colour-bar rows per frame 8 .. 64 (27.7)
--   K3  Contours  zebra stripe density 0.25 .. 4 x (1 x = the photo)
--   K4  Flow      bipolar: the zebra contours stream in / out (centre still)
--   K5  Roll      bipolar: the colour bars roll up / down (centre still)
--   K6  Columns   colour-bar periods across 1.5 .. 8 (3.14)
--   S7  Bars      the colour bars (Off: the beam over the zebra field alone)
--   S8  Tear      the torn rows (zebra tears + the 376 px row shifts + jitter)
--   S9  Stack     the cross repeats at half the vertical period
--   S10 Palette   Photo / Swap (Cr and Cb exchanged: amber beam, blue zebra, cyan bars)
--   S11 Source    Video / Osc (Video: the camera's slewed luma bends the zebra contours and blooms the beam)
--   P12 BEAM      the cross beam's brightness 0 .. 2 x: 0 = the bars and zebra alone, 50 % = the
--                 photo, 100 % = the beam swallows everything but white, yellow and the zebra.
--
-- Engine: per-field sequencer on one serial shift-add multiplier; per-line scalar engine (row,
-- beam dy, table row, tear, block start) and a table engine that lerps the next line's zebra phase
-- (64 entries, value + slope) and amplitude / ramp (32 entries, even / odd banks) into ping-pong
-- line RAMs.  Pixel path: beam DDAs, the cross field t = d1 d2 / c^2 (d = dy -+ s dx); the beam
-- level in the log domain (log profile + log falloff + log gain + ridge + camera bloom, then an exp
-- ROM), the cell ROM, zebra lerp / mantissa-exponent gain / fold, key compare, colour per source in
-- YUV (beam colour ROM, shift-add red / green parts, YUV palette), outline, luma curve, output
-- conditioning.  Latency 24 clocks in every mode.  Colours in the hardware convention (u = Cr,
-- v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.crossbeam_pkg.all;
use work.all;

architecture crossbeam of program_top is
    constant C_LAT   : integer := 24;   -- matches processing_delay_clks
    constant C_SUPER : integer := 117504;   -- 3 block periods (1836 px) in Q.6
    constant C_XC    : integer := 252032;   -- mirror / beam centre 984.5 px, Q.8

    attribute no_rw_check : boolean;
    -- ROMs
    signal r_kca : unsigned(7 downto 0) := (others => '0');
    signal r_kcq : std_logic_vector(13 downto 0) := (others => '0');
    signal r_pta : unsigned(10 downto 0) := (others => '0');
    signal r_ptq : std_logic_vector(7 downto 0) := (others => '0');
    signal r_aga : unsigned(9 downto 0) := (others => '0');
    signal r_agq : std_logic_vector(15 downto 0) := (others => '0');
    signal r_cla : unsigned(8 downto 0) := (others => '0');
    signal r_clq : std_logic_vector(7 downto 0) := (others => '0');
    signal r_fa  : unsigned(8 downto 0) := (others => '0');
    signal r_fq  : std_logic_vector(10 downto 0) := (others => '0');
    signal r_axa : unsigned(7 downto 0) := (others => '0');
    signal r_axq : std_logic_vector(10 downto 0) := (others => '0');
    signal r_exa : unsigned(9 downto 0) := (others => '0');
    signal r_exq : std_logic_vector(7 downto 0) := (others => '0');
    signal r_gya : unsigned(7 downto 0) := (others => '0');
    signal r_gyq : std_logic_vector(29 downto 0) := (others => '0');

    -- line RAMs (ping-pong by line parity): zebra phase {value, slope}; amplitude / ramp even and odd
    type t_pl is array (0 to 127) of std_logic_vector(15 downto 0);
    signal pl : t_pl := (others => (others => '0'));
    attribute no_rw_check of pl : signal is true;
    signal pl_wa, pl_ra : unsigned(6 downto 0) := (others => '0');
    signal pl_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal pl_we : std_logic := '0';
    signal pl_q  : std_logic_vector(15 downto 0) := (others => '0');
    type t_ab is array (0 to 31) of std_logic_vector(15 downto 0);
    signal ae, ao : t_ab := (others => (others => '0'));
    attribute no_rw_check of ae : signal is true;
    attribute no_rw_check of ao : signal is true;
    signal ab_wa : unsigned(4 downto 0) := (others => '0');
    signal ae_ra, ao_ra : unsigned(4 downto 0) := (others => '0');
    signal ab_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal ae_we, ao_we : std_logic := '0';
    signal ae_q, ao_q : std_logic_vector(15 downto 0) := (others => '0');
    -- key line buffer (the previous line's beam-over-bar flag, for the outline)
    type t_wb is array (0 to 2047) of std_logic_vector(0 downto 0);
    signal wb : t_wb := (others => (others => '0'));
    attribute no_rw_check of wb : signal is true;
    signal wb_wa, wb_ra : unsigned(10 downto 0) := (others => '0');
    signal wb_wd : std_logic_vector(0 downto 0) := (others => '0');
    signal wb_q  : std_logic_vector(0 downto 0) := (others => '0');
    -- chroma delay-8 ring for the output conditioning (neutral at power-up, re-neutralised in vblank)
    type t_crg is array (0 to 15) of std_logic_vector(15 downto 0);
    signal crg : t_crg := (others => x"8080");
    attribute no_rw_check of crg : signal is true;
    signal cr_p, cr_wa, cr_ra : unsigned(3 downto 0) := (others => '0');
    signal cr_wd : std_logic_vector(15 downto 0) := x"8080";
    signal cr_q  : std_logic_vector(15 downto 0) := x"8080";

    -- timing
    signal t_xs  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal t_ys  : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_dys : unsigned(10 downto 0) := to_unsigned(342, 11);
    signal t_dyt : unsigned(13 downto 0) := to_unsigned(1928, 14);
    signal t_xk  : unsigned(17 downto 0) := to_unsigned(87552, 18);
    signal t_yk  : unsigned(17 downto 0) := to_unsigned(15534, 18);

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls (glided Q.3: K1, K2, K3, K6, P12)
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(372 * 8, 13), to_unsigned(359 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(258 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal s_s4, s_s5 : std_logic := '0';
    signal s_m4, s_m5 : unsigned(8 downto 0) := (others => '0');
    signal f_bars, f_tear, f_stack, f_alt, f_osc : std_logic := '0';
    signal s_lf : unsigned(15 downto 0) := x"ACE1";

    -- serial multiplier
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 12) := (others => '0');
    signal m_a : unsigned(23 downto 0) := (others => '0');
    signal m_b : unsigned(19 downto 0) := (others => '0');
    signal m_h : unsigned(23 downto 0) := (others => '0');
    signal m_l : unsigned(19 downto 0) := (others => '0');

    -- per-field values (power up at the 1080p defaults)
    signal f_s    : unsigned(11 downto 0) := to_unsigned(949, 12);
    signal f_rows : unsigned(13 downto 0) := to_unsigned(7074, 14);
    signal f_ks   : unsigned(11 downto 0) := to_unsigned(79, 12);
    signal f_sdxm : unsigned(19 downto 0) := to_unsigned(78093, 20);
    signal f_sdx0 : signed(21 downto 0) := to_signed(-78093, 22);
    signal f_dq   : unsigned(15 downto 0) := to_unsigned(1676, 16);
    signal f_kc   : unsigned(13 downto 0) := to_unsigned(4104, 14);
    signal f_incb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal f_b0m  : unsigned(15 downto 0) := to_unsigned(7502, 16);
    signal f_b0   : unsigned(16 downto 0) := to_unsigned(110002, 17);
    signal f_m4sq, f_m5sq : unsigned(17 downto 0) := (others => '0');
    signal f_fst, f_rst : unsigned(12 downto 0) := (others => '0');
    signal f_e0   : unsigned(23 downto 0) := to_unsigned(1488890, 24);   -- SDX0^2 / 2^12
    signal f_sk   : unsigned(31 downto 0) := (others => '0');                -- SDXm kS
    signal f_kk   : unsigned(23 downto 0) := to_unsigned(6241, 24);          -- kS^2
    signal f_f0   : signed(31 downto 0) := to_signed(-12332453, 32);         -- kS^2 - 2 SDXm kS
    signal f_k2   : unsigned(24 downto 0) := to_unsigned(12482, 25);         -- 2 kS^2
    signal f_gm   : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal f_ge   : unsigned(1 downto 0) := "10";
    signal f_lb   : signed(11 downto 0) := (others => '0');
    signal f_lbm  : signed(12 downto 0) := to_signed(-1024, 13);
    signal f_pvq, f_pvh : signed(18 downto 0) := to_signed(250317, 19);
    signal f_dy0  : signed(18 downto 0) := to_signed(90491, 19);
    signal f_fl   : unsigned(15 downto 0) := (others => '0');     -- zebra flow phase, Q.16 of a stripe
    signal f_ro   : unsigned(20 downto 0) := (others => '0');     -- row roll, Q.16 rows mod 28

    -- per-line scalar engine (next line) and committed line values
    signal le_st : unsigned(2 downto 0) := (others => '0');
    signal n_q   : unsigned(20 downto 0) := (others => '0');
    signal n_dy  : signed(18 downto 0) := (others => '0');
    signal n_yq  : unsigned(21 downto 0) := (others => '0');
    signal n_bank : std_logic := '0';
    signal n_r   : unsigned(4 downto 0) := (others => '0');
    signal n_fyh : std_logic := '0';
    signal n_sh  : std_logic := '0';
    signal n_tr  : signed(7 downto 0) := (others => '0');
    signal n_jt  : signed(2 downto 0) := (others => '0');
    signal n_tear : signed(8 downto 0) := (others => '0');
    signal n_bx0 : signed(17 downto 0) := (others => '0');
    signal l_r   : unsigned(4 downto 0) := (others => '0');
    signal l_fyh, l_bank : std_logic := '0';
    signal l_dy  : signed(18 downto 0) := (others => '0');
    signal l_dy2 : unsigned(21 downto 0) := (others => '0');
    signal l_ady : unsigned(16 downto 0) := (others => '0');
    signal sq_n  : unsigned(4 downto 0) := (others => '0');
    signal sq_a  : unsigned(16 downto 0) := (others => '0');
    signal sq_b  : unsigned(16 downto 0) := (others => '0');
    signal sq_h  : unsigned(16 downto 0) := (others => '0');
    signal sq_l  : unsigned(16 downto 0) := (others => '0');
    signal l_tear : signed(8 downto 0) := (others => '0');
    signal l_bx0 : unsigned(16 downto 0) := (others => '0');

    -- table engine (10 clocks per entry: two reads, serial radix-4 lerp, write)
    signal e_go, e_run : std_logic := '0';
    signal e_c   : unsigned(5 downto 0) := (others => '0');
    signal e_k   : unsigned(3 downto 0) := (others => '0');
    signal e_iy  : unsigned(4 downto 0) := (others => '0');
    signal e_iy1 : unsigned(4 downto 0) := (others => '0');
    signal e_f   : unsigned(5 downto 0) := (others => '0');
    signal e_bank, e_first : std_logic := '0';
    signal e_pd, e_pa, e_pprev : unsigned(7 downto 0) := (others => '0');
    signal e_ad, e_aa : std_logic_vector(15 downto 0) := (others => '0');
    signal e_dP, e_dP3 : signed(10 downto 0) := (others => '0');
    signal e_dA, e_dA3 : signed(8 downto 0) := (others => '0');
    signal e_dR, e_dR3, e_dG, e_dG3 : signed(7 downto 0) := (others => '0');
    signal e_xP : signed(16 downto 0) := (others => '0');
    signal e_xA : signed(14 downto 0) := (others => '0');
    signal e_xR, e_xG : signed(13 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- beam
    signal b_xv  : unsigned(19 downto 0) := (others => '0');
    signal b_sdx : signed(21 downto 0) := (others => '0');
    signal b_bx  : unsigned(16 downto 0) := (others => '0');
    signal b_pcf : unsigned(10 downto 0) := (others => '0');   -- free-running pixel clock: stage-k pixel x = pcf - k
    signal b_e   : signed(24 downto 0) := (others => '0');     -- SDX^2 / 2^12 (2nd-order DDA; dips below 0 near the centre)
    signal b_f   : signed(31 downto 0) := (others => '0');

    -- video luma above black (0..255), Osc = none
    signal v_y1 : unsigned(7 downto 0) := (others => '0');
    signal v_ys : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    type t_u4 is array (natural range <>) of unsigned(3 downto 0);
    type t_u5 is array (natural range <>) of unsigned(4 downto 0);
    type t_u6 is array (natural range <>) of unsigned(5 downto 0);
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    type t_u9 is array (natural range <>) of unsigned(8 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal p1_adx : unsigned(9 downto 0) := (others => '0');
    signal p1_dif : signed(25 downto 0) := (others => '0');
    signal p1_asdx : unsigned(20 downto 0) := (others => '0');
    signal p1_bp : unsigned(1 downto 0) := (others => '0');
    signal p1_xi : unsigned(16 downto 0) := (others => '0');
    signal p2_u  : unsigned(9 downto 0) := (others => '0');
    signal p2_j  : unsigned(3 downto 0) := (others => '0');
    signal p2_ad : unsigned(24 downto 0) := (others => '0');
    signal p2_in : std_logic := '0';
    signal p2_darm : unsigned(11 downto 0) := (others => '0');
    signal p3_t  : unsigned(7 downto 0) := (others => '0');
    signal p3_in : std_logic := '0';
    signal p3_rr : unsigned(5 downto 0) := (others => '0');
    signal p3_fu : unsigned(3 downto 0) := (others => '0');
    signal p3_f2 : unsigned(4 downto 0) := (others => '0');
    signal p3_odd : std_logic := '0';
    signal p_fu  : t_u4(4 to 5) := (others => (others => '0'));
    signal p_f2  : t_u5(4 to 5) := (others => (others => '0'));
    signal p_odd : t_sl(4 to 5) := (others => '0');
    signal p_rr  : t_u6(4 to 7) := (others => (others => '0'));
    signal p4_lax, p5_lax, p6_lax : std_logic_vector(10 downto 0) := (others => '0');
    signal p5_pl, p5_ae, p5_ao : std_logic_vector(15 downto 0) := (others => '0');
    signal p5_cell : std_logic_vector(7 downto 0) := (others => '0');
    signal p6_lf : std_logic_vector(10 downto 0) := (others => '0');
    signal p6_pp : signed(13 downto 0) := (others => '0');
    signal p6_ai : unsigned(5 downto 0) := (others => '0');
    signal p6_rr, p6_gr : unsigned(4 downto 0) := (others => '0');
    signal p_cell : t_u8a(6 to 13) := (others => (others => '0'));
    signal p7_s1 : unsigned(10 downto 0) := (others => '0');
    signal p7_val : std_logic := '0';
    signal p7_pl : signed(14 downto 0) := (others => '0');
    signal p7_ag : unsigned(5 downto 0) := (others => '0');
    signal p7_lv : unsigned(6 downto 0) := (others => '0');
    signal p7_rl : unsigned(3 downto 0) := (others => '0');
    signal p_rrc, p_grc : t_u5(7 to 13) := (others => (others => '0'));
    signal p8_s2 : signed(12 downto 0) := (others => '0');
    signal p8_val : std_logic := '0';
    signal p8_pm : signed(20 downto 0) := (others => '0');
    signal p_ag : t_u6(8 to 10) := (others => (others => '0'));
    signal p9_fr : unsigned(7 downto 0) := (others => '0');
    signal p9_sat, p9_zero, p10_sat, p10_zero : std_logic := '0';
    signal p10_st : unsigned(7 downto 0) := (others => '0');
    signal p11_g3 : unsigned(7 downto 0) := (others => '0');
    signal p11_sr : unsigned(7 downto 0) := (others => '0');
    signal p12_zr, p13_zr : unsigned(7 downto 0) := (others => '0');
    signal p_isz, p_isr, p_win : t_sl(12 to 17) := (others => '0');
    signal p_isw : t_sl(12 to 13) := (others => '0');
    signal p14_gy, p14_gu, p14_gv : unsigned(9 downto 0) := (others => '0');
    signal p14_ry, p14_ru : unsigned(8 downto 0) := (others => '0');
    signal p14_rv : unsigned(7 downto 0) := (others => '0');
    signal p14_qy : unsigned(9 downto 0) := (others => '0');
    signal p14_qu, p14_qv : unsigned(8 downto 0) := (others => '0');
    signal p14_py, p14_pu, p14_pv : unsigned(9 downto 0) := (others => '0');
    signal p15_y, p15_u, p15_v : signed(11 downto 0) := (others => '0');
    signal p11_rkey : unsigned(9 downto 0) := (others => '0');
    signal p16_y : unsigned(10 downto 0) := (others => '0');
    signal p16_u, p16_v : signed(11 downto 0) := (others => '0');
    signal p16_dy, p16_du, p16_dv : signed(11 downto 0) := (others => '0');
    signal p16_ed : std_logic := '0';
    signal p17_y : unsigned(10 downto 0) := to_unsigned(64, 11);
    signal s_h18 : signed(11 downto 0) := (others => '0');
    signal p17_u8, p17_v8 : unsigned(7 downto 0) := to_unsigned(128, 8);
    -- output conditioning
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y18 : unsigned(10 downto 0) := to_unsigned(64, 11);
    signal s_y19, s_y20, s_y21, s_y22, s_y22x : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_yf23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel
    -- sync delay ring (C_LAT clocks): {0, field, vsync, hsync}
    type t_sr is array (0 to 31) of std_logic_vector(3 downto 0);
    signal sr : t_sr := (others => (others => '1'));
    attribute no_rw_check of sr : signal is true;
    signal sr_p, sr_wa, sr_ra : unsigned(4 downto 0) := (others => '0');
    signal sr_wd : std_logic_vector(3 downto 0) := (others => '1');
    signal sr_q, sr_q2 : std_logic_vector(3 downto 0) := (others => '1');

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered address and output; a fabric copy follows each)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_kcq <= C_KC(to_integer(r_kca));
            r_ptq <= C_PT(to_integer(r_pta));
            r_agq <= C_AGR(to_integer(r_aga));
            r_clq <= C_CELL(to_integer(r_cla));
            r_fq  <= C_LF(to_integer(r_fa));
            r_axq <= C_LAX(to_integer(r_axa));
            r_exq <= C_EXP(to_integer(r_exa));
            r_gyq <= C_GYUV(to_integer(r_gya));
        end if;
    end process p_rom;

    p_pl : process(clk)
    begin
        if rising_edge(clk) then
            pl_q <= pl(to_integer(pl_ra));
            if pl_we = '1' then pl(to_integer(pl_wa)) <= pl_wd; end if;
        end if;
    end process p_pl;
    p_ae : process(clk)
    begin
        if rising_edge(clk) then
            ae_q <= ae(to_integer(ae_ra));
            if ae_we = '1' then ae(to_integer(ab_wa)) <= ab_wd; end if;
        end if;
    end process p_ae;
    p_ao : process(clk)
    begin
        if rising_edge(clk) then
            ao_q <= ao(to_integer(ao_ra));
            if ao_we = '1' then ao(to_integer(ab_wa)) <= ab_wd; end if;
        end if;
    end process p_ao;
    p_wb : process(clk)
    begin
        if rising_edge(clk) then
            wb_q <= wb(to_integer(wb_ra));
            wb(to_integer(wb_wa)) <= wb_wd;
        end if;
    end process p_wb;
    p_sr : process(clk)
    begin
        if rising_edge(clk) then
            sr_q <= sr(to_integer(sr_ra));
            sr(to_integer(sr_wa)) <= sr_wd;
        end if;
    end process p_sr;
    -- sync delay ring: written every clock, read C_LAT - 3 entries back, then two registers
    p_srctl : process(clk)
    begin
        if rising_edge(clk) then
            sr_p <= sr_p + 1;
            sr_wa <= sr_p;
            sr_wd <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            sr_ra <= sr_p - (C_LAT - 3);
            sr_q2 <= sr_q;
        end if;
    end process p_srctl;
    p_crg : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= crg(to_integer(cr_ra));
            crg(to_integer(cr_wa)) <= cr_wd;
        end if;
    end process p_crg;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_id : integer range 0 to 15;
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(24 downto 0);
        variable v_pr : unsigned(43 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_ro : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            v_id := to_integer(unsigned(registers_in(8)(3 downto 0)));
            t_xs <= unsigned(C_T_XS(v_id)); t_ys <= unsigned(C_T_YS(v_id));
            t_dys <= unsigned(C_T_DYS(v_id)); t_dyt <= unsigned(C_T_DYT(v_id));
            t_xk <= unsigned(C_T_XK(v_id)); t_yk <= unsigned(C_T_YK(v_id));

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
                -- K4 Flow, K5 Roll: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 7 then s_s4 <= '0'; v_u := unsigned(v_x) - 8; s_m4 <= v_u(8 downto 0);
                elsif v_x < -7 then s_s4 <= '1'; v_u := unsigned(-v_x) - 8; s_m4 <= v_u(8 downto 0);
                else s_s4 <= '0'; s_m4 <= (others => '0'); end if;
                v_x := signed(resize(unsigned(registers_in(4)), 11)) - 512;
                if v_x > 7 then s_s5 <= '0'; v_u := unsigned(v_x) - 8; s_m5 <= v_u(8 downto 0);
                elsif v_x < -7 then s_s5 <= '1'; v_u := unsigned(-v_x) - 8; s_m5 <= v_u(8 downto 0);
                else s_s5 <= '0'; s_m5 <= (others => '0'); end if;
                f_bars  <= registers_in(6)(0);
                f_tear  <= registers_in(6)(1);
                f_stack <= registers_in(6)(2);
                f_alt   <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- zebra flow (Q.16 stripes) and row roll (Q.16 rows mod 28)
                if s_s4 = '1' then f_fl <= f_fl - resize(f_fst, 16); else f_fl <= f_fl + resize(f_fst, 16); end if;
                if s_s5 = '1' then
                    v_ro := ('0' & f_ro) - resize(f_rst, 22);
                    if v_ro(21) = '1' then f_ro <= v_ro(20 downto 0) + to_unsigned(1835008, 21); else f_ro <= v_ro(20 downto 0); end if;
                else
                    v_ro := ('0' & f_ro) + resize(f_rst, 22);
                    if v_ro >= 1835008 then f_ro <= v_ro(20 downto 0) - to_unsigned(1835008, 21); else f_ro <= v_ro(20 downto 0); end if;
                end if;
                -- tear jitter LFSR (x^16 + x^14 + x^13 + x^11 + 1)
                if s_lf(0) = '1' then s_lf <= ('0' & s_lf(15 downto 1)) xor x"B400"; else s_lf <= '0' & s_lf(15 downto 1); end if;
            end if;

            -- glides: K1, K2, K3, K6, P12 ease 1/8 of the gap per field (deadband 4); one channel per
            -- four zone clocks (4..23): select, difference, ease, write
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(5)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        if gl_d > 31 or gl_d < -31 then gl_n <= unsigned(v_ng(12 downto 0)); else gl_n <= gl_k; end if;
                        if gl_d > 31 or gl_d < -31 or (gl_d /= 0 and (gl_k(12 downto 3) = 0 or gl_k(12 downto 3) = 1023)) then
                            gl_w <= '1';
                        else
                            gl_w <= '0';
                        end if;
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

            -- ROM addresses for the K6 block scale and the K3 stripe gain, then fabric copies
            r_kca <= s_g(3)(12 downto 5);
            f_kc <= unsigned(r_kcq);
            -- K3 stripe gain = mantissa (16..31) x 2^e;  P12 beam gain in the log domain (128 = x 2):
            -- 2 (p - 512) below the middle (0 = off), (p - 512) / 4 above
            f_gm <= unsigned(C_MM(to_integer(s_g(2)(10 downto 5)))); f_ge <= s_g(2)(12 downto 11);
            v_k := s_g(4)(12 downto 3);
            if v_k(9) = '0' then f_lb <= shift_left(signed(resize(v_k, 12)) - 512, 1);
            else f_lb <= shift_right(signed(resize(v_k, 12)) - 512, 2); end if;
            f_lbm <= resize(f_lb, 13) - 1024;
            -- knob curves (shift-add) and switch constants
            v_k := s_g(0)(12 downto 3);
            f_s <= to_unsigned(205, 12) + resize(v_k & '0', 12) + resize(v_k(9 downto 9), 12);     -- 0.05 + 0.5 k (Q.12)
            v_k := s_g(1)(12 downto 3);
            f_rows <= to_unsigned(2048, 14) + resize(v_k & "0000", 14) - resize(v_k & '0', 14);  -- 8 + 56 k rows (Q.8)
            if f_stack = '1' then
                f_pvq <= to_signed(125158, 19); f_pvh <= to_signed(62579, 19); f_dy0 <= to_signed(-34668, 19);
            else
                f_pvq <= to_signed(250317, 19); f_pvh <= to_signed(125158, 19); f_dy0 <= to_signed(90491, 19);
            end if;
            f_sdx0 <= -signed(resize(f_sdxm, 22));
            f_b0 <= to_unsigned(C_SUPER, 17) - resize(f_b0m, 17);

            -- serial products: slot p = zc/32 (1..12), phase 0 load, 1..20 step, 24 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 12 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 12 and v_ph >= 1 and v_ph <= 20 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 12 loop
                if s_zone = '1' and v_p = k and v_ph = 24 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 1  => m_a <= resize(t_xk, 24);     m_b <= resize(f_s, 20);
                    when 2  => m_a <= to_unsigned(337060, 24); m_b <= resize(f_s, 20);
                    when 3  => m_a <= resize(t_yk, 24);     m_b <= resize(f_rows, 20);
                    when 4  => m_a <= resize(f_kc, 24);     m_b <= resize(t_xs, 20);
                    when 5  => m_a <= resize(f_kc, 24);     m_b <= to_unsigned(7488, 20);
                    when 6  => m_a <= resize(s_m4, 24);     m_b <= resize(s_m4, 20);
                    when 7  => m_a <= resize(f_m4sq, 24);   m_b <= to_unsigned(1132, 20);
                    when 8  => m_a <= resize(s_m5, 24);     m_b <= resize(s_m5, 20);
                    when 9  => m_a <= resize(f_m5sq, 24);   m_b <= to_unsigned(1132, 20);
                    when 10 => m_a <= resize(f_sdxm, 24);   m_b <= f_sdxm;
                    when 11 => m_a <= resize(f_sdxm, 24);   m_b <= resize(f_ks, 20);
                    when others => m_a <= resize(f_ks, 24); m_b <= resize(f_ks, 20);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 25) + resize(m_a, 25); else v_s := resize(m_h, 25); end if;
                m_h <= v_s(24 downto 1);
                m_l <= v_s(0) & m_l(19 downto 1);
                m_b <= '0' & m_b(19 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1)  = '1' then f_ks   <= v_pr(31 downto 20); end if;
            if q_wr(2)  = '1' then f_sdxm <= v_pr(31 downto 12); end if;
            if q_wr(3)  = '1' then f_dq   <= v_pr(31 downto 16); end if;
            if q_wr(4)  = '1' then f_incb <= v_pr(23 downto 14); end if;
            if q_wr(5)  = '1' then f_b0m  <= v_pr(27 downto 12); end if;
            if q_wr(6)  = '1' then f_m4sq <= v_pr(17 downto 0); end if;
            if q_wr(7)  = '1' then f_fst  <= v_pr(28 downto 16); end if;
            if q_wr(8)  = '1' then f_m5sq <= v_pr(17 downto 0); end if;
            if q_wr(9)  = '1' then f_rst  <= v_pr(28 downto 16); end if;
            if q_wr(10) = '1' then f_e0   <= v_pr(35 downto 12); end if;
            if q_wr(11) = '1' then f_sk   <= v_pr(31 downto 0); end if;
            if q_wr(12) = '1' then f_kk   <= v_pr(23 downto 0); end if;
            -- square DDA start: F0 = kS^2 - 2 SDXm kS, step 2 kS^2
            f_f0 <= signed(resize(f_kk, 32)) - signed(shift_left(f_sk, 1));
            f_k2 <= shift_left(resize(f_kk, 25), 1);

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
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
    -- Per-line scalar engine: the next line's row, beam dy, table row, tear and
    -- block start; line 0 in the vblank zone, then advanced after each source
    -- avid rise (which commits the prepared line)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_q  : unsigned(21 downto 0);
        variable v_dy : signed(19 downto 0);
        variable v_b  : signed(17 downto 0);
        variable v_j2 : unsigned(1 downto 0);
        variable v_ab : unsigned(18 downto 0);
        variable v_sq : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            e_go <= '0';
            if s_zone = '1' and s_zc = 900 then
                v_q := ('0' & f_ro) + to_unsigned(1835008 - 4869, 22);
                if v_q >= 1835008 then n_q <= v_q(20 downto 0) - to_unsigned(1835008, 21); else n_q <= v_q(20 downto 0); end if;
                n_dy <= f_dy0; n_yq <= (others => '0'); n_bank <= '0';
                le_st <= to_unsigned(3, 3);
            elsif data_in.avid = '1' and s_avq = '0' then
                le_st <= to_unsigned(1, 3);
            elsif le_st /= 0 then
                if le_st = 6 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
            end if;
            -- serial square of |dy| (17 steps) for the beam field, started once dy is final
            if (s_zone = '1' and s_zc = 901) or le_st = 2 then
                v_ab := unsigned(abs(n_dy));
                sq_a <= v_ab(16 downto 0); sq_b <= v_ab(16 downto 0);
                sq_h <= (others => '0'); sq_l <= (others => '0'); sq_n <= to_unsigned(17, 5);
            elsif sq_n /= 0 then
                if sq_b(0) = '1' then v_sq := ('0' & sq_h) + ('0' & sq_a); else v_sq := '0' & sq_h; end if;
                sq_h <= v_sq(17 downto 1);
                sq_l <= v_sq(0) & sq_l(16 downto 1);
                sq_b <= '0' & sq_b(16 downto 1);
                sq_n <= sq_n - 1;
            end if;
            case to_integer(le_st) is
                when 1 =>
                    v_q := ('0' & n_q) + resize(f_dq, 22);
                    if v_q >= 1835008 then n_q <= v_q(20 downto 0) - to_unsigned(1835008, 21); else n_q <= v_q(20 downto 0); end if;
                    v_dy := resize(n_dy, 20) + signed(resize(t_dys, 20));
                    if v_dy >= resize(f_pvh, 20) then n_dy <= resize(v_dy - resize(f_pvq, 20), 19); else n_dy <= resize(v_dy, 19); end if;
                    n_yq <= n_yq + resize(t_dyt, 22);
                    n_bank <= not n_bank;
                when 3 =>
                    n_r <= n_q(20 downto 16);
                    n_fyh <= n_q(15);
                when 4 =>
                    n_sh <= C_SHIFT(to_integer(n_r))(0) and f_tear;
                    n_tr <= signed(C_TEAR(to_integer(n_r)));
                    case to_integer(n_r(2 downto 0)) is
                        when 0 => v_j2 := s_lf(1 downto 0);   when 1 => v_j2 := s_lf(3 downto 2);
                        when 2 => v_j2 := s_lf(5 downto 4);   when 3 => v_j2 := s_lf(7 downto 6);
                        when 4 => v_j2 := s_lf(9 downto 8);   when 5 => v_j2 := s_lf(11 downto 10);
                        when 6 => v_j2 := s_lf(13 downto 12); when others => v_j2 := s_lf(15 downto 14);
                    end case;
                    n_jt <= signed('0' & v_j2) - 1;
                when 5 =>
                    if f_tear = '1' then n_tear <= resize(n_tr, 9) + resize(n_jt, 9); else n_tear <= (others => '0'); end if;
                    if n_sh = '1' then n_bx0 <= signed(resize(f_b0, 18)) - 24064; else n_bx0 <= signed(resize(f_b0, 18)); end if;
                when 6 =>
                    if n_bx0 < 0 then n_bx0 <= n_bx0 + C_SUPER; end if;
                    e_go <= '1';
                when others => null;
            end case;
            if data_in.avid = '1' and s_avq = '0' then
                l_r <= n_r; l_fyh <= n_fyh; l_dy <= n_dy; l_tear <= n_tear; l_bx0 <= unsigned(n_bx0(16 downto 0));
                l_dy2 <= sq_h & sq_l(16 downto 12); l_ady <= sq_a;
                l_bank <= n_bank;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Table engine: the next line's zebra phase (64 x {value, slope}) and the
    -- amplitude / ramp line (32 entries, even / odd banks), each lerped between two
    -- table rows.  10 clocks per entry (k): 0/1 row addresses, 3 hold row iy,
    -- 4 differences (and 3x), 5..7 radix-4 serial product with the fraction,
    -- 8 values and writes, 9 last zebra word; 640 clocks per line (SD: 858)
    ---------------------------------------------------------------------------
    p_tab : process(clk)
        variable v_v  : signed(17 downto 0);
        variable v_sl : signed(9 downto 0);
        variable v_a  : signed(15 downto 0);
        variable v_fb : unsigned(1 downto 0);
        variable v_tP : signed(10 downto 0);
        variable v_tA : signed(8 downto 0);
        variable v_tR, v_tG : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            pl_we <= '0'; ae_we <= '0'; ao_we <= '0';
            if e_go = '1' then
                e_run <= '1'; e_c <= (others => '0'); e_k <= (others => '0'); e_first <= '1';
                e_iy <= n_yq(20 downto 16); e_f <= n_yq(15 downto 10); e_bank <= n_bank;
                if n_yq(21 downto 16) >= 31 then e_iy <= to_unsigned(31, 5); e_iy1 <= to_unsigned(31, 5);
                else e_iy1 <= n_yq(20 downto 16) + 1; end if;
            elsif e_run = '1' then
                if e_k = 9 then
                    e_k <= (others => '0'); e_c <= e_c + 1;
                    if e_c = 63 then e_run <= '0'; end if;
                else
                    e_k <= e_k + 1;
                end if;
                case to_integer(e_k) is
                    when 0 => r_pta <= e_iy & e_c;  r_aga <= e_iy & e_c(4 downto 0);
                    when 1 => r_pta <= e_iy1 & e_c; r_aga <= e_iy1 & e_c(4 downto 0);
                    when 3 => e_pa <= e_pd; e_aa <= e_ad;
                    when 4 =>
                        v_tP := signed(resize(e_pd, 11)) - signed(resize(e_pa, 11));
                        e_dP <= v_tP; e_dP3 <= shift_left(v_tP, 2) - v_tP;
                        v_tA := signed(resize(unsigned(e_ad(15 downto 10)), 9)) - signed(resize(unsigned(e_aa(15 downto 10)), 9));
                        e_dA <= v_tA; e_dA3 <= shift_left(v_tA, 2) - v_tA;
                        v_tR := signed(resize(unsigned(e_ad(9 downto 5)), 8)) - signed(resize(unsigned(e_aa(9 downto 5)), 8));
                        e_dR <= v_tR; e_dR3 <= shift_left(v_tR, 2) - v_tR;
                        v_tG := signed(resize(unsigned(e_ad(4 downto 0)), 8)) - signed(resize(unsigned(e_aa(4 downto 0)), 8));
                        e_dG <= v_tG; e_dG3 <= shift_left(v_tG, 2) - v_tG;
                        e_xP <= (others => '0'); e_xA <= (others => '0'); e_xR <= (others => '0'); e_xG <= (others => '0');
                    when 5 | 6 | 7 =>
                        if e_k = 5 then v_fb := e_f(5 downto 4); elsif e_k = 6 then v_fb := e_f(3 downto 2); else v_fb := e_f(1 downto 0); end if;
                        case to_integer(v_fb) is
                            when 0 => e_xP <= shift_left(e_xP, 2); e_xA <= shift_left(e_xA, 2); e_xR <= shift_left(e_xR, 2); e_xG <= shift_left(e_xG, 2);
                            when 1 => e_xP <= shift_left(e_xP, 2) + e_dP; e_xA <= shift_left(e_xA, 2) + e_dA;
                                      e_xR <= shift_left(e_xR, 2) + e_dR; e_xG <= shift_left(e_xG, 2) + e_dG;
                            when 2 => e_xP <= shift_left(e_xP, 2) + shift_left(resize(e_dP, 17), 1); e_xA <= shift_left(e_xA, 2) + shift_left(resize(e_dA, 15), 1);
                                      e_xR <= shift_left(e_xR, 2) + shift_left(resize(e_dR, 14), 1); e_xG <= shift_left(e_xG, 2) + shift_left(resize(e_dG, 14), 1);
                            when others => e_xP <= shift_left(e_xP, 2) + e_dP3; e_xA <= shift_left(e_xA, 2) + e_dA3;
                                      e_xR <= shift_left(e_xR, 2) + e_dR3; e_xG <= shift_left(e_xG, 2) + e_dG3;
                        end case;
                    when 8 =>
                        v_v := signed(resize(e_pa, 18)) + resize(shift_right(e_xP, 6), 18);
                        if e_first = '0' then
                            v_sl := signed('0' & std_logic_vector(v_v(7 downto 0))) - signed(resize(e_pprev, 10));
                            if v_sl > 127 then v_sl := to_signed(127, 10); elsif v_sl < -128 then v_sl := to_signed(-128, 10); end if;
                            pl_wa <= e_bank & (e_c - 1);
                            pl_wd <= std_logic_vector(e_pprev) & std_logic_vector(v_sl(7 downto 0));
                            pl_we <= '1';
                        end if;
                        e_first <= '0';
                        e_pprev <= unsigned(v_v(7 downto 0));
                        if e_c < 32 then
                            v_a := signed(resize(unsigned(e_aa(15 downto 10)), 16)) + resize(shift_right(e_xA, 6), 16);
                            ab_wd(15 downto 10) <= std_logic_vector(v_a(5 downto 0));
                            v_a := signed(resize(unsigned(e_aa(9 downto 5)), 16)) + resize(shift_right(e_xR, 6), 16);
                            ab_wd(9 downto 5) <= std_logic_vector(v_a(4 downto 0));
                            v_a := signed(resize(unsigned(e_aa(4 downto 0)), 16)) + resize(shift_right(e_xG, 6), 16);
                            ab_wd(4 downto 0) <= std_logic_vector(v_a(4 downto 0));
                            ab_wa <= e_bank & e_c(4 downto 1);
                            if e_c(0) = '0' then ae_we <= '1'; else ao_we <= '1'; end if;
                        end if;
                    when 9 =>
                        if e_c = 63 then
                            pl_wa <= e_bank & "111111"; pl_wd <= std_logic_vector(e_pprev) & x"00"; pl_we <= '1';
                        end if;
                    when others => null;
                end case;
            end if;
            -- ROM output, then fabric copies
            e_pd <= unsigned(r_ptq); e_ad <= r_agq;
        end if;
    end process p_tab;

    ---------------------------------------------------------------------------
    -- Beam DDAs: virtual x (Q.8), s dx (Q.8 of c / 256 units), block x (Q.6);
    -- a free-running pixel clock (stage-k pixel x = pcf - k) for the key line buffer
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_bx : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xv <= (others => '0'); b_sdx <= f_sdx0; b_bx <= l_bx0; b_pcf <= (others => '0');
                b_e <= signed(resize(f_e0, 25)); b_f <= f_f0;
            else
                b_pcf <= b_pcf + 1;
                if g_avid = '1' then
                    b_xv <= b_xv + resize(t_xs, 20);
                    b_sdx <= b_sdx + signed(resize(f_ks, 22));
                    b_e <= b_e + resize(shift_right(b_f, 12), 25);
                    b_f <= b_f + signed(resize(f_k2, 32));
                    v_bx := resize(b_bx, 18) + resize(f_incb, 18);
                    if v_bx >= C_SUPER then b_bx <= v_bx(16 downto 0) - to_unsigned(C_SUPER, 17); else b_bx <= v_bx(16 downto 0); end if;
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_dx  : signed(20 downto 0);
        variable v_ad  : unsigned(20 downto 0);
        variable v_d   : signed(19 downto 0);
        variable v_u   : signed(11 downto 0);
        variable v_c   : unsigned(1 downto 0);
        variable v_dd  : signed(21 downto 0);
        variable v_ud  : unsigned(21 downto 0);
        variable v_i2  : unsigned(5 downto 0);
        variable v_lo, v_hi : std_logic_vector(15 downto 0);
        variable v_l   : signed(13 downto 0);
        variable v_sh  : signed(20 downto 0);
        variable v_a   : unsigned(7 downto 0);
        variable v_5a  : signed(10 downto 0);
        variable v_sr  : unsigned(13 downto 0);
        variable v_o   : unsigned(7 downto 0);
        variable v_code : unsigned(3 downto 0);
        variable v_kc  : unsigned(3 downto 0);
        variable v_key : unsigned(9 downto 0);
        variable v_rs  : unsigned(8 downto 0);
        variable v_isz, v_isr, v_isw, v_win : std_logic;
        variable v_ri  : unsigned(7 downto 0);
        variable v_r18 : unsigned(17 downto 0);
        variable v_pw  : std_logic_vector(29 downto 0);
        variable v_ed  : std_logic;
        variable v_y, v_cu, v_cv : signed(11 downto 0);
        variable v_t15 : signed(14 downto 0);
        variable v_yc  : unsigned(10 downto 0);
        variable v_yh  : unsigned(10 downto 0);
        variable v_yp  : unsigned(19 downto 0);
        variable v_sc  : unsigned(11 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_vy  : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            -- camera luma above black, 0..255 (x 0.293); Osc = none.  Untapped delay: the camera
            -- reaches the zebra and the bloom a few pixels late (invisible in a contour bend).
            if f_osc = '1' or unsigned(data_in.y) <= 64 then
                v_y1 <= (others => '0');
            else
                v_vy := (unsigned(data_in.y) - 64) * to_unsigned(75, 7);
                if v_vy(16) = '1' then v_y1 <= to_unsigned(255, 8); else v_y1 <= v_vy(15 downto 8); end if;
            end if;
            -- horizontal slew of the camera luma (an analog low-pass, ~8 px): s += y - s/8
            v_ys <= v_ys + resize(v_y1, 11) - resize(v_ys(10 downto 3), 11);

            -- 1: |dx|; dy^2 - (s dx)^2 (Q.16 of c / 256 units); |s dx|; block period and offset
            v_dx := signed('0' & b_xv) - C_XC;
            if v_dx < 0 then v_ad := unsigned(-v_dx); else v_ad := unsigned(v_dx); end if;
            p1_adx <= v_ad(17 downto 8);
            p1_dif <= signed(resize(l_dy2, 26)) - resize(b_e, 26);
            p1_asdx <= resize(unsigned(abs(b_sdx)), 21);
            if b_bx >= 78336 then p1_bp <= "10"; p1_xi <= b_bx - 78336;
            elsif b_bx >= 39168 then p1_bp <= "01"; p1_xi <= b_bx - 39168;
            else p1_bp <= "00"; p1_xi <= b_bx; end if;

            -- 2: folded, torn u; block column; |dy^2 - (s dx)^2|; arm distance ||dy| - |s dx||; falloff ROM address
            v_u := signed(resize(p1_adx, 12)) + resize(l_tear, 12);
            if v_u < 0 then p2_u <= (others => '0');
            elsif v_u > 1023 then p2_u <= (others => '1');
            else p2_u <= unsigned(v_u(9 downto 0)); end if;
            if p1_xi >= 34240 then v_c := "10"; elsif p1_xi >= 19520 then v_c := "01"; else v_c := "00"; end if;
            p2_j <= resize(p1_bp & '0', 4) + resize(p1_bp, 4) + resize(v_c, 4);
            if p1_dif < 0 then p2_ad <= resize(unsigned(-p1_dif), 25); p2_in <= '1';
            else p2_ad <= resize(unsigned(p1_dif), 25); p2_in <= '0'; end if;
            v_dd := signed(resize(l_ady, 22)) - signed(resize(p1_asdx, 22));
            v_ud := unsigned(abs(v_dd));
            p2_darm <= resize(v_ud(21 downto 8), 12);
            if p1_adx(9 downto 2) = 255 then r_axa <= to_unsigned(255, 8); else r_axa <= p1_adx(9 downto 2); end if;

            -- 3: t = |dy^2 - (s dx)^2| / 2^24 (clamped), inside flag; ridge; line RAM and cell ROM addresses
            if p2_ad(24 downto 20) /= 0 then p3_t <= (others => '1'); else p3_t <= p2_ad(19 downto 12); end if;
            p3_in <= p2_in;
            if p2_darm < 60 then p3_rr <= resize(to_unsigned(60, 12) - p2_darm, 6); else p3_rr <= (others => '0'); end if;
            pl_ra <= l_bank & p2_u(9 downto 4);
            v_i2 := ('0' & p2_u(9 downto 5)) + 1;
            if v_i2(5) = '1' then ae_ra <= l_bank & "1111"; else ae_ra <= l_bank & v_i2(4 downto 1); end if;
            ao_ra <= l_bank & p2_u(9 downto 6);
            r_cla <= l_r & p2_j;
            p3_fu <= p2_u(3 downto 0); p3_f2 <= p2_u(4 downto 0); p3_odd <= p2_u(5);

            -- 4: log-profile ROM address; (line RAM / cell / log-falloff ROM output registers)
            r_fa <= p3_in & p3_t;
            p4_lax <= r_axq;
            p_fu(4) <= p3_fu; p_f2(4) <= p3_f2; p_odd(4) <= p3_odd; p_rr(4) <= p3_rr;

            -- 5: fabric copies of the line RAMs and the cell ROM
            p5_pl <= pl_q; p5_ae <= ae_q; p5_ao <= ao_q; p5_cell <= r_clq; p5_lax <= p4_lax;
            p_fu(5) <= p_fu(4); p_f2(5) <= p_f2(4); p_odd(5) <= p_odd(4); p_rr(5) <= p_rr(4);

            -- 6: log-profile copy; zebra phase lerp (value + slope x fu); amplitude / ramp lerps
            p6_lf <= r_fq; p6_lax <= p5_lax;
            p6_pp <= shift_left(resize(signed('0' & p5_pl(15 downto 8)), 14), 4) + resize(signed(p5_pl(7 downto 0)) * signed('0' & p_fu(5)), 14);
            if p_odd(5) = '1' then v_lo := p5_ao; v_hi := p5_ae; else v_lo := p5_ae; v_hi := p5_ao; end if;
            v_l := signed(resize(unsigned(v_lo(15 downto 10)), 14)) + shift_right(resize((signed('0' & v_hi(15 downto 10)) - signed('0' & v_lo(15 downto 10))) * signed('0' & p_f2(5)(4 downto 2)), 14), 3);
            p6_ai <= unsigned(v_l(5 downto 0));
            v_l := signed(resize(unsigned(v_lo(9 downto 5)), 14)) + shift_right(resize((signed('0' & v_hi(9 downto 5)) - signed('0' & v_lo(9 downto 5))) * signed('0' & p_f2(5)(4 downto 2)), 14), 3);
            p6_rr <= unsigned(v_l(4 downto 0));
            v_l := signed(resize(unsigned(v_lo(4 downto 0)), 14)) + shift_right(resize((signed('0' & v_hi(4 downto 0)) - signed('0' & v_lo(4 downto 0))) * signed('0' & p_f2(5)(4 downto 2)), 14), 3);
            p6_gr <= unsigned(v_l(4 downto 0));
            p_cell(6) <= unsigned(p5_cell); p_rr(6) <= p_rr(5);

            -- 7: log beam = log falloff + log profile; zebra + slewed camera (1 stripe full scale); amplitude
            -- gamma; ramp key sum; camera bloom log
            p7_s1 <= resize(unsigned(p6_lax(9 downto 0)), 11) + resize(unsigned(p6_lf(9 downto 0)), 11);
            p7_val <= p6_lax(10) and p6_lf(10);
            p7_pl <= resize(p6_pp, 15) + signed(resize(v_ys(10 downto 3), 15));
            p7_ag <= unsigned(C_AG6(to_integer(p6_ai)));
            p7_lv <= unsigned(C_LV(to_integer(v_ys(10 downto 6))));
            p7_rl <= resize(p_rr(6)(5 downto 3), 4) + resize(p_rr(6)(5 downto 4), 4);
            p_rrc(7) <= p6_rr; p_grc(7) <= p6_gr;
            p_cell(7) <= p_cell(6); p_rr(7) <= p_rr(6);

            -- 8: log beam - 8 octaves + P12 gain + ridge + bloom; stripe gain mantissa product
            p8_s2 <= (signed(resize(p7_s1, 13)) + f_lbm) + (signed(resize(p7_rl, 13)) + signed(resize(p7_lv, 13)));
            p8_val <= p7_val;
            p8_pm <= p7_pl * signed('0' & f_gm);
            p_ag(8) <= p7_ag;

            -- 9: exp ROM address (clamped); stripe phase = (pl x m) >> (6 - e) + flow
            p9_zero <= '0'; p9_sat <= '0';
            if p8_val = '0' or p8_s2 < 0 then r_exa <= (others => '0'); p9_zero <= '1';
            elsif p8_s2 > 1023 then r_exa <= (others => '1'); p9_sat <= '1';
            else r_exa <= unsigned(p8_s2(9 downto 0)); end if;
            case to_integer(f_ge) is
                when 0 => v_sh := shift_right(p8_pm, 6);
                when 1 => v_sh := shift_right(p8_pm, 5);
                when 2 => v_sh := shift_right(p8_pm, 4);
                when others => v_sh := shift_right(p8_pm, 3);
            end case;
            p9_fr <= unsigned(v_sh(7 downto 0)) + f_fl(15 downto 8);

            -- 10: (exp ROM output register) stripe profile clip(5 |f - 128| - 192)
            if p9_fr(7) = '1' then v_a := p9_fr - 128; else v_a := 128 - p9_fr; end if;
            v_5a := signed(resize(v_a & "00", 11)) + signed(resize(v_a, 11)) - 192;
            if v_5a < 0 then p10_st <= (others => '0');
            elsif v_5a > 255 then p10_st <= (others => '1');
            else p10_st <= unsigned(v_5a(7 downto 0)); end if;
            p10_zero <= p9_zero; p10_sat <= p9_sat;

            -- 11: beam level G3; zebra stripe x amplitude
            if p10_zero = '1' then p11_g3 <= (others => '0');
            elsif p10_sat = '1' then p11_g3 <= (others => '1');
            else p11_g3 <= unsigned(r_exq); end if;
            v_sr := p10_st * p_ag(10);
            p11_sr <= v_sr(13 downto 6);
            v_rs := resize(p_rrc(10) & "000", 9) + resize(p_grc(10) & "000", 9);
            p11_rkey <= resize(v_rs, 10) + resize(v_rs(8 downto 3), 10);

            -- 12: zebra red; layer decode; key compare; beam colour ROM address
            v_o := p11_sr;
            p12_zr <= to_unsigned(12, 8) + v_o - resize(v_o(7 downto 3), 8);
            v_code := p_cell(11)(7 downto 4); v_kc := p_cell(11)(3 downto 0);
            v_isz := '0'; v_isr := '0'; v_isw := '0';
            if f_bars = '0' or v_code = 10 then v_isz := '1';
            elsif v_code = 9 or (v_code = 11 and l_fyh = '0') then v_isr := '1';
            elsif v_code = 11 then v_isw := '1'; end if;
            if v_isr = '1' then v_key := p11_rkey;
            elsif v_isw = '1' or v_kc = 15 then v_key := (others => '1');
            else v_key := resize(v_kc & "1000", 10); end if;
            if v_isz = '0' and resize(p11_g3, 10) > v_key then v_win := '1'; else v_win := '0'; end if;
            p_isz(12) <= v_isz; p_isr(12) <= v_isr; p_isw(12) <= v_isw; p_win(12) <= v_win;
            if v_isz = '1' and p11_g3 < 9 then r_gya <= to_unsigned(9, 8); else r_gya <= p11_g3; end if;

            -- 13: (beam colour ROM output register)
            p13_zr <= p12_zr;

            -- 14: beam colour copy; red part (zebra red or ramp red) and ramp green part; palette
            p14_gy <= unsigned(r_gyq(29 downto 20)); p14_gu <= unsigned(r_gyq(19 downto 10)); p14_gv <= unsigned(r_gyq(9 downto 0));
            if p_isz(13) = '1' then v_ri := p13_zr; else v_ri := p_rrc(13) & "000"; end if;
            v_r18 := shift_left(resize(v_ri, 18), 8) + shift_left(resize(v_ri, 18), 6) + shift_left(resize(v_ri, 18), 2);   -- 324 v (263 x 16/13)
            p14_ry <= v_r18(16 downto 8);
            v_r18 := shift_left(resize(v_ri, 18), 9) - shift_left(resize(v_ri, 18), 6) + shift_left(resize(v_ri, 18), 1);   -- 450 v
            p14_ru <= v_r18(16 downto 8);
            v_r18 := shift_left(resize(v_ri, 18), 7) + shift_left(resize(v_ri, 18), 4) + shift_left(resize(v_ri, 18), 3);   -- 152 v
            p14_rv <= v_r18(15 downto 8);
            v_r18 := resize(p_grc(13) * to_unsigned(5079, 13), 18);                                        -- 516 x 8 x 16/13
            p14_qy <= v_r18(17 downto 8);
            v_r18 := resize(p_grc(13) * to_unsigned(3016, 12), 18);
            p14_qu <= v_r18(16 downto 8);
            v_r18 := resize(p_grc(13) * to_unsigned(2384, 12), 18);
            p14_qv <= v_r18(16 downto 8);
            if p_isw(13) = '1' then v_pw := C_PYUV(4); else v_pw := C_PYUV(to_integer(p_cell(13)(7 downto 4))); end if;
            p14_py <= unsigned(v_pw(29 downto 20)); p14_pu <= unsigned(v_pw(19 downto 10)); p14_pv <= unsigned(v_pw(9 downto 0));

            -- 15: composite (zebra: beam colour + red part, i.e. max per channel; beam over bars / ramps
            -- where it wins the key; ramps; bars)
            if p_isz(14) = '1' then
                p15_y <= signed(resize(p14_gy, 12)) + signed(resize(p14_ry, 12));
                p15_u <= signed(resize(p14_gu, 12)) + signed(resize(p14_ru, 12));
                p15_v <= signed(resize(p14_gv, 12)) - signed(resize(p14_rv, 12));
            elsif p_win(14) = '1' then
                p15_y <= signed(resize(p14_gy, 12)); p15_u <= signed(resize(p14_gu, 12)); p15_v <= signed(resize(p14_gv, 12));
            elsif p_isr(14) = '1' then
                p15_y <= 64 + signed(resize(p14_ry, 12)) + signed(resize(p14_qy, 12));
                p15_u <= 512 + signed(resize(p14_ru, 12)) - signed(resize(p14_qu, 12));
                p15_v <= 512 - signed(resize(p14_rv, 12)) - signed(resize(p14_qv, 12));
            else
                p15_y <= signed(resize(p14_py, 12)); p15_u <= signed(resize(p14_pu, 12)); p15_v <= signed(resize(p14_pv, 12));
            end if;
            -- key line buffer: read the stage-13 pixel's x (its flag arrives as it reaches stage 15);
            -- write the stage-17 pixel's flag four clocks after its read
            wb_ra <= b_pcf - 13;
            wb_wa <= b_pcf - 17; wb_wd(0) <= p_win(17);

            -- 16: outline flag (the key flag differs within +-2 px or from the line above); offsets
            v_ed := '0';
            if p_win(15) /= p_win(13) or p_win(15) /= p_win(14) or p_win(15) /= p_win(16) or p_win(15) /= p_win(17)
               or p_win(15) /= wb_q(0) then v_ed := '1'; end if;
            p16_ed <= v_ed and not p_isz(15);
            p16_y <= unsigned(resize(p15_y, 11)); p16_u <= p15_u; p16_v <= p15_v;
            p16_dy <= p15_y - 64; p16_du <= p15_u - 512; p16_dv <= p15_v - 512;

            -- 17: outline x 7/16 toward black / neutral; palette swap exchanges Cr and Cb; 8-bit chroma
            if p16_ed = '1' then
                p17_y <= unsigned(resize(signed(resize(p16_y, 12)) - shift_right(p16_dy, 1) - shift_right(p16_dy, 4), 11));
                v_cu := p16_u - shift_right(p16_du, 1) - shift_right(p16_du, 4);
                v_cv := p16_v - shift_right(p16_dv, 1) - shift_right(p16_dv, 4);
            else
                p17_y <= p16_y; v_cu := p16_u; v_cv := p16_v;
            end if;
            if f_alt = '1' then v_y := v_cv; v_cv := v_cu; v_cu := v_y; end if;
            if v_cu < 0 then p17_u8 <= (others => '0'); elsif v_cu > 1023 then p17_u8 <= (others => '1'); else p17_u8 <= unsigned(v_cu(9 downto 2)); end if;
            if v_cv < 0 then p17_v8 <= (others => '0'); elsif v_cv > 1023 then p17_v8 <= (others => '1'); else p17_v8 <= unsigned(v_cv(9 downto 2)); end if;
            for i in 13 to 17 loop p_win(i) <= p_win(i - 1); p_isz(i) <= p_isz(i - 1); p_isr(i) <= p_isr(i - 1); end loop;
            p_isw(13) <= p_isw(12);
            for i in 9 to 10 loop p_ag(i) <= p_ag(i - 1); end loop;
            for i in 8 to 13 loop p_cell(i) <= p_cell(i - 1); p_rrc(i) <= p_rrc(i - 1); p_grc(i) <= p_grc(i - 1); end loop;

            -- 18..23: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 from 8-bit chroma and a delay-8
            -- ring; luma [1 2 1]); centres 5 clocks after stage 17.  In the vblank zone the ring is
            -- refilled with neutral and the box sums reset, so a stray value can never persist.
            cr_p <= cr_p + 1;
            cr_wa <= cr_p; cr_ra <= cr_p - 6;
            if s_zone = '1' then
                cr_wd <= x"8080";
                s_boxu <= to_unsigned(1024, 11); s_boxv <= to_unsigned(1024, 11);
                s_boxud <= to_unsigned(1024, 11); s_boxvd <= to_unsigned(1024, 11);
            else
                cr_wd <= std_logic_vector(p17_u8) & std_logic_vector(p17_v8);
                s_boxu <= s_boxu + resize(p17_u8, 11) - resize(unsigned(cr_q(15 downto 8)), 11);
                s_boxv <= s_boxv + resize(p17_v8, 11) - resize(unsigned(cr_q(7 downto 0)), 11);
                s_boxud <= s_boxu; s_boxvd <= s_boxv;
            end if;
            v_sc := resize(s_boxu, 12) + s_boxud; s_u23 <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 12) + s_boxvd; s_v23 <= v_sc(11 downto 2);
            -- luma knee over two of the delay stages (sources carry the 16/13 pre-boost for the output's
            -- 13/16; compressed above 724 so drawn white lands near 777)
            s_y18 <= p17_y; s_h18 <= signed(resize(p17_y, 12)) - 724;
            if s_h18 > 0 then
                v_yc := to_unsigned(724, 11) + resize(unsigned(s_h18(10 downto 1)), 11) + resize(unsigned(s_h18(10 downto 6)), 11);
            else
                v_yc := s_y18;
            end if;
            s_y19 <= v_yc(9 downto 0);
            s_y20 <= s_y19; s_y21 <= s_y20; s_y22 <= s_y21; s_y22x <= s_y22;
            v_sy := resize(s_y21, 12) + shift_left(resize(s_y22, 12), 1) + resize(s_y22x, 12);
            s_yf23 <= v_sy(11 downto 2);
            -- 24: luma toward black (white 940 -> 777); blanking gate on the latency-aligned generated avid
            if s_yf23 > 64 then v_dl := s_yf23 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(22) = '1' then
                s_y24 <= s_yf23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= sr_q2(0);
    data_out.vsync_n <= sr_q2(1);
    data_out.field_n <= sr_q2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture crossbeam;
