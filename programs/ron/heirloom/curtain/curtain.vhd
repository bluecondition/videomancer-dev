-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Curtain v0.5.0 -- Heirloom #34 (2026-10-27 rebuild), from WIN_20230521_01_34_12.
--
-- The photo: two line-locked horizontal oscillators into a keyed colourizer.
--   A  the FOLD oscillator (608 px), phase-modulated by a vertical parabola (the drape sweep
--      dx(y) = 0.0012 y^2 + 0.26 y), owns the upper half: green arcs (three contours of A,
--      cyan fringe left, navy fringe right), red/orange columns between them, cyan pools and
--      striped bands;
--   B  the PLEAT oscillator (542 px, a sawtooth reset 15 px before active video).  Its resets
--      cut the upper half (an arc lives only while B - dx > 0.35, red "flames" run on after
--      every cut for a length set by A at the cut, grey domes sit before the cut) and own the
--      lower half: red pleats with dark pleat lines, a striped drape, a shadow dome.
--   The fine magenta/purple striation is a third oscillator (36 px) on the blue channel; it
--   is also added into the keys, so every key edge is torn into stripe-aligned teeth.  Upper
--   half: its per-line phase is a random piecewise-linear walk (chevrons); lower half: a
--   steady slant that steepens toward each cut.
-- Each half's key field K and red shade R are photo-fitted tables over (oscillator phase,
-- row) (curtain.py -> curtain_pkg.vhd), interpolated in both directions: rows by the
-- per-line engine into a line RAM, phase per pixel.
--
--   K1  Folds    fold oscillator: 1.6 .. 8 folds (3.2 = the photo)
--   K2  Pleats   pleat oscillator: 1.8 .. 9 pleats (3.5 = the photo)
--   K3  Stripes  stripe period 72 .. 14 px (36 = the photo)
--   K4  Speed    bipolar (still in the centre): folds slide past the cuts, stripes stream
--   K5  Pools    pool key level: none .. flooding
--   K6  Melt     how far the camera (or Osc) bends the keys and the stripes
--   S7  Tear     On / Off      S8  Piping  On / Off (green arcs)
--   S9  Flames   On / Off      S10 Velvet  Red / Blue (R <-> B)
--   S11 Source   Video / Osc
--   P12 DRAPE    the sweep: 0 = straight hanging folds, 50 % = the photo, 100 % = whipped
--
-- Virtual 1080-line coordinates (timing-ID steps); generated avid; output conditioning
-- ([1 2 1] luma, [1 2 2 2 2 2 2 2 1]/16 chroma, luma 13/16); blanking gate.  Latency 24
-- clocks in every mode.  Colours: u = Cr, v = Cb (hardware convention).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.curtain_pkg.all;
use work.all;

architecture curtain of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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

    type t_dome is array (0 to 31) of std_logic_vector(23 downto 0);
    constant C_DOME : t_dome := (
        x"5A7D96", x"5C7D96", x"5F7D95", x"627D94", x"657C93", x"687C93", x"6A7C92", x"6D7B91",
        x"707B90", x"737B8F", x"767A8F", x"787A8E", x"7B7A8D", x"7E798C", x"81798C", x"84798B",
        x"87788A", x"897889", x"8C7888", x"8F7888", x"927787", x"957786", x"977785", x"9A7685",
        x"9D7684", x"A07683", x"A37582", x"A57581", x"A87581", x"AB7480", x"AE747F", x"B1747E");
    -- memories
    type t_lram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lram0, lram1 : t_lram := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of lram0 : signal is true;
    attribute no_rw_check of lram1 : signal is true;
    signal s_lwe : std_logic := '0';
    signal s_lwa0, s_lwa1, s_lra : unsigned(7 downto 0) := (others => '0');
    signal s_lwd : std_logic_vector(15 downto 0) := (others => '0');
    signal s_lq0, s_lq1 : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ra : unsigned(12 downto 0) := (others => '0');
    signal s_rq, s_rqf : std_logic_vector(7 downto 0) := (others => '0');
    signal s_ta : unsigned(6 downto 0) := (others => '0');
    signal s_tq : std_logic_vector(7 downto 0) := (others => '0');
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal s_wq : std_logic_vector(7 downto 0) := (others => '0');
    signal s_xa : unsigned(6 downto 0) := (others => '0');
    signal s_xqa, s_xqb, s_xqc : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ara : unsigned(8 downto 0) := (others => '0');
    signal s_arq : std_logic_vector(7 downto 0) := (others => '0');
    signal s_xa2 : unsigned(6 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid : std_logic_vector(3 downto 0) := (others => '1');
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glided knobs (10.3), latched knobs, switches
    signal g1, g2, g3, g12 : unsigned(12 downto 0) := (others => '0');
    signal k1v, k2v, k3v, p12v : unsigned(9 downto 0) := (others => '0');
    signal s_k5, s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sp : signed(10 downto 0) := (others => '0');
    signal s_asp : unsigned(9 downto 0) := (others => '0');
    signal s_notear, s_nopipe, s_noflame, s_velvet, s_osc : std_logic := '0';
    signal s_warm : std_logic := '0';

    -- per-field constants
    signal s_incA : unsigned(8 downto 0) := to_unsigned(108, 9);
    signal s_incB : unsigned(8 downto 0) := to_unsigned(121, 9);
    signal s_incS : unsigned(12 downto 0) := to_unsigned(1820, 13);
    signal s_incAr, s_incBr : unsigned(9 downto 0) := (others => '0');
    signal s_incSr : unsigned(13 downto 0) := (others => '0');
    signal s_incOr : unsigned(7 downto 0) := (others => '0');
    signal s_pbpx : unsigned(10 downto 0) := to_unsigned(541, 11);
    signal s_th35 : unsigned(9 downto 0) := to_unsigned(190, 10);
    signal s_kp : unsigned(6 downto 0) := to_unsigned(51, 7);
    signal s_kf : unsigned(7 downto 0) := to_unsigned(69, 8);
    signal s_mg : unsigned(5 downto 0) := (others => '0');
    signal s_drA, s_drS, s_drO : unsigned(15 downto 0) := (others => '0');
    signal s_kA, s_kS, s_c15A, s_aB0 : unsigned(15 downto 0) := (others => '0');
    signal s_tacc : unsigned(12 downto 0) := (others => '0');
    signal s_tseed : unsigned(15 downto 0) := (others => '0');
    -- field sequencer (one serial multiplier, one serial divider)
    signal f_st : unsigned(4 downto 0) := (others => '0');
    signal f_ma : unsigned(23 downto 0) := (others => '0');
    signal f_mb : unsigned(11 downto 0) := (others => '0');
    signal f_acc : unsigned(23 downto 0) := (others => '0');
    signal f_mc : unsigned(3 downto 0) := (others => '0');
    signal f_dn : unsigned(20 downto 0) := (others => '0');
    signal f_dd : unsigned(11 downto 0) := (others => '0');
    signal f_rem : unsigned(12 downto 0) := (others => '0');
    signal f_dc : unsigned(4 downto 0) := (others => '0');
    signal s_linit : std_logic := '0';

    -- per-line engine (computes the NEXT line, kicked at the source avid rise)
    signal e_st : unsigned(5 downto 0) := (others => '0');
    signal e_ma : unsigned(23 downto 0) := (others => '0');
    signal e_mb : unsigned(13 downto 0) := (others => '0');
    signal e_acc : unsigned(23 downto 0) := (others => '0');
    signal e_mc : unsigned(3 downto 0) := (others => '0');
    signal e_vy8 : unsigned(19 downto 0) := (others => '0');
    signal e_vy : unsigned(10 downto 0) := (others => '0');
    signal e_up : std_logic := '1';
    signal e_yl : unsigned(10 downto 0) := (others => '0');
    signal e_j, e_j1 : unsigned(5 downto 0) := (others => '0');
    signal e_fy : unsigned(3 downto 0) := (others => '0');
    signal e_rw : unsigned(31 downto 0) := x"9E3779B9";
    signal e_seg : signed(13 downto 0) := (others => '0');
    signal e_smag : unsigned(11 downto 0) := (others => '0');
    signal e_sneg : std_logic := '0';
    signal e_psi : unsigned(15 downto 0) := (others => '0');
    signal e_tl : unsigned(1 downto 0) := (others => '0');
    signal e_T : signed(7 downto 0) := (others => '0');
    signal e_dx1 : unsigned(10 downto 0) := (others => '0');
    signal e_dxv : unsigned(9 downto 0) := (others => '0');
    signal e_dxn : unsigned(11 downto 0) := (others => '0');
    signal e_wv : unsigned(8 downto 0) := (others => '0');
    signal e_wa, e_wb, e_wc : unsigned(10 downto 0) := (others => '0');
    signal e_dd : signed(11 downto 0) := (others => '0');
    signal e_dxs : unsigned(10 downto 0) := (others => '0');
    signal e_aA0 : unsigned(15 downto 0) := (others => '0');
    signal e_tS : unsigned(15 downto 0) := (others => '0');
    signal e_bank : std_logic := '0';
    -- row loop
    signal r_run : std_logic := '0';
    signal r_c : unsigned(8 downto 0) := (others => '0');
    signal r_n : unsigned(8 downto 0) := (others => '0');
    signal r_v1, r_v2, r_v3 : std_logic := '0';
    signal r_c1, r_c2, r_c3 : unsigned(7 downto 0) := (others => '0');
    signal r_aK, r_aR, r_bK, r_bR : unsigned(7 downto 0) := (others => '0');
    signal r_pK, r_pR : signed(13 downto 0) := (others => '0');
    signal r_w : std_logic := '0';
    signal r_wi : unsigned(5 downto 0) := (others => '0');
    signal r_wt : std_logic := '0';
    -- next-line values
    signal n_up : std_logic := '1';
    signal n_aA0, n_aS0, n_aAr : unsigned(15 downto 0) := (others => '0');
    signal n_thrA : unsigned(10 downto 0) := (others => '0');
    signal n_wd : unsigned(6 downto 0) := (others => '0');
    signal n_shb : unsigned(4 downto 0) := (others => '0');
    signal n_yf : unsigned(7 downto 0) := (others => '0');
    signal n_ty : unsigned(7 downto 0) := (others => '0');
    -- committed line values
    signal l_up : std_logic := '1';
    signal l_bank : std_logic := '0';
    signal l_aA0, l_aS0 : unsigned(15 downto 0) := (others => '0');
    signal l_thrA : unsigned(10 downto 0) := (others => '0');
    signal l_wd : unsigned(6 downto 0) := (others => '0');
    signal l_shb : unsigned(4 downto 0) := (others => '0');
    signal l_yf : unsigned(7 downto 0) := (others => '0');
    signal l_ty : unsigned(7 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- beam (stage 0)
    signal b_aA, b_aB, b_aS, b_aO : unsigned(15 downto 0) := (others => '0');
    signal b_wr, b_first, b_st : std_logic := '0';
    signal b_vx2, b_xbc2 : unsigned(11 downto 0) := (others => '0');

    -- video luma
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal v_d2 : unsigned(9 downto 0) := (others => '0');
    signal v_l3 : unsigned(7 downto 0) := x"80";
    signal s_avg : unsigned(15 downto 0) := x"8000";
    signal s_acnt : unsigned(5 downto 0) := (others => '0');
    signal s_adf : signed(16 downto 0) := (others => '0');
    signal s_adv : std_logic := '0';

    -- pixel pipeline
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_u5 is array (natural range <>) of unsigned(4 downto 0);
    type t_s16 is array (natural range <>) of signed(15 downto 0);
    type t_u16 is array (natural range <>) of unsigned(15 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    type t_u2 is array (natural range <>) of unsigned(1 downto 0);
    signal p_first : t_sl(1 to 5) := (others => '0');
    signal p_st, p_wr : t_sl(1 to 1) := (others => '0');
    signal p_xbc : t_u11(1 to 3) := (others => (others => '0'));
    signal p_xbl : t_u11(2 to 2) := (others => (others => '0'));
    signal p_xl14, p_xl8, p_dm : t_sl(3 to 11) := (others => '0');
    signal p_td : t_u5(3 to 7) := (others => (others => '0'));
    signal p_red : t_sl(9 to 12) := (others => '0');
    signal p_vx : t_u11(1 to 2) := (others => (others => '0'));
    signal p_fr : t_u5(1 to 3) := (others => (others => '0'));
    signal p1_L8 : unsigned(7 downto 0) := x"80";
    signal p1_dL : signed(8 downto 0) := (others => '0');
    signal p1_aS, p1_aA : unsigned(15 downto 0) := (others => '0');
    signal p2_mv : signed(15 downto 0) := (others => '0');
    signal p2_aS : unsigned(15 downto 0) := (others => '0');
    signal p2_d : t_u16(0 to 2) := (others => (others => '0'));
    signal p2_lcur : unsigned(7 downto 0) := (others => '0');
    signal p3_v0, p3_v1 : std_logic_vector(15 downto 0) := (others => '0');
    signal p3_mvK : signed(8 downto 0) := (others => '0');
    signal p3_ad : t_u16(0 to 2) := (others => (others => '0'));
    signal p3_xla : std_logic_vector(15 downto 0) := (others => '0');
    type t_slv16 is array (natural range <>) of std_logic_vector(15 downto 0);
    signal p8_pool : std_logic_vector(15 downto 0) := (others => '0');
    type t_u7 is array (natural range <>) of unsigned(6 downto 0);
    signal p_xi : t_u7(1 to 5) := (others => (others => '0'));
    signal p3_neg : t_sl(0 to 2) := (others => '0');
    signal p3_hw : unsigned(10 downto 0) := (others => '0');
    signal p3_hl, p3_hr : unsigned(7 downto 0) := (others => '0');
    signal p_gate : t_sl(3 to 12) := (others => '0');
    signal p_tooth : t_sl(3 to 12) := (others => '0');
    signal p_orng : t_u5(3 to 12) := (others => (others => '0'));
    signal p3_rU : unsigned(4 downto 0) := (others => '0');
    signal p_rL : t_u8(3 to 4) := (others => (others => '0'));    -- signed, as bits
    signal p_dk : t_u5(3 to 9) := (others => (others => '0'));
    signal p_shx : t_u5(3 to 7) := (others => (others => '0'));
    signal p4_dK, p4_dR : signed(13 downto 0) := (others => '0');
    signal p4_K0, p4_R0 : unsigned(7 downto 0) := (others => '0');
    signal p4_mvK : signed(8 downto 0) := (others => '0');
    signal p_arc : t_u2(4 to 12) := (others => (others => '0'));  -- 0 none, 1 core, 2 left, 3 right
    signal p4_rUy : unsigned(12 downto 0) := (others => '0');
    signal p5_K, p5_R, p5_S : unsigned(7 downto 0) := (others => '0');
    signal p5_ramp : signed(7 downto 0) := (others => '0');
    signal p5_mvK : signed(8 downto 0) := (others => '0');
    signal p6_Kp : signed(10 downto 0) := (others => '0');
    signal p6_jS, p6_jP : signed(7 downto 0) := (others => '0');
    signal p6_sB, p6_sG : unsigned(7 downto 0) := (others => '0');
    signal p6_R : unsigned(7 downto 0) := (others => '0');
    signal p7_gr : unsigned(6 downto 0) := (others => '0');
    signal p7_o : unsigned(4 downto 0) := (others => '0');
    signal p6_d, p6_e : signed(9 downto 0) := (others => '0');
    signal p7_k, p7_kpl : signed(10 downto 0) := (others => '0');

    signal p7_R, p7_rB, p7_sB, p7_sG : unsigned(7 downto 0) := (others => '0');
    signal p7_l, p8_l : std_logic := '0';
    signal p8_cls : unsigned(1 downto 0) := (others => '0');       -- 0 pool, 1 fringe, 2 stripe, 3 red
    signal p8_R, p8_rG, p8_rB, p8_sB, p8_sG : unsigned(7 downto 0) := (others => '0');
    signal p8_bR, p8_bG, p8_bB : unsigned(7 downto 0) := (others => '0');
    signal p8_sh : unsigned(4 downto 0) := (others => '0');
    signal p8_dome : std_logic := '0';
    signal p9_R, p9_G, p9_B : unsigned(7 downto 0) := (others => '0');
    signal p9_bR, p9_bG, p9_bB : unsigned(7 downto 0) := (others => '0');
    signal p9_dome, p9_rim : std_logic := '0';
    signal p9_mR, p9_mG, p9_mB : unsigned(7 downto 0) := (others => '0');
    signal p10_R, p10_G, p10_B : unsigned(7 downto 0) := (others => '0');
    signal p10_mR, p10_mG, p10_mB : unsigned(7 downto 0) := (others => '0');
    signal p11_R, p11_G, p11_B : unsigned(7 downto 0) := (others => '0');
    signal p12_R, p12_G, p12_B : unsigned(7 downto 0) := (others => '0');
    signal p_rgb13, p_sr14, p_o14 : t_u8(0 to 2) := (others => (others => '0'));
    signal p_yr15, p_yg15, p_yb15 : unsigned(15 downto 0) := (others => '0');
    signal p_ur15, p_ug15, p_ub15, p_vr15, p_vg15, p_vb15 : signed(16 downto 0) := (others => '0');
    signal p_ys16 : unsigned(16 downto 0) := (others => '0');
    signal p_us16, p_vs16 : signed(16 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y18, s_y19, s_y20, s_y21, s_y22, s_y23x, s_yf23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_tab : process(clk)
    begin
        if rising_edge(clk) then
            s_rq <= C_TAB(to_integer(s_ra));
            s_rqf <= s_rq;
        end if;
    end process p_tab;

    p_lram0 : process(clk)
    begin
        if rising_edge(clk) then
            s_lq0 <= lram0(to_integer(s_lra));
            if s_lwe = '1' then lram0(to_integer(s_lwa0)) <= s_lwd; end if;
        end if;
    end process p_lram0;

    p_lram1 : process(clk)
    begin
        if rising_edge(clk) then
            s_lq1 <= lram1(to_integer(s_lra));
            if s_lwe = '1' then lram1(to_integer(s_lwa1)) <= s_lwd; end if;
        end if;
    end process p_lram1;

    p_teeth : process(clk)
    begin
        if rising_edge(clk) then
            s_tq <= C_TEETH(to_integer(s_ta));
        end if;
    end process p_teeth;

    p_wave : process(clk)
    begin
        if rising_edge(clk) then
            s_wq <= C_WAVE(to_integer(s_wa));
        end if;
    end process p_wave;

    p_xlut : process(clk)
    begin
        if rising_edge(clk) then
            s_xqa <= C_XLA(to_integer(s_xa2));
            s_xqb <= C_XLB(to_integer(s_xa));
            s_xqc <= C_XLC(to_integer(s_xa));
        end if;
    end process p_xlut;

    p_arcrom : process(clk)
    begin
        if rising_edge(clk) then
            s_arq <= C_ARC(to_integer(s_ara));
        end if;
    end process p_arcrom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_a : unsigned(12 downto 0);
        variable v_t : unsigned(13 downto 0);
        variable v_k : signed(10 downto 0);
        variable v_sp : signed(16 downto 0);
        procedure glide(signal g : inout unsigned(12 downto 0); k : std_logic_vector(9 downto 0); w : std_logic) is
            variable d : signed(14 downto 0);
            variable n : signed(14 downto 0);
        begin
            d := signed(resize(unsigned(k) & "000", 15)) - signed(resize(g, 15));
            if w = '0' then
                g <= unsigned(k) & "000";
            elsif d > 31 or d < -31 then
                n := signed(resize(g, 15)) + shift_right(d, 3);
                g <= unsigned(n(12 downto 0));
            end if;
        end procedure;
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xs <= f_vstep_x(s_tid);
            s_ys <= f_vstep_y(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            k1v <= g1(12 downto 3); k2v <= g2(12 downto 3); k3v <= g3(12 downto 3); p12v <= g12(12 downto 3);
            g_wm1 <= s_w - 1;
            s_linit <= '0';

            if s_zgo = '1' then
                s_warm <= '1';
                glide(g1, registers_in(0), s_warm);
                glide(g2, registers_in(1), s_warm);
                g3 <= unsigned(registers_in(2)) & "000";
                glide(g12, registers_in(7), s_warm);
                s_k5 <= unsigned(registers_in(4));
                s_k6 <= unsigned(registers_in(5));
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;          -- K4 Speed
                if v_s > -8 and v_s < 8 then v_s := (others => '0'); end if;
                s_sp <= v_s;
                if v_s < 0 then s_asp <= unsigned(resize(-v_s, 10)); else s_asp <= unsigned(v_s(9 downto 0)); end if;
                s_notear  <= registers_in(6)(0);
                s_nopipe  <= registers_in(6)(1);
                s_noflame <= registers_in(6)(2);
                s_velvet  <= registers_in(6)(3);
                s_osc     <= registers_in(6)(4);
                if s_wmax /= 0 then s_w <= s_wmax; end if;
                s_wmax <= (others => '0');
                -- motion: folds drift sp/65536 cycle per field, stripes 8x, the Osc 4x; tears re-seed
                v_sp := resize(s_sp, 17);
                s_drA <= unsigned(signed(s_drA) + resize(v_sp, 16));
                s_drS <= unsigned(signed(s_drS) + resize(shift_left(v_sp, 3), 16));
                s_drO <= unsigned(signed(s_drO) + resize(shift_left(v_sp, 2), 16));
                v_t := resize(s_tacc, 14) + resize(s_asp, 14);
                if v_t(13) = '1' then s_tseed <= s_tseed + 1; end if;
                s_tacc <= v_t(12 downto 0);
                f_st <= to_unsigned(1, 5);
            end if;

            -- per-field sequencer: one op per state; the serial units step while their
            -- counters are non-zero and the state machine waits.
            if f_mc /= 0 then
                if f_mb(0) = '1' then f_acc <= f_acc + f_ma; end if;
                f_ma <= f_ma(22 downto 0) & '0';
                f_mb <= '0' & f_mb(11 downto 1);
                f_mc <= f_mc - 1;
            elsif f_dc /= 0 then
                v_a := f_rem(11 downto 0) & f_dn(20);
                if v_a >= resize(f_dd, 13) then
                    f_rem <= v_a - resize(f_dd, 13);
                    f_dn <= f_dn(19 downto 0) & '1';
                else
                    f_rem <= v_a;
                    f_dn <= f_dn(19 downto 0) & '0';
                end if;
                f_dc <= f_dc - 1;
            else
                case to_integer(f_st) is
                    when 1 =>       -- k1^2
                        f_ma <= resize(k1v, 24); f_mb <= resize(k1v, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 2 =>       -- (k1^2 >> 8) * 54
                        f_ma <= resize(f_acc(19 downto 8), 24); f_mb <= to_unsigned(54, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(6, 4);
                        f_st <= f_st + 1;
                    when 3 =>
                        s_incA <= to_unsigned(54, 9) + resize(f_acc(19 downto 10), 9);
                        f_ma <= resize(k2v, 24); f_mb <= resize(k2v, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 4 =>       -- (k2^2 >> 8) * 61
                        f_ma <= resize(f_acc(19 downto 8), 24); f_mb <= to_unsigned(61, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(6, 4);
                        f_st <= f_st + 1;
                    when 5 =>
                        s_incB <= to_unsigned(60, 9) + resize(f_acc(19 downto 10), 9);
                        f_ma <= resize(k3v, 24); f_mb <= resize(k3v, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 6 =>       -- (k3^2 >> 8) * 910
                        f_ma <= resize(f_acc(19 downto 8), 24); f_mb <= to_unsigned(910, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 7 =>
                        s_incS <= to_unsigned(910, 13) + resize(f_acc(21 downto 10), 13);
                        -- PBpx = 65536 / incB
                        f_dn <= shift_left(to_unsigned(65536, 21), 4); f_dd <= resize(s_incB, 12);
                        f_rem <= (others => '0'); f_dc <= to_unsigned(17, 5);
                        f_st <= f_st + 1;
                    when 8 =>
                        s_pbpx <= f_dn(10 downto 0);
                        f_ma <= resize(f_dn(10 downto 0), 24); f_mb <= to_unsigned(90, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(7, 4);
                        f_st <= f_st + 1;
                    when 9 =>
                        s_th35 <= f_acc(17 downto 8);
                        -- per real pixel increments (x step xs/2 virtual px)
                        case to_integer(s_xs) is
                            when 4 =>
                                s_incAr <= s_incA & '0'; s_incBr <= s_incB & '0';
                                s_incSr <= s_incS & '0'; s_incOr <= to_unsigned(188, 8);
                            when 3 =>
                                s_incAr <= resize(s_incA, 10) + resize(s_incA(8 downto 1), 10);
                                s_incBr <= resize(s_incB, 10) + resize(s_incB(8 downto 1), 10);
                                s_incSr <= resize(s_incS, 14) + resize(s_incS(12 downto 1), 14);
                                s_incOr <= to_unsigned(141, 8);
                            when others =>
                                s_incAr <= resize(s_incA, 10); s_incBr <= resize(s_incB, 10);
                                s_incSr <= resize(s_incS, 14); s_incOr <= to_unsigned(94, 8);
                        end case;
                        -- pools: kp = 51 + (k5 - 512) / 8, clamped 0..127; kf = kp + 18
                        v_k := to_signed(51, 11) + shift_right(signed(resize(s_k5, 11)) - 512, 3);
                        if v_k < 0 then s_kp <= (others => '0');
                        elsif v_k > 127 then s_kp <= (others => '1');
                        else s_kp <= unsigned(v_k(6 downto 0)); end if;
                        s_mg <= s_k6(9 downto 4);
                        f_st <= f_st + 1;
                    when 10 =>
                        s_kf <= resize(s_kp, 8) + 18;
                        s_kA <= shift_left(resize(s_incA, 16), 11) - s_drA;
                        s_kS <= shift_left(resize(s_incS, 16), 6) - s_drS;
                        s_c15A <= shift_left(resize(s_incA, 16), 4) - resize(s_incA, 16);
                        s_aB0 <= shift_left(resize(s_incB, 16), 4) - resize(s_incB, 16);
                        s_linit <= '1';            -- start the line engine on line 0
                        f_st <= (others => '0');
                    when others =>
                        null;
                end case;
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
    -- Per-line engine: at the source avid rise it commits the values computed for this
    -- line and starts on the next one: the two key/shade table rows interpolated into the
    -- line RAM (row loop, 128 or 256 ROM reads) while one serial multiplier computes the
    -- line's oscillator starts, sweep, tear, stripe phase and ramps (~250 clocks).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_n : unsigned(19 downto 0);
        variable v_x : unsigned(31 downto 0);
        variable v_i : signed(12 downto 0);
        variable v_u : unsigned(15 downto 0);
        variable v_k : unsigned(7 downto 0);
        variable v_r : unsigned(7 downto 0);
        variable v_kv, v_rv : signed(9 downto 0);
        variable v_a12 : unsigned(11 downto 0);
        variable v_w : integer range 0 to 63;
        variable v_sx : integer range -128 to 127;
        variable v_d : signed(12 downto 0);
        variable v_seg : unsigned(5 downto 0);
        variable v_f4 : unsigned(3 downto 0);
        variable v_a, v_b : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' and s_avq = '0' then
                -- commit this line
                l_up <= n_up; l_bank <= e_bank;
                l_aA0 <= n_aA0; l_aS0 <= n_aS0;
                l_thrA <= n_thrA; l_wd <= n_wd; l_shb <= n_shb; l_yf <= n_yf; l_ty <= n_ty;
            end if;

            ---------------------------------------------------------------
            -- row loop: issue ROM addresses (row j, row j1 alternately), interpolate, write
            s_lwe <= '0';
            r_v1 <= '0';
            if r_run = '1' then
                if r_c(0) = '0' then v_a12 := e_j & r_c(6 downto 1); else v_a12 := e_j1 & r_c(6 downto 1); end if;
                if e_up = '1' then
                    s_ra <= '0' & v_a12;
                else
                    s_ra <= '1' & r_c(7) & v_a12(10 downto 0);
                end if;
                r_v1 <= '1'; r_c1 <= r_c(7 downto 0);
                if r_c = r_n then r_run <= '0'; end if;
                r_c <= r_c + 1;
            end if;
            r_v2 <= r_v1; r_c2 <= r_c1;
            r_v3 <= r_v2; r_c3 <= r_c2;
            r_w <= '0';
            if r_v3 = '1' then
                v_k := unsigned(s_rqf(7 downto 4)) & unsigned(s_rqf(7 downto 4));
                v_r := unsigned(s_rqf(3 downto 0)) & unsigned(s_rqf(3 downto 0));
                if r_c3(0) = '0' then
                    r_aK <= v_k; r_aR <= v_r;
                else
                    v_kv := signed(resize(v_k, 10)) - signed(resize(r_aK, 10));
                    v_rv := signed(resize(v_r, 10)) - signed(resize(r_aR, 10));
                    v_f4 := e_fy; v_f4(1 downto 0) := "00";
                    r_pK <= resize(v_kv * signed('0' & v_f4), 14);
                    r_pR <= resize(v_rv * signed('0' & v_f4), 14);
                    r_bK <= r_aK; r_bR <= r_aR;
                    r_w <= '1'; r_wi <= r_c3(6 downto 1); r_wt <= r_c3(7);
                end if;
            end if;
            if r_w = '1' then
                v_kv := signed(resize(r_bK, 10)) + resize(shift_right(r_pK, 4), 10);
                v_rv := signed(resize(r_bR, 10)) + resize(shift_right(r_pR, 4), 10);
                s_lwd <= std_logic_vector(v_kv(7 downto 0)) & std_logic_vector(v_rv(7 downto 0));
                s_lwa0 <= e_bank & r_wt & r_wi;
                s_lwa1 <= e_bank & r_wt & (r_wi - 1);
                s_lwe <= '1';
            end if;

            ---------------------------------------------------------------
            if s_linit = '1' then
                e_vy8 <= (others => '0');
                e_rw <= x"9E3779B9" xor (x"0000" & s_tseed); e_psi <= (others => '0'); e_seg <= (others => '0'); e_smag <= (others => '0');
                e_tl <= (others => '0'); e_T <= (others => '0');
                e_st <= to_unsigned(2, 6);
            elsif data_in.avid = '1' and s_avq = '0' then
                e_bank <= not e_bank;
                e_st <= to_unsigned(1, 6);
            elsif e_mc /= 0 then
                if e_mb(0) = '1' then e_acc <= e_acc + e_ma; end if;
                e_ma <= e_ma(22 downto 0) & '0';
                e_mb <= '0' & e_mb(13 downto 1);
                e_mc <= e_mc - 1;
            else
                case to_integer(e_st) is
                    when 1 =>       -- advance to the next line
                        e_vy8 <= e_vy8 + resize(s_ys, 20);
                        e_st <= e_st + 1;
                    when 2 =>
                        if e_vy8(19 downto 8) > 1079 then e_vy <= to_unsigned(1079, 11); else e_vy <= e_vy8(18 downto 8); end if;
                        e_st <= e_st + 1;
                    when 3 =>       -- band and row: rowq = yl * (7825 | 3913)
                        if e_vy < 536 then
                            e_up <= '1'; e_yl <= e_vy;
                            e_ma <= resize(e_vy, 24); e_mb <= to_unsigned(7825, 14);
                        else
                            e_up <= '0'; e_yl <= e_vy - 536;
                            e_ma <= resize(e_vy - 536, 24); e_mb <= to_unsigned(3913, 14);
                        end if;
                        e_acc <= (others => '0'); e_mc <= to_unsigned(13, 4);
                        e_st <= e_st + 1;
                    when 4 =>
                        e_fy <= e_acc(15 downto 12);
                        if e_up = '1' then
                            e_j <= e_acc(21 downto 16);
                            if e_acc(21 downto 16) = 63 then e_j1 <= e_acc(21 downto 16); else e_j1 <= e_acc(21 downto 16) + 1; end if;
                            r_n <= to_unsigned(127, 9);
                        else
                            if e_acc(21 downto 16) > 31 then e_j <= to_unsigned(31, 6); else e_j <= e_acc(21 downto 16); end if;
                            if e_acc(21 downto 16) >= 31 then e_j1 <= to_unsigned(31, 6); else e_j1 <= e_acc(21 downto 16) + 1; end if;
                            r_n <= to_unsigned(255, 9);
                        end if;
                        n_up <= e_up;
                        r_c <= (others => '0'); r_run <= '1';
                        -- the line's random word (xorshift32)
                        v_x := e_rw xor shift_left(e_rw, 13); v_x := v_x xor shift_right(v_x, 17); v_x := v_x xor shift_left(v_x, 5);
                        e_rw <= v_x;
                        e_st <= e_st + 1;
                    when 5 =>       -- stripe walk segment; tears
                        if e_seg <= 0 then
                            v_seg := to_unsigned(8, 6) + resize(e_rw(3 downto 0), 6) + resize(e_rw(5 downto 4), 6);
                            e_seg <= signed(shift_left(resize(v_seg, 14), 8));
                            if e_rw(7) = '0' then e_smag <= (others => '0');
                            else e_smag <= to_unsigned(1311, 12) + resize(e_rw(18 downto 8), 12); end if;
                            e_sneg <= e_rw(6);
                            if e_rw(21 downto 19) = "000" then
                                e_psi <= e_psi + (unsigned('0' & e_rw(30 downto 16)) - x"4000");
                            end if;
                        end if;
                        if e_tl /= 0 then
                            e_tl <= e_tl - 1;
                        else
                            e_T <= (others => '0');
                            if s_notear = '0' then
                                if e_vy >= 520 and e_vy < 596 then
                                    if e_rw(31 downto 24) < 31 then
                                        e_tl <= e_rw(11 downto 10);
                                        e_T <= signed(resize(e_rw(17 downto 12), 8)) + signed(resize(e_rw(27 downto 22), 8)) - 63;
                                    end if;
                                elsif e_rw(31 downto 22) < 12 then
                                    e_tl <= e_rw(11 downto 10);
                                    e_T <= signed(resize(e_rw(17 downto 12), 8)) - 32;
                                end if;
                            end if;
                        end if;
                        e_ma <= resize(e_smag, 24); e_mb <= resize(s_ys, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(12, 4);
                        e_st <= e_st + 1;
                    when 6 =>       -- psi += slope * ys / 256; dx: (vy * 1257 >> 10) * vy >> 10 + vy * 4252 >> 14 - 4
                        if e_sneg = '1' then e_psi <= e_psi - e_acc(23 downto 8); else e_psi <= e_psi + e_acc(23 downto 8); end if;
                        e_seg <= e_seg - signed(resize(s_ys, 14));
                        e_ma <= resize(e_vy, 24); e_mb <= to_unsigned(1257, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(11, 4);
                        e_st <= e_st + 1;
                    when 7 =>
                        e_ma <= resize(e_acc(20 downto 10), 24); e_mb <= resize(e_vy, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(11, 4);
                        e_st <= e_st + 1;
                    when 8 =>
                        e_dx1 <= e_acc(20 downto 10);
                        e_ma <= resize(e_vy, 24); e_mb <= to_unsigned(4252, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(13, 4);
                        e_st <= e_st + 1;
                    when 9 =>
                        e_dxn <= resize(e_dx1, 12) + resize(e_acc(23 downto 14), 12) - 4;
                        e_st <= e_st + 1;
                    when 10 =>      -- clamp; sweep dxs = dxv * sw >> 9 (upper half only)
                        if e_dxn(11) = '1' then v_n := (others => '0');
                        elsif e_dxn(10) = '1' then v_n := to_unsigned(1023, 20);
                        else v_n := resize(e_dxn(9 downto 0), 20); end if;
                        e_dxv <= v_n(9 downto 0);
                        e_ma <= resize(v_n(9 downto 0), 24); e_mb <= resize(p12v, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(10, 4);
                        e_st <= e_st + 1;
                    when 11 =>
                        if e_up = '1' then e_dxs <= e_acc(19 downto 9); else e_dxs <= (others => '0'); end if;
                        n_thrA <= resize(e_dxv, 11) + resize(s_th35, 11);
                        e_st <= e_st + 1;
                    when 12 =>      -- aA0 = (2048 + T - dxs - 260) incA - (2048 incA - driftA)
                        v_i := to_signed(1788, 13) + resize(e_T, 13) - signed(resize(e_dxs, 13));
                        e_ma <= resize(unsigned(v_i(11 downto 0)), 24); e_mb <= resize(s_incA, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(9, 4);
                        e_st <= e_st + 1;
                    when 13 =>
                        e_aA0 <= e_acc(15 downto 0) - s_kA;
                        e_ma <= resize(unsigned(resize(e_T, 8) + 64), 24); e_mb <= resize(s_incS, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(13, 4);
                        e_st <= e_st + 1;
                    when 14 =>      -- first-flame phase; stripe tear term; lower slant 2359 yl
                        n_aA0 <= e_aA0;
                        n_aAr <= e_aA0 - s_c15A;
                        e_tS <= e_acc(15 downto 0) - s_kS;
                        e_ma <= resize(e_yl, 24); e_mb <= to_unsigned(2359, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(12, 4);
                        e_st <= e_st + 1;
                    when 15 =>
                        if e_up = '1' then n_aS0 <= e_psi + e_tS; else n_aS0 <= e_tS - e_acc(15 downto 0); end if;
                        if e_vy < 200 then n_yf <= (others => '0');
                        elsif e_vy >= 455 then n_yf <= (others => '1');
                        else n_yf <= resize(e_vy - 200, 8); end if;
                        -- dome width: min(2.25 v, 25 + 1.125 v, 78 + v / 4, 118), v = vy - 355 (an ogive)
                        if e_vy >= 355 then e_wv <= resize(e_vy - 355, 9); else e_wv <= (others => '0'); end if;
                        e_dd <= signed(resize(e_vy, 12)) - 380;
                        -- Osc: the vertical triangle (period 520 lines)
                        e_ma <= resize(e_vy, 24); e_mb <= to_unsigned(126, 14);
                        e_acc <= (others => '0'); e_mc <= to_unsigned(7, 4);
                        e_st <= e_st + 1;
                    when 16 =>
                        v_u := e_acc(15 downto 0) + ('0' & s_drO(15 downto 1));
                        if v_u(15) = '0' then n_ty <= v_u(14 downto 7); else n_ty <= not v_u(14 downto 7); end if;
                        e_wa <= resize(shift_left(resize(e_wv, 11), 1) + shift_right(resize(e_wv, 11), 2), 11);
                        e_wb <= resize(25 + resize(e_wv, 11) + shift_right(resize(e_wv, 11), 3), 11);
                        e_wc <= resize(78 + shift_right(resize(e_wv, 11), 2), 11);
                        if e_dd < 0 then e_dd <= -e_dd; end if;
                        e_st <= e_st + 1;
                    when 17 =>
                        if e_wb < e_wa then e_wa <= e_wb; end if;
                        if e_dd > 247 then e_dd <= to_signed(255, 12); end if;
                        e_st <= e_st + 1;
                    when 18 =>
                        if e_wc < e_wa then e_wa <= e_wc; end if;
                        if e_up = '0' or e_dd > 247 then n_shb <= (others => '0');
                        else n_shb <= to_unsigned(31, 5) - unsigned(e_dd(7 downto 3)); end if;
                        e_st <= e_st + 1;
                    when 19 =>
                        if e_up = '0' or e_wv = 0 then n_wd <= (others => '0');
                        elsif e_wa > 118 then n_wd <= to_unsigned(118, 7);
                        else n_wd <= e_wa(6 downto 0); end if;
                        e_st <= (others => '0');
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Video luma (L8 = 0..255 over black .. white)
    ---------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_l : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_y1 <= unsigned(data_in.y);
            if v_y1 > 64 then v_d2 <= v_y1 - 64; else v_d2 <= (others => '0'); end if;
            v_l := resize(v_d2(9 downto 2), 11) + resize(v_d2(9 downto 5), 11) + resize(v_d2(9 downto 6), 11);
            if v_l > 255 then v_l3 <= (others => '1'); else v_l3 <= v_l(7 downto 0); end if;
        end if;
    end process p_vid;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): fold, pleat, stripe and Osc phases; pixels since the last cut
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_b : unsigned(16 downto 0);
        variable v_a : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            b_st <= '0';
            if g_start = '1' then
                b_aA <= l_aA0; b_aB <= s_aB0; b_aS <= l_aS0; b_aO <= s_drO;
                b_vx2 <= (others => '0'); b_xbc2 <= to_unsigned(30, 12);
                b_first <= '1'; b_wr <= '0'; b_st <= '1';
            elsif g_avid = '1' then
                v_a := b_aA + resize(s_incAr, 16);
                b_aA <= v_a;
                v_b := resize(b_aB, 17) + resize(s_incBr, 17);
                b_aB <= v_b(15 downto 0);
                b_wr <= v_b(16);
                if v_b(16) = '1' then
                    b_first <= '0'; b_xbc2 <= (others => '0');
                    s_ta <= v_a(15 downto 9);              -- flame length at this cut
                else
                    b_xbc2 <= b_xbc2 + resize(s_xs, 12);
                end if;
                b_aS <= b_aS + resize(s_incSr, 16);
                b_aO <= b_aO + resize(s_incOr, 16);
                b_vx2 <= b_vx2 + resize(s_xs, 12);
            else
                s_ta <= n_aAr(15 downto 9);                -- the next line's first flame
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_cell : unsigned(5 downto 0);
        variable v_tx : unsigned(7 downto 0);
        variable v_s16 : unsigned(15 downto 0);
        variable v_x : signed(12 downto 0);
        variable v_t : signed(15 downto 0);
        variable v_u : unsigned(15 downto 0);
        variable v_k0, v_k1, v_r0, v_r1 : unsigned(7 downto 0);
        variable v_d10 : signed(9 downto 0);
        variable v_j : signed(9 downto 0);
        variable v_c : integer range 0 to 7;
        variable v_hw : unsigned(10 downto 0);
        variable v_code : unsigned(1 downto 0);
        variable v_s11 : signed(10 downto 0);
        variable v_g : signed(10 downto 0);
        variable v_m : unsigned(12 downto 0);
        variable v_rr, v_gg, v_bv : unsigned(7 downto 0);
        variable v_o : t_u8(0 to 2);
        variable v_s9 : unsigned(8 downto 0);
        variable v_y : unsigned(16 downto 0);
        variable v_cc : signed(16 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_i : signed(13 downto 0);
        variable v_w : signed(19 downto 0);
        variable v_m16 : unsigned(15 downto 0);
        variable v_u11 : unsigned(10 downto 0);
        variable v_dome : std_logic_vector(23 downto 0);
    begin
        if rising_edge(clk) then
            -------------------------------------------------------------------
            -- 1: line RAM address (A phase above the seam, B phase below; first B cycle uses
            --    its own table), x-LUT address, source luma, the lower stripe slant product
            if l_up = '1' then v_cell := b_aA(15 downto 10); p_fr(1) <= '0' & b_aA(9 downto 6);
            else v_cell := b_aB(15 downto 10); p_fr(1) <= '0' & b_aB(9 downto 6); end if;
            s_lra <= l_bank & (b_first and not l_up) & v_cell;
            s_xa <= b_vx2(11 downto 5);
            s_ara <= b_aA(15 downto 7);
            p_xi(1) <= b_vx2(11 downto 5);
            if s_osc = '1' then
                if b_aO(15) = '0' then v_tx := b_aO(14 downto 7); else v_tx := not b_aO(14 downto 7); end if;
                v_s9 := resize(v_tx, 9) + resize(l_ty, 9);
                v_tx := v_s9(8 downto 1);
            else
                v_tx := v_l3;
            end if;
            p1_L8 <= v_tx;
            p1_dL <= signed(resize(v_tx, 9)) - signed(resize(s_avg(15 downto 8), 9));
            p1_aS <= b_aS; p1_aA <= b_aA;
            p_xbc(1) <= b_xbc2(11 downto 1);
            p_first(1) <= b_first; p_st(1) <= b_st; p_wr(1) <= b_wr;

            -- 2: melt product, stripe phase, arc offsets (Q12), pixels to the next cut, flame length
            p2_mv <= resize(p1_dL * signed('0' & s_mg), 16);
            -- running mean of the source (one sample in 64 active pixels): avg += (L << 8 - avg) / 1024
            s_adv <= '0';
            if s_gav_sr(0) = '1' then
                s_acnt <= s_acnt + 1;
                if s_acnt = 0 then s_adv <= '1'; end if;
            end if;
            s_adf <= signed(resize(p1_L8 & x"00", 17)) - signed(resize(s_avg, 17));
            if s_adv = '1' then
                s_avg <= unsigned(signed(s_avg) + resize(shift_right(s_adf, 10), 16));
            end if;
            p2_aS <= p1_aS;

            v_x := signed(resize(s_pbpx, 13)) - signed(resize(p_xbc(1), 13));
            if v_x(12) = '1' then p_xbl(2) <= (others => '0'); else p_xbl(2) <= unsigned(v_x(10 downto 0)); end if;
            if p_st(1) = '1' or p_wr(1) = '1' then p2_lcur <= unsigned(s_tq); end if;
            p_xbc(2) <= p_xbc(1);
            p_first(2) <= p_first(1); p_fr(2) <= p_fr(1);

            -- 3: wave address, line RAM and x LUTs to fabric, |arc offsets|, arc width, gate, flame test
            v_s16 := p2_aS + unsigned(shift_left(p2_mv, 4));
            s_wa <= v_s16(15 downto 8);
            p3_mvK <= resize(shift_right(p2_mv, 6), 9);
            p3_v0 <= s_lq0; p3_v1 <= s_lq1;
            p_orng(3) <= unsigned(s_xqb(15 downto 11));
            p3_rU <= unsigned(s_xqb(10 downto 6));
            p_shx(3) <= unsigned(s_xqb(5 downto 1));
            p_rL(3) <= unsigned(std_logic_vector(resize(signed(s_xqc(15 downto 9)), 8)));
            p_dk(3) <= unsigned(s_xqc(8 downto 4));
            if s_arq(7) = '1' then
                p3_neg(0) <= '1'; p3_ad(0) <= resize(to_unsigned(0, 8) - unsigned(s_arq), 16);
            else
                p3_neg(0) <= '0'; p3_ad(0) <= resize(unsigned(s_arq), 16);
            end if;
            if p_xbl(2) < 8 then v_c := 0;
            elsif p_xbl(2) >= 64 then v_c := 7;
            else v_c := to_integer(p_xbl(2) - 8) / 8; end if;
            case v_c is
                when 0 => p3_hw <= to_unsigned(27, 11); p3_hl <= to_unsigned(67, 8);  p3_hr <= to_unsigned(61, 8);
                when 1 => p3_hw <= to_unsigned(49, 11); p3_hl <= to_unsigned(89, 8);  p3_hr <= to_unsigned(83, 8);
                when 2 => p3_hw <= to_unsigned(60, 11); p3_hl <= to_unsigned(100, 8); p3_hr <= to_unsigned(94, 8);
                when 3 => p3_hw <= to_unsigned(71, 11); p3_hl <= to_unsigned(111, 8); p3_hr <= to_unsigned(105, 8);
                when 4 => p3_hw <= to_unsigned(82, 11); p3_hl <= to_unsigned(122, 8); p3_hr <= to_unsigned(116, 8);
                when others => p3_hw <= to_unsigned(87, 11); p3_hl <= to_unsigned(127, 8); p3_hr <= to_unsigned(121, 8);
            end case;
            if p_xbc(2) >= l_thrA and p_xbl(2) > 8 then p_gate(3) <= '1'; else p_gate(3) <= '0'; end if;
            if p_xbc(2) < resize(p2_lcur, 11) then p_tooth(3) <= '1'; else p_tooth(3) <= '0'; end if;
            if p_xbl(2) < 14 then p_xl14(3) <= '1'; else p_xl14(3) <= '0'; end if;
            if p_xbl(2) < 8 then p_xl8(3) <= '1'; else p_xl8(3) <= '0'; end if;
            v_s11 := signed(resize(l_wd, 11)) - signed(resize(p_xbl(2), 11));
            if v_s11 <= 0 then p_td(3) <= (others => '0');
            elsif v_s11 >= 124 then p_td(3) <= to_unsigned(31, 5);
            else p_td(3) <= unsigned(v_s11(6 downto 2)); end if;
            if l_up = '1' and l_wd /= 0 and v_s11 > 0 then p_dm(3) <= '1'; else p_dm(3) <= '0'; end if;
            p_first(3) <= p_first(2); p_fr(3) <= p_fr(2);

            -- 4: interpolation products, arc classes, upper ramp product
            v_k0 := unsigned(p3_v0(15 downto 8)); v_k1 := unsigned(p3_v1(15 downto 8));
            v_r0 := unsigned(p3_v0(7 downto 0)); v_r1 := unsigned(p3_v1(7 downto 0));
            v_d10 := signed(resize(v_k1, 10)) - signed(resize(v_k0, 10));
            p4_dK <= resize(v_d10 * signed(p_fr(3)), 14);
            v_d10 := signed(resize(v_r1, 10)) - signed(resize(v_r0, 10));
            p4_dR <= resize(v_d10 * signed(p_fr(3)), 14);
            p4_K0 <= v_k0; p4_R0 <= v_r0;
            p4_mvK <= p3_mvK;
            v_code := "00";
            v_hw := p3_hw;
            if p3_ad(0)(7 downto 0) < v_hw(7 downto 0) then v_code := "01";
            elsif p3_neg(0) = '1' and p3_ad(0)(7 downto 0) < p3_hl then v_code := "10";
            elsif p3_neg(0) = '0' and p3_ad(0)(7 downto 0) < p3_hr then v_code := "11";
            end if;
            p_arc(4) <= v_code;
            p4_rUy <= p3_rU * l_yf;
            p_first(4) <= p_first(3);

            -- 5: K and R (R also addresses the shade LUT), wave to fabric, ramp
            v_d10 := signed(resize(p4_R0, 10)) + resize(shift_right(p4_dR, 4), 10);
            p5_R <= unsigned(v_d10(7 downto 0));
            v_d10 := signed(resize(p4_K0, 10)) + resize(shift_right(p4_dK, 4), 10);
            p5_K <= unsigned(v_d10(7 downto 0));
            p5_S <= unsigned(s_wq);
            if l_up = '1' then
                p5_ramp <= signed(resize(p4_rUy(12 downto 7), 8));
            elsif p_first(4) = '1' then
                p5_ramp <= (others => '0');
            else
                p5_ramp <= signed(p_rL(4));
            end if;
            p5_mvK <= p4_mvK;
            p_first(5) <= p_first(4);

            -- 6: key sum, jitter terms, stripe colours, pool colour address
            s_xa2 <= p_xi(5);
            p6_Kp <= signed(resize(p5_K, 11)) + resize(p5_mvK, 11) + resize(p5_ramp, 11);
            v_j := signed(resize(p5_S, 10)) - 128;
            v_i := resize(shift_left(v_j, 2) + v_j, 14);                                            -- x40 / 256
            p6_jS <= resize(shift_right(v_i, 5), 8);
            if l_up = '1' then
                v_i := resize(shift_left(v_j, 1) + v_j, 14);                                        -- x48 / 256
                p6_jP <= resize(shift_right(v_i, 4), 8);
            else
                p6_jP <= resize(shift_right(v_i, 5), 8);
            end if;
            p6_sB <= 60 + resize(shift_right(p5_S, 1), 8) - resize(shift_right(p5_S, 5), 8);
            v_s9 := resize(p_orng(5), 9) + shift_left(resize(p_orng(5), 9), 1);
            p6_sG <= 4 + resize(v_s9(6 downto 3), 8);
            p6_R <= p5_R;
            p6_d <= signed(resize(p5_R, 10)) - 150;
            p6_e <= 200 - signed(resize(p5_R, 10));

            -- 7: keys, shade LUT to fabric, dome gradient
            p7_k <= p6_Kp + resize(p6_jS, 11);
            p7_kpl <= p6_Kp + resize(p6_jP, 11);
            -- red G factor clip(0.8125 (R - 150), 0, 80); red B 6 + clip(0.14 (200 - R), 0, 20);
            -- the lower half uses 3/4 of the orange ramp
            if p6_d <= 0 then v_u11 := (others => '0');
            else v_u11 := unsigned(resize(p6_d, 11)) - shift_right(unsigned(resize(p6_d, 11)), 2) + shift_right(unsigned(resize(p6_d, 11)), 4); end if;
            if v_u11 > 80 then p7_gr <= to_unsigned(80, 7); else p7_gr <= v_u11(6 downto 0); end if;
            if l_up = '1' then p7_o <= p_orng(6); else p7_o <= p_orng(6) - shift_right(p_orng(6), 2); end if;
            if p6_e <= 0 then v_u11 := (others => '0');
            else v_u11 := shift_right(unsigned(resize(p6_e, 11)), 3) + shift_right(unsigned(resize(p6_e, 11)), 6); end if;
            if v_u11 > 20 then p7_rB <= to_unsigned(26, 8); else p7_rB <= 6 + v_u11(7 downto 0); end if;
            p7_R <= p6_R; p7_sB <= p6_sB; p7_sG <= p6_sG;

            -- 8: classes, red G, dome base colour, shadow strength
            if p7_k >= 163 then p8_cls <= "11";
            elsif p7_kpl < signed(resize(s_kp, 11)) then p8_cls <= "00";
            elsif p7_kpl < signed(resize(s_kf, 11)) then p8_cls <= "01";
            else p8_cls <= "10"; end if;
            v_m := resize(p7_gr * p7_o, 13);
            p8_rG <= v_m(12 downto 5);
            v_dome := C_DOME(to_integer(p_td(7)));
            p8_bR <= unsigned(v_dome(23 downto 16)); p8_bG <= unsigned(v_dome(15 downto 8)); p8_bB <= unsigned(v_dome(7 downto 0));
            v_m := resize(p_shx(7) * l_shb, 13);
            p8_sh <= v_m(9 downto 5);
            p8_dome <= p_dm(7);
            p8_R <= p7_R; p8_rB <= p7_rB; p8_sB <= p7_sB; p8_sG <= p7_sG;
            p8_pool <= s_xqa;

            -- 9: colourizer, dome / shadow multipliers
            case to_integer(p8_cls) is
                when 3 => p9_R <= p8_R; p9_G <= p8_rG; p9_B <= p8_rB;
                when 2 => p9_R <= p8_R; p9_G <= p8_sG; p9_B <= p8_sB;
                when 1 => p9_R <= to_unsigned(70, 8); p9_G <= to_unsigned(25, 8); p9_B <= to_unsigned(175, 8);
                when others =>
                    p9_R <= resize(unsigned(p8_pool(15 downto 12)) & "00", 8);
                    p9_G <= unsigned(p8_pool(11 downto 6)) & "00";
                    p9_B <= unsigned(p8_pool(5 downto 0)) & "00";
            end case;
            if p8_cls = "11" then p_red(9) <= '1'; else p_red(9) <= '0'; end if;
            p9_bR <= p8_bR; p9_bG <= p8_bG; p9_bB <= p8_bB;
            if p8_dome = '1' and p8_cls = "00" then
                -- the dome darkens toward the right: subtract 2 dk from every channel
                p9_dome <= '1';
                p9_mR <= resize(p_dk(8) & '0', 8); p9_mG <= resize(p_dk(8) & '0', 8); p9_mB <= resize(p_dk(8) & '0', 8);
            else
                -- right-hand shadow: subtract 4 / 2 / 3 x strength
                p9_dome <= '0';
                p9_mR <= shift_left(resize(p8_sh, 8), 2);
                p9_mG <= shift_left(resize(p8_sh, 8), 1);
                p9_mB <= resize(shift_left(resize(p8_sh, 8), 1) + resize(p8_sh, 8), 8);
            end if;
            p9_rim <= p_xl14(8);

            -- 10: dome / rim colours
            if p9_dome = '1' then
                if p9_rim = '1' then
                    p10_R <= to_unsigned(235, 8); p10_G <= to_unsigned(80, 8); p10_B <= to_unsigned(20, 8);
                else
                    p10_R <= p9_bR; p10_G <= p9_bG; p10_B <= p9_bB;
                end if;
            else
                p10_R <= p9_R; p10_G <= p9_G; p10_B <= p9_B;
            end if;
            p10_mR <= p9_mR; p10_mG <= p9_mG; p10_mB <= p9_mB;

            -- 11: shadow / dome darkening (saturating subtract)
            if p10_R > p10_mR then p11_R <= p10_R - p10_mR; else p11_R <= (others => '0'); end if;
            if p10_G > p10_mG then p11_G <= p10_G - p10_mG; else p11_G <= (others => '0'); end if;
            if p10_B > p10_mB then p11_B <= p10_B - p10_mB; else p11_B <= (others => '0'); end if;

            -- 12: flames, the lower fringe, arcs
            v_rr := p11_R; v_gg := p11_G; v_bv := p11_B;
            if l_up = '1' then
                if s_noflame = '0' and p_tooth(11) = '1' then
                    v_rr := to_unsigned(250, 8);
                    v_gg := resize(shift_left(p_orng(11), 1), 8) + resize(shift_right(p_orng(11), 1), 8);
                    v_bv := to_unsigned(4, 8);
                end if;
                if s_nopipe = '0' and p_gate(11) = '1' then
                    case to_integer(p_arc(11)) is
                        when 1 => v_rr := to_unsigned(6, 8);  v_gg := to_unsigned(235, 8); v_bv := to_unsigned(100, 8);
                        when 2 => v_rr := to_unsigned(10, 8); v_gg := to_unsigned(190, 8); v_bv := to_unsigned(235, 8);
                        when 3 => v_rr := to_unsigned(30, 8); v_gg := to_unsigned(30, 8);  v_bv := to_unsigned(140, 8);
                        when others => null;
                    end case;
                end if;
            elsif p_xl8(11) = '1' and p_red(11) = '0' then
                v_rr := to_unsigned(60, 8); v_gg := to_unsigned(10, 8); v_bv := to_unsigned(215, 8);
            end if;
            p12_R <= v_rr; p12_G <= v_gg; p12_B <= v_bv;

            -- 13: Velvet (R <-> B)
            if s_velvet = '1' then
                p_rgb13(0) <= p12_B; p_rgb13(1) <= p12_G; p_rgb13(2) <= p12_R;
            else
                p_rgb13(0) <= p12_R; p_rgb13(1) <= p12_G; p_rgb13(2) <= p12_B;
            end if;

            -- 14: register
            p_o14 <= p_rgb13;

            -- 15..17: RGB -> 10-bit YUV (u = Cr, v = Cb), Q6, luma pre-boosted 16/13:
            -- Y = 64 + (81 R + 159 G + 31 B) / 64, Cr = 512 + (113 R - 94 G - 18 B) / 64,
            -- Cb = 512 + (-38 R - 75 G + 113 B) / 64
            v_rr := p_o14(0); v_gg := p_o14(1); v_bv := p_o14(2);
            p_yr15 <= shift_left(resize(v_rr, 16), 6) + shift_left(resize(v_rr, 16), 4) + resize(v_rr, 16);
            p_yg15 <= shift_left(resize(v_gg, 16), 7) + shift_left(resize(v_gg, 16), 5) - resize(v_gg, 16);
            p_yb15 <= shift_left(resize(v_bv, 16), 5) - resize(v_bv, 16);
            p_ur15 <= signed(shift_left(resize(v_rr, 17), 7) - shift_left(resize(v_rr, 17), 4) + resize(v_rr, 17));
            p_ug15 <= -signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 5) - shift_left(resize(v_gg, 17), 1));
            p_ub15 <= -signed(shift_left(resize(v_bv, 17), 4) + shift_left(resize(v_bv, 17), 1));
            p_vr15 <= -signed(shift_left(resize(v_rr, 17), 5) + shift_left(resize(v_rr, 17), 2) + shift_left(resize(v_rr, 17), 1));
            p_vg15 <= -signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 3) + shift_left(resize(v_gg, 17), 1) + resize(v_gg, 17));
            p_vb15 <= signed(shift_left(resize(v_bv, 17), 7) - shift_left(resize(v_bv, 17), 4) + resize(v_bv, 17));
            p_ys16 <= resize(p_yr15, 17) + resize(p_yg15, 17) + resize(p_yb15, 17);
            p_us16 <= p_ur15 + p_ug15 + p_ub15;
            p_vs16 <= p_vr15 + p_vg15 + p_vb15;
            v_y := shift_right(p_ys16, 6) + 64;
            if v_y > 940 then s_y17 <= to_unsigned(940, 10); else s_y17 <= v_y(9 downto 0); end if;
            v_cc := shift_right(p_us16, 6) + 512;
            s_u17 <= unsigned(v_cc(9 downto 0));
            v_cc := shift_right(p_vs16, 6) + 512;
            s_v17 <= unsigned(v_cc(9 downto 0));

            -- 18..23: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent
            -- 8-sums, luma [1 2 1]; centre at stage 22 -> 23)
            s_csu(0) <= s_u17; s_csv(0) <= s_v17;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u17 - s_csu(7);
            s_boxv <= s_boxv + s_v17 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v23 <= v_sc(13 downto 4);
            s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19; s_y21 <= s_y20;
            s_y22 <= s_y21; s_y23x <= s_y22;
            v_sc := resize(s_y21, 15) + shift_left(resize(s_y22, 15), 1) + resize(s_y23x, 15);
            s_yf23 <= v_sc(11 downto 2);
            -- 24: luma toward black (white 940 -> 777); blanking gate
            if s_yf23 > 64 then v_dl := s_yf23 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(22) = '1' then
                s_y24 <= s_yf23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;

            -- delays
            for i in 4 to 12 loop p_gate(i) <= p_gate(i - 1); p_tooth(i) <= p_tooth(i - 1); end loop;
            for i in 4 to 12 loop p_orng(i) <= p_orng(i - 1); end loop;
            for i in 5 to 12 loop p_arc(i) <= p_arc(i - 1); end loop;
            for i in 4 to 9 loop p_dk(i) <= p_dk(i - 1); end loop;
            for i in 2 to 5 loop p_xi(i) <= p_xi(i - 1); end loop;
            for i in 4 to 7 loop p_shx(i) <= p_shx(i - 1); end loop;
            for i in 4 to 11 loop p_xl14(i) <= p_xl14(i - 1); p_xl8(i) <= p_xl8(i - 1); p_dm(i) <= p_dm(i - 1); end loop;
            for i in 4 to 7 loop p_td(i) <= p_td(i - 1); end loop;
            for i in 10 to 12 loop p_red(i) <= p_red(i - 1); end loop;
            p_rL(4) <= p_rL(3);
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture curtain;
