-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Waterfall v0.5.0 -- Heirloom #45 (2026-10-27 rebuild), from WIN_20230610_00_51_13.
--
-- The photo: ONE line-locked chirped oscillator into a banded colourizer.
--   * The oscillator is reset at every line start.  Its first cycle runs slowly from the left
--     edge to "line 0" (x ~ 550): that long cycle is the whole left block area (olive / lavender
--     / pink at the top, flat cyan, green, the blue blob at the bottom left).  After line 0 the
--     frequency rises fast and saturates: lines 107 px apart at line 0, 36 px at the right edge,
--     31 lines across.  Two lines make a colour cycle (odd lines deep, even lines shallow).
--   * The phase psi(x, y) (line k at psi = k) is MEASURED from the photo's line positions and
--     stored as 47 row-knots x 31 knots (64 virtual px); the lines fan out to the right going
--     down and the left lines sweep hard left at the bottom: the waterfall.
--   * The colour is a function of (band, row, phase): four horizontal bands (A rainbow cycles
--     with hue arcs, B cyan, C lavender/blue, D orange/red) and a torn seam between C and D with
--     chevron stripes.  32 colour row-knots x 80 phase bins (the 2-line cycle, the first cycle,
--     the first stripe cycle), YUV565; per line the two knot rows are copied into a line-LUT
--     pair, per pixel the rows are interpolated (3-bit weight + ordered dither) and the phase
--     bin is chosen by ordered dither.
--   * The line cores are fixed-PIXEL features (dark navy cores, red G-low lines after the fall,
--     the B pulse after odd lines): a dip table by signed pixel distance to the nearest line.
--   * R carries a VCA that fades it toward the right (white-cyan -> cyan, lavender -> blue);
--     done in YUV on the red reconstructed from Y and Cr.
--
--   K1  Strings  line count (oscillator gain 0.5x .. 2x)
--   K2  Bands    vertical roll of the colour bands
--   K3  Fade     the red fade to the right (none .. 2x)
--   K4  Speed    bipolar (still in the centre): colour flows through the strings, the fall
--                sways, the seam chevrons roll
--   K5  Seam     height of the torn seam (0 = clean edge .. 2x)
--   K6  Melt     how far the camera (or Osc) bends the strings
--   S7  Beads    On / Off (the sawtooth beading of the line edges)
--   S8  Wind     Off / On (the bend reversed: the strings blow the other way)
--   S9  Dusk     Off / On (Cr <-> Cb: reds turn blue, cyans turn orange)
--   S10 Lines    Sharp / Soft (the line cores)
--   S11 Source   Video / Osc
--   P12 FALL     the bend: 0 = straight hanging strings, 50 % = the photo, 100 % = whipped
--
-- Virtual 1080-line coordinates (timing-ID row step, width-measured column step); generated
-- avid; output conditioning ([1 2 1] luma, [1 2 2 2 2 2 2 2 1]/16 chroma, luma 13/16); blanking
-- gate.  Latency 24 clocks in every mode.  Colours: u = Cr, v = Cb (hardware convention).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.waterfall_pkg.all;
use work.all;

architecture waterfall of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

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

    -- reciprocal mantissas round(2^19 / m), m = 16..31
    type t_rt is array (0 to 15) of unsigned(15 downto 0);
    constant C_RT : t_rt := (
        to_unsigned(32768, 16), to_unsigned(30840, 16), to_unsigned(29127, 16), to_unsigned(27594, 16),
        to_unsigned(26214, 16), to_unsigned(24966, 16), to_unsigned(23831, 16), to_unsigned(22795, 16),
        to_unsigned(21845, 16), to_unsigned(20972, 16), to_unsigned(20165, 16), to_unsigned(19418, 16),
        to_unsigned(18725, 16), to_unsigned(18079, 16), to_unsigned(17476, 16), to_unsigned(16913, 16));
    -- 4 x 4 ordered-dither thresholds (row & column)
    type t_b4 is array (0 to 15) of unsigned(3 downto 0);
    constant C_B4 : t_b4 := (x"0", x"8", x"2", x"A", x"C", x"4", x"E", x"6", x"3", x"B", x"1", x"9", x"F", x"7", x"D", x"5");
    type t_sin16 is array (0 to 15) of integer range -16 to 15;
    constant C_SIN16 : t_sin16 := (0, 5, 9, 12, 13, 12, 9, 5, 0, -5, -9, -12, -13, -12, -9, -5);

    ---------------------------------------------------------------------------
    -- memories
    type t_ram256 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ramE, ramO : t_ram256 := (others => (others => '0'));      -- psi line knots
    signal lutE, lutO : t_ram256 := (others => (others => '0'));      -- colour line LUT
    attribute no_rw_check : boolean;
    attribute no_rw_check of ramE : signal is true;
    attribute no_rw_check of ramO : signal is true;
    attribute no_rw_check of lutE : signal is true;
    attribute no_rw_check of lutO : signal is true;
    signal s_rwe, s_rwo : std_logic := '0';
    signal s_rwa : unsigned(7 downto 0) := (others => '0');
    signal s_rwd : std_logic_vector(15 downto 0) := (others => '0');
    signal s_rae, s_rao : unsigned(7 downto 0) := (others => '0');
    signal s_rqe, s_rqo : std_logic_vector(15 downto 0) := (others => '0');
    signal s_lwe, s_lwo : std_logic := '0';
    signal s_lwa : unsigned(7 downto 0) := (others => '0');
    signal s_lwd : std_logic_vector(15 downto 0) := (others => '0');
    signal s_lae, s_lao : unsigned(7 downto 0) := (others => '0');
    signal s_lqe, s_lqo : std_logic_vector(15 downto 0) := (others => '0');
    signal s_pa : unsigned(10 downto 0) := (others => '0');
    signal s_pq, s_pqf : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ca : unsigned(10 downto 0) := (others => '0');
    signal s_cb : unsigned(8 downto 0) := (others => '0');
    signal s_cqa, s_cqb, s_cqaf, s_cqbf : std_logic_vector(15 downto 0) := (others => '0');
    signal s_da : unsigned(8 downto 0) := (others => '0');
    signal s_dqrg, s_dqrgf : std_logic_vector(15 downto 0) := (others => '0');
    signal s_dqb, s_dqbf : std_logic_vector(7 downto 0) := (others => '0');
    signal s_rca : unsigned(7 downto 0) := (others => '0');
    signal s_rcq : std_logic_vector(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid : std_logic_vector(3 downto 0) := (others => '1');
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls
    signal g1, g12 : unsigned(12 downto 0) := (others => '0');
    signal s_warm : std_logic := '0';
    signal s_k1, s_k2, s_k3, s_k5, s_k6, s_p12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sp : signed(10 downto 0) := (others => '0');
    signal s_nobead, s_wind, s_dusk, s_soft, s_osc : std_logic := '0';
    -- per-field constants
    signal s_vstep : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal s_g8 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_f8 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_gfs : signed(11 downto 0) := to_signed(256, 12);
    signal s_cfa : signed(11 downto 0) := (others => '0');
    signal s_h36 : unsigned(6 downto 0) := to_unsigned(36, 7);
    signal s_inv : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_roll : unsigned(10 downto 0) := (others => '0');
    signal s_bjs : unsigned(15 downto 0) := to_unsigned(6242, 16);
    signal s_tph, s_tacc, s_cdr, s_bph : unsigned(15 downto 0) := (others => '0');
    signal s_tri : signed(7 downto 0) := (others => '0');
    signal s_drift : unsigned(7 downto 0) := (others => '0');
    signal s_troll : signed(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    -- field sequencer (one serial multiplier, one serial divider)
    signal f_st : unsigned(3 downto 0) := (others => '0');
    signal f_ma : unsigned(23 downto 0) := (others => '0');
    signal f_mb : unsigned(11 downto 0) := (others => '0');
    signal f_acc : unsigned(23 downto 0) := (others => '0');
    signal f_mc : unsigned(3 downto 0) := (others => '0');
    signal f_dn : unsigned(17 downto 0) := (others => '0');
    signal f_dd : unsigned(11 downto 0) := (others => '0');
    signal f_rem : unsigned(12 downto 0) := (others => '0');
    signal f_dc : unsigned(4 downto 0) := (others => '0');
    signal f_neg : std_logic := '0';
    signal s_linit : std_logic := '0';

    ---------------------------------------------------------------------------
    -- per-line engine (computes the NEXT line)
    signal e_st : unsigned(4 downto 0) := (others => '0');   -- 0..22
    signal e_vy8 : unsigned(19 downto 0) := (others => '0');
    signal e_vy : unsigned(10 downto 0) := (others => '0');
    signal e_bj : unsigned(15 downto 0) := (others => '0');
    signal e_pk : unsigned(5 downto 0) := (others => '0');
    signal e_pfy : unsigned(8 downto 0) := (others => '0');
    signal e_yr, e_yc : unsigned(10 downto 0) := (others => '0');
    signal e_d800 : signed(11 downto 0) := (others => '0');
    signal e_tear, e_stair : std_logic := '0';
    type t_b3 is array (0 to 2) of unsigned(2 downto 0);
    type t_k5 is array (0 to 2) of unsigned(4 downto 0);
    type t_u6a is array (0 to 2) of unsigned(5 downto 0);
    signal e_band : t_b3 := (others => (others => '0'));
    signal e_kk : t_k5 := (others => (others => '0'));
    signal e_fy : t_u6a := (others => (others => '0'));
    signal e_sl : unsigned(1 downto 0) := (others => '0');
    signal e_ki : unsigned(4 downto 0) := (others => '0');
    signal e_sk0 : signed(12 downto 0) := (others => '0');
    signal e_cthr : unsigned(4 downto 0) := (others => '0');
    signal e_ct1 : unsigned(17 downto 0) := (others => '0');
    signal e_hh : std_logic := '0';
    signal e_k0 : unsigned(4 downto 0) := (others => '0');
    signal e_t : signed(11 downto 0) := (others => '0');
    signal e_sh : std_logic_vector(1 downto 0) := (others => '0');
    signal e_nm : unsigned(3 downto 0) := (others => '0');
    signal e_xt : unsigned(10 downto 0) := (others => '0');
    signal e_shift : signed(10 downto 0) := (others => '0');
    signal e_cb, e_cc : signed(11 downto 0) := (others => '0');
    -- engine MAC: serial radix-4 Booth multiply-accumulate (load + 6 steps)
    signal m_a : signed(13 downto 0) := (others => '0');
    signal m_b : signed(10 downto 0) := (others => '0');
    signal m_go, m_clr : std_logic := '0';
    signal m_sh : signed(25 downto 0) := (others => '0');
    signal m_bb : std_logic_vector(12 downto 0) := (others => '0');
    signal m_cnt : unsigned(2 downto 0) := (others => '0');
    signal m_acc : signed(26 downto 0) := (others => '0');
    -- psi loop: 93 products (rows k, k+1, ref per knot); operand fetched while the previous runs
    signal q_run, q_more, q_wr : std_logic := '0';
    signal q_r, q_cr : unsigned(1 downto 0) := (others => '0');
    signal q_j, q_cj, q_wj : unsigned(4 downto 0) := (others => '0');
    signal q_w : unsigned(1 downto 0) := (others => '0');
    signal q_found, q_neg0, q_done : std_logic := '0';
    signal q_prev, q_rqa, q_rqb : signed(15 downto 0) := (others => '0');
    signal q_jr : unsigned(4 downto 0) := (others => '0');
    -- engine divider
    signal d_n : unsigned(25 downto 0) := (others => '0');
    signal d_d : unsigned(16 downto 0) := (others => '0');
    signal d_r : unsigned(17 downto 0) := (others => '0');
    signal d_c : unsigned(4 downto 0) := (others => '0');
    signal e_x0 : unsigned(10 downto 0) := (others => '0');
    signal e_cnt : signed(15 downto 0) := (others => '0');
    -- colour loop (copies rows k / k+1 of the 112 entries a line uses)
    signal r_run : std_logic := '0';
    signal r_sl : unsigned(1 downto 0) := (others => '0');
    signal r_e : unsigned(5 downto 0) := (others => '0');
    signal r_h : std_logic := '0';
    signal r_v1, r_v2, r_v3 : std_logic := '0';
    signal r_h1, r_h2, r_h3 : std_logic := '0';
    signal r_isb1, r_isb2, r_isb3 : std_logic := '0';
    signal r_la1, r_la2, r_la3 : unsigned(6 downto 0) := (others => '0');
    -- next-line values
    signal e_bank : std_logic := '0';
    signal n_x0 : unsigned(10 downto 0) := (others => '0');
    signal n_rst : unsigned(19 downto 0) := (others => '0');
    signal n_gst : unsigned(12 downto 0) := (others => '0');
    signal n_g0 : unsigned(21 downto 0) := (others => '0');
    signal n_cnt0 : unsigned(9 downto 0) := (others => '0');
    signal n_tear, n_stair : std_logic := '0';
    signal n_xt : unsigned(10 downto 0) := (others => '0');
    signal n_cthr : unsigned(4 downto 0) := (others => '0');
    signal n_s : unsigned(11 downto 0) := (others => '0');
    signal n_band : t_b3 := (others => (others => '0'));
    signal n_fy : t_u6a := (others => (others => '0'));
    signal n_jit : signed(4 downto 0) := (others => '0');
    signal n_oy : unsigned(7 downto 0) := (others => '0');
    -- committed line values
    signal l_bank : std_logic := '0';
    signal l_x0 : unsigned(10 downto 0) := (others => '0');
    signal l_rst : unsigned(19 downto 0) := (others => '0');
    signal l_gst : unsigned(12 downto 0) := (others => '0');
    signal l_g0 : unsigned(21 downto 0) := (others => '0');
    signal l_cnt0 : unsigned(9 downto 0) := (others => '0');
    signal l_tear, l_stair : std_logic := '0';
    signal l_xt : unsigned(10 downto 0) := (others => '0');
    signal l_cthr : unsigned(4 downto 0) := (others => '0');
    signal l_s : unsigned(11 downto 0) := (others => '0');
    signal l_band : t_b3 := (others => (others => '0'));
    signal l_fy : t_u6a := (others => (others => '0'));
    signal l_jit : signed(4 downto 0) := (others => '0');
    signal l_oy : unsigned(7 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_ssr : std_logic_vector(15 downto 0) := (others => '0');   -- g_start delayed (k+1)

    -- beams: stage 0 (x), stage 6 (x, first-cycle DDA, seam chevron counter), stage 13 (x)
    signal b_x16, b6_x16 : signed(16 downto 0) := (others => '0');
    signal b6_a1 : unsigned(24 downto 0) := (others => '0');
    signal b6_cnt : unsigned(9 downto 0) := (others => '0');
    signal b16_g : signed(22 downto 0) := (others => '0');
    signal b_ln, b9_pc : unsigned(1 downto 0) := (others => '0');

    -- video luma
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal v_d2 : unsigned(9 downto 0) := (others => '0');
    signal v_l3 : unsigned(7 downto 0) := x"80";

    -- pixel pipeline
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    type t_s13 is array (natural range <>) of signed(12 downto 0);
    signal p_fr : t_u10(1 to 4) := (others => (others => '0'));
    signal p_jo : t_sl(1 to 3) := (others => '0');
    signal p_mv : t_s13(4 to 6) := (others => (others => '0'));
    signal p1_L8 : unsigned(7 downto 0) := x"80";
    signal p2_dL : signed(8 downto 0) := (others => '0');
    signal p3_mv : signed(15 downto 0) := (others => '0');
    signal p3_ve, p3_vo : std_logic_vector(15 downto 0) := (others => '0');
    signal p4_v0 : signed(16 downto 0) := (others => '0');
    signal p4_dl : signed(10 downto 0) := (others => '0');
    signal p5_v0 : signed(16 downto 0) := (others => '0');
    signal p5_dl : signed(10 downto 0) := (others => '0');
    signal p5_pr : signed(21 downto 0) := (others => '0');
    signal p6_ps : signed(16 downto 0) := (others => '0');
    signal p6_dl : signed(10 downto 0) := (others => '0');
    signal p7_ps : signed(16 downto 0) := (others => '0');
    signal p7_u1, p7_cu : unsigned(9 downto 0) := (others => '0');
    signal p7_chv, p7_late : std_logic := '0';
    signal p7_x, p8_x : unsigned(10 downto 0) := (others => '0');
    signal p8_bin, p8_nxt : unsigned(6 downto 0) := (others => '0');
    signal p8_f5 : unsigned(4 downto 0) := (others => '0');
    signal p8_slot : unsigned(1 downto 0) := (others => '0');
    signal p8_F : std_logic := '0';
    signal p8_frs : signed(7 downto 0) := (others => '0');
    signal p8_par : std_logic := '0';
    signal p9_w, p10_w, p11_w, p12_w : unsigned(3 downto 0) := (others => '0');
    signal p9_rec : unsigned(7 downto 0) := (others => '0');
    signal p9_slot, p10_slot, p11_slot : unsigned(1 downto 0) := (others => '0');
    signal p9_F, p9_par, p10_F, p10_par, p11_F, p11_par : std_logic := '0';
    signal p9_frs : signed(7 downto 0) := (others => '0');
    signal p9_d0, p10_d0, p11_d : signed(12 downto 0) := (others => '0');
    signal p10_dp : signed(16 downto 0) := (others => '0');
    signal p11_qe, p11_qo : std_logic_vector(15 downto 0) := (others => '0');
    type t_u10a3 is array (0 to 2) of unsigned(9 downto 0);
    type t_s7a3 is array (0 to 2) of signed(6 downto 0);
    type t_s11a3 is array (0 to 2) of signed(10 downto 0);
    type t_s12a3 is array (0 to 2) of signed(11 downto 0);
    signal p12_a8, p13_q8, p14_c, p15_c : t_u10a3 := (others => (others => '0'));
    signal p12_d : t_s7a3 := (others => (others => '0'));
    signal p12_ry : unsigned(7 downto 0) := (others => '0');
    signal p12_ru : signed(9 downto 0) := (others => '0');
    signal p13_r8 : unsigned(7 downto 0) := (others => '0');
    signal p13_ng : unsigned(4 downto 0) := (others => '0');
    signal p14_rm : unsigned(12 downto 0) := (others => '0');
    signal p15_fy, p15_fu, p15_fv : unsigned(9 downto 0) := (others => '0');
    signal p15_dp : t_s11a3 := (others => (others => '0'));
    signal p16_s : t_s12a3 := (others => (others => '0'));
    signal p_dok : t_sl(12 to 14) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y22, s_y23, s_y24, s_y25, s_y26, s_y27x, s_yf27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

    function f_exp5(v : std_logic_vector(4 downto 0)) return unsigned is
    begin
        return unsigned(v) & unsigned(v(4 downto 2));
    end function;
    function f_exp6(v : std_logic_vector(5 downto 0)) return unsigned is
    begin
        return unsigned(v) & unsigned(v(5 downto 4));
    end function;
    function f_mirror(x : signed(16 downto 0); m : std_logic) return unsigned is
        variable v : signed(16 downto 0);
    begin
        v := x;
        if m = '1' then v := to_signed(30704, 17) - v; end if;
        if v < 0 then return to_unsigned(0, 15);
        elsif v > 30719 then return to_unsigned(30719, 15);
        else return unsigned(v(14 downto 0)); end if;
    end function;

begin
    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_psirom : process(clk)
    begin
        if rising_edge(clk) then
            s_pq <= C_PSI(to_integer(s_pa));
            s_pqf <= s_pq;
        end if;
    end process p_psirom;

    p_colrom : process(clk)
    begin
        if rising_edge(clk) then
            s_cqa <= C_COLA(to_integer(s_ca));
            s_cqb <= C_COLB(to_integer(s_cb));
            s_cqaf <= s_cqa; s_cqbf <= s_cqb;
        end if;
    end process p_colrom;

    p_diprom : process(clk)
    begin
        if rising_edge(clk) then
            s_dqrg <= C_DIPRG(to_integer(s_da));
            s_dqb <= C_DIPB(to_integer(s_da));
            s_dqrgf <= s_dqrg; s_dqbf <= s_dqb;
        end if;
    end process p_diprom;

    p_recrom : process(clk)
    begin
        if rising_edge(clk) then
            s_rcq <= C_REC(to_integer(s_rca));
        end if;
    end process p_recrom;

    p_rame : process(clk)
    begin
        if rising_edge(clk) then
            s_rqe <= ramE(to_integer(s_rae));
            if s_rwe = '1' then ramE(to_integer(s_rwa)) <= s_rwd; end if;
        end if;
    end process p_rame;
    p_ramo : process(clk)
    begin
        if rising_edge(clk) then
            s_rqo <= ramO(to_integer(s_rao));
            if s_rwo = '1' then ramO(to_integer(s_rwa)) <= s_rwd; end if;
        end if;
    end process p_ramo;
    p_lute : process(clk)
    begin
        if rising_edge(clk) then
            s_lqe <= lutE(to_integer(s_lae));
            if s_lwe = '1' then lutE(to_integer(s_lwa)) <= s_lwd; end if;
        end if;
    end process p_lute;
    p_luto : process(clk)
    begin
        if rising_edge(clk) then
            s_lqo <= lutO(to_integer(s_lao));
            if s_lwo = '1' then lutO(to_integer(s_lwa)) <= s_lwd; end if;
        end if;
    end process p_luto;

    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_a : unsigned(12 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_m : unsigned(11 downto 0);
        variable v_ab : unsigned(9 downto 0);
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

            g_wm1 <= s_w - 1;
            s_linit <= '0';

            if s_zgo = '1' then
                s_warm <= '1';
                glide(g1, registers_in(0), s_warm);
                glide(g12, registers_in(7), s_warm);
                s_k2 <= unsigned(registers_in(1));
                s_k3 <= unsigned(registers_in(2));
                s_k5 <= unsigned(registers_in(4));
                s_k6 <= unsigned(registers_in(5));
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;          -- K4 Speed
                if v_s > -8 and v_s < 8 then v_s := (others => '0'); end if;
                s_sp <= v_s;
                s_nobead <= registers_in(6)(0);
                s_wind   <= registers_in(6)(1);
                s_dusk   <= registers_in(6)(2);
                s_soft   <= registers_in(6)(3);
                s_osc    <= registers_in(6)(4);
                if s_wmax /= 0 then s_w <= s_wmax; end if;
                s_wmax <= (others => '0');
                -- motion: sway phase, seam roll, colour drift, bead phase, Osc time
                if v_s < 0 then v_ab := unsigned(resize(-v_s, 10)); else v_ab := unsigned(v_s(9 downto 0)); end if;
                s_tph <= s_tph + shift_left(resize(v_ab, 16), 2);
                v_t := s_tacc + unsigned(resize(s_sp, 16));
                s_tacc <= v_t;
                s_troll <= signed(v_t(15 downto 6));
                s_cdr <= s_cdr + unsigned(resize(s_sp, 16));
                s_bph <= s_bph + 765;
                s_ot <= s_ot + 1;
                f_st <= to_unsigned(1, 4);
            end if;
            s_k1 <= g1(12 downto 3); s_p12 <= g12(12 downto 3);
            s_drift <= s_cdr(15 downto 8);
            if s_tph(15) = '0' then s_tri <= signed(s_tph(14 downto 7) xor x"80");      -- u - 128
            else s_tri <= signed((not s_tph(14 downto 7)) xor x"80"); end if;

            -- per-field sequencer: one op per state; the serial units step while their
            -- counters are non-zero and the state machine waits.
            if f_mc /= 0 then
                if f_mb(0) = '1' then f_acc <= f_acc + f_ma; end if;
                f_ma <= f_ma(22 downto 0) & '0';
                f_mb <= '0' & f_mb(11 downto 1);
                f_mc <= f_mc - 1;
            elsif f_dc /= 0 then
                v_a := f_rem(11 downto 0) & f_dn(17);
                if v_a >= resize(f_dd, 13) then
                    f_rem <= v_a - resize(f_dd, 13);
                    f_dn <= f_dn(16 downto 0) & '1';
                else
                    f_rem <= v_a;
                    f_dn <= f_dn(16 downto 0) & '0';
                end if;
                f_dc <= f_dc - 1;
            else
                case to_integer(f_st) is
                    when 1 =>       -- vstep = 30720 / W (Q4 virtual px per pixel)
                        f_dn <= to_unsigned(30720, 18); f_dd <= s_w; f_rem <= (others => '0'); f_dc <= to_unsigned(18, 5);
                        f_st <= f_st + 1;
                    when 2 =>
                        if f_dn > 63 then s_vstep <= to_unsigned(63, 6); else s_vstep <= f_dn(5 downto 0); end if;
                        -- k1^2
                        f_ma <= resize(s_k1, 24); f_mb <= resize(s_k1, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 3 =>       -- g8 = 128 + k1/8 + k1^2/4096
                        s_g8 <= to_unsigned(128, 10) + resize(s_k1(9 downto 3), 10) + resize(f_acc(19 downto 12), 10);
                        -- sway: (p12/2) * |tri|
                        f_ma <= resize(s_p12(9 downto 1), 24);
                        if s_tri < 0 then f_mb <= resize(unsigned(-resize(s_tri, 9)), 12); f_neg <= '1';
                        else f_mb <= resize(unsigned(s_tri), 12); f_neg <= '0'; end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(8, 4);
                        f_st <= f_st + 1;
                    when 4 =>       -- f8 = p12/2 +- (p12/2 |tri| 30) >> 15
                        v_m := resize(shift_right(shift_left(f_acc(17 downto 0), 5) - shift_left(f_acc(17 downto 0), 1), 15), 12);
                        if f_neg = '1' then s_f8 <= resize(s_p12(9 downto 1), 10) - resize(v_m, 10);
                        else s_f8 <= resize(s_p12(9 downto 1), 10) + resize(v_m, 10); end if;
                        -- seam: h36 = k5 * 9 / 128; inv = 131071 / max(k5, 64)
                        s_h36 <= resize(shift_right(shift_left(resize(s_k5, 14), 3) + resize(s_k5, 14), 7), 7);
                        f_dn <= to_unsigned(131071, 18);
                        if s_k5 < 64 then f_dd <= to_unsigned(64, 12); else f_dd <= resize(s_k5, 12); end if;
                        f_rem <= (others => '0'); f_dc <= to_unsigned(18, 5);
                        f_st <= f_st + 1;
                    when 5 =>
                        s_inv <= f_dn(10 downto 0);
                        if s_wind = '1' then f_neg <= '1'; else f_neg <= '0'; end if;
                        -- band roll = k2 * 135 / 128
                        s_roll <= resize(shift_right(shift_left(resize(s_k2, 18), 7) + shift_left(resize(s_k2, 18), 2)
                                  + shift_left(resize(s_k2, 18), 1) + resize(s_k2, 18), 7), 11);
                        -- gf = g8 f8 / 256
                        f_ma <= resize(s_g8, 24); f_mb <= resize(s_f8, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 6 =>       -- ca = g8 (256 -+ f8) / 256 (signed); gf negated for Wind
                        if f_neg = '1' then s_gfs <= -signed(resize(f_acc(17 downto 8), 12));
                        else s_gfs <= signed(resize(f_acc(17 downto 8), 12)); end if;
                        f_ma <= resize(s_g8, 24);
                        if s_wind = '1' then f_mb <= resize(256 + s_f8, 12); f_neg <= '0';
                        elsif s_f8 > 256 then f_mb <= resize(s_f8 - 256, 12); f_neg <= '1';
                        else f_mb <= resize(256 - s_f8, 12); f_neg <= '0'; end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 7 =>       -- bead phase step per line = 6242 ys / 256
                        if f_acc(18) = '1' then v_m := to_unsigned(1023, 12); else v_m := resize(f_acc(17 downto 8), 12); end if;
                        if f_neg = '1' then s_cfa <= -signed(v_m);
                        else s_cfa <= signed(v_m); end if;
                        f_ma <= to_unsigned(6242, 24); f_mb <= s_ys; f_acc <= (others => '0'); f_mc <= to_unsigned(12, 4);
                        f_st <= f_st + 1;
                    when 8 =>
                        s_bjs <= f_acc(23 downto 8);
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
    -- line and starts on the next one.
    --   setup (one multiplier op per state): psi row-knot, colour row (band roll, seam),
    --     three colour slots (band, knot, row fraction), R fade slope, staircase, seam row;
    --   then in parallel: the psi loop (3 row-knots -> lerp, fall, gain -> psi line RAM, the
    --     root x0) and the colour loop (3 slots x 80 bins, row lerp -> line LUT);
    --   after the psi loop: x0, the first-cycle rate 2^24 / x0 and its per-pixel step, the
    --     seam chevron counter start.
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Per-line engine: at the source avid rise it commits the values computed for this
    -- line and starts on the next one.
    --   setup (states 1..15, one MAC op each): psi row-knot and fraction, colour row (band roll,
    --     seam), three colour slots (band, uniform knot, 6-bit fraction), R fade slope, the psi
    --     row coefficients cb = gf (1 - fy), cc = gf fy (ca = g (1 - f) per field);
    --   then in parallel: the psi loop (93 ROM reads; per knot Q3 = (cb Qk + cc Qk1 + ca Qref) / 256
    --     on the MAC -> psi line RAM; the root) and the colour loop (3 slots x 80 bins, row lerp
    --     -> line LUT);
    --   then the divider: x0, the first-cycle DDA step vstep 2^20 / x0 and (Mirror) its start,
    --     and the seam chevron counter start.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : signed(11 downto 0);
        variable v_y : unsigned(10 downto 0);
        variable v_u : unsigned(17 downto 0);
        variable v_t : signed(11 downto 0);
        variable v_k0 : integer range 0 to 31;
        variable v_n : integer range 0 to 15;
        variable v_y0 : integer range 0 to 1023;
        variable v_sh : integer range 5 to 7;
        variable v_i : unsigned(5 downto 0);
        variable v_f : unsigned(5 downto 0);
        variable v_q : signed(29 downto 0);
        variable v_a : unsigned(17 downto 0);
        variable v_kk : unsigned(4 downto 0);
        variable v_ca : std_logic_vector(15 downto 0);
        variable v_dr, v_dg, v_db : signed(8 downto 0);
        variable v_r, v_g, v_bb : unsigned(7 downto 0);
        variable v_c : signed(15 downto 0);
        variable v_s : signed(13 downto 0);
        variable v_tw : std_logic_vector(7 downto 0);
    begin
        if rising_edge(clk) then
            s_rwe <= '0'; s_rwo <= '0';
            s_lwe <= '0'; s_lwo <= '0';
            -- serial MAC
            m_go <= '0';
            if m_go = '1' then
                m_sh <= resize(m_a, 26);
                m_bb <= std_logic_vector(resize(m_b, 12)) & '0';
                m_cnt <= to_unsigned(6, 3);
                if m_clr = '1' then m_acc <= (others => '0'); end if;
            elsif m_cnt /= 0 then
                case m_bb(2 downto 0) is
                    when "001" | "010" => m_acc <= m_acc + resize(m_sh, 27);
                    when "011"         => m_acc <= m_acc + resize(shift_left(m_sh, 1), 27);
                    when "100"         => m_acc <= m_acc - resize(shift_left(m_sh, 1), 27);
                    when "101" | "110" => m_acc <= m_acc - resize(m_sh, 27);
                    when others        => null;
                end case;
                m_sh <= shift_left(m_sh, 2);
                m_bb <= m_bb(12) & m_bb(12) & m_bb(12 downto 2);
                m_cnt <= m_cnt - 1;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                -- commit this line
                l_bank <= e_bank;
                l_x0 <= n_x0; l_rst <= n_rst; l_gst <= n_gst; l_g0 <= n_g0; l_cnt0 <= n_cnt0;
                l_tear <= n_tear; l_stair <= n_stair; l_xt <= n_xt; l_cthr <= n_cthr; l_s <= n_s;
                l_band <= n_band; l_fy <= n_fy; l_jit <= n_jit; l_oy <= n_oy;
            end if;

            ---------------------------------------------------------------
            -- psi loop on the serial MAC: per knot j, cb Pk + cc Pk1 + ca Pr accumulate; each
            -- operand is fetched (row k, k + 1 or the reference row 7) while the previous runs
            q_wr <= '0';
            if q_run = '1' then
                if q_w /= 0 then q_w <= q_w - 1; end if;
                if q_w = 0 and m_cnt = 4 and q_more = '1' then
                    case to_integer(q_r) is
                        when 0 => s_pa <= e_pk & q_j;
                        when 1 => s_pa <= (e_pk + 1) & q_j;
                        when others => s_pa <= to_unsigned(7, 6) & q_j;
                    end case;
                end if;
                if q_more = '1' and (q_w = 1 or (q_w = 0 and m_cnt = 1)) then
                    m_a <= resize(signed(s_pqf), 14);
                    case to_integer(q_r) is
                        when 0 => m_b <= resize(e_cb, 11);
                        when 1 => m_b <= resize(e_cc, 11);
                        when others => m_b <= resize(s_cfa, 11);
                    end case;
                    m_go <= '1';
                    if q_r = 0 then m_clr <= '1'; else m_clr <= '0'; end if;
                    q_cr <= q_r; q_cj <= q_j;
                    if q_r = 2 then
                        q_r <= (others => '0');
                        if q_j = 30 then q_more <= '0'; else q_j <= q_j + 1; end if;
                    else
                        q_r <= q_r + 1;
                    end if;
                end if;
                if q_w = 0 and m_cnt = 1 and q_cr = 2 then q_wr <= '1'; q_wj <= q_cj; end if;
            end if;
            if q_wr = '1' then
                v_q := resize(shift_right(m_acc, 8), 30);
                if v_q > 32767 then v_c := to_signed(32767, 16);
                elsif v_q < -32768 then v_c := to_signed(-32768, 16);
                else v_c := v_q(15 downto 0); end if;
                s_rwd <= std_logic_vector(v_c);
                s_rwa <= "000" & e_bank & q_wj(4 downto 1);
                if q_wj(0) = '0' then s_rwe <= '1'; else s_rwo <= '1'; end if;
                if q_wj = 0 then
                    if v_c < 0 then q_neg0 <= '1'; else q_neg0 <= '0'; end if;
                elsif q_found = '0' and v_c >= 0 then
                    q_found <= '1'; q_jr <= q_wj; q_rqa <= q_prev; q_rqb <= v_c;
                end if;
                q_prev <= v_c;
                if q_wj = 30 then q_done <= '1'; q_run <= '0'; end if;
            end if;

            ---------------------------------------------------------------
            -- colour loop: copy rows k / k+1 of the entries this line uses into the line-LUT
            -- pair (lutE = row k, lutO = row k + 1, one address).  Slot 0: bins 0..31 and the
            -- first stripe cycle (64..79) at 0..47; slot 1: the first cycle (32..63) at 48..79;
            -- slot 2: bins 0..31 (staircase rows) or 32..63 (late seam) at 80..111.
            r_v1 <= '0';
            if r_run = '1' then
                v_kk := e_kk(to_integer(r_sl));
                if r_h = '1' then v_kk := v_kk + 1; end if;
                case to_integer(r_sl) is
                    when 0 =>
                        s_ca <= v_kk & r_e;
                        s_cb <= v_kk & r_e(3 downto 0);
                        r_isb1 <= r_e(5); r_la1 <= resize(r_e, 7);
                    when 1 =>
                        s_ca <= v_kk & '1' & r_e(4 downto 0);
                        r_isb1 <= '0'; r_la1 <= resize(r_e, 7) + 48;
                    when others =>
                        s_ca <= v_kk & (not e_stair) & r_e(4 downto 0);
                        r_isb1 <= '0'; r_la1 <= resize(r_e, 7) + 80;
                end case;
                r_v1 <= '1'; r_h1 <= r_h;
                if r_h = '1' then
                    if (r_sl = 0 and r_e = 47) or (r_sl /= 0 and r_e = 31) then
                        r_e <= (others => '0');
                        if r_sl = 2 then r_run <= '0'; else r_sl <= r_sl + 1; end if;
                    else
                        r_e <= r_e + 1;
                    end if;
                end if;
                r_h <= not r_h;
            end if;
            r_v2 <= r_v1; r_h2 <= r_h1; r_isb2 <= r_isb1; r_la2 <= r_la1;
            r_v3 <= r_v2; r_h3 <= r_h2; r_isb3 <= r_isb2; r_la3 <= r_la2;
            if r_v3 = '1' then
                if r_isb3 = '1' then s_lwd <= s_cqbf; else s_lwd <= s_cqaf; end if;
                s_lwa <= e_bank & r_la3;
                if r_h3 = '0' then s_lwe <= '1'; else s_lwo <= '1'; end if;
            end if;

            ---------------------------------------------------------------
            if s_linit = '1' then
                e_vy8 <= (others => '0'); e_bj <= s_bph;
                q_run <= '0'; r_run <= '0';
                e_st <= to_unsigned(2, 5);
            elsif data_in.avid = '1' and s_avq = '0' then
                e_bank <= not e_bank;
                q_run <= '0'; r_run <= '0';
                e_st <= to_unsigned(1, 5);
            elsif d_c /= 0 or m_go = '1' or m_cnt /= 0 then
              if d_c /= 0 then
                v_a := d_r(16 downto 0) & d_n(25);
                if v_a >= resize(d_d, 18) then
                    d_r <= v_a - resize(d_d, 18);
                    d_n <= d_n(24 downto 0) & '1';
                else
                    d_r <= v_a;
                    d_n <= d_n(24 downto 0) & '0';
                end if;
                d_c <= d_c - 1;
              end if;
            else
                case to_integer(e_st) is
                    when 1 =>       -- advance to the next line
                        e_vy8 <= e_vy8 + resize(s_ys, 20);
                        e_bj <= e_bj + s_bjs;
                        e_st <= e_st + 1;
                    when 2 =>       -- row (clamped)
                        if e_vy8(19 downto 8) > 1079 then e_vy <= to_unsigned(1079, 11); else e_vy <= e_vy8(18 downto 8); end if;
                        e_st <= to_unsigned(22, 5);
                    when 22 =>      -- psi row-knot; band roll
                        v_y := e_vy;
                        if v_y < 224 then e_pk <= to_unsigned(0, 6); e_pfy <= (others => '0');
                        elsif v_y < 288 then e_pk <= to_unsigned(1, 6); e_pfy <= resize(shift_left(v_y - 224, 2), 9);
                        elsif v_y < 768 then e_pk <= resize(2 + shift_right(v_y - 288, 5), 6); e_pfy <= resize(shift_left((v_y - 288) and to_unsigned(31, 11), 3), 9);
                        elsif v_y < 944 then e_pk <= resize(17 + shift_right(v_y - 768, 4), 6); e_pfy <= resize(shift_left((v_y - 768) and to_unsigned(15, 11), 4), 9);
                        else e_pk <= resize(28 + shift_right(v_y - 944, 3), 6); e_pfy <= resize(shift_left((v_y - 944) and to_unsigned(7, 11), 5), 9); end if;
                        v_u := resize(v_y, 18) + resize(s_roll, 18);
                        if v_u >= 1080 then v_u := v_u - 1080; end if;
                        e_yr <= v_u(10 downto 0);
                        e_st <= to_unsigned(23, 5);
                    when 23 =>      -- seam offset
                        e_d800 <= signed(resize(e_yr, 12)) - 800;
                        e_st <= to_unsigned(3, 5);
                    when 3 =>       -- seam zone: yt = 800 + (yr - 800) inv / 256
                        m_a <= resize(e_d800, 14); m_b <= signed(resize(s_inv(10 downto 1), 11)); m_go <= '1'; m_clr <= '1';
                        e_st <= e_st + 1;
                    when 4 =>
                        e_st <= e_st + 1;
                    when 5 =>
                        if abs(e_d800) < signed(resize(s_h36, 12)) then
                            v_d := resize(shift_right(m_acc, 7), 12) + 800;
                            if v_d < 764 then e_yc <= to_unsigned(764, 11);
                            elsif v_d > 835 then e_yc <= to_unsigned(835, 11);
                            else e_yc <= unsigned(v_d(10 downto 0)); end if;
                            e_tear <= '1';
                        elsif e_yr >= 764 and e_yr < 836 then
                            if e_yr < 800 then e_yc <= to_unsigned(763, 11); else e_yc <= to_unsigned(837, 11); end if;
                            e_tear <= '0';
                        else
                            e_yc <= e_yr; e_tear <= '0';
                        end if;
                        e_st <= e_st + 1;
                    when 6 =>       -- slot bands, staircase, seam row values, seam shift word
                        if e_yc < 252 then e_band <= ("000", "000", "000");
                        elsif e_yc < 256 then e_band <= ("000", "001", "000");
                        elsif e_yc < 320 then e_band <= ("001", "001", "000");
                        elsif e_yc < 546 then e_band <= ("001", "001", "001");
                        elsif e_yc < 764 then e_band <= ("010", "010", "010");
                        elsif e_yc < 786 then e_band <= ("010", "011", "100");
                        elsif e_yc < 836 then e_band <= ("011", "011", "100");
                        else e_band <= ("100", "100", "100"); end if;
                        if e_yc >= 256 and e_yc < 320 then e_stair <= '1'; else e_stair <= '0'; end if;
                        e_ct1 <= resize(e_yc - 256, 18);
                        if e_yc < 800 then e_xt <= to_unsigned(560, 11);
                        elsif e_yc >= 832 then e_xt <= to_unsigned(304, 11);
                        else e_xt <= to_unsigned(560, 11) - resize(shift_left(e_yc - 800, 3), 11); end if;
                        v_u := resize(e_yc - 764, 18);
                        if e_tear = '1' then s_pa <= v_u(6 downto 1) & "11111"; else s_pa <= to_unsigned(31, 11); end if;
                        e_st <= to_unsigned(24, 5);
                    when 24 =>      -- staircase threshold (yc - 256) 277 / 1024 + 1
                        v_u := shift_right(shift_left(e_ct1, 8) + shift_left(e_ct1, 4) + shift_left(e_ct1, 2) + e_ct1, 10);
                        e_cthr <= resize(v_u(4 downto 0) + 1, 5);
                        e_sl <= (others => '0'); e_hh <= '0';
                        e_st <= to_unsigned(7, 5);
                    when 7 =>       -- slot knot: uniform knots per band (two clocks per slot)
                        if e_hh = '0' then
                            case to_integer(e_band(to_integer(e_sl))) is
                                when 0 => e_k0 <= to_unsigned(0, 5);  e_t <= signed(resize(e_yc, 12));       e_sh <= "00"; e_nm <= to_unsigned(8, 4);
                                when 1 => e_k0 <= to_unsigned(10, 5); e_t <= signed(resize(e_yc, 12)) - 256; e_sh <= "10"; e_nm <= to_unsigned(2, 4);
                                when 2 => e_k0 <= to_unsigned(14, 5); e_t <= signed(resize(e_yc, 12)) - 544; e_sh <= "00"; e_nm <= to_unsigned(7, 4);
                                when 3 => e_k0 <= to_unsigned(23, 5); e_t <= signed(resize(e_yc, 12)) - 768; e_sh <= "00"; e_nm <= to_unsigned(1, 4);
                                when others => e_k0 <= to_unsigned(26, 5); e_t <= signed(resize(e_yc, 12)) - 768; e_sh <= "01"; e_nm <= to_unsigned(4, 4);
                            end case;
                            e_hh <= '1';
                        else
                            v_t := e_t;
                            case e_sh is
                                when "00" => v_i := unsigned(v_t(10 downto 5)); v_f := unsigned(v_t(4 downto 0)) & '0';
                                when "01" => v_i := '0' & unsigned(v_t(10 downto 6)); v_f := unsigned(v_t(5 downto 0));
                                when others => v_i := "00" & unsigned(v_t(10 downto 7)); v_f := unsigned(v_t(6 downto 1));
                            end case;
                            if v_t < 0 then
                                e_kk(to_integer(e_sl)) <= e_k0; e_fy(to_integer(e_sl)) <= (others => '0');
                            elsif v_i > e_nm then
                                e_kk(to_integer(e_sl)) <= e_k0 + resize(e_nm, 5); e_fy(to_integer(e_sl)) <= (others => '1');
                            else
                                e_kk(to_integer(e_sl)) <= e_k0 + resize(v_i, 5); e_fy(to_integer(e_sl)) <= v_f;
                            end if;
                            e_hh <= '0';
                            if e_sl = 2 then e_st <= e_st + 1; else e_sl <= e_sl + 1; end if;
                        end if;
                    when 8 =>       -- seam shift; R slope knots
                        if e_yc(0) = '1' then v_tw := s_pqf(15 downto 8); else v_tw := s_pqf(7 downto 0); end if;
                        if e_tear = '1' then e_shift <= signed(resize(unsigned(v_tw), 11)) - 40 + resize(s_troll, 11);
                        else e_shift <= (others => '0'); end if;
                        e_ki <= e_kk(0);
                        e_st <= e_st + 1;
                    when 9 =>
                        e_sk0 <= to_signed(C_SK(to_integer(e_ki)), 13);
                        e_ki <= e_ki + 1;
                        e_st <= e_st + 1;
                    when 10 =>      -- s lerp: (sk1 - sk0) fy / 64
                        m_a <= resize(to_signed(C_SK(to_integer(e_ki)), 13) - e_sk0, 14); m_b <= signed(resize(e_fy(0), 11));
                        m_go <= '1'; m_clr <= '1';
                        e_st <= e_st + 1;
                    when 11 =>      -- s = sk0 + ...; s k3
                        v_s := resize(e_sk0, 14) + resize(shift_right(m_acc, 6), 14);
                        m_a <= resize(v_s, 14); m_b <= signed(resize(s_k3, 11));
                        m_go <= '1'; m_clr <= '1';
                        e_st <= e_st + 1;
                    when 12 =>      -- s_eff = s k3 / 512; cb = gf (256 - fy) / 256
                        if m_acc < 0 then n_s <= (others => '0');
                        elsif shift_right(m_acc, 9) > 4095 then n_s <= (others => '1');
                        else n_s <= unsigned(m_acc(20 downto 9)); end if;
                        m_a <= resize(s_gfs, 14); m_b <= signed(resize(256 - e_pfy, 11));
                        m_go <= '1'; m_clr <= '1';
                        e_st <= e_st + 1;
                    when 13 =>      -- cc = gf fy / 256
                        e_cb <= resize(shift_right(m_acc, 8), 12);
                        m_a <= resize(s_gfs, 14); m_b <= signed(resize(e_pfy, 11));
                        m_go <= '1'; m_clr <= '1';
                        e_st <= e_st + 1;
                    when 14 =>      -- start the psi loop and the colour loop; line values
                        e_cc <= resize(shift_right(m_acc, 8), 12);
                        s_pa <= e_pk & "00000";
                        q_run <= '1'; q_more <= '1'; q_w <= to_unsigned(3, 2); q_r <= (others => '0'); q_j <= (others => '0');
                        q_cr <= (others => '0');
                        q_found <= '0'; q_done <= '0'; q_neg0 <= '1';
                        r_run <= '1'; r_sl <= (others => '0'); r_e <= (others => '0'); r_h <= '0';
                        if s_nobead = '1' then n_jit <= (others => '0');
                        else n_jit <= to_signed(C_SIN16(to_integer(e_bj(15 downto 12))), 5); end if;
                        n_oy <= e_vy(8 downto 1) + s_ot;
                        n_tear <= e_tear; n_stair <= e_stair; n_xt <= e_xt; n_cthr <= e_cthr; n_band <= e_band; n_fy <= e_fy;
                        e_st <= to_unsigned(16, 5);
                    when 16 =>      -- wait for the psi loop, then x0 (frac of the root segment)
                        if q_done = '1' then
                            if q_neg0 = '0' then
                                e_x0 <= (others => '0'); e_st <= to_unsigned(18, 5);
                            elsif q_found = '0' then
                                e_x0 <= to_unsigned(1920, 11); e_st <= to_unsigned(18, 5);
                            else
                                d_n <= shift_left(resize(unsigned(-q_rqa), 26), 6);   -- (-qa) * 64
                                d_d <= resize(unsigned(resize(q_rqb, 17) - resize(q_rqa, 17)), 17);
                                d_r <= (others => '0'); d_c <= to_unsigned(26, 5);
                                e_st <= e_st + 1;
                            end if;
                        end if;
                    when 17 =>
                        if d_n > 63 then v_u := to_unsigned(63, 18); else v_u := resize(d_n(5 downto 0), 18); end if;
                        e_x0 <= resize(shift_left(resize(q_jr - 1, 11), 6), 11) + resize(v_u(5 downto 0), 11);
                        e_st <= e_st + 1;
                    when 18 =>      -- first-cycle DDA step = vstep 2^20 / max(x0, 128)
                        n_x0 <= e_x0;
                        d_n <= shift_left(resize(s_vstep, 26), 20);
                        if e_x0 < 128 then d_d <= to_unsigned(128, 17); else d_d <= resize(e_x0, 17); end if;
                        d_r <= (others => '0'); d_c <= to_unsigned(26, 5);
                        e_st <= e_st + 1;
                    when 19 =>      -- red fade: g0 = 2^20 + 1000 s, step = s vstep / 16
                        if d_n > 1048575 then n_rst <= (others => '1'); else n_rst <= d_n(19 downto 0); end if;
                        m_a <= signed(resize(n_s, 14)); m_b <= signed(resize(s_vstep, 11));
                        m_go <= '1'; m_clr <= '1';
                        e_st <= e_st + 1;
                    when 20 =>
                        n_gst <= unsigned(m_acc(16 downto 4));
                        m_a <= signed(resize(n_s, 14)); m_b <= to_signed(1000, 11);
                        m_go <= '1'; m_clr <= '1';
                        e_cnt <= -shift_left(resize(e_shift, 16), 4);
                        e_st <= e_st + 1;
                    when 21 =>
                        n_g0 <= to_unsigned(2**20, 22) + unsigned(m_acc(21 downto 0));
                        n_cnt0 <= unsigned(e_cnt(9 downto 0));     -- (- 16 shift) mod 1024
                        e_st <= e_st + 1;
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
    -- Beam (stage 0): virtual x (Q4), the first-cycle DDA, the seam chevron counter
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- Beams: stage 0 (virtual x, Q4), stage 6 (x, the first-cycle DDA, the seam chevron
    -- counter), stage 13 (x for the red fade).  The later beams restart on the delayed g_start
    -- and step with the delayed avid, so they need no pipeline registers.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_a : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            g_ssr <= g_ssr(14 downto 0) & g_start;
            if g_start = '1' then b_ln <= b_ln + 1; end if;
            if g_ssr(7) = '1' then b9_pc <= (others => '0'); elsif s_gav_sr(7) = '1' then b9_pc <= b9_pc + 1; end if;
            if g_start = '1' then
                b_x16 <= resize(l_jit, 17);
            elsif g_avid = '1' then
                b_x16 <= b_x16 + signed(resize(s_vstep, 17));
            end if;
            if g_ssr(5) = '1' then
                b6_x16 <= resize(l_jit, 17);
                b6_a1 <= (others => '0');
                b6_cnt <= l_cnt0;
            elsif s_gav_sr(5) = '1' then
                b6_x16 <= b6_x16 + signed(resize(s_vstep, 17));
                v_a := resize(b6_a1, 26) + resize(l_rst, 26);
                if v_a(25 downto 24) /= "00" then b6_a1 <= to_unsigned(2**24, 25); else b6_a1 <= v_a(24 downto 0); end if;
                b6_cnt <= b6_cnt + resize(s_vstep, 10);
            end if;
            if g_ssr(11) = '1' then
                b16_g <= signed(resize(l_g0, 23));
            elsif s_gav_sr(11) = '1' then
                b16_g <= b16_g - signed(resize(l_gst, 23));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_xm : unsigned(14 downto 0);
        variable v_j : unsigned(4 downto 0);
        variable v_ps : signed(16 downto 0);
        variable v_dl : signed(16 downto 0);
        variable v_bin, v_nxt : unsigned(6 downto 0);
        variable v_pd : signed(16 downto 0);
        variable v_nl : signed(16 downto 0);
        variable v_ea : unsigned(6 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_b : unsigned(2 downto 0);
        variable v_b4 : unsigned(1 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dly : unsigned(9 downto 0);
        variable v_t8 : unsigned(7 downto 0);
        variable v_x : unsigned(10 downto 0);
        variable v_fy : unsigned(5 downto 0);
        variable v_s12 : signed(11 downto 0);
        variable v_t : unsigned(12 downto 0);
        variable v_rm : unsigned(9 downto 0);
        variable v_u15 : unsigned(14 downto 0);
        variable v_o10 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -------------------------------------------------------------------
            -- 1: mirror, knot addresses, source
            v_xm := f_mirror(b_x16, '0');
            v_j := v_xm(14 downto 10);
            s_rae <= "000" & l_bank & resize(shift_right(v_j + 1, 1), 4);
            s_rao <= "000" & l_bank & v_j(4 downto 1);
            p_jo(1) <= v_j(0);
            p_fr(1) <= v_xm(9 downto 0);
            if s_osc = '1' then
                v_t8 := v_xm(13 downto 6) + l_oy;
                if v_t8(7) = '0' then p1_L8 <= resize(v_t8(6 downto 0), 8) + 64; else p1_L8 <= 191 - resize(v_t8(6 downto 0), 8); end if;   -- 64..191
            else
                p1_L8 <= v_l3;
            end if;

            -- 2: source minus mid-grey
            p2_dL <= signed(resize(p1_L8, 9)) - 128;

            -- 3: knot values to fabric; melt product
            p3_ve <= s_rqe; p3_vo <= s_rqo;
            p3_mv <= resize(p2_dL * signed('0' & s_k6(9 downto 4)), 16);

            -- 4: knot pair, difference (clamped to +-1023)
            if p_jo(3) = '0' then
                p4_v0 <= resize(signed(p3_ve), 17);
                v_dl := resize(signed(p3_vo), 17) - resize(signed(p3_ve), 17);
            else
                p4_v0 <= resize(signed(p3_vo), 17);
                v_dl := resize(signed(p3_ve), 17) - resize(signed(p3_vo), 17);
            end if;
            if v_dl > 1023 then p4_dl <= to_signed(1023, 11);
            elsif v_dl < -1024 then p4_dl <= to_signed(-1024, 11);
            else p4_dl <= v_dl(10 downto 0); end if;
            p_mv(4) <= resize(shift_right(p3_mv, 7), 13);

            -- 5: interpolation product
            p5_pr <= resize(p4_dl * signed('0' & p_fr(4)(9 downto 4)), 22);
            p5_v0 <= p4_v0; p5_dl <= p4_dl;

            -- 6: psiS
            p6_ps <= p5_v0 + resize(shift_right(p5_pr, 6), 17);
            p6_dl <= p5_dl;

            -- 7: melt (strings only); reciprocal ROM address (knot step dl >> 2); stage-6 beam ->
            --    first cycle, seam
            if p6_ps >= 0 then p7_ps <= p6_ps + resize(p_mv(6), 17); else p7_ps <= p6_ps; end if;
            if p6_dl < 4 then s_rca <= (others => '0'); else s_rca <= unsigned(p6_dl(9 downto 2)); end if;
            if b6_a1(24) = '1' then p7_u1 <= (others => '1'); else p7_u1 <= b6_a1(23 downto 14); end if;
            p7_cu <= b6_cnt;
            v_x := f_mirror(b6_x16, '0')(14 downto 4);
            p7_x <= v_x;
            if l_tear = '1' and v_x < l_xt then p7_chv <= '1'; else p7_chv <= '0'; end if;
            if l_tear = '1' and v_x >= l_xt then p7_late <= '1'; else p7_late <= '0'; end if;

            -- 8: region, bins, slot; dip line, parity
            v_ps := p7_ps;
            p8_F <= '0';
            if v_ps < 0 then
                p8_F <= '1';
                if p7_chv = '1' then
                    v_bin := "0100000" + resize(p7_cu(9 downto 5), 7);
                    if v_bin = 63 then v_nxt := to_unsigned(32, 7); else v_nxt := v_bin + 1; end if;
                    p8_f5 <= p7_cu(4 downto 0);
                else
                    v_bin := "0100000" + resize(p7_u1(9 downto 5), 7);
                    if v_bin = 63 then v_nxt := to_unsigned(62, 7); p8_f5 <= (others => '0');
                    else v_nxt := v_bin + 1; p8_f5 <= p7_u1(4 downto 0); end if;
                end if;
                if p7_late = '1' then p8_slot <= "10"; else p8_slot <= "01"; end if;
            elsif v_ps < 128 then
                v_bin := "1000000" + resize(unsigned(v_ps(6 downto 3)), 7);
                if v_bin = 79 then v_nxt := (others => '0'); else v_nxt := v_bin + 1; end if;
                p8_f5 <= unsigned(v_ps(2 downto 0)) & "00";
                p8_slot <= "00";
            else
                v_pd := v_ps + signed(resize(s_drift, 17));
                v_bin := resize(unsigned(v_pd(7 downto 3)), 7);
                v_nxt := resize(unsigned(v_pd(7 downto 3)) + 1, 7);
                p8_f5 <= unsigned(v_pd(2 downto 0)) & "00";
                if l_stair = '1' and v_ps(16 downto 8) >= signed(resize(l_cthr, 9)) then p8_slot <= "10"; else p8_slot <= "00"; end if;
            end if;
            p8_bin <= v_bin; p8_nxt <= v_nxt;
            v_nl := shift_right(v_ps + 64, 7);
            p8_frs <= resize(v_ps - shift_left(v_nl, 7), 8);
            p8_par <= v_nl(0);
            p8_x <= p7_x;

            -- 9: ordered-dither bin choice -> line-LUT entry (rows k / k+1 sit in the two RAMs at
            --    one address); reciprocal to fabric; d0; row weight (3 bits + a dither bit)
            if p8_f5 > (C_B4(to_integer(b_ln & b9_pc)) & '0') then v_bin := p8_nxt; else v_bin := p8_bin; end if;
            case to_integer(p8_slot) is
                when 0 =>
                    if v_bin(6) = '1' then v_ea := resize(v_bin(3 downto 0), 7) + 32;
                    else v_ea := resize(v_bin(4 downto 0), 7); end if;
                when 1 => v_ea := resize(v_bin(4 downto 0), 7) + 48;
                when others => v_ea := resize(v_bin(4 downto 0), 7) + 80;
            end case;
            s_lae <= l_bank & v_ea; s_lao <= l_bank & v_ea;
            v_fy := l_fy(to_integer(p8_slot));
            if v_fy(2 downto 0) > C_B4(to_integer((b_ln + 2) & (b9_pc + 1)))(3 downto 1) then
                p9_w <= resize(v_fy(5 downto 3), 4) + 1;
            else
                p9_w <= resize(v_fy(5 downto 3), 4);
            end if;
            p9_rec <= unsigned(s_rcq);
            p9_slot <= p8_slot; p9_F <= p8_F; p9_par <= p8_par; p9_frs <= p8_frs;
            p9_d0 <= signed(resize(p8_x, 13)) - signed(resize(l_x0, 13));

            -- 10: LUT read; dip product frs x rec (px = frs 64 / dl)
            p10_dp <= p9_frs * signed('0' & p9_rec);
            p10_d0 <= p9_d0; p10_w <= p9_w;
            p10_slot <= p9_slot; p10_F <= p9_F; p10_par <= p9_par;

            -- 11: LUT words to fabric; pixel distance to the line
            p11_qe <= s_lqe; p11_qo <= s_lqo;
            if p10_F = '1' then p11_d <= p10_d0; else p11_d <= resize(shift_right(p10_dp, 7), 13); end if;
            p11_w <= p10_w; p11_slot <= p10_slot; p11_F <= p10_F; p11_par <= p10_par;

            -- 12: row lerp operands (YUV565: Y6 U5 V5); red from row k: 27 qy / 8 + 6 (qu - 16)
            --     (= 0.42 q8y + 1.5 (q8u - 128)); dip address
            v_u15 := shift_left(resize(unsigned(p11_qe(15 downto 10)), 15), 5) - shift_left(resize(unsigned(p11_qe(15 downto 10)), 15), 2)
                     - resize(unsigned(p11_qe(15 downto 10)), 15);
            p12_ry <= v_u15(10 downto 3);
            v_s12 := signed(resize(unsigned(p11_qe(9 downto 5)), 12)) - 16;
            p12_ru <= resize(shift_left(v_s12, 2) + shift_left(v_s12, 1), 10);
            p12_a8(0) <= resize(unsigned(p11_qe(15 downto 10)) & "000", 10);
            p12_a8(1) <= resize(unsigned(p11_qe(9 downto 5)) & "000", 10);
            p12_a8(2) <= resize(unsigned(p11_qe(4 downto 0)) & "000", 10);
            p12_d(0) <= signed(resize(unsigned(p11_qo(15 downto 10)), 7)) - signed(resize(unsigned(p11_qe(15 downto 10)), 7));
            p12_d(1) <= signed(resize(unsigned(p11_qo(9 downto 5)), 7)) - signed(resize(unsigned(p11_qe(9 downto 5)), 7));
            p12_d(2) <= signed(resize(unsigned(p11_qo(4 downto 0)), 7)) - signed(resize(unsigned(p11_qe(4 downto 0)), 7));
            p12_w <= p11_w;
            v_b := l_band(to_integer(p11_slot));
            if v_b = 4 then v_b4 := "11"; else v_b4 := v_b(1 downto 0); end if;
            if s_soft = '0' and v_b /= 3 and p11_d >= -12 and p11_d <= 28 then p_dok(12) <= '1'; else p_dok(12) <= '0'; end if;
            v_d := p11_d + 12;
            if v_d < 0 then v_d := (others => '0'); end if;
            if v_d > 40 then v_d := to_signed(40, 13); end if;
            if p11_F = '1' then
                s_da <= v_b4 & '0' & unsigned(v_d(5 downto 0));
            else
                s_da <= v_b4 & p11_par & unsigned(v_d(5 downto 0));
            end if;

            -- 13: q x 8 = a x 8 + (b - a) w; R8 clamped; fade weight (256 - g) / 16 (stage-13 beam)
            for c in 0 to 2 loop
                v_s12 := signed(resize(p12_a8(c), 12)) + resize(p12_d(c) * signed('0' & p12_w), 12);
                p13_q8(c) <= unsigned(v_s12(9 downto 0));
            end loop;
            v_s12 := signed(resize(p12_ry, 12)) + resize(p12_ru, 12);
            if v_s12 < 0 then p13_r8 <= (others => '0');
            elsif v_s12 > 255 then p13_r8 <= (others => '1');
            else p13_r8 <= unsigned(v_s12(7 downto 0)); end if;
            if b16_g < 0 then p13_ng <= to_unsigned(16, 5);
            elsif b16_g(22 downto 12) >= 256 then p13_ng <= (others => '0');
            else p13_ng <= resize(shift_right(to_unsigned(256, 9) - resize(unsigned(b16_g(19 downto 12)), 9), 4), 5); end if;

            -- 14: decode: Y = 64 + 14 q, chroma = 32 + 30 q; Rm x 16 = R8 (256 - g) / 16
            v_t := shift_left(resize(p13_q8(0), 13), 3) - resize(p13_q8(0), 13);
            p14_c(0) <= resize(shift_right(v_t, 2), 10) + 64;
            v_t := shift_left(resize(p13_q8(1), 13), 4) - resize(p13_q8(1), 13);
            p14_c(1) <= resize(shift_right(v_t, 2), 10) + 32;
            v_t := shift_left(resize(p13_q8(2), 13), 4) - resize(p13_q8(2), 13);
            p14_c(2) <= resize(shift_right(v_t, 2), 10) + 32;
            p14_rm <= p13_r8 * p13_ng;

            -- 15: the fade in YUV (Y -= 1.27 Rm, Cr -= 1.75 Rm, Cb += 0.59 Rm); the line-core dips
            v_rm := resize(p14_rm(12 downto 4), 10);
            p15_fy <= v_rm + shift_right(v_rm, 2) + shift_right(v_rm, 6);
            p15_fu <= v_rm + shift_right(v_rm, 1) + shift_right(v_rm, 2);
            p15_fv <= shift_right(v_rm, 1) + shift_right(v_rm, 4) + shift_right(v_rm, 5);
            if p_dok(14) = '1' then
                p15_dp(0) <= shift_left(resize(signed(s_dqrgf(15 downto 8)), 11), 2);
                p15_dp(1) <= shift_left(resize(signed(s_dqrgf(7 downto 0)), 11), 2);
                p15_dp(2) <= shift_left(resize(signed(s_dqbf), 11), 2);
            else
                p15_dp <= (others => (others => '0'));
            end if;
            p15_c <= p14_c;

            -- 16: sums
            p16_s(0) <= signed(resize(p15_c(0), 12)) - signed(resize(p15_fy, 12)) + resize(p15_dp(0), 12);
            p16_s(1) <= signed(resize(p15_c(1), 12)) - signed(resize(p15_fu, 12)) + resize(p15_dp(1), 12);
            p16_s(2) <= signed(resize(p15_c(2), 12)) + signed(resize(p15_fv, 12)) + resize(p15_dp(2), 12);

            -- 17: clamp (Y 64..940, chroma 0..1023); Dusk (Cr <-> Cb)
            if p16_s(0) < 64 then s_y21 <= to_unsigned(64, 10);
            elsif p16_s(0) > 940 then s_y21 <= to_unsigned(940, 10);
            else s_y21 <= unsigned(p16_s(0)(9 downto 0)); end if;
            for c in 1 to 2 loop
                if p16_s(c) < 0 then v_o10 := (others => '0');
                elsif p16_s(c) > 1023 then v_o10 := (others => '1');
                else v_o10 := unsigned(p16_s(c)(9 downto 0)); end if;
                if (c = 1) = (s_dusk = '0') then s_u21 <= v_o10; else s_v21 <= v_o10; end if;
            end loop;

            -- 18..23: output conditioning (s_y21 / s_u21 / s_v21 are the stage-17 colour)
            -- chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums, luma [1 2 1]; centre at
            -- stage 22 -> 23
            s_csu(0) <= s_u21; s_csv(0) <= s_v21;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u21 - s_csu(7);
            s_boxv <= s_boxv + s_v21 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u27 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v27 <= v_sc(13 downto 4);
            s_y22 <= s_y21; s_y23 <= s_y22; s_y24 <= s_y23; s_y25 <= s_y24;
            s_y26 <= s_y25; s_y27x <= s_y26;
            v_sc := resize(s_y25, 15) + shift_left(resize(s_y26, 15), 1) + resize(s_y27x, 15);
            s_yf27 <= v_sc(11 downto 2);
            -- 24: luma toward black (white 940 -> 777); blanking gate
            if s_yf27 > 64 then v_dly := s_yf27 - 64; else v_dly := (others => '0'); end if;
            if s_gav_sr(22) = '1' then
                s_y28 <= s_yf27 - shift_right(v_dly, 3) - shift_right(v_dly, 4);
                s_u28 <= s_u27; s_v28 <= s_v27;
            else
                s_y28 <= to_unsigned(64, 10); s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;

            -- delays
            for i in 2 to 4 loop p_fr(i) <= p_fr(i - 1); end loop;
            for i in 2 to 3 loop p_jo(i) <= p_jo(i - 1); end loop;
            for i in 5 to 6 loop p_mv(i) <= p_mv(i - 1); end loop;
            for i in 13 to 14 loop p_dok(i) <= p_dok(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture waterfall;
