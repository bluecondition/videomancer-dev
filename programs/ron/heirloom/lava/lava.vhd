-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lava v0.5.0 -- Heirloom #43, rebuilt from the photo WIN_20230605_01_14_56.
--
-- The photo: a red, glossy lava sheet hangs over violet / magenta vertical stripes.  Across the right half
-- it is a lattice of rows -- peach-cored worms, a red zigzag rim, olive diamonds with pink hearts, a white web
-- with orange highlights, rimmed red blobs, green / violet scallops -- and on the left the same rows bend up
-- into long hanging drapes whose tips step down to the right, with the stripes showing below them.  Every
-- column of it is ONE period (167 px) of a horizontal oscillator; the rows are a warped vertical coordinate.
--
-- The program draws exactly that: the oscillator phase picks a column of a stored TILE (one period of the
-- photo, fitted: 24 x 164 luma knots, 12 x 82 chroma knots, bilinear), and a stored WARP (48 x 42 knots, fitted)
-- gives the row coordinate t(x, y).  A per-line engine interpolates the warp row for the NEXT line; the pixel
-- path interpolates it across the line, applies the controls to t, adds the 8.5-line comb to the phase and
-- reads the tile.
--
--   K1  Columns  oscillator frequency 0.5x .. 2x (the photo: 11.5 drops across)
--   K2  Bend     how far the rows bend up into drapes (0 = a flat lattice, 1 = the photo, 1.6)
--   K3  Rows     row density about the top of the sheet (0.6x .. 1.6x)
--   K4  Speed    bipolar (centre +-8 still): the drops slide sideways
--   K5  Ripple   the serrated 8.5-line comb on the lava's edges (0 .. 8 px)
--   K6  Melt     how far the video (or Osc) bends the rows
--   S7  Mirror   the left half reflected: twin curtains
--   S8  Ground   Stripes / Black (the background under the lava)
--   S9  Flip     the sheet rises from the bottom
--   S10 Palette  Lava / Ice (U and V exchanged)
--   S11 Source   Video / Osc
--   P12 FLOW     0 % = the lava drawn back into the top-right corner, 50 % = the photo, 100 % = it floods down
--
-- Virtual 1080-line geometry (the same picture at SD, 720p and 1080; 4:3 SD shows the central 1440 virtual px).
-- Generated avid; output conditioning (luma [1 2 1], chroma box pair [1 2 .. 2 1], luma to ~777) before the
-- blanking gate.  Latency 36 clocks in every mode.  Generated colours in the hardware convention (U = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lava_pkg.all;
use work.all;

architecture lava of program_top is
    constant C_LAT : integer := 36;          -- matches processing_delay_clks
    constant C_K0  : integer := 100523;      -- 2^24 / 166.9: the photo's oscillator, phase per virtual px
    constant C_PHC : integer := 1156009;     -- frac((960 - 114) / 166.9) * 2^24

    -- per-timing-ID constants
    type t_mode is record
        vx   : unsigned(9 downto 0);   -- virtual px per pixel, Q8
        ku   : unsigned(11 downto 0);  -- warp cells (42 virtual px) per pixel, Q16
        u0   : unsigned(18 downto 0);  -- warp cell of the first pixel, Q16
        xm   : unsigned(9 downto 0);   -- 960 - first virtual px
        cmid : unsigned(10 downto 0);  -- pixel index of the virtual centre
    end record;
    function f_mode(t : std_logic_vector(3 downto 0)) return t_mode is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return (to_unsigned(512, 10), to_unsigned(3121, 12), to_unsigned(374491, 19),
                                           to_unsigned(720, 10), to_unsigned(360, 11));
            when 5 | 6 | 14     => return (to_unsigned(384, 10), to_unsigned(2341, 12), to_unsigned(0, 19),
                                           to_unsigned(960, 10), to_unsigned(640, 11));
            when others         => return (to_unsigned(256, 10), to_unsigned(1560, 12), to_unsigned(0, 19),
                                           to_unsigned(960, 10), to_unsigned(960, 11));
        end case;
    end function;
    -- virtual lines per line (Q8), knot-row step per line (Q16 of 27 lines), comb step (Q16 of 8.5 lines)
    type t_vmode is record
        vy    : unsigned(10 downto 0);
        jst   : unsigned(13 downto 0);
        jf0   : unsigned(12 downto 0);
        comb  : unsigned(15 downto 0);
    end record;
    function f_vmode(t : std_logic_vector(3 downto 0)) return t_vmode is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return (to_unsigned(1138, 11), to_unsigned(10788, 14), to_unsigned(5394, 13), to_unsigned(34274, 16));
            when 4          => return (to_unsigned(576, 11),  to_unsigned(5460, 14),  to_unsigned(2730, 13), to_unsigned(17348, 16));
            when 8          => return (to_unsigned(960, 11),  to_unsigned(9101, 14),  to_unsigned(4550, 13), to_unsigned(28913, 16));
            when 12         => return (to_unsigned(480, 11),  to_unsigned(4550, 14),  to_unsigned(2275, 13), to_unsigned(14456, 16));
            when 5 | 6 | 14 => return (to_unsigned(384, 11),  to_unsigned(3640, 14),  to_unsigned(1820, 13), to_unsigned(11565, 16));
            when 1 | 2 | 10 => return (to_unsigned(512, 11),  to_unsigned(4854, 14),  to_unsigned(2427, 13), to_unsigned(15420, 16));
            when others     => return (to_unsigned(256, 11),  to_unsigned(2427, 14),  to_unsigned(1213, 13), to_unsigned(7710, 16));
        end case;
    end function;
    function f_isshort(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 | 5 | 6 | 14 => return '1';
            when others => return '0';
        end case;
    end function;
    -- 16-bit phase -> triangle -129 .. 127
    function f_tri(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 10) - signed(resize(v(15 downto 7), 10));
    end function;

    -- timing
    signal t_m  : t_mode := (to_unsigned(256, 10), to_unsigned(1560, 12), to_unsigned(0, 19), to_unsigned(960, 10), to_unsigned(960, 11));
    signal t_v  : t_vmode := (to_unsigned(256, 11), to_unsigned(2427, 14), to_unsigned(1213, 13), to_unsigned(7710, 16));
    signal t_short : std_logic := '0';

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K5, K6, P12 (Q.3), power up at the defaults
    type t_g is array(0 to 5) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(341 * 8, 13), to_unsigned(650 * 8, 13), to_unsigned(409 * 8, 13),
                         to_unsigned(320 * 8, 13), to_unsigned(200 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- serial multiplier (per-field products)
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 6) := (others => '0');
    signal m_a  : unsigned(19 downto 0) := (others => '0');
    signal m_b  : unsigned(17 downto 0) := (others => '0');
    signal m_h  : unsigned(19 downto 0) := (others => '0');
    signal m_l  : unsigned(17 downto 0) := (others => '0');

    -- per-field values
    signal f_fcol  : unsigned(10 downto 0) := to_unsigned(1023, 11);
    signal f_kv    : unsigned(17 downto 0) := to_unsigned(100424, 18);   -- phase per virtual px (Q24)
    signal f_kph   : unsigned(18 downto 0) := to_unsigned(100424, 19);   -- phase per pixel
    signal f_ph0   : unsigned(23 downto 0) := (others => '0');           -- phase of the first pixel
    signal f_drift : unsigned(23 downto 0) := (others => '0');
    signal f_dstep : unsigned(17 downto 0) := (others => '0');
    signal f_sm    : unsigned(8 downto 0) := (others => '0');
    signal f_sneg  : std_logic := '0';
    signal f_bend  : unsigned(7 downto 0) := to_unsigned(128, 8);        -- Q7
    signal f_gb    : unsigned(8 downto 0) := to_unsigned(128, 9);        -- rows * bend, Q7
    signal f_ca    : unsigned(11 downto 0) := (others => '0');           -- comb gain (phase / 2^22 per tri unit)
    signal f_rows  : unsigned(7 downto 0) := to_unsigned(128, 8);        -- Q7
    signal f_ramp  : unsigned(6 downto 0) := to_unsigned(40, 7);         -- comb amplitude, Q4 px
    signal f_melt  : unsigned(7 downto 0) := to_unsigned(50, 8);
    signal f_flow  : signed(10 downto 0) := (others => '0');
    signal f_fk    : signed(10 downto 0) := (others => '0');
    signal f_lph   : unsigned(15 downto 0) := to_unsigned(16384, 16);    -- breath LFO (starts at its zero)
    signal f_br    : signed(6 downto 0) := (others => '0');
    signal f_comb0 : unsigned(15 downto 0) := (others => '0');
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_mir, f_blk, f_flip, f_ice, f_osc : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- per-line engine
    signal e_yv   : unsigned(18 downto 0) := (others => '0');            -- virtual line, Q8
    signal e_jf   : unsigned(21 downto 0) := (others => '0');            -- warp knot row, Q16
    signal e_comb : unsigned(15 downto 0) := (others => '0');
    signal e_wb, r_bank : std_logic := '0';
    signal e_st   : unsigned(4 downto 0) := (others => '0');
    signal e_ye   : unsigned(10 downto 0) := (others => '0');            -- the line's virtual line (flipped)
    signal e_j    : unsigned(5 downto 0) := (others => '0');
    signal e_fy   : unsigned(7 downto 0) := (others => '0');
    signal e_base : unsigned(10 downto 0) := (others => '0');
    signal e_ct   : signed(9 downto 0) := (others => '0');
    signal e_ym   : signed(11 downto 0) := (others => '0');
    signal em_a   : signed(23 downto 0) := (others => '0');             -- serial multiplier
    signal em_b   : unsigned(11 downto 0) := (others => '0');
    signal em_acc : signed(23 downto 0) := (others => '0');
    signal e_lrp  : signed(11 downto 0) := (others => '0');
    signal e_af   : signed(13 downto 0) := (others => '0');
    signal e_iss  : std_logic := '0';
    signal e_i    : unsigned(5 downto 0) := (others => '0');
    signal e_r    : std_logic := '0';
    signal e_tv   : std_logic_vector(0 to 3) := (others => '0');
    signal e_tr   : std_logic_vector(0 to 3) := (others => '0');
    type t_u6a is array (natural range <>) of unsigned(5 downto 0);
    signal e_ti   : t_u6a(0 to 3) := (others => (others => '0'));
    signal r_wa   : unsigned(10 downto 0) := (others => '0');
    signal wr_q, wr_q2 : std_logic_vector(7 downto 0) := (others => '0');
    signal e_ka   : signed(7 downto 0) := (others => '0');
    signal e_dab  : signed(8 downto 0) := (others => '0');
    signal e_ka2  : signed(7 downto 0) := (others => '0');
    signal e_pr   : signed(17 downto 0) := (others => '0');
    signal e_kv, e_kv2 : std_logic := '0';
    signal e_ki, e_ki2 : unsigned(5 downto 0) := (others => '0');
    signal d_ye   : unsigned(10 downto 0) := (others => '0');
    signal d_lrp  : signed(11 downto 0) := (others => '0');
    signal d_af   : signed(13 downto 0) := (others => '0');
    signal l_ye   : unsigned(10 downto 0) := (others => '0');            -- the line on screen
    signal l_lrp  : signed(11 downto 0) := (others => '0');
    signal l_af   : signed(13 downto 0) := (others => '0');

    -- line table (ping-pong by line): even / odd knots, t (13 bit signed)
    type t_lt is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lt_e, lt_o : t_lt := (others => (others => '0'));
    signal lt_we_e, lt_we_o : std_logic := '0';
    signal lt_wa : unsigned(7 downto 0) := (others => '0');
    signal lt_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal lt_rae, lt_rao : unsigned(7 downto 0) := (others => '0');
    signal lt_qe, lt_qo : std_logic_vector(15 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of lt_e : signal is true;
    attribute no_rw_check of lt_o : signal is true;

    -- beam: warp cell (stage 0), the oscillator phase (a late copy of the beam, stage 13), Osc phases
    signal b_u   : unsigned(21 downto 0) := (others => '0');
    signal b_cnt : unsigned(10 downto 0) := (others => '0');
    signal b_dir : std_logic := '0';
    signal b_pa  : unsigned(15 downto 0) := (others => '0');
    signal l_pb  : unsigned(15 downto 0) := (others => '0');
    signal gs_sr : std_logic_vector(0 to 12) := (others => '0');
    signal lp_ph : unsigned(23 downto 0) := (others => '0');
    signal lp_cnt : unsigned(10 downto 0) := (others => '0');
    signal lp_dir : std_logic := '0';

    -- video tap
    type t_u8v is array (natural range <>) of unsigned(7 downto 0);
    signal s_vy : t_u8v(1 to 2) := (others => to_unsigned(128, 8));
    signal s_vs : signed(9 downto 0) := (others => '0');

    -- pixel path (stage N = N registers after the beam)
    signal q1_fx, q2_fx, q3_fx, q4_fx, q5_fx : unsigned(7 downto 0) := (others => '0');
    signal q1_odd, q2_odd, q3_odd : std_logic := '0';
    signal q3_e, q3_o : signed(12 downto 0) := (others => '0');
    signal q4_w0, q4_w1, q5_w0, q6_w0, q7_w0 : signed(12 downto 0) := (others => '0');
    signal q5_dw : signed(13 downto 0) := (others => '0');
    signal q6_ph, q6_pl : signed(17 downto 0) := (others => '0');
    signal q7_pr : signed(21 downto 0) := (others => '0');
    signal q8_t  : signed(13 downto 0) := (others => '0');
    signal q8_src : signed(9 downto 0) := (others => '0');
    signal q9_d  : signed(14 downto 0) := (others => '0');
    signal q9_ms : signed(15 downto 0) := (others => '0');
    type t_bit_a is array (natural range <>) of std_logic;
    signal p_lf  : t_bit_a(9 to 13) := (others => '0');
    signal q10_gh, q10_gl : signed(19 downto 0) := (others => '0');
    signal q10_ms, q11_ms : signed(18 downto 0) := (others => '0');
    signal q11_dg : signed(24 downto 0) := (others => '0');
    signal q12_t3 : signed(15 downto 0) := (others => '0');
    signal q13_tm : unsigned(10 downto 0) := (others => '0');
    signal p_bg  : t_bit_a(13 to 27) := (others => '0');
    signal q14_vy, q14_vc : unsigned(15 downto 0) := (others => '0');
    signal q14_q : unsigned(11 downto 0) := (others => '0');
    signal q15_uy, q15_uc : unsigned(16 downto 0) := (others => '0');
    signal q15_jy, q15_jc : unsigned(7 downto 0) := (others => '0');
    signal q15_fvy : unsigned(3 downto 0) := (others => '0');
    signal q16_iy0, q16_iy1 : unsigned(4 downto 0) := (others => '0');
    signal q16_ic0, q16_ic1 : unsigned(3 downto 0) := (others => '0');
    signal q16_jy0, q16_jy1 : unsigned(7 downto 0) := (others => '0');
    signal q16_jc : unsigned(6 downto 0) := (others => '0');
    type t_u4_a is array (natural range <>) of unsigned(3 downto 0);
    signal p_fuy, p_fuc : t_u4_a(16 to 20) := (others => (others => '0'));
    signal p_fvy : t_u4_a(16 to 23) := (others => (others => '0'));
    type t_u2_a is array (natural range <>) of std_logic_vector(1 downto 0);
    signal p_py : t_u2_a(16 to 19) := (others => (others => '0'));       -- (i0 odd, j0 odd)
    signal p_pc : t_bit_a(16 to 19) := (others => '0');                  -- i0 odd (chroma)
    signal ry_a0, ry_a1, ry_a2, ry_a3 : unsigned(9 downto 0) := (others => '0');
    signal rc_a0, rc_a1 : unsigned(8 downto 0) := (others => '0');
    signal ry_q0, ry_q1, ry_q2, ry_q3 : std_logic_vector(7 downto 0) := (others => '0');
    signal rc_q0, rc_q1 : std_logic_vector(15 downto 0) := (others => '0');
    signal q19_y0, q19_y1, q19_y2, q19_y3 : std_logic_vector(7 downto 0) := (others => '0');
    signal q19_c0, q19_c1 : std_logic_vector(15 downto 0) := (others => '0');
    signal q20_c00, q20_c01, q21_c00, q21_c01 : signed(9 downto 0) := (others => '0');
    signal q20_d0, q20_d1 : signed(9 downto 0) := (others => '0');
    signal q20_r0, q20_b0, q21_r0, q21_b0 : signed(8 downto 0) := (others => '0');
    signal q20_rd, q20_bd : signed(9 downto 0) := (others => '0');
    signal q21_p0, q21_p1 : signed(13 downto 0) := (others => '0');
    signal q21_pr, q21_pb : signed(13 downto 0) := (others => '0');
    signal q22_a, q22_b, q23_a, q24_a : signed(9 downto 0) := (others => '0');
    signal q22_cr, q22_cb, q23_cr, q23_cb, q24_cr, q24_cb, q25_cr, q25_cb : signed(9 downto 0) := (others => '0');
    signal q23_e : signed(9 downto 0) := (others => '0');
    signal q24_pe : signed(13 downto 0) := (others => '0');
    signal q25_y : signed(9 downto 0) := (others => '0');
    signal q26_y, q27_y, q28_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q26_u, q26_v, q27_u, q27_v, q28_u, q28_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- chroma box ring and sync ring
    type t_ring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal rb_ram : t_ring := (others => "1000000000" & "1000000000");
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal rs_ram : t_sring := (others => "111");
    attribute no_rw_check of rb_ram : signal is true;
    attribute no_rw_check of rs_ram : signal is true;
    signal s_wp, rb_ra, rs_ra : unsigned(7 downto 0) := (others => '0');
    signal rb_wd : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal rb_q, rb_q2 : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal rs_q, rs_q2 : std_logic_vector(2 downto 0) := "111";
    signal s_usum, s_vsum, s_ud, s_vd : unsigned(13 downto 0) := (others => '0');
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly : t_u10a(0 to 6) := (others => to_unsigned(64, 10));
    signal q31_y, q32_y, q33_y, q34_y, q35_y, q36_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q31_u, q31_v, q32_u, q32_v, q33_u, q33_v, q34_u, q34_v, q35_u, q35_v, q36_u, q36_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs
    ---------------------------------------------------------------------------
    p_wrom : process(clk)
    begin
        if rising_edge(clk) then
            wr_q <= C_LV_WARP(to_integer(r_wa));
        end if;
    end process p_wrom;

    p_lte : process(clk)
    begin
        if rising_edge(clk) then
            if lt_we_e = '1' then lt_e(to_integer(lt_wa)) <= lt_wd; end if;
            lt_qe <= lt_e(to_integer(lt_rae));
        end if;
    end process p_lte;

    p_lto : process(clk)
    begin
        if rising_edge(clk) then
            if lt_we_o = '1' then lt_o(to_integer(lt_wa)) <= lt_wd; end if;
            lt_qo <= lt_o(to_integer(lt_rao));
        end if;
    end process p_lto;

    p_ty0 : process(clk) begin if rising_edge(clk) then ry_q0 <= C_LV_Y0(to_integer(ry_a0)); end if; end process;
    p_ty1 : process(clk) begin if rising_edge(clk) then ry_q1 <= C_LV_Y1(to_integer(ry_a1)); end if; end process;
    p_ty2 : process(clk) begin if rising_edge(clk) then ry_q2 <= C_LV_Y2(to_integer(ry_a2)); end if; end process;
    p_ty3 : process(clk) begin if rising_edge(clk) then ry_q3 <= C_LV_Y3(to_integer(ry_a3)); end if; end process;
    p_tc0 : process(clk) begin if rising_edge(clk) then rc_q0 <= C_LV_C0(to_integer(rc_a0)); end if; end process;
    p_tc1 : process(clk) begin if rising_edge(clk) then rc_q1 <= C_LV_C1(to_integer(rc_a1)); end if; end process;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field products (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_pr : unsigned(37 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_k  : signed(10 downto 0);
        variable v_13 : signed(14 downto 0);
        variable v_t  : signed(9 downto 0);
        variable v_b  : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            t_m     <= f_mode(registers_in(8)(3 downto 0));
            t_v     <= f_vmode(registers_in(8)(3 downto 0));
            t_short <= f_isshort(registers_in(8)(3 downto 0));

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
                -- K4 Speed: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 8 then f_sneg <= '0'; v_u := unsigned(v_x) - 8; f_sm <= v_u(8 downto 0);
                elsif v_x < -8 then f_sneg <= '1'; v_u := unsigned(-v_x) - 8; f_sm <= v_u(8 downto 0);
                else f_sneg <= '0'; f_sm <= (others => '0'); end if;
                f_mir  <= registers_in(6)(0);
                f_blk  <= registers_in(6)(1);
                f_flip <= registers_in(6)(2);
                f_ice  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- the drops drift (+ = right), the breath LFO, the comb crawls, the Osc drift
                if f_sneg = '1' then f_drift <= f_drift - resize(f_dstep, 24);
                else f_drift <= f_drift + resize(f_dstep, 24); end if;
                f_lph <= f_lph + 91;
                f_comb0 <= f_comb0 + 7417;
                f_ta <= f_ta + 151; f_tb <= f_tb - 113;
            end if;

            -- glides: ease 1/8 of the gap per field (deadband 4); one channel per four zone clocks (4..27)
            if s_zone = '1' and s_zc >= 4 and s_zc < 28 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
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

            -- knob curves (settle long before the sequencer and the pixel path read them)
            v_g := s_g(0)(12 downto 3);                                             -- K1 Columns
            f_fcol <= to_unsigned(512, 11) + resize(shift_right(resize(v_g, 12) + shift_left(resize(v_g, 12), 1), 1), 11);
            v_g := s_g(1)(12 downto 3);                                             -- K2 Bend ~ K2 / 5
            f_bend <= resize(shift_right(v_g, 3), 8) + resize(shift_right(v_g, 4), 8)
                      + resize(shift_right(v_g, 7), 8) + resize(shift_right(v_g, 8), 8);
            v_g := s_g(2)(12 downto 3);                                             -- K3 Rows 0.6 .. 1.6
            f_rows <= to_unsigned(77, 8) + resize(shift_right(v_g, 3), 8);
            f_ramp <= s_g(3)(12 downto 6);                                          -- K5 Ripple, Q4 px
            f_melt <= s_g(4)(12 downto 5);                                          -- K6 Melt
            v_k := signed(resize(s_g(5)(12 downto 3), 11)) - 512;                  -- P12 Flow: +-416 lines
            v_13 := shift_left(resize(v_k, 15), 3) + shift_left(resize(v_k, 15), 2) + resize(v_k, 15);
            f_fk <= resize(shift_right(v_13, 4), 11);
            v_t := f_tri(f_lph);                                                    -- breath: +-20 lines
            v_b := shift_left(resize(v_t, 15), 4) + shift_left(resize(v_t, 15), 2);
            f_br <= resize(shift_right(v_b, 7), 7);
            f_flow <= f_fk + resize(f_br, 11);

            -- serial products: slot p = zc/32 (1..4), phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 6 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 6 and v_ph >= 1 and v_ph <= 18 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 6 loop
                if s_zone = '1' and v_p = k and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 1 => m_a <= to_unsigned(C_K0, 20);       m_b <= resize(f_fcol, 18);
                    when 2 => m_a <= resize(f_kv, 20);            m_b <= resize(t_m.vx, 18);
                    when 3 => m_a <= resize(f_kv, 20);            m_b <= resize(t_m.xm, 18);
                    when 4 => m_a <= resize(f_sm, 20);            m_b <= to_unsigned(277, 18);
                    when 5 => m_a <= resize(f_rows, 20);          m_b <= resize(f_bend, 18);
                    when others => m_a <= resize(f_kv, 20);       m_b <= resize(f_ramp, 18);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1) = '1' then f_kv <= v_pr(27 downto 10); end if;                     -- K0 * fcol / 1024
            if q_wr(2) = '1' then f_kph <= v_pr(26 downto 8); end if;                     -- per pixel
            if q_wr(3) = '1' then f_ph0 <= to_unsigned(C_PHC, 24) + f_drift - v_pr(23 downto 0); end if;
            if q_wr(4) = '1' then f_dstep <= v_pr(17 downto 0); end if;
            if q_wr(5) = '1' then f_gb <= v_pr(15 downto 7); end if;                        -- rows * bend (Q7)
            if q_wr(6) = '1' then f_ca <= v_pr(24 downto 13); end if;                       -- comb gain, ramp kv / 2^13

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
    -- Per-line engine: at the source avid rise of line n, compute line n+1:
    -- its flipped virtual line, the warp knot row and fraction, the comb's
    -- phase offset, and the 48 knot values t = ye + 8 (a + (b - a) fy) of the
    -- line into the bank line n+1 will read.  Line 0 is computed in the zone.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_ye : unsigned(18 downto 0);
        variable v_jf : unsigned(21 downto 0);
        variable v_w  : signed(17 downto 0);
        variable v_a  : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- kick
            if s_zone = '1' and s_zc = 300 then
                e_yv <= resize(t_v.vy(10 downto 1), 19);
                e_jf <= resize(t_v.jf0, 22);
                e_comb <= f_comb0;
                e_wb <= '0'; e_st <= to_unsigned(1, 5);
            elsif data_in.avid = '1' and s_avq = '0' then
                r_bank <= e_wb; e_wb <= not e_wb;
                l_ye <= d_ye; l_lrp <= d_lrp; l_af <= d_af;
                e_yv <= e_yv + resize(t_v.vy, 19);
                e_jf <= e_jf + resize(t_v.jst, 22);
                e_comb <= e_comb + t_v.comb;
                e_st <= to_unsigned(1, 5);
            elsif e_st /= 0 then
                if e_st = 29 then e_st <= (others => '0'); else e_st <= e_st + 1; end if;
            end if;

            -- setup; one serial shift-add multiplier: (ye - 80) rows (states 3..14), tri(comb) gain (16..27)
            case to_integer(e_st) is
                when 1 =>
                    if f_flip = '1' then
                        v_ye := to_unsigned(276480, 19) - e_yv;
                        v_jf := to_unsigned(2621440, 22) - e_jf;
                    else
                        v_ye := e_yv; v_jf := e_jf;
                    end if;
                    e_ye <= v_ye(18 downto 8);
                    if v_jf(21 downto 16) > 40 then e_j <= to_unsigned(40, 6); else e_j <= v_jf(21 downto 16); end if;
                    e_fy <= v_jf(15 downto 8);
                    e_ct <= f_tri(e_comb);
                when 2 =>
                    e_base <= shift_left(resize(e_j, 11), 5) + shift_left(resize(e_j, 11), 4);   -- j * 48
                    em_a <= resize(signed(resize(e_ye, 12)) - 80, 24);
                    em_b <= resize(f_rows, 12);
                    em_acc <= (others => '0');
                    e_iss <= '1'; e_i <= (others => '0'); e_r <= '0';
                when 3 to 14 | 16 to 27 =>
                    if em_b(0) = '1' then em_acc <= em_acc + em_a; end if;
                    em_a <= shift_left(em_a, 1);
                    em_b <= '0' & em_b(11 downto 1);
                when 15 =>
                    v_a := to_signed(80, 20) + resize(shift_right(em_acc, 7), 20) - resize(f_flow, 20);
                    e_af <= resize(v_a, 14);
                    em_a <= resize(e_ct, 24);
                    em_b <= f_ca;
                    em_acc <= (others => '0');
                when 28 =>
                    e_lrp <= resize(shift_right(em_acc, 10), 12);                               -- phase / 4096
                when 29 =>
                    d_ye <= e_ye; d_lrp <= e_lrp; d_af <= e_af;
                when others => null;
            end case;

            -- issue: 48 knots x 2 rows (a = row j, b = row j + 1)
            e_tv(0) <= e_iss; e_tr(0) <= e_r; e_ti(0) <= e_i;
            if e_iss = '1' then
                if e_r = '0' then r_wa <= e_base + resize(e_i, 11);
                else r_wa <= e_base + 48 + resize(e_i, 11); end if;
                if e_r = '1' then
                    if e_i = 47 then e_iss <= '0'; else e_i <= e_i + 1; end if;
                end if;
                e_r <= not e_r;
            end if;
            -- tags: 0 = address, 1 = ROM out, 2 = fabric copy
            for k in 1 to 3 loop
                e_tv(k) <= e_tv(k - 1); e_tr(k) <= e_tr(k - 1); e_ti(k) <= e_ti(k - 1);
            end loop;
            wr_q2 <= wr_q;
            e_kv <= '0';
            if e_tv(2) = '1' then
                if e_tr(2) = '0' then
                    e_ka <= signed(wr_q2);
                else
                    e_dab <= resize(signed(wr_q2), 9) - resize(e_ka, 9);
                    e_ka2 <= e_ka; e_kv <= '1'; e_ki <= e_ti(2);
                end if;
            end if;
            e_kv2 <= e_kv; e_ki2 <= e_ki;
            e_pr <= resize(e_dab * signed('0' & e_fy), 18) + shift_left(resize(e_ka2, 18), 8);
            -- t = ye + (a 256 + (b - a) fy) / 32 -> the line table
            lt_we_e <= '0'; lt_we_o <= '0';
            if e_kv2 = '1' then
                v_w := signed(resize(e_ye, 18)) + shift_right(e_pr, 5);
                lt_wd <= std_logic_vector(resize(v_w, 16));
                lt_wa <= "00" & e_wb & e_ki2(5 downto 1);
                if e_ki2(0) = '0' then lt_we_e <= '1'; else lt_we_o <= '1'; end if;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Beam: the warp cell coordinate (Mirror reverses it at the virtual centre)
    -- and the oscillator phase, run as a late copy of the beam (started by
    -- g_start delayed 13 clocks) so the phase needs no delay line.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                l_pb <= f_tb;
            elsif data_in.avid = '1' and s_avq = '0' then
                l_pb <= l_pb + 121;
            end if;
            if g_start = '1' then
                b_u <= resize(t_m.u0, 22);
                b_cnt <= (others => '0');
                b_dir <= '0';
                b_pa <= f_ta;
            elsif g_avid = '1' then
                b_cnt <= b_cnt + 1;
                b_pa <= b_pa + 89;
                if f_mir = '1' and b_cnt = t_m.cmid - 1 then
                    b_dir <= '1';                                    -- hold: the centre pixel repeats
                elsif b_dir = '0' then
                    b_u <= b_u + resize(t_m.ku, 22);
                else
                    b_u <= b_u - resize(t_m.ku, 22);
                end if;
            end if;
            gs_sr(0) <= g_start;
            for k in 1 to 12 loop gs_sr(k) <= gs_sr(k - 1); end loop;
            if gs_sr(12) = '1' then
                lp_ph <= f_ph0;
                lp_cnt <= (others => '0');
                lp_dir <= '0';
            elsif s_gav_sr(12) = '1' then
                lp_cnt <= lp_cnt + 1;
                if f_mir = '1' and lp_cnt = t_m.cmid - 1 then
                    lp_dir <= '1';
                elsif lp_dir = '0' then
                    lp_ph <= lp_ph + resize(f_kph, 24);
                else
                    lp_ph <= lp_ph - resize(f_kph, 24);
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Rings (block RAM): the chroma box window {U, V} and the sync delay
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rb_ram(to_integer(s_wp)) <= rb_wd;
            rb_q <= rb_ram(to_integer(rb_ra));
            rb_q2 <= rb_q;
        end if;
    end process p_ring;

    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            rs_ram(to_integer(s_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            rs_q <= rs_ram(to_integer(rs_ra));
            rs_q2 <= rs_q;
        end if;
    end process p_sring;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ci  : unsigned(5 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_t3  : signed(15 downto 0);
        variable v_tm  : signed(15 downto 0);
        variable v_q   : unsigned(11 downto 0);
        variable v_jc  : unsigned(7 downto 0);
        variable v_iy0, v_iy1, v_iye, v_iyo : unsigned(4 downto 0);
        variable v_ic0, v_ic1, v_ice, v_ico : unsigned(3 downto 0);
        variable v_jye, v_jyo : unsigned(7 downto 0);
        variable v_ya, v_yb, v_yc, v_yd : signed(9 downto 0);
        variable v_c00, v_c10, v_c01, v_c11 : signed(9 downto 0);
        variable v_ca, v_cb : std_logic_vector(15 downto 0);
        variable v_o   : signed(10 downto 0);
        variable v_os  : signed(10 downto 0);
        variable v_iu, v_iv, v_iy : unsigned(9 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: 2-px luma average -> signed (Y - 512) / 2 (a few px early: invisible in a bend)
            s_vy(1) <= unsigned(data_in.y(9 downto 2)); s_vy(2) <= s_vy(1);
            s_vs <= signed(resize(resize(s_vy(1), 9) + resize(s_vy(2), 9), 10)) - 256;

            -- 1: warp cell, fraction, line-table addresses (knot i in bank i & 1)
            if b_u(21 downto 16) > 46 then v_ci := to_unsigned(46, 6); q1_fx <= (others => '1');
            else v_ci := b_u(21 downto 16); q1_fx <= b_u(15 downto 8); end if;
            q1_odd <= v_ci(0);
            lt_rao <= "00" & r_bank & v_ci(5 downto 1);
            lt_rae <= "00" & r_bank & resize(shift_right(v_ci + 1, 1), 5);
            -- 2: line table (block RAM)
            q2_fx <= q1_fx; q2_odd <= q1_odd;
            -- 3: fabric copies
            q3_e <= signed(lt_qe(12 downto 0)); q3_o <= signed(lt_qo(12 downto 0));
            q3_fx <= q2_fx; q3_odd <= q2_odd;
            -- 4: knots i, i + 1
            if q3_odd = '1' then q4_w0 <= q3_o; q4_w1 <= q3_e;
            else q4_w0 <= q3_e; q4_w1 <= q3_o; end if;
            q4_fx <= q3_fx;
            -- 5
            q5_dw <= resize(q4_w1, 14) - resize(q4_w0, 14);
            q5_w0 <= q4_w0; q5_fx <= q4_fx;
            -- 6: (w1 - w0) fx, split
            q6_ph <= resize(q5_dw * signed('0' & q5_fx(7 downto 4)), 18);
            q6_pl <= resize(q5_dw * signed('0' & q5_fx(3 downto 0)), 18);
            q6_w0 <= q5_w0;
            -- 7
            q7_pr <= shift_left(resize(q6_ph, 22), 4) + resize(q6_pl, 22);
            q7_w0 <= q6_w0;
            -- 8: t (the warp's row coordinate); the source (video luma or Osc)
            v_t := resize(q7_w0, 14) + resize(shift_right(q7_pr, 8), 14);
            q8_t <= v_t;
            if f_osc = '1' then
                q8_src <= shift_right(f_tri(b_pa) + f_tri(l_pb), 2);   -- a quarter of the video's swing
            else
                q8_src <= s_vs;
            end if;
            -- 9: d = t - ye; the lava flag (the comb rides on it); melt products
            q9_d <= resize(q8_t, 15) - signed(resize(l_ye, 15));
            if q8_t < 1060 then p_lf(9) <= '1'; else p_lf(9) <= '0'; end if;
            q9_ms <= resize(q8_src * signed('0' & f_melt(7 downto 3)), 16);
            -- 10: d * (rows bend), split 5 | 4
            q10_gh <= resize(q9_d * signed('0' & f_gb(8 downto 4)), 20);
            q10_gl <= resize(q9_d * signed('0' & f_gb(3 downto 0)), 20);
            q10_ms <= resize(q9_ms, 19);
            -- 11
            q11_dg <= shift_left(resize(q10_gh, 25), 4) + resize(q10_gl, 25);
            q11_ms <= q10_ms;
            -- 12: t3 = 80 + rows (ye - 80) - flow + rows bend d + melt src
            v_t3 := resize(l_af, 16) + resize(shift_right(q11_dg, 7), 16) + resize(shift_right(q11_ms, 5), 16);
            q12_t3 <= v_t3;
            -- 13: clamp to the tile; the background flag
            v_tm := q12_t3 - 80;
            if v_tm < 0 then q13_tm <= (others => '0');
            elsif v_tm > 1034 then q13_tm <= to_unsigned(1034, 11);
            else q13_tm <= unsigned(v_tm(10 downto 0)); end if;
            if q12_t3 > 1076 then p_bg(13) <= '1'; else p_bg(13) <= '0'; end if;
            -- 14: tile rows (Y: 40 / 256 knots per line; C: 20 / 256, rounded); phase (+ comb on the lava)
            q14_vy <= shift_left(resize(q13_tm, 16), 5) + shift_left(resize(q13_tm, 16), 3);
            q14_vc <= shift_left(resize(q13_tm, 16), 4) + shift_left(resize(q13_tm, 16), 2) + 128;
            if p_lf(13) = '1' then q14_q <= lp_ph(23 downto 12) + unsigned(l_lrp);
            else q14_q <= lp_ph(23 downto 12); end if;
            -- 15: phase * 24, * 12
            v_q := q14_q;
            q15_uy <= shift_left(resize(v_q, 17), 4) + shift_left(resize(v_q, 17), 3);
            q15_uc <= shift_left(resize(v_q, 17), 3) + shift_left(resize(v_q, 17), 2);
            q15_jy <= q14_vy(15 downto 8); q15_fvy <= q14_vy(7 downto 4);
            q15_jc <= q14_vc(15 downto 8);
            -- 16: knot indices
            v_iy0 := q15_uy(16 downto 12);
            if v_iy0 = 23 then v_iy1 := (others => '0'); else v_iy1 := v_iy0 + 1; end if;
            v_ic0 := q15_uc(15 downto 12);
            if v_ic0 = 11 then v_ic1 := (others => '0'); else v_ic1 := v_ic0 + 1; end if;
            q16_iy0 <= v_iy0; q16_iy1 <= v_iy1; q16_ic0 <= v_ic0; q16_ic1 <= v_ic1;
            q16_jy0 <= q15_jy; q16_jy1 <= q15_jy + 1;
            if q15_jc > 81 then v_jc := to_unsigned(81, 8); else v_jc := q15_jc; end if;
            q16_jc <= v_jc(6 downto 0);
            p_fuy(16) <= q15_uy(11 downto 8); p_fuc(16) <= q15_uc(11 downto 8); p_fvy(16) <= q15_fvy;
            p_py(16) <= v_iy0(0) & q15_jy(0);
            p_pc(16) <= v_ic0(0);
            -- 17: bank addresses (Y: bank (i & 1) + 2 (j & 1), (j >> 1) * 12 + (i >> 1); C: bank i & 1, j * 6 + (i >> 1))
            if q16_iy0(0) = '0' then v_iye := q16_iy0; v_iyo := q16_iy1; else v_iye := q16_iy1; v_iyo := q16_iy0; end if;
            if q16_jy0(0) = '0' then v_jye := q16_jy0; v_jyo := q16_jy1; else v_jye := q16_jy1; v_jyo := q16_jy0; end if;
            if q16_ic0(0) = '0' then v_ice := q16_ic0; v_ico := q16_ic1; else v_ice := q16_ic1; v_ico := q16_ic0; end if;
            ry_a0 <= shift_left(resize(v_jye(7 downto 1), 10), 3) + shift_left(resize(v_jye(7 downto 1), 10), 2) + resize(v_iye(4 downto 1), 10);
            ry_a1 <= shift_left(resize(v_jye(7 downto 1), 10), 3) + shift_left(resize(v_jye(7 downto 1), 10), 2) + resize(v_iyo(4 downto 1), 10);
            ry_a2 <= shift_left(resize(v_jyo(7 downto 1), 10), 3) + shift_left(resize(v_jyo(7 downto 1), 10), 2) + resize(v_iye(4 downto 1), 10);
            ry_a3 <= shift_left(resize(v_jyo(7 downto 1), 10), 3) + shift_left(resize(v_jyo(7 downto 1), 10), 2) + resize(v_iyo(4 downto 1), 10);
            rc_a0 <= shift_left(resize(q16_jc, 9), 2) + shift_left(resize(q16_jc, 9), 1) + resize(v_ice(3 downto 1), 9);
            rc_a1 <= shift_left(resize(q16_jc, 9), 2) + shift_left(resize(q16_jc, 9), 1) + resize(v_ico(3 downto 1), 9);
            -- 18: tile ROMs (block RAM); 19: fabric copies
            q19_y0 <= ry_q0; q19_y1 <= ry_q1; q19_y2 <= ry_q2; q19_y3 <= ry_q3;
            q19_c0 <= rc_q0; q19_c1 <= rc_q1;
            -- 20: corners in (i, j) order, differences along the phase
            v_ya := signed(resize(unsigned(q19_y0), 10)); v_yb := signed(resize(unsigned(q19_y1), 10));
            v_yc := signed(resize(unsigned(q19_y2), 10)); v_yd := signed(resize(unsigned(q19_y3), 10));
            case p_py(19) is   -- (i0 odd, j0 odd)
                when "00" => v_c00 := v_ya; v_c10 := v_yb; v_c01 := v_yc; v_c11 := v_yd;
                when "10" => v_c00 := v_yb; v_c10 := v_ya; v_c01 := v_yd; v_c11 := v_yc;
                when "01" => v_c00 := v_yc; v_c10 := v_yd; v_c01 := v_ya; v_c11 := v_yb;
                when others => v_c00 := v_yd; v_c10 := v_yc; v_c01 := v_yb; v_c11 := v_ya;
            end case;
            q20_c00 <= v_c00; q20_c01 <= v_c01;
            q20_d0 <= v_c10 - v_c00; q20_d1 <= v_c11 - v_c01;
            if p_pc(19) = '0' then v_ca := q19_c0; v_cb := q19_c1; else v_ca := q19_c1; v_cb := q19_c0; end if;
            q20_r0 <= resize(signed(v_ca(15 downto 8)), 9); q20_b0 <= resize(signed(v_ca(7 downto 0)), 9);
            q20_rd <= resize(signed(v_cb(15 downto 8)), 10) - resize(signed(v_ca(15 downto 8)), 10);
            q20_bd <= resize(signed(v_cb(7 downto 0)), 10) - resize(signed(v_ca(7 downto 0)), 10);
            -- 21: products along the phase
            q21_p0 <= resize(q20_d0 * signed('0' & p_fuy(20)), 14);
            q21_p1 <= resize(q20_d1 * signed('0' & p_fuy(20)), 14);
            q21_pr <= resize(q20_rd * signed('0' & p_fuc(20)), 14);
            q21_pb <= resize(q20_bd * signed('0' & p_fuc(20)), 14);
            q21_c00 <= q20_c00; q21_c01 <= q20_c01; q21_r0 <= q20_r0; q21_b0 <= q20_b0;
            -- 22
            q22_a <= q21_c00 + resize(shift_right(q21_p0, 4), 10);
            q22_b <= q21_c01 + resize(shift_right(q21_p1, 4), 10);
            q22_cr <= resize(q21_r0, 10) + resize(shift_right(q21_pr, 4), 10);
            q22_cb <= resize(q21_b0, 10) + resize(shift_right(q21_pb, 4), 10);
            -- 23
            q23_e <= q22_b - q22_a; q23_a <= q22_a; q23_cr <= q22_cr; q23_cb <= q22_cb;
            -- 24: product along t
            q24_pe <= resize(q23_e * signed('0' & p_fvy(23)), 14);
            q24_a <= q23_a; q24_cr <= q23_cr; q24_cb <= q23_cb;
            -- 25
            q25_y <= q24_a + resize(shift_right(q24_pe, 4), 10);
            q25_cr <= q24_cr; q25_cb <= q24_cb;
            -- 26: compose (Y = 64 + 4 y; U = Cr, V = Cb at 512 + 4 c); Ice; Ground = Black
            v_o := shift_left(resize(q25_cr, 11), 2);
            v_os := to_signed(512, 11) + v_o;
            v_iu := unsigned(v_os(9 downto 0));
            v_o := shift_left(resize(q25_cb, 11), 2);
            v_os := to_signed(512, 11) + v_o;
            v_iv := unsigned(v_os(9 downto 0));
            if f_blk = '1' and p_bg(25) = '1' then
                q26_y <= to_unsigned(64, 10); q26_u <= to_unsigned(512, 10); q26_v <= to_unsigned(512, 10);
            else
                if q25_y < 0 then q26_y <= to_unsigned(64, 10);
                else q26_y <= to_unsigned(64, 10) + shift_left(resize(unsigned(q25_y(7 downto 0)), 10), 2); end if;
                if f_ice = '1' then q26_u <= v_iv; q26_v <= v_iu;
                else q26_u <= v_iu; q26_v <= v_iv; end if;
            end if;
            -- 27, 28
            q27_y <= q26_y; q27_u <= q26_u; q27_v <= q26_v;
            q28_y <= q27_y; q28_u <= q27_u; q28_v <= q27_v;

            -- delay lines
            for k in 10 to 13 loop p_lf(k) <= p_lf(k - 1); end loop;
            for k in 14 to 27 loop p_bg(k) <= p_bg(k - 1); end loop;
            for k in 17 to 20 loop p_fuy(k) <= p_fuy(k - 1); p_fuc(k) <= p_fuc(k - 1); end loop;
            for k in 17 to 23 loop p_fvy(k) <= p_fvy(k - 1); end loop;
            for k in 17 to 19 loop p_py(k) <= p_py(k - 1); p_pc(k) <= p_pc(k - 1); end loop;

            -- 29..: chroma box sums over the ring window: 8 taps at 1080, 4 at 720p / SD (box pair =
            -- [1 2 2 2 2 2 2 2 1] / 16).  The ring's input is forced neutral in the vblank zone and outside
            -- the generated avid; luma rides a register chain aligned with the chroma window's centre.
            s_wp <= s_wp + 1;
            if t_short = '1' then rb_ra <= s_wp - 1; else rb_ra <= s_wp - 5; end if;
            rs_ra <= s_wp - (C_LAT - 3);
            if s_zone = '1' or s_gav_sr(27) = '0' then
                v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10); v_iy := to_unsigned(64, 10);
            else
                v_iu := q28_u; v_iv := q28_v; v_iy := q28_y;
            end if;
            rb_wd <= std_logic_vector(v_iu) & std_logic_vector(v_iv);
            s_ly(0) <= v_iy;
            for k in 1 to 6 loop s_ly(k) <= s_ly(k - 1); end loop;
            if s_zone = '1' and s_zc = 40 then
                if t_short = '1' then s_usum <= to_unsigned(2048, 14); s_vsum <= to_unsigned(2048, 14);
                else s_usum <= to_unsigned(4096, 14); s_vsum <= to_unsigned(4096, 14); end if;
            else
                s_usum <= s_usum + unsigned(rb_wd(19 downto 10)) - unsigned(rb_q2(19 downto 10));
                s_vsum <= s_vsum + unsigned(rb_wd(9 downto 0)) - unsigned(rb_q2(9 downto 0));
            end if;
            -- 31: chroma box pair; luma [1 2 1] centred on the same pixel
            s_ud <= s_usum; s_vd <= s_vsum;
            v_sc := resize(s_usum, 15) + s_ud;
            if t_short = '1' then q31_u <= v_sc(12 downto 3); else q31_u <= v_sc(13 downto 4); end if;
            v_sc := resize(s_vsum, 15) + s_vd;
            if t_short = '1' then q31_v <= v_sc(12 downto 3); else q31_v <= v_sc(13 downto 4); end if;
            if t_short = '1' then
                v_sy := resize(s_ly(2), 12) + shift_left(resize(s_ly(3), 12), 1) + resize(s_ly(4), 12);
            else
                v_sy := resize(s_ly(4), 12) + shift_left(resize(s_ly(5), 12), 1) + resize(s_ly(6), 12);
            end if;
            q31_y <= v_sy(11 downto 2);
            -- 32, 33
            q32_y <= q31_y; q32_u <= q31_u; q32_v <= q31_v;
            q33_y <= q32_y; q33_u <= q32_u; q33_v <= q32_v;
            -- 34: luma toward black (white 940 -> 777); blanking gate
            if q33_y > 64 then v_dl := q33_y - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(33) = '1' then
                q34_y <= q33_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                q34_u <= q33_u; q34_v <= q33_v;
            else
                q34_y <= to_unsigned(64, 10); q34_u <= to_unsigned(512, 10); q34_v <= to_unsigned(512, 10);
            end if;
            -- 35, 36
            q35_y <= q34_y; q35_u <= q34_u; q35_v <= q34_v;
            q36_y <= q35_y; q36_u <= q35_u; q36_v <= q35_v;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(q36_y);
    data_out.u       <= std_logic_vector(q36_u);
    data_out.v       <= std_logic_vector(q36_v);
    data_out.hsync_n <= rs_q2(2);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lava;
