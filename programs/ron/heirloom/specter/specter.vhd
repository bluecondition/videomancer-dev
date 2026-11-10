-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Specter v0.1.0 -- Heirloom (video clips), from the clip WIN_20230904_01_56_58 at 0.5 s.
--
-- The still: a green ghost figure (a striped head, outstretched arms, legs streaming down to the floor) in a green
-- haze of hanging curtains, a dark band under the haze, and a blue floor at the bottom corners.  A camera (Ron, in
-- front of it) through a heavily smeared luma key, colourised green, with a second, inverted key in blue at the
-- bottom; a fixed slanted 39 px comb from an unlocked horizontal oscillator; torn bands at the ghost's neck.
--
-- The patch, as rebuilt (specter.py holds the fitted maps):
--   haze  = bilinear texture map (64 x 32 cells of 32 px), rolling down and drifting left, keyed off at the bottom
--           by a ramp whose threshold follows the texture (brighter curtains hang lower, pointed tips)
--   ghost = bilinear figure map (32 x 32 cells of 19 x 22 px), bobbing; its last row continues down as the legs
--           (fading); times a screen-locked slanted comb; the camera (or the Osc) bends both maps and glows through
--   floor = a blue ramp below the haze with triangle flame tips (233 and 93 px), cut by the ghost's aura and the ghost
--   green / blue palettes summed in YUV
--
--   K1  Ghost     the figure's size, 0.55x .. 1x (512, the still) .. 1.9x
--   K2  Comb      the ghost's vertical stripes: wide deep bars (0) .. the still's faint 39 px comb (512) .. fine deep bars
--   K3  Haze      the curtains' level: none (0) .. the still (512) .. 2.2x, hanging lower
--   K4  Drift     bipolar motion speed (bob, roll, drift, flames); 768 = the clip, centre +-8 = still
--   K5  Drip      the legs: gone (0) .. the still (512) .. full length while the curtains drip far down (1023)
--   K6  Etch      how far the camera (or the Osc) bends the haze and the ghost and glows through them
--   S7  Twin      one ghost / a mirrored pair
--   S8  Tear      Calm (torn bands at the neck, the still) / Torn (bands slip across the whole frame)
--   S9  Ghost     Glow (the still) / Shadow (the haze floods in, the ghost a black cut-out)
--   S10 Palette   Spectre (green / blue, the still) / Ember (amber / violet)
--   S11 Source    Video / Osc
--   P12 TIDE      the haze's bottom and the blue floor together: haze to the bottom (0) .. the still (512) .. a
--                 blue sea up to the middle with tall flames, the ghost's legs cutting through it (100 %)
--
-- Engine: vblank-zone sequencer (16 slots on a 32-clock grid, a shared serial multiplier, shared glide, a sine ROM), a per-line engine kicked at each line start that computes the NEXT line (row weights, the figure's phase,
-- the comb, key and floor terms) on the same multiplier and builds both maps' interpolated rows into ping-pong
-- profile RAMs, a 17-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid;
-- output conditioning (U and V through a [1 2 2 2 2 2 2 2 1]/16 box, Y through [1 2 1]) before the blanking gate;
-- the luma 13/16 bar fix is folded into the palette (drawn whites stay below ~600).  Hardware colours (U = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.specter_pkg.all;
use work.all;

architecture specter of program_top is
    constant C_LAT : integer := 16;   -- = processing_delay_clks

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
    -- signed triangle of a 16-bit phase: +127 at 0, -128 at one half
    function f_tri8(ph : unsigned(15 downto 0)) return signed is
        variable v : unsigned(8 downto 0);
    begin
        v := ph(15 downto 7);
        if v(8) = '1' then
            return signed(resize(v(7 downto 0), 9)) - 128;
        else
            return 127 - signed(resize(v(7 downto 0), 9));
        end if;
    end function;
    -- v * xs (xs = 2, 3, 4), mod 2^16
    function f_mxs(v : unsigned(15 downto 0); xs : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(xs) is
            when 4      => return v(13 downto 0) & "00";
            when 3      => return (v(14 downto 0) & '0') + v;
            when others => return v(14 downto 0) & '0';
        end case;
    end function;

    -- camera line buffer (half resolution): (luma >> 2) - 16, or the Osc lattice
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- profile RAMs (ping-pong banks of interpolated map rows, pairs {v(c), clip(v(c+1) - v(c))})
    type t_pb is array (0 to 127) of std_logic_vector(15 downto 0);
    type t_pf is array (0 to 63) of std_logic_vector(15 downto 0);
    signal pbr : t_pb := (others => x"0000");
    signal pfr : t_pf := (others => x"0000");
    -- chroma box ring {U8, V8} (8-sample delay: read 6 behind the write, plus a fabric copy) and sync ring
    -- {hsync, vsync, field}
    type t_cr is array (0 to 255) of std_logic_vector(15 downto 0);
    signal crr : t_cr := (others => x"8080");
    type t_sy is array (0 to 255) of std_logic_vector(3 downto 0);
    signal syr : t_sy := (others => x"F");
    attribute no_rw_check : boolean;
    attribute no_rw_check of pbr : signal is true;
    attribute no_rw_check of pfr : signal is true;
    attribute no_rw_check of crr : signal is true;
    attribute no_rw_check of syr : signal is true;
    signal pb_we, pf_we : std_logic := '0';
    signal pb_wa, pb_ra : unsigned(6 downto 0) := (others => '0');
    signal pf_wa, pf_ra : unsigned(5 downto 0) := (others => '0');
    signal pb_wd, pf_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal pb_q, pf_q : std_logic_vector(15 downto 0) := (others => '0');
    signal cr_wp : unsigned(7 downto 0) := to_unsigned(6, 8);
    signal cr_rp : unsigned(7 downto 0) := (others => '0');
    signal cr_wd : std_logic_vector(15 downto 0) := x"8080";
    signal cr_q, cr_qq : std_logic_vector(15 downto 0) := x"8080";
    signal cr_we : std_logic := '0';
    signal sy_wp : unsigned(7 downto 0) := to_unsigned(C_LAT - 1, 8);
    signal sy_rp : unsigned(7 downto 0) := (others => '0');
    signal sy_q  : std_logic_vector(3 downto 0) := x"F";
    signal sy_we : std_logic := '0';

    -- ROMs
    signal r_bg_a : unsigned(10 downto 0) := (others => '0');
    signal r_fg_a : unsigned(9 downto 0) := (others => '0');
    signal r_bg_q, r_fg_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_pg_a : unsigned(8 downto 0) := (others => '0');
    signal r_pb_a : unsigned(7 downto 0) := (others => '0');
    signal r_pg_q, r_pb_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_sn_a : unsigned(7 downto 0) := (others => '0');
    signal r_sn_q : std_logic_vector(7 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_torn, s_shadow, s_pal, s_src : std_logic := '0';

    -- shared serial multiplier (the vblank sequencer, then the line engine)
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(15 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(5 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field scratch and constants
    signal z_t    : unsigned(17 downto 0) := (others => '0');
    signal z_cs   : unsigned(11 downto 0) := (others => '0');
    signal z_y7   : unsigned(11 downto 0) := to_unsigned(56, 12);
    signal z_sn1  : signed(7 downto 0) := (others => '0');
    signal z_d    : unsigned(9 downto 0) := (others => '0');
    signal z_dn   : std_logic := '0';
    signal c_spd  : unsigned(10 downto 0) := (others => '0');
    signal c_pb   : unsigned(15 downto 0) := (others => '0');
    signal c_roll : unsigned(17 downto 0) := (others => '0');
    signal c_drift : unsigned(19 downto 0) := (others => '0');
    signal c_f1   : unsigned(15 downto 0) := (others => '0');
    signal c_bob  : signed(8 downto 0) := (others => '0');
    signal c_fcy  : unsigned(10 downto 0) := to_unsigned(600, 11);
    signal c_inv  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_ku   : unsigned(11 downto 0) := to_unsigned(1724, 12);
    signal c_cstep : unsigned(15 downto 0) := to_unsigned(840, 16);
    signal c_ccl  : unsigned(15 downto 0) := (others => '0');
    signal c_cd   : unsigned(7 downto 0) := to_unsigned(28, 8);
    signal c_hz   : unsigned(8 downto 0) := to_unsigned(128, 9);
    signal c_yb   : unsigned(10 downto 0) := to_unsigned(815, 11);
    signal c_yf   : unsigned(10 downto 0) := to_unsigned(1010, 11);
    signal c_legr : unsigned(9 downto 0) := to_unsigned(69, 10);
    signal c_legs0 : unsigned(7 downto 0) := to_unsigned(245, 8);
    signal c_k13  : unsigned(3 downto 0) := to_unsigned(13, 4);
    signal c_eb   : unsigned(7 downto 0) := (others => '0');
    signal c_b140 : unsigned(10 downto 0) := (others => '0');
    signal c_xoff : unsigned(11 downto 0) := (others => '0');
    signal c_tseed : unsigned(15 downto 0) := x"5A5A";
    signal c_fcnt : unsigned(1 downto 0) := (others => '0');
    signal c_flfsr : unsigned(15 downto 0) := x"ACE1";
    -- per-mode constants (combinational)
    signal w_xb0  : unsigned(11 downto 0);
    signal w_cpx, w_fs1 : unsigned(15 downto 0);
    signal w_ky, w_ku, w_kv : unsigned(9 downto 0);

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_first : std_logic := '1';
    signal e_bank : std_logic := '0';
    signal m_go   : std_logic := '0';
    signal m_opa, m_opb : unsigned(15 downto 0) := (others => '0');
    signal e_dya  : unsigned(11 downto 0) := (others => '0');
    signal e_dyn  : std_logic := '0';
    signal e_t1, e_t2, e_t3 : signed(12 downto 0) := (others => '0');
    signal e_bandp : unsigned(7 downto 0) := (others => '1');
    signal e_lfsr : unsigned(15 downto 0) := x"5A5A";
    signal e_step : std_logic := '0';
    signal e_tear : signed(9 downto 0) := (others => '0');
    signal e_dx   : signed(13 downto 0) := (others => '0');
    signal e_yr   : signed(12 downto 0) := (others => '0');
    signal e_vf   : signed(13 downto 0) := (others => '0');
    signal e_dd   : unsigned(11 downto 0) := (others => '0');
    signal e_lf   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal e_ccl  : unsigned(15 downto 0) := (others => '0');
    signal e_wt   : signed(15 downto 0) := (others => '0');
    -- next-line values (loaded into the pixel path at g_start)
    signal n_rb, n_fyb : unsigned(4 downto 0) := (others => '0');
    signal n_rf0, n_rf1, n_fyf : unsigned(4 downto 0) := (others => '0');
    signal n_wt   : unsigned(7 downto 0) := (others => '0');
    signal n_a13  : signed(12 downto 0) := (others => '0');
    signal n_by   : signed(9 downto 0) := (others => '0');
    signal n_w0b, n_w1b : unsigned(8 downto 0) := (others => '0');
    signal n_w0f, n_w1f : unsigned(8 downto 0) := (others => '0');
    signal n_x0   : signed(12 downto 0) := (others => '0');
    signal n_u0   : signed(25 downto 0) := (others => '0');
    signal n_du   : unsigned(15 downto 0) := (others => '0');
    signal n_dcl  : unsigned(4 downto 0) := (others => '0');
    signal n_cp0  : unsigned(15 downto 0) := (others => '0');
    -- profile build
    signal k_run  : std_logic := '0';
    signal k_pc   : unsigned(7 downto 0) := (others => '0');
    signal k_ctl1, k_ctl2, k_ctl3, k_ctl4, k_ctl5 : std_logic_vector(8 downto 0) := (others => '0');   -- {valid, fig, odd, col(5:0)}
    signal k_q2   : unsigned(7 downto 0) := (others => '0');
    signal k_w2   : unsigned(8 downto 0) := (others => '0');
    signal k_pa, k_pb : unsigned(12 downto 0) := (others => '0');
    signal k_p4   : unsigned(16 downto 0) := (others => '0');
    signal k_acc  : unsigned(16 downto 0) := (others => '0');
    signal k_vp, k_v0 : unsigned(7 downto 0) := (others => '0');
    signal k_last : std_logic := '0';
    signal k_lastv : unsigned(7 downto 0) := (others => '0');
    signal k_lastf : std_logic := '0';

    -- beam (stage 1) and line constants
    signal b_xh, b_xn : signed(12 downto 0) := (others => '0');
    signal b_u    : signed(25 downto 0) := (others => '0');
    signal b_dec  : std_logic := '0';
    signal b_cp   : unsigned(15 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal l_bank : std_logic := '0';
    signal l_du   : unsigned(15 downto 0) := (others => '0');
    signal l_wt   : unsigned(7 downto 0) := (others => '0');
    signal l_a13  : signed(12 downto 0) := (others => '0');
    signal l_by   : signed(9 downto 0) := (others => '0');
    signal l_dcl  : unsigned(4 downto 0) := (others => '0');

    -- pixel path (the camera is read 5 pixels ahead, so its bend meets the beam at stage 1)
    signal p4_lm  : unsigned(7 downto 0) := (others => '0');
    signal p5_bl, p5_bh : unsigned(11 downto 0) := (others => '0');
    signal p6_bnd : unsigned(10 downto 0) := (others => '0');
    signal p2_fxb, p3_fxb, p4_fxb : unsigned(5 downto 0) := (others => '0');
    signal p2_fxf, p3_fxf, p4_fxf : unsigned(5 downto 0) := (others => '0');
    signal p2_ok, p3_ok : std_logic := '0';
    signal p2_tc  : signed(8 downto 0) := (others => '0');
    signal p2_d   : signed(15 downto 0) := (others => '0');
    signal p2_g1  : unsigned(10 downto 0) := (others => '0');
    signal p3_au  : unsigned(11 downto 0) := (others => '0');
    signal p3_dc  : signed(14 downto 0) := (others => '0');
    signal p4_av  : unsigned(9 downto 0) := (others => '0');
    signal p4_b0, p4_f0, p5_b0, p5_f0 : unsigned(7 downto 0) := (others => '0');
    signal p4_db, p4_df : signed(7 downto 0) := (others => '0');
    signal p5_bp, p5_fp : signed(14 downto 0) := (others => '0');
    signal p6_tv, p7_tv, p8_tv : unsigned(7 downto 0) := (others => '0');
    signal p6_f   : unsigned(7 downto 0) := (others => '0');
    signal p7_idx : unsigned(5 downto 0) := (others => '0');
    signal p7_fm  : unsigned(17 downto 0) := (others => '0');
    signal p8_w, p8_s, p9_bg, p9_s : unsigned(7 downto 0) := (others => '0');
    signal p9_t1, p9_t2 : signed(8 downto 0) := (others => '0');
    signal p10_g  : unsigned(7 downto 0) := (others => '0');
    signal p10_ft : signed(9 downto 0) := (others => '0');
    signal p10_b1 : signed(11 downto 0) := (others => '0');
    signal lf1    : unsigned(15 downto 0) := (others => '0');
    type t_cm is array (4 to 6) of unsigned(9 downto 0);
    signal p_cm  : t_cm := (others => (others => '0'));
    type t_gl is array (3 to 7) of unsigned(8 downto 0);
    signal p_glw : t_gl := (others => (others => '0'));
    type t_au is array (5 to 9) of unsigned(7 downto 0);
    signal p_aura : t_au := (others => (others => '0'));
    signal p13_g, p13_b : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_yd : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_pbr : process(clk)
    begin
        if rising_edge(clk) then
            pb_q <= pbr(to_integer(pb_ra));
            if pb_we = '1' then pbr(to_integer(pb_wa)) <= pb_wd; end if;
        end if;
    end process p_pbr;

    p_pfr : process(clk)
    begin
        if rising_edge(clk) then
            pf_q <= pfr(to_integer(pf_ra));
            if pf_we = '1' then pfr(to_integer(pf_wa)) <= pf_wd; end if;
        end if;
    end process p_pfr;

    p_crr : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= crr(to_integer(cr_rp));
            if cr_we = '1' then crr(to_integer(cr_wp)) <= cr_wd; end if;
        end if;
    end process p_crr;

    -- syncs: delayed by exactly C_LAT clocks through a block-RAM ring (read 15 behind the write)
    p_syr : process(clk)
    begin
        if rising_edge(clk) then
            sy_q <= syr(to_integer(sy_rp));
            if sy_we = '1' then syr(to_integer(sy_wp)) <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n; end if;
            sy_wp <= sy_wp + 1; sy_rp <= sy_rp + 1; sy_we <= '1';
        end if;
    end process p_syr;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_bg_q <= C_SP_BG(to_integer(r_bg_a));
            r_fg_q <= C_SP_FG(to_integer(r_fg_a));
            r_pg_q <= C_SP_PALG(to_integer(r_pg_a));
            r_pb_q <= C_SP_PALB(to_integer(r_pb_a));
            r_sn_q <= C_SP_SIN(to_integer(r_sn_a));
        end if;
    end process p_rom;

    -- per-mode constants: beam origin (8 pixels ahead of the output filter), flame steps, comb step, palette black
    w_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
    w_cpx <= f_mxs(c_cstep, s_xs);
    w_fs1 <= f_mxs(to_unsigned(141, 16), s_xs);
    w_ky <= unsigned(C_SP_PALG(256)(31 downto 22)) when s_pal = '1' else unsigned(C_SP_PALG(0)(31 downto 22));
    w_ku <= unsigned(C_SP_PALG(256)(21 downto 12)) when s_pal = '1' else unsigned(C_SP_PALG(0)(21 downto 12));
    w_kv <= unsigned(C_SP_PALG(256)(11 downto 2)) when s_pal = '1' else unsigned(C_SP_PALG(0)(11 downto 2));

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock).  Slot k of
    -- the serial multiplier: operands at zc = 32 k + 31, load at 32 k + 32,
    -- product stored at 32 k + 50 (zc(4..0) = 18, zc(10..5) = k + 1).
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_x  : signed(11 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_st : boolean;
        variable v_k  : unsigned(5 downto 0);
        variable v_e  : unsigned(10 downto 0);
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

            s_avq <= data_in.avid;
            -- speed: |k4 - 512| - 8, +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_twin   <= registers_in(6)(0);
                s_torn   <= registers_in(6)(1);
                s_shadow <= registers_in(6)(2);
                s_pal    <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            -- the generated line's last pixel: 1920 / 1280 / 720 active pixels (1080, 720p, SD)
            case to_integer(s_xs) is
                when 4      => g_wm1 <= to_unsigned(719, 12);
                when 3      => g_wm1 <= to_unsigned(1279, 12);
                when others => g_wm1 <= to_unsigned(1919, 12);
            end case;
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

            -- slot decode (zc = 32 k + 30, k = 0 .. 18) and operand select (registered a clock before the load)
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = "11110" and s_zc(10 downto 5) <= 15 and s_zc(10 downto 5) /= 5 then
                z_pre <= '1'; z_sel <= s_zc(10 downto 5);
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_vm, 16);   z_fb <= resize(s_vm, 16);           -- speed^2
                    when 1  => z_fa <= resize(c_spd, 16);  z_fb <= to_unsigned(364, 16);       -- bob phase
                    when 2  => z_fa <= resize(c_spd, 16);  z_fb <= to_unsigned(437, 16);       -- haze roll
                    when 3  => z_fa <= resize(c_spd, 16);  z_fb <= to_unsigned(182, 16);       -- haze drift
                    when 4  => z_fa <= resize(c_spd, 16);  z_fb <= to_unsigned(75, 16);        -- flames
                    when 6  => z_fa <= resize(unsigned(abs(resize(z_sn1, 9))), 16); z_fb <= to_unsigned(110, 16);   -- bob
                    when 7  => z_fa <= c_inv;              z_fb <= to_unsigned(3449, 16);      -- ku
                    when 8  => z_fa <= c_cstep;            z_fb <= resize(z_y7, 16);           -- comb slant per line
                    when 9  =>                                                                  -- comb depth
                        if s_g2(12 downto 3) <= 512 then z_fa <= resize(512 - s_g2(12 downto 3), 16);
                        else z_fa <= resize(s_g2(12 downto 3) - 512, 16); end if;
                        z_fb <= to_unsigned(101, 16);
                    when 10 =>                                                                  -- haze gain
                        if s_g3(12 downto 3) > 512 then z_fa <= resize(s_g3(12 downto 3) - 512, 16);
                        else z_fa <= (others => '0'); end if;
                        z_fb <= to_unsigned(77, 16);
                    when 11 => z_fa <= resize(s_gp(12 downto 3), 16); z_fb <= to_unsigned(831, 16);   -- tide
                    when 12 => z_fa <= resize(s_gp(12 downto 3), 16); z_fb <= to_unsigned(20, 16);
                    when 13 =>                                                                  -- drip: legs fade rate
                        if s_g5(12 downto 3) >= 512 then
                            z_fa <= resize(1023 - s_g5(12 downto 3), 16); z_fb <= to_unsigned(69, 16);
                        else
                            z_fa <= resize(512 - s_g5(12 downto 3), 16);  z_fb <= to_unsigned(414, 16);
                        end if;
                    when 14 =>                                                                  -- drip: legs start
                        if s_g5(12 downto 3) < 512 then z_fa <= resize(512 - s_g5(12 downto 3), 16);
                        else z_fa <= (others => '0'); end if;
                        z_fb <= to_unsigned(55, 16);
                    when others =>                                                              -- etch: bend
                        z_fa <= resize(s_g6(12 downto 3), 16);
                        if s_src = '1' then z_fb <= to_unsigned(384, 16); else z_fb <= to_unsigned(154, 16); end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 32); f_b <= z_fb;
            elsif m_go = '1' then              -- the line engine's request (it never runs during the zone slots)
                f_acc <= (others => '0');
                f_a <= resize(m_opa, 32); f_b <= m_opb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- products and single-clock steps
            v_st := s_zone = '1' and s_zc(4 downto 0) = "10010";
            v_k := s_zc(10 downto 5);
            if v_st and v_k = 1 then z_t <= f_acc(17 downto 0); end if;
            if v_st and v_k = 2 then c_pb <= c_pb + (f_acc(23 downto 8) xor (15 downto 0 => s_vneg)) + ("000000000000000" & s_vneg); end if;
            if v_st and v_k = 3 then c_roll <= c_roll + (f_acc(25 downto 8) xor (17 downto 0 => s_vneg)) + ("00000000000000000" & s_vneg); end if;
            if v_st and v_k = 4 then
                c_drift <= c_drift + (f_acc(27 downto 8) xor (19 downto 0 => not s_vneg)) + ("0000000000000000000" & not s_vneg);
            end if;
            if v_st and v_k = 5 then c_f1 <= c_f1 + (f_acc(23 downto 8) xor (15 downto 0 => s_vneg)) + ("000000000000000" & s_vneg); end if;
            if v_st and v_k = 7 then
                if z_sn1 < 0 then c_bob <= -signed(resize(f_acc(15 downto 7), 9));
                else c_bob <= signed(resize(f_acc(15 downto 7), 9)); end if;
            end if;
            if v_st and v_k = 8 then c_ku <= f_acc(27 downto 16); end if;
            if v_st and v_k = 9 then c_ccl <= f_acc(23 downto 8); end if;
            if v_st and v_k = 10 then c_cd <= 28 + f_acc(15 downto 8); end if;
            if v_st and v_k = 11 then
                if s_g3(12 downto 3) > 512 then c_hz <= 128 + resize(f_acc(15 downto 8), 9);
                else c_hz <= resize(s_g3(12 downto 5), 9); end if;
            end if;
            if v_st and v_k = 12 then c_yb <= to_unsigned(1230, 11) - f_acc(20 downto 10); end if;
            if v_st and v_k = 13 then c_yf <= c_yb + 205 - f_acc(20 downto 10); end if;
            if v_st and v_k = 14 then
                if s_g5(12 downto 3) >= 512 then c_legr <= f_acc(17 downto 8);
                else c_legr <= 138 + f_acc(17 downto 8); end if;
            end if;
            if v_st and v_k = 15 then c_legs0 <= 245 - f_acc(15 downto 8); end if;
            if v_st and v_k = 16 then c_eb <= f_acc(18 downto 11); end if;     -- bend gain / 8

            if s_zone = '1' then
                if s_zc = 20 then
                    c_flfsr <= c_flfsr(14 downto 0) & (c_flfsr(15) xor c_flfsr(13) xor c_flfsr(12) xor c_flfsr(10));
                end if;
                -- ghost scale, linear in 1/scale: 1.82 (k1 = 0) .. 1 (512) .. 0.53 (1023), Q15
                if s_zc = 22 then
                    if s_g1(12 downto 3) < 512 then z_d <= 512 - s_g1(12 downto 3); z_dn <= '1';
                    else z_d <= s_g1(12 downto 3) - 512; z_dn <= '0'; end if;
                end if;
                if s_zc = 23 then
                    if z_dn = '1' then z_t <= shift_left(resize(z_d, 18), 5) + shift_left(resize(z_d, 18), 4) + shift_left(resize(z_d, 18), 2);
                    else z_t <= shift_left(resize(z_d, 18), 5) - shift_left(resize(z_d, 18), 1); end if;
                end if;
                if s_zc = 24 then
                    if z_dn = '1' then c_inv <= 32768 + z_t(15 downto 0); else c_inv <= 32768 - z_t(15 downto 0); end if;
                end if;
                if s_zc = 51 then       -- speed: square law to 248 (= 240, the clip's rate), then 3 per step
                    if s_vm <= 248 then c_spd <= resize(z_t(17 downto 8), 11);
                    else c_spd <= to_unsigned(240, 11) + resize((s_vm - 248) & '0', 11) + resize(s_vm - 248, 11); end if;
                end if;
                if s_zc = 84 then r_sn_a <= c_pb(15 downto 8); end if;
                if s_zc = 86 then z_sn1 <= signed(r_sn_q); end if;
                if s_zc = 147 then c_xoff <= c_drift(19 downto 8) - 32; end if;
                if s_zc = 243 then      -- ghost centre (bob stored at 242); tear zone (the neck, 260 .. 120 lines above the centre)
                    c_fcy <= unsigned(to_signed(600, 11) + resize(c_bob, 11));
                end if;
                -- comb step (cycles Q16 per h): 298 (k2 = 0, 110 px) .. 840 (512, 39 px) .. 2184 (1023, 15 px)
                if s_zc = 26 then
                    if s_g2(12 downto 3) <= 512 then
                        z_cs <= resize(s_g2(12 downto 3), 12) + resize(s_g2(12 downto 7), 12);
                    else
                        v_g := s_g2(12 downto 3) - 512;
                        z_cs <= shift_left(resize(v_g, 12), 1) + resize(shift_right(v_g, 1), 12);
                    end if;
                end if;
                if s_zc = 27 then
                    if s_g2(12 downto 3) <= 512 then c_cstep <= 298 + resize(z_cs, 16);
                    else c_cstep <= 840 + resize(z_cs, 16) + resize(shift_right(s_g2(12 downto 3) - 512, 3), 16); end if;
                    z_y7 <= resize(shift_right(shift_left(resize(s_ys, 15), 3) - resize(s_ys, 15), 5), 12);   -- 56 ys / 256
                end if;
                if s_zc = 531 then z_t <= shift_left(resize(c_eb, 18), 7) + shift_left(resize(c_eb, 18), 3); end if;
                if s_zc = 532 then c_b140 <= resize(shift_right(z_t + shift_left(resize(c_eb, 18), 2), 5), 11); end if;
                if s_zc = 533 then
                    -- drip: the bottom key's slope 13/16 (the still) .. 3/16 (the curtains hang far down)
                    if s_g5(12 downto 3) > 512 then
                        v_g := s_g5(12 downto 3) - 512;
                        c_k13 <= 13 - resize(shift_right(v_g, 6), 4) - resize(shift_right(v_g, 7), 4);
                    else
                        c_k13 <= to_unsigned(13, 4);
                    end if;
                    -- tear seeds: Calm re-draws every 4 fields, Torn every 2
                    c_fcnt <= c_fcnt + 1;
                    if c_fcnt = 0 or (s_torn = '1' and c_fcnt(0) = '0') then
                        if c_flfsr = 0 then c_tseed <= x"5A5A"; else c_tseed <= c_flfsr; end if;
                    end if;
                end if;
            end if;

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
    -- Source side: oscillator (2-D triangle lattice, drifting), half-resolution
    -- line-buffer write of (luma >> 2) - 16
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
            s_lb_we <= data_in.avid and s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on the shared serial multiplier (slot j requested
    -- at lc = 2 + 19 j, product read at lc = 20 + 19 j), then builds both maps'
    -- interpolated rows into the profile bank the pixel path is not reading
    -- (2 ROM reads per entry; done by ~300 clocks, inside SD's 858)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : signed(12 downto 0);
        variable v_t  : signed(13 downto 0);
        variable v_a  : unsigned(15 downto 0);
        variable v_u  : signed(25 downto 0);
        variable v_bd : unsigned(7 downto 0);
        variable v_p  : unsigned(16 downto 0);
        variable v_v  : unsigned(8 downto 0);
        variable v_c  : unsigned(5 downto 0);
        variable v_dv : signed(9 downto 0);
        variable v_m  : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 900 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_first <= '1';
                e_ccl <= (others => '0');
                e_bandp <= (others => '1');
                e_lfsr <= c_tseed;
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_first <= '0';
                e_bank <= not e_bank;
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            m_go <= '0';
            if e_lc = 2 then   m_go <= '1'; m_opa <= c_inv;                           m_opb <= resize(e_dya, 16); end if;
            if e_lc = 21 then  m_go <= '1'; m_opa <= unsigned(resize(abs(e_yr), 16)); m_opb <= to_unsigned(93, 16); end if;
            if e_lc = 40 then  m_go <= '1'; m_opa <= resize(c_hz, 16);                m_opb <= resize(n_fyb, 16); end if;
            if e_lc = 59 then  m_go <= '1'; m_opa <= resize(c_legr, 16);              m_opb <= resize(e_dd, 16); end if;
            if e_lc = 78 then  m_go <= '1'; m_opa <= resize(e_lf, 16);                m_opb <= resize(n_fyf, 16); end if;
            if e_lc = 97 then  m_go <= '1'; m_opa <= resize(c_ku, 16);                m_opb <= unsigned(resize(abs(e_dx), 16)); end if;
            if e_lc = 116 then m_go <= '1'; m_opa <= unsigned(resize(abs(e_t2), 16)); m_opb <= resize(c_k13, 16); end if;

            if e_lc = 1 then
                v_y := signed(resize(e_vy(18 downto 8), 13));
                v_t := resize(v_y, 14) - signed(resize(c_fcy, 14));
                if v_t < 0 then e_dyn <= '1'; e_dya <= unsigned(resize(-v_t, 12)); else e_dyn <= '0'; e_dya <= unsigned(resize(v_t, 12)); end if;
                v_a := resize(e_vy(18 downto 8), 16) - 72 - resize(c_roll(17 downto 8), 16);
                n_rb <= v_a(9 downto 5); n_fyb <= v_a(4 downto 0);
                e_t1 <= v_y - 28;
                e_t2 <= v_y - signed(resize(c_yb, 13)) + 145;
                e_t3 <= v_y - signed(resize(c_yf, 13));
                if s_torn = '1' then v_bd := "00" & e_vy(18 downto 13); else v_bd := e_vy(18 downto 11); end if;
                if v_bd /= e_bandp then e_step <= '1'; e_bandp <= v_bd; else e_step <= '0'; end if;
            end if;
            if e_lc = 2 then
                if e_step = '1' then
                    e_lfsr <= e_lfsr(14 downto 0) & (e_lfsr(15) xor e_lfsr(13) xor e_lfsr(12) xor e_lfsr(10));
                end if;
                -- floor ramp: 1.25 per line (on |t3|, so no adder sees one sign bit on both inputs)
                v_m := resize(unsigned(abs(e_t3)), 14) + resize(shift_right(unsigned(abs(e_t3)), 2), 14);
                if v_m > 511 then
                    if e_t3 < 0 then n_by <= to_signed(-512, 10); else n_by <= to_signed(511, 10); end if;
                elsif e_t3 < 0 then n_by <= -signed(resize(v_m, 10));
                else n_by <= signed(resize(v_m, 10)); end if;
                -- top ramp: 6 per line
                if e_t1 > 0 then
                    e_wt <= signed(shift_left(resize(unsigned(e_t1(11 downto 0)), 16), 2) + shift_left(resize(unsigned(e_t1(11 downto 0)), 16), 1));
                else
                    e_wt <= (others => '0');
                end if;
            end if;
            if e_lc = 3 then
                if e_wt < 0 then n_wt <= (others => '0'); elsif e_wt > 255 then n_wt <= (others => '1');
                else n_wt <= unsigned(e_wt(7 downto 0)); end if;
                n_du <= f_mxs(resize(c_ku, 16), s_xs);
                n_cp0 <= to_unsigned(36976, 16) + e_ccl;                                        -- comb phase: origin + slant
                e_ccl <= e_ccl + c_ccl;
            end if;
            if e_lc = 20 then      -- yr = (y - fcy) / scale
                v_t := signed(resize(f_acc(27 downto 15), 14));
                if e_dyn = '1' then e_yr <= resize(-v_t, 13); else e_yr <= resize(v_t, 13); end if;
            end if;
            if e_lc = 21 then
                if e_yr > signed(resize(c_legs0, 13)) then e_dd <= unsigned(resize(e_yr - signed(resize(c_legs0, 13)), 12));
                else e_dd <= (others => '0'); end if;
                -- tear: Torn = bands across the frame; Calm = bands at the neck (260 .. 120 ghost lines above its centre)
                if s_torn = '1' then
                    if e_lfsr(11 downto 9) < 5 then e_tear <= signed(resize(e_lfsr(8 downto 0), 10)) - 256;
                    else e_tear <= (others => '0'); end if;
                else
                    if e_yr >= -260 and e_yr < -120 then e_tear <= signed(resize(e_lfsr(4 downto 0), 10)) - 16;
                    else e_tear <= (others => '0'); end if;
                end if;
                -- comb depth: full above the shoulders, halving every 30 ghost lines below them (32 levels)
                if e_yr <= 150 then n_dcl <= c_cd(7 downto 3);
                elsif e_yr <= 180 then n_dcl <= "0" & c_cd(7 downto 4);
                elsif e_yr <= 210 then n_dcl <= "00" & c_cd(7 downto 5);
                else n_dcl <= (others => '0'); end if;
            end if;
            if e_lc = 22 then
                n_x0 <= signed(resize(w_xb0, 13)) + resize(e_tear, 13);
            end if;
            if e_lc = 23 then
                if s_twin = '1' then e_dx <= to_signed(1060, 14) - resize(n_x0, 14);
                else e_dx <= resize(n_x0, 14) - to_signed(2016, 14); end if;
            end if;
            if e_lc = 39 then      -- figure map row (Q5): yr * 93 / 64 + 636
                v_t := signed(resize(f_acc(19 downto 6), 14));
                if e_yr < 0 then e_vf <= 636 - v_t; else e_vf <= 636 + v_t; end if;
            end if;
            if e_lc = 40 then
                if e_vf < 0 then
                    n_rf0 <= (others => '0'); n_rf1 <= (others => '0'); n_fyf <= (others => '0');
                elsif e_vf >= 992 then
                    n_rf0 <= (others => '1'); n_rf1 <= (others => '1'); n_fyf <= (others => '0');
                else
                    n_rf0 <= unsigned(e_vf(9 downto 5)); n_rf1 <= unsigned(e_vf(9 downto 5)) + 1; n_fyf <= unsigned(e_vf(4 downto 0));
                end if;
            end if;
            if e_lc = 58 then      -- fy * hz -> row weights of the haze (Q7)
                n_w1b <= f_acc(13 downto 5);
                n_w0b <= c_hz - f_acc(13 downto 5);
            end if;
            if e_lc = 77 then      -- legs fade
                if f_acc(23 downto 8) >= 256 then e_lf <= (others => '0');
                else e_lf <= to_unsigned(256, 9) - f_acc(16 downto 8); end if;
            end if;
            if e_lc = 96 then      -- fy * lf -> row weights of the ghost (Q8)
                n_w1f <= f_acc(13 downto 5);
                n_w0f <= e_lf - f_acc(13 downto 5);
            end if;
            if e_lc = 115 then     -- u0 = dx * ku + 15.5 cells
                v_u := signed(f_acc(25 downto 0));
                if e_dx < 0 then v_u := -v_u; end if;
                n_u0 <= v_u + to_signed(1015808, 26);
            end if;
            if e_lc = 134 then     -- bottom key term (y - yb + 145) * k13 / 16
                if e_t2 < 0 then n_a13 <= -signed(resize(f_acc(16 downto 4), 13));
                else n_a13 <= signed(resize(f_acc(16 downto 4), 13)); end if;
            end if;

            -- profile build: k_pc 0..127 = haze (64 columns x 2 rows), 128..191 = ghost (32 x 2)
            if e_lc = 100 then
                k_run <= '1'; k_pc <= (others => '0');
            elsif k_run = '1' then
                if k_pc = 191 then k_run <= '0'; end if;
                k_pc <= k_pc + 1;
            end if;
            -- stage 0: ROM addresses
            if k_pc(0) = '0' then
                r_bg_a <= n_rb & k_pc(6 downto 1);
                r_fg_a <= n_rf0 & k_pc(5 downto 1);
            else
                r_bg_a <= (n_rb + 1) & k_pc(6 downto 1);
                r_fg_a <= n_rf1 & k_pc(5 downto 1);
            end if;
            if k_pc(7) = '0' then
                k_ctl1 <= k_run & '0' & k_pc(0) & std_logic_vector(k_pc(6 downto 1));
            else
                k_ctl1 <= k_run & '1' & k_pc(0) & '0' & std_logic_vector(k_pc(5 downto 1));
            end if;
            -- stage 1: ROM data (EBR); stage 2: fabric copy + weight
            k_ctl2 <= k_ctl1;
            k_ctl3 <= k_ctl2;
            if k_ctl2(7) = '1' then k_q2 <= unsigned(r_fg_q); else k_q2 <= unsigned(r_bg_q); end if;
            case k_ctl2(7 downto 6) is
                when "00"   => k_w2 <= n_w0b;
                when "01"   => k_w2 <= n_w1b;
                when "10"   => k_w2 <= n_w0f;
                when others => k_w2 <= n_w1f;
            end case;
            -- stage 3: partial products; stage 4: product
            k_pa <= resize(k_q2(3 downto 0) * k_w2, 13);
            k_pb <= resize(k_q2(7 downto 4) * k_w2, 13);
            k_ctl4 <= k_ctl3;
            k_ctl5 <= k_ctl4;
            k_p4 <= resize(k_pa, 17) + shift_left(resize(k_pb, 17), 4);
            -- stage 5: accumulate the two rows and write the pair {v(c-1), clip(v(c) - v(c-1))}
            pb_we <= '0'; pf_we <= '0';
            k_last <= '0';
            if k_ctl5(8) = '1' then
                if k_ctl5(6) = '0' then
                    k_acc <= k_p4;
                else
                    v_p := k_acc + k_p4;
                    if k_ctl5(7) = '0' then v_v := v_p(15 downto 7); else v_v := v_p(16 downto 8); end if;
                    if v_v > 255 then v_v := to_unsigned(255, 9); end if;
                    v_c := unsigned(k_ctl5(5 downto 0));
                    v_dv := signed(resize(v_v(7 downto 0), 10)) - signed(resize(k_vp, 10));
                    if v_dv > 127 then v_dv := to_signed(127, 10); elsif v_dv < -127 then v_dv := to_signed(-127, 10); end if;
                    if v_c = 0 then
                        k_v0 <= v_v(7 downto 0);
                    else
                        if k_ctl5(7) = '0' then
                            pb_we <= '1'; pb_wa <= e_bank & (v_c - 1); pb_wd <= std_logic_vector(k_vp) & std_logic_vector(v_dv(7 downto 0));
                        else
                            pf_we <= '1'; pf_wa <= e_bank & (v_c(4 downto 0) - 1); pf_wd <= std_logic_vector(k_vp) & std_logic_vector(v_dv(7 downto 0));
                        end if;
                    end if;
                    k_vp <= v_v(7 downto 0);
                    if (k_ctl5(7) = '0' and v_c = 63) or (k_ctl5(7) = '1' and v_c = 31) then
                        k_last <= '1'; k_lastv <= v_v(7 downto 0); k_lastf <= k_ctl5(7);
                    end if;
                end if;
            end if;
            if k_last = '1' then       -- the last entry: towards v(0) for the wrapping haze, towards 0 for the ghost
                if k_lastf = '0' then
                    v_dv := signed(resize(k_v0, 10)) - signed(resize(k_lastv, 10));
                else
                    v_dv := -signed(resize(k_lastv, 10));
                end if;
                if v_dv > 127 then v_dv := to_signed(127, 10); elsif v_dv < -127 then v_dv := to_signed(-127, 10); end if;
                if k_lastf = '0' then
                    pb_we <= '1'; pb_wa <= e_bank & "111111"; pb_wd <= std_logic_vector(k_lastv) & std_logic_vector(v_dv(7 downto 0));
                else
                    pf_we <= '1'; pf_wa <= e_bank & "11111"; pf_wd <= std_logic_vector(k_lastv) & std_logic_vector(v_dv(7 downto 0));
                end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in half virtual px, the ghost's map phase (Twin: down to
    -- the centre, then up), the comb phase; line constants at g_start.  The
    -- camera is read 5 pixels ahead (its bend reaches stage 1 with its pixel).
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_ra : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xh <= n_x0;
                b_xn <= n_x0 + signed(resize(s_xs, 13));
                b_u  <= n_u0;
                b_dec <= s_twin;
                b_cp <= n_cp0;
                b_px <= (others => '0');
                l_bank <= e_bank;
                l_du <= n_du; l_wt <= n_wt; l_a13 <= n_a13; l_by <= n_by; l_dcl <= n_dcl;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_xh <= b_xn;
                b_xn <= b_xn + signed(resize(s_xs, 13));
                if b_dec = '1' and b_xn >= 1920 then
                    b_dec <= '0';                    -- the step into the right half holds
                else
                    b_u <= b_u + (signed(resize(l_du, 26)) xor (25 downto 0 => b_dec)) + signed'("0000000000000000000000000" & b_dec);
                end if;
                b_cp <= b_cp + w_cpx;
            end if;
            v_ra := b_px + 5;
            s_lb_ra <= v_ra(10 downto 1);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : signed(15 downto 0);
        variable v_x   : unsigned(11 downto 0);
        variable v_u2  : signed(25 downto 0);
        variable v_i   : signed(13 downto 0);
        variable v_s   : unsigned(8 downto 0);
        variable v_g   : unsigned(8 downto 0);
        variable v_sh  : signed(10 downto 0);
        variable v_y   : unsigned(10 downto 0);
        variable v_c   : signed(11 downto 0);
        variable v_b16 : unsigned(15 downto 0);
        variable v_9   : signed(8 downto 0);
        variable v_a   : unsigned(11 downto 0);
        variable v_g1  : unsigned(10 downto 0);
        variable v_wb  : unsigned(7 downto 0);
        variable v_fv  : unsigned(7 downto 0);
        variable v_bs  : signed(11 downto 0);
        variable v_cm  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- camera (5 pixels ahead): 4 fabric copy, 5 bend partials, 6 bend (h) -- belongs to the stage-1 pixel
            p4_lm <= unsigned(s_lb_q);
            p5_bl <= p4_lm(3 downto 0) * c_eb;
            p5_bh <= p4_lm(7 downto 4) * c_eb;
            v_b16 := resize(p5_bl, 16) + shift_left(resize(p5_bh, 16), 4);
            p6_bnd <= v_b16(15 downto 5);

            -- 2: profile addresses: haze x + drift - half a cell + bend; the ghost's phase + bend (x 2048, cells Q16);
            --    comb triangle; aura distance u - 16.92 cells; glow = bend beyond 140 levels
            v_x := unsigned(b_xh(11 downto 0)) + c_xoff + p6_bnd;
            pb_ra <= l_bank & v_x(11 downto 6);
            p2_fxb <= v_x(5 downto 0);
            v_u2 := b_u + shift_left(resize(signed('0' & p6_bnd), 26), 11);
            pf_ra <= l_bank & unsigned(v_u2(20 downto 16));
            if v_u2(25 downto 21) = "00000" then p2_ok <= '1'; else p2_ok <= '0'; end if;
            p2_fxf <= unsigned(v_u2(15 downto 10));
            p2_tc <= f_tri8(b_cp);
            p2_d <= b_u(25 downto 10) - 1082;                               -- Q6 cells
            if p6_bnd > c_b140 then p2_g1 <= p6_bnd - c_b140; else p2_g1 <= (others => '0'); end if;
            -- 3: profile pairs (EBR); |distance| (Q6 cells, one's complement); comb product; glow (x 0.92)
            p3_fxb <= p2_fxb; p3_fxf <= p2_fxf; p3_ok <= p2_ok;
            if p2_d(15) = '1' then v_d := not p2_d; else v_d := p2_d; end if;
            if v_d(15 downto 12) /= "0000" then p3_au <= (others => '1'); else p3_au <= unsigned(v_d(11 downto 0)); end if;
            p3_dc <= signed('0' & l_dcl) * p2_tc;
            v_g1 := p2_g1 - shift_right(p2_g1, 4) - shift_right(p2_g1, 6);
            if v_g1 > 511 then p_glw(3) <= (others => '1'); else p_glw(3) <= v_g1(8 downto 0); end if;
            for i in 4 to 7 loop p_glw(i) <= p_glw(i - 1); end loop;
            -- 4: profile pairs {v, dv} (fabric); aura slope (x 0.094); comb multiplier 256 + depth x triangle
            p4_b0 <= unsigned(pb_q(15 downto 8)); p4_db <= signed(pb_q(7 downto 0));
            if p3_ok = '1' then
                p4_f0 <= unsigned(pf_q(15 downto 8)); p4_df <= signed(pf_q(7 downto 0));
            else
                p4_f0 <= (others => '0'); p4_df <= (others => '0');
            end if;
            p4_fxb <= p3_fxb; p4_fxf <= p3_fxf;
            p4_av <= resize(shift_right(p3_au, 4), 10) + resize(shift_right(p3_au, 5), 10);
            v_cm := to_signed(256, 11) + resize(shift_right(p3_dc, 4), 11);
            p_cm(4) <= unsigned(v_cm(9 downto 0));
            for i in 5 to 6 loop p_cm(i) <= p_cm(i - 1); end loop;
            -- 5: lerp products; aura = 150 - distance x slope
            p5_b0 <= p4_b0; p5_f0 <= p4_f0;
            p5_bp <= resize(p4_db * signed(resize(p4_fxb, 7)), 15);
            p5_fp <= resize(p4_df * signed(resize(p4_fxf, 7)), 15);
            if p4_av >= 150 then p_aura(5) <= (others => '0'); else p_aura(5) <= to_unsigned(150, 8) - p4_av(7 downto 0); end if;
            for i in 6 to 9 loop p_aura(i) <= p_aura(i - 1); end loop;
            -- 6: haze level Tv, ghost level F
            v_9 := signed(resize(p5_b0, 9)) + resize(shift_right(p5_bp, 6), 9);
            if v_9 < 0 then p6_tv <= (others => '0'); else p6_tv <= unsigned(v_9(7 downto 0)); end if;
            v_9 := signed(resize(p5_f0, 9)) + resize(shift_right(p5_fp, 6), 9);
            if v_9 < 0 then p6_f <= (others => '0'); else p6_f <= unsigned(v_9(7 downto 0)); end if;
            -- 7: the bottom key's index (the brighter the haze, the lower it hangs); combed ghost product
            v_i := resize(l_a13, 14) - signed(resize(p6_tv, 14)) - signed(resize(p6_tv(7 downto 2), 14))
                   + signed(resize(p6_tv(7 downto 5), 14));
            if v_i < 0 then p7_idx <= (others => '0'); elsif v_i > 63 then p7_idx <= (others => '1');
            else p7_idx <= unsigned(v_i(5 downto 0)); end if;
            p7_tv <= p6_tv;
            p7_fm <= p6_f * p_cm(6);
            -- 8: window weight (top ramp; the bottom key 255 - ss(idx)); ghost + glow
            v_wb := 255 - unsigned(C_SP_SS(to_integer(p7_idx)));
            if l_wt < v_wb then p8_w <= l_wt; else p8_w <= v_wb; end if;
            p8_tv <= p7_tv;
            if p7_fm(17 downto 8) > 255 then v_fv := (others => '1'); else v_fv := p7_fm(15 downto 8); end if;
            v_s := resize(v_fv, 9) + resize(p_glw(7), 9);
            if v_s > 255 then p8_s <= (others => '1'); else p8_s <= v_s(7 downto 0); end if;
            -- floor flame phases of the stage-8 pixel
            if s_gav_sr(5) = '1' and s_gav_sr(6) = '0' then
                lf1 <= c_f1;
            else
                lf1 <= lf1 + w_fs1;
            end if;
            -- 9: haze; flame triangles
            p9_bg <= resize(shift_right(p8_tv * p8_w, 8), 8);
            p9_s <= p8_s;
            p9_t1 <= f_tri8(lf1); p9_t2 <= f_tri8((lf1(14 downto 0) & '0') + shift_right(lf1, 1));   -- x 2.5: 93 px
            -- 10: green level; shadow terms; floor terms
            v_g := resize(p9_bg, 9) + resize(p9_s, 9);
            if v_g > 255 then p10_g <= (others => '1'); else p10_g <= v_g(7 downto 0); end if;
            p10_ft <= resize(shift_right(p9_t1, 3), 10) + resize(shift_right(p9_t1, 5), 10)          -- 233 px flames, +-20
                      + resize(shift_right(p9_t2, 4), 10);                                          -- 93 px flames, +-8
            p10_b1 <= resize(l_by, 12) - signed(resize(p_aura(9), 12)) - signed(resize(shift_left(resize(p9_s, 9), 1), 12));
            -- 11: palette addresses: the green level, or its inverted key (Shadow: the dark floods green, the ghost
            --     turns black); floor level
            if s_shadow = '1' then
                if p10_g > 160 then r_pg_a <= s_pal & x"00"; else r_pg_a <= s_pal & (160 - p10_g); end if;
            else
                r_pg_a <= s_pal & p10_g;
            end if;
            v_bs := p10_b1 + resize(p10_ft, 12);
            if v_bs < 0 then r_pb_a <= s_pal & "0000000";
            elsif v_bs > 255 then r_pb_a <= s_pal & "1111111";
            else r_pb_a <= s_pal & unsigned(v_bs(7 downto 1)); end if;
            -- 12: ROM data (EBR); 13: fabric copy
            p13_g <= r_pg_q; p13_b <= r_pb_q;
            -- 14: green + blue in YUV; black outside the generated avid
            if s_gav_sr(11) = '0' then
                s_y14 <= w_ky; s_u14 <= w_ku; s_v14 <= w_kv;
            else
                v_y := resize(unsigned(p13_g(31 downto 22)), 11) + resize(unsigned(p13_b(31 downto 22)), 11);
                if v_y > 940 then s_y14 <= to_unsigned(940, 10); else s_y14 <= v_y(9 downto 0); end if;
                v_c := signed(resize(unsigned(p13_g(21 downto 12)), 12)) + signed(resize(unsigned(p13_b(21 downto 12)), 12)) - 512;
                if v_c < 0 then s_u14 <= (others => '0'); elsif v_c > 1023 then s_u14 <= (others => '1'); else s_u14 <= unsigned(v_c(9 downto 0)); end if;
                v_c := signed(resize(unsigned(p13_g(11 downto 2)), 12)) + signed(resize(unsigned(p13_b(11 downto 2)), 12)) - 512;
                if v_c < 0 then s_v14 <= (others => '0'); elsif v_c > 1023 then s_v14 <= (others => '1'); else s_v14 <= unsigned(v_c(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(11);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (8 bits,
    -- block-RAM delay), Y through [1 2 1]/4 at the same group delay, then the
    -- blanking gate (the luma 13/16 is in the palette)
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_yy : unsigned(11 downto 0);
        variable v_u8, v_v8 : unsigned(7 downto 0);
        variable v_r  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 15: running 8-sums of the 8-bit chroma (rounded); they snap back to the palette black once 8 blank
            --     samples are in, so a glitched sample can't offset the colours for good
            v_r := resize(s_u14, 11) + 2; if v_r(10) = '1' then v_u8 := x"FF"; else v_u8 := v_r(9 downto 2); end if;
            v_r := resize(s_v14, 11) + 2; if v_r(10) = '1' then v_v8 := x"FF"; else v_v8 := v_r(9 downto 2); end if;
            cr_wd <= std_logic_vector(v_u8) & std_logic_vector(v_v8);
            cr_wp <= cr_wp + 1; cr_rp <= cr_rp + 1; cr_we <= '1';
            cr_qq <= cr_q;                -- fabric copy of the ring's EBR output (read 6 behind the write)
            if s_bah = "00000000" then
                v_r := resize(w_ku, 11) + 2; s_bau <= v_r(9 downto 2) & "000";
                v_r := resize(w_kv, 11) + 2; s_bav <= v_r(9 downto 2) & "000";
            else
                s_bau <= s_bau + v_u8 - unsigned(cr_qq(15 downto 8));
                s_bav <= s_bav + v_v8 - unsigned(cr_qq(7 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            -- 16: [1 2 2 2 2 2 2 2 1]/16 (back to 10 bits); luma [1 2 1]/4 centred the same (6 clocks behind 14)
            if s_gav_sr(13) = '1' then
                v_sc := resize(s_bau, 12) + s_baud; s_u16 <= v_sc(11 downto 2);
                v_sc := resize(s_bav, 12) + s_bavd; s_v16 <= v_sc(11 downto 2);
            else
                s_u16 <= w_ku; s_v16 <= w_kv;
            end if;
            s_yd(0) <= s_y14;
            for i in 1 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            v_yy := resize(s_yd(3), 12) + shift_left(resize(s_yd(4), 12), 1) + resize(s_yd(5), 12);
            s_y16 <= v_yy(11 downto 2);
            -- 17: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y17);
    data_out.u       <= std_logic_vector(s_u17);
    data_out.v       <= std_logic_vector(s_v17);
    data_out.hsync_n <= sy_q(0);
    data_out.vsync_n <= sy_q(1);
    data_out.field_n <= sy_q(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture specter;
