-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- ORRERY v0.2.1 - the solar system: the Sun, eight planets on their orbits,
-- Saturn's rings, an asteroid belt and stars, seen from any tilt and centred
-- on any body.
--
-- WHAT IT DRAWS
--   An orrery, not to scale (orbit spacing and planet sizes are compressed to
--   fit a screen).  The planets go round at Kepler's relative speeds, each a
--   ball lit from the Sun's side and carrying its own small surface picture
--   (Earth's continents, Jupiter's bands and red spot, ...) that scrolls
--   round it.  Saturn's ring has striations and a gap.  The view tilts from
--   straight down to nearly edge-on, and Focus centres it on any body, so
--   the rest of the system wheels round that one.
--
-- HOW
--   Coordinates are HALF virtual units: X = xs * pixel (xs = 2 / 3 / 4 for
--   1080 / 720p / SD), Yh = 2 * (virtual line): every mode draws the same
--   picture and everything below is exact integer arithmetic.
--
--   A body is  e = (X - cx)^2 + (Yh - cy)^2 - r^2 < 0, a second-order
--   accumulator per body; all nine share one D accumulator (2 xs X + xs^2)
--   plus a per-body constant -2 xs cx.  The line offsets dy sit in a ring of
--   registers that rotates nine places in the horizontal blanking.
--
--   The orbit field is  ec = (X - CX)^2 + U^2,  U = (Yh - CY) / tilt.  Orbit
--   i is the contour ec = A_i^2; the orbit LINES are where the count of
--   orbits a pixel is inside changes from the pixel to the left or the one
--   above (a 4-bit line buffer): one pixel wide at any tilt.  The asteroid
--   belt is the gap between Mars's and Jupiter's orbits, sprinkled by a hash.
--
--   A planet's own orbit runs through its centre at right angles to the
--   Sun's direction, so its sunward half is "inside its own orbit":
--   shade = 160 + (A^2 - ec) >> n, a soft terminator with no light vector.
--
--   SURFACES.  Planet radii are powers of two (Saturn's ball is cut to 0.71
--   of its power of two, and its ring with it, so Jupiter is the biggest), so for the body picked at a
--   pixel its position across the disc is a shift of X plus a per-body
--   constant worked out in vblank, and a shift of dy (from the ring), with
--   no divide in any video mode.  That addresses
--   a 64 x 32 picture (4-bit index) and then that body's 16-colour palette,
--   both in block RAM.  The column takes a scroll offset, so the picture
--   goes round.  Saturn's ring is the tilted field centred on Saturn; its
--   radius indexes the ring's own palette (striations and a gap).
--
--   The nine line values E live in block RAM with the orbit angles; the
--   line job reads each, steps it, writes it back and loads that body's
--   accumulator.  FOCUS subtracts the chosen body's position (from the
--   field before) from the centre of everything.
--
--   A vblank sequencer (one serial multiplier, a sine ROM with linear
--   interpolation, 24-bit orbit angles in a small RAM) places the nine
--   bodies; a line job rotates the ring and works out U^2 for the orbit
--   field and the ring field.
--
--   The finished picture goes through the horizontal [1 2 1] / 4 filter on
--   Y, U and V (TECHNIQUES: output conditioning against the HD HDMI bars),
--   which also softens the hard edges of the balls and the orbit lines.
--
--   Pipeline: accumulators (S0) -> signs, compares, glow, ring (S1) -> pick
--   the front body (S2) -> its constants, orbit line, dx / dy (S3) ->
--   picture address (S4) -> picture; shade (S5) -> palette address (S6) ->
--   palette (S7, S8) -> colour x shade, sky (S9) -> body or sky (S10) ->
--   [1 2 1] (S11) -> output (S12).  The accumulators run 3 pixels ahead
--   (the first 3 pixels of a line repeat); latency 8 in every mode.
--
-- CONTROLS
--   K1  Zoom      whole system .. inner planets
--   K2  Speed     centre = still; right = forward in time, left = backward
--   K3  Stars     star density, four steps
--   K4  Orbits    brightness of the orbit lines (0 = none)
--   K5  Spin      the planets' own rotation; centre = still
--   K6  Focus     Sun, Mercury, Venus, Earth, Mars, Jupiter, Saturn, Uranus,
--                 Neptune: the body at the centre of the view
--   S7  Sky       Black / Video (the live picture behind everything)
--   S8  Belt      Off / On (asteroid belt)
--   S9  Rings     Off / On (Saturn)
--   S10 Shading   Sunlit / Flat
--   S11 Planets   Small / Large (planets x 4, Sun x 2: the surfaces show)
--   P12 TILT      straight down -> nearly edge-on
--
-- Latency: 8 clocks, constant across all modes.  EBR: 26.  No DSP.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.orrery_pkg.all;

architecture orrery of program_top is

    constant C_LAT : natural := 8;
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

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
            when 0          => return to_unsigned(1138, 12);  -- 480i
            when 4          => return to_unsigned(576, 12);   -- 480p
            when 8          => return to_unsigned(960, 12);   -- 576i
            when 12         => return to_unsigned(480, 12);   -- 576p
            when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p
            when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i
            when others     => return to_unsigned(256, 12);   -- 1080p
        end case;
    end function;

    function f_glide(a : unsigned(13 downto 0); k : std_logic_vector(9 downto 0))
        return unsigned is
        variable v_t : unsigned(13 downto 0);
        variable v_d : signed(14 downto 0);
        variable v_n : signed(14 downto 0);
    begin
        v_t := unsigned(k) & "0000";
        v_d := signed(resize(v_t, 15)) - signed(resize(a, 15));
        v_n := signed(resize(a, 15)) + shift_right(v_d, 4);
        return unsigned(v_n(13 downto 0));
    end function;

    -- the bodies: 0 Sun, 1 Mercury .. 8 Neptune (9 = Saturn's ring, colour only)
    function f_a0(i : unsigned(3 downto 0)) return unsigned is      -- orbit radius
    begin
        case to_integer(i) is
            when 1 => return to_unsigned(105, 10);  when 2 => return to_unsigned(150, 10);
            when 3 => return to_unsigned(205, 10);  when 4 => return to_unsigned(262, 10);
            when 5 => return to_unsigned(385, 10);  when 6 => return to_unsigned(505, 10);
            when 7 => return to_unsigned(615, 10);  when 8 => return to_unsigned(715, 10);
            when others => return to_unsigned(0, 10);
        end case;
    end function;

    function f_rate(i : unsigned(3 downto 0)) return unsigned is    -- (205 / a)^1.5 x 64
    begin
        case to_integer(i) is
            when 1 => return to_unsigned(175, 8);  when 2 => return to_unsigned(102, 8);
            when 3 => return to_unsigned(64, 8);   when 4 => return to_unsigned(44, 8);
            when 5 => return to_unsigned(25, 8);   when 6 => return to_unsigned(17, 8);
            when 7 => return to_unsigned(12, 8);   when 8 => return to_unsigned(10, 8);
            when others => return to_unsigned(0, 8);
        end case;
    end function;

    function f_rk(i : unsigned(3 downto 0)) return unsigned is      -- planet radius = 2^k half units
    begin
        case to_integer(i) is
            when 1 | 4  => return to_unsigned(4, 3);     -- Mercury, Mars
            when 5 | 6  => return to_unsigned(6, 3);     -- Jupiter, Saturn
            when others => return to_unsigned(5, 3);     -- Venus, Earth, Uranus, Neptune
        end case;
    end function;

    -- r^2 = 2^n half units^2: 2 k, but Saturn's ball is 1 / sqrt(2) of that
    -- (0.71 of Jupiter), its picture cropped to the middle
    function f_r2k(i : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when 1 | 4  => return to_unsigned(8, 4);     -- Mercury, Mars
            when 5      => return to_unsigned(12, 4);    -- Jupiter
            when 6      => return to_unsigned(11, 4);    -- Saturn
            when others => return to_unsigned(10, 4);    -- Venus, Earth, Uranus, Neptune
        end case;
    end function;

    function f_blen(x : unsigned) return unsigned is                -- bit length
        variable v_n : unsigned(3 downto 0);
    begin
        v_n := (others => '0');
        for i in 0 to x'length - 1 loop
            if x(x'low + i) = '1' then v_n := to_unsigned(i + 1, 4); end if;
        end loop;
        return v_n;
    end function;

    function f_abs(x : signed) return unsigned is
        variable v_r : signed(x'length - 1 downto 0);
    begin
        if x(x'high) = '1' then v_r := -x; else v_r := x; end if;
        return unsigned(v_r);
    end function;

    -- d * w as gated shift-adds
    function f_mulw(d : signed; w : unsigned) return signed is
        variable v_g : signed(d'length + w'length - 1 downto 0);
        variable v_r : signed(d'length + w'length - 1 downto 0);
    begin
        v_r := (others => '0');
        for i in 0 to w'length - 1 loop
            if w(w'low + i) = '1' then v_g := shift_left(resize(d, v_r'length), i);
            else                       v_g := (others => '0'); end if;
            v_r := v_r + v_g;
        end loop;
        return v_r;
    end function;

    --------------------------------------------------------------------------
    -- Sync / video delay, generated avid
    --------------------------------------------------------------------------
    type t_dsr is array (1 to C_LAT - 1) of unsigned(9 downto 0);
    subtype t_bsr is std_logic_vector(1 to C_LAT - 1);
    signal d_y, d_u, d_v : t_dsr := (others => (others => '0'));
    signal d_hs, d_vs, d_fl, d_av, d_gv : t_bsr := (others => '0');
    signal s_io : t_video_stream_yuv444_30b;

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_start : std_logic;
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_hcnt, s_meas_h : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lcnt  : unsigned(4 downto 0) := (others => '0');
    signal s_ltick, s_lpend : std_logic := '0';
    signal s_quiet : unsigned(11 downto 0) := (others => '0');
    signal s_qfired, s_go : std_logic := '0';
    signal r_run : std_logic;
    signal r_run1, r_run2 : std_logic := '0';

    -- timing class
    signal s_xs  : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_cls : unsigned(1 downto 0) := "00";                    -- 0: xs 2, 1: xs 3, 2: xs 4
    signal s_cx0 : signed(14 downto 0) := to_signed(1914, 15);     -- screen centre - 3 pixels
    signal s_c2  : unsigned(5 downto 0) := to_unsigned(8, 6);      -- 2 xs^2
    signal s_x2  : unsigned(4 downto 0) := to_unsigned(4, 5);      -- xs^2

    --------------------------------------------------------------------------
    -- Sequencer
    --------------------------------------------------------------------------
    signal q_pc   : unsigned(5 downto 0) := (others => '0');
    signal q_bi   : unsigned(3 downto 0) := (others => '0');
    signal q_gi   : unsigned(1 downto 0) := (others => '0');
    signal q_rt   : signed(10 downto 0) := (others => '0');
    signal q_rk   : std_logic_vector(9 downto 0) := (others => '0');
    signal q_mb, q_db : std_logic := '0';                          -- multiplier / divider busy
    signal q_mcnt, q_dcnt : unsigned(4 downto 0) := (others => '0');
    signal m_a    : signed(16 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : signed(17 downto 0) := (others => '0');
    signal m_lo   : unsigned(15 downto 0) := (others => '0');
    signal q_rem  : unsigned(8 downto 0) := (others => '0');
    signal q_quo  : unsigned(11 downto 0) := (others => '0');
    signal q_dbit : std_logic := '0';
    signal a_k1   : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal a_p12  : unsigned(9 downto 0) := to_unsigned(620, 10);
    signal q_zq, q_kq : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal q_spd  : signed(10 downto 0) := (others => '0');
    signal q_foc  : unsigned(3 downto 0) := (others => '0');        -- focus body
    signal q_rot  : unsigned(15 downto 0) := (others => '0');       -- surface scroll
    signal q_tfx, q_tfy : signed(13 downto 0) := (others => '0');   -- the focus body's position
    signal q_cxc, q_cyc : signed(14 downto 0) := (others => '0');   -- centre of the system
    signal q_th   : unsigned(23 downto 0) := (others => '0');
    signal q_sa   : unsigned(9 downto 0) := (others => '0');
    signal q_sd, q_sd2 : std_logic_vector(14 downto 0) := (others => '0');
    signal q_v0, q_s, q_c : signed(14 downto 0) := (others => '0');
    signal q_a    : unsigned(11 downto 0) := (others => '0');
    signal q_px, q_pyp, q_py : signed(13 downto 0) := (others => '0');
    signal q_cx, q_cy : signed(14 downto 0) := (others => '0');
    signal q_cx2  : signed(28 downto 0) := (others => '0');         -- cx^2 - r^2
    signal q_r2   : unsigned(17 downto 0) := (others => '0');       -- r^2 of this body
    signal q_ab   : unsigned(14 downto 0) := (others => '0');
    signal q_u    : unsigned(13 downto 0) := (others => '0');
    signal q_del  : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal q_rc   : unsigned(3 downto 0) := (others => '0');
    signal q_preg : signed(19 downto 0) := (others => '0');
    signal q_prime : std_logic := '0';
    signal s_ready : std_logic := '0';

    -- state RAM: orbit angles (16.8) at 0 .. 8, line values E at 16 .. 24
    type t_st is array (0 to 255) of std_logic_vector(31 downto 0);
    function f_st_init return t_st is
        variable v_r : t_st;
    begin
        v_r := (others => (others => '0'));
        v_r(1) := std_logic_vector(to_unsigned(9000 * 256, 32));
        v_r(2) := std_logic_vector(to_unsigned(30000 * 256, 32));
        v_r(3) := std_logic_vector(to_unsigned(52000 * 256, 32));
        v_r(4) := std_logic_vector(to_unsigned(14000 * 256, 32));
        v_r(5) := std_logic_vector(to_unsigned(41000 * 256, 32));
        v_r(6) := std_logic_vector(to_unsigned(22000 * 256, 32));
        v_r(7) := std_logic_vector(to_unsigned(60000 * 256, 32));
        v_r(8) := std_logic_vector(to_unsigned(3000 * 256, 32));
        return v_r;
    end function;
    signal r_st : t_st := f_st_init;
    signal st_q, st_q2, st_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal st_ra, st_wa : unsigned(4 downto 0) := (others => '0');
    signal st_we : std_logic := '0';

    -- the ring: per-line dy of each body
    type t_dy9 is array (0 to 8) of signed(14 downto 0);
    type t_e9  is array (0 to 8) of signed(28 downto 0);
    signal r_dy : t_dy9 := (others => (others => '0'));

    -- per-frame constants
    type t_d9  is array (0 to 8) of signed(17 downto 0);
    type t_a8  is array (1 to 8) of unsigned(15 downto 0);          -- A^2 / 256
    type t_sh9 is array (0 to 8) of unsigned(3 downto 0);
    signal f_D0  : t_d9 := (others => (others => '0'));
    signal f_a2  : t_a8 := (others => (others => '0'));
    signal f_sh  : t_sh9 := (others => (others => '0'));
    signal f_front : std_logic_vector(8 downto 0) := (others => '0');
    signal f_kinv  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal f_cx6sq, f_xc0 : unsigned(27 downto 0) := (others => '0');
    signal f_kc    : signed(17 downto 0) := (others => '0');        -- -2 xs CX
    signal f_ol    : unsigned(7 downto 0) := (others => '0');
    signal f_off   : unsigned(8 downto 0) := (others => '0');       -- scroll, 1/8 columns
    signal f_stm   : unsigned(9 downto 0) := (others => '1');       -- star mask
    signal f_sky, f_belt, f_rings, f_flat, f_large : std_logic := '0';

    -- per-line values
    signal l_yacc : unsigned(18 downto 0) := (others => '0');
    signal l_yv   : unsigned(10 downto 0) := (others => '0');
    signal l_Ec, l_Er : unsigned(27 downto 0) := (others => '1');
    signal l_sfront, l_first : std_logic := '0';

    --------------------------------------------------------------------------
    -- Pixel path
    --------------------------------------------------------------------------
    signal p_e  : t_e9 := (others => (others => '0'));
    signal p_ec, p_er : unsigned(27 downto 0) := (others => '1');
    signal p_Dg : signed(16 downto 0) := (others => '0');          -- 2 xs X + xs^2
    signal p_px, p_px1, p_px2 : unsigned(10 downto 0) := (others => '0');
    signal p_x, p_x1, p_x2 : unsigned(11 downto 0) := (others => '0');

    signal s1_rf, s1_rb : std_logic_vector(9 downto 0) := (others => '0');
    signal s1_in0 : std_logic := '0';
    signal s1_o   : std_logic_vector(8 downto 1) := (others => '0');
    signal s1_ec  : unsigned(19 downto 0) := (others => '1');     -- ec / 256
    signal s1_g8  : unsigned(7 downto 0) := (others => '0');
    signal s1_rc  : unsigned(3 downto 0) := (others => '0');

    signal s2_sel : unsigned(3 downto 0) := (others => '1');
    signal s2_cnt : unsigned(3 downto 0) := (others => '0');
    signal s2_ec  : unsigned(19 downto 0) := (others => '1');
    signal s2_g8  : unsigned(7 downto 0) := (others => '0');
    signal s2_rc  : unsigned(3 downto 0) := (others => '0');

    signal s3_sel : unsigned(3 downto 0) := (others => '1');
    signal s3_d   : signed(20 downto 0) := (others => '0');
    signal s3_sh  : unsigned(3 downto 0) := (others => '0');
    signal s3_g8  : unsigned(7 downto 0) := (others => '0');
    signal s3_rc  : unsigned(3 downto 0) := (others => '0');
    signal s3_line : std_logic := '0';
    signal s3_cntl, s3_cnt : unsigned(3 downto 0) := (others => '0');
    signal s3_h   : unsigned(15 downto 0) := (others => '0');
    signal s3_xv  : unsigned(10 downto 0) := (others => '0');
    signal s3_dy : signed(12 downto 0) := (others => '0');
    type t_cu9 is array (0 to 8) of unsigned(8 downto 0);
    signal f_cu  : t_cu9 := (others => (others => '0'));          -- 128 + scroll - (cx << sc) / 2
    signal s3_cu, s3_xu : unsigned(8 downto 0) := (others => '0');
    signal s3_sc  : unsigned(2 downto 0) := (others => '0');       -- left shift + 1

    signal s4_sel : unsigned(3 downto 0) := (others => '1');
    signal s4_ds  : signed(8 downto 0) := (others => '0');
    signal s4_g8  : unsigned(7 downto 0) := (others => '0');
    signal s4_rc  : unsigned(3 downto 0) := (others => '0');
    signal s4_line : std_logic := '0';
    signal s4_cnt : unsigned(3 downto 0) := (others => '0');
    signal s4_h   : unsigned(15 downto 0) := (others => '0');
    signal s4_ta  : unsigned(13 downto 0) := (others => '0');

    signal s5_sel : unsigned(3 downto 0) := (others => '1');
    signal s5_rc  : unsigned(3 downto 0) := (others => '0');
    signal q_tx   : std_logic_vector(3 downto 0) := (others => '0');
    signal s6_pa  : unsigned(7 downto 0) := (others => '0');
    signal q_pl   : std_logic_vector(21 downto 0) := (others => '0');
    signal s8_pal : std_logic_vector(21 downto 0) := (others => '0');

    -- carried from S5 to S9: shade, body / sky flag, sky extras, glow
    type t_c27 is array (5 to 8) of std_logic_vector(21 downto 0);
    signal c_bus : t_c27 := (others => (others => '0'));

    signal s9_yb  : unsigned(9 downto 0) := (others => '0');
    signal s9_cu, s9_cb : signed(9 downto 0) := (others => '0');
    signal s9_sky : unsigned(9 downto 0) := (others => '0');
    signal s9_hit : std_logic := '0';
    signal s9_g8  : unsigned(7 downto 0) := (others => '0');
    signal s10_y, s10_u, s10_v : unsigned(9 downto 0) := (others => '0');
    signal w0_y, w0_u, w0_v, w1_y, w1_u, w1_v : unsigned(9 downto 0) := (others => '0');

    -- orbit-line buffer: the count of orbits a pixel is inside, one line back
    type t_lb is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal r_lb : t_lb;
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_lb : signal is true;
    attribute no_rw_check of r_st : signal is true;
    signal lb_q, lb_q2 : std_logic_vector(3 downto 0) := (others => '0');

begin

    -- ====================================================================
    -- Timing class, free-running
    -- ====================================================================
    p_class : process(clk)
        variable v_t : std_logic_vector(3 downto 0);
    begin
        if rising_edge(clk) then
            v_t := registers_in(8)(3 downto 0);
            s_xs <= f_vstep_x(v_t);
            s_ys <= f_vstep_y(v_t);
            case to_integer(unsigned(v_t)) is
                when 0 | 4 | 8 | 12 =>                   -- SD: 1440 wide, xs = 4
                    s_cls <= "10";
                    s_cx0 <= to_signed(1440 - 12, 15);
                    s_c2  <= to_unsigned(32, 6);
                    s_x2  <= to_unsigned(16, 5);
                when 5 | 6 | 14 =>                       -- 720p: xs = 3
                    s_cls <= "01";
                    s_cx0 <= to_signed(1920 - 9, 15);
                    s_c2  <= to_unsigned(18, 6);
                    s_x2  <= to_unsigned(9, 5);
                when others =>                           -- 1080: xs = 2
                    s_cls <= "00";
                    s_cx0 <= to_signed(1920 - 6, 15);
                    s_c2  <= to_unsigned(8, 6);
                    s_x2  <= to_unsigned(4, 5);
            end case;
        end if;
    end process p_class;

    -- ====================================================================
    -- Clean avid, line width, line tick, field tick (as LUNA / FARSIDE)
    -- ====================================================================
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1')
               else '0';
    -- the accumulators step from one clock before the generated avid
    -- (they hold pixel 3's values through blanking: pipeline of 12, latency 8)
    r_run <= g_start or g_avid;

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

            if g_hsr = '1' then
                if s_hcnt /= 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            r_run1 <= r_run;
            r_run2 <= r_run1;

            -- line tick: 12 clocks after the generated avid ends
            g_avd <= g_avid;
            s_ltick <= '0';
            if g_avd = '1' and g_avid = '0' then
                s_lcnt <= to_unsigned(12, 5);
            elsif s_lcnt /= 0 then
                s_lcnt <= s_lcnt - 1;
                if s_lcnt = 1 then s_ltick <= '1'; end if;
            end if;

            -- field tick: no active video for 2048 clocks
            s_go <= '0';
            if data_in.avid = '1' then
                s_quiet  <= (others => '0');
                s_qfired <= '0';
            elsif s_quiet(11) = '0' then
                s_quiet <= s_quiet + 1;
            elsif s_qfired = '0' then
                s_qfired <= '1';
                s_go     <= '1';
            end if;
        end if;
    end process p_gavid;

    -- sine ROM and the state RAM (block RAM), re-registered in fabric
    p_sin : process(clk)
    begin
        if rising_edge(clk) then
            q_sd  <= C_OR_SIN(to_integer(q_sa));
            q_sd2 <= q_sd;
            if st_we = '1' then
                r_st(to_integer(resize(st_wa, 8))) <= st_wd;
            end if;
            st_q  <= r_st(to_integer(resize(st_ra, 8)));
            st_q2 <= st_q;
        end if;
    end process p_sin;

    -- ====================================================================
    -- Sequencer.  pc 1 .. 30: once per field, place the nine bodies.
    -- pc 40 .. 49: after every line, step the ring and the two tilted fields.
    -- A step that launches a product is followed by one that reads it.
    -- ====================================================================
    p_seq : process(clk)
        variable v_p    : signed(33 downto 0);
        variable v_sum  : signed(17 downto 0);
        variable v_r    : unsigned(9 downto 0);
        variable v_k    : unsigned(9 downto 0);
        variable v_sd   : std_logic;
        variable v_mg   : unsigned(8 downto 0);
        variable v_mant : unsigned(11 downto 0);
        variable v_th   : unsigned(23 downto 0);
        variable v_E    : signed(28 downto 0);
        variable v_r2   : unsigned(17 downto 0);
        variable v_ai   : signed(14 downto 0);
        variable v_ab   : unsigned(14 downto 0);
        variable v_new  : signed(28 downto 0);
        variable v_rk   : std_logic_vector(9 downto 0);
        variable v_rt   : signed(10 downto 0);
        variable v_cx8  : signed(17 downto 0);
        variable v_ya   : unsigned(18 downto 0);
        variable v_dn   : signed(14 downto 0);
        variable v_dyc  : signed(15 downto 0);
        variable v_us   : signed(18 downto 0);
        variable v_n    : unsigned(4 downto 0);
        variable v_src  : signed(14 downto 0);
        variable v_fs   : signed(19 downto 0);
        variable v_sc3  : unsigned(2 downto 0);
        variable v_c9   : unsigned(8 downto 0);
        variable v_ga   : unsigned(9 downto 0);
        variable v_gk   : unsigned(9 downto 0);
        variable v_gd   : signed(10 downto 0);
        variable v_f9   : unsigned(12 downto 0);

        procedure mul(a : signed; b : unsigned) is
        begin
            m_a    <= resize(a, 17);
            m_b    <= resize(b, 16);
            m_acc  <= (others => '0');
            m_lo   <= (others => '0');
            q_mcnt <= to_unsigned(16, 5);
            q_mb   <= '1';
        end procedure;

        -- Delta x d for the per-line step (Delta = 1 .. 5), then x 4 + 4 Delta^2
        function f_step(d : signed(14 downto 0); del : unsigned(2 downto 0)) return signed is
            variable v_m : signed(17 downto 0);
            variable v_c : signed(19 downto 0);
        begin
            case to_integer(del) is
                when 0      => v_m := (others => '0');                            v_c := to_signed(0, 20);
                when 1      => v_m := resize(d, 18);                              v_c := to_signed(4, 20);
                when 2      => v_m := shift_left(resize(d, 18), 1);               v_c := to_signed(16, 20);
                when 3      => v_m := shift_left(resize(d, 18), 1) + resize(d, 18);  v_c := to_signed(36, 20);
                when 4      => v_m := shift_left(resize(d, 18), 2);               v_c := to_signed(64, 20);
                when others => v_m := shift_left(resize(d, 18), 2) + resize(d, 18);  v_c := to_signed(100, 20);
            end case;
            return shift_left(resize(v_m, 20), 2) + v_c;
        end function;

        -- knob -> signed rate, still inside +-16 of the centre
        function f_rate11(k : std_logic_vector(9 downto 0)) return signed is
            variable v_s : std_logic;
            variable v_g : unsigned(8 downto 0);
            variable v_m : unsigned(11 downto 0);
        begin
            v_s := not k(9);
            for i in 0 to 8 loop
                v_g(i) := k(i) xor v_s;
            end loop;
            if v_g(8 downto 4) = 0 then
                return to_signed(0, 11);
            end if;
            v_m := (others => '0');
            v_m(4) := '1';
            v_m(3 downto 0) := v_g(5 downto 2);
            v_m := shift_left(v_m, to_integer(v_g(8 downto 6)));
            if v_s = '1' then return -signed(resize(v_m(11 downto 2), 11));
            else              return signed(resize(v_m(11 downto 2), 11)); end if;
        end function;
    begin
        if rising_edge(clk) then
            if s_ltick = '1' then s_lpend <= '1'; end if;
            st_we <= '0';
            -- one |x| for the squares: the centre, then a body's cx, then its cy
            if q_pc = 3 then     v_ai := q_cxc;
            elsif q_pc = 25 then v_ai := q_cy;
            else                 v_ai := q_cx;
            end if;
            v_ab := resize(f_abs(v_ai), 15);
            -- one knob-to-rate unit: Speed on the first pass, Spin on the second
            v_rt := f_rate11(q_rk);
            v_p(33 downto 16) := m_acc;
            v_p(15 downto 0)  := signed(m_lo);
            -- the ring's step for the head (first) or the next slot (after)
            if q_prime = '1' then v_src := r_dy(0); else v_src := r_dy(1); end if;
            v_fs := f_step(v_src, q_del);

            if q_mb = '1' then                           -- serial multiply, 16 steps
                if q_mcnt = 1 then q_mb <= '0'; end if;
                if m_b(0) = '1' then v_sum := m_acc + resize(m_a, 18);
                else                 v_sum := m_acc; end if;
                m_acc  <= shift_right(v_sum, 1);
                m_lo   <= v_sum(0) & m_lo(15 downto 1);
                m_b    <= '0' & m_b(15 downto 1);
                q_mcnt <= q_mcnt - 1;
            elsif q_db = '1' then                        -- 65536 / tilt, restoring
                if q_dcnt = 1 then q_db <= '0'; end if;
                v_r := q_rem & q_dbit;
                if v_r >= resize(q_kq, 10) then
                    v_r := v_r - resize(q_kq, 10);
                    q_quo <= q_quo(10 downto 0) & '1';
                else
                    q_quo <= q_quo(10 downto 0) & '0';
                end if;
                q_rem  <= v_r(8 downto 0);
                q_dbit <= '0';
                q_dcnt <= q_dcnt - 1;
            else
                case to_integer(q_pc) is
                    when 0 =>
                        if s_go = '1' then
                            q_gi <= (others => '0');
                            q_pc <= to_unsigned(1, 6);
                        elsif s_lpend = '1' then
                            s_lpend <= '0';
                            q_pc <= to_unsigned(40, 6);
                        end if;

                    ---------------------------------------------- field
                    when 1 =>                            -- four passes: q_gi = 0 .. 3
                        -- Zoom (pass 0) and Tilt (pass 1) follow their knobs
                        -- with two counts of hysteresis
                        v_gd := signed(resize(unsigned(registers_in(0)), 11)) - signed(resize(a_k1, 11));
                        if v_gd(10 downto 1) /= 0 and v_gd /= -1 then    -- not -1, 0, +1
                            a_k1 <= unsigned(registers_in(0));
                        end if;
                        v_gd := signed(resize(unsigned(registers_in(7)), 11)) - signed(resize(a_p12, 11));
                        if v_gd(10 downto 1) /= 0 and v_gd /= -1 then
                            a_p12 <= unsigned(registers_in(7));
                        end if;
                        -- Speed (pass 0 -> 1) and Spin (pass 1 -> 2) through one rate unit
                        if q_gi(0) = '0' then q_rk <= registers_in(1); else q_rk <= registers_in(4); end if;
                        q_rt <= v_rt;
                        if q_gi = 2 then q_spd <= q_rt; end if;
                        if q_gi = 3 then q_rot <= unsigned(signed(q_rot) + resize(q_rt, 16)); end if;
                        if q_gi = 0 then
                            f_sky   <= registers_in(6)(0);
                            f_belt  <= registers_in(6)(1);
                            f_rings <= registers_in(6)(2);
                            f_flat  <= registers_in(6)(3);
                            f_large <= registers_in(6)(4);
                            f_ol    <= unsigned(registers_in(3)(9 downto 2));
                            f_stm   <= shift_right(to_unsigned(1023, 10), to_integer(unsigned(registers_in(2)(9 downto 8))));
                            g_wm1   <= s_meas_h - 1;
                            -- Focus: nine zones across the knob, with hysteresis
                            v_k  := unsigned(registers_in(5));
                            v_f9 := shift_left(resize(v_k, 13), 3) + resize(v_k, 13);
                            if v_f9(12 downto 10) > q_foc(2 downto 0) and q_foc(3) = '0' and v_f9(9 downto 0) >= 64 then
                                q_foc <= '0' & v_f9(12 downto 10);
                            elsif ('0' & v_f9(12 downto 10)) < q_foc and v_f9(9 downto 0) < 960 then
                                q_foc <= '0' & v_f9(12 downto 10);
                            end if;
                        end if;
                        l_yacc  <= (others => '0');
                        l_yv    <= (others => '0');
                        l_first <= '1';
                        s_lpend <= '0';
                        q_bi <= (others => '0');
                        q_gi <= q_gi + 1;
                        if q_gi = 3 then q_pc <= to_unsigned(2, 6); end if;

                    when 2 =>                            -- zoom, tilt (Q.6, Q.8); the centre
                        v_k  := a_k1;
                        q_zq <= to_unsigned(26, 8) + resize(shift_right(v_k, 3), 8) + resize(shift_right(v_k, 6), 8);
                        v_k  := a_p12;
                        q_kq <= to_unsigned(255, 8) - resize(shift_right(v_k, 2), 8) + resize(shift_right(v_k, 5), 8);
                        f_off <= q_rot(15 downto 7) + 128;        -- scroll + the half-width
                        if q_foc = 0 then
                            q_cxc <= s_cx0;
                            q_cyc <= to_signed(1080, 15);
                        else                             -- follow the focus body
                            q_cxc <= s_cx0 - resize(q_tfx, 15);
                            q_cyc <= to_signed(1080, 15) - resize(q_tfy, 15);
                        end if;
                        q_pc <= to_unsigned(3, 6);

                    when 3 =>
                        q_rem  <= (others => '0');
                        q_quo  <= (others => '0');
                        q_dbit <= '1';
                        q_dcnt <= to_unsigned(17, 5);
                        q_db   <= '1';
                        q_ab   <= v_ab;                  -- |CX|
                        q_pc   <= to_unsigned(4, 6);

                    when 4 =>
                        f_kinv <= q_quo;
                        mul(signed(resize(q_ab, 16)), q_ab);
                        q_pc   <= to_unsigned(30, 6);

                    when 30 =>                           -- the orbit field's x constants
                        f_xc0 <= unsigned(v_p(27 downto 0));
                        case to_integer(s_xs) is
                            when 4      => f_kc <= -shift_left(resize(q_cxc, 18), 3);
                            when 3      => f_kc <= -(shift_left(resize(q_cxc, 18), 2) + shift_left(resize(q_cxc, 18), 1));
                            when others => f_kc <= -shift_left(resize(q_cxc, 18), 2);
                        end case;
                        q_pc <= to_unsigned(5, 6);

                    ---------------------------------------------- body q_bi
                    when 5 =>                            -- angle step = rate x speed
                        mul(q_spd, f_rate(q_bi));
                        st_ra <= resize(q_bi, 5);        -- this body's angle
                        q_pc <= to_unsigned(6, 6);

                    when 6 =>
                        if q_bi = 0 then v_th := unsigned(st_q2(23 downto 0));
                        else             v_th := unsigned(st_q2(23 downto 0)) + unsigned(v_p(23 downto 0)); end if;
                        q_th <= v_th;
                        q_sa <= v_th(23 downto 14);
                        q_pc <= to_unsigned(7, 6);

                    when 7 =>  q_sa <= q_sa + 1;  q_pc <= to_unsigned(8, 6);
                    when 8 =>  q_pc <= to_unsigned(9, 6);
                    when 9 =>  q_v0 <= signed(q_sd2);  q_pc <= to_unsigned(10, 6);

                    when 10 =>                           -- sin, interpolated
                        mul(resize(signed(q_sd2), 16) - resize(q_v0, 16), q_th(13 downto 8));
                        q_pc <= to_unsigned(11, 6);

                    when 11 =>                           -- then the same a quarter turn on
                        q_s  <= q_v0 + v_p(20 downto 6);
                        q_sa <= q_sa + 255;
                        q_pc <= to_unsigned(12, 6);

                    when 12 => q_sa <= q_sa + 1;  q_pc <= to_unsigned(13, 6);
                    when 13 => q_pc <= to_unsigned(32, 6);
                    when 32 => q_v0 <= signed(q_sd2);  q_pc <= to_unsigned(33, 6);

                    when 33 =>
                        mul(resize(signed(q_sd2), 16) - resize(q_v0, 16), q_th(13 downto 8));
                        q_pc <= to_unsigned(14, 6);

                    when 14 =>                           -- cos; then the orbit radius
                        q_c <= q_v0 + v_p(20 downto 6);
                        mul(signed(resize(f_a0(q_bi), 11)), q_zq);
                        q_pc <= to_unsigned(15, 6);

                    when 15 =>
                        q_a  <= unsigned(v_p(16 downto 5));
                        q_pc <= to_unsigned(16, 6);

                    when 16 =>
                        mul(q_c, q_a);
                        q_pc <= to_unsigned(17, 6);

                    when 17 =>
                        q_px <= v_p(27 downto 14);
                        mul(q_s, q_a);
                        q_pc <= to_unsigned(18, 6);

                    when 18 =>
                        q_pyp <= v_p(27 downto 14);
                        q_pc  <= to_unsigned(19, 6);

                    when 19 =>                           -- the tilt squashes y
                        mul(q_pyp, q_kq);
                        q_pc <= to_unsigned(20, 6);

                    when 20 =>
                        q_py <= -v_p(21 downto 8);
                        q_pc <= to_unsigned(21, 6);

                    when 21 =>
                        q_cx <= q_cxc + resize(q_px, 15);
                        q_cy <= q_cyc + resize(q_py, 15);
                        if q_bi = q_foc then             -- next field's focus offset
                            q_tfx <= q_px;
                            q_tfy <= q_py;
                        end if;
                        mul(signed(resize(q_a, 13)), q_a);
                        q_pc <= to_unsigned(24, 6);

                    when 24 =>
                        if q_bi /= 0 then
                            f_a2(to_integer(q_bi)) <= unsigned(v_p(23 downto 8));
                        end if;
                        q_ab <= v_ab;                    -- |cx|
                        if q_bi = 0 then                 -- r^2: the Sun 96 (x 2), planets 2^n (x 16)
                            if f_large = '1' then v_r2 := to_unsigned(36864, 18);
                            else                  v_r2 := to_unsigned(9216, 18); end if;
                        else
                            v_r2 := shift_left(to_unsigned(1, 18), to_integer(f_r2k(q_bi)));
                            if f_large = '1' then v_r2 := shift_left(v_r2, 4); end if;
                        end if;
                        q_r2 <= v_r2;
                        q_pc <= to_unsigned(31, 6);

                    when 31 =>
                        mul(signed(resize(q_ab, 16)), q_ab);
                        q_pc <= to_unsigned(25, 6);

                    when 25 =>
                        q_cx2 <= signed(resize(unsigned(v_p(27 downto 0)), 29)) - signed(resize(q_r2, 29));
                        if q_bi = 6 then
                            f_cx6sq <= unsigned(v_p(27 downto 0));   -- cx^2, for Saturn's ring field
                        end if;
                        q_ab  <= v_ab;                   -- |cy|
                        q_pc  <= to_unsigned(27, 6);

                    when 27 =>
                        mul(signed(resize(q_ab, 16)), q_ab);
                        q_pc <= to_unsigned(26, 6);

                    when 26 =>                           -- store the body
                        v_E := q_cx2 + signed(resize(unsigned(v_p(27 downto 0)), 29));   -- + cy^2
                        st_we <= '1';                    -- E to the state RAM (and its accumulator)
                        st_wa <= '1' & q_bi;
                        st_wd <= std_logic_vector(resize(v_E, 32));
                        for j in 0 to 7 loop             -- dy into the ring
                            r_dy(j) <= r_dy(j + 1);
                        end loop;
                        r_dy(8) <= -q_cy;
                        case to_integer(s_xs) is         -- K = -2 xs cx
                            when 4      => v_cx8 := -shift_left(resize(q_cx, 18), 3);
                            when 3      => v_cx8 := -(shift_left(resize(q_cx, 18), 2) + shift_left(resize(q_cx, 18), 1));
                            when others => v_cx8 := -shift_left(resize(q_cx, 18), 2);
                        end case;
                        f_D0(to_integer(q_bi)) <= v_cx8;
                        if q_py > 0 then f_front(to_integer(q_bi)) <= '1';
                        else             f_front(to_integer(q_bi)) <= '0'; end if;
                        -- shade shift = bits(A) + bits(r) - 15 (the difference is / 256)
                        v_n := resize(f_blen(q_a), 5) + resize(f_rk(q_bi), 5) + 1;
                        if f_large = '1' then v_n := v_n + 2; end if;
                        if v_n < 15 then f_sh(to_integer(q_bi)) <= (others => '0');
                        else             f_sh(to_integer(q_bi)) <= resize(v_n - 15, 4); end if;
                        q_pc <= to_unsigned(28, 6);

                    when 28 =>                           -- the angle back to the state RAM
                        -- the picture column's constant: (128 + scroll) - (cx << sc) / 2
                        if f_large = '1' then v_sc3 := to_unsigned(6, 3) - f_rk(q_bi);
                        else                  v_sc3 := to_unsigned(0, 3) - f_rk(q_bi); end if;
                        case to_integer(v_sc3) is
                            when 0      => v_c9 := unsigned(q_cx(9 downto 1));
                            when 1      => v_c9 := unsigned(q_cx(8 downto 0));
                            when 2      => v_c9 := unsigned(q_cx(7 downto 0)) & "0";
                            when 3      => v_c9 := unsigned(q_cx(6 downto 0)) & "00";
                            when others => v_c9 := unsigned(q_cx(5 downto 0)) & "000";
                        end case;
                        f_cu(to_integer(q_bi)) <= f_off - v_c9;
                        st_we <= '1';
                        st_wa <= '0' & q_bi;
                        st_wd <= std_logic_vector(resize(q_th, 32));
                        if q_bi = 8 then
                            q_pc <= to_unsigned(43, 6);  -- line 0's tilted fields
                        else
                            q_bi <= q_bi + 1;
                            q_pc <= to_unsigned(5, 6);
                        end if;

                    ---------------------------------------------- line
                    when 40 =>                           -- how many virtual lines this line
                        v_ya   := l_yacc + resize(s_ys, 19);
                        l_yacc <= v_ya;
                        q_del  <= resize(v_ya(18 downto 8) - l_yv, 3);
                        l_yv   <= v_ya(18 downto 8);
                        l_first <= '0';
                        q_prime <= '1';
                        st_ra   <= to_unsigned(16, 5);   -- E of body 0
                        q_rc    <= (others => '0');
                        q_pc    <= to_unsigned(41, 6);

                    when 41 | 42 =>                      -- every E: read, step, write back
                        -- cycle t = q_rc: the RAM is addressed with body t; body
                        -- t - 2 arrives (st_q2) and is stepped with q_preg
                        if q_rc < 8 then st_ra <= st_ra + 1; end if;
                        q_rc <= q_rc + 1;
                        q_pc <= to_unsigned(42, 6);
                        if q_rc = 1 then
                            q_preg  <= v_fs;             -- body 0's step (head of the ring)
                            q_prime <= '0';
                        elsif q_rc >= 2 then
                            v_new := signed(st_q2(28 downto 0)) + resize(q_preg, 29);
                            st_we <= '1';
                            st_wa <= '1' & (q_rc - 2);
                            st_wd <= std_logic_vector(resize(v_new, 32));
                            v_dn := r_dy(0) + signed(resize(q_del & '0', 15));
                            for j in 0 to 7 loop
                                r_dy(j) <= r_dy(j + 1);
                            end loop;
                            r_dy(8) <= v_dn;
                            q_preg  <= v_fs;             -- the next body's step (slot 1)
                            if q_rc = 10 then q_pc <= to_unsigned(43, 6); end if;
                        end if;

                    when 43 =>                           -- U = (Yh - CY) / tilt
                        v_dyc := signed(resize(l_yv & '0', 16)) - resize(q_cyc, 16);
                        mul(v_dyc, f_kinv);
                        q_pc <= to_unsigned(44, 6);

                    when 44 =>
                        v_us := v_p(26 downto 8);
                        if v_us(18 downto 13) /= "000000" and v_us(18 downto 13) /= "111111" then
                            q_u <= to_unsigned(8191, 14);
                        else
                            q_u <= f_abs(v_us(13 downto 0));
                        end if;
                        q_pc <= to_unsigned(45, 6);

                    when 45 =>
                        mul(signed(resize(q_u, 15)), q_u);
                        q_pc <= to_unsigned(46, 6);

                    when 46 =>                           -- orbit field's line value
                        l_Ec <= f_xc0 + unsigned(v_p(27 downto 0));
                        mul(r_dy(6), f_kinv);            -- the same for Saturn's ring
                        q_pc <= to_unsigned(47, 6);

                    when 47 =>
                        v_us := v_p(26 downto 8);
                        if v_us(18 downto 13) /= "000000" and v_us(18 downto 13) /= "111111" then
                            q_u <= to_unsigned(8191, 14);
                        else
                            q_u <= f_abs(v_us(13 downto 0));
                        end if;
                        if r_dy(6) > 0 then l_sfront <= '1'; else l_sfront <= '0'; end if;
                        q_pc <= to_unsigned(48, 6);

                    when 48 =>
                        mul(signed(resize(q_u, 15)), q_u);
                        q_pc <= to_unsigned(49, 6);

                    when 49 =>
                        l_Er    <= f_cx6sq + unsigned(v_p(27 downto 0));
                        s_ready <= '1';
                        q_pc    <= (others => '0');

                    when others =>
                        q_pc <= (others => '0');
                end case;
            end if;
        end if;
    end process p_seq;

    -- ====================================================================
    -- S0: accumulators.  e += (2 xs X + xs^2) - 2 xs cx.  Each body's e is
    -- loaded when its line value is written to the state RAM, and holds it
    -- until the line starts.
    -- ====================================================================
    p_acc : process(clk)
        variable v_s : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            if r_run = '1' then
                for i in 0 to 8 loop
                    v_s := resize(p_Dg, 18) + f_D0(i);
                    p_e(i) <= p_e(i) + resize(v_s, 29);
                    if i = 6 then
                        p_er <= unsigned(signed(p_er) + resize(v_s, 28));
                    end if;
                end loop;
                v_s  := resize(p_Dg, 18) + f_kc;
                p_ec <= unsigned(signed(p_ec) + resize(v_s, 28));
                p_Dg <= p_Dg + signed(resize(s_c2, 17));
                p_px <= p_px + 1;
                p_x  <= p_x + resize(s_xs, 12);
            else
                if st_we = '1' and st_wa(4) = '1' then
                    for i in 0 to 8 loop
                        if st_wa(3 downto 0) = i then
                            p_e(i) <= signed(st_wd(28 downto 0));
                        end if;
                    end loop;
                end if;
                p_ec <= l_Ec;
                p_er <= l_Er;
                p_Dg <= signed(resize(s_x2, 17));
                p_px <= (others => '0');
                p_x  <= (others => '0');
            end if;
            p_px1 <= p_px;  p_px2 <= p_px1;
            p_x1  <= p_x;   p_x2  <= p_x1;
        end if;
    end process p_acc;

    -- orbit-line buffer: read the line above two pixels ahead of the write
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            if r_run2 = '1' then
                r_lb(to_integer(p_px2)) <= std_logic_vector(s2_cnt);
            end if;
            lb_q  <= r_lb(to_integer(p_px));
            lb_q2 <= lb_q;
        end if;
    end process p_lb;

    -- surface pictures and palettes (block RAM)
    p_rtx : process(clk) begin if rising_edge(clk) then q_tx <= C_OR_TEX(to_integer(s4_ta)); end if; end process;
    p_rpl : process(clk) begin if rising_edge(clk) then q_pl <= C_OR_PAL(to_integer(s6_pa)); end if; end process;

    -- ====================================================================
    -- Pixel pipeline S1 .. S12
    -- ====================================================================
    p_pix : process(clk)
        variable v_gl   : unsigned(27 downto 0);
        variable v_ers  : unsigned(27 downto 0);
        variable v_ring : std_logic;
        variable v_req  : std_logic_vector(9 downto 0);
        variable v_fw, v_bw : unsigned(3 downto 0);
        variable v_sel  : unsigned(3 downto 0);
        variable v_cnt  : unsigned(3 downto 0);
        variable v_a2   : unsigned(15 downto 0);
        variable v_sc3  : unsigned(2 downto 0);
        variable v_h    : unsigned(15 downto 0);
        variable v_yv   : unsigned(15 downto 0);
        variable v_ds   : signed(20 downto 0);
        variable v_sh   : signed(10 downto 0);
        variable v_tv   : signed(16 downto 0);
        variable v_u9   : unsigned(8 downto 0);
        variable v_v8   : unsigned(7 downto 0);
        variable v_t3   : unsigned(3 downto 0);
        variable v_star, v_belt, v_ex : unsigned(9 downto 0);
        variable v_ym   : signed(14 downto 0);
        variable v_cr, v_cb : signed(9 downto 0);
        variable v_shd  : unsigned(5 downto 0);
        variable v_bg   : unsigned(9 downto 0);
        variable v_sk   : unsigned(11 downto 0);
        variable v_g1   : unsigned(9 downto 0);
        variable v_c11  : signed(10 downto 0);
        variable v_f    : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            ---------------------------------------------------------------- S1
            -- the Sun's glow: 255 at its limb, falling with distance^2
            if p_e(0)(28) = '1' then
                v_gl := (others => '0');
            elsif f_large = '1' then
                v_gl := shift_right(unsigned(p_e(0)(27 downto 0)), 1);
            else
                v_gl := unsigned(p_e(0)(27 downto 0));
            end if;
            if v_gl(27 downto 17) /= 0 then s1_g8 <= (others => '0');
            else                            s1_g8 <= not v_gl(16 downto 9); end if;
            -- Saturn's ring: 1.41 r .. 2.24 r in the tilted field, with
            -- r^2 = 2048 (in units of 2 x field: 8192 .. 20480; the field stays
            -- below 2^27); rc = position across it, 0 .. 11
            if f_large = '1' then v_ers := shift_right(p_er, 3);
            else                  v_ers := p_er(26 downto 0) & '0'; end if;
            if f_rings = '1' and v_ers(27 downto 15) = 0
               and (v_ers(14 downto 13) = "01" or v_ers(14 downto 12) = "100") then
                v_ring := '1';
            else
                v_ring := '0';
            end if;
            s1_rc <= v_ers(14) & v_ers(12 downto 10);
            -- requests masked into the front group and the back group
            v_req(0) := v_ring and l_sfront;             -- ring over the ball
            for i in 1 to 8 loop
                v_req(i) := p_e(i)(28);
            end loop;
            v_req(9) := v_ring;                          -- ring behind the ball
            for i in 1 to 8 loop
                s1_rf(i) <= v_req(i) and f_front(i);
                s1_rb(i) <= v_req(i) and not f_front(i);
            end loop;
            s1_rf(0) <= v_req(0) and f_front(6);
            s1_rb(0) <= v_req(0) and not f_front(6);
            s1_rf(9) <= v_req(9) and f_front(6);
            s1_rb(9) <= v_req(9) and not f_front(6);
            s1_in0 <= p_e(0)(28) and s_ready;
            for i in 1 to 8 loop
                if p_ec(27 downto 24) = 0 and p_ec(23 downto 8) < f_a2(i) then s1_o(i) <= '1';
                else                                                         s1_o(i) <= '0'; end if;
            end loop;
            s1_ec <= p_ec(27 downto 8);

            ---------------------------------------------------------------- S2
            -- the front-most body: front group, then the Sun, then the back group
            v_fw := (others => '1');
            v_bw := (others => '1');
            for i in 9 downto 0 loop                     -- lowest index wins
                if s1_rf(i) = '1' then
                    if i = 0 or i = 9 then v_fw := to_unsigned(9, 4); else v_fw := to_unsigned(i, 4); end if;
                end if;
                if s1_rb(i) = '1' then
                    if i = 0 or i = 9 then v_bw := to_unsigned(9, 4); else v_bw := to_unsigned(i, 4); end if;
                end if;
            end loop;
            if s_ready = '0' then  v_sel := (others => '1');
            elsif v_fw /= 15 then  v_sel := v_fw;
            elsif s1_in0 = '1' then v_sel := (others => '0');
            else                   v_sel := v_bw;
            end if;
            s2_sel <= v_sel;
            v_cnt := (others => '0');
            for i in 1 to 8 loop
                if s1_o(i) = '1' then v_cnt := v_cnt + 1; end if;
            end loop;
            s2_cnt <= v_cnt;
            s2_ec  <= s1_ec;
            s2_g8  <= s1_g8;
            s2_rc  <= s1_rc;

            ---------------------------------------------------------------- S3
            if s2_sel >= 1 and s2_sel <= 8 then
                v_a2  := f_a2(to_integer(s2_sel));
                s3_sh <= f_sh(to_integer(s2_sel));
                s3_cu <= f_cu(to_integer(s2_sel));
                s3_dy <= r_dy(to_integer(s2_sel))(12 downto 0);
            else
                v_a2  := (others => '0');
                s3_sh <= (others => '0');
                s3_cu <= (others => '0');
                s3_dy <= (others => '0');
            end if;
            s3_d <= signed(resize(v_a2, 21)) - signed(resize(s2_ec, 21));     -- > 0 sunward
            -- the shift that takes the picked body's radius to 128 (sc - 1),
            -- applied to X here; the body's own constant is added in S4
            if f_large = '1' then v_sc3 := to_unsigned(6, 3) - f_rk(s2_sel);
            else                  v_sc3 := to_unsigned(0, 3) - f_rk(s2_sel); end if;   -- 8 - k (mod 8)
            s3_sc <= v_sc3;
            case to_integer(v_sc3) is
                when 0      => s3_xu <= p_x2(9 downto 1);
                when 1      => s3_xu <= p_x2(8 downto 0);
                when 2      => s3_xu <= p_x2(7 downto 0) & "0";
                when 3      => s3_xu <= p_x2(6 downto 0) & "00";
                when others => s3_xu <= p_x2(5 downto 0) & "000";
            end case;
            -- orbit line: the orbit count changes from the left or from above
            if (s2_cnt /= s3_cntl and p_px2 /= 0)
               or (s2_cnt /= unsigned(lb_q2) and l_first = '0') then
                s3_line <= s_ready;
            else
                s3_line <= '0';
            end if;
            s3_cntl <= s2_cnt;
            s3_cnt  <= s2_cnt;
            s3_sel  <= s2_sel;
            s3_g8   <= s2_g8;
            s3_rc   <= s2_rc;
            -- hash of the virtual pixel, first half (stars and belt)
            v_yv := resize(l_yv, 16);
            s3_h <= resize(p_x2(11 downto 1), 16) + shift_left(v_yv, 5) + shift_left(v_yv, 2)
                    + to_unsigned(2531, 16);
            s3_xv <= p_x2(11 downto 1);

            ---------------------------------------------------------------- S4
            -- the sunward difference (A^2 - ec) >> n, clamped -128 .. 127
            v_ds := shift_right(s3_d, to_integer(s3_sh));
            if v_ds(20) = '0' and v_ds(19 downto 7) /= 0 then
                s4_ds <= to_signed(127, 9);
            elsif v_ds(20) = '1' and v_ds(19 downto 7) /= to_signed(-1, 13) then
                s4_ds <= to_signed(-128, 9);
            else
                s4_ds <= v_ds(8 downto 0);
            end if;
            v_yv  := resize(l_yv, 16);
            v_h   := s3_h xor shift_right(s3_h, 3) xor shift_left(v_yv, 7);
            v_h   := v_h + shift_left(v_h, 4) + shift_left(resize(s3_xv, 16), 9);
            s4_h  <= v_h xor shift_right(v_h, 7);
            -- picture address: column 0 .. 63 (scrolling), row 0 .. 31
            v_u9 := s3_xu + s3_cu;
            case to_integer(s3_sc) is
                when 0      => v_tv := resize(s3_dy, 17);
                when 1      => v_tv := shift_left(resize(s3_dy, 17), 1);
                when 2      => v_tv := shift_left(resize(s3_dy, 17), 2);
                when 3      => v_tv := shift_left(resize(s3_dy, 17), 3);
                when others => v_tv := shift_left(resize(s3_dy, 17), 4);
            end case;
            if v_tv(16) = '0' and v_tv(15 downto 8) /= 0 then
                v_v8 := (others => '1');
            elsif v_tv(16) = '1' and v_tv(15 downto 8) /= to_signed(-1, 8) then
                v_v8 := (others => '0');
            else
                v_v8 := unsigned(v_tv(8 downto 1)) xor "10000000";
            end if;
            if s3_sel >= 1 and s3_sel <= 8 then v_t3 := s3_sel - 1;
            else                                v_t3 := (others => '0'); end if;
            s4_ta   <= v_t3(2 downto 0) & v_v8(7 downto 3) & v_u9(8 downto 3);
            s4_line <= s3_line;
            s4_cnt  <= s3_cnt;
            s4_sel  <= s3_sel;
            s4_g8   <= s3_g8;
            s4_rc   <= s3_rc;

            ---------------------------------------------------------------- S5
            -- shade 32 .. 255 across the ball; the Sun, the ring, Flat
            v_sh := to_signed(160, 11) + resize(s4_ds, 11);
            if v_sh(8) = '1' then v_sh := to_signed(255, 11); end if;
            if f_flat = '1' then v_sh := to_signed(200, 11); end if;
            if s4_sel = 0 then v_sh := to_signed(255, 11); end if;
            if s4_sel = 9 then v_sh := to_signed(200, 11); end if;
            -- sky extras: stars, the belt, the orbit lines
            if (s4_h(9 downto 0) and f_stm) = 0 then
                v_star := to_unsigned(96, 10) + resize(s4_h(15 downto 10) & "00", 10);
            else
                v_star := (others => '0');
            end if;
            if f_belt = '1' and s4_cnt = 4 and s4_h(5 downto 0) = 5 then
                v_belt := to_unsigned(150, 10);
            else
                v_belt := (others => '0');
            end if;
            if v_belt > v_star then v_ex := v_belt; else v_ex := v_star; end if;
            if s4_line = '1' then v_ex := v_ex + resize(f_ol, 10); end if;
            c_bus(5)(5 downto 0)   <= std_logic_vector(v_sh(7 downto 2));     -- shade / 4
            c_bus(5)(15 downto 6)  <= std_logic_vector(v_ex);
            c_bus(5)(20 downto 16) <= std_logic_vector(s4_g8(7 downto 3));    -- glow / 8
            if s4_sel = 15 then c_bus(5)(21) <= '0'; else c_bus(5)(21) <= '1'; end if;
            s5_sel <= s4_sel;
            s5_rc  <= s4_rc;

            ---------------------------------------------------------------- S6
            -- palette address: the body with its picture's index (q_tx), the
            -- Sun's one colour, or the ring with its position
            if s5_sel = 9 then                       s6_pa <= "1001" & s5_rc;
            elsif s5_sel = 0 or s5_sel = 15 then     s6_pa <= (others => '0');
            else                                     s6_pa <= s5_sel & unsigned(q_tx);
            end if;
            for i in 6 to 8 loop
                c_bus(i) <= c_bus(i - 1);
            end loop;

            ---------------------------------------------------------------- S7, S8
            s8_pal <= q_pl;

            ---------------------------------------------------------------- S9
            -- colour x shade: Y by gated shift-adds (64 shade levels), chroma
            -- in four steps (invisible at 4:2:2)
            v_shd := unsigned(c_bus(8)(5 downto 0));
            v_ym  := f_mulw(signed(resize(unsigned(s8_pal(21 downto 14)), 9)), v_shd);
            s9_yb <= unsigned(v_ym(13 downto 4));        -- 0 .. 689
            v_cr := resize(signed(s8_pal(13 downto 7)), 10);
            v_cb := resize(signed(s8_pal(6 downto 0)), 10);
            case v_shd(5 downto 4) is
                when "00"   => s9_cu <= shift_left(v_cr, 1);                       s9_cb <= shift_left(v_cb, 1);
                when "01"   => s9_cu <= shift_left(v_cr, 2);                       s9_cb <= shift_left(v_cb, 2);
                when "10"   => s9_cu <= shift_left(v_cr, 2) + shift_left(v_cr, 1); s9_cb <= shift_left(v_cb, 2) + shift_left(v_cb, 1);
                when others => s9_cu <= shift_left(v_cr, 3);                       s9_cb <= shift_left(v_cb, 3);
            end case;
            -- sky: black or the live picture, plus the extras and the glow
            if f_sky = '1' and d_av(4) = '1' then v_bg := d_y(4);
            else                                  v_bg := C_BLK; end if;
            v_sk := resize(v_bg, 12) + resize(unsigned(c_bus(8)(15 downto 6)), 12)
                    + resize(unsigned(c_bus(8)(20 downto 16)) & "000", 12);
            if v_sk > 940 then s9_sky <= to_unsigned(940, 10);
            else               s9_sky <= v_sk(9 downto 0); end if;
            s9_hit <= c_bus(8)(21);
            s9_g8  <= unsigned(c_bus(8)(20 downto 16)) & "000";

            ---------------------------------------------------------------- S10
            -- a body, or the sky
            v_g1 := resize(shift_right(s9_g8, 1), 10);
            if s9_hit = '1' then                         -- u carries Cr, v carries Cb
                s10_y <= C_BLK + s9_yb;
                v_c11 := to_signed(512, 11) + resize(s9_cu, 11);
                s10_u <= unsigned(v_c11(9 downto 0));
                v_c11 := to_signed(512, 11) + resize(s9_cb, 11);
                s10_v <= unsigned(v_c11(9 downto 0));
            else
                s10_y <= s9_sky;
                if f_sky = '1' and d_av(5) = '1' then
                    s10_u <= d_u(5);
                    s10_v <= d_v(5);
                else                                     -- the glow is orange
                    s10_u <= C_MID + v_g1;
                    s10_v <= C_MID - v_g1;
                end if;
            end if;

            ---------------------------------------------------------------- S11, S12
            -- horizontal [1 2 1] / 4 on Y, U, V (centre = the pixel one
            -- behind), straight into the gated output register
            w0_y <= s10_y;  w0_u <= s10_u;  w0_v <= s10_v;
            w1_y <= w0_y;   w1_u <= w0_u;   w1_v <= w0_v;
            if d_gv(C_LAT - 1) = '0' then                -- blanking gate
                s_io.y <= std_logic_vector(C_BLK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                v_f := resize(s10_y, 12) + shift_left(resize(w0_y, 12), 1) + resize(w1_y, 12);
                s_io.y <= std_logic_vector(v_f(11 downto 2));
                v_f := resize(s10_u, 12) + shift_left(resize(w0_u, 12), 1) + resize(w1_u, 12);
                s_io.u <= std_logic_vector(v_f(11 downto 2));
                v_f := resize(s10_v, 12) + shift_left(resize(w0_v, 12), 1) + resize(w1_v, 12);
                s_io.v <= std_logic_vector(v_f(11 downto 2));
            end if;
            s_io.hsync_n <= d_hs(C_LAT - 1);
            s_io.vsync_n <= d_vs(C_LAT - 1);
            s_io.field_n <= d_fl(C_LAT - 1);
            s_io.avid    <= d_gv(C_LAT - 1);
        end if;
    end process p_pix;

    -- ====================================================================
    -- Sync / video delay
    -- ====================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_y(1)  <= unsigned(data_in.y);
            d_u(1)  <= unsigned(data_in.u);
            d_v(1)  <= unsigned(data_in.v);
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_av(1) <= data_in.avid;
            d_gv(1) <= g_avid;
            for i in 2 to C_LAT - 1 loop
                d_y(i)  <= d_y(i - 1);
                d_u(i)  <= d_u(i - 1);
                d_v(i)  <= d_v(i - 1);
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_av(i) <= d_av(i - 1);
                d_gv(i) <= d_gv(i - 1);
            end loop;
        end if;
    end process p_delay;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture orrery;
