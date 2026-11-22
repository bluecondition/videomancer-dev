-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lissajous v0.1.1 -- Patchbay: an X-Y scope figure rescanned as a raster.
--
-- The patch:
--   H ramp -> arcsine shaper -> function generator A   Cx  = cos(a * acos(x) + luma)
--   V ramp -> arcsine shaper -> function generator B   Cy+ = cos(b * acos(y) + phi), Cy- = cos(b * acos(y) - phi)
--   F+ = Cx - Cy+, F- = Cx - Cy-          two difference amplifiers (the front and the back strands)
--   window comparators at F+ = 0 and F- = 0 draw the figure x = cos(b t + phi), y = cos(a t): its two strand
--   families slide opposite ways as phi drifts, so the figure turns as if in 3-D
--   |F+| and |F-| -> non-additive mixer (the smaller) -> colouriser: the glow bands around the figure
--   Twin: the two summing amplifiers Cx + Cy+ and Cx + Cy- (generator B inverted) give the figure half a
--   turn away (its mirror), in cyan
--   The line is |F| < w * |grad F|: the gradient is measured from the picture itself (Cx 4 px either side,
--   Cy one line up, the luma slew's line-to-line step), so the weight stays even and the camera can bend it.
--
--   K1  A        function generator A's multiple, 1 .. 8 (a detent at every integer: the figure closes)
--   K2  B        function generator B's multiple, 1 .. 8
--   K3  Glow     the colouriser: a tight halo .. many bands reaching far
--   K4  Speed    phi drift, bipolar: the figure turns one way or the other
--   K5  Colour   colouriser rotate
--   K6  Camera   the camera / Osc level into generator A's phase, x round(K6 / 128) / 2 (8 steps: 25 % = v0.1.0,
--                50 % = twice, 100 % = five times (x4.5 at 7/8; +25 % at the top since 2026-11-15)); Line (the old
--                K5) is fixed at 520
--   S7  Shaper   Arcsine (the true figure in a square) / Linear (a bare lattice of diamonds, whole screen)
--   S8  Twin     the figure's mirror twin (phi + half a turn), cyan with a dark edge; the bands hug both
--   S9  Frame    Square / Wide (stretched to the screen)
--   S10 Look     Bands / Line (the figure alone on black, with a bloom in the colouriser's colour)
--   S11 Source   Video (luma into generator A's phase: the figure bends through the picture) / Osc
--   P12 DETUNE   0 = locked ratio .. 100 % = two LFOs sweep a and b through the whole family
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine run at every line end
-- (it computes two lines ahead: the V shaper, b * theta, two interpolated cosines and their line-to-line
-- steps, sharing the two table ports with the pixel path in hblank), a 30-stage pixel path (x DDA, arcsine
-- table, a * theta multiply, cosine table, finite-difference gradient, four window comparators, the
-- non-additive mix and the colouriser).  Virtual 1080-line coordinates.  Generated avid; output conditioning
-- (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16 as running 8-sums that resync to neutral every blanking
-- interval, luma 13/16) before the blanking gate.  Latency 36 clocks in every mode.  Colours in the hardware
-- convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Line fixed at 520 (C_KFIX_5).
-- 2026-11-15: K6 camera top +25 % (Ron): the camera gain step code tops out at x5 (was x4), x4.5 on the step below.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lissajous_pkg.all;
use work.all;

architecture lissajous of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(520, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 36;   -- = processing_delay_clks

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
    -- line pitch against pixel pitch: 0 = square pixels, 1 = 7/8 (480p), 2 = 1/2 (interlaced fields)
    function f_rho(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return "10";
            when 4                  => return "01";
            when others             => return "00";
        end case;
    end function;
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- the oscillator's line step: 121 virtual lines per 256, times the raster's line pitch
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(537, 10);
            when 4          => return to_unsigned(272, 10);
            when 8          => return to_unsigned(453, 10);
            when 12         => return to_unsigned(226, 10);
            when 5 | 6 | 14 => return to_unsigned(181, 10);
            when 1 | 2 | 10 => return to_unsigned(242, 10);
            when others     => return to_unsigned(121, 10);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- ratio knob (glided, Q3): nearest integer step (1168 per unit) and its base
    function f_rn(u : unsigned(12 downto 0)) return unsigned is
        variable n : unsigned(2 downto 0);
    begin
        n := "000";
        if u >= 584 then n := "001"; end if;
        if u >= 1752 then n := "010"; end if;
        if u >= 2920 then n := "011"; end if;
        if u >= 4088 then n := "100"; end if;
        if u >= 5256 then n := "101"; end if;
        if u >= 6424 then n := "110"; end if;
        if u >= 7592 then n := "111"; end if;
        return n;
    end function;
    function f_rbase(n : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(n) is
            when 0      => return to_unsigned(0, 14);
            when 1      => return to_unsigned(1168, 14);
            when 2      => return to_unsigned(2336, 14);
            when 3      => return to_unsigned(3504, 14);
            when 4      => return to_unsigned(4672, 14);
            when 5      => return to_unsigned(5840, 14);
            when 6      => return to_unsigned(7008, 14);
            when others => return to_unsigned(8176, 14);
        end case;
    end function;
    -- LFO: 12-bit phase -> triangle, sign and magnitude (0 .. 1023), rising through zero at phase 0
    function f_trim(p : unsigned(11 downto 0)) return unsigned is
    begin
        if p(10) = '1' then return not p(9 downto 0); else return p(9 downto 0); end if;
    end function;

    -- tables (one read port each, registered address and data)
    signal t_ra   : unsigned(9 downto 0) := (others => '0');
    signal r_ac_q : unsigned(15 downto 0) := (others => '0');
    signal c_ra   : unsigned(9 downto 0) := (others => '0');
    signal r_cq   : unsigned(15 downto 0) := (others => '0');
    signal r_pa   : unsigned(8 downto 0) := (others => '0');
    signal r_pq   : std_logic_vector(31 downto 0) := (others => '0');

    -- line store (pair resolution): the luma's vertical slew state (Q3)
    type t_ram16 is array (0 to 1023) of std_logic_vector(15 downto 0);
    signal lv : t_ram16 := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lv : signal is true;
    signal lv_ra, lv_wa : unsigned(9 downto 0) := (others => '0');
    signal lv_q, lv_f, lv_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal lv_we : std_logic := '0';

    -- delay rings (256 deep block RAM, written every clock at rc + delay - 2, read at rc, RAM register,
    -- fabric register): Cx 8 back, the chroma box filter's 8-back samples, the syncs C_LAT back
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_ring4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal rc  : unsigned(7 downto 0) := (others => '0');
    signal xr, cru, crv : t_ring16 := (others => (others => '0'));
    signal sr  : t_ring4 := (others => (others => '1'));
    attribute no_rw_check of xr : signal is true;
    attribute no_rw_check of cru : signal is true;
    attribute no_rw_check of crv : signal is true;
    attribute no_rw_check of sr : signal is true;
    signal r_xr, r_cru, r_crv : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sr, s_so : std_logic_vector(3 downto 0) := (others => '1');
    signal s_csu7, s_csv7 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- source side
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_yin  : unsigned(11 downto 0) := (others => '0');
    signal s_yin0 : unsigned(11 downto 0) := (others => '0');   -- K6: input level before the camera gain
    signal s_ya   : unsigned(13 downto 0) := (others => '0');   --     gain terms: 0 / y / 2y / 4y
    signal s_yb   : unsigned(11 downto 0) := (others => '0');   --                 + 0 / y/2 / y
    signal c_cga  : unsigned(1 downto 0) := "10";                --     their selects (per field; default x 2)
    signal c_cgb  : unsigned(1 downto 0) := "00";
    signal s_h1, s_h2 : unsigned(15 downto 0) := (others => '0');
    signal s_wd1, s_wd2, s_wd3 : unsigned(10 downto 0) := (others => '0');
    signal s_av1, s_av2, s_av3 : std_logic := '0';
    signal s_l13  : unsigned(11 downto 0) := (others => '0');
    signal s_loff : unsigned(12 downto 0) := (others => '0');
    signal s_gyl  : unsigned(10 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_rho  : unsigned(1 downto 0) := "00";
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(292 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(146 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(420 * 8, 13);
    signal s_gc : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 Camera (the freed Line channel)
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(260 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_lin, s_twin, s_wide, s_mono, s_src : std_logic := '0';
    signal z_first : std_logic := '1';

    -- per-field values
    signal z_u    : unsigned(12 downto 0) := (others => '0');
    signal z_n, z_nn : unsigned(2 downto 0) := (others => '0');
    signal z_d    : signed(14 downto 0) := (others => '0');
    signal z_a    : unsigned(9 downto 0) := (others => '0');
    signal z_f    : unsigned(9 downto 0) := (others => '0');
    signal z_neg, z_ng2 : std_logic := '0';
    signal z_n3, z_n4 : unsigned(2 downto 0) := (others => '0');
    signal z_t1m, z_t2m : unsigned(9 downto 0) := (others => '0');
    signal z_t1s, z_t2s : std_logic := '0';
    signal z_av   : signed(15 downto 0) := (others => '0');
    signal z_bv   : unsigned(11 downto 0) := (others => '0');
    signal z_kk   : unsigned(7 downto 0) := (others => '0');
    signal z_gqv  : unsigned(7 downto 0) := to_unsigned(40, 8);
    signal z_wv   : unsigned(7 downto 0) := to_unsigned(84, 8);
    signal z_wp   : unsigned(7 downto 0) := to_unsigned(84, 8);
    signal z_x    : unsigned(23 downto 0) := (others => '0');
    signal z_km   : unsigned(9 downto 0) := (others => '0');
    signal z_ks   : std_logic := '0';
    signal z_r1, z_r2 : unsigned(7 downto 0) := (others => '0');
    signal z_p1, z_p2 : unsigned(18 downto 0) := (others => '0');
    signal c_ar, c_br : unsigned(13 downto 0) := to_unsigned(3072, 14);   -- the knobs' ratios (Q10)
    signal c_dd   : unsigned(11 downto 0) := (others => '0');             -- detune depth (Q10)
    signal c_a    : unsigned(13 downto 0) := to_unsigned(3072, 14);       -- a, b (Q10)
    signal c_b    : unsigned(13 downto 0) := to_unsigned(2048, 14);
    signal c_lf1  : unsigned(19 downto 0) := to_unsigned(136314, 20);     -- detune LFO phases
    signal c_lf2  : unsigned(19 downto 0) := to_unsigned(399507, 20);
    signal c_phi  : unsigned(23 downto 0) := to_unsigned(3456106, 24);    -- phi (Q24 cycles)
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_dphi : unsigned(18 downto 0) := (others => '0');
    signal c_ge   : unsigned(1 downto 0) := "01";                         -- colouriser gain: exponent
    signal c_gm   : unsigned(4 downto 0) := to_unsigned(20, 5);           --   and mantissa
    signal c_fi   : unsigned(2 downto 0) := "010";                        -- falloff setting
    signal c_rot8 : unsigned(7 downto 0) := (others => '0');              -- colouriser rotate (Q2 positions)
    signal c_wpx  : unsigned(7 downto 0) := to_unsigned(84, 8);           -- line half-width, real px (Q4)
    signal c_wm   : unsigned(3 downto 0) := to_unsigned(10, 4);           --   as mantissa (8 .. 15)
    signal c_we   : unsigned(1 downto 0) := "10";                         --   and exponent
    signal c_wln  : unsigned(7 downto 0) := to_unsigned(84, 8);           -- line half-width, real lines (Q4)
    signal c_xstep : unsigned(12 downto 0) := to_unsigned(2056, 13);      -- table index per pixel (Q10)
    signal c_xstart : signed(23 downto 0) := (others => '0');
    signal c_off13 : unsigned(12 downto 0) := (others => '0');            -- linear shaper's phase offset
    signal c_ky   : unsigned(11 downto 0) := to_unsigned(1028, 12);       -- V shaper scale

    -- field products (they run on the line engine's serial multiplier, idle in the vblank zone)
    signal z_fa  : unsigned(12 downto 0) := (others => '0');
    signal z_fb  : unsigned(11 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
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

    -- line engine
    type t_ph2 is array (0 to 1) of unsigned(15 downto 0);
    type t_s17x4 is array (0 to 3) of signed(16 downto 0);
    type t_s17x2 is array (0 to 1) of signed(16 downto 0);
    type t_s12x2 is array (0 to 1) of signed(11 downto 0);
    type t_u15x2 is array (0 to 1) of unsigned(14 downto 0);
    type t_u14x2 is array (0 to 1) of unsigned(13 downto 0);
    signal e_cnt  : unsigned(6 downto 0) := (others => '1');
    signal e_evn  : unsigned(1 downto 0) := (others => '0');
    signal e_zk, e_zk0 : std_logic := '0';
    signal e_own  : std_logic := '0';
    signal e_i12  : unsigned(11 downto 0) := (others => '0');
    signal e_ya   : unsigned(9 downto 0) := (others => '0');
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_p    : t_ph2 := (others => (others => '0'));
    signal e_c    : t_s17x4 := (others => (others => '0'));
    signal e_pv   : t_s17x2 := (others => (others => '0'));
    signal e_th   : unsigned(12 downto 0) := (others => '0');
    signal e_thy  : unsigned(13 downto 0) := (others => '0');
    signal e_cy   : signed(16 downto 0) := (others => '0');
    signal e_dn   : std_logic := '0';
    signal e_ph16 : unsigned(15 downto 0) := (others => '0');
    signal e_onx  : std_logic := '0';
    signal y3_q2  : unsigned(12 downto 0) := (others => '0');
    signal y3_neg : std_logic := '0';
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(13 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal r_cy, n_cy   : t_s12x2 := (others => (others => '0'));
    signal r_w, n_w     : t_u15x2 := (others => (others => '0'));
    signal r_w38, n_w38 : t_u14x2 := (others => (others => '0'));
    signal r_outy, n_outy, r_nol, n_nol : std_logic := '0';
    signal e_onl  : std_logic := '0';

    -- pixel path
    type t_u16x2 is array (0 to 1) of unsigned(15 downto 0);
    type t_s13x2 is array (0 to 1) of signed(12 downto 0);
    type t_u12x2 is array (0 to 1) of unsigned(11 downto 0);
    type t_u2x2 is array (0 to 1) of unsigned(1 downto 0);
    signal x_acc  : signed(23 downto 0) := (others => '0');
    signal s2_q2, s3_q2, s4_q2 : unsigned(12 downto 0) := (others => '0');
    signal s2_neg, s3_neg, s4_neg : std_logic := '0';
    signal s2_out : std_logic := '0';
    signal s_osr  : std_logic_vector(26 downto 3) := (others => '0');     -- outside flag by stage (x; x or y from 23)
    signal s4_t   : unsigned(12 downto 0) := (others => '0');
    signal s5_th  : unsigned(13 downto 0) := (others => '0');
    signal s6_p0, s6_p1, s6_p2 : unsigned(17 downto 0) := (others => '0');
    signal s7_ph  : unsigned(12 downto 0) := (others => '0');
    signal s8_sg, s9_sg, s10_sg : std_logic := '0';
    signal s10_m  : unsigned(15 downto 0) := (others => '0');
    signal cx11, cx19, cx20 : signed(11 downto 0) := (others => '0');
    signal s16_dc : signed(12 downto 0) := (others => '0');
    signal s17_gx : unsigned(10 downto 0) := (others => '0');
    signal s18_g  : unsigned(11 downto 0) := (others => '0');
    signal s19_x  : unsigned(15 downto 0) := (others => '0');
    signal s20_xp : unsigned(14 downto 0) := (others => '0');
    signal s21_t, s22_t : t_u16x2 := (others => (others => '0'));
    signal s21_f, s21_g : t_s13x2 := (others => (others => '0'));
    signal s22_af, s22_at : t_u12x2 := (others => (others => '0'));
    signal s23_lv, s23_lt : t_u2x2 := (others => (others => '0'));
    signal s23_sk : std_logic_vector(1 downto 0) := "00";
    signal s23_m01, s23_m23, s24_s : unsigned(11 downto 0) := (others => '0');
    signal s24_cv, s25_cv, s26_cv, s27_cv, s28_cv, s29_cv : unsigned(1 downto 0) := (others => '0');
    signal s24_tg, s25_tg, s26_tg, s27_tg, s28_tg, s29_tg : std_logic := '0';
    signal s24_sk, s25_sk, s26_sk : std_logic := '0';
    signal s25_pa : unsigned(15 downto 0) := (others => '0');
    signal s25_pb : unsigned(11 downto 0) := (others => '0');
    signal s25_tier, s26_tier : unsigned(2 downto 0) := (others => '0');
    signal s26_pos : unsigned(10 downto 0) := (others => '0');
    signal s29_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y30  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);

    constant c_kky : unsigned(9 downto 0) := unsigned(C_LJ_K(31 downto 22));   -- black
    constant c_kku : unsigned(9 downto 0) := unsigned(C_LJ_K(21 downto 12));
    constant c_kkv : unsigned(9 downto 0) := unsigned(C_LJ_K(11 downto 2));

    -- output conditioning
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 box inputs
    signal s_y31, s_y32, s_y33, s_y34, s_y35, s_y36x, s_yf36 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u36, s_v36 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y37 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u37, s_v37 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Tables and line stores
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ac_q <= C_LJ_ACOS(to_integer(t_ra));
            r_cq   <= C_LJ_COS(to_integer(c_ra));
            r_pq   <= C_LJ_PAL(to_integer(r_pa));
        end if;
    end process p_rom;

    p_lv : process(clk)
    begin
        if rising_edge(clk) then
            lv_q <= lv(to_integer(lv_ra));
            if lv_we = '1' then lv(to_integer(lv_wa)) <= lv_wd; end if;
        end if;
    end process p_lv;

    -- rings: the read data is the write data of (offset + 2) clocks before; never the write address
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rc <= rc + 1;
            xr(to_integer(rc + 6)) <= "0000" & std_logic_vector(cx11);
            r_xr <= xr(to_integer(rc));
            cru(to_integer(rc + 6)) <= "000000" & std_logic_vector(s_u30);
            crv(to_integer(rc + 6)) <= "000000" & std_logic_vector(s_v30);
            r_cru <= cru(to_integer(rc));
            r_crv <= crv(to_integer(rc));
            s_csu7 <= unsigned(r_cru(9 downto 0));
            s_csv7 <= unsigned(r_crv(9 downto 0));
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            r_sr <= sr(to_integer(rc));
            s_so <= r_sr;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_ab : unsigned(14 downto 0);
        variable v_r  : unsigned(13 downto 0);
        variable v_dq : unsigned(9 downto 0);
        variable v_22 : unsigned(21 downto 0);
        variable v_8  : unsigned(7 downto 0);
        variable v_24a : unsigned(12 downto 0);
        variable v_f16 : signed(15 downto 0);
        variable v_k7 : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_rho <= f_rho(s_tid);
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
                s_lin  <= registers_in(6)(0);
                s_twin <= registers_in(6)(1);
                s_wide <= registers_in(6)(2);
                s_mono <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                if registers_in(6)(2) = '1' then c_ky <= to_unsigned(971, 12); else c_ky <= to_unsigned(1028, 12); end if;
                case to_integer(s_xsr) is
                    when 4      =>
                        s_oxs <= to_unsigned(204, 9);
                        if registers_in(6)(2) = '1' then c_xstep <= to_unsigned(2913, 13); else c_xstep <= to_unsigned(4112, 13); end if;
                    when 3      =>
                        s_oxs <= to_unsigned(153, 9);
                        if registers_in(6)(2) = '1' then c_xstep <= to_unsigned(1638, 13); else c_xstep <= to_unsigned(3084, 13); end if;
                    when others =>
                        s_oxs <= to_unsigned(102, 9);
                        if registers_in(6)(2) = '1' then c_xstep <= to_unsigned(1092, 13); else c_xstep <= to_unsigned(2056, 13); end if;
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters.  The first field after
            -- configuration loads the knobs directly.
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_gc; s_gr <= unsigned(registers_in(5));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(4));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if z_first = '1' then
                            null;
                        elsif (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) or s_gd > 32 or s_gd < -32 then   -- Camera: +-4
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
                            when 3      => s_gc <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; z_first <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 12 iterations, stored at load + 18.  The
            -- slot number is decoded at 30 + 24 k and the operands are pre-muxed into z_fa / z_fb at
            -- 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 3); z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 3); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 3); z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 3); z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 3); z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 3); z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 3); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_gp(12 downto 3), 13); z_fb <= resize(s_gp(12 downto 3), 12);  -- detune^2
                    when 1  => z_fa <= resize(c_dd, 13);               z_fb <= resize(z_t1m, 12);             -- depth x LFO 1
                    when 2  => z_fa <= resize(c_dd, 13);               z_fb <= resize(z_t2m, 12);             -- depth x LFO 2
                    when 3  => z_fa <= resize(s_vm, 13);               z_fb <= resize(s_vm, 12);              -- speed^2
                    when 4  => z_fa <= resize(s_g3(12 downto 3), 13); z_fb <= resize(s_g3(12 downto 3), 12);  -- glow^2
                    when 5  => z_fa <= (others => '0');               z_fb <= (others => '0');               -- (was line^2; Line is fixed)
                    when others => z_fa <= c_xstep;                    z_fb <= resize(s_w(11 downto 1), 12);  -- x origin
                end case;
            end if;

            -- ratio knob -> ratio (Q10), a free-running four-step pipe from z_u (held two clocks): the nearest integer,
            -- a +-18-step detent around it, 1.16 x the rest
            z_n <= f_rn(z_u);
            z_d <= signed(resize(z_u, 15)) - signed(resize(f_rbase(z_n), 15)); z_nn <= z_n;
            v_ab := unsigned(abs(z_d));
            if v_ab <= 144 then z_a <= (others => '0'); else z_a <= resize(v_ab - 144, 10); end if;
            z_ng2 <= z_d(14); z_n3 <= z_nn;
            z_f <= z_a + shift_right(z_a, 3) + shift_right(z_a, 5);
            z_neg <= z_ng2; z_n4 <= z_n3;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 26 => z_u <= s_g1;
                    when 27 =>
                        -- the detune LFOs (they run faster with Detune)
                        v_dq := s_gp(12 downto 3);
                        c_lf1 <= c_lf1 + (to_unsigned(768, 20) + resize(v_dq, 20) + resize(shift_right(v_dq, 1), 20));
                        c_lf2 <= c_lf2 + (to_unsigned(472, 20) + resize(v_dq, 20) - resize(shift_right(v_dq, 4), 20));
                    when 28 =>
                        z_u <= s_g2;
                        z_t1m <= f_trim(c_lf1(19 downto 8)); z_t1s <= c_lf1(19);
                        z_t2m <= f_trim(c_lf2(19 downto 8)); z_t2s <= c_lf2(19);
                    when 31 =>            -- K1 -> ratio A
                        v_r := shift_left(resize(z_n4, 14) + 1, 10);
                        if z_neg = '1' then c_ar <= v_r - resize(z_f, 14); else c_ar <= v_r + resize(z_f, 14); end if;
                    when 33 =>            -- K2 -> ratio B
                        v_r := shift_left(resize(z_n4, 14) + 1, 10);
                        if z_neg = '1' then c_br <= v_r - resize(z_f, 14); else c_br <= v_r + resize(z_f, 14); end if;
                    -- detune depth = 3 d^2 (Q10)
                    when 50 =>
                        v_22 := resize(m_acc(19 downto 0), 22) + shift_left(resize(m_acc(19 downto 0), 22), 1);
                        c_dd <= v_22(21 downto 10);
                    -- a = A + depth * LFO 1, folded at 1
                    when 74 =>
                        if z_t1s = '1' then z_av <= signed(resize(c_ar, 16)) - signed(resize(m_acc(21 downto 10), 16));
                        else z_av <= signed(resize(c_ar, 16)) + signed(resize(m_acc(21 downto 10), 16)); end if;
                    when 75 =>
                        v_f16 := to_signed(2048, 16) - z_av;
                        if z_av < 1024 then c_a <= unsigned(v_f16(13 downto 0));
                        else c_a <= unsigned(z_av(13 downto 0)); end if;
                    when 76 =>            -- Linear: the phase is a * (theta - 8192), so the lattice is even about the centre
                        v_24a := shift_left(resize(c_a(13 downto 2), 13), 4);
                        if s_lin = '1' then c_off13 <= (not v_24a) + 1; else c_off13 <= (others => '0'); end if;
                    -- b = B - 3/4 depth * LFO 2, folded at 1
                    when 98 => z_bv <= m_acc(21 downto 10) - shift_right(m_acc(21 downto 10), 2);
                    when 99 =>
                        if z_t2s = '1' then z_av <= signed(resize(c_br, 16)) + signed(resize(z_bv, 16));
                        else z_av <= signed(resize(c_br, 16)) - signed(resize(z_bv, 16)); end if;
                    when 100 =>
                        v_f16 := to_signed(2048, 16) - z_av;
                        if z_av < 1024 then c_b <= unsigned(v_f16(13 downto 0));
                        else c_b <= unsigned(z_av(13 downto 0)); end if;
                    -- phi step = 1.328 speed^2
                    when 122 => c_vsq <= m_acc(17 downto 0);
                    when 123 =>
                        z_p1 <= resize(c_vsq, 19) + resize(shift_right(c_vsq, 2), 19);
                        z_p2 <= resize(shift_right(c_vsq, 4), 19) + resize(shift_right(c_vsq, 6), 19);
                    when 124 => c_dphi <= z_p1 + z_p2;
                    when 125 =>
                        if s_vneg = '1' then c_phi <= c_phi - resize(c_dphi, 24);
                        else c_phi <= c_phi + resize(c_dphi, 24); end if;
                    -- Glow: colouriser gain 19 .. 143 as mantissa x 2^exponent; falloff setting 0 .. 6
                    when 146 => z_kk <= m_acc(19 downto 12);
                    when 147 => z_gqv <= to_unsigned(19, 8) + shift_right(z_kk, 1) - shift_right(z_kk, 6);
                    when 148 =>
                        if z_gqv(7) = '1' then c_ge <= "11"; c_gm <= z_gqv(7 downto 3);
                        elsif z_gqv(6) = '1' then c_ge <= "10"; c_gm <= z_gqv(6 downto 2);
                        elsif z_gqv(5) = '1' then c_ge <= "01"; c_gm <= z_gqv(5 downto 1);
                        else c_ge <= "00"; c_gm <= z_gqv(4 downto 0); end if;
                        v_k7 := shift_left(resize(s_g3(12 downto 3), 13), 3) - resize(s_g3(12 downto 3), 13);
                        c_fi <= v_k7(12 downto 10);
                    -- Line (fixed at 520 since v0.1.1): half-width 84 virtual px (Q4), in real px per raster
                    -- (the values v0.1.0 computed for 520), as mantissa x 2^exponent
                    when 173 =>
                        case to_integer(s_xs) is
                            when 4      => c_wpx <= to_unsigned(42, 8);
                            when 3      => c_wpx <= to_unsigned(54, 8);
                            when others => c_wpx <= to_unsigned(84, 8);
                        end case;
                    when 174 =>
                        case to_integer(s_xs) is
                            when 4      => c_we <= "01"; c_wm <= to_unsigned(10, 4);
                            when 3      => c_we <= "01"; c_wm <= to_unsigned(13, 4);
                            when others => c_we <= "10"; c_wm <= to_unsigned(10, 4);
                        end case;
                        case s_rho is
                            when "10"   => c_wln <= shift_right(c_wpx, 1);
                            when "01"   => c_wln <= c_wpx - shift_right(c_wpx, 3);
                            when others => c_wln <= c_wpx;
                        end case;
                    when 194 => z_x <= m_acc(23 downto 0);
                    when 195 => c_xstart <= signed(resize(c_xstep(12 downto 1), 24) - z_x);
                    -- Colour: rotate = 0.332 (k6 - 512), in quarter positions
                    when 196 =>
                        if s_g6(12) = '1' then z_km <= resize(s_g6(11 downto 3), 10); z_ks <= '0';
                        else z_km <= to_unsigned(512, 10) - resize(s_g6(11 downto 3), 10); z_ks <= '1'; end if;
                    when 197 =>
                        z_r1 <= resize(shift_right(z_km, 2), 8) + resize(shift_right(z_km, 4), 8);
                        z_r2 <= resize(shift_right(z_km, 6), 8) + resize(shift_right(z_km, 8), 8);
                    when 198 =>
                        v_8 := z_r1 + z_r2;
                        if z_ks = '1' then c_rot8 <= (not v_8) + 1; else c_rot8 <= v_8; end if;
                    when others => null;
                end case;
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
    -- Source side: the camera's luma (or the oscillator: a 2-D triangle
    -- lattice, drifting, at 3x the camera's level) through two horizontal
    -- slews and one vertical slew (a line store at pair resolution), then
    -- into function generator A's phase input
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_s : unsigned(7 downto 0);
        variable v_ys : unsigned(14 downto 0);
        variable v_gi : unsigned(3 downto 0);
        variable v_e : signed(17 downto 0);
        variable v_n : signed(17 downto 0);
        variable v_g : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then
                s_ot <= s_ot + 1;
                -- K6 camera gain = round(K6 / 128) / 2 (x 1 at 25 %, x 2 at 50 %), 5 at the top (+25 %, was 4;
                -- 4.5 on step 7, was 4); s_yin still clips at 4095
                v_gi := resize(s_gc(12 downto 10), 4) + resize(s_gc(9 downto 9), 4);
                case to_integer(v_gi) is
                    when 0      => c_cga <= "00"; c_cgb <= "00";
                    when 1      => c_cga <= "00"; c_cgb <= "01";
                    when 2      => c_cga <= "01"; c_cgb <= "00";
                    when 3      => c_cga <= "01"; c_cgb <= "01";
                    when 4      => c_cga <= "10"; c_cgb <= "00";
                    when 5      => c_cga <= "10"; c_cgb <= "01";
                    when 6      => c_cga <= "10"; c_cgb <= "10";
                    when 7      => c_cga <= "11"; c_cgb <= "01";
                    when others => c_cga <= "11"; c_cgb <= "10";
                end case;
            end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            -- input level: luma above black, or the oscillator x 12; then x the K6 camera gain (8 steps of
            -- a shift-select, its own stage; one add) and clipped at 4095 (half a cycle of phase)
            if data_in.avid = '0' then
                s_yin0 <= (others => '0');
            elsif s_src = '1' then
                s_yin0 <= shift_left(resize(s_osum, 12), 3) + shift_left(resize(s_osum, 12), 2);
            elsif unsigned(data_in.y) > 64 then
                s_yin0 <= resize(unsigned(data_in.y) - 64, 12);
            else
                s_yin0 <= (others => '0');
            end if;
            case c_cga is
                when "00"   => s_ya <= (others => '0');
                when "01"   => s_ya <= resize(s_yin0, 14);
                when "10"   => s_ya <= resize(s_yin0 & '0', 14);
                when others => s_ya <= s_yin0 & "00";
            end case;
            case c_cgb is
                when "00"   => s_yb <= (others => '0');
                when "01"   => s_yb <= shift_right(s_yin0, 1);
                when others => s_yb <= s_yin0;
            end case;
            v_ys := resize(s_ya, 15) + resize(s_yb, 15);
            if v_ys(14 downto 12) /= "000" then s_yin <= (others => '1'); else s_yin <= v_ys(11 downto 0); end if;
            -- two horizontal slews (Q4; 16 px)
            v_e := signed(resize(s_yin & "0000", 18)) - signed(resize(s_h1, 18));
            v_n := signed(resize(s_h1, 18)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_n(15 downto 0));
            v_e := signed(resize(s_h1, 18)) - signed(resize(s_h2, 18));
            v_n := signed(resize(s_h2, 18)) + shift_right(v_e, 4);
            s_h2 <= unsigned(v_n(15 downto 0));
            -- vertical slew (1/8 per line) on the line store, one update per pixel pair: read (address,
            -- data, fabric copy), then modify and write three clocks behind the read
            lv_ra <= s_wcol(10 downto 1);
            lv_f  <= lv_q;
            s_wd1 <= s_wcol; s_wd2 <= s_wd1; s_wd3 <= s_wd2;
            s_av1 <= data_in.avid; s_av2 <= s_av1; s_av3 <= s_av2;
            lv_we <= '0';
            if s_av3 = '1' and s_wd3(0) = '1' then
                v_e := signed(resize(s_h2(15 downto 1), 18)) - signed(resize(unsigned(lv_f(14 downto 0)), 18));
                v_e := shift_right(v_e, 3);
                v_n := signed(resize(unsigned(lv_f(14 downto 0)), 18)) + v_e;
                -- half the slew's own line-to-line step: the picture's vertical gradient, for the line's weight
                v_g := unsigned(abs(v_e));
                if v_g(17 downto 12) /= "000000" then s_gyl <= (others => '1'); else s_gyl <= v_g(11 downto 1); end if;
                lv_wd <= '0' & std_logic_vector(v_n(14 downto 0));
                lv_wa <= s_wd3(10 downto 1);
                lv_we <= '1';
                s_l13 <= unsigned(v_n(14 downto 3));
            end if;
            s_loff <= resize(s_l13, 13) + c_off13;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine.  An event is the end of every generated line (and four in
    -- the vblank zone, which prime it for lines 0 and 1).  After event m it
    --   reads the cosine table four times for line m + 2 (phases from the last pass) and the
    --     arcsine table once for line m + 3, in hblank, while the pixel path does not need them;
    --   interpolates the two cosines, takes their steps from the line before, scales the steps
    --     by the line weight (results r_*, handed to the pixel path at the next event);
    --   multiplies b * theta for line m + 3 and forms its two phases;
    --   scales |y - centre| for line m + 4 into the arcsine table's index.
    -- One serial multiplier, about 105 clocks a pass.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ev : std_logic;
        variable v_c  : signed(16 downto 0);
        variable v_d  : signed(17 downto 0);
        variable v_a  : unsigned(17 downto 0);
        variable v_y  : signed(20 downto 0);
        variable v_ya : unsigned(20 downto 0);
        variable v_ph : unsigned(15 downto 0);
        variable v_cy : signed(16 downto 0);
        variable v_w  : unsigned(14 downto 0);
        variable v_w3 : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0'; e_zk0 <= '0';
            if s_zone = '1' then
                if s_zc = 448 then e_zk <= '1'; e_zk0 <= '1'; end if;
                if s_zc = 768 or s_zc = 1088 or s_zc = 1408 then e_zk <= '1'; end if;
            end if;
            v_ev := e_zk or (s_gav_sr(0) and not g_avid);
            if v_ev = '1' then
                e_cnt <= (others => '0');
                if e_zk0 = '1' then
                    e_evn <= "00"; e_vy <= (others => '0');
                elsif e_evn /= 3 then
                    e_evn <= e_evn + 1;
                end if;
            elsif e_cnt /= 127 then
                e_cnt <= e_cnt + 1;
            end if;

            -- the serial multiplier; the field sequencer's products load it in the vblank zone
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(13 downto 1);
            if z_ld = '1' then
                m_a <= resize(z_fa, 32); m_b <= resize(z_fb, 14); m_acc <= (others => '0');
            end if;

            -- table data, three clocks after an index is presented
            if s10_sg = '1' then v_c := -signed(resize(s10_m, 17)); else v_c := signed(resize(s10_m, 17)); end if;
            if e_cnt >= 25 and e_cnt <= 28 then
                e_c(3) <= v_c; e_c(2) <= e_c(3); e_c(1) <= e_c(2); e_c(0) <= e_c(1);
            end if;

            case to_integer(e_cnt) is
                -- table reads (the engine owns both ports while e_own is set)
                when 21 | 23 =>
                    e_i12 <= e_p(0)(15 downto 4);
                    if e_cnt = 21 then
                        e_own <= '1';
                        if y3_q2(12) = '1' then e_ya <= (others => '1'); else e_ya <= y3_q2(11 downto 2); end if;
                    end if;
                when 22 | 24 =>
                    e_i12 <= e_p(0)(15 downto 4) + 1;
                    e_p(0) <= e_p(1); e_p(1) <= e_p(0);
                    if e_cnt = 24 then
                        -- hand the last pass's results to the pixel path (its last pixel is past them)
                        n_cy <= r_cy; n_w <= r_w; n_w38 <= r_w38; n_outy <= r_outy; n_nol <= r_nol;
                    end if;
                when 25 =>
                    e_own <= '0';
                    e_th <= s4_t;

                -- per family: interpolate the cosine (4-bit fraction), its step from the line before,
                -- the step times the line weight
                when 30 | 46 =>
                    v_d := resize(e_c(1), 18) - resize(e_c(0), 18);
                    e_dn <= v_d(17);
                    v_a := unsigned(abs(v_d));
                    m_a <= resize(v_a, 32); m_b <= resize(e_p(0)(3 downto 0), 14); m_acc <= (others => '0');
                when 35 | 51 =>
                    if e_dn = '1' then e_cy <= e_c(0) - signed(resize(m_acc(19 downto 4), 17));
                    else e_cy <= e_c(0) + signed(resize(m_acc(19 downto 4), 17)); end if;
                when 36 | 52 =>
                    v_d := resize(e_cy, 18) - resize(e_pv(0), 18);
                    v_a := unsigned(abs(v_d));
                    if v_a > 8191 then v_a := to_unsigned(8191, 18); end if;
                    m_a <= resize(v_a, 32); m_b <= resize(c_wln, 14); m_acc <= (others => '0');
                    e_pv(0) <= e_pv(1); e_pv(1) <= e_cy;
                    v_cy := e_cy + 8;
                    r_cy(0) <= r_cy(1); r_cy(1) <= resize(shift_right(v_cy, 4), 12);
                when 45 | 61 =>
                    if e_evn = 2 then v_w := (others => '0'); else v_w := m_acc(20 downto 6); end if;
                    v_w3 := resize(v_w, 17) + shift_left(resize(v_w, 17), 1);
                    r_w(0) <= r_w(1); r_w(1) <= v_w;
                    r_w38(0) <= r_w38(1); r_w38(1) <= v_w3(16 downto 3) + 4;
                    e_c(0) <= e_c(2); e_c(1) <= e_c(3); e_c(2) <= e_c(0); e_c(3) <= e_c(1);
                    e_p(0) <= e_p(1); e_p(1) <= e_p(0);

                -- the V shaper's output for line m + 3, b * theta, the two phases
                when 64 =>
                    if s_lin = '1' then
                        if y3_neg = '1' then e_thy <= to_unsigned(8192, 14) + resize(y3_q2, 14);
                        else e_thy <= to_unsigned(8192, 14) - resize(y3_q2, 14); end if;
                    elsif y3_neg = '1' then e_thy <= to_unsigned(8192, 14) - resize(e_th, 14);
                    else e_thy <= resize(e_th, 14); end if;
                    r_outy <= e_onx; r_nol <= e_onl;
                    e_onx <= y3_q2(12) and not s_lin;
                    -- no line in the square's outermost rows (index >= 1016 of 1024: the shaper's slope
                    -- is past what one row can resolve there)
                    if s_lin = '0' and (y3_q2(12) = '1' or y3_q2(11 downto 5) = "1111111") then e_onl <= '1'; else e_onl <= '0'; end if;
                when 65 =>
                    m_a <= resize(c_b, 32); m_b <= e_thy; m_acc <= (others => '0');
                when 82 =>
                    v_ph := m_acc(23 downto 8);
                    if s_lin = '1' then
                        v_ph := v_ph - shift_left(resize(c_b, 16), 5);
                    end if;
                    e_ph16 <= v_ph;
                when 83 =>
                    e_p(0) <= e_ph16 + c_phi(23 downto 8);
                    e_p(1) <= e_ph16 - c_phi(23 downto 8);
                -- |y - centre| for line m + 4 -> table index (Q2)
                when 84 =>
                    v_y := signed(resize(e_vy, 21)) - 138240;
                    y3_neg <= v_y(20);
                    v_ya := unsigned(abs(v_y));
                    m_a <= resize(v_ya(18 downto 0), 32); m_b <= resize(c_ky, 14); m_acc <= (others => '0');
                when 102 =>
                    if m_acc(31 downto 28) /= "0000" then y3_q2 <= (others => '1'); else y3_q2 <= m_acc(27 downto 15); end if;
                    e_vy <= e_vy + resize(s_ys, 20);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the x DDA's value of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ax  : unsigned(23 downto 0);
        variable v_q2  : unsigned(12 downto 0);
        variable v_p22 : unsigned(21 downto 0);
        variable v_i13 : unsigned(12 downto 0);
        variable v_i12 : unsigned(11 downto 0);
        variable v_a13 : unsigned(12 downto 0);
        variable v_s13 : signed(12 downto 0);
        variable v_xp  : unsigned(15 downto 0);
        variable v_x38 : unsigned(15 downto 0);
        variable v_t   : unsigned(15 downto 0);
        variable v_a8, v_a4, v_a2 : unsigned(15 downto 0);
        variable v_nol : std_logic;
        variable v_c1, v_c2 : unsigned(1 downto 0);
        variable v_s3  : unsigned(13 downto 0);
        variable v_tr  : unsigned(4 downto 0);
        variable v_ps  : unsigned(16 downto 0);
        variable v_p11 : unsigned(10 downto 0);
        variable v_t3  : unsigned(2 downto 0);
        variable v_p6  : unsigned(5 downto 0);
        variable v_ty, v_tu, v_tv, v_by, v_bu, v_bv : unsigned(9 downto 0);
        variable v_h   : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: x DDA, free-running from each line start (it runs on past the line end)
            if g_start = '1' then x_acc <= c_xstart; else x_acc <= x_acc + signed(resize(c_xstep, 24)); end if;
            -- 2: |x| -> the arcsine table's index (Q2), clipped at the square's edge
            if x_acc(23) = '1' then v_ax := unsigned(-x_acc); else v_ax := unsigned(x_acc); end if;
            if v_ax(23 downto 21) /= "000" then v_q2 := (others => '1'); else v_q2 := v_ax(20 downto 8); end if;
            s2_q2 <= v_q2; s2_neg <= x_acc(23);
            s2_out <= v_q2(12) and not s_lin;
            if e_own = '1' then t_ra <= e_ya;
            elsif v_q2(12) = '1' then t_ra <= (others => '1');
            else t_ra <= v_q2(11 downto 2); end if;
            -- 3: table (EBR)   4: fabric copy
            s3_q2 <= s2_q2; s3_neg <= s2_neg;
            s4_q2 <= s3_q2; s4_neg <= s3_neg;
            s4_t <= r_ac_q(12 downto 0);
            s_osr(3) <= s2_out;
            for i in 4 to 22 loop s_osr(i) <= s_osr(i - 1); end loop;
            s_osr(23) <= s_osr(22) or n_outy;
            for i in 24 to 26 loop s_osr(i) <= s_osr(i - 1); end loop;
            -- 5: theta (2^-14 cycle): the arcsine shaper, or the bare ramp (offset by 8192)
            if s_lin = '1' then
                if s4_neg = '1' then s5_th <= to_unsigned(8192, 14) + resize(s4_q2, 14);
                else s5_th <= to_unsigned(8192, 14) - resize(s4_q2, 14); end if;
            elsif s4_neg = '1' then s5_th <= to_unsigned(8192, 14) - resize(s4_t, 14);
            else s5_th <= resize(s4_t, 14); end if;
            -- 6, 7: a * theta (a in Q8), mod one cycle (2^22)
            s6_p0 <= s5_th * c_a(5 downto 2);
            s6_p1 <= s5_th * c_a(9 downto 6);
            s6_p2 <= s5_th * c_a(13 downto 10);
            v_p22 := resize(s6_p0, 22) + shift_left(resize(s6_p1, 22), 4) + shift_left(resize(s6_p2, 22), 8);
            s7_ph <= v_p22(21 downto 9);
            -- 8: + luma (+ the linear shaper's offset) -> cosine table index, quarter-wave folded
            v_i13 := s7_ph + s_loff;
            if e_own = '1' then v_i12 := e_i12; else v_i12 := v_i13(12 downto 1); end if;
            if v_i12(10) = '1' then c_ra <= not v_i12(9 downto 0); else c_ra <= v_i12(9 downto 0); end if;
            s8_sg <= v_i12(11) xor v_i12(10);
            -- 9: table (EBR)   10: fabric copy   11: Cx, signed; 19 and 20: Cx 8 and 9 px back (ring)
            s9_sg <= s8_sg;
            s10_m <= r_cq; s10_sg <= s9_sg;
            if s10_sg = '1' then cx11 <= -signed(resize(s10_m(15 downto 4), 12)); else cx11 <= signed(resize(s10_m(15 downto 4), 12)); end if;
            cx19 <= signed(r_xr(11 downto 0));
            cx20 <= cx19;
            -- 16: the horizontal difference of Cx over +-4 px   17: magnitude, clipped
            s16_dc <= resize(cx11, 13) - resize(cx19, 13);
            v_a13 := unsigned(abs(s16_dc));
            if v_a13 > 2047 then s17_gx <= (others => '1'); else s17_gx <= v_a13(10 downto 0); end if;
            -- 18: + the picture's vertical gradient (the luma slew's step)
            s18_g <= resize(s17_gx, 12) + resize(s_gyl, 12);
            -- 19, 20: x the line weight (mantissa, then exponent)
            s19_x <= s18_g * c_wm;
            case c_we is
                when "00"   => s20_xp <= resize(s19_x(15 downto 4), 15);
                when "01"   => s20_xp <= resize(s19_x(15 downto 3), 15);
                when "10"   => s20_xp <= resize(s19_x(15 downto 2), 15);
                when others => s20_xp <= s19_x(15 downto 1);
            end case;
            -- 21: per strand family: threshold T = larger + 3/8 smaller + 1 of (the weighted gradient,
            -- the generator's own weighted line-to-line step); F and the twin's F (generator B inverted)
            v_xp := resize(s20_xp, 16);
            v_x38 := resize(shift_right(s20_xp, 2), 16) + resize(shift_right(s20_xp, 3), 16) + 4;
            for k in 0 to 1 loop
                if v_xp > resize(n_w(k), 16) then
                    s21_t(k) <= v_xp + resize(n_w38(k), 16);
                else
                    s21_t(k) <= resize(n_w(k), 16) + v_x38;
                end if;
                s21_f(k) <= resize(cx20, 13) - resize(n_cy(k), 13);
                s21_g(k) <= resize(cx20, 13) + resize(n_cy(k), 13);
            end loop;
            -- 22: magnitudes
            for k in 0 to 1 loop
                v_s13 := abs(s21_f(k)); s22_af(k) <= unsigned(v_s13(11 downto 0));
                v_s13 := abs(s21_g(k)); s22_at(k) <= unsigned(v_s13(11 downto 0));
            end loop;
            s22_t <= s21_t;
            -- 23: the window comparators: |F| < T/8 (full), < T/4 (half); the twin's dark edge at T/2.
            -- No line outside the square or in the first 4 px of a line.  The non-additive mix.
            if s_gav_sr(24) = '0' or s_osr(22) = '1' or n_nol = '1' then v_nol := '1'; else v_nol := '0'; end if;
            for k in 0 to 1 loop
                v_t := s22_t(k);
                v_a8 := shift_left(resize(s22_af(k), 16), 3); v_a4 := shift_left(resize(s22_af(k), 16), 2);
                if v_nol = '1' then s23_lv(k) <= "00";
                elsif v_a8 < v_t then s23_lv(k) <= "10";
                elsif v_a4 < v_t then s23_lv(k) <= "01";
                else s23_lv(k) <= "00"; end if;
                v_a8 := shift_left(resize(s22_at(k), 16), 3); v_a4 := shift_left(resize(s22_at(k), 16), 2);
                v_a2 := shift_left(resize(s22_at(k), 16), 1);
                if v_nol = '1' then s23_lt(k) <= "00";
                elsif v_a8 < v_t then s23_lt(k) <= "10";
                elsif v_a4 < v_t then s23_lt(k) <= "01";
                else s23_lt(k) <= "00"; end if;
                if v_nol = '0' and v_a2 < v_t then s23_sk(k) <= '1'; else s23_sk(k) <= '0'; end if;
            end loop;
            if s22_af(1) < s22_af(0) then s23_m01 <= s22_af(1); else s23_m01 <= s22_af(0); end if;
            if s22_at(1) < s22_at(0) then s23_m23 <= s22_at(1); else s23_m23 <= s22_at(0); end if;
            -- 24: coverage per figure; the colouriser's input
            if s23_lv(0) > s23_lv(1) then v_c1 := s23_lv(0); else v_c1 := s23_lv(1); end if;
            if s23_lt(0) > s23_lt(1) then v_c2 := s23_lt(0); else v_c2 := s23_lt(1); end if;
            if s_twin = '0' then v_c2 := "00"; end if;
            if v_c1 /= "00" then s24_cv <= v_c1; s24_tg <= '0'; else s24_cv <= v_c2; s24_tg <= '1'; end if;
            s24_sk <= (s23_sk(0) or s23_sk(1)) and s_twin;
            if s_twin = '1' and s23_m23 < s23_m01 then s24_s <= s23_m23; else s24_s <= s23_m01; end if;
            -- 25: colouriser gain (mantissa partial products); falloff tier
            s25_pa <= s24_s * c_gm(3 downto 0);
            if c_gm(4) = '1' then s25_pb <= s24_s; else s25_pb <= (others => '0'); end if;
            if c_fi(0) = '1' then v_s3 := resize(s24_s, 14) + shift_left(resize(s24_s, 14), 1);
            else v_s3 := shift_left(resize(s24_s, 14), 2); end if;
            case c_fi is
                when "000" | "001" => v_tr := v_s3(13 downto 9);
                when "010" | "011" => v_tr := '0' & v_s3(13 downto 10);
                when "100" | "101" => v_tr := "00" & v_s3(13 downto 11);
                when others        => v_tr := "000" & v_s3(13 downto 12);
            end case;
            if v_tr(4 downto 3) /= "00" then s25_tier <= "111"; else s25_tier <= v_tr(2 downto 0); end if;
            s25_cv <= s24_cv; s25_tg <= s24_tg; s25_sk <= s24_sk;
            -- 26: position = S * gain
            v_ps := resize(s25_pa, 17) + shift_left(resize(s25_pb, 17), 4);
            case c_ge is
                when "00"   => s26_pos <= resize(v_ps(16 downto 9), 11);
                when "01"   => s26_pos <= resize(v_ps(16 downto 8), 11);
                when "10"   => s26_pos <= resize(v_ps(16 downto 7), 11);
                when others => s26_pos <= v_ps(16 downto 6);
            end case;
            s26_tier <= s25_tier;
            s26_cv <= s25_cv; s26_tg <= s25_tg; s26_sk <= s25_sk;
            -- 27: palette address: position (+ rotate) and tier by Look; outside the square 5 tiers
            -- darker; the twin's dark edge forces a dark tier
            if s_mono = '1' then
                v_p6 := c_rot8(7 downto 2);
                case s26_tier is                     -- the bloom: 2 tier + 3, clamped
                    when "000"  => v_t3 := "011";
                    when "001"  => v_t3 := "101";
                    when others => v_t3 := "111";
                end case;
            else
                v_p11 := s26_pos + resize(c_rot8, 11);
                v_p6 := v_p11(7 downto 2);
                v_t3 := s26_tier;
            end if;
            if s_osr(26) = '1' then
                case v_t3 is
                    when "000"  => v_t3 := "101";
                    when "001"  => v_t3 := "110";
                    when others => v_t3 := "111";
                end case;
            end if;
            if s26_sk = '1' and v_t3 < 6 then v_t3 := "110"; end if;
            r_pa <= v_t3 & v_p6;
            s27_cv <= s26_cv; s27_tg <= s26_tg;
            -- 28: palette (EBR)   29: fabric copy
            s28_cv <= s27_cv; s28_tg <= s27_tg;
            s29_cv <= s28_cv; s29_tg <= s28_tg;
            s29_q <= r_pq;
            -- 30: compose: the line colour over the band colour (full, half or none).  Off the generated
            -- avid: black, with neutral chroma, so the chroma 8-sums can resync in every blanking interval
            if s29_tg = '1' then
                v_ty := unsigned(C_LJ_L2(31 downto 22)); v_tu := unsigned(C_LJ_L2(21 downto 12)); v_tv := unsigned(C_LJ_L2(11 downto 2));
            else
                v_ty := unsigned(C_LJ_L1(31 downto 22)); v_tu := unsigned(C_LJ_L1(21 downto 12)); v_tv := unsigned(C_LJ_L1(11 downto 2));
            end if;
            v_by := unsigned(s29_q(31 downto 22)); v_bu := unsigned(s29_q(21 downto 12)); v_bv := unsigned(s29_q(11 downto 2));
            if s_gav_sr(27) = '0' then
                s_y30 <= c_kky; s_u30 <= to_unsigned(512, 10); s_v30 <= to_unsigned(512, 10);
            elsif s29_cv = "10" then
                s_y30 <= v_ty; s_u30 <= v_tu; s_v30 <= v_tv;
            elsif s29_cv = "01" then
                v_h := resize(v_by, 11) + resize(v_ty, 11); s_y30 <= v_h(10 downto 1);
                v_h := resize(v_bu, 11) + resize(v_tu, 11); s_u30 <= v_h(10 downto 1);
                v_h := resize(v_bv, 11) + resize(v_tv, 11); s_v30 <= v_h(10 downto 1);
            else
                s_y30 <= v_by; s_u30 <= v_bu; s_v30 <= v_bv;
            end if;
            s_bxh <= s_bxh(6 downto 0) & s_gav_sr(27);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning (stages 31 .. 36): chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent running
    -- 8-sums (s_csu7 / s_csv7 = the chroma of 8 clocks before, from the rings), luma [1 2 1].  The
    -- 8-sums snap back to neutral once 8 blank samples are in, so a glitched sample can't offset the
    -- chroma for good.  Then luma toward black (x 13/16) and the blanking gate.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(14 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_bxh = "00000000" then
                s_boxu <= to_unsigned(4096, 14); s_boxv <= to_unsigned(4096, 14);
            else
                s_boxu <= s_boxu + s_u30 - s_csu7;
                s_boxv <= s_boxv + s_v30 - s_csv7;
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u36 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v36 <= v_sc(13 downto 4);
            s_y31 <= s_y30; s_y32 <= s_y31; s_y33 <= s_y32; s_y34 <= s_y33;
            s_y35 <= s_y34; s_y36x <= s_y35;
            v_sy := resize(s_y34, 12) + shift_left(resize(s_y35, 12), 1) + resize(s_y36x, 12);
            s_yf36 <= v_sy(11 downto 2);
            -- 37: luma toward black (white 940 -> 777); blanking gate
            if s_yf36 > 64 then v_dl := s_yf36 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y37 <= s_yf36 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u37 <= s_u36; s_v37 <= s_v36;
            else
                s_y37 <= to_unsigned(64, 10); s_u37 <= to_unsigned(512, 10); s_v37 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y37);
    data_out.u       <= std_logic_vector(s_u37);
    data_out.v       <= std_logic_vector(s_v37);
    data_out.hsync_n <= s_so(2);
    data_out.vsync_n <= s_so(1);
    data_out.field_n <= s_so(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lissajous;
