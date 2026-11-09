-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Starline v0.1.0 -- Heirloom (video phase), from the clip WIN_20231218_21_31_11 at 3.0 s.
--
-- The still: a neon "star" drawn as bundles of thin bright tubes: a left point (313, 775) and a right point
-- (1810, 775) on one row, a ")" arc and a "(" arc bowing toward the centre from them, a V under them dipping to
-- (1062, 913), and a band of rays across the top dipping to (1071, 374).  The tubes are pink-white on the left,
-- yellow and green in the middle, pink again at the right point; a magenta wash fills the left third, a green
-- glow sits above the band's dip, beads (sample-and-hold) chain the tubes on the right, a thin reset line
-- stands at x = 312.  In the clip the star never moves: the colours sweep sideways on a slow LFO (8.7 s).
--
-- The patch, as fitted (starline.py has the tables, notes.md the measurements):
--   T      = |x - 1071| (rounded within 128 px of the centre) + Etch * luma
--   star   tubes where log2 B(y) - log2(T + c(y)) = i / 4, i = 0 .. N-1   (geometric nesting)
--          B = 395 + q (y - 605)^2 above the kink row (the arcs), B = b_k, c = 5 (y - y_k) below it (the V),
--          a parabolic fillet over +-4 rows at the points
--   band   tubes where log2 a(y) - log2 T = j / 2, a = 8.7 (374 - y): rays from the dip
--   colour LUT(u, k): u = (x - x_v) / 2200 the line-locked hue ramp (x_v on the LFO), k = wash + tubes / glow
--
--   K1  Arcs     the arcs' curvature: near-straight sides (0) .. the clip (512) .. tight hooks (1023)
--   K2  Waist    the arcs' apex distance: arcs meet in a lens (0) .. the clip (449) .. pushed to the edges
--   K3  Points   the row of the star points (and where the V starts): high (0) .. the clip (500) .. low
--   K4  Sweep    the colour LFO rate: 60 s (0) .. the clip's 8.7 s (512) .. 1.2 s (1023)
--   K5  Bands    the hue ramp period: 2 lines' worth (0) .. one line, the clip (512) .. 140 px stripes (1023)
--   K6  Etch     how far the camera (or the Osc) bends the tubes
--   S7  Mirror   the star mirrored about its point row: arcs below too, a four-armed star
--   S8  Beads    Zone (the clip: beads where the hue ramp passes green) / All: a 32 x 16 px mosaic everywhere
--   S9  Wash     On (the clip) / Off: tubes and glow on black
--   S10 Colour   Neon (the clip) / Ember (Cb inverted: orange-red wash, cyan-blue tubes)
--   S11 Source   Video / Osc
--   P12 NEON     the tube count: one tube (0) .. three, the clip (187) .. a dozen nested tubes (100 %)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: the arc / V / band terms; their log2 go through the pixel path's log2 in
-- hblank), a beam DDA and a late beam copy, a 40-stage pixel path (log2 ROM, two combs, two exp2 ROMs,
-- distances, profile ROMs, colour LUT).  Virtual 1080-line coordinates (SD shows the centre 4:3).
-- Generated avid; output conditioning (Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box, luma 13/16) before
-- the blanking gate; the colour LUT index is dithered with a period-8 pattern that the box cancels.  Colours in
-- the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.starline_pkg.all;
use work.all;

architecture starline of program_top is
    constant C_LAT : integer := 40;   -- = processing_delay_clks
    constant C_XC  : integer := 2142; -- star centre x (h = half virtual px)

    type t_u8_64 is array (0 to 63) of unsigned(7 downto 0);
    type t_u8_16 is array (0 to 15) of unsigned(7 downto 0);
    constant C_GLUT : t_u8_64 := (
        to_unsigned(255, 8), to_unsigned(248, 8), to_unsigned(229, 8), to_unsigned(205, 8), to_unsigned(181, 8),
        to_unsigned(160, 8), to_unsigned(142, 8), to_unsigned(127, 8), to_unsigned(114, 8), to_unsigned(104, 8),
        to_unsigned(95, 8), to_unsigned(87, 8), to_unsigned(81, 8), to_unsigned(75, 8), to_unsigned(70, 8),
        to_unsigned(66, 8), to_unsigned(62, 8), to_unsigned(59, 8), to_unsigned(56, 8), to_unsigned(53, 8),
        to_unsigned(50, 8), to_unsigned(48, 8), to_unsigned(46, 8), to_unsigned(44, 8), to_unsigned(42, 8),
        to_unsigned(40, 8), to_unsigned(39, 8), to_unsigned(38, 8), to_unsigned(36, 8), to_unsigned(35, 8),
        to_unsigned(34, 8), to_unsigned(33, 8), to_unsigned(32, 8), to_unsigned(31, 8), to_unsigned(30, 8),
        to_unsigned(29, 8), to_unsigned(28, 8), to_unsigned(28, 8), to_unsigned(27, 8), to_unsigned(26, 8),
        to_unsigned(25, 8), to_unsigned(25, 8), to_unsigned(24, 8), to_unsigned(24, 8), to_unsigned(23, 8),
        to_unsigned(23, 8), to_unsigned(22, 8), to_unsigned(22, 8), to_unsigned(21, 8), to_unsigned(21, 8),
        to_unsigned(20, 8), to_unsigned(20, 8), to_unsigned(20, 8), to_unsigned(19, 8), to_unsigned(19, 8),
        to_unsigned(19, 8), to_unsigned(18, 8), to_unsigned(18, 8), to_unsigned(18, 8), to_unsigned(17, 8),
        to_unsigned(17, 8), to_unsigned(17, 8), to_unsigned(16, 8), to_unsigned(16, 8)
    );
    constant C_HA : t_u8_16 := (
        to_unsigned(17, 8), to_unsigned(90, 8), to_unsigned(163, 8), to_unsigned(236, 8), to_unsigned(53, 8),
        to_unsigned(126, 8), to_unsigned(199, 8), to_unsigned(16, 8), to_unsigned(89, 8), to_unsigned(162, 8),
        to_unsigned(235, 8), to_unsigned(52, 8), to_unsigned(125, 8), to_unsigned(198, 8), to_unsigned(15, 8),
        to_unsigned(88, 8)
    );
    constant C_HB : t_u8_16 := (
        to_unsigned(101, 8), to_unsigned(252, 8), to_unsigned(147, 8), to_unsigned(42, 8), to_unsigned(193, 8),
        to_unsigned(88, 8), to_unsigned(239, 8), to_unsigned(134, 8), to_unsigned(29, 8), to_unsigned(180, 8),
        to_unsigned(75, 8), to_unsigned(226, 8), to_unsigned(121, 8), to_unsigned(16, 8), to_unsigned(167, 8),
        to_unsigned(62, 8)
    );


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
    -- osc lattice line step: ((ys * 121) >> 8) * 2
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1074, 11);
            when 4          => return to_unsigned(544, 11);
            when 8          => return to_unsigned(906, 11);
            when 12         => return to_unsigned(452, 11);
            when 5 | 6 | 14 => return to_unsigned(362, 11);
            when 1 | 2 | 10 => return to_unsigned(484, 11);
            when others     => return to_unsigned(242, 11);
        end case;
    end function;
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_tri8(v : unsigned(8 downto 0)) return unsigned is
    begin
        if v(8) = '1' then return not v(7 downto 0); else return v(7 downto 0); end if;
    end function;
    -- position of the leading one (v >= 1); 13-bit input
    function f_msb(v : unsigned(12 downto 0)) return unsigned is
    begin
        for i in 12 downto 1 loop
            if v(i) = '1' then return to_unsigned(i, 4); end if;
        end loop;
        return to_unsigned(0, 4);
    end function;
    function f_xs16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x xor shift_left(x, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        return v;
    end function;
    -- t = mantissa (Q15) >> (15 - e)
    function f_exsh(m : unsigned(15 downto 0); e : unsigned(3 downto 0)) return unsigned is
    begin
        return shift_right(m, 15 - to_integer(e));
    end function;
    -- x << sh or x >> -sh, sh in -4 .. 3
    function f_sh(x : unsigned(20 downto 0); sh : signed(2 downto 0)) return unsigned is
    begin
        if sh >= 0 then return shift_left(x, to_integer(sh)); else return shift_right(x, to_integer(-sh)); end if;
    end function;

    -- line buffer (camera / osc luma, every other pixel): luma - 64, clipped at 0 (10 bits)
    type t_lb is array (0 to 1023) of std_logic_vector(9 downto 0);
    signal lb : t_lb := (others => (others => '0'));
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(9 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(9 downto 0) := (others => '0');
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data; a fabric register after each)
    signal r_sf_a : unsigned(7 downto 0) := (others => '0');
    signal r_sf_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_lg_a : unsigned(7 downto 0) := (others => '0');
    signal r_lg_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_exa_a, r_exb_a : unsigned(7 downto 0) := (others => '0');
    signal r_exa_q, r_exb_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_ba_a, r_bb_a : unsigned(7 downto 0) := (others => '0');
    signal r_ba_q, r_bb_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_c_a : unsigned(10 downto 0) := (others => '0');
    signal r_cy_q, r_cu_q, r_cv_q : std_logic_vector(7 downto 0) := (others => '0');

    -- T delay ring (written at stage 7, read for stage 17)
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_mem : t_r16 := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_mem : signal is true;
    signal s_rwp, s_rra : unsigned(7 downto 0) := (others => '0');
    signal s_rw_d : std_logic_vector(15 downto 0) := (others => '0');
    signal s_rq   : std_logic_vector(15 downto 0) := (others => '0');
    -- sync delay ring (one EBR in place of 120 shift-register FFs): hsync / vsync / field written every clock
    -- at s_rwp, read at s_rwp - 38 (s_sra, registered), EBR read register, fabric register: the output is the
    -- input C_LAT - 1 = 39 clocks earlier, exactly as the old shift register
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal r_smem : t_r4 := (others => (others => '1'));
    attribute no_rw_check of r_smem : signal is true;
    signal s_sra  : unsigned(7 downto 0) := (others => '0');
    signal s_sw_d : std_logic_vector(3 downto 0) := (others => '1');
    signal s_sq   : std_logic_vector(3 downto 0) := (others => '1');
    signal s_so   : std_logic_vector(3 downto 0) := (others => '1');

    -- oscillator source (Source = Osc): 2-D triangle lattice, drifting, sharpened
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(242, 11);
    signal s_osum : unsigned(9 downto 0) := (others => '0');

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
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw sweep, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(449 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(500 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(420 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(187 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_mir, s_bead, s_woff, s_emb, s_src : std_logic := '0';

    -- per-field constants
    signal c_qu   : unsigned(15 downto 0) := to_unsigned(983, 16);
    signal c_ql   : unsigned(15 downto 0) := to_unsigned(1547, 16);
    signal c_a    : unsigned(11 downto 0) := to_unsigned(790, 12);
    signal c_yk   : unsigned(10 downto 0) := to_unsigned(774, 11);
    signal c_bk   : unsigned(11 downto 0) := to_unsigned(1464, 12);
    signal c_lbk  : unsigned(11 downto 0) := to_unsigned(2692, 12);
    signal c_nia, c_nib : unsigned(3 downto 0) := to_unsigned(3, 4);     -- full tubes
    signal c_nqa, c_nqb : unsigned(1 downto 0) := (others => '0');       -- quarters of the next tube
    signal c_nfa, c_nfb : std_logic := '0';                              -- a fractional tube is drawn
    signal c_ima, c_imb : unsigned(3 downto 0) := to_unsigned(2, 4);     -- last tube index
    signal c_gw   : unsigned(2 downto 0) := to_unsigned(7, 3);           -- glow weight (/16)
    signal c_us   : unsigned(16 downto 0) := to_unsigned(3813, 17);      -- hue phase per h (Q24)
    signal c_usx  : unsigned(18 downto 0) := to_unsigned(7626, 19);      -- per pixel
    signal c_vt1, c_vt2 : unsigned(7 downto 0) := (others => '0');       -- reset-line half widths (phase >> 12)
    signal c_rate : unsigned(15 downto 0) := to_unsigned(126, 16);
    signal c_p    : unsigned(15 downto 0) := to_unsigned(60621, 16);     -- LFO phase (Q16)
    signal c_xv   : unsigned(12 downto 0) := to_unsigned(618, 13);
    signal c_u0   : unsigned(23 downto 0) := (others => '0');
    signal c_eg   : unsigned(7 downto 0) := to_unsigned(43, 8);
    signal c_xst  : unsigned(11 downto 0) := to_unsigned(8, 12);         -- beam start x (h)
    signal c_bsh  : std_logic := '0';                                    -- bead block: 64 h (0) / 256 h (1)
    signal c_su   : unsigned(11 downto 0) := to_unsigned(127, 12);       -- the arc's slope at the kink (h/16 per line, >= 0)
    signal c_gu   : unsigned(7 downto 0) := to_unsigned(66, 8);          -- the arc's 1/sqrt(1 + s^2) at the kink

    -- field serial multiplier (static a, b shifting right, accumulator shifting right) and scratch
    signal f_a   : unsigned(17 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(34 downto 0) := (others => '0');
    signal f_cnt : unsigned(4 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal z_m   : unsigned(15 downto 0) := (others => '0');
    signal z_sh  : signed(2 downto 0) := (others => '0');
    signal z_shv : unsigned(20 downto 0) := (others => '0');
    signal z_dk  : unsigned(9 downto 0) := (others => '0');
    signal z_dkn : std_logic := '0';
    signal z_dk2 : unsigned(17 downto 0) := (others => '0');
    signal z_na  : unsigned(7 downto 0) := to_unsigned(48, 8);
    signal z_nx  : unsigned(7 downto 0) := (others => '0');
    signal z_v4  : unsigned(9 downto 0) := (others => '0');
    signal z_v4n : std_logic := '0';
    signal z_fq  : unsigned(15 downto 0) := (others => '0');
    signal z_s2  : unsigned(15 downto 0) := (others => '0');
    signal z_s   : unsigned(15 downto 0) := (others => '0');
    signal z_dx  : unsigned(13 downto 0) := (others => '0');
    signal z_xsel, z_inj : std_logic := '0';
    signal z_exa : unsigned(7 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_y    : unsigned(10 downto 0) := (others => '0');
    signal e_d    : unsigned(10 downto 0) := (others => '0');
    signal e_dn, e_bel, e_ya, e_fil : std_logic := '0';
    signal e_yad  : unsigned(10 downto 0) := (others => '0');
    signal e_dc4  : unsigned(3 downto 0) := (others => '0');
    signal e_b    : unsigned(11 downto 0) := (others => '0');
    signal e_a    : unsigned(12 downto 0) := (others => '0');
    signal e_w1, e_w2 : unsigned(7 downto 0) := (others => '0');
    signal e_p1   : unsigned(14 downto 0) := (others => '0');
    signal e_p2   : unsigned(18 downto 0) := (others => '0');
    signal m_a    : unsigned(21 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(38 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal e_lcnt : unsigned(3 downto 0) := (others => '0');
    signal e_inj  : std_logic := '0';
    signal e_injv : unsigned(12 downto 0) := (others => '0');
    signal e_la, e_lb : unsigned(11 downto 0) := (others => '0');
    -- next-line results (loaded into l_* at g_start)
    signal n_c    : unsigned(11 downto 0) := (others => '0');
    signal n_lba  : unsigned(11 downto 0) := (others => '0');
    signal n_lbb  : unsigned(11 downto 0) := (others => '0');
    signal n_ga   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal n_dyb  : unsigned(10 downto 0) := (others => '0');
    signal n_vb   : std_logic := '0';
    signal n_wl   : unsigned(7 downto 0) := to_unsigned(48, 8);
    signal n_kc   : unsigned(1 downto 0) := to_unsigned(2, 2);
    signal l_c    : unsigned(11 downto 0) := (others => '0');
    signal l_lba  : unsigned(11 downto 0) := (others => '0');
    signal l_lbb  : unsigned(11 downto 0) := (others => '0');
    signal l_ga   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal l_dyb  : unsigned(10 downto 0) := (others => '0');
    signal l_vb   : std_logic := '0';
    signal l_wl   : unsigned(7 downto 0) := to_unsigned(48, 8);
    signal l_kc   : unsigned(1 downto 0) := to_unsigned(2, 2);

    -- beam (stage 0) and the late beam (aligned with stage 24)
    signal b_x  : unsigned(11 downto 0) := (others => '0');
    signal b_u  : unsigned(23 downto 0) := (others => '0');
    signal b_n  : unsigned(10 downto 0) := (others => '0');
    signal q_x  : unsigned(11 downto 0) := (others => '0');
    signal q_u  : unsigned(23 downto 0) := (others => '0');
    signal q_r  : unsigned(15 downto 0) := x"ACE1";
    signal q_n  : unsigned(2 downto 0) := (others => '0');

    -- pixel path
    signal p1_x  : unsigned(11 downto 0) := (others => '0');
    signal p2_dx, p3_dx, p4_dx : unsigned(11 downto 0) := (others => '0');
    signal p3_lt, p4_lt : std_logic := '0';
    signal p3_lm : unsigned(9 downto 0) := (others => '0');
    signal p4_sf : unsigned(6 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(12 downto 0) := (others => '0');
    signal p5_ts, p5_et : unsigned(11 downto 0) := (others => '0');
    signal p6_tp : unsigned(11 downto 0) := (others => '0');
    signal p7_t, p8_t : unsigned(12 downto 0) := (others => '0');
    signal p8_e, p9_e, p10_e, p11_e : unsigned(3 downto 0) := (others => '0');
    signal p11_lg : unsigned(7 downto 0) := (others => '0');
    signal p12_da, p12_db : signed(12 downto 0) := (others => '0');
    signal p13_ia, p13_ib : unsigned(3 downto 0) := (others => '0');
    signal p13_w12 : unsigned(7 downto 0) := (others => '0');
    signal p14_ea, p14_eb, p15_ea, p15_eb, p16_ea, p16_eb : unsigned(3 downto 0) := (others => '0');
    type t_u4_arr is array (14 to 24) of unsigned(3 downto 0);
    signal pi_a, pi_b : t_u4_arr := (others => (others => '0'));
    type t_u8_arr is array (14 to 18) of unsigned(7 downto 0);
    signal pw12 : t_u8_arr := (others => (others => '0'));
    signal p16_xa, p16_xb : unsigned(15 downto 0) := (others => '0');
    signal p17_ta, p17_tb, p17_t : unsigned(12 downto 0) := (others => '0');
    signal p18_ea, p18_eb : unsigned(11 downto 0) := (others => '0');
    signal p18_w3 : unsigned(7 downto 0) := (others => '0');
    signal p19_al, p19_ah : unsigned(15 downto 0) := (others => '0');
    signal p19_b0, p19_b1 : unsigned(16 downto 0) := (others => '0');
    signal p19_m  : unsigned(7 downto 0) := (others => '0');
    signal p20_da, p20_db : unsigned(13 downto 0) := (others => '0');
    signal p20_bf, p21_bf, p22_bf, p23_bf, p24_bf : unsigned(6 downto 0) := (others => '0');
    signal p23_la, p23_ga, p23_lb, p23_gb : unsigned(7 downto 0) := (others => '0');
    signal p24_la, p24_lb, p24_gm : unsigned(7 downto 0) := (others => '0');
    signal p25_la, p25_lb : unsigned(7 downto 0) := (others => '0');
    signal p25_gw : unsigned(10 downto 0) := (others => '0');
    signal p25_bf : unsigned(6 downto 0) := (others => '0');
    signal p25_au : unsigned(11 downto 0) := (others => '0');
    signal p26_lm : unsigned(7 downto 0) := (others => '0');
    signal p26_gw : unsigned(6 downto 0) := (others => '0');
    signal p26_bf : unsigned(6 downto 0) := (others => '0');
    signal p26_vl, p26_va, p27_va : unsigned(7 downto 0) := (others => '0');
    signal p27_i, p27_vl : unsigned(7 downto 0) := (others => '0');
    signal p28_v, p28_at, p28_k : unsigned(7 downto 0) := (others => '0');
    signal p30_a, p31_a, p32_a : unsigned(10 downto 0) := (others => '0');
    signal p35_y, p35_u, p35_v : unsigned(7 downto 0) := (others => '0');
    signal s_y32, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal s_cay : t_u10(0 to 7) := (others => to_unsigned(64, 10));
    signal s_cau, s_cav : t_u8a(0 to 7) := (others => to_unsigned(128, 8));
    signal s_bay, s_bayd : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y34, s_u34, s_v34 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y35, s_u35, s_v35 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y36 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u36, s_v36 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs, T ring
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_sf_q  <= C_SL_SOFT(to_integer(r_sf_a));
            r_lg_q  <= C_SL_LOG2(to_integer(r_lg_a));
            r_exa_q <= C_SL_EXP2(to_integer(r_exa_a));
            r_exb_q <= C_SL_EXP2(to_integer(r_exb_a));
            r_ba_q  <= C_SL_BUMP(to_integer(r_ba_a));
            r_bb_q  <= C_SL_BUMP(to_integer(r_bb_a));
            r_cy_q  <= C_SL_CY(to_integer(r_c_a));
            r_cu_q  <= C_SL_CU(to_integer(r_c_a));
            r_cv_q  <= C_SL_CV(to_integer(r_c_a));
        end if;
    end process p_rom;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rq <= r_mem(to_integer(s_rra));
            r_mem(to_integer(s_rwp)) <= s_rw_d;
        end if;
    end process p_ring;

    s_sw_d <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            s_sq <= r_smem(to_integer(s_sra));
            r_smem(to_integer(s_rwp)) <= s_sw_d;
            s_so <= s_sq;
        end if;
    end process p_sring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : signed(10 downto 0);
        variable v_t  : unsigned(20 downto 0);
        variable v_u  : unsigned(12 downto 0);
        variable v_f  : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_oys <= f_oys(s_tid);

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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mir  <= registers_in(6)(0);
                s_bead <= registers_in(6)(1);
                s_woff <= registers_in(6)(2);
                s_emb  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(408, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(306, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(204, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;

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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (captured at load + 18); the slot is decoded at 30 + 24 k, operands pre-muxed at 31 + 24 k
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(983, 18);            z_fb <= z_m;                      -- QU
                    when 1  => z_fa <= to_unsigned(1547, 18);           z_fb <= z_m;                      -- QL
                    when 2  => z_fa <= resize(s_g2(12 downto 3), 18);  z_fb <= to_unsigned(1802, 16);     -- A
                    when 3  => z_fa <= resize(s_g3(12 downto 3), 18);  z_fb <= to_unsigned(440, 16);      -- YK
                    when 4  => z_fa <= resize(z_dk, 18);               z_fb <= resize(z_dk, 16);          -- dk^2
                    when 5  =>                                                                             -- q dk^2
                        z_fa <= z_dk2;
                        if z_dkn = '1' then z_fb <= c_qu; else z_fb <= c_ql; end if;
                    when 6  => z_fa <= resize(s_gp(12 downto 3), 18);  z_fb <= to_unsigned(176, 16);      -- tubes
                    when 7  => z_fa <= resize(z_nx, 18);               z_fb <= to_unsigned(36, 16);       -- glow
                    when 8  => z_fa <= to_unsigned(3813, 18);           z_fb <= z_m;                      -- US
                    when 9  => z_fa <= resize(z_v4, 18);               z_fb <= to_unsigned(362, 16);      -- sweep exp
                    when 10 => z_fa <= to_unsigned(126, 18);            z_fb <= z_m;                      -- rate
                    when 11 => z_fa <= resize(z_fq, 18);               z_fb <= z_fq;                      -- f^2
                    when 12 => z_fa <= to_unsigned(196608, 18) - shift_left(resize(z_fq, 18), 1); z_fb <= z_s2; -- f^2 (3 - 2f)
                    when 13 => z_fa <= resize(z_s, 18);                z_fb <= to_unsigned(7304, 16);     -- x_v
                    when 14 => z_fa <= resize(c_us, 18);               z_fb <= resize(z_dx, 16);          -- u0
                    when 15 => z_fa <= resize(s_g6(12 downto 3), 18);  z_fb <= resize(s_g6(12 downto 3), 16); -- etch^2
                    when 16 =>                                                                             -- q |dk|
                        z_fa <= resize(z_dk, 18);
                        if z_dkn = '1' then z_fb <= c_qu; else z_fb <= c_ql; end if;
                    when others => z_fa <= resize(c_us, 18); z_fb <= resize(s_xs, 16);                    -- US xs
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb; f_cnt <= to_unsigned(16, 5);
            elsif f_cnt /= 0 then
                f_cnt <= f_cnt - 1;
                if f_b(0) = '1' then v_f := ('0' & f_acc(33 downto 16)) + ('0' & f_a); else v_f := '0' & f_acc(33 downto 16); end if;
                f_acc <= '0' & v_f & f_acc(15 downto 1);
                f_b <= '0' & f_b(15 downto 1);
            end if;
            z_shv <= f_sh(resize(f_acc(34 downto 15), 21), z_sh);

            z_xsel <= '0'; z_inj <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>
                        c_xst <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 2);
                        c_bsh <= s_bead;
                    -- arcs exponent: e = k1 - 512 (+-2 octaves)
                    when 22 =>
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_exa <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8); z_xsel <= '1';
                    when 23 => z_xsel <= '1';
                    when 25 => z_m <= unsigned(r_exa_q);
                    when 51  => c_qu <= z_shv(15 downto 0);
                    when 75  => c_ql <= z_shv(15 downto 0);
                    when 98  => c_a <= f_acc(21 downto 10);
                    when 122 =>
                        v_u := resize(f_acc(19 downto 10), 13) + 560;
                        c_yk <= v_u(10 downto 0);
                    when 123 =>
                        if c_yk >= 605 then z_dk <= resize(c_yk - 605, 10); z_dkn <= '0';
                        else z_dk <= resize(605 - c_yk, 10); z_dkn <= '1'; end if;
                    when 146 => z_dk2 <= f_acc(17 downto 0);
                    when 170 =>
                        v_t := resize(c_a, 21) + resize(f_acc(34 downto 16), 21);
                        if v_t > 4095 then c_bk <= to_unsigned(4095, 12); else c_bk <= v_t(11 downto 0); end if;
                    -- log2 of b_k through the pixel path's log2 (injected at stage 7)
                    when 171 => z_inj <= '1';
                    when 177 => c_lbk <= p11_e & p11_lg;
                    when 194 =>
                        v_u := resize(f_acc(17 downto 10), 13) + 16;
                        if v_u > 192 then z_na <= to_unsigned(192, 8); else z_na <= v_u(7 downto 0); end if;
                    when 195 =>
                        c_nia <= z_na(7 downto 4); c_nqa <= z_na(3 downto 2);
                        if z_na(3 downto 0) /= 0 then c_nfa <= '1'; c_ima <= z_na(7 downto 4);
                        else c_nfa <= '0'; c_ima <= z_na(7 downto 4) - 1; end if;
                        v_u := resize(z_na, 13) + 16;
                        if v_u > 128 then v_u := to_unsigned(128, 13); end if;
                        c_nib <= v_u(7 downto 4); c_nqb <= v_u(3 downto 2);
                        if v_u(3 downto 0) /= 0 then c_nfb <= '1'; c_imb <= v_u(7 downto 4);
                        else c_nfb <= '0'; c_imb <= v_u(7 downto 4) - 1; end if;
                        if z_na > 48 then z_nx <= z_na - 48; else z_nx <= (others => '0'); end if;
                    when 196 =>
                        -- bands exponent: 2 (k5 - 512) above the centre, (k5 - 512) / 2 below
                        if s_g5(12 downto 3) >= 512 then
                            v_e := shift_left(signed(resize(s_g5(12 downto 3), 11)) - 512, 1);
                        else
                            v_e := shift_right(signed(resize(s_g5(12 downto 3), 11)) - 512, 1);
                        end if;
                        z_exa <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8); z_xsel <= '1';
                    when 197 => z_xsel <= '1';
                    when 199 => z_m <= unsigned(r_exa_q);
                    when 218 =>
                        if f_acc(13 downto 10) >= 5 or f_acc(17 downto 14) /= 0 then c_gw <= to_unsigned(2, 3);
                        else c_gw <= to_unsigned(7, 3) - resize(f_acc(12 downto 10), 3); end if;
                    when 219 =>
                        if s_k4 >= 512 then z_v4 <= s_k4 - 512; z_v4n <= '0';
                        else z_v4 <= 512 - s_k4; z_v4n <= '1'; end if;
                    when 243 => c_us <= z_shv(16 downto 0);
                    when 266 =>
                        if z_v4n = '1' then v_e := -signed(resize(f_acc(18 downto 8), 11));
                        else v_e := signed(resize(f_acc(18 downto 8), 11)); end if;
                        z_exa <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8); z_xsel <= '1';
                    when 267 => z_xsel <= '1';
                    when 269 => z_m <= unsigned(r_exa_q);
                    when 291 =>
                        if z_shv(15 downto 0) = 0 then c_rate <= to_unsigned(1, 16); else c_rate <= z_shv(15 downto 0); end if;
                        if c_p(15) = '1' then z_fq <= c_p(14 downto 0) & '0'; else z_fq <= (not c_p(14 downto 0)) & '0'; end if;
                    when 314 => z_s2 <= f_acc(31 downto 16);
                    when 338 => z_s  <= f_acc(31 downto 16);
                    when 362 => c_xv <= to_unsigned(7478, 13) - f_acc(28 downto 16);
                    when 363 =>
                        z_dx <= resize(c_xst, 14) + 8192 - resize(c_xv, 14);
                    when 386 =>
                        c_u0 <= f_acc(23 downto 0) - shift_left(resize(c_us, 24), 13);
                    when 410 => c_eg <= f_acc(19 downto 12);
                    when 411 =>
                        c_p <= c_p + c_rate;
                        c_vt1 <= c_us(16 downto 9) srl 1;
                        c_vt2 <= c_us(16 downto 9);
                    when 458 => c_usx <= f_acc(18 downto 0);
                    when 434 =>
                        if z_dkn = '1' then c_su <= (others => '0'); else c_su <= f_acc(22 downto 11); end if;
                        if f_acc(29 downto 20) /= 0 then c_gu <= C_GLUT(63); else c_gu <= C_GLUT(to_integer(f_acc(19 downto 14))); end if;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
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
    -- Source side: oscillator (2-D triangle lattice, drifting, sharpened),
    -- line-buffer write of luma - 64 (10 bits), every other pixel
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(9 downto 0);
        variable v_s : unsigned(9 downto 0);
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
            v_s := resize(f_tri8(s_ox(15 downto 7)), 10) + resize(f_tri8(s_oy(15 downto 7)), 10);
            -- 3 v - 510, clipped to 0 .. 765
            if v_s < 170 then s_osum <= (others => '0');
            elsif v_s >= 425 then s_osum <= to_unsigned(765, 10);
            else v_s := shift_left(v_s, 1) + v_s - 510; s_osum <= v_s; end if;
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol(10 downto 1);
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y);
                if v_l > 64 then s_lb_wd <= std_logic_vector(v_l - 64); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line's star / band terms on one serial multiplier
    -- (16 iterations, result at load + 17); its two log2 go through the pixel
    -- path's log2 (injected at stage 7) once the line has left it (hblank), or
    -- in the zone after the line-0 kick
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_r  : signed(21 downto 0);
        variable v_q  : signed(14 downto 0);
        variable v_y  : unsigned(10 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_u  : unsigned(12 downto 0);
        variable v_m  : unsigned(22 downto 0);
        variable v_w  : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 20);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            -- serial multiplier: static a, b shifting right, accumulator shifting right (16 steps after each load)
            if m_cnt /= 0 then
                if m_b(0) = '1' then v_m := ('0' & m_acc(37 downto 16)) + ('0' & m_a); else v_m := '0' & m_acc(37 downto 16); end if;
                m_acc <= '0' & v_m & m_acc(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                m_cnt <= m_cnt - 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    v_y := e_vy(18 downto 8);
                    if s_bead = '1' then v_y(3 downto 0) := "0000"; end if;
                    e_y <= v_y;
                when 2 =>
                    if s_mir = '1' and e_y > c_yk then e_y <= shift_left(c_yk, 1) - e_y; end if;
                    -- the points: a parabolic fillet over +-4 rows of the kink (no crease)
                    v_d := signed(resize(e_y, 12)) - signed(resize(c_yk, 12));
                    if v_d >= -4 and v_d <= 4 then e_fil <= '1'; else e_fil <= '0'; end if;
                    if v_d < -4 then e_dc4 <= (others => '0');
                    elsif v_d > 4 then e_dc4 <= to_unsigned(8, 4);
                    else v_d := v_d + 4; e_dc4 <= unsigned(v_d(3 downto 0)); end if;
                when 3 =>
                    if e_y >= 605 then e_d <= e_y - 605; e_dn <= '0'; else e_d <= 605 - e_y; e_dn <= '1'; end if;
                    if e_y > c_yk then e_bel <= '1'; else e_bel <= '0'; end if;
                    if e_y < 374 then e_ya <= '1'; e_yad <= 374 - e_y; else e_ya <= '0'; e_yad <= e_y - 374; end if;
                when 4 =>
                    m_a <= resize(e_d, 22); m_b <= resize(e_d, 16); m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                when 21 =>
                    m_a <= m_acc(21 downto 0); m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    if e_dn = '1' then m_b <= c_qu; else m_b <= c_ql; end if;
                when 38 =>
                    v_t := resize(c_a, 14) + resize(m_acc(37 downto 16), 14);
                    if m_acc(37 downto 29) /= 0 or v_t > 4095 then e_b <= to_unsigned(4095, 12); else e_b <= v_t(11 downto 0); end if;
                    m_a <= resize(e_d, 22); m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    if e_dn = '1' then m_b <= c_qu; else m_b <= c_ql; end if;
                when 55 =>
                    -- slope (Q2 h/h) -> 1 / sqrt(1 + s^2)
                    if e_bel = '1' then n_ga <= to_unsigned(50, 8);
                    elsif m_acc(26 downto 20) /= 0 then n_ga <= C_GLUT(63);
                    else n_ga <= C_GLUT(to_integer(m_acc(19 downto 14))); end if;
                    if e_ya = '1' then m_a <= resize(e_yad, 22); else m_a <= (others => '0'); end if;
                    m_b <= to_unsigned(278, 16); m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                when 72 =>
                    e_a <= m_acc(16 downto 4);
                    -- wash: + (1/16 + 1/64) per line above row 378, - (1/8 + 1/64) per line below row 864
                    if e_y < 378 then v_y := 378 - e_y; else v_y := (others => '0'); end if;
                    e_w1 <= resize(shift_right(v_y, 4) + shift_right(v_y, 6), 8);
                    if e_y > 1079 then v_y := to_unsigned(215, 11);
                    elsif e_y > 864 then v_y := e_y - 864;
                    else v_y := (others => '0'); end if;
                    e_w2 <= resize(shift_right(v_y, 3) + shift_right(v_y, 6), 8);
                when 106 =>
                    if s_woff = '1' then v_y := (others => '0');
                    else v_y := resize(to_unsigned(48, 8) + e_w1 - e_w2, 11); end if;
                    n_wl <= v_y(7 downto 0);
                    if v_y <= 8 then n_kc <= "00"; elsif v_y <= 32 then n_kc <= "01"; elsif v_y <= 64 then n_kc <= "10"; else n_kc <= "11"; end if;
                    -- the V's offset below the kink row: c = 10 (y - y_k)
                    if e_bel = '1' then
                        v_u := shift_left(resize(e_y - c_yk, 13), 3) + shift_left(resize(e_y - c_yk, 13), 1);
                        if v_u > 4095 then n_c <= to_unsigned(4095, 12); else n_c <= v_u(11 downto 0); end if;
                    else
                        n_c <= (others => '0');
                    end if;
                    -- the band below its dip: no rays, a vertical distance for the glow
                    if e_ya = '1' and e_a /= 0 then
                        n_dyb <= (others => '0');
                    else
                        v_u := shift_left(resize(e_yad, 13), 1) + 2;
                        n_dyb <= v_u(10 downto 0);
                    end if;
                    if e_ya = '1' or e_yad < 32 then n_vb <= '1'; else n_vb <= '0'; end if;
                    -- log2 inputs
                    if e_b = 0 then e_b <= to_unsigned(1, 12); end if;
                    if e_a = 0 or e_ya = '0' then e_a <= to_unsigned(1, 13); end if;
                    -- fillet product 1: SU (Dc + 4)
                    m_a <= resize(c_su, 22); m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                    m_b <= resize(e_dc4, 16);
                when 123 =>
                    e_p1 <= m_acc(14 downto 0);
                    -- product 2: (SU + SV) (Dc + 4)^2, SV = 160 (10 h / line) or SU (Mirror)
                    if s_mir = '1' then m_a <= resize(shift_left(resize(c_su, 13), 1), 22); else m_a <= resize(c_su, 22) + 160; end if;
                    m_b <= resize(e_dc4 * e_dc4, 16); m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                when 140 =>
                    e_p2 <= m_acc(18 downto 0);
                    -- product 3: |50 - GU| (Dc + 4)
                    if c_gu > 50 then m_a <= resize(c_gu - 50, 22); else m_a <= resize(50 - c_gu, 22); end if;
                    m_b <= resize(e_dc4, 16); m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                when 157 =>
                    -- c = (4 SU + ((SU + SV) (Dc + 4)^2 >> 4) - SU (Dc + 4)) >> 4 (never negative),
                    -- g = GU + ((50 - GU) (Dc + 4) >> 3)
                    v_w := shift_left(resize(c_su, 20), 2) + resize(e_p2(18 downto 4), 20) - resize(e_p1, 20);
                    if e_fil = '1' then
                        if v_w(19 downto 16) /= 0 then n_c <= to_unsigned(4095, 12); else n_c <= v_w(15 downto 4); end if;
                        if c_gu > 50 then v_q := -signed(resize(m_acc(11 downto 0), 15));
                        else v_q := signed(resize(m_acc(11 downto 0), 15)); end if;
                        v_q := signed(resize(c_gu, 15)) + shift_right(v_q, 3);
                        n_ga <= unsigned(v_q(7 downto 0));
                    end if;
                when others => null;
            end case;

            -- log2 lookups through the pixel path: inject b at stage 7, then a; read them at stage 11
            if (s_gav_sr(9) = '1' and s_gav_sr(8) = '0') or (s_zone = '1' and s_zc = 1500) then
                e_lcnt <= to_unsigned(1, 4);
            elsif e_lcnt /= 0 and e_lcnt /= 15 then
                e_lcnt <= e_lcnt + 1;
            end if;
            e_inj <= '0';
            if e_lcnt = 1 then e_inj <= '1'; e_injv <= resize(e_b, 13); end if;
            if e_lcnt = 2 then e_inj <= '1'; e_injv <= e_a; end if;
            if e_lcnt = 7 then e_la <= p11_e & p11_lg; end if;
            if e_lcnt = 8 then e_lb <= p11_e & p11_lg; end if;
            if e_lcnt = 9 then
                if e_bel = '1' or e_fil = '1' then n_lba <= c_lbk; else n_lba <= e_la; end if;
                if e_ya = '1' then n_lbb <= e_lb; else n_lbb <= (others => '0'); end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (x in h, hue phase, pixel count) and the late beam (the same, 24
    -- clocks later, with the bead LFSR), per-line values loaded at g_start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_x : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x <= c_xst; b_u <= c_u0; b_n <= (others => '0');
                l_c <= n_c; l_lba <= n_lba; l_lbb <= n_lbb; l_ga <= n_ga; l_dyb <= n_dyb; l_vb <= n_vb;
                l_wl <= n_wl; l_kc <= n_kc;
            elsif g_avid = '1' then
                b_x <= b_x + s_xs; b_u <= b_u + c_usx; b_n <= b_n + 1;
            end if;
            -- late beam: holds pixel k while the pixel path has pixel k at stage 24
            if s_gav_sr(22) = '1' and s_gav_sr(23) = '0' then
                q_x <= c_xst; q_u <= c_u0; q_r <= x"ACE1"; q_n <= (others => '0');
            elsif s_gav_sr(23) = '1' then
                v_x := q_x + s_xs;
                q_x <= v_x; q_u <= q_u + c_usx; q_n <= q_n + 1;
                if c_bsh = '1' then
                    if v_x(11 downto 8) /= q_x(11 downto 8) then q_r <= f_xs16(q_r); end if;
                else
                    if v_x(11 downto 6) /= q_x(11 downto 6) then q_r <= f_xs16(q_r); end if;
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u8  : unsigned(7 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_t   : unsigned(12 downto 0);
        variable v_s   : unsigned(17 downto 0);
        variable v_l   : unsigned(12 downto 0);
        variable v_g   : signed(12 downto 0);
        variable v_a   : unsigned(13 downto 0);
        variable v_b   : unsigned(8 downto 0);
        variable v_ln  : unsigned(7 downto 0);
        variable v_h   : unsigned(7 downto 0);
        variable v_dk  : std_logic;
        variable v_sum : unsigned(9 downto 0);
        variable v_k   : unsigned(8 downto 0);
        variable v_kk  : unsigned(9 downto 0);
        variable v_us  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: sample-and-hold zone (the hue ramp between 0.45 and 0.80, or everywhere) holds x
            v_u8 := b_u(23 downto 16);
            if s_bead = '1' then
                p1_x <= b_x(11 downto 6) & "000000";
            elsif v_u8 >= 115 and v_u8 < 205 then
                p1_x <= b_x(11 downto 4) & "0000";
            else
                p1_x <= b_x;
            end if;
            s_lb_ra <= b_n(10 downto 1);
            -- 2: |x - centre|
            v_d := signed(resize(p1_x, 13)) - C_XC;
            v_d := abs(v_d);
            p2_dx <= unsigned(v_d(11 downto 0));
            r_sf_a <= unsigned(v_d(7 downto 0));
            -- 3: rounded-centre ROM; luma
            p3_dx <= p2_dx;
            if p2_dx < 256 then p3_lt <= '1'; else p3_lt <= '0'; end if;
            p3_lm <= unsigned(s_lb_q);
            -- 4: rounded-centre ROM data (fabric copy); etch partials
            p4_sf <= unsigned(r_sf_q(6 downto 0)); p4_dx <= p3_dx; p4_lt <= p3_lt;
            p4_ea <= p3_lm(4 downto 0) * c_eg;
            p4_eb <= p3_lm(9 downto 5) * c_eg;
            -- 5: T rounded; etch = luma * gain >> 8 (x4 for the Osc)
            if p4_lt = '1' then p5_ts <= to_unsigned(128, 12) + resize(p4_sf, 12); else p5_ts <= p4_dx; end if;
            v_s := resize(p4_ea, 18) + shift_left(resize(p4_eb, 18), 5);
            if s_src = '1' then
                p5_et <= v_s(17 downto 8) & "00";
            else
                p5_et <= resize(v_s(17 downto 8), 12);
            end if;
            -- 6: T' = T + etch
            v_t := resize(p5_ts, 13) + resize(p5_et, 13);
            if v_t > 4095 then p6_tp <= to_unsigned(4095, 12); else p6_tp <= v_t(11 downto 0); end if;
            -- 7: + the V's offset (one T for both combs: the band only lives where it is 0); into the ring.
            --    In blanking the line engine / sequencer inject a value here to borrow the log2.
            if e_inj = '1' then
                p7_t <= e_injv;
            elsif z_inj = '1' then
                p7_t <= resize(c_bk, 13);
            else
                v_t := resize(p6_tp, 13) + resize(l_c, 13);
                if v_t > 4095 then p7_t <= to_unsigned(4095, 13); else p7_t <= v_t; end if;
            end if;
            -- 8: log2 exponent
            p8_e <= f_msb(p7_t);
            p8_t <= p7_t;
            -- 9: log2 mantissa -> ROM
            v_l := shift_left(p8_t, 12 - to_integer(p8_e)); r_lg_a <= v_l(11 downto 4);
            p9_e <= p8_e;
            -- 10: ROM; 11: fabric copy
            p10_e <= p9_e;
            p11_lg <= unsigned(r_lg_q); p11_e <= p10_e;
            -- 12: delta = log2 B - log2 T (Q8), star and band
            p12_da <= signed(resize(l_lba, 13)) - signed(resize(p11_e & p11_lg, 13));
            p12_db <= signed(resize(l_lbb, 13)) - signed(resize(p11_e & p11_lg, 13));
            -- 13: nearest tube (rounded at the linear midpoint), clamped; the band fill window (F = 2 delta)
            v_g := shift_right(p12_da + 34, 6);
            if v_g < 0 then p13_ia <= (others => '0');
            elsif v_g > signed(resize(c_ima, 13)) then p13_ia <= c_ima;
            else p13_ia <= unsigned(v_g(3 downto 0)); end if;
            v_g := shift_right(p12_db + 70, 7);
            if v_g < 0 then p13_ib <= (others => '0');
            elsif v_g > signed(resize(c_imb, 13)) then p13_ib <= c_imb;
            else p13_ib <= unsigned(v_g(3 downto 0)); end if;
            if p12_db <= 77 or p12_db >= 423 then p13_w12 <= (others => '0');
            elsif p12_db < 141 then v_g := shift_left(p12_db, 1) - 154; p13_w12 <= unsigned(v_g(7 downto 0));
            elsif p12_db > 358 then v_g := 845 - shift_left(p12_db, 1); p13_w12 <= unsigned(v_g(7 downto 0));
            else p13_w12 <= to_unsigned(128, 8); end if;
            -- 14: log2 of the tube: L = log2 B - i s (s = 1/4, 1/2 octave) -> exp2 ROMs
            v_d := signed(resize(l_lba, 13)) - signed(resize(p13_ia & "000000", 13));
            if v_d < 0 then v_d := (others => '0'); end if;
            if z_xsel = '1' then r_exa_a <= z_exa; else r_exa_a <= unsigned(v_d(7 downto 0)); end if;
            p14_ea <= unsigned(v_d(11 downto 8));
            v_d := signed(resize(l_lbb, 13)) - signed(resize(p13_ib & "0000000", 13));
            if v_d < 0 then v_d := (others => '0'); end if;
            r_exb_a <= unsigned(v_d(7 downto 0));
            p14_eb <= unsigned(v_d(11 downto 8));
            pi_a(14) <= p13_ia; pi_b(14) <= p13_ib; pw12(14) <= p13_w12;
            for k in 15 to 24 loop pi_a(k) <= pi_a(k - 1); pi_b(k) <= pi_b(k - 1); end loop;
            for k in 15 to 18 loop pw12(k) <= pw12(k - 1); end loop;
            -- 15: ROM; 16: fabric copy
            p15_ea <= p14_ea; p15_eb <= p14_eb;
            p16_xa <= unsigned(r_exa_q); p16_xb <= unsigned(r_exb_q);
            p16_ea <= p15_ea; p16_eb <= p15_eb;
            -- 17: the tubes' T; T from the ring (fabric copy)
            p17_ta <= resize(f_exsh(p16_xa, p16_ea), 13);
            p17_tb <= resize(f_exsh(p16_xb, p16_eb), 13);
            p17_t  <= resize(unsigned(s_rq(11 downto 0)), 13);
            -- 18: distances to the nearest tubes along T; fill ramp; seam test
            if p17_t >= p17_ta then v_t := p17_t - p17_ta; else v_t := p17_ta - p17_t; end if;
            if v_t > 4095 then p18_ea <= to_unsigned(4095, 12); else p18_ea <= v_t(11 downto 0); end if;
            if p17_t >= p17_tb then v_t := p17_t - p17_tb; else v_t := p17_tb - p17_t; end if;
            if v_t > 4095 then p18_eb <= to_unsigned(4095, 12); else p18_eb <= v_t(11 downto 0); end if;
            if p17_t > 1552 then p18_w3 <= to_unsigned(128, 8);
            elsif p17_t > 1040 then v_t := p17_t - 1040; p18_w3 <= v_t(9 downto 2);
            else p18_w3 <= (others => '0'); end if;
            -- 19: x the line's 1/sqrt(1 + slope^2) (star: per line; band: 29/256); fill window min
            p19_al <= p18_ea * l_ga(3 downto 0);
            p19_ah <= p18_ea * l_ga(7 downto 4);
            p19_b0 <= shift_left(resize(p18_eb, 17), 4) + shift_left(resize(p18_eb, 17), 3);
            p19_b1 <= shift_left(resize(p18_eb, 17), 2) + resize(p18_eb, 17);
            if pw12(18) < p18_w3 then p19_m <= pw12(18); else p19_m <= p18_w3; end if;
            -- 20: px distances (h), + the band's distance below its dip; fill = min * 27/32
            v_a := resize(shift_right(resize(p19_al, 20) + shift_left(resize(p19_ah, 20), 4), 8), 14);
            p20_da <= v_a;
            v_a := resize(shift_right(resize(p19_b0, 18) + resize(p19_b1, 18), 8), 14);
            p20_db <= v_a + resize(l_dyb, 14);
            v_b := resize(shift_right(shift_left(resize(p19_m, 13), 4) + shift_left(resize(p19_m, 13), 3) +
                                      shift_left(resize(p19_m, 13), 1) + resize(p19_m, 13), 5), 9);
            p20_bf <= v_b(6 downto 0);
            -- 21: profile ROM addresses (half the distance, saturated)
            if p20_da(13 downto 9) /= 0 then r_ba_a <= (others => '1'); else r_ba_a <= p20_da(8 downto 1); end if;
            if p20_db(13 downto 9) /= 0 then r_bb_a <= (others => '1'); else r_bb_a <= p20_db(8 downto 1); end if;
            if l_vb = '0' then p21_bf <= (others => '0'); else p21_bf <= p20_bf; end if;
            -- 22: ROM; 23: fabric copy (the band only where it lives)
            p22_bf <= p21_bf;
            p23_la <= unsigned(r_ba_q(15 downto 8)); p23_ga <= unsigned(r_ba_q(7 downto 0));
            if l_vb = '1' then p23_lb <= unsigned(r_bb_q(15 downto 8)); p23_gb <= unsigned(r_bb_q(7 downto 0));
            else p23_lb <= (others => '0'); p23_gb <= (others => '0'); end if;
            p23_bf <= p22_bf;
            -- 24: the last tube fades in by quarters of the P12 fraction; glow max
            if c_nfa = '0' or pi_a(23) < c_nia then p24_la <= p23_la;
            else
                case c_nqa is
                    when "01"   => p24_la <= shift_right(p23_la, 2);
                    when "10"   => p24_la <= shift_right(p23_la, 1);
                    when "11"   => p24_la <= p23_la - shift_right(p23_la, 2);
                    when others => p24_la <= (others => '0');
                end case;
            end if;
            if c_nfb = '0' or pi_b(23) < c_nib then p24_lb <= p23_lb;
            else
                case c_nqb is
                    when "01"   => p24_lb <= shift_right(p23_lb, 2);
                    when "10"   => p24_lb <= shift_right(p23_lb, 1);
                    when "11"   => p24_lb <= p23_lb - shift_right(p23_lb, 2);
                    when others => p24_lb <= (others => '0');
                end case;
            end if;
            if p23_ga > p23_gb then p24_gm <= p23_ga; else p24_gm <= p23_gb; end if;
            p24_bf <= p23_bf;
            -- 25: band rays fade inward; beads (late beam: bead LFSR and the zone); glow weight; stripes;
            --     reset-line distance; seam
            v_u8 := q_u(23 downto 16);
            v_dk := '0';
            if s_bead = '1' or (v_u8 >= 115 and v_u8 < 205) then v_dk := '1'; end if;
            v_h := q_r(7 downto 0) + C_HA(to_integer(pi_a(24)));
            if v_dk = '1' and ((s_bead = '1' and v_h(7) = '0') or (s_bead = '0' and v_h < 77)) then
                p25_la <= shift_right(p24_la, 2) + shift_right(p24_la, 3);
            else
                p25_la <= p24_la;
            end if;
            case to_integer(pi_b(24)) is
                when 0 | 1  => v_ln := p24_lb;
                when 2 | 3  => v_ln := p24_lb - shift_right(p24_lb, 2);
                when 4 to 7 => v_ln := shift_right(p24_lb, 1);
                when others => v_ln := shift_right(p24_lb, 2);
            end case;
            v_h := q_r(7 downto 0) + C_HB(to_integer(pi_b(24)));
            if v_dk = '1' and ((s_bead = '1' and v_h(7) = '0') or (s_bead = '0' and v_h < 77)) then
                p25_lb <= shift_right(v_ln, 2) + shift_right(v_ln, 3);
            else
                p25_lb <= v_ln;
            end if;
            p25_gw <= p24_gm * c_gw;
            if v_dk = '1' then
                case q_r(15 downto 14) is
                    when "00"   => p25_bf <= p24_bf;
                    when "01"   => p25_bf <= p24_bf - shift_right(p24_bf, 2);
                    when "10"   => p25_bf <= shift_right(p24_bf, 1);
                    when others => p25_bf <= shift_right(p24_bf, 2);
                end case;
            else
                p25_bf <= p24_bf;
            end if;
            v_us := signed(q_u(23 downto 12));
            v_us := abs(v_us);
            p25_au <= unsigned(v_us);
            -- 26: tube max, glow, reset line
            if p25_la > p25_lb then p26_lm <= p25_la; else p26_lm <= p25_lb; end if;
            p26_gw <= p25_gw(10 downto 4);
            p26_bf <= p25_bf;
            -- the reset line's level and its wash attenuation (constants: no adder on the level)
            if p25_au < c_vt1 then
                p26_vl <= to_unsigned(140, 8);
                case l_kc is
                    when "00" => p26_va <= to_unsigned(0, 8);  when "01" => p26_va <= to_unsigned(8, 8);
                    when "10" => p26_va <= to_unsigned(25, 8); when others => p26_va <= to_unsigned(43, 8);
                end case;
            elsif p25_au < c_vt2 then
                p26_vl <= to_unsigned(60, 8);
                case l_kc is
                    when "00" => p26_va <= to_unsigned(0, 8);  when "01" => p26_va <= to_unsigned(3, 8);
                    when "10" => p26_va <= to_unsigned(10, 8); when others => p26_va <= to_unsigned(18, 8);
                end case;
            else
                p26_vl <= (others => '0'); p26_va <= (others => '0');
            end if;
            -- 27: intensity = 13/16 tube + glow + fill
            v_sum := resize(shift_right(resize(p26_lm, 10) * 13, 4), 10) + resize(p26_gw, 10) + resize(p26_bf, 10);
            if v_sum > 255 then p27_i <= to_unsigned(255, 8); else p27_i <= v_sum(7 downto 0); end if;
            p27_vl <= p26_vl; p27_va <= p26_va;
            -- 28: the reset line over the tubes; the per-line wash attenuation of each candidate
            if p27_vl > p27_i then p28_v <= p27_vl; else p28_v <= p27_i; end if;
            case l_kc is
                when "00"   => v_h := (others => '0');
                when "01"   => v_h := shift_right(p27_i, 4);
                when "10"   => v_h := shift_right(p27_i, 3) + shift_right(p27_i, 4);
                when others => v_h := shift_right(p27_i, 2) + shift_right(p27_i, 4);
            end case;
            if p27_vl > p27_i then p28_at <= p27_va; else p28_at <= v_h; end if;
            -- 29: k = wash + (1 - wash) intensity
            v_kk := resize(l_wl, 10) + resize(p28_v, 10) - resize(p28_at, 10);
            if v_kk > 255 then p28_k <= to_unsigned(255, 8); else p28_k <= v_kk(7 downto 0); end if;
            -- 30: colour LUT address {u7, k4}, dithered with a period-8 pattern (cancelled by the box); u and the
            --     dither come straight from the late beam, 5 px ahead of this pixel (invisible on the 2200 px ramp)
            v_k := resize(p28_k, 9) + shift_left(resize(q_n, 9), 1);
            if v_k(8 downto 4) > 15 then v_k := to_unsigned(255, 9); end if;
            v_u8 := resize(shift_right(resize(q_u(23 downto 14), 11) + resize(q_n, 11), 3), 8);
            p30_a <= v_u8(6 downto 0) & v_k(7 downto 4);
            -- 31, 32: pads (latency 40); 33: ROM address
            p31_a <= p30_a; p32_a <= p31_a; r_c_a <= p32_a;
            -- 34: ROM; 35: fabric copy; 36: compose (neutral black outside the generated avid)
            p35_y <= unsigned(r_cy_q); p35_u <= unsigned(r_cu_q); p35_v <= unsigned(r_cv_q);
            if s_gav_sr(34) = '0' then
                s_y32 <= to_unsigned(64, 10); s_u32 <= to_unsigned(512, 10); s_v32 <= to_unsigned(512, 10);
            else
                s_y32 <= p35_y & "10"; s_u32 <= p35_u & "00";
                if s_emb = '1' then s_v32 <= (not (p35_v & "00")) + 1; else s_v32 <= p35_v & "00"; end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(34);
            -- ring write (stage-7 T) and read pointer (data for stage 17)
            s_rwp <= s_rwp + 1;
            s_rra <= s_rwp - 7;
            s_sra <= s_rwp - (C_LAT - 3);
        end if;
    end process p_pix;
    s_rw_d <= "0000" & std_logic_vector(p7_t(11 downto 0));

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box
    -- (two adjacent 8-sums: the still's horizontal softness, the dither cancel
    -- and the HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 37 sums, 38 output; the sums snap back to neutral once 8 blank samples are in
            s_cay(0) <= s_y32; s_cau(0) <= s_u32(9 downto 2); s_cav(0) <= s_v32(9 downto 2);
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(1024, 11); s_bav <= to_unsigned(1024, 11);
            else
                s_bay <= s_bay + s_y32 - s_cay(7);
                s_bau <= s_bau + s_u32(9 downto 2) - s_cau(7);
                s_bav <= s_bav + s_v32(9 downto 2) - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(36) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y34 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u34 <= v_sc(11 downto 2);
                v_sc := resize(s_bav, 14) + s_bavd; s_v34 <= v_sc(11 downto 2);
            else
                s_y34 <= to_unsigned(64, 10); s_u34 <= to_unsigned(512, 10); s_v34 <= to_unsigned(512, 10);
            end if;
            -- 39: luma 13/16
            if s_y34 > 64 then v_dl := s_y34 - 64; else v_dl := (others => '0'); end if;
            s_y35 <= s_y34 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u35 <= s_u34; s_v35 <= s_v34;
            -- 40: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y36 <= s_y35; s_u36 <= s_u35; s_v36 <= s_v35;
            else
                s_y36 <= to_unsigned(64, 10); s_u36 <= to_unsigned(512, 10); s_v36 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y36);
    data_out.u       <= std_logic_vector(s_u36);
    data_out.v       <= std_logic_vector(s_v36);
    data_out.hsync_n <= s_so(0);
    data_out.vsync_n <= s_so(1);
    data_out.field_n <= s_so(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture starline;
