-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Vine v0.1.0 -- Heirloom (video phase), from the clip WIN_20231026_23_13_39 at 0.5 s.
--
-- The still: four blue vines (~134 px wide) swaying down a green ground that is banded by a vertical sine (black
-- bands at rows ~20, 520 and 1040) and darkens toward the right.  The vines carry soft violet / magenta blossoms,
-- a dark ragged rim with short black flame spikes, pale lavender tongues reaching out of their left sides, and
-- small yellow-green / cyan / white leaves floating beside them.  The sway grows from left to right: a horizontal
-- oscillator reset near the line start whose FREQUENCY is swept by a vertical sine (FM).  In the clip the sway
-- wave scrolls DOWN steadily (one wave per 8.45 s), the ground stays still and the blossoms flow and morph.
--
-- The patch, as fitted (vine.py has the tables):
--   phi    = (x + 160) F(y),  F = F0 (1 + m sin(2 pi y / 1125 + theta))     cycles; vine centres at phi = k + 0.688
--   u      = frac(phi - 0.688 + 1/2) - 1/2 (+ Etch * luma)                  0 at a vine centre
--   vine   key = (W - |u|) k_side + N1 + N2 > 0       k_side = 3.125 (left) / 4.5 (right)
--          N1 = blossom noise (44 x 28 virtual px cells), N2 = flame noise (28 x 9), both value noise: per lattice
--          row an xorshift16 stream seeded by a hash of the row, smoothstep down the rows, linear along x
--   rim    -256 < key <= 0: dark blue fading to black; spikes and tongues where the flame noise's peaks push a
--          gentler key (slope 2.625, x1.8 on the right) past 0: dark near the vine, pale lavender / white / cyan
--          farther out (or where the blossom noise is bright); P12 lowers the spikes' onset and raises their reach
--   leaves flame-noise peaks gated by the blossom noise in a band beside the vines
--   ground G = profile(y) - 116 (x / 1900)^2, clipped
--
--   K1  Vines    vine count x0.35 .. x2.8 (512 = the clip, ~3.5 across)
--   K2  Sway     sway depth: straight vines (0) .. the clip (512) .. deep folding arcs (1023)
--   K3  Wave     sway waves down the frame x0.25 .. x4 (512 = the clip, one wave per frame)
--   K4  Speed    bipolar: the sway scrolls down (+) or up (-); 700 = the clip (one wave per 8.45 s)
--   K5  Girth    vine width: hairline stems (0) .. the clip (512) .. a blue wall with green slits (1023)
--   K6  Etch     how far the camera (or the Osc) bends and swells the vines
--   S7  Zigzag   sine sway -> triangle sway at 2.5x depth: sharp chevrons
--   S8  Night    the ground inverted: black, with green glow bands where the clip has black ones
--   S9  Mirror   the vines reflected about the centre: the sway mirrored, the vines curving toward each other
--   S10 Palette  Vine (the clip) / Ember (orange vines, yellow blossoms, deep blue ground)
--   S11 Source   Video / Osc
--   P12 FLAME    calm smooth vines, no leaves (0) .. the clip (409) .. burning: long flame tongues, shredded
--                edges, a swarm of leaves (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide); a per-line engine kicked at each line start
-- (computes the NEXT line: the sway sine, F, the phase start value and step, per noise octave two row hashes and
-- the first three column values, the ground level); a beam with the phase DDA (reversed at the centre under
-- Mirror) and two noise octaves (two row streams each and a serial multiplier for the next column value); a
-- 21-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (U and V through a [1 2 2 2 2 2 2 2 1]/16 box with its history in a block-RAM ring and a resync,
-- luma [1 2 1]/4 (delayed to sit on the chroma) and 13/16, then a vertical [1 1]/2 on luma so no feature is ever a single row) before the
-- blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.vine_pkg.all;
use work.all;

architecture vine of program_top is
    constant C_LAT  : integer := 20;   -- = processing_delay_clks
    constant C_TH0  : integer := 6910685;    -- the still's sway phase (Q24 cycles)
    constant C_C0Q  : integer := 13626455;   -- (1/2 - 0.6878) mod 1, Q24: vine centres at phi = k + 0.6878
    constant C_GPH0 : integer := 66787;      -- ground profile index at row 0, Q8
    constant C_SALTA : unsigned(15 downto 0) := x"5A3C";
    constant C_SALTB : unsigned(15 downto 0) := x"C3A5";

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
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    function f_xs16(x : unsigned(15 downto 0)) return unsigned is
        variable a, b, c : unsigned(15 downto 0);
    begin
        a := x xor shift_left(x, 7);
        b := a xor shift_right(a, 9);
        c := b xor shift_left(b, 8);
        return c;
    end function;
    -- smoothstep, 6 bits -> 6 bits
    type t_ss is array (0 to 63) of integer range 0 to 63;
    constant C_SS : t_ss := (0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 19, 20, 22,
        23, 25, 26, 28, 29, 31, 32, 33, 35, 36, 38, 39, 41, 42, 44, 45, 47, 48, 49, 50, 52, 53, 54, 55, 56, 57, 58,
        59, 60, 61, 61, 62, 62, 63, 63, 63, 63, 63);

    -- line buffer: (luma >> 2) - 16 clipped at 0, or the Osc lattice signed about its mean; every other pixel
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_pa  : unsigned(9 downto 0) := (others => '0');
    signal r_pq  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_sn_a, r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_sn_q, r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_gr_a : unsigned(9 downto 0) := (others => '0');
    signal r_gr_q : unsigned(7 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys, s_ysf : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_k2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_k5 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(409 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_k6 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_zig, s_night, s_mirror, s_pal, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier
    signal f_a   : unsigned(33 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(33 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : signed(10 downto 0) := (others => '0');
    signal z_sh1, z_sh3 : signed(2 downto 0) := (others => '0');
    signal z_mt1 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_hi  : std_logic := '0';
    signal z_dp  : unsigned(17 downto 0) := (others => '0');
    signal z_vsq : unsigned(17 downto 0) := (others => '0');
    signal z_flm : unsigned(9 downto 0) := (others => '0');
    signal z_flc : unsigned(8 downto 0) := (others => '0');

    -- per-field constants (defaults = the clip at 1080p)
    signal c_f0  : unsigned(15 downto 0) := to_unsigned(15071, 16);   -- F0 per h, Q24 cycles
    signal c_m   : unsigned(14 downto 0) := to_unsigned(2130, 15);    -- sway depth, Q14
    signal c_fm  : unsigned(16 downto 0) := to_unsigned(1959, 17);    -- F0 * m
    signal c_dpl : unsigned(19 downto 0) := to_unsigned(14913, 20);   -- sway phase per line, Q24 cycles
    signal c_dth : unsigned(19 downto 0) := to_unsigned(33032, 20);   -- sway scroll per field
    signal c_w   : unsigned(10 downto 0) := to_unsigned(495, 11);     -- vine half-width, Q12 cycles
    signal c_fl  : unsigned(9 downto 0) := to_unsigned(256, 10);      -- flame, Q8
    signal c_g2  : unsigned(7 downto 0) := to_unsigned(128, 8);       -- flame-noise gain into the key, Q4
    signal c_g3  : unsigned(9 downto 0) := to_unsigned(320, 10);      -- flame-spike gain, Q4
    signal c_vb  : unsigned(15 downto 0) := to_unsigned(20000, 16);   -- flame-noise scroll per field
    signal c_lt  : unsigned(7 downto 0) := to_unsigned(172, 8);       -- leaf threshold (n2)
    signal c_lg  : unsigned(7 downto 0) := to_unsigned(150, 8);       -- leaf gate (n1)
    signal c_lc  : unsigned(11 downto 0) := to_unsigned(1311, 12);    -- leaf band centre, Q12
    signal c_r0  : unsigned(7 downto 0) := to_unsigned(159, 8);       -- flame-spike onset (n2)
    signal c_th  : unsigned(23 downto 0) := to_unsigned(C_TH0, 24);   -- sway phase at row 0
    signal c_ra0, c_rb0 : unsigned(25 downto 0) := (others => '0');   -- noise row positions at row 0 (Q16 rows)
    -- per-mode constants
    signal c_sta : unsigned(10 downto 0) := to_unsigned(372, 11);     -- blossom-noise cell step per px (Q14)
    signal c_stb : unsigned(10 downto 0) := to_unsigned(585, 11);     -- flame-noise cell step per px
    signal c_da  : unsigned(13 downto 0) := to_unsigned(2341, 14);    -- blossom-noise rows per line (Q16)
    signal c_db  : unsigned(15 downto 0) := to_unsigned(7282, 16);    -- flame-noise rows per line (9-row cells)
    signal c_gst : unsigned(10 downto 0) := to_unsigned(250, 11);     -- ground profile index per line (Q8)
    signal c_x0  : unsigned(8 downto 0) := (others => '0');           -- first px, virtual x in h (SD 480)
    signal z_v1  : unsigned(14 downto 0) := (others => '0');
    signal c_xo  : unsigned(9 downto 0) := to_unsigned(320, 10);      -- first px + 160 virtual px, in h
    signal c_m1  : unsigned(10 downto 0) := to_unsigned(959, 11);     -- Mirror: centre pixel - 1

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc  : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk  : std_logic := '0';
    signal e_psi : unsigned(23 downto 0) := (others => '0');
    signal e_ra, e_rb : unsigned(25 downto 0) := (others => '0');   -- noise row positions, Q16 (10-bit row index)
    signal e_gph : unsigned(16 downto 0) := (others => '0');
    signal e_q   : unsigned(1 downto 0) := (others => '0');
    signal e_swm : unsigned(14 downto 0) := (others => '0');
    signal e_sgn : std_logic := '0';
    signal e_fv  : signed(18 downto 0) := (others => '0');
    signal e_h   : unsigned(31 downto 0) := (others => '0');
    signal e_s   : unsigned(15 downto 0) := (others => '0');
    signal e_t0, e_t1, e_t2, e_b0, e_b1, e_b2 : unsigned(7 downto 0) := (others => '0');
    signal m_a   : unsigned(31 downto 0) := (others => '0');
    signal m_b   : unsigned(15 downto 0) := (others => '0');
    signal m_acc : unsigned(31 downto 0) := (others => '0');
    -- next line (loaded by the beam at g_start)
    signal n_fs, n_st : unsigned(23 downto 0) := (others => '0');
    type t_u16x2 is array (0 to 1) of unsigned(15 downto 0);
    type t_u8x2  is array (0 to 1) of unsigned(7 downto 0);
    type t_u8x6  is array (0 to 5) of unsigned(7 downto 0);
    signal n_lt, n_lb : t_u16x2 := (others => x"0001");        -- row streams' state at cell 2 (octave A, B)
    signal n_c  : t_u8x6 := (others => x"80");                 -- column values C0, C1, C2 of octave A, then B
    signal n_wya, n_wyb : unsigned(3 downto 0) := (others => '0');
    signal n_gl : unsigned(7 downto 0) := (others => '0');

    -- beam (stage 1): phase, two noise octaves (cell accumulator, top / bottom row streams, column values
    -- C0 C1 C2, a serial multiplier for the next column value), pixel count
    signal b_ph, l_st : unsigned(23 downto 0) := (others => '0');
    signal b_rev : std_logic := '0';
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    type t_u14x2 is array (0 to 1) of unsigned(13 downto 0);
    type t_u4x2  is array (0 to 1) of unsigned(3 downto 0);
    signal o_acc : t_u14x2 := (others => (others => '0'));
    signal o_lt, o_lb : t_u16x2 := (others => x"0001");
    signal o_c0, o_c1, o_c2, y_base : t_u8x2 := (others => x"80");
    type t_s9x2b is array (0 to 1) of signed(8 downto 0);
    type t_s14x2 is array (0 to 1) of signed(13 downto 0);
    signal y_dd  : t_s9x2b := (others => (others => '0'));
    signal y_p   : t_s14x2 := (others => (others => '0'));
    signal y_go1, y_go2, y_go3 : std_logic_vector(1 downto 0) := "00";
    signal l_wya, l_wyb : unsigned(3 downto 0) := (others => '0');
    signal l_gl  : unsigned(7 downto 0) := (others => '0');
    -- ground darkening by x: virtual x of the stage-8 pixel (h), ROM address (stage 9), data (stage 10)
    signal gx    : unsigned(11 downto 0) := (others => '0');
    signal r_dx_a : unsigned(7 downto 0) := (others => '0');
    signal r_dx_q : unsigned(7 downto 0) := (others => '0');

    -- pixel path (register pN holds stage N)
    type t_s9x2  is array (0 to 1) of signed(8 downto 0);
    type t_s16x2 is array (0 to 1) of signed(15 downto 0);
    signal p2_dx : t_s9x2 := (others => (others => '0'));
    signal p2_c0, p3_c0 : t_u8x2 := (others => x"80");
    signal p2_wx : t_u4x2 := (others => (others => '0'));
    signal p3_p  : t_s14x2 := (others => (others => '0'));
    signal p2_v, p3_v : unsigned(11 downto 0) := (others => '0');
    signal p2_ep : signed(14 downto 0) := (others => '0');
    signal p3_du : signed(13 downto 0) := (others => '0');
    signal p4_v  : unsigned(11 downto 0) := (others => '0');
    signal p4_w, p5_w, p6_w, p7_w, p8_w : unsigned(10 downto 0) := (others => '0');
    signal p4_n1, p5_n1, p6_n1, p7_n1, p8_n1, p9_n1, p10_n1, p11_n1 : unsigned(7 downto 0) := (others => '0');
    signal p4_n2, p5_n2, p6_n2, p7_n2, p8_n2, p9_n2, p10_n2, p11_n2 : unsigned(7 downto 0) := (others => '0');
    signal p5_a, p6_a, p7_a : unsigned(11 downto 0) := (others => '0');
    signal p5_l, p6_l, p7_l, p8_l, p9_l, p10_l, p11_l : std_logic := '0';
    signal p5_n1x : signed(8 downto 0) := (others => '0');
    signal p5_n2p : signed(13 downto 0) := (others => '0');
    signal p5_ep  : unsigned(12 downto 0) := (others => '0');
    signal p6_d   : signed(12 downto 0) := (others => '0');
    signal p6_n1k : signed(10 downto 0) := (others => '0');
    signal p6_n2k : signed(12 downto 0) := (others => '0');
    signal p6_ep  : unsigned(13 downto 0) := (others => '0');
    signal p6_epz, p7_epz, p8_epz, p9_epz, p10_epz, p11_epz : std_logic := '0';
    signal p6_ar, p7_ar : unsigned(12 downto 0) := (others => '0');
    signal p7_b   : signed(15 downto 0) := (others => '0');
    signal p7_n12 : signed(14 downto 0) := (others => '0');
    signal p7_n1e, p8_n1e, p9_n1e, p10_n1e : signed(14 downto 0) := (others => '0');
    signal p8_kv  : signed(16 downto 0) := (others => '0');
    signal p8_du  : unsigned(12 downto 0) := (others => '0');
    signal p9_dt, p9_ld : signed(13 downto 0) := (others => '0');
    signal p9_vine, p10_vine, p11_vine : std_logic := '0';
    signal p9_rim, p10_rim, p11_rim, p12_rim : std_logic := '0';
    signal p9_rd, p10_rd, p11_rd, p12_rd : unsigned(3 downto 0) := (others => '0');
    signal p10_ekp : signed(16 downto 0) := (others => '0');
    signal p10_em, p11_em, p12_em : unsigned(5 downto 0) := (others => '0');
    signal p10_ald : unsigned(9 downto 0) := (others => '0');
    signal p11_ek  : signed(17 downto 0) := (others => '0');
    signal p11_lv  : signed(10 downto 0) := (others => '0');
    signal p11_g, p12_g : unsigned(7 downto 0) := (others => '0');
    signal p12_vine, p12_ext, p12_dark, p12_leaf : std_logic := '0';
    signal p12_ts, p12_ls : unsigned(5 downto 0) := (others => '0');
    signal p12_vc : unsigned(6 downto 0) := (others => '0');
    signal p16_q : std_logic_vector(31 downto 0) := (others => '0');
    signal p13_pa : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma box history in a 256 x 20 block-RAM ring (read 8 samples back), luma [1 2 1]
    type t_bx is array (0 to 255) of std_logic_vector(19 downto 0);
    signal bx : t_bx := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of bx : signal is true;
    signal s_bwp : unsigned(7 downto 0) := (others => '0');
    signal s_bxq : std_logic_vector(19 downto 0) := (others => '0');
    signal s_bxd : std_logic_vector(19 downto 0) := (others => '0');
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yh  : t_u10(1 to 3) := (others => to_unsigned(64, 10));
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- vertical [1 1]/2 on the output luma: the previous line's luma (>> 2) in a 2048 x 8 block-RAM line, read two
    -- pixels ahead of the write (never the same address)
    type t_vl is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal vl : t_vl := (others => x"00");
    attribute no_rw_check of vl : signal is true;
    signal s_vc  : unsigned(10 downto 0) := (others => '0');
    signal s_vq, s_vq2 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_vfirst : std_logic := '1';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), chroma ring, ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_bx : process(clk)
    begin
        if rising_edge(clk) then
            s_bxq <= bx(to_integer(s_bwp - 6));
            bx(to_integer(s_bwp)) <= std_logic_vector(s_u17) & std_logic_vector(s_v17);
            s_bwp <= s_bwp + 1;
        end if;
    end process p_bx;

    p_vl : process(clk)
    begin
        if rising_edge(clk) then
            s_vq <= vl(to_integer(s_vc + 2));
            if s_gav_sr(18) = '1' then vl(to_integer(s_vc)) <= std_logic_vector(s_y20(9 downto 2)); end if;
        end if;
    end process p_vl;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_pq   <= C_VN_PAL(to_integer(r_pa));
            r_sn_q <= C_VN_SIN(to_integer(r_sn_a));
            r_ex_q <= C_VN_EXP2(to_integer(r_ex_a));
            r_gr_q <= C_VN_GRN(to_integer(r_gr_a));
            r_dx_q <= C_VN_DX(to_integer(r_dx_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(12 downto 0);
        variable v_u  : unsigned(19 downto 0);
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
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            if v_s > 0 then s_vneg <= '0'; else s_vneg <= '1'; end if;
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
                s_zig   <= registers_in(6)(0);
                s_night <= registers_in(6)(1);
                s_mirror <= registers_in(6)(2);
                s_pal   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_k6    <= unsigned(registers_in(5));
                s_k2    <= unsigned(registers_in(1));
                s_k5    <= unsigned(registers_in(4));
                s_xs    <= s_xsr;
                s_ysf   <= s_ys;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            c_m1 <= resize(s_w(11 downto 1), 11) - 1;

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run from the zone
            -- start by its own phase / channel counters (no zc decode on these enables)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
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
                            when 1      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from load + 17
            -- (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands pre-muxed at
            -- 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(15071, 18);          z_fb <= z_mt1;                         -- F0
                    when 1  =>                                                                                  -- sway depth
                        if s_k2 > 512 then
                            z_fa <= resize(s_k2 - 512, 18); z_fb <= to_unsigned(20357, 16); z_hi <= '1';
                        else
                            z_fa <= resize(s_k2, 18);       z_fb <= to_unsigned(4260, 16);  z_hi <= '0';
                        end if;
                    when 2  => z_fa <= resize(c_f0, 18);                z_fb <= resize(c_m, 16);               -- F0 m
                    when 3  => z_fa <= to_unsigned(14913, 18);          z_fb <= z_mt1;                         -- wave (mt of e3 by now)
                    when 4  => z_fa <= resize(z_dp, 18);                z_fb <= resize(s_ysf, 16);             -- per line
                    when 5  => z_fa <= resize(s_vm, 18);                z_fb <= resize(s_vm, 16);              -- speed^2
                    when 6  =>                                                                                  -- girth
                        if s_k5 > 512 then
                            z_fa <= resize(s_k5 - 512, 18); z_fb <= to_unsigned(2298, 16); z_hi <= '1';
                        else
                            z_fa <= resize(s_k5, 18);       z_fb <= to_unsigned(827, 16);  z_hi <= '0';
                        end if;
                    when 7  =>                                                                                  -- flame
                        if s_gp(12 downto 3) > 409 then
                            z_fa <= resize(s_gp(12 downto 3) - 409, 18); z_fb <= to_unsigned(854, 16); z_hi <= '1';
                        else
                            z_fa <= resize(s_gp(12 downto 3), 18);       z_fb <= to_unsigned(641, 16); z_hi <= '0';
                        end if;
                    when others => z_fa <= resize(z_flc, 18);           z_fb <= to_unsigned(77, 16);      -- leaf threshold
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 34); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(32 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps), per-mode constants
                    when 4 =>
                        if s_vneg = '1' then c_th <= c_th + resize(c_dth, 24); else c_th <= c_th - resize(c_dth, 24); end if;
                        c_ra0 <= c_ra0 - 2341;
                        c_rb0 <= c_rb0 - resize(c_vb, 26);
                    when 5 =>
                        case to_integer(s_xs) is
                            when 4 =>
                                c_sta <= to_unsigned(745, 11); c_stb <= to_unsigned(1170, 11);
                                c_x0 <= to_unsigned(480, 9); c_xo <= to_unsigned(800, 10);
                            when 3 =>
                                c_sta <= to_unsigned(559, 11); c_stb <= to_unsigned(878, 11);
                                c_x0 <= (others => '0'); c_xo <= to_unsigned(320, 10);
                            when others =>
                                c_sta <= to_unsigned(372, 11); c_stb <= to_unsigned(585, 11);
                                c_x0 <= (others => '0'); c_xo <= to_unsigned(320, 10);
                        end case;
                    when 6 =>
                        case to_integer(s_ysf) is
                            when 384  => c_da <= to_unsigned(3511, 14);  c_db <= to_unsigned(10923, 16); c_gst <= to_unsigned(375, 11);  s_oys <= to_unsigned(181, 10);
                            when 512  => c_da <= to_unsigned(4681, 14);  c_db <= to_unsigned(14564, 16); c_gst <= to_unsigned(500, 11);  s_oys <= to_unsigned(242, 10);
                            when 576  => c_da <= to_unsigned(5266, 14);  c_db <= to_unsigned(16384, 16); c_gst <= to_unsigned(563, 11);  s_oys <= to_unsigned(272, 10);
                            when 960  => c_da <= to_unsigned(8777, 14);  c_db <= to_unsigned(27307, 16); c_gst <= to_unsigned(938, 11);  s_oys <= to_unsigned(453, 10);
                            when 1138 => c_da <= to_unsigned(10405, 14); c_db <= to_unsigned(32370, 16); c_gst <= to_unsigned(1112, 11); s_oys <= to_unsigned(537, 10);
                            when 480  => c_da <= to_unsigned(4389, 14);  c_db <= to_unsigned(13653, 16); c_gst <= to_unsigned(469, 11);  s_oys <= to_unsigned(226, 10);
                            when others => c_da <= to_unsigned(2341, 14); c_db <= to_unsigned(7282, 16); c_gst <= to_unsigned(250, 11); s_oys <= to_unsigned(121, 10);
                        end case;

                    -- exponents: vines e1 = 3 (k1 - 512) / 4 (+-1.5 octaves), wave e3 = k3 - 512 (+-2 octaves)
                    when 12 =>
                        v_e := signed(resize(s_g1(12 downto 3), 13)) - 512;
                        v_e := v_e + shift_left(v_e, 1);
                        z_e <= resize(shift_right(v_e, 2), 11);
                    when 13 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh1 <= z_e(10 downto 8);
                    when 16 => z_mt1 <= r_ex_q;
                    when 40 => z_e <= signed(resize(s_g3(12 downto 3), 11)) - 512;
                    when 41 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh3 <= z_e(10 downto 8);
                    when 44 => z_mt1 <= r_ex_q;

                    when 50 =>                       -- F0 = 15071 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_f0 <= f_acc(32 downto 17);
                            when -1     => c_f0 <= f_acc(31 downto 16);
                            when 0      => c_f0 <= f_acc(30 downto 15);
                            when others => c_f0 <= f_acc(29 downto 14);
                        end case;
                    when 74 =>                       -- m (Q14): k2 * 4260 >> 10, or 2130 + (k2 - 512) * 20357 >> 10
                        if z_hi = '1' then c_m <= resize(f_acc(23 downto 10), 15) + 2130;
                        else c_m <= resize(f_acc(23 downto 10), 15); end if;
                    when 75 =>                       -- Zigzag: 2.5 x depth
                        if s_zig = '1' then c_m <= shift_left(c_m, 1) + shift_right(c_m, 1); end if;
                    when 98 => c_fm <= f_acc(30 downto 14);
                    when 122 =>                      -- Dpsi = 14913 mt3 >> (15 - sh3)
                        case to_integer(z_sh3) is
                            when -2     => z_dp <= resize(f_acc(31 downto 17), 18);
                            when -1     => z_dp <= resize(f_acc(31 downto 16), 18);
                            when 0      => z_dp <= resize(f_acc(31 downto 15), 18);
                            when others => z_dp <= f_acc(31 downto 14);
                        end case;
                    when 146 => c_dpl <= f_acc(27 downto 8);
                    when 170 => z_vsq <= f_acc(17 downto 0);
                    when 171 =>
                        v_u := resize(z_vsq, 20) + resize(z_vsq(17 downto 6), 20) + resize(z_vsq(17 downto 8), 20);
                        c_dth <= v_u;
                    when 194 =>                      -- W: 82 + k5 * 827 >> 10, or 495 + (k5 - 512) * 2298 >> 10
                        if z_hi = '1' then c_w <= resize(f_acc(20 downto 10), 11) + 495;
                        else c_w <= resize(f_acc(20 downto 10), 11) + 82; end if;
                    when 218 =>                      -- fl (Q8): p12 * 641 >> 10, or 256 + (p12 - 409) * 854 >> 10
                        if z_hi = '1' then c_fl <= resize(f_acc(19 downto 10), 10) + 256;
                        else c_fl <= resize(f_acc(19 downto 10), 10); end if;
                    when 219 =>
                        if c_fl > 410 then c_g2 <= to_unsigned(205, 8); else c_g2 <= c_fl(8 downto 1); end if;
                        c_g3 <= c_fl + shift_right(c_fl, 2);
                        if c_fl > 256 then z_flm <= c_fl - 256; else z_flm <= (others => '0'); end if;
                        if c_fl > 256 then z_flc <= (others => '0'); else z_flc <= resize(256 - c_fl, 9); end if;
                    -- flame-dependent onsets (shift-adds): spikes 159 - flm / 8, leaf gate 150 - flm / 16, leaf band
                    -- 1311 + 0.625 flm, flame-noise scroll 10000 + 39 fl
                    when 220 => c_r0 <= 159 - resize(z_flm(9 downto 3), 8);
                    when 221 => c_lg <= 150 - resize(z_flm(9 downto 4), 8);
                    when 222 => z_v1 <= resize(shift_left(resize(c_fl, 15), 5), 15) + resize(shift_left(resize(c_fl, 15), 3), 15);
                    when 223 => c_vb <= resize(z_v1, 16) + 10000 - resize(c_fl, 16);
                    when 224 => c_lc <= 1311 + resize(z_flm(9 downto 1), 12);
                    when 225 => c_lc <= c_lc + resize(z_flm(9 downto 3), 12);
                    when 266 => c_lt <= 172 + f_acc(15 downto 8) - resize(z_flm(9 downto 4), 8);
                    when others => null;
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write (every other pixel)
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
            s_lb_we <= data_in.avid and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_osum - 111);           -- signed about the lattice's mean (bipolar bend)
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (signed multiplicand,
    -- result at load + 17, or load + 7 for a 6-bit multiplier): the sway sine
    -- (or triangle), F, the phase start value and step; per noise octave the two
    -- row hashes, the row streams' first three lattice values and the column
    -- values C0..C2 = Lt + (Lb - Lt) wy; the ground level
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_i : unsigned(9 downto 0);
        variable v_f : unsigned(15 downto 0);
        variable v_p : signed(18 downto 0);
        variable v_s : unsigned(15 downto 0);
        variable v_j : unsigned(15 downto 0);
        variable v_c : signed(9 downto 0);
        variable v_d : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 400 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_psi <= c_th; e_ra <= c_ra0; e_rb <= c_rb0; e_gph <= to_unsigned(C_GPH0, 17);
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_psi <= e_psi + resize(c_dpl, 24);
                e_ra <= e_ra + resize(c_da, 26);
                e_rb <= e_rb + resize(c_db, 26);
                e_gph <= e_gph + resize(c_gst, 17);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- column value: C = Lt + (product >> 4), product = (Lb - Lt) wy (4-bit weight)
            v_c := signed(resize(e_t0, 10)) + resize(signed(m_acc(12 downto 4)), 10);
            case to_integer(e_lc) is
                when 1 =>
                    v_i := e_psi(23 downto 14);
                    e_q <= v_i(9 downto 8);
                    if v_i(8) = '1' then r_sn_a <= 255 - v_i(7 downto 0); else r_sn_a <= v_i(7 downto 0); end if;
                    r_gr_a <= s_night & e_gph(16 downto 8);
                    n_wya <= to_unsigned(C_SS(to_integer(e_ra(15 downto 10))) / 4, 4);
                    n_wyb <= to_unsigned(C_SS(to_integer(e_rb(15 downto 10))) / 4, 4);
                when 4 =>
                    v_f := e_psi(21 downto 6);
                    if s_zig = '0' then e_swm <= r_sn_q(14 downto 0);
                    elsif e_psi(22) = '1' then e_swm <= 32767 - resize(v_f(15 downto 1), 15);
                    else e_swm <= v_f(15 downto 1); end if;
                    if s_zig = '1' then e_sgn <= e_psi(23); else e_sgn <= e_q(1); end if;
                    n_gl <= r_gr_q;
                when 5 =>
                    m_a <= resize(c_fm, 32); m_b <= '0' & e_swm; m_acc <= (others => '0');
                when 22 =>
                    v_p := signed(resize(m_acc(31 downto 15), 19));
                    if e_sgn = '1' then e_fv <= signed(resize(c_f0, 19)) - v_p;
                    else e_fv <= signed(resize(c_f0, 19)) + v_p; end if;
                when 23 =>
                    m_a <= unsigned(resize(e_fv, 32)); m_b <= resize(c_xo, 16); m_acc <= (others => '0');
                    case to_integer(s_xs) is
                        when 4      => n_st <= unsigned(resize(shift_left(resize(e_fv, 24), 2), 24));
                        when 3      => n_st <= unsigned(resize(shift_left(resize(e_fv, 24), 1) + resize(e_fv, 24), 24));
                        when others => n_st <= unsigned(resize(shift_left(resize(e_fv, 24), 1), 24));
                    end case;
                when 40 =>
                    n_fs <= m_acc(23 downto 0) + to_unsigned(C_C0Q, 24);
                    m_a <= to_unsigned(40503, 32); m_b <= resize(e_ra(25 downto 16), 16) xor C_SALTA; m_acc <= (others => '0');
                -- hashes: d = mix(j * 40503), seed = d(31..17) & 1; the row's lattice values L(i) = state(i) >> 8
                when 57 | 74 | 119 | 136 =>
                    e_h <= m_acc;
                    if e_lc = 57 then
                        v_j := resize(e_ra(25 downto 16) + 1, 16);
                        m_a <= to_unsigned(40503, 32); m_b <= v_j xor C_SALTA; m_acc <= (others => '0');
                    elsif e_lc = 119 then
                        v_j := resize(e_rb(25 downto 16) + 1, 16);
                        m_a <= to_unsigned(40503, 32); m_b <= v_j xor C_SALTB; m_acc <= (others => '0');
                    end if;
                when 58 | 75 | 120 | 137 => e_h <= e_h xor shift_right(e_h, 15);
                when 59 | 76 | 121 | 138 => e_h <= e_h + shift_left(e_h, 7);
                when 61 | 78 | 123 | 140 =>
                    v_s := e_h(31 downto 17) & '1';
                    if e_lc = 61 or e_lc = 123 then e_t0 <= v_s(15 downto 8); else e_b0 <= v_s(15 downto 8); end if;
                    e_s <= f_xs16(v_s);
                when 62 | 79 | 124 | 141 =>
                    if e_lc = 62 or e_lc = 124 then e_t1 <= e_s(15 downto 8); else e_b1 <= e_s(15 downto 8); end if;
                    e_s <= f_xs16(e_s);
                when 63 | 80 | 125 | 142 =>
                    if e_lc = 63 or e_lc = 125 then e_t2 <= e_s(15 downto 8); else e_b2 <= e_s(15 downto 8); end if;
                    case to_integer(e_lc) is
                        when 63     => n_lt(0) <= e_s;
                        when 80     => n_lb(0) <= e_s;
                        when 125    => n_lt(1) <= e_s;
                        when others => n_lb(1) <= e_s;
                    end case;
                -- column values (4-bit multiplier: result at load + 5, read at load + 7)
                when 81 | 143 =>
                    v_d := signed(resize(e_b0, 9)) - signed(resize(e_t0, 9));
                    m_a <= unsigned(resize(v_d, 32)); m_acc <= (others => '0');
                    if e_lc = 81 then m_b <= resize(n_wya, 16); else m_b <= resize(n_wyb, 16); end if;
                when 88 | 150 =>
                    if e_lc = 88 then n_c(0) <= unsigned(v_c(7 downto 0)); else n_c(3) <= unsigned(v_c(7 downto 0)); end if;
                    e_t0 <= e_t1; e_b0 <= e_b1;
                    v_d := signed(resize(e_b1, 9)) - signed(resize(e_t1, 9));
                    m_a <= unsigned(resize(v_d, 32)); m_acc <= (others => '0');
                    if e_lc = 88 then m_b <= resize(n_wya, 16); else m_b <= resize(n_wyb, 16); end if;
                when 95 | 157 =>
                    if e_lc = 95 then n_c(1) <= unsigned(v_c(7 downto 0)); else n_c(4) <= unsigned(v_c(7 downto 0)); end if;
                    e_t0 <= e_t2; e_b0 <= e_b2;
                    v_d := signed(resize(e_b2, 9)) - signed(resize(e_t2, 9));
                    m_a <= unsigned(resize(v_d, 32)); m_acc <= (others => '0');
                    if e_lc = 95 then m_b <= resize(n_wya, 16); else m_b <= resize(n_wyb, 16); end if;
                when 102 | 164 =>
                    if e_lc = 102 then
                        n_c(2) <= unsigned(v_c(7 downto 0));
                        m_a <= to_unsigned(40503, 32); m_b <= resize(e_rb(25 downto 16), 16) xor C_SALTB; m_acc <= (others => '0');
                    else
                        n_c(5) <= unsigned(v_c(7 downto 0));
                    end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): the phase DDA (Mirror: runs back from the centre), the
    -- two noise octaves (a cell accumulator; at each cell crossing the column
    -- values shift and both row streams step, and a serial 6-bit multiplier
    -- forms the next column value), the pixel count
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_a : unsigned(14 downto 0);
        variable v_t, v_b : unsigned(15 downto 0);
        variable v_c : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            for o in 0 to 1 loop
                -- the next column value C2 = Lt + ((Lb - Lt) wy) >> 4, three clocks after the streams step
                y_go1(o) <= '0'; y_go2(o) <= y_go1(o); y_go3(o) <= y_go2(o);
                if y_go1(o) = '1' then
                    y_base(o) <= o_lt(o)(15 downto 8);
                    y_dd(o) <= signed(resize(o_lb(o)(15 downto 8), 9)) - signed(resize(o_lt(o)(15 downto 8), 9));
                end if;
                if y_go2(o) = '1' then
                    if o = 0 then y_p(o) <= y_dd(o) * signed(resize(l_wya, 5));
                    else y_p(o) <= y_dd(o) * signed(resize(l_wyb, 5)); end if;
                end if;
                if y_go3(o) = '1' then
                    v_c := signed(resize(y_base(o), 10)) + resize(shift_right(y_p(o), 4), 10);
                    o_c2(o) <= unsigned(v_c(7 downto 0));
                end if;
            end loop;
            if g_start = '1' then
                b_ph <= n_fs; l_st <= n_st; b_rev <= '0';
                for o in 0 to 1 loop
                    o_acc(o) <= (others => '0'); o_lt(o) <= n_lt(o); o_lb(o) <= n_lb(o);
                    o_c0(o) <= n_c(3 * o); o_c1(o) <= n_c(3 * o + 1); o_c2(o) <= n_c(3 * o + 2);
                end loop;
                l_wya <= n_wya; l_wyb <= n_wyb; l_gl <= n_gl;
            elsif g_avid = '1' then
                if b_rev = '1' then b_ph <= b_ph - l_st; else b_ph <= b_ph + l_st; end if;
                if s_mirror = '1' and b_px >= c_m1 then b_rev <= '1'; end if;
                for o in 0 to 1 loop
                    if o = 0 then v_a := resize(o_acc(o), 15) + c_sta; else v_a := resize(o_acc(o), 15) + c_stb; end if;
                    o_acc(o) <= v_a(13 downto 0);
                    if v_a(14) = '1' then
                        v_t := f_xs16(o_lt(o)); v_b := f_xs16(o_lb(o));
                        o_lt(o) <= v_t; o_lb(o) <= v_b;
                        o_c0(o) <= o_c1(o); o_c1(o) <= o_c2(o);
                        y_go1(o) <= '1';
                    end if;
                end loop;
            end if;
            if g_avid = '1' then b_px <= b_px + 1; else b_px <= (others => '0'); end if;
            s_lb_ra <= resize(shift_right(b_px + 2, 1), 10);
            -- ground darkening: virtual x of the stage-8 pixel, ROM address, ROM data (stage 10)
            if s_gav_sr(6) = '1' then gx <= gx + s_xs; else gx <= resize(c_x0, 12); end if;
            r_dx_a <= gx(11 downto 4);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel; register pN holds stage N)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n   : signed(9 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_w   : signed(14 downto 0);
        variable v_t   : signed(13 downto 0);
        variable v_k   : signed(17 downto 0);
        variable v_ab  : signed(14 downto 0);
        variable v_lv  : unsigned(6 downto 0);
        variable v_reg : unsigned(1 downto 0);
        variable v_x   : signed(10 downto 0);
        variable v_c   : signed(9 downto 0);
        variable v_ts  : signed(9 downto 0);
        variable v_lm  : signed(8 downto 0);
        variable v_gm  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: column differences; phase; etch product (luma x gain / 8: the camera unsigned, the Osc signed)
            for o in 0 to 1 loop
                p2_dx(o) <= signed(resize(o_c1(o), 9)) - signed(resize(o_c0(o), 9));
                p2_c0(o) <= o_c0(o); p2_wx(o) <= o_acc(o)(13 downto 10);
            end loop;
            p2_v <= b_ph(23 downto 12);
            if s_src = '1' then v_lm := resize(signed(s_lb_q), 9); else v_lm := signed(resize(unsigned(s_lb_q), 9)); end if;
            p2_ep <= v_lm * signed(resize(s_k6(9 downto 5), 6));
            -- 3: x interpolation products; etch (Osc 8x the camera's gain)
            for o in 0 to 1 loop
                p3_p(o) <= p2_dx(o) * signed(resize(p2_wx(o), 5));
                p3_c0(o) <= p2_c0(o);
            end loop;
            if s_src = '1' then p3_du <= shift_left(resize(p2_ep, 14), 1);
            else p3_du <= resize(shift_right(p2_ep, 2), 14); end if;
            p3_v <= p2_v;
            -- 4: the two noises (n1 blossom, n2 flame); etched phase; etched half-width
            v_n := signed(resize(p3_c0(0), 10)) + resize(shift_right(p3_p(0), 4), 10);
            p4_n1 <= unsigned(v_n(7 downto 0));
            v_n := signed(resize(p3_c0(1), 10)) + resize(shift_right(p3_p(1), 4), 10);
            p4_n2 <= unsigned(v_n(7 downto 0));
            p4_v <= p3_v + unsigned(p3_du(11 downto 0));
            v_w := signed(resize(c_w, 15)) + resize(shift_right(p3_du, 2), 15) + resize(shift_right(p3_du, 4), 15);
            if v_w < 0 then p4_w <= (others => '0'); elsif v_w > 2047 then p4_w <= (others => '1');
            else p4_w <= unsigned(v_w(10 downto 0)); end if;
            -- 5: distance from the vine centre and side; noise gains (P12)
            if p4_v(11) = '0' then p5_a <= 2048 - resize(p4_v, 12); p5_l <= '1';
            else p5_a <= resize(p4_v(10 downto 0), 12); p5_l <= '0'; end if;
            p5_n1x <= signed(resize(p4_n1, 9)) - 128;
            p5_n2p <= (signed(resize(p4_n2, 9)) - 128) * signed(resize(c_g2(7 downto 4), 5));
            if p4_n2 > c_r0 then p5_ep <= (p4_n2 - c_r0) * c_g3(9 downto 5); else p5_ep <= (others => '0'); end if;
            p5_n1 <= p4_n1; p5_n2 <= p4_n2; p5_w <= p4_w;
            -- 6: edge distance; N1 = 6.25 (n1 - 128); N2; E; the right side's distance x 1.75
            p6_d <= signed(resize(p5_w, 13)) - signed(resize(p5_a, 13));
            v_x := resize(p5_n1x, 11);
            p6_n1k <= shift_left(v_x, 2) + shift_left(v_x, 1) + shift_right(v_x, 2);
            p6_n2k <= resize(p5_n2p, 13);
            p6_ep  <= resize(p5_ep, 13) & '0';
            if p5_ep /= 0 then p6_epz <= '1'; else p6_epz <= '0'; end if;
            p6_ar  <= resize(p5_a, 13) + resize(p5_a(11 downto 1), 13) + resize(p5_a(11 downto 2), 13);
            p6_a <= p5_a; p6_l <= p5_l; p6_n1 <= p5_n1; p6_n2 <= p5_n2; p6_w <= p5_w;
            -- 7: the key's slope term (3.125 on the left, 4.5 on the right); noise sums
            v_d := p6_d;
            if p6_l = '1' then
                p7_b <= shift_left(resize(v_d, 16), 1) + resize(v_d, 16) + resize(shift_right(v_d, 3), 16);
            else
                p7_b <= shift_left(resize(v_d, 16), 2) + resize(shift_right(v_d, 1), 16);
            end if;
            p7_n12 <= resize(p6_n1k, 15) + resize(p6_n2k, 15);
            p7_n1e <= resize(p6_n1k, 15) + signed(resize(p6_ep, 15));
            p7_a <= p6_a; p7_ar <= p6_ar; p7_l <= p6_l; p7_epz <= p6_epz;
            p7_n1 <= p6_n1; p7_n2 <= p6_n2; p7_w <= p6_w;
            -- 8: the vine key; the side-weighted distance
            p8_kv <= resize(p7_b, 17) + resize(p7_n12, 17);
            if p7_l = '1' then p8_du <= resize(p7_a, 13); else p8_du <= p7_ar; end if;
            p8_n1e <= p7_n1e; p8_epz <= p7_epz; p8_l <= p7_l;
            p8_n1 <= p7_n1; p8_n2 <= p7_n2; p8_w <= p7_w;
            -- 9: vine / rim; distances for the extensions and the leaf band
            p9_dt <= signed(resize(p8_w, 14)) - signed(resize(p8_du, 14));
            p9_ld <= signed(resize(p8_du, 14)) - signed(resize(c_lc, 14));
            if p8_kv > 0 then p9_vine <= '1'; else p9_vine <= '0'; end if;
            if p8_kv > -256 and p8_kv <= 0 then p9_rim <= '1'; else p9_rim <= '0'; end if;
            v_k := resize(p8_kv, 18) + 256;
            if p8_kv >= 0 then p9_rd <= "1111"; else p9_rd <= unsigned(v_k(7 downto 4)); end if;
            p9_n1e <= p8_n1e; p9_epz <= p8_epz; p9_l <= p8_l; p9_n1 <= p8_n1; p9_n2 <= p8_n2;
            -- 10: the extension key's slope term (2.5); the extension colour coordinate em = (du - W) / 0.2 cycle
            v_t := p9_dt;
            p10_ekp <= shift_left(resize(v_t, 17), 1) + resize(shift_right(v_t, 1), 17);
            v_t := -p9_dt;
            if v_t < 0 then p10_em <= (others => '0');
            elsif v_t > 1023 then p10_em <= (others => '1');
            else p10_em <= unsigned(v_t(9 downto 4)); end if;
            v_ab := abs(resize(p9_ld, 15));
            p10_ald <= resize(unsigned(v_ab(13 downto 4)), 10);
            p10_vine <= p9_vine; p10_rim <= p9_rim; p10_rd <= p9_rd;
            p10_n1e <= p9_n1e; p10_epz <= p9_epz; p10_l <= p9_l; p10_n1 <= p9_n1; p10_n2 <= p9_n2;
            -- 11: extension key; leaf level; the ground (level minus the parabola)
            p11_ek <= resize(p10_ekp, 18) + resize(p10_n1e, 18);
            p11_lv <= signed(resize(p10_n2, 11)) - signed(resize(p10_ald, 11));
            v_gm := signed(resize(l_gl, 10)) - signed(resize(r_dx_q, 10));
            if v_gm < 0 then p11_g <= (others => '0'); else p11_g <= unsigned(v_gm(7 downto 0)); end if;
            p11_vine <= p10_vine; p11_rim <= p10_rim; p11_rd <= p10_rd; p11_em <= p10_em;
            p11_epz <= p10_epz; p11_l <= p10_l; p11_n1 <= p10_n1; p11_n2 <= p10_n2;
            -- 12: classes and colour coordinates
            if p11_ek > 0 and p11_epz = '1' and p11_vine = '0' then p12_ext <= '1'; else p12_ext <= '0'; end if;
            if p11_em < 12 and p11_n1 <= 153 then p12_dark <= '1'; else p12_dark <= '0'; end if;
            if p11_lv > signed(resize(c_lt, 11)) and p11_n1 > c_lg and p11_vine = '0' and p11_rim = '0' and
               not (p11_ek > 0 and p11_epz = '1') then
                p12_leaf <= '1';
            else
                p12_leaf <= '0';
            end if;
            v_c := signed(resize(p11_n1, 10)) - 153;
            v_ts := signed(resize(p11_em, 10)) + shift_right(v_c, 3);
            if v_ts < 0 then p12_ts <= (others => '0'); elsif v_ts > 63 then p12_ts <= (others => '1');
            else p12_ts <= unsigned(v_ts(5 downto 0)); end if;
            if p11_l = '0' then v_ts := to_signed(36, 10); else v_ts := to_signed(10, 10); end if;
            v_ts := v_ts + shift_right(signed(resize(p11_n1, 10)) - signed(resize(c_lg, 10)), 1);
            if v_ts < 0 then p12_ls <= (others => '0'); elsif v_ts > 63 then p12_ls <= (others => '1');
            else p12_ls <= unsigned(v_ts(5 downto 0)); end if;
            v_ts := signed(resize(p11_n1, 10)) + shift_right(signed(resize(p11_n2, 10)) - 128, 2);
            if v_ts < 0 then p12_vc <= (others => '0'); elsif v_ts > 255 then p12_vc <= (others => '1');
            else p12_vc <= unsigned(v_ts(7 downto 1)); end if;
            p12_g <= p11_g;
            p12_vine <= p11_vine; p12_rim <= p11_rim; p12_rd <= p11_rd; p12_em <= p11_em;
            -- 13: region and level -> palette address (vine > dark spike > tongue > rim > leaf > ground)
            if p12_vine = '1' then
                v_reg := "01"; v_lv := p12_vc;
            elsif p12_ext = '1' and p12_dark = '1' then
                v_reg := "11"; v_lv := '1' & resize(p12_em, 6);
            elsif p12_ext = '1' then
                v_reg := "10"; v_lv := '0' & p12_ts;
            elsif p12_rim = '1' then
                v_reg := "11"; v_lv := '0' & p12_rd & "00";
            elsif p12_leaf = '1' then
                v_reg := "10"; v_lv := '1' & p12_ls;
            else
                v_reg := "00"; v_lv := p12_g(7 downto 1);
            end if;
            p13_pa <= s_pal & v_reg & v_lv;
            -- 14: palette address; 15: palette ROM (EBR); 16: fabric copy
            r_pa <= p13_pa;
            p16_q <= r_pq;
            -- 17: compose; neutral black outside the generated avid
            if s_gav_sr(14) = '0' then
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            else
                s_y17 <= unsigned(p16_q(31 downto 22)); s_u17 <= unsigned(p16_q(21 downto 12)); s_v17 <= unsigned(p16_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums, the history in the block-RAM ring), Y through [1 2 1]/4;
    -- luma 13/16, a vertical [1 1]/2 on luma, and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_vs : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to neutral (the off-avid input) once 8 blank samples are in, so a
            -- glitched sample can't offset the colours for good.  18: sums, 19: output
            s_bxd <= s_bxq;                                      -- fabric copy of the sample 8 back
            if s_bah = "00000000" then
                s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bau <= s_bau + s_u17 - unsigned(s_bxd(19 downto 10));
                s_bav <= s_bav + s_v17 - unsigned(s_bxd(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 14) + s_baud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v19 <= v_sc(13 downto 4);
            s_yh(1) <= s_y17;
            for i in 2 to 3 loop s_yh(i) <= s_yh(i - 1); end loop;
            v_y := resize(s_yh(1), 12) + shift_left(resize(s_yh(2), 12), 1) + resize(s_yh(3), 12);
            s_y19 <= v_y(11 downto 2);
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: vertical [1 1]/2 on luma (the first line of a field and the first two pixels pass) and the
            -- blanking gate
            s_vq2 <= s_vq;
            if s_gav_sr(18) = '1' then s_vc <= s_vc + 1; else s_vc <= (others => '0'); end if;
            if s_zone = '1' then s_vfirst <= '1';
            elsif s_gav_sr(18) = '0' and s_gav_sr(19) = '1' then s_vfirst <= '0'; end if;
            if s_gav_sr(C_LAT - 2) = '1' then
                if s_vfirst = '1' or s_vc < 2 then
                    s_y21 <= s_y20;
                else
                    v_vs := resize(s_y20, 11) + resize(unsigned(s_vq2) & "00", 11) + 1;
                    s_y21 <= v_vs(10 downto 1);
                end if;
                s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture vine;
