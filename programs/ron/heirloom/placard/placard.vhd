-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Placard v0.1.0 -- Heirloom from the clip WIN_20230221_00_58_22 (0.5 s).
--
-- The picture: a magenta ground with thin black arcs, cut by diagonal CMYK zones (white / cyan / flat
-- magenta / yellow / black, the fold of a diagonal ramp), vertical bars and horizontal stripes keyed inside
-- the white and cyan zones, two "roads" (bands of fine white / cyan / magenta / yellow / black lines), dark
-- mosaic skirts, the whole vertical structure rolling down the frame, inside a black letterbox frame.
--
-- The patch, as fitted (see .heirloom_work/rb_placard/notes.md):
--   phi    = (y - scroll + 118) mod 940          one vertical period = 940 lines (two u-cycles)
--   u      = 2^(a cos(2 pi (phi - 108) / 470))   a vertical oscillator (K2 = its depth)
--   F      = (x - 25) u / lambda                 H ramp through a VCA: contours = the arcs (K1 = lambda)
--   Z      = tri(x - 1.8 y')                     diagonal ramp, folded (S7): white < 165 < cyan < 375 <
--                                                flat magenta < 575 < yellow < 1400 < black  (K5 scales)
--   P      = (x + 200) 1.2 (phi + 4)             product of the H ramp and the V ramp; the roads are shells
--                                                of P (narrow: 53000 < P - 72806 (x+200) < + 78 (x+200);
--                                                wide: P > 48500 and 60826 (x+200) - P > 93000), coloured by
--                                                P's cyclic 16-entry palette (one cycle per 32768)
--   gate   = the zones show between the roads, gated by a vertical square wave (P12 Flood = its duty)
--   bars   = ((x - 25) mod 100.75) < 46 (K3), white / cyan by zone, in three x-regions; stripes (S8) of
--            30 lines in the first region; hatched bars in the gate-off window; mosaic skirts in the
--            yellow (50-px sample and hold of the product field; hashed depths under the wide road)
--   scroll = 172 px/s down (K4, bipolar)
--
--   K1  Arcs      arc pitch x0.35 .. x2.8 (512 = the clip)
--   K2  Squeeze   depth of the vertical oscillator into the arcs: 0 = straight lines .. the clip .. 4x
--   K3  Bars      bar and mosaic-column pitch x0.5 .. x2
--   K4  Speed     bipolar roll (+-8 deadband, square law); 672 = the clip's 172 px/s down
--   K5  Zones     diagonal zone scale x0.41 .. x2.46
--   K6  Etch      the camera (or the Osc) bends the arcs and the zone edges
--   S7  Fold      Fold (the clip) / Ramp (the zone sequence runs one way, black zone visible)
--   S8  Stripes   Stripes (the clip) / Bars in the first region
--   S9  Frame     On (the clip's letterbox) / Off (full screen)
--   S10 Colours   Print (CMYK, the clip) / Neon (navy ground, white / green / red / blue zones)
--   S11 Source    Video / Osc
--   P12 FLOOD     the zone gate's duty: 0 % = arcs and roads only .. 40 % = the clip .. 100 % = zones
--                 and bars everywhere between the roads
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, one shared exponent shifter), a
-- per-line engine kicked at each line start (computes the NEXT line: the arc rate from the cos / exp2
-- ROMs, the product field's row value and step, the zone ramp), per-pixel DDAs (arc phase, product
-- field, four offset ramps, the zone ramp wrapping at its period and folded next stage, bar phase), a
-- 15-stage pixel path whose delays run through block-RAM rings.  Product-field values are Q4 (P / 16).
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (U and
-- V through a [1 2 2 2 2 2 2 2 1]/16 box on a block-RAM ring, luma [1 2 1]/4 and 13/16) before the
-- blanking gate; C_LAT 18 in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.placard_pkg.all;
use work.all;

architecture placard of program_top is
    constant C_LAT : integer := 18;   -- = processing_delay_clks

    -- product-field constants, Q4 (P = (x + 200) * 1.2 * T, divided by 16)
    constant C_C1  : integer := 848000;    -- narrow road top (53000 * 16)
    constant C_CW1 : integer := 776000;    -- wide road lower hyperbola (48500 * 16)
    constant C_CW2 : integer := 1488000;   -- wide road upper hyperbola (93000 * 16)
    constant C_C3  : integer := 1408000;   -- mosaic skirt (88000 * 16)
    constant C_KN  : integer := 4550;      -- 9.6 * 474
    constant C_KNL : integer := 5174;      -- 9.6 * 474 + 78 * 8
    constant C_KW2 : integer := 3802;      -- 9.6 * 396
    constant C_K3  : integer := 9062;      -- 9.6 * 944
    constant C_BAR : unsigned(15 downto 0) := to_unsigned(29950, 16);   -- bar duty 0.457

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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    -- a 36-bit product >> 15, then by the exponent's integer part (-2 .. 1)
    function f_esh(v : unsigned(35 downto 0); sh : signed(2 downto 0)) return unsigned is
        variable r : unsigned(20 downto 0);
    begin
        case to_integer(sh) is
            when -2     => r := resize(v(35 downto 17), 21);
            when -1     => r := resize(v(35 downto 16), 21);
            when 0      => r := resize(v(35 downto 15), 21);
            when others => r := resize(v(34 downto 14), 21);
        end case;
        return r;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- delay rings (block RAM, collision-free: the read pointer trails the write pointer)
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal rg_a0, rg_a1, rg_b0, rg_b1, rg_c0, rg_c1 : t_r16 := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of rg_a0 : signal is true;
    attribute no_rw_check of rg_a1 : signal is true;
    attribute no_rw_check of rg_b0 : signal is true;
    attribute no_rw_check of rg_b1 : signal is true;
    attribute no_rw_check of rg_c0 : signal is true;
    attribute no_rw_check of rg_c1 : signal is true;
    signal rg_wa : unsigned(7 downto 0) := (others => '0');
    signal rg_ra1, rg_ra4, rg_ra6 : unsigned(7 downto 0) := (others => '0');
    signal rg_a0q, rg_a1q, rg_b0q, rg_b1q, rg_c0q, rg_c1q : std_logic_vector(15 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_el_a  : unsigned(7 downto 0) := (others => '0');
    signal r_el_q  : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a  : unsigned(9 downto 0) := (others => '0');
    signal r_cs_q  : signed(7 downto 0) := (others => '0');
    signal r_st_q  : unsigned(3 downto 0) := (others => '0');
    signal r_pl_a  : unsigned(5 downto 0) := (others => '0');
    signal r_pl_q  : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(410 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(672, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_ramp, s_bars, s_noframe, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e1, z_e3n, z_e5 : signed(10 downto 0) := (others => '0');
    signal z_m1, z_m3n, z_m5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_shc  : signed(2 downto 0) := (others => '0');
    signal z_res  : unsigned(20 downto 0) := (others => '0');
    signal z_q    : unsigned(9 downto 0) := (others => '0');
    signal c_p30  : std_logic := '1';
    signal c_invl : unsigned(15 downto 0) := to_unsigned(4810, 16);   -- 2^19 / lambda per h
    signal c_k2g  : unsigned(9 downto 0) := to_unsigned(160, 10);     -- Q8 octaves per unit cos
    signal c_binc : unsigned(9 downto 0) := to_unsigned(325, 10);     -- bar phase per h (Q16)
    signal c_bincx : unsigned(11 downto 0) := to_unsigned(650, 12);   -- per pixel
    signal c_vq4  : unsigned(8 downto 0) := to_unsigned(49, 9);       -- roll per field, Q4 lines (before p30)
    signal c_vf   : unsigned(9 downto 0) := to_unsigned(98, 10);
    signal c_vneg : std_logic := '0';
    signal c_p64  : unsigned(19 downto 0) := to_unsigned(217600, 20); -- zone half period, Q6 h
    signal c_pq   : unsigned(13 downto 0) := to_unsigned(3400, 14);   -- Q0 h
    signal c_2p   : unsigned(14 downto 0) := to_unsigned(6800, 15);   -- 2 pq
    signal c_wx   : unsigned(14 downto 0) := to_unsigned(6798, 15);   -- wrap point - xs (2 pq fold, pq ramp)
    signal c_wl   : unsigned(21 downto 0) := to_unsigned(435200, 22); -- line wrap (2 p64 fold, p64 ramp), Q6
    signal c_th1, c_th2, c_th3, c_th4 : unsigned(13 downto 0) := to_unsigned(330, 14);
    signal c_eg   : unsigned(13 downto 0) := to_unsigned(1300, 14);
    signal c_g2   : unsigned(9 downto 0) := to_unsigned(282, 10);
    signal c_g1   : unsigned(9 downto 0) := (others => '0');
    signal c_dz   : unsigned(11 downto 0) := to_unsigned(230, 12);    -- zone step per line, Q6 h
    signal c_bph0 : unsigned(15 downto 0) := (others => '0');
    signal c_x0h50 : unsigned(8 downto 0) := to_unsigned(50, 9);
    signal c_x0neg : std_logic := '1';
    signal c_xa   : unsigned(9 downto 0) := to_unsigned(400, 10);     -- x0h + 400
    signal c_A    : unsigned(13 downto 0) := to_unsigned(3840, 14);   -- 9.6 (x0h + 400)
    signal c_B    : unsigned(9 downto 0) := to_unsigned(307, 10);     -- 9.6 xs * 16
    signal c_wp   : unsigned(22 downto 0) := to_unsigned(3609600, 23);
    signal c_ws   : unsigned(19 downto 0) := to_unsigned(288580, 20);
    signal c_dp0  : unsigned(15 downto 0) := to_unsigned(3840, 16);
    signal c_dstep : unsigned(11 downto 0) := to_unsigned(307, 12);
    signal c_pw10 : unsigned(25 downto 0) := (others => '0');
    signal c_stw0 : unsigned(19 downto 0) := (others => '0');
    signal c_rn0, c_rl0, c_rw20, c_r30 : unsigned(25 downto 0) := (others => '0');
    signal c_rns, c_rls, c_rw2s, c_r3s : unsigned(15 downto 0) := (others => '0');
    signal c_phi0 : unsigned(13 downto 0) := to_unsigned(1888, 14);   -- line-0 phase, Q4
    signal c_phl0 : unsigned(10 downto 0) := to_unsigned(122, 11);    -- (phi0 >> 4) + 4
    signal c_zuf  : unsigned(21 downto 0) := (others => '0');          -- zone ramp at line 0, Q6 h, unfolded
    signal c_ky, c_ku, c_kv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- black of the palette
    signal z_zs   : signed(21 downto 0) := (others => '0');
    signal z_zt   : signed(22 downto 0) := (others => '0');
    -- exponents in unsigned form (k = the glided knob, 0 .. 1023), so no adder bit sums a sign bit with
    -- itself (a router1 hang): 0.75 (k - 512) = 3k/4 - 384, 0.656 (k - 512) = k/2 + k/8 + k/32 - 336
    signal z_k1, z_k3, z_k5 : unsigned(9 downto 0) := (others => '0');
    signal z_e1w  : unsigned(11 downto 0) := (others => '0');
    signal z_e5a  : unsigned(9 downto 0) := (others => '0');
    signal z_dz   : unsigned(11 downto 0) := (others => '0');
    signal z_zm1, z_zm2 : unsigned(14 downto 0) := (others => '0');
    signal c_zm   : unsigned(14 downto 0) := (others => '0');          -- zone shift per field = 14.375 vf (Q6 h)
    signal z_th3, z_th4 : unsigned(13 downto 0) := (others => '0');
    signal z_eg   : unsigned(13 downto 0) := (others => '0');
    signal z_g    : signed(11 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_phi4 : unsigned(13 downto 0) := to_unsigned(1888, 14);
    signal e_phl  : unsigned(9 downto 0) := to_unsigned(118, 10);
    signal e_wrap, e_pnw : std_logic := '0';
    signal e_pn   : unsigned(14 downto 0) := (others => '0');
    signal e_pnx  : unsigned(13 downto 0) := to_unsigned(1904, 14);
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_zu   : unsigned(21 downto 0) := (others => '0');
    signal e_zop  : unsigned(21 downto 0) := (others => '0');
    signal e_zen  : std_logic := '0';
    signal e_pw10 : unsigned(25 downto 0) := (others => '0');
    signal e_stw  : unsigned(19 downto 0) := (others => '0');
    signal e_pwop : unsigned(25 downto 0) := (others => '0');
    signal e_stop : unsigned(19 downto 0) := (others => '0');
    signal e_pen  : std_logic := '0';
    signal e_cneg : std_logic := '0';
    signal e_e2   : signed(10 downto 0) := (others => '0');
    signal e_sh   : signed(2 downto 0) := (others => '0');
    signal e_fi   : unsigned(16 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_f0   : unsigned(19 downto 0) := (others => '0');
    signal n_fst  : unsigned(18 downto 0) := (others => '0');
    signal n_g1, n_g2, n_p470, n_strip, n_frow : std_logic := '0';
    signal n_hrow : unsigned(1 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_xh   : unsigned(12 downto 0) := (others => '0');
    signal b_f    : unsigned(19 downto 0) := (others => '0');
    signal b_pw1  : unsigned(25 downto 0) := (others => '0');
    signal b_rn, b_rl, b_rw2, b_r3 : unsigned(25 downto 0) := (others => '0');
    signal b_zu   : unsigned(14 downto 0) := (others => '0');
    signal b_bph  : unsigned(15 downto 0) := (others => '0');
    signal b_tick : std_logic := '0';
    signal l_fst  : unsigned(18 downto 0) := (others => '0');
    signal l_stw  : unsigned(15 downto 0) := (others => '0');
    signal l_g1, l_g2, l_p470, l_strip, l_frow : std_logic := '0';
    signal l_hrow : unsigned(1 downto 0) := (others => '0');

    -- pixel path
    signal p2_f   : unsigned(15 downto 0) := (others => '0');
    signal p2_zt  : unsigned(13 downto 0) := (others => '0');
    signal p2_qd  : unsigned(18 downto 0) := (others => '0');
    signal p2_fl  : std_logic_vector(18 downto 0) := (others => '0');   -- qn, qw, xf (x < 1160, > 1800, < 2240, < 2600), fcol, bar, wb2, wb1, nb2, nb1
    signal p2_mb, p2_tick : std_logic := '0';
    signal p3_mbsh, p4_mbsh, p5_mbsh, p6_mbsh, p7_mbsh, p8_mbsh : std_logic := '0';
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(17 downto 0) := (others => '0');
    signal p5_le  : unsigned(21 downto 0) := (others => '0');
    signal p5_f   : unsigned(15 downto 0) := (others => '0');
    signal p5_zt  : unsigned(13 downto 0) := (others => '0');
    signal p6_fe  : unsigned(15 downto 0) := (others => '0');
    signal p6_zr  : unsigned(15 downto 0) := (others => '0');
    signal p7_fe8 : unsigned(7 downto 0) := (others => '0');
    signal p7_zte : unsigned(14 downto 0) := (others => '0');
    signal p8_zone : unsigned(2 downto 0) := (others => '0');
    signal p8_thick, p8_thin : std_logic := '0';
    signal p8_fl  : std_logic_vector(18 downto 0) := (others => '0');
    signal p9_cls, p10_cls, p11_cls, p12_cls : unsigned(4 downto 0) := (others => '0');
    signal p13_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_cau7, s_cav7 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly   : t_u10(1 to 6) := (others => to_unsigned(64, 10));
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs, and
    -- the delay rings: one write pointer and registered read pointers.  A
    -- sample written at edge k is in the RAM's output register after edge
    -- k + m + 1 when the read pointer register is wa - m.  Ring A (arc phase
    -- + zone position, written at the stage-2 edge, read for the stage-5
    -- copy): m = 0.  Ring B (flags, written at the stage-2 edge, read for
    -- the stage-8 copy): m = 3.  Ring C (the output box's 8-back tap through
    -- a copy, so the running sum spans exactly 8 samples): m = 5.
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
            r_ex_q <= C_PL_EXP2(to_integer(r_ex_a));
            r_el_q <= C_PL_EXP2(to_integer(r_el_a));
            r_cs_q <= C_PL_COS(to_integer(r_cs_a));
            r_st_q <= C_PL_STR(to_integer(r_cs_a));
            r_pl_q <= C_PL_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rg_wa <= rg_wa + 1;
            rg_ra1 <= rg_wa; rg_ra4 <= rg_wa - 3; rg_ra6 <= rg_wa - 5;
            rg_a0(to_integer(rg_wa)) <= std_logic_vector(p2_f);
            rg_a0q <= rg_a0(to_integer(rg_ra1));
            rg_a1(to_integer(rg_wa)) <= "00" & std_logic_vector(p2_zt);
            rg_a1q <= rg_a1(to_integer(rg_ra1));
            rg_b0(to_integer(rg_wa)) <= p2_fl(15 downto 0);
            rg_b0q <= rg_b0(to_integer(rg_ra4));
            rg_b1(to_integer(rg_wa)) <= "000000000000" & std_logic_vector(p2_qd(18 downto 15));
            rg_b1q <= rg_b1(to_integer(rg_ra4));
            rg_c0(to_integer(rg_wa)) <= "000000" & std_logic_vector(s_u15);
            rg_c0q <= rg_c0(to_integer(rg_ra6));
            rg_c1(to_integer(rg_wa)) <= "000000" & std_logic_vector(s_v15);
            rg_c1q <= rg_c1(to_integer(rg_ra6));
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_w  : signed(23 downto 0);
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
                s_ramp    <= registers_in(6)(0);
                s_bars    <= registers_in(6)(1);
                s_noframe <= registers_in(6)(2);
                s_pal     <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                s_xs      <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
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

            -- serial products: slot k loads at zc = 56 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18, through the shared exponent shifter z_res).  The slot
            -- number is decoded at 54 + 24 k and the operands are pre-muxed at 55 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 54  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 486 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 510 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_shc <= "000";
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(4810, 36);          z_fb <= z_m1;  z_shc <= z_e1(10 downto 8);   -- 1/lambda
                    when 1  => z_fa <= resize(s_g2(12 downto 3), 36);  z_fb <= resize(s_g2(12 downto 3), 16);       -- k2^2
                    when 2  => z_fa <= to_unsigned(325, 36);           z_fb <= z_m3n; z_shc <= z_e3n(10 downto 8);  -- bar phase step
                    when 3  => z_fa <= resize(s_vm, 36);               z_fb <= resize(s_vm, 16);                    -- speed^2
                    when 4  => z_fa <= to_unsigned(217600, 36);        z_fb <= z_m5;  z_shc <= z_e5(10 downto 8);   -- zone period
                    when 5  => z_fa <= resize(s_g6(12 downto 3), 36);  z_fb <= resize(s_g6(12 downto 3), 16);       -- k6^2
                    when 6  => z_fa <= resize(s_gp(12 downto 3), 36);  z_fb <= to_unsigned(470, 16);                -- flood
                    when 7  => z_fa <= resize(c_x0h50, 36);            z_fb <= resize(c_binc, 16);                  -- bar phase at x0
                    when 8  => z_fa <= resize(c_A, 36);                z_fb <= resize(c_phl0, 16);                  -- P at line 0
                    when 9  => z_fa <= resize(c_B, 36);                z_fb <= resize(c_phl0, 16);                  -- P step at line 0
                    when 10 => z_fa <= resize(c_A, 36);                z_fb <= resize(s_ys, 16);                    -- P per line
                    when 11 => z_fa <= resize(c_B, 36);                z_fb <= resize(s_ys, 16);                    -- step per line
                    when 12 => z_fa <= to_unsigned(C_KN, 36);          z_fb <= resize(c_xa, 16);                    -- ramp starts
                    when 13 => z_fa <= to_unsigned(C_KNL, 36);         z_fb <= resize(c_xa, 16);
                    when 14 => z_fa <= to_unsigned(C_KW2, 36);         z_fb <= resize(c_xa, 16);
                    when 15 => z_fa <= to_unsigned(C_K3, 36);          z_fb <= resize(c_xa, 16);
                    when 16 => z_fa <= to_unsigned(C_KN, 36);          z_fb <= resize(s_xs, 16);                    -- ramp steps
                    when 17 => z_fa <= to_unsigned(C_KNL, 36);         z_fb <= resize(s_xs, 16);
                    when 18 => z_fa <= to_unsigned(C_KW2, 36);         z_fb <= resize(s_xs, 16);
                    when others => z_fa <= to_unsigned(C_K3, 36);      z_fb <= resize(s_xs, 16);
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            z_res <= f_esh(f_acc, z_shc);

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- exponents (Q8 octaves) from the glided knobs; three exp2 ROM reads
                    when 25 =>
                        z_k1 <= s_g1(12 downto 3); z_k3 <= s_g3(12 downto 3); z_k5 <= s_g5(12 downto 3);
                    when 26 =>
                        z_e1w <= resize(z_k1, 12) + shift_left(resize(z_k1, 12), 1);             -- 3 k
                        z_e3n <= 256 - signed(resize(shift_right(z_k3, 1), 11));                 -- -(k - 512)/2 (-+1 oct)
                        z_e5a <= shift_right(z_k5, 1) + shift_right(z_k5, 3);
                    when 27 =>
                        z_e1 <= signed(resize(shift_right(z_e1w, 2), 11)) - 384;                 -- 0.75 (k - 512) (+-1.5 oct)
                        z_e5 <= signed(resize(z_e5a + shift_right(z_k5, 5), 11)) - 336;          -- 0.656 (k - 512) (+-1.3 oct)
                        -- zone step per line 0.9 ys (Q6 h), one op per state
                        z_dz <= s_ys - shift_right(s_ys, 3);
                    when 28 => z_dz <= z_dz + shift_right(s_ys, 5);
                    when 29 => c_dz <= z_dz - shift_right(s_ys, 7);
                    when 30 => r_ex_a <= unsigned(z_e1(7 downto 0));
                    when 31 => r_ex_a <= unsigned(z_e3n(7 downto 0));
                    when 32 => r_ex_a <= unsigned(z_e5(7 downto 0));  z_m1 <= r_ex_q;
                    when 33 => z_m3n <= r_ex_q;
                    when 34 => z_m5 <= r_ex_q;

                    -- per-field constants by timing; motion from the last field's step
                    when 40 =>
                        if s_ys = 256 then c_p30 <= '1'; else c_p30 <= '0'; end if;
                        case to_integer(s_xs) is
                            when 4 =>
                                c_x0h50 <= to_unsigned(430, 9); c_x0neg <= '0'; c_xa <= to_unsigned(880, 10);
                                c_A <= to_unsigned(8448, 14); c_B <= to_unsigned(614, 10);
                            when 3 =>
                                c_x0h50 <= to_unsigned(50, 9); c_x0neg <= '1'; c_xa <= to_unsigned(400, 10);
                                c_A <= to_unsigned(3840, 14); c_B <= to_unsigned(461, 10);
                            when others =>
                                c_x0h50 <= to_unsigned(50, 9); c_x0neg <= '1'; c_xa <= to_unsigned(400, 10);
                                c_A <= to_unsigned(3840, 14); c_B <= to_unsigned(307, 10);
                        end case;
                        -- line-0 phase (Q4 lines, mod 15040): the structure rolls down, so the phase at line 0 falls
                        if c_vneg = '1' then
                            z_zs <= signed(resize(c_phi0, 22)) + signed(resize(c_vf, 22));
                        else
                            z_zs <= signed(resize(c_phi0, 22)) - signed(resize(c_vf, 22));
                        end if;
                        -- zone ramp at line 0: the diagonal moves 3.6 vf h = 14.375 vf (c_zm, Q6), unfolded
                        if c_vneg = '1' then
                            z_zt <= signed(resize(c_zuf, 23)) - signed(resize(c_zm, 23));
                        else
                            z_zt <= signed(resize(c_zuf, 23)) + signed(resize(c_zm, 23));
                        end if;
                        if s_pal = '1' then v_k := C_PL_PAL(39); else v_k := C_PL_PAL(7); end if;
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));
                    when 41 =>
                        if z_zs < 0 then c_phi0 <= unsigned(resize(z_zs + 15040, 14));
                        elsif z_zs >= 15040 then c_phi0 <= unsigned(resize(z_zs - 15040, 14));
                        else c_phi0 <= unsigned(resize(z_zs, 14)); end if;
                        -- wrap into [0, 2 p64) (fold) or [0, p64) (ramp)
                        if z_zt < 0 then c_zuf <= unsigned(resize(z_zt + signed(resize(c_wl, 23)), 22));
                        elsif z_zt >= signed(resize(c_wl, 23)) then c_zuf <= unsigned(resize(z_zt - signed(resize(c_wl, 23)), 22));
                        else c_zuf <= unsigned(resize(z_zt, 22)); end if;
                    when 42 => c_phl0 <= resize(c_phi0(13 downto 4), 11) + 4;
                    -- wrap constants of the product field: A 940, B 940 = x (1024 - 64 - 16 - 4)
                    when 45 =>
                        v_w := signed(shift_left(resize(c_A, 24), 10)) - signed(shift_left(resize(c_A, 24), 6));
                        c_wp <= unsigned(v_w(22 downto 0));
                        v_w := signed(shift_left(resize(c_B, 24), 10)) - signed(shift_left(resize(c_B, 24), 6));
                        c_ws <= unsigned(v_w(19 downto 0));
                    when 46 =>
                        c_wp <= c_wp - shift_left(resize(c_A, 23), 4);
                        c_ws <= c_ws - shift_left(resize(c_B, 20), 4);
                    when 47 =>
                        c_wp <= c_wp - shift_left(resize(c_A, 23), 2);
                        c_ws <= c_ws - shift_left(resize(c_B, 20), 2);

                    -- slot results (z_res = the product >> 15 with the slot's exponent shift)
                    when 74  => c_invl <= z_res(15 downto 0);
                    when 98  => z_q <= f_acc(19 downto 10);
                    when 99  => c_k2g <= z_q - shift_right(z_q, 2) - shift_right(z_q, 3);
                    when 122 => c_binc <= z_res(9 downto 0);
                    when 123 =>
                        case to_integer(s_xs) is
                            when 4      => c_bincx <= shift_left(resize(c_binc, 12), 2);
                            when 3      => c_bincx <= shift_left(resize(c_binc, 12), 1) + resize(c_binc, 12);
                            when others => c_bincx <= shift_left(resize(c_binc, 12), 1);
                        end case;
                    when 146 =>                       -- v = d^2 >> 11 + d >> 2 (Q4 lines per field)
                        c_vq4 <= resize(f_acc(19 downto 11), 9) + resize(shift_right(s_vm, 2), 9);
                        c_vneg <= s_vneg;
                    when 147 =>
                        if c_p30 = '1' then c_vf <= shift_left(resize(c_vq4, 10), 1); else c_vf <= resize(c_vq4, 10); end if;
                    when 148 =>
                        z_zm1 <= shift_left(resize(c_vf, 15), 4) - shift_left(resize(c_vf, 15), 1);
                        z_zm2 <= resize(shift_right(c_vf, 1), 15) - resize(shift_right(c_vf, 3), 15);
                    when 149 => c_zm <= z_zm1 + z_zm2;
                    when 170 => c_p64 <= z_res(19 downto 0);
                    when 171 =>
                        c_pq <= c_p64(19 downto 6);
                        if s_ramp = '1' then c_wl <= resize(c_p64, 22); else c_wl <= shift_left(resize(c_p64, 22), 1); end if;
                    when 172 =>
                        c_2p <= shift_left(resize(c_pq, 15), 1);
                        if s_ramp = '1' then c_wx <= resize(c_pq, 15) - resize(s_xs, 15);
                        else c_wx <= shift_left(resize(c_pq, 15), 1) - resize(s_xs, 15); end if;
                        -- thresholds as fractions of the period: 0.094, 0.219, 0.344, 0.820 (one op per state)
                        c_th1 <= shift_right(c_pq, 3) - shift_right(c_pq, 5);
                        c_th2 <= shift_right(c_pq, 2) - shift_right(c_pq, 5);
                        z_th3 <= shift_right(c_pq, 1) - shift_right(c_pq, 3);
                        z_th4 <= c_pq - shift_right(c_pq, 3);
                    when 173 =>
                        c_th3 <= z_th3 - shift_right(c_pq, 5);
                        z_th4 <= z_th4 - shift_right(c_pq, 4);
                    when 174 => c_th4 <= z_th4 + shift_right(c_pq, 7);
                    when 194 => z_q <= f_acc(19 downto 10);
                    when 195 => z_eg <= shift_left(resize(z_q, 14), 3) + shift_left(resize(z_q, 14), 2);
                    when 196 => c_eg <= z_eg + resize(z_q, 14);                                                   -- 13 q
                    when 218 => c_g2 <= to_unsigned(470, 10) - resize(f_acc(18 downto 10), 10);
                    when 219 => z_g <= signed(resize(c_g2, 12)) - 282;
                    when 220 => z_g <= shift_left(z_g, 2) + z_g;
                    when 221 => z_g <= shift_right(z_g, 1);                              -- 2.5 (g2 - 282)
                    when 222 =>
                        if z_g < 0 then c_g1 <= (others => '0');
                        elsif z_g > 470 then c_g1 <= to_unsigned(470, 10);
                        else c_g1 <= unsigned(z_g(9 downto 0)); end if;
                    when 242 =>
                        if c_x0neg = '1' then c_bph0 <= (not f_acc(15 downto 0)) + 1; else c_bph0 <= f_acc(15 downto 0); end if;
                    when 266 => c_pw10 <= f_acc(25 downto 0);
                    when 290 => c_stw0 <= f_acc(19 downto 0);
                    when 314 => c_dp0 <= f_acc(23 downto 8);
                    when 338 => c_dstep <= f_acc(19 downto 8);
                    when 362 => c_rn0  <= resize(f_acc(23 downto 0), 26) + to_unsigned(C_C1, 26);
                    when 386 => c_rl0  <= resize(f_acc(23 downto 0), 26) + to_unsigned(C_C1, 26);
                    when 410 => c_rw20 <= resize(f_acc(23 downto 0), 26) - to_unsigned(C_CW2, 26);
                    when 434 => c_r30  <= resize(f_acc(23 downto 0), 26) - to_unsigned(C_C3, 26);
                    when 458 => c_rns  <= f_acc(15 downto 0);
                    when 482 => c_rls  <= f_acc(15 downto 0);
                    when 506 => c_rw2s <= f_acc(15 downto 0);
                    when 530 => c_r3s  <= f_acc(15 downto 0);
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
    -- write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: the vertical phase, the arc rate per h
    -- (cos ROM -> exp2 ROM -> x invl), the arc phase at x0, the product
    -- field's row value and step and the zone ramp (one shared adder each),
    -- the stripe / hatch rows from the phase ROM
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_c  : unsigned(7 downto 0);
        variable v_t  : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- row accumulators: one adder each, operand chosen a clock ahead
            if e_pen = '1' then
                e_pw10 <= e_pw10 + e_pwop;
                e_stw  <= e_stw + e_stop;
            end if;
            e_pen <= '0';
            if e_zen = '1' then e_zu <= e_zu + e_zop; end if;
            e_zen <= '0';

            if e_zk = '1' then
                -- line 0: load the field state
                e_phi4 <= c_phi0; e_wrap <= '0';
                e_vy <= (others => '0');
                e_zu <= c_zuf;
                e_pw10 <= c_pw10; e_stw <= c_stw0;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                -- next line: advance the state (the next phase was prepared at e_lc 60 .. 62)
                e_phi4 <= e_pnx; e_wrap <= e_pnw;
                e_vy <= e_vy + resize(s_ys, 20);
                e_pwop <= resize(c_dp0, 26); e_stop <= resize(c_dstep, 20); e_pen <= '1';
                e_zop <= (not resize(c_dz, 22)) + 1; e_zen <= '1';      -- a line down: the diagonal ramp falls by dz
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    if e_wrap = '1' then
                        e_pwop <= (not resize(c_wp, 26)) + 1; e_stop <= (not resize(c_ws, 20)) + 1; e_pen <= '1'; e_wrap <= '0';
                    end if;
                    if e_zu(21) = '1' then e_zop <= c_wl; e_zen <= '1'; end if;     -- went negative: + the period
                    e_phl <= e_phi4(13 downto 4);
                when 2 =>
                    r_cs_a <= e_phl;
                    if e_phl >= c_g1 then n_g1 <= '1'; else n_g1 <= '0'; end if;
                    if e_phl >= 470 then
                        n_p470 <= '1';
                        if e_phl - 470 >= c_g2 then n_g2 <= '1'; else n_g2 <= '0'; end if;
                    else
                        n_p470 <= '0'; n_g2 <= '0';
                    end if;
                    if e_vy(19 downto 8) < 87 or e_vy(19 downto 8) > 990 then n_frow <= '1'; else n_frow <= '0'; end if;
                when 4 =>
                    -- |cos| x k2g (sign kept); stripe and hatch rows
                    if r_cs_q < 0 then v_c := unsigned(-r_cs_q); e_cneg <= '1'; else v_c := unsigned(r_cs_q); e_cneg <= '0'; end if;
                    m_a <= resize(v_c, 36); m_b <= resize(c_k2g, 16); m_acc <= (others => '0');
                    n_strip <= r_st_q(0); n_hrow <= r_st_q(2 downto 1);
                when 21 =>                     -- e2 = +-(k2g |cos|) >> 8, Q8 octaves
                    if e_cneg = '1' then e_e2 <= -signed(resize(m_acc(18 downto 8), 11)); else e_e2 <= signed(resize(m_acc(18 downto 8), 11)); end if;
                when 22 =>
                    r_el_a <= unsigned(e_e2(7 downto 0)); e_sh <= e_e2(10 downto 8);
                when 24 =>
                    m_a <= resize(c_invl, 36); m_b <= r_el_q; m_acc <= (others => '0');
                when 41 =>                     -- finc = invl mant >> 15, << sh
                    v_t := f_esh(m_acc, e_sh);
                    e_fi <= v_t(16 downto 0);
                when 42 =>
                    case to_integer(s_xs) is
                        when 4      => n_fst <= resize(shift_left(resize(e_fi, 19), 2), 19);
                        when 3      => n_fst <= resize(shift_left(resize(e_fi, 19), 1) + resize(e_fi, 19), 19);
                        when others => n_fst <= resize(shift_left(resize(e_fi, 19), 1), 19);
                    end case;
                    m_a <= resize(e_fi, 36); m_b <= resize(c_x0h50, 16); m_acc <= (others => '0');
                when 59 =>                     -- F at x0 = finc (x0h - 50)
                    if c_x0neg = '1' then n_f0 <= (not m_acc(19 downto 0)) + 1; else n_f0 <= m_acc(19 downto 0); end if;
                when 60 => e_pn <= resize(e_phi4, 15) + resize(s_ys(11 downto 4), 15);
                when 61 =>
                    if e_pn >= 15040 then e_pnw <= '1'; else e_pnw <= '0'; end if;
                    e_pnx <= e_pn(13 downto 0);
                when 62 =>
                    if e_pnw = '1' then e_pnx <= e_pnx - 15040; end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: per-pixel DDAs (stage 1).  Arc phase F, the product field Pw1,
    -- four offset ramps (the road and skirt thresholds folded in, so the
    -- tests are plain compares), the zone ramp wrapping at its period, the
    -- bar phase and its column tick.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_bp : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px  <= (others => '0');
                b_xh  <= resize(s_x0, 13);
                b_f   <= n_f0;
                b_pw1 <= e_pw10;
                b_rn <= c_rn0; b_rl <= c_rl0; b_rw2 <= c_rw20; b_r3 <= c_r30;
                b_zu <= e_zu(20 downto 6);
                b_bph <= c_bph0; b_tick <= '1';
                l_fst <= n_fst; l_stw <= e_stw(19 downto 4);
                l_g1 <= n_g1; l_g2 <= n_g2; l_p470 <= n_p470; l_strip <= n_strip; l_frow <= n_frow; l_hrow <= n_hrow;
            elsif g_avid = '1' then
                b_px  <= b_px + 1;
                b_xh  <= b_xh + resize(s_xs, 13);
                b_f   <= b_f + resize(l_fst, 20);
                b_pw1 <= b_pw1 + resize(l_stw, 26);
                b_rn <= b_rn + resize(c_rns, 26); b_rl <= b_rl + resize(c_rls, 26);
                b_rw2 <= b_rw2 + resize(c_rw2s, 26); b_r3 <= b_r3 + resize(c_r3s, 26);
                if b_zu > c_wx then b_zu <= b_zu - c_wx; else b_zu <= b_zu + resize(s_xs, 15); end if;
                v_bp := resize(b_bph, 17) + resize(c_bincx, 17);
                b_bph <= v_bp(15 downto 0);
                b_tick <= v_bp(15) xor b_bph(15);
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_fl  : std_logic_vector(18 downto 0);
        variable v_z   : unsigned(15 downto 0);
        variable v_nb, v_wb, v_above, v_below, v_on1, v_on2, v_gate, v_bars, v_strp, v_hat, v_mos : std_logic;
        variable v_z3  : std_logic;
        variable v_arc : unsigned(1 downto 0);
        variable v_cls : unsigned(4 downto 0);
        variable v_zn  : unsigned(2 downto 0);
        variable v_qn, v_qw : unsigned(3 downto 0);
        variable v_xf  : std_logic_vector(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: road / skirt compares, keys, x-region flags (packed for ring B), the narrow road's
            -- palette index, the zone fold
            if b_pw1 > b_rn then p2_fl(0) <= '1'; else p2_fl(0) <= '0'; end if;
            if b_pw1 < b_rl then p2_fl(1) <= '1'; else p2_fl(1) <= '0'; end if;
            if b_pw1 > C_CW1 then p2_fl(2) <= '1'; else p2_fl(2) <= '0'; end if;
            if b_pw1 < b_rw2 then p2_fl(3) <= '1'; else p2_fl(3) <= '0'; end if;
            if b_pw1 < b_r3 then p2_mb <= '1'; else p2_mb <= '0'; end if;
            if b_bph < C_BAR then p2_fl(4) <= '1'; else p2_fl(4) <= '0'; end if;
            if b_xh < 436 or b_xh > 3654 then p2_fl(5) <= '1'; else p2_fl(5) <= '0'; end if;
            if b_xh < 1160 then p2_fl(6) <= '1'; else p2_fl(6) <= '0'; end if;
            if b_xh > 1800 then p2_fl(7) <= '1'; else p2_fl(7) <= '0'; end if;
            if b_xh < 2240 then p2_fl(8) <= '1'; else p2_fl(8) <= '0'; end if;
            if b_xh < 2600 then p2_fl(9) <= '1'; else p2_fl(9) <= '0'; end if;
            p2_fl(13 downto 10) <= std_logic_vector(b_pw1(18 downto 15));
            p2_qd <= b_pw1(18 downto 0) - b_rn(18 downto 0);
            p2_fl(18 downto 14) <= (others => '0');
            p2_f <= b_f(19 downto 4);
            if s_ramp = '0' and b_zu > c_pq then p2_zt <= resize(c_2p - b_zu, 14); else p2_zt <= b_zu(13 downto 0); end if;
            p2_tick <= b_tick;
            -- 3: the per-column sample and hold of the skirt compare (its own chain to stage 8); luma
            if p2_tick = '1' then p3_mbsh <= p2_mb; end if;
            p4_mbsh <= p3_mbsh; p5_mbsh <= p4_mbsh; p6_mbsh <= p5_mbsh; p7_mbsh <= p6_mbsh; p8_mbsh <= p7_mbsh;
            p3_lm <= unsigned(s_lb_q);
            -- 4: etch partials (luma nibbles x gain)
            p4_ea <= resize(p3_lm(3 downto 0) * c_eg, 18);
            p4_eb <= resize(p3_lm(7 downto 4) * c_eg, 18);
            -- 5: etch; ring A back (fabric copies)
            p5_le <= resize(p4_ea, 22) + shift_left(resize(p4_eb, 22), 4);
            p5_f  <= unsigned(rg_a0q);
            p5_zt <= unsigned(rg_a1q(13 downto 0));
            -- 6: etched arc phase (mod one cycle, bits 19..4) and zone position
            p6_fe <= p5_f + p5_le(19 downto 4);
            p6_zr <= resize(p5_zt, 16) + resize(p5_le(21 downto 12), 16);
            -- 7: clamp (fold) / wrap (ramp) the zone position
            v_z := p6_zr;
            if v_z >= resize(c_pq, 16) then
                if s_ramp = '1' then v_z := v_z - resize(c_pq, 16); else v_z := resize(c_pq, 16); end if;
            end if;
            p7_zte <= v_z(14 downto 0);
            p7_fe8 <= p6_fe(15 downto 8);
            -- 8: arc lines; zone code; ring B back (fabric copy)
            if p7_fe8 < 33 then p8_thick <= '1'; else p8_thick <= '0'; end if;
            if p7_fe8 >= 118 and p7_fe8 <= 138 then p8_thin <= '1'; else p8_thin <= '0'; end if;
            if p7_zte >= resize(c_th4, 15) then v_zn := "100";
            elsif p7_zte >= resize(c_th3, 15) then v_zn := "011";
            elsif p7_zte >= resize(c_th2, 15) then v_zn := "010";
            elsif p7_zte >= resize(c_th1, 15) then v_zn := "001";
            else v_zn := "000"; end if;
            p8_zone <= v_zn;
            p8_fl <= rg_b1q(2 downto 0) & rg_b0q;
            -- 9: compose -> class
            v_fl := p8_fl;
            -- bits: 18..15 qn (3 of them in rg_b1), 13..10 qw, 9 x<2600, 8 x<2240, 7 x>1800, 6 x<1160, 5 fcol, 4 bar,
            -- 3 wb2, 2 wb1, 1 nb2, 0 nb1
            v_qn := unsigned(rg_b1q(3 downto 0)); v_qw := unsigned(v_fl(13 downto 10)); v_xf := v_fl(9 downto 6);
            v_nb := v_fl(0) and v_fl(1);
            v_wb := v_fl(2) and v_fl(3);
            v_above := not v_fl(0);
            v_below := not v_fl(1);
            v_on1 := v_above and not v_wb and l_g1;
            v_on2 := v_below and l_g2;
            v_gate := v_on1 or v_on2;
            v_bars := (v_on1 and v_fl(4) and (v_xf(0) or s_bars)) or (v_on2 and v_fl(4) and not v_xf(2));
            v_strp := v_on1 and (not s_bars) and l_strip and not v_xf(0);
            v_hat  := v_below and (not v_on2) and v_fl(4) and v_xf(1) and v_xf(3);
            if p8_zone = 3 then v_z3 := '1'; else v_z3 := '0'; end if;
            v_mos := v_z3 and l_p470 and v_xf(2) and p8_mbsh;
            if p8_thick = '1' then v_arc := "10"; elsif p8_thin = '1' then v_arc := "01"; else v_arc := "00"; end if;
            v_cls := "000" & v_arc;                                           -- ground
            if v_hat = '1' and v_gate = '0' then v_cls := "01011" + resize(l_hrow, 5); end if;   -- hatch W C M k
            if v_gate = '1' and (v_bars = '1' or v_strp = '1') then
                if p8_zone = 0 then v_cls := "00011"; else v_cls := "00100"; end if;
            end if;
            if v_gate = '1' and p8_zone >= 2 then v_cls := "00011" + resize(p8_zone, 5); end if;  -- M Y k
            if v_mos = '1' then v_cls := "01000" + resize(v_arc, 5); end if;
            if v_nb = '1' then
                if p8_thick = '1' then v_cls := "01111"; else v_cls := '1' & v_qn; end if;
            end if;
            if v_wb = '1' then v_cls := '1' & v_qw; end if;
            if s_noframe = '0' and (v_fl(5) = '1' or l_frow = '1') then v_cls := "00111"; end if;
            p9_cls <= v_cls;
            -- 10 .. 12: delay
            p10_cls <= p9_cls; p11_cls <= p10_cls; p12_cls <= p11_cls;
            -- 13: palette ROM address; 14: ROM data (EBR); 15: fabric copy, black outside the generated avid
            r_pl_a <= s_pal & p12_cls;
            p13_q <= r_pl_q;
            if s_gav_sr(12) = '0' then
                s_y15 <= c_ky; s_u15 <= c_ku; s_v15 <= c_kv;
            else
                s_y15 <= unsigned(p13_q(31 downto 22)); s_u15 <= unsigned(p13_q(21 downto 12)); s_v15 <= unsigned(p13_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums, the 8-back tap from ring C; the HD HDMI bar fix), luma
    -- [1 2 1]/4 aligned to the box's centre, luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ly : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 16: the running 8-sums snap back to the palette black (the off-avid input) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good
            s_cau7 <= unsigned(rg_c0q(9 downto 0));
            s_cav7 <= unsigned(rg_c1q(9 downto 0));
            if s_bah = "00000000" then
                s_bau <= c_ku & "000"; s_bav <= c_kv & "000";
            else
                s_bau <= s_bau + s_u15 - s_cau7;
                s_bav <= s_bav + s_v15 - s_cav7;
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(1) <= s_y15;
            for i in 2 to 6 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 17: box output; luma [1 2 1] at the box's centre (4 back)
            v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            v_ly := resize(s_ly(4), 12) + shift_left(resize(s_ly(5), 12), 1) + resize(s_ly(6), 12);
            s_y17 <= v_ly(11 downto 2);
            -- 18: luma 13/16
            if s_y17 > 64 then v_dl := s_y17 - 64; else v_dl := (others => '0'); end if;
            s_y18 <= s_y17 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u18 <= s_u17; s_v18 <= s_v17;
            -- 19: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y19);
    data_out.u       <= std_logic_vector(s_u19);
    data_out.v       <= std_logic_vector(s_v19);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture placard;
