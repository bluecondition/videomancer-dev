-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Nesting v0.1.0 -- Heirloom (video phase), from the clip WIN_20230627_23_37_56 at 7.0 s.
--
-- The picture: two eyes of nested, bevelled squares (x 547 / 1489, mirrored about x 1018) in blue, cyan and black
-- contour rings with red beyond, overlaid by magenta / purple key rings (centred 21 px further right, 97 px apart)
-- that run on as vertical stripes into a purple vase down the centre; a floor of red / blue diamonds below line
-- 846 with grey-teal ghost teeth along the bottom; every edge torn into a fine zigzag (runs of 2 lines, period 10
-- lines) and gently wobbling.  In the clip an LFO (5.46 s) breathes the contour gain: the eyes hold for ~2 s,
-- then the field collapses into a dense lattice while the magenta rings thin away, and back.
--
-- The patch (nesting.py has the tables, .heirloom_work/rb_nesting/notes.md the measurements):
--   layer A  u = ||x - 1018| - a|, v = |y - 510|; Tu, Tv = triangle folders of g u / 471 and g v / 471;
--            F = max(Tu, Tv) + k min(Tu, Tv); colour = PAL[F * 320 / 1024 + etch * luma]
--            floor (y >= 846): H gain 1.9 sqrt g, V gain 8.6 sqrt g, k = 1 (diamonds), PAL[F * 112 / 1024 + 204]
--   layer B  u' = ||x - 1038| - a| / 97, v' = 0.88 |y - 516| / 97; FB = max + k min; magenta where the ring
--            phase is within the duty (thicker on the v sides, toward the window edge and down the vase)
--   tear     every 2 lines a saw offset (+-6 px, 10 lines) plus a 240-line wobble, on both layers
--
--   K1  Eyes     the two eyes' spacing: merged into one (0) .. the still (512) .. out at the edges (1023)
--   K2  Corners  the rings' corners: 4-point stars (0) .. the still's bevelled squares (420) .. diamonds (1023)
--   K3  Pink     the magenta rings' spacing: 2.3x wider (0) .. the still (97 px) .. 2.3x finer (1023)
--   K4  Breath   the LFO: still (0) .. the clip (512: 5.46 s, gain x1 .. x5) .. fast and deep (1023)
--   K5  Tear     the zigzag and wobble: clean (0) .. the still (512) .. shredded (1023)
--   K6  Etch     how far the camera (or the Osc, 8x) bends both ring systems
--   S7  Quad     two rows of eyes (the V ramps folded again at +-270 lines)
--   S8  Pink     the magenta key layer on (the still) / off (the bare contour field)
--   S9  Palette  Photo / Ember (gold, lime and teal contours)
--   S10 Colours  Forward (the still) / Reverse: the contour colouriser runs backwards (red-cored eyes in blue)
--   S11 Source   Video / Osc
--   P12 NESTING  the contour gain: one huge nested square per eye (0) .. the still (300) .. a dense lattice (100 %)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine with its own serial
-- multiplier (computes the NEXT line), a beam with two folded phase DDAs, a 15-stage pixel path to the compose, then
-- the shell's output conditioning (two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16) and the
-- blanking gate.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; one latency (C_LAT 20)
-- in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.nesting_pkg.all;
use work.all;

architecture nesting of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_CA2  : integer := 2036;     -- layer A mirror x (h = half virtual px)
    constant C_CB2  : integer := 2076;     -- layer B mirror x (h)
    constant C_PHB  : integer := -19594;   -- B ring phase offset (Q16 rings): rings centred 77.5 + 97 n px
    constant C_PHB10 : integer := -306;    -- the same in Q10 rings (the key datapath)
    constant C_PV0  : integer := 314573;   -- floor V phase at line 846 (0.3 units, Q20)

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
    -- '1' for 50 / 60 field-per-second modes (the LFO step halves), '0' for 1080p
    function f_half(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 3 | 7 | 9 | 11 | 13 | 15 => return '0';
            when others                   => return '1';
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- triangle folder of a Q20 phase (period 2 units): 10-bit value, 1023 = the peak
    function f_fold(p : unsigned(20 downto 0)) return unsigned is
    begin
        if p(20) = '1' then return not p(19 downto 10); else return p(19 downto 10); end if;
    end function;
    -- half-wave cosine ROM index for a Q16 turn phase (cos(2 pi p) = COS[idx])
    function f_cidx(p : unsigned(15 downto 0)) return unsigned is
    begin
        if p(15) = '1' then return not p(14 downto 7); else return p(14 downto 7); end if;
    end function;
    constant C_BKY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_BKU : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BKV : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- ghost grey (110, 140, 135), prescaled
    constant C_GHY : unsigned(9 downto 0) := to_unsigned(615, 10);
    constant C_GHU : unsigned(9 downto 0) := to_unsigned(461, 10);
    constant C_GHV : unsigned(9 downto 0) := to_unsigned(521, 10);

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_cos_a, r_lc_a : unsigned(7 downto 0) := (others => '0');
    signal r_cos_q, r_lc_q : unsigned(9 downto 0) := (others => '0');
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_bc_a  : unsigned(7 downto 0) := (others => '0');
    signal r_bc_q  : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_x0   : unsigned(9 downto 0) := (others => '0');
    signal s_half : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw breath, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(420 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_quad, s_pink, s_pal, s_rev, s_src : std_logic := '0';

    -- per-field scratch and constants
    signal z_er   : signed(9 downto 0) := (others => '0');          -- LFO rate exponent, Q8 octaves
    signal z_d4   : unsigned(9 downto 0) := (others => '0');
    signal z_ern  : std_logic := '0';
    signal z_mr   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sr   : signed(2 downto 0) := (others => '0');
    signal z_c0   : unsigned(9 downto 0) := (others => '0');         -- (1 - cos phi) / 2, Q10
    signal z_e0   : signed(11 downto 0) := (others => '0');          -- gain exponent from P12, Q8
    signal z_e0n  : std_logic := '0';
    signal z_d    : unsigned(9 downto 0) := to_unsigned(594, 10);    -- breath depth, Q8 octaves
    signal z_b1   : unsigned(9 downto 0) := (others => '0');
    signal z_b    : unsigned(10 downto 0) := (others => '0');        -- breath, Q10
    signal z_e    : signed(11 downto 0) := (others => '0');          -- gain exponent, Q8
    signal z_me, z_me2, z_mb : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_se, z_se2, z_sb : signed(2 downto 0) := (others => '0');
    signal z_eb   : signed(9 downto 0) := (others => '0');
    signal z_ebn  : std_logic := '0';
    signal z_k1hi : std_logic := '0';
    signal c_phi  : unsigned(15 downto 0) := to_unsigned(59895, 16); -- LFO phase (power-up: the still, 0.47 s
                                                                     -- before the eye-state turnaround)
    signal c_dphi : unsigned(15 downto 0) := to_unsigned(200, 16);
    signal c_phw  : unsigned(15 downto 0) := (others => '0');        -- wobble phase (drifts)
    signal c_a2   : unsigned(10 downto 0) := to_unsigned(942, 11);   -- eye half-spacing, h
    signal c_ab2  : unsigned(10 downto 0) := to_unsigned(940, 11);
    signal c_fa1, c_fa3, c_fb1, c_fb3 : signed(13 downto 0) := (others => '0');
    signal c_kq   : signed(6 downto 0) := to_signed(9, 7);           -- corner mix k, Q6
    signal c_sh   : unsigned(13 downto 0) := to_unsigned(1113, 14);  -- A H step per h, Q20 units
    signal c_shf  : unsigned(13 downto 0) := to_unsigned(2115, 14);  -- floor H step per h
    signal c_svf  : unsigned(16 downto 0) := to_unsigned(19146, 17); -- floor V step per px
    signal c_sb   : unsigned(9 downto 0) := to_unsigned(338, 10);    -- B step per h, Q16 rings
    signal c_svb  : unsigned(10 downto 0) := to_unsigned(595, 11);   -- B V step per px
    signal c_sg   : unsigned(9 downto 0) := to_unsigned(303, 10);    -- ghost slant per line
    signal c_ta   : unsigned(9 downto 0) := to_unsigned(192, 10);    -- tear amplitude, h Q4
    signal c_kl   : unsigned(12 downto 0) := to_unsigned(6554, 13);  -- tear rate, Q16 per line
    signal c_wa   : unsigned(4 downto 0) := to_unsigned(10, 5);      -- wobble amplitude, h
    signal c_eg   : unsigned(8 downto 0) := (others => '0');         -- etch gain
    signal c_wu   : unsigned(5 downto 0) := to_unsigned(22, 6);      -- B duty (Q6), u sides / v sides
    signal c_wv   : unsigned(5 downto 0) := to_unsigned(31, 6);
    signal c_da   : signed(13 downto 0) := to_signed(1094, 14);     -- CA2 - x0 - a2: |c_da - offset| = u at x0
    signal c_db   : signed(13 downto 0) := to_signed(1136, 14);
    signal c_sta  : unsigned(15 downto 0) := to_unsigned(2226, 16); -- per-pixel steps: SH xs, SHf xs, SB xs
    signal c_staf : unsigned(15 downto 0) := to_unsigned(4230, 16);
    signal c_stb  : unsigned(11 downto 0) := to_unsigned(676, 12);

    -- field serial multiplier
    signal z_fa  : unsigned(15 downto 0) := (others => '0');
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

    -- line engine (serial multiplier m_*)
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');       -- Q8 virtual lines
    signal e_par  : std_logic := '0';
    signal e_vp   : unsigned(10 downto 0) := (others => '0');       -- virtual line (px)
    signal e_fl   : std_logic := '0';
    signal e_av   : unsigned(9 downto 0) := (others => '0');        -- |y - 513| (Quad: folded again at 270)
    signal e_off  : signed(7 downto 0) := (others => '0');          -- tear offset, h (held over 2 lines)
    signal e_wob  : signed(7 downto 0) := (others => '0');
    signal e_offt : signed(8 downto 0) := (others => '0');
    signal e_ua, e_ub : unsigned(11 downto 0) := (others => '0');
    signal e_shl  : unsigned(13 downto 0) := (others => '0');
    signal e_tv   : unsigned(9 downto 0) := (others => '0');
    signal e_t2, e_t3 : unsigned(4 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');      -- shared with the field sequencer (zone only)
    signal m_b    : unsigned(16 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    -- next line's values
    signal n_pa0  : unsigned(20 downto 0) := (others => '0');
    signal n_pb0  : unsigned(21 downto 0) := (others => '0');
    signal n_xh0, n_x1, n_x2 : signed(13 downto 0) := (others => '0');

    signal n_vb   : unsigned(15 downto 0) := (others => '0');
    signal n_gh   : std_logic := '0';
    signal n_gofs : unsigned(15 downto 0) := (others => '0');

    -- beam and line constants
    signal b_pa   : unsigned(20 downto 0) := (others => '0');
    signal b_pb   : unsigned(21 downto 0) := (others => '0');
    signal b_da, b_db : std_logic := '0';
    signal b_xh, b_x2 : signed(13 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal l_tv   : unsigned(9 downto 0) := (others => '0');
    signal l_fl   : std_logic := '0';
    signal l_sta  : unsigned(15 downto 0) := (others => '0');
    signal l_stb  : unsigned(11 downto 0) := (others => '0');
    signal l_vb   : unsigned(15 downto 0) := (others => '0');
    signal l_avb  : unsigned(9 downto 0) := (others => '0');
    signal l_t2, l_t3 : unsigned(4 downto 0) := (others => '0');
    signal l_gh   : std_logic := '0';
    signal l_gofs : unsigned(15 downto 0) := (others => '0');
    signal l_kk   : signed(7 downto 0) := (others => '0');

    -- pixel path
    signal p2_mxa, p2_mna : unsigned(9 downto 0) := (others => '0');
    signal p2_mxb, p2_mnb : unsigned(15 downto 0) := (others => '0');
    signal p2_reg : std_logic := '0';
    signal p2_dx  : signed(13 downto 0) := (others => '0');
    signal p2_gs  : unsigned(15 downto 0) := (others => '0');
    signal p3_mxa : unsigned(9 downto 0) := (others => '0');
    signal p3_mxb : unsigned(15 downto 0) := (others => '0');
    signal p3_ka  : unsigned(12 downto 0) := (others => '0');
    signal p3_kb  : signed(15 downto 0) := (others => '0');
    signal p3_kc  : unsigned(18 downto 0) := (others => '0');
    signal p3_kd  : signed(21 downto 0) := (others => '0');
    signal p3_reg, p3_gok : std_logic := '0';
    signal p3_db  : unsigned(12 downto 0) := (others => '0');
    signal p4_mxa : unsigned(9 downto 0) := (others => '0');
    signal p4_mxb : unsigned(15 downto 0) := (others => '0');
    signal p4_kma : signed(11 downto 0) := (others => '0');
    signal p4_kmb : signed(16 downto 0) := (others => '0');
    signal p4_reg, p4_gok, p4_d1, p4_d2, p4_gd : std_logic := '0';
    signal p4_q   : unsigned(2 downto 0) := (others => '0');
    signal p4_dd  : unsigned(12 downto 0) := (others => '0');
    signal p5_fa  : unsigned(11 downto 0) := (others => '0');
    signal p5_fb  : signed(17 downto 0) := (others => '0');
    signal p5_ws  : unsigned(12 downto 0) := (others => '0');
    signal p5_wv  : unsigned(6 downto 0) := (others => '0');
    signal p5_q   : unsigned(2 downto 0) := (others => '0');
    signal p5_gh  : std_logic := '0';
    signal p6_fa  : unsigned(11 downto 0) := (others => '0');
    signal p6_rp  : signed(17 downto 0) := (others => '0');
    signal p6_win, p6_gh : std_logic := '0';
    signal p6_wv  : unsigned(6 downto 0) := (others => '0');
    signal p6_q   : unsigned(2 downto 0) := (others => '0');
    signal p7_f   : unsigned(11 downto 0) := (others => '0');
    signal p7_a2r : unsigned(6 downto 0) := (others => '0');
    signal p7_r2, p7_win, p7_gh : std_logic := '0';
    signal p7_wv  : unsigned(6 downto 0) := (others => '0');
    signal p7_q   : unsigned(2 downto 0) := (others => '0');
    signal p8_ix  : unsigned(19 downto 0) := (others => '0');
    signal p8_key, p8_gh : std_logic := '0';
    signal p8_bi  : unsigned(4 downto 0) := (others => '0');
    signal p9_idx : unsigned(8 downto 0) := (others => '0');
    signal p9_bi : unsigned(4 downto 0) := (others => '0');
    signal p9_key, p9_gh : std_logic := '0';
    signal p12_bc : std_logic_vector(31 downto 0) := (others => '0');
    signal p10_key, p11_key, p12_key, p10_gh, p11_gh, p12_gh, p13_gh : std_logic := '0';
    signal p12_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- luma chain: lumA valid with the pixel at stage 5
    signal g_l1   : unsigned(7 downto 0) := (others => '0');
    signal g_ta, g_tb : signed(14 downto 0) := (others => '0');
    signal g_lum  : signed(11 downto 0) := (others => '0');

    -- output conditioning: the two boxes' 8-sample delays share one block-RAM ring (written every clock,
    -- read 6 behind, so read and write never meet: no_rw_check), plus a fabric register
    type t_cr is array (0 to 255) of std_logic_vector(59 downto 0);
    signal cr : t_cr := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of cr : signal is true;
    signal cr_wa, cr_ra : unsigned(7 downto 0) := (others => '0');
    signal cr_q, cr_q2 : std_logic_vector(59 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs
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
            r_ex_q  <= C_NS_EXP2(to_integer(r_ex_a));
            r_cos_q <= C_NS_COS(to_integer(r_cos_a));
            r_lc_q  <= C_NS_COS(to_integer(r_lc_a));
            r_pal_q <= C_NS_PAL(to_integer(r_pal_a));
            r_bc_q  <= C_NS_BCOL(to_integer(r_bc_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : signed(12 downto 0);
        variable v_t  : unsigned(31 downto 0);
        variable v_u  : unsigned(11 downto 0);
        variable v_s  : signed(12 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_quad <= registers_in(6)(0);
                s_pink <= not registers_in(6)(1);       -- S8: Off = 1 (the switch's "Off" label is value 1)
                s_pal  <= registers_in(6)(2);
                s_rev  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_half <= f_half(s_tid);
                case to_integer(s_xsr) is
                    when 4 =>      s_oxs <= to_unsigned(204, 9); s_x0 <= to_unsigned(480, 10);
                    when 3 =>      s_oxs <= to_unsigned(153, 9); s_x0 <= (others => '0');
                    when others => s_oxs <= to_unsigned(102, 9); s_x0 <= (others => '0');
                end case;
            end if;
            g_wm1 <= s_w - 1;
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

            -- serial products: slot k is decoded at zc = 40 + 24 k (operands pre-muxed), loads one clock later,
            -- 18 iterations, the result is read at zc = 62 + 24 k
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 64  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 88  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 112 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 136 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 160 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 184 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 208 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 240 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 264 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 288 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 312 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 344 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 368 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 392 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 416 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 440 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 464 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 488 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 512 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 =>                                                          -- eye spacing
                        if z_k1hi = '1' then
                            z_fa <= resize(s_g1(12 downto 3) - 512, 16); z_fb <= to_unsigned(391, 16);
                        else
                            z_fa <= resize(s_g1(12 downto 3), 16);       z_fb <= to_unsigned(471, 16);
                        end if;
                    when 1  => z_fa <= resize(s_g2(12 downto 3), 16);    z_fb <= to_unsigned(93, 16);    -- corners
                    when 2  => z_fa <= resize(unsigned(abs(z_e0)), 16);  z_fb <= to_unsigned(236, 16);   -- |p12 - 300| e0
                    when 3  => z_fa <= resize(s_k4, 16);                 z_fb <= to_unsigned(297, 16);   -- depth
                    when 4  =>                                                                            -- LFO step
                        if s_half = '1' then z_fa <= to_unsigned(200, 16); else z_fa <= to_unsigned(400, 16); end if;
                        z_fb <= resize(z_mr, 16);
                    when 5  => z_fa <= resize(z_c0, 16);                 z_fb <= to_unsigned(82, 16);
                    when 6  =>
                        if z_c0 > 358 then z_fa <= resize(z_c0 - 358, 16); else z_fa <= (others => '0'); end if;
                        z_fb <= to_unsigned(1449, 16);
                    when 7  => z_fa <= resize(z_d, 16);                  z_fb <= resize(z_b, 16);         -- D b
                    when 8  => z_fa <= to_unsigned(1113, 16);            z_fb <= resize(z_me, 16);        -- SH
                    when 9  => z_fa <= to_unsigned(2115, 16);            z_fb <= resize(z_me2, 16);       -- SHf
                    when 10 => z_fa <= to_unsigned(19146, 16);           z_fb <= resize(z_me2, 16);       -- SVf
                    when 11 => z_fa <= resize(unsigned(abs(z_eb)), 16);  z_fb <= to_unsigned(154, 16);    -- |k3 - 512| eB
                    when 12 => z_fa <= to_unsigned(338, 16);             z_fb <= resize(z_mb, 16);        -- SB
                    when 13 => z_fa <= resize(c_sb, 16);                 z_fb <= to_unsigned(451, 16);    -- SVB
                    when 14 => z_fa <= resize(c_sb, 16);                 z_fb <= to_unsigned(230, 16);    -- SG
                    when 15 => z_fa <= resize(s_g5(12 downto 3), 16);    z_fb <= resize(s_g5(12 downto 3), 16);   -- tear^2
                    when 16 => z_fa <= resize(s_g6(12 downto 3), 16);    z_fb <= resize(s_g6(12 downto 3), 16);   -- etch^2
                    when 17 => z_fa <= resize(c_sh, 16);                 z_fb <= resize(s_xs, 16);        -- SH xs
                    when 18 => z_fa <= resize(c_shf, 16);                z_fb <= resize(s_xs, 16);        -- SHf xs
                    when others => z_fa <= resize(c_sb, 16);             z_fb <= resize(s_xs, 16);        -- SB xs
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>                        -- LFO: advance the phase (last field's step); rate exponent
                        c_phi <= c_phi + c_dphi;
                        c_phw <= c_phw + 137;
                        z_ern <= not s_k4(9);
                        if s_k4(9) = '1' then
                            z_er <= signed('0' & s_k4(8 downto 0));               -- k4 - 512 (Q8 octaves)
                        else
                            z_d4 <= to_unsigned(512, 10) - s_k4;                  -- 512 - k4
                        end if;
                    when 31 =>                        -- k4 < 512: 0.75 (k4 - 512) = -(d - d / 4) (unsigned, no
                        if z_ern = '1' then           -- shared sign bits in one adder)
                            z_er <= -signed(z_d4 - shift_right(z_d4, 2));
                        end if;
                        z_k1hi <= s_g1(12) and (s_g1(11) or s_g1(10) or s_g1(9) or s_g1(8) or s_g1(7)
                                  or s_g1(6) or s_g1(5) or s_g1(4) or s_g1(3));
                        z_e0n <= '0';
                        if s_gp(12 downto 3) < 300 then z_e0n <= '1'; end if;
                        z_e0 <= signed(resize(s_gp(12 downto 3), 12)) - 300;
                        z_ebn <= not s_g3(12);
                        z_eb <= signed(resize(s_g3(12 downto 3), 10) - 512);
                    when 32 =>
                        r_ex_a <= unsigned(z_er(7 downto 0));
                    when 33 => z_sr <= resize(shift_right(z_er, 8), 3);
                    when 34 =>
                        z_mr <= r_ex_q;
                        z_c0 <= to_unsigned(511, 10) - unsigned(resize(signed(r_cos_q), 10));
                    when 62 =>                        -- a2 = k1 * 471 >> 8, or 942 + (k1 - 512) * 391 >> 8
                        if z_k1hi = '1' then c_a2 <= resize(m_acc(18 downto 8), 11) + 942;
                        else c_a2 <= resize(m_acc(18 downto 8), 11); end if;
                    when 64 =>
                        if c_a2 > 2 then c_ab2 <= c_a2 - 2; else c_ab2 <= (others => '0'); end if;
                        c_fa1 <= to_signed(C_CA2, 14) - signed(resize(c_a2, 14));
                        c_fa3 <= to_signed(C_CA2, 14) + signed(resize(c_a2, 14));
                    when 66 =>
                        c_fb1 <= to_signed(C_CB2, 14) - signed(resize(c_ab2, 14));
                        c_fb3 <= to_signed(C_CB2, 14) + signed(resize(c_ab2, 14));
                        c_da  <= c_fa1 - signed(resize(s_x0, 14));
                    when 68 =>
                        c_db  <= c_fb1 - signed(resize(s_x0, 14));
                    when 86 =>                        -- k = (k2 * 93 >> 10) - 29, Q6
                        c_kq <= signed(resize(m_acc(16 downto 10), 7)) - 29;
                    when 110 =>                       -- e0 = +-(|p12 - 300| * 236 >> 8)
                        if z_e0n = '1' then z_e0 <= -signed(resize(m_acc(19 downto 8), 12));
                        else z_e0 <= signed(resize(m_acc(19 downto 8), 12)); end if;
                    when 134 =>                       -- D = k4 * 297 >> 8, or 594 + (k4 - 512) / 2
                        if s_k4(9) = '1' then z_d <= to_unsigned(594, 10) + resize(s_k4(8 downto 1), 10);
                        else z_d <= resize(m_acc(17 downto 8), 10); end if;
                    when 158 =>                       -- LFO step = base * 2^(er / 256)
                        case to_integer(z_sr) is
                            when -2     => c_dphi <= resize(m_acc(31 downto 17), 16);
                            when -1     => c_dphi <= resize(m_acc(31 downto 16), 16);
                            when 0      => c_dphi <= resize(m_acc(31 downto 15), 16);
                            when others => c_dphi <= resize(m_acc(31 downto 14), 16);
                        end case;
                    when 182 => z_b1 <= resize(m_acc(19 downto 10), 10);
                    when 206 => z_b <= resize(z_b1, 11) + resize(m_acc(20 downto 10), 11);
                    when 230 =>                       -- e = clamp(e0 + D b >> 10, -276, 918)
                        v_e := resize(z_e0, 13) + signed(resize(m_acc(21 downto 10), 13));
                        if v_e < -276 then v_e := to_signed(-276, 13); elsif v_e > 918 then v_e := to_signed(918, 13); end if;
                        z_e <= resize(v_e, 12);
                    when 232 =>
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_se <= resize(shift_right(z_e, 8), 3);
                        -- B duty: u sides 22 - 5e/256 (6..27), v sides 31 - 7e/256 (9..38)
                        v_e := shift_right(resize(z_e, 13) + shift_left(resize(z_e, 13), 2), 8);
                        v_e := to_signed(22, 13) - v_e;
                        if v_e < 6 then c_wu <= to_unsigned(6, 6); elsif v_e > 27 then c_wu <= to_unsigned(27, 6);
                        else c_wu <= unsigned(v_e(5 downto 0)); end if;
                    when 233 =>
                        v_e := shift_right(shift_left(resize(z_e, 13), 3) - resize(z_e, 13), 8);
                        v_e := to_signed(31, 13) - v_e;
                        if v_e < 9 then c_wv <= to_unsigned(9, 6); elsif v_e > 38 then c_wv <= to_unsigned(38, 6);
                        else c_wv <= unsigned(v_e(5 downto 0)); end if;
                    when 234 => z_me <= r_ex_q;
                    when 236 =>
                        r_ex_a <= unsigned(z_e(8 downto 1)); z_se2 <= resize(shift_right(z_e, 9), 3);
                    when 238 => z_me2 <= r_ex_q;
                    when 262 =>                       -- SH = 1113 * 2^(e/256), Q20 per h
                        case to_integer(z_se) is
                            when -2     => c_sh <= resize(m_acc(31 downto 17), 14);
                            when -1     => c_sh <= resize(m_acc(31 downto 16), 14);
                            when 0      => c_sh <= resize(m_acc(31 downto 15), 14);
                            when 1      => c_sh <= resize(m_acc(31 downto 14), 14);
                            when 2      => c_sh <= resize(m_acc(31 downto 13), 14);
                            when others => c_sh <= resize(m_acc(31 downto 12), 14);
                        end case;
                    when 286 =>
                        case to_integer(z_se2) is
                            when -1     => c_shf <= resize(m_acc(31 downto 16), 14);
                            when 0      => c_shf <= resize(m_acc(31 downto 15), 14);
                            when others => c_shf <= resize(m_acc(31 downto 14), 14);
                        end case;
                    when 310 =>
                        case to_integer(z_se2) is
                            when -1     => c_svf <= resize(m_acc(31 downto 16), 17);
                            when 0      => c_svf <= resize(m_acc(31 downto 15), 17);
                            when others => c_svf <= resize(m_acc(31 downto 14), 17);
                        end case;
                    when 334 =>                       -- eB = +-(|k3 - 512| * 154 >> 8)
                        if z_ebn = '1' then z_eb <= -signed(resize(m_acc(17 downto 8), 10));
                        else z_eb <= signed(resize(m_acc(17 downto 8), 10)); end if;
                    when 336 =>
                        r_ex_a <= unsigned(z_eb(7 downto 0)); z_sb <= resize(shift_right(z_eb, 8), 3);
                    when 338 => z_mb <= r_ex_q;
                    when 366 =>                       -- SB = 338 * 2^(eB/256), Q16 rings per h
                        case to_integer(z_sb) is
                            when -2     => c_sb <= resize(m_acc(31 downto 17), 10);
                            when -1     => c_sb <= resize(m_acc(31 downto 16), 10);
                            when 0      => c_sb <= resize(m_acc(31 downto 15), 10);
                            when others => c_sb <= resize(m_acc(31 downto 14), 10);
                        end case;
                    when 390 => c_svb <= resize(m_acc(31 downto 8), 11);
                    when 414 => c_sg <= resize(m_acc(31 downto 8), 10);
                    when 438 =>                       -- tear: tA = 3 k5^2 >> 12 (h Q4); rate; wobble 5 k5 >> 8
                        v_t := m_acc + shift_left(m_acc, 1);
                        c_ta <= resize(v_t(21 downto 12), 10);
                        if s_g5(12) = '1' then
                            c_kl <= to_unsigned(6554, 13) - shift_left(resize(s_g5(11 downto 3), 13), 2);
                        else
                            c_kl <= to_unsigned(6554, 13);
                        end if;
                        v_u := resize(s_g5(12 downto 3), 12) + shift_left(resize(s_g5(12 downto 3), 12), 2);
                        c_wa <= resize(v_u(11 downto 8), 5);
                    when 462 =>                       -- eg = 5 k6^2 >> 14
                        v_t := m_acc + shift_left(m_acc, 2);
                        c_eg <= resize(v_t(22 downto 14), 9);
                    when 486 => c_sta  <= m_acc(15 downto 0);
                    when 510 => c_staf <= m_acc(15 downto 0);
                    when 534 => c_stb  <= m_acc(11 downto 0);
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
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid rise, field-latched width)
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer write of (luma >> 2) - 16
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
    -- Line engine: kicked at each line start (and once in the zone for line 0), computes the NEXT line with a
    -- serial multiplier (one product per 18 clocks): the V folder phase, the tear saw and the wobble, the start
    -- phases of the two beam DDAs, the B line terms.  ~200 clocks, so it fits SD's 858-clock line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(13 downto 0);
        variable v_a  : unsigned(10 downto 0);
        variable v_p  : unsigned(31 downto 0);
        variable v_o  : signed(13 downto 0);
        variable v_sx : unsigned(15 downto 0);
        variable v_t2 : unsigned(12 downto 0);
        variable v_t3 : unsigned(9 downto 0);
        variable v_i  : integer range 0 to 255;
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1500 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0'); e_par <= '0';
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + s_ys; e_par <= not e_par;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            -- serial multiplier: loaded by the field sequencer (z_ld, vblank zone) or at the step that sets
            -- m_a / m_b; 17 iterations.  The two never overlap (the zone has no line starts, the line-0 kick
            -- comes at zc 1500, after the last field slot).
            if z_ld = '1' then
                m_acc <= (others => '0');
                m_a <= resize(z_fa, 32); m_b <= resize(z_fb, 17);
            elsif m_b /= 0 then
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= m_a(30 downto 0) & '0';
                m_b <= '0' & m_b(16 downto 1);
            end if;

            v_i := to_integer(e_lc);
            case v_i is
                when 1 =>                      -- the line's virtual row
                    e_vp <= resize(e_vy(19 downto 8), 11);
                when 2 =>                      -- floor (Max, not Quad); |y - 513|
                    if e_vp >= 846 and s_quad = '0' then e_fl <= '1'; else e_fl <= '0'; end if;
                    if e_vp >= 513 then e_av <= resize(e_vp - 513, 10); else e_av <= resize(513 - e_vp, 10); end if;
                when 3 =>
                    if s_quad = '1' then
                        if e_av >= 270 then e_av <= e_av - 270; else e_av <= 270 - e_av; end if;
                    end if;
                when 4 =>                      -- M1: V folder phase
                    m_acc <= (others => '0');
                    if e_fl = '1' then
                        m_a <= resize(e_vp - 846, 32); m_b <= c_svf;
                    else
                        m_a <= resize(e_av, 32); m_b <= resize(c_sh & '0', 17);
                    end if;
                when 22 =>
                    if e_fl = '1' then v_p := m_acc + to_unsigned(C_PV0, 32); else v_p := m_acc; end if;
                    e_tv <= f_fold(v_p(20 downto 0));
                    -- M2: tear saw phase vp * kL
                    m_acc <= (others => '0');
                    m_a <= resize(e_vp, 32); m_b <= resize(c_kl, 17);
                when 40 =>                     -- M3 (even lines; held over 2): tA * pht
                    v_p := m_acc + shift_left(resize(e_vp(10 downto 5), 32), 13);
                    if e_par = '0' then
                        m_acc <= (others => '0');
                        m_a <= resize(c_ta, 32); m_b <= resize(v_p(15 downto 0), 17);
                    end if;
                when 58 =>
                    if e_par = '0' then        -- off = ((tA pht >> 15) - tA) >> 4  (Q4 -> h)
                        v_o := signed(resize(m_acc(25 downto 15), 14)) - signed(resize(c_ta, 14));
                        e_off <= resize(shift_right(v_o, 4), 8);
                    end if;
                    -- M4: wobble phase vp * 273 (+ drift)
                    m_acc <= (others => '0');
                    m_a <= resize(e_vp, 32); m_b <= to_unsigned(273, 17);
                when 76 =>
                    v_sx := m_acc(15 downto 0) + c_phw;
                    r_lc_a <= f_cidx(v_sx);
                when 78 =>                     -- M5: (cos + 512) * wA (offset binary)
                    m_acc <= (others => '0');
                    m_a <= resize(c_wa, 32);
                    m_b <= resize(unsigned(signed(r_lc_q) + 512), 17);
                when 96 =>                     -- wob = ((cos + 512) wA >> 9) - wA
                    v_o := signed(resize(m_acc(14 downto 9), 14)) - signed(resize(c_wa, 14));
                    e_wob <= resize(v_o, 8);
                when 97 =>
                    e_offt <= resize(e_off, 9) + resize(e_wob, 9);
                    if e_fl = '1' then e_shl <= c_shf; else e_shl <= c_sh; end if;
                when 98 =>                     -- the first pixel's x (h) and |u| there: |c_d - offset|
                    n_xh0 <= signed(resize(s_x0, 14)) + resize(e_offt, 14);
                    v_d := abs(c_da - resize(e_offt, 14));
                    e_ua <= unsigned(v_d(11 downto 0));
                    v_d := abs(c_db - resize(e_offt, 14));
                    e_ub <= unsigned(v_d(11 downto 0));
                when 99 =>                     -- M6: pA0 = uA0 * SHl
                    m_acc <= (others => '0');
                    m_a <= resize(e_ua, 32); m_b <= resize(e_shl, 17);
                    n_x1 <= n_xh0 + signed(resize(s_xs, 14));
                when 100 =>
                    n_x2 <= n_x1 + signed(resize(s_xs, 14));
                when 117 =>
                    n_pa0 <= m_acc(20 downto 0);
                    -- M7: pB0 = uB0 * SB
                    m_acc <= (others => '0');
                    m_a <= resize(e_ub, 32); m_b <= resize(c_sb, 17);
                when 135 =>
                    n_pb0 <= m_acc(21 downto 0);
                    -- M8: vB' = |y - 513| * SVB
                    m_acc <= (others => '0');
                    m_a <= resize(e_av, 32); m_b <= resize(c_svb, 17);
                when 153 =>
                    n_vb <= m_acc(21 downto 6);
                    -- M9: ghost offset (1075 - vp) * SG
                    m_acc <= (others => '0');
                    if e_vp < 1075 then m_a <= resize(1075 - e_vp, 32); else m_a <= (others => '0'); end if;
                    m_b <= resize(c_sg, 17);
                when 171 =>
                    n_gofs <= m_acc(15 downto 0) + to_unsigned(65536 + C_PHB, 16);
                    if e_vp > 985 and s_quad = '0' then n_gh <= '1'; else n_gh <= '0'; end if;
                when 172 =>                    -- B duty terms: t2 = clamp(3 (av - 230) >> 4, 0, 19), t3 = min(av >> 4 + av >> 6, 26)
                    if e_av > 230 then
                        v_a := resize(e_av - 230, 11);
                        v_t2 := shift_left(resize(v_a, 13), 1) + resize(v_a, 13);
                        if v_t2(12 downto 4) > 19 then e_t2 <= to_unsigned(19, 5); else e_t2 <= v_t2(8 downto 4); end if;
                    else
                        e_t2 <= (others => '0');
                    end if;
                    v_t3 := shift_right(e_av, 4) + shift_right(e_av, 6);
                    if v_t3 > 26 then e_t3 <= to_unsigned(26, 5); else e_t3 <= v_t3(4 downto 0); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the two folded phase DDAs (the step direction is that of the destination pixel, from a compare two
    -- pixels ahead), the virtual x, and the line constants
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then              -- b_da / b_db already hold dir(x1), from the hblank compares
                b_pa <= n_pa0; b_pb <= n_pb0;
                b_xh <= n_xh0; b_x2 <= n_x2;
                b_px <= (others => '0');
                -- the engine's e_* registers hold the finished next-line values until its next kick
                l_tv <= e_tv; l_fl <= e_fl;
                if e_fl = '1' then l_sta <= c_staf; else l_sta <= c_sta; end if;
                l_stb <= c_stb;
                l_vb <= n_vb; l_avb <= e_av; l_t2 <= e_t2; l_t3 <= e_t3;
                l_gh <= n_gh; l_gofs <= n_gofs;
                if e_fl = '1' then l_kk <= to_signed(64, 8); else l_kk <= resize(c_kq, 8); end if;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_da = '1' then b_pa <= b_pa + resize(l_sta, 21); else b_pa <= b_pa - resize(l_sta, 21); end if;
                if b_db = '1' then b_pb <= b_pb + resize(l_stb, 22); else b_pb <= b_pb - resize(l_stb, 22); end if;
                b_da <= '0'; b_db <= '0';
                if (b_x2 >= c_fa1) xor (b_x2 >= C_CA2) xor (b_x2 >= c_fa3) then b_da <= '1'; end if;
                if (b_x2 >= c_fb1) xor (b_x2 >= C_CB2) xor (b_x2 >= c_fb3) then b_db <= '1'; end if;
                b_xh <= b_xh + signed(resize(s_xs, 14));
                b_x2 <= b_x2 + signed(resize(s_xs, 14));
            else                               -- hblank: the first step's directions, at the next line's x1
                b_x2 <= n_x1;
                b_da <= '0'; b_db <= '0';
                if (b_x2 >= c_fa1) xor (b_x2 >= C_CA2) xor (b_x2 >= c_fa3) then b_da <= '1'; end if;
                if (b_x2 >= c_fb1) xor (b_x2 >= C_CB2) xor (b_x2 >= c_fb3) then b_db <= '1'; end if;
            end if;
            s_lb_ra <= b_px + 1;                       -- the luma chain reads 1 px ahead (lumA meets stage 5)
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_tu : unsigned(9 downto 0);
        variable v_ka : signed(18 downto 0);
        variable v_ab : signed(13 downto 0);
        variable v_kb : signed(23 downto 0);
        variable v_fa : signed(12 downto 0);
        variable v_d  : unsigned(12 downto 0);
        variable v_w  : unsigned(12 downto 0);
        variable v_p  : signed(21 downto 0);
        variable v_r  : unsigned(5 downto 0);
        variable v_ix : signed(13 downto 0);
        variable v_l  : signed(17 downto 0);
        variable v_y, v_u, v_v : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: the folders, max / min; B max / min; distance to the B mirror; ghost phase
            v_tu := f_fold(b_pa);
            if v_tu >= l_tv then p2_mxa <= v_tu; p2_mna <= l_tv; else p2_mxa <= l_tv; p2_mna <= v_tu; end if;
            r_cos_a <= f_cidx(c_phi);                  -- the LFO's cosine (read in the zone)
            if b_pb(21 downto 6) >= l_vb then
                p2_mxb <= b_pb(21 downto 6); p2_mnb <= l_vb; p2_reg <= '1';
            else
                p2_mxb <= l_vb; p2_mnb <= b_pb(21 downto 6); p2_reg <= '0';
            end if;
            p2_dx <= b_xh - C_CB2;
            p2_gs <= b_pb(15 downto 0) + l_gofs;
            -- 3: k min partial products (k = lo 3 bits + signed hi 4 bits); |dx|; ghost test
            p3_ka <= p2_mna * unsigned(l_kk(2 downto 0));
            p3_kb <= signed('0' & p2_mna) * l_kk(7 downto 3);
            p3_kc <= p2_mnb * unsigned(c_kq(2 downto 0));
            p3_kd <= signed('0' & p2_mnb) * resize(c_kq(6 downto 3), 5);
            p3_mxa <= p2_mxa; p3_mxb <= p2_mxb; p3_reg <= p2_reg;
            v_ab := abs(p2_dx);
            p3_db <= unsigned(v_ab(12 downto 0));
            if p2_gs(15 downto 10) > 19 and p2_gs(15 downto 10) < 45 then p3_gok <= '1'; else p3_gok <= '0'; end if;
            -- 4: k min sums; the distance tests
            v_ka := resize(signed('0' & p3_ka), 19) + shift_left(resize(p3_kb, 19), 3);
            p4_kma <= resize(shift_right(v_ka, 6), 12);
            -- B uses the corner k on every line: (mnB k) >> 6 (Q10 rings)
            v_kb := resize(signed('0' & p3_kc), 24) + shift_left(resize(p3_kd, 24), 3);
            p4_kmb <= resize(shift_right(v_kb, 6), 17);
            p4_mxa <= p3_mxa; p4_mxb <= p3_mxb; p4_reg <= p3_reg; p4_gok <= p3_gok;
            if p3_db < 1120 then p4_d1 <= '1'; else p4_d1 <= '0'; end if;
            if p3_db < 320 then p4_d2 <= '1'; else p4_d2 <= '0'; end if;
            if p3_db < 1040 then p4_gd <= '1'; else p4_gd <= '0'; end if;
            if p3_db < 240 then p4_q <= "000";
            elsif p3_db < 390 then p4_q <= "001";
            elsif p3_db < 540 then p4_q <= "010";
            elsif p3_db < 690 then p4_q <= "011";
            else p4_q <= "100"; end if;
            if p3_db > 1120 then p4_dd <= p3_db - 1120; else p4_dd <= (others => '0'); end if;
            -- 5: F (A) and FB; the window sum; the duty
            v_fa := signed(resize(p4_mxa, 13)) + resize(p4_kma, 13);
            if v_fa < 0 then p5_fa <= (others => '0'); else p5_fa <= unsigned(v_fa(11 downto 0)); end if;
            p5_fb <= signed(resize(p4_mxb, 18)) + resize(p4_kmb, 18);
            v_w := resize(l_avb, 13) + shift_right(p4_dd, 2) - shift_right(p4_dd, 5);
            p5_ws <= v_w;
            if p4_reg = '1' then p5_wv <= resize(c_wu, 7); else p5_wv <= resize(c_wv, 7); end if;
            if p4_d1 = '1' and p4_d2 = '1' then
                if p4_reg = '1' then p5_wv <= resize(c_wu, 7) + l_t2 + l_t3; else p5_wv <= resize(c_wv, 7) + l_t2 + l_t3; end if;
            elsif p4_d1 = '1' then
                if p4_reg = '1' then p5_wv <= resize(c_wu, 7) + l_t2; else p5_wv <= resize(c_wv, 7) + l_t2; end if;
            end if;
            p5_q <= p4_q;
            p5_gh <= l_gh and p4_gd and p4_gok;
            -- 6: B ring phase (+ the luma bend, lumA << 3); the window test
            p6_fa <= p5_fa;
            p6_rp <= p5_fb + shift_left(resize(g_lum, 18), 3) + to_signed(C_PHB10, 18);
            if p5_ws < 365 then p6_win <= '1'; else p6_win <= '0'; end if;
            p6_wv <= p5_wv; p6_q <= p5_q; p6_gh <= p5_gh;
            -- 7: ring phase distance, ring >= 2
            p7_f <= p6_fa;
            v_r := unsigned(p6_rp(9 downto 4));
            if v_r >= 32 then p7_a2r <= resize(shift_left(resize(v_r - 32, 7), 1), 7);
            else p7_a2r <= resize(shift_left(resize(32 - v_r, 7), 1), 7); end if;
            if p6_rp(17 downto 10) >= 2 then p7_r2 <= '1'; else p7_r2 <= '0'; end if;
            p7_win <= p6_win; p7_wv <= p6_wv; p7_q <= p6_q; p7_gh <= p6_gh;
            -- 8: palette index terms (F * 320, or F * 112 on the floor); the key
            if l_fl = '1' then
                p8_ix <= resize(shift_left(resize(p7_f, 20), 7) - shift_left(resize(p7_f, 20), 4), 20);
            else
                p8_ix <= resize(shift_left(resize(p7_f, 20), 8) + shift_left(resize(p7_f, 20), 6), 20);
            end if;
            if s_pink = '1' and p7_win = '1' and p7_a2r < p7_wv then p8_key <= '1'; else p8_key <= '0'; end if;
            p8_bi(4 downto 2) <= p7_q;
            p8_bi(1) <= p7_r2;
            if p7_a2r >= p7_wv - shift_right(p7_wv, 2) then p8_bi(0) <= '1'; else p8_bi(0) <= '0'; end if;
            p8_gh <= p7_gh;
            -- 9: palette index (+ floor lift, + luma)
            v_ix := signed(resize(p8_ix(19 downto 10), 14)) + resize(g_lum, 14);
            if l_fl = '1' then v_ix := v_ix + 204; end if;
            if s_rev = '1' then p9_idx <= not unsigned(v_ix(8 downto 0)); else p9_idx <= unsigned(v_ix(8 downto 0)); end if;
            p9_bi <= p8_bi;
            p9_key <= p8_key; p9_gh <= p8_gh;
            -- 10: palette address
            r_pal_a <= s_pal & p9_idx;
            r_bc_a  <= "000" & p9_bi;
            p10_key <= p9_key; p10_gh <= p9_gh;
            -- 11: ROM data (r_pal_q)
            p11_key <= p10_key; p11_gh <= p10_gh;
            -- 12: fabric copies (palette, key colour)
            p12_q <= r_pal_q;
            p12_bc <= r_bc_q; p12_key <= p11_key; p12_gh <= p11_gh;
            -- 13: the key over the contour colour
            if p12_key = '1' then
                p13_y <= unsigned(p12_bc(31 downto 22)); p13_u <= unsigned(p12_bc(21 downto 12)); p13_v <= unsigned(p12_bc(11 downto 2));
            else
                p13_y <= unsigned(p12_q(31 downto 22)); p13_u <= unsigned(p12_q(21 downto 12)); p13_v <= unsigned(p12_q(11 downto 2));
            end if;
            p13_gh <= p12_gh;
            -- 14: ghost teeth (half way to grey)
            if p13_gh = '1' then
                v_y := resize(p13_y, 11) + C_GHY; v_u := resize(p13_u, 11) + C_GHU; v_v := resize(p13_v, 11) + C_GHV;
                p14_y <= v_y(10 downto 1); p14_u <= v_u(10 downto 1); p14_v <= v_v(10 downto 1);
            else
                p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            end if;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= C_BKY; s_u15 <= C_BKU; s_v15 <= C_BKV;
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);

            -- luma chain (line buffer read 1 px ahead): video lumA = l8 eg >> 8; Osc (l8 - 110) eg >> 5 (8x)
            g_l1 <= unsigned(s_lb_q);
            if s_src = '1' then
                v_l := signed(resize(g_l1, 18)) - 110;
                g_ta <= resize(v_l(8 downto 0) * signed('0' & c_eg(4 downto 0)), 15);
                g_tb <= resize(v_l(8 downto 0) * signed('0' & c_eg(8 downto 5)), 15);
            else
                g_ta <= signed(resize(g_l1 * c_eg(4 downto 0), 15));
                g_tb <= signed(resize(g_l1 * c_eg(8 downto 5), 15));
            end if;
            v_l := resize(g_ta, 18) + shift_left(resize(g_tb, 18), 5);
            if s_src = '1' then g_lum <= resize(shift_right(v_l, 5), 12);
            else g_lum <= resize(shift_right(v_l, 8), 12); end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the HD HDMI bar fix and the
    -- capture's horizontal softness), luma 13/16 and the blanking gate.  The running 8-sums snap back to
    -- black once 8 blank samples are in, so a glitched sample can't offset the colours for good.
    ---------------------------------------------------------------------------
    p_cr : process(clk)
    begin
        if rising_edge(clk) then
            cr(to_integer(cr_wa)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15) &
                                     std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17);
            cr_q  <= cr(to_integer(cr_ra));
            cr_q2 <= cr_q;
            cr_wa <= cr_wa + 1;
            cr_ra <= cr_wa - 5;          -- = the write address - 6: cr_q2 holds the sample written 8 clocks ago
        end if;
    end process p_cr;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 16 sums, stage 17 output
            if s_bah = "00000000" then
                s_bay <= C_BKY & "000"; s_bau <= C_BKU & "000"; s_bav <= C_BKV & "000";
            else
                s_bay <= s_bay + s_y15 - unsigned(cr_q2(59 downto 50));
                s_bau <= s_bau + s_u15 - unsigned(cr_q2(49 downto 40));
                s_bav <= s_bav + s_v15 - unsigned(cr_q2(39 downto 30));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= C_BKY; s_u17 <= C_BKU; s_v17 <= C_BKV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            if s_bbh = "00000000" then
                s_bby <= C_BKY & "000"; s_bbu <= C_BKU & "000"; s_bbv <= C_BKV & "000";
            else
                s_bby <= s_bby + s_y17 - unsigned(cr_q2(29 downto 20));
                s_bbu <= s_bbu + s_u17 - unsigned(cr_q2(19 downto 10));
                s_bbv <= s_bbv + s_v17 - unsigned(cr_q2(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
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

end architecture nesting;
