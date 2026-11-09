-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Plaid v0.1.0 -- Heirloom (phase 3, video clips), from the still WIN_20231204_23_13_02 at 5.5 s.
--
-- The still: a woven plaid.  A smooth colour field (yellow / white on the left through orange to red on the
-- right; warm at the top, a cool white / cyan / teal zone low in the middle) shows through a grid of cells:
-- open rows (between black weft bars) AND open columns (between warp gaps, 277.5 px apart).  Where a cell is
-- closed, the back layer shows: bands of crimson-to-hot-pink sawtooth stripes, curved in S-waves, each band
-- starting on a dead-straight left edge, black between the bands.  In the clip the weft bars slide down (two
-- families, 165 and 334 lines/s, overlapping and splitting), the colour field scrolls down with them, the warp
-- gaps pulse and the magenta bands drift sideways and breathe.
--
-- The patch, as fitted (plaid.py has the tables):
--   weft     bar(y) = slow table(y dens - o1(t)) OR fast table(y dens - o2(t))  (signed-distance ROMs + bias)
--   warp     gap where |frac((x - 35) / 277.5 dens + 1/2) - 1/2| < hw(y, t)  (a PWM'd pulse; the fringe is
--            the gap's outer 10 px: dark green over black, as the still's luma-wider-than-chroma edges)
--   bands    master phi(x) = a x + b x^2 + c (chirped, a second-order DDA; c and b on slow LFOs); a band is lit
--            while frac(phi) < duty(x), and the slave ramp restarts at every phi wrap (hard sync)
--   stripes  slave r = delta(y) + k (x - sync); colour = a 256-entry stripe palette over r
--   colour   W = y / 1100 + frac(phi + 0.05) / 16 + psi(t) + dye * luma; a 32 x 32 (X, W) colour grid: luma
--            bilinear (four parity ROMs), chroma linear in W with a period-8 X dither the output boxes average out
--
--   K1  Warp      warp gap width: none (0) .. the still (512, 29 %) .. 89 % (1023: the cells become slivers)
--   K2  Weft      weft bar width: -60 lines (no bars) .. the still (512) .. +60 lines (bars swallow the cells)
--   K3  Shuttle   bipolar: the weft (and the colour field with it) slides down (+) / up (-); 820 = the clip
--   K4  Bands     magenta band rate: x1/4 (one or two wide bands) .. the still (512) .. x4 (a dozen bands)
--   K5  Stripes   magenta stripe density: x0.35 (fat stripes) .. the still (512) .. x4 (fine stripes)
--   K6  Dye       how far the camera (or the Osc) shifts the colour field: 0 .. 1.8 colour cycles per white
--   S7  Layer     Cells over (the still) / Bands over (the weave inverted: stripes in the cells)
--   S8  Arcs      Waves (the still's S-curves) / Arcs (the stripes swept into steep arcs)
--   S9  Colours   Ember (the still) / Orchid (every chroma turned 90 degrees: pink, violet, green)
--   S10 Twill     Plain (the still) / Twill (the warp leans 45 degrees: parallelogram cells)
--   S11 Source    Video / Osc
--   P12 WEAVE     plaid density, warp and weft together: loose (0, 2.3 columns) .. the still (410) .. fine (x4)
--
-- Engine: vblank-zone sequencer (one serial multiplier, a shared glide, three slow LFOs), a per-line engine
-- kicked at each line start that computes the NEXT line (gap ripple, slave rate and its taper, the stripe phase
-- offset, via a quarter-wave sine ROM and one serial multiplier) in the line's horizontal blanking, beam DDAs,
-- a 21-stage pixel path.  Virtual 1080-line
-- coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (chroma through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes on block-RAM delay rings with the running-sum resync, luma [1 2 1] and 13/16)
-- before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.plaid_pkg.all;
use work.all;

architecture plaid of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (keep in step with imodel.py)
    constant C_C0    : unsigned(31 downto 0) := x"E0418937";   -- master phase offset 0.876, Q32
    constant C_CAMP  : integer := 5494538;         -- c LFO half range (0.3275 cycles), Q24
    constant C_CSTP  : integer := 50887;           -- c LFO step per field, Q24
    constant C_B40   : integer := 369435;          -- master chirp b, Q40 per px^2
    constant C_A32   : integer := 6165389;         -- master rate a, Q32 per px
    constant C_DDMAX : integer := 734003;          -- duty ceiling 0.70, Q20

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
    -- per-line steps (Q16 sine phase, Q24 colour W) for the timing's virtual lines per output line
    function f_dw(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(67799, 17);
            when 4          => return to_unsigned(34317, 17);
            when 8          => return to_unsigned(57195, 17);
            when 12         => return to_unsigned(28597, 17);
            when 5 | 6 | 14 => return to_unsigned(22878, 17);
            when 1 | 2 | 10 => return to_unsigned(30504, 17);
            when others     => return to_unsigned(15252, 17);
        end case;
    end function;
    function f_dpc(t : std_logic_vector(3 downto 0)) return unsigned is   -- 1080 lines (also Arcs' wave)
    begin
        case to_integer(unsigned(t)) is
            when 0 => return to_unsigned(269, 9);  when 4 => return to_unsigned(136, 9);
            when 8 => return to_unsigned(227, 9);  when 12 => return to_unsigned(113, 9);
            when 5 | 6 | 14 => return to_unsigned(91, 9);  when 1 | 2 | 10 => return to_unsigned(121, 9);
            when others => return to_unsigned(60, 9);
        end case;
    end function;
    function f_dpv(t : std_logic_vector(3 downto 0)) return unsigned is   -- 760 lines (Waves' stripe wave)
    begin
        case to_integer(unsigned(t)) is
            when 0 => return to_unsigned(383, 9);  when 4 => return to_unsigned(194, 9);
            when 8 => return to_unsigned(323, 9);  when 12 => return to_unsigned(161, 9);
            when 5 | 6 | 14 => return to_unsigned(129, 9);  when 1 | 2 | 10 => return to_unsigned(172, 9);
            when others => return to_unsigned(86, 9);
        end case;
    end function;
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is   -- Osc lattice step per line
    begin
        case to_integer(unsigned(t)) is
            when 0 => return to_unsigned(1075, 11);  when 4 => return to_unsigned(544, 11);
            when 8 => return to_unsigned(907, 11);   when 12 => return to_unsigned(453, 11);
            when 5 | 6 | 14 => return to_unsigned(363, 11);  when 1 | 2 | 10 => return to_unsigned(484, 11);
            when others => return to_unsigned(242, 11);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- line buffer: smoothed (luma >> 2) - 16, one entry per 2 pixels
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_ls    : unsigned(9 downto 0) := (others => '0');

    -- output delay rings (16 apart in a 256-deep block RAM; the read never meets the write address)
    type t_ra is array (0 to 255) of std_logic_vector(29 downto 0);
    type t_rb is array (0 to 255) of std_logic_vector(19 downto 0);
    signal ra : t_ra := (others => (others => '0'));
    signal rb : t_rb := (others => (others => '0'));
    signal s_ca : unsigned(7 downto 0) := (others => '0');
    signal s_ra_q : std_logic_vector(29 downto 0) := (others => '0');
    signal s_rb_q : std_logic_vector(19 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of ra : signal is true;
    attribute no_rw_check of rb : signal is true;

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(8 downto 0) := (others => '0');
    signal r_sn_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_ws_a, r_wf_a : unsigned(9 downto 0) := (others => '0');
    signal r_ws_q, r_wf_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_bk_a : unsigned(7 downto 0) := (others => '0');
    signal r_bk_q : std_logic_vector(23 downto 0) := (others => '0');
    signal r_yee_a, r_yeo_a, r_yoe_a, r_yoo_a : unsigned(7 downto 0) := (others => '0');
    signal r_yee_q, r_yeo_q, r_yoe_q, r_yoo_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_cwe_a, r_cwo_a : unsigned(8 downto 0) := (others => '0');
    signal r_cwe_q, r_cwo_q : std_logic_vector(15 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 320 x 270 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(242, 11);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_tidl : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ysr  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_sd   : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(410 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(820, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_lay, s_arc, s_col, s_twl, s_src : std_logic := '0';

    -- per-field values
    signal z_d12, z_d5 : signed(10 downto 0) := (others => '0');
    signal z_e12, z_e4, z_e5 : signed(10 downto 0) := (others => '0');
    signal z_m12, z_m4, z_m5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_dens : unsigned(14 downto 0) := to_unsigned(4096, 15);   -- weave density, Q12
    signal c_bnd  : unsigned(14 downto 0) := to_unsigned(4096, 15);   -- band rate, Q12
    signal c_st   : unsigned(14 downto 0) := to_unsigned(4096, 15);   -- stripe density, Q12
    signal m_cfc, m_cph : unsigned(23 downto 0) := (others => '0');   -- mode constants (column)
    signal m_g1   : unsigned(11 downto 0) := to_unsigned(4, 12);      -- f0's b factor, Q10
    signal m_g2   : unsigned(4 downto 0) := to_unsigned(4, 5);        -- cm's b factor (s_xs^2)
    signal m_x0v  : unsigned(7 downto 0) := (others => '0');          -- 240 (SD) / 0
    signal m_x0q  : unsigned(7 downto 0) := (others => '0');          -- 225 (SD) / 0
    signal m_ck   : unsigned(21 downto 0) := (others => '0');         -- slave rate constant
    signal z_bb   : unsigned(15 downto 0) := to_unsigned(4096, 16);   -- bnd^2, Q12
    signal z_a32  : unsigned(27 downto 0) := to_unsigned(C_A32, 28);  -- a bnd, Q32 per px
    signal z_b40  : unsigned(22 downto 0) := to_unsigned(C_B40, 23);  -- b(t) bnd^2, Q40 per px^2
    signal z_f0a  : unsigned(29 downto 0) := (others => '0');
    signal z_ph   : unsigned(33 downto 0) := (others => '0');
    signal z_pc   : unsigned(23 downto 0) := (others => '0');
    signal z_vsq  : unsigned(17 downto 0) := (others => '0');
    signal z_g6sq : unsigned(19 downto 0) := (others => '0');
    signal c_fc   : unsigned(23 downto 0) := to_unsigned(60457, 24);  -- column step per pixel, Q24
    signal c_cph0 : unsigned(23 downto 0) := (others => '0');
    signal c_fw   : unsigned(11 downto 0) := to_unsigned(148, 12);    -- fringe width, Q12 cycles
    signal c_hw0  : unsigned(11 downto 0) := to_unsigned(588, 12);    -- gap half width, Q12 cycles
    signal c_hwa  : unsigned(8 downto 0) := to_unsigned(64, 9);       -- gap ripple amplitude
    signal c_bias : signed(7 downto 0) := (others => '0');            -- weft width offset, lines
    signal c_phi0 : unsigned(33 downto 0) := (others => '0');         -- master phase at the line start, Q32
    signal c_f0   : unsigned(29 downto 0) := (others => '0');         -- master step at the line start, Q32
    signal c_cm   : unsigned(19 downto 0) := (others => '0');         -- master step increment, Q32
    signal c_dd0  : unsigned(19 downto 0) := to_unsigned(308804, 20); -- duty at the line start, Q20
    signal c_dds  : unsigned(9 downto 0) := to_unsigned(242, 10);     -- duty step per pixel, Q20
    signal c_ks   : unsigned(23 downto 0) := (others => '0');         -- slave rate per pixel, Q24
    signal c_da   : unsigned(25 downto 0) := to_unsigned(3690988, 26);-- stripe phase offset amplitude, Q24
    signal c_gx0  : unsigned(21 downto 0) := (others => '0');         -- colour X at the line start, Q16 (signed)
    signal c_gxs  : unsigned(11 downto 0) := to_unsigned(1092, 12);   -- colour X step, Q16
    signal c_dtw  : unsigned(23 downto 0) := (others => '0');         -- twill: warp phase step per line, Q24
    signal c_du   : unsigned(19 downto 0) := to_unsigned(4096, 20);   -- weft step per line, Q12 lines
    signal c_dw   : unsigned(16 downto 0) := to_unsigned(15252, 17);  -- colour W step per line, Q24
    signal c_dpc, c_dpd : unsigned(8 downto 0) := (others => '0');   -- sine phase steps per line, Q16
    signal c_pdb  : unsigned(15 downto 0) := (others => '0');
    signal c_dye  : unsigned(8 downto 0) := to_unsigned(123, 9);      -- dye gain, Q16 W per luma step
    signal c_o1   : unsigned(22 downto 0) := (others => '0');         -- slow weft offset, Q12 lines
    signal c_o2   : unsigned(23 downto 0) := (others => '0');         -- fast weft offset, Q12 lines
    signal c_w0   : unsigned(23 downto 0) := (others => '0');         -- colour scroll, Q24
    signal c_pc0  : unsigned(15 downto 0) := (others => '0');         -- gap ripple time phase
    signal c_pd0  : unsigned(15 downto 0) := (others => '0');         -- stripe wave time phase
    signal c_s1   : unsigned(15 downto 0) := (others => '0');         -- slow weft scroll step per field
    signal lf_co  : signed(23 downto 0) := (others => '0');
    signal lf_cd  : std_logic := '1';                                 -- 1 = falling

    -- field serial multiplier
    signal f_a   : unsigned(37 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(37 downto 0) := (others => '0');
    signal z_fa  : unsigned(37 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (runs in the horizontal blanking after each line, and once in the zone for line 0)
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk, e_go : std_logic := '0';
    signal e_u1   : unsigned(22 downto 0) := (others => '0');
    signal e_u2   : unsigned(23 downto 0) := (others => '0');
    signal e_wl   : unsigned(23 downto 0) := (others => '0');
    signal e_pc, e_pd : unsigned(15 downto 0) := (others => '0');
    signal e_tw   : unsigned(23 downto 0) := (others => '0');
    signal e_sq   : unsigned(15 downto 0) := (others => '0');
    signal e_sc, e_sd : unsigned(7 downto 0) := (others => '0');
    signal e_nc, e_nd : std_logic := '0';
    signal e_hw   : signed(13 downto 0) := (others => '0');
    signal m_a    : unsigned(33 downto 0) := (others => '0');
    signal m_b    : unsigned(7 downto 0) := (others => '0');
    signal m_acc  : unsigned(33 downto 0) := (others => '0');

    -- beam
    signal b_cph  : unsigned(23 downto 0) := (others => '0');
    signal b_phi  : unsigned(32 downto 0) := (others => '0');
    signal b_f    : unsigned(29 downto 0) := (others => '0');
    signal b_p32d : std_logic := '0';
    signal b_phid : unsigned(11 downto 0) := (others => '0');
    signal b_dd   : unsigned(19 downto 0) := (others => '0');
    signal b_r    : unsigned(23 downto 0) := (others => '0');
    signal b_gx   : unsigned(21 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_ln   : unsigned(1 downto 0) := (others => '0');
    signal l_u1   : unsigned(22 downto 0) := (others => '0');
    signal l_u2   : unsigned(23 downto 0) := (others => '0');
    signal l_wl   : unsigned(15 downto 0) := (others => '0');
    signal l_hw, l_hwf : unsigned(11 downto 0) := (others => '0');
    signal l_cph0 : unsigned(23 downto 0) := (others => '0');
    signal l_dl   : unsigned(23 downto 0) := (others => '0');

    -- pixel path
    type t_u4a is array (natural range <>) of unsigned(3 downto 0);
    type t_u5a is array (natural range <>) of unsigned(4 downto 0);
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    type t_flv is array (natural range <>) of std_logic_vector(3 downto 0);
    signal p2_dist : unsigned(10 downto 0) := (others => '0');
    signal p2_mh   : std_logic := '0';
    signal p2_sm   : unsigned(11 downto 0) := (others => '0');
    signal p2_gi, p2_gc : unsigned(4 downto 0) := (others => '0');
    signal p2_ga   : unsigned(3 downto 0) := (others => '0');
    signal p2_lm   : unsigned(7 downto 0) := (others => '0');
    signal p3_gap, p3_frg, p3_mh : std_logic := '0';
    signal p3_w, p4_w, p5_w : unsigned(15 downto 0) := (others => '0');
    signal p3_ea, p3_eb : unsigned(12 downto 0) := (others => '0');
    signal p3_ws, p3_wf : signed(7 downto 0) := (others => '0');
    signal p4_et   : unsigned(16 downto 0) := (others => '0');
    signal s_gi, s_gc : t_u5a(3 to 5) := (others => (others => '0'));
    signal s_ga    : t_u4a(3 to 12) := (others => (others => '0'));
    signal s_rp    : t_u8a(2 to 11) := (others => (others => '0'));
    signal s_fd    : t_flv(4 to 14) := (others => (others => '0'));    -- {bar, mh, gap, frg}
    signal s_aw    : t_u4a(6 to 9) := (others => (others => '0'));
    signal p6_wp, p6_xp, p7_wp, p7_xp, p8_xp, p9_xp, p10_xp, p11_xp : std_logic := '0';
    signal p8_y0e, p8_y1e, p8_y0o, p8_y1o : std_logic_vector(7 downto 0) := (others => '0');
    signal p8_c0, p8_c1 : std_logic_vector(15 downto 0) := (others => '0');
    type t_s9a is array (0 to 3) of signed(8 downto 0);
    type t_s13a is array (0 to 3) of signed(12 downto 0);
    type t_s14a is array (0 to 3) of signed(13 downto 0);
    type t_u8c is array (0 to 3) of unsigned(7 downto 0);
    signal p9_d    : t_s9a := (others => (others => '0'));
    signal p9_c, p10_c : t_u8c := (others => (others => '0'));
    signal p10_p   : t_s13a := (others => (others => '0'));
    signal p11_v   : t_s14a := (others => (others => '0'));
    signal p12_yl, p12_dx, p12_u, p12_v, p13_yl, p13_u, p13_v : signed(13 downto 0) := (others => '0');
    signal p13_px  : signed(17 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p14_bk  : std_logic_vector(23 downto 0) := (others => '0');
    signal s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_qa  : std_logic_vector(29 downto 0) := (others => '0');   -- ring A tap: the stage-15 sample 8 back
    signal s_qb  : std_logic_vector(19 downto 0) := (others => '0');
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_la, s_lb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y17, s_y18, s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), delay rings, ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_ring : process(clk)
        variable v_wa : std_logic_vector(29 downto 0);
        variable v_wb : std_logic_vector(19 downto 0);
    begin
        if rising_edge(clk) then
            v_wa := std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15);
            v_wb := std_logic_vector(s_u17) & std_logic_vector(s_v17);
            s_ca <= s_ca + 1;
            s_ra_q <= ra(to_integer(s_ca + 250));          -- = write address - 6
            ra(to_integer(s_ca)) <= v_wa;
            s_rb_q <= rb(to_integer(s_ca + 250));
            rb(to_integer(s_ca)) <= v_wb;
        end if;
    end process p_ring;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_PL_EXP2(to_integer(r_ex_a));
            r_sn_q <= C_PL_SIN(to_integer(r_sn_a));
            r_ws_q <= C_PL_WS(to_integer(r_ws_a));
            r_wf_q <= C_PL_WF(to_integer(r_wf_a));
            r_bk_q <= C_PL_BACK(to_integer(r_bk_a));
            r_yee_q <= C_PL_YEE(to_integer(r_yee_a));
            r_yeo_q <= C_PL_YEO(to_integer(r_yeo_a));
            r_yoe_q <= C_PL_YOE(to_integer(r_yoe_a));
            r_yoo_q <= C_PL_YOO(to_integer(r_yoo_a));
            r_cwe_q <= C_PL_CWE(to_integer(r_cwe_a));
            r_cwo_q <= C_PL_CWO(to_integer(r_cwo_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_g  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ysr <= f_vstep_y(s_tid);
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
            -- shuttle: bipolar, +-8 deadband, |k3 - 512| - 8
            s_k3 <= unsigned(registers_in(2));
            v_s := signed(resize(s_k3, 11)) - 512;
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
                s_lay <= registers_in(6)(0);
                s_arc <= registers_in(6)(1);
                s_col <= registers_in(6)(2);
                s_twl <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                s_ys  <= s_ysr;
                s_tidl <= s_tid;
                if s_xsr = 4 then s_sd <= '1'; s_oxs <= to_unsigned(408, 9);
                elsif s_xsr = 3 then s_sd <= '0'; s_oxs <= to_unsigned(306, 9);
                else s_sd <= '0'; s_oxs <= to_unsigned(204, 9); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tidl);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
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
                            when 2      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 48 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at load + 18).  The slot number is decoded at 46 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 47 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 46 => z_sel <= to_unsigned(0, 5); z_pre <= '1';
                    when 70 => z_sel <= to_unsigned(1, 5); z_pre <= '1';
                    when 94 => z_sel <= to_unsigned(2, 5); z_pre <= '1';
                    when 118 => z_sel <= to_unsigned(3, 5); z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(4, 5); z_pre <= '1';
                    when 166 => z_sel <= to_unsigned(5, 5); z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(6, 5); z_pre <= '1';
                    when 214 => z_sel <= to_unsigned(7, 5); z_pre <= '1';
                    when 238 => z_sel <= to_unsigned(8, 5); z_pre <= '1';
                    when 262 => z_sel <= to_unsigned(9, 5); z_pre <= '1';
                    when 286 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 310 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 334 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 358 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 382 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 406 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 430 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 454 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;

            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(m_cfc, 38);               z_fb <= resize(c_dens, 16);   -- column step
                    when 1  => z_fa <= resize(m_cph, 38);               z_fb <= resize(c_dens, 16);   -- column offset
                    when 2  => z_fa <= to_unsigned(148, 38);            z_fb <= resize(c_dens, 16);   -- fringe
                    when 3  => z_fa <= to_unsigned(C_A32, 38);          z_fb <= resize(c_bnd, 16);    -- a bnd
                    when 4  => z_fa <= resize(c_bnd, 38);               z_fb <= resize(c_bnd, 16);    -- bnd^2
                    when 5  => z_fa <= to_unsigned(C_B40, 38);          z_fb <= z_bb;                 -- b bnd^2
                    when 6  => z_fa <= resize(z_a32, 38);               z_fb <= resize(s_xs, 16);     -- a dx
                    when 7  => z_fa <= resize(z_b40, 38);               z_fb <= resize(m_g1, 16);     -- f0's b part
                    when 8  => z_fa <= resize(z_b40, 38);               z_fb <= resize(m_g2, 16);     -- cm
                    when 9  => z_fa <= resize(z_a32, 38);               z_fb <= resize(m_x0v, 16);    -- a x0
                    when 10 => z_fa <= resize(z_b40, 38);               z_fb <= resize(m_x0q, 16);    -- b x0^2
                    when 11 => z_fa <= resize(m_ck, 38);                z_fb <= resize(c_st, 16);     -- slave rate
                    when 12 => z_fa <= resize(s_g6(12 downto 3), 38);   z_fb <= resize(s_g6(12 downto 3), 16);  -- k6^2
                    when 13 => z_fa <= resize(z_g6sq, 38);              z_fb <= to_unsigned(494, 16); -- dye gain
                    when 14 => z_fa <= resize(s_vm, 38);                z_fb <= resize(s_vm, 16);     -- speed^2
                    when 15 => z_fa <= resize(z_vsq, 38);               z_fb <= to_unsigned(8203, 16);   -- slow weft step
                    when 16 => z_fa <= resize(c_dens, 38);              z_fb <= resize(s_ys, 16);        -- weft per line
                    when others => z_fa <= resize(c_du, 38);            z_fb <= to_unsigned(60457, 16);  -- twill per line
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(36 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- LFOs and time phases (last field's steps)
                    when 4 =>
                        if lf_cd = '1' then
                            if lf_co < -C_CAMP + C_CSTP then lf_cd <= '0'; end if;
                            lf_co <= lf_co - C_CSTP;
                        else
                            if lf_co > C_CAMP - C_CSTP then lf_cd <= '1'; end if;
                            lf_co <= lf_co + C_CSTP;
                        end if;
                    when 5 =>
                        -- the fast weft at 2x, the colour field at 3.75x (in its own units: 165 lines/s each)
                        if s_vneg = '1' then
                            c_o1 <= c_o1 + resize(c_s1, 23); c_o2 <= c_o2 + shift_left(resize(c_s1, 24), 1);
                            c_w0 <= c_w0 + shift_left(resize(c_s1, 24), 2) - resize(c_s1(15 downto 2), 24);
                        else
                            c_o1 <= c_o1 - resize(c_s1, 23); c_o2 <= c_o2 - shift_left(resize(c_s1, 24), 1);
                            c_w0 <= c_w0 - shift_left(resize(c_s1, 24), 2) + resize(c_s1(15 downto 2), 24);
                        end if;
                        c_pc0 <= c_pc0 + 1437;
                        c_pd0 <= c_pd0 - 352;
                    when 7 =>
                        z_ph <= resize(C_C0, 34) + unsigned(resize(lf_co & x"00", 34));
                    -- exponents (Q8 octaves)
                    when 26 =>
                        z_d12 <= signed(resize(s_gp(12 downto 3), 11)) - 410;
                        z_d5  <= signed(resize(s_g5(12 downto 3), 11)) - 512;
                        z_e4  <= signed(resize(s_g4(12 downto 3), 11)) - 512;
                    when 27 =>
                        if z_d12(10) = '1' then z_e12 <= z_d12 - shift_right(z_d12, 6);
                        else z_e12 <= z_d12 - shift_right(z_d12, 3) - shift_right(z_d12, 5); end if;
                        if z_d5(10) = '1' then z_e5 <= z_d5 - shift_right(z_d5, 2);
                        else z_e5 <= z_d5; end if;
                    when 28 => r_ex_a <= unsigned(z_e12(7 downto 0));
                    when 31 => z_m12 <= unsigned(r_ex_q); r_ex_a <= unsigned(z_e4(7 downto 0));
                    when 34 => z_m4 <= unsigned(r_ex_q); r_ex_a <= unsigned(z_e5(7 downto 0));
                    when 37 => z_m5 <= unsigned(r_ex_q);
                    when 38 =>
                        -- value Q12 = mant (Q15) >> (3 - int), int = e >> 8 in -2 .. 2
                        case to_integer(z_e12(10 downto 8)) is
                            when 6      => c_dens <= resize(shift_right(z_m12, 5), 15);
                            when 7      => c_dens <= resize(shift_right(z_m12, 4), 15);
                            when 0      => c_dens <= resize(shift_right(z_m12, 3), 15);
                            when 1      => c_dens <= resize(shift_right(z_m12, 2), 15);
                            when others => c_dens <= resize(shift_right(z_m12, 1), 15);
                        end case;
                        case to_integer(z_e4(10 downto 8)) is
                            when 6      => c_bnd <= resize(shift_right(z_m4, 5), 15);
                            when 7      => c_bnd <= resize(shift_right(z_m4, 4), 15);
                            when 0      => c_bnd <= resize(shift_right(z_m4, 3), 15);
                            when others => c_bnd <= resize(shift_right(z_m4, 2), 15);
                        end case;
                        case to_integer(z_e5(10 downto 8)) is
                            when 6      => c_st <= resize(shift_right(z_m5, 5), 15);
                            when 7      => c_st <= resize(shift_right(z_m5, 4), 15);
                            when 0      => c_st <= resize(shift_right(z_m5, 3), 15);
                            when others => c_st <= resize(shift_right(z_m5, 2), 15);
                        end case;
                    when 39 =>
                        -- warp half width (K1) and weft bias (K2)
                        v_g := s_g1(12 downto 3);
                        if v_g <= 512 then
                            c_hw0 <= resize(v_g, 12) + resize(v_g(9 downto 3), 12) + resize(v_g(9 downto 6), 12) + resize(v_g(9 downto 7), 12);
                        else
                            v_m := v_g - 512;
                            c_hw0 <= to_unsigned(588, 12) + resize(v_m & '0', 12) + resize(v_m(9 downto 2), 12) + resize(v_m(9 downto 3), 12);
                        end if;
                        v_d := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        c_bias <= resize(shift_right(v_d, 3) - shift_right(v_d, 7), 8);
                    when 40 =>
                        c_hwa <= resize(shift_right(c_hw0, 3), 9) - resize(shift_right(c_hw0, 6), 9);
                    -- slot results
                    when 66  => c_fc <= f_acc(35 downto 12);
                    when 90  => z_pc <= f_acc(35 downto 12);
                    when 91  =>
                        if s_sd = '1' then c_cph0 <= to_unsigned(2 ** 23, 24) + z_pc;
                        else c_cph0 <= to_unsigned(2 ** 23, 24) - z_pc; end if;
                    when 114 => c_fw <= f_acc(23 downto 12);
                    when 138 => z_a32 <= resize(f_acc(37 downto 12), 28);
                    when 162 => z_bb <= f_acc(27 downto 12);
                    when 186 => z_b40 <= f_acc(34 downto 12);
                    when 210 => z_f0a <= f_acc(30 downto 1);
                    when 234 => c_f0 <= z_f0a + resize(f_acc(37 downto 10), 30);
                    when 258 => c_cm <= f_acc(28 downto 9);
                    when 282 => z_ph <= z_ph + f_acc(33 downto 0);
                    when 306 => c_phi0 <= z_ph + f_acc(33 downto 0);
                    when 330 => c_ks <= resize(f_acc(35 downto 16), 24);        -- k st >> 12, Q28 -> Q24
                    when 354 => z_g6sq <= f_acc(19 downto 0);
                    when 378 => c_dye <= f_acc(28 downto 20);
                    when 402 => z_vsq <= f_acc(17 downto 0);
                    when 426 => c_s1 <= f_acc(31 downto 16);
                    when 450 => c_du <= f_acc(27 downto 8);
                    when 474 =>
                        if s_twl = '1' then c_dtw <= f_acc(35 downto 12); else c_dtw <= (others => '0'); end if;
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
    -- Mode and switch constants (static between fields)
    ---------------------------------------------------------------------------
    m_cfc <= to_unsigned(120914, 24) when s_sd = '1' else to_unsigned(90686, 24) when s_xs = 3 else to_unsigned(60457, 24);
    m_cph <= to_unsigned(12393685, 24) when s_sd = '1' else to_unsigned(2115995, 24);
    m_g1  <= to_unsigned(3856, 12) when s_sd = '1' else to_unsigned(9, 12) when s_xs = 3 else to_unsigned(4, 12);
    m_g2  <= to_unsigned(16, 5) when s_sd = '1' else to_unsigned(9, 5) when s_xs = 3 else to_unsigned(4, 5);
    m_x0v <= to_unsigned(240, 8) when s_sd = '1' else to_unsigned(0, 8);
    m_x0q <= to_unsigned(225, 8) when s_sd = '1' else to_unsigned(0, 8);
    m_ck  <= to_unsigned(2131050, 22) when s_sd = '1' else to_unsigned(1698960, 22) when s_xs = 3 else to_unsigned(1132640, 22);
    c_dd0 <= to_unsigned(366685, 20) when s_sd = '1' else to_unsigned(308804, 20);
    c_gx0 <= to_unsigned(229320, 22) when s_sd = '1' else to_unsigned(2 ** 22 - 32768, 22);
    c_dds <= to_unsigned(484, 10) when s_sd = '1' else to_unsigned(363, 10) when s_xs = 3 else to_unsigned(242, 10);
    c_gxs <= to_unsigned(2184, 12) when s_sd = '1' else to_unsigned(1638, 12) when s_xs = 3 else to_unsigned(1092, 12);
    c_dw  <= f_dw(s_tidl);
    c_dpc <= f_dpc(s_tidl);
    c_dpd <= f_dpc(s_tidl) when s_arc = '1' else f_dpv(s_tidl);
    c_pdb <= to_unsigned(49152, 16) when s_arc = '1' else to_unsigned(0, 16);
    c_da  <= to_unsigned(33554432, 26) when s_arc = '1' else to_unsigned(3690988, 26);

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
    ---------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_avd <= g_avid;
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
    -- write of (luma >> 2) - 16, smoothed along the line (1/4 per pixel)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(10 downto 0);
        variable v_n : signed(10 downto 0);
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
            if s_src = '1' then
                v_l := s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then v_l := v_l - 16; else v_l := (others => '0'); end if;
            end if;
            -- smoothing (Q2): ls += (l - ls) / 4
            v_d := signed(resize(v_l & "00", 11)) - signed(resize(s_ls, 11));
            v_n := signed(resize(s_ls, 11)) + shift_right(v_d, 2);
            s_ls <= unsigned(v_n(9 downto 0));
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol(10 downto 1);
            s_lb_wd <= std_logic_vector(s_ls(9 downto 2));
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked when the generated avid falls (and once in the zone
    -- for line 0), it computes the NEXT line in the horizontal blanking (~32
    -- clocks): two sines (gap ripple, the stripe phase offset) from a
    -- quarter-wave ROM, two products on one serial multiplier (8-bit b operand,
    -- result at load + 9), written straight into the line registers (the last
    -- pixels of the line have left stage 3 by then).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_i : unsigned(8 downto 0);
        variable v_h : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 800 then e_zk <= '1'; end if;
            e_go <= g_avd and not g_avid;

            if e_zk = '1' then
                e_u1 <= c_o1;
                e_u2 <= c_o2;
                e_wl <= c_w0;
                e_pc <= c_pc0;
                e_pd <= c_pd0 + c_pdb;
                e_tw <= c_cph0;
                e_lc <= (others => '0');
            elsif e_go = '1' then
                e_u1 <= e_u1 + resize(c_du, 23);
                e_u2 <= e_u2 + resize(c_du, 24);
                e_wl <= e_wl + resize(c_dw, 24);
                e_pc <= e_pc + resize(c_dpc, 16);
                e_pd <= e_pd + resize(c_dpd, 16);
                e_tw <= e_tw + c_dtw;
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(32 downto 0) & '0';
            m_b <= '0' & m_b(7 downto 1);

            case to_integer(e_lc) is
                -- quarter-wave sine: phase e_sq (16-bit cycle) -> |sin| (8 bit) and sign
                when 1  => e_sq <= e_pc;
                when 5  => e_sq <= e_pd;
                when 2 | 6 =>
                    v_i := e_sq(13 downto 5);
                    if e_sq(14) = '1' then v_i := not v_i; end if;
                    r_sn_a <= v_i;
                when 4  => e_sc <= unsigned(r_sn_q); e_nc <= e_sq(15);
                when 8  => e_sd <= unsigned(r_sn_q); e_nd <= e_sq(15);
                -- gap ripple: hw = hw0 +- (hwa s) >> 8
                when 9  => m_a <= resize(c_hwa, 34); m_b <= e_sc; m_acc <= (others => '0');
                when 18 =>
                    if e_nc = '1' then
                        e_hw <= signed(resize(c_hw0, 14)) - signed(resize(m_acc(16 downto 8), 14));
                    else
                        e_hw <= signed(resize(c_hw0, 14)) + signed(resize(m_acc(16 downto 8), 14));
                    end if;
                    -- stripe phase offset: dl = +- (dA s) >> 8
                    m_a <= resize(c_da, 34); m_b <= e_sd; m_acc <= (others => '0');
                when 19 =>
                    if e_hw < 0 then l_hw <= (others => '0');
                    elsif e_hw > 2047 then l_hw <= to_unsigned(2047, 12);
                    else l_hw <= unsigned(e_hw(11 downto 0)); end if;
                    v_h := e_hw - signed(resize(c_fw, 14));
                    if v_h < 0 then l_hwf <= (others => '0');
                    elsif v_h > 2047 then l_hwf <= to_unsigned(2047, 12);
                    else l_hwf <= unsigned(v_h(11 downto 0)); end if;
                when 27 =>
                    if e_nd = '1' then l_dl <= to_unsigned(0, 24) - m_acc(31 downto 8);
                    else l_dl <= m_acc(31 downto 8); end if;
                    l_u1 <= e_u1; l_u2 <= e_u2; l_wl <= e_wl(23 downto 8); l_cph0 <= e_tw;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the line's DDAs (warp phase, master phase and its chirp, duty,
    -- slave ramp with hard sync, colour X); the luma read is issued with them
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_cph  <= l_cph0;
                b_phi  <= c_phi0(32 downto 0);
                b_f    <= c_f0;
                b_p32d <= c_phi0(32);
                b_phid <= c_phi0(31 downto 20);
                b_dd   <= c_dd0;
                b_r    <= l_dl;
                b_gx   <= c_gx0;
                b_px   <= (others => '0');
                b_ln   <= b_ln + 1;
                s_lb_ra <= (others => '0');
            elsif g_avid = '1' then
                v_n := b_px + 1;
                b_px  <= v_n;
                s_lb_ra <= v_n(10 downto 1);
                b_cph <= b_cph + c_fc;
                b_phi <= b_phi + resize(b_f, 33);
                b_f   <= b_f + resize(c_cm, 30);
                b_p32d <= b_phi(32);
                b_phid <= b_phi(31 downto 20);
                if (b_phi(32) xor b_p32d) = '1' then
                    b_r <= l_dl;
                else
                    b_r <= b_r + c_ks;
                end if;
                if b_dd < C_DDMAX then b_dd <= b_dd + resize(c_dds, 20); end if;
                b_gx <= b_gx + resize(c_gxs, 22);
            end if;
            if s_zone = '1' then b_ln <= (others => '0'); end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_f12 : unsigned(11 downto 0);
        variable v_b1, v_b2 : signed(8 downto 0);
        variable v_iw  : unsigned(4 downto 0);
        variable v_ix  : unsigned(4 downto 0);
        variable v_e, v_o : std_logic_vector(7 downto 0);
        variable v_ce, v_co : std_logic_vector(15 downto 0);
        variable v_l, v_r : signed(13 downto 0);
        variable v_v   : signed(17 downto 0);
        variable v_c   : unsigned(9 downto 0);
        variable v_front : std_logic;
        variable v_y, v_u, v_vv : unsigned(9 downto 0);
        variable v_th  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: warp distance |frac - 1/2| (Q12), master high (the previous pixel's phase, so the band edge
            --    meets the slave reset), seam, colour X (bilinear for luma; dithered for chroma, period 8),
            --    weft ROM addresses, luma
            v_f12 := b_cph(23 downto 12);
            if v_f12(11) = '1' then p2_dist <= v_f12(10 downto 0); else p2_dist <= not v_f12(10 downto 0); end if;
            if b_phid < b_dd(19 downto 8) then p2_mh <= '1'; else p2_mh <= '0'; end if;
            p2_sm <= b_phi(31 downto 20) + 205;
            case to_integer(b_px(2 downto 0)) is
                when 0 => v_th := to_unsigned(0, 4);  when 1 => v_th := to_unsigned(8, 4);
                when 2 => v_th := to_unsigned(4, 4);  when 3 => v_th := to_unsigned(12, 4);
                when 4 => v_th := to_unsigned(2, 4);  when 5 => v_th := to_unsigned(10, 4);
                when 6 => v_th := to_unsigned(6, 4);  when others => v_th := to_unsigned(14, 4);
            end case;
            if b_gx(21) = '1' then
                p2_gi <= (others => '0'); p2_ga <= (others => '0'); p2_gc <= (others => '0');
            elsif b_gx(20 downto 16) = "11111" then
                p2_gi <= to_unsigned(30, 5); p2_ga <= to_unsigned(15, 4); p2_gc <= to_unsigned(31, 5);
            else
                p2_gi <= b_gx(20 downto 16); p2_ga <= b_gx(15 downto 12);
                if b_gx(15 downto 12) > v_th then p2_gc <= b_gx(20 downto 16) + 1; else p2_gc <= b_gx(20 downto 16); end if;
            end if;
            r_ws_a <= l_u1(22 downto 13);
            r_wf_a <= l_u2(23 downto 14);
            s_rp(2) <= b_r(23 downto 16);
            p2_lm <= unsigned(s_lb_q);
            -- 3: gap / fringe tests, W = line + seam, dye partials (luma nibbles x gain)
            if p2_dist < l_hw then p3_gap <= '1'; else p3_gap <= '0'; end if;
            if p2_dist >= l_hwf then p3_frg <= '1'; else p3_frg <= '0'; end if;
            p3_mh <= p2_mh;
            p3_w <= l_wl + resize(p2_sm, 16);
            p3_ea <= resize(p2_lm(3 downto 0) * c_dye, 13);
            p3_eb <= resize(p2_lm(7 downto 4) * c_dye, 13);
            s_gi(3) <= p2_gi; s_ga(3) <= p2_ga; s_gc(3) <= p2_gc;
            p3_ws <= signed(r_ws_q); p3_wf <= signed(r_wf_q);     -- fabric copies (constant along the line)
            -- 4: weft bar = slow OR fast (distance + bias > 0); dye sum
            v_b1 := resize(p3_ws, 9) + resize(c_bias, 9);
            v_b2 := resize(p3_wf, 9) + resize(c_bias, 9);
            if v_b1 > 0 or v_b2 > 0 then s_fd(4) <= '1' & p3_mh & p3_gap & p3_frg; else s_fd(4) <= '0' & p3_mh & p3_gap & p3_frg; end if;
            p4_et <= resize(p3_ea, 17) + shift_left(resize(p3_eb, 17), 4);
            p4_w <= p3_w;
            -- 5: W + dye (mod one colour cycle)
            p5_w <= p4_w + p4_et(15 downto 0);
            for i in 5 to 14 loop s_fd(i) <= s_fd(i - 1); end loop;
            for i in 4 to 5 loop s_gi(i) <= s_gi(i - 1); s_gc(i) <= s_gc(i - 1); end loop;
            for i in 4 to 12 loop s_ga(i) <= s_ga(i - 1); end loop;
            for i in 3 to 11 loop s_rp(i) <= s_rp(i - 1); end loop;
            -- 6: colour grid addresses (luma: X / W parity ROMs; chroma: W parity ROMs at the dithered X)
            v_iw := p5_w(15 downto 11);
            v_ix := s_gi(5);
            r_yee_a <= resize(shift_right(v_ix + 1, 1), 4) & resize(shift_right(v_iw + 1, 1), 4);
            r_yeo_a <= resize(shift_right(v_ix + 1, 1), 4) & v_iw(4 downto 1);
            r_yoe_a <= v_ix(4 downto 1) & resize(shift_right(v_iw + 1, 1), 4);
            r_yoo_a <= v_ix(4 downto 1) & v_iw(4 downto 1);
            r_cwe_a <= s_gc(5) & resize(shift_right(v_iw + 1, 1), 4);
            r_cwo_a <= s_gc(5) & v_iw(4 downto 1);
            p6_wp <= v_iw(0); p6_xp <= v_ix(0);
            s_aw(6) <= p5_w(10 downto 7);
            for i in 7 to 9 loop s_aw(i) <= s_aw(i - 1); end loop;
            -- 7: ROMs (EBR) -> 8: W-parity swap straight off the EBR outputs (no arithmetic)
            p7_wp <= p6_wp; p7_xp <= p6_xp;
            if p7_wp = '0' then
                p8_y0e <= r_yee_q; p8_y1e <= r_yeo_q; p8_y0o <= r_yoe_q; p8_y1o <= r_yoo_q; p8_c0 <= r_cwe_q; p8_c1 <= r_cwo_q;
            else
                p8_y0e <= r_yeo_q; p8_y1e <= r_yee_q; p8_y0o <= r_yoo_q; p8_y1o <= r_yoe_q; p8_c0 <= r_cwo_q; p8_c1 <= r_cwe_q;
            end if;
            p8_xp <= p7_xp;
            -- 9: W differences (luma X-even and X-odd columns, U, V)
            p9_d(0) <= signed(resize(unsigned(p8_y1e), 9)) - signed(resize(unsigned(p8_y0e), 9));
            p9_d(1) <= signed(resize(unsigned(p8_y1o), 9)) - signed(resize(unsigned(p8_y0o), 9));
            p9_d(2) <= signed(resize(unsigned(p8_c1(15 downto 8)), 9)) - signed(resize(unsigned(p8_c0(15 downto 8)), 9));
            p9_d(3) <= signed(resize(unsigned(p8_c1(7 downto 0)), 9)) - signed(resize(unsigned(p8_c0(7 downto 0)), 9));
            p9_c(0) <= unsigned(p8_y0e); p9_c(1) <= unsigned(p8_y0o);
            p9_c(2) <= unsigned(p8_c0(15 downto 8)); p9_c(3) <= unsigned(p8_c0(7 downto 0));
            p9_xp <= p8_xp;
            -- 10: products d aw; 11: c 16 + p (Q4)
            for c in 0 to 3 loop
                p10_p(c) <= resize(p9_d(c) * signed(resize(s_aw(9), 5)), 13);
                p11_v(c) <= signed(resize(p10_c(c) & "0000", 14)) + resize(p10_p(c), 14);
                p10_c(c) <= p9_c(c);
            end loop;
            p10_xp <= p9_xp; p11_xp <= p10_xp;
            -- 12: luma X: left / right column by X parity, d = right - left; chroma passes
            if p11_xp = '0' then v_l := p11_v(0); v_r := p11_v(1); else v_l := p11_v(1); v_r := p11_v(0); end if;
            p12_yl <= v_l; p12_dx <= v_r - v_l;
            p12_u <= p11_v(2); p12_v <= p11_v(3);
            -- 13: px = dx ax
            p13_px <= resize(p12_dx * signed(resize(s_ga(12), 5)), 18);
            p13_yl <= p12_yl; p13_u <= p12_u; p13_v <= p12_v;
            -- 14: 10-bit colour: luma (yl 16 + px) >> 6, chroma >> 2
            v_v := shift_left(resize(p13_yl, 18), 4) + p13_px;
            s_y14 <= unsigned(v_v(15 downto 6));
            s_u14 <= unsigned(p13_u(11 downto 2)); s_v14 <= unsigned(p13_v(11 downto 2));
            -- stripe palette: address at 12 (s_rp(k) = stage k), EBR at 13, fabric copy at 14
            r_bk_a <= s_rp(11);
            p14_bk <= r_bk_q;
            -- 15: compose
            v_front := not s_fd(14)(3) and not s_fd(14)(1);
            if s_lay = '1' then v_front := not v_front; end if;
            if s_gav_sr(12) = '0' then
                v_y := to_unsigned(64, 10); v_u := to_unsigned(512, 10); v_vv := to_unsigned(512, 10);
            elsif v_front = '1' then
                v_y := s_y14; v_u := s_u14; v_vv := s_v14;
            elsif s_fd(14)(2) = '1' then
                v_y := unsigned(p14_bk(23 downto 16)) & "00"; v_u := unsigned(p14_bk(15 downto 8)) & "00"; v_vv := unsigned(p14_bk(7 downto 0)) & "00";
            elsif s_fd(14)(1) = '1' and s_fd(14)(0) = '1' then
                v_y := to_unsigned(126, 10); v_u := to_unsigned(476, 10); v_vv := to_unsigned(491, 10);
            else
                v_y := to_unsigned(64, 10); v_u := to_unsigned(512, 10); v_vv := to_unsigned(512, 10);
            end if;
            s_y15 <= v_y;
            if s_col = '1' then             -- Orchid: Cr' = -Cb, Cb' = Cr
                v_c := (not v_vv) + 1;
                if v_vv = 0 then s_u15 <= to_unsigned(1023, 10); else s_u15 <= v_c; end if;
                s_v15 <= v_u;
            else
                s_u15 <= v_u; s_v15 <= v_vv;
            end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (two adjacent 8-sums each, the 8-back sample from a block-RAM ring), Y
    -- through [1 2 1] (aligned with the chroma), luma 13/16 and the blanking
    -- gate.  The HD HDMI bar fix and the still's soft colour edges.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_l  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_qa <= s_ra_q;                 -- = the stage-15 sample of 8 pixels back
            s_qb <= s_rb_q;                 -- = the stage-17 box-A output of 8 pixels back
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);      -- the avid of the pixel entering s_y15
            if s_bah = "00000000" then
                s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bau <= s_bau + s_u15 - unsigned(s_qa(19 downto 10));
                s_bav <= s_bav + s_v15 - unsigned(s_qa(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            -- box B: stage 18 sums, stage 19 output
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);      -- the avid of the pixel entering s_u17
            if s_bbh = "00000000" then
                s_bbu <= to_unsigned(512 * 8, 13); s_bbv <= to_unsigned(512 * 8, 13);
            else
                s_bbu <= s_bbu + s_u17 - unsigned(s_qb(19 downto 10));
                s_bbv <= s_bbv + s_v17 - unsigned(s_qb(9 downto 0));
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
            -- luma [1 2 1] centred 8 back (with the chroma): stage 17, then 18, 19
            s_la <= unsigned(s_qa(29 downto 20)); s_lb <= s_la;
            v_l := resize(unsigned(s_qa(29 downto 20)), 12) + shift_left(resize(s_la, 12), 1) + resize(s_lb, 12);
            s_y17 <= v_l(11 downto 2);
            s_y18 <= s_y17; s_y19 <= s_y18;
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

end architecture plaid;
