-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Bottleneck v0.5.0 -- Heirloom #28, rebuilt from the photo WIN_20230429_22_57_43.
--
-- The photo: two lathe-shaped bottles (a big one left of centre, a smaller one at the right, leaning
-- outward at the bottom, pointed tips at y ~1000 / ~940), lime necks and pink bodies with a band of
-- pink PWM lines between them, glass rims (a lime rim on the entering edge, a dark notch and a lime
-- line on the leaving edge, blue bands round the necks, outer echo lines), on horizontal stripes
-- (blue / black / green / violet above, pink / green / lime lines below) that run brighter between and
-- near the bottles, cut by sliding frame-buffer glitch blocks and a sample-and-hold staircase on the
-- left bottle's lower right edge.
--
-- The patch, as rebuilt (bottleneck.py has every table):
--   edges   measured edge curves per bottle (every 8 virtual lines, interpolated), offset by Width and
--           Spread, swayed (pendulum, growing toward the bottom); the left bottle's right edge is
--           sample-and-held in 45-line steps (the staircase)
--   fill    lime where the inside distance beats a per-line threshold (Level + a 12-line PWM triangle +
--           line noise) through a 17-step pink -> lime ROM; red slews up after the scan enters a pink body
--   rims    P + (1 - a) * under per 4 px of outward distance, per edge side and edge-adjacent fill
--   stripes three per-row tables (left frame edge, middle, right frame edge) through a 64-colour palette;
--           the outer tables blend toward the middle one near a bottle (the halo)
--   glitch  one horizontal displacement window per line from a 12-block list (P12), each block sliding
--           on one of three LFOs
--   camera  luma (or the Osc) moves the outlines and the fill line (Melt)
--
--   K1  Width     bottle width (+-128 px per edge)
--   K2  Spread    distance between the bottles (+-128 px each way)
--   K3  Level     the lime / pink line in the bottles (left: all lime .. right: all pink)
--   K4  Speed     bipolar: the bottles sway, the stripes wobble, the blocks slide, the PWM lines crawl,
--                 the staircase steps slide
--   K5  Melt      how far the picture (or the Osc) bends the outlines and the fill line
--   K6  Stripes   the stripe field rolls vertically (+-256 lines, wrapping)
--   S7  Bottles   Two (the photo) / One
--   S8  Hold      the staircase On (the photo) / Off
--   S9  Rims      glass rims and halos On (the photo) / Off (bare silhouettes)
--   S10 Colours   Photo / Negative (chroma inverted: lilac necks, green bodies)
--   S11 Source    Video / Osc
--   P12 SHATTER   0 = clean bottles; the photo's blocks and staircase at 45 %; 100 % = the frame torn
--                 into 12 sliding blocks and a staircase to the bottom
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each
-- line start that computes the NEXT line (edges, fill threshold, glitch window) on a second serial
-- multiplier and loads the stripe colours after the line has left the pixel path, a 21-stage pixel path.
-- Syncs and the chroma box delay run in block-RAM rings.  Virtual 1080-line coordinates (SD shows the
-- centre 4:3).  Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4,
-- luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.bottleneck_pkg.all;
use work.all;

architecture bottleneck of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

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
    -- PWM phase step per output line (Q8 of an 8-bit phase): 5461 * (virtual lines per line)
    function f_tps(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(24275, 16);
            when 4          => return to_unsigned(12287, 16);
            when 8          => return to_unsigned(20479, 16);
            when 12         => return to_unsigned(10239, 16);
            when 5 | 6 | 14 => return to_unsigned(8191, 16);
            when 1 | 2 | 10 => return to_unsigned(10922, 16);
            when others     => return to_unsigned(5461, 16);
        end case;
    end function;
    -- 8-bit phase -> triangle 0..255
    function f_tri8(p : unsigned(7 downto 0)) return unsigned is
    begin
        if p(7) = '0' then return p(6 downto 0) & '0'; else return (not p(6 downto 0)) & '1'; end if;
    end function;

    -- glitch blocks: y0, y1, x0, x1 (virtual px / lines), activation threshold (P12)
    type t_u11x12 is array (0 to 11) of unsigned(10 downto 0);
    type t_u15x12 is array (0 to 11) of unsigned(14 downto 0);
    constant B_Y0 : t_u11x12 := (to_unsigned(138, 11), to_unsigned(280, 11), to_unsigned(300, 11), to_unsigned(345, 11), to_unsigned(440, 11), to_unsigned(530, 11),
                                 to_unsigned(828, 11), to_unsigned(60, 11), to_unsigned(600, 11), to_unsigned(760, 11), to_unsigned(900, 11), to_unsigned(204, 11));
    constant B_Y1 : t_u11x12 := (to_unsigned(206, 11), to_unsigned(322, 11), to_unsigned(345, 11), to_unsigned(530, 11), to_unsigned(530, 11), to_unsigned(667, 11),
                                 to_unsigned(865, 11), to_unsigned(118, 11), to_unsigned(690, 11), to_unsigned(822, 11), to_unsigned(962, 11), to_unsigned(262, 11));
    constant B_X0 : t_u11x12 := (to_unsigned(672, 11), to_unsigned(340, 11), to_unsigned(1440, 11), to_unsigned(1440, 11), to_unsigned(1200, 11), to_unsigned(1180, 11),
                                 to_unsigned(200, 11), to_unsigned(900, 11), to_unsigned(0, 11), to_unsigned(1000, 11), to_unsigned(560, 11), to_unsigned(0, 11));
    constant B_X1 : t_u11x12 := (to_unsigned(1920, 11), to_unsigned(460, 11), to_unsigned(1920, 11), to_unsigned(1920, 11), to_unsigned(1345, 11), to_unsigned(1420, 11),
                                 to_unsigned(330, 11), to_unsigned(1920, 11), to_unsigned(700, 11), to_unsigned(1920, 11), to_unsigned(1500, 11), to_unsigned(880, 11));
    type t_i12 is array (0 to 11) of integer;
    constant B_TA : t_i12 := (120, 200, 260, 300, 160, 220, 340, 560, 640, 720, 800, 880);        -- P12 activation (loop constants only)
    constant B_SM : t_u15x12 := (to_unsigned(18, 15), to_unsigned(45, 15), to_unsigned(80, 15), to_unsigned(110, 15), to_unsigned(150, 15), to_unsigned(200, 15),
                                 to_unsigned(25, 15), to_unsigned(70, 15), to_unsigned(80, 15), to_unsigned(130, 15), to_unsigned(100, 15), to_unsigned(60, 15));   -- |base shift|
    constant B_SN : std_logic_vector(0 to 11) := "111100100101";                          -- base shift < 0
    type t_u2x12 is array (0 to 11) of unsigned(1 downto 0);
    constant B_G  : t_u2x12 := ("00", "01", "10", "00", "01", "10", "00", "01", "10", "00", "01", "10");  -- slide LFO = k mod 3
    constant B_CM : t_u15x12 := (to_unsigned(1620, 15), to_unsigned(4050, 15), to_unsigned(7200, 15), to_unsigned(9900, 15), to_unsigned(13500, 15), to_unsigned(18000, 15),
                                 to_unsigned(2250, 15), to_unsigned(6300, 15), to_unsigned(7200, 15), to_unsigned(11700, 15), to_unsigned(9000, 15), to_unsigned(5400, 15));  -- |shift| * 90


    -- line buffer: (luma >> 2) - 16, clipped at 0 (Video) or the oscillator (Osc)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data; data re-registered in fabric before use)
    signal r_e_a  : unsigned(9 downto 0) := (others => '0');
    signal r_e_q  : std_logic_vector(11 downto 0) := (others => '0');
    signal r_i_a  : unsigned(10 downto 0) := (others => '0');
    signal r_i_q  : std_logic_vector(5 downto 0) := (others => '0');
    signal r_p_a  : unsigned(5 downto 0) := (others => '0');
    signal r_p_q  : std_logic_vector(23 downto 0) := (others => '0');
    signal r_d_a  : unsigned(8 downto 0) := (others => '0');
    signal r_d_q  : std_logic_vector(31 downto 0) := (others => '0');

    -- output delay rings in block RAM (one write statement each, collision-free):
    -- syncs by C_LAT, chroma by 8 for the box filter
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    type t_r10 is array (0 to 255) of std_logic_vector(9 downto 0);
    signal rs_ram : t_r4 := (others => "1110");
    signal ru_ram, rv_ram : t_r10 := (others => "1000000000");
    attribute no_rw_check of rs_ram : signal is true;
    attribute no_rw_check of ru_ram : signal is true;
    attribute no_rw_check of rv_ram : signal is true;
    signal rg_wp  : unsigned(7 downto 0) := (others => '0');
    signal rs_ra, ru_ra : unsigned(7 downto 0) := (others => '0');
    signal rs_q, rs_q2 : std_logic_vector(3 downto 0) := "1110";
    signal ru_q, rv_q, ru_q2, rv_q2 : std_logic_vector(9 downto 0) := "1000000000";

    -- oscillator source: slow 2-D triangle (960 x 540 virtual px), drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(68, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(11 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(256 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(460 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_sp : signed(10 downto 0) := (others => '0');
    signal s_one, s_hsw, s_rof, s_neg, s_src : std_logic := '0';

    -- per-field constants
    signal c_dw, c_ds : signed(8 downto 0) := (others => '0');     -- width / spread offsets (px)
    signal c_rl   : signed(9 downto 0) := (others => '0');     -- stripe roll (K6)
    type t_s10x2 is array (0 to 1) of signed(9 downto 0);
    signal c_ea, c_eb : t_s10x2 := (others => (others => '0'));    -- per-bottle left / right edge offsets
    type t_s12x3g is array (0 to 2) of signed(11 downto 0);
    signal c_gk   : t_s12x3g := (others => to_signed(256, 12));     -- glitch shift gain per slide LFO
    type t_u10x3 is array (0 to 2) of unsigned(9 downto 0);
    signal c_gkp  : t_u10x3 := (others => (others => '0'));
    signal c_lv   : signed(10 downto 0) := (others => '0');
    signal c_l3   : signed(11 downto 0) := (others => '0');
    signal c_ms   : unsigned(2 downto 0) := to_unsigned(2, 3);        -- Melt shift 0..6
    signal c_gs   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_hon  : std_logic := '1';
    signal c_hend : unsigned(10 downto 0) := to_unsigned(897, 11);
    signal c_hst  : unsigned(5 downto 0) := to_unsigned(45, 6);
    signal c_tq   : unsigned(10 downto 0) := (others => '0');
    signal c_tph  : unsigned(7 downto 0) := (others => '0');
    signal z_t0, z_t1, z_t2, z_t3, z_t4 : unsigned(7 downto 0) := (others => '0');
    signal z_x0, z_x1 : signed(8 downto 0) := (others => '0');
    signal c_sw   : signed(6 downto 0) := (others => '0');
    signal c_wob  : signed(4 downto 0) := (others => '0');
    signal c_pph  : unsigned(5 downto 0) := (others => '0');
    signal c_hoff : unsigned(3 downto 0) := (others => '0');
    signal c_fc   : unsigned(15 downto 0) := (others => '0');
    signal c_lfs  : unsigned(15 downto 0) := to_unsigned(16#ACE1#, 16);   -- texture LFSR seed
    signal c_tps  : unsigned(15 downto 0) := to_unsigned(5461, 16);       -- PWM phase step per line, Q8
    signal c_act  : std_logic_vector(0 to 11) := (others => '0');
    type t_s9x3z is array (0 to 2) of signed(8 downto 0);
    signal c_tc   : t_s9x3z := (others => (others => '0'));        -- slide LFOs (x2, x3, x4 rate), -128 .. 127
    signal c_g    : unsigned(9 downto 0) := to_unsigned(460, 10);

    -- zone serial multiplier
    signal f_a   : unsigned(19 downto 0) := (others => '0');
    signal f_b   : unsigned(9 downto 0) := (others => '0');
    signal f_acc : unsigned(19 downto 0) := (others => '0');
    signal z_fa  : unsigned(19 downto 0) := (others => '0');
    signal z_fb  : unsigned(9 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_lo  : std_logic_vector(0 to 3) := (others => '0');   -- knob below its knee
    signal z_tk  : unsigned(4 downto 0) := (others => '0');
    signal z_sl  : unsigned(2 downto 0) := (others => '0');
    signal z_run : std_logic := '0';
    signal z_st  : std_logic := '0';
    signal z_ss  : unsigned(2 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_v    : unsigned(10 downto 0) := (others => '0');
    signal e_hn   : unsigned(10 downto 0) := to_unsigned(762, 11);
    signal e_hb   : std_logic := '0';
    signal e_yq   : unsigned(10 downto 0) := (others => '0');
    signal e_w0   : unsigned(11 downto 0) := (others => '0');
    type t_s13x4 is array (0 to 3) of signed(12 downto 0);
    signal e_ed   : t_s13x4 := (others => (others => '0'));
    signal e_dd, e_e0 : signed(12 downto 0) := (others => '0');
    signal e_pp   : signed(15 downto 0) := (others => '0');
    signal e_f1   : unsigned(2 downto 0) := (others => '0');      -- frac of the Lr row (hold)
    signal e_r1   : unsigned(7 downto 0) := (others => '0');      -- row of Lr
    type t_s14x2 is array (0 to 1) of signed(13 downto 0);
    type t_s11x4 is array (0 to 3) of signed(10 downto 0);
    signal e_o    : t_s11x4 := (others => (others => '0'));
    signal e_far  : std_logic_vector(0 to 1) := "00";
    signal e_swy  : signed(9 downto 0) := (others => '0');
    signal e_tp   : unsigned(7 downto 0) := (others => '0');
    signal e_tpa  : unsigned(15 downto 0) := (others => '0');      -- PWM phase, Q8
    signal e_lfa, e_lfb : unsigned(15 downto 0) := to_unsigned(16#ACE1#, 16);
    signal e_nq   : unsigned(8 downto 0) := (others => '1');
    signal e_pt   : unsigned(4 downto 0) := (others => '0');
    signal e_ps   : unsigned(3 downto 0) := (others => '0');
    signal e_oa   : signed(15 downto 0) := (others => '0');
    signal e_ob   : unsigned(15 downto 0) := (others => '0');
    signal e_sh   : std_logic_vector(1 downto 0) := "00";
    signal e_pr   : signed(13 downto 0) := (others => '0');
    signal e_sa   : signed(10 downto 0) := (others => '0');
    signal e_cpend : std_logic := '0';
    signal e_cc   : unsigned(3 downto 0) := (others => '0');
    signal e_nz   : signed(8 downto 0) := (others => '0');
    signal e_ri   : unsigned(9 downto 0) := (others => '0');
    signal e_rr   : signed(12 downto 0) := (others => '0');
    signal e_ta   : signed(9 downto 0) := (others => '0');
    signal e_p1, e_p2 : signed(15 downto 0) := (others => '0');
    signal e_pwm  : signed(9 downto 0) := (others => '0');
    type t_s13x2 is array (0 to 1) of signed(12 downto 0);
    signal e_t, e_u : t_s13x2 := (others => (others => '0'));
    signal e_k    : unsigned(3 downto 0) := (others => '0');
    signal e_ka   : unsigned(3 downto 0) := (others => '0');
    signal e_va   : std_logic := '0';
    signal e_ms   : std_logic := '0';                              -- product sign
    signal m_a    : unsigned(21 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(21 downto 0) := (others => '0');
    -- next-line results
    signal n_el, n_er : t_s13x2 := (others => (others => '0'));
    signal n_th : t_s13x2 := (others => (others => '0'));          -- fill threshold - 32
    type t_u2x2 is array (0 to 1) of unsigned(1 downto 0);
    signal n_ty : t_u2x2 := (others => (others => '0'));
    signal n_rk : unsigned(2 downto 0) := (others => '0');
    signal n_tx : signed(4 downto 0) := (others => '0');
    signal n_zl, n_zr : std_logic := '0';
    signal n_md, n_mda : signed(12 downto 0) := (others => '0');
    type t_c24x3 is array (0 to 2) of std_logic_vector(23 downto 0);
    signal n_a0, n_a1 : unsigned(11 downto 0) := (others => '0');
    signal n_sa : signed(10 downto 0) := (others => '0');
    -- current-line registers (loaded at the line start)
    signal l_el, l_er : t_s13x2 := (others => (others => '0'));
    signal l_th : t_s13x2 := (others => (others => '0'));
    signal l_ty : t_u2x2 := (others => (others => '0'));
    signal l_rk : unsigned(2 downto 0) := (others => '0');
    signal l_tx : signed(4 downto 0) := (others => '0');
    signal l_zl, l_zr : std_logic := '0';
    signal l_md, l_mda : signed(12 downto 0) := (others => '0');
    signal l_col : t_c24x3 := (others => (others => '0'));
    signal l_a0, l_a1 : unsigned(11 downto 0) := (others => '0');
    signal l_sa : signed(10 downto 0) := (others => '0');

    -- beam
    signal b_x  : unsigned(11 downto 0) := (others => '0');
    signal b_px : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal p2_x : unsigned(10 downto 0) := (others => '0');
    signal p2_ia, p2_s0, p2_sa : std_logic := '0';
    signal p3_xs : signed(12 downto 0) := (others => '0');
    signal p3_lm : signed(8 downto 0) := (others => '0');
    signal p3_sel : std_logic := '0';
    signal p4_dl, p4_dr, p4_rr : signed(12 downto 0) := (others => '0');
    signal p4_sel : std_logic := '0';
    signal p4_lp  : signed(14 downto 0) := (others => '0');
    signal p5_d, p5_xi, p5_rr : signed(12 downto 0) := (others => '0');
    signal p5_sd, p5_sel : std_logic := '0';
    signal p5_lb : signed(12 downto 0) := (others => '0');
    signal p6_d, p6_xi, p6_r15, p6_lb : signed(12 downto 0) := (others => '0');
    signal p6_sd, p6_sel : std_logic := '0';
    signal p6_z  : unsigned(1 downto 0) := (others => '0');
    signal p7_dn, p7_xi : signed(12 downto 0) := (others => '0');
    signal p7_w, p8_w, p9_w : unsigned(4 downto 0) := (others => '0');
    signal p7_sel, p7_ng, p8_ng, p9_ng, p10_ng, p11_ng : std_logic := '0';
    signal p7_ra, p8_ra, p9_ra : unsigned(8 downto 0) := (others => '0');
    signal p7_z, p8_z, p9_z, p10_z : unsigned(1 downto 0) := (others => '0');
    signal p8_kr : signed(13 downto 0) := (others => '0');
    signal p8_rm : unsigned(7 downto 0) := (others => '0');
    type t_s9x3 is array (0 to 2) of signed(8 downto 0);
    signal p9_df : t_s9x3 := (others => (others => '0'));
    signal p9_kk : unsigned(4 downto 0) := (others => '0');
    signal p9_rp : unsigned(10 downto 0) := (others => '0');
    signal p9_kl : std_logic := '0';
    type t_s15x3 is array (0 to 2) of signed(14 downto 0);
    signal p10_bp : t_s15x3 := (others => (others => '0'));
    signal p10_fc : std_logic_vector(23 downto 0) := (others => '0');
    signal p10_r5, p10_r7 : unsigned(13 downto 0) := (others => '0');
    signal p11_fi, p11_bg : t_s9x3 := (others => (others => '0'));
    signal p12_un : t_s9x3 := (others => (others => '0'));
    signal p12_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal p13_pr : t_s15x3 := (others => (others => '0'));
    signal p13_q  : std_logic_vector(23 downto 0) := (others => '0');
    type t_s10x3 is array (0 to 2) of signed(9 downto 0);
    signal p14_o  : t_s10x3 := (others => (others => '0'));
    type t_s12x3 is array (0 to 2) of signed(11 downto 0);
    signal p15_o  : t_s12x3 := (others => (others => '0'));
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bxu, s_bxv, s_bxud, s_bxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ya : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
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
            r_e_q <= C_BN_EDGE(to_integer(r_e_a));
            r_i_q <= C_BN_IDX(to_integer(r_i_a));
            r_p_q <= C_BN_PAL(to_integer(r_p_a));
            r_d_q <= C_BN_DEC(to_integer(r_d_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Output delay rings: write at the pointer, read (C_LAT - 2) / 6 behind,
    -- plus a fabric copy (C_LAT / 8 clocks in all)
    ---------------------------------------------------------------------------
    p_ringw : process(clk)
    begin
        if rising_edge(clk) then
            rs_ram(to_integer(rg_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & '0';
            ru_ram(to_integer(rg_wp)) <= std_logic_vector(s_u17);
            rv_ram(to_integer(rg_wp)) <= std_logic_vector(s_v17);
        end if;
    end process p_ringw;
    p_ringr : process(clk)
    begin
        if rising_edge(clk) then
            rs_q <= rs_ram(to_integer(rs_ra));
            ru_q <= ru_ram(to_integer(ru_ra));
            rv_q <= rv_ram(to_integer(ru_ra));
        end if;
    end process p_ringr;
    p_ringa : process(clk)
        variable v_wp : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            v_wp := rg_wp + 1;
            rg_wp <= v_wp;
            rs_ra <= v_wp - (C_LAT - 2);   -- address used in the next clock
            ru_ra <= v_wp - 6;
            rs_q2 <= rs_q; ru_q2 <= ru_q; rv_q2 <= rv_q;
        end if;
    end process p_ringa;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_r  : unsigned(19 downto 0);
        variable v_k  : integer range 0 to 7;
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
            -- speed: bipolar, +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            if v_s > -8 and v_s < 8 then
                s_sp <= (others => '0');
            elsif v_s(10) = '1' then
                s_sp <= v_s + 8;
            else
                s_sp <= v_s - 8;
            end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_one  <= registers_in(6)(0);
                s_hsw  <= not registers_in(6)(1);       -- S8: 0 = On (the photo)
                s_rof  <= registers_in(6)(2);           -- S9: 1 = rims off
                s_neg  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 12); s_oxs <= to_unsigned(136, 9);
                    when 3      => s_x0 <= (others => '0');      s_oxs <= to_unsigned(102, 9);
                    when others => s_x0 <= (others => '0');      s_oxs <= to_unsigned(68, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband), four clocks per channel
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
                        elsif s_gd > 0 then
                            s_gd <= to_signed(1, 15);
                        elsif s_gd < 0 then
                            s_gd <= to_signed(-1, 15);
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

            -- per-field values (zone clocks 40 .. 52, after the glide), one small op each
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>
                        c_tq <= unsigned(signed(c_tq) + s_sp);
                        c_fc <= c_fc + 1;
                        c_g  <= s_gp(12 downto 3);
                        s_ot <= s_ot + 1;
                    when 41 =>
                        c_tph <= c_tq(10 downto 3);
                        c_lfs <= x"ACE1" xor resize(c_fc(15 downto 3), 16);
                        c_tps <= f_tps(s_tid);
                        if c_g < 460 then z_lo(3) <= '1'; else z_lo(3) <= '0'; end if;
                        c_l3 <= resize(signed(resize(s_g3(12 downto 3), 11)) - 512, 12);
                        c_dw <= resize(shift_right(signed(resize(s_g1(12 downto 3), 11)) - 512, 2), 9);
                        c_ds <= resize(shift_right(signed(resize(s_g2(12 downto 3), 11)) - 512, 2), 9);
                        c_rl <= resize(shift_right(signed(resize(s_g6(12 downto 3), 11)) - 512, 1), 10);
                    when 42 =>
                        z_t0 <= c_tph + 64;
                        z_t1 <= ('0' & c_tph(7 downto 1)) + 64;
                        z_t2 <= shift_left(c_tph, 1) + 64;
                        z_t3 <= shift_left(c_tph, 1) + c_tph;
                        z_t4 <= shift_left(c_tph, 2) + 64;
                        c_pph <= c_tph(7 downto 2);
                        c_hoff <= c_tph(7 downto 4);
                        c_l3 <= c_l3 + shift_left(c_l3, 1);
                        c_ea(0) <= -resize(c_dw, 10); c_ea(1) <= -resize(c_dw, 10);   -- left edges move out by dw
                    when 43 =>
                        z_t3 <= z_t3 + 64;
                        z_x0 <= signed(resize(f_tri8(z_t0), 9)) - 128;
                        z_x1 <= signed(resize(f_tri8(z_t1), 9)) - 128;
                        c_tc(0) <= signed(resize(f_tri8(z_t2), 9)) - 128;
                        c_lv <= resize(shift_right(c_l3, 2), 11);
                        c_ea(0) <= c_ea(0) - resize(c_ds, 10); c_ea(1) <= c_ea(1) + resize(c_ds, 10);   -- spread: L left, R right
                        c_eb(0) <= resize(c_dw, 10) - resize(c_ds, 10); c_eb(1) <= resize(c_dw, 10) + resize(c_ds, 10);
                    when 44 =>
                        c_tc(1) <= signed(resize(f_tri8(z_t3), 9)) - 128;
                        c_tc(2) <= signed(resize(f_tri8(z_t4), 9)) - 128;
                        c_sw  <= resize(shift_right(shift_left(resize(z_x0, 13), 5) + shift_left(resize(z_x0, 13), 3), 7), 7);
                        c_wob <= resize(shift_right(shift_left(resize(z_x1, 13), 3) + shift_left(resize(z_x1, 13), 2), 7), 5);
                    when 45 =>
                        for k in 0 to 11 loop
                            if c_g >= B_TA(k) then c_act(k) <= '1'; else c_act(k) <= '0'; end if;
                        end loop;
                        if s_g5(12 downto 10) = "111" then c_ms <= "110"; else c_ms <= s_g5(12 downto 10); end if;
                        if c_g >= 200 then c_hon <= s_hsw; else c_hon <= '0'; end if;
                        if c_g < 700 then c_hst <= to_unsigned(45, 6); else c_hst <= to_unsigned(60, 6); end if;
                    when 200 =>                                             -- slide gains = gs +- slide
                        for m in 0 to 2 loop
                            if c_tc(m) < 0 then c_gk(m) <= signed(resize(c_gs, 12)) - signed(resize(c_gkp(m), 12));
                            else c_gk(m) <= signed(resize(c_gs, 12)) + signed(resize(c_gkp(m), 12)); end if;
                        end loop;
                    when others => null;
                end case;
            end if;

            -- serial products: slot timer, tick 0 = pre-mux, 1 = load, 13 = store; 16 clocks per
            -- slot, 5 slots from zone clock 62 (done by 142)
            z_pre <= '0'; z_st <= '0';
            if s_zone = '0' then
                z_run <= '0'; z_tk <= (others => '0'); z_sl <= (others => '0');
            elsif s_zc = 62 then
                z_run <= '1'; z_tk <= (others => '0'); z_sl <= (others => '0');
            elsif z_run = '1' then
                if z_tk = 0 then z_sel <= z_sl; z_pre <= '1'; end if;
                if z_tk = 13 then z_ss <= z_sl; z_st <= '1'; end if;
                if z_tk = 15 then
                    z_tk <= (others => '0');
                    if z_sl = 4 then z_run <= '0'; end if;
                    z_sl <= z_sl + 1;
                else
                    z_tk <= z_tk + 1;
                end if;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 =>                                                    -- glitch shift gain
                        if z_lo(3) = '1' then z_fa <= resize(c_g, 20); z_fb <= to_unsigned(142, 10);
                        else z_fa <= resize(c_g - 460, 20); z_fb <= to_unsigned(320, 10); end if;
                    when 1 =>                                                    -- hold end
                        if z_lo(3) = '1' then z_fa <= resize(c_g, 20); z_fb <= to_unsigned(180, 10);
                        else z_fa <= resize(c_g - 460, 20); z_fb <= to_unsigned(400, 10); end if;
                    when 2      => z_fa <= resize(c_gs, 20); z_fb <= resize(unsigned(abs(c_tc(0))), 10);
                    when 3      => z_fa <= resize(c_gs, 20); z_fb <= resize(unsigned(abs(c_tc(1))), 10);
                    when others => z_fa <= resize(c_gs, 20); z_fb <= resize(unsigned(abs(c_tc(2))), 10);
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(18 downto 0) & '0';
                f_b <= '0' & f_b(9 downto 1);
            end if;
            if z_st = '1' then
                v_r := f_acc;
                case to_integer(z_ss) is
                    when 0 =>
                        if z_lo(3) = '1' then c_gs <= resize(v_r(17 downto 8), 10);
                        else c_gs <= to_unsigned(256, 10) + resize(v_r(17 downto 8), 10); end if;
                    when 1 =>
                        if z_lo(3) = '1' then
                            c_hend <= to_unsigned(717, 11) + resize(v_r(18 downto 9), 11);
                        elsif v_r(18 downto 9) > 182 then
                            c_hend <= to_unsigned(1079, 11);
                        else
                            c_hend <= to_unsigned(897, 11) + resize(v_r(18 downto 9), 11);
                        end if;
                    when 2      => c_gkp(0) <= resize(shift_right(v_r + shift_left(v_r, 1), 10), 10);   -- 3/1024 = slide 0.375
                    when 3      => c_gkp(1) <= resize(shift_right(v_r + shift_left(v_r, 1), 10), 10);
                    when others => c_gkp(2) <= resize(shift_right(v_r + shift_left(v_r, 1), 10), 10);
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
    -- Source side: oscillator (slow 2-D triangle, drifting), line-buffer write
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 8);                                      -- phase t
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 8) + shift_left(resize(s_ot, 16), 7); end if;  -- 1.5 t
            v_s := resize(f_tri8(s_ox(15 downto 8)), 9) + resize(f_tri8(s_oy(15 downto 8)), 9);
            s_osum <= to_unsigned(32, 8) + v_s(8 downto 2);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result
    -- stable at load + 17)
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line.  Products run on one serial multiplier in fixed
    -- slots (operands, shift, sign and destination decoded from the slot
    -- number; one shared post-processor); everything else is one small
    -- operation per clock.  The stripe colours are loaded straight into l_col
    -- after the active line has left the pixel path.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_i  : integer range 0 to 7;
        variable v_e  : integer range 0 to 3;
        variable v_d  : signed(13 downto 0);
        variable v_ky : integer range 0 to 15;
        variable v_n  : integer range 0 to 3;
        variable v_lc : integer range 0 to 511;
        variable v_p  : unsigned(13 downto 0);
        variable v_l  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 900 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_hn <= to_unsigned(717, 11) + c_hst - c_hoff;
                e_tpa <= shift_left(resize(c_pph, 16), 8);
                e_lfa <= c_lfs;
                e_lfb <= to_unsigned(16#1D87#, 16);
                e_nq <= (others => '1');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_tpa <= e_tpa + c_tps;
                e_lc <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            -- serial multiplier
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(20 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            v_lc := to_integer(e_lc);
            case v_lc is
                when 0 =>
                    if e_vy(18 downto 8) > 1079 then e_v <= to_unsigned(1079, 11); else e_v <= e_vy(18 downto 8); end if;
                when 1 =>
                    if e_v >= e_hn then e_hn <= e_hn + c_hst; end if;
                    -- line LFSRs (Galois, taps B400): texture every line, PWM noise every 4 virtual lines
                    if e_lfa(0) = '1' then e_lfa <= ('0' & e_lfa(15 downto 1)) xor x"B400"; else e_lfa <= '0' & e_lfa(15 downto 1); end if;
                    if e_lfb(0) = '1' then e_lfb <= ('0' & e_lfb(15 downto 1)) xor x"B400"; else e_lfb <= '0' & e_lfb(15 downto 1); end if;
                when 2 =>
                    if e_hn > 1079 then e_yq <= to_unsigned(1079, 11); else e_yq <= e_hn; end if;
                    if c_hon = '1' and e_v >= 717 and e_v < c_hend then e_hb <= '1'; else e_hb <= '0'; end if;
                    if e_v(10 downto 2) /= e_nq then e_nq <= e_v(10 downto 2); e_nz <= signed(resize(e_lfb(7 downto 0), 9)) - 128; end if;
                    n_tx <= signed(resize(e_lfa(11 downto 8), 5)) - 8;
                when 3 =>
                    if e_hb = '1' then e_r1 <= e_yq(10 downto 3); e_f1 <= e_yq(2 downto 0);
                    else e_r1 <= e_v(10 downto 3); e_f1 <= e_v(2 downto 0); end if;
                    if e_v >= 760 then n_rk <= (others => '0');
                    elsif e_v <= 704 then n_rk <= to_unsigned(7, 3);
                    else v_p := resize(760 - e_v, 14); n_rk <= v_p(5 downto 3); end if;
                when 4 to 11 =>                                    -- edge ROM addresses: (k, row), (k, row + 1)
                    v_i := v_lc - 4;
                    v_e := v_i / 2;
                    if v_e = 1 then
                        if (v_i mod 2) = 0 then r_e_a <= "01" & e_r1; else r_e_a <= "01" & (e_r1 + 1); end if;
                    else
                        if (v_i mod 2) = 0 then r_e_a <= to_unsigned(v_e, 2) & e_v(10 downto 3);
                        else r_e_a <= to_unsigned(v_e, 2) & (e_v(10 downto 3) + 1); end if;
                    end if;
                when others => null;
            end case;
            -- edge k: row word at lc 6 + 2k, next-row word at 7 + 2k; difference at 7 + 2k,
            -- x frac at 8 + 2k, edge = e0 + (d f >> 3) at 9 + 2k
            if v_lc >= 6 and v_lc <= 13 and (v_lc mod 2) = 0 then
                e_w0 <= unsigned(r_e_q);
            end if;
            if v_lc >= 7 and v_lc <= 13 and (v_lc mod 2) = 1 then
                e_dd <= signed(resize(unsigned(r_e_q), 13)) - signed(resize(e_w0, 13));
                e_e0 <= signed(resize(e_w0, 13)) - 512;
            end if;
            if v_lc >= 8 and v_lc <= 14 and (v_lc mod 2) = 0 then
                if v_lc = 10 then e_pp <= resize(e_dd * signed(resize(e_f1, 4)), 16);
                else e_pp <= resize(e_dd * signed(resize(e_v(2 downto 0), 4)), 16); end if;
            end if;
            if v_lc >= 9 and v_lc <= 15 and (v_lc mod 2) = 1 then
                v_n := (v_lc - 9) / 2;
                e_ed(v_n) <= e_e0 + resize(shift_right(e_pp, 3), 13);
            end if;
            -- glitch blocks covering the line: the first -> window A, the last -> window B
            if v_lc = 4 then e_va <= '0'; e_k <= (others => '0'); end if;
            if v_lc >= 5 and v_lc <= 16 then
                v_ky := to_integer(e_k);
                if c_act(v_ky) = '1' and e_v >= B_Y0(v_ky) and e_v < B_Y1(v_ky) then
                    e_ka <= e_k; e_va <= '1';
                end if;
                e_k <= e_k + 1;
            end if;
            case v_lc is
                when 18 =>                                          -- one bottle
                    if s_one = '1' then
                        e_ed(2) <= to_signed(1990, 13); e_ed(3) <= to_signed(1990, 13);
                    end if;
                when others => null;
            end case;

            -- product slots j = 0..1: operands at 19 + 20 j, load at 20 + 20 j, result at 37 + 20 j,
            -- post-processed at 38 + 20 j, stored at 39 + 20 j:  0 = sway (vy |sw| >> 10),
            -- 1 = the window's shift (|s_k| gain_k >> 8)
            if v_lc >= 19 and v_lc < 59 then
                if e_pt = 19 then e_pt <= (others => '0'); e_ps <= e_ps + 1; else e_pt <= e_pt + 1; end if;
            else
                e_pt <= (others => '0'); e_ps <= (others => '0');
            end if;
            if v_lc >= 19 and v_lc < 59 and e_pt = 0 then
                case to_integer(e_ps) is
                    when 0 =>
                        e_oa <= signed(resize(e_v, 16)); e_ob <= resize(unsigned(abs(c_sw)), 16);
                        e_ms <= c_sw(6); e_sh <= "10";
                    when others =>
                        e_oa <= signed(resize(B_SM(to_integer(e_ka)), 16));
                        e_ob <= unsigned(resize(c_gk(to_integer(B_G(to_integer(e_ka)))), 16));
                        e_ms <= B_SN(to_integer(e_ka)); e_sh <= "00";
                end case;
            end if;
            if v_lc >= 19 and v_lc < 59 and e_pt = 1 then
                m_acc <= (others => '0');
                m_a <= resize(unsigned(e_oa), 22);
                m_b <= e_ob;
            end if;
            if v_lc >= 19 and v_lc < 59 and e_pt = 18 then
                if e_sh = "10" then v_p := resize(m_acc(21 downto 10), 14); else v_p := m_acc(21 downto 8); end if;
                if e_ms = '1' then e_pr <= -signed(v_p); else e_pr <= signed(v_p); end if;
            end if;
            if v_lc >= 19 and v_lc < 59 and e_pt = 19 then
                case to_integer(e_ps) is
                    when 0      => e_swy <= resize(e_pr, 10);
                    when others => e_sa <= resize(e_pr, 11);
                end case;
            end if;

            case v_lc is
                -- edges: + width / spread offsets, -+ sway (left bottle left, right bottle right)
                when 80 =>
                    for b in 0 to 1 loop
                        e_far(b) <= '0';
                        if e_ed(2 * b) > 1900 or e_ed(2 * b) < -50 then e_far(b) <= '1'; end if;
                    end loop;
                    e_o(0) <= resize(c_ea(0), 11) - resize(e_swy, 11); e_o(1) <= resize(c_eb(0), 11) - resize(e_swy, 11);
                    e_o(2) <= resize(c_ea(1), 11) + resize(e_swy, 11); e_o(3) <= resize(c_eb(1), 11) + resize(e_swy, 11);
                    e_ta <= signed(resize(e_tpa(15 downto 8), 10)) - 128;
                when 81 =>
                    for b in 0 to 1 loop
                        if e_far(b) = '0' then
                            n_el(b) <= e_ed(2 * b) + resize(e_o(2 * b), 13);
                            n_er(b) <= e_ed(2 * b + 1) + resize(e_o(2 * b + 1), 13);
                        else
                            n_el(b) <= e_ed(2 * b); n_er(b) <= e_ed(2 * b + 1);
                        end if;
                    end loop;
                    e_ta <= abs(e_ta);
                when 82 =>
                    v_d := resize(n_er(0), 14) + resize(n_el(1), 14);
                    n_md <= resize(shift_right(v_d, 1), 13);
                    e_ta <= e_ta - 64;
                when 83 =>
                    e_p1 <= shift_left(resize(e_ta, 16), 6) - shift_left(resize(e_ta, 16), 2);
                    e_p2 <= shift_left(resize(e_nz, 16), 6) + shift_left(resize(e_nz, 16), 2);
                when 84 =>
                    n_mda <= n_md + resize(e_sa, 13);
                    n_sa <= e_sa;
                    if e_va = '1' then n_a0 <= resize(B_X0(to_integer(e_ka)), 12); n_a1 <= resize(B_X1(to_integer(e_ka)), 12);
                    else n_a0 <= to_unsigned(4095, 12); n_a1 <= (others => '0'); end if;
                    e_p1 <= shift_right(e_p1, 6);
                    e_p2 <= e_p2 + shift_left(resize(e_nz, 16), 1);
                when 85 =>
                    e_pwm <= resize(e_p1 + shift_right(e_p2, 7), 10);
                    e_t(0) <= signed(resize(e_v, 13)) - 310;
                    e_t(1) <= signed(resize(e_v, 13)) - 360;
                when 86 =>
                    for b in 0 to 1 loop e_t(b) <= e_t(b) + resize(c_lv, 13); end loop;
                -- th = 2 t + (13 t >> 6) + pwm, stored minus 32 (the key's half width)
                when 87 =>
                    for b in 0 to 1 loop e_u(b) <= shift_left(e_t(b), 3) + shift_left(e_t(b), 2); end loop;
                when 88 =>
                    for b in 0 to 1 loop e_u(b) <= e_u(b) + e_t(b); end loop;
                when 89 =>
                    for b in 0 to 1 loop n_th(b) <= shift_left(e_t(b), 1) + shift_right(e_u(b), 6); end loop;
                when 90 =>
                    for b in 0 to 1 loop n_th(b) <= n_th(b) + resize(e_pwm, 13); end loop;
                when 91 =>
                    for b in 0 to 1 loop
                        if n_th(b) <= 8 then n_ty(b) <= "00";
                        elsif e_v >= 740 then n_ty(b) <= "10";
                        else n_ty(b) <= "01"; end if;
                    end loop;
                    if e_v <= 1000 then n_zl <= '1'; else n_zl <= '0'; end if;
                    if e_v <= 950 and s_one = '0' then n_zr <= '1'; else n_zr <= '0'; end if;
                    e_rr <= signed(resize(e_v, 13)) + resize(c_rl, 13);
                when 92 =>
                    for b in 0 to 1 loop n_th(b) <= n_th(b) - 32; end loop;
                when 93 =>
                    if e_rr < 0 then e_rr <= e_rr + 1080;              -- the K6 roll wraps
                    elsif e_rr > 1079 then e_rr <= e_rr - 1080; end if;
                when 94 =>
                    e_rr <= e_rr + resize(c_wob, 13);                  -- the Speed wobble clamps
                when 95 =>
                    if e_rr < 0 then e_ri <= (others => '0');
                    elsif e_rr > 1079 then e_ri <= to_unsigned(539, 10);
                    else e_ri <= unsigned(e_rr(10 downto 1)); end if;
                    e_cpend <= '1';
                when others => null;
            end case;

            -- stripe colours: once the active line has left the pixel path, index ROM (3 reads),
            -- palette ROM (3 reads), straight into l_col
            if e_cpend = '1' and g_avid = '0' and s_gav_sr(12) = '0' and e_cc = 0 then
                e_cc <= to_unsigned(1, 4); e_cpend <= '0';
            elsif e_cc /= 0 then
                if e_cc = 9 then e_cc <= (others => '0'); else e_cc <= e_cc + 1; end if;
            end if;
            case to_integer(e_cc) is
                when 1 => r_i_a <= resize(e_ri, 11);
                when 2 => r_i_a <= resize(e_ri, 11) + 540;
                when 3 => r_i_a <= resize(e_ri, 11) + 1080; r_p_a <= unsigned(r_i_q);
                when 4 => r_p_a <= unsigned(r_i_q);
                when 5 => r_p_a <= unsigned(r_i_q); l_col(0) <= r_p_q;
                when 6 => l_col(1) <= r_p_q;
                when 7 => l_col(2) <= r_p_q;
                when others => null;
            end case;

            -- load the computed line at the line start
            if g_start = '1' then
                l_el <= n_el; l_er <= n_er;
                l_th <= n_th; l_ty <= n_ty; l_rk <= n_rk; l_tx <= n_tx;
                l_zl <= n_zl; l_zr <= n_zr;
                l_md <= n_md; l_mda <= n_mda;
                l_a0 <= n_a0; l_a1 <= n_a1; l_sa <= n_sa;
            end if;
        end if;
    end process p_line;


    ---------------------------------------------------------------------------
    -- Beam: virtual x in h units (half px)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= s_x0;
                b_px <= (others => '0');
            elsif g_avid = '1' then
                b_x  <= b_x + s_xs;
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel; one operation per stage).
    -- One bottle per pixel: the left one left of the midpoint between them.
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x  : signed(12 downto 0);
        variable v_k  : signed(11 downto 0);
        variable v_z  : std_logic_vector(23 downto 0);
        variable v_n  : std_logic_vector(23 downto 0);
        variable v_bi : signed(12 downto 0);
        variable v_ba : unsigned(5 downto 0);
        variable v_ty : unsigned(1 downto 0);
        variable v_o  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: x (virtual px), glitch-window tests, bottle select (x >= midpoint + shift)
            p2_x <= b_x(11 downto 1);
            if b_x(11 downto 1) >= l_a0(10 downto 0) and resize(b_x(11 downto 1), 12) < l_a1 and l_a0(11) = '0' then p2_ia <= '1'; else p2_ia <= '0'; end if;
            if signed(resize(b_x(11 downto 1), 13)) >= l_md  then p2_s0 <= '1'; else p2_s0 <= '0'; end if;
            if signed(resize(b_x(11 downto 1), 13)) >= l_mda then p2_sa <= '1'; else p2_sa <= '0'; end if;
            -- 3: displaced x; luma - 96
            v_x := signed(resize(p2_x, 13));
            if p2_ia = '1' then p3_xs <= v_x - resize(l_sa, 13); p3_sel <= p2_sa;
            else p3_xs <= v_x; p3_sel <= p2_s0; end if;
            p3_lm <= signed(resize(unsigned(s_lb_q), 9)) - 96;
            -- 4: edge distances of the selected bottle; melt = (luma - 96) << Melt
            if p3_sel = '1' then
                p4_dl <= l_el(1) - p3_xs; p4_dr <= p3_xs - l_er(1); p4_rr <= l_er(1) - p3_xs;
            else
                p4_dl <= l_el(0) - p3_xs; p4_dr <= p3_xs - l_er(0); p4_rr <= l_er(0) - p3_xs;
            end if;
            p4_sel <= p3_sel;
            p4_lp <= shift_left(resize(p3_lm, 15), to_integer(c_ms));
            -- 5: side = right of the centre (dr >= dl), outward distance = max(dl, dr); 1.5 x the
            --    distance to the right edge; melt / 16
            if p4_dr >= p4_dl then p5_d <= p4_dr; p5_sd <= '1'; else p5_d <= p4_dl; p5_sd <= '0'; end if;
            p5_xi <= -p4_dl; p5_rr <= p4_rr + shift_right(p4_rr, 1);
            p5_sel <= p4_sel;
            p5_lb <= resize(shift_right(p4_lp, 4), 13);
            -- 6: melt applied to the outward distance; inside distance min(x - el, 1.5 (er - x)); zones
            p6_d <= p5_d - p5_lb;
            if p5_xi < p5_rr then p6_r15 <= p5_xi; else p6_r15 <= p5_rr; end if;
            p6_xi <= p5_xi;
            p6_lb <= p5_lb;
            p6_sd <= p5_sd; p6_sel <= p5_sel;
            if p5_sel = '0' and p5_sd = '0' and l_zl = '1' then p6_z <= "01";
            elsif p5_sel = '1' and p5_sd = '1' and l_zr = '1' then p6_z <= "10";
            else p6_z <= "00"; end if;
            -- 7: inside distance + melt; rim ROM address (type 3 = pass-through outside -48 .. 207); halo
            p7_dn <= p6_r15 + p6_lb;
            p7_xi <= p6_xi;
            if p6_sel = '1' then v_ty := l_ty(1); else v_ty := l_ty(0); end if;
            v_bi := shift_right(p6_d + 48, 2);
            if v_bi < 0 or v_bi > 63 or s_rof = '1' then v_ty := "11"; v_ba := (others => '0'); else v_ba := unsigned(v_bi(5 downto 0)); end if;
            p7_ra <= p6_sd & v_ty & v_ba;
            if p6_z = "00" then
                p7_w <= to_unsigned(16, 5);
            elsif p6_d < 0 then
                p7_w <= unsigned(C_BN_HALO(0));
            elsif p6_d > 511 then
                p7_w <= unsigned(C_BN_HALO(63));
            else
                p7_w <= unsigned(C_BN_HALO(to_integer(unsigned(p6_d(8 downto 3)))));
            end if;
            if p6_d < 0 then p7_ng <= '1'; else p7_ng <= '0'; end if;
            p7_sel <= p6_sel; p7_z <= p6_z;
            -- 8: key, ramp base
            if p7_sel = '1' then p8_kr <= resize(p7_dn, 14) - resize(l_th(1), 14);
            else p8_kr <= resize(p7_dn, 14) - resize(l_th(0), 14); end if;
            if p7_xi > 190 then p8_rm <= (others => '0');
            elsif p7_xi < -65 then p8_rm <= (others => '1');
            else p8_rm <= unsigned(resize(190 - p7_xi, 8)); end if;
            p8_w <= p7_w; p8_ng <= p7_ng; p8_ra <= p7_ra; p8_z <= p7_z;
            -- 9: key 0..16, ramp product; background difference (middle - zone table)
            v_k := resize(shift_right(p8_kr, 2), 12);
            if v_k < 0 then p9_kk <= (others => '0'); elsif v_k > 16 then p9_kk <= to_unsigned(16, 5); else p9_kk <= unsigned(v_k(4 downto 0)); end if;
            if v_k < 8 then p9_kl <= '1'; else p9_kl <= '0'; end if;
            p9_rp <= p8_rm * l_rk;
            if p8_z = "01" then v_z := l_col(0); elsif p8_z = "10" then v_z := l_col(2); else v_z := l_col(1); end if;
            v_n := l_col(1);
            p9_df(0) <= signed('0' & v_n(23 downto 16)) - signed('0' & v_z(23 downto 16));
            p9_df(1) <= resize(signed(v_n(15 downto 8)), 9) - resize(signed(v_z(15 downto 8)), 9);
            p9_df(2) <= resize(signed(v_n(7 downto 0)), 9) - resize(signed(v_z(7 downto 0)), 9);
            p9_w <= p8_w; p9_ng <= p8_ng; p9_ra <= p8_ra; p9_z <= p8_z;
            -- 10: fill colour from the key; ramp terms (pink side only); background products;
            --     rim ROM address issued
            p10_fc <= C_BN_FILL(to_integer(p9_kk));
            if p9_kl = '1' then
                p10_r5 <= resize(shift_left(resize(p9_rp, 14), 2) + resize(p9_rp, 14), 14);
                p10_r7 <= resize(shift_left(resize(p9_rp, 14), 3) - resize(p9_rp, 14), 14);
            else
                p10_r5 <= (others => '0'); p10_r7 <= (others => '0');
            end if;
            for c in 0 to 2 loop p10_bp(c) <= resize(p9_df(c) * signed('0' & p9_w), 15); end loop;
            p10_ng <= p9_ng; p10_z <= p9_z;
            r_d_a <= p9_ra;
            -- 11: fill; background (zone table re-selected) + product
            p11_fi(0) <= signed('0' & p10_fc(23 downto 16)) - signed(resize(p10_r5(13 downto 8), 9));
            p11_fi(1) <= resize(signed(p10_fc(15 downto 8)), 9) - signed(resize(p10_r7(13 downto 8), 9));
            p11_fi(2) <= resize(signed(p10_fc(7 downto 0)), 9) + signed(resize(p10_r5(13 downto 9), 9));
            if p10_z = "01" then v_z := l_col(0); elsif p10_z = "10" then v_z := l_col(2); else v_z := l_col(1); end if;
            p11_bg(0) <= signed('0' & v_z(23 downto 16)) + resize(shift_right(p10_bp(0), 4), 9);
            p11_bg(1) <= resize(signed(v_z(15 downto 8)), 9) + resize(shift_right(p10_bp(1), 4), 9);
            p11_bg(2) <= resize(signed(v_z(7 downto 0)), 9) + resize(shift_right(p10_bp(2), 4), 9);
            p11_ng <= p10_ng;
            -- 12: under colour; rim ROM data to fabric
            for c in 0 to 2 loop
                if p11_ng = '1' then p12_un(c) <= p11_fi(c); else p12_un(c) <= p11_bg(c); end if;
            end loop;
            p12_q <= r_d_q;
            -- 13: (16 - alpha) * under
            for c in 0 to 2 loop
                p13_pr(c) <= resize(p12_un(c) * signed('0' & (16 - unsigned(p12_q(4 downto 0)))), 15);
            end loop;
            p13_q <= p12_q(31 downto 8);
            -- 14: out = P + product >> 4
            for c in 0 to 2 loop
                p14_o(c) <= resize(signed(p13_q(23 - 8 * c downto 16 - 8 * c)), 10) + resize(shift_right(p13_pr(c), 4), 10);
            end loop;
            -- 15: 10-bit; line texture on luma; chroma sign flipped for Colours = Negative
            p15_o(0) <= shift_left(resize(p14_o(0), 12) + resize(l_tx, 12), 2);
            for c in 1 to 2 loop
                if s_neg = '1' then p15_o(c) <= 512 - shift_left(resize(p14_o(c), 12), 2);
                else p15_o(c) <= 512 + shift_left(resize(p14_o(c), 12), 2); end if;
            end loop;
            -- 16: clamps
            if p15_o(0) < 0 then p16_y <= (others => '0'); elsif p15_o(0) > 1020 then p16_y <= to_unsigned(1020, 10); else p16_y <= unsigned(p15_o(0)(9 downto 0)); end if;
            if p15_o(1) < 4 then p16_u <= to_unsigned(4, 10); elsif p15_o(1) > 1020 then p16_u <= to_unsigned(1020, 10); else p16_u <= unsigned(p15_o(1)(9 downto 0)); end if;
            if p15_o(2) < 4 then p16_v <= to_unsigned(4, 10); elsif p15_o(2) > 1020 then p16_v <= to_unsigned(1020, 10); else p16_v <= unsigned(p15_o(2)(9 downto 0)); end if;
            -- 17: black outside the generated avid
            if s_gav_sr(14) = '0' then
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            else
                s_y17 <= p16_y; s_u17 <= p16_u; s_v17 <= p16_v;
            end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums, the 8-delay in block RAM), luma [1 2 1]/4 delayed to the box's
    -- centre (4 px lag), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ys : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 18: running 8-sums; 19: output
            s_bxu <= s_bxu + s_u17 - unsigned(ru_q2);
            s_bxv <= s_bxv + s_v17 - unsigned(rv_q2);
            s_bxud <= s_bxu; s_bxvd <= s_bxv;
            v_sc := resize(s_bxu, 15) + s_bxud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bxv, 15) + s_bxvd; s_v19 <= v_sc(13 downto 4);
            s_ya(0) <= s_y17;
            for i in 1 to 5 loop s_ya(i) <= s_ya(i - 1); end loop;
            v_ys := resize(s_ya(3), 12) + shift_left(resize(s_ya(4), 12), 1) + resize(s_ya(5), 12);
            s_y19 <= v_ys(11 downto 2);
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
    data_out.hsync_n <= rs_q2(3);
    data_out.vsync_n <= rs_q2(2);
    data_out.field_n <= rs_q2(1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture bottleneck;
