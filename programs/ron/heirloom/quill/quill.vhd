-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Quill v0.1.0 -- Heirloom (phase 3, video clips), rebuilt from the clip WIN_20230910_00_11_50 at 7.0 s.
--
-- The still: blue feather plumes on black.  A full-blue spine band at the top is crowded with wavy cyan /
-- magenta stripes; below it eight blue quills hang between round-topped black tongues, bow a little to the
-- right, then sweep down and to the left, converging on the left edge.  Dotted cyan barbs with magenta fringes
-- run down every quill; far from the spine the blue is gone and the barbs carry on as dense green / red lines.
-- Every colour is a saturated binary RGB: three hard keys.  In the clip the spine slides down (~140 lines/s)
-- while the key level drops, so the quills retract toward the spine.
--
-- The patch, as fitted (quill.py has the tables, .heirloom_work/rb_quill/notes.md every number):
--   phi    = (x + 130) k(y)                 a VCO hard-synced 130 px before the picture (the HD back porch)
--   k(y)   = k0 2^(g E(a)), a = |y - spine| exponential FM by a parabola in the distance from the spine:
--            E = 2.29e-6 (a - 220)^2 + 0.0011 max(0, 190 - a)  (rms 1.7 % on the measured periods)
--   B      = body key  a gb + tilt + noise < HB[frac phi]   (three plumes per period, of different reach)
--   G, R   = barbs: windows round six line phases, dotted by psi = 0.042 d (n + line phase) (horizontal
--            dashes whose rows fan like the field); R is the window just above each dash
--   band   = near the spine and in the far field: a texture key on frac(d/44 + 0.45 WV(phi) + 0.45 noise)
--   spine  = Spine knob + 380 sin(LFO); the same LFO lowers the key level as the spine descends
--
--   K1  Quills    plume count across, x1/3 .. x3 (the still at 512)
--   K2  Curl      FM depth: splayed (0) .. straight (256) .. the still (512) .. x3, a tight fan (1023)
--   K3  Spine     the spine's position (the centre of its sweep)
--   K4  Sweep     bipolar LFO speed of the spine sweep (still in the centre +-8; 700 = the clip)
--   K5  Barbs     barb width and dot density: faint sparse dots .. the still .. fat solid barbs
--   K6  Warp      how far the camera (or the Osc) bends the quills and grows the plumes
--   S7  Mirror    the sync point mirrored: quills facing a central seam
--   S8  Spines    One / Three: the spine distance folded into three stacked feather rows
--   S9  Palette   Ice (the still) / Ember
--   S10 Invert    the body key inverted: black quills on a blue ground
--   S11 Source    Video / Osc
--   P12 PLUMAGE   the key level: a thin spine band with stubs (0) .. the still (512) .. a blue flood of barbs
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 and sine ROMs), a per-line engine
-- kicked at each line start (computes the NEXT line on one serial multiplier: the FM exponent, k, the
-- line-start phase, the key levels, the barb dot phases), a 15-stage pixel path.  Virtual 1080-line
-- coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (Y, U and V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes, their 8-back taps in two block-RAM rings, with the running-sum resync; luma
-- 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.quill_pkg.all;
use work.all;

architecture quill of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_KB  : integer := 11733;       -- 1 / 715 cycles per virtual px, Q24 per h (half px)
    constant C_TH0 : integer := 14289428;    -- LFO phase at power-up: the still's spine (y 45), Q24 turns

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
    -- clamp a signed value to an exponent range [-768, hi]
    function f_eclamp(v : signed(13 downto 0); hi : integer) return signed is
    begin
        if v < -768 then return to_signed(-768, 12);
        elsif v > hi then return to_signed(hi, 12);
        else return v(11 downto 0); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a, r_lx_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_lx_q : unsigned(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(9 downto 0) := (others => '0');
    signal r_sn_q : signed(15 downto 0) := (others => '0');
    signal r_f_a  : unsigned(7 downto 0) := (others => '0');
    signal r_f_q  : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_spn, s_pal, s_inv, s_src : std_logic := '0';

    -- per-field scratch
    signal s_th   : unsigned(23 downto 0) := to_unsigned(C_TH0, 24);   -- spine LFO phase, Q24 turns
    signal s_om   : unsigned(23 downto 0) := (others => '0');          -- its rate per field (magnitude)
    signal s_omn  : std_logic := '0';
    signal z_sq   : signed(15 downto 0) := (others => '0');
    signal z_sqa  : unsigned(15 downto 0) := (others => '0');
    signal z_sqn  : std_logic := '0';
    signal z_yst  : unsigned(9 downto 0) := (others => '0');
    signal z_pt   : unsigned(9 downto 0) := (others => '0');
    signal z_pe, z_pd : signed(12 downto 0) := (others => '0');
    signal z_e    : signed(13 downto 0) := (others => '0');
    signal z_ec   : signed(11 downto 0) := (others => '0');
    signal z_e1, z_ekb, z_ed, z_ew : signed(11 downto 0) := (others => '0');
    signal z_m1, z_md, z_mw : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_s1, z_sd, z_sw : signed(2 downto 0) := (others => '0');
    signal z_sx   : signed(2 downto 0) := (others => '0');
    signal z_yc   : unsigned(9 downto 0) := (others => '0');
    signal z_av   : unsigned(19 downto 0) := (others => '0');
    signal z_k6   : unsigned(19 downto 0) := (others => '0');

    -- per-field constants
    signal c_ys   : signed(11 downto 0) := to_signed(45, 12);          -- spine (virtual line)
    signal c_k0   : unsigned(17 downto 0) := to_unsigned(C_KB, 18);    -- VCO base frequency, Q24 per h
    signal c_gb, c_gt, c_gf : unsigned(11 downto 0) := to_unsigned(256, 12);   -- level gains, Q8
    signal c_dty  : unsigned(7 downto 0) := to_unsigned(82, 8);        -- barb dot duty, Q8
    signal c_wg   : unsigned(7 downto 0) := to_unsigned(15, 8);        -- barb half-width, 1/1024 period
    signal c_gq   : signed(10 downto 0) := to_signed(256, 11);         -- curl g, Q8
    signal c_gqa  : unsigned(9 downto 0) := to_unsigned(256, 10);      -- |g|
    signal c_wg3  : unsigned(7 downto 0) := to_unsigned(12, 8);        -- 0.75 barb half-width
    signal c_gw   : unsigned(16 downto 0) := (others => '0');          -- warp per luma step, Q24 periods
    signal c_x0p  : unsigned(12 downto 0) := to_unsigned(260, 13);     -- x' + sync offset at pixel 0 (h)
    signal c_ctr  : unsigned(10 downto 0) := to_unsigned(959, 11);     -- mirror: last pixel stepping down

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld1, z_ld : std_logic := '0';
    signal z_oh  : std_logic_vector(9 downto 0) := (others => '0');

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
    signal e_vy   : unsigned(19 downto 0) := (others => '0');           -- next line, Q8 virtual lines
    signal e_m    : unsigned(11 downto 0) := (others => '0');           -- Spines: (y - spine) mod 360
    signal e_dv, e_d : signed(12 downto 0) := (others => '0');
    signal e_r3   : unsigned(10 downto 0) := (others => '0');
    signal e_rneg : std_logic := '0';
    signal e_a    : unsigned(10 downto 0) := (others => '0');
    signal e_u    : signed(12 downto 0) := (others => '0');
    signal e_c    : signed(12 downto 0) := (others => '0');
    signal e_c72  : unsigned(7 downto 0) := (others => '0');
    signal e_e8   : unsigned(12 downto 0) := (others => '0');
    signal e_eg   : signed(13 downto 0) := (others => '0');
    signal e_sh   : signed(3 downto 0) := (others => '0');
    signal e_k    : unsigned(21 downto 0) := (others => '0');
    signal e_at, e_af : unsigned(10 downto 0) := (others => '0');
    signal e_x1, e_x2, e_x3 : signed(12 downto 0) := (others => '0');
    signal e_y1, e_y2, e_y3 : signed(12 downto 0) := (others => '0');
    signal e_lt, e_lf : unsigned(6 downto 0) := (others => '0');
    signal e_wd   : unsigned(6 downto 0) := (others => '0');
    signal e_l1, e_l2 : unsigned(8 downto 0) := (others => '0');
    signal e_rr   : unsigned(12 downto 0) := (others => '0');
    signal e_fr   : unsigned(12 downto 0) := (others => '0');
    signal e_lv   : unsigned(8 downto 0) := (others => '0');
    signal e_cd   : unsigned(17 downto 0) := (others => '0');           -- d * 43, two's complement (Q10 periods)
    signal e_ua   : unsigned(11 downto 0) := (others => '0');
    signal e_lr   : unsigned(7 downto 0) := (others => '0');
    signal e_sel  : unsigned(4 downto 0) := (others => '0');
    signal e_pre, e_ld1, e_ld : std_logic := '0';
    signal e_oh   : std_logic_vector(16 downto 0) := (others => '0');
    signal e_fa   : unsigned(21 downto 0) := (others => '0');
    signal e_fb   : unsigned(15 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    -- next-line staging (n_*), loaded into the active-line registers (l_*) and the beam at the line start
    signal n_ph0  : unsigned(27 downto 0) := (others => '0');
    signal n_inc  : unsigned(23 downto 0) := (others => '0');
    signal n_as   : unsigned(10 downto 0) := (others => '0');
    signal n_ltp  : std_logic := '0';
    signal n_dl   : unsigned(7 downto 0) := (others => '0');
    signal n_rth  : unsigned(7 downto 0) := (others => '0');
    signal n_lev, n_lvc : unsigned(7 downto 0) := (others => '0');
    signal n_lfp  : std_logic := '0';
    signal n_t3h  : unsigned(7 downto 0) := (others => '0');
    signal n_fth  : unsigned(6 downto 0) := (others => '0');
    signal n_cdf  : unsigned(7 downto 0) := (others => '0');
    signal n_cg0, n_cg1, n_cg2, n_cg3, n_cg4, n_cg5 : unsigned(7 downto 0) := (others => '0');
    signal n_pb   : unsigned(7 downto 0) := (others => '0');
    signal n_seed : unsigned(15 downto 0) := to_unsigned(1, 16);
    signal l_as   : unsigned(10 downto 0) := (others => '0');
    signal l_ltp  : std_logic := '0';
    signal l_lfp  : std_logic := '0';
    signal l_dl   : unsigned(7 downto 0) := (others => '0');
    signal l_rth  : unsigned(7 downto 0) := (others => '0');
    signal l_lev, l_lvc : unsigned(7 downto 0) := (others => '0');
    signal l_t3h  : unsigned(7 downto 0) := (others => '0');
    signal l_fth  : unsigned(6 downto 0) := (others => '0');
    signal l_cdf  : unsigned(7 downto 0) := (others => '0');
    signal l_cg0, l_cg1, l_cg2, l_cg3, l_cg4, l_cg5 : unsigned(7 downto 0) := (others => '0');
    signal l_pb   : unsigned(7 downto 0) := (others => '0');

    -- beam: pixel count (stage 1); phase / tilt DDAs and the noise LFSR (stage 5), mirrored on S7
    signal b_gsr, b_fsr : std_logic_vector(3 downto 0) := (others => '0');
    signal b_ph  : unsigned(27 downto 0) := (others => '0');
    signal b_inc : unsigned(23 downto 0) := (others => '0');
    signal b_dn  : std_logic := '0';
    signal b_cl  : unsigned(6 downto 0) := (others => '0');             -- 16-virtual-px cell index (tilt)
    signal b_nz  : unsigned(15 downto 0) := to_unsigned(1, 16);
    signal b_cc  : unsigned(5 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(16 downto 0) := (others => '0');
    signal p5_lw  : unsigned(27 downto 0) := (others => '0');
    signal p6_ph  : unsigned(27 downto 0) := (others => '0');
    signal p6_bsh : unsigned(11 downto 0) := (others => '0');
    signal p6_ntl, p7_ntl : signed(12 downto 0) := (others => '0');
    signal p6_nz, p7_nz : unsigned(3 downto 0) := (others => '0');
    signal p7_n   : unsigned(3 downto 0) := (others => '0');
    signal p7_t   : signed(12 downto 0) := (others => '0');
    signal p8_nc, p9_nc : unsigned(7 downto 0) := (others => '0');
    signal p8_bl, p9_bl : signed(12 downto 0) := (others => '0');
    signal p8_gt, p9_gt, p10_gt : unsigned(7 downto 0) := (others => '0');
    signal p8_dp, p9_dp, p10_dp : unsigned(8 downto 0) := (others => '0');
    signal p8_sn, p9_sn : unsigned(7 downto 0) := (others => '0');
    signal p9_f   : std_logic_vector(31 downto 0) := (others => '0');
    signal p10_b  : std_logic := '0';
    signal p10_ae : unsigned(6 downto 0) := (others => '0');
    signal p10_s8 : unsigned(7 downto 0) := (others => '0');
    signal p10_st : unsigned(7 downto 0) := (others => '0');
    signal p11_g1, p11_g2, p11_g3, p11_f1, p11_t1, p11_t2, p11_t3, p11_b : std_logic := '0';
    signal p12_code : unsigned(2 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- the boxes' 8-back taps come from two block-RAM rings (read pointer = write - 6, then a fabric register)
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal o_ra, o_rb : t_ring := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of o_ra : signal is true;
    attribute no_rw_check of o_rb : signal is true;
    signal o_wp : unsigned(7 downto 0) := (others => '0');
    signal o_rp : unsigned(7 downto 0) := to_unsigned(250, 8);
    signal o_aq, o_aq2, o_bq, o_bq2 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
            r_ex_q <= C_QL_EXP2(to_integer(r_ex_a));
            r_lx_q <= C_QL_EXP2(to_integer(r_lx_a));
            r_sn_q <= C_QL_SIN(to_integer(r_sn_a));
            r_f_q  <= C_QL_ROM(to_integer(r_f_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_fa : unsigned(31 downto 0);
        variable v_fb : unsigned(15 downto 0);
        variable v_e  : signed(13 downto 0);
        variable v_p  : signed(12 downto 0);
        variable v_g  : unsigned(11 downto 0);
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

            -- K4 Sweep: bipolar, +-8 deadband, |v| - 8 (square law on the multiplier)
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
                s_mir <= registers_in(6)(0);
                s_spn <= registers_in(6)(1);
                s_pal <= registers_in(6)(2);
                s_inv <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
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

            -- serial products: slot k loads at zc = 33 + 24 k, 16 iterations, result read at 50 + 24 k.  The
            -- slot number is set at 30 + 24 k, decoded one-hot at 31, the operands fetched at 32.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4); z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4); z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4); z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4); z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4); z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4); z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4); z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 4); z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            -- operand fetch: registered one-hot, then a flat AND-OR (no enables, no decode on the path)
            z_ld1 <= z_pre; z_ld <= z_ld1;
            for i in 0 to 9 loop
                if z_sel = i then z_oh(i) <= '1'; else z_oh(i) <= '0'; end if;
            end loop;
            v_fa := (others => '0'); v_fb := (others => '0');
            if z_oh(0) = '1' then v_fa := v_fa or to_unsigned(380, 32);              v_fb := v_fb or z_sqa;                   end if;  -- spine swing
            if z_oh(1) = '1' then v_fa := v_fa or to_unsigned(435, 32);              v_fb := v_fb or z_sqa;                   end if;  -- level swing
            if z_oh(2) = '1' then v_fa := v_fa or resize(s_g3(12 downto 3), 32);     v_fb := v_fb or to_unsigned(175, 16);    end if;  -- spine centre
            if z_oh(3) = '1' then v_fa := v_fa or resize(s_vm, 32);                  v_fb := v_fb or resize(s_vm, 16);        end if;  -- speed^2
            if z_oh(4) = '1' then v_fa := v_fa or resize(z_av, 32);                  v_fb := v_fb or to_unsigned(216, 16);    end if;
            if z_oh(5) = '1' then v_fa := v_fa or resize(s_g6(12 downto 3), 32);     v_fb := v_fb or resize(s_g6(12 downto 3), 16); end if;  -- warp^2
            if z_oh(6) = '1' then v_fa := v_fa or resize(z_k6, 32);                  v_fb := v_fb or to_unsigned(6567, 16);   end if;
            if z_oh(7) = '1' then v_fa := v_fa or to_unsigned(C_KB, 32);             v_fb := v_fb or z_m1;                    end if;  -- k0
            if z_oh(8) = '1' then v_fa := v_fa or to_unsigned(82, 32);               v_fb := v_fb or z_md;                    end if;  -- dot duty
            if z_oh(9) = '1' then v_fa := v_fa or to_unsigned(61, 32);               v_fb := v_fb or z_mw;                    end if;  -- barb width
            z_fa <= v_fa; z_fb <= v_fb;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- exp2 mantissa as a Q8 gain: m >> (7 - sh), one shared shifter for gb, gt and gf
            case to_integer(z_sx) is
                when -3     => v_g := resize(shift_right(r_ex_q, 10), 12);
                when -2     => v_g := resize(shift_right(r_ex_q, 9), 12);
                when -1     => v_g := resize(shift_right(r_ex_q, 8), 12);
                when 0      => v_g := resize(shift_right(r_ex_q, 7), 12);
                when 1      => v_g := resize(shift_right(r_ex_q, 6), 12);
                when others => v_g := resize(shift_right(r_ex_q, 5), 12);
            end case;
            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- the spine LFO (once per field) and its sine; K1 / K5 exponents
                    when 20 =>
                        if s_omn = '1' then s_th <= s_th - s_om; else s_th <= s_th + s_om; end if;
                        v_s := signed(resize(s_g1(12 downto 3), 11)) - 512;          -- quills: +-log2(3) octaves
                        z_e1 <= resize(v_s - shift_right(v_s, 2) + shift_right(v_s, 5), 12);
                        v_s := signed(resize(s_g5(12 downto 3), 11)) - 512;          -- barbs: +-1.5 octaves
                        z_ekb <= resize(v_s - shift_right(v_s, 2), 12);
                        c_gq <= signed(resize(s_g2(12 downto 3), 11)) - 256;          -- curl g (Q8)
                    when 21 =>
                        if c_gq < 0 then c_gqa <= unsigned(-c_gq(9 downto 0)); else c_gqa <= unsigned(c_gq(9 downto 0)); end if;
                        r_sn_a <= s_th(23 downto 14);
                        r_ex_a <= unsigned(z_e1(7 downto 0)); z_s1 <= z_e1(10 downto 8);
                    when 23 =>
                        z_sq <= r_sn_q;
                        z_m1 <= r_ex_q;
                    when 24 =>
                        if z_sq < 0 then z_sqa <= unsigned(-z_sq); z_sqn <= '1';
                        else z_sqa <= unsigned(z_sq); z_sqn <= '0'; end if;
                        -- x' + sync offset at pixel 0 (Mirror: the line starts at the mirror of the left
                        -- edge), the tilt ramp start and step, the mirror reversal point
                        if s_mir = '1' then c_x0p <= to_unsigned(4100, 13) - resize(s_x0, 13);
                        else c_x0p <= resize(s_x0, 13) + 260; end if;
                        c_ctr <= resize(s_w(11 downto 1), 11) - 1;

                    -- slot results
                    when 50 => z_yst <= resize(f_acc(24 downto 15), 10);               -- 380 |sin| >> 15
                    when 74 => z_pt <= resize(f_acc(24 downto 15), 10);                -- 435 |sin| >> 15
                    when 75 =>                    -- effective level: P12 - 349 - 435 sin
                        if z_sqn = '1' then v_p := signed(resize(s_gp(12 downto 3), 13)) - 349 + signed(resize(z_pt, 13));
                        else v_p := signed(resize(s_gp(12 downto 3), 13)) - 349 - signed(resize(z_pt, 13)); end if;
                        z_pe <= v_p;
                    when 76 => z_pd <= z_pe - 512;
                    -- level exponents (Q8 octaves), each clamped and looked up: address at k, data at k + 2
                    when 77 =>                    -- body gain: -1.156 pd (pd > 0) or -pd
                        if z_pd > 0 then v_e := -(resize(z_pd, 14) + shift_right(resize(z_pd, 14), 3) + shift_right(resize(z_pd, 14), 5));
                        else v_e := -resize(z_pd, 14); end if;
                        z_e <= v_e;
                        -- dot duty and barb width exponents: +0.39 pd, +0.625 max(pd, 0)
                        v_e := shift_right(resize(z_pd, 14), 2) + shift_right(resize(z_pd, 14), 3) + shift_right(resize(z_pd, 14), 6);
                        z_ed <= v_e(11 downto 0);
                        if z_pd > 0 then
                            v_e := shift_right(resize(z_pd, 14), 1) + shift_right(resize(z_pd, 14), 3);
                            z_ew <= v_e(11 downto 0);
                        else z_ew <= (others => '0'); end if;
                    when 78 =>
                        z_ec <= f_eclamp(z_e, 767);
                    when 79 =>
                        r_ex_a <= unsigned(z_ec(7 downto 0)); z_sx <= z_ec(10 downto 8);
                        -- band gain: -0.25 pd (pd > 0) or -0.844 pd
                        if z_pd > 0 then v_e := -shift_right(resize(z_pd, 14), 2);
                        else v_e := -(resize(z_pd, 14) - shift_right(resize(z_pd, 14), 3) - shift_right(resize(z_pd, 14), 5)); end if;
                        z_e <= v_e;
                    when 80 =>
                        z_ec <= f_eclamp(z_e, 767);
                    when 81 =>                    -- gb = m >> (7 - sh)
                        c_gb <= v_g;
                        r_ex_a <= unsigned(z_ec(7 downto 0)); z_sx <= z_ec(10 downto 8);
                        -- far gain: +0.156 pd
                        z_e <= shift_right(resize(z_pd, 14), 3) + shift_right(resize(z_pd, 14), 5);
                    when 82 =>
                        z_ec <= f_eclamp(z_e, 767);
                    when 83 =>                    -- gt
                        c_gt <= v_g;
                        r_ex_a <= unsigned(z_ec(7 downto 0)); z_sx <= z_ec(10 downto 8);
                        z_e <= resize(z_ekb, 14) + resize(z_ed, 14);                   -- dot duty: barbs + level
                    when 84 =>
                        z_ec <= f_eclamp(z_e, 767);
                    when 85 =>                    -- gf
                        c_gf <= v_g;
                        r_ex_a <= unsigned(z_ec(7 downto 0)); z_sd <= z_ec(10 downto 8);
                        z_e <= resize(z_ekb, 14) + resize(z_ew, 14);                   -- barb width: barbs + level
                    when 86 =>
                        z_ec <= f_eclamp(z_e, 767);
                    when 87 =>
                        z_md <= r_ex_q;
                        r_ex_a <= unsigned(z_ec(7 downto 0)); z_sw <= z_ec(10 downto 8);
                    when 89 =>
                        z_mw <= r_ex_q;
                    when 98 =>                    -- spine = centre + 380 sin
                        z_yc <= resize(f_acc(17 downto 8), 10);
                    when 99 =>
                        if z_sqn = '1' then c_ys <= signed(resize(z_yc, 12)) - signed(resize(z_yst, 12));
                        else c_ys <= signed(resize(z_yc, 12)) + signed(resize(z_yst, 12)); end if;
                    when 122 => z_av <= f_acc(19 downto 0);                               -- speed^2
                    when 146 =>                   -- LFO rate: speed^2 * 216 >> 8 (Q24 turns per field)
                        s_om <= f_acc(31 downto 8); s_omn <= s_vneg;
                    when 170 => z_k6 <= resize(f_acc(19 downto 4), 20);                   -- warp^2 >> 4
                    when 194 => c_gw <= f_acc(28 downto 12);                              -- (warp^2 >> 4) * 6567 >> 12
                    when 218 =>                   -- k0 = C_KB * m1 >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_s1) is
                            when -2     => c_k0 <= resize(f_acc(31 downto 17), 18);
                            when -1     => c_k0 <= resize(f_acc(31 downto 16), 18);
                            when 0      => c_k0 <= resize(f_acc(31 downto 15), 18);
                            when others => c_k0 <= resize(f_acc(31 downto 14), 18);
                        end case;
                    when 242 =>                   -- dot duty = min(230, 82 m >> (15 - sh)), sh = -3 .. 2
                        case to_integer(z_sd) is
                            when -3     => z_av <= resize(f_acc(31 downto 18), 20);
                            when -2     => z_av <= resize(f_acc(31 downto 17), 20);
                            when -1     => z_av <= resize(f_acc(31 downto 16), 20);
                            when 0      => z_av <= resize(f_acc(31 downto 15), 20);
                            when 1      => z_av <= resize(f_acc(31 downto 14), 20);
                            when others => z_av <= resize(f_acc(31 downto 13), 20);
                        end case;
                    when 243 =>
                        if z_av > 230 then c_dty <= to_unsigned(230, 8); else c_dty <= z_av(7 downto 0); end if;
                    when 266 =>                   -- barb half-width = 61 m >> (17 - sh) (1/1024 period)
                        case to_integer(z_sw) is
                            when -3     => c_wg <= resize(f_acc(31 downto 20), 8);
                            when -2     => c_wg <= resize(f_acc(31 downto 19), 8);
                            when -1     => c_wg <= resize(f_acc(31 downto 18), 8);
                            when 0      => c_wg <= resize(f_acc(31 downto 17), 8);
                            when 1      => c_wg <= resize(f_acc(31 downto 16), 8);
                            when others => c_wg <= resize(f_acc(31 downto 15), 8);
                        end case;
                    when 267 =>
                        c_wg3 <= c_wg - shift_right(c_wg, 2);
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
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17): the FM exponent and k, the line-start phase, the three level
    -- distances, the key thresholds, the barb dot phases, the texture phase and
    -- the noise seed; ~300 clocks, so it fits SD's 858-clock line
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_cl : unsigned(10 downto 0);
        variable v_r  : unsigned(8 downto 0);
        variable v_t  : signed(13 downto 0);
        variable v_fa : unsigned(21 downto 0);
        variable v_fb : unsigned(15 downto 0);
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
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            -- serial products: slot k is set at its state P (e_pre), decoded one-hot at P + 1, the operands
            -- fetched by a flat AND-OR at P + 2, loaded at P + 3; 16 iterations, the result is read at P + 20
            e_pre <= '0';
            case to_integer(e_lc) is
                when 17   => e_sel <= to_unsigned(0, 5);  e_pre <= '1';
                when 37  => e_sel <= to_unsigned(1, 5);  e_pre <= '1';
                when 58  => e_sel <= to_unsigned(2, 5);  e_pre <= '1';
                when 81  => e_sel <= to_unsigned(3, 5);  e_pre <= '1';
                when 102  => e_sel <= to_unsigned(4, 5);  e_pre <= '1';
                when 122 => e_sel <= to_unsigned(5, 5);  e_pre <= '1';
                when 142 => e_sel <= to_unsigned(6, 5);  e_pre <= '1';
                when 162 => e_sel <= to_unsigned(7, 5);  e_pre <= '1';
                when 185 => e_sel <= to_unsigned(8, 5);  e_pre <= '1';
                when 205 => e_sel <= to_unsigned(9, 5);  e_pre <= '1';
                when 225 => e_sel <= to_unsigned(10, 5); e_pre <= '1';
                when 245 => e_sel <= to_unsigned(11, 5); e_pre <= '1';
                when 265 => e_sel <= to_unsigned(12, 5); e_pre <= '1';
                when 285 => e_sel <= to_unsigned(13, 5); e_pre <= '1';
                when 305 => e_sel <= to_unsigned(14, 5); e_pre <= '1';
                when 325 => e_sel <= to_unsigned(15, 5); e_pre <= '1';
                when 345 => e_sel <= to_unsigned(16, 5); e_pre <= '1';
                when others => null;
            end case;
            e_ld1 <= e_pre; e_ld <= e_ld1;
            for i in 0 to 16 loop
                if e_sel = i then e_oh(i) <= '1'; else e_oh(i) <= '0'; end if;
            end loop;
            v_fa := (others => '0'); v_fb := (others => '0');
            if e_oh(0)  = '1' then v_fa := v_fa or resize(e_ua, 22);              v_fb := v_fb or resize(e_ua, 16);         end if;  -- (a - 220)^2
            if e_oh(1)  = '1' then v_fa := v_fa or m_acc(21 downto 0);            v_fb := v_fb or to_unsigned(38, 16);      end if;  -- * 38
            if e_oh(2)  = '1' then v_fa := v_fa or resize(e_e8, 22);              v_fb := v_fb or resize(c_gqa, 16);        end if;  -- E8 |g|
            if e_oh(3)  = '1' then v_fa := v_fa or resize(c_k0, 22);              v_fb := v_fb or r_lx_q;                   end if;  -- k0 m
            if e_oh(4)  = '1' then v_fa := v_fa or e_k;                           v_fb := v_fb or resize(c_x0p, 16);        end if;  -- k X0
            if e_oh(5)  = '1' then v_fa := v_fa or resize(e_a, 22);               v_fb := v_fb or resize(c_gb, 16);         end if;  -- a gb
            if e_oh(6)  = '1' then v_fa := v_fa or resize(e_a, 22);               v_fb := v_fb or resize(c_gt, 16);         end if;  -- a gt
            if e_oh(7)  = '1' then v_fa := v_fa or resize(e_a, 22);               v_fb := v_fb or resize(c_gf, 16);         end if;  -- a gf
            if e_oh(8)  = '1' then v_fa := v_fa or resize(c_dty, 22);             v_fb := v_fb or resize(e_wd, 16);         end if;  -- duty wd
            if e_oh(9)  = '1' then v_fa := v_fa or resize(e_cd, 22);              v_fb := v_fb or to_unsigned(40, 16);      end if;  -- dot phases
            if e_oh(10) = '1' then v_fa := v_fa or resize(e_cd, 22);              v_fb := v_fb or to_unsigned(139, 16);     end if;
            if e_oh(11) = '1' then v_fa := v_fa or resize(e_cd, 22);              v_fb := v_fb or to_unsigned(148, 16);     end if;
            if e_oh(12) = '1' then v_fa := v_fa or resize(e_cd, 22);              v_fb := v_fb or to_unsigned(206, 16);     end if;
            if e_oh(13) = '1' then v_fa := v_fa or resize(e_cd, 22);              v_fb := v_fb or to_unsigned(220, 16);     end if;
            if e_oh(14) = '1' then v_fa := v_fa or resize(e_cd, 22);              v_fb := v_fb or to_unsigned(234, 16);     end if;
            if e_oh(15) = '1' then v_fa := v_fa or resize(unsigned(resize(e_d, 16)), 22); v_fb := v_fb or to_unsigned(1490, 16); end if;  -- d / 44
            if e_oh(16) = '1' then v_fa := v_fa or resize(e_vy(19 downto 10), 22); v_fb := v_fb or to_unsigned(40503, 16);  end if;  -- seed
            e_fa <= v_fa; e_fb <= v_fb;

            -- one shared clamp of the product >> 8 to 11 bits (the three level distances)
            if m_acc(31 downto 19) /= 0 then v_cl := (others => '1'); else v_cl := m_acc(18 downto 8); end if;
            if e_ld = '1' then
                m_acc <= (others => '0');
                m_a <= resize(e_fa, 32); m_b <= e_fb;
            else
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= m_a(30 downto 0) & '0';
                m_b <= '0' & m_b(15 downto 1);
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    -- d: virtual lines from the spine
                    e_dv <= signed(resize(e_vy(19 downto 8), 13)) - resize(c_ys, 13);
                when 2 =>                     -- Spines: r = d mod 360 (one shared subtract, ten clocks)
                    e_m <= unsigned(e_dv(11 downto 0) + 1800);
                when 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 =>
                    if e_m >= 360 then e_m <= e_m - 360; end if;
                when 13 =>                    -- 3 x the distance to the nearest of a row of spines 360 lines apart
                    v_r := e_m(8 downto 0);
                    if v_r < 180 then
                        e_r3 <= resize(v_r, 11) + shift_left(resize(v_r, 11), 1); e_rneg <= '0';
                    else
                        e_r3 <= resize(360 - v_r, 11) + shift_left(resize(360 - v_r, 11), 1); e_rneg <= '1';
                    end if;
                when 14 =>
                    if s_spn = '1' then
                        if e_rneg = '1' then e_d <= -signed(resize(e_r3, 13)); else e_d <= signed(resize(e_r3, 13)); end if;
                    else
                        e_d <= e_dv;
                    end if;
                when 15 =>
                    if e_d < 0 then v_t := -resize(e_d, 14); else v_t := resize(e_d, 14); end if;
                    if v_t > 2047 then e_a <= to_unsigned(2047, 11); else e_a <= unsigned(v_t(10 downto 0)); end if;
                when 16 =>
                    e_u <= signed(resize(e_a, 13)) - 220;
                    e_c <= 190 - signed(resize(e_a, 13));
                when 17 =>
                    if e_u < 0 then e_ua <= unsigned(-e_u(11 downto 0)); else e_ua <= unsigned(e_u(11 downto 0)); end if;
                    if e_c < 0 then e_c72 <= (others => '0');
                    else e_c72 <= resize(shift_right(shift_left(resize(unsigned(e_c(7 downto 0)), 14), 6) + shift_left(resize(unsigned(e_c(7 downto 0)), 14), 3), 8), 8); end if;
                when 57 =>                    -- E8 = (a - 220)^2 * 38 >> 16 + cusp (Q8 octaves)
                    e_e8 <= resize(m_acc(27 downto 16), 13) + resize(e_c72, 13);
                when 78 =>                    -- g E8 (the curl)
                    if c_gq < 0 then e_eg <= -signed(resize(m_acc(21 downto 8), 14));
                    else e_eg <= signed(resize(m_acc(21 downto 8), 14)); end if;
                when 79 =>                    -- clamp to -3 .. +5 octaves, exp2 lookup
                    if e_eg < -768 then v_t := to_signed(-768, 14);
                    elsif e_eg > 1279 then v_t := to_signed(1279, 14);
                    else v_t := e_eg; end if;
                    r_lx_a <= unsigned(v_t(7 downto 0));
                    e_sh <= v_t(11 downto 8);
                when 101 =>                    -- k = k0 * m >> (15 - sh)
                    case to_integer(e_sh) is
                        when -3     => e_k <= resize(m_acc(31 downto 18), 22);
                        when -2     => e_k <= resize(m_acc(31 downto 17), 22);
                        when -1     => e_k <= resize(m_acc(31 downto 16), 22);
                        when 0      => e_k <= resize(m_acc(31 downto 15), 22);
                        when 1      => e_k <= resize(m_acc(31 downto 14), 22);
                        when 2      => e_k <= resize(m_acc(31 downto 13), 22);
                        when 3      => e_k <= resize(m_acc(31 downto 12), 22);
                        when others => e_k <= resize(m_acc(31 downto 11), 22);
                    end case;
                when 102 =>                    -- per-pixel step = k * xs
                    case to_integer(s_xs) is
                        when 4      => n_inc <= resize(shift_left(resize(e_k, 24), 2), 24);
                        when 3      => n_inc <= resize(shift_left(resize(e_k, 24), 1), 24) + resize(e_k, 24);
                        when others => n_inc <= resize(shift_left(resize(e_k, 24), 1), 24);
                    end case;
                when 122 =>                   -- phase at pixel 0 = k * (x' + 130 px)
                    n_ph0 <= m_acc(27 downto 0);
                when 142 =>                   -- as = a gb (body), at = a gt (band), af = a gf (far field)
                    n_as <= v_cl;
                when 162 =>
                    e_at <= v_cl;
                when 182 =>
                    e_af <= v_cl;
                    e_x1 <= 190 - signed(resize(e_at, 13));
                    e_x3 <= 860 - signed(resize(e_at, 13));
                when 183 =>                   -- band 0.625 (190 - at), dot fade 0.1875 (860 - at), far 0.406 (af - 780)
                    e_y1 <= shift_right(e_x1, 1) + shift_right(e_x1, 3);
                    e_y3 <= shift_right(e_x3, 3) + shift_right(e_x3, 4);
                    e_x2 <= signed(resize(e_af, 13)) - 780;
                when 184 =>
                    e_y2 <= shift_right(e_x2, 2) + shift_right(e_x2, 3);
                    if e_y1 < 0 then e_lt <= (others => '0');
                    elsif e_y1 > 64 then e_lt <= to_unsigned(64, 7);
                    else e_lt <= unsigned(e_y1(6 downto 0)); end if;
                    if e_y3 < 0 then e_wd <= to_unsigned(16, 7);
                    elsif e_y3 > 48 then e_wd <= to_unsigned(64, 7);
                    else e_wd <= unsigned(e_y3(6 downto 0)) + 16; end if;
                when 185 =>
                    e_y2 <= e_y2 + shift_right(e_x2, 5);
                    if e_lt /= 0 then n_ltp <= '1'; else n_ltp <= '0'; end if;
                    e_rr <= shift_left(resize(e_wd, 13), 5) - resize(e_wd, 13);
                when 186 =>
                    if e_y2 < 0 then e_lf <= (others => '0');
                    elsif e_y2 > 64 then e_lf <= to_unsigned(64, 7);
                    else e_lf <= unsigned(e_y2(6 downto 0)); end if;
                    n_rth <= 255 - resize(shift_right(e_rr, 6), 8);
                when 187 =>
                    if e_lf /= 0 then n_lfp <= '1'; else n_lfp <= '0'; end if;
                    n_t3h <= to_unsigned(158, 8) + resize(e_lf, 8);
                    e_l1 <= shift_left(resize(e_lt, 9), 1);
                    e_l2 <= shift_left(resize(e_lf, 9), 1) + resize(e_lf, 9);
                    e_fr <= shift_left(resize(e_lf, 13), 5) + shift_left(resize(e_lf, 13), 4);
                when 188 =>
                    e_l2 <= e_l2 + resize(shift_right(e_lf, 2), 9);
                    e_fr <= e_fr + shift_left(resize(e_lf, 13), 1) + resize(e_lf, 13);
                when 189 =>                   -- texture level max(2 lt, 3.25 lf); far gate
                    if e_l1 > e_l2 then e_lv <= e_l1; else e_lv <= e_l2; end if;
                    n_fth <= resize(shift_right(e_fr, 6), 7) + 12;
                when 190 =>                   -- G below lev, R above 255 - 0.45 lev
                    n_lev <= e_lv(7 downto 0);
                    e_lr <= e_lv(8 downto 1) - resize(shift_right(e_lv, 4), 8) + resize(shift_right(e_lv, 6), 8);
                    -- cd = d * 43 (barb dot rate, Q10 periods), first two terms
                    e_cd <= unsigned(resize(shift_left(resize(e_d, 18), 5) + shift_left(resize(e_d, 18), 3), 18));
                when 191 =>
                    n_lvc <= 255 - e_lr;
                    e_cd <= e_cd + unsigned(resize(shift_left(resize(e_d, 18), 1) + resize(e_d, 18), 18));
                when 205 =>                   -- dot duty for the line = duty * wd >> 6
                    n_dl <= resize(shift_right(m_acc(15 downto 0), 6), 8);
                    n_cdf <= e_cd(9 downto 2);
                when 225 => n_cg0 <= m_acc(17 downto 10);        -- barb line dot phases: frac(cd * line phase), Q8
                when 245 => n_cg1 <= m_acc(17 downto 10);
                when 265 => n_cg2 <= m_acc(17 downto 10);
                when 285 => n_cg3 <= m_acc(17 downto 10);
                when 305 => n_cg4 <= m_acc(17 downto 10);
                when 325 => n_cg5 <= m_acc(17 downto 10);
                when 345 => n_pb <= m_acc(15 downto 8);          -- texture phase d / 44 (Q8, mod one cycle)
                when 365 =>                                       -- noise seed of the 4-line cell row
                    if (m_acc(15 downto 0) xor x"5A5A") = x"0000" then n_seed <= to_unsigned(1, 16);
                    else n_seed <= m_acc(15 downto 0) xor x"5A5A"; end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam.  Stage 1 only counts pixels (the line-buffer read address and the
    -- mirror point); the VCO phase, the tilt ramp and the noise LFSR (one step
    -- per 16 virtual px, screen-fixed) run at stage 5.  Mirror runs the phase and
    -- the tilt down to the centre and back up (x' = 960 + |x - 960|).  The
    -- engine's next-line values become the active-line registers at the start.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_cc : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px <= (others => '0');
            elsif g_avid = '1' then
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
            b_gsr <= b_gsr(2 downto 0) & g_start;
            if g_avid = '1' and b_px = c_ctr then b_fsr <= b_fsr(2 downto 0) & '1'; else b_fsr <= b_fsr(2 downto 0) & '0'; end if;

            if b_gsr(3) = '1' then
                b_ph <= n_ph0;
                b_inc <= n_inc;
                b_dn <= s_mir;
                b_nz <= n_seed;
                b_cc <= (others => '0');
                b_cl <= resize(s_x0(8 downto 5), 7);
                l_as <= n_as; l_ltp <= n_ltp; l_lfp <= n_lfp; l_t3h <= n_t3h;
                l_dl <= n_dl; l_rth <= n_rth; l_lev <= n_lev; l_lvc <= n_lvc; l_fth <= n_fth;
                l_cdf <= n_cdf; l_pb <= n_pb;
                l_cg0 <= n_cg0; l_cg1 <= n_cg1; l_cg2 <= n_cg2; l_cg3 <= n_cg3; l_cg4 <= n_cg4; l_cg5 <= n_cg5;
            elsif s_gav_sr(3) = '1' then
                if b_dn = '1' then b_ph <= b_ph - resize(b_inc, 28); else b_ph <= b_ph + resize(b_inc, 28); end if;
                v_cc := b_cc + resize(s_xs, 6);
                if v_cc >= 32 then
                    b_cc <= v_cc - 32;
                    b_cl <= b_cl + 1;
                    if b_nz(0) = '1' then b_nz <= ('0' & b_nz(15 downto 1)) xor x"B400";
                    else b_nz <= '0' & b_nz(15 downto 1); end if;
                else
                    b_cc <= v_cc;
                end if;
                if b_fsr(3) = '1' then b_dn <= '0'; end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n3 : signed(6 downto 0);
        variable v_e  : signed(7 downto 0);
        variable v_cg : unsigned(7 downto 0);
        variable v_g, v_r, v_b : std_logic;
        variable v_w  : unsigned(11 downto 0);
        variable v_c  : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: luma from the line buffer
            p3_lm <= unsigned(s_lb_q);
            -- 4: warp partials (luma nibbles x gain)
            p4_ea <= p3_lm(3 downto 0) * c_gw(16 downto 4);
            p4_eb <= p3_lm(7 downto 4) * c_gw(16 downto 4);
            -- 5: luma x warp (Q24 periods); Osc runs at four times the depth
            if s_src = '1' then
                p5_lw <= shift_left(resize(p4_ea, 28) + shift_left(resize(p4_eb, 28), 4), 6);
            else
                p5_lw <= shift_left(resize(p4_ea, 28) + shift_left(resize(p4_eb, 28), 4), 4);
            end if;
            -- 6: the camera bends the phase; 1.125 lines of plume per 1/256 period of bend; noise - tilt
            p6_ph <= b_ph + p5_lw;
            p6_bsh <= resize(shift_right(p5_lw, 16), 12) + resize(shift_right(p5_lw, 19), 12);
            v_n3 := signed(resize(b_nz(3 downto 0), 7)) - 8;
            v_n3 := v_n3 + shift_left(v_n3, 1);
            -- tilt 0.04 (x - 700 px) lines = 5/8 of the cell index - 27
            p6_ntl <= resize(shift_right(v_n3, 2), 13) - (signed(resize(shift_right(resize(b_cl, 10) + shift_left(resize(b_cl, 10), 2), 3), 13)) - 27);
            p6_nz <= b_nz(3 downto 0);
            -- 7: phase ROM address; period count; body level
            r_f_a <= p6_ph(23 downto 16);
            p7_n <= p6_ph(27 downto 24);
            p7_t <= signed(resize(l_as, 13)) - signed(resize(p6_bsh, 13));
            p7_ntl <= p6_ntl; p7_nz <= p6_nz;
            -- 8: (ROM read) dot phase n * cd; body level + noise - tilt; barb gate and dot thresholds; texture
            p8_nc <= resize(p7_n * l_cdf, 8);
            p8_bl <= p7_t + p7_ntl;
            v_w := resize(c_wg * p7_nz, 12);
            p8_gt <= c_wg3 + resize(shift_right(v_w, 5), 8);
            p8_dp <= resize(l_dl, 9) + resize(p7_nz, 9) + resize(shift_right(p7_nz, 1), 9);
            p8_sn <= l_pb + shift_left(resize(p7_nz, 8), 3) - resize(p7_nz, 8);
            -- 9: fabric copy of the ROM word
            p9_f <= r_f_q;
            p9_nc <= p8_nc; p9_bl <= p8_bl; p9_gt <= p8_gt; p9_dp <= p8_dp; p9_sn <= p8_sn;
            -- 10: body key, |e|, dot phase, texture phase
            if p9_bl < signed(resize(unsigned(p9_f(31 downto 22)), 13)) then p10_b <= '1'; else p10_b <= '0'; end if;
            v_e := signed(p9_f(21 downto 14));
            if v_e < 0 then p10_ae <= resize(unsigned(-resize(v_e, 9)), 7); else p10_ae <= unsigned(v_e(6 downto 0)); end if;
            case to_integer(unsigned(p9_f(13 downto 11))) is
                when 0      => v_cg := l_cg0;
                when 1      => v_cg := l_cg1;
                when 2      => v_cg := l_cg2;
                when 3      => v_cg := l_cg3;
                when 4      => v_cg := l_cg4;
                when others => v_cg := l_cg5;
            end case;
            v_cg := p9_nc + v_cg;
            p10_s8 <= v_cg;
            p10_st <= p9_sn + resize(unsigned(p9_f(10 downto 4)), 8);
            p10_gt <= p9_gt; p10_dp <= p9_dp;
            -- 11: the compares
            if resize(p10_ae, 8) < p10_gt then p11_g1 <= '1'; else p11_g1 <= '0'; end if;
            if resize(p10_s8, 9) < p10_dp then p11_g2 <= '1'; else p11_g2 <= '0'; end if;
            if p10_s8 > l_rth then p11_g3 <= '1'; else p11_g3 <= '0'; end if;
            if p10_ae < l_fth or l_ltp = '1' then p11_f1 <= '1'; else p11_f1 <= '0'; end if;
            if p10_st < l_lev then p11_t1 <= '1'; else p11_t1 <= '0'; end if;
            if p10_st > l_lvc then p11_t2 <= '1'; else p11_t2 <= '0'; end if;
            if l_lfp = '1' and p10_st >= 158 and p10_st < l_t3h then p11_t3 <= '1'; else p11_t3 <= '0'; end if;
            p11_b <= p10_b xor s_inv;
            -- 12: the three keys -> code R G B
            v_g := (p11_g1 and p11_g2) or (p11_f1 and p11_t1);
            v_r := (p11_g1 and p11_g3) or (p11_f1 and p11_t2);
            v_b := p11_b or (p11_f1 and p11_t3);
            p12_code <= v_r & v_g & v_b;
            -- 13: palette
            v_c := C_QL_PAL(to_integer(s_pal & p12_code));
            p13_y <= unsigned(v_c(31 downto 22)); p13_u <= unsigned(v_c(21 downto 12)); p13_v <= unsigned(v_c(11 downto 2));
            -- 14: carry
            p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix for these crisp, moving,
    -- saturated keys), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            o_wp <= o_wp + 1; o_rp <= o_rp + 1;
            o_ra(to_integer(o_wp)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15) & "00";
            o_aq <= o_ra(to_integer(o_rp));
            o_aq2 <= o_aq;
        end if;
    end process p_ring;

    p_ringb : process(clk)
    begin
        if rising_edge(clk) then
            o_rb(to_integer(o_wp)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17) & "00";
            o_bq <= o_rb(to_integer(o_rp));
            o_bq2 <= o_bq;
        end if;
    end process p_ringb;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank samples
            -- are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y15 - unsigned(o_aq2(31 downto 22));
                s_bau <= s_bau + s_u15 - unsigned(o_aq2(21 downto 12));
                s_bav <= s_bav + s_v15 - unsigned(o_aq2(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y17 - unsigned(o_bq2(31 downto 22));
                s_bbu <= s_bbu + s_u17 - unsigned(o_bq2(21 downto 12));
                s_bbv <= s_bbv + s_v17 - unsigned(o_bq2(11 downto 2));
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

end architecture quill;
