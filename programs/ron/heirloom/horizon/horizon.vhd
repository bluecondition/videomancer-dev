-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Horizon v0.1.0 -- Heirloom from the video clips, built from the still WIN_20230703_01_29_07 t10.0.
--
-- The still: a planet horizon at dusk from an RGB-colouriser patch.  A cyan sky with pale lavender lobes
-- hanging from the top, a magenta sun (a diamond, apex at the horizon) between two soft black mountains,
-- black streaks on the horizon line, then water striped by a chirped horizontal oscillator, and two dark
-- striped masses under a convex horizon arc whose lower edges dissolve into RIBS: a rib comb of yellow-
-- rimmed fingers (tips on the arc's continuation in the centre, narrow gaps opening from below in the masses).
--
-- The patch, as fitted (horizon.py has every table; .heirloom_work/rb_horizon/notes.md the measurements):
--   G  = |cG(y) + hG(|x - 973|)|                        one folded V field (the sky gradient and the water)
--   R  = ramp(y, |u|) x key(sky lobes OR the central column cut by the sun's diamond), the stripe S(y) below
--   B  = 255;  mountains: a black key on a band of a linear H x V field
--   ribs: the rib comb's distance from its peak q against a half-width hw(x, y) = min(0.5 - gap, rounded tip)
--   inside the ribs: R = G = the yellow rim (falling edge) / glow (rising edge), B = the stripe
--
--   K1  Ribs      the rib comb's rate: 6 .. 25 ribs across (12.3 = the still)
--   K2  Bowl      the horizon arc's depth: flat .. twice the still's bowl
--   K3  Tide      where the stripes start: the whole frame (the clip at 9.5 s) .. the horizon (the still) .. none
--   K4  Speed     bipolar: the stripes roll down (+) or up (-); 672 = the clip's 55 lines/s
--   K5  Lobes     the sky lobes' length: none .. the still (286 lines) .. down to the horizon
--   K6  Etch      how far the camera (or the Osc) bends the horizon, the ribs' gaps, the sky and the mountains
--   S7  Fan       the ribs radiate from below: the comb's rate falls toward the top
--   S8  Beads     the stripes patched hard into the rib key: the ribs break into stacks of beads
--   S9  Dawn      the red ramp inverted: a pink-white sky at the top, the sun fading out
--   S10 Mono      the grey look of the patch's sibling clips
--   S11 Source    Video / Osc
--   P12 RIBCAGE   the gap-tip curve: 0 = two solid hills meeting in a V, 460 = the still, 100 % = a full
--                 ribcage of fat yellow-rimmed ribs across the whole horizon
--
-- Engine: vblank-zone sequencer (shared glide, knob curves, one small serial multiplier), a line engine
-- kicked at each line start computing the NEXT line (one serial multiplier: the chirped stripe phase and
-- envelope, the field and ramp constants, the lobe ends, the comb rate and start phase), a beam (x, the rib
-- comb phase) and a 22-stage pixel path (three ROM lookups, the rib key, the sky key, the mountain key, the
-- RGB -> YUV matrix).  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (Y, U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps sit in block-RAM rings,
-- luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.horizon_pkg.all;
use work.all;

architecture horizon of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks

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
    function f_clip8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
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
    signal r_a_a, r_g_a, r_sq_a, r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_a_q, r_g_q, r_c_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sq_q, r_ex_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_e2_a : unsigned(7 downto 0) := (others => '0');
    signal r_e2_q : unsigned(15 downto 0) := (others => '0');

    attribute no_rw_check : boolean;

    -- output box rings (the 8-back taps of the two running sums) and the sync delay ring
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ba1, ba2, bb1, bb2 : t_r16 := (others => x"0000");
    attribute no_rw_check of ba1 : signal is true;
    attribute no_rw_check of ba2 : signal is true;
    attribute no_rw_check of bb1 : signal is true;
    attribute no_rw_check of bb2 : signal is true;
    signal rg_wa, rg_ra, sy_ra : unsigned(7 downto 0) := (others => '0');
    signal ba1_q, ba2_q, bb1_q, bb2_q : std_logic_vector(15 downto 0) := x"0000";
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal syr : t_r4 := (others => "1111");
    attribute no_rw_check of syr : signal is true;
    signal syr_q, syr_c : std_logic_vector(3 downto 0) := "1111";

    -- oscillator source (Source = Osc): 2-D triangle lattice, 320 x 270 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(242, 11);
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

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(672 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(460 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_fan, s_bead, s_dawn, s_mono, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field constants
    signal c_kf0  : unsigned(9 downto 0) := to_unsigned(420, 10);    -- rib comb rate, 1/65536 cycle per virtual px
    signal c_d8   : unsigned(7 downto 0) := to_unsigned(128, 8);     -- bowl depth, 128 = the still
    signal c_sye  : signed(11 downto 0) := to_signed(494, 12);       -- stripe start (virtual line)
    signal c_drift : unsigned(15 downto 0) := (others => '0');       -- stripe phase drift
    signal c_dstep : unsigned(15 downto 0) := (others => '0');
    signal c_dneg  : std_logic := '0';
    signal c_ph0  : unsigned(15 downto 0) := to_unsigned(62172, 16);
    signal c_ytip : signed(10 downto 0) := to_signed(288, 11);       -- lobe ends
    signal c_eg   : unsigned(8 downto 0) := (others => '0');         -- etch gain
    signal c_ygo  : signed(10 downto 0) := (others => '0');          -- gap-tip offset (P12)
    signal c_dy   : unsigned(6 downto 0) := to_unsigned(62, 7);      -- yellow rim width (q8)
    signal z_oct  : std_logic := '0';
    signal z_dd   : unsigned(8 downto 0) := (others => '0');
    signal z_a    : unsigned(25 downto 0) := (others => '0');
    signal z_b    : unsigned(9 downto 0) := (others => '0');
    signal z_acc  : unsigned(25 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';
    signal g_ssr : std_logic_vector(13 downto 0) := (others => '0');   -- g_start delayed 1 .. 14

    -- line engine (computes the NEXT line)
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_y    : unsigned(10 downto 0) := (others => '0');
    signal e_ey   : signed(12 downto 0) := (others => '0');
    signal e_epos : std_logic := '0';
    signal e_e    : unsigned(7 downto 0) := (others => '0');
    signal e_ph : unsigned(15 downto 0) := (others => '0');
    signal e_t1   : unsigned(15 downto 0) := (others => '0');
    signal e_tri, e_t9 : unsigned(7 downto 0) := (others => '0');
    signal e_neg  : std_logic := '0';
    signal e_sel  : unsigned(7 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal n_s    : unsigned(7 downto 0) := (others => '0');
    signal n_strk, n_abv : unsigned(7 downto 0) := (others => '0');
    signal n_y    : unsigned(10 downto 0) := (others => '0');
    signal n_cg   : signed(9 downto 0) := (others => '0');
    signal n_ra   : signed(15 downto 0) := (others => '0');
    signal n_rb   : signed(8 downto 0) := (others => '0');
    signal n_dl   : signed(13 downto 0) := (others => '0');
    signal n_hwl  : signed(10 downto 0) := (others => '0');
    signal n_kf   : unsigned(9 downto 0) := (others => '0');
    signal n_th0  : unsigned(15 downto 0) := (others => '0');
    signal n_kst  : unsigned(10 downto 0) := (others => '0');
    signal n_bf, n_br : signed(6 downto 0) := (others => '0');

    -- per-line values in use (loaded at g_start)
    signal l_s    : unsigned(7 downto 0) := (others => '0');
    signal l_ah   : unsigned(7 downto 0) := (others => '0');
    signal l_y    : signed(11 downto 0) := (others => '0');
    signal l_cg   : signed(9 downto 0) := (others => '0');
    signal l_ra   : signed(15 downto 0) := (others => '0');
    signal l_rb   : signed(8 downto 0) := (others => '0');
    signal l_dl   : signed(13 downto 0) := (others => '0');
    signal l_hwl  : signed(10 downto 0) := (others => '0');
    signal l_t, l_q : signed(11 downto 0) := (others => '0');
    signal l_kst  : unsigned(10 downto 0) := (others => '0');
    signal l_bf, l_br : signed(6 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_th  : unsigned(15 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_t14 : unsigned(15 downto 0) := (others => '0');   -- the comb phase of the stage-14 pixel
    signal c_xb0 : unsigned(11 downto 0) := to_unsigned(16, 12);

    -- pixel path
    signal p2_db, p2_dg : signed(12 downto 0) := (others => '0');
    signal p3_ubh, p3_ugh : unsigned(11 downto 0) := (others => '0');
    signal p3_lf : std_logic := '0';
    signal p4_ub : unsigned(10 downto 0) := (others => '0');
    signal p4_lm : unsigned(7 downto 0) := (others => '0');
    signal p5_u6 : unsigned(13 downto 0) := (others => '0');
    signal p5_cw : signed(11 downto 0) := (others => '0');
    signal p5_u58 : signed(11 downto 0) := (others => '0');
    signal p5_u4 : unsigned(7 downto 0) := (others => '0');
    signal p5_cl : unsigned(16 downto 0) := (others => '0');
    signal p6_arc0, p6_yg0, p6_hg : unsigned(7 downto 0) := (others => '0');
    signal p6_q10 : unsigned(10 downto 0) := (others => '0');
    signal p6_clq : unsigned(9 downto 0) := (others => '0');
    signal p6_dia : signed(14 downto 0) := (others => '0');
    signal p6_cw : signed(11 downto 0) := (others => '0');
    signal p6_gcq : unsigned(4 downto 0) := (others => '0');
    signal p6_su  : unsigned(10 downto 0) := (others => '0');
    signal p6_cl  : unsigned(8 downto 0) := (others => '0');
    signal p6_u58 : signed(11 downto 0) := (others => '0');
    signal p6_u4  : unsigned(7 downto 0) := (others => '0');
    signal p7_aoff : unsigned(15 downto 0) := (others => '0');
    signal p7_t1, p7_q1, p7_x : signed(11 downto 0) := (others => '0');
    signal p7_bl : signed(11 downto 0) := (others => '0');
    signal p7_yg4 : unsigned(9 downto 0) := (others => '0');
    signal p7_v   : signed(10 downto 0) := (others => '0');
    signal p7_kl  : signed(11 downto 0) := (others => '0');
    signal p7_dia, p7_cw : unsigned(7 downto 0) := (others => '0');
    signal p7_gcap : unsigned(6 downto 0) := (others => '0');
    signal p7_rbu : signed(17 downto 0) := (others => '0');
    signal p7_cv  : unsigned(8 downto 0) := (others => '0');
    signal p8_t, p8_dy : signed(12 downto 0) := (others => '0');
    signal p8_v   : signed(11 downto 0) := (others => '0');
    signal p8_kl, p8_key0 : unsigned(7 downto 0) := (others => '0');
    signal p8_rr  : signed(17 downto 0) := (others => '0');
    signal p8_bl : unsigned(7 downto 0) := (others => '0');
    signal p8_d1 : signed(11 downto 0) := (others => '0');
    signal p9_bt, p9_bb, p10_m1, p10_m2, p11_mtn : unsigned(7 downto 0) := (others => '0');
    signal p8_gcap : unsigned(6 downto 0) := (others => '0');
    signal p9_t, p10_t, p11_t, p12_t : signed(6 downto 0) := (others => '0');
    signal p9_pos, p10_pos, p11_pos : std_logic := '0';
    signal p9_gn, p10_gn, p11_gn : signed(8 downto 0) := (others => '0');
    signal p12_cir, p13_cir : unsigned(5 downto 0) := (others => '0');
    signal p12_tbig, p13_tbig, p14_tbig : std_logic := '0';
    signal p9_g, p10_g, p11_g : unsigned(7 downto 0) := (others => '0');
    signal p9_key, p9_rr, p9_m2 : unsigned(7 downto 0) := (others => '0');
    signal p9_gcap, p10_gcap, p11_gcap : unsigned(6 downto 0) := (others => '0');
    signal p10_rs : unsigned(15 downto 0) := (others => '0');
    signal p11_sq, p11_rbg : unsigned(7 downto 0) := (others => '0');
    signal p12_gap : signed(8 downto 0) := (others => '0');
    signal p17_rk, p17_gk, p18_rk, p18_gk : unsigned(12 downto 0) := (others => '0');
    signal p17_bk, p18_bk : unsigned(7 downto 0) := (others => '0');
    signal p15_q : unsigned(7 downto 0) := (others => '0');
    signal p15_fl, p16_fl, p17_fl : std_logic := '0';
    signal p16_bq : signed(9 downto 0) := (others => '0');
    signal p13_hwb, p14_hwb, p15_hwb : signed(9 downto 0) := (others => '0');
    signal p13_fz : unsigned(4 downto 0) := (others => '0');
    signal p13_aok : std_logic := '0';
    signal p14_tp : unsigned(13 downto 0) := (others => '0');
    signal p14_fo : unsigned(7 downto 0) := (others => '0');
    signal p15_tip, p16_hw, p17_dist : signed(10 downto 0) := (others => '0');
    signal p18_k : std_logic := '0';
    signal p18_shy, p18_sha : unsigned(7 downto 0) := (others => '0');
    signal p18_shg : unsigned(6 downto 0) := (others => '0');
    signal p19_r, p19_g, p19_b : unsigned(7 downto 0) := (others => '0');
    signal p20_ry, p20_gy, p20_by : unsigned(11 downto 0) := (others => '0');
    signal p20_rr, p20_gr, p20_br, p20_rb, p20_gb, p20_bb : unsigned(13 downto 0) := (others => '0');
    signal p21_y : unsigned(13 downto 0) := (others => '0');
    signal p21_u, p21_v : signed(15 downto 0) := (others => '0');
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- per-pixel delay lines
    signal sh_lf : std_logic_vector(15 downto 0) := (others => '0');
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    type t_s13a is array (natural range <>) of signed(12 downto 0);
    type t_s7a is array (natural range <>) of signed(6 downto 0);
    type t_u5a is array (natural range <>) of unsigned(4 downto 0);
    signal sh_t : t_s7a(0 to 4) := (others => (others => '0'));
    signal sh_rb, sh_gb : t_u8a(0 to 5) := (others => (others => '0'));
    signal sh_km : t_u5a(0 to 5) := (others => (others => '0'));
    signal sh_aok : std_logic_vector(3 downto 0) := (others => '0');

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);

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

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_a_q  <= C_HZ_A(to_integer(r_a_a));
            r_g_q  <= C_HZ_G(to_integer(r_g_a));
            r_c_q  <= C_HZ_C(to_integer(r_a_a));
            r_sq_q <= C_HZ_SQ(to_integer(r_sq_a));
            r_ex_q <= C_HZ_EX(to_integer(r_ex_a));
            r_e2_q <= C_HZ_EXP2(to_integer(r_e2_a));
        end if;
    end process p_rom;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y23) & std_logic_vector(s_u23(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u23(3 downto 0)) & std_logic_vector(s_v23) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y25) & std_logic_vector(s_u25(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u25(3 downto 0)) & std_logic_vector(s_v25) & "00";
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field constants (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_s  : signed(12 downto 0);
        variable v_u  : unsigned(15 downto 0);
        variable v_w  : unsigned(23 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_fan  <= registers_in(6)(0);
                s_bead <= registers_in(6)(1);
                s_dawn <= registers_in(6)(2);
                s_mono <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(408, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(306, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(204, 9);
                end case;
                -- the stripes' phase drift (K4): one step per field
                if c_dneg = '1' then c_drift <= c_drift - c_dstep; else c_drift <= c_drift + c_dstep; end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(shift_left(resize(s_ys, 20), 8) - shift_left(resize(s_ys, 20), 4) + shift_left(resize(s_ys, 20), 1), 8), 11);   -- 242 ys / 256
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 2);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, seven channels
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 5      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 3      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 5      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 6 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- zone serial multiplier (10-bit b, 10 iterations: the product is read 11 clocks after the load)
            if z_b(0) = '1' then z_acc <= z_acc + z_a; end if;
            z_a <= z_a(24 downto 0) & '0';
            z_b <= '0' & z_b(9 downto 1);

            -- knob curves: zc 40 .. 127, one small operation per clock, every product on the zone multiplier
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>                                        -- K1: kf0 = 420 2^((k1 - 512) / 512)
                        v_d := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        r_e2_a <= unsigned(v_d(8 downto 1)); z_oct <= v_d(9);
                    when 42 =>
                        z_a <= resize(r_e2_q, 26); z_b <= to_unsigned(105, 10); z_acc <= (others => '0');
                    when 53 =>
                        if z_oct = '1' then c_kf0 <= z_acc(23 downto 14); else c_kf0 <= z_acc(22 downto 13); end if;
                        z_a <= resize(s_g3(12 downto 3), 26); z_b <= to_unsigned(372, 10); z_acc <= (others => '0');
                    when 64 =>                                        -- K3: stripe start = 1.453 k3 - 250
                        c_sye <= signed(resize(z_acc(18 downto 8), 12)) - 250;
                        z_a <= resize(s_g5(12 downto 3), 26); z_b <= to_unsigned(179, 10); z_acc <= (others => '0');
                    when 75 =>                                        -- K5: lobe ends = 0.70 k5 - 72
                        c_ytip <= signed(resize(z_acc(17 downto 8), 11)) - 72;
                        z_a <= resize(s_gp(12 downto 3), 26); z_b <= to_unsigned(363, 10); z_acc <= (others => '0');
                    when 86 =>                                        -- P12: gap-tip offset = 652 - 1.418 p
                        c_ygo <= to_signed(652, 11) - signed(resize(z_acc(18 downto 8), 11));
                        if s_gp(12 downto 3) > 460 then z_a <= resize(s_gp(12 downto 3) - 460, 26);
                        else z_a <= (others => '0'); end if;
                        z_b <= to_unsigned(14, 10); z_acc <= (others => '0');
                    when 97 =>                                        -- P12: rim width = 62 + 14 (p - 460) / 256
                        c_dy <= to_unsigned(62, 7) + resize(z_acc(13 downto 8), 7);
                        v_k := s_g4(12 downto 3);
                        if v_k > 512 then c_dneg <= '1'; v_k := v_k - 512; else c_dneg <= '0'; v_k := 512 - v_k; end if;   -- right = the stripes roll down
                        if v_k > 8 then z_dd <= resize(v_k - 8, 9); else z_dd <= (others => '0'); end if;
                    when 98 =>                                        -- K4: drift step = dd^2 * 3 / 64
                        z_a <= resize(z_dd, 26); z_b <= resize(z_dd, 10); z_acc <= (others => '0');
                    when 109 =>
                        v_w := shift_right(resize(z_acc, 24) + shift_left(resize(z_acc, 24), 1), 6);
                        c_dstep <= v_w(15 downto 0);
                        z_a <= resize(s_g6(12 downto 3), 26); z_b <= s_g6(12 downto 3); z_acc <= (others => '0');
                    when 120 =>                                       -- K6: etch gain = k6^2 / 2^14
                        c_eg <= "000" & z_acc(19 downto 14);
                        c_d8 <= "00" & s_g2(12 downto 7);             -- K2: bowl depth, 32 = the still
                    when 121 =>
                        c_ph0 <= to_unsigned(62172, 16) + c_drift;
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line's values with one serial multiplier
    -- (16 iterations, the product read 17 clocks after the load)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_f   : unsigned(7 downto 0);
        variable v_d   : unsigned(8 downto 0);
        variable v_t   : signed(9 downto 0);
        variable v_y   : signed(12 downto 0);
        variable v_w   : signed(15 downto 0);
        variable v_p   : unsigned(23 downto 0);
        variable v_k   : unsigned(10 downto 0);
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

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(19 downto 8) > 1079 then e_y <= to_unsigned(1079, 11); else e_y <= e_vy(18 downto 8); end if;
                when 2 =>
                    e_ey <= signed(resize(e_y, 13)) - resize(c_sye, 13);
                    m_a <= resize(e_y, 32); m_b <= to_unsigned(58581, 16); m_acc <= (others => '0');
                when 3 =>
                    if e_ey <= 0 then r_ex_a <= (others => '0'); e_epos <= '0';
                    elsif e_ey > 1023 then r_ex_a <= to_unsigned(255, 9); e_epos <= '1';
                    else r_ex_a <= "0" & unsigned(e_ey(9 downto 2)); e_epos <= '1'; end if;
                when 5 =>
                    if e_epos = '1' then e_e <= unsigned(r_ex_q); else e_e <= (others => '0'); end if;
                when 19 =>
                    e_ph <= m_acc(21 downto 6);
                    m_a <= resize(e_y, 32); m_b <= to_unsigned(12379, 16); m_acc <= (others => '0');
                when 36 =>
                    e_t1 <= m_acc(23 downto 8);
                when 37 =>
                    m_a <= resize(e_t1, 32); m_b <= resize(e_y, 16); m_acc <= (others => '0');
                when 54 =>
                    e_ph <= e_ph + m_acc(23 downto 8);
                when 56 =>
                    e_ph <= e_ph + c_ph0;
                when 57 =>
                    v_f := e_ph(15 downto 8);
                    if v_f(7) = '1' then v_d := to_unsigned(256, 9) - resize(v_f, 9); else v_d := resize(v_f, 9); end if;
                    if v_d >= 128 then e_tri <= (others => '0');
                    else e_tri <= to_unsigned(255, 8) - (v_d(6 downto 0) & '0'); end if;
                when 58 =>
                    if e_tri > 25 then e_t9 <= e_tri - 25; else e_t9 <= (others => '0'); end if;
                when 59 =>
                    v_k := resize(e_t9, 11) + resize(e_t9(7 downto 3), 11);
                    if v_k > 255 then e_t9 <= (others => '1'); else e_t9 <= v_k(7 downto 0); end if;
                when 60 =>
                    m_a <= resize(e_e, 32); m_b <= resize(e_t9, 16); m_acc <= (others => '0');
                when 77 =>
                    n_s <= m_acc(15 downto 8);
                    -- cG: 172 - 0.527 y (y <= 600), -145 - 0.28 (y - 600) below
                    if e_y <= 600 then
                        m_a <= resize(e_y, 32); m_b <= to_unsigned(135, 16); e_neg <= '0';
                    else
                        m_a <= resize(e_y - 600, 32); m_b <= to_unsigned(72, 16); e_neg <= '1';
                    end if;
                    m_acc <= (others => '0');
                when 78 =>
                    -- the stripe streak gate, the horizon gate, the bead terms (from n_s)
                    if (signed(resize(e_y, 13)) > resize(c_sye, 13)) and (e_y < 585 or c_sye < 400) then
                        if n_s >= 64 then n_strk <= (others => '0');
                        else n_strk <= to_unsigned(255, 8) - (n_s(5 downto 0) & "00"); end if;
                    else
                        n_strk <= (others => '0');
                    end if;
                    v_y := to_signed(510, 13) - signed(resize(e_y, 13));
                    if v_y < -4 then n_abv <= (others => '0');
                    elsif v_y > 3 then n_abv <= (others => '1');
                    else v_w := resize(shift_left(v_y, 5), 16) + 128; n_abv <= unsigned(v_w(7 downto 0)); end if;
                when 79 =>
                    v_t := signed(resize(n_s, 10)) - 128;
                    if s_bead = '1' then                                             -- (S - 128) / 4 on both edges
                        n_bf <= resize(shift_right(v_t, 2), 7); n_br <= resize(shift_right(v_t, 2), 7);
                    else                                                              -- (S - 128) / 16 on the falling edge
                        n_bf <= resize(shift_right(v_t, 4), 7); n_br <= (others => '0');
                    end if;
                when 94 =>
                    if e_neg = '0' then n_cg <= to_signed(172, 10) - signed(resize(m_acc(18 downto 8), 10));
                    else n_cg <= to_signed(-145, 10) - signed(resize(m_acc(18 downto 8), 10)); end if;
                    -- ra: 0.69 y (Q6) normal; (250 - y/2) * 64 dawn
                    m_a <= resize(e_y, 32); m_b <= to_unsigned(177, 16); m_acc <= (others => '0');
                when 111 =>
                    if s_dawn = '1' then
                        n_ra <= shift_left(to_signed(250, 16) - signed(resize(e_y(10 downto 1), 16)), 6);
                        m_a <= resize(e_y, 32); m_b <= to_unsigned(33, 16); e_neg <= '0';
                    else
                        n_ra <= signed(resize(m_acc(17 downto 2), 16));
                        if e_y <= 87 then
                            m_a <= resize(to_unsigned(175, 11) - (e_y(9 downto 0) & '0'), 32); e_neg <= '0';
                        else
                            m_a <= resize((e_y(9 downto 0) & '0') - to_unsigned(175, 11), 32); e_neg <= '1';
                        end if;
                        m_b <= to_unsigned(21, 16);
                    end if;
                    m_acc <= (others => '0');
                when 128 =>
                    if e_neg = '1' then n_rb <= -signed(resize(m_acc(17 downto 9), 9));
                    else n_rb <= signed(resize(m_acc(16 downto 9), 9)); end if;
                    -- DL = (489 - y) * 683 / 64
                    if e_y <= 489 then m_a <= resize(489 - e_y, 32); e_neg <= '0';
                    else m_a <= resize(e_y - 489, 32); e_neg <= '1'; end if;
                    m_b <= to_unsigned(683, 16); m_acc <= (others => '0');
                when 145 =>
                    v_p := m_acc(29 downto 6);
                    if e_neg = '1' then n_dl <= 128 - signed(resize(v_p(12 downto 0), 14));
                    else n_dl <= 128 + signed(resize(v_p(12 downto 0), 14)); end if;
                    -- the lobe ends: hwL = 69 CIR((ytip - y) / 2), -51 below the end
                    v_y := resize(c_ytip, 13) - signed(resize(e_y, 13));
                    e_ey <= v_y;
                    if v_y <= 0 then e_sel <= (others => '0');
                    elsif v_y >= 78 then e_sel <= to_unsigned(39, 8);
                    else e_sel <= resize(unsigned(v_y(6 downto 1)), 8); end if;
                when 146 =>
                    m_a <= resize(C_HZ_CIR(to_integer(e_sel(5 downto 0))), 32); m_b <= to_unsigned(69, 16); m_acc <= (others => '0');
                when 163 =>
                    -- the lobe key term hwL x 8 + 128 (hwL = 69 CIR / 64, -51 below the lobe end)
                    if e_ey <= 0 then n_hwl <= to_signed(-280, 11);
                    else n_hwl <= signed(resize(shift_left(resize(m_acc(13 downto 6), 11), 3), 11)) + 128; end if;
                    -- Fan: kf = kf0 - kf0 (1080 - y) / 2048
                    m_a <= resize(c_kf0, 32); m_b <= resize(to_unsigned(1080, 11) - e_y, 16); m_acc <= (others => '0');
                when 180 =>
                    if s_fan = '1' then n_kf <= c_kf0 - resize(m_acc(20 downto 11), 10);
                    else n_kf <= c_kf0; end if;
                when 181 =>
                    -- the comb's start phase: -((1922 - x0) kf) / 2
                    m_a <= resize(to_unsigned(1922, 12) - c_xb0, 32); m_b <= resize(n_kf, 16); m_acc <= (others => '0');
                    case to_integer(s_xs) is
                        when 4      => n_kst <= n_kf & '0';
                        when 3      => n_kst <= resize(n_kf, 11) + resize(n_kf(9 downto 1), 11);
                        when others => n_kst <= resize(n_kf, 11);
                    end case;
                when 198 =>
                    n_th0 <= to_unsigned(0, 16) - m_acc(16 downto 1);
                    n_y <= e_y;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in h, the rib comb's phase; the camera read address
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_th <= n_th0;
                b_px <= (others => '0');
                l_s <= n_s; l_y <= signed(resize(n_y, 12)); l_cg <= n_cg; l_ra <= n_ra; l_rb <= n_rb;
                l_t <= signed(resize(n_y, 12)) - 565;
                l_q <= signed(resize(n_y, 12)) - resize(c_ygo, 12);
                l_dl <= n_dl; l_hwl <= n_hwl; l_kst <= n_kst; l_bf <= n_bf; l_br <= n_br;
                if n_strk > n_abv then l_ah <= n_strk; else l_ah <= n_abv; end if;
            elsif g_avid = '1' then
                b_x  <= b_x + s_xs;
                b_th <= b_th + resize(l_kst, 16);
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
            -- the comb phase again, 13 clocks behind the beam (the stage-14 pixel's value)
            g_ssr <= g_ssr(12 downto 0) & g_start;
            if g_ssr(12) = '1' then
                b_t14 <= n_th0;
            elsif s_gav_sr(12) = '1' then
                b_t14 <= b_t14 + resize(l_kst, 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel; pK_ = the stage-K pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s  : signed(15 downto 0);
        variable v_s2 : signed(15 downto 0);
        variable v_g  : signed(9 downto 0);
        variable v_h  : signed(10 downto 0);
        variable v_x  : signed(15 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_r  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: offsets from the two axes (h), the comb phase byte
            p2_db <= signed(resize(b_x, 13)) - 2000;
            p2_dg <= signed(resize(b_x, 13)) - 1946;
            -- 3: |u| (h), the comb's distance from its peak and its slope side
            if p2_db < 0 then p3_ubh <= unsigned(resize(-p2_db, 12)); p3_lf <= '1';
            else p3_ubh <= unsigned(p2_db(11 downto 0)); p3_lf <= '0'; end if;
            if p2_dg < 0 then p3_ugh <= unsigned(resize(-p2_dg, 12)); else p3_ugh <= unsigned(p2_dg(11 downto 0)); end if;
            sh_lf <= sh_lf(14 downto 0) & p3_lf;      -- sh_lf(k) = stage 4 + k
            -- 4: ROM addresses (2 virtual px per entry), |uB| in virtual px; the camera luma (fabric copy)
            if p3_ubh(11) = '1' then r_a_a <= (others => '1'); else r_a_a <= p3_ubh(10 downto 2); end if;
            if p3_ugh(11) = '1' then r_g_a <= (others => '1'); else r_g_a <= p3_ugh(10 downto 2); end if;
            p4_ub <= p3_ubh(11 downto 1);
            p4_lm <= unsigned(s_lb_q);
            -- 5: constant multiples of |uB|; camera x etch (ROM data in the EBR)
            p5_u6 <= resize(shift_left(resize(p4_ub, 14), 2) + shift_left(resize(p4_ub, 14), 1), 14);
            p5_cw <= to_signed(400, 12) - signed(resize(p4_ub, 12));
            p5_u58 <= signed(resize(shift_right(shift_left(resize(p4_ub, 14), 2) + resize(p4_ub, 14), 3), 12)) - 505;
            p5_u4 <= "0" & p4_ub(9 downto 3);
            p5_cl <= p4_lm * c_eg;
            -- 6: ROM data (fabric copy), the diamond / column / gap-cap partials, the mountain slope
            p6_arc0 <= unsigned(r_a_q(15 downto 8)); p6_yg0 <= unsigned(r_a_q(7 downto 0));
            p6_hg   <= unsigned(r_g_q(15 downto 8));
            p6_q10  <= shift_left(resize(unsigned(r_g_q(7 downto 0)), 11), 3);
            p6_dia  <= resize(l_dl, 15) - signed(resize(p5_u6, 15));
            p6_gcq <= unsigned(r_c_q(5 downto 1));
            p6_cw <= p5_cw;
            p6_su <= resize(unsigned(r_c_q(15 downto 6)), 11);
            p6_cl <= resize(p5_cl(16 downto 5), 9);
            p6_clq <= resize(p5_cl(16 downto 5), 10) + resize(p5_cl(16 downto 8), 10);
            p6_u58 <= p5_u58; p6_u4 <= p5_u4;
            -- 7: bowl x arc, the line terms + camera, G sum, the lobe key, the diamond / column clips, R ramp product
            p7_aoff <= resize(p6_arc0 * c_d8(5 downto 0), 16);
            p7_t1 <= l_t + signed(resize(p6_cl, 12));
            p7_q1 <= l_q + signed(resize(p6_clq, 12));
            p7_yg4 <= p6_yg0 & "00";
            p7_v <= resize(l_cg, 11) + signed(resize(p6_hg, 11));
            p7_cv <= resize(shift_right(shift_left(resize(p6_cl, 12), 2) + resize(p6_cl, 12), 3), 9);
            p7_kl <= resize(l_hwl, 12) - signed(resize(p6_q10, 12));
            p7_dia <= f_clip8(p6_dia);
            p7_cw <= f_clip8(shift_left(resize(p6_cw, 16), 2));
            p7_gcap <= to_unsigned(44, 7) + resize(p6_gcq, 7);
            p7_rbu <= resize(shift_left(resize(l_rb * signed(resize(p6_u4(6 downto 0), 8)), 18), 1), 18);
            p7_x <= signed(resize(p6_cl(8 downto 1), 12)) - signed(resize(p6_su, 12));
            p7_bl <= l_y + p6_u58;
            -- 8: t, dy, G + camera, the sky key, R ramp sum, the mountain ramps
            p8_t  <= resize(p7_t1, 13) - signed(resize(p7_aoff(15 downto 4), 13));
            p8_dy <= resize(p7_q1, 13) - signed(resize(p7_yg4, 13));
            p8_v  <= resize(p7_v, 12) - signed(resize(p7_cv, 12));
            p8_kl <= f_clip8(p7_kl);
            if p7_dia < p7_cw then p8_key0 <= p7_dia; else p8_key0 <= p7_cw; end if;
            p8_rr <= resize(l_ra, 18) + p7_rbu;
            p8_d1 <= l_y + p7_x;                                  -- y - (288 + 0.406 |u| - cL / 2)
            v_s := shift_left(resize(p7_bl, 16), 5) + 128;
            p8_bl <= f_clip8(v_s);
            p8_gcap <= p7_gcap;
            -- 9: gap ROM address, the CIR lookup, G, the R key, R ramp clip, the mountain key
            if p8_dy > 0 then
                if p8_dy > 1023 then r_sq_a <= to_unsigned(255, 9); else r_sq_a <= "0" & unsigned(p8_dy(9 downto 2)); end if;
                p9_pos <= '1';
            else
                p9_pos <= '0';
            end if;
            v_s := shift_right(resize(p8_dy, 16) + shift_left(resize(p8_dy, 16), 1), 2);
            if v_s < -128 then p9_gn <= to_signed(-128, 9); else p9_gn <= v_s(8 downto 0); end if;
            if p8_t < 0 then p9_t <= to_signed(-1, 7);
            elsif p8_t > 63 then p9_t <= to_signed(63, 7);
            else p9_t <= p8_t(6 downto 0); end if;
            if p8_v < 0 then v_h := resize(-p8_v, 11); else v_h := resize(p8_v, 11); end if;
            if v_h > 255 then p9_g <= (others => '1'); else p9_g <= unsigned(v_h(7 downto 0)); end if;
            if p8_kl > p8_key0 then p9_key <= p8_kl; else p9_key <= p8_key0; end if;
            p9_rr <= f_clip8(shift_right(p8_rr, 6));
            p9_bt <= f_clip8(shift_left(resize(p8_d1, 16), 3) + 128);
            p9_bb <= f_clip8(to_signed(1648, 16) - shift_left(resize(p8_d1, 16), 4));
            if l_ah < p8_bl then p9_m2 <= l_ah; else p9_m2 <= p8_bl; end if;
            p9_gcap <= p8_gcap;
            -- 10: SQ ROM (EBR); R ramp x key; the mountain key
            p10_t <= p9_t; p10_pos <= p9_pos; p10_gn <= p9_gn;
            p10_rs <= resize(p9_rr * p9_key(7 downto 2), 16);
            p10_g <= p9_g;
            if p9_bb < p9_bt then p10_m1 <= p9_bb; else p10_m1 <= p9_bt; end if;
            p10_m2 <= p9_m2;
            p10_gcap <= p9_gcap;
            -- 11: SQ data (fabric copy); R background = max(R ramp x key, stripe)
            p11_sq <= unsigned(r_sq_q);
            p11_t <= p10_t; p11_pos <= p10_pos; p11_gn <= p10_gn;
            p11_gcap <= p10_gcap;
            if p10_rs(13 downto 6) > l_s then p11_rbg <= p10_rs(13 downto 6); else p11_rbg <= l_s; end if;
            p11_g <= p10_g;
            if p10_m2 < p10_m1 then p11_mtn <= p10_m2; else p11_mtn <= p10_m1; end if;
            -- 12: the gap half-width (q8): sqrt below the gap tip, linear above, capped; background x mountain key
            if p11_pos = '1' then
                if p11_sq > resize(p11_gcap, 8) then p12_gap <= signed(resize(p11_gcap, 9));
                else p12_gap <= signed(resize(p11_sq, 9)); end if;
            else
                p12_gap <= p11_gn;
            end if;
            p12_t <= p11_t;
            if p11_t <= 0 then p12_cir <= (others => '0');
            elsif p11_t >= 40 then p12_cir <= to_unsigned(63, 6);
            else p12_cir <= C_HZ_CIR(to_integer(unsigned(p11_t(5 downto 0)))); end if;
            if p11_t >= 40 then p12_tbig <= '1'; else p12_tbig <= '0'; end if;
            sh_rb(0) <= p11_rbg; sh_gb(0) <= p11_g; sh_km(0) <= resize(shift_right(to_unsigned(255, 8) - p11_mtn, 3), 5);
            for i in 1 to 5 loop
                sh_rb(i) <= sh_rb(i - 1); sh_gb(i) <= sh_gb(i - 1); sh_km(i) <= sh_km(i - 1);
            end loop;                                                 -- sh_rb(k) = stage 12 + k
            -- 13: hwB (the rib half-width the gaps allow), fz, the arc-rim gate
            p13_hwb <= to_signed(128, 10) - resize(p12_gap, 10);
            v_g := resize(p12_gap, 10) + resize(shift_right(p12_gap, 2), 10);
            if v_g < 0 then p13_fz <= (others => '0'); elsif v_g > 16 then p13_fz <= to_unsigned(16, 5);
            else p13_fz <= unsigned(v_g(4 downto 0)); end if;
            if p12_gap < 7 and p12_t >= 0 and p12_t < 20 then p13_aok <= '1'; else p13_aok <= '0'; end if;
            p13_cir <= p12_cir; p13_tbig <= p12_tbig;
            sh_t(0) <= p12_t;
            for i in 1 to 4 loop sh_t(i) <= sh_t(i - 1); end loop;   -- sh_t(k) = stage 13 + k
            -- 14: the rounded tip: hwB x CIR
            p14_hwb <= p13_hwb;
            p14_tp <= resize(unsigned(p13_hwb(8 downto 1)) * p13_cir, 14);
            p14_fo <= shift_left(resize(to_unsigned(16, 5) - p13_fz, 8), 3);
            p14_tbig <= p13_tbig;
            sh_aok <= sh_aok(2 downto 0) & p13_aok;                   -- sh_aok(k) = stage 14 + k
            -- 15: the comb's distance from its peak and its slope side (from the stage-14 phase)
            if b_t14(15) = '0' then p15_q <= b_t14(15 downto 8); p15_fl <= '1';
            else p15_q <= to_unsigned(0, 8) - b_t14(15 downto 8); p15_fl <= '0'; end if;
            p15_hwb <= p14_hwb;
            if p14_tbig = '1' then p15_tip <= to_signed(511, 11);
            else p15_tip <= signed(resize(p14_tp(13 downto 5), 11)) + signed(resize(p14_fo, 11)); end if;
            -- 16: hw = min(hwB, tip); the bead term minus q
            if resize(p15_hwb, 11) < p15_tip then p16_hw <= resize(p15_hwb, 11); else p16_hw <= p15_tip; end if;
            if p15_fl = '1' then p16_bq <= resize(l_bf, 10) - signed(resize(p15_q, 10));
            else p16_bq <= resize(l_br, 10) - signed(resize(p15_q, 10)); end if;
            p16_fl <= p15_fl;
            -- 17: distance inside the rib (q8); the background x mountain key products
            p17_dist <= p16_hw + resize(p16_bq, 11);
            p17_fl <= p16_fl;
            p17_rk <= sh_rb(4) * sh_km(4);
            p17_gk <= sh_gb(4) * sh_km(4);
            p17_bk <= resize(shift_right(to_unsigned(255, 8) * sh_km(4), 5), 8);
            -- 18: the key and the shades
            if sh_t(4) > 0 and p17_dist > 0 then p18_k <= '1'; else p18_k <= '0'; end if;
            v_x := shift_left(resize(signed(resize(c_dy, 11)) - p17_dist, 16), 2);       -- x 4
            if p17_fl = '1' then
                if v_x < 12 then p18_shy <= to_unsigned(12, 8); elsif v_x > 206 then p18_shy <= to_unsigned(206, 8);
                else p18_shy <= unsigned(v_x(7 downto 0)); end if;
                p18_shg <= (others => '0');
            else
                p18_shy <= (others => '0');
                if p17_dist > 24 then p18_shg <= to_unsigned(12, 7);
                elsif p17_dist < 0 then p18_shg <= to_unsigned(110, 7);
                else p18_shg <= to_unsigned(110, 7) - (unsigned(p17_dist(4 downto 0)) & "00"); end if;
            end if;
            if sh_aok(3) = '1' then
                v_x := shift_left(resize(to_signed(20, 8) - sh_t(4), 16), 4);
                if sh_lf(13) = '1' then
                    if v_x > 206 then p18_sha <= to_unsigned(206, 8); else p18_sha <= unsigned(v_x(7 downto 0)); end if;
                else
                    p18_sha <= unsigned(v_x(8 downto 1));
                end if;
            else
                p18_sha <= (others => '0');
            end if;
            p18_rk <= p17_rk; p18_gk <= p17_gk; p18_bk <= p17_bk;
            -- 19: compose
            if p18_k = '1' then
                v_r := p18_shy or resize(p18_shg, 8);          -- one of the two is 0 (the slope side)
                if p18_sha > v_r then v_r := p18_sha; end if;
                p19_r <= v_r; p19_g <= v_r;
                if p18_shy >= 103 then p19_b <= l_s - shift_right(l_s, 2); else p19_b <= l_s; end if;
            else
                p19_r <= p18_rk(12 downto 5); p19_g <= p18_gk(12 downto 5); p19_b <= p18_bk;
            end if;
            -- 20: matrix products: Y = 64 + (5R + 10G + 2B) / 4; Cr = 512 + (56R - 47G - 9B) / 32;
            --     Cb = 512 + (-19R - 37G + 56B) / 32   (luma pre-scaled 16/13, capped 940)
            p20_ry <= shift_left(resize(p19_r, 12), 2) + resize(p19_r, 12);
            p20_gy <= shift_left(resize(p19_g, 12), 3) + shift_left(resize(p19_g, 12), 1);
            p20_by <= shift_left(resize(p19_b, 12), 1);
            p20_rr <= shift_left(resize(p19_r, 14), 6) - shift_left(resize(p19_r, 14), 3);
            p20_gr <= shift_left(resize(p19_g, 14), 5) + shift_left(resize(p19_g, 14), 4) - resize(p19_g, 14);
            p20_br <= shift_left(resize(p19_b, 14), 3) + resize(p19_b, 14);
            p20_rb <= shift_left(resize(p19_r, 14), 4) + shift_left(resize(p19_r, 14), 1) + resize(p19_r, 14);
            p20_gb <= shift_left(resize(p19_g, 14), 5) + shift_left(resize(p19_g, 14), 2) + resize(p19_g, 14);
            p20_bb <= shift_left(resize(p19_b, 14), 6) - shift_left(resize(p19_b, 14), 3);
            -- 21: sums
            p21_y <= resize(p20_ry, 14) + resize(p20_gy, 14) + resize(p20_by, 14);
            p21_u <= signed(resize(p20_rr, 16)) - signed(resize(p20_gr, 16)) - signed(resize(p20_br, 16));
            p21_v <= signed(resize(p20_bb, 16)) - signed(resize(p20_rb, 16)) - signed(resize(p20_gb, 16));
            -- 22: offsets, luma cap 940, Mono, black outside the generated avid
            v_y := resize(p21_y(13 downto 2), 12) + 64;
            if s_gav_sr(19) = '0' then
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            else
                if v_y > 940 then s_y22 <= to_unsigned(940, 10); else s_y22 <= v_y(9 downto 0); end if;
                if s_mono = '1' then
                    s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
                else
                    v_x := shift_right(p21_u, 5) + 512; s_u22 <= unsigned(v_x(9 downto 0));
                    v_x := shift_right(p21_v, 5) + 512; s_v22 <= unsigned(v_x(9 downto 0));
                end if;
            end if;
            -- 23
            s_y23 <= s_y22; s_u23 <= s_u22; s_v23 <= s_v22;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(20);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the 8-back taps come from block-RAM rings:
    -- the HD HDMI bar fix), luma 13/16 and the blanking gate; the syncs ride a
    -- block-RAM ring of the same latency
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- ring addresses: one write per clock; the read, issued two clocks ahead and copied once,
            -- is the sample written 8 clocks before the one being added
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 5;
            sy_ra <= rg_wa - 25;
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 24 sums, stage 25 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y23 - s_cay;
                s_bau <= s_bau + s_u23 - s_cau;
                s_bav <= s_bav + s_v23 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(22) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y25 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u25 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v25 <= v_sc(13 downto 4);
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(22);
            -- box B: stage 26 sums, stage 27 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y25 - s_cby;
                s_bbu <= s_bbu + s_u25 - s_cbu;
                s_bbv <= s_bbv + s_v25 - s_cbv;
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y27 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u27 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v27 <= v_sc(13 downto 4);
            -- 28: luma 13/16
            if s_y27 > 64 then v_dl := s_y27 - 64; else v_dl := (others => '0'); end if;
            s_y28 <= s_y27 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u28 <= s_u27; s_v28 <= s_v27;
            -- 29: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y29 <= s_y28; s_u29 <= s_u28; s_v29 <= s_v28;
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y29);
    data_out.u       <= std_logic_vector(s_u29);
    data_out.v       <= std_logic_vector(s_v29);
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture horizon;
