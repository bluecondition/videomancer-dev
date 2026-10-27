-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Vesica v0.5.0 -- Heirloom #13, rebuilt from WIN_20230307_01_04_17.
--
-- The photo: a pale green ground crossed by an X lattice of pastel bands
-- (lavender core, pink, aqua, sky, yellow), the sum of a horizontal and a
-- vertical triangle oscillator through a pastel colourizer, with glitch rows
-- where the vertical waveform notches.  Over it an orange vesica (the lens
-- where two circles meet) striped by a horizontal oscillator hard-synced to
-- its left edge, its right edge torn into flame teeth; a translucent brown
-- plate (a bar and two wings) joins it to its two side copies (a horizontal
-- fold, period 945 px); orange crescents above and below on a second circle
-- family, whose thin arcs run on through the copies; outside the crescents a
-- band of zig-zag dashes and, top and bottom, translucent brown line rows.
--
-- The program:
--   * lattice  F = A(x) + 1.03 B(y) - 1.05 (+ the camera), A a triangle with a
--     soft top knee (ROM) plus a parabola (ROM over x), B a triangle with a
--     glitch ROM; |F| x gain indexes the 64-step band palette;
--   * the orange layer: rho^2 = X^2 + Y^2 on two folded circle families,
--     X^2 by 2nd-order DDAs in half-pixels (reloaded at the folds and, for the
--     torn lens, where the tear starts), Y = d + a - c a^2 per line;
--   * a per-line engine (kicked at each line start, computing the NEXT line;
--     one serial multiplier, one serial square root): the vertical waveform,
--     the lens half-width, the tear sawtooth, the stripe phase, the DDA loads,
--     the lattice's vertical phase, the glitch / line-row ROMs;
--   * per field (vblank zone): knob curves and the eye geometry from ROMs
--     indexed by the glided knobs, fold counters, then a short field-init run
--     of the line engine (speed^2, lattice steps and phases, fold squares).
--
--   K1  Lattice    lattice cell size 0.5x .. 2x
--   K2  Bands      pastel band width (colourizer gain) 2.2x thin .. 0.45x fat
--   K3  Stripes    vesica stripe pitch 24 .. 160 px (71 = the photo)
--   K4  Crescents  crescent width 0 .. 2x
--   K5  Speed      bipolar: stripes stream, lattice slides, teeth ripple, dashes crawl
--   K6  Etch       how hard the camera (Video) or the oscillator (Osc) bends the lattice and stripes
--   S7  Tear       flame teeth on the lens edges On / Off
--   S8  Twins      the side lenses Clear (the photo) / Filled
--   S9  Texture    dash band and brown line rows On / Off
--   S10 Palette    Pastel (the photo) / Azure (chroma mirrored: a blue eye on a rose lattice)
--   S11 Source     Video / Osc
--   P12 Eye        the vesica's opening: a slit .. an eye filling the frame
--
-- Geometry in virtual 1080-line units (round at SD, 720p, 1080p).  Generated
-- avid; output conditioning (chroma 9-tap, luma [1 2 1] and 13/16) before the
-- blanking gate (the chroma 8-back sample and the sync delay ride two block-RAM
-- rings).  Latency 24 clocks in every mode.  Colours in the hardware
-- convention (u = Cr, v = Cb).  Builds with router1.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.vesica_pkg.all;
use work.all;

architecture vesica of program_top is
    constant C_LAT : integer := 24;

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
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_glide(g : unsigned(9 downto 0); d : signed(10 downto 0)) return unsigned is
        variable r : signed(10 downto 0);
    begin
        if d >= 8 or d <= -8 then
            r := signed('0' & g) + shift_right(d, 3);
        elsif d >= 4 then
            r := signed('0' & g) + 1;
        elsif d <= -4 then
            r := signed('0' & g) - 1;
        else
            r := signed('0' & g);
        end if;
        return unsigned(r(9 downto 0));
    end function;
    -- 2 xs w + xs^2 (DDA first difference of w^2 per pixel), xs in {2, 3, 4}
    function f_ddw(w : signed(12 downto 0); xs : unsigned(2 downto 0)) return signed is
        variable a : signed(15 downto 0);
    begin
        a := resize(w, 16);
        case to_integer(xs) is
            when 3      => return shift_left(a, 2) + shift_left(a, 1) + 9;
            when 4      => return shift_left(a, 3) + 16;
            when others => return shift_left(a, 2) + 4;
        end case;
    end function;

    ---------------------------------------------------------------------------
    -- ROMs
    ---------------------------------------------------------------------------
    signal s_pa : unsigned(8 downto 0) := (others => '0');
    signal s_pq_q : std_logic_vector(9 downto 0) := (others => '0');
    signal s_py_q, s_pu_q, s_pv_q : std_logic_vector(7 downto 0) := (others => '0');
    signal s_kn_a : unsigned(7 downto 0) := (others => '0');
    signal s_kn_q : std_logic_vector(11 downto 0) := (others => '0');
    signal s_gl_a, s_rw_a : unsigned(9 downto 0) := (others => '0');
    signal s_gl_q : std_logic_vector(7 downto 0) := (others => '0');
    signal s_rw_q : std_logic_vector(11 downto 0) := (others => '0');
    signal s_cr_q : std_logic_vector(23 downto 0) := (others => '0');
    signal s_kh_q, s_il_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s_k26_a : unsigned(8 downto 0) := (others => '0');
    signal s_k26_q : std_logic_vector(7 downto 0) := (others => '0');
    signal s_ey0_q, s_ey1_q, s_ey2_q, s_ey3_q, s_ey4_q : std_logic_vector(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- timing, controls
    ---------------------------------------------------------------------------
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone : std_logic := '0';
    signal s_zc : unsigned(10 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_g1, s_g2, s_g3, s_g4, s_g6, s_gp, s_gl_g : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_gl_t : std_logic_vector(9 downto 0) := (others => '0');
    signal s_gl_r : unsigned(9 downto 0) := (others => '0');
    signal s_gl_d : signed(10 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- per-field constants
    ---------------------------------------------------------------------------
    signal f_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal f_cls : unsigned(1 downto 0) := (others => '0');
    signal f_hx0 : unsigned(11 downto 0) := (others => '0');
    signal f_2xs2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal f_tear, f_copy, f_tex, f_pal, f_osc : std_logic := '0';
    signal f_sp : unsigned(8 downto 0) := (others => '0');
    signal f_sneg : std_logic := '0';
    signal f_phi : unsigned(18 downto 0) := (others => '0');       -- motion phase (all motions are slices)
    signal f_kh : unsigned(14 downto 0) := to_unsigned(8684, 15);
    signal f_kA : unsigned(16 downto 0) := to_unsigned(17368, 17);
    signal f_phA0 : unsigned(23 downto 0) := (others => '0');
    signal f_gm : unsigned(4 downto 0) := to_unsigned(25, 5);
    signal f_gsh : unsigned(1 downto 0) := to_unsigned(1, 2);
    signal f_invl : unsigned(11 downto 0) := to_unsigned(923, 12);
    signal f_dsl : unsigned(13 downto 0) := to_unsigned(923, 14);
    signal f_gem : unsigned(2 downto 0) := (others => '0');
    signal f_gee : unsigned(2 downto 0) := (others => '0');
    signal f_Rv2 : unsigned(19 downto 0) := to_unsigned(649132, 20);
    signal f_m : unsigned(5 downto 0) := (others => '0');
    signal f_per2 : unsigned(11 downto 0) := to_unsigned(1896, 12);
    signal f_per2h : unsigned(10 downto 0) := to_unsigned(948, 11);
    signal f_pmx, f_trV : unsigned(11 downto 0) := (others => '0');
    signal f_wh16 : unsigned(12 downto 0) := to_unsigned(2483, 13);
    signal f_bar16 : unsigned(11 downto 0) := (others => '0');
    signal f_D016 : unsigned(14 downto 0) := (others => '0');
    signal f_AX2 : unsigned(10 downto 0) := (others => '0');
    signal f_cs9 : unsigned(8 downto 0) := (others => '0');
    signal f_w0t, f_w0b : unsigned(8 downto 0) := (others => '0');
    signal f_cv, f_cc : signed(12 downto 0) := (others => '0');
    signal f_cpv, f_cpc : signed(2 downto 0) := (others => '0');
    signal f_u0V, f_u0C, f_uwV, f_uwC : signed(12 downto 0) := (others => '0');
    signal f_fw0V, f_ft0V, f_pos0V, f_fw0C : std_logic := '0';
    signal f_uw128m : signed(19 downto 0) := (others => '0');
    signal f_SVw, f_SCw, f_SC0 : unsigned(21 downto 0) := (others => '0');
    signal f_DVw, f_DCw, f_DC0 : signed(15 downto 0) := (others => '0');
    signal f_dslw : unsigned(15 downto 0) := (others => '0');
    signal f_stc, f_ds, f_trc : unsigned(15 downto 0);
    signal f_tx : unsigned(19 downto 0);

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    ---------------------------------------------------------------------------
    -- source side
    ---------------------------------------------------------------------------
    signal s_ox, s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_lu : unsigned(7 downto 0) := to_unsigned(128, 8);

    ---------------------------------------------------------------------------
    -- line engine
    ---------------------------------------------------------------------------
    signal e_kick, e_fk : std_logic := '0';
    signal e_st, e_ret : unsigned(5 downto 0) := (others => '0');
    signal e_mi : unsigned(4 downto 0) := (others => '0');
    signal e_ma, e_macc : unsigned(32 downto 0) := (others => '0');
    signal e_mb : unsigned(15 downto 0) := (others => '0');
    signal e_vy : unsigned(19 downto 0) := (others => '0');
    signal e_phB : unsigned(24 downto 0) := (others => '0');
    signal e_a16 : unsigned(13 downto 0) := (others => '0');
    signal e_yc : unsigned(14 downto 0) := (others => '0');
    signal e_inl, e_sgn, e_shm : std_logic := '0';
    signal e_rem : unsigned(13 downto 0) := (others => '0');
    signal e_root : unsigned(9 downto 0) := (others => '0');
    signal e_hw : unsigned(9 downto 0) := (others => '0');
    signal e_T : unsigned(5 downto 0) := (others => '0');
    signal e_inv : unsigned(16 downto 0) := (others => '0');
    signal e_Tr : unsigned(15 downto 0) := (others => '0');
    signal e_va : signed(20 downto 0) := (others => '0');
    signal e_wc : unsigned(6 downto 0) := (others => '0');
    signal e_bot : std_logic := '0';
    -- next-line results
    signal n_YVR7 : signed(14 downto 0) := (others => '0');
    signal n_YCR9 : signed(12 downto 0) := (others => '0');
    signal n_SV0 : unsigned(21 downto 0) := (others => '0');
    signal n_SVc : unsigned(13 downto 0) := (others => '0');
    signal n_DV0, n_DVc : signed(15 downto 0) := (others => '0');
    signal n_sst, n_nfrac, n_dsp : unsigned(15 downto 0) := (others => '0');
    signal n_cra, l_cra : unsigned(6 downto 0) := (others => '0');
    signal n_Fb : signed(14 downto 0) := (others => '0');
    signal n_lz, n_tl, n_bar, n_wing, n_axl : std_logic := '0';
    signal n_Dl2 : signed(12 downto 0) := (others => '0');
    signal n_bot : std_logic := '0';
    signal n_w0 : unsigned(8 downto 0) := (others => '0');
    -- current-line values
    signal l_YVR7 : signed(14 downto 0) := (others => '0');
    signal l_YCR9 : signed(12 downto 0) := (others => '0');
    signal l_SVc : unsigned(13 downto 0) := (others => '0');
    signal l_DVc : signed(15 downto 0) := (others => '0');
    signal l_nfrac, l_dsp : unsigned(15 downto 0) := (others => '0');
    signal l_Fb : signed(14 downto 0) := (others => '0');
    signal l_lz, l_tl, l_bar, l_wing, l_axl : std_logic := '0';
    signal l_Dl2 : signed(12 downto 0) := (others => '0');
    signal l_w0 : unsigned(8 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal c_phA : unsigned(23 downto 0) := (others => '0');
    signal c_hx2 : unsigned(11 downto 0) := (others => '0');
    signal p4_pq : unsigned(9 downto 0) := (others => '0');
    signal c_cV, c_cC : unsigned(11 downto 0) := (others => '0');
    signal c_ta, c_tb : signed(12 downto 0) := (others => '0');
    signal c_cpV, c_cpC : signed(2 downto 0) := (others => '0');
    signal c_posV, c_fwV, c_ftV, c_fwC : std_logic := '0';
    signal c_SV, c_SC : unsigned(21 downto 0) := (others => '0');
    signal c_DV, c_DC : signed(15 downto 0) := (others => '0');
    signal c_s : unsigned(15 downto 0) := (others => '0');

    -- stage 1
    signal p1_kd : signed(16 downto 0) := (others => '0');
    signal p1_t12, p2_t12, p3_t12, p4_A12, p5_A12 : unsigned(11 downto 0) := (others => '0');
    signal p3_kp : std_logic := '0';
    signal p1_dV : signed(15 downto 0) := (others => '0');
    signal p1_dC : signed(13 downto 0) := (others => '0');
    signal p5_PB : signed(15 downto 0) := (others => '0');
    signal st_sr : std_logic_vector(0 to 5) := (others => '0');
    signal pp_fw : std_logic_vector(1 to 6) := (others => '0');
    signal c_s6 : unsigned(15 downto 0) := (others => '0');
    signal p1_uC, p1_uV, p1_td : signed(12 downto 0) := (others => '0');
    signal p1_cenV, p1_cenC, p1_posV : std_logic := '0';
    -- stage 2
    signal p2_kp, p2_inV, p2_wing, p2_axis, p2_tipz : std_logic := '0';
    signal p2_dVs : signed(9 downto 0) := (others => '0');
    signal p2_dCs : signed(10 downto 0) := (others => '0');
    signal p2_auC : unsigned(11 downto 0) := (others => '0');
    signal p2_lin : signed(12 downto 0) := (others => '0');
    -- stage 3
    signal p3_saw1 : unsigned(15 downto 0) := (others => '0');
    signal p3_auC : unsigned(11 downto 0) := (others => '0');
    signal p3_rim, p3_halo, p3_inner, p3_t1, p3_t2, p3_inV : std_logic := '0';
    signal p3_dz, p3_lzr, p3_ech, p3_vis, p3_tipz, p3_ge0, p3_plate, p3_axis : std_logic := '0';
    signal p3_aD : unsigned(10 downto 0) := (others => '0');
    signal p3_d : signed(13 downto 0) := (others => '0');
    signal p3_lin : signed(12 downto 0) := (others => '0');
    signal p3_dCs : signed(10 downto 0) := (others => '0');
    -- stage 4..
    signal p4_saw2 : unsigned(15 downto 0) := (others => '0');
    signal p4_arc : unsigned(1 downto 0) := (others => '0');
    signal p4_mn : signed(12 downto 0) := (others => '0');
    signal p4_ad : unsigned(13 downto 0) := (others => '0');
    signal p5_kq : unsigned(3 downto 0) := (others => '0');
    signal p5_saw : unsigned(15 downto 0) := (others => '0');
    signal p5_mn : signed(12 downto 0) := (others => '0');
    signal p6_F2 : signed(15 downto 0) := (others => '0');
    signal p6_wq : signed(12 downto 0) := (others => '0');
    signal p6_fr1, p6_sq, p6_saw : unsigned(15 downto 0) := (others => '0');
    signal p6_E : signed(12 downto 0) := (others => '0');
    signal p7_F : signed(15 downto 0) := (others => '0');
    signal p7_fr, p7_saw, p7_se : unsigned(15 downto 0) := (others => '0');
    signal p7_cres : std_logic := '0';
    signal p7_c1 : unsigned(1 downto 0) := (others => '0');
    signal p7_df, p7_wq : signed(12 downto 0) := (others => '0');
    signal p8_aF : unsigned(11 downto 0) := (others => '0');
    signal p8_stroke, p8_sor : std_logic := '0';
    signal p8_sh : unsigned(1 downto 0) := (others => '0');
    signal p8_sd : unsigned(14 downto 0) := (others => '0');
    signal p9_prod : unsigned(16 downto 0) := (others => '0');
    signal p4_lu : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal p5_xm : signed(11 downto 0) := (others => '0');
    signal p9_lvl : unsigned(1 downto 0) := (others => '0');
    signal p10_idx : unsigned(5 downto 0) := (others => '0');
    signal p10_layer : unsigned(2 downto 0) := (others => '0');
    signal p10_code : unsigned(3 downto 0) := (others => '0');
    -- pipes of per-pixel bits
    type t_s16 is array (natural range <>) of signed(15 downto 0);
    type t_u16 is array (natural range <>) of unsigned(15 downto 0);
    type t_u2 is array (natural range <>) of unsigned(1 downto 0);
    type t_u4 is array (natural range <>) of unsigned(3 downto 0);
    signal pp_cenV, pp_cenC, pp_posV : std_logic_vector(2 to 10) := (others => '0');
    signal pp_inV, pp_rim, pp_halo, pp_inner, pp_t1, pp_t2, pp_plate, pp_axis : std_logic_vector(4 to 10) := (others => '0');
    signal pp_dz, pp_lzr : std_logic_vector(4 to 10) := (others => '0');
    signal pp_ge0 : std_logic_vector(4 to 6) := (others => '0');
    signal pp_arc : t_u2(5 to 10) := (others => (others => '0'));
    signal pp_dCs : t_s16(4 to 6) := (others => (others => '0'));
    signal pp_code : t_u4(11 to 13) := (others => (others => '0'));
    signal pp_sh : t_u2(9 to 13) := (others => (others => '0'));
    signal pp_cres : std_logic_vector(8 to 10) := (others => '0');
    signal pp_stroke, pp_sor : std_logic_vector(9 to 10) := (others => '0');
    -- colour stages
    signal p13_y, p13_u, p13_v : unsigned(7 downto 0) := (others => '0');
    signal p14_by, p14_bu, p14_bv : unsigned(7 downto 0) := (others => '0');
    signal p14_ky, p14_ku, p14_kv : unsigned(9 downto 0) := (others => '0');
    signal p14_al : unsigned(2 downto 0) := (others => '0');
    signal p15_y, p15_u, p15_v : unsigned(7 downto 0) := (others => '0');
    signal s_y12 : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_u12, s_v12 : unsigned(7 downto 0) := to_unsigned(128, 8);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    -- delay rings in block RAM: chroma 8 back (box filter), syncs + avid 22 back
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_ring4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal ram_cr : t_ring16 := (others => x"8080");
    signal ram_sy : t_ring4 := (others => "1110");
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_cr : signal is true;
    attribute no_rw_check of ram_sy : signal is true;
    signal s_wp : unsigned(7 downto 0) := (others => '0');
    signal s_rc_q, s_rc_f : std_logic_vector(15 downto 0) := x"8080";
    signal s_sy_q, s_sy_f, s_sy_1, s_sy_2 : std_logic_vector(3 downto 0) := "1110";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y13, s_y14, s_y15, s_y16, s_y17, s_y18x : unsigned(7 downto 0) := to_unsigned(16, 8);
    signal s_yf18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_gav_sr : std_logic_vector(0 to 5) := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered reads -> block RAM)
    ---------------------------------------------------------------------------
    p_rom_pal : process(clk)
    begin
        if rising_edge(clk) then
            s_py_q <= C_VES_PY(to_integer(s_pa));
            s_pu_q <= C_VES_PU(to_integer(s_pa));
            s_pv_q <= C_VES_PV(to_integer(s_pa));
        end if;
    end process p_rom_pal;

    p_rom_kn : process(clk)
    begin
        if rising_edge(clk) then
            s_kn_q <= C_VES_KNEE(to_integer(s_kn_a));
        end if;
    end process p_rom_kn;

    p_rom_ln : process(clk)
    begin
        if rising_edge(clk) then
            s_gl_q <= C_VES_GLT(to_integer(s_gl_a));
            s_rw_q <= C_VES_ROWS(to_integer(s_rw_a));
            s_cr_q <= C_VES_CRES(to_integer(l_cra));
            s_pq_q <= C_VES_PQ(to_integer(c_hx2(11 downto 3)));
        end if;
    end process p_rom_ln;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_wp <= s_wp + 1;
            ram_cr(to_integer(s_wp)) <= std_logic_vector(s_u12) & std_logic_vector(s_v12);
            s_rc_q <= ram_cr(to_integer(s_wp - 6));
            ram_sy(to_integer(s_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & g_avid;
            s_sy_q <= ram_sy(to_integer(s_wp - 20));
        end if;
    end process p_ring;

    p_rom_knob : process(clk)
    begin
        if rising_edge(clk) then
            s_kh_q <= C_VES_KH(to_integer(s_g1(9 downto 2)));
            s_il_q <= C_VES_IL(to_integer(s_g3(9 downto 2)));
            s_k26_q <= C_VES_K26(to_integer(s_k26_a));
            s_ey0_q <= C_VES_EYE0(to_integer(s_gp(9 downto 2)));
            s_ey1_q <= C_VES_EYE1(to_integer(s_gp(9 downto 2)));
            s_ey2_q <= C_VES_EYE2(to_integer(s_gp(9 downto 2)));
            s_ey3_q <= C_VES_EYE3(to_integer(s_gp(9 downto 2)));
            s_ey4_q <= C_VES_EYE4(to_integer(s_gp(9 downto 2)));
        end if;
    end process p_rom_knob;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_m : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 2047 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- zone clock 1: width, switches, timing class
            if s_zone = '1' and s_zc = 1 then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                s_w <= s_wmax; s_wmax <= (others => '0');
                f_tear <= not registers_in(6)(0);
                f_copy <= registers_in(6)(1);
                f_tex  <= not registers_in(6)(2);
                f_pal  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                f_xs <= s_xs;
                case to_integer(s_xs) is
                    when 3 =>
                        f_cls <= "01"; f_hx0 <= (others => '0'); f_2xs2 <= to_unsigned(18, 6);
                    when 4 =>
                        f_cls <= "10"; f_hx0 <= to_unsigned(480, 12); f_2xs2 <= to_unsigned(32, 6);
                    when others =>
                        f_cls <= "00"; f_hx0 <= (others => '0'); f_2xs2 <= to_unsigned(8, 6);
                end case;
            end if;
            -- zone clocks 4 .. 27: glides on one shared datapath (load, difference, ease, store)
            if s_zone = '1' and s_zc(10 downto 5) = 0 and s_zc(4 downto 2) /= 0 and s_zc(4 downto 2) /= 7 then
                case to_integer(s_zc(1 downto 0)) is
                    when 0 =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_gl_g <= s_g1; s_gl_t <= registers_in(0);
                            when 2      => s_gl_g <= s_g2; s_gl_t <= registers_in(1);
                            when 3      => s_gl_g <= s_g3; s_gl_t <= registers_in(2);
                            when 4      => s_gl_g <= s_g4; s_gl_t <= registers_in(3);
                            when 5      => s_gl_g <= s_g6; s_gl_t <= registers_in(5);
                            when others => s_gl_g <= s_gp; s_gl_t <= registers_in(7);
                        end case;
                    when 1 =>
                        s_gl_d <= signed('0' & unsigned(s_gl_t)) - signed('0' & s_gl_g);
                    when 2 =>
                        if s_warm = 1 then s_gl_r <= unsigned(s_gl_t); else s_gl_r <= f_glide(s_gl_g, s_gl_d); end if;
                    when others =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_g1 <= s_gl_r;
                            when 2      => s_g2 <= s_gl_r;
                            when 3      => s_g3 <= s_gl_r;
                            when 4      => s_g4 <= s_gl_r;
                            when 5      => s_g6 <= s_gl_r;
                            when others => s_gp <= s_gl_r;
                        end case;
                end case;
            end if;
            -- zone clocks 28 .. 37: the knob / eye ROMs (addressed by the glided knobs), direct terms, folds
            if s_zone = '1' and s_zc = 28 then
                f_cs9 <= s_g4(9 downto 1);
                s_k26_a <= '0' & s_g2(9 downto 2);
                v_s := signed('0' & unsigned(registers_in(4))) - 512;                 -- K5 Speed
                if v_s > 8 then f_sp <= unsigned(v_s(8 downto 0)) - 8; f_sneg <= '0';
                elsif v_s < -8 then f_sp <= unsigned(resize(-v_s, 9)) - 8; f_sneg <= '1';
                else f_sp <= (others => '0'); f_sneg <= '0'; end if;
            end if;
            if s_zone = '1' and s_zc = 29 then s_k26_a <= '1' & s_g6(9 downto 2); end if;
            if s_zone = '1' and s_zc = 30 then
                f_kh <= unsigned(s_kh_q(14 downto 0));
                f_invl <= unsigned(s_il_q(11 downto 0));
                f_gm <= unsigned(s_k26_q(4 downto 0)); f_gsh <= unsigned(s_k26_q(6 downto 5));
                f_Rv2 <= unsigned(s_ey1_q(15 downto 12)) & unsigned(s_ey0_q);
                f_AX2 <= unsigned(s_ey1_q(11 downto 1));
                f_m <= unsigned(s_ey3_q(3 downto 2)) & unsigned(s_ey2_q(2 downto 0)) & s_ey1_q(0);
                f_wh16 <= unsigned(s_ey2_q(15 downto 3));
                f_bar16 <= unsigned(s_ey3_q(15 downto 4));
                f_D016 <= unsigned(s_ey4_q(15 downto 1));
                -- crescent width (K4): top 0.75, bottom 0.828 x cs9
                f_w0t <= resize(shift_right(resize(f_cs9, 11) + shift_left(resize(f_cs9, 11), 1), 2), 9);
            end if;
            if s_zone = '1' and s_zc = 31 then
                f_gem <= unsigned(s_k26_q(2 downto 0)); f_gee <= unsigned(s_k26_q(5 downto 3));
                f_w0b <= f_w0t + resize(f_cs9(8 downto 4), 9) + resize(f_cs9(8 downto 6), 9);
                -- fold period 24 (m + 79) half-px
                v_m := resize(f_m, 12) + 79;
                f_per2 <= shift_left(v_m, 4) + shift_left(v_m, 3);
                case to_integer(f_xs) is
                    when 3      => f_kA <= resize(f_kh, 17) + shift_left(resize(f_kh, 17), 1);
                                   f_dsl <= resize(f_invl, 14) + resize(f_invl(11 downto 1), 14);
                    when 4      => f_kA <= shift_left(resize(f_kh, 17), 2);
                                   f_dsl <= shift_left(resize(f_invl, 14), 1);
                    when others => f_kA <= shift_left(resize(f_kh, 17), 1);
                                   f_dsl <= resize(f_invl, 14);
                end case;
            end if;
            if s_zone = '1' and s_zc = 32 then
                f_per2h <= f_per2(11 downto 1);
                f_pmx <= f_per2 - resize(f_xs, 12);
                f_trV <= resize(f_per2(11 downto 1), 12) + 1 - resize(f_xs, 12);
            end if;
            if s_zone = '1' and s_zc = 33 then
                f_cv <= signed('0' & f_hx0) + signed(resize(f_per2h, 13)) - 2027;
                f_cc <= signed('0' & f_hx0) + signed(resize(f_per2h, 13)) - 1995;
                f_cpv <= (others => '0'); f_cpc <= (others => '0');
                f_uwV <= 1 - signed(resize(f_per2h, 13));
                if f_cls = "01" then f_uwC <= -signed(resize(f_per2h, 13)); else f_uwC <= 1 - signed(resize(f_per2h, 13)); end if;
            end if;
            if s_zone = '1' and (s_zc = 34 or s_zc = 35) then
                if f_cv < 0 then f_cv <= f_cv + signed('0' & f_per2); f_cpv <= f_cpv - 1; end if;
                if f_cc < 0 then f_cc <= f_cc + signed('0' & f_per2); f_cpc <= f_cpc - 1; end if;
            end if;
            if s_zone = '1' and s_zc = 36 then
                f_u0V <= f_cv - signed(resize(f_per2h, 13));
                f_u0C <= f_cc - signed(resize(f_per2h, 13));
                if unsigned(f_cv(11 downto 0)) >= f_pmx then f_fw0V <= '1'; else f_fw0V <= '0'; end if;
                if unsigned(f_cv(11 downto 0)) = f_trV then f_ft0V <= '1'; else f_ft0V <= '0'; end if;
                if f_cv > signed(resize(f_per2h, 13)) then f_pos0V <= '1'; else f_pos0V <= '0'; end if;
                if unsigned(f_cc(11 downto 0)) >= f_pmx then f_fw0C <= '1'; else f_fw0C <= '0'; end if;
                f_uw128m <= shift_left(resize(f_uwV, 20), 7) - 6912;
            end if;
            if s_zone = '1' and s_zc = 37 then
                f_DVw <= f_ddw(f_uwV, f_xs);
                f_DCw <= f_ddw(f_uwC, f_xs);
                f_DC0 <= f_ddw(f_u0C, f_xs);
            end if;
            if s_zone = '1' and s_zc = 38 then          -- magnitudes for the engine's squares
                f_uwV <= abs(f_uwV); f_uwC <= abs(f_uwC); f_u0C <= abs(f_u0C);
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to 5 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
            g_wm1 <= s_w - 1;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid
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
    -- Source side: the camera luma or the Osc oscillator, 8 bits
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zone = '1' and s_zc = 1 then s_ot <= s_ot + 3; end if;
            s_osum <= resize(f_tri8(s_ox(9 downto 2) + s_ot), 8) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 8);
            v_l := to_unsigned(502 - 222, 10) + resize(s_osum, 10) + resize(s_osum(7 downto 2), 10)
                   + resize(s_osum(7 downto 1), 10);
            if f_osc = '1' then
                s_lu <= v_l(9 downto 2);
            else
                s_lu <= unsigned(data_in.y(9 downto 2));
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: the next line's constants
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_t : unsigned(13 downto 0);
        variable v_d : unsigned(10 downto 0);
        variable v_s : signed(21 downto 0);
        variable v_u : unsigned(15 downto 0);
        variable v_w : signed(12 downto 0);
        variable v_n : unsigned(19 downto 0);
        variable v_q : unsigned(6 downto 0);
        variable v_ad : signed(15 downto 0);
        variable v_y : unsigned(14 downto 0);
        variable v_hc : unsigned(10 downto 0);
        variable v_b : unsigned(15 downto 0);
        variable v_ph : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            e_kick <= '0';
            e_fk <= '0';
            if s_zone = '1' and s_zc = 40 then
                e_fk <= '1';
            end if;
            if s_zone = '1' and s_zc = 300 then
                e_vy <= (others => '0');
                e_kick <= '1';
            elsif g_start = '1' then
                e_kick <= '1';
            end if;
            if g_start = '1' then
                l_YVR7 <= n_YVR7; l_YCR9 <= n_YCR9; l_SVc <= n_SVc; l_DVc <= n_DVc;
                l_nfrac <= n_nfrac; l_dsp <= n_dsp; l_cra <= n_cra;
                l_Fb <= n_Fb; l_lz <= n_lz; l_tl <= n_tl; l_bar <= n_bar; l_wing <= n_wing; l_axl <= n_axl;
                l_Dl2 <= n_Dl2; l_w0 <= n_w0;
            end if;

            case to_integer(e_st) is
                when 0 =>
                    if e_fk = '1' then
                        -- field init: speed^2 into the motion phase, lattice steps / phases, fold squares
                        e_ma <= resize(f_sp, 33); e_ret <= to_unsigned(41, 6); e_st <= to_unsigned(62, 6);
                    elsif e_kick = '1' then
                        v_ad := abs(signed('0' & e_vy(18 downto 4)) - 8350);
                        e_a16 <= unsigned(v_ad(13 downto 0));
                        e_st <= to_unsigned(1, 6);
                    end if;
                when 1 =>
                    e_ma <= resize(e_a16, 33); e_ret <= to_unsigned(2, 6); e_st <= to_unsigned(62, 6);
                when 2 =>
                    e_ma <= resize(e_macc(27 downto 12), 33); e_mb <= to_unsigned(670, 16);
                    e_macc <= (others => '0'); e_mi <= (others => '0'); e_ret <= to_unsigned(3, 6); e_st <= to_unsigned(63, 6);
                when 3 =>
                    -- the vertical waveform Y = d + a - c a^2 for the two circle families (Q4)
                    v_y := resize(e_a16, 15) - resize(e_macc(27 downto 12), 15);
                    e_yc <= v_y + 15645;
                    e_va <= signed(resize(v_y + 10645, 21));
                    e_ret <= to_unsigned(4, 6); e_st <= to_unsigned(20, 6);
                when 4 =>
                    v_s := signed(resize(e_macc(28 downto 8), 22)) - signed(resize(f_Rv2, 22));
                    n_YVR7 <= v_s(21 downto 7);
                    e_va <= v_s(20 downto 0);
                    e_ma <= resize(e_yc, 33); e_ret <= to_unsigned(5, 6); e_st <= to_unsigned(62, 6);
                when 5 =>
                    v_s := signed(resize(e_macc(29 downto 8), 22)) - C_VES_RC2;
                    n_YCR9 <= v_s(21 downto 9);
                    -- lens half-width hw = sqrt(R^2 - Y^2): radicand in e_macc(19 .. 0)
                    if e_va < 0 then e_inl <= '1'; v_s := -resize(e_va, 22); e_macc <= resize(unsigned(v_s(19 downto 0)), 33);
                    else e_inl <= '0'; e_macc <= (others => '0'); end if;
                    e_rem <= (others => '0'); e_root <= (others => '0'); e_mi <= (others => '0');
                    e_st <= to_unsigned(6, 6);
                when 6 =>
                    v_t := e_rem(11 downto 0) & e_macc(19 downto 18);
                    e_macc <= e_macc(30 downto 0) & "00";
                    if v_t >= resize(e_root & "01", 14) then
                        e_rem <= v_t - resize(e_root & "01", 14);
                        e_root <= e_root(8 downto 0) & '1';
                    else
                        e_rem <= v_t;
                        e_root <= e_root(8 downto 0) & '0';
                    end if;
                    if e_mi = 9 then e_st <= to_unsigned(7, 6); end if;
                    e_mi <= e_mi + 1;
                when 7 =>
                    e_hw <= e_root;
                    -- tear sawtooth phase (hw - 10) / 36.6 cycles (Q16): hw x 1792 - 17920
                    e_Tr <= resize(shift_left(resize(e_root, 16), 10) + shift_left(resize(e_root, 16), 9)
                                   + shift_left(resize(e_root, 16), 8), 16) - 17920;
                    e_st <= to_unsigned(8, 6);
                when 8 =>
                    -- tooth depth T = 7 .. 51 px: rises over 45 % of the cycle, then holds
                    v_u := e_Tr - f_trc;
                    v_q := resize(v_u(15 downto 10), 7) + resize(v_u(15 downto 11), 7);
                    if v_q > 44 then v_q := to_unsigned(44, 7); end if;
                    if e_inl = '1' and f_tear = '1' then e_T <= resize(v_q, 6) + 7; else e_T <= (others => '0'); end if;
                    -- stripe phase at the line's first pixel: (u0 / 2 + hw - 27) / pitch
                    e_va <= resize(shift_left(resize(f_u0V, 21), 7) + signed(resize(shift_left(resize(e_hw, 20), 8), 21)) - 6912, 21);
                    e_st <= to_unsigned(9, 6);
                when 9 =>
                    e_sgn <= e_va(20);
                    e_va <= abs(e_va);
                    e_st <= to_unsigned(21, 6);
                when 21 =>
                    e_ma <= resize(unsigned(e_va), 33); e_mb <= resize(f_invl, 16);
                    e_macc <= (others => '0'); e_mi <= (others => '0'); e_ret <= to_unsigned(10, 6); e_st <= to_unsigned(63, 6);
                when 10 =>
                    if e_sgn = '1' then v_u := 0 - e_macc(23 downto 8); else v_u := e_macc(23 downto 8); end if;
                    n_sst <= v_u - f_stc;
                    e_wc <= shift_left(resize(e_T, 7), 1) + 1;
                    e_ma <= resize(shift_left(resize(e_T, 7), 1) + 1, 33); e_ret <= to_unsigned(11, 6); e_st <= to_unsigned(62, 6);
                when 11 =>
                    n_SVc <= e_macc(13 downto 0);
                    n_DVc <= f_ddw(signed(resize(e_wc, 13)), f_xs);
                    if f_pos0V = '1' then v_w := f_u0V + signed(resize(shift_left(resize(e_T, 7), 1), 13)); else v_w := f_u0V; end if;
                    n_DV0 <= f_ddw(v_w, f_xs);
                    e_va <= resize(v_w, 21);
                    e_st <= to_unsigned(22, 6);
                when 22 =>
                    e_va <= abs(e_va);
                    e_ret <= to_unsigned(12, 6); e_st <= to_unsigned(20, 6);
                when 12 =>
                    n_SV0 <= e_macc(21 downto 0);
                    s_rw_a <= e_a16(13 downto 4);
                    if e_vy(18 downto 4) < 8350 then e_bot <= '0'; else e_bot <= '1'; end if;
                    if e_a16(13 downto 7) > 63 then v_t := to_unsigned(63, 14); else v_t := resize(e_a16(13 downto 7), 14); end if;
                    if e_vy(18 downto 4) < 8350 then n_cra <= '0' & v_t(5 downto 0); else n_cra <= '1' & v_t(5 downto 0); end if;
                    -- plate wing limit 2 (D0 - 1.53 a) / 16
                    e_va <= resize(signed('0' & f_D016) - signed(resize(e_a16, 16)) - signed(resize(e_a16(13 downto 1), 16))
                            - signed(resize(e_a16(13 downto 5), 16)), 21);
                    -- lattice vertical phase: by + 1/2 + kBv (y - 519.06)
                    v_ad := signed('0' & e_vy(18 downto 4)) - 8305;
                    e_sgn <= v_ad(15);
                    v_ad := abs(v_ad);
                    e_Tr <= unsigned(v_ad);
                    e_st <= to_unsigned(23, 6);
                when 23 =>
                    e_ma <= resize(e_inv, 33); e_mb <= e_Tr;
                    e_macc <= (others => '0'); e_mi <= (others => '0'); e_ret <= to_unsigned(13, 6); e_st <= to_unsigned(63, 6);
                when 13 =>
                    if e_sgn = '1' then v_ph := 0 - e_macc(28 downto 4); else v_ph := e_macc(28 downto 4); end if;
                    v_ph := v_ph + (f_phi(18 downto 0) & "000000") + to_unsigned(8388608, 25);
                    e_phB <= v_ph;
                    s_gl_a <= v_ph(24 downto 15);
                    e_st <= to_unsigned(14, 6);
                when 14 =>
                    e_st <= to_unsigned(15, 6);
                when 15 =>
                    n_Dl2 <= resize(shift_right(e_va, 3), 13);
                    -- lattice row B (triangle of the vertical phase)
                    if e_phB(23) = '1' then v_b := e_phB(22 downto 7); else v_b := not e_phB(22 downto 7); end if;
                    n_Fb <= signed(resize(v_b(15 downto 4), 15)) + signed(resize(v_b(15 downto 9), 15)) - 4301
                            - signed(resize(unsigned(s_gl_q) & "0000", 15));
                    v_n := shift_left(resize(unsigned(s_rw_q), 20), 12) + f_tx;
                    n_nfrac <= v_n(15 downto 0);
                    n_dsp <= f_ds + shift_left(resize(v_n(16 downto 16), 16), 15);
                    if v_n(15 downto 0) > 52429 or v_n(15 downto 0) < 13107 then n_lz <= f_tex; else n_lz <= '0'; end if;
                    if e_a16 < f_bar16 then n_bar <= '1'; else n_bar <= '0'; end if;
                    if e_a16 < f_wh16 then n_wing <= '1'; else n_wing <= '0'; end if;
                    if e_a16 < 19 then n_axl <= '1'; else n_axl <= '0'; end if;
                    if e_T /= 0 then n_tl <= '1'; else n_tl <= '0'; end if;
                    n_bot <= e_bot;
                    if e_bot = '0' then n_w0 <= f_w0t; else n_w0 <= f_w0b; end if;
                    e_vy <= e_vy + resize(s_ys, 20);
                    e_st <= to_unsigned(0, 6);
                -- field init
                when 41 =>
                    if f_sneg = '1' then f_phi <= f_phi - resize(e_macc(17 downto 8), 19);
                    else f_phi <= f_phi + resize(e_macc(17 downto 8), 19); end if;
                    e_ma <= resize(f_kh, 33); e_mb <= to_unsigned(479, 16);
                    e_macc <= (others => '0'); e_mi <= (others => '0'); e_ret <= to_unsigned(42, 6); e_st <= to_unsigned(63, 6);
                when 42 =>
                    e_inv <= e_macc(23 downto 7);                    -- kBv, the lattice phase step per virtual line
                    e_ma <= resize(f_kh, 33); e_mb <= to_unsigned(1944, 16) - resize(f_hx0, 16);
                    e_macc <= (others => '0'); e_mi <= (others => '0'); e_ret <= to_unsigned(43, 6); e_st <= to_unsigned(63, 6);
                when 43 =>
                    f_phA0 <= (f_phi(17 downto 0) & "000000") - e_macc(23 downto 0);
                    e_ma <= resize(unsigned(f_u0C), 33); e_ret <= to_unsigned(44, 6); e_st <= to_unsigned(62, 6);
                when 44 =>
                    f_SC0 <= e_macc(21 downto 0);
                    e_ma <= resize(unsigned(f_uwV), 33); e_ret <= to_unsigned(45, 6); e_st <= to_unsigned(62, 6);
                when 45 =>
                    f_SVw <= e_macc(21 downto 0);
                    e_ma <= resize(unsigned(f_uwC), 33); e_ret <= to_unsigned(46, 6); e_st <= to_unsigned(62, 6);
                when 46 =>
                    f_SCw <= e_macc(21 downto 0);
                    e_ma <= resize(f_per2, 33); e_mb <= resize(f_invl, 16);
                    e_macc <= (others => '0'); e_mi <= (others => '0'); e_ret <= to_unsigned(47, 6); e_st <= to_unsigned(63, 6);
                when 47 =>
                    -- stripe phase step across a fold wrap: dsl - per2 invl / 2
                    f_dslw <= resize(f_dsl, 16) - e_macc(16 downto 1);
                    e_st <= to_unsigned(0, 6);
                when 20 =>
                    e_ma <= resize(unsigned(e_va), 33); e_st <= to_unsigned(62, 6);
                when 62 =>
                    -- square: B = A
                    e_mb <= e_ma(15 downto 0);
                    e_macc <= (others => '0'); e_mi <= (others => '0'); e_st <= to_unsigned(63, 6);
                when 63 =>
                    -- 16-step serial multiply, then back to e_ret
                    if e_mb(0) = '1' then e_macc <= e_macc + e_ma; end if;
                    e_ma <= e_ma(31 downto 0) & '0';
                    e_mb <= '0' & e_mb(15 downto 1);
                    if e_mi = 15 then e_st <= e_ret; end if;
                    e_mi <= e_mi + 1;
                when others =>
                    e_st <= to_unsigned(0, 6);
            end case;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_c : unsigned(11 downto 0);
        variable v_s : signed(22 downto 0);
        variable v_a : signed(12 downto 0);
        variable v_e : signed(18 downto 0);
        variable v_x9 : signed(9 downto 0);
        variable v_xm : signed(11 downto 0);
        variable v_a14 : signed(13 downto 0);
        variable v_i : unsigned(17 downto 0);
        variable v_n : integer range 0 to 7;
        variable v_code : unsigned(3 downto 0);
        variable v_ky, v_ku, v_kv : unsigned(9 downto 0);
        variable v_by, v_bu, v_bv : unsigned(9 downto 0);
        variable v_b8y, v_b8u, v_b8v : unsigned(7 downto 0);
        variable v_sc : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_sw : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: beam counters (aligned with g_avid)
            if g_start = '1' then
                c_phA <= f_phA0;
                c_cV <= unsigned(f_cv(11 downto 0)); c_cpV <= f_cpv; c_posV <= f_pos0V; c_fwV <= f_fw0V; c_ftV <= f_ft0V;
                c_SV <= n_SV0; c_DV <= n_DV0;
                c_cC <= unsigned(f_cc(11 downto 0)); c_cpC <= f_cpc; c_fwC <= f_fw0C; c_SC <= f_SC0; c_DC <= f_DC0;
                -- crescent taper: half-px from the tip, rising (ta) and falling (tb) through the crescent
                if n_bot = '0' then
                    c_ta <= signed('0' & f_hx0) - 1210; c_tb <= 2690 - signed('0' & f_hx0);
                else
                    c_ta <= signed('0' & f_hx0) - 1200; c_tb <= 2736 - signed('0' & f_hx0);
                end if;
            elsif g_avid = '1' then
                c_phA <= c_phA + resize(f_kA, 24);
                -- lens family V: fold, tear start, X^2 DDA, stripe phase
                if c_fwV = '1' then
                    v_c := c_cV - f_pmx;
                    c_cpV <= c_cpV + 1; c_posV <= '0';
                    c_SV <= f_SVw; c_DV <= f_DVw;
                elsif c_ftV = '1' then
                    v_c := c_cV + resize(f_xs, 12);
                    c_posV <= '1';
                    c_SV <= resize(l_SVc, 22); c_DV <= l_DVc;
                else
                    v_c := c_cV + resize(f_xs, 12);
                    c_SV <= c_SV + unsigned(resize(c_DV, 22));
                    c_DV <= c_DV + signed(resize(f_2xs2, 16));
                end if;
                c_cV <= v_c;
                if v_c >= f_pmx then c_fwV <= '1'; else c_fwV <= '0'; end if;
                if v_c = f_trV then c_ftV <= '1'; else c_ftV <= '0'; end if;
                -- crescent family C
                if c_fwC = '1' then
                    v_c := c_cC - f_pmx;
                    c_cpC <= c_cpC + 1;
                    c_SC <= f_SCw; c_DC <= f_DCw;
                else
                    v_c := c_cC + resize(f_xs, 12);
                    c_SC <= c_SC + unsigned(resize(c_DC, 22));
                    c_DC <= c_DC + signed(resize(f_2xs2, 16));
                end if;
                c_cC <= v_c;
                if v_c >= f_pmx then c_fwC <= '1'; else c_fwC <= '0'; end if;
                c_ta <= c_ta + signed(resize(f_xs, 13));
                c_tb <= c_tb - signed(resize(f_xs, 13));
            end if;

            -- late counters: the lattice parabola at stage 4, the stripe phase at stage 6
            st_sr(0) <= g_start; for k in 1 to 5 loop st_sr(k) <= st_sr(k - 1); end loop;
            pp_fw(1) <= c_fwV;
            for k in 2 to 6 loop pp_fw(k) <= pp_fw(k - 1); end loop;
            -- beam x at stage 2 (half-px), for the parabola ROM
            if st_sr(1) = '1' then
                c_hx2 <= f_hx0;
            elsif s_gav_sr(1) = '1' then
                c_hx2 <= c_hx2 + resize(f_xs, 12);
            end if;
            p4_pq <= unsigned(s_pq_q);
            if st_sr(5) = '1' then
                c_s6 <= n_sst;
            elsif s_gav_sr(5) = '1' then
                if pp_fw(6) = '1' then c_s6 <= c_s6 + f_dslw; else c_s6 <= c_s6 + resize(f_dsl, 16); end if;
            end if;

            -- 1
            if c_phA(23) = '1' then
                p1_t12 <= c_phA(22 downto 11);
                p1_kd <= signed('0' & c_phA(22 downto 7)) - 59755;
            else
                p1_t12 <= not c_phA(22 downto 11);
                p1_kd <= signed('0' & (not c_phA(22 downto 7))) - 59755;
            end if;
            p1_dV <= signed(resize(c_SV(21 downto 9), 16)) + resize(l_YVR7, 16);
            p1_dC <= signed(resize(c_SC(21 downto 11), 14)) + resize(l_YCR9, 14);
            p1_uC <= signed('0' & c_cC) - signed(resize(f_per2h, 13));
            p1_uV <= signed('0' & c_cV) - signed(resize(f_per2h, 13));
            if c_ta < c_tb then p1_td <= c_ta; else p1_td <= c_tb; end if;
            if c_cpV = 0 or f_copy = '1' then p1_cenV <= '1'; else p1_cenV <= '0'; end if;
            if c_cpC = 0 then p1_cenC <= '1'; else p1_cenC <= '0'; end if;
            p1_posV <= c_posV;

            -- 2
            if p1_kd > 0 then s_kn_a <= unsigned(p1_kd(12 downto 5)); p2_kp <= '1';
            else s_kn_a <= (others => '0'); p2_kp <= '0'; end if;
            p2_t12 <= p1_t12;
            p2_inV <= p1_dV(15);
            if p1_dV < -512 then p2_dVs <= to_signed(-512, 10);
            elsif p1_dV > 511 then p2_dVs <= to_signed(511, 10);
            else p2_dVs <= p1_dV(9 downto 0); end if;
            if p1_dC < -1023 then p2_dCs <= to_signed(-1023, 11);
            elsif p1_dC > 1023 then p2_dCs <= to_signed(1023, 11);
            else p2_dCs <= p1_dC(10 downto 0); end if;
            p2_auC <= unsigned(resize(abs(p1_uC), 12));
            -- taper 0.866 dCs units per half-px
            p2_lin <= p1_td - shift_right(p1_td, 3) - shift_right(p1_td, 7);
            if p1_td < 10 and p1_td > -180 then p2_tipz <= '1'; else p2_tipz <= '0'; end if;
            if l_wing = '1' and p1_posV = '1' and p1_uV < l_Dl2 then p2_wing <= '1'; else p2_wing <= '0'; end if;
            if l_axl = '1' and (p1_uV > signed(resize(f_AX2, 13)) or p1_uV < -signed(resize(f_AX2, 13))) then p2_axis <= '1'; else p2_axis <= '0'; end if;
            pp_cenV(2) <= p1_cenV; pp_cenC(2) <= p1_cenC; pp_posV(2) <= p1_posV;
            for k in 3 to 10 loop
                pp_cenV(k) <= pp_cenV(k - 1); pp_cenC(k) <= pp_cenC(k - 1); pp_posV(k) <= pp_posV(k - 1);
            end loop;

            -- 3
            p3_auC <= p2_auC;
            p3_t12 <= p2_t12; p3_kp <= p2_kp;
            p3_saw1 <= shift_left(resize(p2_auC, 16), 10) + shift_left(resize(p2_auC, 16), 6);
            p3_inV <= p2_inV;
            if p2_dVs >= -C_VES_TV_RIM and p2_dVs <= C_VES_TV_RIM then p3_rim <= '1'; else p3_rim <= '0'; end if;
            if p2_dVs > 0 and p2_dVs < C_VES_TV_HALO then p3_halo <= '1'; else p3_halo <= '0'; end if;
            if p2_dVs >= -C_VES_TV_INN and p2_dVs < -C_VES_TV_RIM then p3_inner <= '1'; else p3_inner <= '0'; end if;
            if p2_dVs > -C_VES_TV_T1 then p3_t1 <= '1'; else p3_t1 <= '0'; end if;
            if p2_dVs > -C_VES_TV_T2 then p3_t2 <= '1'; else p3_t2 <= '0'; end if;
            if (p2_dCs > C_VES_TC_D0 and p2_dCs < C_VES_TC_D1) or p2_dCs > C_VES_TC_D2 then p3_dz <= '1'; else p3_dz <= '0'; end if;
            if p2_dCs >= C_VES_TC_D1 and p2_dCs <= C_VES_TC_D2 then p3_lzr <= '1'; else p3_lzr <= '0'; end if;
            if p2_dCs > -C_VES_TC_E1 and p2_dCs < -C_VES_TC_E0 then p3_ech <= '1'; else p3_ech <= '0'; end if;
            p3_aD <= unsigned(resize(abs(p2_dCs), 11));
            if p2_auC > 134 then p3_vis <= '1'; else p3_vis <= '0'; end if;
            p3_tipz <= p2_tipz;
            p3_d <= resize(p2_lin, 14) - signed(resize('0' & l_w0, 14));
            p3_lin <= p2_lin;
            if p2_dCs >= 0 then p3_ge0 <= '1'; else p3_ge0 <= '0'; end if;
            p3_dCs <= p2_dCs;
            if (l_bar = '1' or p2_wing = '1') and p2_inV = '0' then p3_plate <= '1'; else p3_plate <= '0'; end if;
            if (l_bar = '1' or p2_wing = '1') and p2_inV = '0' and p2_axis = '1' then p3_axis <= '1'; else p3_axis <= '0'; end if;

            -- 4
            if p3_kp = '1' then p4_A12 <= unsigned(s_kn_q); else p4_A12 <= p3_t12; end if;
            p4_saw2 <= p3_saw1 + shift_left(resize(p3_auC, 16), 2);
            if p3_tipz = '1' then
                if p3_aD <= C_VES_TC_T1 then p4_arc <= "10"; elsif p3_aD <= C_VES_TC_T2 then p4_arc <= "01"; else p4_arc <= "00"; end if;
            else
                if p3_aD <= C_VES_TC_A1 then p4_arc <= "10"; elsif p3_aD <= C_VES_TC_A2 then p4_arc <= "01"; else p4_arc <= "00"; end if;
            end if;
            if p3_ech = '1' and p3_aD > C_VES_TC_A2 and not (p3_tipz = '1' and p3_aD <= C_VES_TC_T2) then p4_arc <= "01"; end if;
            if p3_vis = '0' then p4_arc <= "00"; end if;
            v_a14 := abs(p3_d); p4_ad <= unsigned(v_a14);
            if p3_d(13) = '1' then p4_mn <= p3_lin; else p4_mn <= signed(resize(l_w0, 13)); end if;
            pp_inV(4) <= p3_inV; pp_rim(4) <= p3_rim; pp_halo(4) <= p3_halo and p3_plate; pp_inner(4) <= p3_inner;
            pp_t1(4) <= p3_t1; pp_t2(4) <= p3_t2; pp_plate(4) <= p3_plate; pp_axis(4) <= p3_axis;
            pp_dz(4) <= p3_dz and f_tex; pp_lzr(4) <= p3_lzr; pp_ge0(4) <= p3_ge0;
            pp_dCs(4) <= resize(p3_dCs, 16);
            for k in 5 to 10 loop
                pp_inV(k) <= pp_inV(k - 1); pp_rim(k) <= pp_rim(k - 1); pp_halo(k) <= pp_halo(k - 1);
                pp_inner(k) <= pp_inner(k - 1); pp_t1(k) <= pp_t1(k - 1); pp_t2(k) <= pp_t2(k - 1);
                pp_plate(k) <= pp_plate(k - 1); pp_axis(k) <= pp_axis(k - 1); pp_dz(k) <= pp_dz(k - 1);
                pp_lzr(k) <= pp_lzr(k - 1);
            end loop;
            pp_ge0(5) <= pp_ge0(4); pp_ge0(6) <= pp_ge0(5);
            pp_dCs(5) <= pp_dCs(4); pp_dCs(6) <= pp_dCs(5);

            -- 5
            p5_A12 <= p4_A12;
            p5_PB <= signed(resize(p4_pq, 16)) + resize(l_Fb, 16);
            p5_saw <= p4_saw2 + l_dsp;
            p5_mn <= p4_mn;
            -- the crescent's shoulder: (56 - |d|)^2 / 224 rounds the taper into the full width
            if p4_ad < 64 then p5_kq <= to_unsigned(C_VES_KQ(to_integer(p4_ad(5 downto 0))), 4); else p5_kq <= (others => '0'); end if;
            p4_lu <= s_lu;
            -- etch: (luma - 128) x mantissa, then x 2^e / 16 at stage 6
            v_x9 := signed(resize(p4_lu, 10)) - 128;
            v_xm := (others => '0');
            if f_gem(2) = '1' then v_xm := v_xm + shift_left(resize(v_x9, 12), 2); end if;
            if f_gem(1) = '1' then v_xm := v_xm + shift_left(resize(v_x9, 12), 1); end if;
            if f_gem(0) = '1' then v_xm := v_xm + resize(v_x9, 12); end if;
            p5_xm <= v_xm;
            pp_arc(5) <= p4_arc; for k in 6 to 10 loop pp_arc(k) <= pp_arc(k - 1); end loop;

            -- 6
            p6_F2 <= signed(resize(p5_A12, 16)) + p5_PB;
            p6_wq <= p5_mn - signed(resize(p5_kq, 13));
            p6_fr1 <= l_nfrac - shift_right(p5_saw, 1);
            p6_sq <= shift_right(p5_saw, 5) + shift_right(p5_saw, 6);
            p6_saw <= p5_saw;
            v_e := shift_left(resize(p5_xm, 19), to_integer(f_gee));
            p6_E <= resize(shift_right(v_e, 4), 13);

            -- 7
            p7_F <= p6_F2 + resize(p6_E, 16);
            p7_fr <= p6_fr1 - p6_sq;
            p7_saw <= p6_saw;
            if pp_cenC(6) = '1' and pp_ge0(6) = '1' and pp_dCs(6) < resize(p6_wq, 16) then p7_cres <= '1'; else p7_cres <= '0'; end if;
            if pp_dCs(6) < 24 then p7_c1 <= "01"; else p7_c1 <= "00"; end if;
            p7_df <= resize(p6_wq - resize(pp_dCs(6), 13), 13);
            p7_wq <= p6_wq;
            p7_se <= c_s6 + unsigned(shift_left(resize(p6_E, 16), 4));

            -- 8
            if p7_F < 0 then v_s := resize(-p7_F, 23); else v_s := resize(p7_F, 23); end if;
            if v_s > 4095 then p8_aF <= (others => '1'); else p8_aF <= unsigned(v_s(11 downto 0)); end if;
            if pp_dz(7) = '1' and p7_fr(15) = '0' and p7_saw < 51118 then p8_stroke <= '1'; else p8_stroke <= '0'; end if;
            if p7_fr < 15729 then p8_sor <= '1'; else p8_sor <= '0'; end if;
            -- crescent shading: darker along the inner arc, the outer rim and the thin tips
            if p7_c1(0) = '1' or p7_df < 16 or p7_wq < 100 then p8_sh <= "10"; else p8_sh <= "00"; end if;
            if p7_se(15) = '1' then p8_sd <= not p7_se(14 downto 0); else p8_sd <= p7_se(14 downto 0); end if;
            pp_cres(8) <= p7_cres; pp_cres(9) <= pp_cres(8); pp_cres(10) <= pp_cres(9);

            -- 9
            p9_prod <= p8_aF * f_gm;
            if p8_sd < 2949 then p9_lvl <= "11"; elsif p8_sd < 6226 then p9_lvl <= "10";
            elsif p8_sd < 9830 then p9_lvl <= "01"; else p9_lvl <= "00"; end if;
            pp_stroke(9) <= p8_stroke; pp_sor(9) <= p8_sor; pp_stroke(10) <= pp_stroke(9); pp_sor(10) <= pp_sor(9);
            pp_sh(9) <= p8_sh; for k in 10 to 13 loop pp_sh(k) <= pp_sh(k - 1); end loop;

            -- 10: palette index, layer, blend code
            case to_integer(f_gsh) is
                when 2      => v_i := resize(shift_right(p9_prod, 9), 18);
                when 1      => v_i := resize(shift_right(p9_prod, 10), 18);
                when others => v_i := resize(shift_right(p9_prod, 11), 18);
            end case;
            if v_i > 63 then p10_idx <= to_unsigned(63, 6); else p10_idx <= v_i(5 downto 0); end if;
            if pp_inV(9) = '1' and pp_cenV(9) = '1' then
                p10_layer <= '1' & p9_lvl;
            elsif pp_stroke(9) = '1' then
                if pp_sor(9) = '1' then p10_layer <= "011"; else p10_layer <= "010"; end if;
            elsif pp_plate(9) = '1' or (pp_lzr(9) = '1' and l_lz = '1') then
                p10_layer <= "001";
            else
                p10_layer <= "000";
            end if;
            -- blend code: 0 none, 1 halo, 2 axis, 3 crescent, 4 arc half, 5 arc, 6 tail 1/4,
            -- 7 tail 3/4, 8 rim (right copies), 9 rim, 10 inner rim
            v_code := (others => '0');
            if pp_inV(9) = '0' or pp_cenV(9) = '0' then
                if pp_halo(9) = '1' then v_code := to_unsigned(1, 4); end if;
                if pp_axis(9) = '1' then v_code := to_unsigned(2, 4); end if;
                if pp_cres(9) = '1' then v_code := to_unsigned(3, 4); end if;
                if pp_cres(9) = '0' and pp_arc(9) = "01" then v_code := to_unsigned(4, 4); end if;
                if pp_cres(9) = '0' and pp_arc(9) = "10" then v_code := to_unsigned(5, 4); end if;
            elsif pp_posV(9) = '1' and l_tl = '1' and pp_t2(9) = '1' then
                v_code := to_unsigned(6, 4);
            end if;
            if pp_rim(9) = '1' then
                if pp_cenV(9) = '1' or pp_posV(9) = '0' then v_code := to_unsigned(9, 4); else v_code := to_unsigned(8, 4); end if;
            end if;
            if pp_inner(9) = '1' and pp_inV(9) = '1' and pp_cenV(9) = '1' and pp_posV(9) = '0' then v_code := to_unsigned(10, 4); end if;
            p10_code <= v_code;

            -- 11: palette address; 12: ROM; 13: fabric
            s_pa <= p10_layer & p10_idx;
            pp_code(11) <= p10_code; pp_code(12) <= pp_code(11); pp_code(13) <= pp_code(12);
            p13_y <= unsigned(s_py_q); p13_u <= unsigned(s_pu_q); p13_v <= unsigned(s_pv_q);

            -- 14: blend operands
            case to_integer(pp_code(13)) is
                when 1  => v_ky := resize(unsigned(C_VES_TAN(23 downto 16)), 10); v_ku := resize(unsigned(C_VES_TAN(15 downto 8)), 10);
                           v_kv := resize(unsigned(C_VES_TAN(7 downto 0)), 10); p14_al <= "010";
                           v_ky := shift_left(v_ky, 1); v_ku := shift_left(v_ku, 1); v_kv := shift_left(v_kv, 1);
                when 2  => v_ky := shift_left(resize(unsigned(C_VES_AXIS(23 downto 16)), 10), 2);
                           v_ku := shift_left(resize(unsigned(C_VES_AXIS(15 downto 8)), 10), 2);
                           v_kv := shift_left(resize(unsigned(C_VES_AXIS(7 downto 0)), 10), 2); p14_al <= "100";
                when 3  =>
                    if pp_sh(13)(1) = '1' then
                        v_ky := shift_left(resize(unsigned(C_VES_CDARK(23 downto 16)), 10), 1);
                        v_ku := shift_left(resize(unsigned(C_VES_CDARK(15 downto 8)), 10), 1);
                        v_kv := shift_left(resize(unsigned(C_VES_CDARK(7 downto 0)), 10), 1); p14_al <= "010";
                    else
                        v_ky := (others => '0'); v_ku := (others => '0'); v_kv := (others => '0'); p14_al <= "000";
                    end if;
                when 4  => v_ky := shift_left(resize(unsigned(C_VES_ARC(23 downto 16)), 10), 1);
                           v_ku := shift_left(resize(unsigned(C_VES_ARC(15 downto 8)), 10), 1);
                           v_kv := shift_left(resize(unsigned(C_VES_ARC(7 downto 0)), 10), 1); p14_al <= "010";
                when 5  => v_ky := shift_left(resize(unsigned(C_VES_ARC(23 downto 16)), 10), 2);
                           v_ku := shift_left(resize(unsigned(C_VES_ARC(15 downto 8)), 10), 2);
                           v_kv := shift_left(resize(unsigned(C_VES_ARC(7 downto 0)), 10), 2); p14_al <= "100";
                when 6  => v_ky := shift_left(resize(unsigned(C_VES_TAIL(23 downto 16)), 10), 1);
                           v_ku := shift_left(resize(unsigned(C_VES_TAIL(15 downto 8)), 10), 1);
                           v_kv := shift_left(resize(unsigned(C_VES_TAIL(7 downto 0)), 10), 1); p14_al <= "010";
                when 8  => v_ky := shift_left(resize(unsigned(C_VES_RIMR(23 downto 16)), 10), 2);
                           v_ku := shift_left(resize(unsigned(C_VES_RIMR(15 downto 8)), 10), 2);
                           v_kv := shift_left(resize(unsigned(C_VES_RIMR(7 downto 0)), 10), 2); p14_al <= "100";
                when 9  => v_ky := shift_left(resize(unsigned(C_VES_RIM(23 downto 16)), 10), 2);
                           v_ku := shift_left(resize(unsigned(C_VES_RIM(15 downto 8)), 10), 2);
                           v_kv := shift_left(resize(unsigned(C_VES_RIM(7 downto 0)), 10), 2); p14_al <= "100";
                when 10 => v_ky := shift_left(resize(unsigned(C_VES_INNER(23 downto 16)), 10), 2);
                           v_ku := shift_left(resize(unsigned(C_VES_INNER(15 downto 8)), 10), 2);
                           v_kv := shift_left(resize(unsigned(C_VES_INNER(7 downto 0)), 10), 2); p14_al <= "100";
                when others => v_ky := (others => '0'); v_ku := (others => '0'); v_kv := (others => '0'); p14_al <= "000";
            end case;
            p14_ky <= v_ky; p14_ku <= v_ku; p14_kv <= v_kv;
            if pp_code(13) = 3 then
                v_b8y := unsigned(s_cr_q(23 downto 16)); v_b8u := unsigned(s_cr_q(15 downto 8)); v_b8v := unsigned(s_cr_q(7 downto 0));
            else
                v_b8y := p13_y; v_b8u := p13_u; v_b8v := p13_v;
            end if;
            p14_by <= v_b8y; p14_bu <= v_b8u; p14_bv <= v_b8v;

            -- 15: (base (4 - a) + K a) / 4
            case to_integer(p14_al) is
                when 0 => v_by := shift_left(resize(p14_by, 10), 2); v_bu := shift_left(resize(p14_bu, 10), 2); v_bv := shift_left(resize(p14_bv, 10), 2);
                when 2 => v_by := shift_left(resize(p14_by, 10), 1); v_bu := shift_left(resize(p14_bu, 10), 1); v_bv := shift_left(resize(p14_bv, 10), 1);
                when others => v_by := (others => '0'); v_bu := (others => '0'); v_bv := (others => '0');
            end case;
            v_by := v_by + p14_ky; v_bu := v_bu + p14_ku; v_bv := v_bv + p14_kv;
            p15_y <= v_by(9 downto 2);
            if f_pal = '1' then             -- S10 Azure: chroma mirrored about neutral
                p15_u <= not v_bu(9 downto 2); p15_v <= not v_bv(9 downto 2);
            else
                p15_u <= v_bu(9 downto 2); p15_v <= v_bv(9 downto 2);
            end if;

            -- 16: compose (10 bits)
            s_y12 <= p15_y; s_u12 <= p15_u; s_v12 <= p15_v;

            -- output conditioning (centre of the chroma box at C_LAT - 3)
            -- (8-bit samples; the 10-bit value of a sample s is 4 s + 2)
            s_rc_f <= s_rc_q;                                           -- chroma 8 samples back
            s_boxu <= s_boxu + s_u12 - unsigned(s_rc_f(15 downto 8));
            s_boxv <= s_boxv + s_v12 - unsigned(s_rc_f(7 downto 0));
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 12) + s_boxud; s_u18 <= resize(v_sc(11 downto 2), 10) + 2;
            v_sc := resize(s_boxv, 12) + s_boxvd; s_v18 <= resize(v_sc(11 downto 2), 10) + 2;
            s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14; s_y16 <= s_y15;
            s_y17 <= s_y16; s_y18x <= s_y17;
            s_yf18 <= resize(s_y16, 10) + shift_left(resize(s_y17, 10), 1) + resize(s_y18x, 10) + 2;
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            s_sy_f <= s_sy_q;                                           -- syncs / avid 22 clocks back
            s_sy_1 <= s_sy_f; s_sy_2 <= s_sy_1;
            if s_sy_f(0) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_pix;

    f_stc <= f_phi(12 downto 0) & "000";
    f_ds  <= f_phi(12 downto 0) & "000";
    f_trc <= f_phi(12 downto 0) & "000";
    f_tx  <= f_phi(15 downto 0) & "0000";

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_sy_2(3);
    data_out.vsync_n <= s_sy_2(2);
    data_out.field_n <= s_sy_2(1);
    data_out.avid    <= s_sy_2(0);

end architecture vesica;
