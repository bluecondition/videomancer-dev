-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Chartreuse v0.5.2 -- Heirloom #51, rebuilt from the photo WIN_20231211_00_38_58.
--
-- The photo: a glossy lime surface mirrored about a horizontal axis 460 lines down.  A big soft
-- sphere on the left, then nested ")" crescents (the hourglass, the white C) sweeping right, their
-- arms bending into zig-zag fins at the top and bottom.  Every band is one colour ramp (dark green
-- -> lime -> white, measured channel by channel) folded at the dark grooves, with thin white
-- outline lines (7..24 px, a dark notch before them in scan order, a soft tail after).
--
-- The patch, as rebuilt: no single oscillator field fitted the frame (see SUMMARY.md), so the
-- surface is a knot-grid field (quadratic B-spline, 16 virtual px x 24 lines per knot, folded
-- half plane |y - 460|): F (the shading) through a fold colourizer, S (signed distance to the
-- photo's white lines) drawing the outlines, P gating them.  The live parts sit on top: the
-- colourizer gain (P12) folds the surface into more chrome bands, a dome swells the sphere, the
-- field zooms (crescents) and stretches (arms), a level LFO breathes the bands and lines, a
-- per-line sine sways the surface, and the source (camera luma or the Osc plaid) bends it.
--
--   K1  Crescents  horizontal zoom about the sphere: x0.55 (wide) .. x1.8 (dense, mirrored)
--   K2  Arms       vertical scale of the fold: x0.5 .. x2 (fins and arms spread / squash)
--   K3  Sphere     dome swell, bipolar: shrink .. the photo .. a bright swollen sphere
--   K4  Speed      bipolar, +-24 deadband: breathing rate, sway travel, Osc drift
--   K5  Lines      white outline width 0 .. 19 px (9.5 = the photo)
--   K6  Picture    how far the source (camera luma / Osc) bends the field
--   S7  Twin       second fold of the mirror: two spheres stacked
--   S8  Sway       per-line sine phase on the surface
--   S9  Shade      Gloss (the photo) / Neon (dark surface, glowing lines)
--   S10 Palette    Chartreuse (the photo) / Ember
--   S11 Source     Video / Osc
--   P12 FOLD       colourizer gain x0.25 (soft glow) .. x1 (the photo) .. x4 (dense chrome folds)
--
-- Virtual 1080-line geometry (SD shows the central 1440 virtual px).  Generated avid; output
-- conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the blanking
-- gate.  Latency 24 clocks in every mode.  Colours in the hardware convention (see chartreuse.py).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.chartreuse_pkg.all;
use work.all;

architecture chartreuse of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

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
    -- triangle of a 16-bit phase, -127 .. 127
    function f_tri8(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;
    -- triangle of an 8-bit phase, -64 .. 64
    function f_tri7(p : unsigned(7 downto 0)) return signed is
        variable v : unsigned(7 downto 0);
    begin
        if p(7) = '1' then v := p - 128; else v := 128 - p; end if;
        return to_signed(64, 9) - signed(resize(v, 9));
    end function;
    -- sample-word index with mirror continuation (j < 0: -1 - j, j > 120: 240 - j; lo/hi swapped)
    function f_effw(j : signed(9 downto 0)) return unsigned is
        variable v : signed(9 downto 0);
        variable s : std_logic;
    begin
        if j < 0 then v := -1 - j; s := '1';
        elsif j > 120 then v := 240 - j; s := '1';
        else v := j; s := '0'; end if;
        return s & unsigned(v(6 downto 0));
    end function;
    -- word {P hi5, P lo5, S hi11, S lo11, F hi12, F lo12}; swap lo/hi of each
    function f_sww(w : std_logic_vector(55 downto 0); s : std_logic) return std_logic_vector is
    begin
        if s = '1' then
            return w(50 downto 46) & w(55 downto 51) & w(34 downto 24) & w(45 downto 35) & w(11 downto 0) & w(23 downto 12);
        else return w; end if;
    end function;
    -- smoothstep, 0..31 -> 0..255
    type t_ss is array(0 to 31) of unsigned(7 downto 0);
    constant C_SS : t_ss := (
        to_unsigned(0, 8), to_unsigned(1, 8), to_unsigned(3, 8), to_unsigned(7, 8), to_unsigned(12, 8),
        to_unsigned(19, 8), to_unsigned(27, 8), to_unsigned(36, 8), to_unsigned(46, 8), to_unsigned(57, 8),
        to_unsigned(69, 8), to_unsigned(81, 8), to_unsigned(94, 8), to_unsigned(107, 8), to_unsigned(121, 8),
        to_unsigned(134, 8), to_unsigned(148, 8), to_unsigned(161, 8), to_unsigned(174, 8), to_unsigned(186, 8),
        to_unsigned(198, 8), to_unsigned(209, 8), to_unsigned(219, 8), to_unsigned(228, 8), to_unsigned(236, 8),
        to_unsigned(243, 8), to_unsigned(248, 8), to_unsigned(252, 8), to_unsigned(254, 8), to_unsigned(255, 8),
        to_unsigned(255, 8), to_unsigned(255, 8));
    -- line white, {Y, U, V} per palette (luma pre-boosted, see chartreuse.py)
    constant C_WY0 : unsigned(7 downto 0) := to_unsigned(235, 8);
    constant C_WU0 : unsigned(7 downto 0) := to_unsigned(128, 8);
    constant C_WV0 : unsigned(7 downto 0) := to_unsigned(124, 8);
    constant C_WY1 : unsigned(7 downto 0) := to_unsigned(233, 8);
    constant C_WU1 : unsigned(7 downto 0) := to_unsigned(134, 8);
    constant C_WV1 : unsigned(7 downto 0) := to_unsigned(116, 8);

    -- timing / zone / width
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls (glided, Q.3)
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (others => to_unsigned(512 * 8, 13));   -- K1 K2 K3 K5 P12, power up at the defaults
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal f_twin, f_sway, f_neon, f_pal, f_osc : std_logic := '0';
    signal f_k6 : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal s_sp : signed(10 downto 0) := (others => '0');

    -- per-field values
    signal f_zx   : unsigned(9 downto 0) := to_unsigned(256, 10);   -- Q.8
    signal f_zy   : unsigned(9 downto 0) := to_unsigned(256, 10);   -- Q.8
    signal f_zk   : unsigned(12 downto 0) := to_unsigned(2731, 13); -- zy / 24, Q.16 of knot rows per Q.8 line
    signal f_dx   : unsigned(10 downto 0) := to_unsigned(256, 11);  -- field X per pixel, Q.8
    signal f_x0   : signed(21 downto 0) := to_signed(0, 22);        -- field X of pixel 0, Q.8
    signal f_gain : unsigned(10 downto 0) := to_unsigned(256, 11);  -- Q.8 (fold-line gate)
    signal f_gm   : unsigned(5 downto 0) := to_unsigned(32, 6);     -- gain mantissa 32..63 (Q.5)
    signal f_gn   : unsigned(1 downto 0) := to_unsigned(2, 2);      -- gain octave
    signal f_kg   : unsigned(5 downto 0) := (others => '0');
    signal f_kgl  : unsigned(4 downto 0) := (others => '0');
    signal f_h9   : signed(8 downto 0) := (others => '0');
    signal f_qw   : unsigned(8 downto 0) := to_unsigned(152, 9);    -- Q.4 px
    signal f_kv   : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal f_off  : signed(9 downto 0) := (others => '0');
    signal f_phi  : unsigned(15 downto 0) := (others => '0');
    signal f_phsw : unsigned(7 downto 0) := (others => '0');
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_k1o  : unsigned(9 downto 0) := (others => '0');
    signal f_k1hi : std_logic := '0';

    -- serial multiplier (shared: field slots in the zone, the line engine otherwise)
    signal m_a : unsigned(19 downto 0) := (others => '0');
    signal m_b : unsigned(19 downto 0) := (others => '0');
    signal m_h : unsigned(19 downto 0) := (others => '0');
    signal m_l : unsigned(19 downto 0) := (others => '0');
    signal m_cnt : unsigned(4 downto 0) := (others => '0');
    signal m_busy : std_logic := '0';
    signal m_go_f, m_go_e : std_logic := '0';
    signal m_fa, m_ea : unsigned(19 downto 0) := (others => '0');
    signal m_fb, m_eb : unsigned(12 downto 0) := (others => '0');
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 3) := (others => '0');

    -- per-line engine (computes the next line)
    signal e_st : integer range 0 to 31 := 0;
    signal e_first : std_logic := '0';
    signal e_cnt : unsigned(3 downto 0) := (others => '0');
    signal n_yv  : unsigned(18 downto 0) := (others => '0');
    signal n_bank : std_logic := '0';
    signal e_yv  : unsigned(18 downto 0) := (others => '0');
    signal e_yk  : unsigned(13 downto 0) := (others => '0');
    signal e_v8  : unsigned(7 downto 0) := (others => '0');
    signal e_b0, e_b1, e_b2 : unsigned(9 downto 0) := (others => '0');
    signal e_rb0, e_rb1, e_rb2 : unsigned(11 downto 0) := (others => '0');
    signal e_ph : unsigned(7 downto 0) := (others => '0');
    signal e_sw : signed(14 downto 0) := (others => '0');
    signal n_xst : signed(21 downto 0) := (others => '0');
    signal n_j0  : signed(9 downto 0) := (others => '0');
    signal n_dy  : unsigned(9 downto 0) := (others => '0');
    signal n_tb  : signed(8 downto 0) := (others => '0');
    -- MAC lane
    signal e_mrun : std_logic := '0';
    signal e_mj : unsigned(6 downto 0) := (others => '0');
    signal e_mk : unsigned(1 downto 0) := (others => '0');
    signal e_addr : unsigned(11 downto 0) := (others => '0');
    signal e_v : std_logic_vector(0 to 4) := (others => '0');
    type t_u2a is array(0 to 4) of unsigned(1 downto 0);
    type t_u7a is array(0 to 4) of unsigned(6 downto 0);
    signal e_kd : t_u2a := (others => (others => '0'));
    signal e_jd : t_u7a := (others => (others => '0'));
    signal e_sel1 : unsigned(1 downto 0) := (others => '0');
    signal e_qa, e_qb, e_qc : std_logic_vector(15 downto 0) := (others => '0');
    signal e_w : std_logic_vector(15 downto 0) := (others => '0');
    signal e_pf : unsigned(16 downto 0) := (others => '0');
    signal e_ps : signed(16 downto 0) := (others => '0');
    signal e_pp : unsigned(9 downto 0) := (others => '0');
    signal e_af : unsigned(17 downto 0) := (others => '0');
    signal e_as : signed(17 downto 0) := (others => '0');
    signal e_ap : unsigned(10 downto 0) := (others => '0');
    signal e_c1f, e_c2f : unsigned(11 downto 0) := (others => '0');
    signal e_c1s, e_c2s : signed(10 downto 0) := (others => '0');
    signal e_c1p, e_c2p : unsigned(4 downto 0) := (others => '0');
    signal e_we : std_logic := '0';
    signal e_wa : unsigned(7 downto 0) := (others => '0');
    signal e_wd : std_logic_vector(55 downto 0) := (others => '0');
    signal e_pl : unsigned(1 downto 0) := (others => '0');
    signal e_plv : std_logic_vector(0 to 1) := (others => '0');
    type t_u2b is array(0 to 1) of unsigned(1 downto 0);
    signal e_pli : t_u2b := (others => (others => '0'));
    signal e_pls : std_logic_vector(0 to 1) := (others => '0');
    signal c_pre_ra : unsigned(7 downto 0) := (others => '0');
    signal c_pre_sw : std_logic := '0';
    signal c_pre : std_logic := '0';

    -- committed line values (x start, j0 and the window are read straight from the engine at
    -- g_start / preloaded in the blanking, before the engine touches them again)
    signal l_bank : std_logic := '0';
    signal l_dy  : unsigned(9 downto 0) := (others => '0');
    signal l_tb  : signed(8 downto 0) := (others => '0');

    -- sample-word RAM (2 banks x 128 words); word j = the field at X = 16 j (lo) and 16 j + 8 (hi)
    type t_cram is array(0 to 255) of std_logic_vector(55 downto 0);
    signal ram_w : t_cram := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_w : signal is true;
    signal c_ra : unsigned(7 downto 0) := (others => '0');
    signal c_q : std_logic_vector(55 downto 0) := (others => '0');
    signal c_sw : std_logic_vector(0 to 1) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- beam (stage 0)
    signal b_x  : signed(21 downto 0) := (others => '0');
    signal b_sh : std_logic := '0';   -- this pixel entered a new 16-px segment
    signal w_nx : unsigned(7 downto 0) := (others => '0');   -- swap & word index of w_j + 3

    -- stage 1: window
    type t_w56 is array(0 to 2) of std_logic_vector(55 downto 0);
    signal w_w : t_w56 := (others => (others => '0'));
    signal w_j : signed(9 downto 0) := (others => '0');
    signal w_rf : std_logic_vector(0 to 1) := (others => '0');
    signal p1_u : unsigned(3 downto 0) := (others => '0');
    signal p1_h : std_logic := '0';
    signal p1_dx : unsigned(11 downto 0) := (others => '0');
    signal p1_ta : signed(8 downto 0) := (others => '0');
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- stage 2
    signal p2_af, p2_df : signed(13 downto 0) := (others => '0');
    signal p2_as, p2_ds : signed(11 downto 0) := (others => '0');
    signal p2_ap, p2_dp : signed(6 downto 0) := (others => '0');
    signal p2_u : unsigned(3 downto 0) := (others => '0');
    signal p2_r : unsigned(12 downto 0) := (others => '0');
    signal p2_vs : signed(10 downto 0) := (others => '0');
    -- stage 3
    signal p3_af : signed(13 downto 0) := (others => '0');
    signal p3_mf : signed(18 downto 0) := (others => '0');
    signal p3_as : signed(11 downto 0) := (others => '0');
    signal p3_ms : signed(16 downto 0) := (others => '0');
    signal p3_ap : signed(6 downto 0) := (others => '0');
    signal p3_mp : signed(11 downto 0) := (others => '0');
    signal p3_dome : unsigned(6 downto 0) := (others => '0');
    signal p3_dfv : signed(14 downto 0) := (others => '0');
    -- stage 4
    signal p4_fq : unsigned(12 downto 0) := (others => '0');
    signal p4_sq : signed(12 downto 0) := (others => '0');
    signal p4_p2 : unsigned(4 downto 0) := (others => '0');
    signal p4_dfd : signed(16 downto 0) := (others => '0');
    signal p4_dfv : signed(14 downto 0) := (others => '0');
    -- stage 5
    signal p5_f : signed(14 downto 0) := (others => '0');
    signal p5_df : signed(13 downto 0) := (others => '0');
    signal p5_sq : signed(12 downto 0) := (others => '0');
    signal p5_p2 : unsigned(4 downto 0) := (others => '0');
    -- stage 6
    signal p6_f : signed(14 downto 0) := (others => '0');
    signal p6_ds : signed(12 downto 0) := (others => '0');
    signal p6_sq : signed(12 downto 0) := (others => '0');
    signal p6_p2 : unsigned(4 downto 0) := (others => '0');
    -- stage 7
    signal p7_s : signed(13 downto 0) := (others => '0');
    signal p7_gp : signed(22 downto 0) := (others => '0');
    signal p7_p2 : unsigned(4 downto 0) := (others => '0');
    -- stage 8
    signal p8_g : signed(18 downto 0) := (others => '0');
    signal p8_a : unsigned(12 downto 0) := (others => '0');
    signal p8_ap : unsigned(12 downto 0) := (others => '0');
    signal p8_p2 : unsigned(4 downto 0) := (others => '0');
    -- stage 9
    signal p9_t : unsigned(12 downto 0) := (others => '0');
    signal p9_nb, p9_fl : std_logic := '0';
    signal p9_ss : unsigned(4 downto 0) := (others => '0');
    signal p9_nd : unsigned(1 downto 0) := (others => '0');
    signal p9_p2 : unsigned(4 downto 0) := (others => '0');
    -- stage 10..16
    signal r_pa : unsigned(8 downto 0) := (others => '0');
    signal p10_ss : unsigned(4 downto 0) := (others => '0');
    signal p10_sf : unsigned(4 downto 0) := (others => '0');
    signal p10_nd : unsigned(1 downto 0) := (others => '0');
    signal r_py, r_pu, r_pv : std_logic_vector(7 downto 0) := (others => '0');
    signal p11_s5 : unsigned(4 downto 0) := (others => '0');
    signal p11_nd, p12_nd, p13_nd, p14_nd, p15_nd : unsigned(1 downto 0) := (others => '0');
    signal p12_s8 : unsigned(7 downto 0) := (others => '0');
    type t_c3 is array(0 to 2) of unsigned(7 downto 0);
    signal p12_c, p13_c, p14_c, p15_c : t_c3 := (others => (others => '0'));
    signal p13_so : unsigned(7 downto 0) := (others => '0');
    signal p13_w : t_c3 := (others => (others => '0'));
    type t_s15a is array(0 to 2) of signed(15 downto 0);
    signal p14_d : t_s15a := (others => (others => '0'));
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    type t_ring is array(0 to 255) of std_logic_vector(15 downto 0);
    signal ring_c : t_ring := (others => x"8080");
    signal ring_s : t_ring := (others => x"FFFF");
    attribute no_rw_check of ring_c : signal is true;
    attribute no_rw_check of ring_s : signal is true;
    signal r_wp : unsigned(7 downto 0) := (others => '0');
    signal r_cur : std_logic_vector(15 downto 0) := x"8080";
    signal r_old : std_logic_vector(15 downto 0) := x"8080";
    signal r_sy  : std_logic_vector(15 downto 0) := x"FFFF";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_y17, s_y18, s_y19, s_y20, s_y21, s_y22x, s_yf22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs: knot words (three EBR banks), palette; sample-word RAM
    ---------------------------------------------------------------------------
    p_kn : process(clk)
    begin
        if rising_edge(clk) then
            e_qa <= C_FSA(to_integer(e_addr(10 downto 0)));
            e_qb <= C_FSB(to_integer(e_addr(9 downto 0)));
            e_qc <= C_FSC(to_integer(e_addr(8 downto 0)));
        end if;
    end process p_kn;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            r_py <= C_PY(to_integer(r_pa));
            r_pu <= C_PU(to_integer(r_pa));
            r_pv <= C_PV(to_integer(r_pa));
        end if;
    end process p_pal;

    p_cram_w : process(clk)
    begin
        if rising_edge(clk) then
            if e_we = '1' then
                ram_w(to_integer(e_wa)) <= e_wd;
            end if;
        end if;
    end process p_cram_w;

    p_cram_r : process(clk)
    begin
        if rising_edge(clk) then
            c_q <= ram_w(to_integer(c_ra));
        end if;
    end process p_cram_r;

    -- delay rings: chroma for the 8-box (read 8 back), syncs (read C_LAT - 1 back)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            ring_c(to_integer(r_wp)) <= std_logic_vector(p15_c(1)) & std_logic_vector(p15_c(2));
            r_old <= ring_c(to_integer(r_wp - 8));
            ring_s(to_integer(r_wp)) <= "1111111111111" & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            r_sy <= ring_s(to_integer(r_wp - (C_LAT - 1)));
            r_wp <= r_wp + 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field values, field slots on the multiplier
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_vw : unsigned(13 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_x  : signed(10 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_pr : unsigned(39 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_gn : unsigned(12 downto 0);
        variable v_t  : signed(16 downto 0);
        variable v_pq : unsigned(15 downto 0);
        variable v_kg : signed(13 downto 0);
        variable v_hh : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

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

            -- width: source avid pixels per line, max over the field; virtual width
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            -- speed: bipolar, +-24 deadband
            v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
            if v_x > -24 and v_x < 24 then s_sp <= (others => '0'); else s_sp <= v_x; end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                f_twin <= registers_in(6)(0);
                f_sway <= registers_in(6)(1);
                f_neon <= registers_in(6)(2);
                f_pal  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                f_k6   <= unsigned(registers_in(5));
                f_phi  <= f_phi + unsigned(resize(s_sp, 16)) + unsigned(resize(shift_right(s_sp, 4), 16));
                f_phsw <= f_phsw + unsigned(resize(shift_right(s_sp, 2), 8));
                f_ta <= f_ta + 262; f_tb <= f_tb - 197;
            end if;

            -- glides: K1, K2, K3, K5, P12 ease 1/8 of the gap per field (deadband 4)
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when others => s_g(4) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- knob curves (no multiply; settle long before the slots read them)
            v_k := s_g(0)(12 downto 3);
            if v_k(9) = '0' then f_k1o <= v_k; f_k1hi <= '0'; else f_k1o <= v_k - 512; f_k1hi <= '1'; end if;
            v_k := s_g(1)(12 downto 3);
            if v_k(9) = '0' then f_zy <= to_unsigned(128, 10) + resize(v_k(8 downto 2), 10);
            else f_zy <= to_unsigned(256, 10) + resize(v_k(8 downto 1), 10); end if;
            v_k := s_g(2)(12 downto 3);
            v_hh := shift_left(signed(resize(v_k, 13)) - 512, 2);
            f_h9 <= resize(shift_right(v_hh, 4), 9);
            v_k := s_g(3)(12 downto 3);
            v_s := resize(v_k & "0000", 21) + resize(v_k & '0', 21) + resize(v_k, 21);   -- x19
            f_qw <= v_s(14 downto 6);
            v_k := s_g(4)(12 downto 3);
            v_gn := resize(to_unsigned(256, 10) + resize(v_k(7 downto 3) & "000", 10), 13);
            f_gm <= to_unsigned(32, 6) + resize(v_k(7 downto 3), 6);
            f_gn <= v_k(9 downto 8);
            case to_integer(v_k(9 downto 8)) is
                when 0 => f_gain <= resize(v_gn(12 downto 2), 11);
                when 1 => f_gain <= resize(v_gn(12 downto 1), 11);
                when 2 => f_gain <= resize(v_gn, 11);
                when others => f_gain <= resize(v_gn & '0', 11);
            end case;
            v_kg := shift_right(signed(resize(f_gain, 14)) - 256 + shift_left(signed(resize(f_gain, 14)) - 256, 1), 4);
            if v_kg < 0 then f_kg <= (others => '0');
            elsif v_kg > 32 then f_kg <= to_unsigned(32, 6);
            else f_kg <= unsigned(v_kg(5 downto 0)); end if;
            v_s := resize(f_kg & "00000", 21) - resize(f_kg & '0', 21);   -- x30
            f_kgl <= v_s(9 downto 5);
            v_s := resize(f_k6, 21) + resize(f_k6(9 downto 1), 21);
            f_kv <= v_s(10 downto 6);
            case to_integer(s_xs) is
                when 4      => f_dx <= resize(f_zx & '0', 11);
                when 3      => f_dx <= resize(f_zx, 11) + resize(f_zx(9 downto 1), 11);
                when others => f_dx <= resize(f_zx, 11);
            end case;
            -- breathing: +-256 (Q.12) triangle of the speed phase
            -- (phase + 1/4 so the offset is 0 at phase 0: still at the K4 centre = the photo)
            v_pq := f_phi + 16384;
            if v_pq(15) = '1' then v_t := signed(resize(v_pq - 32768, 17)) - 16384;
            else v_t := signed(resize(32768 - v_pq, 17)) - 16384; end if;
            f_off <= v_t(15 downto 6);

            -- field slots on the serial multiplier: slot p = zc/32 (1..3), 0 load, 24 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 3 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            for k in 1 to 3 loop
                if s_zone = '1' and v_p = k and v_ph = 24 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            m_go_f <= q_ld;
            if q_ld = '1' then
                case to_integer(q_p) is
                    when 1 =>
                        m_fa <= resize(f_k1o, 20);
                        if f_k1hi = '1' then m_fb <= to_unsigned(205, 13); else m_fb <= to_unsigned(115, 13); end if;
                    when 2 => m_fa <= resize(s_vw(11 downto 1) - 480, 20); m_fb <= resize(f_zx, 13);
                    when others => m_fa <= resize(f_zy, 20); m_fb <= to_unsigned(683, 13);
                end case;
            end if;
            v_pr := m_h & m_l;
            if q_wr(1) = '1' then
                if f_k1hi = '1' then f_zx <= to_unsigned(256, 10) + resize(v_pr(17 downto 9), 10);
                else f_zx <= to_unsigned(141, 10) + resize(v_pr(17 downto 9), 10); end if;
            end if;
            if q_wr(2) = '1' then f_x0 <= to_signed(122880, 22) - signed(resize(v_pr(19 downto 0), 22)); end if;
            if q_wr(3) = '1' then f_zk <= v_pr(18 downto 6); end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    -- serial multiplier: 20 x 12, 12 steps; started by the field slots or the line engine
    p_mul : process(clk)
        variable v_s : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            if m_go_f = '1' or m_go_e = '1' then
                if m_go_f = '1' then m_a <= m_fa; m_b <= resize(m_fb, 20); else m_a <= m_ea; m_b <= resize(m_eb, 20); end if;
                m_h <= (others => '0'); m_l <= (others => '0');
                m_cnt <= (others => '0'); m_busy <= '1';
            elsif m_busy = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(19 downto 1);
                m_b <= '0' & m_b(19 downto 1);
                if m_cnt = 19 then m_busy <= '0'; end if;
                m_cnt <= m_cnt + 1;
            end if;
        end if;
    end process p_mul;

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
    -- Per-line engine: kicked at each source avid rise (and for line 0 late in
    -- the vblank zone); computes the next line's sample words into the other
    -- RAM bank; the window preload follows in the blanking (p_pix).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(19 downto 0);
        variable v_s  : signed(21 downto 0);
        variable v_b  : unsigned(9 downto 0);
        variable v_pr : unsigned(39 downto 0);
        variable v_q  : unsigned(6 downto 0);
        variable v_j  : signed(9 downto 0);
        variable v_cf : unsigned(11 downto 0);
        variable v_cs : signed(10 downto 0);
        variable v_cp : unsigned(4 downto 0);
        variable v_lo, v_hi : unsigned(14 downto 0);
        variable v_los, v_his : signed(13 downto 0);
        variable v_lop, v_hip : unsigned(7 downto 0);
        variable v_ew : unsigned(7 downto 0);
        variable v_kc : unsigned(6 downto 0);
        variable v_yk : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            v_pr := m_h & m_l;
            m_go_e <= '0';

            -- commit at the source avid rise (before the beam starts), then kick the next line
            if data_in.avid = '1' and s_avq = '0' then
                l_bank <= n_bank; l_dy <= n_dy;
                l_tb <= n_tb;
                e_st <= 1; e_first <= '0';
            elsif s_zone = '1' and s_zc = 700 then
                e_st <= 1; e_first <= '1';
            else
                case e_st is
                    when 1 =>
                        if e_first = '1' then n_yv <= (others => '0'); n_bank <= '0';
                        else n_yv <= n_yv + resize(s_ys, 19); n_bank <= not n_bank; end if;
                        e_st <= 2;
                    when 2 =>
                        v_d := signed(resize(n_yv, 20)) - 117760;
                        if v_d < 0 then v_d := -v_d; end if;
                        e_yv <= unsigned(v_d(18 downto 0));
                        e_st <= 3;
                    when 3 =>
                        if f_twin = '1' then
                            v_d := signed(resize(e_yv, 20)) - 69120;
                            if v_d < 0 then v_d := -v_d; end if;
                            e_yv <= unsigned(v_d(18 downto 0));
                        end if;
                        e_st <= 4;
                    when 4 =>
                        m_ea <= resize(e_yv, 20); m_eb <= f_zk; m_go_e <= '1'; e_st <= 5; e_cnt <= (others => '0');
                    when 5 =>   -- wait for the product (go + 20 steps)
                        if e_cnt = 13 and m_busy = '0' then
                            if v_pr(31 downto 30) /= "00" then v_yk := (others => '1'); else v_yk := v_pr(29 downto 16); end if;
                            if v_yk >= 6656 then v_yk := 13312 - v_yk; end if;
                            e_yk <= v_yk;
                            e_st <= 6;
                        elsif e_cnt /= 13 then e_cnt <= e_cnt + 1; end if;
                    when 6 =>
                        e_v8 <= e_yk(7 downto 0);
                        e_rb0 <= resize(e_yk(12 downto 8) & "0000000", 12) - resize(e_yk(12 downto 8) & "00", 12) - resize(e_yk(12 downto 8) & '0', 12);
                        e_ph <= e_yk(11 downto 4) + f_phsw;
                        n_dy <= resize(shift_right(resize(e_yk, 16) + shift_left(resize(e_yk, 16), 1), 5), 10);
                        n_tb <= f_tri7(e_yk(11 downto 4) + f_tb(15 downto 8));
                        e_st <= 7;
                    when 7 =>
                        m_ea <= resize(to_unsigned(256, 9) - e_v8, 20); m_eb <= resize(to_unsigned(256, 9) - e_v8, 13); m_go_e <= '1'; e_st <= 8; e_cnt <= (others => '0');
                        e_rb1 <= e_rb0 + 122;
                    when 8 =>
                        if e_cnt = 13 and m_busy = '0' then
                            e_b0 <= v_pr(17 downto 8); e_st <= 9;
                        elsif e_cnt /= 13 then e_cnt <= e_cnt + 1; end if;
                        e_rb2 <= e_rb1 + 122;
                    when 9 =>
                        m_ea <= resize(e_v8, 20); m_eb <= resize(e_v8, 13); m_go_e <= '1'; e_st <= 10; e_cnt <= (others => '0');
                    when 10 =>
                        if e_cnt = 13 and m_busy = '0' then
                            e_b2 <= v_pr(17 downto 8); e_st <= 11;
                        elsif e_cnt /= 13 then e_cnt <= e_cnt + 1; end if;
                    when 11 =>
                        e_b1 <= to_unsigned(512, 10) - e_b0 - e_b2;
                        v_q := e_ph(6 downto 0);
                        m_ea <= resize(v_q, 20); m_eb <= resize(to_unsigned(128, 8) - v_q, 13); m_go_e <= '1'; e_st <= 12; e_cnt <= (others => '0');
                    when 12 =>
                        if e_cnt = 13 and m_busy = '0' then
                            -- s = q (128 - q) / 32, 0..128; sway = s * 80 (Q.8 px)
                            v_s := signed(resize(v_pr(12 downto 5) & "000000", 22)) + signed(resize(v_pr(12 downto 5) & "0000", 22));
                            if e_ph(7) = '1' then v_s := -v_s; end if;
                            e_sw <= v_s(14 downto 0);
                            e_st <= 13;
                        elsif e_cnt /= 13 then e_cnt <= e_cnt + 1; end if;
                    when 13 =>
                        if f_sway = '1' then v_s := f_x0 + resize(e_sw, 22); else v_s := f_x0; end if;
                        n_xst <= v_s;
                        n_j0 <= v_s(21 downto 12);
                        e_mj <= (others => '0'); e_mk <= (others => '0'); e_mrun <= '1';
                        e_st <= 14;
                    when 14 =>
                        if e_mrun = '0' then e_cnt <= (others => '0'); e_st <= 15; end if;
                    when 15 =>   -- MAC flush
                        if e_cnt = 8 then e_st <= 16; else e_cnt <= e_cnt + 1; end if;
                    when 16 =>   -- preload in the blanking
                        if data_in.avid = '0' and g_avid = '0' and s_gav_sr(3) = '0' then
                            e_pl <= (others => '0'); e_st <= 17;
                        end if;
                    when 17 =>
                        if e_pl = 3 then e_cnt <= (others => '0'); e_st <= 18; else e_pl <= e_pl + 1; end if;
                    when 18 =>
                        if e_cnt = 4 then e_st <= 0; else e_cnt <= e_cnt + 1; end if;
                    when others =>
                        null;
                end case;
            end if;

            -- MAC lane: 123 columns x 3 taps (column 122 mirrors 119)
            if e_mrun = '1' then
                if e_mk = 2 then
                    e_mk <= (others => '0');
                    if e_mj = 122 then e_mrun <= '0'; else e_mj <= e_mj + 1; end if;
                else
                    e_mk <= e_mk + 1;
                end if;
            end if;
            if e_mj = 122 then v_kc := to_unsigned(119, 7); else v_kc := e_mj; end if;
            case to_integer(e_mk) is
                when 0 => e_addr <= e_rb0 + resize(v_kc, 12);
                when 1 => e_addr <= e_rb1 + resize(v_kc, 12);
                when others => e_addr <= e_rb2 + resize(v_kc, 12);
            end case;
            e_v(0) <= e_mrun; e_kd(0) <= e_mk; e_jd(0) <= e_mj;
            -- t1: ROM read (p_kn); t2: bank select
            e_v(1) <= e_v(0); e_kd(1) <= e_kd(0); e_jd(1) <= e_jd(0); e_sel1 <= e_addr(11 downto 10);
            e_v(2) <= e_v(1); e_kd(2) <= e_kd(1); e_jd(2) <= e_jd(1);
            case e_sel1 is
                when "10" => e_w <= e_qb;
                when "11" => e_w <= e_qc;
                when others => e_w <= e_qa;
            end case;
            -- t3: products
            e_v(3) <= e_v(2); e_kd(3) <= e_kd(2); e_jd(3) <= e_jd(2);
            case to_integer(e_kd(2)) is
                when 0 => v_b := e_b0;
                when 1 => v_b := e_b1;
                when others => v_b := e_b2;
            end case;
            e_pf <= unsigned(e_w(15 downto 8)) * v_b(8 downto 0);
            e_ps <= resize(signed(e_w(7 downto 1)) * signed('0' & v_b(8 downto 0)), 17);
            if e_w(0) = '1' then e_pp <= v_b; else e_pp <= (others => '0'); end if;
            -- t4: accumulate
            e_v(4) <= e_v(3); e_kd(4) <= e_kd(3); e_jd(4) <= e_jd(3);
            if e_kd(3) = 0 then
                e_af <= resize(e_pf, 18); e_as <= resize(e_ps, 18); e_ap <= resize(e_pp, 11);
            else
                e_af <= e_af + resize(e_pf, 18); e_as <= e_as + resize(e_ps, 18); e_ap <= e_ap + resize(e_pp, 11);
            end if;
            -- t5: column complete -> history; word j-2 = the field at 16(j-2) and 16(j-2) + 8
            e_we <= '0';
            if e_v(4) = '1' and e_kd(4) = 2 then
                v_cf := e_af(16 downto 5); v_cs := e_as(16 downto 6); v_cp := e_ap(9 downto 5);
                e_c2f <= e_c1f; e_c1f <= v_cf; e_c2s <= e_c1s; e_c1s <= v_cs; e_c2p <= e_c1p; e_c1p <= v_cp;
                if e_jd(4) >= 2 then e_we <= '1'; end if;
                e_wa <= n_bank & (e_jd(4) - 2);
                v_lo := resize(e_c2f, 15) + resize(e_c1f, 15);
                v_hi := resize(e_c2f, 15) + resize(e_c1f & "00", 15) + resize(e_c1f & '0', 15) + resize(v_cf, 15);
                v_los := resize(e_c2s, 14) + resize(e_c1s, 14);
                v_his := resize(e_c2s, 14) + shift_left(resize(e_c1s, 14), 2) + shift_left(resize(e_c1s, 14), 1) + resize(v_cs, 14);
                v_lop := resize(e_c2p, 8) + resize(e_c1p, 8);
                v_hip := resize(e_c2p, 8) + resize(e_c1p & "00", 8) + resize(e_c1p & '0', 8) + resize(v_cp, 8);
                e_wd <= std_logic_vector(v_hip(7 downto 3)) & std_logic_vector(v_lop(5 downto 1))
                        & std_logic_vector(v_his(13 downto 3)) & std_logic_vector(v_los(11 downto 1))
                        & std_logic_vector(v_hi(14 downto 3)) & std_logic_vector(v_lo(12 downto 1));
            end if;

            -- preload: three reads eff(j0 .. j0+2) of the new bank (written into the window by p_pix)
            v_j := n_j0 + signed(resize(e_pl, 10));
            v_ew := f_effw(v_j);
            c_pre_ra <= n_bank & v_ew(6 downto 0); c_pre_sw <= v_ew(7);
            if e_st = 17 and e_pl /= 3 then c_pre <= '1'; else c_pre <= '0'; end if;
            e_plv(0) <= c_pre; e_pli(0) <= e_pl - 1; e_pls(0) <= c_pre_sw;
            e_plv(1) <= e_plv(0); e_pli(1) <= e_pli(0); e_pls(1) <= e_pls(0);
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: field X (Q.8), Osc phase
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_nx : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x <= n_xst; b_sh <= '0';
            elsif g_avid = '1' then
                v_nx := b_x + signed(resize(f_dx, 22));
                b_x <= v_nx;
                if v_nx(21 downto 12) /= b_x(21 downto 12) then b_sh <= '1'; else b_sh <= '0'; end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_j  : signed(9 downto 0);
        variable v_ew : unsigned(7 downto 0);
        variable v_a0, v_a1 : unsigned(11 downto 0);
        variable v_s0, v_s1 : signed(11 downto 0);
        variable v_p0, v_p1 : signed(6 downto 0);
        variable v_dx : signed(21 downto 0);
        variable v_ax : unsigned(11 downto 0);
        variable v_mx, v_mn : unsigned(11 downto 0);
        variable v_x  : signed(11 downto 0);
        variable v_sum : signed(14 downto 0);
        variable v_d13 : signed(12 downto 0);
        variable v_pq : signed(8 downto 0);
        variable v_df : signed(15 downto 0);
        variable v_ds : signed(19 downto 0);
        variable v_f5 : signed(15 downto 0);
        variable v_g  : signed(18 downto 0);
        variable v_gm, v_tt : unsigned(12 downto 0);
        variable v_t  : signed(13 downto 0);
        variable v_dd : signed(13 downto 0);
        variable v_a  : unsigned(12 downto 0);
        variable v_dl : signed(13 downto 0);
        variable v_ss : unsigned(9 downto 0);
        variable v_m  : unsigned(7 downto 0);
        variable v_so : unsigned(7 downto 0);
        variable v_c  : t_c3;
        variable v_yy : signed(17 downto 0);
        variable v_cd : signed(9 downto 0);
        variable v_dly : unsigned(9 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            ---------------- stage 1: sample-word window (preload, shift, refill), u, |dX|, source tap
            c_sw(1) <= c_sw(0);
            w_rf(1) <= w_rf(0); w_rf(0) <= '0';
            w_nx <= f_effw(w_j + 3);
            if g_start = '1' then
                w_j <= n_j0;
            elsif g_avid = '1' and b_sh = '1' then
                w_w(0) <= w_w(1); w_w(1) <= w_w(2);
                w_j <= w_j + 1;
                w_rf(0) <= '1';
            end if;
            if b_sh = '1' and g_avid = '1' then
                c_ra <= l_bank & w_nx(6 downto 0); c_sw(0) <= w_nx(7);
            elsif c_pre = '1' then
                c_ra <= c_pre_ra;
            end if;
            if w_rf(1) = '1' then
                w_w(2) <= f_sww(c_q, c_sw(1));
            elsif e_plv(1) = '1' and e_pli(1) /= 3 then
                w_w(to_integer(e_pli(1))) <= f_sww(c_q, e_pls(1));
            end if;
            p1_u <= unsigned(b_x(10 downto 7));
            p1_h <= b_x(11);
            v_dx := b_x - 122880;
            if v_dx < 0 then v_dx := -v_dx; end if;
            if v_dx(21 downto 20) /= "00" then p1_dx <= (others => '1'); else p1_dx <= unsigned(v_dx(19 downto 8)); end if;
            p1_ta <= f_tri7(unsigned(b_x(16 downto 9)) + f_ta(15 downto 8));
            v_y1 <= unsigned(data_in.y);

            ---------------- stage 2: the two samples around the pixel, octagonal r, source value
            if p1_h = '0' then
                v_a0 := unsigned(w_w(0)(11 downto 0)); v_a1 := unsigned(w_w(0)(23 downto 12));
                v_s0 := resize(signed(w_w(0)(34 downto 24)), 12); v_s1 := resize(signed(w_w(0)(45 downto 35)), 12);
                v_p0 := signed(resize(unsigned(w_w(0)(50 downto 46)), 7)); v_p1 := signed(resize(unsigned(w_w(0)(55 downto 51)), 7));
            else
                v_a0 := unsigned(w_w(0)(23 downto 12)); v_a1 := unsigned(w_w(1)(11 downto 0));
                v_s0 := resize(signed(w_w(0)(45 downto 35)), 12); v_s1 := resize(signed(w_w(1)(34 downto 24)), 12);
                v_p0 := signed(resize(unsigned(w_w(0)(55 downto 51)), 7)); v_p1 := signed(resize(unsigned(w_w(1)(50 downto 46)), 7));
            end if;
            p2_af <= signed(resize(v_a0, 14));
            p2_df <= signed(resize(v_a1, 14)) - signed(resize(v_a0, 14));
            p2_as <= v_s0; p2_ds <= v_s1 - v_s0;
            p2_ap <= v_p0; p2_dp <= v_p1 - v_p0;
            p2_u <= p1_u;
            if p1_dx > resize(l_dy, 12) then v_mx := p1_dx; v_mn := resize(l_dy, 12);
            else v_mx := resize(l_dy, 12); v_mn := p1_dx; end if;
            p2_r <= resize(v_mx, 13) + resize(v_mn(11 downto 1), 13);
            if f_osc = '1' then
                v_x := resize(p1_ta, 12) + resize(l_tb, 12);
                p2_vs <= resize(shift_right(shift_left(v_x, 3) - v_x, 1), 11);
            else
                p2_vs <= signed(resize(v_y1, 11)) - 512;
            end if;

            ---------------- stage 3: interpolation products, dome, source product
            p3_af <= p2_af; p3_mf <= p2_df * signed('0' & p2_u);
            p3_as <= p2_as; p3_ms <= p2_ds * signed('0' & p2_u);
            p3_ap <= p2_ap; p3_mp <= p2_dp * signed('0' & p2_u);
            if p2_r >= 511 then p3_dome <= (others => '0');
            else p3_dome <= resize(shift_right(to_unsigned(511, 9) - p2_r(8 downto 0), 2), 7); end if;
            p3_dfv <= resize(shift_right(p2_vs, 2), 9) * signed('0' & f_kv);

            ---------------- stage 4: field values, dome product
            v_sum := resize(p3_af, 15) + resize(shift_right(p3_mf, 4), 15);
            if v_sum < 0 then p4_fq <= (others => '0'); else p4_fq <= unsigned(v_sum(12 downto 0)); end if;
            p4_sq <= resize(p3_as, 13) + resize(shift_right(p3_ms, 4), 13);
            v_pq := resize(p3_ap, 9) + resize(shift_right(p3_mp, 4), 9);
            if v_pq <= 0 then p4_p2 <= (others => '0');
            elsif v_pq >= 8 then p4_p2 <= to_unsigned(16, 5);
            else p4_p2 <= unsigned(v_pq(3 downto 0)) & '0'; end if;
            p4_dfd <= f_h9 * signed('0' & p3_dome);
            p4_dfv <= p3_dfv;

            ---------------- stage 5: F (Q.12) without offset, field offset
            v_f5 := signed(resize(shift_right(resize(p4_fq, 16) + shift_left(resize(p4_fq, 16), 2), 2), 16)) - 1024;
            p5_f <= v_f5(14 downto 0);
            p5_df <= resize(shift_right(p4_dfd, 3), 14) + resize(shift_right(p4_dfv, 1), 14) + resize(f_off, 14);
            p5_sq <= p4_sq; p5_p2 <= p4_p2;

            ---------------- stage 6: F + offset, S offset
            p6_f <= p5_f + resize(p5_df, 15);
            v_ds := resize(p5_df, 20);
            p6_ds <= resize(shift_right(shift_left(v_ds, 4) + shift_left(v_ds, 3) + v_ds, 6), 13);
            p6_sq <= p5_sq; p6_p2 <= p5_p2;

            ---------------- stage 7: S, gain product
            p7_s <= resize(p6_sq, 14) + resize(p6_ds, 14);
            p7_gp <= (resize(p6_f, 16) - 1229) * signed('0' & f_gm);
            p7_p2 <= p6_p2;

            ---------------- stage 8: G, |S|
            case to_integer(f_gn) is
                when 0 => p8_g <= resize(shift_right(p7_gp, 7), 19) + 1229;
                when 1 => p8_g <= resize(shift_right(p7_gp, 6), 19) + 1229;
                when 2 => p8_g <= resize(shift_right(p7_gp, 5), 19) + 1229;
                when others => p8_g <= resize(shift_right(p7_gp, 4), 19) + 1229;
            end case;
            if p7_s < 0 then v_a := unsigned(resize(-p7_s, 13)); else v_a := unsigned(resize(p7_s, 13)); end if;
            p8_a <= v_a; p8_ap <= p8_a;
            p8_p2 <= p7_p2;

            ---------------- stage 9: fold, line level, notch (|S| shrinking: the line is ahead)
            v_gm := unsigned(p8_g(12 downto 0)) + 4096;
            if v_gm >= 4096 then v_tt := v_gm - 4096; else v_tt := 4096 - v_gm; end if;
            p9_t <= v_tt;
            if v_tt >= 1133 and v_tt < 1325 then p9_fl <= '1'; else p9_fl <= '0'; end if;
            v_g := p8_g - 1229;
            if v_g > -123 and v_g < 123 then p9_nb <= '1'; else p9_nb <= '0'; end if;
            v_dl := signed(resize(f_qw, 14)) - signed(resize(p8_a, 14));
            if v_dl <= 0 then p9_ss <= (others => '0');
            elsif v_dl >= 124 then p9_ss <= to_unsigned(31, 5);
            else p9_ss <= unsigned(v_dl(6 downto 2)); end if;
            p9_nd <= "00";
            if p8_a < p8_ap and v_dl <= 0 then
                if v_dl > -48 then p9_nd <= "10"; elsif v_dl > -96 then p9_nd <= "01"; end if;
            end if;
            p9_p2 <= p8_p2;

            ---------------- stage 10: palette address, presence, fold line
            if p9_t(12) = '1' then r_pa <= f_pal & to_unsigned(255, 8);
            else r_pa <= f_pal & p9_t(11 downto 4); end if;
            v_ss := p9_ss * p9_p2;
            p10_ss <= v_ss(8 downto 4);
            if p9_nb = '0' and p9_fl = '1' then p10_sf <= f_kgl; else p10_sf <= (others => '0'); end if;
            p10_nd <= p9_nd;

            ---------------- stage 11: line level (palette ROM read in p_pal)
            if p10_sf > p10_ss then p11_s5 <= p10_sf; else p11_s5 <= p10_ss; end if;
            p11_nd <= p10_nd;

            ---------------- stage 12: smoothstep; palette fabric copy
            p12_s8 <= C_SS(to_integer(p11_s5));
            p12_c <= (unsigned(r_py), unsigned(r_pu), unsigned(r_pv));
            p12_nd <= p11_nd;

            ---------------- stage 13: tail (fast attack, slow decay), Neon, white
            v_so := p13_so - shift_right(p13_so, 2) + shift_right(p13_so, 5);
            if s_gav_sr(11) = '0' then v_so := (others => '0'); end if;
            if p12_s8 > v_so then p13_so <= p12_s8; else p13_so <= v_so; end if;
            v_c := p12_c;
            if f_neon = '1' then
                v_c(0) := to_unsigned(16, 8) + shift_right(v_c(0), 2);
                v_c(1) := to_unsigned(96, 8) + shift_right(v_c(1), 2);
                v_c(2) := to_unsigned(96, 8) + shift_right(v_c(2), 2);
            end if;
            p13_c <= v_c;
            if f_pal = '1' then p13_w <= (C_WY1, C_WU1, C_WV1); else p13_w <= (C_WY0, C_WU0, C_WV0); end if;
            p13_nd <= p12_nd;

            ---------------- stage 14: blend products
            p14_d(0) <= (signed(resize(p13_w(0), 10)) - signed(resize(p13_c(0), 10))) * signed('0' & p13_so(7 downto 3));
            for k in 1 to 2 loop
                v_cd := signed(resize(p13_w(k), 10)) - signed(resize(p13_c(k), 10));
                case to_integer(p13_so(7 downto 6)) is
                    when 0 => p14_d(k) <= (others => '0');
                    when 1 => p14_d(k) <= shift_left(resize(shift_right(v_cd, 2), 16), 5);
                    when 2 => p14_d(k) <= shift_left(resize(shift_right(v_cd, 1), 16), 5);
                    when others => p14_d(k) <= shift_left(resize(shift_right(v_cd, 1), 16) + resize(shift_right(v_cd, 2), 16), 5);
                end case;
            end loop;
            p14_c <= p13_c; p14_nd <= p13_nd;

            ---------------- stage 15: blended colour
            for k in 0 to 2 loop
                v_yy := resize(signed(resize(p14_c(k), 10)), 18) + resize(shift_right(p14_d(k), 5), 18);
                if v_yy < 0 then p15_c(k) <= (others => '0');
                elsif v_yy > 255 then p15_c(k) <= (others => '1');
                else p15_c(k) <= unsigned(v_yy(7 downto 0)); end if;
            end loop;
            p15_nd <= p14_nd;

            ---------------- stage 16: notch (luma), to 10 bits
            case to_integer(p15_nd) is
                when 0 => v_m := p15_c(0);
                when 1 => v_m := p15_c(0) - shift_right(p15_c(0), 5);
                when others => v_m := p15_c(0) - shift_right(p15_c(0), 4);
            end case;
            p16_y <= v_m & "00"; p16_u <= p15_c(1) & "00"; p16_v <= p15_c(2) & "00";

            ---------------- 17..22: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums on the 8-bit chroma, luma [1 2 1]); 23: luma toward black + gate; 24 pad
            r_cur <= std_logic_vector(p15_c(1)) & std_logic_vector(p15_c(2));
            s_boxu <= s_boxu + unsigned(r_cur(15 downto 8)) - unsigned(r_old(15 downto 8));
            s_boxv <= s_boxv + unsigned(r_cur(7 downto 0)) - unsigned(r_old(7 downto 0));
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 13) + s_boxud; s_u22 <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 13) + s_boxvd; s_v22 <= v_sc(11 downto 2);
            s_y17 <= p16_y; s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19;
            s_y21 <= s_y20; s_y22x <= s_y21;
            v_sy := resize(s_y20, 12) + shift_left(resize(s_y21, 12), 1) + resize(s_y22x, 12);
            s_yf22 <= v_sy(11 downto 2);
            if s_yf22 > 64 then v_dly := s_yf22 - 64; else v_dly := (others => '0'); end if;
            if s_gav_sr(21) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dly, 3) - shift_right(v_dly, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= r_sy(2);
    data_out.vsync_n <= r_sy(1);
    data_out.field_n <= r_sy(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture chartreuse;
