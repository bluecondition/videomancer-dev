-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cathedral v0.6.0 -- Heirloom #41 (rebuilt 2026-10-27, phase 2 2026-10-30), from
-- WIN_20230602_01_30_58.
--
-- The photo is an RGB colourizer patch, mirrored left / right and top / bottom:
--   G   a vertical ramp (bright green at the top -> black at the bottom)
--   R   a hard key: the pink shapes and the warm stripes / spikes around them
--   B   the pink layer (diamond, top / bottom triangles, horizontal wedges)
--   teal bands: contour bands of an egg-crate F = h(|Y|) - g(x') (two sine-like
--       shapers, x' = distance to the nearest saddle column), banded by a folder,
--       keyed off inside the lenses; x-order soft rising edges, a dark trailing
--       ramp and a dark-green line after each band.
--
-- The program redraws that patch:
--   band   F = h(|Y| ky) - g(x'), phase = (F - c1) M + flow, band where F > c1 and
--          frac(phase) < 0.445 (inverted by S9)
--   pink   dD = |Y| + 0.82u - RD, dE = 293 - |Y| + 0.86u, dW = ||Y| - 261| - wk (u - 122)
--          (u = |X - 990|), pink where any < 0; its B soft in x-order (two 1/8 poles)
--   warm   any of dD, dE, dW below spill x (250, 300, 80) x (0.2 + 0.8 tri(Y / 68))
--   edges  a 20-pixel look-ahead window on the band key: widened true key,
--          signed distances to the rising edge and the true end -> curve ROMs
--   colour RGB 8-bit, then RGB -> YUV by three contribution ROMs
--
--   K1  Vaults   lenses across (x0.41 .. x2.46; the photo at 50 %)
--   K2  Arch     vertical scale of the arches (x0.5 .. x4; the photo at 33 %)
--   K3  Diamond  pink diamond size (10 .. 569 lines, 226 = the photo at 50 %); its
--                warm halo shrinks with it below the photo size
--   K4  Speed    bipolar: bands stream out of (or into) the lenses, stripes roll,
--                diamond and wedges breathe; still in the centre
--   K5  Spill    reach of the warm stripes around the pink shapes
--   K6  Picture  source depth: bends the bands and the keyed ramps
--   S7  Wedges   the horizontal pink wedges (On)
--   S8  Shade    soft / dark-edged band shading (On) or flat cut bands
--   S9  Invert   bands <-> gaps
--   S10 Palette  Jewel (the photo) / Night (RGB -> BRG)
--   S11 Source   Video / Osc
--   P12 TRACERY  band density: one bold vault ring .. the photo .. dense filigree
--
-- Heirloom v0.2.0 shell: timing-ID virtual-1080 steps, vblank zone with per-field
-- latches, measured width, generated avid, source line buffer, output conditioning
-- (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16), blanking gate on the
-- latency-aligned generated avid.  One serial shift-add multiplier runs the per-field
-- jobs (zone) and the per-line engine (during each line, for the next line).
-- Latency 56 clocks in every mode.  Colours in hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cathedral_pkg.all;
use work.all;

architecture cathedral of program_top is
    constant C_LAT : integer := 56;

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
    function f_tri7(v : unsigned(15 downto 0)) return unsigned is
    begin
        if v(15) = '1' then return not v(14 downto 8); else return v(14 downto 8); end if;
    end function;
    type t_q is array (0 to 63) of integer;
    constant C_QS : t_q := (
        2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47,
        50, 53, 56, 59, 62, 64, 67, 70, 72, 75, 77, 80, 82, 85, 87, 89,
        91, 94, 96, 98, 100, 102, 104, 105, 107, 109, 110, 112, 113, 115, 116, 117,
        118, 119, 120, 121, 122, 123, 124, 125, 125, 126, 126, 126, 127, 127, 127, 127);
    function f_sin8(p : unsigned(7 downto 0)) return signed is
        variable k : integer;
        variable q : unsigned(5 downto 0);
    begin
        if p(6) = '1' then q := not p(5 downto 0); else q := p(5 downto 0); end if;
        k := C_QS(to_integer(q));
        if p(7) = '1' then return to_signed(-k, 8); else return to_signed(k, 8); end if;
    end function;
    -- v x w / 16 helpers, w = 0 .. 16 (one add of gated operands, or the full shift)
    function f_mu(v : unsigned(8 downto 0); w : unsigned(4 downto 0)) return unsigned is
        variable g0, g1, g2, g3 : unsigned(13 downto 0);
    begin
        if w(4) = '1' then return shift_left(resize(v, 14), 4); end if;
        g0 := (others => '0'); g1 := (others => '0'); g2 := (others => '0'); g3 := (others => '0');
        if w(0) = '1' then g0 := resize(v, 14); end if;
        if w(1) = '1' then g1 := shift_left(resize(v, 14), 1); end if;
        if w(2) = '1' then g2 := shift_left(resize(v, 14), 2); end if;
        if w(3) = '1' then g3 := shift_left(resize(v, 14), 3); end if;
        return (g0 + g1) + (g2 + g3);
    end function;
    function f_ms(v : signed(9 downto 0); w : unsigned(4 downto 0)) return signed is
        variable g0, g1, g2, g3 : signed(14 downto 0);
    begin
        if w(4) = '1' then return shift_left(resize(v, 15), 4); end if;
        g0 := (others => '0'); g1 := (others => '0'); g2 := (others => '0'); g3 := (others => '0');
        if w(0) = '1' then g0 := resize(v, 15); end if;
        if w(1) = '1' then g1 := shift_left(resize(v, 15), 1); end if;
        if w(2) = '1' then g2 := shift_left(resize(v, 15), 2); end if;
        if w(3) = '1' then g3 := shift_left(resize(v, 15), 3); end if;
        return (g0 + g1) + (g2 + g3);
    end function;

    ---------------------------------------------------------------------------
    -- ROMs / RAM
    ---------------------------------------------------------------------------
    signal rom_g  : t_g  := C_G;
    signal rom_h  : t_h  := C_H;
    signal rom_bc : t_bc := C_BC;
    signal rom_gr : t_gr := C_GR;
    signal rom_cr : t_cr := C_CR;
    signal rom_ce : t_ce := C_CE;
    signal rom_kt : t_kt := C_KT;
    signal rom_mr : t_mr := C_MR;
    signal rom_mg : t_mg := C_MG;
    signal rom_mb : t_mb := C_MB;
    signal s_g_ra : unsigned(8 downto 0) := (others => '0');
    signal s_h_ra, s_bc_ra, s_cr_ra, s_ce_ra : unsigned(7 downto 0) := (others => '0');
    signal s_gr_ra : unsigned(8 downto 0) := (others => '0');
    signal s_kt_ra : unsigned(9 downto 0) := (others => '0');
    signal s_mr_ra, s_mg_ra, s_mb_ra : unsigned(7 downto 0) := (others => '0');
    signal q_g, q_h, q_bc, q_gr, q_cr, q_ce, q_kt : std_logic_vector(15 downto 0) := (others => '0');
    signal q_mr, q_mg, q_mb : std_logic_vector(31 downto 0) := (others => '0');

    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"6E");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"6E";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"6E";

    ---------------------------------------------------------------------------
    -- timing / controls
    ---------------------------------------------------------------------------
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal g_k1, g_k2, g_k3, g_k5, g_p12 : unsigned(12 downto 0) := (others => '0');
    signal s_warm : unsigned(1 downto 0) := (others => '0');
    signal gl_cnt : unsigned(4 downto 0) := (others => '0');
    signal gl_on : std_logic := '0';
    signal gl_t, gl_c, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal j_slot : unsigned(4 downto 0) := (others => '0');
    signal j_n : unsigned(3 downto 0) := (others => '0');
    signal j_on : std_logic := '0';
    signal zf_gl, zf_j, zf_l0, zf_cp, zf_k0, zf_k1, zf_k2, zf_k3, zf_q0, zf_q1, zf_q2, zf_q3, zf_sv : std_logic := '0';
    signal s_wedge, s_shade, s_inv, s_night, s_osc : std_logic := '0';
    signal s_sp : signed(10 downto 0) := (others => '0');
    -- serial multiplier: a (signed 27) x b (unsigned 16)
    signal m_a : signed(26 downto 0) := (others => '0');
    signal m_b : unsigned(15 downto 0) := (others => '0');
    signal m_acc : signed(43 downto 0) := (others => '0');
    signal m_i : unsigned(4 downto 0) := (others => '0');
    signal m_run, m_done : std_logic := '0';
    signal m_jid : unsigned(4 downto 0) := (others => '0');
    signal m_ld : std_logic := '0';
    -- per-field constants
    signal f_vw74 : unsigned(12 downto 0) := to_unsigned(1994, 13);
    signal f_inc, f_kj, f_spill, f_m : unsigned(15 downto 0) := (others => '0');
    signal f_npre : unsigned(10 downto 0) := to_unsigned(997, 11);
    signal f_incp : unsigned(17 downto 0) := (others => '0');
    signal f_ph0 : unsigned(23 downto 0) := (others => '0');
    signal f_rd : unsigned(9 downto 0) := to_unsigned(226, 10);
    signal f_rdf : signed(11 downto 0) := to_signed(226, 12);
    signal f_rdb : unsigned(15 downto 0) := to_unsigned(27648, 16);
    signal f_rdc : unsigned(7 downto 0) := to_unsigned(226, 8);
    signal f_hq : unsigned(15 downto 0) := to_unsigned(32770, 16);
    signal f_spilld : unsigned(15 downto 0) := (others => '0');
    signal f_flow : unsigned(15 downto 0) := (others => '0');
    signal f_sroll : unsigned(15 downto 0) := (others => '0');
    signal f_lm : unsigned(15 downto 0) := (others => '0');
    signal f_sval, f_sval2 : signed(7 downto 0) := (others => '0');
    signal f_wk : unsigned(5 downto 0) := to_unsigned(23, 6);
    signal f_wk122 : unsigned(5 downto 0) := to_unsigned(10, 6);
    signal f_sinc : unsigned(15 downto 0) := to_unsigned(964, 16);
    signal f_sph0 : unsigned(15 downto 0) := (others => '0');
    signal f_depth : unsigned(7 downto 0) := (others => '0');
    signal f_oxinc : unsigned(7 downto 0) := to_unsigned(51, 8);
    -- per-line engine (computes the next line during the current one)
    signal l_yq8 : unsigned(18 downto 0) := (others => '0');
    signal l_sph : unsigned(15 downto 0) := (others => '0');
    signal l_st : unsigned(4 downto 0) := (others => '0');
    signal l_y : unsigned(10 downto 0) := (others => '0');
    signal l_ay : unsigned(9 downto 0) := (others => '0');
    signal l_thr : unsigned(7 downto 0) := (others => '0');
    signal l_t : unsigned(17 downto 0) := (others => '0');
    signal l_rng : unsigned(15 downto 0) := x"ACE1";
    signal l_fn : signed(4 downto 0) := (others => '0');
    signal l_kick : std_logic := '0';
    -- staging (n_) and active (a_ = field stages, b_ = composite stages) per-line values
    signal n_hy, a_hy : unsigned(11 downto 0) := (others => '0');
    signal n_fn, a_fn : signed(4 downto 0) := (others => '0');
    signal n_ld, a_ld : signed(11 downto 0) := (others => '0');
    signal n_le, a_le : signed(11 downto 0) := (others => '0');
    signal n_lw, a_lw : signed(11 downto 0) := (others => '0');
    signal n_td, n_te, n_tw, a_td, a_te, a_tw : unsigned(9 downto 0) := (others => '0');
    signal n_gb, n_bb, n_gw, n_gwp, n_gc, n_rw, n_bbase : unsigned(7 downto 0) := (others => '0');
    signal b_gb, b_bb, b_gw, b_gwp, b_gc, b_rw, b_bbase : unsigned(7 downto 0) := (others => '0');
    signal s_lend : unsigned(6 downto 0) := (others => '1');
    signal s_gavq : std_logic := '0';
    signal s_cpa, s_cpb : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_osc8 : unsigned(7 downto 0) := to_unsigned(110, 8);
    signal s_vid8 : unsigned(7 downto 0) := to_unsigned(110, 8);

    -- beam
    signal b_x : signed(12 downto 0) := (others => '0');
    signal b_ph : unsigned(23 downto 0) := (others => '0');
    signal b_cnt : unsigned(10 downto 0) := (others => '0');
    signal b_px : unsigned(10 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    type t_sl is array (natural range <>) of std_logic;
    signal d_av : t_sl(1 to 11) := (others => '0');
    signal p1_xp : signed(13 downto 0) := (others => '0');
    signal p2_u, p3_u : unsigned(10 downto 0) := (others => '0');
    signal p3_g : unsigned(11 downto 0) := (others => '0');
    signal p3_src : signed(8 downto 0) := (others => '0');
    signal p3_a1, p3_a2 : unsigned(10 downto 0) := (others => '0');
    signal p4_f : signed(13 downto 0) := (others => '0');
    signal p4_svlo, p4_svhi : signed(13 downto 0) := (others => '0');
    signal p4_a3 : unsigned(16 downto 0) := (others => '0');
    signal p4_dd, p4_de : signed(12 downto 0) := (others => '0');
    signal p5_f : signed(13 downto 0) := (others => '0');
    signal p5_sv : signed(8 downto 0) := (others => '0');
    signal p5_dd, p5_de, p5_dw : signed(12 downto 0) := (others => '0');
    signal p6_f : signed(13 downto 0) := (others => '0');
    signal p6_svk : signed(9 downto 0) := (others => '0');
    signal p6_dd, p6_de, p6_dw : signed(12 downto 0) := (others => '0');
    signal p7_fc : signed(13 downto 0) := (others => '0');
    signal p7_dd, p7_de, p7_dw : signed(12 downto 0) := (others => '0');
    signal p8_pos, p8_pk, p8_kw : std_logic := '0';
    signal p8_fcc : unsigned(11 downto 0) := (others => '0');
    signal p9_m0, p9_m1, p9_m2 : unsigned(15 downto 0) := (others => '0');
    signal p9_pos, p9_pk, p9_kw, p10_pos, p10_pk, p10_kw : std_logic := '0';
    signal p10_ph : unsigned(15 downto 0) := (others => '0');
    signal p11_kb, p11_pk, p11_kw : std_logic := '0';
    signal c0_a1, c0_a0 : std_logic_vector(4 downto 0) := (others => '0');
    type t_u2 is array (0 to 4) of unsigned(1 downto 0);
    signal c0_i1, c0_i0 : t_u2 := (others => (others => '0'));
    -- window (index i = the pixel i + 1 clocks behind the newest)
    signal wv, wp, wb, ww, wa : std_logic_vector(31 downto 0) := (others => '0');
    -- composite stages (pixel q = the one at wv(20))
    signal c1_kv, c1_kx, c1_pk, c1_kw, c1_kb, c1_av : std_logic := '0';
    signal c1_jr, c1_jf : unsigned(4 downto 0) := (others => '0');
    signal c1_sig : unsigned(7 downto 0) := (others => '0');
    signal c_kvf, c_sr, c_se : unsigned(5 downto 0) := (others => '1');
    signal c_pkx, c_first : std_logic := '1';
    signal c2_kv, c2_kx, c2_pk, c2_kw, c2_kb : std_logic := '0';
    signal c2_jr, c2_jf : unsigned(4 downto 0) := (others => '0');
    signal c_s1, c_s2 : unsigned(7 downto 0) := (others => '0');
    signal c3_kx, c3_near, c3_post, c3_pk, c3_kw : std_logic := '0';
    signal c4_kx, c4_near, c4_post, c4_pk, c4_kw : std_logic := '0';
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal d_bs : t_u8(3 to 8) := (others => (others => '0'));
    signal c5_kx, c5_inz, c5_post : std_logic := '0';
    signal c5_ag, c5_ab, c5_sg, c5_sb : unsigned(4 downto 0) := (others => '0');
    signal c5_gg, c5_gr : unsigned(7 downto 0) := (others => '0');
    signal c6_tg : unsigned(7 downto 0) := (others => '0');
    signal c6_wg, c6_wr : unsigned(4 downto 0) := (others => '0');
    signal c6_wbp : unsigned(13 downto 0) := (others => '0');
    signal c6_gg, c6_gr : unsigned(7 downto 0) := (others => '0');
    signal c7_dg : signed(9 downto 0) := (others => '0');
    signal c7_gg, c7_gr : unsigned(7 downto 0) := (others => '0');
    signal c7_wg, c7_wr : unsigned(4 downto 0) := (others => '0');
    signal c7_bb : unsigned(13 downto 0) := (others => '0');
    signal c8_pg : signed(14 downto 0) := (others => '0');
    signal c8_pr : unsigned(13 downto 0) := (others => '0');
    signal c8_gg, c8_gr, c8_bb : unsigned(7 downto 0) := (others => '0');
    signal c9_r, c9_g, c9_b : unsigned(7 downto 0) := (others => '0');
    signal c12_y, c12_r, c12_b : unsigned(10 downto 0) := (others => '0');
    signal c12_cr0, c12_cr1, c12_cr2, c12_cb0, c12_cb1, c12_cb2 : signed(9 downto 0) := (others => '0');
    signal c13_y : unsigned(12 downto 0) := (others => '0');
    signal c13_cr, c13_cb : signed(11 downto 0) := (others => '0');
    signal c14_y : unsigned(11 downto 0) := (others => '0');
    signal c14_u, c14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p20_y, p20_u, p20_v : unsigned(9 downto 0) := (others => '0');
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y21, s_y22, s_y23, s_y24, s_y25, s_y26x, s_yf26 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yo, s_uo, s_vo : t_u10(0 to 1) := (others => to_unsigned(512, 10));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- ROMs and the source line buffer
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            q_g  <= rom_g(to_integer(s_g_ra));
            q_h  <= rom_h(to_integer(s_h_ra));
            q_bc <= rom_bc(to_integer(s_bc_ra));
            q_gr <= rom_gr(to_integer(s_gr_ra));
            q_cr <= rom_cr(to_integer(s_cr_ra));
            q_ce <= rom_ce(to_integer(s_ce_ra));
            q_kt <= rom_kt(to_integer(s_kt_ra));
            q_mr <= rom_mr(to_integer(s_mr_ra));
            q_mg <= rom_mg(to_integer(s_mg_ra));
            q_mb <= rom_mb(to_integer(s_mb_ra));
        end if;
    end process p_rom;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, sequencer, per-line engine
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_vw : unsigned(13 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_k : unsigned(9 downto 0);
        variable v_g : unsigned(12 downto 0);
        variable v_r : signed(43 downto 0);
        variable v_p : signed(28 downto 0);
        variable v_s : signed(11 downto 0);
        variable v_u : unsigned(17 downto 0);
        variable v_i : unsigned(7 downto 0);
        variable v_sp : signed(10 downto 0);
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
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 1023 then
                s_zc <= s_zc + 1;
            end if;

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
            -- speed: bipolar about the centre, +-24 deadband
            v_sp := signed(resize(unsigned(registers_in(3)), 11)) - 512;
            if v_sp > -24 and v_sp < 24 then s_sp <= (others => '0'); else s_sp <= v_sp; end if;

            -- per field: switches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                if s_warm /= 3 then s_warm <= s_warm + 1; end if;
                s_wedge <= registers_in(6)(0);
                s_shade <= registers_in(6)(1);
                s_inv   <= registers_in(6)(2);
                s_night <= registers_in(6)(3);
                s_osc   <= registers_in(6)(4);
                f_depth <= unsigned(registers_in(5)(9 downto 2));
            end if;

            -- registered decodes of the zone counter
            if s_zone = '1' and s_zc = 18 then zf_gl <= '1'; else zf_gl <= '0'; end if;
            if s_zone = '1' and s_zc = 44 then zf_k0 <= '1'; else zf_k0 <= '0'; end if;
            if s_zone = '1' and s_zc = 48 then zf_k1 <= '1'; else zf_k1 <= '0'; end if;
            if s_zone = '1' and s_zc = 52 then zf_k2 <= '1'; else zf_k2 <= '0'; end if;
            if s_zone = '1' and s_zc = 56 then zf_k3 <= '1'; else zf_k3 <= '0'; end if;
            if s_zone = '1' and s_zc = 47 then zf_q0 <= '1'; else zf_q0 <= '0'; end if;
            if s_zone = '1' and s_zc = 51 then zf_q1 <= '1'; else zf_q1 <= '0'; end if;
            if s_zone = '1' and s_zc = 55 then zf_q2 <= '1'; else zf_q2 <= '0'; end if;
            if s_zone = '1' and s_zc = 59 then zf_q3 <= '1'; else zf_q3 <= '0'; end if;
            if s_zone = '1' and s_zc = 63 then zf_j <= '1'; else zf_j <= '0'; end if;
            if s_zone = '1' and s_zc = 228 then zf_sv <= '1'; else zf_sv <= '0'; end if;
            if s_zone = '1' and s_zc = 400 then zf_l0 <= '1'; else zf_l0 <= '0'; end if;
            if s_zone = '1' and s_zc = 600 then zf_cp <= '1'; else zf_cp <= '0'; end if;

            -- glides: one shared datapath, 4 clocks per channel (zc 20 .. 39)
            if zf_gl = '1' then gl_cnt <= (others => '0'); gl_on <= '1';
            elsif gl_on = '1' then
                if gl_cnt = 19 then gl_on <= '0'; end if;
                gl_cnt <= gl_cnt + 1;
            end if;
            case to_integer(gl_cnt(4 downto 2)) is
                when 0 => v_k := unsigned(registers_in(0)); v_g := g_k1;
                when 1 => v_k := unsigned(registers_in(1)); v_g := g_k2;
                when 2 => v_k := unsigned(registers_in(2)); v_g := g_k3;
                when 3 => v_k := unsigned(registers_in(4)); v_g := g_k5;
                when others => v_k := unsigned(registers_in(7)); v_g := g_p12;
            end case;
            gl_t <= v_k & "000"; gl_c <= v_g;
            gl_d <= signed(resize(gl_t, 14)) - signed(resize(gl_c, 14));
            if (gl_d > -8 and gl_d < 8) or s_warm /= 3 then gl_n <= gl_t;
            else
                v_d := signed(resize(gl_c, 14)) + shift_right(gl_d, 3);
                gl_n <= unsigned(v_d(12 downto 0));
            end if;
            if gl_on = '1' and gl_cnt(1 downto 0) = 3 then
                case to_integer(gl_cnt(4 downto 2)) is
                    when 0 => g_k1 <= gl_n;
                    when 1 => g_k2 <= gl_n;
                    when 2 => g_k3 <= gl_n;
                    when 3 => g_k5 <= gl_n;
                    when others => g_p12 <= gl_n;
                end case;
            end if;

            -- knob curves from the KT ROM (index = knob / 4)
            if zf_k0 = '1' then s_kt_ra <= "00" & g_k1(12 downto 5); end if;
            if zf_k1 = '1' then s_kt_ra <= "01" & g_k2(12 downto 5); end if;
            if zf_k2 = '1' then s_kt_ra <= "10" & g_k5(12 downto 5); end if;
            if zf_k3 = '1' then s_kt_ra <= "11" & g_p12(12 downto 5); end if;
            if zf_q0 = '1' then f_inc <= unsigned(q_kt); end if;
            if zf_q1 = '1' then f_kj <= unsigned(q_kt); end if;
            if zf_q2 = '1' then f_spill <= unsigned(q_kt); end if;
            if zf_q3 = '1' then f_m <= unsigned(q_kt); end if;
            -- diamond radius RD = 10 + K3 (19456 + 16 K3) / 2^16 (10 .. 569, 226 at 512);
            -- its warm halo scales by min(RD, 226) / 226 (Q.15, exactly 1 at the photo)
            f_rdb <= to_unsigned(19456, 16) + shift_left(resize(g_k3(12 downto 3), 16), 4);
            if f_rd < 226 then f_rdc <= f_rd(7 downto 0); else f_rdc <= to_unsigned(226, 8); end if;
            f_hq <= shift_left(resize(f_rdc, 16), 7) + shift_left(resize(f_rdc, 16), 4) + resize(f_rdc, 16);
            if zf_q0 = '1' then
                f_vw74 <= resize(s_vw, 13) + 74;
                case to_integer(s_xs) is
                    when 4      => f_oxinc <= to_unsigned(102, 8);
                    when 3      => f_oxinc <= to_unsigned(77, 8);
                    when others => f_oxinc <= to_unsigned(51, 8);
                end case;
            end if;
            -- breathing LFOs (after the lm job)
            if zf_sv = '1' then
                f_sval <= f_sin8(f_lm(15 downto 8));
                v_i := resize(shift_right(resize(f_lm(15 downto 8), 11) * 5, 3), 8) + 40;
                f_sval2 <= f_sin8(v_i);
            end if;

            -- serial multiplier P = a x b: 16 steps of P = (P + b0 a 2^16) / 2.
            -- per-field jobs 0 .. 10 in 24-clock slots from zc 63; per-line jobs 16 .. 19
            m_ld <= '0';
            if zf_j = '1' then
                j_on <= '1'; j_slot <= to_unsigned(23, 5); j_n <= (others => '1');
            elsif j_on = '1' then
                if j_slot = 23 then
                    j_slot <= (others => '0');
                    if j_n = 10 then j_on <= '0';
                    else
                        j_n <= j_n + 1;
                        m_ld <= '1'; m_jid <= resize(j_n + 1, 5);
                    end if;
                else
                    j_slot <= j_slot + 1;
                end if;
            end if;

            -- per-line engine (states 1 .. 20), kicked at the zone (line 0) and every g_start
            l_kick <= '0';
            if zf_l0 = '1' then
                l_yq8 <= (others => '0'); l_sph <= f_sph0; l_kick <= '1';
            elsif g_start = '1' and s_zone = '0' then
                l_kick <= '1';
            end if;
            if l_kick = '1' then
                l_st <= to_unsigned(1, 5);
            elsif l_st /= 0 then
                l_st <= l_st + 1;
                if l_st = 20 then l_st <= (others => '0'); end if;
            end if;
            case to_integer(l_st) is
                when 1 =>
                    l_y <= l_yq8(18 downto 8);
                    l_yq8 <= l_yq8 + resize(s_ys, 19);
                    if l_sph(15) = '1' then l_thr <= l_sph(14 downto 7); else l_thr <= not l_sph(14 downto 7); end if;
                    l_sph <= l_sph + f_sinc;
                    l_rng <= l_rng(14 downto 0) & (l_rng(15) xor l_rng(13) xor l_rng(12) xor l_rng(10));
                when 2 =>
                    v_s := signed(resize(l_y, 12)) - 518;
                    if v_s < 0 then v_s := -v_s; end if;
                    l_ay <= unsigned(v_s(9 downto 0));
                    -- thr = 51 + tri * 204 / 256
                    v_u := shift_left(resize(l_thr, 18), 8) - shift_left(resize(l_thr, 18), 5)
                           - shift_left(resize(l_thr, 18), 4) - shift_left(resize(l_thr, 18), 2);
                    l_t <= v_u;
                    s_gr_ra <= l_y(10 downto 2);
                when 3 =>
                    l_thr <= 51 + l_t(15 downto 8);
                    if l_ay(9 downto 2) > 255 then s_bc_ra <= (others => '1'); else s_bc_ra <= resize(l_ay(9 downto 2), 8); end if;
                    m_ld <= '1'; m_jid <= to_unsigned(16, 5);
                    l_fn <= shift_right(l_fn + resize(signed(l_rng(3 downto 0)), 5), 1);
                when 4 =>
                    n_gw <= unsigned(q_gr(15 downto 8)); n_bbase <= unsigned(q_gr(7 downto 0));
                    n_rw <= 236 + resize(shift_right(shift_left(resize(l_y, 16), 4) + resize(l_y, 16), 10), 8);
                    n_fn <= l_fn;
                when 5 =>
                    n_gb <= unsigned(q_bc(15 downto 8)); n_bb <= unsigned(q_bc(7 downto 0));
                    if n_gw > 243 then n_gc <= to_unsigned(255, 8); else n_gc <= n_gw + 12; end if;
                    if n_gw > 4 then n_gwp <= n_gw - 4; else n_gwp <= (others => '0'); end if;
                    n_ld <= signed(resize(l_ay, 12)) - f_rdf;
                    n_le <= 293 - signed(resize(l_ay, 12));
                    v_s := signed(resize(l_ay, 12)) - 261;
                    if v_s < 0 then v_s := -v_s; end if;
                    n_lw <= v_s + signed(resize(f_wk122, 12));
                when others => null;
            end case;

            m_done <= '0';
            if m_ld = '1' then
                m_acc <= (others => '0'); m_i <= (others => '0'); m_run <= '1';
                case to_integer(m_jid) is
                    when 0 =>     -- pixels left of the band axis: ceil((vw + 74) / xs)
                        m_a <= signed(resize(f_vw74, 27)) + signed(resize(s_xs, 27)) - 1;
                        case to_integer(s_xs) is
                            when 4 =>      m_b <= to_unsigned(16384, 16);
                            when 3 =>      m_b <= to_unsigned(21846, 16);
                            when others => m_b <= to_unsigned(32768, 16);
                        end case;
                    when 1  => m_a <= signed(resize(f_inc, 27));             m_b <= resize(s_xs, 16);
                    when 2  => m_a <= signed(resize(f_incp, 27));            m_b <= resize(f_npre, 16);
                    when 3  => m_a <= signed(resize(g_k3(12 downto 3), 27)); m_b <= f_rdb;
                    when 4  => m_a <= resize(s_sp, 27);                      m_b <= to_unsigned(102, 16);
                    when 5  => m_a <= resize(s_sp, 27);                      m_b <= to_unsigned(580, 16);
                    when 6  => m_a <= resize(s_sp, 27);                      m_b <= to_unsigned(72, 16);
                    when 7  => m_a <= resize(f_sval, 27);                    m_b <= resize(f_rd, 16);
                    when 8  => m_a <= signed(resize(s_ys, 27));              m_b <= to_unsigned(241, 16);
                    when 9  => m_a <= signed(resize(f_spill, 27));           m_b <= f_hq;
                    when 10 => m_a <= signed(resize(f_wk, 27));              m_b <= to_unsigned(122, 16);
                    when 16 => m_a <= signed(resize(l_ay, 27));              m_b <= f_kj;
                    when 17 => m_a <= signed(resize(shift_left(resize(l_thr, 16), 8) - shift_left(resize(l_thr, 16), 2)
                                                    - shift_left(resize(l_thr, 16), 1), 27));
                               m_b <= f_spilld;
                    when 18 => m_a <= signed(resize(shift_left(resize(l_thr, 18), 8) + shift_left(resize(l_thr, 18), 5)
                                                    + shift_left(resize(l_thr, 18), 3) + shift_left(resize(l_thr, 18), 2), 27));
                               m_b <= f_spill;
                    when 19 => m_a <= signed(resize(shift_left(resize(l_thr, 16), 6) + shift_left(resize(l_thr, 16), 4), 27));
                               m_b <= f_spill;
                    when others => m_a <= (others => '0'); m_b <= (others => '0');
                end case;
            elsif m_run = '1' then
                if m_b(0) = '1' then v_p := resize(m_acc(43 downto 16), 29) + resize(m_a, 29);
                else v_p := resize(m_acc(43 downto 16), 29); end if;
                m_acc <= v_p(28 downto 0) & m_acc(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                if m_i = 15 then m_run <= '0'; m_done <= '1'; end if;
                m_i <= m_i + 1;
            end if;

            if m_done = '1' then
                v_r := m_acc;
                case to_integer(m_jid) is
                    when 0  => f_npre <= unsigned(v_r(26 downto 16));
                    when 1  => f_incp <= unsigned(v_r(17 downto 0));
                    when 2  => f_ph0 <= unsigned(v_r(23 downto 0));
                    when 3  => f_rd <= unsigned(v_r(25 downto 16)) + 10;
                    when 4  => f_flow <= f_flow + unsigned(v_r(21 downto 6));
                    when 5  => f_sroll <= f_sroll - unsigned(v_r(23 downto 8));
                    when 6  => f_lm <= f_lm + unsigned(v_r(21 downto 6));
                    when 7  => f_rdf <= signed(resize(f_rd, 12)) + resize(shift_right(v_r, 11), 12);
                               f_wk <= to_unsigned(23, 6) + unsigned(resize(shift_right(resize(f_sval2, 12) * 7, 8), 6));
                    when 8  => f_sinc <= unsigned(v_r(21 downto 6));
                    when 9  => f_sph0 <= f_sroll - 17352;
                               f_spilld <= unsigned(v_r(30 downto 15));
                    when 10 => f_wk122 <= unsigned(v_r(13 downto 8));
                    when 16 =>
                        if v_r(43 downto 23) /= 0 then s_h_ra <= (others => '1'); else s_h_ra <= unsigned(v_r(22 downto 15)); end if;
                        m_ld <= '1'; m_jid <= to_unsigned(17, 5);
                    when 17 =>
                        n_td <= unsigned(v_r(25 downto 16));
                        n_hy <= unsigned(q_h(11 downto 0));
                        m_ld <= '1'; m_jid <= to_unsigned(18, 5);
                    when 18 =>
                        n_te <= unsigned(v_r(25 downto 16));
                        m_ld <= '1'; m_jid <= to_unsigned(19, 5);
                    when 19 =>
                        n_tw <= unsigned(v_r(25 downto 16));
                    when others => null;
                end case;
            end if;

            -- copy strobes: A (field stages) 14 clocks and B (composite stages) 48
            -- clocks after the generated line ends; both after the zone's line-0 run
            s_gavq <= g_avid;
            s_cpa <= '0'; s_cpb <= '0';
            if s_gavq = '1' and g_avid = '0' then
                s_lend <= (others => '0');
            elsif s_lend /= 127 then
                s_lend <= s_lend + 1;
            end if;
            if s_lend = 14 or zf_cp = '1' then s_cpa <= '1'; end if;
            if s_lend = 48 or zf_cp = '1' then s_cpb <= '1'; end if;
            if s_cpa = '1' then
                a_hy <= n_hy; a_fn <= n_fn; a_ld <= n_ld; a_le <= n_le; a_lw <= n_lw;
                a_td <= n_td; a_te <= n_te; a_tw <= n_tw;
            end if;
            if s_cpb = '1' then
                b_gb <= n_gb; b_bb <= n_bb; b_gw <= n_gw; b_gwp <= n_gwp; b_gc <= n_gc; b_rw <= n_rw; b_bbase <= n_bbase;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
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
    -- Source side: line-buffer write (video luma or the oscillator plaid)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_t : signed(10 downto 0);
        variable v_y : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + resize(f_oxinc, 16);
            elsif s_avq = '1' then
                s_ox <= f_lm(11 downto 0) & "0000";
                s_oy <= s_oy + 61;
            end if;
            if s_zone = '1' then s_oy <= not (f_lm(12 downto 0) & "000"); s_ox <= f_lm(11 downto 0) & "0000"; end if;
            v_t := signed(resize(f_tri7(s_ox), 11)) + signed(resize(f_tri7(s_oy), 11)) - 127;
            v_t := shift_right(shift_left(v_t, 1) + v_t, 3);
            v_t := v_t + 110;
            s_osc8 <= unsigned(v_t(7 downto 0));
            v_y := signed(resize(unsigned(data_in.y(9 downto 2)), 11)) - 16;
            if v_y < 0 then s_vid8 <= (others => '0');
            elsif v_y > 219 then s_vid8 <= to_unsigned(219, 8);
            else s_vid8 <= unsigned(v_y(7 downto 0)); end if;
            s_lb_we <= s_avq and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
            if s_osc = '1' then s_lb_wd <= std_logic_vector(s_osc8);
            else s_lb_wd <= std_logic_vector(s_vid8); end if;
            if s_avq = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: virtual X (Q.1, centred), folded band-phase DDA |x - npre| incp
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x <= -signed(resize(s_vw, 13));
                b_ph <= f_ph0;
                b_cnt <= f_npre;
                b_px <= (others => '0');
            elsif g_avid = '1' then
                b_x <= b_x + signed(resize(s_xs, 13));
                b_px <= b_px + 1;
                if b_cnt /= 0 then
                    b_ph <= b_ph - resize(f_incp, 24);
                    b_cnt <= b_cnt - 1;
                else
                    b_ph <= b_ph + resize(f_incp, 24);
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u : unsigned(10 downto 0);
        variable v_x : signed(13 downto 0);
        variable v_f : signed(13 downto 0);
        variable v_la : std_logic_vector(19 downto 0);
        variable v_n : std_logic_vector(3 downto 0);
        variable v_s : unsigned(18 downto 0);
        variable v_kv : std_logic;
        variable v_or : std_logic;
        variable v_jr, v_jf : unsigned(4 downto 0);
        variable v_tf : unsigned(5 downto 0);
        variable v_e : signed(6 downto 0);
        variable v_dr : signed(6 downto 0);
        variable v_i1, v_i2 : signed(11 downto 0);
        variable v_kvf, v_sr, v_se : unsigned(5 downto 0);
        variable v_c : unsigned(8 downto 0);
        variable v_g : signed(10 downto 0);
        variable v_r : unsigned(13 downto 0);
        variable v_y : unsigned(12 downto 0);
        variable v_ch : signed(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_w : unsigned(4 downto 0);
        variable v_cr, v_ce : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: g-ROM address (folded band phase), pink offset, line-buffer read
            if b_ph(23) = '1' then s_g_ra <= b_ph(22 downto 14); else s_g_ra <= not b_ph(22 downto 14); end if;
            p1_xp <= resize(b_x, 14) - 60;
            s_lb_ra <= b_px(10 downto 1);
            d_av(1) <= g_avid;

            -- 2: u = |X - 990|
            if p1_xp < 0 then v_x := -p1_xp; else v_x := p1_xp; end if;
            p2_u <= unsigned(v_x(11 downto 1));

            -- 3: ROM / video fabric copies, pink shift-adds
            p3_g <= unsigned(q_g(11 downto 0));
            p3_src <= signed(resize(unsigned(s_lb_q), 9)) - 110;
            v_u := p2_u;
            p3_a1 <= (v_u - shift_right(v_u, 3)) - (shift_right(v_u, 4) - shift_right(v_u, 7));
            p3_a2 <= (v_u - shift_right(v_u, 3)) - shift_right(v_u, 6);
            p3_u <= v_u;

            -- 4: F = h - g + noise; source x depth (two partial products); wedge slope
            p4_f <= signed(resize(a_hy, 14)) - signed(resize(p3_g, 14)) + resize(a_fn, 14);
            p4_svlo <= resize(p3_src * signed('0' & f_depth(3 downto 0)), 14);
            p4_svhi <= resize(p3_src * signed('0' & f_depth(7 downto 4)), 14);
            p4_a3 <= resize(p3_u * f_wk, 17);
            p4_dd <= resize(a_ld, 13) + signed(resize(p3_a1, 13));
            p4_de <= resize(a_le, 13) + signed(resize(p3_a2, 13));

            -- 5
            v_f := shift_left(p4_svhi, 4) + p4_svlo;
            p5_sv <= resize(shift_right(v_f, 8), 9);
            p5_f <= p4_f;
            p5_dd <= p4_dd; p5_de <= p4_de;
            p5_dw <= resize(a_lw, 13) - signed(resize(p4_a3(16 downto 8), 13));

            -- 6: the source bends the field and the keyed ramps
            p6_f <= p5_f + resize(shift_left(resize(p5_sv, 11), 1), 14) + resize(p5_sv, 14);
            p6_svk <= resize(p5_sv, 10) + resize(shift_right(p5_sv, 2), 10);
            p6_dd <= p5_dd; p6_de <= p5_de; p6_dw <= p5_dw;

            -- 7
            p7_fc <= p6_f + 528;
            p7_dd <= p6_dd - resize(p6_svk, 13);
            p7_de <= p6_de - resize(p6_svk, 13);
            p7_dw <= p6_dw - resize(p6_svk, 13);

            -- 8: clamp Fc; pink and warm keys
            if p7_fc > 0 then p8_pos <= '1'; else p8_pos <= '0'; end if;
            if p7_fc < 0 then p8_fcc <= (others => '0');
            elsif p7_fc > 4095 then p8_fcc <= (others => '1');
            else p8_fcc <= unsigned(p7_fc(11 downto 0)); end if;
            if p7_dd < 0 or p7_de < 0 or (s_wedge = '1' and p7_dw < 0) then p8_pk <= '1'; else p8_pk <= '0'; end if;
            if p7_dd < signed(resize(a_td, 13)) or p7_de < signed(resize(a_te, 13))
               or (s_wedge = '1' and p7_dw < signed(resize(a_tw, 13))) then p8_kw <= '1'; else p8_kw <= '0'; end if;

            -- 9: band phase partial products
            p9_m0 <= p8_fcc * f_m(3 downto 0);
            p9_m1 <= p8_fcc * f_m(7 downto 4);
            p9_m2 <= p8_fcc * f_m(11 downto 8);
            p9_pos <= p8_pos; p9_pk <= p8_pk; p9_kw <= p8_kw;

            -- 10: phase = (Fc M) / 8 + flow
            v_s := shift_left(resize(p9_m2, 19), 8) + shift_left(resize(p9_m1, 19), 4) + resize(p9_m0, 19);
            p10_ph <= v_s(18 downto 3) + f_flow;
            p10_pos <= p9_pos; p10_pk <= p9_pk; p10_kw <= p9_kw;

            -- 11: band key
            if p10_pos = '1' and ((p10_ph < 29164) xor (s_inv = '1')) then p11_kb <= '1'; else p11_kb <= '0'; end if;
            p11_pk <= p10_pk; p11_kw <= p10_kw;
            for k in 2 to 11 loop d_av(k) <= d_av(k - 1); end loop;

            -- 12: the window (newest pixel at index 0)
            v_kv := p11_kb and not p11_pk and d_av(11);
            wv <= wv(30 downto 0) & v_kv;
            wp <= wp(30 downto 0) & (p11_pk and d_av(11));
            wb <= wb(30 downto 0) & p11_kb;
            ww <= ww(30 downto 0) & (p11_kw and d_av(11));
            wa <= wa(30 downto 0) & d_av(11);
            -- C0: groups of four of the look-ahead for the pixel that enters wv(20) next
            -- (its kv(r + j) = wv(19 - j), j = 1 .. 19, and the incoming bit for j = 20)
            v_la := wv(18 downto 0) & v_kv;
            -- v_la(19 - (j - 1)) = kv(r + j): bit 19 = j 1 ... bit 0 = j 20
            for g in 0 to 4 loop
                v_n := v_la(19 - 4 * g downto 16 - 4 * g);       -- j = 4g+1 .. 4g+4 (msb first)
                c0_a1(g) <= v_n(3) or v_n(2) or v_n(1) or v_n(0);
                c0_a0(g) <= not (v_n(3) and v_n(2) and v_n(1) and v_n(0));
                if v_n(3) = '1' then c0_i1(g) <= "00"; elsif v_n(2) = '1' then c0_i1(g) <= "01";
                elsif v_n(1) = '1' then c0_i1(g) <= "10"; else c0_i1(g) <= "11"; end if;
                if v_n(3) = '0' then c0_i0(g) <= "00"; elsif v_n(2) = '0' then c0_i0(g) <= "01";
                elsif v_n(1) = '0' then c0_i0(g) <= "10"; else c0_i0(g) <= "11"; end if;
            end loop;

            -- C1: pixel q at wv(20): widened key, first rise / fall ahead, pink-B signal (q + 11)
            v_or := '0';
            for k in 21 to 31 loop v_or := v_or or wv(k); end loop;
            for k in 15 to 19 loop v_or := v_or or wv(k); end loop;
            c1_kv <= wv(20);
            c1_kx <= wv(20) or (not wp(20) and v_or);
            v_jr := to_unsigned(21, 5); v_jf := to_unsigned(21, 5);
            for g in 4 downto 0 loop       -- the nearest group wins
                if c0_a1(g) = '1' then v_jr := to_unsigned(4 * g + 1, 5) + resize(c0_i1(g), 5); end if;
                if c0_a0(g) = '1' then v_jf := to_unsigned(4 * g + 1, 5) + resize(c0_i0(g), 5); end if;
            end loop;
            c1_jr <= v_jr; c1_jf <= v_jf;
            c1_pk <= wp(20); c1_kw <= ww(20); c1_kb <= wb(20); c1_av <= wa(20);
            if wp(9) = '1' then
                if wb(9) = '1' then v_c := resize(b_bbase, 9) + resize(b_bb(7 downto 1), 9);
                else v_c := resize(b_bbase, 9); end if;
                if v_c > 255 then c1_sig <= (others => '1'); else c1_sig <= v_c(7 downto 0); end if;
            else
                c1_sig <= (others => '0');
            end if;

            -- C2: run counters along the line; pink-B lowpass (two 3/16 poles)
            if c1_av = '0' then
                c_kvf <= (others => '1'); c_sr <= (others => '1'); c_se <= (others => '1'); c_first <= '1';
                c_s1 <= (others => '0'); c_s2 <= (others => '0');
            else
                if c1_kv = '1' then c_kvf <= (others => '0');
                elsif c_kvf /= 63 then c_kvf <= c_kvf + 1; end if;
                c_first <= '0';
                if c1_kx = '1' then
                    if c_pkx = '0' and c_first = '0' then c_sr <= (others => '0');
                    elsif c_sr /= 63 then c_sr <= c_sr + 1; end if;
                    c_se <= (others => '0');
                else
                    if c_pkx = '1' and c_first = '0' then c_se <= to_unsigned(1, 6);
                    elsif c_se /= 63 then c_se <= c_se + 1; end if;
                end if;
                c_pkx <= c1_kx;
                v_i1 := signed(resize(c1_sig, 12)) - signed(resize(c_s1, 12));
                v_i1 := signed(resize(c_s1, 12)) + shift_right(v_i1, 3);
                c_s1 <= unsigned(v_i1(7 downto 0));
                v_i2 := signed(resize(c_s1, 12)) - signed(resize(c_s2, 12));
                v_i2 := signed(resize(c_s2, 12)) + shift_right(v_i2, 3);
                c_s2 <= unsigned(v_i2(7 downto 0));
            end if;
            c2_kv <= c1_kv; c2_kx <= c1_kx; c2_pk <= c1_pk; c2_kw <= c1_kw; c2_kb <= c1_kb;
            c2_jr <= c1_jr; c2_jf <= c1_jf;

            -- C3: signed distances -> curve ROM addresses
            v_kvf := c_kvf; v_sr := c_sr; v_se := c_se;
            if c2_kv = '1' then v_tf := resize(c2_jf, 6) + 10;
            elsif v_kvf <= 11 then v_tf := 11 - v_kvf;
            else v_tf := to_unsigned(31, 6); end if;
            if c2_kx = '1' then
                v_e := -signed(resize(v_tf, 7));
                if v_sr > 47 then v_dr := to_signed(47, 7); else v_dr := signed(resize(v_sr, 7)); end if;
            else
                if v_se > 31 then v_e := to_signed(31, 7); else v_e := signed(resize(v_se, 7)); end if;
                if c2_jr <= 20 then v_dr := 5 - signed(resize(c2_jr, 7)); else v_dr := to_signed(-16, 7); end if;
            end if;
            s_cr_ra <= unsigned(resize(v_dr + 16, 8));
            s_ce_ra <= unsigned(resize(v_e + 32, 8));
            c3_kx <= c2_kx;
            if c2_kx = '0' and c2_jr <= 20 then c3_near <= '1'; else c3_near <= '0'; end if;
            if c2_kx = '0' and c2_jr > 20 and c2_pk = '0' and s_shade = '1' then c3_post <= '1'; else c3_post <= '0'; end if;
            c3_pk <= c2_pk; c3_kw <= c2_kw;
            d_bs(3) <= c_s2;

            -- C4: curve ROMs read
            c4_kx <= c3_kx; c4_near <= c3_near; c4_post <= c3_post; c4_pk <= c3_pk; c4_kw <= c3_kw;

            -- C5: curves (or flat), ground colour
            v_cr := q_cr; v_ce := q_ce;
            if s_shade = '1' then
                c5_ag <= unsigned(v_cr(9 downto 5)); c5_ab <= unsigned(v_cr(4 downto 0));
                c5_sg <= unsigned(v_ce(9 downto 5)); c5_sb <= unsigned(v_ce(4 downto 0));
            else
                if c4_kx = '1' then c5_ag <= to_unsigned(16, 5); else c5_ag <= (others => '0'); end if;
                if c4_kx = '1' then c5_ab <= to_unsigned(16, 5); else c5_ab <= (others => '0'); end if;
                c5_sg <= to_unsigned(16, 5); c5_sb <= to_unsigned(16, 5);
            end if;
            c5_kx <= c4_kx; c5_inz <= c4_kx or c4_near; c5_post <= c4_post;
            if c4_pk = '1' then c5_gg <= b_gwp; elsif c4_kw = '1' then c5_gg <= b_gw; else c5_gg <= b_gc; end if;
            if c4_pk = '1' or c4_kw = '1' then c5_gr <= b_rw; else c5_gr <= to_unsigned(8, 8); end if;

            -- C6: band target and weights
            if c5_inz = '1' then
                if c5_kx = '1' then v_w := c5_sg; else v_w := to_unsigned(16, 5); end if;
                v_r := f_mu(resize(b_gb, 9), v_w);
                c6_tg <= v_r(11 downto 4);
                c6_wg <= c5_ag; c6_wr <= c5_ag;
                if c5_kx = '1' then v_w := c5_sb; else v_w := to_unsigned(16, 5); end if;
                c6_wbp <= f_mu(resize(c5_ab, 9), v_w);
            elsif c5_post = '1' then
                c6_tg <= (others => '0');
                c6_wg <= 16 - c5_sg; c6_wr <= 16 - c5_sb;
                c6_wbp <= (others => '0');
            else
                c6_tg <= (others => '0'); c6_wg <= (others => '0'); c6_wr <= (others => '0');
                c6_wbp <= (others => '0');
            end if;
            c6_gg <= c5_gg; c6_gr <= c5_gr;

            -- C7
            c7_dg <= signed(resize(c6_tg, 10)) - signed(resize(c6_gg, 10));
            c7_gg <= c6_gg; c7_gr <= c6_gr; c7_wg <= c6_wg; c7_wr <= c6_wr;
            c7_bb <= f_mu(resize(b_bb, 9), c6_wbp(8 downto 4));

            -- C8: products
            c8_pg <= f_ms(c7_dg, c7_wg);
            c8_pr <= f_mu(resize(c7_gr, 9), c7_wr);
            c8_gg <= c7_gg; c8_gr <= c7_gr; c8_bb <= c7_bb(11 downto 4);
            for k in 4 to 8 loop d_bs(k) <= d_bs(k - 1); end loop;

            -- C9: R, G, B (8-bit)
            v_g := signed(resize(c8_gg, 11)) + resize(shift_right(c8_pg, 4), 11);
            if v_g < 0 then c9_g <= (others => '0'); elsif v_g > 255 then c9_g <= (others => '1');
            else c9_g <= unsigned(v_g(7 downto 0)); end if;
            c9_r <= c8_gr - c8_pr(11 downto 4);
            if d_bs(8) > c8_bb then c9_b <= d_bs(8); else c9_b <= c8_bb; end if;

            -- C10: palette (Night = R G B -> B R G), matrix ROM addresses
            if s_night = '1' then
                s_mr_ra <= c9_b; s_mg_ra <= c9_r; s_mb_ra <= c9_g;
            else
                s_mr_ra <= c9_r; s_mg_ra <= c9_g; s_mb_ra <= c9_b;
            end if;

            -- C11: matrix ROMs read.  C12: fabric copies
            c12_y <= unsigned(q_mr(31 downto 21)); c12_r <= unsigned(q_mg(31 downto 21)); c12_b <= unsigned(q_mb(31 downto 21));
            c12_cr0 <= signed(q_mr(20 downto 11)); c12_cr1 <= signed(q_mg(20 downto 11)); c12_cr2 <= signed(q_mb(20 downto 11));
            c12_cb0 <= signed(q_mr(10 downto 1));  c12_cb1 <= signed(q_mg(10 downto 1));  c12_cb2 <= signed(q_mb(10 downto 1));

            -- C13: sums
            c13_y <= resize(c12_y, 13) + resize(c12_r, 13) + resize(c12_b, 13);
            c13_cr <= resize(c12_cr0, 12) + resize(c12_cr1, 12) + resize(c12_cr2, 12);
            c13_cb <= resize(c12_cb0, 12) + resize(c12_cb1, 12) + resize(c12_cb2, 12);

            -- C14: luma (Q.1 -> 10-bit), chroma offsets and clamps
            c14_y <= resize(c13_y(12 downto 1), 12) + 64;
            v_ch := c13_cr + 512;
            if v_ch < 0 then c14_u <= (others => '0'); elsif v_ch > 1023 then c14_u <= (others => '1');
            else c14_u <= unsigned(v_ch(9 downto 0)); end if;
            v_ch := c13_cb + 512;
            if v_ch < 0 then c14_v <= (others => '0'); elsif v_ch > 1023 then c14_v <= (others => '1');
            else c14_v <= unsigned(v_ch(9 downto 0)); end if;

            -- C15 (= stage 46): luma knee above 848 (whites land at <= ~777 after the 13/16)
            if c14_y > 848 then
                v_y := resize(c14_y, 13) - 848;
                v_y := shift_right(v_y, 1) + 848;
                if v_y > 1023 then p20_y <= (others => '1'); else p20_y <= v_y(9 downto 0); end if;
            else
                p20_y <= c14_y(9 downto 0);
            end if;
            p20_u <= c14_u; p20_v <= c14_v;

            -- 47 .. 52: output conditioning (centre at 51 -> 52)
            s_csu(0) <= p20_u; s_csv(0) <= p20_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + p20_u - s_csu(7);
            s_boxv <= s_boxv + p20_v - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u26 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v26 <= v_sc(13 downto 4);
            s_y21 <= p20_y; s_y22 <= s_y21; s_y23 <= s_y22; s_y24 <= s_y23;
            s_y25 <= s_y24; s_y26x <= s_y25;
            v_sy := resize(s_y24, 12) + shift_left(resize(s_y25, 12), 1) + resize(s_y26x, 12);
            s_yf26 <= v_sy(11 downto 2);
            -- 53: luma toward black (~777 white); blanking gate
            if s_yf26 > 64 then v_dl := s_yf26 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(C_LAT - 4) = '1' then
                s_y27 <= s_yf26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u27 <= s_u26; s_v27 <= s_v26;
            else
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            end if;
            -- 54 .. 56
            s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;
            s_yo(0) <= s_y28; s_uo(0) <= s_u28; s_vo(0) <= s_v28;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yo(0));
    data_out.u       <= std_logic_vector(s_uo(0));
    data_out.v       <= std_logic_vector(s_vo(0));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cathedral;
