-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Marble v0.1.0 -- Heirloom (phase 3), built from the clip still WIN_20230914_01_59_34 at 4.0 s.
--
-- The still: curved cream bands with cyan nested-rhombus chevrons alternating with yellow / orange / red bands,
-- a green fringe on every cream band's leading edge, and blue-outlined dark figures in the warm bands; the arcs
-- roll up the frame (~150 lines/s), turning chevrons into knees into arcs.
--
-- The patch is an RGB patch (marble.py has every table; .heirloom_work/rb_marble/notes.md the measurements):
--   u    = ((x + 192) f(y) - 0.757) mod 1        a horizontal oscillator hard-synced at hsync, FM'd by a vertical
--          f(y) = f0 (1 + m cos(2 pi (y/Py + pv0)))  sine that rolls (pv0 steps per field)
--   B    = 197 kb(u) + lines                      the cream key (band ROM)
--   G    = 240 kg(u) + (1 - kg) floor             a wider, slower window; the warm floor darkens down the frame and
--                                                 inside the figures
--   R    = ring(dl) * kr(u)                       dl = a nested-rhombus distance: a 24-degree rotated triangle pair
--                                                 over a kaleidoscope-folded x (fold 945 virtual px, axis 535),
--                                                 bent by three drifting melt sines and the camera; kr dips at the
--                                                 cream band's leading edge (the green fringe)
--   figures: Fg = 7 (1.4 tu ty + 0.2 y/H + 0.15 M + 0.125 (1 - b1) - q0 + zigzag); dark inside, a blue line on |Fg|
--
--   K1  Bands     band frequency x1/4 .. x4 (512 = the still's 4 bands)
--   K2  Arcs      the FM sine's vertical frequency x1/4 .. x4 (512 = the still's 1900-line period)
--   K3  Lattice   lattice zoom x0.35 .. x2.8 (the cyan rhombi and the fold), anchored at the line start
--   K4  Speed     bipolar roll of the arcs (+-8 deadband); 698 = the clip's 150 lines/s upward
--   K5  Rings     rings per rhombus x0.30 .. x3.3
--   K6  Figures   figure size (none .. the still .. flooding the warm bands) and how hard the camera / Osc bends
--   S7  Twin      the band oscillator mirrored about the centre
--   S8  Inlay     the cream and warm keys swapped
--   S9  Negative  the lattice inverted (cyan bands with cream chevrons)
--   S10 Guns      RGB / GBR
--   S11 Source    Video / Osc
--   P12 MARBLE    FM depth 0 (straight stripes) .. 0.28 (the still, 321) .. 0.9, plus melt veining of the bands
--                 above 59 %
--
-- Engine: vblank-zone sequencer (13 slots on one serial multiplier, the shared glide); a per-line engine that
-- borrows the same multiplier for the FM cosine product and the band start phase; DDAs for the band phase, the
-- kaleidoscope fold and the rotated lattice pair; a 22-stage pixel path to a fixed RGB -> YCbCr matrix.  Virtual
-- 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16,
-- luma [1 2 1]/4, luma 13/16) before the blanking gate; syncs and the output delays through block-RAM rings.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.marble_pkg.all;
use work.all;

architecture marble of program_top is
    constant C_LAT : integer := 36;   -- = processing_delay_clks

    constant K_F0  : integer := 8648;      -- 2^22 / 485 (band frequency per virtual px, Q22)
    constant K_KV  : integer := 8830;      -- 2^24 / 1900 (FM phase per virtual line, Q24)
    constant C_UC  : integer := 12700352;  -- 0.757 * 2^24
    constant K_PV0 : integer := 8406276;   -- FM phase of line 0 at power-up (peak row 948)
    constant C_HA  : integer := 101656;    -- 262 * 388
    constant C_VA  : integer := 877438;    -- 262 * 3349
    constant C_TY0 : integer := 58605;     -- ty phase at line 0

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- v * xs / 2 (xs = 2, 3, 4)
    function f_xsm(v : signed(15 downto 0); xs : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(xs) is
            when 4      => return shift_left(v, 1);
            when 3      => return v + shift_right(v, 1);
            when others => return v;
        end case;
    end function;
    -- triangle of a Q16 phase: 0 at 0, 255 at 1/2
    function f_tri8(p : unsigned(15 downto 0)) return unsigned is
    begin
        if p(15) = '0' then return p(14 downto 7); else return not p(14 downto 7); end if;
    end function;
    -- triangle of an 11-bit phase: 0 at 0, 1023 at 1/2
    function f_tri10(p : unsigned(10 downto 0)) return unsigned is
    begin
        if p(10) = '0' then return p(9 downto 0); else return not p(9 downto 0); end if;
    end function;

    -- ROMs (one read port each, registered address and data)
    signal r_bd_a : unsigned(7 downto 0) := (others => '0');
    signal r_bd_q : unsigned(15 downto 0) := (others => '0');
    signal r_rg_a : unsigned(9 downto 0) := (others => '0');
    signal r_rg_q : unsigned(7 downto 0) := (others => '0');
    signal r_s1_a, r_s2_a, r_s3_a : unsigned(7 downto 0) := (others => '0');
    signal r_s1_q, r_s2_q, r_s3_q : signed(7 downto 0) := (others => '0');
    signal r_cs_a : unsigned(9 downto 0) := (others => '0');
    signal r_cs_q : signed(15 downto 0) := (others => '0');
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');

    -- block-RAM rings (written every clock, read N back: no collision)
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_oc  : t_ring16 := (others => x"8080");          -- output chroma box: U (8) + V (8)
    type t_ring4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal r_sy  : t_ring4 := (others => (others => '1'));    -- syncs
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_oc : signal is true;
    attribute no_rw_check of r_sy : signal is true;
    signal r_ly  : t_ring16 := (others => x"0040");          -- output luma delay (6 clocks)
    signal r_uv  : t_ring16 := (others => x"8080");          -- output chroma delay (3 clocks)
    attribute no_rw_check of r_ly : signal is true;
    attribute no_rw_check of r_uv : signal is true;
    signal r_lyq : std_logic_vector(15 downto 0) := x"0040";
    signal r_uvq : std_logic_vector(15 downto 0) := x"8080";
    signal r_wp  : unsigned(7 downto 0) := (others => '0');
    signal r_ocq : std_logic_vector(15 downto 0) := (others => '0');
    signal r_syq, r_syq2 : std_logic_vector(3 downto 0) := (others => '1');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ysr  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_half : unsigned(11 downto 0) := to_unsigned(960, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(321 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(698, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_inl, s_neg, s_gun, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field sequencer (one serial multiplier, shared with the line engine)
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(19 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal l_ld, l_ld2, l_neg : std_logic := '0';
    signal z_h   : signed(2 downto 0) := (others => '0');
    signal z_x1, z_x2, z_x3 : unsigned(15 downto 0) := to_unsigned(8192, 16);   -- 2^e (Q13)
    signal z_mq   : unsigned(14 downto 0) := (others => '0');
    signal z_v2   : unsigned(17 downto 0) := (others => '0');
    signal z_kvb  : unsigned(15 downto 0) := to_unsigned(8830, 16);
    signal z_lsh, z_lcv : unsigned(13 downto 0) := (others => '0');

    -- per-field constants
    signal c_f0   : unsigned(15 downto 0) := to_unsigned(8648, 16);   -- band frequency per vpx (Q22)
    signal c_fm   : unsigned(14 downto 0) := to_unsigned(2454, 15);   -- FM depth * f0
    signal c_kv   : unsigned(17 downto 0) := to_unsigned(8830, 18);   -- FM phase per line (Q24)
    signal c_rs   : signed(18 downto 0) := (others => '0');            -- roll per field (Q24, signed)
    signal c_rneg : std_logic := '0';
    signal c_dhp, c_dvp, c_dwp : unsigned(15 downto 0) := (others => '0');   -- lattice / fold per pixel (Q20)
    signal c_dhl, c_dvl : unsigned(19 downto 0) := (others => '0');          -- lattice per line (Q20, two's complement)
    signal c_w0   : unsigned(19 downto 0) := (others => '0');
    signal c_h00, c_v00, c_v0d : unsigned(19 downto 0) := (others => '0');
    signal c_gr   : unsigned(9 downto 0) := to_unsigned(256, 10);     -- rings gain (Q8)
    signal c_q0   : unsigned(8 downto 0) := to_unsigned(248, 9);      -- figure threshold (Q8)
    signal c_cg   : unsigned(2 downto 0) := to_unsigned(4, 3);        -- camera / Osc gain (/4)
    signal c_vg   : unsigned(2 downto 0) := (others => '0');          -- vein gain
    signal c_yfs  : unsigned(15 downto 0) := to_unsigned(60, 16);     -- y / 1080 step per line (Q16)
    signal c_mdx1, c_mdx2, c_mdx3, c_wxs, c_oxs : signed(15 downto 0) := (others => '0');   -- per pixel (Q16)
    signal c_mdy1, c_mdy2, c_mdy3 : signed(15 downto 0) := (others => '0');                -- per line (Q16)
    signal c_wys, c_oys, c_tys : unsigned(15 downto 0) := (others => '0');                 -- per line (Q16)
    signal c_pv0  : unsigned(23 downto 0) := to_unsigned(K_PV0, 24);  -- FM phase of line 0 (rolls)
    signal c_vdr  : unsigned(19 downto 0) := (others => '0');          -- lattice drift
    signal c_fm1  : unsigned(15 downto 0) := to_unsigned(19661, 16);  -- melt / Osc field drifts
    signal c_fm2  : unsigned(15 downto 0) := to_unsigned(58982, 16);
    signal c_fm3, c_fox, c_foy : unsigned(15 downto 0) := (others => '0');
    signal c_uk   : std_logic := '0';                                   -- SD: the 432-px sync offset

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (accumulators = the line being computed; n_* = its derived values)
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_pv   : unsigned(23 downto 0) := (others => '0');
    signal e_hl, e_vl : unsigned(19 downto 0) := (others => '0');
    signal e_m1, e_m2, e_m3, e_wy, e_oy, e_ty : unsigned(15 downto 0) := (others => '0');
    signal e_yf   : unsigned(16 downto 0) := (others => '0');
    signal e_f    : unsigned(16 downto 0) := (others => '0');
    signal n_su   : unsigned(19 downto 0) := (others => '0');
    signal n_us   : unsigned(23 downto 0) := (others => '0');
    signal n_go   : unsigned(7 downto 0) := (others => '0');
    signal n_cl   : signed(9 downto 0) := (others => '0');
    signal n_gm   : unsigned(4 downto 0) := (others => '0');
    signal l_su   : unsigned(19 downto 0) := (others => '0');
    signal l_go   : unsigned(7 downto 0) := (others => '0');
    signal l_cl   : signed(9 downto 0) := (others => '0');
    signal l_gm   : unsigned(4 downto 0) := (others => '0');
    signal l_ty   : unsigned(7 downto 0) := (others => '0');
    signal l_wy, l_oy : unsigned(7 downto 0) := (others => '0');

    -- video source (unipolar luma, black = 0), delayed to stage 4
    signal v_y0 : unsigned(9 downto 0) := (others => '0');
    signal v_s1, v_s2, v_s3, v_s4, v_s5 : unsigned(7 downto 0) := (others => '0');

    -- pixel path
    signal b_u    : unsigned(23 downto 0) := (others => '0');
    signal b_w, b_h, b_v : unsigned(19 downto 0) := (others => '0');
    signal b_m1, b_m2, b_m3, b_wx, b_ox : unsigned(15 downto 0) := (others => '0');
    signal b_tw   : std_logic := '0';
    signal p1_a1, p1_b1 : unsigned(9 downto 0) := (others => '0');
    signal p1_u, p2_u, p3_u, p4_u, p5_u, p6_u : unsigned(11 downto 0) := (others => '0');
    signal p1_wg, p2_wg, p3_wg, p4_wg, p5_wg, p6_wg, p7_wg : signed(5 downto 0) := (others => '0');
    signal p1_ox, p2_ox, p3_ox : unsigned(7 downto 0) := (others => '0');
    signal p2_dla : unsigned(11 downto 0) := (others => '0');
    signal p2_dlb : unsigned(10 downto 0) := (others => '0');
    signal p2_q, p3_q, p4_q, p5_q, p6_q, p7_q : unsigned(4 downto 0) := (others => '0');
    signal p3_s1, p3_s2, p3_s3 : signed(7 downto 0) := (others => '0');
    signal p3_dl  : unsigned(12 downto 0) := (others => '0');
    signal p4_s3  : signed(9 downto 0) := (others => '0');
    signal p4_dh  : unsigned(15 downto 0) := (others => '0');
    signal p4_src : signed(8 downto 0) := (others => '0');
    signal p5_m   : signed(9 downto 0) := (others => '0');
    signal p5_dlg, p6_dlg : unsigned(9 downto 0) := (others => '0');
    signal p5_cs  : signed(9 downto 0) := (others => '0');
    signal p6_mt  : signed(11 downto 0) := (others => '0');
    signal p6_mtt : signed(10 downto 0) := (others => '0');
    signal p6_ct  : signed(7 downto 0) := (others => '0');
    signal p6_vt  : signed(12 downto 0) := (others => '0');
    signal p7_dlr : signed(11 downto 0) := (others => '0');
    signal p7_uv  : unsigned(11 downto 0) := (others => '0');
    signal p7_mm  : signed(7 downto 0) := (others => '0');
    signal p8_tu  : unsigned(6 downto 0) := (others => '0');
    signal p8_f0  : signed(8 downto 0) := (others => '0');
    signal p9_pr  : unsigned(14 downto 0) := (others => '0');
    signal p9_f1  : signed(9 downto 0) := (others => '0');
    signal p10_p1 : unsigned(8 downto 0) := (others => '0');
    signal p10_f1 : signed(9 downto 0) := (others => '0');
    signal p10_r, p11_r, p12_r, p13_r, p14_r, p15_r : unsigned(7 downto 0) := (others => '0');
    signal p10_kb, p11_kb, p12_kb, p13_kb, p14_kb : unsigned(5 downto 0) := (others => '0');
    signal p10_kg, p11_kg, p12_kg : unsigned(5 downto 0) := (others => '0');
    signal p10_kr, p11_kr, p12_kr, p13_kr, p14_kr : unsigned(3 downto 0) := (others => '0');
    signal p11_fg : signed(12 downto 0) := (others => '0');
    signal p12_ln : unsigned(5 downto 0) := (others => '0');
    signal p12_gf : unsigned(7 downto 0) := (others => '0');
    signal p13_g1, p14_g1 : unsigned(7 downto 0) := (others => '0');
    signal p13_ln : unsigned(5 downto 0) := (others => '0');
    signal p14_ln : unsigned(2 downto 0) := (others => '0');
    signal p14_gl : unsigned(10 downto 0) := (others => '0');
    signal p14_b, p15_b : unsigned(7 downto 0) := (others => '0');
    signal p14_q  : unsigned(4 downto 0) := (others => '0');
    signal p15_g, p16_r, p16_g, p16_b, p17_r, p17_g, p17_b : unsigned(7 downto 0) := (others => '0');
    signal p18_yr, p18_yg, p18_yb : unsigned(11 downto 0) := (others => '0');
    signal p18_rr, p18_rg, p18_rb, p18_br, p18_bg, p18_bb : unsigned(12 downto 0) := (others => '0');
    signal p19_y  : unsigned(12 downto 0) := (others => '0');
    signal p19_cr, p19_cb : signed(13 downto 0) := (others => '0');
    signal p20_yn : unsigned(9 downto 0) := (others => '0');
    signal p20_u, p20_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p21_yn : unsigned(9 downto 0) := (others => '0');
    signal p21_u, p21_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y22d, s_y22dd : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y23, s_y27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- ROMs and rings
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_bd_q <= C_MB_BAND(to_integer(r_bd_a));
            r_rg_q <= C_MB_RING(to_integer(r_rg_a));
            r_s1_q <= C_MB_SIN(to_integer(r_s1_a));
            r_s2_q <= C_MB_SIN(to_integer(r_s2_a));
            r_s3_q <= C_MB_SIN(to_integer(r_s3_a));
            r_cs_q <= C_MB_COS(to_integer(r_cs_a));
            r_ex_q <= C_MB_EXP2(to_integer(r_ex_a));
        end if;
    end process p_rom;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wp <= r_wp + 1;
            r_oc(to_integer(r_wp)) <= std_logic_vector(s_u22(9 downto 2)) & std_logic_vector(s_v22(9 downto 2));
            r_ocq <= r_oc(to_integer(r_wp - 7));
            r_sy(to_integer(r_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_syq <= r_sy(to_integer(r_wp - 34));
            r_syq2 <= r_syq;
            r_ly(to_integer(r_wp)) <= "000000" & std_logic_vector(s_y23);
            r_lyq <= r_ly(to_integer(r_wp - 5));
            r_uv(to_integer(r_wp)) <= std_logic_vector(s_u24(9 downto 2)) & std_logic_vector(s_v24(9 downto 2));
            r_uvq <= r_uv(to_integer(r_wp - 1));
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock).  The one
    -- serial multiplier also serves the line engine (two products per line,
    -- requested by l_ld / l_ld2; the zone slots end long before the line-0 kick).
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_dd : unsigned(9 downto 0);
        variable v_x  : unsigned(15 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_dr : signed(19 downto 0);
        variable v_ys : unsigned(15 downto 0);
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
                s_twin <= registers_in(6)(0);
                s_inl  <= registers_in(6)(1);
                s_neg  <= registers_in(6)(2);
                s_gun  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_ys   <= s_ysr;
                c_rneg <= s_vneg;
                -- lattice start (the zoom is anchored at the line start) and the SD flag
                if s_xsr = 4 then
                    c_uk <= '1'; c_w0 <= to_unsigned(721126, 20); c_h00 <= to_unsigned(254081, 20); c_v00 <= to_unsigned(432063, 20);
                else
                    c_uk <= '0'; c_w0 <= to_unsigned(454726, 20); c_h00 <= to_unsigned(353375, 20); c_v00 <= to_unsigned(602037, 20);
                end if;
                c_mdx1 <= f_xsm(to_signed(152, 16), s_xsr);
                c_mdx2 <= f_xsm(to_signed(-187, 16), s_xsr);
                c_mdx3 <= f_xsm(to_signed(73, 16), s_xsr);
                c_wxs  <= f_xsm(to_signed(1170, 16), s_xsr);
                c_oxs  <= f_xsm(to_signed(137, 16), s_xsr);
                -- per-line phase steps (Q16) from the line step ys (Q8 virtual lines)
                v_ys := resize(s_ysr, 16);
                c_mdy1 <= signed(shift_right(v_ys, 2) + shift_right(v_ys, 3));            -- 96/256: 683-line melt wave
                c_mdy2 <= signed(shift_right(v_ys, 1));                                    -- 128/256: 512 lines
                c_mdy3 <= -signed(v_ys - shift_right(v_ys, 3));                            -- -224/256: 293 lines
                c_tys  <= shift_right(v_ys, 1);                                            -- figure rows: 512 lines
                c_wys  <= shift_left(v_ys, 2) + shift_left(v_ys, 1);                       -- zigzag: 42.7 lines
                c_oys  <= shift_right(v_ys, 1) + shift_right(v_ys, 3);                     -- Osc: 410 lines
                c_yfs  <= shift_right(v_ys, 2) - shift_right(v_ys, 6);                     -- y / 1080 (Q16)
            end if;
            g_wm1 <= s_w - 1;
            s_half <= '0' & s_w(11 downto 1);

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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
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
                    when 222 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            if l_ld = '1' then                   -- line engine: fm * |cos|
                z_sel <= to_unsigned(13, 4); z_pre <= '1';
            elsif l_ld2 = '1' then               -- line engine: f * 4 (192 + x0)
                z_sel <= to_unsigned(14, 4); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(K_F0, 20);       z_fb <= z_x1;                      -- band f0
                    when 1  => z_fa <= resize(c_f0, 20);            z_fb <= resize(z_mq, 16);          -- f0 * m
                    when 2  => z_fa <= to_unsigned(K_KV, 20);       z_fb <= z_x2;                      -- FM phase step
                    when 3  => z_fa <= resize(z_kvb, 20);           z_fb <= resize(s_ys, 16);          -- per line
                    when 4  => z_fa <= resize(s_vm, 20);            z_fb <= resize(s_vm, 16);          -- speed^2
                    when 5  => z_fa <= resize(z_v2, 20);            z_fb <= to_unsigned(713, 16);      -- roll
                    when 6  => z_fa <= to_unsigned(871, 20);        z_fb <= z_x3;                      -- lattice cH
                    when 7  => z_fa <= to_unsigned(1491, 20);       z_fb <= z_x3;                      -- sV
                    when 8  => z_fa <= to_unsigned(388, 20);        z_fb <= z_x3;                      -- sH
                    when 9  => z_fa <= to_unsigned(3349, 20);       z_fb <= z_x3;                      -- cV
                    when 10 => z_fa <= to_unsigned(1110, 20);       z_fb <= z_x3;                      -- fold
                    when 11 => z_fa <= resize(z_lsh, 20);           z_fb <= resize(s_ys, 16);          -- dH per line
                    when 12 => z_fa <= resize(z_lcv, 20);           z_fb <= resize(s_ys, 16);          -- dV per line
                    when 13 => z_fa <= resize(c_fm, 20);            z_fb <= unsigned(abs(r_cs_q));     -- line: fm |cos|
                    when others =>                                                                      -- line: start phase
                        z_fa <= resize(e_f, 20);
                        if c_uk = '1' then z_fb <= to_unsigned(1728, 16); else z_fb <= to_unsigned(768, 16); end if;
                end case;
                if z_sel = 13 then l_neg <= r_cs_q(15); end if;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 36); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 4 =>                          -- band: e = k1 - 512 (Q8 octaves)
                        v_d := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_d(7 downto 0)); z_h <= v_d(10 downto 8);
                    when 6 | 8 | 10 | 12 =>            -- 2^e in Q13 from the exp2 mantissa
                        case to_integer(z_h) is
                            when -2     => v_x := shift_right(r_ex_q, 4);
                            when -1     => v_x := shift_right(r_ex_q, 3);
                            when 0      => v_x := shift_right(r_ex_q, 2);
                            when others => v_x := shift_right(r_ex_q, 1);
                        end case;
                        case to_integer(s_zc) is
                            when 6 =>
                                z_x1 <= v_x;
                                v_d := signed(resize(s_g2(12 downto 3), 11)) - 512;      -- arcs: e = k2 - 512
                                r_ex_a <= unsigned(v_d(7 downto 0)); z_h <= v_d(10 downto 8);
                            when 8 =>
                                z_x2 <= v_x;
                                v_d := signed(resize(s_g3(12 downto 3), 11)) - 512;      -- lattice: e = 3/4 (k3 - 512)
                                v_e := shift_right(v_d, 1) + shift_right(v_d, 2);
                                r_ex_a <= unsigned(v_e(7 downto 0)); z_h <= v_e(10 downto 8);
                            when 10 =>
                                z_x3 <= v_x;
                                v_d := signed(resize(s_g5(12 downto 3), 11)) - 512;      -- rings: e = 7/8 (k5 - 512)
                                v_e := v_d - shift_right(v_d, 3);
                                r_ex_a <= unsigned(v_e(7 downto 0)); z_h <= v_e(10 downto 8);
                            when others =>
                                c_gr <= v_x(14 downto 5);                                -- rings gain, Q8
                                -- m = p12 * 29 (Q15)
                                z_mq <= resize(shift_left(resize(s_gp(12 downto 3), 15), 5) - shift_left(resize(s_gp(12 downto 3), 15), 1)
                                        - resize(s_gp(12 downto 3), 15), 15);
                                -- figures: q0 = 248 + d/4 - d/32 + d/256 (d = 512 - k6), camera gain k6 / 128
                                v_d := 512 - signed(resize(s_g6(12 downto 3), 11));
                                v_e := 248 + shift_right(v_d, 2) - shift_right(v_d, 5) + shift_right(v_d, 8);
                                c_q0 <= unsigned(v_e(8 downto 0));
                                c_cg <= s_g6(12 downto 10);
                                -- vein: (p12 - 600) / 64, floored at 0
                                if s_gp(12 downto 3) > 600 then v_dd := s_gp(12 downto 3) - 600; c_vg <= v_dd(8 downto 6); else c_vg <= (others => '0'); end if;
                        end case;
                    when 50  => c_f0 <= f_acc(28 downto 13);
                    when 74  => c_fm <= f_acc(29 downto 15);
                    when 98  => z_kvb <= f_acc(28 downto 13);
                    when 122 => c_kv <= f_acc(25 downto 8);
                    when 146 => z_v2 <= f_acc(17 downto 0);
                    when 170 =>                        -- roll per field, signed (+ = the arcs roll up)
                        if c_rneg = '1' then c_rs <= -signed(resize(f_acc(27 downto 10), 19)); else c_rs <= signed(resize(f_acc(27 downto 10), 19)); end if;
                    when 194 | 218 | 242 | 266 | 290 =>      -- lattice constants * 2^e3
                        v_t := f_acc(28 downto 13);
                        case to_integer(s_zc) is
                            when 194 => c_dhp <= unsigned(f_xsm(signed(v_t), s_xs));
                            when 218 => c_dvp <= unsigned(f_xsm(signed(v_t), s_xs));
                            when 242 => z_lsh <= v_t(13 downto 0);
                            when 266 => z_lcv <= v_t(13 downto 0);
                            when others => c_dwp <= unsigned(f_xsm(signed(v_t), s_xs));
                        end case;
                    when 314 => c_dhl <= (not f_acc(27 downto 8)) + 1;
                    when 338 => c_dvl <= f_acc(27 downto 8);
                    when 700 =>                        -- field motion: roll, lattice drift, melt and Osc drifts
                        v_dr := shift_right(resize(c_rs, 20), 5) + shift_right(resize(c_rs, 20), 7);
                        c_pv0 <= unsigned(signed(c_pv0) + resize(c_rs, 24));
                        c_vdr <= unsigned(signed(c_vdr) + v_dr);
                        c_fm1 <= c_fm1 + 142; c_fm2 <= c_fm2 - 120; c_fm3 <= c_fm3 + 76;
                        c_fox <= c_fox + 131; c_foy <= c_foy + 98;
                    when 701 =>
                        c_v0d <= c_v00 - c_vdr;
                    when others => null;
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
    -- Video source: unipolar luma (black = 0, white = 255), tapped at the input
    -- and carried 8 bits wide to pixel stage 4
    ---------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_yv : unsigned(9 downto 0);
        variable v_sm : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_y0 <= unsigned(data_in.y);
            if v_y0 > 64 then v_yv := v_y0 - 64; else v_yv := (others => '0'); end if;
            v_sm := shift_right(v_yv, 2) + shift_right(v_yv, 5) + shift_right(v_yv, 7) + shift_right(v_yv, 9);
            if v_sm > 255 then v_s1 <= to_unsigned(255, 8); else v_s1 <= v_sm(7 downto 0); end if;
            v_s2 <= v_s1; v_s3 <= v_s2; v_s4 <= v_s3; v_s5 <= v_s4;
        end if;
    end process p_vid;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0).
    -- Its accumulators hold the line being computed: the pixel path loads them
    -- directly at the line start (their old values) as they step on.  Two
    -- products on the shared serial multiplier (p_ctrl): f = f0 + fm cos(pv),
    -- then the start phase f * 4 (192 + x0); n_* are ready ~45 clocks in.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y8 : unsigned(7 downto 0);
        variable v_g  : unsigned(14 downto 0);
        variable v_c  : unsigned(13 downto 0);
        variable v_gm : unsigned(13 downto 0);
        variable v_p  : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_pv <= c_pv0; e_hl <= c_h00; e_vl <= c_v0d;
                if c_uk = '1' then         -- SD: the melt phases start at virtual x 240
                    e_m1 <= c_fm1 + 36480; e_m2 <= c_fm2 + 20656; e_m3 <= c_fm3 + 17520;
                else
                    e_m1 <= c_fm1; e_m2 <= c_fm2; e_m3 <= c_fm3;
                end if;
                e_wy <= (others => '0'); e_oy <= c_foy; e_ty <= to_unsigned(C_TY0, 16); e_yf <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_pv <= e_pv + resize(c_kv, 24);
                e_hl <= e_hl + c_dhl; e_vl <= e_vl + c_dvl;
                e_m1 <= unsigned(signed(e_m1) + c_mdy1); e_m2 <= unsigned(signed(e_m2) + c_mdy2); e_m3 <= unsigned(signed(e_m3) + c_mdy3);
                e_wy <= e_wy + c_wys; e_oy <= e_oy + c_oys; e_ty <= e_ty + c_tys;
                e_yf <= e_yf + resize(c_yfs, 17);
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            l_ld <= '0'; l_ld2 <= '0';
            case to_integer(e_lc) is
                when 1 =>
                    r_cs_a <= e_pv(23 downto 14);
                when 2 =>
                    l_ld <= '1';
                when 4 =>
                    -- the warm floor, the figure offset and the melt gain from y8 = y / 1080 (8 bits)
                    if e_yf(16) = '1' then v_y8 := (others => '1'); else v_y8 := e_yf(15 downto 8); end if;
                    v_g := shift_left(resize(v_y8, 15), 6) + shift_left(resize(v_y8, 15), 4) + shift_left(resize(v_y8, 15), 2) + resize(v_y8, 15);   -- 85
                    n_go <= to_unsigned(180, 8) - resize(v_g(14 downto 8), 8);
                    v_c := shift_left(resize(v_y8, 14), 5) + shift_left(resize(v_y8, 14), 4) + shift_left(resize(v_y8, 14), 1) + resize(v_y8, 14);   -- 51
                    n_cl <= signed(resize(v_c(13 downto 8), 10)) - signed(resize(c_q0, 10));
                    v_gm := shift_left(resize(v_y8, 14) + 51, 4) + shift_left(resize(v_y8, 14) + 51, 2) + shift_left(resize(v_y8, 14) + 51, 1);  -- 22
                    n_gm <= v_gm(12 downto 8);
                when 23 =>                         -- f = f0 +- fm |cos| >> 15 (sign-magnitude, one adder)
                    v_p := resize(f_acc(30 downto 15), 17);
                    if l_neg = '1' then v_p := not v_p; end if;
                    e_f <= resize(c_f0, 17) + v_p + unsigned'("" & l_neg);
                when 24 =>
                    l_ld2 <= '1';
                    case to_integer(s_xs) is
                        when 4      => n_su <= shift_left(resize(e_f, 20), 3);
                        when 3      => n_su <= shift_left(resize(e_f, 20), 2) + shift_left(resize(e_f, 20), 1);
                        when others => n_su <= shift_left(resize(e_f, 20), 2);
                    end case;
                when 44 =>
                    n_us <= f_acc(23 downto 0) - to_unsigned(C_UC, 24);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = the accumulators at g_start)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sx : signed(10 downto 0);
        variable v_tw : std_logic;
        variable v_su : unsigned(23 downto 0);
        variable v_dh, v_dv : unsigned(19 downto 0);
        variable v_pr : signed(14 downto 0);
        variable v_pp : unsigned(9 downto 0);
        variable v_tu : unsigned(8 downto 0);
        variable v_p1 : unsigned(15 downto 0);
        variable v_w  : signed(9 downto 0);
        variable v_f  : signed(10 downto 0);
        variable v_af : unsigned(12 downto 0);
        variable v_ln : signed(13 downto 0);
        variable v_gf : signed(12 downto 0);
        variable v_b  : unsigned(13 downto 0);
        variable v_bb : unsigned(13 downto 0);
        variable v_gl : unsigned(14 downto 0);
        variable v_q  : unsigned(9 downto 0);
        variable v_rr : unsigned(12 downto 0);
        variable v_kb, v_kg : unsigned(5 downto 0);
        variable v_k4 : unsigned(4 downto 0);
        variable v_kr : unsigned(4 downto 0);
        variable v_yn : unsigned(10 downto 0);
        variable v_c  : signed(9 downto 0);
        variable v_y8 : unsigned(7 downto 0);
        variable v_m16 : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: band phase (Twin: back down after the centre), fold, lattice pair, melt / zigzag / Osc phases
            if g_start = '1' then
                b_u <= n_us; b_w <= c_w0; b_h <= e_hl; b_v <= e_vl;
                b_m1 <= e_m1; b_m2 <= e_m2; b_m3 <= e_m3;
                if c_uk = '1' then b_wx <= to_unsigned(18656, 16); b_ox <= c_fox + 32880;
                else b_wx <= (others => '0'); b_ox <= c_fox; end if;
                l_su <= n_su; l_go <= n_go; l_cl <= n_cl; l_gm <= n_gm; b_tw <= '0';
                l_ty <= f_tri8(e_ty);
                l_wy <= f_tri8(e_wy); l_oy <= f_tri8(e_oy);
            elsif g_avid = '1' then
                v_tw := s_twin and b_tw;
                v_su := resize(l_su, 24); if v_tw = '1' then v_su := not v_su; end if;
                b_u <= b_u + v_su + unsigned'("" & v_tw);
                if g_cnt = s_half then b_tw <= '1'; end if;
                b_w <= b_w + resize(c_dwp, 20);
                v_dh := resize(c_dhp, 20); v_dv := resize(c_dvp, 20);
                if b_w(19) = '1' then v_dh := not v_dh; v_dv := not v_dv; end if;
                b_h <= b_h + v_dh + unsigned'("" & b_w(19)); b_v <= b_v + v_dv + unsigned'("" & b_w(19));
                b_m1 <= unsigned(signed(b_m1) + c_mdx1); b_m2 <= unsigned(signed(b_m2) + c_mdx2); b_m3 <= unsigned(signed(b_m3) + c_mdx3);
                b_wx <= unsigned(signed(b_wx) + c_wxs); b_ox <= unsigned(signed(b_ox) + c_oxs);
            end if;
            -- 1: triangles, sine addresses, the zigzag term
            p1_a1 <= f_tri10(b_h(19 downto 9)); p1_b1 <= f_tri10(b_v(19 downto 9));
            r_s1_a <= b_m1(15 downto 8); r_s2_a <= b_m2(15 downto 8); r_s3_a <= b_m3(15 downto 8);
            p1_u <= b_u(23 downto 12);
            v_w := signed(resize(f_tri8(b_wx), 10)) + signed(resize(l_wy, 10)) - 256;
            p1_wg <= v_w(9 downto 4);
            p1_ox <= f_tri8(b_ox);
            -- 2: rhombus distance terms (rings: a1 * 3.266 + b1 * 1.211, Q10); the chain term (1 - b1) / 8
            p2_dla <= shift_left(resize(p1_a1, 12), 1) + resize(p1_a1, 12) + resize(p1_a1(9 downto 2), 12) + resize(p1_a1(9 downto 6), 12);
            p2_dlb <= resize(p1_b1, 11) + resize(p1_b1(9 downto 3), 11) + resize(p1_b1(9 downto 4), 11) + resize(p1_b1(9 downto 6), 11) + resize(p1_b1(9 downto 7), 11);
            p2_q <= not p1_b1(9 downto 5);
            p2_u <= p1_u; p2_wg <= p1_wg; p2_ox <= p1_ox;
            -- 3: sine fabric copies; distance sum
            p3_s1 <= r_s1_q; p3_s2 <= r_s2_q; p3_s3 <= r_s3_q;
            p3_dl <= resize(p2_dla, 13) + resize(p2_dlb, 13);
            p3_u <= p2_u; p3_wg <= p2_wg; p3_q <= p2_q; p3_ox <= p2_ox;
            -- 4: melt sum; rings gain product; the source (camera luma or the Osc lattice)
            p4_s3 <= resize(p3_s1, 10) + resize(p3_s2, 10) + resize(p3_s3, 10);
            p4_dh <= p3_dl(12 downto 4) * c_gr(9 downto 3);
            if s_src = '1' then
                v_sx := shift_left(signed(resize(p3_ox, 11)) + signed(resize(l_oy, 11)) - 255, 1);
                if v_sx > 255 then p4_src <= to_signed(255, 9);
                elsif v_sx < -255 then p4_src <= to_signed(-255, 9);
                else p4_src <= v_sx(8 downto 0); end if;
            else
                p4_src <= signed('0' & v_s5);
            end if;
            p4_u <= p3_u; p4_wg <= p3_wg; p4_q <= p3_q;
            -- 5: M = 0.672 * sum (Q8), dl * rings (Q6), source * gain / 4
            p5_m <= shift_right(p4_s3, 1) + shift_right(p4_s3, 3) + shift_right(p4_s3, 5) + shift_right(p4_s3, 6);
            p5_dlg <= p4_dh(14 downto 5);
            v_pr := resize(p4_src * signed('0' & c_cg), 15);
            p5_cs <= v_pr(11 downto 2);
            p5_u <= p4_u; p5_wg <= p4_wg; p5_q <= p4_q;
            -- 6: melt term (x the per-line gain), M total, the source's lattice term, the vein (M x gain)
            p6_mt <= p5_m(9 downto 4) * signed('0' & l_gm);
            p6_mtt <= resize(p5_m, 11) + resize(p5_cs, 11);
            p6_ct <= resize(shift_right(p5_cs, 3) + shift_right(p5_cs, 6) + shift_right(p5_cs, 7), 8);
            p6_vt <= resize(p5_m * signed('0' & c_vg), 13);
            p6_dlg <= p5_dlg; p6_u <= p5_u; p6_wg <= p5_wg; p6_q <= p5_q;
            -- 7: ring index (Q6 rings); the veined band phase; the figure's melt term
            p7_dlr <= signed(resize(p6_dlg, 12)) + resize(p6_mt(11 downto 4), 12) + resize(p6_ct, 12);
            p7_uv <= p6_u + unsigned(p6_vt(11 downto 0));
            p7_mm <= resize(shift_right(p6_mtt, 3) + shift_right(p6_mtt, 6), 8);
            p7_wg <= p6_wg; p7_q <= p6_q;
            -- 8: ROM addresses (ring: black below 0); tu = 1 - tri(u - 0.82) (7 bits); figure partial sum
            if p7_dlr < 0 then r_rg_a <= (others => '0'); else r_rg_a <= unsigned(p7_dlr(9 downto 0)); end if;
            r_bd_a <= p7_uv(11 downto 4);
            v_pp := p7_uv(11 downto 2) - to_unsigned(840, 10);
            if v_pp(9) = '0' then v_tu := not v_pp(8 downto 0); else v_tu := v_pp(8 downto 0); end if;
            p8_tu <= v_tu(8 downto 2);
            p8_f0 <= resize(p7_mm, 9) + signed(resize(p7_q, 9)) + resize(p7_wg, 9);
            -- 9: (ROMs) tu * ty; + the per-line figure offset
            p9_pr <= p8_tu * l_ty;
            p9_f1 <= resize(p8_f0, 10) + l_cl;
            -- 10: R (Negative: 240 - R) and the keys (Inlay swaps cream and warm); the star term x 1.39
            if s_neg = '1' then p10_r <= to_unsigned(240, 8) - r_rg_q; else p10_r <= r_rg_q; end if;
            v_kb := r_bd_q(15 downto 10); v_kg := r_bd_q(9 downto 4);
            if s_inl = '1' then v_kb := not v_kb; v_kg := not v_kg; end if;
            p10_kb <= v_kb; p10_kg <= v_kg; p10_kr <= r_bd_q(3 downto 0);
            v_p1 := resize(p9_pr, 16) + resize(p9_pr(14 downto 2), 16) + resize(p9_pr(14 downto 3), 16);
            p10_p1 <= v_p1(15 downto 7);
            p10_f1 <= p9_f1;
            -- 11: Fg = 7 (star + the rest)
            v_f := signed(resize(p10_p1, 11)) + resize(p10_f1, 11);
            p11_fg <= shift_left(resize(v_f, 13), 3) - resize(v_f, 13);
            p11_r <= p10_r; p11_kb <= p10_kb; p11_kg <= p10_kg; p11_kr <= p10_kr;
            -- 12: the blue line (|Fg| small) and the warm floor (darker inside the figures)
            if p11_fg < 0 then v_af := unsigned(-p11_fg); else v_af := unsigned(p11_fg); end if;
            v_ln := 96 - signed(resize(v_af, 14)) - signed(resize(v_af(12 downto 2), 14));
            if v_ln < 0 then p12_ln <= (others => '0');
            elsif v_ln > 63 then p12_ln <= to_unsigned(63, 6);
            else p12_ln <= unsigned(v_ln(5 downto 0)); end if;
            if p11_fg < 0 then
                p12_gf <= l_go;
            else
                v_gf := signed(resize(shift_right(shift_left(resize(l_go, 10), 1) + resize(l_go, 10), 2), 13))
                        - (shift_right(p11_fg, 2) - shift_right(p11_fg, 6));
                if v_gf < 22 then p12_gf <= to_unsigned(22, 8); else p12_gf <= unsigned(v_gf(7 downto 0)); end if;
            end if;
            p12_r <= p11_r; p12_kb <= p11_kb; p12_kg <= p11_kg; p12_kr <= p11_kr;
            -- 13: G = floor + kg (240 - floor); the line only outside the G window
            v_k4 := resize(p12_kg(5 downto 3), 5) + resize(p12_kg(5 downto 5), 5);
            v_b := resize(v_k4 * (to_unsigned(240, 8) - p12_gf), 14);
            p13_g1 <= p12_gf + resize(v_b(11 downto 3), 8);
            if p12_kg = 0 then p13_ln <= p12_ln;
            elsif p12_kg(5 downto 4) = "00" then p13_ln <= '0' & p12_ln(5 downto 1);
            else p13_ln <= (others => '0'); end if;
            p13_r <= p12_r; p13_kb <= p12_kb; p13_kr <= p12_kr;
            -- 14: B = 197 kb + 210 line; the line's G cut (3 bits) and the R factor (the fringe dip x the line)
            v_b := shift_left(resize(p13_kb, 14), 7) + shift_left(resize(p13_kb, 14), 6) + shift_left(resize(p13_kb, 14), 2) + resize(p13_kb, 14);
            v_bb := shift_left(resize(p13_ln, 14), 7) + shift_left(resize(p13_ln, 14), 6) + shift_left(resize(p13_ln, 14), 4) + shift_left(resize(p13_ln, 14), 1);
            v_gl := resize(v_b(13 downto 6), 15) + resize(v_bb(13 downto 6), 15);
            if v_gl > 255 then p14_b <= to_unsigned(255, 8); else p14_b <= v_gl(7 downto 0); end if;
            p14_gl <= p13_g1 * p13_ln(5 downto 3);
            p14_g1 <= p13_g1;
            v_kr := resize(p13_kr, 5) + resize(p13_kr(3 downto 3), 5);
            if v_kr > resize(p13_ln(5 downto 3), 5) then p14_q <= v_kr - resize(p13_ln(5 downto 3), 5); else p14_q <= (others => '0'); end if;
            p14_r <= p13_r;
            -- 15: G and R after the line
            v_gl := shift_left(resize(p14_gl, 15), 3) + shift_left(resize(p14_gl, 15), 2) + resize(p14_gl, 15);
            p15_g <= p14_g1 - resize(v_gl(14 downto 7), 8);
            v_rr := p14_r * p14_q;
            p15_r <= v_rr(11 downto 4);
            p15_b <= p14_b;
            -- 16: Guns (RGB / GBR); 17: pad (keeps C_LAT a multiple of 4)
            if s_gun = '1' then p16_r <= p15_g; p16_g <= p15_b; p16_b <= p15_r;
            else p16_r <= p15_r; p16_g <= p15_g; p16_b <= p15_b; end if;
            p17_r <= p16_r; p17_g <= p16_g; p17_b <= p16_b;
            -- 18: the matrix partials (constant shift-adds): Y' = (77 R + 150 G + 29 B) >> 8,
            --     Cr = (225 R - 188 G - 36 B) >> 7, Cb = (-76 R - 149 G + 225 B) >> 7
            v_m16 := shift_left(resize(p17_r, 16), 6) + shift_left(resize(p17_r, 16), 4) - shift_left(resize(p17_r, 16), 2) + resize(p17_r, 16);
            p18_yr <= v_m16(15 downto 4);
            v_m16 := shift_left(resize(p17_g, 16), 7) + shift_left(resize(p17_g, 16), 4) + shift_left(resize(p17_g, 16), 2) + shift_left(resize(p17_g, 16), 1);
            p18_yg <= v_m16(15 downto 4);
            v_m16 := shift_left(resize(p17_b, 16), 5) - shift_left(resize(p17_b, 16), 2) + resize(p17_b, 16);
            p18_yb <= v_m16(15 downto 4);
            v_m16 := shift_left(resize(p17_r, 16), 8) - shift_left(resize(p17_r, 16), 5) + resize(p17_r, 16);
            p18_rr <= v_m16(15 downto 3);
            v_m16 := shift_left(resize(p17_g, 16), 7) + shift_left(resize(p17_g, 16), 6) - shift_left(resize(p17_g, 16), 2);
            p18_rg <= v_m16(15 downto 3);
            v_m16 := shift_left(resize(p17_b, 16), 5) + shift_left(resize(p17_b, 16), 2);
            p18_rb <= v_m16(15 downto 3);
            v_m16 := shift_left(resize(p17_r, 16), 6) + shift_left(resize(p17_r, 16), 3) + shift_left(resize(p17_r, 16), 2);
            p18_br <= v_m16(15 downto 3);
            v_m16 := shift_left(resize(p17_g, 16), 7) + shift_left(resize(p17_g, 16), 4) + shift_left(resize(p17_g, 16), 2) + resize(p17_g, 16);
            p18_bg <= v_m16(15 downto 3);
            v_m16 := shift_left(resize(p17_b, 16), 8) - shift_left(resize(p17_b, 16), 5) + resize(p17_b, 16);
            p18_bb <= v_m16(15 downto 3);
            -- 19: sums
            p19_y  <= resize(p18_yr, 13) + resize(p18_yg, 13) + resize(p18_yb, 13);
            p19_cr <= signed(resize(p18_rr, 14)) - signed(resize(p18_rg, 14)) - signed(resize(p18_rb, 14));
            p19_cb <= signed(resize(p18_bb, 14)) - signed(resize(p18_br, 14)) - signed(resize(p18_bg, 14));
            -- 20: luma x4.25 (16/13 prescale) capped at 876; chroma to 10 bits, clamped
            v_y8 := p19_y(11 downto 4);
            v_yn := shift_left(resize(v_y8, 11), 2) + resize(v_y8(7 downto 2), 11);
            if v_yn > 876 then p20_yn <= to_unsigned(876, 10); else p20_yn <= v_yn(9 downto 0); end if;
            v_c := p19_cr(13 downto 4);
            if v_c < -488 then p20_u <= to_unsigned(24, 10); elsif v_c > 488 then p20_u <= to_unsigned(1000, 10); else p20_u <= to_unsigned(512, 10) + unsigned(v_c); end if;
            v_c := p19_cb(13 downto 4);
            if v_c < -488 then p20_v <= to_unsigned(24, 10); elsif v_c > 488 then p20_v <= to_unsigned(1000, 10); else p20_v <= to_unsigned(512, 10) + unsigned(v_c); end if;
            -- 21: + 64
            p21_yn <= p20_yn + 64; p21_u <= p20_u; p21_v <= p20_v;
            -- 22: compose; neutral outside the generated avid
            if s_gav_sr(19) = '0' then
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            else
                s_y22 <= p21_yn; s_u22 <= p21_u; s_v22 <= p21_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(19);     -- the same pixel as s_u22 / s_v22 (the compose gate)
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma through [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums off a block-RAM ring), luma through [1 2 1]/4 delayed to the same
    -- centre (the HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(12 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_sum3 : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to neutral once 8 blank samples are in, so a glitched sample can't
            -- offset the colours for good.  Stage 23 sums, stage 24 output (centred 4 px back).
            if s_bah = "00000000" then
                s_bau <= to_unsigned(128 * 8, 11); s_bav <= to_unsigned(128 * 8, 11);
            else
                s_bau <= s_bau + s_u22(9 downto 2) - unsigned(r_ocq(15 downto 8));
                s_bav <= s_bav + s_v22(9 downto 2) - unsigned(r_ocq(7 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 13) + s_baud; s_u24 <= v_sc(11 downto 4) & "00";
            v_sc := resize(s_bav, 13) + s_bavd; s_v24 <= v_sc(11 downto 4) & "00";

            -- luma [1 2 1]/4 (centred on the stage-22 pixel one clock back) then six delays, so that at the
            -- luma 13/16 stage it sits on the same pixel as the chroma 9-tap (centred 4 px back)
            s_y22d <= s_y22; s_y22dd <= s_y22d;
            v_sum3 := resize(s_y22, 12) + shift_left(resize(s_y22d, 12), 1) + resize(s_y22dd, 12); s_y23 <= v_sum3(11 downto 2);
            -- (the 6-clock luma delay and the 3-clock chroma delay ride two block-RAM rings, p_ring)
            -- 27: luma 13/16
            if unsigned(r_lyq(9 downto 0)) > 64 then v_dl := unsigned(r_lyq(9 downto 0)) - 64; else v_dl := (others => '0'); end if;
            s_y27 <= unsigned(r_lyq(9 downto 0)) - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u27 <= unsigned(r_uvq(15 downto 8)) & "00"; s_v27 <= unsigned(r_uvq(7 downto 0)) & "00";
            -- 28: blanking gate.  The output avid sits where the input avid was (36 clocks on, an even offset
            -- so the hsync parity holds); the filter-centred picture lands exactly on it.
            if s_gav_sr(29) = '1' then
                s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;
            else
                s_y28 <= to_unsigned(64, 10); s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= r_syq2(0);
    data_out.vsync_n <= r_syq2(1);
    data_out.field_n <= r_syq2(2);
    data_out.avid    <= s_gav_sr(30);

end architecture marble;

