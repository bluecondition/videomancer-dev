-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Brushfire v0.1.0 -- Heirloom from the video clips, built from the still WIN_20230820_01_33_09 t0.5
-- (clip WIN_20230820_01_33_09, 9.5 s; the same patch in clips 01_32_42 and 01_36_37).
--
-- The still: two crowns of flame across the top (red-cored lobes, nested white/yellow arches, long orange and red
-- streaks), red-rimmed chevrons stacked down each tile's axis in a pink/purple/brown zone, and under it all
-- S-curved bands (white, black hairline, olive, yellow-green, blue) that sweep to the lower right, crossed by
-- translucent pink diagonals.  The clip breathes: the flames swell down into tall lens shapes and retreat to the
-- top on a 5.8 s cycle while the bands stream right.
--
-- The patch, as fitted (brushfire.py has the tables; .heirloom_work/rb_brushfire/notes.md the measurements), in
-- virtual 1080-line coordinates:
--   tiles  : a_n = |tile phase| folded (0 = axis, 1023 = tile boundary); 2.03 tiles, axes at 540 / 1485 px
--   y'     : y S + O                    the breathing vertical ramp (P12 Blaze + the 5.8 s breath LFO)
--   flame  : F = V(y') - B(y') H(a_n) - G by(y') bx(a_n - c(y')) + camera
--            V = y', B = 1 above the rim; below it V = 300 + 350 E, B = 1 + 0.65 E, E = e^((y' - 300)/350) - 1
--   key    : flame where F < 300, a_n < wall(y') and above the floor (y' 470, tongues over 110 lines)
--   colour : fire palette by (heat, fine phase of F kS); chevrons (F >= 300) in the warm zone a_n < Wz(y');
--            bands (cool / warm palette by the band phase u M(y) + N(y)) elsewhere, pink diagonals over them.
--
--   K1  Tiles     flame tiles across: 0.72 .. 5.75 (2.03 = the photo)
--   K2  Lobes     the lobe bump's strength G = 420 (k/512)^2 px: 0 = pure arches, max = huge red lobes
--   K3  Streaks   fine streak / chevron density x0.4 .. x2.3 (kS)
--   K4  Speed     bipolar (+-8 deadband at 512): breath, streak flow, band drift; 700 = the clip
--   K5  Bands     band density x0.25 .. x4
--   K6  Camera    how far the picture (or the Osc) bends the flames and the bands
--   S7  Bands     Sweep (the photo) / Zigzag (horizontal zigzag bands, clip 01_32_42)
--   S8  Palette   Photo / Ice (U and V swapped: blue flames over orange bands)
--   S9  Flip      Hang (flames from the top) / Rise (flames from the bottom)
--   S10 Flames    Solid / Lines (only the red streaks and rims, the bands showing through)
--   S11 Source    Video / Osc
--   P12 BLAZE     flame height: 0 = a thin crown at the top, 360 = the photo, 100 % = the frame on fire
--
-- Engine: vblank-zone sequencer (shared glide, exp2 knob curves, one serial multiplier), a per-line engine kicked
-- at each line start computing the NEXT line (y', the exp / gaussian / zone tables, the band polynomials, the band
-- start and step), an 18-stage pixel path (two phase DDAs, the arch / lobe tables, four small products, the
-- streak phase, the fire and band palettes, the pink mix), the shell's output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps sit in block-RAM rings, with the per-line resync; luma 13/16)
-- and the blanking gate.  Generated avid; one latency (24) in every mode; colours in the hardware convention.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.brushfire_pkg.all;
use work.all;

architecture brushfire of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_h_a  : unsigned(8 downto 0) := (others => '0');
    signal r_h_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_bx_a : unsigned(8 downto 0) := (others => '1');
    signal r_bx_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_fi_a : unsigned(8 downto 0) := (others => '0');
    signal r_fi_q : std_logic_vector(23 downto 0) := (others => '0');
    signal r_bd_a : unsigned(7 downto 0) := (others => '0');
    signal r_bd_q : std_logic_vector(23 downto 0) := (others => '0');
    signal r_sq_a : unsigned(8 downto 0) := (others => '0');
    signal r_sq_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_ln_a : unsigned(9 downto 0) := (others => '0');
    signal r_ln_q : std_logic_vector(15 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(708 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(360 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_zig, s_ice, s_flip, s_lines, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal f_a    : unsigned(16 downto 0) := (others => '0');
    signal f_b    : unsigned(15 downto 0) := (others => '0');
    signal f_acc  : unsigned(33 downto 0) := (others => '0');
    signal f_cnt  : unsigned(4 downto 0) := (others => '0');
    signal z_d    : signed(11 downto 0) := (others => '0');
    signal z_t1, z_t2 : signed(19 downto 0) := (others => '0');
    signal z_k    : unsigned(19 downto 0) := (others => '0');
    signal c_dT   : unsigned(16 downto 0) := to_unsigned(17756, 17);
    signal c_dph  : unsigned(17 downto 0) := to_unsigned(17756, 18);
    signal c_ph0  : unsigned(23 downto 0) := (others => '0');
    signal c_xo   : unsigned(9 downto 0) := to_unsigned(952, 10);
    signal c_xsv  : unsigned(8 downto 0) := to_unsigned(8, 9);
    signal c_G    : unsigned(10 downto 0) := to_unsigned(420, 11);
    signal c_kSm  : unsigned(4 downto 0) := to_unsigned(30, 5);
    signal c_kSe  : unsigned(1 downto 0) := "10";
    signal c_zsh  : unsigned(1 downto 0) := "10";
    signal c_kb   : unsigned(14 downto 0) := to_unsigned(4096, 15);
    signal c_kv   : unsigned(21 downto 0) := (others => '0');
    signal c_kn   : unsigned(14 downto 0) := (others => '0');
    signal c_kz   : unsigned(20 downto 0) := (others => '0');
    signal c_gF   : unsigned(4 downto 0) := to_unsigned(3, 5);
    signal c_th   : unsigned(15 downto 0) := to_unsigned(C_TH0, 16);
    signal c_fl   : unsigned(15 downto 0) := (others => '0');   -- streak flow, Q8 (Fk units)
    signal c_bfl  : unsigned(15 downto 0) := (others => '0');   -- band flow, 2^16 per cycle
    signal c_wph  : unsigned(15 downto 0) := (others => '0');   -- band wobble phase
    signal c_dth, c_dfl, c_dwp : signed(11 downto 0) := (others => '0');
    signal c_S    : unsigned(15 downto 0) := to_unsigned(16384, 16);   -- Q14
    signal c_O    : signed(11 downto 0) := (others => '0');
    signal z_cs   : signed(16 downto 0) := (others => '0');
    signal z_fms  : unsigned(3 downto 0) := (others => '0');
    signal z_fld  : std_logic := '0';
    signal z_pa, z_ha : unsigned(9 downto 0) := (others => '0');
    signal z_pc   : unsigned(15 downto 0) := (others => '0');
    signal z_hb   : signed(11 downto 0) := (others => '0');
    signal z_d1, z_d4 : signed(11 downto 0) := (others => '0');
    signal z_hp, z_h : signed(11 downto 0) := (others => '0');

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
    signal e_yv, e_yi : unsigned(10 downto 0) := (others => '0');
    signal e_sg   : unsigned(2 downto 0) := (others => '0');
    signal e_dy   : unsigned(8 downto 0) := (others => '0');
    signal e_k0m, e_k0n : signed(15 downto 0) := (others => '0');
    signal e_sm, e_sn : unsigned(12 downto 0) := (others => '0');
    signal e_nm, e_nn : std_logic := '0';
    signal e_wp   : unsigned(15 downto 0) := (others => '0');
    signal e_wb   : signed(15 downto 0) := (others => '0');
    signal e_yp   : signed(12 downto 0) := (others => '0');
    signal e_d    : signed(12 downto 0) := (others => '0');
    signal e_ad, e_dp, e_yf, e_ypp : unsigned(11 downto 0) := (others => '0');
    signal e_ip   : unsigned(2 downto 0) := (others => '0');
    signal e_e    : unsigned(16 downto 0) := (others => '0');
    signal e_by   : unsigned(7 downto 0) := (others => '0');
    signal e_m14  : unsigned(15 downto 0) := (others => '0');
    signal e_n16  : signed(16 downto 0) := (others => '0');
    signal e_mk   : unsigned(21 downto 0) := (others => '0');
    signal e_nk, e_b0 : unsigned(23 downto 0) := (others => '0');
    signal e_nab  : unsigned(13 downto 0) := (others => '0');
    signal e_nko  : unsigned(23 downto 0) := (others => '0');
    signal m_a    : unsigned(21 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(38 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal e_ms   : unsigned(3 downto 0) := (others => '0');
    signal e_ld   : std_logic := '0';
    signal n_v    : signed(15 downto 0) := (others => '0');
    signal n_b    : unsigned(11 downto 0) := to_unsigned(64, 12);
    signal n_gb   : unsigned(7 downto 0) := (others => '0');
    signal n_c    : unsigned(10 downto 0) := (others => '0');
    signal n_wz, n_aw : unsigned(10 downto 0) := (others => '0');
    signal n_dt   : unsigned(6 downto 0) := (others => '0');
    signal n_pko  : std_logic := '0';
    signal n_pk0  : unsigned(7 downto 0) := (others => '0');
    signal n_b0, n_bst : unsigned(23 downto 0) := (others => '0');

    -- beam and the line's values
    signal b_ph, b_bp : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal c_xb0 : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal l_v   : signed(15 downto 0) := (others => '0');
    signal l_b   : unsigned(11 downto 0) := to_unsigned(64, 12);
    signal l_gb  : unsigned(7 downto 0) := (others => '0');
    signal l_c   : unsigned(10 downto 0) := (others => '0');
    signal l_wz, l_aw : unsigned(10 downto 0) := (others => '0');
    signal l_dt  : unsigned(6 downto 0) := (others => '0');
    signal l_pko : std_logic := '0';
    signal l_pk0 : unsigned(7 downto 0) := (others => '0');
    signal l_bst : unsigned(23 downto 0) := (others => '0');
    signal l_yp  : std_logic := '0';

    -- pixel path
    signal p2_an, p3_an, p4_an : unsigned(9 downto 0) := (others => '0');
    signal p2_dc : signed(11 downto 0) := (others => '0');
    signal p2_bp, p3_bp, p4_bp, p5_bp : unsigned(23 downto 0) := (others => '0');
    signal p6_bp : unsigned(13 downto 0) := (others => '0');
    signal p3_pkd : unsigned(10 downto 0) := (others => '0');
    signal p4_cm  : std_logic_vector(7 downto 0) := (others => '0');
    signal p5_h   : unsigned(9 downto 0) := (others => '0');
    signal p5_hh, p6_hh, p7_hh, p8_hh, p9_hh, p10_hh : unsigned(3 downto 0) := (others => '0');
    signal p5_bx  : unsigned(7 downto 0) := (others => '0');
    signal p5_cp, p6_cp, p7_cp : signed(15 downto 0) := (others => '0');
    signal p5_zg  : unsigned(23 downto 0) := (others => '0');
    signal p6_bhl, p6_bhh : unsigned(16 downto 0) := (others => '0');
    signal p6_gbp : unsigned(15 downto 0) := (others => '0');
    signal p7_bh  : unsigned(21 downto 0) := (others => '0');
    signal p7_gb, p8_gb : unsigned(10 downto 0) := (others => '0');
    signal p7_bq, p8_bq, p9_bq, p10_bq, p11_bq, p12_bq : unsigned(6 downto 0) := (others => '0');
    signal p8_f1  : signed(18 downto 0) := (others => '0');
    signal p8_cf  : signed(11 downto 0) := (others => '0');
    signal p9_f   : signed(16 downto 0) := (others => '0');
    signal p9_hb  : signed(11 downto 0) := (others => '0');
    signal p10_fk : signed(22 downto 0) := (others => '0');
    signal p10_hb : signed(11 downto 0) := (others => '0');
    signal p10_lt, p10_rim : std_logic := '0';
    signal p10_g  : unsigned(5 downto 0) := (others => '0');
    signal p11_fk : unsigned(7 downto 0) := (others => '0');
    signal p11_heat : unsigned(4 downto 0) := (others => '0');
    signal p11_lt, p11_rim : std_logic := '0';
    signal p11_g  : unsigned(5 downto 0) := (others => '0');
    signal p12_fl, p12_ch, p12_rim, p12_wm, p12_pk : std_logic := '0';
    signal p12_pg : unsigned(2 downto 0) := (others => '0');
    signal p12_fk : unsigned(7 downto 0) := (others => '0');
    signal p12_g  : unsigned(5 downto 0) := (others => '0');
    signal p13_use, p13_pk, p14_use, p14_pk, p15_use, p15_pk, p16_pk : std_logic := '0';
    signal p15_fi, p15_bd : std_logic_vector(23 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_v : unsigned(7 downto 0) := (others => '0');
    signal p17_y, p17_u, p17_v : unsigned(7 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_k is array (2 to 12) of std_logic;
    signal q_xp : t_k := (others => '0');
    type t_q is array (3 to 11) of std_logic;
    signal q_inw, q_inz, q_pk : t_q := (others => '0');

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

    -- the pink overlay colour (hardware 8-bit Y, U = Cr, V = Cb) of RGB (236, 112, 168)
    constant C_PY : unsigned(7 downto 0) := to_unsigned(PINK_Y8, 8);
    constant C_PU : unsigned(7 downto 0) := to_unsigned(PINK_U8, 8);
    constant C_PV : unsigned(7 downto 0) := to_unsigned(PINK_V8, 8);

begin

    ---------------------------------------------------------------------------
    -- Memories: line buffer (collisions possible: read-before-write kept),
    -- ROMs, the box rings, the sync ring
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
            r_h_q  <= C_BF_H(to_integer(r_h_a));
            r_bx_q <= C_BF_BX(to_integer(r_bx_a));
            r_fi_q <= C_BF_FIRE(to_integer(r_fi_a));
            r_bd_q <= C_BF_BAND(to_integer(r_bd_a));
            r_sq_q <= C_BF_SEQ(to_integer(r_sq_a));
            r_ln_q <= C_BF_LN(to_integer(r_ln_a));
        end if;
    end process p_rom;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y19) & std_logic_vector(s_u19(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u19(3 downto 0)) & std_logic_vector(s_v19) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y21) & std_logic_vector(s_u21(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u21(3 downto 0)) & std_logic_vector(s_v21) & "00";
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_e  : signed(13 downto 0);
        variable v_m  : unsigned(15 downto 0);
        variable v_q  : unsigned(19 downto 0);
        variable v_h  : signed(12 downto 0);
        variable v_w  : signed(21 downto 0);
        variable v_fs : unsigned(18 downto 0);
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
                s_zig   <= registers_in(6)(0);
                s_ice   <= registers_in(6)(1);
                s_flip  <= registers_in(6)(2);
                s_lines <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
            c_xsv <= c_xb0(9 downto 1);
            c_xo  <= to_unsigned(960, 10) - resize(c_xb0(9 downto 1), 10);

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

            -- serial multiplier, right-shifting accumulator: f_a (17 bits) x f_b (16 bits), 16 iterations; the
            -- product is complete 17 clocks after the load.  Loads use a job code registered one clock ahead
            -- (z_fms / z_fld), so no zone-counter decode sits in front of the operand mux.
            if f_cnt /= 0 then
                if f_b(0) = '1' then v_fs := ('0' & f_acc(33 downto 16)) + resize(f_a, 19);
                else v_fs := '0' & f_acc(33 downto 16); end if;
                f_acc <= v_fs & f_acc(15 downto 1);
                f_b <= '0' & f_b(15 downto 1);
                f_cnt <= f_cnt - 1;
            end if;
            if z_fld = '1' then
                f_acc <= (others => '0'); f_cnt <= to_unsigned(16, 5);
                case to_integer(z_fms) is
                    when 0      => f_a <= resize(unsigned(r_sq_q), 17); f_b <= to_unsigned(17756, 16);
                    when 1      => f_a <= c_dT;                           f_b <= resize(c_xo, 16);
                    when 2      => f_a <= resize(s_g2(12 downto 3), 17);  f_b <= resize(s_g2(12 downto 3), 16);
                    when 3      => f_a <= resize(f_acc(19 downto 8), 17); f_b <= to_unsigned(420, 16);
                    when 4      => f_a <= resize(unsigned(r_sq_q), 17); f_b <= to_unsigned(122, 16);
                    when 5      => f_a <= resize(c_kb, 17);               f_b <= to_unsigned(1075, 16);
                    when 6      => f_a <= resize(c_kb, 17);               f_b <= to_unsigned(437, 16);
                    when 7      => f_a <= resize(s_g6(12 downto 3), 17);  f_b <= resize(s_g6(12 downto 3), 16);
                    when 8      => f_a <= resize(z_pa, 17);               f_b <= z_pc;
                    when 9      => f_a <= resize(z_ha, 17);               f_b <= to_unsigned(196, 16);
                    when others => f_a <= resize(z_ha, 17);               f_b <= to_unsigned(109, 16);
                end case;
            end if;
            z_fld <= '0';

            -- per-field values: zc 30 .. 240 (one multiplier job at a time)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>                                        -- tiles: 17756 * 2^(0.75 (k1 - 512) / 256)
                        v_d := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        v_e := resize(shift_right(shift_left(resize(v_d, 14), 1) + resize(v_d, 14), 2), 14);
                        r_sq_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 31 => z_fms <= to_unsigned(0, 4); z_fld <= '1';      -- job 0 at 32
                    when 49 =>
                        v_q := resize(f_acc(33 downto 15), 20);
                        case to_integer(z_sh) is
                            when -2     => c_dT <= resize(v_q(19 downto 2), 17);
                            when -1     => c_dT <= resize(v_q(19 downto 1), 17);
                            when 0      => c_dT <= resize(v_q, 17);
                            when others => c_dT <= resize(v_q(18 downto 0) & '0', 17);
                        end case;
                        z_fms <= to_unsigned(1, 4); z_fld <= '1';             -- job 1 at 50: dT (960 - x0)
                    when 50 =>                                        -- per-pixel tile step dT * xs / 2
                        case to_integer(s_xs) is
                            when 4      => c_dph <= c_dT & '0';
                            when 3      => c_dph <= resize(c_dT, 18) + resize(c_dT(16 downto 1), 18);
                            when others => c_dph <= resize(c_dT, 18);
                        end case;
                    when 67 =>                                        -- line-start phase: 0.4445 - (960 - x0) dT
                        c_ph0 <= to_unsigned(7457767, 24) - f_acc(23 downto 0);
                        z_fms <= to_unsigned(2, 4); z_fld <= '1';             -- job 2 at 68: k2^2
                    when 84 => z_fms <= to_unsigned(3, 4); z_fld <= '1';      -- job 3 at 85: (k2^2 >> 8) 420
                    when 102 =>
                        c_G <= f_acc(20 downto 10);
                    when 103 =>                                       -- streaks: kS = 122 * 2^(0.8125 (k3 - 512) / 256)
                        v_d := signed(resize(s_g3(12 downto 3), 12)) - 512;
                        v_e := resize(v_d, 14) - resize(shift_right(v_d, 3), 14) - resize(shift_right(v_d, 4), 14);
                        r_sq_a <= '0' & unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 104 => z_fms <= to_unsigned(4, 4); z_fld <= '1';     -- job 4 at 105: 122 * rom
                    when 122 =>
                        v_q := resize(f_acc(33 downto 15), 20);
                        case to_integer(z_sh) is
                            when -2     => z_k <= resize(v_q(19 downto 2), 20);
                            when -1     => z_k <= resize(v_q(19 downto 1), 20);
                            when 0      => z_k <= v_q;
                            when others => z_k <= v_q(18 downto 0) & '0';
                        end case;
                    when 123 =>                                       -- kS as a 5-bit mantissa (16..31) << e, e = 1..4
                        if z_k(8) = '1' then c_kSm <= z_k(8 downto 4); c_kSe <= "11";
                        elsif z_k(7) = '1' then c_kSm <= z_k(7 downto 3); c_kSe <= "10";
                        elsif z_k(6) = '1' then c_kSm <= z_k(6 downto 2); c_kSe <= "01";
                        else c_kSm <= z_k(5 downto 1); c_kSe <= "00"; end if;
                    when 124 =>                                       -- bands: kb = 4096 * 2^((k5 - 512) / 256)
                        v_d := signed(resize(s_g5(12 downto 3), 12)) - 512;
                        r_sq_a <= '0' & unsigned(v_d(7 downto 0)); z_sh <= v_d(10 downto 8);
                    when 126 =>
                        v_m := unsigned(r_sq_q);
                        case to_integer(z_sh) is
                            when -2     => c_kb <= resize(v_m(15 downto 5), 15);
                            when -1     => c_kb <= resize(v_m(15 downto 4), 15);
                            when 0      => c_kb <= resize(v_m(15 downto 3), 15);
                            when others => c_kb <= resize(v_m(15 downto 2), 15);
                        end case;
                        c_zsh <= unsigned(z_sh(1 downto 0)) + 2;      -- the zigzag amplitude octave: 0..3
                        z_fms <= to_unsigned(5, 4); z_fld <= '1';             -- job 5 at 127: kb 1075 (kv)
                    when 127 =>
                        c_kn <= c_kb - resize(c_kb(14 downto 6), 15);
                    when 131 =>                                       -- speed: r = k4 - 520 (or k4 - 504), +-8 deadband
                        v_d := signed(resize(s_g4(12 downto 3), 12));
                        if v_d > 520 then z_d <= v_d - 520;
                        elsif v_d < 504 then z_d <= v_d - 504;
                        else z_d <= (others => '0'); end if;
                    when 132 =>
                        z_d1 <= shift_right(z_d, 1); z_d4 <= shift_right(z_d, 4);
                    when 133 =>                                       -- rates: breath r, flow 1.5 r, bands r, wobble 0.5625 r
                        c_dth <= z_d;
                        c_dfl <= z_d + z_d1;
                        c_dwp <= z_d1 + z_d4;
                    when 134 =>
                        c_th  <= unsigned(signed(c_th) + resize(c_dth, 16));
                        c_fl  <= unsigned(signed(c_fl) + resize(c_dfl, 16));
                        c_bfl <= unsigned(signed(c_bfl) + resize(z_d, 16));
                        c_wph <= unsigned(signed(c_wph) + resize(c_dwp, 16));
                    when 135 =>                                       -- cos(th) = sin(th + 1/4) from the quarter sine
                        v_m := c_th + 16384;
                        if v_m(14) = '1' then r_sq_a <= '1' & not v_m(13 downto 6);
                        else r_sq_a <= '1' & v_m(13 downto 6); end if;
                        z_t1(0) <= v_m(15);
                    when 137 =>
                        if z_t1(0) = '1' then z_cs <= -signed(resize(unsigned(r_sq_q), 17));
                        else z_cs <= signed(resize(unsigned(r_sq_q), 17)); end if;
                    when 144 =>
                        c_kv <= resize(f_acc(32 downto 3), 22);
                        z_fms <= to_unsigned(6, 4); z_fld <= '1';             -- job 6 at 145: kb 437 (kz)
                    when 162 =>
                        c_kz <= resize(f_acc(32 downto 3), 21);
                        z_fms <= to_unsigned(7, 4); z_fld <= '1';             -- job 7 at 163: k6^2
                    when 178 =>                                       -- blaze: |p - 360| and the slope (168 above, 182 below)
                        v_d := signed(resize(s_gp(12 downto 3), 12));
                        if v_d >= 360 then
                            z_pa <= unsigned(resize(v_d - 360, 10)); z_pc <= to_unsigned(168, 16); z_t1(1) <= '0';
                        else
                            z_pa <= unsigned(resize(360 - v_d, 10)); z_pc <= to_unsigned(182, 16); z_t1(1) <= '1';
                        end if;
                    when 180 =>
                        c_gF <= f_acc(19 downto 15);
                        z_fms <= to_unsigned(8, 4); z_fld <= '1';             -- job 8 at 181: |p - 360| slope
                    when 198 =>
                        if z_t1(1) = '1' then z_hp <= -signed(resize(f_acc(19 downto 8), 12));
                        else z_hp <= signed(resize(f_acc(19 downto 8), 12)); end if;
                        z_hb <= resize(shift_right(to_signed(C_COS0, 17) - z_cs, 8), 12);
                    when 199 =>                                       -- h = (w - w0) 256 + hp, (cos0 - cos) / 256
                        v_h := resize(z_hb, 13) + resize(z_hp, 13);
                        if v_h > 512 then z_h <= to_signed(512, 12);
                        elsif v_h < -307 then z_h <= to_signed(-307, 12);
                        else z_h <= v_h(11 downto 0); end if;
                    when 200 =>
                        if z_h < 0 then z_ha <= unsigned(resize(-z_h, 10)); else z_ha <= unsigned(resize(z_h, 10)); end if;
                        z_fms <= to_unsigned(9, 4); z_fld <= '1';             -- job 9 at 201: |h| 196
                    when 218 =>                                       -- e = -sign(h) (|h| 196 >> 8); S = 2^(e / 256)
                        if z_h < 0 then v_w := signed(resize(f_acc(19 downto 8), 22));
                        else v_w := -signed(resize(f_acc(19 downto 8), 22)); end if;
                        r_sq_a <= '0' & unsigned(v_w(7 downto 0)); z_sh <= v_w(10 downto 8);
                        z_fms <= to_unsigned(10, 4); z_fld <= '1';            -- job 10 at 219: |h| 109
                    when 220 =>
                        v_m := unsigned(r_sq_q);
                        case to_integer(z_sh) is
                            when -2     => c_S <= "000" & v_m(15 downto 3);
                            when -1     => c_S <= "00" & v_m(15 downto 2);
                            when others => c_S <= '0' & v_m(15 downto 1);
                        end case;
                    when 236 =>                                       -- O = -sign(h) (|h| 109 >> 8)
                        if z_h < 0 then c_O <= signed(resize(f_acc(19 downto 8), 12));
                        else c_O <= -signed(resize(f_acc(19 downto 8), 12)); end if;
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
    -- 0), computes the NEXT line.  y' = y S + O (Flip: y = 1079 - y), the
    -- rim / exp tables (V, B), the lobe gaussian by and centre c, the warm
    -- zone and wall half-widths, the floor and the pink diagonal's line term;
    -- the bands' M(y), N(y) (piecewise linear + wobble) -> the band phase's
    -- line start and step.  One serial product at a time (complete at +17).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t  : unsigned(12 downto 0);
        variable v_x  : unsigned(24 downto 0);
        variable v_s  : signed(16 downto 0);
        variable v_y  : signed(13 downto 0);
        variable v_ms : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            -- serial multiplier, right-shifting accumulator: m_a (22 bits) x m_b (16 bits); complete 17 clocks after the load
            if m_cnt /= 0 then
                if m_b(0) = '1' then v_ms := ('0' & m_acc(38 downto 16)) + resize(m_a, 24);
                else v_ms := '0' & m_acc(38 downto 16); end if;
                m_acc <= v_ms & m_acc(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                m_cnt <= m_cnt - 1;
            end if;

            if e_ld = '1' then
                m_acc <= (others => '0'); m_cnt <= to_unsigned(16, 5);
                case to_integer(e_ms) is
                    when 0      => m_a <= resize(e_yv, 22);  m_b <= to_unsigned(152, 16);
                    when 1      => m_a <= resize(e_yi, 22);  m_b <= c_S;
                    when 2      => m_a <= resize(e_sm, 22);  m_b <= resize(e_dy, 16);
                    when 3      => m_a <= resize(e_sn, 22);  m_b <= resize(e_dy, 16);
                    when 4      => m_a <= c_kv;              m_b <= e_m14;
                    when 5      => m_a <= resize(c_kn, 22);  m_b <= resize(e_nab, 16);
                    when 6      => m_a <= e_mk;              m_b <= resize(c_xsv, 16);
                    when 7      => m_a <= resize(c_kz, 22);  m_b <= resize(e_yv, 16);
                    when 8      => m_a <= resize(e_dp, 22);  m_b <= to_unsigned(270, 16);
                    when 9      => m_a <= resize(e_e, 22);   m_b <= to_unsigned(350, 16);
                    when 10     => m_a <= resize(e_e, 22);   m_b <= to_unsigned(166, 16);
                    when 11     => m_a <= resize(c_G, 22);   m_b <= resize(e_by, 16);
                    when 12     => m_a <= resize(e_ad, 22);  m_b <= to_unsigned(61, 16);
                    when 13     => m_a <= resize(e_yf, 22);  m_b <= to_unsigned(149, 16);
                    when others => m_a <= resize(e_ypp, 22); m_b <= to_unsigned(312, 16);
                end case;
            end if;
            e_ld <= '0';

            case to_integer(e_lc) is
                when 19 => e_ms <= to_unsigned(1, 4); e_ld <= '1';
                when 36 => e_ms <= to_unsigned(2, 4); e_ld <= '1';
                when 53 => e_ms <= to_unsigned(3, 4); e_ld <= '1';
                when 70 => e_ms <= to_unsigned(4, 4); e_ld <= '1';
                when 87 => e_ms <= to_unsigned(5, 4); e_ld <= '1';
                when 104 => e_ms <= to_unsigned(6, 4); e_ld <= '1';
                when 121 => e_ms <= to_unsigned(7, 4); e_ld <= '1';
                when 138 => e_ms <= to_unsigned(8, 4); e_ld <= '1';
                when 175 => e_ms <= to_unsigned(10, 4); e_ld <= '1';
                when 192 => e_ms <= to_unsigned(11, 4); e_ld <= '1';
                when 209 => e_ms <= to_unsigned(12, 4); e_ld <= '1';
                when 226 => e_ms <= to_unsigned(13, 4); e_ld <= '1';
                when 243 => e_ms <= to_unsigned(14, 4); e_ld <= '1';
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_yv <= to_unsigned(1079, 11); else e_yv <= e_vy(18 downto 8); end if;
                when 2 =>
                    e_ms <= to_unsigned(0, 4); e_ld <= '1';
                    if s_flip = '1' then e_yi <= 1079 - e_yv; else e_yi <= e_yv; end if;
                    if e_yv < 200 then e_sg <= "000";
                    elsif e_yv < 400 then e_sg <= "001";
                    elsif e_yv < 700 then e_sg <= "010";
                    elsif e_yv < 900 then e_sg <= "011";
                    else e_sg <= "100"; end if;
                when 3 =>                                             -- job: y 152 (the band wobble, 2.5 cycles per frame)
                    -- the bands' piecewise-linear M (Q14) and N - 0.112 (Q16): knot value, |slope| (Q8 per line), sign
                    case to_integer(e_sg) is
                        when 0 =>
                            e_dy <= resize(e_yv, 9);
                            e_k0m <= to_signed(16384, 16); e_sm <= to_unsigned(7932, 13); e_nm <= '1';
                            e_k0n <= to_signed(-7340, 16); e_sn <= to_unsigned(5612, 13); e_nn <= '0';
                        when 1 =>
                            e_dy <= resize(e_yv - 200, 9);
                            e_k0m <= to_signed(10187, 16); e_sm <= to_unsigned(2120, 13); e_nm <= '1';
                            e_k0n <= to_signed(-2956, 16); e_sn <= to_unsigned(1778, 13); e_nn <= '0';
                        when 2 =>
                            e_dy <= resize(e_yv - 400, 9);
                            e_k0m <= to_signed(8531, 16); e_sm <= to_unsigned(643, 13); e_nm <= '0';
                            e_k0n <= to_signed(-1567, 16); e_sn <= to_unsigned(410, 13); e_nn <= '1';
                        when 3 =>
                            e_dy <= resize(e_yv - 700, 9);
                            e_k0m <= to_signed(9285, 16); e_sm <= to_unsigned(1096, 13); e_nm <= '1';
                            e_k0n <= to_signed(-2048, 16); e_sn <= to_unsigned(9, 13); e_nn <= '0';
                        when others =>
                            e_dy <= resize(e_yv - 900, 9);
                            e_k0m <= to_signed(8429, 16); e_sm <= to_unsigned(5743, 13); e_nm <= '1';
                            e_k0n <= to_signed(-2041, 16); e_sn <= to_unsigned(2250, 13); e_nn <= '0';
                    end case;
                when 20 =>                                            -- wobble phase; job: y S (y')
                    e_wp <= m_acc(15 downto 0) - c_wph;
                when 21 =>                                            -- wobble: quarter sine
                    if e_wp(14) = '1' then r_ln_a <= "01" & not e_wp(13 downto 6);
                    else r_ln_a <= "01" & e_wp(13 downto 6); end if;
                when 23 =>                                            -- 0.0117 sin in Q16 = (s >> 6) + (s >> 7)
                    v_s := signed(resize(shift_right(unsigned(r_ln_q), 6), 17)) + signed(resize(shift_right(unsigned(r_ln_q), 7), 17));
                    if e_wp(15) = '1' then e_wb <= -v_s(15 downto 0); else e_wb <= v_s(15 downto 0); end if;
                when 37 =>                                            -- y' = y S >> 14 + O; job: |sM| dy
                    v_y := signed(resize(m_acc(26 downto 14), 14)) + resize(c_O, 14);
                    if v_y > 4095 then e_yp <= to_signed(4095, 13);
                    elsif v_y < -4096 then e_yp <= to_signed(-4096, 13);
                    else e_yp <= v_y(12 downto 0); end if;
                when 38 =>
                    e_d <= e_yp - 300;
                    v_y := resize(e_yp, 14) + 512;                    -- zone table row (y' + 512) / 8
                    if v_y < 0 then r_ln_a <= "1000000000";
                    elsif v_y > 2047 then r_ln_a <= "1011111111";
                    else r_ln_a <= "10" & unsigned(v_y(10 downto 3)); end if;
                    if e_yp > 470 then e_yf <= unsigned(resize(e_yp - 470, 12)); else e_yf <= (others => '0'); end if;
                    if e_yp > 0 then e_ypp <= unsigned(e_yp(11 downto 0)); else e_ypp <= (others => '0'); end if;
                    if e_yp > 560 then n_pko <= '1'; else n_pko <= '0'; end if;
                when 39 =>
                    if e_d < 0 then e_ad <= unsigned(resize(-e_d, 12)); e_dp <= (others => '0');
                    else e_ad <= unsigned(resize(e_d, 12)); e_dp <= unsigned(resize(e_d, 12)); end if;
                when 40 =>                                            -- warm zone, wall; the lobe gaussian address
                    n_wz <= resize(unsigned(r_ln_q(15 downto 8)) & "00", 11);
                    n_aw <= resize(unsigned(r_ln_q(7 downto 0)) & "00", 11);
                    if e_yp < -512 then n_aw <= to_unsigned(1024, 11); end if;
                    r_ln_a <= "11" & e_ad(8 downto 1);
                when 42 =>
                    if e_ad(11 downto 1) < 256 then e_by <= unsigned(r_ln_q(7 downto 0)); else e_by <= (others => '0'); end if;
                when 54 =>                                            -- M14; job: |sN| dy
                    if e_nm = '1' then v_s := resize(e_k0m, 17) - signed(resize(m_acc(23 downto 8), 17));
                    else v_s := resize(e_k0m, 17) + signed(resize(m_acc(23 downto 8), 17)); end if;
                    e_m14 <= unsigned(v_s(15 downto 0));
                when 71 =>                                            -- N16 (- 0.112, + the wobble); job: kv M14 (Mk)
                    if e_nn = '1' then v_s := resize(e_k0n, 17) - signed(resize(m_acc(23 downto 8), 17));
                    else v_s := resize(e_k0n, 17) + signed(resize(m_acc(23 downto 8), 17)); end if;
                    e_n16 <= v_s + resize(e_wb, 17);
                when 72 =>
                    v_s := e_n16 + 8192;
                    e_nab <= unsigned(v_s(13 downto 0));
                    e_nko <= shift_left(resize(c_kn, 24), 13);
                when 88 =>                                            -- Mk (band phase Q24 per virtual px); job: kn |N16| (Nk)
                    e_mk <= resize(m_acc(35 downto 16), 22);
                when 105 =>                                           -- Nk; job: x0 Mk
                    e_nk <= m_acc(23 downto 0) - e_nko;
                when 122 =>                                           -- the Sweep line start and step; job: kz y (Zigzag rows)
                    e_b0 <= e_nk + m_acc(23 downto 0) - shift_left(resize(c_bfl, 24), 8);   -- flow: bands stream right
                    if s_zig = '1' then n_bst <= (others => '0');
                    else
                        case to_integer(s_xs) is
                            when 4      => n_bst <= resize(shift_left(e_mk, 1), 24);
                            when 3      => n_bst <= resize(e_mk, 24) + resize(e_mk(21 downto 1), 24);
                            when others => n_bst <= resize(e_mk, 24);
                        end case;
                    end if;
                when 139 =>                                           -- the band line start; job: max(d, 0) 270 (exp table position)
                    if s_zig = '1' then n_b0 <= m_acc(23 downto 0) - shift_left(resize(c_bfl, 24), 8);
                    else n_b0 <= e_b0; end if;
                when 156 =>                                           -- t = d 1.0547 (Q8 octaves), clamped; exp2 read
                    if m_acc(20 downto 8) > 1791 then v_t := to_unsigned(1791, 13); else v_t := m_acc(20 downto 8); end if;
                    r_ln_a <= "00" & v_t(7 downto 0); e_ip <= v_t(10 downto 8);
                when 158 =>                                           -- E = (2^(t/256) - 1) Q8; job: E 350 (V)
                    e_ms <= to_unsigned(9, 4); e_ld <= '1';
                    v_x := shift_left(resize(unsigned(r_ln_q), 25), to_integer(e_ip));
                    e_e <= resize(v_x(23 downto 7), 17) - 256;
                when 176 =>                                           -- V; job: E 166 (B)
                    if e_d < 0 then n_v <= resize(e_yp, 16);
                    elsif m_acc(24 downto 8) > 16083 then n_v <= to_signed(16383, 16);
                    else n_v <= signed(resize(m_acc(24 downto 8), 16)) + 300; end if;
                when 193 =>                                           -- B (Q6); job: G by (the line's lobe gain)
                    if e_d < 0 then n_b <= to_unsigned(64, 12);
                    elsif m_acc(24 downto 10) > 4031 then n_b <= to_unsigned(4095, 12);
                    else n_b <= resize(m_acc(21 downto 10), 12) + 64; end if;
                when 210 =>                                           -- G by >> 11 (8 bits); job: |d| 61 (lobe centre)
                    n_gb <= m_acc(18 downto 11);
                when 227 =>                                           -- c = 580 +- |d| 61 / 128; job: max(y' - 470, 0) 149 (floor)
                    if e_d < 0 then v_y := 580 - signed(resize(m_acc(18 downto 7), 14));
                    else v_y := 580 + signed(resize(m_acc(18 downto 7), 14)); end if;
                    if v_y < 0 then n_c <= (others => '0'); elsif v_y > 2047 then n_c <= (others => '1'); else n_c <= unsigned(v_y(10 downto 0)); end if;
                when 244 =>                                           -- floor threshold; job: max(y', 0) 312 (pink diagonal)
                    if m_acc(19 downto 8) > 64 then n_dt <= to_unsigned(64, 7); else n_dt <= m_acc(14 downto 8); end if;
                when 261 =>
                    n_pk0 <= m_acc(15 downto 8);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): the tile phase and band phase DDAs; the line's values
    -- latched at the line start; the camera read issued from the pixel index
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' then
                l_yp <= '1';
            elsif g_start = '1' then
                b_ph  <= c_ph0;
                b_bp  <= n_b0;
                b_px  <= (others => '0');
                l_v   <= n_v;   l_b  <= n_b;  l_gb <= n_gb; l_c <= n_c;
                l_wz  <= n_wz;  l_aw <= n_aw; l_dt <= n_dt;
                l_pko <= n_pko; l_pk0 <= n_pk0;
                l_bst <= n_bst;
                l_yp  <= not l_yp;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_ph <= b_ph + resize(c_dph, 24);
                b_bp <= b_bp + l_bst;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u  : unsigned(10 downto 0);
        variable v_a  : unsigned(9 downto 0);
        variable v_dc : signed(11 downto 0);
        variable v_cm : signed(8 downto 0);
        variable v_cp : signed(14 downto 0);
        variable v_f  : signed(19 downto 0);
        variable v_k  : unsigned(15 downto 0);
        variable v_hb : signed(17 downto 0);
        variable v_ht : unsigned(5 downto 0);
        variable v_p6 : unsigned(5 downto 0);
        variable v_fl, v_ch, v_wm : std_logic;
        variable v_y, v_u8, v_v8 : unsigned(8 downto 0);
        variable v_bq : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: fold the tile phase: a_n = 0 on the axis, 1023 at the tile boundary; distance to the lobe centre
            v_u := b_ph(23 downto 13);
            if v_u(10) = '0' then v_a := v_u(9 downto 0); else v_a := not v_u(9 downto 0); end if;
            p2_an <= v_a;
            p2_dc <= signed('0' & resize(v_a, 11)) - signed('0' & l_c);
            p2_bp <= b_bp;
            q_xp(2) <= b_px(0);
            for i in 3 to 12 loop q_xp(i) <= q_xp(i - 1); end loop;
            -- 3: ROM addresses (arch ramp by a_n, lobe bump by |a_n - c|); wall / warm-zone tests; 0.906 a_n
            r_h_a <= p2_an(9 downto 1);
            v_dc := abs(p2_dc);
            if v_dc > 511 then r_bx_a <= (others => '1'); else r_bx_a <= unsigned(v_dc(8 downto 0)); end if;
            if resize(p2_an, 11) < l_aw then q_inw(3) <= '1'; else q_inw(3) <= '0'; end if;
            if resize(p2_an, 11) < l_wz then q_inz(3) <= '1'; else q_inz(3) <= '0'; end if;
            p3_pkd <= resize(p2_an, 11) - resize(p2_an(9 downto 3), 11) + resize(p2_an(9 downto 5), 11);
            p3_an <= p2_an; p3_bp <= p2_bp;
            -- 4: ROM data in the EBR output registers; the camera luma; the pink diagonal's phase test
            p4_cm <= s_lb_q;
            if l_pko = '1' and (l_pk0 - p3_pkd(7 downto 0)) < 56 then q_pk(4) <= '1'; else q_pk(4) <= '0'; end if;
            q_inw(4) <= q_inw(3); q_inz(4) <= q_inz(3);
            p4_an <= p3_an; p4_bp <= p3_bp;
            -- 5: fabric copies; the camera product (Osc: zero-mean and x4); the zigzag term a_n << (12 + octave)
            p5_h  <= unsigned(r_h_q(15 downto 6));
            p5_hh <= unsigned(r_h_q(3 downto 0));
            p5_bx <= unsigned(r_bx_q);
            if s_src = '1' then v_cm := signed('0' & unsigned(p4_cm)) - 120; else v_cm := signed('0' & unsigned(p4_cm)); end if;
            v_cp := v_cm * signed('0' & c_gF);
            if s_src = '1' then p5_cp <= shift_left(resize(v_cp, 16), 2); else p5_cp <= resize(v_cp, 16); end if;
            if s_zig = '1' then
                case to_integer(c_zsh) is
                    when 0      => p5_zg <= shift_left(resize(p4_an, 24), 12);
                    when 1      => p5_zg <= shift_left(resize(p4_an, 24), 13);
                    when 2      => p5_zg <= shift_left(resize(p4_an, 24), 14);
                    when others => p5_zg <= shift_left(resize(p4_an, 24), 15);
                end case;
            else
                p5_zg <= (others => '0');
            end if;
            p5_bp <= p4_bp;
            -- 6: B H split in two 5-bit halves of H; the lobe term gb bx; band phase + zigzag
            p6_bhl <= l_b * p5_h(4 downto 0);
            p6_bhh <= l_b * p5_h(9 downto 5);
            p6_gbp <= l_gb * p5_bx;
            p6_bp  <= p5_bp(23 downto 10) + p5_zg(23 downto 10);
            p6_cp  <= p5_cp; p6_hh <= p5_hh;
            -- 7: B H; G b; band phase + camera (top 7 bits kept)
            p7_bh <= shift_left(resize(p6_bhh, 22), 5) + resize(p6_bhl, 22);
            p7_gb <= resize(p6_gbp(15 downto 5), 11);
            v_bq := unsigned(signed(p6_bp) + shift_left(resize(p6_cp, 14), 2));
            p7_bq <= v_bq(13 downto 7);
            p7_cp <= p6_cp; p7_hh <= p6_hh;
            -- 8: V - B H
            p8_f1 <= resize(l_v, 19) - signed(resize(shift_right(p7_bh, 6), 19));
            p8_gb <= p7_gb;
            p8_cf <= resize(shift_right(p7_cp, 4), 12);
            p8_hh <= p7_hh; p8_bq <= p7_bq;
            -- 9: F = V - B H - G b + camera (saturated 17 bits); the lobe heat offset
            v_f := resize(p8_f1, 20) - signed(resize(p8_gb, 20)) + resize(p8_cf, 20);
            p9_f <= v_f(16 downto 0);
            p9_hb <= signed(resize(p8_gb, 12)) - 105;
            p9_hh <= p8_hh; p9_bq <= p8_bq;
            -- 10: F kS (5-bit mantissa); heat_b = (Gb - 105) 49 / 512; the key flags
            p10_fk <= p9_f * signed('0' & c_kSm);
            v_hb := shift_left(resize(p9_hb, 18), 5) + shift_left(resize(p9_hb, 18), 4) + resize(p9_hb, 18);
            p10_hb <= resize(shift_right(v_hb, 9), 12);
            if p9_f < 300 then p10_lt <= '1'; else p10_lt <= '0'; end if;
            if p9_f > 252 then p10_rim <= '1'; else p10_rim <= '0'; end if;
            v_f := 300 - resize(p9_f, 20);
            p10_g <= unsigned(v_f(5 downto 0));
            p10_hh <= p9_hh; p10_bq <= p9_bq;
            -- 11: the streak phase Fk = F kS / 128 + flow (8 bits); heat = heat_a + heat_b
            case to_integer(c_kSe) is
                when 3      => v_k := unsigned(p10_fk(18 downto 3));
                when 2      => v_k := unsigned(p10_fk(19 downto 4));
                when 1      => v_k := unsigned(p10_fk(20 downto 5));
                when others => v_k := unsigned(p10_fk(21 downto 6));
            end case;
            p11_fk <= v_k(7 downto 0) + c_fl(15 downto 8);
            if p10_hb < 0 then v_ht := resize(p10_hh, 6);
            elsif p10_hb > 24 then v_ht := resize(p10_hh, 6) + 24;
            else v_ht := resize(p10_hh, 6) + unsigned(p10_hb(5 downto 0)); end if;
            if v_ht > 24 then p11_heat <= to_unsigned(24, 5); else p11_heat <= v_ht(4 downto 0); end if;
            p11_lt <= p10_lt; p11_rim <= p10_rim; p11_g <= p10_g; p11_bq <= p10_bq;
            -- 12: tongues, Lines, the flame / chevron / warm / pink decisions; the heat page (2x2 dither)
            v_p6 := p11_fk(6 downto 1);
            v_fl := '0';
            if p11_lt = '1' and q_inw(11) = '1' and resize(v_p6, 7) >= l_dt then v_fl := '1'; end if;
            if s_lines = '1' and p11_rim = '0' and v_p6 >= 10 then v_fl := '0'; end if;
            v_ch := '0';
            if q_inz(11) = '1' and p11_lt = '0' and p11_fk < 108 then v_ch := '1'; end if;
            v_wm := q_inz(11) and not v_fl;
            p12_fl <= v_fl; p12_ch <= v_ch; p12_wm <= v_wm;
            p12_pk <= q_pk(11) and not v_wm and not v_fl;
            p12_rim <= p11_rim;
            v_ht := resize(p11_heat, 6) + resize(unsigned'(q_xp(11) & l_yp), 6);
            if v_ht(5 downto 2) > 6 then p12_pg <= to_unsigned(6, 3); else p12_pg <= v_ht(4 downto 2); end if;
            p12_fk <= p11_fk; p12_g <= p11_g; p12_bq <= p11_bq;
            for i in 5 to 11 loop q_inw(i) <= q_inw(i - 1); q_inz(i) <= q_inz(i - 1); q_pk(i) <= q_pk(i - 1); end loop;
            -- 13: palette addresses: rim ramp / fire page x phase / chevron; band (warm bit, phase)
            if p12_fl = '1' and p12_rim = '1' then
                r_fi_a <= "111111" & p12_g(5 downto 3);
            elsif p12_fl = '1' then
                r_fi_a <= p12_pg & p12_fk(6 downto 1);
            else
                r_fi_a <= to_unsigned(448, 9) + resize(p12_fk(7 downto 1), 9);
            end if;
            r_bd_a <= p12_wm & p12_bq;
            p13_use <= p12_fl or p12_ch; p13_pk <= p12_pk;
            -- 14: palette data (EBR)
            p14_use <= p13_use; p14_pk <= p13_pk;
            -- 15: fabric copies
            p15_fi <= r_fi_q; p15_bd <= r_bd_q;
            p15_use <= p14_use; p15_pk <= p14_pk;
            -- 16: flame / chevron over the bands
            if p15_use = '1' then
                p16_y <= unsigned(p15_fi(23 downto 16)); p16_u <= unsigned(p15_fi(15 downto 8)); p16_v <= unsigned(p15_fi(7 downto 0));
            else
                p16_y <= unsigned(p15_bd(23 downto 16)); p16_u <= unsigned(p15_bd(15 downto 8)); p16_v <= unsigned(p15_bd(7 downto 0));
            end if;
            p16_pk <= p15_pk;
            -- 17: the pink diagonals (half pink); Ice swaps U and V
            if p16_pk = '1' then
                v_y := resize(p16_y, 9) + C_PY; v_u8 := resize(p16_u, 9) + C_PU; v_v8 := resize(p16_v, 9) + C_PV;
            else
                v_y := p16_y & '0'; v_u8 := p16_u & '0'; v_v8 := p16_v & '0';
            end if;
            p17_y <= v_y(8 downto 1);
            if s_ice = '1' then p17_u <= v_v8(8 downto 1); p17_v <= v_u8(8 downto 1);
            else p17_u <= v_u8(8 downto 1); p17_v <= v_v8(8 downto 1); end if;
            -- 18: 10 bits
            p18_y <= p17_y & "00"; p18_u <= p17_u & "00"; p18_v <= p17_v & "00";
            -- 19: black outside the generated avid
            if s_gav_sr(16) = '0' then
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            else
                s_y19 <= p18_y; s_u19 <= p18_u; s_v19 <= p18_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the 8-back taps come from block-RAM rings:
    -- the still's analog softness, and the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate; the syncs ride a block-RAM ring of the same latency
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
            sy_ra <= rg_wa - 21;
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y19 - s_cay;
                s_bau <= s_bau + s_u19 - s_cau;
                s_bav <= s_bav + s_v19 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y21 - s_cby;
                s_bbu <= s_bbu + s_u21 - s_cbu;
                s_bbv <= s_bbv + s_v21 - s_cbv;
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23 <= v_sc(13 downto 4);
            -- 24: luma 13/16
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23; s_v24 <= s_v23;
            -- 25: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture brushfire;
