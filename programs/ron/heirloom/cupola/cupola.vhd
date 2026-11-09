-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cupola v0.1.1 -- Heirloom (video phase), from the clip WIN_20230907_00_18_59 at 5.5 s.
-- v0.1.1: the camera's etch gain is 1/16 (was 1/4) and its luma box 8 px (was 4), so the default Etch bends the X's
-- smoothly instead of fraying them line by line; the Osc keeps its v0.1.0 strength (now 16x the camera).
--
-- The still: an RGB patch of the old modular synth.  Red / orange field of tall ovals ("pillars", 5.9 across) with
-- round bottoms, a dark-green band, a yellow band of circles with white arcs, a green floor; two big cupola cells
-- (2.04 across) whose outlines cross in yellow X's at x 535 / 1475, row 515; magenta domes under a cream top strip.
-- Each colour channel carries one layer:
--   R  pillars  P_R( u1^2 + vR(y) )                   u1 = fold of the 325 px oscillator, vR a fitted line table
--   G  cupola   Gbg(y) + GX( u2 + 0.545 |y - 515| - lev ) + GF( y - 838 + 0.137 D2 )
--   B  domes    min( key(y - 17 - 0.109 D2), key(127 - y + 0.469 D2), fade(u2) ) + BA + BX gate(y) + top strip
--   (u2 = fold of the 940 px oscillator, D2 = u2^2 / 256 x Dome), then a horizontal high-boost per channel (the
--   embossed, shadowed edges) and the BT.601 matrix into Y Cr Cb.
-- In the clip the pillar rows roll up (~120 lines/s) and the cupola breathes: the X opens into V / inverted-V and
-- closes again (~2.8 s) while the domes sink and the cream strip widens.
--
--   K1  Pillars   oval columns x0.33 .. x3 (5.9 at the still)
--   K2  Cupolas   cupola cells x0.35 .. x2.8 (2.04 at the still)
--   K3  Rows      the pillar rows' vertical frequency x0.33 .. x3
--   K4  Roll      bipolar: the rows roll up (+) / down (-); 701 = the clip's 120 lines/s
--   K5  Dome      the domes' and arcs' curvature x0.25 .. x4
--   K6  Etch      how far the camera (or the Osc, 16x) bends every contour
--   S7  Flip      the picture upside down (domes hang as bowls)
--   S8  Colours   RGB rotated: blue domes, magenta lattice, red floor
--   S9  Lattice   Single / Double: the X lattice at twice the density
--   S10 Polarity  Positive / Negative (every channel inverted)
--   S11 Source    Video / Osc
--   P12 CUPOLA    the X: pulled apart into arches (0) .. the still's crossing (507) .. pinched into hourglass windows
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line start
-- (computes the NEXT line: the rolled / static line-table reads and the vertical terms), two phase accumulators, a
-- 20-stage pixel path, then the shell's output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma
-- 13/16) and the blanking gate.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cupola_pkg.all;
use work.all;

architecture cupola of program_top is
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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- output lines from the top to the virtual centre (540)
    function f_l540(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(121, 10);
            when 4          => return to_unsigned(240, 10);
            when 8          => return to_unsigned(144, 10);
            when 12         => return to_unsigned(288, 10);
            when 5 | 6 | 14 => return to_unsigned(360, 10);
            when 1 | 2 | 10 => return to_unsigned(270, 10);
            when others     => return to_unsigned(540, 10);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- clamp a signed value into 0 .. 255
    function f_c8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
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
    signal r_lt_a : unsigned(8 downto 0) := (others => '0');
    signal r_lt_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_pr_a : unsigned(8 downto 0) := (others => '0');
    signal r_pr_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_gx_a : unsigned(8 downto 0) := (others => '0');
    signal r_gx_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_gf_a : unsigned(8 downto 0) := (others => '0');
    signal r_gf_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_mr_a, r_mg_a, r_mb_a : unsigned(7 downto 0) := (others => '0');
    signal r_mr_q, r_mg_q, r_mb_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_l540 : unsigned(9 downto 0) := to_unsigned(540, 10);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw roll, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(507 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(701, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_flp, s_rot, s_dbl, s_neg, s_src : std_logic := '0';

    -- per-field constants
    signal z_e1, z_e2, z_e3 : signed(4 downto 0) := (others => '0');   -- 1/8-octave exponents
    signal z_e5 : signed(5 downto 0) := (others => '0');
    signal z_k1b  : unsigned(12 downto 0) := to_unsigned(3226, 13);
    signal z_k2b  : unsigned(11 downto 0) := to_unsigned(1116, 12);
    signal z_n960 : unsigned(9 downto 0) := to_unsigned(960, 10);
    signal z_dp   : unsigned(9 downto 0) := (others => '0');            -- |P12 - 507|
    signal z_dpn  : std_logic := '0';
    signal z_tri  : unsigned(8 downto 0) := (others => '0');
    signal z_dr0  : unsigned(12 downto 0) := (others => '0');
    signal c_k1   : unsigned(15 downto 0) := to_unsigned(3226, 16);     -- phase steps per pixel (Q20 cycles)
    signal c_k2   : unsigned(15 downto 0) := to_unsigned(1116, 16);
    signal c_ph1  : unsigned(19 downto 0) := (others => '0');           -- phases of the first pixel
    signal c_ph2  : unsigned(19 downto 0) := (others => '0');
    signal c_dr   : unsigned(19 downto 0) := to_unsigned(30976, 20);    -- rows: table entries per line (Q16)
    signal c_er0, c_er0f : unsigned(24 downto 0) := (others => '0');    -- rows: table phase of line 0 (Q16)
    signal c_roll : unsigned(24 downto 0) := (others => '0');           -- the rows' roll (Q16 table entries)
    signal c_droll : signed(19 downto 0) := (others => '0');            -- ... per field
    signal c_m2   : unsigned(17 downto 0) := (others => '0');           -- roll^2
    signal c_gd   : unsigned(9 downto 0) := to_unsigned(128, 10);       -- dome gain (128 = x1)
    signal c_e2   : unsigned(19 downto 0) := (others => '0');           -- etch^2
    signal c_g6   : unsigned(5 downto 0) := to_unsigned(2, 6);          -- etch gain
    signal c_lp12 : signed(10 downto 0) := (others => '0');
    signal c_lt46 : unsigned(7 downto 0) := (others => '0');
    signal c_lev  : signed(10 downto 0) := (others => '0');             -- the X level (+ = pulled apart)
    signal c_bofs : unsigned(6 downto 0) := (others => '0');            -- the domes' breath (lines down)
    signal c_lf   : unsigned(8 downto 0) := (others => '0');            -- breath LFO phase (3 per field)
    signal c_ksh  : std_logic := '1';                                    -- boost: 1 = 1/32 (1080), 0 = 1/16

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
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

    -- line engine
    signal e_lc   : unsigned(4 downto 0) := to_unsigned(31, 5);
    signal e_zk   : std_logic := '0';
    signal e_y    : unsigned(19 downto 0) := (others => '0');           -- virtual line, Q8
    signal e_r    : unsigned(24 downto 0) := (others => '0');           -- rows table phase, Q16
    signal e_yv   : signed(11 downto 0) := (others => '0');
    signal e_dy   : unsigned(9 downto 0) := (others => '0');
    signal e_ta   : unsigned(21 downto 0) := (others => '0');
    signal e_fa   : unsigned(18 downto 0) := (others => '0');
    signal e_gt   : signed(11 downto 0) := (others => '0');
    signal n_vr   : signed(11 downto 0) := (others => '0');
    signal n_fd   : signed(11 downto 0) := (others => '0');
    signal n_t3, n_t1, n_t2 : signed(11 downto 0) := (others => '0');
    signal n_gbg  : unsigned(7 downto 0) := (others => '0');
    signal n_bbg  : unsigned(7 downto 0) := (others => '0');
    signal n_gate : unsigned(5 downto 0) := (others => '0');
    signal n_gtop : unsigned(7 downto 0) := (others => '0');

    -- beam and line registers
    signal b_ph1, b_ph2 : unsigned(19 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_vr  : signed(11 downto 0) := (others => '0');
    signal l_fd  : signed(11 downto 0) := (others => '0');
    signal l_t3, l_t1, l_t2 : signed(11 downto 0) := (others => '0');
    signal l_gbg, l_bbg, l_gtop : unsigned(7 downto 0) := (others => '0');
    signal l_gate : unsigned(5 downto 0) := (others => '0');

    -- pixel path
    signal p2_u1  : unsigned(7 downto 0) := (others => '0');
    signal p2_u2, p2_u2x : unsigned(8 downto 0) := (others => '0');
    signal p2_l1, p2_l2, p2_l3, p2_l4, p2_l5, p2_l6, p2_l7 : unsigned(7 downto 0) := (others => '0');
    signal p3_sq1 : unsigned(8 downto 0) := (others => '0');
    signal p3_d2p : unsigned(17 downto 0) := (others => '0');
    signal p3_u2, p3_u2x : unsigned(8 downto 0) := (others => '0');
    signal p3_lm  : signed(8 downto 0) := (others => '0');
    signal p4_d2g : unsigned(19 downto 0) := (others => '0');
    signal p4_fr, p4_fd : signed(12 downto 0) := (others => '0');
    signal p4_et  : signed(14 downto 0) := (others => '0');
    signal p4_kf  : signed(9 downto 0) := (others => '0');
    signal p5_d2g : unsigned(11 downto 0) := (others => '0');
    signal p5_fr, p5_fd : signed(12 downto 0) := (others => '0');
    signal p5_ef  : signed(14 downto 0) := (others => '0');
    signal p5_kf  : unsigned(7 downto 0) := (others => '0');
    signal p6_p3, p6_p1, p6_p2 : unsigned(11 downto 0) := (others => '0');
    signal p6_fr, p6_fd : signed(14 downto 0) := (others => '0');
    signal p6_q3, p6_q1, p6_q2 : signed(13 downto 0) := (others => '0');
    signal p6_kf, p7_kf, p8_kf, p9_kf : unsigned(7 downto 0) := (others => '0');
    signal p7_t3, p7_t1, p7_t2 : signed(14 downto 0) := (others => '0');
    signal p8_k1, p8_k2 : unsigned(7 downto 0) := (others => '0');
    signal p9_pr  : std_logic_vector(7 downto 0) := (others => '0');
    signal p9_gx  : std_logic_vector(15 downto 0) := (others => '0');
    signal p9_bd  : unsigned(7 downto 0) := (others => '0');
    signal p10_pr : std_logic_vector(7 downto 0) := (others => '0');
    signal p10_gx, p10_gf : std_logic_vector(15 downto 0) := (others => '0');
    signal p10_bd : unsigned(7 downto 0) := (others => '0');
    signal p11_g  : signed(10 downto 0) := (others => '0');
    signal p11_b  : signed(11 downto 0) := (others => '0');
    signal p11_bx : unsigned(13 downto 0) := (others => '0');
    signal p11_r  : unsigned(7 downto 0) := (others => '0');
    signal p12_r, p12_g, p12_b : unsigned(7 downto 0) := (others => '0');
    signal p13_r, p13_g, p13_b : unsigned(7 downto 0) := (others => '0');
    signal p13_lr, p13_lg, p13_lb : unsigned(12 downto 0) := (others => '0');
    signal p14_r, p14_g, p14_b : unsigned(7 downto 0) := (others => '0');
    signal p14_er, p14_eg, p14_eb : signed(9 downto 0) := (others => '0');
    signal p15_r, p15_g, p15_b : unsigned(7 downto 0) := (others => '0');
    signal p18_mr, p18_mg, p18_mb : std_logic_vector(31 downto 0) := (others => '0');
    signal p19_y  : unsigned(10 downto 0) := (others => '0');
    signal p19_u, p19_v : signed(12 downto 0) := (others => '0');
    signal p20_y, p20_u, p20_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums, the 8-back tap from a
    -- block-RAM ring), luma [1 2 1]/4 on taps aligned with the chroma centre, luma 13/16
    type t_cring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal cr : t_cring := (others => (others => '0'));
    signal cr_q, cr_f : std_logic_vector(19 downto 0) := (others => '0');
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal sr : t_sring := (others => (others => '1'));
    signal sr_q, sr_f : std_logic_vector(2 downto 0) := (others => '1');
    attribute no_rw_check : boolean;
    attribute no_rw_check of cr : signal is true;
    attribute no_rw_check of sr : signal is true;
    signal rc     : unsigned(7 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly   : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y26, s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);

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
            r_lt_q <= C_CU_LT(to_integer(r_lt_a));
            r_pr_q <= C_CU_PR(to_integer(r_pr_a));
            r_gx_q <= C_CU_GX(to_integer(r_gx_a));
            r_gf_q <= C_CU_GF(to_integer(r_gf_a));
            r_mr_q <= C_CU_MR(to_integer(r_mr_a));
            r_mg_q <= C_CU_MG(to_integer(r_mg_a));
            r_mb_q <= C_CU_MB(to_integer(r_mb_a));
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
        variable v_t  : signed(15 downto 0);
        variable v_u  : unsigned(19 downto 0);
        variable v_l  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_tid  <= registers_in(8)(3 downto 0);
            s_ys   <= f_vstep_y(s_tid);
            s_xsr  <= f_vstep_x(s_tid);
            s_l540 <= f_l540(s_tid);

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
            -- roll: bipolar, +-8 deadband
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
                s_flp <= registers_in(6)(0);
                s_rot <= registers_in(6)(1);
                s_dbl <= registers_in(6)(2);
                s_neg <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); z_k1b <= to_unsigned(6453, 13); z_k2b <= to_unsigned(2231, 12);
                                   z_n960 <= to_unsigned(360, 10); c_ksh <= '0';
                    when 3      => s_oxs <= to_unsigned(153, 9); z_k1b <= to_unsigned(4840, 13); z_k2b <= to_unsigned(1673, 12);
                                   z_n960 <= to_unsigned(640, 10); c_ksh <= '0';
                    when others => s_oxs <= to_unsigned(102, 9); z_k1b <= to_unsigned(3226, 13); z_k2b <= to_unsigned(1116, 12);
                                   z_n960 <= to_unsigned(960, 10); c_ksh <= '1';
                end case;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is                              -- Osc lattice step per line = vstep 121 / 256
                when 0 => s_oys <= to_unsigned(537, 10); when 4 => s_oys <= to_unsigned(272, 10);
                when 8 => s_oys <= to_unsigned(453, 10); when 12 => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10); when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others => s_oys <= to_unsigned(121, 10);
            end case;

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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result read at 50 + 24 k.
            -- The slot number is decoded at 30 + 24 k and the operands are pre-muxed into z_fa / z_fb at
            -- 31 + 24 k, so no zc decode sits on the multiplier's path.
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
                    when 342 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_k1b, 32);  z_fb <= resize(unsigned(z_e1(2 downto 0)), 16) + 8;   -- pillars
                    when 1  => z_fa <= resize(z_k2b, 32);  z_fb <= resize(unsigned(z_e2(2 downto 0)), 16) + 8;   -- cupolas
                    when 2  => z_fa <= resize(c_k1, 32);   z_fb <= resize(z_n960, 16);                           -- phase 1 start
                    when 3  => z_fa <= resize(c_k2, 32);   z_fb <= resize(z_n960, 16);                           -- phase 2 start
                    when 4  => z_fa <= resize(s_ys, 32);   z_fb <= resize(unsigned(z_e3(2 downto 0)), 16) + 8;   -- rows
                    when 5  => z_fa <= resize(z_dr0, 32);  z_fb <= to_unsigned(121, 16);
                    when 6  => z_fa <= resize(c_dr, 32);   z_fb <= resize(s_l540, 16);                           -- rows centre
                    when 7  => z_fa <= resize(s_vm, 32);   z_fb <= resize(s_vm, 16);                             -- roll^2
                    when 8  => z_fa <= resize(c_m2, 32);   z_fb <= to_unsigned(1942, 16);
                    when 9  => z_fa <= resize(s_g6(12 downto 3), 32); z_fb <= resize(s_g6(12 downto 3), 16);    -- etch^2
                    when 10 => z_fa <= resize(c_e2(19 downto 4), 32); z_fb <= to_unsigned(117, 16);
                    when 11 => z_fa <= resize(z_dp, 32);   z_fb <= to_unsigned(205, 16);                         -- the X level
                    when 12 => z_fa <= resize(z_tri, 32);  z_fb <= to_unsigned(46, 16);                          -- breath
                    when others => z_fa <= resize(z_tri, 32); z_fb <= to_unsigned(110, 16);
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: last field's roll and the breath LFO; exponents (1/8 octave); |P12 - 507|
                    when 20 =>
                        c_roll <= unsigned(signed(c_roll) + resize(c_droll, 25));
                        c_lf <= c_lf + 3;
                        v_d := signed(resize(s_g1(12 downto 3), 11)) - 512;                 -- pillars: 26 / 1024
                        v_t := shift_left(resize(v_d, 16), 4) + shift_left(resize(v_d, 16), 3) + shift_left(resize(v_d, 16), 1) + 512;
                        z_e1 <= resize(shift_right(v_t, 10), 5);
                        v_d := signed(resize(s_g2(12 downto 3), 11)) - 512;                 -- cupolas: 24 / 1024
                        v_t := shift_left(resize(v_d, 16), 4) + shift_left(resize(v_d, 16), 3) + 512;
                        z_e2 <= resize(shift_right(v_t, 10), 5);
                        v_d := signed(resize(s_g3(12 downto 3), 11)) - 512;                 -- rows: 26 / 1024
                        v_t := shift_left(resize(v_d, 16), 4) + shift_left(resize(v_d, 16), 3) + shift_left(resize(v_d, 16), 1) + 512;
                        z_e3 <= resize(shift_right(v_t, 10), 5);
                        v_d := signed(resize(s_g5(12 downto 3), 11)) - 512;                 -- dome: 32 / 1024
                        v_t := shift_left(resize(v_d, 16), 5) + 512;
                        z_e5 <= resize(shift_right(v_t, 10), 6);
                        if s_gp(12 downto 3) >= 507 then
                            z_dpn <= '0'; z_dp <= s_gp(12 downto 3) - 507;
                        else
                            z_dpn <= '1'; z_dp <= 507 - s_gp(12 downto 3);
                        end if;
                    when 21 =>
                        if c_lf <= 256 then z_tri <= c_lf; else z_tri <= resize(to_unsigned(512, 10) - resize(c_lf, 10), 9); end if;
                        -- dome gain 128 x (8 + m) / 8 x 2^s
                        case z_e5(5 downto 3) is
                            when "110"  => c_gd <= resize(shift_right(to_unsigned(16, 10) * (resize(unsigned(z_e5(2 downto 0)), 4) + 8), 2), 10);
                            when "111"  => c_gd <= resize(shift_right(to_unsigned(16, 10) * (resize(unsigned(z_e5(2 downto 0)), 4) + 8), 1), 10);
                            when "000"  => c_gd <= resize(to_unsigned(16, 10) * (resize(unsigned(z_e5(2 downto 0)), 4) + 8), 10);
                            when "001"  => c_gd <= resize(shift_left(to_unsigned(16, 10) * (resize(unsigned(z_e5(2 downto 0)), 4) + 8), 1), 10);
                            when others => c_gd <= to_unsigned(512, 10);
                        end case;
                        if z_e5 = 16 then c_gd <= to_unsigned(512, 10); end if;

                    -- products
                    when 50  =>
                        v_u := resize(f_acc(19 downto 3), 20);
                        case z_e1(4 downto 3) is
                            when "10"   => c_k1 <= resize(shift_right(v_u, 2), 16);
                            when "11"   => c_k1 <= resize(shift_right(v_u, 1), 16);
                            when "00"   => c_k1 <= resize(v_u, 16);
                            when others => c_k1 <= resize(shift_left(v_u, 1), 16);
                        end case;
                    when 74  =>
                        v_u := resize(f_acc(19 downto 3), 20);
                        case z_e2(4 downto 3) is
                            when "10"   => c_k2 <= resize(shift_right(v_u, 2), 16);
                            when "11"   => c_k2 <= resize(shift_right(v_u, 1), 16);
                            when "00"   => c_k2 <= resize(v_u, 16);
                            when others => c_k2 <= resize(shift_left(v_u, 1), 16);
                        end case;
                    when 98  => c_ph1 <= to_unsigned(258111, 20) - f_acc(19 downto 0);
                    when 122 => c_ph2 <= to_unsigned(998380, 20) - f_acc(19 downto 0);
                    when 146 =>
                        v_u := resize(f_acc(19 downto 3), 20);
                        case z_e3(4 downto 3) is
                            when "10"   => z_dr0 <= resize(shift_right(v_u, 2), 13);
                            when "11"   => z_dr0 <= resize(shift_right(v_u, 1), 13);
                            when "00"   => z_dr0 <= resize(v_u, 13);
                            when others => z_dr0 <= resize(shift_left(v_u, 1), 13);
                        end case;
                    when 170 => c_dr <= f_acc(19 downto 0);
                    when 194 =>
                        c_er0  <= c_roll + to_unsigned(16777216, 25) - f_acc(24 downto 0);
                        c_er0f <= c_roll + to_unsigned(16777216, 25) + f_acc(24 downto 0);
                    when 218 => c_m2 <= f_acc(17 downto 0);
                    when 242 =>
                        if s_vneg = '1' then c_droll <= -signed(resize(f_acc(28 downto 10), 20));
                        else c_droll <= signed(resize(f_acc(28 downto 10), 20)); end if;
                    when 266 => c_e2 <= f_acc(19 downto 0);
                    when 290 => c_g6 <= f_acc(22 downto 17);
                    when 314 =>
                        if z_dpn = '1' then c_lp12 <= -signed(resize(f_acc(18 downto 9), 11));
                        else c_lp12 <= signed(resize(f_acc(18 downto 9), 11)); end if;
                    when 338 => c_lt46 <= f_acc(15 downto 8);
                    when 362 =>
                        c_bofs <= f_acc(14 downto 8);
                        c_lev  <= signed(resize(c_lt46, 11)) - c_lp12;
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line's terms in ~10 clocks:
    --   yv (virtual line; Flip: 1079 - yv), the rows table phase (scaled about the centre, rolled),
    --   Fd line term 0.545 |yv - 515| - lev, t3 = yv - 838, t1 = yv - 17 - bofs / 2, t2 = 127 + bofs - yv,
    --   the cream top strip, and the two line-table reads (rolled: vR; static: Gbg, Bbg, gate)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : signed(12 downto 0);
        variable v_a  : unsigned(21 downto 0);
        variable v_t  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_y <= (others => '0');
                if s_flp = '1' then e_r <= c_er0f; else e_r <= c_er0; end if;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_y <= e_y + resize(s_ys, 20);
                if s_flp = '1' then e_r <= e_r - resize(c_dr, 25); else e_r <= e_r + resize(c_dr, 25); end if;
                e_lc <= (others => '0');
            elsif e_lc /= 31 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    v_y := signed(resize(e_y(19 downto 8), 13));
                    if s_flp = '1' then v_y := 1079 - v_y; end if;
                    e_yv <= resize(v_y, 12);
                    r_lt_a <= e_r(24 downto 16);                                    -- rolled read: vR
                when 2 =>
                    if e_yv >= 515 then e_dy <= unsigned(resize(e_yv - 515, 10));
                    else e_dy <= unsigned(resize(515 - e_yv, 10)); end if;
                    v_a := shift_left(resize(unsigned(e_yv), 22), 9) - shift_left(resize(unsigned(e_yv), 22), 5);
                    e_ta <= v_a;
                    n_t3 <= e_yv - 838;
                    n_t1 <= e_yv - 17 - signed(resize(c_bofs(6 downto 1), 12));
                    n_t2 <= 127 + signed(resize(c_bofs, 12)) - e_yv;
                    e_gt <= 20 + signed(resize(c_bofs(6 downto 2), 12)) + signed(resize(c_bofs(6 downto 1), 12)) - e_yv;
                when 3 =>
                    v_a := e_ta + shift_left(resize(unsigned(e_yv), 22), 2) + resize(unsigned(e_yv), 22);
                    r_lt_a <= v_a(18 downto 10);                                    -- static read: Gbg, Bbg, gate
                    e_fa <= shift_left(resize(e_dy, 19), 8) + shift_left(resize(e_dy, 19), 4);
                    if e_gt <= 0 then n_gtop <= (others => '0');
                    elsif e_gt >= 8 then n_gtop <= to_unsigned(255, 8);
                    else n_gtop <= unsigned(e_gt(2 downto 0)) & "00000"; end if;
                when 4 =>
                    v_a := resize(e_fa, 22) + shift_left(resize(e_dy, 22), 2) + shift_left(resize(e_dy, 22), 1) + resize(e_dy, 22);
                    n_fd <= signed(resize(v_a(21 downto 9), 12)) - resize(c_lev, 12);
                    n_vr <= signed(r_lt_q(31 downto 20));                           -- rolled read data (address set at lc 1)
                when 6 =>
                    n_gbg  <= unsigned(r_lt_q(19 downto 12));                       -- static read data (address set at lc 3)
                    n_bbg  <= unsigned(r_lt_q(11 downto 6)) & "00";
                    n_gate <= unsigned(r_lt_q(5 downto 0));
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the two oscillator phases (Q20 cycles), the line registers
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ph1 <= c_ph1;
                b_ph2 <= c_ph2;
                b_px  <= (others => '0');
                l_vr <= n_vr; l_fd <= n_fd; l_t3 <= n_t3; l_t1 <= n_t1; l_t2 <= n_t2;
                l_gbg <= n_gbg; l_bbg <= n_bbg; l_gate <= n_gate; l_gtop <= n_gtop;
            elsif g_avid = '1' then
                b_ph1 <= b_ph1 + resize(c_k1, 20);
                b_ph2 <= b_ph2 + resize(c_k2, 20);
                b_px  <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ph : unsigned(19 downto 0);
        variable v_l8 : unsigned(10 downto 0);
        variable v_e  : signed(14 downto 0);
        variable v_a  : signed(15 downto 0);
        variable v_k  : signed(15 downto 0);
        variable v_s  : signed(10 downto 0);
        variable v_d  : signed(13 downto 0);
        variable v_o  : signed(11 downto 0);
        variable v_r, v_g, v_b : unsigned(7 downto 0);
        variable v_y  : unsigned(10 downto 0);
        variable v_c  : signed(12 downto 0);
        variable v_f  : signed(14 downto 0);
        variable first : boolean;
    begin
        if rising_edge(clk) then
            -- 2: folds (0 at a cell centre, max at the cusp); the double lattice folds 2 x phase 2
            if b_ph1(19) = '1' then p2_u1 <= not b_ph1(18 downto 11); else p2_u1 <= b_ph1(18 downto 11); end if;
            if b_ph2(19) = '1' then p2_u2 <= not b_ph2(18 downto 10); else p2_u2 <= b_ph2(18 downto 10); end if;
            v_ph := b_ph2(18 downto 0) & '0';
            if s_dbl = '0' then
                if b_ph2(19) = '1' then p2_u2x <= not b_ph2(18 downto 10); else p2_u2x <= b_ph2(18 downto 10); end if;
            else
                if v_ph(19) = '1' then p2_u2x <= not v_ph(18 downto 10); else p2_u2x <= v_ph(18 downto 10); end if;
            end if;
            -- 3: squares; the luma box
            p3_sq1 <= resize(shift_right(p2_u1 * p2_u1, 7), 9);
            p3_d2p <= p2_u2 * p2_u2;
            p3_u2 <= p2_u2; p3_u2x <= p2_u2x;
            p2_l1 <= unsigned(s_lb_q); p2_l2 <= p2_l1; p2_l3 <= p2_l2; p2_l4 <= p2_l3;
            p2_l5 <= p2_l4; p2_l6 <= p2_l5; p2_l7 <= p2_l6;
            v_l8 := ((resize(unsigned(s_lb_q), 11) + resize(p2_l1, 11)) + (resize(p2_l2, 11) + resize(p2_l3, 11)))
                    + ((resize(p2_l4, 11) + resize(p2_l5, 11)) + (resize(p2_l6, 11) + resize(p2_l7, 11)));
            if s_src = '1' then p3_lm <= signed(resize(v_l8(10 downto 3), 9)) - 111;
            else p3_lm <= signed(resize(v_l8(10 downto 3), 9)); end if;
            -- 4: dome product; R and X sums; etch product; fade operand
            p4_d2g <= resize(p3_d2p(17 downto 8) * c_gd, 20);
            p4_fr <= signed(resize(p3_sq1, 13)) + resize(l_vr, 13);
            p4_fd <= signed(resize(p3_u2x, 13)) + resize(l_fd, 13);
            p4_et <= resize(shift_right(p3_lm, 1) * signed('0' & c_g6), 15);
            p4_kf <= 359 - signed(resize(p3_u2, 10));
            -- 5: D2g (clamped); etch (Osc 4x); fade = (359 - u2) x 27 / 8
            if p4_d2g(19 downto 7) > 4095 then p5_d2g <= to_unsigned(4095, 12);
            else p5_d2g <= p4_d2g(18 downto 7); end if;
            if s_src = '1' then p5_ef <= p4_et; else p5_ef <= shift_right(p4_et, 4); end if;   -- Osc = 16x the camera
            v_k := shift_left(resize(p4_kf, 16), 4) + shift_left(resize(p4_kf, 16), 3) + shift_left(resize(p4_kf, 16), 1) + resize(p4_kf, 16);
            v_k := shift_right(v_k, 3);
            if v_k < 0 then p5_kf <= (others => '0'); elsif v_k > 255 then p5_kf <= to_unsigned(255, 8);
            else p5_kf <= unsigned(v_k(7 downto 0)); end if;
            p5_fr <= p4_fr; p5_fd <= p4_fd;
            -- 6: parabola terms; R and X coordinates with the etch; vertical terms with the etch
            p6_p3 <= shift_right(p5_d2g, 3) + shift_right(p5_d2g, 7) + shift_right(p5_d2g, 8);
            p6_p1 <= shift_right(p5_d2g, 3) - shift_right(p5_d2g, 6);
            p6_p2 <= shift_right(p5_d2g, 1) - shift_right(p5_d2g, 5);
            p6_fr <= resize(p5_fr, 15) + p5_ef;
            p6_fd <= resize(p5_fd, 15) + p5_ef;
            v_e := shift_right(p5_ef, 2);
            p6_q3 <= resize(l_t3, 14) + resize(v_e, 14);
            p6_q1 <= resize(l_t1, 14) + resize(v_e, 14);
            p6_q2 <= resize(l_t2, 14) - resize(v_e, 14);
            p6_kf <= p5_kf;
            -- 7: t3, t1, t2; R and X ROM addresses
            p7_t3 <= resize(p6_q3, 15) + signed(resize(p6_p3, 15));
            p7_t1 <= resize(p6_q1, 15) - signed(resize(p6_p1, 15));
            p7_t2 <= resize(p6_q2, 15) + signed(resize(p6_p2, 15));
            v_f := p6_fr + 1280;
            if v_f < 0 then r_pr_a <= (others => '0');
            elsif v_f >= 4096 then r_pr_a <= (others => '1');
            else r_pr_a <= unsigned(v_f(11 downto 3)); end if;
            if p6_fd < 0 then r_gx_a <= (others => '0');
            elsif p6_fd >= 1024 then r_gx_a <= (others => '1');
            else r_gx_a <= unsigned(p6_fd(9 downto 1)); end if;
            p7_kf <= p6_kf;
            -- 8: floor ROM address; dome-band keys
            v_f := p7_t3 + 128;
            if v_f < 0 then r_gf_a <= (others => '0');
            elsif v_f > 511 then r_gf_a <= (others => '1');
            else r_gf_a <= unsigned(v_f(8 downto 0)); end if;
            v_a := shift_left(resize(p7_t1, 16), 5) + shift_left(resize(p7_t1, 16), 3) + shift_left(resize(p7_t1, 16), 1);
            if v_a < 0 then p8_k1 <= (others => '0'); elsif v_a > 255 then p8_k1 <= to_unsigned(255, 8);
            else p8_k1 <= unsigned(v_a(7 downto 0)); end if;
            v_a := shift_left(resize(p7_t2, 16), 4) + shift_left(resize(p7_t2, 16), 3) + resize(p7_t2, 16);
            if v_a < 0 then p8_k2 <= (others => '0'); elsif v_a > 255 then p8_k2 <= to_unsigned(255, 8);
            else p8_k2 <= unsigned(v_a(7 downto 0)); end if;
            p8_kf <= p7_kf;
            -- 9: R and X ROM data -> fabric; band = min(k1, k2)
            p9_pr <= r_pr_q; p9_gx <= r_gx_q;
            if p8_k1 < p8_k2 then p9_bd <= p8_k1; else p9_bd <= p8_k2; end if;
            p9_kf <= p8_kf;
            -- 10: floor ROM data -> fabric; band = min(band, fade)
            p10_gf <= r_gf_q; p10_pr <= p9_pr; p10_gx <= p9_gx;
            if p9_bd < p9_kf then p10_bd <= p9_bd; else p10_bd <= p9_kf; end if;
            -- 11: G sum; B partial sum; streak x gate
            p11_g <= signed(resize(l_gbg, 11)) + signed(resize(l_gtop, 11)) + resize(signed(p10_gx(15 downto 8)), 11)
                     + resize(signed(p10_gf(15 downto 8)), 11);
            p11_b <= signed(resize(l_bbg, 12)) + signed(resize(l_gtop - shift_right(l_gtop, 2), 12))
                     + signed(resize(p10_bd - shift_right(p10_bd, 4), 12)) + signed(resize(unsigned(p10_gf(7 downto 0)), 12));
            p11_bx <= unsigned(p10_gx(7 downto 0)) * l_gate;
            p11_r <= unsigned(p10_pr);
            -- 12: clamp
            p12_r <= p11_r;
            p12_g <= f_c8(p11_g);
            p12_b <= f_c8(p11_b + signed(resize(p11_bx(13 downto 6), 12)));
            -- 13: high-boost low-pass (one pole, per channel; restarts at each line's first pixel)
            first := s_gav_sr(10) = '1' and s_gav_sr(11) = '0';
            p13_r <= p12_r; p13_g <= p12_g; p13_b <= p12_b;
            if first then
                p13_lr <= resize(p12_r & "0000", 13); p13_lg <= resize(p12_g & "0000", 13); p13_lb <= resize(p12_b & "0000", 13);
            elsif c_ksh = '1' then
                v_d := signed(resize(p12_r & "0000", 14)) - signed(resize(p13_lr, 14));
                p13_lr <= unsigned(signed(p13_lr) + resize(shift_right(v_d, 5), 13));
                v_d := signed(resize(p12_g & "0000", 14)) - signed(resize(p13_lg, 14));
                p13_lg <= unsigned(signed(p13_lg) + resize(shift_right(v_d, 5), 13));
                v_d := signed(resize(p12_b & "0000", 14)) - signed(resize(p13_lb, 14));
                p13_lb <= unsigned(signed(p13_lb) + resize(shift_right(v_d, 5), 13));
            else
                v_d := signed(resize(p12_r & "0000", 14)) - signed(resize(p13_lr, 14));
                p13_lr <= unsigned(signed(p13_lr) + resize(shift_right(v_d, 4), 13));
                v_d := signed(resize(p12_g & "0000", 14)) - signed(resize(p13_lg, 14));
                p13_lg <= unsigned(signed(p13_lg) + resize(shift_right(v_d, 4), 13));
                v_d := signed(resize(p12_b & "0000", 14)) - signed(resize(p13_lb, 14));
                p13_lb <= unsigned(signed(p13_lb) + resize(shift_right(v_d, 4), 13));
            end if;
            -- 14: edge = s - lp
            p14_r <= p13_r; p14_g <= p13_g; p14_b <= p13_b;
            p14_er <= signed(resize(p13_r, 10)) - signed(resize(p13_lr(12 downto 4), 10));
            p14_eg <= signed(resize(p13_g, 10)) - signed(resize(p13_lg(12 downto 4), 10));
            p14_eb <= signed(resize(p13_b, 10)) - signed(resize(p13_lb(12 downto 4), 10));
            -- 15: boosted = s + 2 edge, clamped
            v_o := signed(resize(p14_r, 12)) + shift_left(resize(p14_er, 12), 1); p15_r <= f_c8(v_o);
            v_o := signed(resize(p14_g, 12)) + shift_left(resize(p14_eg, 12), 1); p15_g <= f_c8(v_o);
            v_o := signed(resize(p14_b, 12)) + shift_left(resize(p14_eb, 12), 1); p15_b <= f_c8(v_o);
            -- 16: Colours (rotate RGB -> GBR) and Polarity; matrix ROM addresses
            if s_rot = '1' then v_r := p15_g; v_g := p15_b; v_b := p15_r;
            else v_r := p15_r; v_g := p15_g; v_b := p15_b; end if;
            if s_neg = '1' then v_r := not v_r; v_g := not v_g; v_b := not v_b; end if;
            r_mr_a <= v_r; r_mg_a <= v_g; r_mb_a <= v_b;
            -- 17: matrix ROM data (EBR); 18: fabric
            p18_mr <= r_mr_q; p18_mg <= r_mg_q; p18_mb <= r_mb_q;
            -- 19: sums
            p19_y <= to_unsigned(64, 11) + resize(unsigned(p18_mr(31 downto 22)), 11) + resize(unsigned(p18_mg(31 downto 22)), 11)
                     + resize(unsigned(p18_mb(31 downto 22)), 11);
            p19_u <= 512 + resize(signed(p18_mr(21 downto 11)), 13) + resize(signed(p18_mg(21 downto 11)), 13)
                     + resize(signed(p18_mb(21 downto 11)), 13);
            p19_v <= 512 + resize(signed(p18_mr(10 downto 0)), 13) + resize(signed(p18_mg(10 downto 0)), 13)
                     + resize(signed(p18_mb(10 downto 0)), 13);
            -- 20: luma pre-scaled 16/13 (capped at 940); chroma clamped
            v_y := p19_y - 64;
            v_y := p19_y + shift_right(v_y, 2) - shift_right(v_y, 6);
            if v_y > 940 then p20_y <= to_unsigned(940, 10); else p20_y <= v_y(9 downto 0); end if;
            v_c := p19_u;
            if v_c < 0 then p20_u <= (others => '0'); elsif v_c > 1023 then p20_u <= (others => '1');
            else p20_u <= unsigned(v_c(9 downto 0)); end if;
            v_c := p19_v;
            if v_c < 0 then p20_v <= (others => '0'); elsif v_c > 1023 then p20_v <= (others => '1');
            else p20_v <= unsigned(v_c(9 downto 0)); end if;
            -- 21: filter input, black outside the generated avid
            if s_gav_sr(18) = '0' then
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            else
                s_y21 <= p20_y; s_u21 <= p20_u; s_v21 <= p20_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(18);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma through one [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the 8-back tap from a block-RAM ring), luma [1 2 1]/4 on
    -- taps aligned with the chroma centre (the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate.  The syncs ride a block-RAM ring of C_LAT.
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rc <= rc + 1;
            -- chroma ring: written at rc + 6, read at rc, RAM register, fabric register = the input of 8
            -- clocks before (never the write address)
            cr(to_integer(rc + 6)) <= std_logic_vector(s_u21) & std_logic_vector(s_v21);
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            -- sync ring: written at rc + C_LAT - 2, RAM register, fabric register = C_LAT clocks
            sr(to_integer(rc + to_unsigned(C_LAT - 2, 8))) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
        end if;
    end process p_ring;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_l  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 22: chroma running 8-sums (snap back to neutral once 8 blank samples are in, so a glitched
            -- sample can't offset the colours for good); luma taps
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u21 - unsigned(cr_f(19 downto 10));
                s_bav <= s_bav + s_v21 - unsigned(cr_f(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(0) <= s_y21;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 23: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 (both centred 4 pixels back)
            v_sc := resize(s_bau, 14) + s_baud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v23 <= v_sc(13 downto 4);
            v_l := resize(s_ly(3), 12) + shift_left(resize(s_ly(4), 12), 1) + resize(s_ly(5), 12);
            s_y23 <= v_l(11 downto 2);
            -- 24: luma 13/16
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23; s_v24 <= s_v23;
            -- 25 .. 28: pipeline
            s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            s_y26 <= s_y25; s_u26 <= s_u25; s_v26 <= s_v25;
            s_y27 <= s_y26; s_u27 <= s_u26; s_v27 <= s_v26;
            s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;
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
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cupola;
