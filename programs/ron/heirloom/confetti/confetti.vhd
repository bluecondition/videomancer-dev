-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Confetti v0.1.0 -- Heirloom (phase 3, video clips), rebuilt from the clip WIN_20231123_23_27_50 at 5.5 s.
--
-- The still: magenta petal cards over a striped ground.  Three columns of tall rounded petals (each wider than
-- the last) are sliced by slanted cuts into cards, thin slivers above and triangles below; a big magenta mass
-- fills the top-right corner.  Behind them, orange / teal stripes fan out from the top-left (arcs on the left,
-- fine diagonals on the right), and a red zone with navy horizontal stripes and lens-shaped ends lies across
-- the top.  A card shows the ground through it: magenta over orange or red, violet over teal or navy.  In the
-- clip the petal oscillator scrolls down one frame height per 5.2 s under the fixed cuts, so the cards swell,
-- shrink into slivers and reappear; the fan breathes and the red zone drifts sideways at half that rate.
--
-- The patch, as fitted (confetti.py has the tables, .heirloom_work/rb_confetti/notes.md every number):
--   u      = f1 (0.5 + ub (x - 315) + uc kc (x^2 - 315^2)), h = frac(u)     a linearly FM'd H VCO (columns)
--   card   = Eh(h) + Ev(dy) < T  and  V2(dy) + 1.039 (1 - tri(S)) < 1,  S = (y - m h - 397.6) / P
--            dy = the V oscillator folded (period 1116 lines, scrolling), Eh / Ev / V2 power-law ROMs
--   petal  = a big disc off the top-right corner (x-term ROM + per-line term), cut by a straight diagonal
--   ground = phi = 4.938 P + 1.863 P^2, P = (x + 152.5)(y - y0) / 1e6   (y0 breathes with the LFO)
--   zone   = max(oblique blob, corner lobe) + horizontal stripes (40.5 lines) + stripe term: red / navy levels
--
--   K1  Columns   petal column count, x0.42 .. x2.4 (the still at 512)
--   K2  Chirp     how much the columns widen to the right: narrowing (0) .. even (512) .. the still (768) .. x2
--   K3  Slant     the cuts' slant: tilted the other way (0) .. flat (512) .. the still (760) .. x2 steeper
--   K4  Speed     bipolar petal scroll (+ fan breathing and zone drift), still in the centre +-8; 700 = the clip
--   K5  Size      petal size: small chips (0) .. the still (512) .. fat cards merging into a lattice (1023)
--   K6  Warp      how far the camera (or the Osc) bends the fan, swells the cards and spreads the red zone
--   S7  Zone      On (the still) / Off: no red zone, the fan everywhere
--   S8  Petal     On (the still) / Off: no big top-right mass
--   S9  Palette   Magenta (the still) / Citrus (yellow cards, violet and green ground)
--   S10 Chevron   the cuts reverse on the left half of the screen: they meet in chevrons at the centre
--   S11 Source    Video / Osc
--   P12 CONFETTI  the cuts' frequency: whole uncut petals (0) .. the still (512) .. a shower of thin slivers
--
-- Engine: vblank-zone sequencer (one serial multiplier, operands staged in two registers, shared glide), a
-- per-line engine kicked at each line start (computes the NEXT line on one serial multiplier: the petal and
-- window levels, the window phase, the fan's start phase and steps, the zone quadratic, the lobe / petal /
-- stripe levels, the diagonal cut), a beam of 2nd-order DDAs, two x-indexed ROMs read at stage 1 (lobe and
-- petal x-terms, a 64 x 64 px brightness grid) and a 15-stage pixel path.  Virtual 1080-line coordinates (SD
-- shows the centre 4:3).  Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16
-- boxes, their 8-back taps in two block-RAM rings, with the running-sum resync; luma 13/16) before the
-- blanking gate.  Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.confetti_pkg.all;
use work.all;

architecture confetti of program_top is
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
    -- the zone's upper level 0.36 + rh cos(2 pi (i + 11.5) / 32), Q12 (the stripe phase, rotated 11 steps)
    function f_zt(i : unsigned(4 downto 0)) return signed is
    begin
        case to_integer(i) is
            when 0  => return to_signed(643, 13);  when 1  => return to_signed(462, 13);
            when 2  => return to_signed(319, 13);  when 3  => return to_signed(221, 13);
            when 4  => return to_signed(171, 13);  when 5  => return to_signed(171, 13);
            when 6  => return to_signed(221, 13);  when 7  => return to_signed(319, 13);
            when 8  => return to_signed(462, 13);  when 9  => return to_signed(643, 13);
            when 10 => return to_signed(857, 13);  when 11 => return to_signed(1095, 13);
            when 12 => return to_signed(1347, 13);  when 13 => return to_signed(1603, 13);
            when 14 => return to_signed(1855, 13);  when 15 => return to_signed(2093, 13);
            when 16 => return to_signed(2307, 13);  when 17 => return to_signed(2488, 13);
            when 18 => return to_signed(2631, 13);  when 19 => return to_signed(2729, 13);
            when 20 => return to_signed(2779, 13);  when 21 => return to_signed(2779, 13);
            when 22 => return to_signed(2729, 13);  when 23 => return to_signed(2631, 13);
            when 24 => return to_signed(2488, 13);  when 25 => return to_signed(2307, 13);
            when 26 => return to_signed(2093, 13);  when 27 => return to_signed(1855, 13);
            when 28 => return to_signed(1603, 13);  when 29 => return to_signed(1347, 13);
            when 30 => return to_signed(1095, 13);  when others => return to_signed(857, 13);
        end case;
    end function;
    -- the ground stripe's push on the zone: 0.06 cos(2 pi (i + 0.5) / 8), Q12
    function f_gsr(i : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(i) is
            when 0 | 7  => return to_signed(227, 9);
            when 1 | 6  => return to_signed(94, 9);
            when 2 | 5  => return to_signed(-94, 9);
            when others => return to_signed(-227, 9);
        end case;
    end function;
    -- 2^(E / 256) in Q12 from a Q8-octave exponent (E = -768 .. 767): a linear mantissa (1 + f) shifted
    function f_f12(e : signed(11 downto 0)) return unsigned is
        variable m : unsigned(15 downto 0);
    begin
        m := resize(unsigned('1' & std_logic_vector(e(7 downto 0))), 16);
        case to_integer(e(11 downto 8)) is
            when -3     => return shift_left(m, 1);
            when -2     => return shift_left(m, 2);
            when -1     => return shift_left(m, 3);
            when 0      => return shift_left(m, 4);
            when 1      => return shift_left(m, 5);
            when others => return shift_left(m, 6);
        end case;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0; every second pixel at 1080p / 720p
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_eh_a : unsigned(7 downto 0) := (others => '0');
    signal r_eh_q : unsigned(11 downto 0) := (others => '0');
    signal r_pw_a : unsigned(8 downto 0) := (others => '0');
    signal r_pw_q : unsigned(11 downto 0) := (others => '0');
    signal r_x_a  : unsigned(9 downto 0) := (others => '0');
    signal r_x_q  : unsigned(15 downto 0) := (others => '0');
    signal r_lv_a : unsigned(9 downto 0) := (others => '0');
    signal r_lv_q : unsigned(3 downto 0) := (others => '0');
    signal r_pa   : unsigned(9 downto 0) := (others => '0');
    signal r_py_q, r_pu_q, r_pv_q : unsigned(7 downto 0) := (others => '0');
    signal r_ca   : unsigned(7 downto 0) := (others => '0');
    signal r_cq   : std_logic_vector(23 downto 0) := (others => '0');

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
    signal s_lbh  : std_logic := '1';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(768 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(760 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
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
    signal s_zof, s_pof, s_pal, s_chv, s_src : std_logic := '0';

    -- per-field scratch
    signal s_acc  : unsigned(24 downto 0) := (others => '0');            -- LFO, Q24 turns (bit 24: the slow LFO)
    signal s_rate : unsigned(24 downto 0) := to_unsigned(51448, 25);     -- LFO step per field (two's complement)
    signal z_f1, z_fw : unsigned(14 downto 0) := to_unsigned(4096, 15);  -- 2^(E / 256), Q12
    signal z_kca  : unsigned(8 downto 0) := (others => '0');
    signal z_fneg : std_logic := '0';
    signal z_g3a  : unsigned(8 downto 0) := (others => '0');
    signal z_g3n  : std_logic := '0';
    signal z_tr, z_trb : unsigned(7 downto 0) := (others => '0');
    signal z_fcm  : unsigned(22 downto 0) := (others => '0');            -- |f1 uc kc|, Q40
    signal z_q    : unsigned(20 downto 0) := (others => '0');
    signal z_adx  : unsigned(10 downto 0) := to_unsigned(1380, 11);
    signal z_xa   : unsigned(31 downto 0) := (others => '0');            -- staged multiplier operands
    signal z_xb   : unsigned(15 downto 0) := (others => '0');
    signal z_tc   : unsigned(4 downto 0) := (others => '0');
    signal z_sel  : unsigned(4 downto 0) := (others => '0');
    signal z_srun, z_s1, z_s3, z_s20 : std_logic := '0';
    signal z_selr : unsigned(4 downto 0) := (others => '0');

    -- per-field constants
    signal c_u0   : signed(29 downto 0) := (others => '0');               -- column VCO start, Q24
    signal c_d0   : signed(22 downto 0) := (others => '0');               -- first difference, Q24
    signal c_dd   : signed(13 downto 0) := (others => '0');               -- second difference, Q26
    signal c_ks   : unsigned(9 downto 0) := to_unsigned(62, 10);          -- |m / P|, Q6 cycles per cell
    signal c_ksn  : std_logic := '0';
    signal c_ip   : unsigned(18 downto 0) := to_unsigned(63937, 19);      -- 1 / P, Q24 cycles per line
    signal c_t10  : unsigned(12 downto 0) := to_unsigned(1024, 13);       -- petal size T, Q10
    signal c_gw   : unsigned(8 downto 0) := to_unsigned(25, 9);           -- warp per luma step, Q2
    signal c_yc   : unsigned(10 downto 0) := to_unsigned(649, 11);        -- V oscillator centre (line)
    signal c_y0   : signed(11 downto 0) := to_signed(-600, 12);           -- fan centre y
    signal c_dxn  : std_logic := '1';
    signal c_adx0 : unsigned(10 downto 0) := to_unsigned(1380, 11);       -- |x0 - zone centre x|
    signal c_z0b  : signed(16 downto 0) := (others => '0');               -- 1 - Kxx dx0^2, Q12
    signal c_zd0  : signed(19 downto 0) := (others => '0');               -- zone first difference part, Q24
    signal c_zdd  : signed(9 downto 0) := to_signed(-120, 10);
    signal c_kz0  : unsigned(7 downto 0) := to_unsigned(60, 8);
    signal c_ctr  : unsigned(10 downto 0) := to_unsigned(959, 11);        -- the screen's centre pixel
    signal c_x0h  : unsigned(11 downto 0) := (others => '0');             -- SD offset (h)

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');

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
    signal e_dy, e_dy2, e_dl : signed(11 downto 0) := (others => '0');
    signal e_ady, e_ady2, e_adl, e_adz, e_adp, e_adv : unsigned(9 downto 0) := (others => '0');
    signal e_sdy, e_sdy2, e_sdz, e_sdv : std_logic := '0';
    signal e_cy   : unsigned(11 downto 0) := (others => '0');
    signal e_vc   : unsigned(11 downto 0) := (others => '0');
    signal e_dz, e_dp, e_dv : signed(11 downto 0) := (others => '0');
    signal e_sq   : unsigned(23 downto 0) := (others => '0');
    signal e_be   : unsigned(11 downto 0) := (others => '0');
    signal e_ae   : unsigned(16 downto 0) := (others => '0');
    signal e_xa   : unsigned(31 downto 0) := (others => '0');            -- staged multiplier operands
    signal e_xb   : unsigned(15 downto 0) := (others => '0');
    signal e_tc   : unsigned(4 downto 0) := (others => '0');
    signal e_sel  : unsigned(4 downto 0) := (others => '0');
    signal e_srun, e_s1, e_s2, e_s3, e_s20 : std_logic := '0';
    signal e_selr : unsigned(4 downto 0) := (others => '0');
    signal e_hp   : unsigned(4 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    -- next-line staging (n_*), loaded into the active-line registers (l_*) and the beam at the line start
    signal n_eth  : signed(13 downto 0) := (others => '0');
    signal n_wth  : unsigned(10 downto 0) := (others => '0');
    signal n_sy   : unsigned(11 downto 0) := (others => '0');
    signal n_ph0  : unsigned(23 downto 0) := (others => '0');
    signal n_pd0  : unsigned(21 downto 0) := (others => '0');
    signal n_pdd  : unsigned(11 downto 0) := (others => '0');
    signal n_z0   : signed(17 downto 0) := (others => '0');                -- zone start value, Q12
    signal n_zd0  : signed(23 downto 0) := (others => '0');
    signal n_ly   : signed(9 downto 0) := (others => '0');
    signal n_pth  : unsigned(7 downto 0) := (others => '0');
    signal n_cut  : unsigned(12 downto 0) := (others => '0');
    signal n_zt2  : signed(12 downto 0) := (others => '0');
    signal n_by   : unsigned(4 downto 0) := (others => '0');
    signal l_eth  : signed(13 downto 0) := (others => '0');
    signal l_wth  : unsigned(10 downto 0) := (others => '0');
    signal l_sy   : unsigned(11 downto 0) := (others => '0');
    signal l_ly   : signed(9 downto 0) := (others => '0');
    signal l_pth  : unsigned(7 downto 0) := (others => '0');
    signal l_cut  : unsigned(12 downto 0) := (others => '0');
    signal l_zt2, l_zt1 : signed(12 downto 0) := (others => '0');
    signal l_by   : unsigned(4 downto 0) := (others => '0');

    -- beam: 2nd-order DDAs of the column VCO (Q26), the fan (Q24) and the zone (Q24); stage-1 x counter
    signal b_gsr  : std_logic_vector(3 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_xh   : unsigned(11 downto 0) := (others => '0');
    signal b_u    : signed(31 downto 0) := (others => '0');
    signal b_d    : signed(23 downto 0) := (others => '0');
    signal b_ph   : unsigned(23 downto 0) := (others => '0');
    signal b_pd   : unsigned(21 downto 0) := (others => '0');
    signal b_pdd  : unsigned(11 downto 0) := (others => '0');
    signal b_z    : signed(29 downto 0) := (others => '0');
    signal b_zd   : signed(23 downto 0) := (others => '0');

    -- pixel path
    signal p2_lf, p3_lf, p4_lf, p5_lf : std_logic := '0';
    signal p2_ct, p3_ct, p4_ct, p5_ct, q6_ct, q6_lf : std_logic := '0';
    signal p3_xr, p4_xr, p5_xr : unsigned(15 downto 0) := (others => '0');
    signal p3_lv, p4_lv, p5_lv, p6_lv, p7_lv, p8_lv, p9_lv : unsigned(3 downto 0) := (others => '0');
    signal p3_lm  : unsigned(5 downto 0) := (others => '0');
    signal p4_lw  : unsigned(14 downto 0) := (others => '0');
    signal p5_lw  : unsigned(16 downto 0) := (others => '0');
    signal p6_h8  : unsigned(7 downto 0) := (others => '0');
    signal p6_un, p7_un, p8_un : std_logic := '0';
    signal p6_ph, p7_ph : unsigned(23 downto 0) := (others => '0');
    signal p6_l15 : unsigned(17 downto 0) := (others => '0');
    signal p6_dz, p6_dl, p7_zm : signed(13 downto 0) := (others => '0');
    signal p6_pt, p7_pt, p8_pt, p9_pt : std_logic := '0';
    signal p6_et, p7_et, p8_et : signed(14 downto 0) := (others => '0');
    signal p6_zl, p7_zl : unsigned(12 downto 0) := (others => '0');
    signal p7_kp  : unsigned(17 downto 0) := (others => '0');
    signal p8_eh  : unsigned(11 downto 0) := (others => '0');
    signal p8_s   : unsigned(11 downto 0) := (others => '0');
    signal p8_sx, p9_sx, p10_sx : unsigned(2 downto 0) := (others => '0');
    signal p8_zr  : signed(12 downto 0) := (others => '0');
    signal p9_zr  : signed(13 downto 0) := (others => '0');
    signal p9_sh  : std_logic := '0';
    signal p9_w   : unsigned(10 downto 0) := (others => '0');
    signal p10_lv : unsigned(3 downto 0) := (others => '0');
    signal p10_card, p11_card, p12_card, p13_card : std_logic := '0';
    signal p10_zone : unsigned(1 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v, p13_cy, p13_cu, p13_cv : unsigned(7 downto 0) := (others => '0');
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
            r_eh_q <= C_CF_EH(to_integer(r_eh_a));
            r_pw_q <= C_CF_POW(to_integer(r_pw_a));
            r_x_q  <= C_CF_XR(to_integer(r_x_a));
            r_lv_q <= C_CF_LV(to_integer(r_lv_a));
            r_py_q <= C_CF_PY(to_integer(r_pa));
            r_pu_q <= C_CF_PU(to_integer(r_pa));
            r_pv_q <= C_CF_PV(to_integer(r_pa));
            r_cq   <= C_CF_CARD(to_integer(r_ca));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_i  : signed(12 downto 0);
        variable v_d  : signed(22 downto 0);
        variable v_u  : signed(29 downto 0);
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

            -- K4 Speed: bipolar, +-8 deadband, |v| - 8 (square law on the multiplier)
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
                s_zof <= registers_in(6)(0);
                s_pof <= registers_in(6)(1);
                s_pal <= registers_in(6)(2);
                s_chv <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                case to_integer(s_xsr) is
                    when 4 =>
                        s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); s_lbh <= '0';
                        c_x0h <= to_unsigned(480, 12); c_zdd <= to_signed(-483, 10); c_kz0 <= to_unsigned(241, 8);
                    when 3 =>
                        s_x0 <= (others => '0'); s_oxs <= to_unsigned(153, 9); s_lbh <= '1';
                        c_x0h <= (others => '0'); c_zdd <= to_signed(-271, 10); c_kz0 <= to_unsigned(135, 8);
                    when others =>
                        s_x0 <= (others => '0'); s_oxs <= to_unsigned(102, 9); s_lbh <= '1';
                        c_x0h <= (others => '0'); c_zdd <= to_signed(-120, 10); c_kz0 <= to_unsigned(60, 8);
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

            -- one-off states before the products: exponents, signs, the LFO step and its triangles
            if s_zone = '1' and s_zc = 20 then
                v_s := signed(resize(s_g1(12 downto 3), 11)) - 512;
                z_f1 <= resize(f_f12(resize(v_s, 12) + resize(shift_right(v_s, 2), 12)), 15);       -- 1.25 (K1 - 512)
                v_s := signed(resize(s_gp(12 downto 3), 11)) - 512;
                z_fw <= resize(f_f12(resize(v_s, 12) + resize(shift_right(v_s, 1), 12)), 15);       -- 1.5 (P12 - 512)
                v_s := signed(resize(s_g5(12 downto 3), 11)) - 512;
                v_e := resize(v_s, 12) - resize(shift_right(v_s, 3), 12) - resize(shift_right(v_s, 5), 12);
                c_t10 <= resize(shift_right(f_f12(v_e), 2), 13);
                v_s := signed(resize(s_g2(12 downto 3), 11)) - 512;
                if v_s < 0 then v_s := -v_s; z_fneg <= '0';
                elsif v_s = 0 then z_fneg <= '0';
                else z_fneg <= '1'; end if;
                if v_s > 511 then z_kca <= to_unsigned(511, 9); else z_kca <= unsigned(v_s(8 downto 0)); end if;
                v_s := signed(resize(s_g3(12 downto 3), 11)) - 512;
                if v_s < 0 then v_s := -v_s; z_g3n <= '1'; else z_g3n <= '0'; end if;
                if v_s > 511 then z_g3a <= to_unsigned(511, 9); else z_g3a <= unsigned(v_s(8 downto 0)); end if;
                s_acc <= s_acc + s_rate;
            end if;
            if s_zone = '1' and s_zc = 21 then
                if s_acc(23) = '1' then z_tr <= not s_acc(22 downto 15); else z_tr <= s_acc(22 downto 15); end if;
                if s_acc(24) = '1' then z_trb <= not s_acc(23 downto 16); else z_trb <= s_acc(23 downto 16); end if;
                c_ctr <= resize(s_w(11 downto 1), 11) - 1;
            end if;

            -- serial products: slot k stages its operands at zc = 30 + 24 k (tc 0), loads at tc 3, 16 iterations,
            -- the result is read at tc 20 (zc = 50 + 24 k)
            if s_zone = '1' and s_zc = 29 then
                z_tc <= (others => '0'); z_sel <= (others => '0'); z_srun <= '1';
            elsif z_srun = '1' then
                if z_tc = 23 then
                    z_tc <= (others => '0');
                    if z_sel = 18 then z_srun <= '0'; else z_sel <= z_sel + 1; end if;
                else
                    z_tc <= z_tc + 1;
                end if;
            end if;
            z_selr <= z_sel;
            if z_srun = '1' and z_tc = 0 then z_s1 <= '1'; else z_s1 <= '0'; end if;
            if z_srun = '1' and z_tc = 2 then z_s3 <= '1'; else z_s3 <= '0'; end if;
            if z_srun = '1' and z_tc = 19 then z_s20 <= '1'; else z_s20 <= '0'; end if;
            if z_s1 = '1' then
                case to_integer(z_selr) is
                    when 0 =>       -- f1 ub xs / 2 (first difference), Q24 = f1 * round(37025.7 xs / 4) >> 11
                        case to_integer(s_xs) is
                            when 4      => z_xa <= to_unsigned(37026, 32);
                            when 3      => z_xa <= to_unsigned(27769, 32);
                            when others => z_xa <= to_unsigned(18513, 32);
                        end case;
                        z_xb <= resize(z_f1, 16);
                    when 1 =>       -- f1 ub (315 - x0), Q24 = f1 * round(37025.7 (315 - x0) / 128) >> 5
                        if s_xs = 4 then z_xa <= to_unsigned(21695, 32); else z_xa <= to_unsigned(91117, 32); end if;
                        z_xb <= resize(z_f1, 16);
                    when 2 => z_xa <= to_unsigned(309623, 32); z_xb <= resize(z_kca, 16);            -- uc |kc|
                    when 3 => z_xa <= resize(z_q(15 downto 0), 32); z_xb <= resize(z_f1, 16);         -- f1 uc kc
                    when 4 =>       -- fc (x0^2 - 99225)
                        if s_xs = 4 then z_xa <= to_unsigned(41625, 32); else z_xa <= to_unsigned(99225, 32); end if;
                        z_xb <= resize(z_fcm(22 downto 8), 16);
                    when 5 =>       -- fc (2 x0h xs + xs^2) / 4
                        z_xa <= resize(z_fcm(22 downto 4), 32);
                        case to_integer(s_xs) is
                            when 4      => z_xb <= to_unsigned(3856, 16);
                            when 3      => z_xb <= to_unsigned(9, 16);
                            when others => z_xb <= to_unsigned(4, 16);
                        end case;
                    when 6 =>       -- 2 fc sigma^2
                        z_xa <= resize(z_fcm, 32);
                        case to_integer(s_xs) is
                            when 4      => z_xb <= to_unsigned(16, 16);
                            when 3      => z_xb <= to_unsigned(9, 16);
                            when others => z_xb <= to_unsigned(4, 16);
                        end case;
                    when 7  => z_xa <= resize(z_g3a, 32); z_xb <= resize(z_fw, 16);                   -- slant / P
                    when 8  => z_xa <= to_unsigned(63937, 32); z_xb <= resize(z_fw, 16);              -- 1 / P
                    when 9  => z_xa <= resize(s_g6(12 downto 3), 32); z_xb <= resize(s_g6(12 downto 3), 16);   -- warp^2
                    when 10 => z_xa <= resize(z_q(19 downto 4), 32); z_xb <= to_unsigned(4798, 16);
                    when 11 => z_xa <= resize(s_vm, 32); z_xb <= resize(s_vm, 16);                    -- speed^2
                    when 12 => z_xa <= resize(z_q(17 downto 0), 32); z_xb <= to_unsigned(1626, 16);
                    when 13 => z_xa <= resize(s_acc(23 downto 8), 32); z_xb <= to_unsigned(1116, 16); -- V centre
                    when 14 => z_xa <= resize(z_tr, 32); z_xb <= to_unsigned(1080, 16);               -- fan y0
                    when 15 => z_xa <= resize(z_trb, 32); z_xb <= to_unsigned(960, 16);               -- zone x
                    when 16 => z_xa <= resize(z_adx, 32); z_xb <= resize(z_adx, 16);                  -- dx0^2
                    when 17 => z_xa <= resize(z_q(20 downto 4), 32); z_xb <= to_unsigned(15467, 16);
                    when others =>  -- |dx0| 15467 xs
                        z_xa <= resize(z_adx, 32);
                        case to_integer(s_xs) is
                            when 4      => z_xb <= to_unsigned(61868, 16);
                            when 3      => z_xb <= to_unsigned(46401, 16);
                            when others => z_xb <= to_unsigned(30934, 16);
                        end case;
                end case;
            end if;
            if z_s3 = '1' then
                f_acc <= (others => '0');
                f_a <= z_xa; f_b <= z_xb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            if z_s20 = '1' then
                case to_integer(z_selr) is
                    when 0 => c_d0 <= signed(resize(f_acc(31 downto 11), 23));
                    when 1 =>
                        v_u := signed(resize(shift_left(resize(z_f1, 30), 11), 30)) - signed(resize(f_acc(31 downto 5), 30));
                        c_u0 <= v_u;
                    when 2 => z_q <= resize(f_acc(31 downto 12), 21);
                    when 3 => z_fcm <= resize(f_acc(30 downto 8), 23);
                    when 4 =>
                        if z_fneg = '1' then c_u0 <= c_u0 + signed(resize(f_acc(31 downto 8), 30));
                        else c_u0 <= c_u0 - signed(resize(f_acc(31 downto 8), 30)); end if;
                    when 5 =>
                        if z_fneg = '1' then c_d0 <= c_d0 - signed(resize(f_acc(31 downto 14), 23));
                        else c_d0 <= c_d0 + signed(resize(f_acc(31 downto 14), 23)); end if;
                    when 6 =>
                        if z_fneg = '1' then c_dd <= -signed(resize(f_acc(27 downto 15), 14));
                        else c_dd <= signed(resize(f_acc(27 downto 15), 14)); end if;
                    when 7  => c_ks <= f_acc(23 downto 14); c_ksn <= z_g3n;
                    when 8  => c_ip <= f_acc(30 downto 12);
                    when 9  => z_q <= resize(f_acc(19 downto 0), 21);
                    when 10 => c_gw <= f_acc(28 downto 20);
                    when 11 => z_q <= resize(f_acc(17 downto 0), 21);
                    when 12 =>
                        if s_vneg = '1' then s_rate <= unsigned(-signed(resize(f_acc(29 downto 10), 25)));
                        else s_rate <= resize(f_acc(29 downto 10), 25); end if;
                    when 13 =>      -- yc = 649 + theta 1116 (mod 1116); the window envelope 20 lines up
                        v_y := to_unsigned(649, 12) + resize(f_acc(31 downto 16), 12);
                        if v_y >= 1116 then v_y := v_y - 1116; end if;
                        c_yc <= v_y(10 downto 0);
                    when 14 => c_y0 <= to_signed(-600, 12) - signed(resize(f_acc(18 downto 8), 12));
                    when 15 =>      -- zone centre x = 1380 - 960 tri; dx0 = x0 - xb
                        v_i := signed(resize(c_x0h(11 downto 1), 13)) - (to_signed(1380, 13) - signed(resize(f_acc(17 downto 8), 13)));
                        if v_i < 0 then v_i := -v_i; c_dxn <= '1'; else c_dxn <= '0'; end if;
                        z_adx <= unsigned(v_i(10 downto 0));
                    when 16 => z_q <= f_acc(20 downto 0); c_adx0 <= z_adx;
                    when 17 => c_z0b <= to_signed(4096, 17) - signed(resize(f_acc(31 downto 16), 17));
                    when others =>
                        c_zd0 <= signed(resize(f_acc(27 downto 8), 20)) - signed(resize(c_kz0, 20));
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
            if s_lbh = '1' then s_lb_wa <= s_wcol(10 downto 1); else s_lb_wa <= s_wcol(9 downto 0); end if;
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
    -- computes the NEXT line on one serial multiplier (22 slots of 21 clocks:
    -- operands staged at tc 0, loaded at tc 3, the result read at tc 20): the
    -- petal and window levels, the window phase, the fan's start phase and steps,
    -- the zone quadratic, the lobe / corner-petal / stripe levels and the
    -- diagonal cut; ~480 clocks, so it fits SD's 858-clock line
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t  : unsigned(15 downto 0);
        variable v_vy : signed(12 downto 0);
        variable v_d  : signed(12 downto 0);
        variable v_a  : signed(12 downto 0);
        variable v_z  : signed(29 downto 0);
        variable v_p  : signed(10 downto 0);
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

            v_vy := signed(resize(e_vy(19 downto 8), 13));
            case to_integer(e_lc) is
                when 1 =>
                    v_d := v_vy - signed(resize(c_yc, 13));
                    if v_d < 0 then v_d := v_d + 1116; end if;
                    e_dy <= v_d(11 downto 0);
                    v_d := v_vy - 215;
                    if v_d < 0 then v_d := v_d + 1000; end if;
                    e_dl <= v_d(11 downto 0);
                    v_d := v_vy - resize(c_y0, 13);
                    e_cy <= unsigned(v_d(11 downto 0));
                    e_dz <= resize(v_vy - 210, 12);
                    e_dp <= resize(v_vy - 60, 12);
                    e_dv <= resize(v_vy - 398, 12);
                    e_vc <= resize(unsigned(v_vy(11 downto 0)), 12) + 455;
                when 2 =>
                    if e_dy >= 558 then e_dy <= e_dy - 1116; e_dy2 <= e_dy - 1096;
                    elsif e_dy >= 538 then e_dy2 <= e_dy - 1096;
                    else e_dy2 <= e_dy + 20; end if;
                    if e_dl >= 500 then e_dl <= e_dl - 1000; end if;
                when 3 =>
                    v_a := resize(e_dy, 13);  if v_a < 0 then v_a := -v_a; e_sdy <= '1'; else e_sdy <= '0'; end if;
                    e_ady <= unsigned(v_a(9 downto 0));
                    v_a := resize(e_dy2, 13); if v_a < 0 then v_a := -v_a; e_sdy2 <= '1'; else e_sdy2 <= '0'; end if;
                    e_ady2 <= unsigned(v_a(9 downto 0));
                    v_a := resize(e_dl, 13);  if v_a < 0 then v_a := -v_a; end if;
                    e_adl <= unsigned(v_a(9 downto 0));
                    v_a := resize(e_dz, 13);  if v_a < 0 then v_a := -v_a; e_sdz <= '1'; else e_sdz <= '0'; end if;
                    e_adz <= unsigned(v_a(9 downto 0));
                    v_a := resize(e_dp, 13);  if v_a < 0 then v_a := -v_a; end if;
                    e_adp <= unsigned(v_a(9 downto 0));
                    v_a := resize(e_dv, 13);  if v_a < 0 then v_a := -v_a; e_sdv <= '1'; else e_sdv <= '0'; end if;
                    e_adv <= unsigned(v_a(9 downto 0));
                when 16 =>
                    e_tc <= (others => '0'); e_sel <= (others => '0'); e_srun <= '1';
                when 500 =>
                    n_zt2 <= f_zt(e_hp);
                when others => null;
            end case;

            if e_srun = '1' and e_lc /= 16 then
                if e_tc = 20 then
                    e_tc <= (others => '0');
                    if e_sel = 21 then e_srun <= '0'; else e_sel <= e_sel + 1; end if;
                else
                    e_tc <= e_tc + 1;
                end if;
            end if;
            e_selr <= e_sel;
            if e_srun = '1' and e_tc = 0 then e_s1 <= '1'; else e_s1 <= '0'; end if;
            if e_srun = '1' and e_tc = 1 then e_s2 <= '1'; else e_s2 <= '0'; end if;
            if e_srun = '1' and e_tc = 2 then e_s3 <= '1'; else e_s3 <= '0'; end if;
            if e_srun = '1' and e_tc = 19 then e_s20 <= '1'; else e_s20 <= '0'; end if;
            if e_s1 = '1' then
                case to_integer(e_selr) is
                    when 0 =>
                        e_xa <= resize(e_ady, 32);
                        if e_sdy = '1' then e_xb <= to_unsigned(19373, 16); else e_xb <= to_unsigned(16406, 16); end if;
                    when 1 =>
                        e_xa <= resize(e_ady2, 32);
                        if e_sdy2 = '1' then e_xb <= to_unsigned(18405, 16); else e_xb <= to_unsigned(13345, 16); end if;
                    when 2 => e_xa <= resize(c_ip, 32); e_xb <= resize(e_adv, 16);                  -- window phase
                    when 3 => e_xa <= resize(e_sq(10 downto 0), 32); e_xb <= to_unsigned(1971, 16);  -- (1 - V2) / A
                    when 4 => e_xa <= resize(e_cy, 32); e_xb <= resize(e_cy, 16);                   -- cy^2
                    when 5 => e_xa <= resize(e_sq(23 downto 6), 32); e_xb <= to_unsigned(8389, 16); -- beta
                    when 6 =>
                        if s_xs = 4 then e_xa <= to_unsigned(32515, 32); else e_xa <= to_unsigned(12634, 32); end if;
                        e_xb <= resize(e_cy, 16);
                    when 7 =>
                        if s_xs = 4 then e_xa <= to_unsigned(154056, 32); else e_xa <= to_unsigned(23256, 32); end if;
                        e_xb <= resize(e_be, 16);
                    when 8 =>
                        case to_integer(s_xs) is
                            when 4      => e_xa <= to_unsigned(2651, 32);
                            when 3      => e_xa <= to_unsigned(1988, 32);
                            when others => e_xa <= to_unsigned(1326, 32);
                        end case;
                        e_xb <= resize(e_cy, 16);
                    when 9 =>
                        case to_integer(s_xs) is
                            when 4      => e_xa <= to_unsigned(6296, 32);
                            when 3      => e_xa <= to_unsigned(1839, 32);
                            when others => e_xa <= to_unsigned(1224, 32);
                        end case;
                        e_xb <= resize(e_be, 16);
                    when 10 =>
                        e_xa <= resize(e_be, 32);
                        case to_integer(s_xs) is
                            when 4      => e_xb <= to_unsigned(16, 16);
                            when 3      => e_xb <= to_unsigned(9, 16);
                            when others => e_xb <= to_unsigned(4, 16);
                        end case;
                    when 11 => e_xa <= resize(e_adz, 32); e_xb <= resize(e_adz, 16);                -- zone dy^2
                    when 12 => e_xa <= resize(e_sq(19 downto 3), 32); e_xb <= to_unsigned(28237, 16);
                    when 13 => e_xa <= resize(e_adz, 32); e_xb <= to_unsigned(23718, 16);
                    when 14 =>
                        e_xa <= resize(e_adz, 32);
                        case to_integer(s_xs) is
                            when 4      => e_xb <= to_unsigned(47436, 16);
                            when 3      => e_xb <= to_unsigned(35577, 16);
                            when others => e_xb <= to_unsigned(23718, 16);
                        end case;
                    when 15 => e_xa <= resize(e_ae, 32); e_xb <= resize(c_adx0, 16);
                    when 16 => e_xa <= resize(e_adl, 32); e_xb <= resize(e_adl, 16);                -- lobe
                    when 17 => e_xa <= resize(e_sq(17 downto 0), 32); e_xb <= to_unsigned(77, 16);
                    when 18 => e_xa <= resize(e_adp, 32); e_xb <= resize(e_adp, 16);                -- corner petal
                    when 19 => e_xa <= resize(e_sq(19 downto 4), 32); e_xb <= to_unsigned(16777, 16);
                    when 20 =>      -- stripe phase (y + 4038) / 40.5
                        e_xa <= resize(e_vc, 32); e_xb <= to_unsigned(101, 16);
                    when others =>  -- diagonal cut, h units: (y + 455) / 0.513 * 2
                        e_xa <= resize(e_vc, 32); e_xb <= to_unsigned(250, 16);
                end case;
            end if;
            if e_s3 = '1' then
                m_acc <= (others => '0');
                m_a <= e_xa; m_b <= e_xb;
            else
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= m_a(30 downto 0) & '0';
                m_b <= '0' & m_b(15 downto 1);
            end if;
            -- the power-law ROM data for slots 0 and 1, two clocks after their addresses
            if e_s2 = '1' and e_selr = 1 then
                n_eth <= signed(resize(c_t10, 14)) - signed(resize(r_pw_q, 14));
            end if;
            if e_s2 = '1' and e_selr = 2 then
                if r_pw_q >= 1024 then e_sq <= (others => '0'); else e_sq <= resize(1024 - r_pw_q, 24); end if;
            end if;
            if e_s20 = '1' then
                case to_integer(e_selr) is
                    when 0 =>
                        v_t := m_acc(31 downto 16);
                        if v_t > 255 then r_pw_a <= '0' & x"FF"; else r_pw_a <= '0' & v_t(7 downto 0); end if;
                    when 1 =>
                        v_t := m_acc(31 downto 16);
                        if v_t > 255 then r_pw_a <= '1' & x"FF"; else r_pw_a <= '1' & v_t(7 downto 0); end if;
                    when 2 =>
                        if e_sdv = '1' then n_sy <= unsigned(-signed(m_acc(23 downto 12)));
                        else n_sy <= m_acc(23 downto 12); end if;
                    when 3  => n_wth <= m_acc(20 downto 10);
                    when 4  => e_sq <= m_acc(23 downto 0);
                    when 5  => e_be <= m_acc(29 downto 18);
                    when 6  => n_ph0 <= m_acc(23 downto 0);
                    when 7  => n_ph0 <= n_ph0 + m_acc(27 downto 4);
                    when 8  => n_pd0 <= resize(m_acc(25 downto 4), 22);
                    when 9  => n_pd0 <= n_pd0 + resize(m_acc(27 downto 6), 22);
                    when 10 => n_pdd <= m_acc(16 downto 5);
                    when 11 => e_sq <= m_acc(23 downto 0);
                    when 12 => n_z0 <= resize(c_z0b, 18) - signed(resize(m_acc(31 downto 17), 18));
                    when 13 => e_ae <= m_acc(24 downto 8);
                    when 14 =>
                        if e_sdz = '1' then n_zd0 <= resize(c_zd0, 24) - signed(resize(m_acc(31 downto 8), 24));
                        else n_zd0 <= resize(c_zd0, 24) + signed(resize(m_acc(31 downto 8), 24)); end if;
                    when 15 =>
                        if (e_sdz xor c_dxn) = '1' then n_z0 <= n_z0 - signed(resize(m_acc(27 downto 12), 18));
                        else n_z0 <= n_z0 + signed(resize(m_acc(27 downto 12), 18)); end if;
                    when 16 => e_sq <= m_acc(23 downto 0);
                    when 17 => n_ly <= to_signed(128, 10) - signed(resize(m_acc(25 downto 16), 10));
                    when 18 => e_sq <= m_acc(23 downto 0);
                    when 19 =>      -- 256 - Epy, clamped 0 .. 255
                        v_p := to_signed(256, 11) - signed(resize(m_acc(31 downto 22), 11));
                        if v_p < 0 then n_pth <= (others => '0');
                        elsif v_p > 255 then n_pth <= (others => '1');
                        else n_pth <= unsigned(v_p(7 downto 0)); end if;
                    when 20 => e_hp <= m_acc(11 downto 7);
                    when others => n_cut <= m_acc(18 downto 6); n_by <= e_vy(18 downto 14);
                end case;
            end if;
        end if;
    end process p_line;


    ---------------------------------------------------------------------------
    -- Beam.  Stage 1 counts pixels and the virtual x (line-buffer, x-term ROM
    -- and brightness-grid addresses, the left-half and diagonal-cut flags); the
    -- DDAs run at stage 5.  The engine's next-line values become the active-line
    -- registers at the line start.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px <= (others => '0'); b_xh <= c_x0h;
            elsif g_avid = '1' then
                b_px <= b_px + 1; b_xh <= b_xh + resize(s_xs, 12);
            end if;
            if s_lbh = '1' then s_lb_ra <= b_px(10 downto 1); else s_lb_ra <= b_px(9 downto 0); end if;
            r_x_a <= b_xh(11 downto 2);
            r_lv_a <= l_by & b_xh(11 downto 7);
            if b_px <= c_ctr then p2_lf <= '1'; else p2_lf <= '0'; end if;
            if resize(b_xh, 13) > l_cut then p2_ct <= '1'; else p2_ct <= '0'; end if;
            b_gsr <= b_gsr(2 downto 0) & g_start;

            if b_gsr(3) = '1' then
                b_u <= shift_left(resize(c_u0, 32), 2); b_d <= shift_left(resize(c_d0, 24), 2);
                b_ph <= n_ph0; b_pd <= n_pd0; b_pdd <= n_pdd;
                b_z <= shift_left(resize(n_z0, 30), 12); b_zd <= n_zd0;
                l_eth <= n_eth; l_wth <= n_wth; l_sy <= n_sy; l_ly <= n_ly; l_pth <= n_pth; l_cut <= n_cut;
                l_zt2 <= n_zt2; l_zt1 <= n_zt2 - 1270; l_by <= n_by;
            elsif s_gav_sr(3) = '1' then
                b_u <= b_u + resize(b_d, 32); b_d <= b_d + resize(c_dd, 24);
                b_ph <= b_ph + resize(b_pd, 24); b_pd <= b_pd + resize(b_pdd, 22);
                b_z <= b_z + resize(b_zd, 30); b_zd <= b_zd + resize(c_zdd, 24);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_h  : unsigned(9 downto 0);
        variable v_z  : signed(17 downto 0);
        variable v_l  : signed(10 downto 0);
        variable v_lw : unsigned(13 downto 0);
        variable v_k  : unsigned(11 downto 0);
        variable v_c  : std_logic;
        variable v_zn : unsigned(1 downto 0);
        variable v_zr : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            -- 2 .. 5: ROM words and flags carried to the beam stage
            p3_lf <= p2_lf; p4_lf <= p3_lf; p5_lf <= p4_lf; q6_lf <= p5_lf;
            p3_ct <= p2_ct; p4_ct <= p3_ct; p5_ct <= p4_ct; q6_ct <= p5_ct;
            p3_xr <= r_x_q; p4_xr <= p3_xr; p5_xr <= p4_xr;
            p3_lv <= r_lv_q; p4_lv <= p3_lv; p5_lv <= p4_lv; p6_lv <= p5_lv; p7_lv <= p6_lv; p8_lv <= p7_lv; p9_lv <= p8_lv;
            -- 3: luma from the line buffer (6 bits); 4: x warp; 5: Osc at four times the depth
            p3_lm <= unsigned(s_lb_q(7 downto 2));
            p4_lw <= p3_lm * c_gw;
            if s_src = '1' then p5_lw <= shift_left(resize(p4_lw, 17), 2); else p5_lw <= resize(p4_lw, 17); end if;
            -- 6: cell phase (Chevron: reversed on the left half) -> petal ROM; fan + camera; zone and lobe
            --    levels; corner petal; card threshold + camera
            v_h := unsigned(b_u(25 downto 16));
            if s_chv = '1' and q6_lf = '1' then v_h := not v_h; end if;
            r_eh_a <= v_h(9 downto 2);
            p6_h8 <= v_h(9 downto 2);
            p6_un <= b_u(31);
            p6_ph <= b_ph;
            p6_l15 <= resize(p5_lw, 18) + resize(p5_lw(16 downto 1), 18);
            v_z := b_z(29 downto 12);
            if v_z < -8192 then p6_dz <= to_signed(-8192, 14); elsif v_z > 8191 then p6_dz <= to_signed(8191, 14); else p6_dz <= v_z(13 downto 0); end if;
            v_l := resize(l_ly, 11) - signed(resize(p5_xr(15 downto 8), 11));
            if v_l < -256 then p6_dl <= to_signed(-8192, 14); else p6_dl <= shift_left(resize(v_l, 14), 5); end if;
            if p5_xr(7 downto 0) < l_pth and q6_ct = '1' then p6_pt <= '1'; else p6_pt <= '0'; end if;
            p6_et <= resize(l_eth, 15) + signed(resize(p5_lw(16 downto 3), 15));
            v_lw := resize(p5_lw(16 downto 1), 14) + resize(p5_lw(16 downto 3), 14);
            if v_lw > 8191 then p6_zl <= to_unsigned(8191, 13); else p6_zl <= v_lw(12 downto 0); end if;
            -- 7: window product h x |m / P|; fan phase; zone max
            p7_kp <= p6_h8 * c_ks;
            p7_ph <= p6_ph + shift_left(resize(p6_l15, 24), 12);
            if p6_dz > p6_dl then p7_zm <= p6_dz; else p7_zm <= p6_dl; end if;
            p7_un <= p6_un; p7_et <= p6_et; p7_zl <= p6_zl; p7_pt <= p6_pt;
            -- 8: petal ROM (fabric copy); window phase; stripe phase; zone level
            p8_eh <= r_eh_q;
            v_k := p7_kp(13 downto 2);
            if c_ksn = '1' then p8_s <= l_sy + v_k; else p8_s <= l_sy - v_k; end if;
            p8_sx <= p7_ph(23 downto 21);
            v_zr := resize(p7_zm, 15) + signed(resize(p7_zl, 15));
            if v_zr > 4095 then p8_zr <= to_signed(4095, 13); elsif v_zr < -4096 then p8_zr <= to_signed(-4096, 13); else p8_zr <= v_zr(12 downto 0); end if;
            p8_un <= p7_un; p8_et <= p7_et; p8_pt <= p7_pt;
            -- 9: petal shape; window distance; zone + stripe push
            if signed(resize(p8_eh, 15)) < p8_et and p8_un = '0' then p9_sh <= '1'; else p9_sh <= '0'; end if;
            if p8_s(11) = '1' then p9_w <= not p8_s(10 downto 0); else p9_w <= p8_s(10 downto 0); end if;
            p9_zr <= resize(p8_zr, 14) + resize(f_gsr(p8_sx), 14);
            p9_sx <= p8_sx; p9_pt <= p8_pt;
            -- 10: keys: card = petal shape inside the window, or the corner petal; zone levels
            if p9_sh = '1' and p9_w < l_wth then v_c := '1'; else v_c := '0'; end if;
            p10_card <= v_c or (p9_pt and not s_pof);
            if p9_zr > resize(l_zt2, 14) then v_zn := "10";
            elsif p9_zr > resize(l_zt1, 14) then v_zn := "01";
            else v_zn := "00"; end if;
            if s_zof = '1' then v_zn := "00"; end if;
            p10_zone <= v_zn; p10_sx <= p9_sx; p10_lv <= p9_lv;
            -- 11: palette addresses
            r_pa <= s_pal & p10_zone & p10_sx & p10_lv;
            r_ca <= s_pal & p10_zone & p10_sx & p10_lv(3 downto 2);
            p11_card <= p10_card;
            -- 12: (ROM read)
            p12_card <= p11_card;
            -- 13: fabric copies
            p13_y <= r_py_q; p13_u <= r_pu_q; p13_v <= r_pv_q;
            p13_cy <= unsigned(r_cq(23 downto 16)); p13_cu <= unsigned(r_cq(15 downto 8)); p13_cv <= unsigned(r_cq(7 downto 0));
            p13_card <= p12_card;
            -- 14: card over ground
            if p13_card = '1' then
                p14_y <= p13_cy & "00"; p14_u <= p13_cu & "00"; p14_v <= p13_cv & "00";
            else
                p14_y <= p13_y & "00"; p14_u <= p13_u & "00"; p14_v <= p13_v & "00";
            end if;
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

end architecture confetti;
