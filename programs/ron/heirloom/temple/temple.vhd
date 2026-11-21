-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
-- 2026-11-15: K6 camera top +25 % (Ron)
-- 2026-11-21: v0.2.0 Ron's preview notes: Pink Height and Flame Height knobs (row remaps), Waves (sky flames
--             + stack pinch merged), wider Columns / Flame Width, Edges switch (replaces Tear), the camera in the
--             pink band (a light ramp) and the flames (hotter where bright); the drift runs at the clip's rate.
--
-- Temple v0.2.0 -- Heirloom from the video clips, built from the still WIN_20230816_00_32_31 t5.0
-- (a 12 s clip: one knob-like sweep moves the colouriser through the palette -- the sky corners go orange ->
-- yellow -> white -> cyan, the bottom cycles white -> cyan -> blue -> black -> red, the spindles' arrow wings
-- grow into needles and the dark ring at the top sinks -- while the pink band and the dark shapes hold).
--
-- The still: an orange/yellow/blue mirror-symmetric "sunset temple".  A sky of orange and yellow with four
-- soft cyan flame columns, maroon shapes with black rims (a ring, a crooked centre block, two small blobs,
-- two lens ellipses, a band), a salmon-pink band holding four orange spindles ("flames") with yellow cores,
-- arrow wings and torn white rims, a stack of wavy purple / pink / red / yellow bands with blue rims, a jagged
-- white rim, and a cyan bottom with darker columns and a yellow half-dome.
--
-- The patch, as fitted (temple.py has every table; .heirloom_work/rb_temple/notes.md the measurements):
--   sky, bottom:  S = a(y) + g(y) T(xc)               a colouriser over one H waveform through a per-line VCA
--                 colour = PAL[(clamp(S, 0, 95) + dc) mod 128]     (dc = the sweep: a palette rotation)
--   stack:        S = y + e(y) (M(xc) - 1)            a vertical ramp through a VCA whose gain is the H
--                                                     waveform M (pinched at the columns, stretched at the
--                                                     centre), colour = the band profile at row S
--   middle:       the band (black rim, maroon shade, soft into pink), the pink band, the spindles
--                 (|xc - c| < hw(y) k2 + wings dc, interior G = gp - kk |dx|, white rim on the pink side)
--   dark shapes:  per-line spans (ring, centre block, small blobs, lens ellipses), depth-shaded maroon
--   xc = |x - 960| k1 (the Columns scale, a per-pixel DDA)
--   rows:         table row y = a piecewise-linear remap of the screen row (Pink Height stretches table rows
--                 468..628, the rest shares the remaining height); the flames read their own row
--                 645 + (v - v645) kf (Flame Height, anchored where the flames meet the stack)
--
--   K1  Columns       column spacing (flames, spindles, stack pinches, small shapes): x0.375 .. x3 (512 = x1)
--   K2  Flame Width   the orange flames' width x0.125 .. x3
--   K3  Flame Height  the orange flames' height x0.15 .. x2.5 (tall flames rise through the band into the sky)
--   K4  Pink Height   the pink band's height 0.2x .. 3.7x (the sky and the lower part share the rest)
--   K5  Waves         the column waveform depth: sky / floor flames (0 .. x2.5) and the stack pinch (0 .. x4)
--   K6  Camera        how far the camera's light (or the Osc lattice) bends the fields, lights the pink band
--                     and heats the flames
--   S7  Flip          upright / upside down
--   S8  Palette       Sunset / Ice (Cr and Cb swapped)
--   S9  Shadows       On (the still) / Off: the band and the dark shapes gone, the pink band reaching the sky
--   S10 Edges         Off (the still) / On: white edges on the dark shapes, the band, the flames, the stack rims
--   S11 Source        Video / Osc
--   P12 Sun           the sweep position: dusk (0) .. the still (512) .. blaze (1023), plus the clip's drift
--
-- Engine: vblank-zone sequencer (shared glide, knob curves, the LFO, a serial divider for the remap slopes),
-- a per-line engine kicked at each line start computing the NEXT line (the two row remaps, table reads, serial
-- products: the VCA gain, the stack ramp, the spindle size and the beam's DDA start), a 14-stage pixel path
-- ending in a 1024-entry palette, then the shell's output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps sit in block-RAM rings, luma 13/16, blanking gate).
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Hardware colours.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.temple_pkg.all;
use work.all;

architecture temple of program_top is
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
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- band bulge (rows below 440) by |x - 960| >> 3
    function f_bbo(i : unsigned(6 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when 0 | 1  => return to_unsigned(27, 5);
            when 2      => return to_unsigned(26, 5);
            when 3      => return to_unsigned(25, 5);
            when 4      => return to_unsigned(24, 5);
            when 5      => return to_unsigned(23, 5);
            when 6      => return to_unsigned(21, 5);
            when 7      => return to_unsigned(19, 5);
            when 8      => return to_unsigned(17, 5);
            when 9      => return to_unsigned(14, 5);
            when 10     => return to_unsigned(11, 5);
            when 11     => return to_unsigned(8, 5);
            when 12     => return to_unsigned(4, 5);
            when 13     => return to_unsigned(1, 5);
            when others => return to_unsigned(0, 5);
        end case;
    end function;

    attribute no_rw_check : boolean;

    -- camera line buffer (every other pixel): (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_c_a  : unsigned(8 downto 0) := (others => '0');
    signal r_c_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_r_a  : unsigned(9 downto 0) := (others => '0');
    signal r_r_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_d_a  : unsigned(7 downto 0) := (others => '0');
    signal r_d_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_pa_a : unsigned(9 downto 0) := to_unsigned(643, 10);
    signal r_py_q, r_pu_q, r_pv_q : std_logic_vector(7 downto 0) := (others => '0');

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
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_flip, s_ice, s_shad, s_edge, s_src : std_logic := '0';
    signal z_gp_c : std_logic_vector(1 downto 0) := "00";
    signal z_gc_c : unsigned(2 downto 0) := (others => '0');
    signal z_grun_c : std_logic := '0';

    -- per-field constants
    signal c_k1   : unsigned(8 downto 0) := to_unsigned(128, 9);     -- columns, Q7
    signal c_k2   : unsigned(8 downto 0) := to_unsigned(128, 9);     -- spindle size, Q7
    signal c_k3   : unsigned(9 downto 0) := to_unsigned(256, 10);    -- sky / floor flame depth, Q8 (Waves)
    signal c_k5   : unsigned(9 downto 0) := to_unsigned(256, 10);    -- stack pinch, Q8
    signal c_k6   : unsigned(7 downto 0) := to_unsigned(25, 8);      -- camera gain, Q8
    signal c_dcp  : signed(7 downto 0) := (others => '0');           -- sun position (+-48)
    signal c_dc   : signed(7 downto 0) := (others => '0');           -- sun + LFO
    signal c_lfo  : unsigned(15 downto 0) := x"4000";
    signal c_tri  : signed(6 downto 0) := (others => '0');
    signal c_ycd  : unsigned(10 downto 0) := to_unsigned(688, 11);   -- stack centre line
    signal c_ro   : signed(9 downto 0) := (others => '0');           -- ring row offset
    signal c_dk   : unsigned(10 downto 0) := to_unsigned(256, 11);   -- DDA step per pixel (k1 xs)
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);
    -- row remaps (Pink Height, Flame Height)
    signal c_pht, c_pha, c_fht, c_fha : unsigned(9 downto 0) := (others => '0');
    signal c_ph    : unsigned(9 downto 0) := to_unsigned(160, 10);    -- pink screen rows (table rows 468..628)
    signal c_fh    : unsigned(9 downto 0) := to_unsigned(256, 10);    -- flame height, Q8
    signal c_om   : unsigned(10 downto 0) := to_unsigned(920, 11);   -- 1080 - P
    signal c_pat  : unsigned(20 downto 0) := (others => '0');
    signal c_ptt  : unsigned(15 downto 0) := (others => '0');
    signal c_pa   : unsigned(10 downto 0) := to_unsigned(468, 11);   -- screen row of table row 468
    signal c_pb   : unsigned(10 downto 0) := to_unsigned(628, 11);   -- screen row of table row 628
    signal c_pt   : unsigned(10 downto 0) := to_unsigned(645, 11);   -- screen row of table row 645 (flame anchor)
    signal c_sp   : unsigned(10 downto 0) := to_unsigned(256, 11);   -- pink slope, table rows per screen row Q8
    signal c_so   : unsigned(10 downto 0) := to_unsigned(256, 11);   -- outside slope
    signal c_kf   : unsigned(10 downto 0) := to_unsigned(256, 11);   -- flame slope
    signal c_w2t, c_w3t : unsigned(11 downto 0) := (others => '0');
    signal c_w1, c_w2, c_w3 : signed(11 downto 0) := (others => '0');   -- wing terms 3dc/2, 3dc, 9dc/2
    signal e_wo   : signed(11 downto 0) := (others => '0');
    -- serial divider (12 quotient bits, vblank only)
    signal d_go   : std_logic := '0';
    signal d_num  : unsigned(17 downto 0) := (others => '0');
    signal d_den  : unsigned(10 downto 0) := to_unsigned(1, 11);
    signal d_rem, d_dsh : unsigned(21 downto 0) := (others => '0');
    signal d_q    : unsigned(11 downto 0) := (others => '0');
    signal d_cnt  : unsigned(3 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal e_ox   : signed(9 downto 0) := (others => '0');
    signal e_vl   : unsigned(10 downto 0) := (others => '0');
    signal e_v    : unsigned(10 downto 0) := (others => '0');         -- the line's virtual row (after Flip)
    signal e_lta, e_ltb, e_rsv : std_logic := '0';
    signal e_da, e_db, e_dt : signed(11 downto 0) := (others => '0');
    signal e_base : unsigned(9 downto 0) := (others => '0');
    signal e_r    : unsigned(10 downto 0) := (others => '0');         -- the line's table row (Pink Height)
    signal e_rs   : unsigned(9 downto 0) := (others => '0');          -- the line's flame row (Flame Height)
    signal e_ris  : unsigned(8 downto 0) := (others => '0');
    signal e_s0, e_s1 : std_logic_vector(15 downto 0) := (others => '0');
    signal e_ri   : unsigned(8 downto 0) := (others => '0');
    signal e_vr   : signed(11 downto 0) := (others => '0');
    signal e_w0, e_w1 : std_logic_vector(15 downto 0) := (others => '0');
    signal e_dv   : signed(11 downto 0) := (others => '0');
    signal e_xb   : signed(12 downto 0) := (others => '0');
    signal e_sg   : std_logic := '0';
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(10 downto 0) := (others => '0');
    signal m_acc  : unsigned(23 downto 0) := (others => '0');
    -- results for the next line
    signal n_zone : unsigned(1 downto 0) := (others => '0');
    signal n_a    : unsigned(6 downto 0) := (others => '0');
    signal n_gk   : signed(10 downto 0) := (others => '0');
    signal n_aux  : unsigned(6 downto 0) := (others => '0');          -- ellipse / sparkle / dome half-width / 2
    signal n_kind : unsigned(1 downto 0) := (others => '0');
    signal n_hwe  : unsigned(8 downto 0) := (others => '0');
    signal n_gp   : unsigned(6 downto 0) := (others => '0');
    signal n_kk   : unsigned(5 downto 0) := (others => '0');
    signal n_band, n_prow : std_logic := '0';
    signal n_bsh  : unsigned(4 downto 0) := (others => '0');
    signal n_pv   : unsigned(1 downto 0) := (others => '0');
    signal n_e    : signed(11 downto 0) := (others => '0');
    signal n_v    : unsigned(10 downto 0) := (others => '0');
    signal n_la, n_ra : unsigned(8 downto 0) := (others => '0');
    signal n_lb, n_rb : unsigned(6 downto 0) := (others => '0');
    signal n_lr, n_rr : unsigned(8 downto 0) := (others => '0');
    signal n_aon, n_bon, n_ron : std_logic := '0';
    signal n_xb   : signed(12 downto 0) := (others => '0');
    signal n_xc   : signed(21 downto 0) := (others => '0');
    signal e_hw   : unsigned(6 downto 0) := (others => '0');
    signal e_ww   : unsigned(1 downto 0) := (others => '0');
    signal e_hk   : unsigned(8 downto 0) := (others => '0');

    -- beam (stage 1) and the line constants
    signal b_x   : signed(12 downto 0) := (others => '0');
    signal b_xc  : signed(21 downto 0) := (others => '0');
    signal b_pc  : unsigned(10 downto 0) := (others => '0');
    signal l_zone : unsigned(1 downto 0) := (others => '0');
    signal l_a    : unsigned(6 downto 0) := (others => '0');
    signal l_gk   : signed(10 downto 0) := (others => '0');
    signal l_aux  : unsigned(6 downto 0) := (others => '0');
    signal l_kind : unsigned(1 downto 0) := (others => '0');
    signal l_hwe  : unsigned(8 downto 0) := (others => '0');
    signal l_gp   : unsigned(6 downto 0) := (others => '0');
    signal l_kk   : unsigned(5 downto 0) := (others => '0');
    signal l_band, l_prow : std_logic := '0';
    signal l_bsh  : unsigned(4 downto 0) := (others => '0');
    signal l_pv   : unsigned(1 downto 0) := (others => '0');
    signal l_e    : signed(11 downto 0) := (others => '0');
    signal l_v    : unsigned(10 downto 0) := (others => '0');
    signal l_la, l_ra : unsigned(8 downto 0) := (others => '0');
    signal l_lb, l_rb : unsigned(6 downto 0) := (others => '0');
    signal l_lr, l_rr : unsigned(8 downto 0) := (others => '0');
    signal l_aon, l_bon, l_ron : std_logic := '0';

    -- pixel path
    signal p2_vx  : unsigned(10 downto 0) := (others => '0');
    signal p2_xc  : unsigned(10 downto 0) := (others => '0');
    signal p2_lm  : unsigned(7 downto 0) := (others => '0');
    signal p3_xc  : unsigned(10 downto 0) := (others => '0');
    signal p3_xf  : unsigned(9 downto 0) := (others => '0');
    signal p3_dx1, p3_dx2 : unsigned(10 downto 0) := (others => '0');
    signal p3_al, p3_ar, p3_rl, p3_rr : signed(11 downto 0) := (others => '0');
    signal p3_bl, p3_br : signed(11 downto 0) := (others => '0');
    signal p3_ex, p3_dx : unsigned(10 downto 0) := (others => '0');
    signal p4_dx1 : unsigned(10 downto 0) := (others => '0');
    signal e_lo   : unsigned(8 downto 0) := (others => '0');
    signal c_k2t  : unsigned(12 downto 0) := (others => '0');
    signal p3_cam : unsigned(15 downto 0) := (others => '0');
    signal p4_dxs : unsigned(10 downto 0) := (others => '0');
    signal p4_da, p4_db, p4_dr : signed(11 downto 0) := (others => '0');
    signal p4_ed, p4_dd : signed(11 downto 0) := (others => '0');
    signal p4_bbo : unsigned(4 downto 0) := (others => '0');
    signal p4_cam : unsigned(7 downto 0) := (others => '0');
    signal p5_d   : unsigned(9 downto 0) := (others => '0');
    signal p5_spk : std_logic := '0';
    signal p5_t   : unsigned(6 downto 0) := (others => '0');
    signal p5_mm  : signed(6 downto 0) := (others => '0');
    signal p5_rim : unsigned(10 downto 0) := (others => '0');
    signal p5_bb, p5_pb : unsigned(10 downto 0) := (others => '0');
    signal p5_dk  : unsigned(4 downto 0) := (others => '0');          -- dark-shape depth
    signal p5_dkon : std_logic := '0';
    signal p5_dm  : unsigned(3 downto 0) := (others => '0');          -- dome depth
    signal p5_dmon : std_logic := '0';
    signal p5_cam : unsigned(7 downto 0) := (others => '0');
    signal p5_d7  : unsigned(6 downto 0) := (others => '0');
    signal p6_p1  : signed(17 downto 0) := (others => '0');
    signal p6_p3  : unsigned(12 downto 0) := (others => '0');          -- spindle slope x distance
    signal p6_gc  : unsigned(7 downto 0) := (others => '0');           -- spindle peak + camera / 2
    signal p7_pk, p8_pk : unsigned(4 downto 0) := (others => '0');     -- pink camera ramp index
    signal p8_dkw, p8_dk3, p8_erim, p8_bw : std_logic := '0';
    signal p6_p2  : signed(18 downto 0) := (others => '0');
    signal p6_d   : unsigned(9 downto 0) := (others => '0');
    type t_fl6 is array (6 to 8) of std_logic_vector(31 downto 0);
    signal p_fl   : t_fl6 := (others => (others => '0'));
    signal p6_rim, p6_bb, p6_pb : unsigned(10 downto 0) := (others => '0');
    signal p6_cam : unsigned(7 downto 0) := (others => '0');
    signal p7_s   : signed(10 downto 0) := (others => '0');
    signal p7_st  : signed(13 downto 0) := (others => '0');
    signal p7_g2  : signed(12 downto 0) := (others => '0');
    signal p7_bpb6 : std_logic := '0';
    signal p8_rd  : unsigned(2 downto 0) := (others => '0');
    signal p8_rimln, p8_bnd, p8_mix, p8_bpb6 : std_logic := '0';
    signal p8_rlv, p8_mxl : unsigned(1 downto 0) := (others => '0');
    signal p5_spi, p6_spi, p7_spi, p8_spi : std_logic := '0';
    signal p7_body, p7_rimz : std_logic := '0';
    signal p7_rd  : unsigned(2 downto 0) := (others => '0');
    signal p7_lowst, p7_rimln, p7_stk, p7_bnd, p7_mix : std_logic := '0';
    signal p7_rlv  : unsigned(1 downto 0) := (others => '0');
    signal p7_mxl  : unsigned(1 downto 0) := (others => '0');
    signal p8_fi  : unsigned(6 downto 0) := (others => '0');          -- field palette index (0..127)
    signal p8_si  : unsigned(7 downto 0) := (others => '0');          -- stack profile index
    signal p8_gi  : unsigned(5 downto 0) := (others => '0');          -- spindle ramp index
    signal p8_f   : std_logic_vector(15 downto 0) := (others => '0');
    signal p11_y, p11_u, p11_v : unsigned(7 downto 0) := (others => '0');
    signal p12_y, p12_u, p12_v : unsigned(7 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(7 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(7 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

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
            r_c_q <= C_TP_C(to_integer(r_c_a));
            r_r_q <= C_TP_R(to_integer(r_r_a));
            r_d_q <= C_TP_D(to_integer(r_d_a));
        end if;
    end process p_rom;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            r_py_q <= C_TP_PY(to_integer(r_pa_a));
            r_pu_q <= C_TP_PU(to_integer(r_pa_a));
            r_pv_q <= C_TP_PV(to_integer(r_pa_a));
        end if;
    end process p_pal;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u15(3 downto 0)) & std_logic_vector(s_v15) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u17(3 downto 0)) & std_logic_vector(s_v17) & "00";
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field constants (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k  : unsigned(9 downto 0);
        variable v_t  : unsigned(12 downto 0);
        variable v_s  : signed(12 downto 0);
        variable v_d  : signed(9 downto 0);
        variable v_tr : unsigned(5 downto 0);
        variable v_u9 : unsigned(8 downto 0);
        variable v_u8 : unsigned(7 downto 0);
        variable v_w  : unsigned(20 downto 0);
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
                s_flip <= registers_in(6)(0);
                s_ice  <= registers_in(6)(1);
                s_shad <= registers_in(6)(2);
                s_edge <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, seven channels
            if s_zgo = '1' then
                z_gp_c <= "00"; z_gc_c <= "000"; z_grun_c <= '1';
            elsif z_grun_c = '1' then
                case z_gp_c is
                    when "00" =>
                        case to_integer(z_gc_c) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 5      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp_c <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp_c <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp_c <= "10";
                    when others =>
                        case to_integer(z_gc_c) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 5      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp_c <= "00";
                        if z_gc_c = 6 then z_grun_c <= '0'; end if;
                        z_gc_c <= z_gc_c + 1;
                end case;
            end if;

            -- knob curves, the sweep, the row-remap constants: zc 40 .. 108 (one small operation per clock)
            d_go <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>                                        -- Columns: 48 + 5k/32 | 128 + (k - 512)/2
                        v_k := s_g1(12 downto 3);
                        if v_k(9) = '0' then
                            c_k1 <= resize(48 + resize(shift_right(v_k, 3), 9) + resize(shift_right(v_k, 5), 9), 9);
                        else c_k1 <= resize(128 + shift_right(v_k - 512, 1), 9); end if;
                    when 41 =>                                        -- Flame Width: 16 + 7k/32 | 128 + (k - 512)/2
                        v_k := s_g2(12 downto 3);
                        if v_k(9) = '0' then
                            v_t := resize(shift_right(v_k, 3), 13) + resize(shift_right(v_k, 4), 13) + resize(shift_right(v_k, 5), 13);
                            c_k2t <= v_t;
                        else
                            c_k2t <= resize(112 + shift_right(v_k - 512, 1), 13);
                        end if;
                    when 42 =>                                        -- Waves, sky / floor: k/2 | 256 + 3 (k - 512)/4
                        v_k := s_g5(12 downto 3);
                        if v_k(9) = '0' then c_k3 <= resize(shift_right(v_k, 1), 10);
                        else
                            v_u9 := resize(v_k - 512, 9);
                            c_k3 <= 256 + resize(shift_right(v_u9, 1), 10) + resize(shift_right(v_u9, 2), 10);
                        end if;
                    when 43 =>                                        -- Waves, stack: k / 2 | 256 + 3 (k - 512) / 2
                        v_k := s_g5(12 downto 3);
                        if v_k(9) = '0' then c_k5 <= resize(shift_right(v_k, 1), 10);
                        else
                            v_t := resize(v_k - 512, 13);
                            c_k5 <= resize(256 + v_t + shift_right(v_t, 1), 10);
                        end if;
                    when 44 =>                                        -- camera: (k + k/4) / 8, Q8 (K6 top +25 %)
                        c_k6 <= resize(shift_right(resize(s_g6(12 downto 3), 11) + resize(s_g6(12 downto 5), 11), 3), 8);
                    when 45 =>                                        -- Flame Height, Q8: 38 + 0.426 k | 256 + 3 (k - 512)/4
                        v_k := s_g3(12 downto 3);
                        if v_k(9) = '0' then
                            c_fht <= shift_right(v_k, 2) + shift_right(v_k, 3) + shift_right(v_k, 5);
                            c_fha <= 38 + shift_right(v_k, 6) + shift_right(v_k, 8);
                        else
                            v_u9 := resize(v_k - 512, 9);
                            c_fht <= resize(shift_right(v_u9, 1), 10) + resize(shift_right(v_u9, 2), 10);
                            c_fha <= to_unsigned(256, 10);
                        end if;
                    when 46 =>                                        -- sun: (3p - 1536) / 32 = 3 (p - 512) / 32
                        -- (3p on the unsigned knob: no sign-extended self-add, which hung router1)
                        v_t := shift_left(resize(s_gp(12 downto 3), 13), 1) + resize(s_gp(12 downto 3), 13);
                        v_s := signed(v_t) - 1536;
                        c_dcp <= resize(shift_right(v_s, 5), 8);
                    when 47 =>                                        -- the drift: the clip's 8 s triangle
                        c_lfo <= c_lfo + 136;
                    when 48 =>
                        v_tr := c_lfo(14 downto 9);
                        if c_lfo(15) = '0' then c_tri <= signed(resize(v_tr, 7)) - 32;
                        else c_tri <= 31 - signed(resize(v_tr, 7)); end if;
                    when 49 =>
                        c_dc <= c_dcp + resize(c_tri, 8);
                    when 50 =>
                        c_ycd <= to_unsigned(688, 11) - unsigned(resize(shift_right(c_dc, 1), 11));
                        c_ro  <= shift_left(resize(c_dc, 10), 2);
                    when 51 =>
                        c_dk <= resize(c_k1 * s_xs, 11);
                    when 52 =>                                        -- Pink Height (screen rows): 32 + k/4 | 160 + 0.84 (k - 512)
                        c_k2 <= resize(16 + c_k2t, 9);
                        v_k := s_g4(12 downto 3);
                        if v_k(9) = '0' then
                            c_pht <= shift_right(v_k, 2);
                            c_pha <= to_unsigned(32, 10);
                        else
                            v_u9 := resize(v_k - 512, 9);
                            c_pht <= resize(shift_right(v_u9, 1), 10) + resize(shift_right(v_u9, 2), 10);
                            c_pha <= 160 + resize(shift_right(v_u9, 4), 10) + resize(shift_right(v_u9, 5), 10);
                        end if;
                    when 53 =>
                        c_ph <= c_pht + c_pha;
                        c_fh <= c_fht + c_fha;
                    when 54 =>
                        c_om <= to_unsigned(1080, 11) - resize(c_ph, 11);
                    when 55 =>                                        -- A = 468 (1080 - P) / 920 ~ (1080 - P) 521 / 1024
                        c_pat <= shift_left(resize(c_om, 21), 9) + shift_left(resize(c_om, 21), 3);
                    when 56 =>
                        v_w := c_pat + resize(c_om, 21);
                        c_pa <= v_w(20 downto 10);
                        c_ptt <= shift_left(resize(c_om, 16), 4) + shift_left(resize(c_om, 16), 1);
                    when 57 =>                                        -- B = A + P; 17 (1080 - P) / 920 ~ (1080 - P) 19 / 1024
                        c_pb <= c_pa + resize(c_ph, 11);
                        c_ptt <= c_ptt + resize(c_om, 16);
                    when 58 =>
                        c_pt <= c_pb + resize(c_ptt(15 downto 10), 11);
                    when 60 =>                                        -- slopes (Q8): 160 * 256 / P, 920 * 256 / (1080 - P), 65536 / F
                        d_num <= to_unsigned(40960, 18); d_den <= resize(c_ph, 11); d_go <= '1';
                    when 76 =>
                        c_sp <= resize(d_q, 11);
                        d_num <= to_unsigned(235520, 18); d_den <= c_om; d_go <= '1';
                    when 92 =>
                        c_so <= resize(d_q, 11);
                        d_num <= to_unsigned(65536, 18); d_den <= resize(c_fh, 11); d_go <= '1';
                    when 62 =>                                        -- wing terms from u = dc + 128 (offset binary)
                        v_u8 := unsigned(c_dc); v_u8(7) := not v_u8(7);
                        c_w2t <= shift_left(resize(v_u8, 12), 1) + resize(v_u8, 12);              -- 3u
                        c_w3t <= shift_left(resize(v_u8, 12), 2) + resize(shift_right(v_u8, 1), 12); -- 4u + u/2
                    when 63 =>
                        c_w2 <= signed(c_w2t) - 384;                  -- 3 dc
                        c_w3 <= signed(c_w3t) - 576;                  -- 4 dc + dc/2
                    when 64 =>
                        c_w1 <= shift_right(c_w2, 1);                 -- 3 dc / 2
                    when 108 =>
                        c_kf <= resize(d_q, 11);
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    -- serial restoring divider: d_q = d_num / d_den (12 quotient bits; every use has a quotient < 4096)
    p_div : process(clk)
        variable v_df : unsigned(22 downto 0);
    begin
        if rising_edge(clk) then
            if d_go = '1' then
                d_rem <= resize(d_num, 22);
                d_dsh <= shift_left(resize(d_den, 22), 11);
                d_q   <= (others => '0');
                d_cnt <= to_unsigned(12, 4);
            elsif d_cnt /= 0 then
                v_df := ('0' & d_rem) - ('0' & d_dsh);
                if v_df(22) = '0' then
                    d_rem <= v_df(21 downto 0);
                    d_q <= d_q(10 downto 0) & '1';
                else
                    d_q <= d_q(10 downto 0) & '0';
                end if;
                d_dsh <= shift_right(d_dsh, 1);
                d_cnt <= d_cnt - 1;
            end if;
        end if;
    end process p_div;

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
    -- write of (luma >> 2) - 16 on every other pixel
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
            s_lb_we <= data_in.avid and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
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
    -- 0), computes the NEXT line.  The two row remaps, table reads, then
    -- serial products (11 iterations, read 18 clocks after the load); one
    -- small operation per clock.  ~125 clocks: fits SD's 858-clock line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ri  : unsigned(14 downto 0);
        variable v_r   : unsigned(15 downto 0);
        variable v_s   : signed(12 downto 0);
        variable v_g   : signed(9 downto 0);
        variable v_u   : unsigned(10 downto 0);
        variable v_h   : signed(11 downto 0);
        variable v_p   : signed(23 downto 0);
        variable v_t   : unsigned(13 downto 0);
        variable v_rs  : signed(15 downto 0);
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
                e_lfsr <= e_lfsr(14 downto 0) & (e_lfsr(15) xor e_lfsr(13) xor e_lfsr(12) xor e_lfsr(10));
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(22 downto 0) & '0';
            m_b <= '0' & m_b(10 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_vl <= to_unsigned(1079, 11); else e_vl <= e_vy(18 downto 8); end if;
                when 2 =>
                    if s_flip = '1' then e_v <= 1079 - e_vl; else e_v <= e_vl; end if;
                when 3 =>
                    if e_v < c_pa then e_lta <= '1'; else e_lta <= '0'; end if;
                    if e_v < c_pb then e_ltb <= '1'; else e_ltb <= '0'; end if;
                    e_da <= signed(resize(e_v, 12)) - signed(resize(c_pa, 12));
                    e_db <= signed(resize(e_v, 12)) - signed(resize(c_pb, 12));
                    e_dt <= signed(resize(e_v, 12)) - signed(resize(c_pt, 12));
                when 4 =>
                    -- Pink Height: table row = base + (v - segment start) x slope (Q8)
                    m_acc <= (others => '0');
                    if e_lta = '1' then
                        e_base <= (others => '0'); m_a <= resize(e_v, 24); m_b <= c_so;
                    elsif e_ltb = '1' then
                        e_base <= to_unsigned(468, 10); m_a <= resize(unsigned(e_da(10 downto 0)), 24); m_b <= c_sp;
                    else
                        e_base <= to_unsigned(628, 10); m_a <= resize(unsigned(e_db(10 downto 0)), 24); m_b <= c_so;
                    end if;
                when 22 =>
                    v_t := resize(e_base, 14) + resize(m_acc(20 downto 8), 14);
                    if v_t > 1079 then e_r <= to_unsigned(1079, 11); else e_r <= v_t(10 downto 0); end if;
                    -- Flame Height: flame row = 645 + (v - v645) x kf (Q8)
                    m_a <= resize(unsigned(abs(e_dt)), 24); m_b <= c_kf; m_acc <= (others => '0');
                    e_sg <= e_dt(11);
                when 40 =>
                    if e_sg = '1' then v_rs := 645 - signed(resize(m_acc(21 downto 8), 16));
                    else v_rs := 645 + signed(resize(m_acc(21 downto 8), 16)); end if;
                    if v_rs >= 0 and v_rs < 1024 then e_rsv <= '1'; else e_rsv <= '0'; end if;
                    e_rs <= unsigned(v_rs(9 downto 0));
                when 43 =>
                    v_ri := resize(e_r, 15);
                    v_ri := shift_left(v_ri, 4) - v_ri;                   -- 15 r
                    e_ri <= v_ri(13 downto 5);
                    e_vr <= signed(resize(e_r, 12)) - resize(c_ro, 12);
                    v_ri := resize(e_rs, 15);
                    v_ri := shift_left(v_ri, 4) - v_ri;
                    e_ris <= v_ri(13 downto 5);
                when 44 =>
                    r_r_a <= e_ri & '0';
                    r_d_a <= e_r(8 downto 1);
                when 45 =>
                    r_r_a <= e_ri & '1';
                    if e_vr >= 0 and e_vr < 110 then r_d_a <= unsigned(e_vr(8 downto 1)); n_ron <= s_shad;
                    else n_ron <= '0'; end if;
                when 46 =>
                    e_w0 <= r_r_q;
                    r_r_a <= e_ris & '0';
                    -- blob row (span A only below row 200: the ring has its own read)
                    if e_r(10 downto 9) = "00" and s_shad = '1' then
                        n_la <= unsigned(r_d_q(8 downto 0)); n_ra <= unsigned(r_d_q(17 downto 9));
                        n_lb <= unsigned(r_d_q(24 downto 18)); n_rb <= unsigned(r_d_q(31 downto 25));
                        if e_r >= 200 then n_aon <= '1'; else n_aon <= '0'; end if;
                        n_bon <= '1';
                    else
                        n_aon <= '0'; n_bon <= '0';
                    end if;
                when 47 =>
                    e_w1 <= r_r_q;
                    r_r_a <= e_ris & '1';
                    n_lr <= unsigned(r_d_q(8 downto 0)); n_rr <= unsigned(r_d_q(17 downto 9));
                when 48 =>
                    e_s0 <= r_r_q;
                    n_zone <= unsigned(e_w0(15 downto 14));
                    n_a    <= unsigned(e_w0(6 downto 0));
                    n_aux  <= unsigned(e_w0(13 downto 7));
                    n_kind <= unsigned(e_w1(11 downto 10));
                    n_band <= e_w1(6) and s_shad;
                    n_bsh  <= unsigned(e_w1(11 downto 7));
                    n_prow <= e_w1(14);
                    n_v    <= e_r;
                    n_pv   <= e_lfsr(5 downto 4);
                    -- the VCA gain g (signed, Q2) times Flames (Q8): magnitude product
                    v_g := signed(e_w1(9 downto 0));
                    e_sg <= v_g(9);
                    m_a <= resize(unsigned(abs(v_g)), 24); m_b <= resize(c_k3, 11); m_acc <= (others => '0');
                    -- the stack ramp (r - centre)
                    e_dv <= signed(resize(e_r, 12)) - signed(resize(c_ycd, 12));
                when 49 =>
                    e_s1 <= r_r_q;
                    -- the still's faint line jitter (h) for this line, from the line's LFSR
                    v_r := e_lfsr;
                    if v_r(2 downto 0) = "000" then
                        if v_r(7) = '1' then e_ox <= -signed(resize(v_r(6 downto 4), 10)) - 2;
                        else e_ox <= signed(resize(v_r(6 downto 4), 10)) + 2; end if;
                    else
                        e_ox <= (others => '0');
                    end if;
                when 50 =>
                    e_xb <= signed(resize(c_xb0, 13)) + resize(e_ox, 13);
                    -- the flame words: valid only where the flame row lands in the middle zone
                    if e_rsv = '1' and e_s0(15 downto 14) = "01" then
                        e_hw <= unsigned(e_s0(6 downto 0)); n_gp <= unsigned(e_s0(13 downto 7));
                        n_kk <= unsigned(e_s1(5 downto 0)); e_ww <= unsigned(e_s1(13 downto 12));
                    else
                        e_hw <= (others => '0'); n_gp <= (others => '0');
                        n_kk <= (others => '0'); e_ww <= (others => '0');
                    end if;
                when 51 =>
                    case to_integer(e_ww) is
                        when 1      => e_wo <= c_w1;
                        when 2      => e_wo <= c_w2;
                        when 3      => e_wo <= c_w3;
                        when others => e_wo <= (others => '0');
                    end case;
                when 66 =>
                    if e_sg = '1' then n_gk <= -signed(resize(m_acc(18 downto 8), 11));
                    else n_gk <= signed(resize(m_acc(18 downto 8), 11)); end if;
                    -- the stack ramp times Stack (Q8)
                    m_a <= resize(unsigned(abs(e_dv)), 24); m_b <= resize(c_k5, 11); m_acc <= (others => '0');
                    e_sg <= e_dv(11);
                when 84 =>
                    if m_acc(23 downto 19) /= "00000" then v_u := (others => '1'); else v_u := m_acc(18 downto 8); end if;
                    if e_sg = '1' then n_e <= -signed(resize(v_u, 12)); else n_e <= signed(resize(v_u, 12)); end if;
                    -- the spindle size (Q7)
                    m_a <= resize(e_hw, 24); m_b <= resize(c_k2, 11); m_acc <= (others => '0');
                when 102 =>
                    e_hk <= resize(m_acc(15 downto 7), 9);
                    -- the beam's DDA start: (xb - 1920) k1
                    v_h := resize(e_xb, 12) - 1920;
                    e_sg <= v_h(11);
                    m_a <= resize(unsigned(abs(v_h)), 24); m_b <= resize(c_k1, 11); m_acc <= (others => '0');
                when 103 =>
                    -- wings: hk + 3/2 dc ww, at least 5/8 hk
                    v_s := signed(resize(e_hk, 13)) + resize(e_wo, 13);
                    e_dv <= resize(v_s, 12);
                    e_lo <= e_hk - shift_right(e_hk, 2) - shift_right(e_hk, 3);
                when 104 =>
                    if e_dv < signed(resize(e_lo, 12)) then
                        n_hwe <= e_lo;
                    elsif e_dv > 511 then
                        n_hwe <= (others => '1');
                    else
                        n_hwe <= unsigned(e_dv(8 downto 0));
                    end if;
                when 120 =>
                    v_p := signed(resize(m_acc(22 downto 0), 24));
                    if e_sg = '1' then n_xc <= resize(-v_p, 22); else n_xc <= resize(v_p, 22); end if;
                    n_xb <= e_xb;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in h, the Columns DDA xc (h Q7); the line constants
    -- loaded at the line start; the camera read issued
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= n_xb;
                b_xc <= n_xc;
                b_pc <= (others => '0');
                l_zone <= n_zone; l_a <= n_a; l_gk <= n_gk; l_aux <= n_aux; l_kind <= n_kind;
                l_hwe <= n_hwe; l_gp <= n_gp; l_kk <= n_kk; l_band <= n_band; l_prow <= n_prow;
                l_bsh <= n_bsh; l_pv <= n_pv; l_e <= n_e; l_v <= n_v;
                l_la <= n_la; l_ra <= n_ra; l_lb <= n_lb; l_rb <= n_rb; l_lr <= n_lr; l_rr <= n_rr;
                l_aon <= n_aon; l_bon <= n_bon; l_ron <= n_ron;
            elsif g_avid = '1' then
                b_x  <= b_x + signed(resize(s_xs, 13));
                b_xc <= b_xc + signed(resize(c_dk, 22));
                b_pc <= b_pc + 1;
            end if;
            s_lb_ra <= b_pc(10 downto 1);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ax  : signed(12 downto 0);
        variable v_xa  : signed(21 downto 0);
        variable v_vx  : signed(11 downto 0);
        variable v_a   : signed(11 downto 0);
        variable v_mn  : signed(11 downto 0);
        variable v_d   : signed(11 downto 0);
        variable v_t   : unsigned(10 downto 0);
        variable v_cw  : std_logic_vector(31 downto 0);
        variable v_m   : unsigned(5 downto 0);
        variable v_s   : signed(10 downto 0);
        variable v_st  : signed(13 downto 0);
        variable v_f   : std_logic_vector(31 downto 0);
        variable v_ad  : unsigned(9 downto 0);
        variable v_i7  : unsigned(6 downto 0);
        variable v_ri  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: virtual x, the scaled fold xc, the camera luma
            v_ax := b_x;
            if v_ax < 0 then p2_vx <= (others => '0'); else p2_vx <= unsigned(v_ax(11 downto 1)); end if;
            v_xa := abs(b_xc);
            if v_xa(21 downto 19) /= "000" then p2_xc <= (others => '1'); else p2_xc <= unsigned(v_xa(18 downto 8)); end if;
            p2_lm <= unsigned(s_lb_q);
            -- 3: column ROM address; distances; span edges; the camera product
            if p2_xc(10) = '1' then r_c_a <= (others => '1'); else r_c_a <= p2_xc(9 downto 1); end if;
            p3_xc <= p2_xc;
            v_vx := signed(resize(p2_vx, 12)) - 960;
            v_a := abs(v_vx); p3_xf <= unsigned(v_a(9 downto 0));
            v_a := signed(resize(p2_xc, 12)) - 424; v_a := abs(v_a); p3_dx1 <= unsigned(v_a(10 downto 0));
            v_a := signed(resize(p2_xc, 12)) - 196; v_a := abs(v_a); p3_dx2 <= unsigned(v_a(10 downto 0));
            v_vx := signed(resize(p2_vx, 12)) - 768;
            p3_al <= v_vx - signed(resize(l_la, 12));
            p3_ar <= signed(resize(l_ra, 12)) - v_vx;
            p3_rl <= v_vx - signed(resize(l_lr, 12));
            p3_rr <= signed(resize(l_rr, 12)) - v_vx;
            v_a := signed(resize(p2_xc, 12)) - 200;
            p3_bl <= v_a - signed(resize(l_lb, 12));
            p3_br <= signed(resize(l_rb, 12)) - v_a;
            v_a := signed(resize(p2_xc, 12)) - 742; v_a := abs(v_a); p3_ex <= unsigned(v_a(10 downto 0));
            v_vx := signed(resize(p2_vx, 12)) - 985; v_vx := abs(v_vx); p3_dx <= unsigned(v_vx(10 downto 0));
            p3_cam <= p2_lm * c_k6;
            -- 4: spindle distance (the nearer of the two; the inner one scaled 1/0.7), dark-shape depth, bulge
            v_t := p3_dx2 + shift_right(p3_dx2, 2) + shift_right(p3_dx2, 3) + shift_right(p3_dx2, 4);
            p4_dxs <= v_t; p4_dx1 <= p3_dx1;
            if p3_al < p3_ar then p4_da <= p3_al; else p4_da <= p3_ar; end if;
            if p3_bl < p3_br then p4_db <= p3_bl; else p4_db <= p3_br; end if;
            if p3_rl < p3_rr then p4_dr <= p3_rl; else p4_dr <= p3_rr; end if;
            p4_ed <= signed(resize(l_aux & '0', 12)) - signed(resize(p3_ex, 12));
            p4_dd <= signed(resize(l_aux & '0', 12)) - signed(resize(p3_dx, 12));
            p4_bbo <= f_bbo(p3_xf(9 downto 3));
            p4_cam <= p3_cam(15 downto 8);                    -- c_k6 is Q8 since the K6 +25 % pass (max 158)
            -- 5: the column word (fabric copy of the ROM read) and its fields; shape flags
            v_cw := r_c_q;
            if l_zone = 0 then p5_t <= unsigned(v_cw(6 downto 0)); else p5_t <= unsigned(v_cw(13 downto 7)); end if;
            if l_v < c_ycd then v_m := unsigned(v_cw(19 downto 14)); else v_m := unsigned(v_cw(25 downto 20)); end if;
            p5_mm <= signed(resize(v_m, 7)) - 16;
            p5_rim <= to_unsigned(772, 11) + shift_left(resize(unsigned(v_cw(31 downto 26)), 11), 1);
            p5_bb <= to_unsigned(440, 11) + resize(p4_bbo, 11);
            p5_pb <= to_unsigned(645, 11) - resize(shift_right(shift_left(resize(p4_bbo, 9), 3) + resize(p4_bbo, 9), 4), 11);
            if p4_dx1 < p4_dxs then v_t := p4_dx1; else v_t := p4_dxs; end if;
            if v_t(10) = '1' then p5_d <= (others => '1'); else p5_d <= v_t(9 downto 0); end if;
            if v_t(10 downto 7) /= "0000" then p5_d7 <= (others => '1'); else p5_d7 <= v_t(6 downto 0); end if;
            if (l_kind = 1 or l_kind = 2) and (p4_dx1 < l_aux & '0' or p4_dxs < l_aux & '0') then p5_spk <= '1'; else p5_spk <= '0'; end if;
            if p4_dx1 < l_aux or p4_dxs < l_aux then p5_spi <= '1'; else p5_spi <= '0'; end if;
            -- dark shapes: the deepest wins (A centre block / B small blobs / ring / ellipse in sky rows)
            if l_aon = '1' and p4_da > 0 then v_mn := p4_da;
            elsif l_ron = '1' and p4_dr > 0 then v_mn := p4_dr;
            elsif l_bon = '1' and p4_db > 0 then v_mn := p4_db;
            elsif l_zone = 0 and s_shad = '1' and l_aux /= 0 and p4_ed > 0 then v_mn := p4_ed;
            else v_mn := (others => '0'); end if;
            if v_mn > 0 then
                p5_dkon <= '1';
                if v_mn > 63 then p5_dk <= (others => '1'); else p5_dk <= unsigned(v_mn(5 downto 1)); end if;
            else
                p5_dkon <= '0'; p5_dk <= (others => '0');
            end if;
            if l_zone = 2 and l_kind = 3 and p4_dd > 0 then
                p5_dmon <= '1';
                if p4_dd > 127 then p5_dm <= (others => '1'); else p5_dm <= unsigned(p4_dd(6 downto 3)); end if;
            else
                p5_dmon <= '0'; p5_dm <= (others => '0');
            end if;
            p5_cam <= p4_cam;
            -- 6: the three products (VCA gain x waveform; stack ramp x (M - 1); spindle slope x distance), the
            --    spindle peak plus half the camera (the flames burn hotter where the picture is bright)
            p6_p1 <= resize(l_gk * signed(resize(p5_t, 8)), 18);
            p6_p2 <= resize(l_e * p5_mm, 19);
            p6_p3 <= l_kk * p5_d7;
            p6_gc <= resize(l_gp, 8) + resize(p5_cam(7 downto 1), 8);
            p6_d <= p5_d; p6_spi <= p5_spi;
            p6_rim <= p5_rim; p6_bb <= p5_bb; p6_pb <= p5_pb; p6_cam <= p5_cam;
            p_fl(6) <= x"000" & '0' & p5_spk & p5_dmon & std_logic_vector(p5_dm) & p5_dkon & std_logic_vector(p5_dk)
                       & "0000000";
            -- 7: S (field), the stack row, the spindle G; region decisions
            v_s := signed(resize(l_a, 11)) + resize(shift_right(p6_p1, 9), 11) + signed(resize(p6_cam, 11));
            p7_s <= v_s;
            p7_st <= signed(resize(l_v, 14)) + resize(shift_right(p6_p2, 4), 14) + signed(resize(p6_cam(7 downto 1), 14));
            p7_g2 <= signed(resize(p6_gc, 13)) - signed(resize(shift_right(p6_p3, 4), 13));
            if p6_cam(7) = '1' then p7_pk <= (others => '1'); else p7_pk <= p6_cam(6 downto 2); end if;
            if l_v < p6_pb + 6 then p7_bpb6 <= '1'; else p7_bpb6 <= '0'; end if;
            if signed(resize(p6_d, 11)) < signed(resize(l_hwe, 11)) and l_hwe /= 0 then p7_body <= '1'; else p7_body <= '0'; end if;
            v_d := signed(resize(p6_d, 12)) - signed(resize(l_hwe, 12));
            if v_d >= 0 and v_d < 8 and l_hwe > 4 then p7_rimz <= '1'; else p7_rimz <= '0'; end if;
            p7_rd <= unsigned(v_d(2 downto 0));
            if l_v + 3 < p6_rim then p7_lowst <= '1'; else p7_lowst <= '0'; end if;
            v_ri := signed(resize(l_v, 12)) - signed(resize(p6_rim, 12));
            if v_ri >= -4 and v_ri <= 4 and l_v > 780 then p7_rimln <= '1'; else p7_rimln <= '0'; end if;
            v_ri := abs(v_ri);
            if v_ri(2 downto 1) = "00" then p7_rlv <= "11"; elsif v_ri(2 downto 1) = "01" then p7_rlv <= "10"; else p7_rlv <= "01"; end if;
            if l_v >= p6_pb then p7_stk <= '1'; else p7_stk <= '0'; end if;
            if l_band = '1' and l_v + 3 < p6_bb then p7_bnd <= '1'; else p7_bnd <= '0'; end if;
            if l_band = '1' and l_v + 3 >= p6_bb and l_v < p6_bb + 3 then p7_mix <= '1'; else p7_mix <= '0'; end if;
            v_ri := signed(resize(l_v, 12)) - signed(resize(p6_bb, 12)) + 3;
            p7_mxl <= unsigned(v_ri(2 downto 1));
            p_fl(7) <= p_fl(6); p7_spi <= p6_spi;
            -- 8: clamps and the sun's palette rotation
            if p7_s < 0 then v_i7 := (others => '0'); elsif p7_s > 95 then v_i7 := to_unsigned(95, 7); else v_i7 := unsigned(p7_s(6 downto 0)); end if;
            p8_fi <= v_i7 + unsigned(c_dc(6 downto 0));
            if p7_st < 600 then p8_si <= (others => '0'); elsif p7_st > 855 then p8_si <= (others => '1');
            else v_st := p7_st - 600; p8_si <= unsigned(v_st(7 downto 0)); end if;
            if p7_g2 < 0 then p8_gi <= (others => '0'); elsif p7_g2 > 127 then p8_gi <= (others => '1'); else p8_gi <= unsigned(p7_g2(6 downto 1)); end if;
            v_f := p_fl(7);
            -- p8_f: 15 dkon, 14..10 dk, 9 dmon, 8..5 dm, 4 spk, 3 body, 2 rimz, 1 lowst, 0 stk
            p8_f <= v_f(12) & v_f(11 downto 7) & v_f(17) & v_f(16 downto 13) & v_f(18) & p7_body & p7_rimz & p7_lowst & p7_stk;
            p8_rd <= p7_rd; p8_rimln <= p7_rimln; p8_rlv <= p7_rlv; p8_bnd <= p7_bnd; p8_mix <= p7_mix; p8_mxl <= p7_mxl;
            p8_bpb6 <= p7_bpb6; p8_spi <= p7_spi; p8_pk <= p7_pk;
            -- Edges: white for the dark shapes' rims (depth < 8 px), the band's top rim, the flames' rim
            if s_edge = '1' and v_f(11 downto 9) = "000" then p8_dkw <= '1'; else p8_dkw <= '0'; end if;
            if v_f(8 downto 7) = "11" then p8_dk3 <= '1'; else p8_dk3 <= '0'; end if;
            p8_erim <= s_edge and p7_rimz;
            if s_edge = '1' and l_bsh < 3 then p8_bw <= '1'; else p8_bw <= '0'; end if;
            -- 9: the palette address (priority)
            if p8_f(15) = '1' then
                if p8_dkw = '1' then
                    if p8_dk3 = '1' then v_ad := to_unsigned(674, 10); else v_ad := to_unsigned(675, 10); end if;
                else
                    v_ad := to_unsigned(608, 10) + resize(unsigned(p8_f(14 downto 10)), 10);
                end if;
            elsif l_zone = 0 then
                if p8_f(3) = '1' then                             -- tall flames rising into the sky
                    v_ad := to_unsigned(512, 10) + resize(p8_gi, 10);
                elsif p8_erim = '1' then
                    v_ad := to_unsigned(680, 10) + resize(p8_rd, 10);
                else
                    v_ad := "000" & p8_fi;
                end if;
            elsif l_zone = 1 then
                if p8_f(3) = '1' and p8_bpb6 = '1' then
                    v_ad := to_unsigned(512, 10) + resize(p8_gi, 10);
                elsif p8_erim = '1' and p8_bpb6 = '1' then
                    v_ad := to_unsigned(680, 10) + resize(p8_rd, 10);
                elsif p8_f(0) = '1' then
                    v_ad := "01" & p8_si; v_ad(9) := s_edge;      -- 256 + si, Edges: 768 + si (white rims)
                elsif p8_bnd = '1' then
                    if p8_bw = '1' then v_ad := to_unsigned(675, 10);
                    else v_ad := to_unsigned(576, 10) + resize(l_bsh, 10); end if;
                elsif p8_mix = '1' then
                    if p8_mxl = "11" then v_ad := to_unsigned(643, 10);
                    elsif s_edge = '1' then
                        if p8_mxl = "01" then v_ad := to_unsigned(675, 10); else v_ad := to_unsigned(673, 10); end if;
                    else v_ad := to_unsigned(640, 10) + resize(p8_mxl, 10); end if;
                elsif p8_f(2) = '1' then
                    v_ad := to_unsigned(648, 10) + resize(p8_rd, 10);
                elsif p8_pk = 0 then
                    v_ad := to_unsigned(644, 10) + resize(l_pv, 10);
                else
                    v_ad := to_unsigned(688, 10) + resize(p8_pk, 10);  -- the camera lights the pink band
                end if;
            else
                if p8_f(9) = '1' then
                    v_ad := to_unsigned(656, 10) + resize(unsigned(p8_f(8 downto 5)), 10);
                elsif p8_rimln = '1' then
                    v_ad := to_unsigned(672, 10) + resize(p8_rlv, 10);
                elsif p8_f(1) = '1' then
                    if p8_f(4) = '1' and l_kind = 1 then
                        if p8_spi = '1' then v_ad := to_unsigned(675, 10); else v_ad := to_unsigned(673, 10); end if;
                    elsif p8_f(4) = '1' and l_kind = 2 then
                        if p8_spi = '1' then v_ad := to_unsigned(678, 10); else v_ad := to_unsigned(676, 10); end if;
                    else v_ad := "01" & p8_si; v_ad(9) := s_edge; end if;
                else
                    v_ad := to_unsigned(128, 10) + resize(p8_fi, 10);
                end if;
            end if;
            r_pa_a <= v_ad;
            -- 10: palette data (EBR); 11: fabric copy
            p11_y <= unsigned(r_py_q); p11_u <= unsigned(r_pu_q); p11_v <= unsigned(r_pv_q);
            -- 12, 13, 14: pads (one latency, a multiple of 4)
            p12_y <= p11_y; p12_u <= p11_u; p12_v <= p11_v;
            p13_y <= p12_y; p13_u <= p12_u; p13_v <= p12_v;
            p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            -- 15: Ice (Cr <-> Cb), black outside the generated avid (the box input)
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= p14_y & "00";
                if s_ice = '1' then s_u15 <= p14_v & "00"; s_v15 <= p14_u & "00";
                else s_u15 <= p14_u & "00"; s_v15 <= p14_v & "00"; end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
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
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 5;
            sy_ra <= rg_wa - (C_LAT - 3);
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- box A: stage 16 sums, stage 17 output (the running sums snap to neutral after 8 blank inputs)
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y15 - s_cay;
                s_bau <= s_bau + s_u15 - s_cau;
                s_bav <= s_bav + s_v15 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
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
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y17 - s_cby;
                s_bbu <= s_bbu + s_u17 - s_cbu;
                s_bbv <= s_bbv + s_v17 - s_cbv;
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
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture temple;
