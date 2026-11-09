-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Temple v0.1.0 -- Heirloom from the video clips, built from the still WIN_20230816_00_32_31 t5.0
-- (a 12 s clip: one knob-like sweep moves the colouriser through the palette -- the sky corners go orange ->
-- yellow -> white -> cyan, the bottom cycles white -> cyan -> blue -> black -> red, the spindles' arrow wings
-- grow into needles and the dark ring at the top sinks -- while the pink band and the dark shapes hold).
--
-- The still: an orange/yellow/blue mirror-symmetric "sunset temple".  A sky of orange and yellow with four
-- soft cyan flame columns, maroon shapes with black rims (a ring, a crooked centre block, two small blobs,
-- two lens ellipses, a band), a salmon-pink band holding four orange spindles with yellow cores, arrow wings
-- and torn white rims, a stack of wavy purple / pink / red / yellow bands with blue rims, a jagged white rim,
-- and a cyan bottom with darker columns and a yellow half-dome.
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
--
--   K1  Columns   column spacing (flames, spindles, stack pinches, small shapes): x0.5 .. x2 (512 = the still)
--   K2  Spindles  the spindles' size x0.3 .. x2
--   K3  Flames    the H-waveform depth of the sky and bottom (0 = plain gradients, 512 = the still, x2)
--   K4  Drift     the sweep LFO rate: 0 = still, 512 = the clip (8 s), 1023 = 4 s
--   K5  Stack     the stack's pinch depth: 0 = flat bands, 512 = the still, x4 = deep lenses
--   K6  Camera    how far the camera's light (or the Osc lattice) bends the fields
--   S7  Flip      upright / upside down
--   S8  Palette   Sunset / Ice (Cr and Cb swapped)
--   S9  Shadows   On (the still) / Off: the band and the dark shapes gone, the pink band reaching the sky
--   S10 Tear      Off (the still's faint line jitter) / On (sync-tear bursts)
--   S11 Source    Video / Osc
--   P12 Sun       the sweep position: dusk (0) .. the still (512) .. blaze (1023)
--
-- Engine: vblank-zone sequencer (shared glide, knob curves, the LFO), a per-line engine kicked at each line
-- start computing the NEXT line (table reads, four serial products: the VCA gain, the stack ramp, the spindle
-- size and the beam's DDA start), a 14-stage pixel path ending in a 1024-entry palette, then the shell's output
-- conditioning (Y, U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps sit in block-RAM rings,
-- luma 13/16, blanking gate).  Virtual 1080-line coordinates (SD shows the centre 4:3).  Hardware colours.

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
    signal s_flip, s_ice, s_shad, s_tear, s_src : std_logic := '0';
    signal z_gp_c : std_logic_vector(1 downto 0) := "00";
    signal z_gc_c : unsigned(2 downto 0) := (others => '0');
    signal z_grun_c : std_logic := '0';

    -- per-field constants
    signal c_k1   : unsigned(8 downto 0) := to_unsigned(128, 9);     -- columns, Q7
    signal c_k2   : unsigned(8 downto 0) := to_unsigned(128, 9);     -- spindle size, Q7
    signal c_k3   : unsigned(8 downto 0) := to_unsigned(256, 9);     -- flame depth, Q8
    signal c_k5   : unsigned(9 downto 0) := to_unsigned(256, 10);    -- stack pinch, Q8
    signal c_k6   : unsigned(7 downto 0) := to_unsigned(50, 8);      -- camera gain, Q9
    signal c_dcp  : signed(7 downto 0) := (others => '0');           -- sun position (+-48)
    signal c_dc   : signed(7 downto 0) := (others => '0');           -- sun + LFO
    signal c_lfo  : unsigned(15 downto 0) := x"4000";
    signal c_lst  : unsigned(9 downto 0) := to_unsigned(136, 10);
    signal c_tri  : signed(6 downto 0) := (others => '0');
    signal c_ycd  : unsigned(10 downto 0) := to_unsigned(688, 11);   -- stack centre line
    signal c_ro   : signed(9 downto 0) := (others => '0');           -- ring row offset
    signal c_dk   : unsigned(10 downto 0) := to_unsigned(256, 11);   -- DDA step per pixel (k1 xs)
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);

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
    signal e_bc   : unsigned(5 downto 0) := (others => '0');          -- tear burst lines left
    signal e_bo   : signed(9 downto 0) := (others => '0');            -- tear burst offset (h)
    signal e_bm   : unsigned(8 downto 0) := (others => '0');          -- tear burst magnitude (h)
    signal e_ox   : signed(9 downto 0) := (others => '0');
    signal e_vl   : unsigned(10 downto 0) := (others => '0');
    signal e_v    : unsigned(10 downto 0) := (others => '0');         -- the line's virtual row (after Flip)
    signal e_ri   : unsigned(8 downto 0) := (others => '0');
    signal e_vr   : signed(11 downto 0) := (others => '0');
    signal e_w0, e_w1 : std_logic_vector(15 downto 0) := (others => '0');
    signal e_dv   : signed(11 downto 0) := (others => '0');
    signal e_xb   : signed(12 downto 0) := (others => '0');
    signal e_sg   : std_logic := '0';
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(9 downto 0) := (others => '0');
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
    signal p6_p1  : signed(17 downto 0) := (others => '0');
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
                s_tear <= registers_in(6)(3);
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

            -- knob curves and the sweep: zc 40 .. 60 (one small operation per clock)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>                                        -- columns: 64 + k/8 | 128 + (k - 512)/4
                        v_k := s_g1(12 downto 3);
                        if v_k(9) = '0' then c_k1 <= resize(64 + shift_right(v_k, 3), 9);
                        else c_k1 <= resize(128 + shift_right(v_k - 512, 2), 9); end if;
                    when 41 =>                                        -- spindles: 38 + 90 k / 512 | 128 + (k - 512)/4
                        v_k := s_g2(12 downto 3);
                        if v_k(9) = '0' then
                            v_t := resize(shift_right(v_k, 3), 13) + resize(shift_right(v_k, 5), 13) + resize(shift_right(v_k, 6), 13)
                                 + resize(shift_right(v_k, 9), 13);
                            c_k2t <= v_t;                              -- 38 + k (1/8 + 1/32 + 1/64 + 1/512) = 38 + 0.174 k
                        else
                            c_k2t <= resize(90 + shift_right(v_k - 512, 2), 13);
                        end if;
                    when 42 =>                                        -- flames: k / 2
                        c_k3 <= resize(shift_right(s_g3(12 downto 3), 1), 9);
                    when 43 =>                                        -- stack: k / 2 | 256 + 3 (k - 512) / 2
                        v_k := s_g5(12 downto 3);
                        if v_k(9) = '0' then c_k5 <= resize(shift_right(v_k, 1), 10);
                        else
                            v_t := resize(v_k - 512, 13);
                            c_k5 <= resize(256 + v_t + shift_right(v_t, 1), 10);
                        end if;
                    when 44 =>                                        -- camera: k / 4
                        c_k6 <= resize(shift_right(s_g6(12 downto 3), 2), 8);
                    when 45 =>                                        -- LFO rate: k/4 + k/64 (0 below 16)
                        v_k := s_g4(12 downto 3);
                        if v_k < 16 then c_lst <= (others => '0');
                        else c_lst <= resize(shift_right(v_k, 2), 10) + resize(shift_right(v_k, 6), 10); end if;
                    when 46 =>                                        -- sun: 3 (p - 512) / 32
                        v_s := signed(resize(s_gp(12 downto 3), 13)) - 512;
                        v_s := shift_left(v_s, 1) + v_s;
                        c_dcp <= resize(shift_right(v_s, 5), 8);
                    when 47 =>
                        c_lfo <= c_lfo + resize(c_lst, 16);
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
                    when 52 =>
                        c_k2 <= resize(38 + c_k2t, 9);
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
    -- 0), computes the NEXT line.  Table reads, then serial products (16
    -- iterations, result stable from load + 17); one small operation per clock.
    -- ~110 clocks: fits SD's 858-clock line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_vf  : signed(11 downto 0);
        variable v_r   : unsigned(15 downto 0);
        variable v_ri  : unsigned(14 downto 0);
        variable v_s   : signed(12 downto 0);
        variable v_g   : signed(9 downto 0);
        variable v_u   : unsigned(10 downto 0);
        variable v_h   : signed(11 downto 0);
        variable v_p   : signed(23 downto 0);
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
            m_b <= '0' & m_b(9 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_vl <= to_unsigned(1079, 11); else e_vl <= e_vy(18 downto 8); end if;
                when 2 =>
                    if s_flip = '1' then e_v <= 1079 - e_vl; else e_v <= e_vl; end if;
                when 3 =>
                    v_ri := resize(e_v, 15);
                    v_ri := shift_left(v_ri, 4) - v_ri;                   -- 15 v
                    e_ri <= v_ri(13 downto 5);
                    e_vr <= signed(resize(e_v, 12)) - resize(c_ro, 12);
                when 4 =>
                    r_r_a <= e_ri & '0';
                    r_d_a <= e_v(8 downto 1);
                when 5 =>
                    r_r_a <= e_ri & '1';
                    if e_vr >= 0 and e_vr < 110 then r_d_a <= unsigned(e_vr(8 downto 1)); n_ron <= s_shad;
                    else n_ron <= '0'; end if;
                when 6 =>
                    e_w0 <= r_r_q;
                    -- blob row (span A only below row 200: the ring has its own read)
                    if e_v(10 downto 9) = "00" and s_shad = '1' then
                        n_la <= unsigned(r_d_q(8 downto 0)); n_ra <= unsigned(r_d_q(17 downto 9));
                        n_lb <= unsigned(r_d_q(24 downto 18)); n_rb <= unsigned(r_d_q(31 downto 25));
                        if e_v >= 200 then n_aon <= '1'; else n_aon <= '0'; end if;
                        n_bon <= '1';
                    else
                        n_aon <= '0'; n_bon <= '0';
                    end if;
                when 7 =>
                    e_w1 <= r_r_q;
                    n_lr <= unsigned(r_d_q(8 downto 0)); n_rr <= unsigned(r_d_q(17 downto 9));
                when 8 =>
                    n_zone <= unsigned(e_w0(15 downto 14));
                    n_a    <= unsigned(e_w0(6 downto 0));
                    n_aux  <= unsigned(e_w0(13 downto 7));
                    n_gp   <= unsigned(e_w0(13 downto 7));
                    e_hw   <= unsigned(e_w0(6 downto 0));
                    n_kind <= unsigned(e_w1(11 downto 10));
                    n_kk   <= unsigned(e_w1(5 downto 0));
                    n_band <= e_w1(6) and s_shad;
                    n_bsh  <= unsigned(e_w1(11 downto 7));
                    e_ww   <= unsigned(e_w1(13 downto 12));
                    n_prow <= e_w1(14);
                    n_v    <= e_v;
                    n_pv   <= e_lfsr(5 downto 4);
                    -- the VCA gain g (signed, Q2) times Flames (Q8): magnitude product
                    v_g := signed(e_w1(9 downto 0));
                    e_sg <= v_g(9);
                    m_a <= resize(unsigned(abs(v_g)), 24); m_b <= resize(c_k3, 10); m_acc <= (others => '0');
                    -- the stack ramp (v - centre)
                    e_dv <= signed(resize(e_v, 12)) - signed(resize(c_ycd, 12));
                    e_bm <= 80 + shift_left(resize(e_lfsr(14 downto 9), 9), 2) + resize(e_lfsr(14 downto 9), 9);
                when 9 =>
                    -- tear / jitter offset (h) for this line, from the line's LFSR
                    v_r := e_lfsr;
                    if s_tear = '1' then
                        if e_bc /= 0 then
                            e_bc <= e_bc - 1;
                            e_ox <= e_bo + signed(resize(v_r(1 downto 0), 10)) - 2;
                        elsif v_r(2 downto 0) = "000" then
                            e_bc <= resize(v_r(8 downto 4), 6) + 7;
                            if v_r(15) = '1' then
                                e_bo <= -signed(resize(e_bm, 10)); e_ox <= -signed(resize(e_bm, 10));
                            else
                                e_bo <= signed(resize(e_bm, 10)); e_ox <= signed(resize(e_bm, 10));
                            end if;
                        else
                            e_ox <= (others => '0');
                        end if;
                    else
                        e_bc <= (others => '0');
                        if v_r(2 downto 0) = "000" then
                            if v_r(7) = '1' then e_ox <= -signed(resize(v_r(6 downto 4), 10)) - 2;
                            else e_ox <= signed(resize(v_r(6 downto 4), 10)) + 2; end if;
                        else
                            e_ox <= (others => '0');
                        end if;
                    end if;
                when 10 =>
                    e_xb <= signed(resize(c_xb0, 13)) + resize(e_ox, 13);
                when 26 =>
                    if e_sg = '1' then n_gk <= -signed(resize(m_acc(18 downto 8), 11));
                    else n_gk <= signed(resize(m_acc(18 downto 8), 11)); end if;
                    -- the stack ramp times Stack (Q8)
                    m_a <= resize(unsigned(abs(e_dv)), 24); m_b <= c_k5; m_acc <= (others => '0');
                    e_sg <= e_dv(11);
                when 44 =>
                    if m_acc(23 downto 19) /= "00000" then v_u := (others => '1'); else v_u := m_acc(18 downto 8); end if;
                    if e_sg = '1' then n_e <= -signed(resize(v_u, 12)); else n_e <= signed(resize(v_u, 12)); end if;
                    -- the spindle size (Q7)
                    m_a <= resize(e_hw, 24); m_b <= resize(c_k2, 10); m_acc <= (others => '0');
                when 62 =>
                    e_hk <= resize(m_acc(15 downto 7), 9);
                    -- the beam's DDA start: (xb - 1920) k1
                    v_h := resize(e_xb, 12) - 1920;
                    e_sg <= v_h(11);
                    m_a <= resize(unsigned(abs(v_h)), 24); m_b <= resize(c_k1, 10); m_acc <= (others => '0');
                when 63 =>
                    -- wings: hk + 3/2 dc ww, at least 5/8 hk
                    v_s := signed(resize(e_hk, 13));
                    case to_integer(e_ww) is
                        when 1      => v_s := v_s + shift_right(shift_left(resize(c_dc, 13), 1) + resize(c_dc, 13), 1);
                        when 2      => v_s := v_s + shift_left(resize(c_dc, 13), 1) + resize(c_dc, 13);
                        when 3      => v_s := v_s + shift_left(resize(c_dc, 13), 2) + shift_right(resize(c_dc, 13), 1);
                        when others => null;
                    end case;
                    e_dv <= resize(v_s, 12);
                    e_lo <= e_hk - shift_right(e_hk, 2) - shift_right(e_hk, 3);
                when 64 =>
                    if e_dv < signed(resize(e_lo, 12)) then
                        n_hwe <= e_lo;
                    elsif e_dv > 511 then
                        n_hwe <= (others => '1');
                    else
                        n_hwe <= unsigned(e_dv(8 downto 0));
                    end if;
                when 80 =>
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
            p4_cam <= resize(p3_cam(15 downto 9), 8);
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
            -- 6: the two products (VCA gain x waveform or spindle slope x distance; stack ramp x (M - 1))
            if l_zone = 1 then
                p6_p1 <= signed(resize(l_kk * p5_d, 18));
            else
                p6_p1 <= resize(l_gk * signed(resize(p5_t, 8)), 18);
            end if;
            p6_p2 <= resize(l_e * p5_mm, 19);
            p6_d <= p5_d; p6_spi <= p5_spi;
            p6_rim <= p5_rim; p6_bb <= p5_bb; p6_pb <= p5_pb; p6_cam <= p5_cam;
            p_fl(6) <= x"000" & '0' & p5_spk & p5_dmon & std_logic_vector(p5_dm) & p5_dkon & std_logic_vector(p5_dk)
                       & "0000000";
            -- 7: S (field), the stack row, the spindle G; region decisions
            v_s := signed(resize(l_a, 11)) + resize(shift_right(p6_p1, 9), 11) + signed(resize(p6_cam, 11));
            p7_s <= v_s;
            p7_st <= signed(resize(l_v, 14)) + resize(shift_right(p6_p2, 4), 14) + signed(resize(p6_cam(7 downto 1), 14));
            p7_g2 <= signed(resize(l_gp, 13)) - resize(shift_right(p6_p1, 4), 13);
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
            p8_bpb6 <= p7_bpb6; p8_spi <= p7_spi;
            -- 9: the palette address (priority)
            if p8_f(15) = '1' then
                v_ad := to_unsigned(608, 10) + resize(unsigned(p8_f(14 downto 10)), 10);
            elsif l_zone = 0 then
                v_ad := "000" & p8_fi;
            elsif l_zone = 1 then
                if p8_f(3) = '1' and p8_bpb6 = '1' then
                    v_ad := to_unsigned(512, 10) + resize(p8_gi, 10);
                elsif p8_f(0) = '1' then
                    v_ad := to_unsigned(256, 10) + resize(p8_si, 10);
                elsif p8_bnd = '1' then
                    v_ad := to_unsigned(576, 10) + resize(l_bsh, 10);
                elsif p8_mix = '1' then
                    if p8_mxl = "11" then v_ad := to_unsigned(643, 10); else v_ad := to_unsigned(640, 10) + resize(p8_mxl, 10); end if;
                elsif p8_f(2) = '1' then
                    v_ad := to_unsigned(648, 10) + resize(p8_rd, 10);
                else
                    v_ad := to_unsigned(644, 10) + resize(l_pv, 10);
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
                    else v_ad := to_unsigned(256, 10) + resize(p8_si, 10); end if;
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
