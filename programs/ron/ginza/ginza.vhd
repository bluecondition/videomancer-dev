-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- ginza.vhd  (v0.4)
--
-- GINZA: the live picture reprinted as a 1930s Japanese commercial-art
-- poster.  Five flat inks (cream paper, dusty pink, sky blue, grainy olive,
-- navy).  The picture is cut into a grid of tall cells: mid-tone cells are
-- thin striped PILLARS (blue | cream with a stacked column of navy kanji |
-- pink), dark cells are cream TEXT BARS that launch horizontal navy BEAMS
-- with cream ABCDEFGHI across them, darker-mid cells are big flat textured
-- OLIVE rectangles behind everything.  Every pillar, bar and beam is built
-- into an axonometric prism with a pink cap and a navy or blue side.  A
-- smaller copy of the layout sits in the background; a blue disc of cream
-- kanji and a thick pink ring stamp the layout; the plates misregister.
--
-- The poster is STILL: every cell's luma is smoothed across frames (one
-- pole, ~16 frames) and its ink only changes when the smoothed luma is well
-- past a cut (hysteresis), so video noise never flips a pillar.
--
-- ARCHITECTURE (no framebuffer; five line/grid RAMs, every pixel one clock)
--
--   Analysis    luma is box-averaged over 8 px and cut into five ink bands
--               (Ink biases the cuts; olive takes the widest band).
--   Cell memory a 2048 x 16 grid RAM (16 rows x 128 columns of cells) holds
--               {ink, luma} per cell.  On each row's LAST line (the sample
--               line for the row below) and the frame's first line, every
--               cell start reads its word, blends the fresh 8-px sample in
--               (luma += (sample - luma) / 16), re-classifies with +-24
--               hysteresis against the stored ink and writes the word back
--               (pipelined read-modify-write, cells >= 17 px apart).
--   Cells       the screen is a grid of cw x 8b cells (Grid; Thin = b wide,
--               Wide = 2b).  The row's inks go into a 2048 x 4 line RAM (and
--               a twin for the beams) on the row's sample line; every line
--               of the row reads them.  Row 0's inks and the echo grid are
--               rebuilt from the cell memory by a vblank walker.
--   Caps        the first K lines of each row are the top face: the RAM is
--               read at x - (K - j) * slant on line j, shearing the cell's
--               outline into a parallelogram leaning into the depth
--               direction.  Caps do not restart the extrusion.
--   Beams       a navy cell launches a beam: on the beam's lines (a band of
--               hb lines at mid-row) a countdown of 3 cw px runs from the
--               cell's left edge in the twin RAM, read at x - (Kb - j) *
--               slant on the beam's own cap lines.
--   Extrusion   a 2048 x 16 line RAM carries {ink, age} per column.  Each
--               line reads the previous line's word at column x + slant: a
--               front-face pixel (pillar, bar, beam) restarts age = Depth; a
--               paper/olive pixel with age left inherits the ink and paints
--               the SIDE ink (age - 1).  Column x is written 5 clocks after
--               its read was issued and slant is at most 4, so no read meets
--               a write.
--   Olive       ground: drawn only where nothing else is, never caps or
--               extrudes, stippled by the grain.
--   Echo        a 64 x 32 x 3-bit grid RAM, rebuilt every vblank from the
--               cell memory, drawn at half size in the upper right BEHIND
--               the live structures, as flat rectangles with half-depth
--               extrusions.
--   Lettering   three glyph ROM reads (ginza.py).  Pillar centre stripes
--               and text bars: a 32-px column of navy kanji (one per glyph
--               row, stacked).  Beams: cream Latin across.  Disc: 128-px
--               kanji in cream on blue.
--   Grain       two static (x, y) hashes, fine (2 px) and coarse (8 px),
--               averaged: a clumpy printed stipple of paper holes in olive
--               (full) and on the faces (quarter), fixed to the screen.
--   Circles     r^2 by second differences for two circles: a solid blue disc
--               of cream kanji at (0.30 W, 0.42 H) and a thick pink ring at
--               (0.20 W, 0.80 H); radius from Circles, ring = 0.44..0.63 R.
--   Plates      the final ink stream runs through a 17-deep shift register;
--               the blue plate is tapped +k and the pink plate -k (Register),
--               navy/olive on the centre tap, composited in print order with
--               a violet overprint where blue and pink overlap.  Night swaps
--               paper and navy after the composite.
--   Output      six-entry YUV palette (BT.601, swapped at compose), then
--               damier's HD HDMI conditioning doubled: [1 4 6 4 1] on Y/U/V
--               and luma to 64 + 13/16 (y - 64), gated to neutral outside
--               the avid.
--
--   CLEAN AVID: the program draws its own colour, so it generates its own
--   avid (aurora/cubist recipe: parity-locked start, field-latched width)
--   and blanks to 64/512/512 outside it.
--
-- CONTROLS
--   K1  Ink        shifts all band cuts: more paper <-> more navy
--   K2  Grid       cell size, 17 .. 122 px at HD
--   K3  Slant      depth direction, -4 .. +4 px per line (0 = straight)
--   K4  Register   plate misregistration, 0 .. 8 px
--   K5  Circles    disc and ring radius, 0 .. half the screen height
--   K6  Grain      stipple density in olive and on the faces
--   S7  Pillars    Thin / Wide
--   S8  Echo       background copy Off / On
--   S9  Faces      Flat / Striped (4-line ledges on the sides)
--   S10 Lettering  Off / On
--   S11 Paper      Cream / Night (paper and navy swapped)
--   P12 DEPTH      extrusion length: a shadow line -> half the screen
--
-- Latency: 40 clocks, constant across all modes.  EBR: 23.  No DSP.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.ginza_glyphs_pkg.all;

architecture ginza of program_top is
    -- data_in -> data_out latency, a multiple of 4 as the SDK requires; must
    -- match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 40;

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_HYST : signed(11 downto 0) := to_signed(24, 12);

    -- ink classes (ascending darkness for the luma ladder)
    constant K_PAPER  : unsigned(2 downto 0) := "000";
    constant K_PINK   : unsigned(2 downto 0) := "001";
    constant K_BLUE   : unsigned(2 downto 0) := "010";
    constant K_OLIVE  : unsigned(2 downto 0) := "011";
    constant K_NAVY   : unsigned(2 downto 0) := "100";
    constant K_VIOLET : unsigned(2 downto 0) := "101";

    -- front kinds for the print stage
    constant F_PLAIN  : unsigned(1 downto 0) := "00";
    constant F_PILLAR : unsigned(1 downto 0) := "01";
    constant F_BAR    : unsigned(1 downto 0) := "10";
    constant F_BEAM   : unsigned(1 downto 0) := "11";

    -- side (extrusion) ink and cap (top face) ink per front ink: every
    -- prism shows three different inks
    function f_side(k : unsigned(2 downto 0)) return unsigned is
    begin
        case k is
            when K_NAVY  => return K_BLUE;
            when K_PAPER => return K_PAPER;
            when others  => return K_NAVY;
        end case;
    end function;
    function f_cap(k : unsigned(2 downto 0)) return unsigned is
    begin
        case k is
            when K_PINK  => return K_BLUE;
            when K_PAPER => return K_PAPER;
            when others  => return K_PINK;
        end case;
    end function;
    -- ink from four "luma >= cut" flags (cut 4 brightest)
    function f_band(g1, g2, g3, g4 : std_logic) return unsigned is
    begin
        if    g4 = '1' then return K_PAPER;
        elsif g3 = '1' then return K_PINK;
        elsif g2 = '1' then return K_BLUE;
        elsif g1 = '1' then return K_OLIVE;
        else                return K_NAVY;
        end if;
    end function;

    -- ==== timing / measurement ====
    signal s_timing : t_video_timing_port;
    signal s_hcnt, s_meas_h : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt, s_meas_v : unsigned(11 downto 0) := (others => '0');
    signal s_fh    : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_ilace : std_logic := '0';
    signal s_botf  : std_logic := '0';
    signal s_hd    : std_logic := '1';     -- wide raster: glyph px 2/8, grain 2
    signal s_saw, s_vs_go : std_logic := '0';

    -- ==== controls ====
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6, s_p12r : unsigned(9 downto 0) := (others => '0');
    signal s_sw : std_logic_vector(4 downto 0) := (others => '0');
    signal s_bias : signed(11 downto 0) := (others => '0');
    signal s_t1, s_t2, s_t3, s_t4 : signed(11 downto 0) := (others => '0');
    signal s_t1b, s_t2b, s_t3b, s_t4b : signed(11 downto 0) := (others => '0');  -- cut - 24
    signal s_t1d, s_t2d, s_t3d, s_t4d : signed(11 downto 0) := (others => '0');  -- cut + 24
    signal s_k3x9 : unsigned(13 downto 0) := (others => '0');
    signal s_slant_n, s_slant : signed(3 downto 0) := (others => '0');
    signal s_slabs : unsigned(2 downto 0) := (others => '0');
    signal s_regk_n, s_regk : unsigned(3 downto 0) := (others => '0');
    signal s_k6x3 : unsigned(11 downto 0) := (others => '0');
    signal s_grain_n, s_grain : unsigned(7 downto 0) := (others => '0');
    signal s_grainq : unsigned(7 downto 0) := (others => '0');
    signal s_m   : unsigned(3 downto 0) := "0001";
    signal s_k5l, s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_wide, s_echo, s_stripes, s_type, s_invert : std_logic := '0';

    -- ==== per-frame products (serial shift-add multiplier) ====
    signal q_run : std_logic := '0';
    signal q_op  : unsigned(3 downto 0) := (others => '0');
    signal q_ph  : unsigned(3 downto 0) := (others => '0');
    signal q_a, q_acc : unsigned(23 downto 0) := (others => '0');
    signal q_b   : unsigned(11 downto 0) := (others => '0');
    signal s_b     : unsigned(7 downto 0) := to_unsigned(47, 8);   -- cell unit
    signal s_depth : unsigned(8 downto 0) := to_unsigned(64, 9);   -- lines
    signal s_depth_h : unsigned(8 downto 0) := to_unsigned(32, 9); -- echo depth
    signal s_capk  : unsigned(6 downto 0) := to_unsigned(23, 7);   -- cap lines
    signal s_cap0p, s_bcap0p : unsigned(9 downto 0) := (others => '0');
    signal s_cap0, s_bcap0 : signed(11 downto 0) := (others => '0');
    signal s_RA    : unsigned(10 downto 0) := (others => '0');
    signal s_Ro, s_Ri : unsigned(10 downto 0) := (others => '0');
    signal s_RA2, s_Ro2, s_Ri2 : unsigned(19 downto 0) := (others => '0');
    signal s_cxa, s_cya, s_cxb, s_cyb : unsigned(11 downto 0) := (others => '0');
    signal s_cxa2, s_cya2, s_cxb2, s_cyb2 : unsigned(21 downto 0) := (others => '0');
    signal s_cyam1sq, s_cybm1sq : unsigned(21 downto 0) := (others => '0');
    signal s_incy0a, s_incy0b : signed(13 downto 0) := (others => '0');
    signal s_incys : signed(13 downto 0) := to_signed(2, 14);
    signal s_cw    : unsigned(10 downto 0) := to_unsigned(47, 11);
    signal s_cwm1  : unsigned(10 downto 0) := to_unsigned(46, 11);
    signal s_chm1  : unsigned(10 downto 0) := to_unsigned(375, 11);
    signal s_ch    : unsigned(10 downto 0) := to_unsigned(376, 11);
    signal s_q1, s_q3 : unsigned(10 downto 0) := (others => '0');       -- stripe cuts
    signal s_koffp, s_koffn : unsigned(10 downto 0) := (others => '0'); -- kanji column offsets
    signal s_bL    : unsigned(11 downto 0) := to_unsigned(141, 12);     -- beam length
    signal s_hb, s_bK : unsigned(6 downto 0) := (others => '0');        -- beam height, cap
    signal s_bf0, s_bc0, s_bf1 : unsigned(10 downto 0) := (others => '0');
    signal s_gh    : unsigned(5 downto 0) := to_unsigned(32, 6);        -- Latin glyph lines
    signal s_bvoff : signed(7 downto 0) := (others => '0');
    signal s_w128  : unsigned(12 downto 0) := to_unsigned(15, 13);      -- W / 128
    signal s_h32   : unsigned(12 downto 0) := to_unsigned(33, 13);      -- field lines / 32

    -- ==== analysis chain (A stages) ====
    signal a0_y  : unsigned(9 downto 0) := (others => '0');
    type t_hist is array (1 to 8) of unsigned(9 downto 0);
    signal a_hist : t_hist := (others => (others => '0'));
    signal a1_s8  : unsigned(12 downto 0) := (others => '0');
    type t_ybox is array (2 to 7) of unsigned(9 downto 0);
    signal a_ybox : t_ybox := (others => (others => '0'));   -- a_ybox(4) is B0-aligned

    -- ==== generated avid (aurora recipe) ====
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc  : unsigned(3 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_dly : std_logic_vector(7 downto 0) := (others => '0');

    -- ==== beam (B stages) ====
    signal b_av : std_logic_vector(0 to 9) := (others => '0');  -- b_av(0) = B0 avid
    signal b_fall : std_logic := '0';
    signal bcol  : unsigned(10 downto 0) := (others => '0');
    type t_col is array (1 to 8) of unsigned(10 downto 0);
    signal bcol_d : t_col := (others => (others => '0'));
    signal cnt_x : unsigned(10 downto 0) := (others => '0');
    signal cnt_y : unsigned(10 downto 0) := (others => '0');
    signal ccol  : unsigned(6 downto 0) := (others => '0');
    signal crow, crow_s : unsigned(3 downto 0) := (others => '0');
    signal fline : unsigned(10 downto 0) := (others => '0');
    signal s_fy  : unsigned(10 downto 0) := (others => '0');
    signal s_gy, s_gy2 : unsigned(9 downto 0) := (others => '0');
    signal s_hrow, s_hrow2 : unsigned(3 downto 0) := (others => '0');
    signal wl_r, smp_r, capz_r, bcz_r, bfz_r, ff : std_logic := '0';
    signal brow_v : std_logic := '0';
    signal brow   : unsigned(3 downto 0) := (others => '0');
    signal shift_acc, shift2 : signed(11 downto 0) := (others => '0');
    signal wl_d, smp_d, cst_d : std_logic_vector(1 to 8) := (others => '0');
    signal ff_d, capz_d, bcz_d, bfz_d, bv_d : std_logic_vector(1 to 6) := (others => '0');
    type t_st is array (1 to 5) of unsigned(1 downto 0);
    signal st_d : t_st := (others => (others => '0'));

    -- echo display DDAs
    signal ec_acc : unsigned(11 downto 0) := (others => '0');
    signal ecol   : unsigned(7 downto 0) := (others => '0');   -- 128 per line, wraps past the end
    signal er_acc : unsigned(11 downto 0) := (others => '0');
    signal erow   : unsigned(6 downto 0) := (others => '0');   -- 64 per field

    -- line / grid RAMs
    type t_ram16 is array (0 to 2047) of std_logic_vector(15 downto 0);
    type t_ram4  is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal s_ext  : t_ram16 := (others => (others => '0'));
    signal s_cmem : t_ram16 := (others => (others => '0'));
    signal s_hold : t_ram4  := (others => (others => '0'));
    signal s_beam : t_ram4  := (others => (others => '0'));
    signal s_echo_ram : t_ram4 := (others => (others => '0'));
    attribute ram_style : string;
    attribute ram_style of s_ext  : signal is "block";
    attribute ram_style of s_cmem : signal is "block";
    attribute ram_style of s_hold : signal is "block";
    attribute ram_style of s_beam : signal is "block";
    attribute ram_style of s_echo_ram : signal is "block";
    -- ext: a column is written 5 clocks after its read was issued and the
    -- read address is at most 4 away; cmem: a cell is written 7 clocks
    -- after its read and cells are >= 17 clocks apart; hold/beam: read at x
    -- - shift, written at x - 8 on a line that reads the old row, or by the
    -- vblank walker; echo: written by the walker only.  No read ever needs
    -- the word being written.
    attribute no_rw_check : boolean;
    attribute no_rw_check of s_ext  : signal is true;
    attribute no_rw_check of s_cmem : signal is true;
    attribute no_rw_check of s_hold : signal is true;
    attribute no_rw_check of s_beam : signal is true;
    attribute no_rw_check of s_echo_ram : signal is true;
    signal ext_ra, hold_ra, beam_ra, echo_ra, cm_ra : unsigned(10 downto 0) := (others => '0');
    signal ext_rd, ext_q, cm_rd, cm_q : std_logic_vector(15 downto 0) := (others => '0');
    signal hold_rd, hold_q, beam_rd, beam_q, echo_rd, echo_q : std_logic_vector(3 downto 0) := (others => '0');
    signal ext_wd6 : std_logic_vector(15 downto 0) := (others => '0');
    signal hw_we, bw_we : std_logic := '0';
    signal hw_wa : unsigned(10 downto 0) := (others => '0');
    signal hw_wd : unsigned(2 downto 0) := (others => '0');

    -- cell memory read-modify-write
    signal cm_wa : unsigned(10 downto 0) := (others => '0');
    signal cm_l4, cm_l5, cm_l6, cm_l7 : unsigned(11 downto 0) := (others => '0');
    signal cm_c4, cm_c5, cm_c6, cls7 : unsigned(2 downto 0) := (others => '0');
    signal cm_d4 : signed(13 downto 0) := (others => '0');
    signal cb6, cd6 : std_logic_vector(1 to 4) := (others => '0');
    signal hold_reg2, hold_wd8 : unsigned(2 downto 0) := (others => '0');

    -- vblank walker: row-0 inks, then the echo grid, from the cell memory
    signal w_st   : unsigned(1 downto 0) := (others => '0');
    signal w_act  : std_logic := '0';
    signal w_x    : unsigned(11 downto 0) := (others => '0');
    signal w_cnt  : unsigned(10 downto 0) := (others => '0');
    signal w_ccol : unsigned(6 downto 0) := (others => '0');
    signal w_crow : unsigned(3 downto 0) := (others => '0');
    signal w_ra   : unsigned(10 downto 0) := (others => '0');
    signal w_ph   : std_logic := '0';
    signal w_sub  : unsigned(6 downto 0) := (others => '0');
    signal w_erow : unsigned(5 downto 0) := (others => '0');
    signal w_xe, w_bnd, w_ye, w_bndy : unsigned(12 downto 0) := (others => '0');
    type t_wad is array (1 to 4) of unsigned(10 downto 0);
    signal w_had, w_ead : t_wad := (others => (others => '0'));
    signal w_hwe, w_ewe : std_logic_vector(1 to 4) := (others => '0');
    signal w_cross : std_logic := '0';

    signal beam_prev : unsigned(2 downto 0) := (others => '0');
    signal bcnt : unsigned(11 downto 0) := (others => '0');
    signal bon4 : std_logic := '0';
    signal cur4, ext4, echo4 : unsigned(2 downto 0) := (others => '0');
    signal age4, dec4 : unsigned(8 downto 0) := (others => '0');
    signal nz4, ein4 : std_logic := '0';
    signal ein_d : std_logic_vector(1 to 3) := (others => '0');
    signal ink5, ink6, ink7, cur5 : unsigned(2 downto 0) := (others => '0');
    signal kind5 : unsigned(1 downto 0) := (others => '0');
    signal face5 : std_logic := '0';
    signal wext5 : unsigned(2 downto 0) := (others => '0');
    signal wage5 : unsigned(8 downto 0) := (others => '0');

    -- glyph ROMs
    type t_rom256 is array (0 to 255) of std_logic_vector(15 downto 0);
    function f_romk return t_rom256 is
        variable r : t_rom256;
    begin
        for i in 0 to 255 loop r(i) := C_GLYPHS(i); end loop;
        return r;
    end function;
    function f_roml return t_rom256 is
        variable r : t_rom256;
    begin
        for i in 0 to 255 loop r(i) := C_GLYPHS(256 + i); end loop;
        return r;
    end function;
    signal s_romk  : t_rom256 := f_romk;
    signal s_romk2 : t_rom256 := f_romk;
    signal s_roml  : t_rom256 := f_roml;
    attribute ram_style of s_romk  : signal is "block";
    attribute ram_style of s_romk2 : signal is "block";
    attribute ram_style of s_roml  : signal is "block";
    signal gak1, gak2_1, gal1 : unsigned(7 downto 0) := (others => '0');
    signal rk2, rl2, rk3, rl3, rkb2, rkb3 : std_logic_vector(15 downto 0) := (others => '0');
    signal kp_v, kn_v : std_logic_vector(1 to 3) := (others => '0');
    type t_i4 is array (1 to 3) of unsigned(3 downto 0);
    signal kp_i, kn_i, lx_i, gcolb : t_i4 := (others => (others => '0'));
    signal bitkp4, bitkn4, bitl4, bitkb4 : std_logic := '0';
    signal bitkp5, bitkn5, bitl5, bitkb5, bitkb6 : std_logic := '0';

    -- grain hashes (fine / coarse)
    signal h1_a, h2_b, h3_c, h4_d : unsigned(15 downto 0) := (others => '0');
    signal c1_a, c2_b, c3_c, c4_d : unsigned(15 downto 0) := (others => '0');
    signal gbit5, gq5 : std_logic := '0';

    -- circles
    signal da_dy2, db_dy2 : unsigned(21 downto 0) := (others => '0');
    signal da_incy, db_incy : signed(13 downto 0) := (others => '0');
    signal da_r2, db_r2 : signed(24 downto 0) := (others => '0');
    signal da_inc, db_inc : signed(13 downto 0) := (others => '0');
    signal disc_d, ring_d : std_logic_vector(1 to 6) := (others => '0');

    -- plates
    type t_psr is array (0 to 16) of unsigned(2 downto 0);
    signal psr : t_psr := (others => (others => '0'));
    signal cn17, cb17, cp17 : unsigned(2 downto 0) := (others => '0');
    signal cidx18 : unsigned(2 downto 0) := (others => '0');

    -- palette / output
    signal y19, cb19, cr19 : unsigned(9 downto 0) := (others => '0');
    signal g1_y, g1_cb, g1_cr, g2_y, g2_cb, g2_cr : unsigned(9 downto 0) := (others => '0');
    signal ps_y, ps_cb, ps_cr : unsigned(9 downto 0) := (others => '0');
    signal j1_y, j1_cb, j1_cr, j2_y, j2_cb, j2_cr : unsigned(9 downto 0) := (others => '0');
    signal pt_y, pt_cb, pt_cr : unsigned(9 downto 0) := (others => '0');
    type t_sr4 is array (0 to 34) of std_logic_vector(3 downto 0);
    signal sync_sr : t_sr4 := (others => (others => '0'));
    signal hs_d, vs_d, fl_sd : std_logic_vector(1 to 4) := (others => '1');
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_s : std_logic_vector(3 downto 0) := "0111";

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement (aurora, HW-validated)
    -- ====================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt > 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt > 0 then s_meas_v <= s_vcnt; end if;
                s_vcnt  <= (others => '0');
                s_ilace <= s_timing.is_interlaced;
                s_botf  <= not s_timing.field_n;      -- bottom field = field_n '0'
                if s_timing.is_interlaced = '1' then
                    s_fh <= s_meas_v(10 downto 0) & '0';
                else
                    s_fh <= s_meas_v;
                end if;
                if s_meas_h >= to_unsigned(1000, 12) then
                    s_hd <= '1';
                else
                    s_hd <= '0';
                end if;
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;
        end if;
    end process p_measure;

    -- ====================================================================
    -- Controls: free-running decodes, field latch on the guarded vsync edge
    -- ====================================================================
    p_ctl : process(clk)
        variable v_d : signed(11 downto 0);
        variable v_r : unsigned(10 downto 0);
        variable v_abs : signed(3 downto 0);
        variable v_half : unsigned(10 downto 0);
        variable v_vo : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            s_k1 <= unsigned(registers_in(0));
            s_k2 <= unsigned(registers_in(1));
            s_k3 <= unsigned(registers_in(2));
            s_k4 <= unsigned(registers_in(3));
            s_k5 <= unsigned(registers_in(4));
            s_k6 <= unsigned(registers_in(5));
            s_sw <= registers_in(6)(4 downto 0);
            s_p12r <= unsigned(registers_in(7));

            -- Ink: band cuts (olive takes the widest band, blue and pink
            -- are the accents), shifted together by +-256 (display side)
            v_d := resize(signed('0' & s_k1), 12) - to_signed(512, 12);
            s_bias <= shift_right(v_d, 1);
            s_t1 <= to_signed(170, 12) + s_bias;
            s_t2 <= to_signed(470, 12) + s_bias;
            s_t3 <= to_signed(590, 12) + s_bias;
            s_t4 <= to_signed(710, 12) + s_bias;
            s_t1b <= s_t1 - C_HYST;  s_t1d <= s_t1 + C_HYST;
            s_t2b <= s_t2 - C_HYST;  s_t2d <= s_t2 + C_HYST;
            s_t3b <= s_t3 - C_HYST;  s_t3d <= s_t3 + C_HYST;
            s_t4b <= s_t4 - C_HYST;  s_t4d <= s_t4 + C_HYST;

            -- Slant: (k*9 >> 10) - 4 = -4 .. +4 px per line
            s_k3x9 <= resize(s_k3, 14) + shift_left(resize(s_k3, 14), 3);
            s_slant_n <= resize(signed('0' & s_k3x9(13 downto 10)), 4) - 4;
            -- Register: 0 .. 8 px
            v_r := resize(s_k4, 11) + 64;
            s_regk_n <= v_r(10 downto 7);
            -- Grain: 0 .. 191 of 256
            s_k6x3 <= resize(s_k6, 12) + shift_left(resize(s_k6, 12), 1);
            s_grain_n <= s_k6x3(11 downto 4);

            -- once-per-field pulse: the first vsync edge after active video
            s_vs_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;

            if s_vs_go = '1' then
                g_wm1     <= s_meas_h - 1;
                -- cell unit >= 17 px at HD keeps the grid inside 128 x 16
                if s_k2(9 downto 6) = 0 then s_m <= "0001"; else s_m <= s_k2(9 downto 6); end if;
                s_slant   <= s_slant_n;
                s_regk    <= s_regk_n;
                s_grain   <= s_grain_n;
                s_k5l     <= s_k5;
                s_p12     <= s_p12r;
                s_wide    <= s_sw(0);
                s_echo    <= s_sw(1);
                s_stripes <= s_sw(2);
                s_type    <= s_sw(3);
                s_invert  <= s_sw(4);
            end if;
            s_grainq <= "00" & s_grain(7 downto 2);
            -- |slant| by negate-then-SLICE (a narrowing resize folds +4 to 0)
            if s_slant < 0 then
                v_abs := -s_slant;
            else
                v_abs := s_slant;
            end if;
            s_slabs <= unsigned(v_abs(2 downto 0));
            if s_slant < 0 then
                s_cap0  <= -signed(resize(s_cap0p, 12));
                s_bcap0 <= -signed(resize(s_bcap0p, 12));
            else
                s_cap0  <= signed(resize(s_cap0p, 12));
                s_bcap0 <= signed(resize(s_bcap0p, 12));
            end if;
            s_depth_h <= '0' & s_depth(8 downto 1);

            -- free-running derived geometry (one op each)
            s_cxa <= shift_right(s_meas_h, 2) + shift_right(s_meas_h, 5) + shift_right(s_meas_h, 6);  -- 0.30 W
            s_cya <= shift_right(s_fh, 2) + shift_right(s_fh, 3) + shift_right(s_fh, 5) + shift_right(s_fh, 6);  -- 0.42 H
            s_cxb <= shift_right(s_meas_h, 3) + shift_right(s_meas_h, 4) + shift_right(s_meas_h, 6);  -- 0.20 W
            s_cyb <= shift_right(s_fh, 1) + shift_right(s_fh, 2) + shift_right(s_fh, 5) + shift_right(s_fh, 6);  -- 0.80 H
            s_Ro  <= shift_right(s_RA, 1) + shift_right(s_RA, 3);          -- 0.625 R
            s_Ri  <= shift_right(s_RA, 1) - shift_right(s_RA, 4);          -- 0.4375 R
            s_cyam1sq <= s_cya2 - resize(s_cya & '0', 22) + 1;
            s_cybm1sq <= s_cyb2 - resize(s_cyb & '0', 22) + 1;
            if s_ilace = '0' then
                s_incy0a <= to_signed(1, 14) - signed(resize(s_cya & '0', 14));
                s_incy0b <= to_signed(1, 14) - signed(resize(s_cyb & '0', 14));
                s_incys  <= to_signed(2, 14);
            elsif s_botf = '0' then
                s_incy0a <= to_signed(4, 14) - signed(resize(s_cya & "00", 14));
                s_incy0b <= to_signed(4, 14) - signed(resize(s_cyb & "00", 14));
                s_incys  <= to_signed(8, 14);
            else
                s_incy0a <= to_signed(8, 14) - signed(resize(s_cya & "00", 14));
                s_incy0b <= to_signed(8, 14) - signed(resize(s_cyb & "00", 14));
                s_incys  <= to_signed(8, 14);
            end if;
            s_w128 <= resize(s_meas_h(11 downto 7), 13);
            s_h32  <= resize(s_meas_v(11 downto 5), 13);

            -- cell geometry (field lines): Thin = b x 8b, Wide = 2b x 8b;
            -- cap b/2; beam height b/2 with a b/4 cap, at mid-row
            if s_ilace = '1' then
                s_capk <= resize(shift_right(s_b, 2), 7);
                s_hb   <= resize(shift_right(s_b, 2), 7);
                s_bK   <= resize(shift_right(s_b, 3), 7);
                s_ch   <= resize(s_b & "00", 11);
            else
                s_capk <= resize(shift_right(s_b, 1), 7);
                s_hb   <= resize(shift_right(s_b, 1), 7);
                s_bK   <= resize(shift_right(s_b, 2), 7);
                s_ch   <= resize(s_b & "000", 11);
            end if;
            if s_wide = '0' then
                s_cw <= resize(s_b, 11);
            else
                s_cw <= resize(s_b & '0', 11);
            end if;
            s_chm1 <= s_ch - 1;
            s_cwm1 <= s_cw - 1;
            s_q1   <= shift_right(s_cw, 2);
            s_q3   <= s_cw - s_q1;
            s_bL   <= resize(s_cw, 12) + resize(s_cw & '0', 12);
            -- kanji column (32 px at HD, 16 at SD) centred in the pillar's
            -- middle stripe / in the text bar
            v_half := shift_right(s_cw, 1);
            if s_hd = '1' then
                if s_cw >= 32 then s_koffp <= v_half - 16; else s_koffp <= s_q1; end if;
                if s_cw >= 32 then s_koffn <= v_half - 16; else s_koffn <= (others => '0'); end if;
            else
                if s_cw >= 16 then s_koffp <= v_half - 8; else s_koffp <= s_q1; end if;
                if s_cw >= 16 then s_koffn <= v_half - 8; else s_koffn <= (others => '0'); end if;
            end if;
            -- beam band: front lines [bf0, bf1), cap lines [bc0, bf0)
            v_half := shift_right(s_ch, 1);
            s_bf1 <= v_half;
            s_bf0 <= v_half - resize(s_hb, 11);
            if v_half >= resize(s_hb, 11) + resize(s_bK, 11) then
                s_bc0 <= v_half - resize(s_hb, 11) - resize(s_bK, 11);
            else
                s_bc0 <= (others => '0');
            end if;
            -- Latin glyph height on the beam (field lines) and its centring
            if s_hd = '1' and s_ilace = '0' then
                s_gh <= to_unsigned(32, 6);
            else
                s_gh <= to_unsigned(16, 6);
            end if;
            v_vo := signed(resize(s_hb, 9)) - signed(resize(s_gh, 9));
            s_bvoff <= resize(shift_right(v_vo, 1), 8);
        end if;
    end process p_ctl;

    -- ====================================================================
    -- Per-frame products on one serial shift-add multiplier (12 steps each)
    -- ====================================================================
    p_seq : process(clk)
        variable v_v : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_vs_go = '1' then
                q_run <= '1';
                q_op  <= (others => '0');
                q_ph  <= (others => '0');
            elsif q_run = '1' then
                case to_integer(q_ph) is
                    when 0 =>
                        q_acc <= (others => '0');
                        case to_integer(q_op) is
                            when 0 => q_a <= resize(s_meas_h, 24); q_b <= resize(s_m, 12) + 1;
                            when 1 => q_a <= resize(s_meas_v, 24); q_b <= resize(s_p12, 12);
                            when 2 => q_a <= resize(s_fh, 24);     q_b <= resize(s_k5l, 12);
                            when 3 => q_a <= resize(s_RA, 24);     q_b <= resize(s_RA, 12);
                            when 4 => q_a <= resize(s_cxa, 24);    q_b <= s_cxa;
                            when 5 => q_a <= resize(s_cya, 24);    q_b <= s_cya;
                            when 6 => q_a <= resize(s_cxb, 24);    q_b <= s_cxb;
                            when 7 => q_a <= resize(s_cyb, 24);    q_b <= s_cyb;
                            when 8 => q_a <= resize(s_Ro, 24);     q_b <= resize(s_Ro, 12);
                            when 9 => q_a <= resize(s_Ri, 24);     q_b <= resize(s_Ri, 12);
                            when 10 => q_a <= resize(s_capk, 24);  q_b <= resize(s_slabs, 12);
                            when others => q_a <= resize(s_bK, 24); q_b <= resize(s_slabs, 12);
                        end case;
                        q_ph <= q_ph + 1;
                    when 13 =>
                        case to_integer(q_op) is
                            when 0 => s_b <= resize(q_acc(14 downto 8), 8) + 2;    -- 17..122 at HD
                            when 1 =>
                                v_v := q_acc(22 downto 11);                        -- 0..539
                                if v_v > to_unsigned(499, 12) then v_v := to_unsigned(499, 12); end if;
                                s_depth <= resize(v_v, 9) + 12;                    -- 12..511
                            when 2 => s_RA  <= q_acc(21 downto 11);
                            when 3 => s_RA2 <= q_acc(19 downto 0);
                            when 4 => s_cxa2 <= q_acc(21 downto 0);
                            when 5 => s_cya2 <= q_acc(21 downto 0);
                            when 6 => s_cxb2 <= q_acc(21 downto 0);
                            when 7 => s_cyb2 <= q_acc(21 downto 0);
                            when 8 => s_Ro2 <= q_acc(19 downto 0);
                            when 9 => s_Ri2 <= q_acc(19 downto 0);
                            when 10 => s_cap0p <= q_acc(9 downto 0);
                            when others => s_bcap0p <= q_acc(9 downto 0);
                        end case;
                        if q_op = 11 then q_run <= '0'; else q_op <= q_op + 1; end if;
                        q_ph <= (others => '0');
                    when others =>
                        if q_b(0) = '1' then q_acc <= q_acc + q_a; end if;
                        q_a  <= shift_left(q_a, 1);
                        q_b  <= shift_right(q_b, 1);
                        q_ph <= q_ph + 1;
                end case;
            end if;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Analysis: 8-px luma box average, delayed to the beam stages
    -- ====================================================================
    p_ana : process(clk)
    begin
        if rising_edge(clk) then
            -- A0: input latch
            a0_y <= unsigned(data_in.y);

            -- A1: running sum of the last 8 luma samples
            a_hist(1) <= a0_y;
            for i in 2 to 8 loop a_hist(i) <= a_hist(i - 1); end loop;
            a1_s8 <= a1_s8 + resize(a0_y, 13) - resize(a_hist(8), 13);

            -- A2 .. : the window average, carried so that a_ybox(4) is the
            -- window ending at column c+7 when column c is at stage B0
            a_ybox(2) <= a1_s8(12 downto 3);
            for i in 3 to 7 loop a_ybox(i) <= a_ybox(i - 1); end loop;
        end if;
    end process p_ana;

    -- ====================================================================
    -- Clean avid (aurora/cubist recipe, HW-confirmed): starts 3 or 4 clocks
    -- after the source's first avid rise, whichever keeps the hsync->start
    -- parity at the voted usual value; ends after the field-latched width.
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;    -- hsync_n falling
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;       -- avid rising

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

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            -- beam avid = generated avid 8 clocks later, so that the luma of
            -- column c (the 8-px window ending at c+7) is ready at stage B0
            g_dly <= g_dly(6 downto 0) & g_avid;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Beam counters, line/frame flags, echo DDAs, RAM read issue (B0 -> B1)
    -- ====================================================================
    p_beam : process(clk)
        variable v_cc  : unsigned(4 downto 0);
        variable v_sub : signed(11 downto 0);
        variable v_ex  : unsigned(12 downto 0);
        variable v_ny  : unsigned(10 downto 0);
        variable v_br  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            b_av(0) <= g_dly(6);
            for i in 1 to 9 loop b_av(i) <= b_av(i - 1); end loop;
            b_fall <= b_av(1) and not b_av(0);

            -- column, cell phase and cell column (valid at B0 for bcol)
            if b_av(0) = '0' then
                bcol  <= (others => '0');
                cnt_x <= (others => '0');
                ccol  <= (others => '0');
            else
                bcol <= bcol + 1;
                if cnt_x = s_cwm1 then
                    cnt_x <= (others => '0');
                    ccol  <= ccol + 1;
                else
                    cnt_x <= cnt_x + 1;
                end if;
            end if;

            -- lines: step on the active-video fall only.  The cap shear
            -- starts at capk * slant on a row's first line and steps by
            -- -slant per line; the beam cap's starts at bK * slant on line
            -- bc0.
            if cnt_y = s_chm1 then v_ny := (others => '0'); else v_ny := cnt_y + 1; end if;
            if s_vs_go = '1' then
                cnt_y <= (others => '0');
                crow  <= (others => '0');
                fline <= (others => '0');
                ff    <= '1';
                shift_acc <= s_cap0;
                shift2    <= s_bcap0;
            elsif b_fall = '1' then
                cnt_y <= v_ny;
                if v_ny = 0 then
                    crow <= crow + 1;
                    shift_acc <= s_cap0;
                else
                    shift_acc <= shift_acc - resize(s_slant, 12);
                end if;
                if v_ny = s_bc0 then
                    shift2 <= s_bcap0;
                else
                    shift2 <= shift2 - resize(s_slant, 12);
                end if;
                fline <= fline + 1;
                ff    <= '0';
            end if;
            -- write line (the next row's inks into the line RAMs): the
            -- row's last line; sample line (cell memory update): that, or
            -- the frame's first line (for row 0); cap zone: the row's first
            -- capk lines; beam cap / front bands
            if cnt_y = s_chm1 then wl_r <= '1'; else wl_r <= '0'; end if;
            if cnt_y = s_chm1 or ff = '1' then smp_r <= '1'; else smp_r <= '0'; end if;
            if ff = '1' then crow_s <= (others => '0'); else crow_s <= crow + 1; end if;
            if cnt_y < resize(s_capk, 11) then capz_r <= '1'; else capz_r <= '0'; end if;
            if cnt_y >= s_bc0 and cnt_y < s_bf0 then bcz_r <= '1'; else bcz_r <= '0'; end if;
            if cnt_y >= s_bf0 and cnt_y < s_bf1 then bfz_r <= '1'; else bfz_r <= '0'; end if;
            -- Latin glyph row on the beam (centred; clipped if the beam is
            -- thinner than the glyph)
            v_br := signed(resize(cnt_y, 12)) - signed(resize(s_bf0, 12)) - resize(s_bvoff, 12);
            if v_br >= 0 and v_br < signed(resize(s_gh, 12)) then
                brow_v <= '1';
            else
                brow_v <= '0';
            end if;
            if s_hd = '1' and s_ilace = '0' then
                brow <= unsigned(v_br(4 downto 1));
            else
                brow <= unsigned(v_br(3 downto 0));
            end if;

            -- frame row (interlace: field line * 2 + parity), glyph rows
            -- (32 / 128 px cells at HD), kanji index per glyph row (hash:
            -- one add, one shift)
            if s_ilace = '1' then
                s_fy <= fline(9 downto 0) & s_botf;
            else
                s_fy <= fline;
            end if;
            if s_hd = '1' then
                s_gy  <= s_fy(10 downto 1);
                s_gy2 <= resize(s_fy(10 downto 3), 10);
            else
                s_gy  <= s_fy(9 downto 0);
                s_gy2 <= resize(s_fy(10 downto 2), 10);
            end if;
            v_cc := s_gy(8 downto 4);
            s_hrow <= resize(resize(v_cc & "000", 8) - resize(v_cc, 8) + resize(v_cc(4 downto 2), 8), 4);
            v_cc := s_gy2(8 downto 4);
            s_hrow2 <= resize(resize(v_cc & "000", 8) - resize(v_cc, 8) + resize(v_cc(4 downto 2), 8) + 5, 4);

            -- echo display: half size, upper right (ecol 64..127, erow < 32)
            if b_av(0) = '0' then
                ec_acc <= (others => '0');
                ecol   <= (others => '0');
            else
                v_ex := resize(ec_acc, 13) + 128;
                if v_ex >= resize(s_meas_h, 13) then
                    ec_acc <= resize(v_ex - resize(s_meas_h, 13), 12);
                    ecol   <= ecol + 1;
                else
                    ec_acc <= v_ex(11 downto 0);
                end if;
            end if;
            if s_vs_go = '1' then
                er_acc <= (others => '0');
                erow   <= (others => '0');
            elsif b_fall = '1' then
                v_ex := resize(er_acc, 13) + 64;
                if v_ex >= resize(s_meas_v, 13) then
                    er_acc <= resize(v_ex - resize(s_meas_v, 13), 12);
                    erow   <= erow + 1;
                else
                    er_acc <= v_ex(11 downto 0);
                end if;
            end if;
            if s_echo = '1' and ecol(7 downto 6) = "01" and erow(6 downto 5) = "00" then
                ein_d(1) <= '1';
            else
                ein_d(1) <= '0';
            end if;
            for i in 2 to 3 loop ein_d(i) <= ein_d(i - 1); end loop;

            -- per-pixel carries
            if cnt_x = 0 then cst_d(1) <= '1'; else cst_d(1) <= '0'; end if;
            wl_d(1)   <= wl_r;
            smp_d(1)  <= smp_r;
            ff_d(1)   <= ff;
            capz_d(1) <= capz_r;
            bcz_d(1)  <= bcz_r;
            bfz_d(1)  <= bfz_r;
            bv_d(1)   <= brow_v;
            for i in 2 to 8 loop
                cst_d(i) <= cst_d(i - 1);
                wl_d(i)  <= wl_d(i - 1);
                smp_d(i) <= smp_d(i - 1);
            end loop;
            for i in 2 to 6 loop
                ff_d(i)   <= ff_d(i - 1);
                capz_d(i) <= capz_d(i - 1);
                bcz_d(i)  <= bcz_d(i - 1);
                bfz_d(i)  <= bfz_d(i - 1);
                bv_d(i)   <= bv_d(i - 1);
            end loop;
            bcol_d(1) <= bcol;
            for i in 2 to 8 loop bcol_d(i) <= bcol_d(i - 1); end loop;
            -- pillar stripe: 0 = left quarter, 1 = middle, 2 = right quarter
            if cnt_x < s_q1 then
                st_d(1) <= "00";
            elsif cnt_x < s_q3 then
                st_d(1) <= "01";
            else
                st_d(1) <= "10";
            end if;
            for i in 2 to 5 loop st_d(i) <= st_d(i - 1); end loop;

            -- RAM read addresses (B1): previous line at x + slant (the body
            -- leans the way the caps do), the row's inks at x - cap shift,
            -- the beam twin at x - beam cap shift, the echo grid, the cell
            -- memory (the walker owns it in the vblank)
            v_sub   := resize(signed('0' & bcol), 12) + resize(s_slant, 12);
            ext_ra  <= unsigned(v_sub(10 downto 0));
            if capz_r = '1' then
                v_sub := resize(signed('0' & bcol), 12) - shift_acc;
            else
                v_sub := resize(signed('0' & bcol), 12);
            end if;
            hold_ra <= unsigned(v_sub(10 downto 0));
            if bcz_r = '1' then
                v_sub := resize(signed('0' & bcol), 12) - shift2;
            else
                v_sub := resize(signed('0' & bcol), 12);
            end if;
            beam_ra <= unsigned(v_sub(10 downto 0));
            echo_ra <= erow(4 downto 0) & ecol(5 downto 0);
            if w_act = '1' then
                cm_ra <= w_ra;
            else
                cm_ra <= crow_s & ccol;
            end if;
            if cnt_x = 0 then
                cm_wa <= crow_s & ccol;
            end if;
        end if;
    end process p_beam;

    -- ====================================================================
    -- Vblank walker: fills row 0 of the hold / beam RAMs (one write per
    -- column) and the echo grid (64 x 32 cells, two clocks per sub-step)
    -- from the cell memory.  Reads land 4 clocks after their address.
    -- ====================================================================
    p_walk : process(clk)
        variable v_xe, v_ye : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            w_hwe(1) <= '0';
            w_ewe(1) <= '0';
            case to_integer(w_st) is
                when 0 =>
                    if s_vs_go = '1' then
                        w_st    <= "01";
                        w_act   <= '1';
                        w_x     <= (others => '0');
                        w_cnt   <= (others => '0');
                        w_ccol  <= (others => '0');
                    end if;
                when 1 =>
                    -- row 0: hold[x] = ink of cell {0, x / cw}
                    w_ra <= "0000" & w_ccol;
                    w_had(1) <= w_x(10 downto 0);
                    if w_x < s_meas_h then
                        w_hwe(1) <= '1';
                    end if;
                    if w_cnt = s_cwm1 then
                        w_cnt  <= (others => '0');
                        w_ccol <= w_ccol + 1;
                    else
                        w_cnt <= w_cnt + 1;
                    end if;
                    w_x <= w_x + 1;
                    if w_x = s_meas_h + 4 then
                        w_st    <= "10";
                        w_ph    <= '0';
                        w_sub   <= (others => '0');
                        w_erow  <= (others => '0');
                        w_ccol  <= (others => '0');
                        w_crow  <= (others => '0');
                        w_xe    <= (others => '0');
                        w_bnd   <= resize(s_cw, 13);
                        w_ye    <= (others => '0');
                        w_bndy  <= resize(s_ch, 13);
                        w_cross <= '0';
                    end if;
                when 2 =>
                    -- echo: 32 rows x 128 sub-steps of W/128 px; a cell
                    -- column boundary is crossed at most once per sub-step
                    -- (cw > W/128); every second sub-step is an echo column
                    w_ra <= w_crow & w_ccol;
                    w_ead(1) <= w_erow(4 downto 0) & w_sub(6 downto 1);
                    if w_ph = '0' then
                        -- phase A: advance x, test the cell boundary
                        v_xe := w_xe + s_w128;
                        w_xe <= v_xe;
                        if v_xe >= w_bnd then w_cross <= '1'; else w_cross <= '0'; end if;
                        w_ph <= '1';
                    else
                        -- phase B: apply the crossing, emit the column
                        if w_cross = '1' then
                            w_ccol <= w_ccol + 1;
                            w_bnd  <= w_bnd + resize(s_cw, 13);
                        end if;
                        if w_sub(0) = '1' then
                            w_ewe(1) <= '1';
                        end if;
                        w_ph <= '0';
                        if w_sub = 127 then
                            w_sub  <= (others => '0');
                            w_ccol <= (others => '0');
                            w_xe   <= (others => '0');
                            w_bnd  <= resize(s_cw, 13);
                            v_ye := w_ye + s_h32;
                            w_ye <= v_ye;
                            if v_ye >= w_bndy then
                                w_crow <= w_crow + 1;
                                w_bndy <= w_bndy + resize(s_ch, 13);
                            end if;
                            if w_erow = 31 then
                                w_st <= "11";
                                w_x  <= (others => '0');
                            else
                                w_erow <= w_erow + 1;
                            end if;
                        else
                            w_sub <= w_sub + 1;
                        end if;
                    end if;
                when others =>
                    -- flush the last writes, release the ports
                    w_x <= w_x + 1;
                    if w_x = 6 then
                        w_st  <= "00";
                        w_act <= '0';
                    end if;
            end case;
            for i in 2 to 4 loop
                w_had(i) <= w_had(i - 1);
                w_ead(i) <= w_ead(i - 1);
                w_hwe(i) <= w_hwe(i - 1);
                w_ewe(i) <= w_ewe(i - 1);
            end loop;
        end if;
    end process p_walk;

    -- ====================================================================
    -- RAMs: registered read, fabric re-register, ONE write each (the
    -- hold / beam write port is muxed between the walker and the pipeline
    -- one stage ahead)
    -- ====================================================================
    p_ram : process(clk)
    begin
        if rising_edge(clk) then
            ext_rd <= s_ext(to_integer(ext_ra));                 -- B2
            if b_av(6) = '1' then
                s_ext(to_integer(bcol_d(6))) <= ext_wd6;         -- column c, B6
            end if;

            hold_rd <= s_hold(to_integer(hold_ra));
            beam_rd <= s_beam(to_integer(beam_ra));
            if w_act = '1' then
                hw_we <= w_hwe(4);
                hw_wa <= w_had(4);
                hw_wd <= unsigned(cm_q(14 downto 12));
            else
                hw_we <= b_av(8) and wl_d(8);
                hw_wa <= bcol_d(8);
                hw_wd <= hold_wd8;
            end if;
            if hw_we = '1' then
                s_hold(to_integer(hw_wa)) <= '0' & std_logic_vector(hw_wd);
            end if;
            if hw_we = '1' then
                s_beam(to_integer(hw_wa)) <= '0' & std_logic_vector(hw_wd);
            end if;

            echo_rd <= s_echo_ram(to_integer(echo_ra));
            if w_ewe(4) = '1' then
                s_echo_ram(to_integer(w_ead(4))) <= '0' & cm_q(14 downto 12);
            end if;

            cm_rd <= s_cmem(to_integer(cm_ra));
            if cst_d(8) = '1' and smp_d(8) = '1' and b_av(8) = '1' then
                s_cmem(to_integer(cm_wa)) <= '0' & std_logic_vector(cls7) & std_logic_vector(cm_l7);
            end if;
        end if;
    end process p_ram;

    -- ====================================================================
    -- Cell memory read-modify-write (B3 .. B8): frame-to-frame luma IIR
    -- and hysteresis classification, one cell per cell start on sample
    -- lines; the result feeds the row's hold stream.
    -- ====================================================================
    p_cell : process(clk)
        variable v_lum : signed(11 downto 0);
        variable v_sum : signed(13 downto 0);
        variable v_br, v_dk : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- B3: fabric re-register
            cm_q <= cm_rd;
            -- B4: fresh sample (10.2) minus stored luma
            cm_d4 <= signed(resize(a_ybox(7) & "00", 14)) - signed(resize(unsigned(cm_q(11 downto 0)), 14));
            cm_l4 <= unsigned(cm_q(11 downto 0));
            cm_c4 <= unsigned(cm_q(14 downto 12));
            -- B5: one-pole blend, 1/16 per frame (SLICE the sum: a narrowing
            -- resize of a signed value folds lumas >= 2048)
            v_sum := signed(resize(cm_l4, 14)) + shift_right(cm_d4, 4);
            cm_l5 <= unsigned(v_sum(11 downto 0));
            cm_c5 <= cm_c4;
            -- B6: cuts pre-shifted by the hysteresis (per-frame constants):
            -- cb = "even the luma + 24 clears the cut", cd = "luma - 24 does"
            v_lum := signed(resize(cm_l5(11 downto 2), 12));
            if v_lum >= s_t1b then cb6(1) <= '1'; else cb6(1) <= '0'; end if;
            if v_lum >= s_t2b then cb6(2) <= '1'; else cb6(2) <= '0'; end if;
            if v_lum >= s_t3b then cb6(3) <= '1'; else cb6(3) <= '0'; end if;
            if v_lum >= s_t4b then cb6(4) <= '1'; else cb6(4) <= '0'; end if;
            if v_lum >= s_t1d then cd6(1) <= '1'; else cd6(1) <= '0'; end if;
            if v_lum >= s_t2d then cd6(2) <= '1'; else cd6(2) <= '0'; end if;
            if v_lum >= s_t3d then cd6(3) <= '1'; else cd6(3) <= '0'; end if;
            if v_lum >= s_t4d then cd6(4) <= '1'; else cd6(4) <= '0'; end if;
            cm_l6 <= cm_l5;
            cm_c6 <= cm_c5;
            -- B7: a cell gets brighter only if even the pessimistic luma
            -- says so, darker only if even the optimistic luma says so
            v_dk := f_band(cd6(1), cd6(2), cd6(3), cd6(4));
            v_br := f_band(cb6(1), cb6(2), cb6(3), cb6(4));
            if v_dk < cm_c6 then
                cls7 <= v_dk;
            elsif v_br > cm_c6 then
                cls7 <= v_br;
            else
                cls7 <= cm_c6;
            end if;
            cm_l7 <= cm_l6;
            -- B8: the row's hold stream (the cell's new ink, held across it)
            if cst_d(7) = '1' then
                hold_wd8 <= cls7;
                if b_av(7) = '1' then hold_reg2 <= cls7; end if;
            else
                hold_wd8 <= hold_reg2;
            end if;
        end if;
    end process p_cell;

    -- ====================================================================
    -- Caps, beams, extrusion, echo (B3 .. B6)
    -- ====================================================================
    p_ext : process(clk)
        variable v_bq : unsigned(2 downto 0);
        variable v_ink, v_wext : unsigned(2 downto 0);
        variable v_wage : unsigned(8 downto 0);
        variable v_face : std_logic;
        variable v_kind : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            -- B3: fabric re-registers
            ext_q  <= ext_rd;
            hold_q <= hold_rd;
            beam_q <= beam_rd;
            echo_q <= echo_rd;

            -- B4: this pixel's cell ink, age tests, echo word, beam
            -- countdown from a navy cell's left edge in the twin RAM
            cur4 <= unsigned(hold_q(2 downto 0));
            ext4 <= unsigned(ext_q(11 downto 9));
            age4 <= unsigned(ext_q(8 downto 0));
            dec4 <= unsigned(ext_q(8 downto 0)) - 1;
            if unsigned(ext_q(8 downto 0)) /= 0 and ff_d(3) = '0' then
                nz4 <= '1';
            else
                nz4 <= '0';
            end if;
            echo4 <= unsigned(echo_q(2 downto 0));
            ein4  <= ein_d(3);
            v_bq := unsigned(beam_q(2 downto 0));
            beam_prev <= v_bq;
            if b_av(3) = '0' then
                beam_prev <= K_PAPER;
                bcnt <= (others => '0');
                bon4 <= '0';
            elsif v_bq = K_NAVY and beam_prev /= K_NAVY then
                bcnt <= s_bL - 1;
                bon4 <= '1';
            elsif bcnt /= 0 then
                bcnt <= bcnt - 1;
                bon4 <= '1';
            else
                bon4 <= '0';
            end if;

            -- B5: beam cap > beam front > cell cap > cell front (pillar /
            -- text bar) > side > echo > olive ground > paper
            v_ink  := K_PAPER;
            v_face := '0';
            v_kind := F_PLAIN;
            v_wext := ext4;
            if nz4 = '1' then v_wage := dec4; else v_wage := (others => '0'); end if;
            if b_av(4) = '0' then
                v_wext := (others => '0');
                v_wage := (others => '0');
            elsif bon4 = '1' and bcz_d(4) = '1' then
                v_ink  := K_PINK;
                v_face := '1';
            elsif bon4 = '1' and bfz_d(4) = '1' then
                v_ink  := K_NAVY;
                v_kind := F_BEAM;
                v_wext := K_NAVY;
                v_wage := s_depth;
            elsif capz_d(4) = '1' and cur4 /= K_PAPER and cur4 /= K_OLIVE then
                v_ink  := f_cap(cur4);
                v_face := '1';
            elsif capz_d(4) = '0' and (cur4 = K_BLUE or cur4 = K_PINK) then
                v_ink  := cur4;
                v_kind := F_PILLAR;
                v_wext := cur4;
                v_wage := s_depth;
            elsif capz_d(4) = '0' and cur4 = K_NAVY then
                v_ink  := K_NAVY;
                v_kind := F_BAR;
                v_wext := K_NAVY;
                v_wage := s_depth;
            elsif nz4 = '1' then
                if s_stripes = '1' and age4(2) = '1' then
                    v_ink := ext4;
                else
                    v_ink := f_side(ext4);
                end if;
                v_face := '1';
            elsif ein4 = '1' and echo4 /= K_PAPER then
                v_ink := echo4;
                if echo4 /= K_OLIVE then
                    v_wext := echo4;
                    v_wage := s_depth_h;
                end if;
            elsif cur4 = K_OLIVE then
                v_ink := K_OLIVE;
            end if;
            ink5  <= v_ink;
            face5 <= v_face;
            kind5 <= v_kind;
            cur5  <= cur4;
            wext5 <= v_wext;
            wage5 <= v_wage;

            -- B6: write data
            ext_wd6 <= "0000" & std_logic_vector(wext5) & std_logic_vector(wage5);
        end if;
    end process p_ext;

    -- ====================================================================
    -- Side chains keyed to B0: glyph ROMs, grain hashes, circles
    -- ====================================================================
    p_side : process(clk)
        variable v_gx2 : unsigned(9 downto 0);
        variable v_xg, v_yg : unsigned(10 downto 0);
        variable v_a, v_e, v_f : unsigned(15 downto 0);
        variable v_t : unsigned(8 downto 0);
        variable v_kx : signed(11 downto 0);
        variable v_lim : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- glyph addresses (B1): kanji column (32 px) per glyph row, big
            -- kanji per 128-px cell (index varies along the row) for the
            -- disc, Latin letter = 32-px cell x along the beam
            if s_hd = '1' then
                v_gx2 := resize(bcol(10 downto 3), 10);
                v_lim := to_signed(32, 12);
            else
                v_gx2 := resize(bcol(10 downto 2), 10);
                v_lim := to_signed(16, 12);
            end if;
            gak1   <= s_hrow & s_gy(3 downto 0);
            gak2_1 <= (s_hrow2 + resize(v_gx2(7 downto 4) & "00", 4) + v_gx2(7 downto 4)) & s_gy2(3 downto 0);
            if s_hd = '1' then
                gal1    <= bcol(8 downto 5) & brow;
                lx_i(1) <= bcol(4 downto 1);
            else
                gal1    <= bcol(7 downto 4) & brow;
                lx_i(1) <= bcol(3 downto 0);
            end if;
            gcolb(1) <= v_gx2(3 downto 0);
            -- kanji column bit index relative to the stripe / bar offset
            v_kx := signed(resize(cnt_x, 12)) - signed(resize(s_koffp, 12));
            if v_kx >= 0 and v_kx < v_lim then kp_v(1) <= '1'; else kp_v(1) <= '0'; end if;
            if s_hd = '1' then kp_i(1) <= unsigned(v_kx(4 downto 1)); else kp_i(1) <= unsigned(v_kx(3 downto 0)); end if;
            v_kx := signed(resize(cnt_x, 12)) - signed(resize(s_koffn, 12));
            if v_kx >= 0 and v_kx < v_lim then kn_v(1) <= '1'; else kn_v(1) <= '0'; end if;
            if s_hd = '1' then kn_i(1) <= unsigned(v_kx(4 downto 1)); else kn_i(1) <= unsigned(v_kx(3 downto 0)); end if;
            -- B2: ROM reads
            rk2  <= s_romk(to_integer(gak1));
            rl2  <= s_roml(to_integer(gal1));
            rkb2 <= s_romk2(to_integer(gak2_1));
            for i in 2 to 3 loop
                kp_v(i) <= kp_v(i - 1);  kp_i(i) <= kp_i(i - 1);
                kn_v(i) <= kn_v(i - 1);  kn_i(i) <= kn_i(i - 1);
                lx_i(i) <= lx_i(i - 1);  gcolb(i) <= gcolb(i - 1);
            end loop;
            -- B3: fabric re-register
            rk3 <= rk2;  rl3 <= rl2;  rkb3 <= rkb2;
            -- B4: bit select (MSB = leftmost)
            bitkp4 <= kp_v(3) and rk3(to_integer(not kp_i(3)));
            bitkn4 <= kn_v(3) and rk3(to_integer(not kn_i(3)));
            bitl4  <= rl3(to_integer(not lx_i(3)));
            bitkb4 <= rkb3(to_integer(not gcolb(3)));
            -- B5, B6
            bitkp5 <= bitkp4;  bitkn5 <= bitkn4;  bitl5 <= bitl4 and bv_d(4);  bitkb5 <= bitkb4;
            bitkb6 <= bitkb5;

            -- grain: fine hash (2 px at HD) and coarse hash (8 px), averaged
            if s_hd = '1' then
                v_xg := '0' & bcol(10 downto 1);
                v_yg := '0' & s_fy(10 downto 1);
            else
                v_xg := bcol;
                v_yg := s_fy;
            end if;
            v_a := shift_left(resize(v_xg, 16), 4) xor shift_left(resize(v_yg, 16), 9)
                   xor resize(shift_right(v_xg, 3), 16) xor resize(v_yg, 16);
            h1_a <= v_a;
            h2_b <= h1_a + shift_left(h1_a, 3) + shift_left(h1_a, 5);          -- *41
            h3_c <= h2_b xor shift_right(h2_b, 8);
            h4_d <= h3_c + shift_left(h3_c, 4) + shift_left(h3_c, 9);          -- *529
            v_xg := shift_right(v_xg, 2);
            v_yg := shift_right(v_yg, 2);
            v_a := shift_left(resize(v_xg, 16), 4) xor shift_left(resize(v_yg, 16), 9)
                   xor resize(shift_right(v_xg, 3), 16) xor resize(v_yg, 16) xor x"5A5A";
            c1_a <= v_a;
            c2_b <= c1_a + shift_left(c1_a, 3) + shift_left(c1_a, 5);
            c3_c <= c2_b xor shift_right(c2_b, 8);
            c4_d <= c3_c + shift_left(c3_c, 4) + shift_left(c3_c, 9);
            v_e := h4_d xor shift_right(h4_d, 8);
            v_f := c4_d xor shift_right(c4_d, 8);
            v_t := resize(v_e(15 downto 8), 9) + resize(v_f(15 downto 8), 9);
            if v_t(8 downto 1) < s_grain then
                gbit5 <= '1';
            else
                gbit5 <= '0';
            end if;
            if v_t(8 downto 1) < s_grainq then
                gq5 <= '1';
            else
                gq5 <= '0';
            end if;

            -- circles: r^2 by second differences, line state stepped per
            -- active line, loaded during the first vblank lines
            if b_fall = '1' then
                da_dy2  <= unsigned(signed(da_dy2) + resize(da_incy, 22));
                da_incy <= da_incy + s_incys;
                db_dy2  <= unsigned(signed(db_dy2) + resize(db_incy, 22));
                db_incy <= db_incy + s_incys;
            elsif ff = '1' then
                if s_botf = '1' and s_ilace = '1' then
                    da_dy2 <= s_cyam1sq;
                    db_dy2 <= s_cybm1sq;
                else
                    da_dy2 <= s_cya2;
                    db_dy2 <= s_cyb2;
                end if;
                da_incy <= s_incy0a;
                db_incy <= s_incy0b;
            end if;
            if b_av(0) = '0' then
                da_r2  <= signed(resize(da_dy2, 25)) + signed(resize(s_cxa2, 25));
                da_inc <= to_signed(1, 14) - signed(resize(s_cxa & '0', 14));
                db_r2  <= signed(resize(db_dy2, 25)) + signed(resize(s_cxb2, 25));
                db_inc <= to_signed(1, 14) - signed(resize(s_cxb & '0', 14));
            else
                da_r2  <= da_r2 + resize(da_inc, 25);
                da_inc <= da_inc + 2;
                db_r2  <= db_r2 + resize(db_inc, 25);
                db_inc <= db_inc + 2;
            end if;
            if da_r2 < signed(resize(s_RA2, 25)) then
                disc_d(1) <= '1';
            else
                disc_d(1) <= '0';
            end if;
            if db_r2 < signed(resize(s_Ro2, 25)) and db_r2 >= signed(resize(s_Ri2, 25)) then
                ring_d(1) <= '1';
            else
                ring_d(1) <= '0';
            end if;
            for i in 2 to 6 loop
                disc_d(i) <= disc_d(i - 1);
                ring_d(i) <= ring_d(i - 1);
            end loop;
        end if;
    end process p_side;

    -- ====================================================================
    -- Stripes, lettering, grain, circles, plates, palette (B6 .. B19)
    -- ====================================================================
    p_print : process(clk)
        variable v : unsigned(2 downto 0);
        variable m_navy, m_olive, m_blue, m_pink : boolean;
    begin
        if rising_edge(clk) then
            -- B6: pillar stripes (own ink | cream with a navy kanji column |
            -- the cap ink), text bars (navy kanji column on cream), beams
            -- (cream Latin on navy), then grain
            v := ink5;
            case kind5 is
                when F_PILLAR =>
                    case st_d(5) is
                        when "00"   => v := cur5;
                        when "01"   =>
                            if s_type = '1' and bitkp5 = '1' then v := K_NAVY; else v := K_PAPER; end if;
                        when others => v := f_cap(cur5);
                    end case;
                when F_BAR =>
                    if s_type = '1' and bitkn5 = '1' then v := K_NAVY; else v := K_PAPER; end if;
                when F_BEAM =>
                    if s_type = '1' and bitl5 = '1' then v := K_PAPER; else v := K_NAVY; end if;
                when others => null;
            end case;
            if v = K_OLIVE and gbit5 = '1' then
                v := K_PAPER;
            elsif face5 = '1' and gq5 = '1' then
                v := K_PAPER;
            end if;
            ink6 <= v;

            -- B7: pink ring, then the blue disc with cream kanji, over all
            v := ink6;
            if ring_d(6) = '1' then
                v := K_PINK;
            end if;
            if disc_d(6) = '1' then
                if bitkb6 = '1' and s_type = '1' then v := K_PAPER; else v := K_BLUE; end if;
            end if;
            ink7 <= v;

            -- plates: 17-deep shift register, taps 8 (navy/olive), 8+k
            -- (blue, shifted right), 8-k (pink, shifted left)
            psr(0) <= ink7;
            for i in 1 to 16 loop psr(i) <= psr(i - 1); end loop;
            cn17 <= psr(8);
            cb17 <= psr(8 + to_integer(s_regk));
            cp17 <= psr(8 - to_integer(s_regk));

            -- B18: composite in print order; Night swaps paper and navy
            m_navy  := cn17 = K_NAVY;
            m_olive := cn17 = K_OLIVE;
            m_blue  := cb17 = K_BLUE;
            m_pink  := cp17 = K_PINK;
            if    m_navy           then v := K_NAVY;
            elsif m_blue and m_pink then v := K_VIOLET;
            elsif m_blue           then v := K_BLUE;
            elsif m_pink           then v := K_PINK;
            elsif m_olive          then v := K_OLIVE;
            else                        v := K_PAPER;
            end if;
            if s_invert = '1' then
                if v = K_PAPER then v := K_NAVY;
                elsif v = K_NAVY then v := K_PAPER;
                end if;
            end if;
            cidx18 <= v;

            -- B19: palette (BT.601 Y / Cb / Cr; sampled from the poster)
            case cidx18 is
                when K_PINK   => y19 <= to_unsigned(580, 10); cb19 <= to_unsigned(492, 10); cr19 <= to_unsigned(582, 10);
                when K_BLUE   => y19 <= to_unsigned(442, 10); cb19 <= to_unsigned(617, 10); cr19 <= to_unsigned(427, 10);
                when K_OLIVE  => y19 <= to_unsigned(304, 10); cb19 <= to_unsigned(457, 10); cr19 <= to_unsigned(523, 10);
                when K_NAVY   => y19 <= to_unsigned(146, 10); cb19 <= to_unsigned(534, 10); cr19 <= to_unsigned(490, 10);
                when K_VIOLET => y19 <= to_unsigned(281, 10); cb19 <= to_unsigned(565, 10); cr19 <= to_unsigned(487, 10);
                when others   => y19 <= to_unsigned(853, 10); cb19 <= to_unsigned(475, 10); cr19 <= to_unsigned(533, 10);
            end case;
        end if;
    end process p_print;

    -- ====================================================================
    -- Output conditioning (damier's HD HDMI bar fix, doubled): two [1 2 1]/4
    -- passes on Y/Cb/Cr (= [1 4 6 4 1]/16), luma 64 + 13/16 (y - 64),
    -- neutral outside the generated avid; two plain registers to a total
    -- latency of 40
    -- ====================================================================
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            g1_y <= y19;   g2_y <= g1_y;
            g1_cb <= cb19; g2_cb <= g1_cb;
            g1_cr <= cr19; g2_cr <= g1_cr;
            v_sy := resize(g2_y, 12)  + shift_left(resize(g1_y, 12), 1)  + resize(y19, 12);
            v_su := resize(g2_cb, 12) + shift_left(resize(g1_cb, 12), 1) + resize(cb19, 12);
            v_sv := resize(g2_cr, 12) + shift_left(resize(g1_cr, 12), 1) + resize(cr19, 12);
            ps_y  <= v_sy(11 downto 2);
            ps_cb <= v_su(11 downto 2);
            ps_cr <= v_sv(11 downto 2);

            j1_y <= ps_y;   j2_y <= j1_y;
            j1_cb <= ps_cb; j2_cb <= j1_cb;
            j1_cr <= ps_cr; j2_cr <= j1_cr;
            v_sy := resize(j2_y, 12)  + shift_left(resize(j1_y, 12), 1)  + resize(ps_y, 12);
            v_su := resize(j2_cb, 12) + shift_left(resize(j1_cb, 12), 1) + resize(ps_cb, 12);
            v_sv := resize(j2_cr, 12) + shift_left(resize(j1_cr, 12), 1) + resize(ps_cr, 12);
            pt_y  <= v_sy(11 downto 2);
            pt_cb <= v_su(11 downto 2);
            pt_cr <= v_sv(11 downto 2);

            -- column c's twice-smoothed value arrives here 27 clocks after
            -- its stage B0, i.e. 39 after data_in; the gate's avid matches
            v_d  := pt_y - 64;
            v_yo := pt_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if sync_sr(34)(3) = '0' then
                s_out_y <= C_BLK;
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo;
                s_out_u <= pt_cr;      -- hardware u carries Cr
                s_out_v <= pt_cb;
            end if;
            s_out_s <= sync_sr(34);

            -- sync delay: generated avid 35 clocks, source syncs 4 + 35
            hs_d(1) <= data_in.hsync_n;
            vs_d(1) <= data_in.vsync_n;
            fl_sd(1) <= data_in.field_n;
            for i in 2 to 4 loop
                hs_d(i) <= hs_d(i - 1);
                vs_d(i) <= vs_d(i - 1);
                fl_sd(i) <= fl_sd(i - 1);
            end loop;
            sync_sr(0) <= g_avid & hs_d(4) & vs_d(4) & fl_sd(4);
            for i in 1 to 34 loop sync_sr(i) <= sync_sr(i - 1); end loop;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_out_s(3);
    data_out.hsync_n <= s_out_s(2);
    data_out.vsync_n <= s_out_s(1);
    data_out.field_n <= s_out_s(0);

end architecture ginza;
