-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- ruttetra_xy.vhd - Rutt-Etra scan processor with horizontal AND vertical
-- displacement, for Videomancer
--
-- ruttetra (horizontal scan lines pushed down by luma) with the displacement
-- opened up to both axes.  One knob (K5 Mode) picks how:
--
--   Classic   the ruttetra picture: lines pushed straight down.
--   Slant     the same lines pushed DIAGONALLY: every point moves sideways as
--             well as down (1 or 2 pixels across per row down, left or right),
--             the look of video on both deflection yokes.
--   Mesh      Slant plus rungs: at every grid node a rung drops from the line
--             to the next line, so the terrain reads as a wire mesh.
--   Weave     Classic plus a second family of VERTICAL lines pushed sideways
--             by the same luma (the ruttetra_v picture), woven over the first.
--
-- Engine (see ruttetra.vhd for the base)
--   Every Nth active row is a sampled row; its luma (contrast + horizontal
--   smoothing) becomes {disp[7:0], bright[3:0]} per column (columns x2
--   decimated) in one of 8 rotating banks.  Per display pixel, 8 candidate
--   tests (age_k against the candidate's connected-stroke segment) find the
--   hits; the most recent bank wins (= hidden-line rule).
--
--   Slant: a point displaced by d rows also moves L*d pixels sideways, and a
--   hit needs d = age, so bank k is simply read L*age_k pixels to the side.
--   Each bank has an offset accumulator stepped once per active row beside its
--   age (in whole + 1/16 columns, scaled per timing class so the angle is the
--   same on every raster).  A bank read outside its stored row must neither
--   hit nor cover: a negative address forces the word to 0, and every sampled
--   row ends in a zero word (disp 0 never occurs in real data), after which
--   the bank is dead for the rest of the scanline.  The first valid column of
--   a bank restarts its connected stroke.
--
--   Mesh: a node at source column c0 of a line displaced by d shows at
--   x = c0 + L*d, so a pixel under a line is on a rung when (x - L*d) lands
--   on the node grid, d being the covering line's displacement there.  The
--   grid pitch is the power of two nearest the line spacing (16 / 32 / 64
--   virtual pixels), so the test is a 6-bit add per bank.  Rungs take the
--   covering line's brightness.
--
--   Weave: every Nth column is sampled on every row from the SAME displacement
--   stream, and the last 4 sampled columns become candidates {previous row's
--   displacement, this row's, bright, age} (ruttetra_v's engine).  They do not
--   get an engine of their own: in Weave the scan lines keep banks 0..3 and
--   the vertical lines borrow candidate slots 4..7 -- their words are loaded
--   straight into those slots' bank-output registers (shifting 7 -> 4), the
--   slots' age registers count display pixels instead of rows, and the
--   connected-stroke, classify and resolve stages are shared unchanged.  A
--   256-entry per-column RAM keeps the displacement of the previous row.  The
--   two families hide lines only within themselves.
--
--   Resolve: a one-hot "first candidate that hits or covers, in recency
--   order" per family; a line is visible when that candidate hits.
--
-- Controls
--   Pot 1  (registers_in(0))    : Scan Lines - line density (CW = denser)
--   Pot 2  (registers_in(1))    : Thickness  - stroke 1..8; glow from T=3
--   Pot 3  (registers_in(2))    : Line Color - White/Green/Amber/Cyan/
--                                 Magenta/Video/Fire/Terrain
--   Pot 4  (registers_in(3))    : Contrast   - input gain into disp+shade
--   Pot 5  (registers_in(4))    : Mode       - Mesh < / Slant << / Slant < /
--                                 Classic / Slant > / Slant >> / Mesh > /
--                                 Weave
--   Pot 6  (registers_in(5))    : Smoothing  - horizontal IIR on the profile
--   Tog 7  (registers_in(6)(0)) : Peaks      - Bright rises / Dark rises
--   Tog 8  (registers_in(6)(1)) : Hidden Line- terrain skirts on/off
--   Tog 9  (registers_in(6)(2)) : Shading    - Banded (2-level) / Luma
--   Tog 10 (registers_in(6)(3)) : Roll       - Run / Freeze (fixed slow roll)
--   Tog 11 (registers_in(6)(4)) : Input      - Positive / Negative
--   Fader  (registers_in(7))    : RELIEF     - displacement depth (glided)
--
-- Latency: 16 clocks, identical in every mode.
--
-- Resolution scaling: rows are VIRTUAL 1080-line rows and horizontal work
-- (slant, node grid, vertical lines) is in VIRTUAL 1920-wide pixels (x step
-- 1 / 1.5 / 2 at 1080 / 720p / SD, where the 4:3 picture is a 1440-wide
-- window), so every raster shows the same picture.
--
-- Output conditioning: rungs and vertical lines are near-full-swing
-- pixel-rate edges on every scanline (the HD HDMI thick-bar trigger), so Y, U
-- and V pass a horizontal [1 2 1]/4 before the blanking gate; strokes 1 / 2
-- pixels wide from those two sources are drawn x1.5 / x1.25 ahead of it.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;


architecture ruttetra_xy of program_top is
    -- data_in -> data_out latency (a multiple of 4, as the SDK requires); must
    -- match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 16;

    constant C_LAT : integer := 16;      -- display pipeline registers
    constant NB    : integer := 8;       -- candidate slots / sampled-row banks
    constant NVS    : integer := 4;       -- first slot lent to the vertical lines

    attribute no_rw_check : boolean;

    ----------------------------------------------------------------------
    -- Timing class (free-running from the timing ID)
    ----------------------------------------------------------------------
    -- y step per FIELD row (Q4.8, x2 for interlace) and its ceiling
    signal s_ys    : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_kc    : unsigned(3 downto 0) := to_unsigned(1, 4);
    -- x step: 2 at SD, 1.5 at 720p (alternating 1 / 2), 1 at 1080
    signal s_sd    : std_logic := '0';
    signal s_720   : std_logic := '0';
    signal s_kcx   : unsigned(3 downto 0) := to_unsigned(1, 4);
    -- slant 1: columns (2 pixels) per field row, in 1/16
    signal s_lbase : unsigned(4 downto 0) := to_unsigned(8, 5);

    ----------------------------------------------------------------------
    -- Latched controls (at vsync)
    ----------------------------------------------------------------------
    signal s_dspace : unsigned(6 downto 0) := to_unsigned(39, 7);  -- 8..71
    signal s_dm1    : signed(8 downto 0) := to_signed(38, 9);      -- D - 1
    signal s_dm2    : signed(8 downto 0) := to_signed(37, 9);      -- D - 2
    signal s_dmax   : unsigned(7 downto 0) := to_unsigned(255, 8);
    -- Weave: each family lives 4 samples, so displacement holds there
    signal s_dmaxv  : unsigned(7 downto 0) := to_unsigned(155, 8);
    -- relief normaliser: min(spacing, 32) (32 = unity; window = 8 x spacing)
    signal s_rmul   : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal s_thick  : unsigned(3 downto 0) := to_unsigned(2, 4);   -- 1..8
    -- stroke windows: max(T, ceil(step)) on each axis, and what the
    -- candidate classifier needs from them (free-running derivations)
    signal s_thkw   : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal s_thkx   : unsigned(3 downto 0) := to_unsigned(2, 4);
    -- slots 4..7: rows, or pixels in Weave
    signal s_tb     : unsigned(3 downto 0) := to_unsigned(2, 4);
    -- glow band 8 units wide (else 4); beam-speed dim thresholds 2^n, 2^(n+2)
    signal s_w8     : std_logic := '0';
    signal s_dn     : unsigned(1 downto 0) := "10";
    -- thin strokes integrate less light: small shading lift at T=1..3
    signal s_bboost : unsigned(1 downto 0) := "10";
    -- thin-stroke gain ahead of the [1 2 1] output filter (rungs and
    -- vertical lines), by the stroke's width in PIXELS: "10" x1.5, "01" x1.25
    signal s_tgain  : unsigned(1 downto 0) := "00";
    signal s_zone   : unsigned(2 downto 0) := (others => '0');
    -- Label decodes, free-running.  At program load the firmware writes the
    -- initial_value_label's INDEX (0..7), not a 0..1023 position, so a value
    -- below 8 is the label index itself.
    signal s_k3_idx : unsigned(2 downto 0) := (others => '0');
    signal s_k5_idx : unsigned(3 downto 0) := (others => '0');
    signal s_gain8  : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_smooth : unsigned(2 downto 0) := "010";
    signal s_lagp1  : unsigned(7 downto 0) := to_unsigned(4, 8);
    signal s_scale9 : unsigned(8 downto 0) := to_unsigned(8, 9);   -- glided
    signal s_scale_eff : unsigned(8 downto 0) := to_unsigned(8, 9);
    signal s_peakdark : std_logic := '0';
    signal s_hlr      : std_logic := '1';
    signal s_shade    : std_logic := '1';
    signal s_glow     : std_logic := '0';
    signal s_gsh      : unsigned(2 downto 0) := "010";
    signal s_rfreeze  : std_logic := '0';
    signal s_neg      : std_logic := '0';

    -- K5 Mode
    signal s_weave  : std_logic := '0';
    signal s_mesh   : std_logic := '0';
    signal s_lstep  : signed(6 downto 0) := (others => '0');  -- cols/16 per row
    -- node grid: mask for bits 5..4 of the 6-bit grid phase (pitch 16/32/64);
    -- the phase counter runs down for a right slant
    signal s_gmask  : unsigned(1 downto 0) := "01";
    signal s_rdown  : std_logic := '0';
    signal s_xrinit : unsigned(5 downto 0) := (others => '0');
    -- slant centring: the mid-grey displacement stays on its source column
    signal s_d0     : unsigned(7 downto 0) := (others => '0');   -- rows, <= 128
    signal s_ctr    : signed(9 downto 0) := (others => '0');     -- columns
    signal s_lean2  : std_logic := '0';
    signal s_lneg   : std_logic := '0';
    signal s_lon    : std_logic := '0';
    signal s_pxc0   : signed(10 downto 0) := (others => '0');
    -- display read column + centring (bank addresses come off this)
    signal s_pxc    : signed(10 downto 0) := (others => '0');
    signal s_xr     : unsigned(5 downto 0) := (others => '0');

    -- Roll: line-grid phase in units of one line spacing (wraps free)
    constant C_RSPD : unsigned(13 downto 0) := to_unsigned(2704, 14);
    signal s_rphase : unsigned(15 downto 0) := (others => '0');
    signal s_roff   : unsigned(6 downto 0) := (others => '0');
    -- serial multiplier for the per-field products
    signal m_job    : std_logic := '0';
    signal m_cnt    : unsigned(3 downto 0) := (others => '0');
    signal m_sh     : unsigned(7 downto 0) := (others => '0');
    signal m_acc    : unsigned(14 downto 0) := (others => '0');
    signal s_fade4  : unsigned(3 downto 0) := (others => '0');
    signal s_firstrow : std_logic := '1';
    signal s_rowtop   : std_logic := '0';

    ----------------------------------------------------------------------
    -- Position + row bookkeeping
    ----------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    -- display read position = pixel_x + 2^s, held inside the line end
    signal px_rd        : unsigned(11 downto 0) := (others => '0');
    signal s_wline      : unsigned(9 downto 0) := to_unsigned(959, 10);
    signal s_wlast      : unsigned(9 downto 0) := to_unsigned(957, 10);
    signal s_row_armed  : std_logic := '1';
    signal s_saw_frame  : std_logic := '0';
    signal s_yf         : unsigned(7 downto 0) := (others => '0');
    signal s_ka         : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal s_spacing    : signed(8 downto 0) := (others => '0');
    signal s_samp_nx    : std_logic := '0';
    signal s_sp_nx      : signed(8 downto 0) := (others => '0');
    signal s_sampled    : std_logic := '0';
    signal s_head       : unsigned(2 downto 0) := (others => '0');
    -- slots at or below the head (the newer run of the rotation)
    signal s_hmask      : std_logic_vector(NB - 1 downto 0) := (others => '0');

    -- virtual x: this pixel advances 2 (SD, every other 720p pixel) or 1
    signal s_xf         : std_logic := '0';
    signal s_twod       : std_logic_vector(4 downto 0) := (others => '0');
    -- vertical-line sampler: virtual pixels to the next sampled column - 1
    signal s_spx        : signed(8 downto 0) := (others => '0');
    signal s_hpulse     : std_logic := '0';
    -- first active row of the field (previous-row displacement restarts);
    -- armed by any line without video, never by the vsync edge
    signal s_saw_av     : std_logic := '0';
    signal s_frow_pend  : std_logic := '1';
    signal s_frow       : std_logic := '1';

    type t_b9  is array (0 to NB - 1) of unsigned(8 downto 0);
    type t_b8  is array (0 to NB - 1) of unsigned(7 downto 0);
    type t_b6  is array (0 to NB - 1) of unsigned(5 downto 0);
    type t_b4  is array (0 to NB - 1) of unsigned(3 downto 0);
    type t_b3  is array (0 to NB - 1) of unsigned(2 downto 0);
    type t_b2  is array (0 to NB - 1) of unsigned(1 downto 0);
    type t_b10s is array (0 to NB - 1) of signed(9 downto 0);
    type t_b4v is array (0 to NB - 1) of std_logic_vector(3 downto 0);
    type t_off is array (0 to NB - 1) of signed(13 downto 0);

    -- bank ages in virtual rows, held at 511 (past any displacement)
    signal s_age   : t_b9 := (others => (others => '0'));
    -- bank read offsets: columns sideways, 10.4 signed
    signal s_off   : t_off := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- Zoom (luma zoom about the picture centre)
    ----------------------------------------------------------------------
    signal s_zoom  : std_logic := '0';
    -- Z = displacement at the top / bottom edge at full zoom amount (rows);
    -- S = Z + 1 = the baseline every line is pushed down by
    signal s_zz    : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_zs    : unsigned(7 downto 0) := to_unsigned(9, 8);
    signal s_xcc   : unsigned(8 downto 0) := to_unsigned(480, 9);  -- centre column
    -- relief scale at W3 (Zoom: scaled by the row's distance from centre)
    -- and "the row is above the centre"
    signal s_scale_w : unsigned(8 downto 0) := to_unsigned(8, 9);
    signal s_rneg    : std_logic := '0';
    signal s_hblank  : std_logic := '0';   -- hsync edge of a line without video
    -- per-row sequencer (runs in horizontal blanking, for the next row)
    signal z_st    : unsigned(3 downto 0) := (others => '0');
    signal z_first : std_logic := '0';
    signal z_k     : unsigned(2 downto 0) := (others => '0');
    signal z_rdk   : unsigned(2 downto 0) := (others => '0');
    signal z_cnt   : unsigned(3 downto 0) := (others => '0');
    -- Z x row, one add of Z per virtual row
    signal z_racc  : unsigned(17 downto 0) := (others => '0');
    -- Y' = row - 512 - S as sign + magnitude
    signal z_yneg  : std_logic := '1';
    signal z_ymag  : unsigned(9 downto 0) := (others => '0');
    signal z_rem   : unsigned(9 downto 0) := (others => '0');
    signal z_q     : unsigned(6 downto 0) := (others => '0');
    signal z_p     : signed(16 downto 0) := (others => '0');     -- -(xc x q)
    signal z_sn    : std_logic := '0';                           -- stepper input
    signal z_sm    : unsigned(9 downto 0) := (others => '0');
    signal z_zero  : std_logic := '0';
    signal z_ovf   : std_logic := '0';
    signal z_clamp : std_logic := '0';
    -- one target bank + operand for the offset registers (sequencer in
    -- blanking, slope carries in the active line)
    signal z_tk    : unsigned(2 downto 0) := (others => '0');
    signal z_op    : signed(13 downto 0) := (others => '0');
    signal z_rs    : std_logic := '0';
    signal z_do    : std_logic := '0';
    -- 256 x 16 RAM, 16 words used.  Words 0..7: per-bank {slope q (7),
    -- offset fraction (8)}, read-modify-written round-robin in the active
    -- line, one bank per pixel (the word written is never the word read: the
    -- read is 4 banks ahead).  Words 8..15: per-bank age - S as sign +
    -- magnitude, stepped by the sequencer.
    type t_zram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_zram  : t_zram := (others => (others => '0'));
    signal zr_a    : unsigned(3 downto 0) := (others => '0');
    signal zr_b    : unsigned(3 downto 0) := (others => '0');
    signal zr_c    : unsigned(3 downto 0) := (others => '0');
    signal zr_v    : std_logic_vector(2 downto 0) := (others => '0');
    signal zr_o    : std_logic_vector(15 downto 0) := (others => '0');
    signal zr_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal zw_we   : std_logic := '0';
    signal zw_a    : unsigned(3 downto 0) := (others => '0');
    signal zw_d    : std_logic_vector(15 downto 0) := (others => '0');
    signal s_inc   : std_logic := '0';   -- the read position advanced
    attribute no_rw_check of r_zram : signal is true;

    signal s_valid : std_logic_vector(NB - 1 downto 0) := (others => '0');
    -- valid AND age /= 0 (the bank being written this row is masked: iCE40
    -- EBR read-during-write returns undefined data)
    signal s_live  : std_logic_vector(NB - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Write path (profile capture)
    ----------------------------------------------------------------------
    signal w0_y    : unsigned(9 downto 0) := (others => '0');
    signal w1_prod : unsigned(17 downto 0) := (others => '0');
    signal w2_acc  : unsigned(13 downto 0) := (others => '0');
    signal w3_prod : unsigned(14 downto 0) := (others => '0');
    signal w3_brt  : unsigned(3 downto 0) := (others => '0');
    signal w3_brtv : unsigned(3 downto 0) := (others => '0');
    signal w4_data : std_logic_vector(11 downto 0) := (others => '0');
    signal w4_addr : unsigned(9 downto 0) := (others => '0');
    signal w4_we   : std_logic := '0';
    signal w4_dv   : unsigned(7 downto 0) := (others => '0');
    signal w4_vblk : std_logic := '0';   -- vertical-line sample is black and hidden
    -- Banded shading: the sample is black (below 1/16) or one of the first
    -- three after black; samples still to go before the filter runs again
    signal w2_blk  : std_logic := '0';
    signal w2_since : unsigned(1 downto 0) := "00";

    signal w_av   : std_logic_vector(3 downto 0) := (others => '0');
    signal w_avd  : std_logic := '0';
    -- pixel count at the W3 stage (column address + parity of the write)
    signal w_cnt  : unsigned(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Vertical lines (Weave): sampled-column flags, per-column RAM (the
    -- displacement drawn on the previous row; 256 x 16 = 1 EBR; the read
    -- address is the next column to sample and the write is the column
    -- sampled one clock earlier, so they never collide), candidates
    ----------------------------------------------------------------------
    signal w0_vs, w1_vs, w2_vs, w3_vs, w4_vs, w5_vs, w6_vs : std_logic := '0';
    signal s_colv  : unsigned(7 downto 0) := (others => '0');
    signal cw_addr : unsigned(7 downto 0) := (others => '0');
    type t_vram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_vram : t_vram := (others => (others => '0'));
    signal r_crd  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_cq   : std_logic_vector(15 downto 0) := (others => '0');
    attribute no_rw_check of r_vram : signal is true;
    -- slots 4..7 hold a vertical-line candidate (Weave)
    signal c_val  : std_logic_vector(NB - 1 downto NVS) := (others => '0');

    ----------------------------------------------------------------------
    -- Read-side position flag (aligned with the bank read pipeline)
    ----------------------------------------------------------------------
    signal r_ph    : std_logic_vector(3 downto 1) := (others => '0');
    -- slots 4..7: near end = previous / far end = this (both, in Weave)
    signal r_pha   : std_logic := '0';
    signal r_phb   : std_logic := '0';

    ----------------------------------------------------------------------
    -- Bank read data (A2 fabric registers, after the EBR output register)
    ----------------------------------------------------------------------
    signal a2_disp : t_b8 := (others => (others => '0'));
    -- brightness nibble at each bank's EBR output; it rides a block-RAM delay
    -- (below) to the resolve stage instead of five register stages per bank
    signal r_bbrt  : t_b4 := (others => (others => '0'));
    -- slots 4..7 in Weave: the vertical-line candidates' brightness
    signal s_vb    : t_b4 := (others => (others => '0'));
    signal a2_nz   : std_logic_vector(NB - 1 downto 0) := (others => '0');
    -- the word's brightness is 0 (black source): not drawn in Banded shading
    signal a2_blk  : std_logic_vector(NB - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Connected-stroke segment (B1..B3) + per-bank row state
    ----------------------------------------------------------------------
    signal s_seen  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal s_past  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    -- the previous column was drawable (else the stroke restarts flat)
    signal s_pok   : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal p_d1    : t_b8 := (others => (others => '0'));  -- previous column
    signal b2_ea   : t_b8 := (others => (others => '0'));
    signal b2_eb   : t_b8 := (others => (others => '0'));
    signal b2_v    : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal b3_lo   : t_b8 := (others => (others => '0'));
    signal b3_hi   : t_b8 := (others => (others => '0'));
    signal b3_span : t_b8 := (others => (others => '0'));
    signal b3_v    : std_logic_vector(NB - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Display pipeline
    ----------------------------------------------------------------------
    signal a3_slo  : t_b10s := (others => (others => '0'));
    signal a3_shi  : t_b10s := (others => (others => '0'));   -- far end + T
    signal a3_dim  : t_b2 := (others => (others => '0'));   -- "11" = dead
    signal a3_g    : t_b6 := (others => (others => '0'));   -- node-grid phase

    signal a4_hit  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_cov  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_rung : std_logic_vector(NB - 1 downto 0) := (others => '0');
    -- dim shift as a thermometer (bit i = shift <= i), so OR = the brighter
    signal a4_sh   : t_b4v := (others => (others => '0'));

    signal a5_win  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a5_hit  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a5_rung : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a5_sh   : t_b4v := (others => (others => '0'));
    signal a5_brt  : t_b4 := (others => (others => '0'));

    signal a6_line  : std_logic := '0';
    signal a6_isv   : std_logic := '0';
    signal a6_sh    : std_logic_vector(3 downto 0) := (others => '0');
    signal a6_brt   : unsigned(3 downto 0) := (others => '0');

    signal a7_bf1   : unsigned(5 downto 0) := (others => '0');
    signal a7_alt   : unsigned(2 downto 0) := (others => '0');
    signal a7_line  : std_logic := '0';
    signal a7_cu    : signed(9 downto 0) := (others => '0');
    signal a7_cv    : signed(9 downto 0) := (others => '0');

    signal a8_yo    : unsigned(9 downto 0) := (others => '0');
    signal a8_uo    : signed(9 downto 0) := (others => '0');
    signal a8_vo    : signed(9 downto 0) := (others => '0');
    signal a8_bf1   : unsigned(5 downto 0) := (others => '0');
    signal a8_line  : std_logic := '0';

    signal a9_ym    : unsigned(15 downto 0) := (others => '0');
    signal a9_um    : signed(16 downto 0) := (others => '0');
    signal a9_vm    : signed(16 downto 0) := (others => '0');
    signal a9_line  : std_logic := '0';

    signal a10_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal a10_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a10_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: [1 2 1]/4 window (f1 = centre, f2 = oldest)
    signal f1_y, f2_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal f1_u, f2_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f1_v, f2_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f3_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal f3_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f3_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal a11_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal a11_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a11_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- Sync delay line (tap C_LAT-2 gates blanking at A11, tap C_LAT-1 drives
    -- sync out)
    ----------------------------------------------------------------------
    type t_dly is record
        avid    : std_logic;
        hsync_n : std_logic;
        vsync_n : std_logic;
        field_n : std_logic;
    end record;
    type t_dly_arr is array (0 to C_LAT - 1) of t_dly;
    signal s_dly : t_dly_arr := (others =>
        (avid => '0', hsync_n => '1', vsync_n => '1', field_n => '1'));

    ----------------------------------------------------------------------
    -- Input-chroma delay for the Video line colour: 8 + 8 bits in a 256 x 16
    -- circular buffer (1 EBR).  The read pointer trails the write pointer by
    -- a constant, recomputed every clock, so the two can never drift apart;
    -- the word written is never the word read.
    ----------------------------------------------------------------------
    type t_cdl is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_cdl  : t_cdl := (others => x"8080");
    signal s_cwp  : unsigned(7 downto 0) := (others => '0');
    signal s_crp  : unsigned(7 downto 0) := (others => '0');
    signal r_cdo  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_cdq  : std_logic_vector(15 downto 0) := x"8080";
    attribute no_rw_check of r_cdl : signal is true;
    -- candidate brightness delay: 8 x 4 bits, bank EBR output -> A5, in the
    -- same kind of circular buffer (256 x 32 = 2 EBR, same write pointer)
    type t_bdl is array (0 to 255) of std_logic_vector(31 downto 0);
    signal r_bdl  : t_bdl := (others => (others => '0'));
    signal s_brp  : unsigned(7 downto 0) := (others => '0');
    signal r_bdo  : std_logic_vector(31 downto 0) := (others => '0');
    attribute no_rw_check of r_bdl : signal is true;

    ----------------------------------------------------------------------
    -- Line palettes.  10-bit YUV with U/V pre-swapped for the hardware
    -- encoder (U slot carries Cr): Y stored as offset-from-black (yo, so
    -- 64 + yo*16/16 <= 768, respecting the drawn-luma cap), chroma as
    -- signed offsets from 512.  Zone 5 (Video) takes the source chroma
    -- instead, which is already in hardware order (dry path: no swap).
    ----------------------------------------------------------------------
    type t_pal10 is array (0 to 7) of unsigned(9 downto 0);
    type t_pal10s is array (0 to 7) of signed(9 downto 0);
    -- zones 0..6: White, Green, Amber, Cyan, Magenta, Video, Fire (7=Terrain)
    constant C_ZY : t_pal10 := (to_unsigned(704, 10), to_unsigned(587, 10),
        to_unsigned(647, 10), to_unsigned(614, 10), to_unsigned(516, 10),
        to_unsigned(620, 10), to_unsigned(400, 10), to_unsigned(704, 10));
    constant C_ZU : t_pal10s := (to_signed(0, 10), to_signed(-300, 10),
        to_signed(166, 10), to_signed(-448, 10), to_signed(335, 10),
        to_signed(0, 10), to_signed(347, 10), to_signed(0, 10));
    constant C_ZV : t_pal10s := (to_signed(0, 10), to_signed(-237, 10),
        to_signed(-373, 10), to_signed(151, 10), to_signed(208, 10),
        to_signed(0, 10), to_signed(-181, 10), to_signed(0, 10));
    -- Terrain tint, indexed by the line's luma band: 0 = least sag (peak,
    -- white-hot) .. 7 = deepest sag (valley, dim blue)
    constant C_TY : t_pal10 := (to_unsigned(704, 10), to_unsigned(585, 10),
        to_unsigned(682, 10), to_unsigned(604, 10), to_unsigned(468, 10),
        to_unsigned(414, 10), to_unsigned(271, 10), to_unsigned(111, 10));
    constant C_TU : t_pal10s := (to_signed(30, 10), to_signed(212, 10),
        to_signed(77, 10), to_signed(-121, 10), to_signed(-277, 10),
        to_signed(-238, 10), to_signed(-133, 10), to_signed(-81, 10));
    constant C_TV : t_pal10s := (to_signed(-52, 10), to_signed(-262, 10),
        to_signed(-343, 10), to_signed(-297, 10), to_signed(-118, 10),
        to_signed(165, 10), to_signed(299, 10), to_signed(239, 10));

    ----------------------------------------------------------------------
    -- Candidate classifier.  slo = age minus the segment's near end; e = age
    -- minus its far end minus the stroke thickness T.  The glow band is 4 or
    -- 8 units wide (w8), so every test is a sign or a few-bit pattern: no
    -- comparators.
    --   cov  : at or past the line (skirt)        slo >= 0
    --   core : inside the stroke                  cov and e < 0
    --   halo : glow band either side              -W <= slo < 0, 0 <= e < W
    --   sh   : dim shift (beam speed + glow level; inner half of the halo
    --          one step brighter than the outer half), as a thermometer
    -- dim = "11" marks a dead candidate.
    ----------------------------------------------------------------------
    procedure f_class(
        slo, e   : in signed(9 downto 0);
        live     : in std_logic;
        dim      : in unsigned(1 downto 0);
        w8       : in std_logic;
        glow     : in std_logic;
        gsh      : in unsigned(2 downto 0);
        hit, cov : out std_logic;
        core     : out std_logic;
        sh       : out std_logic_vector(3 downto 0)) is
        variable v_alive, v_lonn, v_eneg : boolean;
        variable v_a8, v_a4, v_a2 : boolean;    -- slo in [-8,0) / [-4,0) / [-2,0)
        variable v_b8, v_b4, v_b2 : boolean;    -- e   in [0,8)  / [0,4)  / [0,2)
        variable v_ha, v_hb, v_ia, v_ib : boolean;
        variable v_cv, v_core, v_halo, v_hin : boolean;
        variable v_sh  : unsigned(2 downto 0);
    begin
        v_alive := live = '1' and dim /= "11";
        v_lonn  := slo(9) = '0';
        v_eneg  := e(9) = '1';
        v_a8    := slo(9 downto 3) = "1111111";
        v_a4    := v_a8 and slo(2) = '1';
        v_a2    := v_a4 and slo(1) = '1';
        v_b8    := e(9 downto 3) = "0000000";
        v_b4    := v_b8 and e(2) = '0';
        v_b2    := v_b4 and e(1) = '0';
        if w8 = '1' then
            v_ha := v_a8; v_hb := v_b8; v_ia := v_a4; v_ib := v_b4;
        else
            v_ha := v_a4; v_hb := v_b4; v_ia := v_a2; v_ib := v_b2;
        end if;
        v_cv    := v_alive and v_lonn;
        v_core  := v_cv and v_eneg;
        v_halo  := glow = '1' and v_alive and (v_ha or v_hb);
        v_hin   := (v_lonn or v_ia) and (v_eneg or v_ib);
        if v_core or v_halo then hit := '1'; else hit := '0'; end if;
        if v_cv then cov := '1'; else cov := '0'; end if;
        if v_core then core := '1'; else core := '0'; end if;
        v_sh := resize(dim, 3);
        if v_halo and not v_core then
            if v_hin then
                v_sh := v_sh + gsh;
            else
                v_sh := v_sh + gsh + 1;
            end if;
        end if;
        case to_integer(v_sh) is
            when 0      => sh := "1111";
            when 1      => sh := "1110";
            when 2      => sh := "1100";
            when 3      => sh := "1000";
            when others => sh := "0000";
        end case;
    end procedure;

begin

    ----------------------------------------------------------------------
    -- Position counters, per-row bank bookkeeping, vsync parameter latch
    ----------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_samp    : std_logic;
        variable v_newhead : unsigned(2 downto 0);
        variable v_inv     : unsigned(9 downto 0);
        variable v_d7      : unsigned(6 downto 0);
        variable v_dm      : unsigned(7 downto 0);
        variable v_dv      : unsigned(8 downto 0);
        variable v_d10     : unsigned(9 downto 0);
        variable v_targ    : unsigned(9 downto 0);
        variable v_diff    : signed(10 downto 0);
        variable v_next    : signed(10 downto 0);
        variable v_se      : unsigned(14 downto 0);
        variable v_sp      : signed(8 downto 0);
        variable v_ag      : unsigned(10 downto 0);
        variable v_opa     : unsigned(8 downto 0);
        variable v_opb     : unsigned(2 downto 0);
        variable v_sum     : unsigned(9 downto 0);
        variable v_t       : unsigned(3 downto 0);
        variable v_tw      : unsigned(3 downto 0);
        variable v_two     : std_logic;
        variable v_a, v_b  : signed(8 downto 0);
        variable v_ls      : signed(6 downto 0);
        variable v_c9      : unsigned(8 downto 0);
        variable v_k9      : unsigned(13 downto 0);
        variable v_z8, v_zc : unsigned(7 downto 0);
        variable v_orst, v_oadd : boolean;
        variable v_obus    : signed(13 downto 0);
        variable v_rowst   : boolean;
    begin
        if rising_edge(clk) then
            if unsigned(registers_in(2)) < 8 then
                s_k3_idx <= unsigned(registers_in(2)(2 downto 0));
            else
                s_k3_idx <= unsigned(registers_in(2)(9 downto 7));
            end if;
            -- K5 has nine labels: index = value x 9 / 1024
            if unsigned(registers_in(4)) < 9 then
                s_k5_idx <= unsigned(registers_in(4)(3 downto 0));
            else
                v_k9 := shift_left(resize(unsigned(registers_in(4)), 14), 3) +
                        resize(unsigned(registers_in(4)), 14);
                s_k5_idx <= v_k9(13 downto 10);
            end if;
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            -- Timing class (free-running from the timing ID): y step per
            -- field row, x step, and the slant-1 sideways step per field row
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 =>                                        -- 480i
                    s_ys <= to_unsigned(1138, 12); s_kc <= to_unsigned(5, 4);
                    s_sd <= '1'; s_720 <= '0'; s_kcx <= to_unsigned(2, 4);
                    s_lbase <= to_unsigned(18, 5);
                when 4 =>                                        -- 480p
                    s_ys <= to_unsigned(576, 12); s_kc <= to_unsigned(3, 4);
                    s_sd <= '1'; s_720 <= '0'; s_kcx <= to_unsigned(2, 4);
                    s_lbase <= to_unsigned(9, 5);
                when 8 =>                                        -- 576i
                    s_ys <= to_unsigned(960, 12); s_kc <= to_unsigned(4, 4);
                    s_sd <= '1'; s_720 <= '0'; s_kcx <= to_unsigned(2, 4);
                    s_lbase <= to_unsigned(15, 5);
                when 12 =>                                       -- 576p
                    s_ys <= to_unsigned(480, 12); s_kc <= to_unsigned(2, 4);
                    s_sd <= '1'; s_720 <= '0'; s_kcx <= to_unsigned(2, 4);
                    s_lbase <= to_unsigned(8, 5);
                when 5 | 6 | 14 =>                               -- 720p
                    s_ys <= to_unsigned(384, 12); s_kc <= to_unsigned(2, 4);
                    s_sd <= '0'; s_720 <= '1'; s_kcx <= to_unsigned(2, 4);
                    s_lbase <= to_unsigned(8, 5);
                when 1 | 2 | 10 =>                               -- 1080i
                    s_ys <= to_unsigned(512, 12); s_kc <= to_unsigned(2, 4);
                    s_sd <= '0'; s_720 <= '0'; s_kcx <= to_unsigned(1, 4);
                    s_lbase <= to_unsigned(16, 5);
                when others =>                                   -- 1080p
                    s_ys <= to_unsigned(256, 12); s_kc <= to_unsigned(1, 4);
                    s_sd <= '0'; s_720 <= '0'; s_kcx <= to_unsigned(1, 4);
                    s_lbase <= to_unsigned(8, 5);
            end case;

            -- the next active row's virtual-row advance and sampled-row
            -- decision, precomputed every clock (rows are >= 858 clocks apart)
            v_ag := resize(s_yf, 11) + resize(s_ys, 11);
            s_ka <= v_ag(10 downto 8);
            v_sp := s_spacing - signed(resize(s_ka, 9));
            if v_sp <= 0 then
                s_samp_nx <= '1';
                s_sp_nx   <= v_sp + signed(resize(s_dspace, 9));
            else
                s_samp_nx <= '0';
                s_sp_nx   <= v_sp;
            end if;

            -- stroke window of slots 4..7 (pixels in Weave)
            if s_weave = '1' then
                s_tb <= s_thkx;
            else
                s_tb <= s_thkw;
            end if;
            -- Slant centring.  Mid-grey is displaced by scale_eff rows (at
            -- most half the 255-row reach); the read column is moved by that
            -- many rows' worth of slant, so mid-grey material stays on its
            -- source column and bright / dark lean opposite ways.  Columns per
            -- row of displacement at slant 1: 1/2 (1080), 3/8 (720p), 1/4 (SD).
            if s_scale_eff(8 downto 7) /= "00" then
                s_d0 <= to_unsigned(128, 8);
            else
                s_d0 <= '0' & s_scale_eff(6 downto 0);
            end if;
            if s_lean2 = '1' then
                v_c9 := s_d0 & '0';
            else
                v_c9 := '0' & s_d0;
            end if;
            if s_sd = '1' then
                v_c9 := shift_right(v_c9, 2);
            elsif s_720 = '1' then
                v_c9 := shift_right(v_c9, 2) + shift_right(v_c9, 3);
            else
                v_c9 := shift_right(v_c9, 1);
            end if;
            if s_lon = '0' then
                s_ctr <= (others => '0');
            elsif s_lneg = '1' then
                s_ctr <= -signed(resize(v_c9, 10));
            else
                s_ctr <= signed(resize(v_c9, 10));
            end if;
            s_pxc0 <= signed(resize(s_lagp1(7 downto 1), 11)) + resize(s_ctr, 11);
            -- Zoom constants.  Z = the displacement a full-amount point gets
            -- 512 rows from the centre: half the relief scale, held 8 short
            -- of half the retrievable window (lines move both ways about the
            -- baseline S = Z + 1, and the bottom rows are up to 568 rows out).
            v_z8 := s_scale_eff(8 downto 1);
            v_zc := ('0' & s_dmax(7 downto 1)) - 8;
            if v_z8 > v_zc then
                v_z8 := v_zc;
            end if;
            s_zz  <= v_z8(6 downto 0);
            s_zs  <= v_z8 + 1;
            -- node-grid phase start (kept on the source columns: minus the
            -- centring); a right slant counts down from Tx - 1
            if s_rdown = '1' then
                s_xrinit <= resize(s_thkx, 6) - 1 - s_d0(5 downto 0);
            else
                s_xrinit <= to_unsigned(0, 6) - s_d0(5 downto 0);
            end if;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- this pixel's virtual x advance: 2 at SD, alternating 1/2 at 720p
            v_two := s_sd or (s_720 and s_xf);
            s_twod <= s_twod(3 downto 0) & v_two;

            -- vertical-line sampler: both outcomes in parallel
            if v_two = '1' then
                v_a := s_spx - 2;
                v_b := s_spx + s_dm2;
            else
                v_a := s_spx - 1;
                v_b := s_spx + s_dm1;
            end if;
            w0_vs    <= '0';
            s_hpulse <= v_h_edge;
            s_hblank <= v_h_edge and not s_saw_av;
            -- the read position advances this clock
            if v_h_edge = '0' and data_in.avid = '1' and
               (px_rd(10 downto 1) < s_wlast or px_rd(0) = '0') then
                s_inc <= '1';
            else
                s_inc <= '0';
            end if;

            if v_h_edge = '1' then
                pixel_x     <= (others => '0');
                px_rd       <= resize(s_lagp1, 12);
                s_pxc       <= s_pxc0;
                s_row_armed <= '1';
                s_xf        <= '0';
                -- roll: first sampled column = phase x spacing
                s_spx       <= signed(resize(s_roff, 9));
                -- a line without video = vertical blanking: the next active
                -- row is the field's first
                if s_saw_av = '0' then
                    s_frow_pend <= '1';
                end if;
                s_saw_av <= '0';
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
                -- (the hold lands on the second pixel of its column)
                if px_rd(10 downto 1) < s_wlast or px_rd(0) = '0' then
                    px_rd <= px_rd + 1;
                    if px_rd(0) = '1' then
                        s_pxc <= s_pxc + 1;
                    end if;
                end if;
                s_wline  <= pixel_x(10 downto 1);   -- ends at the last column
                s_saw_av <= '1';
                s_xf     <= not s_xf;
                if v_a(8) = '1' then
                    s_spx <= v_b;
                    w0_vs <= s_weave;
                else
                    s_spx <= v_a;
                end if;
            end if;

            if data_in.avid = '1' then s_saw_frame <= '1'; end if;

            -- Quasi-static products (inputs change only at vsync), on one
            -- free-running serial multiplier, alternately:
            --   relief scale x window normaliser, and roll phase x spacing
            if m_cnt = 0 then
                if m_job = '0' then
                    s_scale_eff <= m_acc(13 downto 5);
                    m_sh <= s_rphase(15 downto 8);
                else
                    s_roff <= m_acc(14 downto 8);
                    m_sh <= "00" & s_rmul;
                end if;
                m_job <= not m_job;
                m_acc <= (others => '0');
                m_cnt <= to_unsigned(8, 4);
            else
                v_se := m_acc(13 downto 0) & '0';
                if m_sh(7) = '1' then
                    if m_job = '0' then
                        v_se := v_se + resize(s_scale9, 15);
                    else
                        v_se := v_se + resize(s_dspace, 15);
                    end if;
                end if;
                m_acc <= v_se;
                m_sh  <= m_sh(6 downto 0) & '0';
                m_cnt <= m_cnt - 1;
            end if;

            -- Row-start bookkeeping: fires on the FIRST active pixel of each
            -- row.  Everything decided here is first consumed >= 3 pipeline
            -- stages later, so the one-cycle registration lag is safe.
            v_rowst := data_in.avid = '1' and s_row_armed = '1';
            v_samp  := s_samp_nx;
            if v_samp = '1' then
                v_newhead := s_head + 1;
            else
                v_newhead := s_head;
            end if;
            if s_weave = '1' then
                v_newhead(2) := '0';         -- Weave: the scan lines keep banks 0..3
            end if;
            if v_rowst then
                s_row_armed <= '0';
                s_frow      <= s_frow_pend;
                s_frow_pend <= '0';
                v_ag := resize(s_yf, 11) + resize(s_ys, 11);
                s_yf <= v_ag(7 downto 0);
                s_spacing <= s_sp_nx;
                s_sampled <= v_samp;
                if v_samp = '1' then
                    s_rowtop   <= s_firstrow;
                    s_firstrow <= '0';
                end if;
                s_head <= v_newhead;
                for m in 0 to NB - 1 loop
                    if m <= to_integer(v_newhead) and (s_weave = '0' or m < NVS) then
                        s_hmask(m) <= '1';
                    else
                        s_hmask(m) <= '0';
                    end if;
                end loop;
            end if;

            -- Candidate ages.  A bank's age counts virtual rows from its
            -- sampled row (stepped at row start), held at 511; its slant
            -- offset steps beside it while the age is below 256 (past any
            -- displacement the bank is pure skirt).
            -- In Weave, slots 4..7 carry vertical lines instead: their age
            -- counts virtual PIXELS from the sampled column, every clock, and
            -- shifts down a slot (7 newest) when a new column arrives.  One
            -- saturating adder per slot serves both.
            if s_zoom = '1' then
                v_obus := z_op;
            else
                v_obus := resize(s_lstep, 14);
            end if;
            for k in 0 to NB - 1 loop
                v_opa := s_age(k);
                v_opb := s_ka;
                if k >= NVS and s_weave = '1' then
                    if w6_vs = '1' and k < NB - 1 then
                        v_opa := s_age((k + 1) mod NB);
                    end if;
                    if s_twod(3) = '1' then
                        v_opb := "010";
                    else
                        v_opb := "001";
                    end if;
                end if;
                v_sum := resize(v_opa, 10) + resize(v_opb, 10);
                if v_sum(9) = '1' then
                    v_sum := to_unsigned(511, 10);
                end if;
                if k >= NVS and s_weave = '1' then
                    if w6_vs = '1' and k = NB - 1 then
                        s_age(k) <= (others => '0');
                    else
                        s_age(k) <= v_sum(8 downto 0);
                    end if;
                elsif v_rowst then
                    if v_samp = '1' and k = to_integer(v_newhead) then
                        s_age(k)   <= (others => '0');
                        s_valid(k) <= '1';
                        s_live(k)  <= '0';
                    elsif s_valid(k) = '1' then
                        s_age(k) <= v_sum(8 downto 0);
                        s_live(k) <= '1';
                    end if;
                end if;

                -- Sideways read offset, one adder per bank on a shared
                -- operand.  Slant: cleared with the bank, then one step per
                -- row.  Zoom: the sequencer loads the offset of the left edge
                -- in blanking, then the bank's slope carries add a column.
                v_orst := false;
                v_oadd := false;
                if s_zoom = '1' then
                    v_orst := z_rs = '1' and to_integer(z_tk) = k;
                    v_oadd := z_do = '1' and to_integer(z_tk) = k;
                elsif v_rowst then
                    if v_samp = '1' and k = to_integer(v_newhead) then
                        v_orst := true;
                    elsif s_valid(k) = '1' and s_age(k)(8) = '0' then
                        v_oadd := true;
                    end if;
                end if;
                if v_orst then
                    s_off(k) <= (others => '0');
                elsif v_oadd then
                    s_off(k) <= s_off(k) + v_obus;
                end if;
            end loop;

            -- Field/frame start.  Analog vsync serrates (several edges per
            -- field): every action here is idempotent, and the relief glide
            -- and roll step are guarded to one step per field by s_saw_frame.
            if v_v_edge = '1' then
                s_valid    <= (others => '0');
                s_live     <= (others => '0');
                -- roll: first sampled row = phase x spacing; the field's top
                -- line fades in with the phase so lines never pop in
                -- (+ the first row's advance: the row-start test subtracts it)
                s_spacing  <= signed(resize(s_roff, 9)) + signed(resize(s_ys(11 downto 8), 9));
                s_yf       <= (others => '0');
                s_fade4    <= s_rphase(15 downto 12);
                s_firstrow <= '1';
                -- the read-ahead holds two columns inside the measured line
                -- end, so a line a pixel short never reads its end marker
                s_wlast    <= s_wline - 2;
                s_xcc      <= resize(shift_right(s_wline + 1, 1), 9);

                -- K1 Scan Lines: CW = denser.  Spacing 8..71 rows.
                v_inv := 1023 - unsigned(registers_in(0));
                v_d7  := to_unsigned(8, 7) + v_inv(9 downto 4);
                s_dspace <= v_d7;
                s_dm1 <= signed(resize(v_d7, 9)) - 1;
                s_dm2 <= signed(resize(v_d7, 9)) - 2;
                -- Max displacement: a line stays retrievable while its bank /
                -- candidate lives: 8 samples (4 per family in Weave), storage
                -- caps 255.  The hold stops 16 units short of that window, so
                -- the whole stroke of a line at full displacement still lands
                -- inside it (on every raster: a field row spans up to 5
                -- units).  Relief is normalised to the window.
                v_d10 := shift_left(resize(v_d7, 10), 3) - 16;
                if v_d10 > 255 then
                    v_dm := to_unsigned(255, 8);
                else
                    v_dm := v_d10(7 downto 0);
                end if;
                v_dv := shift_left(resize(v_d7, 9), 2) - 16;
                if v_dv > 255 then
                    v_dv := to_unsigned(255, 9);
                end if;
                s_dmaxv <= v_dv(7 downto 0);
                if s_k5_idx = 7 then
                    -- Weave: 4 x spacing, unity at spacing 64
                    s_dmax <= v_dv(7 downto 0);
                    if v_d7 >= 64 then
                        s_rmul <= to_unsigned(32, 6);
                    else
                        s_rmul <= resize(v_d7(6 downto 1), 6);
                    end if;
                else
                    s_dmax <= v_dm;
                    if v_d7 >= 32 then
                        s_rmul <= to_unsigned(32, 6);
                    else
                        s_rmul <= resize(v_d7, 6);
                    end if;
                end if;
                -- node grid: the power of two nearest the spacing
                if v_d7 < 24 then
                    s_gmask <= "00";      -- 16
                elsif v_d7 < 48 then
                    s_gmask <= "01";      -- 32
                else
                    s_gmask <= "11";      -- 64
                end if;

                v_t := to_unsigned(1, 4) + resize(unsigned(registers_in(1)(9 downto 7)), 4);
                s_thick <= v_t;
                v_tw := v_t;
                if v_tw < s_kc then v_tw := s_kc; end if;
                s_thkw <= v_tw;
                v_tw := v_t;
                if v_tw < s_kcx then v_tw := s_kcx; end if;
                s_thkx <= v_tw;
                case registers_in(1)(9 downto 7) is
                    when "000"  => s_bboost <= "11";   -- T=1: +3
                    when "001"  => s_bboost <= "10";   -- T=2: +2
                    when "010"  => s_bboost <= "01";   -- T=3: +1
                    when others => s_bboost <= "00";
                end case;
                -- thin-stroke gain: stroke width in pixels is T at 1080,
                -- about T/1.5 at 720p and T/2 at SD (window >= 1 pixel)
                s_tgain <= "00";
                if s_sd = '1' then
                    case registers_in(1)(9 downto 7) is
                        when "000" | "001" => s_tgain <= "10";
                        when "010" | "011" => s_tgain <= "01";
                        when others        => null;
                    end case;
                elsif s_720 = '1' then
                    case registers_in(1)(9 downto 7) is
                        when "000" | "001" => s_tgain <= "10";
                        when "010"         => s_tgain <= "01";
                        when others        => null;
                    end case;
                else
                    case registers_in(1)(9 downto 7) is
                        when "000"  => s_tgain <= "10";
                        when "001"  => s_tgain <= "01";
                        when others => null;
                    end case;
                end if;
                s_zone   <= s_k3_idx;
                s_gain8  <= unsigned(registers_in(3)(9 downto 2));
                s_smooth <= unsigned(registers_in(5)(9 downto 7));
                case registers_in(5)(9 downto 7) is
                    when "000"  => s_lagp1 <= to_unsigned(1, 8);
                    when "001"  => s_lagp1 <= to_unsigned(2, 8);
                    when "010"  => s_lagp1 <= to_unsigned(4, 8);
                    when "011"  => s_lagp1 <= to_unsigned(8, 8);
                    when "100"  => s_lagp1 <= to_unsigned(16, 8);
                    when "101"  => s_lagp1 <= to_unsigned(32, 8);
                    when "110"  => s_lagp1 <= to_unsigned(64, 8);
                    when others => s_lagp1 <= to_unsigned(128, 8);
                end case;

                -- K5 Mode
                v_ls := signed(resize(s_lbase, 7));
                s_weave <= '0';
                s_mesh  <= '0';
                s_rdown <= '0';
                s_lon   <= '0';
                s_zoom  <= '0';
                case to_integer(s_k5_idx) is
                    when 0 =>                          -- Mesh <
                        s_lstep <= -v_ls;
                        s_mesh  <= '1';
                        s_lon <= '1'; s_lneg <= '1'; s_lean2 <= '0';
                    when 1 =>                          -- Slant <<
                        s_lstep <= -shift_left(v_ls, 1);
                        s_lon <= '1'; s_lneg <= '1'; s_lean2 <= '1';
                    when 2 =>                          -- Slant <
                        s_lstep <= -v_ls;
                        s_lon <= '1'; s_lneg <= '1'; s_lean2 <= '0';
                    when 3 =>                          -- Classic
                        s_lstep <= (others => '0');
                    when 4 =>                          -- Slant >
                        s_lstep <= v_ls;
                        s_lon <= '1'; s_lneg <= '0'; s_lean2 <= '0';
                    when 5 =>                          -- Slant >>
                        s_lstep <= shift_left(v_ls, 1);
                        s_lon <= '1'; s_lneg <= '0'; s_lean2 <= '1';
                    when 6 =>                          -- Mesh >
                        s_lstep <= v_ls;
                        s_mesh  <= '1';
                        s_rdown <= '1';
                        s_lon <= '1'; s_lneg <= '0'; s_lean2 <= '0';
                    when 7 =>                          -- Weave
                        s_lstep <= (others => '0');
                        s_weave <= '1';
                    when others =>                     -- Zoom
                        s_lstep <= (others => '0');
                        s_zoom  <= '1';
                end case;

                s_peakdark <= registers_in(6)(0);
                s_hlr      <= registers_in(6)(1);
                s_shade    <= registers_in(6)(2);
                s_rfreeze  <= registers_in(6)(3);
                case registers_in(1)(9 downto 7) is
                    when "000" | "001" =>
                        s_glow <= '0'; s_gsh <= "010";
                    when "010" | "011" =>
                        s_glow <= '1'; s_gsh <= "010";
                    when others =>
                        s_glow <= '1'; s_gsh <= "001";
                end case;
                -- glow band: 4 units up to T=5, 8 from T=6 (and wherever a
                -- field row spans 4+ units, so the band always lands on one)
                if unsigned(registers_in(1)(9 downto 7)) >= 5 or s_kc >= 4 then
                    s_w8 <= '1';
                else
                    s_w8 <= '0';
                end if;
                -- beam-speed dimming thresholds: 2 / 8 (T=1), 4 / 16 (T=2..4),
                -- 8 / 32 (T=5..8)
                case registers_in(1)(9 downto 7) is
                    when "000"                 => s_dn <= "01";
                    when "001" | "010" | "011" => s_dn <= "10";
                    when others                => s_dn <= "11";
                end case;
                s_neg      <= registers_in(6)(4);

                -- P12 Relief -> displacement scale, glided (ease by diff/4
                -- with a +/-1 deadband) so pot ADC noise cannot make the
                -- whole line field breathe.  Roll: a fixed slow drift.
                if s_saw_frame = '1' then
                    s_saw_frame <= '0';
                    if s_rfreeze = '0' then
                        s_rphase <= s_rphase + C_RSPD;
                    end if;
                    v_targ := to_unsigned(8, 10) + resize(unsigned(registers_in(7)(9 downto 2)), 10);
                    v_diff := signed(resize(v_targ, 11)) - signed(resize(s_scale9, 11));
                    v_next := signed(resize(s_scale9, 11));
                    if v_diff > 3 then
                        v_next := v_next + shift_right(v_diff, 2);
                    elsif v_diff > 1 then
                        v_next := v_next + 1;
                    elsif v_diff < -3 then
                        v_next := v_next + shift_right(v_diff, 2);
                    elsif v_diff < -1 then
                        v_next := v_next - 1;
                    end if;
                    s_scale9 <= unsigned(v_next(8 downto 0));
                end if;
            end if;
        end if;
    end process p_position;

    ----------------------------------------------------------------------
    -- Write path: luma -> (negative) -> contrast gain -> smoothing IIR ->
    -- displacement + brightness, written into bank[head] on sampled rows
    -- (even columns only; the store is column-decimated x2), followed by one
    -- zero word that marks the end of the row.
    ----------------------------------------------------------------------
    p_write : process(clk)
        variable v_arg  : unsigned(9 downto 0);
        variable v_l11  : unsigned(10 downto 0);
        variable v_lum  : unsigned(9 downto 0);
        variable v_l14, v_b14, v_s14, v_n14, v_x14, v_c14 : unsigned(13 downto 0);
        variable v_sum  : unsigned(14 downto 0);
        variable v_d6   : unsigned(5 downto 0);
        variable v_bm   : unsigned(7 downto 0);
        variable v_dr   : unsigned(9 downto 0);
        variable v_disp : unsigned(7 downto 0);
        variable v_m8   : unsigned(7 downto 0);
        variable v_blk  : std_logic;
        variable v_dead : std_logic;
    begin
        if rising_edge(clk) then
            -- W0: input register + S11 negative, less black (64), held at 0
            if s_neg = '1' then
                v_arg := not unsigned(data_in.y);
            else
                v_arg := unsigned(data_in.y);
            end if;
            if v_arg(9 downto 6) = "0000" then
                w0_y <= (others => '0');
            else
                w0_y <= (v_arg(9 downto 6) - 1) & v_arg(5 downto 0);
            end if;
            w_av(0)   <= data_in.avid;

            -- W1: contrast gain (unity at knob centre: x gain8/128, 0..2x);
            -- the multiply has the stage to itself
            w1_prod   <= w0_y * s_gain8;
            w_av(1)   <= w_av(0);

            -- W2: clamp + horizontal smoothing IIR (14-bit accumulator with
            -- 4 fractional bits; reset to the first sample of each row):
            --   acc <= acc - (acc >> s) + (lum >> s)
            -- as one carry chain behind a 3:2 compressor (acc + b + not(sh) + 1),
            -- so the feedback loop is a shift, one LUT and one add.
            v_l11 := w1_prod(17 downto 7);
            if v_l11 > 1023 then
                v_lum := to_unsigned(1023, 10);
            else
                v_lum := v_l11(9 downto 0);
            end if;
            v_l14 := v_lum & "0000";
            case to_integer(s_smooth) is
                when 0      => v_b14 := v_l14;                 v_s14 := w2_acc;
                when 1      => v_b14 := shift_right(v_l14, 1); v_s14 := shift_right(w2_acc, 1);
                when 2      => v_b14 := shift_right(v_l14, 2); v_s14 := shift_right(w2_acc, 2);
                when 3      => v_b14 := shift_right(v_l14, 3); v_s14 := shift_right(w2_acc, 3);
                when 4      => v_b14 := shift_right(v_l14, 4); v_s14 := shift_right(w2_acc, 4);
                when 5      => v_b14 := shift_right(v_l14, 5); v_s14 := shift_right(w2_acc, 5);
                when 6      => v_b14 := shift_right(v_l14, 6); v_s14 := shift_right(w2_acc, 6);
                when others => v_b14 := shift_right(v_l14, 7); v_s14 := shift_right(w2_acc, 7);
            end case;
            v_n14 := not v_s14;
            v_x14 := w2_acc xor v_b14 xor v_n14;
            v_c14 := (w2_acc and v_b14) or (w2_acc and v_n14) or (v_b14 and v_n14);
            v_sum := resize(v_x14, 15) + (v_c14 & '1');
            -- Banded shading draws nothing over black.  An outline is a few
            -- pixels soft, and a line that followed that ramp (or the
            -- filter's own ramp out of black) would climb it as a spike, so
            -- the edge is trimmed: the first three samples after black are
            -- not drawn and reload the filter, and (at W3) the two samples
            -- before black are not drawn either.
            if v_lum(9 downto 6) = "0000" and s_shade = '0' then
                v_blk := '1';
            else
                v_blk := '0';
            end if;
            if w_av(1) = '1' then
                if w_av(2) = '0' or (v_blk = '0' and w2_since /= 0) then
                    w2_acc <= v_l14;
                else
                    w2_acc <= v_sum(13 downto 0);
                end if;
                if v_blk = '1' then
                    w2_since <= "11";
                elsif w2_since /= 0 then
                    w2_since <= w2_since - 1;
                end if;
            end if;
            if v_blk = '1' or w2_since /= 0 then
                w2_blk <= '1';
            else
                w2_blk <= '0';
            end if;
            w_av(2)   <= w_av(1);

            -- W3: sag term x (window-normalised) relief scale, own stage;
            -- the field's top (rolling) line has its brightness faded
            if s_peakdark = '1' then
                v_d6 := w2_acc(13 downto 8);
            else
                v_d6 := 63 - w2_acc(13 downto 8);
            end if;
            w3_prod <= v_d6 * s_scale_w;
            -- (the sample now at W2, and the one at W0 by its raw level)
            if w2_blk = '1' or (v_blk = '1' and w_av(1) = '1') or
               (s_shade = '0' and w_av(0) = '1' and w0_y(9 downto 5) = "00000") then
                v_dead := '1';
            else
                v_dead := '0';
            end if;
            if v_dead = '1' then
                w3_brt <= (others => '0');
            elsif s_rowtop = '1' then
                v_bm := w2_acc(13 downto 10) * s_fade4;
                w3_brt <= v_bm(7 downto 4);
            else
                w3_brt <= w2_acc(13 downto 10);
            end if;
            if v_dead = '1' then
                w3_brtv <= (others => '0');
            else
                w3_brtv <= w2_acc(13 downto 10);
            end if;
            w_av(3)   <= w_av(2);
            -- pixel count at this stage: stops at the line width, which is
            -- where the end-of-row word goes
            if s_hpulse = '1' then
                w_cnt <= (others => '0');
            elsif w_av(2) = '1' then
                w_cnt <= w_cnt + 1;
            end if;

            -- W4: displacement clamp (min 1: a line never lands on its own
            -- write row, and 0 is the end-of-row / empty word) + pack.  Write
            -- on even columns of sampled rows, then one zero word after the
            -- last column.  The vertical lines take the same displacement
            -- held at their own reach.
            v_dr := w3_prod(14 downto 5);
            if s_zoom = '1' then
                -- Zoom: the row's scale already carries its distance from the
                -- centre row; the line moves away from the centre about the
                -- baseline S (up above the centre, down below it)
                v_m8 := w3_prod(13 downto 6);
                if s_rneg = '1' then
                    v_disp := s_zs - v_m8;
                else
                    v_disp := s_zs + v_m8;
                end if;
            elsif v_dr > resize(s_dmax, 10) then
                v_disp := s_dmax;
            elsif v_dr = 0 then
                v_disp := to_unsigned(1, 8);
            else
                v_disp := v_dr(7 downto 0);
            end if;
            if v_dr > resize(s_dmaxv, 10) then
                w4_dv <= s_dmaxv;
            elsif v_dr = 0 then
                w4_dv <= to_unsigned(1, 8);
            else
                w4_dv <= v_dr(7 downto 0);
            end if;
            if w3_brtv = 0 and s_shade = '0' then
                w4_vblk <= '1';
            else
                w4_vblk <= '0';
            end if;
            if w_av(3) = '1' then
                w4_data <= std_logic_vector(v_disp) & std_logic_vector(w3_brt);
            else
                w4_data <= (others => '0');
            end if;
            w4_addr <= w_cnt(10 downto 1);
            w_avd   <= w_av(3);
            if s_sampled = '1' and ((w_av(3) = '1' and w_cnt(0) = '0') or
                                    (w_av(3) = '0' and w_avd = '1')) then
                w4_we <= '1';
            else
                w4_we <= '0';
            end if;
        end if;
    end process p_write;

    ----------------------------------------------------------------------
    -- 8 rotating profile banks: canonical 1W1R, 1024 x 12 each.  Bank k is
    -- read at the display column minus its own sideways offset; an address
    -- off the left end (or wrapped past 1023) reads as the empty word.
    -- In Weave the output registers of slots 4..7 hold the vertical-line
    -- candidates instead: this row's displacement + brightness, loaded at
    -- slot 7 and shifted down on every sampled column.
    ----------------------------------------------------------------------
    gen_banks : for k in 0 to NB - 1 generate
        type t_ram is array (0 to 1023) of std_logic_vector(11 downto 0);
        signal ram    : t_ram := (others => (others => '0'));
        signal rdaddr : unsigned(9 downto 0) := (others => '0');
        signal rdata  : std_logic_vector(11 downto 0) := (others => '0');
        signal neg1   : std_logic := '0';
        signal neg2   : std_logic := '0';
        constant KN   : integer := (k + 1) mod NB;
    begin
        p_bank : process(clk)
            variable v_r : signed(10 downto 0);
        begin
            if rising_edge(clk) then
                v_r := s_pxc - resize(s_off(k)(13 downto 4), 11);
                rdaddr <= unsigned(v_r(9 downto 0));
                neg1   <= v_r(10);
                if w4_we = '1' and to_integer(s_head) = k then
                    ram(to_integer(w4_addr)) <= w4_data;
                end if;
                rdata <= ram(to_integer(rdaddr));
                neg2  <= neg1;
                -- second (fabric) register after the EBR-absorbed one
                if k >= NVS and s_weave = '1' then
                    if w4_vs = '1' then
                        if k = NB - 1 then
                            a2_disp(k) <= w4_dv;
                        else
                            a2_disp(k) <= a2_disp(KN);
                        end if;
                    end if;
                else
                    if neg2 = '1' then
                        a2_disp(k) <= (others => '0');
                    else
                        a2_disp(k) <= unsigned(rdata(11 downto 4));
                    end if;
                end if;
                if neg2 = '0' and rdata(11 downto 4) /= "00000000" then
                    a2_nz(k) <= '1';
                else
                    a2_nz(k) <= '0';
                end if;
                if rdata(3 downto 0) = "0000" then
                    a2_blk(k) <= '1';
                else
                    a2_blk(k) <= '0';
                end if;
            end if;
        end process p_bank;
        r_bbrt(k) <= unsigned(rdata(3 downto 0));
    end generate gen_banks;

    ----------------------------------------------------------------------
    -- Vertical lines: per-column RAM (previous row's displacement), sample
    -- flags, candidate valid bits, node-grid phase
    ----------------------------------------------------------------------
    p_vram : process(clk)
    begin
        if rising_edge(clk) then
            if w4_vs = '1' then
                r_vram(to_integer(cw_addr)) <= "0000000" & w4_vblk & std_logic_vector(w4_dv);
            end if;
            r_crd <= r_vram(to_integer(s_colv));
            r_cq  <= r_crd;
        end if;
    end process p_vram;

    p_vcap : process(clk)
    begin
        if rising_edge(clk) then
            w1_vs <= w0_vs;
            w2_vs <= w1_vs;
            w3_vs <= w2_vs;
            w4_vs <= w3_vs;      -- data push (bank-output register level)
            w5_vs <= w4_vs;
            w6_vs <= w5_vs;      -- age push (two stages on, at A3's input)

            -- column index: the RAM read address has pointed at this column
            -- since the previous sample, so its word is in r_cq at the push;
            -- the write lands one clock after the address moves on
            cw_addr <= s_colv;
            if s_hpulse = '1' then
                s_colv <= (others => '0');
            elsif w3_vs = '1' then
                s_colv <= s_colv + 1;
            end if;

            -- candidate brightness: pushed one stage ahead of the data (it
            -- enters the delay at the bank-EBR-output level)
            if w3_vs = '1' then
                s_vb(NB - 1) <= w3_brtv;
                for k in NVS to NB - 2 loop
                    s_vb(k) <= s_vb(k + 1);
                end loop;
            end if;

            -- candidate valid bits ride with the data; dropped between rows
            if s_dly(2).avid = '0' then
                c_val <= (others => '0');
            elsif w4_vs = '1' then
                c_val <= '1' & c_val(NB - 1 downto NVS + 1);
            end if;

            -- node-grid phase for the rungs, at A3's input alignment
            if s_dly(3).avid = '0' then
                s_xr <= s_xrinit;
            elsif s_rdown = '1' then
                if s_twod(3) = '1' then s_xr <= s_xr - 2; else s_xr <= s_xr - 1; end if;
            else
                if s_twod(3) = '1' then s_xr <= s_xr + 2; else s_xr <= s_xr + 1; end if;
            end if;
        end if;
    end process p_vcap;

    ----------------------------------------------------------------------
    -- Zoom sequencer.  A point with zoom amount e moves from (c, r) to
    --   x = xc + (c - xc)(1 + e),   y = r + S + e (r - yc)
    -- so its line's stored displacement is d = S + e (r - yc) (the write path
    -- scales the relief by the row's distance from the centre and adds or
    -- subtracts it about the baseline S), and a hit still needs d = age.  On
    -- a display row at Y' = y - S - yc, a point of bank k that lands there
    -- came from column
    --   c = x - (x - xc) q,   q = (age_k - S) / Y'
    -- which is one slope per bank per row: the offset starts at -xc q at the
    -- left edge and grows by q per column.  q is held to 0 .. 1/4 (outside
    -- that range no point of the line can be on the row, and any valid
    -- column gives the right skirt answer).  The offset registers keep whole
    -- columns; each bank's slope and offset fraction sit in a small RAM that
    -- is read-modify-written round-robin, one bank per pixel: every 8 pixels
    -- (4 columns) a bank adds 4 q to its fraction and the carry moves its
    -- offset one column.
    --
    -- The sequencer runs once per line in horizontal blanking, for the NEXT
    -- row (~100 clocks), on a datapath kept deliberately small:
    --   * yc is row 512, so the row's relief scale Z x |row - yc| / 512 is the
    --     top bits of a Z x row accumulator (+Z per virtual row), less Z;
    --   * Y' and each bank's (age - S) are kept as sign + magnitude and
    --     stepped by the row advance, so no absolute values are taken (the
    --     per-bank ones live in the RAM; a freshly sampled bank restarts at
    --     -S);
    --   * an 8-step restoring division gives q MSB first, and the same bits
    --     build -(xc x q) by Horner.  A quotient bit in the first two steps
    --     means q >= 1/4: the bits are forced to 0100 0000.
    ----------------------------------------------------------------------
    p_zseq : process(clk)
        variable v_fs  : unsigned(8 downto 0);
        variable v_sn  : std_logic;
        variable v_sm  : unsigned(9 downto 0);
        variable v_t   : signed(11 downto 0);
        variable v_on  : std_logic;
        variable v_om  : unsigned(9 downto 0);
        variable v_r2  : unsigned(10 downto 0);
        variable v_df  : signed(11 downto 0);
        variable v_nb  : std_logic;
        variable v_bit : std_logic;
        variable v_p2  : signed(16 downto 0);
        variable v_sc  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- slope / fraction / age RAM (canonical 1W1R, fabric re-register)
            if zw_we = '1' then
                r_zram(to_integer(zw_a)) <= zw_d;
            end if;
            zr_o <= r_zram(to_integer(zr_a));
            zr_q <= zr_o;

            -- round-robin slope carries while the read position advances
            zr_a <= '0' & px_rd(2 downto 0);
            zr_b <= zr_a;
            zr_c <= zr_b;
            zr_v <= zr_v(1 downto 0) & s_inc;
            v_fs := resize(unsigned(zr_q(7 downto 0)), 9) + (unsigned(zr_q(14 downto 8)) & "00");
            zw_we <= zr_v(2);
            zw_a  <= zr_c;
            zw_d  <= zr_q(15 downto 8) & std_logic_vector(v_fs(7 downto 0));
            z_tk  <= zr_c(2 downto 0);
            z_op  <= to_signed(16, 14);
            z_do  <= zr_v(2) and v_fs(8);
            z_rs  <= '0';

            -- shared sign + magnitude stepper: value + ka (ka = 1..5); its
            -- input is registered one state before the result is used
            v_sn := z_sn;
            v_sm := z_sm;
            if v_sn = '1' then
                v_t := signed(resize(v_sm, 12)) - signed(resize(s_ka, 12));
            else
                v_t := signed(resize(v_sm, 12)) + signed(resize(s_ka, 12));
            end if;
            if v_sn = '1' and v_t(11) = '1' then          -- crossed zero: ka - mag (1..4)
                v_on := '0';
                v_om := resize(unsigned(not v_t(2 downto 0)) + 1, 10);
            elsif v_sn = '1' and v_t = 0 then
                v_on := '0';
                v_om := (others => '0');
            else
                v_on := v_sn;
                v_om := unsigned(v_t(9 downto 0));
            end if;

            if s_zoom = '0' then
                s_scale_w <= s_scale_eff;
                s_rneg    <= '0';
                z_st      <= (others => '0');
            else
                case to_integer(z_st) is
                    when 0 =>                      -- idle
                        null;
                    when 1 =>                      -- field start: row 0
                        if z_first = '1' then
                            z_racc <= (others => '0');
                        end if;
                        z_cnt <= resize(s_ka, 4);
                        z_rdk <= (others => '0');
                        z_st  <= to_unsigned(2, 4);
                    when 2 =>                      -- + Z per virtual row
                        z_racc <= z_racc + resize(s_zz, 18);
                        z_cnt  <= z_cnt - 1;
                        if z_cnt = 1 then
                            z_st <= to_unsigned(3, 4);
                        end if;
                    when 3 =>                      -- row scale; Y' steps
                        -- Z x (row - 512) / 512, as magnitude + "above centre"
                        v_sc := signed(resize(z_racc(17 downto 9), 10)) - signed(resize(s_zz, 10));
                        if v_sc(9) = '1' then
                            s_scale_w <= resize(unsigned(not v_sc(7 downto 0)), 9);
                        else
                            s_scale_w <= resize(unsigned(v_sc(7 downto 0)), 9);
                        end if;
                        s_rneg <= v_sc(9);
                        if z_first = '1' then
                            z_sn <= '1';                  -- -(512 + S)
                            z_sm <= "10" & s_zs;
                        else
                            z_sn <= z_yneg;
                            z_sm <= z_ymag;
                        end if;
                        z_st   <= to_unsigned(4, 4);
                    when 4 =>                      -- Y' steps to the next row
                        z_yneg <= v_on;
                        z_ymag <= v_om;
                        z_k    <= (others => '0');
                        z_st   <= to_unsigned(5, 4);
                    when 5 =>                      -- bank k: age - S on this row
                        if s_sampled = '1' and z_k = s_head then
                            z_sn <= '1';                  -- sampled on this row: -S
                            z_sm <= resize(s_zs, 10);
                        else
                            z_sn <= zr_q(10);
                            z_sm <= unsigned(zr_q(9 downto 0));
                        end if;
                        z_st <= to_unsigned(6, 4);
                    when 6 =>                      -- ... stepped to the next row
                        zw_we   <= '1';
                        zw_a    <= '1' & z_k;
                        zw_d    <= "00000" & v_on & std_logic_vector(v_om);
                        z_rem   <= v_om;
                        z_zero  <= v_on xor z_yneg;
                        z_ovf   <= '0';
                        z_clamp <= '0';
                        z_q     <= (others => '0');
                        z_p     <= (others => '0');
                        z_cnt   <= to_unsigned(8, 4);
                        z_rdk   <= z_k + 1;
                        z_st    <= to_unsigned(7, 4);
                    when 7 =>                      -- q = 256 |num| / |Y'|, -(xc q)
                        v_r2 := z_rem & '0';
                        v_df := signed(resize(v_r2, 12)) - signed(resize(z_ymag, 12));
                        v_nb := not v_df(11);
                        if v_nb = '1' then
                            z_rem <= unsigned(v_df(9 downto 0));
                        else
                            z_rem <= v_r2(9 downto 0);
                        end if;
                        v_bit := v_nb;
                        if z_cnt = 8 then
                            z_ovf <= v_nb;
                            v_bit := '0';
                        elsif z_cnt = 7 then
                            v_bit   := v_nb or z_ovf;
                            z_clamp <= v_nb or z_ovf;
                        elsif z_clamp = '1' then
                            v_bit := '0';
                        end if;
                        if z_zero = '1' then
                            v_bit := '0';
                        end if;
                        z_q  <= z_q(5 downto 0) & v_bit;
                        v_p2 := z_p(15 downto 0) & '0';
                        if v_bit = '1' then
                            v_p2 := v_p2 - signed(resize(s_xcc, 17));
                        end if;
                        z_p   <= v_p2;
                        z_cnt <= z_cnt - 1;
                        if z_cnt = 1 then
                            z_st <= to_unsigned(8, 4);
                        end if;
                    when 8 =>                      -- slope + fraction; offset := 0
                        zw_we <= '1';
                        zw_a  <= '0' & z_k;
                        zw_d  <= '0' & std_logic_vector(z_q) & std_logic_vector(z_p(7 downto 0));
                        z_tk  <= z_k;
                        z_rs  <= '1';
                        z_do  <= '0';
                        z_st  <= to_unsigned(9, 4);
                    when 9 =>                      -- offset := whole columns of -xc q
                        zw_we <= '0';
                        z_tk  <= z_k;
                        z_op  <= resize(z_p(16 downto 8), 10) & "0000";
                        z_do  <= '1';
                        z_st  <= to_unsigned(10, 4);
                    when 10 =>                     -- next bank
                        if z_k = 7 then
                            z_st <= (others => '0');
                        else
                            z_k  <= z_k + 1;
                            z_st <= to_unsigned(5, 4);
                        end if;
                    when others =>
                        z_st <= (others => '0');
                end case;
                -- the sequencer owns the read address while it runs
                if z_st /= 0 then
                    zr_a <= '1' & z_rdk;
                end if;
                -- start: the end of an active line (at the write path's last
                -- stage), or the hsync of a line without video (field start)
                if (w_av(3) = '0' and w_avd = '1') or s_hblank = '1' then
                    z_st    <= to_unsigned(1, 4);
                    z_first <= s_hblank;
                end if;
            end if;
        end if;
    end process p_zseq;

    ----------------------------------------------------------------------
    -- Display pipeline: connected-stroke segment -> candidate tests ->
    -- recency-priority resolve -> palette + shading -> compose -> [1 2 1]
    -- -> blanking
    ----------------------------------------------------------------------
    p_disp : process(clk)
        variable v_ok   : boolean;
        variable v_first : boolean;
        variable v_wv   : boolean;
        variable v_prev : unsigned(7 downto 0);
        variable v_m9   : unsigned(8 downto 0);
        variable v_ab   : signed(8 downto 0);
        variable v_ba   : signed(8 downto 0);
        variable v_at   : signed(9 downto 0);
        variable v_d1, v_d2 : boolean;
        variable v_hit  : std_logic;
        variable v_cov  : std_logic;
        variable v_core : std_logic;
        variable v_live : std_logic;
        variable v_sht  : std_logic_vector(3 downto 0);
        variable v_u, v_um, v_un : std_logic_vector(NB - 1 downto 0);
        variable v_wm, v_wn      : std_logic_vector(NB - 1 downto 0);
        variable v_above_m, v_above_n : std_logic;
        variable v_anyml, v_anymu, v_anynl, v_anynu : std_logic;
        variable v_whl, v_whu, v_anyl, v_anyu : std_logic;
        variable v_visl, v_visu, v_vis, v_rg, v_sb : std_logic;
        variable v_sel  : std_logic_vector(NB - 1 downto 0);
        variable v_pbrt : unsigned(3 downto 0);
        variable v_psh  : std_logic_vector(3 downto 0);
        variable v_c8   : signed(7 downto 0);
        variable v_bf   : unsigned(3 downto 0);
        variable v_bf5  : unsigned(4 downto 0);
        variable v_yi   : unsigned(10 downto 0);
        variable v_ui   : integer range -2048 to 2047;
        variable v_vi   : integer range -2048 to 2047;
        variable v_fs   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- R1..R3: the half-column phase travels beside the bank reads
            r_ph(1) <= px_rd(0);
            r_ph(3 downto 2) <= r_ph(2 downto 1);
            r_pha <= (not r_ph(2)) or s_weave;
            r_phb <= r_ph(2) or s_weave;

            -- B2: connected stroke.  p_d1 holds the previous COLUMN's height
            -- (loaded on the second pixel of each column); phase 0 draws
            -- prev -> mid, phase 1 draws mid -> this column.  Per bank: a
            -- zero word is "no data" (left of the stored row, or its end
            -- marker); the first valid column restarts the stroke (flat),
            -- and once the end marker has been read the bank is dead until
            -- the next scanline.
            -- Weave, slots 4..7: p_d1 holds the candidate's previous-ROW
            -- displacement instead (shifted with the slot) and the stroke
            -- runs prev -> this on every pixel.
            -- Banded shading draws nothing where the source is black: a
            -- brightness-0 word is not a candidate at all (no line, no
            -- skirt), and a stroke never connects to one (it restarts flat).
            for k in 0 to NB - 1 loop
                v_wv := k >= NVS and s_weave = '1';
                if v_wv then
                    v_ok    := c_val(maximum(k, NVS)) = '1' and (s_shade = '1' or s_vb(k) /= 0);
                    v_first := false;
                    if w4_vs = '1' then
                        if k = NB - 1 then
                            if s_frow = '1' or r_cq(8) = '1' then
                                p_d1(k) <= w4_dv;
                            else
                                p_d1(k) <= unsigned(r_cq(7 downto 0));
                            end if;
                        else
                            p_d1(k) <= p_d1((k + 1) mod NB);
                        end if;
                    end if;
                else
                    v_ok    := a2_nz(k) = '1' and s_past(k) = '0' and s_dly(2).avid = '1' and
                               (s_shade = '1' or a2_blk(k) = '0');
                    v_first := v_ok and (s_seen(k) = '0' or s_pok(k) = '0');
                    if r_ph(3) = '1' or v_first then
                        p_d1(k) <= a2_disp(k);
                        if v_ok then s_pok(k) <= '1'; else s_pok(k) <= '0'; end if;
                    end if;
                end if;
                if s_dly(2).avid = '0' then
                    s_seen(k) <= '0';
                    s_past(k) <= '0';
                elsif a2_nz(k) = '1' then
                    s_seen(k) <= '1';
                elsif s_seen(k) = '1' then
                    s_past(k) <= '1';
                end if;
                if v_first then
                    v_prev := a2_disp(k);
                else
                    v_prev := p_d1(k);
                end if;
                v_m9 := resize(v_prev, 9) + resize(a2_disp(k), 9);
                if (k < NVS and r_ph(3) = '0') or (k >= NVS and r_pha = '1') then
                    b2_ea(k) <= v_prev;
                else
                    b2_ea(k) <= v_m9(8 downto 1);
                end if;
                if (k < NVS and r_ph(3) = '1') or (k >= NVS and r_phb = '1') then
                    b2_eb(k) <= a2_disp(k);
                else
                    b2_eb(k) <= v_m9(8 downto 1);
                end if;
                if v_ok then b2_v(k) <= '1'; else b2_v(k) <= '0'; end if;
            end loop;

            -- B3: segment extent (lo = near end, hi = far end, span = hi - lo)
            for k in 0 to NB - 1 loop
                v_ab := signed(resize(b2_ea(k), 9)) - signed(resize(b2_eb(k), 9));
                v_ba := signed(resize(b2_eb(k), 9)) - signed(resize(b2_ea(k), 9));
                if v_ab(8) = '1' then
                    b3_lo(k)   <= b2_ea(k);
                    b3_hi(k)   <= b2_eb(k);
                    b3_span(k) <= unsigned(v_ba(7 downto 0));
                else
                    b3_lo(k)   <= b2_eb(k);
                    b3_hi(k)   <= b2_ea(k);
                    b3_span(k) <= unsigned(v_ab(7 downto 0));
                end if;
            end loop;
            b3_v <= b2_v;

            -- A3: candidate offsets against the segment's near end and its far
            -- end + stroke thickness; beam-speed dimming from the segment's
            -- extent (1/2, then 1/4); node-grid phase of the pixel traced
            -- back along the slant
            for k in 0 to NB - 1 loop
                if k < NVS then
                    v_at := signed(resize(s_age(k), 10)) - signed(resize(s_thkw, 10));
                else
                    v_at := signed(resize(s_age(k), 10)) - signed(resize(s_tb, 10));
                end if;
                a3_slo(k) <= signed(resize(s_age(k), 10)) - signed(resize(b3_lo(k), 10));
                a3_shi(k) <= v_at - signed(resize(b3_hi(k), 10));
                case s_dn is
                    when "01"   => v_d1 := b3_span(k)(7 downto 1) /= "0000000"; v_d2 := b3_span(k)(7 downto 3) /= "00000";
                    when "10"   => v_d1 := b3_span(k)(7 downto 2) /= "000000";  v_d2 := b3_span(k)(7 downto 4) /= "0000";
                    when others => v_d1 := b3_span(k)(7 downto 3) /= "00000";   v_d2 := b3_span(k)(7 downto 5) /= "000";
                end case;
                if b3_v(k) = '0' then
                    a3_dim(k) <= "11";
                elsif v_d2 then
                    a3_dim(k) <= "10";
                elsif v_d1 then
                    a3_dim(k) <= "01";
                else
                    a3_dim(k) <= "00";
                end if;
                a3_g(k)   <= s_xr + b3_lo(k)(5 downto 0);
            end loop;

            -- A4: classify each candidate against the stroke window
            for k in 0 to NB - 1 loop
                if k < NVS then
                    f_class(a3_slo(k), a3_shi(k), s_live(k), a3_dim(k),
                            s_w8, s_glow, s_gsh, v_hit, v_cov, v_core, v_sht);
                else
                    v_live := s_live(k) or s_weave;
                    f_class(a3_slo(k), a3_shi(k), v_live, a3_dim(k),
                            s_w8, s_glow, s_gsh, v_hit, v_cov, v_core, v_sht);
                end if;
                a4_hit(k) <= v_hit;
                a4_cov(k) <= v_cov;
                a4_sh(k)  <= v_sht;
                -- on the node grid, and not inside this candidate's own stroke.
                -- Banded shading (nothing drawn over black, so no next line
                -- need exist below): a rung also ends two grid pitches below
                -- its line.
                if (a3_g(k)(5 downto 4) and s_gmask) = "00" and a3_g(k)(3 downto 0) < s_thkx and
                   (s_shade = '1' or a3_slo(k)(9) = '1' or
                    (a3_slo(k)(9 downto 7) = "000" and (s_gmask(1) = '1' or a3_slo(k)(6) = '0')
                                                   and (s_gmask(0) = '1' or a3_slo(k)(5) = '0'))) then
                    a4_rung(k) <= not v_core;
                else
                    a4_rung(k) <= '0';
                end if;
            end loop;

            -- A5: one-hot "first candidate that hits or covers, in recency
            -- order".  Banks rotate, so recency order is: slots at or below
            -- the head, downward, then the slots above it, downward.  The
            -- highest set bit is found in each half of each run, then the
            -- four are ranked: (upper, lower) of the newer run, then of the
            -- older run.  In Weave slots 0..3 rotate by themselves (the scan
            -- lines) and slots 4..7 are the vertical lines, 7 newest, ranked
            -- on their own: the vector then holds one winner per family.
            for k in 0 to NB - 1 loop
                v_u(k) := a4_hit(k) or a4_cov(k);
            end loop;
            v_um := v_u and s_hmask;
            v_un := v_u and not s_hmask;
            for b in 0 to 1 loop
                v_above_m := '0';
                v_above_n := '0';
                for i in 3 downto 0 loop
                    v_wm(b * 4 + i) := v_um(b * 4 + i) and not v_above_m;
                    v_wn(b * 4 + i) := v_un(b * 4 + i) and not v_above_n;
                    v_above_m := v_above_m or v_um(b * 4 + i);
                    v_above_n := v_above_n or v_un(b * 4 + i);
                end loop;
                if b = 0 then
                    v_anyml := v_above_m; v_anynl := v_above_n;
                else
                    v_anymu := v_above_m; v_anynu := v_above_n;
                end if;
            end loop;
            for k in 0 to 3 loop
                a5_win(k) <= (v_wm(k) and not v_anymu) or
                             (v_wn(k) and not v_anymu and not v_anyml and (s_weave or not v_anynu));
            end loop;
            for k in 4 to 7 loop
                a5_win(k) <= v_wm(k) or
                             (v_wn(k) and not v_anymu and (s_weave or not v_anyml));
            end loop;
            a5_hit  <= a4_hit;
            a5_rung <= a4_rung;
            a5_sh   <= a4_sh;

            -- A6 (payload): a family's line is visible when its winner hits
            -- (Hidden Line off: when anything hits; the payloads are then
            -- OR-ed, which keeps the brighter of two crossing lines).  A
            -- rung on the winning (nearest) surface draws with that line's
            -- brightness, through its glow but not its stroke.  Scan lines go
            -- over vertical ones.
            v_whl := '0'; v_whu := '0'; v_anyl := '0'; v_anyu := '0'; v_rg := '0';
            for k in 0 to 3 loop
                v_whl  := v_whl or (a5_win(k) and a5_hit(k));
                v_anyl := v_anyl or a5_hit(k);
            end loop;
            for k in 4 to 7 loop
                v_whu  := v_whu or (a5_win(k) and a5_hit(k));
                v_anyu := v_anyu or a5_hit(k);
            end loop;
            for k in 0 to NB - 1 loop
                v_rg := v_rg or (a5_win(k) and a5_rung(k));
            end loop;
            if s_hlr = '1' then
                v_visl := v_whl; v_visu := v_whu;
            else
                v_visl := v_anyl; v_visu := v_anyu;
            end if;
            v_vis := v_visl or v_visu;
            v_rg  := v_rg and s_mesh;
            for k in 0 to NB - 1 loop
                if s_hlr = '1' then v_sb := a5_win(k); else v_sb := a5_hit(k); end if;
                if s_weave = '1' then
                    if k < NVS then
                        v_sel(k) := v_sb and v_visl;
                    else
                        v_sel(k) := v_sb and not v_visl;
                    end if;
                elsif v_vis = '1' then
                    v_sel(k) := v_sb;
                else
                    v_sel(k) := a5_win(k);
                end if;
            end loop;
            v_pbrt := (others => '0');
            v_psh  := (others => '0');
            for k in 0 to NB - 1 loop
                if v_sel(k) = '1' then
                    v_pbrt := v_pbrt or a5_brt(k);
                    v_psh  := v_psh or a5_sh(k);
                end if;
            end loop;
            a6_line <= v_vis or v_rg;
            a6_brt  <= v_pbrt;
            if v_rg = '1' then
                a6_sh <= "1111";
            else
                a6_sh <= v_psh;
            end if;
            a6_isv  <= v_rg or (s_weave and not v_visl);

            -- A7: shading factor
            if s_shade = '1' then
                v_bf := a6_brt;
            elsif a6_brt >= 8 then
                v_bf := to_unsigned(15, 4);
            else
                v_bf := to_unsigned(4, 4);
            end if;
            if a6_sh(0) = '1' then
                null;
            elsif a6_sh(1) = '1' then
                v_bf := shift_right(v_bf, 1);
            elsif a6_sh(2) = '1' then
                v_bf := shift_right(v_bf, 2);
            elsif a6_sh(3) = '1' then
                v_bf := shift_right(v_bf, 3);
            else
                v_bf := (others => '0');
            end if;
            -- +1 (full scale at 16) plus the thin-stroke lift, held at the
            -- 768 drawn-luma cap (bf1 = 16)
            v_bf5 := resize(v_bf, 5) + 1 + resize(s_bboost, 5);
            if v_bf5 > 16 then
                v_bf5 := to_unsigned(16, 5);
            end if;
            -- rungs and vertical lines: thin-stroke gain against the output
            -- filter (16 -> 24 / 20)
            a7_bf1 <= resize(v_bf5, 6);
            if a6_isv = '1' then
                case s_tgain is
                    when "10"   => a7_bf1 <= resize(v_bf5, 6) + resize(shift_right(v_bf5, 1), 6);
                    when "01"   => a7_bf1 <= resize(v_bf5, 6) + resize(shift_right(v_bf5, 2), 6);
                    when others => null;
                end case;
            end if;
            a7_alt  <= a6_brt(3 downto 1) xor (2 downto 0 => not s_peakdark);
            a7_line <= a6_line;
            -- A7 (Video lane): source chroma at this display pixel, x2
            -- saturation, clamped to +/-448
            v_c8 := signed(r_cdq(15 downto 8) xor x"80");
            if v_c8 > 56 then
                a7_cu <= to_signed(448, 10);
            elsif v_c8 < -56 then
                a7_cu <= to_signed(-448, 10);
            else
                a7_cu <= shift_left(resize(v_c8, 10), 3);
            end if;
            v_c8 := signed(r_cdq(7 downto 0) xor x"80");
            if v_c8 > 56 then
                a7_cv <= to_signed(448, 10);
            elsif v_c8 < -56 then
                a7_cv <= to_signed(-448, 10);
            else
                a7_cv <= shift_left(resize(v_c8, 10), 3);
            end if;

            -- A8: palette
            if s_zone = 7 then
                a8_yo <= C_TY(to_integer(a7_alt));
                a8_uo <= C_TU(to_integer(a7_alt));
                a8_vo <= C_TV(to_integer(a7_alt));
            elsif s_zone = 5 then
                a8_yo <= C_ZY(5);
                a8_uo <= a7_cu;
                a8_vo <= a7_cv;
            else
                a8_yo <= C_ZY(to_integer(s_zone));
                a8_uo <= C_ZU(to_integer(s_zone));
                a8_vo <= C_ZV(to_integer(s_zone));
            end if;
            a8_bf1  <= a7_bf1;
            a8_line <= a7_line;

            -- A9: shading multiplies (each gets the stage to itself)
            a9_ym   <= a8_yo * a8_bf1;
            a9_um   <= a8_uo * signed(resize(a8_bf1, 7));
            a9_vm   <= a8_vo * signed(resize(a8_bf1, 7));
            a9_line <= a8_line;

            -- A10: compose (line over black).  At unity gain line luma tops
            -- out at 64 + 704 = 768 by construction; a thin stroke's gain can
            -- take it to 64 + 1056, held at 1023 here (the output filter
            -- brings a 1-2 pixel stroke back under the cap, and A11 holds
            -- anything wider at 768).  Chroma is held at 512 +/- 448.
            if a9_line = '1' then
                v_yi := to_unsigned(64, 11) + resize(a9_ym(15 downto 4), 11);
                if v_yi > 1023 then
                    a10_y <= to_unsigned(1023, 10);
                else
                    a10_y <= v_yi(9 downto 0);
                end if;
                v_ui := to_integer(shift_right(a9_um, 4));
                v_vi := to_integer(shift_right(a9_vm, 4));
                if v_ui > 448 then v_ui := 448; elsif v_ui < -448 then v_ui := -448; end if;
                if v_vi > 448 then v_vi := 448; elsif v_vi < -448 then v_vi := -448; end if;
                a10_u <= to_unsigned(512 + v_ui, 10);
                a10_v <= to_unsigned(512 + v_vi, 10);
            else
                a10_y <= to_unsigned(64, 10);
                a10_u <= to_unsigned(512, 10);
                a10_v <= to_unsigned(512, 10);
            end if;

            -- F1..F3: horizontal [1 2 1]/4 on Y, U and V (output = centre)
            f1_y <= a10_y;  f2_y <= f1_y;
            f1_u <= a10_u;  f2_u <= f1_u;
            f1_v <= a10_v;  f2_v <= f1_v;
            v_fs := resize(a10_y, 12) + shift_left(resize(f1_y, 12), 1) + resize(f2_y, 12) + 2;
            f3_y <= v_fs(11 downto 2);
            v_fs := resize(a10_u, 12) + shift_left(resize(f1_u, 12), 1) + resize(f2_u, 12) + 2;
            f3_u <= v_fs(11 downto 2);
            v_fs := resize(a10_v, 12) + shift_left(resize(f1_v, 12), 1) + resize(f2_v, 12) + 2;
            f3_v <= v_fs(11 downto 2);

            -- A11: blanking gate (outputs are neutral whenever the aligned
            -- avid = 0); this is the output register
            if s_dly(C_LAT - 2).avid = '1' then
                if f3_y > 768 then
                    a11_y <= to_unsigned(768, 10);   -- drawn-luma cap
                else
                    a11_y <= f3_y;
                end if;
                a11_u <= f3_u;
                a11_v <= f3_v;
            else
                a11_y <= to_unsigned(64, 10);
                a11_u <= to_unsigned(512, 10);
                a11_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_disp;

    ----------------------------------------------------------------------
    -- Sync delay line + input-chroma delay
    ----------------------------------------------------------------------
    p_dly : process(clk)
    begin
        if rising_edge(clk) then
            s_dly(0).avid    <= data_in.avid;
            s_dly(0).hsync_n <= data_in.hsync_n;
            s_dly(0).vsync_n <= data_in.vsync_n;
            s_dly(0).field_n <= data_in.field_n;
            for i in 1 to C_LAT - 1 loop
                s_dly(i) <= s_dly(i - 1);
            end loop;
        end if;
    end process p_dly;

    p_cdl : process(clk)
        variable v_din : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            s_cwp <= s_cwp + 1;
            s_crp <= s_cwp - 6;
            r_cdl(to_integer(s_cwp)) <= data_in.u(9 downto 2) & data_in.v(9 downto 2);
            r_cdo <= r_cdl(to_integer(s_crp));
            r_cdq <= r_cdo;

            -- brightness delay: written one clock after the bank read
            -- register, read 4 clocks later, fabric register = A5
            for k in 0 to NB - 1 loop
                if k >= NVS and s_weave = '1' then
                    v_din(4 * k + 3 downto 4 * k) := std_logic_vector(s_vb(k));
                else
                    v_din(4 * k + 3 downto 4 * k) := std_logic_vector(r_bbrt(k));
                end if;
            end loop;
            s_brp <= s_cwp - 3;
            r_bdl(to_integer(s_cwp)) <= v_din;
            r_bdo <= r_bdl(to_integer(s_brp));
            for k in 0 to NB - 1 loop
                a5_brt(k) <= unsigned(r_bdo(4 * k + 3 downto 4 * k));
            end loop;
        end if;
    end process p_cdl;

    ----------------------------------------------------------------------
    -- Outputs
    ----------------------------------------------------------------------
    data_out.y <= std_logic_vector(a11_y);
    data_out.u <= std_logic_vector(a11_u);
    data_out.v <= std_logic_vector(a11_v);

    data_out.avid    <= s_dly(C_LAT - 1).avid;
    data_out.hsync_n <= s_dly(C_LAT - 1).hsync_n;
    data_out.vsync_n <= s_dly(C_LAT - 1).vsync_n;
    data_out.field_n <= s_dly(C_LAT - 1).field_n;

end ruttetra_xy;
