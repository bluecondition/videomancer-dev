-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- ruttetra_v.vhd - Vertical Rutt-Etra scan processor for Videomancer
--
-- The vertical-raster sibling of ruttetra: incoming video is re-rendered as a
-- field of VERTICAL lines whose horizontal position is displaced by the
-- picture's luma.  Bright material rises (to the left) out of a dark floor
-- that sags to the right, drawn as glowing lines on black, with optional
-- hidden-line terrain occlusion (each nearer line drops an opaque black
-- skirt to its right that hides the lines behind it).
--
-- Engine
--   Every Nth column is a "sampled" column.  On every row its luma (after
--   contrast + a smoothing IIR that runs DOWN the column, state held in a
--   256-entry per-column RAM) becomes a displacement (sag, applied RIGHTWARD
--   from the source column) and a 4-bit brightness.  A displaced line lands
--   on pixels to the RIGHT of its source column, i.e. later in the same
--   scanline, so no frame or line store of the picture is needed: the last 8
--   sampled columns ride an 8-deep shift register of candidates, each with a
--   running age (distance from its source column).  Per display pixel, 8
--   parallel candidate tests (age_k against the candidate's segment and the
--   stroke window) find the hits and the most recent one wins - which is
--   also the hidden-line rule.  Max displacement = 8 x spacing - 1 (capped
--   255), so denser line fields have proportionally shallower relief
--   (relief is normalised to that window, as in ruttetra).
--
--   Connected strokes: the per-column RAM also keeps the displacement the
--   column had on the previous row; each row draws the run from the previous
--   row's position to its own, so steep relief stays one continuous line.
--   The run is dimmed by its length (a fast beam deposits less light).  The
--   segment is prepared once per sampled column in the write path, so the
--   display side only carries {lo, hi, dim, bright, age} per candidate.
--
--   The display pipeline runs 7 clocks behind the capture pipeline (the
--   candidate for column c appears with age 0 exactly when the display is at
--   pixel c), so everything is one latency in every mode.
--
-- Controls (same panel as ruttetra)
--   Pot 1  (registers_in(0))    : Scan Lines - line density (CW = denser)
--   Pot 2  (registers_in(1))    : Thickness  - stroke width 1..8;
--                                 a graded glow halo fades in from T=3
--   Pot 3  (registers_in(2))    : Line Color - White/Green/Amber/Cyan/
--                                 Magenta/Video/Fire/Terrain (luma tint)
--   Pot 4  (registers_in(3))    : Contrast   - input gain into disp+shade
--   Pot 5  (registers_in(4))    : Roll       - bipolar line-grid drift
--                                 (centre still; CW right, CCW left)
--   Pot 6  (registers_in(5))    : Smoothing  - IIR down each line
--                                 (raw nervous edges -> flowing)
--   Tog 7  (registers_in(6)(0)) : Peaks      - Bright rises / Dark rises
--   Tog 8  (registers_in(6)(1)) : Hidden Line- terrain skirts on/off
--   Tog 9  (registers_in(6)(2)) : Shading    - Banded (2-level) / Luma
--   Tog 10 (registers_in(6)(3)) : Roll       - Run / Freeze
--   Tog 11 (registers_in(6)(4)) : Input      - Positive / Negative
--   Fader  (registers_in(7))    : RELIEF     - displacement depth (glided)
--
-- Latency: 19 clocks + 1 IO-align pad = 20, identical in every mode.
--
-- Resolution scaling: all horizontal geometry -- line spacing (8..71),
-- candidate ages, displacement (<= 255), stroke thickness and roll -- is
-- counted in VIRTUAL 1920-wide pixels (x step 1 at 1080, 1.5 at 720p, 2 at
-- SD, where the 4:3 picture is a 1440-wide window), so every mode shows
-- the same lines at the same size relative to screen height.  The stroke
-- window is at least ceil(x step) wide so a thin line still lands on a
-- pixel.  The smoothing shift drops by 1 (progressive SD, 1080i) or 2
-- (interlaced SD) so its reach down the screen stays comparable.
--
-- Output conditioning: thin vertical lines are near-full-swing pixel-rate
-- edges on every scanline (the HD HDMI thick-bar trigger), so Y, U and V go
-- through a horizontal [1 2 1]/4 before the blanking gate.  That filter
-- halves a 1-pixel line (3/4 for 2 pixels), so strokes 1 / 2 pixels wide are
-- drawn x1.5 / x1.25 ahead of it (v0.1.1); the filtered luma is held at the
-- 768 drawn-luma cap.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;


architecture ruttetra_v of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (19 + 1 = 20 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    signal s_io_align_in  : t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_video_stream_yuv444_30b;

    constant C_LAT  : integer := 19;     -- display pipeline registers
    constant NB     : integer := 8;      -- candidate lines per pixel
    constant C_CDLY : integer := 10;     -- input-chroma delay (Video colour)

    attribute keep : boolean;
    attribute no_rw_check : boolean;

    ----------------------------------------------------------------------
    -- Timing class (free-running from the timing ID)
    ----------------------------------------------------------------------
    signal s_sd   : std_logic := '0';                      -- x step 2
    signal s_720  : std_logic := '0';                      -- x step 1.5
    signal s_kc   : unsigned(3 downto 0) := to_unsigned(1, 4);  -- ceil(x step)
    signal s_sred : unsigned(1 downto 0) := "00";          -- smoothing shift cut

    ----------------------------------------------------------------------
    -- Latched controls (at vsync)
    ----------------------------------------------------------------------
    signal s_dspace : unsigned(6 downto 0) := to_unsigned(39, 7);  -- 8..71
    signal s_dm1    : signed(8 downto 0) := to_signed(38, 9);      -- D - 1
    signal s_dm2    : signed(8 downto 0) := to_signed(37, 9);      -- D - 2
    signal s_dmax   : unsigned(7 downto 0) := to_unsigned(255, 8);
    -- relief normaliser: min(spacing, 32) (32 = unity; window = 8 x spacing)
    signal s_rmul   : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal s_thick  : unsigned(3 downto 0) := to_unsigned(2, 4);   -- 1..8
    -- stroke-window thickness: max(T, ceil(x step))
    signal s_thkw   : unsigned(3 downto 0) := to_unsigned(2, 4);
    -- thin strokes integrate less light: small shading lift at T=1..3
    signal s_bboost : unsigned(1 downto 0) := "10";
    -- thin-stroke gain ahead of the [1 2 1] output filter, by the stroke's
    -- width in PIXELS: "10" = x1.5 (1 px), "01" = x1.25 (2 px), "00" = x1
    signal s_tgain  : unsigned(1 downto 0) := "00";
    signal s_zone   : unsigned(2 downto 0) := (others => '0');
    -- K3 label decode, free-running.  At program load the firmware writes
    -- the initial_value_label's INDEX (0..7), not a 0..1023 position, so a
    -- value below 8 is the label index itself.
    signal s_k3_idx : unsigned(2 downto 0) := (others => '0');
    signal s_gain8  : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_smooth : unsigned(2 downto 0) := "010";
    signal s_scale9 : unsigned(8 downto 0) := to_unsigned(8, 9);   -- glided
    signal s_scale_eff : unsigned(8 downto 0) := to_unsigned(8, 9);
    signal s_peakdark : std_logic := '0';
    signal s_hlr      : std_logic := '1';
    signal s_shade    : std_logic := '1';
    -- K2 glow: halo on from T=3; inner-half shift 2 (T3-4) / 1 (T5-8),
    -- outer half one step dimmer; s_half = ceil(T/2)
    signal s_glow     : std_logic := '0';
    signal s_gsh      : unsigned(2 downto 0) := "010";
    signal s_half     : unsigned(3 downto 0) := to_unsigned(1, 4);
    signal s_rfreeze  : std_logic := '0';
    signal s_neg      : std_logic := '0';

    -- K5 Roll: line-grid phase in units of one line spacing (wraps free)
    signal s_rdir   : std_logic := '0';
    signal s_ru     : unsigned(4 downto 0) := (others => '0');
    signal s_rspd   : unsigned(13 downto 0) := (others => '0');
    signal s_rphase : unsigned(15 downto 0) := (others => '0');
    signal s_roff   : unsigned(6 downto 0) := (others => '0');
    signal s_fade4  : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Position + column bookkeeping
    ----------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal s_row_armed  : std_logic := '1';
    signal s_rowclr     : std_logic := '0';   -- first active pixel (1 clk)
    signal s_saw_frame  : std_logic := '0';
    signal s_saw_av     : std_logic := '0';   -- this line carried video
    -- first active row of the field: the column filters and the previous-row
    -- displacement restart there.  Armed by any line without video (vblank),
    -- never by the vsync edge (decoders assert it early, analog serrates).
    signal s_frow_pend  : std_logic := '1';
    signal s_frow       : std_logic := '1';
    -- virtual x: 720p alternates advances of 1 and 2 (s_xf), SD always 2
    signal s_xf         : std_logic := '0';
    signal s_ka2        : std_logic := '0';   -- registered: advance by 2
    -- virtual pixels to the next sampled column, minus 1 (sample when the
    -- pixel's advance takes it negative)
    signal s_sp         : signed(8 downto 0) := (others => '0');
    signal s_col        : unsigned(7 downto 0) := (others => '0');
    signal s_firstcol   : std_logic := '1';

    ----------------------------------------------------------------------
    -- Write path (profile capture at sampled columns)
    ----------------------------------------------------------------------
    signal w0_y     : unsigned(9 downto 0) := (others => '0');
    signal w0_samp  : std_logic := '0';
    signal w0_first : std_logic := '0';
    signal w0_col   : unsigned(7 downto 0) := (others => '0');

    signal w1_prod  : unsigned(17 downto 0) := (others => '0');
    signal w1_pacc  : unsigned(13 downto 0) := (others => '0');
    signal w1_pd    : unsigned(7 downto 0) := (others => '0');
    signal w1_samp  : std_logic := '0';
    signal w1_first : std_logic := '0';
    signal w1_col   : unsigned(7 downto 0) := (others => '0');

    signal w2_acc   : unsigned(13 downto 0) := (others => '0');
    signal w2_pd    : unsigned(7 downto 0) := (others => '0');
    signal w2_samp  : std_logic := '0';
    signal w2_first : std_logic := '0';
    signal w2_col   : unsigned(7 downto 0) := (others => '0');

    signal w3_prod  : unsigned(14 downto 0) := (others => '0');
    signal w3_brt   : unsigned(3 downto 0) := (others => '0');
    signal w3_acc   : unsigned(13 downto 0) := (others => '0');
    signal w3_pd    : unsigned(7 downto 0) := (others => '0');
    signal w3_samp  : std_logic := '0';
    signal w3_col   : unsigned(7 downto 0) := (others => '0');

    signal w4_disp  : unsigned(7 downto 0) := (others => '0');
    signal w4_pd    : unsigned(7 downto 0) := (others => '0');
    signal w4_brt   : unsigned(3 downto 0) := (others => '0');
    signal w4_samp  : std_logic := '0';
    signal w4_we    : std_logic := '0';
    signal w4_addr  : unsigned(7 downto 0) := (others => '0');
    signal w4_wd    : std_logic_vector(31 downto 0) := (others => '0');

    signal w5_lo    : unsigned(7 downto 0) := (others => '0');
    signal w5_hi    : unsigned(7 downto 0) := (others => '0');
    signal w5_span  : unsigned(7 downto 0) := (others => '0');
    signal w5_brt   : unsigned(3 downto 0) := (others => '0');
    signal w5_samp  : std_logic := '0';

    ----------------------------------------------------------------------
    -- Per-column state: smoothing accumulator (bits 29..16) + the
    -- displacement drawn on the previous row (bits 7..0).  256 x 32 = 2 EBR.
    -- The read address is always the NEXT column to be sampled and the write
    -- is the column sampled 5 clocks ago, so the two never collide.
    ----------------------------------------------------------------------
    type t_cram is array (0 to 255) of std_logic_vector(31 downto 0);
    signal r_cram : t_cram := (others => (others => '0'));
    signal r_crd  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_cq   : std_logic_vector(31 downto 0) := (others => '0');
    attribute no_rw_check of r_cram : signal is true;

    ----------------------------------------------------------------------
    -- Candidates: the last NB sampled columns, index 0 = most recent
    ----------------------------------------------------------------------
    type t_b10 is array (0 to NB - 1) of unsigned(9 downto 0);
    type t_b8  is array (0 to NB - 1) of unsigned(7 downto 0);
    type t_b4  is array (0 to NB - 1) of unsigned(3 downto 0);
    type t_b3  is array (0 to NB - 1) of unsigned(2 downto 0);
    type t_b2  is array (0 to NB - 1) of unsigned(1 downto 0);
    type t_b11s is array (0 to NB - 1) of signed(10 downto 0);

    signal c_age : t_b10 := (others => (others => '0'));
    signal c_lo  : t_b8 := (others => (others => '0'));
    signal c_hi  : t_b8 := (others => (others => '0'));
    signal c_dim : t_b2 := (others => (others => '0'));
    signal c_brt : t_b4 := (others => (others => '0'));
    signal c_val : std_logic_vector(NB - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Display pipeline
    ----------------------------------------------------------------------
    signal a3_slo  : t_b11s := (others => (others => '0'));
    signal a3_shi  : t_b11s := (others => (others => '0'));
    signal a3_dim  : t_b2 := (others => (others => '0'));
    signal a3_brt  : t_b4 := (others => (others => '0'));
    signal a3_val  : std_logic_vector(NB - 1 downto 0) := (others => '0');

    signal a4_hit  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_cov  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_sh   : t_b3 := (others => (others => '0'));  -- dim shift 0..4
    signal a4_brt  : t_b4 := (others => (others => '0'));

    signal a6_hidx   : unsigned(2 downto 0) := (others => '0');
    signal a6_cidx   : unsigned(2 downto 0) := (others => '0');
    signal a6_anyhit : std_logic := '0';
    signal a6_anycov : std_logic := '0';
    signal a6_sh     : t_b3 := (others => (others => '0'));
    signal a6_brt    : t_b4 := (others => (others => '0'));

    signal a7_line  : std_logic := '0';
    signal a7_sh    : unsigned(2 downto 0) := (others => '0');
    signal a7_brt   : unsigned(3 downto 0) := (others => '0');
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
    -- two further output registers (bring the total to a multiple of 4)
    signal a12_y, a13_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal a12_u, a13_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a12_v, a13_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- Sync delay line (tap C_LAT-4 gates blanking at A11, tap C_LAT-1 drives
    -- sync out) and the input-chroma delay feeding the Video line colour
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

    type t_cdly is array (0 to C_CDLY - 1) of std_logic_vector(9 downto 0);
    signal s_cu : t_cdly := (others => "1000000000");
    signal s_cv : t_cdly := (others => "1000000000");

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

begin

    ----------------------------------------------------------------------
    -- Column sampler, row bookkeeping, vsync parameter latch
    ----------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_two     : std_logic;
        variable v_a, v_b  : signed(8 downto 0);
        variable v_inv     : unsigned(9 downto 0);
        variable v_d7      : unsigned(6 downto 0);
        variable v_dm      : unsigned(7 downto 0);
        variable v_targ    : unsigned(9 downto 0);
        variable v_diff    : signed(10 downto 0);
        variable v_next    : signed(10 downto 0);
        variable v_ro      : unsigned(8 downto 0);
        variable v_se      : unsigned(14 downto 0);
        variable v_rp      : unsigned(14 downto 0);
        variable v_tw      : unsigned(3 downto 0);
        variable v_s3      : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            if unsigned(registers_in(2)) < 8 then
                s_k3_idx <= unsigned(registers_in(2)(2 downto 0));
            else
                s_k3_idx <= unsigned(registers_in(2)(9 downto 7));
            end if;
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            -- Timing class: x step 1 / 1.5 / 2 virtual pixels per pixel, and
            -- how many virtual rows a field row spans (smoothing shift cut)
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8 =>                                   -- 480i / 576i
                    s_sd <= '1'; s_720 <= '0'; s_kc <= to_unsigned(2, 4); s_sred <= "10";
                when 4 | 12 =>                                  -- 480p / 576p
                    s_sd <= '1'; s_720 <= '0'; s_kc <= to_unsigned(2, 4); s_sred <= "01";
                when 5 | 6 | 14 =>                              -- 720p
                    s_sd <= '0'; s_720 <= '1'; s_kc <= to_unsigned(2, 4); s_sred <= "00";
                when 1 | 2 | 10 =>                              -- 1080i
                    s_sd <= '0'; s_720 <= '0'; s_kc <= to_unsigned(1, 4); s_sred <= "01";
                when others =>                                  -- 1080p
                    s_sd <= '0'; s_720 <= '0'; s_kc <= to_unsigned(1, 4); s_sred <= "00";
            end case;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- This pixel's virtual advance: 2 at SD, alternating 1/2 at 720p
            v_two := s_sd or (s_720 and s_xf);
            s_ka2 <= v_two;

            -- Column sampler.  s_sp = virtual pixels to the next sampled
            -- column, minus 1; both outcomes are computed in parallel.
            if v_two = '1' then
                v_a := s_sp - 2;
                v_b := s_sp + s_dm2;
            else
                v_a := s_sp - 1;
                v_b := s_sp + s_dm1;
            end if;
            w0_samp  <= '0';
            s_rowclr <= '0';
            if v_h_edge = '1' then
                s_row_armed <= '1';
                s_xf        <= '0';
                -- roll: first sampled column = phase x spacing
                s_sp        <= signed(resize(s_roff, 9));
                s_col       <= (others => '0');
                s_firstcol  <= '1';
                -- a line without video = vertical blanking: the next active
                -- row is the field's first
                if s_saw_av = '0' then
                    s_frow_pend <= '1';
                end if;
                s_saw_av <= '0';
            elsif data_in.avid = '1' then
                s_saw_av <= '1';
                s_xf     <= not s_xf;
                if v_a(8) = '1' then
                    s_sp       <= v_b;
                    w0_samp    <= '1';
                    w0_first   <= s_firstcol;
                    w0_col     <= s_col;
                    s_firstcol <= '0';
                    s_col      <= s_col + 1;
                else
                    s_sp <= v_a;
                end if;
                -- Row start (first active pixel)
                if s_row_armed = '1' then
                    s_row_armed <= '0';
                    s_rowclr    <= '1';
                    s_frow      <= s_frow_pend;
                    s_frow_pend <= '0';
                end if;
            end if;

            if data_in.avid = '1' then s_saw_frame <= '1'; end if;

            -- Quasi-static products (inputs change only at vsync)
            v_se := s_scale9 * s_rmul;
            s_scale_eff <= v_se(13 downto 5);
            s_rspd <= resize(s_ru * s_ru, 10) & "0000";
            v_rp := s_rphase(15 downto 8) * s_dspace;
            s_roff <= v_rp(14 downto 8);

            -- Field/frame start.  Analog vsync serrates (several edges per
            -- field): every action here is idempotent, and the relief glide
            -- and roll step are guarded to one step per field by s_saw_frame.
            if v_v_edge = '1' then
                -- the leftmost (rolling) line fades in with the phase so
                -- lines never pop in
                s_fade4 <= s_rphase(15 downto 12);

                -- K1 Scan Lines: CW = denser.  Spacing 8..71 virtual pixels.
                v_inv := 1023 - unsigned(registers_in(0));
                v_d7  := to_unsigned(8, 7) + v_inv(9 downto 4);
                s_dspace <= v_d7;
                s_dm1 <= signed(resize(v_d7, 9)) - 1;
                s_dm2 <= signed(resize(v_d7, 9)) - 2;
                -- Max displacement: a candidate lives 8 sampled columns, so a
                -- line stays retrievable while disp < 8*spacing (capped 255).
                -- Relief is normalised to that window: x min(spacing,32)/32.
                if v_d7 >= 32 then
                    s_dmax <= to_unsigned(255, 8);
                    s_rmul <= to_unsigned(32, 6);
                else
                    v_dm := shift_left(resize(v_d7, 8), 3) - 1;
                    s_dmax <= v_dm;
                    s_rmul <= resize(v_d7, 6);
                end if;

                v_tw := to_unsigned(1, 4) + resize(unsigned(registers_in(1)(9 downto 7)), 4);
                s_thick  <= v_tw;
                if v_tw < s_kc then
                    v_tw := s_kc;
                end if;
                s_thkw <= v_tw;
                s_half <= shift_right(v_tw + 1, 1);   -- ceil(Tw / 2)
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
                -- K6 Smoothing: IIR shift 0..7 down the column, cut by the
                -- field-row pitch so the reach in screen height is comparable
                v_s3 := unsigned(registers_in(5)(9 downto 7));
                if v_s3 > resize(s_sred, 3) then
                    s_smooth <= v_s3 - resize(s_sred, 3);
                else
                    s_smooth <= (others => '0');
                end if;

                -- K5 Roll: bipolar about centre, quadratic speed, up to
                -- ~0.23 line spacings per field (no wagon-wheel aliasing)
                if registers_in(4)(9) = '1' then
                    s_rdir <= '0';
                    v_ro := unsigned(registers_in(4)(8 downto 0));
                else
                    s_rdir <= '1';
                    v_ro := not unsigned(registers_in(4)(8 downto 0));
                end if;
                s_ru <= v_ro(8 downto 4);

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
                s_neg      <= registers_in(6)(4);

                -- P12 Relief -> displacement scale, glided (ease by diff/4
                -- with a +/-1 deadband) so pot ADC noise cannot make the
                -- whole line field breathe.
                if s_saw_frame = '1' then
                    s_saw_frame <= '0';
                    if s_rfreeze = '1' then
                        null;
                    elsif s_rdir = '0' then
                        s_rphase <= s_rphase + s_rspd;
                    else
                        s_rphase <= s_rphase - s_rspd;
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
    -- Per-column state RAM (canonical 1W1R; fabric re-register on the read)
    ----------------------------------------------------------------------
    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            if w4_we = '1' then
                r_cram(to_integer(w4_addr)) <= w4_wd;
            end if;
            r_crd <= r_cram(to_integer(s_col));
            r_cq  <= r_crd;
        end if;
    end process p_cram;

    ----------------------------------------------------------------------
    -- Write path: luma -> (negative) -> contrast gain -> smoothing IIR down
    -- the column -> displacement + brightness -> segment against the
    -- previous row's displacement.  Runs every pixel; only sampled columns
    -- (w*_samp) write the column RAM and push a candidate.
    ----------------------------------------------------------------------
    p_write : process(clk)
        variable v_arg  : unsigned(9 downto 0);
        variable v_l11  : unsigned(10 downto 0);
        variable v_lum  : unsigned(9 downto 0);
        variable v_diff : signed(14 downto 0);
        variable v_step : signed(14 downto 0);
        variable v_nacc : signed(14 downto 0);
        variable v_d6   : unsigned(5 downto 0);
        variable v_bm   : unsigned(7 downto 0);
        variable v_dr   : unsigned(9 downto 0);
        variable v_disp : unsigned(7 downto 0);
        variable v_ab   : signed(8 downto 0);
        variable v_ba   : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            -- W0: input register + S11 negative (w0_samp/first/col are set
            -- by the column sampler on the same edge)
            if s_neg = '1' then
                w0_y <= to_unsigned(1023, 10) - unsigned(data_in.y);
            else
                w0_y <= unsigned(data_in.y);
            end if;

            -- W1: contrast gain (unity at knob centre: x gain8/128, 0..2x);
            -- capture this column's previous-row state (the RAM read address
            -- has pointed at it since the previous sampled column)
            if w0_y > 64 then
                v_arg := w0_y - 64;
            else
                v_arg := (others => '0');
            end if;
            w1_prod  <= v_arg * s_gain8;
            w1_pacc  <= unsigned(r_cq(29 downto 16));
            w1_pd    <= unsigned(r_cq(7 downto 0));
            w1_samp  <= w0_samp;
            w1_first <= w0_first;
            w1_col   <= w0_col;

            -- W2: clamp + smoothing IIR down the column (14-bit accumulator
            -- with 4 fractional bits; restarts on the field's first row)
            v_l11 := w1_prod(17 downto 7);
            if v_l11 > 1023 then
                v_lum := to_unsigned(1023, 10);
            else
                v_lum := v_l11(9 downto 0);
            end if;
            if s_frow = '1' then
                w2_acc <= v_lum & "0000";
            else
                v_diff := signed(resize(v_lum & "0000", 15)) - signed(resize(w1_pacc, 15));
                case to_integer(s_smooth) is
                    when 0      => v_step := v_diff;
                    when 1      => v_step := shift_right(v_diff, 1);
                    when 2      => v_step := shift_right(v_diff, 2);
                    when 3      => v_step := shift_right(v_diff, 3);
                    when 4      => v_step := shift_right(v_diff, 4);
                    when 5      => v_step := shift_right(v_diff, 5);
                    when 6      => v_step := shift_right(v_diff, 6);
                    when others => v_step := shift_right(v_diff, 7);
                end case;
                v_nacc := signed(resize(w1_pacc, 15)) + v_step;
                w2_acc <= unsigned(v_nacc(13 downto 0));
            end if;
            w2_pd    <= w1_pd;
            w2_samp  <= w1_samp;
            w2_first <= w1_first;
            w2_col   <= w1_col;

            -- W3: sag term x (window-normalised) relief scale, own stage;
            -- the row's leftmost (rolling) line has its brightness faded
            if s_peakdark = '1' then
                v_d6 := w2_acc(13 downto 8);
            else
                v_d6 := 63 - w2_acc(13 downto 8);
            end if;
            w3_prod <= v_d6 * s_scale_eff;
            if w2_first = '1' then
                v_bm := w2_acc(13 downto 10) * s_fade4;
                w3_brt <= v_bm(7 downto 4);
            else
                w3_brt <= w2_acc(13 downto 10);
            end if;
            w3_acc  <= w2_acc;
            w3_pd   <= w2_pd;
            w3_samp <= w2_samp;
            w3_col  <= w2_col;

            -- W4: displacement clamp (min 1: a line never lands on its own
            -- source column) + column-state write-back.  On the field's
            -- first row there is no previous row: the segment is a point.
            v_dr := w3_prod(14 downto 5);
            if v_dr > resize(s_dmax, 10) then
                v_disp := s_dmax;
            elsif v_dr = 0 then
                v_disp := to_unsigned(1, 8);
            else
                v_disp := v_dr(7 downto 0);
            end if;
            w4_disp <= v_disp;
            if s_frow = '1' then
                w4_pd <= v_disp;
            else
                w4_pd <= w3_pd;
            end if;
            w4_brt  <= w3_brt;
            w4_samp <= w3_samp;
            w4_we   <= w3_samp;
            w4_addr <= w3_col;
            w4_wd   <= "00" & std_logic_vector(w3_acc) & x"00" & std_logic_vector(v_disp);

            -- W5: segment extent between the previous row's position and
            -- this row's (lo = left, hi = right, span = hi - lo)
            v_ab := signed(resize(w4_pd, 9)) - signed(resize(w4_disp, 9));
            v_ba := signed(resize(w4_disp, 9)) - signed(resize(w4_pd, 9));
            if v_ab(8) = '1' then
                w5_lo   <= w4_pd;
                w5_hi   <= w4_disp;
                w5_span <= unsigned(v_ba(7 downto 0));
            else
                w5_lo   <= w4_disp;
                w5_hi   <= w4_pd;
                w5_span <= unsigned(v_ab(7 downto 0));
            end if;
            w5_brt  <= w4_brt;
            w5_samp <= w4_samp;
        end if;
    end process p_write;

    ----------------------------------------------------------------------
    -- Candidates (W6): push the new sampled column at index 0 with age 0 -
    -- the display pipeline is at that very column on the clock it appears -
    -- and age everything by the pixel's virtual advance.  An age of 512+
    -- holds (past any displacement: pure skirt), so it can never wrap.
    ----------------------------------------------------------------------
    p_cand : process(clk)
        variable v_t4  : unsigned(7 downto 0);
        variable v_dim : unsigned(1 downto 0);
        variable v_inc : t_b10;
    begin
        if rising_edge(clk) then
            -- beam-speed dimming from the segment length (>T: 1/2, >4T: 1/4)
            v_t4 := shift_left(resize(s_thick, 8), 2);
            if w5_span > v_t4 then
                v_dim := "10";
            elsif w5_span > resize(s_thick, 8) then
                v_dim := "01";
            else
                v_dim := "00";
            end if;

            for k in 0 to NB - 1 loop
                if c_age(k)(9) = '1' then
                    v_inc(k) := c_age(k);
                elsif s_ka2 = '1' then
                    v_inc(k) := c_age(k) + 2;
                else
                    v_inc(k) := c_age(k) + 1;
                end if;
            end loop;

            if w5_samp = '1' then
                c_age(0) <= (others => '0');
                c_lo(0)  <= w5_lo;
                c_hi(0)  <= w5_hi;
                c_dim(0) <= v_dim;
                c_brt(0) <= w5_brt;
                c_val(0) <= '1';
                for k in 1 to NB - 1 loop
                    c_age(k) <= v_inc(k - 1);
                    c_lo(k)  <= c_lo(k - 1);
                    c_hi(k)  <= c_hi(k - 1);
                    c_dim(k) <= c_dim(k - 1);
                    c_brt(k) <= c_brt(k - 1);
                    c_val(k) <= c_val(k - 1);
                end loop;
            else
                for k in 0 to NB - 1 loop
                    c_age(k) <= v_inc(k);
                end loop;
            end if;

            -- New row: drop the previous row's candidates (its last display
            -- pixel left the pipeline during horizontal blanking; this row's
            -- first push is still 5 clocks away)
            if s_rowclr = '1' then
                c_val <= (others => '0');
            end if;
        end if;
    end process p_cand;

    ----------------------------------------------------------------------
    -- Display pipeline: candidate tests -> recency-priority resolve ->
    -- palette + shading -> compose -> [1 2 1] -> blanking
    ----------------------------------------------------------------------
    p_disp : process(clk)
        variable v_t    : signed(10 downto 0);
        variable v_2t   : signed(10 downto 0);
        variable v_nt   : signed(10 downto 0);
        variable v_core : boolean;
        variable v_halo : boolean;
        variable v_hin  : boolean;
        variable v_nh   : signed(10 downto 0);
        variable v_th   : signed(10 downto 0);
        variable v_cov  : boolean;
        variable v_sh   : unsigned(2 downto 0);
        variable v_hidx : integer range 0 to 7;
        variable v_cidx : integer range 0 to 7;
        variable v_anyh : std_logic;
        variable v_anyc : std_logic;
        variable v_line : std_logic;
        variable v_cs   : signed(9 downto 0);
        variable v_alt  : unsigned(2 downto 0);
        variable v_bf   : unsigned(3 downto 0);
        variable v_bf5  : unsigned(4 downto 0);
        variable v_yi   : unsigned(10 downto 0);
        variable v_ui   : integer range -2048 to 2047;
        variable v_vi   : integer range -2048 to 2047;
        variable v_fs   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- A3: candidate offsets against both segment ends
            for k in 0 to NB - 1 loop
                a3_slo(k) <= signed(resize(c_age(k), 11)) - signed(resize(c_lo(k), 11));
                a3_shi(k) <= signed(resize(c_age(k), 11)) - signed(resize(c_hi(k), 11));
                a3_dim(k) <= c_dim(k);
                a3_brt(k) <= c_brt(k);
            end loop;
            a3_val <= c_val;

            -- A4: classify each candidate against the stroke window
            v_t  := signed(resize(s_thkw, 11));
            v_2t := signed(resize(shift_left(resize(s_thkw, 5), 1), 11));
            v_nt := -v_t;
            v_nh := -signed(resize(s_half, 11));
            v_th := v_t + signed(resize(s_half, 11));
            for k in 0 to NB - 1 loop
                v_cov  := a3_val(k) = '1' and a3_slo(k) >= 0;
                v_core := v_cov and a3_shi(k) < v_t;
                v_halo := s_glow = '1' and a3_val(k) = '1' and
                          ((a3_slo(k) >= v_nt and a3_slo(k) < 0) or
                           (a3_shi(k) >= v_t and a3_shi(k) < v_2t));
                if v_core or v_halo then a4_hit(k) <= '1'; else a4_hit(k) <= '0'; end if;
                if v_cov then a4_cov(k) <= '1'; else a4_cov(k) <= '0'; end if;
                -- total dim shift: beam-speed 0..2, plus the glow level for
                -- halo pixels (inner half of the halo / outer half +1)
                v_hin := a3_slo(k) >= v_nh and a3_shi(k) < v_th;
                v_sh := resize(a3_dim(k), 3);
                if v_halo and not v_core then
                    if v_hin then
                        v_sh := v_sh + s_gsh;
                    else
                        v_sh := v_sh + s_gsh + 1;
                    end if;
                end if;
                a4_sh(k)  <= v_sh;
                a4_brt(k) <= a3_brt(k);
            end loop;

            -- A6: priority encode.  Index 0 is the most recent column =
            -- the nearest line = hidden-line priority (lowest index wins).
            v_hidx := 7; v_anyh := '0';
            v_cidx := 7; v_anyc := '0';
            for i in NB - 1 downto 0 loop
                if a4_hit(i) = '1' then v_hidx := i; v_anyh := '1'; end if;
                if a4_cov(i) = '1' then v_cidx := i; v_anyc := '1'; end if;
            end loop;
            a6_hidx   <= to_unsigned(v_hidx, 3);
            a6_cidx   <= to_unsigned(v_cidx, 3);
            a6_anyhit <= v_anyh;
            a6_anycov <= v_anyc;
            a6_sh     <= a4_sh;
            a6_brt    <= a4_brt;

            -- A7: draw decision + winner payload mux.  A hit draws unless a
            -- strictly nearer line's skirt covers this pixel (hidden-line
            -- mode only); skirts and empty space are both black.
            if a6_anyhit = '1' and (s_hlr = '0' or a6_anycov = '0' or a6_hidx <= a6_cidx) then
                v_line := '1';
            else
                v_line := '0';
            end if;
            a7_line <= v_line;
            a7_brt  <= a6_brt(to_integer(a6_hidx));
            a7_sh   <= a6_sh(to_integer(a6_hidx));
            -- A7 (Video lane): source chroma at this display pixel, x2
            -- saturation, clamped to +/-448
            v_cs := signed(s_cu(C_CDLY - 1) xor "1000000000");
            if v_cs > 224 then
                a7_cu <= to_signed(448, 10);
            elsif v_cs < -224 then
                a7_cu <= to_signed(-448, 10);
            else
                a7_cu <= shift_left(v_cs, 1);
            end if;
            v_cs := signed(s_cv(C_CDLY - 1) xor "1000000000");
            if v_cs > 224 then
                a7_cv <= to_signed(448, 10);
            elsif v_cs < -224 then
                a7_cv <= to_signed(-448, 10);
            else
                a7_cv <= shift_left(v_cs, 1);
            end if;

            -- A8: palette + shading factor
            v_alt := a7_brt(3 downto 1) xor (2 downto 0 => not s_peakdark);
            if s_zone = 7 then
                a8_yo <= C_TY(to_integer(v_alt));
                a8_uo <= C_TU(to_integer(v_alt));
                a8_vo <= C_TV(to_integer(v_alt));
            elsif s_zone = 5 then
                a8_yo <= C_ZY(5);
                a8_uo <= a7_cu;
                a8_vo <= a7_cv;
            else
                a8_yo <= C_ZY(to_integer(s_zone));
                a8_uo <= C_ZU(to_integer(s_zone));
                a8_vo <= C_ZV(to_integer(s_zone));
            end if;
            if s_shade = '1' then
                v_bf := a7_brt;
            elsif a7_brt >= 8 then
                v_bf := to_unsigned(15, 4);
            else
                v_bf := to_unsigned(4, 4);
            end if;
            case to_integer(a7_sh) is
                when 0      => null;
                when 1      => v_bf := shift_right(v_bf, 1);
                when 2      => v_bf := shift_right(v_bf, 2);
                when 3      => v_bf := shift_right(v_bf, 3);
                when others => v_bf := (others => '0');
            end case;
            -- +1 (full scale at 16) plus the thin-stroke lift, held at the
            -- 768 drawn-luma cap (bf1 = 16)
            v_bf5 := resize(v_bf, 5) + 1 + resize(s_bboost, 5);
            if v_bf5 > 16 then
                v_bf5 := to_unsigned(16, 5);
            end if;
            -- thin-stroke gain against the output filter (16 -> 24 / 20)
            case s_tgain is
                when "10"   => a8_bf1 <= resize(v_bf5, 6) + resize(shift_right(v_bf5, 1), 6);
                when "01"   => a8_bf1 <= resize(v_bf5, 6) + resize(shift_right(v_bf5, 2), 6);
                when others => a8_bf1 <= resize(v_bf5, 6);
            end case;
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
            -- avid = 0), then two plain output registers
            if s_dly(C_LAT - 4).avid = '1' then
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
            a12_y <= a11_y;  a13_y <= a12_y;
            a12_u <= a11_u;  a13_u <= a12_u;
            a12_v <= a11_v;  a13_v <= a12_v;
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
            s_cu(0) <= data_in.u;
            s_cv(0) <= data_in.v;
            for i in 1 to C_CDLY - 1 loop
                s_cu(i) <= s_cu(i - 1);
                s_cv(i) <= s_cv(i - 1);
            end loop;
        end if;
    end process p_dly;

    ----------------------------------------------------------------------
    -- Outputs
    ----------------------------------------------------------------------
    s_io_align_in.y <= std_logic_vector(a13_y);
    s_io_align_in.u <= std_logic_vector(a13_u);
    s_io_align_in.v <= std_logic_vector(a13_v);

    s_io_align_in.avid    <= s_dly(C_LAT - 1).avid;
    s_io_align_in.hsync_n <= s_dly(C_LAT - 1).hsync_n;
    s_io_align_in.vsync_n <= s_dly(C_LAT - 1).vsync_n;
    s_io_align_in.field_n <= s_dly(C_LAT - 1).field_n;

    -- IO-align pad: video and syncs delayed together
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad;

end ruttetra_v;
