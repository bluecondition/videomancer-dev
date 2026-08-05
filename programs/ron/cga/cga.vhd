-- Videomancer CGA (Color Graphics Adapter) Emulator
-- LZX Industries Community FPGA Program
--
-- Pixelation algorithm: downsample + nearest-neighbour upscale.
--   1. DOWNSAMPLE: each horizontal cell on the FIRST line of its cell row
--      accumulates H input pixels and divides for the average colour.
--   2. STORE: palette-match the average and write the 2-bit index into a
--      per-cell-column buffer indexed by column number.
--   3. UPSCALE (nearest-neighbour): every output pixel reads the palette
--      index for its cell column and emits the corresponding colour, giving
--      solid blocks across all V lines of each cell row.
--
-- Anti-flicker (v1.2):
--   * TEMPORAL HYSTERESIS ("Stability", knob 6): a ping-pong pair of BRAM
--     grids remembers per 16x16-PIXEL tile the current colour and a pending
--     challenger (two-field debounce); a cell
--     only changes colour when the new match beats the incumbent's distance
--     by the Stability margin.  Monte-Carlo modelling puts this at ~10-400x
--     fewer noise-flipped cells (margin 1-2); cell-size quantisation tricks
--     (v1.1's 4-bit coarse match) modelled as a placebo and were reverted.
--     Cells >= 8x8 px map 1:1 to tiles; smaller cells share a tile, which
--     still pins the flat areas where flicker is actually visible.
--   * Pipeline reworked to genuinely close HD timing (74.25 MHz); the old
--     build shipped through timing violations at HD via timing-allow-fail.
--   * Contrast highlight wrap fixed (result now clamps at 1023 instead of
--     wrapping to black on bright pixels at >50% contrast).
--   * Scanlines softened: 25% dim with chroma tracking the same factor.
--
-- Pipeline (10 stages, C_LATENCY = 10):
--   T1  input register, controls, cell tracking, horizontal accumulator
--   T1b average divide (barrel shift) + match-input select
--   T2a contrast multiply (7-bit coefficient) + clamp
--   T2b 4 palette distances (top-5-bit Manhattan) + luma-zone index
--   T3  4-way minimum -> palette index, register buffer write
--   buf per-cell-column BRAM (1 clock read)
--   T4a display palette-entry lookup (buffered or per-pixel index)
--   T4b saturation: subtract 512, 12x9 signed multiply
--   T5  saturation slice/add/clamp, scanlines, output register
--   + interpolator wet/dry mix (dry tap s_yuv_sr(5))
--
-- License: GPL-3.0
-- Author: Claude (Anthropic)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cga of program_top is

  -- ==========================================================================
  -- Constants
  -- ==========================================================================

  constant C_LATENCY  : integer := 10;
  constant C_MAX_COLS : integer := 2048;  -- 2048x2 fits one iCE40 BRAM; covers
                                          -- up to 1920 cells/line at H=1 (Off).

  -- Palette entry type
  type t_pal_entry is record
    y : unsigned(9 downto 0);
    u : unsigned(9 downto 0);
    v : unsigned(9 downto 0);
  end record;

  type t_cga_pal4 is array(0 to 3) of t_pal_entry;
  type t_cga_pals is array(0 to 5) of t_cga_pal4;

  -- Six CGA palettes: 3 modes × 2 intensities (U/V swapped for Videomancer)
  constant C_CGA_PAL : t_cga_pals := (
    -- Palette 0 Low: Black, Red, Green, Yellow
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(204, 10), u => to_unsigned(852, 10), v => to_unsigned(396, 10)),
      (y => to_unsigned(400, 10), u => to_unsigned(226, 10), v => to_unsigned(286, 10)),
      (y => to_unsigned(404, 10), u => to_unsigned(709, 10), v => to_unsigned(283, 10))
    ),
    -- Palette 0 High: Black, Red, Green, Yellow
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(546, 10), u => to_unsigned(853, 10), v => to_unsigned(397, 10)),
      (y => to_unsigned(743, 10), u => to_unsigned(226, 10), v => to_unsigned(286, 10)),
      (y => to_unsigned(947, 10), u => to_unsigned(568, 10), v => to_unsigned(171, 10))
    ),
    -- Palette 1 Low: Black, Cyan, Magenta, Lt Gray
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(478, 10), u => to_unsigned(171, 10), v => to_unsigned(627, 10)),
      (y => to_unsigned(282, 10), u => to_unsigned(798, 10), v => to_unsigned(738, 10)),
      (y => to_unsigned(683, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    ),
    -- Palette 1 High: Black, Lt Cyan, Lt Magenta, White
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(820, 10), u => to_unsigned(171, 10), v => to_unsigned(628, 10)),
      (y => to_unsigned(624, 10), u => to_unsigned(798, 10), v => to_unsigned(739, 10)),
      (y => to_unsigned(1023, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    ),
    -- Palette 2 Low: Black, Cyan, Red, Lt Gray
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(478, 10), u => to_unsigned(171, 10), v => to_unsigned(627, 10)),
      (y => to_unsigned(204, 10), u => to_unsigned(852, 10), v => to_unsigned(396, 10)),
      (y => to_unsigned(683, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    ),
    -- Palette 2 High: Black, Lt Cyan, Lt Red, White
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(820, 10), u => to_unsigned(171, 10), v => to_unsigned(628, 10)),
      (y => to_unsigned(546, 10), u => to_unsigned(853, 10), v => to_unsigned(397, 10)),
      (y => to_unsigned(1023, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    )
  );

  -- ==========================================================================
  -- Manhattan distance on top 5 bits (32 levels per channel).
  -- NOTE: coarsening this to 4 bits was tried (v1.1) and modelled: it does
  -- NOT reduce boundary flicker (the noise-flip zone just moves with the
  -- decision boundary; its size is set by the noise, not the quantum).
  -- ==========================================================================
  function f_dist(
    py, pu, pv : unsigned(9 downto 0);
    cy, cu, cv : unsigned(9 downto 0)
  ) return unsigned is
    variable dy, du, dv : unsigned(4 downto 0);
  begin
    if py(9 downto 5) > cy(9 downto 5) then
      dy := py(9 downto 5) - cy(9 downto 5);
    else
      dy := cy(9 downto 5) - py(9 downto 5);
    end if;
    if pu(9 downto 5) > cu(9 downto 5) then
      du := pu(9 downto 5) - cu(9 downto 5);
    else
      du := cu(9 downto 5) - pu(9 downto 5);
    end if;
    if pv(9 downto 5) > cv(9 downto 5) then
      dv := pv(9 downto 5) - cv(9 downto 5);
    else
      dv := cv(9 downto 5) - pv(9 downto 5);
    end if;
    return resize(dy, 7) + resize(du, 7) + resize(dv, 7);
  end function;

  -- ==========================================================================
  -- Luma-mode palette mapping
  -- ==========================================================================
  -- For each of the 6 palette variants, maps a 2-bit input-luma zone
  -- (0 = darkest quarter of Y, 3 = brightest) to the palette entry index
  -- whose luma sorts to that rank.  Computed offline by sorting each
  -- palette's 4 entries by Y value.  Entries within a palette with the
  -- same rank use the entry whose Y happens to be closest (stable sort).
  type t_luma_map  is array(0 to 3) of unsigned(1 downto 0);
  type t_luma_maps is array(0 to 5) of t_luma_map;
  constant C_LUMA_MAP : t_luma_maps := (
    -- Pal 0 Low:  Y = {0, 204, 400, 404} -> sorted indices {0,1,2,3}
    0 => (0 => "00", 1 => "01", 2 => "10", 3 => "11"),
    -- Pal 0 High: Y = {0, 546, 743, 947} -> {0,1,2,3}
    1 => (0 => "00", 1 => "01", 2 => "10", 3 => "11"),
    -- Pal 1 Low:  Y = {0, 478, 282, 683} -> {0,2,1,3}
    2 => (0 => "00", 1 => "10", 2 => "01", 3 => "11"),
    -- Pal 1 High: Y = {0, 820, 624, 1023} -> {0,2,1,3}
    3 => (0 => "00", 1 => "10", 2 => "01", 3 => "11"),
    -- Pal 2 Low:  Y = {0, 478, 204, 683} -> {0,2,1,3}
    4 => (0 => "00", 1 => "10", 2 => "01", 3 => "11"),
    -- Pal 2 High: Y = {0, 820, 546, 1023} -> {0,2,1,3}
    5 => (0 => "00", 1 => "10", 2 => "01", 3 => "11")
  );

  -- ==========================================================================
  -- Signal declarations
  -- ==========================================================================

  -- Sync handling
  signal s_pixel_y : unsigned(11 downto 0) := (others => '0');
  signal s_prev_hsync_n : std_logic := '1';
  signal s_prev_vsync_n : std_logic := '1';

  -- Shift registers for sync passthrough and dry-tap
  type t_sr_logic is array(0 to C_LATENCY - 1) of std_logic;
  type t_sr_yuv   is array(0 to C_LATENCY - 1) of t_video_stream_yuv444_30b;
  signal s_avid_sr  : t_sr_logic := (others => '0');
  signal s_hsync_sr : t_sr_logic := (others => '1');
  signal s_vsync_sr : t_sr_logic := (others => '1');
  signal s_field_sr : t_sr_logic := (others => '0');
  signal s_yuv_sr   : t_sr_yuv;

  -- Cell sizing (decoded from knobs 2, 3). Registered controls: the cell
  -- tracking below must use these, never the freshly-decoded values, to keep
  -- the slow SPI-RAM register net off the per-pixel critical path.
  signal s_h_size  : unsigned(4 downto 0) := to_unsigned(4, 5);
  signal s_v_size  : unsigned(4 downto 0) := to_unsigned(4, 5);
  signal s_h_shift : unsigned(2 downto 0) := to_unsigned(2, 3);

  -- Cell tracking
  signal s_cell_x   : unsigned(4 downto 0)  := (others => '0');
  signal s_cell_y   : unsigned(4 downto 0)  := (others => '0');
  signal s_cell_col : unsigned(10 downto 0) := (others => '0');

  -- Horizontal accumulator (first line of cell row only)
  signal s_acc_y : unsigned(13 downto 0) := (others => '0');
  signal s_acc_u : unsigned(13 downto 0) := (others => '0');
  signal s_acc_v : unsigned(13 downto 0) := (others => '0');

  -- T1 -> T1b: use the completed cell average (vs raw pixel) next stage
  signal s_use_avg : std_logic := '0';

  -- Per-cell palette-index buffer
  type t_pidx_buf is array(0 to C_MAX_COLS - 1) of unsigned(1 downto 0);
  signal s_pidx_buf : t_pidx_buf := (others => "00");

  -- ==========================================================================
  -- Temporal hysteresis grid (v1.3): per fixed 16x16-PIXEL tile, a 4-bit
  -- entry packing {pending(2b), current(2b)} for the two-field debounce.
  -- 120 x 68 = 8160 entries covers 1080p's full 1080 active lines (the v1.2
  -- 8x8 grid was sized for 540i fields and WRAPPED on progressive HD,
  -- aliasing the bottom half onto the top -- seen directly in Freeze mode).
  -- Cells of 16x16 px map 1:1; smaller cells share a tile, which still
  -- stabilises the flat areas where flicker is visible.  Two banks ping-pong
  -- on field parity: reads always reference the bank written LAST field, so
  -- a same-address read+write collision (the suspected killer of the old C64
  -- hysteresis attempt) is impossible by construction.  Both banks are read
  -- unconditionally and muxed after the register, or yosys maps them to
  -- logic instead of BRAM (C64 double-buffer lesson).  8 EBRs per bank.
  -- ==========================================================================
  constant C_GRID_SIZE : integer := 8192;
  type t_hyst_grid is array(0 to C_GRID_SIZE - 1) of unsigned(3 downto 0);
  signal s_grid_a : t_hyst_grid := (others => "0000");
  signal s_grid_b : t_hyst_grid := (others => "0000");

  signal s_fparity     : std_logic := '0';
  signal s_grid_raddr  : unsigned(12 downto 0) := (others => '0');
  signal s_grid_waddr  : unsigned(12 downto 0) := (others => '0');
  signal s_grid_we     : std_logic := '0';
  signal s_grid_wdata  : unsigned(3 downto 0) := (others => '0');
  signal s_grid_rd_a   : unsigned(3 downto 0) := (others => '0');
  signal s_grid_rd_b   : unsigned(3 downto 0) := (others => '0');
  signal s_grid_prev   : unsigned(3 downto 0) := (others => '0');

  -- Tile coordinates captured at cell end (T1) and folded to a grid address
  -- in T1b (addr = row * 120 + col, done as (row<<7) - (row<<3) + col).
  signal s_trow_d1 : unsigned(6 downto 0) := (others => '0');
  signal s_tcol_d1 : unsigned(6 downto 0) := (others => '0');
  signal s_gaddr_d3, s_gaddr_d4 : unsigned(12 downto 0) := (others => '0');

  -- Active-line-gated row phase (blanking hsyncs must not advance the cell
  -- row/tile phase or the two interlaced fields land on different phases --
  -- the C64 red-bar lesson).
  signal s_line_act : std_logic := '0';
  signal s_act_row  : unsigned(10 downto 0) := (others => '0');  -- 11 bits:
                     -- 1080p has 1080 active lines; 10 bits wrapped at 1024
  signal s_px       : unsigned(10 downto 0) := (others => '0');

  -- Set once active video has been seen this field; cleared when the bank
  -- parity toggles.  Analog vsync is SERRATED (multiple falling edges per
  -- field): a bare edge-toggle would flip parity an even number of times,
  -- making reads hit the bank being written -> same-address EBR collisions
  -- -> garbage.  Resets may repeat harmlessly; the toggle must not.
  signal s_saw_act  : std_logic := '0';

  -- Stability knob (rotary 6): hysteresis margin, 0 = off
  signal s_margin   : unsigned(2 downto 0) := to_unsigned(2, 3);

  -- Bootstrap: for the first fields after configuration the grid holds all
  -- zeros (black); in content far from every palette entry the four
  -- distances nearly tie, so nothing would ever beat a black incumbent by
  -- the margin and the init state would stick.  Seed the grid with the
  -- plain match for two fields before engaging the keep-rule.
  signal s_boot     : unsigned(1 downto 0) := (others => '0');

  -- Decision-side controls (contrast, colour mode) only influence NEW match
  -- decisions, which Freeze suppresses entirely -- so turning them would do
  -- nothing at high Stability.  Snapshot them once per field and re-run the
  -- two-field bootstrap seed when they move, so the image re-quantises with
  -- the new setting and then re-stabilises.
  signal s_con_snap : unsigned(9 downto 0) := to_unsigned(410, 10);
  signal s_cm_snap  : std_logic := '0';

  signal s_buf_we      : std_logic := '0';
  signal s_buf_waddr   : unsigned(10 downto 0) := (others => '0');
  signal s_buf_wdata   : unsigned(1 downto 0) := (others => '0');
  signal s_buf_raddr   : unsigned(10 downto 0) := (others => '0');
  signal s_buf_rdata   : unsigned(1 downto 0) := (others => '0');

  -- Palette match pipeline
  signal s_match_y : unsigned(9 downto 0) := (others => '0');
  signal s_match_u : unsigned(9 downto 0) := (others => '0');
  signal s_match_v : unsigned(9 downto 0) := (others => '0');

  -- T2a -> T2b: contrast-adjusted Y + delayed chroma
  signal s_ycon    : unsigned(9 downto 0) := (others => '0');
  signal s_mu_d1   : unsigned(9 downto 0) := (others => '0');
  signal s_mv_d1   : unsigned(9 downto 0) := (others => '0');

  type t_dist_arr is array(0 to 3) of unsigned(6 downto 0);
  signal s_dist : t_dist_arr := (others => (others => '0'));
  signal s_match_idx : unsigned(1 downto 0) := (others => '0');

  -- Luma-mode: palette entry chosen by input-Y zone (set in T2b, used in T3)
  signal s_luma_idx : unsigned(1 downto 0) := (others => '0');

  -- Switch 10: '0' = nearest-colour (Manhattan), '1' = luma-zone mapping
  signal s_color_mode : std_logic := '0';

  -- Pipelined fill (write to buffer), aligned to the match pipeline depth
  signal s_we_d1, s_we_d2, s_we_d3, s_we_d4, s_we_d5 : std_logic := '0';
  signal s_wcol_d1, s_wcol_d2, s_wcol_d3, s_wcol_d4, s_wcol_d5 : unsigned(10 downto 0) := (others => '0');

  -- T3a -> T3b: challenger + distances for the hysteresis decision
  signal s_chal      : unsigned(1 downto 0) := (others => '0');
  signal s_chal_dist : unsigned(6 downto 0) := (others => '0');
  signal s_prev_dist : unsigned(6 downto 0) := (others => '0');
  signal s_ycon_d1   : unsigned(9 downto 0) := (others => '0');
  signal s_prev_d5   : unsigned(1 downto 0) := (others => '0');  -- current
  signal s_pend_d5   : unsigned(1 downto 0) := (others => '0');  -- pending
  signal s_gaddr_d5  : unsigned(12 downto 0) := (others => '0');

  -- Palette selection (knob 1 + intensity switch). Only changes on a knob
  -- turn, so no per-stage alignment delays are needed.
  signal s_pal_sel : unsigned(2 downto 0) := (others => '0');

  -- T4a -> T4b: display palette entry
  signal s_disp_y, s_disp_u, s_disp_v : unsigned(9 downto 0) := (others => '0');

  -- T4b -> T5 saturation intermediates (12x9 signed product)
  signal s_t4_su_prod : signed(20 downto 0) := (others => '0');
  signal s_t4_sv_prod : signed(20 downto 0) := (others => '0');
  signal s_t4_y       : unsigned(9 downto 0) := (others => '0');

  -- Output
  signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

  -- Controls
  signal s_bypass     : std_logic := '0';
  signal s_pixelate   : std_logic := '0';
  signal s_scanlines  : std_logic := '0';
  signal s_contrast   : unsigned(9 downto 0) := (others => '0');
  signal s_saturation : unsigned(9 downto 0) := to_unsigned(512, 10);
  -- Mix, registered locally and reduced to 8 bits: keeps the slow SPI-RAM
  -- register net off the interpolators and shrinks their per-channel
  -- multiply (the mix multiplier was the HD critical path at 10 bits).
  signal s_mix_t      : unsigned(7 downto 0) := (others => '0');

  signal s_interp_y_result : unsigned(9 downto 0);
  signal s_interp_u_result : unsigned(9 downto 0);
  signal s_interp_v_result : unsigned(9 downto 0);

begin

  s_bypass <= registers_in(6)(4);

  -- ==========================================================================
  -- Pipeline T1: input register, controls, cell tracking, accumulator.
  -- ==========================================================================
  p_t1 : process(clk)
    variable v_h_size  : unsigned(4 downto 0);
    variable v_v_size  : unsigned(4 downto 0);
    variable v_h_shift : unsigned(2 downto 0);
    variable v_pal_idx : unsigned(1 downto 0);
    variable v_at_cell_end : boolean;
    variable v_origin : unsigned(10 downto 0);
  begin
    if rising_edge(clk) then
      s_avid_sr(0)  <= data_in.avid;
      s_hsync_sr(0) <= data_in.hsync_n;
      s_vsync_sr(0) <= data_in.vsync_n;
      s_field_sr(0) <= data_in.field_n;
      s_yuv_sr(0)   <= data_in;
      for i in 1 to C_LATENCY - 1 loop
        s_avid_sr(i)  <= s_avid_sr(i - 1);
        s_hsync_sr(i) <= s_hsync_sr(i - 1);
        s_vsync_sr(i) <= s_vsync_sr(i - 1);
        s_field_sr(i) <= s_field_sr(i - 1);
        s_yuv_sr(i)   <= s_yuv_sr(i - 1);
      end loop;

      s_prev_hsync_n <= data_in.hsync_n;
      s_prev_vsync_n <= data_in.vsync_n;

      -- Controls
      s_pixelate   <= registers_in(6)(0);
      s_scanlines  <= registers_in(6)(2);
      s_color_mode <= registers_in(6)(3);  -- switch 10: nearest vs luma
      s_contrast   <= unsigned(registers_in(3));
      s_saturation <= unsigned(registers_in(4));
      s_mix_t      <= unsigned(registers_in(7)(9 downto 2));

      -- Stability (knob 6): hysteresis margin in top-5-bit distance units.
      -- "111" = Max = hard freeze (keep the stored colour unconditionally
      -- after bootstrap) -- also a hardware diagnostic: if Max still
      -- flashes, the grid BRAM itself is not working on silicon.
      if    unsigned(registers_in(5)) < 205 then s_margin <= "000";  -- Off
      elsif unsigned(registers_in(5)) < 410 then s_margin <= "001";
      elsif unsigned(registers_in(5)) < 615 then s_margin <= "010";
      elsif unsigned(registers_in(5)) < 820 then s_margin <= "011";
      else                                       s_margin <= "111";  -- Freeze
      end if;

      -- H/V sizes. Pixelate OFF forces 1x1 (per-pixel quantisation).
      -- Zone thresholds match the framework's 5-label divide of the 0-1023
      -- register range (5 equal zones of ~205 units), so the displayed
      -- label always matches the actual block size.
      if registers_in(6)(0) = '0' then
        v_h_size  := to_unsigned(1, 5);
        v_v_size  := to_unsigned(1, 5);
        v_h_shift := to_unsigned(0, 3);
      else
        if    unsigned(registers_in(1)) < 205 then v_h_size := to_unsigned( 1, 5); v_h_shift := to_unsigned(0, 3);
        elsif unsigned(registers_in(1)) < 410 then v_h_size := to_unsigned( 2, 5); v_h_shift := to_unsigned(1, 3);
        elsif unsigned(registers_in(1)) < 615 then v_h_size := to_unsigned( 4, 5); v_h_shift := to_unsigned(2, 3);
        elsif unsigned(registers_in(1)) < 820 then v_h_size := to_unsigned( 8, 5); v_h_shift := to_unsigned(3, 3);
        else                                       v_h_size := to_unsigned(16, 5); v_h_shift := to_unsigned(4, 3);
        end if;

        if    unsigned(registers_in(2)) < 205 then v_v_size := to_unsigned( 1, 5);
        elsif unsigned(registers_in(2)) < 410 then v_v_size := to_unsigned( 2, 5);
        elsif unsigned(registers_in(2)) < 615 then v_v_size := to_unsigned( 4, 5);
        elsif unsigned(registers_in(2)) < 820 then v_v_size := to_unsigned( 8, 5);
        else                                       v_v_size := to_unsigned(16, 5);
        end if;
      end if;
      s_h_size  <= v_h_size;
      s_v_size  <= v_v_size;
      s_h_shift <= v_h_shift;

      -- Palette selection (knob 1, 3 zones; switch 8 = intensity)
      v_pal_idx := "00";
      if    unsigned(registers_in(0)) >= 683 then v_pal_idx := "10";
      elsif unsigned(registers_in(0)) >= 341 then v_pal_idx := "01";
      end if;
      s_pal_sel <= v_pal_idx & registers_in(6)(1);

      -- Cell tracking + horizontal accumulator (registered sizes only)
      v_at_cell_end := false;

      if data_in.avid = '1' then
        s_line_act <= '1';
        s_saw_act  <= '1';
        s_px       <= s_px + 1;
        if s_cell_x >= s_h_size - 1 then
          s_cell_x <= (others => '0');
          v_at_cell_end := true;
          s_cell_col <= s_cell_col + 1;
        else
          s_cell_x <= s_cell_x + 1;
        end if;

        if s_cell_y = 0 then
          if s_cell_x = 0 then
            s_acc_y <= resize(unsigned(data_in.y), 14);
            s_acc_u <= resize(unsigned(data_in.u), 14);
            s_acc_v <= resize(unsigned(data_in.v), 14);
          else
            s_acc_y <= s_acc_y + resize(unsigned(data_in.y), 14);
            s_acc_u <= s_acc_u + resize(unsigned(data_in.u), 14);
            s_acc_v <= s_acc_v + resize(unsigned(data_in.v), 14);
          end if;
        end if;
      end if;

      if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
        s_cell_x   <= (others => '0');
        s_cell_col <= (others => '0');
        s_px       <= (others => '0');
        s_pixel_y  <= s_pixel_y + 1;
        -- Advance the row phase on ACTIVE lines only: blanking hsyncs must
        -- not desync it or the interlaced fields land on different phases.
        if s_line_act = '1' then
          s_act_row <= s_act_row + 1;
          if s_cell_y >= s_v_size - 1 then
            s_cell_y <= (others => '0');
          else
            s_cell_y <= s_cell_y + 1;
          end if;
        end if;
        s_line_act <= '0';
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_pixel_y  <= (others => '0');
        s_cell_x   <= (others => '0');
        s_cell_y   <= (others => '0');
        s_cell_col <= (others => '0');
        s_px       <= (others => '0');
        s_act_row  <= (others => '0');
        s_line_act <= '0';
        -- Swap grid banks once per field: only on the FIRST vsync edge
        -- after active video (serration-proof; see s_saw_act).
        if s_saw_act = '1' then
          s_fparity <= not s_fparity;
          s_saw_act <= '0';
          if s_boot /= "11" then
            s_boot <= s_boot + 1;
          end if;
          -- Re-seed on decision-side control changes (threshold rides out
          -- pot jitter; snapshot only updates on trigger so slow turns
          -- accumulate and still fire).
          if (s_contrast > s_con_snap and s_contrast - s_con_snap > 16)
             or (s_con_snap > s_contrast and s_con_snap - s_contrast > 16) then
            s_con_snap <= s_contrast;
            s_boot     <= "01";
          end if;
          if s_color_mode /= s_cm_snap then
            s_cm_snap <= s_color_mode;
            s_boot    <= "01";
          end if;
        end if;
      end if;

      -- T1b selects the completed cell average when this flag is set; the
      -- accumulator registered this edge holds the full cell sum then.
      if v_at_cell_end and s_cell_y = 0 and data_in.avid = '1' then
        s_use_avg <= '1';
      else
        s_use_avg <= '0';
      end if;

      -- Buffer write enable + address (pipelined to match T3 timing).
      -- Not gated on pixelate: in per-pixel mode every pixel is a cell end,
      -- and the hysteresis grid wants those writes too (the display buffer
      -- write is simply unused then).
      if v_at_cell_end and s_cell_y = 0 and data_in.avid = '1' then
        s_we_d1   <= '1';
        s_wcol_d1 <= s_cell_col;
        -- 16x16-px tile of the ENDING cell's origin pixel
        v_origin  := s_px - resize(s_h_size, 11) + 1;
        s_trow_d1 <= s_act_row(10 downto 4);
        s_tcol_d1 <= v_origin(10 downto 4);
      else
        s_we_d1   <= '0';
        s_wcol_d1 <= (others => '0');
      end if;
    end if;
  end process p_t1;

  -- ==========================================================================
  -- Pipeline T1b: average divide (barrel shift) + match-input select.
  -- Kept off T1: the variable shift after the 14-bit accumulate was on the
  -- per-pixel critical path.
  -- ==========================================================================
  p_t1b : process(clk)
  begin
    if rising_edge(clk) then
      if s_use_avg = '1' then
        s_match_y <= resize(s_acc_y srl to_integer(s_h_shift), 10);
        s_match_u <= resize(s_acc_u srl to_integer(s_h_shift), 10);
        s_match_v <= resize(s_acc_v srl to_integer(s_h_shift), 10);
      else
        s_match_y <= unsigned(s_yuv_sr(0).y);
        s_match_u <= unsigned(s_yuv_sr(0).u);
        s_match_v <= unsigned(s_yuv_sr(0).v);
      end if;

      -- Grid address of the cell being matched: row * 120 + col, computed
      -- as (row<<7) - (row<<3) + col in 14 bits (shift_left keeps operand
      -- width, so the intermediate must be wider than the result).
      s_grid_raddr <= resize(
          shift_left(resize(s_trow_d1, 14), 7)
        - shift_left(resize(s_trow_d1, 14), 3)
        + resize(s_tcol_d1, 14), 13);

      s_we_d2   <= s_we_d1;
      s_wcol_d2 <= s_wcol_d1;
    end if;
  end process p_t1b;

  -- ==========================================================================
  -- Pipeline T2a: contrast (7-bit coefficient, clamped result).
  -- ==========================================================================
  p_t2a : process(clk)
    variable v_g    : unsigned(6 downto 0);
    variable v_ce   : unsigned(9 downto 0);
    variable v_sum  : unsigned(10 downto 0);
  begin
    if rising_edge(clk) then
      if s_contrast < 512 then
        v_g    := s_contrast(8 downto 2);
        s_ycon <= resize((s_match_y * v_g) srl 7, 10);
      else
        v_ce  := s_contrast - 512;
        v_g   := v_ce(8 downto 2);
        v_sum := resize(s_match_y, 11) + resize((s_match_y * v_g) srl 7, 11);
        if v_sum > 1023 then
          s_ycon <= to_unsigned(1023, 10);
        else
          s_ycon <= resize(v_sum, 10);
        end if;
      end if;

      s_mu_d1 <= s_match_u;
      s_mv_d1 <= s_match_v;

      s_we_d3    <= s_we_d2;
      s_wcol_d3  <= s_wcol_d2;
      s_gaddr_d3 <= s_grid_raddr;
    end if;
  end process p_t2a;

  -- ==========================================================================
  -- Pipeline T2b: compute 4 palette distances + luma-zone index.
  -- ==========================================================================
  p_t2b : process(clk)
  begin
    if rising_edge(clk) then
      for i in 0 to 3 loop
        s_dist(i) <= f_dist(s_ycon, s_mu_d1, s_mv_d1,
                            C_CGA_PAL(to_integer(s_pal_sel))(i).y,
                            C_CGA_PAL(to_integer(s_pal_sel))(i).u,
                            C_CGA_PAL(to_integer(s_pal_sel))(i).v);
      end loop;

      -- Luma-zone mapping: top 2 bits of contrast-adjusted Y pick the
      -- palette entry whose luma rank matches.
      s_luma_idx <= C_LUMA_MAP(to_integer(s_pal_sel))
                              (to_integer(s_ycon(9 downto 8)));

      -- Last field's palette index for this tile (bank mux after register)
      if s_fparity = '0' then
        s_grid_prev <= s_grid_rd_a;
      else
        s_grid_prev <= s_grid_rd_b;
      end if;

      s_ycon_d1 <= s_ycon;

      s_we_d4    <= s_we_d3;
      s_wcol_d4  <= s_wcol_d3;
      s_gaddr_d4 <= s_gaddr_d3;
    end if;
  end process p_t2b;

  -- ==========================================================================
  -- Pipeline T3: 4-way minimum -> palette index. Register buffer write.
  -- ==========================================================================
  p_t3a : process(clk)
    variable v_chal : unsigned(1 downto 0);
    variable v_min  : unsigned(6 downto 0);
    variable v_pz   : unsigned(1 downto 0);
    variable v_lo, v_hi, v_dy : unsigned(9 downto 0);
  begin
    if rising_edge(clk) then
      -- Challenger: this frame's plain match
      v_min := s_dist(0); v_chal := "00";
      if s_dist(1) < v_min then v_min := s_dist(1); v_chal := "01"; end if;
      if s_dist(2) < v_min then v_min := s_dist(2); v_chal := "10"; end if;
      if s_dist(3) < v_min then v_min := s_dist(3); v_chal := "11"; end if;
      if s_color_mode = '1' then
        -- Luma mode: hysteresis is judged in LUMA-ZONE terms, not colour
        -- distance (which could lock a cell against a legitimate zone
        -- change).  Challenger distance 0 (its zone contains y); incumbent
        -- distance = how far y sits OUTSIDE the stored entry's zone, in the
        -- same 32-code units the Manhattan distances use, so T3b's margin
        -- and debounce apply unchanged.  C_LUMA_MAP is its own inverse
        -- (identity or middle-swap per palette), so it also maps the stored
        -- entry index back to its zone.
        v_pz := C_LUMA_MAP(to_integer(s_pal_sel))
                          (to_integer(s_grid_prev(1 downto 0)));
        v_lo := v_pz & "00000000";
        v_hi := v_lo + 255;
        if s_ycon_d1 < v_lo then
          v_dy := v_lo - s_ycon_d1;
        elsif s_ycon_d1 > v_hi then
          v_dy := s_ycon_d1 - v_hi;
        else
          v_dy := (others => '0');
        end if;
        s_chal      <= s_luma_idx;
        s_chal_dist <= (others => '0');
        s_prev_dist <= resize(v_dy(9 downto 5), 7);
      else
        s_chal      <= v_chal;
        s_chal_dist <= v_min;
        s_prev_dist <= s_dist(to_integer(s_grid_prev(1 downto 0)));
      end if;
      s_prev_d5   <= s_grid_prev(1 downto 0);
      s_pend_d5   <= s_grid_prev(3 downto 2);

      s_we_d5    <= s_we_d4;
      s_wcol_d5  <= s_wcol_d4;
      s_gaddr_d5 <= s_gaddr_d4;
    end if;
  end process p_t3a;

  -- ==========================================================================
  -- Pipeline T3b: temporal hysteresis decision.  Keep last field's colour
  -- for this tile unless the challenger beats it by the Stability margin.
  -- Cannot oscillate on a static input (the incumbent only ever loses to a
  -- strictly better match), and a garbage incumbent loses immediately.
  -- Luma-zone mode bypasses it: judging the luma pick by full colour
  -- distance could lock a cell against a legitimate zone change.
  -- (Split from T3a: min-tree -> margin add/compare in one clock was the
  -- HD critical path.)
  -- ==========================================================================
  p_t3b : process(clk)
    variable v_idx, v_cur, v_pend : unsigned(1 downto 0);
  begin
    if rising_edge(clk) then
      if s_margin = "000" or s_boot /= "11" then
        -- Stability Off / bootstrap: plain match (luma mode included --
        -- its challenger/distances are set in luma-zone terms in T3a)
        v_idx := s_chal; v_cur := s_chal; v_pend := s_chal;
      elsif s_margin = "111" then
        -- Freeze: hold the stored colour unconditionally
        v_idx := s_prev_d5; v_cur := s_prev_d5; v_pend := s_pend_d5;
      elsif resize(s_chal_dist, 8) + s_margin + 4 < resize(s_prev_dist, 8) then
        -- Decisively better: switch immediately (keeps solid regions
        -- rendering correctly even where several cells share a tile)
        v_idx := s_chal; v_cur := s_chal; v_pend := s_chal;
      elsif resize(s_chal_dist, 8) + s_margin < resize(s_prev_dist, 8) then
        -- Moderately better: DEBOUNCE -- adopt only if the same challenger
        -- also won last field.  A cell straddling a colour edge alternates
        -- challengers field to field, never confirms, and holds still --
        -- this is the defence edge jitter needs that no margin can give.
        if s_chal = s_pend_d5 then
          v_idx := s_chal;    v_cur := s_chal;    v_pend := s_chal;
        else
          v_idx := s_prev_d5; v_cur := s_prev_d5; v_pend := s_chal;
        end if;
      else
        -- Not better by margin: hold and clear any pending challenger
        v_idx := s_prev_d5; v_cur := s_prev_d5; v_pend := s_prev_d5;
      end if;
      s_match_idx <= v_idx;

      s_buf_we    <= s_we_d5;
      s_buf_waddr <= s_wcol_d5;
      s_buf_wdata <= v_idx;

      s_grid_we    <= s_we_d5;
      s_grid_waddr <= s_gaddr_d5;
      s_grid_wdata <= v_pend & v_cur;
    end if;
  end process p_t3b;

  -- ==========================================================================
  -- Per-cell-column palette-index BRAM
  -- ==========================================================================
  p_buf : process(clk)
  begin
    if rising_edge(clk) then
      if s_buf_we = '1' then
        s_pidx_buf(to_integer(s_buf_waddr)) <= s_buf_wdata;
      end if;
      s_buf_rdata <= s_pidx_buf(to_integer(s_buf_raddr));
    end if;
  end process p_buf;

  s_buf_raddr <= s_cell_col;

  -- ==========================================================================
  -- Hysteresis grid banks.  Read bank = the one written LAST field, so read
  -- and write never touch the same BRAM (no read-during-write collision).
  -- Each bank: unconditional read + enabled write, 1W1R -> clean EBR inference.
  -- ==========================================================================
  p_grid_a : process(clk)
  begin
    if rising_edge(clk) then
      if s_grid_we = '1' and s_fparity = '1' then
        s_grid_a(to_integer(s_grid_waddr)) <= s_grid_wdata;
      end if;
      s_grid_rd_a <= s_grid_a(to_integer(s_grid_raddr));
    end if;
  end process p_grid_a;

  p_grid_b : process(clk)
  begin
    if rising_edge(clk) then
      if s_grid_we = '1' and s_fparity = '0' then
        s_grid_b(to_integer(s_grid_waddr)) <= s_grid_wdata;
      end if;
      s_grid_rd_b <= s_grid_b(to_integer(s_grid_raddr));
    end if;
  end process p_grid_b;

  -- ==========================================================================
  -- Pipeline T4a: display palette-entry lookup (single muxed ROM read).
  -- In pixelate mode the index comes from the per-column buffer; otherwise
  -- from the per-pixel match (which lags the buffered path by 3 clocks --
  -- a 3-pixel shift in Off mode, invisible at full resolution).
  -- ==========================================================================
  p_t4a : process(clk)
    variable v_idx : unsigned(1 downto 0);
  begin
    if rising_edge(clk) then
      if s_pixelate = '1' then
        v_idx := s_buf_rdata;
      else
        v_idx := s_match_idx;
      end if;
      s_disp_y <= C_CGA_PAL(to_integer(s_pal_sel))(to_integer(v_idx)).y;
      s_disp_u <= C_CGA_PAL(to_integer(s_pal_sel))(to_integer(v_idx)).u;
      s_disp_v <= C_CGA_PAL(to_integer(s_pal_sel))(to_integer(v_idx)).v;
    end if;
  end process p_t4a;

  -- ==========================================================================
  -- Pipeline T4b: start saturation scaling (subtract 512, then 12x9 signed
  -- multiply, 8-bit coefficient).  The slice/add/clamp is deferred to T5.
  -- ==========================================================================
  p_t4b : process(clk)
    variable v_su, v_sv : signed(11 downto 0);
    variable v_sat      : signed(8 downto 0);
  begin
    if rising_edge(clk) then
      v_su  := signed(resize(s_disp_u, 12)) - to_signed(512, 12);
      v_sv  := signed(resize(s_disp_v, 12)) - to_signed(512, 12);
      v_sat := signed('0' & s_saturation(9 downto 2));  -- 128 = unity
      s_t4_su_prod <= v_su * v_sat;
      s_t4_sv_prod <= v_sv * v_sat;
      s_t4_y       <= s_disp_y;
    end if;
  end process p_t4b;

  -- ==========================================================================
  -- Pipeline T5: finish saturation (slice/add/clamp), apply scanlines, output.
  -- ==========================================================================
  p_t5 : process(clk)
    variable v_su_out, v_sv_out : signed(11 downto 0);
    variable v_final_y : unsigned(9 downto 0);
    variable v_u, v_v : unsigned(9 downto 0);
    variable v_cu, v_cv : signed(10 downto 0);
  begin
    if rising_edge(clk) then
      v_su_out := s_t4_su_prod(18 downto 7) + to_signed(512, 12);
      v_sv_out := s_t4_sv_prod(18 downto 7) + to_signed(512, 12);

      if    v_su_out < 0    then v_u := (others => '0');
      elsif v_su_out > 1023 then v_u := to_unsigned(1023, 10);
      else                       v_u := unsigned(v_su_out(9 downto 0));
      end if;
      if    v_sv_out < 0    then v_v := (others => '0');
      elsif v_sv_out > 1023 then v_v := to_unsigned(1023, 10);
      else                       v_v := unsigned(v_sv_out(9 downto 0));
      end if;

      -- Scanlines: 25% dim (was a harsh 50%), and chroma excursion tracks
      -- the same factor so dark lines keep the same hue/saturation balance
      -- instead of reading as oversaturated stripes -- a more even texture.
      v_final_y := s_t4_y;
      if s_scanlines = '1' and s_pixel_y(0) = '1' then
        v_final_y := v_final_y - (v_final_y srl 2);
        v_cu := signed(resize(v_u, 11)) - to_signed(512, 11);
        v_cu := v_cu - shift_right(v_cu, 2) + to_signed(512, 11);
        v_u  := unsigned(v_cu(9 downto 0));
        v_cv := signed(resize(v_v, 11)) - to_signed(512, 11);
        v_cv := v_cv - shift_right(v_cv, 2) + to_signed(512, 11);
        v_v  := unsigned(v_cv(9 downto 0));
      end if;

      s_out_y <= v_final_y;
      s_out_u <= v_u;
      s_out_v <= v_v;
    end if;
  end process p_t5;

  -- ==========================================================================
  -- Interpolator (wet/dry mix)
  -- ==========================================================================
  interp_y : entity work.interpolator_u
    generic map(G_WIDTH => 10, G_FRAC_BITS => 8,
                G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map(clk => clk, enable => '1',
             a => unsigned(s_yuv_sr(5).y), b => s_out_y, t => s_mix_t,
             result => s_interp_y_result, valid => open);

  interp_u : entity work.interpolator_u
    generic map(G_WIDTH => 10, G_FRAC_BITS => 8,
                G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map(clk => clk, enable => '1',
             a => unsigned(s_yuv_sr(5).u), b => s_out_u, t => s_mix_t,
             result => s_interp_u_result, valid => open);

  interp_v : entity work.interpolator_u
    generic map(G_WIDTH => 10, G_FRAC_BITS => 8,
                G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map(clk => clk, enable => '1',
             a => unsigned(s_yuv_sr(5).v), b => s_out_v, t => s_mix_t,
             result => s_interp_v_result, valid => open);

  -- ==========================================================================
  -- Output
  -- ==========================================================================
  data_out.y <= s_yuv_sr(C_LATENCY - 1).y when s_bypass = '1'
                else std_logic_vector(s_interp_y_result);
  data_out.u <= s_yuv_sr(C_LATENCY - 1).u when s_bypass = '1'
                else std_logic_vector(s_interp_u_result);
  data_out.v <= s_yuv_sr(C_LATENCY - 1).v when s_bypass = '1'
                else std_logic_vector(s_interp_v_result);

  data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
  data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
  data_out.field_n <= s_field_sr(C_LATENCY - 1);
  data_out.avid    <= s_avid_sr(C_LATENCY - 1);

end architecture cga;
