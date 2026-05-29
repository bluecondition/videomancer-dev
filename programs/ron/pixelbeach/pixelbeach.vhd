-- Pixel Beach v2: Chunky-pixel ocean with trochoidal-ish waves
-- ---------------------------------------------------------------
-- 8x8 cells, never finer. Three z-stacked wave layers (back -> mid -> front).
-- Front layer can occlude back-layer crests.
-- Wave body uses a 2nd-harmonic-sharpened cosine: sharp narrow crests,
-- broader flatter troughs (Stokes-wave approximation -> "rolling" look).
-- Foam crest band sits on the top 1-2 cells of each wave; foam intensity
-- follows an Attack-Sustain-Release envelope driven by the wave's phase
-- so that foam pops white at the crest, holds, then fades in the wake.
-- Sky is a single solid color picked from an 8-entry palette by knob 5.
--
-- License: GPL-3.0
-- Author: ron

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture pixelbeach of program_top is

  constant C_LATENCY    : integer := 9;  -- S1, S2, S3a, S3b, S3c, S4, S5, S6, S7
  constant C_NUM_LAYERS : integer := 3;
  constant C_CELL_BITS_8  : integer := 3;  -- 8x8 chunky cells
  constant C_CELL_BITS_16 : integer := 4;  -- 16x16 chunky cells

  -- 64-entry sine LUT (signed 8-bit, range -127..127)
  type t_sin_lut is array(0 to 63) of signed(7 downto 0);
  constant C_SIN : t_sin_lut := (
     0 => to_signed(   0, 8),  1 => to_signed(  12, 8),
     2 => to_signed(  25, 8),  3 => to_signed(  37, 8),
     4 => to_signed(  49, 8),  5 => to_signed(  60, 8),
     6 => to_signed(  71, 8),  7 => to_signed(  81, 8),
     8 => to_signed(  90, 8),  9 => to_signed(  98, 8),
    10 => to_signed( 106, 8), 11 => to_signed( 112, 8),
    12 => to_signed( 117, 8), 13 => to_signed( 121, 8),
    14 => to_signed( 124, 8), 15 => to_signed( 126, 8),
    16 => to_signed( 127, 8), 17 => to_signed( 126, 8),
    18 => to_signed( 124, 8), 19 => to_signed( 121, 8),
    20 => to_signed( 117, 8), 21 => to_signed( 112, 8),
    22 => to_signed( 106, 8), 23 => to_signed(  98, 8),
    24 => to_signed(  90, 8), 25 => to_signed(  81, 8),
    26 => to_signed(  71, 8), 27 => to_signed(  60, 8),
    28 => to_signed(  49, 8), 29 => to_signed(  37, 8),
    30 => to_signed(  25, 8), 31 => to_signed(  12, 8),
    32 => to_signed(   0, 8), 33 => to_signed( -12, 8),
    34 => to_signed( -25, 8), 35 => to_signed( -37, 8),
    36 => to_signed( -49, 8), 37 => to_signed( -60, 8),
    38 => to_signed( -71, 8), 39 => to_signed( -81, 8),
    40 => to_signed( -90, 8), 41 => to_signed( -98, 8),
    42 => to_signed(-106, 8), 43 => to_signed(-112, 8),
    44 => to_signed(-117, 8), 45 => to_signed(-121, 8),
    46 => to_signed(-124, 8), 47 => to_signed(-126, 8),
    48 => to_signed(-127, 8), 49 => to_signed(-126, 8),
    50 => to_signed(-124, 8), 51 => to_signed(-121, 8),
    52 => to_signed(-117, 8), 53 => to_signed(-112, 8),
    54 => to_signed(-106, 8), 55 => to_signed( -98, 8),
    56 => to_signed( -90, 8), 57 => to_signed( -81, 8),
    58 => to_signed( -71, 8), 59 => to_signed( -60, 8),
    60 => to_signed( -49, 8), 61 => to_signed( -37, 8),
    62 => to_signed( -25, 8), 63 => to_signed( -12, 8)
  );

  -- Per-layer constants:
  --   offset_y    - pixels below horizon (baseline y)
  --   amp         - crest amplitude in pixels
  --   wshift      - wavelength = 64 << wshift
  --   phase_off   - per-layer phase offset (visual desync)
  --   tide_amp    - per-layer parallax distance the wave field travels
  --                 during one full crash/recede cycle (pixels)
  --   tide_off    - per-layer tide phase offset (layers crash off-cycle)
  --   h2_shift    - 2nd-harmonic weight (cos2 sra h2_shift); smaller = sharper
  --   warble_step - per-frame warble-LFO phase advance (relative primes
  --                 between layers -> phase relationships never align)
  type t_layer is record
    offset_y    : unsigned(8 downto 0);
    amp         : unsigned(5 downto 0);
    wshift      : unsigned(2 downto 0);
    phase_off   : unsigned(5 downto 0);
    tide_amp    : unsigned(8 downto 0);
    tide_off    : unsigned(5 downto 0);
    h2_shift    : unsigned(2 downto 0);
    warble_step : unsigned(7 downto 0);
  end record;
  type t_layer_array is array(0 to C_NUM_LAYERS - 1) of t_layer;
  constant C_LAYER : t_layer_array := (
    -- Back: small, frequent. Tide moves it the least (parallax distance).
    0 => (offset_y    => to_unsigned( 32, 9),
          amp         => to_unsigned( 14, 6),
          wshift      => to_unsigned(  2, 3),  -- 256-pixel wavelength
          phase_off   => to_unsigned(  0, 6),
          tide_amp    => to_unsigned( 60, 9),
          tide_off    => to_unsigned(  0, 6),
          h2_shift    => to_unsigned(  3, 3),  -- gentle 2nd harmonic
          warble_step => to_unsigned( 53, 8)),
    -- Mid: medium.
    1 => (offset_y    => to_unsigned( 96, 9),
          amp         => to_unsigned( 24, 6),
          wshift      => to_unsigned(  3, 3),  -- 512-pixel wavelength
          phase_off   => to_unsigned( 21, 6),
          tide_amp    => to_unsigned(140, 9),
          tide_off    => to_unsigned(  5, 6),
          h2_shift    => to_unsigned(  2, 3),  -- moderate
          warble_step => to_unsigned( 71, 8)),
    -- Front: big rolling waves. Travels furthest each tide cycle.
    2 => (offset_y    => to_unsigned(184, 9),
          amp         => to_unsigned( 40, 6),
          wshift      => to_unsigned(  3, 3),  -- 512-pixel wavelength
          phase_off   => to_unsigned( 43, 6),
          tide_amp    => to_unsigned(260, 9),
          tide_off    => to_unsigned( 11, 6),
          h2_shift    => to_unsigned(  1, 3),  -- sharpest crests (max breaking)
          warble_step => to_unsigned( 83, 8))
  );

  type t_yuv is record
    y : unsigned(9 downto 0);
    u : unsigned(9 downto 0);
    v : unsigned(9 downto 0);
  end record;

  -- Palette constants -- U/V swapped for Videomancer hardware (SDK convention).
  -- In these constants the field named "u" holds BT.601 Cr, and "v" holds Cb.
  -- For aqua: u<512 (low Cr -> cyan-green) and v>512 (high Cb -> blue).
  type t_ocean_palette is array(0 to C_NUM_LAYERS - 1) of t_yuv;
  constant C_OCEAN : t_ocean_palette := (
    -- Back: deep teal-green
    0 => (y => to_unsigned(240, 10), u => to_unsigned(420, 10), v => to_unsigned(548, 10)),
    -- Mid: medium aqua-blue
    1 => (y => to_unsigned(400, 10), u => to_unsigned(440, 10), v => to_unsigned(584, 10)),
    -- Front: light aqua
    2 => (y => to_unsigned(560, 10), u => to_unsigned(458, 10), v => to_unsigned(612, 10))
  );

  -- Foam release-band tint (light aqua, between ocean and white)
  constant C_CREST_MID  : t_yuv := (y => to_unsigned(780, 10), u => to_unsigned(484, 10), v => to_unsigned(556, 10));
  constant C_CREST_HIGH : t_yuv := (y => to_unsigned(880, 10), u => to_unsigned(498, 10), v => to_unsigned(534, 10));
  constant C_FOAM_WHITE : t_yuv := (y => to_unsigned(948, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10));

  -- Sky palette (8 presets, picked by upper bits of knob 5)
  type t_sky_palette is array(0 to 7) of t_yuv;
  constant C_SKY : t_sky_palette := (
    0 => (y => to_unsigned(640, 10), u => to_unsigned(488, 10), v => to_unsigned(560, 10)),  -- light blue (default)
    1 => (y => to_unsigned(740, 10), u => to_unsigned(500, 10), v => to_unsigned(540, 10)),  -- pale blue
    2 => (y => to_unsigned(820, 10), u => to_unsigned(508, 10), v => to_unsigned(522, 10)),  -- almost-white sky
    3 => (y => to_unsigned(440, 10), u => to_unsigned(470, 10), v => to_unsigned(584, 10)),  -- deeper blue
    4 => (y => to_unsigned(280, 10), u => to_unsigned(516, 10), v => to_unsigned(548, 10)),  -- twilight
    5 => (y => to_unsigned(540, 10), u => to_unsigned(612, 10), v => to_unsigned(456, 10)),  -- sunset orange
    6 => (y => to_unsigned(660, 10), u => to_unsigned(572, 10), v => to_unsigned(484, 10)),  -- warm pink
    7 => (y => to_unsigned(140, 10), u => to_unsigned(496, 10), v => to_unsigned(534, 10))   -- night
  );

  -- =====================================================================
  -- Signals
  -- =====================================================================
  type t_pipe is array(0 to C_LATENCY - 1) of t_video_stream_yuv444_30b;
  signal pipe : t_pipe;

  signal s_pixel_x      : unsigned(11 downto 0) := (others => '0');
  signal s_pixel_y      : unsigned(11 downto 0) := (others => '0');
  signal s_prev_hsync_n : std_logic := '1';
  signal s_prev_vsync_n : std_logic := '1';
  signal s_frame_count  : unsigned(15 downto 0) := (others => '0');

  -- Tide state (advances on vsync, drives all layer scroll positions)
  signal s_tide_phase : unsigned(23 downto 0) := (others => '0');
  -- Per-layer pixel-scroll position, signed (positive = pattern shifted left)
  type t_scroll_arr is array(0 to C_NUM_LAYERS - 1) of signed(15 downto 0);
  signal s_scroll : t_scroll_arr := (others => (others => '0'));
  -- Tide velocity sign: '1' = crashing inward, '0' = receding outward
  signal s_tide_inward : std_logic := '1';
  -- Tide velocity signed magnitude (cos of tide_phase): drives amplitude
  -- modulation and smooth foam transition.
  signal s_tide_vel : signed(7 downto 0) := (others => '0');
  -- Tide position (sin of tide_phase): drives amplitude swelling.
  signal s_tide_pos : signed(7 downto 0) := (others => '0');

  -- Per-frame foam parameters (precomputed at vsync from tide_vel so S4
  -- only does reads -- keeps the per-pixel hot path lean).
  signal s_foam_step : unsigned(5 downto 0) := (others => '0');
  signal s_foam_band_thick : std_logic := '0';  -- '1' = thicker (crashing)

  -- Per-frame per-layer amplitude * height_knob, MODULATED by tide_pos.
  -- During crash (tide_pos > 0) amps grow ~+25%; during recede they shrink
  -- ~-25%. Precomputed in S1 vsync so it's free on the per-pixel path.
  type t_amp_x_knob_arr is array(0 to C_NUM_LAYERS - 1) of unsigned(12 downto 0);
  signal s_amp_x_knob : t_amp_x_knob_arr := (others => (others => '0'));

  -- Slow modulator: a per-cell-column sine wave whose phase advances
  -- slowly each frame. Provides organic baseline-shift across the X axis
  -- that drifts continuously over time. Always ON (no choppy gate);
  -- amplitude scales with Chaos. Pipelined through S2..S3c.
  signal s_slow_phase : unsigned(15 downto 0) := (others => '0');
  signal s_col_slow   : signed(7 downto 0)    := (others => '0');
  signal s1_col_slow  : signed(7 downto 0)    := (others => '0');
  signal s2_col_slow  : signed(7 downto 0)    := (others => '0');
  -- Combined noise+slow offset (computed in S3a so S3c has a shorter
  -- adder chain). 8b+8b => 9b signed range +/-254.
  signal s3a_col_off  : signed(8 downto 0) := (others => '0');
  signal s3b_col_off  : signed(8 downto 0) := (others => '0');
  signal s3c_col_off  : signed(8 downto 0) := (others => '0');

  -- Per-layer warble LFO (independent rates -> layers never resync)
  type t_warble_arr is array(0 to C_NUM_LAYERS - 1) of unsigned(15 downto 0);
  signal s_warble_phase : t_warble_arr := (others => (others => '0'));
  type t_warble_val_arr is array(0 to C_NUM_LAYERS - 1) of signed(7 downto 0);
  signal s_warble_val : t_warble_val_arr := (others => (others => '0'));

  -- LFSR-driven per-column noise (gated by Sea State = Choppy).
  -- s_col_noise is PRE-SCALED at the column boundary so the per-pixel hot
  -- path is just an add. Pipelined to stay aligned with wave_top.
  signal s_lfsr_out    : std_logic_vector(9 downto 0);
  signal s_lfsr_reset  : std_logic := '1';  -- seeded at first vsync
  signal s_lfsr_seed   : std_logic_vector(9 downto 0);
  signal s_col_noise   : signed(7 downto 0) := (others => '0');  -- pre-scaled
  signal s1_col_noise  : signed(7 downto 0) := (others => '0');
  signal s2_col_noise  : signed(7 downto 0) := (others => '0');

  -- Per-frame, per-layer pre-scaled warble adjust (computed at vsync).
  type t_warble_adj_arr is array(0 to C_NUM_LAYERS - 1) of signed(7 downto 0);
  signal s_warble_adj : t_warble_adj_arr := (others => (others => '0'));

  -- Per-frame, per-layer constant phase offset = phase_off + warble_adj.
  -- Computed once per vsync so the per-pixel hot path becomes a single add.
  type t_phase_const_arr is array(0 to C_NUM_LAYERS - 1) of unsigned(5 downto 0);
  signal s_phase_const : t_phase_const_arr := (others => (others => '0'));

  -- Per-layer phase total = phase_const + per-column LFSR phase jitter.
  -- Updated at every cell-column edge (inline-forwarded). Per-pixel just
  -- adds raw_phase + s_phase_total(li) -> single 6-bit add per layer.
  signal s_phase_total : t_phase_const_arr := (others => (others => '0'));

  -- ---- S1: cell-snapped coordinates, raw phase per layer ----
  type t_phase_arr is array(0 to C_NUM_LAYERS - 1) of unsigned(5 downto 0);
  signal s1_phase   : t_phase_arr := (others => (others => '0'));
  signal s1_cell_x  : unsigned(8 downto 0) := (others => '0');
  signal s1_cell_y  : unsigned(8 downto 0) := (others => '0');

  -- ---- S2: cos1 and cos2 LUT outputs per layer ----
  type t_sin_arr is array(0 to C_NUM_LAYERS - 1) of signed(7 downto 0);
  signal s2_cos1    : t_sin_arr := (others => (others => '0'));
  signal s2_cos2    : t_sin_arr := (others => (others => '0'));
  signal s2_phase   : t_phase_arr := (others => (others => '0'));
  signal s2_cell_x  : unsigned(8 downto 0) := (others => '0');
  signal s2_cell_y  : unsigned(8 downto 0) := (others => '0');

  -- ---- S3a: amplitude*knob, combined cos, baseline y (one cycle of math) ----
  type t_amp_x_arr is array(0 to C_NUM_LAYERS - 1) of unsigned(12 downto 0);
  type t_comb_arr  is array(0 to C_NUM_LAYERS - 1) of signed(9 downto 0);
  type t_base_arr  is array(0 to C_NUM_LAYERS - 1) of signed(12 downto 0);
  signal s3a_amp_x_knob : t_amp_x_arr := (others => (others => '0'));
  signal s3a_combined   : t_comb_arr  := (others => (others => '0'));
  signal s3a_base       : t_base_arr  := (others => (others => '0'));
  signal s3a_phase      : t_phase_arr := (others => (others => '0'));
  signal s3a_cell_x     : unsigned(8 downto 0) := (others => '0');
  signal s3a_cell_y     : unsigned(8 downto 0) := (others => '0');

  -- ---- S3b: raw products (multiplier output stage) ----
  type t_prod_arr is array(0 to C_NUM_LAYERS - 1) of signed(23 downto 0);
  signal s3b_prod   : t_prod_arr  := (others => (others => '0'));
  signal s3b_base   : t_base_arr  := (others => (others => '0'));
  signal s3b_phase  : t_phase_arr := (others => (others => '0'));
  signal s3b_cell_x : unsigned(8 downto 0) := (others => '0');
  signal s3b_cell_y : unsigned(8 downto 0) := (others => '0');
  -- ---- S3c: wave_top per layer (final pixel-coord wave surface) ----
  type t_wtop_arr is array(0 to C_NUM_LAYERS - 1) of signed(12 downto 0);
  signal s3c_wave_top : t_wtop_arr := (others => (others => '0'));
  signal s3c_phase    : t_phase_arr := (others => (others => '0'));
  signal s3c_cell_x   : unsigned(8 downto 0) := (others => '0');
  signal s3c_cell_y   : unsigned(8 downto 0) := (others => '0');

  -- ---- S4: layer ownership + foam status ----
  -- ownership: 0..2 = layer index, 3 = sky
  signal s4_owner    : unsigned(1 downto 0) := "11";
  -- foam level: 0=none, 1=subtle tint, 2=mid crest, 3=full white
  signal s4_foam_lvl : unsigned(1 downto 0) := (others => '0');
  signal s4_cell_x   : unsigned(8 downto 0) := (others => '0');
  signal s4_cell_y   : unsigned(8 downto 0) := (others => '0');

  -- ---- S5: pre-bright color ----
  signal s5_y, s5_u, s5_v : unsigned(9 downto 0) := (others => '0');

  -- ---- S6: post-bright color ----
  signal s6_y, s6_u, s6_v : unsigned(9 downto 0) := (others => '0');

  -- ---- Controls (latched on vsync) ----
  signal s_speed      : unsigned(9 downto 0) := (others => '0');
  signal s_height     : unsigned(9 downto 0) := (others => '0');
  signal s_foam_life  : unsigned(9 downto 0) := (others => '0');
  signal s_drift      : unsigned(9 downto 0) := (others => '0');
  signal s_sky_idx    : unsigned(2 downto 0) := (others => '0');
  signal s_horizon    : unsigned(11 downto 0) := (others => '0');
  signal s_jagged     : std_logic := '0';
  signal s_chunk16    : std_logic := '0';
  signal s_choppy     : std_logic := '0';
  signal s_mono       : std_logic := '0';
  signal s_bypass     : std_logic := '0';
  signal s_chaos      : unsigned(9 downto 0) := (others => '0');

begin

  -- =====================================================================
  -- LFSR for per-column noise. Reseeded once per vsync from frame_count,
  -- otherwise free-runs. Top bit forced to '1' so the seed is never zero.
  -- 10-bit maximal-length polynomial x^10 + x^7 + 1 = "1001000000".
  -- =====================================================================
  s_lfsr_seed <= '1' & std_logic_vector(s_frame_count(8 downto 0));

  u_lfsr : entity work.lfsr
    generic map (G_DATA_WIDTH => 10)
    port map (
      clk      => clk,
      reset    => s_lfsr_reset,
      enable   => '1',
      seed     => s_lfsr_seed,
      poly     => "1001000000",
      lfsr_out => s_lfsr_out
    );

  -- =====================================================================
  -- S1: input tracking, control latch, tide phase accumulator,
  --     per-layer phase computation (cell-snapped x).
  --
  -- Tide replaces linear scroll: position is a slow sinusoid of frame time,
  -- so all layers crash leftward then recede rightward in a continuous cycle.
  -- s_tide_inward holds the SIGN of the tide velocity for foam physics.
  -- =====================================================================
  p_s1 : process(clk)
    variable v_x_snap        : unsigned(11 downto 0);
    variable v_scrolled      : signed(15 downto 0);
    variable v_scrolled_u    : unsigned(15 downto 0);
    variable v_raw_phase     : unsigned(15 downto 0);
    variable v_tide_step     : unsigned(15 downto 0);
    variable v_tide_addr     : unsigned(5 downto 0);
    variable v_tide_pos      : signed(7 downto 0);
    variable v_tide_vel      : signed(7 downto 0);
    variable v_tide_layer    : signed(7 downto 0);
    variable v_scroll_lay    : signed(17 downto 0);
    variable v_addr_layer    : unsigned(5 downto 0);
    variable v_warble_addr   : unsigned(5 downto 0);
    variable v_warble_val    : signed(7 downto 0);
    variable v_phase_combine : unsigned(7 downto 0);
    variable v_at_col_edge   : boolean;
    variable v_raw_noise     : signed(7 downto 0);
    variable v_step_s        : signed(7 downto 0);
    variable v_amp_base      : unsigned(12 downto 0);
    variable v_amp_q         : unsigned(12 downto 0);  -- amp >> 2 (quarter)
    variable v_amp_e         : unsigned(12 downto 0);  -- amp >> 3 (eighth)
    variable v_amp_mod       : unsigned(13 downto 0);
    variable v_slow_addr     : unsigned(5 downto 0);
    variable v_slow_raw      : signed(7 downto 0);
    variable v_col_noise_new : signed(7 downto 0);  -- scratch for inline forward
    variable v_col_slow_new  : signed(7 downto 0);
    variable v_col_phase_slo : signed(5 downto 0);  -- slow phase (drift, smooth)
    variable v_phase_total_new : t_phase_const_arr;
  begin
    if rising_edge(clk) then

      s_prev_hsync_n <= data_in.hsync_n;
      s_prev_vsync_n <= data_in.vsync_n;

      -- Pipe input through for sync/avid alignment AND bypass passthrough.
      pipe(0).avid    <= data_in.avid;
      pipe(0).hsync_n <= data_in.hsync_n;
      pipe(0).vsync_n <= data_in.vsync_n;
      pipe(0).field_n <= data_in.field_n;
      pipe(0).y <= data_in.y;
      pipe(0).u <= data_in.u;
      pipe(0).v <= data_in.v;
      for i in 1 to C_LATENCY - 1 loop
        pipe(i) <= pipe(i - 1);
      end loop;

      -- Pixel position counters
      if data_in.avid = '1' then
        s_pixel_x <= s_pixel_x + 1;
      end if;
      if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
        s_pixel_x <= (others => '0');
        s_pixel_y <= s_pixel_y + 1;
      end if;

      -- LFSR re-seed pulse: high for one cycle each vsync edge.
      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_lfsr_reset <= '1';
      else
        s_lfsr_reset <= '0';
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_pixel_y <= (others => '0');
        s_frame_count <= s_frame_count + 1;

        -- Latch controls. Slider = Chaos (vital function). Knob 4 = Drift.
        s_speed     <= unsigned(registers_in(0));
        s_height    <= unsigned(registers_in(1));
        s_foam_life <= unsigned(registers_in(2));
        s_drift     <= unsigned(registers_in(3));
        s_sky_idx   <= unsigned(registers_in(4)(9 downto 7));
        s_horizon   <= resize(unsigned(registers_in(5)), 12);
        s_jagged    <= registers_in(6)(0);
        s_chunk16   <= registers_in(6)(1);
        s_choppy    <= registers_in(6)(2);
        s_mono      <= registers_in(6)(3);
        s_bypass    <= registers_in(6)(4);
        s_chaos     <= unsigned(registers_in(7));

        -- Tide phase advances per frame, rate set by Wave Speed knob.
        -- Step = knob * 64. At default knob=600 -> step=38400, full 24-bit
        -- cycle ~= 2^24/38400 = ~437 frames = ~7.3 sec at 60fps.
        -- to_unsigned width must hold value 64 (>= 7 bits); 6 bits wraps to 0.
        v_tide_step := resize(unsigned(registers_in(0)) * to_unsigned(64, 7), 16);
        s_tide_phase <= s_tide_phase + resize(v_tide_step, 24);

        -- Tide position from top 6 bits of phase -> 6-bit sin LUT addr
        v_tide_addr := s_tide_phase(23 downto 18);
        v_tide_pos  := C_SIN(to_integer(v_tide_addr));
        -- Velocity = cos(phase) = sin(phase + pi/2) -> addr + 16 mod 64
        v_tide_vel  := C_SIN(to_integer(v_tide_addr + to_unsigned(16, 6)));

        -- Expose tide state to downstream stages.
        s_tide_pos <= v_tide_pos;
        s_tide_vel <= v_tide_vel;
        if v_tide_vel >= 0 then
          s_tide_inward <= '1';
        else
          s_tide_inward <= '0';
        end if;

        -- Precompute foam parameters from tide_vel for smooth transition.
        -- foam_step = base + foam_life_knob + (tide_vel sra 4), clamped >= 1.
        v_step_s := to_signed(8, 8)
                    + signed('0' & std_logic_vector(resize(unsigned(registers_in(2)(9 downto 7)), 7)))
                    + resize(shift_right(v_tide_vel, 4), 8);
        if v_step_s < to_signed(1, 8) then
          s_foam_step <= to_unsigned(1, 6);
        elsif v_step_s > to_signed(31, 8) then
          s_foam_step <= to_unsigned(31, 6);
        else
          s_foam_step <= unsigned(std_logic_vector(v_step_s(5 downto 0)));
        end if;

        -- Band-thickness toggle: strong inward = thicker. Dead-zone near 0.
        if v_tide_vel > to_signed(32, 8) then
          s_foam_band_thick <= '1';
        else
          s_foam_band_thick <= '0';
        end if;

        -- Advance the slow modulator phase. Rate is set by Drift (knob 4):
        -- step = knob >> 3 -> 0..127 per frame -> 8..inf sec/cycle at 60fps.
        s_slow_phase <= s_slow_phase + resize(unsigned(registers_in(3)(9 downto 3)), 16);

        -- Per-layer scroll position: pos = tide_amp * sin(phase + layer_off) / 128
        -- tide_amp (max 260, 9-bit) * tide_layer (+/-127, 8-bit) -> 17-bit signed
        for li in 0 to C_NUM_LAYERS - 1 loop
          v_addr_layer := v_tide_addr + C_LAYER(li).tide_off;
          v_tide_layer := C_SIN(to_integer(v_addr_layer));
          v_scroll_lay := signed('0' & std_logic_vector(C_LAYER(li).tide_amp))
                          * v_tide_layer;
          -- Divide by 128 (>>7) -> +/-260 pixels of scroll
          s_scroll(li) <= resize(shift_right(v_scroll_lay, 7), 16);

          -- Advance per-layer warble accumulator; sample sin LUT at current phase.
          v_warble_addr      := s_warble_phase(li)(15 downto 10);
          v_warble_val       := C_SIN(to_integer(v_warble_addr));
          s_warble_val(li)   <= v_warble_val;
          s_warble_phase(li) <= s_warble_phase(li) + resize(C_LAYER(li).warble_step, 16);

          -- Pre-scale warble by Chaos (slider). Sea State switch no longer
          -- gates this -- Chaos slider is the master and works always.
          case registers_in(7)(9 downto 8) is
            when "00"   => v_warble_val := (others => '0');
            when "01"   => v_warble_val := shift_right(v_warble_val, 6);
            when "10"   => v_warble_val := shift_right(v_warble_val, 5);
            when others => v_warble_val := shift_right(v_warble_val, 4);
          end case;
          s_warble_adj(li)  <= v_warble_val;
          s_phase_const(li) <= C_LAYER(li).phase_off
                               + unsigned(std_logic_vector(v_warble_val(5 downto 0)));

          -- amp_x_knob (per-frame, per-layer) with tide-driven modulation,
          -- multiplier-free: shift+add gives 5 amplitude levels keyed to
          -- tide_pos top 3 bits. Bigger amps during crash, smaller during
          -- recede. ~+/-25% peak swing.
          v_amp_base := C_LAYER(li).amp * unsigned(registers_in(1)(9 downto 3));
          v_amp_q    := "00" & v_amp_base(12 downto 2);  -- amp >> 2
          v_amp_e    := "000" & v_amp_base(12 downto 3); -- amp >> 3

          case v_tide_pos(7 downto 5) is
            when "010" | "011" => v_amp_mod := resize(v_amp_base, 14) + resize(v_amp_q, 14);  -- +25%
            when "001"          => v_amp_mod := resize(v_amp_base, 14) + resize(v_amp_e, 14); -- +12%
            when "000"          => v_amp_mod := resize(v_amp_base, 14);                       --   0
            when "111"          => v_amp_mod := resize(v_amp_base, 14) - resize(v_amp_e, 14); -- -12%
            when others          => v_amp_mod := resize(v_amp_base, 14) - resize(v_amp_q, 14);-- -25%
          end case;

          if v_amp_mod(13) = '1' then  -- underflow
            s_amp_x_knob(li) <= (others => '0');
          else
            s_amp_x_knob(li) <= v_amp_mod(12 downto 0);
          end if;
        end loop;
      end if;

      -- Cell-size mux: 8x8 vs 16x16. Snap x to cell column, detect col edge.
      if s_chunk16 = '1' then
        v_x_snap  := s_pixel_x(11 downto C_CELL_BITS_16) & to_unsigned(0, C_CELL_BITS_16);
        s1_cell_x <= resize(s_pixel_x(11 downto C_CELL_BITS_16), 9);
        s1_cell_y <= resize(s_pixel_y(11 downto C_CELL_BITS_16), 9);
        v_at_col_edge := (s_pixel_x(3 downto 0) = "0000");
      else
        v_x_snap  := s_pixel_x(11 downto C_CELL_BITS_8) & to_unsigned(0, C_CELL_BITS_8);
        s1_cell_x <= s_pixel_x(11 downto C_CELL_BITS_8);
        s1_cell_y <= s_pixel_y(11 downto C_CELL_BITS_8);
        v_at_col_edge := (s_pixel_x(2 downto 0) = "000");
      end if;

      -- Sample LFSR & slow modulator once per cell column. To keep the
      -- pipeline ALIGNED with the new column, compute new values into vars
      -- and assign BOTH the signals AND the pipe forwards from them.
      --
      -- Chaos slider scales LFSR-based RANDOM effects (always on, ignores
      -- the Sea State switch which had been gating everything to zero):
      --   col_noise        - per-column wave_top jitter        (up to +/-64 px)
      --   col_phase_jitter - per-column phase offset (random, jagged)
      --   warble           - per-layer per-frame phase drift
      --
      -- Drift knob scales the SIN-based SMOOTH modulator. Applied to
      -- BOTH baseline (col_slow) AND phase (col_phase_slo) so peaks in
      -- different regions of the screen have different spacing AND
      -- different heights -- gives organic uneven peak distribution.
      if v_at_col_edge then
        v_raw_noise := signed(s_lfsr_out(9 downto 2));  -- ±127 jitter source
        v_slow_addr := s_slow_phase(15 downto 10) + s_pixel_x(10 downto 5);
        v_slow_raw  := C_SIN(to_integer(v_slow_addr));

        case s_chaos(9 downto 8) is
          when "00"   => v_col_noise_new := (others => '0');
          when "01"   => v_col_noise_new := shift_right(v_raw_noise, 3);  -- +/- 16 px
          when "10"   => v_col_noise_new := shift_right(v_raw_noise, 2);  -- +/- 32 px
          when others => v_col_noise_new := shift_right(v_raw_noise, 1);  -- +/- 64 px
        end case;

        case s_drift(9 downto 8) is
          when "00"   =>
            v_col_slow_new  := (others => '0');
            v_col_phase_slo := (others => '0');
          when "01"   =>
            v_col_slow_new  := shift_right(v_slow_raw, 4);              -- +/-  7 px baseline
            v_col_phase_slo := resize(shift_right(v_slow_raw, 5), 6);   -- +/- 3 phase
          when "10"   =>
            v_col_slow_new  := shift_right(v_slow_raw, 3);              -- +/- 15 px baseline
            v_col_phase_slo := resize(shift_right(v_slow_raw, 4), 6);   -- +/- 7 phase
          when others =>
            v_col_slow_new  := shift_right(v_slow_raw, 2);              -- +/- 31 px baseline
            v_col_phase_slo := resize(shift_right(v_slow_raw, 3), 6);   -- +/- 15 phase
        end case;

        -- Phase total = warble-adjusted const + smooth slow-mod phase.
        -- Only one phase contribution: the spatially-coherent slow modulator.
        -- (LFSR phase jitter dropped -- it produced incoherent noise that
        -- hid peaks rather than just spacing them unevenly.)
        for li in 0 to C_NUM_LAYERS - 1 loop
          v_phase_total_new(li) := s_phase_const(li)
                                   + unsigned(std_logic_vector(v_col_phase_slo));
          s_phase_total(li) <= v_phase_total_new(li);
        end loop;

        s_col_noise  <= v_col_noise_new;
        s_col_slow   <= v_col_slow_new;
        s1_col_noise <= v_col_noise_new;
        s1_col_slow  <= v_col_slow_new;
      else
        for li in 0 to C_NUM_LAYERS - 1 loop
          v_phase_total_new(li) := s_phase_total(li);
        end loop;
        s1_col_noise <= s_col_noise;
        s1_col_slow  <= s_col_slow;
      end if;

      -- Per-layer raw phase + precombined phase_total = single 6-bit add.
      -- v_phase_total_new tracks the same value as s_phase_total but is
      -- valid the same cycle it was computed (signal would lag by 1).
      for li in 0 to C_NUM_LAYERS - 1 loop
        v_scrolled   := resize(signed('0' & std_logic_vector(v_x_snap)), 16)
                        + s_scroll(li);
        v_scrolled_u := unsigned(std_logic_vector(v_scrolled));
        v_raw_phase  := resize(v_scrolled_u srl to_integer(C_LAYER(li).wshift), 16);
        s1_phase(li) <= v_raw_phase(5 downto 0) + v_phase_total_new(li);
      end loop;
    end if;
  end process p_s1;

  -- =====================================================================
  -- S2: sin LUT lookups
  --   cos(phase)    = sin(phase + 16)
  --   cos(2*phase)  = sin(2*phase + 16)
  -- The 2nd harmonic narrows crests and broadens troughs (Stokes shape).
  -- =====================================================================
  p_s2 : process(clk)
    variable v_addr_cos1 : unsigned(5 downto 0);
    variable v_addr_cos2 : unsigned(6 downto 0);
  begin
    if rising_edge(clk) then
      for li in 0 to C_NUM_LAYERS - 1 loop
        v_addr_cos1 := s1_phase(li) + to_unsigned(16, 6);
        v_addr_cos2 := resize(s1_phase(li) & '0', 7) + to_unsigned(16, 7);
        s2_cos1(li) <= C_SIN(to_integer(v_addr_cos1));
        s2_cos2(li) <= C_SIN(to_integer(v_addr_cos2(5 downto 0)));
      end loop;
      s2_phase     <= s1_phase;
      s2_cell_x    <= s1_cell_x;
      s2_cell_y    <= s1_cell_y;
      s2_col_noise <= s1_col_noise;
      s2_col_slow  <= s1_col_slow;
    end if;
  end process p_s2;

  -- =====================================================================
  -- S3a: shape the waveform (jagged or smooth), prep multiplicands.
  --   combined  = cos1 + (cos2 sra 2)            (Stokes 2nd harmonic / 4)
  --   amp_x_knob = layer.amp * height_knob(7b)   (single 6x7 multiply)
  --   base      = horizon + layer.offset_y       (per-layer baseline y)
  -- =====================================================================
  p_s3a : process(clk)
    variable v_cos1       : signed(7 downto 0);
    variable v_cos2       : signed(7 downto 0);
    variable v_combined   : signed(9 downto 0);
    variable v_base       : signed(12 downto 0);
    variable v_layer_off  : unsigned(11 downto 0);
    variable v_tri        : signed(7 downto 0);
    variable v_quarter    : unsigned(1 downto 0);
    variable v_phase      : unsigned(5 downto 0);
    variable v_frac       : unsigned(3 downto 0);
  begin
    if rising_edge(clk) then
      for li in 0 to C_NUM_LAYERS - 1 loop
        v_cos1 := s2_cos1(li);
        v_cos2 := s2_cos2(li);

        if s_jagged = '1' then
          -- Triangle wave at same phase (still cos-addr convention).
          v_phase   := s2_phase(li) + to_unsigned(16, 6);
          v_quarter := v_phase(5 downto 4);
          v_frac    := v_phase(3 downto 0);
          case v_quarter is
            when "00" | "10" =>
              v_tri := signed(shift_left(resize(v_frac, 8), 3));
            when others =>
              v_tri := signed(shift_left(resize(to_unsigned(15, 4) - v_frac, 8), 3));
          end case;
          if v_quarter(1) = '1' then
            v_tri := -v_tri;
          end if;
          v_cos1 := v_tri;
          v_cos2 := (others => '0');
        end if;

        -- Per-layer 2nd-harmonic weight: layer with smaller h2_shift = sharper
        -- crests. h2_shift is constant per li so the sra is a static shift.
        v_combined   := resize(v_cos1, 10)
                        + resize(shift_right(v_cos2, to_integer(C_LAYER(li).h2_shift)), 10);
        v_layer_off  := resize(C_LAYER(li).offset_y, 12);
        v_base       := signed('0' & std_logic_vector(s_horizon + v_layer_off));

        s3a_combined(li)   <= v_combined;
        s3a_amp_x_knob(li) <= s_amp_x_knob(li);  -- precomputed (tide-modulated)
        s3a_base(li)       <= v_base;
      end loop;

      s3a_phase  <= s2_phase;
      s3a_cell_x <= s2_cell_x;
      s3a_cell_y <= s2_cell_y;
      -- Combine col_noise + col_slow into one offset so S3c has a shorter
      -- adder chain (single combined-offset add instead of two).
      s3a_col_off <= resize(s2_col_noise, 9) + resize(s2_col_slow, 9);
    end if;
  end process p_s3a;

  -- =====================================================================
  -- S3b: multiplier-only stage. The 14b*10b multiply is its own pipeline
  -- stage so it doesn't share a clock cycle with adders.
  -- =====================================================================
  p_s3b : process(clk)
    variable v_amp_s : signed(13 downto 0);
  begin
    if rising_edge(clk) then
      for li in 0 to C_NUM_LAYERS - 1 loop
        v_amp_s := signed('0' & std_logic_vector(s3a_amp_x_knob(li)));
        s3b_prod(li) <= v_amp_s * s3a_combined(li);
      end loop;
      s3b_base    <= s3a_base;
      s3b_phase   <= s3a_phase;
      s3b_cell_x  <= s3a_cell_x;
      s3b_cell_y  <= s3a_cell_y;
      s3b_col_off <= s3a_col_off;
    end if;
  end process p_s3b;

  -- =====================================================================
  -- S3c: assemble wave_top = base + (-(prod >> 14)) + col_noise.
  -- Adders only, no multiplier on this path.
  -- =====================================================================
  p_s3c : process(clk)
    variable v_off     : signed(12 downto 0);
    variable v_off_ext : signed(12 downto 0);
  begin
    if rising_edge(clk) then
      v_off_ext := resize(s3b_col_off, 13);
      for li in 0 to C_NUM_LAYERS - 1 loop
        v_off := -resize(s3b_prod(li) sra 14, 13);
        s3c_wave_top(li) <= s3b_base(li) + v_off + v_off_ext;
      end loop;
      s3c_phase   <= s3b_phase;
      s3c_cell_x  <= s3b_cell_x;
      s3c_cell_y  <= s3b_cell_y;
      s3c_col_off <= s3b_col_off;
    end if;
  end process p_s3c;

  -- =====================================================================
  -- S4: determine which layer owns this cell (front-to-back priority)
  --     and compute foam status from the owning layer's phase.
  -- Foam ASR envelope (raw phase 0..63, phase 0 = crest of the wave):
  --   phase  0..7  -> level 3 (white)        Attack + Sustain
  --   phase  8..15 -> level 3 (white)        Sustain (extended by knob 2)
  --   phase 16..23 -> level 2 (mid crest)    Release 1
  --   phase 24..31 -> level 1 (subtle tint)  Release 2
  --   phase 32..63 -> level 0 (none)         Silent
  -- The 'foam_life' knob lengthens the Sustain region (shifts release later).
  -- =====================================================================
  -- =====================================================================
  -- S4: layer ownership + foam status.
  --
  -- ASR envelope is TIDE-DIRECTION AWARE:
  --   Inward crash:  sustain & release windows are LONGER -> foam lingers
  --                  in the wake; band is one cell THICKER -> visible build.
  --   Outward recede: windows SHORT, band thin, and the foam pattern is
  --                   hash-thinned by the col_noise LSBs so it FRAGMENTS
  --                   into chunks as it dies off behind the retreating wave.
  -- This gives the look of foam-with-its-own-physics without needing a
  -- per-column persistent life RAM.
  -- =====================================================================
  p_s4 : process(clk)
    variable v_owner       : unsigned(1 downto 0);
    variable v_top_pix     : signed(12 downto 0);
    variable v_top_cell    : unsigned(8 downto 0);
    variable v_dist        : unsigned(8 downto 0);
    variable v_phase       : unsigned(5 downto 0);
    variable v_sustain_end : unsigned(5 downto 0);
    variable v_rel1_end    : unsigned(5 downto 0);
    variable v_rel2_end    : unsigned(5 downto 0);
    variable v_level       : unsigned(1 downto 0);
    variable v_claimed     : boolean;
    variable v_band_max    : unsigned(8 downto 0);
    variable v_deep_hint   : unsigned(8 downto 0);
  begin
    if rising_edge(clk) then
      v_owner   := "11";
      v_level   := "00";
      v_claimed := false;

      -- Foam params (band and step) are precomputed in S1 vsync from
      -- tide_vel. Use them directly here.
      if s_chunk16 = '1' then
        if s_foam_band_thick = '1' then
          v_band_max  := to_unsigned(1, 9);
          v_deep_hint := to_unsigned(2, 9);
        else
          v_band_max  := to_unsigned(0, 9);
          v_deep_hint := to_unsigned(1, 9);
        end if;
      else
        if s_foam_band_thick = '1' then
          v_band_max  := to_unsigned(2, 9);
          v_deep_hint := to_unsigned(3, 9);
        else
          v_band_max  := to_unsigned(1, 9);
          v_deep_hint := to_unsigned(2, 9);
        end if;
      end if;

      -- Front-to-back: front layer claims first.
      for li in C_NUM_LAYERS - 1 downto 0 loop
        if not v_claimed then
          v_top_pix := s3c_wave_top(li);

          if v_top_pix < 0 then
            v_top_cell := (others => '0');
          elsif v_top_pix > 4095 then
            v_top_cell := (others => '1');
          elsif s_chunk16 = '1' then
            v_top_cell := unsigned(std_logic_vector(v_top_pix(12 downto 4)));
          else
            v_top_cell := unsigned(std_logic_vector(v_top_pix(11 downto 3)));
          end if;

          if s3c_cell_y >= v_top_cell then
            v_owner   := to_unsigned(li, 2);
            v_claimed := true;

            v_phase       := s3c_phase(li);
            v_sustain_end := s_foam_step;
            v_rel1_end    := v_sustain_end + s_foam_step;
            v_rel2_end    := v_rel1_end    + s_foam_step;

            v_dist := s3c_cell_y - v_top_cell;

            if v_dist <= v_band_max then
              if v_phase < v_sustain_end then
                v_level := "11";                                       -- back: white
              elsif v_phase < v_rel1_end then
                v_level := "10";                                       -- back: high
              elsif v_phase < v_rel2_end then
                v_level := "01";                                       -- back: light
              -- Asymmetric leading edge: foam appears just BEFORE the next
              -- crest arrives (phase wrapping from 63 -> 0). Window is half
              -- the trailing wake -> waves look like they're foaming both
              -- sides but the back wake is brighter / longer.
              -- phase >= (64 - s_foam_step/2) -> 'mid' leading band
              -- phase >= (64 - s_foam_step)   -> 'far' leading band
              elsif v_phase >= (to_unsigned(0, 6) - shift_right(s_foam_step, 1)) then
                v_level := "10";                                       -- leading: high
              elsif v_phase >= (to_unsigned(0, 6) - s_foam_step) then
                v_level := "01";                                       -- leading: light
              else
                v_level := "00";
              end if;
            elsif v_dist = v_deep_hint and v_phase < to_unsigned(4, 6) then
              v_level := "01";  -- one cell deeper at sharp peak
            else
              v_level := "00";
            end if;
          end if;
        end if;
      end loop;

      s4_owner    <= v_owner;
      s4_foam_lvl <= v_level;
      s4_cell_x   <= s3c_cell_x;
      s4_cell_y   <= s3c_cell_y;
    end if;
  end process p_s4;

  -- =====================================================================
  -- S5: color selection from owner + foam_lvl
  -- =====================================================================
  p_s5 : process(clk)
    variable v_y, v_u, v_v : unsigned(9 downto 0);
    variable v_ocean : t_yuv;
  begin
    if rising_edge(clk) then
      if s4_owner = "11" then
        v_y := C_SKY(to_integer(s_sky_idx)).y;
        v_u := C_SKY(to_integer(s_sky_idx)).u;
        v_v := C_SKY(to_integer(s_sky_idx)).v;
      else
        v_ocean := C_OCEAN(to_integer(s4_owner));
        case s4_foam_lvl is
          when "11" =>
            v_y := C_FOAM_WHITE.y;
            v_u := C_FOAM_WHITE.u;
            v_v := C_FOAM_WHITE.v;
          when "10" =>
            v_y := C_CREST_HIGH.y;
            v_u := C_CREST_HIGH.u;
            v_v := C_CREST_HIGH.v;
          when "01" =>
            v_y := C_CREST_MID.y;
            v_u := C_CREST_MID.u;
            v_v := C_CREST_MID.v;
          when others =>
            v_y := v_ocean.y;
            v_u := v_ocean.u;
            v_v := v_ocean.v;
        end case;
      end if;

      -- Mono switch: drop chroma to neutral
      if s_mono = '1' then
        v_u := to_unsigned(512, 10);
        v_v := to_unsigned(512, 10);
      end if;

      s5_y <= v_y;
      s5_u <= v_u;
      s5_v <= v_v;
    end if;
  end process p_s5;

  -- =====================================================================
  -- S6: pipeline buffer stage (brightness scaling removed when the slider
  -- was repurposed for Chaos -- the slider is now a vital function).
  -- =====================================================================
  p_s6 : process(clk)
  begin
    if rising_edge(clk) then
      s6_y <= s5_y;
      s6_u <= s5_u;
      s6_v <= s5_v;
    end if;
  end process p_s6;

  -- =====================================================================
  -- S7: output mux with bypass
  -- =====================================================================
  data_out.y <= pipe(C_LATENCY - 1).y when s_bypass = '1'
                else std_logic_vector(s6_y);
  data_out.u <= pipe(C_LATENCY - 1).u when s_bypass = '1'
                else std_logic_vector(s6_u);
  data_out.v <= pipe(C_LATENCY - 1).v when s_bypass = '1'
                else std_logic_vector(s6_v);

  data_out.hsync_n <= pipe(C_LATENCY - 1).hsync_n;
  data_out.vsync_n <= pipe(C_LATENCY - 1).vsync_n;
  data_out.field_n <= pipe(C_LATENCY - 1).field_n;
  data_out.avid    <= pipe(C_LATENCY - 1).avid;

end architecture pixelbeach;
