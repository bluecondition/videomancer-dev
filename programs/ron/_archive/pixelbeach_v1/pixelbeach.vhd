-- Pixel Beach: Generative 2D pixelated beach scene
-- 6 parallax wave layers with organic asymmetric shape, ebb/flow tide,
-- wavy foam bands, and warm sandy beach at bottom-left.
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

  constant C_LATENCY    : integer := 7;
  constant C_NUM_LAYERS : integer := 6;

  -- 64-entry sine LUT
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

  -- Layer constants (amplitudes reduced 25% from original)
  type t_layer_param is record
    base_y : unsigned(11 downto 0);
    amp    : unsigned(5 downto 0);
    wshift : unsigned(2 downto 0);
  end record;
  type t_layer_params is array(0 to C_NUM_LAYERS - 1) of t_layer_param;
  constant C_LAYER : t_layer_params := (
    0 => (base_y => to_unsigned(620, 12), amp => to_unsigned(30, 6), wshift => to_unsigned(3, 3)),
    1 => (base_y => to_unsigned(560, 12), amp => to_unsigned(24, 6), wshift => to_unsigned(4, 3)),
    2 => (base_y => to_unsigned(500, 12), amp => to_unsigned(18, 6), wshift => to_unsigned(4, 3)),
    3 => (base_y => to_unsigned(450, 12), amp => to_unsigned(14, 6), wshift => to_unsigned(5, 3)),
    4 => (base_y => to_unsigned(400, 12), amp => to_unsigned(10, 6), wshift => to_unsigned(5, 3)),
    5 => (base_y => to_unsigned(365, 12), amp => to_unsigned( 6, 6), wshift => to_unsigned(6, 3))
  );

  type t_yuv is record
    y : unsigned(9 downto 0);
    u : unsigned(9 downto 0);
    v : unsigned(9 downto 0);
  end record;
  type t_ocean_colors is array(0 to C_NUM_LAYERS - 1) of t_yuv;
  constant C_OCEAN : t_ocean_colors := (
    0 => (y => to_unsigned(420, 10), u => to_unsigned(420, 10), v => to_unsigned(560, 10)),
    1 => (y => to_unsigned(390, 10), u => to_unsigned(410, 10), v => to_unsigned(575, 10)),
    2 => (y => to_unsigned(360, 10), u => to_unsigned(400, 10), v => to_unsigned(590, 10)),
    3 => (y => to_unsigned(330, 10), u => to_unsigned(390, 10), v => to_unsigned(600, 10)),
    4 => (y => to_unsigned(300, 10), u => to_unsigned(385, 10), v => to_unsigned(610, 10)),
    5 => (y => to_unsigned(280, 10), u => to_unsigned(380, 10), v => to_unsigned(620, 10))
  );

  -- ========================================================================
  -- Signals
  -- ========================================================================
  type t_pipe is array(0 to C_LATENCY - 1) of t_video_stream_yuv444_30b;
  signal pipe : t_pipe;

  signal s_pixel_x : unsigned(11 downto 0) := (others => '0');
  signal s_pixel_y : unsigned(11 downto 0) := (others => '0');
  signal s_prev_hsync_n : std_logic := '1';
  signal s_prev_vsync_n : std_logic := '1';
  signal s_frame_count  : unsigned(15 downto 0) := (others => '0');

  type t_scroll_arr is array(0 to C_NUM_LAYERS - 1) of unsigned(23 downto 0);
  signal s_scroll : t_scroll_arr := (others => (others => '0'));

  signal s_tide_phase : unsigned(23 downto 0) := (others => '0');
  type t_tide_arr is array(0 to C_NUM_LAYERS - 1) of signed(7 downto 0);
  signal s_tide_sin : t_tide_arr := (others => (others => '0'));

  -- S2: three sin addresses per layer (main + harmonic + foam wave)
  type t_addr_arr is array(0 to C_NUM_LAYERS - 1) of unsigned(5 downto 0);
  signal s2_sin_addr  : t_addr_arr := (others => (others => '0'));
  signal s2_sin_addr2 : t_addr_arr := (others => (others => '0'));
  signal s2_foam_addr : t_addr_arr := (others => (others => '0'));
  signal s2_x : unsigned(11 downto 0) := (others => '0');
  signal s2_y : unsigned(11 downto 0) := (others => '0');

  -- S3: wave boundaries + foam boundaries + sin values
  type t_wtop_arr is array(0 to C_NUM_LAYERS - 1) of unsigned(11 downto 0);
  signal s3_wave_top : t_wtop_arr := (others => (others => '0'));
  signal s3_foam_bot : t_wtop_arr := (others => (others => '0'));  -- foam extends to here
  type t_s8_arr is array(0 to C_NUM_LAYERS - 1) of signed(7 downto 0);
  signal s3_sin_val : t_s8_arr := (others => (others => '0'));
  signal s3_x : unsigned(11 downto 0) := (others => '0');
  signal s3_y : unsigned(11 downto 0) := (others => '0');

  -- S4
  signal s4_region : unsigned(2 downto 0) := (others => '0');
  signal s4_active_layer : unsigned(2 downto 0) := (others => '0');
  signal s4_crest_dist : unsigned(7 downto 0) := (others => '0');
  signal s4_foam_on : std_logic := '0';
  signal s4_wave_sin : signed(7 downto 0) := (others => '0');
  signal s4_y : unsigned(11 downto 0) := (others => '0');
  signal s4_x : unsigned(11 downto 0) := (others => '0');

  -- S5
  signal s5_y, s5_u, s5_v : unsigned(9 downto 0) := (others => '0');
  signal s5_foam_alpha : unsigned(7 downto 0) := (others => '0');

  -- S6
  signal s6_y, s6_u, s6_v : unsigned(9 downto 0) := (others => '0');

  -- Controls
  signal s_speed    : unsigned(9 downto 0) := (others => '0');
  signal s_height   : unsigned(2 downto 0) := (others => '0');
  signal s_foam_th  : unsigned(4 downto 0) := (others => '0');
  signal s_tide_amt : unsigned(9 downto 0) := (others => '0');
  signal s_color_sh : signed(9 downto 0) := (others => '0');
  signal s_horizon  : unsigned(11 downto 0) := (others => '0');
  signal s_choppy   : std_logic := '0';
  signal s_sunset   : std_logic := '0';
  signal s_night    : std_logic := '0';
  signal s_deep     : std_logic := '0';
  signal s_bypass   : std_logic := '0';
  signal s_bright   : unsigned(9 downto 0) := (others => '0');

begin

  -- ========================================================================
  -- S1
  -- ========================================================================
  p_s1 : process(clk)
    variable v_base_speed : unsigned(15 downto 0);
    variable v_tide_addr  : unsigned(5 downto 0);
  begin
    if rising_edge(clk) then
      s_prev_hsync_n <= data_in.hsync_n;
      s_prev_vsync_n <= data_in.vsync_n;

      pipe(0).avid    <= data_in.avid;
      pipe(0).hsync_n <= data_in.hsync_n;
      pipe(0).vsync_n <= data_in.vsync_n;
      pipe(0).field_n <= data_in.field_n;
      pipe(0).y <= (others => '0');
      pipe(0).u <= std_logic_vector(to_unsigned(512, 10));
      pipe(0).v <= std_logic_vector(to_unsigned(512, 10));
      for i in 1 to C_LATENCY - 1 loop
        pipe(i) <= pipe(i - 1);
      end loop;

      if data_in.avid = '1' then
        s_pixel_x <= s_pixel_x + 1;
      end if;
      if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
        s_pixel_x <= (others => '0');
        s_pixel_y <= s_pixel_y + 1;
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_pixel_y <= (others => '0');
        s_frame_count <= s_frame_count + 1;

        s_speed    <= unsigned(registers_in(0));
        s_height   <= unsigned(registers_in(1)(9 downto 7));
        s_foam_th  <= unsigned(registers_in(2)(9 downto 5));
        s_tide_amt <= unsigned(registers_in(3));
        s_color_sh <= signed(registers_in(4)) - to_signed(512, 10);
        s_horizon  <= resize(unsigned(registers_in(5)), 12);
        s_choppy   <= registers_in(6)(0);
        s_sunset   <= registers_in(6)(1);
        s_night    <= registers_in(6)(2);
        s_deep     <= registers_in(6)(3);
        s_bypass   <= registers_in(6)(4);
        s_bright   <= unsigned(registers_in(7));

        -- 128× base speed for 1.5 screen-widths of travel per surge
        v_base_speed := resize(unsigned(registers_in(0)), 16);
        v_base_speed := v_base_speed(8 downto 0) & "0000000";

        -- Bidirectional scroll: all layers same speed
        for li in 0 to C_NUM_LAYERS - 1 loop
          if s_tide_sin(li) >= 0 then
            s_scroll(li) <= s_scroll(li) + resize(v_base_speed, 24);
          else
            s_scroll(li) <= s_scroll(li) - resize(v_base_speed - (v_base_speed srl 2), 24);
          end if;
        end loop;

        -- Tide: ~3.5 sec full cycle (~1.75s surge, ~1.75s recede)
        s_tide_phase <= s_tide_phase + to_unsigned(80000, 24);
        for li in 0 to C_NUM_LAYERS - 1 loop
          v_tide_addr := s_tide_phase(23 downto 18) + to_unsigned(li * 11, 6);
          s_tide_sin(li) <= C_SIN(to_integer(v_tide_addr));
        end loop;
      end if;
    end if;
  end process p_s1;

  -- ========================================================================
  -- S2: Sin addresses — main + harmonic + foam wave (different frequency)
  -- ========================================================================
  p_s2 : process(clk)
    variable v_scrolled : unsigned(11 downto 0);
    variable v_addr  : unsigned(11 downto 0);
    variable v_addr2 : unsigned(11 downto 0);
    variable v_faddr : unsigned(11 downto 0);
  begin
    if rising_edge(clk) then
      for i in 0 to C_NUM_LAYERS - 1 loop
        v_scrolled := s_pixel_x + s_scroll(i)(23 downto 12);

        -- Main wave
        v_addr := v_scrolled srl to_integer(C_LAYER(i).wshift);
        s2_sin_addr(i) <= v_addr(5 downto 0);

        -- Harmonic at ~3× frequency for irregular shape
        if C_LAYER(i).wshift > 1 then
          v_addr2 := v_scrolled srl to_integer(C_LAYER(i).wshift - 2);
        else
          v_addr2 := v_scrolled;
        end if;
        s2_sin_addr2(i) <= v_addr2(5 downto 0) + to_unsigned(17, 6);

        -- Foam wave: different frequency and phase from main wave,
        -- creates varying foam depth across the wave
        if C_LAYER(i).wshift > 0 then
          v_faddr := v_scrolled srl to_integer(C_LAYER(i).wshift - 1);
        else
          v_faddr := v_scrolled;
        end if;
        s2_foam_addr(i) <= v_faddr(5 downto 0) + to_unsigned(41, 6);
      end loop;
      s2_x <= s_pixel_x;
      s2_y <= s_pixel_y;
    end if;
  end process p_s2;

  -- ========================================================================
  -- S3: Wave boundary + foam boundary
  --   Asymmetric: crests sharp (full amp), troughs shallow (half amp).
  --   Foam boundary = wave_top + foam_wave * foam_range.
  -- ========================================================================
  p_s3 : process(clk)
    variable v_sin1 : signed(7 downto 0);
    variable v_sin2 : signed(7 downto 0);
    variable v_foam_sin : signed(7 downto 0);
    variable v_prod : signed(15 downto 0);
    variable v_harm : signed(15 downto 0);
    variable v_off  : signed(11 downto 0);
    variable v_top  : signed(12 downto 0);
    variable v_fbot : signed(12 downto 0);
    variable v_foam_depth : signed(11 downto 0);
    variable v_addr : unsigned(5 downto 0);
    variable v_tri_mag : unsigned(6 downto 0);
    variable v_quarter : unsigned(1 downto 0);
    variable v_frac : unsigned(3 downto 0);
    variable v_tide_off : signed(11 downto 0);
  begin
    if rising_edge(clk) then
      for i in 0 to C_NUM_LAYERS - 1 loop
        v_addr := s2_sin_addr(i);

        -- Main waveform
        if s_choppy = '0' then
          v_sin1 := C_SIN(to_integer(v_addr));
        else
          v_quarter := v_addr(5 downto 4);
          v_frac := v_addr(3 downto 0);
          case v_quarter is
            when "00" | "10" =>
              v_tri_mag := v_frac & "000";
            when others =>
              v_tri_mag := (to_unsigned(15, 4) - v_frac) & "000";
          end case;
          if v_quarter(1) = '0' then
            v_sin1 := signed(resize(v_tri_mag, 8));
          else
            v_sin1 := -signed(resize(v_tri_mag, 8));
          end if;
        end if;

        -- ASYMMETRIC: troughs are half amplitude (flat bottom, sharp crest)
        if v_sin1 < 0 then
          v_sin1 := v_sin1 sra 1;
        end if;

        -- Harmonic at ~3× freq
        v_sin2 := C_SIN(to_integer(s2_sin_addr2(i)));

        -- Foam wave (different freq/phase for organic foam bands)
        v_foam_sin := C_SIN(to_integer(s2_foam_addr(i)));

        s3_sin_val(i) <= v_sin1;

        -- Main * amplitude
        v_prod := v_sin1 * resize(signed('0' & std_logic_vector(C_LAYER(i).amp)), 8);
        -- Harmonic * amplitude/4
        v_harm := v_sin2 * resize(signed('0' & std_logic_vector(C_LAYER(i).amp srl 2)), 8);

        -- Combined offset scaled by height knob
        v_off := resize((v_prod + v_harm) sra to_integer(7 - s_height), 12);

        -- Tide surge
        case i is
          when 0      => v_tide_off := resize(s_tide_sin(0), 12) + resize(s_tide_sin(0), 12);
          when 1      => v_tide_off := resize(s_tide_sin(1), 12) + resize(s_tide_sin(1) sra 1, 12);
          when 2      => v_tide_off := resize(s_tide_sin(2), 12);
          when 3      => v_tide_off := resize(s_tide_sin(3) sra 1, 12);
          when 4      => v_tide_off := resize(s_tide_sin(4) sra 2, 12);
          when others => v_tide_off := resize(s_tide_sin(5) sra 3, 12);
        end case;

        -- Wave top
        v_top := signed('0' & std_logic_vector(C_LAYER(i).base_y))
                 + resize(v_off, 13)
                 + resize(v_tide_off, 13);

        if v_top < 0 then
          s3_wave_top(i) <= (others => '0');
        elsif v_top > 4095 then
          s3_wave_top(i) <= to_unsigned(4095, 12);
        else
          s3_wave_top(i) <= unsigned(v_top(11 downto 0));
        end if;

        -- Foam boundary: extends BELOW wave_top by a varying amount.
        -- The foam wave creates undulating foam depth across the wave.
        -- foam_depth = base_foam_depth + foam_sin * foam_depth/2
        -- where base_foam_depth is from the Foam knob.
        -- foam_sin > 0: thick foam. foam_sin < 0: thin foam.
        if v_foam_sin > 0 then
          v_foam_depth := to_signed(to_integer(s_foam_th) * 8, 12)
                        + to_signed(to_integer(unsigned(std_logic_vector(v_foam_sin(6 downto 0)))) * 2, 12);
        else
          v_foam_depth := to_signed(to_integer(s_foam_th) * 4, 12);
        end if;

        v_fbot := v_top + resize(v_foam_depth, 13);
        if v_fbot > 4095 then
          s3_foam_bot(i) <= to_unsigned(4095, 12);
        elsif v_fbot < 0 then
          s3_foam_bot(i) <= (others => '0');
        else
          s3_foam_bot(i) <= unsigned(v_fbot(11 downto 0));
        end if;
      end loop;

      s3_x <= s2_x;
      s3_y <= s2_y;
    end if;
  end process p_s3;

  -- ========================================================================
  -- S4: Layer select + foam (between wave_top and foam_bot) + beach
  -- ========================================================================
  p_s4 : process(clk)
    variable v_region : unsigned(2 downto 0);
    variable v_layer  : unsigned(2 downto 0);
    variable v_cdist  : unsigned(7 downto 0);
    variable v_foam   : std_logic;
    variable v_wsin   : signed(7 downto 0);
    variable v_hash   : unsigned(15 downto 0);
    variable v_beach_width : unsigned(11 downto 0);
    variable v_beach_line  : unsigned(11 downto 0);
  begin
    if rising_edge(clk) then
      v_region := "000";
      v_layer  := "000";
      v_cdist  := (others => '1');
      v_foam   := '0';
      v_wsin   := (others => '0');

      -- Beach
      if s_deep = '0' then
        v_beach_width := resize(s_tide_amt srl 1, 12);
        if s3_x < v_beach_width then
          v_beach_line := to_unsigned(720, 12) - resize(v_beach_width - s3_x, 12);
          if s3_y > v_beach_line and s3_y < s3_wave_top(0) then
            v_region := "111";
          end if;
        end if;
      end if;

      -- Layer check
      if v_region = "000" then
        for i in 0 to C_NUM_LAYERS - 1 loop
          if s3_y >= s3_wave_top(i) and v_region = "000" then
            v_region := to_unsigned(i + 1, 3);
            v_layer  := to_unsigned(i, 3);
            v_wsin   := s3_sin_val(i);

            if (s3_y - s3_wave_top(i)) < 256 then
              v_cdist := resize(s3_y - s3_wave_top(i), 8);
            else
              v_cdist := (others => '1');
            end if;

            -- Foam: between wave_top and foam_bot
            -- Use organic hash to scatter — foam isn't solid, it's noisy
            if s3_y < s3_foam_bot(i) then
              -- Organic hash for foam scatter
              v_hash := resize(s3_x(9 downto 0), 16)
                        + resize(s3_y(8 downto 0) & "0000000", 16)
                        + resize(s_frame_count(3 downto 0), 16);
              v_hash := v_hash xor (v_hash srl 5) xor (v_hash srl 11);

              if v_cdist < 4 then
                -- Crest: solid foam
                v_foam := '1';
              elsif v_cdist < 12 then
                -- Near crest: dense foam (~70%)
                if v_hash(7 downto 0) < 180 then
                  v_foam := '1';
                end if;
              else
                -- Further down: sparser foam, fading with depth
                if v_hash(7 downto 0) < (128 - v_cdist) then
                  v_foam := '1';
                end if;
              end if;

              -- Suppress deep foam on wave trough
              if v_wsin < -32 and v_cdist > 16 then
                v_foam := '0';
              end if;
            end if;
          end if;
        end loop;
      end if;

      s4_region <= v_region;
      s4_active_layer <= v_layer;
      s4_crest_dist <= v_cdist;
      s4_foam_on <= v_foam;
      s4_wave_sin <= v_wsin;
      s4_y <= s3_y;
      s4_x <= s3_x;
    end if;
  end process p_s4;

  -- ========================================================================
  -- S5: Color
  -- ========================================================================
  p_s5 : process(clk)
    variable v_y, v_u, v_v : unsigned(9 downto 0);
    variable v_sky_frac : unsigned(9 downto 0);
    variable v_foam_a   : unsigned(7 downto 0);
    variable v_color_off : signed(9 downto 0);
    variable v_abs_off   : unsigned(8 downto 0);
  begin
    if rising_edge(clk) then
      v_color_off := s_color_sh;

      case s4_region is
        when "000" =>
          v_sky_frac := s4_y(9 downto 0);
          if s_night = '1' then
            v_y := resize(v_sky_frac srl 4, 10) + to_unsigned(20, 10);
            v_u := to_unsigned(470, 10);
            v_v := to_unsigned(560, 10);
          elsif s_sunset = '1' then
            v_y := to_unsigned(200, 10) + resize(v_sky_frac, 10);
            if v_y > 700 then v_y := to_unsigned(700, 10); end if;
            v_u := to_unsigned(530, 10) + resize(v_sky_frac srl 2, 10);
            if v_u > 700 then v_u := to_unsigned(700, 10); end if;
            v_v := to_unsigned(400, 10);
          else
            v_y := to_unsigned(200, 10) + resize(v_sky_frac, 10) + resize(v_sky_frac srl 1, 10);
            if v_y > 700 then v_y := to_unsigned(700, 10); end if;
            v_u := to_unsigned(340, 10) + resize(v_sky_frac srl 2, 10);
            v_v := to_unsigned(680, 10) - resize(v_sky_frac srl 2, 10);
          end if;

        when "111" =>
          -- Warm sandy brown (U/V swapped for Videomancer)
          v_y := to_unsigned(700, 10);
          v_u := to_unsigned(580, 10);
          v_v := to_unsigned(400, 10);

        when others =>
          v_y := C_OCEAN(to_integer(s4_active_layer)).y;
          v_u := C_OCEAN(to_integer(s4_active_layer)).u;
          v_v := C_OCEAN(to_integer(s4_active_layer)).v;

          if v_color_off > 0 then
            v_abs_off := unsigned(std_logic_vector(v_color_off(8 downto 0)));
            if v_u >= resize(v_abs_off, 10) then
              v_u := v_u - resize(v_abs_off, 10);
            else
              v_u := (others => '0');
            end if;
          elsif v_color_off < 0 then
            v_abs_off := unsigned(std_logic_vector(resize(-v_color_off, 9)));
            v_u := v_u + resize(v_abs_off, 10);
            if v_u > 1023 then v_u := to_unsigned(1023, 10); end if;
          end if;
      end case;

      -- Foam alpha
      if s4_foam_on = '1' then
        if s4_crest_dist < 8 then
          v_foam_a := to_unsigned(240, 8);
        elsif s4_crest_dist < 24 then
          v_foam_a := to_unsigned(180, 8);
        elsif s4_crest_dist < 64 then
          v_foam_a := to_unsigned(120, 8);
        else
          v_foam_a := to_unsigned(60, 8);
        end if;
        if s4_wave_sin > 64 then
          null;
        elsif s4_wave_sin > 0 then
          v_foam_a := v_foam_a - (v_foam_a srl 2);
        else
          v_foam_a := v_foam_a srl 1;
        end if;
      else
        v_foam_a := (others => '0');
      end if;

      s5_y <= v_y;
      s5_u <= v_u;
      s5_v <= v_v;
      s5_foam_alpha <= v_foam_a;
    end if;
  end process p_s5;

  -- ========================================================================
  -- S6: Foam blend + brightness
  -- ========================================================================
  p_s6 : process(clk)
    variable v_y, v_u, v_v : unsigned(9 downto 0);
    variable v_foam_add_y : unsigned(9 downto 0);
    variable v_diff_y : unsigned(9 downto 0);
    variable v_bright_scale : unsigned(3 downto 0);
  begin
    if rising_edge(clk) then
      v_y := s5_y;
      v_u := s5_u;
      v_v := s5_v;

      if s5_foam_alpha > 0 then
        v_diff_y := to_unsigned(1023, 10) - v_y;
        v_foam_add_y := (others => '0');
        if s5_foam_alpha(7) = '1' then
          v_foam_add_y := v_foam_add_y + (v_diff_y srl 1);
        end if;
        if s5_foam_alpha(6) = '1' then
          v_foam_add_y := v_foam_add_y + (v_diff_y srl 2);
        end if;
        if s5_foam_alpha(5) = '1' then
          v_foam_add_y := v_foam_add_y + (v_diff_y srl 3);
        end if;
        v_y := v_y + v_foam_add_y;
        if v_y > 1023 then v_y := to_unsigned(1023, 10); end if;

        if v_u < 512 then
          v_u := v_u + resize(s5_foam_alpha srl 2, 10);
        else
          v_u := v_u - resize(s5_foam_alpha srl 2, 10);
        end if;
        if v_v < 512 then
          v_v := v_v + resize(s5_foam_alpha srl 2, 10);
        else
          v_v := v_v - resize(s5_foam_alpha srl 2, 10);
        end if;
      end if;

      v_bright_scale := s_bright(9 downto 6);
      v_y := resize((resize(v_y, 14) * resize(v_bright_scale, 14)) srl 4, 10);

      s6_y <= v_y;
      s6_u <= v_u;
      s6_v <= v_v;
    end if;
  end process p_s6;

  -- ========================================================================
  -- S7: Output + bypass
  -- ========================================================================
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
