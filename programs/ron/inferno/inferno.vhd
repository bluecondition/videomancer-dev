-- Inferno: Procedural flame effect based on Shadertoy fire shader
-- LZX Industries Community FPGA Program
--
-- Key visual elements from the reference shader:
--   1. Sharp 1/x^4 brightness curve (bright white cores, dark red edges)
--   2. Domain-warped multi-frequency noise for organic turbulence
--   3. Time-evolving flame shape from multiple animation phases
--   4. Warm palette: white → yellow → orange → red → dark red → black
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

architecture inferno of program_top is

  constant C_LATENCY : integer := 8;

  -- 64-entry sin LUT
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

  -- Better hash using sin LUT as nonlinear mixer (approximates shader's
  -- rand = fract(sin(cos(dot(...)))*83758)). The sin LUT adds the
  -- crucial nonlinearity that plain XOR-fold lacks.
  function fire_hash(x, y : unsigned(7 downto 0)) return unsigned is
    variable h1, h2 : unsigned(7 downto 0);
    variable addr1, addr2 : unsigned(5 downto 0);
  begin
    -- Two linear combinations with different primes
    addr1 := resize(x + x + x + (x srl 2) + y + (y srl 1), 6);  -- ~3.25x + 1.5y
    addr2 := resize(x + (x srl 1) + y + y + y + (y srl 2), 6);  -- ~1.5x + 3.25y
    -- Nonlinear mix through sin LUT
    h1 := unsigned(std_logic_vector(C_SIN(to_integer(addr1)))) + to_unsigned(128, 8);
    h2 := unsigned(std_logic_vector(C_SIN(to_integer(addr2)))) + to_unsigned(128, 8);
    -- Combine
    return h1 xor (h2 srl 1) xor (h2 sll 3);
  end function;

  -- ========================================================================
  type t_pipe is array(0 to C_LATENCY - 1) of t_video_stream_yuv444_30b;
  signal pipe : t_pipe;

  signal s_pixel_x : unsigned(11 downto 0) := (others => '0');
  signal s_pixel_y : unsigned(11 downto 0) := (others => '0');
  signal s_prev_hsync_n : std_logic := '1';
  signal s_prev_vsync_n : std_logic := '1';

  signal s_phase_a : unsigned(23 downto 0) := (others => '0');
  signal s_phase_b : unsigned(23 downto 0) := x"400000";
  signal s_phase_c : unsigned(23 downto 0) := x"200000";
  signal s_phase_d : unsigned(23 downto 0) := x"100000";

  signal s_lfsr_q : std_logic_vector(15 downto 0);

  signal s_fuel, s_spread, s_heat, s_turbulence, s_rise, s_intensity : unsigned(9 downto 0)
    := to_unsigned(512, 10);
  signal s_tint : signed(9 downto 0) := (others => '0');
  signal s_ember, s_inferno, s_blue, s_overlay, s_bypass : std_logic := '0';

  -- S2: domain-distorted coordinates
  signal s2_px, s2_py : unsigned(11 downto 0) := (others => '0');
  signal s2_height_frac : unsigned(7 downto 0) := (others => '0');  -- 0=top of fire, 255=base

  -- S3: noise sample addresses (4 corners for bilinear, 2 octaves)
  type t_hash8 is array(0 to 7) of unsigned(7 downto 0);
  signal s3_h : t_hash8 := (others => (others => '0'));
  signal s3_fx1, s3_fy1 : unsigned(2 downto 0) := (others => '0');
  signal s3_fx2, s3_fy2 : unsigned(2 downto 0) := (others => '0');
  signal s3_height_frac : unsigned(7 downto 0) := (others => '0');

  -- S4: bilinear interpolated noise
  signal s4_noise : unsigned(7 downto 0) := (others => '0');
  signal s4_height_frac : unsigned(7 downto 0) := (others => '0');

  -- S5: fire value (noise * height → brightness via 1/x^4 curve)
  signal s5_brightness : unsigned(9 downto 0) := (others => '0');

  -- S6: palette
  signal s6_y, s6_u, s6_v : unsigned(9 downto 0) := (others => '0');

  -- S7: effects
  signal s7_y, s7_u, s7_v : unsigned(9 downto 0) := (others => '0');

  signal s5_br_d1 : unsigned(9 downto 0) := (others => '0');

begin

  lfsr_inst : entity work.lfsr16
    port map(clk => clk, enable => data_in.avid, seed => x"DEAD", load => '0', q => s_lfsr_q);

  -- ========================================================================
  -- S1: Timing + controls + 4 animation phases
  -- ========================================================================
  p_s1 : process(clk)
  begin
    if rising_edge(clk) then
      s_prev_hsync_n <= data_in.hsync_n;
      s_prev_vsync_n <= data_in.vsync_n;
      pipe(0) <= data_in;
      for i in 1 to C_LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

      if data_in.avid = '1' then s_pixel_x <= s_pixel_x + 1; end if;
      if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
        s_pixel_x <= (others => '0'); s_pixel_y <= s_pixel_y + 1;
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_pixel_y <= (others => '0');
        s_fuel <= unsigned(registers_in(0));
        s_spread <= unsigned(registers_in(1));
        s_heat <= unsigned(registers_in(2));
        s_turbulence <= unsigned(registers_in(3));
        s_tint <= signed(registers_in(4)) - to_signed(512, 10);
        s_rise <= unsigned(registers_in(5));
        s_ember <= registers_in(6)(0);
        s_inferno <= registers_in(6)(1);
        s_blue <= registers_in(6)(2);
        s_overlay <= registers_in(6)(3);
        s_bypass <= registers_in(6)(4);
        s_intensity <= unsigned(registers_in(7));

        -- 4 phases at different speeds (like shader's multiple iTime offsets)
        s_phase_a <= s_phase_a + resize(unsigned(registers_in(5)) & "000", 24);
        s_phase_b <= s_phase_b + resize(unsigned(registers_in(5)) & "0", 24)
                     + to_unsigned(7000, 24);
        s_phase_c <= s_phase_c + to_unsigned(9000, 24);
        s_phase_d <= s_phase_d + resize(unsigned(registers_in(5)) & "00", 24)
                     + to_unsigned(3000, 24);
      end if;
    end if;
  end process p_s1;

  -- ========================================================================
  -- S2: Domain distortion + height fraction
  -- Like shader: p += sin(p.yx*4+t)*0.04; p += sin(p.yx*8+t)*0.01;
  -- ========================================================================
  p_s2 : process(clk)
    variable v_fire_height : unsigned(11 downto 0);
    variable v_dist_bottom : unsigned(11 downto 0);
    variable v_hfrac : unsigned(7 downto 0);
    variable v_spread : unsigned(2 downto 0);
    variable v_sx, v_sy : unsigned(11 downto 0);
    -- Domain distortion via sin lookups
    variable v_wob1_addr, v_wob2_addr : unsigned(5 downto 0);
    variable v_wob1, v_wob2 : signed(7 downto 0);
    variable v_dx, v_dy : signed(7 downto 0);
  begin
    if rising_edge(clk) then
      v_fire_height := resize(s_fuel, 12) + to_unsigned(100, 12);
      if s_inferno = '1' then v_fire_height := v_fire_height(10 downto 0) & "0"; end if;

      if s_pixel_y < 720 then
        v_dist_bottom := to_unsigned(720, 12) - s_pixel_y;
      else
        v_dist_bottom := (others => '0');
      end if;

      -- Height fraction: 255 at base (bottom), 0 at top of fire zone
      if v_dist_bottom >= v_fire_height then
        v_hfrac := (others => '0');
      else
        v_hfrac := resize(255 - (v_dist_bottom srl (to_integer(v_fire_height(11 downto 9)) + 1)), 8);
        if v_dist_bottom < v_fire_height srl 1 then
          v_hfrac := to_unsigned(255, 8);
        end if;
      end if;
      s2_height_frac <= v_hfrac;

      v_spread := s_spread(9 downto 7);

      -- Domain distortion: sin(y*freq + time) offsets X, creating wobble
      -- Like shader: p += sin(p.yx*4+t)*0.04
      v_wob1_addr := resize(s_pixel_y(7 downto 2) + s_phase_b(23 downto 18), 6);
      v_wob1 := C_SIN(to_integer(v_wob1_addr));
      -- Second wobble at higher freq: p += sin(p.yx*8+t)*0.01
      v_wob2_addr := resize(s_pixel_y(6 downto 1) + s_phase_c(23 downto 18), 6);
      v_wob2 := C_SIN(to_integer(v_wob2_addr));

      v_dx := (v_wob1 sra 2) + (v_wob2 sra 4);  -- ±32 + ±8 pixels wobble
      v_dy := (v_wob1 sra 3);  -- ±16 pixels vertical wobble

      -- Apply wobble + spread scaling
      if v_dx >= 0 then
        v_sx := (s_pixel_x + resize(unsigned(std_logic_vector(v_dx)), 12)) srl to_integer(v_spread);
      else
        v_sx := (s_pixel_x - resize(unsigned(std_logic_vector(-v_dx)), 12)) srl to_integer(v_spread);
      end if;

      -- Y coord: scrolled upward + wobble
      if v_dy >= 0 then
        v_sy := s_pixel_y + s_phase_a(23 downto 12) + resize(unsigned(std_logic_vector(v_dy)), 12);
      else
        v_sy := s_pixel_y + s_phase_a(23 downto 12) - resize(unsigned(std_logic_vector(-v_dy)), 12);
      end if;

      s2_px <= v_sx;
      s2_py <= v_sy;
    end if;
  end process p_s2;

  -- ========================================================================
  -- S3: Hash at 4 corners for 2 octaves (bilinear noise)
  -- ========================================================================
  p_s3 : process(clk)
    variable v_cx1, v_cy1 : unsigned(7 downto 0);
    variable v_cx2, v_cy2 : unsigned(7 downto 0);
  begin
    if rising_edge(clk) then
      -- Octave 1: coarse grid
      v_cx1 := s2_px(10 downto 3);
      v_cy1 := s2_py(9 downto 2);
      s3_h(0) <= fire_hash(v_cx1,     v_cy1);
      s3_h(1) <= fire_hash(v_cx1 + 1, v_cy1);
      s3_h(2) <= fire_hash(v_cx1,     v_cy1 + 1);
      s3_h(3) <= fire_hash(v_cx1 + 1, v_cy1 + 1);
      s3_fx1 <= s2_px(2 downto 0);
      s3_fy1 <= s2_py(1 downto 0) & "0";

      -- Octave 2: finer grid (2× freq, offset to decorrelate)
      v_cx2 := s2_px(9 downto 2) + s_phase_d(23 downto 16);
      v_cy2 := s2_py(8 downto 1) + to_unsigned(73, 8);
      s3_h(4) <= fire_hash(v_cx2,     v_cy2);
      s3_h(5) <= fire_hash(v_cx2 + 1, v_cy2);
      s3_h(6) <= fire_hash(v_cx2,     v_cy2 + 1);
      s3_h(7) <= fire_hash(v_cx2 + 1, v_cy2 + 1);
      s3_fx2 <= s2_px(1 downto 0) & "0";
      s3_fy2 <= s2_py(0) & "00";

      s3_height_frac <= s2_height_frac;
    end if;
  end process p_s3;

  -- ========================================================================
  -- S4: Bilinear interpolation for 2 octaves, combine → raw noise
  -- ========================================================================
  p_s4 : process(clk)
    -- Octave 1 lerp
    variable v_top1, v_bot1, v_n1 : unsigned(7 downto 0);
    variable v_d1a, v_d1b, v_d1c : signed(8 downto 0);
    -- Octave 2 lerp
    variable v_top2, v_bot2, v_n2 : unsigned(7 downto 0);
    variable v_d2a, v_d2b, v_d2c : signed(8 downto 0);
    -- Combined
    variable v_turb : unsigned(3 downto 0);
    variable v_combined : unsigned(9 downto 0);
  begin
    if rising_edge(clk) then
      -- Bilinear octave 1: mix(mix(h00,h10,fx), mix(h01,h11,fx), fy)
      v_d1a := signed('0' & s3_h(1)) - signed('0' & s3_h(0));
      v_top1 := unsigned(resize(signed('0' & s3_h(0)) + resize((v_d1a * signed('0' & s3_fx1)) sra 3, 9), 8));
      v_d1b := signed('0' & s3_h(3)) - signed('0' & s3_h(2));
      v_bot1 := unsigned(resize(signed('0' & s3_h(2)) + resize((v_d1b * signed('0' & s3_fx1)) sra 3, 9), 8));
      v_d1c := signed('0' & v_bot1) - signed('0' & v_top1);
      v_n1 := unsigned(resize(signed('0' & v_top1) + resize((v_d1c * signed('0' & s3_fy1)) sra 3, 9), 8));

      -- Bilinear octave 2
      v_d2a := signed('0' & s3_h(5)) - signed('0' & s3_h(4));
      v_top2 := unsigned(resize(signed('0' & s3_h(4)) + resize((v_d2a * signed('0' & s3_fx2)) sra 3, 9), 8));
      v_d2b := signed('0' & s3_h(7)) - signed('0' & s3_h(6));
      v_bot2 := unsigned(resize(signed('0' & s3_h(6)) + resize((v_d2b * signed('0' & s3_fx2)) sra 3, 9), 8));
      v_d2c := signed('0' & v_bot2) - signed('0' & v_top2);
      v_n2 := unsigned(resize(signed('0' & v_top2) + resize((v_d2c * signed('0' & s3_fy2)) sra 3, 9), 8));

      -- FBM combine: n1 + n2 * turbulence_weight * 0.47
      v_turb := s_turbulence(9 downto 6);
      v_combined := resize(v_n1, 10) + resize(
        (resize(v_n2 srl 1, 10) * resize(v_turb, 10)) srl 4, 10);
      if v_combined > 255 then v_combined := to_unsigned(255, 10); end if;

      -- Per-pixel LFSR flicker
      if unsigned(s_lfsr_q(3 downto 0)) < 4 then
        v_combined := v_combined + to_unsigned(20, 10);
        if v_combined > 255 then v_combined := to_unsigned(255, 10); end if;
      end if;

      s4_noise <= v_combined(7 downto 0);
      s4_height_frac <= s3_height_frac;
    end if;
  end process p_s4;

  -- ========================================================================
  -- S5: 1/x^4 brightness curve (the key to realistic fire look)
  -- fire_value = noise * height_fraction. Low value = bright core, high = dark edge.
  -- brightness = 1 / (fire_value + epsilon)^4, mapped to 0-1023.
  -- ========================================================================
  p_s5 : process(clk)
    variable v_fire_val : unsigned(15 downto 0);
    variable v_scaled : unsigned(7 downto 0);
    variable v_bright : unsigned(9 downto 0);
    variable v_decay : unsigned(7 downto 0);
  begin
    if rising_edge(clk) then
      -- fire_value = (256 - noise) * (256 - height_frac) / 256
      -- This inverts: high noise + high height = LOW fire_value = BRIGHT
      -- So the hottest parts (bottom, high noise) get the brightest output
      v_fire_val := resize(
        (to_unsigned(256, 9) - resize(s4_noise, 9))
        * (to_unsigned(256, 9) - resize(s4_height_frac, 9)), 16);
      v_scaled := v_fire_val(15 downto 8);

      -- Apply heat (decay) knob
      v_decay := not s_heat(9 downto 2);
      if v_scaled < v_decay then
        v_scaled := (others => '0');
      else
        v_scaled := v_scaled - v_decay;
      end if;

      -- 1/x^4 brightness curve (piecewise approximation)
      -- v_scaled 0 = hottest → max brightness. 255 = coldest → zero.
      -- Sharp curve: most brightness concentrated in 0-40 range.
      if v_scaled < 8 then
        v_bright := to_unsigned(1023, 10);  -- white-hot core
      elsif v_scaled < 16 then
        v_bright := to_unsigned(950, 10);
      elsif v_scaled < 24 then
        v_bright := to_unsigned(850, 10);
      elsif v_scaled < 32 then
        v_bright := to_unsigned(720, 10);
      elsif v_scaled < 48 then
        v_bright := to_unsigned(550, 10);
      elsif v_scaled < 64 then
        v_bright := to_unsigned(380, 10);
      elsif v_scaled < 96 then
        v_bright := to_unsigned(220, 10);
      elsif v_scaled < 128 then
        v_bright := to_unsigned(120, 10);
      elsif v_scaled < 160 then
        v_bright := to_unsigned(60, 10);
      elsif v_scaled < 200 then
        v_bright := to_unsigned(25, 10);
      else
        v_bright := (others => '0');
      end if;

      s5_brightness <= v_bright;
    end if;
  end process p_s5;

  -- ========================================================================
  -- S6: Map brightness to warm fire color (R=1.0, G=0.2, B=0.05 weighting)
  -- Convert weighted RGB to YUV for Videomancer (U/V swapped)
  -- ========================================================================
  p_s6 : process(clk)
    variable v_r, v_g, v_b : unsigned(9 downto 0);
    variable v_y, v_u, v_v : unsigned(9 downto 0);
    variable v_abs_tint : unsigned(8 downto 0);
  begin
    if rising_edge(clk) then
      -- Shader: color = vec3(1, 0.2, 0.05) * brightness
      v_r := s5_brightness;                                          -- R = 1.0 × bright
      v_g := resize(s5_brightness srl 2, 10)
           + resize(s5_brightness srl 5, 10);                        -- G ≈ 0.22 × bright
      v_b := resize(s5_brightness srl 4, 10);                        -- B ≈ 0.06 × bright

      -- RGB to YUV (BT.601, then swap U/V for Videomancer)
      -- Y = 0.299R + 0.587G + 0.114B ≈ R/4 + R/16 + G/2 + G/16 + B/8
      v_y := resize(v_r srl 2, 10) + resize(v_r srl 4, 10)
           + resize(v_g srl 1, 10) + resize(v_g srl 4, 10)
           + resize(v_b srl 3, 10);
      if v_y > 1023 then v_y := to_unsigned(1023, 10); end if;

      -- U (Videomancer = Cr): 0.5*R - 0.419*G - 0.081*B + 512
      -- ≈ R/2 - G/2 + G/16 - B/16 + 512
      v_u := to_unsigned(512, 10)
           + resize(v_r srl 1, 10)
           - resize(v_g srl 1, 10)
           + resize(v_g srl 4, 10);

      -- V (Videomancer = Cb): -0.169*R - 0.331*G + 0.5*B + 512
      -- ≈ -R/8 + R/32 - G/4 - G/16 + B/2 + 512
      v_v := to_unsigned(512, 10)
           - resize(v_r srl 3, 10)
           - resize(v_g srl 2, 10)
           + resize(v_b srl 1, 10);

      -- Blue flame: push chroma toward blue
      if s_blue = '1' then
        v_u := to_unsigned(512, 10) - resize(v_r srl 3, 10);
        v_v := to_unsigned(512, 10) + resize(v_r srl 2, 10);
      end if;

      -- Tint shift
      if s_tint > 0 then
        v_abs_tint := unsigned(std_logic_vector(s_tint(8 downto 0)));
        v_u := v_u + resize(v_abs_tint srl 2, 10);
      elsif s_tint < 0 then
        v_abs_tint := unsigned(std_logic_vector(resize(-s_tint, 9)));
        v_v := v_v + resize(v_abs_tint srl 2, 10);
      end if;

      -- Ember sparkles
      if s_ember = '1' and s5_brightness > 200 and s_lfsr_q(4 downto 0) = "00000" then
        v_y := to_unsigned(1023, 10);
        v_u := to_unsigned(512, 10);
        v_v := to_unsigned(512, 10);
      end if;

      s6_y <= v_y; s6_u <= v_u; s6_v <= v_v;
      s5_br_d1 <= s5_brightness;
    end if;
  end process p_s6;

  -- ========================================================================
  -- S7: Overlay + master brightness
  -- ========================================================================
  p_s7 : process(clk)
    variable v_y, v_u, v_v : unsigned(9 downto 0);
    variable v_bright : unsigned(3 downto 0);
    variable v_alpha : unsigned(7 downto 0);
    variable v_in_y, v_in_u, v_in_v : unsigned(9 downto 0);
  begin
    if rising_edge(clk) then
      v_y := s6_y; v_u := s6_u; v_v := s6_v;

      v_bright := s_intensity(9 downto 6);
      v_y := resize((resize(v_y, 14) * resize(v_bright, 14)) srl 4, 10);

      if s_overlay = '1' then
        -- Use brightness as alpha for overlay
        if s5_br_d1 > 512 then
          v_alpha := to_unsigned(255, 8);
        else
          v_alpha := s5_br_d1(8 downto 1);
        end if;
        v_in_y := unsigned(pipe(C_LATENCY - 2).y);
        v_in_u := unsigned(pipe(C_LATENCY - 2).u);
        v_in_v := unsigned(pipe(C_LATENCY - 2).v);
        if v_alpha(7) = '1' then
          null;  -- mostly fire
        elsif v_alpha(6) = '1' then
          v_y := (v_y srl 1) + (v_in_y srl 1);
          v_u := (v_u srl 1) + (v_in_u srl 1);
          v_v := (v_v srl 1) + (v_in_v srl 1);
        elsif v_alpha(5) = '1' then
          v_y := (v_y srl 2) + v_in_y - (v_in_y srl 2);
          v_u := (v_u srl 2) + v_in_u - (v_in_u srl 2);
          v_v := (v_v srl 2) + v_in_v - (v_in_v srl 2);
        else
          v_y := v_in_y; v_u := v_in_u; v_v := v_in_v;
        end if;
      end if;

      s7_y <= v_y; s7_u <= v_u; s7_v <= v_v;
    end if;
  end process p_s7;

  -- ========================================================================
  -- S8: Output
  -- ========================================================================
  data_out.y <= pipe(C_LATENCY - 1).y when s_bypass = '1' else std_logic_vector(s7_y);
  data_out.u <= pipe(C_LATENCY - 1).u when s_bypass = '1' else std_logic_vector(s7_u);
  data_out.v <= pipe(C_LATENCY - 1).v when s_bypass = '1' else std_logic_vector(s7_v);
  data_out.hsync_n <= pipe(C_LATENCY - 1).hsync_n;
  data_out.vsync_n <= pipe(C_LATENCY - 1).vsync_n;
  data_out.field_n <= pipe(C_LATENCY - 1).field_n;
  data_out.avid    <= pipe(C_LATENCY - 1).avid;

end architecture inferno;
