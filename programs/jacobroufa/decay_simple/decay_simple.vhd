-- Copyright (C) 2025 Jacob Roufa
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Decay Simple - Multi-stage video feedback effect
--
-- Description:
--   Combines multiple independently controllable effects for creative video processing:
--
--   1. FRAME STROBE (0-127 frames)
--      - Temporal posterization: holds entire frames to reduce frame rate
--      - 0 = no strobe (every frame updates), 127 = ~2.1 seconds hold @ 60Hz
--      - Creates stop-motion, stutter, or freeze-frame effects
--      - Examples: 0 = pass-through, 1 = every other frame, 60 @ 60Hz = 1 fps
--
--   2. Y CHANNEL DELAY (0-511 pixels)
--      - Horizontal echo for brightness (Y/luma) channel
--      - Independent enable toggle and delay amount
--      - 0 = no delay (pass-through), 511 = maximum delay
--      - At 720p: 511px = 40% screen width, at 480i: 511px = 71% screen width
--
--   3. U CHANNEL DELAY (0-511 pixels)
--      - Horizontal echo for blue-yellow chroma (U/Cb) channel
--      - Independent enable toggle and delay amount
--      - 0 = no delay (pass-through), 511 = maximum delay
--      - Can differ from Y and V for unique color separation effects
--
--   4. V CHANNEL DELAY (0-511 pixels)
--      - Horizontal echo for red-green chroma (V/Cr) channel
--      - Independent enable toggle and delay amount
--      - 0 = no delay (pass-through), 511 = maximum delay
--      - Can differ from Y and U for complex chromatic aberration
--
--   5. DECAY/BLEND (0-100%)
--      - Feedback: mixes current and delayed pixels
--      - 0% = current only, 50% = equal mix, 100% = delayed only
--      - Controls echo intensity and creates ghost images
--
-- Signal Flow:
--   Input → Frame Strobe → Pixel Delay (Y/U/V independent) → Decay Blend → Output
--
-- Resource Usage: ~12 block RAM blocks (38%) for 512-pixel delay buffers
-- Circular buffer addressing with synchronous RAM access for proper inference
-- Independent Y/U/V delay reads meet 74.25 MHz timing without arithmetic

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture decay_simple of program_top is
    -- Resource allocation for ICE40 HX4K
    -- 512 pixels × 30 bits/pixel × 3 channels = 46,080 bits (~12 of 32 BRAM blocks)
    -- Circular buffer with synchronous reads infers block RAM efficiently
    -- Independent Y/U/V delay addressing avoids arithmetic in critical path
    constant C_PIXEL_DELAY_SIZE : integer := 512;
    constant C_MAX_FRAME_STROBE : integer := 127;  -- Max frames (7-bit range, ~2.1 sec @ 60Hz)
    constant C_BASE_LATENCY     : integer := 9;    -- Base + 1 for synchronous RAM read

    -- Control signals
    signal s_bypass_enable   : std_logic;
    signal s_strobe_enable   : std_logic;
    signal s_y_delay_enable  : std_logic;
    signal s_u_delay_enable  : std_logic;
    signal s_v_delay_enable  : std_logic;
    signal s_effect_amount   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_strobe_frames   : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_y_delay_pixels  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_u_delay_pixels  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_v_delay_pixels  : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_strobe_amount   : integer range 0 to C_MAX_FRAME_STROBE;
    signal s_y_delay_amount  : integer range 0 to C_PIXEL_DELAY_SIZE;
    signal s_u_delay_amount  : integer range 0 to C_PIXEL_DELAY_SIZE;
    signal s_v_delay_amount  : integer range 0 to C_PIXEL_DELAY_SIZE;

    -- Frame strobe effect signals
    -- Holds entire frames for temporal posterization effect
    signal s_frame_counter   : integer range 0 to C_MAX_FRAME_STROBE := 0;
    signal s_frame_update    : std_logic := '1';  -- Signal to capture new frame
    signal s_prev_vsync      : std_logic := '1';
    signal s_strobe_y, s_strobe_u, s_strobe_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_strobe_avid : std_logic;

    -- Pixel delay buffers (circular buffers using block RAM)
    -- Indexed 0 to C_PIXEL_DELAY_SIZE-1 for circular buffer addressing
    -- Synthesis tool will infer block RAM from these arrays with synchronous access
    type t_pixel_delay is array (0 to C_PIXEL_DELAY_SIZE - 1) of unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_pixel_buf_y : t_pixel_delay := (others => (others => '0'));
    signal s_pixel_buf_u : t_pixel_delay := (others => (others => '0'));
    signal s_pixel_buf_v : t_pixel_delay := (others => (others => '0'));

    -- Circular buffer write pointer (increments each pixel)
    signal s_write_ptr : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;

    -- Read address calculation (registered for synchronous RAM access)
    signal s_read_addr_y : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;
    signal s_read_addr_u : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;
    signal s_read_addr_v : integer range 0 to C_PIXEL_DELAY_SIZE - 1 := 0;

    -- Pipeline stages
    signal s_delayed_y, s_current_y : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_delayed_u, s_current_u : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_delayed_v, s_current_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_delayed_avid : std_logic;

    -- Blend outputs
    signal s_blend_y, s_blend_u, s_blend_v : unsigned(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_blend_valid : std_logic;

    -- Bypass delay line
    signal s_hsync_delayed, s_vsync_delayed, s_field_delayed : std_logic;
    signal s_y_bypassed, s_u_bypassed, s_v_bypassed : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

begin
    --------------------------------------------------------------------------------
    -- Register mapping (parameters assigned in TOML order)
    --------------------------------------------------------------------------------
    s_strobe_frames   <= unsigned(registers_in(0));  -- Register 0: Frame strobe amount
    s_y_delay_pixels  <= unsigned(registers_in(1));  -- Register 1: Y channel delay
    s_u_delay_pixels  <= unsigned(registers_in(2));  -- Register 2: U channel delay
    s_v_delay_pixels  <= unsigned(registers_in(3));  -- Register 3: V channel delay
    s_strobe_enable   <= registers_in(4)(0);         -- Bit 0: Enable frame strobe
    s_y_delay_enable  <= registers_in(4)(1);         -- Bit 1: Enable Y delay
    s_u_delay_enable  <= registers_in(4)(2);         -- Bit 2: Enable U delay
    s_v_delay_enable  <= registers_in(4)(3);         -- Bit 3: Enable V delay
    s_bypass_enable   <= registers_in(4)(4);         -- Bit 4: Bypass all processing
    s_effect_amount   <= unsigned(registers_in(5));  -- Register 5: Decay/blend amount

    -- Map 10-bit register values to control ranges using bit slicing (no arithmetic)
    -- Strobe: bits [9:3] → 0-127 frames (7 bits, ~2.1 sec @ 60Hz)
    -- Y/U/V delays: bits [9:1] → 0-511 pixels each (9 bits)
    -- All use bit slicing for timing-safe implementation
    s_strobe_amount  <= to_integer(s_strobe_frames(9 downto 3));
    s_y_delay_amount <= to_integer(s_y_delay_pixels(9 downto 1));
    s_u_delay_amount <= to_integer(s_u_delay_pixels(9 downto 1));
    s_v_delay_amount <= to_integer(s_v_delay_pixels(9 downto 1));

    --------------------------------------------------------------------------------
    -- Frame Strobe Effect
    --
    -- Purpose: Creates temporal posterization by holding entire video frames
    -- Operation:
    --   - Counts frame boundaries using vsync_n signal
    --   - Amount = 0: updates every frame (pass-through, no strobe)
    --   - Amount > 0: holds frame for N frames, then captures next
    --   - Effect: reduces effective frame rate (e.g., 60fps → 30fps, 20fps, etc.)
    --
    -- Visual Result:
    --   - 0 frames: no effect (pass-through)
    --   - Low values (1-5 frames): subtle judder/stutter
    --   - Medium values (10-30 frames): stop-motion animation effect
    --   - High values (60-127 frames): freeze-frame that updates periodically
    --
    -- Processing Order: Runs BEFORE pixel delay to reduce processing load
    --------------------------------------------------------------------------------
    p_frame_strobe : process(clk)
    begin
        if rising_edge(clk) then
            -- Track vsync falling edge to detect new frame start
            s_prev_vsync <= data_in.vsync_n;

            -- Detect frame boundary (vsync falling edge: '1' → '0')
            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                if s_strobe_enable = '1' and s_strobe_amount > 0 then
                    -- Frame strobe enabled with amount > 0: count frames and determine update
                    if s_frame_counter >= s_strobe_amount then
                        s_frame_counter <= 0;
                        s_frame_update <= '1';  -- Capture this frame
                    else
                        s_frame_counter <= s_frame_counter + 1;
                        s_frame_update <= '0';  -- Hold previous frame
                    end if;
                else
                    -- Frame strobe disabled or amount = 0: always update (pass through)
                    s_frame_counter <= 0;
                    s_frame_update <= '1';
                end if;
            end if;

            -- Pixel capture: update or hold based on frame_update flag
            if data_in.avid = '1' then
                if s_frame_update = '1' then
                    -- Capture new pixels from this frame
                    s_strobe_y <= unsigned(data_in.y);
                    s_strobe_u <= unsigned(data_in.u);
                    s_strobe_v <= unsigned(data_in.v);
                else
                    -- Hold pixels from previous captured frame
                    -- (s_strobe_y/u/v retain their values)
                end if;
            end if;

            -- Pass through timing signals
            s_strobe_avid <= data_in.avid;
        end if;
    end process p_frame_strobe;

    --------------------------------------------------------------------------------
    -- Pixel Delay Effect with Full Y/U/V Independent Control
    --
    -- Purpose: Creates horizontal trailing/echo by delaying pixel data
    -- Operation:
    --   - Maintains circular buffers in block RAM (Y, U, V channels)
    --   - Uses write pointer that increments each pixel
    --   - Calculates read addresses: (write_ptr - delay_amount) mod buffer_size
    --   - Y channel: independent enable + delay amount (0-511 pixels)
    --   - U channel: independent enable + delay amount (0-511 pixels)
    --   - V channel: independent enable + delay amount (0-511 pixels)
    --   - Synchronous RAM reads for proper block RAM inference
    --   - When disabled or delay = 0: passes through current pixel
    --   - Feeds both current and delayed pixels to blend stage
    --
    -- Visual Result:
    --   - Creates horizontal displacement/echo effect per channel
    --   - At 720p (1280px wide): 511px = 40% screen width (dramatic echoes)
    --   - At 480i (720px wide): 511px = 71% screen width (extreme displacement)
    --   - Full independent control enables complex chromatic aberration effects
    --   - Can delay Y, U, V by different amounts for creative color separation
    --
    -- Processing Order: Runs AFTER frame strobe
    --   - Benefit: If strobe holds frames, delay processes fewer unique pixels
    --   - Example: 30fps strobe @ 60Hz = 50% less pixel processing
    --
    -- Resource Usage: 512 pixels × 30 bits/pixel = 15,360 bits block RAM (~12 blocks)
    --------------------------------------------------------------------------------
    p_pixel_delay_write : process(clk)
    begin
        if rising_edge(clk) then
            if s_strobe_avid = '1' then
                -- Write current pixel to circular buffer at write pointer
                s_pixel_buf_y(s_write_ptr) <= s_strobe_y;
                s_pixel_buf_u(s_write_ptr) <= s_strobe_u;
                s_pixel_buf_v(s_write_ptr) <= s_strobe_v;

                -- Increment write pointer (circular)
                if s_write_ptr = C_PIXEL_DELAY_SIZE - 1 then
                    s_write_ptr <= 0;
                else
                    s_write_ptr <= s_write_ptr + 1;
                end if;

                -- Calculate read addresses for each channel (circular buffer math)
                -- read_addr = (write_ptr - delay_amount + buffer_size) mod buffer_size
                -- Adding buffer_size prevents negative results before modulo
                if s_y_delay_enable = '1' and s_y_delay_amount > 0 then
                    s_read_addr_y <= (s_write_ptr - s_y_delay_amount + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_read_addr_y <= s_write_ptr;  -- Read current position (no delay)
                end if;

                if s_u_delay_enable = '1' and s_u_delay_amount > 0 then
                    s_read_addr_u <= (s_write_ptr - s_u_delay_amount + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_read_addr_u <= s_write_ptr;  -- Read current position (no delay)
                end if;

                if s_v_delay_enable = '1' and s_v_delay_amount > 0 then
                    s_read_addr_v <= (s_write_ptr - s_v_delay_amount + C_PIXEL_DELAY_SIZE) mod C_PIXEL_DELAY_SIZE;
                else
                    s_read_addr_v <= s_write_ptr;  -- Read current position (no delay)
                end if;

                -- Store current strobed pixel for blending
                s_current_y <= s_strobe_y;
                s_current_u <= s_strobe_u;
                s_current_v <= s_strobe_v;
            end if;

            s_delayed_avid <= s_strobe_avid;
        end if;
    end process p_pixel_delay_write;

    -- Synchronous RAM read (separate process for proper block RAM inference)
    -- Read addresses were calculated in previous clock cycle
    p_pixel_delay_read : process(clk)
    begin
        if rising_edge(clk) then
            if s_strobe_avid = '1' then
                -- Synchronous read from block RAM buffers
                s_delayed_y <= s_pixel_buf_y(s_read_addr_y);
                s_delayed_u <= s_pixel_buf_u(s_read_addr_u);
                s_delayed_v <= s_pixel_buf_v(s_read_addr_v);
            end if;
        end if;
    end process p_pixel_delay_read;

    --------------------------------------------------------------------------------
    -- Decay/Blend Effect (using hardware interpolators)
    --
    -- Purpose: Blends current pixels with delayed pixels for feedback effect
    -- Operation:
    --   - Interpolates between current pixel (a) and delayed pixel (b)
    --   - Effect amount (t) controls blend ratio: lerp(a, b, t) = a + (b-a)*t
    --   - Separate interpolators for Y, U, V channels (color-correct blending)
    --
    -- Control Range:
    --   - Effect amount = 0%: output = current pixel only (no echo/delay visible)
    --   - Effect amount = 50%: output = equal mix of current and delayed
    --   - Effect amount = 100%: output = delayed pixel only (full echo effect)
    --
    -- Visual Results (effect combinations):
    --   - Decay alone: subtle to strong feedback loop
    --   - Decay + Strobe: stuttering ghost images with temporal posterization
    --   - Decay + Y Delay: brightness echo trails
    --   - Decay + U/V Delay: color-separated echoes (chromatic aberration)
    --   - All effects: complex spatio-temporal effects with per-channel control
    --
    -- Creative Examples (Y/U/V delay combinations):
    --   - Y=0, U=0, V=0: no delay (pure decay/feedback effect)
    --   - Y=511, U=511, V=511: synchronized echo (all channels delayed equally)
    --   - Y=511, U=300, V=100: progressive color separation (V leads U leads Y)
    --   - Y=0, U=511, V=511: color echo only (brightness stays current)
    --   - Y=150, U=0, V=450: unique V channel lag (red-green trails)
    --   - Y=300, U=100, V=511: asymmetric color separation (blue leads, red lags)
    --   - Full independent per-channel control enables unique glitch aesthetics
    --------------------------------------------------------------------------------
    interpolator_y : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_delayed_avid,
            a      => s_current_y,
            b      => s_delayed_y,
            t      => s_effect_amount,
            result => s_blend_y,
            valid  => s_blend_valid
        );

    interpolator_u : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_delayed_avid,
            a      => s_current_u,
            b      => s_delayed_u,
            t      => s_effect_amount,
            result => s_blend_u,
            valid  => open
        );

    interpolator_v : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_VIDEO_DATA_WIDTH,
            G_FRAC_BITS  => C_VIDEO_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_delayed_avid,
            a      => s_current_v,
            b      => s_delayed_v,
            t      => s_effect_amount,
            result => s_blend_v,
            valid  => open
        );

    --------------------------------------------------------------------------------
    -- Bypass delay line (matches processing latency)
    --------------------------------------------------------------------------------
    p_bypass_delay : process(clk)
        type t_sync_delay is array (0 to C_BASE_LATENCY - 1) of std_logic;
        type t_data_delay is array (0 to C_BASE_LATENCY - 1) of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);

        variable v_hsync : t_sync_delay := (others => '1');
        variable v_vsync : t_sync_delay := (others => '1');
        variable v_field : t_sync_delay := (others => '1');
        variable v_y     : t_data_delay := (others => (others => '0'));
        variable v_u     : t_data_delay := (others => (others => '0'));
        variable v_v     : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            -- Shift delay lines
            v_hsync := data_in.hsync_n & v_hsync(0 to C_BASE_LATENCY - 2);
            v_vsync := data_in.vsync_n & v_vsync(0 to C_BASE_LATENCY - 2);
            v_field := data_in.field_n & v_field(0 to C_BASE_LATENCY - 2);
            v_y     := data_in.y       & v_y(0 to C_BASE_LATENCY - 2);
            v_u     := data_in.u       & v_u(0 to C_BASE_LATENCY - 2);
            v_v     := data_in.v       & v_v(0 to C_BASE_LATENCY - 2);

            -- Output delayed signals
            s_hsync_delayed <= v_hsync(C_BASE_LATENCY - 1);
            s_vsync_delayed <= v_vsync(C_BASE_LATENCY - 1);
            s_field_delayed <= v_field(C_BASE_LATENCY - 1);
            s_y_bypassed    <= v_y(C_BASE_LATENCY - 1);
            s_u_bypassed    <= v_u(C_BASE_LATENCY - 1);
            s_v_bypassed    <= v_v(C_BASE_LATENCY - 1);
        end if;
    end process p_bypass_delay;

    --------------------------------------------------------------------------------
    -- Output multiplexing
    --------------------------------------------------------------------------------
    data_out.y <= std_logic_vector(s_blend_y) when s_bypass_enable = '0' else s_y_bypassed;
    data_out.u <= std_logic_vector(s_blend_u) when s_bypass_enable = '0' else s_u_bypassed;
    data_out.v <= std_logic_vector(s_blend_v) when s_bypass_enable = '0' else s_v_bypassed;

    data_out.avid    <= s_blend_valid;
    data_out.hsync_n <= s_hsync_delayed;
    data_out.vsync_n <= s_vsync_delayed;
    data_out.field_n <= s_field_delayed;

end architecture decay_simple;
