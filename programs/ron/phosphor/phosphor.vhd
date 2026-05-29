-- phosphor.vhd
--
-- Edge detection with rainbow band-trail to the right of each edge.
--
-- At each detected edge the output resets to RED; subsequent pixels on the
-- same scanline step through ORANGE, YELLOW, GREEN, BLUE, INDIGO, VIOLET,
-- then black, each band occupying `band_width` pixels (knob 1). Pipeline
-- resets at hsync so trails never cross lines.
--
-- Edge detect : horizontal gradient |Y[x] - Y[x-1]| > knob 2 threshold.
-- Color path  : 3-bit band index → 8-entry YUV LUT (7 rainbow + black).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture phosphor of program_top is

    constant C_SYNC_DELAY : integer := 11;

    -- 8-entry rainbow palette in 10-bit YUV, BT.601 limited range, with U/V
    -- swapped for Videomancer hardware (SDK convention). The C_PAL_U constant
    -- holds what BT.601 calls Cr, and C_PAL_V holds Cb.
    -- Index 7 is black (past violet).
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(328, 10),  -- Red    (255,   0,   0)
        to_unsigned(584, 10),  -- Orange (255, 128,   0)
        to_unsigned(840, 10),  -- Yellow (255, 255,   0)
        to_unsigned(580, 10),  -- Green  (  0, 255,   0)
        to_unsigned(164, 10),  -- Blue   (  0,   0, 255)
        to_unsigned(192, 10),  -- Indigo ( 75,   0, 130)
        to_unsigned(349, 10),  -- Violet (180,   0, 255)
        to_unsigned(  0, 10)); -- Black
    constant C_PAL_U : t_pal := (
        to_unsigned(960, 10),  -- Red
        to_unsigned(772, 10),  -- Orange
        to_unsigned(584, 10),  -- Yellow
        to_unsigned(136, 10),  -- Green
        to_unsigned(440, 10),  -- Blue
        to_unsigned(608, 10),  -- Indigo
        to_unsigned(756, 10),  -- Violet
        to_unsigned(512, 10)); -- Black
    constant C_PAL_V : t_pal := (
        to_unsigned(360, 10),  -- Red
        to_unsigned(212, 10),  -- Orange
        to_unsigned( 64, 10),  -- Yellow
        to_unsigned(216, 10),  -- Green
        to_unsigned(960, 10),  -- Blue
        to_unsigned(696, 10),  -- Indigo
        to_unsigned(852, 10),  -- Violet
        to_unsigned(512, 10)); -- Black

    -- Parameters
    signal s_knob_width      : unsigned(9 downto 0);
    signal s_knob_thresh     : unsigned(9 downto 0);
    signal s_knob_noise      : unsigned(9 downto 0);
    signal s_knob_poster     : unsigned(9 downto 0);
    signal s_knob_contrast   : unsigned(9 downto 0);
    signal s_knob_curve      : unsigned(9 downto 0);
    signal s_poster_drop     : unsigned(2 downto 0);  -- 0..7 low bits to drop
    signal s_curve_step      : unsigned(4 downto 0);  -- per-band pixel reduction
    signal s_sw_video_key    : std_logic;
    signal s_sw_curve_en     : std_logic;
    signal s_band_width      : unsigned(6 downto 0);  -- 1..128
    signal s_min_run         : unsigned(4 downto 0);  -- 1..16
    signal s_contrast_gain   : unsigned(8 downto 0);  -- 128..255 (1.0x..~2.0x in 7-bit fixed)

    -- Position tracking (just for hsync reset)
    signal s_prev_hsync_n : std_logic := '1';

    -- Effective edge threshold: top 5 bits of knob → 0..31 range so the full
    -- knob rotation covers the usable smoothed-gradient range.
    signal s_effective_thresh : unsigned(6 downto 0);

    -- 4-tap shift registers and smoothed values for all three channels.
    signal s_y_reg0, s_y_reg1, s_y_reg2, s_y_reg3 : unsigned(9 downto 0) := (others => '0');
    signal s_u_reg0, s_u_reg1, s_u_reg2, s_u_reg3 : unsigned(9 downto 0) := (others => '0');
    signal s_v_reg0, s_v_reg1, s_v_reg2, s_v_reg3 : unsigned(9 downto 0) := (others => '0');
    signal s_y_smooth, s_y_smooth_prev : unsigned(9 downto 0) := (others => '0');
    signal s_u_smooth, s_u_smooth_prev : unsigned(9 downto 0) := (others => '0');
    signal s_v_smooth, s_v_smooth_prev : unsigned(9 downto 0) := (others => '0');
    signal s_edge_bit  : std_logic := '0';
    signal s_avid_d1   : std_logic := '0';
    signal s_run_count : unsigned(4 downto 0) := (others => '0');

    -- Contrast pre-processing pipeline (Y channel, 2 register stages)
    signal s_y_cont_prod : signed(19 downto 0) := (others => '0');
    signal s_y_cont      : unsigned(9 downto 0) := (others => '0');

    -- U/V alignment delay: keep chroma in step with Y, which now goes through
    -- posterize (+1) and contrast (+2) = 3 extra stages vs a direct LPF→edge path.
    signal s_u_dly1, s_u_dly2, s_u_dly3 : unsigned(9 downto 0) := (others => '0');
    signal s_v_dly1, s_v_dly2, s_v_dly3 : unsigned(9 downto 0) := (others => '0');

    -- Delayed incoming Y/U/V for video key-through (needs full SYNC_DELAY alignment)
    type t_yuv_delay is array (0 to C_SYNC_DELAY - 1) of std_logic_vector(9 downto 0);
    signal s_y_in_sr : t_yuv_delay := (others => (others => '0'));
    signal s_u_in_sr : t_yuv_delay := (others => (others => '0'));
    signal s_v_in_sr : t_yuv_delay := (others => (others => '0'));

    -- Per-channel abs-diff gradients (registered to break critical path)
    signal s_y_grad : unsigned(9 downto 0) := (others => '0');
    signal s_u_grad : unsigned(9 downto 0) := (others => '0');
    signal s_v_grad : unsigned(9 downto 0) := (others => '0');

    -- Input 2-tap low-pass pre-filter: smooths data_in before contrast/saturation
    -- so the multipliers see denoised values. Convolved with the downstream
    -- 4-tap smoothing, this gives an effective 5-tap triangular kernel.
    signal s_y_in_d1 : unsigned(9 downto 0) := (others => '0');
    signal s_u_in_d1 : unsigned(9 downto 0) := (others => '0');
    signal s_v_in_d1 : unsigned(9 downto 0) := (others => '0');
    signal s_y_lpf   : unsigned(9 downto 0) := (others => '0');
    signal s_u_lpf   : unsigned(9 downto 0) := (others => '0');
    signal s_v_lpf   : unsigned(9 downto 0) := (others => '0');

    -- Posterized Y (luma quantized to 2^(10-drop) levels)
    signal s_y_post : unsigned(9 downto 0) := (others => '0');

    -- Rainbow band state
    signal s_band_pixel : unsigned(6 downto 0) := (others => '0');  -- 0..band_width-1
    signal s_band_idx   : unsigned(2 downto 0) := "111";            -- 7 = black at line start
    signal s_band_size  : unsigned(7 downto 0) := to_unsigned(128, 8);  -- effective curve-adjusted width

    -- Sync shift register
    signal s_avid_sr    : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');
    signal s_field_n_sr : std_logic_vector(C_SYNC_DELAY - 1 downto 0) := (others => '1');

begin

    s_knob_width      <= unsigned(registers_in(0));
    s_knob_thresh     <= unsigned(registers_in(1));
    s_knob_noise      <= unsigned(registers_in(2));
    s_knob_poster   <= unsigned(registers_in(3));
    s_knob_contrast <= unsigned(registers_in(4));
    s_knob_curve    <= unsigned(registers_in(5));
    s_sw_video_key  <= registers_in(6)(0);
    s_sw_curve_en   <= registers_in(6)(1);

    -- Band width: knob top 7 bits + 1 gives 1..128 pixels
    s_band_width <= unsigned(s_knob_width(9 downto 3)) + to_unsigned(1, 7);
    -- Min run length: knob top 4 bits + 1 gives 1..16 pixels
    s_min_run <= resize(unsigned(s_knob_noise(9 downto 6)) + to_unsigned(1, 5), 5);
    -- Edge threshold: combined |ΔY|+|ΔU|+|ΔV| has ~3x the range of Y alone,
    -- so map top 7 bits → 0..127 to cover the full useful range.
    s_effective_thresh <= resize(s_knob_thresh(9 downto 3), s_effective_thresh'length);
    -- Contrast gain: 7-bit fixed-point (128 = 1.0x). 128 + knob(9:3) = 128..255.
    s_contrast_gain <= resize(unsigned(s_knob_contrast(9 downto 3)), 9) + to_unsigned(128, 9);
    -- Curve step: per-band pixel reduction. knob(9:5) = 0..31. Band idx × step
    -- subtracted from base band_width, clamped to >= 1.
    s_curve_step <= s_knob_curve(9 downto 5);
    -- Posterize drop-bits: knob top 3 bits → 0..7. 0 = no quantization,
    -- 7 = 8 luma levels.
    s_poster_drop <= s_knob_poster(9 downto 7);

    ----------------------------------------------------------------------------
    -- Input 2-tap low-pass: (data_in + data_in_prev) / 2. One register stage.
    -- Runs before contrast/saturation so the multipliers operate on denoised
    -- values.
    ----------------------------------------------------------------------------
    p_input_lpf : process(clk)
    begin
        if rising_edge(clk) then
            s_y_in_d1 <= unsigned(data_in.y);
            s_u_in_d1 <= unsigned(data_in.u);
            s_v_in_d1 <= unsigned(data_in.v);

            s_y_lpf <= resize(shift_right(
                resize(unsigned(data_in.y), 11) + resize(s_y_in_d1, 11), 1), 10);
            s_u_lpf <= resize(shift_right(
                resize(unsigned(data_in.u), 11) + resize(s_u_in_d1, 11), 1), 10);
            s_v_lpf <= resize(shift_right(
                resize(unsigned(data_in.v), 11) + resize(s_v_in_d1, 11), 1), 10);
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Luma posterize: mask off the low s_poster_drop bits of Y so small
    -- gradients inside a level disappear and boundaries between levels are
    -- sharp. Runs on the LPF output, feeds the contrast multiplier.
    ----------------------------------------------------------------------------
    p_posterize : process(clk)
    begin
        if rising_edge(clk) then
            case to_integer(s_poster_drop) is
                when 0      => s_y_post <= s_y_lpf;
                when 1      => s_y_post <= s_y_lpf and "1111111110";
                when 2      => s_y_post <= s_y_lpf and "1111111100";
                when 3      => s_y_post <= s_y_lpf and "1111111000";
                when 4      => s_y_post <= s_y_lpf and "1111110000";
                when 5      => s_y_post <= s_y_lpf and "1111100000";
                when 6      => s_y_post <= s_y_lpf and "1111000000";
                when others => s_y_post <= s_y_lpf and "1110000000";
            end case;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Contrast pre-processing: Y_out = 512 + (Y_in - 512) * gain / 128.
    -- Two register stages: (A) compute diff and multiply, (B) shift/add/clamp.
    ----------------------------------------------------------------------------
    p_contrast : process(clk)
        variable v_y_diff : signed(10 downto 0);
        variable v_y_sum  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- Stage A: difference from center, multiply by gain (uses posterized Y)
            v_y_diff := signed('0' & std_logic_vector(s_y_post)) - to_signed(512, 11);
            s_y_cont_prod <= v_y_diff * signed(std_logic_vector(s_contrast_gain));

            -- Stage B: shift back, recenter, clamp (uses pre-edge prod from A)
            v_y_sum := resize(shift_right(s_y_cont_prod, 7), 14) + to_signed(512, 14);
            if v_y_sum < 0 then
                s_y_cont <= (others => '0');
            elsif v_y_sum > 1023 then
                s_y_cont <= (others => '1');
            else
                s_y_cont <= unsigned(std_logic_vector(v_y_sum(9 downto 0)));
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Edge detection on contrast+saturation pre-processed Y, U, V, with 4-tap
    -- smoothing on each channel and a combined |ΔY|+|ΔU|+|ΔV| gradient.
    --
    -- 1. Three 4-deep shift registers → per-channel 4-tap moving average.
    -- 2. Gradient per channel = |smooth - smooth_prev|; combined = sum.
    -- 3. min_run: combined gradient must exceed threshold for s_min_run
    --    consecutive pixels before an edge fires.
    ----------------------------------------------------------------------------
    p_edge : process(clk)
        variable v_y_sum, v_u_sum, v_v_sum : unsigned(11 downto 0);
        variable v_combined_grad : unsigned(11 downto 0);
        variable v_grad_high : std_logic;
    begin
        if rising_edge(clk) then
            -- Y / U / V 4-tap shift registers
            s_y_reg0 <= s_y_cont;
            s_y_reg1 <= s_y_reg0;
            s_y_reg2 <= s_y_reg1;
            s_y_reg3 <= s_y_reg2;

            s_u_reg0 <= s_u_dly3;
            s_u_reg1 <= s_u_reg0;
            s_u_reg2 <= s_u_reg1;
            s_u_reg3 <= s_u_reg2;

            s_v_reg0 <= s_v_dly3;
            s_v_reg1 <= s_v_reg0;
            s_v_reg2 <= s_v_reg1;
            s_v_reg3 <= s_v_reg2;

            -- Moving-average sums (use pre-edge register values)
            v_y_sum := resize(s_y_reg0, 12) + resize(s_y_reg1, 12)
                     + resize(s_y_reg2, 12) + resize(s_y_reg3, 12);
            v_u_sum := resize(s_u_reg0, 12) + resize(s_u_reg1, 12)
                     + resize(s_u_reg2, 12) + resize(s_u_reg3, 12);
            v_v_sum := resize(s_v_reg0, 12) + resize(s_v_reg1, 12)
                     + resize(s_v_reg2, 12) + resize(s_v_reg3, 12);

            s_y_smooth      <= resize(shift_right(v_y_sum, 2), 10);
            s_u_smooth      <= resize(shift_right(v_u_sum, 2), 10);
            s_v_smooth      <= resize(shift_right(v_v_sum, 2), 10);
            s_y_smooth_prev <= s_y_smooth;
            s_u_smooth_prev <= s_u_smooth;
            s_v_smooth_prev <= s_v_smooth;

            s_avid_d1 <= data_in.avid;

            -- Per-channel abs-diff gradients, REGISTERED to break the long
            -- combinational path (3 subtracts + sum + compare + min-run).
            if s_y_smooth >= s_y_smooth_prev then
                s_y_grad <= s_y_smooth - s_y_smooth_prev;
            else
                s_y_grad <= s_y_smooth_prev - s_y_smooth;
            end if;
            if s_u_smooth >= s_u_smooth_prev then
                s_u_grad <= s_u_smooth - s_u_smooth_prev;
            else
                s_u_grad <= s_u_smooth_prev - s_u_smooth;
            end if;
            if s_v_smooth >= s_v_smooth_prev then
                s_v_grad <= s_v_smooth - s_v_smooth_prev;
            else
                s_v_grad <= s_v_smooth_prev - s_v_smooth;
            end if;

            -- Combine + threshold + min-run (uses pre-edge s_*_grad values
            -- from the previous cycle)
            v_combined_grad := resize(s_y_grad, 12) + resize(s_u_grad, 12)
                             + resize(s_v_grad, 12);

            if v_combined_grad > resize(s_effective_thresh, 12) then
                v_grad_high := '1';
            else
                v_grad_high := '0';
            end if;

            if v_grad_high = '1' then
                if s_run_count = s_min_run - 1 then
                    s_edge_bit <= '1';
                else
                    s_edge_bit <= '0';
                end if;
                if s_run_count < 31 then
                    s_run_count <= s_run_count + 1;
                end if;
            else
                s_run_count <= (others => '0');
                s_edge_bit <= '0';
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Rainbow band state machine:
    --   edge        → reset to red (band 0, pixel 0)
    --   within band → increment band_pixel
    --   end of band → advance band_idx (saturates at 7 = black)
    --   hsync edge  → back to black (idx 7)
    --
    -- With curve enabled (switch 8), each band's width = band_width - idx*step,
    -- clamped to 1 pixel minimum. Red stays at full width, colors taper down.
    ----------------------------------------------------------------------------
    ----------------------------------------------------------------------------
    -- Curve: compute the effective band width for the current band_idx.
    -- Registered so the multiply + subtract + compare is out of the p_band
    -- critical path. One cycle of lag on band-size updates is invisible.
    ----------------------------------------------------------------------------
    p_curve : process(clk)
        variable v_decrement : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if s_sw_curve_en = '1' then
                v_decrement := s_band_idx * s_curve_step;  -- 3b × 5b → 8b
                if resize(s_band_width, 8) > v_decrement then
                    s_band_size <= resize(s_band_width, 8) - v_decrement;
                else
                    s_band_size <= to_unsigned(1, 8);
                end if;
            else
                s_band_size <= resize(s_band_width, 8);
            end if;
        end if;
    end process;

    p_band : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_band_idx   <= "111";
                s_band_pixel <= (others => '0');
            elsif s_avid_d1 = '1' then
                if s_edge_bit = '1' then
                    s_band_idx   <= (others => '0');
                    s_band_pixel <= (others => '0');
                elsif resize(s_band_pixel, 8) = s_band_size - 1 then
                    s_band_pixel <= (others => '0');
                    if s_band_idx /= "111" then
                        s_band_idx <= s_band_idx + 1;
                    end if;
                else
                    s_band_pixel <= s_band_pixel + 1;
                end if;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- U/V alignment delay: 3 register stages so chroma arrives at the shift
    -- register at the same pipeline position as Y (which goes through
    -- posterize + contrast A + contrast B = 3 stages after LPF).
    ----------------------------------------------------------------------------
    p_uv_align : process(clk)
    begin
        if rising_edge(clk) then
            s_u_dly1 <= s_u_lpf;
            s_u_dly2 <= s_u_dly1;
            s_u_dly3 <= s_u_dly2;
            s_v_dly1 <= s_v_lpf;
            s_v_dly2 <= s_v_dly1;
            s_v_dly3 <= s_v_dly2;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Video-key delay: incoming Y/U/V delayed by full SYNC_DELAY so it lines
    -- up with data_out. Used when switch 7 is on and the band index falls
    -- past violet — the "black" region is replaced with pass-through video.
    ----------------------------------------------------------------------------
    p_video_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_y_in_sr(0) <= data_in.y;
            s_u_in_sr(0) <= data_in.u;
            s_v_in_sr(0) <= data_in.v;
            for i in 1 to C_SYNC_DELAY - 1 loop
                s_y_in_sr(i) <= s_y_in_sr(i - 1);
                s_u_in_sr(i) <= s_u_in_sr(i - 1);
                s_v_in_sr(i) <= s_v_in_sr(i - 1);
            end loop;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Sync delay: 8 stages — contrast+sat (2 parallel) + smoothing (2) +
    -- smooth_prev/gradient/edge (2) + band (1) + palette (combinational + 1)
    ----------------------------------------------------------------------------
    p_sync_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= s_avid_sr   (C_SYNC_DELAY - 2 downto 0) & data_in.avid;
            s_hsync_n_sr <= s_hsync_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.hsync_n;
            s_vsync_n_sr <= s_vsync_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.vsync_n;
            s_field_n_sr <= s_field_n_sr(C_SYNC_DELAY - 2 downto 0) & data_in.field_n;
        end if;
    end process;

    -- Output mux: if video-key enabled AND band_idx is past-violet (black region),
    -- pass delayed incoming video instead of black palette entry.
    data_out.y <= s_y_in_sr(C_SYNC_DELAY - 1)
                      when s_sw_video_key = '1' and s_band_idx = "111"
                      else std_logic_vector(C_PAL_Y(to_integer(s_band_idx)));
    data_out.u <= s_u_in_sr(C_SYNC_DELAY - 1)
                      when s_sw_video_key = '1' and s_band_idx = "111"
                      else std_logic_vector(C_PAL_U(to_integer(s_band_idx)));
    data_out.v <= s_v_in_sr(C_SYNC_DELAY - 1)
                      when s_sw_video_key = '1' and s_band_idx = "111"
                      else std_logic_vector(C_PAL_V(to_integer(s_band_idx)));
    data_out.avid    <= s_avid_sr   (C_SYNC_DELAY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_SYNC_DELAY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_SYNC_DELAY - 1);
    data_out.field_n <= s_field_n_sr(C_SYNC_DELAY - 1);

end phosphor;
