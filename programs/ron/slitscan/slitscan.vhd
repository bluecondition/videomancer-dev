-- Slit Scan: temporal slit-scan video processor.
--
-- Divides the frame into N discrete slices (horizontal bands in H-mode,
-- vertical columns in V-mode).  Each slice is offset in time by a
-- different amount via the variable_delay_u buffer, so the image appears
-- sheared across time.  The number of slices is selectable (2/4/8/16/32).
--
-- Pipeline (rising-edge clocks from data_in to data_out):
--   T1  p_input_stage:     register inputs, advance x/y counters, shift bypass SR
--   T2  p_slice_delay:     normalise position, quantise to slice boundary,
--                          compute per-slice delay, luma*luma_mod pre-multiply
--   T3  p_delay_assemble:  fold, reverse, offset + luma contrib, UV diverge,
--                          clamp → s_delay_y/u/v presented to variable_delay_u
--   T4  variable_delay_u address-gen stage
--   T5  variable_delay_u BRAM read → s_smear_y/u/v valid
--   T6  p_proc_align:      alignment register before interpolator
--   T7-T10  interpolator_u (4-clock latency) per channel
--   T10 data_out
--
-- BRAM budget (iCE40 HX4K, 32 × 4 Kbit EBR):
--   3 × variable_delay_u(G_WIDTH=10, G_DEPTH=11)  ≈ 3 × 20 Kbit = 60 Kbit
--   128 Kbit total available; ~68 Kbit headroom.
--
-- Register map:
--   registers_in(0) = Delay      (0-1023, ×2 → 0-2046 samples into past)
--   registers_in(1) = Chroma Lag (0-1023 extra samples on U/V)
--   registers_in(2) = Slices     (7 zones: 2/4/8/16/32/64/128 discrete bands)
--   registers_in(3) = Offset     (0-1023, base delay added to all channels)
--   registers_in(4) = Luma Mod   (0-1023, input luma scales additive delay)
--   registers_in(5) = UV Diverge (512=equal; <512 U leads V; >512 V leads U)
--   registers_in(6) = Switches
--                       bit 0 = Axis    (0=Horizontal, 1=Vertical)
--                       bit 1 = Reverse (0=normal, 1=flip now/past ends)
--                       bit 2 = Fold    (0=ramp, 1=V-shape: centre=present)
--                       bit 3 = Pattern (0=Random: bit-reversed scatter,
--                                        1=Sine:   half-sine-wave bump)
--                       bit 4 = Bypass  (clean latency-matched passthrough)
--   registers_in(7) = Mix        (0=dry, 1023=full wet)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture slitscan of program_top is

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DATA_WIDTH    : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_DELAY_BITS    : integer := 11;                  -- 2048-deep buffers
    constant C_TOTAL_LATENCY : integer := 10;

    -- Half-sine LUT: 32 entries, 5-bit output.  Values trace sin(k*pi/31)*31
    -- for k = 0..31.  Bump shape: 0 at edges, 31 at centre (entries 15-16).
    type t_sin_lut is array(0 to 31) of unsigned(4 downto 0);
    constant C_SINE_LUT : t_sin_lut := (
        to_unsigned( 0, 5), to_unsigned( 3, 5), to_unsigned( 6, 5), to_unsigned( 9, 5),
        to_unsigned(12, 5), to_unsigned(15, 5), to_unsigned(18, 5), to_unsigned(20, 5),
        to_unsigned(22, 5), to_unsigned(24, 5), to_unsigned(26, 5), to_unsigned(28, 5),
        to_unsigned(29, 5), to_unsigned(30, 5), to_unsigned(31, 5), to_unsigned(31, 5),
        to_unsigned(31, 5), to_unsigned(31, 5), to_unsigned(30, 5), to_unsigned(29, 5),
        to_unsigned(28, 5), to_unsigned(26, 5), to_unsigned(24, 5), to_unsigned(22, 5),
        to_unsigned(20, 5), to_unsigned(18, 5), to_unsigned(15, 5), to_unsigned(12, 5),
        to_unsigned( 9, 5), to_unsigned( 6, 5), to_unsigned( 3, 5), to_unsigned( 0, 5)
    );

    --------------------------------------------------------------------------
    -- T1 registered inputs and position tracking
    --------------------------------------------------------------------------
    signal s_in_y          : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_in_u          : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_in_v          : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');

    signal s_pixel_x       : unsigned(11 downto 0) := (others => '0');
    signal s_pixel_y       : unsigned(11 downto 0) := (others => '0');
    signal s_prev_hsync_n  : std_logic := '1';
    signal s_prev_vsync_n  : std_logic := '1';

    -- Resolution measurement (latched from previous line/frame)
    signal s_line_width    : unsigned(11 downto 0) := to_unsigned(1280, 12);
    signal s_frame_height  : unsigned(11 downto 0) := to_unsigned(720, 12);

    -- DDA slice accumulators: per-pixel (x) and per-line (y)
    signal s_dda_x_acc     : unsigned(11 downto 0) := (others => '0');
    signal s_dda_y_acc     : unsigned(11 downto 0) := (others => '0');
    signal s_slice_x       : unsigned(7 downto 0) := (others => '0');
    signal s_slice_y       : unsigned(7 downto 0) := (others => '0');
    signal s_num_slices    : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal s_log2_slices   : integer range 1 to 7 := 3;


    --------------------------------------------------------------------------
    -- T2 intermediates: per-slice delay, pre-multiplied luma, delayed data
    --------------------------------------------------------------------------
    signal s_t2_in_y       : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_t2_in_u       : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_t2_in_v       : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_t2_delay_base : unsigned(C_DELAY_BITS - 1 downto 0) := (others => '0');
    signal s_t2_luma_raw   : unsigned(19 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- T3: assembled per-channel delays and pipeline-aligned pixel data
    --------------------------------------------------------------------------
    signal s_delay_y       : unsigned(C_DELAY_BITS - 1 downto 0) := (others => '0');
    signal s_delay_u       : unsigned(C_DELAY_BITS - 1 downto 0) := (others => '0');
    signal s_delay_v       : unsigned(C_DELAY_BITS - 1 downto 0) := (others => '0');
    signal s_t3_in_y       : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_t3_in_u       : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_t3_in_v       : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- T5: variable_delay_u outputs
    --------------------------------------------------------------------------
    signal s_smear_y       : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_smear_u       : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_smear_v       : unsigned(C_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------
    -- T6: alignment register before interpolator
    --------------------------------------------------------------------------
    signal s_proc_y        : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_proc_u        : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_proc_v        : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Bypass / sync delay shift registers (C_TOTAL_LATENCY deep)
    -- SR(0) = 1 clock delayed, SR(i) = (i+1) clocks delayed.
    --------------------------------------------------------------------------
    type t_data_shift is array (0 to C_TOTAL_LATENCY - 1)
        of std_logic_vector(C_DATA_WIDTH - 1 downto 0);
    type t_bit_shift  is array (0 to C_TOTAL_LATENCY - 1) of std_logic;

    signal s_y_sr          : t_data_shift := (others => (others => '0'));
    signal s_u_sr          : t_data_shift := (others => (others => '0'));
    signal s_v_sr          : t_data_shift := (others => (others => '0'));
    signal s_hsync_sr      : t_bit_shift  := (others => '1');
    signal s_vsync_sr      : t_bit_shift  := (others => '1');
    signal s_field_sr      : t_bit_shift  := (others => '1');
    signal s_avid_sr       : t_bit_shift  := (others => '0');

    --------------------------------------------------------------------------
    -- Interpolator outputs and control
    --------------------------------------------------------------------------
    signal s_mix_t         : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_y_result : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_u_result : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_v_result : unsigned(C_DATA_WIDTH - 1 downto 0);

    signal s_bypass_enable : std_logic;

begin

    s_bypass_enable <= registers_in(6)(4);
    s_mix_t         <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- T1: register inputs, measure resolution, advance DDAs, fill bypass SR
    --------------------------------------------------------------------------
    p_input_stage : process(clk)
        variable v_slices_raw : unsigned(9 downto 0);
        variable v_dda_x_next : unsigned(11 downto 0);
        variable v_dda_y_next : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y);
            s_in_u <= unsigned(data_in.u);
            s_in_v <= unsigned(data_in.v);

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            -- Decode slice count from knob 3 (done here so DDAs use it)
            v_slices_raw := unsigned(registers_in(2));
            if v_slices_raw < 146 then
                s_num_slices  <= to_unsigned(2, 8);
                s_log2_slices <= 1;
            elsif v_slices_raw < 293 then
                s_num_slices  <= to_unsigned(4, 8);
                s_log2_slices <= 2;
            elsif v_slices_raw < 439 then
                s_num_slices  <= to_unsigned(8, 8);
                s_log2_slices <= 3;
            elsif v_slices_raw < 585 then
                s_num_slices  <= to_unsigned(16, 8);
                s_log2_slices <= 4;
            elsif v_slices_raw < 732 then
                s_num_slices  <= to_unsigned(32, 8);
                s_log2_slices <= 5;
            elsif v_slices_raw < 878 then
                s_num_slices  <= to_unsigned(64, 8);
                s_log2_slices <= 6;
            else
                s_num_slices  <= to_unsigned(128, 8);
                s_log2_slices <= 7;
            end if;

            -- Horizontal pixel counter + per-pixel DDA (for V-mode columns)
            if data_in.avid = '1' then
                s_pixel_x <= s_pixel_x + 1;

                v_dda_x_next := s_dda_x_acc + resize(s_num_slices, 12);
                if v_dda_x_next >= s_line_width then
                    s_dda_x_acc <= v_dda_x_next - s_line_width;
                    s_slice_x   <= s_slice_x + 1;
                else
                    s_dda_x_acc <= v_dda_x_next;
                end if;
            end if;

            -- hsync falling edge: latch line width, reset x DDA, advance y DDA
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                -- Measurement: store pixel count as line width
                if s_pixel_x > 0 then
                    s_line_width <= s_pixel_x;
                end if;
                s_pixel_x   <= (others => '0');
                s_dda_x_acc <= (others => '0');
                s_slice_x   <= (others => '0');

                -- Per-line DDA (for H-mode horizontal bands)
                v_dda_y_next := s_dda_y_acc + resize(s_num_slices, 12);
                if v_dda_y_next >= s_frame_height then
                    s_dda_y_acc <= v_dda_y_next - s_frame_height;
                    s_slice_y   <= s_slice_y + 1;
                else
                    s_dda_y_acc <= v_dda_y_next;
                end if;

                s_pixel_y <= s_pixel_y + 1;
            end if;

            -- vsync falling edge: latch frame height, reset y DDA, update slew
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_pixel_y > 0 then
                    s_frame_height <= s_pixel_y;
                end if;
                s_pixel_y   <= (others => '0');
                s_pixel_x   <= (others => '0');
                s_dda_x_acc <= (others => '0');
                s_dda_y_acc <= (others => '0');
                s_slice_x   <= (others => '0');
                s_slice_y   <= (others => '0');
            end if;

            -- Bypass and sync shift registers
            s_y_sr(0)     <= data_in.y;
            s_u_sr(0)     <= data_in.u;
            s_v_sr(0)     <= data_in.v;
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_TOTAL_LATENCY - 1 loop
                s_y_sr(i)     <= s_y_sr(i - 1);
                s_u_sr(i)     <= s_u_sr(i - 1);
                s_v_sr(i)     <= s_v_sr(i - 1);
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_input_stage;

    --------------------------------------------------------------------------
    -- T2: read DDA slice index, apply pattern (random / sine), compute delay
    --
    -- The DDAs in T1 divide the active region into exactly N equal slices
    -- regardless of the input resolution.  Here we just pick the right
    -- slice_idx (x or y) and map it through the selected pattern.
    --
    -- Pattern (switch 10, bit 3):
    --   0 = Random  — bit-reversal of slice_idx creates scattered ordering.
    --                  delay = reversed_idx * (max_delay >> log2_N).
    --   1 = Sine    — half-sine-wave bump via 32-entry LUT.
    --                  delay = sine_lut[idx_32] * (max_delay >> 5).
    --------------------------------------------------------------------------
    p_slice_delay : process(clk)
        variable v_max_delay   : unsigned(10 downto 0);  -- knob×2, 0..2046
        variable v_slice_raw   : unsigned(7 downto 0);   -- from DDA (may reach N)
        variable v_slice_idx   : unsigned(6 downto 0);   -- clamped 0..N-1
        variable v_scrambled   : unsigned(6 downto 0);   -- bit-reversed idx
        variable v_lut_idx     : unsigned(4 downto 0);   -- scaled to 0..31
        variable v_sin_val     : unsigned(4 downto 0);   -- LUT output 0..31
        variable v_delay_step  : unsigned(10 downto 0);  -- max_delay / N or /32
        variable v_slice_prod  : unsigned(17 downto 0);  -- mapped_idx × step
    begin
        if rising_edge(clk) then
            -- Delay pixel data for T3 stage
            s_t2_in_y <= s_in_y;
            s_t2_in_u <= s_in_u;
            s_t2_in_v <= s_in_v;

            -- Pick the DDA-produced slice index for the active axis
            if registers_in(6)(0) = '0' then
                v_slice_raw := s_slice_y;   -- H-mode: horizontal bands
            else
                v_slice_raw := s_slice_x;   -- V-mode: vertical columns
            end if;

            -- Clamp to N-1 (DDA can briefly reach N at the last pixel/line)
            if v_slice_raw >= s_num_slices then
                v_slice_idx := s_num_slices(6 downto 0) - 1;
            else
                v_slice_idx := v_slice_raw(6 downto 0);
            end if;

            -- Knob 1 × 2 → max delay 0-2046 (11 bits)
            v_max_delay := unsigned(registers_in(0)) & '0';

            -- Pattern switch: bit 3 of register 6
            if registers_in(6)(3) = '0' then
                ----------------------------------------------------------------
                -- Random: bit-reverse the slice index within its active width.
                -- For N=4 (2 bits) e.g. 0,1,2,3 → 0,2,1,3
                -- For N=8 (3 bits) e.g. 0..7 → 0,4,2,6,1,5,3,7
                ----------------------------------------------------------------
                case s_log2_slices is
                    when 1 =>
                        v_scrambled := "000000" & v_slice_idx(0);
                    when 2 =>
                        v_scrambled := "00000" & v_slice_idx(0) & v_slice_idx(1);
                    when 3 =>
                        v_scrambled := "0000" & v_slice_idx(0)
                                             & v_slice_idx(1) & v_slice_idx(2);
                    when 4 =>
                        v_scrambled := "000" & v_slice_idx(0) & v_slice_idx(1)
                                            & v_slice_idx(2) & v_slice_idx(3);
                    when 5 =>
                        v_scrambled := "00" & v_slice_idx(0) & v_slice_idx(1)
                                           & v_slice_idx(2) & v_slice_idx(3)
                                           & v_slice_idx(4);
                    when 6 =>
                        v_scrambled := '0' & v_slice_idx(0) & v_slice_idx(1)
                                          & v_slice_idx(2) & v_slice_idx(3)
                                          & v_slice_idx(4) & v_slice_idx(5);
                    when others =>
                        v_scrambled := v_slice_idx(0) & v_slice_idx(1)
                                     & v_slice_idx(2) & v_slice_idx(3)
                                     & v_slice_idx(4) & v_slice_idx(5)
                                     & v_slice_idx(6);
                end case;

                -- Delay step = max_delay >> log2(N)
                case s_log2_slices is
                    when 1      => v_delay_step := '0'       & v_max_delay(10 downto 1);
                    when 2      => v_delay_step := "00"      & v_max_delay(10 downto 2);
                    when 3      => v_delay_step := "000"     & v_max_delay(10 downto 3);
                    when 4      => v_delay_step := "0000"    & v_max_delay(10 downto 4);
                    when 5      => v_delay_step := "00000"   & v_max_delay(10 downto 5);
                    when 6      => v_delay_step := "000000"  & v_max_delay(10 downto 6);
                    when others => v_delay_step := "0000000" & v_max_delay(10 downto 7);
                end case;

                v_slice_prod := v_scrambled * v_delay_step;
            else
                ----------------------------------------------------------------
                -- Sine: scale slice_idx into the 32-entry half-sine LUT, then
                -- multiply the LUT output (0..31) by max_delay/32.
                -- For N > 32, multiple slices map to the same LUT entry.
                ----------------------------------------------------------------
                case s_log2_slices is
                    when 1      => v_lut_idx := v_slice_idx(0) & "0000";
                    when 2      => v_lut_idx := v_slice_idx(1 downto 0) & "000";
                    when 3      => v_lut_idx := v_slice_idx(2 downto 0) & "00";
                    when 4      => v_lut_idx := v_slice_idx(3 downto 0) & '0';
                    when 5      => v_lut_idx := v_slice_idx(4 downto 0);
                    when 6      => v_lut_idx := v_slice_idx(5 downto 1);
                    when others => v_lut_idx := v_slice_idx(6 downto 2);
                end case;

                v_sin_val    := C_SINE_LUT(to_integer(v_lut_idx));
                v_delay_step := "00000" & v_max_delay(10 downto 5);  -- /32
                v_slice_prod := resize(v_sin_val * v_delay_step, 18);
            end if;

            -- Max product: 127 × 15 = 1905 (random@128) or 31 × 63 = 1953 (sine)
            s_t2_delay_base <= v_slice_prod(C_DELAY_BITS - 1 downto 0);

            -- Pre-multiply for luma modulation (full 20-bit product, shifted in T3)
            s_t2_luma_raw <= s_in_y * unsigned(registers_in(4));
        end if;
    end process p_slice_delay;

    --------------------------------------------------------------------------
    -- T3: delay assembly — scale, fold, reverse, offset, luma mod, UV diverge
    --------------------------------------------------------------------------
    p_delay_assemble : process(clk)
        variable v_max_delay       : unsigned(10 downto 0);
        variable v_delay_base      : unsigned(10 downto 0);
        variable v_half_max        : unsigned(10 downto 0);
        variable v_fold_diff       : signed(11 downto 0);
        variable v_neg_fold        : signed(11 downto 0);
        variable v_fold_abs        : unsigned(10 downto 0);
        variable v_delay_dir       : unsigned(10 downto 0);
        variable v_luma_contrib    : unsigned(10 downto 0);
        variable v_sum_y           : unsigned(13 downto 0);
        variable v_delay_y_clamped : unsigned(10 downto 0);
        variable v_chroma_lag      : unsigned(10 downto 0);
        variable v_uv_split        : signed(10 downto 0);
        variable v_sum_u           : signed(13 downto 0);
        variable v_sum_v           : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- Pipeline the pixel data
            s_t3_in_y <= s_t2_in_y;
            s_t3_in_u <= s_t2_in_u;
            s_t3_in_v <= s_t2_in_v;

            -- Knob 1 × 2 → max delay 0-2046 (11 bits)
            v_max_delay := unsigned(registers_in(0)) & '0';

            -- Per-slice delay computed in T2
            v_delay_base := s_t2_delay_base;

            -- Fold mode: map delay through V-shape, centre = present (delay=0)
            if registers_in(6)(2) = '1' then
                v_half_max  := '0' & v_max_delay(10 downto 1);
                v_fold_diff := signed(resize(v_delay_base, 12))
                             - signed(resize(v_half_max, 12));
                v_neg_fold  := -v_fold_diff;
                if v_fold_diff(11) = '1' then
                    v_fold_abs := unsigned(v_neg_fold(10 downto 0));
                else
                    v_fold_abs := unsigned(v_fold_diff(10 downto 0));
                end if;
                v_delay_base := resize(v_fold_abs, 11) + resize(v_fold_abs, 11);
                if v_delay_base > v_max_delay then
                    v_delay_base := v_max_delay;
                end if;
            end if;

            -- Reverse: swap which end is "now"
            if registers_in(6)(1) = '1' then
                v_delay_dir := v_max_delay - v_delay_base;
            else
                v_delay_dir := v_delay_base;
            end if;

            -- Luma modulation: (y * luma_mod) >> 10 → 0..1023
            v_luma_contrib := resize(s_t2_luma_raw(19 downto 10), 11);

            -- Assemble Y delay: base + offset (knob 4) + luma contribution
            v_sum_y := resize(v_delay_dir, 14)
                     + resize(unsigned(registers_in(3)), 14)
                     + resize(v_luma_contrib, 14);
            if v_sum_y > 2046 then
                v_delay_y_clamped := to_unsigned(2046, 11);
            else
                v_delay_y_clamped := v_sum_y(10 downto 0);
            end if;
            s_delay_y <= v_delay_y_clamped;

            -- Chroma lag: knob 2 >> 1 → 0-511 extra samples
            v_chroma_lag := resize(unsigned(registers_in(1)), 11);

            -- UV diverge: knob 6 - 512 → signed ±511
            v_uv_split := signed(resize(unsigned(registers_in(5)), 11))
                        - to_signed(512, 11);

            -- U delay: Y + chroma_lag + uv_split, clamped [0, 2046]
            v_sum_u := signed(resize(v_delay_y_clamped, 14))
                     + signed(resize(v_chroma_lag, 14))
                     + resize(v_uv_split, 14);
            if v_sum_u < 0 then
                s_delay_u <= (others => '0');
            elsif v_sum_u > 2046 then
                s_delay_u <= to_unsigned(2046, C_DELAY_BITS);
            else
                s_delay_u <= unsigned(v_sum_u(C_DELAY_BITS - 1 downto 0));
            end if;

            -- V delay: Y + chroma_lag - uv_split, clamped [0, 2046]
            v_sum_v := signed(resize(v_delay_y_clamped, 14))
                     + signed(resize(v_chroma_lag, 14))
                     - resize(v_uv_split, 14);
            if v_sum_v < 0 then
                s_delay_v <= (others => '0');
            elsif v_sum_v > 2046 then
                s_delay_v <= to_unsigned(2046, C_DELAY_BITS);
            else
                s_delay_v <= unsigned(v_sum_v(C_DELAY_BITS - 1 downto 0));
            end if;
        end if;
    end process p_delay_assemble;

    --------------------------------------------------------------------------
    -- T3-T5: variable delay lines (2-clock internal latency each)
    -- s_delay_*/s_t3_in_* registered at T3 → result valid at T5.
    --------------------------------------------------------------------------
    smear_y_inst : entity work.variable_delay_u
        generic map(G_WIDTH => C_DATA_WIDTH, G_DEPTH => C_DELAY_BITS)
        port map(
            clk    => clk,
            enable => '1',
            delay  => s_delay_y,
            a      => s_t3_in_y,
            result => s_smear_y,
            valid  => open
        );

    smear_u_inst : entity work.variable_delay_u
        generic map(G_WIDTH => C_DATA_WIDTH, G_DEPTH => C_DELAY_BITS)
        port map(
            clk    => clk,
            enable => '1',
            delay  => s_delay_u,
            a      => s_t3_in_u,
            result => s_smear_u,
            valid  => open
        );

    smear_v_inst : entity work.variable_delay_u
        generic map(G_WIDTH => C_DATA_WIDTH, G_DEPTH => C_DELAY_BITS)
        port map(
            clk    => clk,
            enable => '1',
            delay  => s_delay_v,
            a      => s_t3_in_v,
            result => s_smear_v,
            valid  => open
        );

    --------------------------------------------------------------------------
    -- T6: alignment register — holds smear output one cycle before interpolator
    --------------------------------------------------------------------------
    p_proc_align : process(clk)
        constant C_BLACK_THRESH : unsigned(C_DATA_WIDTH - 1 downto 0)
            := to_unsigned(16, C_DATA_WIDTH);
    begin
        if rising_edge(clk) then
            if s_smear_y > C_BLACK_THRESH then
                s_proc_y <= s_smear_y;
                s_proc_u <= s_smear_u;
                s_proc_v <= s_smear_v;
            end if;
            -- else: registers retain previous value (hold last valid pixel)
        end if;
    end process p_proc_align;

    --------------------------------------------------------------------------
    -- Interpolator: 4-clock wet/dry crossfade.
    -- Dry tap at SR(5) = 6 clocks delay, aligning with s_proc_* at T6.
    -- Interpolator stage-0 fires at T7; result valid at T10.
    --------------------------------------------------------------------------
    interp_y_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DATA_WIDTH,
            G_FRAC_BITS  => C_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(s_y_sr(5)),
            b      => s_proc_y,
            t      => s_mix_t,
            result => s_interp_y_result,
            valid  => open
        );

    interp_u_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DATA_WIDTH,
            G_FRAC_BITS  => C_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(s_u_sr(5)),
            b      => s_proc_u,
            t      => s_mix_t,
            result => s_interp_u_result,
            valid  => open
        );

    interp_v_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DATA_WIDTH,
            G_FRAC_BITS  => C_DATA_WIDTH,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => unsigned(s_v_sr(5)),
            b      => s_proc_v,
            t      => s_mix_t,
            result => s_interp_v_result,
            valid  => open
        );

    --------------------------------------------------------------------------
    -- Output at T10.
    -- Sync signals always come from the latency-matched SR.
    -- Video uses bypass SR when Bypass is on, interpolator otherwise.
    --------------------------------------------------------------------------
    data_out.hsync_n <= s_hsync_sr(C_TOTAL_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_TOTAL_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_TOTAL_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_TOTAL_LATENCY - 1);

    data_out.y <= s_y_sr(C_TOTAL_LATENCY - 1) when s_bypass_enable = '1'
                  else std_logic_vector(s_interp_y_result);
    data_out.u <= s_u_sr(C_TOTAL_LATENCY - 1) when s_bypass_enable = '1'
                  else std_logic_vector(s_interp_u_result);
    data_out.v <= s_v_sr(C_TOTAL_LATENCY - 1) when s_bypass_enable = '1'
                  else std_logic_vector(s_interp_v_result);

end architecture slitscan;
