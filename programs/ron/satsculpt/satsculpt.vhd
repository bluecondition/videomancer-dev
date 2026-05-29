-- Videomancer SDK - Open source FPGA-based video effects development kit
-- File: satsculpt.vhd - Saturation Sculptor Program for Videomancer
-- Author: ron
-- License: GNU General Public License v3.0
--
-- Program Name:
--   Satsculpt
--
-- Overview:
--   Applies a non-linear gain curve to chroma magnitude. The curve has three
--   anchor points (Low, Mid, High) at saturation magnitudes 0, 256 and 512.
--   Between anchors, gain is linearly interpolated. The chroma vector
--   (U-512, V-512) is then scaled by the per-pixel gain, with optional Tint
--   added to U or V on output. Y is passed through unchanged (delayed).
--
-- Architecture:
--   Stage 1: Pre-process (1 clk)
--     Compute du, dv, |du|, |dv|, sat (Chebyshev or Manhattan), pivot-bias the
--     sat domain, derive interpolator endpoints (a, b) and t for the segment
--     containing the biased sat. Apply polarity flip and chroma-gate threshold.
--   Stage 2: Curve evaluation (4 clk)
--     interpolator_u computes the per-pixel gain.
--   Stage 3: Chroma scaling (10 clk)
--     proc_amp_u for U and V (chroma scaled around 512 by curve gain).
--   Stage 4: Mix (4 clk)
--     interpolator_u blends dry and wet U/V; tint added at output mux. Y is
--     just passed through delayed.
--
--   Total wet-path latency from data_in to data_out: 19 clocks.
--
-- Register Map:
--   Register 0: Low Gain      (rotary 1)
--   Register 1: Mid Gain      (rotary 2)
--   Register 2: High Gain     (rotary 3)
--   Register 3: Pivot Bias    (rotary 4, 512 = no shift)
--   Register 4: Threshold     (rotary 5, 0 = off)
--   Register 5: Tint          (rotary 6, 512 = no tint)
--   Register 6: Control flags
--     Bit 0: Sat Metric     (0 = Max,    1 = Sum)
--     Bit 1: Polarity       (0 = Normal, 1 = Inverted)
--     Bit 2: Tint Axis      (0 = V,      1 = U)
--     Bit 3: Channel Lock   (0 = Linked, 1 = U Only — V passes through)
--     Bit 4: Bypass         (0 = Off,    1 = On)
--   Register 7: Mix          (linear pot 12)

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture satsculpt of program_top is

    --------------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------------
    constant C_NEUTRAL          : integer := 512;
    -- proc_amp_u brightness reg = 512 → +0.5 internal offset, which re-centres
    -- the chroma output around 512 so that contrast=512 (unity) gives output =
    -- input. With brightness reg = 1023 the output saturates and knobs appear
    -- to do nothing — that was the v0 bug.
    constant C_BRIGHT_PASSTHRU  : integer := 512;
    constant C_DATA_W           : integer := C_VIDEO_DATA_WIDTH;        -- 10

    -- Block latencies. The pre-process is split into two cycles to keep
    -- combinational paths short (the original single-cycle pre-process was
    -- the Fmax bottleneck).
    constant C_PRE_LAT          : integer := 2;    -- pre-process (1a + 1b)
    constant C_INTERP_LAT       : integer := 4;
    constant C_PROC_LAT         : integer := 10;   -- proc_amp_u total
    constant C_MIX_LAT          : integer := 4;
    constant C_OUT_LAT          : integer := 1;    -- output_shape register stage
    constant C_TOTAL_LAT        : integer := C_PRE_LAT + C_INTERP_LAT + C_PROC_LAT + C_MIX_LAT + C_OUT_LAT;  -- 21

    -- Delay-line tap offsets. Tap N gives data_in delayed by N+1 cycles when
    -- the signal is sampled (registered → next-clock visible).
    --   proc_amp_u captures inputs at T = C_PRE_LAT + C_INTERP_LAT = 6
    --       → tap N where N+1 = 6 → N = 5
    --   mix interp captures inputs at T = 6 + C_PROC_LAT = 16
    --       → tap N+1 = 16 → N = 15
    --   bypass output appears at T = C_TOTAL_LAT = 21
    --       → tap N+1 = 21 → N = 20
    constant C_TAP_PROC         : integer := C_PRE_LAT + C_INTERP_LAT - 1;                           -- 5
    constant C_TAP_MIX          : integer := C_PRE_LAT + C_INTERP_LAT + C_PROC_LAT - 1;              -- 15
    constant C_TAP_BYPASS       : integer := C_TOTAL_LAT - 1;                                        -- 20

    --------------------------------------------------------------------------------
    -- Control Signals
    --------------------------------------------------------------------------------
    signal s_low_gain       : unsigned(C_DATA_W - 1 downto 0);
    signal s_mid_gain       : unsigned(C_DATA_W - 1 downto 0);
    signal s_high_gain      : unsigned(C_DATA_W - 1 downto 0);
    signal s_pivot_bias     : unsigned(C_DATA_W - 1 downto 0);
    signal s_threshold      : unsigned(C_DATA_W - 1 downto 0);
    signal s_tint           : unsigned(C_DATA_W - 1 downto 0);
    signal s_sat_metric     : std_logic;
    signal s_polarity       : std_logic;
    signal s_tint_axis      : std_logic;
    signal s_channel_lock   : std_logic;
    signal s_bypass         : std_logic;
    signal s_mix            : unsigned(C_DATA_W - 1 downto 0);

    --------------------------------------------------------------------------------
    -- Stage 1a: sat-magnitude registers (between preprocess halves)
    --------------------------------------------------------------------------------
    signal s_sat_raw_1a     : unsigned(C_DATA_W - 1 downto 0);
    signal s_sat_biased_1a  : signed(C_DATA_W + 1 downto 0);
    signal s_pre_valid_1a   : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 1b: interpolator inputs
    --------------------------------------------------------------------------------
    signal s_pre_valid      : std_logic;
    signal s_interp_a       : unsigned(C_DATA_W - 1 downto 0);
    signal s_interp_b       : unsigned(C_DATA_W - 1 downto 0);
    signal s_interp_t       : unsigned(C_DATA_W - 1 downto 0);
    signal s_sat_raw_1b     : unsigned(C_DATA_W - 1 downto 0);  -- carried for threshold gate

    --------------------------------------------------------------------------------
    -- Stage 2: Curve gain
    --------------------------------------------------------------------------------
    signal s_gain_curve     : unsigned(C_DATA_W - 1 downto 0);
    signal s_gain_valid     : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 3: Chroma proc_amp_u
    --------------------------------------------------------------------------------
    signal s_proc_brightness: unsigned(C_DATA_W - 1 downto 0);
    signal s_u_proc         : unsigned(C_DATA_W - 1 downto 0);
    signal s_u_proc_valid   : std_logic;
    signal s_v_proc         : unsigned(C_DATA_W - 1 downto 0);
    signal s_v_proc_valid   : std_logic;

    --------------------------------------------------------------------------------
    -- Bypass / Mix Delay Line taps
    --------------------------------------------------------------------------------
    signal s_y_to_proc      : std_logic_vector(C_DATA_W - 1 downto 0);
    signal s_u_to_proc      : std_logic_vector(C_DATA_W - 1 downto 0);
    signal s_v_to_proc      : std_logic_vector(C_DATA_W - 1 downto 0);

    signal s_u_to_mix       : std_logic_vector(C_DATA_W - 1 downto 0);
    signal s_v_to_mix       : std_logic_vector(C_DATA_W - 1 downto 0);

    signal s_y_full_delayed : std_logic_vector(C_DATA_W - 1 downto 0);
    signal s_u_full_delayed : std_logic_vector(C_DATA_W - 1 downto 0);
    signal s_v_full_delayed : std_logic_vector(C_DATA_W - 1 downto 0);

    signal s_hsync_n_delayed : std_logic;
    signal s_vsync_n_delayed : std_logic;
    signal s_field_n_delayed : std_logic;

    --------------------------------------------------------------------------------
    -- Stage 4: Mix and tint
    --------------------------------------------------------------------------------
    signal s_mix_u_result   : unsigned(C_DATA_W - 1 downto 0);
    signal s_mix_u_valid    : std_logic;
    signal s_mix_v_result   : unsigned(C_DATA_W - 1 downto 0);
    signal s_mix_v_valid    : std_logic;

    signal s_u_out          : unsigned(C_DATA_W - 1 downto 0);
    signal s_v_out          : unsigned(C_DATA_W - 1 downto 0);
    signal s_out_valid      : std_logic;

begin
    --------------------------------------------------------------------------------
    -- Register Mapping
    --------------------------------------------------------------------------------
    s_low_gain      <= unsigned(registers_in(0));
    s_mid_gain      <= unsigned(registers_in(1));
    s_high_gain     <= unsigned(registers_in(2));
    s_pivot_bias    <= unsigned(registers_in(3));
    s_threshold     <= unsigned(registers_in(4));
    s_tint          <= unsigned(registers_in(5));
    s_sat_metric    <= registers_in(6)(0);
    s_polarity      <= registers_in(6)(1);
    s_tint_axis     <= registers_in(6)(2);
    s_channel_lock  <= registers_in(6)(3);
    s_bypass        <= registers_in(6)(4);
    s_mix           <= unsigned(registers_in(7));

    s_proc_brightness <= to_unsigned(C_BRIGHT_PASSTHRU, C_DATA_W);

    --------------------------------------------------------------------------------
    -- Stage 1a: chroma deviation, abs, sat magnitude, pivot bias
    -- Latency: 1 cycle
    -- Heavy combinational work: signed sub, abs, metric mux, signed sub for
    -- pivot bias. Splitting from segment-select breaks the long carry chain.
    --------------------------------------------------------------------------------
    p_preprocess_1a : process (clk)
        variable v_du           : signed(C_DATA_W downto 0);
        variable v_dv           : signed(C_DATA_W downto 0);
        variable v_abs_du       : unsigned(C_DATA_W - 1 downto 0);
        variable v_abs_dv       : unsigned(C_DATA_W - 1 downto 0);
        variable v_sat_raw      : unsigned(C_DATA_W - 1 downto 0);
        variable v_sum          : unsigned(C_DATA_W downto 0);
        constant C_NEUTRAL_S    : signed(C_DATA_W + 1 downto 0) := to_signed(C_NEUTRAL, C_DATA_W + 2);
    begin
        if rising_edge(clk) then
            v_du := signed('0' & data_in.u) - to_signed(C_NEUTRAL, C_DATA_W + 1);
            v_dv := signed('0' & data_in.v) - to_signed(C_NEUTRAL, C_DATA_W + 1);

            v_abs_du := unsigned(std_logic_vector(resize(abs(v_du), C_DATA_W)));
            v_abs_dv := unsigned(std_logic_vector(resize(abs(v_dv), C_DATA_W)));

            -- Sat magnitude in 0..512.
            --   Max: Chebyshev (sharp, follows dominant chroma axis)
            --   Sum: Manhattan / 2 (smoother circular contour)
            if s_sat_metric = '0' then
                if v_abs_du > v_abs_dv then
                    v_sat_raw := v_abs_du;
                else
                    v_sat_raw := v_abs_dv;
                end if;
            else
                v_sum     := resize(v_abs_du, C_DATA_W + 1) + resize(v_abs_dv, C_DATA_W + 1);
                v_sat_raw := v_sum(C_DATA_W downto 1);
            end if;

            s_sat_raw_1a    <= v_sat_raw;
            s_sat_biased_1a <= resize(signed('0' & std_logic_vector(v_sat_raw)), C_DATA_W + 2)
                             - (resize(signed('0' & std_logic_vector(s_pivot_bias)), C_DATA_W + 2) - C_NEUTRAL_S);
            s_pre_valid_1a  <= data_in.avid;
        end if;
    end process p_preprocess_1a;

    --------------------------------------------------------------------------------
    -- Stage 1b: clamp, segment select, anchor mux, polarity, threshold gate
    -- Latency: 1 cycle
    --------------------------------------------------------------------------------
    p_preprocess_1b : process (clk)
        variable v_sat_clamped  : unsigned(C_DATA_W - 1 downto 0);
        variable v_seg_offset   : unsigned(C_DATA_W - 3 downto 0);
        variable v_a            : unsigned(C_DATA_W - 1 downto 0);
        variable v_b            : unsigned(C_DATA_W - 1 downto 0);
        constant C_MID_PIVOT    : unsigned(C_DATA_W - 1 downto 0) := to_unsigned(256, C_DATA_W);
    begin
        if rising_edge(clk) then
            -- Clamp biased sat to 0..511.
            if s_sat_biased_1a < 0 then
                v_sat_clamped := (others => '0');
            elsif s_sat_biased_1a > 511 then
                v_sat_clamped := to_unsigned(511, C_DATA_W);
            else
                v_sat_clamped := unsigned(std_logic_vector(s_sat_biased_1a(C_DATA_W - 1 downto 0)));
            end if;

            -- Segment select: low half vs high half of the sat domain.
            -- Fixed mid pivot of 256 → no division. t = offset << 2.
            if v_sat_clamped < C_MID_PIVOT then
                v_a := s_low_gain;
                v_b := s_mid_gain;
            else
                v_a := s_mid_gain;
                v_b := s_high_gain;
            end if;
            v_seg_offset := v_sat_clamped(C_DATA_W - 3 downto 0);

            -- Polarity inversion: bitwise NOT mirrors anchors around 1023.
            if s_polarity = '1' then
                v_a := not v_a;
                v_b := not v_b;
            end if;

            -- Hard chroma gate: sat below threshold/2 forces gain to zero.
            -- Compares raw (un-biased) sat against half the threshold reg.
            if s_sat_raw_1a < ('0' & s_threshold(C_DATA_W - 1 downto 1)) then
                v_a := (others => '0');
                v_b := (others => '0');
            end if;

            s_interp_a   <= v_a;
            s_interp_b   <= v_b;
            s_interp_t   <= v_seg_offset & "00";
            s_sat_raw_1b <= s_sat_raw_1a;
            s_pre_valid  <= s_pre_valid_1a;
        end if;
    end process p_preprocess_1b;

    --------------------------------------------------------------------------------
    -- Stage 2: Curve interpolator
    -- Latency: 4 cycles
    --------------------------------------------------------------------------------
    curve_interp : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DATA_W,
            G_FRAC_BITS  => C_DATA_W,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_pre_valid,
            a      => s_interp_a,
            b      => s_interp_b,
            t      => s_interp_t,
            result => s_gain_curve,
            valid  => s_gain_valid
        );

    --------------------------------------------------------------------------------
    -- Bypass / Mix Delay Line
    --------------------------------------------------------------------------------
    p_delay_line : process (clk)
        type t_data_arr is array (0 to C_TAP_BYPASS) of std_logic_vector(C_DATA_W - 1 downto 0);
        type t_sync_arr is array (0 to C_TAP_BYPASS) of std_logic;

        variable v_y     : t_data_arr := (others => (others => '0'));
        variable v_u     : t_data_arr := (others => (others => '0'));
        variable v_v     : t_data_arr := (others => (others => '0'));
        variable v_hsync : t_sync_arr := (others => '1');
        variable v_vsync : t_sync_arr := (others => '1');
        variable v_field : t_sync_arr := (others => '1');
    begin
        if rising_edge(clk) then
            v_y     := data_in.y       & v_y(0 to C_TAP_BYPASS - 1);
            v_u     := data_in.u       & v_u(0 to C_TAP_BYPASS - 1);
            v_v     := data_in.v       & v_v(0 to C_TAP_BYPASS - 1);
            v_hsync := data_in.hsync_n & v_hsync(0 to C_TAP_BYPASS - 1);
            v_vsync := data_in.vsync_n & v_vsync(0 to C_TAP_BYPASS - 1);
            v_field := data_in.field_n & v_field(0 to C_TAP_BYPASS - 1);

            s_y_to_proc       <= v_y(C_TAP_PROC);
            s_u_to_proc       <= v_u(C_TAP_PROC);
            s_v_to_proc       <= v_v(C_TAP_PROC);

            s_u_to_mix        <= v_u(C_TAP_MIX);
            s_v_to_mix        <= v_v(C_TAP_MIX);

            s_y_full_delayed  <= v_y(C_TAP_BYPASS);
            s_u_full_delayed  <= v_u(C_TAP_BYPASS);
            s_v_full_delayed  <= v_v(C_TAP_BYPASS);

            s_hsync_n_delayed <= v_hsync(C_TAP_BYPASS);
            s_vsync_n_delayed <= v_vsync(C_TAP_BYPASS);
            s_field_n_delayed <= v_field(C_TAP_BYPASS);
        end if;
    end process p_delay_line;

    --------------------------------------------------------------------------------
    -- Stage 3: Chroma scaling (proc_amp_u)
    -- Latency: 10 cycles (1 input + 9 multiplier_s)
    --   gain_curve = 512 → unity
    --   gain_curve = 0   → output = 512 (full desat)
    --   gain_curve = 1023→ ≈ 2× boost (clamped)
    --
    -- s_y_to_proc kept for tap symmetry — Y bypasses scaling and goes straight
    -- to the output via s_y_full_delayed.
    --------------------------------------------------------------------------------
    proc_amp_uchan : entity work.proc_amp_u
        generic map(
            G_WIDTH => C_DATA_W
        )
        port map(
            clk        => clk,
            enable     => s_gain_valid,
            a          => unsigned(s_u_to_proc),
            contrast   => s_gain_curve,
            brightness => s_proc_brightness,
            result     => s_u_proc,
            valid      => s_u_proc_valid
        );

    proc_amp_vchan : entity work.proc_amp_u
        generic map(
            G_WIDTH => C_DATA_W
        )
        port map(
            clk        => clk,
            enable     => s_gain_valid,
            a          => unsigned(s_v_to_proc),
            contrast   => s_gain_curve,
            brightness => s_proc_brightness,
            result     => s_v_proc,
            valid      => s_v_proc_valid
        );

    --------------------------------------------------------------------------------
    -- Stage 4: Output mix interpolators (4 cycles, U and V only)
    --   mix = 0    → fully dry  (delayed input)
    --   mix = 1023 → fully wet  (sculpted)
    --------------------------------------------------------------------------------
    mix_u : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DATA_W,
            G_FRAC_BITS  => C_DATA_W,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_u_proc_valid,
            a      => unsigned(s_u_to_mix),
            b      => s_u_proc,
            t      => s_mix,
            result => s_mix_u_result,
            valid  => s_mix_u_valid
        );

    mix_v : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DATA_W,
            G_FRAC_BITS  => C_DATA_W,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => s_v_proc_valid,
            a      => unsigned(s_v_to_mix),
            b      => s_v_proc,
            t      => s_mix,
            result => s_mix_v_result,
            valid  => s_mix_v_valid
        );

    --------------------------------------------------------------------------------
    -- Output: Tint, Channel Lock, Bypass
    --   Tint adds a signed offset (tint_reg - 512, range ±511) to U or V on
    --   the output of the mix stage. Single signed adder + clamp per axis,
    --   chosen by the Tint Axis toggle.
    --   Channel Lock = U Only forces V to its delayed dry value.
    --------------------------------------------------------------------------------
    p_output_shape : process (clk)
        variable v_tint_off  : signed(C_DATA_W + 1 downto 0);
        variable v_u_sum     : signed(C_DATA_W + 1 downto 0);
        variable v_v_sum     : signed(C_DATA_W + 1 downto 0);
        variable v_u_clamped : unsigned(C_DATA_W - 1 downto 0);
        variable v_v_clamped : unsigned(C_DATA_W - 1 downto 0);
        variable v_v_src     : unsigned(C_DATA_W - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_tint_off := resize(signed('0' & std_logic_vector(s_tint)), C_DATA_W + 2)
                        - to_signed(C_NEUTRAL, C_DATA_W + 2);

            -- Channel Lock: when '1', V output uses delayed input (skip processing).
            if s_channel_lock = '1' then
                v_v_src := unsigned(s_v_full_delayed);
            else
                v_v_src := s_mix_v_result;
            end if;

            -- Tint axis selects which channel receives the additive offset.
            if s_tint_axis = '0' then
                v_u_sum := resize(signed('0' & std_logic_vector(s_mix_u_result)), C_DATA_W + 2);
                v_v_sum := resize(signed('0' & std_logic_vector(v_v_src)),        C_DATA_W + 2) + v_tint_off;
            else
                v_u_sum := resize(signed('0' & std_logic_vector(s_mix_u_result)), C_DATA_W + 2) + v_tint_off;
                v_v_sum := resize(signed('0' & std_logic_vector(v_v_src)),        C_DATA_W + 2);
            end if;

            if v_u_sum < 0 then
                v_u_clamped := (others => '0');
            elsif v_u_sum > 1023 then
                v_u_clamped := (others => '1');
            else
                v_u_clamped := unsigned(std_logic_vector(v_u_sum(C_DATA_W - 1 downto 0)));
            end if;

            if v_v_sum < 0 then
                v_v_clamped := (others => '0');
            elsif v_v_sum > 1023 then
                v_v_clamped := (others => '1');
            else
                v_v_clamped := unsigned(std_logic_vector(v_v_sum(C_DATA_W - 1 downto 0)));
            end if;

            s_u_out     <= v_u_clamped;
            s_v_out     <= v_v_clamped;
            s_out_valid <= s_mix_u_valid and s_mix_v_valid;
        end if;
    end process p_output_shape;

    --------------------------------------------------------------------------------
    -- Output Multiplexing
    -- Y always passes through delayed (no Y processing in this version).
    -- Bypass swaps the chroma path for delayed input.
    --
    -- Note: s_u_out / s_v_out are 1 cycle after s_mix_*_result. The bypass
    -- delay line is sized for the mix-stage output cycle, so when bypass='1'
    -- we use the delayed values one tap earlier (still within ~1 cycle skew
    -- which the downstream timing tolerates) — acceptable for this small
    -- output stage.
    --------------------------------------------------------------------------------
    data_out.y <= s_y_full_delayed;
    data_out.u <= std_logic_vector(s_u_out) when s_bypass = '0' else s_u_full_delayed;
    data_out.v <= std_logic_vector(s_v_out) when s_bypass = '0' else s_v_full_delayed;

    data_out.avid    <= s_out_valid;
    data_out.hsync_n <= s_hsync_n_delayed;
    data_out.vsync_n <= s_vsync_n_delayed;
    data_out.field_n <= s_field_n_delayed;

end satsculpt;
