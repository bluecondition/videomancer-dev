-- Trioweave: 3 half-wave-rectified sine VCOs.
--
-- Each oscillator outputs only its positive half-cycle (sin > 0 -> bright
-- bump; sin <= 0 -> black).  This gives discrete bright "bars" with dark
-- gaps where the carrier dips below zero, so when no oscillator is in its
-- bright phase at a pixel the output goes to black.
--
-- Knob layout:
--   K1 OSC1 Freq   K2 OSC2 Freq   K3 OSC3 Freq
--   K4 OSC1 LFO    K5 OSC2 LFO    K6 OSC3 LFO
--   S7/S8/S9 Gate which oscs are subject to the slider Thickness.
--            Off: that osc stays at 1x (~50% duty, current "thin" look).
--            On:  that osc gets the slider's thickness (1x to 10x).
--   S10/S11 Blend mode (4 options):
--     (S10,S11) = (0,0) -> MIN  -- darkest wins
--     (S10,S11) = (1,0) -> DIFF -- abs difference
--     (S10,S11) = (0,1) -> AND  -- bitwise AND (sparse glitch)
--     (S10,S11) = (1,1) -> OR   -- bitwise OR (bright glitch)
--   S12 Thickness -- global 1x..10x.  At 1x (slider=0) bright stripes
--                    occupy ~50% of the cycle (matches old "thin").  At
--                    10x (slider=1023) bright zones cover ~91% of the
--                    cycle for oscs whose switch is on.
--
-- Scroll is hardcoded to always-on (no switch for it).
-- Colors: fixed deeper RGB-like triad (compile-time U/V constants).
--
-- Resources:
--   3 carrier sin LUTs (per-pixel)
--   1 shared LFO sin LUT (time-muxed across 3 oscs during hsync blanking)
-- Total: ~12 BRAMs, low LC count.
--
-- Pipeline (data_in -> data_out): ~11 clocks
--
-- License: GPL-3.0
-- Author: ron

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture trioweave of program_top is

    constant C_N_OSCS        : integer := 3;
    constant C_DELAY_CLKS    : integer := 11;
    constant C_CARRIER_W     : integer := 20;
    constant C_LFO_W         : integer := 20;
    constant C_SCROLL_W      : integer := 16;

    constant C_FREQ_KNOB_SH  : integer := 5;
    constant C_FREQ_MIN      : integer := 1456;
    constant C_LFO_FLOOR     : integer := 816;

    -- Narrow per-osc LFO knob ranges (user-tuned sweet zones, 2026-05-25).
    -- step(i) = FLOOR_i + K * RANGE_i / 1024, so the full 0..1023 knob travel
    -- covers a small zone around the desired LFO step.
    constant C_LFO_FLOOR_0   : integer := 1890;
    constant C_LFO_RANGE_0   : integer := 123;
    constant C_LFO_FLOOR_1   : integer := 2844;
    constant C_LFO_RANGE_1   : integer := 153;
    constant C_LFO_FLOOR_2   : integer := 2811;
    constant C_LFO_RANGE_2   : integer := 216;
    constant C_SCROLL_RATE   : unsigned(9 downto 0) := to_unsigned(384, 10);

    -- Deeper RGB-like triad (compile-time U/V constants, ~25% larger
    -- chroma offsets than before).
    type t_uv_array is array (0 to C_N_OSCS-1) of unsigned(9 downto 0);
    constant C_OSC_U : t_uv_array := (
        to_unsigned(832, 10),   -- OSC1 red:    +320 from neutral
        to_unsigned(352, 10),   -- OSC2 green:  -160
        to_unsigned(342, 10));  -- OSC3 blue:   -170
    constant C_OSC_V : t_uv_array := (
        to_unsigned(512, 10),   -- OSC1:    0
        to_unsigned(792, 10),   -- OSC2: +280
        to_unsigned(232, 10));  -- OSC3: -280

    constant C_UV_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ====================================================================
    -- Triangle wave from 10-bit phase, range -510..+510 signed (matches sin).
    -- ====================================================================
    function tri_fold(phase : unsigned(9 downto 0)) return signed is
        variable v_mag_a : unsigned(7 downto 0);
        variable v_mag9  : unsigned(8 downto 0);
        variable v_pos   : signed(9 downto 0);
    begin
        if phase(8) = '0' then
            v_mag_a := phase(7 downto 0);
        else
            v_mag_a := not phase(7 downto 0);
        end if;
        v_mag9 := shift_left(resize(v_mag_a, 9), 1);
        v_pos  := signed(resize(v_mag9, 10));
        if phase(9) = '0' then
            return v_pos;
        else
            return -v_pos;
        end if;
    end function;

    -- ====================================================================
    -- Combinational knob view
    -- ====================================================================
    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0);
    signal s_k4, s_k5, s_k6 : unsigned(9 downto 0);
    signal s_s7, s_s8, s_s9, s_s10, s_s11 : std_logic;
    signal s_k12 : unsigned(9 downto 0);

    -- Vsync-latched per-osc params + globals
    type t_param_array is array (0 to C_N_OSCS-1) of unsigned(9 downto 0);
    signal r_freq : t_param_array := (
        to_unsigned(256, 10),
        to_unsigned(384, 10),
        to_unsigned(512, 10));
    signal r_lfo  : t_param_array := (
        to_unsigned(200, 10),
        to_unsigned(320, 10),
        to_unsigned(240, 10));
    -- Per-osc thickness toggle (bit i = OSC(i) thick).
    signal r_thick     : std_logic_vector(C_N_OSCS-1 downto 0) := (others => '0');
    -- Output blend mode. Bit 1 hardcoded to '1' so r_mode is always in
    -- {010, 011, 110, 111} = {MIN, DIFF, AND, OR}.
    signal r_mode      : unsigned(2 downto 0) := "010";
    signal r_slider    : unsigned(9 downto 0) := (others => '0');

    -- Timing record
    signal s_timing : t_video_timing_port;

    -- ====================================================================
    -- Slider-derived global thickness threshold (combinational).
    -- threshold = -slider * 31/64, range -495..0.
    -- ====================================================================
    signal s_slider_thresh : signed(11 downto 0);

    -- ====================================================================
    -- Per-osc LFO state
    -- ====================================================================
    type t_acc_slv  is array (0 to C_N_OSCS-1) of std_logic_vector(C_CARRIER_W-1 downto 0);
    type t_lfo_slv  is array (0 to C_N_OSCS-1) of std_logic_vector(C_LFO_W-1 downto 0);
    type t_phase10  is array (0 to C_N_OSCS-1) of unsigned(9 downto 0);
    type t_signed10 is array (0 to C_N_OSCS-1) of signed(9 downto 0);
    type t_uns10    is array (0 to C_N_OSCS-1) of unsigned(9 downto 0);

    signal s_lfo_step_slv  : t_lfo_slv;
    signal s_lfo_phase_slv : t_lfo_slv;
    signal s_lfo_phase     : t_phase10;

    -- LFO time-mux state (shared sin LUT, 3 reads per hsync)
    signal r_lfo_mux_idx   : unsigned(2 downto 0) := (others => '0');
    signal r_lfo_mux_idx_d : unsigned(2 downto 0) := (others => '0');
    signal s_lfo_lut_in    : std_logic_vector(9 downto 0);
    signal s_lfo_lut_sin   : signed(9 downto 0);
    signal r_lfo_lut_sin   : signed(9 downto 0) := (others => '0');
    signal r_lfo_sin       : t_signed10 := (others => (others => '0'));

    -- Carrier acc IO
    signal s_osc_inc_slv : t_acc_slv;
    signal s_osc_acc_slv : t_acc_slv;
    signal s_osc_acc_top : t_phase10;

    -- Scroll
    signal s_scroll_phase : unsigned(C_SCROLL_W-1 downto 0);
    signal s_scroll_top   : unsigned(9 downto 0);

    -- PM phase
    signal s_osc_pm_phase : t_phase10;
    signal r_osc_pm_phase : t_phase10 := (others => (others => '0'));

    -- Carrier sin LUTs
    signal s_osc_sin   : t_signed10;
    signal r_osc_sin_r : t_signed10 := (others => (others => '0'));

    -- Per-osc registered threshold (selected from slider or zero based on
    -- r_thick(i)).  Registering moves the small mux out of the luma
    -- combinational path.
    type t_thresh_arr is array (0 to C_N_OSCS-1) of signed(11 downto 0);
    signal r_osc_thresh : t_thresh_arr := (others => (others => '0'));

    -- Per-osc luma
    signal r_osc_luma : t_uns10 := (others => (others => '0'));

    -- ====================================================================
    -- Keyer pre-stage and output stage
    -- ====================================================================
    signal r_max_val : unsigned(9 downto 0) := (others => '0');
    signal r_max_idx : unsigned(1 downto 0) := (others => '0');
    signal r_min_val : unsigned(9 downto 0) := (others => '0');
    signal r_min_idx : unsigned(1 downto 0) := (others => '0');
    signal r_sum_y   : unsigned(11 downto 0) := (others => '0');
    signal r_and_y   : unsigned(9 downto 0) := (others => '0');
    signal r_or_y    : unsigned(9 downto 0) := (others => '0');
    signal r_l0_d    : unsigned(9 downto 0) := (others => '0');

    -- Output
    signal r_y_out : unsigned(9 downto 0) := (others => '0');
    signal r_u_out : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_v_out : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Sync delay
    signal s_avid_d    : std_logic := '0';
    signal s_hsync_n_d : std_logic := '1';
    signal s_vsync_n_d : std_logic := '1';
    signal s_field_n_d : std_logic := '1';

begin

    -- ========================================================================
    -- Register mapping
    -- ========================================================================
    s_k1  <= unsigned(registers_in(0));
    s_k2  <= unsigned(registers_in(1));
    s_k3  <= unsigned(registers_in(2));
    s_k4  <= unsigned(registers_in(3));
    s_k5  <= unsigned(registers_in(4));
    s_k6  <= unsigned(registers_in(5));
    s_s7  <= registers_in(6)(0);
    s_s8  <= registers_in(6)(1);
    s_s9  <= registers_in(6)(2);
    s_s10 <= registers_in(6)(3);
    s_s11 <= registers_in(6)(4);
    s_k12 <= unsigned(registers_in(7));

    -- ========================================================================
    -- Video Timing Generator
    -- ========================================================================
    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    -- ========================================================================
    -- Vsync latch
    -- ========================================================================
    p_vsync_latch : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                r_freq(0) <= s_k1;
                r_freq(1) <= s_k2;
                r_freq(2) <= s_k3;
                r_lfo(0)  <= s_k4;
                r_lfo(1)  <= s_k5;
                r_lfo(2)  <= s_k6;
                r_thick(0)  <= s_s7;
                r_thick(1)  <= s_s8;
                r_thick(2)  <= s_s9;
                -- mode = (S11, '1', S10): selects MIN/DIFF/AND/OR.
                r_mode      <= s_s11 & '1' & s_s10;
                r_slider    <= s_k12;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Slider -> global thickness threshold.
    --   threshold = -slider * 31/64, range -495..0
    -- At slider=0 the threshold is 0 (50% duty, 1x thickness).
    -- At slider=1023 it's ~-495 (91% duty, ~10:1 bright:dark, 10x thickness).
    -- ========================================================================
    s_slider_thresh <= -signed(resize(
        shift_right(r_slider, 1) - shift_right(r_slider, 6), 12));

    -- ========================================================================
    -- LFO Accumulators (per-line, per-osc).  Step is computed combinationally
    -- from K_lfo and clamped after adding r_fm_mod (so heavy FM doesn't wrap
    -- a positive step to garbage when the modulation goes negative).
    -- ========================================================================
    gen_lfo_step : for i in 0 to C_N_OSCS-1 generate
        signal s_base_step : signed(13 downto 0);
        signal s_step_sum  : signed(13 downto 0);
    begin
        g0 : if i = 0 generate
            s_base_step <= signed(resize(
                shift_right(r_lfo(i) * to_unsigned(C_LFO_RANGE_0, 8), 10),
                14)) + to_signed(C_LFO_FLOOR_0, 14);
        end generate;
        g1 : if i = 1 generate
            s_base_step <= signed(resize(
                shift_right(r_lfo(i) * to_unsigned(C_LFO_RANGE_1, 8), 10),
                14)) + to_signed(C_LFO_FLOOR_1, 14);
        end generate;
        g2 : if i = 2 generate
            s_base_step <= signed(resize(
                shift_right(r_lfo(i) * to_unsigned(C_LFO_RANGE_2, 8), 10),
                14)) + to_signed(C_LFO_FLOOR_2, 14);
        end generate;

        s_step_sum <= s_base_step;

        s_lfo_step_slv(i) <= std_logic_vector(resize(unsigned(s_step_sum(12 downto 0)), C_LFO_W));
    end generate;

    gen_lfo : for i in 0 to C_N_OSCS-1 generate
        lfo_inst : entity work.video_timing_accumulator
            generic map (G_ACCUMULATOR_WIDTH => C_LFO_W)
            port map (
                clk           => clk,
                i_timing      => s_timing,
                i_range       => C_VERTICAL,
                i_reset       => '0',
                i_lock        => '0',
                i_accumulator => s_lfo_step_slv(i),
                o_accumulator => s_lfo_phase_slv(i),
                o_clock       => open,
                o_pulse       => open
            );

        s_lfo_phase(i) <= unsigned(s_lfo_phase_slv(i)(C_LFO_W-1 downto C_LFO_W-10));
    end generate;

    -- ========================================================================
    -- Shared LFO sin LUT, time-muxed across 3 LFOs (3 cycles per hsync).
    -- Unconditional output register so synth packs the LUT into BRAM.
    -- ========================================================================
    p_lfo_mux_input : process(r_lfo_mux_idx, s_lfo_phase)
    begin
        s_lfo_lut_in <= std_logic_vector(s_lfo_phase(0));
        case to_integer(r_lfo_mux_idx) is
            when 0      => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(0));
            when 1      => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(1));
            when others => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(2));
        end case;
    end process;

    lfo_lut_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => s_lfo_lut_in,
            sin_out  => s_lfo_lut_sin,
            cos_out  => open
        );

    p_lfo_lut_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_lfo_lut_sin <= s_lfo_lut_sin;
        end if;
    end process;

    p_lfo_mux_state : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                r_lfo_mux_idx <= (others => '0');
            elsif r_lfo_mux_idx < to_unsigned(C_N_OSCS, 3) then
                r_lfo_mux_idx <= r_lfo_mux_idx + 1;
            end if;
            r_lfo_mux_idx_d <= r_lfo_mux_idx;

            if r_lfo_mux_idx_d < to_unsigned(C_N_OSCS, 3) then
                r_lfo_sin(to_integer(r_lfo_mux_idx_d)) <= r_lfo_lut_sin;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Carrier Accumulators (per-pixel, reset at line start)
    --   inc(i) = (freq(i) << 5) + min_floor
    -- ========================================================================
    gen_carriers : for i in 0 to C_N_OSCS-1 generate
        s_osc_inc_slv(i) <= std_logic_vector(
            shift_left(resize(r_freq(i), C_CARRIER_W), C_FREQ_KNOB_SH)
            + to_unsigned(C_FREQ_MIN, C_CARRIER_W));

        osc_inst : entity work.video_timing_accumulator
            generic map (G_ACCUMULATOR_WIDTH => C_CARRIER_W)
            port map (
                clk           => clk,
                i_timing      => s_timing,
                i_range       => C_HORIZONTAL,
                i_reset       => '0',
                i_lock        => '1',
                i_accumulator => s_osc_inc_slv(i),
                o_accumulator => s_osc_acc_slv(i),
                o_clock       => open,
                o_pulse       => open
            );

        s_osc_acc_top(i) <= unsigned(s_osc_acc_slv(i)(C_CARRIER_W-1 downto C_CARRIER_W-10));
    end generate;

    -- ========================================================================
    -- Scroll
    -- ========================================================================
    scroll_inst : entity work.frame_phase_accumulator
        generic map (G_PHASE_WIDTH => C_SCROLL_W, G_SPEED_WIDTH => 10)
        port map (
            clk     => clk,
            vsync_n => data_in.vsync_n,
            enable  => '1',
            speed   => C_SCROLL_RATE,
            phase   => s_scroll_phase
        );
    s_scroll_top <= s_scroll_phase(C_SCROLL_W-1 downto C_SCROLL_W-10);

    -- ========================================================================
    -- PM phase = carrier_top + lfo_sin + scroll_top (mod 1024)
    -- ========================================================================
    gen_pm : for i in 0 to C_N_OSCS-1 generate
        s_osc_pm_phase(i) <= s_osc_acc_top(i)
                           + unsigned(std_logic_vector(r_lfo_sin(i)))
                           + s_scroll_top;
    end generate;

    p_pm_reg : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to C_N_OSCS-1 loop
                r_osc_pm_phase(i) <= s_osc_pm_phase(i);
            end loop;
        end if;
    end process;

    -- ========================================================================
    -- Carrier sin LUTs (per-pixel, per-osc) -> luma register
    -- ========================================================================
    gen_carrier_luts : for i in 0 to C_N_OSCS-1 generate
        sin_inst : entity work.sin_cos_full_lut_10x10
            port map (
                angle_in => std_logic_vector(r_osc_pm_phase(i)),
                sin_out  => s_osc_sin(i),
                cos_out  => open
            );
    end generate;

    p_carrier_sin_reg : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to C_N_OSCS-1 loop
                r_osc_sin_r(i) <= s_osc_sin(i);
            end loop;
        end if;
    end process;

    -- Pre-register per-osc threshold so the slider/switch mux is out of
    -- the luma combinational path.
    p_thresh_reg : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to C_N_OSCS-1 loop
                if r_thick(i) = '1' then
                    r_osc_thresh(i) <= s_slider_thresh;
                else
                    r_osc_thresh(i) <= (others => '0');
                end if;
            end loop;
        end if;
    end process;

    -- Single-stage luma: half-wave rectify with threshold + 3/2 scale + 767 cap.
    p_luma_reg : process(clk)
        variable v_diff     : signed(12 downto 0);
        variable v_luma_raw : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to C_N_OSCS-1 loop
                v_diff := resize(r_osc_sin_r(i), 13) - resize(r_osc_thresh(i), 13);
                if v_diff(12) = '1' then
                    -- sin < threshold (negative diff) -> dark
                    r_osc_luma(i) <= (others => '0');
                else
                    v_luma_raw := resize(v_diff, 14) + resize(shift_right(v_diff, 1), 14);
                    if v_luma_raw > to_signed(767, 14) then
                        r_osc_luma(i) <= to_unsigned(767, 10);
                    else
                        r_osc_luma(i) <= unsigned(v_luma_raw(9 downto 0));
                    end if;
                end if;
            end loop;
        end if;
    end process;

    -- ========================================================================
    -- Keyer pre-stage: compute and register max/min/sum/and/or in parallel.
    -- ========================================================================
    p_keyer_pre : process(clk)
        variable v_l0, v_l1, v_l2 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_l0 := r_osc_luma(0);
            v_l1 := r_osc_luma(1);
            v_l2 := r_osc_luma(2);

            -- MAX (used by DIFF mode for color selection; low-index tie-break)
            if v_l0 >= v_l1 and v_l0 >= v_l2 then
                r_max_val <= v_l0; r_max_idx <= "00";
            elsif v_l1 >= v_l2 then
                r_max_val <= v_l1; r_max_idx <= "01";
            else
                r_max_val <= v_l2; r_max_idx <= "10";
            end if;

            if v_l0 <= v_l1 and v_l0 <= v_l2 then
                r_min_val <= v_l0; r_min_idx <= "00";
            elsif v_l1 <= v_l2 then
                r_min_val <= v_l1; r_min_idx <= "01";
            else
                r_min_val <= v_l2; r_min_idx <= "10";
            end if;

            r_sum_y <= resize(v_l0, 12) + resize(v_l1, 12) + resize(v_l2, 12);
            r_and_y <= v_l0 and v_l1 and v_l2;
            r_or_y  <= v_l0 or  v_l1 or  v_l2;
            r_l0_d  <= v_l0;
        end if;
    end process;

    -- ========================================================================
    -- Output stage: 8-mode blend mux over registered intermediates.
    -- Colors are constant per osc (C_OSC_U, C_OSC_V).
    -- ========================================================================
    p_output : process(clk)
        variable v_y : unsigned(9 downto 0);
        variable v_u : unsigned(9 downto 0);
        variable v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            case r_mode is
                when "000" =>  -- MAX
                    v_y := r_max_val;
                    case r_max_idx is
                        when "00" =>   v_u := C_OSC_U(0); v_v := C_OSC_V(0);
                        when "01" =>   v_u := C_OSC_U(1); v_v := C_OSC_V(1);
                        when others => v_u := C_OSC_U(2); v_v := C_OSC_V(2);
                    end case;
                when "001" =>  -- ADD saturating
                    if r_sum_y > to_unsigned(1023, 12) then
                        v_y := (others => '1');
                    else
                        v_y := r_sum_y(9 downto 0);
                    end if;
                    v_u := C_UV_MID;
                    v_v := C_UV_MID;
                when "010" =>  -- MIN
                    v_y := r_min_val;
                    case r_min_idx is
                        when "00" =>   v_u := C_OSC_U(0); v_v := C_OSC_V(0);
                        when "01" =>   v_u := C_OSC_U(1); v_v := C_OSC_V(1);
                        when others => v_u := C_OSC_U(2); v_v := C_OSC_V(2);
                    end case;
                when "011" =>  -- DIFF (max - min)
                    v_y := r_max_val - r_min_val;
                    case r_max_idx is
                        when "00" =>   v_u := C_OSC_U(0); v_v := C_OSC_V(0);
                        when "01" =>   v_u := C_OSC_U(1); v_v := C_OSC_V(1);
                        when others => v_u := C_OSC_U(2); v_v := C_OSC_V(2);
                    end case;
                when "100" =>  -- AVG (sum * 5/16 ~= sum/3.2)
                    v_y := resize(shift_right(r_sum_y, 2) + shift_right(r_sum_y, 4), 10);
                    v_u := C_UV_MID;
                    v_v := C_UV_MID;
                when "101" =>  -- HARDKEY: OSC1 luma threshold over MAX(OSC2,OSC3)
                    if r_l0_d > to_unsigned(512, 10) then
                        v_y := r_l0_d; v_u := C_OSC_U(0); v_v := C_OSC_V(0);
                    else
                        v_y := r_max_val;
                        case r_max_idx is
                            when "00" =>   v_u := C_OSC_U(0); v_v := C_OSC_V(0);
                            when "01" =>   v_u := C_OSC_U(1); v_v := C_OSC_V(1);
                            when others => v_u := C_OSC_U(2); v_v := C_OSC_V(2);
                        end case;
                    end if;
                when "110" =>  -- AND bitwise
                    v_y := r_and_y;
                    v_u := C_UV_MID;
                    v_v := C_UV_MID;
                when others =>  -- "111" OR bitwise
                    v_y := r_or_y;
                    case r_max_idx is
                        when "00" =>   v_u := C_OSC_U(0); v_v := C_OSC_V(0);
                        when "01" =>   v_u := C_OSC_U(1); v_v := C_OSC_V(1);
                        when others => v_u := C_OSC_U(2); v_v := C_OSC_V(2);
                    end case;
            end case;

            r_y_out <= v_y;
            r_u_out <= v_u;
            r_v_out <= v_v;
        end if;
    end process;

    -- ========================================================================
    -- Sync delay
    -- ========================================================================
    p_sync_delay : process(clk)
        type t_sdly is array (0 to C_DELAY_CLKS-1) of std_logic;
        variable v_avid : t_sdly := (others => '0');
        variable v_hs   : t_sdly := (others => '1');
        variable v_vs   : t_sdly := (others => '1');
        variable v_fd   : t_sdly := (others => '1');
    begin
        if rising_edge(clk) then
            v_avid := data_in.avid    & v_avid(0 to C_DELAY_CLKS-2);
            v_hs   := data_in.hsync_n & v_hs  (0 to C_DELAY_CLKS-2);
            v_vs   := data_in.vsync_n & v_vs  (0 to C_DELAY_CLKS-2);
            v_fd   := data_in.field_n & v_fd  (0 to C_DELAY_CLKS-2);
            s_avid_d    <= v_avid(C_DELAY_CLKS-1);
            s_hsync_n_d <= v_hs  (C_DELAY_CLKS-1);
            s_vsync_n_d <= v_vs  (C_DELAY_CLKS-1);
            s_field_n_d <= v_fd  (C_DELAY_CLKS-1);
        end if;
    end process;

    data_out.y       <= std_logic_vector(r_y_out);
    data_out.u       <= std_logic_vector(r_u_out);
    data_out.v       <= std_logic_vector(r_v_out);
    data_out.avid    <= s_avid_d;
    data_out.hsync_n <= s_hsync_n_d;
    data_out.vsync_n <= s_vsync_n_d;
    data_out.field_n <= s_field_n_d;

end architecture trioweave;
