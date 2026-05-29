-- Trioweave: 3 sine VCOs each with own Freq + LFO, FM-modulated by slider.
--
-- Knob layout:
--   K1 OSC1 Freq   K2 OSC1 LFO rate
--   K3 OSC2 Freq   K4 OSC2 LFO rate
--   K5 OSC3 Freq   K6 OSC3 LFO rate
--   S7 Key polarity / tie-break
--   S8/S9/S10 Blend mode (8 options, bajaweave-style)
--   S11 Scroll on/off
--   S12 FM Depth -- scales how much a global FM source modulates each LFO rate
--
-- Colors: fixed RGB-like triad (compile-time U/V constants, no hue LUT).
--
-- FM source: a dedicated frame-rate phase accumulator runs at a fixed slow
-- speed.  Its triangle output is multiplied by the slider and added to each
-- per-osc LFO step.  Slider=0 -> no FM, LFOs steady at their knob rates.
-- Slider=max -> LFO rates swing by ~+/-50% of base, creating a slow
-- "breathing" wobble in the modulation rate that compounds with each
-- osc's individual LFO rate.
--
-- Resources:
--   3 carrier sin LUTs (per-pixel)
--   1 shared LFO sin LUT (time-muxed across 3 oscs during hsync blanking)
--   1 FM source phase accumulator (frame_phase_accumulator, free-running)
--   1 small 10b*11b signed multiplier for FM_value * slider
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
    constant C_FM_W          : integer := 16;

    constant C_FREQ_KNOB_SH  : integer := 5;
    constant C_FREQ_MIN      : integer := 1456;
    constant C_LFO_FLOOR     : integer := 816;
    constant C_SCROLL_RATE   : unsigned(9 downto 0) := to_unsigned(128, 10);
    constant C_FM_RATE       : unsigned(9 downto 0) := to_unsigned(96, 10);
    constant C_FM_SCALE_SH   : integer := 8;

    -- Fixed UV per osc (matches old trioweave defaults: hue 0, 340, 680).
    type t_uv_array is array (0 to C_N_OSCS-1) of unsigned(9 downto 0);
    constant C_OSC_U : t_uv_array := (
        to_unsigned(767, 10),
        to_unsigned(387, 10),
        to_unsigned(381, 10));
    constant C_OSC_V : t_uv_array := (
        to_unsigned(512, 10),
        to_unsigned(735, 10),
        to_unsigned(292, 10));
    constant C_AVG_U : unsigned(9 downto 0) := to_unsigned(511, 10);  -- mean of C_OSC_U
    constant C_AVG_V : unsigned(9 downto 0) := to_unsigned(513, 10);  -- mean of C_OSC_V

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
    signal r_keypol    : std_logic := '0';
    signal r_mode      : unsigned(2 downto 0) := (others => '0');
    signal r_scroll_en : std_logic := '1';
    signal r_slider    : unsigned(9 downto 0) := (others => '0');

    -- Timing record
    signal s_timing : t_video_timing_port;

    -- ====================================================================
    -- FM source: free-running per-frame phase accumulator.
    -- Triangle of its top 10 bits is the modulating signal.
    -- ====================================================================
    signal s_fm_phase     : unsigned(C_FM_W-1 downto 0);
    signal s_fm_phase_top : unsigned(9 downto 0);
    signal s_fm_tri       : signed(9 downto 0);
    signal r_fm_value     : signed(9 downto 0) := (others => '0');
    -- FM_value * slider, scaled to a magnitude similar to the LFO step range
    signal r_fm_mod       : signed(12 downto 0) := (others => '0');

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
                r_lfo(0)  <= s_k2;
                r_freq(1) <= s_k3;
                r_lfo(1)  <= s_k4;
                r_freq(2) <= s_k5;
                r_lfo(2)  <= s_k6;
                r_keypol    <= s_s7;
                r_mode      <= s_s10 & s_s9 & s_s8;
                r_scroll_en <= s_s11;
                r_slider    <= s_k12;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- FM Source: free-running per-frame phase, triangle output.
    -- Multiplied by slider value to get the modulation magnitude shared
    -- by all per-osc LFO step calculations.
    -- ========================================================================
    fm_inst : entity work.frame_phase_accumulator
        generic map (G_PHASE_WIDTH => C_FM_W, G_SPEED_WIDTH => 10)
        port map (
            clk     => clk,
            vsync_n => data_in.vsync_n,
            enable  => '1',
            speed   => C_FM_RATE,
            phase   => s_fm_phase
        );

    s_fm_phase_top <= s_fm_phase(C_FM_W-1 downto C_FM_W-10);
    s_fm_tri       <= tri_fold(s_fm_phase_top);

    p_fm_mod : process(clk)
        variable v_prod : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            r_fm_value <= s_fm_tri;
            -- 10b signed * 11b signed (slider extended to positive 11b) = 21b signed
            v_prod := r_fm_value * signed(resize(r_slider, 11));
            r_fm_mod <= resize(shift_right(v_prod, C_FM_SCALE_SH), 13);
        end if;
    end process;

    -- ========================================================================
    -- LFO Accumulators (per-line, per-osc).  Step is computed combinationally
    -- from K_lfo and clamped after adding r_fm_mod (so heavy FM doesn't wrap
    -- a positive step to garbage when the modulation goes negative).
    -- ========================================================================
    gen_lfo_step : for i in 0 to C_N_OSCS-1 generate
        signal s_base_step : signed(13 downto 0);
        signal s_step_sum  : signed(13 downto 0);
    begin
        s_base_step <= signed(resize(
            shift_left(resize(r_lfo(i), 12), 1)
            + resize(r_lfo(i), 12)
            + to_unsigned(C_LFO_FLOOR, 12), 14));
        s_step_sum <= s_base_step + resize(r_fm_mod, 14);

        -- Clamp negative to 0 (avoid step underflow turning into huge wrap)
        s_lfo_step_slv(i) <= std_logic_vector(to_unsigned(0, C_LFO_W))
                                when s_step_sum(13) = '1'
                            else std_logic_vector(resize(unsigned(s_step_sum(12 downto 0)), C_LFO_W));
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
            enable  => r_scroll_en,
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

    p_luma_reg : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to C_N_OSCS-1 loop
                r_osc_luma(i) <= unsigned(resize(r_osc_sin_r(i) + to_signed(512, 11), 10));
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

            if r_keypol = '0' then
                if v_l0 >= v_l1 and v_l0 >= v_l2 then
                    r_max_val <= v_l0; r_max_idx <= "00";
                elsif v_l1 >= v_l2 then
                    r_max_val <= v_l1; r_max_idx <= "01";
                else
                    r_max_val <= v_l2; r_max_idx <= "10";
                end if;
            else
                if v_l2 >= v_l1 and v_l2 >= v_l0 then
                    r_max_val <= v_l2; r_max_idx <= "10";
                elsif v_l1 >= v_l0 then
                    r_max_val <= v_l1; r_max_idx <= "01";
                else
                    r_max_val <= v_l0; r_max_idx <= "00";
                end if;
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
                    v_u := C_AVG_U;
                    v_v := C_AVG_V;
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
                    v_u := C_AVG_U;
                    v_v := C_AVG_V;
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
                    v_u := C_AVG_U;
                    v_v := C_AVG_V;
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
