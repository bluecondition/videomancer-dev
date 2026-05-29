-- Oscbank: 8 auto-spread sine oscillators with shared LFO + hue LUTs.
--
-- Knob layout:
--   K1 Base Freq    -- master spatial frequency (1..~22 cycles/screen)
--   K2 Freq Spread  -- per-osc frequency offset
--   K3 LFO Rate     -- master LFO/wobble rate
--   K4 LFO Spread   -- per-osc LFO rate variation
--   K5 Base Hue     -- starting hue (0..360 deg)
--   K6 Rainbow      -- how far the bank spreads across the color wheel
--   S7 Tie Break    -- low-index vs high-index wins ties in MAX/MIN
--   S8/S9 Blend     -- 0 MAX  1 ADD-sat  2 MIN  3 AVG
--   S10 Invert      -- Y -> 1023-Y at output
--   S11 Scroll      -- on/off (fixed ~8 s/cycle)
--   S12 Shape       -- sine <-> triangle morph for carrier and LFO
--
-- Per-osc i (i in 0..7):
--   freq(i)  = base_freq  + i * freq_spread
--   lfo(i)   = lfo_rate   + i * lfo_spread / 4   (smaller per-osc step)
--   hue(i)   = base_hue   + i * rainbow / 8
--
-- Resource sharing:
--   * 1 shared sin LUT for all 8 LFOs, time-muxed during hsync blanking
--     (8 cycles of mux, each registers r_lfo_sin(i)).
--   * 1 shared sin/cos LUT for all 8 hues, time-muxed during vsync.
--   * 8 dedicated per-pixel carrier sin LUTs (the BRAM hogs).
--
-- Pipeline (data_in -> data_out):
--    1 vt_gen
--    2 LFO accumulator
--    1 LFO sin / tri register (mux output captured)
--    1 LFO morph register
--    1 PM phase register
--    1 carrier sin / tri register
--    1 luma register
--    3 keyer max tree
--    1 output mode mux + register
--   = 11 clocks total
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

architecture oscbank of program_top is

    constant C_N_OSCS        : integer := 8;
    constant C_N_OSCS_LOG2   : integer := 3;

    constant C_DELAY_CLKS    : integer := 11;
    constant C_CARRIER_W     : integer := 20;
    constant C_LFO_W         : integer := 20;
    constant C_SCROLL_W      : integer := 16;

    constant C_FREQ_KNOB_SH  : integer := 5;
    constant C_FREQ_MIN      : integer := 1456;
    constant C_LFO_FLOOR     : integer := 816;
    constant C_SCROLL_RATE   : unsigned(9 downto 0) := to_unsigned(128, 10);

    constant C_UV_MID        : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ====================================================================
    -- Triangle and shape-morph functions (same as bajaweave milestone).
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

    function shape_morph(sin_in : signed(9 downto 0);
                         tri_in : signed(9 downto 0);
                         shape  : unsigned(1 downto 0)) return signed is
        variable v_sum  : signed(10 downto 0);
        variable v_diff : signed(10 downto 0);
        variable v_out  : signed(9 downto 0);
    begin
        case shape is
            when "00" =>
                v_out := sin_in;
            when "01" =>
                v_sum := resize(sin_in, 11) + resize(tri_in, 11);
                v_out := resize(shift_right(v_sum, 1), 10);
            when "10" =>
                v_diff := resize(tri_in, 11) - resize(sin_in, 11);
                v_out := tri_in - resize(shift_right(v_diff, 2), 10);
            when others =>
                v_out := tri_in;
        end case;
        return v_out;
    end function;

    -- ====================================================================
    -- Combinational knob view
    -- ====================================================================
    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0);
    signal s_k4, s_k5, s_k6 : unsigned(9 downto 0);
    signal s_s7, s_s8, s_s9, s_s10, s_s11 : std_logic;
    signal s_k12 : unsigned(9 downto 0);

    -- Vsync-latched
    signal r_base_freq    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal r_freq_spread  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal r_lfo_rate     : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal r_lfo_spread   : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal r_base_hue     : unsigned(9 downto 0) := (others => '0');
    signal r_rainbow      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_tiebreak     : std_logic := '0';
    signal r_mode         : unsigned(1 downto 0) := (others => '0');
    signal r_invert       : std_logic := '0';
    signal r_scroll_en    : std_logic := '1';
    signal r_shape        : unsigned(9 downto 0) := (others => '0');

    -- Timing record
    signal s_timing : t_video_timing_port;

    -- ====================================================================
    -- Per-osc state arrays
    -- ====================================================================
    type t_acc_slv  is array (0 to C_N_OSCS-1) of std_logic_vector(C_CARRIER_W-1 downto 0);
    type t_lfo_slv  is array (0 to C_N_OSCS-1) of std_logic_vector(C_LFO_W-1 downto 0);
    type t_phase10  is array (0 to C_N_OSCS-1) of unsigned(9 downto 0);
    type t_signed10 is array (0 to C_N_OSCS-1) of signed(9 downto 0);
    type t_uns10    is array (0 to C_N_OSCS-1) of unsigned(9 downto 0);

    -- LFO acc IO
    signal s_lfo_step_slv  : t_lfo_slv;
    signal s_lfo_phase_slv : t_lfo_slv;
    signal s_lfo_phase     : t_phase10;

    -- LFO time-mux (1 shared sin LUT)
    signal r_lfo_mux_idx   : unsigned(3 downto 0) := (others => '0');
    signal r_lfo_mux_idx_d : unsigned(3 downto 0) := (others => '0');
    signal s_lfo_lut_in    : std_logic_vector(9 downto 0);
    signal s_lfo_lut_sin   : signed(9 downto 0);
    signal r_lfo_lut_sin   : signed(9 downto 0) := (others => '0');  -- unconditional LUT output reg

    -- Per-osc registered LFO sin (output of time-mux) and triangle (combinational)
    signal r_lfo_sin_r : t_signed10 := (others => (others => '0'));
    signal s_lfo_tri   : t_signed10;
    signal r_lfo_tri_r : t_signed10 := (others => (others => '0'));

    -- Morphed LFO (sine<->tri blend)
    signal s_lfo_morph : t_signed10;
    signal r_lfo_sin   : t_signed10 := (others => (others => '0'));

    -- Carrier acc IO
    signal s_osc_inc_slv : t_acc_slv;
    signal s_osc_acc_slv : t_acc_slv;
    signal s_osc_acc_top : t_phase10;

    -- Scroll
    signal s_scroll_phase : unsigned(C_SCROLL_W-1 downto 0);
    signal s_scroll_top   : unsigned(9 downto 0);

    -- PM phase (combinational + registered)
    signal s_osc_pm_phase : t_phase10;
    signal r_osc_pm_phase : t_phase10 := (others => (others => '0'));

    -- Carrier sin LUTs
    signal s_osc_sin   : t_signed10;
    signal s_osc_tri   : t_signed10;
    signal r_osc_sin_r : t_signed10 := (others => (others => '0'));
    signal r_osc_tri_r : t_signed10 := (others => (others => '0'));

    -- Shape-morphed carrier
    signal s_osc_shaped : t_signed10;

    -- Per-osc luma
    signal r_osc_luma : t_uns10 := (others => (others => '0'));

    -- ====================================================================
    -- Hue time-mux state.  Uses sin-only LUT, reading 2N times per vsync:
    -- even counter cycles read sin(hue(i)) -> V slot; odd cycles read
    -- sin(hue(i) + 256) = cos(hue(i)) -> U slot.
    -- ====================================================================
    signal r_hue_cnt   : unsigned(4 downto 0) := (others => '0');  -- 0..2N
    signal r_hue_cnt_d : unsigned(4 downto 0) := (others => '0');
    signal r_hue_active   : std_logic := '0';
    signal r_hue_active_d : std_logic := '0';
    signal s_hue_lut_in   : std_logic_vector(9 downto 0);
    signal s_hue_lut_sin  : signed(9 downto 0);
    signal r_hue_lut_sin  : signed(9 downto 0) := (others => '0');

    -- Per-osc hue and registered U/V
    signal s_osc_hue : t_phase10;
    signal r_u      : t_uns10 := (others => to_unsigned(512, 10));
    signal r_v      : t_uns10 := (others => to_unsigned(512, 10));

    -- Average U/V (for ADD/AVG blend modes), precomputed per-frame
    signal r_avg_u  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_avg_v  : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ====================================================================
    -- Keyer tree (max + min + sum, 3 stages for N=8, balanced binary tree)
    -- Stage 1: 4 pairs -> 4 outputs
    -- Stage 2: 2 pairs -> 2 outputs
    -- Stage 3: 1 final pair -> 1 output
    -- ====================================================================
    type t_s1_luma is array (0 to 3) of unsigned(9 downto 0);
    type t_s1_idx  is array (0 to 3) of unsigned(2 downto 0);
    type t_s2_luma is array (0 to 1) of unsigned(9 downto 0);
    type t_s2_idx  is array (0 to 1) of unsigned(2 downto 0);

    -- Max tree
    signal r_s1_max     : t_s1_luma := (others => (others => '0'));
    signal r_s1_max_idx : t_s1_idx  := (others => (others => '0'));
    signal r_s2_max     : t_s2_luma := (others => (others => '0'));
    signal r_s2_max_idx : t_s2_idx  := (others => (others => '0'));
    signal r_s3_max     : unsigned(9 downto 0) := (others => '0');
    signal r_s3_max_idx : unsigned(2 downto 0) := (others => '0');

    -- Min tree (parallel)
    signal r_s1_min     : t_s1_luma := (others => (others => '0'));
    signal r_s1_min_idx : t_s1_idx  := (others => (others => '0'));
    signal r_s2_min     : t_s2_luma := (others => (others => '0'));
    signal r_s2_min_idx : t_s2_idx  := (others => (others => '0'));
    signal r_s3_min     : unsigned(9 downto 0) := (others => '0');
    signal r_s3_min_idx : unsigned(2 downto 0) := (others => '0');

    -- Sum tree (widening per stage). For N=8 -> max sum = 8*1023 = 8184 -> 13b.
    type t_s1_sum is array (0 to 3) of unsigned(10 downto 0);
    type t_s2_sum is array (0 to 1) of unsigned(11 downto 0);
    signal r_s1_sum : t_s1_sum := (others => (others => '0'));
    signal r_s2_sum : t_s2_sum := (others => (others => '0'));
    signal r_s3_sum : unsigned(12 downto 0) := (others => '0');

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
                r_base_freq   <= s_k1;
                r_freq_spread <= s_k2;
                r_lfo_rate    <= s_k3;
                r_lfo_spread  <= s_k4;
                r_base_hue    <= s_k5;
                r_rainbow     <= s_k6;
                r_tiebreak    <= s_s7;
                r_mode        <= s_s9 & s_s8;
                r_invert      <= s_s10;
                r_scroll_en   <= s_s11;
                r_shape       <= s_k12;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- LFO accumulators (N parallel, per-line, C_VERTICAL)
    --   lfo_step(i) = 3 * lfo_rate + (i * lfo_spread) / 4 + floor
    -- ========================================================================
    gen_lfo : for i in 0 to C_N_OSCS-1 generate
        s_lfo_step_slv(i) <= std_logic_vector(
            shift_left(resize(r_lfo_rate, C_LFO_W), 1)
            + resize(r_lfo_rate, C_LFO_W)
            + resize(shift_right(r_lfo_spread * to_unsigned(i, 4), 2), C_LFO_W)
            + to_unsigned(C_LFO_FLOOR, C_LFO_W));

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
        s_lfo_tri(i)   <= tri_fold(s_lfo_phase(i));
    end generate;

    -- ========================================================================
    -- Shared LFO sin LUT, time-muxed across the 8 LFOs.
    -- At hsync_start the mux index resets to 0; the next 8 clocks cycle the
    -- LUT input through each LFO's phase and register the output.
    -- ========================================================================
    p_lfo_mux_input : process(r_lfo_mux_idx, s_lfo_phase)
    begin
        s_lfo_lut_in <= std_logic_vector(s_lfo_phase(0));
        case to_integer(r_lfo_mux_idx) is
            when 0 => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(0));
            when 1 => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(1));
            when 2 => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(2));
            when 3 => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(3));
            when 4 => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(4));
            when 5 => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(5));
            when 6 => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(6));
            when others => s_lfo_lut_in <= std_logic_vector(s_lfo_phase(7));
        end case;
    end process;

    lfo_lut_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => s_lfo_lut_in,
            sin_out  => s_lfo_lut_sin,
            cos_out  => open
        );

    -- Unconditionally register the LUT output every clock.  This is the
    -- pattern that lets yosys/nextpnr infer the BRAM output register, so
    -- the LUT lands in BRAM instead of LCs.
    p_lfo_lut_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_lfo_lut_sin <= s_lfo_lut_sin;
        end if;
    end process;

    -- Mux state + demux to per-osc slot uses the DELAYED index, matching
    -- the 1-clock latency of r_lfo_lut_sin.
    p_lfo_mux_state : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                r_lfo_mux_idx <= (others => '0');
            elsif r_lfo_mux_idx < to_unsigned(C_N_OSCS, 4) then
                r_lfo_mux_idx <= r_lfo_mux_idx + 1;
            end if;
            r_lfo_mux_idx_d <= r_lfo_mux_idx;

            if r_lfo_mux_idx_d < to_unsigned(C_N_OSCS, 4) then
                r_lfo_sin_r(to_integer(r_lfo_mux_idx_d)) <= r_lfo_lut_sin;
                r_lfo_tri_r(to_integer(r_lfo_mux_idx_d)) <= s_lfo_tri(to_integer(r_lfo_mux_idx_d));
            end if;
        end if;
    end process;

    -- LFO morph mux per osc, then register
    gen_lfo_morph : for i in 0 to C_N_OSCS-1 generate
        s_lfo_morph(i) <= shape_morph(r_lfo_sin_r(i), r_lfo_tri_r(i), r_shape(9 downto 8));
    end generate;

    p_lfo_reg : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to C_N_OSCS-1 loop
                r_lfo_sin(i) <= s_lfo_morph(i);
            end loop;
        end if;
    end process;

    -- ========================================================================
    -- Carrier accumulators (N parallel, per-pixel, C_HORIZONTAL)
    --   inc(i) = (base << 5) + (i * spread) + min_floor
    -- ========================================================================
    gen_carriers : for i in 0 to C_N_OSCS-1 generate
        s_osc_inc_slv(i) <= std_logic_vector(
            shift_left(resize(r_base_freq, C_CARRIER_W), C_FREQ_KNOB_SH)
            + resize(r_freq_spread * to_unsigned(i, 4), C_CARRIER_W)
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
    -- Scroll accumulator
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
    -- Carrier sin LUTs (N instances, per-pixel) + triangle (combinational)
    -- ========================================================================
    gen_carrier_luts : for i in 0 to C_N_OSCS-1 generate
        sin_inst : entity work.sin_cos_full_lut_10x10
            port map (
                angle_in => std_logic_vector(r_osc_pm_phase(i)),
                sin_out  => s_osc_sin(i),
                cos_out  => open
            );
        s_osc_tri(i) <= tri_fold(r_osc_pm_phase(i));
    end generate;

    p_carrier_sin_reg : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to C_N_OSCS-1 loop
                r_osc_sin_r(i) <= s_osc_sin(i);
                r_osc_tri_r(i) <= s_osc_tri(i);
            end loop;
        end if;
    end process;

    gen_carrier_morph : for i in 0 to C_N_OSCS-1 generate
        s_osc_shaped(i) <= shape_morph(r_osc_sin_r(i), r_osc_tri_r(i), r_shape(9 downto 8));
    end generate;

    p_luma_reg : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to C_N_OSCS-1 loop
                r_osc_luma(i) <= unsigned(resize(s_osc_shaped(i) + to_signed(512, 11), 10));
            end loop;
        end if;
    end process;

    -- ========================================================================
    -- Hue: per-osc hue computed combinationally
    --   hue(i) = base_hue + (i * rainbow) / 8   (mod 1024)
    -- ========================================================================
    gen_hue : for i in 0 to C_N_OSCS-1 generate
        s_osc_hue(i) <= r_base_hue
                      + resize(shift_right(r_rainbow * to_unsigned(i, 4), 3), 10);
    end generate;

    -- ========================================================================
    -- Shared hue LUT, time-muxed during vsync.
    -- The state machine activates on vsync_start and reads N hue angles
    -- over the next N clocks.  Sin/cos outputs land in r_u/r_v.
    -- ========================================================================
    -- Mux angle into the shared sin LUT.  Even cnt = pure hue, odd cnt = hue+256
    -- (so the same sin LUT delivers a cos for U on odd cycles).
    p_hue_mux_input : process(r_hue_cnt, s_osc_hue)
        variable v_idx    : integer range 0 to C_N_OSCS-1;
        variable v_angle  : unsigned(9 downto 0);
    begin
        v_idx := to_integer(r_hue_cnt(3 downto 1));
        v_angle := s_osc_hue(v_idx);
        if r_hue_cnt(0) = '1' then
            v_angle := v_angle + to_unsigned(256, 10);
        end if;
        s_hue_lut_in <= std_logic_vector(v_angle);
    end process;

    hue_lut_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => s_hue_lut_in,
            sin_out  => s_hue_lut_sin,
            cos_out  => open
        );

    -- Unconditional LUT output register for BRAM packing.
    p_hue_lut_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_hue_lut_sin <= s_hue_lut_sin;
        end if;
    end process;

    -- Mux state machine: counter 0..2N (16 for N=8).  Demux uses delayed counter.
    --   Even d-cnt: sin(hue(i))      -> V(i)
    --   Odd d-cnt:  sin(hue(i)+256)  -> U(i)
    p_hue_mux_state : process(clk)
        variable v_idx_d : integer range 0 to C_N_OSCS-1;
        variable v_uv    : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                r_hue_cnt    <= (others => '0');
                r_hue_active <= '1';
            elsif r_hue_active = '1' and r_hue_cnt < to_unsigned(2*C_N_OSCS, 5) then
                r_hue_cnt <= r_hue_cnt + 1;
            else
                r_hue_active <= '0';
            end if;
            r_hue_cnt_d    <= r_hue_cnt;
            r_hue_active_d <= r_hue_active;

            if r_hue_active_d = '1' and r_hue_cnt_d < to_unsigned(2*C_N_OSCS, 5) then
                v_idx_d := to_integer(r_hue_cnt_d(3 downto 1));
                v_uv := unsigned(resize(to_signed(512, 11) + resize(shift_right(r_hue_lut_sin, 1), 11), 10));
                if r_hue_cnt_d(0) = '0' then
                    r_v(v_idx_d) <= v_uv;
                else
                    r_u(v_idx_d) <= v_uv;
                end if;
            end if;
        end if;
    end process;

    -- Average U/V (for ADD/AVG modes), separate process to avoid mixing
    -- with the BRAM-output-register path of r_u/r_v.
    p_avg_uv : process(clk)
        variable v_sum_u : unsigned(12 downto 0);
        variable v_sum_v : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            v_sum_u := (others => '0');
            v_sum_v := (others => '0');
            for i in 0 to C_N_OSCS-1 loop
                v_sum_u := v_sum_u + resize(r_u(i), 13);
                v_sum_v := v_sum_v + resize(r_v(i), 13);
            end loop;
            r_avg_u <= v_sum_u(12 downto 3);
            r_avg_v <= v_sum_v(12 downto 3);
        end if;
    end process;

    -- ========================================================================
    -- Keyer pipeline (3 stages of max/min/sum trees, in parallel)
    -- Stage 1: 4 pairs
    -- Stage 2: 2 pairs
    -- Stage 3: 1 final
    --
    -- UV is also pipelined alongside so the winner-index lookup at the end
    -- has matching colours available.
    -- ========================================================================
    p_keyer : process(clk)
    begin
        if rising_edge(clk) then
            -- ----- Stage 1: 4 pairs from 8 inputs -----
            for i in 0 to 3 loop
                -- Max
                if r_osc_luma(2*i) >= r_osc_luma(2*i+1) then
                    r_s1_max(i)     <= r_osc_luma(2*i);
                    r_s1_max_idx(i) <= to_unsigned(2*i, 3);
                else
                    r_s1_max(i)     <= r_osc_luma(2*i+1);
                    r_s1_max_idx(i) <= to_unsigned(2*i+1, 3);
                end if;
                -- Min
                if r_osc_luma(2*i) <= r_osc_luma(2*i+1) then
                    r_s1_min(i)     <= r_osc_luma(2*i);
                    r_s1_min_idx(i) <= to_unsigned(2*i, 3);
                else
                    r_s1_min(i)     <= r_osc_luma(2*i+1);
                    r_s1_min_idx(i) <= to_unsigned(2*i+1, 3);
                end if;
                -- Sum
                r_s1_sum(i) <= resize(r_osc_luma(2*i), 11)
                             + resize(r_osc_luma(2*i+1), 11);
            end loop;

            -- ----- Stage 2: 2 pairs from 4 stage-1 outputs -----
            for i in 0 to 1 loop
                if r_s1_max(2*i) >= r_s1_max(2*i+1) then
                    r_s2_max(i)     <= r_s1_max(2*i);
                    r_s2_max_idx(i) <= r_s1_max_idx(2*i);
                else
                    r_s2_max(i)     <= r_s1_max(2*i+1);
                    r_s2_max_idx(i) <= r_s1_max_idx(2*i+1);
                end if;
                if r_s1_min(2*i) <= r_s1_min(2*i+1) then
                    r_s2_min(i)     <= r_s1_min(2*i);
                    r_s2_min_idx(i) <= r_s1_min_idx(2*i);
                else
                    r_s2_min(i)     <= r_s1_min(2*i+1);
                    r_s2_min_idx(i) <= r_s1_min_idx(2*i+1);
                end if;
                r_s2_sum(i) <= resize(r_s1_sum(2*i), 12)
                             + resize(r_s1_sum(2*i+1), 12);
            end loop;

            -- ----- Stage 3 -----
            if r_s2_max(0) >= r_s2_max(1) then
                r_s3_max     <= r_s2_max(0);
                r_s3_max_idx <= r_s2_max_idx(0);
            else
                r_s3_max     <= r_s2_max(1);
                r_s3_max_idx <= r_s2_max_idx(1);
            end if;
            if r_s2_min(0) <= r_s2_min(1) then
                r_s3_min     <= r_s2_min(0);
                r_s3_min_idx <= r_s2_min_idx(0);
            else
                r_s3_min     <= r_s2_min(1);
                r_s3_min_idx <= r_s2_min_idx(1);
            end if;
            r_s3_sum <= resize(r_s2_sum(0), 13) + resize(r_s2_sum(1), 13);
        end if;
    end process;

    -- ========================================================================
    -- Output stage: pick mode, mux in colour, optionally invert
    -- ========================================================================
    p_output : process(clk)
        variable v_y : unsigned(9 downto 0);
        variable v_u : unsigned(9 downto 0);
        variable v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            case r_mode is
                when "00" =>  -- MAX
                    v_y := r_s3_max;
                    v_u := r_u(to_integer(r_s3_max_idx));
                    v_v := r_v(to_integer(r_s3_max_idx));
                when "01" =>  -- ADD saturating
                    if r_s3_sum > to_unsigned(1023, 13) then
                        v_y := (others => '1');
                    else
                        v_y := r_s3_sum(9 downto 0);
                    end if;
                    v_u := r_avg_u;
                    v_v := r_avg_v;
                when "10" =>  -- MIN
                    v_y := r_s3_min;
                    v_u := r_u(to_integer(r_s3_min_idx));
                    v_v := r_v(to_integer(r_s3_min_idx));
                when others =>  -- AVG (sum >> 3)
                    v_y := r_s3_sum(12 downto 3);
                    v_u := r_avg_u;
                    v_v := r_avg_v;
            end case;
            if r_invert = '1' then
                r_y_out <= not v_y;
            else
                r_y_out <= v_y;
            end if;
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

end architecture oscbank;
