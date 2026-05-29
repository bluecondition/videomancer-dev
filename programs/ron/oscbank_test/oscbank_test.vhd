-- Oscbank Test: N parallel sine oscillators, summed/maxed for resource test.
--
-- Change C_N_OSCS, rebuild, observe LCs/BRAMs/Fmax.
-- This is a STRIPPED-DOWN test program, not a polished effect:
--   * Pure sine carriers, NO LFO/PM (simplest case)
--   * Per-osc carrier accumulator, sin LUT, hue computed from index
--   * Output = max of all oscillator lumas (winner-take-all)
--   * No shape morph, no scroll, no blend modes
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

architecture oscbank_test of program_top is

    -- Knob to tune.  Build, observe, change, rebuild.
    constant C_N_OSCS : integer := 10;

    constant C_DELAY_CLKS    : integer := 6;
    constant C_CARRIER_W     : integer := 20;
    constant C_FREQ_KNOB_SH  : integer := 5;
    constant C_FREQ_MIN      : integer := 1456;

    -- Knob view
    signal s_k1, s_k2 : unsigned(9 downto 0);

    -- Vsync-latched
    signal r_base   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal r_spread : unsigned(9 downto 0) := to_unsigned(128, 10);

    -- Timing
    signal s_timing : t_video_timing_port;

    -- Per-osc state
    type   t_inc_array   is array (0 to C_N_OSCS-1) of std_logic_vector(C_CARRIER_W-1 downto 0);
    type   t_acc_array   is array (0 to C_N_OSCS-1) of std_logic_vector(C_CARRIER_W-1 downto 0);
    type   t_phase_array is array (0 to C_N_OSCS-1) of std_logic_vector(9 downto 0);
    type   t_sin_array   is array (0 to C_N_OSCS-1) of signed(9 downto 0);
    type   t_luma_array  is array (0 to C_N_OSCS-1) of unsigned(9 downto 0);

    signal s_osc_inc   : t_inc_array;
    signal s_osc_acc   : t_acc_array;
    signal s_osc_phase : t_phase_array;
    signal s_osc_sin   : t_sin_array;
    signal r_osc_sin   : t_sin_array  := (others => (others => '0'));
    signal r_osc_luma  : t_luma_array := (others => (others => '0'));

    -- Output stage
    signal r_y_out : unsigned(9 downto 0) := (others => '0');
    signal r_u_out : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_v_out : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Sync delay
    signal s_avid_d    : std_logic := '0';
    signal s_hsync_n_d : std_logic := '1';
    signal s_vsync_n_d : std_logic := '1';
    signal s_field_n_d : std_logic := '1';

begin

    s_k1 <= unsigned(registers_in(0));
    s_k2 <= unsigned(registers_in(1));

    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    p_vsync_latch : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                r_base   <= s_k1;
                r_spread <= s_k2;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Generate N oscillators
    -- ========================================================================
    gen_oscs : for i in 0 to C_N_OSCS-1 generate

        -- Frequency = base*32 + i*spread + min
        -- i is a generate-loop constant, so the multiply collapses to a
        -- handful of shift-adds.  Product is 10b*6b=16b, resize to 20b.
        s_osc_inc(i) <= std_logic_vector(
            shift_left(resize(r_base, C_CARRIER_W), C_FREQ_KNOB_SH)
            + resize(r_spread * to_unsigned(i, 6), C_CARRIER_W)
            + to_unsigned(C_FREQ_MIN, C_CARRIER_W));

        osc_inst : entity work.video_timing_accumulator
            generic map (G_ACCUMULATOR_WIDTH => C_CARRIER_W)
            port map (
                clk           => clk,
                i_timing      => s_timing,
                i_range       => C_HORIZONTAL,
                i_reset       => '0',
                i_lock        => '1',
                i_accumulator => s_osc_inc(i),
                o_accumulator => s_osc_acc(i),
                o_clock       => open,
                o_pulse       => open
            );

        s_osc_phase(i) <= s_osc_acc(i)(C_CARRIER_W-1 downto C_CARRIER_W-10);

        sin_inst : entity work.sin_cos_full_lut_10x10
            port map (
                angle_in => s_osc_phase(i),
                sin_out  => s_osc_sin(i),
                cos_out  => open
            );

        -- Register raw sin LUT output (helps synth use BRAM output register).
        p_sin_reg : process(clk)
        begin
            if rising_edge(clk) then
                r_osc_sin(i) <= s_osc_sin(i);
            end if;
        end process;

        -- Next stage: shift sin to unsigned luma range.
        p_luma_reg : process(clk)
        begin
            if rising_edge(clk) then
                r_osc_luma(i) <= unsigned(resize(r_osc_sin(i) + to_signed(512, 11), 10));
            end if;
        end process;

    end generate;

    -- ========================================================================
    -- Output: max of all oscillator lumas (winner-take-all)
    -- Color = neutral grayscale (focus on resource scaling, not visuals)
    -- ========================================================================
    p_keyer : process(clk)
        variable v_max : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_max := (others => '0');
            for i in 0 to C_N_OSCS-1 loop
                if r_osc_luma(i) > v_max then
                    v_max := r_osc_luma(i);
                end if;
            end loop;
            r_y_out <= v_max;
            r_u_out <= to_unsigned(512, 10);
            r_v_out <= to_unsigned(512, 10);
        end if;
    end process;

    -- ========================================================================
    -- Sync Delay
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

end architecture oscbank_test;
