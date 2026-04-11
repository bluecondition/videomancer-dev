-- Slipframe: datamosh-flavoured video processor.
--
-- Effects (all applied in one pass through the YUV444_30b pipeline):
--   * Horizontal smear     - per-pixel variable delay on Y/U/V via a
--                            1024-sample variable_delay_u. The delay is
--                            base_smear + luma-or-noise modulation + per-line
--                            tear offset, giving the "P-frame slipping"
--                            horizontal drag aesthetic.
--   * Chroma lag           - extra delay added to U and V on top of the Y
--                            smear, optionally in opposite directions on U
--                            and V (Split mode), for classic chromatic bleed.
--   * Scanline tear        - either every alternate line or LFSR-picked lines
--                            get an additional horizontal offset applied to
--                            the smear delay.
--   * Line-to-line persist - dual-bank video_line_buffer holding the previous
--                            line's raw input Y, blended into the current
--                            smeared Y.
--   * Bit-crush            - mask the lowest N bits of Y/U/V.
--   * Glitch               - momentary "boost everything" toggle that forces
--                            smear / mod / tear / chroma lag knobs to max.
--   * Dry/wet mix          - crossfade processed video against the
--                            latency-matched input with interpolator_u.
--   * Bypass               - clean latency-matched passthrough.
--
-- Pipeline (rising-edge clocks from data_in to data_out):
--   T0  data_in combinational
--   T1  p_input_stage: register inputs, pixel_x/ab tracking, LFSR latch,
--       shift bypass line
--   T2  p_delay_calc:  compute per-pixel s_delay_y/u/v, register s_s2_y/u/v
--   T3  variable_delay_u address-gen stage
--   T4  variable_delay_u BRAM read → s_smear_y/u/v valid
--       video_line_buffer input register (for persist)
--       smear_y/u/v first align register
--   T5  video_line_buffer BRAM read
--       smear_y/u/v second align register → s_smear_*_d2 ready
--   T6  (s_persist_y_slv and s_smear_*_d2 are available together here)
--   T7  p_blend_crush:  persist blend + bit-crush registered into s_proc_y/u/v
--   T8-T11 interpolator_u (4-clock latency) per channel
--   T11 data_out
--
-- BRAM budget (iCE40 HX4K, 32 × 4 Kbit EBR):
--   3 × variable_delay_u(G_WIDTH=10, G_DEPTH=11)   ≈ 3 × 20 Kbit
--   1 × video_line_buffer(G_WIDTH=10, G_DEPTH=11)  ≈ 40 Kbit (dual bank)
--   Total ≈ 100 Kbit of 128 Kbit. Drop persist line buffer if this overflows.
--
-- Register map:
--   registers_in(0) = Smear       (0-1023, base horizontal delay samples)
--   registers_in(1) = Smear Mod   (0-1023, strength of luma/noise modulation)
--   registers_in(2) = Chroma Lag  (0-1023, adds 0-511 samples to U/V delay)
--   registers_in(3) = Tear        (0-1023, per-line horizontal offset)
--   registers_in(4) = Persist     (0-1023, blend with previous line Y)
--   registers_in(5) = Crush       (top 3 bits = LSBs to mask, 0-7)
--   registers_in(6) = Switches
--                       bit 0 = Smear Src (0=Luma, 1=Noise)
--                       bit 1 = Tear Mode (0=Lines, 1=Random)
--                       bit 2 = Chroma Dir (0=Same, 1=Split)
--                       bit 3 = Glitch     (boost all knobs to max)
--                       bit 4 = Bypass     (clean passthrough)
--   registers_in(7) = Mix         (0=dry input, 1023=full wet processed)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture slipframe of program_top is

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DATA_WIDTH         : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_SMEAR_DEPTH_BITS   : integer := 11;                  -- 2048 samples
    constant C_PERSIST_DEPTH_BITS : integer := 11;                  -- 2048 per bank
    constant C_TOTAL_LATENCY      : integer := 11;

    --------------------------------------------------------------------------
    -- Registered-input / position tracking
    --------------------------------------------------------------------------
    signal s_in_y                 : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_in_u                 : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_in_v                 : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');

    signal s_pixel_x              : unsigned(C_PERSIST_DEPTH_BITS - 1 downto 0) := (others => '0');
    signal s_prev_hsync_n         : std_logic := '1';
    signal s_prev_vsync_n         : std_logic := '1';
    signal s_ab_toggle            : std_logic := '0';
    signal s_line_lfsr_latched    : std_logic_vector(15 downto 0) := (others => '0');

    -- Per-frame smear drift accumulator: a random walk driven by LFSR,
    -- gated by the Smear Mod knob (and forced on by Glitch). Adds a wandering
    -- baseline offset to the Y smear so the image slips around over time
    -- instead of sitting still.
    signal s_smear_drift          : unsigned(C_SMEAR_DEPTH_BITS - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Stage 2: registered data + computed per-pixel delays fed to smear lines
    --------------------------------------------------------------------------
    signal s_s2_y                 : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_s2_u                 : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_s2_v                 : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');

    signal s_delay_y              : unsigned(C_SMEAR_DEPTH_BITS - 1 downto 0) := (others => '0');
    signal s_delay_u              : unsigned(C_SMEAR_DEPTH_BITS - 1 downto 0) := (others => '0');
    signal s_delay_v              : unsigned(C_SMEAR_DEPTH_BITS - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Stage 4: smear outputs, plus their 2-clock aligned versions
    --------------------------------------------------------------------------
    signal s_smear_y              : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_smear_u              : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_smear_v              : unsigned(C_DATA_WIDTH - 1 downto 0);

    signal s_smear_y_d1           : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_smear_u_d1           : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_smear_v_d1           : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_smear_y_d2           : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_smear_u_d2           : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_smear_v_d2           : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Persist line buffer output
    --------------------------------------------------------------------------
    signal s_persist_y_slv        : std_logic_vector(C_DATA_WIDTH - 1 downto 0);

    --------------------------------------------------------------------------
    -- Stage 7: processed outputs after persist blend + bit-crush
    --------------------------------------------------------------------------
    signal s_proc_y               : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_proc_u               : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_proc_v               : unsigned(C_DATA_WIDTH - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Bypass / dry-path delay shift registers (length = C_TOTAL_LATENCY).
    -- Index 0 = 1 clock delayed, index N = (N+1) clocks delayed.
    --------------------------------------------------------------------------
    type t_data_shift is array (0 to C_TOTAL_LATENCY - 1)
        of std_logic_vector(C_DATA_WIDTH - 1 downto 0);
    type t_bit_shift  is array (0 to C_TOTAL_LATENCY - 1) of std_logic;

    signal s_y_sr                 : t_data_shift := (others => (others => '0'));
    signal s_u_sr                 : t_data_shift := (others => (others => '0'));
    signal s_v_sr                 : t_data_shift := (others => (others => '0'));
    signal s_hsync_sr             : t_bit_shift  := (others => '1');
    signal s_vsync_sr             : t_bit_shift  := (others => '1');
    signal s_field_sr             : t_bit_shift  := (others => '1');
    signal s_avid_sr              : t_bit_shift  := (others => '0');

    --------------------------------------------------------------------------
    -- Interpolator I/O
    --------------------------------------------------------------------------
    signal s_mix_t                : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_enable        : std_logic;
    signal s_interp_y_a           : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_u_a           : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_v_a           : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_y_result      : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_u_result      : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_v_result      : unsigned(C_DATA_WIDTH - 1 downto 0);
    signal s_interp_y_valid       : std_logic;
    signal s_interp_u_valid       : std_logic;
    signal s_interp_v_valid       : std_logic;

    --------------------------------------------------------------------------
    -- LFSR output
    --------------------------------------------------------------------------
    signal s_lfsr_q               : std_logic_vector(15 downto 0);

    --------------------------------------------------------------------------
    -- Extracted control flags
    --------------------------------------------------------------------------
    signal s_bypass_enable        : std_logic;
    signal s_glitch_enable        : std_logic;

begin

    --------------------------------------------------------------------------
    -- Register-fed concurrent assignments
    --------------------------------------------------------------------------
    s_bypass_enable <= registers_in(6)(4);
    s_glitch_enable <= registers_in(6)(3);
    s_mix_t         <= unsigned(registers_in(7));
    s_interp_enable <= '1';

    --------------------------------------------------------------------------
    -- Stage 1: input register, position tracking, LFSR latch, bypass line
    --------------------------------------------------------------------------
    p_input_stage : process(clk)
        variable v_drift_mod   : unsigned(9 downto 0);
        variable v_drift_prod  : unsigned(19 downto 0);
        variable v_drift_step  : unsigned(C_SMEAR_DEPTH_BITS - 1 downto 0);
    begin
        if rising_edge(clk) then
            -- Register the video data for downstream stages
            s_in_y <= unsigned(data_in.y);
            s_in_u <= unsigned(data_in.u);
            s_in_v <= unsigned(data_in.v);

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                s_pixel_x <= s_pixel_x + 1;
            end if;

            -- hsync falling edge = new line: reset x, flip ab, latch LFSR
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_pixel_x           <= (others => '0');
                s_ab_toggle         <= not s_ab_toggle;
                s_line_lfsr_latched <= s_lfsr_q;
            end if;

            -- vsync falling edge = new frame: re-sync ab bank, tick drift
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_ab_toggle <= '0';

                -- Drift: random-walk step scaled by Smear Mod. When Glitch is
                -- on, force max step. When both are zero, step is zero and the
                -- accumulator holds.
                if registers_in(6)(3) = '1' then
                    v_drift_mod := (others => '1');
                else
                    v_drift_mod := unsigned(registers_in(1));
                end if;
                v_drift_prod := v_drift_mod * unsigned(s_lfsr_q(9 downto 0));
                -- >> 10 brings it back to knob-scale, then >> 1 more so each
                -- frame only nudges the baseline by up to ~knob/2 samples.
                v_drift_step := resize(v_drift_prod(19 downto 11),
                                       C_SMEAR_DEPTH_BITS);
                s_smear_drift <= s_smear_drift + v_drift_step;
            end if;

            ------------------------------------------------------------------
            -- Bypass / dry path shift registers (C_TOTAL_LATENCY deep)
            ------------------------------------------------------------------
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
    -- Stage 2: compute per-pixel delay targets
    --------------------------------------------------------------------------
    p_delay_calc : process(clk)
        variable v_smear_knob    : unsigned(9 downto 0);
        variable v_smear_scaled  : unsigned(10 downto 0);
        variable v_mod_knob      : unsigned(9 downto 0);
        variable v_lag_knob      : unsigned(9 downto 0);
        variable v_tear_knob     : unsigned(9 downto 0);

        variable v_mod_src       : unsigned(9 downto 0);
        variable v_mod_prod_full : unsigned(19 downto 0);
        variable v_mod_contrib   : unsigned(10 downto 0);

        variable v_line_rand     : unsigned(9 downto 0);
        variable v_tear_prod     : unsigned(19 downto 0);
        variable v_tear_contrib  : unsigned(10 downto 0);

        variable v_sum_y         : unsigned(13 downto 0);   -- ample headroom
        variable v_clamp_y       : unsigned(C_SMEAR_DEPTH_BITS - 1 downto 0);
        variable v_chroma_lag    : unsigned(9 downto 0);
        variable v_sum_u         : unsigned(13 downto 0);
        variable v_sum_v         : signed(14 downto 0);
    begin
        if rising_edge(clk) then

            -- Forward the input data into the smear delay lines' write port
            s_s2_y <= s_in_y;
            s_s2_u <= s_in_u;
            s_s2_v <= s_in_v;

            -- Glitch toggle: force all amount knobs to their max value
            if s_glitch_enable = '1' then
                v_smear_knob := (others => '1');
                v_mod_knob   := (others => '1');
                v_lag_knob   := (others => '1');
                v_tear_knob  := (others => '1');
            else
                v_smear_knob := unsigned(registers_in(0));
                v_mod_knob   := unsigned(registers_in(1));
                v_lag_knob   := unsigned(registers_in(2));
                v_tear_knob  := unsigned(registers_in(3));
            end if;

            -- Modulation source: current pixel luma, or LFSR noise
            if registers_in(6)(0) = '0' then
                v_mod_src := s_in_y;
            else
                v_mod_src := unsigned(s_lfsr_q(9 downto 0));
            end if;

            -- (mod_knob * src) >> 10 → contribution range 0..1023
            v_mod_prod_full := v_mod_knob * v_mod_src;
            v_mod_contrib   := resize(v_mod_prod_full(19 downto 10), 11);

            -- Per-line tear offset
            if registers_in(6)(1) = '0' then
                -- "Lines" mode: alternate lines carry the offset
                if s_ab_toggle = '1' then
                    v_tear_contrib := resize(v_tear_knob, 11);
                else
                    v_tear_contrib := (others => '0');
                end if;
            else
                -- "Random" mode: LFSR-gated per line
                v_line_rand  := unsigned(s_line_lfsr_latched(9 downto 0));
                v_tear_prod  := v_tear_knob * v_line_rand;
                v_tear_contrib := resize(v_tear_prod(19 downto 10), 11);
            end if;

            -- Scale the smear knob ×2 so the full range reaches the 2048-sample
            -- depth of the variable_delay_u buffers.
            v_smear_scaled := shift_left(resize(v_smear_knob, 11), 1);

            -- Combine base smear + luma/noise mod + per-line tear + drift,
            -- then clamp Y delay to [0, 2047].
            v_sum_y := resize(v_smear_scaled,  14)
                     + resize(v_mod_contrib,   14)
                     + resize(v_tear_contrib,  14)
                     + resize(s_smear_drift,   14);
            if v_sum_y > to_unsigned(2047, 14) then
                v_clamp_y := to_unsigned(2047, C_SMEAR_DEPTH_BITS);
            else
                v_clamp_y := v_sum_y(C_SMEAR_DEPTH_BITS - 1 downto 0);
            end if;
            s_delay_y <= v_clamp_y;

            -- Chroma lag: half the knob value so max ≈ 511 samples
            v_chroma_lag := shift_right(v_lag_knob, 1);

            -- U gets +lag (always positive direction)
            v_sum_u := resize(v_clamp_y, 14) + resize(v_chroma_lag, 14);
            if v_sum_u > to_unsigned(2047, 14) then
                s_delay_u <= to_unsigned(2047, C_SMEAR_DEPTH_BITS);
            else
                s_delay_u <= v_sum_u(C_SMEAR_DEPTH_BITS - 1 downto 0);
            end if;

            -- V gets +lag in Same mode, -lag (floored at 0) in Split mode
            if registers_in(6)(2) = '1' then
                v_sum_v := resize(signed('0' & std_logic_vector(v_clamp_y)), 15)
                         - resize(signed('0' & std_logic_vector(v_chroma_lag)), 15);
                if v_sum_v(14) = '1' then
                    s_delay_v <= (others => '0');
                elsif v_sum_v > to_signed(2047, 15) then
                    s_delay_v <= to_unsigned(2047, C_SMEAR_DEPTH_BITS);
                else
                    s_delay_v <= unsigned(v_sum_v(C_SMEAR_DEPTH_BITS - 1 downto 0));
                end if;
            else
                if v_sum_u > to_unsigned(2047, 14) then
                    s_delay_v <= to_unsigned(2047, C_SMEAR_DEPTH_BITS);
                else
                    s_delay_v <= v_sum_u(C_SMEAR_DEPTH_BITS - 1 downto 0);
                end if;
            end if;

        end if;
    end process p_delay_calc;

    --------------------------------------------------------------------------
    -- Smear delay lines: 3 × variable_delay_u, one per YUV channel
    --------------------------------------------------------------------------
    smear_y_inst : entity work.variable_delay_u
        generic map(
            G_WIDTH => C_DATA_WIDTH,
            G_DEPTH => C_SMEAR_DEPTH_BITS
        )
        port map(
            clk    => clk,
            enable => '1',
            delay  => s_delay_y,
            a      => s_s2_y,
            result => s_smear_y,
            valid  => open
        );

    smear_u_inst : entity work.variable_delay_u
        generic map(
            G_WIDTH => C_DATA_WIDTH,
            G_DEPTH => C_SMEAR_DEPTH_BITS
        )
        port map(
            clk    => clk,
            enable => '1',
            delay  => s_delay_u,
            a      => s_s2_u,
            result => s_smear_u,
            valid  => open
        );

    smear_v_inst : entity work.variable_delay_u
        generic map(
            G_WIDTH => C_DATA_WIDTH,
            G_DEPTH => C_SMEAR_DEPTH_BITS
        )
        port map(
            clk    => clk,
            enable => '1',
            delay  => s_delay_v,
            a      => s_s2_v,
            result => s_smear_v,
            valid  => open
        );

    --------------------------------------------------------------------------
    -- Align smear Y/U/V by 2 clocks to match the persist line buffer's
    -- 2-clock read latency.
    --------------------------------------------------------------------------
    p_smear_align : process(clk)
    begin
        if rising_edge(clk) then
            s_smear_y_d1 <= s_smear_y;
            s_smear_u_d1 <= s_smear_u;
            s_smear_v_d1 <= s_smear_v;
            s_smear_y_d2 <= s_smear_y_d1;
            s_smear_u_d2 <= s_smear_u_d1;
            s_smear_v_d2 <= s_smear_v_d1;
        end if;
    end process p_smear_align;

    --------------------------------------------------------------------------
    -- Persist line buffer: stores raw input Y per line. Read/write at the
    -- live input pixel_x position. The output at any cycle is the previous
    -- line's input Y at that column, with ~2 clocks of internal latency.
    --
    -- We feed raw data_in.y rather than smeared Y so that the address and
    -- data are naturally aligned at the current pixel_x.
    --------------------------------------------------------------------------
    persist_y_inst : entity work.video_line_buffer
        generic map(
            G_WIDTH => C_DATA_WIDTH,
            G_DEPTH => C_PERSIST_DEPTH_BITS
        )
        port map(
            clk       => clk,
            i_ab      => s_ab_toggle,
            i_wr_addr => s_pixel_x,
            i_rd_addr => s_pixel_x,
            i_data    => data_in.y,
            o_data    => s_persist_y_slv
        );

    --------------------------------------------------------------------------
    -- Stage 7: persist blend + bit-crush
    --------------------------------------------------------------------------
    p_blend_crush : process(clk)
        variable v_prev_y       : unsigned(C_DATA_WIDTH - 1 downto 0);
        variable v_persist_t    : unsigned(9 downto 0);
        variable v_inv_t        : unsigned(9 downto 0);
        variable v_cur_mul      : unsigned(19 downto 0);
        variable v_prev_mul     : unsigned(19 downto 0);
        variable v_blend_sum    : unsigned(20 downto 0);
        variable v_blend_y      : unsigned(C_DATA_WIDTH - 1 downto 0);
        variable v_crush_n      : integer range 0 to 7;
        variable v_crush_mask   : unsigned(C_DATA_WIDTH - 1 downto 0);
        variable v_scramble_y   : unsigned(C_DATA_WIDTH - 1 downto 0);
        variable v_out_u        : unsigned(C_DATA_WIDTH - 1 downto 0);
        variable v_out_v        : unsigned(C_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_prev_y := unsigned(s_persist_y_slv);

            -- Persist blend weight (0 = all current, 1023 ≈ all previous)
            v_persist_t := unsigned(registers_in(4));
            v_inv_t     := to_unsigned(1023, 10) - v_persist_t;

            -- Sum of two 10-bit × 10-bit products → 21 bits headroom
            v_cur_mul   := s_smear_y_d2 * v_inv_t;
            v_prev_mul  := v_prev_y     * v_persist_t;
            v_blend_sum := resize(v_cur_mul, 21) + resize(v_prev_mul, 21);

            -- Divide by 1024 (>> 10) and keep 10 bits. The sum's max is
            -- 1023*1023 ≈ 1_046_529 which after >>10 stays ≤ 1023.
            v_blend_y := v_blend_sum(19 downto 10);

            -- Glitch: scramble Y with the live LFSR. XOR corrupts bit values
            -- uniformly across the 10-bit range, giving a heavy "broken data"
            -- look while the Glitch toggle is held.
            if s_glitch_enable = '1' then
                v_scramble_y := v_blend_y xor unsigned(s_lfsr_q(9 downto 0));
            else
                v_scramble_y := v_blend_y;
            end if;

            -- Glitch: chroma swap on LFSR-selected lines. Uses the per-line
            -- latched LFSR so the decision is stable across a whole line.
            if s_glitch_enable = '1' and s_line_lfsr_latched(0) = '1' then
                v_out_u := s_smear_v_d2;
                v_out_v := s_smear_u_d2;
            else
                v_out_u := s_smear_u_d2;
                v_out_v := s_smear_v_d2;
            end if;

            -- Bit-crush mask: top 3 bits of Crush knob pick LSBs to mask
            v_crush_n := to_integer(unsigned(registers_in(5)(9 downto 7)));
            v_crush_mask := (others => '1');
            for b in 0 to C_DATA_WIDTH - 1 loop
                if b < v_crush_n then
                    v_crush_mask(b) := '0';
                end if;
            end loop;

            s_proc_y <= v_scramble_y and v_crush_mask;
            s_proc_u <= v_out_u      and v_crush_mask;
            s_proc_v <= v_out_v      and v_crush_mask;
        end if;
    end process p_blend_crush;

    --------------------------------------------------------------------------
    -- Dry-path taps for interpolator a input. s_proc_* is registered at T7
    -- (7 clocks after data_in), which corresponds to shift-register index 6
    -- (index 0 = 1 clk delayed ⇒ index 6 = 7 clks delayed).
    --------------------------------------------------------------------------
    s_interp_y_a <= unsigned(s_y_sr(6));
    s_interp_u_a <= unsigned(s_u_sr(6));
    s_interp_v_a <= unsigned(s_v_sr(6));

    --------------------------------------------------------------------------
    -- Dry/wet mix: interpolator_u, 4-clock latency per channel.
    -- a = dry (bypass-delayed input), b = wet (processed), t = Mix slider.
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
            enable => s_interp_enable,
            a      => s_interp_y_a,
            b      => s_proc_y,
            t      => s_mix_t,
            result => s_interp_y_result,
            valid  => s_interp_y_valid
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
            enable => s_interp_enable,
            a      => s_interp_u_a,
            b      => s_proc_u,
            t      => s_mix_t,
            result => s_interp_u_result,
            valid  => s_interp_u_valid
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
            enable => s_interp_enable,
            a      => s_interp_v_a,
            b      => s_proc_v,
            t      => s_mix_t,
            result => s_interp_v_result,
            valid  => s_interp_v_valid
        );

    --------------------------------------------------------------------------
    -- LFSR noise source (free-running)
    --------------------------------------------------------------------------
    lfsr_inst : entity work.lfsr16
        port map(
            clk    => clk,
            enable => '1',
            seed   => x"0000",
            load   => '0',
            q      => s_lfsr_q
        );

    --------------------------------------------------------------------------
    -- Output mux at T11. Sync signals always come from the bypass shift
    -- register. Video data comes from the interpolator unless Bypass is on.
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

end architecture slipframe;
