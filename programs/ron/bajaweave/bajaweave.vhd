-- Bajaweave: Two Smooth Scrolling Curvy Sine Lines (Phase-Modulated Carriers)
--
-- Generates two vertical "lines" that sway / undulate down the screen,
-- scrolling horizontally over time.  Lines are smooth because:
--   * Carrier is a continuous sine wave (1024-entry LUT), not a square.
--   * Modulator is a sine LFO (also from the LUT), not triangle/sawtooth.
--   * Modulation is applied as a PHASE OFFSET to the carrier (PM), so the
--     resulting waveform is sin(carrier_phase + lfo_sin(line)) -- no
--     integration artefacts, no discontinuities.
--   * A per-frame scroll accumulator adds another phase offset shared by
--     both oscillators, producing smooth horizontal motion over time.
--
-- The two oscillators are combined by a winner-take-all keyer: at each
-- pixel, the brighter sine wins and contributes its colour; the dimmer
-- one is hidden.  This gives clean colour separation at line crossings
-- with no chunky boundaries.  S7 flips which oscillator wins ties.
--
-- Resources:
--   1x video_timing_generator
--   2x video_timing_accumulator (G_W=20, C_VERTICAL, free-run)  : per-line LFOs
--   2x video_timing_accumulator (G_W=20, C_HORIZONTAL, lock=1)  : per-pixel carriers
--   1x frame_phase_accumulator (G_W=16)                          : scroll
--   2x sin_cos_full_lut_10x10                                    : per-osc LFO sine
--   2x sin_cos_full_lut_10x10                                    : per-osc carrier sine
--   2x sin_cos_full_lut_10x10                                    : per-osc hue -> (U,V)
--
-- Pipeline (data_in -> data_out):
--   1 clk : video_timing_generator
--   2 clk : video_timing_accumulator (input reg + accum)
--   1 clk : register PM phase (carrier_top + LFO_sin + scroll_top)
--   1 clk : carrier sine LUT lookup, register raw sin & tri
--   1 clk : shape morph + 512 -> register luma
--   1 clk : keyer (max / blend mode) register
--   Total: 7 clk
--
-- Register map:
--   registers_in(0)  = K1: OSC1 spatial frequency (bars per screen)
--   registers_in(1)  = K2: OSC1 LFO rate (curve density / wavelength)
--   registers_in(2)  = K3: OSC1 hue
--   registers_in(3)  = K4: OSC2 spatial frequency
--   registers_in(4)  = K5: OSC2 LFO rate
--   registers_in(5)  = K6: OSC2 hue
--   registers_in(6)  = switches:
--                        bit 0 = S7  key polarity / FG-BG flip
--                        bits 1,2,3 = S8,S9,S10 blend mode select (3-bit)
--                        bit 4 = S11 scroll on/off (fixed rate when on)
--   registers_in(7)  = slider 12: Shape (sine -> triangle morph, affects
--                                 both carrier and LFO waveshape)
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

architecture bajaweave of program_top is

    constant C_DELAY_CLKS    : integer := 7;

    constant C_CARRIER_W     : integer := 20;
    constant C_LFO_W         : integer := 20;
    constant C_SCROLL_W      : integer := 16;

    -- Carrier increment scaling.  Per-pixel inc = (base << 5) + C_FREQ_MIN.
    --   At K=0     : inc = 1456,  cycle ~720 px = 1 sine peak across a line
    --   At K=1023  : inc = 34192, cycle ~30  px = ~23 peaks per line
    -- The non-zero floor is important: when inc=0 the carrier doesn't sweep
    -- horizontally at all, and the per-line LFO becomes the only variation
    -- in the picture -- which paints HORIZONTAL stripes instead of the
    -- vertical lines the program is supposed to draw.
    constant C_FREQ_KNOB_SH  : integer := 5;
    constant C_FREQ_MIN      : integer := 1456;

    -- LFO step scaling.  Per-line step = 3*K + C_LFO_FLOOR.
    -- This compresses the old 10-50% knob range into the new 0-100% range:
    --   K=0   : step =  816, cycle ~1284 lines (~2.4 frames), ~0.4 Hz wobble
    --   K=512 : step = 2352, cycle ~ 446 lines (~0.85 frames), ~1.2 Hz
    --   K=1023: step = 3885, cycle ~ 270 lines (~0.51 frames), ~2.0 Hz
    -- Old behaviour at K=10% (step=816) is now at K=0, and old K=50%
    -- (step=4096) is roughly at K=100%, giving fine knob travel in the
    -- visually useful zone.
    constant C_LFO_FLOOR     : integer := 816;

    -- Fixed scroll rate (used when S11 is on).  At G_PHASE_WIDTH=16, a step
    -- of 128 per frame wraps the scroll phase in 512 frames ~= 8.5 s @ 60Hz.
    -- Slow enough to read clearly, fast enough to feel like motion.
    constant C_SCROLL_RATE   : unsigned(9 downto 0) := to_unsigned(128, 10);

    -- Combinational knob view
    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0);
    signal s_k4, s_k5, s_k6 : unsigned(9 downto 0);
    signal s_s7             : std_logic;
    signal s_s8, s_s9, s_s10 : std_logic;
    signal s_s11            : std_logic;
    signal s_k12            : unsigned(9 downto 0);

    -- Vsync-latched (frame-stable)
    signal r_osc1_base : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal r_osc2_base : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal r_osc1_lfo  : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal r_osc2_lfo  : unsigned(9 downto 0) := to_unsigned(320, 10);
    signal r_osc1_hue  : unsigned(9 downto 0) := (others => '0');
    signal r_osc2_hue  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_keypol    : std_logic := '0';
    signal r_mode      : unsigned(2 downto 0) := (others => '0');
    signal r_shape     : unsigned(9 downto 0) := (others => '0');
    signal r_scroll_en : std_logic := '1';

    -- Timing record
    signal s_timing : t_video_timing_port;

    -- LFO accumulator IO
    signal s_lfo1_step_slv  : std_logic_vector(C_LFO_W-1 downto 0);
    signal s_lfo2_step_slv  : std_logic_vector(C_LFO_W-1 downto 0);
    signal s_lfo1_phase_slv : std_logic_vector(C_LFO_W-1 downto 0);
    signal s_lfo2_phase_slv : std_logic_vector(C_LFO_W-1 downto 0);
    signal s_lfo1_phase     : unsigned(C_LFO_W-1 downto 0);
    signal s_lfo2_phase     : unsigned(C_LFO_W-1 downto 0);

    -- LFO sine LUT outputs (combinational; top 10b of LFO phase -> sin)
    signal s_lfo1_sin : signed(9 downto 0);
    signal s_lfo2_sin : signed(9 downto 0);

    -- LFO sine and triangle, registered separately to break the LFO LUT ->
    -- morph critical path (same trick as on the carrier side).
    signal r_lfo1_sin_r : signed(9 downto 0) := (others => '0');
    signal r_lfo2_sin_r : signed(9 downto 0) := (others => '0');
    signal r_lfo1_tri_r : signed(9 downto 0) := (others => '0');
    signal r_lfo2_tri_r : signed(9 downto 0) := (others => '0');

    -- Shape-morphed LFO value registered (used as PM offset).  Stable
    -- across a scanline (LFO ticks per line).
    signal r_lfo1_sin : signed(9 downto 0) := (others => '0');
    signal r_lfo2_sin : signed(9 downto 0) := (others => '0');

    -- LFO and carrier triangle outputs (combinational fold of phase top 10b)
    signal s_lfo1_tri : signed(9 downto 0);
    signal s_lfo2_tri : signed(9 downto 0);
    signal s_osc1_tri : signed(9 downto 0);
    signal s_osc2_tri : signed(9 downto 0);

    -- Carrier sin and tri registered (splits the LUT->morph critical path).
    signal r_osc1_sin_r : signed(9 downto 0) := (others => '0');
    signal r_osc2_sin_r : signed(9 downto 0) := (others => '0');
    signal r_osc1_tri_r : signed(9 downto 0) := (others => '0');
    signal r_osc2_tri_r : signed(9 downto 0) := (others => '0');

    -- Shape-morphed LFO / carrier outputs (combinational mix of sin & tri)
    signal s_lfo1_morph : signed(9 downto 0);
    signal s_lfo2_morph : signed(9 downto 0);
    signal s_osc1_shaped : signed(9 downto 0);
    signal s_osc2_shaped : signed(9 downto 0);

    -- Carrier accumulator IO
    signal s_osc1_inc_slv : std_logic_vector(C_CARRIER_W-1 downto 0);
    signal s_osc2_inc_slv : std_logic_vector(C_CARRIER_W-1 downto 0);
    signal s_osc1_acc_slv : std_logic_vector(C_CARRIER_W-1 downto 0);
    signal s_osc2_acc_slv : std_logic_vector(C_CARRIER_W-1 downto 0);

    -- Scroll accumulator
    signal s_scroll_phase : unsigned(C_SCROLL_W-1 downto 0);

    -- PM phase = carrier[top10] + LFO sin (modular) + scroll[top10]
    -- Combinational sum and registered version (registered breaks the
    -- 3-input add + sin LUT critical path so HD timing closes).
    signal s_osc1_pm_phase : unsigned(9 downto 0);
    signal s_osc2_pm_phase : unsigned(9 downto 0);
    signal r_osc1_pm_phase : unsigned(9 downto 0) := (others => '0');
    signal r_osc2_pm_phase : unsigned(9 downto 0) := (others => '0');

    -- Carrier sine LUT outputs (combinational; PM phase -> sin)
    signal s_osc1_sin : signed(9 downto 0);
    signal s_osc2_sin : signed(9 downto 0);

    -- Registered luma per pixel (sin + 512 mapped to 0..1023)
    signal r_luma1 : unsigned(9 downto 0) := (others => '0');
    signal r_luma2 : unsigned(9 downto 0) := (others => '0');

    -- Hue LUT outputs
    signal s_sin1, s_cos1 : signed(9 downto 0);
    signal s_sin2, s_cos2 : signed(9 downto 0);

    -- Per-frame UV
    signal r_u1, r_v1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_u2, r_v2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_vsync_d  : std_logic := '0';

    -- Keyer output register
    signal r_y_out : unsigned(9 downto 0) := (others => '0');
    signal r_u_out : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_v_out : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Sync delay (align data_in timing with our processed Y/U/V)
    signal s_avid_d    : std_logic := '0';
    signal s_hsync_n_d : std_logic := '1';
    signal s_vsync_n_d : std_logic := '1';
    signal s_field_n_d : std_logic := '1';

    -- ========================================================================
    -- Triangle wave from 10-bit phase, range matches sine LUT (-510..+510).
    -- Cheap: 1 XOR + 1 conditional negate, no multiplier.
    --   phase(9) = sign half (0 = positive, 1 = negative)
    --   phase(8) = fold (1 = ramp down within half)
    --   phase(7:0) = magnitude within fold
    -- ========================================================================
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

    -- ========================================================================
    -- Sine <-> Triangle morph: 4-step linear interpolation on top 2 bits
    -- of the Shape slider.  No multiplier; just adds and shifts.
    --   shape "00": pure sine
    --   shape "01": 50/50 sin + tri
    --   shape "10": 25% sin + 75% tri
    --   shape "11": pure triangle
    -- ========================================================================
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

begin

    -- ========================================================================
    -- Register Mapping
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
    -- Per-frame knob latch + per-frame UV computation
    -- ========================================================================
    p_vsync_latch : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                r_osc1_base <= s_k1;
                r_osc1_lfo  <= s_k2;
                r_osc1_hue  <= s_k3;
                r_osc2_base <= s_k4;
                r_osc2_lfo  <= s_k5;
                r_osc2_hue  <= s_k6;
                r_keypol    <= s_s7;
                r_mode      <= s_s10 & s_s9 & s_s8;
                r_scroll_en <= s_s11;
                r_shape     <= s_k12;
            end if;

            r_vsync_d <= s_timing.vsync_start;

            if r_vsync_d = '1' then
                -- LUT outputs reflect the newly-latched hue values.
                -- UV = 512 + (sin/cos >> 1), landing in 257..767.
                r_u1 <= unsigned(resize(to_signed(512, 11) + resize(shift_right(s_cos1, 1), 11), 10));
                r_v1 <= unsigned(resize(to_signed(512, 11) + resize(shift_right(s_sin1, 1), 11), 10));
                r_u2 <= unsigned(resize(to_signed(512, 11) + resize(shift_right(s_cos2, 1), 11), 10));
                r_v2 <= unsigned(resize(to_signed(512, 11) + resize(shift_right(s_sin2, 1), 11), 10));
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Hue LUTs (combinational; angle = frame-stable hue knob)
    -- ========================================================================
    sin_hue1_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_osc1_hue),
            sin_out  => s_sin1,
            cos_out  => s_cos1
        );
    sin_hue2_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_osc2_hue),
            sin_out  => s_sin2,
            cos_out  => s_cos2
        );

    -- ========================================================================
    -- LFO Accumulators (per-line, free-running, C_VERTICAL)
    -- ========================================================================
    -- step = 3*K + C_LFO_FLOOR (no multiplier: 3K = (K<<1) + K)
    s_lfo1_step_slv <= std_logic_vector(
        shift_left(resize(r_osc1_lfo, C_LFO_W), 1)
        + resize(r_osc1_lfo, C_LFO_W)
        + to_unsigned(C_LFO_FLOOR, C_LFO_W));
    s_lfo2_step_slv <= std_logic_vector(
        shift_left(resize(r_osc2_lfo, C_LFO_W), 1)
        + resize(r_osc2_lfo, C_LFO_W)
        + to_unsigned(C_LFO_FLOOR, C_LFO_W));

    lfo1_inst : entity work.video_timing_accumulator
        generic map (G_ACCUMULATOR_WIDTH => C_LFO_W)
        port map (
            clk           => clk,
            i_timing      => s_timing,
            i_range       => C_VERTICAL,
            i_reset       => '0',
            i_lock        => '0',
            i_accumulator => s_lfo1_step_slv,
            o_accumulator => s_lfo1_phase_slv,
            o_clock       => open,
            o_pulse       => open
        );

    lfo2_inst : entity work.video_timing_accumulator
        generic map (G_ACCUMULATOR_WIDTH => C_LFO_W)
        port map (
            clk           => clk,
            i_timing      => s_timing,
            i_range       => C_VERTICAL,
            i_reset       => '0',
            i_lock        => '0',
            i_accumulator => s_lfo2_step_slv,
            o_accumulator => s_lfo2_phase_slv,
            o_clock       => open,
            o_pulse       => open
        );

    s_lfo1_phase <= unsigned(s_lfo1_phase_slv);
    s_lfo2_phase <= unsigned(s_lfo2_phase_slv);

    -- ========================================================================
    -- LFO Sine LUTs (combinational; top 10b of LFO phase -> sin)
    -- ========================================================================
    sin_lfo1_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(s_lfo1_phase(C_LFO_W-1 downto C_LFO_W-10)),
            sin_out  => s_lfo1_sin,
            cos_out  => open
        );
    sin_lfo2_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(s_lfo2_phase(C_LFO_W-1 downto C_LFO_W-10)),
            sin_out  => s_lfo2_sin,
            cos_out  => open
        );

    -- Triangle wave of LFO phase (combinational, free).
    s_lfo1_tri <= tri_fold(s_lfo1_phase(C_LFO_W-1 downto C_LFO_W-10));
    s_lfo2_tri <= tri_fold(s_lfo2_phase(C_LFO_W-1 downto C_LFO_W-10));

    -- Register raw LFO sin + tri to split the LUT path from the morph path.
    p_lfo_sin_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_lfo1_sin_r <= s_lfo1_sin;
            r_lfo2_sin_r <= s_lfo2_sin;
            r_lfo1_tri_r <= s_lfo1_tri;
            r_lfo2_tri_r <= s_lfo2_tri;
        end if;
    end process;

    -- Morph from registered sin/tri (Shape slider top 2 bits).
    s_lfo1_morph <= shape_morph(r_lfo1_sin_r, r_lfo1_tri_r, r_shape(9 downto 8));
    s_lfo2_morph <= shape_morph(r_lfo2_sin_r, r_lfo2_tri_r, r_shape(9 downto 8));

    -- Register the morphed LFO value.  Stable across a scanline (LFO only
    -- updates per line); this is the value used as PM offset for the
    -- carrier phase add.
    p_lfo_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_lfo1_sin <= s_lfo1_morph;
            r_lfo2_sin <= s_lfo2_morph;
        end if;
    end process;

    -- ========================================================================
    -- Carrier Accumulators (per-pixel, reset at line start, C_HORIZONTAL)
    --
    -- inc = base << C_FREQ_KNOB_SH.  We do NOT FM the increment;
    -- modulation is applied as a PHASE OFFSET below, which gives smoother
    -- waveforms than true FM.
    -- ========================================================================
    s_osc1_inc_slv <= std_logic_vector(shift_left(resize(r_osc1_base, C_CARRIER_W), C_FREQ_KNOB_SH)
                                        + to_unsigned(C_FREQ_MIN, C_CARRIER_W));
    s_osc2_inc_slv <= std_logic_vector(shift_left(resize(r_osc2_base, C_CARRIER_W), C_FREQ_KNOB_SH)
                                        + to_unsigned(C_FREQ_MIN, C_CARRIER_W));

    osc1_inst : entity work.video_timing_accumulator
        generic map (G_ACCUMULATOR_WIDTH => C_CARRIER_W)
        port map (
            clk           => clk,
            i_timing      => s_timing,
            i_range       => C_HORIZONTAL,
            i_reset       => '0',
            i_lock        => '1',
            i_accumulator => s_osc1_inc_slv,
            o_accumulator => s_osc1_acc_slv,
            o_clock       => open,
            o_pulse       => open
        );

    osc2_inst : entity work.video_timing_accumulator
        generic map (G_ACCUMULATOR_WIDTH => C_CARRIER_W)
        port map (
            clk           => clk,
            i_timing      => s_timing,
            i_range       => C_HORIZONTAL,
            i_reset       => '0',
            i_lock        => '1',
            i_accumulator => s_osc2_inc_slv,
            o_accumulator => s_osc2_acc_slv,
            o_clock       => open,
            o_pulse       => open
        );

    -- ========================================================================
    -- Scroll Accumulator (per-frame, shared by both oscillators)
    -- ========================================================================
    scroll_inst : entity work.frame_phase_accumulator
        generic map (
            G_PHASE_WIDTH => C_SCROLL_W,
            G_SPEED_WIDTH => 10
        )
        port map (
            clk     => clk,
            vsync_n => data_in.vsync_n,
            enable  => r_scroll_en,
            speed   => C_SCROLL_RATE,
            phase   => s_scroll_phase
        );

    -- ========================================================================
    -- PM Phase Computation
    --
    -- PM_phase[10b] = carrier_acc[top 10b] + LFO_sin (signed, modular)
    --                                      + scroll_phase[top 10b]
    -- All adds are modulo 1024 -- the sine LUT wraps cleanly.
    -- Signed LFO value cast as unsigned wraps correctly: -511 in two's
    -- complement 10-bit = 0x201 = 513, and 1024 - 511 = 513, so the
    -- modular subtraction works out.
    -- ========================================================================
    s_osc1_pm_phase <= unsigned(s_osc1_acc_slv(C_CARRIER_W-1 downto C_CARRIER_W-10))
                     + unsigned(std_logic_vector(r_lfo1_sin))
                     + s_scroll_phase(C_SCROLL_W-1 downto C_SCROLL_W-10);
    s_osc2_pm_phase <= unsigned(s_osc2_acc_slv(C_CARRIER_W-1 downto C_CARRIER_W-10))
                     + unsigned(std_logic_vector(r_lfo2_sin))
                     + s_scroll_phase(C_SCROLL_W-1 downto C_SCROLL_W-10);

    -- Register PM phase to break the path between the 3-input add and the
    -- sin LUT lookup (otherwise HD timing fails at ~70 MHz).
    p_pm_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_osc1_pm_phase <= s_osc1_pm_phase;
            r_osc2_pm_phase <= s_osc2_pm_phase;
        end if;
    end process;

    -- ========================================================================
    -- Carrier Sine LUTs (per-pixel; registered PM phase -> sin)
    -- ========================================================================
    sin_osc1_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_osc1_pm_phase),
            sin_out  => s_osc1_sin,
            cos_out  => open
        );
    sin_osc2_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_osc2_pm_phase),
            sin_out  => s_osc2_sin,
            cos_out  => open
        );

    -- Triangle wave of registered PM phase (combinational, free).
    s_osc1_tri <= tri_fold(r_osc1_pm_phase);
    s_osc2_tri <= tri_fold(r_osc2_pm_phase);

    -- Register raw sin LUT output and triangle.  Splits the slow LUT path
    -- from the morph path so HD timing closes.
    p_carrier_sin_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_osc1_sin_r <= s_osc1_sin;
            r_osc2_sin_r <= s_osc2_sin;
            r_osc1_tri_r <= s_osc1_tri;
            r_osc2_tri_r <= s_osc2_tri;
        end if;
    end process;

    -- Shape-morph carrier: blend registered sin with registered triangle.
    s_osc1_shaped <= shape_morph(r_osc1_sin_r, r_osc1_tri_r, r_shape(9 downto 8));
    s_osc2_shaped <= shape_morph(r_osc2_sin_r, r_osc2_tri_r, r_shape(9 downto 8));

    -- ========================================================================
    -- Register Luma (per pixel)
    --   luma = shaped + 512, landing in 1..1023 (full 10-bit range)
    -- ========================================================================
    p_luma_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_luma1 <= unsigned(resize(s_osc1_shaped + to_signed(512, 11), 10));
            r_luma2 <= unsigned(resize(s_osc2_shaped + to_signed(512, 11), 10));
        end if;
    end process;

    -- ========================================================================
    -- Keyer: 8 blend modes selected by S8/S9/S10 (3-bit mode select).
    -- S7 (keypol) flips foreground/background where it matters (MAX tie-break,
    -- HARDKEY polarity).
    --
    -- mode (S10 & S9 & S8):
    --   000 MAX     winner-take-all luma; clean line crossings (default)
    --   001 ADD     saturating add; intersections light up bright/white
    --   010 MIN     darkest wins; inverted-feeling silhouettes
    --   011 DIFF    abs(L1-L2); bright where they DON'T align, dark where they do
    --   100 AVG     50/50 luma + chroma; soft hazy mix
    --   101 HARDKEY classic FKG3-style threshold key at mid-luma; S7 picks source
    --   110 AND     bitwise AND of luma; sparse, glitchy
    --   111 OR      bitwise OR of luma; bright everywhere either fires
    -- ========================================================================
    p_keyer : process(clk)
        variable v_sum_y       : unsigned(10 downto 0);
        variable v_sum_u       : unsigned(10 downto 0);
        variable v_sum_v       : unsigned(10 downto 0);
        variable v_winner_is_1 : boolean;
        variable v_key_signal  : unsigned(9 downto 0);
        variable v_key_pass    : boolean;
    begin
        if rising_edge(clk) then
            v_sum_y := ('0' & r_luma1) + ('0' & r_luma2);
            v_sum_u := ('0' & r_u1)    + ('0' & r_u2);
            v_sum_v := ('0' & r_v1)    + ('0' & r_v2);

            -- Tie-break for MAX uses keypol.
            if r_keypol = '0' then
                v_winner_is_1 := (r_luma1 >= r_luma2);
            else
                v_winner_is_1 := (r_luma1 >  r_luma2);
            end if;

            case r_mode is

                when "000" =>  -- MAX winner-take-all
                    if v_winner_is_1 then
                        r_y_out <= r_luma1;
                        r_u_out <= r_u1;
                        r_v_out <= r_v1;
                    else
                        r_y_out <= r_luma2;
                        r_u_out <= r_u2;
                        r_v_out <= r_v2;
                    end if;

                when "001" =>  -- ADD saturating
                    if v_sum_y(10) = '1' then
                        r_y_out <= (others => '1');
                    else
                        r_y_out <= v_sum_y(9 downto 0);
                    end if;
                    r_u_out <= v_sum_u(10 downto 1);  -- average
                    r_v_out <= v_sum_v(10 downto 1);

                when "010" =>  -- MIN darkest wins
                    if r_luma1 <= r_luma2 then
                        r_y_out <= r_luma1;
                        r_u_out <= r_u1;
                        r_v_out <= r_v1;
                    else
                        r_y_out <= r_luma2;
                        r_u_out <= r_u2;
                        r_v_out <= r_v2;
                    end if;

                when "011" =>  -- DIFF abs(L1-L2)
                    if r_luma1 >= r_luma2 then
                        r_y_out <= r_luma1 - r_luma2;
                        r_u_out <= r_u1;
                        r_v_out <= r_v1;
                    else
                        r_y_out <= r_luma2 - r_luma1;
                        r_u_out <= r_u2;
                        r_v_out <= r_v2;
                    end if;

                when "100" =>  -- AVG 50/50
                    r_y_out <= v_sum_y(10 downto 1);
                    r_u_out <= v_sum_u(10 downto 1);
                    r_v_out <= v_sum_v(10 downto 1);

                when "101" =>  -- HARDKEY threshold (FKG3 style)
                    -- S7 picks which oscillator is the key source.
                    -- Wherever the key is above mid-luma, show the FG osc;
                    -- elsewhere show the BG osc.
                    if r_keypol = '0' then
                        v_key_signal := r_luma1;
                    else
                        v_key_signal := r_luma2;
                    end if;
                    v_key_pass := (v_key_signal > to_unsigned(512, 10));
                    if (r_keypol = '0' and v_key_pass)
                    or (r_keypol = '1' and not v_key_pass) then
                        r_y_out <= r_luma1;
                        r_u_out <= r_u1;
                        r_v_out <= r_v1;
                    else
                        r_y_out <= r_luma2;
                        r_u_out <= r_u2;
                        r_v_out <= r_v2;
                    end if;

                when "110" =>  -- AND bitwise (dim, glitchy)
                    r_y_out <= r_luma1 and r_luma2;
                    r_u_out <= v_sum_u(10 downto 1);
                    r_v_out <= v_sum_v(10 downto 1);

                when "111" =>  -- OR bitwise (bright, glitchy)
                    r_y_out <= r_luma1 or r_luma2;
                    if v_winner_is_1 then
                        r_u_out <= r_u1;
                        r_v_out <= r_v1;
                    else
                        r_u_out <= r_u2;
                        r_v_out <= r_v2;
                    end if;

                when others =>
                    r_y_out <= r_luma1;
                    r_u_out <= r_u1;
                    r_v_out <= r_v1;
            end case;
        end if;
    end process;

    -- ========================================================================
    -- Sync Delay (align data_in timing with our processed Y/U/V)
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

    -- ========================================================================
    -- Output assignment
    -- ========================================================================
    data_out.y       <= std_logic_vector(r_y_out);
    data_out.u       <= std_logic_vector(r_u_out);
    data_out.v       <= std_logic_vector(r_v_out);
    data_out.avid    <= s_avid_d;
    data_out.hsync_n <= s_hsync_n_d;
    data_out.vsync_n <= s_vsync_n_d;
    data_out.field_n <= s_field_n_d;

end architecture bajaweave;
