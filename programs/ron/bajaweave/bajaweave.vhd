-- Bajaweave: Two Smooth Curvy Sine Lines (Phase-Modulated Carriers)
--
-- Generates two vertical "lines" whose sway travels up / down the screen.
-- The bars themselves stay put horizontally.  Lines are smooth because:
--   * Carrier is a continuous sine wave (1024-entry LUT), not a square.
--   * Modulator is a sine LFO (also from the LUT), not triangle/sawtooth.
--   * Modulation is applied as a PHASE OFFSET to the carrier (PM), so the
--     resulting waveform is sin(carrier_phase + lfo_sin(line)) -- no
--     integration artefacts, no discontinuities.
--   * One per-field drift accumulator offsets the LFO angle, making the
--     sway pattern travel vertically.  The P12 slider is BIPOLAR: centre
--     = frozen, above centre the flow runs one way (full speed at the
--     top), below centre the other way (full speed at the bottom).  Osc1
--     always follows the slider; S10 selects whether osc2 flows the SAME
--     way or OPPOSITE (counter-flow).
--
-- The two oscillators are combined by a winner-take-all keyer: at each
-- pixel, the brighter sine wins and contributes its colour; the dimmer
-- one is hidden.  This gives clean colour separation at line crossings
-- with no chunky boundaries.  S7 selects Bright (legacy, half-amplitude
-- chroma) or Deep (full chroma, DC-removed ADD, winner-chroma blends).
--
-- The per-line LFO accumulators are FRAME-LOCKED (reset at vsync), so
-- K2/K5 purely set the sway's vertical wavelength.  The ONLY temporal
-- motion is the shared drift accumulator above (slider centre = frozen).
-- S11 selects the waveshape: sine or triangle (carriers and LFOs both).
--
-- Resources:
--   1x video_timing_generator
--   2x video_timing_accumulator (G_W=20, C_VERTICAL, lock=1)    : per-line LFOs
--   2x video_timing_accumulator (G_W=20, C_HORIZONTAL, lock=1)  : per-pixel carriers
--   1x frame_phase_accumulator (G_W=16)                          : sway drift
--   2x sin_cos_full_lut_10x10                                    : per-osc LFO sine
--   2x sin_cos_full_lut_10x10                                    : per-osc carrier sine
--   2x sin_cos_full_lut_10x10                                    : per-osc hue -> (U,V)
--
-- Pipeline (data_in -> data_out):
--   1 clk : video_timing_generator
--   2 clk : video_timing_accumulator (input reg + accum)
--   1 clk : register PM phase (carrier_top + LFO_sin)
--   1 clk : carrier sine LUT lookup, register raw sin & tri
--   1 clk : second sin/tri register (the sine LUT infers as EBR and eats
--           the first register; this one splits EBR read from the output mux)
--   1 clk : sine/triangle select + 512 -> register luma
--   1 clk : keyer (max / blend mode) register
--   Total: 8 clk
--   (The LFO side chain has two extra registers -- the LFO angle =
--   phase+drift register before its sine LUT and the same second sin/tri
--   register.  The LFO value is line-stable, so neither affects the
--   pixel-path delay above.)
--
-- Register map:
--   registers_in(0)  = K1: OSC1 spatial frequency (bars per screen)
--   registers_in(1)  = K2: OSC1 sway wavelength (curve density)
--   registers_in(2)  = K3: OSC1 hue
--   registers_in(3)  = K4: OSC2 spatial frequency
--   registers_in(4)  = K5: OSC2 sway wavelength
--   registers_in(5)  = K6: OSC2 hue
--   registers_in(6)  = switches:
--                        bit 0 = S7  colours: 0 = Bright, 1 = Deep
--                        bits 1,2 = S8,S9 blend mode select (2-bit):
--                                 00 MAX, 01 ADD, 10 DIFF, 11 HARDKEY
--                        bit 3 = S10 flow: 0 = osc2 opposite osc1, 1 = same
--                        bit 4 = S11 shape: 0 = sine, 1 = triangle
--                                (affects both carrier and LFO waveshape)
--   registers_in(7)  = slider 12: Speed, BIPOLAR -- centre = frozen,
--                                 ends = ~0.53 s per sway cycle each way
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

    constant C_DELAY_CLKS    : integer := 8;

    constant C_CARRIER_W     : integer := 20;
    constant C_LFO_W         : integer := 20;

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
    -- The LFO accumulator is frame-locked (reset at vsync), so this step
    -- sets a pure SPATIAL wavelength of the sway down the screen:
    --   K=0   : step =  816, cycle ~1284 lines (~1.2 sway cycles at 1080)
    --   K=512 : step = 2352, cycle ~ 446 lines
    --   K=1023: step = 3885, cycle ~ 270 lines (~4 sway cycles at 1080)
    constant C_LFO_FLOOR     : integer := 816;

    -- Sway travel rate from the BIPOLAR P12 Speed slider (K = 0..1023):
    --   v = K - 512 (signed), |v| < C_SPEED_DEADBAND -> rate 0 (a centre
    --   detent you can actually hit), otherwise rate = 4*|v| per field
    --   into a 16-bit accumulator, run backwards when v < 0 (the rate is
    --   passed as 16-bit two's complement; the phase add wraps mod 2^16).
    -- Full sway cycle = 2^16/rate fields:
    --   slider end    (|v|=512) : rate 2048, ~32 fields ~= 0.53 s @ 60
    --   +/-25% travel (|v|=256) : rate 1024, ~64 fields ~= 1.1 s
    --   +/-6%         (|v|= 64) : rate  256, ~256 fields ~= 4.3 s
    --   centre        (|v|<  8) : frozen -- holds phase, resumes pop-free.
    -- Osc1 always follows the slider direction; osc2 adds or subtracts
    -- the same phase per S10 (Flow: opposite / same).
    constant C_DRIFT_W        : integer := 16;
    constant C_SPEED_DEADBAND : integer := 8;

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
    signal r_deep      : std_logic := '0';
    signal r_mode      : unsigned(1 downto 0) := (others => '0');
    signal r_flow_same : std_logic := '0';
    signal r_speed     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_tri_en    : std_logic := '0';

    -- Bipolar speed -> signed drift rate, two registered stages off the
    -- frame-stable r_speed (magnitude/sign split, then deadband + scale).
    -- r_drift_rate is 16-bit two's complement carried as unsigned.
    signal r_vel_mag    : unsigned(9 downto 0) := (others => '0');
    signal r_vel_neg    : std_logic := '0';
    signal r_drift_rate : unsigned(15 downto 0) := (others => '0');

    -- Timing record
    signal s_timing : t_video_timing_port;

    -- LFO accumulator IO
    signal s_lfo1_step_slv  : std_logic_vector(C_LFO_W-1 downto 0);
    signal s_lfo2_step_slv  : std_logic_vector(C_LFO_W-1 downto 0);
    signal s_lfo1_phase_slv : std_logic_vector(C_LFO_W-1 downto 0);
    signal s_lfo2_phase_slv : std_logic_vector(C_LFO_W-1 downto 0);
    signal s_lfo1_phase     : unsigned(C_LFO_W-1 downto 0);
    signal s_lfo2_phase     : unsigned(C_LFO_W-1 downto 0);

    -- Sway drift accumulator (per-field, rate = Speed slider / 2)
    signal s_drift_phase : unsigned(C_DRIFT_W-1 downto 0);

    -- Registered LFO LUT angle = LFO phase [top 10b] +/- drift [top 10b]
    -- (+ for osc1, - for osc2: equal speed, opposite directions).
    -- Registered to keep the add out of the sine-LUT lookup path.
    signal r_lfo1_angle : unsigned(9 downto 0) := (others => '0');
    signal r_lfo2_angle : unsigned(9 downto 0) := (others => '0');

    -- LFO sine LUT outputs (combinational; registered LFO angle -> sin)
    signal s_lfo1_sin : signed(9 downto 0);
    signal s_lfo2_sin : signed(9 downto 0);

    -- LFO sine and triangle, registered TWICE before the morph: the sine
    -- LUTs infer as block RAM and yosys absorbs the first register into
    -- the EBR output stage, so a single register left the morph hanging
    -- straight off the 2 ns EBR read -- that was the post-route critical
    -- path.  The second stage restores the split (same on the carrier side).
    signal r_lfo1_sin_r, r_lfo1_sin_r2 : signed(9 downto 0) := (others => '0');
    signal r_lfo2_sin_r, r_lfo2_sin_r2 : signed(9 downto 0) := (others => '0');
    signal r_lfo1_tri_r, r_lfo1_tri_r2 : signed(9 downto 0) := (others => '0');
    signal r_lfo2_tri_r, r_lfo2_tri_r2 : signed(9 downto 0) := (others => '0');

    -- Shape-morphed LFO value registered (used as PM offset).  Stable
    -- across a scanline (LFO ticks per line).
    signal r_lfo1_sin : signed(9 downto 0) := (others => '0');
    signal r_lfo2_sin : signed(9 downto 0) := (others => '0');

    -- LFO and carrier triangle outputs (combinational fold of phase top 10b)
    signal s_lfo1_tri : signed(9 downto 0);
    signal s_lfo2_tri : signed(9 downto 0);
    signal s_osc1_tri : signed(9 downto 0);
    signal s_osc2_tri : signed(9 downto 0);

    -- Carrier sin and tri registered TWICE (see LFO note above: stage 1 is
    -- absorbed into the sine-LUT EBR, stage 2 splits EBR read from morph).
    signal r_osc1_sin_r, r_osc1_sin_r2 : signed(9 downto 0) := (others => '0');
    signal r_osc2_sin_r, r_osc2_sin_r2 : signed(9 downto 0) := (others => '0');
    signal r_osc1_tri_r, r_osc1_tri_r2 : signed(9 downto 0) := (others => '0');
    signal r_osc2_tri_r, r_osc2_tri_r2 : signed(9 downto 0) := (others => '0');

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

    -- PM phase = carrier[top10] + LFO sin (modular)
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

    -- Hue LUT outputs, plus a register stage: the LUT -> (mux, add) -> UV
    -- chain is per-frame logic but was the post-route critical path when
    -- combinational end-to-end.  We have all of vblank, so split it.
    signal s_sin1, s_cos1 : signed(9 downto 0);
    signal s_sin2, s_cos2 : signed(9 downto 0);
    signal r_sin1_r, r_cos1_r : signed(9 downto 0) := (others => '0');
    signal r_sin2_r, r_cos2_r : signed(9 downto 0) := (others => '0');

    -- Per-frame UV
    signal r_u1, r_v1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_u2, r_v2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_vsync_d  : std_logic := '0';
    signal r_vsync_d2 : std_logic := '0';

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
                r_deep      <= s_s7;
                r_mode      <= s_s9 & s_s8;
                r_flow_same <= s_s10;
                r_tri_en    <= s_s11;
                r_speed     <= s_k12;
            end if;

            -- Bipolar sway rate (see the header constants).  Computed
            -- every clock from the frame-stable r_speed; consumed by the
            -- drift frame_phase_accumulator at vsync.
            if r_speed >= to_unsigned(512, 10) then
                r_vel_mag <= r_speed - to_unsigned(512, 10);
                r_vel_neg <= '0';
            else
                r_vel_mag <= to_unsigned(512, 10) - r_speed;
                r_vel_neg <= '1';
            end if;

            if r_vel_mag < to_unsigned(C_SPEED_DEADBAND, 10) then
                r_drift_rate <= (others => '0');
            elsif r_vel_neg = '0' then
                r_drift_rate <= resize(r_vel_mag & "00", 16);
            else
                r_drift_rate <= (not resize(r_vel_mag & "00", 16))
                                + to_unsigned(1, 16);
            end if;

            r_vsync_d  <= s_timing.vsync_start;
            r_vsync_d2 <= r_vsync_d;

            -- Register the hue LUT outputs (free-running; they only change
            -- when the hue latches above, and we consume them one clock
            -- after that edge has settled through the LUT).
            r_cos1_r <= s_cos1;
            r_sin1_r <= s_sin1;
            r_cos2_r <= s_cos2;
            r_sin2_r <= s_sin2;

            if r_vsync_d2 = '1' then
                -- Registered LUT outputs reflect the newly-latched hues
                -- (r_deep too: it latched two clocks earlier at vsync_start).
                if r_deep = '1' then
                    -- Deep: full amplitude, UV = 512 +/- 511 -> 1..1023.
                    r_u1 <= unsigned(resize(to_signed(512, 11) + resize(r_cos1_r, 11), 10));
                    r_v1 <= unsigned(resize(to_signed(512, 11) + resize(r_sin1_r, 11), 10));
                    r_u2 <= unsigned(resize(to_signed(512, 11) + resize(r_cos2_r, 11), 10));
                    r_v2 <= unsigned(resize(to_signed(512, 11) + resize(r_sin2_r, 11), 10));
                else
                    -- Bright: UV = 512 + (sin/cos >> 1), landing in 257..767.
                    r_u1 <= unsigned(resize(to_signed(512, 11) + resize(shift_right(r_cos1_r, 1), 11), 10));
                    r_v1 <= unsigned(resize(to_signed(512, 11) + resize(shift_right(r_sin1_r, 1), 11), 10));
                    r_u2 <= unsigned(resize(to_signed(512, 11) + resize(shift_right(r_cos2_r, 1), 11), 10));
                    r_v2 <= unsigned(resize(to_signed(512, 11) + resize(shift_right(r_sin2_r, 1), 11), 10));
                end if;
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
    -- LFO Accumulators (per-line, frame-locked, C_VERTICAL)
    --
    -- lock='1' resets the phase at every vsync, so the sway pattern is a
    -- pure function of line number: K2/K5 set its spatial wavelength and
    -- nothing strobes frame-to-frame.  Temporal motion is added explicitly
    -- via the drift accumulators below.  Interlace note: both fields reset
    -- at their own vsync; the bottom field's half-line offset is <0.2% of
    -- a sway cycle -- visually nil for a slow LFO.
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
            i_lock        => '1',
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
            i_lock        => '1',
            i_accumulator => s_lfo2_step_slv,
            o_accumulator => s_lfo2_phase_slv,
            o_clock       => open,
            o_pulse       => open
        );

    s_lfo1_phase <= unsigned(s_lfo1_phase_slv);
    s_lfo2_phase <= unsigned(s_lfo2_phase_slv);

    -- ========================================================================
    -- Sway Drift Accumulator (per-field, signed rate from the bipolar
    -- Speed slider; a negative rate is 2^16-|rate|, which decrements the
    -- phase via the natural mod-2^16 wrap).  Slider centre = rate 0 = the
    -- accumulator holds its phase, so the picture freezes in place and
    -- resumes without a pop.
    -- ========================================================================
    drift_inst : entity work.frame_phase_accumulator
        generic map (
            G_PHASE_WIDTH => C_DRIFT_W,
            G_SPEED_WIDTH => 16
        )
        port map (
            clk     => clk,
            vsync_n => data_in.vsync_n,
            enable  => '1',
            speed   => r_drift_rate,
            phase   => s_drift_phase
        );

    -- Register LFO angle = phase +/- drift (keeps the add out of the sine
    -- LUT path).  Osc1 always adds the drift (follows the slider); osc2
    -- adds it too when S10 = Same, subtracts when S10 = Opposite
    -- (counter-flow).  One extra clk on the LFO side chain only; the LFO
    -- value is line-stable so the pixel path is unaffected.
    p_lfo_angle_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_lfo1_angle <= s_lfo1_phase(C_LFO_W-1 downto C_LFO_W-10)
                          + s_drift_phase(C_DRIFT_W-1 downto C_DRIFT_W-10);
            if r_flow_same = '1' then
                r_lfo2_angle <= s_lfo2_phase(C_LFO_W-1 downto C_LFO_W-10)
                              + s_drift_phase(C_DRIFT_W-1 downto C_DRIFT_W-10);
            else
                r_lfo2_angle <= s_lfo2_phase(C_LFO_W-1 downto C_LFO_W-10)
                              - s_drift_phase(C_DRIFT_W-1 downto C_DRIFT_W-10);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- LFO Sine LUTs (combinational; registered LFO angle -> sin)
    -- ========================================================================
    sin_lfo1_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_lfo1_angle),
            sin_out  => s_lfo1_sin,
            cos_out  => open
        );
    sin_lfo2_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(r_lfo2_angle),
            sin_out  => s_lfo2_sin,
            cos_out  => open
        );

    -- Triangle wave of LFO angle (combinational, free).
    s_lfo1_tri <= tri_fold(r_lfo1_angle);
    s_lfo2_tri <= tri_fold(r_lfo2_angle);

    -- Register raw LFO sin + tri to split the LUT path from the morph path.
    p_lfo_sin_reg : process(clk)
    begin
        if rising_edge(clk) then
            r_lfo1_sin_r <= s_lfo1_sin;
            r_lfo2_sin_r <= s_lfo2_sin;
            r_lfo1_tri_r <= s_lfo1_tri;
            r_lfo2_tri_r <= s_lfo2_tri;
            r_lfo1_sin_r2 <= r_lfo1_sin_r;
            r_lfo2_sin_r2 <= r_lfo2_sin_r;
            r_lfo1_tri_r2 <= r_lfo1_tri_r;
            r_lfo2_tri_r2 <= r_lfo2_tri_r;
        end if;
    end process;

    -- Waveshape select from registered sin/tri (S11: sine or triangle).
    s_lfo1_morph <= r_lfo1_tri_r2 when r_tri_en = '1' else r_lfo1_sin_r2;
    s_lfo2_morph <= r_lfo2_tri_r2 when r_tri_en = '1' else r_lfo2_sin_r2;

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
    -- PM Phase Computation
    --
    -- PM_phase[10b] = carrier_acc[top 10b] + LFO_sin (signed, modular)
    -- Adds are modulo 1024 -- the sine LUT wraps cleanly.
    -- Signed LFO value cast as unsigned wraps correctly: -511 in two's
    -- complement 10-bit = 0x201 = 513, and 1024 - 511 = 513, so the
    -- modular subtraction works out.
    -- ========================================================================
    s_osc1_pm_phase <= unsigned(s_osc1_acc_slv(C_CARRIER_W-1 downto C_CARRIER_W-10))
                     + unsigned(std_logic_vector(r_lfo1_sin));
    s_osc2_pm_phase <= unsigned(s_osc2_acc_slv(C_CARRIER_W-1 downto C_CARRIER_W-10))
                     + unsigned(std_logic_vector(r_lfo2_sin));

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
            r_osc1_sin_r2 <= r_osc1_sin_r;
            r_osc2_sin_r2 <= r_osc2_sin_r;
            r_osc1_tri_r2 <= r_osc1_tri_r;
            r_osc2_tri_r2 <= r_osc2_tri_r;
        end if;
    end process;

    -- Carrier waveshape select (S11: sine or triangle).
    s_osc1_shaped <= r_osc1_tri_r2 when r_tri_en = '1' else r_osc1_sin_r2;
    s_osc2_shaped <= r_osc2_tri_r2 when r_tri_en = '1' else r_osc2_sin_r2;

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
    -- Keyer: 4 blend modes selected by S8/S9 (2-bit mode select).
    -- S7 (r_deep) selects Bright (legacy math) or Deep colour rendering.
    -- Deep also switches the per-frame chroma to full amplitude (see the
    -- UV block above); in here it changes:
    --   ADD Y   : DC-removed add (L1+L2-1024, clamped at 0) instead of a
    --             saturating add -- intersections glow out of black instead
    --             of washing ~half the screen white (each luma has DC 512,
    --             so the plain sum averages exactly at the clamp point).
    --   ADD U,V : winner's chroma instead of the average -- averaged
    --             chroma cancels to exact grey when the hues are ~180 deg
    --             apart (the default!).
    --
    -- mode (S9 & S8):
    --   00 MAX     winner-take-all luma; clean line crossings (default)
    --   01 ADD     Bright: saturating add / Deep: DC-removed add
    --   10 DIFF    abs(L1-L2); bright where they DON'T align, dark where they do
    --   11 HARDKEY classic FKG3-style threshold key at mid-luma (osc1 keys osc2)
    -- ========================================================================
    p_keyer : process(clk)
        variable v_sum_y       : unsigned(10 downto 0);
        variable v_sum_u       : unsigned(10 downto 0);
        variable v_sum_v       : unsigned(10 downto 0);
        variable v_winner_is_1 : boolean;
        variable v_win_u       : unsigned(9 downto 0);
        variable v_win_v       : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_sum_y := ('0' & r_luma1) + ('0' & r_luma2);
            v_sum_u := ('0' & r_u1)    + ('0' & r_u2);
            v_sum_v := ('0' & r_v1)    + ('0' & r_v2);

            v_winner_is_1 := (r_luma1 >= r_luma2);
            if v_winner_is_1 then
                v_win_u := r_u1;
                v_win_v := r_v1;
            else
                v_win_u := r_u2;
                v_win_v := r_v2;
            end if;

            case r_mode is

                when "00" =>  -- MAX winner-take-all
                    if v_winner_is_1 then
                        r_y_out <= r_luma1;
                    else
                        r_y_out <= r_luma2;
                    end if;
                    r_u_out <= v_win_u;
                    r_v_out <= v_win_v;

                when "01" =>  -- ADD
                    if r_deep = '1' then
                        -- DC-removed: L1+L2-1024, clamped at 0 (max 1022).
                        if v_sum_y(10) = '1' then
                            r_y_out <= v_sum_y(9 downto 0);
                        else
                            r_y_out <= (others => '0');
                        end if;
                        r_u_out <= v_win_u;
                        r_v_out <= v_win_v;
                    else
                        -- Saturating add.
                        if v_sum_y(10) = '1' then
                            r_y_out <= (others => '1');
                        else
                            r_y_out <= v_sum_y(9 downto 0);
                        end if;
                        r_u_out <= v_sum_u(10 downto 1);  -- average
                        r_v_out <= v_sum_v(10 downto 1);
                    end if;

                when "10" =>  -- DIFF abs(L1-L2)
                    if r_luma1 >= r_luma2 then
                        r_y_out <= r_luma1 - r_luma2;
                    else
                        r_y_out <= r_luma2 - r_luma1;
                    end if;
                    r_u_out <= v_win_u;
                    r_v_out <= v_win_v;

                when "11" =>  -- HARDKEY threshold (FKG3 style)
                    -- Wherever osc1 is above mid-luma it keys over osc2.
                    if r_luma1 > to_unsigned(512, 10) then
                        r_y_out <= r_luma1;
                        r_u_out <= r_u1;
                        r_v_out <= r_v1;
                    else
                        r_y_out <= r_luma2;
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
