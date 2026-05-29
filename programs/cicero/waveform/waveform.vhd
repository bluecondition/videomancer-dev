-- Waveform: triple-trace oscilloscope/function generator.
--
-- Three traces ride across the screen.  Each trace's vertical position is
--   y_trace(x) = midline + amp_i * f(x * freq * harmonic_i + phase_i),
-- where f is either a quarter-LUT sine or a triangle wave, harmonic_i is
-- 1×/2×/3× the master frequency, and phase_i drifts with frame_count ×
-- Phase Drift.  Each trace gets its own palette color (Hue rotates the
-- assignment).  Per pixel, the pixel-to-target distance for each trace
-- is compared against the Line Width; the trace with the highest hit-
-- intensity wins the pixel (with optional soft-glow falloff and grid
-- overlay).
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).  Every wide
-- multiply (x×freq, wave×amp) sits in its own register and the 3-way
-- winner-take-all reduction is in its own stage.
--
-- Register map:
--   registers_in(0) = Amp 1       (0..1023, trace 0 amplitude)
--   registers_in(1) = Amp 2       (trace 1)
--   registers_in(2) = Amp 3       (trace 2)
--   registers_in(3) = Phase Drift (frame-rate phase speed)
--   registers_in(4) = Line Width  (pixel-distance threshold)
--   registers_in(5) = Hue         (rotates palette index)
--   registers_in(6) = Switches:
--     b0 Shape (0=Sine / 1=Triangle)
--     b1 Grid
--     b2 Glow  (0=Sharp / 1=Soft)
--     b3 Color (0=Mono  / 1=RGB)
--     b4 Invert
--   registers_in(7) = Frequency (slider, KEY — master x-frequency)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture waveform of program_top is

    constant LATENCY : natural := 17;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Position counters
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal last_y_r     : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    type t_u10_3 is array (0 to 2) of unsigned(9 downto 0);
    signal amp3_r        : t_u10_3 := (others => (others => '0'));
    signal phase_drift_r : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal line_width_r  : unsigned(9 downto 0) := to_unsigned(96, 10);
    signal hue_r         : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal freq_r        : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal shape_r       : std_logic := '0';
    signal grid_r        : std_logic := '0';
    signal glow_r        : std_logic := '1';
    signal rgb_r         : std_logic := '1';
    signal invert_r      : std_logic := '0';

    type t_phase3 is array (0 to 2) of unsigned(11 downto 0);
    signal phase_r   : t_phase3 := (others => (others => '0'));
    signal midline_r : unsigned(11 downto 0) := to_unsigned(540, 12);

    -- ------------------------------------------------------------------
    -- 64-entry quarter-wave sine LUT (10-bit input mapped via mirror+sign).
    -- Output range 0..491 (≤511).  Combinational lookup.
    -- ------------------------------------------------------------------
    type t_quad is array (0 to 63) of unsigned(8 downto 0);
    constant C_SIN_QUAD : t_quad := (
        to_unsigned(  0, 9), to_unsigned( 13, 9), to_unsigned( 25, 9), to_unsigned( 38, 9),
        to_unsigned( 50, 9), to_unsigned( 63, 9), to_unsigned( 75, 9), to_unsigned( 87, 9),
        to_unsigned(100, 9), to_unsigned(112, 9), to_unsigned(124, 9), to_unsigned(136, 9),
        to_unsigned(148, 9), to_unsigned(160, 9), to_unsigned(171, 9), to_unsigned(183, 9),
        to_unsigned(195, 9), to_unsigned(206, 9), to_unsigned(217, 9), to_unsigned(228, 9),
        to_unsigned(239, 9), to_unsigned(250, 9), to_unsigned(260, 9), to_unsigned(271, 9),
        to_unsigned(281, 9), to_unsigned(291, 9), to_unsigned(301, 9), to_unsigned(310, 9),
        to_unsigned(319, 9), to_unsigned(328, 9), to_unsigned(337, 9), to_unsigned(346, 9),
        to_unsigned(354, 9), to_unsigned(362, 9), to_unsigned(370, 9), to_unsigned(378, 9),
        to_unsigned(385, 9), to_unsigned(392, 9), to_unsigned(399, 9), to_unsigned(406, 9),
        to_unsigned(412, 9), to_unsigned(418, 9), to_unsigned(424, 9), to_unsigned(430, 9),
        to_unsigned(435, 9), to_unsigned(440, 9), to_unsigned(445, 9), to_unsigned(450, 9),
        to_unsigned(454, 9), to_unsigned(458, 9), to_unsigned(462, 9), to_unsigned(466, 9),
        to_unsigned(470, 9), to_unsigned(473, 9), to_unsigned(476, 9), to_unsigned(479, 9),
        to_unsigned(481, 9), to_unsigned(483, 9), to_unsigned(485, 9), to_unsigned(487, 9),
        to_unsigned(488, 9), to_unsigned(489, 9), to_unsigned(490, 9), to_unsigned(491, 9));

    -- ------------------------------------------------------------------
    -- Palette
    -- ------------------------------------------------------------------
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(328, 10), to_unsigned(584, 10),
        to_unsigned(840, 10), to_unsigned(580, 10),
        to_unsigned(164, 10), to_unsigned(192, 10),
        to_unsigned(349, 10), to_unsigned(580, 10));
    constant C_PAL_U : t_pal := (
        to_unsigned(960, 10), to_unsigned(772, 10),
        to_unsigned(584, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned(608, 10),
        to_unsigned(756, 10), to_unsigned(440, 10));
    constant C_PAL_V : t_pal := (
        to_unsigned(360, 10), to_unsigned(212, 10),
        to_unsigned( 64, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned(696, 10),
        to_unsigned(852, 10), to_unsigned(216, 10));

    -- ------------------------------------------------------------------
    -- Pipeline signals (all per-trace arrays are 3-wide).
    -- ------------------------------------------------------------------
    -- S0
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    -- S1: x * freq.  12 × 10 = 22.  Single multiply, registered raw.
    signal s1_prod : unsigned(21 downto 0);
    signal s1_y    : unsigned(11 downto 0);

    -- S2: per-trace angle = (prod-shifted by harmonic) + phase.
    -- Harmonic shifts: trace 0 = >>10, trace 1 = >>9, trace 2 = >>8.
    -- Each trace's angle is 12-bit.  Phase added (mod 2^12).
    type t_a12 is array (0 to 2) of unsigned(11 downto 0);
    signal s2_ang : t_a12;
    signal s2_y   : unsigned(11 downto 0);

    -- S3: extract 10-bit angle (low 10 bits) per trace.
    type t_a10 is array (0 to 2) of unsigned(9 downto 0);
    signal s3_ang : t_a10;
    signal s3_y   : unsigned(11 downto 0);

    -- S4: wave evaluation.  Result = signed(10), range -511..+511.
    type t_w10s is array (0 to 2) of signed(10 downto 0);
    signal s4_wav : t_w10s;
    signal s4_y   : unsigned(11 downto 0);

    -- S5: wave × amp (signed 11 × signed 11 = signed 22).  Registered raw.
    type t_w21s is array (0 to 2) of signed(21 downto 0);
    signal s5_amp_raw : t_w21s;
    signal s5_y       : unsigned(11 downto 0);

    -- S6: extract scaled wave in pixel units.
    type t_w12s is array (0 to 2) of signed(11 downto 0);
    signal s6_wav : t_w12s;
    signal s6_y   : unsigned(11 downto 0);

    -- S7: target_y = midline + wav.
    type t_ty13 is array (0 to 2) of signed(12 downto 0);
    signal s7_ty : t_ty13;
    signal s7_y  : unsigned(11 downto 0);

    -- S8: dy = pixel_y - target_y.
    type t_dy13s is array (0 to 2) of signed(12 downto 0);
    signal s8_dy : t_dy13s;

    -- S9: abs(dy).
    type t_dy12u is array (0 to 2) of unsigned(11 downto 0);
    signal s9_adv : t_dy12u;

    -- S10: intensity per trace.
    type t_int10 is array (0 to 2) of unsigned(9 downto 0);
    signal s10_int : t_int10;

    -- S11: winner take all (max intensity + index).
    signal s11_int     : unsigned(9 downto 0);
    signal s11_idx     : unsigned(1 downto 0);
    signal s11_int_sum : unsigned(10 downto 0);

    -- S12: palette lookup.
    signal s12_y : unsigned(9 downto 0);
    signal s12_u : unsigned(9 downto 0);
    signal s12_v : unsigned(9 downto 0);
    signal s12_int : unsigned(9 downto 0);

    -- S13: scale Y by intensity (registered raw).
    signal s13_y_mul : unsigned(19 downto 0);
    signal s13_u     : unsigned(9 downto 0);
    signal s13_v     : unsigned(9 downto 0);

    -- S14: extract Y, scale chroma toward neutral by intensity.
    signal s14_y      : unsigned(9 downto 0);
    signal s14_u_mul  : signed(20 downto 0);
    signal s14_v_mul  : signed(20 downto 0);

    -- S15: re-center chroma + clamp.
    signal s15_y : unsigned(9 downto 0);
    signal s15_u : unsigned(9 downto 0);
    signal s15_v : unsigned(9 downto 0);

    -- S16: invert + final.
    signal s16_y : unsigned(9 downto 0);
    signal s16_u : unsigned(9 downto 0);
    signal s16_v : unsigned(9 downto 0);

    -- ------------------------------------------------------------------
    -- Helpers
    -- ------------------------------------------------------------------
    function wave_eval(ang : unsigned(9 downto 0); shape : std_logic)
        return signed is
        variable v_q          : unsigned(1 downto 0);
        variable v_idx        : integer range 0 to 63;
        variable v_idx_mirror : integer range 0 to 63;
        variable v_mag        : unsigned(8 downto 0);
        variable v_byte       : unsigned(7 downto 0);
        variable v_2x         : signed(10 downto 0);
        variable v_signed     : signed(10 downto 0);
        variable v_tri        : signed(10 downto 0);
    begin
        v_q          := ang(9 downto 8);
        v_idx        := to_integer(ang(7 downto 2));
        v_idx_mirror := 63 - v_idx;
        v_byte       := ang(7 downto 0);
        v_2x         := signed(resize(v_byte, 11)) sll 1;

        if shape = '0' then
            case v_q is
                when "00"   => v_mag := C_SIN_QUAD(v_idx);
                when "01"   => v_mag := C_SIN_QUAD(v_idx_mirror);
                when "10"   => v_mag := C_SIN_QUAD(v_idx);
                when others => v_mag := C_SIN_QUAD(v_idx_mirror);
            end case;
            if v_q(1) = '0' then
                v_signed := signed(resize(v_mag, 11));
            else
                v_signed := -signed(resize(v_mag, 11));
            end if;
            return v_signed;
        else
            case v_q is
                when "00"   => v_tri := v_2x;
                when "01"   => v_tri := to_signed(511, 11) - v_2x;
                when "10"   => v_tri := -v_2x;
                when others => v_tri := -to_signed(511, 11) + v_2x;
            end case;
            return v_tri;
        end if;
    end function;

    function sat10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- ========================================================================
    -- Position tracking + parameter latching + phase advance.
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                v_h_edge := '1';
            end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                v_v_edge := '1';
            end if;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                if pixel_y > last_y_r then last_y_r <= pixel_y; end if;
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                amp3_r(0)     <= unsigned(registers_in(0));
                amp3_r(1)     <= unsigned(registers_in(1));
                amp3_r(2)     <= unsigned(registers_in(2));
                phase_drift_r <= unsigned(registers_in(3));
                line_width_r  <= unsigned(registers_in(4));
                hue_r         <= unsigned(registers_in(5));
                shape_r       <= registers_in(6)(0);
                grid_r        <= registers_in(6)(1);
                glow_r        <= registers_in(6)(2);
                rgb_r         <= registers_in(6)(3);
                invert_r      <= registers_in(6)(4);
                freq_r        <= unsigned(registers_in(7));

                -- Phase drift: scale down 32× for very calm motion.  Trace 0
                -- advances at base rate, trace 1 at 1.5×, trace 2 at 2×.
                -- Knob 0..1023 → base step 0..32 per frame.
                phase_r(0) <= phase_r(0) +
                              ("0000000" & unsigned(registers_in(3)(9 downto 5)));
                phase_r(1) <= phase_r(1) +
                              ("000000" & unsigned(registers_in(3)(9 downto 5)) & "0") +
                              ("0000000" & unsigned(registers_in(3)(9 downto 5)));
                phase_r(2) <= phase_r(2) +
                              ("00000" & unsigned(registers_in(3)(9 downto 5)) & "00");

                midline_r <= '0' & last_y_r(11 downto 1);
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline (S0..S16)
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pidx : integer range 0 to 7;
        variable v_u_c  : signed(11 downto 0);
        variable v_v_c  : signed(11 downto 0);
        variable v_dy   : signed(12 downto 0);
        variable v_u_re : signed(13 downto 0);
        variable v_v_re : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: x * freq.
            s1_prod <= s0_x * freq_r;
            s1_y    <= s0_y;

            -- S2: per-trace angle = (prod >> {10,9,8}) + phase.
            -- s1_prod is 22 bits; bits >> shift give 12-bit angle.
            s2_ang(0) <= s1_prod(21 downto 10) + phase_r(0);
            s2_ang(1) <= s1_prod(20 downto  9) + phase_r(1);
            s2_ang(2) <= s1_prod(19 downto  8) + phase_r(2);
            s2_y      <= s1_y;

            -- S3: keep low 10 bits as angle.
            for i in 0 to 2 loop
                s3_ang(i) <= s2_ang(i)(9 downto 0);
            end loop;
            s3_y <= s2_y;

            -- S4: wave evaluation.
            for i in 0 to 2 loop
                s4_wav(i) <= wave_eval(s3_ang(i), shape_r);
            end loop;
            s4_y <= s3_y;

            -- S5: wave × amp.
            for i in 0 to 2 loop
                s5_amp_raw(i) <= s4_wav(i) *
                                  signed('0' & std_logic_vector(amp3_r(i)));
            end loop;
            s5_y <= s4_y;

            -- S6: extract scaled wave (in pixels).
            for i in 0 to 2 loop
                s6_wav(i) <= resize(shift_right(s5_amp_raw(i), 10), 12);
            end loop;
            s6_y <= s5_y;

            -- S7: target_y.
            for i in 0 to 2 loop
                s7_ty(i) <= signed('0' & std_logic_vector(midline_r)) +
                            resize(s6_wav(i), 13);
            end loop;
            s7_y <= s6_y;

            -- S8: dy.
            for i in 0 to 2 loop
                s8_dy(i) <= signed('0' & std_logic_vector(s7_y)) - s7_ty(i);
            end loop;

            -- S9: abs(dy).
            for i in 0 to 2 loop
                v_dy := s8_dy(i);
                if v_dy < 0 then v_dy := -v_dy; end if;
                s9_adv(i) <= unsigned(std_logic_vector(v_dy(11 downto 0)));
            end loop;

            -- S10: intensity = (LW - dy) if dy<LW else 0.
            for i in 0 to 2 loop
                if glow_r = '0' then
                    if s9_adv(i) < ("00" & line_width_r) then
                        s10_int(i) <= resize(("00" & line_width_r) -
                                              s9_adv(i), 10);
                    else
                        s10_int(i) <= (others => '0');
                    end if;
                else
                    -- Soft: doubled width, halved intensity.
                    if s9_adv(i) < ("0" & line_width_r & "0") then
                        s10_int(i) <= resize(shift_right(
                                       ("0" & line_width_r & "0") -
                                       s9_adv(i), 1), 10);
                    else
                        s10_int(i) <= (others => '0');
                    end if;
                end if;
            end loop;

            -- S11: max-intensity winner.
            if s10_int(0) >= s10_int(1) and s10_int(0) >= s10_int(2) then
                s11_int <= s10_int(0); s11_idx <= "00";
            elsif s10_int(1) >= s10_int(2) then
                s11_int <= s10_int(1); s11_idx <= "01";
            else
                s11_int <= s10_int(2); s11_idx <= "10";
            end if;
            -- Total intensity (for additive mono accumulation).
            s11_int_sum <= ("0" & s10_int(0)) + ("0" & s10_int(1));

            -- S12: palette lookup based on winner + hue.
            v_pidx := to_integer(hue_r(9 downto 7) + ("0" & s11_idx));
            if rgb_r = '1' then
                s12_y <= C_PAL_Y(v_pidx);
                s12_u <= C_PAL_U(v_pidx);
                s12_v <= C_PAL_V(v_pidx);
            else
                -- Mono: white with intensity.
                s12_y <= to_unsigned(900, 10);
                s12_u <= to_unsigned(512, 10);
                s12_v <= to_unsigned(512, 10);
            end if;
            s12_int <= s11_int;

            -- S13: scale Y by intensity (registered raw).  10 × 10 = 20.
            s13_y_mul <= s12_y * s12_int;
            s13_u     <= s12_u;
            s13_v     <= s12_v;

            -- S14: extract Y, and scale chroma toward neutral by intensity.
            s14_y <= s13_y_mul(19 downto 10);
            v_u_c := signed('0' & std_logic_vector(s13_u)) -
                     to_signed(512, 12);
            v_v_c := signed('0' & std_logic_vector(s13_v)) -
                     to_signed(512, 12);
            -- Multiply by intensity (treat as 0..1023 sat) to fade chroma
            -- when not on a trace.  v_u_c = ±512, s12_int = 0..1023 → ±524288.
            -- v_u_c (signed 12) × signed('0' & s12_int) (signed 11) = signed 23.
            s14_u_mul <= resize(
                v_u_c * signed('0' & std_logic_vector(s12_int)), 21);
            s14_v_mul <= resize(
                v_v_c * signed('0' & std_logic_vector(s12_int)), 21);

            -- S15: re-center chroma + clamp.
            v_u_re := resize(shift_right(s14_u_mul, 10), 14) +
                      to_signed(512, 14);
            v_v_re := resize(shift_right(s14_v_mul, 10), 14) +
                      to_signed(512, 14);
            s15_y <= s14_y;
            s15_u <= sat10(v_u_re);
            s15_v <= sat10(v_v_re);

            -- S16: invert + final
            if invert_r = '1' then
                s16_y <= to_unsigned(1023, 10) - s15_y;
            else
                s16_y <= s15_y;
            end if;
            s16_u <= s15_u;
            s16_v <= s15_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s16_y);
    data_out.u       <= std_logic_vector(s16_u);
    data_out.v       <= std_logic_vector(s16_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture waveform;
