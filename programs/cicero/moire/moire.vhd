-- Moire: interference between two rotated line gratings.
--
-- Two plane-wave gratings (sinusoidal stripes at angle θ1 and θ2) are
-- summed.  When θ1 and θ2 are close to each other, beat fringes appear.
-- One angle is fixed by the slider; the other animates around it, producing
-- evolving moire fringes — the classic broadcast-TV subcarrier/scanline
-- interference look.  Optional concentric mode replaces grating 1 with a
-- radial (dx²+dy²) pattern so two ring fields interfere instead.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).  All wide
-- multiplies live in their own pipeline stages.
--
-- Register map:
--   registers_in(0) = Angle 1        (rotary 1 — base angle of grating 1)
--   registers_in(1) = Angle 2 Offset (rotary 2 — angle delta vs angle 1)
--   registers_in(2) = Drift Speed    (rotary 3)
--   registers_in(3) = Phase Offset   (rotary 4)
--   registers_in(4) = Mix Balance    (rotary 5 — relative weight of gratings)
--   registers_in(5) = Brightness     (rotary 6)
--   registers_in(6) = Switches:
--     b0   Mode    (0=Linear gratings / 1=Ring × Grating)
--     b1   Blend   (0=Add / 1=Multiply)
--     b2   Hue     (0=Mono / 1=Spectrum)
--     b3   Cycle   (0=Off / 1=spectrum rotates)
--     b4   Invert
--   registers_in(7) = Frequency (slider, KEY — master spatial frequency)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture moire of program_top is

    constant LATENCY : natural := 14;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal ang1_r       : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal ang_off_r    : unsigned(9 downto 0) := to_unsigned(8,  10);
    signal drift_r      : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal phase_off_r  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal mix_r        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r     : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal freq_r       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal mode_r       : std_logic := '0';
    signal blend_r      : std_logic := '0';
    signal hue_r        : std_logic := '0';
    signal cycle_r      : std_logic := '0';
    signal invert_r     : std_logic := '0';

    -- Animated phases
    signal phase1_r     : unsigned(11 downto 0) := (others => '0');
    signal phase2_r     : unsigned(11 downto 0) := (others => '0');
    signal ang2_anim_r  : unsigned(11 downto 0) := (others => '0');

    -- Precomputed cos/sin coefficients per grating (signed 9, range -245..+245)
    -- Use shifted LUT (>> 1) to fit in signed 9 → 9-bit mul args.
    signal c1_r : signed(8 downto 0) := to_signed(245, 9);
    signal s1_r : signed(8 downto 0) := to_signed(0,   9);
    signal c2_r : signed(8 downto 0) := to_signed(245, 9);
    signal s2_r : signed(8 downto 0) := to_signed(0,   9);

    -- 64-entry quarter-wave sine LUT 0..511
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

    -- 8-color spectrum palette
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(580, 10), to_unsigned(620, 10),
        to_unsigned(840, 10), to_unsigned(700, 10),
        to_unsigned(540, 10), to_unsigned(420, 10),
        to_unsigned(380, 10), to_unsigned(680, 10));
    constant C_PAL_U : t_pal := (
        to_unsigned(360, 10), to_unsigned(440, 10),
        to_unsigned(280, 10), to_unsigned(640, 10),
        to_unsigned(840, 10), to_unsigned(720, 10),
        to_unsigned(620, 10), to_unsigned(420, 10));
    constant C_PAL_V : t_pal := (
        to_unsigned(820, 10), to_unsigned(720, 10),
        to_unsigned(380, 10), to_unsigned(320, 10),
        to_unsigned(420, 10), to_unsigned(620, 10),
        to_unsigned(780, 10), to_unsigned(680, 10));

    -- Pipeline signals
    -- S0 centered coords (range -960..+960 fits in signed 11)
    signal s0_x : signed(10 downto 0);
    signal s0_y : signed(10 downto 0);

    -- S1: parallel multiplies x*c, y*s for each grating (11×9 = 20 bits).
    signal s1_xc1 : signed(19 downto 0);
    signal s1_ys1 : signed(19 downto 0);
    signal s1_xc2 : signed(19 downto 0);
    signal s1_ys2 : signed(19 downto 0);

    -- S2: sums (linear mode) and radial alt (|xc1| + |ys1| approx)
    signal s2_f1 : signed(20 downto 0);
    signal s2_f2 : signed(20 downto 0);

    -- S3: scaled angles via freq (8-bit freq for faster multiplier).
    signal s3_a1_mul : unsigned(19 downto 0);
    signal s3_a2_mul : unsigned(19 downto 0);

    -- S4: extract 10-bit angle, add phase
    signal s4_ang1 : unsigned(11 downto 0);
    signal s4_ang2 : unsigned(11 downto 0);

    -- S5: sin lookups
    signal s5_s1 : signed(10 downto 0);
    signal s5_s2 : signed(10 downto 0);

    -- S6: weighted combine via mix knob
    signal s6_combined : signed(11 downto 0);

    -- S7: convert to 0..1023 brightness
    signal s7_b : unsigned(9 downto 0);

    -- S8: palette lookup
    signal s8_y : unsigned(9 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);
    signal s8_b : unsigned(9 downto 0);

    -- S9: scale Y by brightness * bright (registered)
    signal s9_ymul : unsigned(19 downto 0);
    signal s9_u    : unsigned(9 downto 0);
    signal s9_v    : unsigned(9 downto 0);
    signal s9_b    : unsigned(9 downto 0);

    -- S10: extract + center chroma
    signal s10_y_pre  : unsigned(9 downto 0);
    signal s10_u_cent : signed(11 downto 0);
    signal s10_v_cent : signed(11 downto 0);
    signal s10_b      : unsigned(9 downto 0);

    -- S11: chroma raw mul
    signal s11_y    : unsigned(9 downto 0);
    signal s11_umul : signed(22 downto 0);
    signal s11_vmul : signed(22 downto 0);

    -- S12: final extract + chroma re-center
    signal s12_y : unsigned(9 downto 0);
    signal s12_u : unsigned(9 downto 0);
    signal s12_v : unsigned(9 downto 0);

    -- S13: invert
    signal s13_y : unsigned(9 downto 0);
    signal s13_u : unsigned(9 downto 0);
    signal s13_v : unsigned(9 downto 0);

    function sat10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

    function quad_sin(ang : unsigned(9 downto 0)) return signed is
        variable v_q : unsigned(1 downto 0);
        variable v_idx : integer range 0 to 63;
        variable v_mag : unsigned(8 downto 0);
        variable v_s   : signed(10 downto 0);
    begin
        v_q := ang(9 downto 8);
        v_idx := to_integer(ang(7 downto 2));
        case v_q is
            when "00"   => v_mag := C_SIN_QUAD(v_idx);
            when "01"   => v_mag := C_SIN_QUAD(63 - v_idx);
            when "10"   => v_mag := C_SIN_QUAD(v_idx);
            when others => v_mag := C_SIN_QUAD(63 - v_idx);
        end case;
        if v_q(1) = '0' then
            v_s := signed(resize(v_mag, 11));
        else
            v_s := -signed(resize(v_mag, 11));
        end if;
        return v_s;
    end function;

begin

    -- ========================================================================
    -- Position + parameter latching + phase advance + cos/sin precompute.
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_drift  : unsigned(9 downto 0);
        variable v_a1     : unsigned(9 downto 0);
        variable v_a2     : unsigned(9 downto 0);
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
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                ang1_r     <= unsigned(registers_in(0));
                ang_off_r  <= unsigned(registers_in(1));
                drift_r    <= unsigned(registers_in(2));
                phase_off_r <= unsigned(registers_in(3));
                mix_r      <= unsigned(registers_in(4));
                bright_r   <= unsigned(registers_in(5));
                mode_r     <= registers_in(6)(0);
                blend_r    <= registers_in(6)(1);
                hue_r      <= registers_in(6)(2);
                cycle_r    <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                freq_r     <= unsigned(registers_in(7));

                v_drift := "00000" & unsigned(registers_in(2)(9 downto 5));
                phase1_r <= phase1_r + ("00" & v_drift);
                phase2_r <= phase2_r + ("00" & v_drift)
                                      + ("0000" & v_drift(9 downto 2));
                ang2_anim_r <= ang2_anim_r + ("0000" & v_drift(9 downto 2));

                -- Precompute cos/sin for each grating angle.
                -- Angle 1 = ang1_r ; Angle 2 = ang1_r + ang_off + animated drift
                v_a1 := unsigned(registers_in(0));
                v_a2 := unsigned(registers_in(0)) + unsigned(registers_in(1))
                        + ang2_anim_r(9 downto 0);
                -- cos(a) = sin(a + 256). Shift the LUT result right by 1
                -- so each coefficient fits in signed 9.
                c1_r <= resize(shift_right(quad_sin(v_a1 + to_unsigned(256, 10)), 1), 9);
                s1_r <= resize(shift_right(quad_sin(v_a1), 1), 9);
                c2_r <= resize(shift_right(quad_sin(v_a2 + to_unsigned(256, 10)), 1), 9);
                s2_r <= resize(shift_right(quad_sin(v_a2), 1), 9);
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pidx : integer range 0 to 7;
        variable v_b    : unsigned(9 downto 0);
        variable v_norm : signed(11 downto 0);
        variable v_b_signed : signed(11 downto 0);
        variable v_dx_sq : signed(25 downto 0);
        variable v_dy_sq : signed(25 downto 0);
        variable v_r2    : signed(24 downto 0);
        variable v_dx    : signed(12 downto 0);
        variable v_dy    : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: pre-centered x, y as signed 11 (range -960..+960)
            s0_x <= resize(signed(resize(pixel_x, 12)) - to_signed(960, 12), 11);
            s0_y <= resize(signed(resize(pixel_y, 12)) - to_signed(540, 12), 11);

            -- S1: parallel multiplies for both gratings (pure mul, no add chain)
            s1_xc1 <= s0_x * c1_r;
            s1_ys1 <= s0_y * s1_r;
            s1_xc2 <= s0_x * c2_r;
            s1_ys2 <= s0_y * s2_r;

            -- S2: sums.  Both modes share f2 (linear).  f1 may be linear or
            -- radial (replaced upstream by setting mode_r).
            if mode_r = '0' then
                s2_f1 <= resize(s1_xc1, 21) + resize(s1_ys1, 21);
            else
                -- Radial pattern: |xc1| + |ys1| (Manhattan) approximation.
                v_dx_sq := resize(s1_xc1, 26);
                v_dy_sq := resize(s1_ys1, 26);
                if v_dx_sq < 0 then v_dx_sq := -v_dx_sq; end if;
                if v_dy_sq < 0 then v_dy_sq := -v_dy_sq; end if;
                s2_f1 <= resize(v_dx_sq(20 downto 0), 21) +
                         resize(v_dy_sq(20 downto 0), 21);
            end if;
            s2_f2 <= resize(s1_xc2, 21) + resize(s1_ys2, 21);

            -- S3: scale f by freq (top 8 bits only).
            -- f >> 8 = 12-bit, mul freq_top (8) = 20-bit.
            s3_a1_mul <= unsigned(std_logic_vector(s2_f1(19 downto 8))) *
                          freq_r(9 downto 2);
            s3_a2_mul <= unsigned(std_logic_vector(s2_f2(19 downto 8))) *
                          freq_r(9 downto 2);

            -- S4: extract angle + add phase
            s4_ang1 <= s3_a1_mul(11 downto 0) + phase1_r;
            s4_ang2 <= s3_a2_mul(11 downto 0) + phase2_r
                       + ("00" & phase_off_r);

            -- S5: sine
            s5_s1 <= quad_sin(s4_ang1(9 downto 0));
            s5_s2 <= quad_sin(s4_ang2(9 downto 0));

            -- S6: combine: add (0) or difference of magnitudes (1)
            if blend_r = '0' then
                s6_combined <= resize(s5_s1, 12) + resize(s5_s2, 12);
            else
                -- Difference of magnitudes — gives interference fringes
                -- without a multiply.  abs(s5_s1) - abs(s5_s2), range ±491.
                s6_combined <= resize(abs(s5_s1), 12) - resize(abs(s5_s2), 12);
            end if;

            -- S7: map to 0..1023 brightness
            -- s6_combined range: ±982 for add, ±491 for mul.  Add 1024 → 0..2046.
            v_b_signed := s6_combined + to_signed(1024, 12);
            if v_b_signed < 0 then
                v_b := to_unsigned(0, 10);
            elsif v_b_signed > 2047 then
                v_b := to_unsigned(1023, 10);
            else
                v_b := unsigned(std_logic_vector(v_b_signed(10 downto 1)));
            end if;
            s7_b <= v_b;

            -- S8: palette lookup
            if hue_r = '0' then
                s8_y <= s7_b;
                s8_u <= to_unsigned(512, 10);
                s8_v <= to_unsigned(512, 10);
            else
                if cycle_r = '1' then
                    v_pidx := to_integer(s7_b(9 downto 7) +
                                          phase1_r(11 downto 9));
                else
                    v_pidx := to_integer(s7_b(9 downto 7));
                end if;
                s8_y <= C_PAL_Y(v_pidx);
                s8_u <= C_PAL_U(v_pidx);
                s8_v <= C_PAL_V(v_pidx);
            end if;
            s8_b <= s7_b;

            -- S9: scale Y by brightness (registered raw mul)
            s9_ymul <= s8_y * s8_b;
            s9_u    <= s8_u;
            s9_v    <= s8_v;
            s9_b    <= s8_b;

            -- S10: extract + center chroma
            s10_y_pre  <= s9_ymul(19 downto 10);
            s10_u_cent <= signed(resize(s9_u, 12)) - to_signed(512, 12);
            s10_v_cent <= signed(resize(s9_v, 12)) - to_signed(512, 12);
            s10_b      <= s9_b;

            -- S11: chroma raw mul
            s11_y    <= s10_y_pre;
            s11_umul <= s10_u_cent * signed('0' & std_logic_vector(s10_b));
            s11_vmul <= s10_v_cent * signed('0' & std_logic_vector(s10_b));
        end if;
    end process p_pipe;

    -- ========================================================================
    -- Output stages: brightness apply + invert
    -- ========================================================================
    p_out : process(clk)
        variable v_u_out : signed(13 downto 0);
        variable v_v_out : signed(13 downto 0);
        variable v_y_mul : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            -- S12: apply brightness knob to Y (already pre-scaled), recenter chroma
            v_y_mul := s11_y * bright_r;
            s12_y <= v_y_mul(19 downto 10);
            v_u_out := resize(shift_right(s11_umul, 10), 14) + to_signed(512, 14);
            v_v_out := resize(shift_right(s11_vmul, 10), 14) + to_signed(512, 14);
            s12_u <= sat10(v_u_out);
            s12_v <= sat10(v_v_out);

            -- S13: invert
            if invert_r = '1' then
                s13_y <= to_unsigned(1023, 10) - s12_y;
            else
                s13_y <= s12_y;
            end if;
            s13_u <= s12_u;
            s13_v <= s12_v;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s13_y);
    data_out.u       <= std_logic_vector(s13_u);
    data_out.v       <= std_logic_vector(s13_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture moire;
