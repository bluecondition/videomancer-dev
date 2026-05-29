-- Spiral: rotating spiral arms from screen centre.
--
-- The (dx, dy) coords are rotated by a frame-animated angle.  Manhattan
-- radius is added to the rotated y coordinate with a tightness gain to
-- form a "spiral phase"; sin of that phase produces N-arm spinning
-- spirals.  All multipliers are short (4 rotation + 1 radius scale) and
-- pipelined for HD timing closure.
--
-- Register map:
--   registers_in(0) = Arm Count     (rotary 1 — 1..16 visible arms)
--   registers_in(1) = Spin Speed    (rotary 2)
--   registers_in(2) = Tightness     (rotary 3 — radial wrap gain)
--   registers_in(3) = Width         (rotary 4 — arm thickness)
--   registers_in(4) = Hue Spread    (rotary 5)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Direction (0=CW / 1=CCW)
--     b1   Palette   (0=Mono / 1=Spectrum)
--     b2   Pulse     (0=Off / 1=Frame pulsing brightness)
--     b3   Negative  (0=Lines / 1=Gaps)
--     b4   Invert
--   registers_in(7) = Phase Drift (slider, KEY — secondary radial offset)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture spiral of program_top is

    constant LATENCY : natural := 14;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal arms_r      : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal speed_r     : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal tight_r     : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal width_r     : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal hue_r       : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bright_r    : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal phase_off_r : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal dir_r       : std_logic := '0';
    signal palette_r   : std_logic := '0';
    signal pulse_r     : std_logic := '0';
    signal neg_r       : std_logic := '0';
    signal invert_r    : std_logic := '0';

    signal spin_phase_r : unsigned(11 downto 0) := (others => '0');

    -- Rotation coefficients (signed 9, range -245..+245)
    signal cos_r : signed(8 downto 0) := to_signed(245, 9);
    signal sin_r : signed(8 downto 0) := to_signed(0,   9);

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

    -- 8-color palette
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(700, 10), to_unsigned(620, 10),
        to_unsigned(840, 10), to_unsigned(580, 10),
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
    signal s0_dx : signed(10 downto 0);  -- centered, fits in 11 bits
    signal s0_dy : signed(10 downto 0);

    -- S1: rotation multiplies (11×9 = 20 bits signed)
    signal s1_dx_c : signed(19 downto 0);
    signal s1_dx_s : signed(19 downto 0);
    signal s1_dy_c : signed(19 downto 0);
    signal s1_dy_s : signed(19 downto 0);

    -- S2: rotated coords + manhattan radius
    signal s2_xr : signed(20 downto 0);
    signal s2_yr : signed(20 downto 0);
    signal s2_r  : unsigned(11 downto 0);

    -- S3: extract rotated y (>> 7 to pixel scale), pass radius
    signal s3_yr : signed(13 downto 0);
    signal s3_r  : unsigned(11 downto 0);

    -- S4: yr * arms (signed 14 × signed 11 = 24 bits)
    signal s4_yr_mul : signed(23 downto 0);
    signal s4_r      : unsigned(11 downto 0);

    -- S5: r * tightness (12 × 10 = 22 bits) registered raw
    signal s5_yr_mul : signed(23 downto 0);
    signal s5_r_mul  : unsigned(21 downto 0);

    -- S6: phase = yr_arm + r_tight (signed 14)
    signal s6_phase : signed(13 downto 0);

    -- S7: sin lookup angle (10-bit)
    signal s7_ang : unsigned(9 downto 0);

    -- S8: sin value (signed 11)
    signal s8_sin : signed(10 downto 0);

    -- S9: arm brightness from sin magnitude vs width threshold
    signal s9_arm : unsigned(9 downto 0);
    signal s9_idx : unsigned(2 downto 0);

    -- S10: palette
    signal s10_y : unsigned(9 downto 0);
    signal s10_u : unsigned(9 downto 0);
    signal s10_v : unsigned(9 downto 0);
    signal s10_arm : unsigned(9 downto 0);

    -- S11: Y * arm (raw)
    signal s11_ymul : unsigned(19 downto 0);
    signal s11_u    : unsigned(9 downto 0);
    signal s11_v    : unsigned(9 downto 0);

    -- S12: Y * bright (raw)
    signal s12_y_in   : unsigned(9 downto 0);
    signal s12_ymul2  : unsigned(19 downto 0);
    signal s12_u      : unsigned(9 downto 0);
    signal s12_v      : unsigned(9 downto 0);

    -- S13: final invert
    signal s13_y : unsigned(9 downto 0);
    signal s13_u : unsigned(9 downto 0);
    signal s13_v : unsigned(9 downto 0);

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
    -- Position + parameters + spin animation
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_speed  : unsigned(9 downto 0);
        variable v_a1     : unsigned(9 downto 0);
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

                arms_r      <= unsigned(registers_in(0));
                speed_r     <= unsigned(registers_in(1));
                tight_r     <= unsigned(registers_in(2));
                width_r     <= unsigned(registers_in(3));
                hue_r       <= unsigned(registers_in(4));
                bright_r    <= unsigned(registers_in(5));
                dir_r       <= registers_in(6)(0);
                palette_r   <= registers_in(6)(1);
                pulse_r     <= registers_in(6)(2);
                neg_r       <= registers_in(6)(3);
                invert_r    <= registers_in(6)(4);
                phase_off_r <= unsigned(registers_in(7));

                v_speed := "00000" & unsigned(registers_in(1)(9 downto 5));
                if registers_in(6)(0) = '0' then
                    spin_phase_r <= spin_phase_r + ("00" & v_speed);
                else
                    spin_phase_r <= spin_phase_r - ("00" & v_speed);
                end if;

                v_a1 := spin_phase_r(11 downto 2);
                cos_r <= resize(shift_right(quad_sin(v_a1 + to_unsigned(256, 10)), 1), 9);
                sin_r <= resize(shift_right(quad_sin(v_a1), 1), 9);
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pidx : integer range 0 to 7;
        variable v_sin_mag : unsigned(9 downto 0);
        variable v_sin_signed : signed(10 downto 0);
        variable v_arms : unsigned(4 downto 0);
        variable v_dx_a : signed(10 downto 0);
        variable v_dy_a : signed(10 downto 0);
        variable v_y_mul : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: centered coords (range -960..+960 fits signed 11)
            s0_dx <= resize(signed(resize(pixel_x, 12)) - to_signed(960, 12), 11);
            s0_dy <= resize(signed(resize(pixel_y, 12)) - to_signed(540, 12), 11);

            -- S1: rotation multiplies (4 parallel)
            s1_dx_c <= s0_dx * cos_r;
            s1_dx_s <= s0_dx * sin_r;
            s1_dy_c <= s0_dy * cos_r;
            s1_dy_s <= s0_dy * sin_r;

            -- S2: rotated coords + manhattan radius
            s2_xr <= resize(s1_dx_c, 21) - resize(s1_dy_s, 21);
            s2_yr <= resize(s1_dx_s, 21) + resize(s1_dy_c, 21);
            v_dx_a := s0_dx;
            if v_dx_a < 0 then v_dx_a := -v_dx_a; end if;
            v_dy_a := s0_dy;
            if v_dy_a < 0 then v_dy_a := -v_dy_a; end if;
            s2_r <= resize(unsigned(std_logic_vector(v_dx_a(9 downto 0))) +
                           unsigned(std_logic_vector(v_dy_a(9 downto 0))), 12);

            -- S3: shift yr to pixel scale (>> 7 to undo /2 cos/sin and another /64)
            s3_yr <= resize(shift_right(s2_yr, 7), 14);
            s3_r  <= s2_r;

            -- S4: yr * arms_r (11-bit unsigned arms × 14-bit signed yr)
            -- Arms 0..1023 → effective 0..31 via shift right 5
            v_arms := resize(arms_r(9 downto 5), 5);
            s4_yr_mul <= s3_yr * signed(resize(v_arms, 10));
            s4_r <= s3_r;

            -- S5: r * tightness (12×10 = 22) raw
            s5_r_mul <= s4_r * tight_r;
            s5_yr_mul <= s4_yr_mul;

            -- S6: phase = yr_arm + r_tight (drop low bits)
            s6_phase <= resize(shift_right(s5_yr_mul, 4), 14) +
                        resize(signed('0' & std_logic_vector(s5_r_mul(15 downto 4))), 14);

            -- S7: extract 10-bit angle + add phase_off
            s7_ang <= unsigned(std_logic_vector(s6_phase(9 downto 0))) +
                      phase_off_r;

            -- S8: sin lookup
            s8_sin <= quad_sin(s7_ang);

            -- S9: arm brightness from |sin| compared to width threshold
            if s8_sin < 0 then
                v_sin_signed := -s8_sin;
                v_sin_mag := unsigned(std_logic_vector(v_sin_signed(9 downto 0)));
            else
                v_sin_mag := unsigned(std_logic_vector(s8_sin(9 downto 0)));
            end if;
            -- If width covers most of sin range, broad arms; small width = thin arms.
            -- arm_bright = max(0, width - sin_mag) when sin_mag low (peak of sine).
            -- We want arms where sin ~ 0, so use 1023 - |sin| - threshold
            if v_sin_mag < width_r then
                s9_arm <= width_r - v_sin_mag;
            else
                s9_arm <= (others => '0');
            end if;
            if neg_r = '1' then
                s9_arm <= to_unsigned(1023, 10) - s9_arm;
            end if;
            s9_idx <= s7_ang(9 downto 7);

            -- S10: palette
            if palette_r = '0' then
                s10_y <= to_unsigned(900, 10);
                s10_u <= to_unsigned(512, 10);
                s10_v <= to_unsigned(512, 10);
            else
                v_pidx := to_integer(s9_idx + hue_r(9 downto 7));
                s10_y <= C_PAL_Y(v_pidx);
                s10_u <= C_PAL_U(v_pidx);
                s10_v <= C_PAL_V(v_pidx);
            end if;
            s10_arm <= s9_arm;

            -- S11: Y * arm (raw)
            s11_ymul <= s10_y * s10_arm;
            s11_u    <= s10_u;
            s11_v    <= s10_v;

            -- S12: extract + Y * bright (raw)
            s12_y_in  <= s11_ymul(19 downto 10);
            s12_ymul2 <= s11_ymul(19 downto 10) * bright_r;
            s12_u <= s11_u;
            s12_v <= s11_v;

            -- S13: extract + invert + pulse
            v_y_mul := s12_ymul2;
            if pulse_r = '1' and frame_count(3) = '1' then
                v_y_mul := "00" & v_y_mul(19 downto 2);
            end if;
            if invert_r = '1' then
                s13_y <= to_unsigned(1023, 10) - v_y_mul(19 downto 10);
            else
                s13_y <= v_y_mul(19 downto 10);
            end if;
            s13_u <= s12_u;
            s13_v <= s12_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s13_y);
    data_out.u       <= std_logic_vector(s13_u);
    data_out.v       <= std_logic_vector(s13_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture spiral;
