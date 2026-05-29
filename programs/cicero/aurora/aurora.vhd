-- Aurora: shimmering vertical color curtains.
--
-- Three vertical "curtains" are drawn at base positions (480, 960, 1440).
-- Each curtain's horizontal center wobbles with a sinusoid that depends on
-- screen Y and a per-curtain animated phase:
--     center_x(y) = base_x_i + amp * sin(y * k_i + phase_i(t))
-- Pixel brightness is a soft falloff vs distance from curtain center.
-- Three curtains use distinct palette colors (aurora green, magenta, teal).
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).  Every wide
-- multiply lives in its own pipeline stage; the 3-way curtain blend is
-- accumulated in a separate stage.
--
-- Register map:
--   registers_in(0) = Wavelength 1   (rotary 1 — curtain 0 vertical period)
--   registers_in(1) = Wavelength 2   (rotary 2 — curtain 1 vertical period)
--   registers_in(2) = Wavelength 3   (rotary 3 — curtain 2 vertical period)
--   registers_in(3) = Drift Speed    (rotary 4 — phase advance per frame)
--   registers_in(4) = Glow Width     (rotary 5 — curtain horizontal width)
--   registers_in(5) = Brightness     (rotary 6 — overall scaling)
--   registers_in(6) = Switches:
--     b0   Bands  (0=Three curtains / 1=One curtain)
--     b1   Palette (0=Aurora / 1=Sunset)
--     b2   Blend  (0=Max / 1=Add)
--     b3   Sparkle (0=Off / 1=LFSR twinkle)
--     b4   Invert
--   registers_in(7) = Amplitude (slider, KEY — sinusoid horizontal sway)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture aurora of program_top is

    constant LATENCY : natural := 17;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Position tracking
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    -- Latched parameters
    type t_u10_3 is array (0 to 2) of unsigned(9 downto 0);
    signal wavelen_r    : t_u10_3 := (others => to_unsigned(256, 10));
    signal drift_r      : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal glow_r       : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal bright_r     : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal amp_r        : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal single_r     : std_logic := '0';
    signal palette_r    : std_logic := '0';
    signal blend_r      : std_logic := '0';
    signal sparkle_r    : std_logic := '0';
    signal invert_r     : std_logic := '0';

    -- Animated per-curtain phases
    type t_phase3 is array (0 to 2) of unsigned(11 downto 0);
    signal phase_r : t_phase3 := (others => (others => '0'));

    -- LFSR for sparkle
    signal lfsr_r : unsigned(15 downto 0) := to_unsigned(48771, 16);

    -- 64-entry quarter-wave sine LUT, scaled 0..127
    type t_quad is array (0 to 63) of unsigned(7 downto 0);
    constant C_SIN_QUAD : t_quad := (
        to_unsigned(  0, 8), to_unsigned(  3, 8), to_unsigned(  6, 8), to_unsigned( 10, 8),
        to_unsigned( 13, 8), to_unsigned( 16, 8), to_unsigned( 19, 8), to_unsigned( 22, 8),
        to_unsigned( 25, 8), to_unsigned( 28, 8), to_unsigned( 31, 8), to_unsigned( 35, 8),
        to_unsigned( 38, 8), to_unsigned( 41, 8), to_unsigned( 44, 8), to_unsigned( 47, 8),
        to_unsigned( 49, 8), to_unsigned( 52, 8), to_unsigned( 55, 8), to_unsigned( 58, 8),
        to_unsigned( 61, 8), to_unsigned( 63, 8), to_unsigned( 66, 8), to_unsigned( 69, 8),
        to_unsigned( 71, 8), to_unsigned( 74, 8), to_unsigned( 76, 8), to_unsigned( 79, 8),
        to_unsigned( 81, 8), to_unsigned( 83, 8), to_unsigned( 86, 8), to_unsigned( 88, 8),
        to_unsigned( 90, 8), to_unsigned( 92, 8), to_unsigned( 94, 8), to_unsigned( 96, 8),
        to_unsigned( 98, 8), to_unsigned(100, 8), to_unsigned(102, 8), to_unsigned(104, 8),
        to_unsigned(106, 8), to_unsigned(107, 8), to_unsigned(109, 8), to_unsigned(110, 8),
        to_unsigned(112, 8), to_unsigned(113, 8), to_unsigned(114, 8), to_unsigned(116, 8),
        to_unsigned(117, 8), to_unsigned(118, 8), to_unsigned(119, 8), to_unsigned(120, 8),
        to_unsigned(121, 8), to_unsigned(122, 8), to_unsigned(123, 8), to_unsigned(124, 8),
        to_unsigned(124, 8), to_unsigned(125, 8), to_unsigned(125, 8), to_unsigned(126, 8),
        to_unsigned(126, 8), to_unsigned(127, 8), to_unsigned(127, 8), to_unsigned(127, 8));

    -- Aurora palette (greens/magenta/teal)
    -- Format: Y, U, V (each 10-bit)
    constant C_AURORA_Y : t_u10_3 := (
        to_unsigned(700, 10), to_unsigned(450, 10), to_unsigned(680, 10));
    constant C_AURORA_U : t_u10_3 := (
        to_unsigned(280, 10), to_unsigned(760, 10), to_unsigned(620, 10));
    constant C_AURORA_V : t_u10_3 := (
        to_unsigned(380, 10), to_unsigned(720, 10), to_unsigned(360, 10));

    -- Sunset palette (orange, pink, gold)
    constant C_SUNSET_Y : t_u10_3 := (
        to_unsigned(580, 10), to_unsigned(640, 10), to_unsigned(820, 10));
    constant C_SUNSET_U : t_u10_3 := (
        to_unsigned(340, 10), to_unsigned(620, 10), to_unsigned(420, 10));
    constant C_SUNSET_V : t_u10_3 := (
        to_unsigned(780, 10), to_unsigned(720, 10), to_unsigned(620, 10));

    -- Curtain base x positions (left, center, right)
    type t_x_base is array (0 to 2) of unsigned(11 downto 0);
    constant C_BASE_X : t_x_base := (
        to_unsigned(480,  12),
        to_unsigned(960,  12),
        to_unsigned(1440, 12));

    -- Pipeline signals
    -- S0
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    -- S1: per-curtain angle = y * wavelen >> 6 + phase
    type t_w22 is array (0 to 2) of unsigned(21 downto 0);
    signal s1_y_prod : t_w22;
    signal s1_x      : unsigned(11 downto 0);

    -- S2: angle12 = y_prod[19:8] + phase
    type t_a12 is array (0 to 2) of unsigned(11 downto 0);
    signal s2_ang : t_a12;
    signal s2_x   : unsigned(11 downto 0);

    -- S3: extract 10-bit angle
    type t_a10 is array (0 to 2) of unsigned(9 downto 0);
    signal s3_ang : t_a10;
    signal s3_x   : unsigned(11 downto 0);

    -- S4: wave eval (signed 10)
    type t_w10s is array (0 to 2) of signed(9 downto 0);
    signal s4_wav : t_w10s;
    signal s4_x   : unsigned(11 downto 0);

    -- S5: wave * amp (signed 10 * signed 11 = signed 20) registered
    type t_w20s is array (0 to 2) of signed(20 downto 0);
    signal s5_amp_prod : t_w20s;
    signal s5_x        : unsigned(11 downto 0);

    -- S6: extract horizontal offset = amp_prod >> 7  (gives ±~256 pixels max)
    type t_off12s is array (0 to 2) of signed(11 downto 0);
    signal s6_off : t_off12s;
    signal s6_x   : unsigned(11 downto 0);

    -- S7: center_x = base_x + offset
    type t_cx13s is array (0 to 2) of signed(13 downto 0);
    signal s7_cx : t_cx13s;
    signal s7_x  : unsigned(11 downto 0);

    -- S8: dx = pixel_x - center_x
    type t_dx14s is array (0 to 2) of signed(13 downto 0);
    signal s8_dx : t_dx14s;

    -- S9: abs(dx) → unsigned 13
    type t_adx13 is array (0 to 2) of unsigned(12 downto 0);
    signal s9_adx : t_adx13;

    -- S10: brightness = max(0, glow - adx) clipped
    signal s10_b : t_u10_3;

    -- S11: pick winner curtain (max brightness) + blend brightness (sum)
    signal s11_max  : unsigned(9 downto 0);
    signal s11_idx  : unsigned(1 downto 0);
    signal s11_sum  : unsigned(10 downto 0);
    signal s11_sprk : std_logic;

    -- S12: palette lookup
    signal s12_y      : unsigned(9 downto 0);
    signal s12_u      : unsigned(9 downto 0);
    signal s12_v      : unsigned(9 downto 0);
    signal s12_b      : unsigned(9 downto 0);
    signal s12_sprk   : std_logic;

    -- S13: scale Y by brightness*bright (registered raw mul)
    signal s13_y_mul : unsigned(19 downto 0);
    signal s13_u     : unsigned(9 downto 0);
    signal s13_v     : unsigned(9 downto 0);
    signal s13_b     : unsigned(9 downto 0);
    signal s13_sprk  : std_logic;

    -- S14: extract intermediate Y, prep chroma centered
    signal s14_y_pre    : unsigned(9 downto 0);
    signal s14_u_cent   : signed(11 downto 0);
    signal s14_v_cent   : signed(11 downto 0);
    signal s14_b        : unsigned(9 downto 0);
    signal s14_sprk     : std_logic;

    -- S15: y_pre * bright (registered raw mul), chroma * brightness raw
    signal s15_y_mul    : unsigned(19 downto 0);
    signal s15_u_mul    : signed(22 downto 0);
    signal s15_v_mul    : signed(22 downto 0);
    signal s15_b        : unsigned(9 downto 0);
    signal s15_sprk     : std_logic;

    -- S16: final extract + invert
    signal s16_y : unsigned(9 downto 0);
    signal s16_u : unsigned(9 downto 0);
    signal s16_v : unsigned(9 downto 0);

    function sat10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

    function quad_sin(ang : unsigned(9 downto 0)) return signed is
        variable v_q   : unsigned(1 downto 0);
        variable v_idx : integer range 0 to 63;
        variable v_mag : unsigned(7 downto 0);
        variable v_signed : signed(9 downto 0);
    begin
        v_q   := ang(9 downto 8);
        v_idx := to_integer(ang(7 downto 2));
        case v_q is
            when "00"   => v_mag := C_SIN_QUAD(v_idx);
            when "01"   => v_mag := C_SIN_QUAD(63 - v_idx);
            when "10"   => v_mag := C_SIN_QUAD(v_idx);
            when others => v_mag := C_SIN_QUAD(63 - v_idx);
        end case;
        if v_q(1) = '0' then
            v_signed := signed(resize(v_mag, 10));
        else
            v_signed := -signed(resize(v_mag, 10));
        end if;
        return v_signed;
    end function;

begin

    -- ========================================================================
    -- Position tracking + parameter latching + phase advance.
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_tap    : std_logic;
        variable v_drift_step : unsigned(9 downto 0);
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

            -- LFSR runs every cycle for sparkle
            v_tap := lfsr_r(15) xor lfsr_r(13) xor lfsr_r(12) xor lfsr_r(10);
            lfsr_r <= lfsr_r(14 downto 0) & v_tap;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                wavelen_r(0) <= unsigned(registers_in(0));
                wavelen_r(1) <= unsigned(registers_in(1));
                wavelen_r(2) <= unsigned(registers_in(2));
                drift_r      <= unsigned(registers_in(3));
                glow_r       <= unsigned(registers_in(4));
                bright_r     <= unsigned(registers_in(5));
                single_r     <= registers_in(6)(0);
                palette_r    <= registers_in(6)(1);
                blend_r      <= registers_in(6)(2);
                sparkle_r    <= registers_in(6)(3);
                invert_r     <= registers_in(6)(4);
                amp_r        <= unsigned(registers_in(7));

                -- Per-curtain phase advance: trace 0 = base, 1 = 1.25×, 2 = 1.6×
                -- Drift knob (0..1023) → base step (0..32 per frame).
                v_drift_step := "00000" & unsigned(registers_in(3)(9 downto 5));
                phase_r(0) <= phase_r(0) + ("00" & v_drift_step);
                phase_r(1) <= phase_r(1) + ("00" & v_drift_step)
                                          + ("0000" & v_drift_step(9 downto 2));
                phase_r(2) <= phase_r(2) + ("00" & v_drift_step)
                                          + ("000" & v_drift_step(9 downto 1));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_dx : signed(13 downto 0);
        variable v_b  : signed(13 downto 0);
        variable v_pidx : integer range 0 to 2;
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: y * wavelen for each curtain (12 × 10 = 22)
            for i in 0 to 2 loop
                s1_y_prod(i) <= s0_y * wavelen_r(i);
            end loop;
            s1_x <= s0_x;

            -- S2: angle = y_prod[19:8] + phase (12-bit)
            for i in 0 to 2 loop
                s2_ang(i) <= s1_y_prod(i)(19 downto 8) + phase_r(i);
            end loop;
            s2_x <= s1_x;

            -- S3: extract 10-bit angle (drop top 2)
            for i in 0 to 2 loop
                s3_ang(i) <= s2_ang(i)(9 downto 0);
            end loop;
            s3_x <= s2_x;

            -- S4: wave eval (-511..+511 actually -127..+127 here)
            for i in 0 to 2 loop
                s4_wav(i) <= quad_sin(s3_ang(i));
            end loop;
            s4_x <= s3_x;

            -- S5: wave * amp (signed 10 × signed 11 = signed 21)
            for i in 0 to 2 loop
                s5_amp_prod(i) <= s4_wav(i) *
                                  signed('0' & std_logic_vector(amp_r));
            end loop;
            s5_x <= s4_x;

            -- S6: extract offset = amp_prod >> 7  (range ~±256)
            for i in 0 to 2 loop
                s6_off(i) <= resize(shift_right(s5_amp_prod(i), 7), 12);
            end loop;
            s6_x <= s5_x;

            -- S7: center_x = base + offset
            for i in 0 to 2 loop
                s7_cx(i) <= signed(resize(C_BASE_X(i), 14))
                            + resize(s6_off(i), 14);
            end loop;
            s7_x <= s6_x;

            -- S8: dx = pixel_x - center_x
            for i in 0 to 2 loop
                s8_dx(i) <= signed(resize(s7_x, 14)) - s7_cx(i);
            end loop;

            -- S9: abs(dx)
            for i in 0 to 2 loop
                v_dx := s8_dx(i);
                if v_dx < 0 then v_dx := -v_dx; end if;
                s9_adx(i) <= unsigned(std_logic_vector(v_dx(12 downto 0)));
            end loop;

            -- S10: brightness = max(0, glow_scaled - adx) saturating to 10-bit
            -- glow knob (0..1023) → width in pixels: shift left 1 → ~0..2046 px
            for i in 0 to 2 loop
                if s9_adx(i) < ("000" & glow_r) then
                    s10_b(i) <= resize(("000" & glow_r) - s9_adx(i), 10);
                else
                    s10_b(i) <= (others => '0');
                end if;
            end loop;

            -- S11: pick winner + sum (using single curtain mode masks others)
            if single_r = '1' then
                -- Only curtain 1 (center)
                s11_max <= s10_b(1);
                s11_idx <= "01";
                s11_sum <= "0" & s10_b(1);
            else
                if blend_r = '0' then
                    -- Max blend
                    if s10_b(0) >= s10_b(1) and s10_b(0) >= s10_b(2) then
                        s11_max <= s10_b(0); s11_idx <= "00";
                    elsif s10_b(1) >= s10_b(2) then
                        s11_max <= s10_b(1); s11_idx <= "01";
                    else
                        s11_max <= s10_b(2); s11_idx <= "10";
                    end if;
                    s11_sum <= "0" & s10_b(0);
                else
                    -- Add blend: pass max for chroma index, sum for brightness
                    if s10_b(0) >= s10_b(1) and s10_b(0) >= s10_b(2) then
                        s11_idx <= "00";
                    elsif s10_b(1) >= s10_b(2) then
                        s11_idx <= "01";
                    else
                        s11_idx <= "10";
                    end if;
                    s11_max <= s10_b(0);  -- placeholder, replaced by sum below
                    -- Sum all three (cap to 10-bit)
                    s11_sum <= ("0" & s10_b(0)) + ("0" & s10_b(1));
                end if;
            end if;

            -- Sparkle = LFSR low bit gated by knob
            s11_sprk <= sparkle_r and lfsr_r(0) and lfsr_r(7);

            -- S12: palette lookup
            v_pidx := to_integer(s11_idx(1 downto 0)) mod 3;
            if palette_r = '0' then
                s12_y <= C_AURORA_Y(v_pidx);
                s12_u <= C_AURORA_U(v_pidx);
                s12_v <= C_AURORA_V(v_pidx);
            else
                s12_y <= C_SUNSET_Y(v_pidx);
                s12_u <= C_SUNSET_U(v_pidx);
                s12_v <= C_SUNSET_V(v_pidx);
            end if;
            if blend_r = '1' then
                -- Use sum (cap at 1023)
                if s11_sum > 1023 then
                    s12_b <= to_unsigned(1023, 10);
                else
                    s12_b <= s11_sum(9 downto 0);
                end if;
            else
                s12_b <= s11_max;
            end if;
            s12_sprk <= s11_sprk;

            -- S13: scale Y by brightness (final luma) registered raw mul
            s13_y_mul <= s12_y * s12_b;
            s13_u     <= s12_u;
            s13_v     <= s12_v;
            s13_b     <= s12_b;
            s13_sprk  <= s12_sprk;
        end if;
    end process p_pipe;

    -- ========================================================================
    -- Output stage (S14): apply brightness knob scaling + sparkle + invert
    -- ========================================================================
    p_out : process(clk)
        variable v_y_brt    : unsigned(9 downto 0);
        variable v_u_out    : signed(13 downto 0);
        variable v_v_out    : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- S14: extract Y from s13_y_mul (>> 10), center chroma
            s14_y_pre  <= s13_y_mul(19 downto 10);
            s14_u_cent <= signed(resize(s13_u, 12)) - to_signed(512, 12);
            s14_v_cent <= signed(resize(s13_v, 12)) - to_signed(512, 12);
            s14_b      <= s13_b;
            s14_sprk   <= s13_sprk;

            -- S15: apply brightness multiplies (registered raw)
            s15_y_mul <= s14_y_pre * bright_r;
            s15_u_mul <= s14_u_cent * signed('0' & std_logic_vector(s14_b));
            s15_v_mul <= s14_v_cent * signed('0' & std_logic_vector(s14_b));
            s15_b     <= s14_b;
            s15_sprk  <= s14_sprk;

            -- S16: final extract + sparkle + invert + chroma re-center
            v_y_brt := s15_y_mul(19 downto 10);
            if s15_sprk = '1' and s15_b > to_unsigned(256, 10) then
                if v_y_brt < to_unsigned(768, 10) then
                    v_y_brt := v_y_brt + to_unsigned(255, 10);
                else
                    v_y_brt := to_unsigned(1023, 10);
                end if;
            end if;
            if invert_r = '1' then
                s16_y <= to_unsigned(1023, 10) - v_y_brt;
            else
                s16_y <= v_y_brt;
            end if;
            v_u_out := resize(shift_right(s15_u_mul, 10), 14) + to_signed(512, 14);
            v_v_out := resize(shift_right(s15_v_mul, 10), 14) + to_signed(512, 14);
            s16_u <= sat10(v_u_out);
            s16_v <= sat10(v_v_out);
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s16_y);
    data_out.u       <= std_logic_vector(s16_u);
    data_out.v       <= std_logic_vector(s16_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture aurora;
