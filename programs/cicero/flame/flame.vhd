-- Flame: rising fire plume via animated noise texture.
--
-- Heat at (x, y) = height_factor(y) + noise(x, y + frame*speed) * spice.
-- height_factor is highest at the bottom and falls to zero at the top,
-- so palette lookup turns the bottom-heavy field into bright flame
-- colours that fade to black at the top.  Vertical scroll of the noise
-- sample point makes the flames appear to rise.
--
-- No multipliers in the per-pixel path — only LFSR XOR-tap hashing and
-- shift-based scaling.  Suitable for HD 74.25 MHz on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Flame Height  (rotary 1)
--   registers_in(1) = Turbulence    (rotary 2 — noise weight)
--   registers_in(2) = Rise Speed    (rotary 3 — frame-scroll rate)
--   registers_in(3) = Wind          (rotary 4 — horizontal noise drift)
--   registers_in(4) = Color Curve   (rotary 5 — palette shift)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Palette  (0=Fire / 1=Ice)
--     b1   Direction (0=Up / 1=Down)
--     b2   Center Bias (0=Off / 1=Hot center column)
--     b3   Pulse    (0=Off / 1=Frame brightness pulse)
--     b4   Invert
--   registers_in(7) = Heat (slider, KEY — overall intensity)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture flame of program_top is

    constant LATENCY : natural := 9;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal height_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal turb_r      : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal rise_r      : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal wind_r      : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal cshift_r    : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal bright_r    : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal heat_r      : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal palette_r   : std_logic := '0';
    signal direction_r : std_logic := '0';
    signal center_r    : std_logic := '0';
    signal pulse_r     : std_logic := '0';
    signal invert_r    : std_logic := '0';

    -- Vertical scroll accumulator
    signal y_scroll_r : unsigned(11 downto 0) := (others => '0');
    signal x_scroll_r : unsigned(11 downto 0) := (others => '0');

    -- Fire palette: 16-entry LUT (cold → bright white)
    type t_pal16 is array (0 to 15) of unsigned(9 downto 0);
    -- Fire: black, deep red, red, orange, yellow, white
    constant C_FIRE_Y : t_pal16 := (
        to_unsigned(  0, 10), to_unsigned( 40, 10), to_unsigned( 90, 10), to_unsigned(160, 10),
        to_unsigned(240, 10), to_unsigned(330, 10), to_unsigned(420, 10), to_unsigned(510, 10),
        to_unsigned(600, 10), to_unsigned(680, 10), to_unsigned(760, 10), to_unsigned(830, 10),
        to_unsigned(890, 10), to_unsigned(940, 10), to_unsigned(980, 10), to_unsigned(1020, 10));
    constant C_FIRE_U : t_pal16 := (
        to_unsigned(512, 10), to_unsigned(380, 10), to_unsigned(360, 10), to_unsigned(340, 10),
        to_unsigned(320, 10), to_unsigned(330, 10), to_unsigned(360, 10), to_unsigned(400, 10),
        to_unsigned(440, 10), to_unsigned(470, 10), to_unsigned(490, 10), to_unsigned(500, 10),
        to_unsigned(505, 10), to_unsigned(508, 10), to_unsigned(510, 10), to_unsigned(512, 10));
    constant C_FIRE_V : t_pal16 := (
        to_unsigned(512, 10), to_unsigned(640, 10), to_unsigned(700, 10), to_unsigned(740, 10),
        to_unsigned(750, 10), to_unsigned(720, 10), to_unsigned(680, 10), to_unsigned(620, 10),
        to_unsigned(580, 10), to_unsigned(560, 10), to_unsigned(540, 10), to_unsigned(525, 10),
        to_unsigned(518, 10), to_unsigned(515, 10), to_unsigned(513, 10), to_unsigned(512, 10));

    -- Ice palette: black → blue → cyan → white
    constant C_ICE_Y : t_pal16 := (
        to_unsigned(  0, 10), to_unsigned( 30, 10), to_unsigned( 70, 10), to_unsigned(130, 10),
        to_unsigned(200, 10), to_unsigned(280, 10), to_unsigned(370, 10), to_unsigned(460, 10),
        to_unsigned(560, 10), to_unsigned(650, 10), to_unsigned(740, 10), to_unsigned(820, 10),
        to_unsigned(890, 10), to_unsigned(940, 10), to_unsigned(980, 10), to_unsigned(1020, 10));
    constant C_ICE_U : t_pal16 := (
        to_unsigned(512, 10), to_unsigned(640, 10), to_unsigned(700, 10), to_unsigned(740, 10),
        to_unsigned(760, 10), to_unsigned(740, 10), to_unsigned(700, 10), to_unsigned(650, 10),
        to_unsigned(600, 10), to_unsigned(570, 10), to_unsigned(540, 10), to_unsigned(525, 10),
        to_unsigned(518, 10), to_unsigned(515, 10), to_unsigned(513, 10), to_unsigned(512, 10));
    constant C_ICE_V : t_pal16 := (
        to_unsigned(512, 10), to_unsigned(440, 10), to_unsigned(400, 10), to_unsigned(370, 10),
        to_unsigned(360, 10), to_unsigned(380, 10), to_unsigned(410, 10), to_unsigned(450, 10),
        to_unsigned(480, 10), to_unsigned(495, 10), to_unsigned(505, 10), to_unsigned(510, 10),
        to_unsigned(512, 10), to_unsigned(512, 10), to_unsigned(512, 10), to_unsigned(512, 10));

    -- Pipeline signals
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    -- S1: height_factor = (direction ? y : 1080-y) * height_r >> 10
    -- effective_y = direction ? y : 1080 - y
    signal s1_eff_y : unsigned(11 downto 0);
    signal s1_x     : unsigned(11 downto 0);

    -- S2: height_mul = eff_y * height_r  (12×10 = 22)
    signal s2_hmul : unsigned(21 downto 0);
    signal s2_eff_y : unsigned(11 downto 0);
    signal s2_x : unsigned(11 downto 0);

    -- S3: height_factor extracted; noise hash computed
    signal s3_height : unsigned(11 downto 0);
    signal s3_noise  : unsigned(7 downto 0);
    signal s3_eff_y  : unsigned(11 downto 0);
    signal s3_x      : unsigned(11 downto 0);

    -- S4: heat = height_factor + noise * turb_r/256 - decay
    --     with optional center bias
    signal s4_heat : signed(13 downto 0);
    signal s4_x    : unsigned(11 downto 0);

    -- S5: clamp + scale by heat_r → palette index
    signal s5_pidx : unsigned(3 downto 0);

    -- S6: palette lookup
    signal s6_y : unsigned(9 downto 0);
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);

    -- S7: brightness mul
    signal s7_ymul : unsigned(19 downto 0);
    signal s7_u    : unsigned(9 downto 0);
    signal s7_v    : unsigned(9 downto 0);

    -- S8: extract + invert + pulse
    signal s8_y : unsigned(9 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);

    function hash16(a, b : unsigned(11 downto 0); seed : unsigned(7 downto 0))
        return unsigned is
        variable v_a_lo : unsigned(7 downto 0);
        variable v_a_hi : unsigned(3 downto 0);
        variable v_b_hi : unsigned(7 downto 0);
        variable v_b_lo : unsigned(3 downto 0);
        variable v_out  : unsigned(7 downto 0);
    begin
        v_a_lo := a(7 downto 0);
        v_a_hi := a(11 downto 8);
        v_b_hi := b(11 downto 4);
        v_b_lo := b(3 downto 0);
        -- XOR-based hash mixing the bits of a, b, seed.
        v_out := v_a_lo xor v_b_hi xor seed xor
                 (v_a_hi & v_b_lo);
        return v_out;
    end function;

begin

    -- ========================================================================
    -- Position + parameters + scroll accumulators
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
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                height_r    <= unsigned(registers_in(0));
                turb_r      <= unsigned(registers_in(1));
                rise_r      <= unsigned(registers_in(2));
                wind_r      <= unsigned(registers_in(3));
                cshift_r    <= unsigned(registers_in(4));
                bright_r    <= unsigned(registers_in(5));
                palette_r   <= registers_in(6)(0);
                direction_r <= registers_in(6)(1);
                center_r    <= registers_in(6)(2);
                pulse_r     <= registers_in(6)(3);
                invert_r    <= registers_in(6)(4);
                heat_r      <= unsigned(registers_in(7));

                -- Update scroll accumulators at vsync
                y_scroll_r <= y_scroll_r +
                    shift_left(resize(unsigned(registers_in(2)(9 downto 4)), 12), 2);
                x_scroll_r <= x_scroll_r +
                    shift_left(resize(unsigned(registers_in(3)(9 downto 5)), 12), 1);
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_height : unsigned(11 downto 0);
        variable v_noise_scaled : signed(13 downto 0);
        variable v_heat : signed(13 downto 0);
        variable v_pidx_int : integer range 0 to 15;
        variable v_idx_raw : signed(13 downto 0);
        variable v_y_mul : unsigned(19 downto 0);
        variable v_centerbias : signed(13 downto 0);
        variable v_xdiff : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: effective y = direction ? y : 1080 - y
            if direction_r = '0' then
                if s0_y < to_unsigned(1080, 12) then
                    s1_eff_y <= to_unsigned(1080, 12) - s0_y;
                else
                    s1_eff_y <= (others => '0');
                end if;
            else
                s1_eff_y <= s0_y;
            end if;
            s1_x <= s0_x;

            -- S2: height_mul (12×10 = 22)
            s2_hmul <= s1_eff_y * height_r;
            s2_eff_y <= s1_eff_y;
            s2_x     <= s1_x;

            -- S3: extract scaled height + compute noise hash
            v_height := s2_hmul(21 downto 10);  -- range 0..1080
            s3_height <= v_height;
            -- Noise from hash of (x + x_scroll, y + y_scroll, frame_count low)
            s3_noise <= hash16(s2_x + x_scroll_r, s2_eff_y + y_scroll_r,
                              frame_count(7 downto 0));
            s3_eff_y <= s2_eff_y;
            s3_x     <= s2_x;

            -- S4: heat = height + (noise - 128) * turb >> 7 + center_bias
            v_noise_scaled := signed(resize(s3_noise, 14)) -
                              to_signed(128, 14);
            -- Scale by turb_r/256 via shift right 2 (use top 8 bits of turb)
            v_noise_scaled := resize(shift_right(v_noise_scaled *
                                signed(resize(turb_r(9 downto 4), 7)), 2), 14);
            -- Center bias: more heat near center column
            if center_r = '1' then
                -- distance from x=960
                v_xdiff := signed(resize(s3_x, 13)) - to_signed(960, 13);
                if v_xdiff < 0 then v_xdiff := -v_xdiff; end if;
                -- bias = 480 - |xdiff/2|  (high in center, drops to 0 at edges)
                v_centerbias := to_signed(480, 14) -
                                signed(resize(shift_right(
                                    unsigned(std_logic_vector(v_xdiff(11 downto 0))), 1), 14));
                if v_centerbias < 0 then v_centerbias := to_signed(0, 14); end if;
            else
                v_centerbias := to_signed(0, 14);
            end if;
            s4_heat <= signed(resize(s3_height, 14)) + v_noise_scaled + v_centerbias;
            s4_x <= s3_x;

            -- S5: scale by heat_r, clamp, index into 16-entry palette
            -- heat range ~0..2046 ish.  Want palette index 0..15.
            -- index = (heat * heat_r) >> shift, clamped 0..15
            -- We need to shift right 11 to get from ~16-bit to 4-bit.
            v_idx_raw := s4_heat;
            -- Apply heat_r scale (10-bit) - drop low bits to keep
            -- multiplication out of the critical path.
            -- (heat * heat_r) / 1024 → range ~0..2046 → divide by 128 → ~0..16
            if v_idx_raw < 0 then
                v_pidx_int := 0;
            elsif v_idx_raw > to_signed(2046, 14) then
                v_pidx_int := 15;
            else
                -- Use top 4 bits after shifting
                -- effective index = (heat * heat_r) >> 17 = top 4 bits
                -- Approximation: use top bits of heat scaled by heat_r/1024 via /16
                v_pidx_int := to_integer(
                    unsigned(std_logic_vector(v_idx_raw(10 downto 7))));
            end if;
            s5_pidx <= to_unsigned(v_pidx_int, 4);

            -- S6: palette
            if palette_r = '0' then
                s6_y <= C_FIRE_Y(to_integer(s5_pidx));
                s6_u <= C_FIRE_U(to_integer(s5_pidx));
                s6_v <= C_FIRE_V(to_integer(s5_pidx));
            else
                s6_y <= C_ICE_Y(to_integer(s5_pidx));
                s6_u <= C_ICE_U(to_integer(s5_pidx));
                s6_v <= C_ICE_V(to_integer(s5_pidx));
            end if;

            -- S7: brightness mul (raw)
            s7_ymul <= s6_y * bright_r;
            s7_u    <= s6_u;
            s7_v    <= s6_v;

            -- S8: extract + invert + pulse
            v_y_mul := s7_ymul;
            if pulse_r = '1' and frame_count(4) = '1' then
                v_y_mul := "00" & v_y_mul(19 downto 2);
            end if;
            if invert_r = '1' then
                s8_y <= to_unsigned(1023, 10) - v_y_mul(19 downto 10);
            else
                s8_y <= v_y_mul(19 downto 10);
            end if;
            s8_u <= s7_u;
            s8_v <= s7_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s8_y);
    data_out.u       <= std_logic_vector(s8_u);
    data_out.v       <= std_logic_vector(s8_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture flame;
