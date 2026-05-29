-- Bayer Dither: ordered dithering with quantization.
--
-- A 4x4 Bayer matrix supplies a per-pixel threshold offset that biases the
-- quantization decision: input luma + threshold is then snapped to one of
-- N levels.  Across many pixels the threshold spreads quantization error
-- into a fine ordered noise pattern that masks banding.
--
-- Register map:
--   registers_in(0) = Levels        (rotary 1 — quantization steps)
--   registers_in(1) = Threshold Bias(rotary 2 — overall level shift)
--   registers_in(2) = Pattern Scale (rotary 3 — multiply x,y for finer/coarser)
--   registers_in(3) = Color Shift   (rotary 4 — chroma rotation)
--   registers_in(4) = Saturation    (rotary 5)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Matrix    (0=Bayer 4x4 / 1=LFSR noise)
--     b1   Channel   (0=Luma / 1=Luma+Chroma)
--     b2   Polarity  (0=Black->White / 1=White->Black)
--     b3   2-Color   (0=Greyscale / 1=Duotone)
--     b4   Invert
--   registers_in(7) = Dither Amount (slider, KEY — strength of threshold)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture bayerdither of program_top is

    constant LATENCY : natural := 8;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal levels_r   : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal bias_r     : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal scale_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal cshift_r   : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal sat_r      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r   : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal amount_r   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal matrix_r   : std_logic := '0';
    signal channel_r  : std_logic := '0';
    signal polarity_r : std_logic := '0';
    signal duo_r      : std_logic := '0';
    signal invert_r   : std_logic := '0';

    -- LFSR for noise-mode dither
    signal lfsr_r : unsigned(15 downto 0) := to_unsigned(42330, 16);

    -- 4x4 Bayer matrix scaled 0..15 → fits in 4 bits.  Index = (y[1:0] << 2) | x[1:0]
    type t_bayer is array (0 to 15) of unsigned(3 downto 0);
    constant C_BAYER : t_bayer := (
        to_unsigned( 0, 4), to_unsigned( 8, 4), to_unsigned( 2, 4), to_unsigned(10, 4),
        to_unsigned(12, 4), to_unsigned( 4, 4), to_unsigned(14, 4), to_unsigned( 6, 4),
        to_unsigned( 3, 4), to_unsigned(11, 4), to_unsigned( 1, 4), to_unsigned( 9, 4),
        to_unsigned(15, 4), to_unsigned( 7, 4), to_unsigned(13, 4), to_unsigned( 5, 4));

    -- Pre-shifted x, y for pattern scale
    -- scale_r 0..1023 → shift 0..3 (4 sizes: 1px, 2px, 4px, 8px Bayer cells)
    signal scale_shift_r : unsigned(1 downto 0) := to_unsigned(1, 2);

    -- Quantization level selector: 2, 4, 8, 16, 32, 64 levels
    -- levels_r top 3 bits → level_shift_r (output Y = (input + thresh) >> level_shift_r << level_shift_r)
    signal level_shift_r : unsigned(2 downto 0) := to_unsigned(6, 3);

    -- Pipeline signals
    signal s0_x      : unsigned(11 downto 0);
    signal s0_y      : unsigned(11 downto 0);
    signal s0_y_in   : unsigned(9 downto 0);
    signal s0_u_in   : unsigned(9 downto 0);
    signal s0_v_in   : unsigned(9 downto 0);
    signal s0_lfsr   : unsigned(15 downto 0);

    -- S1: bayer index, scaled coords
    signal s1_bayer_idx : unsigned(3 downto 0);
    signal s1_y_in      : unsigned(9 downto 0);
    signal s1_u_in      : unsigned(9 downto 0);
    signal s1_v_in      : unsigned(9 downto 0);
    signal s1_lfsr      : unsigned(15 downto 0);

    -- S2: bayer matrix value (4-bit, 0..15)
    signal s2_bayer_val : unsigned(3 downto 0);
    signal s2_y_in      : unsigned(9 downto 0);
    signal s2_u_in      : unsigned(9 downto 0);
    signal s2_v_in      : unsigned(9 downto 0);
    signal s2_lfsr      : unsigned(15 downto 0);

    -- S3: threshold offset (signed 10) from bayer or LFSR
    signal s3_thresh : signed(10 downto 0);
    signal s3_y_in   : unsigned(9 downto 0);
    signal s3_u_in   : unsigned(9 downto 0);
    signal s3_v_in   : unsigned(9 downto 0);

    -- S4: input + threshold (signed 12)
    signal s4_y_biased : signed(11 downto 0);
    signal s4_u_in     : unsigned(9 downto 0);
    signal s4_v_in     : unsigned(9 downto 0);

    -- S5: clamped + quantized Y
    signal s5_y    : unsigned(9 downto 0);
    signal s5_u_in : unsigned(9 downto 0);
    signal s5_v_in : unsigned(9 downto 0);

    -- S6: duotone selection or chroma pass + saturation
    signal s6_y : unsigned(9 downto 0);
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);

    -- S7: invert + brightness output
    signal s7_y : unsigned(9 downto 0);
    signal s7_u : unsigned(9 downto 0);
    signal s7_v : unsigned(9 downto 0);

    function sat10s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- ========================================================================
    -- Position + parameters
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_tap    : std_logic;
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

            v_tap := lfsr_r(15) xor lfsr_r(13) xor lfsr_r(12) xor lfsr_r(10);
            lfsr_r <= lfsr_r(14 downto 0) & v_tap;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                levels_r   <= unsigned(registers_in(0));
                bias_r     <= unsigned(registers_in(1));
                scale_r    <= unsigned(registers_in(2));
                cshift_r   <= unsigned(registers_in(3));
                sat_r      <= unsigned(registers_in(4));
                bright_r   <= unsigned(registers_in(5));
                matrix_r   <= registers_in(6)(0);
                channel_r  <= registers_in(6)(1);
                polarity_r <= registers_in(6)(2);
                duo_r      <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                amount_r   <= unsigned(registers_in(7));

                -- Pattern scale: top 2 bits of scale_r select shift 0..3
                scale_shift_r <= unsigned(registers_in(2)(9 downto 8));

                -- Levels: top 3 bits of levels_r selects quantization shift 1..6
                case to_integer(unsigned(registers_in(0)(9 downto 7))) is
                    when 0 | 1  => level_shift_r <= to_unsigned(1, 3); -- 512 levels (no dither)
                    when 2      => level_shift_r <= to_unsigned(4, 3); -- 64 levels
                    when 3      => level_shift_r <= to_unsigned(5, 3); -- 32
                    when 4      => level_shift_r <= to_unsigned(6, 3); -- 16
                    when 5      => level_shift_r <= to_unsigned(7, 3); -- 8
                    when 6      => level_shift_r <= to_unsigned(8, 3); -- 4
                    when others => level_shift_r <= to_unsigned(9, 3); -- 2
                end case;
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_shift : integer range 0 to 3;
        variable v_xs    : unsigned(11 downto 0);
        variable v_ys    : unsigned(11 downto 0);
        variable v_idx   : integer range 0 to 15;
        variable v_thresh_raw : signed(7 downto 0);
        variable v_thresh_scaled : signed(15 downto 0);
        variable v_y_post : unsigned(9 downto 0);
        variable v_lshift : integer range 1 to 9;
        variable v_mask  : unsigned(9 downto 0);
        variable v_pidx  : integer range 0 to 7;
        variable v_y_mul : unsigned(19 downto 0);
        variable v_chroma_u : signed(11 downto 0);
        variable v_chroma_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: capture inputs
            s0_x <= pixel_x;
            s0_y <= pixel_y;
            s0_y_in <= unsigned(data_in.y);
            s0_u_in <= unsigned(data_in.u);
            s0_v_in <= unsigned(data_in.v);
            s0_lfsr <= lfsr_r;

            -- S1: compute Bayer index = (y[1:0] << 2) | x[1:0] after shifting
            v_shift := to_integer(scale_shift_r);
            v_xs := s0_x srl v_shift;
            v_ys := s0_y srl v_shift;
            s1_bayer_idx <= v_ys(1 downto 0) & v_xs(1 downto 0);
            s1_y_in <= s0_y_in;
            s1_u_in <= s0_u_in;
            s1_v_in <= s0_v_in;
            s1_lfsr <= s0_lfsr;

            -- S2: Bayer LUT lookup
            s2_bayer_val <= C_BAYER(to_integer(s1_bayer_idx));
            s2_y_in <= s1_y_in;
            s2_u_in <= s1_u_in;
            s2_v_in <= s1_v_in;
            s2_lfsr <= s1_lfsr;

            -- S3: threshold = (bayer - 8) * amount/64, signed 8 result, scaled
            -- bayer 0..15 maps to -8..+7 → ×amount → range ~±1023.
            -- Use bayer or LFSR top 4 bits.
            if matrix_r = '0' then
                v_thresh_raw := signed(resize(s2_bayer_val, 8))
                                - to_signed(8, 8);
            else
                v_thresh_raw := signed(resize(s2_lfsr(3 downto 0), 8))
                                - to_signed(8, 8);
            end if;
            -- Scale by amount (0..1023) but keep mul small: thresh_raw * amount(top 6 bits)
            v_thresh_scaled := v_thresh_raw *
                               signed(resize(amount_r(9 downto 4), 8));
            -- result range ~ ±63 * 8 = ±504 (signed 11)
            s3_thresh <= resize(v_thresh_scaled(10 downto 0), 11);
            s3_y_in <= s2_y_in;
            s3_u_in <= s2_u_in;
            s3_v_in <= s2_v_in;

            -- S4: y_biased = y_in + thresh + bias_offset
            s4_y_biased <= signed(resize(s3_y_in, 12))
                           + resize(s3_thresh, 12)
                           + signed(resize(bias_r(9 downto 1), 12))
                           - to_signed(256, 12);
            s4_u_in <= s3_u_in;
            s4_v_in <= s3_v_in;

            -- S5: clamp + quantize.
            -- Quantization: mask out low bits via shift-then-shift.
            v_lshift := to_integer(level_shift_r);
            v_y_post := sat10s(s4_y_biased);
            -- Build mask = 0xFFFF << v_lshift (low bits cleared)
            v_mask := (others => '1');
            for i in 0 to 9 loop
                if i < v_lshift then v_mask(i) := '0'; end if;
            end loop;
            s5_y <= v_y_post and v_mask;
            s5_u_in <= s4_u_in;
            s5_v_in <= s4_v_in;

            -- S6: chroma path. Polarity flip, duotone, channel mode.
            if polarity_r = '1' then
                s6_y <= to_unsigned(1023, 10) - s5_y;
            else
                s6_y <= s5_y;
            end if;

            if duo_r = '1' then
                -- Duotone: low Y = color A, high Y = color B (set by cshift)
                if s5_y(9) = '1' then
                    -- High band → warm tint
                    s6_u <= to_unsigned(380, 10) +
                            resize(cshift_r(9 downto 2), 10);
                    s6_v <= to_unsigned(700, 10) -
                            resize(cshift_r(9 downto 3), 10);
                else
                    -- Low band → cool tint
                    s6_u <= to_unsigned(720, 10) -
                            resize(cshift_r(9 downto 2), 10);
                    s6_v <= to_unsigned(320, 10) +
                            resize(cshift_r(9 downto 3), 10);
                end if;
            elsif channel_r = '1' then
                -- Pass chroma scaled by sat_r >> 1 toward neutral
                v_chroma_u := signed(resize(s5_u_in, 12)) - to_signed(512, 12);
                v_chroma_v := signed(resize(s5_v_in, 12)) - to_signed(512, 12);
                -- Use shift right by 1 for saturation reduction (cheap)
                s6_u <= sat10s(resize(shift_right(v_chroma_u, 1), 12) +
                               to_signed(512, 12));
                s6_v <= sat10s(resize(shift_right(v_chroma_v, 1), 12) +
                               to_signed(512, 12));
            else
                -- Mono
                s6_u <= to_unsigned(512, 10);
                s6_v <= to_unsigned(512, 10);
            end if;

            -- S7: brightness mul (raw) + invert
            v_y_mul := s6_y * bright_r;
            if invert_r = '1' then
                s7_y <= to_unsigned(1023, 10) - v_y_mul(19 downto 10);
            else
                s7_y <= v_y_mul(19 downto 10);
            end if;
            s7_u <= s6_u;
            s7_v <= s6_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s7_y);
    data_out.u       <= std_logic_vector(s7_u);
    data_out.v       <= std_logic_vector(s7_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture bayerdither;
