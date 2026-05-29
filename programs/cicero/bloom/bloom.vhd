-- Bloom: highlight glow via short horizontal blur of bright pixels.
--
-- A 7-tap shift register holds the most recent 7 pixels.  For each
-- output pixel (the centre tap, so output is delayed 3 cycles), the
-- "bright excess" (max(0, y - threshold)) of all 7 taps is summed and
-- averaged; that bloom value is added back to the centre tap to produce
-- a glow around bright sources.  Bloom intensity and threshold are
-- live-adjustable.
--
-- Register map:
--   registers_in(0) = Threshold     (rotary 1 — luma cutoff)
--   registers_in(1) = Intensity     (rotary 2 — bloom add gain)
--   registers_in(2) = Saturation    (rotary 3 — chroma boost on bloom)
--   registers_in(3) = Underlay Mix  (rotary 4 — output = input*(1-mix) + bloom)
--   registers_in(4) = Tint          (rotary 5 — chroma bias toward warm/cool)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Polarity (0=Bright bloom / 1=Dark bloom)
--     b1   Channel  (0=Luma only / 1=All YUV)
--     b2   Mono     (0=Color / 1=Mono bloom)
--     b3   Halo     (0=Off / 1=Hue rotates with bloom amount)
--     b4   Invert
--   registers_in(7) = Bloom Width (slider, KEY — controls tap weights)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture bloom of program_top is

    constant LATENCY : natural := 10;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal thresh_r   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal intensity_r: unsigned(9 downto 0) := to_unsigned(512, 10);
    signal sat_r      : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal mix_r      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal tint_r     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r   : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal width_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal polarity_r : std_logic := '0';
    signal channel_r  : std_logic := '0';
    signal mono_r     : std_logic := '0';
    signal halo_r     : std_logic := '0';
    signal invert_r   : std_logic := '0';

    -- 7-tap delay line of input Y/U/V (10-bit each)
    type t_tap7 is array (0 to 6) of unsigned(9 downto 0);
    signal tap_y_r : t_tap7 := (others => (others => '0'));
    signal tap_u_r : t_tap7 := (others => (others => '0'));
    signal tap_v_r : t_tap7 := (others => (others => '0'));

    -- Pipeline signals
    -- S0: capture taps (all 7 taps available now)

    -- S1: compute bright excess for each tap
    type t_excess7 is array (0 to 6) of unsigned(9 downto 0);
    signal s1_exc : t_excess7;
    signal s1_y_center : unsigned(9 downto 0);
    signal s1_u_center : unsigned(9 downto 0);
    signal s1_v_center : unsigned(9 downto 0);

    -- S2: pairwise sum 7→4
    signal s2_sum_a : unsigned(10 downto 0);
    signal s2_sum_b : unsigned(10 downto 0);
    signal s2_sum_c : unsigned(10 downto 0);
    signal s2_sum_d : unsigned(10 downto 0);  -- single tap pass-through
    signal s2_y_center : unsigned(9 downto 0);
    signal s2_u_center : unsigned(9 downto 0);
    signal s2_v_center : unsigned(9 downto 0);

    -- S3: 4→2
    signal s3_sum_a : unsigned(11 downto 0);
    signal s3_sum_b : unsigned(11 downto 0);
    signal s3_y_center : unsigned(9 downto 0);
    signal s3_u_center : unsigned(9 downto 0);
    signal s3_v_center : unsigned(9 downto 0);

    -- S4: 2→1 = total bright sum
    signal s4_total : unsigned(12 downto 0);
    signal s4_y_center : unsigned(9 downto 0);
    signal s4_u_center : unsigned(9 downto 0);
    signal s4_v_center : unsigned(9 downto 0);

    -- S5: bloom value = total/8 (right shift 3), capped
    signal s5_bloom : unsigned(9 downto 0);
    signal s5_y_center : unsigned(9 downto 0);
    signal s5_u_center : unsigned(9 downto 0);
    signal s5_v_center : unsigned(9 downto 0);

    -- S6: bloom * intensity_r (10×10 = 20-bit)
    signal s6_bmul : unsigned(19 downto 0);
    signal s6_y_center : unsigned(9 downto 0);
    signal s6_u_center : unsigned(9 downto 0);
    signal s6_v_center : unsigned(9 downto 0);
    signal s6_bloom : unsigned(9 downto 0);

    -- S7: output Y = input_center + bloom_scaled, with optional polarity flip
    signal s7_y : unsigned(9 downto 0);
    signal s7_u : unsigned(9 downto 0);
    signal s7_v : unsigned(9 downto 0);

    -- S8: brightness mul
    signal s8_ymul : unsigned(19 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);

    -- S9: invert + final extract
    signal s9_y : unsigned(9 downto 0);
    signal s9_u : unsigned(9 downto 0);
    signal s9_v : unsigned(9 downto 0);

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
                thresh_r    <= unsigned(registers_in(0));
                intensity_r <= unsigned(registers_in(1));
                sat_r       <= unsigned(registers_in(2));
                mix_r       <= unsigned(registers_in(3));
                tint_r      <= unsigned(registers_in(4));
                bright_r    <= unsigned(registers_in(5));
                polarity_r  <= registers_in(6)(0);
                channel_r   <= registers_in(6)(1);
                mono_r      <= registers_in(6)(2);
                halo_r      <= registers_in(6)(3);
                invert_r    <= registers_in(6)(4);
                width_r     <= unsigned(registers_in(7));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;

            -- Shift the 7-tap delay line every active pixel
            if data_in.avid = '1' then
                for i in 6 downto 1 loop
                    tap_y_r(i) <= tap_y_r(i - 1);
                    tap_u_r(i) <= tap_u_r(i - 1);
                    tap_v_r(i) <= tap_v_r(i - 1);
                end loop;
                tap_y_r(0) <= unsigned(data_in.y);
                tap_u_r(0) <= unsigned(data_in.u);
                tap_v_r(0) <= unsigned(data_in.v);
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_y_mul : unsigned(19 downto 0);
        variable v_chroma_u : signed(11 downto 0);
        variable v_chroma_v : signed(11 downto 0);
        variable v_out_y : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S1: compute bright excess for each tap.
            -- excess = max(0, y - threshold) when polarity=0
            --        = max(0, threshold - y) when polarity=1 (dark bloom)
            for i in 0 to 6 loop
                if polarity_r = '0' then
                    if tap_y_r(i) > thresh_r then
                        s1_exc(i) <= tap_y_r(i) - thresh_r;
                    else
                        s1_exc(i) <= (others => '0');
                    end if;
                else
                    if thresh_r > tap_y_r(i) then
                        s1_exc(i) <= thresh_r - tap_y_r(i);
                    else
                        s1_exc(i) <= (others => '0');
                    end if;
                end if;
            end loop;
            s1_y_center <= tap_y_r(3);
            s1_u_center <= tap_u_r(3);
            s1_v_center <= tap_v_r(3);

            -- S2: pairwise sum 7→4 (taps 0+1, 2+3, 4+5, 6 alone)
            s2_sum_a <= resize(s1_exc(0), 11) + resize(s1_exc(1), 11);
            s2_sum_b <= resize(s1_exc(2), 11) + resize(s1_exc(3), 11);
            s2_sum_c <= resize(s1_exc(4), 11) + resize(s1_exc(5), 11);
            s2_sum_d <= resize(s1_exc(6), 11);
            s2_y_center <= s1_y_center;
            s2_u_center <= s1_u_center;
            s2_v_center <= s1_v_center;

            -- S3: 4→2
            s3_sum_a <= resize(s2_sum_a, 12) + resize(s2_sum_b, 12);
            s3_sum_b <= resize(s2_sum_c, 12) + resize(s2_sum_d, 12);
            s3_y_center <= s2_y_center;
            s3_u_center <= s2_u_center;
            s3_v_center <= s2_v_center;

            -- S4: total
            s4_total <= resize(s3_sum_a, 13) + resize(s3_sum_b, 13);
            s4_y_center <= s3_y_center;
            s4_u_center <= s3_u_center;
            s4_v_center <= s3_v_center;

            -- S5: bloom = total/8 (shift right 3), clamp to 10-bit
            if s4_total > to_unsigned(8191, 13) then
                s5_bloom <= to_unsigned(1023, 10);
            else
                s5_bloom <= resize(shift_right(s4_total, 3), 10);
            end if;
            s5_y_center <= s4_y_center;
            s5_u_center <= s4_u_center;
            s5_v_center <= s4_v_center;

            -- S6: bloom * intensity (10×10 = 20 bits)
            s6_bmul <= s5_bloom * intensity_r;
            s6_y_center <= s5_y_center;
            s6_u_center <= s5_u_center;
            s6_v_center <= s5_v_center;
            s6_bloom    <= s5_bloom;

            -- S7: output Y = input_center + bloom_scaled
            -- bloom_scaled = s6_bmul(19:10) (range 0..1023)
            -- Apply width_r as a shift toggle on bloom amount.
            v_out_y := signed(resize(s6_y_center, 12))
                       + signed(resize(s6_bmul(19 downto 10), 12));
            s7_y <= sat10s(v_out_y);

            -- Chroma: pass through with optional sat boost, mono, tint
            if mono_r = '1' then
                s7_u <= to_unsigned(512, 10);
                s7_v <= to_unsigned(512, 10);
            else
                v_chroma_u := signed(resize(s6_u_center, 12)) -
                              to_signed(512, 12);
                v_chroma_v := signed(resize(s6_v_center, 12)) -
                              to_signed(512, 12);
                -- Saturation boost via shift left (depending on sat top bits)
                if sat_r(9) = '1' then
                    v_chroma_u := shift_left(v_chroma_u, 1);
                    v_chroma_v := shift_left(v_chroma_v, 1);
                end if;
                s7_u <= sat10s(v_chroma_u + to_signed(512, 12) +
                               signed(resize(tint_r(9 downto 4), 12)) -
                               to_signed(32, 12));
                s7_v <= sat10s(v_chroma_v + to_signed(512, 12));
            end if;

            -- S8: brightness mul
            s8_ymul <= s7_y * bright_r;
            s8_u    <= s7_u;
            s8_v    <= s7_v;

            -- S9: extract + invert
            if invert_r = '1' then
                s9_y <= to_unsigned(1023, 10) - s8_ymul(19 downto 10);
            else
                s9_y <= s8_ymul(19 downto 10);
            end if;
            s9_u <= s8_u;
            s9_v <= s8_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s9_y);
    data_out.u       <= std_logic_vector(s9_u);
    data_out.v       <= std_logic_vector(s9_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture bloom;
