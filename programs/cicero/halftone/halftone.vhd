-- Halftone: newspaper-print dot screen of incoming video.
--
-- The screen is sampled on a rotated grid of cells; within each cell, a
-- disc-shaped dot is drawn whose radius grows as the local luma falls
-- (dark areas → big black dots, bright areas → small dots).  Toggle CMY
-- mode tints the dots one of three colors based on which channel of the
-- input is strongest.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).  Rotation
-- multiplies are split across two stages.
--
-- Register map:
--   registers_in(0) = Angle         (rotary 1 — screen rotation)
--   registers_in(1) = Contrast      (rotary 2 — luma→dot-size curve gain)
--   registers_in(2) = Threshold     (rotary 3 — black/white tipping point)
--   registers_in(3) = Bleed         (rotary 4 — soft edge falloff)
--   registers_in(4) = Tint Hue      (rotary 5 — palette rotation)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Channel  (0=Luma / 1=Chroma magnitude)
--     b1   Shape    (0=Round / 1=Square)
--     b2   Colors   (0=Mono / 1=CMY)
--     b3   Dot Polarity (0=Black on white / 1=White on black)
--     b4   Invert
--   registers_in(7) = Dot Density (slider, KEY — cell size)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture halftone of program_top is

    constant LATENCY : natural := 13;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal angle_r     : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal contrast_r  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal thresh_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bleed_r     : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal tint_r      : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal bright_r    : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal density_r   : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal channel_r   : std_logic := '0';
    signal shape_r     : std_logic := '0';
    signal cmy_r       : std_logic := '0';
    signal polarity_r  : std_logic := '0';
    signal invert_r    : std_logic := '0';

    -- Precomputed cos/sin
    signal cos_r : signed(10 downto 0) := to_signed(491, 11);
    signal sin_r : signed(10 downto 0) := to_signed(0,   11);

    -- Cell size precomputed at vsync from density_r
    -- density 0..1023 → cell_size 8..40
    signal cell_size_r : unsigned(5 downto 0) := to_unsigned(20, 6);

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

    -- Pipeline signals
    signal s0_xc : signed(12 downto 0);
    signal s0_yc : signed(12 downto 0);
    signal s0_y_in : unsigned(9 downto 0);
    signal s0_u_in : unsigned(9 downto 0);
    signal s0_v_in : unsigned(9 downto 0);

    -- S1: rotation multiplies (4 in parallel, 13×11 = 24 bits)
    signal s1_xc_cos : signed(23 downto 0);
    signal s1_xc_sin : signed(23 downto 0);
    signal s1_yc_cos : signed(23 downto 0);
    signal s1_yc_sin : signed(23 downto 0);
    signal s1_y_in   : unsigned(9 downto 0);
    signal s1_u_in   : unsigned(9 downto 0);
    signal s1_v_in   : unsigned(9 downto 0);

    -- S2: rotated coordinates xr = xc_cos - yc_sin (shifted), yr = xc_sin + yc_cos
    signal s2_xr : signed(24 downto 0);
    signal s2_yr : signed(24 downto 0);
    signal s2_y_in : unsigned(9 downto 0);
    signal s2_u_in : unsigned(9 downto 0);
    signal s2_v_in : unsigned(9 downto 0);

    -- S3: take a window of bits as integer rotated coords (mod 4096-ish)
    -- We shift by 9 (since coefficients are ±491 ≈ 2^9) to bring back to ~12-bit
    signal s3_xr : signed(15 downto 0);
    signal s3_yr : signed(15 downto 0);
    signal s3_y_in : unsigned(9 downto 0);
    signal s3_u_in : unsigned(9 downto 0);
    signal s3_v_in : unsigned(9 downto 0);

    -- S4: cell-local coords = xr mod cell_size, then offset by cell_size/2
    -- cell_size is 8..40, not power of two, so use a comparator/subtractor
    -- approach: take low 6 bits (covers 0..63), subtract cell_size if >= cell_size
    -- Actually simpler: take xr mod 32 if cell_size==32, or use bit slice for
    -- power-of-two cell sizes.  Map density to cell_size_pow2 ∈ {3,4,5,6}
    -- meaning cell_size ∈ {8, 16, 32, 64}.  Use cell_size_pow2 bit mask.
    -- Cell modulo: low N bits where N is cell_pow2
    signal s4_cx : signed(6 downto 0);
    signal s4_cy : signed(6 downto 0);
    signal s4_y_in : unsigned(9 downto 0);
    signal s4_u_in : unsigned(9 downto 0);
    signal s4_v_in : unsigned(9 downto 0);

    -- S5: |dx|, |dy|
    signal s5_adx : unsigned(5 downto 0);
    signal s5_ady : unsigned(5 downto 0);
    signal s5_y_in : unsigned(9 downto 0);
    signal s5_u_in : unsigned(9 downto 0);
    signal s5_v_in : unsigned(9 downto 0);

    -- S6: distance = |dx| + |dy|
    signal s6_dist : unsigned(6 downto 0);
    signal s6_y_in : unsigned(9 downto 0);
    signal s6_u_in : unsigned(9 downto 0);
    signal s6_v_in : unsigned(9 downto 0);

    -- S7: compute input luma metric (Y or chroma mag) and apply contrast
    signal s7_metric : unsigned(9 downto 0);
    signal s7_dist   : unsigned(6 downto 0);
    signal s7_u_in   : unsigned(9 downto 0);
    signal s7_v_in   : unsigned(9 downto 0);

    -- S8: threshold = density-derived value modulated by metric
    -- dot_radius = (1023 - metric) * cell_size / (1024 * 4)
    -- Pre-shifted multiplier kept simple via raw mul.
    signal s8_radius_mul : unsigned(15 downto 0);
    signal s8_dist       : unsigned(6 downto 0);
    signal s8_u_in       : unsigned(9 downto 0);
    signal s8_v_in       : unsigned(9 downto 0);

    -- S9: extract radius (top of mul)
    signal s9_radius : unsigned(6 downto 0);
    signal s9_dist   : unsigned(6 downto 0);
    signal s9_u_in   : unsigned(9 downto 0);
    signal s9_v_in   : unsigned(9 downto 0);

    -- S10: dot_on if dist < radius, with soft edge
    signal s10_dot_brt : unsigned(9 downto 0);
    signal s10_u_in    : unsigned(9 downto 0);
    signal s10_v_in    : unsigned(9 downto 0);

    -- S11: assign final Y/U/V from polarity + dot color
    signal s11_y : unsigned(9 downto 0);
    signal s11_u : unsigned(9 downto 0);
    signal s11_v : unsigned(9 downto 0);

    -- S12: invert
    signal s12_y : unsigned(9 downto 0);
    signal s12_u : unsigned(9 downto 0);
    signal s12_v : unsigned(9 downto 0);

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

    -- Cell size in pow-2 form (3..6, corresponding to 8/16/32/64)
    signal cell_pow_r : unsigned(2 downto 0) := to_unsigned(5, 3);

begin

    -- ========================================================================
    -- Position + parameters + cos/sin precompute.
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
                angle_r    <= unsigned(registers_in(0));
                contrast_r <= unsigned(registers_in(1));
                thresh_r   <= unsigned(registers_in(2));
                bleed_r    <= unsigned(registers_in(3));
                tint_r     <= unsigned(registers_in(4));
                bright_r   <= unsigned(registers_in(5));
                channel_r  <= registers_in(6)(0);
                shape_r    <= registers_in(6)(1);
                cmy_r      <= registers_in(6)(2);
                polarity_r <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                density_r  <= unsigned(registers_in(7));

                -- Precompute cos/sin of angle
                cos_r <= quad_sin(unsigned(registers_in(0)) +
                                  to_unsigned(256, 10));
                sin_r <= quad_sin(unsigned(registers_in(0)));

                -- Map density 0..1023 to cell_pow (3..6 = sizes 8..64)
                case to_integer(unsigned(registers_in(7)(9 downto 8))) is
                    when 0 => cell_pow_r <= to_unsigned(3, 3); -- 8 px
                    when 1 => cell_pow_r <= to_unsigned(4, 3); -- 16 px
                    when 2 => cell_pow_r <= to_unsigned(5, 3); -- 32 px
                    when others => cell_pow_r <= to_unsigned(6, 3); -- 64 px
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
        variable v_dx : signed(6 downto 0);
        variable v_dy : signed(6 downto 0);
        variable v_chroma_u : signed(11 downto 0);
        variable v_chroma_v : signed(11 downto 0);
        variable v_u_mag    : unsigned(11 downto 0);
        variable v_v_mag    : unsigned(11 downto 0);
        variable v_inside   : boolean;
        variable v_pow_mask : unsigned(5 downto 0);
        variable v_pow      : integer range 3 to 6;
        variable v_xr_mod   : signed(6 downto 0);
        variable v_yr_mod   : signed(6 downto 0);
        variable v_brt      : unsigned(9 downto 0);
        variable v_metric   : unsigned(9 downto 0);
        variable v_diff     : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: centered coords + capture input YUV
            s0_xc <= signed(resize(pixel_x, 13)) - to_signed(960, 13);
            s0_yc <= signed(resize(pixel_y, 13)) - to_signed(540, 13);
            s0_y_in <= unsigned(data_in.y);
            s0_u_in <= unsigned(data_in.u);
            s0_v_in <= unsigned(data_in.v);

            -- S1: 4 rotation multiplies in parallel
            s1_xc_cos <= s0_xc * cos_r;
            s1_xc_sin <= s0_xc * sin_r;
            s1_yc_cos <= s0_yc * cos_r;
            s1_yc_sin <= s0_yc * sin_r;
            s1_y_in <= s0_y_in;
            s1_u_in <= s0_u_in;
            s1_v_in <= s0_v_in;

            -- S2: rotated coords (signed adds)
            s2_xr <= resize(s1_xc_cos, 25) - resize(s1_yc_sin, 25);
            s2_yr <= resize(s1_xc_sin, 25) + resize(s1_yc_cos, 25);
            s2_y_in <= s1_y_in;
            s2_u_in <= s1_u_in;
            s2_v_in <= s1_v_in;

            -- S3: shift right 9 to get back to pixel-scale rotated coords
            s3_xr <= resize(shift_right(s2_xr, 9), 16);
            s3_yr <= resize(shift_right(s2_yr, 9), 16);
            s3_y_in <= s2_y_in;
            s3_u_in <= s2_u_in;
            s3_v_in <= s2_v_in;

            -- S4: extract cell-local coords.
            -- Take low cell_pow bits → 0..(cell_size-1), subtract cell_size/2 → signed.
            v_pow := to_integer(cell_pow_r);
            v_pow_mask := (others => '0');
            for i in 0 to 5 loop
                if i < v_pow then v_pow_mask(i) := '1'; end if;
            end loop;

            -- Take low 6 bits of xr, mask to cell_size-1, then subtract cell_size/2
            v_xr_mod := signed('0' & std_logic_vector(
                          unsigned(std_logic_vector(s3_xr(5 downto 0))) and v_pow_mask));
            v_yr_mod := signed('0' & std_logic_vector(
                          unsigned(std_logic_vector(s3_yr(5 downto 0))) and v_pow_mask));
            -- Subtract cell_size/2 = 2^(pow-1).  Use pre-shifted constant.
            case v_pow is
                when 3 => v_xr_mod := v_xr_mod - to_signed(4, 7);
                          v_yr_mod := v_yr_mod - to_signed(4, 7);
                when 4 => v_xr_mod := v_xr_mod - to_signed(8, 7);
                          v_yr_mod := v_yr_mod - to_signed(8, 7);
                when 5 => v_xr_mod := v_xr_mod - to_signed(16, 7);
                          v_yr_mod := v_yr_mod - to_signed(16, 7);
                when others => v_xr_mod := v_xr_mod - to_signed(32, 7);
                               v_yr_mod := v_yr_mod - to_signed(32, 7);
            end case;
            s4_cx <= v_xr_mod;
            s4_cy <= v_yr_mod;
            s4_y_in <= s3_y_in;
            s4_u_in <= s3_u_in;
            s4_v_in <= s3_v_in;

            -- S5: |dx|, |dy|
            v_dx := s4_cx;
            if v_dx < 0 then v_dx := -v_dx; end if;
            s5_adx <= unsigned(std_logic_vector(v_dx(5 downto 0)));
            v_dy := s4_cy;
            if v_dy < 0 then v_dy := -v_dy; end if;
            s5_ady <= unsigned(std_logic_vector(v_dy(5 downto 0)));
            s5_y_in <= s4_y_in;
            s5_u_in <= s4_u_in;
            s5_v_in <= s4_v_in;

            -- S6: distance metric.  Round = |dx|+|dy| (Manhattan, diamond
            -- shape); Square = max(|dx|, |dy|).  Approximation but ok.
            if shape_r = '0' then
                s6_dist <= resize(s5_adx, 7) + resize(s5_ady, 7);
            else
                if s5_adx > s5_ady then
                    s6_dist <= resize(s5_adx, 7);
                else
                    s6_dist <= resize(s5_ady, 7);
                end if;
            end if;
            s6_y_in <= s5_y_in;
            s6_u_in <= s5_u_in;
            s6_v_in <= s5_v_in;

            -- S7: metric from luma (or chroma magnitude).
            -- With dark input → big dots; bright → small.  metric = input value.
            if channel_r = '0' then
                v_metric := s6_y_in;
            else
                v_chroma_u := signed(resize(s6_u_in, 12)) - to_signed(512, 12);
                v_chroma_v := signed(resize(s6_v_in, 12)) - to_signed(512, 12);
                if v_chroma_u < 0 then v_chroma_u := -v_chroma_u; end if;
                if v_chroma_v < 0 then v_chroma_v := -v_chroma_v; end if;
                v_u_mag := unsigned(std_logic_vector(v_chroma_u(11 downto 0)));
                v_v_mag := unsigned(std_logic_vector(v_chroma_v(11 downto 0)));
                if v_u_mag(11 downto 2) > v_v_mag(11 downto 2) then
                    v_metric := v_u_mag(11 downto 2);
                else
                    v_metric := v_v_mag(11 downto 2);
                end if;
            end if;
            s7_metric <= v_metric;
            s7_dist   <= s6_dist;
            s7_u_in   <= s6_u_in;
            s7_v_in   <= s6_v_in;

            -- S8: compute dot radius from inverted metric * contrast knob
            -- radius_metric_inv = 1023 - metric (10 bits)
            -- mul by contrast (10) = 20 bits
            s8_radius_mul <= resize(
                (to_unsigned(1023, 10) - s7_metric) *
                resize(contrast_r(9 downto 4), 6), 16);
            s8_dist <= s7_dist;
            s8_u_in <= s7_u_in;
            s8_v_in <= s7_v_in;

            -- S9: extract radius (top of mul) and bound it to cell_size/2.
            -- 6×10 = 16-bit; >> 7 gives 9-bit range; clamp to 0..2^(pow-1).
            s9_radius <= s8_radius_mul(13 downto 7);
            s9_dist   <= s8_dist;
            s9_u_in   <= s8_u_in;
            s9_v_in   <= s8_v_in;

            -- S10: dot brightness (with bleed soft edge)
            v_diff := signed('0' & std_logic_vector(s9_radius)) -
                      signed('0' & std_logic_vector(s9_dist));
            if v_diff > 0 then
                -- inside dot
                v_brt := to_unsigned(1023, 10);
            elsif v_diff > to_signed(-3, 8) then
                -- soft edge
                v_brt := to_unsigned(700, 10);
            else
                v_brt := to_unsigned(0, 10);
            end if;
            s10_dot_brt <= v_brt;
            s10_u_in <= s9_u_in;
            s10_v_in <= s9_v_in;

            -- S11: assemble output color.
            -- Polarity 0 = dot Y = 0 (dark dot on bright bg = 1023)
            -- Polarity 1 = dot Y = 1023 (bright dot on dark bg = 0)
            if polarity_r = '0' then
                s11_y <= to_unsigned(1023, 10) - s10_dot_brt;
            else
                s11_y <= s10_dot_brt;
            end if;
            -- Chroma: in CMY mode, tint by dominant input channel; otherwise neutral
            if cmy_r = '1' then
                -- Tint dot color toward input chroma + tint knob
                s11_u <= resize(s10_u_in(9 downto 1), 10)
                         + resize(tint_r(9 downto 4), 10)
                         + to_unsigned(64, 10);
                s11_v <= resize(s10_v_in(9 downto 1), 10)
                         - resize(tint_r(9 downto 4), 10)
                         + to_unsigned(384, 10);
            else
                s11_u <= to_unsigned(512, 10);
                s11_v <= to_unsigned(512, 10);
            end if;

            -- S12: invert
            if invert_r = '1' then
                s12_y <= to_unsigned(1023, 10) - s11_y;
            else
                s12_y <= s11_y;
            end if;
            s12_u <= s11_u;
            s12_v <= s11_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s12_y);
    data_out.u       <= std_logic_vector(s12_u);
    data_out.v       <= std_logic_vector(s12_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture halftone;
