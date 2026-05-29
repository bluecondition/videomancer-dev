-- Bandcrush: topographic Y-band colorizer.
--
-- Input Y is quantized into N discrete bands (slider, 2..16).  Each band
-- index then drives an 8-entry palette lookup, so the image becomes a
-- false-color topographic map: each Y range gets one solid colour.  At
-- band boundaries (where the quantized Y level changes from one pixel
-- to the next) the optional Contour overlay paints a bright line — the
-- effect is like contour lines on an elevation map, but applied to
-- whatever the video is showing.
--
-- This is *not* a traditional bit-depth posterizer; it's a Y-driven
-- palette-mapping with edge highlighting and selectable Y output curves.
-- Two palettes are provided: "Topo" (greens-blues-purples-whites, like
-- elevation maps) and "Heatmap" (deep-purple-red-yellow-white,
-- thermal-imaging style).
--
-- Y Mode:
--   - Stepped: output Y comes from the palette entry (solid bands).
--   - Input  : output Y passes through (smooth gradient within bands).
-- Curve:
--   - Linear : band index increases with Y.
--   - Reverse: band index decreases with Y (inverts the topo).
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Y Curve     (signed bias added to Y before banding)
--   registers_in(1) = Contour     (highlight strength at band transitions)
--   registers_in(2) = Palette Shift
--   registers_in(3) = Brightness
--   registers_in(4) = Saturation
--   registers_in(5) = Chroma Mix  (blend between input chroma & palette chroma)
--   registers_in(6) = Switches:
--     b0 Y Mode  (0=Stepped / 1=Input)
--     b1 Contour (0=Off / 1=On)
--     b2 Palette (0=Topo / 1=Heatmap)
--     b3 Curve   (0=Linear / 1=Reverse)
--     b4 Invert
--   registers_in(7) = Bands (slider, KEY — 2..16 quantization bands)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture bandcrush of program_top is

    constant LATENCY : natural := 12;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal y_curve_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal contour_r    : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal pal_shift_r  : unsigned(9 downto 0) := (others => '0');
    signal bright_r     : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal sat_r        : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal chroma_mix_r : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal y_input_r    : std_logic := '0';
    signal contour_on_r : std_logic := '1';
    signal heatmap_r    : std_logic := '0';
    signal reverse_r    : std_logic := '0';
    signal invert_r     : std_logic := '0';
    signal bands_r      : unsigned(9 downto 0) := to_unsigned(384, 10);

    -- Pre-computed shift amount based on bands_r.  shift = 8 means 4 bands,
    -- shift = 4 means 64 bands (capped to 16 effective).  We'll keep a
    -- simple mapping: top 3 bits of bands_r → shift 6..3 (8..32 bands).
    signal y_shift_r : unsigned(2 downto 0) := to_unsigned(6, 3);

    -- Palettes (8 entries each)
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    -- Topo: deep blue → cyan → green → yellow → orange → red → magenta → white
    constant C_PY_TOPO : t_pal := (
        to_unsigned(120, 10), to_unsigned(220, 10),
        to_unsigned(400, 10), to_unsigned(580, 10),
        to_unsigned(700, 10), to_unsigned(800, 10),
        to_unsigned(900, 10), to_unsigned(1000, 10));
    constant C_PU_TOPO : t_pal := (
        to_unsigned(820, 10), to_unsigned(720, 10),
        to_unsigned(360, 10), to_unsigned(360, 10),
        to_unsigned(580, 10), to_unsigned(840, 10),
        to_unsigned(750, 10), to_unsigned(512, 10));
    constant C_PV_TOPO : t_pal := (
        to_unsigned(380, 10), to_unsigned(280, 10),
        to_unsigned(400, 10), to_unsigned(640, 10),
        to_unsigned(800, 10), to_unsigned(950, 10),
        to_unsigned(820, 10), to_unsigned(512, 10));
    -- Heatmap: black → purple → red → orange → yellow → white (thermal)
    constant C_PY_HEAT : t_pal := (
        to_unsigned(  0, 10), to_unsigned(160, 10),
        to_unsigned(300, 10), to_unsigned(460, 10),
        to_unsigned(620, 10), to_unsigned(780, 10),
        to_unsigned(900, 10), to_unsigned(1000, 10));
    constant C_PU_HEAT : t_pal := (
        to_unsigned(512, 10), to_unsigned(620, 10),
        to_unsigned(780, 10), to_unsigned(680, 10),
        to_unsigned(580, 10), to_unsigned(520, 10),
        to_unsigned(510, 10), to_unsigned(512, 10));
    constant C_PV_HEAT : t_pal := (
        to_unsigned(512, 10), to_unsigned(620, 10),
        to_unsigned(900, 10), to_unsigned(820, 10),
        to_unsigned(700, 10), to_unsigned(600, 10),
        to_unsigned(540, 10), to_unsigned(512, 10));

    -- ------------------------------------------------------------------
    -- Pipeline
    -- ------------------------------------------------------------------
    -- S0: register input
    signal s0_y : unsigned(9 downto 0);
    signal s0_u : unsigned(9 downto 0);
    signal s0_v : unsigned(9 downto 0);

    -- S1: signed Y after bias from y_curve (curve = 512 → no shift).
    signal s1_y_b : signed(11 downto 0);
    signal s1_u   : unsigned(9 downto 0);
    signal s1_v   : unsigned(9 downto 0);
    signal s1_y_in : unsigned(9 downto 0);

    -- S2: clamped Y; band index = top bits.
    signal s2_y   : unsigned(9 downto 0);
    signal s2_idx : unsigned(2 downto 0);
    signal s2_u   : unsigned(9 downto 0);
    signal s2_v   : unsigned(9 downto 0);
    signal s2_y_in : unsigned(9 downto 0);
    signal s2_prev_idx : unsigned(2 downto 0) := (others => '0');
    signal s2_edge_r   : std_logic := '0';

    -- S3: palette index after shift / reverse.
    signal s3_pidx : unsigned(2 downto 0);
    signal s3_y    : unsigned(9 downto 0);
    signal s3_y_in : unsigned(9 downto 0);
    signal s3_u    : unsigned(9 downto 0);
    signal s3_v    : unsigned(9 downto 0);
    signal s3_edge : std_logic;

    -- S4: palette lookup.
    signal s4_y    : unsigned(9 downto 0);
    signal s4_u    : unsigned(9 downto 0);
    signal s4_v    : unsigned(9 downto 0);
    signal s4_y_in : unsigned(9 downto 0);
    signal s4_u_in : unsigned(9 downto 0);
    signal s4_v_in : unsigned(9 downto 0);
    signal s4_edge : std_logic;

    -- S5: Y mode mux + chroma mix raw products.
    signal s5_y : unsigned(9 downto 0);
    signal s5_pal_u_p : unsigned(19 downto 0);
    signal s5_inv_u_p : unsigned(19 downto 0);
    signal s5_pal_v_p : unsigned(19 downto 0);
    signal s5_inv_v_p : unsigned(19 downto 0);
    signal s5_edge    : std_logic;

    -- S6: chroma mix output.
    signal s6_y : unsigned(9 downto 0);
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);
    signal s6_edge : std_logic;

    -- S7: brightness multiply (raw).
    signal s7_y_mul : unsigned(19 downto 0);
    signal s7_u     : unsigned(9 downto 0);
    signal s7_v     : unsigned(9 downto 0);
    signal s7_edge  : std_logic;

    -- S8: extract Y; sat multiply raw.
    signal s8_y     : unsigned(9 downto 0);
    signal s8_u_mul : signed(22 downto 0);
    signal s8_v_mul : signed(22 downto 0);
    signal s8_edge  : std_logic;

    -- S9: re-center + contour overlay.
    signal s9_y : unsigned(9 downto 0);
    signal s9_u : unsigned(9 downto 0);
    signal s9_v : unsigned(9 downto 0);

    -- S10: invert + final
    signal s10_y : unsigned(9 downto 0);
    signal s10_u : unsigned(9 downto 0);
    signal s10_v : unsigned(9 downto 0);

    function sat10_s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

    function shift_y(v : unsigned(9 downto 0); s : unsigned(2 downto 0))
        return unsigned is
    begin
        case s is
            when "000"  => return v;
            when "001"  => return "0"   & v(9 downto 1);
            when "010"  => return "00"  & v(9 downto 2);
            when "011"  => return "000" & v(9 downto 3);
            when "100"  => return "0000"& v(9 downto 4);
            when "101"  => return "00000" & v(9 downto 5);
            when "110"  => return "000000" & v(9 downto 6);
            when others => return "0000000" & v(9 downto 7);
        end case;
    end function;

begin

    p_position : process(clk)
        variable v_v_edge : std_logic;
        variable v_band_n : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_v_edge := '0';
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if v_v_edge = '1' then
                y_curve_r    <= unsigned(registers_in(0));
                contour_r    <= unsigned(registers_in(1));
                pal_shift_r  <= unsigned(registers_in(2));
                bright_r     <= unsigned(registers_in(3));
                sat_r        <= unsigned(registers_in(4));
                chroma_mix_r <= unsigned(registers_in(5));
                y_input_r    <= registers_in(6)(0);
                contour_on_r <= registers_in(6)(1);
                heatmap_r    <= registers_in(6)(2);
                reverse_r    <= registers_in(6)(3);
                invert_r     <= registers_in(6)(4);
                bands_r      <= unsigned(registers_in(7));

                -- Map bands_r top 4 bits → shift amount 7..3 (giving 8..128
                -- bands which we then clip to 8 via top-bit selection).
                case registers_in(7)(9 downto 7) is
                    when "000"  => y_shift_r <= to_unsigned(7, 3);
                    when "001"  => y_shift_r <= to_unsigned(7, 3);
                    when "010"  => y_shift_r <= to_unsigned(6, 3);
                    when "011"  => y_shift_r <= to_unsigned(6, 3);
                    when "100"  => y_shift_r <= to_unsigned(5, 3);
                    when "101"  => y_shift_r <= to_unsigned(5, 3);
                    when "110"  => y_shift_r <= to_unsigned(4, 3);
                    when others => y_shift_r <= to_unsigned(4, 3);
                end case;
            end if;
        end if;
    end process p_position;

    p_pipe : process(clk)
        variable v_pidx   : integer range 0 to 7;
        variable v_band   : unsigned(9 downto 0);
        variable v_y_re   : signed(12 downto 0);
        variable v_u_c    : signed(11 downto 0);
        variable v_v_c    : signed(11 downto 0);
        variable v_u_re   : signed(13 downto 0);
        variable v_v_re   : signed(13 downto 0);
        variable v_sum_u  : unsigned(20 downto 0);
        variable v_sum_v  : unsigned(20 downto 0);
        variable v_y_out  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_y <= unsigned(data_in.y);
            s0_u <= unsigned(data_in.u);
            s0_v <= unsigned(data_in.v);

            -- S1: bias Y by Y Curve (curve = 512 → no bias).
            s1_y_b <= signed('0' & std_logic_vector(s0_y)) +
                      signed('0' & std_logic_vector(y_curve_r)) -
                      to_signed(512, 12);
            s1_u    <= s0_u;
            s1_v    <= s0_v;
            s1_y_in <= s0_y;

            -- S2: clamp biased Y → 10-bit, quantize.
            if s1_y_b < 0 then s2_y <= to_unsigned(0, 10);
            elsif s1_y_b > 1023 then s2_y <= to_unsigned(1023, 10);
            else s2_y <= unsigned(std_logic_vector(s1_y_b(9 downto 0)));
            end if;
            -- Band index = top 3 bits of (Y >> y_shift_r).  This gives
            -- one of 8 bands regardless of internal shift; lower shifts
            -- spread the bands more finely across the Y range.
            v_band := shift_y(unsigned(std_logic_vector(s1_y_b(9 downto 0))),
                              y_shift_r);
            s2_idx <= v_band(2 downto 0);
            s2_u <= s1_u;
            s2_v <= s1_v;
            s2_y_in <= s1_y_in;

            -- Edge detection: index changed from previous pixel.
            if s2_idx /= s2_prev_idx then
                s2_edge_r <= '1';
            else
                s2_edge_r <= '0';
            end if;
            s2_prev_idx <= s2_idx;

            -- S3: apply curve direction + palette shift to get palette idx.
            if reverse_r = '1' then
                s3_pidx <= (to_unsigned(7, 3) - s2_idx) +
                           pal_shift_r(9 downto 7);
            else
                s3_pidx <= s2_idx + pal_shift_r(9 downto 7);
            end if;
            s3_y    <= s2_y;
            s3_y_in <= s2_y_in;
            s3_u    <= s2_u;
            s3_v    <= s2_v;
            s3_edge <= s2_edge_r;

            -- S4: palette lookup.
            v_pidx := to_integer(s3_pidx);
            if heatmap_r = '1' then
                s4_y <= C_PY_HEAT(v_pidx);
                s4_u <= C_PU_HEAT(v_pidx);
                s4_v <= C_PV_HEAT(v_pidx);
            else
                s4_y <= C_PY_TOPO(v_pidx);
                s4_u <= C_PU_TOPO(v_pidx);
                s4_v <= C_PV_TOPO(v_pidx);
            end if;
            s4_y_in <= s3_y_in;
            s4_u_in <= s3_u;
            s4_v_in <= s3_v;
            s4_edge <= s3_edge;

            -- S5: Y mode mux + chroma mix raw products.
            if y_input_r = '1' then
                s5_y <= s4_y_in;
            else
                s5_y <= s4_y;
            end if;
            -- Chroma mix: chroma_mix_r picks between palette chroma (high
            -- chroma_mix → palette colour dominates) and input chroma.
            s5_pal_u_p <= chroma_mix_r * s4_u;
            s5_pal_v_p <= chroma_mix_r * s4_v;
            s5_inv_u_p <= (to_unsigned(1023, 10) - chroma_mix_r) * s4_u_in;
            s5_inv_v_p <= (to_unsigned(1023, 10) - chroma_mix_r) * s4_v_in;
            s5_edge    <= s4_edge;

            -- S6: chroma mix output.
            s6_y <= s5_y;
            v_sum_u := resize(s5_pal_u_p, 21) + resize(s5_inv_u_p, 21);
            v_sum_v := resize(s5_pal_v_p, 21) + resize(s5_inv_v_p, 21);
            s6_u <= v_sum_u(19 downto 10);
            s6_v <= v_sum_v(19 downto 10);
            s6_edge <= s5_edge;

            -- S7: brightness multiply raw.
            s7_y_mul <= s6_y * bright_r;
            s7_u     <= s6_u;
            s7_v     <= s6_v;
            s7_edge  <= s6_edge;

            -- S8: extract Y; sat multiply.
            s8_y <= s7_y_mul(19 downto 10);
            v_u_c := signed('0' & std_logic_vector(s7_u)) - to_signed(512, 12);
            v_v_c := signed('0' & std_logic_vector(s7_v)) - to_signed(512, 12);
            s8_u_mul <= v_u_c * signed('0' & std_logic_vector(sat_r));
            s8_v_mul <= v_v_c * signed('0' & std_logic_vector(sat_r));
            s8_edge  <= s7_edge;

            -- S9: re-center + contour overlay.
            v_u_re := resize(shift_right(s8_u_mul, 10), 14) +
                      to_signed(512, 14);
            v_v_re := resize(shift_right(s8_v_mul, 10), 14) +
                      to_signed(512, 14);
            if contour_on_r = '1' and s8_edge = '1' then
                -- Contour: lift Y toward white by contour amount.
                v_y_out := ('0' & s8_y) + ('0' & contour_r);
                if v_y_out > 1023 then
                    s9_y <= to_unsigned(1023, 10);
                else
                    s9_y <= v_y_out(9 downto 0);
                end if;
                -- Drain chroma for a cleaner edge.
                s9_u <= to_unsigned(512, 10);
                s9_v <= to_unsigned(512, 10);
            else
                s9_y <= s8_y;
                s9_u <= sat10_s(v_u_re);
                s9_v <= sat10_s(v_v_re);
            end if;

            -- S10: invert + final.
            if invert_r = '1' then
                s10_y <= to_unsigned(1023, 10) - s9_y;
            else
                s10_y <= s9_y;
            end if;
            s10_u <= s9_u;
            s10_v <= s9_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s10_y);
    data_out.u       <= std_logic_vector(s10_u);
    data_out.v       <= std_logic_vector(s10_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture bandcrush;
