-- orbifold.vhd  (v0.1 — iterated fract-ring rosette fractal)
--
-- The whole image is two shader lines iterated:
--     uv = fract(uv * divisions);
--     if (t < length(uv) && length(uv) < t + w)  break;   // draw ring, stop
-- Rings around every lattice corner, folded across a few scales, make the
-- interlocking scallop / quatrefoil rosettes.
--
-- Hardware form (streaming, no BRAM, no line buffer):
--   * divisions is locked to 2 per level, so every fract(uv*2) is a BIT-SLICE
--     of one base coordinate — NLEV=6 depth levels come for free as six
--     8-bit slices of the same 17-bit zoomed coordinate.
--   * only length() costs logic: per level r2 = x^2 + y^2 with the squares
--     decomposed as (16h+l)^2 = (h^2 & l^2) + 32*h*l — two 16-entry LUTs and
--     one 4x4 multiply each, split over two pipeline stages.
--   * the ring test compares r2 against t^2 and (t+w)^2, both computed once
--     per frame by a 10-cycle MSB-first shift-add serial squarer (off the
--     per-pixel critical path).
--   * the shallowest level that hits wins (= the shader's break); its depth
--     index + a screen-scale gradient drive a triangle-wave hue wheel.
--   * continuous self-similar zoom: one 13x9 multiply per axis, pipelined as
--     three 13x3 partial products (one op per stage).  When the zoom phase
--     wraps, the pattern maps onto itself one level deeper; the hue base is
--     stepped by one level's hue offset at the wrap so colours stay seamless.
--
-- Controls: K1 Scale (base cell 512/256/128/64 px), K2 Radius, K3 Thickness,
-- K4 Depth (levels 1..6), K5 Hue, K6 Zoom speed, S7 ring centres
-- Corners/Middles, S8 Rings/Discs, S9 background Black/Video, S10 shape
-- Circles/Squares (L2 vs Chebyshev rings), S11 palette Rainbow/Phosphor,
-- P12 manual Zoom (KEY): 8 octaves via a barrel-shift octave stage stacked
-- on the fractional zoom multiplier, so it composes with the K6 auto-zoom.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture orbifold of program_top is

    constant LATENCY : natural := 12;   -- stage regs s0..s10 (incl s3b); s_io pairs pipe(LATENCY-1)
    constant NLEV    : natural := 6;    -- fract depth levels

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- per-level hue offset (256/6 apart); the zoom wrap compensation adds
    -- HUE_STEP so a ring keeps its colour when it changes generating level.
    constant HUE_STEP : natural := 43;
    type t_hofs is array (0 to NLEV - 1) of unsigned(7 downto 0);
    constant HUE_OFS : t_hofs := (
        to_unsigned(  0, 8), to_unsigned( 43, 8), to_unsigned( 86, 8),
        to_unsigned(128, 8), to_unsigned(171, 8), to_unsigned(213, 8));

    -- 4-bit squares (n*n for n = 0..15)
    type t_sq4 is array (0 to 15) of unsigned(7 downto 0);
    constant SQ4 : t_sq4 := (
        to_unsigned(  0, 8), to_unsigned(  1, 8), to_unsigned(  4, 8), to_unsigned(  9, 8),
        to_unsigned( 16, 8), to_unsigned( 25, 8), to_unsigned( 36, 8), to_unsigned( 49, 8),
        to_unsigned( 64, 8), to_unsigned( 81, 8), to_unsigned(100, 8), to_unsigned(121, 8),
        to_unsigned(144, 8), to_unsigned(169, 8), to_unsigned(196, 8), to_unsigned(225, 8));

    -- triangle wave 0..254 over an 8-bit phase (cheap hue wheel)
    function tri8(a : unsigned(7 downto 0)) return unsigned is
    begin
        if a(7) = '0' then return a(6 downto 0) & '0';
        else               return (not a(6 downto 0)) & '0';
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Position counters + measured active area (centre of zoom)
    --------------------------------------------------------------------------
    signal pixel_x, pixel_y : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n     : std_logic := '1';
    signal prev_vsync_n     : std_logic := '1';
    signal h_cnt, v_cnt     : unsigned(11 downto 0) := (others => '0');
    signal line_had_avid    : std_logic := '0';
    signal measured_h       : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal measured_v       : unsigned(11 downto 0) := to_unsigned(1080, 12);

    --------------------------------------------------------------------------
    -- Per-frame parameters
    --------------------------------------------------------------------------
    signal s_scale    : integer range 6 to 9 := 8;            -- log2(base cell px)
    signal s_dmax     : integer range 0 to NLEV - 1 := NLEV - 1;
    signal s_center   : std_logic := '0';                     -- S7 fold to cell centres
    signal s_fill     : std_logic := '0';                     -- S8 discs
    signal s_bgvid    : std_logic := '0';                     -- S9 video background
    signal s_shape    : std_logic := '0';                     -- S10 squares (Chebyshev)
    signal s_mono     : std_logic := '0';                     -- S11 phosphor palette
    signal s_cx, s_cy : unsigned(11 downto 0) := (others => '0');
    signal s_hue_dyn  : unsigned(7 downto 0) := (others => '0');   -- K5 + wrap comp
    signal s_wrap_acc : unsigned(7 downto 0) := (others => '0');

    -- zoom: total exponent E (Q4.10 octaves) = P12 base + free-running K6 phase;
    -- integer part -> barrel-shift octave, fraction -> m = 256 + frac (Q1.8)
    signal s_zphase   : unsigned(17 downto 0) := (others => '0');
    signal s_zoom_m   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_oct      : integer range 0 to 7 := 0;

    -- serial squarer: t2lo = R^2, t2hi = (R+w)^2, refreshed each vsync
    signal sq_a, sq_b     : unsigned(9 downto 0) := (others => '0');
    signal sq_acc_a       : unsigned(19 downto 0) := (others => '0');
    signal sq_acc_b       : unsigned(19 downto 0) := (others => '0');
    signal sq_cnt         : integer range 0 to 9 := 0;
    signal sq_busy        : std_logic := '0';
    signal s_t2lo : unsigned(19 downto 0) := to_unsigned(19600, 20);
    signal s_t2hi : unsigned(19 downto 0) := to_unsigned(45000, 20);

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    type t_q8  is array (0 to NLEV - 1) of unsigned(7 downto 0);
    type t_q16 is array (0 to NLEV - 1) of unsigned(15 downto 0);
    type t_q17 is array (0 to NLEV - 1) of unsigned(16 downto 0);

    -- S0: centred coords
    signal s0_dx, s0_dy : signed(12 downto 0) := (others => '0');

    -- S1: zoom partial products (three 3-bit chunks of m per axis)
    signal s1_px0, s1_px1, s1_px2 : signed(16 downto 0) := (others => '0');
    signal s1_py0, s1_py1, s1_py2 : signed(16 downto 0) := (others => '0');

    -- S2: first sum + pp2 passthrough
    signal s2_ux, s2_uy           : signed(20 downto 0) := (others => '0');
    signal s2_px2, s2_py2         : signed(16 downto 0) := (others => '0');

    -- S3: zoomed base coords (mod 2^17 — the coarsest cell period)
    signal s3_u, s3_v : unsigned(16 downto 0) := (others => '0');

    -- S3b: octave-shifted coords (P12 manual zoom)
    signal s3b_u, s3b_v : unsigned(16 downto 0) := (others => '0');

    -- S4: per-level 8-bit fract slices (after corner/centre fold) + gradient
    signal s4_qx, s4_qy : t_q8 := (others => (others => '0'));
    signal s4_g         : unsigned(8 downto 0) := (others => '0');

    -- S5: squarer stage A: h^2 & l^2 concat + h*l partial
    signal s5_ccx, s5_ccy : t_q16 := (others => (others => '0'));
    signal s5_hlx, s5_hly : t_q8  := (others => (others => '0'));
    signal s5_g           : unsigned(8 downto 0) := (others => '0');

    -- S6: squares
    signal s6_x2, s6_y2 : t_q16 := (others => (others => '0'));
    signal s6_g         : unsigned(8 downto 0) := (others => '0');

    -- S7: radii^2
    signal s7_r2 : t_q17 := (others => (others => '0'));
    signal s7_g  : unsigned(8 downto 0) := (others => '0');

    -- S8: ring hits
    signal s8_hit : std_logic_vector(0 to NLEV - 1) := (others => '0');
    signal s8_g   : unsigned(8 downto 0) := (others => '0');

    -- S9: winner + hue
    signal s9_hit : std_logic := '0';
    signal s9_sel : integer range 0 to NLEV - 1 := 0;
    signal s9_hue : unsigned(7 downto 0) := (others => '0');

    -- S10: colour
    signal s10_hit           : std_logic := '0';
    signal s10_y, s10_u, s10_v : unsigned(9 downto 0) := (others => '0');

    -- output
    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Position counters, active-area measurement, per-frame parameter latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_d6               : unsigned(12 downto 0);
        variable v_zb, v_e          : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- pixel/line counters (active lines only, so the centre is right)
            if v_h_edge = '1' then
                pixel_x <= (others => '0');
                if line_had_avid = '1' then pixel_y <= pixel_y + 1; end if;
                line_had_avid <= '0';
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
                line_had_avid <= '1';
            end if;

            -- measure active width/height (centre of the zoom)
            if v_h_edge = '1' then
                if h_cnt > 0 then measured_h <= h_cnt; end if;
                h_cnt <= (others => '0');
                if line_had_avid = '1' then v_cnt <= v_cnt + 1; end if;
            elsif data_in.avid = '1' then
                h_cnt <= h_cnt + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                if v_cnt > 0 then measured_v <= v_cnt; end if;
                v_cnt <= (others => '0');

                s_cx <= '0' & measured_h(11 downto 1);
                s_cy <= '0' & measured_v(11 downto 1);

                -- K1 Scale: base cell 512/256/128/64 px
                case to_integer(unsigned(registers_in(0)(9 downto 8))) is
                    when 0      => s_scale <= 9;
                    when 1      => s_scale <= 8;
                    when 2      => s_scale <= 7;
                    when others => s_scale <= 6;
                end case;

                s_center <= registers_in(6)(0);
                s_fill   <= registers_in(6)(1);
                s_bgvid  <= registers_in(6)(2);
                s_shape  <= registers_in(6)(3);
                s_mono   <= registers_in(6)(4);

                -- K4 Depth: 1..6 levels (dmax = K4*6/1024)
                v_d6 := shift_left(resize(unsigned(registers_in(3)), 13), 2)
                      + shift_left(resize(unsigned(registers_in(3)), 13), 1);
                s_dmax <= to_integer(v_d6(12 downto 10));

                -- zoom phase: decrement = zoom in; on wrap the pattern maps
                -- onto itself one level deeper -> step the hue base one level
                -- so ring colours are seamless across the wrap.
                if unsigned(registers_in(5)) /= 0 then
                    if s_zphase < resize(unsigned(registers_in(5)), 18) then
                        s_wrap_acc <= s_wrap_acc + to_unsigned(HUE_STEP, 8);
                    end if;
                    s_zphase <= s_zphase - resize(unsigned(registers_in(5)), 18);
                end if;

                -- total zoom exponent (Q4.10 octaves): P12 gives 0..3.25
                -- octaves of manual zoom-out from the K1 scale (x3.25 =
                -- x3 + x1/4), the K6 phase rides on top.  Integer part ->
                -- S3b barrel shift, fraction -> mantissa.
                v_zb := resize(to_unsigned(1023, 10)
                               - unsigned(registers_in(7)), 14);
                v_zb := shift_left(v_zb, 1) + v_zb + shift_right(v_zb, 2);
                v_e  := v_zb + resize(s_zphase(17 downto 8), 14);
                if v_e(13) = '1' then                       -- >= 8 octaves: hold
                    s_oct    <= 7;
                    s_zoom_m <= to_unsigned(511, 9);
                else
                    s_oct    <= to_integer(v_e(12 downto 10));
                    s_zoom_m <= '1' & v_e(9 downto 2);
                end if;

                s_hue_dyn <= unsigned(registers_in(4)(9 downto 2)) + s_wrap_acc;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Serial squarer: t2lo = R^2, t2hi = (R+w)^2 (MSB-first shift-add,
    -- 10 cycles right after vsync — long before active video).
    --------------------------------------------------------------------------
    p_thresh : process(clk)
        variable v_r  : unsigned(9 downto 0);
        variable v_w  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                v_r := '0' & unsigned(registers_in(1)(9 downto 1));            -- K2/2: 0..511
                v_w := resize(unsigned(registers_in(2)(9 downto 2)), 10) + 2;  -- K3/4+2: 2..257
                sq_a    <= v_r;
                sq_b    <= v_r + v_w;
                sq_acc_a <= (others => '0');
                sq_acc_b <= (others => '0');
                sq_cnt  <= 0;
                sq_busy <= '1';
            elsif sq_busy = '1' then
                if sq_a(9 - sq_cnt) = '1' then
                    sq_acc_a <= shift_left(sq_acc_a, 1) + resize(sq_a, 20);
                else
                    sq_acc_a <= shift_left(sq_acc_a, 1);
                end if;
                if sq_b(9 - sq_cnt) = '1' then
                    sq_acc_b <= shift_left(sq_acc_b, 1) + resize(sq_b, 20);
                else
                    sq_acc_b <= shift_left(sq_acc_b, 1);
                end if;
                if sq_cnt = 9 then
                    sq_busy <= '0';
                else
                    sq_cnt <= sq_cnt + 1;
                end if;
            else
                s_t2lo <= sq_acc_a;
                s_t2hi <= sq_acc_b;
            end if;
        end if;
    end process p_thresh;

    --------------------------------------------------------------------------
    -- Main per-pixel pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_u21      : signed(20 downto 0);
        variable v_q        : unsigned(7 downto 0);
        variable v_f        : unsigned(7 downto 0);
        variable v_hit      : std_logic;
        variable v_sel      : integer range 0 to NLEV - 1;
        variable v_grad     : unsigned(7 downto 0);
        variable v_hue      : unsigned(7 downto 0);
        variable v_tu, v_tv : unsigned(7 downto 0);
        variable v_du, v_dv : signed(10 downto 0);
        variable v_c11      : signed(10 downto 0);
        variable v_lo, v_hi : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- S0: centred pixel coords
            s0_dx <= signed(resize(pixel_x, 13)) - signed(resize(s_cx, 13));
            s0_dy <= signed(resize(pixel_y, 13)) - signed(resize(s_cy, 13));

            -- S1: zoom multiply, three 13x3 partial products per axis
            s1_px0 <= s0_dx * signed('0' & s_zoom_m(2 downto 0));
            s1_px1 <= s0_dx * signed('0' & s_zoom_m(5 downto 3));
            s1_px2 <= s0_dx * signed('0' & s_zoom_m(8 downto 6));
            s1_py0 <= s0_dy * signed('0' & s_zoom_m(2 downto 0));
            s1_py1 <= s0_dy * signed('0' & s_zoom_m(5 downto 3));
            s1_py2 <= s0_dy * signed('0' & s_zoom_m(8 downto 6));

            -- S2: pp0 + pp1<<3
            s2_ux  <= resize(s1_px0, 21) + shift_left(resize(s1_px1, 21), 3);
            s2_uy  <= resize(s1_py0, 21) + shift_left(resize(s1_py1, 21), 3);
            s2_px2 <= s1_px2;
            s2_py2 <= s1_py2;

            -- S3: + pp2<<6, keep low 17 bits (coords tile at the coarsest cell)
            v_u21 := s2_ux + shift_left(resize(s2_px2, 21), 6);
            s3_u  <= unsigned(v_u21(16 downto 0));
            v_u21 := s2_uy + shift_left(resize(s2_py2, 21), 6);
            s3_v  <= unsigned(v_u21(16 downto 0));

            -- S3b: P12 octave zoom — left barrel shift mod 2^17
            s3b_u <= shift_left(s3_u, s_oct);
            s3b_v <= shift_left(s3_v, s_oct);

            -- S4: per-level fract slices (fract(uv*2^i) = fixed bit-slice) +
            -- optional fold to distance-from-cell-centre; screen-scale gradient.
            for i in 0 to NLEV - 1 loop
                case s_scale is
                    when 6      => v_q := s3b_u(13 - i downto 6 - i);
                    when 7      => v_q := s3b_u(14 - i downto 7 - i);
                    when 8      => v_q := s3b_u(15 - i downto 8 - i);
                    when others => v_q := s3b_u(16 - i downto 9 - i);
                end case;
                if s_center = '1' then
                    if v_q(7) = '1' then v_f := v_q - to_unsigned(128, 8);
                    else                 v_f := to_unsigned(128, 8) - v_q; end if;
                    if v_f(7) = '1' then v_q := (others => '1');       -- 128 -> 255
                    else                 v_q := v_f(6 downto 0) & '0'; end if;
                end if;
                s4_qx(i) <= v_q;

                case s_scale is
                    when 6      => v_q := s3b_v(13 - i downto 6 - i);
                    when 7      => v_q := s3b_v(14 - i downto 7 - i);
                    when 8      => v_q := s3b_v(15 - i downto 8 - i);
                    when others => v_q := s3b_v(16 - i downto 9 - i);
                end case;
                if s_center = '1' then
                    if v_q(7) = '1' then v_f := v_q - to_unsigned(128, 8);
                    else                 v_f := to_unsigned(128, 8) - v_q; end if;
                    if v_f(7) = '1' then v_q := (others => '1');
                    else                 v_q := v_f(6 downto 0) & '0'; end if;
                end if;
                s4_qy(i) <= v_q;
            end loop;
            s4_g <= resize(s3b_u(16 downto 9), 9) + resize(s3b_v(16 downto 9), 9);

            -- S5: squarer stage A: q = 16h+l -> h^2&l^2 (concat, l^2<256) + h*l
            for i in 0 to NLEV - 1 loop
                s5_ccx(i) <= SQ4(to_integer(s4_qx(i)(7 downto 4)))
                           & SQ4(to_integer(s4_qx(i)(3 downto 0)));
                s5_hlx(i) <= s4_qx(i)(7 downto 4) * s4_qx(i)(3 downto 0);
                s5_ccy(i) <= SQ4(to_integer(s4_qy(i)(7 downto 4)))
                           & SQ4(to_integer(s4_qy(i)(3 downto 0)));
                s5_hly(i) <= s4_qy(i)(7 downto 4) * s4_qy(i)(3 downto 0);
            end loop;
            s5_g <= s4_g;

            -- S6: squarer stage B: q^2 = (h^2 & l^2) + (h*l << 5)
            for i in 0 to NLEV - 1 loop
                s6_x2(i) <= s5_ccx(i) + shift_left(resize(s5_hlx(i), 16), 5);
                s6_y2(i) <= s5_ccy(i) + shift_left(resize(s5_hly(i), 16), 5);
            end loop;
            s6_g <= s5_g;

            -- S7: r^2 per level — L2 circles, or Chebyshev squares
            -- (max(|x|,|y|)^2 = max(x^2, y^2), so squares are one mux)
            for i in 0 to NLEV - 1 loop
                if s_shape = '1' then
                    if s6_x2(i) >= s6_y2(i) then
                        s7_r2(i) <= resize(s6_x2(i), 17);
                    else
                        s7_r2(i) <= resize(s6_y2(i), 17);
                    end if;
                else
                    s7_r2(i) <= resize(s6_x2(i), 17) + resize(s6_y2(i), 17);
                end if;
            end loop;
            s7_g <= s6_g;

            -- S8: ring test per enabled level (Discs drop the inner bound).
            -- Chebyshev distance tops out at the edge (256) not the corner
            -- (362), so squares mode halves the r^2 band (= radius / sqrt2)
            -- to keep the Radius knob's sweet spot in the same place.
            if s_shape = '1' then
                v_lo := '0' & s_t2lo(19 downto 1);
                v_hi := '0' & s_t2hi(19 downto 1);
            else
                v_lo := s_t2lo;
                v_hi := s_t2hi;
            end if;
            for i in 0 to NLEV - 1 loop
                if (i <= s_dmax)
                   and (resize(s7_r2(i), 20) < v_hi)
                   and (s_fill = '1' or resize(s7_r2(i), 20) >= v_lo) then
                    s8_hit(i) <= '1';
                else
                    s8_hit(i) <= '0';
                end if;
            end loop;
            s8_g <= s7_g;

            -- S9: shallowest hit wins (= shader break) + hue
            v_hit := '0';
            v_sel := 0;
            for i in NLEV - 1 downto 0 loop
                if s8_hit(i) = '1' then v_sel := i; v_hit := '1'; end if;
            end loop;
            v_grad := resize(s8_g(8 downto 2), 8);   -- fixed medium hue spread
            s9_hit <= v_hit;
            s9_sel <= v_sel;
            s9_hue <= s_hue_dyn + HUE_OFS(v_sel) + v_grad;

            -- S10: hue -> YUV via triangle wheel (U/V in quadrature, gain 3)
            v_tu := tri8(s9_hue);
            v_tv := tri8(s9_hue + to_unsigned(64, 8));
            v_du := signed(resize(v_tu, 11)) - to_signed(128, 11);
            v_dv := signed(resize(v_tv, 11)) - to_signed(128, 11);
            if s_mono = '1' then
                -- phosphor: green, luma rides the hue gradient
                s10_y <= to_unsigned(256, 10) + shift_left(resize(v_tu, 10), 1);
                s10_u <= to_unsigned(300, 10);
                s10_v <= to_unsigned(280, 10);
            else
                s10_y <= to_unsigned(672, 10) - to_unsigned(16 * s9_sel, 10);
                -- 512 + 3*(tri-128): range 128..890, always positive -> slice
                v_c11 := to_signed(512, 11) + v_du + shift_left(v_du, 1);
                s10_u <= unsigned(v_c11(9 downto 0));
                v_c11 := to_signed(512, 11) + v_dv + shift_left(v_dv, 1);
                s10_v <= unsigned(v_c11(9 downto 0));
            end if;
            s10_hit <= s9_hit;

            -- S_out: ring colour, else background (black or delayed input video)
            if s10_hit = '1' then
                s_io.y <= std_logic_vector(s10_y);
                s_io.u <= std_logic_vector(s10_u);
                s_io.v <= std_logic_vector(s10_v);
            elsif s_bgvid = '1' then
                s_io.y <= pipe(LATENCY - 1).y;
                s_io.u <= pipe(LATENCY - 1).u;
                s_io.v <= pipe(LATENCY - 1).v;
            else
                s_io.y <= (others => '0');
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture orbifold;
