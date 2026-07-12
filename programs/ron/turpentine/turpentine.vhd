-- turpentine.vhd
--
-- TURPENTINE v2 -- chroma as pigment: the picture repainted in oils.
--
-- v1 (chroma-directed feedback smear) read as subtle mush; scrapped.
-- v2 keeps the fiction -- chroma is paint -- but goes bold: every control
-- makes an immediately legible change to the image.
--
-- Chroma is a 2D vector. v2 reads it as PIGMENT IDENTITY and LOAD:
--   hue angle  -> which pigment from a limited palette (quantized sectors)
--   saturation -> how much paint is loaded there (quantized rings; below
--                 a floor = bare paper)
-- The image is re-rendered as a painting: flat pigment regions, tonal
-- bands, brush strokes oriented along each region's hue, thick-paint
-- relief lit by a movable light, dark outlines at pigment boundaries.
--
-- P12 "Solvent" = master dry video -> full painting crossfade (0 = exact
-- dry by construction); past ~80% a VARNISH zone deepens the result
-- (outline boost + stronger relief).
--
-- Controls (all deliberately high-impact):
--   K1 Pigments  32/16/8/4/2 hue sectors (index bit-mask -- the picture
--                collapses into fewer and fewer colors)
--   K2 Relief    impasto depth (flat poster -> deeply embossed paint)
--   K3 Light     light direction, highlights sweep (S11 = auto orbit)
--   K4 Stroke    brush texture amplitude + pitch, carved into the relief,
--                oriented along each region's hue octant
--   K5 Wash      oil -> watercolour (pigment thins, luma lifts to paper)
--   K6 Outline   dark contour lines at pigment boundaries (paint-by-number)
--   S7 Palette   Free (original saturation, quantized) / Vivid (full-load
--                pigments -- bold false-colour poster)
--   S8 Canvas    paper tooth in the relief
--   S9 Tone      luma posterized into wobbled tonal bands
--   S10 Ink      outline colour Black / Chalk
--   S11 Orbit    light auto-rotation (keeps static input alive)
--
-- Architecture: single streaming pipeline E1..E13 + 4-clock interpolator
-- dry/wet mix, C_LATENCY = 17.
--   * Hue sectors WITHOUT atan2/CORDIC ROMs: fold (cu,cv) to the first
--     octant (signs + swap = 3 bits), then two tangent threshold tests
--     (tan 22.5/11.25/33.75 as shift-add sums of the major component,
--     all three compares in parallel) = 2 more bits -> 5-bit sector
--     index, 32 uniform sectors. Odd octants bit-invert the sub-index.
--   * Palette reconstruction: sector-center angle -> the shared 64-entry
--     quarter-wave qsin (angle registered E6, ROM outputs registered E7,
--     mercurial rule), then 2 multiplies sat_q x sin/cos. K1 masks the
--     index and re-centers the angle, so fewer pigments = wider sectors.
--   * Relief: paint thickness T = pigment-load base + tonal-band term +
--     hue-oriented triangle strokes + canvas hash. One packed row buffer
--     {T8, idx5, gray} 2048x14 (8 EBR, canonical 1W1R, write lagging read
--     = previous-row values, redshift single-bank trick). gx from x-delay
--     registers, gy vs the row above; shade = (gx*lx + gy*ly) >>> rsh
--     (2 multiplies); glint highlight above a threshold.
--   * Outlines: masked pigment index compared against left and above
--     neighbours (index rides the row buffer), 1-px dilate, shift-blend
--     toward ink (black/chalk) in 4 K6 steps.
--   * Dry/wet: 3x interpolator_u on a Solvent ramp (shearline pattern).
--
-- Multiplies (7): 2 palette (u8 x s8), 2 shading (s9 x s8), 3 in the
-- interpolators. Everything else is shift-add. EBR: 8 of 32.
--
-- Timing discipline (house rules): multiplies alone in their stage with
-- registered inputs; BRAM reads land in FFs; qsin angle and output
-- registered; per-frame terms built one add per sequencer step; the
-- E5 sector compares are all parallel against E4-registered thresholds.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture turpentine of program_top is

    constant C_LATENCY : integer := 17;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu8(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 8);
        elsif v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(unsigned(v), 8);
        end if;
    end function;

    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- |s10| -> u9 (clamps -512 to 511)
    function f_abs9(v : signed(9 downto 0)) return unsigned is
        variable v11 : signed(10 downto 0);
    begin
        v11 := resize(v, 11);
        if v11 < 0 then
            v11 := -v11;
        end if;
        if v11 > 511 then
            v11 := to_signed(511, 11);
        end if;
        return resize(unsigned(v11(8 downto 0)), 9);
    end function;

    -- quarter-wave folded sine, 64-entry logic ROM (fireworks/mercurial)
    type t_qsin is array(0 to 63) of signed(9 downto 0);
    constant C_QSIN : t_qsin := (
        to_signed(  0, 10), to_signed( 13, 10), to_signed( 25, 10), to_signed( 38, 10), to_signed( 50, 10), to_signed( 63, 10), to_signed( 75, 10), to_signed( 87, 10),
        to_signed(100, 10), to_signed(112, 10), to_signed(124, 10), to_signed(136, 10), to_signed(148, 10), to_signed(160, 10), to_signed(172, 10), to_signed(184, 10),
        to_signed(196, 10), to_signed(207, 10), to_signed(218, 10), to_signed(230, 10), to_signed(241, 10), to_signed(252, 10), to_signed(263, 10), to_signed(273, 10),
        to_signed(284, 10), to_signed(294, 10), to_signed(304, 10), to_signed(314, 10), to_signed(324, 10), to_signed(334, 10), to_signed(343, 10), to_signed(352, 10),
        to_signed(361, 10), to_signed(370, 10), to_signed(379, 10), to_signed(387, 10), to_signed(395, 10), to_signed(403, 10), to_signed(410, 10), to_signed(418, 10),
        to_signed(425, 10), to_signed(432, 10), to_signed(438, 10), to_signed(445, 10), to_signed(451, 10), to_signed(456, 10), to_signed(462, 10), to_signed(467, 10),
        to_signed(472, 10), to_signed(477, 10), to_signed(481, 10), to_signed(485, 10), to_signed(489, 10), to_signed(492, 10), to_signed(496, 10), to_signed(499, 10),
        to_signed(501, 10), to_signed(503, 10), to_signed(505, 10), to_signed(507, 10), to_signed(509, 10), to_signed(510, 10), to_signed(510, 10), to_signed(511, 10));

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then
            v_i := a(7 downto 2);
        else
            v_i := not a(7 downto 2);
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    -- value-noise hash, split across two pipeline stages (inferno lesson)
    function f_hash_a(cx, cy : unsigned(7 downto 0);
                      seed   : unsigned(15 downto 0)) return unsigned is
        variable h16 : unsigned(15 downto 0);
        variable h   : unsigned(11 downto 0);
    begin
        h16 := ((cx & cy)) xor seed;
        h16 := h16 xor (x"0" & h16(15 downto 4));
        h   := h16(11 downto 0);
        return h + rotate_left(h, 5);
    end function;

    function f_hash_b(hin : unsigned(11 downto 0)) return unsigned is
        variable h : unsigned(11 downto 0);
    begin
        h := hin xor rotate_left(hin, 9);
        h := h + rotate_left(h, 3);
        h := h xor ("00000" & h(11 downto 5));
        return h(9 downto 2);
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms (vblank sequencer)
    ----------------------------------------------------------------------
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_k1  : unsigned(9 downto 0) := (others => '0');
    signal s_k2  : unsigned(9 downto 0) := (others => '0');
    signal s_k4  : unsigned(9 downto 0) := (others => '0');
    signal s_k5  : unsigned(9 downto 0) := (others => '0');
    signal s_k6  : unsigned(9 downto 0) := (others => '0');
    signal s_vivid   : std_logic := '0';                       -- S7
    signal s_canv_on : std_logic := '1';                       -- S8
    signal s_tone_on : std_logic := '1';                       -- S9
    signal s_chalk   : std_logic := '0';                       -- S10
    signal s_orbit   : std_logic := '0';                       -- S11

    signal s_lang      : unsigned(9 downto 0) := (others => '0');
    signal s_orbit_acc : unsigned(9 downto 0) := (others => '0');
    signal s_lx8, s_ly8 : signed(7 downto 0) := to_signed(90, 8);

    signal s_mix10  : unsigned(9 downto 0) := (others => '0'); -- solvent
    signal s_varn   : std_logic := '0';
    signal s_mask5  : unsigned(4 downto 0) := (others => '1');
    signal s_half10 : unsigned(9 downto 0) := to_unsigned(16, 10);
    signal s_rsh    : natural range 3 to 7 := 5;               -- relief
    signal s_amp_on : std_logic := '1';                        -- stroke
    signal s_ash    : natural range 0 to 2 := 1;
    signal s_psh    : natural range 1 to 3 := 2;
    signal s_liftsh : natural range 0 to 3 := 0;               -- wash lift
    signal s_ol_lvl : unsigned(1 downto 0) := (others => '0'); -- outline
    signal s_ink_y  : unsigned(9 downto 0) := to_unsigned(96, 10);
    -- per-frame pigment-load LUTs (wash/vivid folded in by the sequencer)
    signal s_sq1, s_sq2, s_sq3 : unsigned(7 downto 0) := (others => '0');

    signal s_seq   : unsigned(3 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');
    signal s_qs_r  : signed(9 downto 0) := (others => '0');

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';

    -- saturation ring thresholds (sat9 scale, 0..766)
    constant C_GRAY : unsigned(9 downto 0) := to_unsigned(56, 10);
    constant C_RLO  : unsigned(9 downto 0) := to_unsigned(160, 10);
    constant C_RHI  : unsigned(9 downto 0) := to_unsigned(320, 10);
    -- pigment-load thickness per ring (paper, r1, r2, r3)
    constant C_T0 : unsigned(7 downto 0) := to_unsigned(24, 8);
    constant C_T1 : unsigned(7 downto 0) := to_unsigned(64, 8);
    constant C_T2 : unsigned(7 downto 0) := to_unsigned(104, 8);
    constant C_T3 : unsigned(7 downto 0) := to_unsigned(152, 8);

    ----------------------------------------------------------------------
    -- position / line bookkeeping
    ----------------------------------------------------------------------
    signal s_x_count : unsigned(10 downto 0) := (others => '0');
    signal s_aline   : unsigned(9 downto 0) := (others => '0');
    signal s_seen    : std_logic := '0';
    signal s_ln0     : std_logic := '1';

    ----------------------------------------------------------------------
    -- E1..E7: sector index, rings, strokes, palette angle
    ----------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal px1 : unsigned(10 downto 0) := (others => '0');

    signal au2, av2 : unsigned(8 downto 0) := (others => '0');
    signal su2, sv2 : std_logic := '0';
    signal px2 : unsigned(10 downto 0) := (others => '0');

    signal a3, b3 : unsigned(8 downto 0) := (others => '0');
    signal oct3   : unsigned(2 downto 0) := (others => '0');
    signal sat3   : unsigned(9 downto 0) := (others => '0');
    signal px3 : unsigned(10 downto 0) := (others => '0');

    signal t22_4, t11_4, t33_4 : unsigned(8 downto 0) := (others => '0');
    signal b4   : unsigned(8 downto 0) := (others => '0');
    signal oct4 : unsigned(2 downto 0) := (others => '0');
    signal sat4 : unsigned(9 downto 0) := (others => '0');
    signal px4 : unsigned(10 downto 0) := (others => '0');

    signal idx5  : unsigned(4 downto 0) := (others => '0');
    signal gray5 : std_logic := '0';
    signal ring5 : unsigned(1 downto 0) := (others => '0');
    signal px5 : unsigned(10 downto 0) := (others => '0');

    signal m6    : unsigned(4 downto 0) := (others => '0');
    signal ang6  : unsigned(9 downto 0) := (others => '0');
    signal sq6   : unsigned(7 downto 0) := (others => '0');
    signal tri6  : signed(6 downto 0) := (others => '0');
    signal ch_a6 : unsigned(11 downto 0) := (others => '0');
    signal gray6 : std_logic := '0';
    signal ring6 : unsigned(1 downto 0) := (others => '0');
    signal px6 : unsigned(10 downto 0) := (others => '0');

    signal cs7, sn7 : signed(7 downto 0) := (others => '0');
    signal sq7   : unsigned(7 downto 0) := (others => '0');
    signal tri7  : signed(6 downto 0) := (others => '0');
    signal cnv7  : signed(4 downto 0) := (others => '0');
    signal yb7   : unsigned(9 downto 0) := (others => '0');
    signal m7    : unsigned(4 downto 0) := (others => '0');
    signal gray7 : std_logic := '0';
    signal ring7 : unsigned(1 downto 0) := (others => '0');
    signal px7 : unsigned(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- E8..E13: palette mults, thickness + row buffer, shading, outline
    ----------------------------------------------------------------------
    signal pu8, pv8 : signed(16 downto 0) := (others => '0');
    signal T8    : unsigned(7 downto 0) := (others => '0');
    signal yb8   : unsigned(9 downto 0) := (others => '0');
    signal m8    : unsigned(4 downto 0) := (others => '0');
    signal gray8 : std_logic := '0';
    signal px8 : unsigned(10 downto 0) := (others => '0');

    -- packed row buffer {T8, idx5, gray} (previous line, same field)
    type t_row is array (0 to 2047) of std_logic_vector(13 downto 0);
    signal row_ram : t_row := (others => (others => '0'));
    signal row_q   : std_logic_vector(13 downto 0) := (others => '0');
    signal T_up8   : unsigned(7 downto 0) := (others => '0');
    signal iup8    : unsigned(4 downto 0) := (others => '0');
    signal gup8    : std_logic := '0';

    signal T8_d1, T8_d2 : unsigned(7 downto 0) := (others => '0');
    signal m8_d1  : unsigned(4 downto 0) := (others => '0');
    signal gray8_d1 : std_logic := '0';

    signal gx9, gy9 : signed(8 downto 0) := (others => '0');
    signal ucol9, vcol9 : unsigned(9 downto 0) := (others => '0');
    signal ol9   : std_logic := '0';
    signal yb9   : unsigned(9 downto 0) := (others => '0');
    signal ln0_9 : std_logic := '0';

    signal sx10, sy10 : signed(16 downto 0) := (others => '0');
    signal ucol10, vcol10 : unsigned(9 downto 0) := (others => '0');
    signal ol10  : std_logic := '0';
    signal ol9_d1 : std_logic := '0';
    signal yb10  : unsigned(9 downto 0) := (others => '0');

    signal shade11 : signed(8 downto 0) := (others => '0');
    signal ucol11, vcol11 : unsigned(9 downto 0) := (others => '0');
    signal ol11 : std_logic := '0';
    signal yb11 : unsigned(9 downto 0) := (others => '0');

    signal y12 : unsigned(9 downto 0) := (others => '0');
    signal u12, v12 : unsigned(9 downto 0) := (others => '0');
    signal ol12 : std_logic := '0';

    signal wy13, wu13, wv13 : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- dry / sync delay + interpolators
    ----------------------------------------------------------------------
    type t_data_sr is array (0 to 12) of std_logic_vector(9 downto 0);
    signal s_y_sr, s_u_sr, s_v_sr : t_data_sr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

    signal s_iy_a, s_iu_a, s_iv_a : unsigned(9 downto 0);
    signal s_iy_r, s_iu_r, s_iv_r : unsigned(9 downto 0);
    signal s_iy_v, s_iu_v, s_iv_v : std_logic;

begin

    ------------------------------------------------------------------------
    -- shared qsin: sequencer-only user, angle + output registered anyway
    ------------------------------------------------------------------------
    s_qs_a <= s_lang                          when s_seq = 10 else
              s_lang + to_unsigned(256, 10);

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (one add per step)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_z  : natural range 0 to 4;
        variable v_b  : unsigned(7 downto 0);
        variable v_ol : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_p12 <= unsigned(registers_in(7));
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_vivid   <= registers_in(6)(0);   -- switch 7
                s_canv_on <= registers_in(6)(1);   -- switch 8
                s_tone_on <= registers_in(6)(2);   -- switch 9
                s_chalk   <= registers_in(6)(3);   -- switch 10
                s_orbit   <= registers_in(6)(4);   -- switch 11

                s_orbit_acc <= s_orbit_acc + 2;
                if registers_in(6)(4) = '1' then
                    s_lang <= unsigned(registers_in(2)) + s_orbit_acc;
                else
                    s_lang <= unsigned(registers_in(2));
                end if;

                s_seq <= to_unsigned(10, 4);
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 10 =>
                        -- solvent ramp: full painting by ~68%, varnish
                        -- zone beyond 80% (qsin presented: light angle)
                        if s_p12 > 682 then
                            s_mix10 <= (others => '1');
                        else
                            s_mix10 <= resize(s_p12 + s_p12(9 downto 1), 10);
                        end if;
                        if s_p12 > 800 then
                            s_varn <= '1';
                        else
                            s_varn <= '0';
                        end if;
                    when 9 =>
                        -- pigment count: K1 zone -> index mask + recenter
                        -- (qsin presented: light angle + 256)
                        if s_k1(9 downto 7) > 4 then
                            v_z := 4;
                        else
                            v_z := to_integer(s_k1(9 downto 7));
                        end if;
                        case v_z is
                            when 0 => s_mask5 <= "11111";
                                      s_half10 <= to_unsigned(16, 10);
                            when 1 => s_mask5 <= "11110";
                                      s_half10 <= to_unsigned(32, 10);
                            when 2 => s_mask5 <= "11100";
                                      s_half10 <= to_unsigned(64, 10);
                            when 3 => s_mask5 <= "11000";
                                      s_half10 <= to_unsigned(128, 10);
                            when others => s_mask5 <= "10000";
                                      s_half10 <= to_unsigned(256, 10);
                        end case;
                    when 8 =>
                        -- capture two-behind: qsin(light angle) = ly
                        s_ly8 <= resize(shift_right(s_qs_r, 2), 8);
                        if s_varn = '1' and s_k2(9 downto 8) /= "00" then
                            s_rsh <= 7 - to_integer(s_k2(9 downto 8)) - 1;
                        else
                            s_rsh <= 7 - to_integer(s_k2(9 downto 8));
                        end if;
                    when 7 =>
                        -- capture: qsin(light angle + 256) = lx
                        s_lx8 <= resize(shift_right(s_qs_r, 2), 8);
                        -- stroke: amplitude zone + pitch
                        if s_k4(9 downto 8) = "00" then
                            s_amp_on <= '0';
                            s_ash    <= 0;
                        else
                            s_amp_on <= '1';
                            s_ash    <= to_integer(s_k4(9 downto 8)) - 1;
                        end if;
                        if s_k4(7 downto 6) = "00" then
                            s_psh <= 1;
                        elsif s_k4(7 downto 6) = "11" then
                            s_psh <= 3;
                        else
                            s_psh <= to_integer(s_k4(7 downto 6));
                        end if;
                    when 6 =>
                        -- wash: luma lift shift (0 = dry oil)
                        case s_k5(9 downto 8) is
                            when "00"   => s_liftsh <= 0;
                            when "01"   => s_liftsh <= 3;
                            when "10"   => s_liftsh <= 2;
                            when others => s_liftsh <= 1;
                        end case;
                    when 5 =>
                        -- pigment-load LUT ring 3 (wash + vivid folded)
                        if s_vivid = '1' then
                            v_b := to_unsigned(200, 8);
                        else
                            v_b := to_unsigned(216, 8);
                        end if;
                        case s_k5(9 downto 8) is
                            when "00"   => s_sq3 <= v_b;
                            when "01"   => s_sq3 <= v_b - ("00" & v_b(7 downto 2));
                            when "10"   => s_sq3 <= '0' & v_b(7 downto 1);
                            when others => s_sq3 <= "00" & v_b(7 downto 2);
                        end case;
                    when 4 =>
                        if s_vivid = '1' then
                            v_b := to_unsigned(200, 8);
                        else
                            v_b := to_unsigned(152, 8);
                        end if;
                        case s_k5(9 downto 8) is
                            when "00"   => s_sq2 <= v_b;
                            when "01"   => s_sq2 <= v_b - ("00" & v_b(7 downto 2));
                            when "10"   => s_sq2 <= '0' & v_b(7 downto 1);
                            when others => s_sq2 <= "00" & v_b(7 downto 2);
                        end case;
                    when 3 =>
                        if s_vivid = '1' then
                            v_b := to_unsigned(200, 8);
                        else
                            v_b := to_unsigned(88, 8);
                        end if;
                        case s_k5(9 downto 8) is
                            when "00"   => s_sq1 <= v_b;
                            when "01"   => s_sq1 <= v_b - ("00" & v_b(7 downto 2));
                            when "10"   => s_sq1 <= '0' & v_b(7 downto 1);
                            when others => s_sq1 <= "00" & v_b(7 downto 2);
                        end case;
                    when 2 =>
                        -- outline level (varnish boosts one step)
                        v_ol := resize(s_k6(9 downto 8), 3);
                        if s_varn = '1' and v_ol /= 0 then
                            v_ol := v_ol + 1;
                        end if;
                        if v_ol > 3 then
                            s_ol_lvl <= "11";
                        else
                            s_ol_lvl <= v_ol(1 downto 0);
                        end if;
                        if s_chalk = '1' then
                            s_ink_y <= to_unsigned(940, 10);
                        else
                            s_ink_y <= to_unsigned(96, 10);
                        end if;
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline E1..E13
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_cu, v_cv : signed(9 downto 0);
        variable v_swap : std_logic;
        variable v_a, v_b : unsigned(8 downto 0);
        variable v_oct  : unsigned(2 downto 0);
        variable v_c22, v_c11, v_c33 : std_logic;
        variable v_sub  : unsigned(1 downto 0);
        variable v_w12  : unsigned(11 downto 0);
        variable v_w3   : unsigned(2 downto 0);
        variable v_t2   : unsigned(1 downto 0);
        variable v_tri  : signed(6 downto 0);
        variable v_ys   : signed(11 downto 0);
        variable v_yq   : unsigned(9 downto 0);
        variable v_h8   : unsigned(7 downto 0);
        variable v_T    : unsigned(9 downto 0);
        variable v_hd, v_vd : std_logic;
        variable v_sh   : signed(17 downto 0);
        variable v_y    : signed(11 downto 0);
        variable v_lift : unsigned(9 downto 0);
        variable v_iy, v_iu, v_iv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            --------------------------------------------------------------
            -- E1: input registers + position
            --------------------------------------------------------------
            s_in_y <= unsigned(data_in.y);
            s_in_u <= unsigned(data_in.u);
            s_in_v <= unsigned(data_in.v);
            px1    <= s_x_count;
            if data_in.avid = '1' then
                s_x_count <= s_x_count + 1;
                s_seen    <= '1';
            end if;

            s_y_sr(0) <= data_in.y;
            s_u_sr(0) <= data_in.u;
            s_v_sr(0) <= data_in.v;
            for i in 1 to 12 loop
                s_y_sr(i) <= s_y_sr(i - 1);
                s_u_sr(i) <= s_u_sr(i - 1);
                s_v_sr(i) <= s_v_sr(i - 1);
            end loop;

            --------------------------------------------------------------
            -- E2: chroma vector magnitudes + signs
            --------------------------------------------------------------
            v_cu := signed((not s_in_u(9)) & s_in_u(8 downto 0));
            v_cv := signed((not s_in_v(9)) & s_in_v(8 downto 0));
            au2 <= f_abs9(v_cu);
            av2 <= f_abs9(v_cv);
            su2 <= v_cu(9);
            sv2 <= v_cv(9);
            px2 <= px1;

            --------------------------------------------------------------
            -- E3: octant fold (major/minor swap + signs) + saturation
            --------------------------------------------------------------
            v_swap := '0';
            if au2 > av2 then
                v_swap := '1';
                v_a := au2;
                v_b := av2;
            else
                v_a := av2;
                v_b := au2;
            end if;
            a3 <= v_a;
            b3 <= v_b;
            sat3 <= resize(v_a, 10) + resize(v_b(8 downto 1), 10);

            -- octant table: angle from +V axis toward +U, 8 x 45 degrees
            if sv2 = '0' and su2 = '0' then           -- +V +U quadrant
                if v_swap = '0' then v_oct := "000"; else v_oct := "001"; end if;
            elsif sv2 = '1' and su2 = '0' then        -- -V +U
                if v_swap = '1' then v_oct := "010"; else v_oct := "011"; end if;
            elsif sv2 = '1' and su2 = '1' then        -- -V -U
                if v_swap = '0' then v_oct := "100"; else v_oct := "101"; end if;
            else                                      -- +V -U
                if v_swap = '1' then v_oct := "110"; else v_oct := "111"; end if;
            end if;
            oct3 <= v_oct;
            px3  <= px2;

            --------------------------------------------------------------
            -- E4: intra-octant tangent thresholds (shift-add, parallel)
            --   tan 22.50 = 0.4142 ~ 1/2 - 1/16 - 1/32 + 1/128
            --   tan 11.25 = 0.1989 ~ 1/4 - 1/16 + 1/128
            --   tan 33.75 = 0.6682 ~ 1/2 + 1/8 + 1/32 + 1/128
            --------------------------------------------------------------
            t22_4 <= resize(a3(8 downto 1), 9) - resize(a3(8 downto 4), 9)
                     - resize(a3(8 downto 5), 9) + resize(a3(8 downto 7), 9);
            t11_4 <= resize(a3(8 downto 2), 9) - resize(a3(8 downto 4), 9)
                     + resize(a3(8 downto 7), 9);
            t33_4 <= resize(a3(8 downto 1), 9) + resize(a3(8 downto 3), 9)
                     + resize(a3(8 downto 5), 9) + resize(a3(8 downto 7), 9);
            b4   <= b3;
            oct4 <= oct3;
            sat4 <= sat3;
            px4  <= px3;

            --------------------------------------------------------------
            -- E5: sector index (all compares parallel; odd octants run
            -- backwards so the sub-index bit-inverts) + rings + gray
            --------------------------------------------------------------
            v_c22 := '0'; v_c11 := '0'; v_c33 := '0';
            if b4 > t22_4 then v_c22 := '1'; end if;
            if b4 > t11_4 then v_c11 := '1'; end if;
            if b4 > t33_4 then v_c33 := '1'; end if;
            if v_c22 = '1' then
                v_sub(1) := '1';
                v_sub(0) := v_c33;
            else
                v_sub(1) := '0';
                v_sub(0) := v_c11;
            end if;
            if oct4(0) = '1' then
                v_sub := not v_sub;
            end if;
            idx5 <= oct4 & v_sub;

            if sat4 < C_GRAY then
                gray5 <= '1';
            else
                gray5 <= '0';
            end if;
            if sat4 > C_RHI then
                ring5 <= "11";
            elsif sat4 > C_RLO then
                ring5 <= "10";
            else
                ring5 <= "01";
            end if;
            px5 <= px4;

            --------------------------------------------------------------
            -- E6: masked pigment index -> sector-center angle; pigment
            -- load from the per-frame ring LUT; hue-oriented stroke
            -- triangle; canvas hash A
            --------------------------------------------------------------
            m6   <= idx5 and s_mask5;
            ang6 <= shift_left(resize(idx5 and s_mask5, 10), 5) + s_half10;
            case ring5 is
                when "11"   => sq6 <= s_sq3;
                when "10"   => sq6 <= s_sq2;
                when others => sq6 <= s_sq1;
            end case;
            if gray5 = '1' then
                sq6 <= (others => '0');
            end if;

            -- stroke orientation from the hue quadrant: strokes rotate
            -- with the pigment (0/45/90/135 degrees)
            case idx5(4 downto 3) is
                when "00"   => v_w12 := resize(s_aline, 12);
                when "01"   => v_w12 := resize(px5, 12) + resize(s_aline, 12);
                when "10"   => v_w12 := resize(px5, 12);
                when others => v_w12 := resize(px5, 12) - resize(s_aline, 12);
            end case;
            v_w12 := shift_right(v_w12, s_psh);
            v_w3  := v_w12(2 downto 0);
            v_t2  := v_w3(1 downto 0) xor (v_w3(2) & v_w3(2));
            v_tri := resize(signed('0' & v_t2 & '0'), 7) - 3;  -- -3,-1,1,3
            if s_amp_on = '1' then
                tri6 <= shift_left(v_tri, s_ash + 1);          -- to +/-24
            else
                tri6 <= (others => '0');
            end if;

            ch_a6 <= f_hash_a(px5(9 downto 2), s_aline(9 downto 2), x"0111");
            gray6 <= gray5;
            ring6 <= ring5;
            px6   <= px5;

            --------------------------------------------------------------
            -- E7: qsin ROM outputs registered (mercurial rule); tonal
            -- band quantize (wobbled by the stroke); canvas hash B
            --------------------------------------------------------------
            cs7 <= resize(shift_right(f_qsin(ang6 + 256), 2), 8);
            sn7 <= resize(shift_right(f_qsin(ang6), 2), 8);
            sq7 <= sq6;
            tri7 <= tri6;
            v_h8 := f_hash_b(ch_a6);
            if s_canv_on = '1' then
                cnv7 <= signed(resize(v_h8(7 downto 4), 5)) - 8;
            else
                cnv7 <= (others => '0');
            end if;

            -- tonal bands: 8 wobbled luma bands (S9), else straight luma
            v_ys := signed(resize(unsigned(s_y_sr(5)), 12))
                    + resize(tri6, 12);
            v_yq := f_cu10(v_ys);
            if s_tone_on = '1' then
                yb7 <= v_yq(9 downto 7) & "1000000";
            else
                yb7 <= unsigned(s_y_sr(5));
            end if;

            m7    <= m6;
            gray7 <= gray6;
            ring7 <= ring6;
            px7   <= px6;   -- row-buffer read address (see p_row)

            --------------------------------------------------------------
            -- E8: palette multiplies (alone); paint thickness assemble;
            -- land the row-buffer read
            --------------------------------------------------------------
            pu8 <= signed('0' & sq7) * sn7;
            pv8 <= signed('0' & sq7) * cs7;

            case ring7 is
                when "11"   => v_T := resize(C_T3, 10);
                when "10"   => v_T := resize(C_T2, 10);
                when others => v_T := resize(C_T1, 10);
            end case;
            if gray7 = '1' then
                v_T := resize(C_T0, 10);
            end if;
            v_T := v_T + unsigned(resize(signed(resize(tri7, 8)) + 24, 10))
                       + unsigned(resize(signed(resize(cnv7, 8)) + 8, 10))
                       + resize(yb7(9 downto 7) & "00", 10);
            if v_T > 255 then
                T8 <= (others => '1');
            else
                T8 <= resize(v_T, 8);
            end if;

            T_up8 <= unsigned(row_q(13 downto 6));
            iup8  <= unsigned(row_q(5 downto 1));
            gup8  <= row_q(0);

            yb8   <= yb7;
            m8    <= m7;
            gray8 <= gray7;
            px8   <= px7;

            --------------------------------------------------------------
            -- E9: gradients (x from delay regs, y vs the row above),
            -- paint colour compose, outline detect (left + up neighbours)
            --------------------------------------------------------------
            T8_d1 <= T8;
            T8_d2 <= T8_d1;
            gx9 <= signed(resize(T8, 9)) - signed(resize(T8_d2, 9));
            if ln0_9 = '1' then
                gy9 <= (others => '0');
            else
                gy9 <= signed(resize(T_up8, 9)) - signed(resize(T8, 9));
            end if;
            if s_aline = 0 then
                ln0_9 <= '1';
            else
                ln0_9 <= '0';
            end if;

            ucol9 <= f_cu10(to_signed(512, 12)
                            + resize(shift_right(pu8, 7), 12));
            vcol9 <= f_cu10(to_signed(512, 12)
                            + resize(shift_right(pv8, 7), 12));

            -- pigment boundary = the (gray, masked idx) tuple changes;
            -- two gray pixels are the SAME pigment whatever their noise
            -- hue index says, else flat areas fill with outline speckle
            m8_d1    <= m8;
            gray8_d1 <= gray8;
            v_hd := '0';
            if (gray8 /= gray8_d1)
               or (gray8 = '0' and gray8_d1 = '0' and m8 /= m8_d1) then
                v_hd := '1';
            end if;
            v_vd := '0';
            if ln0_9 = '0'
               and ((gray8 /= gup8)
                    or (gray8 = '0' and gup8 = '0'
                        and m8 /= (iup8 and s_mask5))) then
                v_vd := '1';
            end if;
            ol9 <= v_hd or v_vd;

            yb9 <= yb8;

            --------------------------------------------------------------
            -- E10: shading multiplies (alone); outline 1-px dilate
            --------------------------------------------------------------
            sx10 <= gx9 * s_lx8;
            sy10 <= gy9 * s_ly8;
            ol9_d1 <= ol9;
            ol10   <= ol9 or ol9_d1;
            ucol10 <= ucol9;
            vcol10 <= vcol9;
            yb10   <= yb9;

            --------------------------------------------------------------
            -- E11: shade sum + clamp
            --------------------------------------------------------------
            v_sh := resize(sx10, 18) + resize(sy10, 18);
            v_sh := shift_right(v_sh, s_rsh);
            if v_sh > 240 then
                shade11 <= to_signed(240, 9);
            elsif v_sh < -240 then
                shade11 <= to_signed(-240, 9);
            else
                shade11 <= resize(v_sh, 9);
            end if;
            ucol11 <= ucol10;
            vcol11 <= vcol10;
            ol11   <= ol10;
            yb11   <= yb10;

            --------------------------------------------------------------
            -- E12: light the paint (+ wet glint) + wash lift
            --------------------------------------------------------------
            v_y := signed(resize(yb11, 12)) + resize(shade11, 12);
            if shade11 > 176 then
                v_y := v_y + 64;                       -- wet-paint glint
            end if;
            if s_liftsh /= 0 then
                v_lift := shift_right(to_unsigned(1023, 10) - yb11,
                                      s_liftsh);
                v_y := v_y + signed(resize(v_lift, 12));
            end if;
            y12 <= f_cu10(v_y);
            u12 <= ucol11;
            v12 <= vcol11;
            ol12 <= ol11;

            --------------------------------------------------------------
            -- E13: outline apply (shift blends toward ink) -> wet result
            --------------------------------------------------------------
            v_iy := y12;  v_iu := u12;  v_iv := v12;
            if ol12 = '1' then
                case s_ol_lvl is
                    when "01" =>
                        v_iy := ('0' & y12(9 downto 1))
                                + ('0' & s_ink_y(9 downto 1));
                        v_iu := ('0' & u12(9 downto 1)) + 256;
                        v_iv := ('0' & v12(9 downto 1)) + 256;
                    when "10" =>
                        v_iy := ("00" & y12(9 downto 2))
                                + ('0' & s_ink_y(9 downto 1))
                                + ("00" & s_ink_y(9 downto 2));
                        v_iu := ("00" & u12(9 downto 2)) + 384;
                        v_iv := ("00" & v12(9 downto 2)) + 384;
                    when "11" =>
                        v_iy := s_ink_y;
                        v_iu := to_unsigned(512, 10);
                        v_iv := to_unsigned(512, 10);
                    when others =>
                        null;
                end case;
            end if;
            wy13 <= v_iy;
            wu13 <= v_iu;
            wv13 <= v_iv;

            --------------------------------------------------------------
            -- line / field bookkeeping
            --------------------------------------------------------------
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_x_count <= (others => '0');
                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 1000 then
                        s_aline <= s_aline + 1;
                    end if;
                else
                    s_aline <= (others => '0');
                end if;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_aline <= (others => '0');
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- packed row buffer, canonical 1W1R: read at px7 (this pixel, lands
    -- E8 = previous row's value), write {T8, idx, gray} at px8 one clock
    -- later -- the write pointer trails the read pointer, so reads always
    -- see the row above (redshift single-bank trick, zero seams)
    ------------------------------------------------------------------------
    p_row : process(clk)
    begin
        if rising_edge(clk) then
            row_q <= row_ram(to_integer(px6));
            row_ram(to_integer(px8)) <=
                std_logic_vector(T8) & std_logic_vector(m8) & gray8;
        end if;
    end process p_row;

    ------------------------------------------------------------------------
    -- dry/wet mix: interpolator_u per channel, t = Solvent ramp.
    -- wet is valid 13 clocks after data_in -> dry tap index 12.
    ------------------------------------------------------------------------
    s_iy_a <= unsigned(s_y_sr(12));
    s_iu_a <= unsigned(s_u_sr(12));
    s_iv_a <= unsigned(s_v_sr(12));

    interp_y_inst : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1',
                 a => s_iy_a, b => wy13, t => s_mix10,
                 result => s_iy_r, valid => s_iy_v);

    interp_u_inst : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1',
                 a => s_iu_a, b => wu13, t => s_mix10,
                 result => s_iu_r, valid => s_iu_v);

    interp_v_inst : entity work.interpolator_u
        generic map(G_WIDTH => 10, G_FRAC_BITS => 10,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1',
                 a => s_iv_a, b => wv13, t => s_mix10,
                 result => s_iv_r, valid => s_iv_v);

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(s_iy_r);
    data_out.u       <= std_logic_vector(s_iu_r);
    data_out.v       <= std_logic_vector(s_iv_r);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture turpentine;
