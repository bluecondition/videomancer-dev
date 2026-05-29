-- Mandala: folded-coordinate radial kaleidoscope.
--
-- Pixel coordinates are folded into the first octant by abs(x-cx),
-- abs(y-cy) and a swap (so ax >= ay).  This gives natural 8-fold
-- symmetry.  Two derived quantities drive the color:
--   rad   = ax + ay     (Manhattan radius from center)
--   ang   = ax - ay     (wedge-like angle proxy 0..max)
-- The "Symmetry" slider scales how many petals (spokes) appear, by
-- multiplying ang by a 4-bit petal_count and taking the resulting
-- modulus.  Ring Freq similarly scales rad before combining.  Rotation
-- offset comes from frame_count × Rotation parameter.  Center Shift
-- biases the apparent radius so the inner disc can be expanded.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Petals      (0..1023 → petal multiplier scale)
--   registers_in(1) = Ring Freq   (radial ring density)
--   registers_in(2) = Rotation    (frame-rate angular drift)
--   registers_in(3) = Brightness  (final Y multiplier)
--   registers_in(4) = Hue         (palette index offset)
--   registers_in(5) = Center Shift (inner radius bias)
--   registers_in(6) = Switches:
--     b0 Fold (0=Diamond / 1=Square — uses max instead of sum for rad)
--     b1 Rings (0=Smooth / 1=Stepped)
--     b2 Inner (0=Bright / 1=Dark center)
--     b3 Palette (0=Warm / 1=Cool)
--     b4 Invert
--   registers_in(7) = Symmetry (slider, KEY — petal count multiplier)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture mandala of program_top is

    constant LATENCY : natural := 16;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Position counters
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal last_x_r     : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal last_y_r     : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    -- Latched params
    signal petals_r       : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal ring_freq_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal rotation_r     : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bright_r       : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal hue_r          : unsigned(9 downto 0) := (others => '0');
    signal center_shift_r : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal symmetry_r     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal fold_r         : std_logic := '0';
    signal stepped_r      : std_logic := '0';
    signal dark_center_r  : std_logic := '0';
    signal cool_r         : std_logic := '0';
    signal invert_r       : std_logic := '0';

    -- Derived (frame-stable)
    signal cx_r           : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal cy_r           : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal rotation_acc_r : unsigned(15 downto 0) := (others => '0');
    -- Pre-summed (rotation + center_shift), computed at vsync so the
    -- per-pixel S8 stage adds only ONE 18-bit value (carry chain stays
    -- short).
    signal rot_cent_sum_r : unsigned(17 downto 0) := (others => '0');
    -- arms_count_r: how many angular wedges (controlled by Symmetry slider).
    signal arms_count_r   : unsigned(4 downto 0) := to_unsigned(6, 5);
    signal ring_count_r   : unsigned(4 downto 0) := to_unsigned(4, 5);
    -- lobes_count_r: petals knob → number of bumps per ring outline.
    signal lobes_count_r  : unsigned(3 downto 0) := to_unsigned(0, 4);
    -- lobes_amp_r: amplitude of the radial lobe modulation (also from
    -- petals knob), in raw pixels.
    signal lobes_amp_r    : unsigned(9 downto 0) := (others => '0');
    signal fractal_amt_r  : unsigned(3 downto 0) := to_unsigned(4, 4);

    -- ------------------------------------------------------------------
    -- 64-entry quarter-wave sine LUT (for continuous hue rotation).
    -- ------------------------------------------------------------------
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

    function quad_sin(ang : unsigned(9 downto 0)) return signed is
        variable v_q     : unsigned(1 downto 0);
        variable v_idx   : integer range 0 to 63;
        variable v_idx_m : integer range 0 to 63;
        variable v_mag   : unsigned(8 downto 0);
        variable v_out   : signed(10 downto 0);
    begin
        v_q     := ang(9 downto 8);
        v_idx   := to_integer(ang(7 downto 2));
        v_idx_m := 63 - v_idx;
        case v_q is
            when "00"   => v_mag := C_SIN_QUAD(v_idx);
            when "01"   => v_mag := C_SIN_QUAD(v_idx_m);
            when "10"   => v_mag := C_SIN_QUAD(v_idx);
            when others => v_mag := C_SIN_QUAD(v_idx_m);
        end case;
        if v_q(1) = '0' then v_out := signed(resize(v_mag, 11));
        else v_out := -signed(resize(v_mag, 11));
        end if;
        return v_out;
    end function;

    -- ------------------------------------------------------------------
    -- Palette: warm (red→orange→yellow→cream) and cool (blue→teal→violet)
    -- ------------------------------------------------------------------
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PY_WARM : t_pal := (
        to_unsigned(180, 10), to_unsigned(328, 10),
        to_unsigned(584, 10), to_unsigned(840, 10),
        to_unsigned(960, 10), to_unsigned(840, 10),
        to_unsigned(584, 10), to_unsigned(328, 10));
    constant C_PU_WARM : t_pal := (
        to_unsigned(540, 10), to_unsigned(960, 10),
        to_unsigned(772, 10), to_unsigned(584, 10),
        to_unsigned(512, 10), to_unsigned(584, 10),
        to_unsigned(772, 10), to_unsigned(960, 10));
    constant C_PV_WARM : t_pal := (
        to_unsigned(540, 10), to_unsigned(360, 10),
        to_unsigned(212, 10), to_unsigned( 64, 10),
        to_unsigned(512, 10), to_unsigned( 64, 10),
        to_unsigned(212, 10), to_unsigned(360, 10));

    constant C_PY_COOL : t_pal := (
        to_unsigned(800, 10), to_unsigned(580, 10),
        to_unsigned(349, 10), to_unsigned(192, 10),
        to_unsigned(164, 10), to_unsigned(192, 10),
        to_unsigned(349, 10), to_unsigned(580, 10));
    constant C_PU_COOL : t_pal := (
        to_unsigned(440, 10), to_unsigned(136, 10),
        to_unsigned(756, 10), to_unsigned(608, 10),
        to_unsigned(440, 10), to_unsigned(608, 10),
        to_unsigned(756, 10), to_unsigned(136, 10));
    constant C_PV_COOL : t_pal := (
        to_unsigned(960, 10), to_unsigned(216, 10),
        to_unsigned(852, 10), to_unsigned(696, 10),
        to_unsigned(960, 10), to_unsigned(696, 10),
        to_unsigned(852, 10), to_unsigned(216, 10));

    -- ------------------------------------------------------------------
    -- Per-pixel pipeline
    -- ------------------------------------------------------------------
    -- S0
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    -- S1: dx = pixel_x - cx, dy = pixel_y - cy (signed 13).
    signal s1_dx : signed(12 downto 0);
    signal s1_dy : signed(12 downto 0);

    -- S2: abs
    signal s2_ax : unsigned(11 downto 0);
    signal s2_ay : unsigned(11 downto 0);

    -- S3: swap so ax >= ay (gives 8-fold symmetry).
    signal s3_ax : unsigned(11 downto 0);
    signal s3_ay : unsigned(11 downto 0);

    -- S4: rad and ang prep.
    signal s4_rad : unsigned(12 downto 0);  -- sum
    signal s4_max : unsigned(11 downto 0);  -- for "Square" fold mode
    signal s4_ang : unsigned(11 downto 0);  -- ax - ay (≥ 0)

    -- S5: 3 phase multiplications (all registered raw).
    signal s5_arms_prod  : unsigned(16 downto 0);   -- ang × arms (12×5=17)
    signal s5_rings_prod : unsigned(17 downto 0);   -- rad × ring (13×5=18)
    signal s5_lobe_prod  : unsigned(15 downto 0);   -- ang × lobes (12×4=16)
    signal s5_use_max    : unsigned(11 downto 0);

    -- S6: extract top bits for phase contributions.  lobe = sin of phase.
    signal s6_arms_phase  : unsigned(9 downto 0);
    signal s6_rings_phase : unsigned(9 downto 0);
    signal s6_lobe        : signed(10 downto 0);
    signal s6_use_max     : unsigned(11 downto 0);

    -- S7: base phase combined; lobe × amp registered raw.
    signal s7_base_phase  : unsigned(17 downto 0);
    signal s7_lobe_amp    : signed(21 downto 0);  -- 11 × 11 = 22

    -- S6: combine into single phase value (16-bit base).
    signal s6_phase : unsigned(17 downto 0);

    -- S7: add rotation accumulator + fractal sub-wave layer.
    signal s7_phase    : unsigned(17 downto 0);  -- (unused legacy)
    signal s7_fractal1 : signed(10 downto 0);    -- (unused legacy)
    signal s7_fractal2 : signed(10 downto 0);    -- (unused legacy)
    signal s7_fractal3 : signed(10 downto 0);    -- (unused legacy)
    signal s7_petals_amt : unsigned(9 downto 0); -- (unused legacy)

    -- S8: combine fractal layers (weighted by Petals knob) with phase;
    -- bias by center_shift_r.
    signal s8_phase  : unsigned(17 downto 0);

    -- S9: hue_angle (top 10 bits) and Y modulation (next bits down).
    signal s9_hue  : unsigned(9 downto 0);
    signal s9_ymag : unsigned(9 downto 0);

    -- S10: sin/cos at hue_angle (signed 11).
    signal s10_sin  : signed(10 downto 0);
    signal s10_cos  : signed(10 downto 0);
    signal s10_ymag : unsigned(9 downto 0);

    -- S11: brightness multiply (registered raw).
    signal s11_y_mul : unsigned(19 downto 0);
    signal s11_u     : unsigned(9 downto 0);
    signal s11_v     : unsigned(9 downto 0);

    -- S12: extract Y; emit U/V.
    signal s12_y : unsigned(9 downto 0);
    signal s12_u : unsigned(9 downto 0);
    signal s12_v : unsigned(9 downto 0);

    -- S13: identity (slack).
    signal s13_y : unsigned(9 downto 0);
    signal s13_u : unsigned(9 downto 0);
    signal s13_v : unsigned(9 downto 0);

    -- S14: invert.
    signal s14_y : unsigned(9 downto 0);
    signal s14_u : unsigned(9 downto 0);
    signal s14_v : unsigned(9 downto 0);

    function abs_s13(v : signed(12 downto 0)) return unsigned is
        variable t : signed(12 downto 0);
    begin
        if v(12) = '1' then t := -v; else t := v; end if;
        return unsigned(std_logic_vector(t(11 downto 0)));
    end function;

begin

    -- ========================================================================
    -- Position tracking + param latching
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
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if v_h_edge = '1' then
                if pixel_x > last_x_r then last_x_r <= pixel_x; end if;
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                if pixel_y > last_y_r then last_y_r <= pixel_y; end if;
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                petals_r       <= unsigned(registers_in(0));
                ring_freq_r    <= unsigned(registers_in(1));
                rotation_r     <= unsigned(registers_in(2));
                bright_r       <= unsigned(registers_in(3));
                hue_r          <= unsigned(registers_in(4));
                center_shift_r <= unsigned(registers_in(5));
                fold_r         <= registers_in(6)(0);
                stepped_r      <= registers_in(6)(1);
                dark_center_r  <= registers_in(6)(2);
                cool_r         <= registers_in(6)(3);
                invert_r       <= registers_in(6)(4);
                symmetry_r     <= unsigned(registers_in(7));

                -- Center = last_*_r / 2.
                cx_r <= '0' & last_x_r(11 downto 1);
                cy_r <= '0' & last_y_r(11 downto 1);

                -- Symmetry SLIDER → arms_count_r (1..16 angular wedges).
                -- This drives the most visible structural change: more arms
                -- as you turn the knob up.
                arms_count_r  <= ("0" & unsigned(registers_in(7)(9 downto 6))) +
                                 to_unsigned(1, 5);
                -- Ring Freq KNOB → ring_count_r (1..32).
                ring_count_r  <= unsigned(registers_in(1)(9 downto 5)) +
                                 to_unsigned(1, 5);
                -- Petals KNOB → lobes_count_r (0..7 bumps per ring outline)
                -- + lobes_amp_r (bump depth in pixels, also from the knob).
                lobes_count_r <= unsigned(registers_in(0)(9 downto 6));
                lobes_amp_r   <= unsigned(registers_in(0));
                fractal_amt_r <= unsigned(registers_in(5)(9 downto 6));

                -- Rotation accumulator: advance per frame.
                rotation_acc_r <= rotation_acc_r +
                                  ("000000" & unsigned(registers_in(2)));
                -- Pre-sum rotation + center for per-pixel use.
                rot_cent_sum_r <= ("00" & (rotation_acc_r +
                                  ("000000" & unsigned(registers_in(2))))) +
                                  ("00000000" & unsigned(registers_in(5)));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_a : unsigned(11 downto 0);
        variable v_b : unsigned(11 downto 0);
        variable v_y_re : signed(11 downto 0);
        variable v_u_re : signed(13 downto 0);
        variable v_v_re : signed(13 downto 0);
        variable v_pidx : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: dx, dy signed
            s1_dx <= signed('0' & std_logic_vector(s0_x)) -
                     signed('0' & std_logic_vector(cx_r));
            s1_dy <= signed('0' & std_logic_vector(s0_y)) -
                     signed('0' & std_logic_vector(cy_r));

            -- S2: abs
            s2_ax <= abs_s13(s1_dx);
            s2_ay <= abs_s13(s1_dy);

            -- S3: swap so ax >= ay
            if s2_ax >= s2_ay then
                s3_ax <= s2_ax; s3_ay <= s2_ay;
            else
                s3_ax <= s2_ay; s3_ay <= s2_ax;
            end if;

            -- S4: rad (sum) and ang (diff); also max for Square fold mode.
            s4_rad <= ('0' & s3_ax) + ('0' & s3_ay);
            s4_max <= s3_ax;
            s4_ang <= s3_ax - s3_ay;  -- always non-negative because ax>=ay

            -- S5: registered raw products for all three phase contributors.
            --   arms_prod  = ang × arms_count  (Symmetry slider → wedges)
            --   rings_prod = rad × ring_count  (Ring Freq knob → rings)
            --   lobe_prod  = ang × lobes_count (Petals knob → bumps/scallops)
            s5_arms_prod  <= s4_ang * arms_count_r;
            s5_rings_prod <= s4_rad * ring_count_r;
            s5_lobe_prod  <= s4_ang * lobes_count_r;
            s5_use_max    <= s4_max;

            -- S6: extract top 10 bits of each phase contributor; sin of lobe.
            s6_arms_phase  <= s5_arms_prod(16 downto 7);
            s6_rings_phase <= s5_rings_prod(17 downto 8);
            s6_lobe        <= quad_sin(s5_lobe_prod(15 downto 6));
            s6_use_max     <= s5_use_max;

            -- S7: combine arms_phase + rings_phase only (1 add).
            -- Register lobe × amp raw (registered multiply).
            if fold_r = '1' then
                s7_base_phase <= resize(s6_arms_phase, 18) +
                                 resize(s6_use_max, 18);
            else
                s7_base_phase <= resize(s6_arms_phase, 18) +
                                 resize(s6_rings_phase, 18);
            end if;
            s7_lobe_amp <= s6_lobe *
                           signed('0' & std_logic_vector(lobes_amp_r));

            -- S8: combine.  base_phase + rot_cent_sum (1 add).  Then add
            -- lobe modulation (extracted from registered lobe_amp).
            -- Two adds in parallel after register, single chain depth.
            s8_phase <= (s7_base_phase + rot_cent_sum_r) +
                        unsigned(std_logic_vector(
                            resize(shift_right(s7_lobe_amp, 6), 18)));

            -- S9: extract hue angle (full 10-bit) and Y magnitude.  In
            -- stepped mode quantize the magnitude to coarse bands.
            s9_hue  <= s8_phase(17 downto 8) + hue_r;
            if stepped_r = '1' then
                s9_ymag <= s8_phase(13 downto 11) & "0000000";
            else
                s9_ymag <= s8_phase(13 downto 4);
            end if;

            -- S10: sin/cos lookup for chroma vector.  Cool toggle reverses
            -- the rotation direction.
            if cool_r = '0' then
                s10_sin <= quad_sin(s9_hue);
                s10_cos <= quad_sin(s9_hue + to_unsigned(256, 10));
            else
                s10_sin <= quad_sin(s9_hue + to_unsigned(512, 10));
                s10_cos <= quad_sin(s9_hue + to_unsigned(768, 10));
            end if;
            s10_ymag <= s9_ymag;

            -- S11: brightness multiply, derive chroma from sin/cos.
            -- Y baseline depends on dark_center_r: when set, inner disc is
            -- darker; otherwise it's brighter.
            if dark_center_r = '1' then
                s11_y_mul <= s10_ymag * bright_r;
            else
                s11_y_mul <= (to_unsigned(1023, 10) - s10_ymag) * bright_r;
            end if;
            v_u_re := to_signed(512, 14) + resize(shift_right(s10_cos, 1), 14);
            v_v_re := to_signed(512, 14) + resize(shift_right(s10_sin, 1), 14);
            if v_u_re < 0 then s11_u <= to_unsigned(0, 10);
            elsif v_u_re > 1023 then s11_u <= to_unsigned(1023, 10);
            else s11_u <= unsigned(std_logic_vector(v_u_re(9 downto 0)));
            end if;
            if v_v_re < 0 then s11_v <= to_unsigned(0, 10);
            elsif v_v_re > 1023 then s11_v <= to_unsigned(1023, 10);
            else s11_v <= unsigned(std_logic_vector(v_v_re(9 downto 0)));
            end if;

            -- S12: extract Y.
            s12_y <= s11_y_mul(19 downto 10);
            s12_u <= s11_u;
            s12_v <= s11_v;

            -- S13: slack stage.
            s13_y <= s12_y;
            s13_u <= s12_u;
            s13_v <= s12_v;

            -- S14: invert.
            if invert_r = '1' then
                s14_y <= to_unsigned(1023, 10) - s13_y;
            else
                s14_y <= s13_y;
            end if;
            s14_u <= s13_u;
            s14_v <= s13_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s14_y);
    data_out.u       <= std_logic_vector(s14_u);
    data_out.v       <= std_logic_vector(s14_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture mandala;
