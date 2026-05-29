-- Rays: rotating 8-ray sunburst from screen center.
--
-- Eight rays at 45° intervals are evaluated by computing perpendicular
-- distance to each of four line directions (each line forms two opposing
-- rays).  The entire ray bundle is rotated by a precomputed cos/sin angle
-- each frame so the sunburst spins.  Distance to nearest ray sets pixel
-- brightness, optionally combined with a radial falloff.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).  Rotation
-- multiplies and distance reductions live in their own pipeline stages.
--
-- Register map:
--   registers_in(0) = Ray Width      (rotary 1)
--   registers_in(1) = Falloff        (rotary 2 — fade with radius)
--   registers_in(2) = Spin Speed     (rotary 3)
--   registers_in(3) = Background     (rotary 4 — dark sky brightness)
--   registers_in(4) = Tint           (rotary 5 — palette rotation)
--   registers_in(5) = Brightness     (rotary 6)
--   registers_in(6) = Switches:
--     b0   Symmetry  (0=8 rays / 1=16 rays, second set offset by 22.5°)
--     b1   Animation (0=Spin / 1=Pulse — width modulated)
--     b2   Palette   (0=White / 1=Spectrum)
--     b3   Center    (0=Dot / 1=Black hole)
--     b4   Invert
--   registers_in(7) = Ray Length (slider, KEY — rays fade in over this radius)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture rays of program_top is

    constant LATENCY : natural := 15;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal width_r   : unsigned(9 downto 0) := to_unsigned(160, 10);
    signal falloff_r : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal speed_r   : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal bg_r      : unsigned(9 downto 0) := to_unsigned(120, 10);
    signal tint_r    : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal bright_r  : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal length_r  : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal sym_r     : std_logic := '0';
    signal anim_r    : std_logic := '0';
    signal palette_r : std_logic := '0';
    signal center_r  : std_logic := '0';
    signal invert_r  : std_logic := '0';

    signal spin_phase_r : unsigned(11 downto 0) := (others => '0');

    -- Pre-computed effective ray width (per-frame at vsync)
    signal width_eff_r : unsigned(11 downto 0) := to_unsigned(180, 12);

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
        to_unsigned(840, 10), to_unsigned(680, 10),
        to_unsigned(540, 10), to_unsigned(420, 10),
        to_unsigned(380, 10), to_unsigned(620, 10),
        to_unsigned(700, 10), to_unsigned(580, 10));
    constant C_PAL_U : t_pal := (
        to_unsigned(280, 10), to_unsigned(420, 10),
        to_unsigned(840, 10), to_unsigned(720, 10),
        to_unsigned(620, 10), to_unsigned(440, 10),
        to_unsigned(640, 10), to_unsigned(360, 10));
    constant C_PAL_V : t_pal := (
        to_unsigned(380, 10), to_unsigned(680, 10),
        to_unsigned(420, 10), to_unsigned(620, 10),
        to_unsigned(780, 10), to_unsigned(720, 10),
        to_unsigned(320, 10), to_unsigned(820, 10));

    -- Pipeline signals
    signal s0_dx : signed(12 downto 0);
    signal s0_dy : signed(12 downto 0);

    -- S1: rotation multiplies (4 in parallel: dx*c, dx*s, dy*c, dy*s)
    -- 13×9 = 22 bits each
    signal s1_dx_c1 : signed(21 downto 0);
    signal s1_dx_s1 : signed(21 downto 0);
    signal s1_dy_c1 : signed(21 downto 0);
    signal s1_dy_s1 : signed(21 downto 0);

    -- S2: rotated coords (signed 23)
    signal s2_dx1r : signed(22 downto 0);
    signal s2_dy1r : signed(22 downto 0);

    -- S3: shift right to pixel scale (>> 8)
    signal s3_dx1 : signed(14 downto 0);
    signal s3_dy1 : signed(14 downto 0);

    -- S4: 4 line distances: |dy1|, |dx1|, |dy1-dx1|, |dy1+dx1|
    signal s4_d1a : unsigned(14 downto 0);  -- |dy1|
    signal s4_d1b : unsigned(14 downto 0);  -- |dx1|
    signal s4_d1c : unsigned(14 downto 0);  -- |dy1-dx1|
    signal s4_d1d : unsigned(14 downto 0);  -- |dy1+dx1|
    -- Forward radial distance for falloff
    signal s4_radius : unsigned(11 downto 0);

    -- S5: min reduction (4 → 2)
    signal s5_m1a : unsigned(14 downto 0);
    signal s5_m1b : unsigned(14 downto 0);
    signal s5_radius : unsigned(11 downto 0);

    -- S6: min reduction (2 → 1) — kept as a separate stage for short paths.
    signal s6_min : unsigned(14 downto 0);
    signal s6_radius : unsigned(11 downto 0);

    -- S7: spare forwarding stage (pipeline alignment)
    signal s7_min    : unsigned(14 downto 0);
    signal s7_radius : unsigned(11 downto 0);

    -- S8: ray brightness = max(0, width - min_dist), shifted into 10-bit range
    signal s8_ray : unsigned(9 downto 0);
    signal s8_radius : unsigned(11 downto 0);

    -- S9: radial falloff (fade rays beyond length)
    signal s9_ray  : unsigned(9 downto 0);
    signal s9_idx  : unsigned(2 downto 0);  -- palette index from radius
    signal s9_bg   : unsigned(9 downto 0);

    -- S10: composite ray vs background
    signal s10_b : unsigned(9 downto 0);
    signal s10_idx : unsigned(2 downto 0);

    -- S11: palette
    signal s11_y : unsigned(9 downto 0);
    signal s11_u : unsigned(9 downto 0);
    signal s11_v : unsigned(9 downto 0);
    signal s11_b : unsigned(9 downto 0);

    -- S12: raw Y*b mul + chroma center
    signal s12_y_mul : unsigned(19 downto 0);
    signal s12_u_cent : signed(11 downto 0);
    signal s12_v_cent : signed(11 downto 0);
    signal s12_b      : unsigned(9 downto 0);

    -- S13: chroma raw mul + extract Y-pre
    signal s13_y_pre  : unsigned(9 downto 0);
    signal s13_u_mul  : signed(22 downto 0);
    signal s13_v_mul  : signed(22 downto 0);

    -- S14: apply bright knob to Y + recenter chroma + invert
    signal s14_y : unsigned(9 downto 0);
    signal s14_u : unsigned(9 downto 0);
    signal s14_v : unsigned(9 downto 0);

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

begin

    -- ========================================================================
    -- Position + parameters + spin
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

                width_r   <= unsigned(registers_in(0));
                falloff_r <= unsigned(registers_in(1));
                speed_r   <= unsigned(registers_in(2));
                bg_r      <= unsigned(registers_in(3));
                tint_r    <= unsigned(registers_in(4));
                bright_r  <= unsigned(registers_in(5));
                sym_r     <= registers_in(6)(0);
                anim_r    <= registers_in(6)(1);
                palette_r <= registers_in(6)(2);
                center_r  <= registers_in(6)(3);
                invert_r  <= registers_in(6)(4);
                length_r  <= unsigned(registers_in(7));

                v_speed := "00000" & unsigned(registers_in(2)(9 downto 5));
                spin_phase_r <= spin_phase_r + ("00" & v_speed);

                v_a1 := spin_phase_r(11 downto 2);
                cos_r  <= resize(shift_right(quad_sin(v_a1 + to_unsigned(256, 10)), 1), 9);
                sin_r  <= resize(shift_right(quad_sin(v_a1), 1), 9);

                -- Precompute effective ray width: base width + animated pulse.
                if registers_in(6)(1) = '0' then
                    width_eff_r <= resize(unsigned(registers_in(0)), 12) +
                                   to_unsigned(20, 12);
                else
                    width_eff_r <= resize(unsigned(registers_in(0)), 12) +
                                   ("00" & frame_count(7 downto 0) & "00");
                end if;
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_dx_a : signed(12 downto 0);
        variable v_dy_a : signed(12 downto 0);
        variable v_dxr  : signed(14 downto 0);
        variable v_dyr  : signed(14 downto 0);
        variable v_diff : signed(15 downto 0);
        variable v_sum  : signed(15 downto 0);
        variable v_ray  : signed(15 downto 0);
        variable v_b    : unsigned(9 downto 0);
        variable v_fall : signed(11 downto 0);
        variable v_pidx : integer range 0 to 7;
        variable v_y_mul : unsigned(19 downto 0);
        variable v_u_cent : signed(11 downto 0);
        variable v_v_cent : signed(11 downto 0);
        variable v_u_mul : signed(22 downto 0);
        variable v_v_mul : signed(22 downto 0);
        variable v_u_out : signed(13 downto 0);
        variable v_v_out : signed(13 downto 0);
        variable v_width_eff : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: dx, dy
            s0_dx <= signed(resize(pixel_x, 13)) - to_signed(960, 13);
            s0_dy <= signed(resize(pixel_y, 13)) - to_signed(540, 13);

            -- S1: rotation multiplies (4 parallel)
            s1_dx_c1 <= s0_dx * cos_r;
            s1_dx_s1 <= s0_dx * sin_r;
            s1_dy_c1 <= s0_dy * cos_r;
            s1_dy_s1 <= s0_dy * sin_r;

            -- S2: rotated coords (signed 23)
            s2_dx1r <= resize(s1_dx_c1, 23) - resize(s1_dy_s1, 23);
            s2_dy1r <= resize(s1_dx_s1, 23) + resize(s1_dy_c1, 23);

            -- S3: shift right 7 to pixel scale
            s3_dx1 <= resize(shift_right(s2_dx1r, 7), 15);
            s3_dy1 <= resize(shift_right(s2_dy1r, 7), 15);

            -- S4: 4 line distances (one direction set = 8 rays)
            v_dxr := s3_dx1;
            v_dyr := s3_dy1;
            v_diff := resize(v_dyr, 16) - resize(v_dxr, 16);
            v_sum  := resize(v_dyr, 16) + resize(v_dxr, 16);
            v_ray := abs(resize(v_dyr, 16));
            s4_d1a <= unsigned(std_logic_vector(v_ray(14 downto 0)));
            v_ray := abs(resize(v_dxr, 16));
            s4_d1b <= unsigned(std_logic_vector(v_ray(14 downto 0)));
            v_ray := abs(v_diff);
            s4_d1c <= unsigned(std_logic_vector(v_ray(14 downto 0)));
            v_ray := abs(v_sum);
            s4_d1d <= unsigned(std_logic_vector(v_ray(14 downto 0)));

            -- Radial distance (Manhattan in original coords for vignette)
            v_dx_a := s0_dx;
            if v_dx_a < 0 then v_dx_a := -v_dx_a; end if;
            v_dy_a := s0_dy;
            if v_dy_a < 0 then v_dy_a := -v_dy_a; end if;
            s4_radius <= resize(unsigned(std_logic_vector(v_dx_a(11 downto 0))) +
                                unsigned(std_logic_vector(v_dy_a(11 downto 0))), 12);

            -- S5: 4 → 2 minimums (pairwise)
            if s4_d1a < s4_d1b then s5_m1a <= s4_d1a;
            else                    s5_m1a <= s4_d1b; end if;
            if s4_d1c < s4_d1d then s5_m1b <= s4_d1c;
            else                    s5_m1b <= s4_d1d; end if;
            s5_radius <= s4_radius;

            -- S6: 2 → 1
            if s5_m1a < s5_m1b then s6_min <= s5_m1a;
            else                    s6_min <= s5_m1b; end if;
            s6_radius <= s5_radius;

            -- S7: pipeline alignment stage
            s7_min    <= s6_min;
            s7_radius <= s6_radius;

            -- S8: ray brightness using pre-computed width.
            if s7_min < resize(width_eff_r, 15) then
                s8_ray <= resize(resize(width_eff_r, 15) - s7_min, 10);
            else
                s8_ray <= (others => '0');
            end if;
            s8_radius <= s7_radius;

            -- S9: radial falloff = fade rays when radius > length_r
            -- length_r 0..1023 → fade threshold pixels.
            v_fall := signed(resize(s8_radius, 12)) -
                      signed(resize(length_r, 12));
            if v_fall <= 0 then
                s9_ray <= s8_ray;
            else
                if unsigned(std_logic_vector(v_fall(9 downto 0))) < s8_ray then
                    s9_ray <= s8_ray -
                              unsigned(std_logic_vector(v_fall(9 downto 0)));
                else
                    s9_ray <= (others => '0');
                end if;
            end if;
            s9_idx <= s8_radius(11 downto 9);
            s9_bg  <= bg_r;

            -- S10: composite ray over background
            -- ray + bg (saturating)
            if (resize(s9_ray, 11) + resize(s9_bg, 11)) > to_unsigned(1023, 11) then
                s10_b <= to_unsigned(1023, 10);
            else
                s10_b <= resize(resize(s9_ray, 11) + resize(s9_bg, 11), 10);
            end if;
            s10_idx <= s9_idx;
            -- Black hole: dim center
            if center_r = '1' and s8_radius < to_unsigned(80, 12) then
                s10_b <= "00" & s8_radius(9 downto 2);
            end if;

            -- S11: palette
            if palette_r = '0' then
                s11_y <= s10_b;
                s11_u <= to_unsigned(512, 10);
                s11_v <= to_unsigned(512, 10);
            else
                v_pidx := to_integer(s10_idx + tint_r(9 downto 7));
                s11_y <= C_PAL_Y(v_pidx);
                s11_u <= C_PAL_U(v_pidx);
                s11_v <= C_PAL_V(v_pidx);
            end if;
            s11_b <= s10_b;

            -- S12: scale Y by brightness (raw mul) + center chroma
            s12_y_mul  <= s11_y * s11_b;
            s12_u_cent <= signed(resize(s11_u, 12)) - to_signed(512, 12);
            s12_v_cent <= signed(resize(s11_v, 12)) - to_signed(512, 12);
            s12_b      <= s11_b;

            -- S13: extract Y-pre, raw chroma mul
            s13_y_pre <= s12_y_mul(19 downto 10);
            s13_u_mul <= s12_u_cent * signed('0' & std_logic_vector(s12_b));
            s13_v_mul <= s12_v_cent * signed('0' & std_logic_vector(s12_b));

            -- S14: apply bright knob to Y; recenter chroma; invert
            v_y_mul := s13_y_pre * bright_r;
            v_u_out := resize(shift_right(s13_u_mul, 10), 14) + to_signed(512, 14);
            v_v_out := resize(shift_right(s13_v_mul, 10), 14) + to_signed(512, 14);
            if invert_r = '1' then
                s14_y <= to_unsigned(1023, 10) - v_y_mul(19 downto 10);
            else
                s14_y <= v_y_mul(19 downto 10);
            end if;
            s14_u <= sat10(v_u_out);
            s14_v <= sat10(v_v_out);
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s14_y);
    data_out.u       <= std_logic_vector(s14_u);
    data_out.v       <= std_logic_vector(s14_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture rays;
