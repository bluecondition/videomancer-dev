-- Spectrum: 32 vertical bars with sine-animated heights computed per-pixel.
--
-- Bar height is computed on-the-fly from the bar's index and an animated
-- phase; this avoids any 32-element height-array register and its slow
-- variable-index mux on the critical path.  Bar visibility is then a
-- simple comparison against the pixel's y position.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Phase Spread  (rotary 1)
--   registers_in(1) = Anim Speed    (rotary 2)
--   registers_in(2) = Mirror Span   (rotary 3 — width of mirror axis)
--   registers_in(3) = Hue Spread    (rotary 4)
--   registers_in(4) = Gap Size      (rotary 5)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Style    (0=Solid / 1=Outline)
--     b1   Mirror   (0=Off / 1=Mirror about centre)
--     b2   Cap      (0=Off / 1=Peak cap line)
--     b3   Palette  (0=Color / 1=Mono)
--     b4   Invert
--   registers_in(7) = Bar Height (slider, KEY — max bar height)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture spectrum of program_top is

    constant LATENCY : natural := 10;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal phase_r    : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal speed_r    : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal mspan_r    : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal hue_r      : unsigned(9 downto 0) := to_unsigned(64,  10);
    signal gap_r      : unsigned(9 downto 0) := to_unsigned(96,  10);
    signal bright_r   : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal height_r   : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal style_r    : std_logic := '0';
    signal mirror_r   : std_logic := '0';
    signal cap_r      : std_logic := '0';
    signal mono_r     : std_logic := '0';
    signal invert_r   : std_logic := '0';

    signal anim_phase_r : unsigned(11 downto 0) := (others => '0');

    -- 64-entry quarter sine LUT 0..511 (positive part of full sine)
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

    -- 8-color spectrum palette
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(580, 10), to_unsigned(620, 10),
        to_unsigned(840, 10), to_unsigned(700, 10),
        to_unsigned(540, 10), to_unsigned(420, 10),
        to_unsigned(380, 10), to_unsigned(680, 10));
    constant C_PAL_U : t_pal := (
        to_unsigned(360, 10), to_unsigned(440, 10),
        to_unsigned(280, 10), to_unsigned(640, 10),
        to_unsigned(840, 10), to_unsigned(720, 10),
        to_unsigned(620, 10), to_unsigned(420, 10));
    constant C_PAL_V : t_pal := (
        to_unsigned(820, 10), to_unsigned(720, 10),
        to_unsigned(380, 10), to_unsigned(320, 10),
        to_unsigned(420, 10), to_unsigned(620, 10),
        to_unsigned(780, 10), to_unsigned(680, 10));

    -- Pipeline signals
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    -- S1: bar index (5-bit) + in-bar pixel position (6-bit)
    signal s1_bar_idx : unsigned(4 downto 0);
    signal s1_in_x    : unsigned(5 downto 0);
    signal s1_y       : unsigned(11 downto 0);

    -- S2: bar phase = anim_phase + bar_idx * phase_r(9 downto 5)
    --     (5×5=10 multiply, small)
    signal s2_phase_mul : unsigned(9 downto 0);
    signal s2_bar_idx   : unsigned(4 downto 0);
    signal s2_in_x      : unsigned(5 downto 0);
    signal s2_y         : unsigned(11 downto 0);

    -- S3: 10-bit angle for sin lookup
    signal s3_angle  : unsigned(9 downto 0);
    signal s3_bar_idx : unsigned(4 downto 0);
    signal s3_in_x   : unsigned(5 downto 0);
    signal s3_y      : unsigned(11 downto 0);

    -- S4: sin value (0..491 unsigned, positive sine only)
    signal s4_sin : unsigned(8 downto 0);
    signal s4_bar_idx : unsigned(4 downto 0);
    signal s4_in_x : unsigned(5 downto 0);
    signal s4_y    : unsigned(11 downto 0);

    -- S5: sin * height_r (9×10 = 19-bit) raw
    signal s5_hmul : unsigned(18 downto 0);
    signal s5_bar_idx : unsigned(4 downto 0);
    signal s5_in_x : unsigned(5 downto 0);
    signal s5_y    : unsigned(11 downto 0);

    -- S6: bar_top = 1080 - height (signed for compare)
    signal s6_bar_top : signed(11 downto 0);
    signal s6_height  : unsigned(9 downto 0);
    signal s6_bar_idx : unsigned(4 downto 0);
    signal s6_in_x : unsigned(5 downto 0);
    signal s6_y    : unsigned(11 downto 0);

    -- S7: visibility flags + cap line
    signal s7_in_y  : std_logic;
    signal s7_in_x  : std_logic;
    signal s7_cap   : std_logic;
    signal s7_bar_idx : unsigned(4 downto 0);

    -- S8: palette + assemble pixel
    signal s8_y : unsigned(9 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);

    -- S9: brightness mul + invert
    signal s9_y : unsigned(9 downto 0);
    signal s9_u : unsigned(9 downto 0);
    signal s9_v : unsigned(9 downto 0);

    function quad_sin(ang : unsigned(9 downto 0)) return unsigned is
        variable v_q   : unsigned(1 downto 0);
        variable v_idx : integer range 0 to 63;
        variable v_mag : unsigned(8 downto 0);
    begin
        v_q := ang(9 downto 8);
        v_idx := to_integer(ang(7 downto 2));
        case v_q is
            when "00"   => v_mag := C_SIN_QUAD(v_idx);
            when "01"   => v_mag := C_SIN_QUAD(63 - v_idx);
            when "10"   => v_mag := C_SIN_QUAD(v_idx);
            when others => v_mag := C_SIN_QUAD(63 - v_idx);
        end case;
        -- Negative half returns 0 (we only render upward bars).
        if v_q(1) = '1' then
            v_mag := to_unsigned(0, 9);
        end if;
        return v_mag;
    end function;

begin

    -- ========================================================================
    -- Position + parameters
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_speed  : unsigned(9 downto 0);
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

                phase_r   <= unsigned(registers_in(0));
                speed_r   <= unsigned(registers_in(1));
                mspan_r   <= unsigned(registers_in(2));
                hue_r     <= unsigned(registers_in(3));
                gap_r     <= unsigned(registers_in(4));
                bright_r  <= unsigned(registers_in(5));
                style_r   <= registers_in(6)(0);
                mirror_r  <= registers_in(6)(1);
                cap_r     <= registers_in(6)(2);
                mono_r    <= registers_in(6)(3);
                invert_r  <= registers_in(6)(4);
                height_r  <= unsigned(registers_in(7));

                v_speed := "00000" & unsigned(registers_in(1)(9 downto 5));
                anim_phase_r <= anim_phase_r + ("00" & v_speed);
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pidx : integer range 0 to 7;
        variable v_y_mul : unsigned(19 downto 0);
        variable v_height : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: bar_idx = pixel_x >> 6, in-bar position
            if mirror_r = '1' and pixel_x >= to_unsigned(960, 12) then
                -- Fold right half to mirror left
                s1_bar_idx <= to_unsigned(29, 5) -
                              resize(pixel_x(10 downto 6) - to_unsigned(15, 5), 5);
            else
                s1_bar_idx <= resize(pixel_x(10 downto 6), 5);
            end if;
            s1_in_x <= pixel_x(5 downto 0);
            s1_y <= pixel_y;

            -- S2: bar_phase = bar_idx * phase_r(9 downto 5) (5×5 = 10-bit)
            s2_phase_mul <= s1_bar_idx * phase_r(9 downto 5);
            s2_bar_idx <= s1_bar_idx;
            s2_in_x <= s1_in_x;
            s2_y <= s1_y;

            -- S3: angle = anim_phase + bar_phase, take low 10 bits
            s3_angle <= anim_phase_r(9 downto 0) + s2_phase_mul;
            s3_bar_idx <= s2_bar_idx;
            s3_in_x <= s2_in_x;
            s3_y <= s2_y;

            -- S4: sin lookup
            s4_sin <= quad_sin(s3_angle);
            s4_bar_idx <= s3_bar_idx;
            s4_in_x <= s3_in_x;
            s4_y <= s3_y;

            -- S5: sin * height_r (9×10 = 19) raw
            s5_hmul <= s4_sin * height_r;
            s5_bar_idx <= s4_bar_idx;
            s5_in_x <= s4_in_x;
            s5_y <= s4_y;

            -- S6: bar_top = 1080 - height (extract height as 10-bit)
            v_height := s5_hmul(18 downto 9);  -- sin*height/512 → 0..980
            s6_height <= v_height;
            s6_bar_top <= to_signed(1080, 12) - signed(resize(v_height, 12));
            s6_bar_idx <= s5_bar_idx;
            s6_in_x <= s5_in_x;
            s6_y <= s5_y;

            -- S7: visibility
            if signed(resize(s6_y, 12)) >= s6_bar_top then
                s7_in_y <= '1';
            else
                s7_in_y <= '0';
            end if;
            if ("0" & s6_in_x) < (to_unsigned(64, 7) -
                                   resize(gap_r(9 downto 4), 7)) then
                s7_in_x <= '1';
            else
                s7_in_x <= '0';
            end if;
            -- Cap: if within 4 pixels of bar top
            if cap_r = '1' and
               signed(resize(s6_y, 12)) >= s6_bar_top and
               signed(resize(s6_y, 12)) <= (s6_bar_top + to_signed(6, 12)) then
                s7_cap <= '1';
            else
                s7_cap <= '0';
            end if;
            s7_bar_idx <= s6_bar_idx;

            -- S8: color assign
            if s7_cap = '1' then
                -- Cap line bright white
                s8_y <= to_unsigned(1023, 10);
                s8_u <= to_unsigned(512, 10);
                s8_v <= to_unsigned(512, 10);
            elsif (s7_in_y and s7_in_x) = '1' then
                if mono_r = '1' then
                    s8_y <= to_unsigned(900, 10);
                    s8_u <= to_unsigned(512, 10);
                    s8_v <= to_unsigned(512, 10);
                else
                    v_pidx := to_integer(s7_bar_idx(2 downto 0) +
                                          hue_r(9 downto 7));
                    s8_y <= C_PAL_Y(v_pidx);
                    s8_u <= C_PAL_U(v_pidx);
                    s8_v <= C_PAL_V(v_pidx);
                end if;
            else
                s8_y <= to_unsigned(40, 10);
                s8_u <= to_unsigned(512, 10);
                s8_v <= to_unsigned(512, 10);
            end if;

            -- S9: brightness mul + invert
            v_y_mul := s8_y * bright_r;
            if invert_r = '1' then
                s9_y <= to_unsigned(1023, 10) - v_y_mul(19 downto 10);
            else
                s9_y <= v_y_mul(19 downto 10);
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

end architecture spectrum;
