-- Plasma: classic demoscene plasma synthesizer.
--
-- The screen is painted as the sum of four sinusoids in different
-- spatial bases:
--   w1 = sin(x * fx + t)
--   w2 = sin(y * fy + t)
--   w3 = sin((x + y) * fd + t)
--   w4 = sin((x - y) * fd + t)
-- The sum modulates an 8-color palette LUT.  Frame phase 't' drifts at
-- Speed, giving the classic boiling plasma motion.  Sliders shape the
-- frequencies; the Master Scale slider (param 12) multiplies all spatial
-- frequencies together to set the overall feature size.
--
-- A small 64-entry quarter-wave sine LUT (or triangle wave) is used, so
-- no big LUT mux ever sits in a critical path.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Freq X
--   registers_in(1) = Freq Y
--   registers_in(2) = Freq Diagonal (used for both x+y and x-y waves)
--   registers_in(3) = Speed         (phase drift per frame)
--   registers_in(4) = Hue Shift     (palette index offset)
--   registers_in(5) = Brightness
--   registers_in(6) = Switches:
--     b0 Palette (0=Fire / 1=Aqua)
--     b1 Mode    (0=Smooth / 1=Banded)
--     b2 Sum     (0=4-Wave / 1=3-Wave)
--     b3 Wave    (0=Sine / 1=Triangle)
--     b4 Invert
--   registers_in(7) = Master Scale (slider, KEY — global frequency scalar)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture plasma of program_top is

    constant LATENCY : natural := 18;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Position counters
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    -- Latched params
    signal fx_r       : unsigned(9 downto 0);
    signal fy_r       : unsigned(9 downto 0);
    signal fd_r       : unsigned(9 downto 0);
    signal speed_r    : unsigned(9 downto 0);
    signal hue_r      : unsigned(9 downto 0);
    signal bright_r   : unsigned(9 downto 0);
    signal scale_r    : unsigned(9 downto 0);
    signal lumamod_r  : std_logic := '0';
    signal banded_r   : std_logic := '0';
    signal three_r    : std_logic := '0';
    signal tri_r      : std_logic := '0';
    signal invert_r   : std_logic := '0';

    signal t_phase_r  : unsigned(15 downto 0) := (others => '0');

    -- ------------------------------------------------------------------
    -- 64-entry quarter-wave sine LUT
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

    -- Palette Fire: black→red→orange→yellow→white→yellow→orange→red (loop).
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PY_FIRE : t_pal := (
        to_unsigned( 64, 10), to_unsigned(200, 10),
        to_unsigned(360, 10), to_unsigned(584, 10),
        to_unsigned(840, 10), to_unsigned(960, 10),
        to_unsigned(840, 10), to_unsigned(584, 10));
    constant C_PU_FIRE : t_pal := (
        to_unsigned(512, 10), to_unsigned(560, 10),
        to_unsigned(820, 10), to_unsigned(772, 10),
        to_unsigned(584, 10), to_unsigned(512, 10),
        to_unsigned(584, 10), to_unsigned(772, 10));
    constant C_PV_FIRE : t_pal := (
        to_unsigned(512, 10), to_unsigned(640, 10),
        to_unsigned(360, 10), to_unsigned(212, 10),
        to_unsigned( 64, 10), to_unsigned(512, 10),
        to_unsigned( 64, 10), to_unsigned(212, 10));

    constant C_PY_AQUA : t_pal := (
        to_unsigned( 64, 10), to_unsigned(200, 10),
        to_unsigned(380, 10), to_unsigned(580, 10),
        to_unsigned(840, 10), to_unsigned(960, 10),
        to_unsigned(780, 10), to_unsigned(580, 10));
    constant C_PU_AQUA : t_pal := (
        to_unsigned(512, 10), to_unsigned(640, 10),
        to_unsigned(440, 10), to_unsigned(136, 10),
        to_unsigned(584, 10), to_unsigned(512, 10),
        to_unsigned(380, 10), to_unsigned(136, 10));
    constant C_PV_AQUA : t_pal := (
        to_unsigned(512, 10), to_unsigned(440, 10),
        to_unsigned(960, 10), to_unsigned(216, 10),
        to_unsigned( 64, 10), to_unsigned(512, 10),
        to_unsigned(696, 10), to_unsigned(216, 10));

    -- ------------------------------------------------------------------
    -- Per-pixel pipeline
    -- ------------------------------------------------------------------
    -- S0
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    -- S1: pre-compute x+y, x-y (registered).
    signal s1_x : unsigned(11 downto 0);
    signal s1_y : unsigned(11 downto 0);
    signal s1_sum : unsigned(12 downto 0);
    signal s1_dif : signed(12 downto 0);

    -- S2: scale freqs by master Scale (registered raw products).
    signal s2_fx_eff : unsigned(19 downto 0);  -- 10×10
    signal s2_fy_eff : unsigned(19 downto 0);
    signal s2_fd_eff : unsigned(19 downto 0);
    signal s2_x      : unsigned(11 downto 0);
    signal s2_y      : unsigned(11 downto 0);
    signal s2_sum    : unsigned(12 downto 0);
    signal s2_dif    : signed(12 downto 0);

    -- S3: x * fx_eff_byte etc.  Reduce fx_eff to 10-bit.
    signal s3_arg_x  : unsigned(21 downto 0);  -- 12 × 10
    signal s3_arg_y  : unsigned(21 downto 0);
    signal s3_arg_s  : unsigned(22 downto 0);  -- 13 × 10
    signal s3_arg_d  : signed(23 downto 0);    -- signed(13) × signed(11)

    -- S4: add phase + reduce to 10-bit angle for each of 4 waves.
    signal s4_ang_x : unsigned(9 downto 0);
    signal s4_ang_y : unsigned(9 downto 0);
    signal s4_ang_s : unsigned(9 downto 0);
    signal s4_ang_d : unsigned(9 downto 0);

    -- S5: wave eval per axis.  Signed 11-bit.
    signal s5_wx : signed(10 downto 0);
    signal s5_wy : signed(10 downto 0);
    signal s5_ws : signed(10 downto 0);
    signal s5_wd : signed(10 downto 0);

    -- S6: half-sum 1: wx+wy, ws+wd (each can be ±2046).
    signal s6_xy : signed(11 downto 0);
    signal s6_sd : signed(11 downto 0);

    -- S7: total = xy+sd (or just xy when 3-wave).
    signal s7_total : signed(12 downto 0);

    -- S8: bias into 0..1023 by adding ~2048 then >>2.
    signal s8_idx10 : unsigned(9 downto 0);

    -- S9: hue angle (full 10-bit rotation) and Y-magnitude.
    signal s9_hue  : unsigned(9 downto 0);
    signal s9_ymag : unsigned(9 downto 0);

    -- S10: sin(hue) and cos(hue) from quarter-LUT (signed 11-bit ±511).
    signal s10_sin  : signed(10 downto 0);
    signal s10_cos  : signed(10 downto 0);
    signal s10_ymag : unsigned(9 downto 0);

    -- S11: derive Y from plasma magnitude; U/V centered + sin/cos scaled.
    signal s11_y_mul : unsigned(19 downto 0);
    signal s11_u     : unsigned(9 downto 0);
    signal s11_v     : unsigned(9 downto 0);
    signal s11_ymod  : unsigned(6 downto 0);

    -- S12: extract Y; add ymod for texture (clamped).
    signal s12_y : unsigned(9 downto 0);
    signal s12_u : unsigned(9 downto 0);
    signal s12_v : unsigned(9 downto 0);

    -- S13..S15 slack stages + final invert.
    signal s13_y : unsigned(9 downto 0);
    signal s13_u : unsigned(9 downto 0);
    signal s13_v : unsigned(9 downto 0);
    signal s14_y : unsigned(9 downto 0);
    signal s14_u : unsigned(9 downto 0);
    signal s14_v : unsigned(9 downto 0);
    signal s15_y : unsigned(9 downto 0);
    signal s15_u : unsigned(9 downto 0);
    signal s15_v : unsigned(9 downto 0);
    signal s16_y : unsigned(9 downto 0);
    signal s16_u : unsigned(9 downto 0);
    signal s16_v : unsigned(9 downto 0);

    function wave_eval(ang : unsigned(9 downto 0); is_tri : std_logic)
        return signed is
        variable v_q          : unsigned(1 downto 0);
        variable v_idx        : integer range 0 to 63;
        variable v_idx_mirror : integer range 0 to 63;
        variable v_mag        : unsigned(8 downto 0);
        variable v_byte       : unsigned(7 downto 0);
        variable v_2x         : signed(10 downto 0);
        variable v_signed     : signed(10 downto 0);
        variable v_tri        : signed(10 downto 0);
    begin
        v_q          := ang(9 downto 8);
        v_idx        := to_integer(ang(7 downto 2));
        v_idx_mirror := 63 - v_idx;
        v_byte       := ang(7 downto 0);
        v_2x         := signed(resize(v_byte, 11)) sll 1;
        if is_tri = '0' then
            case v_q is
                when "00"   => v_mag := C_SIN_QUAD(v_idx);
                when "01"   => v_mag := C_SIN_QUAD(v_idx_mirror);
                when "10"   => v_mag := C_SIN_QUAD(v_idx);
                when others => v_mag := C_SIN_QUAD(v_idx_mirror);
            end case;
            if v_q(1) = '0' then v_signed := signed(resize(v_mag, 11));
            else v_signed := -signed(resize(v_mag, 11));
            end if;
            return v_signed;
        else
            case v_q is
                when "00"   => v_tri := v_2x;
                when "01"   => v_tri := to_signed(511, 11) - v_2x;
                when "10"   => v_tri := -v_2x;
                when others => v_tri := -to_signed(511, 11) + v_2x;
            end case;
            return v_tri;
        end if;
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
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                fx_r     <= unsigned(registers_in(0));
                fy_r     <= unsigned(registers_in(1));
                fd_r     <= unsigned(registers_in(2));
                speed_r  <= unsigned(registers_in(3));
                hue_r    <= unsigned(registers_in(4));
                bright_r <= unsigned(registers_in(5));
                lumamod_r <= registers_in(6)(0);
                banded_r <= registers_in(6)(1);
                three_r  <= registers_in(6)(2);
                tri_r    <= registers_in(6)(3);
                invert_r <= registers_in(6)(4);
                scale_r  <= unsigned(registers_in(7));

                -- Speed: scaled down 4× for finer control.  Knob 0..1023 →
                -- frame phase advance 0..256.  Full cycle ~256 frames at max.
                t_phase_r <= t_phase_r +
                             ("000000" & "00" &
                              unsigned(registers_in(3)(9 downto 2)));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pidx    : integer range 0 to 7;
        variable v_total   : signed(13 downto 0);
        variable v_y_total : unsigned(10 downto 0);
        variable v_u_s     : signed(10 downto 0);
        variable v_v_s     : signed(10 downto 0);
        variable v_lm_mul  : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: x+y, x-y
            s1_x <= s0_x;
            s1_y <= s0_y;
            s1_sum <= ('0' & s0_x) + ('0' & s0_y);
            s1_dif <= signed('0' & std_logic_vector(s0_x)) -
                      signed('0' & std_logic_vector(s0_y));

            -- S2: scale freqs by scale_r (10 × 10 = 20).
            s2_fx_eff <= fx_r * scale_r;
            s2_fy_eff <= fy_r * scale_r;
            s2_fd_eff <= fd_r * scale_r;
            s2_x      <= s1_x;
            s2_y      <= s1_y;
            s2_sum    <= s1_sum;
            s2_dif    <= s1_dif;

            -- S3: pixel × effective freq.  Broader range: take bits [16:7]
            -- of fx_eff (effectively doubling the visible freq versus
            -- previous [15:6]).
            s3_arg_x <= s2_x * s2_fx_eff(16 downto 7);
            s3_arg_y <= s2_y * s2_fy_eff(16 downto 7);
            s3_arg_s <= s2_sum * s2_fd_eff(16 downto 7);
            s3_arg_d <= s2_dif * signed('0' & std_logic_vector(s2_fd_eff(16 downto 7)));

            -- S4: extract upper bits as angle and add t_phase.  When Luma
            -- Mod is on, the incoming video luma is added as an extra
            -- phase offset — this WARPS the plasma waveform around the
            -- video's brightness contours without ever showing the video
            -- itself.  Plasma colours fill the whole frame; only the
            -- *shape* of the pattern follows the video.
            if lumamod_r = '1' then
                s4_ang_x <= s3_arg_x(16 downto 7) + t_phase_r(9 downto 0) +
                            unsigned(pipe(4).y);
                s4_ang_y <= s3_arg_y(16 downto 7) + t_phase_r(11 downto 2) +
                            unsigned(pipe(4).y);
                s4_ang_s <= s3_arg_s(17 downto 8) + t_phase_r(13 downto 4) +
                            unsigned(pipe(4).y);
                s4_ang_d <= unsigned(std_logic_vector(s3_arg_d(17 downto 8))) +
                            t_phase_r(15 downto 6) +
                            unsigned(pipe(4).y);
            else
                s4_ang_x <= s3_arg_x(16 downto 7) + t_phase_r(9 downto 0);
                s4_ang_y <= s3_arg_y(16 downto 7) + t_phase_r(11 downto 2);
                s4_ang_s <= s3_arg_s(17 downto 8) + t_phase_r(13 downto 4);
                s4_ang_d <= unsigned(std_logic_vector(s3_arg_d(17 downto 8))) +
                            t_phase_r(15 downto 6);
            end if;

            -- S5: wave eval per axis.
            s5_wx <= wave_eval(s4_ang_x, tri_r);
            s5_wy <= wave_eval(s4_ang_y, tri_r);
            s5_ws <= wave_eval(s4_ang_s, tri_r);
            s5_wd <= wave_eval(s4_ang_d, tri_r);

            -- S6: half sums.
            s6_xy <= resize(s5_wx, 12) + resize(s5_wy, 12);
            if three_r = '0' then
                s6_sd <= resize(s5_ws, 12) + resize(s5_wd, 12);
            else
                s6_sd <= resize(s5_ws, 12);  -- skip wd in 3-wave mode
            end if;

            -- S7: total
            s7_total <= resize(s6_xy, 13) + resize(s6_sd, 13);

            -- S8: bias.  Range ±2046, shift+offset to 0..1023.
            v_total := resize(s7_total, 14) + to_signed(2048, 14);
            -- v_total now 0..4095, take >>2 → 0..1023.
            if v_total < 0 then
                s8_idx10 <= (others => '0');
            elsif v_total > 4092 then
                s8_idx10 <= (others => '1');
            else
                s8_idx10 <= unsigned(std_logic_vector(v_total(11 downto 2)));
            end if;

            -- S9: continuous hue rotation.  Hue angle = plasma value + hue_r.
            -- Y magnitude comes from the plasma value's lower band so
            -- brightness still tracks the plasma waveform — unless Luma Mod
            -- is enabled, in which case Y is replaced with the input video
            -- luma at the output stage.
            s9_hue <= s8_idx10 + hue_r;
            if banded_r = '1' then
                -- Banded: quantize plasma to coarse bands.
                s9_ymag <= s8_idx10(9 downto 7) & "0000000";
            else
                s9_ymag <= s8_idx10;
            end if;

            -- S10: sin/cos lookup for chroma vector.  Quarter-LUT result is
            -- signed 11-bit.  Cosine = sin(angle + 256).
            s10_sin  <= wave_eval(s9_hue, '0');
            s10_cos  <= wave_eval(s9_hue + to_unsigned(256, 10), '0');
            s10_ymag <= s9_ymag;

            -- S11: U = 512 + cos>>1, V = 512 + sin>>1 (chroma swing ±255
            -- around neutral). Y comes from brightness × plasma magnitude.
            s11_y_mul <= s10_ymag * bright_r;
            v_u_s := to_signed(512, 11) + resize(shift_right(s10_cos, 1), 11);
            v_v_s := to_signed(512, 11) + resize(shift_right(s10_sin, 1), 11);
            s11_u <= unsigned(std_logic_vector(v_u_s(9 downto 0)));
            s11_v <= unsigned(std_logic_vector(v_v_s(9 downto 0)));
            s11_ymod  <= (others => '0');

            -- S12: extract Y.
            s12_y <= s11_y_mul(19 downto 10);
            s12_u <= s11_u;
            s12_v <= s11_v;

            -- S13/S14/S15 slack
            s13_y <= s12_y; s13_u <= s12_u; s13_v <= s12_v;
            s14_y <= s13_y; s14_u <= s13_u; s14_v <= s13_v;
            s15_y <= s14_y; s15_u <= s14_u; s15_v <= s14_v;

            -- S16: invert.  Luma Mod now warps the plasma SHAPE upstream
            -- (S4 angle injection), so the output Y is always pure plasma —
            -- you never see the source video, only its influence on the
            -- pattern's contours.
            if invert_r = '1' then
                s16_y <= to_unsigned(1023, 10) - s15_y;
            else
                s16_y <= s15_y;
            end if;
            s16_u <= s15_u;
            s16_v <= s15_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s16_y);
    data_out.u       <= std_logic_vector(s16_u);
    data_out.v       <= std_logic_vector(s16_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture plasma;
