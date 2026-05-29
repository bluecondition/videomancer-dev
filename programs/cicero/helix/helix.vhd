-- Helix: rotating DNA double helix viewed edge-on.
--
-- Two strands wind around a vertical centerline.  Strand A's x position
-- at row y is centre + amp * sin(y * pitch + phase); strand B is the same
-- with phase + π (so it mirrors strand A).  Horizontal "rungs" connect
-- the strands at every Rung Spacing rows.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Strand Width   (rotary 1)
--   registers_in(1) = Helix Pitch    (rotary 2 — vertical wavelength)
--   registers_in(2) = Rotation Speed (rotary 3)
--   registers_in(3) = Rung Spacing   (rotary 4)
--   registers_in(4) = Rung Width     (rotary 5)
--   registers_in(5) = Brightness     (rotary 6)
--   registers_in(6) = Switches:
--     b0   Colors  (0=Mono / 1=Bi-Color strands)
--     b1   Background (0=Dark / 1=Glow gradient)
--     b2   Rungs    (0=Off / 1=On)
--     b3   Tint     (0=Cyan-Magenta / 1=Red-Blue)
--     b4   Invert
--   registers_in(7) = Amplitude (slider, KEY — spread of strands)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture helix of program_top is

    constant LATENCY : natural := 16;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal width_r  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal pitch_r  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal speed_r  : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal rspace_r : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal rwid_r   : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal bright_r : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal amp_r    : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal bicolor_r : std_logic := '1';
    signal bg_r      : std_logic := '0';
    signal rungs_r   : std_logic := '1';
    signal tint_r    : std_logic := '0';
    signal invert_r  : std_logic := '0';

    signal phase_r : unsigned(11 downto 0) := (others => '0');

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

    -- ------------------------------------------------------------------
    -- Pipeline signals
    -- ------------------------------------------------------------------
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    -- S1: y * pitch (12×10 = 22 bits)
    signal s1_yp : unsigned(21 downto 0);
    signal s1_x  : unsigned(11 downto 0);
    signal s1_y  : unsigned(11 downto 0);

    -- S2: angle12 = yp[19:8] + phase ; keep mod rung position
    signal s2_ang  : unsigned(11 downto 0);
    signal s2_x    : unsigned(11 downto 0);
    signal s2_y    : unsigned(11 downto 0);

    -- S3: 10-bit angle for sine
    signal s3_ang_a : unsigned(9 downto 0);
    signal s3_ang_b : unsigned(9 downto 0);
    signal s3_x     : unsigned(11 downto 0);
    signal s3_y     : unsigned(11 downto 0);

    -- S4: sine eval signed 10
    signal s4_sa : signed(10 downto 0);
    signal s4_sb : signed(10 downto 0);
    signal s4_x  : unsigned(11 downto 0);
    signal s4_y  : unsigned(11 downto 0);

    -- S5: sin × amp (signed 11 × unsigned 10 = 22)
    signal s5_pa : signed(21 downto 0);
    signal s5_pb : signed(21 downto 0);
    signal s5_x  : unsigned(11 downto 0);
    signal s5_y  : unsigned(11 downto 0);

    -- S6: extract offsets (signed 12), shift right 9 = amp/512 = approx max ±492
    signal s6_offa : signed(11 downto 0);
    signal s6_offb : signed(11 downto 0);
    signal s6_x    : unsigned(11 downto 0);
    signal s6_y    : unsigned(11 downto 0);

    -- S7: center_x for each strand = 960 + offset
    signal s7_xa : signed(13 downto 0);
    signal s7_xb : signed(13 downto 0);
    signal s7_x  : unsigned(11 downto 0);
    signal s7_y  : unsigned(11 downto 0);

    -- S8: dx_a, dx_b = pixel_x - strand_x
    signal s8_dxa : signed(13 downto 0);
    signal s8_dxb : signed(13 downto 0);
    signal s8_x   : unsigned(11 downto 0);
    signal s8_y   : unsigned(11 downto 0);

    -- S9: abs(dx)
    signal s9_adxa : unsigned(12 downto 0);
    signal s9_adxb : unsigned(12 downto 0);
    signal s9_x    : unsigned(11 downto 0);
    signal s9_y    : unsigned(11 downto 0);

    -- S10: strand brightness
    signal s10_ba : unsigned(9 downto 0);
    signal s10_bb : unsigned(9 downto 0);
    signal s10_y  : unsigned(11 downto 0);
    -- Pre-sorted strand x positions (left = min, right = max) so S11's
    -- in-xrange check is just two compares against constants, no mux chain.
    signal s10_xmin : signed(13 downto 0);
    signal s10_xmax : signed(13 downto 0);
    signal s10_x  : unsigned(11 downto 0);

    -- S11: rung brightness (if y near multiple of rspace, between xa..xb)
    -- + winner index (A=0, B=1)
    signal s11_ba : unsigned(9 downto 0);
    signal s11_bb : unsigned(9 downto 0);
    signal s11_rung : unsigned(9 downto 0);
    signal s11_idx  : unsigned(1 downto 0);  -- 00 = A strand, 01 = B, 10 = rung
    signal s11_bg   : unsigned(9 downto 0);   -- background brightness from y

    -- S12: pick winner brightness + assign color
    signal s12_y_pre : unsigned(9 downto 0);
    signal s12_u     : unsigned(9 downto 0);
    signal s12_v     : unsigned(9 downto 0);

    -- S13: scale Y by brightness * bright knob (registered)
    signal s13_ymul : unsigned(19 downto 0);
    signal s13_u    : unsigned(9 downto 0);
    signal s13_v    : unsigned(9 downto 0);

    -- S14: extract + chroma center + raw chroma mul
    signal s14_y_pre  : unsigned(9 downto 0);
    signal s14_u_cent : signed(11 downto 0);
    signal s14_v_cent : signed(11 downto 0);
    signal s14_b      : unsigned(9 downto 0);  -- placeholder for chroma fade

    -- S15: final out
    signal s15_y : unsigned(9 downto 0);
    signal s15_u : unsigned(9 downto 0);
    signal s15_v : unsigned(9 downto 0);

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
    -- Position + parameter latching + per-frame phase advance.
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_drift  : unsigned(9 downto 0);
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

                width_r  <= unsigned(registers_in(0));
                pitch_r  <= unsigned(registers_in(1));
                speed_r  <= unsigned(registers_in(2));
                rspace_r <= unsigned(registers_in(3));
                rwid_r   <= unsigned(registers_in(4));
                bright_r <= unsigned(registers_in(5));

                bicolor_r <= registers_in(6)(0);
                bg_r      <= registers_in(6)(1);
                rungs_r   <= registers_in(6)(2);
                tint_r    <= registers_in(6)(3);
                invert_r  <= registers_in(6)(4);

                amp_r <= unsigned(registers_in(7));

                v_drift := "00000" & unsigned(registers_in(2)(9 downto 5));
                phase_r <= phase_r + ("00" & v_drift);
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_dx : signed(13 downto 0);
        variable v_rung_pos : unsigned(11 downto 0);
        variable v_rung_dist : unsigned(11 downto 0);
        variable v_in_rung : boolean;
        variable v_in_xrange : boolean;
        variable v_max : unsigned(9 downto 0);
        variable v_max_a : unsigned(9 downto 0);
        variable v_max_b : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: y * pitch (12×10=22)
            s1_yp <= s0_y * pitch_r;
            s1_x  <= s0_x;
            s1_y  <= s0_y;

            -- S2: angle = yp[19:8] + phase
            s2_ang <= s1_yp(19 downto 8) + phase_r;
            s2_x   <= s1_x;
            s2_y   <= s1_y;

            -- S3: 10-bit angles (A and B offset by 512 = π)
            s3_ang_a <= s2_ang(9 downto 0);
            s3_ang_b <= s2_ang(9 downto 0) + to_unsigned(512, 10);
            s3_x     <= s2_x;
            s3_y     <= s2_y;

            -- S4: sines
            s4_sa <= quad_sin(s3_ang_a);
            s4_sb <= quad_sin(s3_ang_b);
            s4_x  <= s3_x;
            s4_y  <= s3_y;

            -- S5: sin × amp (signed 11 × signed 11 = signed 22)
            s5_pa <= s4_sa * signed('0' & std_logic_vector(amp_r));
            s5_pb <= s4_sb * signed('0' & std_logic_vector(amp_r));
            s5_x  <= s4_x;
            s5_y  <= s4_y;

            -- S6: extract offsets (>> 9; amp 1023 * sin 491 / 512 ≈ ±980)
            s6_offa <= resize(shift_right(s5_pa, 9), 12);
            s6_offb <= resize(shift_right(s5_pb, 9), 12);
            s6_x    <= s5_x;
            s6_y    <= s5_y;

            -- S7: strand_x = 960 + offset
            s7_xa <= to_signed(960, 14) + resize(s6_offa, 14);
            s7_xb <= to_signed(960, 14) + resize(s6_offb, 14);
            s7_x  <= s6_x;
            s7_y  <= s6_y;

            -- S8: dx
            s8_dxa <= signed(resize(s7_x, 14)) - s7_xa;
            s8_dxb <= signed(resize(s7_x, 14)) - s7_xb;
            s8_x   <= s7_x;
            s8_y   <= s7_y;

            -- S9: abs(dx)
            v_dx := s8_dxa;
            if v_dx < 0 then v_dx := -v_dx; end if;
            s9_adxa <= unsigned(std_logic_vector(v_dx(12 downto 0)));
            v_dx := s8_dxb;
            if v_dx < 0 then v_dx := -v_dx; end if;
            s9_adxb <= unsigned(std_logic_vector(v_dx(12 downto 0)));
            s9_x   <= s8_x;
            s9_y   <= s8_y;

            -- S10: strand brightness = max(0, width - adx)
            if s9_adxa < ("000" & width_r) then
                s10_ba <= resize(("000" & width_r) - s9_adxa, 10);
            else
                s10_ba <= (others => '0');
            end if;
            if s9_adxb < ("000" & width_r) then
                s10_bb <= resize(("000" & width_r) - s9_adxb, 10);
            else
                s10_bb <= (others => '0');
            end if;
            s10_y  <= s9_y;
            -- Pre-sort xa, xb into min/max so S11 only needs 2 compares.
            if s7_xa <= s7_xb then
                s10_xmin <= s7_xa;
                s10_xmax <= s7_xb;
            else
                s10_xmin <= s7_xb;
                s10_xmax <= s7_xa;
            end if;
            s10_x  <= s9_x;

            -- S11: rung calculation: position within rspace cycle
            -- Power-of-2 spacing avoids combinational divider.  Top bits of
            -- rspace_r select spacing 32/64/128/256/512.
            case to_integer(rspace_r(9 downto 8)) is
                when 0 =>
                    v_rung_pos := resize(s10_y(5 downto 0), 12); -- 64 px
                when 1 =>
                    v_rung_pos := resize(s10_y(6 downto 0), 12); -- 128 px
                when 2 =>
                    v_rung_pos := resize(s10_y(7 downto 0), 12); -- 256 px
                when others =>
                    v_rung_pos := resize(s10_y(8 downto 0), 12); -- 512 px
            end case;
            v_rung_dist := v_rung_pos;

            -- Determine if pixel_x is between strands (sorted at S10).
            v_in_xrange := signed(resize(s10_x, 14)) >= s10_xmin and
                           signed(resize(s10_x, 14)) <= s10_xmax;
            v_in_rung := v_rung_dist < ("00" & rwid_r);

            if rungs_r = '1' and v_in_rung and v_in_xrange then
                -- Brightness scaled by inverse rung distance
                s11_rung <= resize(("00" & rwid_r) - v_rung_dist, 10);
            else
                s11_rung <= (others => '0');
            end if;
            s11_ba <= s10_ba;
            s11_bb <= s10_bb;

            -- Background: gradient by y if enabled
            if bg_r = '1' then
                s11_bg <= "00" & s10_y(11 downto 4);  -- 0..255 ramp
            else
                s11_bg <= to_unsigned(40, 10);
            end if;

            -- Pick highest brightness for index
            if s10_ba >= s10_bb then
                s11_idx <= "00";
            else
                s11_idx <= "01";
            end if;

            -- S12: choose winner brightness via parallel pairwise max tree
            -- to keep critical path short.
            if s11_bg >= s11_ba then v_max_a := s11_bg;
            else                     v_max_a := s11_ba; end if;
            if s11_bb >= s11_rung then v_max_b := s11_bb;
            else                       v_max_b := s11_rung; end if;
            if v_max_a >= v_max_b then v_max := v_max_a;
            else                       v_max := v_max_b; end if;
            s12_y_pre <= v_max;

            -- Colors: A strand cyan, B strand magenta (or red/blue if tint=1)
            if tint_r = '0' then
                -- Cyan / Magenta
                if s11_rung > s11_ba and s11_rung > s11_bb then
                    -- rung = white-ish
                    s12_u <= to_unsigned(512, 10);
                    s12_v <= to_unsigned(512, 10);
                elsif (bicolor_r = '0') then
                    -- Mono cyan
                    s12_u <= to_unsigned(680, 10);
                    s12_v <= to_unsigned(380, 10);
                elsif s11_idx = "00" then
                    -- Strand A = cyan
                    s12_u <= to_unsigned(700, 10);
                    s12_v <= to_unsigned(360, 10);
                else
                    -- Strand B = magenta
                    s12_u <= to_unsigned(720, 10);
                    s12_v <= to_unsigned(700, 10);
                end if;
            else
                -- Red / Blue
                if s11_rung > s11_ba and s11_rung > s11_bb then
                    s12_u <= to_unsigned(512, 10);
                    s12_v <= to_unsigned(512, 10);
                elsif (bicolor_r = '0') then
                    s12_u <= to_unsigned(280, 10);
                    s12_v <= to_unsigned(280, 10);
                elsif s11_idx = "00" then
                    -- Red
                    s12_u <= to_unsigned(280, 10);
                    s12_v <= to_unsigned(820, 10);
                else
                    -- Blue
                    s12_u <= to_unsigned(800, 10);
                    s12_v <= to_unsigned(300, 10);
                end if;
            end if;

            -- S13: scale Y by overall brightness knob (registered mul)
            s13_ymul <= s12_y_pre * bright_r;
            s13_u    <= s12_u;
            s13_v    <= s12_v;

            -- S14: extract + center chroma
            s14_y_pre  <= s13_ymul(19 downto 10);
            s14_u_cent <= signed(resize(s13_u, 12)) - to_signed(512, 12);
            s14_v_cent <= signed(resize(s13_v, 12)) - to_signed(512, 12);
            s14_b      <= s13_ymul(19 downto 10);
        end if;
    end process p_pipe;

    -- ========================================================================
    -- Output stage (S15): chroma scale + invert
    -- ========================================================================
    p_out : process(clk)
        variable v_u_mul : signed(22 downto 0);
        variable v_v_mul : signed(22 downto 0);
        variable v_u_out : signed(13 downto 0);
        variable v_v_out : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            v_u_mul := s14_u_cent * signed('0' & std_logic_vector(s14_b));
            v_v_mul := s14_v_cent * signed('0' & std_logic_vector(s14_b));
            v_u_out := resize(shift_right(v_u_mul, 10), 14) + to_signed(512, 14);
            v_v_out := resize(shift_right(v_v_mul, 10), 14) + to_signed(512, 14);
            if invert_r = '1' then
                s15_y <= to_unsigned(1023, 10) - s14_y_pre;
            else
                s15_y <= s14_y_pre;
            end if;
            s15_u <= sat10(v_u_out);
            s15_v <= sat10(v_v_out);
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s15_y);
    data_out.u       <= std_logic_vector(s15_u);
    data_out.v       <= std_logic_vector(s15_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture helix;
