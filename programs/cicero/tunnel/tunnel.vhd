-- Tunnel: hyperspace zoom-into-checker effect.
--
-- For each pixel, compute Manhattan ring index (|dx|+|dy|) >> bits and an
-- angular slice index ((dx + 1920 + dy) >> bits) and XOR them to produce a
-- checkered "tunnel" pattern.  An animated phase added to the ring index
-- creates an outward-zoom motion.  An optional twist offset rotates the
-- angular slices for a corkscrew look.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Ring Freq      (rotary 1 — band thickness)
--   registers_in(1) = Slice Freq     (rotary 2 — angular cell width)
--   registers_in(2) = Speed          (rotary 3 — animation rate)
--   registers_in(3) = Twist          (rotary 4 — corkscrew offset)
--   registers_in(4) = Vignette       (rotary 5 — center darkness)
--   registers_in(5) = Brightness     (rotary 6)
--   registers_in(6) = Switches:
--     b0   Pattern  (0=Checker / 1=Bars)
--     b1   Direction (0=Zoom out / 1=Zoom in)
--     b2   Palette  (0=Mono / 1=Spectrum)
--     b3   Strobe   (0=Off / 1=alternate frames flash)
--     b4   Invert
--   registers_in(7) = Tunnel Depth (slider, KEY — overall scale of rings)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture tunnel of program_top is

    constant LATENCY : natural := 12;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal ring_freq_r : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal slice_freq_r: unsigned(9 downto 0) := to_unsigned(256, 10);
    signal speed_r     : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal twist_r     : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal vign_r      : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bright_r    : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal depth_r     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal pattern_r   : std_logic := '0';
    signal dir_r       : std_logic := '0';
    signal palette_r   : std_logic := '0';
    signal strobe_r    : std_logic := '0';
    signal invert_r    : std_logic := '0';

    signal anim_phase_r : unsigned(15 downto 0) := (others => '0');

    -- 8-color palette
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
    signal s0_dx : signed(12 downto 0);
    signal s0_dy : signed(12 downto 0);

    signal s1_adx : unsigned(11 downto 0);
    signal s1_ady : unsigned(11 downto 0);
    signal s1_sdx : signed(12 downto 0);   -- forwarded for slice
    signal s1_sdy : signed(12 downto 0);

    -- S2: r_manhattan = adx + ady; slice_raw = dx + dy + offset
    signal s2_r       : unsigned(12 downto 0);
    signal s2_slice   : signed(13 downto 0);

    -- S3: ring index from r * ring_freq, slice index from slice_raw * slice_freq
    signal s3_ring_mul : unsigned(22 downto 0);
    signal s3_slice_mul: signed(23 downto 0);

    -- S4: extract ring index + animate, slice index + twist
    signal s4_ring    : unsigned(11 downto 0);
    signal s4_slice   : unsigned(11 downto 0);
    signal s4_vign_in : unsigned(11 downto 0);  -- forwarded r for vignette

    -- S5: ring XOR slice + bit extract → checker on/off
    signal s5_checker : std_logic;
    signal s5_slice   : unsigned(11 downto 0);
    signal s5_ring    : unsigned(11 downto 0);
    signal s5_vign_in : unsigned(11 downto 0);

    -- S6: combine to brightness; pattern_r=0 uses checker; =1 uses ring stripes
    signal s6_b : unsigned(9 downto 0);
    signal s6_idx : unsigned(2 downto 0);
    signal s6_vign_in : unsigned(11 downto 0);

    -- S7: vignette darken near edges (Manhattan radius too large = dark)
    signal s7_b : unsigned(9 downto 0);
    signal s7_idx : unsigned(2 downto 0);

    -- S8: palette lookup
    signal s8_y : unsigned(9 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);
    signal s8_b : unsigned(9 downto 0);

    -- S9: scale Y by brightness (registered raw mul)
    signal s9_ymul : unsigned(19 downto 0);
    signal s9_u    : unsigned(9 downto 0);
    signal s9_v    : unsigned(9 downto 0);
    signal s9_b    : unsigned(9 downto 0);

    -- S10: extract + center chroma
    signal s10_y_pre  : unsigned(9 downto 0);
    signal s10_u_cent : signed(11 downto 0);
    signal s10_v_cent : signed(11 downto 0);
    signal s10_b      : unsigned(9 downto 0);

    -- S11: chroma scale + invert + output
    signal s11_y : unsigned(9 downto 0);
    signal s11_u : unsigned(9 downto 0);
    signal s11_v : unsigned(9 downto 0);

    function sat10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- ========================================================================
    -- Position + parameters + animation phase
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

                ring_freq_r  <= unsigned(registers_in(0));
                slice_freq_r <= unsigned(registers_in(1));
                speed_r      <= unsigned(registers_in(2));
                twist_r      <= unsigned(registers_in(3));
                vign_r       <= unsigned(registers_in(4));
                bright_r     <= unsigned(registers_in(5));
                pattern_r    <= registers_in(6)(0);
                dir_r        <= registers_in(6)(1);
                palette_r    <= registers_in(6)(2);
                strobe_r     <= registers_in(6)(3);
                invert_r     <= registers_in(6)(4);
                depth_r      <= unsigned(registers_in(7));

                v_speed := "000" & unsigned(registers_in(2)(9 downto 3));
                if registers_in(6)(1) = '0' then
                    anim_phase_r <= anim_phase_r + ("000000" & v_speed);
                else
                    anim_phase_r <= anim_phase_r - ("000000" & v_speed);
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
        variable v_pidx : integer range 0 to 7;
        variable v_dx_a : signed(12 downto 0);
        variable v_dy_a : signed(12 downto 0);
        variable v_b    : unsigned(9 downto 0);
        variable v_ring : unsigned(11 downto 0);
        variable v_vign_sub : signed(11 downto 0);
        variable v_y_mul : unsigned(19 downto 0);
        variable v_u_mul : signed(22 downto 0);
        variable v_v_mul : signed(22 downto 0);
        variable v_u_out : signed(13 downto 0);
        variable v_v_out : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: dx, dy centered
            s0_dx <= signed(resize(pixel_x, 13)) - to_signed(960, 13);
            s0_dy <= signed(resize(pixel_y, 13)) - to_signed(540, 13);

            -- S1: abs of dx, dy.  Forward signed for slice calc.
            v_dx_a := s0_dx;
            if v_dx_a < 0 then v_dx_a := -v_dx_a; end if;
            s1_adx <= unsigned(std_logic_vector(v_dx_a(11 downto 0)));
            v_dy_a := s0_dy;
            if v_dy_a < 0 then v_dy_a := -v_dy_a; end if;
            s1_ady <= unsigned(std_logic_vector(v_dy_a(11 downto 0)));
            s1_sdx <= s0_dx;
            s1_sdy <= s0_dy;

            -- S2: Manhattan radius + slice raw = dx - dy with twist offset
            s2_r <= resize(s1_adx, 13) + resize(s1_ady, 13);
            s2_slice <= resize(s1_sdx, 14) - resize(s1_sdy, 14)
                        + signed(resize(twist_r, 14));

            -- S3: scale by per-axis frequencies (10×12 = 22, 10×13 = 23)
            s3_ring_mul  <= s2_r * ring_freq_r;
            s3_slice_mul <= resize(s2_slice *
                            signed('0' & std_logic_vector(slice_freq_r)), 24);

            -- S4: extract ring (drop low + animate) and slice indices
            s4_ring  <= s3_ring_mul(18 downto 7) + anim_phase_r(15 downto 4);
            s4_slice <= unsigned(std_logic_vector(s3_slice_mul(19 downto 8)));
            s4_vign_in <= s2_r(11 downto 0);

            -- S5: ring XOR slice; take a single bit for checker on/off
            s5_checker <= s4_ring(7) xor s4_slice(7);
            s5_ring    <= s4_ring;
            s5_slice   <= s4_slice;
            s5_vign_in <= s4_vign_in;

            -- S6: pattern selection
            if pattern_r = '0' then
                if s5_checker = '1' then
                    s6_b <= to_unsigned(900, 10);
                else
                    s6_b <= to_unsigned(120, 10);
                end if;
            else
                -- Bar pattern: brightness from low bits of ring only
                s6_b <= "00" & s5_ring(7 downto 0);
            end if;
            s6_idx <= s5_ring(11 downto 9);
            s6_vign_in <= s5_vign_in;

            -- S7: vignette = darken pixel when manhattan radius > threshold
            -- (depth_r controls how far the tunnel "reaches").
            -- vign_sub = max(0, r - 4*depth_r) * vign_knob/16 → subtract from b
            v_vign_sub := signed(resize(s6_vign_in, 12)) -
                          signed(resize(depth_r & "00", 12));
            if v_vign_sub < 0 then v_vign_sub := to_signed(0, 12); end if;
            -- Scale by vign_r/256
            -- v_vign_sub max ≈ 4095 - 4092 = 3; we want larger contribution
            v_b := s6_b;
            if v_b > to_unsigned(20, 10) and v_vign_sub > 0 then
                if unsigned(std_logic_vector(v_vign_sub(9 downto 0))) < v_b then
                    v_b := v_b - unsigned(std_logic_vector(v_vign_sub(9 downto 0)));
                else
                    v_b := (others => '0');
                end if;
            end if;
            -- Strobe: dim alternate frames
            if strobe_r = '1' and frame_count(0) = '1' then
                v_b := "00" & v_b(9 downto 2);  -- /4
            end if;
            s7_b <= v_b;
            s7_idx <= s6_idx;

            -- S8: palette
            if palette_r = '0' then
                s8_y <= s7_b;
                s8_u <= to_unsigned(512, 10);
                s8_v <= to_unsigned(512, 10);
            else
                v_pidx := to_integer(s7_idx);
                s8_y <= C_PAL_Y(v_pidx);
                s8_u <= C_PAL_U(v_pidx);
                s8_v <= C_PAL_V(v_pidx);
            end if;
            s8_b <= s7_b;

            -- S9: scale Y by brightness (registered raw mul)
            s9_ymul <= s8_y * s8_b;
            s9_u    <= s8_u;
            s9_v    <= s8_v;
            s9_b    <= s8_b;

            -- S10: extract + center chroma
            s10_y_pre  <= s9_ymul(19 downto 10);
            s10_u_cent <= signed(resize(s9_u, 12)) - to_signed(512, 12);
            s10_v_cent <= signed(resize(s9_v, 12)) - to_signed(512, 12);
            s10_b      <= s9_b;

            -- S11: brightness knob to Y, chroma scale, invert
            v_y_mul := s10_y_pre * bright_r;
            v_u_mul := s10_u_cent * signed('0' & std_logic_vector(s10_b));
            v_v_mul := s10_v_cent * signed('0' & std_logic_vector(s10_b));
            v_u_out := resize(shift_right(v_u_mul, 10), 14) + to_signed(512, 14);
            v_v_out := resize(shift_right(v_v_mul, 10), 14) + to_signed(512, 14);
            if invert_r = '1' then
                s11_y <= to_unsigned(1023, 10) - v_y_mul(19 downto 10);
            else
                s11_y <= v_y_mul(19 downto 10);
            end if;
            s11_u <= sat10(v_u_out);
            s11_v <= sat10(v_v_out);
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s11_y);
    data_out.u       <= std_logic_vector(s11_u);
    data_out.v       <= std_logic_vector(s11_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture tunnel;
