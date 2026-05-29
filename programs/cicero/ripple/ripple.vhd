-- Ripple: sinusoidal horizontal displacement per scanline.
--
-- Each row reads from a horizontally-displaced position in its own line
-- buffer.  Displacement = amp * sin(y * freq + phase), where phase
-- advances per frame.  Makes the image undulate as if seen through a
-- ripple in water.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Wave Freq     (rotary 1 — vertical wavelength)
--   registers_in(1) = Anim Speed    (rotary 2 — phase scroll per frame)
--   registers_in(2) = Phase Offset  (rotary 3 — initial phase)
--   registers_in(3) = Vert Wobble   (rotary 4 — secondary vertical mod)
--   registers_in(4) = Tint          (rotary 5)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Mode      (0=Horizontal / 1=Combined H+V)
--     b1   Direction (0=Outward / 1=Inward)
--     b2   Color     (0=Color / 1=Mono)
--     b3   Tear      (0=Off / 1=Random per-row jitter)
--     b4   Invert
--   registers_in(7) = Amplitude (slider, KEY — max H offset in pixels)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture ripple of program_top is

    constant LATENCY : natural := 8;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal field_flip_r : std_logic := '0';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal freq_r     : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal speed_r    : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal poff_r     : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal vert_r     : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal tint_r     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r   : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal amp_r      : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal mode_r     : std_logic := '0';
    signal dir_r      : std_logic := '0';
    signal mono_r     : std_logic := '0';
    signal tear_r     : std_logic := '0';
    signal invert_r   : std_logic := '0';

    -- Animation phase (frame-scrolling)
    signal anim_phase_r : unsigned(11 downto 0) := (others => '0');

    -- LFSR for tear jitter
    signal lfsr_r : unsigned(15 downto 0) := to_unsigned(54321, 16);

    -- Pre-computed per-row offset (in pixels, signed 10)
    signal row_offset_r : signed(10 downto 0) := (others => '0');

    -- Line buffer
    signal lb_ab        : std_logic;
    signal lb_wr_addr   : unsigned(10 downto 0);
    signal lb_rd_addr   : unsigned(10 downto 0);
    signal lb_y_out     : std_logic_vector(9 downto 0);
    signal lb_u_out     : std_logic_vector(9 downto 0);
    signal lb_v_out     : std_logic_vector(9 downto 0);

    -- 64-entry quarter sine LUT 0..127
    type t_quad is array (0 to 63) of unsigned(7 downto 0);
    constant C_SIN_QUAD : t_quad := (
        to_unsigned(  0, 8), to_unsigned(  3, 8), to_unsigned(  6, 8), to_unsigned( 10, 8),
        to_unsigned( 13, 8), to_unsigned( 16, 8), to_unsigned( 19, 8), to_unsigned( 22, 8),
        to_unsigned( 25, 8), to_unsigned( 28, 8), to_unsigned( 31, 8), to_unsigned( 35, 8),
        to_unsigned( 38, 8), to_unsigned( 41, 8), to_unsigned( 44, 8), to_unsigned( 47, 8),
        to_unsigned( 49, 8), to_unsigned( 52, 8), to_unsigned( 55, 8), to_unsigned( 58, 8),
        to_unsigned( 61, 8), to_unsigned( 63, 8), to_unsigned( 66, 8), to_unsigned( 69, 8),
        to_unsigned( 71, 8), to_unsigned( 74, 8), to_unsigned( 76, 8), to_unsigned( 79, 8),
        to_unsigned( 81, 8), to_unsigned( 83, 8), to_unsigned( 86, 8), to_unsigned( 88, 8),
        to_unsigned( 90, 8), to_unsigned( 92, 8), to_unsigned( 94, 8), to_unsigned( 96, 8),
        to_unsigned( 98, 8), to_unsigned(100, 8), to_unsigned(102, 8), to_unsigned(104, 8),
        to_unsigned(106, 8), to_unsigned(107, 8), to_unsigned(109, 8), to_unsigned(110, 8),
        to_unsigned(112, 8), to_unsigned(113, 8), to_unsigned(114, 8), to_unsigned(116, 8),
        to_unsigned(117, 8), to_unsigned(118, 8), to_unsigned(119, 8), to_unsigned(120, 8),
        to_unsigned(121, 8), to_unsigned(122, 8), to_unsigned(123, 8), to_unsigned(124, 8),
        to_unsigned(124, 8), to_unsigned(125, 8), to_unsigned(125, 8), to_unsigned(126, 8),
        to_unsigned(126, 8), to_unsigned(127, 8), to_unsigned(127, 8), to_unsigned(127, 8));

    -- Pre-computed row_offset_r is registered, used as a stable per-row signal.
    -- This precomputation chain uses a multiply, so we register intermediate
    -- mul result to keep the per-clk critical path short.
    signal row_phase_r  : unsigned(11 downto 0) := (others => '0');
    signal row_sin_r    : signed(8 downto 0) := (others => '0');

    -- Pipeline signals
    signal s0_x : unsigned(11 downto 0);

    -- S1: read addr = x + row_offset
    signal s1_addr : unsigned(10 downto 0);

    -- S2: bram read latency

    -- S3: capture
    signal s3_y : unsigned(9 downto 0);
    signal s3_u : unsigned(9 downto 0);
    signal s3_v : unsigned(9 downto 0);

    -- S4: tint + mono
    signal s4_y : unsigned(9 downto 0);
    signal s4_u : unsigned(9 downto 0);
    signal s4_v : unsigned(9 downto 0);

    -- S5: brightness mul
    signal s5_ymul : unsigned(19 downto 0);
    signal s5_u    : unsigned(9 downto 0);
    signal s5_v    : unsigned(9 downto 0);

    -- S6: extract + invert
    signal s6_y : unsigned(9 downto 0);
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);

    function clamp_addr(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 11); end if;
        if v > 1919 then return to_unsigned(1919, 11); end if;
        return unsigned(std_logic_vector(resize(v, 11)));
    end function;

begin

    -- ========================================================================
    -- Line buffers (Y, U, V each 1 line deep)
    -- ========================================================================
    lb_y_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr,
            i_data    => data_in.y,
            o_data    => lb_y_out
        );
    lb_u_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr,
            i_data    => data_in.u,
            o_data    => lb_u_out
        );
    lb_v_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr,
            i_data    => data_in.v,
            o_data    => lb_v_out
        );

    lb_ab      <= field_flip_r;
    lb_wr_addr <= pixel_x(10 downto 0);
    lb_rd_addr <= s1_addr;

    -- ========================================================================
    -- Position + parameters + per-row offset computation (split into 2 stages)
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_speed  : unsigned(9 downto 0);
        variable v_tap    : std_logic;
        variable v_off_mul : signed(19 downto 0);
        variable v_jitter : signed(10 downto 0);
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

            v_tap := lfsr_r(15) xor lfsr_r(13) xor lfsr_r(12) xor lfsr_r(10);
            lfsr_r <= lfsr_r(14 downto 0) & v_tap;

            -- Stage 1: at every hsync, compute the sin of row*freq + phase.
            -- Stage 2: multiply by amp, register row_offset_r.
            -- This splits the long multiply chain into hsync's separate cycles.

            -- Always-on stage: compute row_phase = pixel_y * freq + anim_phase
            row_phase_r <= resize(pixel_y(9 downto 0) *
                                  resize(freq_r(9 downto 4), 6), 12)
                           + anim_phase_r;
            row_sin_r   <= resize(signed(C_SIN_QUAD(to_integer(row_phase_r(7 downto 2)))), 9);

            -- row_offset = row_sin * amp_r (9×10 = 19), shifted to pixel scale
            v_off_mul := row_sin_r * signed('0' & std_logic_vector(amp_r));
            -- amp 0..1023 × sin ±127 / 256 → ±508 pixels max
            -- Add jitter from LFSR if tear_r
            if tear_r = '1' then
                v_jitter := signed(resize(lfsr_r(7 downto 0), 11))
                            - to_signed(128, 11);
            else
                v_jitter := (others => '0');
            end if;
            row_offset_r <= resize(shift_right(v_off_mul, 8), 11) + v_jitter;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
                field_flip_r <= not field_flip_r;
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                freq_r   <= unsigned(registers_in(0));
                speed_r  <= unsigned(registers_in(1));
                poff_r   <= unsigned(registers_in(2));
                vert_r   <= unsigned(registers_in(3));
                tint_r   <= unsigned(registers_in(4));
                bright_r <= unsigned(registers_in(5));
                mode_r   <= registers_in(6)(0);
                dir_r    <= registers_in(6)(1);
                mono_r   <= registers_in(6)(2);
                tear_r   <= registers_in(6)(3);
                invert_r <= registers_in(6)(4);
                amp_r    <= unsigned(registers_in(7));

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
        variable v_addr : signed(12 downto 0);
        variable v_chroma_u : signed(11 downto 0);
        variable v_chroma_v : signed(11 downto 0);
        variable v_y_mul : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;

            -- S1: rd_addr = clamp(pixel_x + row_offset)
            v_addr := signed(resize(s0_x, 13)) + resize(row_offset_r, 13);
            s1_addr <= clamp_addr(v_addr);

            -- S2: BRAM read happens.

            -- S3: capture
            s3_y <= unsigned(lb_y_out);
            s3_u <= unsigned(lb_u_out);
            s3_v <= unsigned(lb_v_out);

            -- S4: tint + mono
            if mono_r = '1' then
                s4_y <= s3_y;
                s4_u <= to_unsigned(512, 10);
                s4_v <= to_unsigned(512, 10);
            else
                s4_y <= s3_y;
                -- Bias U by tint_r - 512
                if tint_r > to_unsigned(512, 10) then
                    if (s3_u + (tint_r - 512)) > to_unsigned(1023, 10) then
                        s4_u <= to_unsigned(1023, 10);
                    else
                        s4_u <= s3_u + (tint_r - 512);
                    end if;
                else
                    if s3_u > (to_unsigned(512, 10) - tint_r) then
                        s4_u <= s3_u - (to_unsigned(512, 10) - tint_r);
                    else
                        s4_u <= to_unsigned(0, 10);
                    end if;
                end if;
                s4_v <= s3_v;
            end if;

            -- S5: brightness mul
            s5_ymul <= s4_y * bright_r;
            s5_u    <= s4_u;
            s5_v    <= s4_v;

            -- S6: extract + invert
            if invert_r = '1' then
                s6_y <= to_unsigned(1023, 10) - s5_ymul(19 downto 10);
            else
                s6_y <= s5_ymul(19 downto 10);
            end if;
            s6_u <= s5_u;
            s6_v <= s5_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s6_y);
    data_out.u       <= std_logic_vector(s6_u);
    data_out.v       <= std_logic_vector(s6_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture ripple;
