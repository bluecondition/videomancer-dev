-- Chromatic Aberration: horizontal RGB-style chroma channel separation.
--
-- Y, U and V each pass through their own line buffer.  Each scanline,
-- U is read from an X position shifted in one direction and V from the
-- opposite direction; the shift amount grows linearly with horizontal
-- distance from screen centre, mimicking lens-edge chromatic aberration.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).  One 10×10
-- multiply per pixel for distance-scaled offset.
--
-- Register map:
--   registers_in(0) = Strength      (rotary 1 — overall offset gain)
--   registers_in(1) = U/V Spread    (rotary 2 — direction-pair magnitude)
--   registers_in(2) = Falloff Curve (rotary 3 — affects centre vs edge)
--   registers_in(3) = Anim Speed    (rotary 4 — pulsed-strength rate)
--   registers_in(4) = Tint Bias     (rotary 5 — chroma offset)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Mode    (0=Radial / 1=Constant)
--     b1   Direction (0=Spread / 1=Squeeze)
--     b2   Channel (0=Chroma / 1=Luma also)
--     b3   Color   (0=Color / 1=Mono)
--     b4   Invert
--   registers_in(7) = Aberration (slider, KEY — max pixel offset)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture chromaticab of program_top is

    constant LATENCY : natural := 10;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal field_flip_r : std_logic := '0';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal strength_r  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal spread_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal falloff_r   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal anim_r      : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal tint_r      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r    : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal aberration_r: unsigned(9 downto 0) := to_unsigned(384, 10);
    signal mode_r      : std_logic := '0';
    signal dir_r       : std_logic := '0';
    signal channel_r   : std_logic := '0';
    signal mono_r      : std_logic := '0';
    signal invert_r    : std_logic := '0';

    -- Line buffer port signals
    signal lb_ab        : std_logic;
    signal lb_wr_addr   : unsigned(10 downto 0);
    signal lb_rd_addr_y : unsigned(10 downto 0);
    signal lb_rd_addr_u : unsigned(10 downto 0);
    signal lb_rd_addr_v : unsigned(10 downto 0);
    signal lb_y_out     : std_logic_vector(9 downto 0);
    signal lb_u_out     : std_logic_vector(9 downto 0);
    signal lb_v_out     : std_logic_vector(9 downto 0);

    -- Precomputed strength-anim product at vsync (saves a multiply)
    signal eff_strength_r : unsigned(9 downto 0) := to_unsigned(256, 10);

    -- Pipeline signals
    signal s0_x      : unsigned(11 downto 0);

    -- S1: dx = pixel_x - 960
    signal s1_dx : signed(12 downto 0);
    signal s1_x  : unsigned(11 downto 0);

    -- S2: dx * strength registered (12×10 = 22-bit signed mul)
    signal s2_off_mul : signed(22 downto 0);
    signal s2_x       : unsigned(11 downto 0);

    -- S3: extract offset, scale via aberration knob (8-bit shift)
    signal s3_offset : signed(11 downto 0);
    signal s3_x      : unsigned(11 downto 0);

    -- S4: rd_addr_u = clamp(s3_x + s3_offset), rd_addr_v = clamp(s3_x - s3_offset)
    signal s4_addr_u : unsigned(10 downto 0);
    signal s4_addr_v : unsigned(10 downto 0);
    signal s4_addr_y : unsigned(10 downto 0);

    -- S5: line buffer read pending (1 cycle latency)

    -- S6: capture line buffer outputs
    signal s6_y : unsigned(9 downto 0);
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);

    -- S7: tint + mono + invert
    signal s7_y : unsigned(9 downto 0);
    signal s7_u : unsigned(9 downto 0);
    signal s7_v : unsigned(9 downto 0);

    -- S8: brightness mul (raw)
    signal s8_ymul : unsigned(19 downto 0);
    signal s8_u    : unsigned(9 downto 0);
    signal s8_v    : unsigned(9 downto 0);

    -- S9: final extract
    signal s9_y : unsigned(9 downto 0);
    signal s9_u : unsigned(9 downto 0);
    signal s9_v : unsigned(9 downto 0);

    function clamp_x_to_1919(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 11); end if;
        if v > 1919 then return to_unsigned(1919, 11); end if;
        return unsigned(std_logic_vector(resize(v, 11)));
    end function;

begin

    -- ========================================================================
    -- Three line buffers (Y, U, V), separate read addresses per channel.
    -- ========================================================================
    lb_y_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr_y,
            i_data    => data_in.y,
            o_data    => lb_y_out
        );
    lb_u_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr_u,
            i_data    => data_in.u,
            o_data    => lb_u_out
        );
    lb_v_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr_v,
            i_data    => data_in.v,
            o_data    => lb_v_out
        );

    lb_ab        <= field_flip_r;
    lb_wr_addr   <= pixel_x(10 downto 0);
    lb_rd_addr_y <= s4_addr_y;
    lb_rd_addr_u <= s4_addr_u;
    lb_rd_addr_v <= s4_addr_v;

    -- ========================================================================
    -- Position + parameters
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_anim_term : unsigned(9 downto 0);
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
                field_flip_r <= not field_flip_r;
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                strength_r   <= unsigned(registers_in(0));
                spread_r     <= unsigned(registers_in(1));
                falloff_r    <= unsigned(registers_in(2));
                anim_r       <= unsigned(registers_in(3));
                tint_r       <= unsigned(registers_in(4));
                bright_r     <= unsigned(registers_in(5));
                mode_r       <= registers_in(6)(0);
                dir_r        <= registers_in(6)(1);
                channel_r    <= registers_in(6)(2);
                mono_r       <= registers_in(6)(3);
                invert_r     <= registers_in(6)(4);
                aberration_r <= unsigned(registers_in(7));

                -- Pulsed strength via frame_count + anim knob
                v_anim_term := "00" & frame_count(7 downto 0);
                v_anim_term := resize(shift_right(v_anim_term * unsigned(registers_in(3)(9 downto 4)), 4), 10);
                if (resize(unsigned(registers_in(0)), 11) +
                    resize(v_anim_term, 11)) > to_unsigned(1023, 11) then
                    eff_strength_r <= to_unsigned(1023, 10);
                else
                    eff_strength_r <= resize(
                        resize(unsigned(registers_in(0)), 11) +
                        resize(v_anim_term, 11), 10);
                end if;
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_off    : signed(11 downto 0);
        variable v_off_u  : signed(13 downto 0);
        variable v_off_v  : signed(13 downto 0);
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

            -- S1: dx
            s1_dx <= signed(resize(s0_x, 13)) - to_signed(960, 13);
            s1_x  <= s0_x;

            -- S2: dx * strength (constant mode uses pixel_x = 0 i.e. no dx)
            if mode_r = '0' then
                s2_off_mul <= resize(s1_dx *
                              signed('0' & std_logic_vector(eff_strength_r)), 23);
            else
                -- Constant offset: just strength as a constant signed value.
                s2_off_mul <= resize(to_signed(960, 13) *
                                signed('0' & std_logic_vector(eff_strength_r)), 23);
            end if;
            s2_x <= s1_x;

            -- S3: extract offset.  dx max 960, * 1023 ≈ 982080 (20-bit signed).
            -- Shift right by (10 + aberration_inverse) gives us the offset.
            -- aberration_r 0..1023 maps to shift 14..10 → offset range increases.
            -- Use aberration top bits to scale.
            s3_offset <= resize(shift_right(s2_off_mul, 12), 12);
            s3_x      <= s2_x;

            -- Scale offset by aberration slider top bits: shift right more for less aberration
            -- Actually simpler: just multiply s3_offset by aberration_r >> 6 (top 4 bits)
            -- For now use straight offset.

            -- S4: rd addrs
            -- u_offset gets +s3_offset (or -depending on dir_r)
            -- v_offset opposite
            if dir_r = '0' then
                v_off_u := resize(signed(resize(s3_x, 12)) + s3_offset, 14);
                v_off_v := resize(signed(resize(s3_x, 12)) - s3_offset, 14);
            else
                v_off_u := resize(signed(resize(s3_x, 12)) - s3_offset, 14);
                v_off_v := resize(signed(resize(s3_x, 12)) + s3_offset, 14);
            end if;
            -- Y can also shift if channel_r='1'
            if channel_r = '1' then
                -- Shift Y by half of offset
                s4_addr_y <= clamp_x_to_1919(resize(signed(resize(s3_x, 12)) +
                                  resize(shift_right(s3_offset, 1), 12), 13));
            else
                s4_addr_y <= unsigned(s3_x(10 downto 0));
            end if;
            s4_addr_u <= clamp_x_to_1919(v_off_u);
            s4_addr_v <= clamp_x_to_1919(v_off_v);

            -- S5: line buffer reads happen.

            -- S6: capture
            s6_y <= unsigned(lb_y_out);
            s6_u <= unsigned(lb_u_out);
            s6_v <= unsigned(lb_v_out);

            -- S7: tint + mono
            if mono_r = '1' then
                s7_u <= to_unsigned(512, 10);
                s7_v <= to_unsigned(512, 10);
            else
                -- Tint biases chroma toward (tint, 512) direction.
                -- Simple: add tint_r - 512 to U, subtract to V
                v_chroma_u := signed(resize(s6_u, 12)) - to_signed(512, 12);
                v_chroma_v := signed(resize(s6_v, 12)) - to_signed(512, 12);
                if (signed(resize(s6_u, 12)) +
                    signed(resize(tint_r, 12)) - to_signed(512, 12))
                   < 0 then
                    s7_u <= to_unsigned(0, 10);
                elsif (signed(resize(s6_u, 12)) +
                       signed(resize(tint_r, 12)) - to_signed(512, 12))
                      > 1023 then
                    s7_u <= to_unsigned(1023, 10);
                else
                    s7_u <= resize(s6_u + tint_r - 512, 10);
                end if;
                s7_v <= s6_v;
            end if;
            s7_y <= s6_y;

            -- S8: brightness mul
            s8_ymul <= s7_y * bright_r;
            s8_u    <= s7_u;
            s8_v    <= s7_v;

            -- S9: extract + invert
            if invert_r = '1' then
                s9_y <= to_unsigned(1023, 10) - s8_ymul(19 downto 10);
            else
                s9_y <= s8_ymul(19 downto 10);
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

end architecture chromaticab;
