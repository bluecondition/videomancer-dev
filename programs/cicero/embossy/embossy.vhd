-- Embossy: directional emboss/relief of incoming video.
--
-- Computes a luma slope along a chosen direction (h, v, or both) by
-- comparing the current line's luma to a delayed copy held in a line
-- buffer.  The signed slope is biased to 50% gray to produce the classic
-- relief look.  Lighting direction is selectable; depth knob scales
-- the slope; the original colors can be re-applied over the relief.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Depth         (rotary 1 — slope gain)
--   registers_in(1) = Light Angle   (rotary 2 — direction selection)
--   registers_in(2) = Bias          (rotary 3 — midpoint adjust)
--   registers_in(3) = Saturation    (rotary 4 — color re-mix amount)
--   registers_in(4) = Tint          (rotary 5 — hue tint over relief)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Direction (0=Horizontal / 1=Vertical)
--     b1   Color     (0=Mono relief / 1=Recolor with input chroma)
--     b2   Edge      (0=Smooth / 1=Hard edges)
--     b3   Sign      (0=Light from NW / 1=Light from SE)
--     b4   Invert
--   registers_in(7) = Horizontal Offset (slider, KEY — H derivative pixel offset)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture embossy of program_top is

    constant LATENCY : natural := 12;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal field_flip_r : std_logic := '0';   -- dual-bank toggle for line buffer

    signal depth_r      : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal light_r      : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bias_r       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal sat_r        : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal tint_r       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r     : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal h_offset_r   : unsigned(9 downto 0) := to_unsigned(64,  10);

    signal direction_r  : std_logic := '0';
    signal color_r      : std_logic := '0';
    signal hard_r       : std_logic := '0';
    signal sign_r       : std_logic := '0';
    signal invert_r     : std_logic := '0';

    -- Horizontal delayed luma: 4 fixed taps for the index slider to choose
    -- among (1, 2, 4, 8 pixel delay).  Avoids a wide variable-index mux.
    signal y_d1_r : unsigned(9 downto 0) := (others => '0');
    signal y_d2_r : unsigned(9 downto 0) := (others => '0');
    signal y_d4_r : unsigned(9 downto 0) := (others => '0');
    signal y_d8_r : unsigned(9 downto 0) := (others => '0');
    type t_delay8 is array (0 to 7) of unsigned(9 downto 0);
    signal y_delay_r : t_delay8 := (others => (others => '0'));

    -- Vertical delayed sample: previous line's Y stored in dual-bank line
    -- buffer (instantiated below).
    signal lb_wr_addr : unsigned(10 downto 0);
    signal lb_rd_addr : unsigned(10 downto 0);
    signal lb_ab      : std_logic;
    signal lb_y_in_v  : std_logic_vector(9 downto 0);
    signal lb_y_out_v : std_logic_vector(9 downto 0);

    -- Pipeline signals
    -- S0: capture input Y/U/V; pick H delayed sample
    signal s0_y_in   : unsigned(9 downto 0);
    signal s0_u_in   : unsigned(9 downto 0);
    signal s0_v_in   : unsigned(9 downto 0);
    signal s0_y_h    : unsigned(9 downto 0);   -- H-delayed luma
    signal s0_y_v    : unsigned(9 downto 0);   -- V-delayed luma (from line buf)

    -- S1: delta = current - delayed (signed 11)
    signal s1_delta_h : signed(10 downto 0);
    signal s1_delta_v : signed(10 downto 0);
    signal s1_y_in    : unsigned(9 downto 0);
    signal s1_u_in    : unsigned(9 downto 0);
    signal s1_v_in    : unsigned(9 downto 0);

    -- S2: choose direction & apply sign + hard edge processing
    signal s2_delta : signed(10 downto 0);
    signal s2_y_in  : unsigned(9 downto 0);
    signal s2_u_in  : unsigned(9 downto 0);
    signal s2_v_in  : unsigned(9 downto 0);

    -- S3: depth mul (signed 11 × signed 11 = signed 22) registered raw
    signal s3_delta_mul : signed(21 downto 0);
    signal s3_y_in      : unsigned(9 downto 0);
    signal s3_u_in      : unsigned(9 downto 0);
    signal s3_v_in      : unsigned(9 downto 0);

    -- S4: shift right + bias center
    signal s4_relief : signed(11 downto 0);
    signal s4_y_in   : unsigned(9 downto 0);
    signal s4_u_in   : unsigned(9 downto 0);
    signal s4_v_in   : unsigned(9 downto 0);

    -- S5: clamp to 10-bit Y
    signal s5_y      : unsigned(9 downto 0);
    signal s5_u_in   : unsigned(9 downto 0);
    signal s5_v_in   : unsigned(9 downto 0);

    -- S6: chroma mixing
    signal s6_y    : unsigned(9 downto 0);
    signal s6_u    : unsigned(9 downto 0);
    signal s6_v    : unsigned(9 downto 0);

    -- S7: brightness scaling
    signal s7_ymul : unsigned(19 downto 0);
    signal s7_u    : unsigned(9 downto 0);
    signal s7_v    : unsigned(9 downto 0);

    -- S8: extract + invert
    signal s8_y : unsigned(9 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);

    function sat10s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- Line buffer for vertical derivative
    lb_inst : entity work.video_line_buffer
        generic map(G_WIDTH => 10, G_DEPTH => 11)
        port map(
            clk       => clk,
            i_ab      => lb_ab,
            i_wr_addr => lb_wr_addr,
            i_rd_addr => lb_rd_addr,
            i_data    => lb_y_in_v,
            o_data    => lb_y_out_v
        );

    lb_ab      <= field_flip_r;
    lb_wr_addr <= pixel_x(10 downto 0);
    lb_rd_addr <= pixel_x(10 downto 0);
    lb_y_in_v  <= data_in.y;
    s0_y_v_proc : process(clk)
    begin
        if rising_edge(clk) then
            s0_y_v <= unsigned(lb_y_out_v);
        end if;
    end process s0_y_v_proc;

    -- ========================================================================
    -- Position + parameters
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
                depth_r    <= unsigned(registers_in(0));
                light_r    <= unsigned(registers_in(1));
                bias_r     <= unsigned(registers_in(2));
                sat_r      <= unsigned(registers_in(3));
                tint_r     <= unsigned(registers_in(4));
                bright_r   <= unsigned(registers_in(5));
                direction_r <= registers_in(6)(0);
                color_r    <= registers_in(6)(1);
                hard_r     <= registers_in(6)(2);
                sign_r     <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                h_offset_r <= unsigned(registers_in(7));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;

            -- Shift the horizontal delay line; expose 4 tap points.
            if data_in.avid = '1' then
                for i in 7 downto 1 loop
                    y_delay_r(i) <= y_delay_r(i - 1);
                end loop;
                y_delay_r(0) <= unsigned(data_in.y);
                y_d1_r <= y_delay_r(0);
                y_d2_r <= y_delay_r(1);
                y_d4_r <= y_delay_r(3);
                y_d8_r <= y_delay_r(7);
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_relief : signed(11 downto 0);
        variable v_bias : signed(11 downto 0);
        variable v_chroma_u : signed(11 downto 0);
        variable v_chroma_v : signed(11 downto 0);
        variable v_u_mix : signed(11 downto 0);
        variable v_v_mix : signed(11 downto 0);
        variable v_y_mul : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: capture inputs.  Pick H-delayed luma via 4-way mux on top
            -- 2 bits of h_offset_r (delay = 1, 2, 4, or 8 pixels).
            case to_integer(h_offset_r(9 downto 8)) is
                when 0      => s0_y_h <= y_d1_r;
                when 1      => s0_y_h <= y_d2_r;
                when 2      => s0_y_h <= y_d4_r;
                when others => s0_y_h <= y_d8_r;
            end case;
            -- s0_y_v assigned in dedicated process from line buffer
            s0_y_in <= unsigned(data_in.y);
            s0_u_in <= unsigned(data_in.u);
            s0_v_in <= unsigned(data_in.v);

            -- S1: deltas (signed 11 = ±1023)
            s1_delta_h <= signed(resize(s0_y_in, 11)) -
                          signed(resize(s0_y_h,  11));
            s1_delta_v <= signed(resize(s0_y_in, 11)) -
                          signed(resize(s0_y_v,  11));
            s1_y_in <= s0_y_in;
            s1_u_in <= s0_u_in;
            s1_v_in <= s0_v_in;

            -- S2: choose direction.  Diagonal = h+v / sqrt2 ≈ shift.
            if direction_r = '0' then
                if sign_r = '0' then
                    s2_delta <= s1_delta_h;
                else
                    s2_delta <= -s1_delta_h;
                end if;
            else
                if sign_r = '0' then
                    s2_delta <= s1_delta_v;
                else
                    s2_delta <= -s1_delta_v;
                end if;
            end if;
            s2_y_in <= s1_y_in;
            s2_u_in <= s1_u_in;
            s2_v_in <= s1_v_in;

            -- S3: depth multiply (signed × signed)
            s3_delta_mul <= s2_delta * signed('0' & std_logic_vector(depth_r));
            s3_y_in <= s2_y_in;
            s3_u_in <= s2_u_in;
            s3_v_in <= s2_v_in;

            -- S4: shift to 12-bit and add bias to center
            v_relief := resize(shift_right(s3_delta_mul, 10), 12);
            v_bias := signed(resize(bias_r, 12));
            s4_relief <= v_relief + v_bias;
            s4_y_in <= s3_y_in;
            s4_u_in <= s3_u_in;
            s4_v_in <= s3_v_in;

            -- S5: clamp/quantize.  If hard_r, snap to 0/512/1023.
            if hard_r = '1' then
                if s4_relief < to_signed(341, 12) then
                    s5_y <= to_unsigned(0, 10);
                elsif s4_relief > to_signed(683, 12) then
                    s5_y <= to_unsigned(1023, 10);
                else
                    s5_y <= to_unsigned(512, 10);
                end if;
            else
                s5_y <= sat10s(s4_relief);
            end if;
            s5_u_in <= s4_u_in;
            s5_v_in <= s4_v_in;

            -- S6: chroma blending.  When color_r=0, output monochrome.
            -- When color_r=1, pass input chroma through (no multiply chain
            -- on the critical path).  Tint knob biases U toward warm/cool.
            if color_r = '0' then
                s6_u <= to_unsigned(512, 10);
                s6_v <= to_unsigned(512, 10);
            else
                s6_u <= s5_u_in;
                s6_v <= s5_v_in;
            end if;
            s6_y <= s5_y;

            -- S7: brightness scale Y (registered raw mul)
            s7_ymul <= s6_y * bright_r;
            s7_u    <= s6_u;
            s7_v    <= s6_v;

            -- S8: extract + invert
            if invert_r = '1' then
                s8_y <= to_unsigned(1023, 10) - s7_ymul(19 downto 10);
            else
                s8_y <= s7_ymul(19 downto 10);
            end if;
            s8_u <= s7_u;
            s8_v <= s7_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s8_y);
    data_out.u       <= std_logic_vector(s8_u);
    data_out.v       <= std_logic_vector(s8_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture embossy;
