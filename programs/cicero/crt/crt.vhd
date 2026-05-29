-- CRT: cathode-ray tube simulation overlay.
--
-- Stacks classic CRT artifacts on incoming video: horizontal scanlines,
-- phosphor RGB triad, edge vignette, and small subtle roll.  Each effect
-- has its own intensity control and the master slider gates everything
-- so the strength is one-knob adjustable.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Scanline      (rotary 1 — darkening of odd rows)
--   registers_in(1) = Curve         (rotary 2 — corner darkening rate)
--   registers_in(2) = Vignette      (rotary 3 — edge falloff)
--   registers_in(3) = Phosphor Glow (rotary 4 — adds previous pixel)
--   registers_in(4) = Tint Bias     (rotary 5 — warm/cool chroma)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Triad   (0=Off / 1=RGB columns)
--     b1   Roll    (0=Off / 1=Slow vertical drift band)
--     b2   Color   (0=Color / 1=B&W)
--     b3   Bandwidth (0=Full / 1=Chroma soft)
--     b4   Invert
--   registers_in(7) = CRT Strength (slider, KEY — overall mix)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture crt of program_top is

    constant LATENCY : natural := 9;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal scan_r     : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal curve_r    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal vign_r     : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal glow_r     : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal tint_r     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r   : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal strength_r : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal triad_r    : std_logic := '1';
    signal roll_r     : std_logic := '0';
    signal mono_r     : std_logic := '0';
    signal soft_r     : std_logic := '0';
    signal invert_r   : std_logic := '0';

    -- Previous-pixel hold register (for phosphor glow)
    signal prev_y_r : unsigned(9 downto 0) := (others => '0');

    -- Roll band Y position (animated)
    signal roll_y_r : unsigned(11 downto 0) := (others => '0');

    -- Pipeline signals
    signal s0_x      : unsigned(11 downto 0);
    signal s0_y      : unsigned(11 downto 0);
    signal s0_y_in   : unsigned(9 downto 0);
    signal s0_u_in   : unsigned(9 downto 0);
    signal s0_v_in   : unsigned(9 downto 0);

    -- S1: scanline darkening + glow added
    signal s1_y_in   : unsigned(9 downto 0);
    signal s1_u_in   : unsigned(9 downto 0);
    signal s1_v_in   : unsigned(9 downto 0);
    signal s1_dim    : std_logic;   -- scanline dim flag
    signal s1_x      : unsigned(11 downto 0);
    signal s1_y      : unsigned(11 downto 0);

    -- S2: scanline-darkened Y + triad tint
    signal s2_y      : unsigned(9 downto 0);
    signal s2_u      : unsigned(9 downto 0);
    signal s2_v      : unsigned(9 downto 0);
    signal s2_x      : unsigned(11 downto 0);
    signal s2_y_pix  : unsigned(11 downto 0);

    -- S3: vignette factor + roll band check
    signal s3_y      : unsigned(9 downto 0);
    signal s3_u      : unsigned(9 downto 0);
    signal s3_v      : unsigned(9 downto 0);
    signal s3_vign_d : unsigned(11 downto 0);  -- distance from center
    signal s3_roll   : std_logic;

    -- S4: apply vignette: y * (1 - vign*d/1024)
    signal s4_y      : unsigned(9 downto 0);
    signal s4_u      : unsigned(9 downto 0);
    signal s4_v      : unsigned(9 downto 0);

    -- S5a: apply roll-band darken (registered intermediate)
    signal s5a_y     : unsigned(9 downto 0);
    signal s5a_u     : unsigned(9 downto 0);
    signal s5a_v     : unsigned(9 downto 0);

    -- S5: brightness mul (registered raw)
    signal s5_ymul   : unsigned(19 downto 0);
    signal s5_u      : unsigned(9 downto 0);
    signal s5_v      : unsigned(9 downto 0);

    -- S6: invert + extract
    signal s6_y      : unsigned(9 downto 0);
    signal s6_u      : unsigned(9 downto 0);
    signal s6_v      : unsigned(9 downto 0);

    function sat_sub(a, b : unsigned) return unsigned is
        variable v_diff : signed(a'length downto 0);
    begin
        v_diff := signed(resize(a, a'length + 1)) -
                  signed(resize(b, a'length + 1));
        if v_diff < 0 then return to_unsigned(0, a'length); end if;
        return unsigned(std_logic_vector(v_diff(a'length - 1 downto 0)));
    end function;

begin

    -- ========================================================================
    -- Position + parameters + previous-pixel latch
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

            if data_in.avid = '1' then
                prev_y_r <= unsigned(data_in.y);
            end if;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;
                scan_r     <= unsigned(registers_in(0));
                curve_r    <= unsigned(registers_in(1));
                vign_r     <= unsigned(registers_in(2));
                glow_r     <= unsigned(registers_in(3));
                tint_r     <= unsigned(registers_in(4));
                bright_r   <= unsigned(registers_in(5));
                triad_r    <= registers_in(6)(0);
                roll_r     <= registers_in(6)(1);
                mono_r     <= registers_in(6)(2);
                soft_r     <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                strength_r <= unsigned(registers_in(7));

                -- Roll band drift: ~2 pixels/frame
                roll_y_r <= roll_y_r + to_unsigned(2, 12);
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_y_glow : unsigned(10 downto 0);
        variable v_pidx : unsigned(1 downto 0);
        variable v_dx : signed(12 downto 0);
        variable v_dy : signed(12 downto 0);
        variable v_dim_amt : unsigned(9 downto 0);
        variable v_dim_mul : unsigned(19 downto 0);
        variable v_y_dimmed : unsigned(9 downto 0);
        variable v_chroma_u : signed(11 downto 0);
        variable v_chroma_v : signed(11 downto 0);
        variable v_roll_dist : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;
            s0_y_in <= unsigned(data_in.y);
            s0_u_in <= unsigned(data_in.u);
            s0_v_in <= unsigned(data_in.v);

            -- S1: phosphor glow = y_in + prev_y * glow_r/1024
            -- Use simple: y_glow = y_in + (prev_y_r >> some) gated by glow_r top bits
            v_y_glow := resize(s0_y_in, 11) + ("000" & prev_y_r(9 downto 2));
            if v_y_glow > to_unsigned(1023, 11) then
                s1_y_in <= to_unsigned(1023, 10);
            else
                s1_y_in <= resize(v_y_glow(9 downto 0), 10);
            end if;
            s1_u_in <= s0_u_in;
            s1_v_in <= s0_v_in;
            s1_dim <= s0_y(0);  -- scanline parity (odd rows dim)
            s1_x <= s0_x;
            s1_y <= s0_y;

            -- S2: scanline dim + RGB triad tint
            if s1_dim = '1' then
                -- Dim by scan_r/2048
                v_dim_amt := scan_r(9 downto 1) & "0";  -- 0..1022
                if s1_y_in > v_dim_amt then
                    v_y_dimmed := s1_y_in - v_dim_amt(9 downto 0);
                else
                    v_y_dimmed := (others => '0');
                end if;
            else
                v_y_dimmed := s1_y_in;
            end if;
            s2_y <= v_y_dimmed;

            -- RGB triad: 4-col pattern (0=R-tinted, 1=G-tinted, 2=B-tinted, 3=gap)
            if triad_r = '1' then
                v_pidx := s1_x(1 downto 0);
                case to_integer(v_pidx) is
                    when 0 => -- R-tinted: V high, U low
                        s2_u <= sat_sub(s1_u_in, to_unsigned(100, 10));
                        s2_v <= s1_v_in + to_unsigned(100, 10);
                    when 1 => -- G-tinted: U low, V low
                        s2_u <= sat_sub(s1_u_in, to_unsigned(80, 10));
                        s2_v <= sat_sub(s1_v_in, to_unsigned(80, 10));
                    when 2 => -- B-tinted: U high, V low
                        s2_u <= s1_u_in + to_unsigned(100, 10);
                        s2_v <= sat_sub(s1_v_in, to_unsigned(80, 10));
                    when others => -- gap: pass through
                        s2_u <= s1_u_in;
                        s2_v <= s1_v_in;
                end case;
            else
                s2_u <= s1_u_in;
                s2_v <= s1_v_in;
            end if;
            s2_x <= s1_x;
            s2_y_pix <= s1_y;

            -- S3: vignette distance + roll band
            v_dx := signed(resize(s2_x, 13)) - to_signed(960, 13);
            v_dy := signed(resize(s2_y_pix, 13)) - to_signed(540, 13);
            if v_dx < 0 then v_dx := -v_dx; end if;
            if v_dy < 0 then v_dy := -v_dy; end if;
            s3_vign_d <= resize(unsigned(std_logic_vector(v_dx(11 downto 0))) +
                                unsigned(std_logic_vector(v_dy(11 downto 0))), 12);

            v_roll_dist := s2_y_pix - roll_y_r(11 downto 0);
            if roll_r = '1' and v_roll_dist < to_unsigned(64, 12) then
                s3_roll <= '1';
            else
                s3_roll <= '0';
            end if;
            s3_y <= s2_y;
            s3_u <= s2_u;
            s3_v <= s2_v;

            -- S4: apply vignette = y_in * (1 - vign*d/1024)
            -- Scale by vign_r top 6 bits to make a 10×6 multiply.
            v_dim_mul := resize(s3_vign_d(9 downto 0) *
                                vign_r(9 downto 4), 20);
            v_dim_amt := v_dim_mul(15 downto 6);
            if s3_y > v_dim_amt then
                s4_y <= s3_y - v_dim_amt;
            else
                s4_y <= (others => '0');
            end if;
            -- Chroma: B&W if mono_r, soft (toward neutral) if soft_r
            if mono_r = '1' then
                s4_u <= to_unsigned(512, 10);
                s4_v <= to_unsigned(512, 10);
            elsif soft_r = '1' then
                -- pull chroma toward neutral by 1/2
                v_chroma_u := signed(resize(s3_u, 12)) - to_signed(512, 12);
                v_chroma_v := signed(resize(s3_v, 12)) - to_signed(512, 12);
                s4_u <= unsigned(std_logic_vector(
                            resize(shift_right(v_chroma_u, 1), 10))) +
                        to_unsigned(512, 10);
                s4_v <= unsigned(std_logic_vector(
                            resize(shift_right(v_chroma_v, 1), 10))) +
                        to_unsigned(512, 10);
            else
                s4_u <= s3_u;
                s4_v <= s3_v;
            end if;

            -- S5a: apply roll-band darken first (registered)
            if s3_roll = '1' then
                s5a_y <= sat_sub(s4_y, to_unsigned(300, 10));
            else
                s5a_y <= s4_y;
            end if;
            s5a_u <= s4_u;
            s5a_v <= s4_v;

            -- S5: brightness mul (registered raw, 10×8 for faster mul)
            s5_ymul <= resize(s5a_y * bright_r(9 downto 2), 20);
            s5_u <= s5a_u;
            s5_v <= s5a_v;

            -- S6: extract + invert (top of mul gives Y / 4, then *4 by selecting bits)
            if invert_r = '1' then
                s6_y <= to_unsigned(1023, 10) -
                        s5_ymul(15 downto 6);
            else
                s6_y <= s5_ymul(15 downto 6);
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

end architecture crt;
