-- Vectorscope: UV-plane scatter plot of incoming video chroma.
--
-- A 64×64 single-bit persistence buffer (1 iCE40 BRAM) is written by the
-- incoming video stream's (U, V) top-bits during active video and cleared
-- one cell per cycle during vertical blanking.  The output displays the
-- buffer as dots on screen, optionally with a graticule reference grid,
-- so users can see the chroma content of the input live.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Persistence    (rotary 1)
--   registers_in(1) = Trace Hue      (rotary 2 — dot color)
--   registers_in(2) = Graticule Brt  (rotary 3 — grid brightness)
--   registers_in(3) = Underlay Mix   (rotary 4 — input video visibility)
--   registers_in(4) = Gain           (rotary 5 — chroma pre-scale)
--   registers_in(5) = Brightness     (rotary 6)
--   registers_in(6) = Switches:
--     b0   Show Grid    (0=Off / 1=On)
--     b1   Show Crosshair (0=Off / 1=On)
--     b2   Show Input  (0=Black / 1=Dim input video)
--     b3   Trace Style (0=Single / 1=Multi-color by quadrant)
--     b4   Invert
--   registers_in(7) = Display Scale (slider, KEY — UV plane area shown)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture vectorscope of program_top is

    constant LATENCY : natural := 8;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal persist_r   : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal hue_r       : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal grat_brt_r  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal under_r     : unsigned(9 downto 0) := to_unsigned(120, 10);
    signal gain_r      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r    : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal scale_r     : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal grid_r      : std_logic := '1';
    signal cross_r     : std_logic := '1';
    signal show_in_r   : std_logic := '0';
    signal multi_r     : std_logic := '0';
    signal invert_r    : std_logic := '0';

    -- 64×64 × 1-bit persistence BRAM
    type t_bram is array (0 to 4095) of std_logic_vector(0 downto 0);
    signal bram_r : t_bram := (others => "0");

    signal wr_addr : unsigned(11 downto 0);
    signal wr_data : std_logic_vector(0 downto 0);
    signal wr_en   : std_logic;
    signal rd_addr : unsigned(11 downto 0);
    signal rd_data : std_logic_vector(0 downto 0);

    -- Decay counter
    signal decay_cnt_r : unsigned(11 downto 0) := (others => '0');

    -- Pipeline signals
    signal s0_x      : unsigned(11 downto 0);
    signal s0_y      : unsigned(11 downto 0);
    signal s0_y_in   : unsigned(9 downto 0);
    signal s0_u_in   : unsigned(9 downto 0);
    signal s0_v_in   : unsigned(9 downto 0);
    signal s0_active : std_logic;

    -- S1: compute screen-mapped (u, v) cell
    -- Display area: centered 1024×1024 box (clamped to 1080 height)
    -- Scale slider 0..1023 → display half-width 128..768 px
    signal s1_dx    : signed(12 downto 0);
    signal s1_dy    : signed(12 downto 0);
    signal s1_y_in  : unsigned(9 downto 0);
    signal s1_u_in  : unsigned(9 downto 0);
    signal s1_v_in  : unsigned(9 downto 0);

    -- S2: shifted to 0..63 cells
    signal s2_u_cell : unsigned(5 downto 0);
    signal s2_v_cell : unsigned(5 downto 0);
    signal s2_inside : std_logic;
    signal s2_y_in   : unsigned(9 downto 0);
    signal s2_u_in   : unsigned(9 downto 0);
    signal s2_v_in   : unsigned(9 downto 0);

    -- S3: BRAM read (latched here from rd_data)
    signal s3_hit    : std_logic;
    signal s3_inside : std_logic;
    signal s3_y_in   : unsigned(9 downto 0);
    signal s3_u_in   : unsigned(9 downto 0);
    signal s3_v_in   : unsigned(9 downto 0);
    signal s3_u_cell : unsigned(5 downto 0);
    signal s3_v_cell : unsigned(5 downto 0);

    -- S4: assign final Y/U/V
    signal s4_y : unsigned(9 downto 0);
    signal s4_u : unsigned(9 downto 0);
    signal s4_v : unsigned(9 downto 0);

    -- Display geometry (precomputed at vsync)
    signal half_w_r : unsigned(9 downto 0) := to_unsigned(384, 10);

    function sat10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- ========================================================================
    -- BRAM access (dual-port write/read)
    -- ========================================================================
    p_bram : process(clk)
    begin
        if rising_edge(clk) then
            if wr_en = '1' then
                bram_r(to_integer(wr_addr)) <= wr_data;
            end if;
            rd_data <= bram_r(to_integer(rd_addr));
        end if;
    end process p_bram;

    -- Write port driver
    p_write : process(clk)
        variable v_u_top : unsigned(5 downto 0);
        variable v_v_top : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                -- Write a 1 at (U_in_top, V_in_top)
                v_u_top := unsigned(data_in.u(9 downto 4));
                v_v_top := unsigned(data_in.v(9 downto 4));
                wr_addr <= v_v_top & v_u_top;
                wr_data <= "1";
                wr_en   <= '1';
            else
                -- Decay: cycle through cells writing 0
                wr_addr <= decay_cnt_r;
                wr_data <= "0";
                wr_en   <= '1';
                decay_cnt_r <= decay_cnt_r + 1;
            end if;
        end if;
    end process p_write;

    -- Read port: rd_addr = screen-mapped cell
    rd_addr <= s2_v_cell & s2_u_cell;

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
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                persist_r  <= unsigned(registers_in(0));
                hue_r      <= unsigned(registers_in(1));
                grat_brt_r <= unsigned(registers_in(2));
                under_r    <= unsigned(registers_in(3));
                gain_r     <= unsigned(registers_in(4));
                bright_r   <= unsigned(registers_in(5));
                grid_r     <= registers_in(6)(0);
                cross_r    <= registers_in(6)(1);
                show_in_r  <= registers_in(6)(2);
                multi_r    <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                scale_r    <= unsigned(registers_in(7));

                -- Half-width of display area (range 128..768)
                half_w_r <= to_unsigned(128, 10) + unsigned(registers_in(7)(9 downto 1));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_dx_abs : signed(12 downto 0);
        variable v_dy_abs : signed(12 downto 0);
        variable v_x_pos  : signed(12 downto 0);
        variable v_y_pos  : signed(12 downto 0);
        variable v_inside : std_logic;
        variable v_grat   : std_logic;
        variable v_y      : unsigned(9 downto 0);
        variable v_u      : unsigned(9 downto 0);
        variable v_v      : unsigned(9 downto 0);
        variable v_under_y : unsigned(9 downto 0);
        variable v_quadrant : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: capture
            s0_x     <= pixel_x;
            s0_y     <= pixel_y;
            s0_y_in  <= unsigned(data_in.y);
            s0_u_in  <= unsigned(data_in.u);
            s0_v_in  <= unsigned(data_in.v);
            s0_active <= data_in.avid;

            -- S1: dx, dy centered (signed 13)
            s1_dx <= signed(resize(s0_x, 13)) - to_signed(960, 13);
            s1_dy <= signed(resize(s0_y, 13)) - to_signed(540, 13);
            s1_y_in <= s0_y_in;
            s1_u_in <= s0_u_in;
            s1_v_in <= s0_v_in;

            -- S2: map (dx, dy) to (0..63, 0..63) cell, mark inside
            v_x_pos := s1_dx + signed(resize(half_w_r, 13));
            v_y_pos := s1_dy + signed(resize(half_w_r, 13));
            -- inside if both 0..2*half_w
            if v_x_pos >= 0 and v_x_pos < signed(resize(half_w_r & '0', 13)) and
               v_y_pos >= 0 and v_y_pos < signed(resize(half_w_r & '0', 13)) then
                v_inside := '1';
                -- cell_idx = pos / (2*half_w / 64).  Half_w = 128..768.
                -- For simplicity use shift right based on half_w bit range.
                -- We approximate by taking top 6 bits of v_pos after scaling.
                -- pos has range 0..2*half_w ≈ 0..1535.  Top 6 bits of pos>>4 = 0..63.
                -- This is approximate but acceptable for vectorscope view.
                if v_x_pos < to_signed(1024, 13) then
                    s2_u_cell <= unsigned(std_logic_vector(v_x_pos(9 downto 4)));
                else
                    s2_u_cell <= to_unsigned(63, 6);
                end if;
                if v_y_pos < to_signed(1024, 13) then
                    s2_v_cell <= unsigned(std_logic_vector(v_y_pos(9 downto 4)));
                else
                    s2_v_cell <= to_unsigned(63, 6);
                end if;
            else
                v_inside := '0';
                s2_u_cell <= (others => '0');
                s2_v_cell <= (others => '0');
            end if;
            s2_inside <= v_inside;
            s2_y_in <= s1_y_in;
            s2_u_in <= s1_u_in;
            s2_v_in <= s1_v_in;

            -- S3: capture BRAM read.  rd_data is from one clock ago (s2's req).
            s3_hit    <= rd_data(0);
            s3_inside <= s2_inside;
            s3_y_in   <= s2_y_in;
            s3_u_in   <= s2_u_in;
            s3_v_in   <= s2_v_in;
            s3_u_cell <= s2_u_cell;
            s3_v_cell <= s2_v_cell;

            -- S4: produce output color
            -- Underlay video (dimmed)
            v_under_y := unsigned(std_logic_vector(("000" & s3_y_in(9 downto 3))));

            if s3_inside = '0' then
                -- Outside display: optionally show input
                if show_in_r = '1' then
                    v_y := v_under_y;
                    v_u := s3_u_in;
                    v_v := s3_v_in;
                else
                    v_y := to_unsigned(40, 10);
                    v_u := to_unsigned(512, 10);
                    v_v := to_unsigned(512, 10);
                end if;
            else
                -- Inside display area
                -- Graticule: bright line at 16, 32, 48 (axis lines)
                if grid_r = '1' and (
                   s3_u_cell = to_unsigned(16, 6) or
                   s3_u_cell = to_unsigned(48, 6) or
                   s3_v_cell = to_unsigned(16, 6) or
                   s3_v_cell = to_unsigned(48, 6) or
                   (cross_r = '1' and (s3_u_cell = to_unsigned(32, 6) or
                                       s3_v_cell = to_unsigned(32, 6)))
                   ) then
                    v_y := grat_brt_r;
                    v_u := to_unsigned(512, 10);
                    v_v := to_unsigned(512, 10);
                elsif s3_hit = '1' then
                    -- Trace dot: bright phosphor green by default, or quadrant-tinted
                    v_y := to_unsigned(900, 10);
                    if multi_r = '0' then
                        -- Hue-rotated single color
                        v_u := to_unsigned(380, 10) +
                               resize(hue_r(9 downto 2), 10);
                        v_v := to_unsigned(400, 10) -
                               resize(hue_r(9 downto 3), 10);
                    else
                        -- Per-quadrant color by cell location
                        v_quadrant := s3_u_cell(5) & s3_v_cell(5);
                        case to_integer(v_quadrant) is
                            when 0 => v_u := to_unsigned(720, 10);
                                      v_v := to_unsigned(380, 10);
                            when 1 => v_u := to_unsigned(800, 10);
                                      v_v := to_unsigned(720, 10);
                            when 2 => v_u := to_unsigned(280, 10);
                                      v_v := to_unsigned(280, 10);
                            when others =>
                                      v_u := to_unsigned(380, 10);
                                      v_v := to_unsigned(820, 10);
                        end case;
                    end if;
                else
                    -- Dark background
                    v_y := to_unsigned(20, 10);
                    v_u := to_unsigned(512, 10);
                    v_v := to_unsigned(512, 10);
                end if;
            end if;

            if invert_r = '1' then
                v_y := to_unsigned(1023, 10) - v_y;
            end if;

            s4_y <= v_y;
            s4_u <= v_u;
            s4_v <= v_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s4_y);
    data_out.u       <= std_logic_vector(s4_u);
    data_out.v       <= std_logic_vector(s4_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture vectorscope;
