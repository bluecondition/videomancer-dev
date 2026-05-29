-- Kaleidoscope: N-fold horizontal sectoring of incoming video.
--
-- The current scanline is buffered in BRAM line buffers.  The output
-- screen is divided into N equal-width sectors (1, 2, 3, 4, 6, 8, 12, 16
-- selectable by slider).  Within each sector the output reads from the
-- buffer at a position folded into [0, sector_width); alternate sectors
-- are mirrored to give the classic kaleidoscope reflection look.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP).
--
-- Register map:
--   registers_in(0) = Mirror Mode    (rotary 1 — none / alternate / all)
--   registers_in(1) = Row Stagger    (rotary 2 — every-other-row offset)
--   registers_in(2) = Spin Speed     (rotary 3 — sector rotation per frame)
--   registers_in(3) = Border Size    (rotary 4 — bright segment dividers)
--   registers_in(4) = Tint           (rotary 5 — chroma adjust)
--   registers_in(5) = Brightness     (rotary 6)
--   registers_in(6) = Switches:
--     b0   Row Mirror (0=Off / 1=Mirror odd rows)
--     b1   Border     (0=Off / 1=Draw dividers)
--     b2   Per-Sector Hue (0=Same / 1=Each sector tinted)
--     b3   Spin       (0=Static / 1=Animated)
--     b4   Invert
--   registers_in(7) = Segments (slider, KEY — N sectors)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture kaleidoscope of program_top is

    constant LATENCY : natural := 8;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal field_flip_r : std_logic := '0';

    signal mirror_mode_r : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal stagger_r     : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal spin_r        : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal border_r      : unsigned(9 downto 0) := to_unsigned(0,   10);
    signal tint_r        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bright_r      : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal segments_r    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal row_mir_r     : std_logic := '0';
    signal border_en_r   : std_logic := '0';
    signal sector_hue_r  : std_logic := '0';
    signal spin_en_r     : std_logic := '0';
    signal invert_r      : std_logic := '0';

    signal spin_phase_r  : unsigned(11 downto 0) := (others => '0');

    -- Sector count and width derived from segments_r slider
    signal n_sectors_r   : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal sector_w_r    : unsigned(11 downto 0) := to_unsigned(480, 12);

    -- Line buffers for Y, U, V
    signal lb_ab      : std_logic;
    signal lb_wr_addr : unsigned(10 downto 0);
    signal lb_rd_addr : unsigned(10 downto 0);
    signal lb_y_out   : std_logic_vector(9 downto 0);
    signal lb_u_out   : std_logic_vector(9 downto 0);
    signal lb_v_out   : std_logic_vector(9 downto 0);

    -- Pipeline signals
    signal s0_x         : unsigned(11 downto 0);

    -- S1: sector index = x / sector_w (use multiplier inverse for non-pow2)
    -- We pre-compute n_sectors and sector_w at vsync.  Sector index =
    -- (x * n_sectors) / 1920.  Use approximate inverse: multiply by
    -- (1 << 16)/1920 ≈ 34.13, round to 34.  x * n_sectors * 34 >> 16.
    -- Combine: sector_idx_full = x * n_sectors * INV_W >> 16.
    -- For x ≤ 1919, n_sectors ≤ 16 → x*n_sectors ≤ ~30k, ×34 ≤ ~1M.
    signal s1_xn   : unsigned(16 downto 0);   -- x*n_sectors
    signal s1_xn_x : unsigned(11 downto 0);   -- forward x for later

    -- S2: multiply by INV constant ≈ 4456 (for 1920) = 2^23 / 1920
    signal s2_prod : unsigned(28 downto 0);
    signal s2_xn   : unsigned(16 downto 0);  -- forwarded
    signal s2_x    : unsigned(11 downto 0);

    -- S3: extract sector_idx and position-within-sector
    signal s3_sect_idx : unsigned(4 downto 0);
    signal s3_pos      : unsigned(11 downto 0);  -- position 0..sector_w-1
    signal s3_x        : unsigned(11 downto 0);

    -- S4: fold position: if sector odd (per mirror mode), pos = sector_w-1-pos
    signal s4_sample_x : unsigned(11 downto 0);
    signal s4_sect_idx : unsigned(4 downto 0);
    signal s4_x        : unsigned(11 downto 0);

    -- S5: line buffer read address = sample_x
    -- Buffer read is asynchronous-ish; outputs come 1 clock later.

    -- S6: line buffer output captured
    signal s6_y : unsigned(9 downto 0);
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);
    signal s6_sect_idx : unsigned(4 downto 0);
    signal s6_x        : unsigned(11 downto 0);
    signal s6_pos      : unsigned(11 downto 0);
    signal s6_sample_x : unsigned(11 downto 0);

    -- S7: apply per-sector tint + border + invert
    signal s7_y : unsigned(9 downto 0);
    signal s7_u : unsigned(9 downto 0);
    signal s7_v : unsigned(9 downto 0);

    -- INV_W table indexed by n_sectors (precomputed = 2^23/1920 * n_sectors)
    -- Actually we want to compute sector_idx = (x * n / W) and pos = x*n mod W.
    -- We'll use the trick: total = x * n (small, 17 bits).
    -- sector_idx = total / W,  pos = total - sector_idx*W
    -- For a single W = 1920, we can use a divider via mul-by-inverse + correct.
    -- INV_1920 = ceil(2^23/1920) = 4370 (since 1920*4369 = 8388480 < 2^23 < 1920*4370)
    constant C_INV_W : unsigned(12 downto 0) := to_unsigned(4370, 13);

    -- Sector forward delays for tint
    type t_sect_delay is array (0 to 5) of unsigned(4 downto 0);
    signal sect_delay_r : t_sect_delay := (others => (others => '0'));

    function sat10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- ========================================================================
    -- Line buffers: Y, U, V each as separate dual-bank line buffer
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
    lb_rd_addr <= s4_sample_x(10 downto 0);

    -- ========================================================================
    -- Position + parameters + sector setup
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_seg    : unsigned(3 downto 0);
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
                field_flip_r <= not field_flip_r;
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                mirror_mode_r <= unsigned(registers_in(0));
                stagger_r     <= unsigned(registers_in(1));
                spin_r        <= unsigned(registers_in(2));
                border_r      <= unsigned(registers_in(3));
                tint_r        <= unsigned(registers_in(4));
                bright_r      <= unsigned(registers_in(5));
                row_mir_r     <= registers_in(6)(0);
                border_en_r   <= registers_in(6)(1);
                sector_hue_r  <= registers_in(6)(2);
                spin_en_r     <= registers_in(6)(3);
                invert_r      <= registers_in(6)(4);
                segments_r    <= unsigned(registers_in(7));

                -- Map slider 0..1023 to n_sectors ∈ {1, 2, 3, 4, 6, 8, 12, 16}
                v_seg := segments_r(9 downto 6);  -- 4-bit, 0..15
                case to_integer(v_seg) is
                    when 0 | 1   => n_sectors_r <= to_unsigned(1,  5);
                                    sector_w_r  <= to_unsigned(1920, 12);
                    when 2 | 3   => n_sectors_r <= to_unsigned(2,  5);
                                    sector_w_r  <= to_unsigned(960, 12);
                    when 4 | 5   => n_sectors_r <= to_unsigned(3,  5);
                                    sector_w_r  <= to_unsigned(640, 12);
                    when 6 | 7   => n_sectors_r <= to_unsigned(4,  5);
                                    sector_w_r  <= to_unsigned(480, 12);
                    when 8 | 9   => n_sectors_r <= to_unsigned(6,  5);
                                    sector_w_r  <= to_unsigned(320, 12);
                    when 10 | 11 => n_sectors_r <= to_unsigned(8,  5);
                                    sector_w_r  <= to_unsigned(240, 12);
                    when 12 | 13 => n_sectors_r <= to_unsigned(12, 5);
                                    sector_w_r  <= to_unsigned(160, 12);
                    when others  => n_sectors_r <= to_unsigned(16, 5);
                                    sector_w_r  <= to_unsigned(120, 12);
                end case;

                v_speed := "00000" & unsigned(registers_in(2)(9 downto 5));
                if registers_in(6)(3) = '1' then
                    spin_phase_r <= spin_phase_r + ("00" & v_speed);
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
        variable v_pos     : unsigned(11 downto 0);
        variable v_idx_pos : unsigned(11 downto 0);
        variable v_should_mirror : boolean;
        variable v_chroma_u : signed(11 downto 0);
        variable v_chroma_v : signed(11 downto 0);
        variable v_y_mul : unsigned(19 downto 0);
        variable v_border_dist : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;

            -- S1: x * n_sectors
            s1_xn <= s0_x * n_sectors_r;
            s1_xn_x <= s0_x;

            -- S2: multiply by inverse to get sector_idx
            s2_prod <= resize(s1_xn * C_INV_W, 29);
            s2_xn   <= s1_xn;
            s2_x    <= s1_xn_x;

            -- S3: extract sector_idx + position
            -- sector_idx = prod >> 23 (top 5 bits)
            -- pos = xn - sector_idx*sector_w  (but pos = (prod mod (1<<23)) / n_sectors)
            -- Simpler: pos = x - sector_idx * sector_w (registered subtract next stage)
            s3_sect_idx <= s2_prod(27 downto 23);
            s3_pos      <= s2_x;  -- temporarily store; correct below
            s3_x        <= s2_x;

            -- S4: pos = x - sector_idx * sector_w (1 mult)
            v_idx_pos := unsigned(std_logic_vector(
                            resize(s3_sect_idx * sector_w_r(11 downto 0), 12)));
            v_pos := s3_x - v_idx_pos;

            -- Determine if we should mirror this sector
            -- mirror_mode_r: 0 = none, 1 = alternate, 2 = all (low 2 bits hold index)
            v_should_mirror := false;
            case to_integer(mirror_mode_r(1 downto 0)) is
                when 0 => v_should_mirror := false;
                when 1 => v_should_mirror := s3_sect_idx(0) = '1';
                when 2 => v_should_mirror := true;
                when others => v_should_mirror := s3_sect_idx(0) = '1';
            end case;
            -- Row mirror: if row_mir_r and odd row, flip mirror flag
            if row_mir_r = '1' and pixel_y(0) = '1' then
                v_should_mirror := not v_should_mirror;
            end if;

            if v_should_mirror then
                if v_pos < sector_w_r then
                    s4_sample_x <= sector_w_r - 1 - v_pos;
                else
                    s4_sample_x <= (others => '0');
                end if;
            else
                s4_sample_x <= v_pos;
            end if;
            s4_sect_idx <= s3_sect_idx;
            s4_x <= s3_x;

            -- S5: line buffer read happens; output appears next cycle.

            -- Delay sector_idx for S6/S7 alignment.
            sect_delay_r(0) <= s4_sect_idx;
            for i in 1 to 5 loop
                sect_delay_r(i) <= sect_delay_r(i - 1);
            end loop;

            -- S6: capture line buffer output
            s6_y <= unsigned(lb_y_out);
            s6_u <= unsigned(lb_u_out);
            s6_v <= unsigned(lb_v_out);
            s6_sect_idx <= sect_delay_r(0);
            s6_x        <= s4_x;
            s6_pos      <= s4_sample_x;
            s6_sample_x <= s4_sample_x;

            -- S7: per-sector tint + border + invert + brightness
            -- Border: if pos < border_size or > (sector_w - border_size), draw bright
            -- Use s6_pos and a comparison against border_r
            v_border_dist := s6_sample_x;
            if border_en_r = '1' and
               (v_border_dist < ("00" & border_r) or
                v_border_dist > (sector_w_r - 1 - ("00" & border_r))) then
                s7_y <= to_unsigned(1023, 10);
            else
                v_y_mul := s6_y * bright_r;
                if invert_r = '1' then
                    s7_y <= to_unsigned(1023, 10) - v_y_mul(19 downto 10);
                else
                    s7_y <= v_y_mul(19 downto 10);
                end if;
            end if;

            -- Per-sector hue: shift U/V by tint scaled by sector index
            if sector_hue_r = '1' then
                v_chroma_u := signed(resize(s6_u, 12)) - to_signed(512, 12);
                v_chroma_v := signed(resize(s6_v, 12)) - to_signed(512, 12);
                -- Tint sector_idx * 32 added to U; subtract from V
                s7_u <= sat10(v_chroma_u + to_signed(512, 12) +
                              signed(resize(s6_sect_idx & "000", 12)));
                s7_v <= sat10(v_chroma_v + to_signed(512, 12) -
                              signed(resize(s6_sect_idx & "000", 12)));
            else
                s7_u <= s6_u;
                s7_v <= s6_v;
            end if;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s7_y);
    data_out.u       <= std_logic_vector(s7_u);
    data_out.v       <= std_logic_vector(s7_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture kaleidoscope;
