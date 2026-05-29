-- tiler.vhd
--
-- Captures a 64x64 region of the incoming video (position set by knobs 1 and 2)
-- into a BRAM and tiles that region across the whole active screen. Knob 3
-- rotates the tile around its center.
--
-- Write path : when active_pixel/active_line is inside the capture window,
--              store data_in.{y,u,v} at bram[(dy[5:0], dx[5:0])].
-- Read  path : every active pixel, rotate (active_pixel, active_line) around
--              the tile center by knob 3's angle, wrap mod 64, read BRAM.
--
-- Pipeline   : 4-stage rotation (R0..R3) + BRAM read = 5 clk read latency.
-- Sync delay : 5 clocks.
--
-- Resource note: 64x64 x 10-bit x 3 channels = ~120 Kbit, nearly all of the
-- HX4K's 128 Kbit BRAM. If routing fails, shrink C_TILE_BITS to 5 (32x32).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture tiler of program_top is

    constant C_TILE_BITS : integer := 6;
    constant C_TILE_SIZE : integer := 2**C_TILE_BITS;
    constant C_BUF_DEPTH : integer := C_TILE_BITS * 2;
    constant C_BUF_SIZE  : integer := 2**C_BUF_DEPTH;

    constant C_SIN_BITS : integer := 6;
    constant C_SIN_SIZE : integer := 2**C_SIN_BITS;

    -- Sin lookup: sin(2pi * i / 64) * 127, 8-bit signed.
    -- cos(theta) = sin(theta + pi/2) = C_SIN_ROM[(i + 16) mod 64]
    type t_sin_rom is array (0 to C_SIN_SIZE - 1) of signed(7 downto 0);
    constant C_SIN_ROM : t_sin_rom := (
        to_signed(   0, 8), to_signed(  12, 8), to_signed(  25, 8), to_signed(  37, 8),
        to_signed(  49, 8), to_signed(  60, 8), to_signed(  71, 8), to_signed(  81, 8),
        to_signed(  90, 8), to_signed(  98, 8), to_signed( 106, 8), to_signed( 112, 8),
        to_signed( 117, 8), to_signed( 122, 8), to_signed( 125, 8), to_signed( 126, 8),
        to_signed( 127, 8), to_signed( 126, 8), to_signed( 125, 8), to_signed( 122, 8),
        to_signed( 117, 8), to_signed( 112, 8), to_signed( 106, 8), to_signed(  98, 8),
        to_signed(  90, 8), to_signed(  81, 8), to_signed(  71, 8), to_signed(  60, 8),
        to_signed(  49, 8), to_signed(  37, 8), to_signed(  25, 8), to_signed(  12, 8),
        to_signed(   0, 8), to_signed( -12, 8), to_signed( -25, 8), to_signed( -37, 8),
        to_signed( -49, 8), to_signed( -60, 8), to_signed( -71, 8), to_signed( -81, 8),
        to_signed( -90, 8), to_signed( -98, 8), to_signed(-106, 8), to_signed(-112, 8),
        to_signed(-117, 8), to_signed(-122, 8), to_signed(-125, 8), to_signed(-126, 8),
        to_signed(-127, 8), to_signed(-126, 8), to_signed(-125, 8), to_signed(-122, 8),
        to_signed(-117, 8), to_signed(-112, 8), to_signed(-106, 8), to_signed( -98, 8),
        to_signed( -90, 8), to_signed( -81, 8), to_signed( -71, 8), to_signed( -60, 8),
        to_signed( -49, 8), to_signed( -37, 8), to_signed( -25, 8), to_signed( -12, 8)
    );

    -- Parameters
    signal s_knob_x         : unsigned(9 downto 0);
    signal s_knob_y         : unsigned(9 downto 0);
    signal s_knob_rot       : unsigned(9 downto 0);
    signal s_knob_size      : unsigned(9 downto 0);
    signal s_knob_luma_thr  : unsigned(9 downto 0);
    signal s_sw_luma_mod    : std_logic;
    signal s_sw_show_video  : std_logic;

    -- Tile geometry derived from knob 4 (log2 tile size, 0..C_TILE_BITS)
    signal s_tile_bits   : unsigned(2 downto 0) := to_unsigned(C_TILE_BITS, 3);
    signal s_tile_size   : unsigned(11 downto 0) := to_unsigned(C_TILE_SIZE, 12);
    signal s_tile_mask   : unsigned(C_TILE_BITS - 1 downto 0) := (others => '1');
    signal s_tile_center : unsigned(C_TILE_BITS - 1 downto 0) := to_unsigned(C_TILE_SIZE / 2, C_TILE_BITS);

    -- Latched active-region dimensions
    signal s_active_width  : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_active_height : unsigned(11 downto 0) := to_unsigned(1080, 12);

    -- Capture origin (top-left of 64x64 window)
    signal s_capture_x : unsigned(11 downto 0) := (others => '0');
    signal s_capture_y : unsigned(11 downto 0) := (others => '0');

    -- Position counters
    signal s_active_pixel : unsigned(11 downto 0) := (others => '0');
    signal s_active_line  : unsigned(11 downto 0) := (others => '0');
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_avid    : std_logic := '0';

    -- BRAM: 4096-deep 10-bit buffer per channel
    type t_bram is array (0 to C_BUF_SIZE - 1)
        of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal bram_y : t_bram := (others => (others => '0'));
    signal bram_u : t_bram := (others => (9 => '1', others => '0'));  -- neutral chroma
    signal bram_v : t_bram := (others => (9 => '1', others => '0'));

    -- Write-side registered signals (1 stage)
    signal s_wr_addr : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');
    signal s_wr_en   : std_logic := '0';
    signal s_wr_y    : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wr_u    : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_wr_v    : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- Read-side rotation pipeline
    -- R0: centered coords + sin/cos lookups
    signal s_r0_du  : signed(6 downto 0) := (others => '0');
    signal s_r0_dv  : signed(6 downto 0) := (others => '0');
    signal s_r0_sin : signed(7 downto 0) := (others => '0');
    signal s_r0_cos : signed(7 downto 0) := (others => '0');
    -- R1: four signed 8b x 7b multiplies (15-bit results)
    signal s_r1_cos_du : signed(14 downto 0) := (others => '0');
    signal s_r1_sin_dv : signed(14 downto 0) := (others => '0');
    signal s_r1_sin_du : signed(14 downto 0) := (others => '0');
    signal s_r1_cos_dv : signed(14 downto 0) := (others => '0');
    -- R2: combine
    signal s_r2_x_pre : signed(15 downto 0) := (others => '0');
    signal s_r2_y_pre : signed(15 downto 0) := (others => '0');
    -- R3: scale + wrap -> read address
    signal s_rd_addr : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    -- R4: BRAM outputs
    signal s_tile_y : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tile_u : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_tile_v : std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- 5-stage sync shift register
    signal s_avid_sr    : std_logic_vector(4 downto 0) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(4 downto 0) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(4 downto 0) := (others => '1');
    signal s_field_n_sr : std_logic_vector(4 downto 0) := (others => '1');

    -- 5-stage passthrough delay for incoming Y/U/V (for the "show video"
    -- option when a tile fails the luma threshold).
    type t_yuv_delay is array (0 to 4) of std_logic_vector(C_VIDEO_DATA_WIDTH - 1 downto 0);
    signal s_y_sr : t_yuv_delay := (others => (others => '0'));
    signal s_u_sr : t_yuv_delay := (others => (others => '0'));
    signal s_v_sr : t_yuv_delay := (others => (others => '0'));

    -- Luma-mod: decision sampled on the first line of each tile row, cached
    -- per tile column, reused for the rest of the tile row. Delayed 5 clocks
    -- to align with the output pixel.
    signal s_luma_pass_cur : std_logic := '0';
    signal s_luma_pass_sr  : std_logic_vector(4 downto 0) := (others => '0');
    signal s_show_tile     : std_logic;

    -- Per-tile-column decision cache: 512 entries covers tile sizes N >= 4 at
    -- HD (max tile_col = 480). For N = 1 or 2 we bypass the cache and sample
    -- every line (fewer tile pixels per line makes the flicker less visible
    -- than the LC cost of a larger cache, which won't fit on the HX4K).
    -- Read + write in one clocked process, so this stays LUT + FF storage.
    signal s_dec_bits : std_logic_vector(511 downto 0) := (others => '0');

    -- Combinational tile-position derivations
    signal s_tile_col          : unsigned(11 downto 0);
    signal s_tile_col_idx      : integer range 0 to 511;
    signal s_tile_col_next     : unsigned(11 downto 0);  -- tile_col of (active_pixel+1)
    signal s_tile_col_next_idx : integer range 0 to 511;
    signal s_at_tile_edge      : std_logic;
    signal s_first_line_tr     : std_logic;

    -- Registered read of the decision cache, using the look-ahead index so the
    -- value is available at the boundary cycle. Breaks the 512:1 mux out of
    -- the p_luma_decision critical path.
    signal s_dec_read : std_logic := '0';

begin

    s_knob_x        <= unsigned(registers_in(0));
    s_knob_y        <= unsigned(registers_in(1));
    s_knob_rot      <= unsigned(registers_in(2));
    s_knob_size     <= unsigned(registers_in(3));
    s_knob_luma_thr <= unsigned(registers_in(4));
    s_sw_luma_mod   <= registers_in(6)(0);
    s_sw_show_video <= registers_in(6)(1);

    ----------------------------------------------------------------------------
    -- Tile geometry: map knob 4 top 3 bits to log2(tile_size) in 0..6, then
    -- derive size / mask / center. Power-of-2 tile sizes keep address math to
    -- a single AND.
    ----------------------------------------------------------------------------
    p_tile_geometry : process(clk)
        variable v_bits : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            v_bits := s_knob_size(9 downto 7);
            if v_bits = "111" then
                v_bits := to_unsigned(C_TILE_BITS, 3);  -- clamp to max
            end if;
            s_tile_bits <= v_bits;

            case to_integer(v_bits) is
                when 0 =>
                    s_tile_size   <= to_unsigned(1,  12);
                    s_tile_mask   <= "000000";
                    s_tile_center <= to_unsigned(0,  C_TILE_BITS);
                when 1 =>
                    s_tile_size   <= to_unsigned(2,  12);
                    s_tile_mask   <= "000001";
                    s_tile_center <= to_unsigned(1,  C_TILE_BITS);
                when 2 =>
                    s_tile_size   <= to_unsigned(4,  12);
                    s_tile_mask   <= "000011";
                    s_tile_center <= to_unsigned(2,  C_TILE_BITS);
                when 3 =>
                    s_tile_size   <= to_unsigned(8,  12);
                    s_tile_mask   <= "000111";
                    s_tile_center <= to_unsigned(4,  C_TILE_BITS);
                when 4 =>
                    s_tile_size   <= to_unsigned(16, 12);
                    s_tile_mask   <= "001111";
                    s_tile_center <= to_unsigned(8,  C_TILE_BITS);
                when 5 =>
                    s_tile_size   <= to_unsigned(32, 12);
                    s_tile_mask   <= "011111";
                    s_tile_center <= to_unsigned(16, C_TILE_BITS);
                when others =>
                    s_tile_size   <= to_unsigned(64, 12);
                    s_tile_mask   <= "111111";
                    s_tile_center <= to_unsigned(32, C_TILE_BITS);
            end case;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Position counters
    ----------------------------------------------------------------------------
    p_counters : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_active_pixel <= (others => '0');
            elsif data_in.avid = '1' then
                s_active_pixel <= s_active_pixel + 1;
            end if;

            if data_in.avid = '0' and s_prev_avid = '1' then
                s_active_width <= s_active_pixel;
                s_active_line  <= s_active_line + 1;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_active_line /= 0 then
                    s_active_height <= s_active_line;
                end if;
                s_active_line <= (others => '0');
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Capture origin from knobs 1/2 (shift x2, clamp to active size - 64)
    ----------------------------------------------------------------------------
    p_capture_origin : process(clk)
        variable v_x_scaled : unsigned(11 downto 0);
        variable v_y_scaled : unsigned(11 downto 0);
        variable v_max_x    : unsigned(11 downto 0);
        variable v_max_y    : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            v_x_scaled := resize(s_knob_x & '0', 12);
            v_y_scaled := resize(s_knob_y & '0', 12);

            if s_active_width >= s_tile_size then
                v_max_x := s_active_width - s_tile_size;
            else
                v_max_x := (others => '0');
            end if;

            if s_active_height >= s_tile_size then
                v_max_y := s_active_height - s_tile_size;
            else
                v_max_y := (others => '0');
            end if;

            if v_x_scaled > v_max_x then
                s_capture_x <= v_max_x;
            else
                s_capture_x <= v_x_scaled;
            end if;

            if v_y_scaled > v_max_y then
                s_capture_y <= v_max_y;
            else
                s_capture_y <= v_y_scaled;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Write side: stage the write when current pixel is inside capture region
    ----------------------------------------------------------------------------
    p_write_gen : process(clk)
        variable v_in_x : std_logic;
        variable v_in_y : std_logic;
        variable v_dx   : unsigned(11 downto 0);
        variable v_dy   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_active_pixel >= s_capture_x and
               s_active_pixel <  (s_capture_x + s_tile_size) then
                v_in_x := '1';
            else
                v_in_x := '0';
            end if;

            if s_active_line >= s_capture_y and
               s_active_line <  (s_capture_y + s_tile_size) then
                v_in_y := '1';
            else
                v_in_y := '0';
            end if;

            v_dx := s_active_pixel - s_capture_x;
            v_dy := s_active_line  - s_capture_y;

            s_wr_addr <= (v_dy(C_TILE_BITS - 1 downto 0) and s_tile_mask)
                       & (v_dx(C_TILE_BITS - 1 downto 0) and s_tile_mask);
            s_wr_en   <= data_in.avid and v_in_x and v_in_y;
            s_wr_y    <= data_in.y;
            s_wr_u    <= data_in.u;
            s_wr_v    <= data_in.v;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Read R0: center coords on tile midpoint; look up sin and cos for angle
    ----------------------------------------------------------------------------
    p_rot_r0 : process(clk)
        variable v_idx_sin : integer range 0 to C_SIN_SIZE - 1;
        variable v_idx_cos : integer range 0 to C_SIN_SIZE - 1;
        variable v_u       : signed(6 downto 0);
        variable v_v       : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            -- Mask to current tile size, then center on tile midpoint
            v_u := signed('0' & std_logic_vector(s_active_pixel(C_TILE_BITS - 1 downto 0) and s_tile_mask))
                   - signed('0' & std_logic_vector(s_tile_center));
            v_v := signed('0' & std_logic_vector(s_active_line (C_TILE_BITS - 1 downto 0) and s_tile_mask))
                   - signed('0' & std_logic_vector(s_tile_center));
            s_r0_du <= v_u;
            s_r0_dv <= v_v;

            v_idx_sin := to_integer(s_knob_rot(9 downto 10 - C_SIN_BITS));
            v_idx_cos := (v_idx_sin + C_SIN_SIZE / 4) mod C_SIN_SIZE;
            s_r0_sin <= C_SIN_ROM(v_idx_sin);
            s_r0_cos <= C_SIN_ROM(v_idx_cos);
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Read R1: four signed multiplies
    ----------------------------------------------------------------------------
    p_rot_r1 : process(clk)
    begin
        if rising_edge(clk) then
            s_r1_cos_du <= s_r0_cos * s_r0_du;
            s_r1_sin_dv <= s_r0_sin * s_r0_dv;
            s_r1_sin_du <= s_r0_sin * s_r0_du;
            s_r1_cos_dv <= s_r0_cos * s_r0_dv;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Read R2: combine into rotated pre-scaled coords
    ----------------------------------------------------------------------------
    p_rot_r2 : process(clk)
    begin
        if rising_edge(clk) then
            s_r2_x_pre <= resize(s_r1_cos_du, 16) - resize(s_r1_sin_dv, 16);
            s_r2_y_pre <= resize(s_r1_sin_du, 16) + resize(s_r1_cos_dv, 16);
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Read R3: shift right 7 (scale ~1/128), re-center, wrap mod 64, form addr
    ----------------------------------------------------------------------------
    p_rot_r3 : process(clk)
        variable v_x_shifted : signed(8 downto 0);
        variable v_y_shifted : signed(8 downto 0);
        variable v_x_sum     : signed(9 downto 0);
        variable v_y_sum     : signed(9 downto 0);
        variable v_x_addr    : unsigned(C_TILE_BITS - 1 downto 0);
        variable v_y_addr    : unsigned(C_TILE_BITS - 1 downto 0);
    begin
        if rising_edge(clk) then
            v_x_shifted := s_r2_x_pre(15 downto 7);
            v_y_shifted := s_r2_y_pre(15 downto 7);

            v_x_sum := resize(v_x_shifted, 10) + signed('0' & std_logic_vector(resize(s_tile_center, 9)));
            v_y_sum := resize(v_y_shifted, 10) + signed('0' & std_logic_vector(resize(s_tile_center, 9)));

            -- Mask with tile_mask to wrap at the current tile size
            v_x_addr := unsigned(std_logic_vector(v_x_sum(C_TILE_BITS - 1 downto 0))) and s_tile_mask;
            v_y_addr := unsigned(std_logic_vector(v_y_sum(C_TILE_BITS - 1 downto 0))) and s_tile_mask;

            s_rd_addr <= v_y_addr & v_x_addr;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Read R4 / write: BRAM access
    ----------------------------------------------------------------------------
    p_bram : process(clk)
    begin
        if rising_edge(clk) then
            s_tile_y <= bram_y(to_integer(s_rd_addr));
            s_tile_u <= bram_u(to_integer(s_rd_addr));
            s_tile_v <= bram_v(to_integer(s_rd_addr));
            if s_wr_en = '1' then
                bram_y(to_integer(s_wr_addr)) <= s_wr_y;
                bram_u(to_integer(s_wr_addr)) <= s_wr_u;
                bram_v(to_integer(s_wr_addr)) <= s_wr_v;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Tile-position derivations (combinational)
    ----------------------------------------------------------------------------
    p_tile_col_shift : process(s_active_pixel, s_tile_bits)
    begin
        case to_integer(s_tile_bits) is
            when 0      => s_tile_col <= s_active_pixel;
            when 1      => s_tile_col <= shift_right(s_active_pixel, 1);
            when 2      => s_tile_col <= shift_right(s_active_pixel, 2);
            when 3      => s_tile_col <= shift_right(s_active_pixel, 3);
            when 4      => s_tile_col <= shift_right(s_active_pixel, 4);
            when 5      => s_tile_col <= shift_right(s_active_pixel, 5);
            when others => s_tile_col <= shift_right(s_active_pixel, 6);
        end case;
    end process;

    -- Look-ahead: tile_col of (active_pixel + 1) so a registered RAM read
    -- initiated this cycle arrives in time for next cycle's tile boundary.
    p_tile_col_next_shift : process(s_active_pixel, s_tile_bits)
        variable v_next : unsigned(11 downto 0);
    begin
        v_next := s_active_pixel + 1;
        case to_integer(s_tile_bits) is
            when 0      => s_tile_col_next <= v_next;
            when 1      => s_tile_col_next <= shift_right(v_next, 1);
            when 2      => s_tile_col_next <= shift_right(v_next, 2);
            when 3      => s_tile_col_next <= shift_right(v_next, 3);
            when 4      => s_tile_col_next <= shift_right(v_next, 4);
            when 5      => s_tile_col_next <= shift_right(v_next, 5);
            when others => s_tile_col_next <= shift_right(v_next, 6);
        end case;
    end process;

    s_tile_col_idx      <= to_integer(s_tile_col     (8 downto 0));
    s_tile_col_next_idx <= to_integer(s_tile_col_next(8 downto 0));

    -- Registered read of decision cache (breaks the 512:1 mux out of the
    -- p_luma_decision critical path).
    p_dec_read : process(clk)
    begin
        if rising_edge(clk) then
            s_dec_read <= s_dec_bits(s_tile_col_next_idx);
        end if;
    end process;
    s_at_tile_edge  <= '1' when (s_active_pixel(C_TILE_BITS - 1 downto 0) and s_tile_mask)
                                 = to_unsigned(0, C_TILE_BITS) else '0';
    s_first_line_tr <= '1' when (s_active_line (C_TILE_BITS - 1 downto 0) and s_tile_mask)
                                 = to_unsigned(0, C_TILE_BITS) else '0';

    ----------------------------------------------------------------------------
    -- Luma decision: for N >= 4, on the FIRST line of a tile row sample
    -- (data_in.y > threshold) at each tile-column boundary and cache it per
    -- tile column; on subsequent lines of the same tile row reuse the cache.
    -- For N = 1 or N = 2, bypass the cache (tile_col would alias past the
    -- 512-entry array) and just sample at every boundary on every line.
    ----------------------------------------------------------------------------
    p_luma_decision : process(clk)
        variable v_fresh : std_logic;
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' and s_at_tile_edge = '1' then
                if unsigned(data_in.y) > s_knob_luma_thr then
                    v_fresh := '1';
                else
                    v_fresh := '0';
                end if;

                if s_tile_bits < "010" then
                    -- N = 1 or 2: no caching; sample fresh each boundary
                    s_luma_pass_cur <= v_fresh;
                elsif s_first_line_tr = '1' then
                    -- First line of tile row: sample fresh and cache
                    s_luma_pass_cur            <= v_fresh;
                    s_dec_bits(s_tile_col_idx) <= v_fresh;
                else
                    -- Later line of same tile row: use pre-read cached decision
                    s_luma_pass_cur <= s_dec_read;
                end if;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Sync + luma-pass delay: 5 clocks to match R0..R3 + BRAM read
    ----------------------------------------------------------------------------
    p_sync_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr      <= s_avid_sr     (3 downto 0) & data_in.avid;
            s_hsync_n_sr   <= s_hsync_n_sr  (3 downto 0) & data_in.hsync_n;
            s_vsync_n_sr   <= s_vsync_n_sr  (3 downto 0) & data_in.vsync_n;
            s_field_n_sr   <= s_field_n_sr  (3 downto 0) & data_in.field_n;
            s_luma_pass_sr <= s_luma_pass_sr(3 downto 0) & s_luma_pass_cur;
            s_y_sr(0) <= data_in.y;
            s_y_sr(1) <= s_y_sr(0);
            s_y_sr(2) <= s_y_sr(1);
            s_y_sr(3) <= s_y_sr(2);
            s_y_sr(4) <= s_y_sr(3);
            s_u_sr(0) <= data_in.u;
            s_u_sr(1) <= s_u_sr(0);
            s_u_sr(2) <= s_u_sr(1);
            s_u_sr(3) <= s_u_sr(2);
            s_u_sr(4) <= s_u_sr(3);
            s_v_sr(0) <= data_in.v;
            s_v_sr(1) <= s_v_sr(0);
            s_v_sr(2) <= s_v_sr(1);
            s_v_sr(3) <= s_v_sr(2);
            s_v_sr(4) <= s_v_sr(3);
        end if;
    end process;

    s_show_tile <= '1' when s_sw_luma_mod = '0' or s_luma_pass_sr(4) = '1' else '0';

    data_out.y <= s_tile_y when s_show_tile = '1'
             else s_y_sr(4) when s_sw_show_video = '1'
             else (others => '0');
    data_out.u <= s_tile_u when s_show_tile = '1'
             else s_u_sr(4) when s_sw_show_video = '1'
             else "1000000000";
    data_out.v <= s_tile_v when s_show_tile = '1'
             else s_v_sr(4) when s_sw_show_video = '1'
             else "1000000000";
    data_out.avid    <= s_avid_sr(4);
    data_out.hsync_n <= s_hsync_n_sr(4);
    data_out.vsync_n <= s_vsync_n_sr(4);
    data_out.field_n <= s_field_n_sr(4);

end tiler;
