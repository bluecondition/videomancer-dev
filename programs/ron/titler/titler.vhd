-- Titler: low-fi video text overlay generator.
--
-- Renders two lines of 16 characters each (32 cells total) over the input
-- video. Knobs control horizontal position, vertical position, size, and
-- color. An Edit Mode (toggle 7) exposes a single editing cursor that walks
-- top-to-bottom across both lines (knob 5); knob 6 picks the letter under
-- the cursor; knob 4 erases (in edit mode); a Matte toggle (9) lets the
-- incoming video reveal through the text shapes.
--
-- Pipeline (5 cycles total from data_in to data_out):
--   S1: position counters (h_count, v_count) - free-running pixel/line index
--   S2: pixel deltas (dx, dy) relative to text origin (V dim has +1 row of
--       top margin so the cursor box can draw above line 0)
--   S3: char index, line index, glyph x/y, in-text bounds, top margin flag
--   S4: text buffer read, cursor box detect (with margin handling)
--   S5: font ROM read, pixel extract, output mux (text vs matte vs bg)
--
-- Resources:
--   - Font ROM: 64 glyphs x 8 rows x 8 bits = 4 kbit (LUT-ROM)
--   - Text buffer: 32 entries x 6 bits = 192 register bits (2 lines x 16)
--   - Bypass delay: 4 stages x (3*10 + 4) bits
--
-- Font layout note: glyphs are stored shifted right by 1 column inside the
-- 8x8 cell, so column 0 is always blank, columns 1..5 hold the visible
-- glyph, and columns 6..7 are the inter-character gap. Row 7 is the
-- inter-line gap. The cursor box uses these blank cells to draw a frame
-- that fully encloses the letter without overlapping any pixel.
--
-- Register Map:
--   reg(0)    : H Position       (0..1023, scaled to ~0..2046 px)
--   reg(1)    : V Position       (0..1023, scaled to ~0..2046 lines)
--   reg(2)    : Size             (comparator chain: 1x/2x/4x/8x/16x)
--   reg(3)    : Color / Erase    Dual purpose:
--                                  Edit Mode OFF -> top 3 bits select 1 of 8
--                                                   preset colors (latched)
--                                  Edit Mode ON  -> any motion of the top
--                                                   6 bits blanks the char
--                                                   currently under the
--                                                   editing cursor.
--   reg(4)    : Cursor           top 5 bits select 1 of 32 cells; bit 4 is
--                                line index (0/1), bits 3..0 are column.
--   reg(5)    : Letter           (top 6 bits = char code 0..63)
--   reg(6)(0) : Edit Mode        (1 = enable cursor + writes)
--   reg(6)(1) : Background       (0 = passthrough video, 1 = solid black)
--   reg(6)(2) : Matte            (1 = text glyphs reveal incoming video)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.titler_font_pkg.all;

architecture titler of program_top is

    --==========================================================================
    -- Constants
    --==========================================================================
    constant C_LATENCY      : integer := 5;
    constant C_DELAY_DEPTH  : integer := C_LATENCY - 1;  -- shift register depth
    constant C_TEXT_COLS    : integer := 16;             -- chars per line
    constant C_TEXT_ROWS    : integer := 2;              -- number of lines
    constant C_TEXT_LEN     : integer := C_TEXT_COLS * C_TEXT_ROWS;  -- 32
    constant C_COL_BIT      : integer := 4;              -- bits to address cols
    constant C_LINE_BIT     : integer := 1;              -- bits to address rows
    constant C_CURSOR_BIT   : integer := C_COL_BIT + C_LINE_BIT;     -- 5

    --==========================================================================
    -- Decoded parameters (concurrent)
    --==========================================================================
    signal s_h_pos       : unsigned(9 downto 0);
    signal s_v_pos       : unsigned(9 downto 0);
    signal s_size_raw    : unsigned(9 downto 0);
    signal s_size_shift  : unsigned(2 downto 0);  -- 0..4 = 1x..16x
    signal s_cursor_line : unsigned(C_LINE_BIT - 1 downto 0);
    signal s_cursor_col  : unsigned(C_COL_BIT  - 1 downto 0);
    signal s_letter      : unsigned(5 downto 0);
    signal s_edit_mode   : std_logic;
    signal s_bg_black    : std_logic;
    signal s_matte       : std_logic;

    -- Color is latched: only updates when not in edit mode (knob 4 is then
    -- repurposed as the Erase trigger).
    signal s_color_sel   : unsigned(2 downto 0) := "000";  -- 8 preset colors

    -- Erase trigger: top 6 bits of reg(3). In edit mode, any change of these
    -- bits blanks the character currently under the editing cursor.
    signal s_erase_trig    : unsigned(5 downto 0);

    -- Edge detection for non-destructive writes.
    signal prev_letter     : unsigned(5 downto 0) := (others => '0');
    signal prev_erase_trig : unsigned(5 downto 0) := (others => '0');

    --==========================================================================
    -- Position counters (S1)
    --==========================================================================
    signal h_count      : unsigned(11 downto 0) := (others => '0');
    signal v_count      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    --==========================================================================
    -- Text buffer (32 chars x 6 bits = 2 lines x 16 cols)
    -- Indexing: text_buf(line * 16 + col).
    -- Default line 0: "LZX VIDEOMANCER!"
    -- Default line 1: 16 spaces (user fills in via edit mode).
    --==========================================================================
    type t_text_buf is array (0 to C_TEXT_LEN - 1) of unsigned(5 downto 0);
    signal text_buf : t_text_buf := (
        -- Line 0: "LZX VIDEOMANCER!"
        to_unsigned(12, 6),  -- L
        to_unsigned(26, 6),  -- Z
        to_unsigned(24, 6),  -- X
        to_unsigned( 0, 6),  -- (space)
        to_unsigned(22, 6),  -- V
        to_unsigned( 9, 6),  -- I
        to_unsigned( 4, 6),  -- D
        to_unsigned( 5, 6),  -- E
        to_unsigned(15, 6),  -- O
        to_unsigned(13, 6),  -- M
        to_unsigned( 1, 6),  -- A
        to_unsigned(14, 6),  -- N
        to_unsigned( 3, 6),  -- C
        to_unsigned( 5, 6),  -- E
        to_unsigned(18, 6),  -- R
        to_unsigned(39, 6),  -- !
        -- Line 1: blank
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6),
        to_unsigned( 0, 6)
    );

    --==========================================================================
    -- Pipeline stage signals
    --==========================================================================
    -- Stage 2: pixel deltas
    -- s2_dx: physical px from text origin (no shift). dx in [0, glyph area].
    -- s2_dy: physical px from text origin shifted by -1 so dy=0 is the
    --        1px-tall top margin row above line 0. dy in [1, glyph area]
    --        is the actual glyph rows.
    signal s2_dx       : unsigned(11 downto 0);
    signal s2_dy       : unsigned(11 downto 0);
    signal s2_dx_valid : std_logic;
    signal s2_dy_valid : std_logic;
    signal s2_size     : unsigned(2 downto 0);

    -- Stage 3: char/line/glyph indices
    signal s3_char_idx     : unsigned(C_COL_BIT  - 1 downto 0);
    signal s3_line_idx     : unsigned(C_LINE_BIT - 1 downto 0);
    signal s3_glyph_x      : unsigned(2 downto 0);
    signal s3_glyph_y      : unsigned(2 downto 0);
    signal s3_in_text      : std_logic;
    signal s3_in_top_margin: std_logic;

    -- Stage 4: char code + cursor box
    signal s4_char_code    : unsigned(5 downto 0);
    signal s4_glyph_x      : unsigned(2 downto 0);
    signal s4_glyph_y      : unsigned(2 downto 0);
    signal s4_in_text      : std_logic;
    signal s4_in_top_margin: std_logic;
    signal s4_cursor_box   : std_logic;

    --==========================================================================
    -- Bypass delay line (matches pipeline latency)
    --==========================================================================
    type t_pix_delay is array (0 to C_DELAY_DEPTH - 1)
        of std_logic_vector(9 downto 0);
    type t_sync_delay is array (0 to C_DELAY_DEPTH - 1) of std_logic;

    signal bypass_y     : t_pix_delay;
    signal bypass_u     : t_pix_delay;
    signal bypass_v     : t_pix_delay;
    signal bypass_hs    : t_sync_delay := (others => '1');
    signal bypass_vs    : t_sync_delay := (others => '1');
    signal bypass_field : t_sync_delay := (others => '1');
    signal bypass_avid  : t_sync_delay := (others => '0');

    --==========================================================================
    -- Text color palette (combinational)
    --==========================================================================
    signal text_color_y : unsigned(9 downto 0);
    signal text_color_u : unsigned(9 downto 0);
    signal text_color_v : unsigned(9 downto 0);

begin

    --==========================================================================
    -- Register decoding
    --==========================================================================
    s_h_pos       <= unsigned(registers_in(0));
    s_v_pos       <= unsigned(registers_in(1));
    s_size_raw    <= unsigned(registers_in(2));
    s_erase_trig  <= unsigned(registers_in(3)(9 downto 4));  -- top 6 bits
    -- Editing cursor: top 5 bits of reg(4) walk all 32 cells row-major.
    -- Bit 4 (MSB) is the line index, bits 3..0 are the column index.
    s_cursor_line <= unsigned(registers_in(4)(9 downto 9));
    s_cursor_col  <= unsigned(registers_in(4)(8 downto 5));
    s_letter      <= unsigned(registers_in(5)(9 downto 4));  -- top 6 bits
    s_edit_mode   <= registers_in(6)(0);
    s_bg_black    <= registers_in(6)(1);
    s_matte       <= registers_in(6)(2);

    -- Size selector: 5-zone comparator chain over the raw 10-bit pot value.
    s_size_shift <= "000" when s_size_raw < to_unsigned(205, 10) else
                    "001" when s_size_raw < to_unsigned(410, 10) else
                    "010" when s_size_raw < to_unsigned(615, 10) else
                    "011" when s_size_raw < to_unsigned(820, 10) else
                    "100";

    -- Color is latched while in edit mode (knob 4 then becomes Erase trigger).
    p_color_latch : process(clk)
    begin
        if rising_edge(clk) then
            if s_edit_mode = '0' then
                s_color_sel <= unsigned(registers_in(3)(9 downto 7));
            end if;
        end if;
    end process p_color_latch;

    --==========================================================================
    -- Color palette (concurrent select) - 8 presets
    -- Approximate BT.601 YUV constants. Y is full-swing 10-bit (~64..940);
    -- U and V are centered at 512.
    --==========================================================================
    with s_color_sel select
        text_color_y <= to_unsigned(940, 10) when "000",  -- White
                        to_unsigned(290, 10) when "001",  -- Red
                        to_unsigned(550, 10) when "010",  -- Orange
                        to_unsigned(870, 10) when "011",  -- Yellow
                        to_unsigned(620, 10) when "100",  -- Green
                        to_unsigned(820, 10) when "101",  -- Cyan
                        to_unsigned(170, 10) when "110",  -- Blue
                        to_unsigned(440, 10) when "111",  -- Magenta
                        to_unsigned(940, 10) when others;
    with s_color_sel select
        text_color_u <= to_unsigned(512, 10) when "000",
                        to_unsigned(425, 10) when "001",
                        to_unsigned(350, 10) when "010",
                        to_unsigned(130, 10) when "011",
                        to_unsigned(195, 10) when "100",
                        to_unsigned(600, 10) when "101",
                        to_unsigned(880, 10) when "110",
                        to_unsigned(850, 10) when "111",
                        to_unsigned(512, 10) when others;
    with s_color_sel select
        text_color_v <= to_unsigned(512, 10) when "000",
                        to_unsigned(900, 10) when "001",
                        to_unsigned(750, 10) when "010",
                        to_unsigned(560, 10) when "011",
                        to_unsigned(143, 10) when "100",
                        to_unsigned(128, 10) when "101",
                        to_unsigned(580, 10) when "110",
                        to_unsigned(900, 10) when "111",
                        to_unsigned(512, 10) when others;

    --==========================================================================
    -- S1: Position counters
    -- h_count resets at hsync_n falling edge, v_count at vsync_n falling edge.
    --==========================================================================
    p_position : process(clk)
        variable v_hs_falling : std_logic;
        variable v_vs_falling : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_hs_falling := prev_hsync_n and (not data_in.hsync_n);
            v_vs_falling := prev_vsync_n and (not data_in.vsync_n);

            if v_hs_falling = '1' then
                h_count <= (others => '0');
            else
                h_count <= h_count + 1;
            end if;

            if v_vs_falling = '1' then
                v_count <= (others => '0');
            elsif v_hs_falling = '1' then
                v_count <= v_count + 1;
            end if;
        end if;
    end process p_position;

    --==========================================================================
    -- Text buffer write (Edit Mode)
    -- Non-destructive cursor: a write only happens when the user is actually
    -- turning the letter knob (knob 6) or the erase knob (knob 4). Both
    -- writes target the SAME cell - the one currently under the editing
    -- cursor (knob 5). Moving the cursor knob alone never disturbs the
    -- buffer; sweeping it while continuously turning knob 4 erases each
    -- cell the cursor passes over.
    --==========================================================================
    p_text_edit : process(clk)
        variable v_flat_idx : integer range 0 to C_TEXT_LEN - 1;
    begin
        if rising_edge(clk) then
            prev_letter     <= s_letter;
            prev_erase_trig <= s_erase_trig;

            v_flat_idx := to_integer(s_cursor_line) * C_TEXT_COLS
                          + to_integer(s_cursor_col);

            if s_edit_mode = '1' then
                if s_letter /= prev_letter then
                    text_buf(v_flat_idx) <= s_letter;
                elsif s_erase_trig /= prev_erase_trig then
                    text_buf(v_flat_idx) <= to_unsigned(0, 6);
                end if;
            end if;
        end if;
    end process p_text_edit;

    --==========================================================================
    -- S2: Pixel deltas relative to text origin
    -- h/v position knobs (10-bit) are shifted left by 1 to give
    -- a ~0..2046 range that covers 1080p horizontal/vertical extent.
    --
    -- The vertical dimension is offset by -1 line so that dy == 0 represents
    -- a 1px-tall margin row above the first glyph row. dy >= 1 is the actual
    -- glyph area; the cursor box can draw its top edge in the margin row when
    -- the cursor is on line 0.
    --==========================================================================
    p_stage2 : process(clk)
        variable v_h_origin : unsigned(11 downto 0);
        variable v_v_origin : unsigned(11 downto 0);
        variable v_v_plus1  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            v_h_origin := shift_left(resize(s_h_pos, 12), 1);
            v_v_origin := shift_left(resize(s_v_pos, 12), 1);
            v_v_plus1  := resize(v_count, 13) + 1;

            if h_count >= v_h_origin then
                s2_dx       <= h_count - v_h_origin;
                s2_dx_valid <= '1';
            else
                s2_dx       <= (others => '0');
                s2_dx_valid <= '0';
            end if;

            if v_v_plus1 >= resize(v_v_origin, 13) then
                s2_dy       <= resize(v_v_plus1 - resize(v_v_origin, 13), 12);
                s2_dy_valid <= '1';
            else
                s2_dy       <= (others => '0');
                s2_dy_valid <= '0';
            end if;

            s2_size <= s_size_shift;
        end if;
    end process p_stage2;

    --==========================================================================
    -- S3: Char + line + glyph indices and bounds.
    --
    -- Per size:                cell   line_h  text_w   text_h_glyphs
    --   Size 0 (1x):           8 px   8 px    128 px   16 px (2 lines)
    --   Size 1 (2x):          16 px  16 px    256 px   32 px
    --   Size 2 (4x):          32 px  32 px    512 px   64 px
    --   Size 3 (8x):          64 px  64 px   1024 px  128 px
    --   Size 4 (16x):        128 px 128 px   2048 px  256 px
    --
    -- The vertical region also includes a 1px-tall margin row above line 0
    -- (s2_dy == 0). For dy >= 1, the (dy - 1) value is the offset into the
    -- glyph area, from which line_idx and glyph_y are bit-sliced.
    --==========================================================================
    p_stage3 : process(clk)
        variable v_in_x   : std_logic;
        variable v_in_y   : std_logic;
        variable v_dy_g   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            v_in_x := '0';
            v_in_y := '0';

            -- dy with margin removed. Underflows to all-1s when in margin
            -- (s2_dy = 0); the in_top_margin flag covers that case.
            v_dy_g := s2_dy - 1;

            case s2_size is
                when "000" =>  -- 1x
                    s3_char_idx <= s2_dx(6 downto 3);
                    s3_glyph_x  <= s2_dx(2 downto 0);
                    s3_line_idx <= v_dy_g(3 downto 3);
                    s3_glyph_y  <= v_dy_g(2 downto 0);
                    if s2_dx(11 downto 7) = "00000" then v_in_x := '1'; end if;
                    if s2_dy <= to_unsigned(16, 12) then v_in_y := '1'; end if;

                when "001" =>  -- 2x
                    s3_char_idx <= s2_dx(7 downto 4);
                    s3_glyph_x  <= s2_dx(3 downto 1);
                    s3_line_idx <= v_dy_g(4 downto 4);
                    s3_glyph_y  <= v_dy_g(3 downto 1);
                    if s2_dx(11 downto 8) = "0000" then v_in_x := '1'; end if;
                    if s2_dy <= to_unsigned(32, 12) then v_in_y := '1'; end if;

                when "010" =>  -- 4x
                    s3_char_idx <= s2_dx(8 downto 5);
                    s3_glyph_x  <= s2_dx(4 downto 2);
                    s3_line_idx <= v_dy_g(5 downto 5);
                    s3_glyph_y  <= v_dy_g(4 downto 2);
                    if s2_dx(11 downto 9) = "000" then v_in_x := '1'; end if;
                    if s2_dy <= to_unsigned(64, 12) then v_in_y := '1'; end if;

                when "011" =>  -- 8x
                    s3_char_idx <= s2_dx(9 downto 6);
                    s3_glyph_x  <= s2_dx(5 downto 3);
                    s3_line_idx <= v_dy_g(6 downto 6);
                    s3_glyph_y  <= v_dy_g(5 downto 3);
                    if s2_dx(11 downto 10) = "00" then v_in_x := '1'; end if;
                    if s2_dy <= to_unsigned(128, 12) then v_in_y := '1'; end if;

                when others =>  -- 16x
                    s3_char_idx <= s2_dx(10 downto 7);
                    s3_glyph_x  <= s2_dx(6 downto 4);
                    s3_line_idx <= v_dy_g(7 downto 7);
                    s3_glyph_y  <= v_dy_g(6 downto 4);
                    if s2_dx(11) = '0' then v_in_x := '1'; end if;
                    if s2_dy <= to_unsigned(256, 12) then v_in_y := '1'; end if;
            end case;

            s3_in_text <= v_in_x and v_in_y and s2_dx_valid and s2_dy_valid;
            if s2_dy = to_unsigned(0, 12) then
                s3_in_top_margin <= '1';
            else
                s3_in_top_margin <= '0';
            end if;
        end if;
    end process p_stage3;

    --==========================================================================
    -- S4: Text buffer read + cursor box detect
    --
    -- A single editing cursor (knob 5) walks all 32 cells. When edit mode is
    -- on, a 1px-thick box is drawn around the perimeter of the active cell:
    --   left  edge = glyph_x "000"  (column 0 of the cell, blank gap)
    --   right edge = glyph_x "111"  (column 7 of the cell, blank gap)
    --   bottom edge= glyph_y "111"  (row 7 of the cell, inter-line gap)
    --   top edge   = either row 7 of the line above (if cursor on line 1)
    --                or the 1-pixel top margin row (if cursor on line 0).
    -- All four edges fall on cells/rows that the font has guaranteed to be
    -- blank, so the box never overlaps any letter pixel.
    --==========================================================================
    p_stage4 : process(clk)
        variable v_box      : std_logic;
        variable v_flat_idx : integer range 0 to C_TEXT_LEN - 1;
    begin
        if rising_edge(clk) then
            v_flat_idx := to_integer(s3_line_idx) * C_TEXT_COLS
                          + to_integer(s3_char_idx);
            s4_char_code     <= text_buf(v_flat_idx);
            s4_glyph_x       <= s3_glyph_x;
            s4_glyph_y       <= s3_glyph_y;
            s4_in_text       <= s3_in_text;
            s4_in_top_margin <= s3_in_top_margin;

            v_box := '0';
            if s_edit_mode = '1' and s3_char_idx = s_cursor_col then
                if s3_in_top_margin = '1' then
                    -- Top margin row only draws the top edge for a cursor
                    -- positioned on line 0.
                    if s_cursor_line(0) = '0' then
                        v_box := '1';
                    end if;
                elsif s3_line_idx = s_cursor_line then
                    -- Inside the cursor's own cell: left/right/bottom edges.
                    if s3_glyph_x = "000" or s3_glyph_x = "111"
                       or s3_glyph_y = "111" then
                        v_box := '1';
                    end if;
                elsif s_cursor_line(0) = '1' and s3_line_idx(0) = '0'
                      and s3_glyph_y = "111" then
                    -- Cursor on line 1: its top edge lives in row 7 of line 0
                    -- (the inter-line gap row, guaranteed blank).
                    v_box := '1';
                end if;
            end if;
            s4_cursor_box <= v_box;
        end if;
    end process p_stage4;

    --==========================================================================
    -- S5: Font ROM read + pixel extract + output mux
    -- The font row is looked up combinationally and registered into data_out.
    --==========================================================================
    p_stage5 : process(clk)
        variable v_font_row  : std_logic_vector(7 downto 0);
        variable v_pixel_on  : std_logic;
        variable v_show_text : std_logic;
        variable v_bg_y      : std_logic_vector(9 downto 0);
        variable v_bg_u      : std_logic_vector(9 downto 0);
        variable v_bg_v      : std_logic_vector(9 downto 0);
    begin
        if rising_edge(clk) then
            v_font_row := C_FONT_ROM(to_integer(s4_char_code),
                                     to_integer(s4_glyph_y));
            -- Mask the font lookup in the top margin row: s3 sets a
            -- garbage line/glyph index there (v_dy_g underflows), so we
            -- must not treat any font bit as a real pixel.
            v_pixel_on := v_font_row(7 - to_integer(s4_glyph_x))
                          and not s4_in_top_margin;

            v_show_text := s4_in_text and (v_pixel_on or s4_cursor_box);

            -- Background select: passthrough or solid black
            if s_bg_black = '1' then
                v_bg_y := std_logic_vector(to_unsigned(64, 10));
                v_bg_u := std_logic_vector(to_unsigned(512, 10));
                v_bg_v := std_logic_vector(to_unsigned(512, 10));
            else
                v_bg_y := bypass_y(C_DELAY_DEPTH - 1);
                v_bg_u := bypass_u(C_DELAY_DEPTH - 1);
                v_bg_v := bypass_v(C_DELAY_DEPTH - 1);
            end if;

            if v_show_text = '1' then
                if s_matte = '1' and s4_cursor_box = '0' then
                    -- Matte mode: glyph pixels reveal the incoming video
                    -- (the cursor box still uses solid text color so it
                    -- remains visible against any background).
                    data_out.y <= bypass_y(C_DELAY_DEPTH - 1);
                    data_out.u <= bypass_u(C_DELAY_DEPTH - 1);
                    data_out.v <= bypass_v(C_DELAY_DEPTH - 1);
                else
                    data_out.y <= std_logic_vector(text_color_y);
                    data_out.u <= std_logic_vector(text_color_u);
                    data_out.v <= std_logic_vector(text_color_v);
                end if;
            else
                data_out.y <= v_bg_y;
                data_out.u <= v_bg_u;
                data_out.v <= v_bg_v;
            end if;

            data_out.avid    <= bypass_avid(C_DELAY_DEPTH - 1);
            data_out.hsync_n <= bypass_hs(C_DELAY_DEPTH - 1);
            data_out.vsync_n <= bypass_vs(C_DELAY_DEPTH - 1);
            data_out.field_n <= bypass_field(C_DELAY_DEPTH - 1);
        end if;
    end process p_stage5;

    --==========================================================================
    -- Bypass delay line shift register
    -- Index 0 = newest (just-arrived data_in),
    -- Index DEPTH-1 = oldest (consumed by p_stage5).
    --==========================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            bypass_y     <= data_in.y       & bypass_y(0 to C_DELAY_DEPTH - 2);
            bypass_u     <= data_in.u       & bypass_u(0 to C_DELAY_DEPTH - 2);
            bypass_v     <= data_in.v       & bypass_v(0 to C_DELAY_DEPTH - 2);
            bypass_hs    <= data_in.hsync_n & bypass_hs(0 to C_DELAY_DEPTH - 2);
            bypass_vs    <= data_in.vsync_n & bypass_vs(0 to C_DELAY_DEPTH - 2);
            bypass_field <= data_in.field_n & bypass_field(0 to C_DELAY_DEPTH - 2);
            bypass_avid  <= data_in.avid    & bypass_avid(0 to C_DELAY_DEPTH - 2);
        end if;
    end process p_delay;

end architecture titler;
