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
--   S1: position counters (h_count, v_count) and Bresenham-style position
--       accumulators (hpos_acc, vpos_acc) that map physical pixels/lines to
--       a glyph-pixel index at any integer scale 1..32x. The accumulator
--       walks by step = floor(2^16 / scale) per pixel (or per line for the
--       vertical accumulator), so no multiplier is required.
--   S2: extract char_idx / line_idx / glyph_x / glyph_y from the integer
--       part of the accumulators; compute in_text + in_top_margin bounds.
--   S3: text buffer read, cursor box detect (with margin handling).
--   S4: font ROM read; register the 8-bit row plus the pass-through control
--       bits. Splitting the ROM out of the final mux gives the output stage
--       enough slack to clear 74.25 MHz with margin.
--   S5: pixel extract from the registered font row, output mux (text vs
--       matte vs background).
--
-- Resources:
--   - Font ROM: 64 glyphs x 8 rows x 8 bits = 4 kbit (LUT-ROM)
--   - Text buffer: 32 entries x 6 bits = 192 register bits (2 lines x 16)
--   - Step LUT: 32 entries x 17 bits ~= 544 bits (LUT-ROM)
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
--   reg(2)    : Size             top 5 bits + 1 => continuous scale 1..32x
--   reg(3)    : Color / Erase    Dual purpose:
--                                  Edit Mode OFF -> top 3 bits select 1 of 8
--                                                   preset colors (latched)
--                                  Edit Mode ON  -> top 5 bits act as a
--                                                   second cursor: the
--                                                   cursor jumps to the
--                                                   knob's position and the
--                                                   cell there is blanked.
--                                                   Sweeping the knob erases
--                                                   each cell it passes.
--   reg(4)    : Cursor           top 5 bits select 1 of 32 cells (non-
--                                destructive). bit 4 is line index (0/1),
--                                bits 3..0 are column.
--   reg(5)    : Letter           (top 6 bits = char code 0..63)
--   reg(6)(0) : Edit Mode        (1 = enable cursor + writes)
--   reg(6)(1) : Background       (0 = passthrough video, 1 = solid black)
--   reg(6)(2) : Matte            (1 = text glyphs reveal incoming video)
--   reg(6)(3) : Font             (0 = default, 1 = sci-fi)

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

    -- Bresenham position-accumulator format:
    --   bits [C_POS_BITS-1 : C_POS_FRAC]  : 8-bit integer glyph-pixel index
    --   bits [C_POS_FRAC-1 : 0]           : 16-bit fractional remainder
    -- A "past text" sentinel is encoded by forcing the accumulator to all-1s,
    -- which sets bit [C_POS_BITS-1] (the integer-part MSB) and lets us test
    -- "outside the text band" with a single-bit compare.
    constant C_SCALE_MAX    : integer := 32;
    constant C_POS_FRAC     : integer := 16;
    constant C_POS_INT      : integer := 8;
    constant C_POS_BITS     : integer := C_POS_INT + C_POS_FRAC;     -- 24
    constant C_STEP_BITS    : integer := C_POS_FRAC + 1;             -- 17

    type t_step_lut is array (1 to C_SCALE_MAX)
        of unsigned(C_STEP_BITS - 1 downto 0);

    function f_make_step_lut return t_step_lut is
        variable v : t_step_lut;
    begin
        for i in 1 to C_SCALE_MAX loop
            v(i) := to_unsigned(2 ** C_POS_FRAC / i, C_STEP_BITS);
        end loop;
        return v;
    end function;

    constant C_STEP_LUT : t_step_lut := f_make_step_lut;

    --==========================================================================
    -- Decoded parameters (concurrent)
    --==========================================================================
    signal s_h_pos       : unsigned(9 downto 0);
    signal s_v_pos       : unsigned(9 downto 0);
    signal s_size_raw    : unsigned(9 downto 0);
    signal s_size_scale  : integer range 1 to C_SCALE_MAX;
    signal s_size_step   : unsigned(C_STEP_BITS - 1 downto 0);
    signal s_letter      : unsigned(5 downto 0);
    signal s_edit_mode   : std_logic;
    signal s_bg_black    : std_logic;
    signal s_matte       : std_logic;
    signal s_font_sel    : std_logic;

    -- Color is latched: only updates when not in edit mode (knob 4 is then
    -- repurposed as the Erase-and-scroll knob).
    signal s_color_sel   : unsigned(2 downto 0) := "000";  -- 8 preset colors

    -- Dual cursor sources: knob 5 (top 5 bits of reg(4)) walks the cursor
    -- non-destructively; knob 4 (top 5 bits of reg(3)) also drags the cursor
    -- but blanks every cell it lands on. r_cursor is the latched "active
    -- cursor" — it follows whichever source most recently moved.
    signal s_cursor_5p     : unsigned(C_CURSOR_BIT - 1 downto 0);
    signal s_erase_5p      : unsigned(C_CURSOR_BIT - 1 downto 0);
    signal r_cursor        : unsigned(C_CURSOR_BIT - 1 downto 0)
                             := (others => '0');
    signal prev_cursor_5p  : unsigned(C_CURSOR_BIT - 1 downto 0)
                             := (others => '0');
    signal prev_erase_5p   : unsigned(C_CURSOR_BIT - 1 downto 0)
                             := (others => '0');

    -- Downstream consumers see the cursor as (line, col) split, derived from
    -- the latched r_cursor.
    signal s_cursor_line : unsigned(C_LINE_BIT - 1 downto 0);
    signal s_cursor_col  : unsigned(C_COL_BIT  - 1 downto 0);

    -- Edge detection for non-destructive letter writes.
    signal prev_letter     : unsigned(5 downto 0) := (others => '0');

    -- H/V origin (signed conceptually): split into an unsigned magnitude
    -- and a sign flag. When the origin is positive the accumulator behaves
    -- like before — sentinel at hsync/vsync, then transitions to 0 when the
    -- physical scan reaches s_h_origin / s_v_origin. When the origin is
    -- negative the text has been scrolled off-screen to the left/top, so
    -- the accumulator is pre-loaded with hpos_init / vpos_init (computed in
    -- vblank by p_init_compute) — that captures how many glyph-pixels of
    -- the text have already scrolled past the screen edge.
    signal s_h_origin    : unsigned(11 downto 0);
    signal s_v_origin    : unsigned(11 downto 0);
    signal h_origin_neg  : std_logic := '0';
    signal v_origin_neg  : std_logic := '0';
    signal h_origin_abs  : unsigned(12 downto 0) := (others => '0');
    signal v_origin_abs  : unsigned(12 downto 0) := (others => '0');
    signal hpos_init     : unsigned(C_POS_BITS - 1 downto 0)
                           := (others => '1');
    signal vpos_init     : unsigned(C_POS_BITS - 1 downto 0)
                           := (others => '1');

    -- Vblank divide-by-subtraction state machine. Computes the integer
    -- glyph-pixel offset for hpos_init / vpos_init when the corresponding
    -- origin is negative.
    type t_sm_state is (SM_IDLE, SM_H_DIV, SM_V_DIV);
    signal sm_state    : t_sm_state := SM_IDLE;
    signal sm_residue  : unsigned(12 downto 0) := (others => '0');
    signal sm_quotient : unsigned(7 downto 0)  := (others => '0');

    --==========================================================================
    -- Position counters (S1)
    --==========================================================================
    signal h_count      : unsigned(11 downto 0) := (others => '0');
    signal v_count      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    -- Bresenham accumulators (S1). Initialized to the sentinel value so that
    -- the first scan starts cleanly "outside" the text band.
    -- vpos_acc semantics (after the cursor-margin rework):
    --   integer part 0          : margin row (lasts `scale` physical lines,
    --                             so the cursor box top edge has the same
    --                             thickness as its side/bottom edges)
    --   integer part 1..16      : text rows; text_row_idx = int_part - 1
    --                             decodes into line_idx + glyph_y
    --   integer part >= 17      : past text
    signal hpos_acc     : unsigned(C_POS_BITS - 1 downto 0) := (others => '1');
    signal vpos_acc     : unsigned(C_POS_BITS - 1 downto 0) := (others => '1');

    -- Frame counter: advances once per vsync. Drives the cycling cursor
    -- color so the edit-mode box is visually distinct from the text.
    signal frame_counter : unsigned(7 downto 0) := (others => '0');

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
    -- Stage 2: char/line/glyph indices extracted from accumulator integer
    -- parts. in_top_margin is the row of padding above line 0 (used by the
    -- cursor-box logic for the top edge).
    signal s2_char_idx     : unsigned(C_COL_BIT  - 1 downto 0);
    signal s2_line_idx     : unsigned(C_LINE_BIT - 1 downto 0);
    signal s2_glyph_x      : unsigned(2 downto 0);
    signal s2_glyph_y      : unsigned(2 downto 0);
    signal s2_in_text      : std_logic;
    signal s2_in_top_margin: std_logic;

    -- Stage 3: char code + cursor box + edit-mode underscore guide
    signal s3_char_code    : unsigned(5 downto 0);
    signal s3_glyph_x      : unsigned(2 downto 0);
    signal s3_glyph_y      : unsigned(2 downto 0);
    signal s3_in_text      : std_logic;
    signal s3_in_top_margin: std_logic;
    signal s3_cursor_box   : std_logic;
    signal s3_underscore   : std_logic;

    -- Stage 4: registered font-row + pass-through control bits. Putting the
    -- font ROM in its own stage breaks the long S3->output mux chain.
    signal s4_font_row     : std_logic_vector(7 downto 0);
    signal s4_glyph_x      : unsigned(2 downto 0);
    signal s4_in_text      : std_logic;
    signal s4_in_top_margin: std_logic;
    signal s4_cursor_box   : std_logic;
    signal s4_underscore   : std_logic;

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

    --==========================================================================
    -- Cursor color palette (combinational) - cycles through 8 saturated hues
    -- driven by the top bits of the frame counter. The cycle advances roughly
    -- every 8 frames (~135 ms at 60 Hz), so the cursor box pulses through a
    -- full rainbow in about a second.
    --==========================================================================
    signal cursor_color_sel : unsigned(2 downto 0);
    signal cursor_color_y   : unsigned(9 downto 0);
    signal cursor_color_u   : unsigned(9 downto 0);
    signal cursor_color_v   : unsigned(9 downto 0);

begin

    --==========================================================================
    -- Register decoding
    --==========================================================================
    s_h_pos       <= unsigned(registers_in(0));
    s_v_pos       <= unsigned(registers_in(1));
    s_size_raw    <= unsigned(registers_in(2));
    -- Two cursor sources: top 5 bits of reg(4) (knob 5) walks the cursor
    -- without erasing; top 5 bits of reg(3) (knob 4) also walks the cursor
    -- but blanks every cell it lands on. Bit 4 is line index, bits 3..0 are
    -- the column index in both encodings.
    s_cursor_5p   <= unsigned(registers_in(4)(9 downto 5));
    s_erase_5p    <= unsigned(registers_in(3)(9 downto 5));
    s_cursor_line <= r_cursor(C_CURSOR_BIT - 1 downto C_COL_BIT);
    s_cursor_col  <= r_cursor(C_COL_BIT - 1 downto 0);
    s_letter      <= unsigned(registers_in(5)(9 downto 4));  -- top 6 bits
    s_edit_mode   <= registers_in(6)(0);
    s_bg_black    <= registers_in(6)(1);
    s_matte       <= registers_in(6)(2);
    s_font_sel    <= registers_in(6)(3);  -- toggle 10: 0 = default, 1 = sci-fi

    -- Size: top 5 bits of the 10-bit pot give 0..31; add 1 for scale 1..32.
    s_size_scale <= to_integer(unsigned(s_size_raw(9 downto 5))) + 1;
    s_size_step  <= C_STEP_LUT(s_size_scale);

    -- H/V Pos knobs set the CENTER of the text rectangle, so growing the
    -- size knob expands the text symmetrically around the configured center
    -- instead of dragging the lower-right corner.
    --
    --   center_h = h_pos * 4     (4x range so the text can fully clear the
    --                             right side of the screen at scale=32)
    --   half_w   = scale * 64    (text width is 128 glyph-px * scale)
    --   origin_h = center_h - half_w   (signed; negative = off-screen left)
    --
    -- Vertical math is the same with half_h = scale * 8 (16 glyph-rows /2).
    -- All halves and the wider knob mapping are pure left-shifts, so no
    -- multiplier is needed. We register the result to keep the centering
    -- subtractor off the per-pixel accumulator critical path.
    --
    -- The origin is split into a sign bit (h_origin_neg) and an unsigned
    -- magnitude (h_origin_abs). When the origin is non-negative we hand
    -- s_h_origin to the accumulator as the per-line reset point exactly
    -- like before; when it's negative we force s_h_origin to a value
    -- h_count can never reach (0xFFF) and let p_init_compute pre-load
    -- hpos_init with the right starting accumulator value.
    p_center_origin : process(clk)
        variable v_center_h : unsigned(12 downto 0);
        variable v_center_v : unsigned(12 downto 0);
        variable v_half_w   : unsigned(12 downto 0);
        variable v_half_h   : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            v_center_h := shift_left(resize(s_h_pos, 13), 2);
            v_center_v := shift_left(resize(s_v_pos, 13), 2);
            v_half_w   := shift_left(to_unsigned(s_size_scale, 13), 6);
            v_half_h   := shift_left(to_unsigned(s_size_scale, 13), 3);

            if v_center_h >= v_half_w then
                h_origin_neg <= '0';
                h_origin_abs <= resize(v_center_h - v_half_w, 13);
                s_h_origin   <= resize(v_center_h - v_half_w, 12);
            else
                h_origin_neg <= '1';
                h_origin_abs <= resize(v_half_w - v_center_h, 13);
                s_h_origin   <= (others => '1');
            end if;

            if v_center_v >= v_half_h then
                v_origin_neg <= '0';
                v_origin_abs <= resize(v_center_v - v_half_h, 13);
                s_v_origin   <= resize(v_center_v - v_half_h, 12);
            else
                v_origin_neg <= '1';
                v_origin_abs <= resize(v_half_h - v_center_v, 13);
                s_v_origin   <= (others => '1');
            end if;
        end if;
    end process p_center_origin;

    --==========================================================================
    -- p_init_compute: pre-loads hpos_init / vpos_init at the start of every
    -- frame. The hard math is "what value should the Bresenham accumulator
    -- have at the first physical pixel of the scan, if the text has been
    -- scrolled off-screen by |origin| physical pixels?". That's
    --   init = |origin| * step  (= |origin| << 16 / scale, in q16)
    -- which avoids a multiplier by doing a /scale via repeated subtraction.
    -- Each subtraction step advances the integer quotient (= number of glyph
    -- pixels already scrolled past) by 1; after |origin|/scale iterations
    -- the quotient is loaded into the integer part of *_init. We ignore the
    -- residue, accepting up to one physical-pixel of subpixel rounding.
    --
    -- The state machine runs for at most ~128 cycles per frame (H side) plus
    -- ~16 (V side), well inside the vblank interval, so the result is ready
    -- by the time active video begins.
    p_init_compute : process(clk)
        variable v_vs_falling : std_logic;
    begin
        if rising_edge(clk) then
            v_vs_falling := prev_vsync_n and (not data_in.vsync_n);

            case sm_state is
                when SM_IDLE =>
                    if v_vs_falling = '1' then
                        if h_origin_neg = '1' then
                            sm_state    <= SM_H_DIV;
                            sm_residue  <= h_origin_abs;
                            sm_quotient <= (others => '0');
                        else
                            hpos_init <= (others => '1');
                            if v_origin_neg = '1' then
                                sm_state    <= SM_V_DIV;
                                sm_residue  <= v_origin_abs;
                                sm_quotient <= (others => '0');
                            else
                                vpos_init <= (others => '1');
                            end if;
                        end if;
                    end if;

                when SM_H_DIV =>
                    if sm_residue < to_unsigned(s_size_scale, 13) then
                        hpos_init <= shift_left(
                            resize(sm_quotient, C_POS_BITS), C_POS_FRAC);
                        if v_origin_neg = '1' then
                            sm_state    <= SM_V_DIV;
                            sm_residue  <= v_origin_abs;
                            sm_quotient <= (others => '0');
                        else
                            vpos_init <= (others => '1');
                            sm_state  <= SM_IDLE;
                        end if;
                    else
                        sm_residue  <= sm_residue
                                       - to_unsigned(s_size_scale, 13);
                        sm_quotient <= sm_quotient + 1;
                    end if;

                when SM_V_DIV =>
                    if sm_residue < to_unsigned(s_size_scale, 13) then
                        vpos_init <= shift_left(
                            resize(sm_quotient, C_POS_BITS), C_POS_FRAC);
                        sm_state <= SM_IDLE;
                    else
                        sm_residue  <= sm_residue
                                       - to_unsigned(s_size_scale, 13);
                        sm_quotient <= sm_quotient + 1;
                    end if;
            end case;
        end if;
    end process p_init_compute;

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
    -- Cursor color: cycles through 8 high-saturation hues every ~135 ms,
    -- driven by frame_counter(5 downto 3).
    --==========================================================================
    cursor_color_sel <= frame_counter(5 downto 3);

    with cursor_color_sel select
        cursor_color_y <= to_unsigned(290, 10) when "000",  -- Red
                          to_unsigned(550, 10) when "001",  -- Orange
                          to_unsigned(870, 10) when "010",  -- Yellow
                          to_unsigned(620, 10) when "011",  -- Green
                          to_unsigned(820, 10) when "100",  -- Cyan
                          to_unsigned(170, 10) when "101",  -- Blue
                          to_unsigned(440, 10) when "110",  -- Magenta
                          to_unsigned(940, 10) when "111",  -- White
                          to_unsigned(940, 10) when others;
    with cursor_color_sel select
        cursor_color_u <= to_unsigned(425, 10) when "000",
                          to_unsigned(350, 10) when "001",
                          to_unsigned(130, 10) when "010",
                          to_unsigned(195, 10) when "011",
                          to_unsigned(600, 10) when "100",
                          to_unsigned(880, 10) when "101",
                          to_unsigned(850, 10) when "110",
                          to_unsigned(512, 10) when "111",
                          to_unsigned(512, 10) when others;
    with cursor_color_sel select
        cursor_color_v <= to_unsigned(900, 10) when "000",
                          to_unsigned(750, 10) when "001",
                          to_unsigned(560, 10) when "010",
                          to_unsigned(143, 10) when "011",
                          to_unsigned(128, 10) when "100",
                          to_unsigned(580, 10) when "101",
                          to_unsigned(900, 10) when "110",
                          to_unsigned(512, 10) when "111",
                          to_unsigned(512, 10) when others;

    --==========================================================================
    -- S1: Position counters + Bresenham accumulators.
    --
    -- h_count / v_count behave as before. The accumulators are the new bit:
    -- they sit at the all-1s "past text" sentinel outside the text band, jump
    -- to 0 when the active band begins (h_count crossing h_origin, or v_count
    -- reaching v_origin at hsync), and otherwise advance by the per-scale
    -- step value each pixel/line. Bit C_POS_BITS-1 of the accumulator is the
    -- past-text flag (set as soon as the integer part reaches 128 / 16).
    --
    -- The top-margin pulse covers exactly the scanline immediately above the
    -- first text row, and feeds the cursor-box logic so the top edge of the
    -- box on line 0 can draw above the glyphs. With the reworked vpos_acc
    -- semantics the margin is encoded as integer-part 0 of the accumulator,
    -- so it naturally lasts `scale` physical lines and the cursor box ends
    -- up symmetric at every size.
    --
    -- frame_counter ticks on every vsync falling edge; its top bits drive
    -- the cycling cursor color palette.
    --==========================================================================
    p_pipeline_s1 : process(clk)
        variable v_hs_falling : std_logic;
        variable v_vs_falling : std_logic;
        variable v_h_next     : unsigned(11 downto 0);
        variable v_new_line   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_hs_falling := prev_hsync_n and (not data_in.hsync_n);
            v_vs_falling := prev_vsync_n and (not data_in.vsync_n);

            -- h_count update + lookahead for accumulator reset.
            if v_hs_falling = '1' then
                h_count   <= (others => '0');
                v_h_next  := (others => '0');
            else
                h_count   <= h_count + 1;
                v_h_next  := h_count + 1;
            end if;

            -- v_count update + lookahead for vertical accumulator.
            if v_vs_falling = '1' then
                v_count    <= (others => '0');
                v_new_line := (others => '0');
            elsif v_hs_falling = '1' then
                v_count    <= v_count + 1;
                v_new_line := v_count + 1;
            else
                v_new_line := v_count;
            end if;

            -- Frame counter: ticks once per vsync.
            if v_vs_falling = '1' then
                frame_counter <= frame_counter + 1;
            end if;

            -- Horizontal accumulator: load hpos_init at hsync (sentinel for
            -- positive origins, pre-computed in-text value for negative
            -- origins where the text has scrolled off-screen left). For
            -- positive origins the reset to 0 still fires when h_count
            -- reaches s_h_origin; for negative origins s_h_origin is forced
            -- to a value h_count cannot equal, so the accumulator just
            -- walks from hpos_init onward.
            if v_hs_falling = '1' then
                hpos_acc <= hpos_init;
            elsif v_h_next = s_h_origin then
                hpos_acc <= (others => '0');
            elsif hpos_acc(C_POS_BITS - 1) = '0' then
                hpos_acc <= hpos_acc + resize(s_size_step, C_POS_BITS);
            end if;

            -- Vertical accumulator: same idea, but reloaded at vsync and
            -- ticked per line. Resets at v_origin (start of margin) and
            -- walks until the integer part exits the text region.
            if v_vs_falling = '1' then
                vpos_acc <= vpos_init;
            elsif v_hs_falling = '1' then
                if v_new_line = s_v_origin then
                    vpos_acc <= (others => '0');
                elsif vpos_acc(C_POS_BITS - 1) = '0' then
                    vpos_acc <= vpos_acc + resize(s_size_step, C_POS_BITS);
                end if;
            end if;
        end if;
    end process p_pipeline_s1;

    --==========================================================================
    -- Cursor tracking + text buffer write (Edit Mode)
    --
    -- The active cursor (r_cursor) follows whichever input source last moved:
    --   * knob 5 moves the cursor non-destructively (just position).
    --   * knob 4 moves the cursor AND blanks the cell it lands on, so the
    --     user can sweep knob 4 across a span of cells to erase them.
    -- The letter knob (knob 6) writes its current code into whichever cell
    -- the cursor is on. All three actions only take effect in Edit Mode; the
    -- prev_* edge-detect registers update unconditionally so toggling into
    -- Edit Mode never fires a spurious write.
    --==========================================================================
    p_cursor_edit : process(clk)
        variable v_new_cursor : unsigned(C_CURSOR_BIT - 1 downto 0);
        variable v_do_erase   : std_logic;
        variable v_flat_idx   : integer range 0 to C_TEXT_LEN - 1;
    begin
        if rising_edge(clk) then
            prev_letter    <= s_letter;
            prev_cursor_5p <= s_cursor_5p;
            prev_erase_5p  <= s_erase_5p;

            -- Cursor arbitration: knob 5 takes priority if both move on the
            -- same cycle, but knob 4 wins by default since the user actively
            -- intends to erase when they touch it.
            v_new_cursor := r_cursor;
            v_do_erase   := '0';
            if s_cursor_5p /= prev_cursor_5p then
                v_new_cursor := s_cursor_5p;
            elsif s_erase_5p /= prev_erase_5p then
                v_new_cursor := s_erase_5p;
                v_do_erase   := '1';
            end if;
            r_cursor <= v_new_cursor;

            v_flat_idx :=
                to_integer(v_new_cursor(C_CURSOR_BIT - 1 downto C_COL_BIT))
                * C_TEXT_COLS
                + to_integer(v_new_cursor(C_COL_BIT - 1 downto 0));

            if s_edit_mode = '1' then
                if s_letter /= prev_letter then
                    text_buf(v_flat_idx) <= s_letter;
                elsif v_do_erase = '1' then
                    text_buf(v_flat_idx) <= to_unsigned(0, 6);
                end if;
            end if;
        end if;
    end process p_cursor_edit;

    --==========================================================================
    -- S2: Extract glyph indices from accumulator integer parts.
    --
    -- Horizontal integer part [22:16] addresses the 128 glyph-pixels across
    -- the text band: bits[22:19] = char_idx (0..15), bits[18:16] = glyph_x.
    -- bit[23] is the past-text sentinel.
    -- Vertical integer part [23:16] holds 0 (margin), 1..16 (text rows), or
    -- >= 17 (past text). text_row_idx = int - 1 decodes into line_idx (bit 3)
    -- and glyph_y (bits 2..0).
    -- The top-margin region is included in in_text (so the cursor box can be
    -- drawn there); the in_top_margin flag steers downstream stages.
    --==========================================================================
    p_stage2 : process(clk)
        variable v_h_in         : std_logic;
        variable v_v_int        : unsigned(C_POS_INT - 1 downto 0);  -- 8-bit
        variable v_text_row     : unsigned(4 downto 0);
        variable v_v_in_real    : std_logic;
        variable v_in_margin    : std_logic;
    begin
        if rising_edge(clk) then
            s2_char_idx <= hpos_acc(C_POS_FRAC + 6 downto C_POS_FRAC + 3);
            s2_glyph_x  <= hpos_acc(C_POS_FRAC + 2 downto C_POS_FRAC);

            -- Vertical: decode the integer part. text_row covers 0..15 when
            -- the cursor is inside text; underflows to 31 in the margin row
            -- (int = 0), which our in_text_real check rejects via bit 4.
            v_v_int    := vpos_acc(C_POS_BITS - 1 downto C_POS_FRAC);
            v_text_row := resize(v_v_int, 5) - 1;
            s2_line_idx <= v_text_row(3 downto 3);
            s2_glyph_y  <= v_text_row(2 downto 0);

            -- h band: bit 23 (integer MSB) clear means we're below 128.
            v_h_in := not hpos_acc(C_POS_BITS - 1);

            -- Margin row: int == 0 (which uniquely happens after vpos_acc is
            -- reset and before the first step add brings int to 1).
            if v_v_int = to_unsigned(0, C_POS_INT) then
                v_in_margin := '1';
            else
                v_in_margin := '0';
            end if;

            -- Text rows: int in [1, 16]. Equivalent to text_row(4) = 0 AND
            -- the top 3 int bits being zero (which rules out wrap-around).
            if v_text_row(4) = '0' and
               vpos_acc(C_POS_BITS - 1 downto C_POS_FRAC + 5) =
               to_unsigned(0, C_POS_INT - 5) then
                v_v_in_real := '1';
            else
                v_v_in_real := '0';
            end if;

            s2_in_text       <= v_h_in and (v_v_in_real or v_in_margin);
            s2_in_top_margin <= v_h_in and v_in_margin;
        end if;
    end process p_stage2;

    --==========================================================================
    -- S3: Text buffer read + cursor box detect.
    --
    -- A single editing cursor (knob 5) walks all 32 cells. When edit mode is
    -- on, a 1-glyph-pixel-thick box is drawn around the perimeter of the
    -- active cell. All four edges fall on cells/rows the font guarantees to
    -- be blank, so the box never overlaps any letter pixel:
    --   left  edge = glyph_x "000"  (column 0 of the cell, blank gap)
    --   right edge = glyph_x "111"  (column 7 of the cell, blank gap)
    --   bottom edge= glyph_y "111"  (row 7 of the cell, inter-line gap)
    --   top edge   = row 7 of the line above (if cursor on line 1)
    --                or the dedicated top margin row above line 0 (which now
    --                lasts `scale` physical lines, matching the side/bottom
    --                edge thickness at every size).
    -- The box pixels are emitted with the cycling cursor color so they stay
    -- visible against any background or text color.
    --
    -- Edit-mode also draws a flashing underscore guide along the cell
    -- baseline (row 7 of every text cell) so the user can map out where each
    -- letter slot is, even when the cell is empty. The flash uses bit 4 of
    -- the frame counter (~2 Hz at 60 Hz fields), and underscore pixels use
    -- the same cycling cursor color as the cursor box.
    --==========================================================================
    p_stage3 : process(clk)
        variable v_box        : std_logic;
        variable v_underscore : std_logic;
        variable v_flat_idx   : integer range 0 to C_TEXT_LEN - 1;
    begin
        if rising_edge(clk) then
            v_flat_idx := to_integer(s2_line_idx) * C_TEXT_COLS
                          + to_integer(s2_char_idx);
            s3_char_code     <= text_buf(v_flat_idx);
            s3_glyph_x       <= s2_glyph_x;
            s3_glyph_y       <= s2_glyph_y;
            s3_in_text       <= s2_in_text;
            s3_in_top_margin <= s2_in_top_margin;

            v_box := '0';
            if s_edit_mode = '1' and s2_char_idx = s_cursor_col then
                if s2_in_top_margin = '1' then
                    -- Top margin row only draws the top edge for a cursor
                    -- positioned on line 0.
                    if s_cursor_line(0) = '0' then
                        v_box := '1';
                    end if;
                elsif s2_line_idx = s_cursor_line then
                    -- Inside the cursor's own cell: left/right/bottom edges.
                    if s2_glyph_x = "000" or s2_glyph_x = "111"
                       or s2_glyph_y = "111" then
                        v_box := '1';
                    end if;
                elsif s_cursor_line(0) = '1' and s2_line_idx(0) = '0'
                      and s2_glyph_y = "111" then
                    -- Cursor on line 1: its top edge lives in row 7 of line 0
                    -- (the inter-line gap row, guaranteed blank).
                    v_box := '1';
                end if;
            end if;

            -- Underscore guide: glyph_y = 7 of any text row, in edit mode,
            -- gated by the slow frame-counter bit so it flashes. Restricted
            -- to glyph_x in [1, 5] so it only sits under the letter slot,
            -- leaving the inter-character gap (cols 0, 6, 7) dark for a
            -- visible break between adjacent cells.
            if s_edit_mode = '1' and s2_in_text = '1'
               and s2_in_top_margin = '0' and s2_glyph_y = "111"
               and s2_glyph_x /= "000"
               and s2_glyph_x(2 downto 1) /= "11"
               and frame_counter(4) = '0' then
                v_underscore := '1';
            else
                v_underscore := '0';
            end if;
            s3_underscore <= v_underscore;
            s3_cursor_box <= v_box;
        end if;
    end process p_stage3;

    --==========================================================================
    -- S4: Font ROM lookup, registered into s4_font_row.
    -- The full 8-bit row is captured here; the per-pixel column select and
    -- the in_top_margin mask both happen in S5 from the registered row.
    --
    -- The ROM holds two fonts back-to-back (default at 0..63, sci-fi at
    -- 64..127). s_font_sel becomes the top bit of the 7-bit char address,
    -- so font switching is essentially free in logic.
    --==========================================================================
    p_stage4 : process(clk)
        variable v_font_addr : integer range 0 to C_FONT_CHAR_COUNT - 1;
    begin
        if rising_edge(clk) then
            if s_font_sel = '1' then
                v_font_addr := 64 + to_integer(s3_char_code);
            else
                v_font_addr := to_integer(s3_char_code);
            end if;
            s4_font_row     <= C_FONT_ROM(v_font_addr,
                                          to_integer(s3_glyph_y));
            s4_glyph_x      <= s3_glyph_x;
            s4_in_text      <= s3_in_text;
            s4_in_top_margin<= s3_in_top_margin;
            s4_cursor_box   <= s3_cursor_box;
            s4_underscore   <= s3_underscore;
        end if;
    end process p_stage4;

    --==========================================================================
    -- S5: Pixel extract from registered font row + output mux.
    --==========================================================================
    p_stage5 : process(clk)
        variable v_pixel_on  : std_logic;
        variable v_show_text : std_logic;
        variable v_bg_y      : std_logic_vector(9 downto 0);
        variable v_bg_u      : std_logic_vector(9 downto 0);
        variable v_bg_v      : std_logic_vector(9 downto 0);
    begin
        if rising_edge(clk) then
            -- Mask the font lookup in the top margin row: the glyph indices
            -- are sentinel-garbage there, so we must not treat any font bit
            -- as a real pixel.
            v_pixel_on := s4_font_row(7 - to_integer(s4_glyph_x))
                          and not s4_in_top_margin;

            v_show_text := s4_in_text and
                           (v_pixel_on or s4_cursor_box or s4_underscore);

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
                if s4_cursor_box = '1' then
                    -- Cursor box uses the cycling cursor color so it stays
                    -- distinct against any background or text color.
                    data_out.y <= std_logic_vector(cursor_color_y);
                    data_out.u <= std_logic_vector(cursor_color_u);
                    data_out.v <= std_logic_vector(cursor_color_v);
                elsif s4_underscore = '1' then
                    -- Cell-baseline underscores: always solid white so they
                    -- read as a static "where letters can go" guide rather
                    -- than competing with the cycling cursor.
                    data_out.y <= std_logic_vector(to_unsigned(940, 10));
                    data_out.u <= std_logic_vector(to_unsigned(512, 10));
                    data_out.v <= std_logic_vector(to_unsigned(512, 10));
                elsif s_matte = '1' then
                    -- Matte mode: glyph pixels reveal the incoming video.
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
