-- C64: Commodore 64 display processor.
--
-- Quantizes incoming video to the fixed C64 16-colour palette using
-- Manhattan distance matching in YUV space, then applies optional
-- pixelation (chunky character cells), coloured border, and CRT-style
-- scanline dimming.  Zero BRAM — all processing is combinational + LUT.
--
-- Pipeline (10 clocks, data_in to data_out):
--   T1  : input register + timing detection + cell tracking
--   T2  : compute all 16 Manhattan distances in parallel
--   T3  : 16→4 group reduction (four 4-way mins)
--   T4  : 4→1 final reduction → winner index + palette lookup
--   T5  : pixelation sample/hold + border overlay
--   T6  : scanline dimming + output register
--   T7-T10 : interpolator_u wet/dry mix (4 clocks)
--
-- Register map:
--   registers_in(0) = Pixel Size (5 zones: Off/2×/4×/8×/16×)
--   registers_in(1) = Border Wid (0-1023 → 0-255 pixels)
--   registers_in(2) = Border Col (16 zones → C64 palette index)
--   registers_in(3) = Scanline   (0-1023, dimming strength)
--   registers_in(4) = Saturation (0-1023, 512=unity)
--   registers_in(5) = Contrast   (0-1023, 512=unity, pre-match Y expand)
--   registers_in(6) = Switches
--                       bit 0 = Pixelate  (0=Off, 1=On)
--                       bit 1 = Border    (0=Off, 1=On)
--                       bit 2 = Scanlines (0=Off, 1=On)
--                       bit 3 = (reserved)
--                       bit 4 = Bypass
--   registers_in(7) = Mix (0=dry, 1023=full wet)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture c64 of program_top is

    constant C_W          : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_LATENCY    : integer := 10;
    constant C_DIST_WIDTH : integer := 8;  -- Manhattan distance: max 63×3=189

    -- ====================================================================
    -- Commodore 64 16-colour palette (10-bit YUV, U/V centred at 512)
    -- Derived from the Colodore palette (pepto/colodore project).
    -- ====================================================================
    type t_pal_entry is record
        y : unsigned(9 downto 0);
        u : unsigned(9 downto 0);
        v : unsigned(9 downto 0);
    end record;
    type t_palette is array(0 to 15) of t_pal_entry;

    constant C_PAL : t_palette := (
        -- 0  Black
        (y => to_unsigned(   0, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
        -- 1  White
        (y => to_unsigned(1023, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
        -- 2  Red
        (y => to_unsigned( 273, 10), u => to_unsigned(455, 10), v => to_unsigned(660, 10)),
        -- 3  Cyan
        (y => to_unsigned( 600, 10), u => to_unsigned(580, 10), v => to_unsigned(390, 10)),
        -- 4  Purple
        (y => to_unsigned( 337, 10), u => to_unsigned(630, 10), v => to_unsigned(600, 10)),
        -- 5  Green
        (y => to_unsigned( 467, 10), u => to_unsigned(390, 10), v => to_unsigned(420, 10)),
        -- 6  Blue
        (y => to_unsigned( 212, 10), u => to_unsigned(670, 10), v => to_unsigned(512, 10)),
        -- 7  Yellow
        (y => to_unsigned( 730, 10), u => to_unsigned(400, 10), v => to_unsigned(570, 10)),
        -- 8  Orange
        (y => to_unsigned( 350, 10), u => to_unsigned(430, 10), v => to_unsigned(640, 10)),
        -- 9  Brown
        (y => to_unsigned( 220, 10), u => to_unsigned(460, 10), v => to_unsigned(590, 10)),
        -- 10 Light Red
        (y => to_unsigned( 430, 10), u => to_unsigned(470, 10), v => to_unsigned(610, 10)),
        -- 11 Dark Grey
        (y => to_unsigned( 270, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
        -- 12 Medium Grey
        (y => to_unsigned( 430, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
        -- 13 Light Green
        (y => to_unsigned( 700, 10), u => to_unsigned(420, 10), v => to_unsigned(420, 10)),
        -- 14 Light Blue
        (y => to_unsigned( 400, 10), u => to_unsigned(600, 10), v => to_unsigned(530, 10)),
        -- 15 Light Grey
        (y => to_unsigned( 596, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    );

    -- ====================================================================
    -- Manhattan distance helper (5-bit truncated for coarser matching, reduced chroma noise sensitivity)
    -- ====================================================================
    function f_dist(
        py, pu, pv : unsigned(9 downto 0);
        cy, cu, cv : unsigned(9 downto 0)
    ) return unsigned is
        variable dy, du, dv : unsigned(4 downto 0);
    begin
        if py(9 downto 5) > cy(9 downto 5) then
            dy := py(9 downto 5) - cy(9 downto 5);
        else
            dy := cy(9 downto 5) - py(9 downto 5);
        end if;
        if pu(9 downto 5) > cu(9 downto 5) then
            du := pu(9 downto 5) - cu(9 downto 5);
        else
            du := cu(9 downto 5) - pu(9 downto 5);
        end if;
        if pv(9 downto 5) > cv(9 downto 5) then
            dv := pv(9 downto 5) - cv(9 downto 5);
        else
            dv := cv(9 downto 5) - pv(9 downto 5);
        end if;
        return resize(dy, C_DIST_WIDTH) + resize(du, C_DIST_WIDTH)
             + resize(dv, C_DIST_WIDTH);
    end function;

    -- ====================================================================
    -- Signals
    -- ====================================================================

    -- T1: registered input + timing
    signal s_t1_y, s_t1_u, s_t1_v : unsigned(9 downto 0) := (others => '0');
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_x_count      : unsigned(11 downto 0) := (others => '0');
    signal s_y_count      : unsigned(11 downto 0) := (others => '0');
    signal s_line_width   : unsigned(11 downto 0) := to_unsigned(1280, 12);
    signal s_frame_height : unsigned(11 downto 0) := to_unsigned(720, 12);

    -- Cell tracking for pixelation
    signal s_cell_x       : unsigned(4 downto 0) := (others => '0');
    signal s_cell_y       : unsigned(4 downto 0) := (others => '0');
    signal s_cell_origin  : std_logic := '0';  -- '1' at top-left of each cell

    -- T2: distances
    type t_dist_arr is array(0 to 15) of unsigned(C_DIST_WIDTH - 1 downto 0);
    signal s_dist : t_dist_arr;
    signal s_t2_y, s_t2_u, s_t2_v : unsigned(9 downto 0) := (others => '0');

    -- T3: group winners (4 groups of 4)
    signal s_grp_min_a, s_grp_min_b : unsigned(C_DIST_WIDTH - 1 downto 0);
    signal s_grp_min_c, s_grp_min_d : unsigned(C_DIST_WIDTH - 1 downto 0);
    signal s_grp_idx_a, s_grp_idx_b : unsigned(3 downto 0);
    signal s_grp_idx_c, s_grp_idx_d : unsigned(3 downto 0);

    -- T4: final winner + palette lookup
    signal s_match_idx    : unsigned(3 downto 0) := (others => '0');
    signal s_pal_y        : unsigned(9 downto 0) := (others => '0');
    signal s_pal_u        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_pal_v        : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- T5: pixelation hold + border
    signal s_out_y        : unsigned(9 downto 0) := (others => '0');
    signal s_out_u        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_out_v        : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- T6: scanlines + final proc
    signal s_proc_y       : unsigned(9 downto 0) := (others => '0');
    signal s_proc_u       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_proc_v       : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Pixelation held colour (persists across the cell)
    signal s_hold_y       : unsigned(9 downto 0) := (others => '0');
    signal s_hold_u       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_hold_v       : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Pipeline delay for border/scanline flags
    signal s_border_d1    : std_logic := '0';
    signal s_border_d2    : std_logic := '0';
    signal s_border_d3    : std_logic := '0';
    signal s_border_d4    : std_logic := '0';
    signal s_scanline_d1  : std_logic := '0';
    signal s_scanline_d2  : std_logic := '0';
    signal s_scanline_d3  : std_logic := '0';
    signal s_scanline_d4  : std_logic := '0';

    -- Border colour (latched from palette)
    signal s_bord_y       : unsigned(9 downto 0) := (others => '0');
    signal s_bord_u       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bord_v       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bord_y_d1, s_bord_u_d1, s_bord_v_d1 : unsigned(9 downto 0);
    signal s_bord_y_d2, s_bord_u_d2, s_bord_v_d2 : unsigned(9 downto 0);
    signal s_bord_y_d3, s_bord_u_d3, s_bord_v_d3 : unsigned(9 downto 0);
    signal s_bord_y_d4, s_bord_u_d4, s_bord_v_d4 : unsigned(9 downto 0);

    -- Cell-origin pipeline (delay through palette matching stages)
    signal s_cell_origin_d1 : std_logic := '0';
    signal s_cell_origin_d2 : std_logic := '0';
    signal s_cell_origin_d3 : std_logic := '0';

    -- Cell Y coordinate pipeline (for vertical hold detection)
    signal s_cell_y_d1 : unsigned(4 downto 0) := (others => '0');
    signal s_cell_y_d2 : unsigned(4 downto 0) := (others => '0');
    signal s_cell_y_d3 : unsigned(4 downto 0) := (others => '0');

    -- Bypass / sync shift registers
    type t_data_sr is array(0 to C_LATENCY - 1) of std_logic_vector(9 downto 0);
    type t_bit_sr  is array(0 to C_LATENCY - 1) of std_logic;
    signal s_y_sr     : t_data_sr := (others => (others => '0'));
    signal s_u_sr     : t_data_sr := (others => (others => '0'));
    signal s_v_sr     : t_data_sr := (others => (others => '0'));
    signal s_hsync_sr : t_bit_sr  := (others => '1');
    signal s_vsync_sr : t_bit_sr  := (others => '1');
    signal s_field_sr : t_bit_sr  := (others => '1');
    signal s_avid_sr  : t_bit_sr  := (others => '0');

    -- Interpolator
    signal s_mix_t    : unsigned(9 downto 0);
    signal s_interp_y_result : unsigned(9 downto 0);
    signal s_interp_u_result : unsigned(9 downto 0);
    signal s_interp_v_result : unsigned(9 downto 0);
    signal s_bypass   : std_logic;

begin

    s_bypass <= registers_in(6)(4);
    s_mix_t  <= unsigned(registers_in(7));

    -- ====================================================================
    -- T1: register inputs, timing, cell tracking, bypass SR
    -- ====================================================================
    p_t1 : process(clk)
        variable v_cell_size : unsigned(4 downto 0);
        variable v_border_idx : unsigned(3 downto 0);
        variable v_border_wid : unsigned(9 downto 0);
        variable v_in_border  : std_logic;
    begin
        if rising_edge(clk) then
            s_t1_y <= unsigned(data_in.y);
            s_t1_u <= unsigned(data_in.u);
            s_t1_v <= unsigned(data_in.v);

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            -- Pixel counter
            if data_in.avid = '1' then
                s_x_count <= s_x_count + 1;
            end if;

            -- hsync: latch width, reset x, advance y
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_x_count > 0 then
                    s_line_width <= s_x_count;
                end if;
                s_x_count <= (others => '0');
                s_y_count <= s_y_count + 1;
                s_cell_x  <= (others => '0');
                s_cell_y  <= s_cell_y + 1;
            end if;

            -- vsync: latch height, reset y
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_y_count > 0 then
                    s_frame_height <= s_y_count;
                end if;
                s_y_count <= (others => '0');
                s_x_count <= (others => '0');
                s_cell_x  <= (others => '0');
                s_cell_y  <= (others => '0');
            end if;

            -- Cell tracking for pixelation (power-of-2 cell sizes)
            -- Decode cell size from knob 1
            v_cell_size := unsigned(registers_in(0)(9 downto 5));  -- top 5 bits
            if v_cell_size < 6 then
                v_cell_size := to_unsigned(1, 5);   -- Off (1×1 = no pixelation)
            elsif v_cell_size < 13 then
                v_cell_size := to_unsigned(2, 5);
            elsif v_cell_size < 19 then
                v_cell_size := to_unsigned(4, 5);
            elsif v_cell_size < 26 then
                v_cell_size := to_unsigned(8, 5);
            else
                v_cell_size := to_unsigned(16, 5);
            end if;

            -- Advance cell-local counter during active video.
            -- cell_origin fires at the center of each cell (stable interior point).
            -- This samples from the most stable region, avoiding edge transitions.
            if data_in.avid = '1' then
                if s_cell_x >= v_cell_size - 1 then
                    s_cell_x      <= (others => '0');
                    s_cell_origin <= '0';
                elsif s_cell_x = v_cell_size / 2 then
                    s_cell_x      <= s_cell_x + 1;
                    s_cell_origin <= '1';  -- fire at center of cell
                else
                    s_cell_x      <= s_cell_x + 1;
                    s_cell_origin <= '0';
                end if;
            else
                s_cell_origin <= '0';
            end if;

            -- Cell y increments at each hsync, resets every cell_size lines
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_cell_y >= v_cell_size - 1 then
                    s_cell_y <= (others => '0');
                else
                    s_cell_y <= s_cell_y + 1;
                end if;
            end if;

            -- Border detection: pixel is in border if within border_width
            -- of any edge.  Compare against measured resolution.
            v_border_wid := unsigned(registers_in(1)(9 downto 0));
            -- Scale: raw >> 2 → 0-255 pixels of border
            v_border_wid := "00" & v_border_wid(9 downto 2);
            if registers_in(6)(1) = '1' and (
                s_x_count < resize(v_border_wid, 12) or
                s_x_count >= s_line_width - resize(v_border_wid, 12) or
                s_y_count < resize(v_border_wid, 12) or
                s_y_count >= s_frame_height - resize(v_border_wid, 12)
            ) then
                v_in_border := '1';
            else
                v_in_border := '0';
            end if;
            s_border_d1 <= v_in_border;

            -- Scanline flag: odd lines get dimmed
            if registers_in(6)(2) = '1' then
                s_scanline_d1 <= s_y_count(0);
            else
                s_scanline_d1 <= '0';
            end if;

            -- Border colour: decode knob 3 into palette index (16 zones)
            v_border_idx := unsigned(registers_in(2)(9 downto 6));
            s_bord_y <= C_PAL(to_integer(v_border_idx)).y;
            s_bord_u <= C_PAL(to_integer(v_border_idx)).u;
            s_bord_v <= C_PAL(to_integer(v_border_idx)).v;

            -- Cell origin + cell Y pipeline start
            s_cell_origin_d1 <= s_cell_origin;
            s_cell_y_d1      <= s_cell_y;

            -- Bypass and sync shift registers
            s_y_sr(0)     <= data_in.y;
            s_u_sr(0)     <= data_in.u;
            s_v_sr(0)     <= data_in.v;
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LATENCY - 1 loop
                s_y_sr(i)     <= s_y_sr(i - 1);
                s_u_sr(i)     <= s_u_sr(i - 1);
                s_v_sr(i)     <= s_v_sr(i - 1);
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_t1;

    -- ====================================================================
    -- T2: pre-match contrast + posterisation, then 16 palette distances
    --
    -- Contrast (knob 6 "Brightness"): expands Y around midpoint before
    -- matching.  512 = unity.  Higher values push darks darker and lights
    -- lighter, making palette decisions more decisive and reducing flicker.
    --
    -- Posterisation: mask the bottom 3 bits of Y/U/V to remove noise that
    -- causes rapid colour flipping at palette boundaries.
    -- ====================================================================
    p_t2 : process(clk)
        constant C_POSTER_MASK : unsigned(9 downto 0) := "1111111000";  -- mask low 3 bits
        variable v_contrast    : unsigned(9 downto 0);
        variable v_cy_s        : signed(11 downto 0);
        variable v_cy_prod     : signed(22 downto 0);
        variable v_cy_out      : signed(11 downto 0);
        variable v_match_y     : unsigned(9 downto 0);
        variable v_match_u     : unsigned(9 downto 0);
        variable v_match_v     : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_t2_y <= s_t1_y;
            s_t2_u <= s_t1_u;
            s_t2_v <= s_t1_v;

            -- Pre-match contrast: Y_out = clamp((Y - 512) * contrast/512 + 512)
            v_contrast := unsigned(registers_in(5));
            v_cy_s     := signed(resize(s_t1_y, 12)) - to_signed(512, 12);
            v_cy_prod  := v_cy_s * signed('0' & v_contrast);
            v_cy_out   := v_cy_prod(20 downto 9) + to_signed(512, 12);

            if v_cy_out < 0 then
                v_match_y := (others => '0');
            elsif v_cy_out > 1023 then
                v_match_y := to_unsigned(1023, 10);
            else
                v_match_y := unsigned(v_cy_out(9 downto 0));
            end if;

            -- Posterise: zero bottom 2 bits to suppress noise
            v_match_y := v_match_y and C_POSTER_MASK;
            v_match_u := s_t1_u    and C_POSTER_MASK;
            v_match_v := s_t1_v    and C_POSTER_MASK;

            for i in 0 to 15 loop
                s_dist(i) <= f_dist(v_match_y, v_match_u, v_match_v,
                                    C_PAL(i).y, C_PAL(i).u, C_PAL(i).v);
            end loop;

            -- Pipeline border/scanline/cell-origin/cell-y
            s_border_d2      <= s_border_d1;
            s_scanline_d2    <= s_scanline_d1;
            s_cell_origin_d2 <= s_cell_origin_d1;
            s_cell_y_d2      <= s_cell_y_d1;
            s_bord_y_d1 <= s_bord_y;  s_bord_u_d1 <= s_bord_u;  s_bord_v_d1 <= s_bord_v;
        end if;
    end process p_t2;

    -- ====================================================================
    -- T3: 16→4 group reduction
    -- ====================================================================
    p_t3 : process(clk)
        variable v_min  : unsigned(C_DIST_WIDTH - 1 downto 0);
        variable v_idx  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- Group A (colours 0-3)
            v_min := s_dist(0); v_idx := x"0";
            if s_dist(1) < v_min then v_min := s_dist(1); v_idx := x"1"; end if;
            if s_dist(2) < v_min then v_min := s_dist(2); v_idx := x"2"; end if;
            if s_dist(3) < v_min then v_min := s_dist(3); v_idx := x"3"; end if;
            s_grp_min_a <= v_min; s_grp_idx_a <= v_idx;

            -- Group B (colours 4-7)
            v_min := s_dist(4); v_idx := x"4";
            if s_dist(5) < v_min then v_min := s_dist(5); v_idx := x"5"; end if;
            if s_dist(6) < v_min then v_min := s_dist(6); v_idx := x"6"; end if;
            if s_dist(7) < v_min then v_min := s_dist(7); v_idx := x"7"; end if;
            s_grp_min_b <= v_min; s_grp_idx_b <= v_idx;

            -- Group C (colours 8-11)
            v_min := s_dist(8); v_idx := x"8";
            if s_dist(9) < v_min then v_min := s_dist(9); v_idx := x"9"; end if;
            if s_dist(10) < v_min then v_min := s_dist(10); v_idx := x"A"; end if;
            if s_dist(11) < v_min then v_min := s_dist(11); v_idx := x"B"; end if;
            s_grp_min_c <= v_min; s_grp_idx_c <= v_idx;

            -- Group D (colours 12-15)
            v_min := s_dist(12); v_idx := x"C";
            if s_dist(13) < v_min then v_min := s_dist(13); v_idx := x"D"; end if;
            if s_dist(14) < v_min then v_min := s_dist(14); v_idx := x"E"; end if;
            if s_dist(15) < v_min then v_min := s_dist(15); v_idx := x"F"; end if;
            s_grp_min_d <= v_min; s_grp_idx_d <= v_idx;

            -- Pipeline
            s_border_d3      <= s_border_d2;
            s_scanline_d3    <= s_scanline_d2;
            s_cell_origin_d3 <= s_cell_origin_d2;
            s_cell_y_d3      <= s_cell_y_d2;
            s_bord_y_d2 <= s_bord_y_d1;  s_bord_u_d2 <= s_bord_u_d1;  s_bord_v_d2 <= s_bord_v_d1;
        end if;
    end process p_t3;

    -- ====================================================================
    -- T4: 4→1 final reduction + palette colour lookup
    -- ====================================================================
    p_t4 : process(clk)
        variable v_min : unsigned(C_DIST_WIDTH - 1 downto 0);
        variable v_idx : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            v_min := s_grp_min_a; v_idx := s_grp_idx_a;
            if s_grp_min_b < v_min then v_min := s_grp_min_b; v_idx := s_grp_idx_b; end if;
            if s_grp_min_c < v_min then v_min := s_grp_min_c; v_idx := s_grp_idx_c; end if;
            if s_grp_min_d < v_min then v_min := s_grp_min_d; v_idx := s_grp_idx_d; end if;

            s_match_idx <= v_idx;
            s_pal_y <= C_PAL(to_integer(v_idx)).y;
            s_pal_u <= C_PAL(to_integer(v_idx)).u;
            s_pal_v <= C_PAL(to_integer(v_idx)).v;

            -- Pipeline
            s_border_d4   <= s_border_d3;
            s_scanline_d4 <= s_scanline_d3;
            s_bord_y_d3 <= s_bord_y_d2;  s_bord_u_d3 <= s_bord_u_d2;  s_bord_v_d3 <= s_bord_v_d2;
        end if;
    end process p_t4;

    -- ====================================================================
    -- T5: pixelation sample/hold + border overlay
    -- ====================================================================
    p_t5 : process(clk)
    begin
        if rising_edge(clk) then
            -- Pixelation: if enabled, hold palette colour at cell center
            if registers_in(6)(0) = '1' then
                -- Sample at cell center (most stable interior point), hold for rest of cell
                if s_cell_origin_d3 = '1' then
                    s_hold_y <= s_pal_y;
                    s_hold_u <= s_pal_u;
                    s_hold_v <= s_pal_v;
                end if;
                s_out_y <= s_hold_y;
                s_out_u <= s_hold_u;
                s_out_v <= s_hold_v;
            else
                -- No pixelation: pass palette colour through
                s_out_y <= s_pal_y;
                s_out_u <= s_pal_u;
                s_out_v <= s_pal_v;
            end if;

            -- Border overlay: replace with border colour if in border
            if s_border_d4 = '1' then
                s_out_y <= s_bord_y_d3;
                s_out_u <= s_bord_u_d3;
                s_out_v <= s_bord_v_d3;
            end if;

            s_bord_y_d4 <= s_bord_y_d3;
            s_bord_u_d4 <= s_bord_u_d3;
            s_bord_v_d4 <= s_bord_v_d3;
        end if;
    end process p_t5;

    -- ====================================================================
    -- T6: scanline dimming + saturation + output register
    --
    -- Brightness/contrast knob (6) is now used pre-match in T2.
    -- Saturation (knob 5): U/V deviation from 512 scaled by knob/512.
    --                       512 = unity, 0 = greyscale, 1023 ≈ 2× chroma.
    -- ====================================================================
    p_t6 : process(clk)
        variable v_scan_str    : unsigned(9 downto 0);
        variable v_saturation  : unsigned(9 downto 0);
        variable v_dim_y       : unsigned(19 downto 0);
        variable v_y_scan      : unsigned(9 downto 0);
        variable v_su           : signed(11 downto 0);
        variable v_sv           : signed(11 downto 0);
        variable v_su_prod     : signed(22 downto 0);
        variable v_sv_prod     : signed(22 downto 0);
        variable v_su_out      : signed(11 downto 0);
        variable v_sv_out      : signed(11 downto 0);
    begin
        if rising_edge(clk) then
          if s_border_d4 = '1' then
            -- Border pixels: pass through pure palette colour, no processing
            s_proc_y <= s_out_y;
            s_proc_u <= s_out_u;
            s_proc_v <= s_out_v;
          else
            v_scan_str  := unsigned(registers_in(3));
            v_saturation := unsigned(registers_in(4));

            -- Scanline dimming: Y = Y * (1023 - scan_str) >> 10
            if s_scanline_d4 = '1' then
                v_dim_y  := s_out_y * (to_unsigned(1023, 10) - v_scan_str);
                v_y_scan := v_dim_y(19 downto 10);
            else
                v_y_scan := s_out_y;
            end if;

            s_proc_y <= v_y_scan;

            -- Saturation: (ch - 512) * saturation >> 9 + 512  (512 = unity)
            v_su := signed(resize(s_out_u, 12)) - to_signed(512, 12);
            v_sv := signed(resize(s_out_v, 12)) - to_signed(512, 12);

            v_su_prod := v_su * signed('0' & v_saturation);
            v_sv_prod := v_sv * signed('0' & v_saturation);

            v_su_out := v_su_prod(20 downto 9) + to_signed(512, 12);
            v_sv_out := v_sv_prod(20 downto 9) + to_signed(512, 12);

            -- Clamp U to [0, 1023]
            if v_su_out < 0 then
                s_proc_u <= (others => '0');
            elsif v_su_out > 1023 then
                s_proc_u <= to_unsigned(1023, 10);
            else
                s_proc_u <= unsigned(v_su_out(9 downto 0));
            end if;

            -- Clamp V to [0, 1023]
            if v_sv_out < 0 then
                s_proc_v <= (others => '0');
            elsif v_sv_out > 1023 then
                s_proc_v <= to_unsigned(1023, 10);
            else
                s_proc_v <= unsigned(v_sv_out(9 downto 0));
            end if;
          end if;  -- border vs. video area
        end if;
    end process p_t6;

    -- ====================================================================
    -- Interpolator: 4-clock wet/dry crossfade (T7-T10)
    -- Dry tap at SR(5) = 6 clocks, aligning with s_proc at T6.
    -- ====================================================================
    interp_y : entity work.interpolator_u
        generic map(G_WIDTH => C_W, G_FRAC_BITS => C_W,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1',
                 a => unsigned(s_y_sr(5)), b => s_proc_y, t => s_mix_t,
                 result => s_interp_y_result, valid => open);

    interp_u : entity work.interpolator_u
        generic map(G_WIDTH => C_W, G_FRAC_BITS => C_W,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1',
                 a => unsigned(s_u_sr(5)), b => s_proc_u, t => s_mix_t,
                 result => s_interp_u_result, valid => open);

    interp_v : entity work.interpolator_u
        generic map(G_WIDTH => C_W, G_FRAC_BITS => C_W,
                    G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1',
                 a => unsigned(s_v_sr(5)), b => s_proc_v, t => s_mix_t,
                 result => s_interp_v_result, valid => open);

    -- ====================================================================
    -- Output at T10
    -- ====================================================================
    data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);

    data_out.y <= s_y_sr(C_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_interp_y_result);
    data_out.u <= s_u_sr(C_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_interp_u_result);
    data_out.v <= s_v_sr(C_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_interp_v_result);

end architecture c64;
