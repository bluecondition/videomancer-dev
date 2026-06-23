-- C64: Commodore 64 display processor.
--
-- Quantises incoming video to the fixed C64 16-colour palette (Manhattan
-- distance in YUV), with optional chunky pixelation, coloured border, and
-- CRT scanline dimming.
--
-- PIXELATION (rewritten 2026-06-22 to mirror the CGA program, which is
-- confirmed working on this hardware):
--   * On the FIRST line of each cell row, the cell is averaged horizontally,
--     palette-matched, and the 4-bit index is written to a per-column buffer.
--   * Every output pixel reads that buffer -> solid blocks for the whole row.
--   * Controls (pixelate / size / contrast / sat) are plain registered
--     signals used DIRECTLY -- NOT pipelined per-pixel. (A per-pixel pixelate
--     shift-register flag is what broke earlier versions.)
--   * Single buffer, shallow pipeline, full-width addressing -- exactly CGA.
--
-- Pipeline (10 clocks):
--   T1 : controls, cell tracking, h-average, match input, buffer-write setup
--   T2 : pre-match contrast + posterise, 16 palette distances
--   T3 : 16->4 reduction
--   T4 : 4->1 reduction -> index, register buffer write
--   T5 : display (buffer vs per-pixel) palette lookup + border overlay
--   T6 : scanline dim + saturation + output register
--   T7-T10 : interpolator wet/dry mix
--
-- Register map:
--   registers_in(0) = Pixel Size (5 zones: Off/2x/4x/8x/16x)
--   registers_in(1) = Border Wid (0-1023 -> 0-255 px)
--   registers_in(2) = Border Col (16 zones -> palette index)
--   registers_in(3) = Scanline   (0-1023)
--   registers_in(4) = Saturation (0-1023, 512 = unity)
--   registers_in(5) = Contrast   (0-1023, 512 = unity)
--   registers_in(6) = Switches: 0=Pixelate 1=Border 2=Scanlines 4=Bypass
--   registers_in(7) = Mix (0=dry, 1023=wet)

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
    constant C_LATENCY    : integer := 12;
    constant C_DIST_WIDTH : integer := 8;
    constant C_MAX_COLS   : integer := 2048;   -- covers full HD line width

    type t_pal_entry is record
        y : unsigned(9 downto 0);
        u : unsigned(9 downto 0);
        v : unsigned(9 downto 0);
    end record;
    type t_palette is array(0 to 15) of t_pal_entry;

    constant C_PAL : t_palette := (
        (y => to_unsigned(   0, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  -- 0  Black
        (y => to_unsigned(1023, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  -- 1  White
        (y => to_unsigned( 273, 10), u => to_unsigned(455, 10), v => to_unsigned(660, 10)),  -- 2  Red
        (y => to_unsigned( 600, 10), u => to_unsigned(580, 10), v => to_unsigned(390, 10)),  -- 3  Cyan
        (y => to_unsigned( 337, 10), u => to_unsigned(630, 10), v => to_unsigned(600, 10)),  -- 4  Purple
        (y => to_unsigned( 467, 10), u => to_unsigned(390, 10), v => to_unsigned(420, 10)),  -- 5  Green
        (y => to_unsigned( 212, 10), u => to_unsigned(670, 10), v => to_unsigned(512, 10)),  -- 6  Blue
        (y => to_unsigned( 730, 10), u => to_unsigned(400, 10), v => to_unsigned(570, 10)),  -- 7  Yellow
        (y => to_unsigned( 350, 10), u => to_unsigned(430, 10), v => to_unsigned(640, 10)),  -- 8  Orange
        (y => to_unsigned( 220, 10), u => to_unsigned(460, 10), v => to_unsigned(590, 10)),  -- 9  Brown
        (y => to_unsigned( 430, 10), u => to_unsigned(470, 10), v => to_unsigned(610, 10)),  -- 10 Light Red
        (y => to_unsigned( 270, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  -- 11 Dark Grey
        (y => to_unsigned( 430, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)),  -- 12 Medium Grey
        (y => to_unsigned( 700, 10), u => to_unsigned(420, 10), v => to_unsigned(420, 10)),  -- 13 Light Green
        (y => to_unsigned( 400, 10), u => to_unsigned(600, 10), v => to_unsigned(530, 10)),  -- 14 Light Blue
        (y => to_unsigned( 596, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))   -- 15 Light Grey
    );

    function f_dist(
        py, pu, pv : unsigned(9 downto 0);
        cy, cu, cv : unsigned(9 downto 0)
    ) return unsigned is
        variable dy, du, dv : unsigned(4 downto 0);
    begin
        if py(9 downto 5) > cy(9 downto 5) then dy := py(9 downto 5) - cy(9 downto 5);
        else                                    dy := cy(9 downto 5) - py(9 downto 5); end if;
        if pu(9 downto 5) > cu(9 downto 5) then du := pu(9 downto 5) - cu(9 downto 5);
        else                                    du := cu(9 downto 5) - pu(9 downto 5); end if;
        if pv(9 downto 5) > cv(9 downto 5) then dv := pv(9 downto 5) - cv(9 downto 5);
        else                                    dv := cv(9 downto 5) - pv(9 downto 5); end if;
        return resize(dy, C_DIST_WIDTH) + resize(du, C_DIST_WIDTH) + resize(dv, C_DIST_WIDTH);
    end function;

    -- ==== Controls (registered once, used directly -- CGA-style) ===========
    signal s_pixelate   : std_logic := '0';
    signal s_border_en  : std_logic := '0';
    signal s_scan_en    : std_logic := '0';
    signal s_bypass     : std_logic := '0';
    signal s_size       : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal s_shift      : unsigned(2 downto 0) := to_unsigned(3, 3);
    -- 8-bit multiplier coefficients (128 = unity) keep the LUT multipliers small
    signal s_contrast8  : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal s_sat8       : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_scan_str   : unsigned(9 downto 0) := to_unsigned(400, 10);
    signal s_mix_t      : unsigned(9 downto 0);

    -- ==== Timing / cell tracking ===========================================
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_x_count      : unsigned(11 downto 0) := (others => '0');
    signal s_y_count      : unsigned(11 downto 0) := (others => '0');
    signal s_line_width   : unsigned(11 downto 0) := to_unsigned(1280, 12);
    signal s_frame_height : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_cell_x   : unsigned(4 downto 0)  := (others => '0');
    signal s_cell_y   : unsigned(4 downto 0)  := (others => '0');
    signal s_cell_col : unsigned(10 downto 0) := (others => '0');

    signal s_acc_y, s_acc_u, s_acc_v : unsigned(13 downto 0) := (others => '0');

    -- Match-pipeline input (averaged at sample, else raw pixel)
    signal s_match_y : unsigned(9 downto 0) := (others => '0');
    signal s_match_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_match_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- T2: contrast/posterised match values (multiply isolated here)
    signal s_m_y, s_m_u, s_m_v : unsigned(9 downto 0) := (others => '0');

    -- T2b: distances
    type t_dist_arr is array(0 to 15) of unsigned(C_DIST_WIDTH - 1 downto 0);
    signal s_dist : t_dist_arr;

    -- T3: 16->4 winners
    signal s_gmin_a, s_gmin_b, s_gmin_c, s_gmin_d : unsigned(C_DIST_WIDTH - 1 downto 0);
    signal s_gidx_a, s_gidx_b, s_gidx_c, s_gidx_d : unsigned(3 downto 0);

    -- T4: per-pixel match index
    signal s_match_idx : unsigned(3 downto 0) := (others => '0');

    -- Per-column display buffer (single buffer, 1W1R -- exactly CGA)
    type t_pidx_buf is array(0 to C_MAX_COLS - 1) of unsigned(3 downto 0);
    signal s_pidx_buf : t_pidx_buf := (others => "0000");
    signal s_buf_we    : std_logic := '0';
    signal s_buf_waddr : unsigned(10 downto 0) := (others => '0');
    signal s_buf_wdata : unsigned(3 downto 0)  := (others => '0');
    signal s_buf_raddr : unsigned(10 downto 0) := (others => '0');
    signal s_buf_rdata : unsigned(3 downto 0)  := (others => '0');

    -- Buffer-write control pipeline (T1 -> T4)
    signal s_we_d1, s_we_d2, s_we_d3, s_we_d4 : std_logic := '0';
    signal s_wcol_d1, s_wcol_d2, s_wcol_d3, s_wcol_d4 : unsigned(10 downto 0) := (others => '0');

    -- T5 output
    signal s_out_y : unsigned(9 downto 0) := (others => '0');
    signal s_out_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_out_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- T6a -> T6b: saturation products + scanlined Y + border passthrough
    signal s_su_prod, s_sv_prod : signed(20 downto 0) := (others => '0');
    signal s_yscan : unsigned(9 downto 0) := (others => '0');
    signal s_pt_y, s_pt_u, s_pt_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_border_d6 : std_logic := '0';

    -- T6b processed output
    signal s_proc_y : unsigned(9 downto 0) := (others => '0');
    signal s_proc_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_proc_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Border / scanline flag pipelines (per-pixel, position-dependent)
    signal s_border_d1, s_border_d2, s_border_d3, s_border_d4, s_border_d5 : std_logic := '0';
    signal s_scan_d1, s_scan_d2, s_scan_d3, s_scan_d4, s_scan_d5 : std_logic := '0';

    -- Border colour pipeline
    signal s_bord_y, s_bord_u, s_bord_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bord_y1, s_bord_u1, s_bord_v1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bord_y2, s_bord_u2, s_bord_v2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bord_y3, s_bord_u3, s_bord_v3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bord_y4, s_bord_u4, s_bord_v4 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Bypass / sync shift registers
    type t_data_sr is array(0 to C_LATENCY - 1) of std_logic_vector(9 downto 0);
    type t_bit_sr  is array(0 to C_LATENCY - 1) of std_logic;
    signal s_y_sr, s_u_sr, s_v_sr : t_data_sr := (others => (others => '0'));
    signal s_hsync_sr, s_vsync_sr, s_field_sr, s_avid_sr : t_bit_sr := (others => '0');

    signal s_interp_y, s_interp_u, s_interp_v : unsigned(9 downto 0);

begin

    s_mix_t <= unsigned(registers_in(7));

    -- ====================================================================
    -- T1: controls, cell tracking, horizontal average, buffer-write setup
    -- ====================================================================
    p_t1 : process(clk)
        variable v_in_y, v_in_u, v_in_v : unsigned(9 downto 0);
        variable v_size  : unsigned(4 downto 0);
        variable v_shift : unsigned(2 downto 0);
        variable v_pix   : std_logic;
        variable v_at_cell_end : boolean;
        variable v_sum_y, v_sum_u, v_sum_v : unsigned(13 downto 0);
        variable v_bidx  : unsigned(3 downto 0);
        variable v_bwid  : unsigned(9 downto 0);
        variable v_inbord : std_logic;
    begin
        if rising_edge(clk) then
            v_in_y := unsigned(data_in.y);
            v_in_u := unsigned(data_in.u);
            v_in_v := unsigned(data_in.v);

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            -- ---- Controls (registered once, used directly) ----
            v_pix := registers_in(6)(0);
            s_pixelate  <= v_pix;
            s_border_en <= registers_in(6)(1);
            s_scan_en   <= registers_in(6)(2);
            s_bypass    <= registers_in(6)(4);
            s_contrast8 <= unsigned(registers_in(5)(9 downto 2));
            s_sat8      <= unsigned(registers_in(4)(9 downto 2));
            s_scan_str  <= unsigned(registers_in(3));

            -- Pixel size (knob 0 top 5 bits -> 5 zones). Pixelate off => 1x1.
            if v_pix = '0' then
                v_size := to_unsigned(1, 5); v_shift := to_unsigned(0, 3);
            else
                v_size := unsigned(registers_in(0)(9 downto 5));
                if    v_size < 6  then v_size := to_unsigned( 1, 5); v_shift := to_unsigned(0, 3);
                elsif v_size < 13 then v_size := to_unsigned( 2, 5); v_shift := to_unsigned(1, 3);
                elsif v_size < 19 then v_size := to_unsigned( 4, 5); v_shift := to_unsigned(2, 3);
                elsif v_size < 26 then v_size := to_unsigned( 8, 5); v_shift := to_unsigned(3, 3);
                else                   v_size := to_unsigned(16, 5); v_shift := to_unsigned(4, 3);
                end if;
            end if;
            s_size  <= v_size;
            s_shift <= v_shift;

            -- ---- Pixel/line counters ----
            if data_in.avid = '1' then
                s_x_count <= s_x_count + 1;
            end if;

            -- ---- Cell tracking + horizontal accumulator (first line) ----
            v_at_cell_end := false;
            v_sum_y := s_acc_y;  v_sum_u := s_acc_u;  v_sum_v := s_acc_v;

            if data_in.avid = '1' then
                if s_cell_x >= s_size - 1 then
                    s_cell_x   <= (others => '0');
                    s_cell_col <= s_cell_col + 1;
                    v_at_cell_end := true;
                else
                    s_cell_x <= s_cell_x + 1;
                end if;

                if s_cell_y = 0 then
                    if s_cell_x = 0 then
                        v_sum_y := resize(v_in_y, 14);
                        v_sum_u := resize(v_in_u, 14);
                        v_sum_v := resize(v_in_v, 14);
                    else
                        v_sum_y := s_acc_y + resize(v_in_y, 14);
                        v_sum_u := s_acc_u + resize(v_in_u, 14);
                        v_sum_v := s_acc_v + resize(v_in_v, 14);
                    end if;
                    s_acc_y <= v_sum_y;  s_acc_u <= v_sum_u;  s_acc_v <= v_sum_v;
                end if;
            end if;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_x_count > 0 then s_line_width <= s_x_count; end if;
                s_x_count  <= (others => '0');
                s_y_count  <= s_y_count + 1;
                s_cell_x   <= (others => '0');
                s_cell_col <= (others => '0');
                if s_cell_y >= s_size - 1 then
                    s_cell_y <= (others => '0');
                else
                    s_cell_y <= s_cell_y + 1;
                end if;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_y_count > 0 then s_frame_height <= s_y_count; end if;
                s_y_count  <= (others => '0');
                s_x_count  <= (others => '0');
                s_cell_x   <= (others => '0');
                s_cell_y   <= (others => '0');
                s_cell_col <= (others => '0');
            end if;

            -- ---- Match input: cell average at first-line cell-end, else raw ----
            if v_at_cell_end and s_cell_y = 0 and data_in.avid = '1' then
                s_match_y <= resize(v_sum_y srl to_integer(s_shift), 10);
                s_match_u <= resize(v_sum_u srl to_integer(s_shift), 10);
                s_match_v <= resize(v_sum_v srl to_integer(s_shift), 10);
            else
                s_match_y <= v_in_y;
                s_match_u <= v_in_u;
                s_match_v <= v_in_v;
            end if;

            -- ---- Buffer write enable + column (first-line cell-end only) ----
            if v_at_cell_end and s_cell_y = 0 and s_pixelate = '1' and data_in.avid = '1' then
                s_we_d1   <= '1';
                s_wcol_d1 <= s_cell_col;
            else
                s_we_d1   <= '0';
                s_wcol_d1 <= (others => '0');
            end if;

            -- ---- Border detection (position-dependent, pipelined) ----
            v_bwid := "00" & unsigned(registers_in(1)(9 downto 2));   -- 0-255 px
            if registers_in(6)(1) = '1' and (
                s_x_count < resize(v_bwid, 12) or
                s_x_count >= s_line_width - resize(v_bwid, 12) or
                s_y_count < resize(v_bwid, 12) or
                s_y_count >= s_frame_height - resize(v_bwid, 12)
            ) then
                v_inbord := '1';
            else
                v_inbord := '0';
            end if;
            s_border_d1 <= v_inbord;

            if registers_in(6)(2) = '1' then
                s_scan_d1 <= s_y_count(0);
            else
                s_scan_d1 <= '0';
            end if;

            v_bidx := unsigned(registers_in(2)(9 downto 6));
            s_bord_y <= C_PAL(to_integer(v_bidx)).y;
            s_bord_u <= C_PAL(to_integer(v_bidx)).u;
            s_bord_v <= C_PAL(to_integer(v_bidx)).v;

            -- ---- Bypass / sync shift registers ----
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
    -- T2: pre-match contrast + posterise, then 16 palette distances
    -- ====================================================================
    p_t2 : process(clk)
        -- Coarse mask (keep top 4 bits) widens the matching deadband so small
        -- frame-to-frame noise can't tip a block's chosen colour -> less
        -- scattered boundary flicker. Applied unconditionally (no mux on the
        -- critical s_m path); also gives the per-pixel path a cleaner look.
        constant C_POSTER : unsigned(9 downto 0) := "1111000000";  -- mask low 6
        variable v_cy_s   : signed(11 downto 0);
        variable v_cy_p   : signed(20 downto 0);
        variable v_cy_o   : signed(11 downto 0);
        variable v_my, v_mu, v_mv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- contrast: (Y-512) * contrast8 / 128 + 512  (128 = unity)
            v_cy_s := signed(resize(s_match_y, 12)) - to_signed(512, 12);
            v_cy_p := v_cy_s * signed('0' & s_contrast8);
            v_cy_o := v_cy_p(18 downto 7) + to_signed(512, 12);
            if    v_cy_o < 0    then v_my := (others => '0');
            elsif v_cy_o > 1023 then v_my := to_unsigned(1023, 10);
            else                     v_my := unsigned(v_cy_o(9 downto 0)); end if;

            s_m_y <= v_my and C_POSTER;
            s_m_u <= s_match_u and C_POSTER;
            s_m_v <= s_match_v and C_POSTER;

            s_we_d2     <= s_we_d1;
            s_wcol_d2   <= s_wcol_d1;
            s_border_d2 <= s_border_d1;
            s_scan_d2   <= s_scan_d1;
            s_bord_y1 <= s_bord_y;  s_bord_u1 <= s_bord_u;  s_bord_v1 <= s_bord_v;
        end if;
    end process p_t2;

    -- ====================================================================
    -- T2b: 16 palette distances (separated from the contrast multiply)
    -- ====================================================================
    p_t2b : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to 15 loop
                s_dist(i) <= f_dist(s_m_y, s_m_u, s_m_v, C_PAL(i).y, C_PAL(i).u, C_PAL(i).v);
            end loop;

            s_we_d3     <= s_we_d2;
            s_wcol_d3   <= s_wcol_d2;
            s_border_d3 <= s_border_d2;
            s_scan_d3   <= s_scan_d2;
            s_bord_y2 <= s_bord_y1;  s_bord_u2 <= s_bord_u1;  s_bord_v2 <= s_bord_v1;
        end if;
    end process p_t2b;

    -- ====================================================================
    -- T3: 16->4 reduction
    -- ====================================================================
    p_t3 : process(clk)
        variable v_lo, v_hi : unsigned(C_DIST_WIDTH - 1 downto 0);
        variable v_loi, v_hii : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- Group A (0..3) as a 2-deep tree
            if s_dist(1) < s_dist(0) then v_lo := s_dist(1); v_loi := x"1";
            else                          v_lo := s_dist(0); v_loi := x"0"; end if;
            if s_dist(3) < s_dist(2) then v_hi := s_dist(3); v_hii := x"3";
            else                          v_hi := s_dist(2); v_hii := x"2"; end if;
            if v_hi < v_lo then s_gmin_a <= v_hi; s_gidx_a <= v_hii;
            else                s_gmin_a <= v_lo; s_gidx_a <= v_loi; end if;

            -- Group B (4..7)
            if s_dist(5) < s_dist(4) then v_lo := s_dist(5); v_loi := x"5";
            else                          v_lo := s_dist(4); v_loi := x"4"; end if;
            if s_dist(7) < s_dist(6) then v_hi := s_dist(7); v_hii := x"7";
            else                          v_hi := s_dist(6); v_hii := x"6"; end if;
            if v_hi < v_lo then s_gmin_b <= v_hi; s_gidx_b <= v_hii;
            else                s_gmin_b <= v_lo; s_gidx_b <= v_loi; end if;

            -- Group C (8..11)
            if s_dist(9) < s_dist(8) then v_lo := s_dist(9); v_loi := x"9";
            else                          v_lo := s_dist(8); v_loi := x"8"; end if;
            if s_dist(11) < s_dist(10) then v_hi := s_dist(11); v_hii := x"B";
            else                            v_hi := s_dist(10); v_hii := x"A"; end if;
            if v_hi < v_lo then s_gmin_c <= v_hi; s_gidx_c <= v_hii;
            else                s_gmin_c <= v_lo; s_gidx_c <= v_loi; end if;

            -- Group D (12..15)
            if s_dist(13) < s_dist(12) then v_lo := s_dist(13); v_loi := x"D";
            else                            v_lo := s_dist(12); v_loi := x"C"; end if;
            if s_dist(15) < s_dist(14) then v_hi := s_dist(15); v_hii := x"F";
            else                            v_hi := s_dist(14); v_hii := x"E"; end if;
            if v_hi < v_lo then s_gmin_d <= v_hi; s_gidx_d <= v_hii;
            else                s_gmin_d <= v_lo; s_gidx_d <= v_loi; end if;

            s_we_d4     <= s_we_d3;
            s_wcol_d4   <= s_wcol_d3;
            s_border_d4 <= s_border_d3;
            s_scan_d4   <= s_scan_d3;
            s_bord_y3 <= s_bord_y2;  s_bord_u3 <= s_bord_u2;  s_bord_v3 <= s_bord_v2;
        end if;
    end process p_t3;

    -- ====================================================================
    -- T4: 4->1 reduction -> index, register the buffer write
    -- ====================================================================
    p_t4 : process(clk)
        variable v_lo, v_hi : unsigned(C_DIST_WIDTH - 1 downto 0);
        variable v_loi, v_hii, v_idx : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 4->1 as a 2-deep tree
            if s_gmin_b < s_gmin_a then v_lo := s_gmin_b; v_loi := s_gidx_b;
            else                        v_lo := s_gmin_a; v_loi := s_gidx_a; end if;
            if s_gmin_d < s_gmin_c then v_hi := s_gmin_d; v_hii := s_gidx_d;
            else                        v_hi := s_gmin_c; v_hii := s_gidx_c; end if;
            if v_hi < v_lo then v_idx := v_hii; else v_idx := v_loi; end if;
            s_match_idx <= v_idx;

            s_buf_we    <= s_we_d4;
            s_buf_waddr <= s_wcol_d4;
            s_buf_wdata <= v_idx;

            s_border_d5 <= s_border_d4;
            s_scan_d5   <= s_scan_d4;
            s_bord_y4 <= s_bord_y3;  s_bord_u4 <= s_bord_u3;  s_bord_v4 <= s_bord_v3;
        end if;
    end process p_t4;

    -- ====================================================================
    -- Per-column display buffer (single 1W1R BRAM -- exactly CGA's pattern)
    -- ====================================================================
    p_buf : process(clk)
    begin
        if rising_edge(clk) then
            if s_buf_we = '1' then
                s_pidx_buf(to_integer(s_buf_waddr)) <= s_buf_wdata;
            end if;
            s_buf_rdata <= s_pidx_buf(to_integer(s_buf_raddr));
        end if;
    end process p_buf;
    s_buf_raddr <= s_cell_col;

    -- ====================================================================
    -- T5: display palette lookup (buffer for pixelate, per-pixel else) +
    --     border overlay
    -- ====================================================================
    p_t5 : process(clk)
        variable v_idx : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            if s_pixelate = '1' then
                v_idx := to_integer(s_buf_rdata);
            else
                v_idx := to_integer(s_match_idx);
            end if;
            s_out_y <= C_PAL(v_idx).y;
            s_out_u <= C_PAL(v_idx).u;
            s_out_v <= C_PAL(v_idx).v;

            if s_border_d5 = '1' then
                s_out_y <= s_bord_y4;
                s_out_u <= s_bord_u4;
                s_out_v <= s_bord_v4;
            end if;
        end if;
    end process p_t5;

    -- ====================================================================
    -- T6a: scanline dim + saturation multiplies (isolated)
    -- ====================================================================
    p_t6a : process(clk)
        variable v_dim : unsigned(19 downto 0);
        variable v_su, v_sv : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_scan_d5 = '1' then
                v_dim    := s_out_y * (to_unsigned(1023, 10) - s_scan_str);
                s_yscan  <= v_dim(19 downto 10);
            else
                s_yscan  <= s_out_y;
            end if;

            -- saturation: (ch-512) * sat8 / 128 + 512  (128 = unity)
            v_su := signed(resize(s_out_u, 12)) - to_signed(512, 12);
            v_sv := signed(resize(s_out_v, 12)) - to_signed(512, 12);
            s_su_prod <= v_su * signed('0' & s_sat8);
            s_sv_prod <= v_sv * signed('0' & s_sat8);

            s_pt_y <= s_out_y;  s_pt_u <= s_out_u;  s_pt_v <= s_out_v;  -- border passthrough
            s_border_d6 <= s_border_d5;
        end if;
    end process p_t6a;

    -- ====================================================================
    -- T6b: saturation slice/clamp -> processed output
    -- ====================================================================
    p_t6b : process(clk)
        variable v_suo, v_svo : signed(11 downto 0);
    begin
        if rising_edge(clk) then
          if s_border_d6 = '1' then
            s_proc_y <= s_pt_y;  s_proc_u <= s_pt_u;  s_proc_v <= s_pt_v;
          else
            s_proc_y <= s_yscan;
            v_suo := s_su_prod(18 downto 7) + to_signed(512, 12);
            v_svo := s_sv_prod(18 downto 7) + to_signed(512, 12);
            if    v_suo < 0    then s_proc_u <= (others => '0');
            elsif v_suo > 1023 then s_proc_u <= to_unsigned(1023, 10);
            else                    s_proc_u <= unsigned(v_suo(9 downto 0)); end if;
            if    v_svo < 0    then s_proc_v <= (others => '0');
            elsif v_svo > 1023 then s_proc_v <= to_unsigned(1023, 10);
            else                    s_proc_v <= unsigned(v_svo(9 downto 0)); end if;
          end if;
        end if;
    end process p_t6b;

    -- ====================================================================
    -- Interpolator (T7-T10), dry tap at SR(5)
    -- ====================================================================
    interp_y : entity work.interpolator_u
        generic map(G_WIDTH => C_W, G_FRAC_BITS => C_W, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1', a => unsigned(s_y_sr(7)), b => s_proc_y,
                 t => s_mix_t, result => s_interp_y, valid => open);
    interp_u : entity work.interpolator_u
        generic map(G_WIDTH => C_W, G_FRAC_BITS => C_W, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1', a => unsigned(s_u_sr(7)), b => s_proc_u,
                 t => s_mix_t, result => s_interp_u, valid => open);
    interp_v : entity work.interpolator_u
        generic map(G_WIDTH => C_W, G_FRAC_BITS => C_W, G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map(clk => clk, enable => '1', a => unsigned(s_v_sr(7)), b => s_proc_v,
                 t => s_mix_t, result => s_interp_v, valid => open);

    -- ====================================================================
    -- Output at T10
    -- ====================================================================
    data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);

    data_out.y <= s_y_sr(C_LATENCY - 1) when s_bypass = '1' else std_logic_vector(s_interp_y);
    data_out.u <= s_u_sr(C_LATENCY - 1) when s_bypass = '1' else std_logic_vector(s_interp_u);
    data_out.v <= s_v_sr(C_LATENCY - 1) when s_bypass = '1' else std_logic_vector(s_interp_v);

end architecture c64;
