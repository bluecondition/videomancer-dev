-- Videomancer CGA (Color Graphics Adapter) Emulator
-- LZX Industries Community FPGA Program
--
-- Pixelation algorithm: downsample + nearest-neighbour upscale.
--   1. DOWNSAMPLE: each horizontal cell on the FIRST line of its cell row
--      accumulates H input pixels and divides for the average colour.
--   2. STORE: palette-match the average and write the 2-bit index into a
--      per-cell-column buffer indexed by column number.
--   3. UPSCALE (nearest-neighbour): every output pixel reads the palette
--      index for its cell column and emits the corresponding colour, giving
--      solid blocks across all V lines of each cell row.
--
-- No anti-flicker code. Matching uses the top 5 bits of Y/U/V (32 levels per
-- channel). Any frame-to-frame noise in the averaged value that crosses
-- the 32-unit quantum on a boundary between two palette colours will show
-- up as flicker.
--
-- License: GPL-3.0
-- Author: Claude (Anthropic)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cga of program_top is

  -- ==========================================================================
  -- Constants
  -- ==========================================================================

  constant C_LATENCY  : integer := 9;
  constant C_MAX_COLS : integer := 2048;  -- 2048x2 fits one iCE40 BRAM; covers
                                          -- up to 1920 cells/line at H=1 (Off).

  -- Palette entry type
  type t_pal_entry is record
    y : unsigned(9 downto 0);
    u : unsigned(9 downto 0);
    v : unsigned(9 downto 0);
  end record;

  type t_cga_pal4 is array(0 to 3) of t_pal_entry;
  type t_cga_pals is array(0 to 5) of t_cga_pal4;

  -- Six CGA palettes: 3 modes × 2 intensities (U/V swapped for Videomancer)
  constant C_CGA_PAL : t_cga_pals := (
    -- Palette 0 Low: Black, Red, Green, Yellow
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(204, 10), u => to_unsigned(852, 10), v => to_unsigned(396, 10)),
      (y => to_unsigned(400, 10), u => to_unsigned(226, 10), v => to_unsigned(286, 10)),
      (y => to_unsigned(404, 10), u => to_unsigned(709, 10), v => to_unsigned(283, 10))
    ),
    -- Palette 0 High: Black, Red, Green, Yellow
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(546, 10), u => to_unsigned(853, 10), v => to_unsigned(397, 10)),
      (y => to_unsigned(743, 10), u => to_unsigned(226, 10), v => to_unsigned(286, 10)),
      (y => to_unsigned(947, 10), u => to_unsigned(568, 10), v => to_unsigned(171, 10))
    ),
    -- Palette 1 Low: Black, Cyan, Magenta, Lt Gray
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(478, 10), u => to_unsigned(171, 10), v => to_unsigned(627, 10)),
      (y => to_unsigned(282, 10), u => to_unsigned(798, 10), v => to_unsigned(738, 10)),
      (y => to_unsigned(683, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    ),
    -- Palette 1 High: Black, Lt Cyan, Lt Magenta, White
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(820, 10), u => to_unsigned(171, 10), v => to_unsigned(628, 10)),
      (y => to_unsigned(624, 10), u => to_unsigned(798, 10), v => to_unsigned(739, 10)),
      (y => to_unsigned(1023, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    ),
    -- Palette 2 Low: Black, Cyan, Red, Lt Gray
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(478, 10), u => to_unsigned(171, 10), v => to_unsigned(627, 10)),
      (y => to_unsigned(204, 10), u => to_unsigned(852, 10), v => to_unsigned(396, 10)),
      (y => to_unsigned(683, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    ),
    -- Palette 2 High: Black, Lt Cyan, Lt Red, White
    (
      (y => to_unsigned(0, 10),   u => to_unsigned(512, 10), v => to_unsigned(512, 10)),
      (y => to_unsigned(820, 10), u => to_unsigned(171, 10), v => to_unsigned(628, 10)),
      (y => to_unsigned(546, 10), u => to_unsigned(853, 10), v => to_unsigned(397, 10)),
      (y => to_unsigned(1023, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))
    )
  );

  -- ==========================================================================
  -- Manhattan distance on top 5 bits (32 levels per channel)
  -- ==========================================================================
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
    return resize(dy, 8) + resize(du, 8) + resize(dv, 8);
  end function;

  -- ==========================================================================
  -- Luma-mode palette mapping
  -- ==========================================================================
  -- For each of the 6 palette variants, maps a 2-bit input-luma zone
  -- (0 = darkest quarter of Y, 3 = brightest) to the palette entry index
  -- whose luma sorts to that rank.  Computed offline by sorting each
  -- palette's 4 entries by Y value.  Entries within a palette with the
  -- same rank use the entry whose Y happens to be closest (stable sort).
  type t_luma_map  is array(0 to 3) of unsigned(1 downto 0);
  type t_luma_maps is array(0 to 5) of t_luma_map;
  constant C_LUMA_MAP : t_luma_maps := (
    -- Pal 0 Low:  Y = {0, 204, 400, 404} -> sorted indices {0,1,2,3}
    0 => (0 => "00", 1 => "01", 2 => "10", 3 => "11"),
    -- Pal 0 High: Y = {0, 546, 743, 947} -> {0,1,2,3}
    1 => (0 => "00", 1 => "01", 2 => "10", 3 => "11"),
    -- Pal 1 Low:  Y = {0, 478, 282, 683} -> {0,2,1,3}
    2 => (0 => "00", 1 => "10", 2 => "01", 3 => "11"),
    -- Pal 1 High: Y = {0, 820, 624, 1023} -> {0,2,1,3}
    3 => (0 => "00", 1 => "10", 2 => "01", 3 => "11"),
    -- Pal 2 Low:  Y = {0, 478, 204, 683} -> {0,2,1,3}
    4 => (0 => "00", 1 => "10", 2 => "01", 3 => "11"),
    -- Pal 2 High: Y = {0, 820, 546, 1023} -> {0,2,1,3}
    5 => (0 => "00", 1 => "10", 2 => "01", 3 => "11")
  );

  -- ==========================================================================
  -- Signal declarations
  -- ==========================================================================

  -- Sync handling
  signal s_pixel_y : unsigned(11 downto 0) := (others => '0');
  signal s_prev_hsync_n : std_logic := '1';
  signal s_prev_vsync_n : std_logic := '1';

  -- Shift registers for sync passthrough and dry-tap
  type t_sr_logic is array(0 to C_LATENCY - 1) of std_logic;
  type t_sr_yuv   is array(0 to C_LATENCY - 1) of t_video_stream_yuv444_30b;
  signal s_avid_sr  : t_sr_logic := (others => '0');
  signal s_hsync_sr : t_sr_logic := (others => '1');
  signal s_vsync_sr : t_sr_logic := (others => '1');
  signal s_field_sr : t_sr_logic := (others => '0');
  signal s_yuv_sr   : t_sr_yuv;

  -- Cell sizing (decoded from knobs 2, 3)
  signal s_h_size  : unsigned(4 downto 0) := to_unsigned(4, 5);
  signal s_v_size  : unsigned(4 downto 0) := to_unsigned(4, 5);
  signal s_h_shift : unsigned(2 downto 0) := to_unsigned(2, 3);

  -- Cell tracking
  signal s_cell_x   : unsigned(4 downto 0)  := (others => '0');
  signal s_cell_y   : unsigned(4 downto 0)  := (others => '0');
  signal s_cell_col : unsigned(10 downto 0) := (others => '0');

  -- Horizontal accumulator (first line of cell row only)
  signal s_acc_y : unsigned(13 downto 0) := (others => '0');
  signal s_acc_u : unsigned(13 downto 0) := (others => '0');
  signal s_acc_v : unsigned(13 downto 0) := (others => '0');

  -- Per-cell palette-index buffer
  type t_pidx_buf is array(0 to C_MAX_COLS - 1) of unsigned(1 downto 0);
  signal s_pidx_buf : t_pidx_buf := (others => "00");

  signal s_buf_we      : std_logic := '0';
  signal s_buf_waddr   : unsigned(10 downto 0) := (others => '0');
  signal s_buf_wdata   : unsigned(1 downto 0) := (others => '0');
  signal s_buf_raddr   : unsigned(10 downto 0) := (others => '0');
  signal s_buf_rdata   : unsigned(1 downto 0) := (others => '0');

  -- Palette match pipeline input
  signal s_match_y : unsigned(9 downto 0) := (others => '0');
  signal s_match_u : unsigned(9 downto 0) := (others => '0');
  signal s_match_v : unsigned(9 downto 0) := (others => '0');

  type t_dist_arr is array(0 to 3) of unsigned(7 downto 0);
  signal s_dist : t_dist_arr := (others => (others => '0'));
  signal s_match_idx : unsigned(1 downto 0) := (others => '0');

  -- Luma-mode: palette entry chosen by input-Y zone (set in T2, used in T3)
  signal s_luma_idx : unsigned(1 downto 0) := (others => '0');

  -- Switch 10: '0' = nearest-colour (Manhattan), '1' = luma-zone mapping
  signal s_color_mode : std_logic := '0';

  -- Pipelined fill (write to buffer)
  signal s_we_d1, s_we_d2 : std_logic := '0';
  signal s_wcol_d1, s_wcol_d2 : unsigned(10 downto 0) := (others => '0');

  -- Pipelined palette selection
  signal s_pal_sel    : unsigned(2 downto 0) := (others => '0');
  signal s_pal_sel_d1 : unsigned(2 downto 0) := (others => '0');
  signal s_pal_sel_d2 : unsigned(2 downto 0) := (others => '0');
  signal s_pal_sel_d3 : unsigned(2 downto 0) := (others => '0');

  -- T4 -> T5 saturation intermediates
  signal s_t4_su_prod : signed(22 downto 0) := (others => '0');
  signal s_t4_sv_prod : signed(22 downto 0) := (others => '0');
  signal s_t4_y       : unsigned(9 downto 0) := (others => '0');

  -- Output
  signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

  -- Controls
  signal s_bypass     : std_logic := '0';
  signal s_pixelate   : std_logic := '0';
  signal s_scanlines  : std_logic := '0';
  signal s_contrast   : unsigned(9 downto 0) := (others => '0');
  signal s_saturation : unsigned(9 downto 0) := to_unsigned(512, 10);
  signal s_mix_t      : unsigned(9 downto 0) := (others => '0');

  signal s_interp_y_result : unsigned(9 downto 0);
  signal s_interp_u_result : unsigned(9 downto 0);
  signal s_interp_v_result : unsigned(9 downto 0);

begin

  s_bypass <= registers_in(6)(4);
  s_mix_t  <= unsigned(registers_in(7));

  -- ==========================================================================
  -- Pipeline T1: input register, controls, cell tracking, accumulator.
  -- ==========================================================================
  p_t1 : process(clk)
    variable v_h_size  : unsigned(4 downto 0);
    variable v_v_size  : unsigned(4 downto 0);
    variable v_h_shift : unsigned(2 downto 0);
    variable v_pal_idx : unsigned(1 downto 0);
    variable v_in_y, v_in_u, v_in_v : unsigned(9 downto 0);
    variable v_at_cell_end : boolean;
    variable v_sum_y, v_sum_u, v_sum_v : unsigned(13 downto 0);
    variable v_avg_y, v_avg_u, v_avg_v : unsigned(9 downto 0);
  begin
    if rising_edge(clk) then
      s_avid_sr(0)  <= data_in.avid;
      s_hsync_sr(0) <= data_in.hsync_n;
      s_vsync_sr(0) <= data_in.vsync_n;
      s_field_sr(0) <= data_in.field_n;
      s_yuv_sr(0)   <= data_in;
      for i in 1 to C_LATENCY - 1 loop
        s_avid_sr(i)  <= s_avid_sr(i - 1);
        s_hsync_sr(i) <= s_hsync_sr(i - 1);
        s_vsync_sr(i) <= s_vsync_sr(i - 1);
        s_field_sr(i) <= s_field_sr(i - 1);
        s_yuv_sr(i)   <= s_yuv_sr(i - 1);
      end loop;

      s_prev_hsync_n <= data_in.hsync_n;
      s_prev_vsync_n <= data_in.vsync_n;

      -- Controls
      s_pixelate   <= registers_in(6)(0);
      s_scanlines  <= registers_in(6)(2);
      s_color_mode <= registers_in(6)(3);  -- switch 10: nearest vs luma
      s_contrast   <= unsigned(registers_in(3));
      s_saturation <= unsigned(registers_in(4));

      -- H/V sizes. Pixelate OFF forces 1x1 (per-pixel quantisation).
      -- Zone thresholds match the framework's 5-label divide of the 0-1023
      -- register range (5 equal zones of ~205 units), so the displayed
      -- label always matches the actual block size.
      if registers_in(6)(0) = '0' then
        v_h_size  := to_unsigned(1, 5);
        v_v_size  := to_unsigned(1, 5);
        v_h_shift := to_unsigned(0, 3);
      else
        if    unsigned(registers_in(1)) < 205 then v_h_size := to_unsigned( 1, 5); v_h_shift := to_unsigned(0, 3);
        elsif unsigned(registers_in(1)) < 410 then v_h_size := to_unsigned( 2, 5); v_h_shift := to_unsigned(1, 3);
        elsif unsigned(registers_in(1)) < 615 then v_h_size := to_unsigned( 4, 5); v_h_shift := to_unsigned(2, 3);
        elsif unsigned(registers_in(1)) < 820 then v_h_size := to_unsigned( 8, 5); v_h_shift := to_unsigned(3, 3);
        else                                       v_h_size := to_unsigned(16, 5); v_h_shift := to_unsigned(4, 3);
        end if;

        if    unsigned(registers_in(2)) < 205 then v_v_size := to_unsigned( 1, 5);
        elsif unsigned(registers_in(2)) < 410 then v_v_size := to_unsigned( 2, 5);
        elsif unsigned(registers_in(2)) < 615 then v_v_size := to_unsigned( 4, 5);
        elsif unsigned(registers_in(2)) < 820 then v_v_size := to_unsigned( 8, 5);
        else                                       v_v_size := to_unsigned(16, 5);
        end if;
      end if;
      s_h_size  <= v_h_size;
      s_v_size  <= v_v_size;
      s_h_shift <= v_h_shift;

      -- Palette selection (knob 1, 3 zones; switch 8 = intensity)
      v_pal_idx := "00";
      if    unsigned(registers_in(0)) >= 683 then v_pal_idx := "10";
      elsif unsigned(registers_in(0)) >= 341 then v_pal_idx := "01";
      end if;
      s_pal_sel <= v_pal_idx & registers_in(6)(1);

      -- Current input
      v_in_y := unsigned(data_in.y);
      v_in_u := unsigned(data_in.u);
      v_in_v := unsigned(data_in.v);

      -- Cell tracking + horizontal accumulator
      v_at_cell_end := false;
      v_sum_y := s_acc_y;
      v_sum_u := s_acc_u;
      v_sum_v := s_acc_v;

      if data_in.avid = '1' then
        if s_cell_x >= v_h_size - 1 then
          s_cell_x <= (others => '0');
          v_at_cell_end := true;
          s_cell_col <= s_cell_col + 1;
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
          s_acc_y <= v_sum_y;
          s_acc_u <= v_sum_u;
          s_acc_v <= v_sum_v;
        end if;
      end if;

      if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
        s_cell_x   <= (others => '0');
        s_cell_col <= (others => '0');
        s_pixel_y  <= s_pixel_y + 1;
        if s_cell_y >= v_v_size - 1 then
          s_cell_y <= (others => '0');
        else
          s_cell_y <= s_cell_y + 1;
        end if;
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        s_pixel_y  <= (others => '0');
        s_cell_x   <= (others => '0');
        s_cell_y   <= (others => '0');
        s_cell_col <= (others => '0');
      end if;

      -- Match input: averaged colour at cell-end on line 0, else pass input.
      if v_at_cell_end and s_cell_y = 0 and data_in.avid = '1' then
        v_avg_y := resize(v_sum_y srl to_integer(v_h_shift), 10);
        v_avg_u := resize(v_sum_u srl to_integer(v_h_shift), 10);
        v_avg_v := resize(v_sum_v srl to_integer(v_h_shift), 10);
        s_match_y <= v_avg_y;
        s_match_u <= v_avg_u;
        s_match_v <= v_avg_v;
      else
        s_match_y <= v_in_y;
        s_match_u <= v_in_u;
        s_match_v <= v_in_v;
      end if;

      -- Buffer write enable + address (pipeline to match T3 timing)
      if v_at_cell_end and s_cell_y = 0 and s_pixelate = '1' and data_in.avid = '1' then
        s_we_d1   <= '1';
        s_wcol_d1 <= s_cell_col;
      else
        s_we_d1   <= '0';
        s_wcol_d1 <= (others => '0');
      end if;

      s_pal_sel_d1 <= s_pal_sel;
    end if;
  end process p_t1;

  -- ==========================================================================
  -- Pipeline T2: contrast, compute 4 palette distances on match input.
  -- ==========================================================================
  p_t2 : process(clk)
    variable v_y_c : unsigned(9 downto 0);
  begin
    if rising_edge(clk) then
      if s_contrast < 512 then
        v_y_c := resize((s_match_y * resize(s_contrast, 9)) srl 9, 10);
      else
        v_y_c := resize((s_match_y + ((s_match_y * resize(s_contrast - 512, 9)) srl 9)), 10);
      end if;

      for i in 0 to 3 loop
        s_dist(i) <= f_dist(v_y_c, s_match_u, s_match_v,
                            C_CGA_PAL(to_integer(s_pal_sel_d1))(i).y,
                            C_CGA_PAL(to_integer(s_pal_sel_d1))(i).u,
                            C_CGA_PAL(to_integer(s_pal_sel_d1))(i).v);
      end loop;

      -- Luma-zone mapping: top 2 bits of contrast-adjusted Y pick the
      -- palette entry whose luma rank matches.
      s_luma_idx <= C_LUMA_MAP(to_integer(s_pal_sel_d1))
                              (to_integer(v_y_c(9 downto 8)));

      s_pal_sel_d2 <= s_pal_sel_d1;
      s_we_d2   <= s_we_d1;
      s_wcol_d2 <= s_wcol_d1;
    end if;
  end process p_t2;

  -- ==========================================================================
  -- Pipeline T3: 4-way minimum -> palette index. Register buffer write.
  -- ==========================================================================
  p_t3 : process(clk)
    variable v_idx : unsigned(1 downto 0);
    variable v_min : unsigned(7 downto 0);
  begin
    if rising_edge(clk) then
      if s_color_mode = '1' then
        -- Luma-zone mode: use the precomputed luma index directly
        v_idx := s_luma_idx;
      else
        -- Nearest-colour mode: 4-way min of Manhattan distances
        v_min := s_dist(0); v_idx := "00";
        if s_dist(1) < v_min then v_min := s_dist(1); v_idx := "01"; end if;
        if s_dist(2) < v_min then v_min := s_dist(2); v_idx := "10"; end if;
        if s_dist(3) < v_min then v_min := s_dist(3); v_idx := "11"; end if;
      end if;
      s_match_idx <= v_idx;

      s_buf_we    <= s_we_d2;
      s_buf_waddr <= s_wcol_d2;
      s_buf_wdata <= v_idx;

      s_pal_sel_d3 <= s_pal_sel_d2;
    end if;
  end process p_t3;

  -- ==========================================================================
  -- Per-cell-column palette-index BRAM
  -- ==========================================================================
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

  -- ==========================================================================
  -- Pipeline T4: palette lookup + start saturation scaling (subtract 512,
  -- then 12x11 signed multiply).  The multiply alone is too long for the
  -- HD clock; the slice/add/clamp is deferred to T5.
  -- ==========================================================================
  p_t4 : process(clk)
    variable v_disp_y, v_disp_u, v_disp_v : unsigned(9 downto 0);
    variable v_su, v_sv : signed(11 downto 0);
  begin
    if rising_edge(clk) then
      if s_pixelate = '1' then
        v_disp_y := C_CGA_PAL(to_integer(s_pal_sel_d3))(to_integer(s_buf_rdata)).y;
        v_disp_u := C_CGA_PAL(to_integer(s_pal_sel_d3))(to_integer(s_buf_rdata)).u;
        v_disp_v := C_CGA_PAL(to_integer(s_pal_sel_d3))(to_integer(s_buf_rdata)).v;
      else
        v_disp_y := C_CGA_PAL(to_integer(s_pal_sel_d3))(to_integer(s_match_idx)).y;
        v_disp_u := C_CGA_PAL(to_integer(s_pal_sel_d3))(to_integer(s_match_idx)).u;
        v_disp_v := C_CGA_PAL(to_integer(s_pal_sel_d3))(to_integer(s_match_idx)).v;
      end if;

      v_su := signed(resize(v_disp_u, 12)) - to_signed(512, 12);
      v_sv := signed(resize(v_disp_v, 12)) - to_signed(512, 12);
      s_t4_su_prod <= v_su * signed('0' & s_saturation);
      s_t4_sv_prod <= v_sv * signed('0' & s_saturation);
      s_t4_y       <= v_disp_y;
    end if;
  end process p_t4;

  -- ==========================================================================
  -- Pipeline T5: finish saturation (shift/add/clamp), apply scanlines, output.
  -- ==========================================================================
  p_t5 : process(clk)
    variable v_su_out, v_sv_out : signed(11 downto 0);
    variable v_final_y : unsigned(9 downto 0);
    variable v_u, v_v : unsigned(9 downto 0);
  begin
    if rising_edge(clk) then
      v_su_out := s_t4_su_prod(20 downto 9) + to_signed(512, 12);
      v_sv_out := s_t4_sv_prod(20 downto 9) + to_signed(512, 12);

      if    v_su_out < 0    then v_u := (others => '0');
      elsif v_su_out > 1023 then v_u := to_unsigned(1023, 10);
      else                       v_u := unsigned(v_su_out(9 downto 0));
      end if;
      if    v_sv_out < 0    then v_v := (others => '0');
      elsif v_sv_out > 1023 then v_v := to_unsigned(1023, 10);
      else                       v_v := unsigned(v_sv_out(9 downto 0));
      end if;

      v_final_y := s_t4_y;
      if s_scanlines = '1' and s_pixel_y(0) = '1' then
        v_final_y := v_final_y srl 1;
      end if;

      s_out_y <= v_final_y;
      s_out_u <= v_u;
      s_out_v <= v_v;
    end if;
  end process p_t5;

  -- ==========================================================================
  -- Interpolator (wet/dry mix)
  -- ==========================================================================
  interp_y : entity work.interpolator_u
    generic map(G_WIDTH => 10, G_FRAC_BITS => 10,
                G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map(clk => clk, enable => '1',
             a => unsigned(s_yuv_sr(4).y), b => s_out_y, t => s_mix_t,
             result => s_interp_y_result, valid => open);

  interp_u : entity work.interpolator_u
    generic map(G_WIDTH => 10, G_FRAC_BITS => 10,
                G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map(clk => clk, enable => '1',
             a => unsigned(s_yuv_sr(4).u), b => s_out_u, t => s_mix_t,
             result => s_interp_u_result, valid => open);

  interp_v : entity work.interpolator_u
    generic map(G_WIDTH => 10, G_FRAC_BITS => 10,
                G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
    port map(clk => clk, enable => '1',
             a => unsigned(s_yuv_sr(4).v), b => s_out_v, t => s_mix_t,
             result => s_interp_v_result, valid => open);

  -- ==========================================================================
  -- Output
  -- ==========================================================================
  data_out.y <= s_yuv_sr(C_LATENCY - 1).y when s_bypass = '1'
                else std_logic_vector(s_interp_y_result);
  data_out.u <= s_yuv_sr(C_LATENCY - 1).u when s_bypass = '1'
                else std_logic_vector(s_interp_u_result);
  data_out.v <= s_yuv_sr(C_LATENCY - 1).v when s_bypass = '1'
                else std_logic_vector(s_interp_v_result);

  data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
  data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
  data_out.field_n <= s_field_sr(C_LATENCY - 1);
  data_out.avid    <= s_avid_sr(C_LATENCY - 1);

end architecture cga;
