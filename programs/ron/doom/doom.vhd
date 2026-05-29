-- Videomancer Doom Raycaster Tech Demo (Optimized for HX4K)
-- LZX Industries Community FPGA Program
-- Real-time 3D raycasting with 80-column DDA engine
--
-- License: GPL-3.0
-- Author: Claude (Anthropic)
-- Optimizations: 4-stage pipeline, 80 columns, 16x16 map, 8-bit angle, 10.4 fixed-point position

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture doom of program_top is

  -- ==========================================================================
  -- Constants - Optimized for HX4K resource constraints
  -- ==========================================================================

  constant C_TOTAL_LATENCY : integer := 4;   -- Reduced from 6 to 4 stages
  constant C_NUM_COLS      : integer := 80;  -- Reduced from 160
  constant C_MAP_SIZE      : integer := 16;  -- Reduced from 32

  -- Wall color palette entry type
  type t_pal_entry is record
    y : unsigned(9 downto 0);
    u : unsigned(9 downto 0);
    v : unsigned(9 downto 0);
  end record;
  type t_palette is array(0 to 14) of t_pal_entry;

  -- Wall color palette (15 colors)
  constant C_WALL_PALETTE : t_palette := (
    (y => to_unsigned(512, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)), -- 0: neutral gray
    (y => to_unsigned(450, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)), -- 1: dark gray
    (y => to_unsigned(600, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)), -- 2: light gray
    (y => to_unsigned(350, 10), u => to_unsigned(420, 10), v => to_unsigned(570, 10)), -- 3: brown
    (y => to_unsigned(300, 10), u => to_unsigned(380, 10), v => to_unsigned(580, 10)), -- 4: dark brown
    (y => to_unsigned(400, 10), u => to_unsigned(450, 10), v => to_unsigned(560, 10)), -- 5: medium brown
    (y => to_unsigned(273, 10), u => to_unsigned(455, 10), v => to_unsigned(660, 10)), -- 6: red brick
    (y => to_unsigned(220, 10), u => to_unsigned(445, 10), v => to_unsigned(670, 10)), -- 7: dark red
    (y => to_unsigned(320, 10), u => to_unsigned(465, 10), v => to_unsigned(650, 10)), -- 8: medium red
    (y => to_unsigned(470, 10), u => to_unsigned(485, 10), v => to_unsigned(530, 10)), -- 9: light tan
    (y => to_unsigned(550, 10), u => to_unsigned(500, 10), v => to_unsigned(520, 10)), -- 10: very light gray
    (y => to_unsigned(380, 10), u => to_unsigned(435, 10), v => to_unsigned(575, 10)), -- 11: orange-brown
    (y => to_unsigned(430, 10), u => to_unsigned(490, 10), v => to_unsigned(540, 10)), -- 12: beige
    (y => to_unsigned(350, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10)), -- 13: dark gray alt
    (y => to_unsigned(280, 10), u => to_unsigned(512, 10), v => to_unsigned(512, 10))  -- 14: very dark gray
  );

  -- Combined floor/ceiling color (neutral gray)
  constant C_FLOOR_CEIL_Y : unsigned(9 downto 0) := to_unsigned(300, 10);
  constant C_FLOOR_CEIL_U : unsigned(9 downto 0) := to_unsigned(512, 10);
  constant C_FLOOR_CEIL_V : unsigned(9 downto 0) := to_unsigned(512, 10);

  -- Wall height lookup: distance in cells (1-16) maps to pixel height for 720p
  type t_height_lut is array(1 to 16) of unsigned(9 downto 0);
  constant C_HEIGHT_LUT_HD : t_height_lut := (
    to_unsigned(360, 10), to_unsigned(180, 10), to_unsigned(120, 10), to_unsigned(90, 10),
    to_unsigned(72, 10), to_unsigned(60, 10), to_unsigned(51, 10), to_unsigned(45, 10),
    to_unsigned(40, 10), to_unsigned(36, 10), to_unsigned(32, 10), to_unsigned(30, 10),
    to_unsigned(27, 10), to_unsigned(25, 10), to_unsigned(24, 10), to_unsigned(22, 10)
  );

  -- Reciprocal LUT: maps |ray_dir| >> 2 to delta_dist (reduced to 128 entries)
  type t_recip_lut is array(0 to 127) of unsigned(15 downto 0);
  signal s_recip_lut : t_recip_lut := (others => (others => '0'));

  -- 16x16 map: 0=air, 1-15=wall types
  type t_map_cell is array(0 to 15, 0 to 15) of unsigned(3 downto 0);
  constant C_MAP : t_map_cell := (
    ( x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"3", x"3", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"3", x"3", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" ),
    ( x"0", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"0" ),
    ( x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0" )
  );

  -- DDA FSM state enumeration
  type dda_state_t is (
    DDA_IDLE, DDA_INIT, DDA_RAY_SETUP, DDA_LUT_REG,
    DDA_RECIP_X_W1, DDA_RECIP_X_W2, DDA_RECIP_Y_W1, DDA_RECIP_Y_W2,
    DDA_DIST_INIT, DDA_STEP, DDA_MAP_R, DDA_MAP_W1, DDA_MAP_W2, DDA_MAP_CHECK,
    DDA_HEIGHT_DIST, DDA_HEIGHT_CALC, DDA_COL_WRITE, DDA_COL_NEXT, DDA_DONE
  );

  -- ==========================================================================
  -- Signal Declarations - Optimized widths
  -- ==========================================================================

  -- Timing and sync tracking
  signal s_pixel_x      : unsigned(11 downto 0) := (others => '0');
  signal s_pixel_y      : unsigned(11 downto 0) := (others => '0');
  signal s_prev_hsync_n : std_logic := '1';
  signal s_prev_vsync_n : std_logic := '1';
  signal s_line_width   : unsigned(11 downto 0) := to_unsigned(1280, 12);
  signal s_frame_height : unsigned(11 downto 0) := to_unsigned(720, 12);
  signal s_v_center     : unsigned(11 downto 0);
  signal s_h_center     : unsigned(11 downto 0);

  -- DDA engine (8-bit angle, 14-bit position for 10.4 fixed-point)
  signal s_dda_state    : dda_state_t := DDA_IDLE;
  signal s_col_idx      : unsigned(6 downto 0) := (others => '0');  -- 0-79
  signal s_map_x, s_map_y : unsigned(3 downto 0) := (others => '0');  -- 0-15
  signal s_step_x, s_step_y : signed(1 downto 0) := (others => '0');
  signal s_side_dist_x, s_side_dist_y : unsigned(23 downto 0) := (others => '0');
  signal s_delta_dist_x, s_delta_dist_y : unsigned(15 downto 0) := (others => '0');
  signal s_dda_side    : std_logic := '0';
  signal s_dda_steps   : unsigned(4 downto 0) := (others => '0');  -- 0-15
  signal s_dda_hit     : std_logic := '0';
  signal s_dda_wall_type : unsigned(3 downto 0) := (others => '0');
  signal s_perp_dist   : unsigned(23 downto 0) := (others => '0');
  signal s_wall_half_height : unsigned(9 downto 0) := (others => '0');
  signal s_player_pos_x, s_player_pos_y : unsigned(13 downto 0) := (others => '0');  -- 14-bit (10.4)
  signal s_player_angle : unsigned(7 downto 0) := (others => '0');   -- 8-bit angle
  signal s_angle_acc    : unsigned(8 downto 0) := (others => '0');   -- 9-bit accumulator
  signal s_ray_angle    : unsigned(7 downto 0) := (others => '0');   -- 8-bit for LUT
  signal s_ray_dir_x, s_ray_dir_y : signed(9 downto 0) := (others => '0');
  signal s_frame_parity : std_logic := '0';
  signal s_dda_done    : std_logic;

  -- Column buffer BRAM (single bank to save resources)
  type t_cbuf_entry is record
    wall_half_h : unsigned(9 downto 0);
    dda_side    : std_logic;
    wall_type   : unsigned(3 downto 0);
    hit         : std_logic;
  end record;
  type t_cbuf is array(0 to 127) of t_cbuf_entry;  -- 128 entries for 80 columns
  signal s_cbuf : t_cbuf := (others => (
    wall_half_h => (others => '0'),
    dda_side => '0',
    wall_type => (others => '0'),
    hit => '0'
  ));
  signal s_cbuf_rdaddr : unsigned(6 downto 0) := (others => '0');
  signal s_cbuf_wraddr : unsigned(6 downto 0) := (others => '0');
  signal s_cbuf_wren   : std_logic := '0';
  signal s_cbuf_wrdata : t_cbuf_entry;
  signal s_cbuf_rddata : t_cbuf_entry;

  -- sin/cos LUT signals
  signal s_lut_sin_out, s_lut_cos_out : signed(9 downto 0);

  -- Renderer pipeline stages (4 stages now)
  signal s_vcol          : unsigned(6 downto 0);
  signal s_y_rel_d2      : signed(11 downto 0);
  signal s_wall_half_d3  : unsigned(9 downto 0);
  signal s_wall_type_d3  : unsigned(3 downto 0);
  signal s_wall_hit_d3   : std_logic;
  signal s_wall_top_d3   : signed(11 downto 0);
  signal s_wall_bot_d3   : signed(11 downto 0);
  signal s_out_y_d4, s_out_u_d4, s_out_v_d4 : unsigned(9 downto 0);

  -- Sync/control shift registers
  type t_sr_yuv  is array(0 to C_TOTAL_LATENCY - 1) of work.video_stream_pkg.t_video_stream_yuv444_30b;
  signal s_input_sr  : t_sr_yuv := (others => (
    y => (others => '0'), u => (others => '0'), v => (others => '0'),
    avid => '0', hsync_n => '1', vsync_n => '1', field_n => '0'
  ));

begin

  -- ==========================================================================
  -- sin_cos LUT Instantiation
  -- ==========================================================================
  sin_cos_inst : entity work.sin_cos_full_lut_10x10
    port map(
      angle_in  => std_ulogic_vector(resize(s_ray_angle, 10)),
      sin_out   => s_lut_sin_out,
      cos_out   => s_lut_cos_out
    );

  -- ==========================================================================
  -- P_TIMING: Pixel/line counting, sync edge detection, resolution measurement
  -- ==========================================================================
  p_timing : process(clk)
  begin
    if rising_edge(clk) then
      s_prev_hsync_n <= data_in.hsync_n;
      s_prev_vsync_n <= data_in.vsync_n;

      if data_in.avid = '1' then
        s_pixel_x <= s_pixel_x + 1;
      end if;

      if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
        if s_pixel_x > 0 then
          s_line_width <= s_pixel_x;
        end if;
        s_pixel_x <= (others => '0');
        s_pixel_y <= s_pixel_y + 1;
      end if;

      if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
        if s_pixel_y > 0 then
          s_frame_height <= s_pixel_y;
        end if;
        s_pixel_y <= (others => '0');
        s_pixel_x <= (others => '0');
        s_v_center <= resize(s_frame_height, 12) srl 1;
        s_h_center <= resize(s_line_width, 12) srl 1;
      end if;

      -- Sync pipeline
      s_input_sr(0) <= data_in;
      for i in 1 to C_TOTAL_LATENCY - 1 loop
        s_input_sr(i) <= s_input_sr(i - 1);
      end loop;
    end if;
  end process p_timing;

  -- ==========================================================================
  -- P_DDA_FSM: DDA raycaster state machine (optimized for 80 columns)
  -- ==========================================================================
  p_dda_fsm : process(clk)
    variable v_pos_frac_x, v_pos_frac_y : unsigned(3 downto 0);  -- 10.4 uses 4 bits
    variable v_abs_dir_x, v_abs_dir_y : unsigned(9 downto 0);
    variable v_dist_cells : unsigned(7 downto 0);
    variable v_mult_result_x, v_mult_result_y : unsigned(39 downto 0);
  begin
    if rising_edge(clk) then
      s_cbuf_wren <= '0';
      s_cbuf_wraddr <= s_col_idx;

      case s_dda_state is
        when DDA_IDLE =>
          if data_in.vsync_n = '0' then
            s_dda_state <= DDA_INIT;
          end if;

        when DDA_INIT =>
          s_player_pos_x <= unsigned(registers_in(0)(8 downto 0)) & "00000";  -- 14-bit total
          s_player_pos_y <= unsigned(registers_in(1)(8 downto 0)) & "00000";
          s_player_angle <= unsigned(registers_in(2)(7 downto 0));  -- 8-bit angle
          s_col_idx <= (others => '0');
          s_angle_acc <= unsigned(registers_in(2)(7 downto 0)) & "0";
          s_dda_state <= DDA_RAY_SETUP;

        when DDA_RAY_SETUP =>
          s_ray_angle <= s_angle_acc(8 downto 1);
          s_angle_acc <= s_angle_acc + 320;  -- 256/80 ≈ 3.2 units per column
          s_dda_state <= DDA_LUT_REG;

        when DDA_LUT_REG =>
          s_ray_dir_x <= s_lut_cos_out;
          s_ray_dir_y <= s_lut_sin_out;
          s_dda_state <= DDA_RECIP_X_W1;

        when DDA_RECIP_X_W1 | DDA_RECIP_X_W2 =>
          if s_dda_state = DDA_RECIP_X_W1 then
            v_abs_dir_x := unsigned(abs(s_ray_dir_x));
            s_delta_dist_x <= s_recip_lut(to_integer(v_abs_dir_x(8 downto 2)));  -- >> 2 indexing
            s_dda_state <= DDA_RECIP_X_W2;
          else
            s_dda_state <= DDA_RECIP_Y_W1;
          end if;

        when DDA_RECIP_Y_W1 | DDA_RECIP_Y_W2 =>
          if s_dda_state = DDA_RECIP_Y_W1 then
            v_abs_dir_y := unsigned(abs(s_ray_dir_y));
            s_delta_dist_y <= s_recip_lut(to_integer(v_abs_dir_y(8 downto 2)));
            s_dda_state <= DDA_RECIP_Y_W2;
          else
            s_dda_state <= DDA_DIST_INIT;
          end if;

        when DDA_DIST_INIT =>
          if s_ray_dir_x > 0 then
            s_step_x <= to_signed(1, 2);
            v_pos_frac_x := "1111" - s_player_pos_x(3 downto 0);
          else
            s_step_x <= to_signed(-1, 2);
            v_pos_frac_x := s_player_pos_x(3 downto 0);
          end if;

          if s_ray_dir_y > 0 then
            s_step_y <= to_signed(1, 2);
            v_pos_frac_y := "1111" - s_player_pos_y(3 downto 0);
          else
            s_step_y <= to_signed(-1, 2);
            v_pos_frac_y := s_player_pos_y(3 downto 0);
          end if;

          -- Compute side distances: use fixed bit position instead of shift
          s_side_dist_x <= resize((unsigned(v_pos_frac_x) * s_delta_dist_x(15 downto 0)) & "0000", 24);
          s_side_dist_y <= resize((unsigned(v_pos_frac_y) * s_delta_dist_y(15 downto 0)) & "0000", 24);
          s_map_x <= s_player_pos_x(7 downto 4);  -- Extract cell indices (4 bits for 16x16 map)
          s_map_y <= s_player_pos_y(7 downto 4);
          s_dda_steps <= (others => '0');
          s_dda_state <= DDA_STEP;

        when DDA_STEP =>
          if s_side_dist_x < s_side_dist_y then
            s_map_x <= unsigned(signed(s_map_x) + s_step_x);
            s_side_dist_x <= s_side_dist_x + resize(s_delta_dist_x, 24);
            s_dda_side <= '0';
          else
            s_map_y <= unsigned(signed(s_map_y) + s_step_y);
            s_side_dist_y <= s_side_dist_y + resize(s_delta_dist_y, 24);
            s_dda_side <= '1';
          end if;
          s_dda_state <= DDA_MAP_R;

        when DDA_MAP_R =>
          s_dda_state <= DDA_MAP_W1;

        when DDA_MAP_W1 | DDA_MAP_W2 =>
          if s_dda_state = DDA_MAP_W1 then
            s_dda_state <= DDA_MAP_W2;
          else
            s_dda_wall_type <= C_MAP(to_integer(s_map_y), to_integer(s_map_x));
            s_dda_state <= DDA_MAP_CHECK;
          end if;

        when DDA_MAP_CHECK =>
          if s_dda_wall_type /= "0000" or s_dda_steps >= 15 then
            s_dda_hit <= (s_dda_wall_type(0) or s_dda_wall_type(1) or
                          s_dda_wall_type(2) or s_dda_wall_type(3));
            s_dda_state <= DDA_HEIGHT_DIST;
          else
            s_dda_steps <= s_dda_steps + 1;
            s_dda_state <= DDA_STEP;
          end if;

        when DDA_HEIGHT_DIST =>
          -- Compute perpendicular distance
          if s_dda_hit = '1' then
            if s_dda_side = '0' then
              s_perp_dist <= s_side_dist_x - resize(s_delta_dist_x, 24);
            else
              s_perp_dist <= s_side_dist_y - resize(s_delta_dist_y, 24);
            end if;
          else
            s_perp_dist <= (others => '0');
          end if;
          s_dda_state <= DDA_HEIGHT_CALC;

        when DDA_HEIGHT_CALC =>
          -- Read the newly-computed perpendicular distance and convert to wall height
          v_dist_cells := s_perp_dist(17 downto 10);  -- 8-bit slice for 8-bit variable
          if v_dist_cells = 0 or v_dist_cells >= 16 then
            s_wall_half_height <= (others => '0');
          else
            s_wall_half_height <= C_HEIGHT_LUT_HD(to_integer(v_dist_cells));
          end if;
          s_dda_state <= DDA_COL_WRITE;

        when DDA_COL_WRITE =>
          s_cbuf_wren <= '1';
          s_cbuf_wrdata.wall_half_h <= s_wall_half_height;
          s_cbuf_wrdata.dda_side <= s_dda_side;
          s_cbuf_wrdata.wall_type <= s_dda_wall_type;
          s_cbuf_wrdata.hit <= s_dda_hit;
          s_dda_state <= DDA_COL_NEXT;

        when DDA_COL_NEXT =>
          s_col_idx <= s_col_idx + 1;
          if s_col_idx < (C_NUM_COLS - 1) then
            s_dda_state <= DDA_RAY_SETUP;
          else
            s_dda_state <= DDA_DONE;
            s_frame_parity <= not s_frame_parity;
          end if;

        when DDA_DONE =>
          s_dda_done <= '1';
          if data_in.vsync_n = '1' then
            s_dda_state <= DDA_IDLE;
          end if;
      end case;
    end if;
  end process p_dda_fsm;

  -- ==========================================================================
  -- P_RENDERER: 4-stage pixel rendering pipeline (optimized)
  -- ==========================================================================
  p_renderer : process(clk)
    variable v_wall_top, v_wall_bot : signed(11 downto 0);
    variable v_region : unsigned(1 downto 0);
    variable v_base_y, v_base_u, v_base_v : unsigned(9 downto 0);
    variable v_final_y : unsigned(9 downto 0);
    variable v_fog_shift : unsigned(1 downto 0);  -- 2-bit for simpler fog
    variable v_vcol : unsigned(6 downto 0);
  begin
    if rising_edge(clk) then
      -- Stage T1: Register inputs
      if data_in.avid = '1' then
        -- Compute virtual column based on resolution
        -- 80 columns: HD (1280px) needs /16, SD (640px) needs /8
        if s_line_width >= 1000 then  -- HD resolution
          v_vcol := resize(s_pixel_x(11 downto 4), 7);  -- Divide by 16
        else  -- SD resolution
          v_vcol := resize(s_pixel_x(10 downto 3), 7);  -- Divide by 8
        end if;
        s_vcol <= v_vcol;
        s_cbuf_rdaddr <= v_vcol;  -- Use lower 7 bits for 128 column buffer entries
      end if;
      s_y_rel_d2 <= signed(resize(s_pixel_y, 12)) - signed(resize(s_v_center, 12));

      -- Stage T2-T3: BRAM pipeline implicit

      -- Stage T3: BRAM read complete, unpack
      s_wall_half_d3 <= s_cbuf_rddata.wall_half_h;
      s_wall_type_d3 <= s_cbuf_rddata.wall_type;
      s_wall_hit_d3 <= s_cbuf_rddata.hit;
      v_wall_top := signed(resize(s_v_center, 12)) - signed(resize(s_cbuf_rddata.wall_half_h, 12));
      v_wall_bot := signed(resize(s_v_center, 12)) + signed(resize(s_cbuf_rddata.wall_half_h, 12));
      s_wall_top_d3 <= v_wall_top;
      s_wall_bot_d3 <= v_wall_bot;

      -- Stage T4: Pixel classification, color lookup, simple fog
      if s_y_rel_d2 < s_wall_top_d3 then
        v_region := "00";  -- Ceiling
        v_base_y := C_FLOOR_CEIL_Y;
        v_base_u := C_FLOOR_CEIL_U;
        v_base_v := C_FLOOR_CEIL_V;
      elsif s_y_rel_d2 >= s_wall_bot_d3 then
        v_region := "10";  -- Floor
        v_base_y := C_FLOOR_CEIL_Y;
        v_base_u := C_FLOOR_CEIL_U;
        v_base_v := C_FLOOR_CEIL_V;
      else
        v_region := "01";  -- Wall
        if s_wall_type_d3 = "0000" then
          v_base_y := (others => '0');
          v_base_u := (others => '0');
          v_base_v := (others => '0');
        else
          v_base_y := C_WALL_PALETTE(to_integer(s_wall_type_d3) - 1).y;
          v_base_u := C_WALL_PALETTE(to_integer(s_wall_type_d3) - 1).u;
          v_base_v := C_WALL_PALETTE(to_integer(s_wall_type_d3) - 1).v;
        end if;
      end if;

      -- Simple 2-band fog (use bits 8:7 for shift amount)
      v_fog_shift := s_wall_half_d3(8 downto 7);
      case v_fog_shift is
        when "00" => v_final_y := v_base_y;
        when "01" => v_final_y := v_base_y srl 1;
        when "10" => v_final_y := v_base_y srl 2;
        when others => v_final_y := v_base_y srl 3;
      end case;

      s_out_y_d4 <= v_final_y;
      s_out_u_d4 <= v_base_u;
      s_out_v_d4 <= v_base_v;

      -- Output assignment with bypass logic
      if registers_in(10)(0) = '1' then  -- Bypass ON: pass through input
        data_out.y <= std_logic_vector(s_input_sr(C_TOTAL_LATENCY - 1).y);
        data_out.u <= std_logic_vector(s_input_sr(C_TOTAL_LATENCY - 1).u);
        data_out.v <= std_logic_vector(s_input_sr(C_TOTAL_LATENCY - 1).v);
      else  -- Bypass OFF: output rendered scene
        data_out.y <= std_logic_vector(s_out_y_d4);
        data_out.u <= std_logic_vector(s_out_u_d4);
        data_out.v <= std_logic_vector(s_out_v_d4);
      end if;

      -- Output sync signals (latency-matched)
      data_out.hsync_n <= s_input_sr(C_TOTAL_LATENCY - 1).hsync_n;
      data_out.vsync_n <= s_input_sr(C_TOTAL_LATENCY - 1).vsync_n;
      data_out.field_n <= s_input_sr(C_TOTAL_LATENCY - 1).field_n;
      data_out.avid    <= s_input_sr(C_TOTAL_LATENCY - 1).avid;
    end if;
  end process p_renderer;

  -- ==========================================================================
  -- Reciprocal LUT initialization
  -- ==========================================================================
  init_recip_lut : for i in 0 to 127 generate
    s_recip_lut(i) <= to_unsigned(16384 / (i + 1), 16);
  end generate init_recip_lut;

  -- ==========================================================================
  -- Column buffer BRAM read/write (single bank)
  -- ==========================================================================
  cbuf_read : process(clk)
  begin
    if rising_edge(clk) then
      s_cbuf_rddata <= s_cbuf(to_integer(s_cbuf_rdaddr));
    end if;
  end process cbuf_read;

  cbuf_write : process(clk)
  begin
    if rising_edge(clk) then
      if s_cbuf_wren = '1' then
        s_cbuf(to_integer(s_cbuf_wraddr)) <= s_cbuf_wrdata;
      end if;
    end if;
  end process cbuf_write;

end architecture doom;
