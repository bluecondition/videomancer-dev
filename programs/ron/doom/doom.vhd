-- Videomancer Doom Raycaster Tech Demo (v2 rework)
-- LZX Industries Community FPGA Program
--
-- License: GPL-3.0
-- Author: Claude (Anthropic)
--
-- v2: consistent 8.8 fixed-point cell units end to end.
--   * position: 8.8 cells (knob << 2), cell = pos(11:8), frac = pos(7:0)
--   * delta_dist = 131072/|dir| (8.8 cells per axis step, dir amplitude 511)
--   * side_dist init = frac * delta >> 8 (same units as the per-step add)
--   * perp distance in 8.8 -> integer cells = perp(15:8)
--   * angle LUT driven with angle << 2 (full 360 degree coverage)
--   * 120 columns (1920/16 HD), toggles on registers_in(6) bits 0-4
--   * blanking-gated output (neutral Y=64 U=V=512 outside avid)

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
  -- Constants
  -- ==========================================================================

  constant C_TOTAL_LATENCY : integer := 8;    -- input SR depth (9 video regs incl. output)
  constant C_NUM_COLS      : integer := 120;  -- 1920/16
  constant C_MID           : unsigned(9 downto 0) := to_unsigned(512, 10);

  -- Wall color palette entry type
  type t_pal_entry is record
    y : unsigned(9 downto 0);
    u : unsigned(9 downto 0);
    v : unsigned(9 downto 0);
  end record;
  type t_palette is array(0 to 14) of t_pal_entry;

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

  constant C_FLOOR_CEIL_Y : unsigned(9 downto 0) := to_unsigned(300, 10);

  -- Wall half-height per integer cell distance.  HD: 1080p active (half 540).
  -- SD: interlaced field (~243 active lines, half ~120).
  type t_height_lut is array(1 to 16) of unsigned(9 downto 0);
  constant C_HEIGHT_HD : t_height_lut := (
    to_unsigned(540, 10), to_unsigned(270, 10), to_unsigned(180, 10), to_unsigned(135, 10),
    to_unsigned(108, 10), to_unsigned(90, 10),  to_unsigned(77, 10),  to_unsigned(67, 10),
    to_unsigned(60, 10),  to_unsigned(54, 10),  to_unsigned(49, 10),  to_unsigned(45, 10),
    to_unsigned(41, 10),  to_unsigned(38, 10),  to_unsigned(36, 10),  to_unsigned(33, 10)
  );
  constant C_HEIGHT_SD : t_height_lut := (
    to_unsigned(120, 10), to_unsigned(60, 10), to_unsigned(40, 10), to_unsigned(30, 10),
    to_unsigned(24, 10),  to_unsigned(20, 10), to_unsigned(17, 10), to_unsigned(15, 10),
    to_unsigned(13, 10),  to_unsigned(12, 10), to_unsigned(11, 10), to_unsigned(10, 10),
    to_unsigned(9, 10),   to_unsigned(8, 10),  to_unsigned(8, 10),  to_unsigned(7, 10)
  );

  -- Reciprocal LUT: delta_dist = 131072/|dir| in 8.8 cells, indexed by |dir|>>2.
  type t_recip_lut is array(0 to 127) of unsigned(15 downto 0);
  function f_recip_init return t_recip_lut is
    variable r : t_recip_lut;
    variable v : integer;
  begin
    for i in 0 to 127 loop
      v := 131072 / (4 * i + 2);
      if v > 65535 then
        v := 65535;
      end if;
      r(i) := to_unsigned(v, 16);
    end loop;
    return r;
  end function;
  constant C_RECIP_LUT : t_recip_lut := f_recip_init;

  -- one shifted-and-gated term of the mix multiply (m bit constant per frame)
  function f_mterm(d : signed(10 downto 0); en : std_logic; sh : natural) return signed is
  begin
    if en = '1' then
      return shift_left(resize(d, 15), sh);
    else
      return to_signed(0, 15);
    end if;
  end function;

  -- 16x16 map: 0=air, 1-15=wall types.  Fully enclosed border.
  type t_map_cell is array(0 to 15, 0 to 15) of unsigned(3 downto 0);
  constant C_MAP : t_map_cell := (
    ( x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1" ),
    ( x"1", x"0", x"0", x"0", x"0", x"0", x"3", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"1" ),
    ( x"1", x"0", x"6", x"6", x"0", x"0", x"3", x"0", x"0", x"9", x"9", x"9", x"0", x"0", x"0", x"1" ),
    ( x"1", x"0", x"6", x"0", x"0", x"0", x"3", x"0", x"0", x"9", x"0", x"0", x"0", x"0", x"0", x"1" ),
    ( x"1", x"0", x"6", x"0", x"0", x"0", x"0", x"0", x"0", x"9", x"0", x"0", x"0", x"6", x"0", x"1" ),
    ( x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"1" ),
    ( x"1", x"0", x"0", x"0", x"0", x"B", x"B", x"0", x"0", x"0", x"0", x"3", x"0", x"0", x"0", x"1" ),
    ( x"1", x"3", x"3", x"0", x"0", x"B", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"1" ),
    ( x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"6", x"0", x"0", x"0", x"0", x"9", x"0", x"1" ),
    ( x"1", x"0", x"0", x"9", x"0", x"0", x"0", x"0", x"6", x"0", x"0", x"0", x"0", x"0", x"0", x"1" ),
    ( x"1", x"0", x"0", x"9", x"0", x"0", x"0", x"0", x"6", x"6", x"0", x"0", x"0", x"0", x"0", x"1" ),
    ( x"1", x"0", x"0", x"0", x"0", x"3", x"0", x"0", x"0", x"0", x"0", x"0", x"B", x"0", x"0", x"1" ),
    ( x"1", x"0", x"6", x"0", x"0", x"3", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"1" ),
    ( x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"9", x"9", x"0", x"0", x"0", x"3", x"0", x"1" ),
    ( x"1", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"0", x"1" ),
    ( x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1", x"1" )
  );
  -- Separate copy for the pixel-rate HUD read (GHDL chokes on parallel reads
  -- of one array-of-arrays constant).
  constant C_MAP_HUD : t_map_cell := C_MAP;

  type dda_state_t is (
    DDA_IDLE, DDA_INIT, DDA_RAY_SETUP, DDA_LUT_REG, DDA_RECIP_X, DDA_RECIP_Y,
    DDA_PREP, DDA_MXL, DDA_MXW1, DDA_MXW2, DDA_MXG, DDA_MYW1, DDA_MYW2, DDA_MYG,
    DDA_STEP, DDA_CHECK, DDA_HEIGHT_DIST, DDA_HEIGHT_CALC, DDA_COL_WRITE, DDA_COL_NEXT,
    DDA_DONE
  );

  -- ==========================================================================
  -- Signals
  -- ==========================================================================

  -- Timing / raster measurement
  signal s_pixel_x      : unsigned(11 downto 0) := (others => '0');
  signal s_pixel_y      : unsigned(11 downto 0) := (others => '0');
  signal s_prev_hsync_n : std_logic := '1';
  signal s_prev_vsync_n : std_logic := '1';
  signal s_line_width   : unsigned(11 downto 0) := to_unsigned(1920, 12);
  signal s_frame_height : unsigned(11 downto 0) := to_unsigned(1125, 12);
  signal s_v_center     : unsigned(11 downto 0) := to_unsigned(562, 12);
  signal s_h_center     : unsigned(11 downto 0) := to_unsigned(960, 12);
  signal s_is_hd        : std_logic := '1';

  -- Control registers (latched per frame)
  signal s_k6_tint   : unsigned(2 downto 0) := (others => '0');
  signal s_t_hud     : std_logic := '0';
  signal s_t_shade   : std_logic := '0';
  signal s_t_fog     : std_logic := '0';
  signal s_t_cross   : std_logic := '0';
  signal s_t_bypass  : std_logic := '0';
  signal s_mix       : unsigned(3 downto 0) := to_unsigned(8, 4);  -- 0..8

  -- DDA engine (8.8 fixed-point cell units)
  signal s_dda_state  : dda_state_t := DDA_IDLE;
  signal s_dda_armed  : std_logic := '0';
  signal s_col_idx    : unsigned(6 downto 0) := (others => '0');
  signal s_map_x, s_map_y : unsigned(3 downto 0) := (others => '0');
  signal s_step_x, s_step_y : signed(1 downto 0) := (others => '0');
  signal s_frac_x, s_frac_y : unsigned(8 downto 0) := (others => '0');
  signal s_side_dist_x, s_side_dist_y : unsigned(23 downto 0) := (others => '0');
  signal s_delta_dist_x, s_delta_dist_y : unsigned(15 downto 0) := (others => '0');
  signal s_dda_side   : std_logic := '0';
  signal s_dda_steps  : unsigned(4 downto 0) := (others => '0');
  signal s_dda_hit    : std_logic := '0';
  signal s_dda_wall_type : unsigned(3 downto 0) := (others => '0');
  signal s_perp_dist  : unsigned(23 downto 0) := (others => '0');
  signal s_wall_half_height : unsigned(9 downto 0) := (others => '0');
  signal s_player_pos_x, s_player_pos_y : unsigned(11 downto 0) := (others => '0');  -- 4.8
  signal s_angle_acc  : unsigned(10 downto 0) := (others => '0');  -- half-LUT units, wraps mod 2048
  signal s_ray_angle  : unsigned(9 downto 0) := (others => '0');   -- LUT units
  signal s_ray_dir_x, s_ray_dir_y : signed(9 downto 0) := (others => '0');

  -- Shared split multiplier (operands at T, product readable T+3):
  -- two narrow partial products, summed in the second register stage
  signal s_ma  : unsigned(8 downto 0)  := (others => '0');
  signal s_mb  : unsigned(15 downto 0) := (others => '0');
  signal s_m1l : unsigned(20 downto 0) := (others => '0');
  signal s_m1h : unsigned(19 downto 0) := (others => '0');
  signal s_mp  : unsigned(24 downto 0) := (others => '0');

  -- sin/cos LUT
  signal s_lut_sin_out, s_lut_cos_out : signed(9 downto 0);

  -- Column buffer
  type t_cbuf_entry is record
    wall_half_h : unsigned(9 downto 0);
    dda_side    : std_logic;
    wall_type   : unsigned(3 downto 0);
    hit         : std_logic;
  end record;
  type t_cbuf is array(0 to 127) of t_cbuf_entry;
  signal s_cbuf : t_cbuf := (others => (
    wall_half_h => (others => '0'), dda_side => '0',
    wall_type => (others => '0'), hit => '0'));
  signal s_cbuf_rdaddr : unsigned(6 downto 0) := (others => '0');
  signal s_cbuf_wraddr : unsigned(6 downto 0) := (others => '0');
  signal s_cbuf_wren   : std_logic := '0';
  signal s_cbuf_wrdata : t_cbuf_entry := (
    wall_half_h => (others => '0'), dda_side => '0',
    wall_type => (others => '0'), hit => '0');
  signal s_cbuf_rddata : t_cbuf_entry := (
    wall_half_h => (others => '0'), dda_side => '0',
    wall_type => (others => '0'), hit => '0');

  -- Renderer pipeline
  signal s_y_rel_d1     : signed(11 downto 0) := (others => '0');
  signal s_y_rel_d2     : signed(11 downto 0) := (others => '0');
  signal s_iswall_d3    : std_logic := '0';
  signal s_side_d3      : std_logic := '0';
  signal s_pal_idx_d3   : unsigned(3 downto 0) := (others => '0');
  signal s_fog_d3       : unsigned(1 downto 0) := (others => '0');
  signal s_base_y_d4, s_base_u_d4, s_base_v_d4 : unsigned(9 downto 0) := (others => '0');
  signal s_iswall_d4    : std_logic := '0';
  signal s_fog_d4       : unsigned(1 downto 0) := (others => '0');
  signal s_wet_y_d5, s_wet_u_d5, s_wet_v_d5 : unsigned(9 downto 0) := (others => '0');
  signal s_ovl_y_d6, s_ovl_u_d6, s_ovl_v_d6 : unsigned(9 downto 0) := (others => '0');
  signal s_dif_y_d6, s_dif_u_d6, s_dif_v_d6 : signed(10 downto 0) := (others => '0');
  -- mix partial products (m is frame-constant 0..8, decomposed per bit)
  signal s_pa_y_d7, s_pa_u_d7, s_pa_v_d7 : signed(14 downto 0) := (others => '0');
  signal s_pb_y_d7, s_pb_u_d7, s_pb_v_d7 : signed(14 downto 0) := (others => '0');
  signal s_mix_y_d8, s_mix_u_d8, s_mix_v_d8 : unsigned(9 downto 0) := (others => '0');

  -- per-frame derived constants (latched in DDA_INIT)
  signal s_fog_bias   : unsigned(1 downto 0) := "10";
  signal s_bright_off : signed(9 downto 0) := (others => '0');

  -- Overlay flag pipes (index = stage the value was registered at)
  signal s_cross_p : std_logic_vector(1 to 5) := (others => '0');
  signal s_inmm_p  : std_logic_vector(1 to 5) := (others => '0');
  signal s_mm_x_d1, s_mm_y_d1 : unsigned(3 downto 0) := (others => '0');
  signal s_hudw_p  : std_logic_vector(2 to 5) := (others => '0');
  signal s_hudp_p  : std_logic_vector(2 to 5) := (others => '0');

  -- Sync/control shift register
  type t_sr_yuv is array(0 to C_TOTAL_LATENCY - 1) of work.video_stream_pkg.t_video_stream_yuv444_30b;
  signal s_input_sr : t_sr_yuv := (others => (
    y => (others => '0'), u => (others => '0'), v => (others => '0'),
    avid => '0', hsync_n => '1', vsync_n => '1', field_n => '0'
  ));

begin

  -- ==========================================================================
  -- sin/cos LUT: 8-bit game angle << 2 covers the full 1024-entry table
  -- ==========================================================================
  sin_cos_inst : entity work.sin_cos_full_lut_10x10
    port map(
      angle_in => std_logic_vector(s_ray_angle),
      sin_out  => s_lut_sin_out,
      cos_out  => s_lut_cos_out
    );

  -- ==========================================================================
  -- P_TIMING: pixel/line counting, raster measurement, sync pipeline
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
        s_v_center <= '0' & s_frame_height(11 downto 1);
        s_h_center <= '0' & s_line_width(11 downto 1);
        if s_line_width >= 1000 then
          s_is_hd <= '1';
        else
          s_is_hd <= '0';
        end if;
      end if;

      s_input_sr(0) <= data_in;
      for i in 1 to C_TOTAL_LATENCY - 1 loop
        s_input_sr(i) <= s_input_sr(i - 1);
      end loop;
    end if;
  end process p_timing;

  -- ==========================================================================
  -- Shared 2-stage multiplier (9x16)
  -- ==========================================================================
  p_mult : process(clk)
  begin
    if rising_edge(clk) then
      s_m1l <= s_ma(4 downto 0) * s_mb;
      s_m1h <= s_ma(8 downto 5) * s_mb;
      s_mp  <= resize(s_m1l, 25) + shift_left(resize(s_m1h, 25), 5);
    end if;
  end process p_mult;

  -- ==========================================================================
  -- P_DDA_FSM: per-field 120-column raycast (runs in vblank, ~6k cycles)
  -- ==========================================================================
  p_dda_fsm : process(clk)
    variable v_abs_dir  : unsigned(9 downto 0);
    variable v_dist     : unsigned(7 downto 0);
    variable v_wall     : unsigned(3 downto 0);
  begin
    if rising_edge(clk) then
      s_cbuf_wren <= '0';

      -- serrated-vsync guard: arm once per field, mid-frame
      if s_pixel_y = 200 then
        s_dda_armed <= '1';
      end if;

      case s_dda_state is
        when DDA_IDLE =>
          if data_in.vsync_n = '0' and s_prev_vsync_n = '1' and s_dda_armed = '1' then
            s_dda_state <= DDA_INIT;
          end if;

        when DDA_INIT =>
          s_dda_armed <= '0';
          -- knobs 0..1023 -> 0..16 cells in 4.8 fixed point
          s_player_pos_x <= unsigned(registers_in(0)(9 downto 0)) & "00";
          s_player_pos_y <= unsigned(registers_in(1)(9 downto 0)) & "00";
          -- angle knob -> 8-bit angle; acc in half-LUT units, centered on
          -- the view direction minus half the ~63 degree FOV (180 half-units)
          s_angle_acc <= (unsigned(registers_in(2)(9 downto 2)) & "000") - 180;
          -- frame-latched pixel controls
          s_fog_bias  <= unsigned(registers_in(4)(9 downto 8));
          s_bright_off <= signed(resize(unsigned(registers_in(3)(9 downto 1)), 10)) - 256;
          s_k6_tint   <= unsigned(registers_in(5)(9 downto 7));
          s_t_hud     <= registers_in(6)(0);
          s_t_shade   <= registers_in(6)(1);
          s_t_fog     <= registers_in(6)(2);
          s_t_cross   <= registers_in(6)(3);
          s_t_bypass  <= registers_in(6)(4);
          if registers_in(7)(6) = '1' then
            s_mix <= resize(unsigned(registers_in(7)(9 downto 7)), 4) + 1;
          else
            s_mix <= resize(unsigned(registers_in(7)(9 downto 7)), 4);
          end if;
          s_col_idx   <= (others => '0');
          s_dda_state <= DDA_RAY_SETUP;

        when DDA_RAY_SETUP =>
          s_ray_angle <= s_angle_acc(10 downto 1);
          s_angle_acc <= s_angle_acc + 3;  -- 3 half-units/col * 120 = 63 deg FOV
          s_dda_state <= DDA_LUT_REG;

        when DDA_LUT_REG =>
          s_ray_dir_x <= s_lut_cos_out;
          s_ray_dir_y <= s_lut_sin_out;
          s_dda_state <= DDA_RECIP_X;

        when DDA_RECIP_X =>
          v_abs_dir := unsigned(abs(s_ray_dir_x));
          s_delta_dist_x <= C_RECIP_LUT(to_integer(v_abs_dir(8 downto 2)));
          s_dda_state <= DDA_RECIP_Y;

        when DDA_RECIP_Y =>
          v_abs_dir := unsigned(abs(s_ray_dir_y));
          s_delta_dist_y <= C_RECIP_LUT(to_integer(v_abs_dir(8 downto 2)));
          s_dda_state <= DDA_PREP;

        when DDA_PREP =>
          -- distance to the first boundary on each axis, in cell/256 units
          if s_ray_dir_x > 0 then
            s_step_x <= to_signed(1, 2);
            s_frac_x <= to_unsigned(256, 9) - resize(s_player_pos_x(7 downto 0), 9);
          else
            s_step_x <= to_signed(-1, 2);
            s_frac_x <= resize(s_player_pos_x(7 downto 0), 9);
          end if;
          if s_ray_dir_y > 0 then
            s_step_y <= to_signed(1, 2);
            s_frac_y <= to_unsigned(256, 9) - resize(s_player_pos_y(7 downto 0), 9);
          else
            s_step_y <= to_signed(-1, 2);
            s_frac_y <= resize(s_player_pos_y(7 downto 0), 9);
          end if;
          s_dda_state <= DDA_MXL;

        when DDA_MXL =>
          s_ma <= s_frac_x;
          s_mb <= s_delta_dist_x;
          s_dda_state <= DDA_MXW1;

        when DDA_MXW1 =>
          s_dda_state <= DDA_MXW2;

        when DDA_MXW2 =>
          s_dda_state <= DDA_MXG;

        when DDA_MXG =>
          -- side_dist_x = frac_x * delta_x / 256, same 8.8 units as delta
          s_side_dist_x <= resize(shift_right(s_mp, 8), 24);
          s_ma <= s_frac_y;
          s_mb <= s_delta_dist_y;
          s_dda_state <= DDA_MYW1;

        when DDA_MYW1 =>
          s_dda_state <= DDA_MYW2;

        when DDA_MYW2 =>
          s_dda_state <= DDA_MYG;

        when DDA_MYG =>
          s_side_dist_y <= resize(shift_right(s_mp, 8), 24);
          s_map_x <= s_player_pos_x(11 downto 8);
          s_map_y <= s_player_pos_y(11 downto 8);
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
          s_dda_state <= DDA_CHECK;

        when DDA_CHECK =>
          v_wall := C_MAP(to_integer(s_map_y), to_integer(s_map_x));
          if v_wall /= "0000" then
            s_dda_wall_type <= v_wall;
            s_dda_hit <= '1';
            s_dda_state <= DDA_HEIGHT_DIST;
          elsif s_dda_steps >= 24 then
            s_dda_wall_type <= (others => '0');
            s_dda_hit <= '0';
            s_dda_state <= DDA_HEIGHT_DIST;
          else
            s_dda_steps <= s_dda_steps + 1;
            s_dda_state <= DDA_STEP;
          end if;

        when DDA_HEIGHT_DIST =>
          -- perpendicular distance (Lodev): side_dist already stepped past the
          -- hit boundary, so back off one delta on the axis that hit
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
          v_dist := s_perp_dist(15 downto 8);  -- integer cells
          if s_dda_hit = '0' then
            s_wall_half_height <= (others => '0');
          elsif v_dist = 0 then
            if s_is_hd = '1' then
              s_wall_half_height <= C_HEIGHT_HD(1);
            else
              s_wall_half_height <= C_HEIGHT_SD(1);
            end if;
          elsif v_dist >= 16 then
            if s_is_hd = '1' then
              s_wall_half_height <= C_HEIGHT_HD(16);
            else
              s_wall_half_height <= C_HEIGHT_SD(16);
            end if;
          else
            if s_is_hd = '1' then
              s_wall_half_height <= C_HEIGHT_HD(to_integer(v_dist));
            else
              s_wall_half_height <= C_HEIGHT_SD(to_integer(v_dist));
            end if;
          end if;
          s_dda_state <= DDA_COL_WRITE;

        when DDA_COL_WRITE =>
          s_cbuf_wren <= '1';
          s_cbuf_wraddr <= s_col_idx;
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
          end if;

        when DDA_DONE =>
          if data_in.vsync_n = '1' then
            s_dda_state <= DDA_IDLE;
          end if;
      end case;
    end if;
  end process p_dda_fsm;

  -- ==========================================================================
  -- P_RENDERER: 7-register pixel pipeline (S1..S6 + output reg)
  -- ==========================================================================
  p_renderer : process(clk)
    variable v_vcol      : unsigned(6 downto 0);
    variable v_xd        : signed(12 downto 0);
    variable v_yd        : signed(12 downto 0);
    variable v_cross     : std_logic;
    variable v_isw       : std_logic;
    variable v_inmm      : std_logic;
    variable v_mmx, v_mmy : unsigned(3 downto 0);
    variable v_idx       : unsigned(4 downto 0);
    variable v_base_y, v_base_u, v_base_v : unsigned(9 downto 0);
    variable v_by        : signed(11 downto 0);
    variable v_fog_raw   : unsigned(2 downto 0);
    variable v_fog_eff   : unsigned(2 downto 0);
    variable v_fe        : integer range 0 to 7;
    variable v_abs_yrel  : unsigned(11 downto 0);
    variable v_uc, v_vc  : signed(10 downto 0);
    variable v_sum       : signed(11 downto 0);
  begin
    if rising_edge(clk) then
      -- ---------------- S1: column address, y_rel, overlay geometry --------
      if s_is_hd = '1' then
        v_vcol := resize(s_pixel_x(10 downto 4), 7);  -- 1920/16 = 120 cols
      else
        v_vcol := resize(s_pixel_x(9 downto 3), 7);   -- 720/8 = 90 cols
      end if;
      s_cbuf_rdaddr <= v_vcol;
      s_y_rel_d1 <= signed(resize(s_pixel_y, 12)) - signed(resize(s_v_center, 12));

      -- crosshair: cross centered on (h_center, v_center)
      v_xd := signed(resize(s_pixel_x, 13)) - signed(resize(s_h_center, 13));
      v_yd := signed(resize(s_pixel_y, 13)) - signed(resize(s_v_center, 13));
      v_cross := '0';
      if (abs(v_xd) <= 2 and abs(v_yd) <= 16) or
         (abs(v_yd) <= 2 and abs(v_xd) <= 16) then
        v_cross := '1';
      end if;
      s_cross_p(1) <= v_cross;

      -- minimap: 16x16 cells at 8px (HD) / 4px (SD), origin (64, 64)
      v_inmm := '0';
      v_mmx := (others => '0');
      v_mmy := (others => '0');
      if s_is_hd = '1' then
        if s_pixel_x >= 64 and s_pixel_x < 192 and
           s_pixel_y >= 64 and s_pixel_y < 192 then
          v_inmm := '1';
          v_mmx := resize(shift_right(s_pixel_x - 64, 3), 4);
          v_mmy := resize(shift_right(s_pixel_y - 64, 3), 4);
        end if;
      else
        if s_pixel_x >= 64 and s_pixel_x < 128 and
           s_pixel_y >= 64 and s_pixel_y < 128 then
          v_inmm := '1';
          v_mmx := resize(shift_right(s_pixel_x - 64, 2), 4);
          v_mmy := resize(shift_right(s_pixel_y - 64, 2), 4);
        end if;
      end if;
      s_inmm_p(1) <= v_inmm;
      s_mm_x_d1 <= v_mmx;
      s_mm_y_d1 <= v_mmy;

      -- ---------------- S2: cbuf BRAM read happens in cbuf_read; HUD lookup -
      s_y_rel_d2 <= s_y_rel_d1;
      if C_MAP_HUD(to_integer(s_mm_y_d1), to_integer(s_mm_x_d1)) /= "0000" then
        s_hudw_p(2) <= '1';
      else
        s_hudw_p(2) <= '0';
      end if;
      if s_mm_x_d1 = s_player_pos_x(11 downto 8) and
         s_mm_y_d1 = s_player_pos_y(11 downto 8) then
        s_hudp_p(2) <= '1';
      else
        s_hudp_p(2) <= '0';
      end if;
      s_cross_p(2) <= s_cross_p(1);
      s_inmm_p(2)  <= s_inmm_p(1);

      -- ---------------- S3: classify, fog level, palette index -------------
      -- wall band: |y_rel| < wall_half (no v_center adds in this stage)
      v_abs_yrel := unsigned(abs(s_y_rel_d2));
      if s_cbuf_rddata.hit = '1' and v_abs_yrel < resize(s_cbuf_rddata.wall_half_h, 12) then
        v_isw := '1';
      else
        v_isw := '0';
      end if;
      s_iswall_d3 <= v_isw;
      s_side_d3   <= s_cbuf_rddata.dda_side;

      -- fog level: walls by apparent height, floor/ceiling by center distance
      if v_isw = '1' then
        if s_cbuf_rddata.wall_half_h >= 192 then
          v_fog_raw := "000";
        elsif s_cbuf_rddata.wall_half_h >= 96 then
          v_fog_raw := "001";
        elsif s_cbuf_rddata.wall_half_h >= 48 then
          v_fog_raw := "010";
        else
          v_fog_raw := "011";
        end if;
      else
        if v_abs_yrel >= 256 then
          v_fog_raw := "000";
        elsif v_abs_yrel >= 128 then
          v_fog_raw := "001";
        elsif v_abs_yrel >= 64 then
          v_fog_raw := "010";
        else
          v_fog_raw := "011";
        end if;
      end if;
      if s_t_fog = '1' then
        v_fog_eff := resize(v_fog_raw, 3) + resize(s_fog_bias, 3);
        if v_fog_eff >= 2 then
          v_fog_eff := v_fog_eff - 2;
        else
          v_fog_eff := "000";
        end if;
        if v_fog_eff > 3 then
          v_fog_eff := "011";
        end if;
      else
        v_fog_eff := "000";
      end if;
      s_fog_d3 <= v_fog_eff(1 downto 0);

      -- tinted palette index: (wall_type - 1 + tint) mod 15
      if s_cbuf_rddata.wall_type = "0000" then
        s_pal_idx_d3 <= (others => '0');
      else
        v_idx := resize(s_cbuf_rddata.wall_type, 5) - 1 + resize(s_k6_tint, 5);
        if v_idx >= 15 then
          v_idx := v_idx - 15;
        end if;
        s_pal_idx_d3 <= resize(v_idx, 4);
      end if;
      s_cross_p(3) <= s_cross_p(2);
      s_inmm_p(3)  <= s_inmm_p(2);
      s_hudw_p(3)  <= s_hudw_p(2);
      s_hudp_p(3)  <= s_hudp_p(2);

      -- ---------------- S4: base color select + side shading ---------------
      if s_iswall_d3 = '1' then
        v_base_y := C_WALL_PALETTE(to_integer(s_pal_idx_d3)).y;
        v_base_u := C_WALL_PALETTE(to_integer(s_pal_idx_d3)).u;
        v_base_v := C_WALL_PALETTE(to_integer(s_pal_idx_d3)).v;
        if s_t_shade = '1' and s_side_d3 = '1' then
          v_base_y := v_base_y - resize(v_base_y(9 downto 2), 10);
        end if;
      else
        v_base_y := C_FLOOR_CEIL_Y;
        v_base_u := C_MID;
        v_base_v := C_MID;
      end if;
      s_base_y_d4 <= v_base_y;
      s_base_u_d4 <= v_base_u;
      s_base_v_d4 <= v_base_v;
      s_iswall_d4 <= s_iswall_d3;
      s_fog_d4    <= s_fog_d3;
      s_cross_p(4) <= s_cross_p(3);
      s_inmm_p(4)  <= s_inmm_p(3);
      s_hudw_p(4)  <= s_hudw_p(3);
      s_hudp_p(4)  <= s_hudp_p(3);

      -- ---------------- S5: wall brightness + fog attenuation --------------
      v_base_y := s_base_y_d4;
      v_base_u := s_base_u_d4;
      v_base_v := s_base_v_d4;
      if s_iswall_d4 = '1' then
        v_by := signed(resize(v_base_y, 12)) + resize(s_bright_off, 12);
        if v_by < 0 then
          v_base_y := (others => '0');
        elsif v_by > 1023 then
          v_base_y := (others => '1');
        else
          v_base_y := unsigned(v_by(9 downto 0));
        end if;
      end if;
      v_fe := to_integer(s_fog_d4);
      s_wet_y_d5 <= shift_right(v_base_y, v_fe);
      v_uc := signed(resize(v_base_u, 11)) - 512;
      v_vc := signed(resize(v_base_v, 11)) - 512;
      v_uc := shift_right(v_uc, v_fe);
      v_vc := shift_right(v_vc, v_fe);
      s_wet_u_d5 <= unsigned(resize(v_uc + 512, 10));
      s_wet_v_d5 <= unsigned(resize(v_vc + 512, 10));
      s_cross_p(5) <= s_cross_p(4);
      s_inmm_p(5)  <= s_inmm_p(4);
      s_hudw_p(5)  <= s_hudw_p(4);
      s_hudp_p(5)  <= s_hudp_p(4);

      -- ---------------- S6: overlays onto wet, diff vs dry -----------------
      v_base_y := s_wet_y_d5;
      v_base_u := s_wet_u_d5;
      v_base_v := s_wet_v_d5;
      if s_t_hud = '1' and s_inmm_p(5) = '1' then
        if s_hudp_p(5) = '1' then
          v_base_y := to_unsigned(600, 10);   -- player marker: red
          v_base_u := to_unsigned(440, 10);
          v_base_v := to_unsigned(720, 10);
        elsif s_hudw_p(5) = '1' then
          v_base_y := to_unsigned(850, 10);   -- wall cell: near-white
          v_base_u := C_MID;
          v_base_v := C_MID;
        else
          v_base_y := to_unsigned(120, 10);   -- empty cell: dark
          v_base_u := C_MID;
          v_base_v := C_MID;
        end if;
      end if;
      if s_t_cross = '1' and s_cross_p(5) = '1' then
        v_base_y := to_unsigned(900, 10);
        v_base_u := C_MID;
        v_base_v := C_MID;
      end if;
      s_ovl_y_d6 <= v_base_y;
      s_ovl_u_d6 <= v_base_u;
      s_ovl_v_d6 <= v_base_v;
      s_dif_y_d6 <= signed(resize(v_base_y, 11)) - signed(resize(unsigned(s_input_sr(4).y), 11));
      s_dif_u_d6 <= signed(resize(v_base_u, 11)) - signed(resize(unsigned(s_input_sr(4).u), 11));
      s_dif_v_d6 <= signed(resize(v_base_v, 11)) - signed(resize(unsigned(s_input_sr(4).v), 11));

      -- ---------------- S7: mix partial products (m = 0..8, per-bit terms) --
      -- pa = (m3 ? d<<3 : 0) + (m2 ? d<<2 : 0); pb = (m1 ? d<<1 : 0) + (m0 ? d : 0)
      s_pa_y_d7 <= f_mterm(s_dif_y_d6, s_mix(3), 3) + f_mterm(s_dif_y_d6, s_mix(2), 2);
      s_pb_y_d7 <= f_mterm(s_dif_y_d6, s_mix(1), 1) + f_mterm(s_dif_y_d6, s_mix(0), 0);
      s_pa_u_d7 <= f_mterm(s_dif_u_d6, s_mix(3), 3) + f_mterm(s_dif_u_d6, s_mix(2), 2);
      s_pb_u_d7 <= f_mterm(s_dif_u_d6, s_mix(1), 1) + f_mterm(s_dif_u_d6, s_mix(0), 0);
      s_pa_v_d7 <= f_mterm(s_dif_v_d6, s_mix(3), 3) + f_mterm(s_dif_v_d6, s_mix(2), 2);
      s_pb_v_d7 <= f_mterm(s_dif_v_d6, s_mix(1), 1) + f_mterm(s_dif_v_d6, s_mix(0), 0);

      -- ---------------- S8: mix = dry + (pa + pb) / 8 -----------------------
      v_sum := signed(resize(unsigned(s_input_sr(6).y), 12)) + resize(shift_right(s_pa_y_d7 + s_pb_y_d7, 3), 12);
      if v_sum < 0 then
        s_mix_y_d8 <= (others => '0');
      else
        s_mix_y_d8 <= unsigned(v_sum(9 downto 0));
      end if;
      v_sum := signed(resize(unsigned(s_input_sr(6).u), 12)) + resize(shift_right(s_pa_u_d7 + s_pb_u_d7, 3), 12);
      if v_sum < 0 then
        s_mix_u_d8 <= (others => '0');
      else
        s_mix_u_d8 <= unsigned(v_sum(9 downto 0));
      end if;
      v_sum := signed(resize(unsigned(s_input_sr(6).v), 12)) + resize(shift_right(s_pa_v_d7 + s_pb_v_d7, 3), 12);
      if v_sum < 0 then
        s_mix_v_d8 <= (others => '0');
      else
        s_mix_v_d8 <= unsigned(v_sum(9 downto 0));
      end if;

      -- ---------------- S9: output register, bypass + blanking gate --------
      if s_t_bypass = '1' then
        data_out.y <= s_input_sr(C_TOTAL_LATENCY - 1).y;
        data_out.u <= s_input_sr(C_TOTAL_LATENCY - 1).u;
        data_out.v <= s_input_sr(C_TOTAL_LATENCY - 1).v;
      elsif s_input_sr(C_TOTAL_LATENCY - 1).avid = '0' then
        -- neutral blanking: free-running colors here corrupt the encoder's
        -- per-line colour reference
        data_out.y <= std_logic_vector(to_unsigned(64, 10));
        data_out.u <= std_logic_vector(C_MID);
        data_out.v <= std_logic_vector(C_MID);
      else
        data_out.y <= std_logic_vector(s_mix_y_d8);
        data_out.u <= std_logic_vector(s_mix_u_d8);
        data_out.v <= std_logic_vector(s_mix_v_d8);
      end if;

      data_out.hsync_n <= s_input_sr(C_TOTAL_LATENCY - 1).hsync_n;
      data_out.vsync_n <= s_input_sr(C_TOTAL_LATENCY - 1).vsync_n;
      data_out.field_n <= s_input_sr(C_TOTAL_LATENCY - 1).field_n;
      data_out.avid    <= s_input_sr(C_TOTAL_LATENCY - 1).avid;
    end if;
  end process p_renderer;

  -- ==========================================================================
  -- Column buffer read/write
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
