-- spiroscope.vhd
--
-- Spiroscope: a knob-driven phosphor drawing instrument -- an advanced
-- etch-a-sketch crossed with a spirograph.
--
-- A free-running "pen" plots into a 160x90 x 4-bit phosphor CANVAS held in
-- BRAM. The canvas fills the whole screen: each cell = 8x8 screen px, so a
-- 160x90 canvas maps to exactly 1280x720 (>>3 upscale, shift-only). Two pen
-- engines:
--   * SPIROGRAPH  : x = cx + R1*cos(a*t) + R2*cos(b*t+ph) + R3*cos(c*t)
--   * ETCH-A-SKETCH: two knobs drive pen X/Y velocity (classic).
-- Each frame a decay sweep fades every cell (phosphor persistence); the slider
-- sets the fade rate (0 = permanent etch lines, max = fast scope trails/erase).
--
-- Control map:
--   K1 (reg0) Freq A  / X-velocity      S7 (reg6.0) Mode  Spiro/Etch
--   K2 (reg1) Freq B  / Y-velocity      S8 (reg6.1) Harmonic (3rd term)
--   K3 (reg2) Arm     / Step size       S9 (reg6.2) Kaleidoscope (4-fold)
--   K4 (reg3) Phase   / Rotation        S10(reg6.3) Rainbow (hue by position)
--   K5 (reg4) Color (8 palette zones)   S11(reg6.4) Background Black/Video
--   K6 (reg5) Draw Speed                Slider(reg7) Persistence (hero)
--
-- BRAM port plan (single 1W1R EBR canvas, 256x90 x 4; 160 wide used):
--   * Active video : read port -> upscaled readout. write port -> pen plot.
--   * Vblank       : a decay sweep owns BOTH ports (read k, write k-1, fade).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture spiroscope of program_top is

    -- Full-screen canvas: 160x90 cells at 8x8 px = 1280x720 (>>3, shift-only).
    constant C_CW      : integer := 160;   -- canvas width  (cells)
    constant C_CH      : integer := 90;    -- canvas height (cells)
    constant C_USHIFT  : integer := 3;     -- 8 screen px per cell
    constant C_VW      : integer := 1280;  -- canvas viewport width  (px)
    constant C_VH      : integer := 720;   -- canvas viewport height (px)
    constant C_LATENCY : integer := 5;     -- input->output register stages
    constant C_CELLS   : integer := C_CW * C_CH;  -- 14400 (15 EBRs)

    -- canvas address = cy*160 + cx (160 = 128+32 -> two shifts + an add, no DSP).
    -- Computed in the arbiter stage (not p_addr) to keep it off the long
    -- subtract+upscale chain.
    function f_addr(cy : unsigned(6 downto 0); cx : unsigned(7 downto 0)) return unsigned is
        variable v : unsigned(14 downto 0);
    begin
        v := resize(cy, 15);
        return shift_left(v, 7) + shift_left(v, 5) + resize(cx, 15);
    end function;

    -- ==== Timing / pixel counters (c64 pattern) ====
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';
    signal s_x_count      : unsigned(11 downto 0) := (others => '0');
    signal s_y_count      : unsigned(11 downto 0) := (others => '0');
    signal s_line_width   : unsigned(11 downto 0) := to_unsigned(1280, 12);
    signal s_frame_height : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_line_active  : std_logic := '0';
    signal s_frame_cnt    : unsigned(7 downto 0) := (others => '0');

    -- viewport offsets (centre the 1280x720 canvas in the actual raster)
    signal s_x0 : unsigned(11 downto 0) := (others => '0');
    signal s_y0 : unsigned(11 downto 0) := (others => '0');
    -- figure centre = centre of the visible canvas
    signal s_cx_center : unsigned(7 downto 0) := to_unsigned(80, 8);
    signal s_cy_center : unsigned(6 downto 0) := to_unsigned(45, 7);

    -- ==== Canvas BRAM: 256x90 x 4-bit (160 wide used), addr = cy & cx (1W1R) ====
    type t_canvas is array(0 to C_CELLS - 1) of unsigned(3 downto 0);
    signal s_canvas   : t_canvas := (others => (others => '0'));
    signal s_cv_we    : std_logic := '0';
    signal s_cv_waddr : unsigned(14 downto 0) := (others => '0');
    signal s_cv_wdata : unsigned(3 downto 0)  := (others => '0');
    signal s_cv_raddr : unsigned(14 downto 0) := (others => '0');
    signal s_cv_rdata : unsigned(3 downto 0)  := (others => '0');

    -- ==== Readout cell coords (stage 2); address formed in the arbiter ====
    signal s_insq2 : std_logic := '0';
    signal s_cellx2 : unsigned(7 downto 0) := (others => '0');
    signal s_celly2 : unsigned(6 downto 0) := (others => '0');
    signal s_insq3, s_insq4 : std_logic := '0';
    signal s_cellx3 : unsigned(7 downto 0) := (others => '0');
    signal s_celly3 : unsigned(6 downto 0) := (others => '0');
    signal s_cellx4 : unsigned(7 downto 0) := (others => '0');
    signal s_celly4 : unsigned(6 downto 0) := (others => '0');
    signal s_idx : unsigned(2 downto 0) := (others => '0');  -- palette index (pre-muxed)

    -- ==== Decay sweep ====
    signal s_decay_active  : std_logic := '0';
    signal s_decay_started : std_logic := '0';
    signal s_decay_done    : std_logic := '0';
    signal s_decay_k       : unsigned(14 downto 0) := (others => '0');
    signal s_decay_k_d1    : unsigned(14 downto 0) := (others => '0');
    signal s_decay_K_amt   : unsigned(3 downto 0)  := (others => '0');

    -- ==== Pen write request (driven by the plotter FSM) ====
    signal s_pen_we    : std_logic := '0';
    signal s_pen_waddr : unsigned(14 downto 0) := (others => '0');
    signal s_pen_wdata : unsigned(3 downto 0)  := (others => '1');

    -- ==== Spirograph pen plotter ====
    signal s_phaseA  : unsigned(15 downto 0) := (others => '0');
    signal s_phaseB  : unsigned(15 downto 0) := (others => '0');
    signal s_phaseC  : unsigned(15 downto 0) := (others => '0');
    signal s_incA    : unsigned(4 downto 0)  := to_unsigned(3, 5);
    signal s_incB    : unsigned(4 downto 0)  := to_unsigned(5, 5);
    signal s_incC    : unsigned(5 downto 0)  := to_unsigned(8, 6);
    signal s_R1      : unsigned(6 downto 0)  := to_unsigned(28, 7);
    signal s_R2      : unsigned(6 downto 0)  := to_unsigned(16, 7);
    signal s_R3      : unsigned(6 downto 0)  := (others => '0');   -- 3rd harmonic (S8)
    signal s_phaseOff: unsigned(7 downto 0)  := (others => '0');
    signal s_budget  : unsigned(15 downto 0) := (others => '0');
    signal s_st      : integer range 0 to 8  := 0;

    signal s_sinA, s_cosA, s_sinB, s_cosB : signed(9 downto 0) := (others => '0');
    signal s_sinC, s_cosC : signed(9 downto 0) := (others => '0');
    signal s_accX, s_accY : signed(20 downto 0) := (others => '0');

    -- shared sin/cos LUT (combinational) + shared multiplier (registered)
    signal s_lut_angle : std_logic_vector(9 downto 0) := (others => '0');
    signal s_lut_sin   : signed(9 downto 0);
    signal s_lut_cos   : signed(9 downto 0);
    signal s_mul_a     : signed(7 downto 0)  := (others => '0');
    signal s_mul_b     : signed(9 downto 0)  := (others => '0');
    signal s_mul_p     : signed(17 downto 0) := (others => '0');

    -- ==== Per-frame latched control flags ====
    signal s_mode      : std_logic := '0';                  -- 0 spiro, 1 etch
    signal s_rainbow   : std_logic := '0';
    signal s_bg_video  : std_logic := '0';
    signal s_kaleido   : std_logic := '0';
    signal s_color_sel : unsigned(2 downto 0) := to_unsigned(3, 3);

    -- ==== Etch-a-sketch pen (Q?.4 signed; integrated once per line) ====
    signal s_eX : signed(15 downto 0) := to_signed(80 * 16, 16);
    signal s_eY : signed(15 downto 0) := to_signed(45 * 16, 16);
    signal s_evelX, s_evelY : signed(9 downto 0) := (others => '0');

    -- ==== Plot request (plotter/etch -> writer) ====
    signal s_plot_px : unsigned(7 downto 0) := (others => '0');
    signal s_plot_py : unsigned(6 downto 0) := (others => '0');
    signal s_plot_stb : std_logic := '0';

    -- ==== Writer FSM (emits 1 or 4 mirrored canvas writes per request) ====
    signal s_w_active : std_logic := '0';
    signal s_w_step   : integer range 0 to 3 := 0;
    signal s_w_px : unsigned(7 downto 0) := (others => '0');
    signal s_w_py : unsigned(6 downto 0) := (others => '0');

    -- ==== Pen colour palette (8 entries, U/V swapped for HW; 7 = white) ====
    type t_pal is array(0 to 7) of unsigned(9 downto 0);
    constant C_PAL_U : t_pal := (   -- = Cr
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned(608, 10), to_unsigned(756, 10), to_unsigned(512, 10));
    constant C_PAL_V : t_pal := (   -- = Cb
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned(696, 10), to_unsigned(852, 10), to_unsigned(512, 10));

    -- ==== Colour / output pipeline (stage 5) ====
    signal s_d_y   : unsigned(9 downto 0) := (others => '0');
    signal s_d_u   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_d_v   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_drawn : std_logic := '0';

    -- ==== sync / video delay SR ====
    type t_data_sr is array(0 to C_LATENCY - 1) of std_logic_vector(9 downto 0);
    type t_bit_sr  is array(0 to C_LATENCY - 1) of std_logic;
    signal s_y_sr, s_u_sr, s_v_sr : t_data_sr := (others => (others => '0'));
    signal s_hsync_sr, s_vsync_sr, s_field_sr, s_avid_sr : t_bit_sr := (others => '0');

begin

    -- ====================================================================
    -- stage 1: pixel/line counters + per-frame geometry
    -- ====================================================================
    p_count : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                s_x_count     <= s_x_count + 1;
                s_line_active <= '1';
            end if;

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_x_count > 0 then s_line_width <= s_x_count; end if;
                s_x_count <= (others => '0');
                if s_line_active = '1' then
                    s_y_count     <= s_y_count + 1;
                    s_line_active <= '0';
                end if;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_y_count > 0 then s_frame_height <= s_y_count; end if;
                s_y_count     <= (others => '0');
                s_x_count     <= (others => '0');
                s_line_active <= '0';
                s_frame_cnt   <= s_frame_cnt + 1;
                -- latch display control flags
                s_mode      <= registers_in(6)(0);
                s_kaleido   <= registers_in(6)(2);
                s_rainbow   <= registers_in(6)(3);
                s_bg_video  <= registers_in(6)(4);
                s_color_sel <= unsigned(registers_in(4)(9 downto 7));
                -- viewport centring (only matters when raster > 1280x720)
                if s_line_width > to_unsigned(C_VW, 12) then
                    s_x0 <= shift_right(s_line_width - to_unsigned(C_VW, 12), 1);
                else
                    s_x0 <= (others => '0');
                end if;
                if s_frame_height > to_unsigned(C_VH, 12) then
                    s_y0 <= shift_right(s_frame_height - to_unsigned(C_VH, 12), 1);
                else
                    s_y0 <= (others => '0');
                end if;
                -- figure centre = centre of the visible canvas
                if s_line_width >= to_unsigned(C_VW, 12) then
                    s_cx_center <= to_unsigned(80, 8);
                else
                    s_cx_center <= resize(shift_right(s_line_width, 4), 8);
                end if;
                if s_frame_height >= to_unsigned(C_VH, 12) then
                    s_cy_center <= to_unsigned(45, 7);
                else
                    s_cy_center <= resize(shift_right(s_frame_height, 4), 7);
                end if;
            end if;
        end if;
    end process p_count;

    -- ====================================================================
    -- stage 2: geometry -> canvas cell coords -> read address
    -- ====================================================================
    p_addr : process(clk)
        variable v_cx, v_cy   : unsigned(11 downto 0);
        variable v_ccx        : unsigned(7 downto 0);
        variable v_ccy        : unsigned(6 downto 0);
        variable v_in         : boolean;
    begin
        if rising_edge(clk) then
            v_in := (s_x_count >= s_x0) and (s_x_count < s_x0 + to_unsigned(C_VW, 12))
                and (s_y_count >= s_y0) and (s_y_count < s_y0 + to_unsigned(C_VH, 12));
            v_cx  := s_x_count - s_x0;
            v_cy  := s_y_count - s_y0;
            v_ccx := resize(shift_right(v_cx, C_USHIFT), 8);   -- 0..159
            v_ccy := resize(shift_right(v_cy, C_USHIFT), 7);   -- 0..89

            s_cellx2 <= v_ccx;
            s_celly2 <= v_ccy;
            if v_in then s_insq2 <= '1'; else s_insq2 <= '0'; end if;
        end if;
    end process p_addr;

    -- carry in-square / cell coords to align with the 1-cycle BRAM read; also
    -- precompute the palette index here so p_color only does an 8:1 mux (timing)
    p_pipe : process(clk)
        variable v_sum : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            s_insq3  <= s_insq2;   s_insq4  <= s_insq3;
            s_cellx3 <= s_cellx2;  s_cellx4 <= s_cellx3;
            s_celly3 <= s_celly2;  s_celly4 <= s_celly3;

            if s_rainbow = '1' then
                v_sum := resize(s_cellx3, 9) + resize(s_celly3, 9) + resize(s_frame_cnt, 9);
                s_idx <= v_sum(7 downto 5);
            else
                s_idx <= s_color_sel;
            end if;
        end if;
    end process p_pipe;

    -- ====================================================================
    -- stage 3: canvas port arbiter + vblank decay sweep
    -- ====================================================================
    p_canvas_ctrl : process(clk)
        variable v_sl     : unsigned(9 downto 0);
        variable v_pshift : integer range 0 to 7;
        variable v_mask   : unsigned(7 downto 0);
        variable v_en     : boolean;
        variable v_do     : boolean;
        variable v_K      : unsigned(3 downto 0);
        variable v_new    : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s_cv_we <= '0';

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                v_sl := unsigned(registers_in(7));
                if v_sl = 0 then
                    v_en := false; v_K := (others => '0'); v_pshift := 0;
                elsif v_sl < to_unsigned(512, 10) then
                    v_en := true; v_K := to_unsigned(1, 4);
                    v_pshift := to_integer(shift_right(to_unsigned(511, 10) - v_sl, 6));
                else
                    v_en := true; v_pshift := 0;
                    if v_sl > to_unsigned(1000, 10) then
                        v_K := to_unsigned(15, 4);
                    else
                        v_K := resize(shift_right(v_sl - to_unsigned(512, 10), 6), 4) + 1;
                    end if;
                end if;
                s_decay_K_amt <= v_K;

                v_do := false;
                if v_en then
                    v_mask := shift_left(to_unsigned(1, 8), v_pshift) - 1;
                    if (s_frame_cnt and v_mask) = to_unsigned(0, 8) then v_do := true; end if;
                end if;
                if v_do then
                    s_decay_active  <= '1';
                    s_decay_started <= '0';
                    s_decay_done    <= '0';
                    s_decay_k       <= (others => '0');
                end if;
            end if;

            if s_decay_active = '1' then
                s_cv_raddr   <= s_decay_k;
                s_decay_k_d1 <= s_decay_k;
                if s_decay_started = '1' then
                    if s_decay_K_amt = 0 then
                        v_new := s_cv_rdata;
                    elsif s_cv_rdata <= s_decay_K_amt then
                        v_new := (others => '0');
                    else
                        v_new := s_cv_rdata - s_decay_K_amt;
                    end if;
                    s_cv_we    <= '1';
                    s_cv_waddr <= s_decay_k_d1;
                    s_cv_wdata <= v_new;
                end if;
                s_decay_started <= '1';

                if s_decay_done = '1' then
                    s_decay_active <= '0';
                elsif s_decay_k = to_unsigned(C_CELLS - 1, 15) then
                    s_decay_done <= '1';
                else
                    s_decay_k <= s_decay_k + 1;
                end if;
            else
                s_cv_raddr <= f_addr(s_celly2, s_cellx2);   -- address formed here
                if s_pen_we = '1' then
                    s_cv_we    <= '1';
                    s_cv_waddr <= s_pen_waddr;
                    s_cv_wdata <= s_pen_wdata;
                end if;
            end if;
        end if;
    end process p_canvas_ctrl;

    -- ==== stage 4: canvas BRAM (1W1R, c64 p_buf pattern) ====
    p_canvas : process(clk)
    begin
        if rising_edge(clk) then
            if s_cv_we = '1' then
                s_canvas(to_integer(s_cv_waddr)) <= s_cv_wdata;
            end if;
            s_cv_rdata <= s_canvas(to_integer(s_cv_raddr));
        end if;
    end process p_canvas;

    -- ====================================================================
    -- Spirograph pen plotter: shared sin/cos LUT + shared multiplier + FSM
    -- ====================================================================
    lut : entity work.sin_cos_full_lut_10x10
        port map (angle_in => s_lut_angle, sin_out => s_lut_sin, cos_out => s_lut_cos);

    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            s_mul_p <= s_mul_a * s_mul_b;       -- signed(7:0) * signed(9:0)
        end if;
    end process p_mul;

    p_plot : process(clk)
        variable v_r2   : unsigned(5 downto 0);
        variable v_base : integer range 33 to 44;
        variable v_pxs  : signed(9 downto 0);
        variable v_pys  : signed(9 downto 0);
        variable v_px   : unsigned(7 downto 0);
        variable v_py   : unsigned(6 downto 0);
        variable v_ia   : unsigned(4 downto 0);
        variable v_ib   : unsigned(4 downto 0);
        variable v_nx   : signed(15 downto 0);
        variable v_ny   : signed(15 downto 0);
        variable v_accy : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            s_plot_stb <= '0';

            -- ---- per-frame control latch + budget reload ----
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                v_ia       := resize(unsigned(registers_in(0)(9 downto 6)), 5) + 1;  -- 1..16
                v_ib       := resize(unsigned(registers_in(1)(9 downto 6)), 5) + 1;  -- 1..16
                s_incA     <= v_ia;
                s_incB     <= v_ib;
                s_incC     <= resize(v_ia, 6) + resize(v_ib, 6);                     -- sum harmonic
                -- radius budget: R1+R2+R3 = 44 (fills the 90-cell height, ~round)
                if registers_in(6)(1) = '1' then
                    s_R3 <= to_unsigned(11, 7);  v_base := 33;
                else
                    s_R3 <= (others => '0');      v_base := 44;
                end if;
                v_r2 := unsigned(registers_in(2)(9 downto 4));                      -- 0..63
                if v_r2 > to_unsigned(v_base, 6) then v_r2 := to_unsigned(v_base, 6); end if;
                s_R2 <= resize(v_r2, 7);
                s_R1 <= to_unsigned(v_base, 7) - resize(v_r2, 7);
                s_phaseOff <= unsigned(registers_in(3)(9 downto 2));
                s_budget   <= to_unsigned(256, 16) + (resize(unsigned(registers_in(5)), 16) sll 5);
                -- etch pen velocity (Q?.4, fixed >>7 -> <=0.25 cell/line, continuous)
                s_evelX <= resize(shift_right(signed('0' & registers_in(0)) - to_signed(512, 11), 7), 10);
                s_evelY <= resize(shift_right(signed('0' & registers_in(1)) - to_signed(512, 11), 7), 10);
            end if;

            if s_mode = '0' then
                -- ============ SPIROGRAPH ============
                case s_st is
                    when 0 =>
                        -- only the top 8 angle bits drive the LUT (low 2 tied 0)
                        -- so synthesis prunes a quarter of the ROM mux -> faster
                        if s_budget > 0 and s_decay_active = '0' then
                            s_lut_angle <= std_logic_vector(s_phaseA(15 downto 8)) & "00";
                            s_st <= 1;
                        end if;
                    when 1 =>
                        s_sinA <= s_lut_sin;  s_cosA <= s_lut_cos;
                        s_lut_angle <= std_logic_vector(s_phaseB(15 downto 8) + s_phaseOff) & "00";
                        s_st <= 2;
                    when 2 =>
                        s_sinB <= s_lut_sin;  s_cosB <= s_lut_cos;
                        s_lut_angle <= std_logic_vector(s_phaseC(15 downto 8)) & "00";
                        s_mul_a <= signed('0' & s_R1);  s_mul_b <= s_cosA;     -- R1*cosA
                        s_st <= 3;
                    when 3 =>
                        s_accX  <= resize(s_mul_p, 21);                        -- R1*cosA
                        s_sinC <= s_lut_sin;  s_cosC <= s_lut_cos;
                        s_mul_a <= signed('0' & s_R2);  s_mul_b <= s_cosB;     -- R2*cosB
                        s_st <= 4;
                    when 4 =>
                        s_accX  <= s_accX + resize(s_mul_p, 21);               -- + R2*cosB
                        s_mul_a <= signed('0' & s_R3);  s_mul_b <= s_cosC;     -- R3*cosC
                        s_st <= 5;
                    when 5 =>
                        s_accX  <= s_accX + resize(s_mul_p, 21);               -- + R3*cosC
                        s_mul_a <= signed('0' & s_R1);  s_mul_b <= s_sinA;     -- R1*sinA
                        s_st <= 6;
                    when 6 =>
                        s_accY  <= resize(s_mul_p, 21);                        -- R1*sinA
                        s_mul_a <= signed('0' & s_R2);  s_mul_b <= s_sinB;     -- R2*sinB
                        s_st <= 7;
                    when 7 =>
                        s_accY  <= s_accY + resize(s_mul_p, 21);               -- + R2*sinB
                        s_mul_a <= signed('0' & s_R3);  s_mul_b <= s_sinC;     -- R3*sinC
                        s_st <= 8;
                    when 8 =>
                        v_accy := s_accY + resize(s_mul_p, 21);                -- + R3*sinC
                        v_pxs := resize(signed('0' & s_cx_center), 10) + resize(shift_right(s_accX, 9), 10);
                        v_pys := resize(signed('0' & s_cy_center), 10) + resize(shift_right(v_accy, 9), 10);
                        if    v_pxs < 0   then v_px := (others => '0');
                        elsif v_pxs > C_CW - 1 then v_px := to_unsigned(C_CW - 1, 8);
                        else  v_px := unsigned(v_pxs(7 downto 0)); end if;
                        if    v_pys < 0   then v_py := (others => '0');
                        elsif v_pys > C_CH - 1 then v_py := to_unsigned(C_CH - 1, 7);
                        else  v_py := unsigned(v_pys(6 downto 0)); end if;

                        s_plot_px  <= v_px;
                        s_plot_py  <= v_py;
                        s_plot_stb <= '1';

                        s_phaseA <= s_phaseA + resize(s_incA, 16);
                        s_phaseB <= s_phaseB + resize(s_incB, 16);
                        s_phaseC <= s_phaseC + resize(s_incC, 16);
                        if s_budget > 0 then s_budget <= s_budget - 1; end if;
                        s_st <= 0;
                end case;
            else
                -- ============ ETCH-A-SKETCH ============
                if data_in.hsync_n = '0' and s_prev_hsync_n = '1' and s_decay_active = '0' then
                    v_nx := s_eX + resize(s_evelX, 16);
                    v_ny := s_eY + resize(s_evelY, 16);
                    if v_nx < 0 then v_nx := (others => '0');
                    elsif v_nx > to_signed((C_CW - 1) * 16, 16) then v_nx := to_signed((C_CW - 1) * 16, 16); end if;
                    if v_ny < 0 then v_ny := (others => '0');
                    elsif v_ny > to_signed((C_CH - 1) * 16, 16) then v_ny := to_signed((C_CH - 1) * 16, 16); end if;
                    s_eX <= v_nx;
                    s_eY <= v_ny;
                    s_plot_px  <= unsigned(v_nx(11 downto 4));
                    s_plot_py  <= unsigned(v_ny(10 downto 4));
                    s_plot_stb <= '1';
                end if;
            end if;
        end if;
    end process p_plot;

    -- ====================================================================
    -- Pen writer: turns one plot request into 1 (or 4 mirrored) canvas writes
    -- ====================================================================
    p_writer : process(clk)
        variable v_px : unsigned(7 downto 0);
        variable v_py : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            s_pen_we <= '0';
            if s_w_active = '0' then
                if s_plot_stb = '1' then
                    s_w_px <= s_plot_px;
                    s_w_py <= s_plot_py;
                    s_w_step <= 0;
                    s_w_active <= '1';
                end if;
            else
                case s_w_step is
                    when 0 => v_px := s_w_px;                       v_py := s_w_py;
                    when 1 => v_px := to_unsigned(C_CW - 1, 8) - s_w_px; v_py := s_w_py;
                    when 2 => v_px := s_w_px;                       v_py := to_unsigned(C_CH - 1, 7) - s_w_py;
                    when others => v_px := to_unsigned(C_CW - 1, 8) - s_w_px; v_py := to_unsigned(C_CH - 1, 7) - s_w_py;
                end case;
                s_pen_we    <= '1';
                s_pen_wdata <= "1111";
                s_pen_waddr <= f_addr(v_py, v_px);

                if s_kaleido = '0' or s_w_step = 3 then
                    s_w_active <= '0';
                else
                    s_w_step <= s_w_step + 1;
                end if;
            end if;
        end if;
    end process p_writer;

    -- ====================================================================
    -- stage 5: brightness -> luma (phosphor glow); palette sets chroma
    -- ====================================================================
    p_color : process(clk)
        variable v_b   : unsigned(3 downto 0);
        variable v_idx : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_b   := s_cv_rdata;
            v_idx := to_integer(s_idx);
            if s_insq4 = '1' and v_b /= 0 then
                s_d_y   <= v_b & "000000";              -- b * 64 (0..960)
                s_d_u   <= C_PAL_U(v_idx);
                s_d_v   <= C_PAL_V(v_idx);
                s_drawn <= '1';
            else
                s_d_y   <= (others => '0');
                s_d_u   <= to_unsigned(512, 10);
                s_d_v   <= to_unsigned(512, 10);
                s_drawn <= '0';
            end if;
        end if;
    end process p_color;

    -- ====================================================================
    -- sync / video delay (match pipeline depth)
    -- ====================================================================
    p_sr : process(clk)
    begin
        if rising_edge(clk) then
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
    end process p_sr;

    -- ====================================================================
    -- Output: drawn pixels win; empty pixels show video (S11) or black
    -- ====================================================================
    data_out.hsync_n <= s_hsync_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);

    data_out.y <= s_y_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_y);
    data_out.u <= s_u_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_u);
    data_out.v <= s_v_sr(C_LATENCY - 1) when (s_drawn = '0' and s_bg_video = '1')
                  else std_logic_vector(s_d_v);

end architecture spiroscope;
