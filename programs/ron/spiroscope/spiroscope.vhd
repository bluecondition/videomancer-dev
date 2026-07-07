-- spiroscope.vhd
--
-- Spiroscope: a knob-driven phosphor drawing instrument -- an advanced
-- etch-a-sketch crossed with a spirograph.
--
-- v4: quantized gears + drift, bilinear-smoothed readout, pinwheel rainbow.
--
-- A free-running "pen" plots into a 320x180 x 2-bit phosphor CANVAS held in
-- BRAM (29 EBRs). The canvas fills the whole screen (4x4 px cells = 1280x720)
-- and is displayed through a BILINEAR upscaler: each output pixel blends the
-- 2x2 neighbouring cells, so strokes render as smooth glowing vector lines
-- instead of blocky 4x4 tiles. Two pen engines:
--   * SPIROGRAPH  : x = cx + R1*cos(a*t) + R2*cos(b*t+ph) + R3*cos(c*t)
--     Gear frequencies are QUANTIZED to 16 zones with a small fixed detune:
--     ratios sit near small integers, so the figure locks into recognizable
--     N-lobed rosettes that slowly PRECESS (v3's fully-fine ratios never
--     locked into anything recognizable -- scribble/mush).
--   * ETCH-A-SKETCH: K1/K2 velocity vector, rotated by K4, scaled by K3.
-- A decay sweep fades cells (slider; dithered in the slow zone).
--
-- Control map:
--   K1 (reg0) Gear A 1..16 / X-velocity   S7 (reg6.0) Mode  Spiro/Etch
--   K2 (reg1) Gear B 1..16 / Y-velocity   S8 (reg6.1) Harmonic (3rd term)
--   K3 (reg2) Arm     / Speed scale       S9 (reg6.2) Kaleidoscope (4-fold)
--   K4 (reg3) Phase   / Rotation          S10(reg6.3) Rainbow (pinwheel)
--   K5 (reg4) Color (8 palette zones)     S11(reg6.4) Background Black/Video
--   K6 (reg5) Draw Speed                  Slider(reg7) Persistence (hero)
--
-- Readout pipeline (pixel P arrives on data_in during clock t):
--   end t   : stage2  cell coords, fx/fy sub-position, prefetch addrs, flags
--   end t+1 : stage3  arbiter issues prefetch reads; fx3/insq3
--   end t+2 : stage4  h-blend (TLb/TRb/BLb/BRb x fx); insq4/fy4/idx4
--   end t+3 : stage5  v-blend -> t5 (0..48); insq5/idx5
--   end t+4 : stage6  luma = t5*21, palette chroma  -> C_LATENCY = 5
-- Cell prefetch: during span S the arbiter fetches cells (S+1, row) and
-- (S+1, row+1) on sub-pixel phases 0/1; captures land in TRc/BRc via a tag
-- pipeline and commit to the blend registers exactly at the span boundary.
-- Cell column 0 is blanked (its own value is never prefetched).
--
-- BRAM port plan (single 1W1R EBR canvas, 57600 x 2 = 29 EBRs):
--   * Active pixels : read port -> 2 prefetches per 4-px span. write -> pen.
--   * Blanking      : the decay sweep issues reads whenever avid has been low
--     >= 2 clocks (vblank + hblanks); write-back one clock later (pen commits
--     are gated off while the sweep is active).
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

    constant C_CW      : integer := 320;   -- canvas width  (cells)
    constant C_CH      : integer := 180;   -- canvas height (cells)
    constant C_USHIFT  : integer := 2;     -- 4 screen px per cell
    constant C_VW      : integer := 1280;  -- canvas viewport width  (px)
    constant C_VH      : integer := 720;   -- canvas viewport height (px)
    constant C_LATENCY : integer := 5;     -- input->output register stages
    constant C_CELLS   : integer := C_CW * C_CH;  -- 57600 (29 EBRs at 2 bit)

    -- canvas address = cy*320 + cx (320 = 256+64 -> two shifts + an add)
    function f_addr(cy : unsigned(7 downto 0); cx : unsigned(8 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := resize(cy, 16);
        return shift_left(v, 8) + shift_left(v, 6) + resize(cx, 16);
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
    signal s_cx_center : unsigned(8 downto 0) := to_unsigned(160, 9);
    signal s_cy_center : unsigned(7 downto 0) := to_unsigned(90, 8);

    -- ==== Canvas BRAM: 57600 x 2-bit, addr = cy*320 + cx (1W1R) ====
    type t_canvas is array(0 to C_CELLS - 1) of unsigned(1 downto 0);
    signal s_canvas   : t_canvas := (others => (others => '0'));
    signal s_cv_we    : std_logic := '0';
    signal s_cv_waddr : unsigned(15 downto 0) := (others => '0');
    signal s_cv_wdata : unsigned(1 downto 0)  := (others => '0');
    signal s_cv_raddr : unsigned(15 downto 0) := (others => '0');
    signal s_cv_rdata : unsigned(1 downto 0)  := (others => '0');

    -- ==== Readout stage 2 (registered from raw counters) ====
    signal s_insq2 : std_logic := '0';                         -- display gate (col 0 blanked)
    signal s_cellx2 : unsigned(8 downto 0) := (others => '0');
    signal s_fx2, s_fy2 : unsigned(1 downto 0) := (others => '0');
    signal s_ftop2, s_fbot2 : std_logic := '0';                -- prefetch-issue flags
    signal s_addr_top2, s_addr_bot2 : unsigned(15 downto 0) := (others => '0');

    -- ==== Prefetch tags + capture/blend registers ====
    signal s_tag1, s_tag2, s_tag3 : std_logic_vector(1 downto 0) := "00";
    signal s_TRc, s_BRc : unsigned(1 downto 0) := (others => '0');
    signal s_TLb, s_TRb, s_BLb, s_BRb : unsigned(1 downto 0) := (others => '0');

    -- ==== Blend pipeline ====
    signal s_fx3 : unsigned(1 downto 0) := (others => '0');
    signal s_fy3, s_fy4 : unsigned(1 downto 0) := (others => '0');
    signal s_insq3, s_insq4, s_insq5 : std_logic := '0';
    signal s_h0, s_h1 : unsigned(3 downto 0) := (others => '0');  -- 0..12
    signal s_t5 : unsigned(5 downto 0) := (others => '0');        -- 0..48

    -- ==== Rainbow pinwheel pipeline ====
    signal s_celly2  : unsigned(7 downto 0) := (others => '0');
    signal s_adx3    : unsigned(8 downto 0) := (others => '0');
    signal s_ady3    : unsigned(7 downto 0) := (others => '0');
    signal s_sx3, s_sy3 : std_logic := '0';
    signal s_idx4, s_idx5 : unsigned(3 downto 0) := (others => '0');

    -- ==== Decay sweep (dithered slow zone; see v3 notes) ====
    signal s_decay_active : std_logic := '0';
    signal s_decay_done   : std_logic := '0';
    signal s_decay_rd     : std_logic := '0';
    signal s_decay_k      : unsigned(15 downto 0) := (others => '0');
    signal s_decay_k_d1   : unsigned(15 downto 0) := (others => '0');
    signal s_decay_K_amt  : unsigned(1 downto 0)  := (others => '0');
    signal s_decay_dith   : std_logic := '1';
    signal s_decay_thr    : unsigned(9 downto 0)  := to_unsigned(40, 10);

    -- ==== Pen write request (driven by the walker) ====
    signal s_pen_we    : std_logic := '0';
    signal s_pen_waddr : unsigned(15 downto 0) := (others => '0');
    signal s_pen_wdata : unsigned(1 downto 0)  := (others => '1');

    -- ==== Spirograph pen plotter ====
    -- Gears: inc = (zone+1)*128 + 3.  The *128 quantization locks ratios near
    -- small integers m:n (recognizable rosettes); the +3 detune makes the
    -- figure precess slowly. 18-bit phases keep the per-substep angle step
    -- <= ~2.8 degrees so walked chords stay short (no visible polygons).
    signal s_phaseA  : unsigned(17 downto 0) := (others => '0');
    signal s_phaseB  : unsigned(17 downto 0) := (others => '0');
    signal s_phaseC  : unsigned(17 downto 0) := (others => '0');
    signal s_incA    : unsigned(11 downto 0) := to_unsigned(643, 12);
    signal s_incB    : unsigned(11 downto 0) := to_unsigned(1027, 12);
    signal s_incC    : unsigned(12 downto 0) := to_unsigned(1670, 13);
    signal s_R1      : unsigned(6 downto 0)  := to_unsigned(56, 7);
    signal s_R2      : unsigned(6 downto 0)  := to_unsigned(32, 7);
    signal s_R3      : unsigned(6 downto 0)  := (others => '0');   -- 3rd harmonic (S8)
    signal s_phaseOff: unsigned(7 downto 0)  := (others => '0');
    signal s_budget  : unsigned(15 downto 0) := (others => '0');
    signal s_st      : integer range 0 to 11 := 0;
    signal s_pmode   : std_logic := '0';

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

    -- ==== Etch-a-sketch pen (Q10.10 signed; integrated once per line) ====
    signal s_eX : signed(19 downto 0) := to_signed(160 * 1024, 20);
    signal s_eY : signed(19 downto 0) := to_signed( 90 * 1024, 20);
    signal s_evelX, s_evelY : signed(9 downto 0) := (others => '0');
    signal s_ev0X, s_ev0Y   : signed(7 downto 0) := (others => '0');
    signal s_ek3   : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal s_ek4   : unsigned(7 downto 0) := (others => '0');
    signal s_etick : std_logic := '0';

    -- ==== Plot request (plotter/etch -> writer) ====
    signal s_plot_px : unsigned(8 downto 0) := (others => '0');
    signal s_plot_py : unsigned(7 downto 0) := (others => '0');
    signal s_plot_stb : std_logic := '0';

    -- ==== Pen writer: line-walker + mirror emitter ====
    signal s_w_active : std_logic := '0';
    signal s_w_step   : integer range 0 to 5 := 0;   -- 5 = arm, 0 = walk, 1..4 = mirrors
    signal s_wk_valid : std_logic := '0';
    signal s_wk_x  : unsigned(8 downto 0) := (others => '0');
    signal s_wk_y  : unsigned(7 downto 0) := (others => '0');
    signal s_wk_tx : unsigned(8 downto 0) := (others => '0');
    signal s_wk_ty : unsigned(7 downto 0) := (others => '0');

    -- ==== Palettes (U/V swapped for HW: stored U = Cr, stored V = Cb) ====
    -- 8-entry solid palette (K5): rainbow hues + white
    type t_pal8 is array(0 to 7) of unsigned(9 downto 0);
    constant C_PAL_U : t_pal8 := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned(608, 10), to_unsigned(756, 10), to_unsigned(512, 10));
    constant C_PAL_V : t_pal8 := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned(696, 10), to_unsigned(852, 10), to_unsigned(512, 10));
    -- 16-entry hue wheel for rainbow mode: (Cb,Cr) = 512 + 220*(cos,sin)(k*22.5deg)
    type t_pal16 is array(0 to 15) of unsigned(9 downto 0);
    constant C_WHL_U : t_pal16 := (   -- = Cr = 512 + 220*sin
        to_unsigned(512, 10), to_unsigned(596, 10), to_unsigned(668, 10), to_unsigned(715, 10),
        to_unsigned(732, 10), to_unsigned(715, 10), to_unsigned(668, 10), to_unsigned(596, 10),
        to_unsigned(512, 10), to_unsigned(428, 10), to_unsigned(356, 10), to_unsigned(309, 10),
        to_unsigned(292, 10), to_unsigned(309, 10), to_unsigned(356, 10), to_unsigned(428, 10));
    constant C_WHL_V : t_pal16 := (   -- = Cb = 512 + 220*cos
        to_unsigned(732, 10), to_unsigned(715, 10), to_unsigned(668, 10), to_unsigned(596, 10),
        to_unsigned(512, 10), to_unsigned(428, 10), to_unsigned(356, 10), to_unsigned(309, 10),
        to_unsigned(292, 10), to_unsigned(309, 10), to_unsigned(356, 10), to_unsigned(428, 10),
        to_unsigned(512, 10), to_unsigned(596, 10), to_unsigned(668, 10), to_unsigned(715, 10));

    -- ==== Colour / output (stage 6) ====
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
                s_mode      <= registers_in(6)(0);
                s_kaleido   <= registers_in(6)(2);
                s_rainbow   <= registers_in(6)(3);
                s_bg_video  <= registers_in(6)(4);
                s_color_sel <= unsigned(registers_in(4)(9 downto 7));
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
                if s_line_width >= to_unsigned(C_VW, 12) then
                    s_cx_center <= to_unsigned(C_CW / 2, 9);
                else
                    s_cx_center <= resize(shift_right(s_line_width, 3), 9);
                end if;
                if s_frame_height >= to_unsigned(C_VH, 12) then
                    s_cy_center <= to_unsigned(C_CH / 2, 8);
                else
                    s_cy_center <= resize(shift_right(s_frame_height, 3), 8);
                end if;
            end if;
        end if;
    end process p_count;

    -- ====================================================================
    -- stage 2: geometry -> cell coords + sub-position + prefetch addresses
    -- ====================================================================
    p_addr : process(clk)
        variable v_cx, v_cy : unsigned(11 downto 0);
        variable v_ccx      : unsigned(8 downto 0);
        variable v_ccy      : unsigned(7 downto 0);
        variable v_ccx1     : unsigned(8 downto 0);
        variable v_ccy1     : unsigned(7 downto 0);
        variable v_in       : boolean;
    begin
        if rising_edge(clk) then
            v_in := (s_x_count >= s_x0) and (s_x_count < s_x0 + to_unsigned(C_VW, 12))
                and (s_y_count >= s_y0) and (s_y_count < s_y0 + to_unsigned(C_VH, 12));
            v_cx := s_x_count - s_x0;
            v_cy := s_y_count - s_y0;
            v_ccx := resize(shift_right(v_cx, C_USHIFT), 9);   -- 0..319
            v_ccy := resize(shift_right(v_cy, C_USHIFT), 8);   -- 0..179
            -- prefetch target: span S+1's RIGHT neighbour = cell S+2
            if v_ccx >= to_unsigned(C_CW - 2, 9) then v_ccx1 := to_unsigned(C_CW - 1, 9);
            else v_ccx1 := v_ccx + 2; end if;
            if v_ccy = to_unsigned(C_CH - 1, 8) then v_ccy1 := v_ccy;
            else v_ccy1 := v_ccy + 1; end if;

            s_cellx2 <= v_ccx;
            s_celly2 <= v_ccy;
            s_fx2 <= v_cx(1 downto 0);
            s_fy2 <= v_cy(1 downto 0);
            -- prefetch the (span+2) cell pair on sub-pixel phases 0 and 1
            s_addr_top2 <= f_addr(v_ccy,  v_ccx1);
            s_addr_bot2 <= f_addr(v_ccy1, v_ccx1);
            if v_in and v_cx(1 downto 0) = "00" then s_ftop2 <= '1'; else s_ftop2 <= '0'; end if;
            if v_in and v_cx(1 downto 0) = "01" then s_fbot2 <= '1'; else s_fbot2 <= '0'; end if;
            -- display gate: cell columns 0/1 blanked (no valid prefetch can
            -- exist for them at line start -- 8 px at the screen edge)
            if v_in and v_ccx >= 2 then s_insq2 <= '1'; else s_insq2 <= '0'; end if;
        end if;
    end process p_addr;

    -- ====================================================================
    -- stage 3: canvas port arbiter -- decay sweep (blanking) / prefetch
    -- ====================================================================
    p_canvas_ctrl : process(clk)
        variable v_sl   : unsigned(9 downto 0);
        variable v_new  : unsigned(1 downto 0);
        variable v_hash : unsigned(9 downto 0);
        variable v_rd   : boolean;
    begin
        if rising_edge(clk) then
            s_cv_we <= '0';

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                v_sl := unsigned(registers_in(7));
                -- slider 0     : permanent (no sweep)
                -- 1..511 slow  : dithered fade-by-1, P = thr/1024 per cell/frame
                -- 512..1000    : fade by K = 1..2 every frame
                -- >1000        : K = 3, one-frame erase (shake to clear)
                if v_sl < to_unsigned(512, 10) then
                    s_decay_dith <= '1';
                    s_decay_thr  <= v_sl;
                    s_decay_K_amt <= to_unsigned(1, 2);
                else
                    s_decay_dith <= '0';
                    if v_sl > to_unsigned(1000, 10) then
                        s_decay_K_amt <= to_unsigned(3, 2);
                    elsif v_sl >= to_unsigned(768, 10) then
                        s_decay_K_amt <= to_unsigned(2, 2);
                    else
                        s_decay_K_amt <= to_unsigned(1, 2);
                    end if;
                end if;

                if v_sl /= 0 then
                    s_decay_active <= '1';
                    s_decay_done   <= '0';
                    s_decay_k      <= (others => '0');
                end if;
            end if;

            -- ---- read port ----
            -- sweep needs avid low >=2 clocks (protects the trailing fetches)
            v_rd := (s_decay_active = '1') and (s_decay_done = '0')
                    and (data_in.avid = '0') and (s_avid_sr(0) = '0') and (s_avid_sr(1) = '0');
            if v_rd then
                s_cv_raddr   <= s_decay_k;
                s_decay_k_d1 <= s_decay_k;
                if s_decay_k = to_unsigned(C_CELLS - 1, 16) then
                    s_decay_done <= '1';
                else
                    s_decay_k <= s_decay_k + 1;
                end if;
                s_decay_rd <= '1';
                s_tag1 <= "00";
            else
                s_decay_rd <= '0';
                if s_ftop2 = '1' then
                    s_cv_raddr <= s_addr_top2;
                    s_tag1 <= "01";
                elsif s_fbot2 = '1' then
                    s_cv_raddr <= s_addr_bot2;
                    s_tag1 <= "10";
                else
                    s_cv_raddr <= s_addr_top2;
                    s_tag1 <= "00";
                end if;
            end if;

            -- ---- write port ----
            if s_decay_rd = '1' then
                if s_decay_dith = '1' then
                    v_hash := (s_decay_k_d1(9 downto 0) xor s_decay_k_d1(15 downto 6))
                              + resize(s_frame_cnt, 10);
                    if (v_hash < s_decay_thr) and (s_cv_rdata /= 0) then
                        v_new := s_cv_rdata - 1;
                    else
                        v_new := s_cv_rdata;
                    end if;
                else
                    if s_cv_rdata <= s_decay_K_amt then
                        v_new := (others => '0');
                    else
                        v_new := s_cv_rdata - s_decay_K_amt;
                    end if;
                end if;
                s_cv_we    <= '1';
                s_cv_waddr <= s_decay_k_d1;
                s_cv_wdata <= v_new;
                if s_decay_done = '1' then
                    s_decay_active <= '0';
                end if;
            elsif s_decay_active = '0' and s_pen_we = '1' then
                s_cv_we    <= '1';
                s_cv_waddr <= s_pen_waddr;
                s_cv_wdata <= s_pen_wdata;
            end if;
        end if;
    end process p_canvas_ctrl;

    -- ==== canvas BRAM (1W1R, c64 p_buf pattern) ====
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
    -- Prefetch captures + span-boundary commit (tag pipeline).
    -- The captures are the "register the 29-EBR read mux" timing fix.
    -- ====================================================================
    p_fetch : process(clk)
    begin
        if rising_edge(clk) then
            s_tag2 <= s_tag1;
            s_tag3 <= s_tag2;
            if s_tag2 = "01" then
                s_TRc <= s_cv_rdata;
            elsif s_tag2 = "10" then
                s_BRc <= s_cv_rdata;
            end if;
            if s_tag3 = "10" then          -- span boundary: rotate right -> left
                s_TLb <= s_TRb;  s_BLb <= s_BRb;
                s_TRb <= s_TRc;  s_BRb <= s_BRc;
            end if;
        end if;
    end process p_fetch;

    -- ====================================================================
    -- stages 3-5: bilinear blend + pinwheel rainbow index
    -- ====================================================================
    p_blend : process(clk)
        variable v_wxl : unsigned(2 downto 0);   -- 4-fx
        variable v_wyl : unsigned(2 downto 0);   -- 4-fy
        variable v_dx  : signed(9 downto 0);
        variable v_dy  : signed(8 downto 0);
        variable v_oct : unsigned(2 downto 0);
        variable v_m   : boolean;
    begin
        if rising_edge(clk) then
            -- pipes
            s_fx3 <= s_fx2;
            s_fy3 <= s_fy2;   s_fy4 <= s_fy3;
            s_insq3 <= s_insq2;  s_insq4 <= s_insq3;  s_insq5 <= s_insq4;

            -- stage 3->4: horizontal lerp (blend regs are span-stable)
            v_wxl := "100" - resize(s_fx3, 3);
            s_h0 <= resize(s_TLb * v_wxl, 4) + resize(s_TRb * s_fx3, 4);
            s_h1 <= resize(s_BLb * v_wxl, 4) + resize(s_BRb * s_fx3, 4);

            -- stage 4->5: vertical lerp -> t (0..48)
            v_wyl := "100" - resize(s_fy4, 3);
            s_t5 <= resize(s_h0 * v_wyl, 6) + resize(s_h1 * s_fy4, 6);

            -- rainbow pinwheel: octant of (cell - centre), radius ring, rotation
            v_dx := signed(resize(s_cellx2, 10)) - signed(resize(s_cx_center, 10));
            v_dy := signed(resize(s_celly2, 9))  - signed(resize(s_cy_center, 9));
            if v_dx >= 0 then s_sx3 <= '1'; s_adx3 <= resize(unsigned(v_dx), 9);
            else              s_sx3 <= '0'; s_adx3 <= resize(unsigned(-v_dx), 9); end if;
            if v_dy >= 0 then s_sy3 <= '1'; s_ady3 <= resize(unsigned(v_dy), 8);
            else              s_sy3 <= '0'; s_ady3 <= resize(unsigned(-v_dy), 8); end if;

            v_m := resize(s_ady3, 9) > s_adx3;
            if    s_sx3 = '1' and s_sy3 = '1' and not v_m then v_oct := "000";
            elsif s_sx3 = '1' and s_sy3 = '1' and     v_m then v_oct := "001";
            elsif s_sx3 = '0' and s_sy3 = '1' and     v_m then v_oct := "010";
            elsif s_sx3 = '0' and s_sy3 = '1' and not v_m then v_oct := "011";
            elsif s_sx3 = '0' and s_sy3 = '0' and not v_m then v_oct := "100";
            elsif s_sx3 = '0' and s_sy3 = '0' and     v_m then v_oct := "101";
            elsif s_sx3 = '1' and s_sy3 = '0' and     v_m then v_oct := "110";
            else                                               v_oct := "111";
            end if;
            -- idx = octant*2 + radius ring + slow rotation (wraps mod 16)
            s_idx4 <= (v_oct & '0')
                      + resize(s_adx3(4 downto 4), 4)
                      + resize(s_ady3(4 downto 4), 4)
                      + resize(s_frame_cnt(7 downto 4), 4);
            s_idx5 <= s_idx4;
        end if;
    end process p_blend;

    -- ====================================================================
    -- Pen plotter: shared sin/cos LUT + shared multiplier + FSM
    -- ====================================================================
    lut : entity work.sin_cos_full_lut_10x10
        port map (angle_in => s_lut_angle, sin_out => s_lut_sin, cos_out => s_lut_cos);

    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            s_mul_p <= s_mul_a * s_mul_b;       -- signed(7:0) * signed(9:0)
        end if;
    end process p_mul;

    -- Operands set in state k are seen by p_mul during k+1, so the product is
    -- readable in state k+2. Both FSMs follow that schedule strictly.
    p_plot : process(clk)
        variable v_r2   : unsigned(6 downto 0);
        variable v_base : integer range 66 to 88;
        variable v_m    : std_logic;
        variable v_pxs  : signed(9 downto 0);
        variable v_pys  : signed(9 downto 0);
        variable v_px   : unsigned(8 downto 0);
        variable v_py   : unsigned(7 downto 0);
        variable v_ia   : unsigned(11 downto 0);
        variable v_ib   : unsigned(11 downto 0);
        variable v_nx   : signed(19 downto 0);
        variable v_ny   : signed(19 downto 0);
        variable v_accy : signed(20 downto 0);
        variable v_rst  : boolean;
    begin
        if rising_edge(clk) then
            s_plot_stb <= '0';
            v_rst := false;

            -- ---- per-frame control latch + budget reload ----
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                -- gears: (zone+1)*128 + 3  (zone = knob top 4 bits -> 1..16)
                v_ia := shift_left(resize(unsigned(registers_in(0)(9 downto 6)), 12) + 1, 7)
                        + to_unsigned(3, 12);
                v_ib := shift_left(resize(unsigned(registers_in(1)(9 downto 6)), 12) + 1, 7)
                        + to_unsigned(3, 12);
                s_incA <= v_ia;
                s_incB <= v_ib;
                s_incC <= resize(v_ia, 13) + resize(v_ib, 13);
                -- radius budget: R1+R2+R3 = 88 (fills the 180-cell height)
                if registers_in(6)(1) = '1' then
                    s_R3 <= to_unsigned(22, 7);  v_base := 66;
                else
                    s_R3 <= (others => '0');      v_base := 88;
                end if;
                v_r2 := unsigned(registers_in(2)(9 downto 3));                      -- 0..127
                if v_r2 > to_unsigned(v_base, 7) then v_r2 := to_unsigned(v_base, 7); end if;
                s_R2 <= v_r2;
                s_R1 <= to_unsigned(v_base, 7) - v_r2;
                s_phaseOff <= unsigned(registers_in(3)(9 downto 2));
                -- substeps this frame: 8..519 (K6): slow watch-it-draw at the
                -- bottom, up to ~4 revolutions/frame at the top
                s_budget   <= to_unsigned(8, 16) + resize(unsigned(registers_in(5)(9 downto 1)), 16);
                s_ev0X  <= resize(shift_right(signed('0' & registers_in(0)) - to_signed(512, 11), 2), 8);
                s_ev0Y  <= resize(shift_right(signed('0' & registers_in(1)) - to_signed(512, 11), 2), 8);
                s_ek3   <= unsigned(registers_in(2)(9 downto 3));
                s_ek4   <= unsigned(registers_in(3)(9 downto 2));
                s_etick <= '1';
                v_m := registers_in(6)(0);
                if v_m /= s_pmode then v_rst := true; end if;
                s_pmode <= v_m;
            end if;

            if s_mode = '0' then
                -- ============ SPIROGRAPH ============
                -- LUT gets only the top 8 angle bits (low 2 tied 0): prunes a
                -- quarter of the ROM mux (critical-path fix).
                case s_st is
                    when 0 =>
                        if s_budget > 0 and s_decay_active = '0' then
                            s_lut_angle <= std_logic_vector(s_phaseA(17 downto 10)) & "00";
                            s_st <= 1;
                        end if;
                    when 1 =>
                        s_sinA <= s_lut_sin;  s_cosA <= s_lut_cos;
                        s_lut_angle <= std_logic_vector(s_phaseB(17 downto 10) + s_phaseOff) & "00";
                        s_st <= 2;
                    when 2 =>
                        s_sinB <= s_lut_sin;  s_cosB <= s_lut_cos;
                        s_lut_angle <= std_logic_vector(s_phaseC(17 downto 10)) & "00";
                        s_mul_a <= signed('0' & s_R1);  s_mul_b <= s_cosA;     -- set R1*cosA
                        s_st <= 3;
                    when 3 =>
                        s_sinC <= s_lut_sin;  s_cosC <= s_lut_cos;
                        s_mul_a <= signed('0' & s_R2);  s_mul_b <= s_cosB;     -- set R2*cosB
                        s_st <= 4;
                    when 4 =>
                        s_accX  <= resize(s_mul_p, 21);                        -- read R1*cosA
                        s_mul_a <= signed('0' & s_R3);  s_mul_b <= s_cosC;     -- set R3*cosC
                        s_st <= 5;
                    when 5 =>
                        s_accX  <= s_accX + resize(s_mul_p, 21);               -- read R2*cosB
                        s_mul_a <= signed('0' & s_R1);  s_mul_b <= s_sinA;     -- set R1*sinA
                        s_st <= 6;
                    when 6 =>
                        s_accX  <= s_accX + resize(s_mul_p, 21);               -- read R3*cosC
                        s_mul_a <= signed('0' & s_R2);  s_mul_b <= s_sinB;     -- set R2*sinB
                        s_st <= 7;
                    when 7 =>
                        s_accY  <= resize(s_mul_p, 21);                        -- read R1*sinA
                        s_mul_a <= signed('0' & s_R3);  s_mul_b <= s_sinC;     -- set R3*sinC
                        s_st <= 8;
                    when 8 =>
                        s_accY  <= s_accY + resize(s_mul_p, 21);               -- read R2*sinB
                        s_st <= 9;
                    when 9 =>
                        -- stall until the walker is free (s_mul_p holds R3*sinC)
                        if s_w_active = '0' then
                            v_accy := s_accY + resize(s_mul_p, 21);            -- read R3*sinC
                            v_pxs := resize(signed('0' & s_cx_center), 10) + resize(shift_right(s_accX, 9), 10);
                            v_pys := resize(signed('0' & s_cy_center), 10) + resize(shift_right(v_accy, 9), 10);
                            if    v_pxs < 0   then v_px := (others => '0');
                            elsif v_pxs > C_CW - 1 then v_px := to_unsigned(C_CW - 1, 9);
                            else  v_px := unsigned(v_pxs(8 downto 0)); end if;
                            if    v_pys < 0   then v_py := (others => '0');
                            elsif v_pys > C_CH - 1 then v_py := to_unsigned(C_CH - 1, 8);
                            else  v_py := unsigned(v_pys(7 downto 0)); end if;

                            s_plot_px  <= v_px;
                            s_plot_py  <= v_py;
                            s_plot_stb <= '1';

                            s_phaseA <= s_phaseA + resize(s_incA, 18);
                            s_phaseB <= s_phaseB + resize(s_incB, 18);
                            s_phaseC <= s_phaseC + resize(s_incC, 18);
                            if s_budget > 0 then s_budget <= s_budget - 1; end if;
                            s_st <= 0;
                        end if;
                    when others =>
                        s_st <= 0;
                end case;
            else
                -- ============ ETCH-A-SKETCH ============
                case s_st is
                    when 0 =>
                        if s_etick = '1' then
                            s_lut_angle <= std_logic_vector(s_ek4) & "00";
                            s_etick <= '0';
                            s_st <= 1;
                        end if;
                    when 1 =>
                        s_sinA <= s_lut_sin;  s_cosA <= s_lut_cos;
                        s_st <= 2;
                    when 2 =>
                        s_mul_a <= s_ev0X;  s_mul_b <= s_cosA;                 -- set X*cos
                        s_st <= 3;
                    when 3 =>
                        s_mul_a <= s_ev0Y;  s_mul_b <= s_sinA;                 -- set Y*sin
                        s_st <= 4;
                    when 4 =>
                        s_accX <= resize(s_mul_p, 21);                         -- read X*cos
                        s_mul_a <= s_ev0X;  s_mul_b <= s_sinA;                 -- set X*sin
                        s_st <= 5;
                    when 5 =>
                        s_accX <= s_accX - resize(s_mul_p, 21);                -- read Y*sin (rx done)
                        s_mul_a <= s_ev0Y;  s_mul_b <= s_cosA;                 -- set Y*cos
                        s_st <= 6;
                    when 6 =>
                        s_accY <= resize(s_mul_p, 21);                         -- read X*sin
                        s_sinB <= resize(shift_right(s_accX, 8), 10);          -- rx -> 10-bit
                        s_st <= 7;
                    when 7 =>
                        s_accY <= s_accY + resize(s_mul_p, 21);                -- read Y*cos (ry done)
                        s_st <= 8;
                    when 8 =>
                        s_cosB <= resize(shift_right(s_accY, 8), 10);          -- ry -> 10-bit
                        s_mul_a <= signed('0' & s_ek3);  s_mul_b <= s_sinB;    -- set K3*rx
                        s_st <= 9;
                    when 9 =>
                        s_mul_a <= signed('0' & s_ek3);  s_mul_b <= s_cosB;    -- set K3*ry
                        s_st <= 10;
                    when 10 =>
                        s_evelX <= resize(shift_right(s_mul_p, 9), 10);        -- read K3*rx
                        s_st <= 11;
                    when 11 =>
                        s_evelY <= resize(shift_right(s_mul_p, 9), 10);        -- read K3*ry
                        s_st <= 0;
                end case;

                -- once per line: integrate velocity (Q10.10 -> sub-cell smooth)
                if data_in.hsync_n = '0' and s_prev_hsync_n = '1' and s_decay_active = '0' then
                    v_nx := s_eX + resize(s_evelX, 20);
                    v_ny := s_eY + resize(s_evelY, 20);
                    if v_nx < 0 then v_nx := (others => '0');
                    elsif v_nx > to_signed((C_CW - 1) * 1024, 20) then v_nx := to_signed((C_CW - 1) * 1024, 20); end if;
                    if v_ny < 0 then v_ny := (others => '0');
                    elsif v_ny > to_signed((C_CH - 1) * 1024, 20) then v_ny := to_signed((C_CH - 1) * 1024, 20); end if;
                    s_eX <= v_nx;
                    s_eY <= v_ny;
                    s_plot_px  <= unsigned(v_nx(18 downto 10));
                    s_plot_py  <= unsigned(v_ny(17 downto 10));
                    s_plot_stb <= '1';
                end if;
            end if;

            if v_rst then s_st <= 0; end if;
        end if;
    end process p_plot;

    -- ====================================================================
    -- Pen writer: 8-connected line-walker (solid strokes) + kaleido mirrors
    -- ====================================================================
    p_writer : process(clk)
        variable v_px  : unsigned(8 downto 0);
        variable v_py  : unsigned(7 downto 0);
        variable v_dx  : unsigned(8 downto 0);
        variable v_dy  : unsigned(7 downto 0);
        variable v_wx  : unsigned(8 downto 0);
        variable v_wy  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_pen_we <= '0';
            if s_w_active = '0' then
                if s_plot_stb = '1' then
                    s_wk_tx    <= s_plot_px;
                    s_wk_ty    <= s_plot_py;
                    s_w_step   <= 5;
                    s_w_active <= '1';
                end if;
            else
                if s_w_step = 5 then
                    -- arm: teleport on big jumps (pipelined off the accept clock)
                    if s_wk_x > s_wk_tx then v_dx := s_wk_x - s_wk_tx;
                    else                     v_dx := s_wk_tx - s_wk_x; end if;
                    if s_wk_y > s_wk_ty then v_dy := s_wk_y - s_wk_ty;
                    else                     v_dy := s_wk_ty - s_wk_y; end if;
                    if s_wk_valid = '0' or v_dx > 24 or v_dy > 24 then
                        s_wk_x <= s_wk_tx;
                        s_wk_y <= s_wk_ty;
                    end if;
                    s_wk_valid <= '1';
                    s_w_step   <= 0;
                elsif s_w_step = 0 then
                    if s_wk_x < s_wk_tx then s_wk_x <= s_wk_x + 1;
                    elsif s_wk_x > s_wk_tx then s_wk_x <= s_wk_x - 1; end if;
                    if s_wk_y < s_wk_ty then s_wk_y <= s_wk_y + 1;
                    elsif s_wk_y > s_wk_ty then s_wk_y <= s_wk_y - 1; end if;
                    s_w_step <= 1;
                else
                    v_wx := s_wk_x;
                    v_wy := s_wk_y;
                    case s_w_step is
                        when 1      => v_px := v_wx;                            v_py := v_wy;
                        when 2      => v_px := to_unsigned(C_CW - 1, 9) - v_wx; v_py := v_wy;
                        when 3      => v_px := v_wx;                            v_py := to_unsigned(C_CH - 1, 8) - v_wy;
                        when others => v_px := to_unsigned(C_CW - 1, 9) - v_wx; v_py := to_unsigned(C_CH - 1, 8) - v_wy;
                    end case;
                    s_pen_we    <= '1';
                    s_pen_wdata <= "11";
                    s_pen_waddr <= f_addr(v_py, v_px);

                    if (s_kaleido = '0') or (s_w_step = 4) then
                        if s_wk_x = s_wk_tx and s_wk_y = s_wk_ty then
                            s_w_active <= '0';
                        else
                            s_w_step <= 0;
                        end if;
                    else
                        s_w_step <= s_w_step + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_writer;

    -- ====================================================================
    -- stage 6: t -> luma (t*21, max 1008); palette / hue-wheel chroma
    -- ====================================================================
    p_color : process(clk)
        variable v_t : unsigned(5 downto 0);
        variable v_y : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_t := s_t5;
            v_y := resize(v_t & "0000", 11) + resize(v_t & "00", 11) + resize(v_t, 11);
            if s_insq5 = '1' and v_t /= 0 then
                s_d_y <= v_y(9 downto 0);
                if s_rainbow = '1' then
                    s_d_u <= C_WHL_U(to_integer(s_idx5));
                    s_d_v <= C_WHL_V(to_integer(s_idx5));
                else
                    s_d_u <= C_PAL_U(to_integer(s_color_sel));
                    s_d_v <= C_PAL_V(to_integer(s_color_sel));
                end if;
                -- video-background key only for solid-enough pixels (soft
                -- fringes key over black, not over video)
                if v_t >= 8 then s_drawn <= '1'; else s_drawn <= '0'; end if;
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
