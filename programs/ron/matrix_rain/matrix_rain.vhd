-- matrix_rain.vhd
-- Copyright (C) 2026  bluecondition
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Matrix Rain - classic "digital rain" generator for Videomancer.
--
-- A black field is divided into fixed 16x16 glyph cells.  Every vertical column
-- has an independently-seeded falling "head" that advances once per video frame
-- (vsync tick) at a per-column randomized speed.  The head glyph is a bright
-- near-white green; trailing glyphs fade bright-green -> dark-green -> black over
-- a configurable tail length (a fixed per-row brightness curve, cut off at the
-- tail length).  Glyphs mutate to new characters at random, staggered times.
-- All randomness is LFSR / integer-hash based (no DRAM); the only stateful
-- memory is a 128-entry BRAM of per-column head positions and the glyph ROM.
-- Pure synthesis: the incoming video is ignored (only its sync is genlocked).
--
-- Glyph ROM: matrix_rain_glyphs_pkg.vhd (16x16 1bpp), generated at build time
-- by matrix_rain.py from the vendored GNU Unifont subset (half-width katakana
-- U+FF66..FF9D + digits + a few Latin/symbols).
--
-- =====================================================================
-- Register / front-panel map  (registers_in indices, see docs/abi-format.md)
-- ---------------------------------------------------------------------
--   0x00  K1  rotary_potentiometer_1   Fall Speed        (faster -> bigger step)
--   0x01  K2  rotary_potentiometer_2   Density           (column spawn fraction)
--   0x02  K3  rotary_potentiometer_3   Tail Length       (visible trail rows)
--   0x03  K4  rotary_potentiometer_4   Mutation Rate     (glyph re-roll speed)
--   0x04  K5  rotary_potentiometer_5   Hue / Tint        (green hue shift)
--   0x05  K6  rotary_potentiometer_6   Brightness        (overall luma scale)
--   0x06b0 S7 toggle_switch_7          Head Glow / bloom on/off
--   0x06b1 S8 toggle_switch_8          Mirror glyphs horizontally on/off
--   0x07  P12 linear_potentiometer_12  Glyph Size  (cell = 16<<zoom: 16/32/64/128 px)
-- =====================================================================
--
-- Colour: BT.601 YUV, stored U/V-SWAPPED for the Videomancer hardware convention
-- (stored U = Cr, stored V = Cb), matching ziffern / the verified phosphor table.
--   Body green : Y=831 U=155 V=262   (saturated phosphor green, ziffern hue idx7)
--   Head white : luma raised to ~1000, chroma pulled halfway to neutral (512) so
--                the head glyph reads as a hot near-white green.
-- Derivation (pre-swap BT.601 from RGB(0,255,80)-ish phosphor green):
--   Y = 0.299R+0.587G+0.114B, Cb=0.564(B-Y), Cr=0.713(R-Y), scaled to 10-bit,
--   then Cb<->Cr swapped for output.  We reuse ziffern's hardware-checked values.
--
-- =====================================================================
-- Pipeline (one pixel per clock; latencies in clocks after the source pixel):
--   S0  p_acc      cell/pixel accumulators -> col,row,gx,gy,render; set RAM raddr
--   --  RAM        per-column head BRAM 1-cycle registered read (aligned to S1)
--   S1  p_s1       delay stage so head_q lines up with the cell coords
--   S2  p_dist     head distance, lit/head flags, brightness index, glyph hash
--   S3  p_idx      glyph index (mutation gen) + per-cell green/head colour pick
--   S4  p_rom      glyph ROM rows (centre + above/below for glow), mirror applied
--   S5  p_pix      glyph bit extract + 1px head-glow dilation
--   S6  p_color    final YUV mux (head / body-fade / glow / background)
--   --  p_io       output registers
-- Total datapath latency S0..p_io = 7 register stages + the RAM read.  The sync
-- passthrough (t_syncp) is delayed C_SYNCD stages to land with S6; tune C_SYNCD
-- by +/-1 if the image shears horizontally in simulation.
-- =====================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.matrix_rain_glyphs_pkg.all;

architecture matrix_rain of program_top is

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Base glyph bitmap is 16x16.  The on-screen cell is 16<<zoom px (zoom 0..3
    -- from P12), so col = x>>(4+zoom) and the glyph pixel is (x>>zoom) mod 16 -
    -- a nearest-neighbour upscale.  Power-of-two -> pure bit-slicing, no dividers.
    constant C_CELLB : integer := 4;          -- log2(16) base cell

    -- Per-frame mutation shift: glyph re-rolls every ~2^C_MUT_RATE / T_inc frames.
    constant C_MUT_RATE : integer := 7;

    -- Sync passthrough depth (see pipeline note above).  Matches ziffern's
    -- proven 7-stage datapath (s0..s6) -> consume index 5 at the colour stage.
    -- Tune +/-1 if the image shears horizontally in simulation.
    constant C_SYNCD : integer := 5;

    -- Body phosphor green (stored U/V-swapped for HW).  Head colour derived.
    constant C_GRN_Y : unsigned(9 downto 0) := to_unsigned(831, 10);
    constant C_GRN_U : unsigned(9 downto 0) := to_unsigned(155, 10);
    constant C_GRN_V : unsigned(9 downto 0) := to_unsigned(262, 10);

    --------------------------------------------------------------------------
    -- Fixed brightness-fade curve, indexed by distance-below-head (0..15).
    -- index 0 = the head row body luma (head also whitens its chroma); the
    -- curve decays to a dim ember by index 15.  The tail-length knob sets the
    -- cut-off (lit = dist < tail); this table only shapes the falloff, so the
    -- whole thing stays shift/LUT-only (no per-pixel divide).
    --------------------------------------------------------------------------
    type t_fade is array (0 to 15) of unsigned(9 downto 0);
    constant C_FADE : t_fade := (
        to_unsigned(1023,10), to_unsigned(900,10), to_unsigned(790,10), to_unsigned(690,10),
        to_unsigned( 600,10), to_unsigned(520,10), to_unsigned(450,10), to_unsigned(385,10),
        to_unsigned( 325,10), to_unsigned(270,10), to_unsigned(220,10), to_unsigned(175,10),
        to_unsigned( 135,10), to_unsigned(100,10), to_unsigned( 70,10), to_unsigned( 45,10));

    --------------------------------------------------------------------------
    -- Helpers
    --------------------------------------------------------------------------
    function mix16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x;
        v := v xor shift_left(v, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        v := v xor shift_right(v, 5);
        return v;
    end function;

    -- shift-only 8-step brightness scaler (top 3 bits of a knob select gain).
    function scale8(val : unsigned(9 downto 0); sel : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case sel is
            when "000" => r := shift_right(val, 3);
            when "001" => r := shift_right(val, 2);
            when "010" => r := shift_right(val, 2) + shift_right(val, 3);
            when "011" => r := shift_right(val, 1);
            when "100" => r := shift_right(val, 1) + shift_right(val, 3);
            when "101" => r := shift_right(val, 1) + shift_right(val, 2);
            when "110" => r := shift_right(val, 1) + shift_right(val, 2) + shift_right(val, 3);
            when others => r := val;
        end case;
        return r;
    end function;

    -- pixel at column p of a 16-bit glyph row, MSB = leftmost (col 0).
    function getbit(rb : std_logic_vector(15 downto 0); p : integer) return std_logic is
    begin
        if p >= 0 and p <= 15 then return rb(15 - p); else return '0'; end if;
    end function;

    -- reverse a 16-bit row (horizontal mirror).
    function revb(rb : std_logic_vector(15 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(15 downto 0);
    begin
        for i in 0 to 15 loop r(i) := rb(15 - i); end loop;
        return r;
    end function;

    --------------------------------------------------------------------------
    -- Knob / switch reads
    --------------------------------------------------------------------------
    signal s_speedknob  : unsigned(9 downto 0);
    signal s_densknob   : unsigned(9 downto 0);
    signal s_tailknob   : unsigned(9 downto 0);
    signal s_mutknob    : unsigned(9 downto 0);
    signal s_hueknob    : unsigned(9 downto 0);
    signal s_brightknob : unsigned(9 downto 0);
    signal s_glow_en    : std_logic;
    signal s_mirror     : std_logic;
    signal s_fade       : unsigned(9 downto 0);

    --------------------------------------------------------------------------
    -- Timing / resolution
    --------------------------------------------------------------------------
    signal s_timing  : t_video_timing_port;
    signal s_h_count : unsigned(11 downto 0);
    signal s_v_count : unsigned(11 downto 0);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_vsync_prev  : std_logic := '1';
    signal s_vsync_pulse : std_logic := '0';
    signal s_firstline   : std_logic := '1';
    -- current pixel coordinates (own accumulators; pixel_counter's v_count has a
    -- first-line off-by-one we avoid).  s_x resets each line, s_y per frame.
    signal s_x : unsigned(11 downto 0) := (others => '0');
    signal s_y : unsigned(11 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-frame latched parameters (all shift-only math).
    --------------------------------------------------------------------------
    signal s_zoom     : unsigned(1 downto 0)  := "01";                -- cell = 16<<zoom px
    signal s_ncols    : unsigned(7 downto 0)  := to_unsigned(60, 8);   -- columns (<=128)
    signal s_nrows    : unsigned(7 downto 0)  := to_unsigned(33, 8);   -- rows
    signal s_tail     : unsigned(5 downto 0)  := to_unsigned(12, 6);   -- visible tail rows
    signal s_dens8    : unsigned(7 downto 0)  := to_unsigned(180, 8);  -- spawn threshold
    signal s_basespd  : unsigned(8 downto 0)  := to_unsigned(40, 9);   -- frac/frame base
    signal s_T        : unsigned(23 downto 0) := (others => '0');      -- global mutation tick
    signal s_body_u   : unsigned(9 downto 0)  := C_GRN_U;
    signal s_body_v   : unsigned(9 downto 0)  := C_GRN_V;
    signal s_head_u   : unsigned(9 downto 0)  := to_unsigned(334, 10);
    signal s_head_v   : unsigned(9 downto 0)  := to_unsigned(387, 10);
    signal s_bg_y     : unsigned(9 downto 0)  := (others => '0');
    signal s_bg_u     : unsigned(9 downto 0)  := C_MID;
    signal s_bg_v     : unsigned(9 downto 0)  := C_MID;
    signal s_bright3  : unsigned(2 downto 0)  := "111";

    --------------------------------------------------------------------------
    -- Per-column head-position BRAM (16-bit fixed point: [15:6]=signed row int,
    -- [5:0]=fraction).  Read for render (addr = current col) and read/written by
    -- the per-frame update walk (which only runs during vblank, so the single
    -- read port is time-multiplexed with no conflict).
    --------------------------------------------------------------------------
    type t_headmem is array (0 to 127) of std_logic_vector(15 downto 0);
    signal s_head_mem : t_headmem := (others => (others => '0'));
    attribute ram_style : string;
    attribute ram_style of s_head_mem : signal is "block";

    signal s_head_raddr : unsigned(6 downto 0) := (others => '0');
    signal s_head_q     : std_logic_vector(15 downto 0) := (others => '0');
    signal s_hw_we      : std_logic := '0';
    signal s_hw_waddr   : unsigned(6 downto 0) := (others => '0');
    signal s_hw_wdata   : std_logic_vector(15 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Glyph ROM flattened into a single-port BRAM (addr = glyph*16 + row).
    -- Built from the generated 2D constant so it initialises the EBR; one
    -- registered read per pixel keeps it to ~1.2 block RAMs instead of a large
    -- triplicated LUT-ROM.
    --------------------------------------------------------------------------
    constant C_GROM_DEPTH : integer := C_NGLYPH * 16;
    type t_grom is array (0 to C_GROM_DEPTH - 1) of std_logic_vector(15 downto 0);
    function build_grom return t_grom is
        variable r : t_grom;
    begin
        for g in 0 to C_NGLYPH - 1 loop
            for y in 0 to 15 loop
                r(g * 16 + y) := C_GLYPHS(g, y);
            end loop;
        end loop;
        return r;
    end function;
    signal s_grom    : t_grom := build_grom;
    attribute ram_style of s_grom : signal is "block";

    --------------------------------------------------------------------------
    -- Per-frame head update FSM + global LFSR.
    --------------------------------------------------------------------------
    -- Read latency to s_head_q is 3 cycles: s_upd_raddr -> registered raddr mux
    -- in p_acc -> registered BRAM output.  Hence two wait states before U_USE.
    type t_upd is (U_IDLE, U_REQ, U_W1, U_W2, U_USE);
    signal s_ustate : t_upd := U_IDLE;
    signal s_ucnt   : unsigned(7 downto 0) := (others => '0');
    signal s_busy   : std_logic := '0';
    signal s_upd_raddr : unsigned(6 downto 0) := (others => '0');
    signal s_lfsr   : std_logic_vector(15 downto 0) := x"ACE1";

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    -- S0
    signal s0_col  : unsigned(6 downto 0) := (others => '0');
    signal s0_row  : unsigned(7 downto 0) := (others => '0');
    signal s0_gx, s0_gy : unsigned(3 downto 0) := (others => '0');
    signal s0_render : std_logic := '0';
    -- S1 (delay to align with head BRAM read)
    signal s1_col  : unsigned(6 downto 0) := (others => '0');
    signal s1_row  : unsigned(7 downto 0) := (others => '0');
    signal s1_gx, s1_gy : unsigned(3 downto 0) := (others => '0');
    signal s1_render : std_logic := '0';
    -- S2
    signal s2_base   : unsigned(15 downto 0) := (others => '0');
    signal s2_gx, s2_gy : unsigned(3 downto 0) := (others => '0');
    signal s2_lit    : std_logic := '0';
    signal s2_head   : std_logic := '0';
    signal s2_bidx   : integer range 0 to 15 := 0;
    signal s2_render : std_logic := '0';
    -- S3
    signal s3_idx  : integer range 0 to C_NGLYPH - 1 := 0;
    signal s3_gx, s3_gy : unsigned(3 downto 0) := (others => '0');
    signal s3_lit, s3_head, s3_render : std_logic := '0';
    signal s3_bidx : integer range 0 to 15 := 0;
    -- S4
    signal s4_rb_c : std_logic_vector(15 downto 0) := (others => '0');
    signal s4_gx   : unsigned(3 downto 0) := (others => '0');
    signal s4_lit, s4_head, s4_render : std_logic := '0';
    signal s4_bidx : integer range 0 to 15 := 0;
    -- S5
    signal s5_on, s5_glow, s5_lit, s5_head, s5_render : std_logic := '0';
    signal s5_bidx : integer range 0 to 15 := 0;
    -- S6
    signal s6_y, s6_u, s6_v : unsigned(9 downto 0) := (others => '0');
    signal s6_sync : std_logic_vector(4 downto 0) := (others => '0');

    -- sync passthrough pipeline: [field avid vsync hsync vparity]
    type t_syncp is array (0 to C_SYNCD) of std_logic_vector(4 downto 0);
    signal s_syncp : t_syncp := (others => (others => '0'));

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Control reads
    --------------------------------------------------------------------------
    s_speedknob  <= unsigned(registers_in(0));
    s_densknob   <= unsigned(registers_in(1));
    s_tailknob   <= unsigned(registers_in(2));
    s_mutknob    <= unsigned(registers_in(3));
    s_hueknob    <= unsigned(registers_in(4));
    s_brightknob <= unsigned(registers_in(5));
    s_glow_en    <= registers_in(6)(0);   -- S7
    s_mirror     <= registers_in(6)(1);   -- S8
    s_fade       <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Timing infrastructure (genlocked to the incoming sync).
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    pixel_counter_inst : entity work.pixel_counter
        port map (clk => clk, timing => s_timing,
                  h_count => s_h_count, v_count => s_v_count);

    p_measure_resolution : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then s_measured_h <= s_h_pixel_counter; end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then s_measured_v <= s_v_line_counter; end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    p_vsync_edge : process(clk)
    begin
        if rising_edge(clk) then
            s_vsync_prev <= s_timing.vsync_n;
            if s_vsync_prev = '0' and s_timing.vsync_n = '1' then
                s_vsync_pulse <= '1';
            else
                s_vsync_pulse <= '0';
            end if;
        end if;
    end process;

    p_firstline : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_firstline <= '1';
            elsif s_timing.avid_start = '1' then
                s_firstline <= '0';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame parameter latch (at vsync).  All shift-only.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_off  : signed(10 downto 0);
        variable v_du   : signed(7 downto 0);
        variable v_u, v_v : integer;
        variable v_z    : integer range 0 to 3;
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                -- Glyph size (P12): cell = 16 << zoom px (16/32/64/128).  Cell
                -- count scales with it; all derivations stay shift-only.
                v_z := to_integer(s_fade(9 downto 8));
                s_zoom  <= to_unsigned(v_z, 2);
                s_ncols <= resize(shift_right(s_measured_h, 4 + v_z), 8);
                s_nrows <= resize(shift_right(s_measured_v, 4 + v_z), 8);
                s_tail  <= resize(to_unsigned(2, 6) + s_tailknob(9 downto 5), 6);   -- 2..33
                s_dens8 <= s_densknob(9 downto 2);                                  -- 0..255
                s_basespd <= resize(to_unsigned(8, 9) + s_speedknob(9 downto 2), 9);-- 8..263
                -- mutation tick: 1..16 per frame.
                s_T <= s_T + (resize(s_mutknob(9 downto 6), 24) + 1);

                -- Hue/tint: rotate the green chroma a little around its centre.
                v_off := signed('0' & s_hueknob) - to_signed(512, 11);  -- -512..511
                v_du  := resize(shift_right(v_off, 4), 8);              -- ~ +-32
                v_u := to_integer(C_GRN_U) + to_integer(v_du);
                v_v := to_integer(C_GRN_V) - to_integer(v_du);
                if v_u < 0 then v_u := 0; elsif v_u > 1023 then v_u := 1023; end if;
                if v_v < 0 then v_v := 0; elsif v_v > 1023 then v_v := 1023; end if;
                s_body_u <= to_unsigned(v_u, 10);
                s_body_v <= to_unsigned(v_v, 10);
                -- Head whitens: chroma halfway to neutral, luma near max.
                s_head_u <= resize(shift_right(to_unsigned(v_u, 11) + resize(C_MID, 11), 1), 10);
                s_head_v <= resize(shift_right(to_unsigned(v_v, 11) + resize(C_MID, 11), 1), 10);

                s_bright3 <= s_brightknob(9 downto 7);

                -- Background is plain black (P12 now controls glyph size above).
                s_bg_y <= (others => '0');
                s_bg_u <= C_MID;
                s_bg_v <= C_MID;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Free-running LFSR (16-bit Fibonacci, taps 16,14,13,11).
    --------------------------------------------------------------------------
    p_lfsr : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            v_fb := s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10);
            s_lfsr <= s_lfsr(14 downto 0) & v_fb;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame head update: walk columns 0..ncols-1, head += speed[col];
    -- when the head + full tail have scrolled off the bottom, respawn it a
    -- randomized few rows above the top.  Runs entirely within vblank.
    -- Speed[col] = basespd + per-column hash jitter (shift-only).
    --------------------------------------------------------------------------
    p_headupd : process(clk)
        variable v_old   : signed(15 downto 0);
        variable v_int   : signed(9 downto 0);
        variable v_spd   : unsigned(8 downto 0);
        variable v_jit   : unsigned(15 downto 0);
        variable v_new   : signed(15 downto 0);
        variable v_limit : signed(9 downto 0);
        variable v_gap   : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            s_hw_we <= '0';
            case s_ustate is
                when U_IDLE =>
                    if s_vsync_pulse = '1' then
                        s_ucnt   <= (others => '0');
                        s_busy   <= '1';
                        s_ustate <= U_REQ;
                    end if;
                when U_REQ =>
                    -- present the read address for column s_ucnt (mux'd into the
                    -- BRAM read port via p_acc while s_busy = '1').
                    s_upd_raddr <= s_ucnt(6 downto 0);
                    s_ustate    <= U_W1;
                when U_W1 =>
                    s_ustate <= U_W2;           -- raddr mux settles
                when U_W2 =>
                    s_ustate <= U_USE;          -- BRAM data valid next cycle
                when U_USE =>
                    v_old := signed(s_head_q);
                    -- per-column speed
                    v_jit := mix16(resize(s_ucnt, 16) xor x"A5A5");
                    v_spd := s_basespd + resize(v_jit(4 downto 0), 9);
                    v_new := v_old + signed(resize(v_spd, 16));
                    v_int := v_new(15 downto 6);
                    v_limit := signed(resize(s_nrows, 10)) + signed(resize(s_tail, 10));
                    if v_int > v_limit then
                        -- respawn a random 0..15 rows above the top edge.
                        v_gap := signed(resize(unsigned(s_lfsr(3 downto 0)), 10));
                        v_new := to_signed(0, 16) - (v_gap & "000000");
                    end if;
                    s_hw_waddr <= s_ucnt(6 downto 0);
                    s_hw_wdata <= std_logic_vector(v_new);
                    s_hw_we    <= '1';
                    if s_ucnt + 1 >= s_ncols then
                        s_busy   <= '0';
                        s_ustate <= U_IDLE;
                    else
                        s_ucnt   <= s_ucnt + 1;
                        s_ustate <= U_REQ;
                    end if;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Head BRAM: 1 write port (update) + 1 registered read port (render/update).
    --------------------------------------------------------------------------
    p_head_ram : process(clk)
    begin
        if rising_edge(clk) then
            if s_hw_we = '1' then
                s_head_mem(to_integer(s_hw_waddr)) <= s_hw_wdata;
            end if;
            s_head_q <= s_head_mem(to_integer(s_head_raddr));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sync passthrough delay line.
    --------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_syncp(0) <= s_v_count(0) & data_in.field_n & data_in.avid &
                          data_in.vsync_n & data_in.hsync_n;
            for k in 1 to C_SYNCD loop
                s_syncp(k) <= s_syncp(k - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S0: pixel/cell coordinates (power-of-two grid => plain bit slicing).
    -- Also drives the head-BRAM read address (mux'd with the update walk).
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable cur_x, cur_y : unsigned(11 downto 0);
        variable v_col : unsigned(6 downto 0);
        variable v_gx, v_gy : unsigned(3 downto 0);
        variable v_row : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.avid_start = '1' then
                cur_x := (others => '0');
                if s_firstline = '1' then cur_y := (others => '0');
                else                      cur_y := s_y + 1; end if;
                s_x <= to_unsigned(1, 12);
                s_y <= cur_y;
            elsif s_timing.avid = '1' then
                cur_x := s_x; cur_y := s_y;
                s_x <= s_x + 1;
            else
                cur_x := s_x; cur_y := s_y;
            end if;

            -- Cell = 16 << zoom px.  col = x>>(4+z); glyph pixel = (x>>z) mod 16
            -- (nearest-neighbour upscale of the 16x16 bitmap).  Pure bit-slicing.
            case to_integer(s_zoom) is
                when 0 =>
                    v_col := resize(cur_x(11 downto 4), 7); v_gx := cur_x(3 downto 0);
                    v_row := resize(cur_y(11 downto 4), 8); v_gy := cur_y(3 downto 0);
                when 1 =>
                    v_col := resize(cur_x(11 downto 5), 7); v_gx := cur_x(4 downto 1);
                    v_row := resize(cur_y(11 downto 5), 8); v_gy := cur_y(4 downto 1);
                when 2 =>
                    v_col := resize(cur_x(11 downto 6), 7); v_gx := cur_x(5 downto 2);
                    v_row := resize(cur_y(11 downto 6), 8); v_gy := cur_y(5 downto 2);
                when others =>
                    v_col := resize(cur_x(11 downto 7), 7); v_gx := cur_x(6 downto 3);
                    v_row := resize(cur_y(11 downto 7), 8); v_gy := cur_y(6 downto 3);
            end case;

            s0_col <= v_col; s0_gx <= v_gx;
            s0_row <= v_row; s0_gy <= v_gy;
            s0_render <= s_timing.avid or s_timing.avid_start;

            if s_busy = '1' then
                s_head_raddr <= s_upd_raddr;
            else
                s_head_raddr <= v_col;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1: delay so the head BRAM output lines up with the cell coordinates.
    --------------------------------------------------------------------------
    p_s1 : process(clk)
    begin
        if rising_edge(clk) then
            s1_col <= s0_col; s1_row <= s0_row;
            s1_gx <= s0_gx;   s1_gy <= s0_gy;
            s1_render <= s0_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2: head distance -> lit / head flags + brightness index; column active
    -- (density) test; per-cell hash base for the glyph picker.
    --------------------------------------------------------------------------
    p_dist : process(clk)
        variable v_int   : signed(9 downto 0);
        variable v_dist  : signed(10 downto 0);
        variable v_active : std_logic;
        variable v_dh    : unsigned(15 downto 0);
        variable v_d     : integer;
    begin
        if rising_edge(clk) then
            v_int  := signed(s_head_q(15 downto 6));
            v_dist := resize(v_int, 11) - signed(resize(s1_row, 11));

            -- density: column active if its hash falls under the threshold.
            v_dh := mix16(resize(s1_col, 16) xor x"3C3C");
            if unsigned(v_dh(7 downto 0)) < s_dens8 then v_active := '1';
            else v_active := '0'; end if;

            if (v_active = '1') and (v_dist >= 0) and
               (v_dist < signed(resize(s_tail, 11))) then
                s2_lit <= '1';
            else
                s2_lit <= '0';
            end if;
            if v_dist = 0 then s2_head <= '1'; else s2_head <= '0'; end if;

            v_d := to_integer(v_dist);
            if v_d < 0 then v_d := 0; elsif v_d > 15 then v_d := 15; end if;
            s2_bidx <= v_d;

            s2_base <= mix16(shift_left(resize(s1_col, 16), 7) + resize(s1_row, 16));
            s2_gx <= s1_gx; s2_gy <= s1_gy;
            s2_render <= s1_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S3: glyph mutation -> glyph index.  Per-cell phase staggers the re-roll
    -- so glyphs flip at different times; index reduced to 0..C_NGLYPH-1 with a
    -- single conditional subtract (no divider).
    --------------------------------------------------------------------------
    p_idx : process(clk)
        variable v_gen : unsigned(23 downto 0);
        variable v_h   : unsigned(15 downto 0);
        variable v_i   : integer range 0 to 127;
    begin
        if rising_edge(clk) then
            v_gen := shift_right(s_T + resize(s2_base(7 downto 0), 24), C_MUT_RATE);
            v_h   := mix16(s2_base xor (v_gen(7 downto 0) & v_gen(7 downto 0)) xor x"7E5A");
            v_i   := to_integer(v_h(6 downto 0));     -- 0..127
            if v_i >= C_NGLYPH then v_i := v_i - C_NGLYPH; end if;
            s3_idx <= v_i;
            s3_gx <= s2_gx; s3_gy <= s2_gy;
            s3_lit <= s2_lit; s3_head <= s2_head; s3_render <= s2_render;
            s3_bidx <= s2_bidx;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S4: glyph ROM read (single registered BRAM port: addr = idx*16 + gy).
    -- The *16 is a constant power-of-two shift, so no real multiplier.
    --------------------------------------------------------------------------
    p_rom : process(clk)
        variable v_addr : integer range 0 to C_GROM_DEPTH - 1;
    begin
        if rising_edge(clk) then
            v_addr := s3_idx * 16 + to_integer(s3_gy);
            s4_rb_c <= s_grom(v_addr);
            s4_gx <= s3_gx;
            s4_lit <= s3_lit; s4_head <= s3_head; s4_render <= s3_render;
            s4_bidx <= s3_bidx;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S5: glyph bit extract (+ horizontal mirror) + 1px horizontal head glow.
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable rbc : std_logic_vector(15 downto 0);
        variable p   : integer range -1 to 16;
        variable v_center, v_glow : std_logic;
    begin
        if rising_edge(clk) then
            if s_mirror = '1' then rbc := revb(s4_rb_c); else rbc := s4_rb_c; end if;
            p := to_integer(s4_gx);
            v_center := getbit(rbc, p) and s4_lit;
            v_glow := ( getbit(rbc, p-1) or getbit(rbc, p+1) )
                      and s4_lit and s4_head and (not v_center);
            s5_on   <= v_center;
            s5_glow <= v_glow;
            s5_lit  <= s4_lit;
            s5_head <= s4_head;
            s5_render <= s4_render;
            s5_bidx <= s4_bidx;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S6: final colour mux.
    --   head cell ON  -> bright near-white green (luma ~max, whitened chroma)
    --   body cell ON  -> green, luma = fade-curve[dist] scaled by brightness
    --   head glow     -> dim head green halo
    --   else          -> background (black, or P12 dim green wash)
    --------------------------------------------------------------------------
    p_color : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_luma : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s5_on = '1' then
                if s5_head = '1' then
                    v_y := scale8(to_unsigned(1000, 10), s_bright3);
                    v_u := s_head_u; v_v := s_head_v;
                else
                    v_luma := C_FADE(s5_bidx);
                    v_y := scale8(v_luma, s_bright3);
                    v_u := s_body_u; v_v := s_body_v;
                end if;
            elsif s5_glow = '1' and s_glow_en = '1' then
                v_y := scale8(shift_right(to_unsigned(1000, 10), 1), s_bright3);
                v_u := s_head_u; v_v := s_head_v;
            else
                v_y := s_bg_y; v_u := s_bg_u; v_v := s_bg_v;
            end if;

            s6_y <= v_y; s6_u <= v_u; s6_v <= v_v;
            s6_sync <= s_syncp(C_SYNCD);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output registers.
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io.y       <= std_logic_vector(s6_y);
            s_io.u       <= std_logic_vector(s6_u);
            s_io.v       <= std_logic_vector(s6_v);
            s_io.hsync_n <= s6_sync(0);
            s_io.vsync_n <= s6_sync(1);
            s_io.avid    <= s6_sync(2);
            s_io.field_n <= s6_sync(3);
        end if;
    end process;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture matrix_rain;
