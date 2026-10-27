-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Lineage v0.1.0: catalogue combo (2026-10-20), the first one drawn only from
-- catalogue effects no earlier combo used -- Schmitt-trigger posterize (the
-- "hysteresis-edge state glitch") + a cellular automaton seeded by the video.
--
-- The picture is posterized with hysteresis: a held level only lets go once
-- the luma has moved past it by more than the band, so edges smear into
-- streaks and noise settles into stable runs.  Every place the held level
-- jumps plants a seed in a 1-D (Wolfram) automaton whose generations run down
-- the screen, one row of cells per generation.  The seeds grow by the chosen
-- rule (Sierpinski, Rule 30 chaos, Rule 110 gliders, floods), carrying the
-- colour of the run that spawned them, until their lifetime runs out.
--
--   K1 Levels    posterize levels 2 .. 16 (continuous step)
--   K2 Hyst      hysteresis band: 0 = plain posterize .. lines frozen solid
--   K3 Burst     rolling bands of extra hysteresis (stick and release)
--   K4 Rule      16 elementary CA rules (labelled)
--   K5 Cells     CA cell size, 2 .. 128 px on the 1080-line canvas (log)
--   K6 Ink       living cells: Carry (seed colour) / Invert (negative of the
--                picture) / Lit (seed colour, bright) / Shadow (dark)
--   S7  Smear    Across: the Schmitt state runs along each line / Down: it
--                runs down each column (streaks fall, seeds at the top edges)
--   S8  Chroma   Held (colour sampled at each jump) / Live
--   S9  Seeds    Edges (every jump seeds) / Bands (odd levels seed, solid)
--   S10 Grout    dark gaps round the living cells (a tiled chart)
--   S11 Mask     Free / Lit: cells only live over the brighter half
--   P12 Reach    lineage lifetime, glided, squared: 0% = fringes ~24 lines
--                long, 100% = immortal (the automaton takes the frame)
--
-- Schmitt stage.  A 512-entry LUT (rebuilt in vblank) gives each luma code
-- its level {index, value}.  Across: one loop register pair (hi, lo) =
-- value +- T; a pixel outside [lo, hi] reloads the state (a "jump").  T =
-- step/2 + K2 band + the burst band of this line.  Down: the state lives in
-- a 2048 x 15 column RAM {index, u5, v5, row parity}, read-modify-write once
-- per pixel (each column once per line, so no hazard).
--
-- Automaton.  1024 x 32 cell RAM {life 11, y8, u6, v6, parity}; alive =
-- life /= 0.  Cell geometry runs in virtual 1080-line units (square cells at
-- every mode).  ONE bank: cell c+1 is read at the start of cell c (window
-- l / c / r), and cell c is written after it ends, so every read precedes
-- its own overwrite.  The row's first line is a generation line: new state =
-- rule(l, c, r) (needs a living parent), life = max(parent lives) - cell
-- height (infinite at the top of P12), colour from the strongest parent.
-- Every line also writes seeds into the current row (life = reach, colour of
-- the held run at the cell's end), so 1-line horizontal edges seed too.
-- The first line of a field reads all-dead, so nothing carries over frames.
-- The displayed generation lags one line (invisible; Grout covers it).
--
-- Stored colours replay on other rows, so their hsync->avid parity is kept
-- and U/V swapped where it differs from the current row's (drape recipe).
--
-- Hardware contracts: vblank zone (no avid for 1024 clocks) drives every
-- per-field event, no vsync dependence; output conditioning (luma [1 2 1],
-- chroma [1 2 2 2 2 2 2 2 1]/16, luma scaled to white ~777); blanking-gated;
-- no_rw_check on every RAM (all collision-free).  Latency 20 clocks, all
-- modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lineage of program_top is
    -- data_in -> data_out latency (multiple of 4); matches processing_delay_clks.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;
    constant C_LAT   : integer := 20;
    constant C_FORCE : integer := 16;   -- line-end columns that always reload the Schmitt state
    constant C_SEEDX : integer := 24;   -- no seeds this close to either end (capture blanking columns)

    -- K1: step (Q4 luma codes) for 2 + 14 j / 63 levels
    type t_step is array (0 to 63) of integer range 0 to 16383;
    constant C_STEP : t_step := (
        14016, 11468, 9703, 8410, 7420, 6639, 6007, 5485, 5046, 4672, 4350, 4069, 3823, 3604, 3409, 3234,
        3077, 2934, 2803, 2684, 2574, 2473, 2380, 2294, 2213, 2138, 2068, 2002, 1941, 1883, 1828, 1777,
        1728, 1682, 1638, 1597, 1557, 1520, 1484, 1450, 1417, 1386, 1356, 1328, 1300, 1274, 1249, 1225,
        1201, 1179, 1157, 1136, 1116, 1097, 1078, 1060, 1043, 1026, 1009, 993, 978, 963, 948, 934);
    -- K5: cell size in virtual (1080-line) pixels, 2 .. 128 log
    type t_cell is array (0 to 63) of integer range 0 to 255;
    constant C_CELL : t_cell := (
        2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 6, 6, 7, 7, 7, 8, 9, 9, 10, 10, 11, 12, 13, 14, 14, 15,
        17, 18, 19, 20, 22, 23, 25, 26, 28, 30, 32, 34, 37, 39, 42, 45, 48, 51, 54, 58, 62, 66, 71, 75, 81, 86,
        92, 98, 105, 112, 120, 128);
    -- K4: elementary rules (bit (l c r) = next state); all even, so an empty
    -- neighbourhood never ignites
    type t_rule is array (0 to 15) of integer range 0 to 255;
    constant C_RULE : t_rule := (90, 18, 146, 60, 102, 150, 182, 126, 22, 54, 110, 124, 106, 30, 250, 254);

    -- x step per pixel (Q.1 virtual units: 2 / 3 / 4 = 1080 / 720 / SD)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- y step in Q.8 virtual units per FIELD line
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);  -- 480i
            when 4          => return to_unsigned(576, 12);   -- 480p
            when 8          => return to_unsigned(960, 12);   -- 576i
            when 12         => return to_unsigned(480, 12);   -- 576p
            when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p
            when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i
            when others     => return to_unsigned(256, 12);   -- 1080p
        end case;
    end function;

    -- chroma to 5 / 6 bits, rounded, about 512 (codes 16 / 32 = neutral)
    function f_q5(u : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(10 downto 0);
    begin
        v := resize(u, 11) + 16;
        if v(10) = '1' then return to_unsigned(31, 5); end if;
        return v(9 downto 5);
    end function;
    function f_q6(u : unsigned(9 downto 0)) return unsigned is
        variable v : unsigned(10 downto 0);
    begin
        v := resize(u, 11) + 8;
        if v(10) = '1' then return to_unsigned(63, 6); end if;
        return v(9 downto 4);
    end function;

    -- cell word: life(31:21) y8(20:13) u6(12:7) v6(6:1) parity(0)
    constant C_DEAD : std_logic_vector(31 downto 0) :=
        "00000000000" & "00010000" & "100000" & "100000" & "0";
    function f_life(w : std_logic_vector(31 downto 0)) return unsigned is
    begin
        return unsigned(w(31 downto 21));
    end function;

    -- RAMs
    type t_lut is array (0 to 511) of std_logic_vector(13 downto 0);       -- {idx 4, value 10}
    type t_col is array (0 to 2047) of std_logic_vector(14 downto 0);      -- {idx 4, u5, v5, parity}
    type t_ca  is array (0 to 1023) of std_logic_vector(31 downto 0);
    signal lut    : t_lut := (others => "0000" & "0001000000");
    signal colram : t_col := (others => "0000" & "10000" & "10000" & "0");
    signal caram  : t_ca  := (others => C_DEAD);
    attribute no_rw_check : boolean;
    attribute no_rw_check of lut, colram, caram : signal is true;

    signal s_lut_ra : unsigned(8 downto 0) := (others => '0');
    signal s_lut_q  : std_logic_vector(13 downto 0) := "0000" & "0001000000";
    signal s_lut_wa : unsigned(8 downto 0) := (others => '0');
    signal s_lut_wd : std_logic_vector(13 downto 0) := (others => '0');
    signal s_lut_we : std_logic := '0';
    signal s_col_ra, s_col_wa : unsigned(10 downto 0) := (others => '0');
    signal s_col_q  : std_logic_vector(14 downto 0) := "0000" & "10000" & "10000" & "0";
    signal s_col_wd : std_logic_vector(14 downto 0) := (others => '0');
    signal s_col_we : std_logic := '0';
    signal s_ca_ra, s_ca_wa : unsigned(9 downto 0) := (others => '0');
    signal s_ca_q   : std_logic_vector(31 downto 0) := C_DEAD;
    signal s_ca_wd  : std_logic_vector(31 downto 0) := C_DEAD;
    signal s_ca_we  : std_logic := '0';

    -- level table (index -> value) for the Down state
    type t_tab is array (0 to 15) of unsigned(9 downto 0);
    signal s_tab : t_tab := (others => to_unsigned(64, 10));
    signal s_tab_wi : unsigned(3 downto 0) := (others => '0');
    signal s_tab_wd : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_tab_we : std_logic := '0';

    -- LUT builder
    signal s_bi, s_bwa : unsigned(8 downto 0) := (others => '0');
    signal s_bwe : std_logic := '0';
    signal s_bS : unsigned(13 downto 0) := (others => '0');
    signal s_bidx : unsigned(3 downto 0) := (others => '0');
    signal s_bval, s_bthr : unsigned(17 downto 0) := (others => '0');

    -- controls
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_j1 : unsigned(5 downto 0) := to_unsigned(18, 6);
    signal s_step : unsigned(13 downto 0) := to_unsigned(2803, 14);
    signal s_k2h : unsigned(5 downto 0) := (others => '0');
    signal s_hsqr : unsigned(11 downto 0) := (others => '0');
    signal s_hy : unsigned(9 downto 0) := (others => '0');
    signal s_ba : unsigned(9 downto 0) := (others => '0');
    signal s_live4, s_live6 : std_logic := '0';
    signal s_jr : unsigned(3 downto 0) := (others => '0');
    signal s_rulek : unsigned(7 downto 0) := to_unsigned(90, 8);
    signal s_j5 : unsigned(5 downto 0) := to_unsigned(21, 6);
    signal s_zk : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal s_inkk : unsigned(1 downto 0) := "00";
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff, s_gd2, s_gd3 : signed(12 downto 0) := (others => '0');
    signal s_g, s_g2, s_g3 : unsigned(11 downto 0) := (others => '0');   -- glided P12 / K2 / K3, Q.2
    -- field-latched
    signal s_z : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal s_rule : std_logic_vector(7 downto 0) := x"5A";
    signal s_ink : unsigned(1 downto 0) := "00";
    signal s_down, s_live, s_bands, s_grout, s_mask : std_logic := '0';
    signal s_roll : unsigned(8 downto 0) := (others => '0');
    signal s_w, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_ncells, s_ncmax : unsigned(10 downto 0) := (others => '0');
    -- derived (free-running from the latched values)
    signal s_z2 : unsigned(8 downto 0) := to_unsigned(16, 9);      -- 2Z, Q.1
    signal s_zy : unsigned(15 downto 0) := to_unsigned(2048, 16);  -- row height Q.8, >= one field line
    signal s_zdec : unsigned(7 downto 0) := to_unsigned(8, 8);     -- virtual lines per generation
    signal s_gxw : unsigned(5 downto 0) := to_unsigned(2, 6);      -- grout width, Q.1
    signal s_gyw : unsigned(12 downto 0) := to_unsigned(256, 13);  -- grout height, Q.8
    signal s_wf, s_wsx : unsigned(11 downto 0) := (others => '0');
    signal s_g6 : unsigned(5 downto 0) := (others => '0');
    signal s_g6sq : unsigned(11 downto 0) := (others => '0');
    signal s_reach : unsigned(10 downto 0) := to_unsigned(24, 11);
    signal s_inf : std_logic := '0';
    -- per-line hysteresis T (burst band)
    signal s_bph : unsigned(8 downto 0) := (others => '0');
    signal s_tri4 : unsigned(9 downto 0) := (others => '0');
    signal s_hb : unsigned(9 downto 0) := (others => '0');
    signal s_t : unsigned(9 downto 0) := (others => '0');

    -- line
    signal s_hsq, s_ph, s_rpar : std_logic := '0';
    signal s_cy : unsigned(15 downto 0) := (others => '0');
    signal s_vy : unsigned(19 downto 0) := (others => '0');
    signal s_gen, s_gen_n, s_gyl, s_first, s_first_n : std_logic := '0';

    -- pixel path (suffix = stage)
    signal s_y1, s_y2, s_y3, s_y4, s_y5, s_y6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ud, s_vd : t_u10(1 to 7) := (others => to_unsigned(512, 10));
    signal s_rx1 : unsigned(11 downto 0) := (others => '0');
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    signal s_rxd : t_u11(2 to 6) := (others => (others => '0'));
    signal s_cx1 : unsigned(8 downto 0) := (others => '0');
    signal s_c1 : unsigned(9 downto 0) := (others => '0');
    type t_c is array (natural range <>) of unsigned(9 downto 0);
    signal s_cd : t_c(2 to 9) := (others => (others => '0'));
    type t_sl is array (natural range <>) of std_logic;
    signal s_cs : t_sl(1 to 8) := (others => '0');
    signal s_gx : t_sl(2 to 9) := (others => '0');
    signal s_bey : t_sl(2 to 8) := (others => '0');
    signal s_fz : t_sl(2 to 3) := (others => '0');
    signal s_sx : t_sl(2 to 7) := (others => '0');
    -- Schmitt
    signal s_lq3 : std_logic_vector(13 downto 0) := "0000" & "0001000000";
    signal s_qidx4, s_qidx5, s_qidx6 : unsigned(3 downto 0) := (others => '0');
    signal s_qval4, s_qval5, s_qval6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_qhi4, s_qlo4 : signed(11 downto 0) := (others => '0');
    signal s_uq4, s_vq4, s_uq5, s_vq5, s_uq6, s_vq6 : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_fa4, s_fd4, s_fd5, s_fd6 : std_logic := '1';
    signal s_cq4 : std_logic_vector(14 downto 0) := "0000" & "10000" & "10000" & "0";
    -- Across loop state (stage 5) and its delays
    signal s_aidx : unsigned(3 downto 0) := (others => '0');
    signal s_aval : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ahi, s_alo : signed(11 downto 0) := (others => '0');
    signal s_au, s_av : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_aj5, s_aj6, s_aj7 : std_logic := '0';
    signal s_aidx6, s_aidx7 : unsigned(3 downto 0) := (others => '0');
    signal s_aval6, s_aval7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_au6, s_av6, s_au7, s_av7 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- Down path
    signal s_didx5, s_didx6 : unsigned(3 downto 0) := (others => '0');
    signal s_dval5, s_dval6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_du5, s_dv5, s_du6, s_dv6 : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_dpar5, s_dpar6 : std_logic := '0';
    signal s_dhi6, s_dlo6 : signed(11 downto 0) := (others => '0');
    signal s_nidx7 : unsigned(3 downto 0) := (others => '0');
    signal s_nval7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_nu7, s_nv7 : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_npar7, s_nj7 : std_logic := '0';
    -- held state (stage 8, 9)
    signal s_hval8, s_hval9 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_hidx8 : unsigned(3 downto 0) := (others => '0');
    signal s_bu8, s_bv8, s_bu9, s_bv9 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bpar8, s_bpar9 : std_logic := '0';
    signal s_seed8 : std_logic := '0';
    -- automaton window (stage 9) and generation pipe
    signal s_ca_d8, s_ca_dp : std_logic_vector(31 downto 0) := C_DEAD;
    signal s_wm1, s_w0, s_wp1 : std_logic_vector(31 downto 0) := C_DEAD;
    signal s_sacc : std_logic := '0';
    signal s_lcol : std_logic_vector(20 downto 0) := "00010000" & "100000" & "100000" & "0";
    signal s_lmask : std_logic := '0';
    signal s_gf9, s_gf10, s_gf11, s_gf12 : std_logic := '0';
    signal s_gl9, s_gc9, s_gr9, s_gc10, s_gc11, s_gc12 : std_logic_vector(31 downto 0) := C_DEAD;
    signal s_gm10, s_gm11 : std_logic_vector(31 downto 0) := C_DEAD;
    signal s_gs9, s_gs10, s_gs11, s_gs12 : std_logic := '0';
    signal s_gk9, s_gk10, s_gk11, s_gk12 : std_logic := '0';
    signal s_grb10, s_grb11 : std_logic := '0';
    signal s_gcol9, s_gcol10, s_gcol11, s_gcol12 : std_logic_vector(20 downto 0) := (others => '0');
    signal s_gmc12 : std_logic_vector(20 downto 0) := (others => '0');
    signal s_gwa9, s_gwa10, s_gwa11, s_gwa12 : unsigned(9 downto 0) := (others => '0');
    signal s_glife12 : unsigned(10 downto 0) := (others => '0');
    signal s_gok12 : std_logic := '0';
    -- compose
    signal s_al10, s_gr10, s_ipar10, s_bpar10 : std_logic := '0';
    signal s_iy10, s_by10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_iu10, s_iv10, s_bu10, s_bv10 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y11, s_y12, s_y13, s_y14, s_y15, s_y16, s_y17x : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_yf17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');   -- s_avid_sr(k) = avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- RAMs: one write statement, registered read, no conditional read
    ---------------------------------------------------------------------------
    p_lut : process(clk)
    begin
        if rising_edge(clk) then
            s_lut_q <= lut(to_integer(s_lut_ra));
            if s_lut_we = '1' then lut(to_integer(s_lut_wa)) <= s_lut_wd; end if;
        end if;
    end process p_lut;

    p_col : process(clk)
    begin
        if rising_edge(clk) then
            s_col_q <= colram(to_integer(s_col_ra));
            if s_col_we = '1' then colram(to_integer(s_col_wa)) <= s_col_wd; end if;
        end if;
    end process p_col;

    p_ca : process(clk)
    begin
        if rising_edge(clk) then
            s_ca_q <= caram(to_integer(s_ca_ra));
            if s_ca_we = '1' then caram(to_integer(s_ca_wa)) <= s_ca_wd; end if;
        end if;
    end process p_ca;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, per-line hysteresis
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k  : unsigned(9 downto 0);
        variable v_g  : signed(12 downto 0);
        variable v_b  : signed(11 downto 0);
        variable v_t  : unsigned(11 downto 0);
        variable v_zy : unsigned(15 downto 0);
        variable v_ph : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            -- vblank zone: no avid for 1024 clocks (hblank is at most 370)
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- K1 Levels (64 zones, +-3 count deadband after warm-up)
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(3 downto 0) >= 3 or v_k(9 downto 4) = 0) and
                                (v_k(3 downto 0) <= 12 or v_k(9 downto 4) = 63)) then
                s_j1 <= v_k(9 downto 4);
            end if;
            s_step <= to_unsigned(C_STEP(to_integer(s_j1)), 14);
            -- K2 Hyst (glided): squared, 0 .. 992 codes
            s_gd2 <= signed(resize(unsigned(registers_in(1)) & "00", 13)) - signed(resize(s_g2, 13));
            s_k2h <= s_g2(11 downto 6);
            s_hsqr <= s_k2h * s_k2h;
            s_hy <= s_hsqr(11 downto 2);
            -- K3 Burst (glided)
            s_gd3 <= signed(resize(unsigned(registers_in(2)) & "00", 13)) - signed(resize(s_g3, 13));
            s_ba <= s_g3(11 downto 2);
            -- K4 Rule (16 labels): the firmware loads the default as the label
            -- INDEX; honour it until the knob first reads a real position
            v_k := unsigned(registers_in(3));
            if v_k >= 16 then s_live4 <= '1'; end if;
            if s_live4 = '0' and v_k < 16 then
                s_jr <= v_k(3 downto 0);
            elsif s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                   (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_jr <= v_k(9 downto 6);
            end if;
            s_rulek <= to_unsigned(C_RULE(to_integer(s_jr)), 8);
            -- K5 Cells
            v_k := unsigned(registers_in(4));
            if s_warm /= 15 or ((v_k(3 downto 0) >= 3 or v_k(9 downto 4) = 0) and
                                (v_k(3 downto 0) <= 12 or v_k(9 downto 4) = 63)) then
                s_j5 <= v_k(9 downto 4);
            end if;
            s_zk <= to_unsigned(C_CELL(to_integer(s_j5)), 8);
            -- K6 Ink (4 labels), same index rule
            v_k := unsigned(registers_in(5));
            if v_k >= 4 then s_live6 <= '1'; end if;
            if s_live6 = '0' and v_k < 4 then
                s_inkk <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_inkk <= v_k(9 downto 8);
            end if;
            -- P12 Reach glide (1/8 of the gap a field)
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            -- per field (zone entry: exactly once per vertical blank)
            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                s_roll <= s_roll + 3;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                v_g := signed(resize(s_g2, 13)) + shift_right(s_gd2, 3);
                s_g2 <= unsigned(v_g(11 downto 0));
                v_g := signed(resize(s_g3, 13)) + shift_right(s_gd3, 3);
                s_g3 <= unsigned(v_g(11 downto 0));
                s_z <= s_zk;
                s_rule <= std_logic_vector(s_rulek);
                s_ink <= s_inkk;
                s_down  <= registers_in(6)(0);
                s_live  <= registers_in(6)(1);
                s_bands <= registers_in(6)(2);
                s_grout <= registers_in(6)(3);
                s_mask  <= registers_in(6)(4);
            end if;

            -- derived geometry (settles within a few clocks of the latch)
            s_z2 <= s_z & '0';
            v_zy := shift_left(resize(s_z, 16), 8);
            if v_zy < s_ys then v_zy := resize(s_ys, 16); end if;
            s_zy <= v_zy;
            s_zdec <= s_zy(15 downto 8);
            if resize(s_z(7 downto 2), 6) < s_xs then s_gxw <= resize(s_xs, 6); else s_gxw <= s_z(7 downto 2); end if;
            if shift_left(resize(s_z, 13), 5) < s_ys then
                s_gyw <= resize(s_ys, 13);
            else
                s_gyw <= shift_left(resize(s_z, 13), 5);
            end if;
            s_wf  <= s_w - C_FORCE;
            s_wsx <= s_w - C_SEEDX;
            -- reach = 24 + (g/16)^2 / 2 virtual lines; top of the slider = immortal
            s_g6 <= s_g(11 downto 6);
            s_g6sq <= s_g6 * s_g6;
            s_reach <= to_unsigned(24, 11) + resize(s_g6sq(11 downto 1), 11);
            if s_g(11 downto 2) >= 1000 then s_inf <= '1'; else s_inf <= '0'; end if;

            -- hysteresis T for the coming line: step/2 + K2 band + burst band.
            -- Burst: a triangle over 512 virtual lines rolling ~3 lines a field;
            -- K3 widens the band (above 1023 - K3) and deepens it.
            v_ph := s_vy(16 downto 8) + s_roll;
            s_bph <= v_ph;
            if s_bph(8) = '1' then s_tri4 <= (not s_bph(7 downto 0)) & "00"; else s_tri4 <= s_bph(7 downto 0) & "00"; end if;
            v_b := signed(resize(s_tri4, 12)) + signed(resize(s_ba, 12)) - 1023;
            if v_b <= 0 then
                s_hb <= (others => '0');
            elsif v_b >= 512 then
                s_hb <= (others => '1');
            else
                s_hb <= unsigned(v_b(8 downto 0)) & '0';
            end if;
            v_t := resize(s_step(13 downto 5), 12) + resize(s_hy, 12) + resize(s_hb, 12);
            if v_t > 1023 then s_t <= (others => '1'); else s_t <= v_t(9 downto 0); end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Line events: hsync->avid parity, cell-row DDA (active lines only),
    -- width / cell-count maxima latched per field
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_n : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            s_hsq <= data_in.hsync_n;
            if s_hsq = '1' and data_in.hsync_n = '0' then s_ph <= '0'; else s_ph <= not s_ph; end if;

            if s_zone = '1' then
                s_cy <= (others => '0');
                s_vy <= (others => '0');
                s_gen_n <= '1';
                s_first_n <= '1';
            elsif data_in.avid = '1' and s_avid_sr(0) = '0' then
                s_rpar <= s_ph;
                s_gen <= s_gen_n;
                s_first <= s_first_n;
                s_first_n <= '0';
                if s_cy < s_gyw then s_gyl <= '1'; else s_gyl <= '0'; end if;
            elsif data_in.avid = '0' and s_avid_sr(0) = '1' then
                v_n := resize(s_cy, 17) + s_ys;
                if v_n >= s_zy then
                    s_cy <= resize(v_n - s_zy, 16);
                    s_gen_n <= '1';
                else
                    s_cy <= v_n(15 downto 0);
                    s_gen_n <= '0';
                end if;
                s_vy <= s_vy + s_ys;
            end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_ncells <= s_ncmax; s_ncmax <= (others => '0');
            elsif data_in.avid = '0' and s_avid_sr(0) = '1' then
                if s_rx1 + 1 > s_wmax then s_wmax <= s_rx1 + 1; end if;
                if resize(s_c1, 11) + 1 > s_ncmax then s_ncmax <= resize(s_c1, 11) + 1; end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Level LUT, rebuilt every vblank: entry i = level of luma code 2i+1.
    -- Levels sit at 64 + k * step (Q4), switching midway; capped at 940.
    ---------------------------------------------------------------------------
    p_build : process(clk)
        variable v_v : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' then
                s_bi <= s_bi + 1;
                s_bwa <= s_bi;
                s_bwe <= '1';
                if s_bi = 0 then
                    s_bS <= s_step;
                    s_bidx <= (others => '0');
                    s_bval <= to_unsigned(1024, 18);
                    s_bthr <= to_unsigned(1024, 18) + resize(s_step(13 downto 1), 18);
                elsif resize(s_bi & "10000", 18) >= s_bthr and s_bidx /= 15 then
                    s_bidx <= s_bidx + 1;
                    s_bval <= s_bval + s_bS;
                    s_bthr <= s_bthr + s_bS;
                end if;
            else
                s_bi <= (others => '0');
                s_bwe <= '0';
            end if;
            v_v := s_bval(17 downto 4);
            if v_v > 940 then v_v := to_unsigned(940, 14); end if;
            s_lut_wa <= s_bwa;
            s_lut_wd <= std_logic_vector(s_bidx) & std_logic_vector(v_v(9 downto 0));
            s_lut_we <= s_bwe and s_zone;
            s_tab_wi <= s_bidx;
            s_tab_wd <= v_v(9 downto 0);
            s_tab_we <= s_bwe and s_zone;
            if s_tab_we = '1' then s_tab(to_integer(s_tab_wi)) <= s_tab_wd; end if;
        end if;
    end process p_build;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n   : unsigned(9 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_d, v_dp : std_logic_vector(31 downto 0);
        variable v_fire : std_logic;
        variable v_l, v_c, v_r : std_logic;
        variable v_ri  : integer range 0 to 7;
        variable v_lf  : signed(11 downto 0);
        variable v_w   : std_logic_vector(31 downto 0);
        variable v_t   : unsigned(10 downto 0);
        variable v_iy  : unsigned(9 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: input; column; cell DDA (virtual x, Q.1)
            s_y1 <= unsigned(data_in.y);
            s_ud(1) <= unsigned(data_in.u);
            s_vd(1) <= unsigned(data_in.v);
            s_lut_ra <= unsigned(data_in.y(9 downto 1));
            s_cs(1) <= '0';
            if data_in.avid = '1' then
                if s_avid_sr(0) = '0' then
                    s_rx1 <= (others => '0');
                    s_cx1 <= (others => '0');
                    s_c1 <= (others => '0');
                    s_cs(1) <= '1';
                else
                    s_rx1 <= s_rx1 + 1;
                    v_n := resize(s_cx1, 10) + s_xs;
                    if v_n >= s_z2 then
                        s_cx1 <= resize(v_n - s_z2, 9);
                        s_cs(1) <= '1';
                        if s_c1 /= 1023 then s_c1 <= s_c1 + 1; end if;
                    else
                        s_cx1 <= v_n(8 downto 0);
                    end if;
                end if;
            end if;

            -- 2: column RAM address; grout x, beyond-the-last-cell, force / seed zones
            s_y2 <= s_y1;
            s_col_ra <= s_rx1(10 downto 0);
            s_rxd(2) <= s_rx1(10 downto 0);
            if s_cx1 < s_gxw then s_gx(2) <= '1'; else s_gx(2) <= '0'; end if;
            if resize(s_c1, 11) + 1 >= s_ncells then s_bey(2) <= '1'; else s_bey(2) <= '0'; end if;
            if s_rx1 < C_FORCE or s_rx1 >= s_wf then s_fz(2) <= '1'; else s_fz(2) <= '0'; end if;
            if s_rx1 >= C_SEEDX and s_rx1 < s_wsx then s_sx(2) <= '1'; else s_sx(2) <= '0'; end if;
            s_cd(2) <= s_c1;

            -- 3: LUT entry (fabric register)
            s_y3 <= s_y2;
            s_lq3 <= s_lut_q;
            s_fz(3) <= s_fz(2);

            -- 4: candidate level +- T; Down force; column state (fabric)
            s_y4 <= s_y3;
            s_qidx4 <= unsigned(s_lq3(13 downto 10));
            s_qval4 <= unsigned(s_lq3(9 downto 0));
            s_qhi4 <= signed(resize(unsigned(s_lq3(9 downto 0)), 12)) + signed(resize(s_t, 12));
            s_qlo4 <= signed(resize(unsigned(s_lq3(9 downto 0)), 12)) - signed(resize(s_t, 12));
            s_uq4 <= f_q5(s_ud(3));
            s_vq4 <= f_q5(s_vd(3));
            s_fa4 <= s_fz(3) or not s_avid_sr(2);
            s_fd4 <= s_fz(3) or not s_avid_sr(2) or s_first;
            s_cq4 <= s_col_q;

            -- 5: Across Schmitt loop -- reload when forced or when the luma
            --    leaves [held - T, held + T]
            v_y := signed(resize(s_y4, 12));
            s_aj5 <= '0';
            if s_fa4 = '1' or v_y > s_ahi or v_y < s_alo then
                s_aidx <= s_qidx4;
                s_aval <= s_qval4;
                s_ahi  <= s_qhi4;
                s_alo  <= s_qlo4;
                s_au   <= s_ud(4);
                s_av   <= s_vd(4);
                if s_fa4 = '0' and s_qidx4 /= s_aidx then s_aj5 <= '1'; end if;
            end if;
            --    Down: the column's held state
            s_didx5 <= unsigned(s_cq4(14 downto 11));
            s_dval5 <= s_tab(to_integer(unsigned(s_cq4(14 downto 11))));
            s_du5 <= unsigned(s_cq4(10 downto 6));
            s_dv5 <= unsigned(s_cq4(5 downto 1));
            s_dpar5 <= s_cq4(0);
            s_y5 <= s_y4;
            s_qidx5 <= s_qidx4; s_qval5 <= s_qval4;
            s_uq5 <= s_uq4; s_vq5 <= s_vq4;
            s_fd5 <= s_fd4;

            -- 6: Down band; delays
            s_dhi6 <= signed(resize(s_dval5, 12)) + signed(resize(s_t, 12));
            s_dlo6 <= signed(resize(s_dval5, 12)) - signed(resize(s_t, 12));
            s_didx6 <= s_didx5; s_dval6 <= s_dval5;
            s_du6 <= s_du5; s_dv6 <= s_dv5; s_dpar6 <= s_dpar5;
            s_y6 <= s_y5;
            s_qidx6 <= s_qidx5; s_qval6 <= s_qval5;
            s_uq6 <= s_uq5; s_vq6 <= s_vq5;
            s_fd6 <= s_fd5;
            s_aidx6 <= s_aidx; s_aval6 <= s_aval; s_au6 <= s_au; s_av6 <= s_av; s_aj6 <= s_aj5;

            -- 7: Down decision + write-back (each column once per line)
            v_y := signed(resize(s_y6, 12));
            if s_fd6 = '1' or v_y > s_dhi6 or v_y < s_dlo6 then
                s_nidx7 <= s_qidx6; s_nval7 <= s_qval6;
                s_nu7 <= s_uq6; s_nv7 <= s_vq6; s_npar7 <= s_rpar;
                if s_fd6 = '0' and s_qidx6 /= s_didx6 then s_nj7 <= '1'; else s_nj7 <= '0'; end if;
                s_col_wd <= std_logic_vector(s_qidx6) & std_logic_vector(s_uq6) & std_logic_vector(s_vq6) & s_rpar;
            else
                s_nidx7 <= s_didx6; s_nval7 <= s_dval6;
                s_nu7 <= s_du6; s_nv7 <= s_dv6; s_npar7 <= s_dpar6;
                s_nj7 <= '0';
                s_col_wd <= std_logic_vector(s_didx6) & std_logic_vector(s_du6) & std_logic_vector(s_dv6) & s_dpar6;
            end if;
            s_col_wa <= s_rxd(6);
            s_col_we <= s_avid_sr(5);
            s_aidx7 <= s_aidx6; s_aval7 <= s_aval6; s_au7 <= s_au6; s_av7 <= s_av6; s_aj7 <= s_aj6;

            -- 8: held state of the active mode; base chroma; seed bit
            if s_down = '1' then
                s_hval8 <= s_nval7; s_hidx8 <= s_nidx7;
            else
                s_hval8 <= s_aval7; s_hidx8 <= s_aidx7;
            end if;
            if s_live = '1' then
                s_bu8 <= s_ud(7); s_bv8 <= s_vd(7); s_bpar8 <= s_rpar;
            elsif s_down = '1' then
                s_bu8 <= s_nu7 & "00000"; s_bv8 <= s_nv7 & "00000"; s_bpar8 <= s_npar7;
            else
                s_bu8 <= s_au7; s_bv8 <= s_av7; s_bpar8 <= s_rpar;
            end if;
            if s_sx(7) = '0' or s_avid_sr(6) = '0' then
                s_seed8 <= '0';
            elsif s_bands = '1' then
                if s_down = '1' then s_seed8 <= s_nidx7(0); else s_seed8 <= s_aidx7(0); end if;
            else
                if s_down = '1' then s_seed8 <= s_nj7; else s_seed8 <= s_aj7; end if;
            end if;
            --    automaton read: address = cell + 1 at each cell start (stage 6),
            --    0 through blanking so cell 0 is waiting at the line start
            if s_avid_sr(4) = '0' then
                s_ca_ra <= (others => '0');
            elsif s_cs(5) = '1' then
                s_ca_ra <= s_cd(5) + 1;
            end if;
            s_ca_d8 <= s_ca_q;
            s_ca_dp <= s_ca_d8;

            -- 9: window shift at each cell start; snapshot the ended cell
            v_d := s_ca_d8;
            v_dp := s_ca_dp;
            if s_first = '1' then v_d := C_DEAD; v_dp := C_DEAD; end if;
            if s_bey(8) = '1' then v_d := C_DEAD; end if;
            v_fire := (s_cs(8) and s_avid_sr(7) and s_avid_sr(8)) or (not s_avid_sr(7) and s_avid_sr(8));
            s_gf9 <= v_fire;
            if v_fire = '1' then
                s_gl9 <= s_wm1; s_gc9 <= s_w0; s_gr9 <= s_wp1;
                s_gs9 <= s_sacc;
                s_gcol9 <= s_lcol;
                s_gk9 <= s_lmask or not s_mask;
                s_gwa9 <= s_cd(9);
            end if;
            if s_avid_sr(7) = '1' and s_avid_sr(8) = '0' then
                s_wm1 <= C_DEAD; s_w0 <= v_dp; s_wp1 <= v_d;
            elsif s_cs(8) = '1' then
                s_wm1 <= s_w0; s_w0 <= s_wp1; s_wp1 <= v_d;
            end if;
            if s_cs(8) = '1' then s_sacc <= s_seed8; else s_sacc <= s_sacc or s_seed8; end if;
            s_lcol <= std_logic_vector(s_hval8(9 downto 2)) & std_logic_vector(f_q6(s_bu8))
                      & std_logic_vector(f_q6(s_bv8)) & s_bpar8;
            if s_hval8 >= 500 then s_lmask <= '1'; else s_lmask <= '0'; end if;
            s_hval9 <= s_hval8; s_bu9 <= s_bu8; s_bv9 <= s_bv8; s_bpar9 <= s_bpar8;

            -- 10: generation A -- rule bit (needs a living parent); stronger of l / r
            v_l := '0'; v_c := '0'; v_r := '0';
            if f_life(s_gl9) /= 0 then v_l := '1'; end if;
            if f_life(s_gc9) /= 0 then v_c := '1'; end if;
            if f_life(s_gr9) /= 0 then v_r := '1'; end if;
            v_ri := to_integer(unsigned'(v_l & v_c & v_r));
            s_grb10 <= s_rule(v_ri) and (v_l or v_c or v_r);
            if f_life(s_gl9) >= f_life(s_gr9) then s_gm10 <= s_gl9; else s_gm10 <= s_gr9; end if;
            s_gc10 <= s_gc9; s_gs10 <= s_gs9; s_gcol10 <= s_gcol9; s_gk10 <= s_gk9;
            s_gwa10 <= s_gwa9; s_gf10 <= s_gf9;
            --     display: living cell?  ink candidates
            if f_life(s_w0) /= 0 then s_al10 <= '1'; else s_al10 <= '0'; end if;
            s_gr10 <= s_grout and (s_gx(9) or s_gyl);
            v_iy := unsigned(s_w0(20 downto 13)) & "00";
            case to_integer(s_ink) is
                when 0 =>       -- Carry
                    s_iy10 <= v_iy;
                    s_iu10 <= unsigned(s_w0(12 downto 7)) & "0000";
                    s_iv10 <= unsigned(s_w0(6 downto 1)) & "0000";
                    s_ipar10 <= s_w0(0);
                when 1 =>       -- Invert: negative of the picture under the cell
                    s_iy10 <= to_unsigned(1004, 10) - s_hval9;
                    s_iu10 <= not s_bu9;
                    s_iv10 <= not s_bv9;
                    s_ipar10 <= s_bpar9;
                when 2 =>       -- Lit
                    v_t := resize(v_iy, 11) + 896;
                    s_iy10 <= v_t(10 downto 1);
                    s_iu10 <= unsigned(s_w0(12 downto 7)) & "0000";
                    s_iv10 <= unsigned(s_w0(6 downto 1)) & "0000";
                    s_ipar10 <= s_w0(0);
                when others =>  -- Shadow
                    s_iy10 <= 64 + shift_right(v_iy - 64, 2);
                    s_iu10 <= ('0' & unsigned(s_w0(12 downto 7)) & "000") + 256;
                    s_iv10 <= ('0' & unsigned(s_w0(6 downto 1)) & "000") + 256;
                    s_ipar10 <= s_w0(0);
            end case;
            s_by10 <= s_hval9; s_bu10 <= s_bu9; s_bv10 <= s_bv9; s_bpar10 <= s_bpar9;

            -- 11: generation B -- strongest parent (centre wins ties)
            if f_life(s_gc10) >= f_life(s_gm10) then s_gm11 <= s_gc10; else s_gm11 <= s_gm10; end if;
            s_grb11 <= s_grb10;
            s_gc11 <= s_gc10; s_gs11 <= s_gs10; s_gcol11 <= s_gcol10; s_gk11 <= s_gk10;
            s_gwa11 <= s_gwa10; s_gf11 <= s_gf10;
            --     compose: living cell (grout / ink, U/V swapped where the
            --     stored row parity differs) over the Schmitt picture; gate
            if s_avid_sr(9) = '0' then
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            elsif s_al10 = '1' and s_gr10 = '1' then
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            elsif s_al10 = '1' then
                s_y11 <= s_iy10;
                if s_ipar10 /= s_rpar then s_u11 <= s_iv10; s_v11 <= s_iu10;
                else s_u11 <= s_iu10; s_v11 <= s_iv10; end if;
            else
                s_y11 <= s_by10;
                if s_bpar10 /= s_rpar then s_u11 <= s_bv10; s_v11 <= s_bu10;
                else s_u11 <= s_bu10; s_v11 <= s_bv10; end if;
            end if;

            -- 12: generation C -- lifetime after this generation
            v_lf := signed(resize(f_life(s_gm11), 12)) - signed(resize(s_zdec, 12));
            if s_inf = '1' then
                s_glife12 <= f_life(s_gm11);
                s_gok12 <= s_grb11;
            elsif v_lf > 0 then
                s_glife12 <= unsigned(v_lf(10 downto 0));
                s_gok12 <= s_grb11;
            else
                s_glife12 <= (others => '0');
                s_gok12 <= '0';
            end if;
            s_gmc12 <= s_gm11(20 downto 0);
            s_gc12 <= s_gc11; s_gs12 <= s_gs11; s_gcol12 <= s_gcol11; s_gk12 <= s_gk11;
            s_gwa12 <= s_gwa11; s_gf12 <= s_gf11;

            -- 13: write the cell: generation line = rule result, other lines =
            --     unchanged; seeds on any line; Mask kills cells over dark runs
            if s_gen = '1' then
                if s_gok12 = '1' then
                    v_w := std_logic_vector(s_glife12) & s_gmc12;
                else
                    v_w := C_DEAD;
                end if;
            else
                v_w := s_gc12;
            end if;
            if s_gs12 = '1' then
                if s_inf = '1' then
                    v_w := "11111111111" & s_gcol12;
                else
                    v_w := std_logic_vector(s_reach) & s_gcol12;
                end if;
            end if;
            if s_gk12 = '0' then v_w(31 downto 21) := (others => '0'); end if;
            s_ca_we <= s_gf12;
            s_ca_wa <= s_gwa12;
            s_ca_wd <= v_w;

            -- 12..17: output conditioning.  Chroma [1 2 2 2 2 2 2 2 1]/16 as
            -- two adjacent 8-sums (centre at stage 16 -> s_u17); luma [1 2 1]
            -- centred at stage 16 -> s_yf17.
            s_csu(0) <= s_u11; s_csv(0) <= s_v11;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u11 - s_csu(7);
            s_boxv <= s_boxv + s_v11 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u17 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v17 <= v_sc(13 downto 4);
            s_y12 <= s_y11; s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14;
            s_y16 <= s_y15; s_y17x <= s_y16;
            v_sy := resize(s_y15, 12) + shift_left(resize(s_y16, 12), 1) + resize(s_y17x, 12);
            s_yf17 <= v_sy(11 downto 2);
            -- 18: luma toward black, 64 + 13/16 (y - 64) (white 940 -> 777); gate
            if s_yf17 > 64 then v_dl := s_yf17 - 64; else v_dl := (others => '0'); end if;
            if s_avid_sr(16) = '1' then
                s_y18 <= s_yf17 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            -- 19..20: pad to the 20-clock latency
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            -- delays
            for i in 2 to 7 loop s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            for i in 3 to 6 loop s_rxd(i) <= s_rxd(i - 1); end loop;
            for i in 3 to 9 loop s_cd(i) <= s_cd(i - 1); end loop;
            for i in 2 to 8 loop s_cs(i) <= s_cs(i - 1); end loop;
            for i in 3 to 9 loop s_gx(i) <= s_gx(i - 1); end loop;
            for i in 3 to 8 loop s_bey(i) <= s_bey(i - 1); end loop;
            for i in 3 to 7 loop s_sx(i) <= s_sx(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture lineage;
