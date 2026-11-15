-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- APPLIQUE -- re-renders video as flat matte regions (luma levels x chroma
-- classes, cleaned by mode filters) and cuts every region from a sheet of
-- patterned cloth: stripes, checks, plaids, houndstooth, dots, weaves, bricks,
-- scales, chevrons and quilt blocks, drawn as two tones with an embossed edge.
-- The geometric sibling of VENEER (organic textures): the matte pipeline is
-- VENEER's, taken over unchanged (fork of programs/ron/veneer/veneer.vhd,
-- 2026-11-14); the noise fields and shaping stages are replaced by a pattern
-- generator.  Read DESIGN.md first.
--
-- Controls
--   K1   Pattern set (8): Quilt, Weave, Stripe, Check, Dots, Tile, Deco, Hatch
--   K2   Hue (16 steps)
--   K3   Depth (contrast of the two tones and of the emboss)
--   K4   Colour: Source / Tint / Palette / Patchwork
--   K5   Scale (continuous, x1 .. x16)
--   K6   Drift (the cloth slides; 0 = still)
--   S7   Seams: Off / On (black line on region boundaries)
--   S8   Thresholds: Fixed / Adaptive
--   S9   Invert (the two tones swap)
--   S10  Relief: Flat / Embossed
--   S11  Angle: Straight / Bias (every pattern turned 45 degrees)
--   P12  Levels 2..8
--
-- Architecture: an F domain (every clock) and an H domain (one step per pixel
-- pair, clock enable `ce`).  The matte analysis runs at half horizontal
-- resolution so every line buffer is 1024 deep (chroma 512).  Stage offsets in
-- comments are H steps after the capture of column m ("stage k holds column m
-- during step m+k").  The pattern is label independent up to its recipe and
-- runs at the full pixel rate beside the late H stages.  Latency 84 clocks,
-- all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture applique of program_top is

    ------------------------------------------------------------------------
    -- BUILD CONFIGURATION (the VENEER build switches, fixed for APPLIQUE: kept
    -- so the shared matte code stays line-for-line VENEER's)
    ------------------------------------------------------------------------
    constant C_HAS_OUTLINE  : boolean := true;     -- region outline layer (the seams)
    constant C_HAS_SKETCH   : boolean := false;    -- DoG sketch layer (VENEER INK only)
    constant C_HAS_CANNY    : boolean := false;    -- Canny-lite contour layer (VENEER INK only)
    constant C_HAS_TEMPORAL : boolean := false;    -- temporal label vote (128x64 cells: blocky on smooth gradients; off)
    constant C_SIM_FAST     : boolean := false;    -- previews only: adaptive thresholds settle in one frame

    ------------------------------------------------------------------------
    -- latency: 2 * 37 H steps + 3 (front end) + 7 F stages = 84
    ------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : natural := 84;
    constant C_RING_RD : integer := 79;      -- sync ring read offset
    constant C_NH      : integer := 37;      -- last H stage (hand-off "current")

    -- tuning constants (8-bit luma units unless noted)
    constant C_HYST   : integer := 1;        -- spatial hysteresis band (3 dragged level edges sideways on smooth gradients)
    constant C_SAT1   : integer := 10;       -- grey below (octagonal |c|)
    constant C_SAT2   : integer := 36;       -- mid below, high above
    constant C_TOPG   : integer := 6;        -- lines blanked at the top of frame
    constant C_BLKY   : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID    : unsigned(9 downto 0) := to_unsigned(512, 10);

    ------------------------------------------------------------------------
    -- helpers
    ------------------------------------------------------------------------
    function f_c8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return resize(unsigned(v(7 downto 0)), 8); end if;
    end function;

    function f_b2sl(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

    function f_umax(a, b : unsigned) return unsigned is
    begin
        if a >= b then return a; else return b; end if;
    end function;

    function f_umin(a, b : unsigned) return unsigned is
    begin
        if a <= b then return a; else return b; end if;
    end function;

    -- |v| of a 9-bit signed in the range -255..255 -> 8 bits
    function f_abs8(v : signed(8 downto 0)) return unsigned is
        variable t : signed(8 downto 0);
    begin
        if v < 0 then t := -v; else t := v; end if;
        return unsigned(t(7 downto 0));
    end function;

    function f_pop7(v : std_logic_vector(6 downto 0)) return unsigned is
        variable n : unsigned(2 downto 0) := (others => '0');
    begin
        for i in 0 to 6 loop
            if v(i) = '1' then n := n + 1; end if;
        end loop;
        return n;
    end function;

    -- 5-sample mode of 3-bit symbols: partner counts, then pick (ties -> centre)
    type t_sym5 is array (0 to 4) of unsigned(2 downto 0);
    type t_cnt5 is array (0 to 4) of unsigned(2 downto 0);
    function f_pop4(v : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case v is
            when "0000" => return "000";
            when "0001" | "0010" | "0100" | "1000" => return "001";
            when "0111" | "1011" | "1101" | "1110" => return "011";
            when "1111" => return "100";
            when others => return "010";
        end case;
    end function;
    type t_sym5c is array (0 to 4) of unsigned(4 downto 0);
    function f_cnt5c(s : t_sym5c) return t_cnt5 is
        variable c : t_cnt5;
        variable e : std_logic_vector(3 downto 0);
        variable k : integer;
    begin
        for i in 0 to 4 loop
            k := 0;
            for j in 0 to 4 loop
                if i /= j then
                    if s(i) = s(j) then e(k) := '1'; else e(k) := '0'; end if;
                    k := k + 1;
                end if;
            end loop;
            c(i) := f_pop4(e);
        end loop;
        return c;
    end function;
    function f_cnt5(s : t_sym5) return t_cnt5 is
        variable c : t_cnt5;
        variable e : std_logic_vector(3 downto 0);
        variable k : integer;
    begin
        for i in 0 to 4 loop
            k := 0;
            for j in 0 to 4 loop
                if i /= j then
                    if s(i) = s(j) then e(k) := '1'; else e(k) := '0'; end if;
                    k := k + 1;
                end if;
            end loop;
            c(i) := f_pop4(e);
        end loop;
        return c;
    end function;
    -- argmax of five 3-bit counts as parallel threshold logic (no compare
    -- chains).  Ties: centre first (2, 1, 3, 0, 4).
    function f_pick5(c : t_cnt5) return integer is
        variable s4, s3, s2, s1, sel : std_logic_vector(0 to 4);
    begin
        for i in 0 to 4 loop
            s4(i) := c(i)(2);
            s3(i) := c(i)(2) or (c(i)(1) and c(i)(0));
            s2(i) := c(i)(2) or c(i)(1);
            s1(i) := c(i)(2) or c(i)(1) or c(i)(0);
        end loop;
        if s4 /= "00000" then sel := s4;
        elsif s3 /= "00000" then sel := s3;
        elsif s2 /= "00000" then sel := s2;
        elsif s1 /= "00000" then sel := s1;
        else sel := "11111"; end if;
        if sel(2) = '1' then return 2;
        elsif sel(1) = '1' then return 1;
        elsif sel(3) = '1' then return 3;
        elsif sel(0) = '1' then return 0;
        else return 4; end if;
    end function;

    -- chroma of hue h (16 steps of 22.5 degrees, counter-clockwise from +Cb in
    -- internal BT.601 order U = Cb, V = Cr) at saturation code s (0 grey, 1 =
    -- radius 160, 2 = 320, 3 = 240 in 10-bit units).  Returns U8 & V8, 8-bit
    -- chroma centred on 128 (x4 = 10-bit).  The region classes of the matte
    -- path are the odd hues (sector k = hue 2k+1).
    function f_hue_uv(h : unsigned(3 downto 0); s : unsigned(1 downto 0)) return std_logic_vector is
        type t_m5 is array (0 to 4) of integer;
        variable m : t_m5;
        variable r : integer range 0 to 3;
        variable cu, cv : integer;
    begin
        case s is
            when "01"   => m := (40, 37, 28, 15, 0);
            when "10"   => m := (80, 74, 57, 31, 0);
            when "11"   => m := (60, 55, 42, 23, 0);
            when others => m := (0, 0, 0, 0, 0);
        end case;
        r := to_integer(h(1 downto 0));
        case h(3 downto 2) is
            when "00"   => cu := m(r);      cv := m(4 - r);
            when "01"   => cu := -m(4 - r); cv := m(r);
            when "10"   => cu := -m(r);     cv := -m(4 - r);
            when others => cu := m(4 - r);  cv := -m(r);
        end case;
        return std_logic_vector(to_unsigned(128 + cu, 8)) & std_logic_vector(to_unsigned(128 + cv, 8));
    end function;

    ------------------------------------------------------------------------
    -- pattern generator helpers
    ------------------------------------------------------------------------
    -- one of two 4-bit levels
    function f_sel(c : std_logic; a, b : integer) return unsigned is
    begin
        if c = '1' then return to_unsigned(a, 4); else return to_unsigned(b, 4); end if;
    end function;
    -- three levels by how many of two bands cross (gingham, plaid)
    function f_g3(a, b : std_logic; l0, l1, l2 : integer) return unsigned is
    begin
        if a = '1' and b = '1' then return to_unsigned(l2, 4);
        elsif a = '0' and b = '0' then return to_unsigned(l0, 4);
        else return to_unsigned(l1, 4); end if;
    end function;
    -- distance of a 6-bit cell position from the cell's centre line (0..31)
    function f_fold(p : unsigned(5 downto 0)) return unsigned is
    begin
        if p(5) = '1' then return p(4 downto 0); else return not p(4 downto 0); end if;
    end function;
    -- square of a 5-bit distance (a 32-entry table, not a multiplier)
    type t_sqtab is array (0 to 31) of integer range 0 to 961;
    constant C_SQ : t_sqtab := (0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100, 121, 144, 169, 196, 225,
                                256, 289, 324, 361, 400, 441, 484, 529, 576, 625, 676, 729, 784, 841, 900, 961);
    function f_sq5(d : unsigned(4 downto 0)) return unsigned is
    begin
        return to_unsigned(C_SQ(to_integer(d)), 10);
    end function;
    -- 4 x 4 ordered-dither levels
    type t_bayer is array (0 to 15) of integer range 0 to 15;
    constant C_BAYER : t_bayer := (0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5);
    function f_bayer(i : unsigned(3 downto 0)) return unsigned is
    begin
        return to_unsigned(C_BAYER(to_integer(i)), 4);
    end function;

    ------------------------------------------------------------------------
    -- ROM contents (generated: programs/ron/applique/tables.py rewrites this block)
    ------------------------------------------------------------------------
    -- TABLES BEGIN
    type t_rom16 is array (0 to 255) of std_logic_vector(15 downto 0);
    -- set ROM: 2 words per (set, slot): recipe, colours (see tables.py)
    constant C_SROM : t_rom16 := (
        x"005A", x"092F", x"0047", x"FB9F", x"006A", x"A9EF", x"0003", x"4C0F",
        x"0048", x"C8DF", x"007A", x"6A1F", x"004B", x"E40F", x"0111", x"E40F",
        x"0102", x"FB9F", x"005C", x"0F9F", x"000B", x"59DF", x"000C", x"6E1F",
        x"006B", x"961F", x"0212", x"55DF", x"0010", x"740F", x"0032", x"840F",
        x"0041", x"080F", x"0021", x"FC0F", x"0141", x"480F", x"0271", x"092F",
        x"0211", x"E80F", x"0341", x"680F", x"0261", x"E40F", x"0011", x"440F",
        x"0044", x"0AAF", x"0005", x"000F", x"0013", x"480F", x"0064", x"4A2F",
        x"0143", x"E80F", x"0063", x"C40F", x"0013", x"E80F", x"0033", x"340F",
        x"0466", x"19AF", x"0076", x"0B6F", x"0406", x"480F", x"0067", x"CA2F",
        x"0416", x"E8DF", x"0429", x"6A2F", x"0846", x"340F", x"0816", x"C80F",
        x"004D", x"016F", x"105E", x"0B2F", x"0078", x"C80F", x"005F", x"5A1F",
        x"001D", x"036F", x"0147", x"6A1F", x"104E", x"C61F", x"003D", x"021F",
        x"004C", x"01EF", x"0077", x"BA2F", x"107E", x"CA2F", x"0058", x"01EF",
        x"0009", x"7E2F", x"006C", x"BA1F", x"014A", x"780F", x"0057", x"840F",
        x"081F", x"55DF", x"0123", x"55DF", x"0111", x"55DF", x"0321", x"65DF",
        x"002F", x"65DF", x"0C16", x"65DF", x"0261", x"65DF", x"0010", x"740F",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000"
    );
    -- TABLES END

    ------------------------------------------------------------------------
    -- memories (all 1W1R; read every clock, written on ce_d or to the other
    -- bank: never a read/write collision, so no_rw_check everywhere)
    ------------------------------------------------------------------------
    type t_lb8   is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_lb16  is array (0 to 511)  of std_logic_vector(15 downto 0);
    type t_tmp   is array (0 to 511)  of std_logic_vector(7 downto 0);
    type t_hist  is array (0 to 255)  of unsigned(15 downto 0);
    type t_ring  is array (0 to 127)  of std_logic_vector(7 downto 0);
    type t_dring is array (0 to 31)   of std_logic_vector(15 downto 0);

    signal lby1, lby2, lby3, lby4 : t_lb8 := (others => x"10");   -- raw luma rows
    signal lbc1  : t_lb16 := (others => x"8080");                 -- chroma row
    signal lbl1, lbl2, lbl3, lbl4 : t_lb8 := (others => (others => '0'));  -- label rows
    signal lbink : t_lb8 := (others => (others => '0'));          -- ink history
    signal tmpr  : t_tmp := (others => (others => '0'));          -- temporal cells
    signal hist  : t_hist := (others => (others => '0'));         -- luma histogram
    signal ring  : t_ring := (others => (others => '0'));         -- syncs
    signal dring : t_dring := (others => (others => '0'));        -- 12-step delay
    attribute no_rw_check : boolean;
    attribute no_rw_check of lby1  : signal is true;
    attribute no_rw_check of lby2  : signal is true;
    attribute no_rw_check of lby3  : signal is true;
    attribute no_rw_check of lby4  : signal is true;
    attribute no_rw_check of lbc1  : signal is true;
    attribute no_rw_check of lbl1  : signal is true;
    attribute no_rw_check of lbl2  : signal is true;
    attribute no_rw_check of lbl3  : signal is true;
    attribute no_rw_check of lbl4  : signal is true;
    attribute no_rw_check of lbink : signal is true;
    attribute no_rw_check of tmpr  : signal is true;
    attribute no_rw_check of hist  : signal is true;
    attribute no_rw_check of ring  : signal is true;
    attribute no_rw_check of dring : signal is true;

    signal lby_q1, lby_q2, lby_q3, lby_q4 : std_logic_vector(7 downto 0) := (others => '0');
    signal lbc_q1 : std_logic_vector(15 downto 0) := (others => '0');
    signal lbl_q1, lbl_q2, lbl_q3, lbl_q4 : std_logic_vector(7 downto 0) := (others => '0');
    signal lbink_q : std_logic_vector(7 downto 0) := (others => '0');
    signal tmp_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal hist_q : unsigned(15 downto 0) := (others => '0');
    signal ring_q : std_logic_vector(7 downto 0) := (others => '0');
    signal dr_q, dr_c : std_logic_vector(15 downto 0) := (others => '0');
    signal dr_w : unsigned(4 downto 0) := (others => '0');
    signal srom_q, wc_f : std_logic_vector(15 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- controls (free-running decode)
    ------------------------------------------------------------------------
    signal k1, k2, k3, k4, k5, k6, p12 : unsigned(9 downto 0) := (others => '0');
    signal s_adapt, s_white, s_ground, s_both, s_paper, s_inv, s_relief, s_bias : std_logic := '0';
    signal c_il    : std_logic := '0';
    signal c_lout, c_sk_on, c_ct_on : std_logic := '0'; -- seams / (sketch / contour: VENEER INK, never on)
    signal c_lmask : unsigned(7 downto 0) := (others => '1');   -- label bits that make a seam
    signal c_hrot  : unsigned(3 downto 0) := "0000";   -- hue turn
    signal c_set   : unsigned(2 downto 0) := "000";    -- pattern set
    signal c_col   : unsigned(1 downto 0) := "00";     -- 0 source, 1 tint, 2 palette, 3 patchwork
    -- K5 Scale: deadbanded target, glided value (Q10.4), per-mode steps (Q.10 pattern positions)
    signal c_ktg   : unsigned(9 downto 0) := (others => '0');
    signal c_kd    : signed(10 downto 0) := (others => '0');
    signal c_kf    : unsigned(13 downto 0) := (others => '0');
    signal c_gd    : signed(14 downto 0) := (others => '0');
    signal c_mant  : unsigned(13 downto 0) := to_unsigned(8192, 14);
    signal c_vs    : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal c_tm    : unsigned(2 downto 0) := "000";    -- timing class (see p_ctl)
    signal c_xa, c_xb, c_xs : unsigned(13 downto 0) := (others => '0');
    signal c_ya    : unsigned(14 downto 0) := (others => '0');
    signal c_yb    : signed(13 downto 0) := (others => '0');
    signal c_ys    : unsigned(14 downto 0) := (others => '0');
    signal c_dspd  : unsigned(5 downto 0) := (others => '0');   -- K6 Drift speed
    signal c_nz    : unsigned(2 downto 0) := "010";
    signal c_nlev  : integer range 2 to 8 := 4;
    signal c_width : unsigned(1 downto 0) := "00";     -- outline 0 thin 1 medium 2 bold
    signal c_xthr  : unsigned(6 downto 0) := to_unsigned(60, 7);      -- sketch threshold, 1/16 code
    signal c_cgain : unsigned(4 downto 0) := "01010";  -- contour pre-gain 2..17
    signal c_depth : unsigned(3 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- per-frame values (threshold sequencer)
    ------------------------------------------------------------------------
    -- thr(0) = black (16), never written; thr(k >= nlev) = 511 (never passed)
    type t_thr is array (0 to 7) of unsigned(8 downto 0);
    type t_cen is array (0 to 7) of unsigned(7 downto 0);
    signal thr : t_thr := (to_unsigned(16, 9), to_unsigned(71, 9), to_unsigned(126, 9), to_unsigned(181, 9),
                           to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9));
    signal cen : t_cen := (to_unsigned(43, 8), to_unsigned(98, 8), to_unsigned(153, 8), to_unsigned(208, 8),
                           to_unsigned(235, 8), to_unsigned(235, 8), to_unsigned(235, 8), to_unsigned(235, 8));
    signal thr_wd  : unsigned(8 downto 0) := (others => '0');
    signal cen_wd  : unsigned(7 downto 0) := (others => '0');
    signal thr_we  : std_logic_vector(1 to 7) := (others => '0');
    signal cen_we  : std_logic_vector(0 to 7) := (others => '0');
    signal f_nlev  : integer range 2 to 8 := 4;        -- levels in force this frame
    signal sq      : integer range 0 to 31 := 0;
    signal sq_k    : integer range 0 to 8 := 0;
    signal sq_i    : unsigned(5 downto 0) := (others => '0');
    signal sq_acc  : unsigned(8 downto 0) := (others => '0');
    signal sq_fdel : unsigned(6 downto 0) := to_unsigned(55, 7);
    signal sq_reload : std_logic := '1';
    signal sq_nprev : integer range 2 to 8 := 2;
    signal sq_hval : unsigned(15 downto 0) := (others => '0');
    signal sq_j, sq_jm : integer range 0 to 8 := 0;      -- thresholds / medians placed
    signal sq_cum, sq_tt, sq_mt, sq_step : unsigned(17 downto 0) := (others => '0');
    signal pl_thr, pl_med : std_logic := '0';
    signal pl_old, pl_prev2, pl_new : unsigned(8 downto 0) := (others => '0');
    signal pl_dif : signed(9 downto 0) := (others => '0');
    signal pl_lp : signed(10 downto 0) := (others => '0');
    signal hist_sa : unsigned(7 downto 0) := (others => '0');
    signal hist_swe, seq_hist : std_logic := '0';
    signal dv_q    : unsigned(17 downto 0) := (others => '0');
    signal dv_rem  : unsigned(4 downto 0) := (others => '0');
    signal dv_n    : unsigned(4 downto 0) := (others => '0');
    signal div_busy : std_logic := '0';
    signal hcnt, htot : unsigned(17 downto 0) := (others => '0');   -- histogram samples this / last frame

    ------------------------------------------------------------------------
    -- F domain front end
    ------------------------------------------------------------------------
    signal f0_y, f0_u, f0_v : unsigned(9 downto 0) := (others => '0');
    signal f0_avid, f0_hs, f0_vs, f0_fld : std_logic := '0';
    signal f0_x  : unsigned(11 downto 0) := (others => '0');
    signal f0p_y8, f0p_u8, f0p_v8 : unsigned(7 downto 0) := (others => '0');
    signal f0p_avid : std_logic := '0';
    signal ce, ce_d : std_logic := '0';
    attribute keep : boolean;
    attribute keep of ce : signal is true;
    signal meas_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal aline  : unsigned(10 downto 0) := (others => '0');
    signal vs_prev, saw_act, vs_edge : std_logic := '0';
    signal hs_prev : std_logic := '1';
    signal par_ph, par_usual, line_off : std_logic := '0';
    signal par_cnt : unsigned(3 downto 0) := (others => '0');
    signal ring_w  : unsigned(6 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- H domain
    ------------------------------------------------------------------------
    signal hv_c, hf_c : std_logic_vector(0 to C_NH) := (others => '0');
    signal hx : unsigned(9 downto 0) := (others => '0');         -- column at H0
    signal h0_y, h0_u, h0_v : unsigned(7 downto 0) := (others => '0');
    type t_col5 is array (0 to 4) of unsigned(7 downto 0);
    signal ny : t_col5 := (others => (others => '0'));            -- rows r-4 .. r (ny0 oldest)
    signal hx_d1 : unsigned(9 downto 0) := (others => '0');
    signal h1_u, h1_v : unsigned(7 downto 0) := (others => '0');
    signal cq_u, cq_v : unsigned(7 downto 0) := (others => '0');  -- quarter-res chroma
    signal cq_v1 : std_logic := '0';
    signal c1_r1 : std_logic_vector(15 downto 0) := (others => '0');
    -- line block
    signal x2_c3 : unsigned(9 downto 0) := (others => '0');
    type t_c3s is array (0 to 2) of unsigned(9 downto 0);
    signal c3s : t_c3s := (others => (others => '0'));
    signal x5_g1 : unsigned(11 downto 0) := (others => '0');
    signal xa10, xa22, xa32, ca32 : unsigned(1 downto 0) := (others => '0');  -- fine-line alpha (stage 10 / 22 / at the ink RAM write)
    signal ys17, ys18 : unsigned(7 downto 0) := (others => '0');  -- smoothed luma
    -- chroma class
    signal c_fill : std_logic := '0';
    signal cvs_u, cvs_v, cvp_u, cvp_v : unsigned(8 downto 0) := (others => '0');
    signal us8, vs8 : unsigned(7 downto 0) := x"80";
    signal c4_du, c4_dv : signed(8 downto 0) := (others => '0');
    signal c4_au, c4_av : unsigned(7 downto 0) := (others => '0');
    signal c5_sat : unsigned(8 downto 0) := (others => '0');
    signal c5_sdu, c5_sdv, c5_gt : std_logic := '0';
    signal c6_hue : unsigned(2 downto 0) := (others => '0');
    signal c6_sat : unsigned(1 downto 0) := (others => '0');
    signal cc0 : unsigned(4 downto 0) := (others => '0');
    -- quantizer
    signal q18_h : std_logic_vector(6 downto 0) := (others => '0');
    signal q19_b, q20_b : unsigned(2 downto 0) := (others => '0');
    signal q19_tlo, q19_thi : unsigned(8 downto 0) := (others => '0');
    signal ys19 : unsigned(7 downto 0) := (others => '0');
    signal q20_lo, q20_hi : std_logic := '0';
    signal tmp_c : std_logic_vector(7 downto 0) := (others => '0');
    signal lab21 : unsigned(7 downto 0) := (others => '0');       -- {lum, hue, sat}
    signal cnt18 : unsigned(9 downto 0) := (others => '0');
    signal hist_ra : unsigned(5 downto 0) := (others => '0');
    signal hs_c : std_logic_vector(0 to 2) := (others => '0');
    signal hq20 : unsigned(15 downto 0) := (others => '0');
    signal tmp_ra : unsigned(8 downto 0) := (others => '0');
    -- mode filters
    type t_lab6 is array (0 to 5) of unsigned(7 downto 0);
    signal mw : t_lab6 := (others => (others => '0'));
    signal mh_cl, mh_cc : t_cnt5 := (others => (others => '0'));
    signal rm26 : unsigned(7 downto 0) := (others => '0');
    signal cnt26, cnt26_d : unsigned(9 downto 0) := (others => '0');
    type t_lab5 is array (0 to 4) of unsigned(7 downto 0);
    signal mv : t_lab5 := (others => (others => '0'));            -- 0 = row r .. 4 = row r-4
    signal mv_cl, mv_cc : t_cnt5 := (others => (others => '0'));
    signal mvc : t_lab5 := (others => (others => '0'));
    signal cl29, ab29 : unsigned(7 downto 0) := (others => '0');
    -- label delay, fill lookups
    type t_ld is array (30 to 34) of unsigned(7 downto 0);
    signal ld : t_ld := (others => (others => '0'));
    signal ym35 : unsigned(7 downto 0) := (others => '0');
    signal lum35 : unsigned(2 downto 0) := (others => '0');
    signal slot32 : unsigned(2 downto 0) := (others => '0');
    signal r33_w : std_logic_vector(15 downto 0) := (others => '0');   -- recipe word
    signal wc34 : std_logic_vector(15 downto 0) := (others => '0');
    signal c35_ha, c35_hb : unsigned(3 downto 0) := (others => '0');
    signal c35_sa, c35_sb : unsigned(1 downto 0) := (others => '0');
    signal ua36, va36, ua37, va37 : unsigned(7 downto 0) := x"80";
    signal du36, dv36 : signed(8 downto 0) := (others => '0');
    -- ink
    type t_lab3 is array (0 to 2) of unsigned(7 downto 0);
    signal cw3 : t_lab3 := (others => (others => '0'));
    signal cnt30, cnt30_d, cnt30_dd : unsigned(9 downto 0) := (others => '0');
    signal da30, da31, t32, dr32 : std_logic := '0';
    signal inkq32 : std_logic_vector(7 downto 0) := (others => '0');
    type t_bit3 is array (0 to 2) of std_logic;
    signal srt, srdr, srt1, srt2 : t_bit3 := (others => '0');
    type t_a2 is array (0 to 1) of unsigned(1 downto 0);
    signal srxs, srxc : t_a2 := (others => (others => '0'));
    signal ia35 : unsigned(1 downto 0) := (others => '0');        -- ink alpha 0 / quarter / half / full
    -- hand-off
    signal ho_n_y, ho_c_y : unsigned(7 downto 0) := (others => '0');
    signal ho_n_lum, ho_c_lum : unsigned(2 downto 0) := (others => '0');
    signal ho_n_a, ho_c_a : unsigned(1 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pattern generator (F domain)
    ------------------------------------------------------------------------
    -- relief line buffer: per pixel pair, the top two bits of t of the line
    -- above (3:2) and of the line above that (1:0)
    type t_rl is array (0 to 1023) of std_logic_vector(3 downto 0);
    signal rlram : t_rl := (others => (others => '0'));
    attribute no_rw_check of rlram : signal is true;
    signal rl_q, rl_d0, rl_d1, rl_d2 : std_logic_vector(3 downto 0) := (others => '0');
    -- coordinates: Q11.10 pattern positions (64 per cell at size 0)
    signal xacc, yacc, xoff, yoff : unsigned(20 downto 0) := (others => '0');
    signal yfirst : std_logic := '1';
    signal px : unsigned(10 downto 0) := (others => '0');
    signal run, st_r, rc_r, ls_r : std_logic := '0';
    signal g_x, g_y : unsigned(10 downto 0) := (others => '0');
    signal g_u, g_v : unsigned(12 downto 0) := (others => '0');
    -- T1..T4
    signal f1_a, f1_b : unsigned(10 downto 0) := (others => '0');
    signal f1_typ, f2_typ, f3_typ : std_logic_vector(3 downto 0) := (others => '0');
    signal f1_var, f2_var, f3_var, f1_size : std_logic_vector(1 downto 0) := (others => '0');
    signal f1_stg, f1_hst, f1_inv, f2_inv, f3_inv : std_logic := '0';
    signal f2_pa, f2_pb, f3_pa, f3_pb : unsigned(5 downto 0) := (others => '0');
    signal f2_da, f2_db, f3_da, f3_db : unsigned(4 downto 0) := (others => '0');
    signal f2_ia0, f2_ib0, f3_ia0, f3_ib0 : std_logic := '0';
    signal f3_gt, f3_hg, f3_gs : std_logic := '0';
    signal f3_sm : unsigned(6 downto 0) := (others => '0');
    signal f3_df, f3_l1, f3_ch : unsigned(5 downto 0) := (others => '0');
    signal f3_li, f3_sc : unsigned(4 downto 0) := (others => '0');
    signal f3_r2 : unsigned(10 downto 0) := (others => '0');
    -- T4..T8
    signal f5_px0 : std_logic := '0';
    signal f6_m : unsigned(2 downto 0) := (others => '0');
    signal f4_t, f7_ua, f7_va, f8_u, f8_v : unsigned(7 downto 0) := x"80";
    signal f5_rel : signed(6 downto 0) := (others => '0');
    signal f5_tc : signed(7 downto 0) := (others => '0');
    signal f5_ts : unsigned(9 downto 0) := (others => '0');
    signal f6_e, f7_mu, f7_mv : signed(8 downto 0) := (others => '0');
    signal f7_q1, f7_q2, f8_ey : signed(12 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- F domain output
    ------------------------------------------------------------------------
    signal o1_y, o1_u, o1_v : unsigned(9 downto 0) := (others => '0');
    signal o1_a : unsigned(1 downto 0) := (others => '0');
    signal o1_bg, o1_noink : std_logic := '0';
    signal o3_y, o3_u, o3_v : unsigned(9 downto 0) := (others => '0');
    signal o3_py, o3_pu, o3_pv : signed(10 downto 0) := (others => '0');
    signal o4_y, o4_u, o4_v : unsigned(9 downto 0) := (others => '0');
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal o6_y, o6_u, o6_v : unsigned(9 downto 0) := (others => '0');
    signal o7_y, o7_u, o7_v : unsigned(9 downto 0) := (others => '0');
    signal rq_avid_d, out_sel, out_avid : std_logic := '0';
    signal out_y, out_u, out_v : unsigned(9 downto 0) := (others => '0');
    signal out_hs, out_vs, out_fld : std_logic := '1';

begin

    ------------------------------------------------------------------------
    -- controls: free-running decode.  Stepped knobs take a new zone only when
    -- the knob sits at least 8 counts inside it (hysteresis without adders);
    -- a labelled default arrives as the label INDEX (< n) at power-up.
    ------------------------------------------------------------------------
    p_ctl : process(clk)
        variable v_q7 : unsigned(12 downto 0);
        variable v_h  : signed(14 downto 0);
        variable v_y  : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            k1 <= unsigned(registers_in(0)); k2 <= unsigned(registers_in(1));
            k3 <= unsigned(registers_in(2)); k4 <= unsigned(registers_in(3));
            k5 <= unsigned(registers_in(4)); k6 <= unsigned(registers_in(5));
            p12 <= unsigned(registers_in(7));
            -- switches: S7 Seams, S8 Thresholds, S9 Invert, S10 Relief, S11 Angle
            c_lout <= registers_in(6)(0);
            s_adapt <= registers_in(6)(1);
            s_inv <= registers_in(6)(2);
            s_relief <= registers_in(6)(3);
            s_bias <= registers_in(6)(4);
            -- (VENEER INK controls of the shared matte code: fixed)
            s_white <= '0'; s_ground <= '0'; s_both <= '0'; s_paper <= '0';
            c_sk_on <= '0'; c_ct_on <= '0';

            -- timing class from the timing ID: interlace flag (field lines) and the
            -- steps that put the pattern in virtual 1080-line units
            --   0 1080p   1 1080i   2 720p   3 480i   4 576i   5 480p   6 576p
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0           => c_il <= '1'; c_tm <= "011";
                when 8           => c_il <= '1'; c_tm <= "100";
                when 1 | 2 | 10  => c_il <= '1'; c_tm <= "001";
                when 4           => c_il <= '0'; c_tm <= "101";
                when 12          => c_il <= '0'; c_tm <= "110";
                when 5 | 6 | 14  => c_il <= '0'; c_tm <= "010";
                when others      => c_il <= '0'; c_tm <= "000";
            end case;

            -- K1 Pattern set, 8 zones
            if k1 < 8 then c_set <= k1(2 downto 0);
            elsif k1(6 downto 3) /= "0000" and k1(6 downto 3) /= "1111" then c_set <= k1(9 downto 7); end if;
            -- K2 Hue: turns every colour by 16 steps of 22.5 degrees
            if k2(5 downto 2) /= "0000" and k2(5 downto 2) /= "1111" then c_hrot <= k2(9 downto 6); end if;
            -- K3 Depth
            c_depth <= k3(9 downto 6);
            -- K4 Colour: Source / Tint / Palette / Patchwork
            if k4 < 4 then c_col <= k4(1 downto 0);
            elsif k4(7 downto 3) /= "00000" and k4(7 downto 3) /= "11111" then c_col <= k4(9 downto 8); end if;
            -- seams follow what the fill follows: in Palette the luma level alone
            if c_col = "10" then c_lmask <= "11100000"; else c_lmask <= (others => '1'); end if;
            c_width <= "00";                                    -- seam footprint: thin (one sample: 2 px x 1 line at 1080p)

            -- K5 Scale.  A 4-count deadband holds the target against pot noise (the
            -- far edge of the screen magnifies one count of scale), then the value
            -- glides a quarter of the way per frame.
            c_kd <= signed('0' & k5) - signed('0' & c_ktg);
            if c_kd > 3 or c_kd < -4 then c_ktg <= k5; end if;
            c_gd <= signed(resize(shift_left(resize(c_ktg, 14), 4), 15)) - signed(resize(c_kf, 15));
            if vs_edge = '1' then
                if C_SIM_FAST then c_kf <= shift_left(resize(c_ktg, 14), 4);
                else
                    v_h := signed('0' & c_kf) + shift_right(c_gd, 2);
                    c_kf <= unsigned(v_h(13 downto 0));
                end if;
            end if;
            -- step in pattern positions per virtual pixel (Q.10): 4 at 0%, halving
            -- every quarter of the knob (straight lines between the octaves)
            c_mant <= to_unsigned(8192, 14) - resize(c_kf(11 downto 0), 14);
            case c_kf(13 downto 12) is
                when "00"   => c_vs <= c_mant(13 downto 1);
                when "01"   => c_vs <= resize(c_mant(13 downto 2), 13);
                when "10"   => c_vs <= resize(c_mant(13 downto 3), 13);
                when others => c_vs <= resize(c_mant(13 downto 4), 13);
            end case;
            -- per pixel: x1 (1080), x1.5 (720p), x2 (SD)
            case c_tm is
                when "010"                   => c_xa <= resize(c_vs, 14); c_xb <= resize(shift_right(c_vs, 1), 14);
                when "011" | "100" | "101" | "110" => c_xa <= shift_left(resize(c_vs, 14), 1); c_xb <= (others => '0');
                when others                  => c_xa <= resize(c_vs, 14); c_xb <= (others => '0');
            end case;
            c_xs <= c_xa + c_xb;
            -- per (field) line: x1, x2 (1080i), x1.5 (720p), x4.5 (480i), x3.75 (576i),
            -- x2.25 (480p), x1.875 (576p)
            case c_tm is
                when "001"  => c_ya <= shift_left(resize(c_vs, 15), 1); c_yb <= (others => '0');
                when "010"  => c_ya <= resize(c_vs, 15);                c_yb <= signed(resize(shift_right(c_vs, 1), 14));
                when "011"  => c_ya <= shift_left(resize(c_vs, 15), 2); c_yb <= signed(resize(shift_right(c_vs, 1), 14));
                when "100"  => c_ya <= shift_left(resize(c_vs, 15), 2); c_yb <= -signed(resize(shift_right(c_vs, 2), 14));
                when "101"  => c_ya <= shift_left(resize(c_vs, 15), 1); c_yb <= signed(resize(shift_right(c_vs, 2), 14));
                when "110"  => c_ya <= shift_left(resize(c_vs, 15), 1); c_yb <= -signed(resize(shift_right(c_vs, 3), 14));
                when others => c_ya <= resize(c_vs, 15);                c_yb <= (others => '0');
            end case;
            v_y := signed('0' & c_ya) + resize(c_yb, 16);
            c_ys <= unsigned(v_y(14 downto 0));

            -- K6 Drift: 0..63 eighths of a pattern position per frame (0 = still)
            c_dspd <= k6(9 downto 4);

            -- P12 Levels: 7 zones (2..8) = top bits of 7 x p12
            v_q7 := (p12 & "000") - resize(p12, 13);
            if v_q7(9 downto 6) /= "0000" and v_q7(9 downto 6) /= "1111" then c_nz <= v_q7(12 downto 10); end if;
            c_nlev <= to_integer(c_nz) + 2;
        end if;
    end process p_ctl;

    ------------------------------------------------------------------------
    -- F front end: input latch (U/V swapped to BT.601 internally), counters,
    -- half-rate enable, hsync->avid parity vote, sync ring write
    ------------------------------------------------------------------------
    p_front : process(clk)
        variable v_x : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- F0
            f0_y <= unsigned(data_in.y); f0_u <= unsigned(data_in.v); f0_v <= unsigned(data_in.u);
            f0_avid <= data_in.avid; f0_hs <= data_in.hsync_n; f0_vs <= data_in.vsync_n; f0_fld <= data_in.field_n;
            if data_in.avid = '1' and f0_avid = '0' then v_x := (others => '0'); else v_x := f0_x + 1; end if;
            f0_x <= v_x;
            -- previous pixel (8-bit)
            f0p_y8 <= f0_y(9 downto 2); f0p_u8 <= f0_u(9 downto 2); f0p_v8 <= f0_v(9 downto 2);
            f0p_avid <= f0_avid;
            -- half-rate enable: '1' while F0 holds the second pixel of a pair (H0
            -- then captures the pair average), toggling through blanking
            -- (re-aligned at every active rise)
            if data_in.avid = '1' then ce <= v_x(0); else ce <= not ce; end if;
            ce_d <= ce;

            -- measured width, active line counter
            if f0_avid = '0' and f0p_avid = '1' then
                meas_w <= f0_x;                       -- f0_x = last index + 1 here
                if aline /= 2047 then aline <= aline + 1; end if;
            end if;
            -- frame anchor: first vsync falling edge after active video
            vs_prev <= f0_vs;
            vs_edge <= '0';
            if f0_avid = '1' then saw_act <= '1'; end if;
            if f0_vs = '0' and vs_prev = '1' and saw_act = '1' then
                saw_act <= '0'; vs_edge <= '1';
                aline <= (others => '0');
            end if;

            -- hsync -> avid phase (the encoder pairs Cb/Cr counting from hsync)
            hs_prev <= f0_hs;
            if f0_hs = '0' and hs_prev = '1' then par_ph <= '0'; else par_ph <= not par_ph; end if;
            if f0_avid = '1' and f0p_avid = '0' then
                line_off <= par_ph xor par_usual;
                if par_ph = '1' then
                    if par_cnt /= 15 then par_cnt <= par_cnt + 1; end if;
                else
                    if par_cnt /= 0 then par_cnt <= par_cnt - 1; end if;
                end if;
            end if;
            if par_cnt = 15 then par_usual <= '1'; elsif par_cnt = 0 then par_usual <= '0'; end if;

            ring_w <= ring_w + 1;
        end if;
    end process p_front;

    -- sync ring: the output syncs are the input syncs 80 clocks later, so they
    -- line up with the video whatever the pipeline does
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            ring_q <= ring(to_integer(ring_w - C_RING_RD));
            ring(to_integer(ring_w)) <= "000" & line_off & f0_fld & f0_vs & f0_hs & f0_avid;
        end if;
    end process p_ring;

    ------------------------------------------------------------------------
    -- line buffer memories (read every clock, write on ce_d: never a collision)
    ------------------------------------------------------------------------
    p_lby : process(clk)
    begin
        if rising_edge(clk) then
            lby_q1 <= lby1(to_integer(hx)); lby_q2 <= lby2(to_integer(hx));
            lby_q3 <= lby3(to_integer(hx)); lby_q4 <= lby4(to_integer(hx));
            if ce_d = '1' and hv_c(1) = '1' then
                lby1(to_integer(hx_d1)) <= std_logic_vector(ny(4));
                lby2(to_integer(hx_d1)) <= std_logic_vector(ny(3));
                lby3(to_integer(hx_d1)) <= std_logic_vector(ny(2));
                lby4(to_integer(hx_d1)) <= std_logic_vector(ny(1));
            end if;
        end if;
    end process p_lby;

    -- chroma: one quarter-res row (the class uses this row and the live one)
    p_lbc : process(clk)
    begin
        if rising_edge(clk) then
            lbc_q1 <= lbc1(to_integer(hx(9 downto 1)));
            if ce_d = '1' and cq_v1 = '1' then
                lbc1(to_integer(hx_d1(9 downto 1))) <= std_logic_vector(cq_u) & std_logic_vector(cq_v);
            end if;
        end if;
    end process p_lbc;

    -- label rows for the vertical mode.  Rows 1, 2 hold row-modes; row 3 is
    -- written two stages later with the CLEANED label of the centre row, so
    -- rows 3 and 4 of the window are cleaned rows: the row above the centre is
    -- exactly what was drawn (the outline needs it) and the vote inherits from
    -- above (vertical hysteresis), with no extra line buffer.
    p_lbl : process(clk)
    begin
        if rising_edge(clk) then
            lbl_q1 <= lbl1(to_integer(cnt26)); lbl_q2 <= lbl2(to_integer(cnt26));
            lbl_q3 <= lbl3(to_integer(cnt26)); lbl_q4 <= lbl4(to_integer(cnt26));
            if ce_d = '1' and hv_c(27) = '1' then
                lbl1(to_integer(cnt26_d)) <= std_logic_vector(mv(0));
                lbl2(to_integer(cnt26_d)) <= std_logic_vector(mv(1));
                lbl4(to_integer(cnt26_d)) <= std_logic_vector(mv(3));
            end if;
            if ce_d = '1' and hv_c(30) = '1' then
                lbl3(to_integer(cnt30)) <= std_logic_vector(cw3(0));
            end if;
        end if;
    end process p_lbl;

    -- ink history (8 bits; fields a build does not drive are pruned):
    --   (7:6) sketch alpha of this line, (5:4) one line old -> read back two lines old
    --   (3:2) contour alpha of this line -> read back one line old
    --   (1) outline boundary bit one line old, (0) two lines old (footprint)
    p_lbink : process(clk)
    begin
        if rising_edge(clk) then
            lbink_q <= lbink(to_integer(cnt30_d));
            if ce_d = '1' and hv_c(32) = '1' then
                lbink(to_integer(cnt30_dd)) <= std_logic_vector(xa32) & inkq32(7 downto 6) & std_logic_vector(ca32) & t32 & inkq32(1);
            end if;
        end if;
    end process p_lbink;

    g_tmp_ram : if C_HAS_TEMPORAL generate
        p_tmp : process(clk)
        begin
            if rising_edge(clk) then
                tmp_q <= tmpr(to_integer(tmp_ra));
                if ce_d = '1' and hv_c(30) = '1' and cnt30(5 downto 0) = 32 and aline(5 downto 0) = 32 then
                    tmpr(to_integer(aline(10 downto 6) & cnt30(9 downto 6))) <= std_logic_vector(cw3(0));
                end if;
            end if;
        end process p_tmp;
    end generate;

    -- histogram: pixel path samples during active video, sequencer owns it in
    -- vblank.  ONE read and ONE write statement (addresses/data muxed first),
    -- or yosys builds the table out of flip-flops.
    p_hist : process(clk)
        variable v_a  : unsigned(7 downto 0);
        variable v_we : std_logic;
        variable v_wd : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if seq_hist = '1' then
                v_a := hist_sa; v_we := hist_swe; v_wd := (others => '0');
            else
                v_a := "00" & hist_ra; v_we := ce_d and hs_c(2); v_wd := hq20 + 1;
            end if;
            hist_q <= hist(to_integer(v_a));
            if v_we = '1' then hist(to_integer(v_a)) <= v_wd; end if;
        end if;
    end process p_hist;

    -- delay ring: {smoothed luma (stage 5), sketch alpha (stage 10), chroma class}
    -- come back 12 H steps
    -- later (write and read pointers are 10 apart, never equal)
    p_dring : process(clk)
    begin
        if rising_edge(clk) then
            dr_q <= dring(to_integer(dr_w - 10));
            if ce = '1' then
                dring(to_integer(dr_w)) <= std_logic_vector(x5_g1(11 downto 4)) & std_logic_vector(xa10) & '0' & std_logic_vector(cc0);
            end if;
        end if;
    end process p_dring;

    ------------------------------------------------------------------------
    -- H domain: matte pipeline (every register advances on ce)
    ------------------------------------------------------------------------
    p_h : process(clk)
        variable v_s5 : t_sym5;
        variable v_pick : integer range 0 to 4;
        variable v_lum, v_b, v_blo, v_bhi : unsigned(2 downto 0);
        variable v_c5 : t_sym5c;
        variable v_hue : unsigned(2 downto 0);
        variable v_t : std_logic;
        variable v_vs : unsigned(8 downto 0);
        variable v_oa : unsigned(1 downto 0);
        variable v_h1, v_ha, v_hb : unsigned(3 downto 0);
        variable v_ca, v_cb : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            if ce = '1' then
                ----------------------------------------------------------
                -- valid / first chains
                ----------------------------------------------------------
                hv_c(0) <= f0_avid and f0p_avid and f0_x(0);
                if f0_x = 1 then hf_c(0) <= '1'; else hf_c(0) <= '0'; end if;
                for i in 1 to C_NH loop
                    hv_c(i) <= hv_c(i - 1); hf_c(i) <= hf_c(i - 1);
                end loop;

                ----------------------------------------------------------
                -- STAGE 0: pair average of F0 (this pixel) and the previous
                -- one; column counter = line-buffer read address
                ----------------------------------------------------------
                h0_y <= resize(shift_right(resize(f0_y(9 downto 2), 9) + resize(f0p_y8, 9) + 1, 1), 8);
                h0_u <= resize(shift_right(resize(f0_u(9 downto 2), 9) + resize(f0p_u8, 9) + 1, 1), 8);
                h0_v <= resize(shift_right(resize(f0_v(9 downto 2), 9) + resize(f0p_v8, 9) + 1, 1), 8);
                if (f0_avid and f0p_avid and f0_x(0)) = '1' then
                    if f0_x = 1 then hx <= (others => '0'); else hx <= hx + 1; end if;
                end if;

                ----------------------------------------------------------
                -- STAGE 1: newest luma column (rows r-4..r), chroma quarter sample
                ----------------------------------------------------------
                hx_d1 <= hx;
                ny(0) <= unsigned(lby_q4); ny(1) <= unsigned(lby_q3); ny(2) <= unsigned(lby_q2);
                ny(3) <= unsigned(lby_q1); ny(4) <= h0_y;
                h1_u <= h0_u; h1_v <= h0_v;
                cq_u <= resize(shift_right(resize(h0_u, 9) + resize(h1_u, 9) + 1, 1), 8);
                cq_v <= resize(shift_right(resize(h0_v, 9) + resize(h1_v, 9) + 1, 1), 8);
                cq_v1 <= hv_c(0) and hx(0);
                c1_r1 <= lbc_q1;

                -- SMOOTHER = the 3x3 binomial of the line block (x5_g1, stage 5),
                -- through the delay ring -> stage 17 -> ys18.  The ring also carries
                -- the sketch alpha (stage 10 -> 22).
                dr_w <= dr_w + 1;
                dr_c <= dr_q;
                ys17 <= unsigned(dr_c(15 downto 8));
                ys18 <= ys17;
                xa22 <= unsigned(dr_c(7 downto 6));

                ----------------------------------------------------------
                -- CHROMA CLASS: quarter-res 2(v) x 2(h) box (8 px x 2 lines at
                -- HD) of this row and the stored one, hue sector (8) and
                -- saturation level (3).  No chroma hysteresis: the mode filters
                -- clean chroma specks.
                ----------------------------------------------------------
                if hf_c(1) = '1' then c_fill <= '1'; end if;
                if hx_d1(0) = '1' then
                    v_vs := resize(unsigned(c1_r1(15 downto 8)), 9) + resize(cq_u, 9);
                    cvs_u <= v_vs;
                    cvs_v <= resize(unsigned(c1_r1(7 downto 0)), 9) + resize(cq_v, 9);
                    if c_fill = '1' then
                        cvp_u <= v_vs;
                        cvp_v <= resize(unsigned(c1_r1(7 downto 0)), 9) + resize(cq_v, 9);
                        c_fill <= '0';
                    else
                        cvp_u <= cvs_u; cvp_v <= cvs_v;
                    end if;
                end if;
                us8 <= resize(shift_right(resize(cvs_u, 10) + resize(cvp_u, 10), 2), 8);
                vs8 <= resize(shift_right(resize(cvs_v, 10) + resize(cvp_v, 10), 2), 8);
                c4_du <= signed(resize(us8, 9)) - to_signed(128, 9);
                c4_dv <= signed(resize(vs8, 9)) - to_signed(128, 9);
                c4_au <= f_abs8(signed(resize(us8, 9)) - to_signed(128, 9));
                c4_av <= f_abs8(signed(resize(vs8, 9)) - to_signed(128, 9));
                -- octagonal saturation and octant bits
                c5_sat <= resize(f_umax(c4_au, c4_av), 9) + resize(shift_right(f_umin(c4_au, c4_av), 1), 9);
                c5_sdu <= c4_du(8); c5_sdv <= c4_dv(8);
                if c4_au > c4_av then c5_gt <= '1'; else c5_gt <= '0'; end if;
                -- sector: 0..7 counter-clockwise from +U (Cb)
                if c5_sdv = '0' and c5_sdu = '0' then
                    if c5_gt = '1' then v_hue := "000"; else v_hue := "001"; end if;
                elsif c5_sdv = '0' and c5_sdu = '1' then
                    if c5_gt = '0' then v_hue := "010"; else v_hue := "011"; end if;
                elsif c5_sdv = '1' and c5_sdu = '1' then
                    if c5_gt = '1' then v_hue := "100"; else v_hue := "101"; end if;
                else
                    if c5_gt = '0' then v_hue := "110"; else v_hue := "111"; end if;
                end if;
                c6_hue <= v_hue;
                if c5_sat < C_SAT1 then c6_sat <= "00"; elsif c5_sat < C_SAT2 then c6_sat <= "01"; else c6_sat <= "10"; end if;
                -- the class of quarter column q lands at step 2q+8; delayed 12 more
                -- steps (delay ring) it meets the luma of half-res column m at stage 20
                cc0 <= c6_hue & c6_sat;

                ----------------------------------------------------------
                -- STAGE 18: histogram sample + temporal read address
                ----------------------------------------------------------
                if hv_c(17) = '1' then
                    if hf_c(17) = '1' then cnt18 <= (others => '0'); else cnt18 <= cnt18 + 1; end if;
                end if;
                hs_c(0) <= '0';
                if hv_c(18) = '1' and cnt18(2 downto 0) = "000" then
                    hist_ra <= ys18(7 downto 2); hs_c(0) <= '1';
                    hcnt <= hcnt + 1;
                end if;
                hs_c(1) <= hs_c(0); hs_c(2) <= hs_c(1);
                hq20 <= hist_q;
                tmp_ra <= aline(10 downto 6) & cnt18(9 downto 6);
                tmp_c <= tmp_q;

                ----------------------------------------------------------
                -- STAGE 18..21: QUANTIZER.  bin = count of thresholds passed; a
                -- sample within C_HYST of a threshold may take the neighbouring bin
                -- (temporal vote, then left-neighbour vote) -> label {lum, hue, sat}
                ----------------------------------------------------------
                -- stage 18: thresholds passed (from ys17: one stage ahead of ys18, so the
                -- hysteresis arithmetic gets a stage of its own)
                for k in 1 to 7 loop
                    if thr(k)(8) = '0' and ys17 > thr(k)(7 downto 0) then q18_h(k - 1) <= '1'; else q18_h(k - 1) <= '0'; end if;
                end loop;
                -- stage 19: the bin, and the two thresholds around the sample
                v_b := f_pop7(q18_h);
                q19_b <= v_b;
                q19_tlo <= thr(to_integer(v_b));
                if v_b = 7 then q19_thi <= to_unsigned(511, 9); else q19_thi <= thr(to_integer(v_b) + 1); end if;
                ys19 <= ys18;
                -- stage 20: is the sample within C_HYST of the threshold below / above?
                q20_b <= q19_b;
                if q19_b /= 0 and resize(ys19, 9) - q19_tlo <= C_HYST then q20_lo <= '1'; else q20_lo <= '0'; end if;
                if q19_b /= 7 and q19_thi(8) = '0' and q19_thi - resize(ys19, 9) < C_HYST then q20_hi <= '1'; else q20_hi <= '0'; end if;
                -- stage 21: near a threshold the neighbouring bin is also a candidate; last
                -- frame's label of this cell wins if it is one, else the left
                -- neighbour's, else the plain bin
                v_blo := q20_b; v_bhi := q20_b;
                if q20_lo = '1' then v_blo := q20_b - 1; end if;
                if q20_hi = '1' then v_bhi := q20_b + 1; end if;
                v_lum := q20_b;
                if C_HAS_TEMPORAL and v_blo /= v_bhi
                   and unsigned(tmp_c(7 downto 5)) >= v_blo and unsigned(tmp_c(7 downto 5)) <= v_bhi then
                    v_lum := unsigned(tmp_c(7 downto 5));
                elsif v_blo /= v_bhi and lab21(7 downto 5) >= v_blo and lab21(7 downto 5) <= v_bhi then
                    v_lum := lab21(7 downto 5);
                end if;
                lab21 <= v_lum & unsigned(dr_c(4 downto 0));

                ----------------------------------------------------------
                -- STAGE 22..26: CLEANUP, horizontal 5-sample mode of the luma bin and of the
                -- chroma class {hue, sat}
                -- (= the minimum-run-length filter: a label survives only with
                -- 3 of the 5 samples around it)
                ----------------------------------------------------------
                if hf_c(21) = '1' then
                    for i in 0 to 5 loop mw(i) <= lab21; end loop;
                else
                    mw(0) <= lab21;
                    for i in 1 to 5 loop mw(i) <= mw(i - 1); end loop;
                end if;
                for i in 0 to 4 loop v_s5(i) := mw(i)(7 downto 5); v_c5(i) := mw(i)(4 downto 0); end loop;
                mh_cl <= f_cnt5(v_s5); mh_cc <= f_cnt5c(v_c5);
                -- the counts were taken one step ago, so the sample picked at
                -- index p now sits at mw(p + 1)
                rm26 <= mw(f_pick5(mh_cl) + 1)(7 downto 5) & mw(f_pick5(mh_cc) + 1)(4 downto 0);
                if hv_c(25) = '1' then
                    if hf_c(25) = '1' then cnt26 <= (others => '0'); else cnt26 <= cnt26 + 1; end if;
                end if;
                cnt26_d <= cnt26;

                ----------------------------------------------------------
                -- STAGE 27..29: CLEANUP, vertical 5-row mode -> cl29.  Rows 0..2
                -- are row-modes (this row, r-1, centre r-2); rows 3, 4 are the
                -- already cleaned rows above (see p_lbl).
                ----------------------------------------------------------
                mv(0) <= rm26; mv(1) <= unsigned(lbl_q1); mv(2) <= unsigned(lbl_q2);
                mv(3) <= unsigned(lbl_q3); mv(4) <= unsigned(lbl_q4);
                for i in 0 to 4 loop v_s5(i) := mv(i)(7 downto 5); v_c5(i) := mv(i)(4 downto 0); end loop;
                mv_cl <= f_cnt5(v_s5); mv_cc <= f_cnt5c(v_c5);
                mvc <= mv;
                cl29 <= mvc(f_pick5(mv_cl))(7 downto 5) & mvc(f_pick5(mv_cc))(4 downto 0);
                ab29 <= mvc(3);                                  -- the cleaned row above

                ----------------------------------------------------------
                -- STAGE 30..37: label delay, matte luma (bin centroid), the
                -- fill lookups (set ROM -> recipe at 33, colours at 36)
                ----------------------------------------------------------
                ld(30) <= cl29;
                for i in 31 to 34 loop ld(i) <= ld(i - 1); end loop;
                ym35 <= cen(to_integer(ld(34)(7 downto 5)));
                lum35 <= ld(34)(7 downto 5);
                -- slot = which of the set's 8 entries this region uses: its luma
                -- bin, or (Patchwork) a mix of bin, hue and saturation
                if c_col = "11" then
                    slot32 <= ld(31)(7 downto 5) + ld(31)(4 downto 2) + (ld(31)(3 downto 2) & '0') + ('0' & ld(31)(1 downto 0));
                else
                    slot32 <= ld(31)(7 downto 5);
                end if;
                -- recipe word (label at 32 -> stage 33)
                r33_w <= srom_q;
                -- colour word (captured in the F domain one clock later) -> stage 34
                wc34 <= wc_f;
                -- stage 35: the two colours as (hue16, saturation) codes
                --   Source    : the region's own colour; accent = neighbour hue, one step richer
                --   Tint      : the region's hue turned by the set's base hue
                --   Palette / Patchwork : the set's colours
                v_h1 := ld(34)(4 downto 2) & '1';
                case c_col is
                    when "00" =>
                        v_ha := v_h1; c35_sa <= ld(34)(1 downto 0);
                        if ld(34)(1 downto 0) = "00" then
                            v_hb := v_h1; c35_sb <= "00";
                        else
                            v_hb := v_h1 + unsigned(resize(signed(wc34(3 downto 1)), 4));
                            if wc34(0) = '1' then c35_sb <= "10"; else c35_sb <= ld(34)(1 downto 0); end if;
                        end if;
                    when "01" =>
                        v_ha := v_h1 + unsigned(wc34(15 downto 12));
                        if ld(34)(1 downto 0) = "00" then c35_sa <= "01"; else c35_sa <= ld(34)(1 downto 0); end if;
                        v_hb := v_ha + unsigned(resize(signed(wc34(3 downto 1)), 4));
                        c35_sb <= "10";
                    when others =>
                        v_ha := unsigned(wc34(15 downto 12)); c35_sa <= unsigned(wc34(11 downto 10));
                        v_hb := unsigned(wc34(9 downto 6));   c35_sb <= unsigned(wc34(5 downto 4));
                end case;
                c35_ha <= v_ha + c_hrot; c35_hb <= v_hb + c_hrot;      -- K2 Hue
                -- stage 36: chroma A and the step A -> B
                v_ca := f_hue_uv(c35_ha, c35_sa); v_cb := f_hue_uv(c35_hb, c35_sb);
                ua36 <= unsigned(v_ca(15 downto 8)); va36 <= unsigned(v_ca(7 downto 0));
                du36 <= signed(resize(unsigned(v_cb(15 downto 8)), 9)) - signed(resize(unsigned(v_ca(15 downto 8)), 9));
                dv36 <= signed(resize(unsigned(v_cb(7 downto 0)), 9)) - signed(resize(unsigned(v_ca(7 downto 0)), 9));

                ----------------------------------------------------------
                -- STAGE 30..35: INK.  Outline = the cleaned label differs from
                -- the left / right neighbour or the cleaned row above; sketch and
                -- contour come back from the ink history RAM (two lines / one
                -- line old) so they land on the matte row.
                ----------------------------------------------------------
                cw3(0) <= cl29; cw3(1) <= cw3(0); cw3(2) <= cw3(1);
                if hf_c(29) = '1' then cw3(1) <= cl29; cw3(2) <= cl29; end if;
                if hv_c(29) = '1' then
                    if hf_c(29) = '1' then cnt30 <= (others => '0'); else cnt30 <= cnt30 + 1; end if;
                end if;
                cnt30_d <= cnt30; cnt30_dd <= cnt30_d;
                -- (APPLIQUE: only the label fields that change the fill make a seam: c_lmask)
                if ((cl29 xor ab29) and c_lmask) /= 0 then da30 <= '1'; else da30 <= '0'; end if;
                da31 <= da30;
                -- (cw3 at step v+31 holds (v+1, v, v-1); da31 = differs from above (v))
                if ((cw3(1) xor cw3(2)) and c_lmask) /= 0 or da31 = '1' then t32 <= '1'; else t32 <= '0'; end if;
                if ((cw3(1) xor cw3(0)) and c_lmask) /= 0 then dr32 <= '1'; else dr32 <= '0'; end if;
                inkq32 <= lbink_q;
                -- 3-wide windows for the outline footprint (centre = sr(1))
                srt(0) <= t32; srt(1) <= srt(0); srt(2) <= srt(1);
                srdr(0) <= dr32; srdr(1) <= srdr(0); srdr(2) <= srdr(1);
                srt1(0) <= inkq32(1); srt1(1) <= srt1(0); srt1(2) <= srt1(1);
                srt2(0) <= inkq32(0); srt2(1) <= srt2(0); srt2(2) <= srt2(1);
                srxs(0) <= unsigned(inkq32(5 downto 4)); srxs(1) <= srxs(0);
                srxc(0) <= unsigned(inkq32(3 downto 2)); srxc(1) <= srxc(0);
                if hf_c(32) = '1' then
                    srt(1) <= t32; srt(2) <= t32; srdr(1) <= dr32; srdr(2) <= dr32;
                    srt1(1) <= inkq32(1); srt1(2) <= inkq32(1); srt2(1) <= inkq32(0); srt2(2) <= inkq32(0);
                end if;
                -- stage 35: outline bit by footprint, sketch and contour alpha: the
                -- three layers are independent, the strongest ink wins
                if c_width = "00" then
                    v_t := srt(1);
                elsif c_width = "01" then
                    v_t := srt(1) or srt(2) or srt1(1) or srdr(1);
                else
                    v_t := srt(0) or srt(1) or srt(2) or srdr(0) or srdr(1) or srdr(2)
                           or srt1(0) or srt1(1) or srt1(2) or srt2(0) or srt2(1) or srt2(2);
                end if;
                if c_lout = '1' and v_t = '1' then v_oa := "11"; else v_oa := "00"; end if;
                if c_sk_on = '1' and srxs(1) > v_oa then v_oa := srxs(1); end if;
                if c_ct_on = '1' and srxc(1) > v_oa then v_oa := srxc(1); end if;
                ia35 <= v_oa;

                ----------------------------------------------------------
                -- hand-off: stage 36 (next) and 37 (current)
                ----------------------------------------------------------
                ho_n_y <= ym35; ho_n_lum <= lum35; ho_n_a <= ia35;
                ua37 <= ua36; va37 <= va36;
                ho_c_y <= ho_n_y; ho_c_lum <= ho_n_lum; ho_c_a <= ho_n_a;
            end if;

            -- histogram sample count of the frame (the adaptive placement needs
            -- the total); frame anchor is in vblank, the streams are idle
            if vs_edge = '1' then
                htot <= hcnt; hcnt <= (others => '0');
            end if;
        end if;
    end process p_h;

    -- set ROM: 2 words per (set, slot): word 0 = recipe while ce = '0' (read
    -- back by the H register r33 at the next enable), word 1 = colours (caught
    -- here one clock later, into the H register wc34 an enable after that)
    p_srom : process(clk)
    begin
        if rising_edge(clk) then
            srom_q <= C_SROM(to_integer(c_set & slot32 & ce));
            if ce = '0' then wc_f <= srom_q; end if;
        end if;
    end process p_srom;

    ------------------------------------------------------------------------
    -- LINE BLOCK on the raw rows.  g1 = 3x3 binomial of rows r-3..r-1 (centre
    -- r-2): it is both the SMOOTHER of the matte path and the fine scale of
    -- the sketch.  c3s (the vertical [1 2 1] column sums) is shared with the
    -- contour's Sobel.
    ------------------------------------------------------------------------
    p_g1 : process(clk)
    begin
        if rising_edge(clk) then
            if ce = '1' then
                x2_c3 <= resize(ny(1), 10) + resize(ny(3), 10) + shift_left(resize(ny(2), 10), 1);
                c3s(0) <= x2_c3; c3s(1) <= c3s(0); c3s(2) <= c3s(1);
                if hf_c(2) = '1' then c3s(1) <= x2_c3; c3s(2) <= x2_c3; end if;
                -- c3 window centre u at step u+4 -> g1 at stage 5
                x5_g1 <= resize(c3s(0), 12) + shift_left(resize(c3s(1), 12), 1) + resize(c3s(2), 12);
            end if;
        end if;
    end process p_g1;

    ------------------------------------------------------------------------
    -- SKETCH: DoG ink.  3x3 vs 5x5 binomial in 1/16 code, ink on the dark side
    -- where -DoG >= threshold (6.0 .. 1.6 codes over K2; the 99th percentile of
    -- |DoG| on a photo is ~10 codes), alpha 1..3 over the next 2 codes, at
    -- stage 10; then the delay ring (-> 22) and ten registers (-> 32).
    ------------------------------------------------------------------------
    g_sketch : if C_HAS_SKETCH generate
        signal x2_a, x2_b : unsigned(8 downto 0) := (others => '0');
        signal x2_y2 : unsigned(7 downto 0) := (others => '0');
        signal x3_c5 : unsigned(9 downto 0) := (others => '0');          -- vertical [1 4 6 4 1] / 4
        type t_c5s is array (0 to 4) of unsigned(9 downto 0);
        signal c5s : t_c5s := (others => (others => '0'));
        signal x6_g1, x7_g1, x8_g1 : unsigned(11 downto 0) := (others => '0');
        signal x7_a, x7_b : unsigned(10 downto 0) := (others => '0');
        signal x7_c : unsigned(9 downto 0) := (others => '0');
        signal x8_g2 : unsigned(13 downto 0) := (others => '0');         -- 5x5 sum / 4
        signal x9_d : signed(12 downto 0) := (others => '0');            -- DoG, 1/16 code
        type t_xa_d is array (23 to 32) of unsigned(1 downto 0);
        signal xa_d : t_xa_d := (others => (others => '0'));
    begin
        p_sketch : process(clk)
            variable v_t : signed(12 downto 0);
            variable v_c5 : unsigned(11 downto 0);
        begin
            if rising_edge(clk) then
                if ce = '1' then
                    x2_a <= resize(ny(0), 9) + resize(ny(4), 9);
                    x2_b <= resize(ny(1), 9) + resize(ny(3), 9);
                    x2_y2 <= ny(2);
                    v_c5 := resize(x2_a, 12) + shift_left(resize(x2_b, 12), 2) + shift_left(resize(x2_y2, 12), 2) + shift_left(resize(x2_y2, 12), 1);
                    x3_c5 <= v_c5(11 downto 2);
                    c5s(0) <= x3_c5;
                    for i in 1 to 4 loop c5s(i) <= c5s(i - 1); end loop;
                    if hf_c(3) = '1' then for i in 1 to 4 loop c5s(i) <= x3_c5; end loop; end if;
                    -- g1 (stage 5) waits for the 5-wide window: c5 centre at u+6 -> g2 at 8
                    x6_g1 <= x5_g1; x7_g1 <= x6_g1; x8_g1 <= x7_g1;
                    x7_a <= resize(c5s(0), 11) + resize(c5s(4), 11);
                    x7_b <= resize(c5s(1), 11) + resize(c5s(3), 11);
                    x7_c <= c5s(2);
                    x8_g2 <= resize(x7_a, 14) + shift_left(resize(x7_b, 14), 2) + shift_left(resize(x7_c, 14), 2) + shift_left(resize(x7_c, 14), 1);
                    -- DoG in 1/16 code: g1 (sum of 16) - g2 / 16
                    x9_d <= signed(resize(x8_g1, 13)) - signed(resize(x8_g2(13 downto 2), 13));
                    -- S10 (ink build): strokes on both sides of an edge (|DoG|)
                    if s_both = '1' and x9_d > 0 then v_t := x9_d - signed(resize(c_xthr, 13));
                    else v_t := -x9_d - signed(resize(c_xthr, 13)); end if;
                    if v_t < 0 then xa10 <= (others => '0');
                    elsif v_t >= 32 then xa10 <= "11";           -- 2+ codes past the threshold
                    elsif v_t >= 16 then xa10 <= "10";
                    else xa10 <= "01"; end if;
                    xa_d(23) <= xa22;
                    for i in 24 to 32 loop xa_d(i) <= xa_d(i - 1); end loop;
                end if;
            end if;
        end process p_sketch;
        xa32 <= xa_d(32);
    end generate;
    g_nosketch : if not C_HAS_SKETCH generate
        xa10 <= "00"; xa32 <= "00";
    end generate;

    ------------------------------------------------------------------------
    -- CONTOUR: Canny-lite.  Sobel on the centre row r-2, |gx|+|gy| with a
    -- pre-gain (K2) to 4 bits, two magnitude rows in one RAM, non-maximum
    -- suppression along the quantized direction, hysteresis from the left and
    -- the row above.  Alpha at stage 13, one line behind the Sobel row.
    ------------------------------------------------------------------------
    g_canny : if C_HAS_CANNY generate
        type t_lbm is array (0 to 1023) of std_logic_vector(7 downto 0);
        type t_lbd is array (0 to 1023) of std_logic_vector(3 downto 0);
        signal lbmag : t_lbm := (others => (others => '0'));
        signal lbdir : t_lbd := (others => (others => '0'));
        attribute no_rw_check of lbmag : signal is true;
        attribute no_rw_check of lbdir : signal is true;
        signal lbmag_q : std_logic_vector(7 downto 0) := (others => '0');
        signal lbdir_q : std_logic_vector(3 downto 0) := (others => '0');
        signal n2_gy : signed(8 downto 0) := (others => '0');
        type t_g3s is array (0 to 2) of signed(8 downto 0);
        signal gys : t_g3s := (others => (others => '0'));
        signal n5_gx, n5_gy : signed(11 downto 0) := (others => '0');
        signal n6_ax, n6_ay : unsigned(10 downto 0) := (others => '0');
        signal n6_sx, n6_sy : std_logic := '0';
        signal n7_mag : unsigned(7 downto 0) := (others => '0');
        signal n7_dir : unsigned(1 downto 0) := (others => '0');
        signal n8_mp : unsigned(12 downto 0) := (others => '0');
        signal n8_dir, n9_dir, n10_dir, n11_dir, n12_dir, n13_dir, n14_dir : unsigned(1 downto 0) := (others => '0');
        signal n9_m4, n10_m0, n11_m0, n11_m1 : unsigned(3 downto 0) := (others => '0');
        signal cnt9, cnt9_d, cnt9_dd, cnt14 : unsigned(9 downto 0) := (others => '0');
        signal n14_acc : std_logic := '0';
        signal n10_mq : std_logic_vector(7 downto 0) := (others => '0');
        signal n10_dq : std_logic_vector(3 downto 0) := (others => '0');
        type t_m3 is array (0 to 2) of unsigned(3 downto 0);
        type t_d3 is array (0 to 2) of unsigned(1 downto 0);
        type t_b3 is array (0 to 2) of std_logic;
        signal w0, w1, w2 : t_m3 := (others => (others => '0'));
        signal wd : t_d3 := (others => (others => '0'));
        signal wa : t_b3 := (others => '0');
        signal n12_nms, n12_strong, n12_weak, n12_accA : std_logic := '0';
        signal n12_mc : unsigned(3 downto 0) := (others => '0');
        signal n13_acc : std_logic := '0';
        signal ca13 : unsigned(1 downto 0) := (others => '0');
        type t_ca_d is array (15 to 32) of unsigned(1 downto 0);
        signal ca_d : t_ca_d := (others => (others => '0'));
    begin
        p_canny_ram : process(clk)
        begin
            if rising_edge(clk) then
                lbmag_q <= lbmag(to_integer(cnt9));
                lbdir_q <= lbdir(to_integer(cnt9));
                -- magnitude rows {rho, rho-1} written at stage 11; direction of the
                -- Sobel row + accept of the NMS row written at stage 14 (read next line)
                if ce_d = '1' and hv_c(11) = '1' then
                    lbmag(to_integer(cnt9_dd)) <= std_logic_vector(n11_m0) & std_logic_vector(n11_m1);
                end if;
                if ce_d = '1' and hv_c(14) = '1' then
                    lbdir(to_integer(cnt14)) <= std_logic_vector(n14_dir) & n14_acc & '0';
                end if;
            end if;
        end process p_canny_ram;

        p_canny : process(clk)
            variable v_na, v_nb : unsigned(3 downto 0);
            variable v_m : unsigned(11 downto 0);
        begin
            if rising_edge(clk) then
                if ce = '1' then
                    -- stage 2: vertical difference of the newest column (rows r-3, r-1)
                    n2_gy <= signed(resize(ny(3), 9)) - signed(resize(ny(1), 9));
                    gys(0) <= n2_gy; gys(1) <= gys(0); gys(2) <= gys(1);
                    if hf_c(2) = '1' then gys(1) <= n2_gy; gys(2) <= n2_gy; end if;
                    -- stage 5: gradients for the centre column (window centre at step u+4)
                    n5_gx <= signed(resize(c3s(0), 12)) - signed(resize(c3s(2), 12));
                    n5_gy <= resize(gys(0), 12) + shift_left(resize(gys(1), 12), 1) + resize(gys(2), 12);
                    -- stage 6: magnitudes and signs
                    if n5_gx < 0 then n6_ax <= unsigned(resize(-n5_gx, 11)); else n6_ax <= unsigned(resize(n5_gx, 11)); end if;
                    if n5_gy < 0 then n6_ay <= unsigned(resize(-n5_gy, 11)); else n6_ay <= unsigned(resize(n5_gy, 11)); end if;
                    n6_sx <= n5_gx(11); n6_sy <= n5_gy(11);
                    -- stage 7: |gx|+|gy| -> 8 bits, 4 directions (same signs -> down-right)
                    v_m := resize(n6_ax, 12) + resize(n6_ay, 12);
                    if v_m(11) = '1' then n7_mag <= (others => '1'); else n7_mag <= v_m(10 downto 3); end if;
                    if shift_left(resize(n6_ay, 13), 1) + resize(shift_right(n6_ay, 1), 13) < resize(n6_ax, 13) then
                        n7_dir <= "00";                                   -- gradient horizontal: vertical edge
                    elsif shift_left(resize(n6_ax, 13), 1) + resize(shift_right(n6_ax, 1), 13) < resize(n6_ay, 13) then
                        n7_dir <= "10";                                   -- gradient vertical
                    elsif n6_sx = n6_sy then
                        n7_dir <= "01";                                   -- down-right
                    else
                        n7_dir <= "11";                                   -- up-right
                    end if;
                    -- stage 8..9: pre-gain -> 4-bit magnitude.  A strong edge (m4 >= 8)
                    -- is a step of 256/gain codes: 128 .. 15 over K2
                    n8_mp <= n7_mag * c_cgain;
                    n8_dir <= n7_dir;
                    if n8_mp(12 downto 8) /= "00000" then n9_m4 <= (others => '1'); else n9_m4 <= n8_mp(7 downto 4); end if;
                    n9_dir <= n8_dir;
                    if hv_c(8) = '1' then
                        if hf_c(8) = '1' then cnt9 <= (others => '0'); else cnt9 <= cnt9 + 1; end if;
                    end if;
                    cnt9_d <= cnt9; cnt9_dd <= cnt9_d;
                    -- stage 10: current row + RAM rows (rho-1, rho-2) and dir(rho-1)
                    n10_m0 <= n9_m4; n10_mq <= lbmag_q; n10_dq <= lbdir_q; n10_dir <= n9_dir;
                    n11_m0 <= n10_m0; n11_m1 <= unsigned(n10_mq(7 downto 4));
                    if hv_c(13) = '1' then
                        if hf_c(13) = '1' then cnt14 <= (others => '0'); else cnt14 <= cnt14 + 1; end if;
                    end if;
                    -- 3-wide windows (centre index 1)
                    w0(0) <= n10_m0; w0(1) <= w0(0); w0(2) <= w0(1);
                    w1(0) <= unsigned(n10_mq(7 downto 4)); w1(1) <= w1(0); w1(2) <= w1(1);
                    w2(0) <= unsigned(n10_mq(3 downto 0)); w2(1) <= w2(0); w2(2) <= w2(1);
                    wd(0) <= unsigned(n10_dq(3 downto 2)); wd(1) <= wd(0); wd(2) <= wd(1);
                    wa(0) <= n10_dq(1); wa(1) <= wa(0); wa(2) <= wa(1);
                    n11_dir <= n10_dir;
                    -- stage 12: non-maximum suppression on the centre row
                    case wd(1) is
                        when "00"   => v_na := w1(0); v_nb := w1(2);
                        when "10"   => v_na := w0(1); v_nb := w2(1);
                        when "01"   => v_na := w0(0); v_nb := w2(2);     -- down-right / up-left
                        when others => v_na := w2(0); v_nb := w0(2);     -- up-right / down-left
                    end case;
                    if w1(1) >= v_na and w1(1) > v_nb then n12_nms <= '1'; else n12_nms <= '0'; end if;
                    if w1(1) >= 8 then n12_strong <= '1'; else n12_strong <= '0'; end if;
                    if w1(1) >= 4 then n12_weak <= '1'; else n12_weak <= '0'; end if;
                    n12_accA <= wa(0) or wa(1) or wa(2);
                    n12_mc <= w1(1);
                    n12_dir <= n11_dir;
                    -- stage 13: hysteresis from the left neighbour and the row above
                    if n12_nms = '1' and (n12_strong = '1' or (n12_weak = '1' and (n13_acc = '1' or n12_accA = '1'))) then
                        n13_acc <= '1';
                        if n12_mc >= 8 then ca13 <= "11"; elsif n12_mc >= 6 then ca13 <= "10"; else ca13 <= "01"; end if;
                    else
                        n13_acc <= '0';
                        ca13 <= (others => '0');
                    end if;
                    n14_acc <= n13_acc;
                    n13_dir <= n12_dir; n14_dir <= n13_dir;
                    ca_d(15) <= ca13;
                    for i in 16 to 32 loop ca_d(i) <= ca_d(i - 1); end loop;
                end if;
            end if;
        end process p_canny;
        ca32 <= ca_d(32);
    end generate;
    g_nocanny : if not C_HAS_CANNY generate
        ca32 <= "00";
    end generate;

    ------------------------------------------------------------------------
    -- per-frame sequencer (vblank): thresholds and bin centroids.
    --   Fixed    : equal steps between 16 and 235
    --   Adaptive : equal-population (percentile) placement in ONE pass over
    --              last frame's 64-bin histogram; the representative luma of a
    --              bin is its median sample.  Both are low-passed 1/4 per frame
    --              and kept monotonic.
    -- Register files are written only through a value bus + a registered
    -- one-hot enable, and every state does one operation (timing).
    ------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_rem : unsigned(4 downto 0);
        variable v_t : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            hist_swe <= '0';
            for k in 1 to 7 loop
                if thr_we(k) = '1' then thr(k) <= thr_wd; end if;
            end loop;
            for k in 0 to 7 loop
                if cen_we(k) = '1' then cen(k) <= cen_wd; end if;
            end loop;
            thr_we <= (others => '0'); cen_we <= (others => '0');
            -- divider by the level count: restoring, 18 steps, 5-bit remainder
            if div_busy = '1' then
                v_rem := dv_rem(3 downto 0) & dv_q(17);
                if v_rem >= to_unsigned(f_nlev, 5) then
                    dv_rem <= v_rem - to_unsigned(f_nlev, 5);
                    dv_q <= dv_q(16 downto 0) & '1';
                else
                    dv_rem <= v_rem;
                    dv_q <= dv_q(16 downto 0) & '0';
                end if;
                if dv_n = 0 then div_busy <= '0'; else dv_n <= dv_n - 1; end if;
            end if;

            if vs_edge = '1' then
                sq <= 1;
            else
                case sq is
                    when 0 => null;
                    when 1 =>
                        -- latch this frame's level count; reload the table when it changes or fixed
                        f_nlev <= c_nlev;
                        if c_nlev /= sq_nprev or s_adapt = '0' then sq_reload <= '1'; else sq_reload <= '0'; end if;
                        sq_nprev <= c_nlev;
                        case c_nlev is
                            when 2 => sq_fdel <= to_unsigned(110, 7);
                            when 3 => sq_fdel <= to_unsigned(73, 7);
                            when 4 => sq_fdel <= to_unsigned(55, 7);
                            when 5 => sq_fdel <= to_unsigned(44, 7);
                            when 6 => sq_fdel <= to_unsigned(37, 7);
                            when 7 => sq_fdel <= to_unsigned(31, 7);
                            when others => sq_fdel <= to_unsigned(27, 7);
                        end case;
                        sq_acc <= to_unsigned(16, 9); sq_k <= 1;
                        sq <= 2;
                    when 2 =>
                        -- fixed thresholds / centres, one bin per pass (k = 1..8)
                        cen_wd <= resize(sq_acc + resize(shift_right(sq_fdel, 1), 9), 8);
                        if sq_k < f_nlev then thr_wd <= sq_acc + resize(sq_fdel, 9);
                        else thr_wd <= to_unsigned(511, 9); end if;
                        if sq_reload = '1' then
                            if sq_k <= f_nlev then cen_we(sq_k - 1) <= '1'; end if;
                            if sq_k <= 7 then thr_we(sq_k) <= '1'; end if;
                        end if;
                        sq_acc <= sq_acc + resize(sq_fdel, 9);
                        sq <= 3;
                    when 3 =>
                        if sq_k = 8 then
                            if s_adapt = '1' and sq_reload = '0' then sq <= 10; else sq <= 0; end if;
                        else
                            sq_k <= sq_k + 1; sq <= 2;
                        end if;
                    when 10 =>
                        -- population step = samples of the frame / levels
                        seq_hist <= '1';
                        dv_q <= htot; dv_rem <= (others => '0');
                        dv_n <= to_unsigned(17, 5); div_busy <= '1';
                        sq <= 12;
                    when 12 =>
                        if div_busy = '0' then
                            sq_step <= dv_q;
                            sq_tt <= dv_q; sq_mt <= '0' & dv_q(17 downto 1);
                            sq_cum <= (others => '0'); sq_i <= (others => '0'); sq_j <= 0; sq_jm <= 0;
                            sq <= 14;
                        end if;
                    when 14 =>
                        hist_sa <= resize(sq_i, 8); sq <= 15;
                    when 15 => sq <= 16;
                    when 16 =>
                        sq_hval <= hist_q; hist_swe <= '1';                 -- read, then clear
                        sq <= 17;
                    when 17 =>
                        sq_cum <= sq_cum + resize(sq_hval, 18);
                        sq <= 18;
                    when 18 =>
                        -- which placement fires for this bin (one compare each)
                        if sq_j < f_nlev - 1 and sq_cum >= sq_tt then pl_thr <= '1'; else pl_thr <= '0'; end if;
                        if sq_jm < f_nlev and sq_cum >= sq_mt then pl_med <= '1'; else pl_med <= '0'; end if;
                        sq <= 22;
                    when 22 =>
                        -- fetch the register being updated (and the one below it)
                        if pl_thr = '1' then
                            pl_old <= thr(sq_j + 1); pl_prev2 <= thr(sq_j) + 2;
                            pl_new <= shift_left(resize(sq_i, 9), 2) + 4;      -- upper edge of this bin
                            sq <= 23;
                        elsif pl_med = '1' then
                            pl_old <= '0' & cen(sq_jm); pl_prev2 <= (others => '0');
                            pl_new <= shift_left(resize(sq_i, 9), 2) + 2;      -- centre of this bin
                            sq <= 23;
                        elsif sq_i = 63 then
                            sq <= 19;
                        else
                            sq_i <= sq_i + 1; sq <= 14;
                        end if;
                    when 23 =>
                        if pl_new > 235 then pl_new <= to_unsigned(235, 9); end if;
                        sq <= 24;
                    when 24 =>
                        pl_dif <= signed(resize(pl_new, 10)) - signed(resize(pl_old, 10));
                        sq <= 25;
                    when 25 =>
                        -- low-pass 1/4 of the way (previews: all the way)
                        if C_SIM_FAST then pl_lp <= signed(resize(pl_new, 11));
                        else pl_lp <= signed(resize(pl_old, 11)) + resize(shift_right(pl_dif, 2), 11); end if;
                        sq <= 26;
                    when 26 =>
                        -- monotonic (thresholds only) ...
                        if pl_thr = '1' and pl_lp < signed(resize(pl_prev2, 11)) then pl_lp <= signed(resize(pl_prev2, 11)); end if;
                        sq <= 27;
                    when 27 =>
                        -- ... then clamped onto the bus
                        v_t := pl_lp;
                        if v_t > 235 then v_t := to_signed(235, 11); end if;
                        if pl_thr = '1' then
                            thr_wd <= unsigned(v_t(8 downto 0)); thr_we(sq_j + 1) <= '1';
                            sq_tt <= sq_tt + sq_step; sq_j <= sq_j + 1;
                        else
                            cen_wd <= unsigned(v_t(7 downto 0)); cen_we(sq_jm) <= '1';
                            sq_mt <= sq_mt + sq_step; sq_jm <= sq_jm + 1;
                        end if;
                        sq <= 20;                                              -- land the write
                    when 20 => sq <= 18;                                   -- write landed; re-check
                    when 19 =>
                        -- anything not reached: white
                        seq_hist <= '0';
                        if sq_j < f_nlev - 1 then
                            thr_wd <= to_unsigned(235, 9); thr_we(sq_j + 1) <= '1'; sq_j <= sq_j + 1; sq <= 21;
                        elsif sq_jm < f_nlev then
                            cen_wd <= to_unsigned(235, 8); cen_we(sq_jm) <= '1'; sq_jm <= sq_jm + 1; sq <= 21;
                        else
                            sq <= 0;
                        end if;
                    when 21 => sq <= 19;
                    when others => sq <= 0;
                end case;
            end if;
        end if;
    end process p_seq;

    ------------------------------------------------------------------------
    -- PATTERN COORDINATES (F domain, label independent).  The cloth is one
    -- fixed sheet seen through the regions.  Two accumulators in Q11.10
    -- pattern positions (64 positions per cell at size 0): x steps every pixel,
    -- y every active line; the steps carry the Scale knob and the timing's
    -- pixel size (virtual 1080-line units), the start values the Drift.
    -- T0 registers x, y and the bias axes x + y, x - y one clock ahead of T1.
    -- (strobes are registered a clock ahead, as in VENEER: `ce and hf_c(k)`
    -- taken combinationally was a 13 ns routing path)
    ------------------------------------------------------------------------
    p_geo : process(clk)
    begin
        if rising_edge(clk) then
            st_r <= (not ce) and hf_c(32);           -- '1' in the cycle before pixel 0 enters T1
            rc_r <= (not ce) and (not hv_c(32));
            ls_r <= (not ce) and hf_c(1);            -- line start, in blanking
            if run = '1' or st_r = '1' then xacc <= xacc + resize(c_xs, 21); end if;
            if run = '1' then px <= px + 1; end if;
            if rc_r = '1' then run <= '0'; end if;
            if st_r = '1' then run <= '1'; end if;
            if ls_r = '1' then
                run <= '0'; px <= (others => '0');
                xacc <= xoff;
                if yfirst = '1' then yacc <= yoff; else yacc <= yacc + resize(c_ys, 21); end if;
                yfirst <= '0';
            end if;
            -- frame start (vblank): drift, y back to the top
            if vs_edge = '1' then
                run <= '0'; yfirst <= '1';
                xoff <= xoff + shift_left(resize(c_dspd, 21), 7);
                yoff <= yoff + shift_left(resize(c_dspd, 21), 6);
            end if;
            -- T0
            g_x <= xacc(20 downto 10); g_y <= yacc(20 downto 10);
            g_u <= xacc(20 downto 8) + yacc(20 downto 8);
            g_v <= xacc(20 downto 8) - yacc(20 downto 8);
        end if;
    end process p_geo;

    -- relief line buffer.  Address A = px / 2 (px runs 4 ahead of the pixel
    -- whose t is at T4, a constant offset).  Read at A + 2 (not yet rewritten on
    -- this line), three pair delays -> rl_d2 = the word of A - 1: the sample 2..3
    -- pixels to the left, one and two lines up.  Write {t of this line, t of the
    -- line above} at A: read and write addresses are always 2 apart.
    p_relief : process(clk)
    begin
        if rising_edge(clk) then
            rl_q <= rlram(to_integer(px(10 downto 1) + 2));
            if run = '1' and px(0) = '0' then
                rlram(to_integer(px(10 downto 1))) <= std_logic_vector(f4_t(7 downto 6)) & rl_d1(3 downto 2);
            end if;
            if px(0) = '1' then rl_d0 <= rl_q; rl_d1 <= rl_d0; rl_d2 <= rl_d1; end if;
        end if;
    end process p_relief;

    ------------------------------------------------------------------------
    -- PATTERN + FILL COLOUR (F domain, per pixel; the recipe and the colours
    -- of the pixel's region are read from the H stages named below.  An F
    -- stage 2k clocks before O1 reads H stage 37 - k for both pixels of a
    -- pair, so H values are only read at even offsets.)
    --   T1 (-8, H 33) axes: straight (x, y) or bias (x + y, x - y), turned or not
    --   T2 (-7)       size (8-bit field: 2 cell bits + 6 position bits), stagger,
    --                 folds (distance from the cell's centre lines)
    --   T3 (-6)       features: sums, differences, compares, squares
    --   T4 (-5)       pattern -> t (16 levels; 0 = tone A, 15 = tone B): bit tests only
    --   T5 (-4)       relief (t against the sample up-left), smoothed t for the colour ramp
    --   T6 (-3)       e = (t - 128) + relief; ramp position m (0..4, dithered)
    --   T7 (-2, H 36) chroma step m/4 x (B - A); luma partial products e x depth
    --   T8 (-1)       chroma = A + step; luma delta
    -- Recipe word: [3:0] pattern [5:4] variant [7:6] size [8] bias [9] turn
    --              [10] stagger [11] invert [12] half stagger   (tables.py)
    ------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_a, v_b : unsigned(10 downto 0);
        variable v_fa, v_fb : unsigned(7 downto 0);
        variable v_pa, v_pb, v_xp, v_x : unsigned(5 downto 0);
        variable v_da, v_db : unsigned(4 downto 0);
        variable v_sqa, v_sqb : unsigned(9 downto 0);
        variable v_t : unsigned(3 downto 0);
        variable v_p5, v_q5, v_tw0, v_tw1, v_c, v_d : std_logic;
        variable v_line, v_ja, v_jb, v_m0, v_plus, v_star : boolean;
        variable v_la, v_lb : unsigned(3 downto 0);
        variable v_s3 : unsigned(2 downto 0);
        variable v_ref : unsigned(1 downto 0);
        variable v_r3 : signed(2 downto 0);
        variable v_g1, v_g2 : signed(8 downto 0);
        variable v_q : signed(12 downto 0);
        variable v_ts : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------
            -- T1: axes
            ----------------------------------------------------------
            if (r33_w(8) xor s_bias) = '1' then v_a := g_u(12 downto 2); v_b := g_v(12 downto 2);
            else v_a := g_x; v_b := g_y; end if;
            if r33_w(9) = '1' then f1_a <= v_b; f1_b <= v_a; else f1_a <= v_a; f1_b <= v_b; end if;
            f1_typ <= r33_w(3 downto 0); f1_var <= r33_w(5 downto 4); f1_size <= r33_w(7 downto 6);
            f1_stg <= r33_w(10); f1_inv <= r33_w(11); f1_hst <= r33_w(12);

            ----------------------------------------------------------
            -- T2: size (a case of fixed slices, no barrel), stagger, folds
            ----------------------------------------------------------
            case f1_size is
                when "00"   => v_fa := f1_a(7 downto 0);  v_fb := f1_b(7 downto 0);
                when "01"   => v_fa := f1_a(8 downto 1);  v_fb := f1_b(8 downto 1);
                when "10"   => v_fa := f1_a(9 downto 2);  v_fb := f1_b(9 downto 2);
                when others => v_fa := f1_a(10 downto 3); v_fb := f1_b(10 downto 3);
            end case;
            v_pa := v_fa(5 downto 0);
            -- every other row of cells (or half row) slides half a cell
            if ((f1_stg and v_fb(6)) xor (f1_hst and v_fb(5))) = '1' then v_pa(5) := not v_pa(5); end if;
            f2_pa <= v_pa; f2_pb <= v_fb(5 downto 0);
            f2_ia0 <= v_fa(6); f2_ib0 <= v_fb(6);
            f2_da <= f_fold(v_pa); f2_db <= f_fold(v_fb(5 downto 0));
            f2_typ <= f1_typ; f2_var <= f1_var; f2_inv <= f1_inv;

            ----------------------------------------------------------
            -- T3: features (every carry chain of the generator is here)
            ----------------------------------------------------------
            f3_pa <= f2_pa; f3_pb <= f2_pb; f3_da <= f2_da; f3_db <= f2_db;
            f3_ia0 <= f2_ia0; f3_ib0 <= f2_ib0;
            f3_gt <= f_b2sl(f2_pa > f2_pb);                                  -- above the cell's diagonal
            f3_sm <= resize(f2_pa, 7) + resize(f2_pb, 7);                   -- a + b
            f3_df <= f2_pa - f2_pb;                                          -- a - b (mod 64)
            f3_hg <= f_b2sl(f2_da > f2_db);
            if f2_da > f2_db then f3_li <= f2_da; else f3_li <= f2_db; end if;   -- square distance
            f3_l1 <= resize(f2_da, 6) + resize(f2_db, 6);                   -- diamond distance
            v_sqa := f_sq5(f2_da); v_sqb := f_sq5(f2_db);
            f3_r2 <= resize(v_sqa, 11) + resize(v_sqb, 11);                 -- round distance, squared
            f3_ch <= f2_pb + resize(f2_da, 6);                              -- chevron phase
            f3_sc <= (not f2_pb(4 downto 0)) - v_sqa(9 downto 5);           -- scallop phase
            f3_gs <= f_b2sl(f2_pb(5 downto 1) > f2_da);                     -- inside the goose triangle
            f3_typ <= f2_typ; f3_var <= f2_var; f3_inv <= f2_inv;

            ----------------------------------------------------------
            -- T4: pattern (lookup-style logic on the features: no carries
            -- except the brick shade)
            ----------------------------------------------------------
            v_pa := f3_pa; v_pb := f3_pb; v_da := f3_da; v_db := f3_db;
            v_xp := v_pa xor v_pb;
            v_p5 := v_pa(5); v_q5 := v_pb(5);
            v_la := v_pa(4 downto 1); v_lb := v_pb(4 downto 1);
            v_line := f3_df(5 downto 2) = "0000" or f3_sm(5 downto 2) = "1111";      -- the cell's two diagonals
            v_t := "1000";
            case f3_typ is
                when "0000" =>                                               -- PLAIN
                    case f3_var is
                        when "00"   => v_t := "1000";
                        when "01"   => v_t := f_sel(v_xp(2), 9, 7);                  -- canvas
                        when "10"   => v_t := f_sel(v_pb(3), 9, 7);                  -- fine rib
                        when others => v_t := f_sel(f3_sm(3), 9, 7);                 -- fine twill
                    end case;
                when "0001" =>                                               -- STRIPE
                    case f3_var is
                        when "00"   => v_t := f_sel(v_p5, 15, 0);
                        when "01"   => v_t := f_sel(v_pa(4), 15, 0);
                        when "10"   => v_t := f_sel(f_b2sl(v_pa(5 downto 3) = "000"), 2, 12);       -- pinstripe
                        when others =>                                                           -- regimental
                            if v_p5 = '0' then v_t := f_sel(v_pa(4), 15, 0);
                            else v_t := f_sel(f_b2sl(v_pa(4 downto 2) = "100"), 15, 6); end if;
                    end case;
                when "0010" =>                                               -- RIDGE (ramps)
                    case f3_var is
                        when "00"   => v_t := v_pa(4 downto 1);
                        when "01"   => v_t := v_da(4 downto 1);
                        when "10"   => v_t := v_pa(5 downto 2);
                        when others =>
                            if v_pa(4) = '1' then v_t := v_pa(3 downto 0); else v_t := not v_pa(3 downto 0); end if;
                    end case;
                when "0011" =>                                               -- CHECK
                    case f3_var is
                        when "00"   => v_t := f_sel(v_xp(5), 15, 0);
                        when "01"   => v_t := f_g3(v_p5, v_q5, 1, 8, 15);                -- gingham
                        when "10"   => v_t := f_sel(f_b2sl(v_pa(5 downto 2) = "0000" or v_pb(5 downto 2) = "0000"), 2, 12);  -- windowpane
                        when others => v_t := f_sel(v_xp(4), 15, 0);
                    end case;
                when "0100" =>                                               -- PLAID
                    case f3_var is
                        when "00"   =>
                            if v_la = "0100" or v_lb = "0100" then v_t := "1111";
                            elsif v_la = "1100" or v_lb = "1100" then v_t := "0000";
                            else v_t := f_g3(v_p5, v_q5, 3, 8, 13); end if;
                        when "01"   =>
                            if v_la = "0100" or v_la = "0110" or v_lb = "0100" or v_lb = "0110" then v_t := "1111";
                            else v_t := f_g3(v_p5, v_q5, 3, 8, 13); end if;
                        when "10"   =>
                            if f3_ia0 = '1' then v_c := v_p5; else v_c := v_p5 and v_pa(4); end if;
                            if f3_ib0 = '1' then v_d := v_q5; else v_d := v_q5 and v_pb(4); end if;
                            if v_la = "0010" or v_lb = "0010" then v_t := "0000";
                            else v_t := f_g3(v_c, v_d, 3, 8, 13); end if;
                        when others =>                                                           -- madras
                            v_s3 := resize(v_pa(5 downto 4), 3) + resize(v_pb(5 downto 4), 3);
                            v_t := v_s3 & '1';
                    end case;
                when "0101" =>                                               -- HOUNDSTOOTH (2/2 twill of 4 + 4 threads)
                    if f3_sm(4) = '0' then v_tw0 := v_p5; else v_tw0 := v_q5; end if;
                    if f3_sm(3) = '0' then v_tw1 := v_p5; else v_tw1 := v_q5; end if;
                    case f3_var is
                        when "00"   => v_t := f_sel(v_tw0, 15, 0);
                        when "01"   => v_t := f_sel(v_tw1, 15, 0);
                        when "10"   =>
                            if (f3_ia0 xor f3_ib0) = '1' then v_t := f_sel(v_tw0, 15, 0); else v_t := f_sel(v_xp(3), 11, 4); end if;
                        when others =>
                            if v_p5 = v_q5 then v_t := f_sel(v_p5, 15, 0); else v_t := f_sel(f3_sm(4), 11, 4); end if;
                    end case;
                when "0110" =>                                               -- DOT
                    case f3_var is
                        when "00"   => v_t := f_sel(f_b2sl(f3_r2(10 downto 8) = "000"), 15, 1);
                        when "01"   => v_t := f_sel(f_b2sl(f3_r2(10 downto 7) = "0000"), 15, 1);
                        when "10"   => v_t := f_sel(f_b2sl(f3_r2(10 downto 9) = "00"), 15, 1);
                        when others => v_t := f_sel(f_b2sl(f3_r2(10 downto 9) = "00" and f3_r2(8 downto 7) /= "00"), 15, 1);   -- ring
                    end case;
                when "0111" =>                                               -- TARGET
                    case f3_var is
                        when "00"   => v_t := f_sel(f3_li(3), 15, 0);
                        when "01"   => v_t := f_sel(f3_li(2), 15, 0);
                        when "10"   => v_t := f_sel(f3_r2(8), 15, 0);
                        when others => v_t := f_sel(f3_l1(3), 15, 0);
                    end case;
                when "1000" =>                                               -- DIAMOND
                    case f3_var is
                        when "00"   => v_t := f_sel(f3_l1(5), 0, 15);
                        when "01"   =>                                                           -- argyle
                            if f3_l1(5) = '0' then v_t := f_sel(f_b2sl(v_line), 0, 12);
                            else v_t := f_sel(f_b2sl(v_line), 15, 3); end if;
                        when "10"   => v_t := f_sel(f_b2sl(f3_l1(5 downto 4) = "00"), 15, 1);
                        when others => v_t := f_sel(f_b2sl(f3_l1(5 downto 3) <= "100"), 15, 0);  -- snowball
                    end case;
                when "1001" =>                                               -- STUD
                    case f3_var is
                        when "00"   => v_t := not f3_li(4 downto 1);
                        when "01"   => v_t := f3_li(4 downto 1);
                        when "10"   =>
                            if f3_r2(10) = '1' then v_t := "0000"; else v_t := not f3_r2(9 downto 6); end if;
                        when others => v_t := (not f3_l1(5 downto 3)) & '1';
                    end case;
                when "1010" =>                                               -- TRIANGLES
                    case f3_var is
                        when "00"   => v_t := f_sel(f3_gt, 15, 0);
                        when "01"   =>                                                           -- pinwheel
                            if f3_ia0 = '0' and f3_ib0 = '0' then v_c := f3_gt;
                            elsif f3_ia0 = '1' and f3_ib0 = '0' then v_c := f3_sm(6);
                            elsif f3_ia0 = '1' then v_c := not f3_gt;
                            else v_c := not f3_sm(6); end if;
                            v_t := f_sel(v_c, 15, 0);
                        when "10"   => v_t := f_sel(f3_hg, 15, 0);                               -- hourglass
                        when others => v_t := f_sel(f3_gs, 15, 0);                               -- geese
                    end case;
                when "1011" =>                                               -- BASKET
                    if f3_var = "10" then v_d := f3_ia0 xor f3_ib0; else v_d := v_xp(5); end if;
                    if v_d = '1' then v_x := v_pa; else v_x := v_pb; end if;
                    case f3_var is
                        when "00"   => v_t := v_x(3 downto 0);
                        when "01"   => v_t := f_sel(f_b2sl(v_x(3 downto 1) = "000"), 1, 12);
                        when "10"   => v_t := v_x(4 downto 1);
                        when others => v_t := f_sel(v_x(3), 12, 4);
                    end case;
                when "1100" =>                                               -- ZIGZAG
                    case f3_var is
                        when "00"   => v_t := f_sel(f3_ch(4), 15, 0);
                        when "01"   =>                                                           -- herringbone
                            if v_p5 = '1' then v_c := f3_sm(3); else v_c := f3_df(3); end if;
                            v_t := f_sel(v_c, 15, 0);
                        when "10"   => v_t := f3_ch(4 downto 1);
                        when others => v_t := f_sel(f3_ch(3), 15, 0);
                    end case;
                when "1101" =>                                               -- BRICK
                    v_ja := v_pa(4 downto 2) = "000"; v_jb := v_pb(4 downto 2) = "000";
                    v_m0 := v_jb or (v_ja and v_p5 = v_q5);
                    case f3_var is
                        when "00"   => v_t := f_sel(f_b2sl(v_m0), 2, 12);
                        when "01"   => v_t := f_sel(f_b2sl(v_ja or v_jb), 2, 12);
                        when "10"   => v_t := f_sel(f_b2sl(v_ja or v_pb(5 downto 2) = "0000"), 2, 12);
                        when others =>
                            if v_m0 then v_t := "0010"; else v_t := resize(v_pb(4 downto 2), 4) + 6; end if;
                    end case;
                when "1110" =>                                               -- SCALE
                    case f3_var is
                        when "00"   => v_t := f_sel(f_b2sl(f3_sc(4 downto 2) = "000"), 2, 12);
                        when "01"   => v_t := f3_sc(4 downto 1);
                        when "10"   => v_t := f_sel(f3_sc(4), 15, 0);
                        when others => v_t := f_sel(f3_sc(2), 13, 3);
                    end case;
                when others =>                                               -- CROSS
                    v_plus := (v_da(4 downto 3) = "00" and v_db(4 downto 3) /= "11") or (v_db(4 downto 3) = "00" and v_da(4 downto 3) /= "11");
                    v_star := (f3_l1(5 downto 4) = "00" and f3_l1(3 downto 2) /= "11") or v_da(4 downto 2) = "000" or v_db(4 downto 2) = "000";
                    case f3_var is
                        when "00"   => v_t := f_sel(f_b2sl(v_plus), 15, 1);
                        when "01"   => v_t := f_sel(f_b2sl(v_line), 2, 12);                      -- lattice
                        when "10"   => v_t := f_bayer(v_pb(5 downto 4) & v_pa(5 downto 4));      -- screen
                        when others => v_t := f_sel(f_b2sl(v_star), 15, 1);
                    end case;
            end case;
            if (f3_inv xor s_inv) = '1' then v_t := not v_t; end if;           -- recipe invert, S9 Invert
            f4_t <= v_t & v_t;

            ----------------------------------------------------------
            -- T5: relief = top two bits of t against the sample 2..3 pixels left
            -- and two lines up (one field line when interlaced), -3..3 -> x21;
            -- smoothed t for the colour ramp (chroma never swings at pixel rate)
            ----------------------------------------------------------
            if c_il = '1' then v_ref := unsigned(rl_d2(3 downto 2)); else v_ref := unsigned(rl_d2(1 downto 0)); end if;
            v_r3 := signed(resize(f4_t(7 downto 6), 3)) - signed(resize(v_ref, 3));
            if px(10 downto 4) = 0 then
                f5_rel <= (others => '0');                                     -- the line's first pixels have no sample yet
            else
                case v_r3 is
                    when "001"  => f5_rel <= to_signed(21, 7);
                    when "010"  => f5_rel <= to_signed(42, 7);
                    when "011"  => f5_rel <= to_signed(63, 7);
                    when "111"  => f5_rel <= to_signed(-21, 7);
                    when "110"  => f5_rel <= to_signed(-42, 7);
                    when "101"  => f5_rel <= to_signed(-63, 7);
                    when others => f5_rel <= (others => '0');
                end case;
            end if;
            f5_tc <= signed(f4_t xor x"80");                                    -- t - 128
            f5_ts <= f5_ts + resize(f4_t, 10) - resize(shift_right(f5_ts, 2), 10);
            f5_px0 <= px(0);

            ----------------------------------------------------------
            -- T6: S10 Relief (x2: the edge is as strong as the two tones)
            ----------------------------------------------------------
            if s_relief = '1' then f6_e <= resize(f5_tc, 9) + shift_left(resize(f5_rel, 9), 1);
            else f6_e <= resize(f5_tc, 9); end if;
            if f5_px0 = '1' then v_ts := resize(f5_ts(9 downto 2), 9) + 48; else v_ts := resize(f5_ts(9 downto 2), 9) + 16; end if;
            f6_m <= v_ts(8 downto 6);

            ----------------------------------------------------------
            -- T7 (H stage 36): one add of gated operands each
            ----------------------------------------------------------
            if f6_m = "100" then v_g1 := du36; elsif f6_m(1) = '1' then v_g1 := shift_right(du36, 1); else v_g1 := (others => '0'); end if;
            if f6_m(2) = '0' and f6_m(0) = '1' then v_g2 := shift_right(du36, 2); else v_g2 := (others => '0'); end if;
            f7_mu <= v_g1 + v_g2;
            if f6_m = "100" then v_g1 := dv36; elsif f6_m(1) = '1' then v_g1 := shift_right(dv36, 1); else v_g1 := (others => '0'); end if;
            if f6_m(2) = '0' and f6_m(0) = '1' then v_g2 := shift_right(dv36, 2); else v_g2 := (others => '0'); end if;
            f7_mv <= v_g1 + v_g2;
            f7_ua <= ua36; f7_va <= va36;
            v_q := (others => '0');
            if c_depth(0) = '1' then v_q := resize(f6_e, 13); end if;
            if c_depth(1) = '1' then f7_q1 <= v_q + shift_left(resize(f6_e, 13), 1); else f7_q1 <= v_q; end if;
            v_q := (others => '0');
            if c_depth(2) = '1' then v_q := shift_left(resize(f6_e, 13), 2); end if;
            if c_depth(3) = '1' then f7_q2 <= v_q + shift_left(resize(f6_e, 13), 3); else f7_q2 <= v_q; end if;
            -- T8
            v_g1 := signed(resize(f7_ua, 9)) + f7_mu; f8_u <= unsigned(v_g1(7 downto 0));
            v_g1 := signed(resize(f7_va, 9)) + f7_mv; f8_v <= unsigned(v_g1(7 downto 0));
            f8_ey <= f7_q1 + f7_q2;
        end if;
    end process p_fill;

    ------------------------------------------------------------------------
    -- F composite.  O1 holds pixel x during cycle t_in + x + 79.
    --   O1 fill colour (H stage 37 + the T8 pattern values), seam alpha
    --   O2/O3 ink over the fill   O4 compose
    --   O5..O7 [1 2 1] on Y/U/V + luma toward black (HD HDMI bar conditioning)
    --   O8 blanking gate, parity-corrected avid
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_a3 : unsigned(2 downto 0);
        variable v_a2 : unsigned(1 downto 0);
        variable v_cy, v_cu, v_cv : unsigned(9 downto 0);
        variable v_dy, v_du, v_dv : signed(10 downto 0);
        variable v_sum : signed(11 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d : unsigned(9 downto 0);
        variable v_av : std_logic;
        variable v_e : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------
            -- O1: matte luma (bin centroid) + pattern, chroma from T8
            ----------------------------------------------------------
            v_e := resize(shift_right(f8_ey, 3), 12);                          -- e x depth / 8
            v_sum := signed(resize(ho_c_y & "00", 12)) + v_e;
            if v_sum < 64 then o1_y <= to_unsigned(64, 10);
            elsif v_sum > 940 then o1_y <= to_unsigned(940, 10);
            else o1_y <= unsigned(v_sum(9 downto 0)); end if;
            o1_u <= f8_u & "00"; o1_v <= f8_v & "00";
            o1_bg <= '0';
            -- ink build S11 Paper: lines only, on blank paper (black paper for white ink)
            if s_paper = '1' then
                if s_white = '1' then o1_y <= to_unsigned(64, 10); else o1_y <= to_unsigned(940, 10); end if;
                o1_u <= C_MID; o1_v <= C_MID; o1_bg <= '0';
            end if;
            -- ink alpha code 0..3 (0 / quarter / half / full); the odd pixel takes
            -- the rounded mean of this and the next sample (horizontal AA)
            v_a3 := resize(ho_c_a, 3) + resize(ho_n_a, 3) + 1;
            if ce = '1' then o1_a <= v_a3(2 downto 1); else o1_a <= ho_c_a; end if;
            o1_noink <= '0';
            if aline < C_TOPG then o1_a <= (others => '0'); o1_bg <= '1'; o1_noink <= '1'; end if;

            ----------------------------------------------------------
            -- O2/O3: ink delta scaled by the alpha code; O4: compose
            ----------------------------------------------------------
            if o1_bg = '1' then
                v_cy := C_BLKY; v_cu := C_MID; v_cv := C_MID;
            else
                v_cy := o1_y; v_cu := o1_u; v_cv := o1_v;
            end if;
            if s_white = '1' then v_dy := to_signed(775, 11) - signed(resize(v_cy, 11));
            else v_dy := to_signed(64, 11) - signed(resize(v_cy, 11)); end if;
            v_du := to_signed(512, 11) - signed(resize(v_cu, 11));
            v_dv := to_signed(512, 11) - signed(resize(v_cv, 11));
            if o1_noink = '1' then v_a2 := "00"; else v_a2 := o1_a; end if;
            case v_a2 is
                when "00"   => o3_py <= (others => '0'); o3_pu <= (others => '0'); o3_pv <= (others => '0');
                when "01"   => o3_py <= shift_right(v_dy, 2); o3_pu <= shift_right(v_du, 2); o3_pv <= shift_right(v_dv, 2);
                when "10"   => o3_py <= shift_right(v_dy, 1); o3_pu <= shift_right(v_du, 1); o3_pv <= shift_right(v_dv, 1);
                when others => o3_py <= v_dy; o3_pu <= v_du; o3_pv <= v_dv;
            end case;
            o3_y <= v_cy; o3_u <= v_cu; o3_v <= v_cv;
            -- the blend is convex (0..1023): SLICE the 12-bit sum, never resize a
            -- signed value to 10 bits (that keeps the sign and drops bit 9)
            v_sum := signed(resize(o3_y, 12)) + resize(o3_py, 12); o4_y <= unsigned(v_sum(9 downto 0));
            v_sum := signed(resize(o3_u, 12)) + resize(o3_pu, 12); o4_u <= unsigned(v_sum(9 downto 0));
            v_sum := signed(resize(o3_v, 12)) + resize(o3_pv, 12); o4_v <= unsigned(v_sum(9 downto 0));

            ----------------------------------------------------------
            -- O5..O7: [1 2 1] on Y/U/V, luma toward black
            ----------------------------------------------------------
            g1_y <= o4_y; g1_u <= o4_u; g1_v <= o4_v;
            g2_y <= g1_y; g2_u <= g1_u; g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(o4_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(o4_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(o4_v, 12);
            o6_y <= v_sy(11 downto 2); o6_u <= v_su(11 downto 2); o6_v <= v_sv(11 downto 2);
            if o6_y > 64 then v_d := o6_y - 64; else v_d := (others => '0'); end if;
            o7_y <= o6_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            o7_u <= o6_u; o7_v <= o6_v;

            ----------------------------------------------------------
            -- O8: blanking gate, parity-corrected avid (ring bits: 0 avid,
            -- 1 hsync, 2 vsync, 3 field, 4 hsync->avid parity differs)
            ----------------------------------------------------------
            rq_avid_d <= ring_q(0);
            if ring_q(0) = '1' and rq_avid_d = '0' then out_sel <= ring_q(4); end if;
            if ring_q(0) = '1' and rq_avid_d = '0' then
                if ring_q(4) = '1' then v_av := '0'; else v_av := '1'; end if;
            elsif out_sel = '1' then v_av := rq_avid_d;
            else v_av := ring_q(0); end if;
            out_avid <= v_av;
            if ring_q(0) = '0' then
                out_y <= C_BLKY; out_u <= C_MID; out_v <= C_MID;
            else
                out_y <= o7_y; out_u <= o7_v; out_v <= o7_u;         -- back to hardware order
            end if;
            out_hs <= ring_q(1); out_vs <= ring_q(2); out_fld <= ring_q(3);
        end if;
    end process p_out;

    data_out.y <= std_logic_vector(out_y);
    data_out.u <= std_logic_vector(out_u);
    data_out.v <= std_logic_vector(out_v);
    data_out.avid <= out_avid;
    data_out.hsync_n <= out_hs;
    data_out.vsync_n <= out_vs;
    data_out.field_n <= out_fld;

end architecture applique;
