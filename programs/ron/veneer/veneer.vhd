-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- VENEER -- re-renders video as flat matte regions (luma levels x chroma
-- classes, cleaned by mode filters) and fills them.  Two builds of this file:
--
--   VENEER (C_HAS_TEX)        Ink fill   : the region's estimated colour, optional wash
--                             Texture    : a generative organic texture per region
--                                          (Stone, Timber, Tide, Moss, Hide, Ember,
--                                          Orchid, Paper); value always from the picture
--   VENEER INK (line layers)  flat ink mattes + three independent ink layers:
--                             Outline (region boundaries), Sketch (DoG strokes),
--                             Contour (Canny-lite)
--
-- The matte pipeline is ~62% of the HX4K, the texture machinery ~30% and the
-- two fine-line layers ~25%, so one bitstream cannot hold both; variants.py
-- stamps the build constants below.  Built from the FLATLINE proof of concept;
-- read DESIGN.md first.
--
-- Controls            VENEER                         VENEER INK
--   K1                Texture set (8)                Palette (8)
--   K2                Hue (16 steps)                 Hue
--   K3                Depth (wash / contrast)        Sketch amount (0 = off)
--   K4                Colour: Source / Tint / Palette / Patchwork     (both)
--   K5                Drift (px per frame)           Contour amount (0 = off)
--   K6                Relief: Flat..Deep             Outline: Off / Thin / Medium / Bold
--   S7                Fill: Ink / Texture            Ground: Black / Colour
--   S8                Thresholds: Fixed / Adaptive   (both)
--   S9                Invert texture                 Ink: Black / White
--   S10               Light: Left / Right            Strokes: Dark Side / Both
--   S11               Detail: Normal / Dense         Paper: Colour / Blank
--   P12               Levels 2..8                    (both)
--
-- Architecture: an F domain (every clock) and an H domain (one step per pixel
-- pair, clock enable `ce`).  The matte analysis runs at half horizontal
-- resolution so every line buffer is 1024 deep (chroma 512).  Stage offsets in
-- comments are H steps after the capture of column m ("stage k holds column m
-- during step m+k").  The texture fields are label independent and run at the
-- full pixel rate beside the late H stages.  Latency 84 clocks, all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture veneer of program_top is

    ------------------------------------------------------------------------
    -- BUILD CONFIGURATION (stamped per variant by variants.py; edit the master
    -- in programs/ron/veneer/ and restamp)
    ------------------------------------------------------------------------
    constant C_VARIANT      : string  := "T";      -- T = VENEER (texture), I = VENEER INK
    constant C_HAS_OUTLINE  : boolean := false;    -- region outline layer
    constant C_HAS_SKETCH   : boolean := false;    -- DoG sketch layer
    constant C_HAS_CANNY    : boolean := false;    -- Canny-lite contour layer
    constant C_HAS_TEX      : boolean := true;     -- texture fill
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
    -- ROM contents (generated: programs/ron/veneer/tables.py rewrites this block)
    ------------------------------------------------------------------------
    -- TABLES BEGIN
    type t_rom16 is array (0 to 255) of std_logic_vector(15 downto 0);
    -- noise ROM: 256 random words
    constant C_NROM : t_rom16 := (
        x"2DBC", x"04AB", x"D584", x"2170", x"5D34", x"EB49", x"A32F", x"E968",
        x"0276", x"753B", x"10D1", x"83EE", x"21EB", x"1854", x"0C65", x"6437",
        x"0D57", x"8D6E", x"1C16", x"99A4", x"4E08", x"07F4", x"4CAE", x"E2D4",
        x"7FEC", x"0397", x"A717", x"ADF5", x"BE51", x"DC8F", x"AFC4", x"4DB6",
        x"8118", x"B350", x"F172", x"6F21", x"F911", x"42FA", x"CD28", x"0C00",
        x"7CE0", x"D8F6", x"C314", x"64F9", x"551B", x"AF28", x"BA44", x"4863",
        x"225C", x"CDE7", x"2A76", x"E636", x"DBD9", x"E076", x"1255", x"5B7C",
        x"7B59", x"C055", x"CC54", x"EC3B", x"64E1", x"450E", x"C976", x"652C",
        x"5052", x"8660", x"0CC8", x"848A", x"7088", x"A966", x"BCB2", x"6AF6",
        x"5A0E", x"8AFD", x"7766", x"1946", x"4946", x"D66D", x"4668", x"06CB",
        x"0031", x"BDDD", x"284D", x"B344", x"29A4", x"ACF3", x"AC57", x"F429",
        x"D92B", x"2013", x"1376", x"7961", x"F931", x"5F1D", x"F9A1", x"24FE",
        x"835D", x"37FB", x"62D0", x"515C", x"AD91", x"3DDD", x"3500", x"6B42",
        x"535D", x"17FC", x"DDA3", x"9499", x"C150", x"F5F7", x"DE05", x"175B",
        x"F0F6", x"855B", x"2810", x"D32C", x"9B30", x"3FA5", x"B686", x"80E8",
        x"6F3A", x"D0CA", x"B6BB", x"B659", x"9E23", x"BC2A", x"103B", x"CF14",
        x"237D", x"D944", x"04BB", x"CAFB", x"B51C", x"9CF6", x"8C82", x"A56E",
        x"999F", x"D757", x"01DF", x"D73B", x"EE4B", x"E573", x"C3E4", x"2C4E",
        x"E16D", x"6862", x"D9A9", x"9C29", x"F8DD", x"AAC8", x"22A2", x"C110",
        x"878B", x"342B", x"F927", x"7C6E", x"4FA2", x"A50B", x"E606", x"0D82",
        x"A4B0", x"8BA7", x"F4C6", x"4EE5", x"007B", x"B620", x"946A", x"36C6",
        x"BF29", x"FDCA", x"64E9", x"5B9E", x"CA41", x"8078", x"5F7A", x"F633",
        x"0C72", x"9FA8", x"C3FD", x"4FFB", x"1AA7", x"6A97", x"3B89", x"C670",
        x"C69D", x"2963", x"21C5", x"861D", x"5994", x"3FF9", x"D38C", x"C4AC",
        x"0B97", x"9FE1", x"E5DC", x"3408", x"EFD2", x"1293", x"537F", x"B0CE",
        x"363F", x"8F46", x"DC72", x"2170", x"F46D", x"9B4C", x"6910", x"28EA",
        x"A511", x"8478", x"5722", x"137B", x"ED04", x"6446", x"E852", x"3D37",
        x"0523", x"E36C", x"440B", x"DE24", x"1138", x"903D", x"34FE", x"F198",
        x"D5D4", x"F5BE", x"4A40", x"A58F", x"AC60", x"C092", x"5CFE", x"7FB6",
        x"4D28", x"3A3F", x"8984", x"E01F", x"85E0", x"B269", x"2E13", x"9B65",
        x"A51D", x"64F7", x"0C4D", x"BE18", x"8D31", x"5107", x"789F", x"91CD",
        x"977B", x"21B5", x"2881", x"8FD1", x"19EC", x"A3B1", x"1019", x"AED0"
    );
    -- set ROM: 2 words per (set, slot): recipe, colours (see tables.py)
    constant C_SROM : t_rom16 := (
        x"3C04", x"135F", x"4581", x"65DF", x"4401", x"75AF", x"2003", x"D40F",
        x"0004", x"740F", x"2437", x"661F", x"0000", x"840F", x"E802", x"01DF",
        x"4041", x"55AF", x"6001", x"695F", x"2051", x"799F", x"4041", x"75AF",
        x"6001", x"85AF", x"3D49", x"761F", x"2051", x"85DF", x"1C00", x"85DF",
        x"E9B1", x"FB5F", x"4401", x"EB2F", x"0000", x"DBDF", x"E9B1", x"CB9F",
        x"0405", x"D6EF", x"7C02", x"C7AF", x"E400", x"D40F", x"1C00", x"C40F",
        x"E405", x"BA5F", x"2003", x"AA2F", x"0405", x"9AEF", x"3C04", x"A62F",
        x"2003", x"96EF", x"E400", x"8A9F", x"E802", x"95DF", x"0000", x"869F",
        x"E405", x"680F", x"41AE", x"695F", x"4436", x"01DF", x"E405", x"796F",
        x"2659", x"799F", x"41AE", x"75AF", x"2659", x"85DF", x"E405", x"85AF",
        x"4806", x"49AF", x"3C04", x"59EF", x"4401", x"59EF", x"4806", x"692F",
        x"6001", x"6A2F", x"0405", x"796F", x"0004", x"795F", x"0000", x"899F",
        x"4401", x"2B2F", x"E9B1", x"382F", x"6001", x"1AAF", x"4436", x"4B6F",
        x"0405", x"E8EF", x"4401", x"2A2F", x"7C02", x"C8EF", x"2003", x"371F",
        x"E400", x"740F", x"0000", x"740F", x"2051", x"740F", x"E802", x"01DF",
        x"1C00", x"740F", x"E400", x"840F", x"7C02", x"740F", x"0000", x"01DF",
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
    signal s_tex, s_adapt, s_white, s_ground, s_both, s_paper, s_inv, s_light, s_dense : std_logic := '0';
    signal c_il    : std_logic := '0';
    signal c_lz    : unsigned(1 downto 0) := "00";     -- K6 zone
    signal c_lout, c_sk_on, c_ct_on : std_logic := '0'; -- outline / sketch / contour layer on
    signal c_hrot  : unsigned(3 downto 0) := "0000";   -- hue turn
    signal c_set   : unsigned(2 downto 0) := "000";    -- texture set
    signal c_relief : unsigned(1 downto 0) := "00";
    signal c_drift : unsigned(3 downto 0) := "0000";
    signal c_col   : unsigned(1 downto 0) := "00";     -- 0 source, 1 tint, 2 palette, 3 patchwork
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
    signal r33_g1, r33_g2, r33_shape : unsigned(2 downto 0) := (others => '0');
    signal r33_ex, r33_ey : unsigned(3 downto 0) := (others => '1');
    signal r33_yneg, r33_z1, r33_z2 : std_logic := '0';
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
    -- texture fields (F domain); index 0 = octave 1 (large), 1 = octave 2
    ------------------------------------------------------------------------
    type t_nram is array (0 to 255) of std_logic_vector(31 downto 0);
    signal nram : t_nram := (others => x"80008000");             -- node states {s1, s2} (8.8)
    attribute no_rw_check of nram : signal is true;
    signal nq, rd, wd : std_logic_vector(31 downto 0) := (others => '0');
    type t_o12 is array (0 to 1) of unsigned(11 downto 0);
    type t_s20 is array (0 to 1) of signed(19 downto 0);
    type t_s16 is array (0 to 1) of signed(15 downto 0);
    type t_s14 is array (0 to 1) of signed(13 downto 0);
    type t_s10 is array (0 to 1) of signed(9 downto 0);
    type t_o8  is array (0 to 1) of unsigned(7 downto 0);
    type t_o6  is array (0 to 1) of unsigned(5 downto 0);
    type t_o2  is array (0 to 1) of unsigned(1 downto 0);
    signal nx : t_o12 := (others => (others => '0'));            -- x offset of the next line (px)
    signal ndr : unsigned(11 downto 0) := (others => '0');       -- octave 2 drift
    signal tly : unsigned(10 downto 0) := (others => '0');       -- line being displayed
    signal tv : t_s20 := (others => (others => '0'));            -- DDA value 11.9 (one pixel ahead)
    signal tstp : t_s16 := (others => (others => '0'));          -- DDA step
    signal tk, pr : t_o6 := (others => (others => '0'));         -- pixel in cell; roll-forward left
    signal nn0, nn1 : t_s10 := (others => (others => '0'));      -- node ahead, node after it
    signal ncc : t_o8 := (others => (others => '0'));            -- next node to fetch
    signal pend, nrdy : t_o2 := (others => "00");                -- nodes wanted / delivered this line
    signal sacc : t_s14 := (others => (others => '0'));          -- smoothed value (x8 / x4)
    signal ini1, ini2, pre : std_logic_vector(1 downto 0) := "00";
    signal px : unsigned(10 downto 0) := (others => '0');
    signal run, eo, nwe, st_r, rc_r, ls_r : std_logic := '0';
    signal est : integer range 0 to 7 := 0;
    signal ek : unsigned(1 downto 0) := "01";                    -- pole shift - 3
    signal hv, hay : unsigned(15 downto 0) := (others => '0');   -- random word of the node; this line's offsets
    signal hq : std_logic_vector(15 downto 0) := (others => '0');
    signal hra, nra : unsigned(7 downto 0) := (others => '0');
    signal lsd : std_logic_vector(2 downto 0) := "000";
    signal d1, d2 : signed(16 downto 0) := (others => '0');
    signal gq : signed(9 downto 0) := (others => '0');
    -- shaping / fill colour pipeline T1..T8
    signal f1_a, f1_b : signed(13 downto 0) := (others => '0');
    signal f1_c, f1_d, f2_cd, f3_p : unsigned(9 downto 0) := (others => '0');
    signal f1_yneg, f1_adj, f5_px0 : std_logic := '0';
    signal f1_shape, f2_shape, f3_shape, f6_m : unsigned(2 downto 0) := (others => '0');
    signal f2_ab : signed(14 downto 0) := (others => '0');
    signal f3_c : signed(9 downto 0) := (others => '0');
    signal f4_t, f5_d1, f5_d2, f5_d3, f7_ua, f7_va, f8_u, f8_v : unsigned(7 downto 0) := x"80";
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
    begin
        if rising_edge(clk) then
            k1 <= unsigned(registers_in(0)); k2 <= unsigned(registers_in(1));
            k3 <= unsigned(registers_in(2)); k4 <= unsigned(registers_in(3));
            k5 <= unsigned(registers_in(4)); k6 <= unsigned(registers_in(5));
            p12 <= unsigned(registers_in(7));
            -- switches
            s_adapt <= registers_in(6)(1);
            if C_HAS_TEX then
                -- VENEER: S7 Fill, S9 Invert, S10 Light, S11 Detail
                s_tex <= registers_in(6)(0); s_inv <= registers_in(6)(2);
                s_light <= registers_in(6)(3); s_dense <= registers_in(6)(4);
                s_white <= '0'; s_ground <= '0'; s_both <= '0'; s_paper <= '0';
            else
                -- VENEER INK: S7 Ground, S9 Ink, S10 Sketch sides, S11 Paper
                s_tex <= '0'; s_inv <= '0'; s_light <= '0'; s_dense <= '0';
                s_ground <= registers_in(6)(0); s_white <= registers_in(6)(2);
                s_both <= registers_in(6)(3); s_paper <= registers_in(6)(4);
            end if;

            -- interlaced timings (field lines)
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8 | 1 | 2 | 10 => c_il <= '1';
                when others             => c_il <= '0';
            end case;

            -- K1 Texture set / palette, 8 zones
            if k1 < 8 then c_set <= k1(2 downto 0);
            elsif k1(6 downto 3) /= "0000" and k1(6 downto 3) /= "1111" then c_set <= k1(9 downto 7); end if;
            -- K2 Hue: turns every colour by 16 steps of 22.5 degrees
            if k2(5 downto 2) /= "0000" and k2(5 downto 2) /= "1111" then c_hrot <= k2(9 downto 6); end if;
            -- K3: Depth (VENEER) / Sketch amount (VENEER INK): off in the bottom 3%,
            -- then the threshold falls 6.0 -> 1.6 codes (in 1/16 code: 96 - 2.25 e)
            c_depth <= k3(9 downto 6);
            if C_HAS_SKETCH and k3(9 downto 5) /= "00000" then c_sk_on <= '1'; else c_sk_on <= '0'; end if;
            c_xthr <= to_unsigned(96, 7) - resize(shift_left(resize(k3(9 downto 5), 7), 1), 7) - resize(shift_right(k3(9 downto 5), 2), 7);
            -- K4 Colour: Source / Tint / Palette / Patchwork
            if k4 < 4 then c_col <= k4(1 downto 0);
            elsif k4(7 downto 3) /= "00000" and k4(7 downto 3) /= "11111" then c_col <= k4(9 downto 8); end if;
            -- K5: Drift, 0..15 px per frame (VENEER) / Contour amount (VENEER INK): off
            -- in the bottom 3%, then the Canny pre-gain 2..17
            c_drift <= k5(9 downto 6);
            if C_HAS_CANNY and k5(9 downto 5) /= "00000" then c_ct_on <= '1'; else c_ct_on <= '0'; end if;
            c_cgain <= resize(k5(9 downto 6), 5) + 2;
            -- K6: Relief, Flat / Soft / Embossed / Deep (VENEER) / Outline, Off / Thin /
            -- Medium / Bold (VENEER INK)
            if k6 < 4 then c_lz <= k6(1 downto 0);
            elsif k6(7 downto 3) /= "00000" and k6(7 downto 3) /= "11111" then c_lz <= k6(9 downto 8); end if;
            c_relief <= c_lz;
            if C_HAS_OUTLINE and c_lz /= "00" then c_lout <= '1'; else c_lout <= '0'; end if;
            c_width <= c_lz - 1;

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
                -- noise gains: the recipe's shift (0..3, 7 = unused), one more when Dense
                if srom_q(15 downto 13) = "111" then r33_g1 <= "111";
                elsif s_dense = '1' then r33_g1 <= unsigned(srom_q(15 downto 13)) + 1;
                else r33_g1 <= unsigned(srom_q(15 downto 13)); end if;
                if srom_q(12 downto 10) = "111" then r33_g2 <= "111";
                elsif s_dense = '1' then r33_g2 <= unsigned(srom_q(12 downto 10)) + 1;
                else r33_g2 <= unsigned(srom_q(12 downto 10)); end if;
                r33_yneg <= srom_q(3); r33_shape <= unsigned(srom_q(2 downto 0));
                -- stripe shifts: code c (1..7) is X << (c + 1), 15 = no stripe term
                if srom_q(9 downto 7) = "000" then r33_ex <= "1111"; else r33_ex <= resize(unsigned(srom_q(9 downto 7)), 4) + 1; end if;
                if srom_q(6 downto 4) = "000" then r33_ey <= "1111"; else r33_ey <= resize(unsigned(srom_q(6 downto 4)), 4) + 1; end if;
                -- the noise terms are centred; their offset is 512 << g each, which
                -- matters only modulo the 1024 phase: 512 when exactly one final g is 0
                r33_z1 <= f_b2sl(srom_q(15 downto 13) = "000" and s_dense = '0');
                r33_z2 <= f_b2sl(srom_q(12 downto 10) = "000" and s_dense = '0');
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
                if cl29 /= ab29 then da30 <= '1'; else da30 <= '0'; end if;
                da31 <= da30;
                -- (cw3 at step v+31 holds (v+1, v, v-1); da31 = differs from above (v))
                if cw3(1) /= cw3(2) or da31 = '1' then t32 <= '1'; else t32 <= '0'; end if;
                if cw3(1) /= cw3(0) then dr32 <= '1'; else dr32 <= '0'; end if;
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
    -- TEXTURE FIELDS (F domain, label independent).  Two octaves of smooth
    -- noise, n1 (nodes 64 px apart) and n2 (16 px):
    --   every node is its own vertical random process: a two-pole filter of
    --     random bytes R[(node + offset(line)) mod 256] (R = a 256 x 16 random
    --     ROM), state {s1, s2} per node in a 256 x 32 RAM, advanced once per
    --     line just ahead of the beam (so there is no row lattice at all, and
    --     the filters simply run on from frame to frame)
    --   the node grid is sheared one pixel every other line (no vertical
    --     column structure) and octave 2 drifts K5 pixels per frame
    --   between nodes: a linear DDA, then one pixel-rate pole rounds the kinks
    -- At a line start three nodes per octave are fetched; after the second the
    -- DDA is rolled forward to its phase inside the first cell (blanking); from
    -- then on every cell boundary asks for one more node.
    -- Outputs sacc(o) >> a, centred, +-511.
    ------------------------------------------------------------------------
    g_tex : if C_HAS_TEX generate
    begin
    p_tex : process(clk)
        variable v_st : std_logic;
        variable v_inc, v_dec, v_ld : std_logic_vector(1 downto 0);
        variable v_c : unsigned(7 downto 0);
        variable v_mk, v_u16, v_s1, v_s2, v_s1d : unsigned(15 downto 0);
        variable v_n : signed(16 downto 0);
        variable v_g : signed(15 downto 0);
        variable v_s1n, v_s2n : unsigned(15 downto 0);
        variable v_o1 : std_logic;
        variable v_x1 : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            nwe <= '0';
            v_inc := "00"; v_dec := "00"; v_ld := "00";

            -- per pixel.  First pixel: pair 0 enters H stage 33 at this edge; the
            -- DDA value runs one pixel ahead of the smoothed value, so it takes its
            -- first step here and the pole starts a clock later.
            -- (the strobes below are registered a clock ahead: `ce and hf_c(k)` taken
            -- combinationally was a 13 ns routing path into every enable here)
            v_st := st_r;
            st_r <= (not ce) and hf_c(32);           -- '1' in the cycle that ends at that edge
            rc_r <= (not ce) and (not hv_c(32));
            ls_r <= (not ce) and hf_c(1);
            -- octave 1 (64 px cells): the roll-forward takes 8-pixel strides first
            if pre(0) = '1' and pr(0)(5 downto 3) /= "000" then v_x1 := shift_left(resize(tstp(0), 20), 3);
            else v_x1 := resize(tstp(0), 20); end if;
            if run = '1' or v_st = '1' or (pre(0) = '1' and pr(0) /= 0) then tv(0) <= tv(0) + v_x1; end if;
            if run = '1' or v_st = '1' or (pre(1) = '1' and pr(1) /= 0) then tv(1) <= tv(1) + resize(tstp(1), 20); end if;
            if run = '1' or v_st = '1' then
                if tk(0) = 63 then tk(0) <= (others => '0'); v_ld(0) := '1'; v_inc(0) := '1'; else tk(0) <= tk(0) + 1; end if;
                if tk(1)(3 downto 0) = 15 then tk(1) <= (others => '0'); v_ld(1) := '1'; v_inc(1) := '1'; else tk(1) <= tk(1) + 1; end if;
            end if;
            if run = '1' then
                sacc(0) <= sacc(0) + resize(tv(0)(19 downto 9), 14) - resize(shift_right(sacc(0), 3), 14);
                sacc(1) <= sacc(1) + resize(tv(1)(19 downto 9), 14) - resize(shift_right(sacc(1), 2), 14);
                px <= px + 1;
            end if;
            if rc_r = '1' then run <= '0'; end if;
            if v_st = '1' then run <= '1'; end if;

            -- line start roll-forward (blanking): k0 pixels from the left node
            if pre(0) = '1' then
                if pr(0) = 0 then pre(0) <= '0'; sacc(0) <= shift_left(resize(tv(0)(19 downto 9), 14), 3);
                elsif pr(0)(5 downto 3) /= "000" then pr(0) <= pr(0) - 8;
                else pr(0) <= pr(0) - 1; end if;
            end if;
            if pre(1) = '1' then
                if pr(1) = 0 then pre(1) <= '0'; sacc(1) <= shift_left(resize(tv(1)(19 downto 9), 14), 2);
                else pr(1) <= pr(1) - 1; end if;
            end if;

            -- step of the cell being entered = (next node - this node) / D, in 11.9
            -- (cells 64 / 16 px: << 3 / << 5); at the line start the DDA is set to
            -- the left node at the same moment and the roll-forward begins
            -- (resize BEFORE the subtraction: two 10-bit nodes can differ by more than 511)
            if v_ld(0) = '1' or ini1(0) = '1' then tstp(0) <= shift_left(resize(nn1(0), 16) - resize(nn0(0), 16), 3); end if;
            if v_ld(1) = '1' or ini1(1) = '1' then tstp(1) <= shift_left(resize(nn1(1), 16) - resize(nn0(1), 16), 5); end if;
            for o in 0 to 1 loop
                if ini1(o) = '1' then tv(o) <= shift_left(resize(nn0(o), 20), 9); end if;
            end loop;
            pre <= pre or ini2;
            ini2 <= ini1; ini1 <= "00";

            ----------------------------------------------------------
            -- node engine: one node at a time, 7 clocks:
            --   0 addresses (state RAM, noise ROM)   1 -   2 RAM / ROM data
            --   3 differences   4 new state (written next clock)   5 gain   6 deliver
            -- Octave 2 goes first, except at a line start where octave 1's first two
            -- nodes follow octave 2's first two (both roll-forwards start early).
            ----------------------------------------------------------
            case est is
                when 0 =>
                    if pend(0) /= "00" and (pend(1) = "00" or (nrdy(1)(1) = '1' and nrdy(0)(1) = '0')) then v_o1 := '1'; else v_o1 := '0'; end if;
                    if v_o1 = '1' then
                        eo <= '0'; v_c := ncc(0); nra <= "00" & v_c(5 downto 0);
                        hra <= v_c + hay(7 downto 0);
                        ncc(0) <= v_c + 1; v_dec(0) := '1';
                        est <= 1;
                    elsif pend(1) /= "00" then
                        eo <= '1'; v_c := ncc(1); nra <= '1' & v_c(6 downto 0);
                        hra <= v_c + hay(15 downto 8);
                        ncc(1) <= v_c + 1; v_dec(1) := '1';
                        est <= 1;
                    end if;
                when 1 =>
                    est <= 2;
                when 2 =>
                    hv <= unsigned(hq); rd <= nq;
                    est <= 3;
                when 3 =>
                    -- s1 += (u - s1) >> k ; s2 += (s1 - s2) >> k  (s2 sees last line's
                    -- s1).  Stochastic rounding: random bits fill the bits the shift
                    -- drops, so the poles have no dead zone.  k = 5 (octave 1) / 4
                    -- (octave 2), one less on interlaced fields; ek = k - 3.
                    if eo = '0' then ek <= "10" - ("0" & c_il); else ek <= "01" - ("0" & c_il); end if;
                    if eo = '0' and c_il = '0' then v_mk := x"001F";
                    elsif eo = '1' and c_il = '1' then v_mk := x"0007";
                    else v_mk := x"000F"; end if;
                    v_u16 := hv(15 downto 8) & (hv(7 downto 0) and v_mk(7 downto 0));
                    v_s1 := unsigned(rd(31 downto 16)); v_s2 := unsigned(rd(15 downto 0));
                    v_s1d := (v_s1 and not v_mk) or (resize(hv(10 downto 2), 16) and v_mk);
                    d1 <= signed('0' & v_u16) - signed('0' & v_s1);
                    d2 <= signed('0' & v_s1d) - signed('0' & v_s2);
                    est <= 4;
                when 4 =>
                    case ek is
                        when "00"   => v_n := shift_right(d1, 3);
                        when "01"   => v_n := shift_right(d1, 4);
                        when others => v_n := shift_right(d1, 5);
                    end case;
                    v_n := signed('0' & unsigned(rd(31 downto 16))) + v_n;
                    v_s1n := unsigned(v_n(15 downto 0));
                    case ek is
                        when "00"   => v_n := shift_right(d2, 3);
                        when "01"   => v_n := shift_right(d2, 4);
                        when others => v_n := shift_right(d2, 5);
                    end case;
                    v_n := signed('0' & unsigned(rd(15 downto 0))) + v_n;
                    v_s2n := unsigned(v_n(15 downto 0));
                    wd <= std_logic_vector(v_s1n) & std_logic_vector(v_s2n);
                    nwe <= '1';
                    est <= 5;
                when 5 =>
                    -- node value: the centred state x gain (the gain follows k so the
                    -- range is filled either way: k 3: /16, k 4: x1.5/16, k 5: /8),
                    -- clamped to +-511
                    v_g := signed((not wd(15)) & wd(14 downto 0));
                    if ek = "10" then v_g := shift_right(v_g, 3); else v_g := shift_right(v_g, 4); end if;
                    if ek = "01" then v_g := v_g + shift_right(v_g, 1); end if;
                    if v_g > 511 then gq <= to_signed(511, 10);
                    elsif v_g < -511 then gq <= to_signed(-511, 10);
                    else gq <= v_g(9 downto 0); end if;
                    est <= 6;
                when 6 =>
                    if eo = '1' then
                        nn0(1) <= nn1(1); nn1(1) <= gq;
                        if nrdy(1) /= "11" then nrdy(1) <= nrdy(1) + 1; end if;
                        if nrdy(1) = "01" then ini1(1) <= '1'; end if;
                    else
                        nn0(0) <= nn1(0); nn1(0) <= gq;
                        if nrdy(0) /= "11" then nrdy(0) <= nrdy(0) + 1; end if;
                        if nrdy(0) = "01" then ini1(0) <= '1'; end if;
                    end if;
                    est <= 0;
                when others =>
                    -- line start hold: wait for this line's table offsets
                    if lsd = "000" then est <= 0; end if;
            end case;
            for o in 0 to 1 loop
                if v_inc(o) = '1' and v_dec(o) = '0' then pend(o) <= pend(o) + 1;
                elsif v_inc(o) = '0' and v_dec(o) = '1' then pend(o) <= pend(o) - 1; end if;
            end loop;

            -- after the line start strobe the noise ROM is read at [line mod 256] and
            -- [line / 8]; the two words combine into this line's two table offsets
            -- (one per octave), so no two lines read the table the same way
            lsd <= lsd(1 downto 0) & '0';
            if lsd(0) = '1' then hra <= tly(10 downto 3); end if;
            if lsd(1) = '1' then hay <= unsigned(hq); end if;
            if lsd(2) = '1' then
                hay(15 downto 8) <= hay(15 downto 8) + unsigned(hq(7 downto 0));
                hay(7 downto 0) <= hay(7 downto 0) + unsigned(hq(15 downto 8));
            end if;

            -- line start: pair 0 enters H stage 2, 62 clocks before its first pixel.
            -- Latch this line's first node and phase per octave, ask for three
            -- nodes each, then shear the grids for the next line.
            if ls_r = '1' then
                tly <= tly + 1; run <= '0'; px <= (others => '0');
                hra <= tly(7 downto 0) + 1; lsd <= "001";
                est <= 7;
                ncc(0) <= resize(nx(0)(11 downto 6), 8); tk(0) <= nx(0)(5 downto 0); pr(0) <= nx(0)(5 downto 0);
                ncc(1) <= nx(1)(11 downto 4);            tk(1) <= "00" & nx(1)(3 downto 0); pr(1) <= "00" & nx(1)(3 downto 0);
                pend <= (others => "11"); nrdy <= (others => "00"); pre <= "00";
                if c_il = '1' or tly(0) = '1' then nx(0) <= nx(0) + 1; end if;
                if c_il = '1' or tly(0) = '0' then nx(1) <= nx(1) - 1; end if;
            end if;

            -- frame start (vblank): drift (K5 pixels per frame), grids back to line 0
            if vs_edge = '1' then
                tly <= (others => '1'); run <= '0';
                ndr <= ndr - resize(c_drift, 12);
                nx(0) <= (others => '0'); nx(1) <= ndr;
                pend <= (others => "00"); pre <= "00"; est <= 0;
            end if;
        end if;
    end process p_tex;

    -- node state RAM (one read, one write statement; the write follows the read
    -- of the same node by four clocks) and the noise ROM
    p_texmem : process(clk)
    begin
        if rising_edge(clk) then
            nq <= nram(to_integer(nra));
            if nwe = '1' then nram(to_integer(nra)) <= wd; end if;
            hq <= C_NROM(to_integer(hra));
        end if;
    end process p_texmem;

    ------------------------------------------------------------------------
    -- TEXTURE SHAPING + FILL COLOUR (F domain, per pixel; the recipe and the
    -- colours of the pixel's region are read from the H stages named below.
    -- An F stage 2k clocks before O1 reads H stage 37 - k for both pixels of a
    -- pair, so H values are only read at even offsets.)
    --   T1 (-8, H 33) terms: n1 << g1, n2 << g2, X << ex, Y << ey
    --   T2 (-7)       noise sum, stripe sum
    --   T3 (-6)       phase (10 bit, wraps) and the centred noise sum (clamped)
    --   T4 (-5)       shape -> texture value t (0..255)
    --   T5 (-4)       relief (t - t three pixels back), smoothed t for the colour ramp
    --   T6 (-3)       e = (t - 128) +- relief x K6; ramp position m (0..4, dithered)
    --   T7 (-2, H 36) chroma step m/4 x (B - A); luma partial products e x depth
    --   T8 (-1)       chroma = A + step; luma delta
    ------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_n1, v_n2 : signed(9 downto 0);
        variable v_ty : unsigned(11 downto 0);
        variable v_c10 : signed(9 downto 0);
        variable v_v8, v_tr8, v_a9 : unsigned(8 downto 0);
        variable v_t : unsigned(7 downto 0);
        variable v_rel : signed(8 downto 0);
        variable v_g1, v_g2 : signed(8 downto 0);
        variable v_q : signed(12 downto 0);
        variable v_ts : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- T1
            v_n1 := sacc(0)(12 downto 3);
            v_n2 := sacc(1)(11 downto 2);
            if r33_g1 = "111" then f1_a <= (others => '0');
            else f1_a <= shift_left(resize(v_n1, 14), to_integer(r33_g1)); end if;
            if r33_g2 = "111" then f1_b <= (others => '0');
            else f1_b <= shift_left(resize(v_n2, 14), to_integer(r33_g2)); end if;
            if r33_ex = "1111" then f1_c <= (others => '0');
            else f1_c <= resize(shift_left(resize(px, 19), to_integer(r33_ex)), 10); end if;
            if c_il = '1' then v_ty := tly & '0'; else v_ty := '0' & tly; end if;
            if r33_ey = "1111" then f1_d <= (others => '0');
            else f1_d <= resize(shift_left(resize(v_ty, 20), to_integer(r33_ey)), 10); end if;
            f1_yneg <= r33_yneg; f1_adj <= r33_z1 xor r33_z2; f1_shape <= r33_shape;
            -- T2
            f2_ab <= resize(f1_a, 15) + resize(f1_b, 15);
            if f1_yneg = '1' then f2_cd <= f1_c - f1_d + (f1_adj & "000000000");
            else f2_cd <= f1_c + f1_d + (f1_adj & "000000000"); end if;
            f2_shape <= f1_shape;
            -- T3
            f3_p <= unsigned(f2_ab(9 downto 0)) + f2_cd;
            if f2_ab > 511 then f3_c <= to_signed(511, 10);
            elsif f2_ab < -512 then f3_c <= to_signed(-512, 10);
            else f3_c <= f2_ab(9 downto 0); end if;
            f3_shape <= f2_shape;
            -- T4: shape (lookup-style logic only: no carries)
            v_c10 := f3_c;
            v_v8 := '0' & (not v_c10(9)) & unsigned(v_c10(8 downto 2));        -- (c >> 2) + 128
            if f3_p(9) = '0' then v_tr8 := '0' & f3_p(8 downto 1); else v_tr8 := '0' & (not f3_p(8 downto 1)); end if;
            if v_c10(9) = '1' then v_a9 := unsigned(not v_c10(8 downto 0)); else v_a9 := unsigned(v_c10(8 downto 0)); end if;
            case f3_shape is
                when "000" => v_t := v_v8(7 downto 0);                           -- direct
                when "001" => v_t := v_tr8(7 downto 0);                          -- bands (triangle)
                when "010" =>                                                    -- thin dark line
                    if v_tr8(7 downto 6) = "00" then v_t := v_tr8(5 downto 0) & "00"; else v_t := (others => '1'); end if;
                when "011" =>                                                    -- camouflage: 3 flat levels
                    if v_v8(7 downto 5) <= "010" then v_t := x"00"; elsif v_v8(7 downto 5) >= "101" then v_t := x"FF"; else v_t := x"80"; end if;
                when "100" =>                                                    -- veins: dark where the field crosses its mean
                    if v_a9(8 downto 7) = "00" then v_t := v_a9(6 downto 0) & '0'; else v_t := (others => '1'); end if;
                when "101" =>                                                    -- blobs: soft threshold
                    if v_c10(9 downto 6) = "0000" or v_c10(9 downto 6) = "1111" then v_t := (not v_c10(6)) & unsigned(v_c10(5 downto 0)) & '0';
                    elsif v_c10(9) = '1' then v_t := x"00"; else v_t := x"FF"; end if;
                when "110" =>                                                    -- zebra: soft-thresholded bands
                    if v_tr8(7 downto 5) <= "010" then v_t := x"00";
                    elsif v_tr8(7 downto 5) >= "101" then v_t := x"FF";
                    else v_t := '0' & v_tr8(7) & v_tr8(4 downto 0) & '0'; end if;
                when others =>                                                   -- terraces: 3-level bands
                    if v_tr8(7 downto 5) <= "010" then v_t := x"00"; elsif v_tr8(7 downto 5) >= "101" then v_t := x"FF"; else v_t := x"80"; end if;
            end case;
            if s_inv = '1' then f4_t <= not v_t; else f4_t <= v_t; end if;     -- S9 Invert
            -- T5
            f5_d1 <= f4_t; f5_d2 <= f5_d1; f5_d3 <= f5_d2;
            v_rel := signed(resize(f4_t, 9)) - signed(resize(f5_d3, 9));
            if v_rel > 63 then f5_rel <= to_signed(63, 7); elsif v_rel < -64 then f5_rel <= to_signed(-64, 7); else f5_rel <= v_rel(6 downto 0); end if;
            f5_tc <= signed(f4_t xor x"80");                                    -- t - 128
            f5_ts <= f5_ts + resize(f4_t, 10) - resize(shift_right(f5_ts, 2), 10);
            f5_px0 <= px(0);
            -- T6
            -- relief: K6 amount (x0, x1/2, x1, x2), S10 light from the left / right
            case c_relief is
                when "00"   => v_rel := (others => '0');
                when "01"   => v_rel := resize(shift_right(f5_rel, 1), 9);
                when "10"   => v_rel := resize(f5_rel, 9);
                when others => v_rel := shift_left(resize(f5_rel, 9), 1);
            end case;
            if s_tex = '0' then f6_e <= resize(f5_tc, 9);
            elsif s_light = '1' then f6_e <= resize(f5_tc, 9) - v_rel;
            else f6_e <= resize(f5_tc, 9) + v_rel; end if;
            if f5_px0 = '1' then v_ts := resize(f5_ts(9 downto 2), 9) + 48; else v_ts := resize(f5_ts(9 downto 2), 9) + 16; end if;
            if s_tex = '1' then f6_m <= v_ts(8 downto 6); else f6_m <= "000"; end if;
            -- T7 (H stage 36): one add of gated operands each
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
    end generate;
    -- ink build: no texture; the fill is the region colour (one more register so
    -- it is read at H stage 37 like the luma)
    g_notex : if not C_HAS_TEX generate
        f8_u <= ua37; f8_v <= va37; f8_ey <= (others => '0');
    end generate;

    ------------------------------------------------------------------------
    -- F composite.  O1 holds pixel x during cycle t_in + x + 79.
    --   O1 fill colour (H stage 37 + the T8 texture values), ink alpha
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
            -- O1: matte luma (bin centroid) + texture / wash, chroma from T8
            ----------------------------------------------------------
            if s_tex = '1' then v_e := resize(shift_right(f8_ey, 4), 12);      -- texture: e x depth / 16
            else v_e := resize(shift_right(f8_ey, 6), 12); end if;             -- ink wash: a quarter of that
            v_sum := signed(resize(ho_c_y & "00", 12)) + v_e;
            if v_sum < 64 then o1_y <= to_unsigned(64, 10);
            elsif v_sum > 940 then o1_y <= to_unsigned(940, 10);
            else o1_y <= unsigned(v_sum(9 downto 0)); end if;
            o1_u <= f8_u & "00"; o1_v <= f8_v & "00";
            -- ink fill: the darkest bin is the black ground (ink build S7: keep its colour)
            if s_tex = '0' and s_ground = '0' and ho_c_lum = "000" then o1_bg <= '1'; else o1_bg <= '0'; end if;
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

end architecture veneer;
