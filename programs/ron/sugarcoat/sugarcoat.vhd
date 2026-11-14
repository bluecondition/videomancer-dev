-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- SUGARCOAT v2 -- video re-rendered as candy.  The picture is flattened into
-- clean matte regions (luma levels x chroma classes, cleaned by mode filters:
-- the VENEER / APPLIQUE matte pipeline, taken over unchanged) and every
-- region is filled from a bank of ~100 generated confections: chocolate bars,
-- cookies, piped frosting, hard candy, jellies, liquorice.  The candies are
-- drawn, not painted flat: scattered pieces with random colour, size and
-- turn, dome and tube shading with a specular highlight, an embossed edge
-- with a drop shadow, crumb / sugar grain and a sprinkle overlay.
--
-- This file is a fork of programs/ron/applique/applique.vhd (2026-11-14,
-- .sugarcoat_work/fork.py).  The matte code is kept line for line so that
-- fixes VENEER / APPLIQUE get on hardware can be ported; the cloth pattern
-- generator is replaced by the candy generator.  Read DESIGN.md first.
--
-- Controls
--   K1   Theme (8): Candy Shop, Bakery, Chocolate, Frosting, Liquorice, Pastel, Gummy, Peppermint
--   K2   Sprinkles (density of the sprinkle overlay; 0 = none)
--   K3   Scale (continuous, x1 .. x16)
--   K4   Shade (how much of the picture's light and dark the candy keeps)
--   K5   Level (bipolar: more dark candy / more light candy; centre = unchanged)
--   K6   Gloss (shading, highlights and emboss; 0 = flat cut paper)
--   S7   Lines: Off / On (thin black line where the candy changes)
--   S8   Split: Fixed / Adaptive thresholds
--   S9   Bank: A / B (a second helping of each theme)
--   S10  Angle: Straight / Bias (every candy turned 45 degrees)
--   S11  Map: Chroma (coloured regions get a hue-matched candy) / Luma
--   P12  Variety: 2..8 luma levels (and 2 / 4 / 8 hue groups)
--
-- Architecture: an F domain (every clock) and an H domain (one step per pixel
-- pair, clock enable `ce`).  The matte analysis runs at half horizontal
-- resolution so every line buffer is 1024 deep (chroma 512).  Stage offsets in
-- comments are H steps after the capture of column m ("stage k holds column m
-- during step m+k").  The candy generator runs at the full pixel rate beside
-- the late H stages.  Latency 84 clocks, all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture sugarcoat of program_top is

    ------------------------------------------------------------------------
    -- BUILD CONFIGURATION (the VENEER build switches, fixed for SUGARCOAT: kept
    -- so the shared matte code stays line-for-line VENEER's)
    ------------------------------------------------------------------------
    constant C_HAS_OUTLINE  : boolean := true;     -- region outline layer (the lines)
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

    ------------------------------------------------------------------------
    -- candy generator helpers
    ------------------------------------------------------------------------
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
    -- 5-bit phase -> 4-bit triangle
    function f_tri4(x : unsigned(4 downto 0)) return unsigned is
    begin
        if x(4) = '1' then return not x(3 downto 0); else return x(3 downto 0); end if;
    end function;
    -- 16-bit hash: 4 rounds of (xor constant, 4-bit S-boxes, bit permutation).
    -- No carry chains: four LUT levels.  Checked numerically (model.py /
    -- .sugarcoat_work/spntest.py): every output bit balanced, neighbour
    -- correlations at the level of a random table.
    function f_sbox(x : std_logic_vector(3 downto 0)) return std_logic_vector is
    begin
        case x is
            when x"0" => return x"C"; when x"1" => return x"5"; when x"2" => return x"6"; when x"3" => return x"B";
            when x"4" => return x"9"; when x"5" => return x"0"; when x"6" => return x"A"; when x"7" => return x"D";
            when x"8" => return x"3"; when x"9" => return x"E"; when x"A" => return x"F"; when x"B" => return x"8";
            when x"C" => return x"4"; when x"D" => return x"7"; when x"E" => return x"1"; when others => return x"2";
        end case;
    end function;
    function f_spn(k : std_logic_vector(15 downto 0); c0, c1, c2, c3 : std_logic_vector(15 downto 0)) return std_logic_vector is
        variable x, y, z : std_logic_vector(15 downto 0);
    begin
        x := k;
        for r in 0 to 3 loop
            case r is
                when 0 => x := x xor c0;
                when 1 => x := x xor c1;
                when 2 => x := x xor c2;
                when others => x := x xor c3;
            end case;
            for i in 0 to 3 loop
                y(4 * i + 3 downto 4 * i) := f_sbox(x(4 * i + 3 downto 4 * i));
            end loop;
            if r = 3 then
                x := y;
            else
                for i in 0 to 14 loop z((i * 4) mod 15) := y(i); end loop;
                z(15) := y(15);
                x := z;
            end if;
        end loop;
        return x;
    end function;
    -- class of a region: label {lum, hue, sat} -> 0..7 luma slot (the levels in
    -- force spread over the 8 slots, dark -> light) or, for a coloured region
    -- in the Chroma map, 8 + hue sector (2 / 4 / 8 hue groups by the levels)
    type t_spread is array (0 to 63) of integer range 0 to 7;
    constant C_SPREAD : t_spread := (0, 7, 7, 7, 7, 7, 7, 7,   0, 4, 7, 7, 7, 7, 7, 7,   0, 2, 5, 7, 7, 7, 7, 7,
                                     0, 2, 4, 5, 7, 7, 7, 7,   0, 1, 3, 4, 6, 7, 7, 7,   0, 1, 2, 4, 5, 6, 7, 7,
                                     0, 1, 2, 3, 4, 5, 6, 7,   0, 1, 2, 3, 4, 5, 6, 7);
    function f_cls(lab : unsigned(7 downto 0); lumamap : std_logic; nl : unsigned(2 downto 0)) return unsigned is
        variable hm : unsigned(2 downto 0);
    begin
        -- nl = levels - 2
        if nl(2 downto 1) = "00" then hm := "100"; elsif nl(2 downto 1) = "01" then hm := "110"; else hm := "111"; end if;
        if lumamap = '0' and lab(1 downto 0) /= "00" then
            return '1' & (lab(4 downto 2) and hm);
        else
            return '0' & to_unsigned(C_SPREAD(to_integer(nl & lab(7 downto 5))), 3);
        end if;
    end function;
    -- compare against a constant as plain gates (a `<` operator would be built
    -- as a carry chain: one logic cell per bit)
    function f_lt(x : unsigned; c : natural) return boolean is
        constant n : natural := x'length;
        variable xv : unsigned(n - 1 downto 0);
        variable cv : unsigned(n - 1 downto 0);
        variable lt : std_logic := '0';
        variable eq : std_logic := '1';
    begin
        if c >= 2 ** n then return true; end if;
        xv := x; cv := to_unsigned(c, n);
        for i in n - 1 downto 0 loop
            if cv(i) = '1' then lt := lt or (eq and not xv(i)); eq := eq and xv(i);
            else eq := eq and not xv(i); end if;
        end loop;
        return lt = '1';
    end function;
    function f_slt(x : signed; c : integer) return boolean is
        constant n : natural := x'length;
        variable xv : unsigned(n - 1 downto 0);
    begin
        xv := unsigned(x); xv(n - 1) := not xv(n - 1);
        return f_lt(xv, c + 2 ** (n - 1));
    end function;
    -- gem facets by (not na, not nb, not hg)
    type t_facet is array (0 to 7) of integer range 0 to 15;
    constant C_FACET : t_facet := (13, 11, 10, 8, 7, 5, 4, 2);

    ------------------------------------------------------------------------
    -- ROM contents (generated: programs/ron/sugarcoat/tables.py rewrites this block)
    ------------------------------------------------------------------------
    -- TABLES BEGIN
    type t_rom16 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_rom8  is array (0 to 255) of std_logic_vector(7 downto 0);
    -- palette: Y / 16, U / 32, V / 32
    constant C_PAL : t_rom16 := (
        x"61BA", x"9517", x"C0D4", x"898A", x"7EEA", x"72B2", x"8E38", x"EA10",
        x"C214", x"BD94", x"D572", x"EA10", x"C5ED", x"BE6E", x"BA71", x"BE2C",
        x"8D17", x"B4D3", x"952D", x"91D7", x"51D9", x"5692", x"66EB", x"816D",
        x"35F2", x"55D3", x"8D54", x"A993", x"6994", x"B593", x"D5D1", x"49D2",
        x"2A10", x"55BA", x"81EA", x"AE15", x"E610", x"DDF1", x"D172", x"49F8",
        x"29F1", x"BA30", x"A216", x"8249", x"4EAE", x"A154", x"A8F4", x"4E10",
        x"A98F", x"85D8", x"A272", x"AA8C", x"4633", x"6996", x"7590", x"7E10",
        x"7E10", x"7E10", x"7E10", x"7E10", x"7E10", x"7E10", x"7E10", x"EA10",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210",
        x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210", x"2210"
    );
    -- candy geometry, 2 words per candy
    constant C_GROM : t_rom16 := (
        x"008D", x"0001", x"28CD", x"0001", x"008D", x"0001", x"00CD", x"0015",
        x"4881", x"0015", x"4881", x"001D", x"A855", x"0001", x"040B", x"0001",
        x"48C7", x"0001", x"00B0", x"0001", x"0885", x"0018", x"8064", x"0021",
        x"8064", x"0025", x"C024", x"0025", x"48E7", x"0001", x"4882", x"0081",
        x"88C2", x"0002", x"1091", x"0001", x"020E", x"0001", x"04CA", x"0000",
        x"00AD", x"0001", x"008C", x"0000", x"02F2", x"0001", x"02EE", x"0001",
        x"040F", x"0001", x"0894", x"0001", x"10B1", x"0001", x"0092", x"0001",
        x"00B3", x"0002", x"00A0", x"001D", x"00A0", x"0002", x"0160", x"0081",
        x"0874", x"0001", x"0010", x"0001", x"002B", x"0002", x"4886", x"0002",
        x"4886", x"0002", x"0887", x"0002", x"4881", x"0002", x"000B", x"0042",
        x"4821", x"0003", x"4808", x"0002", x"48A8", x"0001", x"0041", x"0001",
        x"0809", x"0001", x"0073", x"0002", x"4848", x"0001", x"8004", x"0021",
        x"4883", x"0022", x"48A2", x"0003", x"4821", x"0019", x"48A6", x"0001",
        x"02EA", x"0018", x"4065", x"0021", x"004B", x"0002", x"044B", x"0002",
        x"10F1", x"0001", x"8025", x"0001", x"8025", x"0002", x"C045", x"0039",
        x"4823", x"0035", x"4821", x"0002", x"C043", x"0022", x"10D1", x"0001",
        x"02CA", x"0000", x"C024", x"0020", x"4887", x"0002", x"4821", x"0019",
        x"08B4", x"0001", x"4848", x"0002", x"4881", x"0002", x"48C2", x"0001",
        x"4821", x"0003", x"004F", x"0001", x"48A6", x"0001", x"4821", x"0003",
        x"48A6", x"0001", x"48C2", x"0001", x"48A2", x"0003", x"8064", x"0025",
        x"4868", x"0002", x"4883", x"0022", x"00D3", x"0002", x"48A2", x"0003",
        x"4821", x"0019", x"040F", x"0001", x"10D1", x"0018", x"0894", x"0001",
        x"0410", x"0001", x"4863", x"0022", x"48E6", x"0002", x"0829", x"0001",
        x"0869", x"0015", x"006C", x"0000", x"0050", x"0002", x"02AA", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000"
    );
    -- candy colours, 2 words per candy
    constant C_CROM : t_rom16 := (
        x"061E", x"003F", x"065E", x"003F", x"079E", x"003F", x"02DE", x"0028",
        x"0668", x"001F", x"061E", x"003F", x"069E", x"0035", x"0668", x"003F",
        x"066D", x"003F", x"069E", x"0018", x"02E8", x"003F", x"12D8", x"003F",
        x"061B", x"001C", x"079F", x"0028", x"0A24", x"003F", x"0765", x"0023",
        x"06DE", x"0027", x"071E", x"0024", x"075E", x"003F", x"08DD", x"0019",
        x"099E", x"001B", x"08E6", x"000A", x"075E", x"0027", x"0B5E", x"001A",
        x"08DE", x"003F", x"0208", x"003F", x"099E", x"003F", x"095E", x"003F",
        x"02DE", x"003F", x"091E", x"003F", x"065E", x"003F", x"08DE", x"003F",
        x"0925", x"003F", x"031E", x"003F", x"0865", x"0022", x"0848", x"003F",
        x"088C", x"003F", x"300B", x"003F", x"300B", x"003F", x"320B", x"003F",
        x"0B9E", x"003F", x"0848", x"003F", x"3225", x"003F", x"3224", x"003F",
        x"009F", x"0001", x"095E", x"003F", x"320B", x"003F", x"084B", x"003F",
        x"300B", x"003F", x"3425", x"003F", x"340B", x"003F", x"240B", x"003F",
        x"301E", x"003F", x"080B", x"002A", x"0BE0", x"003F", x"0874", x"003F",
        x"020B", x"000C", x"12C8", x"003F", x"069E", x"003F", x"0C4B", x"003F",
        x"2325", x"001F", x"3019", x"003F", x"11A0", x"003F", x"065E", x"001E",
        x"0C4B", x"0019", x"1C0B", x"003F", x"118F", x"003F", x"054E", x"003F",
        x"03B2", x"003F", x"014E", x"003F", x"0188", x"003F", x"0C48", x"000B",
        x"0508", x"003F", x"0C5E", x"003F", x"0409", x"003F", x"044A", x"003F",
        x"048A", x"003F", x"0C1E", x"000B", x"05CC", x"003F", x"060C", x"0022",
        x"0ACF", x"003F", x"0ACF", x"003F", x"03DE", x"003F", x"058D", x"003F",
        x"0B0D", x"003F", x"0CDE", x"003F", x"021E", x"000D", x"0668", x"003F",
        x"0C1E", x"003F", x"014E", x"003F", x"010B", x"003F", x"0668", x"003F",
        x"0C4B", x"0017", x"084B", x"0008", x"085E", x"000B", x"034B", x"0008",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000"
    );
    -- theme map: bank, theme, class -> candy
    constant C_TROM : t_rom8 := (
        x"00", x"36", x"3D", x"2C", x"35", x"22", x"26", x"30",
        x"43", x"46", x"29", x"28", x"4C", x"24", x"50", x"53",
        x"0D", x"0E", x"11", x"17", x"0C", x"12", x"0F", x"20",
        x"44", x"47", x"10", x"14", x"4D", x"4F", x"21", x"55",
        x"00", x"04", x"1E", x"01", x"08", x"05", x"09", x"02",
        x"3D", x"0B", x"37", x"06", x"3F", x"4F", x"03", x"07",
        x"57", x"1E", x"18", x"1F", x"1A", x"1D", x"20", x"1C",
        x"44", x"19", x"49", x"1B", x"58", x"21", x"52", x"55",
        x"3E", x"36", x"37", x"0E", x"35", x"40", x"0A", x"03",
        x"45", x"5D", x"37", x"2C", x"4B", x"4E", x"51", x"5A",
        x"31", x"1F", x"39", x"2A", x"3C", x"38", x"2E", x"2B",
        x"44", x"56", x"47", x"1A", x"4D", x"21", x"52", x"5F",
        x"37", x"34", x"31", x"32", x"33", x"30", x"3B", x"2E",
        x"43", x"46", x"48", x"4A", x"4C", x"4E", x"51", x"53",
        x"00", x"37", x"11", x"22", x"23", x"24", x"2F", x"1C",
        x"59", x"5E", x"29", x"3A", x"58", x"24", x"21", x"5A",
        x"37", x"01", x"08", x"34", x"23", x"25", x"42", x"2B",
        x"45", x"1F", x"5E", x"2C", x"4B", x"4E", x"27", x"5A",
        x"5B", x"0B", x"3F", x"16", x"13", x"0A", x"15", x"14",
        x"43", x"0F", x"10", x"12", x"4D", x"21", x"52", x"55",
        x"00", x"5B", x"0B", x"07", x"0D", x"3D", x"3F", x"03",
        x"05", x"40", x"04", x"08", x"09", x"4F", x"02", x"0E",
        x"57", x"09", x"49", x"19", x"21", x"1D", x"1B", x"2D",
        x"45", x"1F", x"18", x"1A", x"58", x"24", x"52", x"56",
        x"00", x"3E", x"04", x"36", x"0B", x"0D", x"05", x"35",
        x"59", x"3E", x"37", x"06", x"2C", x"4F", x"51", x"3D",
        x"42", x"56", x"27", x"39", x"5F", x"3B", x"20", x"41",
        x"43", x"2A", x"5D", x"38", x"3C", x"4D", x"50", x"54",
        x"3D", x"48", x"26", x"25", x"5C", x"32", x"28", x"4B",
        x"59", x"46", x"5C", x"4A", x"4B", x"4C", x"50", x"54",
        x"5B", x"36", x"11", x"5E", x"29", x"22", x"2F", x"2D",
        x"45", x"23", x"37", x"2C", x"58", x"24", x"50", x"5A"
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
    signal grom_q, wg_f, crom_q, wcc_f, pal_q : std_logic_vector(15 downto 0) := (others => '0');
    signal trom_q : std_logic_vector(7 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- controls (free-running decode)
    ------------------------------------------------------------------------
    signal k1, k2, k3, k4, k5, k6, p12 : unsigned(9 downto 0) := (others => '0');
    signal s_adapt, s_white, s_ground, s_both, s_paper, s_bank, s_luma, s_bias : std_logic := '0';
    signal c_il    : std_logic := '0';
    signal c_lout, c_sk_on, c_ct_on : std_logic := '0'; -- lines / (sketch / contour: VENEER INK, never on)
    signal c_theme : unsigned(2 downto 0) := "000";    -- K1 theme
    signal c_sprk, c_sprkf : unsigned(7 downto 0) := (others => '0');   -- K2 sprinkle density, and its floor for "always sprinkled" candies
    signal c_shade : unsigned(3 downto 0) := (others => '0');           -- K4
    signal c_lev   : signed(7 downto 0) := (others => '0');             -- K5 level offset (8-bit luma codes)
    signal c_gloss : unsigned(3 downto 0) := (others => '0');           -- K6
    -- K3 Scale: deadbanded target, glided value (Q10.4), per-mode steps (Q.10 pattern positions)
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
    signal c_nz    : unsigned(2 downto 0) := "010";
    signal c_nlev  : integer range 2 to 8 := 4;
    signal c_width : unsigned(1 downto 0) := "00";     -- outline 0 thin 1 medium 2 bold
    signal c_xthr  : unsigned(6 downto 0) := to_unsigned(60, 7);      -- sketch threshold, 1/16 code
    signal c_cgain : unsigned(4 downto 0) := "01010";  -- contour pre-gain 2..17

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
    signal f_nl    : unsigned(2 downto 0) := "010";    -- the same, minus 2
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
    signal ys17, ys18 : unsigned(7 downto 0) := (others => '0');  -- smoothed luma (after K5 Level)
    signal ys17r, ys18r : unsigned(7 downto 0) := (others => '0');  -- smoothed luma as captured (histogram)
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
    type t_kc3 is array (0 to 2) of unsigned(3 downto 0);
    signal kc3 : t_kc3 := (others => (others => '0'));            -- class of cw3 (the lines follow the candy)
    signal cand30, cand31, cand32 : unsigned(6 downto 0) := (others => '0');      -- candy id
    signal r31_w, r32_w0, r33_w0 : std_logic_vector(15 downto 0) := (others => '0');   -- geometry word 0
    signal r32_w1, r33_w1, r34_w1 : std_logic_vector(7 downto 0) := (others => '0');   -- geometry word 1
    signal cw33, cw34 : std_logic_vector(15 downto 0) := (others => '0');              -- colour word 0
    signal cx34 : std_logic_vector(5 downto 0) := (others => '0');                     -- colour word 1
    signal ym33 : unsigned(7 downto 0) := (others => '0');
    signal ps34a : signed(10 downto 0) := (others => '0');
    signal ps34b, ps35 : signed(12 downto 0) := (others => '0');
    signal ps36 : signed(10 downto 0) := (others => '0');          -- the picture's shade (10-bit luma units)
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
    -- candy generator (F domain)
    ------------------------------------------------------------------------
    -- relief line buffer: per pixel pair, the height of the line above (3:2)
    -- and of the line above that (1:0)
    type t_rl is array (0 to 1023) of std_logic_vector(3 downto 0);
    signal rlram : t_rl := (others => (others => '0'));
    attribute no_rw_check of rlram : signal is true;
    signal rl_q, rl_d0, rl_d1, rl_d2 : std_logic_vector(3 downto 0) := (others => '0');
    -- coordinates: Q14.10 pattern positions (64 per cell at size 0)
    signal xacc, yacc : unsigned(23 downto 0) := (others => '0');
    signal yfirst : std_logic := '1';
    signal px : unsigned(10 downto 0) := (others => '0');
    signal run, runx, st_r, st_d1, st_d2, rc_r, ls_r : std_logic := '0';
    signal g_x, g_y : unsigned(13 downto 0) := (others => '0');
    signal g_u, g_v : unsigned(15 downto 0) := (others => '0');
    -- C1 axes, C2 size / stagger
    signal f1_a, f1_b : unsigned(13 downto 0) := (others => '0');
    signal f1_size : std_logic_vector(1 downto 0) := "00";
    signal f1_stg, f1_hst, f1_wide : std_logic := '0';
    signal f2_pa, f2_pb, f3_pa, f3_pb, f4_pa, f4_pb, f5_pa, f5_pb : unsigned(5 downto 0) := (others => '0');
    signal f2_ca, f2_cb : unsigned(4 downto 0) := (others => '0');
    -- C3 hash, C4 jitter / folds / quarter turn
    signal f3_h : std_logic_vector(15 downto 0) := (others => '0');
    signal f3_ia0, f3_ib0, f3_rot, f3_sprk, f4_ia0, f4_ib0, f5_ia0, f5_ib0 : std_logic := '0';
    signal f3_jit, f3_var : std_logic_vector(1 downto 0) := "00";
    signal f4_da, f4_db, f5_da, f5_db : unsigned(4 downto 0) := (others => '0');
    signal f4_na, f4_nb, f4_sw, f5_na, f5_nb : std_logic := '0';
    signal f4_hk, f5_hk : unsigned(2 downto 0) := "000";
    signal f4_hp, f5_hp, f4_cs, f5_cs, f4_hs, f5_hs : std_logic_vector(1 downto 0) := "00";   -- presence, chip size, silhouette / size bits
    signal f4_cr : unsigned(4 downto 0) := (others => '0');
    signal f4_cr2 : unsigned(3 downto 0) := (others => '0');
    -- C5 features
    signal f5_r2 : unsigned(10 downto 0) := (others => '0');
    signal f5_r2e : unsigned(12 downto 0) := (others => '0');
    signal f5_l1, f5_oct, f5_df, f5_ch : unsigned(5 downto 0) := (others => '0');
    signal f5_dd : signed(5 downto 0) := (others => '0');
    signal f5_li, f5_sc : unsigned(4 downto 0) := (others => '0');
    signal f5_sm : unsigned(6 downto 0) := (others => '0');
    signal f5_hg, f5_gs, f5_cp1, f5_cp2, f5_cpa, f5_cpb : std_logic := '0';
    signal f5_lit : signed(6 downto 0) := (others => '0');
    signal f5_shape : std_logic_vector(4 downto 0) := (others => '0');
    signal f5_var : std_logic_vector(1 downto 0) := "00";
    -- C6 shape
    signal f6_reg, f6_ht, f6_lsel : unsigned(1 downto 0) := "00";
    signal f6_k : unsigned(2 downto 0) := "000";
    signal f6_lsh, f6_limb, f6_spc, f6_linv : std_logic := '0';
    signal f6_r4 : unsigned(3 downto 0) := "1000";
    signal f6_lit : signed(6 downto 0) := (others => '0');
    signal f6_lita : signed(5 downto 0) := (others => '0');
    -- C7 shade, C8 palette index, C9..C12 colour
    signal f7_sel, f7_k : unsigned(2 downto 0) := "000";
    signal f7_sv : signed(8 downto 0) := (others => '0');
    signal f7_rel : signed(6 downto 0) := (others => '0');
    signal f7_ca, f7_cb, f7_cc : unsigned(5 downto 0) := (others => '0');
    signal f7_asst : std_logic_vector(1 downto 0) := "00";
    signal f8_idx : unsigned(5 downto 0) := (others => '0');
    signal f8_e : signed(9 downto 0) := (others => '0');
    signal f9_q1 : signed(11 downto 0) := (others => '0');
    signal f9_q2, f10_ey : signed(13 downto 0) := (others => '0');
    signal f10_y6 : unsigned(5 downto 0) := (others => '0');
    signal f10_u5, f10_v5, f11_u5, f11_v5, f12_u5, f12_v5 : unsigned(4 downto 0) := "10000";
    signal f11_yb : signed(11 downto 0) := (others => '0');
    signal f11_ps : signed(10 downto 0) := (others => '0');
    signal f12_ys : signed(12 downto 0) := (others => '0');
    -- sprinkle overlay
    signal s1_h : std_logic_vector(15 downto 0) := (others => '0');
    signal s1_pa, s1_pb : unsigned(5 downto 0) := (others => '0');
    signal s2_da, s2_db, s3_da, s3_db : unsigned(4 downto 0) := (others => '0');
    signal s2_same : std_logic := '0';
    signal s2_o, s3_o : std_logic_vector(1 downto 0) := "00";
    signal s2_p, s3_p : unsigned(7 downto 0) := (others => '0');
    signal s2_k, s3_k, s4_k, s5_k, s6_k : unsigned(2 downto 0) := "000";
    signal s3_big, s3_small : unsigned(5 downto 0) := (others => '0');
    signal s4_on, s5_on, s6_on : std_logic := '0';
    -- grain
    signal g1_h : std_logic_vector(15 downto 0) := (others => '0');
    signal g1_dot : std_logic := '0';
    type t_gr is array (2 to 6) of std_logic_vector(5 downto 0);     -- {k(2:0), np, light, dark}
    signal gr : t_gr := (others => (others => '0'));

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
        variable v_l  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            k1 <= unsigned(registers_in(0)); k2 <= unsigned(registers_in(1));
            k3 <= unsigned(registers_in(2)); k4 <= unsigned(registers_in(3));
            k5 <= unsigned(registers_in(4)); k6 <= unsigned(registers_in(5));
            p12 <= unsigned(registers_in(7));
            -- switches: S7 Lines, S8 Split, S9 Bank, S10 Angle, S11 Map
            c_lout <= registers_in(6)(0);
            s_adapt <= registers_in(6)(1);
            s_bank <= registers_in(6)(2);
            s_bias <= registers_in(6)(3);
            s_luma <= registers_in(6)(4);
            -- (VENEER INK controls of the shared matte code: fixed)
            s_white <= '0'; s_ground <= '0'; s_both <= '0'; s_paper <= '0';
            c_sk_on <= '0'; c_ct_on <= '0';

            -- timing class from the timing ID: interlace flag (field lines) and the
            -- steps that put the candy in virtual 1080-line units
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

            -- K1 Theme, 8 zones
            if k1 < 8 then c_theme <= k1(2 downto 0);
            elsif (k1(6 downto 3) /= "0000" and k1(6 downto 3) /= "1111") or k1(9 downto 3) = "1111111" then c_theme <= k1(9 downto 7); end if;
            -- (the top of the last zone is not a zone edge: a knob resting at full scale must still select it)
            -- K2 Sprinkles: share of the sprinkle cells that carry one (0 = none);
            -- "always sprinkled" candies never go below 150 / 256
            c_sprk <= k2(9 downto 2);
            if k2(9 downto 2) < 150 then c_sprkf <= to_unsigned(150, 8); else c_sprkf <= k2(9 downto 2); end if;
            -- K4 Shade, K6 Gloss
            c_shade <= k4(9 downto 6);
            c_gloss <= k6(9 downto 6);
            -- K5 Level: bipolar offset of the luma going into the quantizer, +/-64
            -- codes, with a dead band so the centre is exactly unchanged
            v_l := signed('0' & k5) - 512;
            if v_l > -16 and v_l < 16 then c_lev <= (others => '0');
            else c_lev <= v_l(10 downto 3); end if;
            c_width <= "00";                                    -- line footprint: thin (one sample: 2 px x 1 line at 1080p)

            -- K3 Scale.  A 4-count deadband holds the target against pot noise (the
            -- far edge of the screen magnifies one count of scale), then the value
            -- glides a quarter of the way per frame.
            c_kd <= signed('0' & k3) - signed('0' & c_ktg);
            if c_kd > 3 or c_kd < -4 then c_ktg <= k3; end if;
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

            -- P12 Variety: 7 zones (2..8 levels) = top bits of 7 x p12
            v_q7 := (p12 & "000") - resize(p12, 13);
            if (v_q7(9 downto 6) /= "0000" or v_q7(12 downto 10) = "000")
               and (v_q7(9 downto 6) /= "1111" or v_q7(12 downto 10) = "110") then c_nz <= v_q7(12 downto 10); end if;
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
        variable v_k29 : unsigned(3 downto 0);
        variable v_ysl : signed(9 downto 0);
        variable v_d9 : signed(8 downto 0);
        variable v_pq : signed(12 downto 0);
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
                -- (SUGARCOAT K5 Level: the quantizer sees the luma plus an offset; the
                -- histogram keeps the luma as captured, so Adaptive does not cancel it)
                v_ysl := signed(resize(unsigned(dr_c(15 downto 8)), 10)) + resize(c_lev, 10);
                if v_ysl < 0 then ys17 <= (others => '0'); elsif v_ysl > 255 then ys17 <= (others => '1');
                else ys17 <= unsigned(v_ysl(7 downto 0)); end if;
                ys18 <= ys17;
                ys17r <= unsigned(dr_c(15 downto 8)); ys18r <= ys17r;
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
                    hist_ra <= ys18r(7 downto 2); hs_c(0) <= '1';
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
                -- class of the region (from the stage 29 label, p_roms) -> candy id at 30;
                -- geometry words at 31 / 32, colour words at 33 / 34, carried to where
                -- the F stages read them
                cand30 <= unsigned(trom_q(6 downto 0)); cand31 <= cand30; cand32 <= cand31;
                r31_w <= grom_q; r32_w0 <= r31_w; r33_w0 <= r32_w0;
                r32_w1 <= wg_f(7 downto 0); r33_w1 <= r32_w1; r34_w1 <= r33_w1;
                cw33 <= crom_q; cw34 <= cw33; cx34 <= wcc_f(5 downto 0);
                -- K4 Shade: (bin centroid - 128) x shade / 4 in 10-bit units, at stage 36
                -- (one add of gated operands per stage)
                ym33 <= cen(to_integer(ld(32)(7 downto 5)));
                v_d9 := signed(resize(ym33, 9)) - to_signed(128, 9);
                v_pq := (others => '0');
                if c_shade(0) = '1' then v_pq := resize(v_d9, 13); end if;
                if c_shade(1) = '1' then ps34a <= resize(v_pq + shift_left(resize(v_d9, 13), 1), 11); else ps34a <= resize(v_pq, 11); end if;
                v_pq := (others => '0');
                if c_shade(2) = '1' then v_pq := shift_left(resize(v_d9, 13), 2); end if;
                if c_shade(3) = '1' then ps34b <= v_pq + shift_left(resize(v_d9, 13), 3); else ps34b <= v_pq; end if;
                ps35 <= resize(ps34a, 13) + ps34b;
                ps36 <= resize(shift_right(ps35, 2), 11);

                ----------------------------------------------------------
                -- STAGE 30..35: INK.  Outline = the cleaned label differs from
                -- the left / right neighbour or the cleaned row above; sketch and
                -- contour come back from the ink history RAM (two lines / one
                -- line old) so they land on the matte row.
                ----------------------------------------------------------
                cw3(0) <= cl29; cw3(1) <= cw3(0); cw3(2) <= cw3(1);
                if hf_c(29) = '1' then cw3(1) <= cl29; cw3(2) <= cl29; end if;
                -- (SUGARCOAT: a line is drawn only where the candy changes: compare classes)
                v_k29 := f_cls(cl29, s_luma, f_nl);
                kc3(0) <= v_k29; kc3(1) <= kc3(0); kc3(2) <= kc3(1);
                if hf_c(29) = '1' then kc3(1) <= v_k29; kc3(2) <= v_k29; end if;
                if hv_c(29) = '1' then
                    if hf_c(29) = '1' then cnt30 <= (others => '0'); else cnt30 <= cnt30 + 1; end if;
                end if;
                cnt30_d <= cnt30; cnt30_dd <= cnt30_d;
                if v_k29 /= f_cls(ab29, s_luma, f_nl) then da30 <= '1'; else da30 <= '0'; end if;
                da31 <= da30;
                -- (cw3 at step v+31 holds (v+1, v, v-1); da31 = differs from above (v))
                if kc3(1) /= kc3(2) or da31 = '1' then t32 <= '1'; else t32 <= '0'; end if;
                if kc3(1) /= kc3(0) then dr32 <= '1'; else dr32 <= '0'; end if;
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
                ho_c_y <= ho_n_y; ho_c_lum <= ho_n_lum; ho_c_a <= ho_n_a;
            end if;

            -- histogram sample count of the frame (the adaptive placement needs
            -- the total); frame anchor is in vblank, the streams are idle
            if vs_edge = '1' then
                htot <= hcnt; hcnt <= (others => '0');
            end if;
        end if;
    end process p_h;

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
                        f_nlev <= c_nlev; f_nl <= to_unsigned(c_nlev - 2, 3);
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
    -- ROMs (block RAM, read every clock).  The theme map is addressed straight
    -- from the cleaned label of stage 29 (its class is a few LUTs); the candy
    -- ROMs hold 2 words per candy: word 0 is read while ce = '0' (caught by an H
    -- register at the next enable), word 1 a clock later (caught here, then by
    -- an H register an enable after that).
    ------------------------------------------------------------------------
    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            trom_q <= C_TROM(to_integer(s_bank & c_theme & f_cls(cl29, s_luma, f_nl)));
            grom_q <= C_GROM(to_integer(cand30 & ce));
            if ce = '0' then wg_f <= grom_q; end if;
            crom_q <= C_CROM(to_integer(cand32 & ce));
            if ce = '0' then wcc_f <= crom_q; end if;
            pal_q <= C_PAL(to_integer("00" & f8_idx));
        end if;
    end process p_roms;

    ------------------------------------------------------------------------
    -- CANDY COORDINATES (F domain, label independent).  The confectionery is
    -- one fixed sheet seen through the regions.  Two accumulators in Q14.10
    -- pattern positions (64 positions per cell at size 0): x steps every pixel,
    -- y every active line; the steps carry the Scale knob and the timing's
    -- pixel size (virtual 1080-line units).  T0 registers x, y and the bias
    -- axes x + y, x - y one clock ahead of C1.  Nothing moves on its own.
    -- (strobes are registered a clock ahead, as in VENEER: `ce and hf_c(k)`
    -- taken combinationally was a 13 ns routing path)
    ------------------------------------------------------------------------
    p_geo : process(clk)
    begin
        if rising_edge(clk) then
            st_r <= (not ce) and hf_c(30);           -- '1' in the cycle before pixel 0 enters C1
            st_d1 <= st_r; st_d2 <= st_d1;           -- the relief counter starts two stages later (C6 where APPLIQUE had T4)
            rc_r <= (not ce) and (not hv_c(30));
            ls_r <= (not ce) and hf_c(1);            -- line start, in blanking
            if runx = '1' or st_r = '1' then xacc <= xacc + resize(c_xs, 24); end if;
            if run = '1' then px <= px + 1; end if;
            if rc_r = '1' then run <= '0'; runx <= '0'; end if;
            if st_r = '1' then runx <= '1'; end if;
            if st_d2 = '1' then run <= '1'; end if;
            if ls_r = '1' then
                run <= '0'; runx <= '0'; px <= (others => '0');
                xacc <= (others => '0');
                if yfirst = '1' then yacc <= (others => '0'); else yacc <= yacc + resize(c_ys, 24); end if;
                yfirst <= '0';
            end if;
            -- frame start (vblank): y back to the top
            if vs_edge = '1' then
                run <= '0'; runx <= '0'; yfirst <= '1';
            end if;
            -- T0
            g_x <= xacc(23 downto 10); g_y <= yacc(23 downto 10);
            g_u <= xacc(23 downto 8) + yacc(23 downto 8);
            g_v <= xacc(23 downto 8) - yacc(23 downto 8);
        end if;
    end process p_geo;

    -- relief line buffer.  Address A = px / 2 (px runs 4 ahead of the pixel
    -- whose height is at C6, a constant offset).  Read at A + 2 (not yet rewritten on
    -- this line), three pair delays -> rl_d2 = the word of A - 1: the sample 2..3
    -- pixels to the left, one and two lines up.  Write {height of this line,
    -- height of the line above} at A: read and write addresses are always 2 apart.
    p_relief : process(clk)
    begin
        if rising_edge(clk) then
            rl_q <= rlram(to_integer(px(10 downto 1) + 2));
            if run = '1' and px(0) = '0' then
                rlram(to_integer(px(10 downto 1))) <= std_logic_vector(f6_ht) & rl_d1(3 downto 2);
            end if;
            if px(0) = '1' then rl_d0 <= rl_q; rl_d1 <= rl_d0; rl_d2 <= rl_d1; end if;
        end if;
    end process p_relief;

    ------------------------------------------------------------------------
    -- CANDY + COLOUR (F domain, per pixel; the recipe and the colours of the
    -- pixel's region are read from the H stages named below.  An F stage 2k
    -- clocks before O1 reads H stage 37 - k for both pixels of a pair, so H
    -- values are only read at even offsets.)
    --   C1  (-12, H 31) axes: straight (x, y) or bias (x + y, x - y), turned or not
    --   C2  (-11)       size (11-bit field: 5 cell bits + 6 position bits), stagger
    --   C3  (-10, H 32) cell hash
    --   C4  (-9)        jitter, folds (distance from the cell's centre lines), random quarter turn
    --   C5  (-8, H 33)  features: sums, differences, compares, squares, the light term
    --   C6  (-7)        shape -> region, assortment index, height, shading selects: bit tests only
    --   C7  (-6, H 34)  shade (light term, shading model, grain, sprinkles), relief
    --   C8  (-5)        palette index; e = shade + relief
    --   C9  (-4)        palette ROM; partial products e x gloss
    --   C10 (-3)        palette word in fabric; e x gloss
    --   C11 (-2, H 36)  luma = palette + e x gloss / 8
    --   C12 (-1)        + the picture's shade
    -- Geometry word 0: [4:0] shape [6:5] variant [8:7] size [9] bias [10] turn [11] stagger
    --                  [12] half stagger [13] wide [15:14] jitter
    --          word 1: [1:0] shading [3:2] grain [4] grain on the piece [5] quarter turn
    --                  [6] 1-D hash [7] always sprinkled                       (tables.py)
    ------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_a, v_b : unsigned(13 downto 0);
        variable v_fa, v_fb : unsigned(10 downto 0);
        variable v_hi, v_pa, v_pb, v_ja, v_jb, v_w1, v_w2, v_l1, v_x, v_big, v_small : unsigned(5 downto 0);
        variable v_cb, v_da, v_db, v_fa5, v_fb5, v_c5 : unsigned(4 downto 0);
        variable v_rp, v_st, v_na, v_nb, v_sw, v_hg, v_useb, v_n, v_same : std_logic;
        variable v_key : std_logic_vector(15 downto 0);
        variable v_h : std_logic_vector(15 downto 0);
        variable v_cs : std_logic_vector(1 downto 0);
        variable v_sqa, v_sqb : unsigned(9 downto 0);
        variable v_dd, v_ad : signed(5 downto 0);
        variable v_lit : signed(6 downto 0);
        variable v_d : unsigned(4 downto 0);
        variable v_d2 : signed(6 downto 0);
        -- shape
        variable q16, q32, q64, q128, q192, q256, q384, q512, q640, q768, q960 : boolean;
        variable e64, e128, e192, e256, e384, e512, e768 : boolean;
        variable l7, l10, l14, v_ins, v_lim, v_sp, v_big2, v_w, v_rim, v_rind, v_mem, v_txt, v_line, v_frame, v_ridge : boolean;
        variable v_dz, v_edge, v_sa, v_sb, v_top, v_pearl, v_hole, v_circ, v_two, v_na1, v_nb1 : boolean;
        variable v_reg, v_ht, v_lsel, v_hta, v_htb : unsigned(1 downto 0);
        variable v_k : unsigned(2 downto 0);
        variable v_lsh, v_limb, v_spc, v_linv : std_logic;
        variable v_r4, v_t4 : unsigned(3 downto 0);
        variable v_hs, v_hd : signed(6 downto 0);
        variable v_var : integer range 0 to 3;
        variable v_s6 : unsigned(5 downto 0);
        -- shade
        variable v_L : signed(7 downto 0);
        variable v_piece, v_gz, v_gd, v_gl, v_gn, v_spec : boolean;
        variable v_ci : integer range -64 to 127;
        variable v_cv : std_logic_vector(3 downto 0);
        variable v_sel : unsigned(2 downto 0);
        variable v_sv : signed(8 downto 0);
        variable v_ref : unsigned(1 downto 0);
        variable v_r3 : signed(2 downto 0);
        variable v_m, v_idx : unsigned(5 downto 0);
        variable v_q : signed(13 downto 0);
        variable v_dens : unsigned(7 downto 0);
        variable v_rod : boolean;
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------
            -- C1: axes
            ----------------------------------------------------------
            if (r31_w(9) xor s_bias) = '1' then v_a := g_u(15 downto 2); v_b := g_v(15 downto 2);
            else v_a := g_x; v_b := g_y; end if;
            if r31_w(10) = '1' then f1_a <= v_b; f1_b <= v_a; else f1_a <= v_a; f1_b <= v_b; end if;
            f1_size <= r31_w(8 downto 7); f1_stg <= r31_w(11); f1_hst <= r31_w(12); f1_wide <= r31_w(13);

            ----------------------------------------------------------
            -- C2: size (a case of fixed slices, no barrel); wide = two rows of
            -- cells per cell height; stagger = every other row slides half a cell
            -- (a real add: the cell index must follow for the hash)
            ----------------------------------------------------------
            case f1_size is
                when "00"   => v_fa := f1_a(10 downto 0); v_fb := f1_b(10 downto 0);
                when "01"   => v_fa := f1_a(11 downto 1); v_fb := f1_b(11 downto 1);
                when "10"   => v_fa := f1_a(12 downto 2); v_fb := f1_b(12 downto 2);
                when others => v_fa := f1_a(13 downto 3); v_fb := f1_b(13 downto 3);
            end case;
            if f1_wide = '1' then v_pb := v_fb(4 downto 0) & '0'; v_cb := v_fb(9 downto 5); v_rp := v_fb(5);
            else v_pb := v_fb(5 downto 0); v_cb := v_fb(10 downto 6); v_rp := v_fb(6); end if;
            v_st := (f1_stg and v_rp) xor (f1_hst and v_fb(5));
            if v_st = '1' then v_hi := v_fa(10 downto 5) + 1; else v_hi := v_fa(10 downto 5); end if;
            f2_pa <= v_hi(0) & v_fa(4 downto 0); f2_ca <= v_hi(5 downto 1);
            f2_pb <= v_pb; f2_cb <= v_cb;

            ----------------------------------------------------------
            -- C3 (H 32): cell hash (1-D for candies coloured per column)
            ----------------------------------------------------------
            if r32_w1(6) = '1' then v_cb := (others => '0'); else v_cb := f2_cb; end if;
            v_key := std_logic_vector(v_cb(2 downto 0)) & std_logic_vector(f2_ca(2 downto 0)) & std_logic_vector(v_cb) & std_logic_vector(f2_ca);
            f3_h <= f_spn(v_key, x"A53C", x"3C96", x"69A5", x"5AC3");
            f3_pa <= f2_pa; f3_pb <= f2_pb; f3_ia0 <= f2_ca(0); f3_ib0 <= f2_cb(0);
            f3_jit <= r32_w0(15 downto 14); f3_rot <= r32_w1(5); f3_var <= r32_w0(6 downto 5); f3_sprk <= r32_w1(7);

            ----------------------------------------------------------
            -- C4: jitter (sign-extended slices of the hash), folds, quarter turn
            ----------------------------------------------------------
            v_h := f3_h;
            case f3_jit is
                when "01"   => v_ja := unsigned(resize(signed(v_h(5 downto 3)), 6));
                               v_jb := unsigned(resize(signed(v_h(10 downto 8)), 6));
                when "10"   => v_ja := unsigned(resize(signed(v_h(6 downto 3)), 6));
                               v_jb := unsigned(resize(signed(v_h(11 downto 8)), 6));
                when "11"   => v_ja := unsigned(resize(signed(v_h(7 downto 3)), 6));
                               v_jb := unsigned(resize(signed(v_h(12 downto 8)), 6));
                when others => v_ja := (others => '0'); v_jb := (others => '0');
            end case;
            v_pa := f3_pa + v_ja; v_pb := f3_pb + v_jb;
            v_sw := f3_rot and v_h(13);
            if v_sw = '1' then
                f4_da <= f_fold(v_pb); f4_db <= f_fold(v_pa); f4_na <= not v_pb(5); f4_nb <= not v_pa(5);
            else
                f4_da <= f_fold(v_pa); f4_db <= f_fold(v_pb); f4_na <= not v_pa(5); f4_nb <= not v_pb(5);
            end if;
            f4_pa <= v_pa; f4_pb <= v_pb; f4_sw <= v_sw;
            f4_hk <= unsigned(v_h(2 downto 0)); f4_hp <= v_h(10 downto 9); f4_hs <= v_h(15 downto 14);
            f4_ia0 <= f3_ia0; f4_ib0 <= f3_ib0;
            -- chip size index (by variant / hash) and its two radii
            case f3_var is
                when "00"   => v_cs := "11";
                when "01"   => v_cs := "01";
                when "10"   => v_cs := "00";
                when others => v_cs := v_h(12 downto 11);
            end case;
            f4_cs <= v_cs;
            case v_cs is
                when "00"   => f4_cr <= to_unsigned(10, 5); f4_cr2 <= to_unsigned(8, 4);
                when "01"   => f4_cr <= to_unsigned(12, 5); f4_cr2 <= to_unsigned(9, 4);
                when "10"   => f4_cr <= to_unsigned(17, 5); f4_cr2 <= to_unsigned(13, 4);
                when others => f4_cr <= to_unsigned(20, 5); f4_cr2 <= to_unsigned(15, 4);
            end case;

            ----------------------------------------------------------
            -- C5 (H 33): features (every carry chain of the generator is here)
            ----------------------------------------------------------
            v_da := f4_da; v_db := f4_db;
            v_sqa := f_sq5(v_da); v_sqb := f_sq5(v_db);
            f5_r2 <= resize(v_sqa, 11) + resize(v_sqb, 11);                 -- round distance, squared
            f5_r2e <= resize(v_sqa, 13) + shift_left(resize(v_sqb, 13), 2);   -- 2:1 ellipse
            v_l1 := resize(v_da, 6) + resize(v_db, 6);                      -- diamond distance
            v_dd := signed(resize(v_da, 6)) - signed(resize(v_db, 6));
            v_hg := f_b2sl(v_da > v_db);
            v_w1 := resize(v_da, 6) + resize(v_db(4 downto 1), 6);
            v_w2 := resize(v_db, 6) + resize(v_da(4 downto 1), 6);
            f5_l1 <= v_l1; f5_dd <= v_dd; f5_hg <= v_hg;
            if v_hg = '1' then f5_oct <= v_w1; f5_li <= v_da; else f5_oct <= v_w2; f5_li <= v_db; end if;   -- octagon / square distance
            f5_sm <= resize(f4_pa, 7) + resize(f4_pb, 7);
            f5_df <= f4_pa - f4_pb;
            f5_ch <= f4_pb + resize(v_da, 6);                               -- chevron phase
            f5_sc <= (not f4_pb(4 downto 0)) - v_sqa(9 downto 5);           -- scallop phase
            f5_gs <= f_b2sl(f4_pb(5 downto 1) > v_da);                      -- inside the triangle
            f5_cp1 <= f_b2sl(v_w1 < resize(f4_cr, 6)); f5_cp2 <= f_b2sl(v_w2 < resize(f4_cr, 6));
            f5_cpa <= f_b2sl(v_da < resize(f4_cr2, 5)); f5_cpb <= f_b2sl(v_db < resize(f4_cr2, 5));
            -- the light term (light from the upper left), sign-magnitude so there
            -- is no negation: straight axes -(a + b), bias axes -2 x the axis that
            -- faces the light
            if (r33_w0(9) xor s_bias) = '0' then
                if f4_na = f4_nb then
                    if f4_na = '1' then v_lit := signed('0' & v_l1); else v_lit := not signed('0' & v_l1); end if;
                else
                    if f4_na = '1' then v_lit := resize(v_dd, 7); else v_lit := not resize(v_dd, 7); end if;
                end if;
            else
                v_useb := f4_sw xor r33_w0(10);
                if v_useb = '1' then v_d := v_db; v_n := f4_nb; else v_d := v_da; v_n := f4_na; end if;
                if v_n = '1' then v_lit := signed('0' & v_d & '0'); else v_lit := not signed('0' & v_d & '0'); end if;
            end if;
            f5_lit <= v_lit;
            f5_da <= v_da; f5_db <= v_db; f5_na <= f4_na; f5_nb <= f4_nb; f5_pa <= f4_pa; f5_pb <= f4_pb;
            f5_hk <= f4_hk; f5_hp <= f4_hp; f5_hs <= f4_hs; f5_cs <= f4_cs; f5_ia0 <= f4_ia0; f5_ib0 <= f4_ib0;
            f5_shape <= r33_w0(4 downto 0); f5_var <= r33_w0(6 downto 5);

            ----------------------------------------------------------
            -- C6: shape (lookup-style logic on the features)
            --   reg  0 ground (colour A)  1 piece (B, assorted by k)  2 detail (C)  3 white
            --   ht   relief height        lsel 0 flat  1 dome light  2 tube light  3 ramp r4
            ----------------------------------------------------------
            q16 := f_lt(f5_r2, 16); q32 := f_lt(f5_r2, 32); q64 := f_lt(f5_r2, 64); q128 := f_lt(f5_r2, 128); q192 := f_lt(f5_r2, 192);
            q256 := f_lt(f5_r2, 256); q384 := f_lt(f5_r2, 384); q512 := f_lt(f5_r2, 512); q640 := f_lt(f5_r2, 640); q768 := f_lt(f5_r2, 768);
            q960 := f_lt(f5_r2, 960);
            e64 := f_lt(f5_r2e, 64); e128 := f_lt(f5_r2e, 128); e192 := f_lt(f5_r2e, 192); e256 := f_lt(f5_r2e, 256); e384 := f_lt(f5_r2e, 384);
            e512 := f_lt(f5_r2e, 512); e768 := f_lt(f5_r2e, 768);
            l7 := f5_lit(6) = '0' and f5_lit(5 downto 3) /= "000";                       -- light not f_lt(term, 8)
            l10 := f5_lit(6) = '0' and (f5_lit(5 downto 4) /= "00" or f5_lit(3 downto 2) = "11");   -- >= 12
            l14 := f5_lit(6) = '0' and f5_lit(5 downto 4) /= "00";                      -- >= 16
            v_na1 := f5_na = '1'; v_nb1 := f5_nb = '1';
            v_w := (f5_na xor f5_nb xor f5_hg) = '1';
            v_da := f5_da; v_db := f5_db; v_pa := f5_pa; v_pb := f5_pb;
            v_var := to_integer(unsigned(f5_var));
            v_big2 := f5_hs(0) = '1';
            v_reg := "00"; v_k := f5_hk; v_ht := "00"; v_lsel := "00"; v_lsh := '0'; v_limb := '0'; v_spc := '0';
            v_r4 := "1000"; v_linv := '0';
            v_ins := false; v_lim := false; v_sp := false;
            case to_integer(unsigned(f5_shape)) is
                when 1 =>                                                    -- DISC
                    case v_var is
                        when 0 => v_ins := q768; v_lim := not q512; v_sp := l14 and q256;
                        when 1 => v_ins := q512; v_lim := not q384; v_sp := l10 and q128;
                        when 2 => v_ins := q256; v_lim := not q192; v_sp := l7 and q64;
                        when others =>
                            if v_big2 then v_ins := q512; v_lim := not q384; v_sp := l10 and q128;
                            else v_ins := q256; v_lim := not q192; v_sp := l7 and q64; end if;
                    end case;
                    if v_ins then v_reg := "01"; v_ht := "10"; end if;
                    v_lsel := "01"; if v_var /= 0 then v_lsh := '1'; end if;
                    v_limb := f_b2sl(v_ins and v_lim); v_spc := f_b2sl(v_sp);
                when 2 =>                                                    -- RING / filled disc
                    case v_var is
                        when 0 =>
                            v_ins := q768;
                            if q256 then v_reg := "10"; v_ht := "11"; elsif v_ins then v_reg := "01"; v_ht := "10"; end if;
                            v_limb := f_b2sl(v_ins and not q512); v_spc := f_b2sl(l7 and q128);
                        when 1 =>
                            v_ins := q768 and not q128;
                            if v_ins then v_reg := "01"; v_ht := "10"; end if;
                            v_limb := f_b2sl(v_ins and (not q512 or q192)); v_spc := f_b2sl(l14 and q512 and not q256);
                        when 2 =>
                            v_ins := q512;
                            if q128 then v_reg := "10"; v_ht := "11"; elsif v_ins then v_reg := "01"; v_ht := "10"; end if;
                            v_limb := f_b2sl(v_ins and not q384); v_spc := f_b2sl(l7 and q64);
                        when others =>
                            v_ins := q512 and not q64;
                            if v_ins then v_reg := "01"; v_ht := "10"; end if;
                            v_limb := f_b2sl(v_ins and not q384); v_spc := f_b2sl(l10 and q384 and not q192);
                    end case;
                    v_lsel := "01"; if v_var >= 2 then v_lsh := '1'; end if;
                when 3 =>                                                    -- BEAN (2:1 ellipse)
                    case v_var is
                        when 0 => v_ins := e768; v_lim := not e512; v_sp := l10 and e192;
                        when 1 => v_ins := e512; v_lim := not e384; v_sp := l7 and e128;
                        when 2 => v_ins := e256; v_lim := not e192; v_sp := l7 and e64;
                        when others =>
                            if v_big2 then v_ins := e768; v_lim := not e512; v_sp := l10 and e192;
                            else v_ins := e512; v_lim := not e384; v_sp := l7 and e128; end if;
                    end case;
                    if v_ins then v_reg := "01"; v_ht := "10"; end if;
                    v_lsel := "01"; v_lsh := '1'; v_limb := f_b2sl(v_ins and v_lim); v_spc := f_b2sl(v_sp);
                when 4 =>                                                    -- CHIP (rounded chunk, four silhouettes)
                    case f5_cs is
                        when "00"   => v_circ := q128;
                        when "01"   => v_circ := q192;
                        when "10"   => v_circ := q384;
                        when others => v_circ := q512;
                    end case;
                    v_ins := v_circ and f5_hp /= "00";
                    case f5_hs is
                        when "01"   => v_ins := v_ins and f5_cpb = '1';
                        when "10"   => v_ins := v_ins and f5_cpa = '1';
                        when "11"   => v_ins := v_ins and f5_cp1 = '1' and f5_cp2 = '1';
                        when others => null;
                    end case;
                    if v_ins then v_reg := "01"; v_ht := "11"; end if;
                    v_lsel := "01"; v_lsh := '1'; v_spc := f_b2sl(v_ins and l7 and f_lt(f5_l1, 6));
                when 5 =>                                                    -- SQUARE (rounded square piece)
                    case v_var is
                        when 0 =>
                            if f5_hs(0) = '1' then v_ins := f_lt(f5_li, 30); else v_ins := f_lt(f5_li, 28); end if;
                            if f5_hs(1) = '1' then v_ins := v_ins and f_lt(f5_l1, 54); else v_ins := v_ins and f_lt(f5_l1, 46); end if;
                            v_lim := not f_lt(f5_li, 26);
                        when 1 => v_ins := f_lt(f5_li, 21) and f_lt(f5_l1, 36); v_lim := not f_lt(f5_li, 18);
                        when 2 => v_ins := f_lt(f5_li, 15) and f_lt(f5_l1, 26); v_lim := not f_lt(f5_li, 12);
                        when others => v_ins := f_lt(f5_li, 23) and f_lt(f5_l1, 40); v_lim := not f_lt(f5_li, 20);
                    end case;
                    if v_ins then
                        v_reg := "01"; v_ht := "11";
                        if v_var = 3 and f_lt(v_db, 8) then v_reg := "10"; end if;
                    end if;
                    v_lsel := "01"; if v_var = 2 then v_lsh := '1'; end if;
                    v_limb := f_b2sl(v_ins and v_lim); v_spc := f_b2sl(v_ins and l10 and f_lt(f5_li, 12));
                when 6 =>                                                    -- WEDGE disc
                    if v_var = 1 then                                        -- citrus slice
                        v_ins := q768; v_rind := v_ins and not q512;
                        v_mem := (not f_slt(f5_dd, -1) and f_slt(f5_dd, 2)) or f_lt(v_da, 2) or f_lt(v_db, 2);
                        if v_ins then
                            v_ht := "10";
                            if (not v_rind) and v_mem then v_reg := "11"; else v_reg := "01"; end if;
                        end if;
                        v_lsel := "11";
                        if v_rind then v_r4 := "0110"; elsif v_mem then v_r4 := "1000"; else v_r4 := "1100"; end if;
                    else
                        if v_var = 2 then v_ins := q512; v_rim := not q384; v_lim := not q384; v_sp := l10 and q128; v_lsh := '1';
                        else v_ins := q768; v_rim := not q640; v_lim := not q512; v_sp := l14 and q256; end if;
                        if v_var = 3 then v_w := (f5_na xor f5_nb) = '1'; end if;
                        if v_ins then
                            v_ht := "10";
                            if v_w and not v_rim then v_reg := "01"; else v_reg := "11"; end if;
                        end if;
                        v_lsel := "01"; v_limb := f_b2sl(v_ins and v_lim); v_spc := f_b2sl(v_sp);
                    end if;
                when 7 =>                                                    -- TARGET (rings, cups, sandwich cookie)
                    case v_var is
                        when 0 =>
                            v_ins := q768; v_k := f5_oct(4 downto 2);
                            if v_ins then v_reg := "01"; v_ht := "10"; end if;
                            v_lsel := "01"; v_limb := f_b2sl(v_ins and not q512); v_spc := f_b2sl(l14 and q256);
                        when 1 =>
                            v_ins := f_lt(f5_oct, 28);
                            if v_ins then
                                v_ht := "10";
                                if f5_oct(2) = '1' then v_reg := "01"; else v_reg := "10"; end if;
                            end if;
                            v_lsel := "01"; v_limb := f_b2sl(v_ins and not f_lt(f5_oct, 24));
                        when others =>
                            v_ins := q768;
                            if v_ins then v_reg := "01"; v_ht := "10"; end if;
                            v_lsel := "11";
                            if q512 then
                                if v_var = 2 then
                                    if q256 and not q192 then v_r4 := "0101"; else v_r4 := "1000"; end if;
                                else
                                    if q64 then v_r4 := "1000";
                                    elsif q384 then
                                        if f5_oct(1) = '1' then v_r4 := "1011"; else v_r4 := "0110"; end if;
                                    else v_r4 := "1000"; end if;
                                end if;
                            else
                                if (f5_l1(2) xor f5_dd(2)) = '1' then v_r4 := "1100"; else v_r4 := "0100"; end if;
                            end if;
                    end case;
                when 8 =>                                                    -- HEART / gems
                    if f5_nb = '1' then v_hs := resize(f5_dd, 7); v_hd := signed('0' & f5_l1);
                    else v_hs := signed('0' & f5_l1); v_hd := resize(f5_dd, 7); end if;
                    if v_var < 2 then
                        v_ins := f_slt(v_hs, 26) and not f_slt(v_hs, -10) and (not v_nb1 or f_lt(v_db, 16)) and f_lt(v_da, 23) and f_slt(v_hd, 34);
                        if v_ins then v_reg := "01"; v_ht := "10"; end if;
                        v_lsel := "01";
                        if v_var = 0 then
                            v_spc := f_b2sl(v_ins and l14 and f_lt(f5_l1, 22) and v_na1 and v_nb1);
                        else
                            v_txt := (v_nb1 and not f_lt(v_db, 4) and f_lt(v_db, 6) and f_lt(v_da, 10)) or ((not v_nb1) and not f_lt(v_db, 2) and f_lt(v_db, 4) and f_lt(v_da, 7));
                            if v_ins and v_txt then v_reg := "10"; end if;
                        end if;
                    else
                        if v_var = 2 then v_ins := f_lt(f5_l1, 28); else v_ins := f_lt(f5_li, 22); end if;
                        if v_ins then v_reg := "01"; v_ht := "11"; end if;
                        v_lsel := "11";
                        v_r4 := to_unsigned(C_FACET(to_integer(unsigned'((not f5_na) & (not f5_nb) & (not f5_hg)))), 4);
                        if v_var = 3 and f_lt(f5_li, 10) then v_r4 := "1010"; end if;
                        v_spc := f_b2sl(v_ins and v_na1 and v_nb1 and f_lt(f5_l1, 7));
                    end if;
                when 9 =>                                                    -- CORN (triangles in rows)
                    v_ins := f5_gs = '1';
                    if v_ins then
                        v_ht := "10"; v_reg := "01";
                        case v_var is
                            when 0 =>
                                if v_pb(5 downto 4) = "11" then v_reg := "01"; elsif v_pb(5 downto 4) = "10" then v_reg := "10"; else v_reg := "11"; end if;
                            when 1 => v_ht := "11";
                            when 2 => null;
                            when others =>
                                if not f_lt(v_pb, 56) then v_reg := "10"; elsif not f_lt(v_pb, 52) then v_reg := "11"; else v_reg := "01"; end if;
                        end case;
                    end if;
                    if v_var = 1 then
                        v_lsel := "11";
                        if v_na1 then v_r4 := "1011"; else v_r4 := "0101"; end if;
                    else
                        v_lsel := "01";
                    end if;
                when 10 =>                                                   -- STRIPE
                    case v_var is
                        when 0 => v_reg := '0' & v_pa(5);
                        when 1 =>
                            if f_lt(v_pa, 24) then v_reg := "01";
                            elsif (not f_lt(v_pa, 36) and f_lt(v_pa, 42)) or (not f_lt(v_pa, 50) and f_lt(v_pa, 56)) then v_reg := "10"; end if;
                        when 2 =>
                            case v_pa(5 downto 4) is
                                when "00"   => v_reg := "01";
                                when "01"   => v_reg := "00";
                                when "10"   => v_reg := "10";
                                when others => v_reg := "11";
                            end case;
                        when others => v_reg := "01"; v_k := v_pa(5 downto 3);
                    end case;
                when 11 =>                                                   -- TUBE (sticks)
                    v_ins := f_lt(v_da, 29);
                    v_s6 := f5_sm(5 downto 0);
                    if v_ins then
                        v_ht := "10"; v_reg := "01";
                        if v_var = 1 then
                            if f_lt(v_s6, 20) then v_reg := "01";
                            elsif (not f_lt(v_s6, 28) and f_lt(v_s6, 34)) or (not f_lt(v_s6, 42) and f_lt(v_s6, 48)) then v_reg := "10";
                            else v_reg := "11"; end if;
                        elsif v_var = 3 and f_lt(v_da, 5) then v_reg := "10"; end if;
                    end if;
                    v_lsel := "10"; v_lsh := '1'; v_limb := f_b2sl(v_ins and not f_lt(v_da, 25));
                    v_spc := f_b2sl(v_na1 and not f_lt(v_da, 11) and f_lt(v_da, 14));
                    if v_var = 2 then
                        v_lsel := "11"; v_r4 := f_tri4(f5_sm(4 downto 0));
                        v_spc := f_b2sl(v_na1 and not f_lt(v_da, 8) and f_lt(v_da, 16) and v_r4 = 15);
                    end if;
                when 12 =>                                                   -- CHECK
                    v_line := f_lt(v_pa(4 downto 0), 2) or f_lt(v_pb(4 downto 0), 2);
                    case v_var is
                        when 0 =>
                            if not v_line then
                                v_ht := "10";
                                if (v_pa(5) xor v_pb(5)) = '1' then v_reg := "01"; else v_reg := "10"; end if;
                            end if;
                        when 1 => if (v_pa(5) xor v_pb(5)) = '1' then v_reg := "01"; else v_reg := "10"; end if;
                        when 2 => if (v_pa(4) xor v_pb(4)) = '1' then v_reg := "01"; else v_reg := "10"; end if;
                        when others =>
                            if v_pa(5) = '1' and v_pb(5) = '1' then v_reg := "01";
                            elsif v_pa(5) = '1' or v_pb(5) = '1' then v_reg := "10"; end if;
                    end case;
                when 13 =>                                                   -- PIPS (chocolate bar segments)
                    v_reg := "01"; v_lsel := "11";
                    if f_lt(f5_li, 25) then v_ht := "11"; elsif f_lt(f5_li, 27) then v_ht := "10"; elsif f_lt(f5_li, 29) then v_ht := "01"; end if;
                    v_frame := f5_li = 14 or f5_li = 15;
                    if (v_var = 0 or v_var = 3) and v_frame then v_r4 := "0101"; end if;
                    if (v_var = 1 or v_var = 3) and q64 then v_reg := "10"; end if;
                when 14 =>                                                   -- WAFER (raised lattice)
                    if v_var = 0 or v_var = 2 then
                        v_ridge := not f_lt(v_da, 26) or not f_lt(v_db, 26);
                    else
                        v_ridge := not f_lt(f_tri4(v_pa(4 downto 0)), 13) or not f_lt(f_tri4(v_pb(4 downto 0)), 13);
                    end if;
                    v_lsel := "11";
                    if v_ridge then v_reg := "01"; v_ht := "11"; v_r4 := "1010";
                    else
                        v_r4 := "0110";
                        if v_var >= 2 then v_reg := "10"; else v_reg := "01"; end if;
                    end if;
                when 15 =>                                                   -- RIDGE (piped lines)
                    if v_var = 1 then v_two := f5_ia0 = '1'; else v_two := v_pa(5) = '1'; end if;
                    v_reg := "01";
                    if (v_var = 1 or v_var = 3) and not v_two then v_reg := "10"; end if;
                    v_lsel := "10"; v_lsh := '1';
                    v_limb := f_b2sl(not f_lt(v_da, 27) or (v_var >= 2 and f_lt(v_da, 2)));
                    if f_lt(v_da, 27) then v_ht := "10"; end if;
                    v_spc := f_b2sl(v_na1 and not f_lt(v_da, 11) and f_lt(v_da, 14));
                when 16 =>                                                   -- ZIGZAG
                    v_c5 := f5_ch(4 downto 0);
                    v_reg := "01";
                    case v_var is
                        when 0 => v_lsel := "11"; v_r4 := f_tri4(v_c5);
                        when 1 => v_dz := f_lt(v_c5, 5); if v_dz then v_reg := "10"; v_ht := "11"; end if;
                        when 2 => if f5_ch(4) = '0' then v_reg := "10"; end if;
                        when others =>
                            v_dz := f_lt(f5_ch, 6) or (not f_lt(f5_ch, 30) and f_lt(f5_ch, 33));
                            if v_dz then v_reg := "10"; v_ht := "11"; end if;
                    end case;
                when 17 =>                                                   -- SCALLOP
                    v_edge := f5_sc(4 downto 2) = "000";
                    v_reg := "01";
                    case v_var is
                        when 0 => if v_edge then v_reg := "10"; v_ht := "11"; end if;
                        when 1 => v_lsel := "11"; v_r4 := f5_sc(4 downto 1); if not v_edge then v_ht := "10"; end if;
                        when 2 => if f5_sc(4) = '0' then v_reg := "10"; end if;
                        when others =>
                            case f5_sc(4 downto 3) is
                                when "00"   => v_reg := "01";
                                when "10"   => v_reg := "10";
                                when others => v_reg := "00";
                            end case;
                    end case;
                when 18 =>                                                   -- BASKET
                    if v_var = 2 then v_top := (f5_ia0 xor f5_ib0) = '1'; else v_top := (v_pa(5) xor v_pb(5)) = '1'; end if;
                    if v_top then v_x := v_pa; else v_x := v_pb; end if;
                    v_reg := "01"; v_lsel := "11";
                    case v_var is
                        when 0 => v_r4 := v_x(3 downto 0);
                        when 1 => if v_x(3 downto 1) = "000" then v_r4 := "0010"; else v_r4 := "1010"; end if;
                        when 2 => v_r4 := v_x(4 downto 1);
                        when others =>                                       -- lattice strips over a filling
                            v_sa := not f_lt(v_pa(4 downto 0), 12); v_sb := not f_lt(v_pb(4 downto 0), 12);
                            v_r4 := "1001";
                            if not (v_sa or v_sb) then v_reg := "10"; end if;
                            v_hta := "00"; v_htb := "00";
                            if v_sa then if v_top then v_hta := "11"; else v_hta := "10"; end if; end if;
                            if v_sb then if v_top then v_htb := "10"; else v_htb := "11"; end if; end if;
                            if v_hta > v_htb then v_ht := v_hta; else v_ht := v_htb; end if;
                    end case;
                when 19 =>                                                   -- QUILT (lattice lines, pearls)
                    v_line := f5_df(5 downto 2) = "0000" or f5_sm(5 downto 2) = "1111";
                    v_reg := "01"; v_lsel := "11";
                    if v_var /= 3 then
                        if v_line then v_r4 := "0101"; else v_r4 := "1000"; v_ht := "10"; end if;
                    end if;
                    case v_var is
                        when 0 => v_pearl := false;
                        when 1 => v_pearl := q32;
                        when 2 => v_pearl := q32 or (not f_lt(v_da, 27) and not f_lt(v_db, 27));
                        when others => v_pearl := q64;
                    end case;
                    if v_pearl then v_reg := "11"; v_ht := "11"; v_lsel := "01"; v_lsh := '1'; end if;
                    if v_var = 3 then v_spc := f_b2sl(v_pearl and l7 and q16); end if;
                when 20 =>                                                   -- ROSETTE
                    v_ins := q960;
                    if v_ins then v_reg := "01"; v_ht := "10"; end if;
                    v_lsel := "11";
                    case v_var is
                        when 0 => v_r4 := (not f5_oct(4 downto 2)) & f5_oct(1);
                        when 1 => v_r4 := (not f5_oct(4 downto 2)) & f_b2sl(v_w);
                        when 2 =>
                            v_r4 := (not f5_oct(4 downto 2)) & '0';
                            if v_ins and f5_oct(2) = '0' then v_reg := "10"; end if;
                        when others => v_lsel := "01"; v_spc := f_b2sl(q16);
                    end case;
                when 21 =>                                                   -- HOLES
                    case v_var is
                        when 0 =>
                            case f5_hs is
                                when "00"   => v_hole := q256;
                                when "01"   => v_hole := q128;
                                when "10"   => v_hole := q64;
                                when others => v_hole := false;
                            end case;
                        when 1 => v_hole := q64;
                        when 2 => if f5_hs(0) = '1' then v_hole := q128; else v_hole := q64; end if;
                        when others => v_hole := q256;
                    end case;
                    v_lsh := '1';
                    if v_hole then v_reg := "10"; v_lsel := "01"; v_linv := '1'; else v_reg := "01"; v_ht := "10"; end if;
                when others =>                                               -- FULL (plain surface)
                    v_reg := "01";
                    case v_var is
                        when 1 => v_lsel := "11"; v_r4 := f_tri4(f5_ch(4 downto 0));
                        when 2 => v_lsel := "01";
                        when 3 => v_lsel := "11"; v_t4 := f_tri4(f5_sm(5 downto 1)); v_r4 := "10" & v_t4(3 downto 2);
                        when others => null;
                    end case;
            end case;
            -- the sprinkle overlay lies on top of everything
            if s5_on = '1' then v_ht := "11"; end if;
            f6_reg <= v_reg; f6_k <= v_k; f6_ht <= v_ht; f6_lsel <= v_lsel; f6_lsh <= v_lsh; f6_limb <= v_limb;
            f6_spc <= v_spc; f6_r4 <= v_r4; f6_linv <= v_linv;
            f6_lit <= f5_lit;
            if f5_na = '1' then f6_lita <= signed('0' & f5_da); else f6_lita <= not signed('0' & f5_da); end if;

            ----------------------------------------------------------
            -- C7 (H 34): shade.  Light term by the shape's select, the candy's
            -- shading model (0 matte, 1 soft, 2 glossy, 3 jelly), limb and grain
            -- as one constant; grain and sprinkles pick the colour; relief
            ----------------------------------------------------------
            case f6_lsel is
                when "01"   => v_L := resize(f6_lit, 8);
                when "10"   => v_L := resize(f6_lita & '0', 8);
                when "11"   => v_L := shift_left(resize(signed(f6_r4 xor "1000"), 8), 3);
                when others => v_L := (others => '0');
            end case;
            if f6_lsh = '1' and f6_lsel /= "11" then v_L := shift_left(v_L, 1); end if;
            if f6_linv = '1' then v_L := not v_L; end if;
            if f6_reg = "00" then v_L := (others => '0'); end if;
            case r34_w1(1 downto 0) is
                when "00"   => v_L := shift_right(v_L, 1);
                when "11"   => if v_L(7) = '1' then v_L := not v_L; end if;
                when others => null;
            end case;
            v_piece := f6_reg /= "00";
            v_gz := v_piece = (r34_w1(4) = '1');
            v_gd := v_gz and gr(6)(0) = '1'; v_gl := v_gz and gr(6)(1) = '1'; v_gn := v_gz and gr(6)(2) = '1';
            v_cv := f6_limb & f_b2sl(r34_w1(1 downto 0) = "11")
                    & f_b2sl(r34_w1(3 downto 2) = "01" and v_gl and not v_gd) & f_b2sl(r34_w1(3 downto 2) = "10" and v_gl);
            case v_cv is                                                     -- limb -32, jelly -24, crumb light +40, sugar +72
                when "0000" => v_ci := 0;    when "0001" => v_ci := 72;   when "0010" => v_ci := 40;   when "0011" => v_ci := 112;
                when "0100" => v_ci := -24;  when "0101" => v_ci := 48;   when "0110" => v_ci := 16;   when "0111" => v_ci := 88;
                when "1000" => v_ci := -32;  when "1001" => v_ci := 40;   when "1010" => v_ci := 8;    when "1011" => v_ci := 80;
                when "1100" => v_ci := -56;  when "1101" => v_ci := 16;   when "1110" => v_ci := -16;  when others => v_ci := 56;
            end case;
            v_sv := resize(v_L, 9) + to_signed(v_ci, 9);
            v_sel := '0' & f6_reg; v_k := f6_k;
            if r34_w1(3 downto 2) = "01" and v_gd then v_sel := "010"; end if;              -- crumbs take colour C
            if r34_w1(3 downto 2) = "11" and v_gn then v_sel := "100"; v_k := unsigned(gr(6)(5 downto 3)); end if;   -- nonpareils
            v_spec := (f6_spc = '1' and r34_w1(1) = '1') or (r34_w1(3 downto 2) = "10" and v_gl and v_gd);
            if s6_on = '1' then
                v_sel := "100"; v_k := s6_k; v_sv := to_signed(16, 9); v_spec := false;
            end if;
            if v_spec and c_gloss(3 downto 1) /= "000" then v_sel := "011"; v_sv := (others => '0'); end if;
            f7_sel <= v_sel; f7_k <= v_k; f7_sv <= v_sv;
            f7_ca <= unsigned(cw34(5 downto 0)); f7_cb <= unsigned(cw34(11 downto 6)); f7_asst <= cw34(13 downto 12);
            f7_cc <= unsigned(cx34);
            -- relief = height against the sample 2..3 pixels left and two lines up
            -- (one field line when interlaced), -3..3 -> x21
            if c_il = '1' then v_ref := unsigned(rl_d2(3 downto 2)); else v_ref := unsigned(rl_d2(1 downto 0)); end if;
            v_r3 := signed(resize(f6_ht, 3)) - signed(resize(v_ref, 3));
            if px(10 downto 4) = 0 then
                f7_rel <= (others => '0');                                     -- the line's first pixels have no sample yet
            else
                case v_r3 is
                    when "001"  => f7_rel <= to_signed(21, 7);
                    when "010"  => f7_rel <= to_signed(42, 7);
                    when "011"  => f7_rel <= to_signed(63, 7);
                    when "111"  => f7_rel <= to_signed(-21, 7);
                    when "110"  => f7_rel <= to_signed(-42, 7);
                    when "101"  => f7_rel <= to_signed(-63, 7);
                    when others => f7_rel <= (others => '0');
                end case;
            end if;

            ----------------------------------------------------------
            -- C8: palette index (A / B with the assortment bits / C / white /
            -- a bright by index); e = shade + 2 x relief
            ----------------------------------------------------------
            case f7_asst is
                when "01"   => v_m := "000001";
                when "10"   => v_m := "000011";
                when "11"   => v_m := "000111";
                when others => v_m := "000000";
            end case;
            case f7_sel is
                when "000"  => v_idx := f7_ca;
                when "001"  => v_idx := (f7_cb and not v_m) or (("000" & f7_k) and v_m);
                when "010"  => v_idx := f7_cc;
                when "011"  => v_idx := "111111";
                when others => v_idx := "000" & f7_k;
            end case;
            f8_idx <= v_idx;
            f8_e <= resize(f7_sv, 10) + resize(f7_rel & '0', 10);

            ----------------------------------------------------------
            -- C9..C12: e x gloss / 8 (one add of gated operands per stage) on the
            -- palette luma, + the picture's shade
            ----------------------------------------------------------
            v_q := (others => '0');
            if c_gloss(0) = '1' then v_q := resize(f8_e, 14); end if;
            if c_gloss(1) = '1' then f9_q1 <= resize(v_q + shift_left(resize(f8_e, 14), 1), 12); else f9_q1 <= resize(v_q, 12); end if;
            v_q := (others => '0');
            if c_gloss(2) = '1' then v_q := shift_left(resize(f8_e, 14), 2); end if;
            if c_gloss(3) = '1' then f9_q2 <= v_q + shift_left(resize(f8_e, 14), 3); else f9_q2 <= v_q; end if;
            f10_ey <= resize(f9_q1, 14) + f9_q2;
            f10_y6 <= unsigned(pal_q(15 downto 10)); f10_u5 <= unsigned(pal_q(9 downto 5)); f10_v5 <= unsigned(pal_q(4 downto 0));
            f11_yb <= signed(resize(f10_y6 & "0000", 12)) + resize(shift_right(f10_ey, 3), 12);
            f11_u5 <= f10_u5; f11_v5 <= f10_v5;
            f11_ps <= ps36;
            f12_ys <= resize(f11_yb, 13) + resize(f11_ps, 13);
            f12_u5 <= f11_u5; f12_v5 <= f11_v5;

            ----------------------------------------------------------
            -- SPRINKLE OVERLAY (straight axes, 64-position cells): one rod per
            -- cell at a random place, one of four directions and eight colours,
            -- present when the cell's hash byte is below the density
            --   S1 (-12) hash   S2 (-11) jitter, folds   S3 (-10) sums   S4 (-9) rod, presence
            ----------------------------------------------------------
            v_key := std_logic_vector(g_y(8 downto 6)) & std_logic_vector(g_x(8 downto 6)) & std_logic_vector(g_y(10 downto 6)) & std_logic_vector(g_x(10 downto 6));
            s1_h <= f_spn(v_key, x"1D2E", x"B74A", x"52F1", x"E08D");
            s1_pa <= g_x(5 downto 0); s1_pb <= g_y(5 downto 0);
            v_pa := s1_pa + unsigned(resize(signed(s1_h(6 downto 3)), 6));
            v_pb := s1_pb + unsigned(resize(signed(s1_h(10 downto 7)), 6));
            s2_da <= f_fold(v_pa); s2_db <= f_fold(v_pb); s2_same <= not (v_pa(5) xor v_pb(5));
            s2_o <= s1_h(12 downto 11); s2_p <= unsigned(s1_h(15 downto 8)); s2_k <= unsigned(s1_h(2 downto 0));
            v_l1 := resize(s2_da, 6) + resize(s2_db, 6);
            v_dd := signed(resize(s2_da, 6)) - signed(resize(s2_db, 6));
            if v_dd(5) = '1' then v_ad := not v_dd; else v_ad := v_dd; end if;
            if s2_same = '1' then s3_big <= v_l1; s3_small <= unsigned(v_ad);
            else s3_big <= unsigned(v_ad); s3_small <= v_l1; end if;
            s3_da <= s2_da; s3_db <= s2_db; s3_o <= s2_o; s3_p <= s2_p; s3_k <= s2_k;
            case s3_o is
                when "00"   => v_rod := f_lt(s3_da, 12) and f_lt(s3_db, 3);
                when "01"   => v_rod := f_lt(s3_db, 12) and f_lt(s3_da, 3);
                when "10"   => v_rod := f_lt(s3_small, 4) and f_lt(s3_big, 17);
                when others => v_rod := f_lt(s3_big, 4) and f_lt(s3_small, 17);
            end case;
            if f3_sprk = '1' then v_dens := c_sprkf; else v_dens := c_sprk; end if;
            s4_on <= f_b2sl(v_rod and s3_p < v_dens); s4_k <= s3_k;
            s5_on <= s4_on; s5_k <= s4_k; s6_on <= s5_on; s6_k <= s5_k;

            ----------------------------------------------------------
            -- GRAIN (straight axes, 4-position micro cells, a 3 x 3 dot in each):
            -- dark 1/8, light 1/16, nonpareil 1/4 with a colour index
            ----------------------------------------------------------
            g1_h <= f_spn(std_logic_vector(g_y(9 downto 2)) & std_logic_vector(g_x(9 downto 2)), x"7C15", x"29E3", x"C6B8", x"914F");
            g1_dot <= f_b2sl(g_x(1 downto 0) /= "00" and g_y(1 downto 0) /= "00");
            gr(2) <= g1_h(11 downto 9) & (g1_dot and f_b2sl(g1_h(8 downto 7) = "00"))
                     & (g1_dot and f_b2sl(g1_h(6 downto 3) = "0000")) & (g1_dot and f_b2sl(g1_h(2 downto 0) = "000"));
            for i in 3 to 6 loop gr(i) <= gr(i - 1); end loop;
        end if;
    end process p_fill;

    ------------------------------------------------------------------------
    -- F composite.  O1 holds pixel x during cycle t_in + x + 79.
    --   O1 fill colour (the C12 candy values), line alpha
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
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------
            -- O1: the candy's luma (palette + shading + the picture's shade), clamped;
            -- palette chroma
            ----------------------------------------------------------
            if f_slt(f12_ys, 64) then o1_y <= to_unsigned(64, 10);
            elsif not f_slt(f12_ys, 941) then o1_y <= to_unsigned(940, 10);
            else o1_y <= unsigned(f12_ys(9 downto 0)); end if;
            o1_u <= f12_u5 & "00000"; o1_v <= f12_v5 & "00000";
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

end architecture sugarcoat;
