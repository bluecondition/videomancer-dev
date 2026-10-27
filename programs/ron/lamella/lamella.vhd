-- Copyright (C) 2026 bluecondition
-- SPDX-License-Identifier: GPL-3.0-only
--
-- lamella.vhd  (v0.1.0 -- "Lamella": the picture re-rendered as a wall of
-- glossy metal tiles)
--
-- The incoming frame is averaged into a coarse store (64x64 luma, 8 bit;
-- 32x32 chroma U4V4; two banks ping-ponged by field parity) and every output
-- pixel is redrawn from it: the screen is cut into leaning columns of
-- rounded tiles, each tile a small convex lens that magnifies the image in
-- its middle and crushes ~1.4 tiles of it into its rim.  Adjacent columns
-- are staggered (a stair), so the picture steps diagonally across the wall.
-- A thin rim highlight and a dark groove ring every tile, the tone is pushed
-- through a metal palette (copper / lime / steel, ice, brass, chrome) and the
-- dark tiles carry a brushed grain.  Bilinear readback keeps the lenses
-- smooth at full resolution.
--
-- Controls:
--   K1 Columns  3 .. 16 across            K2 Rows   2 .. 12 tiles high
--   K3 Stair    bipolar stagger per column (up to half a tile)
--   K4 Lean     bipolar column slant (up to one column over the height)
--   K5 Gloss    rim highlight / groove strength
--   K6 Palette  Copper / Ice / Brass / Chrome
--   S7 Colour   Palette / Source     S8 Shape    Convex / Concave
--   S9 Shuffle  random per-column stagger on top of the stair
--   S10 Brushed grain in the dark tiles   S11 Negative
--   P12 Depth   lens strength, flat .. 2x (rim crush and centre zoom)
--
-- Store: luma sub-bank (c(0), r(0)) holds {bank, r>>1, c>>1} (4 x 2048 x 8),
-- chroma sub-bank (cc(0), rc(0)) holds {bank, rc>>1, cc>>1} (4 x 512 x 8);
-- any 2x2 neighbourhood is one cell per sub-bank, so one read per sub-bank
-- per clock gives the four bilinear taps.  Reads wrap (modulo 64 cells).
-- 23 EBR (20 store + 1 profile ROM + 2 palette ROM).
--
-- Latency: C_LAT clocks data_in -> data_out, identical in every mode; the
-- output is drawn from the store only (last field), so no live video is
-- carried through the pipe.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture lamella of program_top is
    constant C_LAT  : natural := 40;        -- 38 pixel stages + 2 pad
    constant C_SYNC : natural := C_LAT - 1; -- sync taps sy(0 .. C_SYNC-1)

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- read origin: -0.5 cell (bilinear centring) + 1/32 cell (weight floor), Q16
    constant C_XC0  : signed(21 downto 0) := to_signed(-30720, 22);

    -- tile profile, indexed by the rounded distance from the tile centre
    -- (0 .. 255 over half a tile): delta(15..6) = read displacement minus
    -- the distance, signed Q8 tiles (negative = magnify, positive = crush);
    -- highlight(5..0) signed (bright line inside the rim, dark groove past it)
    type t_rom16 is array (natural range <>) of std_logic_vector(15 downto 0);
    constant PROM : t_rom16(0 to 255) := (
        x"0000", x"0000", x"0000", x"0000", x"0000", x"FFC0", x"FFC0", x"FFC0",
        x"FFC0", x"FFC0", x"FFC0", x"FFC0", x"FFC0", x"FFC0", x"FFC0", x"FF80",
        x"FF80", x"FF80", x"FF80", x"FF80", x"FF80", x"FF80", x"FF80", x"FF80",
        x"FF80", x"FF40", x"FF40", x"FF40", x"FF40", x"FF40", x"FF40", x"FF40",
        x"FF40", x"FF40", x"FF40", x"FF00", x"FF00", x"FF00", x"FF00", x"FF00",
        x"FF00", x"FF00", x"FF00", x"FF00", x"FF00", x"FEC0", x"FEC0", x"FEC0",
        x"FEC0", x"FEC0", x"FEC0", x"FEC0", x"FEC0", x"FEC0", x"FEC0", x"FE80",
        x"FE80", x"FE80", x"FE80", x"FE80", x"FE80", x"FE80", x"FE80", x"FE80",
        x"FE80", x"FE40", x"FE40", x"FE40", x"FE40", x"FE40", x"FE40", x"FE40",
        x"FE40", x"FE40", x"FE40", x"FE40", x"FE00", x"FE00", x"FE00", x"FE00",
        x"FE00", x"FE00", x"FE00", x"FE00", x"FE00", x"FE00", x"FE00", x"FDC0",
        x"FDC0", x"FDC0", x"FDC0", x"FDC0", x"FDC0", x"FDC0", x"FDC0", x"FDC0",
        x"FDC0", x"FDC0", x"FDC0", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80",
        x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80",
        x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80",
        x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80",
        x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80", x"FD80",
        x"FD80", x"FD80", x"FD80", x"FD80", x"FDC0", x"FDC0", x"FDC0", x"FDC0",
        x"FDC0", x"FDC0", x"FE00", x"FE00", x"FE00", x"FE00", x"FE00", x"FE40",
        x"FE40", x"FE40", x"FE80", x"FE80", x"FE80", x"FE80", x"FEC0", x"FEC0",
        x"FF00", x"FF00", x"FF40", x"FF40", x"FF40", x"FF80", x"FF80", x"FFC0",
        x"0000", x"0000", x"0040", x"0040", x"0080", x"00C0", x"0100", x"0100",
        x"0140", x"0180", x"01C0", x"0200", x"0240", x"0280", x"0280", x"0300",
        x"0340", x"0380", x"03C0", x"0400", x"0440", x"0480", x"0500", x"0540",
        x"05C0", x"0600", x"0640", x"06C0", x"0740", x"0780", x"0800", x"0880",
        x"08C0", x"0940", x"09C0", x"0A40", x"0AC0", x"0B40", x"0C00", x"0C80",
        x"0D00", x"0DC0", x"0E40", x"0F00", x"0F81", x"1041", x"1102", x"1183",
        x"1244", x"1305", x"1407", x"14C9", x"158C", x"164F", x"1752", x"1816",
        x"191A", x"1A1E", x"1B21", x"1C24", x"1D26", x"1E28", x"1F28", x"2067",
        x"2166", x"22A3", x"23E0", x"251D", x"2659", x"2795", x"28D2", x"2A4E",
        x"2B8B", x"2D09", x"2E86", x"3005", x"3183", x"337A", x"34F3", x"36AE",
        x"386A", x"3A26", x"3BE2", x"3D9E", x"3F9B", x"4158", x"4354", x"4551"
    );
    -- palettes: address = palette & tone(7 downto 1); U(15..8) V(7..0) signed
    -- in 4-code units, authored in the hardware's (swapped) chroma order
    constant PALROM : t_rom16(0 to 511) := (
        x"03FD", x"03FD", x"03FD", x"03FD", x"03FD", x"04FD", x"04FD", x"04FD",
        x"04FC", x"04FC", x"04FC", x"04FC", x"04FC", x"05FC", x"05FC", x"05FC",
        x"05FC", x"05FB", x"05FB", x"05FB", x"05FB", x"06FB", x"06FB", x"06FB",
        x"07FA", x"08F9", x"09F8", x"0AF7", x"0BF7", x"0CF6", x"0DF5", x"0EF4",
        x"0EF3", x"0FF3", x"10F2", x"11F1", x"12F0", x"13EF", x"14EF", x"15EE",
        x"16ED", x"17EC", x"18EB", x"1AEA", x"1CE8", x"1EE6", x"20E4", x"22E2",
        x"24E0", x"26DE", x"28DD", x"2ADB", x"2CD9", x"2ED7", x"2DD8", x"29DB",
        x"25DF", x"21E3", x"1DE6", x"19EA", x"15EE", x"11F1", x"0DF5", x"09F9",
        x"04FD", x"0000", x"FC04", x"FB05", x"FB05", x"FB04", x"FB04", x"FB04",
        x"FB04", x"FB04", x"FB03", x"FB03", x"FB03", x"FB03", x"FB03", x"FB02",
        x"FB02", x"FA02", x"FA02", x"FA02", x"FA01", x"FA01", x"FA01", x"FA01",
        x"FA01", x"FA00", x"FAFF", x"FAFD", x"FAFC", x"FAFA", x"FAF8", x"FAF6",
        x"FBF5", x"FBF3", x"FBF1", x"FBF0", x"FBEE", x"FBEC", x"FBEA", x"FBE9",
        x"FBE7", x"FBE5", x"FBE4", x"FBE2", x"FBE0", x"FBE0", x"FCE1", x"FCE1",
        x"FCE2", x"FCE3", x"FDE4", x"FDE4", x"FDE5", x"FDE6", x"FEE6", x"FEE7",
        x"FEE8", x"FEE8", x"FEE9", x"FFEA", x"FFEB", x"FFEB", x"FFEC", x"00ED",
        x"FE05", x"FE05", x"FD05", x"FD06", x"FC06", x"FC06", x"FB07", x"FB07",
        x"FA07", x"FA08", x"F908", x"F808", x"F809", x"F709", x"F70A", x"F60A",
        x"F60A", x"F50B", x"F50B", x"F40B", x"F40C", x"F30C", x"F20C", x"F20D",
        x"F10D", x"F10D", x"F10E", x"F10F", x"F20F", x"F210", x"F311", x"F312",
        x"F312", x"F413", x"F414", x"F515", x"F515", x"F616", x"F617", x"F718",
        x"F718", x"F819", x"F81A", x"F91B", x"F91B", x"F91C", x"FA1D", x"FA1E",
        x"FB1E", x"FB1F", x"FA1E", x"F91E", x"F81D", x"F71D", x"F61D", x"F51C",
        x"F41C", x"F31C", x"F21B", x"F21B", x"F11A", x"F01A", x"EF1A", x"EE19",
        x"ED19", x"EC19", x"EB18", x"EA18", x"E918", x"E917", x"E817", x"E816",
        x"E816", x"E816", x"E815", x"E815", x"E815", x"E914", x"E914", x"E913",
        x"E913", x"E913", x"E912", x"E912", x"EA12", x"EA11", x"EA11", x"EA10",
        x"EA10", x"EA10", x"EA0F", x"EB0F", x"EB0F", x"EB0E", x"EB0E", x"EB0D",
        x"EB0D", x"EC0D", x"EC0D", x"ED0C", x"EE0C", x"EE0C", x"EF0B", x"EF0B",
        x"F00B", x"F00A", x"F10A", x"F20A", x"F209", x"F309", x"F309", x"F409",
        x"F408", x"F508", x"F508", x"F607", x"F707", x"F707", x"F806", x"F806",
        x"F906", x"F906", x"FA05", x"FA05", x"FB05", x"FC04", x"FC04", x"FD04",
        x"03FC", x"04FB", x"04FA", x"05FA", x"05F9", x"06F9", x"06F8", x"06F7",
        x"07F7", x"07F6", x"08F5", x"08F5", x"09F4", x"09F4", x"0AF3", x"0AF2",
        x"0BF2", x"0BF1", x"0CF1", x"0CF0", x"0DEF", x"0DEF", x"0EEE", x"0EED",
        x"0EED", x"0FEC", x"0FEC", x"10EB", x"10EA", x"11EA", x"11E9", x"12E9",
        x"12E8", x"13E7", x"13E7", x"14E6", x"14E6", x"15E5", x"15E4", x"15E4",
        x"16E3", x"16E2", x"16E1", x"16E1", x"17E0", x"17DF", x"17DE", x"17DE",
        x"18DD", x"18DC", x"18DB", x"18DB", x"19DA", x"19D9", x"19D9", x"19D8",
        x"1AD7", x"1AD6", x"1AD6", x"1AD5", x"1BD4", x"1BD3", x"1BD3", x"1BD2",
        x"1CD1", x"1CD0", x"1CD0", x"1CCF", x"1DCE", x"1DCE", x"1DCD", x"1DCD",
        x"1CCD", x"1CCD", x"1CCE", x"1BCE", x"1BCE", x"1BCE", x"1ACF", x"1ACF",
        x"19CF", x"19CF", x"19D0", x"18D0", x"18D0", x"18D0", x"17D1", x"17D1",
        x"17D1", x"16D1", x"16D2", x"15D2", x"15D2", x"15D3", x"14D3", x"14D3",
        x"14D3", x"13D4", x"13D4", x"13D4", x"12D4", x"12D5", x"11D5", x"11D6",
        x"10D7", x"10D8", x"0FD9", x"0FDB", x"0EDC", x"0EDD", x"0DDE", x"0DDF",
        x"0CE0", x"0CE1", x"0BE3", x"0BE4", x"0AE5", x"0AE6", x"09E7", x"09E8",
        x"08E9", x"08EB", x"07EC", x"07ED", x"06EE", x"06EF", x"05F0", x"05F2",
        x"FF02", x"FF02", x"FF02", x"FF02", x"FF02", x"FF02", x"FE03", x"FE03",
        x"FE03", x"FE03", x"FE03", x"FE03", x"FE03", x"FE03", x"FE03", x"FE03",
        x"FE03", x"FE04", x"FE04", x"FE04", x"FE04", x"FE04", x"FD04", x"FD04",
        x"FD04", x"FD04", x"FD04", x"FD04", x"FD04", x"FD05", x"FD05", x"FD05",
        x"FD05", x"FD05", x"FD05", x"FD05", x"FD05", x"FD05", x"FC05", x"FC05",
        x"FC06", x"FC06", x"FC06", x"FC06", x"FC06", x"FC06", x"FC06", x"FC06",
        x"FC06", x"FC06", x"FC06", x"FC06", x"FC07", x"FC07", x"FC07", x"FB07",
        x"FB07", x"FB07", x"FB07", x"FB07", x"FB07", x"FB07", x"FB07", x"FB07",
        x"FB07", x"FB07", x"FB07", x"FB07", x"FB08", x"FB08", x"FB08", x"FB08",
        x"FB08", x"FB08", x"FB08", x"FB08", x"FB08", x"FB08", x"FB08", x"FB08",
        x"FB08", x"FB07", x"FB07", x"FB07", x"FB07", x"FB07", x"FB07", x"FB07",
        x"FB06", x"FB06", x"FC06", x"FC06", x"FC06", x"FC06", x"FC06", x"FC06",
        x"FC05", x"FC05", x"FC05", x"FC05", x"FC05", x"FC05", x"FC05", x"FC04",
        x"FD04", x"FD04", x"FD04", x"FD04", x"FD03", x"FD03", x"FD03", x"FE03",
        x"FE03", x"FE03", x"FE02", x"FE02", x"FE02", x"FE02", x"FF02", x"FF02",
        x"FF01", x"FF01", x"FF01", x"FF01", x"FF01", x"0000", x"0000", x"0000"
    );

    -- bipolar knob: |k - 512| minus a 16-step centre deadband (0..496)
    function bip_mag(k : unsigned(9 downto 0)) return unsigned is
        variable d : signed(10 downto 0);
        variable a : signed(10 downto 0);
    begin
        d := signed('0' & k) - to_signed(512, 11);
        if d < 0 then a := -d; else a := d; end if;
        if a > 16 then
            return resize(unsigned(a(9 downto 0)) - 16, 9);
        else
            return to_unsigned(0, 9);
        end if;
    end function;

    -- signed d times a 4-bit weight: AND-gated partials, balanced adds
    function smul4(d : signed; w : unsigned(3 downto 0); n : natural) return signed is
        variable dd, p0, p1, p2, p3 : signed(n - 1 downto 0);
    begin
        dd := resize(d, n);
        p0 := (others => '0'); p1 := (others => '0');
        p2 := (others => '0'); p3 := (others => '0');
        if w(0) = '1' then p0 := dd; end if;
        if w(1) = '1' then p1 := shift_left(dd, 1); end if;
        if w(2) = '1' then p2 := shift_left(dd, 2); end if;
        if w(3) = '1' then p3 := shift_left(dd, 3); end if;
        return (p0 + p1) + (p2 + p3);
    end function;

    -- signed d times a 5-bit coefficient (0..31)
    function smul5(d : signed; w : unsigned(4 downto 0); n : natural) return signed is
        variable dd, p0, p1, p2, p3, p4 : signed(n - 1 downto 0);
    begin
        dd := resize(d, n);
        p0 := (others => '0'); p1 := (others => '0'); p2 := (others => '0');
        p3 := (others => '0'); p4 := (others => '0');
        if w(0) = '1' then p0 := dd; end if;
        if w(1) = '1' then p1 := shift_left(dd, 1); end if;
        if w(2) = '1' then p2 := shift_left(dd, 2); end if;
        if w(3) = '1' then p3 := shift_left(dd, 3); end if;
        if w(4) = '1' then p4 := shift_left(dd, 4); end if;
        return ((p0 + p1) + (p2 + p3)) + p4;
    end function;

    function rotl16(v : unsigned(15 downto 0); n : natural) return unsigned is
    begin
        return v(15 - n downto 0) & v(15 downto 16 - n);
    end function;

    --------------------------------------------------------------------------
    -- Control decode (free-running).
    --------------------------------------------------------------------------
    signal k_ncol_a, k_ntile_a : unsigned(11 downto 0) := (others => '0');
    signal k_ncol  : unsigned(8 downto 0) := to_unsigned(112, 9);   -- Q4 columns
    signal k_ntile : unsigned(7 downto 0) := to_unsigned(64, 8);    -- Q4 tiles
    signal k_stair : unsigned(6 downto 0) := (others => '0');       -- Q8 tiles/column
    signal k_stair_neg : std_logic := '0';
    signal k_lean  : unsigned(7 downto 0) := (others => '0');       -- Q8 columns/height
    signal k_lean_neg : std_logic := '0';
    signal k_gloss : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal k_pal   : unsigned(1 downto 0) := (others => '0');
    signal k_depth : unsigned(8 downto 0) := to_unsigned(256, 9);   -- Q8, 0..2
    signal sw_src, sw_conc, sw_shuf, sw_brush, sw_neg : std_logic := '0';

    --------------------------------------------------------------------------
    -- Input raster, measurement, frame anchor.
    --------------------------------------------------------------------------
    type t_sync is record
        hsync_n, vsync_n, avid, field_n : std_logic;
    end record;
    type t_sync_pipe is array (natural range <>) of t_sync;
    signal sy : t_sync_pipe(0 to C_SYNC - 1) := (others => ('1', '1', '0', '1'));
    signal din1 : t_video_stream_yuv444_30b;

    signal prev_hsync_n, prev_vsync_n, prev_avid : std_logic := '1';
    signal px   : unsigned(11 downto 0) := (others => '0');
    signal py   : unsigned(10 downto 0) := (others => '0');
    signal line_act, frame_act, frame_armed : std_logic := '0';
    signal w_meas : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal h_meas : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal g_h    : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_k4   : std_logic := '1';       -- IIR shift 4 (wide) / 3 (narrow)
    signal par    : std_logic := '0';
    signal anchor : std_logic := '0';
    signal hpar, line_par : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame sequencer (vblank): serial multiplier + serial divider.
    --------------------------------------------------------------------------
    signal pc    : unsigned(3 downto 0) := (others => '0');
    signal sq_go : std_logic := '0';
    signal ma, mb : unsigned(15 downto 0) := (others => '0');
    signal macc  : unsigned(31 downto 0) := (others => '0');
    signal mcnt  : unsigned(4 downto 0) := (others => '0');
    signal dnum  : unsigned(22 downto 0) := (others => '0');
    signal dden  : unsigned(11 downto 0) := (others => '0');
    signal drem  : unsigned(12 downto 0) := (others => '0');
    signal dq    : unsigned(15 downto 0) := (others => '0');
    signal dcnt  : unsigned(4 downto 0) := (others => '0');

    signal kxc, kyc, cpc, cpt : unsigned(15 downto 0) := (others => '0');
    signal commit : std_logic := '0';
    signal c_dcx, c_dry : unsigned(15 downto 0) := (others => '0');
    signal c_dcy    : signed(11 downto 0) := (others => '0');
    signal c_stairt : signed(8 downto 0) := (others => '0');    -- Q8 tiles/column
    signal c_stairc : signed(11 downto 0) := (others => '0');   -- Q6 cells/column
    signal c_sv, c_sh : unsigned(7 downto 0) := (others => '0'); -- Q3 cells/tile, cells/column

    --------------------------------------------------------------------------
    -- Pixel pipeline (stage k = clocks after data_in).
    --------------------------------------------------------------------------
    signal ca, ca_line : signed(23 downto 0) := (others => '0');   -- Q8.16 columns
    signal xc : signed(21 downto 0) := C_XC0;                      -- Q6.16 cells
    signal ra_line : unsigned(21 downto 0) := (others => '0');     -- Q6.16 tiles
    signal yc_line : signed(21 downto 0) := C_XC0;

    signal s02_c8  : signed(7 downto 0) := (others => '0');
    signal s02_lx8 : unsigned(7 downto 0) := (others => '0');
    signal s02_xc, s02_yc : unsigned(15 downto 0) := (others => '0');   -- Q6.10
    signal s02_ra  : unsigned(21 downto 0) := (others => '0');

    signal s03_stt : signed(16 downto 0) := (others => '0');
    signal s03_stc : signed(19 downto 0) := (others => '0');
    signal s03_stag : unsigned(7 downto 0) := (others => '0');
    signal s03_lx8 : unsigned(7 downto 0) := (others => '0');
    signal s03_xc, s03_yc : unsigned(15 downto 0) := (others => '0');
    signal s03_ra  : unsigned(21 downto 0) := (others => '0');
    signal s03_c8  : signed(7 downto 0) := (others => '0');

    signal s04_tp  : unsigned(21 downto 0) := (others => '0');
    signal s04_ycs : unsigned(15 downto 0) := (others => '0');
    signal s04_stag : unsigned(7 downto 0) := (others => '0');
    signal s04_lx8 : unsigned(7 downto 0) := (others => '0');
    signal s04_xc  : unsigned(15 downto 0) := (others => '0');
    signal s04_c8  : signed(7 downto 0) := (others => '0');

    signal s05_tp  : unsigned(21 downto 0) := (others => '0');
    signal s05_ycs, s05_xc : unsigned(15 downto 0) := (others => '0');
    signal s05_lx8 : unsigned(7 downto 0) := (others => '0');
    signal s05_c8  : signed(7 downto 0) := (others => '0');

    signal s06_t6  : unsigned(5 downto 0) := (others => '0');
    signal s06_ly8 : unsigned(7 downto 0) := (others => '0');
    signal s06_c8  : unsigned(7 downto 0) := (others => '0');
    signal s06_d8, s06_e8 : unsigned(7 downto 0) := (others => '0');
    signal s06_sgn, s06_hsgn : std_logic := '0';
    signal s06_ycs, s06_xc : unsigned(15 downto 0) := (others => '0');

    signal s07_eex : unsigned(4 downto 0) := (others => '0');
    signal s07_elin : unsigned(7 downto 0) := (others => '0');
    signal s07_d8  : unsigned(7 downto 0) := (others => '0');
    signal s07_sgn, s07_hsgn : std_logic := '0';
    signal s07_ycs, s07_xc : unsigned(15 downto 0) := (others => '0');
    signal s07_ha1, s07_ha2, s07_hb1, s07_hb2, s07_hc1, s07_hc2 : unsigned(15 downto 0) := (others => '0');

    signal s08_eex2 : unsigned(9 downto 0) := (others => '0');
    signal s08_elin, s08_d8 : unsigned(7 downto 0) := (others => '0');
    signal s08_sgn, s08_hsgn : std_logic := '0';
    signal s08_ycs, s08_xc : unsigned(15 downto 0) := (others => '0');
    signal s08_ha, s08_hb, s08_hc : unsigned(15 downto 0) := (others => '0');

    signal s09_d8c, s09_hd : unsigned(7 downto 0) := (others => '0');
    signal s09_sgn, s09_hsgn : std_logic := '0';
    signal s09_ycs, s09_xc : unsigned(15 downto 0) := (others => '0');
    signal s09_ha, s09_hbc : unsigned(15 downto 0) := (others => '0');

    signal s10_prom : std_logic_vector(15 downto 0) := (others => '0');
    signal s10_hd  : unsigned(7 downto 0) := (others => '0');
    signal s10_sgn, s10_hsgn : std_logic := '0';
    signal s10_ycs, s10_xc : unsigned(15 downto 0) := (others => '0');
    signal s10_h   : unsigned(15 downto 0) := (others => '0');

    signal s11_prom : std_logic_vector(15 downto 0) := (others => '0');
    signal s11_hd  : unsigned(7 downto 0) := (others => '0');
    signal s11_sgn, s11_hsgn : std_logic := '0';
    signal s11_ycs, s11_xc : unsigned(15 downto 0) := (others => '0');
    signal s11_h   : unsigned(15 downto 0) := (others => '0');

    signal s12_dvh, s12_dvl : signed(14 downto 0) := (others => '0');
    signal s12_duh, s12_dul : unsigned(11 downto 0) := (others => '0');
    signal s12_hl  : signed(5 downto 0) := (others => '0');
    signal s12_sgn, s12_hsgn : std_logic := '0';
    signal s12_ycs, s12_xc : unsigned(15 downto 0) := (others => '0');
    signal s12_h   : unsigned(15 downto 0) := (others => '0');

    signal s13_dv  : signed(17 downto 0) := (others => '0');
    signal s13_du  : unsigned(15 downto 0) := (others => '0');
    signal s13_sgn, s13_hsgn : std_logic := '0';
    signal s13_ycs, s13_xc : unsigned(15 downto 0) := (others => '0');
    signal s13_n4  : signed(3 downto 0) := (others => '0');

    signal s14_dvs, s14_dus : signed(16 downto 0) := (others => '0');
    signal s14_ycs, s14_xc : unsigned(15 downto 0) := (others => '0');

    signal s15_v, s15_u : unsigned(15 downto 0) := (others => '0');
    signal s16_v, s16_u, s16_vc, s16_uc : unsigned(15 downto 0) := (others => '0');

    -- highlight + rim-side + grain: delay lines (k=12/13 -> k=31/26)
    type t_hl_pipe is array (natural range <>) of signed(5 downto 0);
    type t_b_pipe  is array (natural range <>) of std_logic;
    type t_n_pipe  is array (natural range <>) of signed(3 downto 0);
    signal hl_d  : t_hl_pipe(13 to 30) := (others => (others => '0'));
    signal sgn_d : t_b_pipe(13 to 30) := (others => '0');
    signal n_d   : t_n_pipe(14 to 26) := (others => (others => '0'));

    -- k=17 addresses, k=18 EBR, k=19.. bilinear
    signal ya00, ya10, ya01, ya11 : unsigned(10 downto 0) := (others => '0');
    signal ca00, ca10, ca01, ca11 : unsigned(8 downto 0) := (others => '0');
    -- side band: 17 Lx 16 Ty 15..12 fu 11..8 fv 7 cLx 6 cTy 5..2 fuc 1..0 (fvc hi)
    signal sb17, sb18 : std_logic_vector(19 downto 0) := (others => '0');

    type t_yram is array (0 to 2047) of std_logic_vector(7 downto 0);
    type t_cram is array (0 to 511) of std_logic_vector(7 downto 0);
    signal yr00, yr10, yr01, yr11 : t_yram := (others => (others => '0'));
    signal cr00, cr10, cr01, cr11 : t_cram := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of yr00 : signal is true;
    attribute no_rw_check of yr10 : signal is true;
    attribute no_rw_check of yr01 : signal is true;
    attribute no_rw_check of yr11 : signal is true;
    attribute no_rw_check of cr00 : signal is true;
    attribute no_rw_check of cr10 : signal is true;
    attribute no_rw_check of cr01 : signal is true;
    attribute no_rw_check of cr11 : signal is true;
    signal q00, q10, q01, q11 : std_logic_vector(7 downto 0) := (others => '0');
    signal e00, e10, e01, e11 : std_logic_vector(7 downto 0) := (others => '0');

    signal s19_ytl, s19_ytr, s19_ybl, s19_ybr : signed(8 downto 0) := (others => '0');
    signal s19_utl, s19_utr, s19_ubl, s19_ubr : signed(4 downto 0) := (others => '0');
    signal s19_vtl, s19_vtr, s19_vbl, s19_vbr : signed(4 downto 0) := (others => '0');
    signal s19_wx, s19_wy, s19_wxc, s19_wyc : unsigned(3 downto 0) := (others => '0');

    signal s20_ytl, s20_ybl, s20_dyt, s20_dyb : signed(8 downto 0) := (others => '0');
    signal s20_utl, s20_ubl, s20_vtl, s20_vbl : signed(4 downto 0) := (others => '0');
    signal s20_dut, s20_dub, s20_dvt, s20_dvb : signed(5 downto 0) := (others => '0');
    signal s20_wx, s20_wy, s20_wxc, s20_wyc : unsigned(3 downto 0) := (others => '0');

    signal s21_ytl, s21_ybl : signed(8 downto 0) := (others => '0');
    signal s21_pyt, s21_pyb : signed(12 downto 0) := (others => '0');
    signal s21_utl, s21_ubl, s21_vtl, s21_vbl : signed(4 downto 0) := (others => '0');
    signal s21_put, s21_pub, s21_pvt, s21_pvb : signed(9 downto 0) := (others => '0');
    signal s21_wy, s21_wyc : unsigned(3 downto 0) := (others => '0');

    signal s22_yt : signed(12 downto 0) := (others => '0');
    signal s22_dy : signed(13 downto 0) := (others => '0');
    signal s22_ut, s22_vt : signed(9 downto 0) := (others => '0');
    signal s22_du, s22_dv : signed(10 downto 0) := (others => '0');
    signal s22_wy, s22_wyc : unsigned(3 downto 0) := (others => '0');

    signal s23_yt : signed(12 downto 0) := (others => '0');
    signal s23_py : signed(17 downto 0) := (others => '0');
    signal s23_ut, s23_vt : signed(9 downto 0) := (others => '0');
    signal s23_pu, s23_pv : signed(14 downto 0) := (others => '0');

    signal s24_y  : signed(16 downto 0) := (others => '0');
    signal s24_u, s24_v : signed(12 downto 0) := (others => '0');

    signal s25_y8 : unsigned(7 downto 0) := (others => '0');
    signal s25_cu, s25_cv : unsigned(9 downto 0) := C_MID;
    signal s26_y8 : unsigned(7 downto 0) := (others => '0');
    signal s26_amp : unsigned(4 downto 0) := (others => '0');
    signal s26_cu, s26_cv : unsigned(9 downto 0) := C_MID;
    signal s27_y8 : unsigned(7 downto 0) := (others => '0');
    signal s27_np : signed(8 downto 0) := (others => '0');
    signal s27_cu, s27_cv : unsigned(9 downto 0) := C_MID;
    signal s28_y8 : unsigned(7 downto 0) := (others => '0');
    signal s28_cu, s28_cv : unsigned(9 downto 0) := C_MID;
    signal s29_y : unsigned(9 downto 0) := (others => '0');
    signal s29_cu, s29_cv : unsigned(9 downto 0) := C_MID;
    signal s30_y : unsigned(7 downto 0) := (others => '0');
    signal s30_cu, s30_cv : unsigned(9 downto 0) := C_MID;
    signal s31_pa : unsigned(8 downto 0) := (others => '0');
    signal s31_y3 : unsigned(9 downto 0) := (others => '0');
    signal s31_hlp : signed(11 downto 0) := (others => '0');
    signal s31_sgn : std_logic := '0';
    signal s31_cu, s31_cv : unsigned(9 downto 0) := C_MID;
    signal s32_pal : std_logic_vector(15 downto 0) := (others => '0');
    signal s32_yb : unsigned(9 downto 0) := C_BLKY;
    signal s32_hlh : signed(11 downto 0) := (others => '0');
    signal s32_cu, s32_cv : unsigned(9 downto 0) := C_MID;
    signal s33_pal : std_logic_vector(15 downto 0) := (others => '0');
    signal s33_yb : unsigned(9 downto 0) := C_BLKY;
    signal s33_hl2 : signed(9 downto 0) := (others => '0');
    signal s33_cu, s33_cv : unsigned(9 downto 0) := C_MID;
    signal s34_y : signed(11 downto 0) := (others => '0');
    signal s34_u, s34_v : unsigned(9 downto 0) := C_MID;
    signal s35_y, s35_u, s35_v : unsigned(9 downto 0) := C_MID;
    signal x5_y, x5_u, x5_v : unsigned(9 downto 0) := C_BLKY;
    signal x5_bl : std_logic := '1';
    signal d6_y, d6_u, d6_v, d7_y, d7_u, d7_v : unsigned(9 downto 0) := C_BLKY;
    signal d6_bl : std_logic := '1';
    signal s_out : t_video_stream_yuv444_30b;
    type t_pad is array (0 to 1) of t_video_stream_yuv444_30b;
    signal s_pad : t_pad;

    --------------------------------------------------------------------------
    -- Write side: cell DDA + IIR on the input (k=1), pulse pipelines.
    --------------------------------------------------------------------------
    signal w_prev : std_logic := '0';
    signal w_x    : unsigned(12 downto 0) := (others => '0');
    signal w_c    : unsigned(5 downto 0) := (others => '0');
    signal w_yi, w_ui, w_vi : unsigned(13 downto 0) := (others => '0');
    signal w_ey, w_eu, w_ev : signed(14 downto 0) := (others => '0');
    signal w_yacc : unsigned(11 downto 0) := (others => '0');
    signal w_r    : unsigned(5 downto 0) := (others => '0');
    signal w_lsn, w_csn : std_logic := '0';
    signal w_lsl, w_csl, w_par : std_logic := '0';
    signal w_rl   : unsigned(5 downto 0) := (others => '0');

    signal l1_vy, l1_vc : std_logic := '0';
    signal l1_c : unsigned(5 downto 0) := (others => '0');
    signal l1_r : unsigned(5 downto 0) := (others => '0');
    signal l1_y : unsigned(7 downto 0) := (others => '0');
    signal l1_u, l1_v : signed(8 downto 0) := (others => '0');
    signal wy_v, wy_px, wy_py : std_logic := '0';
    signal wy_a : unsigned(10 downto 0) := (others => '0');
    signal wy_d : std_logic_vector(7 downto 0) := (others => '0');
    signal wc_v, wc_px, wc_py : std_logic := '0';
    signal wc_a : unsigned(8 downto 0) := (others => '0');
    signal wc_d : std_logic_vector(7 downto 0) := (others => '0');

begin

    --------------------------------------------------------------------------
    -- Free-running control decode (geometry knobs use their top 8 bits so
    -- pot noise never jitters the column grid).
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable rk1, rk2, rk3, rk4, rk5, rk6, rp12 : unsigned(9 downto 0);
        variable rsw : std_logic_vector(4 downto 0);
        variable k8  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            rk1  := unsigned(registers_in(0)(9 downto 0));
            rk2  := unsigned(registers_in(1)(9 downto 0));
            rk3  := unsigned(registers_in(2)(9 downto 0));
            rk4  := unsigned(registers_in(3)(9 downto 0));
            rk5  := unsigned(registers_in(4)(9 downto 0));
            rk6  := unsigned(registers_in(5)(9 downto 0));
            rsw  := registers_in(6)(4 downto 0);
            rp12 := unsigned(registers_in(7)(9 downto 0));

            -- K1: columns 3.0 .. 15.9 (Q4) = 48 + k8*13/16
            k8 := rk1(9 downto 2);
            k_ncol_a <= resize(k8 & "000", 12) + resize(k8 & "00", 12) + resize(k8, 12);
            k_ncol   <= resize(k_ncol_a(11 downto 4), 9) + 48;
            -- K2: rows 2.0 .. 11.9 (Q4) = 32 + k8*10/16
            k8 := rk2(9 downto 2);
            k_ntile_a <= resize(k8 & "000", 12) + resize(k8 & '0', 12);
            k_ntile   <= resize(k_ntile_a(11 downto 4), 8) + 32;
            -- K3 stair: 0 .. 124 (Q8 tiles per column)   K4 lean: 0 .. 248
            k_stair <= resize(bip_mag(rk3)(8 downto 2), 7); k_stair_neg <= not rk3(9);
            k_lean  <= resize(bip_mag(rk4)(8 downto 1), 8); k_lean_neg  <= not rk4(9);
            k_gloss <= rk5(9 downto 5);
            -- K6 Palette (labelled: firmware may load the label INDEX 0..3)
            if rk6 < 4 then k_pal <= rk6(1 downto 0); else k_pal <= rk6(9 downto 8); end if;
            k_depth <= rp12(9 downto 1);

            sw_src   <= rsw(0);   -- S7  Colour Palette / Source
            sw_conc  <= rsw(1);   -- S8  Shape Convex / Concave
            sw_shuf  <= rsw(2);   -- S9  Shuffle
            sw_brush <= rsw(3);   -- S10 Brushed
            sw_neg   <= rsw(4);   -- S11 Negative
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Input raster: measurement, hsync->avid parity, frame anchor.
    --------------------------------------------------------------------------
    p_input : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- encoder pairs Cb/Cr by counting from hsync: record each line's
            -- hsync->avid offset parity (drape pattern)
            hpar <= not hpar;
            if v_h_edge = '1' then hpar <= '0'; end if;
            if data_in.avid = '1' and prev_avid = '0' then line_par <= hpar; end if;

            if v_h_edge = '1' then
                if px > 32 then w_meas <= px; end if;
                px <= (others => '0');
                if line_act = '1' then
                    line_act <= '0';
                    py       <= py + 1;
                    h_meas   <= resize(py, 12) + 1;
                end if;
            elsif data_in.avid = '1' then
                frame_armed <= '1';
                line_act    <= '1';
                px <= px + 1;
                if frame_act = '0' then
                    frame_act <= '1';
                    py        <= (others => '0');
                end if;
            end if;

            anchor <= '0';
            if v_v_edge = '1' and frame_armed = '1' then
                frame_armed <= '0';
                frame_act   <= '0';
                py          <= (others => '0');
                anchor      <= '1';
                par         <= not par;
                g_w <= w_meas;
                g_h <= h_meas;
                if w_meas >= 1024 then g_k4 <= '1'; else g_k4 <= '0'; end if;
            end if;
        end if;
    end process p_input;

    --------------------------------------------------------------------------
    -- Per-frame sequencer.  One 16x16 serial multiplier (16 steps) and one
    -- 23/12 restoring divider (23 steps); each step issues one op and reads
    -- the previous result.  ~250 clocks after the anchor.
    --   kxc = 2^22/W   kyc = 2^22/H         (store cells per px / line, Q16)
    --   cpc = 2^18/ncol  cpt = 2^18/ntile   (cells per column / tile, Q8)
    --   dcx = ncol kxc / 64   dry = ntile kyc / 64   dcy = lean kyc / 2^14
    --   stairC = stair cpt    Sv = depth cpt    Sh = depth cpc
    --------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_rem : unsigned(12 downto 0);
        variable v_mul, v_div : std_logic;
        variable v_ma, v_mb : unsigned(15 downto 0);
        variable v_num : unsigned(22 downto 0);
        variable v_den : unsigned(11 downto 0);
        variable v_t   : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            commit <= '0';

            if mcnt /= 0 then
                if mb(15) = '1' then
                    macc <= (macc(30 downto 0) & '0') + resize(ma, 32);
                else
                    macc <= macc(30 downto 0) & '0';
                end if;
                mb   <= mb(14 downto 0) & '0';
                mcnt <= mcnt - 1;
            end if;

            if dcnt /= 0 then
                v_rem := drem(11 downto 0) & dnum(22);
                if v_rem >= resize(dden, 13) then
                    drem <= v_rem - resize(dden, 13);
                    dq   <= dq(14 downto 0) & '1';
                else
                    drem <= v_rem;
                    dq   <= dq(14 downto 0) & '0';
                end if;
                dnum <= dnum(21 downto 0) & '0';
                dcnt <= dcnt - 1;
            end if;

            v_mul := '0'; v_div := '0';
            v_ma := (others => '0'); v_mb := (others => '0');
            v_num := (others => '0'); v_den := (others => '0');

            if anchor = '1' then sq_go <= '1'; end if;

            if mcnt = 0 and dcnt = 0 then
                pc <= pc + 1;
                case to_integer(pc) is
                    when 0 =>
                        if sq_go = '0' then pc <= (others => '0'); else sq_go <= '0'; end if;
                    when 1 =>
                        v_div := '1'; v_num(22) := '1'; v_den := g_w;
                    when 2 =>
                        kxc <= dq;
                        v_div := '1'; v_num(22) := '1'; v_den := g_h;
                    when 3 =>
                        kyc <= dq;
                        v_div := '1'; v_num(18) := '1'; v_den := resize(k_ncol, 12);
                    when 4 =>
                        cpc <= dq;
                        v_div := '1'; v_num(18) := '1'; v_den := resize(k_ntile, 12);
                    when 5 =>
                        cpt <= dq;
                        v_mul := '1'; v_ma := resize(k_ncol, 16); v_mb := kxc;
                    when 6 =>
                        c_dcx <= macc(25 downto 10);
                        v_mul := '1'; v_ma := resize(k_ntile, 16); v_mb := kyc;
                    when 7 =>
                        c_dry <= macc(25 downto 10);
                        v_mul := '1'; v_ma := resize(k_lean, 16); v_mb := kyc;
                    when 8 =>
                        if k_lean_neg = '1' then
                            c_dcy <= -signed(resize(macc(25 downto 14), 12));
                        else
                            c_dcy <= signed(resize(macc(25 downto 14), 12));
                        end if;
                        v_mul := '1'; v_ma := resize(k_stair, 16); v_mb := cpt;
                    when 9 =>
                        if k_stair_neg = '1' then
                            c_stairc <= -signed(resize(macc(21 downto 10), 12));
                            c_stairt <= -signed(resize(k_stair, 9));
                        else
                            c_stairc <= signed(resize(macc(21 downto 10), 12));
                            c_stairt <= signed(resize(k_stair, 9));
                        end if;
                        v_mul := '1'; v_ma := resize(k_depth, 16); v_mb := cpt;
                    when 10 =>
                        v_t := macc(28 downto 13);
                        if v_t > 192 then c_sv <= to_unsigned(192, 8); else c_sv <= v_t(7 downto 0); end if;
                        v_mul := '1'; v_ma := resize(k_depth, 16); v_mb := cpc;
                    when 11 =>
                        v_t := macc(28 downto 13);
                        if v_t > 128 then c_sh <= to_unsigned(128, 8); else c_sh <= v_t(7 downto 0); end if;
                        commit <= '1';
                        pc <= (others => '0');
                    when others =>
                        pc <= (others => '0');
                end case;
            end if;

            if v_mul = '1' then
                ma <= v_ma; mb <= v_mb;
                macc <= (others => '0'); mcnt <= to_unsigned(16, 5);
            end if;
            if v_div = '1' then
                dnum <= v_num; dden <= v_den;
                drem <= (others => '0'); dq <= (others => '0');
                dcnt <= to_unsigned(23, 5);
            end if;
        end if;
    end process p_seq;

    --------------------------------------------------------------------------
    -- k=1: DDAs.  Per pixel: column phase ca (Q8.16) and store x (Q6.16);
    -- per active line (at the avid fall): lean, tile row phase, store y.
    --------------------------------------------------------------------------
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            sy(0) <= (data_in.hsync_n, data_in.vsync_n, data_in.avid, data_in.field_n);
            for i in 1 to C_SYNC - 1 loop sy(i) <= sy(i - 1); end loop;
            din1 <= data_in;

            if commit = '1' then
                ca_line <= (others => '0');
                ra_line <= (others => '0');
                yc_line <= C_XC0;
            elsif data_in.avid = '0' and prev_avid = '1' then
                ca_line <= ca_line + resize(c_dcy, 24);
                ra_line <= ra_line + resize(c_dry, 22);
                yc_line <= yc_line + signed(resize(kyc, 22));
            end if;
            if data_in.avid = '1' then
                ca <= ca + signed(resize(c_dcx, 24));
                xc <= xc + signed(resize(kxc, 22));
            else
                ca <= ca_line;
                xc <= C_XC0;
            end if;
        end if;
    end process p_dda;

    --------------------------------------------------------------------------
    -- k=2 .. k=16: tile geometry -> lens -> store read coordinates.
    --------------------------------------------------------------------------
    p_geom : process(clk)
        variable v_h1  : unsigned(7 downto 0);
        variable v_d7, v_e7 : unsigned(6 downto 0);
        variable v_e8  : unsigned(7 downto 0);
        variable v_s9  : unsigned(8 downto 0);
        variable v_eq  : unsigned(9 downto 0);
        variable v_t6  : unsigned(5 downto 0);
        variable v_ly  : unsigned(7 downto 0);
        variable v_c   : unsigned(7 downto 0);
        variable v_dv  : signed(18 downto 0);
        variable v_du  : unsigned(15 downto 0);
        variable v_h   : unsigned(15 downto 0);
        variable v_dvs : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            -- k=2: column index / fraction; per-line values enter the pipe
            s02_c8  <= ca(23 downto 16);
            s02_lx8 <= unsigned(ca(15 downto 8));
            s02_xc  <= unsigned(xc(21 downto 6));
            s02_yc  <= unsigned(yc_line(21 downto 6));
            s02_ra  <= ra_line;

            -- k=3: stair products (tiles Q8, cells Q6); column hash
            s03_stt <= s02_c8 * c_stairt;
            s03_stc <= s02_c8 * c_stairc;
            v_h1 := resize(unsigned(s02_c8) * to_unsigned(91, 8), 8);   -- c*91 mod 256
            if sw_shuf = '1' then
                s03_stag <= v_h1 xor (v_h1(4 downto 0) & v_h1(7 downto 5));
            else
                s03_stag <= (others => '0');
            end if;
            s03_lx8 <= s02_lx8; s03_xc <= s02_xc; s03_yc <= s02_yc; s03_ra <= s02_ra;
            s03_c8 <= s02_c8;

            -- k=4: tile phase = row phase + stair (mod 64 tiles); content y + stair
            s04_tp  <= s03_ra + unsigned(shift_left(resize(s03_stt, 22), 8));
            s04_ycs <= s03_yc + (unsigned(s03_stc(11 downto 0)) & x"0");
            s04_stag <= s03_stag; s04_lx8 <= s03_lx8; s04_xc <= s03_xc; s04_c8 <= s03_c8;

            -- k=5: + shuffle stagger
            s05_tp  <= s04_tp + resize(unsigned(s04_stag) & "00000000", 22);
            s05_ycs <= s04_ycs; s05_xc <= s04_xc; s05_lx8 <= s04_lx8; s05_c8 <= s04_c8;

            -- k=6: tile index, local coords, distances from the tile centre
            v_t6 := s05_tp(21 downto 16);
            v_ly := s05_tp(15 downto 8);
            s06_t6  <= v_t6;
            s06_ly8 <= v_ly;
            s06_c8  <= unsigned(s05_c8);
            if v_ly(7) = '0' then v_d7 := not v_ly(6 downto 0); else v_d7 := v_ly(6 downto 0); end if;
            if s05_lx8(7) = '0' then v_e7 := not s05_lx8(6 downto 0); else v_e7 := s05_lx8(6 downto 0); end if;
            s06_d8 <= v_d7 & '0';
            s06_e8 <= v_e7 & '0';
            s06_sgn  <= v_ly(7) xor sw_conc;    -- '1': lower half reads down
            s06_hsgn <= s05_lx8(7);             -- '1': right half reads right
            s06_ycs <= s05_ycs; s06_xc <= s05_xc;

            -- k=7: corner excess (last 1/8 of the column width), linear x lens;
            -- grain hash partials
            if s06_e8 >= 224 then s07_eex <= resize(s06_e8 - 224, 5); else s07_eex <= (others => '0'); end if;
            s07_elin <= resize(s06_e8(7 downto 3), 8) + resize(s06_e8(7 downto 5), 8);
            s07_d8 <= s06_d8; s07_sgn <= s06_sgn; s07_hsgn <= s06_hsgn;
            s07_ycs <= s06_ycs; s07_xc <= s06_xc;
            s07_ha1 <= resize(s06_ly8 & "0000000", 16) + resize(s06_ly8 & "00000", 16);           -- ly*(128+32)
            s07_ha2 <= resize(s06_ly8 & "00", 16) + resize(s06_ly8, 16);                           -- ly*5
            s07_hb1 <= resize(s06_c8 & "00000", 16) + resize(s06_c8 & "000", 16);                  -- c*40
            s07_hb2 <= resize(s06_c8 & "00", 16) + resize(s06_c8, 16);                             -- c*5
            s07_hc1 <= resize(s06_t6 & "000000", 16) + resize(s06_t6 & "0000", 16);                -- t*80
            s07_hc2 <= resize(s06_t6 & "000", 16) + resize(s06_t6 & '0', 16);                      -- t*10

            -- k=8: corner excess squared (32-entry table)
            s08_eex2 <= resize(s07_eex * s07_eex, 10);
            s08_elin <= s07_elin; s08_d8 <= s07_d8; s08_sgn <= s07_sgn; s08_hsgn <= s07_hsgn;
            s08_ycs <= s07_ycs; s08_xc <= s07_xc;
            s08_ha <= s07_ha1 - s07_ha2;        -- ly*157
            s08_hb <= s07_hb1 + s07_hb2;        -- c*45
            s08_hc <= s07_hc1 + s07_hc2;        -- t*90

            -- k=9: rounded-corner distance; x lens profile (Q7 columns)
            v_s9 := resize(s08_d8, 9) + resize(s08_eex2(9 downto 2), 9);
            if v_s9 > 255 then s09_d8c <= (others => '1'); else s09_d8c <= v_s9(7 downto 0); end if;
            s09_hd <= s08_elin + resize(s08_eex2(9 downto 3), 8);
            s09_sgn <= s08_sgn; s09_hsgn <= s08_hsgn; s09_ycs <= s08_ycs; s09_xc <= s08_xc;
            s09_ha <= s08_ha;
            s09_hbc <= s08_hb + s08_hc;

            -- k=10: profile ROM (EBR, registered address)
            s10_prom <= PROM(to_integer(s09_d8c));
            s10_hd <= s09_hd; s10_sgn <= s09_sgn; s10_hsgn <= s09_hsgn;
            s10_ycs <= s09_ycs; s10_xc <= s09_xc;
            s10_h <= s09_ha xor rotl16(s09_hbc, 5);

            -- k=11: fabric re-register
            s11_prom <= s10_prom;
            s11_hd <= s10_hd; s11_sgn <= s10_sgn; s11_hsgn <= s10_hsgn;
            s11_ycs <= s10_ycs; s11_xc <= s10_xc;
            s11_h <= s10_h + (s10_h(11 downto 0) & "0000");

            -- k=12: lens products, operands split hi/lo
            s12_dvh <= signed(s11_prom(15 downto 6)) * signed('0' & c_sv(7 downto 4));
            s12_dvl <= signed(s11_prom(15 downto 6)) * signed('0' & c_sv(3 downto 0));
            s12_duh <= s11_hd * c_sh(7 downto 4);
            s12_dul <= s11_hd * c_sh(3 downto 0);
            s12_hl  <= signed(s11_prom(5 downto 0));
            s12_sgn <= s11_sgn; s12_hsgn <= s11_hsgn; s12_ycs <= s11_ycs; s12_xc <= s11_xc;
            v_h := s11_h xor ("000000" & s11_h(15 downto 6));
            s12_h <= v_h;

            -- k=13: combine (dv: Q8 tiles x Q3 cells/tile = Q11 -> Q10)
            v_dv := shift_left(resize(s12_dvh, 19), 4) + resize(s12_dvl, 19);
            s13_dv <= v_dv(18 downto 1);
            v_du := shift_left(resize(s12_duh, 16), 4) + resize(s12_dul, 16);
            s13_du <= v_du;
            s13_sgn <= s12_sgn; s13_hsgn <= s12_hsgn; s13_ycs <= s12_ycs; s13_xc <= s12_xc;
            s13_n4 <= signed(s12_h(3 downto 0) xor s12_h(11 downto 8));
            hl_d(13) <= s12_hl; sgn_d(13) <= s12_sgn;

            -- k=14: sign per tile half
            if s13_sgn = '1' then v_dvs := s13_dv; else v_dvs := -s13_dv; end if;
            s14_dvs <= v_dvs(16 downto 0);
            if s13_hsgn = '1' then
                s14_dus <= signed(resize(s13_du, 17));
            else
                s14_dus <= -signed(resize(s13_du, 17));
            end if;
            s14_ycs <= s13_ycs; s14_xc <= s13_xc;
            n_d(14) <= s13_n4;

            -- k=15: read coordinates (luma cells Q6.10, wrap)
            s15_v <= s14_ycs + unsigned(s14_dvs(15 downto 0));
            s15_u <= s14_xc  + unsigned(s14_dus(15 downto 0));

            -- k=16: chroma coordinates (-0.5 luma cell more; sliced /2 later)
            s16_v <= s15_v; s16_u <= s15_u;
            s16_vc <= s15_v - 512;
            s16_uc <= s15_u - 512;

            -- delay lines for the highlight, rim side and grain
            for i in 14 to 30 loop hl_d(i) <= hl_d(i - 1); sgn_d(i) <= sgn_d(i - 1); end loop;
            for i in 15 to 26 loop n_d(i) <= n_d(i - 1); end loop;
        end if;
    end process p_geom;

    --------------------------------------------------------------------------
    -- k=17 .. k=24: sub-bank addresses, EBR, bilinear.
    --------------------------------------------------------------------------
    p_read : process(clk)
        variable v_iu, v_iv : unsigned(5 downto 0);
        variable v_iuc, v_ivc : unsigned(4 downto 0);
        variable v_colE, v_colO, v_rowE, v_rowO : unsigned(4 downto 0);
        variable v_ccolE, v_ccolO, v_crowE, v_crowO : unsigned(3 downto 0);
        variable v_rb : std_logic;
        variable v_lx, v_ty : std_logic;
        variable v_ytl, v_ytr, v_ybl, v_ybr : std_logic_vector(7 downto 0);
        variable v_ctl, v_ctr, v_cbl, v_cbr : std_logic_vector(7 downto 0);
        variable v_t13, v_b13 : signed(12 downto 0);
        variable v_t10, v_b10 : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- k=17: integer / fraction, sub-bank addresses
            v_iu  := s16_u(15 downto 10);
            v_iv  := s16_v(15 downto 10);
            v_iuc := s16_uc(15 downto 11);
            v_ivc := s16_vc(15 downto 11);
            v_colO := v_iu(5 downto 1);
            if v_iu(0) = '1' then v_colE := v_colO + 1; else v_colE := v_colO; end if;
            v_rowO := v_iv(5 downto 1);
            if v_iv(0) = '1' then v_rowE := v_rowO + 1; else v_rowE := v_rowO; end if;
            v_ccolO := v_iuc(4 downto 1);
            if v_iuc(0) = '1' then v_ccolE := v_ccolO + 1; else v_ccolE := v_ccolO; end if;
            v_crowO := v_ivc(4 downto 1);
            if v_ivc(0) = '1' then v_crowE := v_crowO + 1; else v_crowE := v_crowO; end if;
            v_rb := not par;
            ya00 <= v_rb & v_rowE & v_colE;
            ya10 <= v_rb & v_rowE & v_colO;
            ya01 <= v_rb & v_rowO & v_colE;
            ya11 <= v_rb & v_rowO & v_colO;
            ca00 <= v_rb & v_crowE & v_ccolE;
            ca10 <= v_rb & v_crowE & v_ccolO;
            ca01 <= v_rb & v_crowO & v_ccolE;
            ca11 <= v_rb & v_crowO & v_ccolO;
            sb17 <= v_iu(0) & v_iv(0) & std_logic_vector(s16_u(9 downto 6)) & std_logic_vector(s16_v(9 downto 6))
                  & v_iuc(0) & v_ivc(0) & std_logic_vector(s16_uc(10 downto 7)) & std_logic_vector(s16_vc(10 downto 7));

            -- k=18 = EBR read (p_ram)
            sb18 <= sb17;

            -- k=19: reorder taps (left = sub-bank iu(0), top = iv(0)), weights
            v_lx := sb18(19); v_ty := sb18(18);
            if v_lx = '0' and v_ty = '0' then
                v_ytl := q00; v_ytr := q10; v_ybl := q01; v_ybr := q11;
            elsif v_lx = '1' and v_ty = '0' then
                v_ytl := q10; v_ytr := q00; v_ybl := q11; v_ybr := q01;
            elsif v_lx = '0' and v_ty = '1' then
                v_ytl := q01; v_ytr := q11; v_ybl := q00; v_ybr := q10;
            else
                v_ytl := q11; v_ytr := q01; v_ybl := q10; v_ybr := q00;
            end if;
            s19_ytl <= signed('0' & v_ytl); s19_ytr <= signed('0' & v_ytr);
            s19_ybl <= signed('0' & v_ybl); s19_ybr <= signed('0' & v_ybr);
            v_lx := sb18(9); v_ty := sb18(8);
            if v_lx = '0' and v_ty = '0' then
                v_ctl := e00; v_ctr := e10; v_cbl := e01; v_cbr := e11;
            elsif v_lx = '1' and v_ty = '0' then
                v_ctl := e10; v_ctr := e00; v_cbl := e11; v_cbr := e01;
            elsif v_lx = '0' and v_ty = '1' then
                v_ctl := e01; v_ctr := e11; v_cbl := e00; v_cbr := e10;
            else
                v_ctl := e11; v_ctr := e01; v_cbl := e10; v_cbr := e00;
            end if;
            s19_utl <= resize(signed(v_ctl(7 downto 4)), 5); s19_vtl <= resize(signed(v_ctl(3 downto 0)), 5);
            s19_utr <= resize(signed(v_ctr(7 downto 4)), 5); s19_vtr <= resize(signed(v_ctr(3 downto 0)), 5);
            s19_ubl <= resize(signed(v_cbl(7 downto 4)), 5); s19_vbl <= resize(signed(v_cbl(3 downto 0)), 5);
            s19_ubr <= resize(signed(v_cbr(7 downto 4)), 5); s19_vbr <= resize(signed(v_cbr(3 downto 0)), 5);
            s19_wx  <= unsigned(sb18(17 downto 14));
            s19_wy  <= unsigned(sb18(13 downto 10));
            s19_wxc <= unsigned(sb18(7 downto 4));
            s19_wyc <= unsigned(sb18(3 downto 0));

            -- k=20: horizontal differences
            s20_ytl <= s19_ytl; s20_ybl <= s19_ybl;
            s20_dyt <= s19_ytr - s19_ytl; s20_dyb <= s19_ybr - s19_ybl;
            s20_utl <= s19_utl; s20_ubl <= s19_ubl;
            s20_dut <= resize(s19_utr, 6) - resize(s19_utl, 6); s20_dub <= resize(s19_ubr, 6) - resize(s19_ubl, 6);
            s20_vtl <= s19_vtl; s20_vbl <= s19_vbl;
            s20_dvt <= resize(s19_vtr, 6) - resize(s19_vtl, 6); s20_dvb <= resize(s19_vbr, 6) - resize(s19_vbl, 6);
            s20_wx <= s19_wx; s20_wy <= s19_wy; s20_wxc <= s19_wxc; s20_wyc <= s19_wyc;

            -- k=21: x weight products
            s21_ytl <= s20_ytl; s21_ybl <= s20_ybl;
            s21_pyt <= smul4(s20_dyt, s20_wx, 13); s21_pyb <= smul4(s20_dyb, s20_wx, 13);
            s21_utl <= s20_utl; s21_ubl <= s20_ubl; s21_vtl <= s20_vtl; s21_vbl <= s20_vbl;
            s21_put <= smul4(s20_dut, s20_wxc, 10); s21_pub <= smul4(s20_dub, s20_wxc, 10);
            s21_pvt <= smul4(s20_dvt, s20_wxc, 10); s21_pvb <= smul4(s20_dvb, s20_wxc, 10);
            s21_wy <= s20_wy; s21_wyc <= s20_wyc;

            -- k=22: top / bottom rows (x16) and their difference
            v_t13 := shift_left(resize(s21_ytl, 13), 4) + s21_pyt;
            v_b13 := shift_left(resize(s21_ybl, 13), 4) + s21_pyb;
            s22_yt <= v_t13; s22_dy <= resize(v_b13, 14) - resize(v_t13, 14);
            v_t10 := shift_left(resize(s21_utl, 10), 4) + s21_put;
            v_b10 := shift_left(resize(s21_ubl, 10), 4) + s21_pub;
            s22_ut <= v_t10; s22_du <= resize(v_b10, 11) - resize(v_t10, 11);
            v_t10 := shift_left(resize(s21_vtl, 10), 4) + s21_pvt;
            v_b10 := shift_left(resize(s21_vbl, 10), 4) + s21_pvb;
            s22_vt <= v_t10; s22_dv <= resize(v_b10, 11) - resize(v_t10, 11);
            s22_wy <= s21_wy; s22_wyc <= s21_wyc;

            -- k=23: y weight products
            s23_yt <= s22_yt; s23_ut <= s22_ut; s23_vt <= s22_vt;
            s23_py <= smul4(s22_dy, s22_wy, 18);
            s23_pu <= smul4(s22_du, s22_wyc, 15);
            s23_pv <= smul4(s22_dv, s22_wyc, 15);

            -- k=24: interpolated value x256
            s24_y <= shift_left(resize(s23_yt, 17), 4) + resize(s23_py, 17);
            s24_u <= shift_left(resize(s23_ut, 13), 4) + resize(s23_pu, 13);
            s24_v <= shift_left(resize(s23_vt, 13), 4) + resize(s23_pv, 13);
        end if;
    end process p_read;

    p_ram : process(clk)
    begin
        if rising_edge(clk) then
            q00 <= yr00(to_integer(ya00));
            q10 <= yr10(to_integer(ya10));
            q01 <= yr01(to_integer(ya01));
            q11 <= yr11(to_integer(ya11));
            e00 <= cr00(to_integer(ca00));
            e10 <= cr10(to_integer(ca10));
            e01 <= cr01(to_integer(ca01));
            e11 <= cr11(to_integer(ca11));

            if wy_v = '1' and wy_px = '0' and wy_py = '0' then yr00(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '1' and wy_py = '0' then yr10(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '0' and wy_py = '1' then yr01(to_integer(wy_a)) <= wy_d; end if;
            if wy_v = '1' and wy_px = '1' and wy_py = '1' then yr11(to_integer(wy_a)) <= wy_d; end if;
            if wc_v = '1' and wc_px = '0' and wc_py = '0' then cr00(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '1' and wc_py = '0' then cr10(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '0' and wc_py = '1' then cr01(to_integer(wc_a)) <= wc_d; end if;
            if wc_v = '1' and wc_px = '1' and wc_py = '1' then cr11(to_integer(wc_a)) <= wc_d; end if;
        end if;
    end process p_ram;

    --------------------------------------------------------------------------
    -- k=25 .. k=38: tone (negative, grain, contrast), palette, gloss,
    -- chroma select + parity swap, blanking gate, [1 2 1] conditioning.
    --------------------------------------------------------------------------
    p_tone : process(clk)
        variable v_a  : signed(8 downto 0);
        variable v_s  : signed(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_y  : signed(11 downto 0);
        variable v_u, v_v : signed(10 downto 0);
        variable v_hl : signed(11 downto 0);
        variable v_sum : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- k=25: tone 0..239; stored chroma -> codes; negative
            if sw_neg = '1' then
                s25_y8 <= to_unsigned(239, 8) - unsigned(s24_y(15 downto 8));
            else
                s25_y8 <= unsigned(s24_y(15 downto 8));
            end if;
            v_u := resize(s24_u(12 downto 2), 11) + 512;
            v_v := resize(s24_v(12 downto 2), 11) + 512;
            s25_cu <= unsigned(v_u(9 downto 0));
            s25_cv <= unsigned(v_v(9 downto 0));

            -- k=26: grain amplitude (dark tones only)
            v_a := to_signed(96, 9) - signed(resize(s25_y8, 9));
            if v_a < 0 then s26_amp <= (others => '0'); else s26_amp <= unsigned(v_a(6 downto 2)); end if;
            s26_y8 <= s25_y8; s26_cu <= s25_cu; s26_cv <= s25_cv;

            -- k=27: grain product
            if sw_brush = '1' then
                s27_np <= smul5(n_d(26), s26_amp, 9);
            else
                s27_np <= (others => '0');
            end if;
            s27_y8 <= s26_y8; s27_cu <= s26_cu; s27_cv <= s26_cv;

            -- k=28: add grain, clamp
            v_s := signed(resize(s27_y8, 10)) + resize(s27_np(8 downto 3), 10);
            if v_s < 0 then s28_y8 <= (others => '0');
            elsif v_s > 239 then s28_y8 <= to_unsigned(239, 8);
            else s28_y8 <= unsigned(v_s(7 downto 0)); end if;
            s28_cu <= s27_cu; s28_cv <= s27_cv;

            -- k=29: contrast x1.5 about 120: y + y/2 (then - 60)
            s29_y <= resize(s28_y8, 10) + resize(s28_y8(7 downto 1), 10);
            s29_cu <= s28_cu; s29_cv <= s28_cv;

            -- k=30: - 60, clamp 0..239
            v_s := signed(resize(s29_y, 10)) - 60;
            if v_s < 0 then s30_y <= (others => '0');
            elsif v_s > 239 then s30_y <= to_unsigned(239, 8);
            else s30_y <= unsigned(v_s(7 downto 0)); end if;
            s30_cu <= s29_cu; s30_cv <= s29_cv;

            -- k=31: palette address; luma x3; highlight x gloss
            s31_pa  <= k_pal & s30_y(7 downto 1);
            s31_y3  <= resize(s30_y, 10) + resize(s30_y & '0', 10);
            s31_hlp <= smul5(hl_d(30), k_gloss, 12);
            s31_sgn <= sgn_d(30);
            s31_cu <= s30_cu; s31_cv <= s30_cv;

            -- k=32: palette ROM (EBR); Y base; lower-rim highlight halved
            s32_pal <= PALROM(to_integer(s31_pa));
            s32_yb  <= s31_y3 + 64;
            if s31_sgn = '1' and s31_hlp > 0 then
                s32_hlh <= shift_right(s31_hlp, 1);
            else
                s32_hlh <= s31_hlp;
            end if;
            s32_cu <= s31_cu; s32_cv <= s31_cv;

            -- k=33: fabric re-register; highlight x5/16
            s33_pal <= s32_pal;
            v_hl := s32_hlh + shift_right(s32_hlh, 2);
            s33_hl2 <= v_hl(11 downto 2);
            s33_yb <= s32_yb; s33_cu <= s32_cu; s33_cv <= s32_cv;

            -- k=34: Y + highlight; chroma source
            s34_y <= signed(resize(s33_yb, 12)) + resize(s33_hl2, 12);
            if sw_src = '1' then
                s34_u <= s33_cu; s34_v <= s33_cv;
            else
                v_u := resize(signed(s33_pal(15 downto 8)) & "00", 11) + 512;
                v_v := resize(signed(s33_pal(7 downto 0)) & "00", 11) + 512;
                s34_u <= unsigned(v_u(9 downto 0));
                s34_v <= unsigned(v_v(9 downto 0));
            end if;

            -- k=35: Y clamp; chroma parity swap (drape pattern)
            v_y := s34_y;
            if v_y < 64 then v_y := to_signed(64, 12); elsif v_y > 800 then v_y := to_signed(800, 12); end if;
            s35_y <= unsigned(v_y(9 downto 0));
            if line_par = '1' then
                s35_u <= s34_v; s35_v <= s34_u;
            else
                s35_u <= s34_u; s35_v <= s34_v;
            end if;

            -- k=36: blanking gate (avid of sy(34) is this pixel)
            if sy(34).avid = '0' then
                x5_y <= C_BLKY; x5_u <= C_MID; x5_v <= C_MID; x5_bl <= '1';
            else
                x5_y <= s35_y; x5_u <= s35_u; x5_v <= s35_v; x5_bl <= '0';
            end if;

            -- k=37: centre tap; k=38: [1 2 1]/4 on Y, U and V, re-gated
            d6_y <= x5_y; d6_u <= x5_u; d6_v <= x5_v; d6_bl <= x5_bl;
            d7_y <= d6_y; d7_u <= d6_u; d7_v <= d6_v;
            if d6_bl = '1' then
                s_out.y <= std_logic_vector(C_BLKY);
                s_out.u <= std_logic_vector(C_MID);
                s_out.v <= std_logic_vector(C_MID);
            else
                v_sum := resize(x5_y, 12) + (resize(d6_y, 12) sll 1) + resize(d7_y, 12) + 2;
                s_out.y <= std_logic_vector(v_sum(11 downto 2));
                v_sum := resize(x5_u, 12) + (resize(d6_u, 12) sll 1) + resize(d7_u, 12) + 2;
                s_out.u <= std_logic_vector(v_sum(11 downto 2));
                v_sum := resize(x5_v, 12) + (resize(d6_v, 12) sll 1) + resize(d7_v, 12) + 2;
                s_out.v <= std_logic_vector(v_sum(11 downto 2));
            end if;
            s_out.hsync_n <= sy(36).hsync_n;
            s_out.vsync_n <= sy(36).vsync_n;
            s_out.avid    <= sy(36).avid;
            s_out.field_n <= sy(36).field_n;

            -- k=39, 40: pad (latency a multiple of 4)
            s_pad(0) <= s_out;
            s_pad(1) <= s_pad(0);
        end if;
    end process p_tone;

    data_out <= s_pad(1);

    --------------------------------------------------------------------------
    -- Write side.  The input at k=1 is averaged by a one-pole IIR along the
    -- line (k = 4 or 3 for luma, +1 for chroma; 4 fraction bits) and
    -- captured at each cell end (64 cells across).  Luma rows are sampled on
    -- the line holding the row's middle, chroma rows (2x2 luma cells) on the
    -- line crossing from an even into an odd luma row.
    --------------------------------------------------------------------------
    p_write : process(clk)
        variable v_av : std_logic;
        variable v_x  : unsigned(12 downto 0);
        variable v_y  : unsigned(12 downto 0);
        variable v_ty, v_tu, v_tv : signed(14 downto 0);
        variable v_ny, v_nu, v_nv : signed(14 downto 0);
        variable v_pulse : std_logic;
        variable v_w15 : unsigned(14 downto 0);
        variable v_q : unsigned(8 downto 0);
        variable v_cu, v_cv : unsigned(9 downto 0);
        variable v_s9 : signed(8 downto 0);
        variable v_u4, v_v4 : signed(3 downto 0);
        variable x_y, x_u, x_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_av := sy(0).avid;
            x_y := unsigned(din1.y); x_u := unsigned(din1.u); x_v := unsigned(din1.v);
            v_pulse := '0';

            -- IIR split so each clock is one carry chain:
            --   e <= x*16/2^k - acc/2^k     acc <= acc + e
            if g_k4 = '1' then
                v_ty := signed(resize(x_y, 15)) - signed(resize(shift_right(w_yi, 4), 15));
                v_tu := signed(resize(x_u(9 downto 1), 15)) - signed(resize(shift_right(w_ui, 5), 15));
                v_tv := signed(resize(x_v(9 downto 1), 15)) - signed(resize(shift_right(w_vi, 5), 15));
            else
                v_ty := signed(resize(shift_left(resize(x_y, 14), 1), 15)) - signed(resize(shift_right(w_yi, 3), 15));
                v_tu := signed(resize(x_u, 15)) - signed(resize(shift_right(w_ui, 4), 15));
                v_tv := signed(resize(x_v, 15)) - signed(resize(shift_right(w_vi, 4), 15));
            end if;
            v_ny := signed(resize(w_yi, 15)) + w_ey;
            v_nu := signed(resize(w_ui, 15)) + w_eu;
            v_nv := signed(resize(w_vi, 15)) + w_ev;

            if v_av = '1' and w_prev = '0' then
                w_x  <= (others => '0');
                w_c  <= (others => '0');
                w_yi <= shift_left(resize(x_y, 14), 4);
                w_ui <= shift_left(resize(x_u, 14), 4);
                w_vi <= shift_left(resize(x_v, 14), 4);
                w_ey <= (others => '0'); w_eu <= (others => '0'); w_ev <= (others => '0');
                w_lsl <= w_lsn;
                w_csl <= w_csn;
                w_rl  <= w_r;
                w_par <= line_par;
            elsif v_av = '1' then
                v_x := w_x + 64;
                if v_x >= resize(g_w, 13) then
                    w_x <= v_x - resize(g_w, 13);
                    if w_c /= 63 then
                        v_pulse := '1';
                        w_c <= w_c + 1;
                    end if;
                else
                    w_x <= v_x;
                end if;
                w_yi <= unsigned(v_ny(13 downto 0));
                w_ui <= unsigned(v_nu(13 downto 0));
                w_vi <= unsigned(v_nv(13 downto 0));
                w_ey <= v_ty; w_eu <= v_tu; w_ev <= v_tv;
            elsif w_prev = '1' then
                v_pulse := '1';
                v_y := resize(w_yacc, 13) + 64;
                if v_y >= resize(g_h, 13) then
                    w_yacc <= resize(v_y - resize(g_h, 13), 12);
                    if w_r /= 63 then w_r <= w_r + 1; end if;
                else
                    w_yacc <= v_y(11 downto 0);
                end if;
            end if;
            w_prev <= v_av;

            if anchor = '1' then
                w_yacc <= (others => '0');
                w_r    <= (others => '0');
            end if;

            if resize(w_yacc, 13) <= resize(g_h(11 downto 1), 13)
               and resize(w_yacc, 13) + 64 > resize(g_h(11 downto 1), 13) then
                w_lsn <= '1';
            else
                w_lsn <= '0';
            end if;
            if resize(w_yacc, 13) + 64 >= resize(g_h, 13) and w_r(0) = '0' then
                w_csn <= '1';
            else
                w_csn <= '0';
            end if;

            -- L1 (cell-end pulse): luma 0..239 in 4-code units above black,
            -- chroma canonical signed in 4-code units
            if v_pulse = '1' then
                l1_vy <= w_lsl;
                l1_vc <= w_csl and w_c(0);
            else
                l1_vy <= '0';
                l1_vc <= '0';
            end if;
            l1_c <= w_c; l1_r <= w_rl;
            v_w15 := shift_right(resize(w_yi, 15) + 32, 6);
            v_q := v_w15(8 downto 0);
            if v_q < 16 then l1_y <= (others => '0');
            elsif v_q > 255 then l1_y <= to_unsigned(239, 8);
            else l1_y <= resize(v_q - 16, 8); end if;
            v_w15 := shift_right(resize(w_ui, 15) + 32, 6);
            v_cu := v_w15(9 downto 0);
            v_w15 := shift_right(resize(w_vi, 15) + 32, 6);
            v_cv := v_w15(9 downto 0);
            if w_par = '1' then
                l1_u <= resize(signed(v_cv) - 128, 9);
                l1_v <= resize(signed(v_cu) - 128, 9);
            else
                l1_u <= resize(signed(v_cu) - 128, 9);
                l1_v <= resize(signed(v_cv) - 128, 9);
            end if;

            -- L2: luma write
            wy_v  <= l1_vy;
            wy_d  <= std_logic_vector(l1_y);
            wy_px <= l1_c(0); wy_py <= l1_r(0);
            wy_a  <= par & l1_r(5 downto 1) & l1_c(5 downto 1);

            -- L2: chroma write (rounded to 4-bit signed, -7..7)
            wc_v  <= l1_vc;
            v_s9 := l1_u + 8;
            if v_s9 > 119 then v_u4 := to_signed(7, 4); elsif v_s9 < -112 then v_u4 := to_signed(-7, 4);
            else v_u4 := v_s9(7 downto 4); end if;
            v_s9 := l1_v + 8;
            if v_s9 > 119 then v_v4 := to_signed(7, 4); elsif v_s9 < -112 then v_v4 := to_signed(-7, 4);
            else v_v4 := v_s9(7 downto 4); end if;
            wc_d  <= std_logic_vector(v_u4) & std_logic_vector(v_v4);
            wc_px <= l1_c(1); wc_py <= l1_r(1);
            wc_a  <= par & l1_r(5 downto 2) & l1_c(5 downto 2);
        end if;
    end process p_write;

end architecture lamella;
