-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- whorl.vhd  (Vellum set, v0.1.0 -- nested polygons whose shape evolves as they go out, whirling)
--
-- ONE figure filling the screen: a nest of polygons whose shape changes with the radius.  A sharp,
-- slightly pinched N-gon at the centre rounds into a circle mid-screen and opens into a soft N-lobed
-- flower at the edges, each band turned a little more than the one inside (a twist), so the nest whirls.
-- Nautilus whorls, rose windows, Vasarely's later polygon spirals.  Luma only (U = V = 512).
--
--   L     = log2(r / 120 virtual px)                      (the Hypnos log-polar engine: log2 r, theta)
--   psi   = theta + twist * L                             twist = 7e/64 (~ e ln2 / 2pi) turns per octave
--   P_N   = log2 cos(pi/N) - log2 cos(a)   (<= 0)         a = angle from the middle of the polygon side,
--                                                          a ROM over the folded sector (5 tables, 64 steps)
--   alpha = clamp(1 - 6 * twist * L, -0.44 e, 1 + e/4)   the shape: > 1 pinched polygon, 1 the N-gon,
--                                                          0 a circle, < 0 a flower (e = P12 Evolve / 50 %)
--   phase = m (L - alpha P_N) + arms theta / 2pi + camera - drift      (bands, mod 4)
--   tone  = S9 Grey: four grey steps by band (black, dark, light, white) ; Ink: black / white
--           S10 Bold: equal bands ; Fine: lines 1/6 of a band, never under 2 px
-- Anti-aliasing: every edge is a one-pixel ramp from a signed distance and the ANALYTIC phase gradient
--   |grad| = m / (r ln2) * |(R, B)|,  R = 1 + e (g P - alpha tan a),  B = alpha tan a  (|(R, B)| ~ max + min / 2)
-- (alpha tan a from the same table), the width rounded up to a half octave so the divide is a shift; where
-- the bands fall under 2 px (the centre) the tone fades to its flat grey.
--
-- Pipeline (stage N = N registers after s_dx; one latency, 42 clocks, in every mode):
--   S1 .. S15   |dx|, |dy| -> log2 hi, log2 lo -> log2 r, theta                         (Hypnos)
--   S16 .. S18  L ; Y = twist * L
--   S19 .. S20  psi -> the sector fold -> table address ; alpha = clamp(1 - 6 Y)
--   S21 .. S24  the table (EBR, fabric register), interpolation -> P ; alpha tan a, R', |B|
--   S25 .. S27  alpha * P, Phi = L - alpha P ; |R| -> max + min / 2
--   S28 .. S30  m * Phi -> phase + arms theta + camera - drift ; log2 |grad| -> half octave, fade
--   S31 .. S36  signed distance -> coverage -> tone = X + Y coverage (X, Y: the fade ROM)
--   S37 .. S41  luma [1 2 1] / 4, 11/16 toward black (white = 768), blanking gate
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.whorl_pkg.all;

architecture whorl of program_top is
    -- data_in -> data_out sync latency; must match processing_delay_clks in the TOML
    constant C_PROCESSING_DELAY_CLKS : natural := 42;

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_DB    : integer := 14;                 -- centre-knob detent deadband
    constant C_LREF  : integer := 113163;             -- log2(120 virtual px), Q14: the shape's pivot radius

    -- stage map.  An EBR delay line written from stage W and read (straight from the EBR output
    -- register) by the logic building stage C runs its read address X = C - 2 - W behind the write.
    constant ST_OUT : integer := 41;                  -- output register
    constant C_XH   : integer := 15 - 2 - 8;          -- log2 hi + flags      S8  -> S15
    constant C_XA   : integer := 27 - 2 - 16;         -- L & the width base   S16 -> S27
    constant C_XT   : integer := 30 - 2 - 16;         -- theta                S16 -> S30
    constant C_XS   : integer := 37;                  -- syncs: aligned with S37

    constant C_SQ_END : integer := 64;

    type t_rom256 is array(0 to 255) of std_logic_vector(15 downto 0);


    -- C_LOGV .. C_UAS: the Hypnos log / polar tables.  C_EXM: exp2 mantissa, Q14 (the sequencer's ROM).
    constant C_LOGV : t_rom256 := (
        x"0000", x"005C", x"00B8", x"0113", x"016E", x"01C9", x"0224", x"027E",
        x"02D7", x"0331", x"038A", x"03E2", x"043B", x"0493", x"04EB", x"0542",
        x"0599", x"05F0", x"0646", x"069C", x"06F2", x"0748", x"079D", x"07F2",
        x"0846", x"089A", x"08EE", x"0942", x"0995", x"09E9", x"0A3B", x"0A8E",
        x"0AE0", x"0B32", x"0B84", x"0BD5", x"0C26", x"0C77", x"0CC7", x"0D18",
        x"0D68", x"0DB7", x"0E07", x"0E56", x"0EA5", x"0EF4", x"0F42", x"0F90",
        x"0FDE", x"102C", x"1079", x"10C6", x"1113", x"1160", x"11AC", x"11F8",
        x"1244", x"1290", x"12DB", x"1326", x"1371", x"13BC", x"1406", x"1450",
        x"149A", x"14E4", x"152E", x"1577", x"15C0", x"1609", x"1652", x"169A",
        x"16E2", x"172A", x"1772", x"17B9", x"1801", x"1848", x"188F", x"18D5",
        x"191C", x"1962", x"19A8", x"19EE", x"1A33", x"1A79", x"1ABE", x"1B03",
        x"1B48", x"1B8D", x"1BD1", x"1C15", x"1C59", x"1C9D", x"1CE1", x"1D24",
        x"1D67", x"1DAA", x"1DED", x"1E30", x"1E72", x"1EB5", x"1EF7", x"1F39",
        x"1F7B", x"1FBC", x"1FFD", x"203F", x"2080", x"20C1", x"2101", x"2142",
        x"2182", x"21C2", x"2202", x"2242", x"2282", x"22C1", x"2300", x"233F",
        x"237E", x"23BD", x"23FC", x"243A", x"2479", x"24B7", x"24F5", x"2532",
        x"2570", x"25AE", x"25EB", x"2628", x"2665", x"26A2", x"26DE", x"271B",
        x"2757", x"2794", x"27D0", x"280C", x"2847", x"2883", x"28BE", x"28FA",
        x"2935", x"2970", x"29AB", x"29E6", x"2A20", x"2A5B", x"2A95", x"2ACF",
        x"2B09", x"2B43", x"2B7D", x"2BB6", x"2BF0", x"2C29", x"2C62", x"2C9B",
        x"2CD4", x"2D0D", x"2D45", x"2D7E", x"2DB6", x"2DEE", x"2E26", x"2E5E",
        x"2E96", x"2ECE", x"2F05", x"2F3D", x"2F74", x"2FAB", x"2FE2", x"3019",
        x"3050", x"3087", x"30BD", x"30F4", x"312A", x"3160", x"3196", x"31CC",
        x"3202", x"3237", x"326D", x"32A2", x"32D8", x"330D", x"3342", x"3377",
        x"33AC", x"33E0", x"3415", x"3449", x"347E", x"34B2", x"34E6", x"351A",
        x"354E", x"3582", x"35B6", x"35E9", x"361D", x"3650", x"3683", x"36B6",
        x"36E9", x"371C", x"374F", x"3781", x"37B4", x"37E7", x"3819", x"384B",
        x"387D", x"38AF", x"38E1", x"3913", x"3945", x"3976", x"39A8", x"39D9",
        x"3A0A", x"3A3C", x"3A6D", x"3A9E", x"3ACF", x"3AFF", x"3B30", x"3B61",
        x"3B91", x"3BC2", x"3BF2", x"3C22", x"3C52", x"3C82", x"3CB2", x"3CE2",
        x"3D12", x"3D41", x"3D71", x"3DA0", x"3DCF", x"3DFF", x"3E2E", x"3E5D",
        x"3E8C", x"3EBB", x"3EE9", x"3F18", x"3F47", x"3F75", x"3FA3", x"3FD2");
    constant C_LOGS : t_rom256 := (
        x"005C", x"005C", x"005B", x"005B", x"005B", x"005B", x"005A", x"0059",
        x"005A", x"0059", x"0058", x"0059", x"0058", x"0058", x"0057", x"0057",
        x"0057", x"0056", x"0056", x"0056", x"0056", x"0055", x"0055", x"0054",
        x"0054", x"0054", x"0054", x"0053", x"0054", x"0052", x"0053", x"0052",
        x"0052", x"0052", x"0051", x"0051", x"0051", x"0050", x"0051", x"0050",
        x"004F", x"0050", x"004F", x"004F", x"004F", x"004E", x"004E", x"004E",
        x"004E", x"004D", x"004D", x"004D", x"004D", x"004C", x"004C", x"004C",
        x"004C", x"004B", x"004B", x"004B", x"004B", x"004A", x"004A", x"004A",
        x"004A", x"004A", x"0049", x"0049", x"0049", x"0049", x"0048", x"0048",
        x"0048", x"0048", x"0047", x"0048", x"0047", x"0047", x"0046", x"0047",
        x"0046", x"0046", x"0046", x"0045", x"0046", x"0045", x"0045", x"0045",
        x"0045", x"0044", x"0044", x"0044", x"0044", x"0044", x"0043", x"0043",
        x"0043", x"0043", x"0043", x"0042", x"0043", x"0042", x"0042", x"0042",
        x"0041", x"0041", x"0042", x"0041", x"0041", x"0040", x"0041", x"0040",
        x"0040", x"0040", x"0040", x"0040", x"003F", x"003F", x"003F", x"003F",
        x"003F", x"003F", x"003E", x"003F", x"003E", x"003E", x"003D", x"003E",
        x"003E", x"003D", x"003D", x"003D", x"003D", x"003C", x"003D", x"003C",
        x"003D", x"003C", x"003C", x"003B", x"003C", x"003B", x"003C", x"003B",
        x"003B", x"003B", x"003B", x"003A", x"003B", x"003A", x"003A", x"003A",
        x"003A", x"003A", x"0039", x"003A", x"0039", x"0039", x"0039", x"0039",
        x"0039", x"0038", x"0039", x"0038", x"0038", x"0038", x"0038", x"0038",
        x"0038", x"0037", x"0038", x"0037", x"0037", x"0037", x"0037", x"0037",
        x"0037", x"0036", x"0037", x"0036", x"0036", x"0036", x"0036", x"0036",
        x"0035", x"0036", x"0035", x"0036", x"0035", x"0035", x"0035", x"0035",
        x"0034", x"0035", x"0034", x"0035", x"0034", x"0034", x"0034", x"0034",
        x"0034", x"0034", x"0033", x"0034", x"0033", x"0033", x"0033", x"0033",
        x"0033", x"0033", x"0032", x"0033", x"0033", x"0032", x"0032", x"0032",
        x"0032", x"0032", x"0032", x"0032", x"0031", x"0032", x"0031", x"0031",
        x"0032", x"0031", x"0031", x"0031", x"0030", x"0031", x"0031", x"0030",
        x"0031", x"0030", x"0030", x"0030", x"0030", x"0030", x"0030", x"0030",
        x"002F", x"0030", x"002F", x"002F", x"0030", x"002F", x"002F", x"002F",
        x"002F", x"002E", x"002F", x"002F", x"002E", x"002E", x"002F", x"002E");
    constant C_UG : t_rom256 := (
        x"2000", x"1E0B", x"1C2C", x"1A64", x"18B1", x"1713", x"158B", x"1417",
        x"12B8", x"116C", x"1034", x"0F0E", x"0DFA", x"0CF7", x"0C04", x"0B21",
        x"0A4D", x"0987", x"08CF", x"0823", x"0784", x"06F0", x"0666", x"05E6",
        x"0570", x"0503", x"049D", x"0440", x"03E9", x"0399", x"034F", x"030B",
        x"02CC", x"0293", x"025D", x"022C", x"01FF", x"01D6", x"01AF", x"018C",
        x"016C", x"014E", x"0133", x"0119", x"0102", x"00ED", x"00DA", x"00C8",
        x"00B7", x"00A8", x"009A", x"008E", x"0082", x"0077", x"006D", x"0064",
        x"005C", x"0054", x"004D", x"0047", x"0041", x"003C", x"0037", x"0032",
        x"002E", x"002A", x"0027", x"0024", x"0021", x"001E", x"001B", x"0019",
        x"0017", x"0015", x"0013", x"0012", x"0010", x"000F", x"000E", x"000D",
        x"000C", x"000B", x"000A", x"0009", x"0008", x"0007", x"0007", x"0006",
        x"0006", x"0005", x"0005", x"0004", x"0004", x"0004", x"0003", x"0003",
        x"0003", x"0003", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0000", x"0000", x"0000",
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
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");
    constant C_UGS : t_rom256 := (
        x"FE0B", x"FE21", x"FE38", x"FE4D", x"FE62", x"FE78", x"FE8C", x"FEA1",
        x"FEB4", x"FEC8", x"FEDA", x"FEEC", x"FEFD", x"FF0D", x"FF1D", x"FF2C",
        x"FF3A", x"FF48", x"FF54", x"FF61", x"FF6C", x"FF76", x"FF80", x"FF8A",
        x"FF93", x"FF9A", x"FFA3", x"FFA9", x"FFB0", x"FFB6", x"FFBC", x"FFC1",
        x"FFC7", x"FFCA", x"FFCF", x"FFD3", x"FFD7", x"FFD9", x"FFDD", x"FFE0",
        x"FFE2", x"FFE5", x"FFE6", x"FFE9", x"FFEB", x"FFED", x"FFEE", x"FFEF",
        x"FFF1", x"FFF2", x"FFF4", x"FFF4", x"FFF5", x"FFF6", x"FFF7", x"FFF8",
        x"FFF8", x"FFF9", x"FFFA", x"FFFA", x"FFFB", x"FFFB", x"FFFB", x"FFFC",
        x"FFFC", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFE", x"FFFE",
        x"FFFE", x"FFFE", x"FFFF", x"FFFE", x"FFFF", x"FFFF", x"FFFF", x"FFFF",
        x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"0000", x"FFFF", x"0000",
        x"FFFF", x"0000", x"FFFF", x"0000", x"0000", x"FFFF", x"0000", x"0000",
        x"0000", x"FFFF", x"0000", x"0000", x"0000", x"0000", x"0000", x"FFFF",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"FFFF", x"0000", x"0000", x"0000",
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
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");
    constant C_UA : t_rom256 := (
        x"8000", x"7C79", x"78F3", x"7570", x"71F3", x"6E7C", x"6B0E", x"67A9",
        x"644F", x"6101", x"5DC1", x"5A8F", x"576D", x"545C", x"515B", x"4E6C",
        x"4B90", x"48C7", x"4610", x"436D", x"40DE", x"3E62", x"3BF9", x"39A4",
        x"3763", x"3534", x"3318", x"310F", x"2F19", x"2D34", x"2B61", x"299E",
        x"27ED", x"264C", x"24BA", x"2338", x"21C5", x"2060", x"1F09", x"1DC0",
        x"1C84", x"1B54", x"1A31", x"1919", x"180D", x"170B", x"1614", x"1527",
        x"1444", x"136A", x"129A", x"11D1", x"1111", x"1059", x"0FA9", x"0F00",
        x"0E5E", x"0DC3", x"0D2E", x"0CA0", x"0C17", x"0B94", x"0B17", x"0A9F",
        x"0A2C", x"09BE", x"0955", x"08F0", x"088F", x"0832", x"07D9", x"0784",
        x"0733", x"06E5", x"069A", x"0652", x"060E", x"05CC", x"058D", x"0551",
        x"0517", x"04E0", x"04AB", x"0479", x"0448", x"041A", x"03ED", x"03C3",
        x"039A", x"0373", x"034D", x"0329", x"0307", x"02E6", x"02C7", x"02A9",
        x"028C", x"0270", x"0256", x"023C", x"0224", x"020D", x"01F7", x"01E1",
        x"01CD", x"01B9", x"01A7", x"0195", x"0184", x"0173", x"0163", x"0154",
        x"0146", x"0138", x"012B", x"011E", x"0112", x"0106", x"00FB", x"00F1",
        x"00E6", x"00DD", x"00D3", x"00CA", x"00C2", x"00BA", x"00B2", x"00AA",
        x"00A3", x"009C", x"0095", x"008F", x"0089", x"0083", x"007E", x"0078",
        x"0073", x"006E", x"006A", x"0065", x"0061", x"005D", x"0059", x"0055",
        x"0051", x"004E", x"004B", x"0048", x"0045", x"0042", x"003F", x"003C",
        x"003A", x"0037", x"0035", x"0033", x"0030", x"002E", x"002C", x"002B",
        x"0029", x"0027", x"0025", x"0024", x"0022", x"0021", x"001F", x"001E",
        x"001D", x"001C", x"001A", x"0019", x"0018", x"0017", x"0016", x"0015",
        x"0014", x"0014", x"0013", x"0012", x"0011", x"0010", x"0010", x"000F",
        x"000E", x"000E", x"000D", x"000D", x"000C", x"000C", x"000B", x"000B",
        x"000A", x"000A", x"0009", x"0009", x"0009", x"0008", x"0008", x"0008",
        x"0007", x"0007", x"0007", x"0006", x"0006", x"0006", x"0006", x"0005",
        x"0005", x"0005", x"0005", x"0004", x"0004", x"0004", x"0004", x"0004",
        x"0004", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003", x"0003",
        x"0003", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002", x"0002",
        x"0002", x"0002", x"0002", x"0002", x"0002", x"0001", x"0001", x"0001",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001",
        x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001", x"0001");
    constant C_UAS : t_rom256 := (
        x"FC79", x"FC7A", x"FC7D", x"FC83", x"FC89", x"FC92", x"FC9B", x"FCA6",
        x"FCB2", x"FCC0", x"FCCE", x"FCDE", x"FCEF", x"FCFF", x"FD11", x"FD24",
        x"FD37", x"FD49", x"FD5D", x"FD71", x"FD84", x"FD97", x"FDAB", x"FDBF",
        x"FDD1", x"FDE4", x"FDF7", x"FE0A", x"FE1B", x"FE2D", x"FE3D", x"FE4F",
        x"FE5F", x"FE6E", x"FE7E", x"FE8D", x"FE9B", x"FEA9", x"FEB7", x"FEC4",
        x"FED0", x"FEDD", x"FEE8", x"FEF4", x"FEFE", x"FF09", x"FF13", x"FF1D",
        x"FF26", x"FF30", x"FF37", x"FF40", x"FF48", x"FF50", x"FF57", x"FF5E",
        x"FF65", x"FF6B", x"FF72", x"FF77", x"FF7D", x"FF83", x"FF88", x"FF8D",
        x"FF92", x"FF97", x"FF9B", x"FF9F", x"FFA3", x"FFA7", x"FFAB", x"FFAF",
        x"FFB2", x"FFB5", x"FFB8", x"FFBC", x"FFBE", x"FFC1", x"FFC4", x"FFC6",
        x"FFC9", x"FFCB", x"FFCE", x"FFCF", x"FFD2", x"FFD3", x"FFD6", x"FFD7",
        x"FFD9", x"FFDA", x"FFDC", x"FFDE", x"FFDF", x"FFE1", x"FFE2", x"FFE3",
        x"FFE4", x"FFE6", x"FFE6", x"FFE8", x"FFE9", x"FFEA", x"FFEA", x"FFEC",
        x"FFEC", x"FFEE", x"FFEE", x"FFEF", x"FFEF", x"FFF0", x"FFF1", x"FFF2",
        x"FFF2", x"FFF3", x"FFF3", x"FFF4", x"FFF4", x"FFF5", x"FFF6", x"FFF5",
        x"FFF7", x"FFF6", x"FFF7", x"FFF8", x"FFF8", x"FFF8", x"FFF8", x"FFF9",
        x"FFF9", x"FFF9", x"FFFA", x"FFFA", x"FFFA", x"FFFB", x"FFFA", x"FFFB",
        x"FFFB", x"FFFC", x"FFFB", x"FFFC", x"FFFC", x"FFFC", x"FFFC", x"FFFC",
        x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFD", x"FFFE",
        x"FFFD", x"FFFE", x"FFFE", x"FFFD", x"FFFE", x"FFFE", x"FFFF", x"FFFE",
        x"FFFE", x"FFFE", x"FFFF", x"FFFE", x"FFFF", x"FFFE", x"FFFF", x"FFFF",
        x"FFFF", x"FFFE", x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"FFFF",
        x"0000", x"FFFF", x"FFFF", x"FFFF", x"FFFF", x"0000", x"FFFF", x"FFFF",
        x"0000", x"FFFF", x"0000", x"FFFF", x"0000", x"FFFF", x"0000", x"FFFF",
        x"0000", x"FFFF", x"0000", x"0000", x"FFFF", x"0000", x"0000", x"FFFF",
        x"0000", x"0000", x"FFFF", x"0000", x"0000", x"0000", x"FFFF", x"0000",
        x"0000", x"0000", x"FFFF", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"FFFF", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"FFFF", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"FFFF", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000",
        x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000");
    constant C_EXM : t_rom256 := (
        x"4000", x"402C", x"4059", x"4086", x"40B2", x"40DF", x"410C", x"4139",
        x"4167", x"4194", x"41C2", x"41EF", x"421D", x"424B", x"4279", x"42A7",
        x"42D5", x"4304", x"4332", x"4361", x"4390", x"43BF", x"43EE", x"441D",
        x"444C", x"447B", x"44AB", x"44DB", x"450A", x"453A", x"456A", x"459B",
        x"45CB", x"45FB", x"462C", x"465D", x"468D", x"46BE", x"46F0", x"4721",
        x"4752", x"4784", x"47B5", x"47E7", x"4819", x"484B", x"487D", x"48AF",
        x"48E2", x"4914", x"4947", x"497A", x"49AD", x"49E0", x"4A13", x"4A47",
        x"4A7A", x"4AAE", x"4AE2", x"4B16", x"4B4A", x"4B7E", x"4BB3", x"4BE7",
        x"4C1C", x"4C51", x"4C86", x"4CBB", x"4CF0", x"4D26", x"4D5B", x"4D91",
        x"4DC7", x"4DFD", x"4E33", x"4E69", x"4E9F", x"4ED6", x"4F0D", x"4F44",
        x"4F7B", x"4FB2", x"4FE9", x"5021", x"5058", x"5090", x"50C8", x"5100",
        x"5138", x"5171", x"51A9", x"51E2", x"521B", x"5254", x"528D", x"52C6",
        x"52FF", x"5339", x"5373", x"53AD", x"53E7", x"5421", x"545B", x"5496",
        x"54D1", x"550C", x"5547", x"5582", x"55BD", x"55F9", x"5634", x"5670",
        x"56AC", x"56E8", x"5725", x"5761", x"579E", x"57DB", x"5818", x"5855",
        x"5892", x"58CF", x"590D", x"594B", x"5989", x"59C7", x"5A05", x"5A44",
        x"5A82", x"5AC1", x"5B00", x"5B3F", x"5B7F", x"5BBE", x"5BFE", x"5C3E",
        x"5C7E", x"5CBE", x"5CFE", x"5D3F", x"5D80", x"5DC1", x"5E02", x"5E43",
        x"5E84", x"5EC6", x"5F08", x"5F4A", x"5F8C", x"5FCE", x"6011", x"6053",
        x"6096", x"60D9", x"611C", x"6160", x"61A3", x"61E7", x"622B", x"626F",
        x"62B4", x"62F8", x"633D", x"6382", x"63C7", x"640C", x"6451", x"6497",
        x"64DD", x"6523", x"6569", x"65AF", x"65F6", x"663D", x"6684", x"66CB",
        x"6712", x"675A", x"67A2", x"67E9", x"6832", x"687A", x"68C2", x"690B",
        x"6954", x"699D", x"69E6", x"6A30", x"6A7A", x"6AC4", x"6B0E", x"6B58",
        x"6BA2", x"6BED", x"6C38", x"6C83", x"6CCF", x"6D1A", x"6D66", x"6DB2",
        x"6DFE", x"6E4A", x"6E97", x"6EE4", x"6F30", x"6F7E", x"6FCB", x"7019",
        x"7066", x"70B4", x"7103", x"7151", x"71A0", x"71EF", x"723E", x"728D",
        x"72DD", x"732C", x"737C", x"73CC", x"741D", x"746D", x"74BE", x"750F",
        x"7560", x"75B2", x"7604", x"7655", x"76A8", x"76FA", x"774D", x"779F",
        x"77F2", x"7846", x"7899", x"78ED", x"7941", x"7995", x"79E9", x"7A3E",
        x"7A93", x"7AE8", x"7B3D", x"7B93", x"7BE8", x"7C3E", x"7C95", x"7CEB",
        x"7D42", x"7D99", x"7DF0", x"7E47", x"7E9F", x"7EF7", x"7F4F", x"7FA7");

    attribute no_rw_check : boolean;

    ----------------------------------------------------------------------
    -- controls: glided knobs and the per-field terms (all written in blanking by
    -- the sequencer and read directly by the pixel path)
    ----------------------------------------------------------------------
    type t_gl   is array(0 to 5) of unsigned(13 downto 0);      -- K1 K2 K3 K5 K6 P12, Q4
    signal s_gl   : t_gl := (to_unsigned(512 * 16, 14), to_unsigned(512 * 16, 14), to_unsigned(420 * 16, 14),
                             to_unsigned(720 * 16, 14), to_unsigned(1023 * 16, 14), to_unsigned(512 * 16, 14));
    signal s_gd, s_gs : signed(14 downto 0) := (others => '0');
    signal s_glide : std_logic := '1';                          -- S7
    signal s_spi, s_ink, s_fine, s_src : std_logic := '0';      -- S8 Spiral, S9 Ink, S10 Fine, S11 Osc
    signal s_ni    : unsigned(2 downto 0) := (others => '0');   -- K4: N = 3 4 5 6 8
    signal s_nv    : std_logic := '0';                          -- K4 latched once (the first field latches unconditionally)

    signal s_cxs   : signed(21 downto 0) := to_signed(960 * 256, 22);   -- centre (glided), Q8; power-up = 1080 centre
    signal s_cys   : signed(21 downto 0) := to_signed(540 * 256, 22);

    signal s_sq    : unsigned(6 downto 0) := (others => '0');
    -- per-field terms
    signal s_lm8   : unsigned(10 downto 0) := to_unsigned(623, 11);    -- log2 m, Q8 (m = bands per octave)
    signal s_m9    : unsigned(8 downto 0) := to_unsigned(331, 9);      -- its mantissa: m = m9 / 256 << mo
    signal s_mo    : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_nlb   : signed(11 downto 0) := to_signed(350 - 155, 12);  -- 6 octaves - log2 m (Q6) - log2(1 / ln 2)
    signal s_tq    : unsigned(9 downto 0) := to_unsigned(452, 10);     -- twist, turns per octave, Q12
    signal s_ahi   : signed(14 downto 0) := to_signed(4096 + 1024, 15);   -- alpha clamps, Q12
    signal s_alo   : signed(14 downto 0) := to_signed(-1840, 15);
    signal s_gc    : unsigned(7 downto 0) := to_unsigned(191, 8);      -- camera gain
    -- DRIFT (K5): band travel per field and the accumulated phase (Q20 bands, mod 4 bands)
    signal s_vneg, s_vzero : std_logic := '1';
    signal s_av    : unsigned(9 downto 0) := (others => '0');
    signal s_u     : unsigned(9 downto 0) := (others => '0');
    signal s_ls    : unsigned(9 downto 0) := (others => '0');
    signal s_ssh   : unsigned(3 downto 0) := (others => '0');
    signal s_spd   : unsigned(19 downto 0) := (others => '0');
    signal s_anim  : unsigned(21 downto 0) := (others => '0');

    -- sequencer ROM port (exp2 mantissa)
    signal s_ra    : unsigned(7 downto 0) := (others => '0');
    signal s_rq, s_rqf : std_logic_vector(15 downto 0) := (others => '0');

    -- serial multiplier (signed x 12 unsigned, 12 clocks): the centre
    signal sm_a    : signed(25 downto 0) := (others => '0');
    signal sm_b    : unsigned(11 downto 0) := (others => '0');
    signal sm_acc  : signed(25 downto 0) := (others => '0');
    signal sm_n    : unsigned(3 downto 0) := (others => '0');

    -- pixel-path views of the per-field terms
    signal w_cn    : unsigned(1 downto 0);                           -- corner up: the sector offset (quarters)
    signal w_ec    : unsigned(5 downto 0);                           -- e, Q5
    signal w_gb    : std_logic;                                      -- Grey Bold (full bands)

    ----------------------------------------------------------------------
    -- raster measurement
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_saw_avid   : std_logic := '0';
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wv    : std_logic := '0';
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    -- resolution scaling (as Hypnos): field-line step in x-pixel units (Q.8), the mode class
    -- (00 1080 / 01 720p / 10 SD) and log2(120) - log2(x step)
    signal s_ystp  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_mcls  : std_logic_vector(1 downto 0) := "00";
    signal s_lacc  : unsigned(19 downto 0) := (others => '0');
    signal s_lref  : signed(19 downto 0) := to_signed(C_LREF, 20);
    -- source field per mode: the vertical IIR shift
    signal s_ksh   : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal s_avid_q, s_avid_r : std_logic := '0';

    -- coordinate accumulators
    signal s_dx, s_dyc : signed(14 downto 0) := (others => '0');
    signal s_dyb      : signed(22 downto 0) := (others => '0');   -- next line's dy, Q.8
    signal s_gav_p    : std_logic := '0';
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- EBR delay lines (one write, one registered read each; a read never meets the write address)
    ----------------------------------------------------------------------
    type t_dl32 is array(0 to 255) of std_logic_vector(31 downto 0);
    type t_dl16 is array(0 to 255) of std_logic_vector(15 downto 0);
    type t_dl4  is array(0 to 255) of std_logic_vector(3 downto 0);
    signal dlh  : t_dl32;      -- log2 hi + quadrant flags
    signal dla  : t_dl32;      -- L & the filter-width base
    signal dlt  : t_dl16;      -- theta (Q16 turns), the spiral
    signal dls  : t_dl4;       -- syncs
    attribute no_rw_check of dlh : signal is true;
    attribute no_rw_check of dla : signal is true;
    attribute no_rw_check of dlt : signal is true;
    attribute no_rw_check of dls : signal is true;
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal dlh_q, dla_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dlt_q : std_logic_vector(15 downto 0) := (others => '0');
    signal dls_q, s_sy38, s_sy39, s_sy40, s_sy41 : std_logic_vector(3 downto 0) := "0110";

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    -- S1..S8: |dx|,|dy| -> hi/lo -> log2 hi, log2 lo (Q14, EBR PWL tables)
    signal s1_a, s1_b : unsigned(12 downto 0) := (others => '0');
    signal s1_dxs, s1_dys : std_logic := '0';
    signal s2_hi, s2_lo : unsigned(12 downto 0) := (others => '0');
    signal s2_sw, s2_dxs, s2_dys, s2_lz : std_logic := '0';
    signal s3_hi, s3_lo : unsigned(12 downto 0) := (others => '0');
    signal s3_mh, s3_ml : unsigned(3 downto 0) := (others => '0');
    signal s4_ih, s4_il : unsigned(7 downto 0) := (others => '0');
    signal s4_th, s4_tl, s5_th, s5_tl, s6_th, s6_tl : unsigned(3 downto 0) := (others => '0');
    signal s4_mh, s4_ml, s5_mh, s5_ml, s6_mh, s6_ml, s7_mh, s7_ml : unsigned(3 downto 0) := (others => '0');
    signal lvh_q, lsh_q, lvl_q, lsl_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s6_lvh, s6_lsh, s6_lvl, s6_lsl : std_logic_vector(15 downto 0) := (others => '0');
    signal s7_lph, s7_lpl : unsigned(10 downto 0) := (others => '0');
    signal s7_lvh, s7_lvl : unsigned(13 downto 0) := (others => '0');
    signal s8_lh, s8_ll : unsigned(17 downto 0) := (others => '0');
    type t_q is array(3 to 8) of std_logic_vector(3 downto 0);    -- sw, dxs, dys, lz
    signal qsr : t_q := (others => (others => '0'));

    -- S9..S15: u = log2(hi/lo) -> g(u) = 0.5 log2(1+4^-u), atan(2^-u)
    signal s9_ua  : unsigned(17 downto 0) := (others => '0');
    signal s10_ia : unsigned(7 downto 0) := (others => '0');
    signal s10_tu, s11_tu, s12_tu : unsigned(4 downto 0) := (others => '0');
    signal ug_q, ugs_q, ua_q, uas_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s12_g, s12_gs, s12_a, s12_as : std_logic_vector(15 downto 0) := (others => '0');
    signal s12_tu8 : signed(5 downto 0) := (others => '0');
    signal s13_gp  : signed(21 downto 0) := (others => '0');
    signal s13_ap  : signed(21 downto 0) := (others => '0');
    signal s13_g, s13_a : unsigned(15 downto 0) := (others => '0');
    signal s14_g   : unsigned(13 downto 0) := (others => '0');
    signal s14_a   : unsigned(15 downto 0) := (others => '0');
    signal s15_lr  : unsigned(17 downto 0) := (others => '0');   -- log2 r, Q14 (real pixels)
    signal s15_th  : unsigned(17 downto 0) := (others => '0');   -- polar angle, 2^18/turn

    -- S16..S20: L, the twist, psi -> the sector fold, alpha
    signal s16_Lp  : signed(18 downto 0) := (others => '0');     -- log2(r / 120 virtual px), Q14
    signal s16_Lq  : signed(16 downto 0) := (others => '0');     -- the same, Q12
    signal s16_nb  : signed(11 downto 0) := (others => '0');     -- minus the filter-width base (Q6)
    signal s16_th, s17_th, s18_th : unsigned(17 downto 0) := (others => '0');
    signal s17_yh, s17_yl : signed(22 downto 0) := (others => '0');
    signal s18_y   : signed(20 downto 0) := (others => '0');     -- twist * L, Q18 turns
    signal s19_u   : unsigned(17 downto 0) := (others => '0');
    signal s19_ar  : signed(17 downto 0) := (others => '0');     -- alpha before the clamp, Q12
    signal s20_addr : unsigned(8 downto 0) := (others => '0');
    signal s20_fr, s21_fr, s22_fr : unsigned(5 downto 0) := (others => '0');
    signal s20_sgn, s21_sgn : std_logic := '0';
    signal s20_al, s21_al, s22_al, s23_al, s24_al : signed(14 downto 0) := (others => '0');
    signal s20_tr, s21_tr, s22_tr, s23_tr : std_logic := '0';
    -- S21..S24: the polygon table ; the gradient's alpha tan a
    signal pv_q, ps_q, s22_pv, s22_ps : std_logic_vector(15 downto 0) := (others => '0');
    signal s22_ac  : signed(5 downto 0) := (others => '0');      -- +-alpha, Q4 (the sign of a)
    signal s23_ip  : unsigned(15 downto 0) := (others => '0');
    signal s23_pv  : signed(15 downto 0) := (others => '0');
    signal s23_qn  : signed(12 downto 0) := (others => '0');     -- alpha tan a, Q9
    signal s23_gp  : signed(6 downto 0) := (others => '0');      -- g P (Q6, from the segment start)
    signal s24_P   : signed(13 downto 0) := (others => '0');     -- P_N, Q12 (<= 0)
    signal s24_ra  : signed(9 downto 0) := (others => '0');      -- (R - 1) / e, Q6
    signal s24_abn, s25_abn : unsigned(8 downto 0) := (others => '0');   -- |B| = |alpha tan a|, Q6
    -- S25..S30: alpha * P, Phi, m * Phi ; |grad|
    signal s25_ph, s25_pl : signed(21 downto 0) := (others => '0');
    signal s25_er  : signed(13 downto 0) := (others => '0');
    signal s26_ap  : signed(16 downto 0) := (others => '0');     -- alpha P, Q14
    signal s26_arn : unsigned(11 downto 0) := (others => '0');
    signal s26_abn : unsigned(8 downto 0) := (others => '0');
    signal s27_phi : signed(19 downto 0) := (others => '0');
    signal s27_G   : unsigned(12 downto 0) := (others => '0');
    signal s27_nb, s28_nb : signed(11 downto 0) := (others => '0');
    signal s28_mph : signed(24 downto 0) := (others => '0');
    signal s28_mpl : signed(25 downto 0) := (others => '0');
    signal s28_lg  : unsigned(5 downto 0) := (others => '0');    -- log2 G: msb & 2-bit mantissa
    signal s29_ph  : unsigned(17 downto 0) := (others => '0');
    signal s29_nl  : signed(12 downto 0) := (others => '0');     -- - log2 w (cycles per pixel), Q6
    signal s29_wi, s30_wi, s31_wi : unsigned(4 downto 0) := (others => '0');   -- ri & rh
    -- S30..S36: coverage, tone
    signal s30_phase : unsigned(17 downto 0) := (others => '0');
    signal s30_cq  : unsigned(3 downto 0) := (others => '0');    -- the fade, 0 .. 15
    signal s30_h   : unsigned(15 downto 0) := (others => '0');
    signal s30_wh, s31_wh : unsigned(15 downto 0) := (others => '0');
    signal s31_s   : signed(16 downto 0) := (others => '0');
    signal s31_fa  : unsigned(7 downto 0) := (others => '0');    -- the fade ROM address: case & c
    signal fx_q, fy_q, s33_fx, s33_fy : std_logic_vector(15 downto 0) := (others => '0');
    signal s34_fx, s35_fx : unsigned(8 downto 0) := (others => '0');
    signal s34_fy  : signed(9 downto 0) := (others => '0');
    signal s32_hq, s32_lq, s33_hq, s33_lq : std_logic := '0';
    signal s32_rh  : std_logic := '0';
    signal s32_s2  : signed(12 downto 0) := (others => '0');
    signal s33_s3  : signed(12 downto 0) := (others => '0');
    signal s34_cov : unsigned(8 downto 0) := (others => '0');
    signal s35_pr  : signed(19 downto 0) := (others => '0');
    signal s36_tone : unsigned(8 downto 0) := (others => '0');
    -- S37..S41: output conditioning
    signal s37_m, s38_m, s39_m : unsigned(8 downto 0) := (others => '0');
    signal s39_f   : unsigned(10 downto 0) := (others => '0');
    signal s40_y   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal o_y     : unsigned(9 downto 0) := to_unsigned(64, 10);

    ----------------------------------------------------------------------
    -- source field (camera luma or the Osc lattice -> box -> vertical IIR)
    ----------------------------------------------------------------------
    signal s_osum  : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_col   : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram  : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q    : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt  : unsigned(7 downto 0) := (others => '0');
    signal s_bsum  : unsigned(14 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb    : unsigned(11 downto 0) := (others => '0');        -- box, Q4
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram  : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal s_eq    : unsigned(12 downto 0) := (others => '0');        -- IIR state, Q5
    signal lb_wa   : unsigned(9 downto 0) := (others => '0');
    signal lb_we   : std_logic := '0';
    signal s_qh, s_qc : unsigned(12 downto 0) := (others => '0');
    signal s_c11   : unsigned(10 downto 0) := (others => '0');
    signal s_camoff : unsigned(17 downto 0) := (others => '0');
    signal s_cpb   : unsigned(17 downto 0) := (others => '0');        -- camera - drift, Q16 bands

    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;

    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- the four grey steps (tone 0 .. 256)
    function f_tg(k : unsigned(1 downto 0)) return unsigned is
    begin
        case k is
            when "00"   => return to_unsigned(0, 9);
            when "01"   => return to_unsigned(85, 9);
            when "10"   => return to_unsigned(171, 9);
            when others => return to_unsigned(256, 9);
        end case;
    end function;

    -- Exact signed product a * m, radix-4 Booth on m: ceil(len(m)/2)
    -- partials of {0, +-a, +-2a} (negatives as one's complement, the +1s
    -- gathered in one correction word in a spare slot), balanced adds.
    function bmul(a : signed; m : signed) return signed is
        constant NA  : natural := a'length;
        constant NML : natural := m'length;
        constant NM  : natural := NML + (NML mod 2);
        constant NP  : natural := NA + NML;
        constant NPP : natural := NM / 2;
        type t_pp is array (0 to 7) of signed(NP - 1 downto 0);
        variable va   : signed(NA - 1 downto 0);
        variable vm   : signed(NM - 1 downto 0);
        variable me   : std_logic_vector(NM downto 0);
        variable a1, a2, mag, pp, corr : signed(NP - 1 downto 0);
        variable d    : std_logic_vector(2 downto 0);
        variable p    : t_pp;
    begin
        va := a;
        vm := resize(m, NM);
        me := std_logic_vector(vm) & '0';
        a1 := resize(va, NP);
        a2 := shift_left(a1, 1);
        corr := (others => '0');
        for i in 0 to 7 loop
            p(i) := (others => '0');
        end loop;
        for i in 0 to NPP - 1 loop
            d := me(2 * i + 2 downto 2 * i);
            case d is
                when "001" | "010" | "101" | "110" => mag := a1;
                when "011" | "100"                 => mag := a2;
                when others                        => mag := (others => '0');
            end case;
            if d(2) = '1' and d /= "111" then
                pp := not mag;
                corr(2 * i) := '1';
            else
                pp := mag;
            end if;
            p(i) := shift_left(pp, 2 * i);
        end loop;
        p(NPP) := corr;
        return ((p(0) + p(1)) + (p(2) + p(3))) + ((p(4) + p(5)) + (p(6) + p(7)));
    end function;

begin

    ------------------------------------------------------------------------
    -- raster measurement (Hypnos): the timing-ID steps, the line width, the field height.  The
    -- source avid is used as it is: the output is luma only (U = V = 512 on every pixel), so the
    -- encoders' Cb / Cr pairing (what Hypnos's generated avid protects) cannot swap anything.
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;

            -- per-mode steps from the timing ID (static while a mode runs).  s_ystp = the field
            -- line pitch in x-pixel units; s_ksh = the source field's vertical IIR shift.
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0          => s_ystp <= to_unsigned(569, 10); s_mcls <= "10"; s_ksh <= to_unsigned(3, 3);  -- 480i
                when 4          => s_ystp <= to_unsigned(288, 10); s_mcls <= "10"; s_ksh <= to_unsigned(4, 3);  -- 480p
                when 8          => s_ystp <= to_unsigned(480, 10); s_mcls <= "10"; s_ksh <= to_unsigned(3, 3);  -- 576i
                when 12         => s_ystp <= to_unsigned(240, 10); s_mcls <= "10"; s_ksh <= to_unsigned(4, 3);  -- 576p
                when 5 | 6 | 14 => s_ystp <= to_unsigned(256, 10); s_mcls <= "01"; s_ksh <= to_unsigned(4, 3);  -- 720p
                when 1 | 2 | 10 => s_ystp <= to_unsigned(512, 10); s_mcls <= "00"; s_ksh <= to_unsigned(4, 3);  -- 1080i
                when others     => s_ystp <= to_unsigned(256, 10); s_mcls <= "00"; s_ksh <= to_unsigned(5, 3);  -- 1080p
            end case;
            -- log2(120 virtual px) - log2(s): SD 1 octave, 720p log2(1.5) = 0.585 (Q14 9585)
            case s_mcls is
                when "10"   => s_lref <= to_signed(C_LREF - 16384, 20);
                when "01"   => s_lref <= to_signed(C_LREF - 9585, 20);
                when others => s_lref <= to_signed(C_LREF, 20);
            end case;

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                -- hysteresis: analog line lengths wander +/-1 px (the first line latches unconditionally)
                if s_xcnt > 16 and (s_Wv = '0' or s_xcnt > s_W + 2 or s_xcnt + 2 < s_W) then
                    s_W  <= s_xcnt;
                    s_Wv <= '1';
                end if;
                s_xcnt <= (others => '0');
                s_lacc <= s_lacc + s_ystp;
            end if;
            if s_vs_pulse = '1' then
                -- field height in x-pixel units (1080p: the line count)
                if s_lacc(19 downto 8) > 8 then
                    s_H <= s_lacc(19 downto 8);
                end if;
                s_lacc <= (others => '0');
            end if;

        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-field sequencer.  The field tick is the first vsync edge after active
    -- video (analog vsync serrates); it restarts the sequence, which runs in
    -- blanking, one small operation per state:
    --   8 .. 31   six knobs, 4 states each (3 used): the glided value moves only when the knob
    --             is 2 counts or more away (pot jitter: a hold, not an ease), then by
    --             1/16 of the gap per field (Drift 1/8; S7 On; K1 / K2 at 1/16 count = 1/2 px) or
    --             to the knob itself (S7 Off: no easing, used this field)
    --   32 .. 64  sides, bands per octave (exp2 ROM), the drift and its phase, twist and the alpha
    --             clamps from Evolve, the camera gain, the centre (serial multiplier)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_ch   : integer range 0 to 7;
        variable v_k    : signed(14 downto 0);
        variable v_sh   : signed(14 downto 0);
        variable v_mv   : boolean;
        variable v_reg  : unsigned(9 downto 0);
        variable v_wh   : unsigned(11 downto 0);
        variable v_g    : unsigned(13 downto 0);
        variable v_gl   : unsigned(13 downto 0);
        variable v_e    : unsigned(9 downto 0);
        variable v_z5   : unsigned(12 downto 0);
        variable v_zn   : unsigned(2 downto 0);
        variable v_zp   : unsigned(9 downto 0);
        variable v_v    : signed(10 downto 0);
        variable v_g6   : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.avid = '1' then s_saw_avid <= '1'; end if;

            -- serial units; the sequencer's loads (below) override these steps
            if sm_n /= 0 then                                     -- multiplier
                if sm_b(0) = '1' then sm_acc <= sm_acc + sm_a; end if;
                sm_a <= shift_left(sm_a, 1);
                sm_b <= shift_right(sm_b, 1);
                sm_n <= sm_n - 1;
            end if;
            if s_ssh /= 0 then                                    -- speed: mantissa << octave, 1 bit per clock
                s_spd <= shift_left(s_spd, 1);
                s_ssh <= s_ssh - 1;
            end if;

            v_ch := to_integer(s_sq(4 downto 2));
            if s_sq(3) = '0' then v_wh := s_W; else v_wh := s_H; end if;                                -- 48 vs 61
            -- the knob and its glided value for this channel (v_ch - 2: 0 K1, 1 K2, 2 K3, 3 K5, 4 K6, 5 P12)
            case v_ch is
                when 2      => v_reg := unsigned(registers_in(0)); v_gl := s_gl(0);
                when 3      => v_reg := unsigned(registers_in(1)); v_gl := s_gl(1);
                when 4      => v_reg := unsigned(registers_in(2)); v_gl := s_gl(2);
                when 5      => v_reg := unsigned(registers_in(4)); v_gl := s_gl(3);
                when 6      => v_reg := unsigned(registers_in(5)); v_gl := s_gl(4);
                when others => v_reg := unsigned(registers_in(7)); v_gl := s_gl(5);
            end case;
            -- K1 (state 34) or K2 (state 48) about the detent
            if s_sq(4) = '0' then v_g := s_gl(0); else v_g := s_gl(1); end if;
            v_k := signed('0' & v_g) - to_signed(8192, 15);                     -- Q4 counts about 512
            if abs(v_k) < C_DB * 16 then v_k := (others => '0'); end if;
            v_e := s_gl(5)(13 downto 4);                         -- Evolve e = P12 / 512, Q9
            v_g6 := s_gl(4)(13 downto 4);

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                s_vs_pulse <= '1';
                s_saw_avid <= '0';
                s_glide <= registers_in(6)(0);            -- S7 Glide
                s_spi   <= registers_in(6)(1);            -- S8 Arms: Rings / Spiral
                s_ink   <= registers_in(6)(2);            -- S9 Tone: Grey / Ink
                s_fine  <= registers_in(6)(3);            -- S10 Weight: Bold / Fine
                s_src   <= registers_in(6)(4);            -- S11 Source: Video / Osc
                s_sq    <= to_unsigned(8, 7);

            elsif s_sq /= 0 and data_in.avid = '0' then
                if s_sq = C_SQ_END then s_sq <= (others => '0'); else s_sq <= s_sq + 1; end if;

                if s_sq(6 downto 5) = "00" then
                    case to_integer(s_sq(1 downto 0)) is
                        when 1 =>
                            s_gd <= signed(shift_left(resize(v_reg, 15), 4)) - signed(resize(v_gl, 15));
                        when 2 =>
                            v_mv := s_gd > 31 or s_gd < -31 or v_reg = 0 or v_reg = 1023;
                            if v_ch = 5 then v_sh := shift_right(s_gd, 3);       -- Drift: 1/8 (Hypnos)
                            else             v_sh := shift_right(s_gd, 4); end if;
                            if not v_mv then                                    s_gs <= (others => '0');
                            elsif s_glide = '0' or v_sh = 0 then                s_gs <= s_gd;
                            else                                                s_gs <= v_sh; end if;
                        when 3 =>
                            v_sh := signed(resize(v_gl, 15)) + s_gs;
                            case v_ch is
                                when 2      => s_gl(0) <= unsigned(v_sh(13 downto 0));
                                when 3      => s_gl(1) <= unsigned(v_sh(13 downto 0));
                                when 4      => s_gl(2) <= unsigned(v_sh(13 downto 0));
                                when 5      => s_gl(3) <= unsigned(v_sh(13 downto 0));
                                when 6      => s_gl(4) <= unsigned(v_sh(13 downto 0));
                                when others => s_gl(5) <= unsigned(v_sh(13 downto 0));
                            end case;
                        when others => null;
                    end case;
                else
                    case to_integer(s_sq) is
                        when 32 =>
                            -- SIDES (K4): 5 zones (K4 * 5 / 1024), hysteresis of 8 counts with the two end
                            -- exceptions; the first field latches unconditionally
                            v_z5 := shift_left(resize(unsigned(registers_in(3)), 13), 2) + resize(unsigned(registers_in(3)), 13);
                            v_zn := v_z5(12 downto 10);
                            v_zp := v_z5(9 downto 0);
                            if s_nv = '0' or (v_zn /= s_ni and ((v_zp >= 40 and v_zp <= 983) or
                                              (v_zn = 0 and v_zp < 40) or (v_zn = 4 and v_zp > 983))) then
                                s_ni <= v_zn;
                                s_nv <= '1';
                            end if;
                            -- SCALE (K3): log2 m = 1 + K3 * 7 / 2048 octaves (Q8), 2 .. 22.6 bands per octave
                            s_lm8 <= to_unsigned(256, 11) + resize(s_gl(2)(13 downto 4), 11) - resize(s_gl(2)(13 downto 7), 11);
                        when 33 =>
                            -- DRIFT (K5, glided 1/8 like Hypnos's Speed): bipolar about 512
                            v_v := signed(resize(s_gl(3)(13 downto 4), 11)) - to_signed(512, 11);
                            s_vneg <= v_v(10);
                            s_av <= unsigned(resize(abs(v_v), 10));
                        when 34 =>
                            -- centre x = W / 2 + (K1 - 512) * W / 256 (detent +-14 counts)
                            sm_a <= resize(v_k, 26); sm_b <= s_W; sm_acc <= (others => '0'); sm_n <= to_unsigned(12, 4);
                            s_ra <= s_lm8(7 downto 0);
                        -- DRIFT decode (Hypnos): |v| < 24 still; else 2^-12 .. 2^-2 band per field, exponential
                        when 36 =>
                            if s_av < 24 then s_vzero <= '1'; s_u <= (others => '0');
                            else              s_vzero <= '0'; s_u <= s_av - 24; end if;
                        when 37 =>
                            s_m9 <= unsigned(s_rqf(14 downto 6));      -- the exp2 ROM read issued at 34
                            s_mo <= s_lm8(10 downto 8);
                            s_ls <= s_u + shift_right(s_u, 2) + shift_right(s_u, 4);      -- Q6 octaves, 0 .. 640
                            -- minus the filter-width base: 6 octaves (the gradient is Q6) - log2 m (Q6) - log2(1 / ln 2) (34)
                            s_nlb <= to_signed(350, 12) - signed(resize(s_lm8(10 downto 2), 12));
                        when 38 =>
                            s_ra <= s_ls(5 downto 0) & "00";
                        -- EVOLVE (P12): twist = 7/8 e / 8 turns per octave (~ e ln2 / 2pi) ; alpha clamps 1 + e / 4, -3.5 e / 8
                        when 39 =>
                            s_tq <= v_e - shift_right(v_e, 3);
                        when 40 =>
                            s_ahi <= to_signed(4096, 15) + signed(resize(shift_left(resize(v_e, 11), 1), 15));
                        when 41 =>
                            if s_vzero = '1' then s_spd <= (others => '0');
                            else                  s_spd <= resize(unsigned(s_rqf(14 downto 6)), 20); end if;   -- 2^-12 .. 2^-11 band
                            s_ssh <= s_ls(9 downto 6);
                        when 42 =>
                            s_alo <= signed(resize(shift_right(v_e, 1), 15)) - signed(resize(shift_left(resize(v_e, 12), 2), 15));
                        -- CAMERA (K6): 2.55 bands at white ; Osc 2 x (2.3 x the camera's swing)
                        when 46 =>
                            s_gc <= resize(shift_right(v_g6, 3), 8) + resize(shift_right(v_g6, 4), 8);
                        when 56 =>
                            -- the band phase: up = outward
                            if s_vneg = '1' then s_anim <= s_anim - resize(s_spd, 22);
                            else                 s_anim <= s_anim + resize(s_spd, 22); end if;
                        -- centre = W / 2 + (K1 - 512) * W / 256 from the GLIDED knob (Q4 counts: 1/16 count of
                        -- travel is half a pixel, so a glide moves smoothly and lands where the glide lands)
                        when 48 =>
                            s_cxs <= shift_left(resize(signed('0' & v_wh(11 downto 1)), 22), 8) + resize(shift_right(sm_acc, 4), 22);
                            sm_a <= resize(v_k, 26); sm_b <= s_H; sm_acc <= (others => '0'); sm_n <= to_unsigned(12, 4);
                        when 61 =>
                            s_cys <= shift_left(resize(signed('0' & v_wh(11 downto 1)), 22), 8) + resize(shift_right(sm_acc, 4), 22);
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process p_frame;

    with s_ni select w_cn <= "11" when "000", "01" when "010", "10" when "011", "00" when others;
    w_ec <= s_gl(5)(13 downto 8);
    w_gb <= not s_ink and not s_fine;

    ------------------------------------------------------------------------
    -- coordinate accumulators on the source avid.  dx loads -cx at each
    -- line start and steps per pixel; dy seeds at the first line of the frame
    -- and steps per line by s_ystp (Q.8 x-pixel units, seeded +0.5).
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            s_gav_p <= data_in.avid;
            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif data_in.avid = '1' and s_gav_p = '0' then
                if s_newframe = '1' then
                    v_dy0 := shift_left(-resize(s_cys(21 downto 8), 23), 8);
                    v_dy0(7) := '1';                                   -- +0.5
                    s_dyc <= v_dy0(22 downto 8);
                    s_dyb <= v_dy0 + signed(resize(s_ystp, 23));
                    s_newframe <= '0';
                else
                    s_dyc <= s_dyb(22 downto 8);
                    s_dyb <= s_dyb + signed(resize(s_ystp, 23));
                end if;
                s_dx <= -resize(s_cxs(21 downto 8), 15);
            elsif data_in.avid = '1' then
                s_dx <= s_dx + 1;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- EBR delay lines.  Write address = free-running counter; each read runs X behind it.
    ------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_wa <= s_wa + 1;
            dlh(to_integer(s_wa)) <= "0000000000" & qsr(8) & std_logic_vector(s8_lh);
            dlh_q  <= dlh(to_integer(s_wa - C_XH));
            dla(to_integer(s_wa)) <= '0' & std_logic_vector(s16_Lp) & std_logic_vector(s16_nb);
            dla_q  <= dla(to_integer(s_wa - C_XA));
            dlt(to_integer(s_wa)) <= std_logic_vector(s16_th(17 downto 2));
            dlt_q  <= dlt(to_integer(s_wa - C_XT));
            dls(to_integer(s_wa)) <= data_in.avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            dls_q  <= dls(to_integer(s_wa - C_XS));
        end if;
    end process p_dl;

    -- table ROMs (EBR): every address is a register, every read is registered
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lvh_q <= C_LOGV(to_integer(s4_ih));
            lsh_q <= C_LOGS(to_integer(s4_ih));
            lvl_q <= C_LOGV(to_integer(s4_il));
            lsl_q <= C_LOGS(to_integer(s4_il));
            ug_q  <= C_UG(to_integer(s10_ia));
            ugs_q <= C_UGS(to_integer(s10_ia));
            ua_q  <= C_UA(to_integer(s10_ia));
            uas_q <= C_UAS(to_integer(s10_ia));
            pv_q  <= C_PV(to_integer(s20_addr));
            ps_q  <= C_PS(to_integer(s20_addr));
            fx_q  <= C_FX(to_integer(s31_fa));
            fy_q  <= C_FY(to_integer(s31_fa));
            s_rq  <= C_EXM(to_integer(s_ra));
            s_rqf <= s_rq;
        end if;
    end process p_rom;

    ------------------------------------------------------------------------
    -- S1..S8: |dx|,|dy| (13-bit), hi = max, lo = min, and their log2 (Q14):
    -- priority-encode, normalise, 256-segment PWL from EBR.   (Hypnos)
    ------------------------------------------------------------------------
    p_logs : process(clk)
        variable va, vb : unsigned(14 downto 0);
        variable vh, vl : unsigned(12 downto 0);
        variable mh, ml : integer range 0 to 12;
    begin
        if rising_edge(clk) then
            -- S1
            va := unsigned(abs(s_dx));
            vb := unsigned(abs(s_dyc));
            s1_a <= va(12 downto 0);              -- |dx|, |dy| < 8192: the centre stays within 2 screens
            s1_b <= vb(12 downto 0);
            s1_dxs <= s_dx(14);
            s1_dys <= s_dyc(14);
            -- S2
            if s1_a >= s1_b then s2_hi <= s1_a; s2_lo <= s1_b; s2_sw <= '0';
            else                 s2_hi <= s1_b; s2_lo <= s1_a; s2_sw <= '1'; end if;
            if s1_a = 0 or s1_b = 0 then s2_lz <= '1'; else s2_lz <= '0'; end if;
            s2_dxs <= s1_dxs;
            s2_dys <= s1_dys;
            -- S3: msb (hi >= 1 always: the exact centre reads as 1)
            vh := s2_hi; if vh = 0 then vh := to_unsigned(1, 13); end if;
            vl := s2_lo; if vl = 0 then vl := to_unsigned(1, 13); end if;
            mh := 0; ml := 0;
            for i in 0 to 12 loop
                if vh(i) = '1' then mh := i; end if;
                if vl(i) = '1' then ml := i; end if;
            end loop;
            s3_hi <= vh; s3_lo <= vl;
            s3_mh <= to_unsigned(mh, 4); s3_ml <= to_unsigned(ml, 4);
            -- S4: normalise -> 8-bit segment index + 4-bit fraction
            vh := shift_left(s3_hi, 12 - to_integer(s3_mh));
            vl := shift_left(s3_lo, 12 - to_integer(s3_ml));
            s4_ih <= vh(11 downto 4); s4_th <= vh(3 downto 0);
            s4_il <= vl(11 downto 4); s4_tl <= vl(3 downto 0);
            s4_mh <= s3_mh; s4_ml <= s3_ml;
            -- S5, S6 (EBR read, fabric register)
            s5_th <= s4_th; s5_tl <= s4_tl; s5_mh <= s4_mh; s5_ml <= s4_ml;
            s6_th <= s5_th; s6_tl <= s5_tl; s6_mh <= s5_mh; s6_ml <= s5_ml;
            s6_lvh <= lvh_q; s6_lsh <= lsh_q; s6_lvl <= lvl_q; s6_lsl <= lsl_q;
            -- S7
            s7_lph <= resize(unsigned(s6_lsh(6 downto 0)) * s6_th, 11);
            s7_lpl <= resize(unsigned(s6_lsl(6 downto 0)) * s6_tl, 11);
            s7_lvh <= unsigned(s6_lvh(13 downto 0));
            s7_lvl <= unsigned(s6_lvl(13 downto 0));
            s7_mh <= s6_mh; s7_ml <= s6_ml;
            -- S8: log2 = msb + PWL(mantissa), Q14
            s8_lh <= shift_left(resize(s7_mh, 18), 14) + resize(s7_lvh, 18) + resize(shift_right(s7_lph, 4), 18);
            s8_ll <= shift_left(resize(s7_ml, 18), 14) + resize(s7_lvl, 18) + resize(shift_right(s7_lpl, 4), 18);

            -- quadrant flags ride along
            qsr(3) <= s2_sw & s2_dxs & s2_dys & s2_lz;
            for i in 4 to 8 loop qsr(i) <= qsr(i - 1); end loop;
        end if;
    end process p_logs;

    ------------------------------------------------------------------------
    -- S9..S15: u = log2(hi/lo) >= 0 ;  log2 r = log2 hi + 0.5 log2(1 + 4^-u)
    -- and the octant angle atan(2^-u), both from EBR tables (1/16 octave
    -- steps, interpolated).   (Hypnos)
    ------------------------------------------------------------------------
    p_polar : process(clk)
        variable vu   : unsigned(17 downto 0);
        variable vmag : unsigned(17 downto 0);
        variable q    : std_logic_vector(3 downto 0);
        variable vlh  : unsigned(17 downto 0);
        variable vth  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- S9
            vu := s8_lh - s8_ll;
            -- lo = 0 (on an axis): u -> the table end (g ~ 0, angle ~ 0)
            if qsr(8)(0) = '1' then vu := to_unsigned(16 * 16384 - 1, 18); end if;
            s9_ua <= vu;
            -- S10: 1/16-octave segment + fraction
            s10_ia <= s9_ua(17 downto 10);
            s10_tu <= s9_ua(9 downto 5);
            -- S11, S12 (EBR read, fabric register)
            s11_tu <= s10_tu;
            s12_tu <= s11_tu;
            s12_tu8 <= signed(resize(s11_tu, 6));
            s12_g <= ug_q; s12_gs <= ugs_q; s12_a <= ua_q; s12_as <= uas_q;
            -- S13: interpolation products (slopes are negative)
            s13_gp <= resize(bmul(signed(s12_gs(10 downto 0)), s12_tu8), 22);
            s13_ap <= resize(bmul(signed(s12_as(10 downto 0)), s12_tu8), 22);
            s13_g  <= unsigned(s12_g);
            s13_a  <= unsigned(s12_a);
            -- S14
            s14_g <= resize(unsigned(signed(resize(s13_g, 17)) + resize(shift_right(s13_gp, 5), 17)), 14);
            s14_a <= resize(unsigned(signed(resize(s13_a, 17)) + resize(shift_right(s13_ap, 5), 17)), 16);
            -- S15: log2 r ; full polar angle from the octant angle + swap + signs
            vlh := unsigned(dlh_q(17 downto 0));
            s15_lr <= vlh + resize(s14_g, 18);
            q := dlh_q(21 downto 18);
            if q(3) = '1' then vmag := to_unsigned(65536, 18) - resize(s14_a, 18);
            else               vmag := resize(s14_a, 18); end if;
            if    q(2) = '0' and q(1) = '0' then vth := vmag;
            elsif q(2) = '1' and q(1) = '0' then vth := to_unsigned(131072, 18) - vmag;
            elsif q(2) = '1' and q(1) = '1' then vth := to_unsigned(131072, 18) + vmag;
            else                                 vth := to_unsigned(0, 18) - vmag; end if;
            s15_th <= vth;
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- S16..S28: the shape.  L, the twist Y = tq * L, psi = theta + Y, the sector
    -- coordinate u = frac(N psi) folded to v = |2u - 1| (one's complement), the
    -- polygon table P_N(v) (interpolated), alpha = clamp(1 - 6.25 Y), alpha * P,
    -- Phi = L - alpha P ; and the gradient terms alpha tan a, R, B.
    ------------------------------------------------------------------------
    p_shape : process(clk)
        variable vl  : signed(19 downto 0);
        variable vy  : signed(27 downto 0);
        variable vb  : unsigned(17 downto 0);
        variable vs  : std_logic;
        variable vp  : unsigned(17 downto 0);
        variable vu  : unsigned(17 downto 0);
        variable vd  : signed(23 downto 0);
        variable vat : unsigned(16 downto 0);
        variable vpc : signed(11 downto 0);
        variable vq6 : signed(9 downto 0);
        variable va  : signed(28 downto 0);
        variable vrn : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- S16: L (Q14, Q12) ; minus the filter-width base ; theta
            vl := signed(resize(s15_lr, 20)) - s_lref;
            s16_Lp <= vl(18 downto 0);
            s16_Lq <= vl(18 downto 2);
            s16_nb <= s_nlb + signed(resize(s15_lr(17 downto 8), 12));
            s16_th <= s15_th;
            -- S17: Y = tq * L, split on tq
            s17_yh <= bmul(s16_Lq, signed('0' & s_tq(9 downto 5)));
            s17_yl <= bmul(s16_Lq, signed('0' & s_tq(4 downto 0)));
            s17_th <= s16_th;
            -- S18: Q24 -> Q18 turns
            vy := shift_left(resize(s17_yh, 28), 5) + resize(s17_yl, 28);
            s18_y <= vy(26 downto 6);
            s18_th <= s17_th;
            -- S19: u = N (theta + Y) + the corner offset (N = 4 +- the select: 3 4 5 6 8) ;
            -- alpha = 1 - 6 Y
            vp := s18_th + unsigned(s18_y(17 downto 0));
            case s_ni is
                when "000"  => vb := vp; vs := '1';
                when "001"  => vb := (others => '0'); vs := '0';
                when "010"  => vb := vp; vs := '0';
                when "011"  => vb := shift_left(vp, 1); vs := '0';
                when others => vb := shift_left(vp, 2); vs := '0';
            end case;
            vu := f_addsub(shift_left(vp, 2), vb, vs);
            vu(17 downto 16) := vu(17 downto 16) + w_cn;
            s19_u <= vu;
            vd := (to_signed(262144, 24) - shift_left(resize(s18_y, 24), 2)) - shift_left(resize(s18_y, 24), 1);
            s19_ar <= vd(23 downto 6);
            -- S20: fold v = |2u - 1| (one's complement below the middle) -> 64 segments x 6-bit fraction ;
            -- alpha clamp and the "in transition" flag
            if s19_u(17) = '1' then vat := s19_u(16 downto 0); else vat := not s19_u(16 downto 0); end if;
            s20_addr <= s_ni & vat(16 downto 11);
            s20_fr   <= vat(10 downto 5);
            s20_sgn  <= s19_u(17);
            if s19_ar <= resize(s_alo, 18) then
                s20_al <= s_alo; s20_tr <= '0';
            elsif s19_ar >= resize(s_ahi, 18) then
                s20_al <= s_ahi; s20_tr <= '0';
            else
                s20_al <= s19_ar(14 downto 0); s20_tr <= '1';
            end if;
            -- S21 (EBR read)
            s21_fr <= s20_fr; s21_sgn <= s20_sgn; s21_al <= s20_al; s21_tr <= s20_tr;
            -- S22 (fabric register) ; +-alpha (Q4) by the side of the sector middle
            s22_pv <= pv_q; s22_ps <= ps_q;
            if s21_sgn = '1' then s22_ac <= s21_al(13 downto 8);
            else                  s22_ac <= to_signed(0, 6) - s21_al(13 downto 8); end if;
            s22_fr <= s21_fr; s22_al <= s21_al; s22_tr <= s21_tr;
            -- S23: interpolation product ; alpha tan a (Q5 x Q4) ; g P = 11 P / 16 from the segment start (Q6)
            s23_ip <= unsigned(s22_ps(15 downto 6)) * s22_fr;
            s23_pv <= signed(s22_pv);
            s23_qn <= bmul(signed('0' & s22_ps(5 downto 0)), s22_ac);
            vpc := resize(signed(s22_pv(15 downto 8)), 12);
            vpc := shift_left(vpc, 3) + shift_left(vpc, 1) + vpc;
            s23_gp <= vpc(10 downto 4);
            s23_al <= s22_al; s23_tr <= s22_tr;
            -- S24: P (Q12) ; R' = g P (in transition) - alpha tan a ; |B| = |alpha tan a| (Rings and Spiral:
            -- the spiral's own angular term is under 0.44 of the radial one and is left out)
            vl := resize(s23_pv, 20) + signed(resize(s23_ip(15 downto 6), 20));
            s24_P <= vl(15 downto 2);
            vq6 := resize(s23_qn(12 downto 3), 10);
            if s23_tr = '1' then s24_ra <= resize(s23_gp, 10) - vq6;
            else                 s24_ra <= to_signed(0, 10) - vq6; end if;
            vq6 := abs(vq6);
            s24_abn <= unsigned(vq6(8 downto 0));
            s24_al <= s23_al;
            -- S25: alpha * P split on alpha (Q12 x Q12) ; e R' (e to 1/4)
            s25_ph <= bmul(s24_P, s24_al(14 downto 7));
            s25_pl <= bmul(s24_P, signed('0' & s24_al(6 downto 0)));
            s25_er <= bmul(s24_ra, signed('0' & w_ec(5 downto 3)));
            s25_abn <= s24_abn;
            -- S26: alpha P (Q14) ; |R| = |1 + e R'|
            va := shift_left(resize(s25_ph, 29), 7) + resize(s25_pl, 29);
            s26_ap <= va(26 downto 10);
            vrn := abs(to_signed(64, 13) + resize(s25_er(13 downto 2), 13));
            s26_arn <= unsigned(vrn(11 downto 0));
            s26_abn <= s25_abn;
            -- S27: Phi = L - alpha P ; G = max + min / 2
            s27_phi <= resize(signed(dla_q(30 downto 12)), 20) - resize(s26_ap, 20);
            s27_nb  <= signed(dla_q(11 downto 0));
            if s26_arn >= resize(s26_abn, 12) then
                s27_G <= resize(s26_arn, 13) + resize(shift_right(s26_abn, 1), 13);
            else
                s27_G <= resize(s26_abn, 13) + resize(shift_right(s26_arn, 1), 13);
            end if;
        end if;
    end process p_shape;

    ------------------------------------------------------------------------
    -- S28..S30: the phase (m * Phi + arms * theta + camera - drift, Q16 bands mod 4) and the
    -- filter width: |grad| = m / (r ln 2) * |(R, B)|, |(R, B)| ~ max + min / 2, log2 (Q6, msb and a
    -- 2-bit mantissa) rounded UP to a half octave so the divide is a shift (Hypnos):
    -- 1 / w = 2^ri {1, sqrt2}^rh, ri & rh = clamp(-log2 w / 32, 0, 31).  The fade: 1 at bands (or
    -- Ink / Fine periods) >= 4 px, 0 at <= 2 px.
    ------------------------------------------------------------------------
    p_phase : process(clk)
        variable vm  : signed(25 downto 0);
        variable vt  : unsigned(17 downto 0);
        variable vg  : unsigned(12 downto 0);
        variable vpp : integer range 0 to 12;
        variable vm2 : unsigned(1 downto 0);
        variable vnl : signed(12 downto 0);
        variable vc  : signed(12 downto 0);
        variable vi  : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            -- S28: m * Phi split on m9 ; log2 G = msb & the next two bits (G = 0 reads as 1)
            s28_mph <= bmul(s27_phi, signed('0' & s_m9(8 downto 5)));
            s28_mpl <= bmul(s27_phi, signed('0' & s_m9(4 downto 0)));
            vg := s27_G; if vg = 0 then vg := to_unsigned(1, 13); end if;
            vpp := 0;
            for i in 0 to 12 loop
                if vg(i) = '1' then vpp := i; end if;
            end loop;
            vg := shift_left(vg, 12 - vpp);
            vm2 := vg(11 downto 10);
            s28_lg <= to_unsigned(vpp, 4) & vm2;
            s28_nb <= s27_nb;
            -- S29: Phi * m9 << mo -> Q16 bands (mod 4) ; - log2 w = - base - log2 G (64 msb + 16 m2 + 8) ;
            -- the half-octave index
            vm := shift_left(resize(s28_mph, 26), 5) + resize(s28_mpl, 26);
            case s_mo is
                when "001"  => s29_ph <= unsigned(vm(22 downto 5));
                when "010"  => s29_ph <= unsigned(vm(21 downto 4));
                when "011"  => s29_ph <= unsigned(vm(20 downto 3));
                when others => s29_ph <= unsigned(vm(19 downto 2));
            end case;
            vnl := resize(s28_nb, 13) - signed(resize(s28_lg & "1000", 13));
            s29_nl <= vnl;
            if vnl < 0 then s29_wi <= (others => '0');
            elsif vnl >= 1024 then s29_wi <= (others => '1');
            else s29_wi <= unsigned(vnl(9 downto 5)); end if;
            -- S30: + arms * theta (8 arms in Grey, 4 in Ink: whole bands per turn, seamless) + camera -
            -- drift ; the window and the line half widths (tables) ; the fade
            case std_logic_vector'(s_spi & s_ink) is
                when "10"   => vt := shift_left(resize(unsigned(dlt_q), 18), 3);
                when "11"   => vt := shift_left(resize(unsigned(dlt_q), 18), 2);
                when others => vt := (others => '0');
            end case;
            s30_phase <= s29_ph + vt + s_cpb;
            vi := to_integer(s29_wi);
            s30_wh <= C_WH(vi);
            if w_gb = '1' then    s30_h <= to_unsigned(32768, 16);
            elsif s_fine = '1' then s30_h <= C_HF(vi);
            else                  s30_h <= to_unsigned(16384, 16); end if;
            if w_gb = '1' then vc := s29_nl - to_signed(64, 13);
            else               vc := s29_nl - to_signed(128, 13); end if;
            if vc < 0 then s30_cq <= (others => '0');
            elsif vc >= 60 then s30_cq <= to_unsigned(15, 4);
            else s30_cq <= unsigned(vc(5 downto 2)); end if;
            s30_wi <= s29_wi;
        end if;
    end process p_phase;

    ------------------------------------------------------------------------
    -- S31..S36: the tone.  Signed distance s = h - d into the band (Grey Bold: the whole band,
    -- h = 1/2) or the line (Ink Bold h = 1/4; Fine h = 1/12, never under 2 px: h >= w, up to
    -- 1/4); coverage = 1/2 + s / w, a one-pixel ramp; tone = X + Y * coverage, X and Y from the fade
    -- ROM (the band's step against its neighbour's, or the line over black, pulled toward the flat
    -- grey by the fade).
    ------------------------------------------------------------------------
    p_tone : process(clk)
        variable vd  : unsigned(15 downto 0);
        variable vs2 : signed(35 downto 0);
        variable vc  : signed(12 downto 0);
        variable vcs : unsigned(3 downto 0);
        variable vo  : signed(10 downto 0);
        variable vsh : integer range 0 to 16;
    begin
        if rising_edge(clk) then
            -- S31: s = h - d (d: distance of the phase from the band middle) ; the fade ROM address
            if s30_phase(15) = '1' then vd := '0' & s30_phase(14 downto 0);
            else                        vd := '0' & (not s30_phase(14 downto 0)); end if;
            s31_s <= signed(resize(s30_h, 17)) - signed(resize(vd, 17));
            s31_wh <= s30_wh;
            s31_wi <= s30_wi;
            if w_gb = '1' then    vcs := '0' & s30_phase(17 downto 15);
            elsif s_ink = '0' then vcs := "10" & s30_phase(17 downto 16);
            else                  vcs := "1100"; end if;
            s31_fa <= vcs & s30_cq;
            -- S32: outside the window either way ; bits 17 .. 5 of s << (ri + rh)   (fade ROM: EBR read)
            if s31_s >= signed(resize(s31_wh, 17)) then s32_hq <= '1'; else s32_hq <= '0'; end if;
            if s31_s <= -signed(resize(s31_wh, 17)) then s32_lq <= '1'; else s32_lq <= '0'; end if;
            vsh := to_integer(s31_wi(4 downto 1)) + to_integer(s31_wi(0 downto 0));
            vs2 := shift_left(resize(s31_s, 36), vsh);
            s32_s2 <= vs2(17 downto 5);
            s32_rh <= s31_wi(0);
            -- S33: * 0.70703 on a half-octave step   (fade ROM: fabric register)
            if s32_rh = '1' then
                s33_s3 <= shift_right(s32_s2, 1) + shift_right(s32_s2, 3) + shift_right(s32_s2, 4) + shift_right(s32_s2, 6);
            else
                s33_s3 <= s32_s2;
            end if;
            s33_hq <= s32_hq; s33_lq <= s32_lq;
            s33_fx <= fx_q; s33_fy <= fy_q;
            -- S34: coverage 0 .. 256
            vc := shift_right(s33_s3, 3) + to_signed(128, 13);
            if s33_hq = '1' then s34_cov <= to_unsigned(256, 9);
            elsif s33_lq = '1' or vc < 0 then s34_cov <= (others => '0');
            elsif vc > 256 then s34_cov <= to_unsigned(256, 9);
            else s34_cov <= unsigned(vc(8 downto 0)); end if;
            s34_fx <= unsigned(s33_fx(8 downto 0));
            s34_fy <= signed(s33_fy(9 downto 0));
            -- S35: Y * coverage
            s35_pr <= bmul(s34_fy, signed('0' & s34_cov));
            s35_fx <= s34_fx;
            -- S36: tone = X + Y * coverage / 256
            vo := signed(resize(s35_fx, 11)) + resize(s35_pr(18 downto 8), 11);
            s36_tone <= unsigned(vo(8 downto 0));
        end if;
    end process p_tone;

    ------------------------------------------------------------------------
    -- S37..S41: output conditioning.  [1 2 1] / 4 with replicated picture edges, luma
    -- 11/16 toward black (white = 768), blanking gate.  U = V = 512 always.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable vl, vr : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- S37..S39: three-pixel window; the centre is s38_m
            s37_m <= s36_tone; s38_m <= s37_m; s39_m <= s38_m;
            s_sy38 <= dls_q; s_sy39 <= s_sy38; s_sy40 <= s_sy39; s_sy41 <= s_sy40;
            if dls_q(3) = '1' then vr := s37_m; else vr := s38_m; end if;
            if s_sy39(3) = '1' then vl := s39_m; else vl := s38_m; end if;
            s39_f <= resize(vl, 11) + shift_left(resize(s38_m, 11), 1) + resize(vr, 11);
            -- S40: Y = 64 + f * 11 / 16  (= the [1 2 1] / 4 mean, scaled toward black: white = 768)
            s40_y <= to_unsigned(64, 10) + resize(shift_right(s39_f, 1), 10) + resize(shift_right(s39_f, 3), 10)
                     + resize(shift_right(s39_f, 4), 10);
            -- S41: blanking gate on the latency-aligned avid
            if s_sy40(3) = '1' then o_y <= s40_y; else o_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    ------------------------------------------------------------------------
    -- SOURCE FIELD (the shell's [D], lifted from Kinetic riley via Hardline blaze): camera luma,
    -- or the Osc lattice (512 virtual px triangles, static: only Drift moves the picture), -> box
    -- (128 px at 1080, 64 px otherwise; edge-replicated and reloaded at every line start) -> vertical
    -- IIR in a half-resolution line store -> * K6 -> a phase (bands).  Free-running,
    -- tapped shallow: the bend sits a few pixels beside the subject, which is never shown.
    -- Black input = 0 = the clean figure.
    ------------------------------------------------------------------------
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    p_src : process(clk)
        variable v_l   : unsigned(7 downto 0);
        variable v_ox, v_oy : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d   : signed(14 downto 0);
        variable v_st  : signed(14 downto 0);
        variable v_c   : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: two triangle waves from the pixel and line counters, 512 virtual px (768 at 720p)
            if s_mcls = "10" then v_ox := s_xcnt(7 downto 0); v_oy := s_lacc(15 downto 8);
            else                  v_ox := s_xcnt(8 downto 1); v_oy := s_lacc(16 downto 9); end if;
            s_osum <= resize(f_tri7(v_ox), 8) + resize(f_tri7(v_oy), 8);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 128 (64) samples; the samples before the line start count as the first one
            if s_sv = '1' then
                bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else
                bx_wp <= (others => '0');
                if s_mcls = "00" then bx_rp <= to_unsigned(129, 8); else bx_rp <= to_unsigned(193, 8); end if;
            end if;
            if (s_mcls = "00" and s_bcnt < 128) or (s_mcls /= "00" and s_bcnt < 64) then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                if s_mcls = "00" then s_bsum <= shift_left(resize(s_sl, 15), 7);
                else                  s_bsum <= shift_left(resize(s_sl, 15), 6); end if;
                s_bcnt <= to_unsigned(1, 8);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + resize(s_sl, 15) - resize(v_tap, 15);
                if s_bcnt /= 128 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on the odd columns: q += (box - q) >> ksh   (box Q4, q Q5)
            if s_mcls = "00" then s_eb <= s_bsum(14 downto 3); else s_eb <= s_bsum(13 downto 2); end if;
            s_eq <= lb_q1(12 downto 0);
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb & '0', 15)) - signed(resize(s_eq, 15));
            v_st := shift_right(v_d, to_integer(s_ksh));
            v_d := signed(resize(s_eq, 15)) + v_st;
            lb_wd <= "000" & unsigned(v_d(12 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the stream: the newest column's value of the half-resolution store, held two pixels -> * gain
            -- -> - the drift phase
            if lb_we = '1' then s_qh <= lb_wd(12 downto 0); end if;
            s_qc <= s_qh;
            s_c11 <= resize(shift_right(s_qc, 2), 11);
            v_c := s_c11 * s_gc;
            if s_src = '1' then s_camoff <= v_c(17 downto 0); else s_camoff <= v_c(18 downto 1); end if;   -- Osc: 2 x
            s_cpb <= s_camoff - s_anim(21 downto 4);
        end if;
    end process p_src;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(C_MID);
    data_out.v       <= std_logic_vector(C_MID);
    data_out.avid    <= s_sy41(3);
    data_out.hsync_n <= s_sy41(2);
    data_out.vsync_n <= s_sy41(1);
    data_out.field_n <= s_sy41(0);

end architecture whorl;
