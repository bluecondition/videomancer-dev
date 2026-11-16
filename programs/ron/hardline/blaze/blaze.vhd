-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- blaze.vhd  (Hardline set, v0.1.0 -- Bridget Riley's "Blaze": a sunburst whose rays zigzag ring by ring)
--
-- Black and white only (U = V = 512 on every pixel).  Polar chassis = Hypnos v1.1.0: timing, generated
-- avid, centre glide, the log-domain polar engine (S1 .. S15) and the analytic one-pixel box-filter
-- anti-aliasing.  The field is self-similar to the centre:
--
--   v    = m * log2(r / 512)                  m = rings per octave (K3 Period, 0.5 .. 4)
--   PHI  = Rays * theta / 2pi + Twist * (zig(v) - 1/2) + camera          S8 Form = Zigzag
--        = Rays * theta / 2pi + Twist * v + camera                        S8 Form = Checker (a log spiral)
--   zig  = triangle in 0 .. 1 over each ring, apex at Skew (K5): t / s below it, (1 - t) / (1 - s) above
--   ink  = frac(PHI) < duty   (S10 Weight: 1/2 or a thin line)            S9 Shade = Flat
--        = frac(PHI) as a sawtooth ramp (S10: 4 flat steps / smooth)      S9 Shade = Shade
--   Checker: ink XOR (frac(v) < Skew), the rings counterchange the rays.
--   camera = box-filtered luma (or the Osc lattice) * K6, added to PHI (up to 2 periods at 100 %).
--
-- Both gradient terms fall as 1 / r, so the filter width is log2 w = LG(segment) - log2 r with one
-- per-field constant per zigzag segment; the centre fades to flat grey on its own.
--
-- Pipeline (stage N = N registers after s_dx; one latency, 40 clocks, in every mode):
--   S1 .. S15   |dx|, |dy| -> log2 hi, log2 lo -> log2 r, theta                       (Hypnos)
--   S16         L = log2 r - LREF ; Rays * theta ; log2 r (Q6)
--   S17, S18    v = m * L
--   S19 .. S21  zigzag segment, X * Y (t / s scaled by Twist, or L * Twist * m)
--   S22         PHI = Rays * theta + z + (camera + offset)
--   S20 .. S25  log2 w -> widen near Nyquist / pick the 4-step phase -> w, reciprocal shift
--   S26 .. S31  box filter (unit A: rays, unit B: rings)
--   S32 .. S34  tone (band / sawtooth / 4 steps), checker XOR
--   S35 .. S39  luma [1 2 1] / 4, 13/16 toward black, blanking gate
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture blaze of program_top is
    -- data_in -> data_out sync latency; must match processing_delay_clks in the TOML
    constant C_PROCESSING_DELAY_CLKS : natural := 40;

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_DB    : integer := 14;                 -- centre-knob detent deadband
    constant C_GL    : integer := 4;                  -- centre glide shift (1/16 of the gap per field)
    constant C_LREF  : integer := 9 * 16384;          -- log2(512 virtual px), Q14: the Period anchor radius

    -- stage map.  An EBR delay line written from stage W and read (straight from the EBR output
    -- register) by the logic building stage C runs its read address X = C - 2 - W behind the write.
    constant ST_OUT : integer := 39;                  -- output register
    constant C_XH   : integer := 15 - 2 - 8;          -- log2 hi + flags      S8  -> S15
    constant C_XA   : integer := 19 - 2 - 16;         -- L, log2 r (Q6)       S16 -> S19
    constant C_XN   : integer := 22 - 2 - 17;         -- Rays * theta         S17 -> S22
    constant C_XR   : integer := 29 - 2 - 18;         -- ring phase           S18 -> S29
    constant C_XT   : integer := 32 - 2 - 28;         -- tone base + flags    S28 -> S32
    constant C_XP   : integer := 25 - 2 - 22;         -- PHI                  S22 -> S25
    constant C_XS   : integer := 35;                  -- syncs: aligned with S35

    constant C_SQ_END : integer := 121;

    type t_rom256 is array(0 to 255) of std_logic_vector(15 downto 0);
    type t_la is array(0 to 15) of signed(9 downto 0);

    -- RAYS (K4): N = (4 + index mod 4) << (index / 4) = 4 5 6 7 8 10 12 14 16 20 24 28 32 40 48, and 64;
    -- C_LA = log2(N / 2 pi) + 1/4 octave, Q6 (the 1/4 stands for the sum with the radial gradient)
    constant C_LA : t_la := (
        to_signed(-26, 10), to_signed(-5, 10), to_signed(12, 10), to_signed(26, 10), to_signed(38, 10), to_signed(59, 10),
        to_signed(76, 10), to_signed(90, 10), to_signed(102, 10), to_signed(123, 10), to_signed(140, 10), to_signed(154, 10),
        to_signed(166, 10), to_signed(187, 10), to_signed(204, 10), to_signed(230, 10));

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
    signal s_gl   : t_gl := (to_unsigned(512 * 16, 14), to_unsigned(512 * 16, 14), to_unsigned(401 * 16, 14),
                             to_unsigned(512 * 16, 14), to_unsigned(1023 * 16, 14), to_unsigned(840 * 16, 14));
    signal s_gd, s_gs : signed(14 downto 0) := (others => '0');
    signal s_glide : std_logic := '1';                          -- S7
    signal s_chk, s_shade, s_fine, s_src : std_logic := '0';    -- S8 S9 S10 S11
    signal s_ri    : unsigned(3 downto 0) := to_unsigned(10, 4);   -- K4 ray-table index

    signal s_cxs, s_cys : signed(21 downto 0) := (others => '0');   -- centre (glided), Q8
    signal c_gd    : signed(21 downto 0) := (others => '0');
    signal c_gs, c_gm : std_logic := '0';

    signal s_sq    : unsigned(6 downto 0) := (others => '0');
    signal s_tneg  : std_logic := '0';                              -- Twist < 0
    signal s_tw14  : unsigned(15 downto 0) := (others => '0');      -- |Twist|, Q14 periods
    signal s_s12   : unsigned(11 downto 0) := to_unsigned(2048, 12);   -- apex s, Q12
    signal s_m     : unsigned(10 downto 0) := to_unsigned(379, 11);    -- rings per octave, Q8
    signal s_m9    : unsigned(8 downto 0) := to_unsigned(379, 9);      -- its mantissa: m = m9 << (K3 octave) >> 1
    signal s_la    : signed(9 downto 0) := to_signed(140, 10);
    signal s_lc    : signed(11 downto 0) := (others => '0');
    signal s_lgz   : signed(11 downto 0) := (others => '0');
    signal s_Yr, s_Yf : unsigned(12 downto 0) := (others => '0');   -- |slope| of the rising / falling segment, Q8
    signal s_LG1, s_LG0 : signed(11 downto 0) := to_signed(140, 12);
    signal s_dh    : signed(7 downto 0) := (others => '0');         -- ring filter width - ray filter width, half octaves
    signal s_g7    : unsigned(6 downto 0) := (others => '0');

    -- sequencer ROM port (exp2 mantissa)
    signal s_ra    : unsigned(7 downto 0) := (others => '0');
    signal s_rq, s_rqf : std_logic_vector(15 downto 0) := (others => '0');

    -- serial multiplier (signed x 12 unsigned, 12 clocks)
    signal sm_a    : signed(27 downto 0) := (others => '0');
    signal sm_b    : unsigned(11 downto 0) := (others => '0');
    signal sm_acc  : signed(27 downto 0) := (others => '0');
    signal sm_n    : unsigned(3 downto 0) := (others => '0');
    -- serial divider (22 / 12 -> 13, 22 clocks)
    signal dv_d    : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal dv_n    : unsigned(15 downto 0) := (others => '0');
    signal dv_r    : unsigned(11 downto 0) := (others => '0');
    signal dv_q    : unsigned(12 downto 0) := (others => '0');
    signal dv_c    : unsigned(4 downto 0) := (others => '0');
    -- serial normaliser for log2 (16 bit, up to 15 clocks)
    signal lz_x    : unsigned(15 downto 0) := (others => '0');
    signal lz_n    : unsigned(3 downto 0) := (others => '0');

    -- pixel-path views of the per-field terms
    signal w_mh    : signed(4 downto 0);
    signal w_ml    : signed(5 downto 0);
    signal w_mi    : unsigned(1 downto 0);
    signal w_nk    : unsigned(2 downto 0);                           -- rays = (4 + nm) << nk
    signal w_nm    : unsigned(1 downto 0);
    signal w_yrh, w_yfh : signed(6 downto 0);
    signal w_yrl, w_yfl : signed(7 downto 0);
    signal w_Dr    : unsigned(15 downto 0);
    signal w_steps : std_logic;

    ----------------------------------------------------------------------
    -- raster measurement + generated avid
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_saw_avid   : std_logic := '0';
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wv    : std_logic := '0';
    signal s_Wl    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    -- resolution scaling (as Hypnos): field-line step in x-pixel units (Q.8), the mode class
    -- (00 1080 / 01 720p / 10 SD) and LREF - log2(s)
    signal s_ystp  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_mcls  : std_logic_vector(1 downto 0) := "00";
    signal s_lacc  : unsigned(19 downto 0) := (others => '0');
    signal s_lref  : signed(19 downto 0) := to_signed(C_LREF, 20);
    -- source field per mode: the vertical IIR shift
    signal s_ksh   : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal s_avid_q, s_avid_r : std_logic := '0';
    signal s_hs_q, s_hs_r     : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_rsr   : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc    : unsigned(3 downto 0) := "1000";
    signal g_cnt   : unsigned(11 downto 0) := (others => '0');

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
    signal dla  : t_dl32;      -- L & log2 r (Q6)
    signal dln  : t_dl16;      -- Rays * theta
    signal dlp  : t_dl16;      -- PHI
    signal dlt  : t_dl16;      -- tone base, window flags
    signal dlr  : t_dl16;      -- ring phase
    signal dls  : t_dl4;       -- syncs
    attribute no_rw_check of dlh : signal is true;
    attribute no_rw_check of dla : signal is true;
    attribute no_rw_check of dln : signal is true;
    attribute no_rw_check of dlp : signal is true;
    attribute no_rw_check of dlt : signal is true;
    attribute no_rw_check of dlr : signal is true;
    attribute no_rw_check of dls : signal is true;
    signal s_wa : unsigned(7 downto 0) := (others => '0');
    signal dlh_q, dla_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dln_q, dlp_q, dlr_q, dlt_q : std_logic_vector(15 downto 0) := (others => '0');
    signal dls_q, s_sy36, s_sy37, s_sy38, s_sy39 : std_logic_vector(3 downto 0) := "0110";

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
    signal s10_tu, s11_tu, s12_tu : unsigned(6 downto 0) := (others => '0');
    signal ug_q, ugs_q, ua_q, uas_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s12_g, s12_gs, s12_a, s12_as : std_logic_vector(15 downto 0) := (others => '0');
    signal s12_tu8 : signed(7 downto 0) := (others => '0');
    signal s13_gp  : signed(21 downto 0) := (others => '0');
    signal s13_ap  : signed(21 downto 0) := (others => '0');
    signal s13_g, s13_a : unsigned(15 downto 0) := (others => '0');
    signal s14_g   : unsigned(13 downto 0) := (others => '0');
    signal s14_a   : unsigned(15 downto 0) := (others => '0');
    signal s15_lr  : unsigned(17 downto 0) := (others => '0');   -- log2 r, Q14 (real pixels)
    signal s15_th  : unsigned(17 downto 0) := (others => '0');   -- polar angle, 2^18/turn
    signal s15_th19 : signed(18 downto 0) := (others => '0');

    -- S16..S22: the field
    signal s16_L   : signed(17 downto 0) := (others => '0');     -- log2(r / 512 virtual px), Q12
    signal s16_thk : unsigned(15 downto 0) := (others => '0');   -- theta << nk
    signal s16_lrq : unsigned(9 downto 0) := (others => '0');    -- log2 r, Q6
    signal s17_ph  : signed(22 downto 0) := (others => '0');
    signal s17_pl  : signed(23 downto 0) := (others => '0');
    signal s17_nt  : unsigned(15 downto 0) := (others => '0');   -- Rays * theta, Q16 cycles
    signal s18_v   : unsigned(15 downto 0) := (others => '0');   -- frac(m * L), Q16
    signal s19_x   : signed(17 downto 0) := (others => '0');
    signal s19_yh  : signed(6 downto 0) := (others => '0');
    signal s19_yl  : signed(7 downto 0) := (others => '0');
    signal s19_rise : std_logic := '0';
    signal s19_lrq : unsigned(9 downto 0) := (others => '0');
    signal s20_qh  : signed(24 downto 0) := (others => '0');
    signal s20_ql  : signed(25 downto 0) := (others => '0');
    signal s21_z   : unsigned(15 downto 0) := (others => '0');
    signal s22_phi : unsigned(15 downto 0) := (others => '0');

    -- S20..S25: filter width (unit A = rays)
    signal s20_lg, s21_lg, s22_lw : signed(11 downto 0) := (others => '0');
    signal s22_ss, s23_ss, s24_ss, s25_ss, s26_ss, s27_ss, s28_ss : std_logic := '0';
    signal s23_h, s23_sat, s23_rh : std_logic := '0';
    signal s23_iw  : integer range -16 to 0 := -16;
    signal s23_ri, s24_ri, s25_ri, s26_ri, s27_ri, s28_ri : integer range 0 to 15 := 0;
    signal s24_rh, s25_rh, s26_rh, s27_rh, s28_rh, s29_rh : std_logic := '0';
    signal s24_w   : unsigned(15 downto 0) := (others => '1');
    signal s25_phi : unsigned(17 downto 0) := (others => '0');
    signal s25_wh, s26_wh : unsigned(14 downto 0) := (others => '0');
    signal s24_ds, s25_ds, s26_ds, s27_ds : unsigned(2 downto 0) := to_unsigned(4, 3);   -- duty code

    -- S28..S31: unit B (rings): one-pixel ramp across each ring edge, its width from unit A's
    signal s28b_h, s29b_h : unsigned(4 downto 0) := (others => '0');
    signal s28b_flat, s29b_flat, s30b_flat : std_logic := '0';
    signal s29b_e  : signed(15 downto 0) := (others => '0');
    signal s30b_d  : signed(16 downto 0) := (others => '0');
    signal s30b_sh : unsigned(3 downto 0) := (others => '0');
    signal s31b_c5 : unsigned(4 downto 0) := (others => '0');

    -- S26..S31: one-pixel box filter -> coverage 0..256
    signal s26_lo18 : unsigned(17 downto 0) := (others => '0');
    signal s26_hi  : unsigned(15 downto 0) := (others => '0');
    signal s27_k, s27_full, s28_full, s29_full, s30_full : std_logic := '0';
    signal s27_mh, s27_ml : unsigned(15 downto 0) := (others => '0');
    signal s27_ts  : unsigned(16 downto 0) := (others => '0');
    signal s27_q2  : unsigned(1 downto 0) := (others => '0');
    signal s28_n   : unsigned(15 downto 0) := (others => '0');
    signal s28_tb  : unsigned(8 downto 0) := (others => '0');
    signal s28_k, s28_q3 : std_logic := '0';
    signal s29_nsh : unsigned(9 downto 0) := (others => '0');
    signal s30_cp  : unsigned(10 downto 0) := (others => '0');
    signal s31_cov : unsigned(8 downto 0) := (others => '0');

    -- S32..S39: tone, checker, output conditioning
    signal s32_tone, s33_tone : unsigned(8 downto 0) := (others => '0');
    signal s32_c5  : unsigned(4 downto 0) := (others => '0');
    signal s33_a, s33_b : signed(15 downto 0) := (others => '0');
    signal s34_mix, s35_m, s36_m, s37_m : unsigned(8 downto 0) := (others => '0');
    signal s37_f   : unsigned(10 downto 0) := (others => '0');
    signal s38_y   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal o_y     : unsigned(9 downto 0) := to_unsigned(64, 10);

    ----------------------------------------------------------------------
    -- source field (camera luma or the Osc lattice -> box -> vertical IIR)
    ----------------------------------------------------------------------
    signal s_ot    : unsigned(7 downto 0) := (others => '0');
    signal s_osum  : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_col   : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram  : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q    : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt  : unsigned(6 downto 0) := (others => '0');
    signal s_bsum  : unsigned(13 downto 0) := (others => '0');
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
    signal s_qh, s_qc, s_qp : unsigned(12 downto 0) := (others => '0');
    signal s_c11   : unsigned(10 downto 0) := (others => '0');
    signal s_camoff : unsigned(15 downto 0) := (others => '0');

    function f_sat(x : unsigned; w : integer) return unsigned is
    begin
        if x > to_unsigned(2**w - 1, x'length) then
            return to_unsigned(2**w - 1, w);
        end if;
        return resize(x, w);
    end function;

    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;

    function f_duty(c : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(c) is
            when 0      => return to_unsigned(10923, 16);
            when 1      => return to_unsigned(11648, 16);
            when 2      => return to_unsigned(16384, 16);
            when 3      => return to_unsigned(23296, 16);
            when 4      => return to_unsigned(32768, 16);
            when others => return to_unsigned(65535, 16);
        end case;
    end function;

    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
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
    -- raster measurement + GENERATED AVID (Hypnos / cubist v0.3.3): start 2 or
    -- 3 clocks after the source's first avid rise, whichever keeps the
    -- hsync->start parity at the source's usual value, fixed field-latched width.
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;

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
            -- log2(s): SD 1 octave, 720p log2(1.5) = 0.585 (Q14 9585)
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

            if s_hs_r = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if s_avid_r = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                if g_cnt + 1 >= s_Wl then g_avid <= '0'; end if;
                g_cnt <= g_cnt + 1;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-field sequencer.  The field tick is the first vsync edge after active
    -- video (analog vsync serrates); it restarts the sequence, which runs in
    -- blanking, one small operation per state:
    --   8 .. 31   six knobs, 4 states each (3 used): the glided value moves only when the knob
    --             is 2 counts or more away (pot jitter: a hold, not an ease), then by
    --             1/16 of the gap per field (S7 On; K1 / K2 glide in pixels below) or
    --             to the knob itself (S7 Off: no easing, used this field)
    --   32 ..     rays, period, skew, twist, centre, the slopes A = Tw / s and
    --             B = Tw / (1 - s), Tm = Tw * m, and the filter-width logs
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_ch   : integer range 0 to 7;
        variable v_k    : signed(10 downto 0);
        variable v_sh   : signed(14 downto 0);
        variable v_mv   : boolean;
        variable v_gc, v_gdl : signed(21 downto 0);
        variable v_z    : unsigned(3 downto 0);
        variable v_s    : unsigned(5 downto 0);
        variable v_g, v_gl : unsigned(13 downto 0);
        variable v_reg  : unsigned(9 downto 0);
        variable v_wh   : unsigned(11 downto 0);
        variable v_tm   : unsigned(12 downto 0);
        variable v_d    : signed(13 downto 0);
        variable v_t    : signed(14 downto 0);
        variable v_g8   : unsigned(6 downto 0);
        variable v_lg   : signed(11 downto 0);
        variable v_mx   : signed(11 downto 0);
        variable v_r    : unsigned(12 downto 0);
        variable v_dh   : signed(11 downto 0);
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
            if dv_c /= 0 then                                     -- divider: restoring, MSB first
                v_r := dv_r & dv_n(15);
                dv_n <= dv_n(14 downto 0) & '0';
                if v_r >= resize(dv_d, 13) then
                    dv_r <= resize(v_r - resize(dv_d, 13), 12);
                    dv_q <= dv_q(11 downto 0) & '1';
                else
                    dv_r <= v_r(11 downto 0);
                    dv_q <= dv_q(11 downto 0) & '0';
                end if;
                dv_c <= dv_c - 1;
            end if;
            if lz_x(15) = '0' and lz_n /= 15 then                 -- normaliser: 15 - lz_n = the msb position
                lz_x <= lz_x(14 downto 0) & '0';
                lz_n <= lz_n + 1;
            end if;

            v_ch := to_integer(s_sq(4 downto 2));
            if s_sq(3) = '0' then v_gc := s_cxs; v_wh := s_W; else v_gc := s_cys; v_wh := s_H; end if;   -- 48..50 vs 61..63
            -- the knob and its glided value for this channel (v_ch - 2: 0 K1, 1 K2, 2 K3, 3 K5, 4 K6, 5 P12)
            case v_ch is
                when 2      => v_reg := unsigned(registers_in(0)); v_gl := s_gl(0);
                when 3      => v_reg := unsigned(registers_in(1)); v_gl := s_gl(1);
                when 4      => v_reg := unsigned(registers_in(2)); v_gl := s_gl(2);
                when 5      => v_reg := unsigned(registers_in(4)); v_gl := s_gl(3);
                when 6      => v_reg := unsigned(registers_in(5)); v_gl := s_gl(4);
                when others => v_reg := unsigned(registers_in(7)); v_gl := s_gl(5);
            end case;
            -- log2 (Q6): msb & linear mantissa; an all-zero input reads as 2^-32
            if lz_x(15) = '0' then v_lg := to_signed(-2048, 12);
            else v_lg := signed(resize((to_unsigned(15, 4) - lz_n) & lz_x(14 downto 9), 12)); end if;
            if s_la > s_lgz then v_mx := resize(s_la, 12); else v_mx := s_lgz; end if;
            -- K1 (state 34) or K2 (state 48) about the detent
            if s_sq(4) = '0' then v_g := s_gl(0); else v_g := s_gl(1); end if;
            v_k := signed(resize(v_g(13 downto 4), 11)) - to_signed(512, 11);
            if abs(v_k) < C_DB then v_k := (others => '0'); end if;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_saw_avid = '1' then
                s_vs_pulse <= '1';
                s_saw_avid <= '0';
                s_glide <= registers_in(6)(0);            -- S7 Glide
                s_chk   <= registers_in(6)(1);            -- S8 Form: Zigzag / Checker
                s_shade <= registers_in(6)(2);            -- S9 Shade
                s_fine  <= registers_in(6)(3);            -- S10 Weight
                s_src   <= registers_in(6)(4);            -- S11 Source
                s_Wl    <= s_W;
                s_sq    <= to_unsigned(8, 7);

            elsif s_sq /= 0 and data_in.avid = '0' then
                if s_sq = C_SQ_END then s_sq <= (others => '0'); else s_sq <= s_sq + 1; end if;

                if s_sq(6 downto 5) = "00" then
                    case to_integer(s_sq(1 downto 0)) is
                        when 1 =>
                            s_gd <= signed(shift_left(resize(v_reg, 15), 4)) - signed(resize(v_gl, 15));
                        when 2 =>
                            v_mv := s_gd > 31 or s_gd < -31 or v_reg = 0 or v_reg = 1023;
                            v_sh := shift_right(s_gd, 4);
                            if not v_mv then                                    s_gs <= (others => '0');
                            elsif s_glide = '0' or v_ch < 4 or v_sh = 0 then    s_gs <= s_gd;
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
                            -- RAYS (K4): 16 steps, hysteresis with the two end exceptions
                            v_z := unsigned(registers_in(3)(9 downto 6));
                            v_s := unsigned(registers_in(3)(5 downto 0));
                            if v_z /= s_ri and ((v_s >= 8 and v_s <= 55) or (v_z = 0 and v_s < 8) or (v_z = 15 and v_s > 55)) then
                                s_ri <= v_z;
                            end if;
                            -- PERIOD (K3): log2 m = K3 / 256 - 1 octaves, m = 0.5 .. 8 rings per octave
                            s_ra <= s_gl(2)(11 downto 4);
                            -- TWIST (P12): +-8 counts of dead centre, then 5 / 1024 period per count
                            v_g := s_gl(5);
                            v_d := signed((not v_g(13)) & v_g(12 downto 0));          -- P12 - 512, Q4
                            s_tneg <= v_d(13);
                            if v_d(13) = '1' then v_t := resize(signed(not std_logic_vector(v_d)), 15) - to_signed(127, 15);
                            else                  v_t := resize(v_d, 15) - to_signed(128, 15); end if;
                            if v_t(14) = '1' then v_tm := (others => '0'); else v_tm := unsigned(v_t(12 downto 0)); end if;
                            s_tw14 <= shift_left(resize(v_tm, 16), 2) + resize(v_tm, 16);
                        when 33 =>
                            -- SKEW (K5): apex s = 1/8 .. 7/8 (Q12)
                            s_s12 <= to_unsigned(512, 12) + resize(shift_right(s_gl(3), 3), 12) + resize(shift_right(s_gl(3), 4), 12);
                        when 34 =>
                            -- A = (Twist << 6) / s
                            dv_n <= s_tw14; dv_d <= s_s12;
                            dv_r <= (others => '0'); dv_q <= (others => '0'); dv_c <= to_unsigned(22, 5);
                            -- centre x = W / 2 + (K1 - 512) * W / 256 (detent +-14 counts)
                            sm_a <= resize(v_k, 28); sm_b <= s_W; sm_acc <= (others => '0'); sm_n <= to_unsigned(12, 4);
                        when 35 =>
                            s_m9 <= unsigned(s_rqf(14 downto 6));
                            case s_gl(2)(13 downto 12) is                           -- m = 2^(K3 / 256 - 1), Q8
                                when "00"   => s_m <= resize(unsigned(s_rqf(15 downto 7)), 11);
                                when "01"   => s_m <= resize(unsigned(s_rqf(15 downto 6)), 11);
                                when "10"   => s_m <= unsigned(s_rqf(15 downto 5));
                                when others => s_m <= resize(unsigned(s_rqf(15 downto 4)), 11);
                            end case;
                            s_la <= C_LA(to_integer(s_ri));
                            -- log2 m + log2(1 / ln 2) + 1/4 octave - 8 octaves (the slopes are Q8), Q6
                            s_lc <= signed(resize(s_gl(2)(13 downto 6), 12)) - to_signed(526, 12);
                        -- centre: target = W / 2 + product; the glides share one datapath, x at 48..50,
                        -- y at 61..63 (ease, never slower than 1 px / field, land exactly; S7 Off: snap)
                        when 48 | 61 =>
                            c_gd <= shift_left(resize(signed('0' & v_wh(11 downto 1)), 22), 8) + resize(sm_acc, 22) - v_gc;
                            if s_sq(3) = '0' then      -- centre y = H / 2 + (K2 - 512) * H / 256
                                sm_a <= resize(v_k, 28); sm_b <= s_H; sm_acc <= (others => '0'); sm_n <= to_unsigned(12, 4);
                            else                       -- Tm = Twist * m
                                sm_a <= signed(resize(s_tw14, 28)); sm_b <= resize(s_m, 12); sm_acc <= (others => '0'); sm_n <= to_unsigned(12, 4);
                            end if;
                        when 49 | 62 =>
                            -- under 1 px to go: land; under 16 px: 1 px per field; else 1/16 of the gap
                            if s_glide = '0' or c_gd(21 downto 8) = 0 or c_gd(21 downto 8) = -1 then c_gs <= '1'; else c_gs <= '0'; end if;
                            if c_gd(21 downto 12) = 0 or c_gd(21 downto 12) = -1 then c_gm <= '1'; else c_gm <= '0'; end if;
                        when 50 | 63 =>
                            if    c_gs = '1' then v_gdl := c_gd;
                            elsif c_gm = '1' then
                                if c_gd(21) = '0' then v_gdl := to_signed(256, 22); else v_gdl := to_signed(-256, 22); end if;
                            else  v_gdl := shift_right(c_gd, C_GL); end if;
                            if s_sq(3) = '0' then s_cxs <= v_gc + v_gdl;
                            else                  s_cys <= v_gc + v_gdl; end if;
                        when 58 =>
                            -- A ; start B = (Twist << 6) / (1 - s) and log2 A
                            s_Yr <= dv_q;
                            lz_x <= resize(dv_q, 16); lz_n <= (others => '0');
                            dv_n <= s_tw14; dv_d <= to_unsigned(0, 12) - s_s12;
                            dv_r <= (others => '0'); dv_q <= (others => '0'); dv_c <= to_unsigned(22, 5);
                        when 82 =>
                            s_Yf <= dv_q;
                            lz_x <= resize(dv_q, 16); lz_n <= (others => '0');
                        when 102 =>
                            lz_x <= resize(unsigned(sm_acc(26 downto 14)), 16); lz_n <= (others => '0');
                        -- log2 of (the radial gradient * r), 1/4 octave up; LG = the larger of it and the angular one
                        when 76 | 100 =>
                            if lz_x(15) = '0' then s_lgz <= v_lg; else s_lgz <= v_lg + s_lc; end if;
                        when 119 =>
                            if lz_x(15) = '0' then s_lgz <= v_lg; else s_lgz <= v_lg - to_signed(462, 12); end if;
                        when 77 => s_LG1 <= v_mx;
                        when 101 => s_LG0 <= v_mx;
                        when 120 =>
                            -- Checker: both segments are the spiral's
                            if s_chk = '1' then
                                s_LG1 <= v_mx; s_LG0 <= v_mx;
                                s_Yr <= unsigned(sm_acc(26 downto 14)); s_Yf <= unsigned(sm_acc(26 downto 14));
                            end if;
                            -- CAMERA (K6): 0 .. 2 periods at white (Q5); Osc is fed at 1.5 x
                            v_g8 := resize(s_gl(4)(13 downto 8), 7);
                            if s_src = '1' then v_g8 := v_g8 + shift_right(v_g8, 1); end if;
                            s_g7 <= v_g8;
                        when 121 =>
                            -- the rings' filter width against the rays', in half octaves:
                            -- (LG1 - log2 m - log2(1 / ln 2) + rounding) / 32
                            v_dh := s_LG1 - signed(resize(s_gl(2)(13 downto 6), 12)) + to_signed(46, 12);
                            s_dh <= resize(shift_right(v_dh, 5), 8);
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process p_frame;

    w_mh <= signed('0' & s_m9(8 downto 5));
    w_ml <= signed('0' & s_m9(4 downto 0));
    w_mi <= s_gl(2)(13 downto 12);
    w_nk <= "100" when s_ri = 15 else '0' & s_ri(3 downto 2);
    w_nm <= "00" when s_ri = 15 else s_ri(1 downto 0);
    w_yrh <= signed('0' & s_Yr(12 downto 7));
    w_yrl <= signed('0' & s_Yr(6 downto 0));
    w_yfh <= signed('0' & s_Yf(12 downto 7));
    w_yfl <= signed('0' & s_Yf(6 downto 0));
    w_Dr  <= s_s12 & "0000";
    w_steps <= s_shade and not s_fine;

    ------------------------------------------------------------------------
    -- coordinate accumulators on the GENERATED avid.  dx loads -cx at each
    -- line start and steps per pixel; dy seeds at the first line of the frame
    -- and steps per line by s_ystp (Q.8 x-pixel units, seeded +0.5).
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            s_gav_p <= g_avid;
            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif g_avid = '1' and s_gav_p = '0' then
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
            elsif g_avid = '1' then
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
            dla(to_integer(s_wa)) <= "0000" & std_logic_vector(s16_L) & std_logic_vector(s16_lrq);
            dla_q  <= dla(to_integer(s_wa - C_XA));
            dln(to_integer(s_wa)) <= std_logic_vector(s17_nt);
            dln_q  <= dln(to_integer(s_wa - C_XN));
            dlp(to_integer(s_wa)) <= std_logic_vector(s22_phi);
            dlp_q  <= dlp(to_integer(s_wa - C_XP));
            dlr(to_integer(s_wa)) <= std_logic_vector(s18_v);
            dlt(to_integer(s_wa)) <= "0000" & s28_k & s28_ss & s28_q3 & std_logic_vector(s28_tb);
            dlt_q  <= dlt(to_integer(s_wa - C_XT));
            dlr_q  <= dlr(to_integer(s_wa - C_XR));
            dls(to_integer(s_wa)) <= g_avid & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
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
            s1_a <= f_sat(va, 13);
            s1_b <= f_sat(vb, 13);
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
            s10_tu <= s9_ua(9 downto 3);
            -- S11, S12 (EBR read, fabric register)
            s11_tu <= s10_tu;
            s12_tu <= s11_tu;
            s12_tu8 <= signed(resize(s11_tu, 8));
            s12_g <= ug_q; s12_gs <= ugs_q; s12_a <= ua_q; s12_as <= uas_q;
            -- S13: interpolation products (slopes are negative)
            s13_gp <= resize(bmul(signed(s12_gs(10 downto 0)), s12_tu8), 22);
            s13_ap <= resize(bmul(signed(s12_as(10 downto 0)), s12_tu8), 22);
            s13_g  <= unsigned(s12_g);
            s13_a  <= unsigned(s12_a);
            -- S14
            s14_g <= resize(unsigned(signed(resize(s13_g, 17)) + resize(shift_right(s13_gp, 7), 17)), 14);
            s14_a <= resize(unsigned(signed(resize(s13_a, 17)) + resize(shift_right(s13_ap, 7), 17)), 16);
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
            s15_th   <= vth;
            s15_th19 <= signed(resize(vth, 19));
        end if;
    end process p_polar;

    ------------------------------------------------------------------------
    -- S16..S24: the field.  v = m * L (rings), the zigzag segment, one multiply
    -- X * Y (Zigzag: t or 1 - t times the segment's slope; Checker: L times
    -- Twist * m, a log spiral), PHI = Rays * theta + z + camera.
    ------------------------------------------------------------------------
    p_field : process(clk)
        variable vl  : signed(19 downto 0);
        variable vp  : unsigned(20 downto 0);
        variable vt  : unsigned(11 downto 0);
        variable vx  : unsigned(12 downto 0);
        variable vz  : unsigned(19 downto 0);
        variable vg1, vg0 : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- S16: L (Q12) ; theta << nk
            vl := signed(resize(s15_lr, 20)) - s_lref;
            s16_L   <= vl(19 downto 2);
            s16_thk <= shift_left(s15_th(17 downto 2), to_integer(w_nk));
            s16_lrq <= s15_lr(17 downto 8);
            -- S17: m * L, split on m ; Rays * theta = (4 + nm) * (theta << nk): whole rays, so mod one
            -- cycle is seamless
            s17_ph <= bmul(s16_L, w_mh);
            s17_pl <= bmul(s16_L, w_ml);
            if w_nm(1) = '1' then vg1 := shift_left(s16_thk, 1); else vg1 := (others => '0'); end if;
            if w_nm(0) = '1' then vg0 := s16_thk; else vg0 := (others => '0'); end if;
            s17_nt <= shift_left(s16_thk, 2) + vg1 + vg0;
            -- S18: the ring fraction, Q16: bits of L * m9 << (octave of m)
            vp := shift_left(resize(unsigned(s17_ph(15 downto 0)), 21), 5) + unsigned(s17_pl(20 downto 0));
            case w_mi is
                when "00"   => s18_v <= vp(20 downto 5);
                when "01"   => s18_v <= vp(19 downto 4);
                when "10"   => s18_v <= vp(18 downto 3);
                when others => s18_v <= vp(17 downto 2);
            end case;
            -- S19: ring fraction t (Q12) against the apex ; operands of the twist multiply
            vt := s18_v(15 downto 4);
            if vt < s_s12 then
                s19_rise <= '1';
                vx := resize(vt, 13);
                s19_yh <= w_yrh; s19_yl <= w_yrl;
            else
                s19_rise <= '0';
                vx := to_unsigned(4096, 13) - resize(vt, 13);
                s19_yh <= w_yfh; s19_yl <= w_yfl;
            end if;
            if s_chk = '1' then s19_x <= signed(dla_q(27 downto 10));
            else                s19_x <= signed(resize(vx, 18)); end if;
            s19_lrq <= unsigned(dla_q(9 downto 0));
            -- S20: X * Y, split on Y
            s20_qh <= bmul(s19_x, s19_yh);
            s20_ql <= bmul(s19_x, s19_yl);
            -- S21: Q20 cycles -> Q16
            vz := shift_left(resize(unsigned(s20_qh(12 downto 0)), 20), 7) + unsigned(s20_ql(19 downto 0));
            s21_z <= vz(19 downto 4);
            -- S22: the phase (Q16 cycles); a negative Twist subtracts z
            s22_phi <= f_addsub(unsigned(dln_q), s21_z, s_tneg) + s_camoff;
        end if;
    end process p_field;

    ------------------------------------------------------------------------
    -- S20..S25: filter width.  log2 w = LG - log2 r (cycles per pixel, Q6): LG is
    -- per field, one per zigzag segment.  Above 1/4 cycle per pixel w is widened so
    -- it reaches a full cycle (flat grey) at Nyquist.  In the 4-step Shade the unit
    -- works on 4 * PHI (the quarter boundaries) below that.  w is rounded UP to a
    -- half octave so the divide is a shift.   (Hypnos S31..S35)
    ------------------------------------------------------------------------
    p_grad : process(clk)
        variable vlg : signed(11 downto 0);
        variable vlw : signed(11 downto 0);
        variable viw : integer;
        variable vsh : integer;
        variable vw  : unsigned(15 downto 0);
        variable vm  : unsigned(6 downto 0);
        variable vrr : signed(12 downto 0);
        variable vq  : integer range -40 to 8;
    begin
        if rising_edge(clk) then
            -- S20
            if s19_rise = '1' then s20_lg <= s_LG1 - signed(resize(s19_lrq, 12));
            else                   s20_lg <= s_LG0 - signed(resize(s19_lrq, 12)); end if;
            -- S21
            vlg := s20_lg;
            if vlg > 255 then vlg := to_signed(255, 12); end if;
            s21_lg <= vlg;
            -- S22 (2 lg + 128 as a shift: no self-add)
            if s21_lg > -128 then
                s22_lw <= shift_left(s21_lg, 1) + 128; s22_ss <= '0';
            elsif w_steps = '1' then
                s22_lw <= s21_lg + 128; s22_ss <= '1';
            else
                s22_lw <= s21_lg; s22_ss <= '0';
            end if;
            -- S23: clamp to 16 octaves ; quantise UP to a half octave: w in {1, sqrt2} * 2^Iw
            vlw := s22_lw;
            if vlw < -1023 then vlw := to_signed(-1023, 12); end if;
            vlw := vlw + 31;
            vlw(4 downto 0) := (others => '0');
            if vlw >= 0 then s23_sat <= '1'; else s23_sat <= '0'; end if;
            s23_h <= vlw(5);
            viw := to_integer(shift_right(vlw, 6));
            if viw > 0 then viw := 0; end if;
            s23_iw <= viw;
            if vlw >= 0 then vrr := (others => '0'); else vrr := -resize(vlw, 13); end if;
            s23_ri <= to_integer(shift_right(vrr, 6)) mod 16;
            s23_rh <= vrr(5);
            s23_ss <= s22_ss;
            -- S24: filter width w (Q16) = {64, 91} * 2^(Iw + 10)
            if s23_sat = '1' then
                vw := (others => '1');
            else
                vsh := s23_iw + 10;
                if s23_h = '1' then vm := to_unsigned(91, 7); else vm := to_unsigned(64, 7); end if;
                if vsh >= 0 then vw := resize(shift_left(resize(vm, 16), vsh), 16);
                else             vw := resize(shift_right(resize(vm, 16), -vsh), 16); end if;
                if vw = 0 then vw := to_unsigned(1, 16); end if;
            end if;
            s24_w  <= vw;
            s24_ri <= s23_ri; s24_rh <= s23_rh; s24_ss <= s23_ss;
            -- duty code.  Shade: 1 (code 7).  Bold: 1/2 (4).  Fine: a line 1/6 of the period (0), never
            -- under 2 px at 1080 (1 px in the other modes), up to 1/2: w steps by half octaves, so the
            -- floor c * w is one of 11648 / 16384 / 23296 / 32768 (codes 1 .. 4)
            vq := 2 * s23_iw;
            if s23_h = '1' then vq := vq + 1; end if;
            if s_mcls = "00" then vq := vq + 2; end if;
            if s_shade = '1' then                 s24_ds <= to_unsigned(7, 3);
            elsif s_fine = '0' or s23_sat = '1' then s24_ds <= to_unsigned(4, 3);
            elsif vq <= -6 then                   s24_ds <= to_unsigned(0, 3);
            elsif vq = -5 then                    s24_ds <= to_unsigned(1, 3);
            elsif vq = -4 then                    s24_ds <= to_unsigned(2, 3);
            elsif vq = -3 then                    s24_ds <= to_unsigned(3, 3);
            else                                  s24_ds <= to_unsigned(4, 3); end if;
            -- S25: phase (Q16 of the cycle, or of the quarter with the quarter index on top), half width
            if s24_ss = '1' then s25_phi <= unsigned(dlp_q) & "00";
            else                 s25_phi <= "00" & unsigned(dlp_q); end if;
            s25_wh <= s24_w(15 downto 1);
            s25_ri <= s24_ri; s25_rh <= s24_rh; s25_ss <= s24_ss;
            s25_ds <= s24_ds;
        end if;
    end process p_grad;

    ------------------------------------------------------------------------
    -- S28..S31: unit B, the rings of the Checker.  Signed distance (in cycles) to
    -- the nearer edge of the band frac(v) in [0, Skew), times 1 / w = a one-pixel
    -- ramp, 17 levels.  Its w is unit A's times a per-field ratio (both gradients
    -- fall as 1 / r), in half octaves rounded to the sharper whole octave.  Rings
    -- finer than about 3 px: the flat duty grey.
    ------------------------------------------------------------------------
    p_ring : process(clk)
        variable vh  : signed(7 downto 0);
        variable ve  : unsigned(15 downto 0);
        variable vq  : signed(16 downto 0);
        variable vr  : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- S28: 1 / w in half octaves (unit A's is 2 Ri + Rh), 0 .. 24
            vh := signed(resize(to_unsigned(s27_ri, 4) & s27_rh, 8)) + s_dh;
            if s27_ss = '1' then vh := vh + 4; end if;             -- unit A works on 4 PHI there: its w is 4 x the rays'
            if vh < 3 then s28b_flat <= '1'; else s28b_flat <= '0'; end if;
            if vh < 0 then s28b_h <= (others => '0');
            elsif vh > 24 then s28b_h <= to_unsigned(24, 5);
            else s28b_h <= unsigned(vh(4 downto 0)); end if;
            -- S29: offset from the band's middle (Skew / 2), wrapping
            s29b_e <= signed(unsigned(dlr_q) - resize(w_Dr(15 downto 1), 16));
            s29b_h <= s28b_h; s29b_flat <= s28b_flat;
            -- S30: distance inside the band = Skew / 2 - |offset|
            if s29b_e(15) = '1' then ve := unsigned(s29b_e); else ve := unsigned(not s29b_e) + 1; end if;
            s30b_d <= signed(resize(w_Dr(15 downto 1), 17)) + resize(signed(ve), 17);
            vr := resize(s29b_h, 6) + 1;                                -- round the half octave to the sharper side
            s30b_sh <= to_unsigned(12, 4) - vr(4 downto 1);
            s30b_flat <= s29b_flat;
            -- S31: coverage = 1/2 + distance / w, 0 .. 16
            vq := shift_right(s30b_d, to_integer(s30b_sh));
            if s30b_flat = '1' then s31b_c5 <= '0' & w_Dr(15 downto 12);
            elsif vq > 7 then       s31b_c5 <= to_unsigned(16, 5);
            elsif vq < -8 then      s31b_c5 <= to_unsigned(0, 5);
            else                    s31b_c5 <= unsigned(vq(4 downto 0)) + 8; end if;
        end if;
    end process p_ring;

    ------------------------------------------------------------------------
    -- S26..S31: one-pixel box filter of the band square-wave (Hypnos S36..S41).
    --   band = frac in [0, D) ; P(x) = floor(x) * D + min(frac x, D)
    --   coverage = (P(phi + w/2) - P(phi - w/2)) / w   (divide = * 2^-log2 w)
    -- Shade: D = 1 and the low clamp is forced, so the result is the part of the
    -- window past the sawtooth's reset: mean = lo + w/2 - coverage.
    ------------------------------------------------------------------------
    p_cov : process(clk)
        variable vn  : signed(17 downto 0);
        variable vs  : unsigned(24 downto 0);
        variable vc  : unsigned(9 downto 0);
        variable vq  : unsigned(1 downto 0);
        variable vD  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- S26
            s26_lo18 <= s25_phi - resize(s25_wh, 18);
            s26_hi   <= s25_phi(15 downto 0) + resize(s25_wh, 16);
            s26_wh   <= s25_wh;
            s26_ds   <= s25_ds;
            s26_ri <= s25_ri; s26_rh <= s25_rh; s26_ss <= s25_ss;
            -- S27
            if s26_hi < s26_lo18(15 downto 0) then s27_k <= '1'; else s27_k <= '0'; end if;
            vD := f_duty(s26_ds);
            if s_shade = '0' and s26_hi >= s26_lo18(15 downto 0) and s26_hi < vD then s27_full <= '1'; else s27_full <= '0'; end if;
            if s26_hi < vD then s27_mh <= s26_hi; else s27_mh <= vD; end if;
            if s_shade = '0' and s26_lo18(15 downto 0) < vD then s27_ml <= s26_lo18(15 downto 0); else s27_ml <= vD; end if;
            s27_ts <= resize(s26_lo18(15 downto 0), 17) + resize(s26_wh, 17);
            s27_q2 <= s26_lo18(17 downto 16);
            s27_ds <= s26_ds;
            s27_ri <= s26_ri; s27_rh <= s26_rh; s27_ss <= s26_ss;
            -- S28
            vn := signed(resize(s27_mh, 18)) - signed(resize(s27_ml, 18));
            if s27_k = '1' then vn := vn + signed(resize(f_duty(s27_ds), 18)); end if;
            if vn < 0 then vn := (others => '0'); end if;
            s28_n   <= unsigned(vn(15 downto 0));
            s28_ri  <= s27_ri; s28_rh <= s27_rh; s28_full <= s27_full;
            -- tone base: the sawtooth's mean before the reset part, or the step level 0 / 85 / 170 / 255
            vq := s27_q2;
            if s27_ss = '1' then s28_tb <= resize(vq & vq & vq & vq, 9);
            else                 s28_tb <= s27_ts(16 downto 8); end if;
            s28_k <= s27_k; s28_ss <= s27_ss;
            if vq = "11" then s28_q3 <= '1'; else s28_q3 <= '0'; end if;
            -- S29: (N << Ri) >> 7  (N <= w, so N << Ri < 2^17: a slice, no clamp)
            vs := shift_left(resize(s28_n, 25), s28_ri);
            s29_nsh <= vs(16 downto 7);
            s29_rh  <= s28_rh; s29_full <= s28_full;
            -- S30: / 2, or * sqrt2/2 = .70703 (shift-adds) on a half-octave step
            if s29_rh = '1' then
                s30_cp <= resize(shift_right(s29_nsh, 1) + shift_right(s29_nsh, 3)
                                 + shift_right(s29_nsh, 4) + shift_right(s29_nsh, 6), 11);
            else
                s30_cp <= resize(shift_right(s29_nsh, 1), 11);
            end if;
            s30_full <= s29_full;
            -- S31: coverage 0..256
            vc := resize(s30_cp, 10);
            if vc > 256 or s30_full = '1' then vc := to_unsigned(256, 10); end if;
            s31_cov <= resize(vc, 9);
        end if;
    end process p_cov;

    ------------------------------------------------------------------------
    -- S32..S39: tone, checker, output conditioning.
    --   Flat: tone = coverage.  Shade smooth: the filtered sawtooth.  Shade 4 steps:
    --   the step level, + coverage / 3 across a step, 255 - coverage across the reset.
    --   Checker: tone XOR ring = tone + (256 - 2 tone) * ring (17 ring levels, shift-adds).
    --   Output: [1 2 1] / 4 with replicated picture edges, luma 13/16 toward black
    --   (white = 768), blanking gate.  U = V = 512 always.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable vt  : unsigned(9 downto 0);
        variable vc3 : unsigned(8 downto 0);
        variable vd  : signed(15 downto 0);
        variable vg  : signed(15 downto 0);
        variable vm  : signed(15 downto 0);
        variable vl, vr : unsigned(8 downto 0);
        variable vf  : unsigned(17 downto 0);
        variable v_tb : unsigned(8 downto 0);
        variable v_q3, v_ss, v_k : std_logic;
    begin
        if rising_edge(clk) then
            -- S32
            v_tb := unsigned(dlt_q(8 downto 0)); v_q3 := dlt_q(9); v_ss := dlt_q(10); v_k := dlt_q(11);
            vc3 := shift_right(s31_cov, 2) + shift_right(s31_cov, 4) + shift_right(s31_cov, 6) + shift_right(s31_cov, 8);
            if s_shade = '0' then
                vt := resize(s31_cov, 10);
            elsif v_k = '0' then
                vt := resize(v_tb, 10);
            elsif v_ss = '1' then
                if v_q3 = '1' then
                    if s31_cov(8) = '1' then vt := (others => '0');
                    else vt := to_unsigned(255, 10) - resize(s31_cov, 10); end if;
                else
                    vt := resize(v_tb, 10) + resize(vc3, 10);
                end if;
            else
                if v_tb < s31_cov then vt := (others => '0');
                else vt := resize(v_tb, 10) - resize(s31_cov, 10); end if;
                if vt > 256 then vt := to_unsigned(256, 10); end if;
            end if;
            s32_tone <= vt(8 downto 0);
            if s_chk = '1' then s32_c5 <= s31b_c5; else s32_c5 <= (others => '0'); end if;
            -- S33: (256 - 2 tone) * c5 as two sums of gated shifts
            vd := to_signed(256, 16) - signed(resize(shift_left(resize(s32_tone, 10), 1), 16));
            if s32_c5(4) = '1' then
                s33_a <= shift_left(vd, 4); s33_b <= (others => '0');
            else
                if s32_c5(3) = '1' then vg := shift_left(vd, 3); else vg := (others => '0'); end if;
                if s32_c5(2) = '1' then s33_a <= vg + shift_left(vd, 2); else s33_a <= vg; end if;
                if s32_c5(1) = '1' then vg := shift_left(vd, 1); else vg := (others => '0'); end if;
                if s32_c5(0) = '1' then s33_b <= vg + vd; else s33_b <= vg; end if;
            end if;
            s33_tone <= s32_tone;
            -- S34
            vm := signed(resize(s33_tone, 16)) + shift_right(s33_a + s33_b, 4);
            s34_mix <= unsigned(vm(8 downto 0));
            -- S35..S37: three-pixel window; the centre is s36_m
            s35_m <= s34_mix; s36_m <= s35_m; s37_m <= s36_m;
            s_sy36 <= dls_q; s_sy37 <= s_sy36; s_sy38 <= s_sy37; s_sy39 <= s_sy38;
            if dls_q(3) = '1' then vr := s35_m; else vr := s36_m; end if;
            if s_sy37(3) = '1' then vl := s37_m; else vl := s36_m; end if;
            s37_f <= resize(vl, 11) + shift_left(resize(s36_m, 11), 1) + resize(vr, 11);
            -- S38: Y = 64 + f * 11 / 16  (= the [1 2 1] / 4 mean, scaled toward black: white = 768)
            vf := shift_left(resize(s37_f, 18), 3) + shift_left(resize(s37_f, 18), 1) + resize(s37_f, 18) + 8;
            s38_y <= to_unsigned(64, 10) + vf(13 downto 4);
            -- S39: blanking gate on the generated, aligned avid
            if s_sy38(3) = '1' then o_y <= s38_y; else o_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    ------------------------------------------------------------------------
    -- SOURCE FIELD (the shell's [D], lifted from Kinetic riley): camera luma, or
    -- the Osc lattice (640 x 540 virtual px triangles, drifting), -> box (64 px at
    -- 1080, 32 px otherwise; edge-replicated and reloaded at every line start) ->
    -- vertical IIR in a half-resolution line store -> interpolated -> * K6 -> a
    -- phase.  Free-running, tapped shallow: the bend sits a few pixels beside the
    -- subject, which is never shown.  Black input = 0 = the clean figure.
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
        variable v_q2  : unsigned(13 downto 0);
        variable v_c   : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: two triangle waves (512 virtual px) from the pixel and line counters, drifting
            if s_vs_pulse = '1' then s_ot <= s_ot + 1; end if;
            if s_mcls = "10" then v_ox := s_xcnt(7 downto 0) + s_ot; v_oy := s_lacc(15 downto 8) + s_ot;
            else                  v_ox := s_xcnt(8 downto 1) + s_ot; v_oy := s_lacc(16 downto 9) + s_ot; end if;
            s_osum <= resize(f_tri7(v_ox), 8) + resize(f_tri7(v_oy), 8);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 64 (32) samples; the samples before the line start count as the first one
            if s_sv = '1' then
                bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else
                bx_wp <= (others => '0');
                if s_mcls = "00" then bx_rp <= to_unsigned(193, 8); else bx_rp <= to_unsigned(225, 8); end if;
            end if;
            if (s_mcls = "00" and s_bcnt < 64) or (s_mcls /= "00" and s_bcnt < 32) then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                if s_mcls = "00" then s_bsum <= shift_left(resize(s_sl, 14), 6);
                else                  s_bsum <= shift_left(resize(s_sl, 14), 5); end if;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + resize(s_sl, 14) - resize(v_tap, 14);
                if s_bcnt /= 64 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on the odd columns: q += (box - q) >> ksh   (box Q4, q Q5)
            if s_mcls = "00" then s_eb <= s_bsum(13 downto 2); else s_eb <= s_bsum(12 downto 1); end if;
            s_eq <= lb_q1(12 downto 0);
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb & '0', 15)) - signed(resize(s_eq, 15));
            v_st := shift_right(v_d, to_integer(s_ksh));
            v_d := signed(resize(s_eq, 15)) + v_st;
            lb_wd <= "000" & unsigned(v_d(12 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the stream: the newest column's value, held two pixels, averaged with its one-pixel delay
            -- (a linear interpolation of the half-resolution store) -> * gain -> + the per-field offset
            if lb_we = '1' then s_qh <= lb_wd(12 downto 0); end if;
            s_qc <= s_qh; s_qp <= s_qc;
            v_q2 := resize(s_qc, 14) + resize(s_qp, 14);
            s_c11 <= v_q2(13 downto 3);
            v_c := s_c11 * s_g7;
            s_camoff <= v_c(15 downto 0);
        end if;
    end process p_src;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(C_MID);
    data_out.v       <= std_logic_vector(C_MID);
    data_out.avid    <= s_sy39(3);
    data_out.hsync_n <= s_sy39(2);
    data_out.vsync_n <= s_sy39(1);
    data_out.field_n <= s_sy39(0);

end architecture blaze;
