-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- nat20.vhd
--
-- NAT20 -- a polyhedral die (d4, d6, d8, d10, d20) as a clean product shot.
--
-- v0.3.0: the MINIATURE, numbered.  The die is small (K5 Size: 5..22 % of the frame
-- height, default ~10 %) and drawn as a vector shape with exact anti-
-- aliasing on every outline and crease.  Flat-shaded faces, turned by hand
-- (K2 yaw, K3 pitch), K1 picks the die, K4 the body colour, P12 swings the
-- key light.  Rounded edges, highlights and engraved numerals stack on this
-- renderer next (see DESIGN.md).
--
-- ARCHITECTURE (streaming, one line buffer, orthographic camera):
--   GEOMETRY ENGINE: a small microcoded processor (nat20_ucode.py, assembled
--     by nat20_uasm.py; CUBIST's engine) projects the die and turns every
--     visible edge into a CROSSING RECORD: the sub-lines it crosses, its x
--     there and its slope (Q16 px), and the colour step across it.  It runs
--     DURING the field before the one it draws; the records are double-
--     buffered (a bank bit) and swap in vertical blanking.
--   RENDERER (p_rseq, p_dep): after each line's window has been displayed,
--     the next line is drawn into the line buffer at 16 sub-lines a line.
--     Each record active on the line steps its x once a sub-line and
--     deposits its colour step at the crossing's pixel p as two terms per
--     channel, D += step and F += step * frac / 16 (frac in 1/16 px).  A
--     crossing's deposits gather in a register while it stays in one pixel
--     and are written once when it moves on: one read-modify-write a run.
--     The work is per crossing, not per pixel: ~20-230 clocks a line.
--   DISPLAY (p_disp): across the window, a running sum of D minus F over
--     16 is the exact box-filtered coverage along every sub-line:
--         pixel = backdrop + (sum(D[0..p]) - F[p]) / 16
--     cleared behind the beam for the next line; then HD HDMI output
--     conditioning and the blanking gate.
--
-- Formats: see nat20_ucode.py.  Video internals are BT.601; U/V are swapped
-- at the output for the hardware.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture nat20 of program_top is

    constant C_LATENCY : integer := 12;
    constant C_XSHIFT  : integer := 3;           -- display path 9 clocks, sync tap 12
    constant C_BGT     : integer := 660;         -- backdrop luma at the top
    constant C_BGU8    : integer := 130;         -- backdrop chroma, 8-bit (BT.601)
    constant C_BGV8    : integer := 127;

    -- @@BEGIN GENERATED (nat20_regen.py)
    -- 995 microcode words; table 506/512 words
    type t_ucode is array (0 to 1023) of std_logic_vector(31 downto 0);
    constant C_UCODE : t_ucode := (
        x"90000044", x"20F18008", x"D8F1E03F", x"2101800E", x"D9020001", x"9802000A",
        x"7111E000", x"10F1E040", x"70F1E000", x"9000000F", x"79000040", x"18F203C0",
        x"7111E000", x"18F1E040", x"70F1E000", x"18F1E440", x"D8E180FF", x"60F1E380",
        x"20F1E008", x"10D223C0", x"2101800F", x"D9020001", x"98020018", x"D0026000",
        x"18D00340", x"D0026000", x"78E01000", x"28E1C002", x"10C18380", x"90000001",
        x"28C18006", x"78E00000", x"78F00000", x"7900001A", x"28F1E001", x"A8018025",
        x"10F1E040", x"28C18001", x"28E1C001", x"A001E36A", x"18F1E340", x"10E1C040",
        x"19020040", x"98020022", x"D0026000", x"A801A032", x"78E00001", x"28E1C00F",
        x"28E1C001", x"10D1A380", x"28E1800F", x"28E1C001", x"10E1C340", x"D0026000",
        x"78E00000", x"78F00001", x"28F1E00F", x"28F1E00D", x"1101C3C0", x"A0118400",
        x"18C18400", x"20E1C001", x"10E1C3C0", x"90000041", x"20E1C001", x"20F1E002",
        x"9801E03A", x"D0026000", x"78000000", x"78100001", x"CD30000A", x"C8C00000",
        x"79100005", x"60D18440", x"20D1A00A", x"A811844E", x"08D18000", x"9000005C",
        x"980A605C", x"78E00018", x"18F18380", x"38F1E000", x"60F1E440", x"20F1E00A",
        x"11018380", x"78E003FF", x"31020380", x"61020440", x"2102000A", x"382043C0",
        x"30204400", x"9000005D", x"0821A000", x"C8C00003", x"7910000C", x"60D18440",
        x"20D1A00A", x"A8118464", x"08D18000", x"90000072", x"980A6072", x"78E00010",
        x"18F18380", x"38F1E000", x"60F1E440", x"20F1E00A", x"11018380", x"78E003FF",
        x"31020380", x"61020440", x"2102000A", x"383063C0", x"30306400", x"90000073",
        x"0831A000", x"90000090", x"980A607C", x"18D18440", x"A011A039", x"08D1A000",
        x"9000007A", x"18D00340", x"78E00003", x"A011A3BD", x"09118000", x"9801E080",
        x"28C22004", x"90000083", x"78E00200", x"18C22380", x"28C18006", x"980A608E",
        x"18D18480", x"9801E087", x"9000008B", x"28D1A00F", x"28D1A001", x"20D1A00F",
        x"20D1A001", x"20D1A003", x"11224340", x"D0026000", x"09218000", x"D0026000",
        x"C8C00001", x"09110000", x"09208000", x"78F00001", x"79300096", x"90000074",
        x"08822000", x"08424000", x"C8C00002", x"09112000", x"0920A000", x"78F00001",
        x"7930009E", x"90000074", x"08922000", x"08524000", x"C8C00004", x"09116000",
        x"0920E000", x"78F00000", x"793000A6", x"90000074", x"08B22000", x"08724000",
        x"C8C0000C", x"09114000", x"0920C000", x"78F00000", x"793000AE", x"90000074",
        x"08A22000", x"08624000", x"08C08000", x"793000B3", x"90000001", x"0D31A000",
        x"08C08000", x"793000B7", x"9000001A", x"0D41A000", x"08C0A000", x"793000BB",
        x"90000001", x"0D51A000", x"08C0A000", x"793000BF", x"9000001A", x"0D61A000",
        x"0C0A8000", x"7C100000", x"0C2A6000", x"643AB4C8", x"0C4AC000", x"60CAB508",
        x"1C500300", x"60CAD4C8", x"1C600300", x"0C7AA000", x"648AD508", x"CD300006",
        x"CD400007", x"CD50000F", x"78C015C3", x"60C0E300", x"20C1800E", x"78D00666",
        x"10C18340", x"652A8308", x"C8D0000D", x"60DA4348", x"156A4340", x"2D7AA003",
        x"2D8A8003", x"18CA7540", x"20C18001", x"88018011", x"C8C00008", x"C8D00009",
        x"14902300", x"18E02340", x"34A18380", x"2CA94004", x"28C04003", x"85918000",
        x"85A18001", x"84C18002", x"84D18003", x"84E18004", x"84F18005", x"85018006",
        x"85118007", x"65BAD008", x"60CA50C8", x"1DE00300", x"65CAD048", x"60CA5108",
        x"1DF00300", x"65DAD088", x"60CA5148", x"1E000300", x"79100000", x"090B2000",
        x"BE120000", x"BE220001", x"BE320002", x"60CB7848", x"60DB9888", x"10C18340",
        x"60DBB8C8", x"10C18340", x"20C18002", x"10C195C0", x"F8022314", x"60CBD848",
        x"60DBF888", x"10C18340", x"60DC18C8", x"10C18340", x"20C18002", x"10C19600",
        x"F8022320", x"78C00003", x"11020300", x"11122040", x"A03236B6", x"28C0C001",
        x"0DE18000", x"79300111", x"90000001", x"0DC1A000", x"08CBC000", x"79300115",
        x"9000001A", x"0DD1A000", x"78C00137", x"60C19708", x"7DE00FA0", x"1DEBC300",
        x"60CBB788", x"1DF00300", x"78C00CCD", x"60C19708", x"66019788", x"78C008CD",
        x"60C19708", x"78D00385", x"10C18340", x"66119788", x"7E200A5C", x"7E300264",
        x"7E400BF4", x"653817C8", x"60C87808", x"153A6300", x"60C8D848", x"153A6300",
        x"654837C8", x"60C89808", x"154A8300", x"60C8F848", x"154A8300", x"655857C8",
        x"60C8B808", x"155AA300", x"60C91848", x"155AA300", x"65681888", x"60C878C8",
        x"156AC300", x"60C8D908", x"156AC300", x"65783888", x"60C898C8", x"157AE300",
        x"60C8F908", x"157AE300", x"65885888", x"60C8B8C8", x"158B0300", x"60C91908",
        x"158B0300", x"28C06001", x"10C180C0", x"85918028", x"85A18029", x"85B1802A",
        x"2E5A4002", x"1E501940", x"7E600000", x"10C99980", x"81018000", x"DDC2000F",
        x"21020004", x"DDD2000F", x"21020004", x"DDE2000F", x"F5FB8014", x"F62B8020",
        x"F60BA014", x"F63BA020", x"F61BC014", x"F64BC020", x"18CC17C0", x"18DC9880",
        x"60E18340", x"18CC7880", x"18DC37C0", x"60F18340", x"18E1C3C0", x"F80CC02C",
        x"A861D976", x"090CC000", x"980A2166", x"90000167", x"210CC001", x"28C20001",
        x"10C18400", x"11019400", x"BE720000", x"BE820001", x"BE920002", x"66ACF4C8",
        x"60CD1508", x"16AD4300", x"60CD3548", x"16AD4300", x"26AD4002", x"66BCF588",
        x"60CD15C8", x"16BD6300", x"60CD3608", x"16BD6300", x"26BD6002", x"66CCF0C8",
        x"60CD1108", x"16CD8300", x"60CD3148", x"16CD8300", x"26CD8002", x"3EAD4000",
        x"3EBD6000", x"78C00EB8", x"66DD4308", x"78C004CD", x"60DD6308", x"16DDA340",
        x"78C00148", x"60DD8308", x"16DDA340", x"78C00266", x"16DDA300", x"78C00060",
        x"60DDA308", x"78C0007F", x"38D1A000", x"30D1A300", x"78C00080", x"10D1A300",
        x"7111A000", x"78C00040", x"18DB2300", x"60D1A440", x"20D1A008", x"10D1A300",
        x"78C00300", x"3121A300", x"78C00200", x"18DB4300", x"60D1A440", x"20D1A008",
        x"10D1A300", x"78C003FF", x"38D1A000", x"30D1A300", x"20D1A002", x"28D1A00A",
        x"11224340", x"78C00200", x"18DB6300", x"60D1A440", x"20D1A008", x"10D1A300",
        x"78C003FF", x"38D1A000", x"30D1A300", x"20D1A002", x"28D1A00F", x"28D1A003",
        x"11224340", x"20D22003", x"28D1A00F", x"28D1A00B", x"11224340", x"F80CC4AC",
        x"166CC040", x"A05CD34D", x"7CB00000", x"7E600000", x"10C9D980", x"81018000",
        x"DD32001F", x"21020005", x"DD420003", x"21020002", x"DD52001F", x"25620007",
        x"F59A602C", x"F5AAA02C", x"980B21C7", x"980B41CA", x"90000246", x"980B41C9",
        x"900001CA", x"980AC246", x"10C994C0", x"81018000", x"D8D2000F", x"21020004",
        x"D8E2000F", x"21020004", x"D8F2000F", x"980A81D5", x"0D71A000", x"0D81C000",
        x"900001DC", x"18CA8040", x"980181DA", x"0D71C000", x"0D81E000", x"900001DC",
        x"0D71E000", x"0D81A000", x"F5BAE014", x"F5CAE020", x"F5DB0014", x"F5EB0020",
        x"1E0BD700", x"980C01E3", x"90000246", x"A070182E", x"08CB2000", x"0D9B4000",
        x"0DA18000", x"08CB6000", x"0DBBA000", x"0DD18000", x"08CB8000", x"0DCBC000",
        x"0DE18000", x"1E001800", x"1DFBB6C0", x"1E1B9280", x"18C92040", x"980181F3",
        x"900001F5", x"161C2040", x"261C2001", x"1E2BD280", x"18C92040", x"980181F9",
        x"900001FB", x"162C4040", x"262C4001", x"A89C3886", x"A07BE03F", x"08CBE000",
        x"90000200", x"18C017C0", x"28C1800C", x"18D92040", x"9801A204", x"90000205",
        x"28C18001", x"08DC0000", x"20EC0001", x"10C18380", x"7930020A", x"9000001E",
        x"0E31C000", x"A88BE00D", x"1E3018C0", x"2E4B600C", x"78C00800", x"164C8300",
        x"08CC2000", x"18D92040", x"9801A214", x"90000215", x"10C18300", x"10C19280",
        x"18C19700", x"98018219", x"9000021B", x"20CC6001", x"164C8300", x"D8CB23FF",
        x"D8DB43FF", x"1E518340", x"DE5CA7FF", x"20CB200A", x"D8C180FF", x"20DB400A",
        x"D8D1A0FF", x"18C18340", x"D8C181FF", x"28C1800B", x"165CA300", x"20CB200F",
        x"20C18003", x"D8C180FF", x"20DB400F", x"20D1A003", x"D8D1A0FF", x"18C18340",
        x"D8C181FF", x"28C1800F", x"28C18005", x"165CA300", x"980B2238", x"78C00003",
        x"28C1800F", x"28C1800E", x"165CA300", x"9000023D", x"980B423D", x"78C00001",
        x"28C1800F", x"28C1800E", x"165CA300", x"28CC400F", x"10C19840", x"880192C0",
        x"880C92C1", x"880C72C2", x"880CB2C3", x"14B96040", x"78C00020", x"A8996308",
        x"166CC040", x"A06CD3FA", x"7D700000", x"28C04001", x"10C18080", x"78D00065",
        x"10C18340", x"E5318000", x"10C18040", x"75418000", x"10C18040", x"75518000",
        x"78C0007A", x"70D18000", x"08E06000", x"9801C257", x"9000025A", x"20D1A001",
        x"18E1C040", x"90000255", x"D8D1A001", x"28C1A001", x"10C18340", x"78D00074",
        x"15618340", x"0D99A000", x"980A2262", x"90000263", x"2599A001", x"7D800000",
        x"091B0000", x"980A2267", x"90000268", x"111B1600", x"F5A2202C", x"980B426B",
        x"900003D7", x"10C98440", x"85B18000", x"85C18001", x"7DE00001", x"98004271",
        x"7DE00003", x"7DD00000", x"79200073", x"20CB6000", x"D8C1800F", x"F8024300",
        x"F8024304", x"20CB6004", x"D8C1800F", x"F8024301", x"F8024305", x"20CB6008",
        x"D8C1800F", x"F8024302", x"F8024306", x"D8CB600F", x"78E00000", x"980A2286",
        x"20DB6004", x"D8D1A00F", x"F8024344", x"9000028D", x"20CB8004", x"78E00018",
        x"78D00003", x"A8A0434C", x"20CB8008", x"78E0000C", x"D8C1800F", x"F8024303",
        x"78D00041", x"1121C340", x"20FB600C", x"D8F1E003", x"98004294", x"08FBA000",
        x"F0C1E073", x"15F18040", x"98004298", x"9000029D", x"15FB0040", x"78C00003",
        x"18C04300", x"9801829D", x"0DFB0000", x"7E300000", x"7E400000", x"7E500000",
        x"7E600000", x"7E700000", x"7E800000", x"7DC00000", x"10C1F700", x"F1018073",
        x"F1120014", x"F1020020", x"10C25700", x"E0D18000", x"60E22348", x"163C6380",
        x"60E20348", x"164C8380", x"10C25700", x"78D00004", x"10C18340", x"E0D18000",
        x"60E22348", x"165CA380", x"60E20348", x"166CC380", x"10C25700", x"78D00008",
        x"10C18340", x"E0D18000", x"60E22348", x"167CE380", x"60E20348", x"168D0380",
        x"15CB8040", x"78C00004", x"A0AB8324", x"7E10000F", x"7E200010", x"0E0BE000",
        x"78C0000A", x"A0BBE30D", x"7E200020", x"7E000001", x"1E1BE300", x"A0BC2317",
        x"7E000002", x"1E1C2300", x"900002D7", x"78C00003", x"A0B04317", x"78C00006",
        x"18CBE300", x"980182D3", x"7E00000A", x"78C00009", x"18CBE300", x"980182D7",
        x"7E00000B", x"60CCB4C8", x"10C198C0", x"2E318007", x"60CCB548", x"1E500300",
        x"667CF508", x"60CCF880", x"20C18001", x"1E3C6300", x"28CCA003", x"28DCA001",
        x"10C18340", x"1E3C6300", x"60CCD4C8", x"10C19900", x"2E418007", x"60CCD548",
        x"1E600300", x"668D1508", x"60CD1880", x"20C18001", x"1E4C8300", x"28CCC003",
        x"28DCC001", x"10C18340", x"1E4C8300", x"669CF980", x"60CCBA00", x"1E9D2300",
        x"20DD2007", x"78C00410", x"A0F1A315", x"78C00001", x"28C1800F", x"28C1800A",
        x"793002FC", x"9000001E", x"0EA1C000", x"66BCDA80", x"26BD6009", x"A0CD6002",
        x"08CD6000", x"90000303", x"18C01AC0", x"20C1800E", x"980183D5", x"66CD1A80",
        x"26CD8009", x"1EC01B00", x"A0CD800B", x"08CD8000", x"9000030C", x"18C01B00",
        x"20C1800E", x"980183D5", x"66DCBA80", x"26DDA009", x"1ED01B40", x"A0CDA014",
        x"08CDA000", x"90000315", x"18C01B40", x"20C1800E", x"980183D5", x"66ECFA80",
        x"26EDC009", x"A0CDC01C", x"08CDC000", x"9000031D", x"18C01B80", x"20C1800E",
        x"980183D5", x"60CCF880", x"28DCA002", x"10E1B940", x"28D1C002", x"0EFC6000",
        x"0F0C6000", x"A0C18028", x"170E0300", x"90000329", x"16FDE300", x"A0C1A02C",
        x"170E0340", x"9000032D", x"16FDE340", x"60CD1880", x"28DCC002", x"10E1B980",
        x"28D1C002", x"0F1C8000", x"0F2C8000", x"A0C18036", x"172E4300", x"90000337",
        x"171E2300", x"A0C1A03A", x"172E4340", x"9000033B", x"171E2340", x"26FDE00B",
        x"1EFDE040", x"3EFDE000", x"270E000B", x"78C00002", x"170E0300", x"C8C0000F",
        x"18C18040", x"370E0300", x"1F0E1BC0", x"271E2007", x"1F1E3280", x"271E2004",
        x"18C92040", x"9801834B", x"9000034C", x"271E2001", x"272E4007", x"1F2E5280",
        x"272E4004", x"18C92040", x"98018352", x"90000353", x"272E4001", x"78C00002",
        x"172E4300", x"28CDE00B", x"78D00400", x"10C18340", x"191198C0", x"21122004",
        x"28CE2004", x"78D00008", x"10C18340", x"60C19240", x"10C19280", x"28C18007",
        x"19219900", x"21224004", x"65CD6440", x"60CDA480", x"15CB8300", x"25CB8007",
        x"78C00800", x"1DCB8300", x"25CB8004", x"66AD8440", x"60CDC480", x"16AD4300",
        x"26AD4007", x"78C00800", x"1EAD4300", x"26AD4004", x"18C92040", x"98018373",
        x"90000375", x"16DDBB40", x"16EDDB80", x"08CD2000", x"79300378", x"90000036",
        x"20E1C007", x"78C00010", x"38E1C300", x"78C000FF", x"3691C300", x"78C00004",
        x"111AE300", x"28CE400B", x"10C19C40", x"28DC000F", x"28D1A007", x"10C18340",
        x"28DC200F", x"28D1A00B", x"10C18340", x"88018448", x"28CE0009", x"10C19BC0",
        x"28DD200F", x"28D1A002", x"10C18340", x"88018449", x"08CD8000", x"08DD6000",
        x"79300392", x"9000002D", x"8801C44A", x"08CDC000", x"08DDA000", x"79300397",
        x"9000002D", x"8801C44B", x"08CD4000", x"08DB8000", x"7930039C", x"9000002D",
        x"8801C44D", x"212B400F", x"2122400B", x"D922403F", x"29224003", x"70DAC000",
        x"78C00040", x"18D1A300", x"60D1A480", x"20D1A008", x"10D1A300", x"78C00300",
        x"30D1A300", x"D8CB43FF", x"1E01A300", x"DE0C07FF", x"10DAC040", x"70D1A000",
        x"78C00200", x"18D1A300", x"60D1A480", x"20D1A008", x"10D1A300", x"78C003FF",
        x"38D1A000", x"30D1A300", x"20D1A002", x"20CB400A", x"D8C180FF", x"18D1A300",
        x"D8D1A1FF", x"28D1A00B", x"160C0340", x"10DAC040", x"10D1A040", x"70D1A000",
        x"78C00200", x"18D1A300", x"60D1A480", x"20D1A008", x"10D1A300", x"78C003FF",
        x"38D1A000", x"30D1A300", x"20D1A002", x"20CB400F", x"20C18003", x"D8C180FF",
        x"18D1A300", x"D8D1A1FF", x"28D1A00F", x"28D1A005", x"160C0340", x"880C044C",
        x"157AE040", x"78C0000C", x"A8FAE319", x"15DBA040", x"A09BB7B2", x"158B0040",
        x"A09B1664", x"880AE012", x"C8C0000E", x"78D03F1A", x"60C18340", x"20C18004",
        x"60C19240", x"88018014", x"88096010", x"88000019", x"C0000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000", x"00000000", x"00000000",
        x"00000000", x"00000000", x"00000000", x"00000000" );
    type t_tbl is array (0 to 511) of std_logic_vector(15 downto 0);
    constant C_TABLE : t_tbl := (
        x"004C", x"0004", x"0058", x"0004", x"005C", x"0006", x"0062", x"0000", x"006E", x"0008", x"0086", x"000C",
        x"0092", x"0012", x"00A4", x"0001", x"00B6", x"0006", x"00C8", x"0008", x"00D0", x"000C", x"00DC", x"0000",
        x"00F4", x"000C", x"0118", x"0014", x"012C", x"001E", x"014A", x"0001", x"0168", x"000C", x"018C", x"0014",
        x"01A0", x"001E", x"01BE", x"0000", x"0106", x"01C9", x"02E7", x"00BB", x"026F", x"01DD", x"0079", x"0207",
        x"01FF", x"032D", x"01CC", x"0219", x"014E", x"01DF", x"014F", x"0122", x"0293", x"0242", x"02FD", x"0240",
        x"01B0", x"01EF", x"012B", x"030B", x"015F", x"0251", x"012F", x"0267", x"0213", x"02C7", x"0131", x"020C",
        x"01FB", x"02AD", x"00BD", x"0295", x"FCEA", x"38C9", x"E2A3", x"CA43", x"DF28", x"F4A1", x"3233", x"DF28",
        x"E9B4", x"06A0", x"08E8", x"3F08", x"0201", x"0302", x"0321", x"0103", x"0023", x"00A0", x"01A1", x"2002",
        x"2143", x"20A2", x"F960", x"F718", x"C0F8", x"35BD", x"20D8", x"0B5F", x"0316", x"C737", x"1D5D", x"CDCD",
        x"20D8", x"164C", x"3489", x"ED46", x"1F64", x"1B3F", x"CF37", x"E0CC", x"2946", x"30C9", x"0385", x"0FFC",
        x"12BA", x"C4ED", x"F004", x"ED46", x"3B13", x"D6BA", x"CF37", x"FC7B", x"E4C1", x"30C9", x"1F34", x"CB77",
        x"12BA", x"E09C", x"0137", x"0517", x"2457", x"2647", x"0326", x"0736", x"2015", x"2405", x"0102", x"0312",
        x"2204", x"2624", x"43C6", x"44C8", x"40C0", x"45CA", x"42C4", x"41C2", x"1328", x"042A", x"0527", x"1029",
        x"0321", x"24A4", x"022B", x"0025", x"1147", x"25A3", x"0141", x"22C3", x"EA19", x"E5F8", x"C9CA", x"C4A9",
        x"0000", x"17FA", x"F640", x"3A78", x"E7DD", x"09C0", x"C588", x"1823", x"3B57", x"0000", x"E806", x"15E7",
        x"1A08", x"3636", x"C896", x"0D6E", x"E2EF", x"376A", x"F292", x"1D11", x"1F97", x"0D6E", x"C9FC", x"E069",
        x"F292", x"3604", x"FAC4", x"C2E2", x"EDBE", x"053C", x"3D1E", x"1242", x"0204", x"0215", x"0314", x"0305",
        x"2412", x"2502", x"2403", x"2513", x"02A0", x"0323", x"0026", x"01A5", x"0221", x"03A2", x"0124", x"00A7",
        x"2240", x"22C1", x"2342", x"23C3", x"EF39", x"EC39", x"C57E", x"3541", x"2349", x"FC23", x"0ABB", x"CD35",
        x"256D", x"D0CB", x"2349", x"18F2", x"2F35", x"DCB7", x"E70E", x"F545", x"32CB", x"DA93", x"CABF", x"DCB7",
        x"03DD", x"10C7", x"13C7", x"3A82", x"1F7C", x"08A6", x"F0CB", x"0F61", x"FE9B", x"DF76", x"F09F", x"0165",
        x"208A", x"DD37", x"08A6", x"FC7B", x"FA2D", x"0D21", x"DEF8", x"05D3", x"F2DF", x"2108", x"020A", x"3EEA",
        x"0B8E", x"E66C", x"FE9B", x"E6AF", x"1994", x"0165", x"1951", x"22C9", x"F75A", x"0385", x"E084", x"F75A",
        x"0F35", x"FDF6", x"C116", x"F472", x"0826", x"0852", x"059B", x"0589", x"0406", x"0410", x"07AB", x"073A",
        x"0236", x"02A3", x"091B", x"0901", x"0086", x"0098", x"0A5B", x"0A25", x"0346", x"0374", x"017B", x"0147",
        x"62A4", x"564D", x"5953", x"65AA", x"5449", x"60A0", x"5851", x"5143", x"67AE", x"5347", x"0585", x"024C",
        x"16AB", x"19A5", x"0552", x"0781", x"0048", x"14AF", x"0450", x"18A7", x"0389", x"0844", x"0991", x"11A1",
        x"0742", x"200C", x"0946", x"018D", x"014A", x"034E", x"0AAF", x"11A4", x"3C96", x"2821", x"DB5F", x"21D3",
        x"1A24", x"24A1", x"D27E", x"C674", x"E71B", x"F329", x"CE51", x"18E5", x"1FBF", x"31AF", x"E71B", x"E041",
        x"398C", x"18E5", x"0CD7", x"E5DC", x"DB5F", x"2D82", x"D7DF", x"24A1", x"DE2D", x"F551", x"EE5C", x"C36A",
        x"EEA5", x"FD3E", x"3D8A", x"0A06", x"34AB", x"22F4", x"CC23", x"219B", x"10A5", x"34CD", x"219B", x"F2A2",
        x"D0AB", x"02C2", x"D502", x"0880", x"C7EC", x"1DA6", x"CB33", x"DE65", x"0D5E", x"2F55", x"FD3E", x"2AFE",
        x"F780", x"3814", x"E25A", x"115B", x"02C2", x"C276", x"F5FA", x"CB55", x"DD0C", x"33DD", x"DE65", x"EF5B",
        x"294A", x"273B", x"1206", x"2849", x"1605", x"1713", x"0B57", x"2102", x"1812", x"1318", x"165A", x"1A5B",
        x"2B9A", x"1824", x"0A46", x"1938", x"0507", x"1B39", x"1426", x"0701", x"09A7", x"03A2", x"0230", x"0124",
        x"0833", x"0447", x"02A9", x"29A5", x"04A8", x"06B2", x"0942", x"242D", x"22A1", x"07C9", x"08AF", x"00B1",
        x"0752", x"26A3", x"01A0", x"002E", x"222A", x"0350", x"052B", x"05A6", x"274A", x"2341", x"27C3", x"0640",
        x"28AC", x"264B", x"EF39", x"EC39", x"C57E", x"3FC2", x"FED8", x"0572", x"CCD1", x"FED8", x"2667", x"EFDC",
        x"19D6", x"C7B7", x"E623", x"D939", x"2BDC", x"2E35", x"2308", x"1B17", x"2D2D", x"D939", x"177D", x"E72B",
        x"2308", x"2F75", x"EAE2", x"3BB6", x"0935", x"16CA", x"3BB6", x"FC9E", x"E936", x"C44A", x"0362", x"151E",
        x"C44A", x"F6CB", x"18D5", x"DCF8", x"D08B", x"D2D3", x"26C7", x"E883", x"D1CB", x"DCF8", x"E4E9", x"19DD",
        x"26C7", x"D424", x"1024", x"E62A", x"3849", x"332F", x"0128", x"D999", x"C03E", x"0128", x"FA8E", x"10C7",
        x"13C7", x"3A82", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000" );
    type t_erom is array (0 to 255) of std_logic_vector(15 downto 0);
    constant C_EROM : t_erom := (
        x"0000", x"0065", x"00C9", x"012D", x"0191", x"01F5", x"0259", x"02BC", x"031F", x"0381", x"03E3", x"0444",
        x"04A5", x"0505", x"0564", x"05C2", x"061F", x"067C", x"06D7", x"0732", x"078B", x"07E3", x"083A", x"088F",
        x"08E4", x"0937", x"0988", x"09D8", x"0A26", x"0A73", x"0ABF", x"0B08", x"0B50", x"0B97", x"0BDB", x"0C1E",
        x"0C5E", x"0C9D", x"0CDA", x"0D15", x"0D4E", x"0D85", x"0DB9", x"0DEC", x"0E1C", x"0E4B", x"0E77", x"0EA1",
        x"0EC8", x"0EEE", x"0F11", x"0F31", x"0F50", x"0F6C", x"0F85", x"0F9C", x"0FB1", x"0FC4", x"0FD4", x"0FE1",
        x"0FEC", x"0FF5", x"0FFB", x"0FFF", x"1000", x"0555", x"0555", x"0556", x"0000", x"0AAB", x"FAAB", x"FAAA",
        x"0000", x"0000", x"F000", x"1000", x"0000", x"0400", x"0400", x"0400", x"0400", x"0400", x"0400", x"FC00",
        x"FC00", x"1000", x"F000", x"0000", x"0000", x"0000", x"0000", x"0000", x"1000", x"1000", x"0000", x"0000",
        x"F000", x"0000", x"F000", x"1000", x"0000", x"0800", x"1265", x"1FDC", x"0000", x"3B1F", x"763E", x"FF6A",
        x"288C", x"463B", x"0614", x"39FD", x"2123", x"FF9B", x"1B02", x"3A7A", x"036A", x"01F1", x"0206", x"0071",
        x"0204", x"01FF", x"0848", x"0000", x"0000", x"0000", x"0000", x"0000", x"0000", x"0020", x"002C", x"0035",
        x"003C", x"0043", x"0049", x"004E", x"0053", x"0057", x"005C", x"0060", x"0063", x"0067", x"006B", x"006E",
        x"0071", x"0075", x"0078", x"007B", x"007D", x"0080", x"0083", x"0086", x"0088", x"008B", x"008D", x"0090",
        x"0092", x"0095", x"0097", x"0099", x"009B", x"009E", x"00A0", x"00A2", x"00A4", x"00A6", x"00A8", x"00AA",
        x"00AC", x"00AE", x"00B0", x"00B2", x"00B4", x"00B5", x"00B7", x"00B9", x"00BB", x"00BD", x"00BE", x"00C0",
        x"00C2", x"00C3", x"00C5", x"00C7", x"00C8", x"00CA", x"00CC", x"00CD", x"00CF", x"00D0", x"00D2", x"00D3",
        x"00D5", x"00D6", x"00D8", x"00D9", x"00DB", x"00DC", x"00DE", x"00DF", x"00E1", x"00E2", x"00E3", x"00E5",
        x"00E6", x"00E8", x"00E9", x"00EA", x"00EC", x"00ED", x"00EE", x"00F0", x"00F1", x"00F2", x"00F4", x"00F5",
        x"00F6", x"00F7", x"00F9", x"00FA", x"00FB", x"00FC", x"00FE", x"00FF", x"0100", x"0101", x"0102", x"0104",
        x"0105", x"0106", x"0107", x"0108", x"0109", x"010B", x"010C", x"010D", x"010E", x"010F", x"0110", x"0111",
        x"0113", x"0114", x"0115", x"0116", x"0117", x"0118", x"0119", x"011A", x"011B", x"011C", x"011D", x"011F",
        x"0120", x"0121", x"0122", x"0123" );
    type t_gly is array (0 to 1023) of std_logic_vector(3 downto 0);
    constant C_GLY0 : t_gly := (
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"A", x"8", x"D", x"F", x"F", x"F", x"F", x"5", x"4", x"7", x"0", x"C", x"F",
        x"F", x"C", x"0", x"E", x"F", x"7", x"5", x"F", x"F", x"A", x"2", x"F", x"F", x"A", x"2", x"F", x"F", x"9", x"2", x"F", x"F", x"A", x"1", x"F",
        x"F", x"B", x"1", x"F", x"F", x"9", x"3", x"F", x"F", x"F", x"1", x"A", x"F", x"4", x"8", x"F", x"F", x"F", x"B", x"2", x"0", x"6", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"9", x"D", x"F", x"F",
        x"F", x"F", x"E", x"4", x"0", x"D", x"F", x"F", x"F", x"F", x"2", x"7", x"0", x"D", x"F", x"F", x"F", x"F", x"E", x"F", x"0", x"D", x"F", x"F",
        x"F", x"F", x"F", x"F", x"0", x"D", x"F", x"F", x"F", x"F", x"F", x"F", x"0", x"D", x"F", x"F", x"F", x"F", x"F", x"F", x"0", x"D", x"F", x"F",
        x"F", x"F", x"8", x"1", x"1", x"1", x"7", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"A", x"8", x"C", x"F", x"F", x"F", x"F", x"5", x"4", x"7", x"0", x"A", x"F", x"F", x"E", x"4", x"E", x"F", x"6", x"6", x"F",
        x"F", x"F", x"F", x"F", x"F", x"3", x"9", x"F", x"F", x"F", x"F", x"F", x"7", x"4", x"F", x"F", x"F", x"F", x"F", x"7", x"4", x"F", x"F", x"F",
        x"F", x"F", x"7", x"3", x"C", x"C", x"C", x"F", x"F", x"C", x"1", x"1", x"1", x"1", x"4", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"A", x"8", x"B", x"F", x"F", x"F", x"F", x"6", x"3", x"7", x"0", x"9", x"F",
        x"F", x"F", x"5", x"D", x"F", x"7", x"6", x"F", x"F", x"F", x"F", x"F", x"B", x"3", x"A", x"F", x"F", x"F", x"F", x"E", x"6", x"1", x"A", x"F",
        x"F", x"F", x"C", x"F", x"F", x"8", x"4", x"F", x"F", x"E", x"0", x"B", x"F", x"5", x"6", x"F", x"F", x"F", x"9", x"2", x"0", x"5", x"E", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"E", x"9", x"E", x"F",
        x"F", x"F", x"F", x"F", x"5", x"0", x"E", x"F", x"F", x"F", x"F", x"8", x"5", x"1", x"E", x"F", x"F", x"F", x"B", x"2", x"F", x"1", x"E", x"F",
        x"F", x"E", x"2", x"B", x"F", x"1", x"E", x"F", x"F", x"8", x"1", x"3", x"3", x"0", x"3", x"F", x"F", x"F", x"E", x"E", x"E", x"1", x"E", x"F",
        x"F", x"F", x"F", x"F", x"F", x"1", x"E", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"D", x"9", x"9", x"9", x"A", x"F", x"F", x"F", x"B", x"3", x"5", x"5", x"B", x"F", x"F", x"F", x"8", x"7", x"F", x"F", x"F", x"F",
        x"F", x"F", x"5", x"4", x"3", x"7", x"F", x"F", x"F", x"F", x"C", x"C", x"C", x"3", x"8", x"F", x"F", x"F", x"F", x"F", x"F", x"7", x"5", x"F",
        x"F", x"F", x"7", x"E", x"E", x"3", x"9", x"F", x"F", x"F", x"6", x"1", x"1", x"7", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"B", x"9", x"F", x"F", x"F", x"F", x"F", x"C", x"0", x"A", x"F", x"F",
        x"F", x"F", x"F", x"3", x"8", x"F", x"F", x"F", x"F", x"F", x"6", x"5", x"6", x"8", x"F", x"F", x"F", x"D", x"0", x"8", x"B", x"3", x"7", x"F",
        x"F", x"B", x"2", x"F", x"F", x"9", x"3", x"F", x"F", x"E", x"0", x"C", x"F", x"5", x"6", x"F", x"F", x"F", x"9", x"2", x"0", x"5", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"C", x"9", x"9", x"9", x"9", x"9", x"F",
        x"F", x"C", x"5", x"5", x"5", x"5", x"5", x"F", x"F", x"F", x"F", x"F", x"F", x"1", x"B", x"F", x"F", x"F", x"F", x"F", x"8", x"4", x"F", x"F",
        x"F", x"F", x"F", x"F", x"1", x"B", x"F", x"F", x"F", x"F", x"F", x"8", x"3", x"F", x"F", x"F", x"F", x"F", x"F", x"1", x"A", x"F", x"F", x"F",
        x"F", x"F", x"8", x"3", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"A", x"8", x"C", x"F", x"F", x"F", x"F", x"5", x"6", x"9", x"0", x"B", x"F", x"F", x"F", x"0", x"D", x"F", x"6", x"7", x"F",
        x"F", x"F", x"5", x"7", x"A", x"1", x"B", x"F", x"F", x"F", x"6", x"4", x"6", x"2", x"B", x"F", x"F", x"C", x"0", x"F", x"F", x"8", x"4", x"F",
        x"F", x"D", x"0", x"C", x"F", x"6", x"5", x"F", x"F", x"F", x"8", x"2", x"0", x"4", x"E", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"A", x"8", x"C", x"F", x"F", x"F", x"F", x"5", x"5", x"7", x"0", x"A", x"F",
        x"F", x"D", x"0", x"F", x"F", x"8", x"5", x"F", x"F", x"D", x"0", x"C", x"F", x"6", x"5", x"F", x"F", x"F", x"8", x"1", x"2", x"0", x"A", x"F",
        x"F", x"F", x"F", x"F", x"6", x"5", x"F", x"F", x"F", x"F", x"F", x"9", x"1", x"E", x"F", x"F", x"F", x"F", x"B", x"1", x"B", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"E", x"D", x"F", x"F", x"F", x"F", x"F", x"F", x"2", x"8", x"F", x"F",
        x"F", x"F", x"F", x"5", x"6", x"F", x"F", x"F", x"F", x"F", x"8", x"3", x"9", x"B", x"F", x"F", x"F", x"F", x"0", x"5", x"8", x"0", x"9", x"F",
        x"F", x"B", x"1", x"F", x"F", x"9", x"3", x"F", x"F", x"D", x"1", x"F", x"F", x"7", x"5", x"F", x"F", x"F", x"6", x"2", x"3", x"2", x"D", x"F",
        x"F", x"F", x"D", x"D", x"C", x"D", x"D", x"F", x"F", x"F", x"4", x"4", x"4", x"4", x"7", x"F", x"F", x"F", x"F", x"D", x"C", x"F", x"F", x"F",
        x"F", x"F", x"7", x"1", x"4", x"1", x"C", x"F", x"F", x"E", x"0", x"D", x"F", x"7", x"6", x"F", x"F", x"D", x"0", x"E", x"F", x"7", x"5", x"F",
        x"F", x"F", x"5", x"2", x"4", x"0", x"9", x"F", x"F", x"F", x"F", x"D", x"9", x"3", x"F", x"F", x"F", x"F", x"F", x"B", x"0", x"C", x"F", x"F",
        x"F", x"F", x"E", x"1", x"8", x"F", x"F", x"F", x"F", x"F", x"D", x"D", x"D", x"D", x"D", x"F", x"F", x"F", x"4", x"4", x"4", x"4", x"7", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F" );
    constant C_GLY1 : t_gly := (
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"D", x"8", x"A", x"F", x"F", x"F", x"F", x"C", x"0", x"7", x"4", x"5", x"F", x"F",
        x"F", x"5", x"7", x"F", x"E", x"0", x"C", x"F", x"F", x"2", x"A", x"F", x"F", x"2", x"A", x"F", x"F", x"1", x"A", x"F", x"F", x"2", x"9", x"F",
        x"F", x"3", x"9", x"F", x"F", x"1", x"B", x"F", x"F", x"8", x"4", x"F", x"A", x"1", x"F", x"F", x"F", x"F", x"6", x"0", x"2", x"B", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"B", x"9", x"F", x"F", x"F",
        x"F", x"F", x"9", x"0", x"5", x"F", x"F", x"F", x"F", x"8", x"2", x"8", x"5", x"F", x"F", x"F", x"F", x"F", x"F", x"8", x"5", x"F", x"F", x"F",
        x"F", x"F", x"F", x"8", x"5", x"F", x"F", x"F", x"F", x"F", x"F", x"8", x"5", x"F", x"F", x"F", x"F", x"F", x"F", x"8", x"5", x"F", x"F", x"F",
        x"F", x"F", x"1", x"1", x"1", x"1", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"D", x"8", x"9", x"F", x"F", x"F", x"F", x"C", x"0", x"7", x"5", x"3", x"F", x"F", x"F", x"6", x"7", x"F", x"E", x"0", x"E", x"F",
        x"F", x"F", x"F", x"F", x"A", x"1", x"F", x"F", x"F", x"F", x"F", x"D", x"1", x"A", x"F", x"F", x"F", x"F", x"D", x"2", x"9", x"F", x"F", x"F",
        x"F", x"D", x"2", x"9", x"C", x"C", x"F", x"F", x"F", x"4", x"1", x"1", x"1", x"1", x"C", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"D", x"9", x"9", x"F", x"F", x"F", x"F", x"D", x"0", x"7", x"5", x"3", x"F", x"F",
        x"F", x"7", x"6", x"F", x"F", x"0", x"E", x"F", x"F", x"F", x"F", x"C", x"8", x"4", x"F", x"F", x"F", x"F", x"F", x"6", x"4", x"5", x"F", x"F",
        x"F", x"F", x"E", x"F", x"F", x"1", x"C", x"F", x"F", x"6", x"5", x"F", x"B", x"0", x"D", x"F", x"F", x"F", x"5", x"0", x"2", x"9", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"9", x"9", x"F", x"F",
        x"F", x"F", x"F", x"B", x"0", x"6", x"F", x"F", x"F", x"F", x"E", x"2", x"9", x"6", x"F", x"F", x"F", x"F", x"5", x"8", x"9", x"6", x"F", x"F",
        x"F", x"8", x"5", x"F", x"9", x"6", x"F", x"F", x"F", x"1", x"3", x"3", x"3", x"2", x"7", x"F", x"F", x"E", x"E", x"E", x"9", x"6", x"E", x"F",
        x"F", x"F", x"F", x"F", x"9", x"6", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"9", x"9", x"9", x"9", x"F", x"F", x"F", x"F", x"3", x"5", x"5", x"5", x"F", x"F", x"F", x"F", x"0", x"F", x"F", x"F", x"F", x"F",
        x"F", x"D", x"0", x"3", x"4", x"B", x"F", x"F", x"F", x"F", x"A", x"D", x"9", x"0", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"0", x"D", x"F",
        x"F", x"C", x"A", x"F", x"9", x"2", x"F", x"F", x"F", x"B", x"3", x"0", x"3", x"C", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"9", x"9", x"F", x"F", x"F", x"F", x"F", x"6", x"4", x"F", x"F", x"F",
        x"F", x"F", x"9", x"2", x"E", x"F", x"F", x"F", x"F", x"D", x"0", x"7", x"6", x"C", x"F", x"F", x"F", x"6", x"3", x"B", x"8", x"0", x"E", x"F",
        x"F", x"3", x"A", x"F", x"F", x"1", x"B", x"F", x"F", x"6", x"6", x"F", x"B", x"0", x"E", x"F", x"F", x"F", x"5", x"0", x"2", x"A", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"9", x"9", x"9", x"9", x"9", x"C", x"F",
        x"F", x"5", x"5", x"5", x"5", x"0", x"C", x"F", x"F", x"F", x"F", x"F", x"8", x"4", x"F", x"F", x"F", x"F", x"F", x"F", x"1", x"B", x"F", x"F",
        x"F", x"F", x"F", x"8", x"4", x"F", x"F", x"F", x"F", x"F", x"F", x"1", x"B", x"F", x"F", x"F", x"F", x"F", x"8", x"3", x"F", x"F", x"F", x"F",
        x"F", x"F", x"1", x"A", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"C", x"8", x"A", x"F", x"F", x"F", x"F", x"B", x"0", x"9", x"6", x"4", x"F", x"F", x"F", x"7", x"6", x"F", x"E", x"0", x"F", x"F",
        x"F", x"B", x"1", x"A", x"7", x"5", x"F", x"F", x"F", x"B", x"2", x"6", x"4", x"6", x"F", x"F", x"F", x"4", x"8", x"F", x"F", x"0", x"C", x"F",
        x"F", x"5", x"6", x"F", x"C", x"0", x"D", x"F", x"F", x"E", x"4", x"0", x"2", x"8", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"D", x"8", x"9", x"F", x"F", x"F", x"F", x"B", x"0", x"7", x"5", x"4", x"F", x"F",
        x"F", x"5", x"7", x"F", x"F", x"0", x"D", x"F", x"F", x"6", x"6", x"F", x"C", x"0", x"D", x"F", x"F", x"D", x"3", x"1", x"5", x"3", x"F", x"F",
        x"F", x"F", x"F", x"C", x"0", x"C", x"F", x"F", x"F", x"F", x"F", x"3", x"8", x"F", x"F", x"F", x"F", x"F", x"5", x"4", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"D", x"D", x"F", x"F", x"F", x"F", x"F", x"8", x"2", x"E", x"F", x"F",
        x"F", x"F", x"B", x"0", x"C", x"F", x"F", x"F", x"F", x"F", x"2", x"9", x"A", x"F", x"F", x"F", x"F", x"7", x"1", x"8", x"5", x"2", x"F", x"F",
        x"F", x"3", x"9", x"F", x"F", x"1", x"B", x"F", x"F", x"5", x"8", x"F", x"E", x"0", x"D", x"F", x"F", x"C", x"2", x"3", x"1", x"7", x"F", x"F",
        x"F", x"D", x"D", x"C", x"D", x"D", x"F", x"F", x"F", x"7", x"4", x"4", x"4", x"4", x"F", x"F", x"F", x"F", x"F", x"C", x"D", x"F", x"F", x"F",
        x"F", x"D", x"2", x"4", x"2", x"6", x"F", x"F", x"F", x"6", x"6", x"F", x"E", x"0", x"E", x"F", x"F", x"5", x"7", x"F", x"F", x"0", x"D", x"F",
        x"F", x"B", x"0", x"4", x"2", x"2", x"F", x"F", x"F", x"F", x"F", x"D", x"2", x"A", x"F", x"F", x"F", x"F", x"F", x"5", x"6", x"F", x"F", x"F",
        x"F", x"F", x"7", x"2", x"F", x"F", x"F", x"F", x"F", x"D", x"D", x"D", x"D", x"D", x"F", x"F", x"F", x"7", x"4", x"4", x"4", x"4", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F" );
    constant C_GLY2 : t_gly := (
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"B", x"2", x"1", x"6", x"F", x"F", x"F", x"F", x"1", x"A", x"F", x"3", x"8", x"F",
        x"F", x"B", x"1", x"F", x"F", x"9", x"3", x"F", x"F", x"9", x"2", x"F", x"F", x"A", x"1", x"F", x"F", x"A", x"2", x"F", x"F", x"A", x"2", x"F",
        x"F", x"C", x"0", x"E", x"F", x"7", x"5", x"F", x"F", x"F", x"5", x"4", x"7", x"0", x"C", x"F", x"F", x"F", x"F", x"A", x"8", x"D", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"A", x"1", x"D", x"F", x"F",
        x"F", x"F", x"8", x"1", x"0", x"D", x"F", x"F", x"F", x"F", x"6", x"D", x"0", x"D", x"F", x"F", x"F", x"F", x"F", x"F", x"0", x"D", x"F", x"F",
        x"F", x"F", x"F", x"F", x"0", x"D", x"F", x"F", x"F", x"F", x"F", x"F", x"0", x"D", x"F", x"F", x"F", x"F", x"8", x"8", x"0", x"8", x"8", x"F",
        x"F", x"F", x"9", x"9", x"9", x"9", x"9", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"B", x"2", x"0", x"4", x"F", x"F", x"F", x"F", x"1", x"A", x"F", x"4", x"7", x"F", x"F", x"F", x"C", x"F", x"F", x"5", x"6", x"F",
        x"F", x"F", x"F", x"F", x"C", x"0", x"C", x"F", x"F", x"F", x"F", x"D", x"2", x"A", x"F", x"F", x"F", x"F", x"D", x"2", x"9", x"F", x"F", x"F",
        x"F", x"E", x"2", x"6", x"4", x"4", x"5", x"F", x"F", x"C", x"9", x"9", x"9", x"9", x"9", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"B", x"3", x"0", x"4", x"E", x"F", x"F", x"F", x"2", x"9", x"F", x"5", x"7", x"F",
        x"F", x"F", x"C", x"F", x"F", x"6", x"7", x"F", x"F", x"F", x"F", x"E", x"3", x"8", x"F", x"F", x"F", x"F", x"F", x"F", x"E", x"6", x"6", x"F",
        x"F", x"C", x"5", x"F", x"F", x"8", x"4", x"F", x"F", x"F", x"4", x"4", x"7", x"0", x"9", x"F", x"F", x"F", x"F", x"9", x"8", x"C", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"A", x"1", x"E", x"F",
        x"F", x"F", x"F", x"D", x"1", x"1", x"E", x"F", x"F", x"F", x"F", x"3", x"A", x"1", x"E", x"F", x"F", x"F", x"6", x"6", x"F", x"1", x"E", x"F",
        x"F", x"A", x"3", x"B", x"B", x"1", x"B", x"F", x"F", x"A", x"6", x"6", x"6", x"0", x"6", x"F", x"F", x"F", x"F", x"F", x"F", x"1", x"E", x"F",
        x"F", x"F", x"F", x"F", x"F", x"9", x"E", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"C", x"1", x"1", x"1", x"9", x"F", x"F", x"F", x"9", x"6", x"D", x"D", x"F", x"F", x"F", x"F", x"7", x"9", x"A", x"E", x"F", x"F",
        x"F", x"F", x"5", x"5", x"5", x"0", x"B", x"F", x"F", x"F", x"F", x"F", x"F", x"6", x"6", x"F", x"F", x"F", x"F", x"F", x"F", x"6", x"6", x"F",
        x"F", x"E", x"0", x"6", x"7", x"0", x"C", x"F", x"F", x"F", x"D", x"9", x"9", x"E", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"4", x"5", x"F", x"F", x"F", x"F", x"F", x"8", x"3", x"F", x"F", x"F",
        x"F", x"F", x"B", x"1", x"D", x"F", x"F", x"F", x"F", x"F", x"2", x"2", x"3", x"1", x"B", x"F", x"F", x"C", x"0", x"F", x"F", x"7", x"4", x"F",
        x"F", x"C", x"2", x"F", x"F", x"8", x"4", x"F", x"F", x"F", x"3", x"6", x"7", x"0", x"A", x"F", x"F", x"F", x"F", x"9", x"8", x"C", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"C", x"1", x"1", x"1", x"1", x"4", x"F",
        x"F", x"F", x"D", x"D", x"D", x"6", x"8", x"F", x"F", x"F", x"F", x"F", x"C", x"0", x"F", x"F", x"F", x"F", x"F", x"F", x"5", x"7", x"F", x"F",
        x"F", x"F", x"F", x"C", x"0", x"E", x"F", x"F", x"F", x"F", x"F", x"5", x"7", x"F", x"F", x"F", x"F", x"F", x"C", x"0", x"E", x"F", x"F", x"F",
        x"F", x"F", x"9", x"A", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"A", x"2", x"1", x"5", x"F", x"F", x"F", x"F", x"1", x"B", x"F", x"4", x"8", x"F", x"F", x"F", x"1", x"C", x"F", x"4", x"8", x"F",
        x"F", x"F", x"B", x"0", x"2", x"7", x"F", x"F", x"F", x"E", x"0", x"B", x"E", x"5", x"6", x"F", x"F", x"B", x"0", x"F", x"F", x"8", x"3", x"F",
        x"F", x"F", x"2", x"6", x"8", x"1", x"8", x"F", x"F", x"F", x"F", x"9", x"8", x"B", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"A", x"2", x"0", x"5", x"F", x"F", x"F", x"F", x"0", x"B", x"F", x"5", x"7", x"F",
        x"F", x"D", x"0", x"F", x"F", x"8", x"5", x"F", x"F", x"F", x"2", x"6", x"8", x"1", x"7", x"F", x"F", x"F", x"E", x"9", x"A", x"1", x"E", x"F",
        x"F", x"F", x"F", x"E", x"1", x"A", x"F", x"F", x"F", x"F", x"F", x"4", x"6", x"F", x"F", x"F", x"F", x"F", x"9", x"9", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"7", x"5", x"F", x"F", x"F", x"F", x"F", x"A", x"0", x"D", x"F", x"F",
        x"F", x"F", x"D", x"0", x"B", x"F", x"F", x"F", x"F", x"F", x"4", x"4", x"1", x"4", x"D", x"F", x"F", x"C", x"0", x"C", x"F", x"6", x"5", x"F",
        x"F", x"B", x"2", x"F", x"F", x"9", x"3", x"F", x"F", x"F", x"1", x"9", x"B", x"2", x"8", x"F", x"F", x"F", x"D", x"6", x"5", x"9", x"F", x"F",
        x"F", x"F", x"5", x"5", x"5", x"5", x"7", x"F", x"F", x"F", x"C", x"C", x"C", x"C", x"C", x"F", x"F", x"F", x"D", x"6", x"4", x"8", x"F", x"F",
        x"F", x"F", x"2", x"8", x"C", x"3", x"8", x"F", x"F", x"D", x"0", x"F", x"F", x"9", x"5", x"F", x"F", x"F", x"0", x"9", x"C", x"4", x"6", x"F",
        x"F", x"F", x"B", x"5", x"6", x"0", x"C", x"F", x"F", x"F", x"F", x"F", x"4", x"8", x"F", x"F", x"F", x"F", x"F", x"6", x"4", x"F", x"F", x"F",
        x"F", x"F", x"9", x"6", x"D", x"F", x"F", x"F", x"F", x"F", x"5", x"5", x"5", x"5", x"7", x"F", x"F", x"F", x"C", x"C", x"C", x"C", x"C", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F" );
    constant C_GLY3 : t_gly := (
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"6", x"0", x"2", x"B", x"F", x"F", x"F", x"8", x"4", x"F", x"A", x"1", x"F", x"F",
        x"F", x"3", x"9", x"F", x"F", x"1", x"B", x"F", x"F", x"1", x"A", x"F", x"F", x"2", x"9", x"F", x"F", x"2", x"A", x"F", x"F", x"2", x"A", x"F",
        x"F", x"5", x"7", x"F", x"E", x"0", x"C", x"F", x"F", x"C", x"0", x"7", x"4", x"5", x"F", x"F", x"F", x"F", x"D", x"8", x"A", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"5", x"5", x"F", x"F", x"F",
        x"F", x"D", x"3", x"5", x"5", x"F", x"F", x"F", x"F", x"C", x"8", x"8", x"5", x"F", x"F", x"F", x"F", x"F", x"F", x"8", x"5", x"F", x"F", x"F",
        x"F", x"F", x"F", x"8", x"5", x"F", x"F", x"F", x"F", x"F", x"F", x"8", x"5", x"F", x"F", x"F", x"F", x"F", x"8", x"8", x"5", x"8", x"F", x"F",
        x"F", x"F", x"9", x"9", x"9", x"9", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"6", x"1", x"2", x"9", x"F", x"F", x"F", x"8", x"4", x"E", x"B", x"0", x"F", x"F", x"F", x"D", x"D", x"F", x"D", x"0", x"E", x"F",
        x"F", x"F", x"F", x"F", x"6", x"5", x"F", x"F", x"F", x"F", x"F", x"7", x"4", x"F", x"F", x"F", x"F", x"F", x"7", x"4", x"F", x"F", x"F", x"F",
        x"F", x"7", x"0", x"5", x"4", x"4", x"C", x"F", x"F", x"9", x"9", x"9", x"9", x"9", x"C", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"6", x"1", x"1", x"8", x"F", x"F", x"F", x"9", x"3", x"E", x"B", x"0", x"E", x"F",
        x"F", x"E", x"D", x"F", x"E", x"0", x"F", x"F", x"F", x"F", x"F", x"6", x"1", x"B", x"F", x"F", x"F", x"F", x"F", x"E", x"B", x"0", x"D", x"F",
        x"F", x"7", x"9", x"F", x"F", x"1", x"C", x"F", x"F", x"A", x"0", x"7", x"5", x"3", x"F", x"F", x"F", x"F", x"C", x"8", x"9", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"3", x"6", x"F", x"F",
        x"F", x"F", x"F", x"6", x"7", x"6", x"F", x"F", x"F", x"F", x"A", x"4", x"9", x"6", x"F", x"F", x"F", x"D", x"0", x"D", x"9", x"6", x"F", x"F",
        x"F", x"3", x"9", x"B", x"9", x"6", x"B", x"F", x"F", x"6", x"6", x"6", x"6", x"5", x"8", x"F", x"F", x"F", x"F", x"F", x"9", x"6", x"F", x"F",
        x"F", x"F", x"F", x"F", x"A", x"9", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"4", x"1", x"1", x"1", x"F", x"F", x"F", x"F", x"1", x"D", x"D", x"D", x"F", x"F", x"F", x"E", x"1", x"B", x"C", x"F", x"F", x"F",
        x"F", x"C", x"3", x"5", x"2", x"5", x"F", x"F", x"F", x"F", x"F", x"F", x"E", x"0", x"D", x"F", x"F", x"F", x"F", x"F", x"D", x"0", x"E", x"F",
        x"F", x"7", x"3", x"7", x"3", x"6", x"F", x"F", x"F", x"F", x"A", x"8", x"B", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"B", x"1", x"B", x"F", x"F", x"F", x"F", x"E", x"1", x"9", x"F", x"F", x"F",
        x"F", x"F", x"5", x"7", x"E", x"F", x"F", x"F", x"F", x"9", x"0", x"3", x"1", x"5", x"F", x"F", x"F", x"4", x"8", x"F", x"E", x"0", x"C", x"F",
        x"F", x"4", x"9", x"F", x"F", x"1", x"C", x"F", x"F", x"A", x"1", x"8", x"5", x"4", x"F", x"F", x"F", x"F", x"C", x"8", x"A", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"4", x"1", x"1", x"1", x"1", x"C", x"F",
        x"F", x"D", x"D", x"D", x"C", x"1", x"F", x"F", x"F", x"F", x"F", x"F", x"5", x"8", x"F", x"F", x"F", x"F", x"F", x"C", x"0", x"E", x"F", x"F",
        x"F", x"F", x"F", x"5", x"7", x"F", x"F", x"F", x"F", x"F", x"C", x"0", x"E", x"F", x"F", x"F", x"F", x"F", x"5", x"7", x"F", x"F", x"F", x"F",
        x"F", x"D", x"9", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"5", x"1", x"2", x"A", x"F", x"F", x"F", x"8", x"4", x"F", x"B", x"1", x"F", x"F", x"F", x"8", x"4", x"F", x"C", x"1", x"F", x"F",
        x"F", x"F", x"7", x"2", x"0", x"B", x"F", x"F", x"F", x"6", x"5", x"E", x"B", x"0", x"E", x"F", x"F", x"4", x"8", x"F", x"F", x"0", x"B", x"F",
        x"F", x"8", x"1", x"8", x"6", x"2", x"F", x"F", x"F", x"F", x"B", x"8", x"9", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"6", x"1", x"2", x"9", x"F", x"F", x"F", x"7", x"5", x"F", x"C", x"0", x"F", x"F",
        x"F", x"5", x"8", x"F", x"F", x"0", x"D", x"F", x"F", x"9", x"1", x"8", x"6", x"0", x"F", x"F", x"F", x"F", x"A", x"8", x"5", x"7", x"F", x"F",
        x"F", x"F", x"F", x"8", x"3", x"F", x"F", x"F", x"F", x"F", x"A", x"0", x"C", x"F", x"F", x"F", x"F", x"F", x"9", x"B", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"D", x"5", x"9", x"F", x"F", x"F", x"F", x"F", x"4", x"7", x"F", x"F", x"F",
        x"F", x"F", x"7", x"4", x"F", x"F", x"F", x"F", x"F", x"B", x"0", x"3", x"2", x"8", x"F", x"F", x"F", x"5", x"6", x"F", x"C", x"0", x"D", x"F",
        x"F", x"3", x"A", x"F", x"F", x"1", x"B", x"F", x"F", x"8", x"4", x"B", x"8", x"2", x"F", x"F", x"F", x"F", x"8", x"5", x"6", x"D", x"F", x"F",
        x"F", x"7", x"5", x"5", x"5", x"5", x"F", x"F", x"F", x"C", x"C", x"C", x"C", x"C", x"F", x"F", x"F", x"F", x"9", x"4", x"5", x"C", x"F", x"F",
        x"F", x"9", x"3", x"B", x"9", x"2", x"F", x"F", x"F", x"5", x"8", x"F", x"F", x"1", x"D", x"F", x"F", x"7", x"3", x"C", x"9", x"0", x"E", x"F",
        x"F", x"F", x"7", x"5", x"7", x"5", x"F", x"F", x"F", x"F", x"F", x"A", x"1", x"E", x"F", x"F", x"F", x"F", x"C", x"0", x"A", x"F", x"F", x"F",
        x"F", x"F", x"6", x"7", x"F", x"F", x"F", x"F", x"F", x"7", x"5", x"5", x"5", x"5", x"F", x"F", x"F", x"C", x"C", x"C", x"C", x"C", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F",
        x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F", x"F" );
    -- @@END GENERATED

    -- d << sh when en, else 0 (16 bits): a shift-add term
    function f_sel(d : signed(10 downto 0); en : std_logic; sh : natural) return signed is
    begin
        if en = '1' then return shift_left(resize(d, 16), sh); end if;
        return to_signed(0, 16);
    end function;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return resize(unsigned(v), 10); end if;
    end function;

    ----------------------------------------------------------------------
    -- raster measurement, generated avid
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_sawact     : std_logic := '0';
    signal s_fstart     : std_logic := '0';
    signal s_avid_q     : std_logic := '0';
    signal s_avid_r     : std_logic := '0';
    signal s_hs_q       : std_logic := '0';
    signal s_hs_r       : std_logic := '0';
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_W          : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_Wl         : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_wlm1       : unsigned(11 downto 0) := to_unsigned(719, 12);   -- s_Wl - 1
    signal s_ycnt       : unsigned(10 downto 0) := (others => '0');
    signal s_H          : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_hf         : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_ilace      : std_logic := '0';
    signal s_field      : std_logic := '0';
    signal s_tid_il     : std_logic := '0';
    signal s_xa         : signed(11 downto 0) := (others => '0');
    signal s_rhf        : unsigned(11 downto 0) := to_unsigned(971, 12);
    signal s_bw256      : std_logic := '1';      -- window 256 px (HD) or 128 (SD)
    signal s_estart, s_earm : std_logic := '0';
    signal s_idle       : unsigned(12 downto 0) := (others => '0');
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avid_q : std_logic := '0';
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc  : unsigned(3 downto 0) := "1000";
    signal g_cnt : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- engine
    ----------------------------------------------------------------------
    signal s_zfirst : std_logic := '1';
    signal s_frans  : std_logic := '0';
    signal s_ustart : std_logic := '0';

    signal mu_pa  : signed(19 downto 0) := (others => '0');
    signal mu_ph  : signed(21 downto 0) := (others => '0');
    signal mu_bp  : std_logic := '0';
    signal mu_pl  : unsigned(15 downto 0) := (others => '0');
    signal mu_cnt : unsigned(3 downto 0) := (others => '0');
    signal mu_bsy : std_logic := '0';

    type t_rf is array (0 to 127) of std_logic_vector(31 downto 0);
    signal rf_a, rf_b : t_rf := (others => (others => '0'));
    signal u_pc  : unsigned(9 downto 0) := (others => '0');
    signal u_st  : unsigned(2 downto 0) := (others => '0');
    signal u_gwt : unsigned(1 downto 0) := (others => '0');
    signal u_ir, u_rom : std_logic_vector(31 downto 0) := (others => '0');
    signal u_fa  : unsigned(9 downto 0) := (others => '0');
    signal u_jr  : unsigned(9 downto 0) := (others => '0');
    signal u_wlast : std_logic := '0';
    signal u_ra, u_wa : unsigned(6 downto 0) := (others => '0');
    signal u_rda, u_rdb, u_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal u_slv, u_ctlv : signed(31 downto 0) := (others => '0');
    signal u_we, u_slw : std_logic := '0';
    signal u_done : std_logic := '1';
    signal u_slp : unsigned(4 downto 0) := (others => '0');
    signal u_sls : unsigned(4 downto 0) := (others => '0');
    signal u_sh  : signed(31 downto 0) := (others => '0');
    signal u_shc : unsigned(3 downto 0) := (others => '0');
    signal u_blt, u_bnz : std_logic := '0';
    signal u_rsel : unsigned(3 downto 0) := (others => '0');
    signal u_rwr, u_cin : std_logic := '0';
    signal u_ctlr : signed(31 downto 0) := (others => '0');
    signal u_romr : signed(16 downto 0) := (others => '0');

    -- engine ROM (quarter sine, face gamma) and the die table RAM
    signal t_sk  : unsigned(7 downto 0) := (others => '0');
    signal t_srd : std_logic_vector(15 downto 0) := (others => '0');
    signal tbl   : t_tbl := C_TABLE;
    signal tb_era, tb_wa : unsigned(8 downto 0) := (others => '0');
    signal tb_wd, tb_rd : std_logic_vector(15 downto 0) := (others => '0');
    signal tb_we : std_logic := '0';

    ----------------------------------------------------------------------
    -- double-buffered engine outputs: the crossing records (4 words an
    -- edge, 32 edges a bank) and the per-field globals
    ----------------------------------------------------------------------
    signal d_bank : std_logic := '0';            -- the bank on screen
    signal s_swap : std_logic := '0';
    type t_wram is array (0 to 511) of std_logic_vector(31 downto 0);
    signal wram : t_wram := (others => (others => '0'));
    signal w_wa, w_ra : unsigned(8 downto 0) := (others => '0');
    signal w_wd, w_rd : std_logic_vector(31 downto 0) := (others => '0');
    signal w_we : std_logic := '0';
    signal e_we, e_pend : std_logic := '0';            -- engine write request
    signal e_wa, e_pa : unsigned(8 downto 0) := (others => '0');
    signal e_wd, e_pd : std_logic_vector(31 downto 0) := (others => '0');
    signal r_wwe : std_logic := '0';                   -- renderer write-back
    signal r_wwa : unsigned(8 downto 0) := (others => '0');
    signal r_wwd : std_logic_vector(31 downto 0) := (others => '0');
    signal n_ne, d_ne : unsigned(5 downto 0) := (others => '0');
    signal n_ns, d_ns : unsigned(3 downto 0) := (others => '0');
    signal n_wx0, d_wx0 : unsigned(11 downto 0) := (others => '0');
    signal n_bgs : signed(19 downto 0) := (others => '0');   -- static per mode

    ----------------------------------------------------------------------
    -- renderer
    ----------------------------------------------------------------------
    signal r_fsr  : std_logic_vector(3 downto 0) := (others => '0');
    signal r_clr  : std_logic := '0';
    signal r_cc   : unsigned(7 downto 0) := (others => '0');
    signal r_run, r_lp, r_wb, r_go : std_logic := '0';
    signal r_e    : unsigned(5 downto 0) := (others => '0');
    signal r_t    : unsigned(3 downto 0) := (others => '0');
    signal r_ph, r_em, s_eemit : std_logic := '0';    -- phase: edges / numerals
    signal r_line : unsigned(10 downto 0) := (others => '0');
    signal r_jb   : signed(15 downto 0) := (others => '0');
    signal r_y0, r_y1 : signed(15 downto 0) := (others => '0');
    signal r_w1, r_w2, r_xl : signed(25 downto 0) := (others => '0');
    signal r_w3   : std_logic_vector(30 downto 0) := (others => '0');
    signal r_d0, r_d1 : signed(15 downto 0) := (others => '0');
    signal r_k, r_khi : unsigned(4 downto 0) := (others => '0');
    signal r_first : std_logic := '0';
    signal r_bg  : unsigned(9 downto 0) := (others => '0');
    signal r_x, r_s : signed(25 downto 0) := (others => '0');
    signal r_dy  : signed(10 downto 0) := (others => '0');
    signal r_du, r_dv : signed(8 downto 0) := (others => '0');
    signal s_rp  : unsigned(7 downto 0);         -- the crossing's pixel ...
    signal s_rfi : unsigned(3 downto 0);         -- ... and 1/16 px fraction

    -- numeral sprites: the record being walked (rs_), the span (r_s*), the
    -- glyph pipe's constants for the span in it (pa_)
    signal r_si, r_s4 : unsigned(3 downto 0) := (others => '0');
    signal rs_ly0, rs_ly1 : unsigned(10 downto 0) := (others => '0');
    signal rs_ga, rs_gb, pa_ga, pa_gb : unsigned(3 downto 0) := (others => '0');
    signal rs_sx0, rs_sw, rs_k, pa_k, pa_sx0, r_n : unsigned(7 downto 0) := (others => '0');
    signal rs_dux, rs_dvx, rs_duy, rs_dvy : signed(15 downto 0) := (others => '0');
    signal pa_ba, pa_bb : unsigned(9 downto 0) := (others => '0');
    signal pa_na, pa_nb, s_semit, g_fresh : std_logic := '0';
    signal r_su, r_sv, r_us, r_vs : signed(19 downto 0) := (others => '0');
    signal rs_w4 : std_logic_vector(28 downto 0) := (others => '0');
    signal pa_dy : signed(10 downto 0) := (others => '0');
    signal pa_du, pa_dv : signed(8 downto 0) := (others => '0');
    signal sp_mk : std_logic := '0';
    -- glyph pipe
    signal g1_vl, g1_mk, g1_last, g1_sx, g1_sy : std_logic := '0';
    signal g1_okx, g1_oky : std_logic_vector(1 downto 0) := (others => '0');
    signal g2_vl, g2_mk, g2_last, g2_sx, g2_sy : std_logic := '0';
    signal g3_vl, g3_mk, g3_last, g3_sx, g3_sy : std_logic := '0';
    signal g4_vl, g4_mk, g4_last, g5_vl, g5_mk, g5_last : std_logic := '0';
    signal g6_vl, g6_mk, g6_last, g7_vl, g7_mk, g7_last : std_logic := '0';
    signal g8_vl, g8_mk, g10_vl, g10_mk, g11_vl, g11_mk : std_logic := '0';
    signal g11_p : unsigned(7 downto 0) := (others => '0');
    signal g1_fx, g1_fy, g2_fx, g2_fy, g3_fx, g3_fy, g4_fx, g4_fy, g5_fy
        : unsigned(3 downto 0) := (others => '0');
    signal g1_x0, g1_x1, g1_y0, g1_y1 : signed(7 downto 0) := (others => '0');
    signal g2_ok, g3_ok : std_logic_vector(3 downto 0) := (others => '0');
    signal g_ra0, g_ra1, g_ra2, g_ra3 : unsigned(9 downto 0) := (others => '0');
    signal g_rd0, g_rd1, g_rd2, g_rd3 : std_logic_vector(3 downto 0) := (others => '1');
    signal g4_t00, g4_t10, g4_t01, g4_t11 : unsigned(3 downto 0) := (others => '0');
    signal g5_top, g5_bot : signed(10 downto 0) := (others => '0');
    signal g6_d  : signed(10 downto 0) := (others => '0');
    signal g6_hi, g6_lo, g7_hi, g7_lo : std_logic := '0';
    signal g7_m  : signed(19 downto 0) := (others => '0');
    signal g8_al : unsigned(4 downto 0) := (others => '0');

    -- deposit pipe: q1 sample, q2 partial products, q3 product, then the
    -- run accumulator (D 16 bits, F 20 bits: 4 bits finer than stored)
    type t_s11x3 is array (0 to 2) of signed(10 downto 0);
    type t_s15x3 is array (0 to 2) of signed(14 downto 0);
    type t_s16x3 is array (0 to 2) of signed(15 downto 0);
    type t_s17x3 is array (0 to 2) of signed(16 downto 0);
    type t_s20x3 is array (0 to 2) of signed(19 downto 0);
    signal q1_v, q1_m, q2_v, q2_m, q3_v, q3_m : std_logic := '0';
    signal q1_p, q2_p, q3_p : unsigned(7 downto 0) := (others => '0');
    signal q1_fi : unsigned(3 downto 0) := (others => '0');
    signal q1_d, q2_d, q3_d : t_s16x3 := (others => (others => '0'));
    signal q2_pa, q2_pb, q3_pr : t_s15x3 := (others => (others => '0'));
    signal acc_v : std_logic := '0';
    signal acc_p : unsigned(7 downto 0) := (others => '0');
    signal acc_d : t_s16x3 := (others => (others => '0'));
    signal acc_f : t_s20x3 := (others => (others => '0'));
    signal fl_v  : std_logic := '0';
    signal fl_p  : unsigned(7 downto 0) := (others => '0');
    signal fl_d, fl_f : t_s16x3 := (others => (others => '0'));
    signal g10_c, g11_e, g_cp : t_s16x3 := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- line buffer: D and F for Y, U8, V8; one shared read and write port
    -- (the renderer works only while the window is not on screen)
    ----------------------------------------------------------------------
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lb_dy, lb_fy, lb_du, lb_fu, lb_dv, lb_fv : t_lb := (others => (others => '0'));
    signal lb_ra, lb_wa : unsigned(7 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_wdy, lb_wfy, lb_wdu, lb_wfu, lb_wdv, lb_wfv : std_logic_vector(15 downto 0) := (others => '0');
    signal lb_rdy, lb_rfy, lb_rdu, lb_rfu, lb_rdv, lb_rfv : std_logic_vector(15 downto 0) := (others => '0');
    signal lq_d, lq_f : t_s16x3 := (others => (others => '0'));   -- read data, fabric

    attribute ram_style : string;
    attribute ram_style of rf_a : signal is "block";
    attribute ram_style of rf_b : signal is "block";
    attribute ram_style of tbl  : signal is "block";
    attribute ram_style of wram : signal is "block";
    attribute ram_style of lb_dy : signal is "block";
    attribute ram_style of lb_fy : signal is "block";
    attribute ram_style of lb_du : signal is "block";
    attribute ram_style of lb_fu : signal is "block";
    attribute ram_style of lb_dv : signal is "block";
    attribute ram_style of lb_fv : signal is "block";
    attribute no_rw_check : boolean;
    attribute no_rw_check of rf_a : signal is true;
    attribute no_rw_check of rf_b : signal is true;
    attribute no_rw_check of tbl  : signal is true;
    attribute no_rw_check of wram : signal is true;
    attribute no_rw_check of lb_dy : signal is true;
    attribute no_rw_check of lb_fy : signal is true;
    attribute no_rw_check of lb_du : signal is true;
    attribute no_rw_check of lb_fu : signal is true;
    attribute no_rw_check of lb_dv : signal is true;
    attribute no_rw_check of lb_fv : signal is true;

    ----------------------------------------------------------------------
    -- display
    ----------------------------------------------------------------------
    signal dp1_w, dp1_f, dp2_w, dp2_f, dp3_w, dp3_f, dp4_w, dp5_w, dp6_w : std_logic := '0';
    signal dc_we : std_logic := '0';
    signal dc_wa : unsigned(7 downto 0) := (others => '0');
    signal sm    : t_s16x3 := (others => (others => '0'));   -- running sums
    signal dp4_f : t_s16x3 := (others => (others => '0'));
    signal dp5_v : t_s17x3 := (others => (others => '0'));
    signal n_y, n_u, n_v : unsigned(9 downto 0) := (others => '0');
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal p9_y, p9_u, p9_v : unsigned(9 downto 0) := (others => '0');
    signal o_y, o_u, o_v : unsigned(9 downto 0) := (others => '0');
    signal bg_acc : signed(27 downto 0) := (others => '0');
    signal bg_y   : unsigned(9 downto 0) := to_unsigned(600, 10);

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement (width, active-line count) and the generated avid
    -- (CUBIST v0.5.6, HW-validated)
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            -- CLEAN AVID.  The encoders get no data-enable -- only hsync and
            -- the 4:2:2 bus -- so they pair Cb/Cr by counting from hsync, while
            -- the core's 4:4:4->4:2:2 packer restarts on Cb at every avid
            -- rise.  An analog source's avid wanders a clock against hsync and
            -- can drop out mid-line, which swaps U/V for that line.  So the
            -- program makes its own avid: it starts 2 or 3 clocks after the
            -- source's first avid rise of the line, whichever keeps the
            -- hsync->start distance at the source's USUAL parity (a saturating
            -- vote), and lasts exactly the measured width.
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
            g_avid_q <= g_avid;
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
            elsif g_avid = '1' and g_cnt = s_wlm1 then   -- a registered W - 1
                g_avid <= '0';
            end if;
            -- pixel index of the generated line
            if g_avid = '1' then g_cnt <= g_cnt + 1;
            else g_cnt <= (others => '0'); end if;

            -- count active lines on the ACTIVE edge (never latch height at
            -- vsync from a serrated counter)
            if s_avid_r = '1' then
                s_ycnt <= s_ycnt + 1;
                s_sawact <= '1';
            end if;

            -- Field work starts once active video has been absent for two
            -- line widths (early in vertical blanking, never off an hsync
            -- edge).  Re-arms only on active video.
            s_estart <= '0';
            if data_in.avid = '1' then
                s_idle <= (others => '0');
                s_earm <= '1';
            else
                if s_idle /= "1111111111111" then s_idle <= s_idle + 1; end if;
                if s_earm = '1' and s_idle = (s_W & '0') then
                    s_estart <= '1';
                    s_earm   <= '0';
                end if;
            end if;
            -- only the FIRST vsync edge after active video is a frame boundary
            s_fstart <= '0';
            if s_vs_pulse = '1' and s_sawact = '1' then
                if s_ycnt > 16 then s_H <= s_ycnt; end if;
                s_ycnt <= (others => '0');
                s_sawact <= '0';
                s_fstart <= '1';
                s_field <= data_in.field_n;
                s_ilace <= s_tid_il;
            end if;
        end if;
    end process p_measure;

    -- timing-ID classes (static while a mode runs): interlace, SD pixel
    -- aspect (Q12 offset on the projected x axis), 2^20 / frame height, and
    -- the window width (128 px at SD, 256 at HD)
    p_tid : process(clk)
    begin
        if rising_edge(clk) then
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8 | 1 | 2 | 10 => s_tid_il <= '1';
                when others             => s_tid_il <= '0';
            end case;
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0      => s_xa <= to_signed(455, 12);  s_rhf <= to_unsigned(2158, 12);
                when 4      => s_xa <= to_signed(512, 12);  s_rhf <= to_unsigned(2185, 12);
                when 8 | 12 => s_xa <= to_signed(-256, 12); s_rhf <= to_unsigned(1820, 12);
                when 5 | 6 | 14 => s_xa <= to_signed(0, 12); s_rhf <= to_unsigned(1456, 12);
                when others => s_xa <= to_signed(0, 12);    s_rhf <= to_unsigned(971, 12);
            end case;
            if registers_in(8)(1 downto 0) = "00" then s_bw256 <= '0';   -- 0 4 8 12: SD
            else s_bw256 <= '1'; end if;
        end if;
    end process p_tid;

    ------------------------------------------------------------------------
    -- Field sequencing.  In vertical blanking, once the engine has finished
    -- the next field's records: swap the banks and start the engine on the
    -- field after.  The renderer then clears the line buffer and draws line
    -- 0 (p_rseq, off s_estart).
    ------------------------------------------------------------------------
    p_fsetup : process(clk)
    begin
        if rising_edge(clk) then
            if s_fstart = '1' then
                if s_ilace = '1' then s_hf <= resize(s_H & '0', 12);
                else                  s_hf <= resize(s_H, 12); end if;
            end if;
            s_swap   <= '0';
            s_ustart <= '0';
            if s_estart = '1' and u_done = '1' then
                s_swap   <= '1';
                s_ustart <= '1';
                s_Wl <= s_W;
                s_wlm1 <= s_W - 1;
                s_frans <= '1';
            end if;
            if s_swap = '1' then
                d_bank <= not d_bank;
                d_ne   <= n_ne;
                d_ns   <= n_ns;
                d_wx0  <= n_wx0 + C_XSHIFT;
            end if;
            -- the first-field snap is consumed once a field has been computed
            if u_done = '1' and s_frans = '1' and s_ustart = '0' then s_zfirst <= '0'; end if;
        end if;
    end process p_fsetup;

    ------------------------------------------------------------------------
    -- SLW decoder: the engine's outputs go to the bank NOT on screen
    ------------------------------------------------------------------------
    p_slotw : process(clk)
    begin
        if rising_edge(clk) then
            e_we <= '0';
            if u_slw = '1' then
                case to_integer(u_slp) is
                    -- crossing records: bank | 0 | edge | word
                    when 0 | 1 | 2 | 3 =>
                        e_wa <= (not d_bank) & '0' & u_sls & u_slp(1 downto 0);
                        e_we <= '1';
                    -- numeral sprites: bank | 1 | sprite + 4 | word
                    when 8 | 9 | 10 | 11 | 12 | 13 =>
                        e_wa <= (not d_bank) & '1' & u_sls(3 downto 0) & u_slp(2 downto 0);
                        e_we <= '1';
                    when 16 => n_ne  <= unsigned(u_slv(5 downto 0));
                    when 17 => n_wx0 <= unsigned(u_slv(11 downto 0));
                    when 18 => n_ns  <= unsigned(u_slv(3 downto 0));
                    when 20 => n_bgs <= u_slv(19 downto 0);
                    when others => null;
                end case;
                e_wd <= std_logic_vector(u_slv);
            end if;
        end if;
    end process p_slotw;

    ------------------------------------------------------------------------
    -- MICROCODED GEOMETRY ENGINE (CUBIST's, without the divider)
    --
    -- One ALU, the register file in two block-RAM copies (dual read), the
    -- program in block RAM.  Three clocks an instruction, four for a taken
    -- jump; shifts one clock a bit; a radix-4 Booth serial multiplier (MULSH
    -- starts it and waits for the product in one instruction); MIN / MAX off
    -- the operand compare; signed (ROMS, SRDS) and unsigned table reads.
    ------------------------------------------------------------------------
    p_ueng : process(clk)
        variable v_a, v_b : signed(31 downto 0);
        variable v_r      : signed(31 downto 0);
        variable v_op     : integer range 0 to 31;
        variable v_op2    : integer range 0 to 31;
        variable v_imm    : signed(13 downto 0);
        variable v_stl    : boolean;
        variable v_jmp    : boolean;
        variable v_ms     : signed(21 downto 0);
        variable v_mx     : signed(21 downto 0);
        variable v_m23    : signed(22 downto 0);
        variable v_s1, v_s2, v_neg : std_logic;
        variable v_pr     : signed(31 downto 0);
        variable v_c33    : signed(32 downto 0);
        variable v_fa, v_fb : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            u_wlast <= u_we;
            -- serial multiply, radix-4 Booth, 8 steps: {mu_ph, mu_pl} =
            -- mu_pa * B; ONE adder: the digit picks 0 / A / 2A, a negative
            -- digit inverts it and carries in the +1
            if mu_bsy = '1' then
                v_s1  := mu_pl(0) xor mu_bp;
                v_s2  := (mu_pl(1) xor mu_pl(0)) and not (mu_pl(0) xor mu_bp);
                v_neg := mu_pl(1) and not (mu_pl(0) and mu_bp);
                if v_s2 = '1' then
                    v_mx := shift_left(resize(mu_pa, 22), 1);
                elsif v_s1 = '1' then
                    v_mx := resize(mu_pa, 22);
                else
                    v_mx := (others => '0');
                end if;
                if v_neg = '1' then v_mx := not v_mx; end if;
                v_m23 := (mu_ph & '1') + (v_mx & v_neg);
                v_ms := v_m23(22 downto 1);
                mu_ph <= shift_right(v_ms, 2);
                mu_pl <= unsigned(v_ms(1 downto 0)) & mu_pl(15 downto 2);
                mu_bp <= mu_pl(1);
                if mu_cnt = 0 then mu_bsy <= '0';
                else mu_cnt <= mu_cnt - 1; end if;
            end if;

            u_we   <= '0';
            u_slw  <= '0';
            tb_we  <= '0';

            if s_ustart = '1' then
                u_pc   <= (others => '0');
                u_st   <= "000";
                u_gwt  <= (others => '0');
                u_done <= '0';
            elsif u_done = '0' then
                case to_integer(u_st) is

                    -- FETCH (after a jump only): the ROM answers next cycle
                    when 0 =>
                        u_st <= "001";

                    -- DECODE: latch the instruction; the register-file reads
                    -- are addressed straight off the ROM output this cycle
                    when 1 =>
                        u_ir <= u_rom;
                        u_st <= "010";

                    -- OPERAND LATCH: block-RAM outputs into fabric before any
                    -- arithmetic; every unit launches from here
                    when 2 =>
                        v_op2 := to_integer(unsigned(u_ir(31 downto 27)));
                        -- forwarding: the previous write lands on this read's
                        -- edge (an EBR read-during-write is undefined)
                        if u_wlast = '1' and unsigned(u_ir(19 downto 13)) = u_wa then
                            v_fa := u_wd;
                        else
                            v_fa := u_rda;
                        end if;
                        if u_wlast = '1' and unsigned(u_ir(12 downto 6)) = u_wa then
                            v_fb := u_wd;
                        else
                            v_fb := u_rdb;
                        end if;
                        u_sh  <= signed(v_fa);
                        if v_op2 = 3 then                  -- SUB = A + not B + 1
                            u_wd <= not v_fb;  u_cin <= '1';
                        else
                            u_wd <= v_fb;      u_cin <= '0';
                        end if;
                        u_ctlr <= u_ctlv;
                        case v_op2 is
                            when 1 | 4 | 5    => u_rsel <= x"0"; u_rwr <= '1';
                            when 2 | 3        => u_rsel <= x"1"; u_rwr <= '1';
                            when 27           => u_rsel <= x"2"; u_rwr <= '1';
                            when 15           => u_rsel <= x"3"; u_rwr <= '1';
                            when 11 | 12      => u_rsel <= x"4"; u_rwr <= '1';
                            when 6 | 7        => u_rsel <= x"5"; u_rwr <= '1';
                            when 14 | 16 | 23 | 28 => u_rsel <= x"6"; u_rwr <= '1';
                            when 25           => u_rsel <= x"8"; u_rwr <= '1';
                            when 31           => u_rsel <= x"9"; u_rwr <= '1';
                            when others       => u_rsel <= x"0"; u_rwr <= '0';
                        end case;
                        u_shc <= unsigned(u_ir(3 downto 0));
                        u_wa  <= unsigned(u_ir(26 downto 20));
                        if signed(v_fa) < signed(v_fb) then u_blt <= '1';
                        else u_blt <= '0'; end if;
                        if v_fa /= x"00000000" then u_bnz <= '1';
                        else u_bnz <= '0'; end if;
                        u_jr <= unsigned(v_fa(9 downto 0));
                        case v_op2 is
                            when 10 | 12 =>                -- MUL / MULSH
                                mu_pa  <= signed(v_fa(19 downto 0));
                                mu_pl  <= unsigned(v_fb(15 downto 0));
                                mu_ph  <= (others => '0');
                                mu_bp  <= '0';
                                mu_cnt <= to_unsigned(7, 4);
                                mu_bsy <= '1';
                            when 14 | 28 => t_sk <= unsigned(v_fa(7 downto 0));  -- ROM
                            when 16 | 23 =>                -- SRD address
                                tb_era <= unsigned(v_fa(8 downto 0))
                                          + resize(unsigned(u_ir(6 downto 0)), 9);
                            when 22 =>                     -- SWR
                                tb_wa <= unsigned(v_fa(8 downto 0));
                                tb_wd <= v_fb(15 downto 0);
                            when 30 =>                     -- LDX address
                                u_ra <= unsigned(v_fa(6 downto 0))
                                        + unsigned(u_ir(6 downto 0));
                            when 31 =>                     -- STX address
                                u_wa <= unsigned(v_fa(6 downto 0))
                                        + resize(unsigned(u_ir(5 downto 0)), 7);
                            when 17 =>                     -- SLW operands
                                u_slp <= unsigned(u_ir(4 downto 0));
                                u_slv <= signed(v_fa);
                                u_sls <= unsigned(v_fb(4 downto 0));
                            when others => null;
                        end case;
                        u_st  <= "011";

                    -- same one-cycle read latency on the indexed address
                    when 4 =>
                        u_st <= "101";

                    -- INDEXED second access (LDX)
                    when 5 =>
                        u_st <= "001";
                        u_pc <= u_pc + 1;
                        u_wa <= unsigned(u_ir(26 downto 20));
                        u_wd <= u_rda;
                        u_we <= '1';

                    -- EXECUTE.  Everything read here is a register.
                    when others =>
                        v_op  := to_integer(unsigned(u_ir(31 downto 27)));
                        v_imm := signed(u_ir(13 downto 0));
                        v_a   := u_sh;
                        v_b   := signed(u_wd);
                        v_stl := false;
                        v_jmp := false;

                        case to_integer(u_rsel) is
                            when 0 => v_r := u_sh;          -- MOV, SHR/SHL
                            when 1 => v_c33 := (v_a & '1') + (v_b & u_cin);
                                      v_r := v_c33(32 downto 1);
                            when 2 => v_r := v_a and signed(resize(unsigned(u_ir(12 downto 0)), 32));
                            when 3 => v_r := resize(v_imm, 32);
                            when 4 =>                        -- MRD: low 32 bits
                                v_pr := signed(std_logic_vector'(
                                            std_logic_vector(mu_ph(15 downto 0))
                                            & std_logic_vector(mu_pl)));
                                if u_ir(3) = '1' then v_r := shift_right(v_pr, 12);
                                else                  v_r := v_pr; end if;
                            when 5 =>                        -- MIN / MAX
                                if (u_blt = '1') = (v_op = 6) then v_r := u_sh;
                                else v_r := v_b; end if;
                            when 6 => v_r := resize(u_romr, 32);  -- ROM/SRD
                            when 9 => v_r := v_b;           -- STX data
                            when others => v_r := u_ctlr;   -- CTL
                        end case;

                        case v_op is
                            when 4 | 5 =>                  -- SHR / SHL
                                if u_shc /= 0 then
                                    if v_op = 4 then u_sh <= shift_right(u_sh, 1);
                                    else             u_sh <= shift_left(u_sh, 1); end if;
                                    u_shc <= u_shc - 1;
                                    v_stl := true;
                                end if;
                            when 11 | 12 => if mu_bsy = '1' then v_stl := true; end if;
                            when 14 | 16 | 23 | 28 =>      -- ROM / SRD (signed: ROMS, SRDS)
                                if u_gwt = 0 then
                                    u_gwt <= "01";
                                    v_stl := true;
                                elsif u_gwt = 1 then
                                    case v_op is
                                        when 14     => u_romr <= signed('0' & t_srd);
                                        when 28     => u_romr <= signed(t_srd(15) & t_srd);
                                        when 23     => u_romr <= signed(tb_rd(15) & tb_rd);
                                        when others => u_romr <= signed('0' & tb_rd);
                                    end case;
                                    u_gwt <= "10";
                                    v_stl := true;
                                else
                                    u_gwt <= (others => '0');
                                end if;
                            when 17 => u_slw <= '1';       -- SLW (operands latched)
                            when 22 => tb_we <= '1';       -- SWR (operands latched)
                            when 18 => v_jmp := true;
                                       u_pc <= unsigned(u_ir(9 downto 0));
                            when 19 => if u_bnz = '1' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(9 downto 0));
                                       end if;
                            when 20 => if u_blt = '1' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(23 downto 20))
                                                   & unsigned(u_ir(5 downto 0));
                                       end if;
                            when 21 => if u_blt = '0' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(23 downto 20))
                                                   & unsigned(u_ir(5 downto 0));
                                       end if;
                            when 26 => v_jmp := true;      -- JR
                                       u_pc <= u_jr;
                            when 24 => u_done <= '1';
                            when others => null;
                        end case;

                        if v_stl then
                            u_st <= "011";
                        elsif v_op = 30 then
                            u_st <= "100";
                        elsif v_jmp then
                            u_st <= "000";
                        else
                            u_st <= "001";
                            u_pc <= u_pc + 1;
                        end if;

                        u_wd <= std_logic_vector(resize(v_r, 32));
                        if u_rwr = '1' and not v_stl then
                            u_we <= '1';
                        end if;
                end case;
            end if;
        end if;
    end process p_ueng;

    -- CTL: the outside world.  The microcode reads each control once a
    -- field, so the knobs are read live (K6 and the switches are unused in
    -- this phase and read 0).
    with to_integer(unsigned(u_ir(4 downto 0))) select u_ctlv <=
        signed(resize(unsigned(registers_in(0)), 32)) when 0,
        signed(resize(unsigned(registers_in(1)), 32)) when 1,
        signed(resize(unsigned(registers_in(2)), 32)) when 2,
        signed(resize(unsigned(registers_in(3)), 32)) when 3,
        signed(resize(unsigned(registers_in(4)), 32)) when 4,
        signed(resize(s_Wl, 32)) when 6,
        signed(resize(s_hf, 32)) when 7,
        (0 => s_ilace, others => '0')  when 8,
        -- the engine computes the field AFTER next, whose parity is the
        -- one just shown
        (0 => s_field, others => '0')  when 9,
        (0 => s_zfirst, others => '0') when 10,
        signed(resize(unsigned(registers_in(7)), 32)) when 12,
        resize(s_xa, 32) when 13,
        signed(resize(s_rhf, 32)) when 14,
        (7 => not s_bw256, 8 => s_bw256, others => '0') when 15,
        (others => '0') when others;

    -- the ROM reads pc+1 during EXEC and LDX's second access, so a
    -- straight-line instruction decodes on the very next clock
    u_fa <= u_pc + 1 when (u_st = "011" or u_st = "101") else u_pc;

    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            u_rom <= C_UCODE(to_integer(u_fa));
        end if;
    end process p_urom;

    p_rfa : process(clk)
        variable v_ra : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_a(to_integer(u_wa)) <= u_wd; end if;
            if u_st = "001" then v_ra := unsigned(u_rom(19 downto 13));
            else                 v_ra := u_ra; end if;
            u_rda <= rf_a(to_integer(v_ra));
        end if;
    end process p_rfa;

    p_rfb : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_b(to_integer(u_wa)) <= u_wd; end if;
            u_rdb <= rf_b(to_integer(unsigned(u_rom(12 downto 6))));
        end if;
    end process p_rfb;

    p_erom : process(clk)
    begin
        if rising_edge(clk) then
            t_srd <= C_EROM(to_integer(t_sk));
        end if;
    end process p_erom;

    p_tbl : process(clk)
    begin
        if rising_edge(clk) then
            if tb_we = '1' then tbl(to_integer(tb_wa)) <= tb_wd; end if;
            tb_rd <= tbl(to_integer(tb_era));
        end if;
    end process p_tbl;

    ------------------------------------------------------------------------
    -- RECORD TABLE (bank | 8 bits): crossing records at 0..127 (edge | word),
    -- each record's x at the line start at 128..159, numeral sprites at
    -- 160..255 (sprite + 4 | word; words 6 and 7 the texel coordinates at
    -- the line start).  The engine writes the bank NOT on screen, the
    -- renderer reads the one on screen and writes back its running words
    -- there; they share one write port, the renderer first (an engine write
    -- that meets it waits a clock or two: SLWs are 3+ clocks apart).
    ------------------------------------------------------------------------
    p_wport : process(clk)
    begin
        if rising_edge(clk) then
            w_we <= '0';
            if r_wwe = '1' then
                w_we <= '1';
                w_wa <= r_wwa;
                w_wd <= r_wwd;
                if e_we = '1' then
                    e_pend <= '1';
                    e_pa   <= e_wa;
                    e_pd   <= e_wd;
                end if;
            elsif e_pend = '1' then
                w_we   <= '1';
                w_wa   <= e_pa;
                w_wd   <= e_pd;
                e_pend <= e_we;
                e_pa   <= e_wa;
                e_pd   <= e_wd;
            elsif e_we = '1' then
                w_we <= '1';
                w_wa <= e_wa;
                w_wd <= e_wd;
            end if;
        end if;
    end process p_wport;

    p_wram_w : process(clk)
    begin
        if rising_edge(clk) then
            if w_we = '1' then wram(to_integer(w_wa)) <= w_wd; end if;
        end if;
    end process p_wram_w;

    p_wram_r : process(clk)
    begin
        if rising_edge(clk) then
            w_rd <= wram(to_integer(w_ra));
        end if;
    end process p_wram_r;

    -- the crossing's pixel in the window (clamped) and its 1/16 px fraction
    s_rp  <= (others => '0') when r_x(25) = '1' else
             (others => '1') when r_x(24) = '1' else
             unsigned(r_x(23 downto 16));
    s_rfi <= (others => '0') when r_x(25) = '1' else
             (others => '1') when r_x(24) = '1' else
             unsigned(r_x(15 downto 12));
    -- this clock steps an edge sub-line / a numeral pixel (p_rseq's LOOP
    -- branches)
    s_eemit <= '1' when r_fsr(3) = '0' and r_clr = '0' and r_go = '0' and dp1_w = '0'
                        and r_run = '1' and r_ph = '0' and r_lp = '1' else '0';
    s_semit <= '1' when r_fsr(3) = '0' and r_clr = '0' and r_go = '0' and dp1_w = '0'
                        and r_run = '1' and r_ph = '1' and r_lp = '1' else '0';

    ------------------------------------------------------------------------
    -- RENDER SEQUENCER.  Starts once a line's window has gone by (r_go) and
    -- draws the NEXT line.
    --   EDGES: for each crossing record, fetch its 4 words and its x at the
    --   line start; skip it if it crosses none of the line's 16 sub-lines,
    --   else step through the sub-lines it crosses, one a clock, sending
    --   (x, colour step) down the deposit pipe; then a run marker and the x
    --   write-back.  ~5 clocks a record off the line, 9 + its sub-lines one
    --   on it.
    --   NUMERALS: for each sprite, fetch its 8 words; skip it off its lines,
    --   else walk its span one pixel a clock (texel coordinates + their
    --   per-pixel steps) into the glyph pipe, the last pixel blank; then a
    --   run marker and the next line's texel coordinates written back.
    -- At the field start (vertical blanking) it first clears the line
    -- buffer, then draws line 0.
    ------------------------------------------------------------------------
    p_rseq : process(clk)
        variable v_y, v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            r_wwe <= '0';
            r_em  <= '0';
            sp_mk <= '0';
            r_fsr <= r_fsr(2 downto 0) & s_estart;

            if r_fsr(3) = '1' then
                r_clr  <= '1';
                r_cc   <= (others => '0');
                r_run  <= '0';
                r_lp   <= '0';
                r_wb   <= '0';
                r_line <= (others => '0');
            elsif r_clr = '1' then
                r_cc <= r_cc + 1;
                if r_cc = 255 then
                    r_clr <= '0';
                    r_run <= '1';
                    r_bg  <= bg_y;
                    r_ph  <= '0';
                    r_e   <= (others => '0');
                    r_t   <= (others => '0');
                    r_jb  <= (others => '0');
                end if;
            elsif r_go = '1' then
                r_line <= r_line + 1;
                r_jb   <= signed(resize(r_line + 1, 12) & "0000");
                r_run  <= '1';
                r_bg   <= bg_y;                 -- this line's backdrop
                r_ph   <= '0';
                r_lp   <= '0';
                r_wb   <= '0';
                r_e    <= (others => '0');
                r_t    <= (others => '0');
            elsif dp1_w = '1' then
                -- the window is on screen: the line buffer is the display's
                -- (never happens with the render times measured; a guard)
                r_run <= '0';
                r_lp  <= '0';
                r_wb  <= '0';
            elsif r_run = '1' and r_ph = '0' then
                ------------------------------------------------ edges
                if r_lp = '1' then
                    -- one sub-line: a crossing at x (p_dep takes it)
                    r_x <= r_x + r_s;
                    r_k <= r_k + 1;
                    if r_k + 1 = r_khi then
                        r_lp <= '0';
                        r_wb <= '1';
                    end if;
                elsif r_wb = '1' then
                    -- end of the record: close its run, keep its x
                    r_em  <= '1';
                    r_wwe <= '1';
                    r_wwa <= d_bank & "100" & r_e(4 downto 0);
                    r_wwd <= std_logic_vector(resize(r_x, 32));
                    r_wb  <= '0';
                    r_e   <= r_e + 1;
                    r_t   <= (others => '0');
                else
                    case to_integer(r_t) is
                        when 0 =>
                            if r_e >= d_ne then
                                r_ph <= '1';        -- on to the numerals
                                r_si <= (others => '0');
                            else
                                w_ra <= d_bank & '0' & r_e(4 downto 0) & "00";
                                r_t  <= to_unsigned(1, 4);
                            end if;
                        when 1 =>
                            w_ra <= d_bank & '0' & r_e(4 downto 0) & "01";
                            r_t  <= to_unsigned(2, 4);
                        when 2 =>                   -- w0 arrives
                            w_ra <= d_bank & '0' & r_e(4 downto 0) & "10";
                            r_y0 <= signed(resize(unsigned(w_rd(14 downto 0)), 16));
                            r_y1 <= signed(resize(unsigned(w_rd(29 downto 15)), 16));
                            r_t  <= to_unsigned(3, 4);
                        when 3 =>                   -- w1 arrives
                            w_ra <= d_bank & '0' & r_e(4 downto 0) & "11";
                            r_w1 <= signed(w_rd(25 downto 0));
                            r_d0 <= r_y0 - r_jb;
                            r_d1 <= r_y1 - r_jb;
                            r_t  <= to_unsigned(4, 4);
                        when 4 =>                   -- w2 arrives
                            w_ra <= d_bank & "100" & r_e(4 downto 0);
                            r_w2 <= signed(w_rd(25 downto 0));
                            if r_d0 < 16 and r_d1 > 0 then
                                if r_d0 < 0 then
                                    r_k     <= (others => '0');
                                    r_first <= '0';
                                else
                                    r_k     <= unsigned(r_d0(4 downto 0));
                                    r_first <= '1';
                                end if;
                                if r_d1 > 16 then r_khi <= to_unsigned(16, 5);
                                else r_khi <= unsigned(r_d1(4 downto 0)); end if;
                                r_t <= to_unsigned(5, 4);
                            else
                                r_e <= r_e + 1;     -- misses the line
                                r_t <= (others => '0');
                            end if;
                        when 5 =>                   -- w3 arrives
                            r_w3 <= w_rd(30 downto 0);
                            r_t  <= to_unsigned(6, 4);
                        when 6 =>                   -- x at the line start arrives
                            r_xl <= signed(w_rd(25 downto 0));
                            r_t  <= to_unsigned(7, 4);
                        when others =>
                            -- start x, slope, and the colour step: a
                            -- silhouette's stored face colour less (left
                            -- side) or plus (right side) this line's backdrop
                            if r_first = '1' then r_x <= r_w1;
                            else                  r_x <= r_xl; end if;
                            r_s <= r_w2;
                            v_y := resize(signed(r_w3(10 downto 0)), 12);
                            v_u := resize(signed(r_w3(19 downto 11)), 12);
                            v_v := resize(signed(r_w3(28 downto 20)), 12);
                            if r_w3(29) = '1' then
                                if r_w3(30) = '1' then
                                    v_y := v_y + signed(resize(r_bg, 12));
                                    v_u := v_u + C_BGU8;
                                    v_v := v_v + C_BGV8;
                                else
                                    v_y := v_y - signed(resize(r_bg, 12));
                                    v_u := v_u - C_BGU8;
                                    v_v := v_v - C_BGV8;
                                end if;
                            end if;
                            r_dy <= v_y(10 downto 0);
                            r_du <= v_u(8 downto 0);
                            r_dv <= v_v(8 downto 0);
                            r_lp <= '1';
                    end case;
                end if;
            elsif r_run = '1' then
                ------------------------------------------------ numerals
                if r_lp = '1' then
                    -- one pixel of the span into the glyph pipe (p_glyph
                    -- takes r_su, r_sv this clock)
                    r_su <= r_su + resize(rs_dux, 20);
                    r_sv <= r_sv + resize(rs_dvx, 20);
                    r_n  <= r_n - 1;
                    if r_n = 0 then
                        r_lp <= '0';
                        r_wb <= '1';
                    end if;
                elsif r_wb = '1' then
                    -- end of the span: close its run, keep the next line's
                    -- texel coordinates (two writes)
                    if r_t = 0 then
                        sp_mk <= '1';
                        r_wwe <= '1';
                        r_wwa <= d_bank & '1' & r_s4 & "110";
                        r_wwd <= std_logic_vector(resize(r_us + resize(rs_duy, 20), 32));
                        r_t   <= to_unsigned(1, 4);
                    else
                        r_wwe <= '1';
                        r_wwa <= d_bank & '1' & r_s4 & "111";
                        r_wwd <= std_logic_vector(resize(r_vs + resize(rs_dvy, 20), 32));
                        r_wb  <= '0';
                        r_si  <= r_si + 1;
                        r_t   <= (others => '0');
                    end if;
                else
                    case to_integer(r_t) is
                        when 0 =>
                            if r_si >= d_ns then
                                r_run <= '0';       -- the line is done
                            else
                                w_ra <= d_bank & '1' & r_s4 & "000";
                                r_t  <= to_unsigned(1, 4);
                            end if;
                        when 1 =>
                            w_ra <= d_bank & '1' & r_s4 & "001";
                            r_t  <= to_unsigned(2, 4);
                        when 2 =>                   -- w0: lines, glyphs
                            w_ra <= d_bank & '1' & r_s4 & "010";
                            rs_ly0 <= unsigned(w_rd(10 downto 0));
                            rs_ly1 <= unsigned(w_rd(21 downto 11));
                            rs_ga  <= unsigned(w_rd(25 downto 22));
                            rs_gb  <= unsigned(w_rd(29 downto 26));
                            r_t    <= to_unsigned(3, 4);
                        when 3 =>                   -- w1: span, AA scale
                            w_ra <= d_bank & '1' & r_s4 & "011";
                            rs_sx0 <= unsigned(w_rd(7 downto 0));
                            rs_sw  <= unsigned(w_rd(16 downto 9));
                            rs_k   <= unsigned(w_rd(24 downto 17));
                            if r_line >= rs_ly0 and r_line < rs_ly1 then
                                r_t <= to_unsigned(4, 4);
                            else
                                r_si <= r_si + 1;   -- off its lines
                                r_t <= (others => '0');
                            end if;
                        when 4 =>                   -- w2: steps across
                            w_ra <= d_bank & '1' & r_s4 & "100";
                            rs_dux <= signed(w_rd(15 downto 0));
                            rs_dvx <= signed(w_rd(31 downto 16));
                            r_first <= '0';
                            if r_line = rs_ly0 then r_first <= '1'; end if;
                            r_t  <= to_unsigned(5, 4);
                        when 5 =>                   -- w3: steps down
                            w_ra <= d_bank & '1' & r_s4 & "101";
                            rs_duy <= signed(w_rd(15 downto 0));
                            rs_dvy <= signed(w_rd(31 downto 16));
                            r_t  <= to_unsigned(6, 4);
                        when 6 =>                   -- w4: ink step
                            w_ra <= d_bank & '1' & r_s4 & "110";
                            rs_w4 <= w_rd(28 downto 0);
                            r_t   <= to_unsigned(7, 4);
                        when 7 =>                   -- w5: texel coords at ly0
                            w_ra <= d_bank & '1' & r_s4 & "111";
                            if r_first = '1' then
                                r_su <= shift_left(resize(signed(w_rd(15 downto 0)), 20), 4);
                                r_sv <= shift_left(resize(signed(w_rd(31 downto 16)), 20), 4);
                            end if;
                            r_t   <= to_unsigned(8, 4);
                        when 8 =>                   -- w6: u at this line
                            if r_first = '0' then r_su <= signed(w_rd(19 downto 0)); end if;
                            r_t   <= to_unsigned(9, 4);
                        when 9 =>                   -- w7: v at this line
                            if r_first = '0' then r_sv <= signed(w_rd(19 downto 0)); end if;
                            r_t   <= to_unsigned(10, 4);
                        when others =>
                            -- the span starts, with the glyph pipe's
                            -- constants (safe to switch: the last span is
                            -- 12+ clocks out of the pipe)
                            r_us  <= r_su;
                            r_vs  <= r_sv;
                            r_n   <= rs_sw;
                            pa_sx0 <= rs_sx0;
                            pa_ba <= resize(shift_left(resize(rs_ga, 10), 6) + shift_left(resize(rs_ga, 10), 4), 10);
                            pa_bb <= resize(shift_left(resize(rs_gb, 10), 6) + shift_left(resize(rs_gb, 10), 4), 10);
                            pa_na <= '0';
                            pa_nb <= '0';
                            if rs_ga > 11 then pa_na <= '1'; end if;
                            if rs_gb > 11 then pa_nb <= '1'; end if;
                            pa_k  <= rs_k;
                            pa_dy <= signed(rs_w4(10 downto 0));
                            pa_du <= signed(rs_w4(19 downto 11));
                            pa_dv <= signed(rs_w4(28 downto 20));
                            r_lp  <= '1';
                            r_t   <= (others => '0');
                    end case;
                end if;
            end if;
        end if;
    end process p_rseq;

    r_s4 <= r_si + 4;

    ------------------------------------------------------------------------
    -- GLYPH PIPE: one numeral pixel a clock.  g1 texel cell and fractions;
    -- g2 the four taps' bank addresses (2 x 2 interleave: the taps of a
    -- bilinear sample sit in four different banks); g3 ROM; g4 taps in
    -- fabric, outside the glyph = far; g5/g6 bilinear; g7/g8 coverage over a
    -- 1-px ramp at k px a texel (x16; the span's last pixel blank); g9/g10
    -- the ink's colour step times the coverage; g11 its change from the
    -- last pixel, into the deposit pipe.
    ------------------------------------------------------------------------
    p_glyph : process(clk)
        variable v_ix, v_iy : signed(7 downto 0);
        variable v_x, v_y : signed(7 downto 0);
        variable v_g : unsigned(3 downto 0);
        variable v_ok : std_logic;
        variable v_a : unsigned(9 downto 0);
        variable v_t0, v_t1, v_t2, v_t3 : unsigned(3 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_al : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            ------------------------------------------------------ g1
            v_ix := r_su(19 downto 12);
            v_iy := r_sv(19 downto 12);
            g1_vl <= s_semit;  g1_mk <= sp_mk;
            if r_n = 0 then g1_last <= '1'; else g1_last <= '0'; end if;
            g1_fx <= unsigned(r_su(11 downto 8));
            g1_fy <= unsigned(r_sv(11 downto 8));
            g1_sx <= v_ix(0);
            g1_sy <= v_iy(0);
            -- each bank's tap: the one of ix, ix + 1 with its parity, and
            -- whether it is inside the glyph box (32 x 20 texels)
            if v_ix(0) = '0' then v_x := v_ix; v_y := v_ix + 1;
            else                  v_x := v_ix + 1; v_y := v_ix; end if;
            g1_x0 <= v_x;  g1_x1 <= v_y;
            g1_okx(0) <= '0';  g1_okx(1) <= '0';
            if v_x >= 0 and v_x <= 31 then g1_okx(0) <= '1'; end if;
            if v_y >= 0 and v_y <= 31 then g1_okx(1) <= '1'; end if;
            if v_iy(0) = '0' then v_x := v_iy; v_y := v_iy + 1;
            else                  v_x := v_iy + 1; v_y := v_iy; end if;
            g1_y0 <= v_x;  g1_y1 <= v_y;
            g1_oky(0) <= '0';  g1_oky(1) <= '0';
            if v_x >= 0 and v_x <= 19 then g1_oky(0) <= '1'; end if;
            if v_y >= 0 and v_y <= 19 then g1_oky(1) <= '1'; end if;

            ------------------------------------------------------ g2
            g2_vl <= g1_vl;  g2_mk <= g1_mk;  g2_last <= g1_last;
            g2_fx <= g1_fx;  g2_fy <= g1_fy;  g2_sx <= g1_sx;  g2_sy <= g1_sy;
            for b in 0 to 3 loop
                if b mod 2 = 0 then v_x := g1_x0; else v_x := g1_x1; end if;
                if b / 2 = 0 then v_y := g1_y0; else v_y := g1_y1; end if;
                v_ok := '1';
                if v_x(4) = '0' then
                    v_a := pa_ba;
                    if pa_na = '1' then v_ok := '0'; end if;
                else
                    v_a := pa_bb;
                    if pa_nb = '1' then v_ok := '0'; end if;
                end if;
                if g1_okx(b mod 2) = '0' or g1_oky(b / 2) = '0' then
                    v_ok := '0';
                end if;
                v_a := v_a + resize(unsigned(v_y(4 downto 1)) & unsigned(v_x(3 downto 1)), 10);
                case b is
                    when 0 => g_ra0 <= v_a;
                    when 1 => g_ra1 <= v_a;
                    when 2 => g_ra2 <= v_a;
                    when others => g_ra3 <= v_a;
                end case;
                g2_ok(b) <= v_ok;
            end loop;

            ------------------------------------------------------ g3 (ROM)
            g3_vl <= g2_vl;  g3_mk <= g2_mk;  g3_last <= g2_last;
            g3_fx <= g2_fx;  g3_fy <= g2_fy;  g3_sx <= g2_sx;  g3_sy <= g2_sy;
            g3_ok <= g2_ok;

            ------------------------------------------------------ g4
            -- taps into fabric (outside the glyph: far), back in order
            v_t0 := unsigned(g_rd0);  v_t1 := unsigned(g_rd1);
            v_t2 := unsigned(g_rd2);  v_t3 := unsigned(g_rd3);
            if g3_ok(0) = '0' then v_t0 := "1111"; end if;
            if g3_ok(1) = '0' then v_t1 := "1111"; end if;
            if g3_ok(2) = '0' then v_t2 := "1111"; end if;
            if g3_ok(3) = '0' then v_t3 := "1111"; end if;
            -- bank (bx, by) holds the tap whose x/y parity is bx/by; the
            -- left/top tap has the cell's own parity (sx, sy)
            if g3_sx = '0' and g3_sy = '0' then
                g4_t00 <= v_t0;  g4_t10 <= v_t1;  g4_t01 <= v_t2;  g4_t11 <= v_t3;
            elsif g3_sx = '0' then
                g4_t00 <= v_t2;  g4_t10 <= v_t3;  g4_t01 <= v_t0;  g4_t11 <= v_t1;
            elsif g3_sy = '0' then
                g4_t00 <= v_t1;  g4_t10 <= v_t0;  g4_t01 <= v_t3;  g4_t11 <= v_t2;
            else
                g4_t00 <= v_t3;  g4_t10 <= v_t2;  g4_t01 <= v_t1;  g4_t11 <= v_t0;
            end if;
            g4_vl <= g3_vl;  g4_mk <= g3_mk;  g4_last <= g3_last;
            g4_fx <= g3_fx;  g4_fy <= g3_fy;

            ------------------------------------------------------ g5
            -- across: top / bottom rows, Q4 code
            g5_top <= shift_left(signed(resize(g4_t00, 11)), 4)
                      + (signed(resize(g4_t10, 6)) - signed(resize(g4_t00, 6)))
                        * signed('0' & g4_fx);
            g5_bot <= shift_left(signed(resize(g4_t01, 11)), 4)
                      + (signed(resize(g4_t11, 6)) - signed(resize(g4_t01, 6)))
                        * signed('0' & g4_fx);
            g5_vl <= g4_vl;  g5_mk <= g4_mk;  g5_last <= g4_last;
            g5_fy <= g4_fy;

            ------------------------------------------------------ g6
            -- down: the distance code, Q8, less the zero level (8 x 256)
            v_d  := resize((g5_bot - g5_top) * signed('0' & g5_fy), 14);
            v_d  := shift_left(resize(g5_top, 14), 4) + v_d - 2048;
            -- past +-1024 the ramp is over either way (k >= 1 px a texel):
            -- saturate, and multiply only the 11 bits between
            g6_hi <= '0';  g6_lo <= '0';
            if v_d > 1023 then g6_hi <= '1'; end if;
            if v_d < -1024 then g6_lo <= '1'; end if;
            g6_d <= v_d(10 downto 0);
            g6_vl <= g5_vl;  g6_mk <= g5_mk;  g6_last <= g5_last;

            ------------------------------------------------------ g7
            g7_m <= g6_d * signed('0' & pa_k);
            g7_vl <= g6_vl;  g7_mk <= g6_mk;  g7_last <= g6_last;
            g7_hi <= g6_hi;  g7_lo <= g6_lo;

            ------------------------------------------------------ g8
            -- coverage x16 = 8 - k d / 2048, clamped; the span's last pixel 0
            v_al := to_signed(8, 14) - resize(shift_right(g7_m, 11), 14);
            if g7_last = '1' or g7_hi = '1' or v_al < 0 then g8_al <= (others => '0');
            elsif g7_lo = '1' or v_al > 16 then g8_al <= to_unsigned(16, 5);
            else g8_al <= unsigned(v_al(4 downto 0)); end if;
            g8_vl <= g7_vl;  g8_mk <= g7_mk;

            ------------------------------------------------------ g9
            -- luma: ink step x coverage (coverage <= 16); chroma switches
            -- whole at half cover (4:2:2 and the output smoothing hide it)
            if g8_al(4) = '1' then
                g10_c(0) <= shift_left(resize(pa_dy, 16), 4);
            else
                g10_c(0) <= f_sel(pa_dy, g8_al(3), 3) + f_sel(pa_dy, g8_al(2), 2)
                            + f_sel(pa_dy, g8_al(1), 1) + f_sel(pa_dy, g8_al(0), 0);
            end if;
            if g8_al >= 8 then
                g10_c(1) <= shift_left(resize(pa_du, 16), 4);
                g10_c(2) <= shift_left(resize(pa_dv, 16), 4);
            else
                g10_c(1) <= (others => '0');
                g10_c(2) <= (others => '0');
            end if;
            g10_vl <= g8_vl;  g10_mk <= g8_mk;

            ------------------------------------------------------ g11
            -- the change from the last pixel (a span ends blank, so the next
            -- starts from 0)
            g11_vl <= g10_vl and not dp1_w;
            g11_mk <= g10_mk and not dp1_w;
            if g10_vl = '1' then
                for c in 0 to 2 loop
                    g11_e(c) <= g10_c(c) - g_cp(c);
                    g_cp(c)  <= g10_c(c);
                end loop;
                -- the pixel: the span's first, then one on (a span's run
                -- marker re-arms the first)
                if g_fresh = '1' then g11_p <= pa_sx0;
                else                  g11_p <= g11_p + 1; end if;
                g_fresh <= '0';
            end if;
            if g10_mk = '1' then g_fresh <= '1'; end if;
            if r_fsr(3) = '1' or r_go = '1' then
                g_cp <= (others => (others => '0'));
                g_fresh <= '1';
            end if;
        end if;
    end process p_glyph;

    p_g0 : process(clk)
    begin
        if rising_edge(clk) then g_rd0 <= C_GLY0(to_integer(g_ra0)); end if;
    end process p_g0;
    p_g1 : process(clk)
    begin
        if rising_edge(clk) then g_rd1 <= C_GLY1(to_integer(g_ra1)); end if;
    end process p_g1;
    p_g2 : process(clk)
    begin
        if rising_edge(clk) then g_rd2 <= C_GLY2(to_integer(g_ra2)); end if;
    end process p_g2;
    p_g3 : process(clk)
    begin
        if rising_edge(clk) then g_rd3 <= C_GLY3(to_integer(g_ra3)); end if;
    end process p_g3;

    ------------------------------------------------------------------------
    -- DEPOSIT PIPE.  q1: a crossing (pixel, fraction, colour step) or a
    -- numeral pixel (pixel, change of its ink step; no fraction), whose
    -- line-buffer read was issued with it; q2: step * fraction in two
    -- halves; q3: the product, read data in fabric; then the run
    -- accumulator: a sample in a new pixel flushes the last run (one
    -- write) and starts from the buffer's word, a sample in the same pixel
    -- adds on.  Within a record or a span the pixels only move one way, so a
    -- run's write never meets a read of the same word; records and spans
    -- are 8+ clocks apart.
    ------------------------------------------------------------------------
    p_dep : process(clk)
        variable v_new : std_logic;
    begin
        if rising_edge(clk) then
            -- q1
            q1_v <= '0';
            q1_m <= r_em or g11_mk;
            if s_eemit = '1' then
                q1_v  <= '1';
                q1_p  <= s_rp;
                q1_fi <= s_rfi;
                q1_d(0) <= resize(r_dy, 16);
                q1_d(1) <= resize(r_du, 16);
                q1_d(2) <= resize(r_dv, 16);
            elsif g11_vl = '1' then
                q1_v  <= '1';
                q1_p  <= g11_p;
                q1_fi <= (others => '0');
                q1_d  <= g11_e;
            end if;
            -- q2
            q2_v <= q1_v;  q2_m <= q1_m;  q2_p <= q1_p;  q2_d <= q1_d;
            for c in 0 to 2 loop
                q2_pa(c) <= (others => '0');
                q2_pb(c) <= (others => '0');
                if q1_fi(3) = '1' and q1_fi(2) = '1' then
                    q2_pa(c) <= shift_left(resize(q1_d(c), 15), 3) + shift_left(resize(q1_d(c), 15), 2);
                elsif q1_fi(3) = '1' then
                    q2_pa(c) <= shift_left(resize(q1_d(c), 15), 3);
                elsif q1_fi(2) = '1' then
                    q2_pa(c) <= shift_left(resize(q1_d(c), 15), 2);
                end if;
                if q1_fi(1) = '1' and q1_fi(0) = '1' then
                    q2_pb(c) <= shift_left(resize(q1_d(c), 15), 1) + resize(q1_d(c), 15);
                elsif q1_fi(1) = '1' then
                    q2_pb(c) <= shift_left(resize(q1_d(c), 15), 1);
                elsif q1_fi(0) = '1' then
                    q2_pb(c) <= resize(q1_d(c), 15);
                end if;
            end loop;
            -- q3
            q3_v <= q2_v;  q3_m <= q2_m;  q3_p <= q2_p;  q3_d <= q2_d;
            for c in 0 to 2 loop
                q3_pr(c) <= q2_pa(c) + q2_pb(c);
            end loop;
            -- run accumulator
            if q3_v = '1' and (acc_v = '0' or q3_p /= acc_p) then v_new := '1';
            else v_new := '0'; end if;
            fl_v <= '0';
            if (v_new = '1' or q3_m = '1') and acc_v = '1' then
                fl_v <= '1';
                fl_p <= acc_p;
                for c in 0 to 2 loop
                    fl_d(c) <= acc_d(c);
                    fl_f(c) <= acc_f(c)(19 downto 4);
                end loop;
            end if;
            if q3_v = '1' then
                for c in 0 to 2 loop
                    if v_new = '1' then
                        acc_d(c) <= lq_d(c) + q3_d(c);
                        acc_f(c) <= shift_left(resize(lq_f(c), 20), 4) + resize(q3_pr(c), 20);
                    else
                        acc_d(c) <= acc_d(c) + q3_d(c);
                        acc_f(c) <= acc_f(c) + resize(q3_pr(c), 20);
                    end if;
                end loop;
                acc_p <= q3_p;
                acc_v <= '1';
            elsif q3_m = '1' then
                acc_v <= '0';
            end if;
        end if;
    end process p_dep;

    ------------------------------------------------------------------------
    -- LINE BUFFER.  One read address: the display's while the window is on
    -- screen, else the renderer's sample pixel.  One write: the display
    -- clearing behind itself, the field-start clear, or a run flush.
    ------------------------------------------------------------------------
    p_lbport : process(clk)
        variable v_rel : unsigned(11 downto 0);
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            v_rel := g_cnt - d_wx0;
            v_win := '0';
            if g_avid = '1' and g_cnt >= d_wx0 and v_rel(11 downto 8) = "0000"
               and (s_bw256 = '1' or v_rel(7) = '0') then
                v_win := '1';
            end if;
            dp1_w <= v_win;
            if v_rel = 0 then dp1_f <= '1'; else dp1_f <= '0'; end if;
            if v_win = '1' then
                lb_ra <= v_rel(7 downto 0);
            elsif s_eemit = '1' then
                lb_ra <= s_rp;
            else
                lb_ra <= g11_p;
            end if;
            -- clear behind the display: the word read last clock
            dc_we <= dp1_w;
            dc_wa <= lb_ra;

            lb_we <= '0';
            if dc_we = '1' then
                lb_we <= '1';
                lb_wa <= dc_wa;
                lb_wdy <= (others => '0'); lb_wfy <= (others => '0');
                lb_wdu <= (others => '0'); lb_wfu <= (others => '0');
                lb_wdv <= (others => '0'); lb_wfv <= (others => '0');
            elsif r_clr = '1' then
                lb_we <= '1';
                lb_wa <= r_cc;
                lb_wdy <= (others => '0'); lb_wfy <= (others => '0');
                lb_wdu <= (others => '0'); lb_wfu <= (others => '0');
                lb_wdv <= (others => '0'); lb_wfv <= (others => '0');
            elsif fl_v = '1' and dp1_w = '0' then
                lb_we <= '1';
                lb_wa <= fl_p;
                lb_wdy <= std_logic_vector(fl_d(0)); lb_wfy <= std_logic_vector(fl_f(0));
                lb_wdu <= std_logic_vector(fl_d(1)); lb_wfu <= std_logic_vector(fl_f(1));
                lb_wdv <= std_logic_vector(fl_d(2)); lb_wfv <= std_logic_vector(fl_f(2));
            end if;

            -- read data into fabric before any arithmetic
            lq_d(0) <= signed(lb_rdy);  lq_f(0) <= signed(lb_rfy);
            lq_d(1) <= signed(lb_rdu);  lq_f(1) <= signed(lb_rfu);
            lq_d(2) <= signed(lb_rdv);  lq_f(2) <= signed(lb_rfv);
        end if;
    end process p_lbport;

    p_lb_dy : process(clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then lb_dy(to_integer(lb_wa)) <= lb_wdy; end if;
            lb_rdy <= lb_dy(to_integer(lb_ra));
        end if;
    end process p_lb_dy;

    p_lb_fy : process(clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then lb_fy(to_integer(lb_wa)) <= lb_wfy; end if;
            lb_rfy <= lb_fy(to_integer(lb_ra));
        end if;
    end process p_lb_fy;

    p_lb_du : process(clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then lb_du(to_integer(lb_wa)) <= lb_wdu; end if;
            lb_rdu <= lb_du(to_integer(lb_ra));
        end if;
    end process p_lb_du;

    p_lb_fu : process(clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then lb_fu(to_integer(lb_wa)) <= lb_wfu; end if;
            lb_rfu <= lb_fu(to_integer(lb_ra));
        end if;
    end process p_lb_fu;

    p_lb_dv : process(clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then lb_dv(to_integer(lb_wa)) <= lb_wdv; end if;
            lb_rdv <= lb_dv(to_integer(lb_ra));
        end if;
    end process p_lb_dv;

    p_lb_fv : process(clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then lb_fv(to_integer(lb_wa)) <= lb_wfv; end if;
            lb_rfv <= lb_fv(to_integer(lb_ra));
        end if;
    end process p_lb_fv;

    ------------------------------------------------------------------------
    -- backdrop: a vertical luma gradient, one accumulator stepped per line
    ------------------------------------------------------------------------
    p_bg : process(clk)
    begin
        if rising_edge(clk) then
            if r_fsr(0) = '1' then
                bg_acc <= to_signed(C_BGT, 12) & to_signed(0, 16);
            elsif g_avid = '0' and g_avid_q = '1' then
                bg_acc <= bg_acc + resize(n_bgs, 28);
            end if;
            if bg_acc(27) = '1' then bg_y <= (others => '0');
            else bg_y <= unsigned(bg_acc(25 downto 16)); end if;
        end if;
    end process p_bg;

    ------------------------------------------------------------------------
    -- DISPLAY.  Stage 1 is the read address (p_lbport); the words arrive in
    -- fabric at stage 3.  Running sums of D across the window (restarting
    -- at its first pixel), less F, over 16 sub-lines, over the backdrop.
    -- The renderer starts on the next line once the window has gone by.
    ------------------------------------------------------------------------
    p_disp : process(clk)
        variable v_y, v_u, v_v : signed(13 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d, v_yo : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            dp2_w <= dp1_w;  dp2_f <= dp1_f;
            dp3_w <= dp2_w;  dp3_f <= dp2_f;
            dp4_w <= dp3_w;
            dp5_w <= dp4_w;
            dp6_w <= dp5_w;
            r_go  <= dp6_w and not dp5_w;

            ------------------------------------------------------ stage 3
            for c in 0 to 2 loop
                if dp3_f = '1' then sm(c) <= lq_d(c);
                else                 sm(c) <= sm(c) + lq_d(c); end if;
                dp4_f(c) <= lq_f(c);
            end loop;

            ------------------------------------------------------ stage 4
            for c in 0 to 2 loop
                dp5_v(c) <= resize(sm(c), 17) - resize(dp4_f(c), 17);
            end loop;

            ------------------------------------------------------ stage 5
            -- Y: backdrop + sum / 16; U/V (8-bit deposits): 4 x backdrop +
            -- sum / 4, both rounded
            v_y := resize(shift_right(dp5_v(0) + 8, 4), 14) + signed(resize(bg_y, 14));
            v_u := resize(shift_right(dp5_v(1) + 2, 2), 14) + to_signed(4 * C_BGU8, 14);
            v_v := resize(shift_right(dp5_v(2) + 2, 2), 14) + to_signed(4 * C_BGV8, 14);
            if dp5_w = '1' then
                n_y <= f_clamp10(v_y);
                n_u <= f_clamp10(v_u);
                n_v <= f_clamp10(v_v);
            else
                n_y <= bg_y;
                n_u <= to_unsigned(4 * C_BGU8, 10);
                n_v <= to_unsigned(4 * C_BGV8, 10);
            end if;

            ------------------------------------------------------ stage 6
            -- output conditioning (HD HDMI thick-bar fix, damier v0.4.4):
            -- horizontal [1 2 1]/4 on Y, U and V, centred one pixel back
            g1_y <= n_y;  g2_y <= g1_y;
            g1_u <= n_u;  g2_u <= g1_u;
            g1_v <= n_v;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(n_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(n_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(n_v, 12);
            p9_y <= v_sy(11 downto 2);
            p9_u <= v_su(11 downto 2);
            p9_v <= v_sv(11 downto 2);

            ------------------------------------------------------ stage 7
            -- luma scaled toward black, 64 + 13/16 (y - 64); neutral outside
            -- the (latency-aligned) generated avid; U/V swapped for hardware
            v_d  := p9_y - 64;
            v_yo := p9_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            if s_avid_sr(C_LATENCY - 2) = '1' then
                o_y <= v_yo;
                o_u <= p9_v;
                o_v <= p9_u;
            else
                o_y <= to_unsigned(64, 10);
                o_u <= to_unsigned(512, 10);
                o_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_disp;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= g_avid          & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_u);
    data_out.v       <= std_logic_vector(o_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture nat20;
