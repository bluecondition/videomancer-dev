-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- dicefall.vhd
--
-- DICEFALL -- many small dice tumbling down the screen, each a random
-- colour, numbered, with every outline and crease exactly anti-aliased.
--
-- v0.1.0.  K1 count, K2 tumble rate, K3 light, K4 size, P12 fall speed,
-- S7 a random mix of d4..d20 (else all d20s).
--
-- ARCHITECTURE (NAT20's scanline vector renderer, many dice):
--   GEOMETRY ENGINE (dicefall_ucode / df_ucode.py): NAT20's microcoded
--     engine, running continuously.  Field after field it computes every
--     die in screen order, just in time: its crossing records and numeral
--     sprites go into the next free entry of a 4-entry RING, which the
--     renderer takes as the beam reaches the die and frees once the beam
--     has passed it.  So only the dice near the beam are stored.  A die's
--     colour, kind, x and spin come from a hash of its place in the fall
--     and its generation: no per-die state.  All fall at one speed with
--     staggered phases, so never more than two share a scanline.
--   RENDERER (p_rseq): at each line start it draws the NEXT line: retires
--     the dice the beam has passed, takes the next ring entries whose top
--     has come into one of two SLOTS (128 px each), then draws each slot's
--     die exactly as NAT20 draws its one: crossing records at 16 sub-lines
--     a line into a D/F line buffer, numerals through the glyph pipe.  At
--     the field start it clears the buffers and draws lines -128..-1 dry
--     (nothing deposited), so dice straddling the top arrive in step.
--   LINE BUFFERS: two physical banks (the line drawn, the line shown) of
--     four channels (D and F of Y, D of U8 and V8; colour steps a whole
--     pixel at a time, which 4:2:2 hides) x two slots.
--   DISPLAY (p_disp): each slot's running sum, placed at its die's x, over
--     the backdrop; cleared behind the beam; HD HDMI output conditioning
--     and the blanking gate.
--
-- Formats: see df_ucode.py.  Video internals are BT.601; U/V are swapped
-- at the output for the hardware.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture dicefall of program_top is

    constant C_LATENCY : integer := 12;
    constant C_XSHIFT  : integer := 3;           -- display path 9 clocks, sync tap 12
    constant C_BGT     : integer := 660;         -- backdrop luma at the top
    constant C_BGU8    : integer := 130;         -- backdrop chroma, 8-bit (BT.601)
    constant C_BGV8    : integer := 127;
    constant C_PRE     : integer := 128;         -- dry lines before line 0

    -- @@BEGIN GENERATED (df_regen.py)
    -- 1023 microcode words; table 506/512 words
    type t_ucode is array (0 to 1023) of std_logic_vector(31 downto 0);
    constant C_UCODE : t_ucode := (
        x"9000004E", x"20708008", x"D870E03F", x"2080800E", x"D8810001", x"9801000A",
        x"7090E000", x"1070E040", x"7070E000", x"9000000F", x"78800040", x"187101C0",
        x"7090E000", x"1870E040", x"7070E000", x"1870E240", x"D86080FF", x"6070E180",
        x"2070E008", x"105121C0", x"2080800F", x"D8810001", x"98010018", x"D0016000",
        x"18500140", x"D0016000", x"78601000", x"2860C002", x"10408180", x"90000001",
        x"A800A023", x"78600001", x"2860C00F", x"2860C001", x"1050A180", x"2860800F",
        x"2860C001", x"1060C140", x"D0016000", x"78600000", x"78700001", x"2870E00F",
        x"2870E00D", x"1080C1C0", x"A0008231", x"18408200", x"2060C001", x"1060C1C0",
        x"90000032", x"2060C001", x"2070E002", x"9800E02B", x"D0016000", x"7840147D",
        x"655AA100", x"78400AAB", x"155AA100", x"DD5ABFFF", x"604AB540", x"20408006",
        x"D8409FFF", x"D0016000", x"08A16000", x"78B00041", x"90000035", x"28508003",
        x"78B00044", x"90000035", x"2040800A", x"78600004", x"18408180", x"98008049",
        x"78400004", x"28408001", x"6049A100", x"20408004", x"1050A100", x"D0014000",
        x"78000000", x"78100001", x"7CE00000", x"C8400010", x"1859C100", x"D850A0FF",
        x"9800A056", x"9000005C", x"1860A040", x"9800C059", x"9000005C", x"78600080",
        x"A010A191", x"0CE08000", x"CD700007", x"CD800006", x"C8400008", x"C8500009",
        x"14102100", x"18602140", x"34208180", x"2C284004", x"C8400003", x"785008F5",
        x"60408140", x"2040800A", x"7850051F", x"10408140", x"64AAE108", x"2D294001",
        x"78400020", x"152A4100", x"2D1AE004", x"151A3480", x"284A2001", x"40609480",
        x"CD900011", x"3060D640", x"1060C040", x"C8400000", x"60408180", x"2040800A",
        x"20408001", x"2CF08001", x"78400002", x"3CF9E100", x"450A33C0", x"C840000C",
        x"20408003", x"78500004", x"10408140", x"60409040", x"14B96100", x"A0297447",
        x"1CB97440", x"14C98040", x"90000083", x"C8400001", x"14D9A100", x"78500080",
        x"2850A00C", x"A029A14D", x"1CD9A140", x"C840000E", x"78503F1A", x"60408140",
        x"20408004", x"60409040", x"88008014", x"184A32C0", x"10409400", x"18408040",
        x"40509400", x"A020B3D9", x"78500000", x"0D30A000", x"7D600000", x"C840000F",
        x"9800809E", x"9000009B", x"604A7400", x"104092C0", x"0D898000", x"A0209464",
        x"18409440", x"158B0040", x"1D409480", x"7840001D", x"655B0100", x"7840018D",
        x"604A6100", x"155AA100", x"784003F5", x"155AA100", x"DD5ABFFF", x"78B000AF",
        x"90000035", x"78B000B1", x"90000035", x"7850000C", x"60408140", x"2030800D",
        x"78B000B6", x"90000035", x"78500005", x"60408140", x"2020800D", x"C840000B",
        x"D8408001", x"980080BD", x"78200004", x"78B000BF", x"90000035", x"C8500006",
        x"2590A001", x"78600080", x"186B2180", x"60408180", x"25A0800D", x"D84A6001",
        x"980080C8", x"900000C9", x"15AB5640", x"880B4005", x"78B000CC", x"9000003E",
        x"0D90A000", x"78B000CF", x"9000003E", x"0DA0A000", x"084B2000", x"78B000D3",
        x"90000001", x"0DB0A000", x"084B2000", x"78B000D7", x"9000001A", x"0DC0A000",
        x"084B4000", x"78B000DB", x"90000001", x"0DD0A000", x"084B4000", x"78B000DF",
        x"9000001A", x"0DE0A000", x"0B8B8000", x"7B900000", x"0BAB6000", x"63BBB6C8",
        x"0BCBC000", x"604BB708", x"1BD00100", x"604BD6C8", x"1BE00100", x"0BFBA000",
        x"640BD708", x"C850000D", x"60594148", x"15994140", x"7D700400", x"258A4001",
        x"158B1500", x"28404003", x"85A08000", x"85B08001", x"84408002", x"84508003",
        x"84608004", x"84708005", x"84808006", x"84908007", x"65CB2E08", x"60494EC8",
        x"1DF00100", x"65DB2E48", x"60494F08", x"1E000100", x"65EB2E88", x"60494F48",
        x"1E100100", x"78900000", x"088B4000", x"BE210000", x"BE310001", x"BE410002",
        x"604B9888", x"605BB8C8", x"10408140", x"605BD908", x"10408140", x"20408002",
        x"104095C0", x"F801210C", x"604BF888", x"605C18C8", x"10408140", x"605C3908",
        x"10408140", x"20408002", x"10409600", x"F8012118", x"78400003", x"10810100",
        x"10912040", x"A04136C5", x"C8400002", x"28408005", x"0E208000", x"78B00121",
        x"90000001", x"0E00A000", x"084C4000", x"78B00125", x"9000001A", x"0E10A000",
        x"78400137", x"60409808", x"7E200FA0", x"1E2C4100", x"604C3888", x"1E300100",
        x"78400CCD", x"60409808", x"66409888", x"784008CD", x"60409808", x"78500385",
        x"10408140", x"66509888", x"7E600A5C", x"7E700264", x"7E800BF4", x"657718C8",
        x"60477908", x"157AE100", x"6047D948", x"157AE100", x"658738C8", x"60479908",
        x"158B0100", x"6047F948", x"158B0100", x"659758C8", x"6047B908", x"159B2100",
        x"60481948", x"159B2100", x"65A71988", x"604779C8", x"15AB4100", x"6047DA08",
        x"15AB4100", x"65B73988", x"604799C8", x"15BB6100", x"6047FA08", x"15BB6100",
        x"65C75988", x"6047B9C8", x"15CB8100", x"60481A08", x"15CB8100", x"28406001",
        x"104080C0", x"85D08028", x"85E08029", x"85F0802A", x"2E994002", x"1E901A40",
        x"7EA00000", x"10489A80", x"80808000", x"DE01000F", x"20810004", x"DE11000F",
        x"20810004", x"DE21000F", x"F63C000C", x"F66C0018", x"F64C200C", x"F67C2018",
        x"F65C400C", x"F68C4018", x"184C98C0", x"185D1980", x"60608140", x"184CF980",
        x"185CB8C0", x"60708140", x"1860C1C0", x"F80D4024", x"A870DA46", x"088D4000",
        x"98092176", x"90000177", x"208D4001", x"28410001", x"10408200", x"10809200",
        x"BEB10000", x"BEC10001", x"BED10002", x"66ED75C8", x"604D9608", x"16EDC100",
        x"604DB648", x"16EDC100", x"26EDC002", x"66FD7688", x"604D96C8", x"16FDE100",
        x"604DB708", x"16FDE100", x"26FDE002", x"670D6EC8", x"604D8F08", x"170E0100",
        x"604DAF48", x"170E0100", x"270E0002", x"3EEDC000", x"3EFDE000", x"78400EB8",
        x"671DC108", x"784004CD", x"605DE108", x"171E2140", x"78400148", x"605E0108",
        x"171E2140", x"78400266", x"171E2100", x"78400060", x"605E2108", x"7840007F",
        x"3850A000", x"3050A100", x"78400080", x"1050A100", x"7090A000", x"78400040",
        x"185BA100", x"6050A240", x"2050A008", x"1050A100", x"78400300", x"30A0A100",
        x"78400200", x"185BC100", x"6050A240", x"2050A008", x"1050A100", x"784003FF",
        x"3850A000", x"3050A100", x"2050A002", x"2850A00A", x"10A14140", x"78400200",
        x"185BE100", x"6050A240", x"2050A008", x"1050A100", x"784003FF", x"3850A000",
        x"3050A100", x"2050A002", x"2850A00F", x"2850A003", x"10A14140", x"20512003",
        x"2850A00F", x"2850A00B", x"10A14140", x"F80D42A4", x"16AD4040", x"A05D515D",
        x"7C300000", x"7EA00000", x"1048DA80", x"80808000", x"DD71001F", x"20810005",
        x"DD810003", x"20810002", x"DD91001F", x"25A10007", x"F5DAE024", x"F5EB2024",
        x"980BA1D7", x"980BC1DA", x"90000256", x"980BC1D9", x"900001DA", x"980B4256",
        x"104895C0", x"80808000", x"D851000F", x"20810004", x"D861000F", x"20810004",
        x"D871000F", x"980B01E5", x"0DB0A000", x"0DC0C000", x"900001EC", x"184B0040",
        x"980081EA", x"0DB0C000", x"0DC0E000", x"900001EC", x"0DB0E000", x"0DC0A000",
        x"F5FB600C", x"F60B6018", x"F61B800C", x"F62B8018", x"1E4C5800", x"980C81F3",
        x"90000256", x"A070193E", x"084BA000", x"0DDBC000", x"0DE08000", x"084BE000",
        x"0DFC2000", x"0E108000", x"084C0000", x"0E0C4000", x"0E208000", x"1E401900",
        x"1E3C37C0", x"1E5C1080", x"18482040", x"98008203", x"90000205", x"165CA040",
        x"265CA001", x"1E6C5080", x"18482040", x"98008209", x"9000020B", x"166CC040",
        x"266CC001", x"A89CB996", x"A08C600F", x"084C6000", x"90000210", x"184018C0",
        x"2840800C", x"18582040", x"9800A214", x"90000215", x"28408001", x"085C8000",
        x"206C8001", x"10408180", x"46708140", x"A88C601B", x"1E7019C0", x"2E8BE00C",
        x"78400800", x"168D0100", x"084CA000", x"18582040", x"9800A222", x"90000223",
        x"10408100", x"10409080", x"18409800", x"98008227", x"90000229", x"204CE001",
        x"168D0100", x"D84BA3FF", x"D85BC3FF", x"1E908140", x"DE9D27FF", x"204BA00A",
        x"D84080FF", x"205BC00A", x"D850A0FF", x"18408140", x"D84081FF", x"2840800B",
        x"169D2100", x"204BA00F", x"20408003", x"D84080FF", x"205BC00F", x"2050A003",
        x"D850A0FF", x"18408140", x"D84081FF", x"2840800F", x"28408005", x"169D2100",
        x"980BA246", x"78400003", x"2840800F", x"2840800E", x"169D2100", x"9000024B",
        x"980BC24B", x"78400001", x"2840800F", x"2840800E", x"169D2100", x"084CC000",
        x"085CA000", x"78B0024F", x"9000001E", x"8800D0C0", x"880D10C1", x"880CF0C2",
        x"880D30C3", x"14386040", x"78400014", x"A8986118", x"16AD4040", x"A07D51CA",
        x"7DB00000", x"28404001", x"10408080", x"78500065", x"10408140", x"E5708000",
        x"10408040", x"75808000", x"10408040", x"75908000", x"7840007A", x"70508000",
        x"08606000", x"9800C267", x"9000026A", x"2050A001", x"1860C040", x"90000265",
        x"D850A001", x"2840A001", x"10408140", x"78500074", x"15A08140", x"0DD8A000",
        x"98092272", x"90000273", x"25D8A001", x"7DC00000", x"089B8000", x"98092277",
        x"90000278", x"109B9700", x"F5E12024", x"980BC27B", x"900003D9", x"10488240",
        x"85F08000", x"86008001", x"7E200001", x"98004281", x"7E200003", x"7E100000",
        x"78A00077", x"204BE000", x"D840800F", x"F8014100", x"F8014104", x"204BE004",
        x"D840800F", x"F8014101", x"F8014105", x"204BE008", x"D840800F", x"F8014102",
        x"F8014106", x"D84BE00F", x"78600000", x"98092296", x"205BE004", x"D850A00F",
        x"F8014144", x"9000029D", x"204C0004", x"78600018", x"78500003", x"A8A0415C",
        x"204C0008", x"7860000C", x"D840800F", x"F8014103", x"78500041", x"10A0C140",
        x"207BE00C", x"D870E003", x"980042A4", x"087C2000", x"F040E077", x"16308040",
        x"980042A8", x"900002AD", x"163B8040", x"78400003", x"18404100", x"980082AD",
        x"0E3B8000", x"7E700000", x"7E800000", x"7E900000", x"7EA00000", x"7EB00000",
        x"7EC00000", x"7E000000", x"1040F800", x"F0808077", x"F091000C", x"F0810018",
        x"10415800", x"E0508000", x"60612148", x"167CE180", x"60610148", x"168D0180",
        x"10415800", x"78500004", x"10408140", x"E0508000", x"60612148", x"169D2180",
        x"60610148", x"16AD4180", x"10415800", x"78500008", x"10408140", x"E0508000",
        x"60612148", x"16BD6180", x"60610148", x"16CD8180", x"160C0040", x"78400004",
        x"A0AC0134", x"7E50000F", x"7E600010", x"0E4C6000", x"7840000A", x"A0BC611D",
        x"7E600020", x"7E400001", x"1E5C6100", x"A0BCA127", x"7E400002", x"1E5CA100",
        x"900002E7", x"78400003", x"A0B04127", x"78400006", x"184C6100", x"980082E3",
        x"7E40000A", x"78400009", x"184C6100", x"980082E7", x"7E40000B", x"604D35C8",
        x"104099C0", x"2E708007", x"604D3648", x"1E900100", x"66BD7608", x"604D7980",
        x"20408001", x"1E7CE100", x"284D2003", x"285D2001", x"10408140", x"1E7CE100",
        x"604D55C8", x"10409A00", x"2E808007", x"604D5648", x"1EA00100", x"66CD9608",
        x"604D9980", x"20408001", x"1E8D0100", x"284D4003", x"285D4001", x"10408140",
        x"1E8D0100", x"66DD7A80", x"604D3B00", x"1EDDA100", x"205DA007", x"78400410",
        x"A0F0A117", x"78400001", x"2840800F", x"2840800A", x"46E08140", x"66FD5B80",
        x"26FDE009", x"204DE00E", x"10408040", x"20408001", x"980083D7", x"670D9B80",
        x"270E0009", x"1F001C00", x"204E000E", x"10408040", x"20408001", x"980083D7",
        x"671D3B80", x"271E2009", x"1F101C40", x"204E200E", x"10408040", x"20408001",
        x"980083D7", x"672D7B80", x"272E4009", x"204E400E", x"10408040", x"20408001",
        x"980083D7", x"604D7980", x"285D2002", x"1060BA40", x"2850C002", x"0F3CE000",
        x"0F4CE000", x"A0C0802E", x"174E8100", x"9000032F", x"173E6100", x"A0C0A032",
        x"174E8140", x"90000333", x"173E6140", x"604D9980", x"285D4002", x"1060BA80",
        x"2850C002", x"0F5D0000", x"0F6D0000", x"A0C0803C", x"176EC100", x"9000033D",
        x"175EA100", x"A0D0A000", x"176EC140", x"90000341", x"175EA140", x"273E600B",
        x"1F3E6040", x"3F3E6000", x"274E800B", x"78400002", x"174E8100", x"7840007F",
        x"374E8100", x"1F4E9CC0", x"275EA007", x"1F5EB080", x"275EA004", x"18482040",
        x"98008350", x"90000351", x"275EA001", x"276EC007", x"1F6ED080", x"276EC004",
        x"18482040", x"98008357", x"90000358", x"276EC001", x"78400002", x"176EC100",
        x"284E600B", x"78500400", x"10408140", x"189099C0", x"20912004", x"284EA004",
        x"78500008", x"10408140", x"60409040", x"10409080", x"28408007", x"18A09A00",
        x"20A14004", x"660DE240", x"604E2280", x"160C0100", x"260C0007", x"78400800",
        x"1E0C0100", x"66EE0240", x"604E4280", x"16EDC100", x"26EDC007", x"78400800",
        x"1EEDC100", x"18482040", x"98008376", x"90000378", x"171E3C40", x"172E5C80",
        x"084DA000", x"78B0037B", x"90000027", x"2060C007", x"78400010", x"3860C100",
        x"784000FF", x"36D0C100", x"7840000B", x"109B6100", x"D84ECFFF", x"2840800C",
        x"D85EAFFF", x"10408140", x"285C800F", x"2850A009", x"10408140", x"285CA00F",
        x"2850A00D", x"10408140", x"88008248", x"284E8009", x"10409CC0", x"285DA00F",
        x"2850A002", x"10408140", x"88008249", x"084E0000", x"085DE000", x"78B00397",
        x"9000001E", x"8800C24A", x"084E4000", x"085E2000", x"78B0039C", x"9000001E",
        x"8800C24B", x"880C024D", x"880DC24E", x"20ABC00F", x"20A1400B", x"D8A1403F",
        x"28A14003", x"705B4000", x"78400040", x"1850A100", x"6050A280", x"2050A008",
        x"1050A100", x"78400300", x"3050A100", x"D84BC3FF", x"1E40A100", x"DE4C87FF",
        x"105B4040", x"7050A000", x"78400200", x"1850A100", x"6050A280", x"2050A008",
        x"1050A100", x"784003FF", x"3850A000", x"3050A100", x"2050A002", x"204BC00A",
        x"D84080FF", x"1850A100", x"D850A1FF", x"2850A00B", x"164C8140", x"105B4040",
        x"1050A040", x"7050A000", x"78400200", x"1850A100", x"6050A280", x"2050A008",
        x"1050A100", x"784003FF", x"3850A000", x"3050A100", x"2050A002", x"204BC00F",
        x"20408003", x"D84080FF", x"1850A100", x"D850A1FF", x"2850A00F", x"2850A005",
        x"164C8140", x"880C824C", x"15BB6040", x"78400005", x"A8FB611B", x"161C2040",
        x"A0AC3882", x"15CB8040", x"A09B9774", x"089B6000", x"1FEA9080", x"27EFC004",
        x"18482040", x"980083E1", x"900003E2", x"27EFC001", x"1FFA9080", x"17FFF480",
        x"27FFE004", x"18482040", x"980083E8", x"900003E9", x"27FFE001", x"17FFE040",
        x"D84FCFFF", x"D85FEFFF", x"2850A00C", x"10408140", x"D859C0FF", x"2850A00F",
        x"2850A009", x"10408140", x"88008004", x"28412005", x"104090C0", x"88008006",
        x"D849C0FF", x"88008011", x"153A6040", x"A0FA73FB", x"7D300000", x"156AC040",
        x"A02AD3DB", x"14E9C040", x"90000051", x"00000000" );
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
    signal s_sd         : std_logic := '0';
    signal s_ncap       : unsigned(4 downto 0) := to_unsigned(20, 5);
    signal s_estart, s_earm : std_logic := '0';
    signal s_idle       : unsigned(12 downto 0) := (others => '0');
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avid_q : std_logic := '0';
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc  : unsigned(3 downto 0) := "1000";
    signal g_cnt : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- engine
    ----------------------------------------------------------------------

    signal mu_pa  : signed(19 downto 0) := (others => '0');
    signal mu_ph  : signed(21 downto 0) := (others => '0');
    signal mu_bp  : std_logic := '0';
    signal mu_pl  : unsigned(15 downto 0) := (others => '0');
    signal mu_cnt : unsigned(3 downto 0) := (others => '0');
    signal mu_bsy : std_logic := '0';
    signal dv_n, dv_q : unsigned(25 downto 0) := (others => '0');
    signal dv_d, dv_r : unsigned(21 downto 0) := (others => '0');
    signal dv_cnt : unsigned(4 downto 0) := (others => '0');
    signal dv_bsy : std_logic := '0';

    type t_rf is array (0 to 127) of std_logic_vector(31 downto 0);
    signal rf : t_rf := (others => (others => '0'));
    signal u_pc  : unsigned(9 downto 0) := (others => '0');
    signal u_st  : unsigned(2 downto 0) := (others => '0');
    signal u_gwt : unsigned(1 downto 0) := (others => '0');
    signal u_ir, u_rom : std_logic_vector(31 downto 0) := (others => '0');
    signal u_fa  : unsigned(9 downto 0) := (others => '0');
    signal u_jr  : unsigned(9 downto 0) := (others => '0');
    signal u_wlast : std_logic := '0';
    signal u_ra, u_wa : unsigned(6 downto 0) := (others => '0');
    signal u_rd, u_rda, u_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal u_rbop : std_logic := '0';
    signal u_slv, u_ctlv : signed(31 downto 0) := (others => '0');
    signal u_we, u_slw : std_logic := '0';
    signal u_done : std_logic := '0';            -- runs from power-up, never ends
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
    -- the ring of die records (4 entries x 128 words: edges at 4 e,
    -- header at 80, numeral sprites at 8 (s + 11)) and its state
    ----------------------------------------------------------------------
    type t_wram is array (0 to 511) of std_logic_vector(31 downto 0);
    signal wram : t_wram := (others => (others => '0'));
    signal w_wa, w_ra : unsigned(8 downto 0) := (others => '0');
    signal w_wd, w_rd : std_logic_vector(31 downto 0) := (others => '0');
    signal w_we : std_logic := '0';
    signal e_we, e_pend : std_logic := '0';            -- engine write request
    signal e_wa, e_pa : unsigned(8 downto 0) := (others => '0');
    signal e_wd, e_pd : std_logic_vector(31 downto 0) := (others => '0');
    signal e_done : std_logic := '0';                  -- engine: entry complete
    signal e_tag  : unsigned(7 downto 0) := (others => '0');
    signal r_wwe : std_logic := '0';                   -- renderer write-back
    signal r_wwa : unsigned(8 downto 0) := (others => '0');
    signal r_wwd : std_logic_vector(31 downto 0) := (others => '0');
    type t_st4 is array (0 to 3) of unsigned(1 downto 0);   -- 0 free 1 ready 2 active
    type t_tag4 is array (0 to 3) of unsigned(7 downto 0);
    signal rg_st  : t_st4 := (others => (others => '0'));
    signal rg_tag : t_tag4 := (others => (others => '0'));
    signal rg_wp, rg_ap : unsigned(1 downto 0) := (others => '0');
    -- per entry, registered a clock behind (ACT reads one bit, not a tag
    -- compare): ready, this field's, the next field's
    signal f_rdy, f_cur, f_nxt : std_logic_vector(3 downto 0) := (others => '0');
    signal r_fc1  : unsigned(7 downto 0) := to_unsigned(1, 8);    -- r_fc + 1
    signal f_ret  : std_logic_vector(1 downto 0) := (others => '0'); -- slot passed
    signal h_ok, s_on : std_logic := '0';    -- header's top reached; sprite on line
    signal s_wfree : std_logic := '1';
    signal n_bgs : signed(19 downto 0) := (others => '0');   -- static per mode

    ----------------------------------------------------------------------
    -- renderer
    ----------------------------------------------------------------------
    signal r_fsr  : std_logic_vector(3 downto 0) := (others => '0');
    signal r_fc   : unsigned(7 downto 0) := (others => '0');   -- field counter
    signal r_st   : unsigned(2 downto 0) := (others => '0');   -- FSM
    signal r_cc   : unsigned(7 downto 0) := (others => '0');
    signal r_clr, r_go, r_lp, r_wb, r_ph, r_em, s_eemit : std_logic := '0';
    signal r_cs   : std_logic := '0';                          -- the slot drawn
    signal r_e    : unsigned(5 downto 0) := (others => '0');
    signal r_t    : unsigned(3 downto 0) := (others => '0');
    signal r_line : signed(11 downto 0) := (others => '0');    -- the line drawn
    signal r_dry  : std_logic := '0';
    signal r_jb   : signed(15 downto 0) := (others => '0');
    signal r_y0, r_y1 : signed(15 downto 0) := (others => '0');
    signal r_w1, r_w2 : signed(25 downto 0) := (others => '0');
    signal r_w3   : std_logic_vector(30 downto 0) := (others => '0');
    signal r_d0, r_d1 : signed(15 downto 0) := (others => '0');
    signal r_k, r_khi : unsigned(4 downto 0) := (others => '0');
    signal r_bg  : unsigned(9 downto 0) := (others => '0');
    signal r_x, r_s : signed(25 downto 0) := (others => '0');
    signal r_dy  : signed(10 downto 0) := (others => '0');
    signal r_du, r_dv : signed(8 downto 0) := (others => '0');
    signal s_rp  : unsigned(6 downto 0);         -- the crossing's pixel ...
    signal s_rfi : unsigned(3 downto 0);         -- ... and 1/16 px fraction
    -- the activation's header fields
    signal h_top, h_bot : signed(11 downto 0) := (others => '0');
    signal h_x0  : unsigned(11 downto 0) := (others => '0');
    -- the two slots: on, ring entry, bottom line, x, counts
    type t_s12x2 is array (0 to 1) of signed(11 downto 0);
    type t_u12x2 is array (0 to 1) of unsigned(11 downto 0);
    type t_u2x2  is array (0 to 1) of unsigned(1 downto 0);
    type t_u5x2  is array (0 to 1) of unsigned(4 downto 0);
    type t_u3x2  is array (0 to 1) of unsigned(2 downto 0);
    signal sl_on, nx_on, dl_on : std_logic_vector(1 downto 0) := (others => '0');
    signal sl_ent : t_u2x2 := (others => (others => '0'));
    signal sl_bot : t_s12x2 := (others => (others => '0'));
    signal sl_x0, nx_x0, dl_x0 : t_u12x2 := (others => (others => '0'));
    signal sl_ne : t_u5x2 := (others => (others => '0'));
    signal sl_ns : t_u3x2 := (others => (others => '0'));
    signal c_ent : unsigned(1 downto 0) := (others => '0');    -- the drawn slot's
    signal c_ne  : unsigned(4 downto 0) := (others => '0');
    signal c_ns  : unsigned(2 downto 0) := (others => '0');

    -- numeral sprites: the record being walked (rs_), the span (r_s*), the
    -- glyph pipe's constants for the span in it (pa_)
    signal r_si, r_s11 : unsigned(3 downto 0) := (others => '0');
    signal rs_ly0, rs_ly1 : signed(11 downto 0) := (others => '0');
    signal rs_ga, rs_gb, pa_ga, pa_gb : unsigned(3 downto 0) := (others => '0');
    signal rs_sx0, rs_sw, rs_k, pa_k, pa_sx0, r_n : unsigned(7 downto 0) := (others => '0');
    signal rs_dux, rs_dvx, rs_duy, rs_dvy : signed(15 downto 0) := (others => '0');
    signal pa_ba, pa_bb : unsigned(9 downto 0) := (others => '0');
    signal pa_na, pa_nb, pa_sl, pa_dr, s_semit, g_fresh : std_logic := '0';
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
    signal g11_p : unsigned(6 downto 0) := (others => '0');
    signal g11_sl : std_logic := '0';
    signal g1_fx, g1_fy, g2_fx, g2_fy, g3_fx, g3_fy, g4_fx, g4_fy, g5_fy
        : unsigned(3 downto 0) := (others => '0');
    signal g1_x0, g1_x1, g1_y0, g1_y1 : signed(7 downto 0) := (others => '0');
    signal g2_ok, g3_ok : std_logic_vector(3 downto 0) := (others => '0');
    signal g_ra0, g_ra1, g_ra2, g_ra3 : unsigned(9 downto 0) := (others => '0');
    signal g_rd0, g_rd1, g_rd2, g_rd3 : std_logic_vector(3 downto 0) := (others => '1');
    signal g4_t00, g4_t10, g4_t01, g4_t11 : unsigned(3 downto 0) := (others => '0');
    signal g5_top, g5_bot : signed(10 downto 0) := (others => '0');
    signal g6_d  : signed(13 downto 0) := (others => '0');
    signal g7_hi, g7_lo : std_logic := '0';
    signal g7_m  : signed(19 downto 0) := (others => '0');
    signal g8_al : unsigned(4 downto 0) := (others => '0');

    -- deposit pipe (4 channels: DY FY DU DV; c = 0..2 for Y U V, F only Y)
    type t_s15x3 is array (0 to 2) of signed(14 downto 0);
    type t_s16x3 is array (0 to 2) of signed(15 downto 0);
    type t_s17x3 is array (0 to 2) of signed(16 downto 0);
    signal q1_v, q1_m, q2_v, q2_m, q3_v, q3_m : std_logic := '0';
    signal q1_sl, q2_sl, q3_sl : std_logic := '0';
    signal q1_p, q2_p, q3_p : unsigned(6 downto 0) := (others => '0');
    signal q1_fi : unsigned(3 downto 0) := (others => '0');
    signal q1_d, q2_d, q3_d : t_s16x3 := (others => (others => '0'));
    signal q2_pa, q2_pb : signed(14 downto 0) := (others => '0');
    signal q3_pr : signed(14 downto 0) := (others => '0');
    signal acc_v : std_logic := '0';
    signal acc_p : unsigned(6 downto 0) := (others => '0');
    signal acc_sl : std_logic := '0';
    signal acc_d : t_s16x3 := (others => (others => '0'));
    signal acc_f : signed(19 downto 0) := (others => '0');
    signal fl_v  : std_logic := '0';
    signal fl_a  : unsigned(7 downto 0) := (others => '0');
    signal fl_d  : t_s16x3 := (others => (others => '0'));
    signal fl_f  : signed(15 downto 0) := (others => '0');
    signal g10_c, g11_e, g_cp : t_s16x3 := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- line buffers: bank A / B (the line drawn and the line shown swap each
    -- line), each four 256 x 16 RAMs addressed slot | pixel
    ----------------------------------------------------------------------
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal la_dy, la_fy, la_du, la_dv, lb_dy, lb_fy, lb_du, lb_dv : t_lb := (others => (others => '0'));
    signal la_ra, la_wa, lb_ra, lb_wa : unsigned(7 downto 0) := (others => '0');
    signal la_we, lb_we : std_logic := '0';
    signal la_wdy, la_wfy, la_wdu, la_wdv : std_logic_vector(15 downto 0) := (others => '0');
    signal lb_wdy, lb_wfy, lb_wdu, lb_wdv : std_logic_vector(15 downto 0) := (others => '0');
    signal la_rdy, la_rfy, la_rdu, la_rdv : std_logic_vector(15 downto 0) := (others => '0');
    signal lb_rdy, lb_rfy, lb_rdu, lb_rdv : std_logic_vector(15 downto 0) := (others => '0');
    signal s_rbank : std_logic := '0';           -- the bank drawn into (r_line(0))
    signal rq_d : t_s16x3 := (others => (others => '0'));    -- renderer's read data
    signal rq_f : signed(15 downto 0) := (others => '0');
    signal dq_d : t_s16x3 := (others => (others => '0'));    -- display's read data
    signal dq_f : signed(15 downto 0) := (others => '0');
    signal r_ra : unsigned(7 downto 0) := (others => '0');     -- renderer read address

    attribute ram_style : string;
    attribute ram_style of rf   : signal is "block";
    attribute ram_style of tbl  : signal is "block";
    attribute ram_style of wram : signal is "block";
    attribute ram_style of la_dy : signal is "block";
    attribute ram_style of la_fy : signal is "block";
    attribute ram_style of la_du : signal is "block";
    attribute ram_style of la_dv : signal is "block";
    attribute ram_style of lb_dy : signal is "block";
    attribute ram_style of lb_fy : signal is "block";
    attribute ram_style of lb_du : signal is "block";
    attribute ram_style of lb_dv : signal is "block";
    attribute no_rw_check : boolean;
    attribute no_rw_check of rf   : signal is true;
    attribute no_rw_check of tbl  : signal is true;
    attribute no_rw_check of wram : signal is true;
    attribute no_rw_check of la_dy : signal is true;
    attribute no_rw_check of la_fy : signal is true;
    attribute no_rw_check of la_du : signal is true;
    attribute no_rw_check of la_dv : signal is true;
    attribute no_rw_check of lb_dy : signal is true;
    attribute no_rw_check of lb_fy : signal is true;
    attribute no_rw_check of lb_du : signal is true;
    attribute no_rw_check of lb_dv : signal is true;

    ----------------------------------------------------------------------
    -- display
    ----------------------------------------------------------------------
    signal dp1_w, dp1_f, dp2_w, dp2_f, dp3_w, dp3_f, dp4_w, dp5_w : std_logic := '0';
    signal dp1_a, dc_wa : unsigned(7 downto 0) := (others => '0');
    signal dc_we : std_logic := '0';
    signal sm    : t_s16x3 := (others => (others => '0'));   -- running sums
    signal dp4_f : signed(15 downto 0) := (others => '0');
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
    -- aspect (Q12 offset on the projected x axis), 2^20 / frame height, SD,
    -- and the dice the engine keeps up with (SD: 4 d20s, 6 of a mix)
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
            if registers_in(8)(1 downto 0) = "00" then           -- 0 4 8 12: SD
                s_sd <= '1';
                if registers_in(6)(0) = '1' then s_ncap <= to_unsigned(6, 5);
                else                             s_ncap <= to_unsigned(4, 5); end if;
            else
                s_sd   <= '0';
                s_ncap <= to_unsigned(20, 5);
            end if;
        end if;
    end process p_tid;

    -- per-field frame height; the generated avid's width, taken in vertical
    -- blanking
    p_fsetup : process(clk)
    begin
        if rising_edge(clk) then
            if s_fstart = '1' then
                if s_ilace = '1' then s_hf <= resize(s_H & '0', 12);
                else                  s_hf <= resize(s_H, 12); end if;
            end if;
            if s_estart = '1' then
                s_Wl   <= s_W;
                s_wlm1 <= s_W - 1;
            end if;
        end if;
    end process p_fsetup;

    ------------------------------------------------------------------------
    -- SLW decoder: the engine's records go to the ring entry at the write
    -- pointer (edges at 4 e + word, header at 80 + word, sprites at
    -- 8 (s + 11) + word); "done" completes the entry
    ------------------------------------------------------------------------
    p_slotw : process(clk)
    begin
        if rising_edge(clk) then
            e_we   <= '0';
            e_done <= '0';
            if u_slw = '1' then
                case to_integer(u_slp) is
                    when 0 | 1 | 2 | 3 =>
                        e_wa <= rg_wp & u_sls & u_slp(1 downto 0);
                        e_we <= '1';
                    when 4 | 5 | 6 =>
                        e_wa <= rg_wp & "10100" & u_slp(1 downto 0);
                        e_we <= '1';
                    when 8 | 9 | 10 | 11 | 12 | 13 | 14 =>
                        e_wa <= rg_wp & u_sls(3 downto 0) & u_slp(2 downto 0);
                        e_we <= '1';
                    when 17 =>
                        e_done <= '1';
                        e_tag  <= unsigned(u_slv(7 downto 0));
                    when 20 => n_bgs <= u_slv(19 downto 0);
                    when others => null;
                end case;
                e_wd <= std_logic_vector(u_slv);
            end if;
            if rg_st(to_integer(rg_wp)) = "00" then s_wfree <= '1';
            else                                    s_wfree <= '0'; end if;
        end if;
    end process p_slotw;

    ------------------------------------------------------------------------
    -- MICROCODED GEOMETRY ENGINE (CUBIST's, without the divider)
    --
    -- One ALU, the register file in ONE block-RAM copy (an instruction that
    -- reads register B takes a clock more: B is read after A), the program
    -- in block RAM.  Three or four clocks an instruction, +1 for a taken
    -- jump; shifts one clock a bit; a radix-4 Booth serial multiplier (MULSH
    -- starts it and waits for the product in one instruction); a restoring
    -- divider (DIV, 26 clocks); MIN / MAX off the operand compare; signed
    -- (ROMS, SRDS) and unsigned table reads.  It never stops: it computes
    -- the dice field after field, as the ring has room (see the microcode).
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
        variable v_dr : unsigned(22 downto 0);
    begin
        if rising_edge(clk) then
            if u_st = "001" then u_wlast <= u_we; end if;
            -- divider, restoring, one quotient bit a clock
            if dv_bsy = '1' then
                v_dr := dv_r(21 downto 0) & dv_n(25);
                dv_n <= dv_n(24 downto 0) & '0';
                if v_dr >= ('0' & dv_d) then
                    dv_r <= resize(v_dr - ('0' & dv_d), 22);
                    dv_q <= dv_q(24 downto 0) & '1';
                else
                    dv_r <= v_dr(21 downto 0);
                    dv_q <= dv_q(24 downto 0) & '0';
                end if;
                if dv_cnt = 0 then dv_bsy <= '0';
                else dv_cnt <= dv_cnt - 1; end if;
            end if;
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

            if u_done = '0' then
                case to_integer(u_st) is

                    -- FETCH (after a jump only): the ROM answers next cycle
                    when 0 =>
                        u_st <= "001";

                    -- DECODE: latch the instruction; register A is read
                    -- straight off the ROM output this cycle, and B next
                    -- cycle (state 6) for the ops that use it
                    when 1 =>
                        u_ir <= u_rom;
                        case to_integer(unsigned(u_rom(31 downto 27))) is
                            when 2 | 3 | 6 | 7 | 8 | 10 | 12 | 17 | 20 | 21 | 22 | 31 =>
                                u_st <= "110";
                            when others =>
                                u_st <= "010";
                        end case;

                    -- REGISTER B: A has arrived, B is addressed
                    when 6 =>
                        u_rda  <= u_rd;
                        u_rbop <= '1';
                        u_st   <= "010";

                    -- OPERAND LATCH: block-RAM outputs into fabric before any
                    -- arithmetic; every unit launches from here
                    when 2 =>
                        v_op2 := to_integer(unsigned(u_ir(31 downto 27)));
                        -- forwarding: the previous write lands on this read's
                        -- edge (an EBR read-during-write is undefined)
                        if u_wlast = '1' and unsigned(u_ir(19 downto 13)) = u_wa then
                            v_fa := u_wd;
                        elsif u_rbop = '1' then
                            v_fa := u_rda;
                        else
                            v_fa := u_rd;
                        end if;
                        v_fb := u_rd;           -- (read after the write landed)
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
                            when 8            => u_rsel <= x"7"; u_rwr <= '1';
                            when 6 | 7        => u_rsel <= x"5"; u_rwr <= '1';
                            when 14 | 16 | 23 | 28 => u_rsel <= x"6"; u_rwr <= '1';
                            when 25           => u_rsel <= x"8"; u_rwr <= '1';
                            when 31           => u_rsel <= x"9"; u_rwr <= '1';
                            when others       => u_rsel <= x"0"; u_rwr <= '0';
                        end case;
                        u_shc <= unsigned(u_ir(3 downto 0));
                        u_rbop <= '0';
                        u_wa  <= unsigned(u_ir(26 downto 20));
                        if signed(v_fa) < signed(v_fb) then u_blt <= '1';
                        else u_blt <= '0'; end if;
                        if v_fa /= x"00000000" then u_bnz <= '1';
                        else u_bnz <= '0'; end if;
                        u_jr <= unsigned(v_fa(9 downto 0));
                        case v_op2 is
                            when 8 =>                      -- DIV
                                dv_n   <= unsigned(v_fa(25 downto 0));
                                dv_d   <= unsigned(v_fb(21 downto 0));
                                dv_r   <= (others => '0');
                                dv_q   <= (others => '0');
                                dv_cnt <= to_unsigned(25, 5);
                                dv_bsy <= '1';
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
                        u_wd <= u_rd;           -- (the indexed read)
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
                            when 7 => v_r := signed(resize(dv_q, 32));  -- DIV
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
                            when 8       => if dv_bsy = '1' then v_stl := true; end if;
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

    -- CTL: the outside world, read live
    with to_integer(unsigned(u_ir(4 downto 0))) select u_ctlv <=
        signed(resize(unsigned(registers_in(0)), 32)) when 0,
        signed(resize(unsigned(registers_in(1)), 32)) when 1,
        signed(resize(unsigned(registers_in(2)), 32)) when 2,
        signed(resize(unsigned(registers_in(3)), 32)) when 3,
        signed(resize(s_Wl, 32)) when 6,
        signed(resize(s_hf, 32)) when 7,
        (0 => s_ilace, others => '0')  when 8,
        (0 => s_field, others => '0')  when 9,
        signed(resize(unsigned(registers_in(6)), 32)) when 11,
        signed(resize(unsigned(registers_in(7)), 32)) when 12,
        resize(s_xa, 32) when 13,
        signed(resize(s_rhf, 32)) when 14,
        (0 => s_wfree, others => '0') when 15,
        signed(resize(r_fc, 32)) when 16,
        signed(resize(s_ncap, 32)) when 17,
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

    -- the register file, one copy: A off the ROM in DECODE, B off the
    -- instruction in state 6, else the indexed address (LDX)
    p_rf : process(clk)
        variable v_ra : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf(to_integer(u_wa)) <= u_wd; end if;
            if u_st = "001" then    v_ra := unsigned(u_rom(19 downto 13));
            elsif u_st = "110" then v_ra := unsigned(u_ir(12 downto 6));
            else                    v_ra := u_ra; end if;
            u_rd <= rf(to_integer(v_ra));
        end if;
    end process p_rf;

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

    -- the crossing's pixel in its slot (clamped) and its 1/16 px fraction
    s_rp  <= (others => '0') when r_x(25) = '1' else
             (others => '1') when r_x(24 downto 23) /= "00" else
             unsigned(r_x(22 downto 16));
    s_rfi <= (others => '0') when r_x(25) = '1' else
             (others => '1') when r_x(24 downto 23) /= "00" else
             unsigned(r_x(15 downto 12));
    -- this clock steps an edge sub-line / a numeral pixel (p_rseq's loops)
    s_eemit <= '1' when r_fsr(3) = '0' and r_clr = '0' and r_go = '0' and r_st = 5
                        and r_ph = '0' and r_lp = '1' else '0';
    s_semit <= '1' when r_fsr(3) = '0' and r_clr = '0' and r_go = '0' and r_st = 5
                        and r_ph = '1' and r_lp = '1' else '0';
    s_rbank <= r_line(0);
    r_s11   <= r_si + 11;

    ------------------------------------------------------------------------
    -- RING STATE and RENDER SEQUENCER.  At each displayed line's start
    -- (r_go) it draws the NEXT line:
    --   RET  the slots whose die the beam has passed: off, entry freed;
    --   ACT  ring entries in order: an old field's is freed; this field's,
    --        once its top line is reached, takes a free slot (header read);
    --   PUB  the slots to show on that line;
    --   SLOT / DIE  each slot's die, as NAT20 draws its one: the crossing
    --        records, then the numeral sprites.
    -- At the field start: buffers cleared, then lines -128 .. -1 dry (no
    -- deposits) and line 0.  States: 0 idle, 2 RET, 3 ACT, 4 PUB, 6 SLOT,
    -- 5 DIE, 7 END.
    ------------------------------------------------------------------------
    p_rseq : process(clk)
        variable v_y, v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            r_wwe <= '0';
            r_em  <= '0';
            sp_mk <= '0';
            r_fsr <= r_fsr(2 downto 0) & s_estart;
            r_go  <= g_avid and not g_avid_q;          -- a displayed line starts

            for i in 0 to 3 loop
                f_rdy(i) <= '0';  f_cur(i) <= '0';  f_nxt(i) <= '0';
                if rg_st(i) = "01" then f_rdy(i) <= '1'; end if;
                if rg_tag(i) = r_fc then f_cur(i) <= '1'; end if;
                if rg_tag(i) = r_fc1 then f_nxt(i) <= '1'; end if;
            end loop;

            -- the engine completes the entry at its write pointer
            if e_done = '1' then
                rg_st(to_integer(rg_wp))  <= "01";
                rg_tag(to_integer(rg_wp)) <= e_tag;
                rg_wp <= rg_wp + 1;
            end if;

            if r_fsr(3) = '1' then
                -- field start: the dice still on are done with
                r_fc  <= r_fc + 1;
                r_fc1 <= r_fc + 2;
                for i in 0 to 3 loop
                    if rg_st(i) = "10" then rg_st(i) <= "00"; end if;
                end loop;
                sl_on  <= "00";
                nx_on  <= "00";
                r_clr  <= '1';
                r_cc   <= (others => '0');
                r_st   <= to_unsigned(0, 3);
                r_line <= to_signed(-C_PRE, 12);
                r_lp   <= '0';
                r_wb   <= '0';
            elsif r_clr = '1' then
                r_cc <= r_cc + 1;
                if r_cc = 255 then
                    r_clr <= '0';
                    r_bg  <= bg_y;
                    r_st  <= to_unsigned(2, 3);
                end if;
            elsif r_go = '1' then
                -- show what was drawn for this line, draw the next
                dl_on  <= nx_on;
                dl_x0  <= nx_x0;
                r_line <= r_line + 1;
                r_bg   <= bg_y;
                r_st   <= to_unsigned(2, 3);
                r_lp   <= '0';
                r_wb   <= '0';
            else
                case to_integer(r_st) is
                    when 2 =>                               -- RET
                        for s in 0 to 1 loop
                            f_ret(s) <= '0';
                            if sl_on(s) = '1' and r_line >= sl_bot(s) then
                                f_ret(s) <= '1';
                            end if;
                        end loop;
                        r_jb  <= shift_left(resize(r_line, 16), 4);
                        r_dry <= r_line(11);
                        r_t   <= (others => '0');
                        r_st  <= to_unsigned(1, 3);
                    when 1 =>                               -- RET, applied
                        for s in 0 to 1 loop
                            if f_ret(s) = '1' then
                                sl_on(s) <= '0';
                                rg_st(to_integer(sl_ent(s))) <= "00";
                            end if;
                        end loop;
                        r_st  <= to_unsigned(3, 3);
                    when 3 =>                               -- ACT
                        case to_integer(r_t) is
                            when 0 =>
                                if f_rdy(to_integer(rg_ap)) = '0' then
                                    r_st <= to_unsigned(4, 3);      -- none waiting
                                elsif f_cur(to_integer(rg_ap)) = '1' then
                                    w_ra <= rg_ap & "1010000";      -- header h0
                                    r_t  <= to_unsigned(1, 4);
                                elsif f_nxt(to_integer(rg_ap)) = '1' then
                                    r_st <= to_unsigned(4, 3);      -- next field's
                                else
                                    rg_st(to_integer(rg_ap)) <= "00";   -- stale
                                    rg_ap <= rg_ap + 1;
                                end if;
                            when 1 =>
                                w_ra <= rg_ap & "1010001";          -- h1
                                r_t  <= to_unsigned(2, 4);
                            when 2 =>                               -- h0 arrives
                                w_ra  <= rg_ap & "1010010";         -- h2
                                h_top <= signed(w_rd(11 downto 0));
                                h_bot <= signed(w_rd(23 downto 12));
                                r_t   <= to_unsigned(3, 4);
                            when 3 =>                               -- h1 arrives
                                h_x0 <= unsigned(w_rd(11 downto 0));
                                h_ok <= '0';
                                if h_top <= r_line then h_ok <= '1'; end if;
                                r_t  <= to_unsigned(4, 4);
                            when others =>                          -- h2 arrives
                                if h_ok = '1' and (sl_on(0) = '0' or sl_on(1) = '0') then
                                    if sl_on(0) = '0' then
                                        sl_on(0)  <= '1';
                                        sl_ent(0) <= rg_ap;
                                        sl_bot(0) <= h_bot;
                                        sl_x0(0)  <= h_x0;
                                        sl_ne(0)  <= unsigned(w_rd(4 downto 0));
                                        sl_ns(0)  <= unsigned(w_rd(7 downto 5));
                                    else
                                        sl_on(1)  <= '1';
                                        sl_ent(1) <= rg_ap;
                                        sl_bot(1) <= h_bot;
                                        sl_x0(1)  <= h_x0;
                                        sl_ne(1)  <= unsigned(w_rd(4 downto 0));
                                        sl_ns(1)  <= unsigned(w_rd(7 downto 5));
                                    end if;
                                    rg_st(to_integer(rg_ap)) <= "10";
                                    rg_ap <= rg_ap + 1;
                                    r_t   <= (others => '0');
                                else
                                    r_st <= to_unsigned(4, 3);
                                end if;
                        end case;
                    when 4 =>                               -- PUB
                        if r_dry = '0' then
                            nx_on <= sl_on;
                            nx_x0 <= sl_x0;
                        end if;
                        r_cs <= '0';
                        r_st <= to_unsigned(6, 3);
                    when 6 =>                               -- SLOT
                        if sl_on(0) = '1' and r_cs = '0' then
                            c_ent <= sl_ent(0);  c_ne <= sl_ne(0);  c_ns <= sl_ns(0);
                            r_st  <= to_unsigned(5, 3);
                        elsif sl_on(1) = '1' and r_cs = '1' then
                            c_ent <= sl_ent(1);  c_ne <= sl_ne(1);  c_ns <= sl_ns(1);
                            r_st  <= to_unsigned(5, 3);
                        elsif r_cs = '0' then
                            r_cs <= '1';
                        else
                            r_st <= to_unsigned(7, 3);
                        end if;
                        r_e  <= (others => '0');
                        r_si <= (others => '0');
                        r_ph <= '0';
                        r_t  <= (others => '0');
                        r_lp <= '0';
                        r_wb <= '0';
                    when 7 =>                               -- END
                        if r_dry = '1' then
                            r_line <= r_line + 1;           -- the next dry line,
                            r_st   <= to_unsigned(2, 3);    -- or line 0
                        else
                            r_st   <= to_unsigned(0, 3);
                        end if;
                    when 5 =>                               -- DIE
                        if r_ph = '0' then
                            -------------------------------------- edges
                            if r_lp = '1' then
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
                                r_wwa <= c_ent & r_e(4 downto 0) & "01";
                                r_wwd <= std_logic_vector(resize(r_x, 32));
                                r_wb  <= '0';
                                r_e   <= r_e + 1;
                                r_t   <= (others => '0');
                            else
                                case to_integer(r_t) is
                                    when 0 =>
                                        if r_e >= c_ne then
                                            r_ph <= '1';    -- on to the numerals
                                        else
                                            w_ra <= c_ent & r_e(4 downto 0) & "00";
                                            r_t  <= to_unsigned(1, 4);
                                        end if;
                                    when 1 =>
                                        w_ra <= c_ent & r_e(4 downto 0) & "01";
                                        r_t  <= to_unsigned(2, 4);
                                    when 2 =>               -- w0 arrives
                                        w_ra <= c_ent & r_e(4 downto 0) & "10";
                                        r_y0 <= signed(w_rd(15 downto 0));
                                        r_y1 <= signed(w_rd(31 downto 16));
                                        r_t  <= to_unsigned(3, 4);
                                    when 3 =>               -- w1 (x) arrives
                                        w_ra <= c_ent & r_e(4 downto 0) & "11";
                                        r_w1 <= signed(w_rd(25 downto 0));
                                        r_d0 <= r_y0 - r_jb;
                                        r_d1 <= r_y1 - r_jb;
                                        r_t  <= to_unsigned(4, 4);
                                    when 4 =>               -- w2 arrives
                                        r_w2 <= signed(w_rd(25 downto 0));
                                        if r_d0 < 16 and r_d1 > 0 then
                                            if r_d0 < 0 then r_k <= (others => '0');
                                            else r_k <= unsigned(r_d0(4 downto 0)); end if;
                                            if r_d1 > 16 then r_khi <= to_unsigned(16, 5);
                                            else r_khi <= unsigned(r_d1(4 downto 0)); end if;
                                            r_t <= to_unsigned(5, 4);
                                        else
                                            r_e <= r_e + 1; -- misses the line
                                            r_t <= (others => '0');
                                        end if;
                                    when 5 =>               -- w3 arrives
                                        r_w3 <= w_rd(30 downto 0);
                                        r_t  <= to_unsigned(6, 4);
                                    when others =>
                                        -- x (at y0, or this line's), slope,
                                        -- colour step: a silhouette's face
                                        -- colour less (left) or plus (right)
                                        -- this line's backdrop
                                        r_x <= r_w1;
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
                        else
                            -------------------------------------- numerals
                            if r_lp = '1' then
                                -- one pixel of the span into the glyph pipe
                                r_su <= r_su + resize(rs_dux, 20);
                                r_sv <= r_sv + resize(rs_dvx, 20);
                                r_n  <= r_n - 1;
                                if r_n = 0 then
                                    r_lp <= '0';
                                    r_wb <= '1';
                                end if;
                            elsif r_wb = '1' then
                                -- end of the span: close its run, keep the
                                -- next line's texel coordinates
                                if r_t = 0 then
                                    sp_mk <= '1';
                                    r_wwe <= '1';
                                    r_wwa <= c_ent & r_s11 & "101";
                                    r_wwd <= std_logic_vector(resize(r_us + resize(rs_duy, 20), 32));
                                    r_t   <= to_unsigned(1, 4);
                                else
                                    r_wwe <= '1';
                                    r_wwa <= c_ent & r_s11 & "110";
                                    r_wwd <= std_logic_vector(resize(r_vs + resize(rs_dvy, 20), 32));
                                    r_wb  <= '0';
                                    r_si  <= r_si + 1;
                                    r_t   <= (others => '0');
                                end if;
                            else
                                case to_integer(r_t) is
                                    when 0 =>
                                        if r_si >= c_ns then
                                            r_cs <= '1';        -- the die is done
                                            if r_cs = '1' then r_st <= to_unsigned(7, 3);
                                            else               r_st <= to_unsigned(6, 3); end if;
                                        else
                                            w_ra <= c_ent & r_s11 & "000";
                                            r_t  <= to_unsigned(1, 4);
                                        end if;
                                    when 1 =>
                                        w_ra <= c_ent & r_s11 & "001";
                                        r_t  <= to_unsigned(2, 4);
                                    when 2 =>               -- w0: lines, glyphs
                                        w_ra <= c_ent & r_s11 & "010";
                                        rs_ly0 <= signed(w_rd(11 downto 0));
                                        rs_ly1 <= signed(w_rd(23 downto 12));
                                        rs_ga  <= unsigned(w_rd(27 downto 24));
                                        rs_gb  <= unsigned(w_rd(31 downto 28));
                                        r_t    <= to_unsigned(3, 4);
                                    when 3 =>               -- w1: span, AA scale
                                        w_ra <= c_ent & r_s11 & "011";
                                        rs_sx0 <= unsigned(w_rd(7 downto 0));
                                        rs_sw  <= unsigned(w_rd(16 downto 9));
                                        rs_k   <= unsigned(w_rd(24 downto 17));
                                        s_on   <= '0';
                                        if r_line >= rs_ly0 and r_line < rs_ly1 then
                                            s_on <= '1';
                                        end if;
                                        r_t <= to_unsigned(4, 4);
                                    when 4 =>               -- w2: steps across
                                        w_ra <= c_ent & r_s11 & "100";
                                        rs_dux <= signed(w_rd(15 downto 0));
                                        rs_dvx <= signed(w_rd(31 downto 16));
                                        if s_on = '1' then
                                            r_t  <= to_unsigned(5, 4);
                                        else
                                            r_si <= r_si + 1;   -- off its lines
                                            r_t  <= (others => '0');
                                        end if;
                                    when 5 =>               -- w3: steps down
                                        w_ra <= c_ent & r_s11 & "101";
                                        rs_duy <= signed(w_rd(15 downto 0));
                                        rs_dvy <= signed(w_rd(31 downto 16));
                                        r_t  <= to_unsigned(6, 4);
                                    when 6 =>               -- w4: ink step
                                        w_ra <= c_ent & r_s11 & "110";
                                        rs_w4 <= w_rd(28 downto 0);
                                        r_t   <= to_unsigned(7, 4);
                                    when 7 =>               -- w5: u at this line
                                        r_su <= signed(w_rd(19 downto 0));
                                        r_t  <= to_unsigned(8, 4);
                                    when 8 =>               -- w6: v at this line
                                        r_sv <= signed(w_rd(19 downto 0));
                                        r_t  <= to_unsigned(9, 4);
                                    when others =>
                                        -- the span starts, with the glyph
                                        -- pipe's constants (safe to switch:
                                        -- the last span is 12+ clocks out)
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
                                        pa_sl <= r_cs;
                                        pa_dr <= r_dry;
                                        r_lp  <= '1';
                                        r_t   <= (others => '0');
                                end case;
                            end if;
                        end if;
                    when others => null;                    -- idle
                end case;
            end if;
        end if;
    end process p_rseq;

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
            -- across: top / bottom rows, Q4 code, both less 128 (the left
            -- taps less 8: their top bit flipped, read signed), so g6 needn't
            -- subtract the zero level
            g5_top <= shift_left(resize(signed(g4_t00 xor "1000"), 11), 4)
                      + (signed(resize(g4_t10, 6)) - signed(resize(g4_t00, 6)))
                        * signed('0' & g4_fx);
            g5_bot <= shift_left(resize(signed(g4_t01 xor "1000"), 11), 4)
                      + (signed(resize(g4_t11, 6)) - signed(resize(g4_t01, 6)))
                        * signed('0' & g4_fx);
            g5_vl <= g4_vl;  g5_mk <= g4_mk;  g5_last <= g4_last;
            g5_fy <= g4_fy;

            ------------------------------------------------------ g6
            -- down: the distance code, Q8, less the zero level (8 x 256)
            v_d  := resize((g5_bot - g5_top) * signed('0' & g5_fy), 14);
            v_d  := shift_left(resize(g5_top, 14), 4) + v_d;
            g6_d <= v_d;
            g6_vl <= g5_vl;  g6_mk <= g5_mk;  g6_last <= g5_last;

            ------------------------------------------------------ g7
            -- past +-1024 the ramp is over either way (k >= 1 px a texel):
            -- saturate, and multiply only the 11 bits between (the compare
            -- sits beside the multiply, not after g6's sum)
            g7_m <= g6_d(10 downto 0) * signed('0' & pa_k);
            g7_hi <= '0';  g7_lo <= '0';
            if g6_d > 1023 then g7_hi <= '1'; end if;
            if g6_d < -1024 then g7_lo <= '1'; end if;
            g7_vl <= g6_vl;  g7_mk <= g6_mk;  g7_last <= g6_last;

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
            g11_vl <= g10_vl and not pa_dr;     -- (dry lines deposit nothing)
            g11_mk <= g10_mk;
            g11_sl <= pa_sl;
            if g10_vl = '1' then
                for c in 0 to 2 loop
                    g11_e(c) <= g10_c(c) - g_cp(c);
                    g_cp(c)  <= g10_c(c);
                end loop;
                -- the pixel: the span's first, then one on (a span's run
                -- marker re-arms the first)
                if g_fresh = '1' then g11_p <= pa_sx0(6 downto 0);
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
    -- DEPOSIT PIPE.  q1: a crossing (slot, pixel, fraction, colour step) or
    -- a numeral pixel (slot, pixel, change of its ink step; no fraction),
    -- whose line-buffer read was issued with it (dry lines: none); q2: the
    -- luma step * fraction in two halves; q3: the product, read data in
    -- fabric; then the run accumulator: a sample in a new pixel flushes
    -- the last run (one write) and starts from the buffer's word, a sample
    -- in the same pixel adds on.  Within a record or a span the pixels only
    -- move one way, so a run's write never meets a read of the same word;
    -- records and spans are 8+ clocks apart.
    ------------------------------------------------------------------------
    p_dep : process(clk)
        variable v_new : std_logic;
    begin
        if rising_edge(clk) then
            -- q1
            q1_v <= '0';
            q1_m <= r_em or g11_mk;
            if s_eemit = '1' then
                q1_v  <= not r_dry;
                q1_p  <= s_rp;
                q1_sl <= r_cs;
                q1_fi <= s_rfi;
                q1_d(0) <= resize(r_dy, 16);
                q1_d(1) <= resize(r_du, 16);
                q1_d(2) <= resize(r_dv, 16);
            elsif g11_vl = '1' then
                q1_v  <= '1';
                q1_p  <= g11_p;
                q1_sl <= g11_sl;
                q1_fi <= (others => '0');
                q1_d  <= g11_e;
            end if;
            -- q2: luma step x fraction
            q2_v <= q1_v;  q2_m <= q1_m;  q2_p <= q1_p;  q2_sl <= q1_sl;  q2_d <= q1_d;
            q2_pa <= (others => '0');
            q2_pb <= (others => '0');
            if q1_fi(3) = '1' and q1_fi(2) = '1' then
                q2_pa <= shift_left(resize(q1_d(0), 15), 3) + shift_left(resize(q1_d(0), 15), 2);
            elsif q1_fi(3) = '1' then
                q2_pa <= shift_left(resize(q1_d(0), 15), 3);
            elsif q1_fi(2) = '1' then
                q2_pa <= shift_left(resize(q1_d(0), 15), 2);
            end if;
            if q1_fi(1) = '1' and q1_fi(0) = '1' then
                q2_pb <= shift_left(resize(q1_d(0), 15), 1) + resize(q1_d(0), 15);
            elsif q1_fi(1) = '1' then
                q2_pb <= shift_left(resize(q1_d(0), 15), 1);
            elsif q1_fi(0) = '1' then
                q2_pb <= resize(q1_d(0), 15);
            end if;
            -- q3
            q3_v <= q2_v;  q3_m <= q2_m;  q3_p <= q2_p;  q3_sl <= q2_sl;  q3_d <= q2_d;
            q3_pr <= q2_pa + q2_pb;
            -- run accumulator
            if q3_v = '1' and (acc_v = '0' or q3_p /= acc_p or q3_sl /= acc_sl) then v_new := '1';
            else v_new := '0'; end if;
            fl_v <= '0';
            if (v_new = '1' or q3_m = '1') and acc_v = '1' then
                fl_v <= '1';
                fl_a <= acc_sl & acc_p;
                fl_d <= acc_d;
                fl_f <= acc_f(19 downto 4);
            end if;
            if q3_v = '1' then
                for c in 0 to 2 loop
                    if v_new = '1' then acc_d(c) <= rq_d(c) + q3_d(c);
                    else                acc_d(c) <= acc_d(c) + q3_d(c); end if;
                end loop;
                if v_new = '1' then acc_f <= shift_left(resize(rq_f, 20), 4) + resize(q3_pr, 20);
                else                acc_f <= acc_f + resize(q3_pr, 20); end if;
                acc_p  <= q3_p;
                acc_sl <= q3_sl;
                acc_v  <= '1';
            elsif q3_m = '1' then
                acc_v <= '0';
            end if;
        end if;
    end process p_dep;

    ------------------------------------------------------------------------
    -- LINE BUFFERS.  Bank r_line(0) is drawn into, the other shown.  Each
    -- bank's one read address: the renderer's sample pixel, or the
    -- display's; its one write: the field-start clear, the display clearing
    -- behind itself, or a run flush.
    ------------------------------------------------------------------------
    p_lbport : process(clk)
        variable v_rel0, v_rel1 : signed(12 downto 0);
        variable v_in0, v_in1 : std_logic;
        variable v_da, v_ra : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- the display: which slot's die is under the beam
            v_rel0 := signed('0' & g_cnt) - signed('0' & (dl_x0(0) + C_XSHIFT));
            v_rel1 := signed('0' & g_cnt) - signed('0' & (dl_x0(1) + C_XSHIFT));
            v_in0 := '0';
            v_in1 := '0';
            if g_avid = '1' and dl_on(0) = '1' and v_rel0 >= 0 and v_rel0 < 128 then v_in0 := '1'; end if;
            if g_avid = '1' and dl_on(1) = '1' and v_rel1 >= 0 and v_rel1 < 128 then v_in1 := '1'; end if;
            dp1_f <= '0';
            if v_in0 = '1' then
                v_da := '0' & unsigned(v_rel0(6 downto 0));
                if v_rel0 = 0 then dp1_f <= '1'; end if;
            else
                v_da := '1' & unsigned(v_rel1(6 downto 0));
                if v_rel1 = 0 then dp1_f <= '1'; end if;
            end if;
            dp1_w <= v_in0 or v_in1;
            dp1_a <= v_da;
            -- the renderer: the sample issued this clock
            if s_eemit = '1' then v_ra := r_cs & s_rp;
            else                  v_ra := g11_sl & g11_p; end if;
            if s_rbank = '0' then la_ra <= v_ra; lb_ra <= v_da;
            else                  la_ra <= v_da; lb_ra <= v_ra; end if;
            -- clear behind the display: the word read last clock
            dc_we <= dp1_w;
            dc_wa <= dp1_a;

            la_we <= '0';
            lb_we <= '0';
            if r_clr = '1' then
                la_we <= '1';  la_wa <= r_cc;
                lb_we <= '1';  lb_wa <= r_cc;
                la_wdy <= (others => '0'); la_wfy <= (others => '0');
                la_wdu <= (others => '0'); la_wdv <= (others => '0');
                lb_wdy <= (others => '0'); lb_wfy <= (others => '0');
                lb_wdu <= (others => '0'); lb_wdv <= (others => '0');
            else
                if s_rbank = '0' then
                    -- A drawn, B shown
                    if fl_v = '1' then
                        la_we <= '1';  la_wa <= fl_a;
                        la_wdy <= std_logic_vector(fl_d(0)); la_wfy <= std_logic_vector(fl_f);
                        la_wdu <= std_logic_vector(fl_d(1)); la_wdv <= std_logic_vector(fl_d(2));
                    end if;
                    if dc_we = '1' then
                        lb_we <= '1';  lb_wa <= dc_wa;
                        lb_wdy <= (others => '0'); lb_wfy <= (others => '0');
                        lb_wdu <= (others => '0'); lb_wdv <= (others => '0');
                    end if;
                else
                    if fl_v = '1' then
                        lb_we <= '1';  lb_wa <= fl_a;
                        lb_wdy <= std_logic_vector(fl_d(0)); lb_wfy <= std_logic_vector(fl_f);
                        lb_wdu <= std_logic_vector(fl_d(1)); lb_wdv <= std_logic_vector(fl_d(2));
                    end if;
                    if dc_we = '1' then
                        la_we <= '1';  la_wa <= dc_wa;
                        la_wdy <= (others => '0'); la_wfy <= (others => '0');
                        la_wdu <= (others => '0'); la_wdv <= (others => '0');
                    end if;
                end if;
            end if;

            -- read data into fabric before any arithmetic
            if s_rbank = '0' then
                rq_d(0) <= signed(la_rdy); rq_f <= signed(la_rfy);
                rq_d(1) <= signed(la_rdu); rq_d(2) <= signed(la_rdv);
                dq_d(0) <= signed(lb_rdy); dq_f <= signed(lb_rfy);
                dq_d(1) <= signed(lb_rdu); dq_d(2) <= signed(lb_rdv);
            else
                rq_d(0) <= signed(lb_rdy); rq_f <= signed(lb_rfy);
                rq_d(1) <= signed(lb_rdu); rq_d(2) <= signed(lb_rdv);
                dq_d(0) <= signed(la_rdy); dq_f <= signed(la_rfy);
                dq_d(1) <= signed(la_rdu); dq_d(2) <= signed(la_rdv);
            end if;
        end if;
    end process p_lbport;

    p_la_dy : process(clk)
    begin
        if rising_edge(clk) then
            if la_we = '1' then la_dy(to_integer(la_wa)) <= la_wdy; end if;
            la_rdy <= la_dy(to_integer(la_ra));
        end if;
    end process p_la_dy;

    p_la_fy : process(clk)
    begin
        if rising_edge(clk) then
            if la_we = '1' then la_fy(to_integer(la_wa)) <= la_wfy; end if;
            la_rfy <= la_fy(to_integer(la_ra));
        end if;
    end process p_la_fy;

    p_la_du : process(clk)
    begin
        if rising_edge(clk) then
            if la_we = '1' then la_du(to_integer(la_wa)) <= la_wdu; end if;
            la_rdu <= la_du(to_integer(la_ra));
        end if;
    end process p_la_du;

    p_la_dv : process(clk)
    begin
        if rising_edge(clk) then
            if la_we = '1' then la_dv(to_integer(la_wa)) <= la_wdv; end if;
            la_rdv <= la_dv(to_integer(la_ra));
        end if;
    end process p_la_dv;

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

    p_lb_dv : process(clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then lb_dv(to_integer(lb_wa)) <= lb_wdv; end if;
            lb_rdv <= lb_dv(to_integer(lb_ra));
        end if;
    end process p_lb_dv;

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
    -- fabric at stage 3.  Running sums of D across a die's slot (restarting
    -- at its first pixel), less F (luma), over 16 sub-lines, over the
    -- backdrop.
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

            ------------------------------------------------------ stage 3
            for c in 0 to 2 loop
                if dp3_f = '1' then sm(c) <= dq_d(c);
                else                 sm(c) <= sm(c) + dq_d(c); end if;
            end loop;
            dp4_f <= dq_f;

            ------------------------------------------------------ stage 4
            dp5_v(0) <= resize(sm(0), 17) - resize(dp4_f, 17);
            dp5_v(1) <= resize(sm(1), 17);
            dp5_v(2) <= resize(sm(2), 17);

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

end architecture dicefall;
