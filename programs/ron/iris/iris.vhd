-- iris.vhd  (v0.4 -- IRIS: perspective dot-tunnel eye, three-layer colour split)
--
-- THE PICTURE: a large iris circle nearly filling the screen, a dark pupil
-- anywhere inside it, and hundreds of round dots on spiral arms sweeping
-- from the pupil to the rim, growing geometrically as they go.  Drawn three
-- times (R, G, B) with a distance-growing rotational/radial split.
--
-- GEOMETRY (the whole trick): the pupil circle and the iris circle are two
-- nested circles.  Every pair of nested circles is a coaxal pencil with two
-- LIMITING POINTS a (inside the pupil) and b (outside the iris); the Moebius
-- map  w = (z-a)/(z-b)  sends BOTH circles to circles about w=0.  In the
-- w-plane the dot field is an ordinary log-polar lattice (geometric rings,
-- A dots per ring, twist = per-ring rotation); because a Moebius map sends
-- circles to circles, every ring and every dot is a TRUE CIRCLE on screen,
-- squeezed on the near side and fanned on the far side -- the cone seen in
-- perspective.  We never form w: a vectoring CORDIC gives |z-a|, arg(z-a),
-- |z-b|, arg(z-b); a per-frame log table turns the magnitudes straight into
-- ring units and the angles subtract to theta = arg w.
--   u     = (L - L_ref) / log2(g)      ring coordinate (ring k = floor u)
--   v     = theta*A + T*(k+1/2+phase)  arm coordinate (dot a = round v)
--   dot   :  S(dv) <= B(du)
--           B(du) = (1 - (e^x-1)^2/K^2) e^-x, x = du*ln g   (per-frame table)
--           S(dv) = 2(1 - cos(dv*2pi/A)) / K^2                (per-frame table)
--           -> |w_pix/w_dot - 1|^2 <= K^2 exactly: a true circle in w-space
--           K = fill*min(tanh(ln g/2), sin(pi/A)): dots kiss their neighbours
--
-- LINE-RATE GEOMETRY ENGINE: the log-polar coordinates are smooth, so they
-- are evaluated every 8 px by ONE serial CORDIC (4 steps of 3 iterations,
-- lanes a and b time-shared, 8 clocks per sample) plus one log lane, one
-- line ahead of the beam, into a 4-EBR sample buffer.  There is no
-- multiplier in the engine: the log table is pre-scaled to ring units
-- (u), theta*A comes from two small per-frame tables (hi/lo halves of the
-- angle) and the layer split from a table indexed by u.  The pixel path
-- runs a DDA between samples and does the per-pixel dot work with ONE dot
-- machine time-shared by the three layers (each layer is evaluated every
-- third pixel and held), table lookups, and an anti-alias ramp scaled by the
-- local conformal factor |z-a||z-b|/|a-b|, capped for sub-pixel dots.
-- The ring lattice is anchored at a REFERENCE pupil of 6% of the iris.
--
-- SEQUENCER: a microcoded accumulator machine (register file and program in
-- EBR) runs in blanking only (measured hblank/vblank budgets), probes the
-- engine for every log/angle reference it needs (so CORDIC gain and ROM
-- rounding cancel by construction), one serial multiplier / divider, and
-- rebuilds the five per-frame tables across the following hblanks.
--
-- Controls: K1/K2 pupil X/Y, K3 pupil size, K4 arms (6..96), K5 rings
-- (3..48), K6 twist (bipolar, dead zone), P12 split; S9 Light/Ink,
-- S10 Still/Flow, S11 Framed/Full.  S7/S8 (video dots/backdrop) are inert
-- in this build: the video halftone path did not fit the HX4K.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture iris of program_top is

    constant C_LATENCY  : integer := 20;
    constant C_MID      : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLANK_MARGIN : integer := 40;  -- cycles kept free at the end of a blank

    -- CORDIC: 12 iterations as 4 serial steps of 3, 2 guard bits, 18-bit data
    constant C_CG : integer := 2;
    constant C_CW : integer := 19;   -- 18 overflowed at the screen corners (magnitude x CORDIC gain)

    type t_c_atan is array(0 to 15) of integer;
    constant C_ATAN : t_c_atan := (16384, 9672, 5110, 2594, 1302, 652, 326, 163, 81, 41, 20, 10, 5, 3, 1, 1);
    type t_c_fun is array(0 to 511) of unsigned(15 downto 0);
    constant C_FUN : t_c_fun := (
        to_unsigned(0,16), to_unsigned(402,16), to_unsigned(804,16), to_unsigned(1206,16), to_unsigned(1608,16), to_unsigned(2009,16), to_unsigned(2411,16), to_unsigned(2811,16),
        to_unsigned(3212,16), to_unsigned(3612,16), to_unsigned(4011,16), to_unsigned(4410,16), to_unsigned(4808,16), to_unsigned(5205,16), to_unsigned(5602,16), to_unsigned(5998,16),
        to_unsigned(6393,16), to_unsigned(6787,16), to_unsigned(7180,16), to_unsigned(7571,16), to_unsigned(7962,16), to_unsigned(8351,16), to_unsigned(8740,16), to_unsigned(9127,16),
        to_unsigned(9512,16), to_unsigned(9896,16), to_unsigned(10279,16), to_unsigned(10660,16), to_unsigned(11039,16), to_unsigned(11417,16), to_unsigned(11793,16), to_unsigned(12167,16),
        to_unsigned(12540,16), to_unsigned(12910,16), to_unsigned(13279,16), to_unsigned(13646,16), to_unsigned(14010,16), to_unsigned(14373,16), to_unsigned(14733,16), to_unsigned(15091,16),
        to_unsigned(15447,16), to_unsigned(15800,16), to_unsigned(16151,16), to_unsigned(16500,16), to_unsigned(16846,16), to_unsigned(17190,16), to_unsigned(17531,16), to_unsigned(17869,16),
        to_unsigned(18205,16), to_unsigned(18538,16), to_unsigned(18868,16), to_unsigned(19195,16), to_unsigned(19520,16), to_unsigned(19841,16), to_unsigned(20160,16), to_unsigned(20475,16),
        to_unsigned(20788,16), to_unsigned(21097,16), to_unsigned(21403,16), to_unsigned(21706,16), to_unsigned(22006,16), to_unsigned(22302,16), to_unsigned(22595,16), to_unsigned(22884,16),
        to_unsigned(23170,16), to_unsigned(23453,16), to_unsigned(23732,16), to_unsigned(24008,16), to_unsigned(24279,16), to_unsigned(24548,16), to_unsigned(24812,16), to_unsigned(25073,16),
        to_unsigned(25330,16), to_unsigned(25583,16), to_unsigned(25833,16), to_unsigned(26078,16), to_unsigned(26320,16), to_unsigned(26557,16), to_unsigned(26791,16), to_unsigned(27020,16),
        to_unsigned(27246,16), to_unsigned(27467,16), to_unsigned(27684,16), to_unsigned(27897,16), to_unsigned(28106,16), to_unsigned(28311,16), to_unsigned(28511,16), to_unsigned(28707,16),
        to_unsigned(28899,16), to_unsigned(29086,16), to_unsigned(29269,16), to_unsigned(29448,16), to_unsigned(29622,16), to_unsigned(29792,16), to_unsigned(29957,16), to_unsigned(30118,16),
        to_unsigned(30274,16), to_unsigned(30425,16), to_unsigned(30572,16), to_unsigned(30715,16), to_unsigned(30853,16), to_unsigned(30986,16), to_unsigned(31114,16), to_unsigned(31238,16),
        to_unsigned(31357,16), to_unsigned(31471,16), to_unsigned(31581,16), to_unsigned(31686,16), to_unsigned(31786,16), to_unsigned(31881,16), to_unsigned(31972,16), to_unsigned(32058,16),
        to_unsigned(32138,16), to_unsigned(32214,16), to_unsigned(32286,16), to_unsigned(32352,16), to_unsigned(32413,16), to_unsigned(32470,16), to_unsigned(32522,16), to_unsigned(32568,16),
        to_unsigned(32610,16), to_unsigned(32647,16), to_unsigned(32679,16), to_unsigned(32706,16), to_unsigned(32729,16), to_unsigned(32746,16), to_unsigned(32758,16), to_unsigned(32766,16),
        to_unsigned(0,16), to_unsigned(356,16), to_unsigned(714,16), to_unsigned(1073,16), to_unsigned(1435,16), to_unsigned(1799,16), to_unsigned(2164,16), to_unsigned(2532,16),
        to_unsigned(2902,16), to_unsigned(3273,16), to_unsigned(3647,16), to_unsigned(4022,16), to_unsigned(4400,16), to_unsigned(4780,16), to_unsigned(5162,16), to_unsigned(5546,16),
        to_unsigned(5932,16), to_unsigned(6320,16), to_unsigned(6710,16), to_unsigned(7102,16), to_unsigned(7496,16), to_unsigned(7893,16), to_unsigned(8292,16), to_unsigned(8693,16),
        to_unsigned(9096,16), to_unsigned(9501,16), to_unsigned(9908,16), to_unsigned(10318,16), to_unsigned(10730,16), to_unsigned(11144,16), to_unsigned(11560,16), to_unsigned(11979,16),
        to_unsigned(12400,16), to_unsigned(12823,16), to_unsigned(13249,16), to_unsigned(13676,16), to_unsigned(14106,16), to_unsigned(14539,16), to_unsigned(14974,16), to_unsigned(15411,16),
        to_unsigned(15850,16), to_unsigned(16292,16), to_unsigned(16737,16), to_unsigned(17183,16), to_unsigned(17633,16), to_unsigned(18084,16), to_unsigned(18538,16), to_unsigned(18995,16),
        to_unsigned(19454,16), to_unsigned(19915,16), to_unsigned(20379,16), to_unsigned(20846,16), to_unsigned(21315,16), to_unsigned(21786,16), to_unsigned(22260,16), to_unsigned(22737,16),
        to_unsigned(23216,16), to_unsigned(23698,16), to_unsigned(24183,16), to_unsigned(24670,16), to_unsigned(25160,16), to_unsigned(25652,16), to_unsigned(26148,16), to_unsigned(26645,16),
        to_unsigned(27146,16), to_unsigned(27649,16), to_unsigned(28155,16), to_unsigned(28664,16), to_unsigned(29175,16), to_unsigned(29690,16), to_unsigned(30207,16), to_unsigned(30727,16),
        to_unsigned(31249,16), to_unsigned(31775,16), to_unsigned(32303,16), to_unsigned(32834,16), to_unsigned(33369,16), to_unsigned(33906,16), to_unsigned(34446,16), to_unsigned(34988,16),
        to_unsigned(35534,16), to_unsigned(36083,16), to_unsigned(36635,16), to_unsigned(37190,16), to_unsigned(37747,16), to_unsigned(38308,16), to_unsigned(38872,16), to_unsigned(39439,16),
        to_unsigned(40009,16), to_unsigned(40582,16), to_unsigned(41158,16), to_unsigned(41738,16), to_unsigned(42320,16), to_unsigned(42906,16), to_unsigned(43495,16), to_unsigned(44087,16),
        to_unsigned(44682,16), to_unsigned(45280,16), to_unsigned(45882,16), to_unsigned(46487,16), to_unsigned(47095,16), to_unsigned(47707,16), to_unsigned(48322,16), to_unsigned(48940,16),
        to_unsigned(49562,16), to_unsigned(50187,16), to_unsigned(50815,16), to_unsigned(51447,16), to_unsigned(52082,16), to_unsigned(52721,16), to_unsigned(53363,16), to_unsigned(54008,16),
        to_unsigned(54658,16), to_unsigned(55310,16), to_unsigned(55966,16), to_unsigned(56626,16), to_unsigned(57289,16), to_unsigned(57956,16), to_unsigned(58627,16), to_unsigned(59301,16),
        to_unsigned(59979,16), to_unsigned(60661,16), to_unsigned(61346,16), to_unsigned(62035,16), to_unsigned(62727,16), to_unsigned(63424,16), to_unsigned(64124,16), to_unsigned(64828,16),
        to_unsigned(15,16), to_unsigned(383,16), to_unsigned(751,16), to_unsigned(1119,16), to_unsigned(1486,16), to_unsigned(1839,16), to_unsigned(2206,16), to_unsigned(2559,16),
        to_unsigned(2926,16), to_unsigned(3278,16), to_unsigned(3631,16), to_unsigned(3998,16), to_unsigned(4350,16), to_unsigned(4702,16), to_unsigned(5053,16), to_unsigned(5390,16),
        to_unsigned(5742,16), to_unsigned(6094,16), to_unsigned(6445,16), to_unsigned(6782,16), to_unsigned(7133,16), to_unsigned(7469,16), to_unsigned(7805,16), to_unsigned(8142,16),
        to_unsigned(8493,16), to_unsigned(8829,16), to_unsigned(9165,16), to_unsigned(9500,16), to_unsigned(9821,16), to_unsigned(10157,16), to_unsigned(10492,16), to_unsigned(10813,16),
        to_unsigned(11148,16), to_unsigned(11469,16), to_unsigned(11804,16), to_unsigned(12125,16), to_unsigned(12460,16), to_unsigned(12780,16), to_unsigned(13100,16), to_unsigned(13420,16),
        to_unsigned(13740,16), to_unsigned(14060,16), to_unsigned(14380,16), to_unsigned(14699,16), to_unsigned(15004,16), to_unsigned(15324,16), to_unsigned(15643,16), to_unsigned(15948,16),
        to_unsigned(16267,16), to_unsigned(16571,16), to_unsigned(16876,16), to_unsigned(17195,16), to_unsigned(17499,16), to_unsigned(17803,16), to_unsigned(18107,16), to_unsigned(18411,16),
        to_unsigned(18715,16), to_unsigned(19019,16), to_unsigned(19323,16), to_unsigned(19626,16), to_unsigned(19915,16), to_unsigned(20219,16), to_unsigned(20522,16), to_unsigned(20811,16),
        to_unsigned(21114,16), to_unsigned(21402,16), to_unsigned(21691,16), to_unsigned(21994,16), to_unsigned(22282,16), to_unsigned(22570,16), to_unsigned(22858,16), to_unsigned(23147,16),
        to_unsigned(23450,16), to_unsigned(23737,16), to_unsigned(24010,16), to_unsigned(24298,16), to_unsigned(24586,16), to_unsigned(24874,16), to_unsigned(25161,16), to_unsigned(25434,16),
        to_unsigned(25721,16), to_unsigned(25994,16), to_unsigned(26281,16), to_unsigned(26554,16), to_unsigned(26841,16), to_unsigned(27114,16), to_unsigned(27401,16), to_unsigned(27673,16),
        to_unsigned(27945,16), to_unsigned(28217,16), to_unsigned(28489,16), to_unsigned(28761,16), to_unsigned(29033,16), to_unsigned(29305,16), to_unsigned(29577,16), to_unsigned(29849,16),
        to_unsigned(30121,16), to_unsigned(30392,16), to_unsigned(30649,16), to_unsigned(30921,16), to_unsigned(31192,16), to_unsigned(31449,16), to_unsigned(31720,16), to_unsigned(31977,16),
        to_unsigned(32248,16), to_unsigned(32504,16), to_unsigned(32761,16), to_unsigned(33032,16), to_unsigned(33288,16), to_unsigned(33544,16), to_unsigned(33800,16), to_unsigned(34057,16),
        to_unsigned(34328,16), to_unsigned(34584,16), to_unsigned(34839,16), to_unsigned(35080,16), to_unsigned(35336,16), to_unsigned(35592,16), to_unsigned(35848,16), to_unsigned(36104,16),
        to_unsigned(36359,16), to_unsigned(36600,16), to_unsigned(36856,16), to_unsigned(37111,16), to_unsigned(37352,16), to_unsigned(37607,16), to_unsigned(37848,16), to_unsigned(38103,16),
        to_unsigned(38343,16), to_unsigned(38584,16), to_unsigned(38839,16), to_unsigned(39079,16), to_unsigned(39319,16), to_unsigned(39560,16), to_unsigned(39815,16), to_unsigned(40055,16),
        to_unsigned(40295,16), to_unsigned(40535,16), to_unsigned(40775,16), to_unsigned(41015,16), to_unsigned(41255,16), to_unsigned(41495,16), to_unsigned(41734,16), to_unsigned(41959,16),
        to_unsigned(42199,16), to_unsigned(42439,16), to_unsigned(42678,16), to_unsigned(42903,16), to_unsigned(43143,16), to_unsigned(43382,16), to_unsigned(43607,16), to_unsigned(43846,16),
        to_unsigned(44071,16), to_unsigned(44310,16), to_unsigned(44535,16), to_unsigned(44774,16), to_unsigned(44998,16), to_unsigned(45223,16), to_unsigned(45462,16), to_unsigned(45686,16),
        to_unsigned(45910,16), to_unsigned(46134,16), to_unsigned(46358,16), to_unsigned(46583,16), to_unsigned(46822,16), to_unsigned(47046,16), to_unsigned(47270,16), to_unsigned(47494,16),
        to_unsigned(47717,16), to_unsigned(47926,16), to_unsigned(48150,16), to_unsigned(48374,16), to_unsigned(48598,16), to_unsigned(48822,16), to_unsigned(49045,16), to_unsigned(49254,16),
        to_unsigned(49478,16), to_unsigned(49701,16), to_unsigned(49910,16), to_unsigned(50133,16), to_unsigned(50342,16), to_unsigned(50566,16), to_unsigned(50789,16), to_unsigned(50997,16),
        to_unsigned(51206,16), to_unsigned(51429,16), to_unsigned(51638,16), to_unsigned(51861,16), to_unsigned(52069,16), to_unsigned(52277,16), to_unsigned(52486,16), to_unsigned(52709,16),
        to_unsigned(52917,16), to_unsigned(53125,16), to_unsigned(53333,16), to_unsigned(53541,16), to_unsigned(53750,16), to_unsigned(53973,16), to_unsigned(54181,16), to_unsigned(54389,16),
        to_unsigned(54596,16), to_unsigned(54789,16), to_unsigned(54997,16), to_unsigned(55205,16), to_unsigned(55413,16), to_unsigned(55621,16), to_unsigned(55829,16), to_unsigned(56036,16),
        to_unsigned(56229,16), to_unsigned(56437,16), to_unsigned(56644,16), to_unsigned(56837,16), to_unsigned(57045,16), to_unsigned(57252,16), to_unsigned(57445,16), to_unsigned(57652,16),
        to_unsigned(57845,16), to_unsigned(58052,16), to_unsigned(58245,16), to_unsigned(58452,16), to_unsigned(58645,16), to_unsigned(58852,16), to_unsigned(59044,16), to_unsigned(59237,16),
        to_unsigned(59444,16), to_unsigned(59636,16), to_unsigned(59828,16), to_unsigned(60021,16), to_unsigned(60228,16), to_unsigned(60420,16), to_unsigned(60612,16), to_unsigned(60804,16),
        to_unsigned(60996,16), to_unsigned(61188,16), to_unsigned(61381,16), to_unsigned(61588,16), to_unsigned(61780,16), to_unsigned(61972,16), to_unsigned(62163,16), to_unsigned(62340,16),
        to_unsigned(62532,16), to_unsigned(62724,16), to_unsigned(62916,16), to_unsigned(63108,16), to_unsigned(63300,16), to_unsigned(63491,16), to_unsigned(63668,16), to_unsigned(63860,16),
        to_unsigned(64052,16), to_unsigned(64243,16), to_unsigned(64420,16), to_unsigned(64612,16), to_unsigned(64803,16), to_unsigned(64980,16), to_unsigned(65171,16), to_unsigned(65348,16));
    type t_c_prog is array(0 to 1279) of unsigned(15 downto 0);
    constant C_PROG : t_c_prog := (
        to_unsigned(47104,16), to_unsigned(55296,16), to_unsigned(16390,16), to_unsigned(8192,16), to_unsigned(14339,16), to_unsigned(6144,16), to_unsigned(4096,16), to_unsigned(55297,16),
        to_unsigned(16390,16), to_unsigned(8193,16), to_unsigned(14339,16), to_unsigned(6145,16), to_unsigned(4097,16), to_unsigned(55298,16), to_unsigned(16390,16), to_unsigned(8194,16),
        to_unsigned(14339,16), to_unsigned(6146,16), to_unsigned(4098,16), to_unsigned(55301,16), to_unsigned(16390,16), to_unsigned(8197,16), to_unsigned(14339,16), to_unsigned(6149,16),
        to_unsigned(4101,16), to_unsigned(55302,16), to_unsigned(16390,16), to_unsigned(8198,16), to_unsigned(14339,16), to_unsigned(6150,16), to_unsigned(4102,16), to_unsigned(55299,16),
        to_unsigned(16390,16), to_unsigned(4099,16), to_unsigned(55300,16), to_unsigned(16390,16), to_unsigned(4100,16), to_unsigned(55306,16), to_unsigned(59467,16), to_unsigned(32810,16),
        to_unsigned(2130,16), to_unsigned(30763,16), to_unsigned(10240,16), to_unsigned(8199,16), to_unsigned(4108,16), to_unsigned(14080,16), to_unsigned(34866,16), to_unsigned(10496,16),
        to_unsigned(4108,16), to_unsigned(30775,16), to_unsigned(2060,16), to_unsigned(12544,16), to_unsigned(38967,16), to_unsigned(12032,16), to_unsigned(4108,16), to_unsigned(2060,16),
        to_unsigned(6151,16), to_unsigned(4103,16), to_unsigned(55306,16), to_unsigned(59466,16), to_unsigned(32869,16), to_unsigned(55306,16), to_unsigned(59468,16), to_unsigned(32835,16),
        to_unsigned(55306,16), to_unsigned(59469,16), to_unsigned(32869,16), to_unsigned(2056,16), to_unsigned(12316,16), to_unsigned(59471,16), to_unsigned(4104,16), to_unsigned(14309,16),
        to_unsigned(38992,16), to_unsigned(2166,16), to_unsigned(18558,16), to_unsigned(20480,16), to_unsigned(12289,16), to_unsigned(4214,16), to_unsigned(14354,16), to_unsigned(4220,16),
        to_unsigned(2167,16), to_unsigned(14335,16), to_unsigned(4215,16), to_unsigned(39013,16), to_unsigned(2166,16), to_unsigned(18558,16), to_unsigned(20480,16), to_unsigned(12289,16),
        to_unsigned(4214,16), to_unsigned(14355,16), to_unsigned(4216,16), to_unsigned(2166,16), to_unsigned(16397,16), to_unsigned(14355,16), to_unsigned(4217,16), to_unsigned(2166,16),
        to_unsigned(14341,16), to_unsigned(59470,16), to_unsigned(14337,16), to_unsigned(12328,16), to_unsigned(4215,16), to_unsigned(55306,16), to_unsigned(59466,16), to_unsigned(36972,16),
        to_unsigned(10240,16), to_unsigned(4216,16), to_unsigned(4217,16), to_unsigned(4220,16), to_unsigned(2168,16), to_unsigned(8314,16), to_unsigned(14337,16), to_unsigned(6266,16),
        to_unsigned(4218,16), to_unsigned(2169,16), to_unsigned(8315,16), to_unsigned(14337,16), to_unsigned(6267,16), to_unsigned(4219,16), to_unsigned(2172,16), to_unsigned(8317,16),
        to_unsigned(14341,16), to_unsigned(6269,16), to_unsigned(4221,16), to_unsigned(2051,16), to_unsigned(16392,16), to_unsigned(18498,16), to_unsigned(20496,16), to_unsigned(13311,16),
        to_unsigned(12801,16), to_unsigned(4107,16), to_unsigned(2057,16), to_unsigned(16392,16), to_unsigned(4108,16), to_unsigned(2059,16), to_unsigned(8204,16), to_unsigned(4109,16),
        to_unsigned(14144,16), to_unsigned(39054,16), to_unsigned(2061,16), to_unsigned(12480,16), to_unsigned(34958,16), to_unsigned(30866,16), to_unsigned(2059,16), to_unsigned(12416,16),
        to_unsigned(14344,16), to_unsigned(4105,16), to_unsigned(2052,16), to_unsigned(16392,16), to_unsigned(18499,16), to_unsigned(20496,16), to_unsigned(13056,16), to_unsigned(4107,16),
        to_unsigned(2058,16), to_unsigned(16392,16), to_unsigned(4108,16), to_unsigned(2059,16), to_unsigned(8204,16), to_unsigned(4109,16), to_unsigned(14144,16), to_unsigned(39076,16),
        to_unsigned(2061,16), to_unsigned(12480,16), to_unsigned(34980,16), to_unsigned(30888,16), to_unsigned(2059,16), to_unsigned(12416,16), to_unsigned(14344,16), to_unsigned(4106,16),
        to_unsigned(55305,16), to_unsigned(4213,16), to_unsigned(55304,16), to_unsigned(18549,16), to_unsigned(20480,16), to_unsigned(4112,16), to_unsigned(55306,16), to_unsigned(59468,16),
        to_unsigned(32948,16), to_unsigned(2064,16), to_unsigned(16385,16), to_unsigned(4112,16), to_unsigned(55303,16), to_unsigned(16387,16), to_unsigned(4114,16), to_unsigned(2064,16),
        to_unsigned(14337,16), to_unsigned(4115,16), to_unsigned(2064,16), to_unsigned(16392,16), to_unsigned(18500,16), to_unsigned(20496,16), to_unsigned(4116,16), to_unsigned(2066,16),
        to_unsigned(18450,16), to_unsigned(20480,16), to_unsigned(4108,16), to_unsigned(2067,16), to_unsigned(18451,16), to_unsigned(20480,16), to_unsigned(6156,16), to_unsigned(62561,16),
        to_unsigned(4117,16), to_unsigned(2069,16), to_unsigned(16396,16), to_unsigned(22548,16), to_unsigned(49152,16), to_unsigned(62509,16), to_unsigned(8279,16), to_unsigned(4108,16),
        to_unsigned(2060,16), to_unsigned(18439,16), to_unsigned(20480,16), to_unsigned(14348,16), to_unsigned(4108,16), to_unsigned(2060,16), to_unsigned(59471,16), to_unsigned(16388,16),
        to_unsigned(62599,16), to_unsigned(6237,16), to_unsigned(4109,16), to_unsigned(2068,16), to_unsigned(18445,16), to_unsigned(20496,16), to_unsigned(4118,16), to_unsigned(2060,16),
        to_unsigned(14348,16), to_unsigned(32997,16), to_unsigned(2070,16), to_unsigned(16385,16), to_unsigned(4118,16), to_unsigned(2050,16), to_unsigned(18434,16), to_unsigned(20496,16),
        to_unsigned(18503,16), to_unsigned(20496,16), to_unsigned(12550,16), to_unsigned(4119,16), to_unsigned(2070,16), to_unsigned(18455,16), to_unsigned(20496,16), to_unsigned(4120,16),
        to_unsigned(2070,16), to_unsigned(18502,16), to_unsigned(20496,16), to_unsigned(4121,16), to_unsigned(2072,16), to_unsigned(8217,16), to_unsigned(39161,16), to_unsigned(2073,16),
        to_unsigned(30970,16), to_unsigned(2072,16), to_unsigned(4109,16), to_unsigned(2070,16), to_unsigned(14342,16), to_unsigned(4108,16), to_unsigned(2070,16), to_unsigned(8205,16),
        to_unsigned(8204,16), to_unsigned(14320,16), to_unsigned(18518,16), to_unsigned(20496,16), to_unsigned(4122,16), to_unsigned(14320,16), to_unsigned(39177,16), to_unsigned(10256,16),
        to_unsigned(4122,16), to_unsigned(2072,16), to_unsigned(18557,16), to_unsigned(20496,16), to_unsigned(6168,16), to_unsigned(4120,16), to_unsigned(2048,16), to_unsigned(8273,16),
        to_unsigned(6266,16), to_unsigned(16385,16), to_unsigned(18458,16), to_unsigned(20496,16), to_unsigned(4123,16), to_unsigned(2049,16), to_unsigned(8273,16), to_unsigned(6267,16),
        to_unsigned(16385,16), to_unsigned(18458,16), to_unsigned(20496,16), to_unsigned(4124,16), to_unsigned(2075,16), to_unsigned(18459,16), to_unsigned(20480,16), to_unsigned(4108,16),
        to_unsigned(2076,16), to_unsigned(18460,16), to_unsigned(20480,16), to_unsigned(6156,16), to_unsigned(62561,16), to_unsigned(4125,16), to_unsigned(2077,16), to_unsigned(14335,16),
        to_unsigned(39214,16), to_unsigned(10241,16), to_unsigned(4125,16), to_unsigned(4123,16), to_unsigned(10240,16), to_unsigned(4124,16), to_unsigned(2074,16), to_unsigned(8221,16),
        to_unsigned(35122,16), to_unsigned(31041,16), to_unsigned(2074,16), to_unsigned(16400,16), to_unsigned(22557,16), to_unsigned(49152,16), to_unsigned(4108,16), to_unsigned(2075,16),
        to_unsigned(18444,16), to_unsigned(20496,16), to_unsigned(4123,16), to_unsigned(2076,16), to_unsigned(18444,16), to_unsigned(20496,16), to_unsigned(4124,16), to_unsigned(2074,16),
        to_unsigned(4125,16), to_unsigned(2075,16), to_unsigned(28672,16), to_unsigned(16400,16), to_unsigned(22557,16), to_unsigned(49152,16), to_unsigned(4128,16), to_unsigned(2075,16),
        to_unsigned(35146,16), to_unsigned(31053,16), to_unsigned(2080,16), to_unsigned(26624,16), to_unsigned(4128,16), to_unsigned(2076,16), to_unsigned(28672,16), to_unsigned(16400,16),
        to_unsigned(22557,16), to_unsigned(49152,16), to_unsigned(4129,16), to_unsigned(2076,16), to_unsigned(35158,16), to_unsigned(31065,16), to_unsigned(2081,16), to_unsigned(26624,16),
        to_unsigned(4129,16), to_unsigned(2077,16), to_unsigned(16400,16), to_unsigned(22550,16), to_unsigned(49152,16), to_unsigned(4134,16), to_unsigned(2072,16), to_unsigned(16400,16),
        to_unsigned(22550,16), to_unsigned(49152,16), to_unsigned(4135,16), to_unsigned(2086,16), to_unsigned(18470,16), to_unsigned(20496,16), to_unsigned(4109,16), to_unsigned(2087,16),
        to_unsigned(18471,16), to_unsigned(20496,16), to_unsigned(4110,16), to_unsigned(2141,16), to_unsigned(6157,16), to_unsigned(8206,16), to_unsigned(4130,16), to_unsigned(2082,16),
        to_unsigned(18466,16), to_unsigned(20496,16), to_unsigned(4108,16), to_unsigned(2061,16), to_unsigned(16386,16), to_unsigned(4110,16), to_unsigned(2060,16), to_unsigned(8206,16),
        to_unsigned(4108,16), to_unsigned(39292,16), to_unsigned(10241,16), to_unsigned(4108,16), to_unsigned(2060,16), to_unsigned(16396,16), to_unsigned(62561,16), to_unsigned(16386,16),
        to_unsigned(4110,16), to_unsigned(2082,16), to_unsigned(6158,16), to_unsigned(14338,16), to_unsigned(4110,16), to_unsigned(2086,16), to_unsigned(16399,16), to_unsigned(22542,16),
        to_unsigned(49152,16), to_unsigned(4133,16), to_unsigned(2085,16), to_unsigned(18464,16), to_unsigned(20496,16), to_unsigned(4131,16), to_unsigned(2085,16), to_unsigned(18465,16),
        to_unsigned(20496,16), to_unsigned(4132,16), to_unsigned(2086,16), to_unsigned(6214,16), to_unsigned(62492,16), to_unsigned(4136,16), to_unsigned(2088,16), to_unsigned(37274,16),
        to_unsigned(10241,16), to_unsigned(4136,16), to_unsigned(2088,16), to_unsigned(62509,16), to_unsigned(8285,16), to_unsigned(28672,16), to_unsigned(4141,16), to_unsigned(14272,16),
        to_unsigned(39331,16), to_unsigned(10304,16), to_unsigned(4141,16), to_unsigned(2093,16), to_unsigned(16388,16), to_unsigned(22538,16), to_unsigned(49152,16), to_unsigned(4142,16),
        to_unsigned(2058,16), to_unsigned(16408,16), to_unsigned(22573,16), to_unsigned(49152,16), to_unsigned(4205,16), to_unsigned(14354,16), to_unsigned(33203,16), to_unsigned(10241,16),
        to_unsigned(16402,16), to_unsigned(14335,16), to_unsigned(4205,16), to_unsigned(2157,16), to_unsigned(4143,16), to_unsigned(10240,16), to_unsigned(4111,16), to_unsigned(2095,16),
        to_unsigned(14347,16), to_unsigned(33217,16), to_unsigned(2095,16), to_unsigned(14337,16), to_unsigned(4143,16), to_unsigned(2063,16), to_unsigned(12289,16), to_unsigned(4111,16),
        to_unsigned(31159,16), to_unsigned(2095,16), to_unsigned(4204,16), to_unsigned(57357,16), to_unsigned(2063,16), to_unsigned(57358,16), to_unsigned(2156,16), to_unsigned(51215,16),
        to_unsigned(4205,16), to_unsigned(10241,16), to_unsigned(16407,16), to_unsigned(22550,16), to_unsigned(49152,16), to_unsigned(4137,16), to_unsigned(2165,16), to_unsigned(16400,16),
        to_unsigned(22550,16), to_unsigned(49152,16), to_unsigned(4138,16), to_unsigned(2066,16), to_unsigned(16400,16), to_unsigned(22550,16), to_unsigned(49152,16), to_unsigned(26624,16),
        to_unsigned(8227,16), to_unsigned(4108,16), to_unsigned(57344,16), to_unsigned(2067,16), to_unsigned(16400,16), to_unsigned(22550,16), to_unsigned(49152,16), to_unsigned(26624,16),
        to_unsigned(8228,16), to_unsigned(4109,16), to_unsigned(57345,16), to_unsigned(2089,16), to_unsigned(57348,16), to_unsigned(2090,16), to_unsigned(57349,16), to_unsigned(2060,16),
        to_unsigned(6179,16), to_unsigned(4139,16), to_unsigned(2061,16), to_unsigned(6180,16), to_unsigned(4140,16), to_unsigned(2083,16), to_unsigned(18475,16), to_unsigned(20496,16),
        to_unsigned(4110,16), to_unsigned(2084,16), to_unsigned(18476,16), to_unsigned(20496,16), to_unsigned(6158,16), to_unsigned(4110,16), to_unsigned(2141,16), to_unsigned(8206,16),
        to_unsigned(57346,16), to_unsigned(2084,16), to_unsigned(18475,16), to_unsigned(20496,16), to_unsigned(4110,16), to_unsigned(2083,16), to_unsigned(18476,16), to_unsigned(20496,16),
        to_unsigned(4108,16), to_unsigned(2062,16), to_unsigned(8204,16), to_unsigned(57347,16), to_unsigned(2083,16), to_unsigned(18473,16), to_unsigned(20496,16), to_unsigned(26624,16),
        to_unsigned(57350,16), to_unsigned(2084,16), to_unsigned(18473,16), to_unsigned(20496,16), to_unsigned(57351,16), to_unsigned(2084,16), to_unsigned(18474,16), to_unsigned(20496,16),
        to_unsigned(26624,16), to_unsigned(57352,16), to_unsigned(2083,16), to_unsigned(18474,16), to_unsigned(20496,16), to_unsigned(26624,16), to_unsigned(57353,16), to_unsigned(2094,16),
        to_unsigned(59475,16), to_unsigned(62599,16), to_unsigned(6237,16), to_unsigned(4150,16), to_unsigned(2094,16), to_unsigned(14352,16), to_unsigned(33321,16), to_unsigned(14335,16),
        to_unsigned(33317,16), to_unsigned(2102,16), to_unsigned(14337,16), to_unsigned(4150,16), to_unsigned(31276,16), to_unsigned(2102,16), to_unsigned(14338,16), to_unsigned(4150,16),
        to_unsigned(31276,16), to_unsigned(2102,16), to_unsigned(14339,16), to_unsigned(4150,16), to_unsigned(2102,16), to_unsigned(6224,16), to_unsigned(4108,16), to_unsigned(2102,16),
        to_unsigned(8272,16), to_unsigned(16400,16), to_unsigned(22540,16), to_unsigned(49152,16), to_unsigned(4151,16), to_unsigned(2149,16), to_unsigned(22537,16), to_unsigned(49152,16),
        to_unsigned(62627,16), to_unsigned(16385,16), to_unsigned(4152,16), to_unsigned(2104,16), to_unsigned(8247,16), to_unsigned(35392,16), to_unsigned(2103,16), to_unsigned(31297,16),
        to_unsigned(2104,16), to_unsigned(18505,16), to_unsigned(20496,16), to_unsigned(4153,16), to_unsigned(2105,16), to_unsigned(18489,16), to_unsigned(20496,16), to_unsigned(4154,16),
        to_unsigned(2105,16), to_unsigned(18489,16), to_unsigned(20480,16), to_unsigned(4184,16), to_unsigned(10240,16), to_unsigned(4185,16), to_unsigned(2136,16), to_unsigned(14352,16),
        to_unsigned(33368,16), to_unsigned(2136,16), to_unsigned(14337,16), to_unsigned(4184,16), to_unsigned(2137,16), to_unsigned(12289,16), to_unsigned(4185,16), to_unsigned(31310,16),
        to_unsigned(2152,16), to_unsigned(22616,16), to_unsigned(49152,16), to_unsigned(4186,16), to_unsigned(55306,16), to_unsigned(14340,16), to_unsigned(59466,16), to_unsigned(33380,16),
        to_unsigned(2138,16), to_unsigned(18518,16), to_unsigned(20496,16), to_unsigned(4186,16), to_unsigned(2105,16), to_unsigned(62509,16), to_unsigned(8285,16), to_unsigned(4155,16),
        to_unsigned(2058,16), to_unsigned(16396,16), to_unsigned(24591,16), to_unsigned(26624,16), to_unsigned(57354,16), to_unsigned(2072,16), to_unsigned(16400,16), to_unsigned(22550,16),
        to_unsigned(49152,16), to_unsigned(4108,16), to_unsigned(2086,16), to_unsigned(6156,16), to_unsigned(62492,16), to_unsigned(4109,16), to_unsigned(37497,16), to_unsigned(10241,16),
        to_unsigned(4109,16), to_unsigned(2061,16), to_unsigned(62509,16), to_unsigned(8285,16), to_unsigned(16388,16), to_unsigned(18541,16), to_unsigned(20496,16), to_unsigned(4108,16),
        to_unsigned(2058,16), to_unsigned(16396,16), to_unsigned(6156,16), to_unsigned(8200,16), to_unsigned(57356,16), to_unsigned(2085,16), to_unsigned(18469,16), to_unsigned(20496,16),
        to_unsigned(4108,16), to_unsigned(2141,16), to_unsigned(8204,16), to_unsigned(4108,16), to_unsigned(2060,16), to_unsigned(62509,16), to_unsigned(8285,16), to_unsigned(4108,16),
        to_unsigned(2070,16), to_unsigned(14340,16), to_unsigned(62509,16), to_unsigned(4109,16), to_unsigned(2060,16), to_unsigned(8205,16), to_unsigned(8251,16), to_unsigned(6240,16),
        to_unsigned(57355,16), to_unsigned(2054,16), to_unsigned(8284,16), to_unsigned(35486,16), to_unsigned(10240,16), to_unsigned(31391,16), to_unsigned(10241,16), to_unsigned(57370,16),
        to_unsigned(2054,16), to_unsigned(18438,16), to_unsigned(20496,16), to_unsigned(18516,16), to_unsigned(20496,16), to_unsigned(4108,16), to_unsigned(2054,16), to_unsigned(18517,16),
        to_unsigned(20496,16), to_unsigned(6156,16), to_unsigned(22538,16), to_unsigned(49152,16), to_unsigned(4206,16), to_unsigned(2053,16), to_unsigned(8273,16), to_unsigned(4156,16),
        to_unsigned(13312,16), to_unsigned(13313,16), to_unsigned(39611,16), to_unsigned(2108,16), to_unsigned(13311,16), to_unsigned(13311,16), to_unsigned(12289,16), to_unsigned(35517,16),
        to_unsigned(10240,16), to_unsigned(4156,16), to_unsigned(31422,16), to_unsigned(4156,16), to_unsigned(31422,16), to_unsigned(4156,16), to_unsigned(2108,16), to_unsigned(18441,16),
        to_unsigned(20480,16), to_unsigned(18478,16), to_unsigned(20496,16), to_unsigned(18504,16), to_unsigned(20496,16), to_unsigned(26624,16), to_unsigned(4157,16), to_unsigned(13312,16),
        to_unsigned(13312,16), to_unsigned(35534,16), to_unsigned(10241,16), to_unsigned(16394,16), to_unsigned(13311,16), to_unsigned(4157,16), to_unsigned(2109,16), to_unsigned(13311,16),
        to_unsigned(13311,16), to_unsigned(12289,16), to_unsigned(39640,16), to_unsigned(10241,16), to_unsigned(16394,16), to_unsigned(13311,16), to_unsigned(26624,16), to_unsigned(4157,16),
        to_unsigned(2109,16), to_unsigned(14338,16), to_unsigned(57360,16), to_unsigned(2109,16), to_unsigned(16388,16), to_unsigned(18440,16), to_unsigned(20496,16), to_unsigned(4110,16),
        to_unsigned(2109,16), to_unsigned(14338,16), to_unsigned(16386,16), to_unsigned(4161,16), to_unsigned(2113,16), to_unsigned(14337,16), to_unsigned(6158,16), to_unsigned(4149,16),
        to_unsigned(10240,16), to_unsigned(4159,16), to_unsigned(2101,16), to_unsigned(59471,16), to_unsigned(57369,16), to_unsigned(2111,16), to_unsigned(57368,16), to_unsigned(43014,16),
        to_unsigned(2101,16), to_unsigned(6209,16), to_unsigned(4149,16), to_unsigned(2111,16), to_unsigned(12289,16), to_unsigned(4159,16), to_unsigned(14208,16), to_unsigned(35562,16),
        to_unsigned(2113,16), to_unsigned(16391,16), to_unsigned(26624,16), to_unsigned(4149,16), to_unsigned(2113,16), to_unsigned(14337,16), to_unsigned(6197,16), to_unsigned(6158,16),
        to_unsigned(4149,16), to_unsigned(2101,16), to_unsigned(59471,16), to_unsigned(57369,16), to_unsigned(2111,16), to_unsigned(57368,16), to_unsigned(43014,16), to_unsigned(2101,16),
        to_unsigned(6209,16), to_unsigned(4149,16), to_unsigned(2111,16), to_unsigned(12289,16), to_unsigned(4159,16), to_unsigned(14080,16), to_unsigned(35585,16), to_unsigned(2056,16),
        to_unsigned(57361,16), to_unsigned(2056,16), to_unsigned(14340,16), to_unsigned(57362,16), to_unsigned(2058,16), to_unsigned(57359,16), to_unsigned(10240,16), to_unsigned(4159,16),
        to_unsigned(4211,16), to_unsigned(2111,16), to_unsigned(12545,16), to_unsigned(4144,16), to_unsigned(13824,16), to_unsigned(33574,16), to_unsigned(2096,16), to_unsigned(45056,16),
        to_unsigned(53248,16), to_unsigned(14340,16), to_unsigned(18540,16), to_unsigned(20480,16), to_unsigned(14348,16), to_unsigned(31527,16), to_unsigned(2156,16), to_unsigned(4209,16),
        to_unsigned(8307,16), to_unsigned(4210,16), to_unsigned(14305,16), to_unsigned(35630,16), to_unsigned(10271,16), to_unsigned(4210,16), to_unsigned(2163,16), to_unsigned(16389,16),
        to_unsigned(6258,16), to_unsigned(57369,16), to_unsigned(2111,16), to_unsigned(57368,16), to_unsigned(43011,16), to_unsigned(2161,16), to_unsigned(4211,16), to_unsigned(2111,16),
        to_unsigned(12289,16), to_unsigned(4159,16), to_unsigned(14080,16), to_unsigned(35609,16), to_unsigned(2057,16), to_unsigned(57363,16), to_unsigned(2057,16), to_unsigned(16389,16),
        to_unsigned(4161,16), to_unsigned(10240,16), to_unsigned(4149,16), to_unsigned(4159,16), to_unsigned(2101,16), to_unsigned(59475,16), to_unsigned(57369,16), to_unsigned(2111,16),
        to_unsigned(57368,16), to_unsigned(43012,16), to_unsigned(2101,16), to_unsigned(6209,16), to_unsigned(4149,16), to_unsigned(2111,16), to_unsigned(12289,16), to_unsigned(4159,16),
        to_unsigned(14208,16), to_unsigned(35652,16), to_unsigned(10240,16), to_unsigned(4149,16), to_unsigned(2101,16), to_unsigned(14338,16), to_unsigned(57369,16), to_unsigned(2111,16),
        to_unsigned(57368,16), to_unsigned(43012,16), to_unsigned(2101,16), to_unsigned(6153,16), to_unsigned(4149,16), to_unsigned(2111,16), to_unsigned(12289,16), to_unsigned(4159,16),
        to_unsigned(14080,16), to_unsigned(35668,16), to_unsigned(10240,16), to_unsigned(4149,16), to_unsigned(4159,16), to_unsigned(2101,16), to_unsigned(14337,16), to_unsigned(4108,16),
        to_unsigned(13312,16), to_unsigned(13995,16), to_unsigned(35695,16), to_unsigned(10241,16), to_unsigned(16394,16), to_unsigned(12629,16), to_unsigned(4108,16), to_unsigned(2060,16),
        to_unsigned(57369,16), to_unsigned(2111,16), to_unsigned(57368,16), to_unsigned(43013,16), to_unsigned(2101,16), to_unsigned(6254,16), to_unsigned(4149,16), to_unsigned(2111,16),
        to_unsigned(12289,16), to_unsigned(4159,16), to_unsigned(14080,16), to_unsigned(35685,16), to_unsigned(10240,16), to_unsigned(4159,16), to_unsigned(2094,16), to_unsigned(16391,16),
        to_unsigned(26624,16), to_unsigned(4158,16), to_unsigned(2094,16), to_unsigned(4161,16), to_unsigned(10240,16), to_unsigned(4113,16), to_unsigned(2137,16), to_unsigned(14332,16),
        to_unsigned(39820,16), to_unsigned(26624,16), to_unsigned(4113,16), to_unsigned(10240,16), to_unsigned(4111,16), to_unsigned(2110,16), to_unsigned(14344,16), to_unsigned(59475,16),
        to_unsigned(62599,16), to_unsigned(4110,16), to_unsigned(2110,16), to_unsigned(14360,16), to_unsigned(33703,16), to_unsigned(14335,16), to_unsigned(33694,16), to_unsigned(12290,16),
        to_unsigned(33699,16), to_unsigned(2062,16), to_unsigned(8286,16), to_unsigned(14338,16), to_unsigned(4110,16), to_unsigned(31655,16), to_unsigned(2062,16), to_unsigned(16385,16),
        to_unsigned(6237,16), to_unsigned(4110,16), to_unsigned(31655,16), to_unsigned(2062,16), to_unsigned(8285,16), to_unsigned(14337,16), to_unsigned(4110,16), to_unsigned(2062,16),
        to_unsigned(16392,16), to_unsigned(18446,16), to_unsigned(20496,16), to_unsigned(4109,16), to_unsigned(14362,16), to_unsigned(33712,16), to_unsigned(2151,16), to_unsigned(4109,16),
        to_unsigned(2062,16), to_unsigned(6237,16), to_unsigned(14338,16), to_unsigned(4110,16), to_unsigned(2061,16), to_unsigned(51217,16), to_unsigned(18522,16), to_unsigned(20496,16),
        to_unsigned(24591,16), to_unsigned(4109,16), to_unsigned(2147,16), to_unsigned(8205,16), to_unsigned(4108,16), to_unsigned(28672,16), to_unsigned(14353,16), to_unsigned(33731,16),
        to_unsigned(2148,16), to_unsigned(4109,16), to_unsigned(31686,16), to_unsigned(2060,16), to_unsigned(28672,16), to_unsigned(4109,16), to_unsigned(2061,16), to_unsigned(16398,16),
        to_unsigned(22542,16), to_unsigned(49152,16), to_unsigned(4109,16), to_unsigned(2060,16), to_unsigned(35791,16), to_unsigned(2061,16), to_unsigned(31697,16), to_unsigned(2061,16),
        to_unsigned(26624,16), to_unsigned(4109,16), to_unsigned(8273,16), to_unsigned(35798,16), to_unsigned(2145,16), to_unsigned(4109,16), to_unsigned(2061,16), to_unsigned(6225,16),
        to_unsigned(39899,16), to_unsigned(2146,16), to_unsigned(4109,16), to_unsigned(2111,16), to_unsigned(57368,16), to_unsigned(2061,16), to_unsigned(57369,16), to_unsigned(43009,16),
        to_unsigned(2110,16), to_unsigned(6209,16), to_unsigned(4158,16), to_unsigned(2111,16), to_unsigned(12289,16), to_unsigned(4159,16), to_unsigned(14080,16), to_unsigned(35725,16),
        to_unsigned(2150,16), to_unsigned(22537,16), to_unsigned(49152,16), to_unsigned(4161,16), to_unsigned(10240,16), to_unsigned(4149,16), to_unsigned(4159,16), to_unsigned(2101,16),
        to_unsigned(14346,16), to_unsigned(62627,16), to_unsigned(4108,16), to_unsigned(18444,16), to_unsigned(20480,16), to_unsigned(18522,16), to_unsigned(20496,16), to_unsigned(24665,16),
        to_unsigned(4109,16), to_unsigned(8272,16), to_unsigned(35839,16), to_unsigned(10241,16), to_unsigned(16397,16), to_unsigned(14335,16), to_unsigned(4109,16), to_unsigned(2111,16),
        to_unsigned(33809,16), to_unsigned(2061,16), to_unsigned(8256,16), to_unsigned(14339,16), to_unsigned(4108,16), to_unsigned(14329,16), to_unsigned(35849,16), to_unsigned(10247,16),
        to_unsigned(4108,16), to_unsigned(2111,16), to_unsigned(14335,16), to_unsigned(57368,16), to_unsigned(2112,16), to_unsigned(16387,16), to_unsigned(6156,16), to_unsigned(57369,16),
        to_unsigned(43010,16), to_unsigned(2061,16), to_unsigned(4160,16), to_unsigned(2101,16), to_unsigned(6209,16), to_unsigned(4149,16), to_unsigned(2111,16), to_unsigned(12289,16),
        to_unsigned(4159,16), to_unsigned(14079,16), to_unsigned(35823,16), to_unsigned(30720,16), to_unsigned(4108,16), to_unsigned(8229,16), to_unsigned(28672,16), to_unsigned(4109,16),
        to_unsigned(2085,16), to_unsigned(18444,16), to_unsigned(20496,16), to_unsigned(4110,16), to_unsigned(2141,16), to_unsigned(8206,16), to_unsigned(14337,16), to_unsigned(4110,16),
        to_unsigned(2061,16), to_unsigned(16399,16), to_unsigned(22542,16), to_unsigned(49152,16), to_unsigned(63488,16), to_unsigned(4207,16), to_unsigned(37937,16), to_unsigned(10241,16),
        to_unsigned(4207,16), to_unsigned(10257,16), to_unsigned(4208,16), to_unsigned(2159,16), to_unsigned(14353,16), to_unsigned(37949,16), to_unsigned(2159,16), to_unsigned(16385,16),
        to_unsigned(4207,16), to_unsigned(2160,16), to_unsigned(14335,16), to_unsigned(4208,16), to_unsigned(31795,16), to_unsigned(2159,16), to_unsigned(14345,16), to_unsigned(59470,16),
        to_unsigned(12544,16), to_unsigned(4144,16), to_unsigned(45056,16), to_unsigned(53248,16), to_unsigned(14340,16), to_unsigned(4145,16), to_unsigned(2096,16), to_unsigned(12289,16),
        to_unsigned(4144,16), to_unsigned(13824,16), to_unsigned(33873,16), to_unsigned(2096,16), to_unsigned(45056,16), to_unsigned(53248,16), to_unsigned(14340,16), to_unsigned(4146,16),
        to_unsigned(31827,16), to_unsigned(2130,16), to_unsigned(4146,16), to_unsigned(2159,16), to_unsigned(59483,16), to_unsigned(4147,16), to_unsigned(2098,16), to_unsigned(8241,16),
        to_unsigned(16391,16), to_unsigned(18483,16), to_unsigned(20496,16), to_unsigned(6193,16), to_unsigned(4145,16), to_unsigned(2160,16), to_unsigned(16396,16), to_unsigned(6193,16),
        to_unsigned(63488,16), to_unsigned(4201,16), to_unsigned(4110,16), to_unsigned(10241,16), to_unsigned(4202,16), to_unsigned(2153,16), to_unsigned(14338,16), to_unsigned(4201,16),
        to_unsigned(33901,16), to_unsigned(2154,16), to_unsigned(16385,16), to_unsigned(4202,16), to_unsigned(31845,16), to_unsigned(2154,16), to_unsigned(16385,16), to_unsigned(8275,16),
        to_unsigned(35956,16), to_unsigned(2131,16), to_unsigned(4202,16), to_unsigned(31863,16), to_unsigned(2154,16), to_unsigned(16385,16), to_unsigned(4202,16), to_unsigned(2062,16),
        to_unsigned(4201,16), to_unsigned(10245,16), to_unsigned(4203,16), to_unsigned(2153,16), to_unsigned(22634,16), to_unsigned(49152,16), to_unsigned(6250,16), to_unsigned(14337,16),
        to_unsigned(4202,16), to_unsigned(2155,16), to_unsigned(14335,16), to_unsigned(4203,16), to_unsigned(38011,16), to_unsigned(2154,16), to_unsigned(63488,16), to_unsigned(4148,16),
        to_unsigned(14345,16), to_unsigned(12416,16), to_unsigned(4144,16), to_unsigned(45056,16), to_unsigned(53248,16), to_unsigned(4145,16), to_unsigned(2096,16), to_unsigned(14081,16),
        to_unsigned(33943,16), to_unsigned(2096,16), to_unsigned(12289,16), to_unsigned(45056,16), to_unsigned(53248,16), to_unsigned(4146,16), to_unsigned(31897,16), to_unsigned(2141,16),
        to_unsigned(4146,16), to_unsigned(2100,16), to_unsigned(59483,16), to_unsigned(4147,16), to_unsigned(2098,16), to_unsigned(8241,16), to_unsigned(16391,16), to_unsigned(18483,16),
        to_unsigned(20496,16), to_unsigned(6193,16), to_unsigned(63488,16), to_unsigned(4148,16), to_unsigned(14347,16), to_unsigned(4144,16), to_unsigned(45056,16), to_unsigned(53248,16),
        to_unsigned(4145,16), to_unsigned(2096,16), to_unsigned(12289,16), to_unsigned(45056,16), to_unsigned(53248,16), to_unsigned(4146,16), to_unsigned(2100,16), to_unsigned(59484,16),
        to_unsigned(4147,16), to_unsigned(2098,16), to_unsigned(8241,16), to_unsigned(16389,16), to_unsigned(18483,16), to_unsigned(20496,16), to_unsigned(6193,16), to_unsigned(63488,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16),
        to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16), to_unsigned(0,16));
    type t_c_rfinit is array(0 to 127) of integer;
    constant C_RFINIT : t_c_rfinit := (19648, 36032, 20288, 30592, 39296, 45824, 20800, 0, 0, 48, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 90, 45, 120, 19899, 3932, 39322, 2400, 60293, 1, 2, 4, 8, 255, 4095, 8192, 32768, 4096, 65535, 1017, 349, 42598, 49152, 0, 0, 0, 511, 2047, 65536, 196608, 94560, 171783, 32767, -32768, 2048, 131072, 524288, 1048576, 67108864, -2147483648, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 625341585, 60, 0, 0, 0, 0, 0, 0, 69069, 0);


    ----------------------------------------------------------------------
    -- EBR tables
    ----------------------------------------------------------------------
    type t_ram16 is array(0 to 255) of std_logic_vector(15 downto 0);
    type t_ram4  is array(0 to 255) of std_logic_vector(3 downto 0);
    type t_ram32 is array(0 to 127) of std_logic_vector(31 downto 0);
    type t_ram512 is array(0 to 511) of std_logic_vector(15 downto 0);
    type t_prog is array(0 to 1279) of std_logic_vector(15 downto 0);

    function f_init_fun return t_ram512 is
        variable r : t_ram512;
    begin
        for i in 0 to 511 loop r(i) := std_logic_vector(C_FUN(i)); end loop;
        return r;
    end function;
    function f_init_prog return t_prog is
        variable r : t_prog;
    begin
        for i in 0 to 1279 loop r(i) := std_logic_vector(C_PROG(i)); end loop;
        return r;
    end function;
    function f_init_rf return t_ram32 is
        variable r : t_ram32;
    begin
        for i in 0 to 127 loop r(i) := std_logic_vector(to_signed(C_RFINIT(i), 32)); end loop;
        return r;
    end function;

    signal t1_ram   : t_ram16 := (others => (others => '0'));             -- B/K2 (Q11 signed), 256 entries
    signal tk_ram   : t_ram16 := (others => (others => '0'));             -- twist term per ring: 2T(k+1/2)+T*ph (Q12 arm)
    signal t2_ram   : t_ram16 := (others => (others => '0'));             -- S/K2 (13b Q11) & slope(3)
    signal tm_ram   : t_ram16 := (others => (others => '0'));             -- scaled log mantissa: value(11) & slope(5)
    signal tv_ram   : t_ram16 := (others => (others => '0'));             -- theta*A: 0..127 hi part, 128..255 lo part
    signal ts_ram   : t_ram16 := (others => (others => '0'));             -- layer split vs ring
    signal sb0, sb1, sb2, sb3 : t_ram16 := (others => (others => '0'));   -- line sample buffer (64-bit words)
    signal sd_ram   : t_ram4 := (others => (others => '0'));                -- sync delay
    signal rom_fun  : t_ram512 := f_init_fun;                               -- sin / exp2 / log2 mantissa
    signal rf_mem   : t_ram32 := f_init_rf;
    signal prog_mem : t_prog := f_init_prog;

    attribute ram_style : string;
    attribute ram_style of t1_ram : signal is "block";
    attribute ram_style of tk_ram : signal is "block";
    attribute ram_style of t2_ram : signal is "block";
    attribute ram_style of tm_ram : signal is "block";
    attribute ram_style of tv_ram : signal is "block";
    attribute ram_style of ts_ram : signal is "block";
    attribute ram_style of sb0 : signal is "block";
    attribute ram_style of sb1 : signal is "block";
    attribute ram_style of sb2 : signal is "block";
    attribute ram_style of sb3 : signal is "block";
    attribute ram_style of sd_ram : signal is "block";
    attribute ram_style of rom_fun : signal is "block";
    attribute ram_style of rf_mem : signal is "block";
    attribute ram_style of prog_mem : signal is "block";

    ----------------------------------------------------------------------
    -- raster measurement / timing
    ----------------------------------------------------------------------
    signal av_r, av_q, vs_r, vs_q, fl_r, hs_r, hs_q : std_logic := '0';
    signal xcnt   : unsigned(11 downto 0) := (others => '0');
    signal act_w  : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal aline  : unsigned(11 downto 0) := (others => '0');
    signal act_h  : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal frame_act : std_logic := '0';
    signal ilace  : std_logic := '0';
    signal anc_fld : std_logic := '0';
    signal vstep_base : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal blank_cnt : unsigned(9 downto 0) := (others => '0');
    signal hbl_len   : unsigned(9 downto 0) := to_unsigned(100, 10);
    signal hcnt      : unsigned(11 downto 0) := (others => '0');
    signal hline     : unsigned(11 downto 0) := to_unsigned(800, 12);
    signal nb        : unsigned(1 downto 0) := (others => '0');
    signal in_vblank : std_logic := '0';
    signal in_vblank_q : std_logic := '0';
    signal blank_ok  : std_logic := '0';
    signal seq_req   : std_logic := '0';
    signal dcnt      : unsigned(7 downto 0) := (others => '0');
    signal sw_ink, sw_flow, sw_full : std_logic := '0';
    signal sw_mesh, sw_glow : std_logic := '0';                    -- S7 Dots/Mesh, S8 Hard/Glow
    -- local copies of the control registers: the SPI-decoded words come from the far
    -- side of the die, and the sequencer's operand mux should not reach that far
    type t_regs is array(0 to 7) of unsigned(9 downto 0);
    signal reg_q  : t_regs := (others => (others => '0'));
    signal ilace_q, fld_q : std_logic := '0';

    ----------------------------------------------------------------------
    -- per-frame constants written by the sequencer
    ----------------------------------------------------------------------
    -- The iris is FIXED and CENTRED, so the screen is normalised to make it the unit
    -- circle and the map is the disc automorphism w = (z-a)/(1 - conj(a) z).  Both lanes
    -- are then affine in the pixel coordinates and bounded, so each is a plain Q16
    -- accumulator with constant per-sample and per-line steps.
    signal c_xa0, c_ya0, c_xb0, c_yb0 : signed(19 downto 0) := (others => '0');   -- line-0 starts
    signal c_sax, c_say : signed(19 downto 0) := (others => '0');                 -- lane A: per sample, per line
    signal c_sbx, c_sby : signed(19 downto 0) := (others => '0');                 -- lane B: per sample
    signal c_sblx, c_sbly : signed(19 downto 0) := (others => '0');               -- lane B: per line
    signal c_lp     : signed(17 downto 0) := (others => '0');   -- lattice anchor (reference pupil), normalised units
    signal c_cdot   : signed(19 downto 0) := (others => '0');   -- AA normaliser (needs 19 bits: ~151k)
    signal c_up     : signed(19 downto 0) := (others => '0');   -- pupil edge in u (Q12)
    signal c_invn   : unsigned(10 downto 0) := to_unsigned(1024, 11);  -- rings per octave, normalised
    signal c_inve   : unsigned(2 downto 0) := to_unsigned(2, 3);        -- ... shift
    signal c_n      : signed(7 downto 0) := to_signed(30, 8);
    signal c_nm1    : signed(7 downto 0) := to_signed(29, 8);
    signal c_t      : signed(10 downto 0) := (others => '0');    -- twist, Q10 arm/ring
    signal c_ph     : unsigned(11 downto 0) := (others => '0');
    signal c_fade   : unsigned(7 downto 0) := (others => '0');   -- outermost ring's level (flow phase)
    signal c_fl     : unsigned(3 downto 0) := (others => '0');   -- per-frame flags: bit 0 = split is zero
    signal c_a      : unsigned(3 downto 0) := (others => '0');   -- arms mod 16 (the arm coordinate's jump at the angle cut)

    ----------------------------------------------------------------------
    -- geometry engine
    ----------------------------------------------------------------------
    signal eng_busy   : std_logic := '0';
    signal eng_oh     : std_logic_vector(7 downto 0) := (0 => '1', others => '0');  -- one-hot phase
    signal eng_k      : unsigned(8 downto 0) := (others => '0');  -- sample being loaded
    signal eng_nsamp  : unsigned(8 downto 0) := to_unsigned(91, 9);
    signal eng_last   : unsigned(8 downto 0) := to_unsigned(93, 9);   -- nsamp + 2: last period of a run
    signal eng_kl     : std_logic := '0';
    signal pend_line, pend_row0 : std_logic := '0';
    signal st_go      : std_logic := '0';                            -- run start, registered
    signal st_kind    : unsigned(1 downto 0) := (others => '0');     -- 0 line, 2 row 0
    signal nsamp_r, last_r : unsigned(8 downto 0) := to_unsigned(91, 9);
    signal row0_req   : std_logic := '0';                          -- from the CPU (LINE0)
    signal sxa, sya   : signed(19 downto 0) := (others => '0');   -- lane A position (z - a)
    signal sxb, syb   : signed(19 downto 0) := (others => '0');   -- lane B position (1 - conj(a) z)
    signal lxb, lyb   : signed(19 downto 0) := (others => '0');   -- ... its line start
    -- serial CORDIC
    -- 3-stage CORDIC pipeline: stage s holds one lane and runs iterations 4s..4s+3
    signal c0x, c0y, c1x, c1y, c2x, c2y : signed(C_CW - 1 downto 0) := (others => '0');
    signal c0z, c1z, c2z : signed(15 downto 0) := (others => '0');
    signal c0f, c1f, c2f : std_logic_vector(2 downto 0) := (others => '0');   -- sw & sx & sy
    -- one-hot iteration index, one KEPT copy per CORDIC stage: the select needs no
    -- decode and the placer can sit each copy beside its own stage
    signal cj0, cj1, cj2 : std_logic_vector(3 downto 0) := "0001";
    signal eng_oh2    : std_logic_vector(7 downto 0) := (0 => '1', others => '0');  -- phase copy for the sample buffer
    attribute keep : boolean;
    attribute keep of cj0 : signal is true;
    attribute keep of cj1 : signal is true;
    attribute keep of cj2 : signal is true;
    attribute keep of eng_oh2 : signal is true;
    signal ld_hi, ld_lo : unsigned(14 downto 0) := (others => '0'); -- prepared CORDIC load operands
    signal la_x, la_y : unsigned(14 downto 0) := (others => '0');   -- ... after abs/saturate, before the swap
    signal la_s       : std_logic_vector(1 downto 0) := (others => '0');
    signal ld_f       : std_logic_vector(2 downto 0) := (others => '0');
    signal ra_x, rb_x : unsigned(C_CW - 1 downto 0) := (others => '0');
    signal ra_z, rb_z : signed(15 downto 0) := (others => '0');
    signal ra_sw, ra_sx, ra_sy, rb_sw, rb_sx, rb_sy : std_logic := '0';
    -- angle fold results
    signal fa, fb     : unsigned(16 downto 0) := (others => '0');
    signal th         : unsigned(16 downto 0) := (others => '0');
    -- log lane (shared, phase-scheduled): normalise, table, interpolate, exponent product
    signal lg_in      : unsigned(C_CW - 1 downto 0) := (others => '0');
    signal lg_e0      : unsigned(4 downto 0) := (others => '0');  -- exponent (msb index)
    signal lg_n1      : unsigned(C_CW - 1 downto 0) := (others => '0');
    signal lg_f1      : unsigned(1 downto 0) := (others => '0');
    signal lg_e1      : unsigned(4 downto 0) := (others => '0');
    signal lg_hi2     : unsigned(7 downto 0) := (others => '0');
    signal lg_lo2     : unsigned(3 downto 0) := (others => '0');
    signal lg_e2      : unsigned(4 downto 0) := (others => '0');
    signal lg_w3      : std_logic_vector(15 downto 0) := (others => '0');
    signal lg_lo3     : unsigned(3 downto 0) := (others => '0');
    signal lg_e3      : unsigned(4 downto 0) := (others => '0');
    signal lg_v4      : unsigned(10 downto 0) := (others => '0');
    signal lg_q4      : unsigned(4 downto 0) := (others => '0');
    signal lg_lo4     : unsigned(3 downto 0) := (others => '0');
    signal lg_e4      : unsigned(4 downto 0) := (others => '0');
    signal lg_p5      : unsigned(9 downto 0) := (others => '0');
    signal lg_v5      : unsigned(10 downto 0) := (others => '0');
    signal lg_ep5     : unsigned(15 downto 0) := (others => '0');
    signal lg_out     : unsigned(16 downto 0) := (others => '0');
    signal lg_a, lg_b : unsigned(16 downto 0) := (others => '0');
    signal ea, eb     : unsigned(4 downto 0) := (others => '0');   -- lane exponents / mantissa tops
    signal ha, hb     : unsigned(7 downto 0) := (others => '0');
    -- tail
    signal e_l        : signed(17 downto 0) := (others => '0');
    signal e_ld       : signed(18 downto 0) := (others => '0');
    signal e_gs       : unsigned(17 downto 0) := (others => '0');
    signal e_rd, e_rd2 : unsigned(3 downto 0) := (others => '0');
    signal e_at, e_at2 : unsigned(5 downto 0) := (others => '0');   -- resolution fade, same pipeline as e_rd
    signal e_u        : signed(19 downto 0) := (others => '0');
    signal e_vh       : unsigned(15 downto 0) := (others => '0');
    signal e_vl       : unsigned(15 downto 0) := (others => '0');
    signal e_v        : unsigned(15 downto 0) := (others => '0');   -- arm coordinate, 4 int + 12 frac
    signal vcut       : unsigned(3 downto 0) := (others => '0');    -- arms to add so v stays continuous across the cut
    signal thq_p      : unsigned(1 downto 0) := (others => '0');    -- previous sample's angle quadrant
    signal e_sep      : signed(13 downto 0) := (others => '0');
    signal tv_ad      : unsigned(7 downto 0) := (others => '0');
    signal tv_w       : std_logic_vector(15 downto 0) := (others => '0');
    signal ts_ad      : unsigned(7 downto 0) := (others => '0');
    signal ts_w       : std_logic_vector(15 downto 0) := (others => '0');
    signal stx_val : signed(20 downto 0) := (others => '0');
    signal stx_sel : unsigned(4 downto 0) := (others => '0');
    signal stx_en  : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel path (true stages)
    ----------------------------------------------------------------------
    signal px      : unsigned(10 downto 0) := (others => '1');  -- pixel index + 1 (true 1)
    signal slot    : unsigned(1 downto 0) := (others => '0');   -- layer slot 0 R, 1 G, 2 B
    signal sq0, sq1, sq2, sq3 : std_logic_vector(15 downto 0) := (others => '0');
    signal cur_u, nxt_u : signed(19 downto 0) := (others => '0');
    signal cur_v, nxt_v : signed(15 downto 0) := (others => '0');
    signal nxt_s   : signed(13 downto 0) := (others => '0');
    -- (all modular: only u's 20 bits and v's low 12 ever reach the dot test, so
    -- the accumulators carry exactly those bits plus the three of DDA fraction)
    signal du8     : signed(19 downto 0) := (others => '0');
    signal dv8     : signed(14 downto 0) := (others => '0');
    signal acc_u   : signed(22 downto 0) := (others => '0');
    signal acc_v   : signed(14 downto 0) := (others => '0');
    signal sep6    : signed(13 downto 0) := (others => '0');
    signal rd_h1   : unsigned(3 downto 0) := (others => '0');
    -- P7
    signal ul7     : signed(19 downto 0) := (others => '0');
    signal slot7   : unsigned(1 downto 0) := (others => '0');
    signal av7     : std_logic := '0';                          -- the pixel is inside active video
    signal v7      : unsigned(11 downto 0) := (others => '0');
    -- P8
    signal ug8     : signed(19 downto 0) := (others => '0');
    signal vg8     : unsigned(11 downto 0) := (others => '0');
    signal tk_q    : std_logic_vector(15 downto 0) := (others => '0');
    -- P9
    signal kk9     : signed(7 downto 0) := (others => '0');
    signal fg9     : unsigned(11 downto 0) := (others => '0');
    signal tk9     : signed(13 downto 0) := (others => '0');
    signal vg9     : unsigned(11 downto 0) := (others => '0');
    -- P9/P10/P11: dot attenuation.  The pupil is no longer a mask over the finished
    -- picture but a ramp on the dots' own coverage, so it darkens ring by ring from the
    -- centre out -- and, because it lands before the ink/light composite, it goes WHITE
    -- in Ink just as it goes black in Light.  The outermost ring's share of the same
    -- ramp fades the ring out across its own width as Flow pushes it through the rim.
    signal pu9     : unsigned(7 downto 0) := (others => '0');
    signal pu10    : unsigned(7 downto 0) := (others => '0');
    signal att11   : unsigned(7 downto 0) := (others => '0');
    -- how far past the sampler's reach this span's geometry is: the engine samples every
    -- 8 px, so ring structure finer than that cannot be interpolated, only guessed at
    signal res6    : unsigned(7 downto 0) := (others => '0');
    -- P10
    signal ig10    : unsigned(7 downto 0) := (others => '0');
    signal jg10    : unsigned(7 downto 0) := (others => '0');
    signal lvg10   : unsigned(2 downto 0) := (others => '0');
    signal vldg10, fdg10 : std_logic := '0';
    -- P11 EBR outs
    signal t1g11, t2g11 : std_logic_vector(15 downto 0) := (others => '0');
    signal lvg11   : unsigned(2 downto 0) := (others => '0');
    -- P12
    signal bvg12   : signed(15 downto 0) := (others => '0');
    signal svg12   : unsigned(12 downto 0) := (others => '0');
    signal ssg12   : unsigned(2 downto 0) := (others => '0');
    signal lvg12   : unsigned(2 downto 0) := (others => '0');
    -- P13
    signal dfg13   : signed(16 downto 0) := (others => '0');
    -- P14
    signal bar14_g : signed(8 downto 0) := (others => '0');
    -- P15 (held per layer)
    signal al15_r, al15_g, al15_b : unsigned(7 downto 0) := (others => '0');   -- latest evaluation of each layer
    signal pv_r, pv_g, pv_b : unsigned(7 downto 0) := (others => '0');         -- ...and the one before it
    signal fr_r, fr_b : std_logic := '0';                                       -- first evaluation of the line pending
    -- P16
    signal cr16, cg16, cb16 : unsigned(7 downto 0) := (others => '0');
    -- P17 / P18
    signal yy17 : unsigned(11 downto 0) := (others => '0');
    signal r17, b17 : unsigned(7 downto 0) := (others => '0');
    signal y18 : unsigned(7 downto 0) := (others => '0');
    signal u18, v18 : signed(8 downto 0) := (others => '0');
    -- output (P19)
    signal s_out_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_out_u : unsigned(9 downto 0) := C_MID;
    signal s_out_v : unsigned(9 downto 0) := C_MID;
    signal sd_q  : std_logic_vector(3 downto 0) := "0110";
    signal sd_o  : std_logic_vector(3 downto 0) := "0110";

    -- bit carries (d(k) holds true stage k+1)
    type t_pa is array(7 to 15) of unsigned(2 downto 0);                       -- active & slot
    signal lay_d  : t_pa := (others => (others => '0'));
    type t_att is array(12 to 14) of unsigned(7 downto 0);
    signal att_d  : t_att := (others => (others => '0'));
    type t_k is array(10 to 14) of std_logic;
    signal vldg_d : t_k := (others => '0');

    ----------------------------------------------------------------------
    -- micro-sequencer
    ----------------------------------------------------------------------
    -- instruction word (16 bits): op(15:11) arg(10:0): reg = arg(6:0), imm = arg signed, jump target = arg(10:0)
    constant OP_LINE0 : integer := 0;
    constant OP_LD   : integer := 1;
    constant OP_ST   : integer := 2;
    constant OP_ADD  : integer := 3;
    constant OP_SUB  : integer := 4;
    constant OP_LDI  : integer := 5;
    constant OP_ADDI : integer := 6;
    constant OP_SHR  : integer := 7;
    constant OP_SHL  : integer := 8;
    constant OP_MUL  : integer := 9;
    constant OP_LDP  : integer := 10;
    constant OP_DIV  : integer := 11;
    constant OP_SHRV : integer := 12;
    constant OP_NEG  : integer := 13;
    constant OP_ABS  : integer := 14;
    constant OP_JMP  : integer := 15;
    constant OP_JZ   : integer := 16;
    constant OP_JN   : integer := 17;
    constant OP_JNZ  : integer := 18;
    constant OP_JP   : integer := 19;
    constant OP_TW   : integer := 21;
    constant OP_ROM  : integer := 22;
    constant OP_WAITF : integer := 23;
    constant OP_LDQ  : integer := 24;
    constant OP_SHLV : integer := 25;
    constant OP_LDF  : integer := 26;
    constant OP_LDX  : integer := 27;
    constant OP_STX  : integer := 28;
    constant OP_AND  : integer := 29;
    constant OP_CALL : integer := 30;
    constant OP_RET  : integer := 31;

    signal cs_st    : unsigned(1 downto 0) := (others => '0');
    signal pc       : unsigned(10 downto 0) := (others => '0');
    signal lnk, lnk2 : unsigned(10 downto 0) := (others => '0');   -- two-deep call stack
    signal ir       : std_logic_vector(15 downto 0) := (others => '0');
    signal d_op     : unsigned(4 downto 0) := (others => '0');          -- second (fabric) decode register
    signal d_arg    : std_logic_vector(10 downto 0) := (others => '0');
    signal sp_r     : signed(31 downto 0) := (others => '0');            -- special operand, captured early
    signal acc      : signed(31 downto 0) := (others => '0');
    signal rf_q     : std_logic_vector(31 downto 0) := (others => '0');
    signal rf_we    : std_logic := '0';
    signal rf_wa    : unsigned(6 downto 0) := (others => '0');
    signal sh_cnt   : unsigned(5 downto 0) := (others => '0');
    signal sh_busy  : std_logic := '0';
    signal m_am   : unsigned(31 downto 0) := (others => '0');
    signal m_acc  : unsigned(51 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal m_busy, m_go, m_neg : std_logic := '0';
    signal d_n    : unsigned(31 downto 0) := (others => '0');
    signal d_d    : unsigned(15 downto 0) := (others => '0');
    signal d_rem  : unsigned(16 downto 0) := (others => '0');
    signal d_cnt  : unsigned(5 downto 0) := (others => '0');
    signal d_busy, d_go : std_logic := '0';
    signal busy   : std_logic := '0';
    signal fun_addr : unsigned(8 downto 0) := (others => '0');
    signal fun_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal tw_addr : unsigned(8 downto 0) := (others => '0');
    signal tw_d16  : std_logic_vector(15 downto 0) := (others => '0');
    signal tw_we   : unsigned(2 downto 0) := (others => '0');   -- 0 none, 1 T1, 2 T2, 3 TM, 4 TV, 5 TS, 6 TK
    signal alu_a, alu_b : signed(31 downto 0) := (others => '0');   -- ALU operands registered at execute, summed next clock
    signal alu_c, alu_pend, alu_and : std_logic := '0';

    function f_lzc(v : unsigned(C_CW - 1 downto 0)) return unsigned is
        variable e : unsigned(4 downto 0) := (others => '0');
    begin
        e := (others => '0');
        for i in 0 to C_CW - 1 loop
            if v(i) = '1' then e := to_unsigned(i, 5); end if;
        end loop;
        return e;
    end function;

    -- (clamped to +-256: the alpha ramp only ever looks at -128..127 of it)
    function f_bar(d : signed(16 downto 0); r : unsigned(3 downto 0)) return signed is
        variable v : signed(24 downto 0);
    begin
        v := shift_right(d & x"00", to_integer(r));
        if v > 255 then return to_signed(255, 9);
        elsif v < -256 then return to_signed(-256, 9);
        else return v(8 downto 0); end if;
    end function;

    -- alpha ramp: one constant add (the +128 bias) and one clamp (cap is always <= 255,
    -- so it subsumes the 255 ceiling).  The attenuation is NOT folded in here: b
    -- saturates at 511 over a dot's whole interior, so an offset on the ramp only
    -- shrinks the edge -- it can never dim a dot's core.  It is subtracted after the
    -- clamp instead, which carries a dot smoothly all the way to nothing.
    -- (The former sub-pixel cap is gone: the resolution fade has removed every
    -- dot below one ring per 4 px long before the cap would have engaged.)
    -- With `edge` the ramp is centred on the geometric edge, as anti-aliasing
    -- wants; without it the ramp starts AT the edge, which is what Glow wants.
    function f_alpha(b : signed(8 downto 0); vld : std_logic; edge : std_logic) return unsigned is
        variable v : signed(9 downto 0);
    begin
        if edge = '1' then v := resize(b, 10) + to_signed(128, 10);
        else               v := resize(b, 10); end if;
        if vld = '0' or v <= 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;

    function f_comp(al : unsigned(7 downto 0); ink : std_logic) return unsigned is
    begin
        if ink = '1' then return not al; else return al; end if;
    end function;

    -- one CORDIC vectoring iteration; the shift amount base+j (j = 0..3) is
    -- muxed at the adder inputs so each pipeline stage has one add/sub set.
    procedure cordic_it(variable x, y : inout signed(C_CW - 1 downto 0); variable z : inout signed(15 downto 0);
                        constant base : in integer; constant j : in std_logic_vector(3 downto 0)) is
        variable d : std_logic;
        variable mp, mn, cp, cn : signed(C_CW - 1 downto 0);
        variable zp, zc : signed(15 downto 0);
        variable tx, ty : signed(C_CW - 1 downto 0);
        variable at : signed(15 downto 0);
    begin
        case j is
            when "0001" => tx := shift_right(x, base);     ty := shift_right(y, base);     at := to_signed(C_ATAN(base), 16);
            when "0010" => tx := shift_right(x, base + 1); ty := shift_right(y, base + 1); at := to_signed(C_ATAN(base + 1), 16);
            when "0100" => tx := shift_right(x, base + 2); ty := shift_right(y, base + 2); at := to_signed(C_ATAN(base + 2), 16);
            when others => tx := shift_right(x, base + 3); ty := shift_right(y, base + 3); at := to_signed(C_ATAN(base + 3), 16);
        end case;
        d  := y(C_CW - 1);
        mp := (others => d);  mn := (others => not d);
        cp := (others => '0'); cp(0) := d;
        cn := (others => '0'); cn(0) := not d;
        zp := (others => d);  zc := (others => '0'); zc(0) := d;
        x := x + (ty xor mp) + cp;
        y := y + (tx xor mn) + cn;
        z := z + (at xor zp) + zc;
    end procedure;

begin
    p_relay : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to 7 loop reg_q(i) <= unsigned(registers_in(i)); end loop;
            ilace_q <= ilace; fld_q <= fl_r;
            sw_ink  <= registers_in(6)(2);
            sw_flow <= registers_in(6)(3);
            sw_full <= registers_in(6)(4);
            sw_mesh <= registers_in(6)(0);
            sw_glow <= registers_in(6)(1);
        end if;
    end process p_relay;

    ------------------------------------------------------------------------
    -- raster measurement, blank budgets, frame anchor
    ------------------------------------------------------------------------
    p_time : process(clk)
    begin
        if rising_edge(clk) then
            av_r  <= data_in.avid;
            av_q  <= av_r;
            vs_r  <= data_in.vsync_n;
            vs_q  <= vs_r;
            fl_r  <= data_in.field_n;
            hs_r  <= data_in.hsync_n;
            hs_q  <= hs_r;
            dcnt  <= dcnt + 1;

            if av_r = '1' then
                xcnt <= xcnt + 1;
            elsif av_q = '1' then
                if xcnt > 16 then act_w <= xcnt; end if;
                xcnt  <= (others => '0');
                aline <= aline + 1;
            end if;

            -- blank budgets: hblank length (avid fall -> rise) and the hsync period;
            -- vblank = two hsyncs without active video
            if hs_r = '0' and hs_q = '1' then
                hcnt <= (others => '0');
                if hcnt > 100 then hline <= hcnt; end if;
                if nb /= 3 then nb <= nb + 1; end if;
            elsif hcnt /= 4095 then
                hcnt <= hcnt + 1;
            end if;
            if av_r = '1' then
                blank_cnt <= (others => '0');
                nb <= (others => '0');
                if av_q = '0' and blank_cnt /= 1023 then hbl_len <= blank_cnt; end if;
            elsif blank_cnt /= 1023 then
                blank_cnt <= blank_cnt + 1;
            end if;
            if nb >= 2 then in_vblank <= '1'; else in_vblank <= '0'; end if;
            in_vblank_q <= in_vblank;

            blank_ok <= '0';
            if av_r = '0' and blank_cnt >= 2 then
                if in_vblank = '1' then
                    if hcnt + 64 < hline then blank_ok <= '1'; end if;
                else
                    if blank_cnt + C_BLANK_MARGIN < hbl_len then blank_ok <= '1'; end if;
                end if;
            end if;

            if vs_r = '0' and vs_q = '1' then
                frame_act <= '0';
            elsif av_r = '1' and av_q = '0' and frame_act = '0' then
                frame_act <= '1';
                if aline > 50 then act_h <= aline; end if;
                aline <= (others => '0');
                if fl_r /= anc_fld then ilace <= '1'; else ilace <= '0'; end if;
                anc_fld <= fl_r;
            end if;

            if ilace = '1' then
                if act_h >= 400 then    vstep_base <= to_unsigned(16, 6);
                elsif act_h >= 270 then vstep_base <= to_unsigned(15, 6);
                else                    vstep_base <= to_unsigned(18, 6); end if;
            else
                if act_h >= 700 then    vstep_base <= to_unsigned(16, 6);
                elsif act_h >= 540 then vstep_base <= to_unsigned(15, 6);
                else                    vstep_base <= to_unsigned(18, 6); end if;
            end if;
        end if;
    end process p_time;

    ------------------------------------------------------------------------
    -- Geometry engine: one sample every 8 clocks, 8 px apart, one line ahead.
    -- Lane a is loaded at ph0, lane b at ph4, into a 3-stage CORDIC pipeline
    -- (one iteration per clock per stage, 12 clocks per lane): lane a of
    -- sample k is delivered at ph4 of k+1, lane b at ph0 of k+2.  The log
    -- lane (6 stages) is loaded with lane b at ph1 and lane a at ph5; the
    -- tail is phase-scheduled and a sample's word is written at ph7 three
    -- periods after it was loaded.
    --
    -- Units: the log table is pre-scaled by the sequencer to ring units
    -- (normalised: rings per octave = invn << inve), so a lane value is
    -- L_n = E*invn + M[hi] + lo*slope and u = (La_n - Lb_n - lp_n) << inve.
    -- theta*A is the sum of two table halves of the 14-bit angle; the
    -- split is a table of u.  The AA shift uses exponents + top mantissa
    -- bits as log2 (error < 0.09 octave, below the shift quantum).
    ------------------------------------------------------------------------
    p_eng : process(clk)
        variable vx, vy : signed(C_CW - 1 downto 0);
        variable vz     : signed(15 downto 0);
        variable v0x, v0y, v1x, v1y, v2x, v2y : signed(C_CW - 1 downto 0);
        variable v0z, v1z, v2z : signed(15 downto 0);
        variable ax, ay : signed(15 downto 0);
        variable hi, lo : unsigned(14 downto 0);
        variable sw     : std_logic;
        variable lx, ly : signed(15 downto 0);
        variable v_m    : signed(17 downto 0);
        variable v_fold : unsigned(16 downto 0);
        variable v_th   : unsigned(16 downto 0);
        variable v_say, v_sblx, v_sbly : signed(19 downto 0);
        variable v_n    : unsigned(C_CW - 1 downto 0);
        variable v_u    : signed(21 downto 0);
        variable v_sh   : signed(9 downto 0);
        variable v_dl   : signed(26 downto 0);
        variable v_xc   : unsigned(C_CW - 1 downto 0);
        variable v_rz   : signed(15 downto 0);
        variable v_rsw, v_rsx, v_rsy : std_logic;
        variable v_rx   : unsigned(C_CW - 1 downto 0);
        variable v_sh0  : unsigned(4 downto 0);
        variable v_es   : unsigned(5 downto 0);
        variable v_hs   : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then

            -- requests
            if av_r = '1' and av_q = '0' then pend_line <= '1'; end if;
            if vs_r = '0' and vs_q = '1' then pend_row0 <= '1'; end if;
            -- Row 0 is re-seeded ONLY at vsync.  Honouring the sequencer's request the
            -- moment it arrives would reset the vertical accumulators mid-frame whenever
            -- the per-frame pass ran past the vertical blanking interval, restarting the
            -- iris partway down the screen (seen on hardware as displaced arcs).
            if stx_en = '1' then
                case stx_sel is
                    when "00000" => c_xa0 <= stx_val(19 downto 0);
                    when "00001" => c_ya0 <= stx_val(19 downto 0);
                    when "00010" => c_xb0 <= stx_val(19 downto 0);
                    when "00011" => c_yb0 <= stx_val(19 downto 0);
                    when "00100" => c_sax <= stx_val(19 downto 0);
                    when "00101" => c_say <= stx_val(19 downto 0);
                    when "00110" => c_sbx <= stx_val(19 downto 0);
                    when "00111" => c_sby <= stx_val(19 downto 0);
                    when "01000" => c_sblx <= stx_val(19 downto 0);
                    when "01001" => c_sbly <= stx_val(19 downto 0);
                    when others  => null;                       -- selectors above 9 belong to the CPU
                end case;
            end if;

            v_say  := c_say; v_sblx := c_sblx; v_sbly := c_sbly;
            if ilace = '1' then
                v_say  := shift_left(v_say, 1);
                v_sblx := shift_left(v_sblx, 1);
                v_sbly := shift_left(v_sbly, 1);
            end if;

            -- free-running log lane pipeline (shared by both lanes)
            v_sh0 := to_unsigned(C_CW - 1, 5) - lg_e0;
            case v_sh0(4 downto 2) is
                when "000"  => lg_n1 <= lg_in;
                when "001"  => lg_n1 <= shift_left(lg_in, 4);
                when "010"  => lg_n1 <= shift_left(lg_in, 8);
                when "011"  => lg_n1 <= shift_left(lg_in, 12);
                when others => lg_n1 <= shift_left(lg_in, 16);
            end case;
            lg_f1 <= v_sh0(1 downto 0); lg_e1 <= lg_e0;
            v_n := shift_left(lg_n1, to_integer(lg_f1));
            -- the normaliser puts the leading one at bit C_CW-1, so the mantissa slices
            -- must track C_CW (they were hard-wired for an 18-bit datapath)
            lg_hi2 <= v_n(C_CW - 2 downto C_CW - 9); lg_lo2 <= v_n(C_CW - 10 downto C_CW - 13); lg_e2 <= lg_e1;
            lg_w3 <= tm_ram(to_integer(lg_hi2)); lg_lo3 <= lg_lo2; lg_e3 <= lg_e2;
            lg_v4 <= unsigned(lg_w3(15 downto 5)); lg_q4 <= unsigned(lg_w3(4 downto 0)); lg_lo4 <= lg_lo3; lg_e4 <= lg_e3;
            lg_p5 <= lg_lo4 * (lg_q4 & '0'); lg_v5 <= lg_v4;
            lg_ep5 <= lg_e4 * c_invn;
            lg_out <= resize(lg_ep5, 17) + resize(lg_v5, 17) + resize(lg_p5(9 downto 5), 17);

            -- last-period flag, registered (eng_k only changes at ph7)
            if eng_k = eng_last then eng_kl <= '1'; else eng_kl <= '0'; end if;

            -- table reads for theta*A and the split (addresses registered by the tail)
            tv_w <= tv_ram(to_integer(tv_ad));
            ts_w <= ts_ram(to_integer(ts_ad));

            -- CORDIC load preparation, registered in two steps so neither is a long
            -- combinational chain: select + abs + saturate at ph6 (lane a) / ph2 (lane b),
            -- compare + swap at ph7 / ph3, load at ph0 / ph4.
            if eng_oh(6) = '1' then
                lx := sxa(18 downto 3); ly := sya(18 downto 3);     -- lane A: z - a, in Q13
            else
                lx := sxb(18 downto 3); ly := syb(18 downto 3);     -- lane B: 1 - conj(a) z, in Q13
            end if;
            if lx(15) = '1' then ax := -lx; else ax := lx; end if;
            if ly(15) = '1' then ay := -ly; else ay := ly; end if;
            if ax(15) = '1' then ax := to_signed(32767, 16); end if;
            if ay(15) = '1' then ay := to_signed(32767, 16); end if;
            if eng_oh(6) = '1' or eng_oh(2) = '1' then
                la_x <= unsigned(ax(14 downto 0)); la_y <= unsigned(ay(14 downto 0));
                la_s <= lx(15) & ly(15);
            end if;
            if eng_oh(7) = '1' or eng_oh(3) = '1' then
                if la_x >= la_y then
                    ld_hi <= la_x; ld_lo <= la_y; ld_f <= '0' & la_s;
                else
                    ld_hi <= la_y; ld_lo <= la_x; ld_f <= '1' & la_s;
                end if;
            end if;

            -- free-running run lengths (act_w is static), so the start path has no adders
            nsamp_r <= resize(act_w(11 downto 3), 9) + 1;
            last_r  <= resize(act_w(11 downto 3), 9) + 3;

            -- run start in two steps: pick the pending request (a priority chain over three
            -- flags), then load the run registers from a registered 2-bit selector
            if st_go = '1' then
                st_go <= '0'; eng_busy <= '1';
                eng_oh <= "00100000"; eng_oh2 <= "00100000"; eng_k <= (others => '1');
                cj0 <= "0001"; cj1 <= "0001"; cj2 <= "0001";   -- runs start at ph5, so the index reads 3 at ph0
                eng_nsamp <= nsamp_r; eng_last <= last_r;
                sxa <= c_xa0;
                if st_kind(1) = '0' then                       -- ordinary line: advance down the frame
                    sya <= sya + v_say;
                    lxb <= lxb + v_sblx;  sxb <= lxb + v_sblx;
                    lyb <= lyb + v_sbly;  syb <= lyb + v_sbly;
                elsif ilace = '1' and fl_r = '0' then          -- row 0, bottom field: start half a line down
                    sya <= c_ya0 + shift_right(v_say, 1);
                    lxb <= c_xb0 + shift_right(v_sblx, 1);  sxb <= c_xb0 + shift_right(v_sblx, 1);
                    lyb <= c_yb0 + shift_right(v_sbly, 1);  syb <= c_yb0 + shift_right(v_sbly, 1);
                else                                           -- row 0
                    sya <= c_ya0;  lxb <= c_xb0;  sxb <= c_xb0;  lyb <= c_yb0;  syb <= c_yb0;
                end if;
            elsif eng_busy = '0' then
                if pend_line = '1' then
                    pend_line <= '0'; st_go <= '1'; st_kind <= "00";
                elsif pend_row0 = '1' then
                    pend_row0 <= '0'; st_go <= '1'; st_kind <= "10";
                end if;
            else
                eng_oh <= eng_oh(6 downto 0) & eng_oh(7); eng_oh2 <= eng_oh2(6 downto 0) & eng_oh2(7);
                -- CORDIC pipeline: each stage runs one iteration per clock (j = ph-1 mod 4);
                -- at ph0/ph4 (j = 3) the stages hand over: stage 0 loads a lane (a at
                -- ph0, b at ph4), stage 2 delivers a lane (a at ph4, b at ph0).
                cj0 <= cj0(2 downto 0) & cj0(3); cj1 <= cj1(2 downto 0) & cj1(3); cj2 <= cj2(2 downto 0) & cj2(3);
                vx := c0x; vy := c0y; vz := c0z; cordic_it(vx, vy, vz, 0, cj0); v0x := vx; v0y := vy; v0z := vz;
                vx := c1x; vy := c1y; vz := c1z; cordic_it(vx, vy, vz, 4, cj1); v1x := vx; v1y := vy; v1z := vz;
                vx := c2x; vy := c2y; vz := c2z; cordic_it(vx, vy, vz, 8, cj2); v2x := vx; v2y := vy; v2z := vz;
                c0x <= v0x; c0y <= v0y; c0z <= v0z;
                c1x <= v1x; c1y <= v1y; c1z <= v1z;
                c2x <= v2x; c2y <= v2y; c2z <= v2z;
                -- stage-0 load (lane a at ph0, lane b at ph4) from values prepared one clock earlier
                if eng_oh(0) = '1' or eng_oh(4) = '1' then
                    c0x <= shift_left(signed(resize(ld_hi, C_CW)), C_CG);
                    c0y <= shift_left(signed(resize(ld_lo, C_CW)), C_CG);
                    c0z <= (others => '0');
                    c0f <= ld_f;
                    c1x <= v0x; c1y <= v0y; c1z <= v0z; c1f <= c0f;
                    c2x <= v1x; c2y <= v1y; c2z <= v1z; c2f <= c1f;
                    if v2x < 0 then v_xc := (others => '0'); else v_xc := unsigned(v2x); end if;
                    if eng_oh(0) = '1' then
                        rb_x <= v_xc; rb_z <= v2z; rb_sw <= c2f(2); rb_sx <= c2f(1); rb_sy <= c2f(0);
                        sxa <= sxa + c_sax;                         -- lane A's y is constant along a line
                    else
                        ra_x <= v_xc; ra_z <= v2z; ra_sw <= c2f(2); ra_sx <= c2f(1); ra_sy <= c2f(0);
                        sxb <= sxb + c_sbx; syb <= syb + c_sby;
                    end if;
                end if;
                -- shared angle fold (lane b at ph1, lane a at ph5)
                if eng_oh(1) = '1' then
                    v_rz := rb_z; v_rsw := rb_sw; v_rsx := rb_sx; v_rsy := rb_sy; v_rx := rb_x;
                else
                    v_rz := ra_z; v_rsw := ra_sw; v_rsx := ra_sx; v_rsy := ra_sy; v_rx := ra_x;
                end if;
                if v_rsw = '1' then v_m := to_signed(32768, 18) - resize(v_rz, 18);
                else                v_m := resize(v_rz, 18); end if;
                if v_rsx = '0' and v_rsy = '0' then    v_fold := unsigned(v_m(16 downto 0));
                elsif v_rsx = '1' and v_rsy = '0' then v_fold := unsigned(resize(to_signed(65536, 18) - v_m, 17));
                elsif v_rsx = '1' and v_rsy = '1' then v_fold := unsigned(resize(to_signed(65536, 18) + v_m, 17));
                else                                   v_fold := unsigned(resize(-v_m, 17)); end if;
                if eng_oh(1) = '1' or eng_oh(5) = '1' then
                    lg_in <= v_rx; lg_e0 <= f_lzc(v_rx);
                    if eng_oh(1) = '1' then fb <= v_fold; else fa <= v_fold; end if;
                end if;

                -- tail.  Sample k is loaded in period k; lane a's log is in lg_out
                -- during ph4..7 of k+2, lane b's during ph0..3 of k+3 (each log-lane
                -- stage holds for 4 clocks).  The word of sample k is written at
                -- ph7 of period k+3.
                if eng_oh(0) = '1' then
                    ea <= lg_e2; ha <= lg_hi2;                                          -- lane A, sample k-2
                    lg_b <= lg_out;                                                     -- lane b, sample k-3
                    e_v <= e_vh + e_vl + (vcut & x"000");                              -- sample k-2, cut-corrected
                end if;
                if eng_oh(1) = '1' then
                    e_l <= signed(resize(lg_a, 18)) - signed(resize(lg_b, 18));        -- sample k-3
                end if;
                if eng_oh(2) = '1' then
                    v_th := fa - fb;
                    th <= v_th;                                                         -- sample k-2
                    e_ld <= resize(e_l, 19) - resize(c_lp, 19);                        -- sample k-3
                    -- The arm coordinate holds 16 arms, so where the angle wraps (its cut,
                    -- a curve running out of the pupil) v jumps by A mod 16 arms.  That is
                    -- a whole number of arms, so the dot test is right AT the samples --
                    -- but the pixel-path DDA interpolates between the two samples that
                    -- straddle the cut and sweeps through those arms in 8 px, planting one
                    -- phantom dot per arm along the cut.  Count the crossings along the
                    -- line (by angle quadrant, 3->0 forward, 0->3 back) and add that many
                    -- times A mod 16 to v, which keeps consecutive samples continuous and
                    -- changes nothing mod one arm.
                    if eng_k(8) = '0' and eng_k >= 2 then                              -- real samples only
                        if eng_k = 2 then vcut <= (others => '0');
                        elsif thq_p = "11" and v_th(16 downto 15) = "00" then vcut <= vcut + c_a;
                        elsif thq_p = "00" and v_th(16 downto 15) = "11" then vcut <= vcut - c_a;
                        end if;
                        thq_p <= v_th(16 downto 15);
                    end if;
                end if;
                if eng_oh(3) = '1' then
                    tv_ad <= '0' & th(16 downto 10);
                    v_dl := shift_left(resize(e_ld, 27), to_integer(c_inve));
                    if v_dl > 524287 then e_u <= to_signed(524287, 20);
                    elsif v_dl < -524288 then e_u <= to_signed(-524288, 20);
                    else e_u <= v_dl(19 downto 0); end if;
                end if;
                if eng_oh(4) = '1' then
                    lg_a <= lg_out;                                                     -- lane a, sample k-2
                    eb <= lg_e2; hb <= lg_hi2;                                          -- lane b, sample k-2
                    tv_ad <= '1' & th(9 downto 3);
                    if e_u(19) = '1' then ts_ad <= (others => '0');
                    elsif e_u(18) = '1' then ts_ad <= (others => '1');
                    else ts_ad <= unsigned(e_u(17 downto 10)); end if;
                end if;
                if eng_oh(5) = '1' then
                    e_vh <= unsigned(tv_w);
                    -- the anti-alias ramp normalises d(log|w|)/d(pixel) = (1-|a|^2)/(r1 r2 R),
                    -- so it needs log2(r1 r2) from both lanes' exponents and mantissa tops
                    v_es := resize(ea, 6) + resize(eb, 6);
                    v_hs := resize(ha, 9) + resize(hb, 9);
                    e_gs <= (v_es & x"000") + resize(v_hs & "0000", 18);
                end if;
                if eng_oh(6) = '1' then
                    e_vl <= unsigned(tv_w);
                    e_sep <= signed(ts_w(13 downto 0));                                 -- sample k-3
                    v_u  := resize(c_cdot, 22) - signed(resize(e_gs, 22));
                    -- round to the nearest octave for the AA shift with a 10-bit
                    -- increment off the same subtraction, not a second 22-bit add
                    if v_u(11) = '1' then v_sh := v_u(21 downto 12) + 1;
                    else                  v_sh := v_u(21 downto 12); end if;
                    e_rd2 <= e_rd;                                                      -- sample k-3
                    -- four bits: the resolution fade has emptied everything past rd 10,
                    -- so the barrel never needs a fifth shift stage
                    if v_sh < 0 then e_rd <= (others => '0');
                    elsif v_sh > 15 then e_rd <= to_unsigned(15, 4);
                    else e_rd <= unsigned(v_sh(3 downto 0)); end if;
                    -- Resolution fade.  v_u is log2 of the lattice density in Q12 ring
                    -- units per pixel: f_bar's shift turns a Q12 ring distance into a
                    -- 256-per-pixel alpha ramp, which pins rd = log2(units per pixel)
                    -- exactly.  Because the map is conformal that ONE number covers both
                    -- axes -- rings and arms crowd together by the same factor -- so the
                    -- arms need no test of their own (and a difference of the arm
                    -- coordinate could not supply one anyway: it wraps every 16 arms, and
                    -- the wrap would black out a radial spoke, a seam of the kind being
                    -- cured here).  Past one ring per 8 px (rd 9) the lattice is finer
                    -- than the sampler's own pitch, the DDA interpolates across structure
                    -- it never saw, and the dots degenerate into coloured hash; by one
                    -- ring per 4 px (rd 10) there is nothing left to draw.  That span is
                    -- exactly one octave and sits on a power of two, so the ramp between
                    -- them is a plain slice of the subtraction already made above.
                    e_at2 <= e_at;                                                      -- sample k-3
                    if v_u(21 downto 12) < to_signed(9, 10) then e_at <= (others => '0');
                    elsif v_u(21 downto 12) > to_signed(9, 10) then e_at <= (others => '1');
                    else e_at <= unsigned(v_u(11 downto 6)); end if;
                end if;
                if eng_oh(7) = '1' then
                    if eng_kl = '1' then
                        eng_busy <= '0';
                    else
                        eng_k <= eng_k + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_eng;

    ------------------------------------------------------------------------
    -- sample buffer (4 EBR, single bank: the engine's writes for the next
    -- line trail the beam's reads of this line) + sync delay line
    ------------------------------------------------------------------------
    p_sbuf : process(clk)
        variable v_ra : unsigned(8 downto 0);
        variable v_wk : unsigned(8 downto 0);
        variable v_d0, v_d1, v_d2, v_d3 : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            -- engine write of the word of sample k-3 at ph7
            v_wk := eng_k - 3;
            v_d0 := std_logic_vector(e_u(15 downto 0));
            v_d1 := std_logic_vector(e_u(19 downto 16)) & std_logic_vector(e_v(11 downto 0));
            v_d2 := "0" & std_logic_vector(e_v(15 downto 12)) & std_logic_vector(e_sep(10 downto 0));
            v_d3 := std_logic_vector(e_sep(13 downto 11)) & "000" & std_logic_vector(e_rd2) & std_logic_vector(e_at2);
            if eng_busy = '1' and eng_oh2(7) = '1' and eng_k >= 3 and v_wk < eng_nsamp then
                sb0(to_integer(v_wk(7 downto 0))) <= v_d0;
                sb1(to_integer(v_wk(7 downto 0))) <= v_d1;
                sb2(to_integer(v_wk(7 downto 0))) <= v_d2;
                sb3(to_integer(v_wk(7 downto 0))) <= v_d3;
            end if;
            -- beam reads (px = pixel-1, span k = pixels 8k..8k+7 uses samples k, k+1):
            -- at px=1 mod 8 fetch sample k+1 (the pair's next member, loaded at px=2);
            -- at px=2 mod 8 fetch sample k-1, whose AA shift is captured at px=3 for
            -- stage 14 (13 clocks behind the input, i.e. span k-1)
            if av_q = '0' then v_ra := (others => '0');
            elsif px(2 downto 0) = "010" then v_ra := resize(px(10 downto 3), 9) - 1;
            else v_ra := resize(px(10 downto 3), 9) + 1; end if;
            sq0 <= sb0(to_integer(v_ra(7 downto 0)));
            sq1 <= sb1(to_integer(v_ra(7 downto 0)));
            sq2 <= sb2(to_integer(v_ra(7 downto 0)));
            sq3 <= sb3(to_integer(v_ra(7 downto 0)));
            if px(2 downto 0) = "011" then
                rd_h1 <= unsigned(sq3(9 downto 6));     -- (word: sep[15:13] 000[12:10] rd[9:6] fade[5:0])
                res6  <= unsigned(sq3(5 downto 0)) & "00";
            end if;
            sd_ram(to_integer(dcnt)) <= av_r & hs_r & vs_r & fl_r;
            sd_q <= sd_ram(to_integer(dcnt - 19));   -- 17, plus the two pixels the layer holds now trail by
        end if;
    end process p_sbuf;

    ------------------------------------------------------------------------
    -- pixel front: sample pair registers + DDA between samples (stage 6:
    -- the accumulators ARE the stage-6 values)
    ------------------------------------------------------------------------
    p_dda : process(clk)
        variable v_du : signed(19 downto 0);
        variable v_dv : signed(15 downto 0);
        variable v_wu : std_logic_vector(19 downto 0);
        variable v_wv : std_logic_vector(15 downto 0);
        variable v_ws : std_logic_vector(13 downto 0);
    begin
        if rising_edge(clk) then
            if av_q = '0' then px <= (others => '1'); else px <= px + 1; end if;
            if av_q = '0' then slot <= (others => '0'); else slot <= slot + 1; end if;   -- G,R,G,B
            if av_q = '0' or px(2 downto 0) = "010" then
                cur_u <= nxt_u; cur_v <= nxt_v; sep6 <= nxt_s;
                v_wu := sq1(15 downto 12) & sq0;
                v_wv := sq2(14 downto 11) & sq1(11 downto 0);
                v_ws := sq3(15 downto 13) & sq2(10 downto 0);
                nxt_u <= signed(v_wu);
                nxt_v <= signed(v_wv);
                nxt_s <= signed(v_ws);
            end if;
            -- reload during blanking as well, otherwise the accumulators free-run
            -- through the whole blanking interval and the first pixels of every line
            -- are drawn from stale values (random dots down the left edge)
            if av_q = '0' or px(2 downto 0) = "011" then
                du8   <= nxt_u - cur_u;
                -- (The 16-bit arm coordinate wraps between samples, but only its low 12
                -- bits reach the dot test, and the wrap error is exactly 16 whole arms --
                -- a multiple of 4096 -- so the exact difference is already correct here.)
                v_dv  := nxt_v - cur_v;
                dv8   <= v_dv(14 downto 0);
                v_du  := cur_u - signed(resize(c_ph, 20));                          -- flow phase folded in
                acc_u <= v_du & "000";
                acc_v <= cur_v(11 downto 0) & "000";
            else
                acc_u <= acc_u + resize(du8, 23);
                acc_v <= acc_v + dv8;
            end if;
        end if;
    end process p_dda;
    ------------------------------------------------------------------------
    -- One dot machine shared by the three layers: at pixel p it evaluates
    -- layer slot(p) (0 R, 1 G, 2 B) with that layer's split offset, and the
    -- resulting alpha is held for the next three pixels.  Stage k of the
    -- machine is true stage k.
    --
    -- P7: layer offset, pupil alpha
    -- P8: twist term (per-ring EBR table)
    ------------------------------------------------------------------------
    p_lat : process(clk)
        variable v_u6  : signed(19 downto 0);
        variable v_v6  : unsigned(11 downto 0);
        variable v_ul  : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- P7
            v_u6 := acc_u(22 downto 3);                -- already offset by the flow phase
            v_v6 := unsigned(acc_v(14 downto 3));
            v_ul := v_u6;
            -- Four-phase layer schedule G,R,G,B: green (which carries most of the
            -- luminance, and so most of the apparent sharpness) is refreshed every
            -- second pixel instead of every third; red and blue, which only carry the
            -- colour fringing, are refreshed every fourth -- and smoothed to a 2 px
            -- cadence in p_out.
            case slot is
                when "01" =>                       -- red leads
                    ul7 <= v_ul + resize(shift_right(sep6, 1), 20);
                    v7  <= v_v6 + unsigned(sep6(11 downto 0));
                when "11" =>                       -- blue trails
                    ul7 <= v_ul - resize(shift_right(sep6, 2), 20);
                    v7  <= v_v6 - unsigned(sep6(11 downto 0));
                when others =>                     -- green anchors (slots 0 and 2)
                    ul7 <= v_ul;
                    v7  <= v_v6;
            end case;
            slot7 <= slot;
            av7   <= av_q;
            -- P8 (twist term from the per-ring table)
            ug8 <= ul7;
            vg8 <= v7;
            tk_q <= tk_ram(to_integer(unsigned(ul7(19 downto 12))));
        end if;
    end process p_lat;

    ------------------------------------------------------------------------
    -- P9: ring/frac split, twist offset
    -- P10: arm coordinate (mod 1) -> |dv| -> table indices, validity flags
    ------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_tk : unsigned(11 downto 0);
        variable v_vg : unsigned(11 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_ad : unsigned(11 downto 0);
        variable v_pd : signed(20 downto 0);
        variable v_rc : std_logic_vector(19 downto 0);
    begin
        if rising_edge(clk) then
            -- P9: ring/frac split, and the pupil fade.  The distance is measured from the
            -- RING'S OWN CENTRE (its dots' u), not from the pixel's u, so every dot in a
            -- ring gets one and the same attenuation: the rings go out one at a time as
            -- the knob turns instead of each ring shading across itself.  Two rings from
            -- full to black, so only one ring is ever in transition.
            kk9 <= ug8(19 downto 12); fg9 <= unsigned(ug8(11 downto 0));
            v_rc := std_logic_vector(ug8(19 downto 12)) & x"800";
            v_pd := resize(c_up, 21) - resize(signed(v_rc), 21);
            if v_pd <= 0 then pu9 <= (others => '0');
            elsif v_pd(20 downto 13) /= x"00" then pu9 <= x"FF";
            else pu9 <= unsigned(v_pd(12 downto 5)); end if;
            tk9 <= signed(tk_q(13 downto 0));
            vg9 <= vg8;
            -- P10: v = theta*A (+sep) + T(k+1/2+ph), then |dv|, indices
            v_tk := unsigned(tk9(11 downto 0));
            v_vg := vg9 + v_tk;
            v_d := signed(v_vg xor x"800");
            if v_d(11) = '1' then v_ad := unsigned(-v_d); else v_ad := unsigned(v_d); end if;
            if v_ad(11) = '1' then v_ad := x"7FF"; end if;
            jg10 <= v_ad(10 downto 3); lvg10 <= v_ad(2 downto 0);
            ig10 <= fg9(11 downto 4);
            pu10 <= pu9;                       -- (ig10 doubles as the rim fade's position across the ring)
            if kk9 < c_n then vldg10 <= '1'; else vldg10 <= '0'; end if;
            if kk9 = c_nm1 then fdg10 <= '1'; else fdg10 <= '0'; end if;
        end if;
    end process p_cell;

    ------------------------------------------------------------------------
    -- P11: table EBR reads   P12: re-register   P13: diff = B - (S + slope*lo)
    ------------------------------------------------------------------------
    p_dot : process(clk)
        variable v_pg : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            t1g11 <= t1_ram(to_integer(ig10));
            t2g11 <= t2_ram(to_integer(jg10));
            lvg11 <= lvg10;

            bvg12 <= signed(t1g11); svg12 <= unsigned(t2g11(15 downto 3)); ssg12 <= unsigned(t2g11(2 downto 0));
            lvg12 <= lvg11;

            v_pg := ssg12 * lvg12;
            dfg13 <= resize(bvg12, 17) - signed(resize(svg12, 17)) - signed(resize(v_pg, 17));
        end if;
    end process p_dot;

    ------------------------------------------------------------------------
    -- bit carries: pupil alpha / layer id (P7 -> P15), flags (P10 -> P14)
    ------------------------------------------------------------------------
    p_carry : process(clk)
        variable v_rm : unsigned(7 downto 0);
        variable v_at : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            lay_d(7) <= av7 & slot7;
            for i in 8 to 15 loop lay_d(i) <= lay_d(i - 1); end loop;
            vldg_d(10) <= vldg10;
            for i in 11 to 14 loop
                vldg_d(i) <= vldg_d(i - 1);
            end loop;
            -- P11: total attenuation.  The outermost ring's level is a PER-FRAME
            -- constant (c_fade = flow phase), so the whole ring dims together as it
            -- drifts out through the rim and is gone by the time it would cross -- rather
            -- than shading from one side of itself to the other.  It also keeps the field
            -- inside the rim: Flow carries the band up to a ring past |w| = 1, and this
            -- is what stops it being drawn there.
            if fdg10 = '1' then v_rm := c_fade; else v_rm := (others => '0'); end if;
            v_at := resize(pu10, 10) + resize(v_rm, 10) + resize(res6, 10);
            if v_at(9 downto 8) /= "00" then att11 <= x"FF"; else att11 <= v_at(7 downto 0); end if;
            att_d(12) <= att11;
            att_d(13) <= att_d(12);
            att_d(14) <= att_d(13);
        end if;
    end process p_carry;

    ------------------------------------------------------------------------
    -- P14: AA barrel   P15: alpha, demuxed into per-layer holds
    -- P16: composite + pupil  P17: Y product   P18: Y8/U/V   P19: output
    --
    -- Red and blue are evaluated once every four pixels and used to be simply
    -- HELD, so the edge of every red or blue dot was a staircase of 4 px
    -- treads -- the "pixelated" coloured edges.  Now each layer keeps its
    -- previous evaluation too, and for the two pixels after a fresh one it
    -- shows the midpoint of the two before showing the new value: a 2 px
    -- cadence with a half step between, the same quality as green's own
    -- 2 px hold.  That output trails the beam by two or three pixels, so
    -- green is taken from ITS previous evaluation (two or three behind at
    -- its cadence) to match, and the sync tap moves by two so nothing on
    -- screen shifts.  At the first evaluation of a line the "previous" value
    -- is the line above's tail, so it is seeded with the fresh one instead.
    -- With the split at zero all three layers sit on the same lattice and
    -- would draw the SAME dots at different sampling phases -- the coloured
    -- fringe seen at 0 %.  The sequencer raises c_fl(0) there and red and
    -- blue simply become copies of green.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_rd : unsigned(3 downto 0);
        variable v_sl : unsigned(1 downto 0);
        variable v_ar, v_ab : unsigned(8 downto 0);
        variable v_r, v_b : unsigned(7 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_bmy, v_rmy : signed(9 downto 0);
        variable v_u, v_v : signed(9 downto 0);
        variable v_yo : unsigned(10 downto 0);
        variable v_uo, v_vo : signed(11 downto 0);
        variable v_ax : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- P14.  Glow: shift by a constant 11 -- the scale at which a dot is one
            -- pixel wide -- so the ramp spans the whole dot instead of one pixel of
            -- its edge, and every dot becomes a soft ball with its peak at the centre.
            if sw_glow = '1' then v_rd := "1011"; else v_rd := rd_h1; end if;
            bar14_g <= f_bar(dfg13, v_rd);
            -- P15
            v_ax := f_alpha(bar14_g, vldg_d(13), not sw_glow);
            if v_ax > att_d(14) then v_ax := v_ax - att_d(14); else v_ax := (others => '0'); end if;
            if lay_d(13)(2) = '1' and lay_d(14)(2) = '0' then    -- first active pixel of the line
                fr_r <= '1'; fr_b <= '1';
            end if;
            if lay_d(13)(2) = '1' then                            -- (nothing moves during blanking)
                case lay_d(13)(1 downto 0) is
                    when "01"   => if fr_r = '1' then pv_r <= v_ax; else pv_r <= al15_r; end if;
                                   al15_r <= v_ax; fr_r <= '0';
                    when "11"   => if fr_b = '1' then pv_b <= v_ax; else pv_b <= al15_b; end if;
                                   al15_b <= v_ax; fr_b <= '0';
                    when others => pv_g <= al15_g; al15_g <= v_ax;
                end case;
            end if;
            -- P16 composite (light: additive on black; ink: subtractive on white).
            -- Red was evaluated at slot 1, so slots 1 and 2 show its midpoint; blue
            -- at slot 3, so slots 3 and 0 show its midpoint.
            v_sl := lay_d(14)(1 downto 0);
            v_ar := resize(al15_r, 9) + resize(pv_r, 9);
            v_ab := resize(al15_b, 9) + resize(pv_b, 9);
            if v_sl(0) /= v_sl(1) then v_r := v_ar(8 downto 1); else v_r := al15_r; end if;
            if v_sl(0) = v_sl(1)  then v_b := v_ab(8 downto 1); else v_b := al15_b; end if;
            if c_fl(0) = '1' then v_r := pv_g; v_b := pv_g; end if;
            cr16 <= f_comp(v_r, sw_ink);
            cg16 <= f_comp(pv_g, sw_ink);
            cb16 <= f_comp(v_b, sw_ink);
            -- P17: Y*16 = 5R + 9G + 2B
            v_y := shift_left(resize(cr16, 12), 2) + resize(cr16, 12)
                 + shift_left(resize(cg16, 12), 3) + resize(cg16, 12)
                 + shift_left(resize(cb16, 12), 1);
            yy17 <= v_y;
            r17 <= cr16; b17 <= cb16;
            -- P18: Y8, U = 0.5625 (B-Y), V = 0.719 (R-Y)
            y18 <= yy17(11 downto 4);
            v_bmy := signed(resize(b17, 10)) - signed(resize(yy17(11 downto 4), 10));
            v_rmy := signed(resize(r17, 10)) - signed(resize(yy17(11 downto 4), 10));
            v_u := shift_right(v_bmy, 1) + shift_right(v_bmy, 4);
            v_v := shift_right(v_rmy, 1) + shift_right(v_rmy, 2) - shift_right(v_rmy, 5);
            u18 <= resize(v_u, 9);
            v18 <= resize(v_v, 9);
            -- P19: to 10-bit (Y 64..~768, chroma 512 +/- 3*U8), blanking gate, U/V swap
            v_yo := to_unsigned(64, 11) + shift_left(resize(y18, 11), 1)
                    + resize(shift_right(y18, 1), 11) + resize(shift_right(y18, 2), 11);
            v_uo := to_signed(512, 12) + resize(u18, 12) + resize(u18, 12) + resize(u18, 12);
            v_vo := to_signed(512, 12) + resize(v18, 12) + resize(v18, 12) + resize(v18, 12);
            sd_o <= sd_q;
            if sd_q(3) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo(9 downto 0);
                s_out_u <= unsigned(v_vo(9 downto 0));
                s_out_v <= unsigned(v_uo(9 downto 0));
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd_o(3);
    data_out.hsync_n <= sd_o(2);
    data_out.vsync_n <= sd_o(1);
    data_out.field_n <= sd_o(0);
    ------------------------------------------------------------------------
    -- table write ports (one write statement per RAM)
    ------------------------------------------------------------------------
    p_twrite : process(clk)
    begin
        if rising_edge(clk) then
            if tw_we = 1 then t1_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 2 then t2_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 3 then tm_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 4 then tv_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 5 then ts_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 6 then tk_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
        end if;
    end process p_twrite;

    busy <= m_busy or d_busy or m_go or d_go;

    ------------------------------------------------------------------------
    -- micro-sequencer: accumulator machine, 3 cycles per instruction
    -- (fetch / register read / execute), gated by blank_ok.  Multi-cycle
    -- instructions (shifts, serial units, probe, frame wait) hold in the
    -- execute phase.  Every load goes through the one adder.
    ------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_op   : integer range 0 to 31;
        variable v_reg  : unsigned(6 downto 0);
        variable v_imm  : signed(10 downto 0);
        variable v_sel  : integer range 0 to 31;
        variable v_rq   : signed(31 downto 0);
        variable v_ms   : signed(32 downto 0);
        variable v_am   : signed(32 downto 0);
        variable v_opa, v_opb : signed(31 downto 0);
        variable v_sub  : std_logic;
        variable v_lx   : unsigned(19 downto 0);
        variable v_rem  : unsigned(16 downto 0);
        variable v_done : std_logic;
        variable v_jump : std_logic;
    begin
        if rising_edge(clk) then
            -- The serial units and the register-file write port run FREE, outside the
            -- blanking gate: they are self-contained, their results are only consumed by
            -- the gated CPU, and keeping them off that gate's fanout both shortens its
            -- routing and lets a multiply or divide finish during active video.
            m_go <= '0'; d_go <= '0'; stx_en <= '0';
            tw_we <= (others => '0');
            rf_we <= '0'; row0_req <= '0';
            if in_vblank = '1' and in_vblank_q = '0' then seq_req <= '1'; end if;
            if rf_we = '1' then
                rf_mem(to_integer(rf_wa)) <= std_logic_vector(acc);
            end if;

            -- serial multiplier (signed 32 x signed 20, 20 steps; the last step subtracts)
            if m_go = '1' then
                m_cnt  <= (others => '0');
                m_busy <= '1';
            elsif m_busy = '1' then
                v_ms := resize(signed(m_acc(51 downto 20)), 33);
                v_am := resize(signed(m_am), 33);
                if m_acc(0) = '1' then
                    if m_cnt = 19 then v_ms := v_ms - v_am; else v_ms := v_ms + v_am; end if;
                end if;
                m_acc <= unsigned(v_ms(32 downto 0)) & m_acc(19 downto 1);
                m_cnt <= m_cnt + 1;
                if m_cnt = 19 then
                    m_busy <= '0';
                end if;
            end if;

            -- serial divider (32 / 16 -> 32, restoring)
            if d_go = '1' then
                d_rem  <= (others => '0');
                d_cnt  <= (others => '0');
                d_busy <= '1';
            elsif d_busy = '1' then
                -- the remainder is always below the 16-bit divisor, so 17 bits suffice
                v_rem := d_rem(15 downto 0) & d_n(31);
                if v_rem >= resize(d_d, 17) then
                    v_rem := v_rem - resize(d_d, 17);
                    d_n   <= d_n(30 downto 0) & '1';
                else
                    d_n   <= d_n(30 downto 0) & '0';
                end if;
                d_rem <= v_rem;
                d_cnt <= d_cnt + 1;
                if d_cnt = 31 then d_busy <= '0'; end if;
            end if;


            if blank_ok = '1' then
                -- CPU.  The instruction register is absorbed into the program EBR's
                -- output register, so everything downstream decodes from a SECOND,
                -- fabric-registered copy (d_op / d_arg) captured during the register-read
                -- state; only the register-file address is taken straight off the EBR.
                v_op  := to_integer(d_op);
                v_reg := unsigned(d_arg(6 downto 0));
                v_imm := signed(d_arg);
                v_sel := to_integer(unsigned(d_arg(4 downto 0)));
                v_rq  := signed(rf_q);
                v_done := '1';
                v_jump := '0';


                v_sub := '0';
                v_opa := (others => '0');
                case v_op is
                    when OP_ADD  => v_opa := acc; v_opb := v_rq;
                    when OP_ADDI => v_opa := acc; v_opb := resize(v_imm, 32);
                    when OP_SUB  => v_opa := acc; v_opb := v_rq; v_sub := '1';
                    when OP_NEG | OP_ABS => v_opb := acc; v_sub := '1';
                    when OP_LDI  => v_opb := resize(v_imm, 32);
                    when OP_LDP | OP_LDQ | OP_LDF | OP_LDX => v_opb := sp_r;
                    when others  => v_opb := v_rq;
                end case;
                if v_sub = '1' then v_opb := not v_opb; end if;

                -- ALU writeback (operands registered at execute; the add is its own clock)
                if alu_pend = '1' then
                    alu_pend <= '0';
                    if alu_and = '1' then acc <= alu_a and alu_b;
                    else acc <= alu_a + alu_b + ("0000000000000000000000000000000" & alu_c); end if;
                end if;

                case to_integer(cs_st) is
                when 0 =>
                    ir <= prog_mem(to_integer(pc));
                    fun_q <= rom_fun(to_integer(fun_addr));
                    cs_st <= to_unsigned(1, 2);
                when 1 =>
                    -- direct slices of the EBR output only: no logic on this hop
                    rf_q  <= rf_mem(to_integer(unsigned(ir(6 downto 0))));
                    d_op  <= unsigned(ir(15 downto 11));
                    d_arg <= ir(10 downto 0);
                    cs_st <= to_unsigned(2, 2);
                    sh_busy <= '0';
                when 2 =>
                    -- the special operand sources get a clock of their own, decoded from
                    -- the fabric copy of the instruction rather than from the ROM output
                    case v_op is
                        when OP_LDQ => sp_r <= signed(d_n);
                        when OP_LDF => sp_r <= signed(resize(unsigned(fun_q), 32));
                        when OP_LDP =>
                            if v_sel = 0 then sp_r <= signed(m_acc(31 downto 0));
                            else              sp_r <= signed(m_acc(47 downto 16)); end if;
                        when others =>
                            case v_sel is
                                when 0 to 5 => v_lx := resize(reg_q(to_integer(unsigned(d_arg(2 downto 0)))), 20);
                                when 6  => v_lx := resize(reg_q(7), 20);
                                when 7  => v_lx := resize(act_w, 20);
                                when 8  => v_lx := resize(act_h, 20);
                                when 9  => v_lx := resize(vstep_base, 20);
                                when 10 => v_lx := (0 => sw_flow, 1 => sw_full, 2 => ilace_q, 3 => fld_q, 4 => sw_mesh, others => '0');
                                when others => v_lx := (others => '0');
                            end case;
                            sp_r <= signed(resize(v_lx, 32));
                    end case;
                    cs_st <= to_unsigned(3, 2);
                when others =>
                    case v_op is
                    when OP_LD | OP_LDI | OP_ADD | OP_ADDI | OP_SUB | OP_NEG | OP_LDP | OP_LDQ | OP_LDF | OP_LDX =>
                        alu_a <= v_opa; alu_b <= v_opb; alu_c <= v_sub; alu_and <= '0'; alu_pend <= '1';
                    when OP_ABS  =>
                        alu_a <= v_opa; alu_b <= v_opb; alu_c <= v_sub; alu_and <= '0';
                        if acc < 0 then alu_pend <= '1'; end if;
                    when OP_ST   => rf_we <= '1'; rf_wa <= v_reg;
                    when OP_AND  => alu_a <= acc; alu_b <= v_rq; alu_and <= '1'; alu_pend <= '1';
                    when OP_SHR | OP_SHL | OP_SHRV | OP_SHLV =>
                        if sh_busy = '0' then
                            if v_op = OP_SHR or v_op = OP_SHL then sh_cnt <= unsigned(v_imm(5 downto 0));
                            else sh_cnt <= unsigned(v_rq(5 downto 0)); end if;
                            sh_busy <= '1';
                            v_done  := '0';
                        elsif sh_cnt /= 0 then
                            if v_op = OP_SHR or v_op = OP_SHRV then acc <= shift_right(acc, 1);
                            else                                     acc <= shift_left(acc, 1); end if;
                            sh_cnt <= sh_cnt - 1;
                            v_done := '0';
                        end if;
                    when OP_MUL =>
                        if sh_busy = '0' then
                            m_am  <= unsigned(acc);
                            m_acc <= x"00000000" & unsigned(v_rq(19 downto 0));
                            m_go <= '1';
                            sh_busy <= '1'; v_done := '0';
                        elsif busy = '1' then
                            v_done := '0';
                        end if;
                    when OP_DIV =>
                        if sh_busy = '0' then
                            d_n <= unsigned(acc); d_d <= unsigned(v_rq(15 downto 0)); d_go <= '1';
                            sh_busy <= '1'; v_done := '0';
                        elsif busy = '1' then
                            v_done := '0';
                        end if;
                    when OP_LINE0 => row0_req <= '1';
                    when OP_WAITF =>
                        if seq_req = '1' then seq_req <= '0'; else v_done := '0'; end if;
                    when OP_JMP  => v_jump := '1';
                    when OP_JZ   => if acc = 0 then v_jump := '1'; end if;
                    when OP_JNZ  => if acc /= 0 then v_jump := '1'; end if;
                    when OP_JN   => if acc < 0 then v_jump := '1'; end if;
                    when OP_JP   => if acc > 0 then v_jump := '1'; end if;
                    when OP_CALL => v_jump := '1'; lnk <= pc + 1; lnk2 <= lnk;
                    when OP_RET  => lnk <= lnk2;
                    when OP_ROM  => fun_addr <= unsigned(acc(8 downto 0));
                    when OP_TW   => tw_we <= to_unsigned(v_sel, 3);
                    when OP_STX =>
                        stx_val <= acc(20 downto 0); stx_sel <= to_unsigned(v_sel, 5); stx_en <= '1';
                        case v_sel is
                            when 10 => c_lp <= acc(17 downto 0);
                            when 11 => c_cdot <= acc(19 downto 0);
                            when 12 => c_up <= acc(19 downto 0);
                            when 13 => c_invn <= unsigned(acc(10 downto 0));
                            when 14 => c_inve <= unsigned(acc(2 downto 0));
                            when 15 => c_n <= acc(7 downto 0); c_nm1 <= acc(7 downto 0) - 1;
                            when 16 => c_t <= acc(10 downto 0);
                            when 17 => c_ph <= unsigned(acc(11 downto 0));
                            when 18 => c_fade <= unsigned(acc(7 downto 0));
                            when 24 => tw_addr <= unsigned(acc(8 downto 0));
                            when 25 => tw_d16 <= std_logic_vector(acc(15 downto 0));
                            when 26 => c_fl <= unsigned(acc(3 downto 0));
                            when 19 => c_a <= unsigned(acc(3 downto 0));
                            when others => null;
                        end case;
                    when others => null;
                    end case;

                    if v_done = '1' then
                        if v_op = OP_RET then     pc <= lnk;
                        elsif v_jump = '1' then  pc <= unsigned(v_imm(10 downto 0));
                        else                     pc <= pc + 1; end if;
                        cs_st   <= (others => '0');
                        sh_busy <= '0';
                    end if;
                end case;
            end if;
        end if;
    end process p_seq;

end architecture iris;
