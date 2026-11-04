#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
LATTICE v0.1.0 - build hook.  Writes lattice_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  Heirloom phase 3 (video clips), from the still WIN_20230401_00_20_03 at 0.5 s.

The patch, as fitted (see .heirloom_work/rb_lattice/notes.md):
  X(x)  = 1 - 2 g(u),  g(u) = u - 0.146 u (1 - u)      a shaped triangle: u = 0 at the big-diamond column (x 1008),
          1 at the small-diamond column, period 944.3 virtual px, held at its x = 66 value left of x = 66
  Y(y)  = 1.0095 (tri(y) + 0.4645 ramp(y))              the V triangle (peak at y 257.5, period 524.8 lines) mixed with
          the same oscillator's ramp (reset at y 410, softened over 38 lines); less (y - 600)^2 7e-7 at the bottom
  s     = X + Y.  Big diamonds where s > 1.62 (blue), fringe 1.18 .. 1.62 (yellow, orange, dark); small diamonds
          where s < -1.86, fringe -1.5 .. -1.86
  q     = (Tx + Ty) sign(dX/dx)                          the pure-triangle trellis (the diagonal net through every
          saddle), signed by the scan direction.  The ground shades across it (a bright edge, a dark crease) and
          the blades are the band |q + 0.041| < hq(y): hq = a parabola over the phase of a SECOND vertical
          oscillator (rate = the rows at the still), 0.27 at the centre of a 0.58-period window
  blade colour = the profile across the band from scan entry to exit (orange, dark, magenta, violet, lavender,
          white-pink), normalised to the band width

Package contents (also read by imodel.py):
  C_LT_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_LT_XROM  1024 x 12  X(u9) in 1/1024: [0..511] Diamond (shaped triangle), [512..1023] Bulb (smoothstep cosine)
  C_LT_PAL   1024 x 32  {Y, U, V, 00}: index = pal * 512 + region * 64 + i; regions 0-1 big-side fringe (i = 0 at the
                        fill edge, rising outward, 128 entries over 0.5 of s), 2-3 small-side fringe, 4 rings, 5 blade
                        profile, 6 ground (shaded across the trellis by q), 7 fill (by camera luma or gem depth)
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

CONSTS = {
    "XC": 1008.0643200000002,
    "PX": 944.2611199999996,
    "PY": 524.8000000000002,
    "YT": 257.49062399999997,
    "YR": 410.0,
    "K": 1.0095199999999998,
    "C": 0.46447999999999995,
    "E": -0.14628,
    "A24": 15923230,
    "STEPV": 17768,
    "KY": 31969,
    "PHY0": 8545555,
    "DR": 4875539,
    "SOFT0": 950,
    "PHI0": 2007639
}

XROM = [
    1022, 1019, 1015, 1012, 1009, 1005, 1002, 998, 995, 991, 988, 985, 981, 978, 974, 971,
    967, 964, 960, 957, 954, 950, 947, 943, 940, 936, 933, 929, 926, 922, 919, 915,
    912, 908, 905, 901, 898, 894, 891, 887, 884, 880, 877, 873, 870, 866, 863, 859,
    856, 852, 849, 845, 842, 838, 834, 831, 827, 824, 820, 817, 813, 810, 806, 803,
    799, 795, 792, 788, 785, 781, 778, 774, 770, 767, 763, 760, 756, 752, 749, 745,
    742, 738, 734, 731, 727, 724, 720, 716, 713, 709, 706, 702, 698, 695, 691, 687,
    684, 680, 677, 673, 669, 666, 662, 658, 655, 651, 647, 644, 640, 636, 633, 629,
    625, 622, 618, 614, 611, 607, 603, 600, 596, 592, 589, 585, 581, 577, 574, 570,
    566, 563, 559, 555, 551, 548, 544, 540, 537, 533, 529, 525, 522, 518, 514, 510,
    507, 503, 499, 495, 492, 488, 484, 480, 477, 473, 469, 465, 462, 458, 454, 450,
    446, 443, 439, 435, 431, 428, 424, 420, 416, 412, 409, 405, 401, 397, 393, 389,
    386, 382, 378, 374, 370, 367, 363, 359, 355, 351, 347, 344, 340, 336, 332, 328,
    324, 320, 317, 313, 309, 305, 301, 297, 293, 290, 286, 282, 278, 274, 270, 266,
    262, 258, 255, 251, 247, 243, 239, 235, 231, 227, 223, 219, 215, 212, 208, 204,
    200, 196, 192, 188, 184, 180, 176, 172, 168, 164, 160, 156, 152, 149, 145, 141,
    137, 133, 129, 125, 121, 117, 113, 109, 105, 101, 97, 93, 89, 85, 81, 77,
    73, 69, 65, 61, 57, 53, 49, 45, 41, 37, 33, 29, 25, 21, 17, 13,
    9, 5, 1, -4, -8, -12, -16, -20, -24, -28, -32, -36, -40, -44, -48, -52,
    -56, -60, -64, -69, -73, -77, -81, -85, -89, -93, -97, -101, -105, -109, -114, -118,
    -122, -126, -130, -134, -138, -142, -146, -151, -155, -159, -163, -167, -171, -175, -180, -184,
    -188, -192, -196, -200, -204, -209, -213, -217, -221, -225, -229, -234, -238, -242, -246, -250,
    -255, -259, -263, -267, -271, -275, -280, -284, -288, -292, -296, -301, -305, -309, -313, -318,
    -322, -326, -330, -334, -339, -343, -347, -351, -356, -360, -364, -368, -373, -377, -381, -385,
    -390, -394, -398, -402, -407, -411, -415, -419, -424, -428, -432, -437, -441, -445, -449, -454,
    -458, -462, -467, -471, -475, -479, -484, -488, -492, -497, -501, -505, -510, -514, -518, -523,
    -527, -531, -536, -540, -544, -549, -553, -557, -562, -566, -570, -575, -579, -583, -588, -592,
    -597, -601, -605, -610, -614, -618, -623, -627, -632, -636, -640, -645, -649, -654, -658, -662,
    -667, -671, -676, -680, -684, -689, -693, -698, -702, -706, -711, -715, -720, -724, -729, -733,
    -737, -742, -746, -751, -755, -760, -764, -769, -773, -778, -782, -786, -791, -795, -800, -804,
    -809, -813, -818, -822, -827, -831, -836, -840, -845, -849, -854, -858, -863, -867, -872, -876,
    -881, -885, -890, -894, -899, -903, -908, -912, -917, -921, -926, -930, -935, -940, -944, -949,
    -953, -958, -962, -967, -971, -976, -981, -985, -990, -994, -999, -1003, -1008, -1013, -1017, -1022,
    1024, 1024, 1024, 1024, 1024, 1023, 1023, 1023, 1022, 1022, 1021, 1021, 1020, 1020, 1019, 1018,
    1018, 1017, 1016, 1015, 1014, 1013, 1012, 1011, 1010, 1009, 1008, 1007, 1006, 1004, 1003, 1002,
    1000, 999, 997, 996, 994, 993, 991, 989, 988, 986, 984, 982, 980, 978, 976, 974,
    972, 970, 968, 966, 964, 962, 959, 957, 955, 952, 950, 947, 945, 942, 940, 937,
    935, 932, 929, 927, 924, 921, 918, 915, 912, 910, 907, 904, 901, 897, 894, 891,
    888, 885, 882, 878, 875, 872, 868, 865, 862, 858, 855, 851, 848, 844, 840, 837,
    833, 829, 826, 822, 818, 814, 811, 807, 803, 799, 795, 791, 787, 783, 779, 775,
    771, 767, 763, 758, 754, 750, 746, 741, 737, 733, 728, 724, 720, 715, 711, 706,
    702, 697, 693, 688, 684, 679, 674, 670, 665, 660, 655, 651, 646, 641, 636, 632,
    627, 622, 617, 612, 607, 602, 597, 592, 587, 582, 577, 572, 567, 562, 557, 552,
    546, 541, 536, 531, 526, 520, 515, 510, 505, 499, 494, 489, 483, 478, 472, 467,
    462, 456, 451, 445, 440, 434, 429, 423, 418, 412, 407, 401, 396, 390, 384, 379,
    373, 368, 362, 356, 351, 345, 339, 333, 328, 322, 316, 311, 305, 299, 293, 288,
    282, 276, 270, 264, 258, 253, 247, 241, 235, 229, 223, 218, 212, 206, 200, 194,
    188, 182, 176, 170, 164, 158, 152, 147, 141, 135, 129, 123, 117, 111, 105, 99,
    93, 87, 81, 75, 69, 63, 57, 51, 45, 39, 33, 27, 21, 15, 9, 3,
    -3, -9, -15, -21, -27, -33, -39, -45, -51, -57, -63, -69, -75, -81, -87, -93,
    -99, -105, -111, -117, -123, -129, -135, -141, -147, -152, -158, -164, -170, -176, -182, -188,
    -194, -200, -206, -212, -218, -223, -229, -235, -241, -247, -253, -258, -264, -270, -276, -282,
    -288, -293, -299, -305, -311, -316, -322, -328, -333, -339, -345, -351, -356, -362, -368, -373,
    -379, -384, -390, -396, -401, -407, -412, -418, -423, -429, -434, -440, -445, -451, -456, -462,
    -467, -472, -478, -483, -489, -494, -499, -505, -510, -515, -520, -526, -531, -536, -541, -546,
    -552, -557, -562, -567, -572, -577, -582, -587, -592, -597, -602, -607, -612, -617, -622, -627,
    -632, -636, -641, -646, -651, -655, -660, -665, -670, -674, -679, -684, -688, -693, -697, -702,
    -706, -711, -715, -720, -724, -728, -733, -737, -741, -746, -750, -754, -758, -763, -767, -771,
    -775, -779, -783, -787, -791, -795, -799, -803, -807, -811, -814, -818, -822, -826, -829, -833,
    -837, -840, -844, -848, -851, -855, -858, -862, -865, -868, -872, -875, -878, -882, -885, -888,
    -891, -894, -897, -901, -904, -907, -910, -912, -915, -918, -921, -924, -927, -929, -932, -935,
    -937, -940, -942, -945, -947, -950, -952, -955, -957, -959, -962, -964, -966, -968, -970, -972,
    -974, -976, -978, -980, -982, -984, -986, -988, -989, -991, -993, -994, -996, -997, -999, -1000,
    -1002, -1003, -1004, -1006, -1007, -1008, -1009, -1010, -1011, -1012, -1013, -1014, -1015, -1016, -1017, -1018,
    -1018, -1019, -1020, -1020, -1021, -1021, -1022, -1022, -1023, -1023, -1023, -1024, -1024, -1024, -1024, -1024]

PAL = [
    0x3c1d5d8c, 0x3b9d7d60, 0x3b1d9d34, 0x3a9dbd04, 0x39dddcd4, 0x395dfca4, 0x385e1c74, 0x37de3c44,
    0x371e5c10, 0x369e7bdc, 0x35de9bac, 0x355ebb78, 0x349edb44, 0x341efb10, 0x331f1adc, 0x329f3aa8,
    0x31df5a74, 0x315f7a40, 0x305f9a0c, 0x2fdfb9d4, 0x2f1fc9a0, 0x2e9fe968, 0x2ddff934, 0x2d601904,
    0x2ca038d8, 0x2c6058ac, 0x2be06884, 0x2b208858, 0x2ae0a834, 0x2ae0b820, 0x2ae0d80c, 0x2ae0e7f4,
    0x2ae107e0, 0x2ae117cc, 0x2be127c0, 0x2c6137b8, 0x2d6147ac, 0x2de157a4, 0x2ee17798, 0x30a1a78c,
    0x32a1d780, 0x34620774, 0x36623768, 0x3822675c, 0x3ca2e740, 0x41636724, 0x45e3e704, 0x4aa466e8,
    0x4f24e6cc, 0x57a5969c, 0x5fe65668, 0x68670634, 0x7127c600, 0x79a875cc, 0x8028f5a8, 0x85a95588,
    0x8ba9b568, 0x912a1548, 0x96aa7528, 0x9a6a9518, 0x9d2a7510, 0x9faa5508, 0xa1ea44fc, 0xa46a24f4,
    0xa7aa24ec, 0xab2a24e4, 0xaeaa24d8, 0xb26a34d0, 0xb5aa34c8, 0xb92a24c0, 0xbc29f4bc, 0xbf29c4b8,
    0xc1e994b4, 0xc52964b0, 0xc7a904b0, 0xc86824bc, 0xc96744cc, 0xcaa654d8, 0xcba574e4, 0xcc6494f0,
    0xcc63a500, 0xcca2b50c, 0xcd21b51c, 0xcd20c528, 0xcd5fd538, 0xcd5f0544, 0xcd5e3550, 0xcd9d7560,
    0xcd9ca56c, 0xcd9bd578, 0xcd9b4584, 0xcddac58c, 0xcdda4594, 0xce59c59c, 0xce5945a4, 0xce58e5a8,
    0xce58a5a8, 0xce9855a8, 0xce9815a8, 0xce97c5ac, 0xce97a5ac, 0xce9785b0, 0xce9775b0, 0xce9765b0,
    0xce9745b4, 0xce5735b4, 0xce5735b8, 0xce5725b8, 0xce5725bc, 0xce5725bc, 0xce5715bc, 0xce5715bc,
    0xce9715bc, 0xce9705bc, 0xced705b8, 0xced705b8, 0xced705b8, 0xced705b8, 0xced705b8, 0xced705b8,
    0xced705b8, 0xced705b8, 0xced705b8, 0xced705b8, 0xced705b8, 0xced705b8, 0xced705b8, 0xced705b8,
    0x405cbe7c, 0x409cbe7c, 0x409cce7c, 0x409cbe78, 0x411cbe78, 0x411cbe78, 0x411cae78, 0x415cae74,
    0x415cae74, 0x415cbe74, 0x415cbe74, 0x419cbe74, 0x419cce74, 0x411cde50, 0x405cfe20, 0x3fdd1df0,
    0x3f1d3dc0, 0x3e9d5d90, 0x3ddd7d5c, 0x3d5dad1c, 0x3c9dcce0, 0x3c1dfca4, 0x3b9e1c68, 0x3b1e4c2c,
    0x3a9e6bf0, 0x3a5e8bb4, 0x39debb78, 0x395edb3c, 0x391f0afc, 0x389f3ab8, 0x385f6a70, 0x381f9a2c,
    0x37dfd9e8, 0x37a009a4, 0x37a04960, 0x37a08920, 0x37a0c8e0, 0x37a108a0, 0x37e14860, 0x39618834,
    0x3ae1c808, 0x3c6207dc, 0x3de247b0, 0x3fe28780, 0x43e2e760, 0x48235740, 0x4c63c71c, 0x50a426fc,
    0x552496dc, 0x5b2526ac, 0x6265b680, 0x69264650, 0x6fe6e624, 0x76a775f4, 0x7c2815cc, 0x81a8b5ac,
    0x86e94588, 0x8c29e564, 0x91aa8540, 0x952af52c, 0x97eb3520, 0x9a6b7518, 0x9cabc50c, 0x9f2c0500,
    0xa1ec34f4, 0xa4ac34e8, 0xa7ec44dc, 0xaaec54d4, 0xadac64c8, 0xb06c54c0, 0xb32c34bc, 0xb5ac04b8,
    0xb82be4b4, 0xbaabb4b4, 0xbcab84b0, 0xbeab24b0, 0xc06ac4b4, 0xc2aa64b4, 0xc46a04b4, 0xc5e9a4b8,
    0xc6a8d4c4, 0xc72804d0, 0xc7e734d8, 0xc86674e4, 0xc925a4f0, 0xc9a4c4fc, 0xc9e3e508, 0xca62f514,
    0xcaa21520, 0xcb21352c, 0xcba07538, 0xcbdfb540, 0xcc1ef54c, 0xcc5e3554, 0xcc9d7560, 0xcd1cc56c,
    0xcd1c3574, 0xcd5b9580, 0xcd5b058c, 0xcd5a6598, 0xcd5a05a0, 0xcd99b5a4, 0xcd9965a8, 0xcd9915ac,
    0xcd98c5b0, 0xcd9885b4, 0xcd9855b4, 0xcd9825b4, 0xcd97f5b8, 0xcdd7c5b8, 0xcdd795b8, 0xcdd785b8,
    0xce5775b8, 0xce5765b8, 0xce5745b4, 0xce9735b4, 0xce5735b4, 0xce5735b4, 0xce5725b8, 0xce5725b8,
    0xce5725b8, 0xce5715b4, 0xce5715b4, 0xce9715b4, 0xce9715b4, 0xced705b4, 0xced705b4, 0xced705b4,
    0xced705c0, 0xcdd58634, 0xcd5406a4, 0xcc528718, 0xcb91078c, 0xca8f87fc, 0xc98e0870, 0xc88c88e4,
    0xc80b0954, 0xc689a9c8, 0xb90b9a40, 0xabcd7ab4, 0x9e4f6b2c, 0x91114ba0, 0x83932c18, 0x76551c8c,
    0x6896fd04, 0x5b18ed7c, 0x4f1acde4, 0x52dc6dbc, 0x565e1d94, 0x599fcd6c, 0x5d616d44, 0x60a31d1c,
    0x6464ccf8, 0x67e66cd0, 0x6ba81ca8, 0x6ee9ac7c, 0x726adc3c, 0x75abfbf8, 0x792d2bb8, 0x7cae4b74,
    0x802f7b34, 0x83b09af4, 0x86f1bab0, 0x8a72ea70, 0x8e73ba1c, 0x93b2d970, 0x9931f8c0, 0x9e710814,
    0xa3f02768, 0xa96f46b8, 0xaeae560c, 0xb42d755c, 0xb9ac94b0, 0xbaaba43c, 0xa9aa74a0, 0x99294508,
    0x88681570, 0x77a6e5d8, 0x66e5b640, 0x55e486a8, 0x45634710, 0x34621778, 0x2be0f7b8, 0x3c1ff788,
    0x4c5ef754, 0x5c9df720, 0x6d1cf6f0, 0x7d5bf6bc, 0x8d9af68c, 0x9dda0658, 0xae590624, 0xbe9805f4,
    0xc7a0c528, 0xc2a21514, 0xb8a4a4e8, 0xa7a614e4, 0x8f267510, 0x7726e578, 0x5fe75620, 0x54a8c6f8,
    0x55eb4800, 0x5d6d9904, 0x69efca08, 0x74714ae0, 0x7cb21b88, 0x81f24bf0, 0x8431dc20, 0x84711c40,
    0x82effc50, 0x80aebc58, 0x7dad6c50, 0x7aac2c40, 0x772b0c2c, 0x742a0c10, 0x71a93bec, 0x70288bc0,
    0x6fa7fb88, 0x70279b50, 0x71e76b18, 0x75674af0, 0x79a74ad4, 0x7ee76ac0, 0x8567aabc, 0x8be7fac0,
    0x92a84acc, 0x99a8bad8, 0xa0694ae4, 0xa6a9daec, 0xac6a8af0, 0xb16b2af0, 0xb5abcaec, 0xb96c5ae4,
    0xbbecead8, 0xbded4acc, 0xbfad7ac4, 0xc06d9abc, 0xc12dbab8, 0xc12dcab8, 0xc06ddabc, 0xbfedfac0,
    0xbeee1ac8, 0xbd2e5ad4, 0xbaaebae8, 0xb7af2afc, 0xb3efbb18, 0xaff04b38, 0xabf0db54, 0xa9310b68,
    0xa7f0cb70, 0xa86f2b50, 0xab2c4b04, 0xb0a8da6c, 0xb8a4d98c, 0xbf2178ac, 0xc45e87cc, 0xc6dd175c,
    0xced7d5b4, 0xced7d5b4, 0xced7d5b4, 0xced7e5b4, 0xced7e5b4, 0xced7e5b4, 0xced7e5b4, 0xced7f5b4,
    0xced7f5b4, 0xced7f5b4, 0xced805b4, 0xced805b4, 0xced805b4, 0xced805b4, 0xced815b4, 0xced815b4,
    0xced815b4, 0xced815b0, 0xced825b0, 0xced825b0, 0xced825b0, 0xcf1835b0, 0xcf1835b0, 0xcf1835b0,
    0xcf1835b0, 0xcf1845b0, 0xcf1845b0, 0xcf1845b0, 0xcf1845b0, 0xcf1855b0, 0xced805b4, 0xced735bc,
    0xced675c8, 0xced5d5d0, 0xced5d5d0, 0xced5e5d0, 0xced5e5d0, 0xced5f5d0, 0xced5f5cc, 0xced605cc,
    0xced605cc, 0xced615cc, 0xced625cc, 0xced625cc, 0xced635cc, 0xced635cc, 0xced645c8, 0xced655c8,
    0xced655c8, 0xced665c8, 0xced675c8, 0xced675c8, 0xced675c8, 0xced675c8, 0xced675c8, 0xced675c8,
    0xced675c8, 0xced675c8, 0xced675c8, 0xced675c8, 0xced675c8, 0xced675c8, 0xced675c8, 0xced675c8,
    0x3fdc6e80, 0x465c9e4c, 0x4adcae24, 0x4f1cce04, 0x531cdde4, 0x56dcfdc4, 0x5a9d0da4, 0x5e5d2d88,
    0x619d3d6c, 0x651d4d50, 0x681d5d38, 0x6b9d6d1c, 0x6e9d8d04, 0x719d9ce8, 0x749dacd0, 0x77ddbcb8,
    0x7a9dcca0, 0x7d9ddc88, 0x805dec70, 0x839dfc58, 0x865e0c40, 0x891e1c2c, 0x8bde2c14, 0x8e9e3bfc,
    0x915e4be8, 0x941e5bd0, 0x969e6bbc, 0x995e7ba4, 0x9bde8b90, 0x9e9e9b78, 0xa11eab64, 0xa3debb50,
    0xa65ecb38, 0xa91edb24, 0xab9eeb10, 0xaddefafc, 0xb05f0ae8, 0xb2df1ad4, 0xb55f2ac0, 0xb7df3aac,
    0xba5f4a94, 0xbc9f5a80, 0xbf1f5a70, 0xc19f6a5c, 0xc41f7a48, 0xc69f8a34, 0xc91f9a20, 0xcb1faa0c,
    0xcd9fb9f8, 0xd01fc9e4, 0xd25fd9d0, 0xd49fd9c0, 0xd71fe9ac, 0xd95ff998, 0xdbe00984, 0xdde01974,
    0xe0602960, 0xe2a0394c, 0xe520393c, 0xe7204928, 0xe9605914, 0xeb206904, 0xeb2078f0, 0xeb2088e0,
    0x341f7a48, 0x33dfca2c, 0x33200a10, 0x32a049f0, 0x31a099d0, 0x3120d9b4, 0x30211994, 0x2fa15974,
    0x2ea19954, 0x2d61d934, 0x2c621914, 0x2ba248f0, 0x2a6288d0, 0x2922b8b0, 0x27e2e88c, 0x26a32868,
    0x25235848, 0x24235834, 0x23e34834, 0x23a33834, 0x23232834, 0x22a31830, 0x22630830, 0x21e2f830,
    0x21a2e830, 0x2162e82c, 0x2122d82c, 0x20a2c82c, 0x2062b82c, 0x2062b82c, 0x2062b82c, 0x2062b82c,
    0x2062b82c, 0x2062b82c, 0x2122d82c, 0x2162e82c, 0x21e2f830, 0x22630830, 0x22e32830, 0x24235834,
    0x25238838, 0x2663b83c, 0x2723e83c, 0x28640840, 0x2b248848, 0x2de4f850, 0x31257858, 0x33e5f860,
    0x36a66864, 0x3ba73874, 0x40a81880, 0x45e8f890, 0x4b29d89c, 0x506aa8ac, 0x542b58b4, 0x57ebe8bc,
    0x5b2c78c8, 0x5ead08d0, 0x61ed98d8, 0x646df8e0, 0x65ee48e4, 0x67ae88e8, 0x692ec8ec, 0x6aaf08f0,
    0x6c6f58f4, 0x6eafb8fc, 0x70b00900, 0x72f06904, 0x7530c90c, 0x76f11910, 0x78b16914, 0x7ab1b91c,
    0x7c71f920, 0x7e724924, 0x7ff28928, 0x8072a928, 0x8132b92c, 0x8172d92c, 0x81f2f930, 0x82b30930,
    0x82f31930, 0x82f31930, 0x8472e914, 0x8c31d86c, 0x93b0c7c4, 0x99efd738, 0xa06ee6ac, 0xa6ae061c,
    0xad2d1590, 0xb3ac2504, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58,
    0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58, 0x345f5a58,
    0x345f5a58, 0x345f7a48, 0x33dfda28, 0x33203a00, 0x326089dc, 0x31a0e9b4, 0x30a14990, 0x2fe19968,
    0x2ee1f944, 0x2e62491c, 0x2d2298f4, 0x2c22e8d0, 0x2b2348a8, 0x29e3a878, 0x2923f84c, 0x28641840,
    0x28640840, 0x28240840, 0x28240840, 0x28240840, 0x28240840, 0x28240840, 0x28240840, 0x29643844,
    0x2a645844, 0x2b248848, 0x2c24a84c, 0x2d24d84c, 0x2fa54854, 0x3265b85c, 0x34a61860, 0x37a68868,
    0x39e6f870, 0x3de7a878, 0x41e85884, 0x46690890, 0x4a69b89c, 0x4eaa68a4, 0x51eaf8b0, 0x552b78b8,
    0x586c08c0, 0x5bac98c8, 0x5eed18d0, 0x612d78d8, 0x62adb8dc, 0x642df8e0, 0x65ae38e4, 0x672e78e8,
    0x68aec8ec, 0x6aaf08f0, 0x6c6f58f4, 0x6e6fa8f8, 0x702ff900, 0x71b03904, 0x73307908, 0x74b0b90c,
    0x7670f910, 0x77f13914, 0x79717918, 0x7a71a918, 0x7bb1d91c, 0x7cb20920, 0x7db23924, 0x7ef26924,
    0x7f327928, 0x7fb28928, 0x80329928, 0x8072a928, 0x80b2b92c, 0x8132c92c, 0x8172c92c, 0x8172d92c,
    0x81b2e92c, 0x873228b8, 0x8db15830, 0x943087ac, 0x9a6fa728, 0xa06ed6a0, 0xa6adf61c, 0xabad45ac,
    0xb06c9544, 0xb52be4dc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc, 0xb6abb4bc,
    0x999c9ae8, 0x979f0b04, 0x95a16b1c, 0x93a3db34, 0x91a64b4c, 0x8fa8bb64, 0x8dab2b7c, 0x8bad9b94,
    0x89b00bb0, 0x87b25bc0, 0x85730b24, 0x8333ba88, 0x80b469ec, 0x7eb51950, 0x7c75c8b4, 0x79f6781c,
    0x77f72780, 0x75b7d6e4, 0x74385650, 0x7c36e608, 0x843575c4, 0x8c340580, 0x9432953c, 0x9c3114f8,
    0xa42fa4b0, 0xac2e346c, 0xb42cc428, 0xbbeb53e8, 0xc0a9c3c8, 0xc5e833ac, 0xcb26a390, 0xd0252370,
    0xd5639354, 0xdaa20334, 0xdfa07318, 0xe49ef2fc, 0xe85d52f8, 0xe41b5378, 0xe01953f8, 0xdbd75478,
    0xd7d544f8, 0xd3934578, 0xcf9145f8, 0xcb0f467c, 0xc70d46fc, 0xc04c1764, 0xaf0e3774, 0x9dd05780,
    0x8c92678c, 0x7b94879c, 0x6a96a7a8, 0x5958b7b8, 0x481ad7c4, 0x36dcf7d4, 0x2c5e87f0, 0x379e583c,
    0x425e2888, 0x4d5de8d4, 0x581db920, 0x62dd896c, 0x6ddd59b8, 0x789d2a04, 0x83dcfa50, 0x8e9cca9c,
    0xc7a0c528, 0xc2a21514, 0xb8a4a4e8, 0xa7a614e4, 0x8f267510, 0x7726e578, 0x5fe75620, 0x73225744,
    0xb117e8e4, 0xd252f9a8, 0xd5d36994, 0xd953c984, 0xdbd41978, 0xdd544970, 0xddd4596c, 0xddd4696c,
    0xdd945970, 0xdd143974, 0xdc142978, 0xdad4097c, 0xd9d3e980, 0xd913c984, 0xd853b988, 0xd7d3a98c,
    0xd7d3998c, 0xd7d3a98c, 0xd853b988, 0xd953d984, 0xda93f97c, 0xdc542974, 0xde54696c, 0xe014a960,
    0xe254e958, 0xe455294c, 0xe6956944, 0xe855a938, 0xe9d5d930, 0xeb160928, 0xeb163920, 0xeb16591c,
    0xeb166918, 0xeb167914, 0xeb168914, 0xeb169910, 0xeb169910, 0xeb169910, 0xeb169910, 0xeb168914,
    0xeb168914, 0xeb167918, 0xeb165918, 0xeb164920, 0xeb161924, 0xeb15f92c, 0xe9d5d930, 0xe915b934,
    0xe895a938, 0xe915b934, 0xe995c930, 0xeb160928, 0xeb16491c, 0xeb168914, 0xeb16b90c, 0xeb16d908,
    0xbb2c0480, 0xbb2c0480, 0xbb2c047c, 0xbbac047c, 0xbbac047c, 0xbbac047c, 0xbbec0478, 0xbbec1478,
    0xbbec1478, 0xbc2c1474, 0xbc2c1474, 0xbc2c1474, 0xbc2c1470, 0xbc6c1470, 0xbc6c1470, 0xbc6c146c,
    0xbcac146c, 0xbcac246c, 0xbcac246c, 0xbd2c2468, 0xbd2c2468, 0xbd2c2468, 0xbd6c2464, 0xbd6c2464,
    0xbd6c2464, 0xbd6c2460, 0xbdac3460, 0xbdac3460, 0xbdac345c, 0xbdec345c, 0xbc2c1474, 0xb7ebc4ac,
    0xb3ab74e4, 0xb02b4510, 0xb02b4510, 0xb06b450c, 0xb06b450c, 0xb0ab4508, 0xb0ab4508, 0xb12b5504,
    0xb16b5500, 0xb16b54fc, 0xb1ab54fc, 0xb1eb64f8, 0xb1eb64f4, 0xb26b64f4, 0xb2ab64f0, 0xb2eb74ec,
    0xb2eb74ec, 0xb32b74e8, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4,
    0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4, 0xb3ab74e4,
    0x345f5a58, 0x3b9f8a34, 0x409fba18, 0x455fda04, 0x49dff9ec, 0x4de019d8, 0x51e039c4, 0x55e059b0,
    0x59a0799c, 0x5d608988, 0x60a0a978, 0x6460c964, 0x67e0e954, 0x6b20f944, 0x6ea11934, 0x71e12920,
    0x75214910, 0x78615900, 0x7ba178f0, 0x7ea188e0, 0x81a1a8d0, 0x84a1b8c4, 0x87e1d8b4, 0x8ae1e8a4,
    0x8de20894, 0x90a21884, 0x93e23878, 0x96a24868, 0x99a25858, 0x9c62784c, 0x9f22883c, 0xa262a830,
    0xa522b820, 0xa7e2c810, 0xaaa2e804, 0xad62f7f4, 0xb02307e8, 0xb2e327dc, 0xb5a337cc, 0xb86347c0,
    0xbae357b0, 0xbda377a4, 0xc0638798, 0xc2e39788, 0xc5a3b77c, 0xc823c770, 0xcae3d760, 0xcd63e754,
    0xd0240748, 0xd2a41738, 0xd564272c, 0xd7e43720, 0xda644714, 0xdd246708, 0xdfa476f8, 0xe1e486ec,
    0xe46496e0, 0xe6e4b6d4, 0xe9a4c6c8, 0xeb24d6b8, 0xeb24e6ac, 0xeb24f6a0, 0xeb251694, 0xeb252688]

EXP2 = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def write_pkg(path):
    L = []
    L.append('-- Generated by lattice.py: do not edit.')
    L.append('library ieee;')
    L.append('use ieee.std_logic_1164.all;')
    L.append('use ieee.numeric_std.all;')
    L.append('package lattice_pkg is')
    L.append('    type t_lt_exp2 is array (0 to 255) of unsigned(15 downto 0);')
    L.append('    type t_lt_xrom is array (0 to 1023) of signed(11 downto 0);')
    L.append('    type t_lt_pal  is array (0 to 1023) of std_logic_vector(31 downto 0);')
    L.append('    constant C_LT_EXP2 : t_lt_exp2 := (')
    L.append(',\n'.join('        to_unsigned(%d, 16)' % v for v in EXP2) + ');')
    L.append('    constant C_LT_XROM : t_lt_xrom := (')
    L.append(',\n'.join('        to_signed(%d, 12)' % v for v in XROM) + ');')
    L.append('    constant C_LT_PAL : t_lt_pal := (')
    L.append(',\n'.join('        x"%08x"' % v for v in PAL) + ');')
    L.append('end package lattice_pkg;')
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    write_pkg(os.path.join(HERE, 'lattice_pkg.vhd'))
    print('lattice.py: wrote lattice_pkg.vhd')
