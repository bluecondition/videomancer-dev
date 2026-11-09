#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CATEYE v0.1.1 - build hook.  Writes cateye_pkg.vhd next to this file (pure python: the builder runs it with the
system python3; the proto / integer model in .heirloom_work/rb_cateye import the same tables).

Fitted to the still WIN_20240102_00_30_49 at 1.0 s (see .heirloom_work/rb_cateye/notes.md):

  u     = x' - T(y) w(x')      the horizontal ramp zoomed by a vertical triangle (a multiplier):
          w = max(0.306 (476 - x), min(0.3135 (x - 476), 49.3 + 0.115 x))   (virtual px)
          T = TL (left of x = 476) / TR, measured per row (soft-kneed triangles, apex on line 540.5)
  walls = left d = 390 - u, right d = u - 1062 (px)  -> palettes L / R (the still's top row, read outward)
  cream = 390 < u < 1062 -> palette D by u (the top row), with the socket dip d = 0..53 px (u 620..1014)
  eye   = r = max(((dx/416)^2 + (dy/228)^2)^(1/2), (dx + 0.635 |dy|) / 143.4) about (1002, 537)
          phi(r) = pupil 100 .. pupil line 160 (r .215) .. moat 250 (r .55) .. rim line 160 (r .845) ..
          dark edge .. 0 (r 1.25);  d_E = G base(r) + Gm moat bump(r)  -> palette E (rings beyond 182)
  halo  = a pink-red glow (255, 18, 84) outside the eye, by alpha: shown where it is deeper than the background
  stripes = 99.2-line square wave: 'blue' rows recolour the line bands

Package contents (index widths as the RTL uses them):
  C_CE_PY/PU/PV  2048 x 8   palette Y (pre-scaled 16/13, capped 940) / U (= Cr) / V (= Cb), >> 2
                            address = parity & fam(2) & idx(8); fam 0 = L, 1 = R, 2 = D, 3 = E
                            L/R: idx = d / 2 (px); D: idx = (u - 390) / 4 (px); E: idx = d / 2 below 182,
                            91 + ring phase (16 entries per 32 px) beyond (192 / 208 / 224 + phase: the darker
                            cycles of Rings above 640), 128 + alpha / 2 for the glow
  C_CE_SQ        1024 x 8   r * 128 = sqrt(i / 256) * 128 (saturating at 255)
  C_CE_PHI       256 x 16   base(r) (min(phi, 182)) & bump(r) (phi - 182 >= 0), by r * 128
  C_CE_T         512 x 16   TL & TR (Q7) by |y - 540.5| in 2-line steps
Colours are in the hardware convention (U register = Cr, V register = Cb).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model
Y0 = 540.5
XW0 = 476.0
WL, WM, WR0, WR1 = 0.306, 164.3 / 524.0, 49.3, 0.115
XL0, XR0 = 390.0, 1062.0
DIP = (620.0, 966.0, 1014.0, 53.0)
EYE_CX, EYE_CY, EYE_A, EYE_B, EYE_K, EYE_C = 1002.0, 537.0, 416.0, 228.0, 0.635, 143.4
ST_P, ST_PH = 99.2, 82.0
TL_K = [(0, 1.0), (40, .95), (72, .87), (88, .80), (108, .73), (128, .65), (144, .58), (164, .50), (184, .42),
        (204, .36), (220, .30), (236, .25), (256, .20), (276, .155), (292, .13), (312, .09), (340, .06),
        (380, .035), (420, .012), (460, 0.0), (1100, 0.0)]
TR_K = [(0, 1.0), (40, .95), (64, .85), (80, .78), (104, .66), (128, .56), (144, .50), (164, .41), (180, .33),
        (200, .25), (220, .19), (240, .14), (256, .10), (276, .06), (290, .04), (304, .02), (316, .01),
        (340, 0.0), (1100, 0.0)]
SH_K = [(0, 0.95), (60, 0.94), (100, 0.91), (150, 0.84), (190, 0.76), (230, 0.66), (265, 0.62), (300, 0.66),
        (340, 0.78), (380, 0.86), (420, 0.93), (460, 0.98), (500, 1.0), (1100, 1.0)]
SHR_K = [(0, 0.80), (100, 0.78), (150, 0.74), (190, 0.68), (230, 0.56), (265, 0.50), (300, 0.55), (340, 0.66),
         (400, 0.85), (450, 0.95), (500, 1.0), (1100, 1.0)]
PHI_K = [(0.0, 100), (0.075, 104), (0.1, 108), (0.125, 118), (0.15, 126), (0.215, 160), (0.55, 250),
         (0.845, 160), (0.9, 132), (0.95, 118), (1.25, 0), (9.0, 0)]
E_PER = 32.0
E_PK = [(0, (255, 140, 150)), (40, (255, 120, 140)), (100, (255, 49, 123)), (108, (255, 37, 115)),
        (118, (242, 9, 73)), (126, (226, 4, 44)), (134, (180, 0, 30)), (146, (215, 10, 14)), (153, (252, 110, 35)),
        (160, (255, 172, 72)), (167, (252, 110, 35)), (174, (222, 0, 14)), (182, (212, 0, 24))]
E_PKB = [(0, (255, 140, 150)), (40, (255, 120, 140)), (100, (255, 40, 118)), (108, (250, 20, 112)),
         (118, (200, 12, 140)), (126, (150, 25, 150)), (134, (100, 40, 190)), (146, (60, 110, 225)), (153, (45, 180, 240)),
         (160, (45, 214, 243)), (167, (45, 180, 240)), (174, (70, 60, 200)), (182, (115, 0, 130))]
E_RED, E_MAR = (212, 0, 24), (128, 0, 16)
RING_HI = [((220, 0, 22), (92, 0, 14)), ((228, 0, 20), (56, 0, 14)), ((236, 8, 20), (24, 0, 16))]   # (red, dark) per cycle
HALO = (255, 18, 84)
HALO_BG = (247, 238, 140)      # the cream the glow was measured over

# the still's top row (median of 'red' and 'blue' stripe rows 36..228 and 886..1068), every 2 px, RGB hex
TOP_RED = (
    "ba0352f0006dff007ef50088f40096ff009cff0096ff008eff008eff008bff0089ff0087ff0085ff0083ff0083ff0080"
    "ff007eff007bff0078ff0076ff0073ff0070ff006efe006cfc0069f90067f80065f60063f40062f00060ee005eec005b"
    "e8005ae40057e20054dd0051db004ed6004ad10046ce0044ca0043c60040c4003dc2003cc1003abe0038be0037be0037"
    "c00037c00034c10034c10034c20032c40030c80031ca0031ce0030d1002ed6002dd9002ddb002cdf002be3002ae8002b"
    "ec002aef002af30029f50029f60029f70028f80027f90026f80025f90023f80022f80020f7001ef6001bf60019f60017"
    "f50016f50015f40013f30011f20011f2000ff1000df0000ced000be6000ae50009e20008e70009ec000bf1000ef30013"
    "f50017f7001bf7001ff80022f90024f90026f90028f80028f50027f60025f80020f9001cfa0019fa0017fd0115ff0a16"
    "ff1b18ff2a1cff3a1eff4616ff4d0eff4e03ff4b00ff4600ff3e00ff3000fa2100e91100d60600ca0200c30003c30007"
    "bf0015c1001ac60022d4002ae00034f0003dfb0040ff024aff0654ff0c5aff1261ff1769ff1c6eff2272ff2875ff2c78"
    "ff317aff367cff3a7eff4080ff4480ff4b81ff5083ff5685ff5a85ff6086ff6587ff6b89ff708aff768aff7a8aff808e"
    "ff858eff8a8fff8f8ffe9490fe9892fe9d94fda294fda795fcac95fbb095fbb496fbb996fbbd97fac198fac598fac99b"
    "facc9cf8d09cf9d39cf8d79cf8da9cf8dc9ef7de9ff8e19ef7e39ef7e59ff7e69ff7e89ff7e99ff6ea9ef6eb9ef7ec9e"
    "f6ed9ef7ed9ff7ee9ff7ee9ff7ef9ef6ef9ef7f09ef7f09ef6f09ef6f19ef6f19ef6f19df6f19df5f19df5f29df5f29d"
    "f5f29cf5f29af5f29af5f29af6f29cf5f29cf5f29cf5f29af6f399f6f399f6f398f6f398f6f398f6f398f6f398f6f399"
    "f6f397f6f397f6f397f6f397f6f396f6f396f6f396f6f396f5f396f5f396f5f396f5f396f6f395f6f395f6f395f6f394"
    "f5f394f5f493f5f493f5f493f6f492f6f492f6f492f6f492f6f391f6f391f6f390f6f390f6f392f6f392f6f392f6f390"
    "f5f390f5f48ff5f48ff5f48ff6f38ef6f38ef6f38ef6f38ef6f38ef6f48ef6f48ef6f48ef6f38ef6f38df6f38df6f38d"
    "f6f38cf6f48cf6f48cf6f48af6f38af6f48af6f48af6f48af6f389f6f389f6f389f6f489f6f489f6f489f6f489f6f489"
    "f6f388f6f388f6f388f6f388f6f388f6f386f6f386f6f386f6f386f6f384f6f384f6f384f6f385f6f385f6f385f6f386"
    "f6f386f7f387f7f387f7f386f7f388f7f388f7f387f7f387f7f387f7f386f7f386f7f388f7f288f7f288f7f288f7f288"
    "f7f286f7f286f7f286f7f286f7f286f7f286f7f286f7f288f7f288f7f288f7f288f7f288f7f289f7f288f7f289f7f28a"
    "f7f188f7f187f7f188f7f188f7f289f7f189f7f189f7f18af7f188f7f188f7f188f7f188f6f088f6f089f6f089f6f08a"
    "f6f08af6f08af6f08af6f08bf7ef88f7ef88f7ef88f7ef89f6ee8af6ed8cf6ed8cf7ed8cf7ed89f7ed8af7ed8af7ed8a"
    "f7ea88f7ea88f7ea88f7e988f7e988f7e888f7e788f7e787f8e588f8e588f8e586f8e488f8e388f8e388f9e288f9e288"
    "f9e089f9df88f9df88f9de8af9dd8af9dc8af9db8afadb8af9da88f9d987f9d887f9d788fad688fad589fad489fad38a"
    "fad28afad18afad18afad18afbd08afbcf8bfbcf8cfbcf8cfbce8dfbcd8cfbcd8cfbcc8cfbcd8bfbcc8cfbcb8cfbca8d"
    "fbc98efbc98efbc98efbc98efbc88ffbc88ffbc88ffcc78ffcc78efcc68efcc78efcc68efbc58ffbc58efbc48efcc48f"
    "fcc390fcc290fcc290fcc190fcc191fcc090fcc090fdbf90fdbe92fcbe91fcbe91fdbd92fcbc91fdbc91fdbc91fdbb92"
    "fdba92fdba92fdba92fdb992fdb992fdb892fdb892fdb893feb793feb792fdb792fdb792fdb692fdb692fdb692fdb693"
    "feb494feb494feb494fdb494fdb393fdb392fdb392feb392feb292feb192feb192feb191fdb090fdb090fdb090feaf92"
    "feaf93feaf93feaf92fdaf92feaf92feaf93feaf93feaf93feaf93feaf92feb093fdb193fcb293fcb293fdb494fdb695"
    "fcb895fbbb95fbbd95fbc297f9c796f8cc96f7d298f7d999f6e19cf6e79bf6eb9ef6ee9ff6ef9ff5f19ef6f19ef6f29e"
    "f6f29ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39ef6f39e"
    "f7f29ef7f19cf7f19cf6f09cf7ef9cf7ed9cf6ec9cf8ea9cf8e79cf8e49cfae09bfbdb9bfcd79bfdce98ffc494ffba91"
    "ffad8effa187ff9785ff8c82ff8182ff7680ff6e80ff637cff5b78ff5175ff4972ff406eff386cff3167ff2a66ff2163"
    "ff1c60fd185cf2124ef20c43f60e3afb1835ff222fff2628ff2a18ff2710ff2606ff2200fe1e00f81b00f01700f01500"
    "ee1500e91600e61600e01700d61704ca1407c01010b00a13a4001c98001b8c001b82001b79001c7000196c0016660015"
    "6200145d00126000116200136300146800157100167b001c87001e9200219900219d00249c00249c00259a0026980027"
    "9600289300259200259200259200269100269100269000268f00268e00268d00278d00288e00268e00268e00268e0026"
    "8e00278f002790002792002893002994002a95002a99002b99002c9c002c9e002da0002ea3002fa60030a80031ac0032"
    "ae0033b20033b60035ba0037be0038c20039c6003bcb003dd0003ed50040dc0043e10045e40048ea004af0004df60051"
    "f80053fa0056fc0058fe005cff005fff0062ff0066ff006aff006cff0071ff0073ff0077ff007aff007eff0081ff0085"
    "ff0088ff008bff008eff0092ff0095ff0098ff009bff009cff009cff009cff009cff009cff009dff009cff009eff009e"
    "ff009eff009eff009fff009eff009eff009eff009eff00a0ff00a0ff00a0ff00a1ff00a2ff00a1ff00a1ff00a2ff00a2"
    "ff00a2ff00a4ff00a4ff00a4ff00a2ff00a2ff00a3ff00a4ff00a4ff00a6ff00a6ff00a6ff00a7ff00a8ff00a8ff00a8"
    "ff00a9ff00a8ff00aaff00aaff00a8ff00a9ff00a9ff00aaff00abff00acff00acff00acff00abff00abff00abff00ac"
    "ff00adff00aeff00aeff00aeff00aeff00afff00afff00aeff00aeff00aeff00aeff00aeff00aeff00aeff00aeff00ae"
    "ff00b0ff00b0ff00b0ff00b0ff00b0ff00b0ff00b2ff00b1ff00b2ff00b3ff00b3ff00b3ff00b3ff00b2ff00b2ff00b3"
    "ff00b4ff00b4ff00b4ff00b4ff00b4ff00b4ff00b4ff00b6ff00b6ff00b8ff00b8ff00b8ff00b8ff00b8ff00b8ff00b8"
    "ff00b6ff00b7ff00b7ff00b7ff00b7ff00b7ff00b8ff00b8ff00b6ff00b6ff00b6ff00b6ff00b6ff00b7ff00b7ff00b7"
    "ff00b8ff00b8ff00b8ff00b8ff00b9ff00baff00baff00baff00baff00baff00baff00baff00baff00b9ff00b9ff00bb"
    "ff00bcff00bdff00bdff00bdff00bdff00bdff00bdff00bdff00beff00beff00bdff00beff00beff00beff00beff00be"
    "ff00beff00bdff00beff00beff00beff00beff00bdff00beff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00bf"
    "ff00bfff00bfff00c0ff00bfff00c0ff00bfff00c0ff00c0ff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00c0"
    "ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0"
    "ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0"
    "ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0ff00c0"
    "ff00c0ff00bfff00bfff00c0ff00bfff00bfff00bfff00bfff00beff00beff00beff00bdff00beff00beff00beff00be"
    "ff00beff00beff00bfff00bdff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00beff00beff00beff00bf"
    "ff00bfff00bfff00bfff00bfff00beff00beff00beff00beff00bcff00bcff00bfff00beff00beff00beff00b9ff00ae"
)
TOP_BLUE = (
    "bd0254f1006eff007ff7008af40098ff009eff0098ff0091ff008fff008cff008aff0089ff0087ff0085ff0082ff0080"
    "ff007fff007bff0079ff0076ff0074ff0071ff006ffe006dfd006afc0067fb0066f90063f60061f30060f1005fee005c"
    "e70059e20056dd0052d90050d6004dcf0048cd0045c80042c40042bf003ebd003cbb003ab80037b60036b50035b60035"
    "b30033b30032b40030b50030b5002fb6002eb8002dbb002cbc002cbf002ac20029c60028c8002acc0029d00029d20028"
    "d90028df0028e00028e40029ec002af10029f10029f40029f50028f60028f50028f40025f40022f40020f3001ef2001c"
    "f1001aee0017ed0016eb0014e80014e50012e30010e1000fdc000dda000cd7000bd5000bd6000bdb000de3000fed0014"
    "f00016f2001bf3001ef50022f50024f60027f30029ed002ae00030ce003fb90052ae00729e00948211a7722bb66c3ec0"
    "6c52d25a70de468ce74298ea2da6f217aff608b0f809a4f40496fb147ef51a67ee214feb3234d64c1cc2630cae6e05a4"
    "82008b9a0080ae0070bc005ece0058e60054f60057fa075bff0d5eff1362ff1868ff1e6dff2371ff2975ff2e77ff327a"
    "ff377bff3b7dff407fff4681ff4b82ff4f83ff5485ff5a86ff5f87ff6688ff6a89ff708bff758cff7a8dff808eff8490"
    "ff8890ff8d91fe9292fe9793fe9c94fda196fda497fcaa98fcae98fbb399fbb79afabb9afabf9bfac39cf9c79cf9cb9d"
    "f8ce9ef8d29ff8d59ef8d89ef8dba0f7dda0f7e0a0f7e3a2f7e5a1f6e6a1f6e8a1f6e8a1f7eaa1f6eba0f6eca1f7eda1"
    "f7eda1f6eea2f6eea2f6eea2f6efa2f6efa2f6f0a0f6f0a1f6f1a1f5f1a1f5f1a1f5f1a1f6f19ff6f19ff6f19ff6f19f"
    "f5f2a1f5f2a3f5f2a3f5f2a3f5f2a1f5f2a2f5f2a1f5f2a2f5f2a2f5f3a0f5f39df5f39df5f39df5f39df5f39df5f39d"
    "f5f39df5f39df5f39df5f39df5f39cf5f39cf5f39cf5f39cf5f39bf5f39bf5f39bf5f39bf6f39bf6f39bf6f39bf6f39b"
    "f5f39af5f39bf5f39bf5f39bf5f39bf5f39bf5f39bf5f39af6f398f6f398f6f398f6f398f6f39af5f39af5f39af6f39a"
    "f5f398f6f397f6f397f5f396f6f396f6f396f6f396f6f396f6f396f6f396f6f396f6f395f6f398f6f398f6f398f6f398"
    "f6f393f6f393f6f393f6f393f6f393f6f393f6f393f6f391f6f38ff6f38ff6f38ff6f38ff6f38ff6f38ff6f38ff6f38f"
    "f6f391f6f391f6f391f6f38ff6f38cf6f38bf6f38bf6f38bf6f38cf6f38cf6f38cf6f38bf6f38bf6f389f6f389f6f38b"
    "f6f389f6f389f6f389f6f38bf5f388f5f388f5f388f5f388f5f386f5f388f5f388f6f388f6f388f6f388f6f388f6f388"
    "f6f28af6f28af6f28af6f28af7f28af7f28af7f28af7f288f6f284f6f282f6f282f6f282f7f282f6f282f6f282f7f284"
    "f6f185f6f185f6f185f6f185f7f185f7f185f7f185f7f185f7f188f7f188f7f188f7f188f6f089f6f087f6f087f6f087"
    "f6ef89f6ef89f6ef89f6ef89f6ef89f6ef89f6ef89f6ef89f6ee89f6ee89f6ee89f6ee89f6ee89f6ee89f6ee89f6ed8b"
    "f7ec8cf7ec8df7ec8cf7eb8bf7eb8af7ea88f7ea88f8e989f8e789f8e788f8e788f9e687f8e685f9e485f8e485f9e386"
    "f9e187f9e186f8e085f9df85f9de85fadd85f9dd85f9dc85fadc85f9da84f9d984f9d884f9d786fad685fad585fad485"
    "fad384fad284fad284fbd284fbd184fbd084fbd083fccf85fccf86fbce86fbce86fbce86fcce86fbcd86fbcd86fccc87"
    "fccb87fcc989fcc989fcc888fcc888fcc787fcc787fcc787fcc687fcc687fcc687fcc689fdc488fdc488fdc488fdc489"
    "fdc28afdc088fdc088fdc088fdc189fcc089fdc089fcbf89fdbe89fdbe8afdbd8afdbd8afdbc8afdbb89fdbb89fdbb89"
    "fdba8bfdba8afdba8afdba8afdb98afdb98afdb98afeb98afeb88bfdb78bfdb78bfdb78afeb78bfeb68afdb58afeb68c"
    "feb58efeb48efeb48efeb28efeb28efeb28efeb28efeb28ffeb38cfeb28cfeb28cfeb28efeb28efeb08efeb08efeb18e"
    "feb08efeb08efeb08efeb08dfeaf8dfeaf8dfeaf8dfeaf8efeb08efeb18ffeb290fdb390fdb38ffdb490fdb590fdb692"
    "fcb894fcbc94fcbe94fbc295fac994f8ce95f8d396f7da97f7df98f7e699f6ea98f7ed99f6ef99f5f09bf6f199f6f199"
    "f6f29bf6f29cf6f29cf6f29cf6f39cf6f39cf6f39cf6f39cf6f39cf6f39cf6f39cf6f39cf6f39cf6f39cf6f39cf6f39c"
    "f6f29cf6f29cf6f19cf7f09cf7ef9bf7ed9af7ec9bf7e99bf8e798f9e396fadf97fbdc98fdd795fece91ffc48fffbb8b"
    "ffae8bffa386ff9883ff8d80ff837dff777cff6d7aff6376ff5874ff4f70ff466fff3e6aff3767ff2f66ff2963ff235f"
    "ff1e5ce41e56c62054ab24588c356e74518c6867a35870b8427cd12a82dc1a83e8137bf10778f70276f7036cf20964ee"
    "115ae42052d82d4bca3643bc443aae502ea05c23956117855e0c6f5b026457005752004a4e003c46003344002c420024"
    "44002046001d46001b4b001a56001b64001c73001e7d002282002386002683002780002a81002a80002a7e00297f0029"
    "7e00287c00287c00277c00277c00267c00267c00257c00247c00247c00247c00247c00247e00237e00247e00247e0024"
    "8000258200258200268400278500278600278900288c00298e002992002994002a97002c9c002c9f002ea2002fa60030"
    "a80031ab0033b00034b60036bb0038c00039c4003bcb003dd0003fd60040da0042e00046e70048ec004bf2004ef70050"
    "f80053fa0056fc0059fe005dff005fff0062ff0065ff0069ff006cff0070ff0073ff0077ff0079ff007dff0081ff0085"
    "ff0088ff008cff008eff0092ff0095ff0098ff009bff009fff00a1ff00a4ff00a4ff00a5ff00a6ff00a8ff00a7ff00a8"
    "ff00a6ff00a6ff00a5ff00a6ff00a4ff00a4ff00a5ff00a6ff00a5ff00a7ff00a8ff00a7ff00a6ff00a7ff00a7ff00a8"
    "ff00a9ff00acff00acff00abff00a9ff00a9ff00a9ff00acff00aeff00afff00afff00afff00afff00b1ff00b1ff00b1"
    "ff00b0ff00b1ff00b1ff00b1ff00b1ff00b1ff00b1ff00b1ff00afff00afff00afff00b1ff00b1ff00b3ff00b3ff00b2"
    "ff00b3ff00b4ff00b4ff00b3ff00b3ff00b3ff00b3ff00b3ff00b3ff00b5ff00b5ff00b4ff00b3ff00b3ff00b3ff00b3"
    "ff00b5ff00b5ff00b4ff00b5ff00b6ff00b6ff00b6ff00b6ff00b6ff00b7ff00b7ff00b9ff00b9ff00b9ff00b9ff00b9"
    "ff00b7ff00b8ff00b9ff00b8ff00b8ff00b8ff00b8ff00b8ff00b9ff00b9ff00b8ff00b9ff00bbff00bbff00bbff00bb"
    "ff00b9ff00b9ff00b9ff00b9ff00b9ff00baff00baff00bbff00bbff00bdff00bdff00bdff00bdff00bdff00bdff00bd"
    "ff00bdff00bdff00bdff00bdff00bdff00bdff00bdff00bdff00beff00beff00bfff00bfff00beff00bfff00bfff00bf"
    "ff00bfff00bdff00bfff00beff00bdff00bdff00beff00beff00beff00bdff00beff00beff00beff00beff00beff00be"
    "ff00bfff00bfff00bfff00bfff00beff00beff00bfff00bfff00bfff00bfff00bfff00bfff00beff00bfff00bfff00bf"
    "ff00bfff00bfff00bfff00beff00bdff00beff00beff00beff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00bf"
    "ff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00beff00beff00bfff00bfff00bf"
    "ff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00bfff00c0ff00bfff00bfff00bfff00c0ff00c0ff00c0ff00c0"
    "ff00bfff00bfff00bfff00bfff00beff00bfff00bfff00c0ff00c0ff00c0ff00c0ff00c0ff00bfff00bfff00bfff00bd"
    "ff00bfff00bfff00bfff00bfff00beff00c0ff00c0ff00c0ff00bcff00bcff00bcff00bdff00bcff00bcff00bbff00bb"
    "ff00bcff00bcff00bcff00bcff00baff00baff00beff00bcff00bcff00bbff00bcff00bbff00bbff00bbff00bbff00bb"
    "ff00bbff00bbff00bbff00bbff00bbff00bbff00bbff00bbff00bbff00bbff00baff00b9ff00b9ff00baff00b0ff00aa"
)


def _top(parity):
    s = "".join(TOP_BLUE if parity else TOP_RED)
    return [tuple(int(s[i + j:i + j + 2], 16) for j in (0, 2, 4)) for i in range(0, len(s), 6)]


def top_at(parity, x):
    """top-row colour at virtual px x (linear between the 2-px samples)"""
    t = _top(parity)
    x = min(1917.999, max(0.0, x)) / 2.0
    i = int(math.floor(x)); f = x - i
    return tuple(t[i][c] * (1 - f) + t[i + 1][c] * f for c in range(3))


def interp(k, x):
    if x <= k[0][0]:
        return k[0][1]
    for (x0, y0), (x1, y1) in zip(k, k[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return k[-1][1]


def pkinterp(k, d):
    if d <= k[0][0]:
        return k[0][1]
    for (x0, c0), (x1, c1) in zip(k, k[1:]):
        if d <= x1:
            w = (d - x0) / (x1 - x0)
            return tuple(c0[i] + w * (c1[i] - c0[i]) for i in range(3))
    return k[-1][1]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    return (y2, min(1000, max(24, int(u))), min(1000, max(24, int(v))))


def pal_rgb(parity, fam, idx):
    """the RGB colour of one palette entry"""
    if fam == 0:                                    # left wall: d = 2 idx px outward from the cream edge
        return top_at(parity, max(12.0, XL0 - 2.0 * idx))
    if fam == 1:                                    # right wall
        return top_at(parity, XR0 + 2.0 * idx)
    if fam == 2:                                    # cream / socket, by u (4 px per entry)
        return top_at(parity, min(XR0, XL0 + 4.0 * idx))
    if idx < 91:                                    # eye below the rings
        return pkinterp(E_PKB if parity else E_PK, 2.0 * idx)
    if 128 <= idx <= 160:                           # the glow: pink-red over the cream, by alpha = 2 (idx - 128)
        a = min(1.0, 2.0 * (idx - 128) / 64.0)
        return tuple(HALO[c] * a + HALO_BG[c] * (1 - a) for c in range(3))
    if idx >= 192:                                  # Rings above 640: three darker, bolder ring cycles
        lo, hi = RING_HI[min(2, (idx - 192) >> 4)]
        m = (1 - math.cos(2 * math.pi * (idx & 15) / 16.0)) / 2
        return tuple(lo[c] * (1 - m) + hi[c] * m for c in range(3))
    if idx > 160:
        return E_RED
    j = (idx - 91) % 16                             # rings: red <-> maroon, 32 px of d per cycle
    m = (1 - math.cos(2 * math.pi * j / 16.0)) / 2
    return tuple(E_RED[c] * (1 - m) + E_MAR[c] * m for c in range(3))


def pal_rom():
    ys, us, vs = [], [], []
    for parity in range(2):
        for fam in range(4):
            for idx in range(256):
                y, u, v = prescale(rgb_to_hw10(pal_rgb(parity, fam, idx)))
                ys.append(min(255, (y + 2) >> 2)); us.append(min(255, (u + 2) >> 2)); vs.append(min(255, (v + 2) >> 2))
    return ys, us, vs


def sq_rom():
    return [min(255, int(round(128 * math.sqrt(i / 256.0)))) for i in range(1024)]


def phi_rom():
    out = []
    for i in range(256):
        p = interp(PHI_K, i / 128.0)
        base = int(round(min(p, 182)))
        bump = int(round(max(0.0, p - 182)))
        out.append((base << 8) | bump)
    return out


def t_rom():
    return [(int(round(128 * interp(TL_K, 2 * i))) << 8) | int(round(128 * interp(TR_K, 2 * i))) for i in range(512)]


def sh_rom():
    return [(int(round(64 * interp(SH_K, 8 * i))) << 8) | int(round(64 * interp(SHR_K, 8 * i))) for i in range(128)]


def vhd_table(name, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of unsigned(%d downto 0);" % ("t_" + name.lower(), depth - 1, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    ys, us, vs = pal_rom()
    parts = ["-- Generated by cateye.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package cateye_pkg is",
             vhd_table("C_CE_PY", 2048, 8, ys),
             vhd_table("C_CE_PU", 2048, 8, us),
             vhd_table("C_CE_PV", 2048, 8, vs),
             vhd_table("C_CE_SQ", 1024, 8, sq_rom()),
             vhd_table("C_CE_PHI", 256, 16, phi_rom()),
             vhd_table("C_CE_T", 512, 16, t_rom()),
             "end package cateye_pkg;", ""]
    with open(os.path.join(HERE, "cateye_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
