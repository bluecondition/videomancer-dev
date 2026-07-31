#!/usr/bin/env python3
"""Emit the PAL_ROM constant for pixelbeach.vhd.

RGB -> 10-bit YUV, full-range Y, BT.601 chroma -- stored with U and V
SWAPPED: hardware output needs Cr in the U slot and Cb in the V slot
(confirmed on HW 2026-07-18: standard-order palette rendered the ocean
orange and the sand light blue; same convention mondrian needed).
"""

PALS = [
 ("Day", dict(SKYHI=(104,176,232), SKYLO=(168,220,240), SUN=(252,224,112),
              SAND=(232,200,144), SANDW=(176,144,96), SURF=(120,216,208),
              AQUA=(56,180,196), MID=(32,112,204), DEEP=(16,64,152),
              FOAM=(248,252,252), GULL=(60,60,72), GULL2=(238,240,244), SAND2=(216,184,120),
              SANDM=(208,176,122))),
 ("Golden", dict(SKYHI=(232,160,96), SKYLO=(248,208,136), SUN=(255,200,64),
              SAND=(232,192,128), SANDW=(168,132,88), SURF=(152,208,184),
              AQUA=(72,168,168), MID=(48,104,168), DEEP=(24,56,120),
              FOAM=(252,244,224), GULL=(64,52,56), GULL2=(240,235,225), SAND2=(216,176,112),
              SANDM=(196,158,104))),
 ("Sunset", dict(SKYHI=(120,64,144), SKYLO=(240,120,96), SUN=(255,120,48),
              SAND=(200,160,112), SANDW=(140,104,76), SURF=(168,144,176),
              AQUA=(48,104,136), MID=(56,72,144), DEEP=(24,40,104),
              FOAM=(248,224,216), GULL=(48,40,56), GULL2=(230,215,215), SAND2=(184,146,100),
              SANDM=(166,128,90))),
 ("Night", dict(SKYHI=(16,24,64), SKYLO=(40,56,104), SUN=(232,240,248),
              SAND=(96,96,104), SANDW=(64,64,76), SURF=(96,160,168),
              AQUA=(48,104,136), MID=(24,64,112), DEEP=(8,32,72),
              FOAM=(216,232,240), GULL=(32,32,48), GULL2=(200,208,218), SAND2=(88,88,96),
              SANDM=(76,76,86))),
]
ORDER = ["SKYHI","SKYLO","SUN","SAND","SANDW","SURF",
         "AQUA","MID","DEEP","FOAM","GULL","SAND2","SANDM","GULL2"]

def yuv(r, g, b):
    y = 0.299*r + 0.587*g + 0.114*b
    cb = 128 + 0.564*(b - y)
    cr = 128 + 0.713*(r - y)
    q = lambda x: max(0, min(1023, int(round(x*4))))
    return q(y), q(cb), q(cr)

print("    type t_pal is array (0 to 63) of std_logic_vector(29 downto 0);")
print("    constant PAL_ROM : t_pal := (")
for pi, (pname, pal) in enumerate(PALS):
    print(f"        -- {pi}: {pname} (U/V swapped for hardware)")
    for ci, cn in enumerate(ORDER):
        y, cb, cr = yuv(*pal[cn])
        word = f"{y:010b}{cr:010b}{cb:010b}"   # U slot <- Cr, V slot <- Cb
        print(f'        {pi*16+ci:2d} => "{word}",  -- {cn}')
    for ci in range(14, 16):
        idx = pi*16 + ci
        comma = "" if idx == 63 else ","
        print(f'        {idx:2d} => "{0:030b}"{comma}')
print("    );")


# ---- v3.8 sunset gradient ROMs ----------------------------------------------
# SKYG: 16 keyframes each for day-hi, day-lo, night-hi, night-lo (idx 0 = P12
# low/black, 15 = P12 high).  Pinkish-red lands at idx ~6-7 where the sun's
# bottom touches the ocean (P12 ~ 41% at 720p geometry).
DAY_HI = [
 (0,0,0),(14,7,20),(30,14,40),(50,22,62),(72,30,80),(98,38,84),
 (128,46,84),(160,56,86),(196,70,92),(222,86,100),(235,100,105),
 (242,130,100),(246,165,110),(235,205,160),(165,200,230),(104,176,232)]
DAY_LO = [
 (24,56,120),(24,56,120),(24,56,120),(24,56,120),(24,56,120),(24,56,120),
 (24,56,120),(28,64,128),(32,72,136),(36,80,144),(42,92,156),
 (48,104,168),(60,136,168),(66,152,168),(72,168,168),(72,168,168)]
NGT_HI = [
 (0,0,0),(3,4,10),(6,9,19),(9,13,28),(12,18,38),(15,22,47),
 (18,27,56),(20,31,64),(23,35,71),(25,38,77),(27,40,82),
 (28,42,86),(29,43,88),(30,44,89),(30,45,90),(30,45,90)]
NGT_LO = [
 (8,32,72),(8,32,72),(8,32,72),(8,32,72),(8,32,72),(8,32,72),
 (8,32,72),(10,38,80),(13,44,88),(16,50,96),(20,57,104),
 (24,64,112),(32,78,120),(40,92,128),(48,104,136),(48,104,136)]

DAY_GLOW = [
 (0,0,0),(24,10,26),(52,20,50),(88,32,70),(126,44,82),(168,58,88),
 (205,78,98),(232,102,104),(246,132,104),(250,164,112),(250,196,132),
 (245,218,168),(230,225,205),(205,222,232),(185,215,238),(168,220,240)]
NGT_GLOW = [
 (0,0,0),(6,8,16),(12,16,30),(18,24,44),(24,32,57),(29,38,68),
 (33,44,78),(37,49,86),(40,53,93),(43,56,98),(45,59,102),
 (46,61,105),(47,62,107),(48,63,108),(49,64,109),(50,65,110)]

# SUNG: sun disc colour, 16 keyframes day (yellow -> orange -> bright red ->
# deep red near the bottom) + 16 night (moon: white -> pale grey -> gone-dark)
SUN_DAY = [
 (140,25,35),(140,25,35),(140,25,35),(140,25,35),(140,25,35),(145,27,36),
 (152,29,37),(165,32,38),(195,42,40),(222,58,42),(238,88,46),
 (245,125,54),(250,158,64),(252,186,76),(253,208,92),(252,224,112)]
SUN_NGT = [
 (40,44,52),(70,76,86),(100,108,118),(130,138,148),(155,163,172),(175,182,190),
 (192,198,206),(205,211,218),(215,220,226),(222,227,232),(227,231,236),
 (229,234,240),(231,236,243),(232,238,245),(232,239,247),(232,240,248)]
DEEP_BASE = {0:(16,64,152), 1:(24,56,120), 2:(24,40,104), 3:(8,32,72)}

def emit_rom(name, colors, size):
    print(f"    type t_{name.lower()} is array (0 to {size-1}) of std_logic_vector(29 downto 0);")
    print(f"    constant {name} : t_{name.lower()} := (")
    for i,(r,g,b) in enumerate(colors):
        y,cb,cr = yuv(r,g,b)
        comma = "" if i == size-1 else ","
        print(f'        {i:2d} => "{y:010b}{cr:010b}{cb:010b}"{comma}')
    print("    );")

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "sunset":
        emit_rom("SKYG_ROM", DAY_HI + DAY_LO + DAY_GLOW + [(0,0,0)]*16 + NGT_HI + NGT_LO + NGT_GLOW + [(0,0,0)]*16, 128)
        emit_rom("SUNG_ROM", SUN_DAY + SUN_NGT, 32)
        emit_rom("DEEPB_ROM", [DEEP_BASE[i] for i in range(4)], 4)
        sys.exit(0)
