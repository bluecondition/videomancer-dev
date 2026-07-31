#!/usr/bin/env python3
"""Generate the 512-entry packed palette ROM (fire + blue flame) as VHDL."""
import numpy as np

def ramp(tp, rp, gp, bp):
    t = np.arange(256) / 255.0
    return (np.interp(t, tp, rp), np.interp(t, tp, gp), np.interp(t, tp, bp))

# classic fire: yellow base -> orange -> red -> dark red -> black tips
fire = ramp([0.00, 0.12, 0.30, 0.50, 0.75, 0.92, 1.00],
            [0.00, 0.25, 0.55, 0.95, 1.00, 1.00, 1.00],
            [0.00, 0.01, 0.03, 0.12, 0.55, 0.85, 0.93],
            [0.00, 0.00, 0.00, 0.01, 0.02, 0.06, 0.12])

# blue flame: black -> deep blue -> blue -> cyan -> blue-white
blue = ramp([0.00, 0.10, 0.30, 0.55, 0.80, 1.00],
            [0.00, 0.00, 0.02, 0.10, 0.50, 0.95],
            [0.00, 0.01, 0.10, 0.35, 0.75, 1.00],
            [0.00, 0.15, 0.50, 0.95, 1.00, 1.00])

# layer 2 in BLUE FLAME mode: violet -> magenta -> pink -> white.  Same ramp
# structure as the classic fire so the two layers read as the same MATERIAL
# at different temperatures, but the hue is unmistakably distinct.
violet = ramp([0.00, 0.12, 0.30, 0.50, 0.75, 0.92, 1.00],
              [0.00, 0.15, 0.40, 0.70, 0.90, 1.00, 1.00],
              [0.00, 0.00, 0.02, 0.05, 0.15, 0.45, 0.85],
              [0.00, 0.20, 0.55, 0.85, 1.00, 1.00, 1.00])

def yuv(rgb):
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = 0.564 * (b - y)
    cr = 0.713 * (r - y)
    Y = np.clip(np.round(y * 1023), 0, 1023).astype(int)
    U = np.clip(np.round(512 + cr * 1023), 0, 1023).astype(int)  # U <= Cr (HW swap)
    V = np.clip(np.round(512 + cb * 1023), 0, 1023).astype(int)  # V <= Cb (HW swap)
    return Y, U, V

# layer 2 in NORMAL mode: deep dark red — embers/dying fire behind the main
# flames.  No yellow anywhere in the ramp, so it never competes with layer 1's
# hot core; it reads as depth rather than a second effect.
darkred = ramp([0.00, 0.15, 0.40, 0.70, 1.00],
               [0.00, 0.20, 0.45, 0.70, 0.85],
               [0.00, 0.00, 0.02, 0.06, 0.15],
               [0.00, 0.00, 0.00, 0.01, 0.03])

vals = []
for pal in (fire, darkred, blue, violet):
    Y, U, V = yuv(pal)
    for i in range(256):
        vals.append((Y[i] << 20) | (U[i] << 10) | V[i])

print('  type t_pal is array (0 to 1023) of std_logic_vector(29 downto 0);')
print('  -- packed Y(29:20) U(28:10)=Cr V(9:0)=Cb.  Addr = blue & layer2 & heat:')
print('  --   0..255 fire (L1), 256..511 deep dark red (L2),')
print('  --   512..767 blue flame (L1, S9), 768..1023 violet (L2, S9)')
lines = []
for i in range(0, 1024, 8):
    chunk = ', '.join(f'{i+j} => "{vals[i+j]:030b}"' for j in range(8))
    lines.append('    ' + chunk)
print('  constant C_PAL : t_pal := (\n' + ',\n'.join(lines) + '\n  );')
