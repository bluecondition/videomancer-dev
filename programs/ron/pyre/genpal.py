#!/usr/bin/env python3
"""Generate the 256-entry fire palette ROM (10-bit YUV, limited range, U/V swapped
for hardware) as a VHDL constant.  Index = cell temperature 0..255; entries below
24 are black so faint cool parcels vanish instead of glowing dark red.

Usage: genpal.py > pal.txt   (splice_pal.py puts it into pyre.vhd)"""
import numpy as np

def ramp(tp, rp, gp, bp):
    t = np.clip((np.arange(256) - 24) / (255.0 - 24), 0, 1)
    return (np.interp(t, tp, rp), np.interp(t, tp, gp), np.interp(t, tp, bp))

# black -> dark red -> red -> orange -> yellow -> white-yellow core
# low end compressed (black -> red within the first ~20%) so the flame
# boundary reads crisp through the cell interpolation
fire = ramp([0.00, 0.06, 0.20, 0.45, 0.70, 0.88, 0.97, 1.00],
            [0.00, 0.30, 0.70, 0.98, 1.00, 1.00, 1.00, 1.00],
            [0.00, 0.01, 0.06, 0.22, 0.48, 0.72, 0.90, 0.97],
            [0.00, 0.00, 0.00, 0.01, 0.02, 0.05, 0.25, 0.70])

def yuv(rgb):
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = 0.564 * (b - y)
    cr = 0.713 * (r - y)
    yl = 64 + y * 876
    yl = np.where(yl > 704, 704 + (yl - 704) / 2, yl)
    Y = np.clip(np.round(yl), 64, 768).astype(int)
    U = np.clip(np.round(512 + cr * 896), 64, 960).astype(int)  # U <= Cr (HW swap)
    V = np.clip(np.round(512 + cb * 896), 64, 960).astype(int)  # V <= Cb (HW swap)
    return Y, U, V

Y, U, V = yuv(fire)
vals = [(Y[i] << 20) | (U[i] << 10) | V[i] for i in range(256)]
print('  type t_pal is array (0 to 255) of std_logic_vector(29 downto 0);')
print('  -- packed Y(29:20) U(19:10)=Cr V(9:0)=Cb, limited range, U/V swapped for HW')
lines = []
for i in range(0, 256, 4):
    lines.append('    ' + ', '.join(f'{i+j} => "{vals[i+j]:030b}"' for j in range(4)))
print('  constant C_PAL : t_pal := (\n' + ',\n'.join(lines) + '\n  );')
