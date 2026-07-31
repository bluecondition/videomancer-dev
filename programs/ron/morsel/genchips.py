#!/usr/bin/env python3
"""
MORSEL -- chocolate-chip shape ROM generator.

Builds a real 3-D chocolate morsel (a flat-based solid of revolution whose
sides bulge up from the base and whose tip curls over to one side), then
photographs it from 8 different 3-D orientations and stores each SILHOUETTE
as a polar radius table R(theta).

Why polar: the hardware already has to work out the angle of a pixel about
the chip centre (it needs it for the rim lighting anyway), so a radius-vs-
angle table gives an ARBITRARY silhouette -- flat bottom, rounded shoulders,
curled tip -- for one BRAM read and one compare.  Rotating a chip is then a
subtract on the angle index: free, and continuous (128 steps = 2.8 deg), not
the 4 flips of the sugarcoat triangle this forked from.

Emits morsel_rom_pkg.vhd:
  C_SHAPE  1024 x 8   -- 8 variants x 128 angles, radius in 1/4 cell-units
  C_RTAB   1024 x 12  -- (mn,mx) -> {r*4 (8b), sub-angle (4b)}: exact radius
                         and octant fraction with no divider and no multiplier
  C_COS    128 x 8    -- signed cosine for the rim light
"""

import math
import numpy as np

# ---------------------------------------------------------------------------
# 3-D morsel model
# ---------------------------------------------------------------------------
# Units: base radius = 1.  z runs 0 (flat base) .. H (tip).
#
#   r(t)  side profile     t = z/H, r(0) = 1 (flat bottom), r(1) = 0 (tip)
#   a(t)  axis curl        the tip leans/curls over toward the +curl azimuth


def profile(t, bulge, tipsharp):
    """Half-width of the morsel at normalised height t (0 = base, 1 = tip).

    (1 - t**p)**q: p slightly above 1 lets the sides leave the flat base
    almost vertically and bulge gently outward; q below 1 keeps them convex
    all the way up but still closes to a real point rather than a dome.
    """
    t = np.clip(t, 0.0, 1.0)
    return np.power(np.clip(1.0 - np.power(t, bulge), 0.0, 1.0), tipsharp)


def curl(t, amt):
    """Sideways lean of the axis: nothing at the base, hooking over at the tip."""
    return amt * (t ** 2.6) + 0.55 * amt * (t ** 9.0)


def silhouette(phi_deg, psi_deg, H, bulge, tipsharp, curl_amt,
               N=384, EXT=1.75):
    """Rasterise the projected outline.

    A solid of revolution projects to the UNION of its stacked disks.  A
    horizontal disk of radius r centred (cx,cy,z) seen from elevation phi
    projects to an ellipse centred (cx, z*cos(phi) - cy*sin(phi)) with
    semi-axes (r, r*sin(phi)).  phi = 0 is a pure side view (flat bottom
    reads as a straight edge), phi = 90 is straight down (a round chip with
    the curled tip poking out to one side).
    """
    phi = math.radians(phi_deg)
    psi = math.radians(psi_deg)
    sp, cp = math.sin(phi), math.cos(phi)

    ax = np.linspace(-EXT, EXT, N)
    U, V = np.meshgrid(ax, ax)                       # V is screen-up
    mask = np.zeros((N, N), dtype=bool)

    for t in np.linspace(0.0, 1.0, 220):
        r = float(profile(t, bulge, tipsharp))
        if r <= 1e-4:
            continue
        a = float(curl(t, curl_amt))
        cx = a * math.cos(psi)
        cy = a * math.sin(psi)
        z = t * H
        eu = cx                                      # ellipse centre, screen u
        ev = z * cp - cy * sp                        # ellipse centre, screen v
        au = r                                       # semi-axis u
        av = max(r * sp, 1e-3)                       # semi-axis v (foreshortened)
        du = (U - eu) / au
        dv = (V - ev) / av
        mask |= (du * du + dv * dv) <= 1.0
        if sp < 0.02:                                # pure side view: draw the
            seg = (np.abs(V - ev) <= (2.0 * EXT / N)) & (np.abs(U - eu) <= au)
            mask |= seg                              # degenerate disk as a line

    return mask, ax


def polar_table(mask, ax, nang=128):
    """R(theta) about the silhouette centroid, in WORLD units.

    Screen convention matches the hardware: x right, y DOWN, angle index
    counts from +x toward +y (so index 32 = straight down the screen).
    """
    N = len(ax)
    step = ax[1] - ax[0]
    ys, xs = np.nonzero(mask)
    cu = ax[xs].mean()
    cv = ax[ys].mean()                               # ax[] here indexes V rows

    R = np.zeros(nang)
    for i in range(nang):
        th = 2.0 * math.pi * i / nang
        dx = math.cos(th)
        dy = math.sin(th)                            # screen-down
        last = 0.0
        d = 0.0
        while d < 2.4:
            u = cu + dx * d
            v = cv - dy * d                          # v is up, y is down
            iu = int(round((u - ax[0]) / step))
            iv = int(round((v - ax[0]) / step))
            if 0 <= iu < N and 0 <= iv < N and mask[iv, iu]:
                last = d
            d += step * 0.5
        R[i] = last
    return R


# --- the 8 photographed orientations -------------------------------------
# elevation 0 = full side view (flat bottom, curled tip: the classic morsel)
# elevation 85 = almost straight down (round, with the tip poking off one side)
VARIANTS = [
    # phi   psi    H     bulge tipsharp curl
    #
    # psi is the curl azimuth; note that at LOW elevation only cos(psi)
    # projects to screen, so psi must sit near 0/180 there or the hook points
    # straight at the camera and vanishes.  The per-cell rotation scatters
    # which way each chip's point ends up facing, so fixing psi here costs
    # nothing.
    (  0,    0,  1.62,  1.30,  0.85,  0.30),   # 0 side-on: flat base, tip curls
    (  9,  180,  1.55,  1.45,  0.80,  0.36),   # 1 barely tipped, hooks the other way
    ( 20,   15,  1.68,  1.25,  0.90,  0.32),   # 2 leaning back, taller
    ( 32,  195,  1.50,  1.40,  0.82,  0.44),   # 3
    ( 45,  175,  1.58,  1.30,  0.86,  0.55),   # 4 three-quarter view
    ( 58,  205,  1.45,  1.50,  0.78,  0.62),   # 5 fat, strong hook
    ( 72,    0,  1.55,  1.35,  0.52,  0.78),   # 6 mostly round + a clear point
    ( 87,  180,  1.50,  1.40,  0.48,  0.86),   # 7 top-down: round, small point
]

# world scale: 1 base-radius -> S cell-units.  Sized so the biggest variant
# lands just under 24 units (R*4 <= ~96), leaving headroom for the size knob.
S = 19.0


def build_shape_rom():
    rows = []
    raw = []
    for (phi, psi, H, bulge, tipsharp, curl_amt) in VARIANTS:
        mask, ax = silhouette(phi, psi, H, bulge, tipsharp, curl_amt)
        R = polar_table(mask, ax)
        raw.append(R)
        q = np.clip(np.round(R * S * 4.0), 4, 255).astype(int)
        rows.append(q)
    return rows, raw


def build_rtab():
    """(mn,mx) -> packed {r*4 : 8b, sub-angle : 4b}.

    mn/mx are the smaller/larger of |cx|,|cy| (already halved by the pixel
    path when they exceed 31).  One BRAM read replaces both a square-root and
    a divider: exact radius plus the fraction of the 45-degree octant.
    """
    tab = []
    for mn in range(32):
        for mx in range(32):
            r4 = int(round(4.0 * math.hypot(mn, mx)))
            r4 = min(r4, 255)
            if mx == 0:
                sub = 0
            else:
                sub = int(round(math.atan2(mn, mx) * 16.0 / (math.pi / 4.0)))
                sub = max(0, min(15, sub))
            tab.append((r4 << 4) | sub)
    return tab


def build_cos():
    return [int(round(127 * math.cos(2 * math.pi * i / 128))) & 0xFF
            for i in range(128)]


# ---------------------------------------------------------------------------
# VHDL emit
# ---------------------------------------------------------------------------
def emit(path, shape_rows, rtab, costab):
    L = []
    A = L.append
    A("-- morsel_rom_pkg.vhd -- GENERATED by genchips.py, do not hand-edit.")
    A("--")
    A("-- C_SHAPE : 8 chip variants x 128 angle steps, silhouette radius in")
    A("--           1/4 cell-units.  Variant = a different 3-D orientation of")
    A("--           the same modelled morsel (0 = side-on flat-bottomed")
    A("--           teardrop with a curled tip .. 7 = top-down round chip with")
    A("--           a small point off one side).  Angle index counts from +x")
    A("--           toward +y (screen down), 128 steps = 2.8 deg.")
    A("-- C_RTAB  : (min,max of |cx|,|cy|) -> radius*4 (bits 11:4) and the")
    A("--           octant sub-angle 0..15 (bits 3:0).  No sqrt, no divide.")
    A("-- C_COS   : signed cosine, 128 steps, for the rim light.")
    A("")
    A("library ieee;")
    A("use ieee.std_logic_1164.all;")
    A("use ieee.numeric_std.all;")
    A("")
    A("package morsel_rom_pkg is")
    A("")
    A("    type t_shape is array (0 to 1023) of unsigned(7 downto 0);")
    A("    type t_rtab  is array (0 to 1023) of unsigned(11 downto 0);")
    A("    type t_cos   is array (0 to 127)  of signed(7 downto 0);")
    A("")

    A("    constant C_SHAPE : t_shape := (")
    flat = []
    for v, row in enumerate(shape_rows):
        flat.extend(int(x) for x in row)
    for i in range(0, 1024, 8):
        chunk = ", ".join("to_unsigned(%d, 8)" % flat[i + j] for j in range(8))
        end = "," if i + 8 < 1024 else ""
        note = "  -- variant %d" % (i // 128) if i % 128 == 0 else ""
        A("        %s%s%s" % (chunk, end, note))
    A("    );")
    A("")

    A("    constant C_RTAB : t_rtab := (")
    for i in range(0, 1024, 8):
        chunk = ", ".join("to_unsigned(%d, 12)" % rtab[i + j] for j in range(8))
        end = "," if i + 8 < 1024 else ""
        A("        %s%s" % (chunk, end))
    A("    );")
    A("")

    A("    constant C_COS : t_cos := (")
    for i in range(0, 128, 8):
        chunk = ", ".join('to_signed(%d, 8)' % (costab[i + j] - 256
                                                if costab[i + j] > 127
                                                else costab[i + j])
                          for j in range(8))
        end = "," if i + 8 < 128 else ""
        A("        %s%s" % (chunk, end))
    A("    );")
    A("")
    A("end package morsel_rom_pkg;")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


def contact_sheet(shape_rows, path):
    """Draw the 8 stored silhouettes so the shapes can be eyeballed."""
    from PIL import Image
    CELL = 128
    img = np.zeros((CELL, CELL * 8, 3), dtype=np.uint8)
    for v, row in enumerate(shape_rows):
        for py in range(CELL):
            for px in range(CELL):
                dx = px - CELL // 2
                dy = py - CELL // 2
                r4 = 4.0 * math.hypot(dx, dy) / 2.0        # 2 px per cell-unit
                ang = int(round(math.atan2(dy, dx) * 128 / (2 * math.pi))) & 127
                R = row[ang]
                if r4 <= R:
                    edge = R - r4
                    lit = math.cos((ang - 80) * 2 * math.pi / 128)
                    band = 3 if edge < R / 8 else (2 if edge < R / 4 else
                                                   (1 if edge < R / 2 else 0))
                    sh = int(lit * band * 22)
                    val = max(0, min(255, 66 + sh))
                    img[py, px + v * CELL] = (val, int(val * 0.62), int(val * 0.44))
                else:
                    img[py, px + v * CELL] = (206, 162, 100)
    Image.fromarray(img).resize((CELL * 8 * 2, CELL * 2),
                                Image.NEAREST).save(path)


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    shape_rows, raw = build_shape_rom()
    for v, R in enumerate(raw):
        print("variant %d: Rmin %.2f Rmax %.2f -> units %.1f..%.1f"
              % (v, R.min(), R.max(), R.min() * S, R.max() * S))
    rtab = build_rtab()
    costab = build_cos()
    emit(os.path.join(here, "morsel_rom_pkg.vhd"), shape_rows, rtab, costab)
    contact_sheet(shape_rows, os.path.join(here, "shapes.png"))
    print("wrote morsel_rom_pkg.vhd + shapes.png")
