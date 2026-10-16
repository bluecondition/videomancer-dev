#!/usr/bin/env python3
"""Generate Screenprint's ROM constants (v0.2.0) and splice them into screenprint.vhd.

C_SIN  quarter-wave sine, 1024 entries: round(16384 * sin((i + 0.5) / 1024 * pi / 2))
C_INV  cell-units per pixel for the P12 Size curve, 1024 entries:
       (2^20 / cell) / 8, cell = 4 * 40^(i / 1023) px (4 .. 160 px, exponential)
Ink table: BT.601 limited range from sRGB via 8-bit codes x4, U/V swapped for the
hardware (Red/Orange/Yellow/Green/Blue match the scope-verified phosphor rainbow table).

Usage: python3 gen_tables.py   (rewrites the block between the GEN markers)
"""
import math
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
VHD = HERE / "screenprint.vhd"


def rom(name, vals, comment):
    lines = [f"    -- {comment}", f"    constant {name} : t_rom := ("]
    for r in range(0, len(vals), 12):
        chunk = ", ".join(f'x"{v:04X}"' for v in vals[r:r + 12])
        sep = "," if r + 12 < len(vals) else ""
        lines.append(f"        {chunk}{sep}")
    lines.append("    );")
    return "\n".join(lines)


def ink(rgb):
    r, g, b = (c / 255 for c in rgb)
    y = 0.299 * r + 0.587 * g + 0.114 * b
    y8 = round(16 + 219 * y)
    cb8 = round(128 + 224 * (b - y) / 1.772)
    cr8 = round(128 + 224 * (r - y) / 1.402)
    return y8 * 4, cr8 * 4, cb8 * 4     # (Y, U, V) with U <- Cr, V <- Cb


def main():
    sin = [round(16384 * math.sin((i + 0.5) / 1024 * math.pi / 2)) for i in range(1024)]
    assert max(sin) <= 16384
    cells = [4 * 40 ** (i / 1023) for i in range(1024)]
    inv = [round(2 ** 20 / c / 8) for c in cells]
    assert max(inv) <= 0xFFFF and min(inv) > 0

    names = ["Black", "Red", "Orange", "Yellow", "Green", "Cyan", "Blue", "Magenta"]
    rgbs = [None, (255, 0, 0), (255, 128, 0), (255, 255, 0), (0, 255, 0),
            (0, 255, 255), (0, 0, 255), (255, 0, 255)]
    yuv = [(64, 512, 512)] + [ink(c) for c in rgbs[1:]]
    # scope-verified rainbow entries (memory phosphor_rainbow_palette)
    verified = {"Red": (328, 960, 360), "Orange": (584, 772, 212), "Yellow": (840, 584, 64),
                "Green": (580, 136, 216), "Blue": (164, 440, 960)}
    for k, n in enumerate(names):
        if n in verified:     # the computed chroma matches; Red's luma differs by 4 (rounding)
            assert yuv[k][1:] == verified[n][1:], (n, yuv[k], verified[n])
            yuv[k] = verified[n]
    a = [max(0, y - 64) for y, _, _ in yuv]
    an = [round(x * 1023 / 876) for x in a]
    tab = lambda vals: ", ".join(f"to_unsigned({v}, 10)" for v in vals)
    ink_block = "\n".join([
        "    -- " + " / ".join(names) + ": BT.601 10-bit, U/V swapped for the",
        "    -- hardware (the scope-verified phosphor rainbow values where they exist)",
        f"    constant C_INK_A  : t_ink := ({tab(a)});",
        f"    constant C_INK_AN : t_ink := ({tab(an)});",
        f"    constant C_INK_U  : t_ink := ({tab([u for _, u, _ in yuv])});",
        f"    constant C_INK_V  : t_ink := ({tab([v for _, _, v in yuv])});",
    ])

    block = "\n".join([
        "    -- GEN BEGIN (gen_tables.py)",
        rom("C_SIN", sin, "quarter-wave sine: 16384 * sin((i + 0.5) / 1024 * pi / 2)"),
        rom("C_INV", inv, "cell-units per pixel / 8: 2^20 / cell / 8, cell = 4 * 40^(i / 1023) px"),
        ink_block,
        "    -- GEN END",
    ])
    src = VHD.read_text()
    head, rest = src.split("    -- GEN BEGIN (gen_tables.py)\n", 1)
    _, tail = rest.split("    -- GEN END\n", 1)
    VHD.write_text(head + block + "\n" + tail)
    for n, v, x, xn in zip(names, yuv, a, an):
        print(f"{n:8s} Y/U/V {v}  a {x}  a_n {xn}")
    print(f"inv: {inv[0]} .. {inv[-1]}  (cell {cells[0]:.1f} .. {cells[-1]:.1f} px); "
          f"8 px at index {round(1023 * math.log(2) / math.log(40))}")


if __name__ == "__main__":
    main()
