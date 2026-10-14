#!/usr/bin/env python3
# Copyright (C) 2026
# SPDX-License-Identifier: GPL-3.0-only
"""
GAMUT - build hook and palette compiler.

Reads palettes.txt (the human-readable library), validates it, and emits
  gamut_palettes_pkg.vhd   ROM constants (10-bit BT.601 YUV, unswapped; swap at output compose)
  contact/manifest.json    everything the VHDL knows, plus hex/OKLCH, for tooling
With --sheets it also renders contact sheets under contact/ for curation:
  NN_<shelf>_dark.png / _light.png   one card per palette, in K1 order
  00_library_dark.png / _light.png   every palette as a strip, shelf by shelf
  index.html                          swatches with hex readouts and the check report

ROM layout (see the package header for the VHDL view):
  colour word (32 bits) : Y[31:22] U[21:12] V[11:2] role[1:0]   role 0=D 1=S 2=A 3=N
  palette word (16 bits): [14:4] first colour offset, [3:0] colour count - 1, [15] hushed
  shelf word  (16 bits) : [15:8] first palette index, [7:0] palette count

Run from anywhere: paths are relative to this file. The builder invokes it
with no arguments (ROM only); use --sheets when curating.
"""

import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "palettes.txt")
PKG = os.path.join(HERE, "gamut_palettes_pkg.vhd")
CONTACT = os.path.join(HERE, "contact")

NAME_MAX = 16
MAX_COLOURS = 16
MIN_COLOURS = 2
DUP_DE = 0.045          # OKLab distance under which two matched colours count as "the same"
CONTRAST_MIN = 0.22     # OKLab L span a non-hushed palette must reach
ROLE_BITS = {"D": 0, "S": 1, "A": 2, "N": 3}

# ----------------------------------------------------------------------------- colour math

def srgb_to_linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

def linear_to_srgb(c):
    c = max(0.0, min(1.0, c))
    return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055

def hex_to_rgb(h):
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (1, 3, 5))

def rgb_to_hex(rgb):
    return "#%02X%02X%02X" % tuple(int(round(max(0, min(1, c)) * 255)) for c in rgb)

def rgb_to_oklab(rgb):
    r, g, b = (srgb_to_linear(c) for c in rgb)
    l = 0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b
    m = 0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b
    s = 0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b
    l, m, s = (math.copysign(abs(v) ** (1 / 3), v) for v in (l, m, s))
    return (0.2104542553 * l + 0.7936177850 * m - 0.0040720468 * s,
            1.9779984951 * l - 2.4285922050 * m + 0.4505937099 * s,
            0.0259040371 * l + 0.7827717662 * m - 0.8086757660 * s)

def oklab_to_lch(lab):
    L, a, b = lab
    return (L, math.hypot(a, b), (math.degrees(math.atan2(b, a)) + 360.0) % 360.0)

def de(lab1, lab2):
    return math.dist(lab1, lab2)

def rgb_to_yuv10(rgb):
    """sRGB (0..1) -> legal-range 10-bit BT.601 (Y, Cb, Cr), chroma centred at 512."""
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = -0.168736 * r - 0.331264 * g + 0.5 * b
    cr = 0.5 * r - 0.418688 * g - 0.081312 * b
    clip = lambda v, lo, hi: max(lo, min(hi, int(round(v))))
    return clip(64 + 876 * y, 64, 940), clip(512 + 896 * cb, 64, 960), clip(512 + 896 * cr, 64, 960)

def yuv10_to_rgb(yuv):
    Y, U, V = yuv
    y = (Y - 64) / 876.0
    cb = (U - 512) / 896.0
    cr = (V - 512) / 896.0
    return (max(0.0, min(1.0, y + 1.402 * cr)),
            max(0.0, min(1.0, y - 0.344136 * cb - 0.714136 * cr)),
            max(0.0, min(1.0, y + 1.772 * cb)))

def xy_to_rgb(x, y):
    """CIE xy chromaticity -> brightest in-gamut sRGB with that chromaticity (clipped if outside)."""
    X, Y, Z = x / y, 1.0, (1 - x - y) / y
    r = 3.2406 * X - 1.5372 * Y - 0.4986 * Z
    g = -0.9689 * X + 1.8758 * Y + 0.0415 * Z
    b = 0.0557 * X - 0.2040 * Y + 1.0570 * Z
    r, g, b = (max(0.0, c) for c in (r, g, b))
    peak = max(r, g, b)
    return tuple(linear_to_srgb(c / peak) for c in (r, g, b))

# ----------------------------------------------------------------------------- parsing

TOKEN = re.compile(r"^(#[0-9A-Fa-f]{6}|yuv\(\s*\d+\s*,\s*\d+\s*,\s*\d+\s*\)|xy\(\s*[0-9.]+\s*,\s*[0-9.]+\s*\))$")

def parse_token(tok, where):
    if tok.startswith("#"):
        rgb = hex_to_rgb(tok.upper())
        return {"hex": tok.upper(), "rgb": rgb, "yuv": rgb_to_yuv10(rgb), "token": tok.upper()}
    if tok.startswith("yuv("):
        Y, U, V = (int(v) for v in tok[4:-1].split(","))
        if not (4 <= Y <= 1019 and 64 <= U <= 960 and 64 <= V <= 960):
            raise ValueError("%s: yuv() out of legal range: %s" % (where, tok))
        rgb = yuv10_to_rgb((Y, U, V))
        return {"hex": rgb_to_hex(rgb), "rgb": rgb, "yuv": (Y, U, V), "token": tok}
    if tok.startswith("xy("):
        x, y = (float(v) for v in tok[3:-1].split(","))
        rgb = xy_to_rgb(x, y)
        return {"hex": rgb_to_hex(rgb), "rgb": rgb, "yuv": rgb_to_yuv10(rgb), "token": tok}
    raise ValueError("%s: bad colour token %r" % (where, tok))

def parse(path):
    shelves = []
    errors = []
    with open(path, encoding="utf-8") as f:
        for ln, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            where = "line %d" % ln
            if line.lower().startswith("shelf "):
                name = line[6:].strip().upper()
                if len(name) > NAME_MAX:
                    errors.append("%s: shelf name > %d chars: %s" % (where, NAME_MAX, name))
                shelves.append({"name": name, "palettes": []})
                continue
            if not shelves:
                errors.append("%s: palette before any shelf" % where)
                continue
            parts = [p.strip() for p in line.split("|")]
            if len(parts) < 2:
                errors.append("%s: expected NAME | colours [| keys]" % where)
                continue
            name = parts[0].upper()
            if not name or len(name) > NAME_MAX:
                errors.append("%s: name empty or > %d chars: %r" % (where, NAME_MAX, name))
            toks = parts[1].split()
            colours = []
            for t in toks:
                if not TOKEN.match(t):
                    errors.append("%s: bad colour token %r" % (where, t))
                    continue
                try:
                    colours.append(parse_token(t, where))
                except ValueError as e:
                    errors.append(str(e))
            if not (MIN_COLOURS <= len(colours) <= MAX_COLOURS):
                errors.append("%s: %s has %d colours (need %d..%d)" % (where, name, len(colours), MIN_COLOURS, MAX_COLOURS))
            keys = {}
            for kv in " ".join(parts[2:]).split():
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    keys[k.lower()] = v
                else:
                    keys[kv.lower()] = True
            roles = keys.get("roles")
            if roles is not None:
                roles = roles.upper()
                if len(roles) != len(colours) or any(r not in ROLE_BITS for r in roles):
                    errors.append("%s: %s roles=%s must be %d letters from DSAN" % (where, name, roles, len(colours)))
                    roles = None
            for c in colours:
                c["lab"] = rgb_to_oklab(c["rgb"])
                c["lch"] = oklab_to_lch(c["lab"])
            shelves[-1]["palettes"].append({
                "name": name, "colours": colours, "roles": roles,
                "hushed": bool(keys.get("hushed")), "src": keys.get("src"), "line": ln,
            })
    return shelves, errors

# ----------------------------------------------------------------------------- checks + derivation

def derive_roles(p):
    """WEIGHTED view fallback: lowest chroma -> dominant, highest -> accent, rest secondary."""
    if p["roles"]:
        return p["roles"]
    chroma = [c["lch"][1] for c in p["colours"]]
    roles = ["S"] * len(chroma)
    roles[chroma.index(min(chroma))] = "D"
    roles[chroma.index(max(chroma))] = "A"
    return "".join(roles)

def match_distance(pa, pb):
    """Greedy order-insensitive matching; returns the worst matched OKLab distance."""
    a = [c["lab"] for c in pa["colours"]]
    b = [c["lab"] for c in pb["colours"]]
    worst = 0.0
    used = set()
    for la in a:
        best, bi = 9.0, -1
        for i, lb in enumerate(b):
            if i in used:
                continue
            d = de(la, lb)
            if d < best:
                best, bi = d, i
        used.add(bi)
        worst = max(worst, best)
    return worst

def run_checks(shelves):
    warnings, errors = [], []
    flat = [(s["name"], p) for s in shelves for p in s["palettes"]]
    seen = {}
    for sname, p in flat:
        key = p["name"]
        if key in seen:
            warnings.append("duplicate name %s in %s and %s" % (key, seen[key], sname))
        seen[key] = sname
        Ls = [c["lab"][0] for c in p["colours"]]
        span = max(Ls) - min(Ls)
        p["l_span"] = span
        if span < CONTRAST_MIN and not p["hushed"]:
            warnings.append("%s/%s: low lightness contrast (OKLab L span %.2f); tag hushed if intended" % (sname, key, span))
        for i in range(len(p["colours"])):
            for j in range(i + 1, len(p["colours"])):
                ci, cj = p["colours"][i], p["colours"][j]
                if ci["token"].startswith("yuv(") or cj["token"].startswith("yuv("):
                    continue  # video-level steps can be identical on an sRGB display by design
                d = de(ci["lab"], cj["lab"])
                if d < 0.02:
                    warnings.append("%s/%s: colours %d and %d nearly identical (dE %.3f)" % (sname, key, i, j, d))
        p["roles_eff"] = derive_roles(p)
    for i in range(len(flat)):
        for j in range(i + 1, len(flat)):
            sa, pa = flat[i]
            sb, pb = flat[j]
            if len(pa["colours"]) != len(pb["colours"]):
                continue
            d = match_distance(pa, pb)
            if d < DUP_DE:
                errors.append("near-duplicate palettes: %s/%s and %s/%s (worst dE %.3f)" % (sa, pa["name"], sb, pb["name"], d))
    return warnings, errors

# ----------------------------------------------------------------------------- emit

def emit_pkg(shelves, path):
    colours, pal_words, shelf_words, names = [], [], [], []
    for s in shelves:
        first_pal = len(pal_words)
        for p in s["palettes"]:
            off = len(colours)
            n = len(p["colours"])
            for c, r in zip(p["colours"], p["roles_eff"]):
                Y, U, V = c["yuv"]
                colours.append((Y << 22) | (U << 12) | (V << 2) | ROLE_BITS[r])
            pal_words.append(((1 if p["hushed"] else 0) << 15) | (off << 4) | (n - 1))
            names.append((s["name"], p["name"]))
        shelf_words.append((first_pal << 8) | len(s["palettes"]))

    def pow2(n):
        return 1 << max(1, (n - 1).bit_length())
    n_col, n_pal, n_shelf = len(colours), len(pal_words), len(shelf_words)
    col_depth, pal_depth, shelf_depth = pow2(n_col), pow2(n_pal), pow2(n_shelf)
    if col_depth > 2048:
        raise SystemExit("too many colours for an 11-bit offset: %d" % n_col)
    maxn = max(len(s["palettes"]) for s in shelves)

    out = []
    w = out.append
    w("-- Generated by gamut.py from palettes.txt -- DO NOT EDIT")
    w("-- %d shelves, %d palettes, %d colours" % (n_shelf, n_pal, n_col))
    w("library ieee;")
    w("use ieee.std_logic_1164.all;")
    w("use ieee.numeric_std.all;")
    w("")
    w("package gamut_palettes_pkg is")
    w("  constant C_NUM_SHELVES    : natural := %d;" % n_shelf)
    w("  constant C_NUM_PALETTES   : natural := %d;" % n_pal)
    w("  constant C_NUM_COLOURS    : natural := %d;" % n_col)
    w("  constant C_MAX_SHELF_LEN  : natural := %d;" % maxn)
    w("  constant C_COLOUR_DEPTH   : natural := %d;  -- power-of-2 ROM depth" % col_depth)
    w("  constant C_PALETTE_DEPTH  : natural := %d;" % pal_depth)
    w("  constant C_SHELF_DEPTH    : natural := %d;" % shelf_depth)
    w("")
    w("  -- colour word: Y[31:22] U[21:12] V[11:2] role[1:0]  (BT.601 10-bit legal range, UNSWAPPED;")
    w("  -- role 0 = dominant, 1 = secondary, 2 = accent, 3 = neutral)")
    w("  type t_colour_rom is array (0 to C_COLOUR_DEPTH - 1) of std_logic_vector(31 downto 0);")
    w("  -- palette word: [15] hushed, [14:4] first colour offset, [3:0] colour count - 1")
    w("  type t_palette_rom is array (0 to C_PALETTE_DEPTH - 1) of std_logic_vector(15 downto 0);")
    w("  -- shelf word: [15:8] first palette index, [7:0] palette count")
    w("  type t_shelf_rom is array (0 to C_SHELF_DEPTH - 1) of std_logic_vector(15 downto 0);")
    w("")
    w("  constant C_COLOUR_ROM : t_colour_rom := (")
    for i in range(col_depth):
        v = colours[i] if i < n_col else 0
        tail = "," if i < col_depth - 1 else ""
        w("    %4d => x\"%08X\"%s" % (i, v, tail))
    w("  );")
    w("")
    w("  constant C_PALETTE_ROM : t_palette_rom := (")
    for i in range(pal_depth):
        v = pal_words[i] if i < n_pal else 0
        tail = "," if i < pal_depth - 1 else ""
        note = ("  -- %s / %s" % names[i]) if i < n_pal else ""
        w("    %3d => x\"%04X\"%s%s" % (i, v, tail, note))
    w("  );")
    w("")
    w("  constant C_SHELF_ROM : t_shelf_rom := (")
    for i in range(shelf_depth):
        v = shelf_words[i] if i < n_shelf else 0
        tail = "," if i < shelf_depth - 1 else ""
        note = ("  -- %s" % shelves[i]["name"]) if i < n_shelf else ""
        w("    %2d => x\"%04X\"%s%s" % (i, v, tail, note))
    w("  );")
    w("end package gamut_palettes_pkg;")
    with open(path, "w") as f:
        f.write("\n".join(out) + "\n")
    return n_col, n_pal, n_shelf, col_depth

def emit_manifest(shelves, warnings, errors, path):
    data = {"shelves": [], "warnings": warnings, "errors": errors}
    idx = 0
    for s in shelves:
        sh = {"name": s["name"], "palettes": []}
        for p in s["palettes"]:
            sh["palettes"].append({
                "index": idx, "name": p["name"], "hushed": p["hushed"], "roles": p["roles_eff"],
                "roles_given": p["roles"] is not None, "src": p["src"], "l_span": round(p["l_span"], 3),
                "colours": [{"token": c["token"], "hex": c["hex"], "yuv": list(c["yuv"]),
                             "oklch": [round(v, 3) for v in c["lch"]]} for c in p["colours"]],
            })
            idx += 1
        data["shelves"].append(sh)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=1)

# ----------------------------------------------------------------------------- contact sheets

THEMES = {
    "dark":  {"bg": (24, 24, 28), "bg2": (34, 34, 40), "text": (225, 225, 230), "dim": (140, 140, 150),
              "shadow": (0, 0, 0, 150), "card": (40, 40, 46)},
    "light": {"bg": (246, 243, 237), "bg2": (236, 232, 224), "text": (40, 40, 44), "dim": (120, 118, 112),
              "shadow": (60, 60, 80, 70), "card": (255, 255, 255)},
}

def _font(size):
    from PIL import ImageFont
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()

def _rgb8(c):
    return tuple(int(round(max(0, min(1, v)) * 255)) for v in c["rgb"])

def draw_card(img, x, y, w, h, p, theme, radius=16, shadow=True):
    from PIL import Image, ImageDraw, ImageFilter
    t = THEMES[theme]
    if shadow:
        pad = 28
        sh = Image.new("RGBA", (w + 2 * pad, h + 2 * pad), (0, 0, 0, 0))
        ImageDraw.Draw(sh).rounded_rectangle((pad, pad + 8, pad + w, pad + h + 8), radius=radius, fill=t["shadow"])
        sh = sh.filter(ImageFilter.GaussianBlur(10))
        img.alpha_composite(sh, (x - pad, y - pad))
    card = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(card)
    n = len(p["colours"])
    for i, c in enumerate(p["colours"]):
        x0 = round(i * w / n)
        x1 = round((i + 1) * w / n)
        d.rectangle((x0, 0, x1, h), fill=_rgb8(c))
    mask = Image.new("L", (w, h), 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, w - 1, h - 1), radius=radius, fill=255)
    img.paste(card, (x, y), mask)

def render_shelf(shelf, num, theme, outdir):
    from PIL import Image, ImageDraw
    t = THEMES[theme]
    cols = 4
    cw, ch, gx, gy = 300, 150, 40, 76
    margin, header = 48, 90
    rows = math.ceil(len(shelf["palettes"]) / cols)
    W = margin * 2 + cols * cw + (cols - 1) * gx
    H = header + margin + rows * (ch + gy)
    img = Image.new("RGBA", (W, H), t["bg"] + (255,))
    d = ImageDraw.Draw(img)
    d.text((margin, 34), "%02d  %s" % (num, shelf["name"]), fill=t["text"], font=_font(30))
    d.text((W - margin - 220, 44), "%d palettes" % len(shelf["palettes"]), fill=t["dim"], font=_font(18))
    f_name, f_meta = _font(18), _font(14)
    for i, p in enumerate(shelf["palettes"]):
        x = margin + (i % cols) * (cw + gx)
        y = header + (i // cols) * (ch + gy)
        draw_card(img, x, y, cw, ch, p, theme)
        d = ImageDraw.Draw(img)
        d.text((x, y + ch + 12), "%02d  %s" % (i, p["name"]), fill=t["text"], font=f_name)
        meta = "%d colours" % len(p["colours"])
        if p["hushed"]:
            meta += "  · hushed"
        if p["roles"]:
            meta += "  · roles " + p["roles"]
        d.text((x, y + ch + 36), meta, fill=t["dim"], font=f_meta)
    path = os.path.join(outdir, "%02d_%s_%s.png" % (num, shelf["name"].lower().replace(" ", "_"), theme))
    img.convert("RGB").save(path)
    return path

def render_library(shelves, theme, outdir):
    from PIL import Image, ImageDraw
    t = THEMES[theme]
    sw, sh, gap, lab = 360, 26, 6, 150
    cols = 3
    margin = 40
    blocks = []
    for s in shelves:
        blocks.append((s, math.ceil(len(s["palettes"]) / cols)))
    H = margin
    for s, rows in blocks:
        H += 44 + rows * (sh + gap) + 24
    W = margin * 2 + cols * (lab + sw + 24)
    img = Image.new("RGBA", (W, H), t["bg"] + (255,))
    d = ImageDraw.Draw(img)
    f_h, f_n = _font(22), _font(13)
    y = margin
    for num, (s, rows) in enumerate(blocks):
        d.text((margin, y), "%02d  %s" % (num, s["name"]), fill=t["text"], font=f_h)
        y += 40
        for i, p in enumerate(s["palettes"]):
            x = margin + (i % cols) * (lab + sw + 24)
            yy = y + (i // cols) * (sh + gap)
            d.text((x, yy + 5), "%02d %s" % (i, p["name"]), fill=t["dim"], font=f_n)
            draw_card(img, x + lab, yy, sw, sh, p, theme, radius=7, shadow=False)
            d = ImageDraw.Draw(img)
        y += rows * (sh + gap) + 24
    path = os.path.join(outdir, "00_library_%s.png" % theme)
    img.convert("RGB").save(path)
    return path

def render_html(shelves, warnings, errors, outdir):
    rows = []
    for num, s in enumerate(shelves):
        rows.append('<h2>%02d &middot; %s <small>%d palettes</small></h2>' % (num, s["name"], len(s["palettes"])))
        for i, p in enumerate(s["palettes"]):
            sw = "".join(
                '<div class="sw" style="background:%s" title="%s  Y%d U%d V%d  L%.2f C%.2f h%.0f"><span>%s</span></div>'
                % (c["hex"], c["token"], *c["yuv"], *c["lch"], c["hex"][1:]) for c in p["colours"])
            tags = []
            if p["hushed"]:
                tags.append("hushed")
            tags.append("roles " + p["roles_eff"] + ("" if p["roles"] else " (derived)"))
            if p["src"]:
                tags.append("src " + p["src"])
            rows.append('<div class="pal"><div class="nm">%02d %s<br><small>%s</small></div><div class="card">%s</div></div>'
                        % (i, p["name"], " &middot; ".join(tags), sw))
    report = "".join("<li class=err>%s</li>" % e for e in errors) + "".join("<li>%s</li>" % w for w in warnings)
    html = """<!doctype html><meta charset="utf-8"><title>GAMUT library</title>
<style>
:root{color-scheme:light dark}
body{font:14px system-ui,sans-serif;margin:32px;background:#181a1c;color:#e4e4e8}
body.light{background:#f6f3ed;color:#2a2a2c}
h2{margin:40px 0 12px;font-weight:600} h2 small{color:#888;font-weight:400;margin-left:8px}
.pal{display:flex;align-items:center;gap:16px;margin:8px 0}
.nm{width:230px;flex:none} .nm small{color:#8a8a90}
.card{display:flex;flex:none;width:560px;height:72px;border-radius:12px;overflow:hidden;box-shadow:0 8px 20px rgba(0,0,0,.35)}
body.light .card{box-shadow:0 6px 18px rgba(60,60,90,.18)}
.sw{flex:1;position:relative} .sw span{position:absolute;left:4px;bottom:3px;font:10px ui-monospace,monospace;color:#fff;mix-blend-mode:difference;opacity:.8}
button{position:fixed;top:16px;right:16px;padding:8px 14px;border-radius:999px;border:1px solid #888;background:transparent;color:inherit;cursor:pointer}
ul{color:#c9a227} li.err{color:#e5484d}
</style>
<button onclick="document.body.classList.toggle('light')">theme</button>
<h1>GAMUT library</h1>
<p>Hover a swatch for its token, 10-bit YUV and OKLCH. Generated by gamut.py from palettes.txt.</p>
<ul>%s</ul>
%s
""" % (report or "<li>no warnings</li>", "\n".join(rows))
    path = os.path.join(outdir, "index.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
    return path

def render_sheets(shelves, warnings, errors):
    os.makedirs(CONTACT, exist_ok=True)
    for old in os.listdir(CONTACT):
        if old.endswith(".png"):
            os.remove(os.path.join(CONTACT, old))
    paths = []
    for theme in ("dark", "light"):
        paths.append(render_library(shelves, theme, CONTACT))
        for num, s in enumerate(shelves):
            paths.append(render_shelf(s, num + 1, theme, CONTACT))
    paths.append(render_html(shelves, warnings, errors, CONTACT))
    return paths

# ----------------------------------------------------------------------------- main

def main(argv):
    sheets = "--sheets" in argv
    shelves, errors = parse(SRC)
    warnings, check_errors = run_checks(shelves)
    errors += check_errors
    for wmsg in warnings:
        print("warning: " + wmsg)
    for e in errors:
        print("ERROR: " + e, file=sys.stderr)
    emit_manifest(shelves, warnings, errors, os.path.join(CONTACT, "manifest.json"))
    if sheets:
        for pth in render_sheets(shelves, warnings, errors):
            print("wrote " + os.path.relpath(pth, HERE))
    if errors:
        return 1
    n_col, n_pal, n_shelf, depth = emit_pkg(shelves, PKG)
    per_shelf = ", ".join("%s %d" % (s["name"], len(s["palettes"])) for s in shelves)
    print("gamut: %d shelves, %d palettes, %d colours -> ROM depth %d (%d EBR at 32 bits); %d warnings"
          % (n_shelf, n_pal, n_col, depth, depth * 32 // 4096, len(warnings)))
    print("       " + per_shelf)
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
