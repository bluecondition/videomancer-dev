"""GILT style lab: shared helpers for the Phase 1 reference analysis.

Everything here is deliberately plain and documented, because these scripts become
the conformance tests of spec section 7 (they will be run on model renders later).

Conventions
- Images are sRGB 8-bit. Colour science in CIELAB (D65) via scikit-image.
- "Y10/Cb10/Cr10" = BT.709 Y'CbCr, 10-bit legal range (Y 64..940, C 512 +/- 448),
  computed from gamma-encoded R'G'B' -- the convention of the spec's section 3.2.
  (Hardware constants are generated later from sRGB; this is for reporting only.)
- Spatial scales are fractions of the canvas WIDTH, so results do not depend on the
  resolution of the reproduction.
"""
import json
import os

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage as ndi
from skimage import color

Image.MAX_IMAGE_PIXELS = None

HERE = os.path.dirname(os.path.abspath(__file__))
GILT = os.path.dirname(HERE)
REFS = os.path.join(GILT, "refs")
FIGS = os.path.join(HERE, "figs")
ZONES = json.load(open(os.path.join(HERE, "zones.json")))

PAINTINGS = ["kiss", "adele1", "stoclet_fulfillment", "stoclet_expectation",
             "stoclet_treeoflife", "danae", "judith1", "fritza_riedler", "hope2",
             "water_serpents1"]
ZONED = ["kiss", "adele1", "stoclet_fulfillment"]
# in-situ photo: perspective + room light, used for qualitative motif notes only
QUALITATIVE = ["beethoven_kiss"]


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------
def load(key, long_side=2048, short_side=None):
    """Return (rgb uint8 HxWx3, crop box in full-image px, full image size).
    The canvas crop from zones.json is applied, then the image is box-filtered
    (area average) down so its long side is at most `long_side` (None = native).
    short_side=1080 instead gives the "1080-equivalent" scale: the painting's short
    side mapped onto 1080 video lines, the scale at which texture is judged."""
    im = Image.open(os.path.join(REFS, key + ".jpg")).convert("RGB")
    W, H = im.size
    x0, y0, x1, y1 = ZONES["canvas"][key]
    box = (round(x0 * W), round(y0 * H), round(x1 * W), round(y1 * H))
    im = im.crop(box)
    if short_side:
        s = short_side / min(im.size)
        im = im.resize((round(im.width * s), round(im.height * s)), Image.BOX)
    elif long_side and max(im.size) > long_side:
        s = long_side / max(im.size)
        im = im.resize((round(im.width * s), round(im.height * s)), Image.BOX)
    return np.asarray(im), box, (W, H)


def label_map(key, shape, box, full):
    """Rasterise the zone layers of zones.json. Returns (labels uint8, names)."""
    names = ["field"]
    lab = np.zeros(shape[:2], np.uint8)
    spec = ZONES["paintings"].get(key)
    if spec is None:
        return lab, names
    W, H = full
    sx = shape[1] / (box[2] - box[0])
    sy = shape[0] / (box[3] - box[1])
    for layer in spec["layers"]:
        name = layer["label"]
        if name not in names:
            names.append(name)
        m = Image.new("L", (shape[1], shape[0]), 0)
        d = ImageDraw.Draw(m)
        for poly in layer["polys"]:
            d.polygon([((x * W - box[0]) * sx, (y * H - box[1]) * sy) for x, y in poly],
                      fill=1)
        m = np.asarray(m).astype(bool)
        if "within" in layer:
            m &= np.isin(lab, [names.index(w) for w in layer["within"] if w in names])
        lab[m] = names.index(name)
    return lab, names


def group_masks(lab, names):
    """Masks for the spec zone groups (flesh, hair, robe, field, meadow)."""
    out = {}
    for i, n in enumerate(names):
        g = ZONES["group"][n]
        out[g] = out.get(g, np.zeros(lab.shape, bool)) | (lab == i)
    return out


def label_masks(lab, names):
    return {n: lab == i for i, n in enumerate(names)}


# ---------------------------------------------------------------------------
# colour
# ---------------------------------------------------------------------------
def to_lab(rgb8):
    return color.rgb2lab(rgb8).astype(np.float32)


def lab_to_rgb8(L, a, b):
    rgb = color.lab2rgb(np.array([[[L, a, b]]], dtype=np.float64))[0, 0]
    return tuple(int(round(v * 255)) for v in np.clip(rgb, 0, 1))


def ycbcr709_10(rgb8):
    f = np.asarray(rgb8, np.float32) / 255.0
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    y = 0.2126 * r + 0.7152 * g + 0.0722 * b
    return (64 + 876 * y, 512 + 896 * (b - y) / 1.8556, 512 + 896 * (r - y) / 1.5748)


def hexrgb(rgb):
    return "#%02X%02X%02X" % tuple(int(v) for v in rgb)


def lch(lab3):
    L, a, b = lab3[..., 0], lab3[..., 1], lab3[..., 2]
    return L, np.hypot(a, b), np.degrees(np.arctan2(b, a)) % 360.0


# Pixel families (spec 3.2 "coverage budgets"). Thresholds in CIELAB:
#   black   : L* < 20                                   (inlay, hair, slits)
#   gold    : hue 40-105 deg and C* >= 12, or the dark browns hue 30-105, C* >= 8, L* < 45
#             (umber .. brass .. orange gold .. cream all count as the gold family)
#   green   : hue 105-175 and C* >= 12                  (meadow, floor)
#   red     : hue < 40 or >= 330, C* >= 20              (crimson, rose)
#             or hue 40-58 with C* >= 40                (vermilion / red-orange enamel; measured
#             bimodal: enamel reds peak at 40-50 deg, golds at 65-90 deg incl. orange gold)
#   violet  : hue 280-330, C* >= 20
#   blue    : hue 175-280, C* >= 20                     (lapis, cobalt, turquoise)
#   neutral : everything else (flesh, silver, white, grey: low chroma)
FAMILIES = ["gold", "black", "neutral", "green", "red", "violet", "blue"]


def classify(lab3):
    L, C, h = lch(lab3)
    fam = np.full(L.shape, FAMILIES.index("neutral"), np.uint8)
    fam[((h >= 40) & (h < 105) & (C >= 12)) | ((h >= 30) & (h < 105) & (C >= 8) & (L < 45))] = 0
    fam[(h >= 105) & (h < 175) & (C >= 12)] = FAMILIES.index("green")
    fam[(((h < 40) | (h >= 330)) & (C >= 20)) | ((h >= 40) & (h < 58) & (C >= 40))] = FAMILIES.index("red")
    fam[(h >= 280) & (h < 330) & (C >= 20)] = FAMILIES.index("violet")
    fam[(h >= 175) & (h < 280) & (C >= 20)] = FAMILIES.index("blue")
    fam[L < 20] = FAMILIES.index("black")
    return fam


def coverage(fam, mask=None):
    f = fam[mask] if mask is not None else fam.ravel()
    n = max(f.size, 1)
    return {name: float(np.count_nonzero(f == i)) / n for i, name in enumerate(FAMILIES)}


# ---------------------------------------------------------------------------
# k-means in CIELAB (k-means++ init, deterministic)
# ---------------------------------------------------------------------------
def kmeans(X, k, iters=50, seed=0, sample=150000):
    rng = np.random.default_rng(seed)
    if len(X) > sample:
        X = X[rng.choice(len(X), sample, replace=False)]
    X = X.astype(np.float64)
    centers = [X[rng.integers(len(X))]]
    d2 = ((X - centers[0]) ** 2).sum(1)
    for _ in range(1, k):
        centers.append(X[rng.choice(len(X), p=d2 / d2.sum())])
        d2 = np.minimum(d2, ((X - centers[-1]) ** 2).sum(1))
    C = np.array(centers)
    for _ in range(iters):
        lab_ = ((X[:, None, :] - C[None]) ** 2).sum(2).argmin(1)
        newC = np.array([X[lab_ == j].mean(0) if np.any(lab_ == j) else C[j] for j in range(k)])
        if np.allclose(newC, C, atol=1e-3):
            break
        C = newC
    lab_ = ((X[:, None, :] - C[None]) ** 2).sum(2).argmin(1)
    w = np.bincount(lab_, minlength=k) / len(X)
    order = np.argsort(-w)
    return C[order], w[order]


def palette_entries(centers, weights):
    out = []
    for (L, a, b), w in zip(centers, weights):
        rgb = lab_to_rgb8(L, a, b)
        Y, Cb, Cr = ycbcr709_10(np.array(rgb, np.float32))
        out.append(dict(hex=hexrgb(rgb), weight=round(float(w), 4),
                        lab=[round(float(L), 1), round(float(a), 1), round(float(b), 1)],
                        y10=int(round(float(Y))), cb10=int(round(float(Cb))),
                        cr10=int(round(float(Cr)))))
    return out


# ---------------------------------------------------------------------------
# band energy (always measured at the 1080-equivalent scale, see load())
# ---------------------------------------------------------------------------
# DoG sigma pairs in px at 1080-equivalent:
BANDS = {
    "grain":  (0.5, 1.5),    # 1-3 px: leaf texture, glitter, dust
    "detail": (1.5, 5.0),    # 3-10 px: rims, small motifs, flecks
    "motif":  (5.0, 20.0),   # 10-40 px: cells, discs, bars
}


def band_energy(Y, masks, erode_px=12):
    """Mean |DoG| of Y10 per mask, per band. Masks are eroded by erode_px so zone
    boundaries themselves do not count as texture."""
    res = {}
    em = {k: ndi.binary_erosion(m, iterations=erode_px) if m.any() else m for k, m in masks.items()}
    for bname, (s1, s2) in BANDS.items():
        a = np.abs(ndi.gaussian_filter(Y, s1) - ndi.gaussian_filter(Y, s2))
        res[bname] = {k: (round(float(a[m].mean()), 1) if m.sum() > 100 else None) for k, m in em.items()}
    return res
