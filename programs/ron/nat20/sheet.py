#!/usr/bin/env python3
"""Preview sheet: renders a grid of model frames into one PNG."""
import sys
import numpy as np
from PIL import Image, ImageDraw
import nat20_model as M


def sheet(cells, out, W=640, H=360, cols=3):
    imgs = []
    for c in cells:
        c = dict(c)
        label = c.pop("label", "")
        if "face" in c:
            R = M.present(c["die"], c.pop("face"), tilt=c.pop("tilt", None))
        else:
            R = M.rot(c.pop("yaw", 25), c.pop("pitch", -24), c.pop("roll", 0))
        im = Image.fromarray(M.render(R=R, W=W, H=H, **c))
        if label:
            ImageDraw.Draw(im).text((8, 6), label, fill=(30, 30, 30))
        imgs.append(np.array(im))
    rows = [np.concatenate(imgs[i:i + cols] + [np.full_like(imgs[0], 255)] * (cols - len(imgs[i:i + cols])), 1)
            for i in range(0, len(imgs), cols)]
    Image.fromarray(np.concatenate(rows, 0)).save(out)
    print("wrote", out)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "dice"
    if which == "dice":
        sheet([
            dict(die="d4", face=0, color="emerald", ink="gold", label="d4 Emerald / Gold"),
            dict(die="d6", face=1, color="bone", ink="black", label="d6 Bone / Black"),
            dict(die="d8", face=2, color="midnight", ink="gold", label="d8 Midnight / Gold"),
            dict(die="d10", face=3, color="amethyst", ink="white", label="d10 Amethyst / White"),
            dict(die="d20", face=4, color="crimson", ink="white", label="d20 Crimson / White"),
            dict(die="d6", face=0, color="ember", ink="white", pips=True, label="d6 pips Ember"),
        ], "prev_dice.png")
    if which == "finish":
        sheet([dict(die="d20", face=4, color=c, ink=i, finish=f, label=f"{f} / {c} / {i}")
               for f, c, i in [("gloss", "crimson", "white"), ("matte", "lagoon", "white"),
                               ("pearl", "frost", "silver"), ("marble", "midnight", "gold"),
                               ("gem", "amethyst", "white"), ("glitter", "obsidian", "silver"),
                               ("wood", "ember", "black"), ("metal", "sunflower", "black"),
                               ("metal", "frost", "black")]], "prev_finish.png")
    if which == "ink":
        sheet([dict(die="d6", face=1, color="crimson", ink=i, label=f"ink {i}")
               for i in ["white", "gold", "silver", "black", "red", "tonal", "unpainted"]]
              + [dict(die="d6", face=1, color="bone", ink="unpainted", label="bone unpainted"),
                 dict(die="d6", face=1, color="crimson", ink="unpainted", edges="sharp", label="sharp edges")],
              "prev_ink.png")
    if which == "check":
        sheet([dict(die="d20", face=f, color="crimson", ink="white", label=f"d20 face idx {f}") for f in range(20)]
              + [dict(die="d10", face=f, color="amethyst", ink="white", label=f"d10 face idx {f}") for f in range(10)]
              + [dict(die="d8", face=f, color="midnight", ink="gold", label=f"d8 face idx {f}") for f in range(8)]
              + [dict(die="d6", face=f, color="bone", ink="black", label=f"d6 face idx {f}") for f in range(6)]
              + [dict(die="d4", face=f, color="emerald", ink="gold", label=f"d4 vertex-present {f}") for f in range(4)],
              "prev_check.png", W=480, H=270, cols=6)
    if which == "light":
        sheet([dict(die="d20", face=4, color="emerald", ink="unpainted", light=l, label=f"P12 light {l:.2f}")
               for l in [0.0, 0.17, 0.33, 0.5, 0.67, 1.0]], "prev_light.png")
