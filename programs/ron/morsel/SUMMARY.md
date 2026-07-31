# MORSEL

**The whole screen is one chocolate-chip cookie, and the video is baked into
it.**

Forked from the single "choc-chip cookie" swatch inside **Sugarcoat** — which
was one of eight candy materials there, drawn as a 4-way-flipped triangle in
an 8/16/32/64 px cell. Here that one material becomes the entire program, and
the chip is rebuilt from an actual 3-D model.

---

## The chip

`genchips.py` builds a real morsel — flat circular base, sides bulging up from
it, tip curling over to one side — as a solid of revolution with a curled
axis, then **photographs it from 8 different 3-D orientations** and stores each
projected **silhouette** as a polar radius table `R(theta)`, 128 steps.

| variant | elevation | reads as |
|---|---|---|
| 0 | 0° (side-on) | classic morsel: flat bottom, rounded shoulders, hooked tip |
| 1-3 | 9-32° | tipped teardrops, base foreshortening into an ellipse |
| 4-5 | 45-58° | three-quarter view, strong hook |
| 6-7 | 72-87° (top-down) | round chip with a small point off one side |

A solid of revolution projects to the **union of its stacked disks**: a
horizontal disk of radius `r` at height `z` seen from elevation `phi` becomes
an ellipse centred `(cx, z·cos phi − cy·sin phi)` with semi-axes
`(r, r·sin phi)`. `shapes.png` is the contact sheet of all 8.

Why polar rather than a rotation matrix: the pixel path already has to work
out its angle about the chip centre (the rim light needs it), so a
radius-vs-angle table buys an **arbitrary** silhouette — flat bottom, curl,
point — for one BRAM read and one compare. **Rotating a chip is then a
subtract on the angle index**: free, and continuous at 2.8° steps rather than
the 4 flips it was forked from.

Per chip, from the cell hash: free 128-step rotation, ±19% size jitter,
position jitter scaled to whatever slack the cell has left, dark/milk
chocolate, and (S11) white chocolate.

**Lighting** — one fixed upper-left rim light per chip: a dome gradient whose
strength rises toward the rim (where a convex body's normal swings outward),
a tight specular glint on the lit shoulder, a dark contact edge opposite, and
a soft shadow thrown onto the dough beside it. Chips and dough both carry
hash grain, so nothing is a flat fill.

## Baking the video in

Two mechanisms, deliberately at different frequencies:

* **Dough tone** follows the video luma pixel-for-pixel through the Bake ramp
  — fine detail, and always stays a cookie colour at both ends.
* **Chip density and size** follow a **per-cell** luma sample (S7) — the
  coarse halftone that makes a face read from across the room. Dark cells get
  more, and bigger, chips.
* S10 **Emboss** adds relief from the video's own gradient, so the cookie
  surface is sculpted by the picture rather than merely tinted by it.
* S8 **Tint** lets the dough take the video's chroma as well.

## Anti-lattice (the one non-obvious structural decision)

A plain cell grid reads as **rows of dots**, and per-cell jitter can never fix
it: a chip may only wander as far as the slack left inside its own cell, which
is nearly nothing once the chips are large. So every cell **column** is slid
vertically by its own pseudo-random phase. The cell boundaries move with the
chip, so nothing is ever clipped, and the rows disappear entirely.

## Controls

| | |
|---|---|
| K1 | **Scale** — cell 8 / 16 / 32 / 64 / 128 px |
| K2 | **Shape** — which slice of the 8 3-D orientations: side-on teardrops ↔ round chips with a point |
| K3 | **Bake** — pale golden dough → dark baked |
| K4 | **Relief** — shading depth + specular glint |
| K5 | **Grain** — dough surface texture |
| K6 | **Chip Size** |
| P12 | **Chips** — density (headline: plain dough → loaded) |
| S7 | **Luma Mod** — image drives chip density + size |
| S8 | **Tint** — dough takes the video's chroma |
| S9 | **Invert** — flip the luma polarity |
| S10 | **Emboss** — relief from the video gradient |
| S11 | **White Chips** — white chocolate in the mix |

## Implementation

Streaming, `C_LATENCY = 21`, no CORDIC and no per-pixel divides.

```
1-5    cell lattice: column -> column hash -> vertical phase -> cell row
6-9    cell hash -> rotation/variant/size/jitter/presence + per-cell luma
10-12  polar: |cx|,|cy| -> C_RTAB -> exact radius + octant fraction -> angle
13-15  C_SHAPE silhouette radius, C_COS rim light, size multiply
16-17  signed distance inside the silhouette; coverage + shading DECISIONS
18-19  one shading add, one dough add, chip and dough colours
20-21  alpha scale, composite
```

`C_RTAB` (1024×12) returns **both** `radius*4` and the octant sub-angle from
one read — no square root and no divider anywhere in the pixel path. Only
three multiplies exist at all (chip size, bake ramp, noise gain); the
composite is a shift-add because alpha only spans 0..4.

Everything with a ladder or a multiply in it — the size table, the jitter
table, the shape window, the bake constants — is built one item per cycle by
the **vblank sequencer**, so the pixel path only ever sees a table read or a
mux.

`preview_morsel.py` is a numpy transcription of the exact integer pipeline; it
renders a full frame in a second and is what the design was tuned against.

### Timing close (HD needs 74.25 MHz)

First build: **57 MHz**. Every fix came from profiling the nextpnr critical
path (`--timing-allow-fail -l log`, read "Critical path report"), never from
guessing — the classic symptom was there, an Fmax that barely moved with big
area changes.

1. **The dough stage** did two grain multiplies and five adds in one cycle.
   The grain now finishes 14 stages earlier (free-running noise needs no
   alignment), the emboss is pre-shifted at insertion rather than where it
   meets the dough tone, and the cast shadow is pre-applied into a second
   offset so the final stage picks one and adds once. **57 → 66 MHz.**
2. **The vblank sequencer** then became the critical path — a reminder that
   per-frame logic is timed at the pixel clock like everything else (same trap
   sugarcoat hit). Its chains were split across slots, the jitter-room
   calculation `(126 − 1.73·szb)/4` became the `C_JMAX` table, every
   non-constant multiply was removed, and the slot ranges were **aligned to
   power-of-two boundaries** so a table index is just the low bits of `s_seq`
   — no subtract, and the range decode is bit-matching. **66 → 73 → 93 MHz**
   pre-route.
3. Barrel-shifting the grain coordinates straight into the hash put
   `s_scl → shift → hash` in one cone; the shift is now registered.

All 6 configs close with margin (68% LC, 11/32 BRAM, no clock divisor):

| config | Fmax | required |
|---|---|---|
| HD Analog | 80.69 MHz (seed 2) | 74.25 |
| SD Analog | 80.10 MHz | 27 |
| HD HDMI | 79.13 MHz | 74.25 |
| SD HDMI | 82.13 MHz | 27 |
| HD Dual | 85.01 MHz | 74.25 |
| SD Dual | 80.63 MHz | 27 |

Note: **HD Analog seed 1 stalls router2** (reproducibly — both full builds hit
it); seed 2 routes in ~390 s and passes. Build with `SEED=2` if iterating on
that config alone.

### Hash trap (worth remembering)

The first `f_pack` was `(x & y_low) xor (y & x_low)` — which, worked through,
**depends only on `x xor y`**. Every cell on a diagonal shared a hash
(measured diagonal correlation 0.29, visible as cross-hatching in the dough),
and the column phase — whose second argument was `x xor const` — collapsed to
a **single constant for every column**, silently restoring the exact row
lattice the phase was there to destroy. Rotating one half before the xor
fixes both (diagonal correlation −0.03).

## Colour convention

Authored in **standard BT.601** (U = Cb, V = Cr) and swapped at both ends,
because the Videomancer carries chroma U↔V swapped — same as sugarcoat and
gilt. Headless BT.601 sims therefore show the cookie blue; the Python preview
renders the intended colours.

## Status

All 6 configs built and timing-closed, `.vmprog` packaged.
**Not yet hardware-tested.**
