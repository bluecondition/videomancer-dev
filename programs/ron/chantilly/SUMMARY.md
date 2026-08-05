# Chantilly — black-lace halftone

**v0.1** (2026-08-03). Source: a photo of black Chantilly-style lace — bands of
white/cream dots (fans, rosettes, pebble rows) on black, with a thick black
wavy cordonnet breaking over the dots. The program re-renders incoming video
as that lace.

## The grid-free mapping

The brief demanded circles "geometrically arranged, not on a grid". The trick:

- The frame is cut into horizontal **lace bands** (256 lace units tall). Each
  band owns a row of **scallop centres** (pitch P, half-P staggered on
  alternate bands — one XOR on a local-x bit).
- Every pixel goes to polar (r, θ) about its nearest centre via a **CORDIC
  vectoring pipeline** (10 iterations, 2/stage, 5 guard bits — hypnos
  playbook, +1 stage for angle precision since the *angle* here rasterises
  dot edges).
- Dots sit on **concentric arcs**: `ring = r/G`, and dots-per-ring = `6·ring`
  — so arc cells stay ≈square at every radius (2πr/6·ring ≈ 1.05·G) but
  **never align radially**. Phyllotaxis-like, zero Cartesian structure.
- Dot test in **normalized cell units** (penrose trick): `frac_r² + frac_θ²
  < radius²`, all Q7, one compare renders every dot at every scale.
  `frac_θ` comes from `θ·ring·6 mod 4096` — the mod-4096 slice makes the
  angular wrap seamless by construction.
- **Ring 0** is the fan's centre "eye": distance taken straight from r.
- **Twist (K6)** adds `ring·k` cells of per-ring rotation — phyllotaxis shear.

Four band styles cycle down the frame (bits of the band index):

| style | centre | P | G | look |
|---|---|---|---|---|
| 0 PEBBLE | 2 bands deep | 512 | 32 | gentle arcs of medium circles |
| 1 FAN | band base | 512 | 32 | radiating spiderweb fans |
| 2 SEED | 2 bands deep | 512 | 16 | dense small-dot packing |
| 3 BLOOM | mid-band | 256 | 16 | full rosette flowers |

S10 Weave B replaces fans/blooms with pebble/seed (calm set).

## Video mapping

- Dot radius = `clamp(bias + luma·gain>>8)` from the **delayed input luma**
  (analog halftone): bright = fat merging cream dots, dark = pinpricks.
  K1 Fill = floor, K2 Contrast = slope.
- **Cordonnet for free**: `r > R_edge(style) − K5` is painted black — the
  fan's own outer radius IS a scalloped thick border, cusping exactly at fan
  seams. No sine tables. A thin seam wisp at |dx|≈P/2 joins the cusps.
- **Ink (S9/K4)**: horizontal luma gradient (4-px baseline, 6-px dilate) →
  black strokes cutting down through the dots.
- **Scale (P12)**: screen x,y are accumulated in Q8 lace units (`u += s` per
  pixel) so every pitch stays a power of two in lace space while the screen
  scale zooms continuously — no variable modulo anywhere. SD gets an extra
  >>1. **Drift (K3)** scrolls lace space; everything wraps mod 4096 lace
  units so the 16-band cycle is seamless.
- S8 invert (black net on white), S7 ivory/mono tint, S11 ghost (dim video
  in the black ground). Output blanking-gated (Y=64, U=V=512 outside avid).

## Implementation notes

- Streaming, no BRAM, no line buffer. Per-pixel multiplies, each in its own
  stage: θ×ring (13×7), ring×twist (7×8), three Q7 squares, luma×gain (8×9).
- 18-stage pipeline; luma/ink/ghost taps counted to land exactly at the
  compose stage (see stage numbers #n in the source).
- vblank sequencer: one op per slot, shared multiplier for (W/2)·s and
  (H/2)·s; vsync trigger guarded with `s_lcnt > 8` so serrated analog vsync
  can't double-step the drift.
- Traps dodged (from the memory playbooks): signed-resize-on-abs (|dy| hits
  exactly 2048 — sliced, not resized), 10-bit overflow in the Y map (64+1020
  — widened to 11), concat-into-variable to normalise `&` ranges, toggle
  switches on registers_in(6) bits 0–4.

## Status

- v0.1 all 6 configs pass POST-ROUTE, ~5320 LC (69%), 0 BRAM:
  HD Analog 77.33 (seed 2), HD HDMI 77.04 (seed 1), HD Dual 74.96 (seed 1)
  vs 74.25 required; SD 71–78 vs 27. Packaged to out/rev_b/ron/chantilly.vmprog.
- Timing history: first build's hd_hdmi needed 6 seeds and closed at 74.37 —
  routed critical path was the single C1 stage (18-bit subtract → clamp →
  invert mux → cut mux → ghost zero-test in one cone). Splitting C1 into
  C1a (subtract+clamp) / C1b (compose muxes) closed every HD config at
  seed 1–2 with real margin.
- **Not HW-tested.**
