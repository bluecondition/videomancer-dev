# Overlook

**The Overlook Hotel carpet (David Hicks' "Hexagon" from The Shining) as a
zoomable pattern generator** — v1.0, built from the user's exact vector
spec.  `proto_v22.png` is the accepted prototype.

## The pattern (definitive spec)

- The orange figure is a set of identical horizontal **serpentine
  polylines** stacked vertically at the same phase.  Their upward peaks
  and downward V-tips interlock without touching.  The hexagon "cells"
  are pure **negative space** — there are no closed shapes.
- Solid red hexagons float in the dark field; none touch the orange.
- User tile 100×142 px, stroke 12.5, **miter joins**; scaled ×10.24 →
  1024×1454 texture units, stroke 128.  Half-period polyline (x-fold
  symmetric about 0 and 512):
  `A(0,0) B(374,210) C(374,650) D(143,773) E(143,1213) F(512,1423)`.
- Red hexagons: regular pointy-top, circumradius 169, at (0,430) and
  (512,993) per tile.
- Colors (user spec): plum #341A23, orange #F05632, red #B41E37.

## How it works

- The stroke is evaluated **exactly**: per-segment perpendicular slabs
  clipped by the miter lines at each join — 24 half-plane tests
  (segments 1–5 of this row, segment 5 of the row above whose V-tip dips
  in, segment 1 of the row below whose apex pokes up).  All plane
  normals quantize to shift-add slopes, so the whole figure needs just
  **five shared x-products** (9/16, 17/32, 73/128, 19/32, 37/64 of the
  folded x) and **five 12-bit adders**; every plane is then a constant
  compare.  Hexagons: `37/64·|dx| + |dy| ≤ 169` and `|dx| ≤ 146`.
- Pipeline: R1 fold, R2 products, R3 sums, R4 compares+select, R5 paint.
  No BRAM, no line buffer, no pixel-rate multiply.  v1.1: all pattern
  coordinates carry 4 FRACTIONAL bits (edges quantize at 1/16 texel) and
  every slope product is a single wide sum with ONE final shift — both
  fixes for the ragged-edge regression (multi-truncated shift sums made
  irregular staircases).
- **Zoom DDA** (P12): u 16-bit free-wrap (tile = 1024), v 17-bit wrap
  93056 by conditional subtract.  v1.1 zoom law
  `du = (128 + t[6:0]) << t[9:7] >> 2` is CONTINUOUS across octave
  boundaries (the old 64+t[6:0] law jumped backwards ~33% at every
  octave crossing = the "random jumps"), and s_du GLIDES to the target
  through a 1/16-per-frame IIR that also swallows P12 ADC jitter.
  Screen-centre anchoring via measured W/H through the shared vblank
  multiplier; interlace doubles line step + offsets odd fields.
- Palette U/V-swapped for hardware (mondrian convention).

## Controls (v1.2)

| Control | Function |
|---|---|
| K1/K2 | **Drift X/Y** — bipolar texture drift, deadbanded at centre |
| K3 | **Glide** — zoom smoothing strength (1/2..1/256 per frame) |
| K4 | **Weight** — serpentine stroke width, ~4..20 px at spec scale |
| K5 | **Hex Size** — red hexagons ~60..140% |
| K6 | **Scheme** — five stepped palettes: Classic / Gold Room / Lobby Blue / Deep Night / Mono Amber |
| S7 | **Glide** on/off (off = raw slider tracking) |
| S8 | **Weave** — texture-anchored fibre dither on luma |
| S9 | **Video Ground** — dark field becomes dimmed input video |
| S10 | **Breathe** — slow ±3% zoom triangle, ~17 s cycle |
| S11 | **Night** — field ¼, orange ½, reds full |
| P12 | **Zoom** — exponential, continuous, centre-anchored, glided |

All control math (palette select, slab bounds, drift, breathe) runs per
frame in a 63-step vblank sequencer through one shared registered
multiplier — zero pixel-rate cost.  The miter clip lines never move;
only slab half-widths and hex bounds are per-frame registers.
Timing lessons from this rev: a smooth 9-channel palette crossfade
(double-variable palette mux into the shared multiplier) built a
17 ns sequencer cone — stepped schemes replaced it; and the glide's
variable barrel shift had to be pipelined over three sequencer steps
(diff / shift / apply) to keep it off the critical path.

## Build (2026-07-19, v1.2)

All 6 configs full clock (HD Analog seed 2, rest seed 1): 77.8–85.1 MHz
on the HD configs (SD floors trivially met), 4,576–4,593 LCs (60%),
0 BRAM.  Packaged to `out/rev_b/ron/overlook.vmprog` (v1.2.0).
Not HW-tested.

## History

Eleven geometry iterations from verbal specs and image readings (closed
hexagons, stems, eyes, channels, necks — all wrong) until the user
supplied the exact serpentine polyline vertices.  Lesson recorded in
memory: for a known GRAPHIC design, ask for exact vector geometry first.

## Ideas / next mods (discussed, not built)

- Slow drift/scroll (walk u0/v0 per frame)
- Luma-modulated stroke or field from input video
- Alternate palettes (Kubrick blue lobby, gold ballroom)
