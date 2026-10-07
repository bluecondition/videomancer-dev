# Overlook

**The Overlook Hotel carpet (David Hicks' "Hexagon" from The Shining) as a
zoomable pattern generator** — v1.0 built from the user's exact vector
spec (`proto/proto_v22.png` is the accepted prototype); v1.3 (2026-10-05) adds a
corridor-floor perspective, a video halftone and the hardware contracts.

**Status: v1.3.1 FINAL except presets — signed off by Ron 2026-10-06.**
Presets are hand-set placeholders; recapture with `./capture_preset.sh`.
Build with router1.

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

## v1.3 (2026-10-05) — finalization pass

**Per-line perspective engine.**  Runs during the previous line (57 steps
on the shared vblank multiplier, swapped in at active-video rise; line 0 is
recomputed on every blanking hsync before the field, idempotently).  For
frame row s' from centre (|s| under Mirror):
`D = 64*H/2 + K*s'`, `r = (H/2 << 18)/D` (16-step serial divider, Q4.12),
`du = du0*r` (6 frac bits), `u0 = ucen - du*W/2`, `v = vcen + s'*du`
(mod 93056, 9-step ladder).  K = 0 reproduces the flat carpet exactly
(r = 4096).  This is a true ground-plane homography with square texels at
the centre row; the horizon enters from the top at K = 25% and sits 3/8
down at 100%.  Beyond the horizon (D <= H/32*2) clamps and paints black.
Fog = min(depth table linear in 1/r — i.e. in screen rows — full at r <= 2;
density fade du 3072..5120).  Depth-linear fog was only 9-34 rows deep and
read as a hard horizon.  The engine also pre-fades the palette per line
(fog x Night) via a generic lerp pipeline, so the pixel path only selects.
The u accumulator gained 6 fractional bits (22-bit): integer per-line du
would jitter edges ~5 px at the screen sides.

**Contract fixes:** program_type `processing` (S9 reads data_in; synthesis
routes no input = green); outputs gated to neutral outside avid; serrated-
vsync saw-active guard (each extra edge re-ran the sequencer: drift/breathe/
glide sped up and `s_ilace` cleared itself); bottom field = field_n '0'
(was the top field — a crawling comb on every diagonal); latency 5 -> 6
(the colour was one clock behind avid); Night fades toward black 64 (was
y/4 = Y 23..46, below black).

**Control changes:** K3 Glide + S7 Glide (two controls for something you
can't see) -> fixed 1/16 glide, K3 Perspective, S7 Mirror.  Drift steps by
d*du (constant screen speed, was ~250x slower zoomed out).  Zoom law over 6
octaves (du 32..2040, tile 2048..32 px; was 8 px = aliasing).  Breathe
+-25% / 8.5 s (was +-3%).  Weave = 3-round add-mixed hash (max neighbour
correlation 0.08; the old xor-only hash made diagonal stripes), +-32 codes,
cell ~1-2 px per zoom octave, scaled by line fog.  K6 Scheme labelled
(equal fifths, v<5 = label index).  S9 Halftone replaces Video Ground:
hr = hr(K5) - 2900 + 4*(Y-64) per pixel, clamped 0..6880 (strict compare so
radius 0 draws nothing).

## v1.3.1 (2026-10-05)

S10 Breathe -> **Inverse** (Ron didn't like Breathe; picked from Inverse /
Outline / Pile relief / Blood flood).  Vblank step 30 swaps the field and
orange palette registers after every frame's fresh load (so it never
toggles cumulatively); Night factors stay by role (ground 1/4, serpentine
1/2).  Breathe and its multiply steps removed.  The Room 237 preset is now
Night + Inverse.  Categories fixed to Pattern / Film / Render (Generative
and Art are not in the firmware's 37-name list).  Build (router1): all 6 configs seed 1 — HD Analog 87.0,
HD HDMI 89.8, HD Dual 89.2 MHz (SD 83.7 / 88.1 / 84.7); 5,636–5,657 LC (74%).

## Controls (v1.3)

| Control | Function |
|---|---|
| K1/K2 | **Drift X/Y**: bipolar, deadbanded, constant screen speed (Y = walk the corridor) |
| K3 | **Perspective**: flat carpet -> corridor floor, horizon from the top, fogged |
| K4 | **Weight**: serpentine stroke width |
| K5 | **Hex Size**: red hexagons ~60..140% (halftone gain/offset under S9) |
| K6 | **Scheme**: Classic / Gold Room / Lobby Blue / Deep Night / Mono Amber |
| S7 | **Mirror**: rows mirror about centre; with Perspective, a carpet valley |
| S8 | **Weave**: texture-anchored pile grain |
| S9 | **Halftone**: input luma sets each red hexagon's size |
| S10 | **Inverse**: field and serpentine colours swap (orange ground, plum serpentines; the hexagon gaps read as orange cells) |
| S11 | **Night**: field 1/4, orange 1/2 toward black, reds full |
| P12 | **Zoom**: exponential, continuous, centre-anchored, glided |

P12 values were remapped (`P' = 1023 - 4/3*(1023-P)`) so the old default
zoom (719 -> 618) looks the same under the 6-octave law.  Presets are
hand-set placeholders; recapture from hardware.

## Build (v1.3)

2026-10-05, **build with router1**: `NEXTPNR_ROUTER="--router router1"
./build_programs.sh ron overlook`.  All 6 configs seed 1: HD Analog 85.1,
HD HDMI 85.1, HD Dual 90.8 MHz (SD Analog 91.3, SD HDMI 81.4, SD Dual 85.5);
5,745–5,764 LC (75%), 0 BRAM.  Packaged `out/rev_b/ron/overlook.vmprog`
(v1.3.0).  Not HW-tested.  (router2 on the same netlist: seeds 1 and 3
stalled, seed 2 missed, seed 4 reached 83 MHz.)  The first v1.3 build sat at
74.4 MHz, limited by the control if/elsif chain gating ~200 line-engine
clock-enables plus the raw sync pins; registered sync events + independent
ifs fixed it (see TECHNIQUES).

## Controls (v1.2, superseded)

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

## Build (2026-07-19, v1.2, superseded)

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

- Luma-modulated stroke weight (needs slab tests rewritten as
  |plane - centre| <= hd + delta, ~+400 LC)
- Generated avid (cubist, ~25 LC) if thin coloured lines show on HW
