# Chantilly — black-lace halftone

**v1.0.0 (2026-10-09): FINAL except presets — signed off by Ron ("I think we
can finalize Chantilly").** v1.0.0 = the v0.3.0 design, version bump only;
presets are still hand-written (capture from HW with capture_preset.sh).
History: v0.1 2026-08-03, v0.2.0 / v0.3.0 / v1.0.0 2026-10-09. Source: a photo of black Chantilly-style lace — bands of
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

## v0.2.0 — hardware-contract pass before first flash (2026-10-09)

Audit of v0.1 against the CLAUDE.md hardware contracts found:

- **P12 / SD scale were inverted.** `s` = lace units per pixel, so bigger s =
  SMALLER lace. v0.1's SD `>>1` made SD lace 2x BIGGER (bands ~512 px on a
  480-line frame), and P12 up made lace finer while the presets assumed the
  opposite. Now `s = 2^((511-P12)/512)` (P12 up = bigger lace, HD s 506..128)
  and SD gets `s<<1` (bands 128 px). The 511 keeps svi <= 0 so SD s <= 1013.
- **Luma cap**: Y = 64 + key*2.75 (max 766; was 957, above the ~768 clip).
- **Ivory** ~2x stronger (BT.601 U -key/4, V +key/8) and U/V-swapped at the
  output compose. v0.1 would have shown a mint tint on hardware.
- **Interlace**: +1-line seed now on the bottom field (`field_n='0'`).
- **W/H hysteresis +-2** (analog line-length jitter shimmered the u origin).
  Two-step compare (capture, then compare vs free-running W+-2/H+-2): the
  fused add+compare was the routed critical path on HD Analog/HDMI.
- **Generated avid** (aurora pattern, hsync-parity vote, fixed W width) —
  the ivory tint is program-drawn chroma.
- **Raw luma tapped early** (y_sr 0..4); radius (8b), ink flag and ghost
  (7b) are computed there and delayed, alignment identical to v0.1.

Build (router1 — router2 missed every seed on HD Analog/HDMI, router1 passed
3/3 in a seed sweep): `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron chantilly`
~5700 LC (74%), 0 BRAM. HD Analog 76.97 (**seed 2**), HD HDMI 82.20 (seed 1),
HD Dual 76.33 (seed 1); SD Analog 70.38 (**seed 2**, seed 1 stalled), SD HDMI
73.31, SD Dual 73.62. **Not HW-tested.** v0.1 sources in `.chantilly_work/`.

## v0.3.0 — look pass (2026-10-09)

- **Rounder dots**: dot radius reads a 2-D blurred luma: an exact 8-px
  running box (add new, drop 8th), then a vertical one-pole IIR
  `new = above + (box - above)/4` held in ONE 4096x8 line buffer (8 EBR),
  bank = line parity (read other bank, write own: no read/write collision).
  Column label = s_col-5 at the IIR stage; read issued at s_col-2 (EBR +
  fabric re-register = 3 cycles). s_col SATURATES at 2047 so the blanking
  tail can't wrap onto the start of this line's bank. First line of a frame
  takes the box straight.
- **2-D ink**: vertical gradient = box - IIR(above) (horizontal edges ink,
  with a few-line tail below as the IIR settles) OR the 4-px horizontal one.
- **K4 Ink bottom detent = no ink**; **S9 = Colour**: source chroma rounded
  to 5 bits (512 = exact code 16), times the 3-bit dot key, clamped +-448
  (~1.75x saturation); dry path, so no U/V swap. S7 Ivory adds its warm
  offset on top (Mono = pure source colour).
- **Ghost on White ground**: bright video (Y 384..765) in the white ground;
  cuts stay black.
- **K3 Flow** = scroll + spin: angle += (K3-512)/4 Q4 per frame, added
  before theta*ring (mod 4096 keeps the 6*ring cell wrap seamless); odd
  bands counter-rotate.
- **P12 +-1.75 octaves** (exponent Q6 = e/4 - e/32): HD s 861..76, SD x2
  (s is now 11 bits).
- Presets: S9 now means Colour, so the four old presets have S9=0; added
  "Video Lace" (Colour on, slow flow).

Timing lessons (two critical paths, both fixed by restructuring, not seeds):
1. The vsync trigger's `s_lcnt > 8` compare fed every sequencer-slot enable
   (9.5 of 13.7 ns routing) once the new slots raised the fanout. Fix: a
   free-running `s_lgt8` flag and a registered `s_vtrig`.
2. EBR read -> subtract -> abs -> compare in one stage (vertical ink) pinned
   HD HDMI at 67.40 on every seed/router (logic-bound). Fix: register the
   signed differences, then compare against +th / -th in parallel (no abs).

Build (router1): `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron chantilly`
~6300 LC (82%), 8/32 EBR. HD Analog 83.98, HD HDMI 88.62, HD Dual 86.73
(all seed 1); SD Analog 83.54 (**seed 2**, seed 1 stalled), SD HDMI 88.57
(**seed 2**, seed 1 stalled), SD Dual 81.40. **Not HW-tested.**
v0.2.0 sources + vmprog kept in `.chantilly_work/`.

## v1.0.0 build (2026-10-09)

Version-bump repack. The builder wipes .asc/.bin before every attempt, so
it re-placed every config (synth cached) with the DEFAULT router2: all six
passed on seed 1 — HD Analog 79.00, HD HDMI 79.50, HD Dual 79.04, SD Analog
74.37, SD HDMI 75.34, SD Dual 75.70 MHz; 82% LC, 8/32 EBR. Since the v0.3
timing fixes (registered vsync trigger, abs-free ink compare) router2 closes
too; router1 (`NEXTPNR_ROUTER="--router router1"`) gives more margin (HD 84-89).

