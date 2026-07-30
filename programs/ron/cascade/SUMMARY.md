# HYPNOS CASCADE — coupled concentric circles (v0.8: 24 discs, octagon slack, per-ring ramps — S9 Steps/Smooth, S11 multi-ring fade; Invert removed)

Concentric black/white circles that **push each other**. Move the centre (K1/K2)
and the innermost circle leads; each outer ring holds still until the one inside
closes its slack, then is shoved rigidly outward — a wave of motion travelling
outward ring-by-ring **through a 24-ring chain** (the outer field follows as one
body once ring 23 is pushed). The rings are hard-coupled and **never overlap** —
a band of the opposite colour always survives — and every edge is an **exact
integer circle**. When you stop, they either glide home concentric
(**Catch-Up = Home**) or freeze in the dragged nest (**Stay**).

A dedicated fork of **Hypnos** (v0.3 of the main program is untouched).

## v4.1 — 24 discs, ~87 % slack (octagon clamp)

Per user request: 24 modelled discs, and rings should nearly TOUCH before
yielding — the minimum edge-to-edge gap at the push point is now **~12 % of
the ring spacing** (v3: 41 %; the first v4 misread the request and shrank the
slack instead — lesson recorded).  That much slack overlaps rings on diagonal
drags with a per-axis box clamp, so the chain clamp is an **OCTAGON**
(per-axis ±A plus 45°-rotated ±A·√2, A ≈ 0.875·spacing, computed per frame as
pure shift-adds; the coupling scan is fully serialized through one shared
step/clamp unit, 14 vblank phases per ring, with the clamp bound
pre-registered — single-cycle versions of both the slack decode and the clamp
cone were the critical path).  A 640-px slack cap keeps offsets inside the
15-bit chain words.  24 discs did not fit the v3 architecture (per-ring
comparators cost ~60 LC each; 24 rings × 8 engine slots = 208 clks > every
hblank), so:

- **Render — 1bpp mark line buffer.** The span engine writes each disc's two
  scanline edges as single-bit **marks** (one EBR bank per line, double-banked);
  the pixel path is ONE running-parity XOR over the marks. Strict nesting
  (guaranteed by the chain clamp) makes edge-count parity equal the innermost
  disc's parity (C_NR even). Discs overhanging the left screen edge toggle a
  line-start init bit; off-right edges are simply dropped; the displayed bank
  is cleared 2 px behind the read; a vblank pass wipes both banks. Only disc
  23 keeps equality toggles — its exact in/out is the handoff to the outer
  tracker. Disc count is now nearly free: pixel-path cost is constant.
- **Engine runs a line early.** The span pass for line N runs during line
  N−1's ACTIVE video (chain RAM, shared multiply, sqrt units are all idle
  then — only the adds-only tracker runs per pixel). ~210 clks fit even SD's
  720-clk active line. The tracker line-seed (E_LINE, now ~50 clks) stays in
  the hblank. Frame startup: vblank chain E_CLEAR → E_SPAN → E_LINE.
- **Outer field** (rings 24+): unchanged v3 incremental r² band tracker,
  seamed exactly onto disc 23.
- **Timing fixes for 74.25 MHz**: E_LINE bracket corrections retimed to 2
  slots each (compare signs registered before the update — kills a 29-bit
  carry → enable-cone path), and the per-pixel tracker no longer maintains
  the 12-bit ring index: crossings toggle a 1-bit **parity flop** (t_p); the
  full index lives only in the hblank seed. Without these, no seed passed.

## v0.2–v0.4 — K4 GRADIENT width + S11 Shading (Stepped/Smooth)

The marks now carry an enter/leave **direction bit** (the EBR banks' free
second bit), so the pixel path counts the TRUE ring index k (5-bit, mod 32;
line-start index seeded by the engine's overhang count; the outer tracker
keeps a matching mod-32 counter).  In **Stepped** mode EVERY
RING carries its own ramp-in/ramp-out (dark at both edges, bright
mid-ring): a second 16-bit phase runs at ring period (Q1 = 2^16/spacing
per radial pixel) and is **RE-ANCHORED TO ZERO AT EVERY TRUE DISC EDGE**
via the mark buffer, so each ring's shading is welded to its own moving
edges and cascades with them.  K4 sets the FINENESS: zone 0 = the classic
hard black/white, zones 1–6 = the ramp quantized to **2 / 4 / 8 / 16 / 32
/ 64 gray steps within each ring** (top bits of the ramp, bit-replicated
to exact 0..255).  Timing subtleties: the mark data trails the crossing
detect by one cycle, so the crossing direction is REGISTERED and applied
together with the mark gate (one fewer output delay reg keeps latency 6);
a mark landing exactly at x=0 is invisible to the pixel path, so the
engine flags it (m0_done) and the line seed snaps to 0.  Known mid-drag
artifact: between edges the ramp advances by single-centre radius, so
scanlines near a dragged ring's tangent hold-then-snap (horizontal
streaks); exact when concentric/settled.  **S11 Smooth** = a continuous ramp fading evenly
black↔white over the K4 width: a 16-bit PHASE accumulates ±Q =
2^16/(2w·spacing) at every integer-radius crossing and its top bits fold
straight into luma — a per-pixel-smooth radial gradient with zero per-pixel
multiplies, anchored at the outer centre (exact in Uniform and settled
states; during an active drag it glides with the chain rather than
squishing per ring — per-disc smooth distance is not computable on this
chip).  S9 Invert applies to both.

The machinery that made Smooth affordable also REPLACED the outer-field
renderer: an integer-RADIUS tracker (the r² bracket tracker at spacing 1 —
its taps degenerate to +4/0/+2 and 17-bit arithmetic) detects every radial
pixel crossing; `r mod spacing` and ring-index counters ride the same
crossings.  The 28-bit r²-bracket tracker, its candidate arms and the
squared-bracket line seed are GONE (the line seed is now a linear ±s
ladder), and the disc-23 hit comparators fell out too (the crossing counter
itself knows "outside all discs").  Leanest cascade build yet: **88 % LC**.
Funded by dropping E_SPAN to ONE sqrt unit (15-slot ring cadence, ~370 clks
during the previous line's active video — the line-early scheme had the
budget).

**Verified pixel-exact** against an integer-exact Python model — 0
mismatching pixels in nine scenarios (uniform bullseye, concentric nest,
dragged nest, per-ring stepped ramps at 16/64 steps concentric + 64 steps
dragged, 2/32-ring smooth fades, smooth fade on a dragged nest) at 480×270
sim raster.

## Per-circle gradient — moved to `nacre`

The natural next step for this program ("each ring's gradient should squeeze
and swell with the ring it sits in, measured against the circle nested
inside it") does **not fit here**: the knot engine it needs measures ~4350
LC, and a v0.9 cascade carrying it synthesised to **11454 / 7680 LC (149 %)**
even after dropping to 16 rings. It now lives in `programs/ron/nacre/`
(work in progress). This file is unchanged v0.8 behaviour.

The design study for it is in this directory and is worth keeping:
`percircle_int2.py` (the validated integer model), `shading_compare.png`,
`shading_settled.png`, `shading_percircle.png`, `percircle_lin.png`.

## Control map (unchanged)

| Phys | Control | Range / detent |
|---|---|---|
| K1 | **Center X** | bipolar ±2 W, detent = frame centre |
| K2 | **Center Y** | bipolar ±2 H, detent = frame centre |
| K3 | **Period** | ring density (exp), spacing 4..2048 px |
| S7 | **Glide** | one-pole slew on the centre (drives the cascade). Default ON |
| S8 | **Cascade** | Uniform (classic bullseye) / Cascade (sequential push). Default ON |
| S9 | **Ramp** | Steps (K4 fineness, classic hard at 0) / Smooth (full-res per-ring ramp, K4 ignored — the soft gradient cascades with the rings). Invert is gone (v0.8) |
| S10 | **Catch-Up** | Stay (freeze dragged) / Home (glide back concentric). Default ON |
| K4 | **Gradient** | fineness: classic / 2..64 steps within each ring. Default classic |
| S11 | **Shading** | Stepped (per-ring levels) / Smooth (continuous ramp). Default Stepped |
| K5, K6, P12 | **spare** | |

## Simulator notes (cost hours — read before simming)

- **TOML defaults apply in sim.** S7/S8/S10 default ON; a "clean" scenario
  must explicitly `--set toggle_switch_7=0` etc., or the glide/settle
  dynamics contaminate the capture (settle's floor-shift walks each ring
  −1 px/frame once the centre is still).
- The sim raster is `video-mode / decimation` (default 1080p/4 = **480×270**),
  NOT the input image size; frame 1 publishes with unmeasured W/H defaults
  (720/480) — a bit-exact model must replay that transient.
- v4+ needs **no tool hblank patch** at decimation ≤ 4: E_LINE (50 clks) fits
  the fixed 64-clk sim hblank and E_SPAN (~370 clks) runs during active.
  Deeper decimation starves E_SPAN.
- The tool renders **BT.601 studio range**: png = round((Y10−64)·255/876),
  clipped.  0/255 endpoints survive, mid-grays do NOT (85 → 81, 170 → 180):
  exact-compare models must apply this mapping.
- K4 also defaults non-zero via the TOML — set `rotary_potentiometer_4=0`
  for hard-mode scenarios.

## Fit / timing

**~7.0k LC (91 %), 4 EBR** (chain x/y + 2 mark banks). All six configs pass
**icetime sign-off STA** (never trust the build's pre-route estimate).  An
early HW test showed rough ring edges + occasional full-width noise lines;
that build's hd_analog margin was only 0.2 % — since then seeds are chosen
for ≥ 2.7 % margin on every HD config as the suspected fix:

| Config | Fmax (routed) | seed |
|---|---|---|
| HD Analog | 78.6 MHz | 4 |
| HD HDMI | 78.0 MHz | 6 |
| HD Dual | 80.2 MHz | 7 |
| SD Analog | 59.4 MHz (Fmin 27) | 1 |
| SD HDMI | 76.1 MHz (Fmin 27) | 1 |
| SD Dual | 74.6 MHz (Fmin 27) | 1 |

Shipped `out/rev_b/ron/cascade.vmprog` (program v0.8; presets: Gradient
Waves = S11 Smooth, Velvet Rings = S9 Smooth) repacked from these six
bitstreams, validation PASSED. (v0.8 HD seeds were swept manually — the
builder's own run best-efforted hd_analog at 70.6 — then icepacked into
the build tree and repacked.)  (Packer caveat: it will happily pack with a MISSING
config bitstream — count 6 .bins before shipping.)  Timing lessons this round: the vblank SEQUENCER never
executes slot 0 (schedule nothing there); the single-slot spacing decode
(negate+ROM+barrel) and an un-preregistered one-hot multiplier operand were
the hd_hdmi critical paths — both now pipelined across free slots. **HW-iterating** (margin fix + octagon slack + gradient awaiting a
hardware pass; if edge roughness survives these margins, report the output
video mode). A plain rebuild may land failing HD seeds; reseed with the
scratchpad `sweep_cfg.sh`.
