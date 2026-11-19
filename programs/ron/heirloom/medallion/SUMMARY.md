# Medallion — Heirloom #12: fire-ring diamonds and a plaid medallion

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/medallion.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (`Medallion Diag`,
`out/rev_b/ron/_diag/medallion_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`). Production v0.5.1
(`programs/ron/heirloom/medallion/`) is untouched. Phase-2 working files: `.heirloom_work/rb_medallion/p2/`
(patch_vhd.py, sweep.py, imodel6.py, sheet6_model.png, check.sh, psim.sh, renders).

## Phase 2 (v0.6.0)
Ron left no notes for medallion (free hand, 2026-11-03 batch). The map follows the phase-2 brief: the two global
scales (K1 diamond count = an X zoom of the diamond field, K4 stripe density = a Y zoom of the stripes) become
element parameters, the ring density gets a wider exponential range, and Etch reaches twice as far. The photo stays
at the default position, bit-exact.

### Controls: was → now
| Control | v0.5.1 | v0.6.0 diag | Full sweep at 1080p |
|---|---|---|---|
| K1 | **Diamonds**: diamonds across, 1 + 5k² (2.05 at 468) | **Lean** (renamed), bipolar: each diamond's inner and outer half get their own slope. Left = the diamond centres slide out to the screen edges, right = in under the medallion. The fold and the outer peaks stay put; 2.04 diamonds (the photo's) at every setting. Photo at 50 % (512), ±4 deadband | Centres at x 152 / 1866 (0 %) .. 527 / 1466 (photo) .. 916 / 1106 px (100 %): ~375 px (20 % of the width) each way. The halves split up to 9 : 1, so the rings turn into arrowheads (out: bow tie; in: long chevrons fanning to the edges, tight centres hugging the fold) |
| K2 | **Rings** 0.3 .. 3x, square law (1.0 at 521) | **Rings** 0.25 .. 1 .. 4x, exponential (2^(k/256), photo at 50 %) | Ring spacing ~1045 px .. 261 px (photo) .. ~65 px |
| K3 | **Reach** | unchanged | |
| K4 | **Stripes**: density 0.25 .. 3x (1.0 at 534) | **Bands** (renamed), bipolar per-band heights; the photo's 18 measured boundaries at 50 % (512), ±4 deadband. Left = **Cascade** (bands thin at the top, swelling down: ps' = ps − a(u² − 1), a to 0.5); right = **Lens** (the middle bands swell, the outer ones crowd: ps' = ps + b·u(u² − 1), b to 0.9). The warp keeps the phase at ±8 half-cycles (lines ~108 / 998) | Photo bands 40–72 lines. Cascade 0 %: top half ~2x as dense as the bottom (top bands ~35, bottom 94 lines). Lens 100 %: centre band 236 lines (22 % of the height), outer bands 23–28, 25 boundaries |
| K5 | **Speed** | unchanged | |
| K6 | **Etch** 0 .. 400 h per full luma, square law (60 at 396) | **Etch** 0 .. 800 h per full luma, square law; default moved to 280 = the same gain as v0.5's 396 | Camera bends the rings by up to ~400 px (was ~200) |
| S7 Flip, S8 Steps, S9 Swap, S11 Source | | unchanged | |
| S10 | **Wave** Tri / Saw | **Wave** Tri / Saw; Saw now runs on the Lean DDA (rising ramps of period H2), so Lean stays live in Saw: the teeth alternate long / short | |
| P12 | **Medallion** | unchanged (size 0 .. 2.1x, photo at 48 %) | |

### Engine changes
- **Lean DDA.** The triangle is now a Q6 distance-to-peak DDA with a per-half step: inner halves (fold side)
  step by xs·H2Q/L_in, outer halves by xs·H2Q/L_out (L_in + L_out = 2·H2, H2 fixed at 939 h). The half type flips
  at each minimum (Tri) or each wrap (Saw); the minimum reflection uses 2·H2Q − max(step) − z, exact at the photo and
  within one step elsewhere. The vblank sequencer computes m = (K1 − 512)·1502 >> 10, L_in / L_out, both slopes on
  the serial divider (16 steps each, replacing the old H2 division), the steps, thresholds and the line-start state
  (which half, direction, z = u·slope on the serial multiplier).
- **Bands warp** in the line engine, before the stripe gain: three serial multiplies (ps², the Lens coefficient,
  the warp term), +53 clocks (the engine now ends at clock 193 of SD's 858).
- **Rings:** a 256 × 9 exp ROM (2^(i/256)), one product with 250, a 4-way shift.
- **Etch:** gain register 9 → 10 bits, etch term 10 → 11 bits.
- Removed: the K1 square / count products, the K4 gain products (the stripe gain is the photo's constant 255/256),
  the old saw DDA. Latency unchanged (24); no pixel-path stage added or removed.

### Presets (re-aimed; hand-written placeholders)
Photo (the defaults), Open Field (P12 0), Test Card (P12 100 %), Arrowheads (Lean 100 %, P12 29 %), Bow Tie
(Lean 0 %, P12 37 %), Cascade (Bands 12 %, Rings 63 %), Lens Bands (Bands 100 %, Lean 74 %), Inside Out (Swap On,
Lean 88 %, P12 41 %).

### Build
router2 (builder default; `.heirloom_work/diag_build.sh medallion`, 2026-11-03). Post-route Fmax: HD Analog s1 75.79, SD Analog s1 68.78, HD HDMI s1 82.12, SD HDMI s1 78.34, HD Dual s3 82.07 (seeds 1–2 missed), SD Dual s1 77.91 MHz. 6959–6982 / 7680 LC (90.9 %, was 82 %), 15 / 32 EBR (+1: the Rings exp ROM). iceunpack OK × 6. No pin-skew screen; the builder's seeds are kept. vmprog 453,789 bytes.

### Checks
- GHDL analyse / elaborate / synth front end: clean (`p2/check.sh`).
- `p2/sweep.py` (RTL widths, bit-level emulation of the new logic):
  - every K1 value at 1080 / 720 / SD: the 16-step divider equals exact division and every signal fits;
  - the DDA over whole lines in Tri and Saw;
  - the Rings exp curve is monotonic, 62 .. 250 .. 998;
  - Etch reaches 858;
  - the Bands warp, for every K4 value × 1080 lines, fits the 12 / 13 / 14 / 15-bit line-engine signals;
  - at the default the Lean DDA equals v0.5's triangle exactly in all three modes, and Bands is the identity.
- Integer model `p2/imodel6.py` (v0.5's imodel plus the new paths): the default frame equals the v0.5 model (0 px).
  Control sheet: `p2/sheet6_model.png`.
- GHDL 720p60 (decimation 1), Osc, defaults, diag against production v0.5.1 rendered side by side: **0 px differ**
  (the raw 10-bit output is identical). Files: `p2/diag_osc.png`, `p2/prod_osc.png`.
- GHDL 720p60 with K1 = K4 = 1023 (glided to 631 after two fields), Speed still, black input: **bit-exact with
  imodel6** over the interior (0 px at the tester's usual 3-px offset). This validates the Lean DDA at unequal slopes
  and the Lens warp in RTL. File: `p2/diag_ext.png`.
- Not verified in RTL: Saw on the Lean DDA and the Cascade branch (covered only by the bit-level emulation in
  sweep.py); SD and 1080 renders; no hardware.

**v0.5.0 (2026-10-21): rebuilt from the photo WIN_20230305_23_51_38, built, NOT flashed.**
The v0.1 (generic folded stripes and a posterized-video circle) was too abstract. This version is
measured from the photo. Two diamond systems of stepped fire rings (dark red to yellow) are cut by
horizontal stripes. Every other stripe shows the rings half a colour cycle later. An oval medallion
in the middle keys in a CMY/grey plaid.

## Reading of the photo (measured; .heirloom_work/rb_medallion/notes.md)
- **One field, one 3-bit colourizer.** The whole warm field is w = T(x) + V(y) through a colourizer
  of 8 flat levels: dark red (143,34,8), (186,33,10), (223,63,4), red-orange (254,63,1), orange
  (254,97,1), (254,138,1), (254,178,1), yellow (254,236,0).
  - The steps are sharp vertically (2 lines) and soft horizontally (camera, 10–14 px).
  - The B stripes are the A stripes with the colourizer's top bit flipped. All 27 code steps in the
    B stripes sit within ±2 w-units of the A stripes' step k→k+4 mod 8.
  - The floors confirm it: dark red (code 0) in A, orange (code 4) in B. B's sharp dark-red/yellow
    jumps are A crossing 3↔4.
- **Diamonds.** T is a horizontal triangle:
  - period 939.34 px (2.044 per 1920), peak on the central fold at x 996.6;
  - rounded peak (parabolic, ρ 30) and filleted minimum (ρ 24): the U-shaped apexes;
  - held at its x = 57 value at the line start, so every contour bends flat into the left edge with
    no mirror image.
  - V(d) = 0.9744 d + 0.000792 d², d = |y − 522.4|. The arms are ~1 px/line at the centre and
    1.74 px/line at the apexes.
  - Fitted by palette purity per w-bin per stripe type (variance 0.0007).
- **Rings.** level = min(ring waveform, outer envelope).
  - The ring waveform is two measured periods between the plateau centres (w 83.5 and 605.8). The
    ring spacing shrinks from 280 to 242 px, and this chirp is baked into the table.
  - The yellow plateaus (code 7) are clipped. Trough 1 is a V down to level ~2.6; trough 2 is broad
    and flat at ~3.5.
  - The envelope makes the outer descent to the flat floor: one level per 17.63 w, level 7 at
    w = 645. Code steps along w: 32, 135, 155, 178, 198, 222, 238, 260, 281, 300, 320, 408, 426, 445,
    465, 511, 530, 548, 567, 645, 662, 680, 698, 716, 733, 751.
  - A small orange "eye" (code 6) sits at each diamond centre.
- **Stripes.**
  - 18 hard boundaries: 108, 178, 250, 304, 350, 391, 439, 487, 533, 577, 623, 667, 707, 749, 800,
    864, 928, 998.
  - Symmetric about line 553: ~45 lines apart in the middle, ~70 near the top and bottom, with wide
    end bands.
  - A sine phase 9.79·sin((y − 553)/424) fits to ±6 px. The hardware uses the measured boundaries.
- **Medallion.** An ellipse, centre (980.34, 533.19), 459.11 × 430.9 px (rms 1.46 px).
  - Hard key, ~10 px soft, with a thin green chroma fringe.
  - There is no rim: the white at its edges is the plaid's white column and row.
- **Plaid.** Strictly periodic, 304.15 × 207.5 px, origin (690, 134.5).
  - 8 equal row bands of 25.94 lines: two grey-band rows, three colour rows, a magenta/white row, a
    small-cyan-bar row and a white row.
  - Measured column edges: cyan 0–91, white 91–151, a 3-px cyan sliver, magenta, a yellow bar at
    171–203, magenta.
  - Colours: white, cyan, magenta, yellow, black (16,41,20), dark grey, grey, light grey.
- **No** noise, tearing or interlace comb.

## What it does
1. **T.** A distance-to-peak DDA in half virtual pixels, reflecting at the peak and the minimum and
   held left of x = 57. Small tables give the rounded peak (z²/120) and the filleted minimum (d²/96).
   S10 Saw replaces it with a one-sided ramp.
2. **V.** Per line, from the per-line engine. w = T + V + Etch·luma.
3. **Ring.** Ring index = (w − 83.5)·Rings·0.98 − flow, through a 1024-entry two-period ring table
   (level·32).
4. **Envelope.** E = 224 + D − D/16 − D/32 with D = Reach − w. level = min(ring, E), clamped 0..255.
5. **Colour.** Palette address = level with its MSB flipped on B stripes (S7), in a 512-entry ROM:
   bank 0 posterised (the photo), bank 1 smooth (S8).
6. **Stripes.** The stripe phase per line comes from a 4-line-sampled table of the measured
   boundaries, interpolated, × Stripes − roll. Its parity selects the stripe type.
7. **Medallion.** A per-line chord half-width √(R² − dy²)·rx/ry by a serial square root, then a
   per-pixel |x − cx| < half-width test. S9 Swap inverts it.
8. **Plaid.** A column DDA (Q16) and a per-line row → 2048 × 4 tile ROM → 8 colours. It drifts with
   Speed.
9. **Output.** Chroma and luma [1 2 2 2 2 2 2 2 1]/16, luma 13/16 (palette luma pre-scaled 16/13,
   capped so whites land at ~777), blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Diamonds | the two diamond systems | diamonds across, 1 + 5k² (2.05 at the default); the peak stays on the central fold under the medallion |
| K2 Rings | the nested fire rings | ring density 0.3–3× (1.0 at the default) |
| K3 Reach | where the rings stop and the flat dark-red/orange stripes begin | envelope position w = 120..1400 (645 at the default) |
| K4 Stripes | the horizontal stripes | stripe density 0.25–3× (1.0 at the default) |
| K5 Speed | ring flow / stripe roll / plaid drift | bipolar, still in the centre ±8. Positive: rings stream out of the diamond centres (a ring in ~12 s), stripes roll down (one per ~4 s), the plaid drifts (~8 px/s) |
| K6 Etch | the warm field bent by the picture | Video (or Osc) luma added to w, 0–400 px per full luma, square law (60 at the default) |
| S7 Flip | the stripes' half-cycle colour flip | On (the photo) / Off: plain nested diamonds |
| S8 Steps | the 8 posterised colour steps | Posterize (the photo) / Smooth ramps between the levels |
| S9 Swap | the medallion key | plaid outside, fire rings inside the medallion |
| S10 Wave | the horizontal triangle | Tri (diamonds) / Saw (slanted ramps with hard vertical cuts) |
| S11 Source | Video / Osc | Video (default) / Osc |
| P12 Medallion | the medallion | size 0..2.1× (1.0 = the photo at 48 %). 0 % = the open diamond field with the central fold revealed; 100 % = a plaid test card with warm corners. Glided 1/8 per field |

## Video
- The camera enters the warm field the way an analog patch would: its luma is added to the diamond
  field, so the fire rings bend and ripple around the picture.
- At the default Etch (60 px per full luma) the photo's composition reads clearly, with the picture
  woven into the rings. At 100 % the rings shred around the image.
- The medallion's plaid is synthetic in the photo, so it stays a plaid.
- In Osc mode a 2-D triangle lattice (341 × 205 virtual px, slowly drifting) feeds the same line
  buffer and Etch path.

## Engine
- **Vblank-zone sequencer** (zone = no avid for 1024 clocks), one small operation per zone clock:
  - a shared glide over K1/K2/K3/K4/K6/P12;
  - one serial multiplier: 17 products (knob squares × constants, the medallion radius², the speed
    square and its four rates, the plaid column start);
  - one serial divider: H2 = 491520 / n8, the triangle half-period;
  - the triangle's line-start state from (max(x0, 114) − 1993) mod 2·H2;
  - motion accumulators (ring flow, stripe roll, plaid drift with tile-size wraps).
- **Per-line engine** (kicked at each line start, computes the NEXT line, ~140 clocks; fits SD's
  858-clock line) on one serial multiplier:
  - V;
  - the stripe phase (two ROM reads + interpolation, × the Stripes gain, − roll) and its parity;
  - the medallion chord: dy², a digit-by-digit square root (11 clocks), × rx/ry;
  - the plaid row.
- **Pixel path (24 stages):**
  - S2–S3: rounding tables, |x − cx|, tile address.
  - S4–S6: w and the etch (nibble partials).
  - S7–S10: ring product as 3 + 3 + 4-bit row partials, the flow folded into the high half; the
    envelope.
  - S10–S12: ring ROM + fabric copy. S13: min. S14–S16: palette ROM + fabric copy.
  - S17: compose with the plaid class (carried 3 bits wide).
  - S18–S24: conditioning and the gate.
- **ROMs:** ring 1024 × 8 (2 EBR), stripe phase 512 × 12 (2), palette 512 × 32 (4), plaid tile
  2048 × 4 (2), line buffer 2048 × 8 (4) = 14 EBR. medallion.py (pure python: the build hook) holds
  every fitted constant and generates medallion_pkg.vhd.
- **Contracts:**
  - virtual 1080-line coordinates (x step 2/3/4 half-pixels, y step from the timing ID);
  - SD is the centre 1440 virtual px;
  - generated avid (parity-locked);
  - one 24-clock latency in every mode;
  - colours in the hardware convention (U = Cr, V = Cb).
- **Verification:** the integer model (imodel.py) is bit-exact against the GHDL render at 720p60 with
  Speed still and etch 0; the default Osc render differs only at ring edges (Osc phase not modelled);
  motion matches the model frame for frame.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). The Y box resyncs too (to 8 x 64). Post-route Fmax: HD Analog s1 77.74, SD Analog s1 83.70, HD HDMI s1 81.97, SD HDMI s1 78.85, HD Dual s2 78.64, SD Dual s1 73.65 MHz; 6272-6285 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-21, NOT flashed.** router2 (`MAKE_TIMEOUT=900 ./build_heirloom.sh medallion`).
- **Build seeds:** HD Analog = seed 2 and HD HDMI = seed 2 (seed 1 missed on both); the other four
  are seed 1.
- **Post-route Fmax:** HD Analog 77.97, HD HDMI 78.24, HD Dual 81.19, SD Analog 77.13, SD HDMI 77.99,
  SD Dual 83.21 MHz.
- **Fit:** 6260–6282 / 7680 LC (82 %), 14 / 32 EBR. iceunpack OK × 6.
- **If a later version gets tight:** try `NEXTPNR_ROUTER="--router router1"` first.
- Presets are hand-written placeholders: Photo, Open Field, Test Card, Swapped, Saw Smooth, Dense Rings.

## Deviations from the photo, and why
- **Whites and yellow are capped.** The shell's output scaling (the HD HDMI bar fix, drawn luma
  capped at ~777) turns the plaid's white (photo luma ~930) into light grey on hardware, and the
  yellow is slightly duller.
- **Edges are a little crisper.** The 9-tap luma and chroma filters give ~7 px edges, against the
  camera's 10–14 px; the photo's medallion key edge is a touch sharper than its plaid edges, while
  the model uses one blur for both.
- **Diamond centre.** The very centre of each diamond (~10 px) shows levels 4–5 instead of 6, because
  the ring table is periodic.
- **Stripe edges** are within ±1 line of the photo (4-line table samples).
- **Capture border.** The capture's dark top/bottom rows and left columns are not drawn.
- **Kept as photographed.** The medallion stays an ellipse (aspect 1.065), identical at every
  resolution.
- **SD.** SD shows the centre 4:3 of the picture: the left-edge hold region and the outer right arms
  are cropped; shapes and the medallion are unchanged. 720p is identical to 1080p.
- **Motion.** The photo is a still, so the motion is designed: ring flow, stripe roll, plaid drift.

## Working files
`.heirloom_work/rb_medallion/`:
- `ref.jpg`; crops `c_*.png`.
- `fit1.py` / `fit2.py` / `edge*.py` / `apex.py`: field fits. `stripes.py`; `circ.py`; `plaid*.py`,
  `tilefit.py`.
- `joint*.py` / `viterbi.py` / `refine_u.py` / `stair.py` / `codes.py`: colourizer fits.
- `model.py` / `model2.py` / `proto2.py`: the float model. `proto_final.png`, `cmp_proto.png`.
- `ctl.py` / `sheet.py`: `sheet.png`.
- `imodel.py`: the integer model.
- `rtl_*.png`; `cmp_final.png`, `cmp_motion.png`, `cmp_flow.png`.
- `build.log`, `notes.md`.


## Open questions for Ron
1. **Lean replaced the diamond count** (1 .. 6 across). Do you want the count back, e.g. as a second mode of Lean or
   on a switch (S8 Steps is the weakest switch)?
2. **Bands replaced the stripe density.** Is Cascade (left) strong enough? It is a ~2 : 1 density gradient top to
   bottom; it could go further, but past ~0.55 the bottom bands fold into one wide floor band. Or would you rather
   have a symmetric Pinch (middle bands thin, outer bands swell) on the left?
3. **Arm curvature** ("Bow": the diamonds' arms bowed in to 4-point stars or out to round lens rings, lozenge-style)
   was designed but not built, since all 12 controls are taken. Worth a control?
4. Should the camera also bend the **stripe boundaries** (Etch into the band phase), so video shows in the stripes as
   well as the rings? That changes the default Video picture slightly.

## Hardware watch list
- **Renamed controls:** K1 Diamonds → Lean, K4 Stripes → Bands (the diag and production labels differ on purpose).
- **Default positions moved** (same photo): K1 468 → 512, K2 521 → 512, K4 534 → 512, K6 396 → 280.
- **Saw** (S10) looks a little different from v0.5 (it now runs on the Lean DDA).
- **Thin timing:** HD Analog closed at 75.79 MHz on seed 1 (+1.5 MHz); HD Dual needed seed 3 (82.07). If HD Analog misbehaves on hardware, re-seed it first.
- Nothing was cut for fit.

## Next step
On approval: `./heirloom_diag.sh merge medallion`, then repack the previewed diag bitstreams under the production
TOML instead of re-routing (copy `build/programs/ron/medallion_diag/rev_b/bitstreams/*.bin` into the production
build dir, regenerate `program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`),
update SUMMARY / README / PHASE2 and commit only medallion's files and rows.


## K6 top +25 % and default 100 % (2026-11-15)

Ron (2026-11-15): the camera effect at K6 = 100 % 25 % stronger; K6 default 100 % (initial 1023, v0.6.2).
- **Classification: bend** (Etch adds camera luma to the diamond field w), as in production.
- **Change.** Etch ×1.25 at every K6 position: the square-law constant 860 -> 1075, the gain register c_eq
  10 -> 11 bits (0 .. 1072), the etch sum 18 -> 19 bits and the etch term 11 -> 12 bits (≤ 2135); w (14 bits)
  has room. The same change as production medallion (no render for the diag).
- **Build:** router1; HD Analog s1 91.37, SD Analog s1 87.02, HD HDMI s1 84.25, SD HDMI s1 89.53, HD Dual s1 88.87, SD Dual s1 86.97 MHz; worst HD 84.25 MHz; 6968-6995 / 7680 LC; all seed 1. vmprog: out/rev_b/ron/_diag/medallion_diag_v0.6.2.vmprog.
