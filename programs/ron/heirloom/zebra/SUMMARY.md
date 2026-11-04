# Zebra — a neon zebra swirl: chevron stripes cut into staggered ribbons by concentric arc seams

**v0.1.0 (2026-11-04): built, NOT flashed.** Heirloom phase 3 (video clips), made from the still
`photos/vid/WIN_20230404_23_59_18_t9p5.png` (clip WIN_20230404_23_59_18 at 9.5 s). The still shows a neon zebra
swirl. A stripe field of nested tall diamonds runs through a folded rainbow colouriser with black troughs, and
concentric arc seams from the lower left cut it into ribbons. Each ribbon is shifted half a cycle, so the black cells
stagger like bricks. The stripes curl into every seam, which carries a violet/brown rim, and beyond the outermost
seam the stripes run flat and dense. Presets are hand-written placeholders. Working files: `.heirloom_work/rb_zebra/`
(`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **Palette.** The x = 100 column ramp and palfit (k = 12) give one cyclic colouriser, dark → blue, then mirrored back:
  - dark (12,12,10) → dark red (113,37,21) → orange (255,110,56) → tan (205,163,81);
  - → green (38,229,129) → mint (0,247,154) → pale cyan (86,212,179) → lavender (206,158,208);
  - → pink (255,115,227) → violet (147,35,255) → blue (28,5,255).

  Black (max RGB < 45) covers 33.7 % of the frame, and 0.21-0.47 per 4×3 region.
- **Colouriser.** Between big black bands the field rises and falls in humps. The humps alternate between a blue
  peak and a lavender peak, and 12 px thin black lines separate them at the dark folds. That is F = G·tri(W) − T
  folded at every integer: G ≈ 3.55, T ≈ 0.9 (hump peak 2.65 = lavender). The big black cells are F < 0.
- **Seams.** These are the rim pixels, which sit far from the palette ramp because they are blends.
  - A 5-term phase S0 = −4.094u − 5.074v − 4.514uv − 1.426v² − 1.486uv² − 0.177 fits them, with
    u, v = (x − 960, y − 540)/1000. Its phase coherence is 0.869 over 40k rim pixels.
  - The outermost seam, (80,1067) → (533,0), lies on S0 = 3.04.
  - Seam spacing is 85-175 px (median 146).
  - Per line the phase is linear in x. As a patch, that is an H ramp whose gain a V parabola bends, plus a V
    parabola, into a staircase.
- **Stripe field.** A structure-tensor orientation fit scores 0.665 (all-horizontal stripes score −0.11). It finds
  nested tall diamonds centred at (1000, 520):
  - side slopes 2.36 (left) and 2.17 (right) in units of the vertical term;
  - flat, horizontal stripes beyond x = 332 and x = 1645;
  - a sharp horizontal break in orientation at y = 520, the vertical triangle's peak.

  The in-ribbon displacement fitted to about 0 (D/f = 0.02), so the seams are a **staircase** in the stripe phase,
  not a continuous saw.
- **Densities.** Outside the outermost seam (upper left) the vertical period is 473 px (black rows at 0-80 and
  472-554). Inside, the crease zone's stripes are 1.34× denser per palette step (55 vs 74 px per F unit), which
  gives f = 0.55 inside and a vertical gain of 1.8 in the outer ring.
- **Per-ribbon step.** The fitted step is δ = 0.5 cycle per ribbon: alternate ribbons swap black and stripes,
  which is the brick stagger. Its blurred black-mask correlation is 0.34. The exact per-ribbon offsets can't be
  recovered (free per-ribbon fits reach only IoU 0.3-0.46).
- **Rims** (6× zooms). On the outer side of each seam there is:
  - a ~2 px dark line;
  - a ~3 px brown line (150,100,75);
  - a ~8 px lavender glow (150,120,225) blended over stripes that curl into the seam just before it.
- **Alternate stills of the same patch.**
  - t 8.0 s: denser seams.
  - 23_56_31 t31.5: shredded, with chroma noise.
  - 00_01_57 t0.5: 2.5 min later, the seam field alone through a red/yellow colouriser (S10 Fire).
  - 00_09_34 t5.0: blue and torn, a later patch.

## Motion in the clip
- **Seams.** Fitted per m-frame (coherence 0.83-0.85), the seam count across the frame diagonal goes 16.5 (t−0.5 s)
  → 17.3 (t) → 12.2 (t+0.5 s) → 11.0 (t+1.0 s). The clip ends at t+1.0 s. Ron was sweeping the staircase
  frequency down, ×0.70 in half a second.
- **Pivot.** The seams spread about a pivot just outside the outermost seam (solved S0 = 3.38 / 3.56 from the two
  later frames; 3.5 is used). So the outer horizontal zone grows as the density falls.
- **What stays still.** The stripe field doesn't move: the outer zone's patches correlate 0.85-1.0 at zero shift.
  Every ribbon is re-cut as the seams move.
- **In the program.** That sweep is P12 Swirl, with the same pivot. The program's +0.5 s and +1.0 s renders at
  Swirl ×0.705 and ×0.636 reproduce the clip's coarser ribbons over the static base.
- **Default motion.** K4 Speed 640 drifts the seams outward at about 0.17 seam/s (~25 px/s at 1080) while the
  stripes creep the other way (0.02 cycle/s). At full Speed the seams move 3 seams/s.

## What it does
1. **Seam phase.** S = 3.5 + sc·(S0 − 3.5) + drift + ¼·etch, where sc is the Swirl. The outer ring is the region
   where the undrifted S ≥ 3. Elsewhere the ribbon is n = ⌊S⌋ and σ = frac S.
2. **Stripe phase.** W = f·ky·(|y − 520|·m(x) + crease(x)) + δ·n + w0 + hook(σ) + etch + flow.
   - crease = k·|x − 1000|, clipped at x = 332 / 1645; the right slope is 0.919k.
   - m ramps from 1 to 1.8 over 220 px beyond the clips.
   - The outer ring instead uses W = 1.8·f·ky·|y − 520| + 0.5: dense horizontal stripes with no crease.
3. **Colour.** F = G·tri(W) − T. Black where F < 0, otherwise palette(fold F); the palette's dark end is black,
   which makes the thin zebra lines.
4. **Rims.** At σ ≈ 0: a dark line, then a brown line, then a 4-level lavender glow. All three widths scale with
   the seam spacing (they are multiplied by sc).
5. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the photo's softness, and the HD HDMI bar
   fix), then luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bands | the colouriser's gain G, 1.26 .. 3.55 (512, the photo) .. 10: fat single sweeps per hump .. hairline zebra bands |
| K2 Chevron | the diamonds' crease slope k = 6·(k2 − 512)/511: inverted V needles (0) .. flat horizontal stripes (512) .. the photo's Λ (713, k = 2.36) .. steep needles (1023) |
| K3 Shift | how far each ribbon's stripes jump at a seam: 0 (seams show only their rims and curl) .. ½ cycle (512, the photo's bricks) .. 1.35 cycles |
| K4 Speed | bipolar, still within ±8 of the centre, square law. Right: the seams drift out of the swirl (up to 3 seams/s) while the stripes flow against them at ⅛. Left: the reverse. 640 = the default slow drift |
| K5 Cells | the black threshold T, 0 .. 2.2: no black cells (0) .. the photo (420, T = 0.9) .. mostly black, with neon slivers |
| K6 Etch | how far the camera (or the Osc) bends the stripes: 0 .. 3 cycles per full luma (square law). The seams take a quarter of it. 180 = 0.09 cycle |
| S7 Curl | On (the photo: stripes curl into each seam, lavender glow) / Off (clean cuts; the dark and brown lines stay) |
| S8 Mirror | the seam field mirrored about the centre: a symmetric arch of seams |
| S9 Seams | Arcs (the photo) / Saddle: hyperbolic seams (S0 = −16uv), with two calm outer-ring corners |
| S10 Colours | Neon (the photo) / Fire (the same patch's red-yellow-pale colouriser, from clip WIN_20230405_00_01_57) |
| S11 Source | Video / Osc |
| P12 Swirl | seam density sc = 2^(3.3·(p − 0.45)) (exponential, glided): 0 % = ×0.36, a few wide ribbons and a large outer zone (the direction of the clip's sweep); 461 = the photo (~17 seams across the diagonal); 100 % = ×3.5, a shredded swirl (~60 seams) |

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, is added to the stripe phase W,
  and a quarter of it to the seam phase S.
  - The stripes ripple around the picture and the seams bend with it, as a luma CV patched into both oscillators
    would.
  - At the default Etch (180) the swirl's composition reads clearly. On a heavily textured source (the
    `selfie.jpg` render) the stripes fray at Etch 300 and above, which is why the default is 180.
- **Osc mode.** lozenge's 640 × 540 virtual-px triangle lattice, drifting, feeds the same path, so Etch stays live.

## Engine
- **Per-field sequencer** (vblank zone). It has one serial shift-add multiplier, with the slot decoded two clocks
  ahead and the operands pre-muxed, and a shared glide for K1, K2, K3, K5, K6 and P12. Per field it computes:
  - the Swirl sc from a 256-entry exp2 ROM;
  - the 6 seam coefficients × sc, written into a coefficient block RAM (Arcs or Saddle constants);
  - the crease slope, its per-pixel steps and its held height;
  - G (exp2), T, the etch gain (Etch²), and Speed² → the drift and flow steps;
  - the three rim widths × sc, and the ribbon step δ.
- **Fold table.** It is then refilled: 1024 entries of {black, palette position} for F = (G·4i >> 6) − T, built by
  an adder DDA during the zone.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in about 185 clocks, so it fits SD's
  858-clock line. It uses one serial multiplier for 10 products, with coefficients read from the RAM 3 clocks ahead:
  - C1 = KA1 + v·(KA4 + v·KC3) and A0 = v·(KA2 + v·KA5);
  - the seam's line start K0 + A0 − C1·|u0| and its per-pixel step C1·du;
  - Wy and Wr0 from |y − 520|;
  - the side-ramp start and step.
- **Beam DDAs.**
  - Seam: S += D >> 8. For Mirror, the centre pixel's increment is negated.
  - Crease: held, descending to x = 1000, ascending to the right clip, then held.
  - Side ramp: down to 0 at the left clip, up from the right clip.

  Their zone flags are registered a clock ahead from x + 2 steps.
- **Pixel path (18 stages).**
  1. S (Q14) with the drift; the outer-ring test.
  2. + ramp.
  3. Luma.
  4. Etch nibbles.
  5. Etch sum.
  6. Etch into S and into both W branches.
  7. n and σ: the ribbon-step partials, the hook address, the rim flags.
  8. Ribbon step.
  9. Hook, + step.
  10. W select.
  11. tri → fold-table address.
  12. Fold table.
  13. Fabric copy.
  14. Palette address.
  15. Palette ROM.
  16. Fabric copy.
  17. Base colour and glow difference.
  18. Rim lines.

  Then the compose and the output conditioning. The output boxes' 8-sample histories ride a 256 × 64 block-RAM ring
  (written every clock, read 6 behind, data plus a fabric register), with the per-line running-sum resync.
- **Memories.** Line buffer 4 EBR, palette 512 × 32 (Neon, Fire) 4, fold table 1024 × 16 4, ring 4, coefficient
  RAM 2, hook ROM 1, exp2 ROM 1: **19 EBR**. The fold table, ring and coefficient RAM carry `no_rw_check`; the only
  transparency emulation is the line buffer's.
- **Contracts.**
  - Generated avid; one latency (C_LAT 24) in every mode.
  - Blanking gate; luma pre-scaled 16/13 and capped at 940 (whites ≈ 777 out).
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Verified (GHDL, 720p60, decimation 1).**
  - The default Osc frame matches the integer model `imodel.py` on 97.0 % of pixels within 8 levels (mean 2.0;
    the residual is the Osc etch's one-line buffer lag).
  - The Video frame (camera `selfie.jpg`) has a mean difference of 5.3.
  - A full-Speed Osc frame after 6 warm-up fields equals the model's field 6 (97 % within 8 levels), so the drift
    accumulates as designed.

## Status
- **v0.1.0, built 2026-11-04 with router2 (the queue default). NOT flashed.**
- `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh zebra` → `out/rev_b/ron/heirloom/zebra.vmprog`.

| Config | Seed | Post-route Fmax (MHz) |
|---|---|---|
| HD Analog | **3** (seed 1 stalled the router, seed 2 missed) | 75.83 |
| SD Analog | 1 | 76.35 |
| HD HDMI | 1 | 77.24 |
| SD HDMI | 1 | 81.77 |
| HD Dual | 1 | 75.52 |
| SD Dual | **2** (seed 1 stalled) | 69.51 (Fmin 27) |

- 6917-6940 / 7680 LC (90 %), 19 / 32 EBR. Every bitstream passed iceunpack, and the packed config carries the
  labels and presets.
- **HD margins are thin (75.5-77.2 MHz).** An identical netlist (only the Etch default differed) closed all six on
  seed 1 an hour earlier (HD Analog 78.53, HD HDMI 79.23, HD Dual 75.66) under less machine load. If a rebuild
  struggles, try `NEXTPNR_ROUTER="--router router1"` before restructuring.
- No pin-skew seed screening was run.
- **Fit history.** The first RTL (multipliers in the pixel path, FF box histories, 22 stages, a Zone-plate S9)
  overflowed: 8758 / 7680 LC (114 %). The fix:
  - crease and side ramp → DDAs;
  - colouriser → a per-field fold table;
  - seam coefficients → block RAM;
  - box histories → block-RAM ring;
  - 18 stages;
  - Zone → Saddle (Zone needed a second-order seam DDA).

  That brought it to 3883 LUT / 2871 FF → 90 %.

## Deviations
- **Cell placement.** Inside each ribbon the cells follow the fitted staircase law (half a cycle per ribbon), not
  the photo's individual per-ribbon offsets, so single black cells sit elsewhere. Composition, seams, stripe
  orientation and density, black share (0.36 vs 0.37) and the brick stagger all match.
- **Eye.** The photo's closed-ring eye at (470, 880) is not placed. A nested-diamond eye appears where a ribbon
  leaves the diamond centre (1000, 520) uncut.
- **Seam fit.** The seams are a 5-term polynomial. The outermost seam is off by up to ~20 px at the lower-left
  corner (S0 = 2.81 at (80, 1067)).
- **Curl and rims.** The curl has one fixed exponential shape in every ribbon. The rim glow is 4 discrete levels
  (½, ¼, ⅛, 1/16), and the dark and brown lines are hard, where the photo's are analog blurs.
- **Side zones.** The density ramp beyond x = 332 / 1645 is linear over 220 px.
- **Default motion.** It is a slow outward seam drift. The clip's own motion was Ron sweeping the seam density,
  which is P12 Swirl.
- **Cut for budget.** The Zone-plate rings switch (replaced by Saddle), finer glow levels, and full-precision
  colouriser gain (now 1/64 steps) and crease slope (11 bits).
- **Resolutions.** 720p shows the same geometry as 1080p. **SD shows the centre 4:3** (virtual x 240-1680), so most
  of the outer horizontal ring at the left is cropped on SD (only its edge shows at the top left), and SD is softer
  (the output blur is per output pixel).
