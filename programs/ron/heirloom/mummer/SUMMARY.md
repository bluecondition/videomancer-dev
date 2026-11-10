# Mummer — green mask faces fluted by stripes, rolling down through red and violet lightning

**v0.1.1 built 2026-11-10, NOT flashed.** v0.1.1 removes the single-row lines: the run check finds 0 at the defaults and
at P12 0 / 100 % over 30 roll phases at 720p and 1080p. Heirloom from the video clips (phase 3b), made from the still
`photos/vid/WIN_20230804_22_27_52_t6p0.png` (clip WIN_20230804_22_27_52, 12 s, frame at 6.0 s; the same patch at 0.5 s and
in clip WIN_20230804_22_26_56 at 0.5 s). Two green mask faces sit side by side: each is a wide lens with pointed cheeks over
a narrower chin, fluted by soft vertical stripes and glowing cyan in the middle, outlined dark green, black, blue and
violet. Around them is a busy red / crimson / magenta / violet ground cut by zigzag lightning and thin cyan and white lines,
with nested blue chevrons over each face, a red band across the tops of the faces, green lobes at the top and green blobs
between the faces at the bottom. In the clip the whole structure rolls down, the faces narrowing as they descend while green
comes in from the top. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_mummer/`:
- `notes.md`: the working notes;
- `pal12_*`: palfit at k = 12;
- `r2m.py`, `dpm.py`, `dp3.py`, `feat.py`: the field fits;
- `palette.py`: the palette design;
- `proto.py`: the float proto;
- `imodel.py`: the integer model of the RTL;
- `thindiag.py`, `thinsweep.py`: the single-row line checks;
- `fr/`: the clip at 10 fps.

Measured elements:
- **Faces.** The mirror axes are x = 550.5 and 1483.5 (flip correlation), one horizontal period of 933 px. The
  between-face columns (x ≈ 84, 1017) are mirror axes too.
  - Green half-width per row (both faces, both sides): 18 px (row 348), 136 (384), 216 (420), 296 (456), 333 (492) and 360
    (rows 516–552, the pointed cheeks).
  - Below that: 347 (600), 314 (648), 265 (684), 210 (720), 123 (756), the chin at 108–114 (rows 780–830), 84 (900), 35
    (960) and 16 (984).
  - The cyan core fills rows 516–672, half-width up to 217 px.
- **Field.** Every row is one colouriser over F = V(y) + g(y) T1 + g2(y) T2, where T1 is the face triangle oscillator
  (0..466 px from an axis) and T2 is the same triangle at twice the rate.
  - The face rows were solved exactly from the green-edge (F = 353) and red-start (F = 437) positions. The rest came from a
    per-row DP fit (`dp3.py`) and hand-set knots.
  - The colouriser is a triangle fold, mirrored at F = −200 (cyan) and at F = 640 (the dark-red core). That fold is why
    green comes back between the faces at the bottom.
- **Palette** (palfit k = 12): red (249, 7, 16) 16.2 %, green (0, 236, 2) 12.9 %, dark green (1, 186, 3) 10.5 %, magenta
  (225, 57, 191) 10.4 %, violet (79, 7, 222) 9.3 %, purple (131, 11, 139) 9.1 %, crimson (166, 10, 41) 9.1 %, cyan (8, 234,
  236) 6.5 %, dark (33, 17, 43) 5.4 %.
  - Outward from the green: light rim, dark green, black (~25 px), navy, blue, a cyan line, violet, purple, magenta, pink, a
    white-pink line, then red.
  - The RTL default frame: red 21.2 %, green 12.3 %, dark green 8.3 %, magenta 13.8 %, violet 9.5 %, purple 6.5 %, crimson
    7.3 %, cyan 8.1 %.
- **Stripes.** Brightness flutes inside the green have a period of 109.5–110.3 px (FFT), brightest at the face axis. The same
  period appears in the background (B autocorrelation peaks at 106 / 215 / 324 px).
  - It is not a harmonic of the face oscillator (the phase is off by ~0.46 cycle at the second face), so it is an
    independent horizontal oscillator.
- **Lightning.** Near-vertical edges in the background swing in a triangle wave of ~32–45 rows, ±20 px (x 1000–1080, rows
  420–500). Chains at the left edge have a period of ~28–30 rows. The face outlines stay smooth.
- **Red band.** Rows 270–340 run across the whole width, where the vertical multiplier term is near zero.
- **Textures.** The still has an interlace line-pair texture and a few short partial band discontinuities (rows ~765,
  ~935). Neither is modelled, because single-row features are banned on hardware.

## Motion in the clip
- **The structure rolls down for the whole clip.** Vertical cross-correlation of frames 0.2 s apart gives 32–68 lines (mean
  ~46): about 230 lines/s.
- **The green fraction breathes with a ~5.4 s period.** It reads 0.42 / 0.05 / 0.34 / 0.14 / 0.39 at 0.7 / 3.9 / 6.3 /
  9.4 / 11.3 s, so the rolling vertical oscillator passes screen-fixed structure.
- **The faces morph as they roll.** The m-frames match the still best at ±132 lines per 0.5 s, with large residuals:
  - lens with a double cyan core (−0.5 s);
  - lens with a chin (the still);
  - a narrow mushroom (+0.5 s);
  - gone (+1.0 s) while a lobed green band comes in from the top.
- **In the program.**
  - The scroll table holds the still's rows less a screen-fixed ramp (P = 0.22 (Y − 540)) and gain (Gf = 1 + 0.6 (Y −
    540)⁺ / 540), plus a 320-line cosine bridge, period 1400 lines.
  - At K4 = 700 it rolls 3.93 virtual lines per field down (236 lines/s at 60 Hz, a 5.9 s cycle). The faces narrow as they
    descend, and the lobed green band enters at the top at +0.5 to +1.5 s, as in the clip.
  - The lightning crawls on its own (three triangle phases).
  - Proto against the m-frames: `cmp_motion.png`. GHDL frames 3 and 7 (`cmp_rtl_motion.png`) are 10 rows apart at 720p.

## What it does
1. **Line engine** (per line, for the next line):
   - Mirror the line if S9 is on, then read the scroll table at its fractional position: (Y × 1024/1400 [×2 for Twin] −
     roll), with 256 entries and 64 sub-steps per entry.
   - Add the screen ramp P(Y) and the gain Gf(Y) = 1024 + 2.125 P below the centre.
   - Apply P12 and K5: V' = 853 + (V − 853) kv kr − off, g = gs Gf kr and g2 = g2s Gf kr (Q11).
   - Compute the stripes gain ga = |g| ka and the lightning jitter zig = za (2 t1 + t2 + t3 − 256) / 128.
   - Look up the tear offset (S10) from a 256-entry table indexed by the 64-line band.
   - Form the start phases of the face oscillator (tear plus the K3 share of zig) and the stripes oscillator (tear plus
     zig).
   - Fold g and g2 into one per-pixel product: near half (hn < 512) (g + 2 g2), far half (g − 2 g2) with
     V + 2046 g2.
2. **Pixel.**
   - F0 = V_half + g_half hn + Etch × luma (mod 4096).
   - The stripes depth w(F0) comes from a 64-entry table: 1.5 inside the green, 0.125 across the outline, 1.6 in the
     ground.
   - Inside the green the stripes oscillator is read without the jitter (straight flutes); outside it, with the jitter (the
     lightning).
   - F = F0 − ga R w.
3. **Colour.** A 1024-entry folded palette (2048 with Gold) is read at (F + 488) / 4. The thin lines have soft luma ramps,
   so no single entry jumps in brightness.
4. **Output conditioning.** Chroma goes through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum resync, luma through
   [1 2 1] on the same centre, then luma 13/16 and the blanking gate. Palettes are pre-scaled 16/13 and capped at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Faces | the face oscillator's frequency: one wide face (0, period ×2.9) .. the clip's two faces (512, 933 px) .. six narrow faces (1023) |
| K2 Stripes | the vertical flutes' depth (the stripes oscillator into the field): smooth faces and a calm ground (0) .. the clip (512) .. deep fluting, the ground breaking into vertical bars (1023, ×2.66) |
| K3 Lightning | the zigzag jitter of the stripes oscillator: straight streaks (0) .. the clip (512, up to ±35 px) .. up to ±118 px with the faces' own oscillator taking 60 % of it, so lightning runs through the outlines (1023) |
| K4 Roll | bipolar roll speed of the vertical oscillator, ±8 deadband (stops; the lightning still crawls), square law; 700 = the clip (236 lines/s down); left of centre rolls up |
| K5 Bands | the colouriser's fold density about the green edge, ×0.44 .. ×2.3: broad blue/green glow bands (0) .. the clip (512) .. dense contour rings and ringed cores (1023) |
| K6 Etch | how far the camera (or the Osc) pushes the field: 0 .. 0.43 field units per luma step; 140 ≈ 0.11, a bend rather than a shred |
| S7 Twin | one row of faces / two (the vertical oscillator at twice the rate) |
| S8 Colours | Mask (the clip) / Gold: gold masks with pale cores on a teal, blue and navy ground, orange lines |
| S9 Mirror | off / the lower half mirrors the upper half: a totem of lenses meeting at the centre line |
| S10 Tear | off / sync slips: 64-line bands shifted sideways by up to ±179 px (a quarter of them unshifted), rerolled every 48 fields |
| S11 Source | Video / Osc |
| P12 MASK | the faces' level and depth: small green faces in a busy violet / red ground (0 %) .. the clip (50 %) .. swollen faces merging into a glowing cyan band with dark eyes and ringed cores (100 %) |

## Video
- **Video mode (the default).**
  - The camera luma, (Y >> 2) − 16, times the Etch gain is added to the field, so the faces' outlines, the flutes and the
    chevrons bend around the picture.
  - At the default Etch the composition stays clear (`rtl_cam.png`, the selfie).
  - With no input, Video mode is the still: the RTL with a black input is bit-exact with the integer model.
- **Osc mode.** A drifting 2-D triangle lattice (240 × 180 virtual px) replaces the camera, fed at 3× the camera gain (a
  per-field gain). The faces crinkle into lobed masks, a second chin bulb pinches off, and Etch stays live.

## Engine
- **Per-field sequencer** (vblank zone, zc 20–600).
  - One serial shift-add multiplier with 16 slots (18 iterations each); the slot is decoded two clocks ahead and the
    operands are pre-muxed.
  - An exp2 ROM serves Faces, Bands and Mask.
  - A shared glide covers K1, K2, K3, K5, K6 and P12 (1/8 of the gap per field, ±2 deadband).
  - Per field it computes the face step and the centre-anchored start phase, kr / kv / kvr, the Mask level, the stripes
    gain, the lightning depth and face-jitter share, the etch gain (×3 for Osc) and the roll step.
  - The stripes start and step and the lightning steps are per-mode constants.
  - At zc 20 the roll, the three lightning phases and the tear reroll counter (48 fields) advance; the line-0 kick is at
    zc 600.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in 292 clocks (SD's line is 858):
  - 16 serial products: D, P, three interpolation fractions, gs·Gf, ×kr, g2s·Gf, ×kr, ga, V', zig, the face jitter, and
    the two start phases plus the inside-the-green phase offset;
  - the near/far field gains and V + 2046 g2;
  - the scroll- and tear-table reads.
- **Pixel path (17 stages).**
  1. Beam (face and stripes phases).
  2. hn, the half select; luma.
  3. (g ± 2 g2) × hn partial products (split 5 / 5).
  4. Sum; etch partials.
  5. Sign; etch sum.
  6–7. F0 (12 bits).
  7. Stripes ROM addresses.
  8. Depth-table index.
  9. {inside, w}; ROM data.
  10. R select, |R| w.
  11. ga × rw partials.
  12. Stripes term.
  13. F.
  14. Palette address.
  15. Palette EBR.
  16. Fabric copy.
  17. Compose.

  Then conditioning stages 18–21. One latency (C_LAT 20) in every mode.
- **EBR (18).** Palette 2048 × 24 (12), scroll table 256 × 32 (2), stripes cosine 2 × 256 × 8 (2), tear table 256 × 11
  (1), exp2 256 × 16 (1).
- **Contracts.**
  - The generated avid, one latency in every mode, and the blanking gate.
  - The HD HDMI conditioning, with the running-sum resync.
  - Virtual 1080-line coordinates (h = half a virtual px, steps 2 / 3 / 4; SD shows the centre 1440 virtual px).
  - Colours in the hardware convention (U = Cr), luma pre-scaled 16/13 and capped at 940.
- **Verified (GHDL, 720p60, decimation 1).**
  - The default frame (black input, frame 3) and frame 7 are bit-exact with `imodel.py` (fields 3 and 7) on 100 % of pixels
    from column 12 on. The tester's output column n equals the model's column n − 8, the model's 8-px beam head start.
  - Frame 7 has rolled 10 rows down.
  - The Osc and camera renders were checked by eye (`cmp_final.png`).
- **Single-row lines** (`thinsweep.py`: runs of 24 px or more, one row darker or brighter than both neighbours by
  > 120 luma):
  - 0 at the defaults, P12 0 and P12 1023 over 30 roll phases, at 720p and 1080p. Both RTL frames: 0.
  - Corners: K5 1023 has 4 runs (1080p) / 13 (720p, ≤ 58 px). K1 0 has 2 runs of 24 px (720p). Twin has 5 (1080p, ≤ 38
    px) / 27 (720p, ≤ 137 px).

## Status
- **v0.1.1 built 2026-11-10, NOT flashed.**
- **Build command:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh mummer`.
- **Results:** ROUTER1, all six configs on seed 1, 18 / 32 EBR, every bitstream passed iceunpack.

  | Config | Post-route Fmax | LC (of 7680) |
  |---|---|---|
  | HD Analog | 92.31 MHz | 7160 |
  | HD HDMI | 86.37 MHz | 7151 |
  | HD Dual | 91.84 MHz | 7143 |
  | SD Analog | 90.88 MHz | 7140 |
  | SD HDMI | 97.61 MHz | 7134 |
  | SD Dual | 86.33 MHz | 7145 |

- **Output:** `out/rev_b/ron/heirloom/mummer.vmprog`.
- **History.**
  - **v0.1.0** (router1, seed 1 ×6, 6891–6917 LC; dupin found 0 `CARRY I0==I1`). Its run check left 0–7 single-row runs
    per field at the defaults (≤ 41 px) and up to 155 px at P12 100 %.
  - **v0.1.1, the sources of the runs.**
    - Per-row quantization noise toggled the thin lines' edges: the table was read at whole lines, and g / g2 were at
      Q8 / Q7.
    - Steep crossings of the lines happened where table transitions roll into the lower screen, and in the bridge.
  - **v0.1.1, the fixes.**
    - The table is read at the fractional line position with 64 sub-steps per entry, and g / g2 are carried at Q11.
    - The bridge is 320 lines (vertical period 1400), the screen gain is 0.6, and two transitions are blended wider.
    - The thin lines have soft luma ramps.
  - **v0.1.1, trims to hold the LC.** The field path is 12 bits, with one per-pixel field multiplier. Five sequencer
    products became per-mode constants, and Gf is derived from P.
  - **v0.1.1, tried and dropped.** A vertical luma filter.
  - **v0.1.1, look change.** The default frame differs from v0.1.0 by mean |ΔRGB| 9.4. The error against the still is
    56.3 (it was 56.8).

## Deviations
- **Faces.**
  - The silhouettes match the still to a few px row by row.
  - The lens cheeks are slightly rounder.
  - The still's thin dark line along the face tops is softened into a gradient. The red band's edge into the face dome is
    blended over ~80 lines, so the dome starts about 8 rows higher.
- **Chevrons.** The nested blue chevrons over each face come out as a diamond of nested bands (blue, navy, black core) with
  pink / magenta rings, not the still's open stack of chevrons.
- **Ground.**
  - The ground is redder than the still: red 21 % against 16 %; the other colour shares are within 1–3 points.
  - The lightning is regular triangle teeth (three summed triangles), not the still's irregular bolts.
  - The left-edge chains (x < 100) are not drawn.
  - Mean |RGB| error against the still is 56: band positions are misregistered over large areas of the busy ground. The
    composition, palette shares and textures are what match.
- **Palette lines.** The thin lines are wider than the still's (~9 F) with soft luma ramps. The background cyan line is an
  azure line and the pink lines are dimmer, so no crossing becomes a single bright row.
- **Motion.**
  - The roll and its direction match, and green enters at the top as in the clip.
  - The faces narrow as they descend, but less than in the clip and over a 5.9 s cycle (the clip's is 5.4 s). They persist
    where the clip shows a mushroom at +0.5 s and no face by +1.0 s.
  - The lightning crawls at fixed rates.
- **Twin** still shows a few single-row runs (it doubles every vertical gradient): up to 137 px at 720p, up to 38 px at
  1080p.
- **Not modelled.** The interlace line-pair texture and the short partial tear lines of the capture.
- **LC.** 7134–7160 (93 %), slightly over the ~7,100 target. Nothing visible was cut.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (virtual x 240–1680): both faces, without the outer ground.
