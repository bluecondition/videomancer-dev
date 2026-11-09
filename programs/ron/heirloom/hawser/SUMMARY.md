# Hawser — amber rope tubes swinging over thin cords, with red glints

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20231116_23_41_37_t3p5.png` (clip WIN_20231116_23_41_37, 9.5 s, frame at 3.5 s; the same patch at
0.5 s and in clip 23_43_35, where it is turned to lemon). Solid amber rope tubes with dark-orange edges and a silver
rim swing in S-curves over bundles of thin peach / amber / silver cords on a green-black ground. Red glints sit on the
cords in two rows. The swing grows from nothing at the left edge to about 300 px at the right, and the S-curves travel
up the frame. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_hawser/` (`notes.md`; `pal14_*` = palfit k = 14; `thick.py` = tube runs per row;
`fitfm.py` / `fitmot.py` = the FM fits; `unwarp*.png` = the still with one family straightened; `profx2.py` =
the colour profiles; `proto.py` + `glint2.py` = the float proto; `imodel.py` = the integer model of the RTL).
- **Two strand families, each a frequency-modulated horizontal oscillator.** The phase is
  p = (x + x0) k (1 ∓ a w(y)), with w = (1 − cos 2π (y F + φ)) / 2. The swing is linear in x, so the left edge holds still
  and the right edge swings the most. Unwarping the still by either fit straightens that family (`unwarp.png`,
  `unwarp2.png`).
  - **Tubes.** Yellow-core left edges at y = 10: 103, 535, 955, 1385, 1810, so the period is 420 px (4.6 across).
    At w = 1 the strands move +67 / +155 / ~+235 / +293 px, which is 0.18 (x + 290). The ramp's zero sits 290 px
    left of the picture, about the HD hblank: the oscillator is hard-synced at hsync. The waveform is a cosine,
    1/F = 1120 lines, minimum at y = 125 (rms 9-11 px over 3 traced strands). Tube widths grow with the swing
    (82 → 100 px), as FM predicts.
  - **Cords.** Period 304 px, x0 = 146, depth −0.053 (they counter-swing a little), 1/F = 567 lines (≈ the tubes'
    second harmonic). They are bundles of three strands 55-58 px apart: peach (232, 175, 129), amber (219, 143, 62)
    and silver (203, 182, 183), each 25-30 px wide, then about 120 px of black.
- **Tube profile** (by tube phase): black to 0.85, then a dark-orange edge (137, 80, 0) at 0.85-0.92 and a yellow
  core (246, 178-209, 37-109, two highlights) at 0.93-0.11. It falls to dark olive (80, 66, 40) at 0.14, then a silver
  rim (169, 151, 147) at 0.17, and is black from 0.23. The tube is opaque from 0.85 round to 0.22 (~155 px).
- **Crossings.** In the (tube phase, cord phase) colour map the tube band covers every cord phase, so the tubes are
  keyed in front of the cords. Where a tube swings across a bundle you get the rope / braid look and black lens eyes.
- **Glints.** Red (216, 37, 24), salmon (222, 94, 81) and pink (239, 145, 141) with white-pink cores, ~5 % of
  pixels. They sit in two rows (y ≈ 200-460 and 480-800), one cluster per cord bundle (the red share peaks at cord
  phase 0.61, on the amber cord), drawn over everything. They take the form of diamond rings, red smudges and
  zigzag "lightning".
- **Palette** (palfit k = 14): black (8, 19, 6) 22 %; yellows / ambers 8 + 8 + 7.7 + 6.5 %; dark browns 8 + 7.7 %;
  orange 6 %; silver / greys 4 + 3.7 + 3 + 3 %.
- **Model vs still at defaults** (`cmpstat.py`, 960 × 540 classes):
  - black 22.4 / 25.2 %, IoU 0.70;
  - yellow 25.9 / 28.0 %, IoU 0.63;
  - silver 11.4 / 8.0 %;
  - glints 7.6 / 5.4 %;
  - mean |RGB| 27.3.

## Motion in the clip
- **Tube wave.** Its phase over the m-frames (t − 0.5, t, + 0.5, + 1.0, + 1.5 s) is 285, 167, −4, −198, −432 lines.
  The S-curves travel **up** at ≈ 360 lines/s: 0.32 cycle/s, ~3.1 s per writhe. The depth stays 0.16-0.18.
- **Cord wave.** It drifts up ≈ 20 lines/s, nearly static.
- **No horizontal drift** of either family (±6 px). The left 100 px are still (temporal std ≈ 0).
- **Glints** stay in their rows on the cords, but their shapes and colours change every frame.
- **The alternate stills** have the same geometry (tube depth 0.17-0.185, λ 1050-1190; cords −0.053, λ 560). Clip
  23_43_35 has the colouriser turned to lemon yellow, maroon rims, white cords and orange-red glints. Ron's knob across
  the clips was the colour, which is S8's Lemon page.
- **In the program.**
  - φ1 advances 5695 / 2^20 cycle per field at the default Speed (700), so the S-curves rise like the clip:
    `cmp_motion.png` is the model at + 0.5 / + 1.0 / + 1.5 s next to the m-frames.
  - The cord wave runs at 1/16 of that.
  - The glint cells re-roll every 2^15 of a glint phase that runs with |Speed| (≈ 2.6 × /s at the default), and their
    zigzag edges crawl.

## What it does
1. **Line engine** (per line, the NEXT line, ≈ 150 clocks):
   - θ1 = y F1 + φ1 and θ2 = 2 y F1 + φ2; under Braid, θ2 = θ1.
   - w = (1 − cos θ) / 2 from a quarter-sine ROM.
   - g1 = k1 (1 − a1 w1) and g2 = k2 (1 + a2 w2), the per-pixel phase steps.
   - The line starts p10 = (x_first + 290) g1 and p20 = (x_first + 146) g2.
   - The glint row terms: row (310 / 620 lines), 0.586 |dy|, a zigzag 24 tri(y / 62 + t), and the cell offset.
2. **Beam.** Two 29-bit phase DDAs (5 integer bits = the bundle index) and a zigzag DDA. Under Mirror they reverse at
   the centre pixel.
3. **Camera bend.** The camera / Osc sample × K6 gain is added to both phases.
4. **Colour.**
   - The tube ROM (keyed; Wire hollows its core) over the cord ROM (Braid swaps in a black-keyed tube page).
   - Glints on top: G = size(cell) − |dx + slant · dy| − 0.586 |dy| − zigzags + camera, with dx = (cord phase − 0.56)
     × cord period (px). Each cell (bundle × row × tick) reads a 256-entry table: size 8..118 px, 2-bit slant.
   - Ring colours by G: red, salmon, pink, a white line, pink, red, tinted red (the strand's luma), red, tinted red.
5. **Output.** The shell's two [1 2 2 2 2 2 2 2 1]/16 boxes (Y, U, V), luma 13/16, the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tubes | The tube oscillator's frequency, so the tube count: period 420 px × 2^(−1.75 (k − 512) / 512) = 1.4 (0) .. 4.6 (512, the photo) .. 15 (1023) across. The tube width keeps its share of the period |
| K2 Wave | The LFO wavelength for both families: 1120 × 2^(−2 (k − 512) / 512) lines = 4480 (one gentle bow) .. 1120 (the photo) .. 280 (tight ripples). The cords run at half the wavelength |
| K3 Cords | The cord oscillator's frequency: period 304 px × 2^(−1.75 (k − 512) / 512) = 1.9 .. 6.3 (the photo) .. 21 bundles across. The glint cells follow the bundles |
| K4 Speed | Bipolar (±8 deadband at 512): the S-curves travel up (right) or down (left), v = d² · 45 / 256 / 2^20 cycle per field (d = k − 520). 700 = the clip (0.32 cycle/s), 1023 = 2.5 cycles/s. The cords move at 1/16, and the glints re-roll with it |
| K5 Glints | Glint size: (k − 512) / 4 px added to every cell. 0 = no glints, 512 = the photo, 1023 = big red rings filling the two rows |
| K6 Camera | How far the camera (or the Osc) bends the tubes and cords and sprouts glints (square law). The default 384 = 0.067 period (28 px of tube) per full luma plus ~34 px of glint; 1023 = 0.48 period |
| S7 Braid | Off (the photo) / On: the cords become a second set of tubes, black between them, swinging against the first on the same wave at 0.9 × the twist. A braid of crossing tubes with black eyes |
| S8 Palette | Amber (the photo) / Lemon (clip 23_43_35: lemon tubes, maroon / crimson rims, cool-white cords, black ground, orange-red glints) |
| S9 Mirror | Off / On: the right half mirrors the left. The still edges are at both sides and the wildest swing meets in the middle |
| S10 Tubes | Solid / Wire: the tube cores are hollow, so only the dark-orange edges and the silver rim remain and the cords show through |
| S11 Source | Video / Osc |
| P12 Twist | The tubes' swing depth a1 = 23 p / 65536. 0 = straight hanging tubes and cords, 435 = the photo (0.153), 1023 = 0.36 (the right-hand tubes swing ~2.4 periods, crossing their neighbours in a tangle). The cords counter-swing at 0.367 a1 |

## Video
- **Video mode (the default).**
  - Camera luma ((Y >> 2) − 16, one line buffer) × the K6 gain is added to both strand phases, so the tubes and cords
    bend around the picture.
  - Its positive part also sprouts glints: bright parts of the picture grow red rings.
  - At the default Camera the composition stays intact. With no input, Video mode is the photo.
- **Osc mode.** The shell's drifting 640 × 540 virtual-px triangle lattice, zero-mean and × 8, feeds the same path.
  - The ropes bend into slow zigzag kinks.
  - Glints sprout at the lattice peaks (glint term at 1/4 of Video's).
  - K6 scales both.

## Engine
- **Per-field sequencer** (vblank zone, zc 20..175).
  - A shared glide for K1..K6 and P12.
  - One serial 17 × 16 multiplier; loads use job codes registered a clock ahead.
  - An exp2 / quarter-sine ROM, shared with the line engine by zone time.
  - It computes k1, k2, the cord period in px, F1, the speed (d², × 45), the camera gain (k6², × 2298), a1 / a2, the
    glint size offset, and the per-resolution origin, centre and zigzag step.
  - It advances φ1, φ2 and the glint phase.
- **Per-line engine.** Kicked at each line start (line 0 at zc 1300) and computing the next line: one serial 22 × 16
  multiplier, 8 jobs.
- **Pixel path.** 18 stages to the colour register:
  - phases ride to stage 6;
  - the camera sample, then a 12 × 10 camera product;
  - bend on both phases;
  - palette and cell addresses;
  - dx = df × period (9 × 10);
  - EBR, then fabric copies;
  - key / compose, slant, |dx|, G, ring, glint colour, a pad, 10 bits.
- **Output conditioning** from the shell: two 9-tap boxes whose 8-back taps sit in block-RAM rings, with the per-line
  resync, luma 13/16, the blanking gate, and a sync ring.
- **ROMs** (`hawser.py` → `hawser_pkg.vhd`; Y / U / V in the hardware convention, U = Cr, luma pre-scaled 16 / 13,
  cap 940):
  - TUBE 512 × 32 (Amber / Lemon × 256: Y8 U8 V8, key, core);
  - CORD 1024 × 24 (cords Amber / Lemon, Braid tube pages);
  - SEQ 512 × 16 (exp2 + quarter sine);
  - SBOX 256 × 16 (glint cells);
  - plus a 2048 × 8 line buffer, four box rings and a sync ring.
- **Contracts.**
  - Generated avid; one latency (24) in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 1440 virtual px).
  - No pixel-path multiply shares a stage with another operation.
- **Verified.**
  - GHDL 720p60 at decimation 1 with black input is **bit-exact with `imodel.py`** (mean |diff| 0.00).
  - Osc and camera renders match the model's structure.
  - Extremes at 30 warmup frames match the glide-aware model (`xcheck_all.png`): K1 / K2 / K3 at 0 with P12 1023,
    and all at 1023 with P12 0; Braid + Lemon + Mirror + Wire at Speed 1023; Osc with K6 1023 and Speed 0.
  - Motion: at Speed 1023 the RTL's φ1 keeps advancing (52000 after 5 fields against the model's 51535, and still
    rising at 9).
  - The sim caught a speed fold: `resize(v_d - 520, 9)` folds |d| ≥ 256 in GHDL *simulation*. It is now a slice.
    GHDL *synthesis* truncates (a test entity shows `n7(8 downto 0)`), so both builds produced the same bitstreams.

## Status
- **v0.1.0 built 2026-11-09 with ROUTER1, NOT flashed.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh hawser`
    (or `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh hawser`).
  - Output: `out/rev_b/ron/heirloom/hawser.vmprog` (454,454 bytes, unsigned). iceunpack OK × 6.
- **Fmax (post-route, the builder's routed figure), all on seed 1.**

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 85.34 | 6,840 |
  | HD HDMI | 1 | 90.99 | 6,834 |
  | HD Dual | 1 | 93.86 | 6,840 |
  | SD Analog | 1 | 90.65 | 6,836 |
  | SD HDMI | 1 | 92.82 | 6,836 |
  | SD Dual | 1 | 94.61 | 6,824 |

  6,824-6,840 / 7,680 LC (89 %), 22 / 32 EBR.
- `dupin.py`: 0 `CARRY I0==I1`. The one LUT with a duplicated sign input (the slant's 1.25 dy) was split before the
  build.
- Presets are hand-written placeholders: capture Ron's looks with `./capture_preset.sh`.
- **Watch on hardware:** no pin-skew screen was run. If HD HDMI shows lines, re-seed that config.

## Deviations from the photo
- **Glints** are a stylised generator: ragged nested diamond rings, one cell per cord bundle and row, re-rolled about
  2.6 × /s. The photo's are camera-like red smudges and zigzag lightning that change every frame. Placement and
  sizes differ (hashed cells), and the share is 7.6 % against 5.4 %.
- **Crossings** are a hard key (tube in front). The dark kinks / "thorns" at the photo's overlaps (red thorns in clip
  23_43_35) are not drawn.
- **The LFO** is a pure cosine. The photo's tubes have a steeper local fall near y ≈ 870-890.
- **Colours.** Silver reads a little more (11 % against 8 %) and yellow a little less (26 % against 28 %). The black
  ground share is 22 % against 25 %.
- **Softness** is horizontal only (the shell's output boxes). The photo's camera blur is also vertical, and its faint
  scanline texture is not drawn.
- **Mirror** does not mirror the camera / Osc bend.
- **Resolutions.**
  - 720p is identical to 1080p.
  - SD shows the centre 4:3 (virtual x 240..1680), so the photo's still left edge (the ramp's zero, x ≈ −290) is
    further off-screen. At SD the leftmost visible tube already swings ~±50 px.
  - Line widths are in virtual px, so they look the same at every resolution.
