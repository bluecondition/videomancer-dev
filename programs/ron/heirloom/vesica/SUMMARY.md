# Vesica — an orange eye in a pastel lattice

**v0.5.0 (2026-10-27): built, NOT flashed.** Heirloom #13, rebuilt from the photo
WIN_20230307_01_04_17 (a capture of Ron's analog synth). A pale green ground crossed by an X
lattice of pastel bands, with an orange vesica in the middle (the lens where two circles meet).
The vesica is finely striped, and its right edge is torn into flame teeth. A translucent brown
plate joins it to two transparent side copies. Orange crescents sit above and below, their thin
arcs running on through the copies. Zig-zag dashes and brown line rows fill the top and bottom.
Presets are hand-written placeholders.

## Reading of the photo
Fitted numerically (`.heirloom_work/rb_vesica/`: latfit*.py, rhofit*.py, notes.md) and held up
in a float prototype (proto.py), then in an RTL-exact integer model (imodel3.py).
- **Lattice.** Two summed triangle oscillators into a pastel colourizer:
  F = A(x) + 1.03 B(y) − 1.05.
  - A: period 966.0 px, peak at x 972.0, with a soft top knee (w 0.044, so the bands turn
    horizontal at the left edge) plus a slow 0.126 u² − 0.0096 u bend (u = (x − 980)/960).
  - B: period 516.3 lines, peak at y 260.9.
  - Bands by |F|, from the core outward: lavender (169,180,255) below 0.056; pink (240,179,255)
    0.094–0.131; aqua (165,254,228) 0.18–0.23; sky (166,244,253) 0.256–0.294; yellow
    (251,255,178) 0.331–0.356; the pale green ground (202,255,175) above 0.42.
  - Purity cost 0.0112; the green/yellow contour sits within 5–15 px of the photo everywhere.
  - Glitch rows at y 253–293 and 746–788, near B's peaks: notches in the vertical waveform that
    remap whole rows to lavender, pink or yellow.
- **Orange layer.** Two circle families on one horizontal fold (period 944.8 px). They share the
  mirror axis y = 521.9 and a compressive vertical waveform Y = d + a − 0.639e-3 a²
  (a = |y − 521.9|).
  - Vesica circle: cx 1013.3, d 665.3, R 804.9, giving tips at x 560 / 1466, top 369, bottom 676.
  - Crescent circle: cx 997.4, d 977.8, R 1233.2. It forms the crescents' inner edges and the
    thin arcs that fold at x 525 / 1470 through the copies.
  - Fit error 1.9 px rms over 368 edge points. Independent circles disagree (vesica r 706 at
    d 553, crescent r 792 at d 472); one shared compressive waveform explains both.
- **Vesica.** Translucent orange over the lattice, so the lattice's X shows through darker.
  - Stripes: copies of the lens's left edge every 69.5–71 px, i.e. a horizontal oscillator
    hard-synced to the left edge.
  - The right (falling) edge is torn: each row is pulled in by T = 7–51 px. T is a sawtooth of
    the lens half-width with a 37 px period that rises over 45 % of each cycle, then holds,
    giving hooked flame teeth with dark-red tails.
  - A darker red rim, and a bright inner rim on the left half.
- **Plate.** Translucent brown (per-channel blend), between and beside the lenses:
  - a bar |dy| < 67 between the lenses;
  - wings |dy| < 152 on the right of each copy, out to a 1.53-slope diagonal;
  - a tan halo at the lens edge and a thin axis line between the tips.
- **Side copies** at x 68 and 1958: transparent, with a thin orange outline. The left copy's torn
  right half is the plate's ragged edge.
- **Crescents.** Between the crescent circle and 40 px (top) / 44 px (bottom) beyond it. They
  taper linearly, with a rounded shoulder, to tips at x 605 / 1345 (top) and 600 / 1368 (bottom).
  Top: orange (241,65,12) → tan (227,164,82) outward. Bottom: red (226,75,23) → (241,88,26).
- **Textures.**
  - A dash band 40–75 px outside the crescent circle, and beyond 132 px: short slanted zig-zag
    strokes on ~9.5-line rows, with a 60 px horizontal period and a half-period stagger.
  - Brown translucent line rows in the top and bottom ~150 lines, at |dy| = 518, 509, 495, 484,
    469 … 338 (mirror-symmetric, so a deterministic waveform, not noise).

## What it does
1. **Lattice.**
   - A: a 24-bit phase accumulator folded to a triangle. Its top knee comes from a ROM that
     returns the corrected value directly; the bend comes from a 512-entry ROM over x.
   - B: per line from the engine (one multiply), with the glitch notches from a ROM.
   - |F| × gain (5-bit mantissa × 2^e) indexes the 64-step band palette sampled from the photo.
   - In Video mode the camera luma (Osc: the shell oscillator) is added to F.
2. **Orange layer.** For each family, X² comes from a second-order DDA in half-pixels.
   - The DDAs reload at every fold wrap from per-field constants. Fold periods are kept a multiple
     of 24 half-px, so every wrap lands on the same residue in all three timing classes.
   - For the torn lens, X² also reloads, per line, where the tear starts (w = u + 2T).
   - (ρ² − R²) against fixed thresholds (set at the photo's radii) gives the lens, its rim, inner
     rim, halo and tails; the crescent band, its shading and the arcs; and the dash / line-row zones.
3. **Stripes.** A 16-bit phase accumulator per pixel that steps across each fold wrap by a
   per-field constant. The camera / Osc etch is added to the stripe phase too.
4. **Colour.** A 512-entry palette (layer × |F| index: lattice, plate, two dash colours, four
   stripe levels), then one blend stage (weights 0 / ½ / 1) for rims, arcs, tails, halo, axis and
   the shaded crescent. S10 Azure mirrors the chroma.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Lattice | the X lattice's cells | cell size 0.5×–2× (ROM over the glided knob); 512 = the photo |
| K2 Bands | the pastel bands | band width: colourizer gain 2.2× (thin) … 0.45× (fat) |
| K3 Stripes | the vesica's stripes | stripe pitch 24 … 71 (the photo) … 160 px |
| K4 Crescents | the crescents | crescent width 0 … 2× (the photo at 512); taper slope fixed |
| K5 Speed | everything that moves | bipolar, still within ±8 of centre; speed² per field into one motion phase. Stripes stream right into the lens, the lattice (with its glitch rows) slides and rises, dashes and line rows crawl, teeth ripple. Default 640 |
| K6 Etch | the lattice bands and stripes | how hard the camera (Video) or the oscillator (Osc) bends them: 230 (k/1023)², as a 3-bit mantissa × 2^e. Default 400 |
| S7 Tear | the flame teeth | On (the photo) / Off (a clean lens) |
| S8 Twins | the side copies | Clear (the photo) / Filled (a row of striped vesicas) |
| S9 Texture | the dash band and line rows | On / Off |
| S10 Palette | all colours | Pastel (the photo) / Azure (chroma mirrored: a blue eye on a rose lattice) |
| S11 Source | — | Video / Osc |
| P12 Eye | the vesica | opening h = R − d: 24 (a slit between the crescents) … 140 px (the photo, at 50 %) … 290 px (the eye fills the frame height and swallows the crescents; the copies are pushed to the edges). The fold period grows with the eye's tip so the copies never overlap. Glided 1/8 per field |

## Video
- The camera luma (8 bits, tapped a few pixels early, which is invisible in a bend field) is
  added to the lattice field F, so the band edges fray and bend around the picture.
- It is also added to the stripe phase. At the default Etch the composition reads unchanged.
- Osc substitutes the shell's triangle oscillator (`s_ox/s_oy/s_ot`, `f_tri8`) in the same path,
  so K6 stays live in Osc mode.

## Engine
- **Per field** (vblank zone):
  - Six shared-datapath glides (K1–K4, K6, P12).
  - Knob ROMs indexed by the glided knob >> 2: the K1 lattice step, the K3 1/pitch, the K2 gain
    and K6 etch as mantissa/exponent pairs.
  - Five eye ROMs indexed by P12 >> 2, computed by `vesica.py` with integer arithmetic: R², the
    tip, the half-height, the plate bar, the wing limit and the fold period.
  - The fold-counter starts.
  - A field-init run of the line engine's multiplier: speed² into the single motion phase (every
    motion is a power-of-two slice of it), the lattice steps and phase, the three fold-wrap
    squares, and the stripe wrap step.
- **Per line** (kicked at each line start, computing the NEXT line, ~280 clocks; one serial
  multiplier, with squares through a B = A pre-state, and one serial square root):
  - the vertical waveform and both Y²;
  - the lens half-width (square root) and the tear depth (shift-adds);
  - the stripe start phase;
  - the X² DDA loads (tear start, first pixel);
  - the lattice's vertical phase and the glitch notch;
  - the line-row phase and the plate / wing / axis flags;
  - the crescent-colour ROM address.
  Every multiplier operand is a plain register.
- **Pixel path:** 16 stages to the blend, then the shell's conditioning (24 in total).
  - Stage-0 counters: the lattice phase, the two folds with their X² DDAs, and the crescent taper
    counters.
  - The lattice parabola runs late at stage 2 (a ROM over x) and the stripe accumulator at stage 6,
    so no wide pipes are needed.
- **Output:** the shell's conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1] and 13/16, on
  8-bit samples), with the chroma 8-back sample and the sync / avid delay carried in two 256-deep
  block-RAM rings (`no_rw_check`). Then the blanking gate on the generated avid. Latency 24 clocks
  in every mode.
- **ROMs:** palette 3 × 512 × 8, parabola, knee, glitch, line rows, crescent colour, the knob ROMs,
  five eye ROMs, plus the two rings: 22 EBR in all.
- **Verification:** the 720p60 GHDL render matches imodel3.py bit for bit (0 of 921,600 pixels
  differ) with motion and etch off. 22 % of pixels change over 6 fields at default Speed.

## Status
- v0.5.0 built 2026-10-27 with **router1**:
  `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh vesica`. All six configs on seed 1, first
  try; deterministic across two builds.
- Fmax (post-route):

  | Config | Fmax (MHz) |
  |---|---|
  | HD Analog | 80.70 |
  | HD HDMI | 85.11 |
  | HD Dual | 81.88 |
  | SD Analog | 84.78 |
  | SD HDMI | 78.80 |
  | SD Dual | 84.38 |

- 7,413–7,439 / 7,680 LC (96.9 %, above the 92 % heirloom budget), 22 / 32 EBR. iceunpack OK ×6.
- router2 does not close this netlist (HD Analog 67.5 / 70.8 MHz).
- Not flashed.

## Deviations from the photo, and why
- **About 12 % darker overall.** The required 13/16 output luma scale against the HD HDMI bars
  sits on a 1.086 ROM pre-scale. The brightest yellow lands at about 796 — slightly above the ~777
  bar-fix level (accepted: pastel areas only; if HD HDMI bars appear, lower the palette ROM's top end).
  The photo's pastels sit at Y 830–890; matching them would break the drawn-luma cap.
- **Crisper than the photo.** Edges are sharper than the camera-blurred capture, and the textured
  top/bottom bands have more local contrast.
- **Stripes in the torn half run straight**, cut by the teeth, instead of bending to follow them.
  That bending (a divider and three multiplies per line) was cut to fit.
- **Regular tear.** The teeth follow a regular sawtooth of the half-width (period 36.6 px); the
  photo's rows are irregular.
- **Not drawn:** the diagonal streak strip under the bottom crescent, the white flecks at the lens
  tips, and the photo's stronger pink-red X inside the lens.
- **Fixed crescent radius.** It does not follow P12, so a wide-open eye swallows the crescents.
- **S10 is Azure** (a chroma mirror) rather than a second palette, to halve the palette ROM.
- **SD crop.** SD shows the central 4:3 crop of the virtual 16:9 picture, as the other heirloom
  programs do. 720p and 1080p render the same picture.

## Working files
`.heirloom_work/rb_vesica/`:
- `notes.md`: the method, measurements, fit history and builds.
- Fits: `latfit*.py`, `rhofit*.py`.
- `proto.py` / `proto_k.py`: the float prototype and its knob-driven version.
- `imodel3.py`: the RTL-exact model.
- `cmp_final.png`, `sheet3.png`.
- `syn/`: the standalone-yosys ablations.
- `v050/`: staged sources and earlier RTL versions.
