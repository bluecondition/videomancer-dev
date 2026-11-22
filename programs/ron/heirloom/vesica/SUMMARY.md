# Vesica — an orange eye in a pastel lattice

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/vesica.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Vesica Diag`, `out/rev_b/ron/_diag/vesica_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/vesica/`) is untouched.
v0.5.0 (2026-10-27): built, NOT flashed. Heirloom #13, rebuilt from the photo
WIN_20230307_01_04_17 (a capture of Ron's analog synth). A pale green ground crossed by an X
lattice of pastel bands, with an orange vesica in the middle (the lens where two circles meet).
The vesica is finely striped, and its right edge is torn into flame teeth. A translucent brown
plate joins it to two transparent side copies. Orange crescents sit above and below, their thin
arcs running on through the copies. Zig-zag dashes and brown line rows fill the top and bottom.

## Phase 2 (v0.6.0)
No notes from Ron for vesica (free hand, 2026-11-09 batch). Every v0.5 control was live, but the
knobs spanned only about ×0.5..×2 around the photo, K1 zoomed the whole lattice, the crescents
could only widen, and the side copies never moved below the photo's spacing. The round:
- splits the lattice's two oscillators on K1 (the horizontal one runs two octaves each way, the
  vertical one one), so the X tilts as well as scales;
- widens Bands, Stripes, Speed and Etch to about two octaves each way of the photo;
- gives the crescent family a position as well as a width (K4): the crescents, their arcs and the
  dash band close in on the eye or drift away;
- lets the side copies crowd in with the eye's tips below 50 % of P12, so P12 0 % is a row of
  closed eyes;
- above the default Etch, lets the camera (or the Osc plaid) push the eye's outline itself.

Every new effect is exactly zero at the default knob value; no control was renamed and no switch
changed. Phase-2 working files: `.heirloom_work/rb_vesica/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K1 Lattice: cell size ×0.5 .. ×2 (both axes together) | **K1 Lattice**: the horizontal oscillator **×1/4 .. ×1 .. ×4**, the vertical one ×1/2 .. ×1 .. ×2 (one ROM, read at full and half rate); photo at 50 % (exact) | horizontal period 3864 px (0 %: one broad, flat V across the frame, the bands nearly level) .. 966 (the photo) .. 241 px (100 %: eight columns of upright diamonds 241 px × 258 lines). The X's angle swings with the size |
| K2 Bands: colourizer gain ×2.2 (thin) .. ×0.45 (fat) | **×4 .. ×1 .. ×1/4**, exponential, photo at 50 % (exact) | 0 % = hairline lattice lines on the green ground; 100 % = the core colours (lavender, pink, aqua) flood the frame |
| K3 Stripes: pitch 24 .. 71 .. 160 px | **12 .. 71 .. 281 px**, exponential each side, photo at 50 % (exact) | from a fine comb (75 stripes across the lens) to three broad bands |
| K4 Crescents: width 0 .. ×2 | **width 0 .. ×1 .. ×3, plus the crescent family's distance**: out 96 px (0 %) .. the photo (50 %) .. in 192 px (100 %) | the crescents' inner edge from ~260 lines off the axis (the photo) to ~66 at 100 %: a 120 px orange band hugging the eye's rim. The thin arcs and the dash band move with them (at 100 % the dash texture fills most of the frame). Below 50 % the crescents thin and drift apart (half width, 48 px out at 25 %; gone at 0 %) |
| K5 Speed: bipolar square law, top 988 phase units/field | same law to half travel (default **bit-exact**), twice as steep beyond: top **2197** (×2.2) | stripes stream about 1100 px/s at the ends (was ~500); the default drift is unchanged |
| K6 Etch: camera / Osc into the bands and stripes, 230 (k/1023)² | same curve to the default (400), then steeper to **896 (×25 the default; was ×6.6)**, **plus the camera pushes the eye's outline**: ±20 / 40 / 81 px from K6 404 / 612 / 820 | 100 % = the bands turn into a contour map of the picture and the eye's edge (rims, teeth, tails) bulges with the camera's light and dark; in Osc the plaid shakes the eye |
| P12 Eye: opening 24 .. 140 (50 %) .. 290 px; the fold grew with the tip from 50 % up | same opening; **below 50 % the side copies close in with the tips**: fold period 948 → 420 px | 0 % = a row of five slit eyes with the crescents as a garland of arches over the middle; 50 % = the photo; 100 % = the eye fills the frame (as v0.5) |

Unchanged: S7 Tear, S8 Twins (Clear / Filled), S9 Texture, S10 Palette (Pastel / Azure), S11 Source (Video / Osc, default Video).

**Presets** (preset 0 = the defaults): **Photo**; **Row of Slits** (P12 0 %, wider crescents);
**Wide Eye** (P12 100 %, faster); **Hairlines** (small upright cells, hairline bands, fine stripes);
**Fat Bands** (broad flat lattice, flooded colours, broad stripes); **Embrace** (crescents ×3 hugging
the eye); **Camera Eye** (Etch 100 %: the camera pushes the eye); **Azure Storm** (Azure, fast,
heavy etch, fine stripes). Dropped: Slit (→ Row of Slits), Azure Osc, Storm (→ Azure Storm), Twins.

**Engine (new).**
- **K1.** The lattice-step ROM now spans two octaves each way. The horizontal step reads entry K1/4;
  the vertical one reads entry 64 + K1/8 (the same curve at half the rate), straight from the ROM
  output into the field-init kBv product. The horizontal pixel step widened to 18 bits.
- **K4.** The crescent-colour ROM's free upper half holds 128 K4 words {distance d_C (15 bits),
  width / 2 (9)}, read in vblank (the zone switches the address). d_C replaces the engine's constant
  15645 in Y_C = d_C + a − c a²; the width feeds the existing taper widths (now 10 bits).
- **K5.** Past half travel the speed operand becomes 2 sp − 256 before the field-init square
  (phase-step slice widened to 12 bits).
- **K6.** The video-gain ROM is steeper above the default; its exponent reaches 7 and the etch term
  widened to 15 bits. The ROM's spare top bits carry a lens-etch code: c_le = (luma − 128) << 1..3,
  added at stage 1 into the vesica family's radius test (ρ² − R²)/128, so the rims, inner rim, halo,
  tails and teeth all follow it. Zero at and below the default.
- **K2 / K3.** The gain exponent reaches 3 (×4); 1/pitch widened to 13 bits.
- **P12.** The eye ROM's fold floor dropped from m = 79 to 35 (period 948 → 420 px; the copy count's
  bit 6 rides EYE4 bit 0). The crescent copy gate was removed: the taper (x 605..1345) already keeps
  the crescents inside the centre fold cell at the photo's spacing and above, so it was a no-op there;
  below, every crowded copy gets its slice of crescent.
- **Fit (no feature lost).**
  - a second block-RAM delay ring (256 × 22, `no_rw_check`, read pointer write − 3 plus a fabric
    register = 5 clocks) carries the stage-4 flags to stage 9, the crescent shade bit, the generated
    avid and the output luma (−~90 FF);
  - the static ROM outputs (eye geometry, 1/pitch, the vertical lattice step, the K4 width) are read
    directly instead of through per-field copies (−~110 FF);
  - the per-field half period is a wire;
  - the glide drops its result register (ease and store in one phase, from registered operands);
  - the dash saw, crescent shading and stripe level carry 1–2-bit flags instead of 13–16-bit values;
  - the engine reuses the square root for the half-width and recomputes 2T + 1.
  - Standalone yosys vs v0.5.1: LUT +~150, FF −~260.
- Latency 24 in every mode; no pixel stages added or removed (U/V parity unchanged).

**Build** (ROUTER1: `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh vesica`, 2026-11-09):
all six on seed 1. HD Analog 83.13, HD HDMI 84.14, HD Dual 79.01, SD Analog 77.21, SD HDMI 84.88,
SD Dual 77.60 MHz. 7,373–7,420 / 7,680 LC (96.0–96.6 %; v0.5.1 96.9 %), 24 / 32 EBR (+2: delay
ring 2); no RAM transparency emulation; iceunpack OK ×6; the packed config carries the new labels and
presets. The first build (HD 75.03 / 75.17 / 77.51) had the glide's knob mux, ease and store in one
clock as its critical path (13.3 ns); restoring the registered load stage fixed it.

**Checks** (working files in `.heirloom_work/rb_vesica/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: every new knob path at RTL widths over all knob values and timing classes: no overflow;
  the defaults are exact (kh 8684, 1/pitch 923, K2 entry 57, K6 entry 28 with lens 0, fold m 79,
  d_C 15645, crescent width 256).
- Default frame (GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture, production v0.5.1 simulated the same
  way): raw 10-bit output **byte-identical**, on the first cut and again on the final source.
- One extreme frame (`r06_x.png`: P12 0 %, K4 100 %, K1 0 %, K6 100 %, Osc): a row of slit eyes; the
  crescents come in as two broad mounds and the dash band follows them; the lattice bends hard.
- Not verified: the lens etch with a real camera, K2 / K3 extremes, K5 top speed, S8 Filled at low P12.

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
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). s_gav_sr was extended to index 14. Post-route Fmax: HD Analog s1 81.28, SD Analog s1 84.67, HD HDMI s1 83.56, SD HDMI s1 83.51, HD Dual s1 86.16, SD Dual s1 82.04 MHz; 7427-7474 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
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

## Open questions for Ron (2026-11-09 batch)
1. K1 now tilts the lattice as it scales (horizontal ×1/4..×4, vertical ×1/2..×2). Would you rather
   have one plain zoom, or the vertical oscillator on its own control?
2. The lens etch above the default Etch steps in three jumps (±20 / 40 / 81 px). Smooth it (about
   +30 LC), or is stepped fine?
3. P12 below 50 % crowds the side copies into a row of slit eyes. Keep it, or hold the copies at the
   photo's spacing as in v0.5?
4. S7 Tear and S9 Texture are still On / Off. Should they become bigger look-flips (e.g. Tear: Right /
   Both edges)?

## Hardware watch list
- No renamed controls and no switch changes; the new behaviour lives at the knob ends and above the
  default Etch.
- Lens etch (K6 above ~40 %): the eye's outline follows the camera (Video) or the Osc plaid, in three
  steps of amplitude.
- P12 below 50 %: the copies crowd in (fold 948 → 420 px); watch the fold seams and the crescent garland.
- K4 near 100 %: the dash texture fills most of the frame (S9 Texture Off clears it).
- SD: K1 near 100 % (241 px diamonds) and K3 near 0 % (12 px stripes) alias at SD's half resolution.
- Delay ring 2 (block RAM) carries the luma and the layer flags: posterized or wrong colours would point
  at it. The static ROM outputs are read directly by the engine and the pixel path (timing passed).
- ROUTER1 (as v0.5). 24 EBR (+2). No pin-skew screen; if HD HDMI shows lines, re-seed that config with router1.
- Cut for fit: nothing.

## Next step
Wait for Ron's preview notes. Rebuild the diag with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh vesica`. On approval: `./heirloom_diag.sh merge vesica`,
then repack the previewed diag bitstreams under the production TOML (copy
`build/programs/ron/vesica_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
`program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`) instead of
re-routing; update the production SUMMARY / README / PHASE2 row and commit only vesica's files.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 loads at 100 % (initial_value 1023), and bend / element programs get a 25 % stronger camera at the top of the knob.
- **Bend.** K6 Etch bends the lattice bands and the stripes by the camera (or Osc) luma. Above its default it also pushes the eye's outline (the lens etch).
- **Change.** This is the production vesica change. The etch takes the luma as (L − 128) × 1.25.
  - How: `s_lq = x + x/4` is one 10-bit add on the registered luma. It feeds the stage-4 etch register and the lens case.
  - Effect: the field etch, the stripe etch and the lens push are all ×1.25 at every K6 position.
  - Why the luma and not the gain: the K6 ROM word is full here (lens 2 | exponent 3 | mantissa 3). Its top, 896, has no ×1.25 step, since the exponent is capped at 7.
  - Headroom: |xm| ≤ 1120, E ≤ 8960 (15 bit), |F2 + E| ≤ 30.5 k (16 bit), lens ≤ 1280 (12 bit). Latency unchanged.
- **Build** v0.6.2 (`k6build.sh diag vesica`), router1, all six seed 1. HD Analog 84.18, HD HDMI 82.26, HD Dual 74.72 MHz (worst HD 74.72, a thin margin). 7397–7423 / 7680 LC. `out/rev_b/ron/_diag/vesica_diag_v0.6.2.vmprog`. Not flashed. No render (diag copy).

## v0.6.3: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`N - x` as `x - N`) while the GHDL
simulator computes it correctly, so every render showed the intended picture and the hardware did not (CLAUDE.md
VHDL traps; audit in `.heirloom_work/ghdl_sub_check/`). Rewritten as `to_signed(N, w) - x` / `-x + N`:

- **line 569** (major): Lens family V (the eye and its side copies): u at the first pixel after each fold wrap, u =
  1 - per2h (negative). f_DVw = f_ddw(f_uwV) (= 2 xs u + xs^2, the X^2 DDA first difference) is computed from the
  SIGNED value at … On hardware it was: f_uwV = per2h - 1 (positive): S is still right at the wrap but D starts
  positive, so S grows as (|u0| + n xs)^2 from per2h^2 to 4 per2h^2 instead of falling to 0. dV is clamped to +511:
  no lens, rim, inner rim or halo. The central eye is the fold AFTER the first wrap (wrap at x ~540 px at the photo
  fold), so its whole …
- **line 570** (major): Crescent family C: u after each fold wrap (= 1 - per2h at 1080p and SD, = -per2h at 720p via
  unary minus); f_DCw = f_ddw(f_uwC) uses the signed value. The C family has no mid-fold reload, so this D drives
  the crescent … On hardware it was: 1080p/SD (else branch): f_uwC = per2h - 1, D positive, so the crescent circle
  seen in the central fold is X_eff = u + 2 per2h - 1, i.e. the circle centred one fold to the LEFT (x ~49 px), and
  S = X_eff^2 overflows c_SC's 22 bits just right of the fold centre (X_eff = 2048 half-px). From the first C wrap
  (x ~523 px) …
- **line 899** (major): Top crescent taper: c_tb = 2690 - hx falls 2 690 -> across the line, c_ta rises; td =
  min(ta, tb) is a tent that is 0 at the tips x 605 / 1345 px, so the crescent width min(0.866 td, w0) tapers to
  points at both ends … On hardware it was: c_tb starts at hx0 - 2690 (-2690 at HD) and decrements, so td = tb is
  hugely negative (no crescent) until the 13-bit counter wraps at x ~703 px (HD/720p); after the wrap td >= 198
  everywhere, so the crescent starts almost blunt at x ~703 px and runs at full width to the RIGHT screen edge (past
  its 1345 px tip, into …
- **line 901** (major): Bottom crescent taper, as line 899 with tips at x 600 / 1368 px. On hardware it was: Same as
  899 for the bottom crescent: nothing left of ~703 px (HD/720p; ~1184 virtual px at SD), then full width with a
  blunt start to the right screen edge; no right tip.

**Build** v0.6.3, router1: HD Analog 74.64 (builder seed; replaced by router1 s13, 79.56 MHz, whose HDMI data is <= -0.63 where seed 1 had +0.89), SD Analog 81.40, HD HDMI 80.10 (builder seed, replaced: see HD skew), SD HDMI 77.24, HD Dual 84.68 (s2), SD Dual
82.06 (s2). 7387-7407 / 7680 LC, 24 / 32 EBR; seeds other than 1: HD Dual, SD Dual. iceunpack OK. Not simulated, not
flashed.
**HD skew**: NO WINDOW PLACEMENT exists (this netlist places HSYNC low; cf. chartreuse, lotus) in r1 s1-160 + r2 s1-40 (router1 max hs +0.33; router2 39-59 MHz): HD HDMI
router1 s1 (-1.11 / <= -1.29, 80.10 MHz) -> router1 s49 (+0.33 / <= -0.94, 78.70 MHz, sub-window); spare router1 s81
(+0.10 / <= -0.83, 83.40 MHz). A full builder rebuild re-routes seed 1: re-install the seeds with
`.heirloom_work/skew_tools/install.sh` afterwards (HD HDMI router1 s49, HD Analog router1 s13). If HD HDMI shows thin
flashing lines, that is this sub-window placement: try the spare, then report it.
