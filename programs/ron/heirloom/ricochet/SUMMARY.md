# Ricochet — Heirloom #30

**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230515_23_08_59_Pro.jpg`. Work files: `.heirloom_work/rb_ricochet/` (`notes.md` has the full
measurement log; `cmp_final.png` = photo | float proto | RTL Osc | RTL Video).

## Reading of the photo
One scalar field, coloured by a 1-D colouriser. Every number below was measured and fitted (`fit*.py`,
`hfit_hw3.py`, `fitj2.py`); the fitted constants live in `ricochet.py`.
- **Colour path.** Every pixel lies on one path (k-means, 12 centres, median 4 RGB off it): red (224,14,3) →
  orange (255,75,1) → salmon (244,120,109) → pink (232,130,145) → lavender (182,147,204) → periwinkle
  (150,157,224) → cyan (24,196,237). The scalar *s* is the arc position along it.
- **Two chevron systems.** Cores on lines 256.0 and 779.0 (flip-correlation), seams on 517.5 and −5.5: a vertical
  triangle with a period of 523 lines (2.065 per frame).
- **Field.** Fitted first by palette purity, then by joint s-error (photo units):
  - `phiA = softmax(−56.3 + 0.411 x − 4.32e−5 x², −35.1) + Y (1.347 − 3.73e−4 x)`. This is a horizontal ramp
    with a soft start (the bands bend horizontal over the first ~80 px) and a slightly falling slope, plus the
    triangle `Y = |y − core|` through a gain that falls across the line (an x·Y product term).
  - `phiB = 332.2 − 0.102 x + 3.977 (261.5 − Y)`: the same triangle, inverted and 4× steeper.
  - `phi = min(phiA, phiB)`. The chevron arms kink into the seam stripes along the crease where the two meet, and
    the flat ground is darkest exactly on that crease, which is what identifies the min.
  - Vertex rounding is only ~4 lines (measured by tracking one contour), so the tips are sharp V's. A
    purity-only fit wanted 22, which renders U-shaped tips.
- **Colouriser** (fitted to the measured LUT, error 0.00033):
  - 8.34 bands between the core (phi 103) and the reach (397 beyond it). A 0.28 chirp takes the band period
    from 60 to 42.
  - The bands ride a base that rises from orange to cyan (0.318 + 0.599 w), amplitude 0.198.
  - The red core is a steep wall with a red → orange shoulder `0.06 + 0.005 (d + 27)`.
  - At the reach a steep cut (0.029 per phi unit) drops to the flat ground (s = 0.205), which then darkens
    slowly (−0.00046 per phi unit).
- **Bottom flyback.** From line 1032 the triangle keeps going at 1.2× instead of folding at the bottom seam. That
  gives the red strip at the bottom right and the fine stripes at the bottom left (error 0.113 → 0.006 on
  those rows).
- **No textures.** No tearing, comb or noise. The 9-line seam stripes keep full amplitude (vertical blur under
  1.5 lines); the softness is the sinusoidal colouriser itself.

## What it does
1. **Per field.** A vblank-zone sequencer (one serial multiplier on fixed slots, one serial divider, one shared
   glide; one small operation per zone clock) computes, from the glided knobs:
   - the vertical phase step and the flyback step;
   - the core level P0, the reach D, 2^21/D and the reach end;
   - the band frequency, the integer band count for the chirp, the seam gain c and the etch gain;
   - the band-flow step.
2. **Per line.** An engine kicked at each line start computes the next line on one signed serial multiplier:
   - the triangle Y from a 20-bit phase, plus the bottom flyback;
   - the DDA starts `Y·gA0`, `Y·gA1·xs` and `gB0 + c·(4096 − Y)`.
3. **Per pixel:**
   - **Fields.** The F ramp is a 2nd-order DDA with a quadratic knee. The Y-gain and seam-field DDAs run 2 and 3
     pixels late, so they meet the pipe at the stage that uses them. Twin holds and reverses all three at the
     centre. The field is `phi = min(F + YA, B) + etch·luma`.
   - **Band phase.** `(phi − P0)·Nb/Ddef + chirp ROM·Nb − t`, as 3-bit row partial products summed over two
     stages, into a full-wave cosine ROM.
   - **Colour.** `s = clip(max(min(base − a0·cos, limit), shoulder))` into the palette ROM along the photo's
     path. The side values {base, limit, shoulder} ride a 2-EBR ring.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Chevrons | the two chevron systems | Vertical triangle frequency at fixed amplitude: 1–6 systems per frame height (2.07 at the default 473). More systems give flatter arms. |
| K2 Core | the red teardrops | Core level P0 = 20–260 (103 = the photo): small cores ↔ big red cores pushing the bands right. |
| K3 Bands | the bands in each chevron | 2–24 bands across the photo's reach (8.3 = the photo). |
| K4 Seams | the stripe bands at the seams | Seam slope c from 13.7 (thin dense seams) to 0.75 (seams take over the ground); 3.98 = the photo at 512. |
| K5 Speed | the bands | Bipolar band flow, ±8 still. Right streams the rings out of the cores toward the vertex (default 640 ≈ 0.42 bands/s). |
| K6 Etch | the whole band field | Picture luma pushed into the field (up to ~19 bands); moderate at the default 200. |
| S7 Point | chevron direction | Right (photo) / Left. |
| S8 Twin | the chevrons | Off / On: chevrons fired from both sides, meeting in a bowtie of flat ground. |
| S9 Ground | the flat orange ground | Flat (photo) / Echo: the chevron block repeats past the reach. |
| S10 Palette | the colours | Photo / Azure: chroma mirrored (azure ground, teal cores, orange outer chevrons). |
| S11 Source | — | Video / Osc. |
| **P12 Reach** | the outermost chevron / where the ground begins | 0 % = red cores with one or two rings on flat orange. Default 490 = the photo (vertex at 83 % of the width). 100 % = no ground: chevrons and seam stripes over the whole frame, the outer area glowing cyan. Glided 1/8 per field. |

## Video
The photo is fully synthetic, so the camera is woven into the patch rather than shown. Its luma (stored as
`(y >> 2) − 16` in an 8-bit line buffer) is added to the field before the colouriser:
- the band contours bend around the picture;
- bright areas push the reach outward, so silhouettes appear along the ground boundary.

At the default Etch a full-white area shifts the bands by about half a band, and the composition stays intact.
Osc mode feeds the shell's drifting triangle lattice through the same path (the bands wobble gently), so Etch
stays live.

## Engine
- **ROMs.** `ricochet.py` → `ricochet_pkg.vhd`:
  - chirp, 512×12;
  - full-wave cosine, 1024×9;
  - knee square, 128×8 (LUT logic);
  - palette, 256×32. Y is pre-scaled 16/13 for the output luma scale; Azure is a chroma mirror applied at the
    output.
  - DDA constants: a 9-entry table, 3 width classes × Right / Left / Twin.
- **Units.** 4096 hw = the triangle amplitude (261.5 photo units). The vertical phase is Q20; the DDAs are Q12,
  with F's differences in Q20.
- **Geometry.** Virtual 1920×1080 from the timing ID: an x step per class (1 / 1.5 / 2.667), the y step from the
  shell table.
- **EBR (13).** 4 luma line buffer, 2 side-value ring (`no_rw_check`; read 1 behind write), 3 cosine,
  2 chirp/table, 2 palette.
- **Contracts.**
  - generated avid (parity-locked);
  - one 28-clock latency in every mode;
  - chroma 9-tap, luma [1 2 1] and luma 13/16 before the blanking gate on the latency-aligned generated avid;
  - the line engine is restarted from the vblank zone every field;
  - vsync is used only through the shell's idle-zone detector.

## Status
- v0.1.0 (2026-10-20): abstract four-colour chevrons, judged not like the photo.
- v0.5.0 (2026-10-21): rebuilt by the REBUILD.md method. **Built with ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh ricochet`); all six configs on seed 1, post-route:

  | Config | Fmax |
  |---|---|
  | HD Analog | 86.33 MHz |
  | HD HDMI | 84.43 MHz |
  | HD Dual | 83.81 MHz |
  | SD Analog | 78.78 MHz |
  | SD HDMI | 82.77 MHz |
  | SD Dual | 86.56 MHz |

  - 6840–6866 / 7680 LC (89 %), 13 / 32 EBR, iceunpack OK ×6.
  - router2 does not close this netlist (69.5 MHz on HD Analog).
  - **Not flashed.**
- RTL verification (720p60, decimation 1): every per-line value is bit-exact with the integer model (`imodel.py`).
  Osc mode at default knobs matches it to a mean of 4.3 RGB. The bands flow outward between captures 6 fields
  apart.

## Deviations from the photo
- The capture borders (black top rows, dark left edge) are not drawn.
- The bands sit within a few px of the photo's, not exactly on them (mean abs RGB error 10.7 for the float proto,
  11.1 for the integer model). The residual is sub-band phase.
- The colouriser is one smooth function, so the photo's slight band-to-band colour unevenness is not reproduced.
- Three small simplifications, all invisible at a glance:
  - vertex tips are drawn sharp (the ~4-line rounding is folded into the ramp offset);
  - the band amplitude is constant (the photo's falls by ~0.04 across the reach);
  - the chirp is scaled by the integer band count.
- Knobs start at the TOML defaults and glide, so after power-up the picture eases to the real knob positions
  within about a second.
- **SD:** x is mapped to the full width, so on 4:3 the chevrons are 25 % narrower (steeper arms). The seam stripes
  (~11 lines at 1080) are only ~2.5 lines at 480i and will alias. 720p matches 1080.
