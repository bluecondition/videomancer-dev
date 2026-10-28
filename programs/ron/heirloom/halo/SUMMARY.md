# Halo — glowing rings in red crescents between blue hourglass columns

**v0.5.0 (2026-10-27): built, NOT flashed.** Heirloom #47, rebuilt from the photo
`photos/WIN_20230801_00_19_00_Pro.jpg`. v0.1.0 was judged too abstract. This version measures the photo and
reproduces it as one contour lattice with a ring layer keyed into its cells. Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_halo/` (notes.txt, track.py / sv_fit3.py / sfull.py / vfull.py,
bpal3.py, glowfit.py, proto4.py, imodel.py).
- **Lattice.** 2.02 cells across: the bands repeat every 949 px and the rings every 942 px. The vertical
  period is 542 lines, with the ring rows at y 511 (and −31 / 1053) and the cusp rows at y 255 / 780. The
  picture mirrors top-to-bottom about y 511 (flip correlation 0.99).
- **Blue columns = one contour field.** G = S(w) + V(|dy|).
  - S(w) = 304 − 1.272 w + 0.000979 w² − 5.7e-7 w³, with w = px from the seam at x 994.5. This is a
    curved triangle: slope 1.27 at the seam, 0 at the cell centre.
  - V ≈ 0.0021·dy² (tabulated), 152 at the cusp.
  - Fitted from 132 tracked contour lines, residual 0.85 px. The concave diamonds at the 4-way points are
    the top of G.
- **Two colourizer families.** Left of the seam, and right of a cut 22 px past it. The right family holds
  the white noisy band at G ≈ 205–255, with its blobs stacked down the lines. The palettes are binned
  means over G, sharpened.
  - Shares: blue (8,64,235) 15 %, teal (10,74,93) 11 %, olive (38,56,2) 10 %, light blue 9 %, white 7.5 %,
    orange-red (159,55,2) 6 %.
  - Region means on a 4×4 grid land within about 10/255 per channel.
- **Cells.** G < 58; the teal boundary line belongs to the band palette.
  - **Ring.** A true ellipse (superellipse exponent 1.9–2.0), semi-axes 164 × 185 px, centred 71 px right
    of the cell centre.
  - **Glow.** A white core (blue σ ≈ 10 px) inside a yellow-green glow. The glow reaches 40/25 px inside
    and outside on the left, and 56/50 px on the right.
  - **Interior.** Dark olive (38,56,2), D-shaped, with a jagged ±2 px right edge (a 9-line sawtooth plus
    noise).
  - **Red crescent.** R ≈ 175 is fired at the cell's left (key) edge and decays as exp(−x/125 px) to about
    36 with G 56 and B 0, so it runs orange-red into brown.
- **Tears.** Strips symmetric about each cusp row.
  - Damped-cosine line-start phase errors (−150 px, overshooting to +28 px near x 430) at |dy| 138–175 /
    354–397 and in the centre strip (228–289).
  - Exponential stretches (60–115 px over 45 px) in between.
  - The first 58 px of every line has a pale-cyan flare: olive in the shift strips, blue in the centre
    strip.

## What it does
1. **H phase.** A 24-bit H phase is anchored at the screen centre, with Cells across the measured width.
   Flip mirrors it. A line-start tear adds shape(x) × amp(line), where the shape comes from a damped-cosine
   / decay ROM and the strip type and amplitude are chosen by the line's distance from the cusp.
2. **Band field.** G = S[fold] + V[|dy|]·Pinch + warp·source.
   - Right of the cut (and below the diamonds) the white band's per-line value noise is added, fading over
     about 130 px.
   - The palette address is side(1) & (G − 32): two 512-entry Y/U/V colourizers.
3. **Ring field.** E2 = (|dx|·kx)² (square ROM) + dy² (per line) + warp. The glow index is
   t = (E2 − 1)·s·(1 − 0.42·dx), where s = 1/Halo.
   - The glow ROM (512 × 32) holds the cell colour by t with the red level factored out:
     colour = T(t) + red·(1 − glow)·k.
   - The interior key is t < threshold(Halo) + per-line jitter. Because it shares the glow's skew, the
     interior is D-shaped.
4. **Red one-shot.** A ROM indexed by the distance since the cell key's leading edge (Ember ×4 lengthens
   it).
5. **Output.** Luma knee (whites to about 740); cell layer or band palette; line-start flare.
6. **Shell.** Heirloom output conditioning and blanking gate, with generated avid. Latency 32 in every
   mode.

| Control | Photo element it drives |
|---|---|
| K1 Rings | The ring ellipses: size 45–150 %, square law (100 % = 164 × 185 px, at 54 %) |
| K2 Cells | The cell lattice: 1–4 cells across (2.02 at 34 %); rings, columns and diamonds zoom together |
| K3 Pinch | The hourglass and diamonds: 0 = straight blue columns with no diamonds, 160 % = deep diamonds (100 % at 62 %) |
| K4 Speed | Bipolar vertical roll (deadband ±8, square law). Cusps, diamonds, rings and tear strips ride with it; the default rolls slowly down (about 0.8 line per field) |
| K5 Tear | The torn line starts: 0–200 % (100 % at 50 %) |
| K6 Warp | How far the source (camera luma / Osc) bends the contour lines and dents the rings (15 %) |
| S7 Seam | Cut (the photo: the right family with its white noise band) / Mirror (symmetric seams) |
| S8 Noise | The white band boils (On) / a clean white contour (Off) |
| S9 Ember | Red crescent Short (the photo, 125 px) / Long (500 px: the cells glow ember-red) |
| S10 Flip | The lattice mirrored left-right (tears and flare stay at the line start) |
| S11 Source | Video / Osc |
| P12 HALO | The ring glow, ×0.3 to ×4 (exponential, photo at 46 %): 0 = bare white ellipse outlines on olive; 100 % = the interior is gone and every cell is a blazing yellow-white disc |

## Video
- **Video mode.** The camera luma, (Y − 512)/4, is added to the band field, so the contours bend around
  the picture and carry its texture (about ±10 px at the default Warp). It is also added to the ring field,
  so the rings dent.
- **Osc mode.** Two slowly drifting triangle oscillators (1150 px across, 700 lines down) feed the same
  path. Warp stays live in both modes.

## Engine
- **Per field.** The vblank sequencer runs one serial shift-add multiplier, slot per product:
  - H increment and the centre-anchored start phase;
  - V increment, and the start phase plus roll;
  - the tear factor;
  - ring kx = 988 + 145·(1023 − K1)²/2^16 (×4/3 at 4:3 SD), and ky;
  - Speed².
  The glides cover K1, K2, K3, K5, K6 and P12. Halo reads a 256-entry EBR table: s = 256/halo, plus the
  interior threshold.
- **Per line.** An engine is kicked at g_start and computes the next line in 154 clocks on one serial
  multiplier:
  - V phase and fold, the V ROM, V·Pinch;
  - |dy|·ky and dy²;
  - the tear strip type and amplitude;
  - the key jitter scaled by s;
  - the white band's noise lattice row (two ROM reads and a lerp).
- **Pixel path.** 22 stages plus 8 of conditioning plus 2 pad = 32.
- **ROMs (23 EBR).**

  | ROM | EBR |
  |---|---|
  | Palette 2 × 512 × 24 | 6 |
  | S | 2 |
  | dx² | 2 |
  | Glow 512 × 32 | 4 |
  | Tear | 1 |
  | Red | 1 |
  | Noise | 1 |
  | V | 1 |
  | P12 | 2 |
  | Chroma-window and sync rings | 2 |

- **Contracts.**
  - Generated avid.
  - One 32-clock latency in every mode.
  - [1 2 1] luma, [1 2 2 2 2 2 2 2 1]/16 chroma and 13/16 luma before the gate.
  - Palettes in the hardware convention (U = Cr).
  - Virtual 1080-line geometry. At SD the virtual width is 1440, so the rings stay round.
- **Fit history.**
  - The first RTL was 10.6 k LC (137 %).
  - Slimmed to about 6.85 k by:
    - precomputing the glow colour per glow index;
    - deriving the interior key from the glow index;
    - narrow multipliers;
    - moving the P12 table and the sync / chroma rings to EBR;
    - x counters instead of delay lines;
    - per-line instead of 2-D white-band noise;
    - running the ring curve on the serial multiplier (no divider).
- **Verification.**
  - The RTL matches the integer model `imodel.py`: mean |diff| 0.01 at the 1080p ID (960×540 raster) and
    0.06 at 720p60.
  - That check found three bugs, all fixed: the blue core term overflowed (lime rings), the tear product
    overflowed, and the noise row was latched from the previous line.

## Status
- v0.5.0 built 2026-10-27 with router2. All 6 configs on seed 1; every bitstream passes iceunpack.

| Config | Fmax (MHz) | Fmin (MHz) |
|---|---|---|
| HD Analog | 80.48 | 74.25 |
| HD HDMI | 76.37 | 74.25 |
| HD Dual | 81.59 | 74.25 |
| SD Analog | 73.68 | 27 |
| SD HDMI | 76.93 | 27 |
| SD Dual | 87.84 | 27 |

- 6,849–6,862 / 7,680 LC (89 %), 23 / 32 EBR.
- NOT flashed.
- HD HDMI has the thinnest margin. If a later edit misses, try `NEXTPNR_ROUTER="--router router1"` before
  restructuring.

## Deviations from the photo
- **Diamond core.** The off-centre heart and oval at each diamond's core are drawn as a small symmetric
  diamond core (the left palette mirrored).
- **White band noise.** It is per-line value noise, so its blobs stack down the lines. In the photo they
  also vary a little across the band.
- **Tears.** Modelled as two line-start phase-error shapes chosen by cusp distance. The compressed "<<<"
  chevron strip at the cusp row is approximated.
- **Line smear.** The analog trailing smear on thin lines (a sharp leading edge and a ~6 px tail) is not
  modelled. The lines come from the palette means, so they are slightly symmetric.
- **Periods.** Rings and bands share one horizontal period (949). V is symmetric about the cusp; in the
  photo the diamonds are slightly taller below.
- **Brightness.** The white glow peaks are soft-kneed to about 740 output luma (the drawn-luma cap). Blue is
  about 8 % low overall.
- **4:3 SD.** The rings stay round, but the blue columns are squeezed to ¾ width because the cells across
  are count-based. 720p and 1080p are identical.
