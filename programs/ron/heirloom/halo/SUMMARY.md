# Halo — glowing rings in red crescents between blue hourglass columns

**v0.6.0 (2026-11-02, phase 2): built, awaiting HW.** Ron approved the diag preview (`Halo Diag`) on 2026-11-02 and it
was merged; see [Phase 2](#phase-2-v060).
v0.5.0 (2026-10-27): built, NOT flashed. Heirloom #47, rebuilt from the photo
`photos/WIN_20230801_00_19_00_Pro.jpg`. v0.1.0 was judged too abstract. This version measures the photo and
reproduces it as one contour lattice with a ring layer keyed into its cells. Presets are hand-written
placeholders.

## Phase 2 (v0.6.0)
Ron, 2026-11-02 (on v0.5.1): "broaden the range 100 % in both directions on K1, K2, K3, K5 and K6. Make P9 more of a
prominent change." Read as: each knob reaches **twice as far on each side of the photo setting** (twice the octaves
for the scale knobs, as on beacon / blossom / bunting), and "P9" = **S9 Ember** (there is no P9). Built first as a diag
copy (`.heirloom_work/rb_halo/diag/halo_diag/`); Ron approved it ("approve", 2026-11-02) and it was merged here.

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K1 Rings: size ×0.45 .. ×1.5, square law, photo at 54 % | **×0.19 .. ×2.25**, exponential (3.5 octaves of kx, piecewise-linear exp2), photo at **68 %** (694) | kx = 659 · 2^e on the per-field multiplier; f_kx is 14 bits (×4/3 at SD up to 10,502). Above ~×1.5 the rings outgrow the cells and show as arcs cut by the blue columns |
| K2 Cells: 1 .. 4 across, linear, photo 2.02 at 34 % | **0.5 .. 8 across**, exponential (4 octaves), photo 2.02 at **50 %** (515; 2.023, was 2.020) | 0.5 = half a cell fills the screen (one huge ring / hourglass); 8 = a dense lattice of 8 × 8 cells |
| K3 Pinch: 0 .. 160 %, photo at 62 % | **−100 .. 220 %**, linear, photo still at 62 % (639, bit-exact) | Below 0 the V term inverts: the cells widen at the cusp rows and the blue columns pinch into stacked lenses (they close at −100 %). Above 160 % the diamonds take over the cells and their pale cores grow |
| K5 Tear: 0 .. 200 %, photo at 50 % | **−100 .. 300 %**, linear, photo still at 50 % (bit-exact) | Negative tears the line starts the other way (sign applied with Flip at the torn-phase stage). The tear chain was widened (f_tamp 13 bits, pixel amplitude 12 bits) so 300 % × 8 cells can't overflow |
| K6 Warp: 0 .. 100 %, photo at 15 % | **−15 .. 185 %**, linear, photo still at 15 % (153, bit-exact) | Negative bends the contours and dents the rings against the source; 185 % is 2.3× v0.5's maximum |
| S9 Ember: Short (photo) / Long (the crescent decays 4× slower: subtle) | **S9 Ember: Crescent (photo) / Core** | Core: the red follows the ring field instead of the cell's left edge: full heat inside every ring (the olive D interior takes the full red, not a quarter), cooling over the ring's own scale outside it (red index = e/2, e = (E2 − 1)/512) to brown at the cell edges. The cells flip from "red crescent left, olive disc" to "red-hot disc in a yellow ring, brown surround" |

Unchanged: K4 Speed, S7 Seam, S8 Noise, S10 Flip, S11 Source, P12 Halo. The K1 and K2 displays are now 0–100 %
(their curves are exponential). Presets: Bare Rings and Straight re-aimed to zero tear (K5 256) and zero pinch
(K3 320), Lattice re-aimed; new **Lenses** (K3 0) and **Embers** (S9 Core). The defaults draw the v0.5 picture.

Build (router2, 2026-11-02, diag and production identical): HD Analog s1 79.57, SD Analog s1 81.44, HD HDMI s3 76.58 (s1 stalled the router, s2
missed), SD HDMI s1 80.76, HD Dual s1 79.71, SD Dual s1 84.45 MHz; 7,048–7,065 / 7,680 LC (92 %, +~180 over v0.5.1),
23 / 32 EBR. No pin-skew screen. No sims run; the changes are range / sign widenings plus the S9 red
source, reasoned from the code (curve values checked in integer arithmetic).

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
4. **Red.** A ROM indexed by the distance since the cell key's leading edge (Crescent), or by the ring field's
   distance outside the ring, e/2, with full heat inside it (Core; the interior then takes the full red).
5. **Output.** Luma knee (whites to about 740); cell layer or band palette; line-start flare.
6. **Shell.** Heirloom output conditioning and blanking gate, with generated avid. Latency 32 in every
   mode.

| Control | Photo element it drives |
|---|---|
| K1 Rings | The ring ellipses: size ×0.19–×2.25, exponential (the photo's 164 × 185 px at 68 %) |
| K2 Cells | The cell lattice: 0.5–8 cells across, exponential (2.02 at 50 %); rings, columns and diamonds zoom together |
| K3 Pinch | The hourglass and diamonds: −100 % = the columns pinch shut into stacked lenses, 0 = straight blue columns, 100 % = the photo (62 %), 220 % = diamonds fill the cells |
| K4 Speed | Bipolar vertical roll (deadband ±8, square law). Cusps, diamonds, rings and tear strips ride with it; the default rolls slowly down (about 0.8 line per field) |
| K5 Tear | The torn line starts: −100–300 % (100 % at 50 %); negative tears the other way |
| K6 Warp | How far the source (camera luma / Osc) bends the contour lines and dents the rings: −15–185 % (15 %); negative bends against the source |
| S7 Seam | Cut (the photo: the right family with its white noise band) / Mirror (symmetric seams) |
| S8 Noise | The white band boils (On) / a clean white contour (Off) |
| S9 Ember | Crescent (the photo: red fired at the cell's left edge, 125 px decay) / Core (red-hot inside every ring, cooling to brown away from it) |
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
  - ring kx = 659 · 2^(3.5 (1023 − K1)/1023) (piecewise-linear exp2 in the knob decode, ×659 on the
    multiplier; ×4/3 at 4:3 SD), and ky;
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
- **v0.6.0 built 2026-11-02 (phase 2), NOT flashed** (the diag preview was approved). Post-route Fmax (router2): HD Analog s1 79.57, SD Analog s1 81.44, HD HDMI s3 76.58 (s1 stalled, s2 missed; the builder finds s3 on its own), SD HDMI s1 80.76, HD Dual s1 79.71, SD Dual s1 84.45 MHz; 7,048–7,065 / 7,680 LC (92 %), 23 / 32 EBR.
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 77.19, SD Analog s1 77.91, HD HDMI s1 80.10, SD HDMI s1 77.65, HD Dual s1 87.24, SD Dual s1 81.97 MHz; 6860-6889 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
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
