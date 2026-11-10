# Vine — blue vines with magenta blossoms swaying down a banded green ground

**v0.1.0 built 2026-11-10, not flashed.** Heirloom (video phase), from the clip `WIN_20231026_23_13_39` at 0.5 s
(`photos/vid/WIN_20231026_23_13_39_t0p5.png`).

- **The still.** Four blue vines, about 134 px wide, sway down a pure green ground. A vertical sine bands the ground
  with black (rows ~20, 520 and 1040), and it darkens toward the right.
- **The vines' detail.**
  - Soft violet and magenta blossoms.
  - A dark ragged rim with short black flame spikes.
  - Pale lavender tongues reaching out of the left sides.
  - Small yellow-green, cyan and white leaves floating beside them.
- **The patch.** The sway grows from left to right: a horizontal oscillator, reset near the line start, whose
  frequency a vertical sine sweeps (FM).
- **The motion.** In the clip the sway wave scrolls down steadily, one wave every 8.45 s. The ground stays still and
  the blossoms flow and morph.

Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_vine/`: `notes.md`, `fitF2.py`, `motionP.py`, `anaP2.py`, `bg6.py`, `edgeu.py`,
`rag.py`, `leaves.py`, `vinecol.py`, `proto.py`, `imodel.py`, `sheet.py`, `rtlcmp2.py`.
- **Vines (FM).** Strand centres fit φ = (x + 160) F(y) = k + 0.688, with F(y) = F0 (1 + m sin(2π y / 1125 + θ)).
  - Constants: F0 = 1 / 556.6 px, m = 0.130, θ = 2.588 rad at the still.
  - The fit's rms is 14.7 px over all strand-centre points of 1055 rows. The period-modulated form fits 13.8 px.
  - The fitted origin, 160 px left of the active start, is about where HD's hsync pulse ends (148 px back porch).
  - The wavelength, 1125 lines, is one whole frame including blanking: a vertical oscillator running just off the
    frame rate.
- **Width and edges.**
  - The blue spans 0.242 cycle: 134 px median, 106..176 px for 10..90 %. It is centred on the fitted phase.
  - The edge position has an sd of 0.043 cycle (~26 px, including fit error). Its vertical correlation length is about
    4 rows: 0.72 / 0.51 / 0.26 / 0.05 at 1 / 2 / 4 / 8 rows.
  - The left edges throw longer tongues (1 % tail 68 px) than the right edges (42 px).
  - A dark blue to black rim, 4..8 px wide, sits between the blue and the green.
- **Ground.** Pure green (R = B ≈ 0).
  - Vertical profile: G = 144.5 + 125.9 cos(2π (y − 257) / 524) − 21.3 cos(4π (y − 257) / 524), rms 2.3 levels.
  - Minus a horizontal parabola of 116 (x / 1900)² levels: 0 at the left, ~115 at x 1850.
- **Palette** (k = 14, whole still):
  - black (0,3,3) 18.5 %
  - blue (21,1,245) 17.4 %
  - greens (1,248,0) 9.9 % and (1,219,0) 6.9 %, plus four darker greens
  - violet (139,5,251) 5.8 %
  - dark blue (11,10,126) 4.0 %
  - lavender (137,163,214) 1.0 %
  - teal (46,117,104) 0.9 %
- **Blossoms.** The vine cores are B 230..255 and G ≈ 0; R is the blossom level.
  - R: median 20, 75 % 70, 90 % 134, 99 % 251 (blue → violet → magenta, with a pink-white tip).
  - Texture autocorrelation: 0.35 at 32 px across, 0.45 at 16 rows down.
- **Leaves.** About 220 small streaks, 18..42 px wide and 8..23 px tall (aspect 2..5).
  - Colours: yellow-green (173,220,134), cyan (78,222,196), pale lavender (154,197,229) and pink (222,139,198).
  - Most sit 0.2..0.3 cycle from a vine centre, more on the left side, as tongue tips.
- **Model vs still at the defaults** (integer model, 1080p):

  | Measure | Model | Still |
  |---|---|---|
  | green | 48.4 % | 48.3 % |
  | black | 22.6 % | 22.1 % |
  | blue | 17.3 % | 16.9 % |
  | violet | 6.2 % | 6.3 % |
  | magenta | 1.5 % | 1.4 % |
  | dark-blue rim | 1.7 % | 2.8 % |
  | leaves and tongues | 2.3 % | 2.3 % |
  | vine-core R quantiles | 1, 25, 77, 138, 181, 244 | 0, 25, 76, 137, 176, 246 |
  | strand spacing, mean | 561.5 px | 564.1 px |
  | strand spacing, range | 482..666 px | 482..653 px |

  The 4 × 4 region means agree within 1.5..11 RGB levels, except the top-right cell (15.5), where the still has a leaf
  cluster.

## Motion in the clip
Measured on all 337 unique frames of the 11.3 s clip (29.92 fps), with `motionP.py` and `anaP2.py`.
- **The sway scrolls down steadily.**
  - θ falls 0.743 rad/s: 0.118 Hz, one wave per 8.45 s, 133 lines/s, 2.2 lines per 60 Hz field.
  - The spacing and depth stay fixed for the whole clip: P0 561 ± 2 px, amplitude 72 ± 3 px. No knob moved, and no
    other look appears in the clip.
- **The ground is still.** The dark band's centroid sits at row 519 ± 1.5 for 11 s.
- **The blossoms flow** at 60..120 px/s, mostly down along the vines, and decorrelate to ~0.45 within 0.5 s. The leaves
  flicker and drift with them.
- **In the program:**
  - The K4 default (700) gives exactly the clip's scroll: dθ = 33032 / 2^24 cycle per field, which is 0.118 cycle/s
    at 60 fields/s, downward.
  - The blossom noise scrolls down one virtual line per field (60 lines/s). The flame noise scrolls about 165 lines/s
    at the default Flame, and faster with P12. So the texture morphs as it flows while the ground stays put.
  - `cmp_motion.png` shows the clip at t, t + 0.5 s and t + 1.0 s beside the model at fields 0, 30 and 60.

## What it does
1. **Phase.**
   - Per line, F = F0 (1 + m sw(ψ)), where sw is the sine (Zigzag: a triangle) of the sway phase ψ = θ + y / L.
   - The phase starts at (x0 + 160 virtual px) F and steps F per pixel. Etch adds the camera's luma.
   - Under Mirror, the phase DDA runs back from the centre pixel.
2. **Noise.** Two value-noise octaves:
   - blossom noise n1, with 44 × 28 virtual px cells;
   - flame noise n2, with 28 × 9 cells.

   Each octave comes from two xorshift16 lattice-row streams, seeded by a hash of the row index. Rows are blended with
   a 4-bit smoothstep weight and columns with a 4-bit linear weight. The rows scroll down every field.
3. **Vine key.** u is the distance from the nearest vine centre.
   - key = (W − |u|) · 3.125 (left side) or · 4.5 (right side), + 6.25 (n1 − 128) + G2 (n2 − 128).
   - Vine where key > 0.
   - Rim (dark blue → black, 16 levels) where −256 < key ≤ 0.
4. **Flames.** An extension key pushes past 0 where the flame noise peaks. It has a gentler slope (2.5, with the
   right side's distance × 1.75) plus a spike term (n2 − r0) · G3.
   - Near the vine (within 0.19 cycle), where the blossom noise is low, it draws dark spikes.
   - Farther out, or where the blossom noise is bright, it draws pale tongues (violet → lavender → white → cyan →
     yellow-green).
5. **Leaves.** Flame-noise peaks, gated by the blossom noise, in a band about 0.32 cycle beside the vines.
6. **Colours.**
   - Vine colour comes from the still's 64 quantile steps of the vine cores. It is indexed through the model's CDF of
     n1 + (n2 − 128) / 4.
   - The ground is the green level (profile minus parabola), or Night's inverted profile.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Vines | the vines' count and spacing: F0 × 0.35 (one giant vine) .. the clip (512, ~3.5 vines across) .. × 2.83 (~10 vines); exponential, glided |
| K2 Sway | the sway's depth: 0 = straight vines .. m 0.13 (512, the clip) .. 0.75 (1023): deep folding arcs that crowd on the right |
| K3 Wave | sway waves down the frame: × 0.25 (one gentle bow) .. the clip (512, one wave per frame) .. × 4 (tight zigzags); glided |
| K4 Speed | the sway's scroll, bipolar (±8 deadband, square law): 700 = the clip (one wave per 8.45 s, down); 1023 ≈ 0.9 waves/s down; below 512 it scrolls up. At the centre the sway stands still while the blossoms and flames keep flowing |
| K5 Girth | the vines' width: 0.02 cycle (hairline stems) .. 0.121 (512, the clip) .. 0.40 (a blue wall with green slits) |
| K6 Etch | how far the camera (or the Osc lattice, at 8× the camera's gain) bends the vines and swells their girth: default 256 = a 0.12-cycle bend for full white; 1023 = 0.48 cycle |
| S7 Zigzag | sine sway → triangle sway at 2.5× depth: sharp chevrons |
| S8 Night | the ground inverted: black, with green glow bands where the clip has its black bands |
| S9 Mirror | the vines reflected about the centre: the sway mirrored, with two vines crossing in an X in the middle |
| S10 Palette | Vine (the clip) / Ember: orange-red vines with yellow blossoms on a deep blue banded ground, cyan leaves |
| S11 Source | Video (default) / Osc |
| P12 Flame | the flames, tongues and leaves: 0 = calm smooth vines (no tongues, no leaves); 409 = the clip; 100 % = burning: rougher edges, spikes reaching 3× farther from a lower onset, a swarm of pale tongues and leaves, faster flicker |

Glides (1/8 of the gap per field): K1, K3, P12. K2, K5 and K6 are read per field. K4 has a ±8 deadband.

## Video
- **Video (default).** The camera's luma, (Y >> 2) − 16 from every other pixel through a one-line buffer, is added to
  the vine phase, and 5/16 of that to the vine half-width.
  - The vines bend around the picture and swell where it is bright; their flames and leaves follow.
  - At the default Etch (256) the four vines stay clearly readable on a face (`rtlF_cam.png`).
  - At Etch 1023 the picture's shapes take over.
  - With no input, Video mode is the still.
- **Osc.** The shell's drifting 2-D triangle lattice (640 × 540 virtual px), written signed about its mean, feeds the
  same path at 8× the camera's gain.
  - At the default Etch it throws the vines into sideways kinks and swollen lobes: an unmistakable flip.
  - At Etch 1023 it makes a lattice of blue diamonds.
  - Etch stays live in both modes.

## Engine
- **Sequencer** (vblank zone, "no avid for 1024 clocks"). It has one serial shift-add multiplier with 9 slots, a
  256-entry exp2 ROM, and a shared glide for K1, K3 and P12. Per field it computes:
  - F0 (Vines);
  - the sway depth m and F0 m (Sway, with Zigzag's 2.5×);
  - the sway phase per line (Wave);
  - speed² and the scroll step;
  - the half-width (Girth);
  - the flame level and its gains, onsets and leaf band (P12);
  - the noise scrolls and the per-mode steps.
- **Line engine.** It is kicked at each line start and computes the NEXT line in ~165 clocks, so it fits SD's
  858-clock line. It runs on its own serial multiplier, with a signed multiplicand, and computes:
  - the sway sine (256 × 16 quarter ROM) or triangle;
  - F = F0 + F0 m sw;
  - the phase start, (x0 + 160) F + C0, and the step F · xs;
  - per noise octave: the two row hashes ((j xor salt) · 40503, then xorshift-add mixing), the first three lattice
    values of each row stream, and the column values C0..C2 = Lt + (Lb − Lt) wy;
  - the ground level, from a 1024 × 8 profile ROM with normal and Night halves.
- **Beam.**
  - The phase DDA, reversed at the centre under Mirror.
  - Per octave, a cell accumulator. At each cell crossing, both row streams step and the column values shift, and a
    three-clock pipeline forms the next column value.
- **Pixel path (21 stages).**
  - Stages 2–4: column interpolation and the noises; the etch product; the etched phase and half-width.
  - Stages 5–8: |u| and side; the gains; the slope term; the vine key and the side-weighted distance.
  - Stages 9–12: vine / rim; the extension key; the leaf level; the ground (level minus the x-darkening ROM); the
    classes and colour coordinates.
  - Stage 13: palette address. Stage 14: address register. Stage 15: palette ROM (EBR). Stage 16: fabric copy.
    Stage 17: compose.
- **Output.**
  - U and V go through [1 2 2 2 2 2 2 2 1]/16: two adjacent 8-sums, with the history in a 256 × 20 block-RAM ring
    (`no_rw_check`).
  - Luma goes through [1 2 1]/4 and 13/16, then a vertical [1 1]/2. The previous line sits in a 2048 × 8 block RAM,
    read two pixels ahead of the write. So no drawn feature reaches the screen as a hard single row.
  - Then the blanking gate.
  - The running sums snap back to neutral after 8 blank samples (the family's resync).
- **Contracts:**
  - generated avid (parity-locked start);
  - one 20-clock latency in every mode;
  - the blanking gate;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr);
  - luma pre-scaled 16/13 and capped at 940 (drawn whites ~777).
- **ROMs and RAMs (21 EBR).**
  - Palette, 1024 × 32 (8 EBR): 2 palette sets × 4 regions (ground / vine / tongue + leaf / rim + spike) × 128 levels.
  - Sine; exp2; the ground profile (2 EBR); the x-darkening ROM.
  - The camera line buffer (1024 × 8); the chroma ring (256 × 20); the vertical-average line (2048 × 8).
- **Verified (GHDL, 720p60 decimation 1, real timing ID).**
  - Video with a black input: bit-exact with `imodel.py` on all 1278 × 719 compared pixels, for Y, U and V. As in the
    family, the RTL's picture sits 5 px (luma) / 9 px (chroma) right of the model's centred filters.
  - K4 = 1023 after two fields: bit-exact, once the model applies the speed with its one-field latency (the first
    zone still uses the default step).
  - Osc: 96 / 94 / 93 % (Y / U / V). The rest is the model's one-line approximation of the lattice.
- **Fit history.**
  - The first cut used staggered lattice rows (four streams, six per-pixel multipliers) and a Braid switch (a second
    vine set crossing the first): 9151 LC (119 %).
  - Cut to fit:
    - the noise went to aligned rows, with the vertical blend done once per cell;
    - the noise weights dropped to 4 bits;
    - Braid was replaced by Mirror;
    - the gain and etch multipliers were narrowed;
    - the ground parabola became a ROM;
    - the field sequencer lost four slots to shift-adds;
    - the shading next to the vines was dropped;
    - a few constant factors were rounded (6.375 → 6.25, 2.625 → 2.5, 1.8125 → 1.75).
  - Result: 7077 LC.

## Status
- **v0.1.0 (2026-11-10), router2.** Build command: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh vine`.

  | Config | Seed | Fmax (post-route) |
  |---|---|---|
  | HD Analog | 1 | 79.41 MHz |
  | SD Analog | 1 | 80.25 MHz |
  | HD HDMI | 1 | 81.95 MHz |
  | SD HDMI | 1 | 76.32 MHz |
  | HD Dual | **2** (seed 1 missed) | 85.03 MHz |
  | SD Dual | 1 | 81.47 MHz |

  - 7077–7089 / 7680 LC (92 %), 21 / 32 EBR.
  - Every bitstream passed iceunpack, and the packed config carries the labels.
- `dupin.py`: 0 `CARRY I0==I1`.
- **Single-row check.**
  - True single-row features in the drawing at the defaults (≥ 24 px, before the vertical filter): 0 / 5 runs at 720p
    (fields 0 / 40) and 3 / 5 at 1080p. They are 1-row rim slivers on horizontal lobe edges and 1-row tongue tips.
  - After the vertical [1 1]/2, each is a 2-row band at half contrast.
  - A velvet-style threshold count on the final output (dark rows with Y < 300 over non-dark neighbours) gives 0 / 1 at
    720p and 2 / 0 at 1080p, all ≤ 30 px.
  - At P12 0 the count is 0 everywhere.
- Output: `out/rev_b/ron/heirloom/vine.vmprog`. **Not flashed.**

## Deviations
- **Edges are cleaner than the still's.** The still's camera texture gives finer serrations and longer, sharper black
  spikes. The program's flame noise is a 28 × 9 virtual-px lattice, so its spikes are a little taller and rounder.
  Occasional green holes open inside a vine where both noises dip together.
- **Leaves are smaller.** They are single flame-noise peaks, about 20..40 px. The still's largest leaves (60..130 px
  yellow-green streaks at the top right) are not reached.
- **Blossoms** follow the still's colour distribution, but their blobs come from a noise lattice: smaller and a little
  horizontal, rather than the still's large rounded patches.
- **The rim is thinner:** 1.7 % of pixels against the still's 2.8 %. The shading of the green next to the vines was cut
  for budget.
- **Whites** come out at the shell's ~777 luma cap, so the pale tongue tips read slightly grey.
- **Cut for budget:** the staggered lattice (row artifacts are faint in the blossoms), the Braid switch (replaced by
  Mirror), the green shading beside the vines, and 6-bit noise weights (now 4-bit).
- **Resolutions.**
  - 720p shows the same picture as 1080p.
  - SD shows the centre 4:3 (virtual x 240..1680), so about half a vine is cropped at each side.
  - SD's noise is a different realisation, because its lattice restarts at the SD line start.
  - At SD a flame-noise row is about 2 field lines.
- **Edge artifacts.** The left edge fades in over ~9 px (filter warm-up). The first line of a field skips the vertical
  average.
