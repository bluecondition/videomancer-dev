# Starburst — Heirloom #3, a mirror kaleidoscope of bowties and CMYK chevrons

**v0.5.0 (2026-10-21): rebuilt from the photo (`photos/WIN_20221122_23_37_36_Pro.jpg`), built, NOT
flashed.** v0.1/v0.2 drew a generic saddle-field kaleidoscope that Ron judged too abstract; v0.5.0
replays the photo's own pattern through the mirrors, with its palette, teeth, feathers and starburst.
Presets are hand-written placeholders.

## Reading of the photo (measured)
Work files: `.heirloom_work/rb_starburst/` (`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).
- **Mirrors.** A four-fold kaleidoscope. Flip-correlation puts the vertical mirror axes at x = 493.5,
  963.5 and 1440 and the horizontal ones at y = 275.5, 539 and 803.5. The translation period is
  946 px × 528 lines, so a mirror cell is 473 × 264 px: 4.1 cells per screen height, aspect 1.79.
  The big white bowtie X sits on the u = 0 corner of the cell (493, 275), the grey X on the u = 1
  corner at the screen centre (963, 539).
- **Everything is in the folded source.** High-pass luma correlates 0.95 across a mirror and 0.93 /
  0.89 across one period: the teeth and feathers are part of the mirrored pattern, not noise over it.
- **Palette** (core medians of a 12-colour k-means, RGB): white (253,254,255), lilac-grey (227,212,226),
  grey (149,143,149), dark grey (102,98,104), charcoal (62,61,62), black (25,27,14), yellow (254,255,43),
  magenta (237,1,248), lavender (155,107,249), cyan (74,245,248). Thin olive (black→yellow) and pink
  (yellow→magenta) fringes. Shares about 9–12 % each.
- **One colourizer cycle** (label adjacency): white → lilac → grey → dark → charcoal → black (the
  bowties) → yellow → pink → magenta → lavender → cyan → white (the chevrons and the diamond rings).
  Fitted band starts: 0, .05, .13, .21, .28, .36, .42, .53, .545, .645, .675, .79 of a cycle.
- **Teeth.** The horizontal edges above and below the big X's are yellow-and-black sawtooth teeth,
  period 33 px (row ACF 0.39–0.42): an H oscillator thrown into the V ramp. It is a chirped saw,
  about 14 teeth per cell, on the left half of the cell.
- **Feathers and starburst.** The grey X is feathered with a 21-line vertical period (ACF 0.41); the
  bowtie "bricks" have 24–25 lines. This is the V oscillator thrown into the H ramp. A yellow
  starburst sparks off the X arms at (u 0.76, v 0.46).
- **No tearing:** per-line shifts are only ±1–2 px.
- **Source field.** No analytic form fits (superellipse 0.34 label purity, saddle 0.32). The source
  is an 81 × 49 knot grid fitted to the median of the 25 folded copies, by ICM in colour space under
  the cyclic colourizer, with white peak/valley hints and the tooth displacement in the model
  (RMS about 34/255).

## What it does
1. **Mirrors.** 20-bit H and V phase ramps (period two cells, virtual-1080 steps) are folded into the
   16-bit cell coordinates u, v. Tile mode uses the ramps unfolded.
2. **Line engine.** Kicked at each line start, it computes the NEXT line in about 140 clocks (fits
   SD's 858). Steps:
   - v plus the drift pan, folded; the starburst tent's v term.
   - The V-tooth saw `0.5 − frac(12.6·ts·v + crawl)` × feather depth, and the per-line fur noise
     (xorshift, held 2–6 lines, restarted every field) × fur depth.
   - Then, for the 81 knot columns: a chirped tooth-phase DDA (`φ += D; D += DD`) gives the saw. Times
     the tooth depth (fading to 0 over 8 columns before the mask column), it displaces v; fold.
   - ×48 → row and fraction; read the two neighbouring rows in one clock (row-parity ROMs); lerp;
     write F8.4 into a ping-pong row RAM (even/odd column RAMs).
3. **Pixel path.**
   - u plus the drift pan, folded.
   - Feather mask `clip((u − u0)·10) + 2·tent` and fur mask `clip((u − .45)·8) + 4·tent`, with
     `tent = tv − |u − .76|·8`. Those masks times the line's feather and fur values displace u; fold.
   - u × 80 → column and fraction; read columns iu and iu + 1; lerp → F.
   - F + (luma − 502) × Etch − pivot, × the Bands gain, + the breathing offset → a colourizer index
     (one cycle = 256).
   - 11 band thresholds → band; colour ROM {Greys, palette, band}.
   - Output conditioning, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Cells | The mirror lattice: cells per screen height 2–8 (4.1 = the photo), so more or fewer X's and diamonds |
| K2 Bands | The colour band sequence: colourizer gain 0.5×–3×. Fewer, broader bands (yellow/magenta dominate) … many fine rings |
| K3 Teeth | The yellow/black sawtooth teeth and the grey X's feathers: count 0.5×–2× (1× ≈ 14 teeth per cell) |
| K4 Speed | Bipolar, still at the centre (±8). The teeth crawl along the edges, the feathers roll and the bands breathe (±0.025 cycle); the sign sets the direction |
| K5 Etch | How hard the picture bends the bands: camera luma (Video) or the oscillator (Osc) added to the field, up to ±0.27 cycle |
| K6 Palette | Band colours: CMYK (the photo), Ember, Ice, Acid, Rose, Mono, Neon, Gold |
| S7 Centre | Which X sits at the screen centre: Grey X (the photo) / White X |
| S8 Greys | The bowties: Grey (the photo) / Colour (grey bands recoloured from the palette's accents) |
| S9 Source | Video (default) / Osc |
| S10 Drift | The pattern drifts under the mirrors (±0.22 cell in u, ±0.14 in v; triangle LFOs of about 10 s and 14 s): X arms swell and shrink, diamonds bloom |
| S11 Mirror | Fold (the kaleidoscope) / Tile (ramps unfolded: the raw cell repeated with hard seams) |
| P12 Burst | Headline: tooth depth, feathers and fur. 0 % = calm, barely serrated bands; 42 % = the photo; past 50 % the teeth spread across the whole cell and deepen to about 0.22 cell and the feathers take the right half; 100 % = every X and chevron shredded into spikes |

Glides (1/8 of the gap per field, ±3 hold) on K1, K2, K3, K5 and P12. At P12 = 0 the teeth keep a
0.02-cell floor so K3 still shows, and Etch stays live in Osc mode.

## Video
- **Video** (default): the camera luma, read from a 256-entry ring at the same x, is added to the
  field. The picture shows as contours bending through the kaleidoscope (face edges become band
  edges); at the default Etch the photo's composition still reads.
- **Osc**: two drifting triangle waves (341 px and 205 lines) feed the same path, so the bands bend in
  slow waves and Etch stays live.

## Engine and contracts
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks, then about 530 clocks).
  - Glides: one shared datapath, four clocks per channel.
  - One serial shift-add multiplier, 15 slots of 32 clocks, with the slot decode registered a clock
    ahead. It computes the H/V phase steps and centred start phases, the tooth DDA constants, the
    feather count, the Burst curves (p², tooth / feather / fur depth, mask positions), the gain, and
    the drift and breathing LFOs (bit-logic triangles).
  - The line engine has its own 13-step serial multiplier for its three per-line products.
- **ROMs** (`starburst.py` → `starburst_pkg.vhd`): field 2 × 2048 × 8 (8 EBR); colour 256 × 30
  (2 EBR); the band map is 11 threshold comparators (`C_BT`).
- **RAMs:** row 2 × 128 × 12 and luma ring 256 × 8 (3 EBR). All are collision-free, with
  `no_rw_check`.
- **Contracts:**
  - The heirloom generated avid.
  - One 36-clock latency in every mode.
  - Chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1] and luma 13/16 (whites ~777) before the blanking gate.
  - Colours in the hardware convention (U register = Cr).
- **Resolution:** geometry is in virtual 1080-line units, so the cells keep the same shape at SD, 720p
  and 1080p. **SD 4:3 shows the central 3 of the 4 cells across** (cells sized to the height, sides
  cropped); 720p matches 1080p.

## Status
- **v0.5.0 built 2026-10-21, NOT flashed.** router2,
  `SEED=3 MAKE_TIMEOUT=900 ./build_heirloom.sh starburst`.
- **Build seeds: all six configs = seed 3** (first try). On seed 1 router2 stalls on HD Analog, and
  seed 2 gave HD Analog only 74.77 MHz.
- Fmax (post-route): HD Analog 83.66, HD HDMI 83.64, HD Dual 79.66, SD Analog 79.97, SD HDMI 83.19,
  SD Dual 84.70 MHz.
- 6824–6854 / 7680 LC (89 %), 13/32 EBR. Every bitstream passes iceunpack.
- RTL GHDL renders (720p60, decimation 1, Osc and Video) match the float prototype and the integer
  model (`imodel.py`). Motion: 17 % of pixels change over 6 frames at default knobs; Speed 0 + Etch 0
  is still.
- **Fit / timing history:**
  - First build: 8395 LC (109 %). Fixes: fold as XOR, band ROM → comparators, shared glides,
    serialised line products, narrowed pixel products, column counters.
  - First timing: 68 MHz. Fixes: column multiply split into nibble partials, glides over four clocks,
    registered slot decode, adder-free LFOs, an extra register after the luma ring (its read register
    had been absorbed into the EBR), a staged etch subtract.

## Deviations from the photo
- **The feathers inside the grey X and the stair-steps of the bowtie bricks are approximated.** The
  knot grid (about 6 px per knot) can't carry 21-line detail, and a full-cell feather oscillator tore
  the smooth chevrons beside it. The feather saw is masked to the grey-X zone (u > 0.78) and the
  starburst; static fur noise covers the right half.
- **Teeth are displaced per knot column** (80 per cell), so a tooth's reset edge is about 6 px wide at
  1080p, and K3 tops out at 2×.
- **All luma is scaled 13/16** (the HD-HDMI bar fix): whites are about 777, and the photo's camera-blown
  yellow reads as a darker gold. Edges are crisper than the camera-blurred photo.
- **The lattice is screen-centred.** The photo's is 7 px right of centre and 13 lines down from the
  capture's black top band, and its cell is 1.4 % larger than the default Cells.
