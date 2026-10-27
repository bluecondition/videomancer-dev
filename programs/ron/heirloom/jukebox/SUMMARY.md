# Jukebox — a wall of rainbow capsules

**v0.5.0 (2026-10-21): built, NOT flashed.** Rebuilt from the photo WIN_20230315_00_28_42 (Heirloom #16):
a wall of tall rainbow capsules, 9.15 columns by 2.08 rows. In each column the contour bands run flat
across and curl into rounded noses at the column's left side, right after a hard cut. The cores on the
row centres are violet/blue and the seams between rows are red/yellow. The columns alternate between
two kinds, half a band apart. Presets are hand-written placeholders.

## Reading of the photo
Measured on a 420 × 261 tile. The photo is exactly one 2-column × 520-line tile, mirrored about the
row centres; the repeats deviate by 1–3.6 RGB levels.
- **Grid.**
  - 2-column period 419.8 px (4.574 per 1920 px, so 9.15 columns).
  - Vertical period 520 lines (2.08 rows); row centres on lines 259.45 + 520 k, seams halfway between.
  - Column cuts at x = 91 + 419.8 k (B columns, ~207 px wide, red-topped) and 298 + 419.8 k
    (A columns, ~213 px, yellow-topped).
- **Three channels, one vertical field.** Each RGB channel is its own function of a vertical field
  V(d), where d is the distance from the row centre and dn = d / 260.
  - **G (the rainbow contours):** a sharp triangle fold, `G = 10 + 233 tri(f)`, with 5.3 folds from
    centre to seam. The fold spacing narrows outward: 63.5, 54.6, 48.1, 42.7, 38.8 px.
  - **The field:** `V = 0.0078 + 3.625 dn + 1.711 dn²` folds, minus a seam dip
    `0.768 clip(1 − 31.5 (1 − dn))`. The dip makes the thin red seam line in the B columns. Joint fit
    over both column kinds: rms 0.036.
  - **R:** `R = r(x) + 36.5 V − 0.09 G` (rms 1.2). R sees 0.35 of the seam dip.
  - **B:** `B = clip(bo(x) − 79 bg(x) V − 0.085 G)` (rms 1.8).
- **Column kinds.**
  - The G field differs by half a fold: E_A = 0.016, E_B = −0.474.
  - The B offset is 434 in A columns and 394 in B columns.
  - R falls from 94 to 52 across B columns and rises (accelerating) from 64 to 103 across A columns.
- **Column transient (every cut).**
  - For about 45 px the vertical gain drops to ~0.35 and the contrast to 0.1–0.6, giving a teal strip.
  - Then a nose offset of ~1 fold at 46–52 px decays with a time constant of about 15–20 px. This
    curls the bands into the "⊂" noses.
  - The noses were measured by tracking contours: the fit with gain a ≈ 1 holds from 46 px on.
- **No textures.** There is no tearing and no noise. The picture is soft horizontally only (analog
  bandwidth); vertical blur is under 1 line.
- **Patch reading.**
  - A vertical triangle runs through a squaring stage. It feeds a many-fold wavefolder on G, a rising
    ramp on R and an inverted, clipped ramp on B.
  - A horizontal oscillator at half the column rate adds a square (the two column kinds) and a
    triangle-ish wave on R.
  - It restarts each column with a reset transient. The transient kicks the fields (the noses) and
    gates the vertical gain (the strip).

## What it does
1. **Per line.**
   - `dn` comes from the vertical phase: a triangle, or a saw for Stack.
   - `V0 = 2 + 928 dn + 438 dn²` (Q8 folds), then the seam dip.
   - `Bands × Vg` for G, and the red and blue ramps `36.5 (Vr + tint)` and `79 (V − tint)`.
2. **Per pixel.**
   - The 2-column phase gives the column kind and the position (0..255).
   - Mirror reverses the B columns; Pairs Off makes every column kind A; Nose stretches the
     position past the reset strip.
   - The measured column tables give E, a, c, o (G), r (R), and bo, bg (B).
3. **G.** `f = a (Bands Vg + Bands·etch·luma) + E − t` (mod 2 folds), then
   `G = 10 + 233 (o + c tri(f))`.
4. **R and B.** `R = r + Rv − 0.09 G` and `B = 184 + bo − bg Bv − 0.086 G`. Palette Swap exchanges
   R and B.
5. **Output.** BT.601 to 10-bit YUV through three colour-matrix ROMs, with luma pre-scaled 16/13
   against the output stage's 13/16 and clamped at 940. Then the output conditioning and the
   blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Columns | capsule columns across: 2 + 22 k², 2–24 (583 = 9.15, the photo) |
| K2 Rows | capsule rows per frame height: 1 + 5 k², 1–6 (475 = 2.08, the photo) |
| K3 Nose | how far the rounded noses reach into each column: the nose tail is stretched 0.31×–3.3× (512 = the photo); the reset strip is kept |
| K4 Speed | band flow, bipolar, still within ±8 of centre; right = bands stream out of the cores; (\|v\|−8)²/4 folds per field (600 = 0.38 folds/s) |
| K5 Tint | red / blue balance of the capsules: ±0.8 fold on the R and B ramps, centre = the photo (±8 deadband) |
| K6 Etch | picture luma pushed into the contours: 3 k² folds per full luma (460 = 0.61) |
| S7 Mirror | every other column reversed: closed ⊂⊃ capsules |
| S8 Pairs | On = the photo's two alternating column kinds / Off = all columns alike |
| S9 Palette | Photo / Swap (red and blue exchanged: red cores, blue seams) |
| S10 Stack | Mirror (the photo) / Saw: rows repeat and are cut flat at every seam |
| S11 Source | Video / Osc |
| **P12 Bands** | contour bands per capsule, 0.25–5× (406 = the photo): 0 % = one soft rainbow gradient per capsule, 100 % = fine rainbow filigree |

## Video
- **How the camera enters.** Its luma (line buffer, `(y >> 2) − 16`) is added to the G field, scaled
  by Etch × Bands. The contours bend around the picture while the palette and the capsule layout
  stay. At the default Etch the face reads inside the capsules and the photo's composition holds.
- **Osc.** Source = Osc substitutes the shell oscillator, a sum of slow x and y triangles drifting
  per field, through the same path. Every video control stays live.

## Engine
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks), one serial shift-add multiplier,
  14 product slots every 24 zone clocks:
  - k1², horizontal step, line-start phase.
  - k2², rows × the timing ID's virtual lines, vertical step.
  - p², Bands; k6², Etch; Bands × Etch; speed², flow.
  - Tint by shift-add; nose stretch from a 64 × 8 LUT.
  - One shared glide (1/8 of the gap per field) on K1, K2, K3, K5, K6 and P12.
- **Per-line engine**, kicked at each line start, computes the NEXT line in ~34 clocks (fits SD's
  858-clock line):
  - dn² and Bands × Vg on a serial multiplier.
  - V0, the seam dip and the R/B ramps as one shift-add per clock.
- **Pixel path**, 21 stages plus 7 of output conditioning = 28, the same latency in every mode:
  - Column kind and position, Mirror / Pairs, nose stretch (two 8×4 partials).
  - Four column-table ROMs, 512 × 16, address = kind & position.
  - Etch partials, a × gs and bg × Bv partials, fold phase and triangle, c × tri.
  - G (×233/256 by shift-add), R and B with the G crosstalk, Palette Swap.
  - Three colour-matrix ROMs (256 × 32), the Y/U/V sums.
  - Every ROM output gets a fabric register before any arithmetic.
- **EBR:** line buffer 4, column tables 8, colour ROMs 6 = 18.
- **Contracts:**
  - damier's generated avid (parity-locked).
  - Output conditioning before the blanking gate: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1],
    luma 13/16.
  - Colours in the hardware convention (U register = Cr, V = Cb).
- **Resolution.** Geometry is in virtual 1080-line units from the timing ID, so 720p shows the same
  picture. **SD (4:3) shows the centre of the HD picture: 6.9 columns of the same capsule shape.**
  Interlaced modes get no field offset (field_n is tied high on hardware).
- **Build hook.** `jukebox.py` holds the measured tables as integers (pure python, as the builder
  needs) and writes `jukebox_pkg.vhd`.

## Status
- **v0.5.0 built 2026-10-21, router2 (the builder default), NOT flashed.**
- **Seeds:** HD Dual = seed 4 (seed 1 stalled the router, seeds 2–3 missed timing); the other five
  are seed 1.
- **Fmax (post-route, last line):** HD Analog 79.16, HD HDMI 80.04, HD Dual 76.94, SD Analog 81.55,
  SD HDMI 84.36, SD Dual 85.40 MHz.
- **Resources:** 6467–6483 / 7680 LC (84 %), 18 / 32 EBR. Every bitstream passed iceunpack.
- **Verification:**
  - The RTL is bit-exact against the integer model `imodel.py` in 720p GHDL sims (black video,
    warm-up 2 and 10).
  - The default motion moves the contours outward from the row centres.
- **If HD Dual regresses:** try `NEXTPNR_ROUTER="--router router1"` before restructuring.

## Deviations from the photo, and why
- **The reset strip.** In the first ~45 px of every column the photo has 2–3 thin bowed lines and
  small loops near the seams. The program draws a smooth teal/olive band there.
  - The column tables are separable (per-position offset, gain and contrast), and that structure
    cannot carry the strip's 2-D detail.
  - Eight non-separable models (crossfade, fold shift, reset pulse, d-weighted contrast, monotone
    offset, blur-only and others) fitted no better than the chosen tables. Those reach rms 0.079,
    against 0.02–0.03 in the column interiors.
- **Corners.** The top-left and bottom-left corners of the outer bands are a little squarer than
  the photo's sweep.
- **Pale line.** A thin pale line runs the full height at the B-column cuts; in the photo it is
  visible near the seams only.
- **Column widths.** Both column kinds are 209.9 px wide; the photo alternates 207 / 213 px.
- **Capture borders.** The capture's black top rows and dark left edge are not drawn.
- **Motion is designed.** The photo is a still: the bands stream out of the cores and the colours
  slide through the R/B ramps.

## Working files
`.heirloom_work/rb_jukebox/`:
- `notes.md`: measurement log.
- `tile.npy`: the averaged tile.
- `tables.json`: the fitted column tables, in float.
- `protoF.py`: float prototype in the hardware's structure.
- `imodel.py`: integer model of the RTL; it imports `jukebox.py`.
- `gen_hook.py`, `hook_template.py`: write `jukebox.py`.
- `sheet.png`, `isheet.png`: control sheets.
- `cmp_proto.png`, `cmp_final.png`, `cmp_motion.png`: comparisons.
