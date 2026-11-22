# Corolla (Vellum)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/vellum/corolla.vmprog`.

## What it draws

The overlapping-circles grid: the Flower of Life (Leonardo's studies in the Codex Atlanticus) and the Japanese
shippo, read as translucent vellum discs, one on every point of a triangular (or square) lattice. At the
power-up default the disc radius equals the lattice pitch, so every disc passes through the centres of its six
neighbours: the Flower of Life fills the screen, five pitches per screen height.

The discs are films of three densities, one per sublattice of the triangular lattice's 3-colouring (weights
1, 3, 4; the square lattice's checkerboard: 1, 3), and the weighted overlap folds through a triangle into four
stepped greys: the curved triangles are black and each petal takes the grey of the sublattice its two flanking
discs share, so the six petals round every lattice point alternate two greys and the white petals join into a
large hexagonal honeycomb. Ink is the parity of the overlap count: the classic black-and-white petal checker.

    lattice  (u, w) by DDAs; fold into a quarter cell; 10 candidate centres (hex) / 6 (square)
    d_k^2    = (a_k - u)^2 + (c_k - w)^2 from two per-field tables (G u^2, G u) and constants
    Bold     cov_k = clamp(1/2 + (R^2 - d_k^2) / 2R)        Fine  rings of half width P / 28
    Grey     tone = triangle(sum w_k cov_k / 8)              Ink   tone = triangle(sum cov_k / 2)

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | pans the lattice +-512 virtual px across (the slow turn's centre moves with it) |
| K2 | Y | the same, down |
| K3 | Scale | pitch 48 .. 384 virtual px, exponential (default 740 = 216 v-px, five pitches per height) |
| K4 | Oval | horizontal stretch 1 .. 2 (exponential), default 0 = round: discs become ovals, the petals stretch, and lean as the lattice turns |
| K5 | Drift | bipolar speed (Hypnos v1.1.0 decode), +-24 counts still, 2^-12 .. 2^-3 pitch per field: the lattice glides along its own row and turns slowly; default 650 (+27 %) |
| K6 | Camera | the box-filtered luma moves the disc radius about the field's mean level, 0.45 pitch per full swing at 100 % (default 100 %) |
| S7 | Glide | Off / On (default On): eases X, Y, Scale, Oval, Camera, Bloom (1/16 of the gap per field, at least one count, landing exactly, behind a +-1-count hold) and the Drift speed (1/8); Off = this field's knob values |
| S8 | Lattice | Hex (the Flower of Life) / Square (shippo: the same disc size on a sqrt2 P square grid, so R = pitch gives the four-petal shippo) |
| S9 | Tone | Grey (the three films' four stepped greys) / Ink (black and white, overlap parity) |
| S10 | Weight | Bold (discs) / Fine (each circle as a ring, the compass drawing; ring width P / 14) |
| S11 | Source | Video / Osc |
| P12 | BLOOM | R / pitch: 0 % = 0.35 (separate dots in three greys); 16 % = 0.5 (kissing coins); 25 % (white and grey hexagons); 49 % (interlocking rings); 68 .. 72 % = exactly 1.0, a detent (the Flower of Life, power-up 70 %); 100 % = 1.3 (dense lace, the climax) |

## Motion

Two per-field accumulators in the engine's register file, advanced once per field at the zone start (no avid
for 1024 clocks): the glide phase along the lattice's row (Q22 of 3 pitches, the period of the tile AND its
3-colouring; square: 2 spacings, the checkerboard's period) and the turn (Q20 of 60 degrees hex / 90 square,
the lattice's symmetry). Both wrap exactly one period, so nothing ever jumps. The glide runs 2^-12 .. 2^-3
pitch per field (the top capped at 2^-3 rather than 2^-2, to keep well under half a period per field), the turn at 1/32 of the
glide in units of the symmetry period. Default 650: about one pitch in 15 s (14 px/s at 1080p) and about
2 degrees in 15 s. Drift below centre reverses both. Only Drift at centre stops it.

## Camera

The camera is never shown: it sets the disc radius, about the field's running mean level (an exponential mean
of the camera level over about a million pixels, latched per field):

    R(L) = R0 + 0.45 pitch x K6 x (L - Lmean) / 219      (area-linear, clamped 0 .. 1.30 pitch)

Brighter than the mean swells the discs (more overlap: the petals fatten, white opens), darker shrinks them
(black opens between thin petals). K6 25 % is a gentle swerve (+-0.03 pitch typical), 50 % bends the petals,
100 % turns the subject into a dark black-lace mass against lighter lace (the bear reads as a silhouette at the
default Scale, more clearly at smaller Scale: the elements are large). A black input (L = mean = 0), or K6 0 %,
is exactly the clean figure. Luma -> box (64 px at 1080, 32 px at 720p and SD: 48 / 64 virtual px) ->
vertical one-pole IIR (32 .. 48 virtual lines) -> camera level.
Osc substitutes the shell's drifting triangle lattice at the same point at full swing (about twice a picture's).

## Engine

- Per field: Hardline flute's microcoded engine (37-bit accumulator, 256 x 32 register file in block RAM, one
  adder, serial 18 x 18 multiplier, serial shift), started at every zone start from address 0. The
  microprogram and its emulator are in corolla.py (551 words); the integer model runs the emulator. It does the
  knob holds and glides, the Drift speed / phase / turn, the pitch (2^(i/256) table), the turn's sin / cos
  (interpolated quarter sine), the oval, the four DDA steps and two start values, the edge scale G = 8 P_a / R
  (a 1 / R table), the candidates' constants, the ring width and the camera table constants. 3,100 .. 3,400
  clocks; its T0 output (~1,450 .. 1,700 clocks) starts the fills, which run in parallel.
- Fills (vblank): u and w tables (512 x 32: G t^2 and G t by second differences), the 256-entry camera table.
- Per line: two line DDAs (Q22 of the span). Per pixel (stages 10 .. 29): u3 = 3u (the u-cell index mod 3 for
  the colouring) -> fold -> tables -> six X and four W variants -> ten edge distances (saturated to 11 bits)
  -> Bold / Fine coverage (17 levels per edge) -> film weight -> sum mod 128 -> tone triangle -> tone ROM ->
  luma [1 2 1]/4 -> 13/16 toward black (white 776) -> blanking gate. No pixel-path multiplier. U = V = 512.
- One latency (32) in every mode; generated avid with the shell's parity-locked start; virtual 1080-line
  coordinates (SD shows the centre 4:3); AA unit 1 / 1.5 / 2 / 3 virtual px by mode.

## Status

- Built 2026-11-22 through the shared queue with **router1**, every config on **seed 1**. Not flashed.
- Post-route Fmax (the builder's routed figure; every config passed on its first seed): HD Analog 82.56 MHz,
  HD HDMI 84.96, HD Dual 81.45 (HD needs 74.25); SD Analog 80.97, SD HDMI 77.87, SD Dual 82.56. All six
  bitstreams pass iceunpack.
- 5,406 .. 5,421 / 7,680 LC (70 %), 23 / 32 EBR (u and w tables 4 + 4, engine table ROM 4, microprogram 4,
  register file 2, camera line store 3, camera table 1, box 1). No `emulate_transparency`; dupin: no
  duplicate-input carries or LUTs. Router2 was not tried.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum corolla`.
- Model = RTL (GHDL), pixel diff 0 on every render compared (the model runs the same microprogram in its
  emulator, so the per-field arithmetic is checked too):
  - 720p decimation 1 (921,600 pixels): the power-up defaults with a black input (Glide On, Drift 650 moving):
    0 against the model after 3 fields, ~10 % against 2 or 4 fields; bear.png at K6 100 % (the model fed the
    tester's stimulus luma and the RTL's running mean level, 104); Source = Osc (mean level 92);
  - 720p decimation 2: Drift still with Glide Off (0); the power-up defaults after 9 fields (0; 9.6 % one field
    either side: with the 3-field capture above, two captures 6 fields apart travel exactly as the model);
    Drift +100 % with Glide Off (0; ~50 % one field either side);
  - NTSC decimation 1, both fields (720 x 243): 0 (the same picture: the centre 4:3, five pitches per height).
  Rasters rendered: 720p (decimation 1 and 2) and NTSC; no 1080p or 1080i render. No stepped knobs (the Bloom
  detent is a range; its ends are 0.35 and 1.3). The Osc carries Hardline dazzle's line-0 fix (rosette's path).
- Renders in `.vellum_work/rb_corolla/`: `cmp_final.png`, `sheet.png`, `strip.png`, `motion.png`, `camera.png`
  (integer model), `rtl_default.png` (GHDL, 720p decimation 1, power-up defaults), `presets.png`, `proto_*.png`.

## Cut / deviations

- **Weighted films (the spec: tone = the plain overlap count).** At R = pitch the plain count is only 3 or 4,
  two tones, which made Grey a grey copy of Ink; the three sublattices' films (1, 3, 4) give the four tones the
  spec asked for, and the triangle fold keeps every Bloom position on the full black-to-white range.
- **Camera about the field mean (the spec: luma adds to R).** Adding only (swell where bright, toward 1.45) was
  illegible: near R = 1 the mean tone rises only up to R = 1.15 and falls after, and a bright background swelled
  the whole frame. Relative to the mean, dark areas shrink and bright swell: a black input still moves nothing.
- **10 candidate centres, R capped at 1.30 pitch** (12 would reach 1.45): the camera cannot swell past 1.30, so
  at Bloom 100 % only the darker-than-mean areas move (they still visibly shrink).
- **Oval stretches along the screen x axis**, so the petals lean only as the lattice turns (at the power-up
  angle the stretched flower stays mirror-symmetric).
- **Drift top 2^-3 pitch per field** (not 2^-2), turn = 1/32 of the glide speed.
- **Square lattice spacing sqrt2 x pitch** (keeps the disc size, makes the default the shippo).
- Fine rings in Grey take their sublattice's grey (0.25 / 0.75 / 1); the square lattice's Fine reading is the
  dimmest combination (two grey rings).
- Not verified on hardware: anything. The microcoded engine is flute's (GHDL-verified there and here, not
  flashed anywhere). Full-swing luma edges move; the [1 2 1] + 13/16 conditioning is built in; if HD HDMI bars
  show, Ink + Fine at a small Scale is the first place to look.
