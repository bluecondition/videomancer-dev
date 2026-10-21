# Crenel — hot-pink crenels and camera domes, tiled by a bent oscillator

**v0.5.0 (2026-10-21): built, NOT flashed.** Heirloom #10, rebuilt from the photo
`photos/WIN_20230223_01_46_41_Pro.jpg` (the v0.1.0 comb-and-emboss sketch was judged too abstract).
Work files, measurements and the full history: `.heirloom_work/rb_crenel/` (`notes.md`).

## Reading of the photo
Held up in a float prototype (`proto.py`, `proto_final.png`) and an integer model that mirrors the
RTL (`imodel4.py`).
- **Tiling.** Every row is one source picture repeated with period T(y), and the phase resets at
  every line start. Feature positions / T agree across both tiles to 0.01 (V centres 527/935 =
  0.564 and 1462/940 = 1.555; arch centres at 52 and 1012). Rectified to tile phase, the two tiles
  differ by a mean |d| of 2–5, against 25–45 shuffled. That is a camera picture squeezed about 2x
  and tiled by a horizontal oscillator.
- **The oscillator is bent (FM).** Autocorrelation per 4 lines: T = 1064 px at line 36, 1023 at
  100, 989 at 140, 962 at 205 (the minimum), 977 at 272. The period then slips in horizontal
  bands: 950 → 928 at 300–411, 1062 → 1020 at 412–477, 935–941 at 478–557, 955 → 975 at 558–645,
  925–950 at 646–790, 1058 → 1029 at 791–852, 990 → 962 at 852–1018, then 924–946.
- **Crenel row (y 38–276), exactly 3 per tile** (period T/3). Averaged over all teeth in crenel
  phase c:
  - the shape: a bar (y 106–198) through teeth (y 42–270). The left edge is a "<" chevron at
    c = 0.535 at the bar centre, 0.64–0.67 at the tooth ends. The right edge is at c = 0.952 with
    rounded outer corners (r 12) and concave fillets into the bar (r 14);
  - a hot-pink ridge (R 210–250) on the left edge, decaying rightwards over about 25 px, with a
    dark undershoot inside the bar;
  - thin pink rims (165,8,128) on the horizontal edges and the right edge;
  - fill purple (86,3,122), hw10 Y 207 U 623 V 671, above y 149, and magenta (110,2,118), hw10
    Y 239 U 686 V 647, below it;
  - outside the teeth: two feedback echoes of the ridge, 24 and 57 px to the left (the far echo is
    stronger near the bar), a dim purple zone between the near echo and the ridge, and a blue glow
    (B +50, about 36 px) on both sides.
- **Below the crenels:**
  - a bright-blue camera dome: a disc arc with its top at y 303, R 458, centred at t 0.03;
  - a slipped row (412–478) carrying the top of a pink V;
  - pink V's at t 0.56, tips at y 557, edges 0.488 → 0.547 and 0.635 → 0.555;
  - camera blocks in the crenel gaps (574–645);
  - "(" arcs, 13 per tile (645–700);
  - staggered dash rows 7 lines tall with a random period of 11–15 per tile (700–790);
  - camera teeth on a navy-blue bar (B 120) (790–852);
  - a soft pink arch ring centred at t 0.04, y 1064, R 172 (crest y 874), cut at t 0.155 (852–948);
  - pink blocks on teeth 1 and 3 with camera in the gaps (948–1018);
  - a dash floor.
- **Palette** (k-means, hw10 in the hardware convention):
  - navy ground (4,9,54..130), Y 107–137, U 470–490, V 593–727;
  - bright blue (3,37,248), Y 239, U 391, V 904;
  - hot pink (193,2,118), Y 312, U 816, V 603;
  - green fringe (11,106,108) / (5,119,76).
  The blue areas run through a periodic colouriser of a soft key (navy → deep blue → blue → cyan
  folds), which draws the photo's concentric ripples on the dome flank and the vertical stripes
  in the blocks.
- **Texture.** The camera rows are torn sideways in runs of lines: about 1 run in 4 is shifted by
  4–26 px.

## What it does
1. **Tile phase.** A 24-bit beam accumulator, `t = phi + x * inc(y)`. inc(y) = 2^24 / T(y) comes
   from a measured 540-entry table (every second line, interpolated). It is FM-scaled about the
   centre (2^24 / 980) by P12 Bend (with a +-12 % LFO) and by K2 Tiles. The tile parity drives
   Mirror.
2. **Crenel phase.** `c = frac(k * t)`, k = 1..6 (K1), by shift-add, with the tooth index from the
   carry. The arcs/dash phase is 4c plus a per-line offset (and a sine wobble on the arcs row).
3. **Per band, a pink window:** from the per-line edges (the chevron and fillets from the line ROM,
   the V edges as linear slopes in y), the ridge profile by distance from the left edge (64-entry
   ROM x a per-line amplitude), rims, the bar undershoot, and the fill woven by the source.
4. **Echo lines.** A 1024 x 8 profile ROM (near-bar class x left distance) times the K3 gain.
   The blue glow comes from a 32-entry profile of the outside distance.
5. **Camera unit.** A soft key on a window of t (torn), c (torn) or the arcs phase. It rises over
   2^ashl and falls over 2^ashr phase units (K5 widens both). It is multiplied by the source
   (line buffer: camera luma, or the Osc oscillator) and turned into a colouriser level. The
   arcs/dash rows add a blob envelope round t 0.6.
6. **Arch ring.** `r^2 - R^2` from an EBR squarer of the 10-bit tile-phase offset plus the
   per-line `(y - 1064)^2 - 172^2`, giving a ring-profile level.
7. **Priority** (pink > arch > echo > camera > ground) gives class(3) & level(6) into a colour ROM
   (4 palettes x 8 classes x 64 levels), then the output conditioning and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Crenels | the pink "+" crosses: 1–6 per tile (3 = the photo). Every crenel-locked row follows: the blocks, the camera teeth, the pink lower blocks, the arc and dash spacing |
| K2 Tiles | how many copies of the picture lie across the line: 1–4 (2 = the photo) |
| K3 Echo | the feedback echoes left of every tooth: none (clean edges) → strong double echoes (square law) |
| K4 Drift | bipolar slide of the whole tiled picture along the line (still in the centre ±8); the photo's patch drifts slowly right |
| K5 Ripples | the camera blobs' soft-key width: hard flat blobs → wide flanks with many ripple folds |
| K6 Palette | Photo (fitted) / Ember / Jade / Rust |
| S7 Tear | the sideways tears of the camera rows (default On) |
| S8 Outline | the pink shapes reduced to their ridges and rims |
| S9 Mirror | every other tile mirrored: crenels and V's meet in chevrons |
| S10 Roll | the band stack rolls up the screen (V-hold lost, 2 lines per field) |
| S11 Source | Video / Osc (default Video) |
| P12 Bend | the oscillator FM depth (0–2.5x of the measured bend): 0 = upright battlements, straight V's, round dome; 40 % = the photo; 100 % = crenels leaning into long slashes and the slipped rows torn apart. The tear amplitude follows (0.25x → 2x) |

Motion at defaults:
- the tiled picture drifts right (about 36 s per tile);
- the bend breathes ±12 %;
- the tears are redrawn every 8 fields;
- the Osc source's oscillator walks.
K1, K2, K3, K5 and P12 glide (1/8 of the gap per field); K4, K6 and the switches are latched per
field. Image-tester note: labelled knobs sim at 512 (K6 = Jade), so sims pass
`--set rotary_potentiometer_6=0` for the Photo palette.

## Video
- The camera luma is written into a 2 x 1024-entry line buffer (one whole source line per tile),
  so the camera is tiled across exactly as the photo's picture is, one line late.
- The source feeds every camera area through the same keys and the rippled colouriser, and it
  weaves the pink fills (±4 levels).
- In Video mode the dome, blocks, arcs, dashes and teeth show the camera picture. The crenels, V's
  and arch keep the photo's composition.
- Osc substitutes a bright level dipped by a triangle oscillator (5 per tile), phase-bent by a slow
  vertical triangle. It drifts per field and gives the soft dark lobes the colouriser folds into
  the photo's ripple arcs.
- Every camera-path control (K5, S7, K4, P12) stays live in both modes.

## Engine
- **Field values** (vblank zone):
  - glides on a shared 4-clock-per-channel datapath;
  - K1 → shift-add selects; K2 → tile scale; K3 → echo gain (square-law LUT); K5 → ripple shift;
    P12 → bend;
  - the bend LFO on the shared serial multiplier;
  - the drift and roll phases, the tear seed and the tear scale.
- **Per-line engine**, kicked at the source avid rise. It computes the NEXT line in 12 slots x 20
  clocks (about 240 clocks, so it fits SD's 858-clock line) on one serial 18 x 11 shift-add
  multiplier:
  - the line ROM reads (inc pair, chevron, ridge gain, dome half-width, arc bow, fillet, dash hash)
    and a sequential band search;
  - the xorshift32 tear run generator;
  - FM bend, tile scale, the tear offset in t and in c, the V edge slopes, the dash row index and
    the arch dy^2.
  The line record is assembled in the horizontal blanking, 24 clocks after the source avid fall,
  when the line's last pixel has left the pixel path. It is single-buffered; line 0 is computed
  late in the vblank zone.
- **Pixel path, 24 stages:**
  - S1: tile phase / mirror;
  - S2: crenel shift-add, torn phase;
  - S3: crenel / arcs phases and the line-buffer read;
  - S4: window offsets;
  - S5: windows and 8-bit distances;
  - S6: rims, ridge ROM, echo-ROM address, key, squarer address;
  - S7–S8: amplitude, glow, key gain, envelopes;
  - S9–S10: echo level, colouriser level, ring radius;
  - S11: priority;
  - S15–S17: colour ROM and fabric copy;
  - S18–S23: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1];
  - S24: luma 13/16 and the gate.
- **ROMs** (`crenel.py` → `crenel_pkg.vhd`): colour 3 x 2048 x 8 (12 EBR), line table 1024 x 16
  (4), echo profiles 1024 x 8 (2), squares 256 x 16 (1), line buffer 2048 x 8 (4, no_rw_check),
  for 23 EBR.
- **Contracts:**
  - generated avid (parity-locked start, field-latched width);
  - one 24-clock latency in every mode;
  - virtual-1080 rows and a fixed virtual tile width;
  - output conditioning before the blanking gate;
  - colours in the hardware convention (U = Cr).

## Status
- **v0.5.0, 2026-10-21: built, NOT flashed.** **ROUTER1**:
  `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh crenel`. All six configs are seed 1.
  Post-route Fmax: HD Analog 77.59, HD HDMI 84.03, HD Dual 86.27, SD Analog 79.81, SD HDMI 85.27,
  SD Dual 85.65 MHz. 7048–7066 / 7680 LC (92 %), 23 / 32 EBR. All bitstreams pass iceunpack.
- **router2 stalls on SD Analog** (overused = 2 frozen for >7000 iterations): build with router1.
- **Fit history:**
  - v1: a generic per-line record with two camera units, double-buffered, 9628 LC (125 %).
  - Ablations (standalone yosys + nextpnr --pack-only): the pixel datapath was ~5000 LC.
  - v2: one camera unit, single-buffered record, 8-bit distances, the arcs phase derived from the
    crenel phase: 7933 LC.
  - v3: echo profile ROM, one Osc oscillator, no V shadow, integer K1: 7014 LC.
- **RTL bugs found by diffing the GHDL render against the integer model:**
  - the arch offset was zero-extended;
  - the arch level was folded by a narrowing resize;
  - a dash gain overflowed 4 bits;
  - SD's line-buffer write step left stale entries (a hatch in every camera area).
  The final 720p render matches the model with a per-band mean |d| of 0.1–5 (the arcs row 15: a
  2–3 px offset).

## Deviations from the photo, and why
- The blob-for-blob content inside the blue areas is a real camera picture in the photo. Video
  mode shows the live camera there; Osc only matches the statistics (bright level, soft dark
  lobes, ripple folds), so Osc's camera areas are cleaner and more geometric than the photo's.
- Dropped to fit the HX4K (the first RTL was 125 %):
  - the slipped row's second camera wedge, and the blocks row's dim middle zone;
  - the V's dark shadow through the arch bar;
  - the echo spacing control and its wobble (K3 sets the strength only);
  - the Osc source's second oscillator.
- Not reproduced: the second V's separate pink stroke in the slipped row (that tile is not periodic
  in the photo), and the green specks / red fringe as separate objects (they appear only as
  colouriser folds).
- S7 Tear is the subtlest switch: it slices only the camera rows, as in the photo.
- Resolution: 720p renders the same picture as 1080p. SD 4:3 shows 1.5 tiles across instead of 2;
  the tile width is fixed in virtual units so the arch stays round and the crenels keep their shape.
