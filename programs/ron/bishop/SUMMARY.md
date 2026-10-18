# Bishop — dense wireframe head (v1.0.0)

## What it does
A faceted wireframe human head drawn as single-colour lines on black or over
the incoming video. The face morphs: the mouth opens and closes, the eyes
blink shut, and five expressions blend into each other. A noise field can
wobble every vertex. Eyes and mouth can be solid-filled. The slider grows the
head from its natural size to ~95% of the screen height.

## How it works
Pure line-buffer rasteriser. `build_svg_mesh.py` turns `face12.svg` into
`bishop_mesh_pkg.vhd`: 256 edges in Q9.7 DDA form plus a sparse per-vertex
morph table (51 moving vertices across 6 morphs). Once per frame, a vblank
copy engine (`cp_proc`) resolves scale, morph blend and noise per edge on one
shared serial multiplier and writes the active mesh into EBR. During each
scanline a shared rasteriser walks the active-edge list and stamps the next
row into a 1-bit dual-bank line buffer; stroke width is slope-scaled so lines
read the same weight at every angle. Solid fill uses an enter/leave
set-reset walker over separate boundary buffers (no read-modify-write, so it
stays in EBR). The per-pixel path is one buffer read and an output mux.

## Controls
- K1 Mouth — 0% closed .. 100% open
- K2 Line Weight — 1 / 3 / 5 / 7 px (at 1080; scaled per mode, never under 1 px — see v1.0.3)
- K3 Palette — Green, Amber, Cyan, Red, White, Magenta, Yellow, Blue
- K4 Expression — blends Neutral → Happy → Sad → Angry → Surprised
- K5 Noise — vertex wobble amplitude; the wobble speed rises with it (one knob, never gated)
- K6 Position — horizontal pan, -100..100% = -512..+511 px about the measured centre (1080; scaled per mode)
- S7 Fill — Outline / Solid (eyes and mouth filled)
- S8 Eyes — Closed / Open
- S9 Noise — Smooth / Streaks
- S10 Invert — Off / On: black lines (and fills) on a palette-colour field; over Video it is black ink
- S11 Background — Black / Video
- P12 Head Size — natural size .. ~95% of screen height (same range in every mode since v1.0.3)

## Presets
Neutral, Happy, Angry, Surprised, Solid Fill (magenta, filled, half size),
Static (1 px green, 75% streaky noise), Big Head (amber 5 px, max size),
Video Ink (black filled ink over video, panned right). Hand-written — worth
re-capturing from the hardware once the look is dialled in. (The old presets
had S8 Eyes = Closed; they are Open now.)

## Status
**v1.0.0 FINAL, HW-validated, signed off 2026-10-09.** v1.0.3 (2026-10-17) adds resolution scaling — built, not flashed. Open ideas (not planned): S8 Open/Blink animation, K6 Tilt rotation.

- 2026-05-31: expressions milestone HW-validated ("looks very good").
- 2026-06-01: horizontal thickness fix HW-approved; slope-scaled diagonal
  pad HW-rejected and reverted (the v0.3 uniform stroke is a different
  mechanism: per-edge dynamic x-extent clamp + slope-scaled extension).
- 2026-08-27..30: v0.3 rework — uniform stroke, eye-spur fix, P12 head
  size, streak-free fill, HD timing rework. Built, NOT flashed.
- 2026-10-07: v0.3.0 contract pass (this build) — `processing` type so
  S11 Video actually gets input, blanking gate (y 64 / uv 512 outside avid),
  black raised 0 → 64, palette U/V swapped at output compose (K3 colour
  names were unswapped BT.601, so Red/Blue/Amber/Cyan/Yellow would have
  shown wrong hues), saw-active guard on the vsync edge (serrated analog
  vsync could re-zero the line counter and throw the head off-screen).
  Awaiting hardware check.
- 2026-10-07 v0.4.0 control pass: K5 Noise Speed + K6 Noise Spread merged
  into one K5 Noise (speed was invisible at the default Spread 0); K6 is now
  Position (pan, applied to the measured line centre one register after
  hsync, so the hsync cone is untouched); S10 Brightness replaced by Invert
  (output-mux only; the old 3/4-luma "Normal" level is reused as the field
  luma so an inverted field is not blinding); 8 presets. HW: "looks
  alright" except 4-5 thin vertical lines left of the head at power-up
  that a K6 pan erased.
- 2026-10-08 v0.4.1: those lines were stale bits in the line buffer outside
  the head's clear window (random EBR content / stamps before the mesh and
  geometry settled). New R_WIPE state clears 32 entries per line at a
  rolling pointer (whole buffer every 64 lines), before the stamps so it
  can never erase them. 32 cycles off the HD line budget. HW: "everything
  looking correct" except hue flips on thin lines (blue → red in places,
  gone at higher line weight).
- 2026-10-08 v0.4.2: the hue flips are the core's 444→422 packer taking U
  from the EVEN avid-relative pixel of each pair and V from the ODD one
  (yuv444_30b_to_yuv422_20b.vhd), so an isolated 1-px stroke pixel reaches
  the encoder with one chroma axis only. New registered output stage gives
  both pixels of a pair the pair-averaged chroma (alpha = hits in the pair,
  0/1/2; luma stays per-pixel); needs one pixel of lookahead, so LATENCY 3
  → 5 (lookahead + output register) for ALL modes. HW: "colors are fixed"
  but the vertical lines were back, and not on every load.
- 2026-10-08 v0.4.3: the wipe had a parity hole — one pointer advancing 32
  per line while the banks alternate per line, and 2048/32 = 64 chunks is
  even, so bank A only ever saw the even chunks and bank B the odd ones;
  half of each bank was never wiped (lines showed only when load garbage
  landed in an unwiped half). One wipe pointer per bank now; each bank is
  fully wiped every 128 lines. HW: lines gone.
- 2026-10-09 **v1.0.0 FINAL** — Ron signed off ("we can finalize bishop for
  now"). HW-validated: contracts, colours, pan, invert, wipe. Presets are
  hand-written and could still be re-captured from hardware.
- 2026-10-17 **v1.0.3 resolution scaling** (scaling-work pass; 1080p output
  bit-identical to v1.0.2 in the GHDL sim, 0 differing pixels). Before, the
  mesh was fixed pixels (~516 x 777 rows): 720p cropped the chin, SD showed a
  ~4x-zoomed fragment. Now every mode draws the 1080p head scaled to the
  screen height (virtual 1080-line units; SD is a 1440-unit 4:3 window, so
  the head keeps its shape on a 4:3 screen):
  - the per-mode factor k (x: 1, 2/3, 1/2; y: 1080 / field lines) is folded
    into the copy engine's P12 scale product — sc_mx_r / sc_my_r =
    256*(k*(1+m/256) - 1), a signed 9-bit factor per axis from a serial
    shift-add in size_proc (the shared multiplier is now 11x9). y gets a
    per-mode row offset (the head centre lands at HEAD_Y_MID*k) and the bottom
    field of interlaced SD half a field line lower;
  - P12's ceiling is the 1080p 83 in every mode (was 0 below 820 lines);
  - stroke half-widths per axis (padx px, pady field lines): the 1080 stroke
    scaled and rounded to the nearest odd width, min 1 px — 720p 1/3/3/5 px;
    SD x 1/1/3/3 px, 480i/576i rows 1 line (576i 7 px: 3); 1080p unchanged;
  - K6 pan, the head clear window hw, and the noise span floor scale too;
  - SD wipes 8 entries/line (was 32); R_CLEAR never ends before the 256-edge
    walk (N + 12 cycles);
  - overrun guard: an hsync arriving while the raster is still stamping ends
    the pass (remaining stamps on that row dropped) instead of being missed
    (a missed hsync skipped the next row's R_CLEAR walk: DDA step lost, edges
    starting there loaded stale x — a kinked/displaced head).
  Sheet: .scaling_work/sheets/bishop_v2.png (r720 0.98, rSD 0.92; was
  0.22 / 0.09).

## Known limits
- Line budget (cycle model, .scaling_work/bishop_tmp/budget2.py, K4 swept):
  SD 480i, eyes/mouth open: 82-97% at P12 0, up to ~105% at P12 50%,
  ~114% at P12 max — one or two rows per field (the forehead band) overrun
  for some expressions and lose a few stamps (guarded). Eyes/mouth closed
  with 3-7 px lines: 103-120%. 576i similar; 576i 7 px (3-line strokes)
  up to ~140-170%. 720p: open <= 87%, closed 7 px up to ~127%. The fixed
  cost (R_CLEAR's 268-cycle edge walk / clear window + ~7 cycles per active
  edge) is ~55% of an SD line, so SD has little room for stamps.
- 1080p already overruns with eyes/mouth closed at 5-7 px (model: up to
  ~126% at P12 max, the near-flat lid edges sweep 170-191 stamps per row,
  mostly clamped away). Pre-existing; since v1.0.3 those rows lose stamps
  instead of skipping a row.
- Worst-case HD line budget, eyes open ≈ 1960 of 2200 cycles (7 px lines, full size, incl. the 32-cycle wipe).
- Mesh ceilings: 256 edges, max |x| ≤ 255 at scale 1, stamp ≤ 127.

## Build notes
- Build with router1: `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron bishop`.
  router2 left HD configs 0.5–1.9 MHz over target and seed-fragile
  (HD Analog needed seed 4 on the Aug 30 build).
- Seeds / Fmax for the current build:

2026-10-17 v1.0.3 (resolution scaling), `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron bishop`,
**all 6 configs seed 1**, 6850/7680 LC (89%, was ~6320), 22/32 EBR.

| Config | Seed | Fmax (MHz) |
|--------|------|------------|
| HD Analog | 1 | 79.92 |
| HD HDMI | 1 | 87.61 |
| HD Dual | 1 | 80.87 |
| SD Analog | 1 | 78.43 |
| SD HDMI | 1 | 81.61 |
| SD Dual | 1 | 75.96 |

Previous build:

2026-10-09 v1.0.0 (= v0.4.3 bitstreams, version bump only), router1,
~6320 LC (82%), 22/32 EBR. **HD Dual needs SEED=2** (seed 1 stalled router1 and timed out; the builder retried).
(v0.4.2 was 84.2 / 82.0 / 80.5 on HD, all seed 1.)

| Config | Seed | Fmax (MHz) |
|--------|------|------------|
| HD Analog | 1 | 77.69 |
| HD HDMI | 1 | 78.59 |
| HD Dual | **2** | 76.28 |
| SD Analog | 1 | 79.55 |
| SD HDMI | 1 | 88.43 |
| SD Dual | 1 | 85.04 |

## Files
- `bishop.vhd` — program; `bishop_mesh_pkg.vhd` — generated mesh (do not edit by hand)
- `build_svg_mesh.py` — SVG → mesh package; `preview_face.py <morph>` — renders any morph overlay for tuning
- `face12.svg` — source model; `face_labels_*.svg` — per-expression previews
- `face_mesh.py`, `build_face_mesh.py`, `extract_mesh.py` — the earlier sparse-mesh generators (unused on this branch)

## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

v1.0.2 (2026-10-15): **seed 3**, set as `[firmware] seed = 3` in bishop.toml. LZX's stock build uses one seed for all 8 configs.
- **Why:** v1.0.2 adds the labelled-default decode (K2/K3). With it, seed 1 stalls router1 on HD Analog (overuse stuck at 6858 for 10+ min). v1.0.1 had built on seed 1.
- **Sweep:** seed 2 built 7/8 (HD Analog stalled); seed 3 built 8/8.
- **Own HD timing** on seed 3 (scratch core without LZX's Sync Out generator): 79.7 / 81.1 / 75.1 MHz (Analog / HDMI / Dual). Max 7057 LC (SD Standalone).
- The dev flow (router2) is unchanged.
