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
- K2 Line Weight — 1 / 3 / 5 / 7 px
- K3 Palette — Green, Amber, Cyan, Red, White, Magenta, Yellow, Blue
- K4 Expression — blends Neutral → Happy → Sad → Angry → Surprised
- K5 Noise — vertex wobble amplitude; the wobble speed rises with it (one knob, never gated)
- K6 Position — horizontal pan, -100..100% = -512..+511 px about the measured centre
- S7 Fill — Outline / Solid (eyes and mouth filled)
- S8 Eyes — Closed / Open
- S9 Noise — Smooth / Streaks
- S10 Invert — Off / On: black lines (and fills) on a palette-colour field; over Video it is black ink
- S11 Background — Black / Video
- P12 Head Size — natural size .. ~95% of screen height

## Presets
Neutral, Happy, Angry, Surprised, Solid Fill (magenta, filled, half size),
Static (1 px green, 75% streaky noise), Big Head (amber 5 px, max size),
Video Ink (black filled ink over video, panned right). Hand-written — worth
re-capturing from the hardware once the look is dialled in. (The old presets
had S8 Eyes = Closed; they are Open now.)

## Status
**v1.0.0 FINAL, HW-validated, signed off 2026-10-09.** Open ideas (not planned): S8 Open/Blink animation, K6 Tilt rotation, SD shrink so SD stops overrunning.

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

## Known limits
- SD (858 clk/line) cannot hold the mesh at natural size; the rasteriser
  overruns and SD shows a cropped, degraded face. HD is the target.
- Worst-case HD line budget ≈ 1950 of 2200 cycles (7 px lines, full size, incl. the 32-cycle wipe).
- Mesh ceilings: 256 edges, max |x| ≤ 255 at scale 1, stamp ≤ 127.

## Build notes
- Build with router1: `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron bishop`.
  router2 left HD configs 0.5–1.9 MHz over target and seed-fragile
  (HD Analog needed seed 4 on the Aug 30 build).
- Seeds / Fmax for the current build:

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
