# Spiroscope (v6.3 — spirograph / harmonograph drawing machine)

**SHELVED 2026-10-11** (Ron: "i think shelve spiroscope for now"), right after the v6.4 ghosts were rejected and v6.3 restored. Source and `out/rev_b/ron/spiroscope.vmprog` (v6.3) left in place, nothing committed; v6.4 source in `.spiroscope_work/`.

**v6.4 REJECTED on HW** (2026-10-11, Ron: "Unfortunately I don't like the
ghosts"). Reverted to v6.3 (Engrave on S8, Kaleido on S9); don't re-offer
ghost echoes. The v6.4 source is kept in `.spiroscope_work/spiroscope_v6_4.vhd`
(+ .toml); the build/budget notes below stay for reference.

**v6.4** (2026-10-11, Ron: "is there any way to do trails"): **ghost-echo
Trails** on S9 (replaces Kaleido), **Ghosts 1 / 3** on S8 (replaces Engrave;
it only matters with Trails on — Ron's choice).
- The ring splits into 2 or 4 slots. The generator writes the live figure into
  slot `g_cur`, which moves on every 8 fields, so the other slots hold the figure
  as it was 8 / 16 / 24 fields ago. Each slot is a whole figure (wide deltas):
  1 ghost = 2 x 2048 points (2 x 1024 thick / 720p glow), 3 ghosts = 4 x 1024.
  SD always gets 1 ghost (3 would overrun most SD rows).
- A slot's unused chunks are written empty (ymin > ymax, never hit) after
  each generation; a mode change empties the whole table once. The prescan
  walks all 256 chunks in LOGICAL order (live slot first, then ghosts by age),
  physical slot = g_cur - logical; the rewind maps back.
- Ghost look: the slot's age code replaces the top 2 bits of the chunk age
  (live 12-15, ghosts 8-11 / 4-7 / 0-3; 1 ghost = 4-7), so ghosts take the
  older colours of the K5 ramp, and the fade depth is forced to 16 (live
  81-100%, ghosts ~60 / 40 / 15%). A ghost never takes over the age of a lit
  pair (the live figure stays on top whatever the scan order); glow adds light
  only within one slot (a ghost under the live figure doesn't burn white).
- Visible at any Trace: comets leave a chain of echoes; in the top zone the
  drifting twist leaves echoes of the morphing figure; any knob move echoes.
  Side effect: a 1024-point figure's comet laps 4x faster (same chunks/field,
  as the thick modes already did 2x).
- Budget (proto/trails_budget*.py): 1 ghost fits every HD row. 3 ghosts at
  1080p fit simple figures (5:-8, the 16:-2 default) and overrun 3-23% of rows
  on busy ones (16:-15, harmonographs); the live slot always finishes, so only
  the oldest ghost loses pieces on those rows. 720p 3 ghosts overrun 20-31% on
  busy figures; SD 1 ghost 21-30%.
- Freed for it: Kaleido's D4 write transforms + x-extents, Engrave's luma
  multiplies and video luma tap.
- Presets: "Neon Mandala" -> "Neon Echo" (5:-8, glow, 3 ghosts, drifting top
  zone), "Engraving" -> "Ghost Comet" (7:-12 comet, 3 ghosts, rainbow).

**v6.3** (2026-10-11, Ron): S11 Ground = Black / **Video** instead of Paper.
The stroke's light is added over the live input (same blend as black: the
ground constant 64/512 is swapped for the input Y and U/V). The input is tapped
at S7 straight off `data_in` (no raw-video carry: 300 FFs or 2 EBR the chip
doesn't have), so the picture sits ~10 px left of the source, and the last
~10 px of each line are black where the source line has ended. Engrave keeps
working (weight from luma; no darkness inversion any more). Presets: "Paper
Rose" -> "Video Rose" (white lines over video), "Engraving" -> black ground.
7536-7549 LC (98%), 29/32 EBR.

**v6.2** (2026-10-11): Ron didn't like the v6.1 zoom ("I preferred the twist
drift and full tracing of the previous version"). The zoom was removed and
Trace's top ~12% is v6.0's again: the whole figure at once, blooming, its twist
drifting (easing home below the zone). The 8192-point dense ring stays, and is
now allowed in the top zone too. The zoom's off-screen culling, signed word
indices and centre-offset x stay (harmless, sim-tested). 7548-7585 LC (98.8%),
29/32 EBR.

**v6.1** (2026-10-11, Ron: "push it to the limit of the chip"): 8192-point
figures and a zoom.
- **Ring**: the point RAM is now 4096x16. Dense mode packs two points per word
  (4-bit deltas, 32-point chunks) for 8192 points. It's used for 1080p
  progressive, Spiro (incl. Kaleido), crisp, weight <= 3, outside the zoom zone
  (modelled worst row ~1.5k of 2200 clocks; proto/cycles2.py). Every other mode
  keeps one point per word with 8-bit deltas, so the delta-unit scaling is gone
  and no mode quantizes positions. Densities: 4096 default, 2048 thick/720p
  glow, 1024 SD/720p glow+thick.
- **Zoom**: Trace's top ~12% scales the base radius 1x -> ~2x (capped at 1023,
  the datapath width), replacing the bloom and twist drift. Chunks store x as a
  centre offset; segments off-screen in x are culled after the scan, and the
  word iterator skips words left of the screen (signed word index).
- **Fit**: 7438-7471 LC (97%), 29/32 EBR (2 below the 30-block cliff).
  Saturation is now a sign-extension check and Engrave luma is a plain slice.
- Smoke renders: 1080p dense comet (pairing continuous), 720p Kaleido at full
  zoom (clean clipping).


**v6.0** (2026-10-11): ground-up rebuild after the finalize review. v5 stored
the curve as per-scanline intervals (8 slots/row), which could never hold a
whole figure: the default 5:8 rosette needs ~6.5k pen steps (trail max 2110 =
32% of it) and, painted whole, ~13k row crossings (~180 kbit, more than the
chip's 128 kbit of BRAM) — 45% of the line dropped at 8 slots. On top of that
the 1080p29.97 HW raster hit the row-pairing branch (540-row res), the 8-bit
angle LUT put points up to 6 px off the curve, strokes at Y=1023 clipped all
colour, S11 video read `data_in` under `synthesis` (green), and the source
avid was passed through (U/V line swaps). v6 fixes all of these.

## Architecture

- **Generator (vblank, 8 clocks/point)** rebuilds the whole figure every
  field, so every knob is live and continuous — no zones, no clear/redraw, no
  eraser. Spiro = 2-arm epicycle (`X = R1 cos A + R2 cos B`,
  `Y = R1 sin A + R2 sin(B + Twist)`); Harmonograph = decaying 2-pendulum
  Lissajous (y weights swapped: Arm blends lateral -> rotary; pendulum A
  detuned one turn per swing; `env -= env>>11` per point, ~e^-2 = 0.135
  at the swing's end, shift scaled with the ring density).
  Frequencies are chosen so every figure **closes exactly on the ring**
  (spiro: a/g, b/g turns; harmonograph: a·k, b·k with k = round(18/max)).
  20-bit angles, quarter-wave cosine ROM (10-bit angle), one shared 10x10
  multiplier (4 products/point, +2 env products in harmonograph).
- **Ring** (as of v6.1): 4096x16 point RAM (16 EBR) + chunk table {x0, y0,
  y-extent as centre offsets, age} (3 EBR). 8192 points as 2x4-bit deltas per
  word (32-point chunks) for thin crisp 1080p Spiro; else 4096 / 2048 (thick)
  / 1024 (SD, 720p glow+thick) as 8-bit deltas, 16-point chunks. Pen-up
  marker dx = -8 / -128 (window end, harmonograph swing wrap).
- **Kaleido (S9)**: 8 D4 copies (mirrors + quarter turns) of the pen, each 1/8
  of the trace window; chunk y-extents for x<->y copies come from the x-extent.
- **Renderer (one row ahead)**: prescan of chunk y-extents (1-entry hit
  buffer, rewind when busy) -> vertex scan (integrate deltas) -> per-segment
  job {words, E0, |u|}: `|E0 - k|u||` = 2*distance (Q6) to the segment's line
  at pixel k; u, v = 2*direction/L from a 256x16 ROM; sign algebra folds the
  direction so the stamper only subtracts |u|. Job FIFO 2 deep (pipe stalls on
  full) -> word iterator (4-pixel words, 1/clk) -> stamper RMW with 3-deep
  forwarding into a dual-bank 512x24 line buffer: 4-bit closeness per pixel
  (max = crisp, saturating add = glow; glow adds only across different ages,
  so a polyline doesn't double at its own joints) + 4-bit age per 4:2:2 pair.
  The beam clears behind itself; short lines get a trailing clear.
- **Beam (C_L = 8)**: crisp/engrave = weight threshold with half-pixel AA
  (engrave: weight x input luma); glow = coverage + white-hot overlaps;
  palette ROM (8 colours x 16 ages, U/V-swapped BT.601, luma capped ~750)
  with comet fade by age (depth from Trace); ground black or the live input
  (S11).
  Chroma: ONE blend that carries U on even pixels and V on odd ones (exactly
  what the 4:2:2 packer keeps), with pair-averaged coverage.
- **Contracts**: processing type; generated avid (cubist/aurora parity
  recipe); 64/512/512 outside it; one latency in every mode; field work keyed
  on "avid absent 4096 clocks" (no vsync edges anywhere).

## Controls

| Ctrl | Function |
|------|----------|
| K1 | Gear A 1..16 (labelled; index-at-load fix) |
| K2 | Gear B -16..16 (negative = hypo/inside, 0 = fixed arm) |
| K3 | Arm 5..92% (glided) |
| K4 | Twist 0..360 deg: y-phase of arm B — epi <-> hypo through "3D" lattices |
| K5 | Colour: Neon, Ember, Ice, Rainbow, Gold, Aurora, Phosphor, White |
| K6 | Weight 1..8 px (also the Engrave maximum) |
| S7 | Machine: Spiro / Harmonograph |
| S8 | Engrave: line weight follows the input luma |
| S9 | Kaleido: 8 mirrored pens |
| S10 | Glow: additive light, overlaps go white-hot |
| S11 | Ground: Black / Video (strokes add light over the input) |
| P12 | Trace: comet -> whole figure (quadratic); top ~12% = whole figure, blooming, twist drifting |

Dropped from v5 (Ron chose the layout): Paint (layering), the 3rd-arm switch,
plain video background.

## Budgets (modelled in proto/)

- proto/model.py renders the maths; proto/cycles.py models the row pipeline.
  Worst rows at 4-px words / FIFO 2: 1080p crisp w3 ~1.2k of 2200 clocks;
  heavy weights/glow rely on the density halving above.
- Generation: 4096 x 8 = 33k clocks (1080p vblank ~94k, 720p ~45k).

## Fit / timing history

- First cut 9158 LC (119%). Ablations: generator ~2650 LUT, beam colour ~865,
  stamper ~650. Cuts: serial multiplier shared, per-state single carry chains,
  20-bit angles (closed figures -> no reload), offset-y chunks, 8->4-px words,
  FIFO 4->2, vertex pipe 9->6 stages, one chroma blend, age counter instead of
  a division -> 7173 LC (93%), 23/32 EBR.
- Critical paths fixed: shared glide unit (13.7 ns), beam final stage
  (hot*20 + 2 adds + clamps), row-start decision fanning out to the flush.
- hd_analog router1: seeds 1-4 = 85.5 / 77.9 / 84.8 / 89.9 MHz.

## Build v6.3 (restored 2026-10-11, router1: `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron spiroscope`)

| Config | Seed | Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 77.04 MHz | 7536 |
| SD Analog | 1 | 74.03 MHz | 7544 |
| **HD HDMI** | **2** | 76.66 MHz | 7541 (seed 1 missed) |
| SD HDMI | 1 | 77.57 MHz | 7549 (98%) |
| HD Dual | 1 | 76.58 MHz | 7540 |
| SD Dual | 1 | 76.91 MHz | 7547 |

29/32 EBR. Identical seeds/Fmax/LC to the original v6.3 build, which confirms
the restored source matches it.

## Build v6.4 — rejected, for reference (2026-10-11, router1: `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron spiroscope`)

| Config | Seed | Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 74.74 MHz (thin: +0.7%) | 7246 (94%) |
| SD Analog | 1 | 78.55 MHz | 7232 |
| HD HDMI | 1 | 79.30 MHz | 7260 |
| SD HDMI | 1 | 78.68 MHz | 7253 |
| **HD Dual** | **2** | 77.80 MHz | 7267 (seed 1 missed) |
| SD Dual | 1 | 74.54 MHz | 7248 |

v6.3 for reference: 7536-7549 LC, HD HDMI seed 2. Removing Kaleido + Engrave
freed ~290 LC net of the trails logic.

29/32 EBR. The chip is full: any new feature needs a trade. (v6.2: 7548-7585 LC,
HD Analog seed 2; v6.1: HD Dual seed 3; v6.0: 7252-7271 LC, 23 EBR, all seed 1.) HD margins are thin (~3%); at this fill nextpnr is seed-sensitive
(earlier hd_analog spread 78-90 MHz) — rebuild with the next seed rather than
restructuring if a config misses.

Smoke renders (sim-only copy with the Trace glide preset, 720p dec 1 — the
image tester's 8-line fake vblank only fits short windows / Kaleido): a 5:-8
comet with gradient + fade, and Harmonograph 2:3 + Kaleido + Glow (open D4
mandala, white-hot overlaps). The sim shows per-pixel chroma striping: that is
the U-on-even / V-on-odd blend, correct on the 4:2:2 hardware path.

## Status

v6.3 restored + rebuilt after the v6.4 ghosts were rejected (out/rev_b/ron/spiroscope.vmprog). v6.0 was liked on HW
("I like the looks of this"); defaults captured from the hardware 2026-10-11
(Gear A 16, Gear B -2, Arm 216, Twist 680, 2 px, Engrave, Trace 1023 = whole
figure + full bloom/drift; in v6.2 that's the 8192-point ring). v6.1 (zoom)
was rejected on HW. **v6.4 ghosts rejected on HW; v6.2/v6.3 not hardware-validated.** Presets are starting
points; recapture from the hardware.
