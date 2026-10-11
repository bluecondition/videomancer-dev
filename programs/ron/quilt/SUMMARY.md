# QUILT (v1.0.0 — recursive-subdivision improv quilt)

**STATUS: FINAL except presets — signed off by Ron 2026-10-11** ("Quilt can be
marked as finalized and ready"). v1.0.0 = the v0.6.0 bitstreams (byte-identical)
repacked with the version bump. The 8 presets are hand-written placeholders —
capture real ones from the device with `./capture_preset.sh` and rebuild. No
hardware notes were recorded for v0.2–v1.0 (v0.1 was HW-tested in June); the
flash checklist at the bottom is what to look at first if anything seems off.

A generative patchwork: every pixel marches a fixed-depth pipeline (coarse
grid cell → optional off-grid overlap → 4 levels of hash-driven BSP splits →
leaf colour + embossed texture). No per-pixel multiply. Full history in the
Claude memory file `quilt_milestone.md`.

## Controls

| | Control | Notes |
|---|---|---|
| K1 | Grid Scale | Large / Big / Medium / Small (256 / 128 / 64 / 32 px cells) |
| K2 | Seed | re-randomises the whole layout |
| K3 | Evolve | bipolar: centre freezes the quilt as it stands; right = one random piece at a time changes colour OR texture; left = the change snakes from piece to touching piece along a Hilbert path (new path each lap) |
| K4 | Threshold | Luma Mod: 0 = off; above, input luma drives piecing density and cells darker than the threshold go black. Needs a video input |
| K5 | Palette Tint | xor-permutes the palette (0–15) |
| K6 | Texture Scale | Fine / Small / Medium / Coarse (16–128 px periods) |
| S7 | Seams | black seam lines on / off |
| S8 | Relief | Flat (soft emboss) / Puffed (deep emboss + patches shaded darker toward their seams) |
| S9 | Grid | Square / Not Square (jittered, overlapping, 1–2 cell rectangles) |
| S10 | Piecing | Crazy (random split axis) / Log Cabin (alternating, concentric frames) |
| S11 | Palette | Fabric / Rainbow |
| P12 | Density | master recursion control |

## v0.6.0 (2026-10-11) — Snake walks piece to piece; Puff

* **Snake (K3 left) is now piece-centred**, at Ron's request ("should be
  centered on individual pieces rather than going down the screen line by
  line"). The slot is the Hilbert-curve index of the leaf's CENTRE on a 16 px
  grid; the screen is two 64×64-cell squares side by side (rows below 1024
  fold into the last row), each walked by a Hilbert curve, joined edge to
  edge. Consecutive slots are adjacent cells, so the change hops from piece to
  touching piece and fills small regions before moving on. The lap count picks
  one of 8 orientation/order variants (all with an adjacent junction), so each
  lap takes a different path. Two pipeline stages (S10h, S10h2; LATENCY 22 →
  24); the state machine and all 8 variants were checked against a reference
  Hilbert implementation (0 mismatches, every step adjacent). The curve fills
  the ring, so the v0.5 band wrap is gone. SD only uses part of the left
  square, so SD Snake still pauses between passes.
* **Puff, folded into S8 Relief (Flat / Puffed)** — Ron's pick. Puffed = the
  old Deep emboss plus luma pulled down toward each patch's edges, quadratic
  profile (150·(t/15)², 150 at the edge), radius by grid scale (8 / 16 / 16 /
  32 px) read as fixed slices of the edge distance. Flat = the old Soft emboss.
  Emboss and puff are one signed delta built in S11; S12 is a single add plus
  the 64..880 clamp. The seam test is now a 1-bit flag from S10b.

## v0.5.0 (2026-10-10) — finalisation pass (built, not flashed)

* **Luma Mod works on hardware.** `program_type` is now `"processing"` (it was
  `"synthesis"`, so the input read 0 and any K4 above 0 blacked the screen).
  Processing brings two contracts with it:
  * **Generated avid** (cubist/aurora recipe): start 3–4 clocks after the
    source avid rise at the voted hsync parity, end on the per-field width.
    `pixel_x` counts it; the output avid is it, delayed by LATENCY.
  * **Blanking gate**: neutral 64/512/512 outside the generated avid.
* **Luma Mod rebuilt as a per-frame block map** (was: one luma pixel per cell,
  sampled on the cell row's top line). 32×32 block averages of the input luma
  (a box across 32 pixels, an IIR down the lines that stays a true mean in a
  partial bottom block row) go into one bank of a 2-bank map (4096×8, 8 EBR,
  index row·60 + col); the picture reads the previous frame's bank, so every
  reader of a block sees one value all frame. A cell takes its centre block.
  Fixes: single-pixel noise (piece popping / hide flicker on live video), the
  half-cell vertical offset, the half-cell top-line glitch, and — the big one
  — **Not-Square rectangles fractured along the hidden grid** because each
  pixel used its own grid cell's luma. Each candidate rectangle now carries
  its own cell's dens/hide (prefetched per column: own row + up row, the left
  pair is the previous column's), and S1c picks the winner's.
* **Rows anchored to the first active line** (was: counted from vsync, ~41
  blank rows at 1080p). The quilt sits ~41 px higher than v0.4; the top row of
  cells is now whole.
* **Luma range**: shadows and hidden cells floor at 64 (were 0 — below black);
  highlights cap at 880 (the brightest palette entry; were up to ~950).
* **K3 centre is a true freeze** — no longer snaps every evolved piece back to
  its base colour, and the Random/Snake choice is held at centre.
* **Snake no longer idles ~45% of each lap**: in Snake mode the cursor wraps at
  the measured last band with a carry into the lap count. (Superseded in v0.6
  by the Hilbert key, which fills the ring.)
* **S1c argmax without carry chains** (the HD HDMI critical path after the
  map went in): the coverage-free pairwise priority compares are registered in
  S1b; S1c combines them with the coverage bits in single LUTs. Exhaustively
  checked against the old pairwise tree (0 mismatches). Per-frame cell sizes
  are registered instead of computed in S1b.
* TOML: categories `Pattern, Craft, Demo` ("Generative" is not a valid
  category); header comments and descriptions brought up to date; the Luma
  Mod preset uses Medium cells so a silhouette reads.

## Build

Build with router1 (router2 never closed Quilt):
`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron quilt`

v0.6.0 (2026-10-11, router1, **seed 1 on all 6 configs**), 6723–6741/7680 LC
(88%), 9/32 EBR (8 luma map + 1 line IIR):

| Config | Fmax (MHz) | Target |
|---|---|---|
| HD Analog | 80.17 | 74.25 |
| HD HDMI | 80.62 | 74.25 |
| HD Dual | 76.51 | 74.25 |
| SD Analog | 81.82 | 27 |
| SD HDMI | 79.71 | 27 |
| SD Dual | 78.19 | 27 |

v0.5.0 (same flow, seed 1 everywhere, 86% LC): HD Analog 84.48, HD HDMI
81.59, HD Dual 81.95. Before the S1c rewrite the same design ran HD HDMI 72.7–77.5 across seeds 1–4
and HD Dual needed seed 4. v0.4 (for reference): HD Analog seed 6, HD Dual
seed 8, ~78% LC.

## Flash checklist (v0.5.0)

1. Default look, P12 sweep, S9 Off-Grid, S10 Log Cabin, S8 Relief.
2. K3 right (Random) and left (Snake): Snake should hop piece to touching
   piece, meandering through regions (not sweeping rows), with no long pause;
   returning K3 to centre should freeze, not reset.
3. S8 Puffed vs Flat: patches should read as padded (darker toward seams),
   with seams on and off, across all four grid scales.
4. With a video input: K4 up from 0 — patches should follow the picture's
   brightness and dark areas go black, stable (no flicker or popping); try it
   with S9 Not Square (big rectangles should stay whole).
5. No thin coloured lines across flat patches (generated-avid fix), no colour
   cast on analog out (blanking gate).
6. Brightest fabric (cream/yellow) highlights no longer blow out to white.
