# Ticker — camera video through a sample-and-hold mosaic (Heirloom #20)

**v0.5.2 (2026-11-15): K6 camera pass, built, NOT flashed** (see [K6 camera pass](#k6-camera-pass-2026-11-15-v052)).
**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Source: `photos/WIN_20230319_01_27_56_Pro.jpg`.
v0.1.0 (hashed bar columns scrambled sideways) was judged too abstract. This version runs the camera
through the photo's processing, and Osc mode substitutes a generator with the photo's statistics.
Working files: `.heirloom_work/rb_ticker/` (`notes.md`, `proto2.py`, `imodel.py`, `sheet.png`, `cmp_final.png`).

## K6 camera pass (2026-11-15, v0.5.2)
- **K6 = Camera** (Ron's rule for the Heirloom / Kinetic / Patchbay sets: K6 is the camera strength, default
  50 %). Ticker is a picture OF the camera (a sample-and-hold mosaic), so K6 sets how strongly the camera shows.
- **Law:** the source value (camera L, or the Osc greys and highlights) becomes
  320 + (v − 320) · k / 512 before it is written to the cell RAM (320 L = a mid grey), clamped to black .. 1.0.
  One 10 × 9-bit product stage and one add / clamp stage; they took v0.5's two output pad stages, so the latency
  stays 36. The Osc wedge (its stripes and dashes) is exempt, so the wedge stays when the camera fades.
  - 0 % = every cell flat grey (B halves darker, so the cell grid still shows): the camera faded out to a
    neutral ground. Osc keeps its dark wedge on the grey.
  - 25 % = half the v0.5 contrast. 50 % (default) = exactly the v0.5 look (gain 1, nothing clamps).
  - 100 % = 2× contrast: darks crush to black, many more cells reach the cyan window and white; the selfie
    still reads (no washout).
  - S11 Osc goes through the same law (wedge exempt).
- **K6's old function (Shadow)** moved to **S8**, the weakest control (Roll / Lock left a still unchanged on the
  control sheet; phase 2's notes say the same): S8 Shadow = Photo (the old K6 at 512) / Deep (the old K6 at
  800: wedge 1.56×, Video crush below 0.20 L). Rows is fixed at Roll. No preset used K6 or S8.
- Working files: `.heirloom_work/rb_ticker/k6/` (`k6sheet.png` = K6 0 / 256 / 512 / 1023, Osc at 512, Osc + S8
  Deep, Osc at 0; 720p dec 2 with the selfie).
- **Build** (2026-11-15, `.kinetic_work/tools/k6build.sh heirloom ticker`): **router1**; HD Analog s1 95.38, SD
  Analog s1 88.21, HD HDMI s1 81.56, SD HDMI s1 85.98, **HD Dual seed 2** 94.07 (router1 stalled on seed 1 and
  timed out), SD Dual s1 88.92 MHz. Worst HD 81.56. 6,831–6,872 / 7,680 LC (89 %), 2 / 32 EBR.
  `out/rev_b/ron/heirloom/ticker.vmprog` (400,918 bytes); the packed config carries the new labels. Not flashed.
  - The S8 Shadow constants are per-state constants, not shift-adds on the two-valued setting: as adds, its bits
    were copies of the one switch flag and gave 3 SB_CARRY cells with I0 == I1 (the router1 self-add hang, which
    stalled HD Dual seed 1 on the first build). Fixed (0 now, dupin.py), and 75 LUTs smaller. HD Dual seed 1
    still stalled once afterwards; the netlist keeps 4 pre-existing LUTs with a duplicated input (sign bits of
    s_incv / s_sp / s_tba / s_trib), so stay on router1 with seed retries, or router2.
- The pending diag copy (`.heirloom_work/rb_ticker/diag/ticker_diag/`, v0.6.1) got the same design with S9 Edges
  as its weak control (see its SUMMARY); router1, all seed 1, worst HD 81.96 MHz, 7,174 LC, 7 EBR.

## Reading of the photo (measured)
The frame is camera video through a sample-and-hold, not a generated pattern:
- **Horizontal S&H, locked to H.** Every cyan blob centre sits on one grid, pitch **45.91 px**, phase 25.8
  (concentration 0.95 over 188 blobs; white blobs 0.87). Rising edges cluster at cell phase −12; falling
  edges spread over +3..+22. Each cell is a **24.5 px "A" half**, showing the sample at full level, and a
  **21.4 px "B" half**, holding an independent sample at **black + 0.44 (v − black)**. B never keys, and
  it is uncorrelated with both neighbouring A halves (the regression residual equals its own spread).
- **Line hold on two free-running vertical clocks.** Every line inside a row is identical. The row edges
  are the union of clock 1 (period **19.76 lines**, phase 14) and clock 2 (**23.28 lines**, phase 38).
  Where the two nearly coincide they leave 1–8-line slivers. About half the cells change at each tick.
- **Held bands.** There are no ticks over 587–643, 745–832 and 902–1001, and one 27-line row at 430. A
  triangle wave (period 168 lines, peak at 610) compared with a falling ramp
  (θ = 0.45 − 0.0015 (y − 610)) fits all four (429–458, 588–644, 746–825, 904–1003). The bottom of the
  frame becomes tall vertical bars.
- **Luma window key.** Plateau luma runs continuously from 0.07 to 0.62, is **empty from 0.63 to 0.93**,
  and white sits at 0.94–1.0. The cyan (rgb 24,236,238; hw10 Y 657 U 139 V 641; plateau Y′ 0.714) fills
  exactly that gap, in A halves only.
- **Soft edges and fringe.** Horizontal transitions take 15 px from 10 to 90 % (a 16-px box); row edges
  are hard. Cr = 0.083 (Y[x+4] − Y[x−4]) (correlation 0.82): pink on rising edges, green on falling.
  The greys are warm: Cr +15 → +8 and Cb −13 → −4 (10-bit) from black to 0.55.
- **Content statistics** (A cells outside the dark region): dark 27 %, grey 52 %, cyan 11 %, white 10 %.
  Cyan and white cells are **never horizontally adjacent**; they sit 2, 3 or 5 cells apart (the row
  spectrum peaks at 2.4 cells ≈ 77 px). The 8-cell mean variance is 0.11 (close to iid).
- **The dark region** (the camera's dark object): an inverted triangle, x 370..1560 at the top with its
  apex near (600, 555). Alternate rows read 0.08 / 0.12 luma, and two columns of dim dashes (x ≈ 745,
  1125) run down the lighter rows.

## What it does
1. **Beam.** A 16-bit cell phase (one cell = 65536) steps by `inc = 34.1 · cells` per virtual pixel.
   The grid is anchored at the screen centre, with a 17-px correction for the edge boxes' pipeline lag,
   so it lands on the photo's phase. A is where the phase < 0.534. Every half cell is an entry (A even,
   B odd).
2. **Rows.** Two 20-bit vertical clocks step once per line, and a carry is a tick. A triangle hold wave
   and a Q8 ramp suppress ticks (P12). S10 drops clock 2.
3. **Sampling.** On a tick line, the first pixel of each half cell takes the source value and writes it
   into a 256-entry cell RAM. Every other line reads the RAM back (the line hold).
   - Video: camera luma in photo-luma units (1024 = 1.0), averaged over 4 px. Darks below the Shadow
     threshold are crushed to black.
   - Osc: a 24-bit add/xor hash of (entry, j), where j = (line + boil + bit-reversed entry) / 32, so
     values re-roll on staggered 32-line steps.
     - Greys come from a 256-entry quantile ROM fitted to the photo.
     - Highlights (0.63..0.95) appear where a 77-px triangle oscillator is near its peak AND the hash
       passes.
     - The dark wedge (ragged edge, row stripes, dashes) goes on top.
4. **Display.**
   - B halves: black + gain (K5) × (v − black).
   - A halves: v in [kc, kc + 0.17) → cyan (or magenta), above that → white (0.83), otherwise v.
   - Then a 16-px luma box and an 8-px key box (S9 bypasses both), the Cr fringe, the warm tint and the
     key-colour mix, followed by the shell's output conditioning and blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Cells | the 46-px cell grid | cells across the frame, 12..100 (41.8 = the photo); centre-anchored |
| K2 Rows | the ~20-line rows | vertical clock rate, 8..263 rows per frame (54.6 = the photo); clock 2 runs at 0.85× |
| K3 Cyan | the cyan picks | key window position: low = cyan floods the greys, 50 % = the photo, high = only the brightest cells turn cyan and white disappears |
| K4 Speed | the drift of the patch | bipolar, still in the centre ±8: rows roll, the held bands drift, Osc highlights hop sideways and values boil |
| K5 Gaps | the dark B half of every cell | B gain 0 (black seams) .. 0.87 (seamless); 0.44 = the photo |
| K6 Camera | the camera itself | camera strength (K6 camera pass, v0.5.2): the source's contrast about a mid grey before the cell hold. 0 % = flat grey cells (the camera faded out; the Osc wedge stays), 50 % (default) = the v0.5 look, 100 % = 2× contrast (more black, cyan and white cells) |
| S7 Picks | the key colour | Cyan / Magenta |
| S8 Shadow | the dark triangle | the old K6 Shadow (v0.5.2): Photo (the old K6 at 512: the photo's wedge in Osc, crush below 0.15 in Video) / Deep (the old K6 at 800: a 1.56× wedge, crush below 0.20). The old Rows switch is fixed at Roll |
| S9 Edges | the soft analog edges | Soft / Hard blocks |
| S10 Clocks | the row slivers | Two clocks (the photo) / One (regular rows) |
| S11 Source | the camera | Video (default) / Osc |
| **P12 Hold** | the tall held bars | 0 = none (fine rows everywhere); 50 % = the photo's three bands; 100 % = the lower frame melts into tall bars under a few bands of rows |

P12, K1, K2, K3, K5 and K6 glide (1/8 of the gap per field, ±4 deadband). The control sheet is
`.heirloom_work/rb_ticker/sheet.png`; every control changes the picture.

## Video
The camera goes through exactly the processing in the photo:
- the line hold on the two clocks, with the held bands;
- the H-locked half cells, with the B attenuation;
- the cyan/white window key;
- the soft edges, fringe and warm tint.

Its chroma is dropped, because the photo is luma plus the key colour. Osc substitutes the generator
described above. Every control acts in both modes; S8 Shadow (K6 until v0.5.1) is the wedge in Osc and the dark-crush in Video.

## Engine
- **Zone sequencer** (vblank = "no avid for 1024 clocks").
  - It runs the glides, the switch latches and the per-field phase motion.
  - One serial shift-add multiplier computes k2², rows × the timing scale, the hold-ramp slope and
    (W/2 − 17) × inc.
  - Every other per-field constant is a shift-add register at most two adders deep.
- **Per-line engine** (5 clocks after the source avid falls): clock carries, the hold triangle against
  the ramp, the tick, row parity, the highlight phase and the wedge edges. Its results are committed at
  the next avid rise.
- **Pixel path** (36 stages), in order:
  - beam → half cell / entry → wedge distances and stagger;
  - 5-stage hash → grey ROM (EBR + fabric register) → source value → cell RAM / hold;
  - key + B gain (the multiply split hi/lo) → luma scale → boxes → fringe and tint → key mix;
  - output conditioning → gate.
- **2 EBR:** the cell RAM (256×16, `no_rw_check`: tick lines only write, other lines only read) and the
  grey ROM.
- **Contracts:**
  - generated avid;
  - virtual 1080-line coordinates (timing-ID steps; centred grid and wedge — 720p renders the same picture);
  - one 36-clock latency in every mode;
  - output conditioning (luma [1 2 1] + 13/16, chroma 9-tap) before the blanking gate;
  - colours in the hardware convention (u = Cr).

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Also gated: the mid-pipe luma 16-box and the key-flag box (up to ~8 px change at the line edges). Post-route Fmax: HD Analog s1 74.25, SD Analog s1 79.68, HD HDMI s1 80.36, SD HDMI s1 86.50, HD Dual s1 75.67, SD Dual s1 82.07 MHz; 6832-6865 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-21, router2, not flashed.**
  - **Build seeds: SD Analog = seed 2** (seed 1 stalled the router) and **HD HDMI = seed 2** (seed 1
    missed timing). The other four are seed 1.
  - Post-route Fmax: HD Analog 77.19, HD HDMI 78.06, HD Dual 87.96, SD Analog 81.51, SD HDMI 81.51,
    SD Dual 80.82 MHz.
  - Fit: 6830–6870 / 7680 LC (89 %), 2/32 EBR. All six bitstreams pass iceunpack.
  - The earlier build of the same pipeline, before the grid-phase constant changed, closed 6/6 on
    seed 1 (HD 76.5–83.7 MHz).
- HD margins are about 4 %. If a later version misses, try `NEXTPNR_ROUTER="--router router1"` before
  restructuring.
- GHDL renders at 1080p decimation 1, defaults, in exact hardware hues (raw2hw):
  - Osc class mix dark/grey/cyan/white 0.41/0.43/0.08/0.08 (photo 0.40/0.44/0.09/0.08).
  - The cyan grid phase is 26.1 px (photo 26.1).
  - The held bands are 429–458, 588–645, 746–825 and 904–995.
  - Raw cyan output: Y 645 U 149 V 636 (photo 657 / 139 / 641).
- Motion, at 720p, 6 fields apart: the rows roll, highlights hop and the held bars resample, with
  about 14 % of pixels changing per 0.1 s at the default Speed.

## Deviations from the photo
- Whites are 0.83 of full scale, not the photo's 0.96: the shell's 13/16 output scale (the HD HDMI bar
  fix) caps them. Greys and cyan are pre-compensated and match.
- Osc content is a statistical substitute: it has the same class mix, bright-cell spacing, row
  behaviour and dark wedge, but a different arrangement of cells. In the photo the wedge edge is a
  cell staircase from the camera; Osc draws a ragged diagonal.
- In the photo the clocks drift slightly (rows of 22–27 lines in the middle third); here both clocks
  run at a fixed rate.
- The grey ROM is an inline constant in `ticker.vhd`, not a build-hook package, because there is only
  one 256-entry table.
