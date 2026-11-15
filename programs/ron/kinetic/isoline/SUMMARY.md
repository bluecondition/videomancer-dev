# Isoline (Kinetic)

**v0.1.1 (K6 camera pass) built 2026-11-15, not flashed** (v0.1.0 built 2026-11-14). Flash `out/rev_b/ron/kinetic/isoline.vmprog`; see the last section.

## What it draws

Metaballs drawn as a topographic map: hypsometric tints and contour lines, islands merging through
saddles. A lava lamp as cartography. The default frame is an archipelago in the Ocean palette: four soft
hills strung along a wide ellipse, their flanks fused into one island, drawn as flat bands of blue, pale
cyan and cream on navy, every band edge a clean anti-aliased contour. The hills travel round the ellipse
(about 11 s a lap at the default Rate) and each one swells and shrinks, so necks open and close and the
contours pass through saddles as islands join and part.

## Element and state

Element: a blob (four of them). State: its place on the path, path(phase + Spread * j), and its radius.

    F(x, y) = sum_j w(q_j) + source        q_j = ((x - cx_j)^2 + (y - cy_j)^2) / R_j^2
    w(q)    = (1 - q)^2 for q < 1, else 0  (a smooth cap with a finite footprint; peak 1)
    PHI     = bands * F                    contour c where PHI = c; the tint is floor(PHI)
    1 / R_j^2 = (1 / R^2) * (1 +- 0.62 Depth^2 sin(4 theta_j + 0.38 j))      the pulse

Tint of band b: 0 = ground, then A, B, accent, B, A, B, accent, ... (the palette's own four colours, flat).
Coverage at an edge comes from the distance to the nearest contour, |PHI - c| / |grad PHI|, with the gradient
from finite differences (previous pixel, line above), so tint edges and lines are anti-aliased per real
pixel in every video mode (9 levels). Lines are 2 pixels wide, index lines 3 to 5.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | path size: half-axes 0 .. 767 x 511 virtual px (3:2), glided. 0 = every blob at the centre |
| K2 | Shape | contour interval: 2 .. 32 bands per blob height, exponential. Low = a few broad flat tints, high = many fine bands (they fade to their mean before they alias) |
| K3 | Colour | the 16 shared Kinetic palettes (ground, A, B, accent), with hysteresis; default Ocean. (Was K6 until the K6 camera pass; the old K3 Depth, the radial pulsing, is fixed at its default 600: radius x 0.78 .. 1.6 at the top would be Depth 100 %) |
| K4 | Rate | travel along the path, bipolar, centre = still; exponential, 60 s per lap just off the deadband to 2 laps per second |
| K5 | Spread | phase step per blob, square law, 0 .. 1/2 lap. 0 = all four stacked into one tall hill; the default strings them out over 3/5 of the path; the top puts them in two opposite pairs |
| K6 | Camera | how strongly the picture (or the Osc) raises the terrain, linear: 0 % = none (the black-input picture), 25 % = v0.1.0's fixed strength, 50 % (default) = twice it, 100 % = about four times. See "K6 camera pass" |
| S7 | Wave | Smooth / Snap: the blobs hop between 8 stations on the path, dwelling 3/4 of the time and moving on a sine ease |
| S8 | Field | Orbit (the ellipse round the centre) / Drift (a figure of eight, so the blobs cross in the middle and merge there) |
| S9 | Look | Tints (filled bands) / Lines (contours alone, colour A on the ground) |
| S10 | Index | Off / On: every fourth contour, starting with the coastline, drawn heavier in the accent colour (over the tints too) |
| S11 | Source | Video / Osc |
| P12 | MERGE | the headline: blob radius 128 .. 664 virtual px, exponential, glided. 0 % = four small separate islands, about 50 % = the default archipelago, 100 % = one continent whose contours wrap the whole group, with saddle contours inside |

## Video

`program_type = "processing"`. The picture's luma adds to F before the contours are cut, so the bright areas
of the picture rise as terrain and bend the coastlines: luma, cored at about 30 % (a dim background adds
nothing), -> horizontal 1/32 IIR (reloaded at each line start) -> vertical 1/32 IIR in a half-resolution
line store (16-bit state) -> half gain. White adds about 0.3 of a blob's height (1.5 contours at the default
interval). With a black input it adds nothing and the blobs alone run.
Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice at the same point, uncored and at
twice the camera's gain (up to 0.87 of a blob height, 4 contours), so a lattice of diamond terraces drifts
under the islands. The source field lags the picture a little to the right and below (the two IIRs).

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): knob glides (K1, K2, K3, K5, P12); one serial
  16-step multiplier: 5 global slots (radius octaves through a 2^(i/256) ROM, Depth^2, Spread^2, rate), then
  12 slots per blob: centre (two sines from a quarter-wave ROM, 4096 points per lap), pulse, 1 / R_j^2, and
  the five DDA constants Q (q at the pre-pixel of line 0), E (its step per line), EE (the step's step), D0
  (the step per pixel at the pre-pixel), DD (its step). The scale 1 / R_j^2 is folded into all of them, so
  the distance needs no per-pixel or per-line multiply. They go into a block-RAM register file. About 1,500
  zone clocks. The palette (4 colours x 3, 8 bits) is read from the ROM.
- **Per line** (in the horizontal blanking, 32 clocks): the register file is walked: Q += E, E += EE are
  written back and each blob's beam is loaded with the next line's pre-pixel values through its own adders.
- **Per pixel** (29 stages, latency 36 in every mode): per blob b_q += b_d, b_d += c_dd (35 bits, 26
  fraction bits) -> 1 - q -> (1 - q)^2 from a 256-entry square ROM plus an 8 x 6 cross-term multiplier ->
  sum of four + source -> PHI = F x bands (16 x 9 multiplier, split 4 + 5) -> gradient (previous pixel; the
  line above from a quarter-resolution line store) -> octagonal magnitude -> 5-step divider (distance in
  1/8 pixel) -> coverage and the two palette colours -> shift-add blend -> output conditioning (chroma
  [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]/4 at the same group delay, luma 13/16 toward
  black) -> blanking gate.
- Chassis from riley: timing-ID virtual-1080 steps (SD shows the centre 4:3), generated avid, width
  measurement, glides, rate deadband, palette hysteresis, source oscillator, output conditioning.
  Colours in the hardware convention (U register = Cr).

## Status

**Shipped vmprog: the ROUTER1 rebuild of 2026-11-14** (same sources). Post-route Fmax: HD Analog 76.96 MHz; SD Analog 83.15 MHz; HD HDMI 88.62 MHz; SD HDMI 80.59 MHz; HD Dual 80.85 MHz; SD Dual 81.49 MHz. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic isoline`. The router2 figures below are the first build, kept for the record; it was timing-marginal.


- v0.1.0 built 2026-11-14 through the shared queue, router2 (the builder default). Not flashed.
- Post-route Fmax (HD needs 74.25, SD 27):

  | Config | Seed | Fmax | LC |
  |---|---|---|---|
  | HD Analog | **6** (seeds 1 .. 5 missed) | **74.33 MHz** (0.1 % margin) | 7,384 |
  | HD HDMI | 1 | 76.76 MHz | 7,377 |
  | HD Dual | **3** (seeds 1, 2 missed) | 78.93 MHz | 7,386 |
  | SD Analog | 1 | 76.00 MHz | 7,385 |
  | SD HDMI | 1 | 69.21 MHz | 7,383 |
  | SD Dual | 1 | 75.71 MHz | 7,400 |

- 7,377 .. 7,400 / 7,680 LC (**96 %, over the set's 7,100 budget**), 19 / 32 block RAMs. The six bitstreams
  pass iceunpack. HD Analog closed only on the last seed and by 0.08 MHz (the last "Max frequency" line of
  that run was read from the live log: 74.33 MHz PASS), so it is run-marginal: a rebuild may need other seeds.
- Rebuild: `./build_set.sh kinetic isoline`.
- Model = RTL (GHDL, black input): pixel diff **0 of 918,720** on a full 720p60 decimation-1 frame at the
  defaults; 0 of 228,960 on a 720p60 decimation-2 frame (the top-left quarter) at the defaults with the Moss
  palette; 0 of 228,960 on a decimation-2 frame 4 fields later with Snap + Drift + Lines + Index + Vega (so
  the motion, both wave shapes, both paths, both looks and the index lines match). The GHDL frame is the
  model's shifted 4 columns right with 4 blank columns (the generated avid), as on riley. Knob values other
  than the defaults were NOT rendered in GHDL (the glides would need a sim-only copy); the control sheet is
  the integer model's. Rasters rendered in GHDL: 720p60 (full and
  decimation 2) and NTSC 480i (both fields, 720 x 243 each: pixel diff 0 of 173,988 on each); 1080i / 1080p
  were not rendered.
- The camera and Osc renders (full 720p GHDL frames, in cmp_final.png) were checked by eye only: the model
  does not mirror the source path's exact column phase, so they are not pixel-diffed.
- Renders: `.kinetic_work/rb_isoline/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion).
- If the fit has to come down: `python3 .kinetic_work/rb_isoline/gen_vhd.py ring` rewrites isoline.vhd (from body.vhd) with the chroma
  filter's 8-sample delay in a block-RAM ring instead of 160 flip-flops (about -150 LC, same samples). It
  analyses in GHDL but has been neither simulated nor built.

## Cut / deviations

- **Four blobs, not five or six**, and the tints are the palette's four flat colours cycling by band, not an
  interpolated hypsometric ramp. The first RTL (5 blobs, a 64-entry interpolated tint table, 17-level AA)
  came to 5,473 LUT / 4,319 FF standalone, far over the chip; the per-line state moved into a block-RAM
  register file, the tint table and its fill engine went, the AA dropped to 9 levels.
- Tints step ground -> A -> B -> accent and then back and forth through B and A (never back to the ground),
  so tall merged hills keep their bands.
- Orbit is a 3:2 ellipse, not a circle (it fills a 16:9 frame; SD shows it whole). Drift is one figure of
  eight shared by the blobs (path(phase + Spread * j)), not four independent Lissajous figures.
- The camera's default contribution is deliberately modest and cored: at full gain the picture's texture
  shredded the coastlines.
- The vertical gradient is sampled every fourth pixel (quarter-resolution line store) and held.
- Index line weight is set in real pixels per mode (5 virtual px at 1080p, about 5 at 720p and 6 at SD);
  plain lines are 2 real pixels in every mode (2 / 3 / 4 virtual px), so SD and 720p lines are a little heavier.
- `field_n` is passed through one register (the core ties it high on every input).
- Weak spots seen on the control sheet: Snap is a modest flip on a still frame (the blobs sit on 8 stations;
  it reads in motion); Depth 0 % against the default is subtle; Lines in the Ocean default is blue on navy,
  so it is faint until Index is on or another palette is chosen.
- 480i has 243 lines per field: fine bands (K2 high, small radius) fade to their mean sooner than at HD.

## K6 camera pass (2026-11-15), v0.1.1

- **K6 = Camera** (new, 0 .. 100 %, default 50 %): how strongly the source field raises the terrain. The gain is
  applied to the 8-bit source sample (cored camera luma, or the Osc lattice) before the two IIRs, by one 8 x 5
  multiply in its own stage: sample x (K6 >> 5) / 16 for the camera, / 8 for the Osc (the Osc keeps twice the
  camera's gain). Linear law, 32 steps:
  - 0 % = none: the black-input picture (with Osc as well);
  - 25 % (K6 256) = x 1/2 = exactly v0.1.0's fixed strength (white adds about 0.34 of a blob's height);
  - 50 % (K6 512, default) = twice v0.1.0 (white about 0.68 of a blob height, 3.4 contours at the default
    interval; the Osc up to 1.7);
  - 100 % (K6 1023) = 31 / 8 = about 3.9 x v0.1.0 (white about 1.3 blob heights; the Osc up to 3.4).
  K6 is glided in the shared glide (channel 2, the old Depth channel: 1/8 of the gap per field) with a +-4
  deadband on this channel (the others keep +-2), so pot noise cannot step the gain.
- Looks (integer-model preview with the selfie, `.kinetic_work/rb_isoline/k6/model/msheet.png`, and the GHDL sheet `.kinetic_work/rb_isoline/k6/k6sheet.png`: 720p60 decimation 2, the top-left quarter, K6 0 / 256 / 512 / 1023 and Osc at 512): 50 % bends the coastlines clearly and stays clean (a little serration where a contour
  runs along the picture's bright edges); 100 % lifts the picture's mid-grey background over the coring, so the
  sea breaks into ragged islands with the horizontal IIR's streaks: bold, still a contour map, not washed out.
  Osc at 50 % is a full lattice of diamond terraces; at 100 % dense concentric diamonds.
- **Frozen: K3 Depth at its default 600** (the pulse depth c_d10 is now the constant 600^2 >> 10 = 351; serial
  slot 1 is gone). **K6's old function, Colour, moved to K3** (same 16 palettes, hysteresis, default Ocean); the
  presets' Colour values moved with it and their old Depth values (Atolls 800, Lava Lamp 850, Continent 700) are
  dropped, so those presets pulse at the default depth.
- Fit: freezing Depth freed very little; the gain stage, the wider IIR (the sample is now Q8.2) and the camera
  deadband cost about +70 LUT standalone. To pay for it, the chroma filter's 8-sample delay moved from 160
  flip-flops to a block-RAM ring (the "ring" option noted in Status above, ported by hand: same samples, the
  8-sums' per-line resync unchanged). Standalone yosys: 4,357 LUT / 3,126 FF / 21 EBR (v0.1.0: 4,271 / 3,231 /
  19); no emulate_transparency.
- The source field now sits 2 px further right (the gain stage). Latency unchanged (36).
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic isoline`): **router1, every config on seed 1**.
  Post-route Fmax HD Analog 84.73, HD HDMI 86.74, HD Dual 90.82 MHz (worst HD 84.73, better than v0.1.0's
  76.96); SD Analog 86.58, SD HDMI 86.14, SD Dual 83.59. 7,339 .. 7,361 / 7,680 LC (96 %, slightly under
  v0.1.0), 21 / 32 EBR. Not flashed. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic isoline`.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023, set with the version bump), and on bend / element programs the camera is 25 % stronger at the top of the knob (here x 1.25 at every position, 0 stays 0).

- **Bend-type**: K6 raises the contour terrain by the camera.
- Change: the camera part only, in the existing source-sample stage: the cored camera sample (luma >> 2 less 80) x 1.25
  (v + v / 4, <= 218 of 255), so with the unchanged K6 / 32 gain every position is x 1.25 and 100 % is about 4.8 x
  v0.1.0 (was 3.9 x); x 1 is now at about 20 %. Scaling the shared gain instead would have wrapped the Osc through the
  IIRs (s_slg up to 988, s_h up to 31,616 of 32,767, the line store up to 63,232 of 65,535), so the Osc keeps its old
  range (now 1.6 x the camera's gain instead of 2 x). One 8-bit add, no new multiplier, latency unchanged.
- Look: `.kinetic_work/rb_isoline/k6b/k6b_cmp.png` (GHDL, bear, Ocean, 720p60 / 4, glide start set to 1023 in scratch
  copies): the same contour map with the coastlines pushed further (more land); clean.
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic isoline`): **router1, seed 1 except SD HDMI seed 2**. Post-route Fmax HD Analog 88.19, SD Analog 87.86, HD HDMI 78.88, SD HDMI 84.73, HD Dual 89.34, SD Dual 85.29 MHz (worst HD 78.88); 7,372 / 7,680 LC (96.0 %). Not flashed.
