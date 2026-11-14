# VENEER — design notes

Status and build record: [SUMMARY.md](SUMMARY.md). Control cards: [CONTROLS.md](CONTROLS.md).
VENEER is the finished-program follow-up to the FLATLINE lab (`programs/ron/flatline/`,
read its DESIGN.md for the matte pipeline's history). Ron's brief (2026-11-13), from the
FLATLINE preview cards he picked: an **ink** mode (colour estimates, with the different edge
options, mattes only at the low end) and a **texture** mode (the solid colours replaced by
sets of generative organic textures, related to the colour or not).

## 1. Two builds, and why

| | `veneer` | `veneer_ink` |
|---|---|---|
| Fill | Ink (region colour, optional wash) **or** Texture (S7) | Ink (flat) |
| Lines | none | Outline + Sketch + Contour, three independent layers |
| Build constants | `C_HAS_TEX` | `C_HAS_OUTLINE`, `C_HAS_SKETCH`, `C_HAS_CANNY` |

One source (`veneer.vhd`, master = the texture build), `variants.py` stamps the constants
and generates both TOMLs. They are two bitstreams because the measured costs do not add up
to one chip (standalone yosys, LUT4 + carry + FF; routed LC is about 0.94 x that + the core's
1137 LC share already included in the routed figures):

| Piece | LUT + carry + FF | EBR | share of the HX4K |
|---|---|---|---|
| Matte pipeline, no lines, no texture (with the temporal vote; ~50 less without) | 4338 | 21 | 56% (routed with the core: ~63%) |
| Texture: noise fields (two octaves, node engine) | +1292 | +3 | 17% |
| Texture: shaping, colour ramp, relief | +1361 | 0 | 18% |
| Outline + ink compositing | +435 | +1 | 6% |
| Sketch (DoG) | +586 | +3 | 8% |
| Contour (Canny-lite) | +729 | +3 | 9% |
| **veneer** total | 6991 | 24 | routed 6552-6583 LC (85-86%) |
| **veneer_ink** total | 6131 | 28 | routed 6022-6038 LC (78-79%) |

Everything in one bitstream would be 8700+ (113%); the first complete version, with a value-noise
engine and a Scale control, synthesized to 126%.

The split keeps both fills together (they share everything but the line layers) and puts
all three line layers in the build that has room for them.

## 2. Matte pipeline (both builds; H domain = one step per pixel pair)

"Stage k holds half-res column m during step m+k". Latency 84 clocks in every mode.

| Stage | What |
|---|---|
| 0 | pair average of two pixels (Y, U, V to 8 bits) |
| 1 | newest luma column, rows r-4..r (4 line buffers); quarter-res chroma sample |
| 2..5 | **Smoother**: 3x3 binomial `g1` of rows r-3..r-1 (the fine scale of the Sketch layer, so it is free); through the delay ring to stage 17 |
| 2..8 | chroma class: 2x2 quarter-res box (this row + one stored row), 8 hue sectors x 3 saturations |
| 18..21 | **Quantizer**: bin = thresholds passed; within `C_HYST` = 1 code of a threshold the neighbouring bin is a candidate and the left neighbour's label wins if it is one. Label = {lum3, hue3, sat2} |
| 22..26 | **Cleanup**, horizontal: 5-sample mode of the luma bin and of the chroma class (the minimum-run-length filter) |
| 27..29 | **Cleanup**, vertical: 5-row mode. Rows 3 and 4 of the window are already-cleaned rows (the label line buffer for row 3 is written with the cleaned label), so the outline has the exact row above and the vote inherits from above |
| 30..37 | label delay; bin centroid luma; set ROM lookups (recipe at 33, colours at 36); ink layers; hand-off |

Thresholds (per frame, vblank): Fixed = equal steps; Adaptive = equal-population placement
in one pass over last frame's 64-bin histogram, low-passed 1/4 per frame.

## 3. Texture machinery (`veneer` build; F domain, full pixel rate)

Label independent fields, then per-region shaping:

- **Two noise octaves**, n1 (nodes 64 px apart) and n2 (16 px). Every node is its own
  vertical random process: a two-pole filter of random bytes `R[(node + offset(line)) mod 256]`
  (R = 256 random words in one EBR), state {s1, s2} per node in a 256x32 RAM, advanced once per
  line just ahead of the beam. No lattice rows exist; the node grid is sheared one pixel every
  other line so there are no vertical columns either. Between nodes: a linear DDA and one
  pixel-rate pole. Stochastic rounding (random bits in the bits the shift drops) keeps the poles
  alive. The filters run on from frame to frame, so the field is static; K5 drifts octave 2.
- **Line start**: three nodes per octave are fetched in blanking; after the second the DDA is
  rolled forward to its phase inside the first cell (8-pixel strides for octave 1), so no
  multiplier is needed.
- **Shaping** (T1..T4): `phase = (n1 << g1) + (n2 << g2) + (X << sx) +- (Y << sy)`, then one of
  eight shapes (direct, bands, thin line, camouflage, veins, blobs, zebra, terraces), all
  lookup-style logic. The recipe (16 bits) comes from the set ROM by (set, slot); slot = the
  region's luma bin, or a mix of bin, hue and saturation in Patchwork.
- **Colour** (T5..T8): two palette colours A, B per (set, slot) as (hue16, saturation) codes;
  chroma = A + m/4 x (B - A) with m from a smoothed copy of the texture value (so chroma never
  swings at pixel rate: the HD HDMI bar trigger); luma = the bin centroid + (t - 128 +- relief) x
  Depth / 16. **Value always comes from the picture**, which is why the image stays readable.
- **Ink fill** in this build is the same path with the ramp off and a quarter of the depth
  (a wash); Depth 0 is the flat matte.

`tables.py` holds the ROM contents (recipes, sets, palettes, the random table) and rewrites the
constant block of the VHDL; `model.py` is an integer model of the fill that mirrors the RTL
(the simulated noise octaves correlate 0.99 with it) plus an approximate matte model, for
trying recipes at true 1080p in seconds.

## 4. Line layers (`veneer_ink` build)

- **Outline**: the cleaned label differs from the left / right neighbour or the cleaned row
  above; footprint thin / 2x2 / 3x3 from a two-line bit history.
- **Sketch**: DoG of the 3x3 and 5x5 binomials in 1/16 code; ink on the dark side where
  -DoG passes the K3 threshold (6.0 down to 1.6 codes), alpha over the next 2 codes.
- **Contour**: Canny-lite (Sobel, |gx|+|gy| x K5 gain to 4 bits, NMS along the quantized
  direction, hysteresis from the left and the row above).
- The three alphas share one 8-bit ink history line buffer and the strongest wins, so any
  combination works. Each layer is off at the bottom of its knob.

## 5. What changed from the request and from FLATLINE, and why

- **Two bitstreams, not two modes in one** (section 1).
- **No Scale control for the textures.** A variable node spacing cost ~200 LUT/FF in barrel
  shifters plus a serial multiplier for the line start; with it the texture build was 92-98%.
  Nodes are fixed at 64 / 16 px. Detail (S11) doubles the band density instead.
- **One smoother.** Median and Kuwahara mattes differ in 7% of pixels and look the same; both
  are replaced by the 3x3 binomial (-740 LUT/FF against the median). Quantization and the
  mode filters re-sharpen what the blur softens.
- **Mode filters work on {luma bin} and {chroma class}** (two symbols, not three): -250.
- **Quantizer hysteresis from two subtractions** around the bin's own thresholds instead of
  three comparator sets: -230.
- **The temporal vote is compiled out and the hysteresis band is 1 code, not 3.** The first
  full-resolution renders showed both on a face (a smooth gradient crossing several
  thresholds): the 128x64-cell temporal vote drew rectangular blocks, and the 3-code
  left-neighbour vote dragged level edges sideways into flat-bottomed shapes. FLATLINE's
  cards had the temporal vote off and fewer levels on the face, so neither showed there.
  Both are constants (`C_HAS_TEMPORAL`, `C_HYST`) to revisit if region edges boil on real,
  noisy video: that is the first thing to watch on hardware.
- **No bilateral, soft quantization, drop shadow, wipe, lines-only view, coarse chroma**
  (none was among the picked looks). The ink build has a Paper switch (lines on blank paper).
- **Chroma class uses one stored row** (this row + the row above): -2 EBR.
- **Sample count for the adaptive placement is counted**, not multiplied out.
- **Colour modes** (K4): Source = the region's own colour; Tint = the region's hue turned by
  the set; Palette = the set's colours by luma bin; Patchwork = by a mix of bin, hue and
  saturation. This is the "related to the colour or not" setting; it also gives the ink build
  designer palettes.

## 6. Resolution consistency (warnings for Ron)

Everything here is pixel-kernel based, so the picture is **not** the same at SD / 720p /
1080p: mattes are cleaned over 10 px x 5 lines, texture nodes are 64 / 16 px. At 720p the
mattes and textures are 1.5x larger relative to the screen; at SD about 2.7x larger and the
fields are line-doubled (no field parity reaches a program). Interlaced timings halve the
vertical filter lengths so cells stay roughly square. HD is where this design is judged.

## 7. Timing closure

Rules that held (see FLATLINE DESIGN.md section 7 for the first set): one carry chain or
barrel per stage; sequencer register files written through a value bus + one-hot enable; and,
new here:

- **Line strobes are registered a clock ahead** (`(not ce) and hf_c(k)` is '1' in the cycle
  that ends at the wanted edge). `ce and hf_c(k)` taken combinationally into ~20 enables was
  a 13.5 ns routing path (67 MHz).
- **The quantizer starts a stage early** (from ys17) so the hysteresis subtractions get a
  stage of their own (64 -> 74+ MHz in the ink build).
- **The sequencer's monotonic clamp and the bus write are separate states.**
- **Resize before subtracting node values** (two 10-bit signed nodes differ by more than 511).
