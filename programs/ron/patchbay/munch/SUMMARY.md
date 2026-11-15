# Munch — munching squares: ramps, ADCs, logic, window comparators, a 3-channel DAC

**v0.1.1 built 2026-11-15 (K6 camera pass), NOT flashed.** Patchbay (a patch that was never built on the rack). A digital-logic
patch: the H and V ramps go through two 8-bit ADCs on a divided clock, a logic module combines the two words, three
window comparators watch the result against a sweeping threshold ("munching squares"), and the word's bit planes
drive a 3-channel DAC colouriser. Working files: `.patchbay_work/rb_munch/` (`notes.txt`, `proto.py`, `imodel.py`,
`cmp_final.png`, `sheet.png`, `strip.png`, `presets.png`). Presets are hand-written.

## The patch

```
H ramp ──┐                      ┌─ Cell (clock divider) ─┐
V ramp ──┴─ Mirror (fold) ─ + drift ─► 8-bit ADC x 2 ─► Code (binary / Gray) ─► Skew (H word += V word)
                                                              │
                         LOGIC  XOR · AND · OR · ADD · Lattice · Carry · Mul · Rings  ─► word w (8 bit)
                                                              │
   frame counter ─► threshold T ─► 3 window comparators (R, G, B):  lit = (w − T_c) mod 256 < Window
        (Speed)        MUNCH spreads T_R, T_G, T_B apart (two slow LFOs wobble the spread) and, above 60 %,
                       opens a second window per comparator half a range away
                                                              │
   w ─► top six planes (Bits fixed at 7-2) ─► three 2-bit DACs (R: 7,4 · G: 6,3 · B: 5,2), each gated by its
                       comparator; the G and B DACs settle late ─► colouriser (Colour: 32 hue mappings)
                                                              │
   KEYER: no window open ─► camera luma (Source), or the dimmed quilt at low MUNCH ─► output
   camera luma ─► leaky average ─► sample-and-hold per cell ─► narrows the comparator windows
```

- **ADCs.** One ADC code is 3.75 virtual px, so 256 codes span half the screen width. The clock divider makes a
  cell of `step` codes (2 .. 32, i.e. 7.5 .. 120 virtual px) and each cell holds `floor(cell index × step)`. At
  step = 2, 4, 8, 16, 32 (detents on the knob) the words are clean multiples and the picture is the textbook quilt;
  in between, the low bits beat against the clock and the quilt goes ragged, as a real ADC on a free-running
  divider would.
- **Logic.** XOR (munching squares), AND (Sierpinski), OR, ADD (diagonals; diamonds under Mirror), Lattice
  `(A+B) xor (A−B)`, Carry `((A+B) xor A xor B) >> 1`, Mul `(A·B) >> 3` (hyperbolas), Rings `(A² + B²) >> 3`
  (zone-plate circles).
- **Comparators.** One per DAC channel. At MUNCH 0 the three thresholds coincide (one window, full colour).
  MUNCH moves them to thirds of the range, narrows the windows to 30 % of the Window knob, and above 60 % gives
  each a second window 128 codes away: six fronts munching in three colours.
- **DAC / colouriser.** Six bit planes, two per channel. Every non-black code is a bright saturated colour
  (the colouriser's bands); the channel colours start at red / green / blue and turn through 270° of hue. The G
  and B channels lag R by 8 and 16 px at 1080p (the same width in virtual px at every resolution), which puts a
  thin fringe of another colour on every vertical edge.
- **Keyer.** Where no window is open the picture is black, the camera (Video), or at MUNCH < 35 % the quilt
  itself at a reduced DAC reference (55 % at 0).

## What it draws

At the defaults: a 4-way mirrored XOR quilt of 30-px cells (64 × 36 of them) in saturated lime, cyan, hot pink and
blue blocks with flat black where the key is closed, soft horizontal edges with colour fringes and hard
line-to-line edges. The threshold climbs 0.63 codes per field, so the lit set changes every 8 codes (about 5 times
a second) in the munching-squares order, the three channels a few steps apart; meanwhile both ramps drift (19 and
7 virtual px per second): the quilt flows sideways through the fold and, under Mirror, hops one cell vertically
every 4 s (smooth both ways with Mirror off). The motions have unrelated periods and the spread between the
channels breathes on two more LFOs (32 s and 45 s), so it does not loop.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Cell | ADC clock divider | cell size 7.5 .. 120 virtual px (exponential, glided); detents at 7.5 / 15 / 30 / 60 / 120 px give the clean quilts, between them the ragged mosaics. Default 30 px |
| K2 Logic | logic module | XOR · AND · OR · ADD · Lattice · Carry · Mul · Rings (stepped, with hysteresis) |
| K3 Window | window comparators | window width 20 .. 249 codes of 256: a sparse confetti of lit cells .. nearly everything lit. Default 136 |
| K4 Speed | frame counter | bipolar, ±8 deadband, square law: the threshold's rate (0 .. ±4 codes per field; 720 = 0.63) and direction, and the drift of both ramps with it |
| K5 Colour | colouriser | the three DAC channel colours turned through 270° of hue in 32 steps (0 = red / green / blue). Was K6 before v0.1.1 |
| K6 Camera | keyer + comparators | camera (or Osc) strength: 0 % = none (the black-input picture), the keyed-through picture fades in to full level by 25 %; the per-cell luma's bite out of the windows is K6 / 256 x the v0.1.0 amount (25 % = v0.1.0, 50 % default = twice, 100 % = four times). See "K6 camera pass" |
| S7 Mirror | ramps | On: both ramps folded about the centre (4-way symmetry) / Off: plain ramps from the top left |
| S8 Code | ADCs | Binary / Gray-coded ADC words |
| S9 Skew | logic input | Off / On: the V word is added to the H word before the logic (diagonals, eyes) |
| S10 Planes | DAC output | Filled / Outline: each DAC channel is drawn only where its level changes (circuit-board line art) |
| S11 Source | keyer + comparators | Video / Osc |
| P12 Munch | comparators + keyer | 0 % the whole quilt, with one full-colour window passing over the dimmed rest · 35 % the key goes black: munching squares · 100 % the cascade: three thresholds at thirds, each doubled, narrow windows |

## Video

(Amounts below are the v0.1.0 ones, K6 Camera at 25 %; v0.1.1 defaults to twice the bite, see "K6 camera pass".)

- **Video (default).** The camera's luma is the keyer's background: wherever no comparator window is open the
  picture shows through, monochrome and at full resolution, so the logic chews holes of colour into it. The same
  luma, through a leaky average and a sample-and-hold per cell (taken on the line above each cell row, so cells
  stay rigid), narrows the windows by up to 72 codes: bright parts of the picture keep fewer lit cells. With a
  black input the patch gives its full picture on black.
- **Osc.** A drifting 640 × 540 virtual-px triangle lattice replaces the camera at the same point (line-buffer
  write), at up to 96 codes of narrowing: grey diamonds glow through the key and thin the cells out.

## Engine

- **Per field (vblank zone, one serial shift-add multiplier, 16 slots).** Glide for K1, K3, P12. Cell: exp2 ROM →
  `step` (Q8 codes per cell) and `inv` (cells per half-px, Q20), cells per pixel and per line; Speed² → threshold
  step and ramp drift; the H and V start phases and start codes; window widths, threshold offsets (two triangle
  LFOs), the keyed-out quilt level. Threshold and drift accumulate here.
- **Per line.** A vertical DDA (cell phase as a 20-bit fraction, the V ADC code as an accumulator stepped on each
  carry, reloaded at the fold line) runs one line ahead, so every line knows whether it is the last of its cell
  row: that line samples the camera for the next row and stores the DAC codes for Outline. No multiplies.
- **Per pixel (25 stages, C_LAT 24).** 1 cell phase · 2 H code · 3 Gray, hold RAM address · 4 Skew · 5 logic
  candidates and 8×4 partial products · 6 product, Lattice, Carry · 7 + B², function select, window widths ·
  8 distances past the three thresholds, bit rotation · 9 comparators · 10–11 DAC lag lines · 12 keyer, code ·
  13 Outline (code line buffer) · 14–16 colouriser ROM, camera · 17 DAC reference (4 small multipliers) ·
  18–19 compose · 20–24 output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16; luma [1 3 4 4 3 1]/16 plus a quarter
  of its difference from the 9-tap average, i.e. a little edge peaking; luma 13/16) · 25 blanking gate.
- **Block RAM (18).** Colouriser 2048 × 16 (8), source luma line buffer (4), DAC-code line buffer (4), per-cell
  hold 4 banks × 512 (1), exp2 (1). All collision-free (`no_rw_check`, no transparency emulation).
- **Contracts.** Generated avid (the fold is centred on the source raster), one latency in every mode, blanking
  gate, resynced running sums, virtual 1080-line coordinates (SD shows the centre 4:3), colours in the hardware
  convention (U = Cr), the engine restarted every field.

## Status

- **v0.1.0 built 2026-11-14 with ROUTER1, all six configs on seed 1. Not flashed.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .kinetic_work/tools/build_queue.sh patchbay munch`
    (plain rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay munch`).
  - Post-route Fmax: HD Analog 100.21, HD HDMI 96.62, HD Dual 95.44, SD Analog 95.47, SD HDMI 85.67, SD Dual
    95.37 MHz. 6,478-6,503 / 7,680 LC (85 %), 18 / 32 EBR.
  - Output: `out/rev_b/ron/patchbay/munch.vmprog` (440,136 bytes).
- **Router history.** The default router2 closed HD Analog on seed 1 (82.88 MHz) and SD Analog (91.83 MHz), then
  stalled on HD HDMI seed 1 (7+ minutes against 50 s for HD Analog; in an earlier, overloaded run SD Analog seed 1
  timed out too and passed on seed 2). `dupin.py` on the synth netlist is clean (no CARRY I0 == I1), so the build
  was moved to router1, which closes all six in 60-115 s each. Stay on router1.
- **Standalone yosys:** 3,479 LUT4, 2,542 FF, 0 `emulate_transparency`.
- **Checks (GHDL image sim; files in `.patchbay_work/rb_munch/`).**
  - `imodel.py` is the RTL arithmetic in numpy. **Every render below differs from it in 0 pixels** (Y, U and V,
    whole frame), on the shipped sources:
    720p60 default, black input (field 3, and field 8 for the motion: 56 % of the pixels change in between);
    720p60 Source Osc; 720p60 camera = `selfie.jpg` (the model fed the tester's own image conversion);
    NTSC, both fields; 1080i59.94, both fields; test A (Rings, Gray, Skew, Outline, Mirror Off, Bits 4-7,
    Colour 900); test B on a sim-only copy with the glides bypassed (Cell 640, Mul, Window 800, Speed 300,
    MUNCH 1023); test C (Carry, Outline).
  - `cmp_final.png`: prototype | model | RTL black input | RTL Osc | RTL camera. `cmp_modes.png`: 720p, 1080i and
    NTSC side by side: the same picture (SD = the centre 4:3). `sheet.png`, `strip.png`, `presets.png`: the model.
  - `thincheck.py`: no single-row features in the model's output at 720p / 1080p over 20 fields x 6 settings
    (camera grain apart); at NTSC only Outline with the smallest cells.
  - Not simulated: PAL / 480p / 576p timing IDs (their constants are reasoned), the other presets in RTL.
- **Watch on hardware.** No pin-skew seed screening. The picture is hard cell edges in motion, the case the
  output conditioning exists for; if HD HDMI still bars, widen the luma filter first (drop the peaking term).
  The top line of the camera's per-cell hold uses the previous field's samples. K2 is a labelled knob (K5 was too, before v0.1.1):
  values below 8 are read as the label index (the load-time default), so a pot resting at the very bottom of
  its travel could flicker between the first labels.

## K6 camera pass (2026-11-15) — v0.1.1

- **K6 Camera** (0 .. 100 %, default 50 %): camera (or Osc) strength, on both places the camera enters.
  - **Keyed-through picture** (where no window is open): its weight is the old one x min(K6, 256) / 256. It is
    folded per field into the background's camera weight `c_cg32` (one 6 x 6 product in the zone), so 0 % is
    black, 25 % and up the full v0.1.0 level. It is not boosted further, because the mono luma already reaches
    white (x 1.25 before the shell's 13/16), and twice would clip into a washout.
  - **Window narrowing** (the held per-cell luma's bite out of the comparator windows): held step x 12 g codes
    (camera) or 16 g (Osc). g = K6 / 256 up to 50 % (x 2), then 2 + (K6 - 512) / 512, reaching x 3 at 100 %.
    That is (min(3k, 1536) + 3k) / 128 for the camera and (min(k, 512) + k) / 32 for the Osc, worked out per
    field. The bite is a 3 x 6 product in the pixel stage that already formed it.
    25 % = v0.1.0 (up to 84 codes), 50 % = twice (168), 100 % = three times (245). Bright parts of the picture
    close their windows down to the 8-code floor, so the picture takes the bright cells and the quilt munches on
    in the dark ones.
    A straight x 4 at 100 % (tried) left only scattered confetti over a bright picture, which no longer reads as
    munching squares.
  - K6 has its own glide channel (1/8 per field, ±4 deadband).
  - Looks (selfie, 720p): 0 % = the quilt on black (the black-input picture). 25 % = v0.1.0. 50 % = the bright
    figures and background break through as mono picture, and the quilt holds the darker half. 100 % = the
    picture fills the frame, with the quilt munching only in the dark areas and confetti in the bright ones.
    Osc at 50 %: grey lattice diamonds with the cells thinned to confetti in their bright centres.
- **K5 Colour** is the old K6, unchanged. **Bits (old K5) is fixed at "7-2"** (the top six planes, its default,
  value 448); the rotation mux constant-folds away. Presets: the old K6 values moved to K5; "Gray Plaid" lost its
  Bits "4-7" (it now shows the default planes).
- Sheet: `.patchbay_work/rb_munch/k6sheet.png` (720p60 decimation 2, `selfie.jpg`, glides bypassed in a sim-only
  copy; K6 = 0 / 256 / 512 / 768 / 1023, Osc at 512). `k6_top_cmp.png`: 100 % with the rejected x 4 law vs x 3.
- **Build v0.1.1 (2026-11-15): router1, all six configs on seed 1.** Post-route Fmax HD Analog 99.18, HD HDMI
  104.68, HD Dual 90.24, SD Analog 101.92, SD HDMI 99.86, SD Dual 96.69 MHz (worst HD 90.24). 6,646 .. 6,655 /
  7,680 LC (87 %; v0.1.0 had 6,503), 18 / 32 EBR. `out/rev_b/ron/patchbay/munch.vmprog`. Not flashed.

## Cut / deviations

- **P12 0 % is not a frozen quilt.** The spec asked for "a still XOR quilt" at 0 %. At 0 % the whole quilt is on
  screen (keyed-out cells at 55 %), but one comparator window still passes over it and the ramps still drift, so
  K3 Window and K4 Speed stay alive at every slider position. Speed at centre stops it.
- **K1 Cell starts at 7.5 virtual px, not 4.** One ADC code is 3.75 px, and cells under two codes add nothing but
  aliasing (and vanish under the output conditioning at SD).
- **The camera does not enter the H word.** Luma added to the H word (or to the threshold's phase) turned the
  quilt into confetti with the test picture. It is the keyer's background and narrows the windows instead.
- **Under Mirror the V ramp drifts in whole cells** (the H ramp stays smooth). A smooth drift thinned the cell row
  at the fold to a single line every few seconds; a hop per cell suits the frame-counter patch anyway.
- Mul and Rings were added to the spec's logic list; there is no (x+y) xor (x−y) switch because it is K2's
  "Lattice".
- Stepped controls (K2, K5 Colour) jump; Colour's 32 hue steps are 8.7° apart.
- SD: cells under about 15 px are one or two field lines tall, so K1's lowest quarter looks uneven at SD and
  Outline's horizontal lines there are single field lines. The DAC lag and Outline line width are per mode
  (8 / 5 / 4 px), so fringes look alike at every resolution; the output filter is in pixels, so SD is softer.

## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. **Mixed**: K6 drives two things. The keyed-through camera picture fades in to full level by 25 % (`c_gb5`);
that part is picture-type and is unchanged. The held per-cell luma also narrows the comparator windows (the
"bite", `c_nw`); that part is element-type and gets the +25 %.

- **Change:** sequencer state 422 now loads `c_nw` from `z_nwq + (z_nwq >> 2)` (14-bit sum), not from `z_nwq`.
  So the bite is x1.25 at every K6 position: at the top, camera 35 -> 44 and Osc 47 -> 59 per held-luma step
  (6-bit register, max 63). The largest sums are 5756 (camera) and 1918 (Osc), so the slices don't wrap. The
  product `p6_lw` (3 x 6 bits) is unchanged at max 441, and the window subtract still saturates as before. No
  pixel-path change, latency unchanged. K6's default is 100 % (set by the orchestrator).
- **Look check** (720p60 decimation 4, bear.png, Logic = XOR, glide bypassed in scratch copies, K6 100 % old vs
  new; `.patchbay_work/rb_munch/k6b/k6b_cmp.png`): the same frame, with a few more quilt cells closed over the
  bright parts of the bear (more picture, less quilt). About 6 % of pixels differ. No wrap or new artifacts.
- **Build (router1, all six configs on seed 1):** HD Analog 100.07, SD Analog 96.95, HD HDMI 93.89, SD HDMI
  97.63, HD Dual 97.21, SD Dual 97.95 MHz (worst HD 93.89); 6,683 .. 6,693 / 7,680 LC, 18 / 32 EBR.
  `out/rev_b/ron/patchbay/munch.vmprog`. Not flashed.
