# Archway — a pointed arch of stepped mosaic, black arcs and cyan lines

**v0.6.0 (2026-10-28, phase 2 round 1): built, NOT flashed.** v0.5.0 flashed by Ron 10-28; his notes are under
[Phase 2](#phase-2). Heirloom #36, rebuilt from the photo
`photos/WIN_20230523_01_26_43_Pro.jpg`. The photo shows a tall pointed arch drawn three ways at once:
its flanks are a staircase mosaic of square colour cells (yellow, olive, hot pink, maroon, blue, navy
going outward) cut by four smooth black arcs; its middle is a fan of cyan arch lines, thin over the
mosaic on the left half and fat cyan with navy gaps on the right half; a pink chevron with pale stepped
cells stands at its foot, holding nested cyan lines. Thin streaks of other colours ring through the
cells next to the chevron. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_archway/` (`notes.md` has every step).
- **Field.** Every element is a contour of ONE field, F = |x − 1013.1| − s(y), with
  s(y) = 3.8736e-4·y² + 0.012796·y − 2.498 = 3.87e-4·(y + 16.5)² − 2.6 (s(1080) = 463).
  - Fitted to four traced black arcs (`trace.py`, `sfit.py`): rms 1.35 px. The left/right arcs mirror
    about x = 1013.1 within ±1.5 px over the whole height.
  - The contours are horizontal translates of one parabola (the spacing of two traced arcs is 109–112 px
    at every height), so this is an H fold plus a squared V ramp, not two circle arcs.
- **Mosaic (a 2-D sample-and-hold).** Cells are 52 × 52 in (x, s):
  - columns at x = 35 + 52.00·k (fitted over both halves);
  - rows at equal steps of s: s = 5.5 + 52·j, which puts the row edges at y = 129, 378, 519, 631, 729, 813,
    890, 966, 1032; the rows shrink toward the bottom;
  - columns right of x = 1439 step 12 units of s (a quarter row) later.
  - Each cell holds F sampled at its left edge and its row top, Fs. One colour per step of
    n = floor((Fs − 20)/52):
    - n = −1 yellow, 0 olive, 1 maroon, 2–3 yellow (behind the left lines);
    - 6 yellow, 7 olive, 8 hot pink, 9 maroon, 10 blue, 11–12 navy, then black;
    - on the right half, step 6 reads olive.
- **Black arcs.** At F = 282, 380, 486, 600, 15 px wide, unquantised (they cut through the cells).
- **Cyan fan.** Two summed triangle oscillators of F: tri((F − 36.7)/37.05) + 0.636·tri(F/263.3 + …), fitted
  per row (`linefit2.py`). Each half has its own thresholds into cyan / blue rim / teal / see-through:
  - left: cyan 26 %, rim 23 %, teal 14 %, the mosaic between;
  - right: cyan 73 %, rim 16 %, teal 5 %, navy gaps (12, 52, 82);
  - lines turn green within 14 px of the centre column.
- **Colour path delay.** The mosaic's window is F' < 642 on a field 46 px behind the line path. On the right
  half its inner edge (F' = 331) leaves a solid black gap between the last line and the second arc, where
  the left half shows yellow/olive cells.
- **Chevron.** Pink = F in [−218, −60] AND Fs in [−188, −32); pale = the Fs band alone (IoU pink 0.73,
  pale 0.56, `lamfit2.py`). This one rule gives each leg one smooth edge and one stepped edge. The apex is
  at y 485 (s ≥ 95). Inside it: stepped black cells, a pale band at s 317–421 right of centre, and nested
  cyan Λ lines (the same oscillator continued below F = −258).
- **Streaks.** The first ~30 % of each row in the chevron's outer cells (and alternate cells under the left
  lines) rings through a measured 30-entry sequence:
  olive → magenta → hot pink → violet → blue → magenta → violet/blue → hot pink → dark magenta → hot pink.
  This is the sample-and-hold settling after each row step.
- **Palette (sRGB medians).**
  - mosaic: yellow (247, 202, 0), olive (56, 50, 8), hot pink (255, 0, 164), maroon (99, 10, 45),
    blue (21, 10, 182), navy (21, 14, 48), black (21, 16, 13);
  - chevron: pink (246, 114, 233), pale (229, 201, 196);
  - lines: cyan (1, 205, 252), blue rim (25, 130, 211), teal (26, 91, 118), green (0, 232, 31).
- **Texture.** Horizontal edges 10–90 % in 9–11 px, vertical in 2–3 lines. The line edges wobble per line
  (±0.75 px rms, a ~12-line period).

## What it does
1. The field F = |x' − xc| − s(y) + Etch·luma, in half-pixel units. x' is held near the line start; s(y) is
   computed per line from (vy + 16.5)² × K, where K comes from P12.
2. **Sample-and-hold.** F is latched at every column edge, and s is replaced by the row top: Fs. The grid is
   locked to the centre and crawls with Speed. The step is n = ((Fs − 40) >> 3)·79 >> 10. With Flip the line
   engine runs y from the bottom up and the row tops step down as well as up.
3. **Regions.** Three region-flag ROMs (2048 bins of 4 half-px) on F, F − d and Fs − d give the arcs, the line
   region, the window, the right half's gap, the chevron bands, the inside and the ring band. d is the chevron
   offset (K5 plus the breathing).
4. **Line oscillator.** φ = (F + tear − 73)·rate − drift. V = tri(φ) + 0.636·tri(9φ/64 + c), compared with
   three thresholds per half.
5. **Colour.** A 64-entry palette per palette choice: 15 steps, black, the right step 6, pink, pale, cyan, rim,
   teal, green, navy and the 30 streak colours. Each ROM word holds palette p (A) and palette p + 1 (B). K6's
   morph is a dither: in every 8 pixels, nb show B and (when the count is odd in half-pixels) one shows
   (A + B)/2, spread by a bit-reversed pixel phase, alternating on line pairs. The output box sums exactly
   8 pixels, so the mix comes out flat at A + (B − A)·f/32 with no visible pattern.
6. **Motion at the default Speed.** The lines stream out of the arch (~0.85 lines/s), the sample-and-hold grid
   crawls sideways (~13 px/s) and the chevron breathes ±10 px over ~8 s. Away from 640 the two separate: see K4.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Steps | the mosaic cells: 5.5–194 px (exponential, −3.25 / +1.9 octaves; 512 = the photo's 52 px). The colour levels stay put, so small cells give fine staircases (nearly the Smooth look) and large cells give big blocks |
| K2 Lines | the cyan fan's line density: period ×2.3 (fat bands) .. ×0.44 (fine hatching); 512 = the photo |
| K3 Etch | how far the camera (or the Osc) bends every contour, cell and line: cube law, 0–~750 px at full luma; 400 = 49 px (as v0.5), 700 ≈ 240 px |
| K4 Speed | bipolar, still within ±8 of the centre. **Left:** the lines stream into the arch fast (square law, to 0.25 periods a field) while the S&H grid creeps (linear, to 0.02 cells a field). **Right:** the grid races (cube law, to 0.32 cells a field) while the lines drift out slowly (linear, to 0.06 periods a field). 640 = the v0.5 motion exactly. The chevron breathes with speed² on both sides |
| K5 Chevron | 0 = gone (its apex sits below the bottom line at any Arch); rising, the chevron climbs from the foot; 381 = the photo (at the default Arch); past ~475 its apex leaves the top and the band moves outward until, at 1023, the pink band lies on the outermost arc with the nested lines and black cells filling the arch |
| K6 Palette | a continuous morph (display 1.0–8.0): 1 Archway (the photo) → 2 Lagoon → 3 Orchard → 4 Jade → 5 Ultra → 6 Citrus (RGB permutations of the photo palette) → 7 Ember → 8 Frost (luma ramps), 32 steps between neighbours (16 per line, line pairs alternate), glided |
| S7 Split | On (the photo: thin lines over the mosaic on the left, fat cyan with a black gap on the right) / Off (both halves like the left) |
| S8 Arcs | the four black arcs On / Off |
| S9 Flip | Off / On: the arch upside down, the chevron at the top. The generated field only: the camera's etch stays upright (flipping it would need a frame store) |
| S10 Mosaic | Stepped (the photo) / Smooth: the S&H bypassed, smooth arch bands |
| S11 Source | Video / Osc |
| P12 Arch | the arch's curvature, ×0.2 .. ×3.5 (exponential, glided): 0 % straight pillars (a colonnade of vertical bands), 575 = the photo, 100 % a squat dome flaring over a pool of nested lines |

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 from a one-line buffer, is added to the field
  (Etch).
  - Every arc, line, cell level and the chevron bend around the picture.
  - The sample-and-hold samples the bent field, so the mosaic turns the picture into blocky staircase detail.
  - At the default Etch the arch still reads clearly. With no input, Video mode is the photo.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting, feeds the same path. Every video-path
  control stays live.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier, slot numbers decoded two clocks ahead, operands pre-muxed. The slots
    compute:
    - the cell size (exp2 ROM) and its column phase, row phase and lag;
    - the line rate, the curvature K, the etch gain;
    - speed²: the drift, crawl and breathing steps;
    - the tear wobble step and phase.
    - v0.6 adds: etch³ and its gain; speed³; the lines / grid speeds per side; the crawl in Q4 h
      (cells-per-field × cell); s at the bottom line (from the curvature); the chevron's K5 product. Slots
      9–21 chain through one temporary register.
  - A shared glide for K1, K2, K3, K5, K6 and P12 (1/8 per field, ±2 deadband).
  - The chevron offset d = 150 − s(1080) + (s(1080) + 1170)·K5/1024 − breathing (h), and the S&H grid origin
    (a stepping loop to the first column edge right of the line start, up to 299 cells).
  - The palette position (K6 + 1)·7/32: palette index and morph fraction.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in ~85 clocks (fits SD's 858):
  - s(y) on two serial products (under Flip the squarer takes 1079 − y);
  - the S&H row tops (normal and lagged), stepped one row per clock either way until they hold s (line 0 of a
    flipped field takes up to ~300 clocks, in the zone), and the row position scaled to the streak index;
  - the chevron gates;
  - the tear triangle.
- **Pixel path (24 stages).**
  - Stages 3–6: hold, |dx|, the etch nibble products, F / F − d / F + tear.
  - Stage 7: the S&H latch and the flag-ROM addresses.
  - Stages 8–10: the colouriser step and the line phase; the ROM flags merged.
  - Stage 11: the base entry (mosaic / chevron / streak / gap).
  - Stages 11–13: the slow triangle and V. Stage 15: the line class. Stage 16: the palette address.
  - Stage 16: the palette address (the black entry outside the generated avid). Stage 17: the palette ROM
    {A, B}. Stage 18: a fabric copy and the dither choice. Stage 19: compose A, B or (A + B)/2.
  - Stages 20–21: one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V (the photo's softness, the HD HDMI bar fix
    and the morph's mixer; the beam starts 4 px early for its delay).
  - Stage 24: luma 13/16 and the blanking gate on the generated avid.
- **ROMs.** Line buffer 2048 × 8, exp2 256 × 16, palette 512 × 64, flags 2048 × 8 + 2 × 2048 × 4. Palettes are
  in the hardware convention (U = Cr), luma pre-scaled 16/13 (`archway.py`). The F − d and Fs − d flag ROM
  addresses clamp at ±4096 h (the K5 extremes reach −4600 / +6700).
- **Contracts.** Generated avid (parity-locked); one latency (24) in every mode; virtual-1080 steps.

## Phase 2
**Ron's notes on v0.5.0 (flashed 2026-10-28):**
- K1 Steps: the range could be wider, especially on the 0 % side.
- K2 Lines: seems OK (suggestions welcome).
- K3 Etch: more upper range, squish.
- K4 Speed: centred still; on the left the lines move faster and the blocks slower; on the right the lines
  slower and the blocks faster.
- K5 Chevron: at the bottom the chevron completely gone; at the top it expands to line up with the outside of
  the overall shape (further than now).
- K6 Palette: can it be a smooth transition?
- S9 Chevron On / Off: K5 now removes the chevron, so repurpose it, maybe a Y flip of the whole screen.

**v0.6.0 (round 1) answers:**
- **K1:** cells 5.5–194 px (was 17–156): 3.25 octaves below the photo's 52 px, 1.9 above. The column-origin
  loop runs up to 299 cells and the ring scale is 15 bits for the small cells.
- **K3:** cube law. The default 400 is unchanged (49 px at full luma); 700 ≈ 240 px (was 137); 1023 ≈ 750 px
  (was 292). The low end is squished (200 ≈ 6 px).
- **K4:** each side has its own laws, meeting v0.5's motion at 640. Left: lines m² (to 0.25 periods a
  field), grid m (to 0.02 cells a field). Right: grid m³ (to 0.32 cells a field), lines m (to 0.06 periods a
  field). The crawl is now in cells per field, so small cells don't alias. Direction kept: the left side
  still runs the lines into the arch and the grid backwards.
- **K5:** d = 150 − s(1080) + (s(1080) + 1170)·K5/1024. 0 puts the apex below the bottom line at any Arch;
  381 = the photo (new default); 1023 lays the pink band on the outermost arc (F 884–1200 h), with nested
  lines and black stepped cells filling the arch. The direction is reversed from v0.5 (0 was the huge
  chevron), and Arch now moves the chevron too.
- **K6:** a continuous morph across the eight palettes, 32 steps between neighbours, glided. A real
  per-pixel lerp cost 312 LUT and put the build at 99 % LC, so it is a dither mixed exactly by the output box
  (see What it does, 5).
- **S9 Flip:** the generated arch upside down (chevron at the top, streaks at the foot of each row: a true
  mirror). The camera's etch stays upright.
- **K2 suggestions (not built):** see Notes.

## Status
- **v0.6.0 built 2026-10-28, router2, NOT flashed.** Build seeds: **HD Analog 2, SD Analog 3, HD HDMI 2,
  HD Dual 2**; SD HDMI and SD Dual seed 1. SD Analog seeds 1–2 and HD Dual seed 1 stalled router2 (timeout);
  HD Analog and HD HDMI seed 1 missed timing.
- Fmax (post-route): HD Analog 79.70, HD HDMI 77.18, HD Dual 76.09, SD Analog 72.03, SD HDMI 70.93,
  SD Dual 80.77 MHz (SD needs 27).
- 7327–7357 / 7680 LC (**96 %**), 20/32 EBR. Every bitstream passed iceunpack. Standalone yosys: LUT 3718 → 4167,
  carry 1599 → 1744, FF 2637 → 2690 against v0.5. Little headroom is left for round 2: trade, don't add.
- **Fit history (v0.6).** The first v0.6 cut was 7618 LC (99 %). Ablations put the cost at +312 LUT for the
  per-pixel lerp, +166 for the new sequencer slots and +102 for the two-way row stepping. Replacing the lerp
  with the box-mixed dither and chaining slots 9–21 through one temporary got to 7357.
- **Verification.** A 720p GHDL render of the default frame (Speed centred, black input) matches v0.5's
  `rtl3_still.png`: the mosaic, arcs, chevron and colours are identical; only the line edges differ, by the
  tear wobble's phase (mean |diff| 0.69 / 255). The new ranges were not simulated (hardware check).

- **v0.5.0 built 2026-10-27, router2.** Build seeds: **SD Dual = seed 2**; the other five are seed 1.
- Fmax (post-route): HD Analog 82.14, HD HDMI 80.78, HD Dual 79.36, SD Analog 86.20, SD HDMI 73.08,
  SD Dual 81.25 MHz.
- 6825–6839 / 7680 LC (89 %), 16/32 EBR. Every bitstream passed iceunpack.
- **Verification.** The GHDL render matches the integer model (`imodel.py`) within 4×4 region means of
  0.8 / 1.6 (of 255). Against the photo: RTL 5.1 / 11.2, float proto 5.2 / 11.9.
- **Fit history.**
  - The first cut was 7983 LC (103 %, failed to place).
  - Relative chevron thresholds, one output box and a narrower line multiplier brought it to 7556 LC (98 %).
  - The colour-path field folded into per-side thresholds, region-flag ROMs replacing ~20 comparators, a
    piecewise hold, one tear triangle and a cheaper 0.636·T2 brought it to 6830.

## Deviations from the photo, and why
- **Softness.** One 9-tap box (~7 px edges) against the photo's 9–11 px. The second box was cut for fit.
- **Tear.** A regular 12.5-line wobble on the lines only, where the photo's is irregular. The cell and arc
  edges stay clean.
- **Streaks.** One fixed 30-entry sequence (measured in two cells) on three cell sets; the photo's vary row
  to row.
- **Inside the chevron.** The photo's dark smear right of the nested lines is drawn as stepped black cells
  plus a pale band.
- **SD.** SD shows the centre 4:3 (virtual x 240–1680), so the outermost flank cells are cropped. 720p and
  1080p show the full picture.

## Working files
`.heirloom_work/rb_archway/`:
- `photo.jpg`;
- `notes.md`: the measurements and every iteration;
- `trace.py`, `sfit.py`, `gridfit.py`, `ffs3.py`, `lamfit2.py`, `linefit2.py`: the fits;
- `proto.py` (geometry iterations) and `proto_ctl.py` = `proto_final.py` (the control model);
- `sheet.png`: the control sheet;
- `imodel.py`: the integer model (imports `archway.py`);
- `cmp_final.png`, `cmp_motion.png`, `metric.py`;
- `rtl3_still.png`, `rtl_osc.png`, `rtl_vid.png`: the GHDL 720p renders.

## Notes
- v0.5: only the VHDL header comments changed after the final build; the logic is unchanged.
- v0.6: `imodel.py` still models v0.5. It imports `archway.py`, whose palette ROM is now {A, B} 64-bit words.
- **K2 ideas for a later round** (Ron: K2 "seems OK"):
  - spread the two halves' line rates apart (left denser, right fatter), turning S7 Split into a continuous
    control;
  - the depth or ratio of the slow triangle (0.636·tri(9φ/64)): from even hatching to lines clumped in
    bundles that beat;
  - a line-width offset on the three thresholds: hairlines to solid cyan bands, separate from density.
