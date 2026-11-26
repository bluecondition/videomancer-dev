# Hachure: a figure drawn in four hatchings

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 9. A scanner chooses among four stripe textures by the level of
a diamond gradient (plus the camera), like an engraver's hatching of a relief: each zone is ruled at its own angle
and filled with its own Matte colour, and where the scanner crossfades two hatchings weave a moire seam. Flash
`out/rev_b/ron/onevolt/hachure.vmprog`. Presets are hand-written.

## The rack

- **LZX ramps -> Navigator** (Expedition): rotates the H / V ramp pair (K4), with a slow 29 s sway.
- **LZX Angles**: fixed-ratio mixes of the rotated ramps; four outputs at 0, 45, 90 and 135 degrees.
- **LZX Stairs x 4**: Steps (K3) is the pre-fold gain: each angled ramp folded into triangle stripes. S9 swaps them
  for **LZX Keychain** keys of the same ramps (hard stripes).
- **LZX DSG3**: Fold 1 on H and V (triangles); output Average (diamond) or Absolute (saltire), S7.
- **LZX Proc**: bias (K1) and gain (P12, the headline) on the DSG3 output, with the camera luma summed in (K6).
- **BSO Scanner** (4-input video-rate interpolating scanner): the four Stairs outputs on its inputs, the Proc output
  on SCAN, crossfade width on K2. At K2 0 % it is the **Visible Signals Quarterizer** (hard four-band switching).
- **LZX Matte + SMX3**: one colour per scanner input, picked by the scan level's zone (K5 colour set, S10 Zones /
  Tint).
- Camera decoder (S11 swaps in an oscillator lattice) and the ESG3-style encoder (the shell's output conditioning).

## The patch

    H, V ramps -> Navigator (K4 rotation + 29 s sway) -> Angles out 0 / 45 / 90 / 135
         -> Stairs 1..4 (K3 Steps)  [S9: Keychain 1..4, threshold 0.3, 1-px edge]  -> Scanner in 1..4
         [S8 Pitches: the Angles 45 out multed to all four Stairs at Steps x1/2, x1, x2, x4]
    H, V ramps -> DSG3 (Fold 1 on both) -> Average out [S7: Absolute out] -> Proc B in (P12 gain)
    camera luma -> 8-px cells -> vertical slew -> horizontal slew -> Proc A in (K6)   [S11: Osc lattice, 2x level]
    Proc bias (K1 + a 16 s triangle LFO, +- half a zone) -> Proc out -> Scanner SCAN
    K2 -> Scanner crossfade width (all three boundaries together)
    Scanner out -> encoder luma; SCAN zone -> SMX3 picks Matte colour 1..4 (K5 set, S10 Tint) -> encoder chroma and
         luma level
    each Stairs' phase creeps at its own slow rate (9 / 12 / 15 / 18 s per stripe)

## What it draws

At the defaults: a large diamond of nested bands spans the screen, two passes of the four textures from the corners
to the centre. The outer zone is ruled with horizontal stripes in deep blue, the next with 45-degree stripes in
crimson-magenta, the next with vertical stripes in orange, the centre zone with 135-degree stripes in pale yellow,
then the cycle repeats (a small blue diamond at the centre). Soft triangle stripes about 48 virtual px a cycle;
the colours rise in brightness toward the centre like a relief. At every zone border the scanner crossfades the two
hatchings over a quarter of a zone on each side, so the stripes cross and weave a moire seam.

Motion at the defaults: the zones breathe in and out by half a zone over 16 s (the Proc bias LFO), the whole set of
directions sways +- 5.6 degrees over 29 s, and each texture slides along its normal at its own slow rate.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Scan | Proc bias | which texture sits where: a full turn moves the zones through one whole cycle of the four textures (the two ends meet); 512 = the outer zone horizontal |
| K2 Blend | Scanner crossfade width | 0 % = the Quarterizer: hard switching, each zone exactly one texture, crisp borders (a collage); 50 % (default) = seams a quarter of a zone wide on each side; 100 % = full interpolation: everywhere two hatchings superimposed in proportion |
| K3 Pitch | Stairs Steps | stripe period 240 .. 10 virtual px (a few fat bars .. fine hatching), exponential; textures fade to their mean as they approach ~4 px a cycle (by the raster's coarser axis) |
| K4 Angle | Navigator rotation | turns all four directions together, 0 .. 180 degrees (interpolated sine, no steps) |
| K5 Colour | Matte / SMX3 | eight designed four-colour sets (Engraver, CMY, Lagoon, Neon, Relief, Bauhaus, Moss, Ice), each flat over half of its eighth of the travel and blended linearly into the next; the top blends Ice back into Engraver; 512 = Relief |
| K6 Camera | camera into Proc | 0 = none (the black-input picture exactly) .. 2 passes of the textures per full luma (default 100 %): the subject is redrawn as luma contours in the four hatchings |
| S7 Gradient | DSG3 output | Average (diamond zones) / Absolute (saltire: X-shaped zones) |
| S8 Textures | Angles | Four angles (0 / 45 / 90 / 135 at one pitch) / Four pitches (one 45-degree angle at x1/2, x1, x2, x4: a density ramp; the finest fade to flat tone) |
| S9 Shaper | Stairs / Keychain | Stairs (soft triangle stripes) / Keychain (hard stripes: 70 % colour, 30 % black rules, 1-px edges) |
| S10 Colour | Matte columns | Zones (four colours per set) / Tint (one hue per set, the zones differing only in brightness and whiteness: a copper-plate look) |
| S11 Source | input module | Video (camera) / Osc (a drifting 640 x 540 triangle lattice at twice the camera level) |
| P12 Span | Proc gain on SCAN (headline) | 0 % = half a pass: two broad zones across the whole figure (one big 45-degree field round an orange vertical-hatched diamond); 50 % (default) = 2 passes, the eight-band diamond; 100 % = 6 passes: a dense nest of narrow hatched rings, each about one stripe wide. Exponential (one linear segment per octave, 2 octaves below the centre and 1.5 above), glided |

## Video

The camera is patched into the Proc that feeds the Scanner's SCAN input: bright parts of the picture push the scan
level up, so the subject is drawn as contour bands in the four hatchings and colours, laid over the diamond. Camera
luma ((Y >> 2) - 16) is averaged over 8-px cells, slewed down the screen (one pole, 1/16 per line, 12-bit state per
cell in a block RAM) and across (one pole, 1/8 per pixel), then scaled by K6 (0 .. 2 passes per full luma). Osc
puts the shell's drifting triangle lattice on the same jack at twice the level (zones become a lattice of cells).

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").** Glide of K1-K6 and P12 (1/8 of the gap per field), the
  Proc bias and sway LFOs, a field counter for the creep. The Navigator's sine and cosine from a 256-entry
  quarter-wave ROM with 7-bit interpolation. Then 57 slots on one serial multiply-accumulate (20 clocks each): the
  H / V tent steps, the stripe frequency (exponential), A = F sin, B = F cos and their 45-degree rotations, the fade
  source, per texture the pixel step, the line step and the top-left phase (accumulated: creep + S x centre -
  C x 540, loaded straight into the line accumulator), the camera gain, and the twelve K5 blends of the Matte
  colours. Then the table writer: one DDA engine (with a restoring divider) rewrites the crossfade table t(u)
  (128 x 8) and the four Stairs / Keychain shaper tables (256 x 8 each). About 3,300 zone clocks in all (the
  sim's 480i blanking gives 5,248; real timings give 14,000 or more).
- **Per line.** The V tent (reflected at the centre line) and the four texture line phases.
- **Per pixel (14 stages).** H tent DDA and four 18-bit texture phase DDAs (stage 1); fold to triangles -> shaper
  table addresses, DSG3 sum or difference (2); shaper EBR, |d| or sum, Proc bias + camera (3); fabric copies,
  scan level (4); zone, neighbour and distance from the zone centre -> crossfade table, the pair of textures (5);
  B - A (6); t (7); (B - A) t (8); the pair (9); delays (10, 11); the scanner's mono output and the zone's Matte
  colour (12); colour x mono (13); offsets, black off the generated avid (14).
- **Output.** Chroma through the shell's two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync; luma through
  [1 2 1]/4 taken one pixel ahead of the chroma centre (a faint chroma lag to the right, like an analog path delay);
  luma 13/16; the blanking gate. One latency, C_LAT 20, in every mode.
- **Contracts.** Generated avid; virtual 1080-line coordinates; colours in the hardware convention (U = Cr); drawn
  whites ~777; every written RAM carries `no_rw_check`.
- **Block RAM (12).** Shapers 4, crossfade table 1, sine ROM 1, Matte ROM 1, camera cell store 1, box rings 4.

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI re-placed for the encoder pin skew: router2 seed 19 (HSYNC +1.58 ns, data <= -0.94 ns, 81.99 MHz), the most positive in-window placement of 76 screened; the builder's router1 seed 1 sat at -0.43 ns. Spare: router1 seed 11 (+1.04 / <= -1.16, 92.46 MHz). A plain rebuild gives the builder's placement again: re-run .heirloom_work/skew_tools/install.sh hachure router2 19 hd_hdmi. Screen data: .heirloom_work/rb_hachure/skew/.

- **v0.1.0 built 2026-11-25 with router1, all six configs on seed 1, not flashed.** Rebuild:
  `NEXTPNR_ROUTER="--router router1" ./build_set.sh onevolt hachure` -> `out/rev_b/ron/onevolt/hachure.vmprog`
  (433,056 bytes). dupin on the synthesis netlist: 0 `CARRY I0==I1`, 0 duplicate LUT inputs.

  | Config | router1 (shipped) seed / Fmax MHz | router2 (same source) seed / Fmax MHz | LC |
  |---|---|---|---|
  | HD Analog | 1 / 89.78 | 1 / 77.23 | 6,683 |
  | SD Analog | 1 / 86.10 | 1 / 77.07 | 6,673 |
  | HD HDMI | 1 / 91.57 | 1 / 77.72 | 6,675 |
  | SD HDMI | 1 / 85.80 | 1 / 78.95 | 6,682 |
  | HD Dual | 1 / 92.28 | **2** / 76.31 (seed 1 missed) | 6,693 |
  | SD Dual | 1 / 89.01 | 1 / 78.70 | 6,694 |

  6,673 - 6,694 / 7,680 LC (87.2 %), 12 / 32 EBR. iceunpack OK x6; the packed config carries the labels and the
  eight presets. The router2 vmprog of the same source is kept in `.onevolt_work/rb_hachure/router2/`. Fmax is the
  builder's routed figure. No pin-skew screening.
- **Checks** (working files in `.onevolt_work/rb_hachure/`): `check.sh` clean; `scan.py`: 0 files, 0 sites;
  `thincheck.py` on the model (720p, 1080p, 480i; 18 settings x 2 times): no single-row hard runs of 24 px or more
  (longest 4 px).
- **GHDL renders vs the integer model `imodel.py`** (`diff.py`, rows 1 on, the tester's capture 4 px left of the
  generated avid). Final source (byte-identical to the one built):
  - 720p60 decimation 1, black input, every control at its default: **0 differing pixels** (`rtl_default.png`);
    its raw output is byte-identical to the first build's default frame (the swap does not change the default).
  - P12 Span at 0 and at 1023, K2 Blend at 0 and at 1023 (a sim-only copy with the glide landing in one field), and
    S7 + S9 + S10 together: **0 differing pixels** each (`rtl9_*.png`).
  - Run on the first-build source, before the swap (the swap only moved the Span / Blend decodes and widened two
    registers; the default path is unchanged): each of S7, S8, S9, S10 alone, a field-10 motion frame (875,892
    pixels changed against field 3), one 480i field and one 1080i field: **0 differing pixels** each; Osc, selfie
    and bear frames checked by eye (`rtl_osc.png`, `rtl_selfie.png`, `rtl_bear.png`; the model's camera filter does
    not reproduce the RTL's cell / slew start-up, so those are not compared pixel by pixel).
  - No stepped knobs (K5's detents are a continuous blend).
  - An earlier RTL frame differed in 75,680 pixels around every zone centre: the crossfade-table writer issued a
    129th write that wrapped onto t(0). Fixed before the first build.
- Images: `sheet.png` (control sheet + the 16 switch combinations), `strip.png` (8 frames, 3.5 s), `camera.png`
  (selfie and bear at K6 0 / 25 / 50 / 100 %), `p12_ends.png` (every control at both ends with P12 at 0 % and at
  100 %), all from the integer model; `rtl_*.png` / `rtl9_*.png` from GHDL; `cmp_final.png`.

## Cut / deviations

- **Control map (coordinator's send-back, 2026-11-25):** P12 and K2 swapped against the spec. P12 is Span (the Proc
  gain, the boldest continuous control: half a pass .. 6 passes) and K2 is Blend (the Scanner crossfade width). The
  default frame is bit-identical to the first build's.
- **Blend tops out at full pair interpolation (b = 1 zone).** Two wider tops were built and measured and both cut
  for fit: the four-input leak (the average of all four textures mixed in above ~70 %, ~150 LC) and a three-way tent
  (each neighbour a quarter at every zone centre at 100 %: a far-neighbour weight table and a second multiply,
  measured 204 LUT + 38 FF by ablation, which would have put the build at about 6,920 LC).
- **The Matte colour is picked, not crossfaded**: the zone's colour switches at the zone border (the spec's "picks
  one of four Matte colours"), softened only by the chroma boxes; K2 crossfades the textures. A crossfaded colour
  table (3 EBR, ~420 LUT) was built and verified, then cut for fit; the picked colours also read bolder.
- **Luma conditioning is [1 2 1]/4, not the shell's two 8-boxes** (chroma keeps both boxes, the CLAUDE.md lever):
  two 8-boxes on Y would erase every hatching finer than ~20 px except the near-horizontal ones. The vertical
  [1 1] mix of separatrix is not used (the thin-line check is clean without it).
- P12 and K3 use piecewise-linear exponentials (one linear segment per octave: within 6 % of a true exponential,
  smooth, monotonic).
  The sway is +- 5.6 degrees (spec 6). Matte colours are kept to 8 bits per channel.
- The fade toward a flat tone near the pixel limit uses the raster's coarser axis for every texture (conservative:
  diagonal textures fade slightly early at HD).
- **Resolutions**: 720p and 1080p draw the same picture. At SD the diamond spans the 4:3 width, so the zones are
  squarer than at HD, and the fine end of K3 fades much sooner, because the fade follows the coarser axis (a 480i
  field line is 4.4 virtual lines) for every texture: on SD the hatchings start to fade from K3 ~61 % and are flat
  tone (colour zones only) from ~83 %. At 720p only the top few % of K3 fade slightly; 1080p never fades.
