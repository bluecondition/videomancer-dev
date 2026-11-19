# Starburst — Heirloom #3, a mirror kaleidoscope of bowties and CMYK chevrons

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/starburst.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`. Ron's hardware-captured defaults from the heirloom v0.5.3 (2026-11-18) are carried over, converted to the diag's knob curves so the picture is the same: Cells 477→570, Bands 667→791, Teeth 290→483 (Speed 188, Etch 1014, Burst 438, Drift On unchanged; at Speed 188 the phase-2 mirror swing is active, since it starts past the photo's speed).

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Starburst Diag`, `out/rev_b/ron/_diag/starburst_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/starburst/`) is untouched.
v0.5.0 (2026-10-21): rebuilt from the photo (`photos/WIN_20221122_23_37_36_Pro.jpg`), built, NOT
flashed. v0.1/v0.2 drew a generic saddle-field kaleidoscope that Ron judged too abstract; v0.5.0
replays the photo's own pattern through the mirrors, with its palette, teeth, feathers and starburst.

## Phase 2 (v0.6.0)
No notes from Ron for starburst (free hand, 2026-11-09 batch). In v0.5 the source pattern sat still under the
mirrors unless S10 Drift was on, K3 Teeth only retextured the band edges (×0.5..×2), K2 Bands and K1 Cells spanned
about one octave each way, and S7 Centre only translated the lattice by half a period. The round:
- puts K1, K2 and K3 on exponential curves with twice the octaves, the photo at 50 %;
- makes the kaleidoscope **turn**: past the photo's speed, K4 swings the mirrors across the source (the shapes
  under the mirrors split and re-form) and breathes the bands through the palette;
- patches the picture into the **mirrors** (K5 past 50 %): the camera (or the Osc) bends the kaleidoscope sideways,
  not only the bands;
- turns S7 into a waveform switch for the two oscillators (teeth and feathers Saw / Square);
- deepens S10 Drift, and moves Source to S11 like every other heirloom program (Mirror to S9).

Every new effect is exactly zero at the default knob value; the default frame is byte-identical to v0.5.1
(checks below). Phase-2 working files: `.heirloom_work/rb_starburst/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K1 Cells: 2 .. 8 cells per screen height, linear (default 356 = 4.1) | **K1 Cells**: 1 .. 4.1 (50 %) .. 16.3, exponential (an octave per quarter turn); readout −100..100, default 512 = the photo (exact) | cell 1080 → 264 → 66 lines tall (1934 → 473 → 118 px wide). 0 % = one giant cell, the grey X filling the screen with the white X's cut by the top and bottom edges; 100 % = a fine lattice about 29 cells across |
| K2 Bands: colourizer gain ×0.5 .. ×3 (default 205) | **K2 Bands**: ×0.25 .. ×1 (50 %) .. ×4, exponential; readout −100..100, default 512 = the photo (exact) | 0 % = the field spans about 0.3 of a colour cycle: 3–4 broad bands, the X's as flat white / grey / charcoal silhouettes, most chevron colours gone. 100 % = about 5 colour cycles across the field: fine contour rings in every shape |
| K3 Teeth: tooth / feather count ×0.5 .. ×2 (default 341) | **K3 Teeth**: ×0.25 .. ×1 (50 %) .. ×2; readout −100..100, default 512 = the photo (exact) | tooth period 132 → 33 → 17 px, feather period 84 → 21 → 11 lines (photo cells). The top stays at v0.5's ×2: above it the 80 knot columns alias the teeth |
| K4 Speed: bipolar, still at the centre; crawl, roll, breathing ±0.025 cycle | **K4 Speed**: as before up to the photo's speed (K4 376..648); **past it the mirrors swing** across the source on the two drift LFOs, depth (\|speed\| − 128) × 133 (u) and half that (v), and the LFOs speed up; **the breathing deepens** from ±0.025 to ±0.4 cycle | at either end the source slides ±0.78 cell under the mirrors (±370 px at the photo's cells; v ±0.39 cell, ±103 lines) on 5.3 s / 6.3 s triangles: X arms split, diamonds bloom and re-form continuously; the colours sweep about five bands back and forth every ~2 s. Exactly 0 at the default 640 |
| K6 Etch: picture bends the bands, up to ±0.27 cycle (default 245) | **K5 Etch**: 0 .. 50 % = band bend 0 .. the photo's (default 512 = the photo, exact); 50 .. 100 % = band bend to ±0.62 cycle (×8.4) **plus the mirror warp**: the same luma pushes the kaleidoscope sideways | at 100 % the camera's whites and blacks push the mirror ±0.43 cell (about ±200 px at the photo's cells; Osc ±0.22 cell, slow waves): the kaleidoscope bends around a face. Below 50 % the warp is 0 |
| S7 Centre: Grey X / White X at the screen centre (a half-period lattice shift) | **S7 Teeth: Saw / Square** (renamed): the teeth and feather oscillators' waveform | Square = crenellations: every serrated edge becomes battlements of full depth (about twice the saw's), the grey X's feathers become steps; at high Burst and few teeth, big castle blocks. The centre is always the grey X (the photo); K4 / S10 swings pass through the white-X arrangement |
| S9 Source: Video / Osc | **S9 Mirror: Fold / Tile** (moved from S11) | as v0.5's S11 |
| S10 Drift: ±0.22 cell u, ±0.14 v | **S10 Drift**: ±0.44 cell u, ±0.22 v (deeper; K4's swing adds) | ±208 px / ±58 lines at the photo's cells, 10 s / 14 s |
| S11 Mirror: Fold / Tile | **S11 Source: Video / Osc** (moved from S9, like the other heirloom programs) | — |

Unchanged: K6 Palette, S8 Greys, P12 Burst.

**Presets** (preset 0 = the defaults): **Photo**; **Calm Bands** (as before); **Full Burst** (as before); **Drifting
Glass** (Drift, 3.2 cells, Acid); **Carved Face** (Etch 86 %: bands and mirrors bent by the camera, more bands);
**Giant X** (one cell, fewer bands, Burst 68 %); **Kaleido Swing** (Speed near max + Drift, 5 cells: the
kaleidoscope turning); **Castle** (Square teeth, few teeth, Burst 80 %, 3 cells).

**Engine (new).**
- **Exponential knobs.** One exponential, `(256 + t(7:0)) << t(10:8)` (an octave per 256 of t, linear within it;
  t = 1024 gives 4096), run three times in the vblank direct-term clocks (zone clocks 24..31, one op per clock):
  N16 = 65·e / 4096 (cells), TS8 = 63·e / 4096 (teeth), GQ = 328·e / 4096 (bands), all constant multiplies as
  shift-adds. K3's right half runs at half rate (t = 1024 + d / 2), so it tops out at ×2. The old slot-12 gain
  multiply is gone. Widened for the range: N16 9 bits, H step 14, T3 19, V step 16, GQ 11.
- **Swing.** Zone clocks 2..6: E = max(0, \|speed\| − 128); swing depth A_u = 133·E (+ 28836 with Drift),
  A_v = A_u / 2; breathing depth 1638 + 64·E; LFO rates 109 / 78 + E / 4. The three LFO slots of the serial
  multiplier take these depths instead of constants. The pan is now a 17-bit modular value (two cells = 2^17), so
  any depth folds correctly (the fold only reads bits 16..0).
- **Mirror warp.** Pixel stage 0: (luma − 126) × W, W = (K5 − 512) / 2 (0..255), the luma being the ring's write
  data (camera or Osc, the source pixel a few px ahead of the beam: invisible). Stage 1 adds the pan, stage 2 the
  sum to u, stage 3 folds (no new stages: the product sits beside the mirror-phase stage).
- **Etch** depth widened to 7 bits (15·k / 512 below 50 %, 15 + 7·d / 32 above).
- **Square.** The column DDA's saw (teeth) and the line engine's V saw (feathers) are replaced by ±127.
- **Chroma ring.** The output chroma filter's two 9-stage 10-bit delay lines are one 256 × 32 block-RAM ring
  (`no_rw_check`, read = write − 6, plus a fabric register = 8 clocks): −160 FF, +2 EBR.
- No pipeline stages added or removed: latency 36 in every mode, U/V parity unchanged.

**Build** (router2, `SEED=3 MAKE_TIMEOUT=900 .heirloom_work/diag_build.sh starburst`, 2026-11-09, 13 forks sharing
the machine): HD Analog **s4** 81.39 (seed 3 stalled the router at the 900 s timeout), SD Analog s3 79.46, HD HDMI
s3 **75.68** (thin), SD HDMI s3 76.68, HD Dual s3 **75.13** (thin), SD Dual s3 81.94 MHz. 7,247–7,290 / 7,680 LC
(94.6 %, +~400 over v0.5.1), 15 / 32 EBR (+2: the chroma ring); iceunpack OK ×6; the packed config carries the
new labels and presets. No pin-skew screen. If a rebuild misses HD HDMI / HD Dual, rerun those configs on another
seed, or try router1, before restructuring.

**Checks** (working files in `.heirloom_work/rb_starburst/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: every new knob path at RTL widths over all knob values (exp curves, step widths for every timing
  ID, K5 split, K4 depths, LFO products, warp and etch products, the gain index): no overflow; the photo values
  are exact (N16 65, TS8 63, GQ 328, Etch depth 15, warp 0, swing 0, breathing 1638 at the default speed).
- Default frame (GHDL 720p60 dec 1, Osc, CMYK, 2 warm-up + 1 capture, production v0.5.1 simulated the same way):
  raw 10-bit output **byte-identical** (0 of 921,600 px; a real frame, Y mean 479, 43 % bright).
- Not verified in sim (reasoned from the code): the swing at depth, the mirror warp, Square, the extreme exp
  settings.

## Reading of the photo (measured)
Work files: `.heirloom_work/rb_starburst/` (`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).
- **Mirrors.** A four-fold kaleidoscope. Flip-correlation puts the vertical mirror axes at x = 493.5,
  963.5 and 1440 and the horizontal ones at y = 275.5, 539 and 803.5. The translation period is
  946 px × 528 lines, so a mirror cell is 473 × 264 px: 4.1 cells per screen height, aspect 1.79.
  The big white bowtie X sits on the u = 0 corner of the cell (493, 275), the grey X on the u = 1
  corner at the screen centre (963, 539).
- **Everything is in the folded source.** High-pass luma correlates 0.95 across a mirror and 0.93 /
  0.89 across one period: the teeth and feathers are part of the mirrored pattern, not noise over it.
- **Palette** (core medians of a 12-colour k-means, RGB): white (253,254,255), lilac-grey (227,212,226),
  grey (149,143,149), dark grey (102,98,104), charcoal (62,61,62), black (25,27,14), yellow (254,255,43),
  magenta (237,1,248), lavender (155,107,249), cyan (74,245,248). Thin olive (black→yellow) and pink
  (yellow→magenta) fringes. Shares about 9–12 % each.
- **One colourizer cycle** (label adjacency): white → lilac → grey → dark → charcoal → black (the
  bowties) → yellow → pink → magenta → lavender → cyan → white (the chevrons and the diamond rings).
  Fitted band starts: 0, .05, .13, .21, .28, .36, .42, .53, .545, .645, .675, .79 of a cycle.
- **Teeth.** The horizontal edges above and below the big X's are yellow-and-black sawtooth teeth,
  period 33 px (row ACF 0.39–0.42): an H oscillator thrown into the V ramp. It is a chirped saw,
  about 14 teeth per cell, on the left half of the cell.
- **Feathers and starburst.** The grey X is feathered with a 21-line vertical period (ACF 0.41); the
  bowtie "bricks" have 24–25 lines. This is the V oscillator thrown into the H ramp. A yellow
  starburst sparks off the X arms at (u 0.76, v 0.46).
- **No tearing:** per-line shifts are only ±1–2 px.
- **Source field.** No analytic form fits (superellipse 0.34 label purity, saddle 0.32). The source
  is an 81 × 49 knot grid fitted to the median of the 25 folded copies, by ICM in colour space under
  the cyclic colourizer, with white peak/valley hints and the tooth displacement in the model
  (RMS about 34/255).

## What it does
1. **Mirrors.** 20-bit H and V phase ramps (period two cells, virtual-1080 steps) are folded into the
   16-bit cell coordinates u, v. Tile mode uses the ramps unfolded.
2. **Line engine.** Kicked at each line start, it computes the NEXT line in about 140 clocks (fits
   SD's 858). Steps:
   - v plus the drift pan, folded; the starburst tent's v term.
   - The V-tooth saw `0.5 − frac(12.6·ts·v + crawl)` × feather depth, and the per-line fur noise
     (xorshift, held 2–6 lines, restarted every field) × fur depth.
   - Then, for the 81 knot columns: a chirped tooth-phase DDA (`φ += D; D += DD`) gives the saw. Times
     the tooth depth (fading to 0 over 8 columns before the mask column), it displaces v; fold.
   - ×48 → row and fraction; read the two neighbouring rows in one clock (row-parity ROMs); lerp;
     write F8.4 into a ping-pong row RAM (even/odd column RAMs).
3. **Pixel path.**
   - u plus the drift pan, folded.
   - Feather mask `clip((u − u0)·10) + 2·tent` and fur mask `clip((u − .45)·8) + 4·tent`, with
     `tent = tv − |u − .76|·8`. Those masks times the line's feather and fur values displace u; fold.
   - u × 80 → column and fraction; read columns iu and iu + 1; lerp → F.
   - F + (luma − 502) × Etch − pivot, × the Bands gain, + the breathing offset → a colourizer index
     (one cycle = 256).
   - 11 band thresholds → band; colour ROM {Greys, palette, band}.
   - Output conditioning, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Cells | The mirror lattice: cells per screen height 2–8 (4.1 = the photo), so more or fewer X's and diamonds |
| K2 Bands | The colour band sequence: colourizer gain 0.5×–3×. Fewer, broader bands (yellow/magenta dominate) … many fine rings |
| K3 Teeth | The yellow/black sawtooth teeth and the grey X's feathers: count 0.5×–2× (1× ≈ 14 teeth per cell) |
| K4 Speed | Bipolar, still at the centre (±8). The teeth crawl along the edges, the feathers roll and the bands breathe (±0.025 cycle); the sign sets the direction |
| K6 Etch | How hard the picture bends the bands: camera luma (Video) or the oscillator (Osc) added to the field, up to ±0.27 cycle |
| K5 Palette | Band colours: CMYK (the photo), Ember, Ice, Acid, Rose, Mono, Neon, Gold |
| S7 Centre | Which X sits at the screen centre: Grey X (the photo) / White X |
| S8 Greys | The bowties: Grey (the photo) / Colour (grey bands recoloured from the palette's accents) |
| S9 Source | Video (default) / Osc |
| S10 Drift | The pattern drifts under the mirrors (±0.22 cell in u, ±0.14 in v; triangle LFOs of about 10 s and 14 s): X arms swell and shrink, diamonds bloom |
| S11 Mirror | Fold (the kaleidoscope) / Tile (ramps unfolded: the raw cell repeated with hard seams) |
| P12 Burst | Headline: tooth depth, feathers and fur. 0 % = calm, barely serrated bands; 42 % = the photo; past 50 % the teeth spread across the whole cell and deepen to about 0.22 cell and the feathers take the right half; 100 % = every X and chevron shredded into spikes |

Glides (1/8 of the gap per field, ±3 hold) on K1, K2, K3, K5 and P12. At P12 = 0 the teeth keep a
0.02-cell floor so K3 still shows, and Etch stays live in Osc mode.

## Video
- **Video** (default): the camera luma, read from a 256-entry ring at the same x, is added to the
  field. The picture shows as contours bending through the kaleidoscope (face edges become band
  edges); at the default Etch the photo's composition still reads.
- **Osc**: two drifting triangle waves (341 px and 205 lines) feed the same path, so the bands bend in
  slow waves and Etch stays live.

## Engine and contracts
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks, then about 530 clocks).
  - Glides: one shared datapath, four clocks per channel.
  - One serial shift-add multiplier, 15 slots of 32 clocks, with the slot decode registered a clock
    ahead. It computes the H/V phase steps and centred start phases, the tooth DDA constants, the
    feather count, the Burst curves (p², tooth / feather / fur depth, mask positions), the gain, and
    the drift and breathing LFOs (bit-logic triangles).
  - The line engine has its own 13-step serial multiplier for its three per-line products.
- **ROMs** (`starburst.py` → `starburst_pkg.vhd`): field 2 × 2048 × 8 (8 EBR); colour 256 × 30
  (2 EBR); the band map is 11 threshold comparators (`C_BT`).
- **RAMs:** row 2 × 128 × 12 and luma ring 256 × 8 (3 EBR). All are collision-free, with
  `no_rw_check`.
- **Contracts:**
  - The heirloom generated avid.
  - One 36-clock latency in every mode.
  - Chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1] and luma 13/16 (whites ~777) before the blanking gate.
  - Colours in the hardware convention (U register = Cr).
- **Resolution:** geometry is in virtual 1080-line units, so the cells keep the same shape at SD, 720p
  and 1080p. **SD 4:3 shows the central 3 of the 4 cells across** (cells sized to the height, sides
  cropped); 720p matches 1080p.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 80.44, SD Analog s1 78.00, HD HDMI s1 79.43, SD HDMI s1 78.17, HD Dual s2 82.17, SD Dual s1 82.05 MHz; 6829-6866 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-21, NOT flashed.** router2,
  `SEED=3 MAKE_TIMEOUT=900 ./build_heirloom.sh starburst`.
- **Build seeds: all six configs = seed 3** (first try). On seed 1 router2 stalls on HD Analog, and
  seed 2 gave HD Analog only 74.77 MHz.
- Fmax (post-route): HD Analog 83.66, HD HDMI 83.64, HD Dual 79.66, SD Analog 79.97, SD HDMI 83.19,
  SD Dual 84.70 MHz.
- 6824–6854 / 7680 LC (89 %), 13/32 EBR. Every bitstream passes iceunpack.
- RTL GHDL renders (720p60, decimation 1, Osc and Video) match the float prototype and the integer
  model (`imodel.py`). Motion: 17 % of pixels change over 6 frames at default knobs; Speed 0 + Etch 0
  is still.
- **Fit / timing history:**
  - First build: 8395 LC (109 %). Fixes: fold as XOR, band ROM → comparators, shared glides,
    serialised line products, narrowed pixel products, column counters.
  - First timing: 68 MHz. Fixes: column multiply split into nibble partials, glides over four clocks,
    registered slot decode, adder-free LFOs, an extra register after the luma ring (its read register
    had been absorbed into the EBR), a staged etch subtract.

## Deviations from the photo
- **The feathers inside the grey X and the stair-steps of the bowtie bricks are approximated.** The
  knot grid (about 6 px per knot) can't carry 21-line detail, and a full-cell feather oscillator tore
  the smooth chevrons beside it. The feather saw is masked to the grey-X zone (u > 0.78) and the
  starburst; static fur noise covers the right half.
- **Teeth are displaced per knot column** (80 per cell), so a tooth's reset edge is about 6 px wide at
  1080p, and K3 tops out at 2×.
- **All luma is scaled 13/16** (the HD-HDMI bar fix): whites are about 777, and the photo's camera-blown
  yellow reads as a darker gold. Edges are crisper than the camera-blurred photo.
- **The lattice is screen-centred.** The photo's is 7 px right of centre and 13 lines down from the
  capture's black top band, and its cell is 1.4 % larger than the default Cells.

## Open questions for Ron (2026-11-09 batch)
1. S7 Centre (grey X / white X at the screen centre) was replaced by the Teeth Saw / Square switch. Want Centre
   back, for example in place of Square, or as a third option on K6's palette knob?
2. K3 Teeth still only retextures the serrated edges and the grey X's feathers (its top is held at ×2 because the
   80 knot columns alias finer teeth). Would you rather K3 drive something bigger, such as how far across the cell
   the teeth reach, with P12 keeping only the depth?
3. The K4 swing returns home on its own when Speed comes back inside 376..648 (its depth follows the excess
   speed). Would you prefer a free-running turn that stays where you leave it?
4. K5's mirror warp pushes the kaleidoscope sideways only. Should the camera also push it up and down (the line
   engine would need the camera on the previous line)?

## Hardware watch list
- Renamed / moved: **S7 Centre → Teeth (Saw / Square)**; **Source moved S9 → S11** and **Mirror S11 → S9** (as in
  every other heirloom program); K1 / K2 / K3 / K5 defaults moved to 512 (the same picture), readouts −100..100
  (K5 0..100 %). Presets re-aimed: "Giant X", "Kaleido Swing", "Castle" are new.
- New motion only past the photo's speed: with K4 outside 376..648 the whole pattern swings (up to ±370 px) and the
  colours sweep. If the default picture moves more than v0.5's, K4 isn't at the default.
- The mirror warp reads the camera a few px ahead of the beam (invisible) and is 0 below Etch 50 %.
- Chroma filter delay now in a block-RAM ring (watch for colour shifts or thin coloured lines; none expected).
- At K1 100 % (16 cells) the knot grid is 1.5 px per column and K2 100 % gives ~5 colour cycles: busy, by design.
- Timing: see the build line (router2, base seed 3).

## Next step
Wait for Ron's preview notes. Rebuild the diag with `SEED=3 MAKE_TIMEOUT=900 .heirloom_work/diag_build.sh starburst`.
On approval: `./heirloom_diag.sh merge starburst`, then repack the previewed diag bitstreams under the production
TOML (copy `build/programs/ron/starburst_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
`program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`) instead of re-routing;
update the production SUMMARY / README / PHASE2 row and commit only starburst's files.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Here the camera strength was on K5 (Etch), so K5 and K6 swapped: **K6 = Etch** (the camera strength, default now 512 = 50 %), **K5 = Palette** (was K6). Nothing else changed: the VHDL swaps the two knob inputs (the processing is identical), the TOML blocks and preset keys moved with them (the Controls rows above are renumbered). v0.6.1, rebuilt with router1: all six seed 1; HD worst 79.17 MHz (HD Analog 84.61, HD HDMI 85.80, HD Dual 79.17), LC LC. Rebuild: `NEXTPNR_ROUTER="--router router1" ./heirloom_diag.sh build starburst`. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 loads at 100 % (initial_value 1023), and bend / element programs get a 25 % stronger camera at the top of the knob.
- **Bend.** K6 Etch bends the bands by the camera (or Osc) luma, and past 50 % the same luma bends the mirrors (the warp).
- **Change.** The production change is g + g/4 on the etch depth. Here both camera gains get it, at zone clock 26 (one small op each, after the zone-25 decode), so both are ×1.25 at every position.
  - Etch depth `s_dep`: 0 .. 157 at 100 % (was 0 .. 126), widened to 8 bits. |vp| ≤ 129 × 157 = 20253 in its 18-bit register; F + vp/8 ≤ 12.8 k in 15 bits.
  - Mirror warp `s_wg`: 0 .. 318 (was 0 .. 255), widened to 9 bits. |vw| ≤ 41022 in its 18-bit register, added to the modular 17-bit mirror coordinate as before.
  - Latency unchanged.
- **Build** v0.6.2 (`k6build.sh diag starburst`), router1. HD Analog and HD HDMI seed 2, the rest seed 1. HD Analog 82.15, HD HDMI 81.63, HD Dual 87.05 MHz (worst HD 81.63). 7311–7360 / 7680 LC. `out/rev_b/ron/_diag/starburst_diag_v0.6.2.vmprog`. Not flashed. No render (diag copy).
