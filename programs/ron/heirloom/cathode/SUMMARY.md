# Cathode — a cycling S&H colour mosaic with three-tube purple flames

**v0.6.0 (2026-10-31, phase 2 round 1): built, NOT flashed** (see [Phase 2](#phase-2)).
v0.5.0 (2026-10-27) was the rebuild from the photo. Heirloom #23, from
`photos/WIN_20230407_23_48_05_Pro.jpg`. The v0.1.0 version (posterized video plus two parabolic
tubes) was judged too abstract and replaced. The photo is a sample-and-hold colour mosaic whose
colours cycle down every column, with two purple flames keyed over it. Each flame is three shaded
tubes. Presets are hand-written placeholders.

## Reading of the photo (measured)
The work is in `.heirloom_work/rb_cathode/` (notes.md):
- `rows.py` / `cols2.py` / `grid.py` / `letters.py` / `colfit3.py`: the cell grid and colourizer.
- `tubefit2.py` / `edgeprof.py`: the flames.
- `fringe.py` / `spikes.py`: the fringes.
- `flstat.py` / `share.py`: statistics against the photo.

**Cell grid.**
- Cells are 42.2 px × 13.88 lines: 45.5 across and 77.8 rows. The first boundaries are at x 12 and
  y 10. Row edges are hard. The per-row horizontal jitter is about 2 px (sd).

**Colourizer.** The mosaic is a CYCLIC four-colour map of a per-cell phase:
- The four colours are green (0,255,0), pink (255,12,188), yellow (254,240,1) and mint (2,250,135),
  then back to green.
- It is a 2-bit Gray code, and the red channel is one of the two bits.
- In-between colours appear at about 10 %: orange (242,117,54) between pink and yellow, lime
  (130,229,31) between yellow and mint, and a violet-magenta (232,11,254).
- Centre-of-frame shares: green 28–33 %, yellow 27 %, pink 14–17 %, mint 13–14 %, violet-magenta
  3–5 %, lime 4 %, orange 3 %.

**Columns.** Each column's phase ramps down the screen at its own rate: 0, ±1/32, ±1/8, ±1/4 or
±1/4 ± 1/32 cycle per row, i.e. a vertical ramp oscillator times a horizontal sample-and-hold
waveform.
- The per-column fits match 0.56–1.00 of cells (about 0.7 in the centre).
- x 50–135 is solid yellow.
- The column at x ≈ 120 runs the whole green → pink → yellow → mint cycle in 8-row bands; the column
  at x ≈ 1860 runs it backwards.
- What the ramps don't explain is mostly the red bit (pink snapping to green, mint to yellow).

**Fringe.** A blue spike sits at the leading edge of every green cell entered from another colour:
- blue 0.94, green dipping to 0.2, about 12 px wide, centred 4 px into the cell;
- present at 93–100 % of boundaries into green, 19 % of green→green, never green→other.
- Other transitions are 8–12 px wide (10–90 %).

**Flames.** The pattern repeats horizontally every 947 px (teardrop centres 535 and 1482). Each flame
is three opaque tubes:
- a teardrop: centre +4, half-width 82, ending at y 670;
- a body: centre +168 → 0 at y 1100, half-width 57–70, with a magenta stripe (R 0.86, about 20 px)
  on its left edge;
- a left arm: centre −184 → −56, half-width 20 → 44 → 30, ending about y 880.

**Tube shading.** Edges ramp over about 30 px with a 16 px dark rim. Blue is saturated inside. Red
peaks at 0.57 at the teardrop centre and is about 0.4 in the body and 0.36 in the arm.

**Inside the flames' outline.**
- The mosaic loses its green and its blue becomes its red: pink and yellow cells turn magenta,
  green and mint cells turn black, and the blue fringe lines survive.
- In the upper gaps this shows as 4-on/4-off magenta stripes; the lower V between the tubes is black.
- A dark-green halo fades the mosaic back over about 80 px outside the outline.
- Per 120-line band, the purple / magenta / dark shares in the flame windows match the photo within
  a few %: top band 60/11/12 in the photo against 57/11/14 here.

## What it does
1. **Cell grid.** A Q16 cell-phase accumulator anchored at virtual x 12, with cell numbers 0–63. A
   row accumulator anchored at virtual y 10 marks each row's first line.
2. **Phase per cell.** `phi = ph(i)·128 + rate(i)·Jk (mod 8192)`.
   - `ph` and `rate` come from a 64-entry column table fitted to the photo.
   - `Jk = ((row << 8) + roll)·cycle >> 8`.
   - The top two bits of `phi` pick green / pink / yellow / mint.
3. **Osc noise.** A hash of (cell, row, seed) applies the photo's red-bit statistics, at a per-column
   level from the table:
   - mint → yellow or pink → green with probability 0.75·n / 0.44·n;
   - yellow → orange or lime with 0.36·n;
   - pink → violet-magenta with 0.3·n;
   - the flame-region columns also break up below row 48.
4. **Video.** The cell-sampled camera luma (held per row through a 64-entry cell RAM) sets the red
   bit: on above its running mean, with an in-between colour within ±10 of the mean. The ramp keeps
   the other bit.
5. **Edges and fringe.** An 8-entry RGB palette. The blue fringe covers the first 7 virtual px of a
   green cell entered from another colour (S7). Then a 2-px box.
6. **Flame tables.** Per line: five 80-entry tables at 16-line steps (teardrop width, body centre and
   width, arm centre and width), linearly interpolated at `y / length` and scaled by `width`. The
   flame coordinate is periodic with period `W / count`.
   - **Motion (v0.6).** A wave runs down each tube from its base to its tip: 1.45 turns down the
     screen, its amplitude growing from 0.19× at the base to 1× at the screen's far edge. Each tube
     rides it at its own phase and rate, so the three writhe out of step and cross.
   - Each tube's length flickers ±32 on `sy8`, on a triangle LFO taken from the next tube's wave
     phase, and its width scale moves the other way (thinner as it stretches).
7. **Tubes per pixel.** Three tubes in parallel; teardrop over body over arm, all opaque.
   - Blue ramps over the rim (±16 px).
   - Red is a shift-add tent (the body's left 22 px are magenta). In Video mode red is the live
     camera luma.
8. **Halo key.** K in 32nds; the halo fades 3.25 per px (S8 Off: a 4-px edge).
   - Mosaic green becomes G·(1−K); blue becomes a lerp toward max(R, fringe).
   - Lower flame (table-y ≥ 720): the inside of the outline is black.
9. **Output.** Tubes over the keyed mosaic → RGB patch (K6) → YUV in the hardware convention (u = Cr)
   with luma pre-boosted 16/13 and capped at 940 (white 777 after the shell's 13/16) → shell
   conditioning → blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Cells | the mosaic cells | cell width 160 → 12 px, square law (42.2 px at 487) |
| K2 Rows | the mosaic rows | row height 64 → 4 lines, square law (13.9 at 502) |
| K3 Cycle | the colours cycling down each column | ramp gain 0 → 3× (1× at 342). 0 = every column one flat colour |
| K4 Speed | motion | bipolar, still in the centre ±8: column colours roll down/up, noisy cells re-roll, the flames writhe (tube waves at 0.75 / 1 / 0.625 × speed, travelling base → tip; tip → base when turned left) and their tips lick |
| K5 Flames | the two flames | flames across, 1 → 4 (2.03 at 351) |
| K6 Patch | the whole palette | RGB cable patch: RGB (photo) / RBG / GRB / GBR / BRG / BGR. A labelled default (index < 6) is read as its index |
| S7 Fringe | the blue spike on green cells | Blue (photo) / Off |
| S8 Glow | the dark-green halo | On (photo) / Off (hard 4-px key edge) |
| S9 Core | the teardrop tube | On (photo) / Off |
| S10 Flip | the flames' direction | Down (photo) / Up |
| S11 Source | the mosaic's red bit, the tubes' red | Video (default) / Osc |
| P12 Flames | the flames (headline) | 0 % = short stubs at the top (×0.35 length, ×0.75 width, barely writhing); 50 % = the photo's flames, writhing up to ~73 px; 100 % = two tall purple pillars (×2.2, ×1.25) writhing like tentacles (up to ~135 px), with the mosaic only between them |

Defaults: K1 487, K2 502, K3 342, K4 600, K5 351, labelled switches and K6 at label 0, P12 512.

## Video
The photo's mosaic is synthetic: per-column ramps no camera scene would produce. So the camera is
woven in rather than used as the mosaic's source.
- **Mosaic.** The camera's cell-sampled luma, held for each row through a 64-entry cell RAM, drives
  the colourizer's red bit. Cells above the picture's running mean turn pink or yellow, cells below
  turn green or mint, and those near the mean get orange or lime. The column ramps keep the other
  bit, so the photo's column rhythm stays.
- **Tubes.** The tubes' red is the live camera luma at full resolution, a purple/blue duotone
  picture inside the flames.
- **Tested alternatives.** Adding luma to the phase, or replacing the phase with luma, were both
  illegible (`cmp_pfv*.png`).
- **Osc.** Substitutes the photo-fitted per-column base phase plus the hashed red-bit noise. Every
  mosaic control (K1, K2, K3, K4) acts identically in both modes.

## Engine
- **Field sequencer.** Runs in the vblank zone, about 260 clocks, with one serial shift-add
  multiplier (25-bit) and one restoring divider (quotient shifted into the dividend register). It
  computes:
  - the cell and row increments (two-step k² products) and the increment per real line;
  - length (from P12), `isy = 65536/length`, width;
  - the virtual width and flame period `Pf = Wv·1024/(1024 + 3·k5)`, and the flame origin;
  - three per-tube `isy = 65536 / (sy8 ± tri)` divisions (the length flicker) and the writhe
    amplitude `aw = 16 + 3·p12/16`;
  - the grid anchors and the cycle gain.
  - Motion: roll += sp/8 + sp/32 (Q8 rows); re-seed every 8192/|sp| fields; the three tube
    phases (10-bit turn) += sp × 3/16, sp/4, sp × 5/32.
- **Per-line engine.** Kicked at the source avid rise; it computes the next line in about 260 clocks
  (inside SD's 858). It does:
  - the row step and first-line flag;
  - Jk;
  - the wave position (yv·11/8) and amplitude (aw·(yv + 256)/2048), then per tube a parabolic
    sine h·(512 − h) of (position − phase) times the amplitude, which becomes the tube's centre
    offset;
  - five table reads (one flame-table EBR), each at its own tube's length (yv·isy), with 4-bit
    linear interpolation, ×width for the centres and ×(width ∓ tri) for the widths;
  - the outline left/right edges, plus the lower-flame and alive flags.
- **Pixel path, 24 stages, one latency in every mode:**
  - 1–6: 16-bit 3-round add/xorshift hash; column ROM; rate × Jk as shift-adds; noise thresholds;
    tube distances and edge ramps; halo K.
  - 7: colour index (Osc noise or Video red bit) and tube priority.
  - 8: previous-cell tracking and the fringe; tube red = r·(eb >> 2) >> 3.
  - 9: palette. 10: 2-px box. 11–13: keyed lerps on K. 14: tube mux and patch.
  - 15–17: RGB→YUV in Q6.
  - 18–23: the shell's chroma [1 2 2 2 2 2 2 2 1]/16 and luma [1 2 1].
  - 24: luma 13/16 and the blanking gate on the generated avid.
- **Contracts.** damier's generated avid; virtual 1080-line coordinates from the timing ID; the cell
  RAM carries `no_rw_check`, since its reads on first-of-row lines are discarded.
- **Fit history.**
  - First build: 7846 LC (102 %; the core alone is 1145).
  - Cut to about 7020 by: the 16-bit hash; a 2-px box (was 4); latency 28 → 24; K3/K5 latched rather
    than glided; no length flicker; narrower serial units; 8-bit line registers; the halo in 32nds
    with one lerp multiply each; tube red as an 8×4 multiply; YUV in Q6.
- **Hash.** Correlations < 0.05 (`imodel.py` checks).

## Phase 2
Ron's note 2026-10-31 (on v0.5.0): instead of the flames just lightly swaying back and forth, they
need more dynamic shape and movement. His biggest complaint with the program so far.

**v0.6.0 (round 1):**
- **The sway is gone.** It bent the whole flame group by ±40 px at the bottom, slowly.
- **A wave runs down each tube** from its base to its tip (1.45 turns down the screen), its
  amplitude growing toward the tip: 0.19× at the base, 1× at the screen's far edge. In virtual
  pixels: up to ~73 px at the photo's P12, 10 at 0 %, ~135 at 100 %.
- **Each tube rides its own phase** at 0.75 / 1 / 0.625 of the K4 rate (teardrop / body / arm),
  starting a third of a turn apart. The tubes writhe out of step and cross. Teardrop over body
  over arm stays the priority, so they pass in front of each other, and the magenta / black gaps
  between them open and close. At the default speed the body wave takes ~0.8 s per turn.
- **Tips lick.** Each tube's length moves ±32 on `sy8` (±12.5 % at the photo's length, ±35 % for
  the 0 % stubs) on a triangle LFO, the next tube's wave phase. Its width scale moves ∓32, so a
  tube thins as it stretches.
- **K4 turned left** runs the waves tip → base. The centre (±8) still freezes everything.
- **P12** now also sets the writhe: `aw = 16 + 3·p12/16`. 100 % gives tall pillars writhing like
  tentacles.
- **Controls unchanged otherwise.** No control was re-assigned. The amount of writhe rides on P12
  rather than a knob of its own; if Ron wants it separate, a control would have to give way.
- **Implementation.** The line engine computes the wave position and amplitude per line, then for
  each tube a parabolic sine h·(512 − h) of (position − phase) times the amplitude. That becomes
  the tube's centre offset (the teardrop centre is now a per-line value). The five table reads each
  use their own tube's `isy`. The field sequencer does three `isy` divisions instead of one. The
  sway's sine ROM, vy² product and per-line flame-origin wrap are gone, and the beam takes the
  field's flame origin directly.
- **Fit.** 7298 LC (95 %, v0.5: 7020). Trims: the tube offsets go straight into the next-line
  centre registers; 10-bit phases; the amplitude's +256 as an accumulator preload. A `rom_style`
  attribute did not move the column table (64 × 15) into EBR.
- **Checks.**
  - `proto6.py` (imodel.py with the new flame functions) gave the amplitudes: `p6_motion.png`,
    `p6_amp.png`, `p6_int.png`.
  - The GHDL render (720p60, decimation 1, Osc, defaults; `rtl6_osc_def.png`) draws the writhing
    tubes, halo and dark lower gaps as the proto does. K6 sat at 512 in that sim (patch GBR), so
    its colours are green / red; on hardware K6 defaults to RGB.

## Status
- **v0.6.0** built 2026-10-31, router2. 7268–7298 / 7680 LC (95 %), 2 / 32 EBR, iceunpack OK × 6.
  - Builder seeds: HD Analog s1 75.79, SD Analog **s2** 85.67 (s1 ran into the 360 s timeout),
    HD HDMI s2 77.91, SD HDMI s1 82.77, HD Dual s1 79.33, SD Dual s1 84.35 MHz.
  - **HD seeds re-picked for HDMI pin skew** (router2 screen of HD HDMI s1–24 and HD Analog / HD
    Dual s1–12, `.heirloom_work/rb_cathode/skew/screen.txt`), hand-installed and repacked:
    - HD HDMI **s8**: 79.42 MHz, HSYNC +2.14 / data ≤ −1.00 ns. The only seed inside the window;
      the builder's s2 sat at −0.66 / ≤ −0.58.
    - HD Analog **s9**: 77.96 MHz, HDMI HSYNC −1.01 / data ≤ −1.26 ns (s1 was −1.15 / ≤ −0.87).
    - HD Dual **s2**: 79.95 MHz, −2.93 / ≤ −1.86 ns (s1's data reached −0.34).
  - A plain rebuild gives the builder seeds; to reproduce the shipped HD bitstreams, route the
    regenerated json with `skew/route.sh` (`RT=router2`) at those seeds (byte-exact: s1 / s2
    reproduced the builder's bins). The builder's vmprog is kept as
    `skew/cathode_v060_builder.vmprog`.
  - `out/rev_b/ron/heirloom/cathode.vmprog` (438,307 bytes). Not flashed.
- **v0.5.0** built 2026-10-27, router2, all six configs on **seed 1**.
- Fmax (last post-route line): HD Analog 86.16, **HD HDMI 75.15 (thin)**, HD Dual 79.65, SD Analog
  77.94, SD HDMI 81.56, SD Dual 79.02 MHz.
- 7020–7041 / 7680 LC (91.6 %), 2 / 32 EBR, iceunpack OK × 6.
- `out/rev_b/ron/heirloom/cathode.vmprog` (426,527 bytes). Not flashed.
- An earlier build of nearly the same RTL needed seed 2 on HD HDMI, and on SD HDMI, where seed 1
  stalled router2. If HD HDMI regresses, try `SEED=2` or router1 before restructuring.
- **Verified in GHDL** at 720p60 decimation 1 (full virtual canvas):
  - Osc defaults match the float proto and the integer model;
  - Video (selfie) shows the camera in the tubes;
  - motion: 5 % of pixels (19 % in the centre) change over 6 fields at default speed;
  - NTSC: each field is the full picture.

## Deviations from the photo
- **Mosaic realisation.** The cells match by statistics, not one for one: colour shares within
  about 3 %, 4×4 region means within about 20 levels. The fitted column table reproduces the column
  structure (yellow column, cycling columns, green column) at 0.56–1.0 per column.
- **Identical flames.** The pair is one periodic pattern. In the photo the right flame's left arm is
  dimmer and partly hidden, and the two differ slightly.
- **Opaque tubes.** The photo has magenta mosaic blocks crossing the lower teardrop in some rows;
  here they show only in the gaps.
- **Edges.** A 2-px box plus the shell filters gives crisper luma edges than the photo's ~10 px
  (chroma ~9 px).
- **Top end.** Yellow peaks at 777 output luma (the HDMI-bar scale) against about 810 in the photo.
- **Video.** It can't be "the photo's processing on the camera", because the photo's mosaic is
  synthetic. The camera enters as the red bit and the tubes' red. At default Cycle the column
  ramps dominate; lower K3 shows the camera's posterized shapes.
- **Resolution.** 720p is the 1080p picture. On SD (4:3, 1440 virtual px) the mosaic shows the
  photo's left 34 of 45 columns, and the flames (sized to height) take a larger share of the width.
- **Grid jitter.** The per-row sampling jitter (about 2 px) isn't modelled.

## Working files
`.heirloom_work/rb_cathode/`:
- `ref.jpg`: the photo.
- `notes.md`: look, measurements, proto log, controls, model, RTL, build.
- `proto_final.py` / `proto_final.png`: the float proto. `sheet.py` / `sheet.png`: the control
  sheet.
- `imodel.py` and `tables.py`: the integer model and the fitted tables; `mkvhd_tables.py` emits the
  VHDL constants.
- `rtl_osc_final.png`, `rtl_vid_final.png`, `rtl_mot_final.png`, `rtl_sd_final.png`: GHDL renders.
- `cmp_final.png`: photo / proto / RTL Osc / RTL Video.
- `ys/pack.sh`: a 75-s LC count (full yosys + `nextpnr --pack-only`).
- `old_v010_*`: the replaced v0.1.0.
