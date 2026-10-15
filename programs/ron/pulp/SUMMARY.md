# PULP — an authentic newsprint / comic book press

**v1.0.0 FINAL** — signed off by Ron on hardware, 2026-10-11. Same bitstreams
as the v0.2.0 finalization pass (hardware-contract fixes, control fixes, HD
HDMI bar conditioning, S11 Spot); only the TOML changed (version, preset 6).
All 6 configs routed at full clock on **seed 1** with router1 — repack only
for TOML edits (`vmprog_pack.py`), so the screened HD HDMI placement stays.

v0.1.0 (2026-07-16): HW-tested, look approved; the reported blue flicker (an
interlace screen-phase bug) was fixed but never re-tested. v0.1 sources and
vmprog are kept in `.pulp_work/v010/`.

Reproduces incoming video as a 1965 four-colour newspaper page: clustered-dot
halftone screens at the classic press angles (C 15°, M 75°, Y 0°, K 45° —
rosettes emerge from the interference), a symmetric spot function (ink dots in
highlights, checkerboard at exactly 50%, **white dots on solid ink** in
shadows), dot gain, inks multiplied over warm pulp paper, plate
misregistration, ragged dot perimeters, paper grain. On top: comic posterize
flats, solid ink linework, panel frames with paper gutters, and speech /
thought / starburst bubbles with baked-in grawlix lettering. P12 **Press**
runs the image from nearly-clean video through newsprint to Ben-Day pop art.

## Controls

| Control | Function |
|---|---|
| K1 Screen | halftone cell 8..96 px (16-step exponential ladder; knob up = bigger cells) |
| K2 Shape | dot shape: line / diamond / round / elliptical / square (5 zones) |
| K3 Ink | density: threshold bias + dot gain |
| K4 Paper | 4 stocks: bright white → cream → aged → pulp (grain rises) |
| K5 Linework | comic outline weight (off / hairline .. heavy, 0–3 px dilate) |
| K6 Register | plate misregistration (linear) |
| S7 Color | B&W single 45° screen ↔ CMYK process |
| S8 Posterize | full comic flats at any Press; off, the Press ramp tops out at half |
| S9 Panels | comic page layout — 8 templates, re-rolled every time the switch flips on |
| S10 Bubbles | speech balloon generator — 2–3 on screen, one changing ~every second |
| S11 Spot | two-colour press: black + spot red (with S7 B&W: one red ink) |
| P12 Press | master: clean video → newsprint → pop art (screen/ink/posterize macro) |

K1, K2 and K4 are stepped; each is held as a 16-zone index with ±8-count
hysteresis so pot noise parked on a step edge can't flip the whole screen.

## Look development

`pulp_sim.py` is the reference renderer (numpy/PIL, ~1080p stills in seconds).
It implements the spec look: p-norm morphable spot function with the
dc/dk centre/corner scheme, CMY(K) with GCR, multiplicative ink-over-paper,
misregistration with drift, AGED streaks/banding/dropouts, panels, procedural
bubbles with grawlix. Verified renders: B&W coarse w/ dot inversion, fine CMYK
rosettes, Lichtenstein pop-art, comic page w/ panels+bubbles, aged Sunday
paper, line-screen engraving (`sim_*.png`, `sim_tc_*` on the synthetic
portrait test card). Key sim lessons that shaped the curves:

* Pop-art (slider top) needs the dot-gain to RELAX past ~70% press or the
  page turns to mud; bold flats, not heavy ink.
* Ragged-edge noise amplitude must shrink as cells grow (fixed threshold
  noise reads as spray-paint fuzz on giant dots).
* Aged dropouts must sit on the rotated screen LATTICE, not the pixel grid.

## Hardware architecture (what actually fits an HX4K)

The first two VHDL attempts were honest transcriptions of the sim math and
came in at **21k / 14.5k LC demand against 7,680** — the writeup below is the
sequence of structural moves that got it to **~7,580 LCs (98.6%), 15/32 EBR**:

1. **Squared-distance spot engine** (the classic euclidean-composite dot)
   instead of approximated p-norm metrics: fold each cell coordinate to the
   nearer of centre/corner (one region bit), square the two shape-selected
   8-bit components, threshold `T = sum` (centre) / `1024 − sum` (corner).
   Exact circles, checkerboard at exactly 50%, white dots in shadows — and
   the only per-pixel multipliers in the design are these eight 8×8 squares.
   Dot shapes become an input select: line = v, diamond = u+v, round = (u,v),
   ellipse = (u,v) with a scale shift, square = max(u,v). K2 is 5 discrete
   zones (the continuous morph blend didn't earn its LCs).
2. **Plate algebra**: only C and K carry DDA accumulators. M (75°) is C (15°)
   reflected: `u_M = 2f·cos15·y − v_C`, `v_M = 2f·sin15·y − u_C` — two
   subtracts from per-line registers. K (45°) derives `v_K = 2f·cos45·y − u_K`
   from its own u. Y (0°) is a vertical line screen (u only, half a plate) —
   authentic for the yellow plate and invisible under the rosette.
3. **Ink-combination LUT in EBR**: the 16 C/M/Y/K overlap composites
   (paper × transmittance products, RGB→YUV) are built by the vblank
   sequencer through one shared 13×13 multiplier into a 16×30 BRAM; the
   pixel path reads it with the 4-bit ink combo as address (port shared with
   the sequencer — vblank reads are invisible). Keeping this LUT in
   registers cost ~1,200 LCs of mux trees.
4. **Bubbles as 2-bit sprite ROMs** (64×64, three shapes × 2 EBR): fill /
   outline / grawlix baked in, generated by `pulp_sim.py`'s own shape math,
   scaled ×2 (SD) / ×4 (HD) by shifts, tail mirrored by address reflection.
   Replaced ~1,400 LCs of SDF adders with ~300 LCs + 6 EBR.
5. **Roberts cross** instead of full Sobel for the linework (1 line buffer
   instead of 2, saves 4 EBR + ~200 LC). With threshold + dilate the comic
   outline look is indistinguishable.
6. Tone conditioning per plate: posterize off/half/full, dot gain as a
   single 4-position shift of `min(t, 1−t)` (provably can't overflow → no
   clamps), bias+noise folded once for all four plates.
7. One's-complement folds for `|frac − ½|` (off by 1 LSB in 512 — free).

## Interlace: the screen phase must be a pure function of (x, y)

**Hardware-reported bug (fixed): flickery horizontal lines over blue.**

The screen's per-line advance ran on *every* hsync — including the ~20
vertical-blanking lines. That made the dot phase a function of blanking
geometry rather than of `y`. In interlace one field's vsync coincides with
an hsync edge while the other's falls mid-line (that offset **is**
interlace), so the two fields see blanking-hsync counts differing by one,
and one field picked up an extra `4·Ac` of phase — a near-arbitrary
fraction of a cell (48% at an 8 px screen, 13% at 30 px). The fields' dot
grids then failed to interleave, adjacent display lines disagreed, and the
result was 30 Hz twitter.

Why **blue** specifically:
* Sky blue lands C at ~50%, which is exactly the **checkerboard** region of
  the spot function — where the pattern is most sensitive to line phase.
* Blue is drawn *only* by the two rotated screens (C 15°, M 75°). Y is a
  vertical line screen with no vertical structure at all, and K ≈ 0 there
  (GCR takes K from `min(C,M,Y)`, and blue's Y is near zero). Greys and
  neutrals are carried by K at 45° and don't show it.

The fix: advance the line registers when **active video ends**, never on a
blanking hsync. Phase is then pure in (x, y):

| | field 0, line L | field 1, line L |
|---|---|---|
| y | 2L | 2L+1 |
| phase | 4·Ac·L | 2·Ac + 4·Ac·L |
| = | 2·Ac·y ✓ | 2·Ac·y ✓ |

Both fields interleave into one coherent screen. Paper grain / ragged-edge
noise is likewise keyed to the true frame line (`s_aline(6..0) & field` — a
bit concat, free) so both fields agree on it: grain is a property of the
paper at (x, y), not of a field line.

**It also paid for itself**: folding the plate phase offsets into the vsync
init deleted three "use copy" registers (41 bits) and four line-rate adders,
and the enable is a free DFFE clock-enable — so the hsync load is now a
plain register copy with no logic on it at all.

**Structure matters more than the fix**: the first attempt guarded the
advance with a mux (`lu_c → adder → mux → acc`) and cost **8 MHz** — HD
Analog fell to 66 MHz. Moving the advance to end-of-active-video made the
enable free and the load logic-less, and recovered all of it.

## Bubbles: 3 at once for the price of the per-line trick (S10)

Three bubbles are live at a time (each slot ~75% occupied, so 2–3 on screen),
with one re-rolled round-robin every ~0.6–1.6 s.

The trick is the same one the panel engine uses: **a sprite is 128 field
lines tall (HD) / 64 (SD) but a third of the field is 180 / 80, so three
bubbles in three vertical bands can never overlap.** That means the active
slot can be chosen **per line** — two compares driving a RAM address — and
the entire per-pixel path (address, bounds test, ROM read, decode) is
completely unchanged. Three bubbles cost nothing per pixel.

Slot state lives in an **EBR, not registers**. As three register sets it
needed a 3-way write decode *and* a 25-bit 3:1 read mux (~125 LCs) and
overflowed the device (99.7%, two configs failing routed timing). In a block
RAM the storage, the write decode and the read mux all vanish into the
block; all that remains is the address. Only one x anchor is stored per slot
(`bcx ± half`): the mirror direction is fixed at spawn, so only one of the
two is ever used.

## Panel layout engine (S9)

Real comic-page grammar, **row-major** so it costs almost nothing per pixel:
the sequencer emits two row boundaries (`ry1`/`ry2`, parked at 1023 = off),
one column split per row — `rxaA` for the top row, `rxaB` for the rows below
(parked at 2047 = off) — and an `xgate`. Splits only apply right of `xgate`,
which is what expresses "tall panel on the left, stack of panels on the
right" without free-form rectangles.

Per **line** (in a free-running stage, settled during blanking): pick the
row's column split, compute the y gutter/border/margin/ring flags. Per
**pixel**: one distance compare against the line's column split, plus the
gate and the page-edge compares. That's it.

**Every panel is fully outlined.** The page edge gets the same two-level
treatment as an interior split — paper margin, then a border ring — so
panels touching the page edge are boxed exactly like interior ones. It's
cheap because `|x - 0| = x` and `|x - W| = W - x`, so the edges need no
distance math, just pre-computed thresholds. A row or column gutter crossing
the ring still wins (the ring never clears `v_gut`), which correctly breaks
the outline *between* panels instead of boxing the whole page.

**One gutter, everywhere.** Both axes derive from a single value: `guy` from
the field height, and `gux = 2·guy` **only when the source is interlaced**
(`s_w > 2·s_h`), because a field line covers ~2× the display height that a
pixel covers in width — that's what makes the gaps look square rather than
merely be equal in pixels. Progressive sources get `gux = guy`. And since a
split lays paper on *both* sides of its rule (`|x-P| < g` spans `2g-1` px),
the page margin is set to exactly `2g-1` — so the space around the page is
identical to the space between two panels. Measured, all modes: margin =
gap, left = right = top = bottom, to the pixel.

Eight layouts, rolled from the LFSR **on the rising edge of the switch** —
flip PANELS off and on and you get a different page:

| | layout |
|---|---|
| 0 | six even panels — 3 tiers of 2 |
| 1 | tall panel left + 3 stacked right |
| 2 | 2×2 grid |
| 3 | three newspaper strips |
| 4 | splash across the top + 2 below |
| 5 | 2 up top + splash across the bottom |
| 6 | staggered 2×2 (⅜ split top, ⅝ split bottom) |
| 7 | strip across the top + 2×2 page below (5 panels) |

`pulp_sim.py` carries the same eight templates for look-dev.

Each row carries exactly **one** column split. The second split existed only
to serve a 2×3 grid and cost ~170 LCs (a subtract, an abs, its compares, two
registers, and its slice of the 8-way template mux) — dropping it paid for
the border ring outright, and six panels as 3 tiers of 2 is the more classic
comic page anyway.

**Bug found via GHDL sim (fixed)**: only vertical splits ever appeared. Root
cause was the frame-height capture, not the layout logic — `s_h <= s_aline`
was latched at vsync, but the **first blanking hsync precedes vsync and has
already reset `s_aline` to 0**, so `s_h` read 0 every frame. That made
`ry1 = 0` (no horizontal split anywhere) and `s_hmg = 0 - guy` underflow to
~1019 (no bottom margin). `s_w` was unaffected because it's captured at
hsync on lines that had active video — which is exactly why the vertical
splits worked and the horizontal ones didn't. Height is now captured on
every active line (`s_h <= s_aline + 1`), where it's actually known; it
holds the last field's height through blanking, which is when the sequencer
reads it. Verified in GHDL: template 0 renders the 2×3 grid, template 1 the
tall-left + 3-right with the x-gate correctly stopping the horizontal rules.

Known cosmetic: thirds are `1/4+1/16+1/64` = 0.328, so 3-column layouts run
~236/248/236 instead of 240/240/240 (~5% wider middle panel). Left as-is —
it reads as hand-drawn, and at 98.8% LC the extra add isn't worth it.

Dry/wet Press blend: BRAM delay ring + 4-step shift blend with exact
endpoints. Latency 13 clocks.

### Documented approximations vs the sim
* Misregistration is screen-phase-only (both axes); true per-channel
  sampling shifts would need line buffers per channel. Phase dominates
  visually (verified in sim). No slow drift in v0.1.
* AGED (S11) was **removed** to pay for 3 simultaneous bubbles — the device
  was full. It had been dot dropouts + faded ink + raggeder edges. Removing
  it properly also let the K accumulator shrink 17→12 bits (the top 5 bits
  existed *only* to index the dropout hash's cell lattice) and collapsed the
  ink-transmittance registers back to constants — so it freed more than the
  ~101 LC an ablation predicted. S11 is now a free switch.
* Panels: per-panel screen re-seed cut (every panel prints on one press
  pass). Layouts are row-major, so a full-height panel is expressed as an
  x-gate rather than as free-form rectangles.
* Paper grain applies to everything (print mottle), not just open paper.

## Build

```
NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron pulp
```

**Use router1.** v0.2.0, all routed post-route, all **seed 1**:

| config | routed | LCs | required |
|---|---|---|---|
| HD Analog | **80.08** | 7,640 | 74.25 |
| HD HDMI | **77.74** | 7,630 | 74.25 |
| HD Dual | **77.72** | 7,623 | 74.25 |
| SD Analog / HDMI / Dual | 72.0 / 78.8 / 79.4 | 7,636–7,653 | 27 |

RAMs 17/32. 99.4–99.6% LC — the chip is FULL; anything new must pay for
itself. v0.1.0 on today's SDK with router1 seed 1 was 7,532 LC and 74.13 MHz
(FAIL) on HD Analog; on router2 it needed seeds 3/4/4 and passed HD HDMI by
0.03 MHz. v0.2 is +179 LUT / +29 FF net, yet routes 3–6 MHz faster.

HD HDMI pin-skew screen (`tools/hdmi_pin_skew.py`, router1 seeds 1–10; seed 5
stalled in router1 congestion): the shipped **seed 1** is the best placement —
HSYNC **+1.36 ns** (window +1.2..+2), closest data bit −0.57 (damier's clean
build was −0.56). Seeds 2/3/6/7 put HSYNC at +0.3..+0.9 and 4/8/9/10 below 0.
Only re-route HD HDMI with a hardware A/B.

v0.1.0 build record (router2): HD Analog seed 3 76.72, HD HDMI seed 4 74.28,
HD Dual seed 4 74.60, SD seed 1; 7,510–7,558 LC.

**Critical-path lesson**: at ~95% utilization the design first missed HD
timing (routed 69.7 vs pre-route estimate 79–81 — the two "Max frequency"
lines lesson again). The path was the bubble sprite addressing: negate →
variable shift → +32 bias → bounds compare, all in one stage. Folding the
mirror into the E1 subtract choice (`bcx ± half` precomputed per frame) and
pre-biasing the coordinates turned E2 into slice-selects and zero-tests —
that one change moved every config from ~70 to 77–86 MHz routed.

**Two more timing traps, both found by reading the critical path** (and
both costing several blind build cycles first — read the path *first*):
* `s_h → s_omxb`: the whole gutter derivation (add → aspect-mux → shift →
  subtract → add) sat in ONE vblank sequencer step. Vblank code is still
  STA-timed every clock. Split one-add-per-step (free — the registers
  already existed): the design's own header states this rule.
* `lut_ram RDATA → wy12`: a **variable barrel shift** (`grain >> s_gsh`)
  sitting after the ink-LUT BRAM read, which alone is 2.15 ns clk-to-q.
  The grain is latched on a 4px lattice and pipelined 7 stages, so the
  shift moved to latch time and the bubble lift folded in at E10 — leaving
  E11 a single add. **68 → 80 MHz pre-route, and −30 LCs.** Never leave a
  variable shift downstream of a BRAM read.

**Counter-lesson (measured, don't repeat)**: after the panel engine landed,
the per-line layout cone showed up as hd_dual's critical path, so it was
split across two registered stages — behaviourally free, since `al_nx` is
stable for a whole line. It made things *worse*: HD Analog and HD HDMI then
missed on all 6 seeds. At 98.8% LC the ~21 added registers cost more in
routing/placement pressure than the shorter logic path bought. Reverted to
the single stage. At this utilization, **remove logic, don't restructure it**.

## v0.2 finalization pass (2026-10-11)

An audit against the hardware contracts learned since July found Pulp was
designed around FIELDS (480i / 1080i) and predated most of them.

**HD (1080p) geometry**
* The screen DDAs stepped two frame rows per line unconditionally — right for
  an interlaced field, wrong for progressive. On 1080p every screen was
  squashed 2:1 vertically: flat oval dots, and the plate angles landed at
  ~28° / 82° / 63° instead of 15° / 75° / 45°, so no proper rosette. Now
  each advance is one row; it fires TWICE per line (4 and 5 clocks after avid
  falls) when the source is interlaced (`s_wide`: width > 2 × field height)
  and once otherwise. Two clock-enable terms; no registers, no mux on the
  load path.
* Bubbles were 256 px × 128 lines on 1080p (designed for 1080i fields).
  The sprite now scales x and y independently per mode — 1080p ×4/×4,
  1080i ×4/×2, 720p ×2/×2, SD ×2/×1 — square on screen and always shorter
  than a slot band.
* The line counter was 10 bits and capped at 1000: on 1080p the bottom 80
  lines reused one row of paper grain (vertical streaks) and the page read
  as 1000 tall (a ~93-line bottom margin against ~13 at the top). Now 11
  bits, no cap; every height register widened (`s_h`, row splits, bands,
  margins, slot anchor). Gutters re-derived from the true height
  (3 + h/128 lines; border 2/3/5 by height band).

**Hardware contracts**
* Blanking gate added (neutral Y 64 / UV 512 outside the latency-aligned
  avid). v0.1 drew paper and ink colours into blanking.
* Interlace field polarity flipped: the bottom (odd-row) field is
  `field_n = '0'`. v0.1 seeded the opposite field (ribbon's 2 px crawling
  comb on diagonals).
* Ink LUT built in video range: Y = 64 + 0.6875·Y′ (bright paper ~742, under
  the ~768 drawn-luma cap; full CMYK ~75 instead of ~10, below black), chroma
  512 ± 0.876·C′ (no clamps needed). v0.1's paper at Y ≈ 987 was clipped to
  plain white on HW, killing the stock tints — so K4 mostly only changed the
  grain.
* U/V swapped at the input latch and stored in hardware order in the LUT
  (gilt convention); the dry path is untouched.
* HD HDMI bar conditioning: [1 2 1] horizontal smoothing on Y, U and V
  (damier's HW-A/B fix) — halftone dots are the textbook trigger. The luma
  half of that fix is the LUT scaling above. Latency 13 → 15, all modes.
* Vsync serration guard: the per-frame start fires on the first vsync edge
  after active video only (analog sources ran the bubble timer 2–5× fast).
* `no_rw_check` on the five collision-free RAMs (`emulate_transparency` 4 →
  0). Found while auditing it: the slot-RAM write enable was never cleared,
  so the slot was rewritten all frame long — fixed (one write per re-roll).

**Controls**
* K1 Screen ran backwards against its 8 → 96 px label (knob up made cells
  smaller). Inverted; preset K1 values mirrored. The px label is exact at
  the ends; the ladder is exponential, so mid-knob readings are approximate.
* K6 Register is linear (was K6²: the bottom third, and the default 160,
  moved the plates by <0.2 px). Default now ~1.4 px-equivalent of phase.
* S8 Posterize was dead above 70% Press (Press forced full flats there).
  Now S8 = full flats at any Press; with S8 off, the Press ramp stops at
  half — so the switch always flips the look.
* K1/K2/K4 stepped-control hysteresis (above). K2 zones are now
  0–191 line / 192–383 diamond / 384–639 round / 640–831 ellipse / 832+ square.
* **S11 Spot** (new): a two-colour press — C and Y print as bare paper, M
  becomes a warm spot red (Q8 transmittance 245/52/40), K stays black; with
  S7 on B&W the single K screen prints red. Built entirely in the LUT
  (zero pixel-path cost) plus one compare: the red plate prints
  min(M, Y) so blue sky stays off it. Linework/borders follow the inks.
* Bubble x placement: centres now span 1/8–7/8 of the width (v0.1's ladder
  topped out at 9/16 — the right ~40% of the page never got a bubble).

### v0.2 hardware checks (for Ron)
1. HD 1080p source, K2 on Round, coarse K1: dots must be ROUND (not flat
   ovals) and the rosette visible.
2. SD interlaced source over blue sky: no flicker / twitter (the v0.1 re-test
   that never happened), no crawling comb on dot edges.
3. U/V: K4 fully clockwise (pulp stock) must look yellow-brown, not teal;
   on colour bars yellow prints as fine vertical lines. If the paper is
   teal, the input/LUT swap goes back out (free wiring).
4. Bottom of a 1080p picture with Panels on: margins equal top and bottom,
   no streaky grain in the last inch.
5. HD HDMI with motion: no thick bright bars, no thin coloured lines.
6. S11 Spot on/off at a few Press positions; K6 low end now visible.

## Status / next

* **v1.0.0 FINAL**, signed off on hardware 2026-10-11
  (`out/rev_b/ron/pulp.vmprog`). Presets 1-5 are hand-written (K1 values
  mirrored for v0.2); preset 6 "Misprint Comic" was captured from the device.
  Defaults are still the hand-set ones; more presets can be captured with
  `capture_preset.sh` and repacked.
* v0.2 candidates: press drift breathing, roller streaks, per-panel screen
  re-seed, channel sampling misregistration via the spare EBR, glyph
  variety on the sprite sheets, starburst on a control gesture.

## Community port — fit for the stock SDK (v1.0.2, 2026-10-14)
- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. v1.0.1 packed to 7957/7680 LC there (HD Dual 7846) and could not place.
- v1.0.2 replaces the seven per-pixel dot-distance squarers at E4
  (`bx3*bx3`, `by3*by3`, ~514 LUT) with seven 256 x 10 square ROMs (`C_SQ`,
  one EBR each). Each ROM is addressed by the value that used to load `bx3`/`by3`
  and its output register takes their place, so the squares arrive on the same
  clock as before and are re-registered into fabric (`qa4`/`qb4`) before the E5
  adds. Only bits 14..5 of the 15-bit square were ever used; the ROM stores
  exactly those.
- Proven bit-exact against v1.0.1: GHDL side-by-side, 40 frames (403 x 201
  raster), knobs and switches changing mid-picture, 0 mismatches in 3.24 M clocks.
- Packed LC (stock SDK): SD Standalone 7957 → 7514, HD Dual 7846 → 7395,
  HD HDMI 7778 → 7337. EBR 17 → 24 (HD Dual 25).
