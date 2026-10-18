# MONDRIAN (v1.0.0 — De Stijl composition generator)

**STATUS: FINAL except presets — signed off by Ron 2026-10-10** ("finalize
this program"). v1.0.0 is v0.2.3's logic with the defaults captured from the
hardware: K1 501, K2 603, K3 12, K4 Centered, K5 350, K6 0 (2 px lines), S8
Black Blocks and S9 Not Square on, P12 100%. Remaining: the five presets are
v0.1 hand-typed placeholders (K1 ran the other way then). Capture real ones
with `./capture_preset.sh` and rebuild.

**v1.0.2 (2026-10-17): resolution scaling.** Counters run in virtual
1080-line units from the timing ID (registers_in(8)); 1080p renders
pixel-identical to v1.0.1, 720p matches it, SD shows the same composition as
a 4:3 window with square cells. Seam half-width clamped to half a field step
vertically. Built router1 seed 4: HD Analog 78.84, SD Analog 77.01, HD HDMI
78.45, SD HDMI 75.59, HD Dual 74.37 (thin), SD Dual 80.80 MHz; 4933–4946 LC
(64%). Not yet flashed.

**Build: router1, base SEED=4. All six configs pass on seed 4.**
`SEED=4 NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron mondrian`.
Seed 1 also passes, but HD Dual lands at exactly 74.25 MHz (no margin). A
router1 HD Dual sweep of seeds 1–9 gave 74.25 / 79.49 / 76.46 / **79.90** /
77.24 / 76.29 / 76.07 / 74.86 / 76.88. With router2, HD Dual seed 1 misses
(71.6, on v0.2.0).

| Config | Fmax (MHz, seed 4, v1.0.0) | LC |
|---|---|---|
| HD Analog | 78.11 | 4844 |
| SD Analog | 80.30 (target 27) | 4848 |
| HD HDMI | 77.77 | 4857 |
| SD HDMI | 82.62 | 4849 |
| HD Dual | 77.89 | 4843 |
| SD Dual | 78.71 | 4852 |

63% LC, 2/32 EBR. (The seed sweep above was run on v0.2.1.)

## What it is

A white canvas carved into axis-aligned rectangles by thick black grid
lines, with scattered red, yellow and blue blocks (optionally black). It is a
repaint of quilt's recursive BSP engine: a continuous 32–256 px base grid,
3 subdivision levels and LATENCY 17. Cell counters advance as the raster
scans, so any cell size works with no divide. S9 merges grid-aligned cells
into 2×1, 1×2 and 2×2 rectangles.

Controls: K1 Grid Scale, K2 Composition, K3 Luma Level, K4 Split Jitter,
K5 Colour Amount, K6 Line Width, S7 Grid Lines, S8 Black Blocks, S9 Grid
(Square / Not Square), S10 Video Fill, S11 Luma Mod, P12 Density.

## v0.2.3 (2026-10-10): K3 Luma Level

Ron flashed v0.2.2: tying the line to K5 "didn't have the desired effect".
One knob was doing two jobs (colour amount and the luma line), so the line
couldn't stay selective while the colour amount went up. He asked for an
adjustment for the line itself and chose to give up K3 Sub-Pieces for it.

* **K3 Luma Level** sets the Luma Mod line: T = 64 + r − r/8 (64 at 0%, 512
  at 50% = the old fixed line and the default, 960 at 100%). A cell is
  coloured when its luma is above T. It is two free-running stages, latched
  at vsync. K3 does nothing while S11 is off.
* **K5 is Colour Amount only again** (v0.2.2's coupling removed).
* **The sub-piece seed is a constant** (`C_SUB_SEED` = x"A30F", what K3 gave
  at its old default 620), so the default painting is unchanged. K2 still
  re-rolls the sub-pieces, since every level's hash comes from the cell's
  root seed.
* The presets' K3 values are now luma lines (they'll be recaptured anyway).

## v0.2.2 (2026-10-10): K5 spreads Luma Mod colour (REJECTED on HW)

Ron wanted a little more colour fill in Luma Mod, especially at the top of
K5. At the top, K5 already colours ~100% of blocks in bright cells; the limit
was the fixed 512 threshold (most of a picture sits below mid-grey). In Luma
Mod, K5 now also sets the threshold: with k = K5/4,
T = 512 − k − 1.25·max(0, k − 128). That gives 512 at 0, 457 at the default
(220), 384 at half, 240 at 75% and 99 at full. A cell is coloured when its
luma is above T, so the top of K5 colours all but near-black areas. It is two
free-running stages off the SPI register, latched at vsync. Outside Luma Mod,
K5 is unchanged.

## v0.2.1 (2026-10-10): Luma Mod keeps the white canvas

Ron asked for the screen to stay filled with white squares and rectangles,
with colour only where there is luma. S11 no longer blacks out dark cells:
the full composition (white blocks plus black lines) always shows. A block
takes its red, yellow, blue or black only where its cell's input luma is
above 512; otherwise it stays white. K5 still sets how many blocks are
coloured inside the bright areas (100% colours every block there). Video
Fill follows the same rule, since it only fills coloured blocks.

* **Merged cells are coherent.** Each base column keeps a bit for the
  current cell row and one for the row above. A pixel takes the bit of its
  owning cell's origin through the same 2×2 winner select (S1b) that S9
  merging uses. The left column's bits come from the previous pixel at a
  column change, so there's no second 64:1 read. A merged rectangle is all
  colour or all white, never split along the removed grid line.
* The decision applies at the colour pick (S10b, `hide_pipe(13)`), not at
  the output, so grid lines are still drawn over dark areas.
* Luma is also sampled on the field's first active line, so a top cell row
  that starts in vblank no longer inherits the previous frame's bottom-row
  luma.
* +~120 LC (61% → 63%).

## v0.2.0 (2026-10-10): hardware-contract pass

* **`processing`, not `synthesis`.** S10 Video Fill and S11 Luma Mod read
  `data_in`, and a synthesis program gets no input (chroma code 0 is green).
  Still to check on hardware: how it behaves with no source connected.
* **Generated avid** (the cubist/aurora recipe). Its start is parity-locked
  to hsync and its end comes from a field-latched width measured from the
  source avid run. It drives the pixel and cell counters and
  `data_out.avid`. This targets the thin U/V-swap lines that flat primaries
  attract.
* **Blanking gate:** 64/512/512 wherever the aligned generated avid is low.
  Video Fill also needs the source avid to be live, so the 2–3 columns where
  the generated line overruns the source keep the block colour.
* **Video Fill delay in a 16-deep EBR ring** (read = write − 15, plus a
  fabric re-register, which equals `pipe(16)`), with `no_rw_check`
  (`emulate_transparency` = 0). The old FF tap was 17 deep, the depth that
  went green on HD HDMI in cascade and rollthebones. Moving it also freed
  ~500 delay FFs (65% → 61% LC).
* **No below-black levels:** grid lines, black blocks and Luma Mod's hidden
  cells are all Y=64 (they were 48, 48 and 0).
* **K4 labelled-default fix:** a register value below 4 is read as the label
  index, so the program opens on "Slight" as intended, not "Centered".
* **K6 Line Width is continuous:** half-width 1..16 px per leaf edge, so a
  shared seam is 2–32 px (it was 2–8 px, hairlines at 1080p). Default 448 =
  16 px. Making it numeric also removes its labelled-default trap.
* **K1 uses its full travel:** 32 + r/4 − r/32 gives 32..256 px. The old
  32 + r/4 clamped for the top 12% and disagreed with the display. It is
  computed free-running, so the vsync latch is only a −1 and two shifts.
* The Luma Mod per-cell sample now fires during active video only. The cell
  counter holds through hblank and could resample the blanking level.
* Categories: Grid, Pattern, Shape ("Generative" and "Art" aren't valid).

**HD Dual is no longer a hybrid.** v0.1.0 shipped 5 continuous bitstreams
plus an HD Dual slot built from the stepped source (309f843). That slot was
effectively a different program: K1 reversed, 4 levels, LATENCY 21, none of
the above fixes. Continuous HD Dual had hit a routing wall at 65% LC. After
v0.2 it routes in about two minutes (router2 included), and router1 closes
it at 76.95 MHz. The likely cause of the wall was the ~500 FF-only video
delay cells; that is inferred, not ablated. The old hybrid vmprog is kept at
`.mondrian_work/mondrian_v0.1.0_hybrid.vmprog`.

## Open items (from the 2026-10-10 review)

* Hardware check: yellow reads as yellow (the palette is stored
  U/V-swapped), no coloured lines over the primaries, Video Fill and Luma Mod
  work, and S9 merges read well.
* Merged cells: with K4 at Centered, a 2-wide cell's level-0 split lands on
  the grid line the merge removed.
* Y counters count ~41 vblank lines on 1080p, so the top cell row is a
  mode-dependent sliver. Count active lines only, and optionally centre the
  grid.
* K1 seeds by cell origin, so each size step re-rolls the composition.
  Seeding by cell index would turn K1 into a zoom.
* Luma Mod: a single-pixel sample against the threshold with no
  hysteresis, so cells near the threshold may flicker between colour and
  white.
* S8 Black Blocks is weak (~3% of leaves at the default K5). The
  Boogie Woogie idea is on the table.
* SD interlace: cells come out 2:1 tall (Y counts field lines).
