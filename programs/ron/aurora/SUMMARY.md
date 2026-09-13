# AURORA (v1.7 — anchored snaking rainbow ribbon)

**v1.6** replaces the fold with a **non-crossing meander**, and the color circle
with the real phosphor palette:

* **Colors** are now the seven scope-verified phosphor hues (deep red Y=328,
  orange, yellow, green, blue, indigo, violet), interpolated to 512 steps and
  baked into a constant LUT. The old vectorscope circle held luma constant,
  which is why red never read as a distinctive red. Solid mode addresses the
  LUT at the segment starts, so it shows the seven pure colors flat.
* **P12 is COIL: both ends stay pinned at the left and right screen edges,
  and the ribbon never overlaps itself.** Raising the slider lengthens the
  ribbon between those fixed ends; the extra length has nowhere to go but
  into curvature, so it snakes more deeply. Horizontal position advances
  monotonically from the left edge to the right edge, which makes the path a
  function graph — *mathematically incapable* of self-intersecting. That is a
  structural guarantee, not a tuning compromise, and it holds at every
  setting of every control.
* **The ends stay anchored.** An envelope that is exactly zero at both screen
  edges multiplies the meander, so the points where the ribbon meets the left
  and right edges do not move no matter what Coil, Height, Wiggle or the
  animation phase do. All the added length goes into the middle — the ribbon
  reads as if it were growing and struggling to fit between two fixed points.
* **Constant ribbon width through every bend.** Distance is measured
  PERPENDICULAR to the curve, not vertically, so the ribbon no longer pinches
  on steep sections and fattens at the crests.
* **The rainbow follows the ribbon.** Colour is keyed to the signed
  perpendicular offset too, so the bands stay square to the ribbon through
  every bend instead of shearing. (Free: the magnitude is already computed
  for the width correction, so this is a sign mux.)
* **More length at full Coil** — ~4¼ waves across the screen instead of
  ~2¾, so the ribbon bunches up considerably more.
* **Smooth edges at any Coil.** Four things were needed, in order of effect:
  1. **One stored column per PIXEL** (line buffer 1024 → 2048 deep). At
     half-pixel (HD) or quarter-pixel (SD) columns each column painted 2 or
     4 screen pixels, so as the curve steepened the edge became a staircase
     with multi-pixel treads. This was the dominant cause — invisible while
     shallow, worsening steadily with Coil.
  2. **The envelope is interpolated.** Its lookup index advances only ~0.53
     entries per column, so un-interpolated it stepped on ALTERNATE columns,
     and since it scales the amplitude that was a ~2.5-row chop at full
     Coil — the "every other vertical line" symptom.
  3. **The slope is averaged over two columns.** The width correction is a
     steep function of slope, so raw slope noise near a crest became a comb.
  4. The slope comes from the FULL-PRECISION product (12 bits finer than the
     rounded height) and the correction table is interpolated.
* At coil 0 the ribbon is one nearly-straight horizontal pass with only the
  wiggle on it. Amplitude is clamped against the measured frame height, so it
  always stays fully on screen.

## Control reference card

| Ctrl | Name     | Function |
|------|----------|----------|
| K1   | Height   | Ribbon thickness: whisper-thin stripe → ~40%-of-screen sash (soft-feathered edges at every size) |
| K2   | Position | Vertical home line of the ribbon |
| K3   | Wiggle   | Amplitude of the organic undulation. **Fully CCW = perfectly still** (also freezes the coil's slow crawl) |
| K4   | Rate     | Undulation speed, ~17 s glacial drift → ~1 s lively. No effect while Wiggle is zero |
| K5   | Flow     | Rainbow cycling across the ribbon. **Center detent (±5%) = true stop**; either side flows the colors, square-law speed |
| K6   | Spectrum | Rainbow cycles across the ribbon's thickness: 0.5 → 4. Default = one full spectrum. In Solid mode this sets stripe width |
| S7   | Color    | **Gradient** = continuous phosphor spectrum / **Solid** = the 7 pure phosphor colors as flat stripes (same geometry, same motion) |
| S8   | Backdrop | **Black** field / **Video** = ribbon composites over the incoming picture (soft edges blend into it) |
| S9   | Freeze   | Hard-stops all animation at its current phase (wiggle, coil crawl, color flow) |
| P12  | **COIL** | The performance centerpiece: bottom = one near-straight pass; rising lengthens the ribbon so it coils back on itself, 2–3 doublings at the top. The ribbon never crosses itself at any setting |

Power-on look: medium sash, centered, gentle wiggle, slow color flow,
gradient colors, black backdrop, coil at zero.

Presets: **First Light** (power-on look), **Silk Flag** (solid stripes,
mid coil), **Deep Water** (full coil, slow, compressed spectrum),
**Over the Air** (thin ribbon coiling over live video).

## Architecture

Once per line period a scanner sweeps **left to right, one column per clock**,
and stamps `{hue[9], alpha[5]}` into a dual-bank line buffer for the next
display row:

```
y(x) = P + env(x) · [ A·sin(θ1) + (A/8)·sin(θ2) ]
env(x) = sin(π·x/W)        θ1 = k·x + φ1(frame)
                           θ2 = 1.5·k·x + φ2(frame)
```

Three properties fall out of this form, each a structural guarantee rather
than a tuned result:

1. **It cannot overlap itself.** `x` advances monotonically and `y` is
   single-valued in `x`, so the path is a function graph.
2. **Both ends are anchored.** `env` is exactly 0 at `x = 0` and `x = W`, so
   `y = P` at both edges for *any* amplitude, wavenumber or phase. Coil
   changes only the middle.
3. **Every column is written exactly once**, so there are no gaps to fill, no
   overlap to arbitrate and no read-modify-write.

- **Coil** raises both the wavenumber `k` (¼ → ~4¼ waves across the screen)
  and the amplitude `A`, so arc length grows and the ribbon meanders more
  deeply. The lobes sit at a non-harmonic 1.5 ratio so the shape never looks
  like a plain sine; `φ1/φ2` drift per frame, which is what makes it swim.
- **Constant width**: the vertical distance `|y−R|` is scaled by
  `1/√(1+slope²)` to give distance perpendicular to the curve. The slope
  comes from the difference of consecutive *un-rounded* amplitude products
  (which is dy/dx, at 12 bits finer than the height), and the 16-entry table
  is interpolated — both are needed for a smooth edge.
  Without this the ribbon pinches wherever the curve is steep. The same
  signed quantity keys the colour, so the rainbow bands follow the bends.
  **The factor is clamped at 1.5×**: this estimate is really the distance to
  the *tangent line*, which underestimates badly where the curve bends
  tighter than the ribbon is thick, and unclamped it grows horns above each
  crest.
- **Coil-dependent thinning**: the ribbon narrows ~25% at full Coil. Past a
  point the curve's bend radius drops below the ribbon's half-thickness, and
  *no* constant-width non-overlapping ribbon can exist there — the offset
  curve genuinely self-intersects. Thinning keeps the extreme clean; turning
  Height down does the same thing by hand.
- **Amplitude clamp**: `A + A/8 ≤ 0.75 · (screen_half − thickness − margin)`,
  computed per frame from the measured raster, so it is always fully on
  screen at any Height.
- **Color**: a 512-entry constant LUT holds the seven scope-verified phosphor
  colors interpolated (packed Y/U/V). Gradient reads hue directly; Solid snaps
  to segment starts (k·73) where the LUT holds the pure colors. Values are in
  the raw hardware register convention, so the **simulator shows mirrored
  hues** while hardware shows ROYGBIV.
- **Line buffer**: ONE bank, 2048 × 16 b. The stamp for row R+1 writes
  column c about 18 clocks *after* the beam has read column c of row R, so a
  single buffer serves both — every read precedes its own overwrite by the
  depth of the marcher pipeline. Because every column is stamped (alpha 0
  outside the ribbon) there is nothing to clear, so the second write source
  is gone: one write statement, one read, and half the EBR (24 → 18).
  The word is padded to a power-of-2 width with **live** bits — hue10 +
  alpha6, where the extra hue bit stays live through solid mode's `hue·7`
  segment math and the extra alpha bit doubles feather resolution at the
  same feather width. A 14-bit word at 2048 deep had forced a 7-block
  2048×2 EBR mapping with two write sources contending for the one port.
- Per line the scan costs `cols + ~28` clocks — one column per pixel, so
  1920/1280/720 for HD/720p/SD against 2200/1650/858 available.
- Latency 8 clocks, constant across modes. 24/32 EBR. No DSP.

## Status

v1.7 — built and packaged, all 6 configs timing-closed (post-route):
HD Analog 77.1 (seed 2), HD HDMI 80.6, HD Dual 79.2 vs 74.25 required;
SD Analog 75.9 / SD HDMI 80.2 / SD Dual 79.9 vs 27.
**6648/7680 LC (87%)**, **18/32 EBR**. HW-iterating.

**v1.7 is a fix for a v1.6 hardware regression** (colors railed to red/blue
bands, ribbon glitchy) that no simulation reproduced. What the investigation
established:

* Ruled out: per-line clock budget (HD scan 1948 of 2200 clocks, 720p
  1308/1650, SD 748/858), mode-dependent latency (C_LAT constant), the
  blanking gate (the core blanks downstream of the program:
  `s_program_out → yuv444_30b_blanking`), vsync-serration guards, and every
  piece of class-dependent arithmetic in the v1.5→v1.6 diff.
* **Closed a real verification gap**: every prior sim had been SD/NTSC, and
  `--video-mode` selects the *core config*, so the HD path had never been
  simulated. A 720p60 decimation-1 render (config `hd_hdmi`) shows correct
  structure — so the fault is in the class of bug simulation cannot catch.
* **Fixed the one candidate left in that class**: the 2048×14 dual-bank line
  buffer. See the Architecture note above.

The palette was investigated and **exonerated** — a gamut check that used a
÷448 chroma divisor instead of the correct ÷896 wrongly condemned all 512
entries. With the correct 10-bit BT.601 inverse every entry is in gamut and
red decodes to exactly rgb(1.00, 0.00, 0.00).

Measured on the 720p render at Coil 70%: ribbon thickness still varies 38%
(81–122 px) because the width correction is clamped at 1.5×, and the top
edge carries ~48 columns of >3 px jitter. Both are known, quantified, and
open — see the constant-width note below.

Edge quality at full Coil, measured on the rendered frame (second difference
of the ribbon's top edge, per column — lower is smoother): half-pixel
columns 2.42 mean / 16 max with 4 spikes; full-pixel + smoothing
**1.07 mean / 6 max, zero spikes**.

GHDL-sim verified on the shipped RTL at Coil 0 / mid / full and in solid
stripe mode: ends anchored, no crossings, constant width through the bends.

Note: cicero also ships a program displayed as "Aurora" (borealis curtains);
this one is `com.ron.aurora` — distinct id, same menu name.

## Design history — four geometries

1. **fold** — prolate cycloid running backward past a threshold, producing
   true self-overlapping loops. Rejected: the ribbon must not cross itself.
2. **coil** — y descending monotonically, switch-back traverses stacked down
   the screen. Non-overlapping, but the ends ran free and it read as a zigzag.
3. **meander** — monotone x, length as curvature. Non-overlapping and
   endpoints reach the edges, but the ends still slid as Coil changed, and the
   ribbon pinched on steep sections.
4. **anchored meander (this one)** — envelope pins both ends; perpendicular
   distance keeps the width constant. This is what makes it read as one
   ribbon lengthening between two fixed points.

## Two bugs worth remembering

- **`to_unsigned(4096, 12)` silently wraps to 0.** Unity in the Q12
  correction table is 4096, which does not fit its own 12-bit field. Every
  place the curve went momentarily flat (slope index 0 — i.e. every crest
  and trough) the factor became zero, alpha saturated, and that column
  rendered as a solid vertical line top to bottom. Cap such tables at 4095.
- **Diagnose by bisecting the signal, not by staring at the code.** Forcing
  the correction to a constant made the lines vanish, which proved the
  pipeline was sound and the table value was at fault — after two wrong
  guesses (an overflow clamp, then a record-array read) had each cost a
  3-minute render.

## Timing lessons

- **Precompute row-scoped comparisons.** A termination test that was a 16-bit
  subtract+add+compare fanning into the pipeline valid chain moved the design
  from ~66–73 MHz to 75–80 MHz once folded into one precomputed threshold.
- **Never let an adder feed a multiply in the same stage.** Registering the
  summed operand ahead of the amplitude multiply took HD 68.6 → 76.5 MHz;
  the identical mistake later (a negate feeding the hue multiply) cost
  another 4 MHz, and a 27-bit subtract-then-negate feeding a clamp cost 12.
- **A variable shift by a signal infers a full barrel shifter**, even when
  only two shift amounts are ever used. Select between constant shifts.
- **Split array multiplies hi/lo** — each was ~6 ns of logic on its own.
- **Vblank sequencers are STA-timed at the pixel clock**: the restoring
  divider's divisor mux fed a 15-bit borrow chain and became the critical
  path despite running only in blanking. Register the divisor.
- **EBR data must hit a fabric FF before arithmetic** (52 → 68 MHz).
- **Constant functions belong in EBR**: the runtime palette interpolation
  (~480 LC of muxes/multiplies/adds) became a constant LUT.

Debugging note: an earlier geometry rendered with a regular dashed pattern the
model said was impossible. Measuring the rendered PNG showed every gap was
exactly 2 columns — a 3-column jump a single-column bridge could only
half-fill. Measure the artifact; don't re-read the model.
