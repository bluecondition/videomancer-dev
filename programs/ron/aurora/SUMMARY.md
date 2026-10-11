# AURORA (v3.0.3 — big smooth travelling rainbow wave)

**STATUS: FINAL except presets — signed off by Ron 2026-10-09** ("we can
finalize aurora for now"). The v3 look, the generated-avid line fix and the
clean ROYGBIV were HW-confirmed on v3.0.2; the v3.0.3 Mirror reflection was
built the same day, before sign-off. Remaining: the four presets are
hand-typed placeholders — capture real ones from the device with
`./capture_preset.sh` and rebuild. Ship build = v3.0.3 (HD HDMI seed 3,
HD Dual seed 2; see Status).


**v3.0.3 (2026-10-08) — Mirror is a true reflection.** v3.0.2 was
HW-confirmed by Ron ("both problems are fixed… almost finalized"), who then
asked why the mirrored halves don't meet where their gradients line up. The
reflected copy had been coloured as a second wave (red on its own top edge,
colour offset negated), so where the bands overlap the nearer-copy switch on
the centre line put one band's violet edge against the other's red edge: a
hard seam. Now the reflected copy takes the reflected row's own offset —
its spectrum runs violet-on-top, like a rainbow in water — and both copies
are identical on the centre line, so the fold is seamless and the top half
is an exact mirror of the bottom. The axis also moved from row 540 to the
true raster centre (fh−1)/2 = 539.5 (row r ↔ row fh−1−r), with the wave
centred on it, which removes a one-row step. Along mode was already
seamless (its colour depends only on the column). The over/under braid look
is gone by design.


**v3.0.2 (2026-10-08).** Ron on v3.0.1, HD HDMI: still glitchy horizontal
lines, rainbow colours still not correct. So the v3.0.1 LUT-split was NOT the
cause (the packed LUT did map to an odd 3-block shape; the split stays as
hygiene, but it changed nothing on screen). Two program-side changes:

* **Generated avid (cubist's HW-confirmed recipe).** The encoders get no
  data-enable and pair Cb/Cr by counting from hsync; the core's packer
  restarts on Cb at every avid rise. Aurora passed the source avid straight
  through, so any line whose avid lands a clock off its usual hsync parity
  swaps U/V for that line — thin wrong-coloured lines across the band only
  (black is immune), and blue/indigo swapped reads as RED. Aurora now makes
  its own avid: start 3 or 4 clocks after the source's first avid rise,
  whichever keeps the hsync→start parity at the voted usual value; end
  after the field-latched width (or at hsync). The beam column counter runs
  on it, the output is neutral outside it, and the video backdrop is gated
  to black where the source is blank (the 3–4 columns at the right edge).
* **One clean rainbow.** v3.0.0 changed the Spectrum default to 256, which
  is 1.375 spectra across the band: red→violet then pink, red and orange
  again. And even exactly one cycle includes the violet→red wrap. Across
  mode now spans 0.875 of the hue cycle over the band's solid core at 100 %
  (hscale = cyc·112 / Hq2) — red at the top edge, violet at the bottom, no
  wrap — and the default is back to 146 (= 100 %). Initial colour phase
  0.4375 cycle centres it.

If thin lines persist on HD HDMI only after this, the next step is the
documented HD HDMI re-seed + A/B (see memory hdmi_tx_placement_skew), and
the Solid-mode stripe colours tell whether the palette itself reads right.


**v3.0.1 (2026-10-08) — colour LUT split (diagnosis turned out WRONG; see v3.0.2).** Ron flashed v3.0.0: the
wave looked right but the colours were wrong and thin red horizontal lines
ran through the band. The build log showed why: the colour LUT was one
packed 512×32 constant whose two top bits are always zero, so yosys pruned
it to 30 live bits and spread it over THREE EBR blocks — the odd-mapping
trap (sims bit-exact, hardware garbage). Bad entries at specific hue
addresses read as thin wrong-coloured lines across the band, because hue
runs vertically across it. This is the same LUT that v1.6 introduced when
its colours first "railed red/blue"; the v1.7 line-buffer rework was a
wrong diagnosis (harmless, kept). Fix: six independent 256×10 ROMs (Y/U/V ×
low/high half), each the same 256-deep ≤16-bit shape as the sine ROMs that
render correctly on hardware, with a 2:1 half mux after the read. 20/32 EBR.
HW: no change — lines and wrong colours persisted.


**v3.0.0 (2026-10-07) is a complete redesign of the look.** The "Coil"
compression idea (ribbon pinned at the screen edges, lengthening into tighter
and tighter meanders) never felt right, so it is gone. Aurora is now a big,
smooth, horizontal rainbow wave that travels across the screen, with
controls for speed, noise, amplitude, wavelength, thickness and colour.

What is new:

* **The wave.** y(x) = C + A·sin(k·x − φ) + Noise·A·(sin(1.61k·x + φ₂) +
  sin(0.38k·x + φ₃))/2. The two noise lobes sit at incommensurate
  wavenumbers and drift at their own rates, so the shape never repeats and
  never looks like a textbook sine unless Noise is at zero.
* **Constant VERTICAL thickness** (flag-style), not perpendicular. There is no
  slope correction any more, so the band stays smooth at any steepness, the
  horns and the 38 % width wobble of the old design are gone, and the
  amplitude can go all the way to half the screen.
* **Noise breathes the thickness** too (H_eff = H·(1 + noise/2)), so the band
  bulges where it bends — organic rather than mechanical.
* **P12 Swell** is the performance gesture: amplitude from a ~3 % ripple to
  crests touching the top and bottom edges. With Height at 100 % and Swell
  at full the whole screen becomes a waving rainbow wall.
* **S11 Mirror** draws a second copy of the wave reflected about the centre
  line, so two waves braid in an X pattern with the nearer one on top. It
  costs only a second vertical-distance test per column, not a second pass.
* **S10 Rainbow: Across / Along.** Across runs the spectrum across the band's
  thickness (stripes that ride the wave); Along lays the spectrum across the
  screen width so the colours are a field the wave passes through. Spectrum
  sets the cycle count in both.
* Kept from v2.0: Fill (Rainbow / Video), Backdrop (Black / Video), Color
  (Gradient / Solid), the phosphor LUT, the saw-active vsync guard, black at
  Y 64, the single-bank line buffer.

## Control reference card

| Ctrl | Name     | Function |
|------|----------|----------|
| K1   | Height   | Band thickness, square-law: hairline → the whole screen |
| K2   | Waves    | Wavelength, square-law: 0.5 → 5 waves across the screen |
| K3   | Speed    | Wave travel, bipolar; **centre (±5 %) = still**; square-law either side, positive = rightward |
| K4   | Noise    | Organic wobble of the path AND the thickness; 0 = a pure sine |
| K5   | Flow     | Rainbow drift; **centre = stopped**; square-law either side |
| K6   | Spectrum | Rainbow cycles across the band (Across) or the screen (Along): 50–400 % |
| S7   | Color    | **Gradient** = continuous phosphor spectrum / **Solid** = 7 pure phosphor stripes |
| S8   | Backdrop | **Black** / **Video** = the wave composites over the incoming picture |
| S9   | Fill     | **Rainbow** / **Video** = band luma from the live picture (half-mixed with the rainbow's), rainbow chroma |
| S10  | Rainbow  | **Across** the band / **Along** the screen |
| S11  | Mirror   | **Off** / **On**: the top half is a true reflection of the bottom (spectrum flipped, like a rainbow in water) — a seamless kaleidoscope fold on the centre line |
| P12  | **SWELL**| Amplitude: gentle ripple → crests at the screen edges |

Power-on look: medium band, ~1.2 waves, gentle rightward travel, a little
noise, slow colour flow, one spectrum across the band, black backdrop,
Swell at 35 %.

Presets (placeholders until captured from the hardware): **First Light**
(power-on look), **Tidal Wall** (thick, full Swell, stretched spectrum),
**Braid** (Mirror + Solid + Along), **Over the Air** (video backdrop, video
fill).

## Architecture (v3)

Once per line period the scanner sweeps **left to right, one column per
clock**, and stamps `{hue[10], alpha[6]}` into the single-bank 2048×16 line
buffer for the next display row. Per column:

1. Three quarter-wave sine reads (each at idx and idx+1 → interpolated; 6 EBR).
2. Noise lobes summed and scaled by the Noise level (Q6).
3. Wave + noise, times amplitude A (split hi/lo multiply). In parallel the
   half-thickness is scaled by the same noise term (H_eff).
4. y = centre + product. Signed vertical distance to this row, and to the row
   reflected about the centre (the mirrored wave), both with magnitudes.
5. Nearer copy wins (when Mirror is on); offset saturated to 14 bits.
6. alpha = feather(H_eff − |d|) (8-row feather, 64 levels); hue = colour
   phase − d·hscale (Across) or colour phase + column·kx (Along, from a
   per-column accumulator that runs with the writer).

Everything geometric is in quarter-rows of the MEASURED raster: centre =
fh/2, amplitude A = fh/32 + Swell·(fh/2 − fh/32), half-thickness
H = (fh/2)·Height², wavenumber k = kwav·(0.5 + 4.5·Waves²) with kwav =
65536/cols. The per-frame sequencer runs 8 products on one shared shift-add
multiplier (24-bit accumulator — fh·2 × 1023 overflows 22 bits) plus one
divide (hscale = cycles·128 / H31), all in blanking.

Pipeline depth 15 stages; column c of row R+1 is written ~16 clocks after
the beam read column c of row R, so one bank serves both. Scan = cols + 20
clocks per line (1940 of 2200 at HD). Latency 8 clocks, all modes. 20/32
EBR (8 line buffer, 6 sine, 6 colour). No DSP.

## Status

v3.0.3 (2026-10-08) — built and packaged, all 6 configs timing-closed
(post-route): HD Analog 75.1 (seed 1) / **HD HDMI 76.1 on SEED 3** (seed 1
missed, seed 2 stalled router2) / **HD Dual 75.1 on SEED 2** vs 74.25; SD
Analog 79.1 / SD HDMI 79.5 / SD Dual 80.5 vs 27. 6777–6802/7680 LC (88%),
20/32 EBR. HD margins are under 2 MHz; if a rebuild misses, try router1
before touching logic. Awaiting hardware.

v3.0.2 (2026-10-08) — HW-CONFIRMED (lines and colours fixed); built and packaged, all 6 configs timing-closed
(post-route): **HD Analog 78.0 on SEED 2** (seed 1 missed) / HD HDMI 76.4 /
HD Dual 76.0 vs 74.25 required; SD Analog 73.7 / SD HDMI 75.7 / SD Dual
81.1 vs 27. 6773–6801/7680 LC (88%), 20/32 EBR. Awaiting hardware.

v3.0.1 (2026-10-08) — built and packaged, all 6 configs timing-closed
(post-route): HD Analog 78.1 / HD HDMI 77.0 / **HD Dual 74.6 on SEED 2**
(seed 1 missed) vs 74.25 required; SD Analog 79.3 / SD HDMI 79.1 / SD Dual
82.6 vs 27. 6625–6640/7680 LC (86%), 20/32 EBR. HD Dual's margin is
sub-MHz and run-marginal under router2: if a rebuild misses it, solo-route
other seeds or use router1 before touching logic. Awaiting hardware.

v3.0.0 (2026-10-07) — built and packaged, all 6 configs timing-closed on
**seed 1** (post-route): HD Analog 81.6 / HD HDMI 75.3 / HD Dual 76.5 vs
74.25 required; SD Analog 80.3 / SD HDMI 80.2 / SD Dual 82.8 vs 27.
6621–6631/7680 LC (86%), 18/32 EBR. HD HDMI's 1 MHz margin is
run-marginal under router2 — if a rebuild misses there, re-seed or use
router1 before touching logic. **Not yet flashed.** Nothing from v1.7
onwards has been seen on hardware, so the first flash verifies the single-bank
line buffer, the v2.0 contract fixes and the v3 look together.

---

# Design history (v1.x – v2.0, superseded)

v2.0.0 (2026-10-07) was a finalization pass on the coil design: S9 Freeze →
Fill, saw-active vsync guard, black 64, dead-code purge. v1.7 reworked the
dual-bank line buffer into one bank after v1.6 railed colours on hardware.
The sections below describe the v1.x coil geometry and are kept for the
lessons in them.

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
- Latency 8 clocks, constant across modes. 18/32 EBR. No DSP.

## Status

v2.0.0 (2026-10-07) — built and packaged, all 6 configs timing-closed on
**seed 1** (post-route): HD Analog 77.4 / HD HDMI 80.7 / HD Dual 79.4 vs
74.25 required; SD Analog 79.3 / SD HDMI 82.1 / SD Dual 77.9 vs 27.
6646–6677/7680 LC (87%), 18/32 EBR. **Awaiting hardware** — the v1.7
single-bank line-buffer fix for the v1.6 colour-rail regression has not been
seen on the screen yet either, so the first flash of v2.0.0 verifies both.

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
