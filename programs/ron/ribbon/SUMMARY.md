# RIBBON

**Full-screen glossy candy ribbons** — a standalone fork of the *rainbow
taffy* texture from [SUGARCOAT](../sugarcoat/SUMMARY.md), blown up to fill the
frame and grown into a full instrument.  No video in: a pure generator
(`program_type = "synthesis"`).

Diagonal palette-ramp stripes — a continuous crossfade through 7 candy ramps
(K4, Bakery brown among them) — are rotated (K1), sized (K3), set flowing
(K2), bent by a cross-axis ripple (K5, smooth or zigzag on S9) to a depth set
on the slider, rounded and lit by a specular gloss (K6), and can be mirrored
into arrowheads (S7), woven over/under into tartan (S8), folded into
symmetric ropes (S10) or wrapped in dark outlines (S11).

## Controls

| ctl | name | what it does | |
|---|---|---|---|
| **P12** | **Warp** | bend depth, squared, 0 → ~180 px per axis (fold-over climax) | **continuous** |
| K1 | **Angle** | stripe rotation, 0 → 180° | 128 steps, glided |
| K2 | **Flow** | bipolar: stripes march + ripples travel; centre = still | **continuous** |
| K3 | **Scale** | ribbon size, 7 octaves, exponential | **continuous**, glided |
| K4 | **Palette** | crossfade through 7 ramps, wrapping | **continuous** |
| K5 | **Ripple** | warp wavelength, ~8000 → ~64 px | **continuous**, glided |
| K6 | **Gloss** | rounding + specular line + seam emboss | |
| S7 | **Chevron** | mirror the field about screen centre | |
| S8 | **Plaid** | over/under basket weave with a second set at 90° | |
| S9 | **Wave** | ripple shape: Smooth (sine) / Zigzag (triangle) | |
| S10 | **Rope** | mirror the ramp → symmetric twisted ropes | |
| S11 | **Outline** | dark seam between stops (top ribbon only under Plaid) | |

Presets: Rainbow, Bakeshop, Bunting, Tartan, Licorice Rope, Flat Bands,
Sugar Storm (translated from v0.2 to keep each one's size and bend; still to
be re-captured from hardware).  Defaults reproduce SUGARCOAT's taffy pitch:
45°, 32-unit stops.

## v0.3.0 (2026-10-05) — finalization pass

Hardware contracts (none of these can be caught in sim):
- **Generated avid** — CUBIST's parity-locked start + fixed per-field length.
  The v0.2 "all kinds of horizontal noise" was put down to interlace field
  polarity, but the HD hardware is 1080p *progressive*, where that path never
  runs; on a wall-to-wall coloured generator the hsync-parity U/V-swap lines
  are the likelier cause.
- **Blanking gate** — y/u/v forced to 64/512/512 whenever the output avid is
  low (it used to hold the last palette colour through blanking).
- **Frame start = avid absent for 2 line widths**, not vsync: serrated analog
  vsync re-ran the interlace detect and flipped it off every field.
- **Legal luma** — output clamped 64..940 (v0.2 let gloss drive to 1023).
  Palettes: see v0.3.1.
- **Glide + deadband** on K1/K3/K5 (10.4 fixed point, diff>>3 per frame,
  frozen within 2.5 LSB) and deadband on P12: with the field anchored at the
  top-left, 1 LSB of pot noise used to move the far corner ~8 px.

Controls:
- **S9 Bakery removed** — it pinned both crossfade ends, so K4 went dead
  (and Bakery is ramp 3 of K4 anyway).  S9 is now **Wave** Smooth/Zigzag:
  the ripple's triangle fold runs through a 64-entry quarter-sine ROM, so the
  ribbons undulate instead of kinking.
- **K2 Density + K3 Scale merged** into one exponential Scale (octave = the
  barrel-shift pitch, mantissa = a 256-entry 2^-x zoom table), freeing K2.
- **K2 Flow** — two per-frame offsets on the accumulator seeds: stripes march
  in *stops per frame* (constant feel at any Scale) and the ripple phase
  travels.  Piecewise-exponential speed, ±16 deadband at centre.  Zero
  pixel-path cost.
- **P12 Warp squared**, ~4x the old depth at the top: the bend folds the
  ribbons over themselves.  The depth is held in PIXELS (scaled by the zoom
  each frame), so Scale never changes the bend.
- **Plaid is an over/under weave**: the top set alternates in a checkerboard
  of stop parities (it used to be "darker wins").

## Everything continuous, everything on accumulators

SUGARCOAT drove Warp off a **7-entry threshold shift ladder** (0/1/3/7/15/31/63
px), Scale and Palette off bit-slices (4 sizes, 7 hard ramps).  All three are
now smooth, and none of it costs a pixel-path multiply:

- **The projection is an accumulator pair.**  `A` advances by `ca` per pixel
  and by `sa` per active line, so `A = x·cos θ + y·sin θ` with no multiply at
  all — a continuous **Angle** is free (videosky's trick).  `ca`/`sa` come from
  a 65-entry quarter cosine table scaled by 362 = 256·√2, so at 45° they are
  exactly 256 and the accumulator reproduces a plain `dx+dy` diagonal.
- **Scale is a per-frame zoom on that step.**  `ca = cos·zoom >> 8` — one
  *vblank* multiply, not a pixel one.  Since v0.3 the zoom is only the
  one-octave mantissa (257..512, from a 2^-x table) and the octave is the
  barrel-shift pitch, so K3 is exponential over 7 octaves.
- **A is in 1/256 px.**  That sub-pixel resolution is what lets Warp and Scale
  move smoothly rather than snapping to whole pixels.
- **Warp is a multiply, not a shift.**  Both axes' displacements always enter
  the projection with equal weight, so they are **summed before** the multiply
  — one 8×11 multiply for the whole bend, and the *only* significant one in
  the pixel path.
- **Ripple is two more accumulators** (one per axis), so the warp wavelength is
  continuous too.  All four accumulators are seeded at 0 each frame rather than
  at screen centre — the field is periodic, so the origin is an invisible phase.
- **Palette is crossfaded in vblank into an 8-entry register file.**  The pixel
  path reads it as a plain 8:1 mux: no ROM, no lerp, no multiply per pixel.

Sim-verified continuity: K5 = 500/550/600/650, K3 = 300/310/320/330 and K4 =
100/110/120/130 each sat inside a *single* step of the old design and rendered
identically; now every one differs, monotonically, with the palette showing a
clean linear ramp (mean colour delta 5.1 → 9.0 → 13.3).

## Gloss that carries weight

On a flat generated field a luma-gradient emboss only ever fires at the seams,
so it stays a 1 px line.  The **sub-stop position** (the low 5 bits of the same
barrel shift that yields the stop index) is treated as the ribbon's surface
normal instead:

- a linear ramp across it → each ribbon reads as a **rounded rope**
  (capped at ±255 — a deeper diffuse just crushes the candy colour);
- a narrow band near the light side → the **specular line** running along it;
- the original line-RAM emboss on top → a lit/shadowed **crease at every fold**,
  blowing out to white above the K6 threshold.

K6 = 0 is fully matte: clean flat bands.

## v0.3.1 (2026-10-05) — after the first v0.3 hardware look

Ron: "looks AWESOME", but (1) quick glitchy horizontal lines at random times,
(2) "my true rainbow palette seems to be gone".

- **Palettes restored.**  v0.3.0 re-encoded them strictly legal (RGB × 0.817
  into 64..940 / chroma 896): hue-exact on paper, but 28 % less chroma and
  18 % less luma than SUGARCOAT's deliberately overdriven encoding (Y·1023,
  chroma ×4), and the vivid rainbow was gone.  Now the original values
  verbatim, only Y clamped to 64..940 (9 entries: one dark 53→64, the near-white
  stops →940).  Don't "fix" the encoding again.
- **Emboss line-RAM collision** (since v0.1): read and write hit the same
  address in one cycle — undefined on iCE40 EBR, invisible in GHDL.  The
  write now trails the read by one pixel (morsel's fix).
- **Interlace vote**: field_n must alternate 4 fields running before the
  interlaced line step engages, so a wobbling field flag on a progressive
  source can't flash a frame.

v0.3.1 build: HD Analog **82.10**, HD HDMI **81.00** (seed 1), HD Dual
**75.47 (seed 3** — seed 1 missed, seed 2 stalled router2; thin margin),
SD 79.76 / 78.58 / 80.89 (seed 1).  6751–6794 LC (88 %), 6 EBR.

## v0.3.2 (2026-10-05) — the flickering thin lines

Ron on v0.3.1 (HD HDMI): much better, but still occasional thin black lines
flashing, "particularly when I am changing settings like the slider" — and
NOT the core's HDMI-skew lines (those are colour-swapped; these are black).

- **Root cause: ripple-shape quantization.**  The bend used 64 levels per
  quarter-wave (`a(7:2)` of the phase).  Harmless at v0.2's shallow warp,
  but the v0.3 squared Warp is ~4x deeper, so one level = a 3–6 px jump of
  the whole row (and column).  Contours stair-stepped; wherever the level
  skipped or repeated (sine table near its peak, fractional phase steps) a
  row tore and the gloss emboss drew a crisp dark line along the tear.
  Moving the slider changes the jump size, so the lines flashed.  Confirmed
  in a 720p/dec-1 still at full Warp: 16–28 outlier rows (>2x the median
  row-to-row change) -> **0** after the fix.
- **Fix:** 256 levels per quarter.  Smooth and Zigzag share one 512x8 table
  (`C_WAVE`, index = zig & position) — one EBR read per axis, ~0 LC; the bend
  sum is 10 bits and the warp multiply 10x13 (>>3 instead of >>1, same depth).
- **Top-row emboss line:** the frame's first row compared against the line RAM,
  which still held the previous frame's bottom row.  Vertical gradient now
  zeroed on that row (`s_row0`).

v0.3.2 build: HD Analog **76.44**, HD HDMI **77.75** (seed 1), HD Dual
**76.23 (seed 2**, seed 1 missed), SD 78.48 / 78.66 / 76.88 (seed 1).
6764–6788 LC (88 %), 8 EBR (the two C_WAVE reads mapped to EBR as planned).
HD margins are ~3 % — the 10x13 warp multiply cost ~4 MHz; split its operand
hi/lo if a later change squeezes it.

## v0.3.3 (2026-10-05) — the recurring HDMI bars, found

Ron then saw the cross-program ~10 px bars (the CGA/Fray ones), none at Warp 0,
worse up to 100%.  Bisected on hardware:
- only the Penny Candy palette bars (same Warp => same multiplier activity on
  every palette, so not internal switching — it is OUTPUT CONTENT);
- 6 Penny variants (exact copy, black lifted, chroma halved, luma squeezed, MONO,
  yellow halved) all bar => luma pattern, not colour or slot;
- Rope ON (no 7->0 wrap) => no bars: that wrap is the only place full white (940)
  sits against true black (64); bar starts align with the stripe edges;
- a diag build with S11 = horizontal 1-2-1 luma low-pass => bars gone.

**Fix:** the 1-2-1 luma low-pass is now permanent (chroma untouched), so no
pixel-to-pixel step is ever full-swing; a hard edge becomes a 2-px ramp.
C_LATENCY 14 -> 16 for every mode (C_GATE_IDX = 11 keeps the inner blanking
gate on its pipeline stage), outputs re-gated after the filter.

v0.3.3 build: HD HDMI **77.33** (seed 1), HD Dual **75.55 (seed 2**, seed 1
stalled router2), HD Analog **80.10 (seed 6**, hand-installed: the builder's
seed 2 was only 74.59; seed 4 fails at 72.75), SD 72.42 / 74.90 / 79.76
(seed 1).  6849–6892 LC (89–90 %), 8 EBR.

## Architecture

Streaming, **C_LATENCY = 14**, one line RAM, **two multiplies in the pixel
path** (the 8×13 warp and a 6×9 for the rounding) — everything else is adders,
shifts, compares and slices.

```
A       accumulator snapshot + ripple shape (fold + sine ROM)
W1..W3  signed bend sum -> one multiply -> added to both projections
Q       ONE barrel shift yields the stop index AND the sub-stop position
ST      rope fold + weave order + outline decision
R1/R2   palette register file x2 -> weave select -> outline darken
S1/S2   ribbon rounding + specular line
L0..L4  line RAM -> gx/gy -> lit -> shift -> spec add + clamp
O       colour + shade + sheen, blanking-gated, clamped 64..940, U/V swapped
```

## Timing (HX4K, no DSP)

v0.3.0: all 6 pass — HD Analog **78.34** (seed 2), HD HDMI **78.58**, HD Dual
**82.47**, SD **71.10 / 72.17 / 79.18** MHz (seed 1).  **6711–6763 / 7680 LC
(87–88 %)**, 6 EBRs (the 2^-x zoom table took one).  Build with
`MAKE_TIMEOUT=900`: HD Analog seed 1 *stalled router2* (not a timing miss)
under a loaded machine; seed 2 passed.

The first v0.3 build was 95 % LC: four separate per-frame multipliers
(cos·zoom, sin·zoom, warp², warp·zoom).  They now share ONE free-running
12×11 multiplier (operands set by the sequencer, product read 2 states
later) — −600 LC, and HD Dual went 76.8 (seed 2) → 82.5 (seed 1).

v0.2 history: all 6 configs passed, each at seed 1–2: HD **83.06 / 76.16 / 77.13**, SD
**72.41 / 72.03 / 74.18** MHz.  **5831/7680 LCs (76 %)**, 5 EBRs, no PLL on HD.
v0.2 flashed once (horizontal noise + "palette only 1-2 ramps" reported);
v0.2.1 and v0.3.0 not yet flashed.

Two vblank fixes got it there, both instances of the same rule — **per-frame
logic is timed at the pixel clock, and it does not get to be sloppy**:

1. **The crossfade subtract fed the multiplier in one cycle.**  `b_ay → b_py`
   was the whole design's critical path at 13.62 ns.  The subtract now gets its
   own sequencer cycle.
2. **The write into the 240-flop palette register file was routing-bound**
   (`b_au → s_plu`: 3.1 ns logic, **10.6 ns routing**) — that file places
   scattered, and driving it through an add+clamp *and* a decoded enable meant
   six hops across the die.  Now the blend forms the value and a **pre-decoded
   one-hot enable** one cycle early, so the hop into the file is bare
   flop→flop.  HD Analog **71.5 → 83.1 MHz**.

Also: a router2 **stall** killed HD HDMI at the builder's default 360 s cap —
that is not a timing failure.  Rebuild with `MAKE_TIMEOUT=900`.

## Notes carried over from SUGARCOAT

- **U/V swap on the way out.**  Palettes are authored standard BT.601, but the
  hardware's `u` wire carries Cr and `v` carries Cb (verified on hardware
  2026-07-27).  The image-tester sim renders *standard* U/V, so sim hues come
  out swapped — judge structure in sim, not hue.
- `resize` on SIGNED keeps the sign bit, so it cannot bit-truncate; and
  `unsigned * natural` returns **double** width.  Both bit here.
- The sim raster is decimated (480×270), so pitch and warp amplitude read ~4×
  larger there than on a 1920-wide hardware raster.  Presets are sized for
  hardware.
