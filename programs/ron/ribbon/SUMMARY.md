# RIBBON

**Full-screen glossy candy ribbons** — a standalone fork of the *rainbow
taffy* texture from [SUGARCOAT](../sugarcoat/SUMMARY.md), blown up to fill the
frame and grown into a full instrument.  No video in: a pure generator
(`program_type = "synthesis"`).

Diagonal palette-ramp stripes — a continuous crossfade through 7 candy ramps
(K4), or the brown **Bakery** ramp (S9) — are rotated (K1), bent by a
cross-axis triangle ripple (K5) to a depth set on the slider, rounded and lit
by a specular gloss (K6), and can be mirrored into arrowheads (S7),
cross-woven into tartan (S8), folded into symmetric ropes (S10) or wrapped in
dark outlines (S11).

## Controls

| ctl | name | what it does | |
|---|---|---|---|
| **P12** | **Warp** | bend depth, 0 → 63 px | **continuous** |
| K1 | **Angle** | stripe rotation, 0 → 180° | **continuous** (128 steps) |
| K2 | **Density** | pitch octave | 4 steps |
| K3 | **Scale** | ribbon size, 16:1 zoom | **continuous** |
| K4 | **Palette** | crossfade through 7 ramps, wrapping | **continuous** |
| K5 | **Ripple** | warp wavelength, ~8000 → ~64 px | **continuous** |
| K6 | **Gloss** | rounding + specular line + seam emboss | |
| S7 | **Chevron** | mirror the field about screen centre | |
| S8 | **Plaid** | cross-weave a second ribbon set at 90° | |
| S9 | **Bakery** | force the brown Bakery ramp | |
| S10 | **Rope** | mirror the ramp → symmetric twisted ropes | |
| S11 | **Outline** | dark seam between stops | |

Presets: Rainbow, Bakeshop, Bunting, Tartan, Licorice Rope, Flat Bands,
Sugar Storm.  Defaults reproduce SUGARCOAT's taffy exactly: 45°, unity zoom,
32 px pitch.

## Everything continuous, everything on accumulators

SUGARCOAT drove Warp off a **7-entry threshold shift ladder** (0/1/3/7/15/31/63
px), Scale and Palette off bit-slices (4 sizes, 7 hard ramps).  All three are
now smooth, and none of it costs a pixel-path multiply:

- **The projection is an accumulator pair.**  `A` advances by `ca` per pixel
  and by `sa` per active line, so `A = x·cos θ + y·sin θ` with no multiply at
  all — a continuous **Angle** is free (videosky's trick).  `ca`/`sa` come from
  a 65-entry quarter cosine table scaled by 362 = 256·√2, so at 45° they are
  exactly 256 and the accumulator reproduces a plain `dx+dy` diagonal.
- **Scale is a per-frame zoom on that step.**  `ca = cos·zoom >> 8`, zoom
  32..512 — one *vblank* multiply, not a pixel one, and a 16:1 continuous
  range.  K3 = 512 gives exactly unity.
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

## Architecture

Streaming, **C_LATENCY = 15**, one line RAM, **two multiplies in the pixel
path** (the 8×11 warp and a 6×9 for the rounding) — everything else is adders,
shifts, compares and slices.

```
A       accumulator snapshot (A, B, ripple phases, x)
W1..W3  triangle bend -> one multiply -> added to both projections
Q       ONE barrel shift yields the stop index AND the sub-stop position
ST      rope fold + outline decision
R1/R2   palette register file x2 -> plaid select -> outline darken
S1/S2   ribbon rounding + specular line
L0..L4  line RAM -> gx/gy -> lit -> shift -> spec add + clamp
O       colour + shade + sheen, clamped, U/V swapped for hardware
```

## Timing (HX4K, no DSP)

All 6 configs pass, each at seed 1–2: HD **83.06 / 76.16 / 77.13**, SD
**72.41 / 72.03 / 74.18** MHz.  **5831/7680 LCs (76 %)**, 5 EBRs, no PLL on HD.
Not yet hardware-tested.

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
