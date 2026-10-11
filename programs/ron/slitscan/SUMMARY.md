# Slit Scan — clean blocks of lines, offset in simulated time

## What it does
The picture is cut into clean bands of lines (Horizontal) or strips of columns
(Vertical). Each band shows the live picture slid sideways by its own amount, as if
every band were a different moment of a moving shot (the classic frame-buffer
slit-scan look). Videomancer has no frame buffer, so the time is simulated. A virtual
camera sways or drifts, and its position is remembered for 256 frames. Band k is
offset by how far that camera moved since its moment, plus a staircase pattern scaled
by P12. The bands ripple down the screen. Everything stays at full resolution with
sharp edges.

## History
- **v1** used a 2048-sample delay line, less than one HD line. It only slid bands
  sideways, with blanking wrapped into the picture and no blanking gate.
- **v2.0.0** (2026-10-11) was a true time-strip: a slit sampled into a coarse EBR
  canvas. Ron liked parts of it, but it isn't what he means by slit scan. It lives on
  as **programs/ron/timestrip** (com.ron.timestrip).
- **v3.0.0** (2026-10-11) is this program. Ron chose "pattern + time-lag" offsets and
  "strips slide sideways" for Vertical; a true vertical shift would need many stored
  lines.

## How it works
- **Line buffer:** sidewinder's design. It is dual-bank ping-pong by line parity,
  with Y 2048 deep and U/V 2:1 decimated (22 EBR). Three run-along read pointers
  (Y, U, V) step ±1 per pixel and wrap or bounce at the inset bounds [12, W−13]. The
  picture shows the line above, which is invisible.
- **Per-frame sequencer (vblank):**
  - Knob curves.
  - Camera position X: sway = M·sin(φ); drift = eases 1/16 per frame toward a random
    target, which is re-picked each time φ wraps. X is written to a 256-entry
    history.
  - Then for each band k:
    - pattern(k)·A, plus X(now) − X(now − k·span/N);
    - chroma split ±;
    - fold each channel's start column into [L, R] (sidewinder's reduce steps);
    - write {dir, pointer} into three 256-entry tables.
  - It takes 43 clocks per band (5.5k for 128 bands; NTSC has ~13k after vsync).
  - Multiplier loads are requested one step ahead through a registered strobe and
    select code.
- **Horizontal:** a band DDA per line picks the table entry in hblank.
- **Vertical:**
  - Strip 0 loads at the line start.
  - A pixel DDA, kept as the signed excess `(pN mod W) + N − W` so its sign bit is
    the boundary flag, reloads the pointers at each strip boundary.
  - The next strip's entry is prefetched during the current strip, via a fabric
    re-register.
  - Strip starts are computed per frame as `ceil(kW/N)`, so they match the pixel DDA.
- **Gaps:** black for the first ~W/512 px of each strip, or ~H/512 lines of each band
  (band 0 excepted).
- **Output:** latency 3 in every mode, with a blanking gate.

## Controls
- **K1 Bands:** 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128 (labelled; a raw value
  below 13 is treated as the label index).
- **K2 Pattern:**
  - Ramp (staircase);
  - Fold (edges far, centre home);
  - Random (golden-ratio scatter);
  - Wave (one cycle);
  - Zigzag (alternate ±).
- **K3 Motion:** size of the camera move, W/256 to W/4 (quadratic).
- **K4 Span:** how far back the last band reaches, 4 to 255 frames (quadratic).
- **K5 Speed:** sway / drift rate. The phase step is 32 + K5²/256 per frame, so the
  period runs from ~2048 frames (34 s) to ~16 frames (¼ s).
- **K6 Split:** chroma dispersion: U slides further than Y, V less.
- **S7 Axis:** Horizontal bands / Vertical strips.
- **S8 Edges:** Wrap / Mirror.
- **S9 Gaps:** thin black lines between the blocks.
- **S10 Motion:** Sway / Drift.
- **S11 Flip:** reverses the band order (pattern and lag).
- **P12 Offset:** staircase amount, W/64 to W (quadratic).

## Defaults
Captured from the hardware on 2026-10-11:
- Bands 48, Pattern Random, Vertical strips, Drift.
- Motion 793, Span 217, Speed 184 (a new drift target every ~400 frames ≈ 6.7 s),
  Split 352.
- Wrap, Gaps off, Flip off, Offset 840.

## Build (2026-10-11, builder default router2)

| Config    | Fmax (MHz) | LC          | Seed |
|-----------|-----------:|------------:|-----:|
| HD Analog | 89.58      | 4355 (57%)  | 1    |
| SD Analog | 73.69      | 4366        | 1    |
| HD HDMI   | 77.27      | 4382        | 1    |
| SD HDMI   | 83.06      | 4384        | 1    |
| HD Dual   | 78.36      | 4377        | 1    |
| SD Dual   | 79.18      | 4379        | 1    |

27/32 EBR: 22 line buffer, 3 tables, history, sine ROM. Three timing fixes, all
routing-bound paths at only 57% LC:
- the strip DDA's add→compare→enable became a sign bit;
- the band DDA was split into two steps;
- table RDATA is re-registered before pointer loads, and the sequencer's multiplier
  operand enables became registered strobes.

The HD Analog seed spread went from 67–80 to 76–90 MHz.

## Status
- v3.0.0 (2026-10-11): timing-closed 6/6; NOT flashed (an 11:54 build had thin HD
  margins and is superseded by the 12:08 one).
