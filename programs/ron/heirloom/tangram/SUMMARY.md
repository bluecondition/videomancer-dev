# Tangram — Heirloom #9, rebuilt from the photo

**v0.5.0 (2026-10-27): built, NOT flashed.** Rebuilt from `photos/WIN_20230222_00_06_47_Pro.jpg`. It now
redraws the photo itself: an additive RGB patch with pink log-chirped stripe fans (red), a stepped green
zig-zag plateau and lower sawtooth ramps (green), and a bright blue wedge and lower V on navy (blue).
Work files are in `.heirloom_work/rb_tangram/`:
- `notes.txt`, `proto.py` (float proto) and `imodel6.py` (integer model);
- `mc2.py` (engine ISA, assembler, ISS), `mk_hook2.py` and `cmp_final.png`.

## Reading of the photo (measured)
- **Mechanism:** an ADDITIVE RGB composite. Each colour channel carries one independent oscillator layer,
  summed by an encoder with slight crosstalk.
  - Fitted matrix: R = .924r − .043g − .049b + .084, G = −.027r + 1.031g − .030b + .025,
    B = −.051r − .071g + 1.004b + .018. Median error 5/5/5 per channel.
  - Typical colours: navy (13,9,122), bright blue (8,25,217), pink (250,7,113), cyan-green (3,218,198).
- **Red: pink stripe fans.**
  - Binary stripes with soft edges and a log-chirped phase (k = 3 ln(x − x0(y)) + m, rms 0.08–0.15
    stripe). They are thin and dense on one side, and each next stripe is ~1.4× wider.
  - Two families, "\" from the left and "/" from the top right, meet in "<" elbows at (1140,630),
    (1268,555), (1393,475), (1515,420) and (1650,355). Left stripes 2..5 bow ")".
  - Pink is off inside a top-centre triangle. The key lines are x = 274.5 + 1.078y (through the stripe
    tips) and x = 2108.6 − 0.739y (cutting the right family's lower arms).
  - Two torn 135-line bands of near-vertical stripes sit at the top left (x < 640).
  - Glitch rows 626..643 are shifted 0..34 px right of x 1060.
  - RC smear: rise 0.2, fall 0.125 per pixel. Duty 0.46.
  - The whole phase field is fitted as a quadratic B-spline on 64 px × 64 line knots: 8779 stripe samples,
    rms 0.013 stripe, p95 0.027. The torn bands have their own 5×12 tables.
- **Green: sawtooth ramps.**
  - Lower stripes rise to the right with phase −7.06435 + 12.56432Y − 3.04163Y² + 1.82364X (X, Y in
    1000 px; edges 1.2 px rms). The ramp runs 22 → 252 downward, and the edge spacing grows from 124 to
    200 lines.
  - The plateau has three bands: T1 lines 100..297, T2 298..548, T3 549..610.
    - Each is a per-line sawtooth between breakpoint curves: lit start, drops d1..d4, key edge, and the T3
      lead drop.
    - The drops zig-zag in ">" shapes with a ~240 px period.
    - T1 and T2 ramps are linear (25 → 240). The T3 ramp is convex (f^1.6).
- **Blue: navy base (B 125 at the top → 115 at the bottom).**
  - Wedge: |x − 996.2| < hw(y) = 398.34 − 0.42259y + 6.4545e-5y² (rms 0.5 px).
  - Lower V: y > 1019.44 − 0.36695h − 8.071e-5h², h = |x − 997| (rms 1.3).
  - Both have a 60-px dark fringe outside the edge, a sharp jump to 240 at the edge, and stripes parallel
    to it: wedge period 148 (74 sub), V period 166 (83 sub). Thin dark lines at v ≈ 96 and 432.
  - Small black dots sit on the axis at y = 25, 385 and 810.

## What it does
- **Pink:**
  - Every line, a coefficient engine blends three rows of the fitted spline table (weights × Fan) into
    32 coefficients. It samples the spline every 32 px into a ping-pong RAM of (P, Q) segments.
  - The pixel evaluates P + Q·g/32 and adds the flow and the source bend. It applies the two key lines,
    the torn-band tables and the glitch shift, takes the stripe duty, then applies the RC smear.
- **Green plateau:** per line, the engine evaluates 8 breakpoint curves for the current band (zig curve
  plus 7 slots) and the per-pixel ramp increments. The pixel finds its segment with comparators and runs
  a ramp DDA that restarts at each breakpoint and runs backwards on the mirrored half.
- **Green lower ramps:** a polynomial line offset plus a per-pixel DDA.
- **Blue:** closed form. Wedge distance and V distance address the measured profile ROMs (edge part and
  one stripe period plus flow). The dots are dimmed toward black.
- **Colour:** the three 8-bit layers go through three RGB→YUV ROMs with the fitted matrix folded in.
  Luma is pre-boosted 16/13.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Width | pink stripe width, duty 0.08 .. 0.9 (photo 0.46) |
| K2 Wedge | blue wedge opening ×0.42 .. ×2.4, bipolar, centre = photo (rays keep their 74 px pitch) |
| K3 Zigzag | plateau zig-zag depth ×0 .. ×2 (0 = straight drops) |
| K4 Speed | bipolar flow, still at centre (±8 deadband, square law): pink fans stream, green ramps scroll, blue rays flow; default slow forward |
| K5 Ramps | lower green sawtooth count ×0.5 .. ×2.9 (photo ×1) |
| K6 Bend | how far the source bends the pink stripes (and 0.6× the lower green ramps) |
| S7 Tear | torn top-left bands + glitch rows (On = photo) / clean field |
| S8 Key | centre-triangle key + right cut of the pink fans (On = photo) / pink everywhere |
| S9 Palette | Photo / Rotate (pink → green, green → blue, blue → red) |
| S10 Mirror | pink and green layers mirrored about the wedge axis (symmetric chevrons) |
| S11 Source | Video (default) / Osc |
| P12 Fan | pink stripe count ×0.46 .. ×4.2: 0 % = 3–4 broad chevron bands, 35 % = photo, 100 % = dense fine fans |

## Video
- Camera luma, (Y >> 2) − 128, is scaled by Bend.
  - It is added to the pink stripe phase: ±1/8 stripe at the default, up to ±0.8 at maximum.
  - About 0.6× of it goes into the lower green ramp phase.
- The stripes and ramps bend around the picture while the layout stays the photo's.
- The video tap is luma only, into geometry, never to the output.
- Osc mode substitutes two triangle oscillators that drift diagonally per pixel, per line and per field,
  averaged into a soft moving wave. Bend is live in both modes.

## Engine
- **Microcoded engine** (`mc2.py` → `tangram.py` UC/RFI):
  - **Datapath:** one 20-bit accumulator ALU. The register file (256×16) and the program (337 of 512
    words) live in block RAM.
  - **Pipeline:** F–D–O–E at one instruction per clock. A taken branch costs 4 clocks.
  - **Arithmetic:** a radix-4 Booth multiplier (10 clocks) and a 16/13 serial divider (18 clocks).
  - **Hazards:** the assembler inserts the NOPs; there is no forwarding.
- **Field program (vblank):**
  - glides for K1/K2/K3/K5/P12 (1/8 per field, ±4 deadband);
  - exponential knob curves for Fan, Wedge and Ramps;
  - key thresholds, duty, bend gain, speed and flow;
  - the 24 plateau curve pointers.
- **Line program (one line ahead, at the source's avid rise):**
  - spline weights × Fan, then the pink coefficient engine kick;
  - wedge, ramp and key polynomials and the dot phase;
  - breakpoint curves with zig offset, and ramp increments via the divider.
  - XFER then copies the 19-word block into the line registers in hblank.
  - Worst line: 771 clocks at HD, 743 of 858 at NTSC, 715 of 864 at PAL.
- **Pink coefficient engine:** a 14×4 multiplier over the weights' 4-bit digits (Horner, 12 clocks per
  coefficient), then a 4-stage chord pipeline. 96 taps per line in ~400 clocks.
- **Pixel path and shell:**
  - The pixel path is 25 stages, then the conditioning: luma [1 2 1], 8-bit chroma
    [1 2 2 2 2 2 2 2 1]/16 with its delay in EBR (re-synced every blanking), luma ×13/16.
  - The gate and avid sit at the 4-pixel filter centre (the escalator shell gated 4 px early; fixed here,
    output timing relative to hsync unchanged). Syncs go through an EBR delay of 28.
  - The generated avid is parity-locked (escalator shell), and the timing-ID virtual-1080 steps apply.
  - One latency of 28 in every mode.
- **ROMs:** pink tables 768×16, curves 3×256×16, YE 512×16, BW/BV 512×8, L3 256×8, colour 6×256.

## Status
- v0.5.0 built 2026-10-27 with **ROUTER1**; router2 misses at 61–73 MHz on HD Analog.
- Seeds: all configs seed 1 except **HD Dual (seed 5)**.
- Post-route Fmax: HD Analog 85.85, HD HDMI 77.81, HD Dual 78.82, SD Analog 78.40, SD HDMI 74.83,
  SD Dual 84.57 MHz.
- 7084–7113/7680 LC (92–93 %), 24/32 EBR. iceunpack OK.
- GHDL render at 720p60, decimation 1, matches the integer model exactly on Y, U and V (Osc, default
  knobs; Video and motion checked).
- NOT flashed.

## Deviations from the photo, and why
- The pink phase is evaluated as 32-px chords of the fitted spline (one multiply per pixel, to fit the
  chip). The error is ≤ 0.02 stripe at the photo's Fan and 0.08 at maximum Fan.
- The plateau edges are breakpoint curves; the T2 bottom curls use 4 breakpoints. The zig is one shared
  per-line offset. Ramps are per-pixel DDAs, so at SD/720p a ramp can start up to one virtual pixel late.
- The left-edge dashes (x < 70) are only approximated by flattening the V edge near the line start.
- The glitch rows are a fixed ramp shift, not the photo's exact displacement.
- Osc bend is a smooth synthetic wave, not the photo's (unknown) camera content. In Video mode, raw luma
  frays the stripe edges on textured sources.
- Colour comes from a fitted 3×4 matrix (median error 5 per channel). The dots are dimmed to 1/8, not
  pure black.
- At SD (4:3) the 16:9 composition spans the full width, so shapes are 25 % narrower. Interlaced modes
  draw each field over the full height (line-doubled).
- The picture starts 3–4 px right of the source's active window (the generated-avid recipe). Sims show
  a 4-px blank left column.
