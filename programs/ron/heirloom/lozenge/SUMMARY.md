# Lozenge — nested black diamonds with road-marking edges, cut by squeezing stripes

**v0.5.0 (2026-10-27): built, NOT flashed.** Heirloom #24, rebuilt from the photo
`photos/WIN_20230410_01_10_37_Pro.jpg`. The photo shows nested diamonds with thick black bands
carrying navy/grey rings and hot pink and olive bands between them. Horizontal stripes swap pink and
olive, and they squeeze from 71 lines to 27 lines going down the frame. Thin "road marking" lines
run along the black bands, and on alternate stripes they become white dashes. Presets are
hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_lozenge/`.
- **Field.** The field is a horizontal triangle plus a vertical triangle: t = aL·|x − 993| (left,
  aL = 1.0102) or |x − 993| (right), plus 1.800·(518.6 − y) above the centre or 1.790·(y − 518.6) below.
  - It was fitted to 1451 pink/olive edge points with rms 0.89 px (`fit1.py`).
  - The left 66 px are held at their x = 66 value (the line-start hold), so every contour bends flat
    into the left edge.
- **Colouriser.** It is a sawtooth of t with a period of 915.2 px, about 2.2 periods across the
  half-width. Each period, as a fraction u, reads:
  - ring (navy/grey): from the end of the previous black band to 0.165;
  - black: 0.165–0.357 (centre 0.2611, half-width 0.0962);
  - narrow band A: to 0.437;
  - wide band B: |u − 0.6218| < 0.1847;
  - narrow band A: to 0.887;
  - black: centred on 0.9834 with the same half-width, wrapping into the next period.

  A small eye (t < 72) holds the centre. The band edges in px are 72, 150, 328, 400, 738, 813, and
  again at +915.
- **Stripes.** A vertical square wave swaps the pairs: pink ↔ olive and navy ↔ grey. The band
  positions do not change; only the colours swap.
  - Its frequency glides on a sigmoid in log-frequency: f = (1/87.37)·2^(1.415·(σ((y − 496.35)/74.2) − ½))
    cycles per line, which is 1/142.6 at the top and 1/53.5 at the bottom.
  - The phase at line 0 is −0.4024.
  - All 27 boundaries fit with rms 0.73 lines.
- **Road markings.** These are one-shots, and only where the scan (left to right) ENTERS a black band:
  - yellow after olive or navy;
  - light pink after pink;
  - a double white line, 24.5 px apart, after the grey ring. It exists only in pink-type stripes,
    which is what makes the dashes.

  Where the scan leaves a black band there is only a soft edge. The line profiles were measured
  per type and deconvolved from the photo's horizontal blur.
- **Palette.** In RGB:
  - black (17, 18, 10)
  - grey (25, 26, 18)
  - navy (22, 26, 54)
  - pink (254, 10, 187)
  - olive (58, 61, 9)
  - line peaks about (225, 165, 25) yellow, (240, 130, 205) pink, (205, 168, 171) white
- **Texture.** Horizontal edges are soft: 10–90 % takes 9 px for Y and Cr and 16 px for Cb. Vertical
  edges are sharp. There is no noise or tearing.

## What it does
1. **Ring phase.** u = frac((t·gain + Etch·luma − flow) / period).
2. **Band.** Each pixel gets a band from three windowed compares. The eye is keyed on the un-etched
   field, so it stays a fixed pupil.
3. **Colour.** It comes from the band, the stripe parity and the palette.
4. **Marks.** A run counter follows the band changes along the line. At each black band's leading
   edge it reads a line profile ROM: 64 entries per type, in virtual px, or 4 px per entry for Glow.
5. **Output.** Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes. These give the photo's
   softness and are also the HD HDMI bar fix. Luma is then scaled 13/16 before the blanking gate.
   Palette luma is pre-scaled 16/13 and capped at 940.
6. **Motion.** At the default Speed the rings stream out of the centre at about one ring per 10 s,
   and the stripes roll down.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Stripes | the stripes' density, ×0.25 .. ×4 (512 = the photo) |
| K2 Chirp | how the stripes squeeze toward the bottom: bipolar (±8 deadband); left = squeeze upward, centre = even, 772 = the photo, right = stronger |
| K3 Aspect | the diamonds' shape: tall (0) .. the photo (512) .. flat (1023) |
| K4 Speed | bipolar (±8 deadband, square law): rings stream out of (+) or into (−) the centre and the stripes roll; full = 1 ring/s; 680 = the default drift |
| K5 Black | the black bands' width: thin black lines (0) .. the photo (512) .. black dominating (1023) |
| K6 Etch | how far the camera (or the Osc) bends the diamonds: 0 .. 800 px per full luma (square law), 320 = 78 px |
| S7 Stripes | On (the photo) / Off: plain nested pink/olive diamonds |
| S8 Marks | Road (the photo's edge lines and dashes) / Glow: a long luminous decay across each black band, like chrome tubes |
| S9 Twin | one lozenge / two side by side (the H triangle at double rate) |
| S10 Colours | Pink Olive (the photo) / Cyan Rust (cyan and rust bands, teal and plum ring, recoloured lines) |
| S11 Source | Video / Osc |
| P12 Rings | ring density, ×0.35 .. ×6 (exponential, glided): 0 % = one giant black lozenge with a navy eye; 379 = the photo; 100 % = a tunnel of about 13 fine nested diamonds, each lined |

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 from a one-line buffer, is added to
  the field.
  - The rings bend and fray around the picture, and the road markings follow every contour.
  - At the default Etch, the diamond composition still reads clearly.
  - With no input, Video mode is the photo.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting about 75 px/s, feeds the same
  path. At the default Etch the rings wobble gently, and Etch scales it.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier: 19 slots, with the slot number decoded two clocks ahead and the
    operands pre-muxed.
  - A shared glide for K1, K2, K3, K5, K6 and P12, run by its own phase and channel counters.
  - A 256-entry exp2 ROM for the exponential controls. Per field it computes:
    - the ring gain G = 9166·2^e per half-pixel, and the left gain;
    - the vertical slopes times Aspect, the chirp gain and the stripe frequency times the line step;
    - the flow and roll steps (Speed²), the black half-width and the etch gain;
    - the beam start value and steps, and the reflection reloads.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in about 75 clocks
  (it fits SD's 858-clock line):
  - DYG = (|vy − cy|·slope >> 12)·G >> 8;
  - the stripe parity, from a Q24 phase accumulator stepped by f = FS·2^(chirp·σ(vy/8)), using a
    256 × 12 sigmoid ROM and the exp2 ROM.
- **Beam.** A DDA carries t_x·G exactly in half-pixel units. It holds left of x = 66 and descends to
  the fold. It reflects at the fold (or at the Twin minima 480 / 1440 px and the peak at 960 px),
  where it reloads r·G or 960·G − r·G. Its event flags are registered a clock ahead.
- **Pixel path (21 stages).**
  1. Add the line term.
  2. Read the luma.
  3. Etch nibble products.
  4. Sum, then add <<6.
  5. Subtract the flow; eye test.
  6. Band distances.
  7. Windowed compares.
  8. Band code.
  9. Run counter.
  10. ROM address.
  11. Profile ROM (EBR), then a fabric register.
  12. Compose (black outside the generated avid).
  13. Two box filters.
  14. Luma 13/16.
  15. Gate.
- **ROMs.** The profile ROM is 1024 × 32 (palette × marks × type × offset, 8 EBR). There are two
  exp2 copies, a sigmoid table and a 2048 × 8 line buffer: 15 EBR in all.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 20, in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Verified (GHDL, 720p real timing ID).**
  - The still and Glow + Twin + Cyan Rust frames are bit-exact against the integer model `imodel.py`.
  - Default Osc frames 4 and 10 are 96 % exact; the remainder is the Osc lattice's 1-px write lag.
  - Integer model vs photo at defaults: 5.24 mean RGB; all 16 cells of the 4×4 region-mean grid within
    1–3 levels; 64-px block luma std 25.2 vs the photo's 24.4.

## Status
- **v0.5.0 (2026-10-27), built with ROUTER1.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 ./build_heirloom.sh lozenge`.
  - All six configs on seed 1. Fmax (post-route): HD Analog 80.80, HD HDMI 82.81, HD Dual 88.35,
    SD Analog 88.57, SD HDMI 90.88, SD Dual 91.11 MHz.
  - 6469–6489 / 7680 LC (84 %), 15 / 32 EBR. Every bitstream passed iceunpack.
  - Output: `out/rev_b/ron/heirloom/lozenge.vmprog`. **Not flashed.**
- **Timing history.**
  - The first router2 build passed HD Analog only on seed 2 (74.81 MHz) and missed HD HDMI on seeds
    1–5. Two vblank-sequencer control paths were the cause: the multiplier operand mux and the glide
    enables, both fed from the zone counter decode (about 13 ns of routing). Both were moved off it.
  - After that, HD HDMI passed router1 seeds 1–3 (85.5 / 80.0 / 82.5 MHz) and router2 seeds 1 and 3.
    If a later version gets tight, stay on router1.

## Deviations from the photo
- Stripe boundaries are hard row edges, within ±1 line of the photo's. The photo's boundary rows are
  half-mixed (a sub-line position); they account for 44 % of the remaining mean error (5.24 RGB at
  defaults).
- The slight rounding at the top and bottom vertices (an apex dip of about 4 px) is not drawn.
- The photo's blue chroma (Cb) trails its Y and Cr by about 5 px. The RTL blurs Y, U and V alike,
  with an edge 10–90 % of about 9 px.
- The brightest line cores come out at about 777 output luma, the shell's white cap. The 99.9 %
  luma is 170 against the photo's 183.
- The left edge fades in from black over 8 px (filter warm-up). The photo's dark capture border is
  about 3 px. A 2-row dash glitch at the photo's far left is not modelled, and the capture borders
  are not drawn.
- The eye is a fixed pupil. Under Speed the rings flow out from under it, and Etch bends the rings
  but not the eye.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3, so the outer side vertices and the left-edge hold fall outside the
    screen. SD is also softer, because the blur is per output pixel.
  - The line widths are in virtual px, so they look the same at every resolution.
