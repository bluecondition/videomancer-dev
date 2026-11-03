# Lozenge — nested black diamonds with road-marking edges, cut by squeezing stripes

**v0.6.0 FINAL (phase 2, approved by Ron 2026-11-03), merged.** The diag preview
(`Lozenge Diag`, `out/rev_b/ron/_diag/lozenge_diag_v0.6.0.vmprog`) went into production unchanged;
`out/rev_b/ron/heirloom/lozenge.vmprog` holds its six bitstreams. See [Phase 2](#phase-2-v060) and [Status](#status).
v0.5.0 (2026-10-27): built, NOT flashed. Heirloom #24, rebuilt from the photo
`photos/WIN_20230410_01_10_37_Pro.jpg`. The photo shows nested diamonds with thick black bands
carrying navy/grey rings and hot pink and olive bands between them. Horizontal stripes swap pink and
olive, and they squeeze from 71 lines to 27 lines going down the frame. Thin "road marking" lines
run along the black bands, and on alternate stripes they become white dashes. Presets are
hand-written placeholders.

## Phase 2 (v0.6.0)
No notes from Ron for lozenge (free hand, 2026-11-02). The v0.5 controls were all live and bold, so the round
re-assigns the one global-scale knob and widens three others:
- **K3 Aspect** was the V triangle's scale (tall .. flat diamonds), effectively a Y zoom of the diamond field.
  It is now **Bulge**: a waveshaper on both triangles before they are summed, so the diamond's sides bend and
  every ring gets its own spacing by its place. In modular terms, a quadratic shaper patched into the H and V
  ramps.
- **K1, K2 and K5** each reach about twice as far on each side of the photo setting.
- The default frame is bit-exact with v0.5: the model shows 0 differing pixels, and the RTL check is below.

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K1 Stripes: density ×0.25 .. ×4 (exponential, photo at 512) | **×1/8 .. ×8** (±3 octaves), photo still at 512 (bit-exact) | 0 = about one pink/olive swap across the frame; 1023 = hairline stripes (~7 lines at the bottom at HD; SD aliases at the very top of the range) |
| K2 Chirp: bipolar squeeze, ±2.83 octaves top-to-bottom (photo 1.41 at 772) | **±4.2 octaves** (3× the photo), photo still at 772 (bit-exact) | The slope is unchanged up to the photo's setting, then 1.96× steeper; 1023 = coarse stripes at the top (~1 per 380 lines) crushed to ~1 per 20 lines at the bottom. Left mirrors it (squeeze upward) |
| K3 Aspect: vertical slope ×0.35 .. ×2.8 (tall .. flat diamonds) | **K3 Bulge** (renamed), bipolar, photo at 512 (±8 deadband, bit-exact) | u += c·(tx² + ty²), c ∝ (k − 512)², square law. **Left = stars:** the sides bow in, the rings widen outward (a perspective tunnel) and fold back past the fold radius. ~380 is a soft star, ~200 a 4-point star with arcs, 0 a quatrefoil of four black discs. **Right = rounds:** the sides bow out to lens and oval rings that crowd toward the edges. c scales with the ring gain, so the shape is screen-geometric whatever P12 is. The diamonds keep the photo's 1.8 aspect (no tall/flat any more) |
| K5 Black: half-width ×0.15 .. ×2 (thin black lines .. black dominating) | **0 .. ×3**, photo still at 512 (bit-exact) | 0 = no black at all: the navy/grey ring widens into its own band, giving pink/olive/navy diamonds and no road marks (they only fire on black). 1023 = black swallows all but thin pink/olive lines |

Unchanged: K4 Speed, K6 Etch, S7 Stripes, S8 Marks, S9 Twin (Bulge follows the twin triangle), S10 Colours,
S11 Source, P12 Rings.
**Presets:**
- Photo, Twin Glow and Cyan Rush are as before.
- Tunnel now also stars the rings (K3 300).
- New: **Quatrefoil** (K3 0), **Rounds** (K3 1023, faster), **No Black** (K5 0, Glow marks, fewer stripes) and
  **Star Twins** (K3 180, Twin, Cyan Rust).

**Engine (new).**
- **The table.** c·r² sits in a 1024 × 16 block RAM (4 EBR, `no_rw_check`), indexed by r = |x − centre| or
  |y − centre| in field px. Each vblank a second-order DDA refills it from c, with no multiplier:
  Q(r + 1) = Q(r) + D, D += 2c, in Q16.16 mod 2^32, keeping the integer part = the phase mod one period. It
  runs at zc 160..1185, and the zone counter is now 11 bits.
- **c per field.** Two serial-multiplier slots, freed by Aspect: c = d²·G / 2^18 with d = |k3 − 512| − 8.
  That is 1/530 per px at the photo's ring gain and full knob.
- **Horizontal part.** Pixel stage 2 takes the distance to the fold (the twin uses the folds at 960 / 2880 h,
  and the hold region uses the hold point's distance). Stage 3 forms the address, stage 4 reads the table,
  stage 5 is a fabric copy. Stage 6 adds the line term, and stage 7 adds the result to the phase in place of
  the old "− flow".
- **Vertical part.** The line engine reads the table at |y| in hblank, 5 clocks after the generated avid falls
  (or in the zone for line 0, so the kick moved to zc 1300), and loads c·ty² − flow with the line.
- Pixel depth and C_LAT (20) are unchanged, so the U/V parity holds.
- The Aspect slots are gone and KAT / KAB are the photo's constants.
- K1's exponent is 1.5·(k − 512) (sh −3..2). K2 is two-slope. K5's half-width is k·6305/512 below the photo
  and 6305 + (k − 512)·25268/1024 above it. The line engine's multiplier is 40 bits, with stripe-step shifts
  −3..2.

**Build** (router1, `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh lozenge`, 2026-11-02): all six on seed 1.
HD Analog 82.91, SD Analog 81.77, HD HDMI 83.06, SD HDMI 80.80, HD Dual 81.82, SD Dual 84.18 MHz. 7,149–7,157 /
7,680 LC (93 %, +~605 over v0.5.1), 19 / 32 EBR (+4: the bulge table). iceunpack OK ×6, and the packed config
carries the new labels and presets. No pin-skew screen.

**Checks** (working files in `.heirloom_work/rb_lozenge/p2/`):
- GHDL analyse / elaborate / synth front end clean (`check.sh`). Standalone yosys: +434 LUT, +358 FF; the only
  transparency emulation is the line buffer's, as in v0.5 (the table carries `no_rw_check`).
- `imodel6.py` is the v0.5 integer model plus the v0.6 arithmetic. Its default frame equals v0.5's model on all
  pixels.
- `sweep6.py`: every new per-field and per-line value fits its RTL slice over the full K1 / K2 / K3 / K5 / P12
  travel, including the stripe-step product at SD with K1 = ×8.
- GHDL 720p60, decimation 1, Speed still (`sims.sh`):
  - **default frame byte-identical to v0.5's `out_exact.txt`**;
  - **Bulge 200 (star) frame bit-exact with `imodel6.py`** (100 % of pixels). This verifies the table fill, the
    horizontal and vertical lookups and their stage alignment.
- `sheet6.png` and `presets6.png` (model, 720p) show the new ranges and presets.
- Not checked by sim: the round side, Twin + Bulge, SD, motion over time (reasoned from the code and the model).

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
| K1 Stripes | the stripes' density, ×1/8 .. ×8 (512 = the photo) |
| K2 Chirp | how the stripes squeeze toward the bottom: bipolar (±8 deadband); left = squeeze upward, centre = even, 772 = the photo, 1023 = 3× the photo's squeeze |
| K3 Bulge | the diamonds' sides and ring spacing: 4-point stars with rings widening outward and folding back (0 = quatrefoil) .. the photo's straight diamonds (512) .. round lens / oval rings crowding outward (1023) |
| K4 Speed | bipolar (±8 deadband, square law): rings stream out of (+) or into (−) the centre and the stripes roll; full = 1 ring/s; 680 = the default drift |
| K5 Black | the black bands' width: no black, navy rings (0) .. the photo (512) .. 3×, black dominating (1023) |
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
- **v0.6.0 FINAL, approved by Ron 2026-11-03 after previewing the diag; merged.**
  - `out/rev_b/ron/heirloom/lozenge.vmprog` holds the approved diag's six bitstreams, repacked with the
    production TOML (id `com.czartroniks.lozenge`, name "Lozenge").
  - The merged sources match the diag's byte for byte, apart from the "(phase 2 final)" header comments in
    `lozenge.vhd`, `lozenge.py` and `lozenge.toml`.
  - Seeds (router1): all six on seed 1. See the Phase 2 section for Fmax.
  - To rebuild from source, run `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh lozenge`.
  - Both open questions are settled as built: Bulge replaces Aspect, and the star side stays uncapped (0 % is the
    quatrefoil).
  - Presets are still hand-written. Capture Ron's looks with `./capture_preset.sh`.
- v0.6.0 diag built 2026-11-02 (phase 2; see the Phase 2 section).
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Its off-avid input stays the palette black (the reset is 8 x that black), so active video is unchanged. Post-route Fmax: HD Analog s1 81.26, SD Analog s1 82.95, HD HDMI s1 85.34, SD HDMI s1 79.99, HD Dual s1 78.71, SD Dual s1 79.71 MHz; 6536-6551 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
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

## Open questions for Ron (2026-11-02 batch): settled 2026-11-03

Ron approved the diag as built, so both are settled as built:
1. Do you miss Aspect (tall/flat diamonds)? Bulge stays in its place.
2. Should Bulge's star side be capped so 0 % doesn't fold into a quatrefoil? No cap: 0 % stays the quatrefoil.

**Watch on hardware:** no pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config.
