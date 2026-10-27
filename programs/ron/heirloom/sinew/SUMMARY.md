# Sinew — ropes and sinews, Heirloom #18

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230315_00_53_56_Pro.jpg` (1920×1080 capture, 2023-03-15). Replaces v0.1.0 (the picture wrung
into S-curves by a vertical sine), which Ron judged too abstract and not like the photo. Presets are
hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_sinew/` (`notes.md` has every number).
- **Composition.** Three horizontal bands (row-difference scan):
  - top, rows 2–148: yellow / orange / green vertical bars;
  - middle, rows 150–900: seven orange ropes S-curving down the frame, crossed by thin blue/pink "sinew"
    lines that bulge into six ")" arcs at mid-height (y ≈ 540) and fan into green and black shingles above
    and below, with dense "<" chevrons at the bottom;
  - bottom, rows 902–1077: pink / navy / teal bars with U-shaped teal contours.
- **Mechanism (channel split).** Two horizontal oscillators, each restarting at the line start (hsync-reset),
  into a three-channel colourizer:
  - **Rope oscillator θ (R).** R is a clipped trapezoid of θ in every band. Per row, R's rising edges are
    exactly linear in x (rms 0.01–0.02 cycle):
    - top band: period 297.8 px, duty 0.558;
    - middle band: period 271.8 − 15.8u − 14.2u² px (u = (y − 540)/360, so 280 → 245 px from top to bottom),
      start phase 0.290 + 0.153u + 0.004u² − 0.179u³ (the ropes' S-curve, rms 0.018 cycle),
      duty 0.511 − 0.031u − 0.011u² (rise and fall halves run at different rates);
    - bottom band: period 263 → 251 px, duty 0.49.
  - **Sinew oscillator ψ (B).** B is thin pulses at integer ψ. Found by 2-D ridge detection and
    row-to-row order tracking:
    - each row is ψ = a(y)·x + b(y) + 0.166 cos 2π(θ + 0.22): the sinews are phase-modulated by the ropes;
    - the period follows a vertical triangle: P = 280.55 − 0.5297·s px, s = the soft distance to the nearest
      triangle peak (period 688 lines, centred y = 537, vertex rounded over 64 lines), rms 2.6 px.
      Lines are 280 px apart at the bulge and ~120 px at the vertices (y ≈ 193 and 881);
    - offset b = 408.3/P − 1.443 (the oscillator effectively restarts 408 px before active video), plus a
      ±0.38-cycle correction table along the triangle (a jump at the top seam, y ≈ 220).
  - **Colour.**
    - R is a function of (θ, Ω), R² 0.97–0.99, with Ω = ψ/2 mod 1: alternate sinew cycles differ (a
      divide-by-2). Halving ψ raises G's fit from 0.48 to 0.69 (upper zone) and from 0.63 to 0.76 (bulge).
    - G's mapping drifts along the triangle (upper / bulge / lower zones differ), so it is fitted as three
      slices. About 45 % of G's variance is grain: marbling inside the ropes, x period ~30 px, vertical
      coherence ~13 rows, bending along the sinew lines.
    - B = table(θ, Ω) plus a vertical ramp 25.2u² + 23.9u − 6.9 (u = distance along the triangle / 360 lines):
      the ropes turn pinker toward the bottom.
    - Bottom band: G is a function of (θ, y), the U-shaped contours.
- **Palette (k-means 12, share).** Green (21,152,30) 13.9 %, dark green (18,58,40) 13.5 %, orange
  (247,133,26) 11.8 %, red-orange (247,81,26) 9.8 %, yellow (248,203,8) 9.5 %, teal (12,157,118) 7.9 %,
  salmon (249,115,107) 6.7 %, olive (121,161,27) 6.4 %, navy (18,53,131) 6.0 %, pink (246,39,115) 5.7 %,
  brown (125,51,47) 5.2 %, grey 3.5 %.
- **Texture.** Torn strips at the triangle vertices (y ≈ 150–237 and 852–900: rows displaced up to ±50 px in
  runs of a few lines) and one 5 px slip at y = 744. Soft horizontal edges, no interlace comb.
- **Fit quality (float proto, then RTL).** Middle-band local contrast 0.93 of the photo (9×9 luma σ); 4×4
  region-mean colour error max 8, mean 4.4 (of 255). RTL at 720p: 0.90, max 9, mean 5.4.

## What it does
1. **Rope oscillator.** Per line: the two increments (rise half, fall half) and the start phase from the fitted
   cubics (top/bottom bands: their own constants). Ropes scales the increments; Twist scales the S-curve.
   Per pixel: a 24-bit two-rate DDA.
2. **Sinew oscillator.** Per line:
   - the triangle distance (mod 688 lines; Saw option), the soft triangle s;
   - P = P0·Sinews − Pinch·s and a = 1/P (serial divider);
   - b = 408.3a − 1.443 + the correction table + the stream phase.
   Per pixel: a 22-bit DDA plus the rope-locked wiggle (cosine ROM, 0.166 cycle).
3. **Melt.** Camera luma (or the Osc) × Melt is added to ψ (up to ±0.69 cycle) and, at 1/8, to θ.
4. **Colourizer.**
   - Middle band: R = table(θ6, Ω5); G = three slices along the triangle, blended per line; B = table + the
     blue ramp; plus the grain: a triangle of 9θ + a per-rope random walk + 0.375Ω, ±32 in the ropes,
     ±8 in the gaps.
   - Top band: R, G curves of θ.
   - Bottom band: R, B curves of θ; G = a (θ, row) table interpolated over 16 rows.
5. **Palette routing**, then RGB → 10-bit YUV in the hardware convention (U = Cr, V = Cb). Luma is
   pre-compensated ×16/13 for the output conditioning.
6. **Tear.** Inside 44 lines of each triangle vertex: random phase jumps (both oscillators, up to ±40/256
   cycle), held 1–4 lines, on half the lines. Plus slips: 1 line in 512 starts a ±4..19/256-cycle shift lasting
   7–38 lines.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Pinch** | The sinews' bulge and fan, i.e. the vertical modulator's depth: 0 = straight, evenly spaced sinew lines (no bulge); 50 % = the photo; 100 % = the vertices pinched to 24 px, dense knots and chevrons. Glided. |
| K1 Ropes | Rope count 0.5–2× (the photo: 7 across), all three bands. Glided. |
| K2 Sinews | Sinew line density 0.5–2× (base period 561 → 280 → 140 px). Glided. |
| K3 Twist | The ropes' S-curve 0–4× (centre = the photo; 0 = straight leaning ropes). Glided. |
| K4 Speed | Bipolar, still at the centre ±8. Positive: the pinch rolls down, the sinews stream, the ropes drift. Default 640 ≈ 28 lines/s (one triangle cycle in ~24 s), sinews 0.11 cycle/s. |
| K5 Melt | How far the camera (or Osc) bends the sinews and ropes. Default 420 (gentle bend); max ±0.7 sinew cycle. Glided. |
| K6 Palette | 1 the photo, 2 R/B/G, 3 G/R/B, 4 G/B/R, 5 B/R/G, 6 B/G/R, 7 the photo with G inverted, 8 the negative of 2. |
| S7 Tear | Torn strips at the vertices and slips (default On). |
| S8 Grain | The ropes' marbling (default On). |
| S9 Wave | Triangle (the photo) / Saw: the fan runs one way and resets at the vertex. |
| S10 Bands | The top and bottom bar bands (default On); Off: the sinew patch fills the frame. |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma, (Y − 512)/4 tapped straight from `data_in` (raw input stays shallow), × Melt is
  added into the sinew phase and, at 1/8, the rope phase. The sinews and ropes bend around the picture; at
  default knobs the photo's composition still reads (selfie test: the bears bend the lower right).
- **Osc:** two triangle waves (~5 cycles across, ~4 down, drifting per field) drive the same path, so Melt stays
  live: at max they bend the sinews into chevrons.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks; states 1–17 at zone clock 64), on the shared multiplier:
  Ropes scale × the timing step, Sinews base period, Twist, Pinch slope, Melt gain.
- **Glides:** a shared bank (±4 deadband, 1/8 ease per field) on K1, K2, K3, K5 and P12.
- **Per line** (kicked one clock after the source avid rise of line n, computing line n+1; line 0 at zone
  clock 600; ~650 clocks, fits SD's 858-clock line):
  - band select; xorshift tear and grain walks;
  - three Horner cubics (coefficients registered during each multiply); Twist; Ropes scale;
  - triangle distance, soft triangle, P, the 1/P division, b;
  - one 256×16 ROM read: correction offset + blue ramp; slice position and bottom row by shift-adds;
  - tear offsets and rope drift.
  The results are double-buffered (n_* → c_* at the next avid rise).
- **One serial multiplier** (24×18): signed operands loaded into one register pair, magnitudes taken in a
  pre-cycle, the signed product registered in a finish cycle. **One restoring divider** (17-bit quotient).
- **Pixel path, 24 stages:**
  - two DDAs; cosine ROM (+ fabric register); melt (4-bit gain × 8-bit source);
  - ψ, θ; table addresses; ROMs (+ fabric register);
  - slice blend (9×5) and bottom blend (9×4); blue ramp; grain; band mux; palette;
  - RGB → YUV shift-adds;
  - output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums, luma [1 2 1], luma to ~777;
  - blanking gate on the latency-aligned generated avid.
  `C_LAT` = `processing_delay_clks` = 24 in every mode.
- **Memories (22 EBR):** colour tables LR, LG0–2, LB (5 × 2048×6), bottom G 1024×12, cosine 256×16,
  band curves 2 × 256×8, correction/ramp 256×16. All generated by `sinew.py` into `sinew_pkg.vhd`.
- **Shell:** heirloom v0.2.0 (escalator): timing-ID virtual-1080 steps (x spans the width, y = 1080 virtual
  lines), vblank zone, width measurement, generated avid (`p_gavid` / `g_start`).
- **Fit history:**
  - The first RTL was 7807 LC (101 %) and failed placement. Ablations: the line program was 2200 LUT.
  - Cuts: the shared signed multiplier (−360 LUT); tear as direct phase offsets; the correction table and the
    blue ramp in one EBR word (no interpolation, 6 multiplies gone); shift-add slice position and bottom row;
    4 grain walkers; 4-bit melt gain. 6802 LC.
  - Timing (critical paths fixed in turn): engine kicks registered a clock ahead with an explicit idle state;
    one op per state in two shift-add chains; Horner coefficient registered; signed product registered.
    HD Dual router1 went from 74.5 to 88.2 MHz on seed 1 (82–95 MHz over seeds 1–4).
- **Verification** (GHDL 720p60 dec 1 vs the RTL-exact `imodel.py`):
  - Osc defaults: 99.7 % of pixels within ±12/255, mean 1.0 (after the tester's 4-px capture offset).
  - Motion: K4 max, frame 7 = model frame 7 (99.7 %); frame 4 differs (48 %), so it moves as designed.
  - Video = selfie: 94 % within ±12, 99.5 % within ±32 (the camera tap sits a few px off the beam).

## Status
- **v0.5.0 built 2026-10-27, NOT flashed.** **ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh sinew`), all six configs seed 1.
- Post-route Fmax: HD Analog 82.37, HD HDMI 83.67, HD Dual 88.21, SD Analog 87.18, SD HDMI 85.93,
  SD Dual 86.02 MHz.
- 6871–6900 / 7680 LC (≈ 89.7 %), 22/32 EBR. All bitstreams pass iceunpack. `sinew.vmprog` 487,484 B.
- router2 missed HD (65–74 MHz) on an earlier version of the design; it was not retried on the final one.

## Deviations from the photo
- **The bulge.** The photo's lime flash just right of each ")" arc is weaker; orange runs 20–30 px past the
  arc. The shingle stripes are a little softer than the photo's (64×32 table cells, per-cell median fit).
- **The grain** is synthetic (matched in scale, ~30 px across and ~13-row coherence, and in contrast), not the
  photo's exact marbling.
- **The torn strips** are the same field with random phase jumps: less multicoloured than the photo's top strip.
  The tear realisation is random.
- **The left edge** (x < 40): the photo's line-start smear at the bulge isn't reproduced.
- **SD 4:3:** the same picture (seven ropes across, same rows), compressed horizontally by 3/4; nothing in it is
  round, so nothing distorts visibly.
- **Interlaced modes** draw the same picture in both fields (no half-line offset).
- **Colour tables** are 6 bits per channel; the bottom band's G is 16 rows, interpolated.

## Working files
`.heirloom_work/rb_sinew/`:
- `notes.md`: every measurement, fit, proto round and build.
- Fits: `rcross.py`, `thfit2.py`, `ridge.py`, `track.py`, `psifit2.py`, `trifit2.py`, `ropepoly.py`, `tables.py`.
- `fmodel.py` (the field model), `build_data.py` (the table fit), `proto.py` (float proto),
  `imodel.py` (RTL-exact integer model), `gen_hook.py` (writes `sinew.py`).
- `sheet_final.py` / `sheet.png` (control sheet), `cmp_proto.png`, `cmp_final.png`.
- `abl/` (ablations), `synth_sa.sh` (standalone synth), `check_local.sh` (GHDL check including `sinew_pkg.vhd`).
