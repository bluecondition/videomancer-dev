# Argus — Heirloom #21 (v0.6.1)

**v0.6.1 (2026-11-15), K6 camera pass:** K6 is now Camera (silhouette strength, 50 % = the shipped look);
the old K6 Grain is fixed at its default. See [K6 camera pass](#k6-camera-pass-2026-11-15).

**v0.6.0 (2026-10-29), phase 2 round 1: S7 Mirror is Fold / Off (Off replaces Repeat).** Ron flashed v0.5.0
and liked everything except the Repeat option. v0.5.0 (2026-10-27) was rebuilt from the photo
WIN_20230325_11_04_55. Processing,
latency 28. v0.1.0 was judged too abstract; this version is measured from the photo. Presets: Photo, Zip,
Broad Bands, Kaleido, Camera Echo (hand-written placeholders). Work files: `.heirloom_work/rb_argus/`
(`notes.md`, `imodel10.py`, `cmp_final.png`).

## Reading of the photo (measured)

- **Layout:** a horizontal central band (rows ~290..750) of vertical "ribs" — magenta / lavender / white /
  white-green sawtooth columns ~158 px apart — between a dark olive block on the left (x 60..300) and an olive
  ">" chevron band on the right (apex x ~1710). Cyan / aquamarine zones with stacked ">" stripes above and below,
  blue patches near the top and bottom centre, violet / pink / white slanted stripes in the four corners.
- **Mirror:** top/bottom only, about row 522 (r <-> 1044 - r). Mean |diff| 13.4 vs a 76 baseline; the best
  left/right axis scores 47. High-pass texture correlation across the fold 0.73.
- **Channels:** B is a sawtooth field (sharp drop, rise over a stripe) forced to ~0 inside dark silhouettes (the
  key, 31.6 % of the frame); G is a smooth large-scale field; R is a smooth field plus the saw.
- **Stripes:** one horizontal ramp, reset inside the key and frequency-modulated after it.
  - Rib-band drops at x = 338, 394, 493, 628, 786, 943, 1106, 1263, 1426, 1584 (spacings 56, 99, 135, 158, 157,
    163, 157, 163, 158).
  - Echo test: the drops are locked to each silhouette's trailing edge (+43, +136, +268, +425 px) as it moves.
  - Fit: period P = 159.9 px; RC charge inside the key tau 24.7 px, decay after it tau 78.7 px; FM depth m 4.02
    (4.6 with the darkness model); line-start phase 0.0215 y + 0.694 cycles (the corner stripes slant -3.7 px per
    line).
  - The transient strength differs per silhouette (left block ~1.9 cycles, upper blob ~1.2, lower arm ~0.6) and
    follows darkness, not width. Charge target clip(0.55 + 4 D): mean capped phase error 0.12 cycle, 53 % of the
    photo's drops within 0.1 cycle, rib band +-0.05.
- **Colour laws** (non-key pixels):
  - G = clip(0.64 |250 (1 - |x - 954| / 380) + 386 Ty| + saw terms), error 9.7 levels (Ty = vertical triangle,
    0 at the top and the fold, 1 at row 261).
  - R = vertical ramp times |x - 954| plus the saw, error 15.9.
  - B = mostly the saw, error 24.5.
- **Key:**
  - A 64 x 61 bilinear node grid (32 px x 522/63 lines) of a fitted darkness reproduces it at 99.0 % pixel
    accuracy; trailing edges median 1 px.
  - Key colours form a triangle black (15,15) / brown (110,15) / green (45,135) in R-G, with B 0..10.
  - The interior follows the outside field: R_in ~ 35 + 0.3 R_out, G_in ~ 64 + 0.25 G_out (brown next to magenta,
    green next to cyan).
  - Thin dark contour lines; short horizontal B dropouts (specks, 8-40 px, 1-5 rows) over the pastels.

## What it does

A darkness store (64 rows x 61 columns: Osc = fitted to the photo; Video = the camera's top half, written live)
is folded about the fold line and bilinearly read into a darkness D' (+ Shadow). D' > 0 is the key. A
horizontal phase ramp runs along each line: it is reset inside the key, and an RC charge E of the key's darkness
(charged inside, decaying after) speeds it up, so every silhouette's trailing edge is echoed by bunching ribs.

The saw s = phase drives an RGB colourizer:
- R = per-line saw table + |x| V term (vertical ramp);
- G = |G-fold constant - H-V slope| + saw table;
- B = saw table.

Inside the key, B is cut and the colour blends from the law colour to an olive palette (brown..green by the local
G - R plus mottling), with dark contour rings of the darkness field, dark dashes and a per-line streak. Specks
drop B over the pastels. RGB -> YUV -> output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma
13/16) -> blanking gate.

## Controls

| Control | Range / default | Photo element it drives |
|---|---|---|
| K1 Echo | 0..100 %, 50 % | FM depth of the charge: how hard the ribs bunch behind each silhouette (the 56/99/135-px first ribs); 0 = even ribs, 100 % = 3.4x |
| K2 Shadow | 0..100 %, 50 % | silhouette level (+-0.25 D): the olive keys shrink to cores or flood |
| K3 Tint | 0..100 %, 50 % | offset inside the G fold: the cyan/white centre field turns magenta/violet or cyan/white |
| K4 Speed | -100..100, 640 | the ribs stream out of the silhouettes (bipolar, still at the centre +-8, square law) |
| K5 Spread | 0..100 %, 50 % | width of the cyan centre field (G's horizontal V slope, 0.36..2.8x) |
| K6 Camera | 0..100 %, 50 % | (K6 camera pass, v0.6.1) silhouette strength of the darkness store, Video and Osc alike: 0 % = no silhouettes (the bare, evenly spaced rib field), 50 % = the shipped key (bit-exact), 100 % = deeper, slightly wider keys (2.47x) |
| S7 Mirror | Fold / Off | Fold = the photo's top/bottom mirror about row 522. Off = the silhouettes run unmirrored down the whole frame (Video: the full camera picture; Osc: the photo's top half stretched to full height); the colour field stays folded behind them |
| S8 Specks | On / Off | the B dropouts over the pastels |
| S9 Swap | Off / On | exchanges the R and B laws (pink <-> blue ribs) |
| S10 Kaleido | Off / On | left/right mirror of the silhouettes and the colour field |
| S11 Source | Video / Osc | camera darkness / the photo's fitted darkness |
| P12 Ribs | 0..100 %, 50 % | stripe period 640 .. 160 (the photo) .. 40 virtual px, exponential |

## Video

The camera enters as darkness. Luma is scaled to 8 bits, smoothed along the line (IIR, ~16 px), and sampled at
the 61 node columns on the first line of each of the 64 node rows in the top half of the frame (row 63 on the
first line past the fold). It is written straight into the store banks during active video as
clip(128 + 1.25 (89 - Y8)), so anything darker than ~35 % luma becomes a silhouette.

With Mirror Off, the store's 64 rows span the whole frame (row = vy / 2, ~17 lines per row): the writer
captures rows over the full camera picture and the read runs straight down without folding. The colour field
(G fold, R ramp, V triangle, line-start slant) still folds, so the camera's real silhouettes are keyed over
the photo's symmetric background.

From there the processing is the same as the photo's: fold, key, charge, echoing ribs, colourizer, olive fill.
Osc substitutes the photo-fitted darkness store (a ROM). Video is the default.

## Engine

- **Shell:** virtual 1920 x 1080 coordinates from the measured width and active lines (serial divider);
  generated avid; vblank zone (no avid for 1024 clocks); glides on K1, K2, K5, P12; per-field products on one
  serial 18 x 13 shift-add multiplier (stripe rate, FM coefficient, flow, slopes, ring width).
- **Line engine:** kicked at each line start (line 0 in the vblank), it computes the next line on the shared
  multiplier, 12 products: store row and fraction, vertical ramp, vertical triangle, line-start phase, the R law
  coefficients A/B/C (x ramp, x triangle), the G fold constant, and the R V-term slope.
- **Row writer:** in the horizontal blank, it y-lerps two store rows into a row buffer that holds each column's
  darkness and the step to the next column (2 EBR, banked by line parity).
- **R saw table:** 64 entries per line by running sums (1 EBR). The G and B saw shaping are one constant 64-entry
  ROM.
- **Pixel path, 28 stages:**
  - bilinear D' and the key;
  - charge target, then the charge IIR as a carry-save 4-operand sum (E += tk - (E >> a) - s (E >> b), shift
    pairs per key state and resolution set, so the time constants hold in virtual px);
  - FM increment and the phase accumulator, reset in the key;
  - V-term DDAs and the table reads;
  - laws (8-bit), swap, fill (palette, depth blend, rings, dashes, streak), specks;
  - RGB -> YUV with rounded shift-add coefficients;
  - output conditioning, with the chroma's 8-back values in an EBR ring and the box sums restarted in every
    blank.
- **Specks:** decided in the horizontal blank from a xorshift LFSR.
- `argus.py` emits `argus_pkg.vhd`: four darkness bank ROMs (8 EBR), the exp ROM for P12 / K5, and the G/B saw
  ROM.

## Status

- **v0.6.0, built 2026-10-29, all six configs on seed 1 (router2). Not flashed.**
  - **Fmax:** HD Analog 82.49, HD HDMI 80.89, HD Dual 77.66 MHz; SD Analog 80.53, SD HDMI 79.53, SD Dual
    82.09 MHz.
  - **Size:** 7,198-7,237 LC (HD HDMI 7,206, 93.8 %; the highest is SD Analog at 94.2 %), 22/32 EBR. All six
    bitstreams pass iceunpack.
- v0.5.0, built 2026-10-27, all six configs on seed 1 (router2). Flashed by Ron: everything OK except S7's
  Repeat option.
- **Fmax:** HD Analog 83.44, HD HDMI 79.38, HD Dual 77.41 MHz; SD Analog 74.65, SD HDMI 83.92, SD Dual 86.59 MHz
  (SD minimum 27).
- **Size:** 7,186-7,212 LC (HD HDMI 7,189, 93.6 %), 22/32 EBR. All six bitstreams pass iceunpack.
- **Verification:** GHDL renders match the integer model (Osc mean |dRGB| 4.6 at matched field count; 4x4 region
  means within 1 level). Motion is verified (ribs stream away from the silhouettes).
- **Not flashed.**

## Phase 2

- **Round 1 (v0.6.0), Ron 2026-10-29 on v0.5.0:** the only note was that he dislikes S7's Repeat option. Repeat
  tiled the top half, which left a hard seam across mid-frame where the fold row jumped back to row 0, colour
  field included. It is replaced by **Off**: no mirror for the silhouettes. Only the store-row operand of the
  line engine's product 0 changes (vy / 2 instead of the folded vy). The writer's top-half gate extends to line
  1044, and the colour laws keep the folded vy. Fold is unchanged, so the photo and the defaults are untouched.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Argus had no camera-strength control, so
K6 is new (**v0.6.1**).

- **What K6 does.** The camera reaches Argus as a frame store: the darkness store's silhouettes (Video:
  the camera's IIR-smoothed luma at 64 x 61 nodes; Osc: the photo-fitted ROM). So K6 sets how strongly
  the store shows, the same way in both sources. The row writer (horizontal blank, off the pixel path)
  now scales each column's darkness before it goes into the row buffer:
  `D' = g (D + 16) - 16` (darkness units; Q.2 in the RTL), clipped to +-200, so the charge target stays in
  range. The pivot of 16 makes the silhouettes shrink onto their darkest cores as g falls instead of
  popping off at 0.
- **Gain law.** K6 is glided in the existing glide slot machine (a fifth slot: 1/8 of the gap per field,
  deadband 4). g = k / 512 below 50 % (Q.6), then 1 + 3 (k - 512) / 1024 above.
  - 0 % = no silhouettes: the bare rib field, slanted and evenly spaced (no keys, so no echo).
  - 25 % = g 0.5: the silhouettes shrink to their dark cores and the echo weakens.
  - 50 % = g 1: the shipped look, bit-exact (Video and Osc; the Photo preset is unchanged).
  - 100 % = g 2.47: deeper olive keys with denser contour rings, slightly wider (they grow by about 10
    darkness units), and stronger echo bunching. The ribs and colour field still read; no washout.
  - Because 50 % is the shipped look, this follows the rule for "the camera IS the picture" programs
    (the silhouettes are a photocopy of the camera); doubling at 50 % would also have moved the Osc
    photo anchor.
- **K6's old function.** Grain (key texture) was the weakest control on the sheet: 512 and 1023 look
  the same, and only below 12 % (flat olive) does it change anything. So it was dropped, fixed at its
  default 512 (contour-ring line offset 19, mottle level 2, never flat). Nothing else moved. Its
  per-field product on the serial multiplier is gone, and the ring thresholds are constants.
- **Sheet:** `.heirloom_work/rb_argus/k6/k6sheet.png` (selfie, 720p60 decimation 2, glides snapped in a
  sim-only copy).
- **Build (v0.6.1, 2026-11-15):** router1, **seed 1 on all six configs**. HD Analog 88.02, SD Analog
  82.39, HD HDMI 89.29, SD HDMI 95.15, HD Dual 83.68, SD Dual 88.05 MHz (worst HD 83.68). 7392-7416 /
  7680 LC (96.6 %, +~180 over v0.6.0), 22 / 32 EBR. **Not flashed.**

## Deviations from the photo, and why

- **Key texture is procedural:** dark contour rings of the darkness field, mottled brown/green, a per-line streak
  and dashes. The photo's contour lines are camera content.
- **Stripe phases** in the cyan zones and right of the ">" band are off by up to ~0.3 cycle in places: the
  per-silhouette echo strength is only partly captured by the darkness store.
- **B's deep minima** just after the keys (the lime fringes) are weaker.
- **G and B saw shaping is constant down the frame** (one table); only R keeps per-line shaping. This was done to
  fit the chip.
- **Rounded colour matrix:** Y = R + 2G + 3B/8, Cr = (184R - 152G - 32B)/128, Cb = (3B - R - 2G)/2. Neutral stays
  exact; luma and Cr are within ~3 %, Cb is 5 % stronger.
- **Specks and dashes share one generator** (~19 % of lines, 8-39 px, 1-3 lines).
- **Output whites ~763** after the 13/16 luma scale (required against the HD HDMI bars).
- **The olive fill drifts slowly** as the ribs flow (the key restarts the phase at the flow value, as in the proto).
- **Picture shift:** the generated avid offsets the picture 2-4 px to the right.
- **LC:** 7,189 is slightly over the ~7,100 budget; all configs closed on the first seed.
- **SD 4:3:** the same composition is squeezed horizontally (the layout spans the width; nothing here is round).
  720p matches 1080p.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023) so the full camera effect shows on load. This is a picture-type program (K6 sets how strongly the camera image itself shows), so its camera range is unchanged (the +25 % top boost went only to bend / element programs). The vmprog was repacked from the same bitstreams (only the settings block differs).
