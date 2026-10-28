# Cathedral — teal egg-crate arches around a pink diamond
**v0.5.0 (rebuilt 2026-10-27): built, NOT flashed.** Heirloom #41, from `photos/WIN_20230602_01_30_58_Pro.jpg`
(the same patch session as facet, four minutes later). v0.1.0 drew parabola arches over a rainbow and Ron judged it
too abstract. This version is rebuilt from a numeric reading of the photo: a float prototype iterated to a
near-replica, then an integer model, then the RTL. Work files are in `.heirloom_work/rb_cathedral/`.

## Reading of the photo (measured)
- **An RGB colourizer.** The channels separate cleanly:
  - **G** is one vertical ramp, 246 at the top → 143 mid-frame → 0 at the bottom (a 4th-order fit, rms 2.4/255).
    The green ground runs ~12 above it.
  - **R** is a hard key: the warm stripes plus the pink shapes (R ≈ 236 → 254 going down the frame).
  - **B** carries the pink layer and the teal bands.
  - So the ground is (0, G, 0) green → black, warm is (R, G, 0) yellow → orange → red, pink is (R, G, B)
    white / peach / pink / magenta, and the bands are (0, ~100, ~120).
- **Mirror axes.** Flip-correlation puts them at x 997 and y 518 (B channel; R gives 1003 / 520).
- **Teal bands.** Every band edge is a contour of one egg-crate field, F = h(|Y|) − g(x′).
  - x′ is the distance to the nearest saddle column: x 527 / 1467, period 940.
  - g ≈ 0.88 sin(πx′/840), flat from 420 to 470. h ≈ g(0.91|Y|), flattening at 0.85 near the frame edge, which is
    what turns the arms of the V's and teardrops vertical at the top and bottom.
  - A nonparametric separable fit over 2,400 edge points from the folded quarter lands within 1–3 px rms.
  - Contour levels: −0.258 (the lens arc), −0.049, 0.212, 0.424, 0.689. Bands are a periodic fold of F: period
    0.473, duty 0.445, keyed off inside the lenses.
  - That one field gives the lens rings, the X crossings, the V with its tip at (527, 440), the teardrop with its
    tip at (527, 215), the M shapes and the blue frame-corner regions.
- **Band colour** goes from blue (0, 82, 150) at |Y| 0 and 480 to teal-green (0, 147, 78) at the wedge rows.
- **Pink shapes.**
  - Diamond: |Y| + 0.82u < 227, centred at (987, 518).
  - Triangles: |Y| − 0.86u > 293.
  - Wedges: ||Y| − 261| < 0.09(u − 122), tips at x 878 / 1123.
  - Their B is a base level (166 → 190 going down) plus half the band's B where a band lies under the shape. That
    is the white core of the top triangle and the lavender-versus-peach split of the wedges.
  - B is soft in x-order, with a ~30 px edge.
- **Warm stripes and spikes** are the same pink distances compared against a vertical triangle wave of period 68
  lines: warm where min(dD/250, dE/300, dW/80) < 0.2 + 0.8·tri(y). This matches 93.1 % of pixels (a sawtooth gave
  89.9 %, all-cool 61 %). It produces the spikes along the lens arc and the patches in the band gaps.
- **Band edges** (x-order):
  - The rising edge is soft: G over −9 … +6 px, B from −14 to +40 px.
  - The true key is the fitted contour widened 5 px on the left and 11 px on the right.
  - The trailing end darkens: G × 0.48 and B → 0 over the last ~30 / 24 px.
  - After the end, G holds at ~0.57 × the ground for ~9 px and recovers over 18 px; R ramps back over 1 … 18 px.
- **Texture.**
  - Band-edge jitter is 0.6 px rms (lag-1 0.7).
  - Field-noise streaks appear only where a contour runs flat (the frame corners).
  - Soft camera blur.

## What it does
1. **Band field.** A folded phase DDA gives x′, and a 512-entry ROM gives g. A per-line ROM gives h(|Y|·ky).
   The source (luma × K6) and a ±0.004 per-line noise are added to F. Then phase = (F − c1)·M + flow, and the band
   is where F > c1 and frac(phase) < 0.445 (S9 inverts this).
2. **Pink and warm keys.**
   - u = |X − 990|, and per-line terms give dD, dE, dW. The source bends them too.
   - Pink where any of them is below 0; warm where any is below spill × (250, 300, 80) × (0.2 + 0.8 tri).
   - The stripe triangle comes from a per-line DDA.
3. **Edges.** The band key passes through a 20-pixel look-ahead window. Groups-of-four priority encoders give the
   first rise and first fall ahead, and run counters give the widened true key and the signed distances to its
   rising edge (dr) and true end (e). Those distances address two curve ROMs: the rising curves, the trailing
   shade, and the post-band curves (weights in 1/16).
4. **Colour** is mixed in 8-bit RGB:
   - the ground is warm, cool or pink;
   - the band target is blended in by aG / aB and shaded;
   - after the band, R and G ramp back;
   - pink B is the base plus half the band's B, through two 1/8 x-order poles, and the larger of that and the band
     B wins.
5. **RGB → YUV** uses three contribution ROMs (luma pre-boosted ×16/13), with a luma knee above 848 so drawn
   whites stay at or below ~777. Night rotates R G B → B R G. Then the output conditioning and the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Vaults | lenses / saddles | lens period: ×0.41 … ×2.46 lenses (the photo at 50 %) |
| K2 Arch | V's, teardrops, arch height | vertical scale of h, ×0.5 … ×2 |
| K3 Diamond | pink diamond | radius 0 … 460 lines (226 = the photo) |
| K4 Speed | motion | bipolar, ±24 deadband. Bands stream out of (or into) the lenses at 1 band period per 5.4 s at the default; the stripes roll; the diamond (±6 %) and the wedges (±15 %) breathe; one master phase |
| K5 Spill | warm stripes and spikes | reach of the keyed ramps around the pink shapes, ×0.35 … ×2.8 |
| K6 Picture | the whole field | source depth into F and the pink distances (default 20 %) |
| S7 Wedges | horizontal pink wedges | on / off (On) |
| S8 Shade | band shading | Soft (rising blend, dark trailing ramp, dark-green line) / Flat |
| S9 Invert | bands | bands and gaps swap |
| S10 Palette | all colours | Jewel (the photo) / Night (R G B → B R G) |
| S11 Source | — | Video / Osc |
| **P12 Tracery** | the teal bands | band period 1.89 … 0.118 in F (exponential): one bold vault ring → the photo (50 %) → dense filigree |

K1, K2, K3, K5 and P12 glide 1/8 of the gap per field and snap on the first fields.

## Video
The camera's luma is added to the egg-crate field and to the pink and warm distances. The band contours bend around
the picture, and the keyed ramps (stripes, spikes, wedge and diamond edges) swell on bright areas. At the default
20 % the photo's composition reads clearly. Osc substitutes a slow drifting triangle plaid (±48/255) through the same
path, so every video-path control stays live in Osc.

## Engine
- **Serial multiplier** (27 × 16, 16 steps).
  - **Per field (zone):** 11 jobs in 24-clock slots: pixels left of the axis; the phase increment × xs; the start
    phase; RD; flow, stripe roll and LFO phases; diamond breathing; the stripe-phase increment; the wedge offset.
    The knob curves come from a 1024 × 16 KT ROM (lens increment, h-index scale, spill, band gain M).
  - **Per line:** kicked at each g_start, it computes the next line: |Y|, the stripe triangle and threshold,
    j = |Y|·KJ → h ROM, TD / TE / TW = spill × threshold × (250, 300, 80), the ROM colours, the pink-line terms and
    the LFSR noise. The results are copied at line end + 14 (field stages) and + 48 (composite stages).
- **Pixel path, 56 stages:**
  - 1–11: fields and keys.
  - 12–32: the window (the key 20 px ahead).
  - 33–37: counters, distances, curve ROMs.
  - 37–41: blends.
  - 41–47: matrix ROMs and the luma knee.
  - 48–56: conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], 13/16) and the gate.
- **ROMs (20 EBR):** g 512×16, h, band colour, curves ×2 (256×16 each), ground 512×16, KT 1024×16,
  matrix 3 × 256×32, source line buffer 1024×8 (`no_rw_check`). The tables are generated by `cathedral.py`.
- **Shell:** the heirloom v0.2.0 shell — timing-ID virtual-1080 steps, generated avid, one latency in every mode
  (C_LAT 56), colours in hardware convention.

## Status
v0.5.0 built 2026-10-27, router2, **seed 1 on every config**. The rebuild was deterministic.

| Config | Fmax (MHz) |
|---|---|
| HD Analog | 81.12 |
| HD HDMI | 77.85 |
| HD Dual | 75.10 |
| SD Analog | 74.52 |
| SD HDMI | 71.54 |
| SD Dual | 75.30 |

SD Fmin is 27 MHz. 6851–6864 / 7680 LC (89 %), 20 / 32 EBR. All bitstreams pass iceunpack. NOT flashed. HD Dual
has the thinnest margin: if a later edit regresses it, try `NEXTPNR_ROUTER="--router router1"` first.

Verification:
- GHDL 720p vs the integer model: median |d| 0, mean 1.1–1.2 codes.
- SD field-to-field: every contour, wedge edge and the diamond edge move.

## Deviations from the photo
- **Luma cap.** Whites and yellows (top triangle, top wedges, yellow stripes) sit 50–85 codes below the photo, to
  keep drawn luma at or below ~777 after the 13/16 scale. The greys are untouched.
- **Outer half-lenses** are periodic copies, so the blue frame-corner regions are ~30 px shorter left of x ≈ 57.
- **Stripes.** The photo's ±10-line phase irregularity is not reproduced; the stripes are a clean 68-line wave.
- **Jitter and noise.** The 0.6 px band-edge jitter is left out (invisible after the conditioning). The corner
  streaks are a different noise realisation.
- **Shape centre.** The diamond, triangles and wedges share x = 990 (photo: 987 / 988 / 1000), so the wedge tips
  are ~10 px off.
- **Camera blur** is not reproduced beyond the output conditioning.
- **SD** shows the central 1440 virtual px, with the frame-corner half-lens regions cut. Every field renders the
  full picture (line-doubled).
