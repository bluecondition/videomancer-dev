# Herringbone — blue and white stripes that zigzag band to band, bunched by a rolling oscillator

**v0.1.0 built 2026-11-09, not flashed.** Heirloom phase 3 (video clips), from the clip `WIN_20231119_00_47_47`
at 0.5 s (`photos/vid/WIN_20231119_00_47_47_t0p5.png`). The still shows horizontal bands that alternate
lavender / blue and cyan / navy. Each band is filled with soft vertical stripes that lean one way in one band
and the other way in the next: a herringbone zigzag, slight at the left edge and steep at the right. The bands
are thin at the top, two tall ones sit in the lower middle, and they thin again at the bottom. The stripes widen
going down the frame. In the clip the tall-band zone rolls upward and the stripes swing as the bands pass under
them. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_herringbone/` (`notes.md`, `vfit.py`, `rowfit.py`, `x0fit.py`, `fit5.py`).
- **One colouriser.** palfit (k = 10) finds four main colours and six transition colours, all lying on one
  polyline (median distance 3.3 RGB):
  - navy (5, 14, 89), 16 %;
  - blue (20, 115, 214), 20 %;
  - cyan (146, 215, 237), 19 %;
  - lavender (232, 212, 232), 21 %;
  - the in-between colours (4, 40, 125), (7, 75, 169), (53, 150, 238), (100, 190, 240), (178, 210, 238),
    (208, 212, 235).

  Every stripe edge travels along that path: lavender → cyan → blue in the A bands, cyan → blue → navy in the
  B bands. So the picture is a 1-D colouriser over v = stripe (0..2) + band A (+1). The A rows span v 0.9–3.0
  and the B rows 0–2.05.
- **Bands = a vertical VCO, exponentially FM'd.**
  - Boundaries (hard row edges): 32, 82, 130, 185, 248, 336, 512, 788, 948, 1031. The band heights are 50, 48,
    55, 63, 88, 176, 276, 160 and 83 lines.
  - Fit: θ(y) = Σ f, with f = fmax · 2^(−D (1 + cos 2π(y − yc)/P) / 2). fmax = 0.0103 cycles/line (48-line
    bands), D = 2.70 octaves, P = 1119 lines (a free-running modulator at about frame rate), yc = 633.
    The rms is about 2 lines; a triangle modulator fits 2.6× worse.
  - Band A (lavender / blue) is where frac(θ) < ½.
- **Stripes = a horizontal ramp whose period is driven by the band VCO's triangle.**
  - Per row, the rising-edge period is constant along x. It is linear in the CV:
    p(y) = 168.9 + 0.0706 y − 4.64 T(θ), with rms 0.95 px. T = +1 at the start of a band A and −1 at its end.
    That swing is the zigzag. Linear-in-frequency fits 3.1 px and log-frequency 1.95 px.
  - The phase is (x + 178.9) / p(y) + C, with one constant C for every row (rms 0.9 px). The ramp resets 179 px
    before the active start, which is why the zigzag is slight on the left and steep on the right.
  - The period runs from 169 px at the top to 242 px at the bottom.
  - **Left-edge squeeze:** content is pushed right by 32 px at x = 10, 14.5 at 130, 4.6 at 210 and 0 by 380.
    The first stripe of every row is 7–12 % short.
- **Edges** (in v, constant in px):
  - light → dark is a sharp drop then an exponential tail (τ ≈ 11 px);
  - dark → light is an S-curve, close to a critically damped two-pole response (τ ≈ 6 px).

  Fitted as two one-pole stages: rise 1/6.6 then 1/5.7, fall 1/11 then 1/1.9. Duty 0.499.
- **Texture:** low noise (sd 1–5 RGB) and faint blue-channel ringing at rising edges. The capture borders are
  dark: top 2 rows, bottom 5, left 4 columns.

## Motion in the clip
- The strip's first three frames are the clip's first frame (clamped), so the valid samples are t = 0, 0.5 (the
  still), 1.0, 1.5, 2.0 and 2.5 s.
- **The bunch rolls up.** Refitting yc per frame gives 682, 634, 590, 546, 498 and 450: −94 lines/s, steady.
  That is one 1119-line cycle per ~11.9 s.
- **What stays fixed:**
  - The band phase at the top line stays at 0.66–0.72, so the band VCO is reset each frame.
  - The H period and phase at the top (169 px) and bottom (242 px) are identical in every frame.
  - So the stripes never drift; they only swing as the bands' triangle passes under them.
- **Ron's knob = the band FM depth.**
  - Clip `WIN_20231119_00_46_57` is the same patch: the same fmax, and an identical H section
    (p = 169.1 + 0.0689 y − 4.7 T, x0 = −178), but D = 1.11 octaves (bands of 47–105 lines).
  - Clip `00_47_47` at 11.5 s has D = 2.72 again, just rolled (yc 740).
  - So the knob is K5 Bunch; the alternative clip sits at about K5 = 210.
- **The program:**
  - The default Roll (693) rolls the modulator 94 virtual lines/s upward, with the band phase pinned at the top.
  - The RTL-exact integer model at t = −0.5 … +1.5 s tracks the m-frames frame by frame (`cmp_motion.png`).

## What it does
1. **Band VCO** (per line):
   - ψ = ψ0 + y / 1119 (ψ0 rolls per field);
   - m = (1 + cos 2πψ) / 2 from a cos ROM;
   - f = fmax · 2^(−D m) from an exp2 ROM and a shifter;
   - θ += f;
   - band A = frac(θ) < ½;
   - Tq = the triangle of θ.
2. **Stripe period** p = A (1 + g y/1080 − z T) for the next line.
3. **Stripe rate** F = N / p, on a serial divider in virtual px.
4. **Line-start phase**: C + X0 · F.
5. **Beam**: the phase steps F per pixel, F + F/8 below 210 virtual px and F + F/32 to 420 (the left-edge
   squeeze). With Mirror it steps −F left of the centre.
6. **Pixel path**:
   - add the camera / Osc luma × Etch;
   - square (light where frac < ½) or ramp;
   - edge stage 1 (symmetric 1/8 per px);
   - edge stage 2 (rise 1/4, fall 1/2);
   - colouriser ROM at v = s + 64 · band A;
   - two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V, luma 13/16, gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Stripes | the stripe period, ×¼ .. ×4 (512 = the still, 169 px at the top). 0 = fine twill (~42 px); 1023 = a few broad slabs |
| K2 Fan | how the stripes widen going down: 0 = they narrow to 15 % at the bottom (moiré fan), 504–520 = even top to bottom (deadband), 772 = the still (+45 %), 1023 = 3× wider at the bottom (a perspective floor) |
| K3 Bands | the band VCO's top rate, ×¼ .. ×4 (512 = the still's ~48-line bands). 0 = a few tall bands; 1023 = dozens of thin zigzag bands |
| K4 Roll | bipolar (±8 deadband, square law): the bunch zone rolls up (+) or down (−); 693 = the clip's 94 lines/s, full = 800 lines/s. The stripes swing as it rolls |
| K5 Bunch | the band VCO's FM depth (Ron's knob in the clip): 0 = even bands (a plain herringbone), ~210 = clip 00_46_57, 512 = the still (2.70 octaves), 1023 = 5.4 octaves, one huge band with slivers crowding the edges |
| K6 Etch | how far the camera (or the Osc) pushes the stripes sideways: 0 .. one full period per full luma (square law); 512 = a quarter period |
| S7 Mirror | Off (the still) / On: the right half mirrored onto the left, so the chevrons meet in a V at the centre |
| S8 Colours | Blue (the still's navy / blue / cyan / lavender) / Ember (maroon / red / amber / cream) |
| S9 Weave | Off / On: every cyan-navy band's stripes shifted half a period, giving a woven check |
| S10 Wave | Square (the still) / Ramp: each stripe a gradient through the whole palette with one hard edge |
| S11 Source | Video / Osc |
| P12 Zigzag | the herringbone depth (the triangle's swing into the stripe period, square law, glided): 0 = straight stripes, only the fan; 337 = the still; 1023 = 9× the still, deep chevrons slanting near-horizontal on the right |

`sheet.png` (from the float proto with the control map) shows every control at min / default / max and P12 at
0 / 50 / 100 %. All four S7–S10 switches flip the whole frame at the default state of every other control. Every
knob stays visible at P12 = 0.

## Video
- **Video mode (the default).** Camera luma enters the stripe oscillator's phase, as a patch cord into the H
  ramp would:
  - the luma is (Y >> 2) − 16, smoothed 1/4 per pixel, through a one-line buffer;
  - it is added at up to one stripe period per full luma.

  The stripes bend and fray around the picture. At the default Etch (a quarter period) the bands and the zigzag
  still read clearly (`cmp_final.png`, bottom panel). With no input, Video mode is the still: the RTL is
  bit-exact with the integer model.
- **Osc mode.** The shell's drifting triangle lattice feeds the same path: 320 × 270 virtual px, drifting about
  38 px/s across and 24 lines/s down. The stripes ripple into soft waves. Etch stays live.

## Engine
- **Per-field sequencer** (vblank zone = no avid for 1024 clocks):
  - one serial shift-add multiplier, 13 slots, with operands pre-muxed and no zone-counter decode on its path;
  - a shared glide for K1, K2, K3, K5, K6 and P12;
  - exp2 ROM. Per field it computes:
    - A = 2702 · 2^((k1 − 512)/256) (Q4 px);
    - the fan Bq = A |g| · 971 / 2^16, with g two-slope from K2;
    - Z = A z;
    - D = k5 · 1379 / 1024 (Q8 octaves);
    - E0 = (k3 − 512) + log2(line step) + 4452;
    - the roll step (k4 − 512 ∓ 8)² · 3239 / 2^16 (Q20 per field);
    - the ψ step per line;
    - the etch gain;
    - the per-mode divider numerator N, the line-start offset X0 (Q1 px, Mirror variant) and the edge-filter
      coefficient set.
- **Per-line engine**, kicked at each line start (and once in the zone for line 0). It computes the NEXT line in
  about 110 clocks, which fits SD's 858:
  - cos ROM (512 × 10, index rounded) → D · m → exp2 ROM → a 0–15-step serial shifter → f;
  - θ (Q24) and the band / triangle;
  - Bq · y and Z · Tq;
  - p, clamped to 8 px;
  - a 30-step restoring divider, Fq = N / p (Q24 cycles per output pixel);
  - X0 · Fq → the line-start phase;
  - F + F/8, F + F/32 and −F.
- **Pixel path** (21 stages, C_LAT = 20):
  1. Beam.
  2–3. Line-buffer luma.
  4–5. Etch nibble products.
  6. Phase + etch.
  7. Square / ramp target.
  8–9. The two edge stages (one-clock feedback each; the stage-2 direction is the previous pixel's).
  10. Colouriser address.
  11. Palette ROM (EBR).
  12. Fabric copy.
  13–14. Delay.
  15. Compose (off-avid colour = the palette's navy / maroon).
  16–19. Two boxes, with the running-sum resync.
  20. Luma 13/16.
  21. Gate.
- **ROMs / RAMs (12 EBR):**
  - palette 512 × 32 (2 palettes × 256 entries, Y pre-scaled 16/13 and capped at 940);
  - two exp2 copies, 256 × 16;
  - cos 512 × 10;
  - line buffer 2048 × 8.

  All tables are generated by `herringbone.py` into `herringbone_pkg.vhd`.
- **Contracts:**
  - generated avid (parity-locked start);
  - one latency (20) in every mode;
  - blanking gate on the latency-aligned generated avid;
  - output conditioning before the gate;
  - virtual 1080-line coordinates (SD shows the centre 4:3; the edge filter is 1/4 per pixel at 720p and SD);
  - colours in the hardware convention (U = Cr, V = Cb);
  - the box sums resync to the off-avid colour after 8 blank samples.

  Only the line buffer emulates transparency (collisions possible, read-before-write kept). dupin: 0
  `CARRY I0==I1`.
- **Verified:**
  - check (with the pkg) is clean.
  - GHDL 720p60 at decimation 1, black input, default knobs: **100.00 % bit-exact** with `imodel.py`.
  - K4 = 1023: bit-exact too, rolling 12505 Q20 (13.3 lines) per field in the clip's direction.
  - Float proto vs still: 8.45 mean RGB. Integer model vs still: 11.2 (the rest is the luma cap).

## Status
- **v0.1.0 built 2026-11-09, not flashed.** router2, **all six configs on seed 1**. Post-route Fmax:

  | Config | Fmax |
  |---|---|
  | HD Analog | 91.99 MHz |
  | SD Analog | 84.17 MHz |
  | HD HDMI | 93.71 MHz |
  | SD HDMI | 92.90 MHz |
  | HD Dual | 93.14 MHz |
  | SD Dual | 90.99 MHz |
- 5974–5986 / 7680 LC (78 %), 12 / 32 EBR. iceunpack OK ×6, and the packed config carries the labels.
- Output: `out/rev_b/ron/heirloom/herringbone.vmprog` (410,010 bytes).
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh herringbone` (or `./build_heirloom.sh herringbone`).
- Presets are hand-written. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Lavender is darker.** It comes out at Y 777 (the shell's white cap before the 13/16 scale) against the still's
  821, so it reads slightly darker and pinker. Cyan, blue and navy are exact.
- **Band boundaries** are within 4 lines of the still (the integer model, fitted on the boundaries). The band
  phase is pinned at the top line; in the clip it wanders 0.66–0.72.
- **Edge filter.** Its coefficients are powers of two (fit 9.4 vs the best free fit 8.8 RGB). The blue-channel
  ringing at rising edges is not drawn.
- **Left edge.** The capture borders are not drawn, but the boxes fade in from navy over ~8 px at the left
  edge. The left-edge squeeze is two linear segments (F/8 to 210 px, F/32 to 420 px).
- **Additions beyond the clip.** Mirror drops the left-edge squeeze; the Weave, Ramp and Ember looks are new.
- **Resolutions.**
  - 720p shows the same geometry as 1080p, with edges about 25 % crisper (filter per output pixel).
  - SD shows the centre 4:3 (x 240–1680 virtual px) with the same bands. Edges there are softer, because the
    boxes work per output pixel.
  - P12 near 100 % and K1 near 0 alias at SD.
