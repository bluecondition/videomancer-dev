# Cusp — Heirloom #17, rebuilt from the photo

**v0.5.0 (2026-10-27): built, NOT flashed.** Rebuilt from `photos/WIN_20230315_00_30_57_Pro.jpg` after
Ron judged v0.1.0 too abstract. It now redraws the photo: rows of fine green/navy/blue stripes around
black cores, cut by right-pointing arrowheads with blue X tips; cyan "fish" that grow from a needle to fat
bullets left to right; broad rainbow bands (cyan, purple, white-pink, magenta) running up to the seams;
orange/yellow rows with nested cyan arcs and a red X after every reset; the left sync strip. Work files:
`.heirloom_work/rb_cusp/` (`notes.txt`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **Rows.** Period 520.0 lines (2.08 rows per frame); mirror axes at y = 259.5 (black-core row) and 519
  (orange row). Hard seams at y = 90, 443, 609, 963: black-core rows (A zone) 353 lines, orange rows
  (B zone) 166 lines, 7 lines off the mirror symmetry.
- **Two layers, one key.** Every pixel is either the L layer (the broad rainbow of the left strip) or the
  S layer (the fine stripes / black core). Both are functions of the vertical position only:
  - L: red 74 (A centre) → 248 (B centre); blue saturated in the A rows, falling to 27 at the B centre;
    green folded 2.9 times over the half period (peaks 36, 146, 234 lines from the A centre).
  - S: black below 82 lines from the A centre, then green folding every ~28–30 lines; blue 20 + 1.26 per
    line (saturating in the B rows); red 0 in the A rows, 40–70 in the B rows.
- **The key.** A horizontal oscillator (saw) against a threshold:
  - The oscillator period per row is 456–486 px in the A rows (486 at the row centre) and 392–410 px in
    the B rows (392 at the centre). It chirps across the frame (A 482 → 464 px, B 389 → 402 px). Its phase
    follows 1 − cos of the vertical position (~92 px deep): the pointed ">" chevrons at the resets (sharp
    tips from an extra |d| term).
  - The threshold is a horizontal ramp minus that vertical cosine, onset x ≈ 138 + 1450 (1 − cos) px,
    clipped between a floor of 0.24–0.30 (rising toward the seams) and a ceiling of ~0.78.
  - A rows: L where phase < threshold, starting at each reset. B rows: the mirrored threshold, so the L
    share of a cycle shrinks while the A rows' grows: A 0.38 / 0.76 / 0.77 (needle at x ≈ 795, bullet tip
    at 1452, fat bullet to the edge), B 0.24 / 0.24 / 0.45 / 0.78 / 0.76.
  - The left sync strip (the oscillator held) ends at x = 206 (far from the row centre) to 253 (at it).
- **Edges.** At every key edge the key is low-passed per colour channel before the channel's clip / fold:
  roughly red 5, green 11–15, blue 2–4 px. That makes the green rims, the blue X at each black tip and the
  nested arcs. After each reset the L layer's vertical gain rings: A +0.62 e^(−x/22) cos(2πx/51 − 0.34),
  B −0.76 e^(−x/29) cos(2πx/45 − 0.94). That makes the X's, red in the orange rows.
- **No tearing** (mirror-row shift test: 0/1 px).
- Fit: edge least squares, then image-rms coordinate descent on spline tables (`opt5..opt10.py`). The
  integer model (hardware hues, output filters) is 30.3 RGB rms from the photo at Bend 0; 4×4 region means
  within 7 levels; local luma contrast 28.0 (photo) vs 28.9.

## What it does
1. **Per line:**
   - Line phase q (virtual lines × Rows); zone A/B from q; distance from the zone centre.
   - Nine piecewise-linear tables (knots every 1/64 of the row period):
     - phase start, scaled by Point, plus the drift;
     - phase increment, scaled by Cusps × kx;
     - threshold floor, ramp start and ceiling, all scaled by Point;
     - sync-strip width, scaled by Point;
     - the S layer's red, green and blue pre-signals (green × Stripes; B rows −2 fold cycles).
2. **Per pixel:**
   - Key and threshold:
     - phase DDA (second order, the chirp); floor and ramp DDAs;
     - u = frac(phase + source × Bend), Flip reverses it;
     - threshold = max(floor, min(ceiling, ramp));
     - key: A rows L = u < threshold (or inside the sync strip); B rows L = u ≤ 1 − threshold.
   - L layer:
     - virtual distance since the last S → L edge → ringing gain → L layer index;
     - L ROM: red, green fold argument, blue, red tint; red tint × (u − 0.35).
   - Mix and colour:
     - three soft keys (two-pole low-pass per channel);
     - s = S + K (L − S);
     - red and blue clipped, green folded by a triangle (dark navy below zero);
     - palette swap; RGB → YUV.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Cusps | cusps across: key-oscillator frequency ×0.4 … ×2.5 (1× at the centre = the photo) |
| K2 Rows | rows down the frame: row period ×0.35 … ×2.8 (0.73 … 5.8 rows; 2.08 = the photo) |
| K3 Fish | how fast the fish grow across the frame: threshold ramp 0 … 2× (0 = thin bands only) |
| K4 Speed | the cusps stream right / left (bipolar, still in the centre ±16, square law); default ≈ 0.5 px/field to the right, so the cusps grow into fish as they travel; max ≈ 9 px/field |
| K5 Stripes | density of the fine stripes ×0.4 … ×2.5 |
| K6 Bend | how far the source (camera luma / Osc) bends the cusps: 0 … ~0.3 cycle at full-scale luma, default 30 % |
| S7 Flip | Right / Left: the key ramp runs backwards, the arrowheads point left |
| S8 Rows | Alternate (the photo) / Same: every row a black-core row, no orange rows |
| S9 Glow | Soft (the photo) / Hard: key edges without glow, rims, arcs or X's |
| S10 Palette | Photo / Swap: red and blue exchanged, an amber / orange version |
| S11 Source | Video (default) / Osc |
| P12 Point | the vertical modulation (chevron depth, needle and fish shape, strip edge): 0 % = flat vertical bars; 47 % = the photo; 100 % = 2.4× deep needle spikes |

Glides: K1, K2, K3, K5, K6 and P12 move 1/8 of the gap per field (deadband 4).

## Video
- **How the camera enters:** camera luma is added to the key oscillator's phase, so the arrowheads, fish
  and bands bend around the picture. At the default Bend the composition still reads clearly (GHDL render
  fed the selfie collage in `cmp_final.png`).
- **Osc mode:** two slow drifting triangles (~0.9 cycle across, ~1 cycle down, ±0.6 of the camera range)
  stand in for the camera on the same path; Bend stays live.
- **What it never does:** the video tap is luma into geometry only; it never reaches the output.

## Engine
- **Per field (vblank zone, one serial multiplier, 15 slots):**
  - glides and knob curves (two segments about the centre);
  - Speed² × 349 (drift), Cusps × kx, (Cusps × kx)² → chirp per zone;
  - ramp slope × Fish per zone, line step (Rows), top-line phase;
  - floor / ramp slopes × kx are per-timing-class constants; drift accumulator.
- **Per line (kicked at g_start, ~280 clocks, fits SD's 858; line 0 in the vblank zone):**
  - |q|, zone, distance;
  - nine table ops, each: two knots from a 1024×16 knot ROM, 8-bit interpolation and a control scaling
    on a 28-bit serial multiplier;
  - the DDAs load the results at g_start; the per-line constants are committed there too.
- **Pixel path (32 stages):**
  - 32-bit phase DDA with chirp, 28-bit floor / ramp DDAs, 8×6 bend product, 16-bit threshold compares;
  - key, Flip, virtual distance counter, env ROM → 10×8 L index product → L ROM (256×48) plus a fabric
    copy, 8×7 red tint product;
  - three two-pole soft keys (one rate per channel: red τ ≈ 5.9, green 12.3, blue 2.2 px at HD; SD and
    720p one shift faster);
  - 7-bit K × (L − S) as gated shift-add pairs;
  - clip / fold, swap, RGB → YUV (2-term constants, luma pre-boosted 16/13, summed directly);
  - output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777); blanking gate on the
    generated avid.
- **Memory:** 11 EBR.

  | Block | EBR |
  |---|---|
  | knot ROM | 4 |
  | L ROM | 3 |
  | env ROM | 1 |
  | sync delay ring (32 clocks) | 1 |
  | chroma box delay rings (8 clocks, U and V) | 2 |

  The rings carry `no_rw_check`; the yosys log has 0 `emulate_transparency`.
- **Contracts:**
  - heirloom v0.2.0 shell: virtual 1080-line coordinates, generated avid, vblank zone, measured width;
  - one 32-clock latency in every mode (the syncs go through the block-RAM ring, exactly 32);
  - colours in the hardware convention (U = Cr, V = Cb).
- **Fit history:**
  - The first RTL (a direct port of the integer model) came to 9589 LC (124 %).
  - Standalone yosys ablations: soft-key alternatives 1194 LUT, K × (L − S) multipliers ~1000, other
    pixel multipliers ~920, line engine ~640.
  - Fixes: one rate per soft key, narrower products, blue tint dropped, line engine kicked at g_start
    with a 28-bit multiplier, slopes as class constants, delay lines in block-RAM rings.
  - Result: 7204–7224 LC.
  - A shared-post-adder rewrite of the line engine was larger and was reverted.

## Status
- v0.5.0 built 2026-10-27 with router2. **All six configs at seed 1.** Every bitstream passed iceunpack.

  | Config | Fmax (MHz) |
  |---|---|
  | HD Analog | 77.14 |
  | HD HDMI | 78.88 |
  | HD Dual | 78.15 |
  | SD Analog | 79.41 |
  | SD HDMI | 83.69 |
  | SD Dual | 77.30 |

- Size: 7204–7224 / 7680 LC (94 %), 11 / 32 EBR.
- Verification:
  - GHDL 720p60 Osc render vs the integer model: 0.53 mean abs after the avid / drift offset.
  - Motion checked: max Speed moves the cusps right ~8–9 px per field.
  - Video mode checked (selfie collage).
- **Not flashed.** Presets are hand-written placeholders (Photo, Flat Bars, Needles, Amber Swarm, Hard Left).
- If timing regresses: LC is at 94 %; try router1 or other seeds before restructuring. The next LC to
  reclaim is the chirp (~150 LC, costs 2.3 rms) or the red tint (~120).

## Deviations from the photo
- The fine stripes stop straight at the left sync strip instead of curling into it.
- The local wobble on fish 3's outline (a capture glitch) is not modelled.
- The orange rows' cyan bands show about 2 nested arcs instead of 3–4; the red X is a little smaller.
- Soft-key rise and fall rates are unified, and the blue tint along the bands is dropped (needed to fit;
  0.15 rms). The left strip's red bump along x is approximated by a constant tint.
- The camera's softness is reproduced only by the output filters, so hardware will look crisper than
  the photo.
- The default Bend (30 %) wiggles the cusps about ±20 px in Osc mode compared with the static photo;
  Bend 0 reproduces the fit exactly.
- Resolution: 4:3 SD shows the whole layout about 19 % narrower relative to the height. The glow / rim /
  arc widths come out about 1.33× wider than HD at SD and about 0.75× at 720p (the soft-key rate has two
  settings).
