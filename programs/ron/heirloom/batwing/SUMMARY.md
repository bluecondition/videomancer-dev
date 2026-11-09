# Batwing — a mirrored magenta bat-wing kaleido with a lilac bowtie in rolling violet, yellow and green bands

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20230927_23_58_07` at 2.5 s
(`photos/vid/WIN_20230927_23_58_07_t2p5.png`). Three mirrored tiles across the frame. In each a lilac, camera-textured
"bat wing" bowtie with ragged, torn side edges sits inside nested contour bands: magenta, a violet ramp, dark green,
violet, yellow and dark-green pairs, compressing toward the bowtie. The seams between tiles are magenta columns that pinch
into a long dark-green waist, and a magenta dome sits under every bowtie. In the clip the vertical oscillator rolls the
bands down, one 1117-line period every 7 s, while the seams and the bowtie's sides stay put. Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_batwing/` (`notes.md`, `mirror.py`, `axis_track.py`, `kymo.py`, `fit2.py`,
`fit3.py`, `fitA.py`, `seq.py`, `proto.py`, `imodel.py`, `sheet.py`, `thincheck.py`).
- **Tiles.** Vertical mirror axes (flip correlation) at x = 60, 553.5, 1029, 1497.5: tile centres at 60 and 1029,
  seams at 553.5 and 1497.5. The half-tiles are 493.5 / 475.5 / 468.5 px (the analog ramp is slightly compressed to the
  right); the program uses a uniform 952 px tile (2.02 per frame) centred on x = 1029.
- **Palette** (k = 12, whole still): violet-lilac (178,70,249) 11.8 %, yellow (215,223,27) 11.0 %, dark green
  (17,54,7) 10.0 %, dark purple (70,21,155) 9.9 %, navy (59,25,109) 9.4 %, magenta (252,0,255) 9.4 %, olive (132,127,56)
  7.4 % (yellow/green edge blends), dark (44,31,59) 7.0 %, violet (99,8,243) 6.9 %, blue-violet (82,15,200) 6.8 %,
  (146,23,249) 5.9 %, (198,19,254) 4.4 %.
- **Band order** (from the low end; the same sequence on the top and bottom paths at the tile centre): magenta flat ->
  an R ramp to blue-violet -> dark green -> violet -> yellow / dark green / yellow -> violet -> yellow / dark green /
  yellow -> violet -> dark green / violet repeats -> yellow -> the bowtie. About 13 bands between the magenta edge
  (y 44 top, 842 bottom at x = 1029) and the bowtie (y 281 / 587).
- **Field** (fitted; the RTL arithmetic is in `imodel.py`):
  - xp = the distance from the seam, a mirrored horizontal triangle (0 .. 476 px).
  - The rolling vertical oscillator: period 1117 lines, peak row 441 in the still; its waveform Dv(phase) is asymmetric
    (measured from the band positions on the top path, 27 knots) and rounded at the trough (the magenta dome).
  - The wing sides: sA = 0.0282 (xp + 65 S (1 - xp/170))^0.6 - 0.165, a sqrt-like ramp off the seam, bent by a static
    frame-locked hump S (a clipped parabola centred on line 704, half-width 410). Fitted to the seam column (magenta to
    y 374, ramp to 486, dark green 498-890, symmetric about 704) and the side band positions at y = 250 (rms 0.012).
  - The bowtie's top and bottom: sB = (795.6 - Dv (1 + 0.00154 xpb) - 0.226 xp) / 419.6 (two-contour solve: magenta
    edge Dv 397 at the centre and 506 at xp 288, bowtie edge 155 and 225), xpb = xp with the centre fold rounded
    (r 120 px), which rounds the magenta dome.
  - s = min(sA, sB) -> the palette.
  - The bowtie is a separate key, not a level of s (the sides show ~6 bands before it, the top and bottom ~13):
    xp > 188 px and Dv < 155 + 0.40 (476 - xp). Measured edges: sides at x 745 / 1320, top 175 -> 280, bottom 720 -> 589.
  - Violets read darker (navy / dark purple) where the side term wins.
- **Textures.** Soft edges (3-6 px). The bowtie's side edges tear in horizontal strips with yellow chunks; inside, a
  mottled lilac (R 150-210) with diagonal ripples and mirrored "A"-shaped camera islands; fine scan lines.
- **Fit quality** (integer model vs the still at the defaults): 4x4 region means within 8.9 RGB on average (worst
  cell 23.5; the RTL frame: 8.3 / 25.3); class shares photo / model: magenta 11.8 / 14.9 %, yellow 12.0 / 10.2 %,
  dark green 9.6 / 12.1 %, lilac 14.8 / 24.0 % (the photo's bowtie is broken up by camera islands); band positions on
  the centre column within 2-5 px of the photo's (per-pixel mean error is high, ~54 RGB, because ~13 thin bands are
  never pixel-aligned).

## Motion in the clip
- The clip (10.9 s, 108 frames at 10 fps, `axis_track.py`, `kymo.py`) shows the vertical oscillator rolling DOWN: the
  magenta dome's mirror axis moves 157-163 lines/s, and frame 0 repeats at 7.0 s (rms 36) -> period ~1117 lines, one
  period per 7.0 s. (Rigid block matching gives 115-120 px/s because the static seams pull it down.)
- The horizontal structure (seams, waist, the bowtie's sides) is static; the bowtie's camera content changes ("A"
  islands become "3 / B" and "?" shapes).
- No knob turned in this clip. The patch's other clips: 00_00_05 (green / yellow zigzag bands around a lilac core, no
  side term) and 00_02_04 (yellow / green hexagon rings with the lilac OUTSIDE: an inverted key) -> the Zigzag, Colours
  and Invert switches.
- **The program:** at the default Roll (772) the oscillator phase moves 40,188 / 2^24 per field: 2.67 virtual lines
  per field = 160 lines/s at 60 Hz (133 at 50 Hz), one period per 7.0 s. `cmp_motion.png` shows the integer model at
  -0.5 / +0.5 / +1.0 / +1.5 s beside the m-frames.

## What it does
1. **Tiles.** A horizontal phase accumulator (Q24 per tile), folded into xp = 0 (seam) .. 511 (centre) units. Mirror
   off = sawtooth.
2. **Wing sides.** sA = TA(xp + waist (184 - xp)/184) - 64 (TA: a 512-entry sqrt-like ROM), times the Bands gain.
3. **Bowtie top / bottom.** sB = LB0 + ((511 - xp) LE + (511 - xpb) L1) / 4096, with the line terms
   LB0 = (C0 - C1 Dv) G and L1 = Dv KM ws G from the waveform table, LE = KE ws G per field. Wingspan scales only the
   x-coupling about the tile centre (at 0 the bands are horizontal and keep the centre column's levels).
4. **Colour.** s = min(sA, sB), corners chamfered (16), plus the camera -> a 512-entry palette per colour set.
   Where the side term wins, the violet entries are replaced by a dark navy (Meadow: dark green), the photo's navy sides.
5. **Bowtie key.** xp + camera/4 >= 202 - fray (per 8-line strip, +-12, reseeded every 8 fields) and
   Dv - 155 < KW (511 - xp) + camera/4 -> lilac, with a built-in diagonal ripple and the camera's luma as texture.
   Invert keys s < 0.11 instead (lilac fills the magenta regions, the bands continue inside the bowtie).
6. **Output.** Chroma [1 2 2 2 2 2 2 2 1]/16 (two 8-boxes with the resync), luma [1 2 1]/4 on the same 8-px delay,
   luma 13/16, then the blanking gate (latency 20 in every mode).

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tiles | the bat wings across: x0.35 (one wide wing, 0) .. 2.02 (the photo, 512) .. x2.8 (six narrow wings, 1023), exponential |
| K2 Rows | the vertical oscillator's frequency: one huge bowtie (0) .. 1117 lines (the photo, 512) .. ~2.8 rows of bowties (1023) |
| K3 Roll | bipolar (+-8 deadband, square law): the bands roll down (+) or up (-); 772 = the clip's 160 lines/s; full = 4x |
| K4 Bands | band density between the magenta edge and the bowtie: x0.33 (a few wide bands, 0) .. the photo (512) .. x3 (fine hairlines, 1023) |
| K5 Waist | the static hump that pinches the seams: straight magenta seams (0) .. the photo's dark-green waist (512) .. a fat yellow waist bulging into the wings (1023) |
| K6 Etch | how far the camera (or the Osc) bends the bands and frays the bowtie: 0 .. ~3.4 palette spans per full luma (square law); 120 (default) = 0.04 span, a gentle bend |
| S7 Mirror | On (the photo) / Off: sawtooth tiles, one-sided wings with hard vertical seams |
| S8 Zigzag | Off / On: no wing sides: chevron zigzag bands across the frame and a zigzag lilac strip (the green / yellow alt clip) |
| S9 Colours | Magenta (the photo) / Meadow: yellow, green, purple lines and a dusky lilac core (the alt clip's palette) |
| S10 Invert | Off / On: lilac fills the magenta seams and domes, rings fill the bowtie (the hexagon alt clip) |
| S11 Source | Video / Osc |
| P12 Wingspan | the bowtie's horizontal coupling: 0 % = wings folded flat (horizontal bands, a horizontal lilac strip); 384 = the photo; 100 % = 3x: steep chevron wings, a narrow pointed bowtie |

Glides (1/8 of the gap per field) on K1, K2, K4, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) - 16 from a one-line buffer, box-filtered over 4 px (it bends,
  not shreds), times the Etch gain, is added to the field (the bands bend around the picture) and widens the bowtie
  key (the edges fray around it). Inside the bowtie the camera's own luma textures the lilac, as in the photo.
  With no input, Video mode is the still.
- **Osc mode.** A drifting 640 x 540 virtual-px triangle lattice feeds the same path at 4x the camera's gain plus a
  floor (gain = 4 Etch + 16, about 9x the camera's at the default): the bands wave and swing across the wings and the
  bowtie's edges ripple, an unmistakable flip at the defaults.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier, 13 slots (tile step, line-start phase, rows
  step and per-line step, roll speed^2 and step, waist, etch^2, wingspan, ws*G, LE, KM, KW), a 256-entry exp2 ROM for
  the exponential controls, the shared glide, the roll accumulator and the fray seed.
- **Per-line engine**, kicked at each line start, computes the NEXT line in ~90 clocks (fits SD's 858) on its own
  serial multiplier: the waveform Dv from a 1024-entry table at the oscillator's phase, L1 = Dv KM, the hump
  S = clip(1.15 (1 - dy^2/410^2)) and the waist term, LB0 = (C0 - C1 Dv) G, the fray (hash of the 8-line strip),
  the bowtie threshold and the ripple phases.
- **Pixel path (16 stages):** fold; waist product; TA index, the camera's 4-px box; TA address, xpb, camera product;
  511 - xpb, bowtie slope product; KM and KE partial products (split 6/6 and 6/7 bits), the key limits, the ripple;
  sA x Bands, KM / KE sums, the lilac's ripple and camera terms; sA + camera, the B sum, LB0 + camera, the lilac
  luma; sB; min and both differences; chamfer; palette address, key, lilac clip; palette ROM; compose from the
  palette word (shade = a constant navy / dark green). Then box A (17-18), box B and the luma [1 2 1] (19-20), luma
  13/16 with the gate (21). C_LAT 20.
- **Output filters:** the 8-sample tap delays of both chroma boxes (and the luma's) live in two 256-word block-RAM
  rings (no_rw_check, read 8 back), not in 400 shift-register flip-flops.
- **ROMs / RAMs:** TA 512 x 9, waveform 1024 x 10, fold rounding 512 x 8, palettes 1024 x 24, exp2 256 x 16, line
  buffer 2048 x 8, output rings 256 x 32 and 256 x 20: 21 EBR.
- **Contracts:** damier's generated avid; one 20-clock latency in every mode; the blanking gate; the output
  conditioning with the running-sum resync; virtual 1080-line coordinates (SD shows the centre 4:3); colours in the
  hardware convention (U = Cr).
- **Checks:** GHDL analyse / elaborate / synth front end clean (`check_pkg.sh`, which adds the pkg). Standalone yosys:
  4149 LUT, 2317 FF, 21 EBR, 0 `CARRY I0==I1` (dupin; 3 harmless LUT duplicate inputs), one transparency emulation
  (the line buffer, as in the shell). `thincheck.py`: 0 hard single-row lines >= 24 px at the defaults,
  P12 0 and P12 100 (four roll phases each), and at K2 / K4 max and every switch.

## Status
- **v0.1.0 built 2026-11-09 with ROUTER1, not flashed.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh batwing`.
  - **Build seeds: SD HDMI = seed 2** (seed 1 stalled the router, timeout); the other five are seed 1.
  - Fmax (post-route): HD Analog 87.00, HD HDMI 89.45, HD Dual 92.82, SD Analog 91.40, SD HDMI 87.75, SD Dual
    89.06 MHz.
  - 6865-6881 / 7680 LC (89.6 %), 21 / 32 EBR. Every bitstream passed iceunpack; the packed config carries the new
    labels. Output: `out/rev_b/ron/heirloom/batwing.vmprog`.
  - Fit history: the first cut (shift-register output filters, 24-clock latency with pad stages, a 9/16 shade
    multiply, 15-bit L1) routed HD Analog at 85.3 MHz but filled 7594 LC (98.9 %) and stalled router1 on seed 1; it was
    killed and restructured (notes.md) before this build.
  - GHDL (720p60, decimation 1, real timing ID): the black-input Video frame matches the integer model at field 3
    within 2.1 mean RGB (after the output filters' 12-px delay); the Osc and selfie frames are in `cmp_final.png`.
    Motion: a second capture 9 fields later (12 warm-up frames) is shifted 16 px down at 720p, exactly 9 x 2.67
    virtual lines, and matches the integer model at field 12 within 2.0 mean RGB (`cmp_motion_rtl.png`).
  - Presets are hand-written placeholders; capture Ron's looks with `./capture_preset.sh`.
  - If a later version gets tight or router1 stalls: the stalls were on SD configs / the overfull first cut only;
    dupin shows 0 `CARRY I0==I1`. Try another seed before restructuring.

## Deviations
- The photo's band sequence is reproduced on the centre column (top and bottom paths within a few px); elsewhere the
  fitted field places the bands within roughly a band width, so the exact yellow / green order in the wing corners
  differs. The photo's three tiles are slightly unequal (493 / 476 / 468 px); the program's are equal.
- The bowtie interior: the photo's mottled camera texture and the mirrored "A"-shaped islands are camera content. With
  no input the program draws a flat lilac with a faint built-in diagonal ripple; with a camera, the camera supplies the
  texture and frays the edges as in the photo. The fine scan lines inside the lilac and the small dark teeth along the
  vertical band edges are not drawn.
- The bowtie's top and bottom edges are clean (only the side edges tear per 8-line strip): a torn horizontal edge
  would draw single-row slivers.
- In the clip the bowtie lingers slightly near mid-frame as the oscillator rolls (a static component in the vertical
  mix); the program rolls uniformly at the clip's average 160 lines/s.
- The palette's narrowest runs are widened to >= 3 % of the span (no band thins to a single row); band edges are
  softened over ~2 px.
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3 (1440 virtual px): 1.5 tiles instead
  of 2.02, with the bowtie and the seams at the same size. The roll is per field, so at 50 Hz it runs at 133 lines/s.
