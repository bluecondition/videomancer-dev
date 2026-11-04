# Grommets — two black keyhole holes drifting down a purple scanline lattice, with fanning stems

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom from the video clips, from the still
`photos/vid/WIN_20230216_01_27_34_t2p5.png` (clip WIN_20230216_01_27_34 at 2.5 s, 11.5 s long). The still
shows two big black keyhole holes over a purple / blue / magenta lattice of contours built from 9.5-line colour
bands, a bright cyan X of contours crossing between the holes, a reddish "eye" of rings at the bottom centre,
thin black stems leaving the holes' bottom tips and fanning out going down, and a dark green rim on the left
of every black edge, dark red on the right. In the clip the holes scroll down at 224 px/s while the stems and
the bands stay put. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_grommets/` (`notes.md` has every number).
- **Holes.** Left hole centred at x 537.5, right at 1474.5 (mirror axis x 1006); 545 px wide at the widest row
  (y 297–325), rows 70–581 (511 lines), round sides, pointed top and bottom (superellipse exponent ~1.5).
- **Lattice.** The hole outline is a level set of F = 0.805·tri(x) + cos(y): an H triangle of period 936 px with
  its trough at the centre x 1006 (peaks at 538 / 1474, troughs at 70 / 1942) plus a V cosine of period 1152
  lines with its maximum at y 319.5. Key level T = 0.860 (black where F > T). Fit to the outline: 5.3 px rms
  (`fit5.py`; a cosine H wave gives 16 px, so the H wave is a triangle). The saddles of F are the cyan X, its
  minimum (1006, 898) the eye, so the whole composition is this one field.
- **Stems.** A ridge x′ = |x − 1006| = 381.3 + 0.04223·y + 2.147e-4·y² (quadratic keystone, 1.9 px rms,
  `stemfit.py`), the same in every frame of the clip: a bump 2.10·max(0, 1 − |x′ − xs(y)|/15 px) added to F
  (stem widths 5.6 px rms, `stemw.py`). It joins the hole's upper shoulder from above and leaves the bottom tip.
- **Bands.** Horizontal colour bands of 9.49 capture lines (FFT and a dark-row fit; phase 4.0); the capture is
  line-doubled (rows 2k / 2k+1 match: 16 vs 34 mean row difference), so the band is a 4.745-line-pair sawtooth.
  Inside a band the colour sweeps dark → indigo → blue → violet → magenta → red; the sweep does NOT shift with
  the field (`dom.py`), so the ramp is not summed into the colouriser: colour = TABLE[zone(F)][phase].
- **Rim.** Scanning the rim colour around the hole by angle (`rim.py`): pure dark green (7, 56, 15) on the whole
  left half, pure dark red (41, 2, 1) on the right half, split exactly at the top and bottom; the stems show the
  same polarity (deep blue left, mauve right). That is the sign of dF/dx: a scan-order transient in the
  colouriser input. The chroma lag is only 1.5–4 px, so it is not a capture artefact.
- **Palette.** 12-colour k-means: black 32 %, blue-violet (77, 25, 237), violet (159, 28, 243), maroon
  (77, 19, 47), indigo (50, 21, 134), magenta (146, 15, 146), cyan (61, 187, 236), dark green (30, 73, 47),
  pale blue (162, 187, 239), mauve (126, 90, 108), red (177, 20, 49), green (71, 143, 90). Non-black luma
  mean 87, std 46; saturation mean 66.
- **Serration.** The hole edge moves 22–24 px across each band (widest just after the dark row): the key is
  taken from the band's colour, which the 2-D table reproduces.

## Motion in the clip
- The widest hole row sits at 183 / 297 / 410 / 516 / 630 in the frames 0.5 s apart around the still: the
  V wave scrolls DOWN at 224 px/s (3.73 lines per field), and the t = 0.5 s frame (widest row 999) gives the
  vertical period, 1152 lines (one hole row every 5.1 s). The holes' x, the stems and the bands do not move
  (the stem x per screen row is the same in every frame; the band phase is 3.8–4.5 in every frame).
- The other candidate stills of this clip (0.5, 4.0, 6.0, 8.0, 9.5 s) are phases of the same scroll, so Ron's
  hands were off the knobs during this clip. The clips 01_23_08 (4 min earlier) and 01_16_43 (11 min earlier)
  show the same patch with different colouriser settings: 01_16_43 is a rainbow ring colouriser with bigger black
  regions, which is the Rainbow palette here.
- The program scrolls the V wave at the measured 224 px/s (K4 at 764) with the period, the stems and the bands
  static; a GHDL capture at full Speed is bit-exact with the model's scroll of 14.93 virtual lines per field.

## What it does
1. **Field.** F = GH·tri(x) + GV·cos(y − vt) + stem bump + Etch·luma, in zone units (256 per zone; F = 1 is
   9362 units at Lattice = 1).
   - The H triangle is a beam DDA in half-pixel units, exact, reloaded to ∓GH at every fold (trough / peak,
     every half period).
   - The V term is per line: a 512-entry half-cosine ROM (or a triangle for Diamond) times GV on a serial
     multiplier.
   - The stem bump is GS − min(|x − XL|, |x − XR|, 30 h)·SS; XL / XR come from a second-order DDA per line.
   - The camera (or the Osc) adds ((luma >> 2) − 16)·EG.
2. **Zone and phase.** zone = clamp(F + Holes, 0..32767) >> 8 (128 zones, 95+ black), inverted by Negative;
   phase = the band accumulator (13813 per line pair, 41367 at line 0) >> 13, 8 steps.
3. **Colour.** The 2048 × 32 table (palette, zone, phase) gives Y, U (= Cr), V (= Cb) and a rim term R;
   U is moved by −R / +R according to the sign of dF/dx (the beam direction plus the stem's slope), so black
   edges get the green / red rim.
4. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the capture's softness and the HD HDMI
   bar fix), luma 13/16, then the blanking gate. Palette luma is pre-scaled 16/13 and capped at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Lattice | the colouriser gain over the whole field, ×0.25 .. ×4 (512 = the still): flat purple with hairline stems .. black diamonds with thin contour lines |
| K2 Columns | the hole spacing (the H period): 1 .. 4 holes across (512 = the still's 2.05); the stems scale with it |
| K3 Rows | the hole row spacing (the V period): 2304 .. 576 lines (512 = the still's 1152: one row on screen .. two rows) |
| K4 Speed | the scroll of the holes: bipolar, ±8 deadband, square law; 764 = the clip's 224 px/s down, 1023 = 896 px/s, left = up |
| K5 Stems | the stems' strength 0 .. 4× (square law; 512 = the still): none .. thick stems merging into the holes |
| K6 Etch | how far the camera (or the Osc) bends the lattice: 0 .. 2 F per full luma (square law; 512 = 0.5 F) |
| S7 Bands | Fine (9.49-line bands, the still) / Bold (19-line bands) |
| S8 Palette | Violet (the still's table) / Rainbow (three hue cycles of rings, the 01_16_43 clip's colouriser) |
| S9 Shape | Round (cosine rows, the still) / Diamond (triangle rows: diamond holes, straight contours, a diamond eye) |
| S10 Negative | Off / On: the zone index inverted, black at the lattice minima (the eyes), the holes coloured |
| S11 Source | Video / Osc |
| P12 Holes | the key level (the holes' size): 0 % = no black at all, a warm core where each hole was; 40 % = the still; 100 % = black floods everything but the coloured eyes at the minima |

## Video
- **Video mode (the default).** Camera luma ((Y >> 2) − 16 from a one-line buffer) is added to the field, so
  the picture bends the contours, frays the hole edges along its bright parts and pushes the key: at the default
  Etch (0.5 F per full luma) the composition still reads; at full Etch the picture replaces the lattice. With no
  input, Video mode is the still exactly.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice drifting about 75 px/s, at half amplitude, feeds the same
  path: the holes breathe and the contours wobble gently at the default Etch, and Etch scales it.

## Engine
- **Per-field sequencer (vblank zone).** One serial shift-add multiplier, 25 slots (24 clocks each, slot number
  decoded two clocks ahead, operands pre-muxed): the gain GV = 9362·2^((K1−512)/256) and GH = 0.805 GV, the
  half period HP = 936·2^((512−K2)/512) h and the slope SLP8 = 1024 GH / Px (Q8 per h), the first fold and the
  beam start value (a nine-step subtract loop finds the fold at or right of the beam origin), M = 2^24 / Py,
  INCV = M·ys / 256, the speed (Q8 lines per field) and the scroll step DV, the phase at line 0, the stem base
  L0 = 762.6·2^e h and its DDA steps, GS and SS = GS / 30, EG and the Holes offset. A shared glide (1/8 of the gap
  per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12. The exp2 ROM is read four times per field.
- **Per-line engine.** Kicked at each line start (and at zone clock 700 for line 0), computes the NEXT line in
  ~20 clocks: the V phase (+INCV), the cosine ROM address (half wave, mirrored) or the triangle, |W|·GV on a
  16-step serial multiplier, the stem x′ DDA (xs += d, d += dd), the line pairs crossed (0..3 per line) and the
  band phase.
- **Beam.** P = the H triangle times the gain in Q8 h units: ±SLPX per pixel, reloaded at the fold to ∓GH ± SLP8·r
  (r = 1..4 h, four pre-multiplied values), the next fold = fold + HP; the fold event is registered a clock
  ahead. The beam starts 8 px early (the output boxes' group delay), as lozenge does.
- **Pixel path (17 stages).** 2: H + V, distances to the two stems. 3: + Holes, |d| clamped, inside / rising
  flags, luma. 4: nearest stem distance, the stem slope term, etch nibble products. 5: distance × SS, etch,
  the x slope (±SLP8 + the stem's ±SS·256). 6: bump = max(GS − product, 0), + etch, slope sign. 7: + bump.
  8: the table (EBR; the address is the clamped sum's zone, inverted for Negative, with the palette and phase
  bits). 9, 10: fabric copies. 11: compose (U ∓ R, black outside the generated avid). 12–15: the two boxes.
  16: luma 13/16. 17: the gate. C_LAT = 16.
- **ROMs.** The colour table 2048 × 32 (16 EBR), the line buffer 2048 × 8 (4), the half cosine 512 × 12 (2),
  exp2 256 × 16 (1): 23 EBR.
- **Contracts.** The generated avid (parity-locked start); one latency, C_LAT 16, in every mode; virtual
  1080-line coordinates (SD shows the centre 1440 virtual px: both holes stay on screen); the running-sum
  output boxes with the per-line resync; the blanking gate; colours in the hardware convention (U = Cr).
- **Verified (GHDL, 720p60, decimation 1).** The default Video frame with a black input is 99.99 % bit-exact
  with `imodel.py` (mean |diff| 0.00 in 10-bit Y/U/V, at the scroll of three vblank zones; the tester captures
  4 px earlier than the program's avid); a capture at full Speed is 100 % bit-exact at DV = 14.93 lines per
  field; Osc and Video (selfie.jpg) frames rendered (`cmp_final.png`). A knob sweep of every per-field value
  over the full travel of K1–K6 and P12 at every raster stays inside its RTL width.

## Status
- **v0.1.0 built 2026-11-04 with ROUTER2** (`MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh grommets`,
  i.e. `./build_heirloom.sh grommets`). **Build seeds: HD HDMI = seed 3** (seeds 1–2 missed), **HD Dual =
  seed 2** (seed 1 missed); the other four are seed 1. Post-route Fmax: HD Analog 80.03, HD HDMI 79.21,
  HD Dual 76.51, SD Analog 75.23, SD HDMI 65.71, SD Dual 83.51 MHz (SD Fmin 27). 6650–6675 / 7680 LC (87 %),
  23 / 32 EBR. Every bitstream built; `out/rev_b/ron/heirloom/grommets.vmprog` (474043 bytes). **Not flashed.**
- To rebuild from source: `MAKE_TIMEOUT=900 ./build_heirloom.sh grommets` (router2; if HD HDMI misses on
  seeds 1–2 again the builder's retry reaches seed 3).
- Presets are hand-written placeholders; capture Ron's looks with `./capture_preset.sh`.
- No pin-skew seed screening was run: if HD HDMI shows lines on hardware, re-seed that config.

## Deviations from the photo
- The still's analog shimmer (the 4.745-line sawtooth sampled per line, plus noise) is reproduced only as the
  line-pair beat of the band phase; the per-line random variation is not modelled (a jitter was tried and
  rejected: it ragged the hole edges).
- The eye at the bottom centre is a muddy magenta / maroon blob of bands; the still has fine red / blue / green
  rings there.
- The cyan X is cyan / lavender bands; the still's is whiter and hotter.
- The rim term moves Cr only (the still's rim also differs slightly in Cb and luma, mean 4 levels).
- Zone quantisation (0.027 F = 8 px at the hole edge) shows as small steps in the rim; the table's dominant
  colour per cell is a cleaner version of the still's noisy sweeps.
- The hole outline is a triangle / cosine level set (5.3 px rms); the still's hole is a touch wider in its upper
  half, where the stem joins.
- The left 8 px of each line fade in from black (the output boxes' warm-up); the picture is drawn 8 px early so
  the geometry lands in place.
- The oscillator runs at half amplitude: at full it grew the holes about 20 % at the default Etch.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 1440 virtual px (both holes stay on
  screen, the outer eyes are off), the bands become 4.2 lines (what the original SD patch had), and the stems'
  keystone is in virtual lines, so it looks the same.
