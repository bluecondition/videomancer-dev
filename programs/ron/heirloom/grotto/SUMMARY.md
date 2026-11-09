# Grotto — a blue cavern face: a keyed black eye and slit inside white bands with green edges, breathing

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20240104_00_30_04_t3p0.png` (clip WIN_20240104_00_30_04, frame at 3.0 s; the same patch at 0.5, 5.0
and 10.5 s, and in clip WIN_20240104_00_15_02, where the colouriser is turned to gold). A hard-keyed black eye with a
pale rim sits right of centre, a black slit is keyed above it to the right, soft black lenses and a basin fill the
corners, and white bands with green edges fold around everything, crossing in an X over the eye. Across the clip the
eye grows and sinks while the slit closes, then reverses (~10 s). Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_grotto/` (`notes.md`; `level.py` = colour -> palette position; `geod.py`,
`label.py`, `refine3.py`, `aniso3.py`, `smoothkey.py`, `kfill.py` = the field fit; `proto.py` = the float proto;
`imodel.py` = the integer model of the RTL; `gen.py` writes `grotto.py`).
- **One field through a folding colouriser.** Every colour lies on one 1-D palette: black -> navy (0,2,30) -> blue
  (0,0,155) -> (3,57,215) -> cornflower (39,92,242) -> green (0,249,24, a narrow band) -> teal -> light blue
  (129,202,240) -> pale (186-215, 234-243, 238-240). Profiles show sharp V crests at pale and sharp V creases at navy
  (B 99-102, never black): a triangle fold of a smooth field S, not creases of S itself. The soft black lenses and
  basin are S below the first branch (they fade through navy, no hard edge).
- **The key.** The eye and the slit are filled with (0,0,22-28), not the soft regions' (0,0,0), with a hard edge and
  a pale ring: a key where S passes a level on the third rising branch at palette position 0.71, before the crest
  (fit error 0.0058 -> 0.0041 when the key level moved off the crest).
  Eye: x 952-1392, y 412-776, 107k px, centroid (1148, 588). Slit: x 1268-1880, y 216-372, 41.5k px.
- **Field fit.** S was unwrapped from the folded palette position by a geodesic distance from the black regions
  (cost |dL|), per-pixel branch labels, then a bilinear knot table fitted by gradient descent on the fold model (data
  term + thin-plate smoothness, key-interior ramp, black hinge), multi-scale, on a 32 px x 17.1-line grid (the fine
  vertical step keeps the top band's chevrons, tips of nested lens contours on a horizontal axis at y ~165).
  Smoothing is boosted within 320 px of the keys and the key interiors are filled biharmonically, so the eye grows and
  shrinks cleanly. Integer model vs photo at defaults (720p): mean |dRGB| 14.9; 4x4 region means max 15.4, mean 3.9;
  key fill 9.0 % (photo 9.7 %), black 8.8 % (8.6 %), green 13.6 % (12.0 %).
- **Seam.** A vertical dark line at x ~365 with the strip left of it jogged a few px (an H-ramp restart); it drifts
  left as the eye grows (365 -> 280 -> 250 px over the clip).
- **Palettes.** Green band at p 0.435 (half-width ~0.03 in the still). The gold clip: black -> maroon (30,2,0) ->
  olive-brown (80,62,0) -> amber (225-250, 160-198, 0) -> pale pink cores (245,205,227), an olive-green (105,150,0)
  band where the green is.

## Motion in the clip
- Eye area per 0.5 s (key-colour mask): 84k (2.5 s), 107k (3.0 s, the still), 128k, 144k, 153k, 157k (5.0 s); the
  centroid sinks 557 -> 588 -> 619 -> 649 -> 683 -> 714 (~63 px/s). The slit shrinks 77k -> 41.5k -> 3k -> 0. At
  0.5 s the slit is a 155k-px black wing and the eye a 12k-px almond with almost no green; at 10.5 s the slit is back:
  an oscillation of ~10 s, the eye largest near 5.5 s. The green band widens over the clip. The top band stays put.
- **The program:** a breathing LFO (sine, 10 s at the default Speed, starting at the still's rising zero) drives the
  eye bump's amplitude (+65 / -140 w at the peaks), its centre (+-140 lines: it sinks as it grows), the slit bump
  (-128 / +302 w: it closes as the eye opens) and the seam (+-60 px). `cmp_motion.png`: the model at +0.5 / +1.0 /
  +1.5 s next to the clip's m-frames (the eye grows, rounds and sinks; the slit is gone by +1.0 s).

## What it does
1. **Field.** S0 = the measured table, bilinear: the line engine builds the next line's 64 knot values (bilinear in
   y) into a ping-pong block RAM; the pixel path interpolates in x.
2. **Bumps.** S = S0 + eye bump + slit bump + Etch x luma. Each bump is separable: an H profile times a V profile
   ((1 - (d/a)^2)^2, a = 700 / 760 px across, 480 / 330 lines down; an analog H x V VCA shape) times its amplitude.
3. **Key.** S >= 470 (w units, w = 145.5 S + 110.5) fills the eye and the slit (Hole), or keeps folding inside them
   (Iris).
4. **Colour.** w2 = S + Tide, folded Rings times faster above the first crest; p = a black ramp below the first crest
   (w 110 .. 256), then a triangle between navy (p 31) and pale (p 255) every 256 w; palette(p); a green band where
   |p - 111| < Green (half-weight edge 3 steps wide).
5. **Seam** (3/4 toward black, 6 px), **output conditioning**, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Rings | how often the bands fold above the first crest: x0.33 (0: a broad pale field, only the eye and the black basins) .. x1 (512, the photo) .. x3.0 (1023: dense rings packed around the eye and through the bands), exponential |
| K2 Tide | the field's level, bipolar (+-8 deadband), +-1.27 S: left = black floods in (lenses and basin swell, bands sink toward the eye), right = black drains away (pale wash, blue rings). The eye and slit keep their shape |
| K3 Green | the green bands' width: 0 = none (white bands edged blue, the clip's 0.5 s look) .. 512 = the photo .. 1023 = broad green (the clip's 5 s look) |
| K4 Speed | the breathing LFO: 0-7 held (frozen at its phase), 512 = 10 s per cycle (the clip), 1023 = 1.25 s, exponential (+-3 octaves) |
| K5 Slit | the slit's level, bipolar (+-8 deadband): left closes it (a pale wash at top right), right opens it into a big black wing (the clip's 0.5 s look) |
| K6 Etch | how far the camera (or the Osc) bends the field: 0 .. 1.6 S per full luma, square law; 448 = 0.30 S per full luma |
| S7 Twin | Off / On: the field mirrored about the centre: two eyes face to face, the slit on both sides |
| S8 Iris | Hole (the photo's black eye) / Rings: the fold carries on inside the eye and the slit (concentric pale / blue rings) |
| S9 Palette | Blue (the photo) / Gold (the gold clip: maroon, amber, pale pink, olive-green band) |
| S10 Wave | Off / On: the field's x lookup swims +-150 px in a sine 420 lines tall, drifting (H phase modulated at V rate); reflected at the picture edges |
| S11 Source | Video / Osc |
| P12 Eye | the eye's amplitude (headline): 0 % = shut (the hole closes to a pale lid with a small bruise, the slit opens wide); 50 % = the photo; 100 % = wide open (a round eye ~2.3x the area, sunk 150 lines, rings crowding round it, the slit closed). The LFO breathes around it |

Glides: K1, K2, K3, K5, K6 and P12 move 1/8 of the gap per field.

## Video
- **Video mode (default).** Camera luma (Y - 64) x 0.28, an 8-pixel IIR low-pass, minus a 32-step dead zone (dark
  parts of the picture leave the field alone), times Etch, is added to the field before the key: the bands bend around
  bright objects and the eye and the slit reach toward them. At the default Etch the composition still reads on a busy
  card (`cmp_final.png`, selfie.jpg); bright camera content lifts the field, so the black basins get lighter. With no
  camera (black) Video mode is exactly the photo. The camera never reaches the output directly.
- **Osc mode.** A diamond lattice (two triangle oscillators, 640 px across x 540 lines down, bipolar, x2) drifting at
  0.06 / 0.04 cycles/s feeds the same path: at the default Etch the eye and the bands warp both ways. Etch stays live.

## Engine
- **Per-field sequencer (vblank zone = no avid for 1024 clocks).** Glides; the LFO phase, wave and Osc drifts; one
  serial shift-add multiplier, 15 slots (slot decoded two clocks ahead, operands pre-muxed; sign-magnitude):
  the eye amplitude, centre and slit amplitude from P12 / K5 and the LFO, Tide, the Rings exponent -> exp2 ROM ->
  fold frequency (Q7), the Speed exponent -> exp2 -> LFO step, Etch^2 x 233, the green width, the seam drift.
- **Per-line engine** (kicked at each line start, computes the NEXT line in ~195 clocks; fits SD's 858): its own
  serial multiplier: table row and fraction (kicks the slice builder), the two bump V profiles and amplitudes
  ((1 - (dy/b)^2)^2 x A >> 6), the wave shift (sine ROM).
- **Slice builder**: 128 table reads (rows j, j+1) -> (b - a) fy (two nibble partial products) -> {s(k), s(k+1)}
  pairs into the bank the pixel path is not reading (2 x 64 x 20, `no_rw_check`).
- **Pixel path (29 stages = C_LAT + 1):** wave / seam flags (2), Twin + seam jog (3), edge reflection (4), slice
  address and bump distances (5), slice RAM and bump ROM addresses (6), fabric copy and bump ROMs (7), difference and
  bump fabric copies (8), x interpolation and bump / etch nibble products (9), combine (10), sums (11, 12), key, Tide,
  clamp (13), Rings partial products (14), combine (15), w2 (16), fold parts (17), p (18), palette address and green
  distance (19), palette ROM (20), fabric copy (21), compose (22), seam and black outside the generated avid (23), two
  [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V with the running-sum resync (24-27; the 8-back samples from two
  256 x 32 block-RAM rings), luma 13/16 (28), blanking gate (29).
- **Memories (23 EBR):** field table 4096 x 10 (10), palette 512 x 24 (3), bump profiles 2 x 256 x 8 (2), sine
  256 x 8 (1), exp2 256 x 16 (1), slice banks 128 x 20 (2), output box rings 2 x 256 x 32 (4). Standalone yosys:
  0 `emulate_transparency`.
- **Contracts:** generated avid (parity-locked); one latency (C_LAT 28) in every mode; virtual 1080-line coordinates
  (SD shows the centre 4:3); the HD HDMI output conditioning (Y, U, V boxes, luma 13/16, whites ~777) before the
  gate; the running-sum resync; colours in the hardware convention (U = Cr).
- **Verified (GHDL 720p60, decimation 1, real timing ID):** default Video with a black input, default Osc, Wave, and
  Twin + Iris + Gold + Wave frames are **bit-exact with `imodel.py`** on every pixel right of the boxes' 12-px left
  warm-up (the RTL sits 12 px right of the model: 4 px shell offset + 8 px from the causal boxes). A motion frame
  (Speed 1023, Eye 800 glided, frame 6) is bit-exact with the model at 5 fields (LFO, glide, bumps, seam drift). The
  camera frame (selfie.jpg) was judged by eye. Bugs the sims caught and fixed: an 11-bit overflow in the seam
  darkening, a 13-bit overflow of the Wave lookup at the right edge and under Twin, and a narrowing signed resize in
  the edge reflection (now a slice).

## Status
- **v0.1.0 built 2026-11-09, NOT flashed.** `out/rev_b/ron/heirloom/grotto.vmprog`.
- Router2 (builder default). Seeds: **HD Dual = seed 2** (seed 1 missed); the other five on seed 1. Post-route Fmax:
  HD Analog 86.58, HD HDMI 85.03, HD Dual 90.14, SD Analog 90.38, SD HDMI 93.27, SD Dual 87.88 MHz.
  6,870-6,884 / 7,680 LC (90 %), 23 / 32 EBR. iceunpack OK x6; the packed config carries the new labels and presets.
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh grotto` (or `./build_heirloom.sh grotto`).
- Fit history: the first build (FF delay lines in the output boxes) was 7,288-7,314 LC (95 %); the boxes' 8-back
  samples moved to two block-RAM rings (-412 FF, +4 EBR).
- No pin-skew seed screening was run; if HD HDMI shows lines, re-seed that config.

## Deviations from the photo
- The field is a fitted 32 px x 17-line knot table: the top band's chevrons are slightly rounded, with a small notch
  at the left ">" (x ~740, y 120-180) and a tick at the top-right corner left by the fit; the eye's right side is
  rounder than the photo's shield shape; a thin bright line shows on the bottom row.
- The photo's thin green key-rim line is not drawn (the key edge is a pale -> fill step, softened by the output
  boxes).
- Pale crests come out greyer (~777 output luma, the shell's white cap) than the photo's (215,243,238).
- The clip's +1.5 s white wash on the left and the faint horizontal streaks by the seam are not modelled; the seam is
  a fixed 6 px darkening with a 10 px jog that drifts with the LFO.
- The table fixes the composition: element changes come from the bumps, Tide, Rings, Wave, Twin and Etch, not from
  re-patching the oscillators that made the photo (which could not be identified).
- **Resolutions:** 720p shows the same picture as 1080p. SD shows the centre 4:3 (virtual x 240-1680), so the slit's
  right end and the left lens are cut, and it is softer (the boxes are per output pixel).
