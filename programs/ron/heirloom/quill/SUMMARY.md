# Quill — blue feather quills hanging from a sliding spine, dotted cyan barbs, a green far field

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom phase 3 (video clips), from the clip `WIN_20230910_00_11_50`
at 7.0 s (`photos/vid/WIN_20230910_00_11_50_t7p0.png`; 9.0 s clip, the same patch in `00_11_34` and `00_12_21`).
A full-blue spine band at the top crowded with wavy cyan / magenta stripes; below it eight blue quills hang between
round-topped black tongues, bow a little to the right, then sweep down and to the left and converge on the left
edge. Dotted cyan barbs with magenta fringes run down every quill; far from the spine the blue is gone and the barbs
carry on as dense green / red lines. Every colour is a saturated binary RGB: three hard keys. In the clip the spine
slides down while the key level drops, so the quills retract into the band. Presets are hand-written.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_quill/` (`notes.md` has every number).
- **Colours.** Black 48 % of the pixels, blue (0, 0, 246) 26 %, cyan (0, 238, 249) 11 %, green (0, 250, 0) 6.8 %,
  magenta (243, 0, 255) 4.5 %, red (231, 0, 0) 1.8 %, white 1.2 %, yellow 0.6 %. Thresholding R, G and B at 128
  separates them cleanly: B is the quill body, G the barbs, R a fringe beside G.
- **Field.** One phase field explains every quill: phi = (x + 130) k(y). Seven tongue centrelines tracked over
  y 330–690 meet a common sync point 130 px left of the picture with rms 2.6 px (`fit2.py`): a VCO hard-synced at
  hsync (about the HD back porch).
- **Frequency.** The macro period (three plumes) by phase dispersion with the sync point fixed (`fit3.py`): 690 px
  at y 370, 668 (480), 626 (560), 570 (640), 508 (720), 448 (800), 389 (880), 335 (960), 276 (1040). Below the hook
  it is P = 715 · 2^(−2.29e-6 (y − 265)²), rms 1.7 % in log: exponential FM by a parabola. Above y 265 the period
  falls faster (690 at 300, 601 at 100, 576 at 60): a cusp at the spine (y ≈ 45).
- **Quills and tongues.** 8 tongues across at y 370 (centres 133, 351, 602, 823, 1055, 1298, 1513, 1740), tops at
  y 256–300 on the left and ~300–340 on the right (0.04 line per px). By macro phase the blue ends at y ~630
  (phase 0.11–0.19), ~690 (0.50–0.61) and ~740 (0.75–0.97, a double plume); the tongues sit at 0.00–0.09,
  0.25–0.47 (wide) and 0.64–0.72 (`hb.py`). That reach profile is the body ROM.
- **Barbs.** Dotted lines at phases 0.16, 0.54 / 0.58 and 0.80 / 0.86 / 0.91 (`glines.py`). Dot spacing along a
  line is 16–24 rows at x 300–1000 and 8–10 rows at x 1000–1800 (spacing ∝ 1/x, `dots.py`). R sits on the
  low-phase side (8 px left / 2 up in the quills, 6 lines above in the far field).
- **Band and far field.** The band (y 0–180) is G 43–59 %, R 23–31 %; the far field (y > 900) G 30–57 %, R 10–18 %,
  a little blue. Between them the G share drops to 3–5 %.
- **Texture.** No tear: 78 % of the rows have zero offset to the row above, the rest +1..+2 px (the slant). The key
  edges are slightly ragged.

## Motion in the clip
All 526 decoded frames (`fr/`, `motion.py`, `drift.py`, `spine.py`).
- **Ron plays two things.** The key level: the black share of the lower half goes 8 % (5.5 s) → 67 % (7.0 s) →
  99 % (8.75 s), as the quills retract and the far-field barbs thin out. And the spine position: the full-blue
  band's centre goes 134 (7.0 s) → 197 (8.0) → 313 (8.5) → 374 (8.75), about 140 lines/s down, while the band
  narrows from 256 to 144 lines. The m-frames show the spine at y ≈ 210 / 290 / 380 at +0.5 / +1.0 / +1.5 s, with
  quills on both sides of it, shorter each time.
- **Nothing free-runs.** Frame to frame (1/27 s) the picture moves 0–3 px and correlates 0.8–0.97. The tongue x
  positions on row 400 scale about the sync point by +6.4 % over 0.73 s and then stop: that is k(y) changing as the
  spine moves, not a phase scroll.
- **The program.** A sine LFO sweeps the spine (±380 lines about y 350, a ~10.2 s cycle at the default Sweep) and
  the same CV lowers the key level as the spine descends. It is anchored on the still at power-up (spine y 45,
  moving down at 140 lines/s). At +0.5 / +1.0 / +1.5 s the spine is at y 128 / 231 / 346 and the quills retract
  into a narrowing band, as in the m-frames (`cmp_motion.png`, top = the clip, bottom = the model).

## What it does
1. **VCO.** Per line, E = g · (2.29e-6 (a − 220)² + 0.0011 max(0, 190 − a)) octaves with a = |y − spine| (Spines:
   3 × the distance to the nearest of a row of spines 360 lines apart), k = k0 · 2^E (clamped −3 .. +5 octaves).
   A 28-bit DDA steps phi = (x′ + 130) k across the line (Mirror: x′ = 960 + |x − 960|).
2. **Body key (B).** Blue where a·gb + tilt + noise < HB[frac phi] (256-entry reach profile, the still's), with the
   tilt 0.04 (x − 700) lines and ±6 lines of 16-px noise. Invert flips it.
3. **Barbs (G, R).** |e| = the distance to the nearest of six line phases (ROM). Inside the window
   (0.015 period × Barbs, ×(0.75 + noise)), G where frac(psi) < duty and R in the slot just above, with
   psi = 0.042 d (n + line phase): horizontal dashes whose rows fan like the field. The duty fades with the band
   distance to a 0.25 floor.
4. **Band and far field.** A texture key on frac(d/44 + 0.45 · wiggle(phi) + 0.45 · noise): G below the level,
   R above 255 − 0.45 level, a blue slot in the far field. The level is 2 × (band ramp) near the spine, 3.25 ×
   (far ramp) from a·gf > 780, gated to widened barb windows in the far field.
5. **Colour.** The three key bits pick one of 8 colours (Ice = the still's, Ember), luma pre-scaled 16/13.
6. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (block-RAM taps, running-sum resync), luma
   13/16, the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Quills | the number of quills across: ×1/3 .. ×3 (exponential), 512 = the still's 8 tongues; 0 = two huge plumes, 1023 = a dense fan |
| K2 Curl | the FM depth (how hard the quills bend and converge): 0 = splayed out to the right, 256 = straight hanging quills, 512 = the still, 1023 = ×3, a tight fan into the left corner |
| K3 Spine | the spine's position (the centre of its sweep): 0 = above the frame (only the converging far field), 512 = the still, 1023 = low, quills growing up and down from it |
| K4 Sweep | the spine LFO: bipolar, still in the centre ±8, square law; 700 = the clip (~10 s cycle), full = ~1.3 s; the key level rides the same LFO |
| K5 Barbs | barb window and dot duty: 0 = faint sparse dots, 512 = the still, 1023 = fat solid cyan barbs |
| K6 Warp | how far the camera (or the Osc) bends the quills (0 .. 1.6 periods at white, square law) and grows the blue body where it is bright; 320 = the default |
| S7 Mirror | Off / On: the sync point mirrored, quills facing a central seam (a heart-shaped fold) |
| S8 Spines | One / Three: the spine distance folded into three stacked feather rows |
| S9 Palette | Ice (the still: blue, cyan, magenta, green, red) / Ember (red-orange body, yellow barbs, cyan far field) |
| S10 Invert | Off / On: the body key inverted, black quills on a blue ground |
| S11 Source | Video / Osc |
| P12 Plumage | the key level: 0 % = a thin spine band with short hooks and sparse dotted green quill lines (the clip's 8.75 s); 512 = the still; 100 % = a blue flood laced with dense cyan barbs along every quill |

**Presets:** Still; Held Still (Sweep off); Flood (P12 900); Seam (Mirror, Spine 700); Three Rows (Spines, slower
sweep); Ember Fan (Ember, Curl 820); Splayed (Curl 60, Quills 380); Fast Sweep.

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, is added to the VCO phase
  (luma × Warp gain) and lowers the body threshold (1.125 lines per 1/256 period of bend). The quills fray and bend
  around the picture and bright areas grow blue; at the default Warp the composition still reads (`cmp_final.png`,
  last panel). With no input, Video mode is the still.
- **Osc mode.** The shell's 640 × 540 virtual-px triangle lattice, drifting, feeds the same path at four times the
  depth: at the default Warp the quills fold into angular zigzags and the body floods the lattice cells. Warp stays
  live in both modes.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier, 10 slots (slot number → registered
  one-hot → flat AND-OR operand fetch): 380 sin and 435 sin (spine swing, level swing), the spine centre, speed²
  and the LFO rate, warp² and the warp gain, k0, the dot duty, the barb width. A 256 × 16 exp2 ROM serves six
  exponents (k0, body / band / far gains through one shared shifter, duty, width); a 1024 × 16 sine ROM the LFO.
  Shared glide (1/8 per field) on K1, K2, K3, K5, K6, P12.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in 365 clocks (fits SD's 858) on a
  second serial multiplier with 17 slots, the operands fetched the same one-hot way: Spines' d mod 360 (ten
  subtract clocks), a, (a − 220)², × 38, the cusp, × |g|, exp2, k, the step, phi0 = k (x′ + 130), a·gb, a·gt, a·gf,
  the band / far / dot ramps (shift-adds), duty × fade, the R and texture thresholds, cd = 43 d, the six dot phases,
  the texture phase and the noise seed. Next-line values are staged and loaded at the line start.
- **Pixel path (15 stages, C_LAT 20).** 5: phase DDA (add / subtract for Mirror), 16-bit LFSR noise stepped every
  16 virtual px, the cell index (tilt). 4–6: luma × warp (nibble products), the body shift, noise − tilt. 7: phase
  ROM address. 8: n · cd, body level, barb and duty thresholds, texture phase. 9: ROM fabric copy. 10: body compare,
  |e|, dot phase (six-way mux), texture phase. 11: compares. 12: keys. 13: palette. 14: carry. 15: compose. Boxes
  16–19, luma 20, gate 21.
- **ROMs / RAMs (16 EBR):** phase ROM 256 × 32 (2), exp2 × 2 (2), sine 1024 × 16 (4), line buffer 2048 × 8 (4), two
  box rings 256 × 32 (4, `no_rw_check`).
- **Contracts:** the generated avid (parity-locked); one latency (C_LAT 20) in every mode; virtual 1080-line
  coordinates (SD shows the centre 4:3); colours in the hardware convention (U = Cr) from `quill.py`, luma pre-scaled
  16/13, capped 940; output conditioning with the running-sum resync before the blanking gate.
- **Verified (GHDL 720p60, decimation 1).** The default black-input Video frame (Sweep running, field 2) is
  **100 % bit-exact** with the integer model `imodel.py`, and so is a Mirror + Spines + Ember + Invert frame with
  K1 700, K2 300, K5 800, K6 700, P12 300 (glides modelled). Osc and camera frames checked by eye. RTL vs still at
  720p: mean colour (18, 44, 105) vs (26, 51, 107); 4 × 4 region means within 15 levels (median), 36 (max); B share
  by 180-line band 0.99 / 0.84 / 0.40 / 0.17 / 0.02 / 0.09 vs 0.98 / 0.79 / 0.42 / 0.24 / 0.06 / 0.07, G share
  0.50 / 0.09 / 0.06 / 0.06 / 0.04 / 0.31 vs 0.47 / 0.15 / 0.04 / 0.03 / 0.09 / 0.40.
- **Fit and timing history.** The first build came to 7518 LC (98 %) and passed HD Analog only on seed 5 (74.5 MHz):
  the critical path was the line engine's state decode into its multiplier operand mux (11.5 ns of routing). The
  operand fetch became a registered one-hot AND-OR, the ×38 moved onto the multiplier, the level ramps became
  shift-adds, the tilt came from the noise cell index, the dot phases went to 8 bits, the phase to 28 bits with an
  add / subtract for Mirror, one clamp and one shifter are shared, Spines' modulo moved into the engine and both
  multipliers went to 32 bits: 7018–7057 LC, 85–88 MHz on HD.

## Status
- **v0.1.0, built 2026-11-09, router2 (the default), all six configs on seed 1.**
  `cd /home/ron/videomancer-dev && MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh quill`. Not flashed.
- Post-route Fmax: HD Analog 88.32, SD Analog 85.24, HD HDMI 85.26, SD HDMI 71.00, HD Dual 87.67, SD Dual 91.18 MHz.
- 7018–7057 / 7680 LC (91.4–91.9 %), 16 / 32 EBR. iceunpack OK × 6; the packed config carries the labels and the
  8 presets. `out/rev_b/ron/heirloom/quill.vmprog`.
- router1 also closes this netlist (HD Analog seed 1: 94.8 MHz in a profile run); use it if a later version gets
  tight. dupin: 0 `CARRY I0==I1` (8 LUT duplicate inputs from sign extensions, harmless).
- No pin-skew seed screen was run. If HD HDMI shows lines on hardware, re-seed that config.

## Deviations from the photo
- **Band texture.** The spine band is noisy horizontal cyan / magenta stripes (a d/44 texture key with a three-hump
  wiggle and noise), not the still's larger wavy blobs with "C" / "<" chevrons; the chevrons of the later frames
  are not drawn.
- **Far field.** Regular striations along the quill contours broken by 16 px × 4-line noise cells, where the still
  has bigger blobby green / red streaks; a little less green at the bottom (31 % vs 40 %).
- **Barbs.** Six fixed line phases with horizontal dashes; the still's dots are blobbier, and the output boxes
  (the HD HDMI bar fix) soften them.
- **Borders.** The capture's 40 px black left border is not drawn; the field starts at x = 0. The tongue-top tilt
  follows the screen x, not the mirrored x′.
- **Motion.** The clip's two gestures (spine down, level down) are one coupled sine LFO here, anchored on the still;
  at +0.5 s the spine lags the clip (y 128 vs ~210), then tracks it. The denser look just before the still comes
  back at the top of the sweep, or with P12 ≈ 600. The LFO steps per field, so 50 Hz sources sweep 5/6 as fast.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3, so the leftmost tongue and the
  convergence point on the left edge fall outside the screen, and SD is softer.
- Nothing visible was cut for budget; the tilt became a cell-index ramp and the dot phases 8 bits.
