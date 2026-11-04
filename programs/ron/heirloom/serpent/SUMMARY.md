# Serpent — a coiled ribbon cut into fins and lens holes, a red halo glowing through

**v0.1.0 (2026-11-04): built, NOT flashed.** Heirloom phase 3, from the clip `WIN_20230529_01_27_21` at 0.5 s
(still `photos/vid/WIN_20230529_01_27_21_t0p5.png`). The still shows a coiled ribbon on black, an S that reads cyan,
green and yellow, with hard-edged purple and magenta tongues. Its edges are cut into fins and teeth, and rows of
vertical lens holes show a red layer behind it, striped with black, that also glows as a halo around the body. Below
the middle the coil fans out into thin tails running off to the right. Presets are hand-written placeholders.
Working files: `.heirloom_work/rb_serpent/` (`notes.md`, `proto_final.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **One colour field per line.** Each row was fitted with a phasor fit of G as frac(m x + h), with coherence
  0.93–0.98 on every row. So u(x, y) = phi(y) + m(y)(x − 1000)/1000: a horizontal ramp whose offset and gain are
  vertical CVs (`rowfit.py`).
  - The period 1/m is 967 px at the tongue row (y 128), 1358 px at y 300 and ~1040 px over y 450–620. It then drops
    to 523 px at y 800 and 371 px at y 1000 (the fan).
  - Fit (`fitlines.py`), with th = 2π(y − 110)/790 and the fan E = max(0, y − 630)/1080:
    - phi = 0.866 + 0.271 cos(th − 0.104) + 5.48 E (rms 0.046);
    - m = 0.910 + 6.36 E (a ±0.03 ripple is dropped).
- **Colour** (`colfit.py`, with X, Y = px/1000):
  - G is a sawtooth of u: 10 + 213 frac(u), rms 31, linear by bins from 10 to 223.
  - R and B are a diagonal ramp with a small step at each wrap: R = 100X + 90Y − 20 frac(u) (rms 5) and
    B = 297 − 136X − 55Y − 25 frac(u) (rms 17).
  - So the S is cyan and green on the left and yellow on the right. Purple follows a wrap on the left and magenta a
    wrap on the right.
- **Column oscillator.** The ribbon's on→off edges sit at x = 18.6 + 85.4 k, which is 22.48 columns per 1920 px
  (measured resets 531 … 1641, ±4 px).
  - Fins have a vertical right edge and a curved left edge, so the key is a sawtooth with a fast fall compared
    against a smooth threshold field T.
  - The same comparator makes the teeth along the lower-left edge and the vertical right edges of the body.
- **Body window.** From the 85-px on-fraction (`tsmooth.py`):
  - The left edge is xL = 726.5 + 74.4 cos(th − 2.726) − 1477 E px (rms 29 px). It wiggles in phase with the coil
    (664 at y 128, 811 at y 450), then sweeps down-left to 190 at the bottom.
  - The right edge is xR = 1396.6 + 194.0 cos(th − 3.080): 1216 at the top and 1563 at y 400–600. In the fan it is a
    vertical cut at x 1390.
- **Holes.** Vertical lenses 8–60 px wide and 150–400 lines tall, in rows at the coil's turning points (y ≈ 110,
  450, 780, 1000). In the middle row they fall in alternate columns. They show the red layer.
- **Tails.** Integer contours of u beyond x 1390, at (1500, 810/1040) and (1920, 760/940). That is gentler than the
  field inside the body, so it is a second field there. Each tail is a red band carrying yellow/orange fins split
  along the wrap.
- **Palette.** Black (0, 7, 0) covers 45 %, the red layer (246, 9, 1) 8.4 %, then the ribbon's greens, purples and
  magentas. The red halo is 60–80 px wide outside the body, with a black stripe in it.

## Motion in the clip
- **The coil scrolls down at 160 lines/s.** The row of minimum period moves y 40 → 128 → 200 → 280 → 360 at
  t − 0.5 … t + 1.5 s, which is 80 lines per 0.5 s = 0.20 turns/s.
- **The fan stays put.** The period drops below y ≈ 630 in every frame.
- **One cycle every ~4.7 s.** The 0.0 s look returns at ~4.6 s and ~9.6 s (Pv / 160 = 4.9 s). Through a cycle the
  tails multiply from 1 to 3 as the coil's steep side passes through the fan. The lower right fills with a red block
  of vertical fins (3.6–4.0 s), then a new tongue enters at the top.
- **No knob moves**: the clip shows only this look.
- **In the program** the default Speed (747) scrolls the oscillator at 0.20 turns/s, down. The fan is fixed in y, so
  the coil rolls through it. The integer model at t + 0.5 / 1.0 / 1.5 s tracks the m-frames' tongue (`cmp_motion.png`).

## What it does
1. **The vertical oscillator.** th = 2π(Turns · (y − 110)/1080 − scroll). Mirror folds y about the centre line.
2. **Per line** (the next line, on the line engine):
   - phi, m, xL and xR (the cosines at their fitted phases, plus the fan term).
   - The hole size H = Ha · max(0, |cos(th + 0.12 + 6.8 f E)| − 0.62), using Holes.
   - The u start value and step, and the tail-field start 19/32 · u(1390) + 0.41.
   - Bands scales the whole u field (phi and m) by ×1/4 … ×4.
3. **Per pixel.** Each of these is a DDA that runs at the stage that consumes it:
   - u (switching to the tail field at x 1390 on fan lines), plus Etch × luma;
   - x;
   - the column phase;
   - the red/blue ramps.
4. **The key.** The threshold is T = max(min(window, tail), −0.12):
   - window = max((xL − x)/256 px, (x − xR)/128 px) + 0.5;
   - tail = dist(u, integer)/0.18, right of xL + 250 px. In the upper half it applies only to the integer
     round(Bands), which is the tip.
   - The pixel is on where saw > T, minus the lens holes (a centred triangle < H, even columns). Lattice inverts the
     key inside the halo.
5. **The red layer.** Its amount is (1 − Tr)·5, where Tr = min(window, 0.5 + 3.6 · dist), with no right halo past
   the kink on fan lines. There are black stripes at window 0.05–0.22 and at Tr 0.725–0.85.
6. **Colour.** The guns go through per-gun YCrCb ROMs and are summed (Ice swaps the red and blue guns). Luma is
   pre-scaled 16/13 and capped at 940.
7. **Output.**
   - Chroma goes through [1 2 2 2 2 2 2 2 1]/16, luma through [1 2 1]/4, both centred 4 px back.
   - Then luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Turns | the coil's vertical frequency: 0.41 … 4.6 turns per screen, exponential (1.37 = the still, at 512). 0 = one slow bend, 1023 = a stack of tongues |
| K2 Bands | the colour bands (u scaled ×1/4 … ×4, ±2 octaves, 512 = the still): 0 = one smooth gradient and no tails; 1023 = a dozen hard-edged bands and a comb of tails |
| K3 Columns | the fin/tooth/hole column oscillator: 8 … 64 across, exponential (22.5 = the still at 512). 0 = a few huge fins and holes, 1023 = fine comb teeth |
| K4 Speed | bipolar (±8 deadband, square law): the coil rolls down (+) or up (−). 747 = the clip's 160 lines/s (0.20 turns/s), full = 1 turn/s |
| K5 Holes | the lens holes: none (0) … the still (512) … 3× wide and tall (1023) |
| K6 Etch | how far the camera (or the Osc) bends the colour field: 0 … 1.5 bands per full luma, square law. 400 = 0.23 |
| S7 Mirror | Off / On: the lower half folded up, giving the fan and tails at both top and bottom and the coil meeting its reflection |
| S8 Halo | Red (the still) / Off: no red layer; the ribbon is perforated on black |
| S9 Key | Fins (the still) / Lattice: the key inverted inside the halo; a red serpent with ribbon-coloured lens slots and fins |
| S10 Palette | Still / Ice: red and blue guns swapped, giving a blue halo and a re-hued ribbon |
| S11 Source | Video / Osc |
| P12 Coil | the coil's swing and the fan together: 0 % = a straight vertical ribbon (no S, no fan); 33 % (341) = the still; 100 % = a wild coil (2.5× swing) with a dense comb of tails at 2× fan |

## Video
- **Video mode (the default).**
  - The camera's luma is read as (Y >> 2) − 16 from a one-line buffer, multiplied by the Etch gain and added to u.
  - So the colour bands, the hard purple/magenta wraps, the tip's fins and the tails bend around the picture.
  - The body's shape, the holes and the halo stay put, so the serpent still reads at the default Etch.
  - With no input, Video mode is the still.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice drifting ~75 px/s feeds the same path. At the default Etch
  the coil's bands wobble by up to a quarter band. Etch scales it.

## Engine
- **Per-field sequencer (vblank zone).**
  - The shared glide covers K1, K2, K3, K5, K6 and P12.
  - There are 30 serial-multiplier slots (the line engine's multiplier, idle in the zone), using a 256-entry exp2 ROM
    for Turns, Bands and Columns.
  - The line engine's 15 constants (turns, phase, phi0, cosine amplitudes, fan slopes, blend, hole gain, m0/m1, N0,
    tail offset) go into a 16 × 32 block RAM, not fabric registers.
  - The pixel-path constants stay in registers: column step and origin, tail step, Etch gain, ramp steps.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in ~260 clocks, which fits SD's
  858-clock line.
  - It uses 15 sign-magnitude products on one 16-iteration serial multiplier.
  - The cosines are fetched from a 1024 × 16 ROM just before use.
- **Pixel path (20 stages).** No wide value rides a register pipe: each ramp is a DDA stepped by the generated avid
  delayed to its stage (u at 5, x at 6, column phase at 9, red/blue at 10). The stages:
  - 3: luma. 4–5: Etch (8 × 9 nibbles). 6: u + etch.
  - 7: frac, distance, integer, window distances. 8: window/halo forms, tail thresholds. 9: T, Tr.
  - 10: T·7/8, red amount, hole phase. 11: column key and holes. 12: on/Lattice and clamps.
  - 13: gun ROM addresses. 14: ROM data. 15: fabric copy. 16: sums. 17: compose.
  - 18–19: filters. 20: 13/16 and the gate.
- **Memories (18 EBR):**
  - line buffer 4;
  - cosine 4;
  - exp2 1;
  - gun ROMs 3 × 2;
  - chroma ring 1 (`no_rw_check`);
  - constant RAM 2 (`no_rw_check`).
- **Contracts:**
  - the generated avid (parity-locked);
  - one latency of 20 in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr);
  - the chroma running sums resync to neutral after 8 blank samples.
- **Verified (GHDL, 720p60, decimation 1).**
  - The black-input frame is **bit-exact with `imodel.py`** on every interior pixel (Y, U and V, max difference 0).
    So is a full-Speed frame after 3 fields, and the coil moved down 12.75 virtual lines per field as designed.
  - The Osc and Video (selfie) frames show the composition intact (`cmp_final.png`).
  - The integer model against the still: mean |RGB| difference 23.3, ribbon/red/black 43.5/6.5/50.0 % against
    41.0/8.8/50.1 %.
- **Fit history.**
  - The first cut came to 9291 LC (120 %), mostly from register pipes of wide values. Its biggest consumers were u,
    the ramps, the column phase, the window and a 24-deep output.
  - The fix:
    - delayed DDAs instead of pipes;
    - a block-RAM chroma ring;
    - C_LAT 24 → 20;
    - a 9-bit Etch gain;
    - one scratch register;
    - the line constants in a RAM;
    - one shared multiplier.
  - Result: 7106 LC.

## Status
- **v0.1.0 built 2026-11-04 with ROUTER1, all six configs on seed 1. NOT flashed.**
  - Post-route Fmax: HD Analog 77.11, SD Analog 73.96, HD HDMI 75.32, SD HDMI 79.78, **HD Dual 74.46 (thin)**,
    SD Dual 70.44 MHz.
  - 7086–7107 / 7680 LC (92.5 %), 18 / 32 EBR. iceunpack OK ×6.
  - Output: `out/rev_b/ron/heirloom/serpent.vmprog` (486,926 bytes, unsigned).
- **Rebuild:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh serpent`.
- `hd_clock_divisor` untouched. No pin-skew seed screen was run; if HD HDMI shows lines, re-seed that config.
- **If HD gets tight:**
  - HD Dual has 0.2 MHz of margin.
  - Re-seed first.
  - LC to reclaim: store xL/xR at integer h (−~60), move the colour-ramp constants into the constant RAM (−~70),
    use a block-RAM ring for the luma delay (−~50).

## Deviations
- **Holes** are regular lens rows in alternate columns. The still's holes vary irregularly in size (its top row sits
  only under the tongue, its bottom ones are arch-topped), and the body interior has more of them.
- **The fan** is fixed, as measured. In the clip the lower region also changes shape through a cycle (tails 1 → 3, a
  red block of vertical fins); here the tails shift as the coil rolls through the fan but follow a fixed pattern.
- **The tails** are a second, gentler field placed by four points. Their fins are smaller and more regular than the
  still's.
- **Small departures:**
  - m's ±0.03 ripple is dropped.
  - The window ramps are 256 / 128 px against the fitted 220 / 110.
  - The column seams (faint vertical lines at the resets) and the source's colour fringes are not drawn.
  - A thin red line marks the kink line at the lower body's right edge.
- **Etch** uses a 9-bit gain, which is coarse below K6 ≈ 150.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (virtual x 240–1680), so the tails' right ends and the left margin are cropped.
- The picture sits 3–4 px right of the source timing (the generated avid's start), as in the other generated
  programs.
