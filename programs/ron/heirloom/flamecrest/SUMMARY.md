# Flamecrest — a mirrored crest of rainbow flames hanging on black

**v0.1.1 built 2026-11-09, NOT flashed** (v0.1.0 built the same day). Heirloom phase 3 (video clips), from clip
WIN_20240213_23_38_52 at 3.0 s (`photos/vid/WIN_20240213_23_38_52_t3p0.png`). A mirrored crest of flames hangs from the
top of the frame on black: pink and white arch tops, cyan and green tongues on the sides, a yellow, orange and red fan
converging on a central X, black diamond notches with small islands, hard red strokes with olive shadows, a wavy pink
strip along the bottom and horizontal bars at the left edge. In the clip the colouriser offset cycles: the flames fall,
the black recedes, the diamonds bloom into nested rings, then black opens inside the top arches. Presets are
hand-written placeholders.

## v0.1.1 (2026-11-09): K5 Strokes made bold
The review of v0.1.0 found K5 near-dead: at sheet scale, 0, 512 and 1023 looked almost the same. Changes:
- **0 → 512:** the stroke half-width grows 0 → 153 (0.15 of a ring cycle). The tongue gate loosens with the knob:
  strokes appear where S > 280 - 5 k5 / 8, so none at 0 and the still's -10 px at 512.
- **512 → 1023:**
  - the width grows to 242;
  - the gate opens fully;
  - strokes may reach into the black ground: the allowed palette range widens by (k5 - 512) / 8 entries on each side;
  - the olive shadow offset grows from 8 to 15 px at 1080p.
- **Result:** 0 = clean flames; 512 = the still (bit-exact with v0.1.0); 768 = heavy strokes; 1023 = red strokes
  scorching every tongue and slashing into the black floor, with the flames visible between them.
- **Checks:**
  - the GHDL default frame is 100 % equal to `imodel.py`;
  - with K5 set to 1023, the frame-3 capture (glided to k5 = 680) is 100 % equal;
  - the thin-line check shows no new runs for K5 0 / 768 / 1023.
- **Cost:** +117 to +125 LC.

## Reading of the photo
Working files are in `.heirloom_work/rb_flamecrest/`: notes.md, proto*.py, imodel.py, masks.py, cprof.py, mirror.py.
- **Mirror axis** x = 1010. Flip correlation of the G channel: 0.49 / 0.62 / 0.67 on rows 0-300 / 300-600 / 600-900.
- **Diamond notches** at x 540 / 1480 (±470 px). Black diamond apex at y ~400-420. Island (~60 x 70 px) at y ~505.
- **Black onset** (column scan): sides y 440-560; the centre flame body ends at y 780-830 in red tips ~80 px apart,
  with a soft red glow.
- **Colour classes:** black 35.3 %, red 19.6 %, orange/yellow 13.6 %, pink/white 11.3 %, green/cyan 11.1 %, blue 1.1 %,
  dim fringe 8.1 %.
- **k-means (14 clusters):** black (1,8,0) 34 %, red (249,21,6) 11.5 %, yellow (234,211,13) 8.5 %, pale pink (221,197,214)
  7.3 %, cyan (36,217,229) 6.6 %, orange (242,114,21) 5.4 %, magenta-pink (230,90,204) 4.5 %, green (92,235,44) 4.4 %.
- **Tongue periods:** autocorrelation of the top band peaks at 67 / 136 px.
- **Bottom strip** y 990-1075, ripple periods 264 / 403 px, ±15 lines. **Left strip** x 0-~100: horizontal bars.
- **Strokes:** hard red, 30-60 px wide, with a dark olive shadow ~8 px to their right.
- **Mechanism as modelled:**
  - A vertical ramp (retrace near the bottom) multiplied by a horizontal shape of the folded distance from the axis.
    This is a fan of contours that deepens down the frame.
  - A diamond-ring and tongue oscillator (asymmetric triangle + two-slope vertical + tongue waveform) feeds a sawtooth.
  - Both go into a cyclic colouriser whose 1.4-unit cycle holds a wide black window. The centre-zone palette runs
    red, yellow, white, pink; the side-zone palette runs yellow-green, green, cyan, blue, pink.

## Motion in the clip
- No horizontal motion. Contours fall ~1.1 lines per field at the top (3-8 px per 0.33 s at 480x270).
- t 2.0-2.5 s: a black diamond with a red dot.
- t 2.6-3.7 s: colour floods in from the edges.
- t 3.8-4.3 s: nested rings (yellow core, cyan ring, red ring).
- t 4.5-5 s: black opens inside the top arches with magenta rims. One cycle is ~6-7 s.
- The same patch's other clips (23_35_24, 23_36_12, 23_37_31, and t 11 of this clip) are other colouriser gains and
  offsets: the ranges of P12 Blaze and the cycle.
- **The program** advances the colouriser offset 19 C per field (0.2 units/s; 7 s per cycle) at the default Speed.
  The black recedes, the notches bloom and black reaches the top as in the clip, a little slower in the notches.

## What it does
1. d = |x - 1010| (Twin: 2 |d - 505|). t = the mirror fold of d x Diamonds on a 940 px lattice.
2. s = P0(t) - su(t) R above the diamond row, Q0(t) - sd(t) (R - Rd) below. R is the ramp, with a 128-line retrace at
   y 952 + ripple. s is capped at 0.88, then the notch profile (hole + island) is added.
3. r = T(t) + g(|y - 505|) + A2 S(d x Tongues) in Q2 px. The phase is u = frac(r x Rings + Etch x luma).
4. pre = s - 0.05 + 0.24 (0.5 - u) + Etch x luma / 32.
   - Blaze: (pre - 0.12) G + 0.12 + lift. The retrace and the left bars keep unit gain.
   - The floor sits inside the black window.
5. index = (pre + offset) mod 8192 C, into a 4-zone palette (centre .. side, Bayer-dithered zone boundary).
6. Strokes: hard red where frac(u) is within the K5 half-width of 0.62.
   - The edge is jittered by two triangle waves.
   - Strokes appear only where the tongue waveform S exceeds the K5 gate, and only in the K5 palette range (the lit
     colours, widening into the black window above 512).
   - The olive shadow is drawn 8-15 virtual px to the right.
7. Output: one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V (block-RAM ring, resync to black), luma 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tongues | the flame tongues' density, x0.4 .. x3 (512 = the still's 140 px tongues); 0 = a few broad tongues, 1023 = fine lace |
| K2 Diamonds | the diamond lattice (notches, islands, the X), x0.5 .. x3; 0 = one wide crest with the notches off-screen, 1023 = three crests |
| K3 Rings | the nested colour bands inside each tongue, x0.33 .. x3; 0 = smooth broad flames, 1023 = dense bands |
| K4 Speed | bipolar colour cycling (±8 deadband, square law): flames fall (right) or rise (left); 680 = the clip's 0.2 units/s; full = ~10x |
| K5 Strokes | the hard red strokes and their olive shadows: none (0) .. the still (512) .. heavy, wide, ungated strokes slashing across every tongue and into the black ground, with longer shadows (1023) |
| K6 Etch | how much the camera (or the Osc lattice) bends the tongues and shifts their colour; 400 = a gentle bend |
| S7 Rise | the crest flipped: flames rising from the bottom like a bonfire |
| S8 Twin | two crests side by side (the pattern at double rate about x 505 / 1515) |
| S9 Frost | U and V exchanged: blue, teal and violet flames |
| S10 Embers | the black ground glows dark red, showing the hidden rings |
| S11 Source | Video / Osc |
| P12 Blaze | colouriser gain x0.5 .. x2.5 plus lift (-0.25 .. +0.35): 0 = a thin dying crest on black; 420 = the still; 1023 = the whole frame ablaze with dense rings and blooming diamonds |

## Video
- **Video (default):** camera luma, (Y >> 2) - 16 from a one-line buffer, times Etch (k6^2 / 2048; 78 at the default),
  enters two places:
  - the ring phase: 0.28 ring cycles at full white, so the tongues bend around the picture;
  - the colour level: up to 0.1 units, so bright areas push the flames' colours on.
  At the default the still's structure stays readable (RTL render with selfie.jpg). With no input it is the still.
- **Osc:** the 640 x 540 virtual-px drifting triangle lattice feeds the same path at 6x the camera gain.
  - At the default Etch it is an unmistakable flip: the black lower half fills with flames and the top darkens.
  - At full Etch it becomes a lattice of flaming diamonds.

## Engine
- **Per-field sequencer (vblank zone):**
  - One serial shift-add multiplier over 13 slots, with operands pre-muxed a clock ahead.
  - A shared glide (K1, K2, K3, K5, K6, P12).
  - A 256-entry exp2 ROM for the exponential knobs (Tongues, Diamonds, Rings, Blaze).
  - Also per field: the two-slope stroke width, the stroke gate, palette range and shadow offset, the etch gain (x6
    for Osc), speed^2 x 51 and the Blaze lift. The colour-cycle offset advances by the speed.
- **Line engine:**
  - Kicked at each line start, it computes the NEXT line in ~170 clocks (fits SD's 858) on a 12-bit serial multiplier.
  - Values: the ramp R, g(|dy|), the notch's vertical term, the retrace base, the strip edge and the strip bar level,
    and the two jitter phases.
- **Pixel path (28 stages, C_LAT 28 in every mode):**
  - hold left of the strip edge; |x - axis|; Twin fold;
  - d x Diamonds and d x Tongues (split products); lattice fold;
  - knot ROM 256 x 53 (P0, Q0, su, sd) read straight from the EBR register; tongue ROM; ripple ROM;
  - T and rd_x shift-adds; notch ROM 256 x 14;
  - VCA product split hi/lo; ring and etch products;
  - cap; ring term; stroke distance; Blaze split product; floor;
  - palette ROM 1024 x 32 (4 zones x 256, luma pre-scaled 16/13, cap 940); compose (strokes, shadow, embers, Frost);
  - box, luma 13/16, gate.
- **Memories (23 EBR):** line buffer 4, palette 8, knot ROM 4, notch, tongue, ripple, exp2, box ring 2, sync ring.
  `no_rw_check` is set on both rings.
- **Contracts:** generated avid (parity-locked), one latency, the HD HDMI conditioning with the running-sum resync,
  virtual 1080-line coordinates, colours in the hardware convention (U = Cr).
- **Verified (v0.1.1):**
  - GHDL 720p60 at decimation 1 with a black input: 100 % of Y, U and V samples equal `imodel.py`, both at the
    defaults and with K5 set to 1023 (glided to 680 by frame 3). The output sits 4 px right because of the generated
    avid.
  - Speed-1023 captures (v0.1.0) at frames 3 and 10 show the bands falling and the notches blooming.
  - Thin-line check on the model (720p / 1080p / SD; defaults, P12 0 / 100, each switch, K1-K3 at max, K5 0 / 768 /
    1023): 0 single-row runs of 24 px or more, except one 27 px soft bright row in the bottom strip at one cycle phase.

## Status
- **v0.1.1 built 2026-11-09 with ROUTER1, NOT flashed.** `out/rev_b/ron/heirloom/flamecrest.vmprog`.
- **All six configs on seed 1.** Post-route Fmax: HD Analog 92.97, SD Analog 95.14, HD HDMI 99.51, SD HDMI 92.37,
  HD Dual 97.68, SD Dual 84.76 MHz.
- 7186-7203 / 7680 LC (93.6-93.8 %), 23 / 32 EBR. iceunpack OK x6. dupin: no CARRY I0 == I1. The packed config
  carries the labels and presets.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh flamecrest`.
- **LC note:** v0.1.1 is ~90 LC over the heirloom ~7,100 target. If space is needed, drop one of the two stroke-jitter
  oscillators (~90 LC); the stroke edges become slightly less ragged.
- v0.1.0 (2026-11-09): router1, all six on seed 1; HD 102.26 / 93.37 / 100.51 MHz; 7069-7089 LC.
  - Fit history: the first build was 7494 LC (97.6 %). The output-box and sync delays moved into block-RAM rings, the
    jitter and Bayer pipes were replaced by reading the pixel counters ahead, and the knot ROM's fabric copy was
    removed. That gave 7069.

## Deviations from the photo
- **Tongues:** more regular than the still's organic, camera-blurred ones (one mirrored waveform).
- **Strokes at the default:** jagged V-hooks on one side of each tongue rather than the still's long irregular strokes.
- **Top centre:** pink and white arch tops where the still has red/yellow arches.
- **Island:** a small orange diamond rather than a green core with red pieces.
- **Flame-body tips:** thin spikes reaching y ~860 rather than rounded U-tips with a broad glow at ~800.
- **Bottom strip:** a rainbow retrace (red, yellow, cyan, pink) rather than pink/white/olive with green patches.
- **Left-edge bars:** shaded as a triangle wave (no hard row steps), so they cannot read as sync glitches.
- **Motion:** a constant-rate offset; the clip fills the notches with colour faster than the model does.
- **Resolutions:** 720p shows the same picture as 1080p. SD shows the centre 4:3 (virtual x 240-1680), so the left strip
  is off-screen there.
- Drawn whites are ~777 (the shell's 13/16 luma). One 9-tap box replaces the still's softer camera blur.
