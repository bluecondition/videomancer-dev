# Tartan — Heirloom #25: a neon A-frame cut by squeezed dark bands

**v0.5.0 (2026-10-27): rebuilt from the photo WIN_20230410_01_13_24, built, NOT flashed.**
The v0.1 (generic diagonal stripes sliced into offset bands) was too abstract. This version is measured
from the photo. A big A-frame of nested diagonal stripes is cut by horizontal bands. The bands alternate
between a bright colourizer (cyan, green, orange, magenta) and a dark one (black with cyan and green
stripes, edged in yellow). The bands are thin in the middle of the frame and wide at the top and lower
down.

## Reading of the photo (measured; .heirloom_work/rb_tartan/notes.txt)
- **Palette (k-means, 10 clusters).**
  - Black (16,21,10) 28 %, cyan (2,217,218) 25 %, green (4,245,137) 20 %, orange (251,96,7) 13.5 %,
    magenta (253,18,174) 3 %.
  - The yellow fringe peaks at (205,222,93) on cyan → black and (73,241,28) on green → black. It is about
    7 px wide and sits only on edges INTO black, moving right; the edges out of black are clean.
- **Two colourizers.**
  - In the bright bands only cyan–green, green–orange and orange–magenta ever touch, so the bright bank is
    a 1-D colourizer, C < G < O < M.
  - The dark bank shows black, cyan and green.
- **Bands.** The boundaries fit phase(y) = 0.2284 + 0.011381·y + 0.924·sin(2π(y − 463.2)/729.2), bright
  where frac(phase) < 0.4986.
  - That is a vertical square wave whose frequency is modulated by a slow vertical sine: 10.8 bands per
    frame, 1.48 modulator cycles.
  - Band heights: 26–28 lines in the middle (y 400–550), 92–135 at the top and at y 700–910.
  - Fit: within ±3 lines, 96.7 % of rows typed correctly.
- **A-frame.** u = |x − 984.5| − 0.9 y. A general T(x) + V(y) fit by contour alignment gives T linear
  (slope 1.01 on both sides, flattening only at x < 55) and V linear (0.89).
  - In u-space every stripe edge is a vertical line through all bands. The colour is therefore a measured
    barcode of u, one per band type and side, accurate to 91.5 % (bright) and 92.6 % (dark) of pixels.
  - The inner ripple (period ~80 px) is symmetric about the apex. The outer parts differ left and right.
- **Junction.** The left family's stripes run 55 px (bright bands) or 75 px (dark bands) past the apex
  and curl into hooks.
- **Corner.** The far-left stripes (u > 438) are orange/magenta in the top bands and cyan/green below:
  the same edges two colourizer levels apart.
- **Glitches.**
  - A dotted yellow line (32 px period, 11 px dashes) on the first 1–2 lines of each dark band, x < 520.
  - A horizontal slip of the first ~18 lines of the top dark band.
- **Edges.** Soft horizontally (10–90 % over 9–12 px); band edges sharp (1–2 lines).

## What it does
1. **u (Q9).** g·(±(x − xc)) − g·s·y + scroll + curl + Bend·luma, wrapping mod 2^21.
   - The left family (−) holds to xc + 55 px in bright bands and xc + 75 px in dark bands.
   - In dark bands the left family adds a curl, 0.004·(x − xc)² px, past the apex: the hooks.
   - Mirror uses |x − xc| with the left table on both sides.
2. **Barcode.** Index = u/512 + 1024 (mod 4096), folded back at the table ends, into a 4096 × 8 ROM.
   - Address: side (1 bit) and index (11 bits).
   - Data: the bright code, the dark code and the corner flag.
3. **Band.** Per line, phase = roll + b·y + c·sin(2π(y − y0)/729) from a quarter-sine ROM. Bright where
   frac(phase) < duty (P12).
4. **Corner.** Bright bands that start above line 280 keep the warm top-left corner. Below that, orange
   becomes cyan and magenta becomes green there, so the corner edge always lies on a band boundary.
5. **Fringe.** A 7-px look-ahead window (5 at 720p, 4 at SD): a non-black dark-band pixel with black
   within the next F px turns pale yellow (on cyan) or yellow-green (on green).
6. **Tear.** The dotted line on the first 2 virtual lines of every dark band (x < 520). One dark band in
   four, picked by an LFSR, slips lines 2–19 by ±32 px; the slips re-roll every 32 fields.
7. **Palette and output.**
   - Two 8-entry palettes, luma pre-boosted 16/13 (drawn luma ≤ 759 after the output's 13/16).
   - [1 2 … 2 1]/32 on Y/U/V at 1080, /16 at 720p and SD (the photo's soft horizontal edges).
   - Luma 13/16, then the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Stripes | the nested A-frames | colourizer gain 0.35–3× (square law; 1.0 = the photo at 507) |
| K2 Slope | the A-frame's legs | leg slope 0.2–2.5 lines per px (square law; 0.9 = the photo) |
| K3 Rows | the horizontal bands | row-oscillator frequency, 3–40 bands per frame (square law; 12.3 = the photo) |
| K4 Squeeze | the bands thinning in the middle | FM depth 0–1.9 cycles: even bands at 0, the photo at 0.92, bands folding back near max |
| K5 Speed | stripes, bands, squeeze | bipolar, still in the centre ±8 (square law). The A-frames climb (6 px/field at full), the bands roll (0.02 cycle/field), the squeeze point drifts (4 lines/field). At the default one band rolls by in ~6.6 s |
| K6 Bend | the stripes bent by the picture | Video (or the Osc lattice) luma added to u, 0–360 px per full luma (square law; ~32 at the default) |
| S7 Fringe | the yellow fringes into black | On (the photo) / Off |
| S8 Mirror | the A's apex | the left family on both sides: a symmetric, seamless A |
| S9 Tear | the dotted sync glitch and slipped band tops | On (the photo) / Off |
| S10 Palette | the colours | Photo / Night (navy, blue, violet, amber, pink) |
| S11 Source | Video / Osc | Video (default) / Osc |
| P12 Shade | the dark bands | dark-band duty: 0 = the all-bright tartan, 50 % = the photo, 100 % = the all-dark tartan with yellow-edged black stripes. Glided 1/8 per field |

## Video
- The camera enters the patch the way an analog artist would. Its luma (unipolar, 8-px box) is added to
  the A-frame coordinate, so the stripes bend around the picture. Black adds nothing, so with no camera
  Video mode shows the photo.
- At the default Bend the photo's composition reads clearly, with the picture worked into the stripes.
  At 100 % the stripes shred around the image.
- In Osc mode a drifting 2-D triangle lattice (341 × 205 virtual px, 0..127, AC-coupled) feeds the same
  Bend path. At the default it gives a gentle ±11 px wobble.

## Engine
- **Vblank-zone sequencer** (zone = no avid for 1024 clocks), one small operation per zone clock:
  - a shared glide over K1, K2, K3, K4, K6 and P12;
  - one serial multiplier for 16 products: the knob squares × constants, g, s, g·s, the beam start
    g·(xc − x0), the beam step g·xs, b, c, the Bend gain and the three speed rates;
  - the motion accumulators (scroll, roll, modulator offset), duty and the LFSR seed.
- **Per-line engine** (kicked at each line start, computes the NEXT line, ~58 clocks), on one serial
  multiplier:
  - V·g;
  - b·y;
  - 90·y − θ into the quarter-sine ROM, then c·sin;
  - the phase (Q20) and the band type;
  - band starts: the LFSR draw, the slip and dot flags, the corner latch;
  - the line term.
- **Beam:** A = g8·(x − xc) by DDA (Q9).
- **Pixel path (32 stages, latency 32 in every mode):**
  - S1–S5: side, u, curl (8×8 product), Bend (two nibble partials).
  - S6: fold and ROM address.
  - S7–S8: EBR and fabric copy.
  - S9: code and corner.
  - S10–S16: fringe window.
  - S17: fringe and dots.
  - S18: palette, neutral outside the line.
  - S19–S30: conditioning and the gate.
- **ROMs:** barcode 4096 × 8, quarter sine 256 × 16, palette 16 × 30 (7 EBR in all). tartan.py (pure
  Python, the build hook) holds the run-length barcodes and generates tartan_pkg.vhd.
- **Contracts:**
  - virtual 1080-line coordinates (x in half virtual px, steps 2/3/4; SD shows the centre 1440 virtual
    px);
  - generated avid (parity-locked);
  - one 32-clock latency in every mode;
  - colours in the hardware convention (U = Cr, V = Cb).
- **Verification:**
  - imodel.py is bit-exact against the GHDL render at 720p60 with Speed still: mean |diff| 0.02, no pixel
    off by more than 24 after the 4-px generated-avid offset.
  - The default (Osc, moving) render matches the model to within 0.43, and the motion frames 6 fields
    apart at full speed to within 0.42.
  - The P12 glide matches.

## Status
- **v0.5.0 built 2026-10-27, NOT flashed.** router2 (`MAKE_TIMEOUT=900 ./build_heirloom.sh tartan`).
- **Build seeds:** HD HDMI = seed 2 (seed 1 missed); the other five are seed 1.
- **Post-route Fmax:** HD Analog 83.21, HD HDMI 76.25, HD Dual 83.98, SD Analog 78.79, SD HDMI 78.43,
  SD Dual 71.97 MHz (SD Fmin 27).
- **Fit:** 5710–5727 / 7680 LC (74.5 %), 7 / 32 EBR. iceunpack OK × 6. tartan.vmprog 368,161 bytes.
- **If a later version gets tight:** HD HDMI is the thinnest (76.25); try
  `NEXTPNR_ROUTER="--router router1"` or another seed first.
- Presets are hand-written placeholders: Photo, Bright Tartan, Dark Tartan, Even Bands, Folded Night,
  Camera Weave.

## Deviations from the photo, and why
- **Centre junction.** The two stripe families meet at a vertical seam (xc + 55/75 px), with a curl
  standing in for the photo's hooks. The following are not drawn: the concentric "(" arcs (dark band at
  y 644–695 right of centre, and the top-left dark band), the magenta arch, and the solid orange and
  black blocks right of centre in the two widest bands.
  - The centre-right 4×4 cell is 22 RGB short of red.
  - Tried and rejected: a rounded apex, an additive two-family field, a second-triangle decomposition,
    slanted seams and a ripple envelope near the centre. None beat the barcode model by eye.
- **Top edge.** The fitted band oscillator puts a 13-line dark sliver at the very top. The photo's bright
  band starts at line 4.
- **Tear.** The slip is random per band (one in four); the photo shows one slipped band.
- **Edges and luma.** The yellow fringe is the brightest drawn colour (759 after the 13/16 output
  scale); nothing is capped.
- **Osc default.** In Osc mode the lattice gives a gentle ±11 px wobble that the still photo does not
  have: Bend stays live in Osc, as the method requires.
- **SD.** SD shows the centre 4:3 of the picture (the A-frame stays centred); 720p is identical to 1080p.
- **Motion.** The photo is a still, so the motion is designed: the A-frames climb, the bands roll and the
  squeeze drifts.

## Working files
`.heirloom_work/rb_tartan/`:
- `ref.jpg`, crops `c_*.png` / `z_*.png`, label maps `lab_*.png`, `chan.png`, `uview.png`, `bscat.png`.
- `rows.py` / `rowfit.py`: the band fit.
- `trans*.py`: colourizer order.
- `mkbar.py` / `common.py`: the barcodes.
- `tvfit.py`: the T/V fit.
- `split.py`, `hsplit.py`, `linesplit.py`, `dsplit2.py`, `bendfit.py`, `envtest.py`: the junction.
- `decomp*.py`, `addfit.py`, `softabs.py`: rejected models.
- `model.py`: the float proto. `proto_final.png`, `cmp_proto.png`.
- `sheet.py`: `sheet.png`.
- `imodel.py`: the integer model.
- `rtl_*.png`; `cmp_final.png`, `cmp_motion.png`, `cmp_p12.png`, `cmp_res.png`.
- `build3.log`, `notes.txt`.
- `old_v010/`: the v0.1 files.
