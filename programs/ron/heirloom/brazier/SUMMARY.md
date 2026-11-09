# Brazier — a violet nested diamond breathing over a red ground of crescents, fins and flames

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230930_01_37_53_t0p5.png` (clip WIN_20230930_01_37_53, 13.3 s, frame at 0.5 s; the same look in clip
01_38_42, no other looks). A blue / violet nested diamond with glowing pink rims and red / green fringes is keyed over a
vertical ground ramp (near black at the top, bright red at the bottom). Three rows of keyed fins ride the ground: grey
crescents hanging from the top edge, violet-pink fins across the middle (stretched into long spikes on the right), and
magenta flames rising from the bottom. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_brazier/` (`notes.md`; `pal16_*` = palfit k = 16; `fins.py` / `finfit*.py` /
`midfit.py` / `ffit.py` / `orgfit.py` = the fin fit; `resets.py` / `runs.py` / `dfit.py` = the diamond rings;
`qstat.py` = colour classes by quadrant; `proto*.py` = float protos; `imodel.py` = the integer model of the RTL).
- **Ground.** The red channel follows the height: 35 at the top, 8 at y 240 (the darkest, slightly green-black), 50 at
  y 510, 98 at 720, 171 at 960, 190 at the bottom. Green is 14-16 at the top and about 5 at the bottom; blue is 0.
- **Fins: one ramp, frequency-modulated, wrapped and keyed.** Every rim (the hard right edge of a fin) lies on an
  integer of phi = F(y) (x + 134) + 0.7786, with F(y) = (1/226.66 px) (1 − 0.1408 cos(2 pi (y − 338) / 570)).
  - The per-row fin period fits this cosine to rms 1.3 px over all three rows (198.6 px at y 50 and y 600, 263 px at
    y 900).
  - The rims fit to rms 8 px.
  - The phase origin is fixed at x = −134, so the shapes skew more toward the right edge. That is why the right-hand
    crescents hook and the right-hand middle fins become long spikes.
- **Fin keys.** A fin is lit where frac(phi) > T(y), and every other fin adds 0.25 to the threshold (a sub-octave), so
  big and small fins alternate. T was measured per row:
  - top row: 0.407 + 0.0043 y (big crescents reach y 124, small ones y 66);
  - middle row: a dip to 0.46 at y 510;
  - bottom row: 0.963 at y 856, falling to 0 at y 1036, where the big and small flames merge.
- **Fin colours.** Fitted by 1 − frac(phi) (64 bins per row). Each fin has a bright rim at the reset: lavender-white
  (153, 149, 170), pink-white (215, 80, 250) or (240, 50, 250). Behind the rim:
  - crescents: grey (88, 86, 102) brightening to (104, 112, 108) at the leading edge;
  - middle fins: violet (183, 8, 255) warming to (202, 34, 244);
  - flames: pure magenta (255, 0, 255) with a violet dip near the rim.
- **Diamond.** Centre (1012-1020, 515). The field is D = |x − cx| + V(y):
  - above the centre, V = 0.95 (515 − y);
  - below it the sides steepen, and V = 522 (1 − exp(−(y − 515)/400)) fits ring 2 and the outer boundary at
    y 600 / 700 / 800 / 900 to within about 10 px.
- **Ring resets** (pink-white rim plus red/green fringe) sit at D = 86 (the inner diamond), 398 (ring 2, apex at y 85)
  and 575 (the outer boundary, keyed). The cubic g = 0.45 + 7.859e-3 D − 1.897e-5 D² + 2.263e-8 D³ passes through all
  three.
- **Ring palette.** Outward from each reset: pink-white rim (≈ 3 % of the cycle), magenta, violet (163, 8, 252) /
  (84, 5, 243), blue (19, 7, 203), navy (14, 11, 97), black (12, 15, 5), then green (39, 63, 17) and dark red
  (105, 8, 3) just inside the next reset.
  - Right of the centre the gradient runs the other way: a pink rim on the inner side, red/green outside, and a pink
    outer boundary that merges into the spikes.
  - The colour mix changes with height. At the top: black 34 %, navy 18 %, green 16 %. In the lower quadrants: violet
    36 %, blue 21 %, magenta 11-15 %.
- **Texture.** Edges are jagged by small horizontal offsets held over a few lines. Chunks at the top are torn (a green
  blob at x 950-1060, y 230-330, and a lavender lump at the top right). The violet is finely striated and the chroma
  is soft.
- **Palette shares** (palfit): black-green (12, 15, 5) 14 %, browns 20 %, magenta (243, 7, 253) 9 %, reds 22 %,
  violets 11 %, blue / navy 10 %, green 3.7 %, lavender-grey 2.4 %.
- **Integer model vs the still at defaults (1080p):**
  - mean |RGB| difference 24.2;
  - the 4 × 4 region means agree to 1-17 levels;
  - luma mean 43.7 vs 42.9, luma sd 34.3 vs 34.5;
  - the colour-class shares per quadrant are within a few % (`qstat.py`).

## Motion in the clip
- **One LFO with a 5.5 s period drives everything.** The fin parameters at t = 0.5 s and t = 6.0 s are identical to
  within 1 px.
- **Fins: the FM wave scrolls down.** A row's fin period swings between 199 and 261 px: the top row peaks at
  t = 2.75 s, row 960 at 1.2 s and row 1020 at 1.75 s. That is the vertical cosine moving down at 103.6 lines/s, one
  570-line wavelength per cycle. The phase at x = −134 never moves, so the fins hook, stretch into slashes and flatten
  from the right while the left-hand ones barely shift (`finmot.py`).
- **Diamond: it breathes** (keyframes from the 2 fps sheets, phase 0 = the still):
  - **Squeeze.** It narrows (kx 1.0 → 1.12 at phase 0.18 → 1.9 at 0.36), the wobble grows, and it becomes a wavy
    sliver (kx ≈ 2.8-3.2 at 0.45-0.5) with liquid tearing.
  - **Drop.** It falls away and re-appears from below, flat-topped with pillars inside (centre at y ≈ 950-1000 at
    0.55-0.64).
  - **Rise.** It climbs back: the top edge is at y 380 at phase 0.73, 280 at 0.82 and 200 at 0.91, then the still
    again.
- **In the program.**
  - The LFO advances once per field. At K4 = 700 it takes 198 / 65536 of a cycle per field, a 5.5 s period at 60 Hz
    (6.6 s at 50 Hz).
  - The fin wave phase is θ = (y − 338)/570 − L.
  - A 256-entry breath table holds (kx − 1, centre drop, wobble, jitter) for one LFO cycle, scaled by Melt.
  - RTL frames with the LFO preset to +0.5 s and +1.0 s move like the clip's m-frames (`cmp_motion_rtl.png`). The
    RTL frame at phase 0.45 shows the sliver, and its fins match the clip at t = 3.0 s (`cmp_br45.png`).

## What it does
1. **Ground.** A per-line colour from a 256-entry table (the measured ramp), which also carries the fin row
   (crescent / fin / flame).
2. **Fins.**
   - A phase accumulator, phi += F(y)·xs per pixel, starts each line at F(y) (x0 + 268 − 2·slip) + 0.7786.
   - The line's frequency is F = F0 ∓ A |cos θ| (Sweep sets A).
   - A fin is lit where the phase's top byte is greater than T(y), or T(y) + 64 on odd fins (the phase's integer
     bit).
   - The colour comes from a 256 × 32 palette indexed by row and 1 − frac(phi).
3. **Diamond.**
   - A signed beam accumulator S = KX (cx(y) − x) gives |S| plus the line's V term (plus Etch × luma) = D, in
     quarter px.
   - D/2 px indexes a 1024-entry ring table holding frac(m·g(D)). A cubic forward-difference DDA refills it every
     vblank from Rings.
   - The colour index is the table's top 6 bits, inverted right of the centre (S < 0). A 512 × 32 ring palette has
     8 vertical zones: greener and darker at the top, magenta at the bottom.
   - The diamond is keyed at D < 575 px. A dark outline (the ground at half level) runs to 589 px, under the fins.
4. **Per-line distortions.**
   - Wobble: Melt × breath × sin(y/240 + 2L).
   - Jitter: ±(r × jit)/128 px, redrawn every 8 virtual lines.
   - Tears: ±(r × jit)/32 px, 1 band in 32, lasting 2-9 bands.
   - Slip (S9): ±0-255 px on 1 band in 4, bands of 16 virtual lines, shifting the diamond and the fins together.
5. **Compose.** Diamond over fins over outline over ground, with Copper (diamond U↔V) and Ember (fin Cb inverted)
   applied here.
6. **Output conditioning.** Chroma through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum resync, luma [1 2 1] on
   the same centre, luma 13/16, then the blanking gate. Palettes are pre-scaled 16/13 and capped at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Rings | the diamond's ring density, ×1/4 .. ×4 (exponential, 512 = the still). 0 = one huge soft gradient diamond; 1023 = about 12 nested rings inside the same outline |
| K2 Fins | fin count across all three rows, ×0.35 .. ×2.8 (512 = the still). 0 = a few wide crescents and flames; 1023 = a dense comb of thin fins and slashes |
| K3 Sweep | the fins' FM depth (the vertical cosine), 0 .. 2.8× the still (360). 0 = identical upright fins and crescents; 1023 = long horizontal slashes and hooks |
| K4 Speed | bipolar LFO rate, ±8 deadband (stops, holding the phase), square law. 700 = the clip's 5.5 s cycle; 1023 = a 0.7 s cycle; left of centre runs it backward (the diamond rises then squeezes, the wave scrolls up) |
| K5 Size | diamond zoom, 0.3× .. 3.3× (512 = the still). 0 = a small diamond among the fins; 1023 = the diamond fills the frame |
| K6 Etch | how far the camera (or the Osc) pushes the diamond field (0 .. 64 px per luma step, square law; 300 ≈ 5 px per step, 80 px at full luma) and the fin phase |
| S7 Twin | one diamond / two half-size diamonds side by side |
| S8 Flames | Rose (the still) / Ember: the fins' Cb is inverted, giving orange-red flames and fins (the grey crescents stay grey) |
| S9 Slip | Off (the still's small jitter) / On: sync slips tear whole 16-line bands of the diamond and the fins sideways by up to 255 px |
| S10 Colours | Violet (the still) / Copper: the diamond's U and V are swapped, giving red / orange rings with blue fringes |
| S11 Source | Video / Osc |
| P12 Melt | breath depth plus wobble plus tear. 0 % = a crisp, still, geometric nested diamond; 512 = the clip; 100 % = twice the clip: a liquid, writhing sliver that drops right out of frame on each breath |

## Video
- **Video mode (the default).** The camera luma, as (Y >> 2) − 16 → top 4 bits, comes from a one-line buffer (2048 × 4).
  - It is added to the diamond field, so bright areas push the rings and the outline inward and the picture shows
    through as warped contours.
  - It is also added, at gd/4096 cycle per luma step (0.08 cycle at full luma at the default Etch), to the fin phase,
    so the fins fray along the picture's edges.
  - At the default Etch the composition still reads.
  - With no input, Video mode is the still (RTL with a black input is bit-exact with the integer model).
- **Osc mode.** A 2-D triangle lattice (160 × 135 virtual px, drifting) feeds the same path, so the rings crinkle into
  a lattice of bumps. Etch stays live.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier: 20 slots, 18 iterations, slot decoded two clocks ahead.
  - A shared glide for K1, K2, K3, K5, K6 and P12.
  - A 256-entry exp2 ROM for the exponential controls (rings m, fin F0, size kd).
  - Per field it computes:
    - the ring DDA's Q0, D1, D2 and D3 (m × constants);
    - F0, and A = F0 × Sweep × 205 / 2^19;
    - kd;
    - the breath entry at the LFO phase times Melt (kx − 1, the centre drop, wobble, jitter);
    - KX = kd (1 + kxe) (×2 for Twin) and KX × xs;
    - the etch gain Etch²/4096;
    - the LFO step from Speed² × 201 / 2^15;
    - the per-line wave steps;
    - the twin reload KX × 1960.
  - The ring table fill runs at zc 600-1624 and the line-0 kick at zc 1700.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in about 90 clocks (SD's line is 858):
  - cos / sin from a 512 × 16 ROM (sine plus the V profile, the profile interpolated over its 4-line steps);
  - Fl and its pixel step, and the fin start phase;
  - wobble, jitter and tears from xorshift32, and slip;
  - the centre cx, V × KV, and S0 = KX (cx − x0);
  - the ground colour, threshold and zone.
- **Beam.** A 28-bit signed S accumulator (Twin adds KX × 1960 where x crosses the centre) and a 25-bit fin phase.
- **Pixel path (15 stages).**
  1. Beam.
  2. |S| >> 7 (ones' complement) and side.
  3. + V.
  4. Luma.
  5. Etch product.
  6. + etch.
  7. Ring table address; key / outline / fin tests; fin palette address.
  8. EBR reads.
  9. Fabric copies.
  10. Colour index → ring palette address.
  11. EBR read.
  12. Copy.
  13. Copper / Ember.
  14. Copy.
  15. Compose.

  Then the conditioning stages 16-21.
- **EBR (18).**
  - ring table 1024 × 16 (`no_rw_check`; written in vblank only);
  - ring palette 512 × 32;
  - fin palette 256 × 32;
  - ground 256 × 32;
  - thresholds 512 × 8;
  - breath 256 × 32;
  - sine / V profile 512 × 16;
  - line buffer 2048 × 4;
  - exp2.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 20, in every mode.
  - The blanking gate.
  - The HD HDMI conditioning, with the running-sum resync.
  - Virtual 1080-line coordinates (x in half virtual px, steps 2/3/4; SD shows the centre 1440 virtual px).
  - Colours in the hardware convention (U = Cr), palettes pre-scaled 16/13.
- **Verified (GHDL, 720p60, decimation 1).**
  - The default frame (Speed stopped, Melt 512, black input) is bit-exact with `imodel.py` on 100 % of pixels from
    column 8 on (99.92 % overall; the first 8 columns are filter warm-up). This needed the xorshift offset of the two
    warm-up frames.
  - The same at breath phase 0.45 (the sliver).
  - Camera and Osc renders checked by eye.
  - Motion: frames with the LFO preset to +0.5 s / +1.0 s vs the clip.

## Status
- **v0.1.0 built 2026-11-09 with router1**, all six configs on seed 1:
  `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh brazier`.
- **Fmax (post-route):**
  - HD Analog 85.06, HD HDMI 81.91, HD Dual 86.01 MHz;
  - SD Analog 85.90, SD HDMI 72.53, SD Dual 86.74 MHz (SD Fmin 27).
- **Resources.** 6724-6754 / 7680 LC (87.6-87.9 %), 18 / 32 EBR. iceunpack OK on all six. The packed config carries
  the labels.
- **Output.** `out/rev_b/ron/heirloom/brazier.vmprog`. **Not flashed.**
- **router2 stalls.** The first router2 build stalled on HD HDMI (overused = 3 frozen after 3000 iterations), so stay
  on router1. dupin shows 0 `CARRY I0==I1`. One LUT with a duplicated input (the 0.953 |t| subtraction) was removed
  by splitting it over two clocks.

## Deviations
- **The diamond is the clean model, not the still's distortion.**
  - The still's torn top (a green blob, a lavender lump at the top right, chunks shifted sideways) is drawn only as
    the general jitter and tears.
  - The violet "S" bands and the fine striations inside the cycles are not modelled.
  - The program's diamond is more regular and symmetric than the photo.
- **The seam.** The reversed colour index right of the centre (fitted from the left / right rings) leaves a visible
  vertical seam through the diamond's top and bottom (jagged by the jitter). The still's upper-right ring 2 does not
  follow the reversal; it was kept because the right half's pink outer edge and lower rims need it.
- **Fins.** The fins' leading edges are hard (softened only by the chroma box). The photo's rims deviate from the fit
  by about 8 px rms. The bottom 8-line capture border is not drawn.
- **The breath** is a keyframed table read from 2 fps frames, not a fitted law. The drop / rise phase is
  approximated by moving the diamond's centre down.
- **Resolutions.**
  - 720p matches 1080p.
  - SD shows the centre 4:3: the outermost fins at the left and right edges fall outside, and the diamond is
    complete.
  - Interlaced inputs draw both fields from the same virtual lines (line-doubled), as field_n is not available.
- **Speed.** The LFO steps per field, so at 50 Hz sources the cycle is 6.6 s instead of 5.5 s.
