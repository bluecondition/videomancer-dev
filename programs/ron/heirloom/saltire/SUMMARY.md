# Saltire — a neon X: jagged magenta bands with white-fringed teeth, flaming at the crossing, over wavy green stripes

**v0.1.0 (2026-11-04): built, NOT flashed.** Heirloom (video phase), rebuilt from the clip
`WIN_20230620_00_50_54` at 0.5 s (`photos/vid/WIN_20230620_00_50_54_t0p5.png`). The still is mirrored
top to bottom. In each half a magenta band runs from the left edge to a vertex at x ≈ 1530, where the
two halves nearly touch, and then climbs as a yellow arm to the corner: an X. Scallop teeth bite the
bands' inner edges and turn into flames at the crossing. A second yellow, red and magenta arm comes
down from the top edge. Left of it lies a blue field with soft black blobs. Behind everything run wavy
green and teal stripes that scroll down. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_saltire/`; `notes.md` has every number.
- **Mirror.** Flip-correlation puts the mirror axis at y = 516.5 (per 240-px column, 510..522). The
  program uses v = |y − 517|. The stripe wave and the teeth follow y, not v, so they are not mirrored.
- **Three keys.** R, G and B are separate keys, one per gun:
  - R is the X bands.
  - B is the blue field plus the magenta parts.
  - G is the stripes plus the yellow parts.

  Colour adjacency counts (yel–red 1217, red–mag 1619, mag–blu 1430, blu–red 88, blu–yel 7) show a hue
  staircase yellow < red < magenta < blue, cut by keyed black.
- **Stripes.** A ramp oscillator, hard-synced to the line start and frequency-modulated by a vertical
  sine:
  - Left edges on the centre line are at 44, 265, 509, 762, 1035, 1267, 1522 and 1776. The period is
    252 px and the oscillator starts at 13 px; the first cycle starts late, at 44.
  - The wave displacement grows with x (3 px at x 40, 24 at 760, 36 at 1270, 47 at 1780), which is
    0.027·x. That makes it FM: phase = x(1 − 0.027 sin(2π(y − yw)/224))/252.
  - The vertical period is 222–227 lines (4.8 cycles per frame), with all stripes in phase.
  - Each stripe has a bright left-edge spike that decays over about 12 px, a plateau about 104 px wide
    (duty 0.41), and a combed right end.
  - Stripe by stripe: k0 is solid bright green (10,255,0); k1 is green, spike 232 to 135; k3 and k4 are
    teal with a rising ramp; k5..k7 are dark green, spike 188 to about 50. A line ripple of ±10/55 runs
    every ~15 lines.
- **The X.** One band per half, a skewed V in (x, v).
  - The inner edge, teeth removed, runs (0,266) (300,225) (450,172) (1050,165) (1250,105) (1450,70)
    to the vertex (1530,55), then rises at slope about 0.95 to (1875,380).
  - The outer edge runs (0,370) (700,280) (1050,262) (1450,205) (1530,200), then (1680,340) (1780,465).
- **Teeth.** The folded edge profile gives scallops: pointed black cusps rising into the magenta and
  rounded magenta U's hanging down. They repeat every 84 px, which is P0/3, the stripe oscillator's 3rd
  harmonic. The depth is ±24 px on the left. At the vertex the band ends in a right-pointing arrowhead
  of tongues, with horns above and below.
- **M arm.** A diagonal from the top edge (x 690..1100 at v 500) down into the arrowhead. Its right edge
  is x = 1098 + 0.79(500 − v), and it has teeth. Across its width it reads yellow (right 0.4), red, then
  magenta.
- **Blobs.** A blue field with soft black blobs (the top-left corner, and x 720..900 at v 290..460).
  The same blobs make red patches in the magenta band (150..330 and 620..800). Their edges are soft,
  about 30 px.
- **Drips.** Yellow lumps hang under the inner edge at x 250..400 and 600..700, and a small yellow ">"
  chevron sits in the left wedge at x 420..560.
- **Neon fringe.** At magenta's trailing edges (scanning right into dark), G ramps from 2 to 200 over
  the last ~12 px, giving white (211,189,222).
- **Palette** (k-means, 12 clusters):
  - black (17,15,7) 34 %
  - blue (12,2,253) 13.6 %
  - magenta (247,3,251) 12.4 %
  - yellow (230,230,6) 10.7 %
  - dark green (13,70,9) 10 %
  - green (6,193,33) 5.5 %
  - red (240,9,6) 5.4 %
  - fringe white 2.3 %

## Motion in the clip
- **Wave scroll.** The vertical wave scrolls DOWN. The displacement minimum moves 183.8 → 233.9 → 310
  → 357.5 → 415.5 rows (m0, still, m1, m2, m3; +50, +76, +48, +58 per 0.5 s). The 8-frame strip agrees
  (+52..+68 per 0.5 s). That is about 112 lines/s, one wave cycle per ~2 s.
- **Teeth and flames.** They show no steady horizontal drift (+4, −30, −60, +16, +38 … px per 0.5 s).
  They ride the wave: they are the FM'd oscillator's harmonic, so they swing ±16..48 px as it scrolls.
- **Everything else.** The band, the X, the M arm and the blobs are static over the 2.5 s (top-left
  mean frame difference 7).
- **Other looks.** The clip shows no other look of this patch, so it gives no knob ranges.
- **The program.** At the default Speed the wave phase steps 547/65536 per field: 1.87 lines per field,
  which is 112 lines/s at 60 Hz. The stripes, teeth, flames and tongues all ride it.
  - In the integer model the wave-minimum row goes 18 → 75 → 127 at 0 / 0.5 / 1.0 s; the clip's goes
    8 → 84 → 131.5.
  - In GHDL, an RTL frame captured 6 fields later shows the wave moved +11.5 lines (+11.2 expected), and
    it is bit-exact with the model at that field.

## What it does
1. **Stripes.** A Q20 phase accumulator steps by (1 − b sin)·kf/252 per virtual px. The sine is read
   once per line. The stripe index and phase address a 16 × 32 profile ROM (green or teal level). A
   per-line LFSR adds the level ripple and the combed ends.
2. **Teeth.** A second accumulator carries 3 × the stripe phase (Blade: 1×). Near the vertex its phase
   is fanned by near(x)·|v − 160|, so the flames lean outward. It addresses a scallop ROM (Blade: a saw
   ROM). The offset is amp(x)·tooth, where amp = A0 + dA·near(x) and both scale with P12 Flames.
3. **Band.** An x-ROM (512 × 4 px) holds the inner and outer edges as max(L band, R arm), the flame zone
   and the drip lumps. A pixel is in the wedge if v′ < inner + teeth, and in the band if it lies between
   that and the outer edge (+ flames near the vertex).
   - Per line, an arrowhead tip x is computed (1560 − 1.4|v′ − 160| + a tongue scallop + the wave's
     displacement). The magenta L part ends there.
   - The yellow R part starts at a split that wiggles with the wave.
4. **M arm.** A per-line diagonal, toothed by the same offset, gives yellow, red and magenta by distance.
5. **Blobs.** cos(x) + cos(v′) + 0.9 cos(x + 1.2 v′) + the top-left corner ramp + camera luma is compared
   with four thresholds (T/m). The result is quarter-step holes: blue → black in the field and magenta →
   red in the band.
6. **Colour.** Each pixel gets an 8-bit palette index. A 12-px look-ahead turns magenta into the neon
   white fringe before a dark pixel. The palette ROM has two gun banks. Then come the shell's two
   [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16 and the gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Stripes | the stripe oscillator's frequency, ×1/3 .. ×3 (exponential, 512 = the still's 252 px); the teeth follow as its 3rd harmonic. 0 = three fat stripes and big teeth, 1023 = fine stripes and a saw-edged X |
| K2 Wave | FM depth of the vertical sine, 0 .. 6.5× (square law, 512 = the still's 0.027): straight stripes and upright teeth at 0, deep zigzag stripes and writhing flames at full |
| K3 Cross | slides the X (band, arms, blobs, drips) ±704 px; 512 = the still. 0 puts the crossing at mid-frame, 1023 pushes it off the right edge (only the magenta jaws remain) |
| K4 Speed | bipolar (±8 deadband, square law): the wave scrolls down (+) or up (−); 680 = the clip's 112 lines/s, full = ~1100 lines/s |
| K5 Spread | vertical scale of the whole X, 0.45 .. 2.2: arms hugging the centre line (a thin X) .. the still (512) .. arms out of the frame with only the vertex and the R arms left |
| K6 Blobs | blob depth m = 0.36 .. 2.8 (exponential, 512 = the still); 0 = no blobs and no camera. Also scales how far the camera / Osc punches holes and bends the band (luma >> 3..0). Sampled per field (no glide) |
| S7 Fold | Mirror (the still) / Quad: v folded twice, two mirrored X's stacked in the frame |
| S8 Teeth | Scallop (the still) / Blade: one big leaning saw per stripe at double depth (the band becomes saw blades) |
| S9 Twin | Off / On: x folded about the centre, two X's facing each other joined by the band |
| S10 Guns | RGB (the still) / BRG: the guns rotated, giving a yellow band with green patches, cyan arms, a red field and blue stripes |
| S11 Source | Video / Osc |
| P12 Flames | teeth and tongue depth: 0 = smooth bands and a blunt arrowhead; 300 = the still; 1023 = 3.4×, a shredded inferno with long tongues streaming right |

## Video
- **Video mode (the default).** Camera luma, taken as (Y >> 2) − 16 from a one-line buffer, enters at
  two points:
  - The blob sum (+15/16 luma). Bright parts of the picture become black holes in the blue field and
    red patches in the band, posterized by the same quarter-step keys as the photo's blobs.
  - The band coordinate (luma >> ke, ke set by K6; full white = 55 px at the default). The jaws and the
    X bend around bright shapes.

  The X, the stripes and the arms stay where they are, so the composition reads at the default.
  A black input is exactly the still.
- **Osc mode.** The shell's drifting 640 × 540 triangle lattice feeds the same two points. The blobs
  swell and drift, and the band edges bulge gently. K6 scales it.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial 18 × 16 shift-add multiplier runs 23 slots:
    - the stripe gain C and its exp2;
    - the FM depth (k2²);
    - the spread gain;
    - the speed step (s²);
    - the flame amplitudes;
    - the blob 1/m and its four thresholds;
    - the Cross shift;
    - the wave step per line;
    - the blob x start;
    - the Osc y step.
  - Operands are pre-muxed in two registered halves.
  - Exponents index a 256 × 16 exp2 ROM; the shifts are folded into constant operands.
  - A shared glide covers K1, K2, K3, K5 and P12. K6 is sampled per field.
- **Per-line engine.**
  - A run is kicked at each line start (or twice in the zone) and computes the NEXT line. It has four
    prep clocks, then 17 micro-ops dest = base ± (src·K >> sh) on one serial 36 × 17 multiplier, at
    21 clocks each (≈ 362 clocks, inside SD's 858). The ops produce:
    - the shear base;
    - |bs| and the stripe increment;
    - the teeth increment and the start phases;
    - the arrowhead tip (with the wave's displacement and the tongue);
    - the arm diagonal;
    - the drip chevron;
    - the blob diagonal phase;
    - the corner;
    - the split;
    - vp and the next hblank's cos(v) and tongue addresses.
  - sin θ, the tongue's tooth and cos(v′) are read in the hblank before, when the ROM ports are free.
- **Beam.** Virtual x (Q4), the stripe phase and the teeth phase (Q20), and the blob x phase as a DDA.
- **Pixel path.** 16 stages to the palette index. Only narrow differences and flags are carried:
  vv − edges saturated to ±511, the arm x saturated to 10 bits, and 1-bit tests.
  - The x-ROM (4 EBR), the stripe ROM (1), the tooth ROM (1) and two cosine ROMs (2) are each read
    with a fabric copy.
  - The index rides a 256 × 16 block-RAM ring for the 12-px fringe look-ahead. The sync and avid delay
    lines' middle taps ride the same ring.
  - Then come the 512 × 32 palette ROM (4 EBR) and the shell's output boxes, with their 8-back taps in
    six block-RAM rings and the running-sum resync.
- **Contracts.** One latency, C_LAT 40, in every mode. The generated avid. The blanking gate. Luma is
  capped at 940 before the 13/16 scale. Virtual 1080-line coordinates. Colours are in the hardware
  convention (U = Cr, `hwcolor.py`). Every collision-free RAM carries `no_rw_check`: the line buffer
  writes 4 addresses ahead of the beam's read, and yosys reports 0 transparency emulations.
- **Verified (GHDL, 720p60, decimation 1, against `imodel.py`).**
  - The default frame with a black input is 99.8 % bit-exact (excluding the 16-px box start-up at the
    left edge).
  - A frame 6 fields later is 99.8 % bit-exact at its field.
  - The camera frame is 84 % exact: the model reads the camera one line late, the RTL the current line
    4 px back.
  - Osc and camera renders are in `cmp_final.png`.
- **Integer model against the still.** Mean abs RGB 37. The 4×4 region means are within 2.5–22.5.
  Palette shares (still / model): black 34.4 / 34.6 %, blue 13.6 / 13.1 %, magenta 12.4 / 15.6 %,
  yellow 10.7 / 11.0 %, dark green 10.1 / 12.8 %.

## Status
- **v0.1.0 built 2026-11-04 with ROUTER1. NOT flashed.**
- `out/rev_b/ron/heirloom/saltire.vmprog`; iceunpack OK on all six bitstreams.
- Seeds and post-route Fmax (all seed 1):

  | Config | Fmax (MHz) | LC |
  |---|---|---|
  | HD Analog | 78.88 | 7187 |
  | SD Analog | 77.94 | 7179 |
  | HD HDMI | 76.80 | 7189 |
  | SD HDMI | 77.56 | 7183 |
  | HD Dual | **75.27 (thin)** | 7200 |
  | SD Dual | 75.47 | 7183 |

  LC is out of 7,680 (93.5–93.8 %); EBR is 24 / 32.
- **Rebuild.** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh saltire`
- **Fit history.**
  - The first build came to 8,865 LC (115 %). The line engine's shift-add steps took ~1,000 LUTs, and
    20 stages carried wide values.
  - Moving the line engine onto micro-ops, putting the zone arithmetic on multiplier slots and
    restructuring the pixel path brought it to 7,664 (99 %).
  - Further savings brought it to ~7,190:
    - the R arm folded into the x-ROM;
    - the blob x phase as a DDA;
    - exp2 shifts as constant operands;
    - narrower carried differences;
    - Q20 phases;
    - the fringe and sync delays in a block-RAM ring;
    - the line buffer made collision-free;
    - the soft outer-edge blend dropped.
  - Timing: the v2 netlist's critical path was the zone operand mux (17.7 ns of routing, 49.9 MHz). It
    is now pre-muxed in two registered halves, and the line engine's write-back is split over two clocks.
- **HD Dual is the thin one (75.3 MHz).** If a later edit regresses it, try router1 seeds 2–3, then
  profile.
- **No pin-skew screen.** If HD HDMI shows lines on hardware, re-seed that config.

## Deviations from the photo
- **Blobs.** The blobs are a three-cosine lattice plus a corner ramp. They land the photo's main holes:
  the top-left corner, (240, v 300) and (820, v 360). Their exact outlines are rounder and more regular
  than the photo's organic shapes. The photo's extra red streak beside the M arm and the small blue
  island at (1100..1250, 150..330) are not drawn.
- **Neon fringe.** The fringe is narrower and less frequent than the photo's: 0.4 % of pixels against
  2.3 %. It fires only on pure-magenta trailing edges, not on blob-blended magenta, red or yellow. The
  red trailing smear and the yellow's B pulse are not drawn.
- **Flame zone.** The flames are a stylised version of the photo's: scallop tongues fanned by a
  per-line pivot (|v − 160|, not the band's true centre), plus a per-line arrowhead of horizontal
  tongues. The photo's crescent horns are more varied.
- **Drips.** The drips are a fixed chevron band cut by the teeth phase, plus two lumps. The photo's
  yellow pieces are more irregular.
- **Soft edges.** The band's soft magenta → blue outer edge (one quarter-step blend) was cut for
  budget. The outer edge is hard, then softened only by the output boxes.
- **R arm.** The R arm's edges come from the 4-px x-ROM, so the steep yellow arm has 4-px stair steps
  that the output blur softens.
- **Brightness.** The brightest yellow comes out at the shell's luma cap (13/16, about 777).
- **Twin and blobs.** Under Twin the blobs are not folded (their x phase is a DDA on the unfolded x).
- **Resolutions.**
  - 720p is the same picture as 1080p.
  - SD (4:3) spans the same 1920 virtual px across the narrower frame, so the X is squeezed to 3/4
    horizontally (steeper arms, narrower stripes). The vertical structure is identical. Ron should
    expect this at SD.
