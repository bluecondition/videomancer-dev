# Bottleneck — two glass bottles on glowing stripes

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Rebuilt from the capture
`photos/WIN_20230429_22_57_43_Pro.jpg`: two lathe-shaped bottles with lime necks, pink bodies and
glass rims stand on horizontal stripes that glow brighter around them. Sliding frame-buffer glitch
blocks and a sample-and-hold staircase cut through them. The v0.1 version (keyed widths over blurred
bands) is replaced. Presets are hand-written placeholders.

## Reading of the photo (measured)
Work dir: `.heirloom_work/rb_bottleneck/` (`notes.md`, `geom.py`, `tables.py`, `bgmodel.py`, `decyuv.py`,
`proto7.py`, `imodel.py`).
- **Bottles.** Two silhouettes, each a pair of measured edge curves: control points refined on per-row
  rim features, left-edge rms 2.9 px, PCHIP between them. Both lean outward at the bottom.
  - Left bottle: neck x 580–690 running off the top edge, shoulders, widest 330–877 at y ≈ 500, a
    near-vertical left edge (x 268–290) for y 600–900, a pointed tip at about (355, 1000).
  - Right bottle: the lime fill narrows to a point at (1405, 60) while the blue rim bands carry on to
    the top edge; body 1188–1741 at y 430–750; tip at about (1640, 937).
  - Above the right bottle's shoulder and below both tips the edge curves cross, so the rims meet in a
    point.
- **Fill.**
  - Colours: lime (184,244,52) in the necks, pink (246,58,147) in the bodies.
  - The change is a band of pink PWM lines through the lime. On the left bottle: all lime to y ≈ 360;
    then the inside-distance threshold alternates every 6 rows between 80 and 218 px (a 12-line PWM);
    all pink from y 414. The right bottle runs about 50 lines lower.
  - The lime survives longest in the bottle's core, so the key is the inside distance,
    min(x − el, 1.5·(er − x)).
  - Red slews up for ~190 px after the scan enters a pink body (R 130 → 245, G/B flat at 60/147).
- **Rims** (fitted per row, feature-aligned, as offset + alpha over the fill or background):
  - Entering edge of the body: a lime rim at 0–10 px, a dark gap, a second green line at ~35–40 px.
  - Leaving edge: a dark violet notch at 0–8 px, a lime line at 20–28 px.
  - Neck rows: a blue band 0–60 px outside, then a grey-violet echo line.
  - Below y 740 the bright stripe rows run through the rims.
- **Stripes.** Horizontal rows:
  - Upper frame edges: thin violet lines (period ~30 at the top), green (y 120–135, 295–310, 400–420,
    540–650, 740–760), blue (190–260, 340–370, 470–500, 680–700), black between.
  - Below ~765: pink / violet / green rows with thin lime and white lines.
  - Between the bottles the same rows run much brighter (green → lime, black → violet-grey).
  - Measured as three per-row tables: the left frame edge, the middle, the right frame edge.
- **Halo.** Near a bottle the outer stripes blend toward the middle colours: weight 0.56 within 40 px,
  decaying exp(−(d − 40)/170).
- **Palette** (k = 14): pink 19.7 %, dark pink 10.6, black 9.6, blue 9.5, violet-grey 7.9, navy 7.1,
  olive 6.0, lime 5.9, green 5.3, dark green 5.0, grey 4.5, violet 3.2, pale lime 2.9, orange 2.8.
- **Glitch.** Rectangular horizontal displacement windows, each a frame-buffer slip:
  - rows 138–206, x ≥ 672: −18 px
  - rows 280–322, x 340–460: −45
  - right neck, x ≥ 1440: rows 300–345 −80, rows 345–530 −110
  - a dark blue block, rows 440–530, x 1200–1345: +150
  - the displaced rim, rows 530–667, x 1180–1420: +200
  - rows 828–865, x 200–330: −25
- **Staircase.** The left bottle's right edge is sample-and-held in 45-line steps from y 717 to 897.
  Each riser carries a copy of the rim.
- **Mechanism.** A camera-free patch:
  - a per-line shape generator (two lathe edges);
  - a keyed lime/pink colouriser of the inside distance, plus a ramp and a fine vertical PWM oscillator;
  - rim bands from the edge distance;
  - a stripe colouriser keyed brighter near the shapes;
  - a frame-buffer / TBC slipping blocks and holding one edge.

  A single cyclic colouriser of one field was tested: the ring-SOM phase map is smooth, but the zone
  colours are not a constant phase shift of each other. The stripes were rebuilt from measured tables
  instead.
- **Fit.** proto_final vs photo: mean |RGB| 17.4; the 4×4 region means are mostly within 10; 64-px
  local luma std is 28.1 in the photo and 25.8 in the proto, correlation 0.88. The RTL render (720p,
  Osc) vs the photo: 21.3 and std 24.5, correlation 0.83.

## What it does
1. **Edges.** Per line, the four edges come from a 1024 × 12 ROM sampled every 8 virtual lines and
   linearly interpolated.
   - Width and Spread are added as edge offsets.
   - Speed's pendulum sway grows linearly toward the bottom.
   - With Hold on, the left bottle's right edge is read at the end of its 45-line group (60-line steps
     above 70 % Shatter).
2. **One bottle per pixel.** The left bottle is used left of the midpoint between them. The side is
   right of the centre when dr ≥ dl; the outward distance is d = max(dl, dr) minus the Melt term.
3. **Fill.**
   - key = (din − th) / 4, clipped to 0..16, into a 17-step pink → lime ROM.
   - th = 2.2 (y − y0 + Level) + a 12-line PWM triangle + LFSR line noise, held 4 lines.
   - Red slews over 190 px after the entering edge on the pink side, fading out below y 760.
4. **Rims.** out = P + (16 − a)/16 · under per 4 px of d (−48 … 207 px), per edge side × edge-adjacent
   fill (lime / pink / lower pink). under = the fill inside the bottle, the background outside.
5. **Background.** Three per-row stripe tables (2-line resolution, 64-colour palette). Left of the left
   bottle and right of the right bottle they blend toward the middle table by the halo LUT (16ths of
   d / 8). K6 rolls the row index (wrapping); Speed wobbles it ±12 lines (clamped).
6. **Glitch.**
   - 12 blocks: the photo's 7 switch on as P12 passes 120..340, and 5 more at 560..880.
   - Each block's shift is base · gain, the gain 0 → 1 at 45 % and 2.25 at 100 %, plus a ±37.5 % slide
     on one of three LFOs.
   - One window per line (the last active block that covers the line).
7. A line texture of ±8 luma steps; luma pre-scaled 16/13 for the shell's 13/16 scale (whites ≈ 777).

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Width | the bottles' width | each edge moves out by (k − 512)/4 px: thin stems ↔ fat bottles touching |
| K2 Spread | distance between the bottles | the bottles move apart / together by (k − 512)/4 px each |
| K3 Level | the lime neck / pink body line | fill threshold + 0.75 (k − 512) lines: 0 = all lime, 100 % = all pink |
| K4 Speed | the moving parts | bipolar, still in the centre ±8. The bottles sway (±40 px at the bottom), the stripes wobble (±12 lines), the glitch blocks slide on three LFOs, the PWM lines crawl and the stair steps slide |
| K5 Melt | outlines and fill line | the camera's (or Osc's) luma moves them: (luma − 96)·2^(k/128)/16 px, ±25 px at the default, ±400 px at 100 % |
| K6 Stripes | the stripe field | rolls the stripes ±256 lines (wrapping); 512 = the photo |
| S7 Bottles | bottle count | Two (the photo) / One |
| S8 Hold | the staircase | On (the photo) / Off |
| S9 Rims | the glass rims and halos | On (the photo) / Off (bare silhouettes on the stripes) |
| S10 Colours | palette | Photo / Negative (chroma inverted: lilac necks, green bodies, violet stripes) |
| S11 Source | camera | Video (default) / Osc |
| P12 Shatter | glitch blocks and staircase | 0 = clean bottles; 45 % = the photo's 7 blocks and 4 stairs; 100 % = 12 blocks at 2.25× shift sliding over the frame, stairs to the bottom in 60-line steps |

## Video
- The bottles and stripes in the photo are synthetic, so the camera is woven in as an analog artist
  would patch it: its luma (line buffer, (Y >> 2) − 16) is added to the edge distance and the fill
  key.
- The outlines fray and bend around the picture, and the lime/pink line follows its light and dark.
  At default Melt (±25 px) the composition reads unchanged (`rtl_vid.png`, fed selfie.jpg).
- Osc substitutes a slow 2-D triangle (960 × 540 virtual px, drifting a phase step per field), whose
  luma bends the outlines the same way, so Melt stays live in Osc mode.

## Engine
- **Zone sequencer** (vblank: no avid for 1024 clocks; one 20 × 10 serial multiplier):
  - the shared glide (1/8 of the gap per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12;
  - the motion phase and LFOs, the sway and wobble amplitudes, the block activations;
  - the glitch gain, the hold end line and the three slide gains.
- **Line engine** (kicked at each line start, computes the NEXT line, about 100 clocks plus a 9-clock
  colour fetch, so it fits SD's 858-clock line):
  - edge ROM reads and interpolation; the hold group;
  - the block search;
  - two serial products on a slot table (sway, window shift);
  - the edges and midpoints, the PWM / threshold arithmetic, the LFSRs;
  - the stripe-row index (roll wrap + wobble clamp).
  - The stripe colours (index ROM → palette ROM) are loaded straight into the line registers once the
    line has left the pixel path.
- **Pixel path (21 stages, C_LAT 20):**
  - x and the window test;
  - shifted x and luma;
  - edge distances of the selected bottle and the melt shift;
  - side and outward distance;
  - melt, inside distance and zones;
  - rim ROM address and halo;
  - key and ramp;
  - fill ROM and background products;
  - fill and background;
  - under colour and the rim data's fabric copy;
  - alpha product, compose, 10-bit with chroma sign, clamps, the avid gate;
  - output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums, the 8-delay in block
    RAM), luma [1 2 1]/4 delayed to the box's centre, luma 13/16, the blanking gate.
- **Block RAM:**
  - the line buffer (2048 × 8, `no_rw_check`);
  - edge ROM 1024 × 12, stripe-index ROM 2048 × 6, palette 64 × 24, rim ROM 512 × 32;
  - sync / chroma delay rings (`no_rw_check`): 17 EBR in all. emulate_transparency = 0.
- **Contracts:** damier's generated avid, one latency in every mode, the HD HDMI bar fix before the
  gate, virtual-1080 coordinates from the timing ID, colours in the hardware convention (U = Cr,
  V = Cb), tables generated by `bottleneck.py` → `bottleneck_pkg.vhd`.
- **Fit history:**
  - The first full version came to 10.7k LC (139 %): per-bottle scale and centre multiplies, both
    bottles evaluated per pixel, two glitch windows, a melt multiplier, FF delay lines.
  - Moving the syncs and chroma delay into EBR rings, slotting the engine's products, evaluating one
    bottle per pixel and adding the fill ROM brought it to 8.5k.
  - Additive Width / Spread, one window, the melt as a shift and LFSR noise brought it to 7.4k.

## Status
- **v0.5.0 built 2026-10-27, ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh bottleneck`).
- **Seeds:** SD HDMI = seed 2 (seed 1 stalled the router); the other five are seed 1.
- **Fmax (post-route):** HD Analog 79.55, HD HDMI 86.23, HD Dual 78.15, SD Analog 75.96, SD HDMI 86.53,
  SD Dual 90.21 MHz.
- **Fit:** 7377–7409 / 7680 LC (≈ 96 %), 17 / 32 EBR. Every bitstream passed iceunpack.
- **Artifact:** `out/rev_b/ron/heirloom/bottleneck.vmprog` (480,848 bytes, unsigned). **Not flashed.**
- router1 stalled seed 1 on the HDMI configs in an earlier build of a near-identical netlist (seed 2
  closed both). If a rebuild hangs, let the retry run, or try router2.
- RTL verified in GHDL at 720p decimation 1:
  - Osc and Video at defaults;
  - a motion capture at full Speed: 3.7 % of pixels change in two fields (PWM crawl, stair and block
    slides, sway).
  - RTL vs the integer model: residual = noise realisation and Osc phase only.

## Deviations from the photo
- **LC ≈ 96 %**, above the 92 % target (timing still closes on every HD config).
- **Rims.** One fitted profile per edge side × fill type (lime / pink / lower pink). The photo's rims
  vary row by row and are dotted where the bright stripe rows cross them in the lower half.
- **Stripe bend dropped.** The ~50-line downward bend of the stripes along the left bottle's right side
  (y 220–700) is too costly for this fit.
- **One glitch window per line.** On rows 440–530 the middle dark block wins, so the right-neck
  displacement (x ≥ 1440) is missing there.
- **Tables.** The stripe tables are a 64-colour palette at 2-line resolution (mean quantisation error
  ~3 / 256). The edge curves are sampled every 8 lines.
- **Noise.** The PWM line noise and the fill texture are a different realisation (LFSRs).
- **Brightness.** The top-right quarter renders slightly brighter than the photo (region mean 90,70,78 vs
  67,46,89).
- **SD.** SD shows the centre 4:3 (virtual x 240–1680), so the right bottle's right flank (to x 1741)
  and the frame-edge stripe zones are cropped. 720p matches 1080p.

## Working files
`.heirloom_work/rb_bottleneck/`:
- `notes.md`: measurements and history.
- `geom.py`: edge control points.
- `tables.py`, `tabs2.npy`, `bgmodel.py`: stripe tables and halo.
- `decyuv.py`, `dec.npz`: rim fit.
- `mktables.py`, `tables.npz`, `mkhook.py`: build-hook tables.
- `proto7.py`: proto_final.png.
- `imodel.py`: integer model of the RTL.
- `sheet.png`: control sheet.
- `cmp_proto.png`, `cmp_final.png`, `cmp_motion.png`, `cmp_vid.png`.
