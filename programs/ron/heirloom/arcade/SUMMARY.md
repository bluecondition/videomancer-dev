# Arcade — textured globes in orange rings, blue ring stripes arching between them, chevrons rising from below

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20230825_00_00_20` at 6.5 s
(`photos/vid/WIN_20230825_00_00_20_t6p5.png`). A row of three globes (two full, two halves at the edges), each a
white/tan/cyan textured disc in an orange-to-red ring; blue ring stripes arch between them and meet in cusps;
orange double chevrons rise between the globes from the bottom; two thin tear lines cross the frame. Presets are
hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_arcade/` (`notes.md`, `meas_circ.py`, `radial.py`, `ringprof.py`,
`legfit.py`, `weavefit.py`, `track.py`, `track2.py`, `tears.py`, `proto.py`, `proto_k.py`, `imodel.py`).
- **Globes = one circle generator on an H fold.** Centres x = 50 / 997 / 1946 (period P = 948 px), y = 64 (only
  the lower halves are on screen). Distance from the nearest centre, through a colouriser with level bands:
  - disc r < 368: texture;
  - ring 368..462, a symmetric hue profile: tan (241,199,126) at 374, amber 380, orange 392-410, red (255,49,0)
    at 418-422, orange 434-452, yellow 458;
  - white rim 462-476, cyan halo 482, then the arches.
- **Arches.** Blue ring stripes of the same field from r 482 to ~795 px, dark lines every 56 px (r = 512, 568,
  624, 680, 736, 792); arcs of neighbouring globes meet in ^ cusps on the midlines (x 523 / 1471, y ~700).
  A broad cyan / blue weave crosses them (at x = 997: cyan 590-650, blue 656-686, cyan 692-734, blue 740-776),
  following rings of the FAR neighbour's distance; lines are dark (17,42,54) under blue, grey (130,155,185) under
  cyan. Plane-wave and single-field fits of the weave were weak (corr < 0.46), so it is drawn as a far-distance
  ring pattern.
- **Chevrons.** Orange double ^ at the midlines, outer edge apex y 692 at x 526; legs nearly straight, |dy/dx|
  1.1-1.25 (half-width 52 at y 760, 122 at 840, 216 at 940, 292 at 1060). Two sub-bands ~78 px (red peaks at
  737 / 815), a navy line at 854-866, then texture whose bands nest as chevrons (~50 px). The straight legs and
  sharp apex fit an H triangle summed with a V ramp; a lens-of-circles fit gave a too-gothic arch.
- **Texture.** White ground, tan pointed lenses in rows between stripes, thin wavy cyan bands, green / dark /
  navy stripe dashes every ~142 px (x = 65, 210 at the left), thin grey outlines; inside a globe the bands follow
  the circle field (period ~90-125 px of r).
- **Palette** (k = 14): white (225,232,230) 15 %, sky (5,153,250) 9.5 %, blue (2,90,237) 9.1 %, orange
  (252,107,1) 8.8 %, amber (243,173,21) 8.5 %, cyan (20,226,247) 8.5 %, pale cyan 7.9 %, tan (234,198,129) 7.6 %,
  steel 6.3 %, olive (85,110,6) 5 %, red (246,32,1) 4.6 %, navy (20,36,60) 3.6 %.
- **Tears.** Thin bright cyan lines at y 556 and 866, with a few rows offset by ~6-10 px.

## Motion in the clip
Measured on all 282 frames (9.46 s, 29.9 fps):
- The globe row rises 62-66 px/s (centre y 405 -> -59 between 1.8 and 8.8 s); its red ring radius grows
  343 -> 409 px (Ron turning a size knob — P12 here).
- The chevron row rises ~90 px/s (apex 1034 at 2.9 s -> 456 at 9.3 s), so the chevrons climb into the arches.
- The tears ride the rows: one at 60-65 px/s (globes), one at ~98 px/s (chevrons).
- The other candidate stills of the clip (0.5 s, 3.5 s) are the same patch earlier in the climb; the sibling clip
  `WIN_20230825_00_05_28` is the same patch with finer arch rings (~28 px) and a finer texture: those are K2 / K5.
- **The program:** the globe row rises 1.06 virtual lines per field (64 px/s at 60 Hz), the chevron row x1.406
  (90 px/s); each row wraps with a 1900-line period at its own tear line (802 lines below the globe centre, 156
  lines above the chevron edge, as in the still), so both tears move with their rows. GHDL: the 720p frames after
  2 fields at the default and at full Speed equal the integer model's (100 % of pixels), and `cmp_motion.png`
  sets the model 30 / 60 / 90 fields on beside the clip's +0.5 / +1.0 / +1.5 s frames.

## What it does
1. **Fold.** u = position in the fold period P (centre circle at x = 996 px); the beam carries u², (P - u)² and
   kch·|x - midline| as second-order / linear DDAs (exact integers; jittered start values come from the line
   engine).
2. **Globe field.** dA² = min-side square + tyA² (row A), sqrt by even-shift normalize to a 10-bit mantissa, a
   1024 x 11 ROM and denormalize.
3. **Zones by dA:** disc (texture) < rd; ring / rim / halo palette (1 px per entry) to rd + 128 px; arches to
   rd + 417 px; texture beyond. rd = 368 px x the P12 size.
4. **Arches:** phase φ = (dA - (rd + 138 px)) / λ (dark line at φ = 0); weave = 3 bits of the far circle's
   squared distance (bit position from λ); a 256-entry 2-D palette (φ5 x weave3) holds the line profile and the
   blue / cyan mix.
5. **Chevrons:** hB = (y - yB) - kch·|x - midline| (S8 Valley: +): band 0..156 px (palette), navy line, texture
   of hB to 600 px.
6. **Texture:** tile ROM (band parity, stripe phase 5 bits, band phase 5 bits) -> 8 colour classes; band phase =
   F x gain (F = dA, or hB inside a chevron) + etch; stripe phase = screen x.
7. **Tears:** at each row's wrap a light-cyan line (2 lines) and 4 rows of ±12 px jitter.
8. **Output:** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Globes | the globe row's spacing: 1 (0) .. 2.03 (512, the still) .. 4 globes across (1023), exponential, in 12 px steps; below 2 globes' width the rings overlap and cusp |
| K2 Arches | the arch ring spacing λ: 140 px (0) .. 56 (512) .. 22 px (1023), exponential (4.4 % steps); the weave grain follows in octaves |
| K3 Chevrons | the chevron slope: 0.51 (0, a wide flat zig-zag) .. 1.2 (512) .. 2.78 (1023, steep spires) |
| K4 Speed | bipolar (±8 deadband, square law): both rows rise (right) or sink (left); 700 = the clip's 64 / 90 px/s; full = 500 / 700 px/s |
| K5 Texture | texture scale (band and stripe periods together): x0.4 (0, the sibling clip's fine grain) .. x1 (512) .. x2.5 (1023, coarse blobs) |
| K6 Etch | how far the camera (or the Osc) bends the texture bands (and a quarter as much the arch rings): 0 .. 6 texture cycles at full luma, square law; 520 = 1.5 |
| S7 Globe | Texture (the still) / Rings: the arch rings and weave run on into the globes (blue targets in orange rings) |
| S8 Chevrons | Peak (the still) / Valley: the chevron field inverted, V's hang between the globes |
| S9 Weave | On (the still) / Off: plain deep-blue arches with dark rings |
| S10 Palette | Photo / Neon: hue turned 90 degrees (magenta rings and chevrons, green arches) |
| S11 Source | Video / Osc |
| P12 Globe Size | the globe radius: 0.35x (0 %, small beads with thick rings in a sea of arches) .. 1x (50 %, the still) .. 2x (100 %, giant globes whose rings sweep the frame and cusp at the midlines), glided |

## Video
- **Video mode (default).** Camera luma (8 bits, a one-line buffer) times the Etch gain is added to the texture
  band phase (and a quarter of it to the arch ring phase), so the globes' textures and the outer texture curl
  around the picture while rings, arches and chevrons keep the composition. With no input it is the still.
- **Osc mode.** The shell's drifting 640 x 540 virtual-px triangle lattice feeds the same path, so the textures
  wobble slowly; K6 stays live in both modes.

## Engine
- **Per field** (vblank zone): a shared glide (K1, K2, K3, K5, K6, P12); one serial shift-add multiplier, 10 slots
  (n = 79·2^e, P², kch, ks, kch·P/2, the stripe origin, speed², vA, etch², etch gain); exp2-ROM lookups for the
  gains, coded as a 5-bit mantissa and a shift (arch λ, texture A / B), the weave bit position, the size ROM (rd),
  the fold start u0 (modulo by repeated add), row motion.
- **Per line** (kicked at each line start, computes the NEXT line in ~57 clocks): both rows' wrapped vertical
  terms, tear count and jitter, |tyA|², and the beam start values for the jittered start u (u², (P-u)², kch·w)
  on one 12-iteration multiplier.
- **Pixel path (24 stages):** 1 squares + chevron field; 2 normalize, weave bits; 3 mantissa; 4 sqrt ROM; 5 fabric;
  6 denormalize; 7 zones and operands; 8 mantissa products; 9 gain shifts; 10 etch, zone palette address;
  11 tile address; 12 tile ROM; 13 fabric; 14 palette address; 15 palette ROM; 16 compose (8-bit); 17-20 two
  box filters (8-back taps in block-RAM rings); 21 luma 13/16; 22-23 pad; 24 gate.
- **Block RAM (19):** line buffer 2048 x 8 (4), sqrt 1024 x 11 (3), tile 2048 x 3 (2), palette 512 x 24 (3), exp2,
  size, box rings 256 x 32 (2 + 2), sync ring 256 x 4 (1). The rings carry `no_rw_check`.
- **Contracts:** generated avid; one latency (24) in every mode; virtual 1080-line coordinates (SD shows the centre
  4:3); blanking gate; output conditioning; running-sum resync (boxes snap to black after 8 blank samples);
  colours in the hardware convention (U = Cr).
- **Verified (GHDL 720p60, decimation 1):** default Speed-still frame, default frame (2 fields of motion) and
  full-Speed frame are bit-exact with `imodel.py` (100 % of pixels; output 4 px right of the model = the avid
  offset).
- **Fit history:** the first RTL (second sqrt for the weave, ring width scaled by a multiply, 11-bit gain
  multiplies, a globe warp of the stripes, FF delay lines) packed to 8,807 LC (114 %); the cuts above brought it
  to 6,881.

## Status
- **v0.1.0 built 2026-11-09, not flashed.** Build: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh arcade`
  (router2, the builder default). **Build seeds: SD Dual = seed 2** (seed 1 stalled the router); the other five
  are seed 1.
- Fmax (post-route, last line): HD Analog 86.48, HD HDMI 89.37, HD Dual 92.22, SD Analog 83.15, SD HDMI 87.12,
  SD Dual 88.74 MHz. 6,866–6,891 / 7,680 LC (89.7 %), 19 / 32 EBR. iceunpack OK ×6.
- Output: `out/rev_b/ron/heirloom/arcade.vmprog`. The packed config carries the labels and the 8 presets.
- If a later version gets tight, router1 (`NEXTPNR_ROUTER="--router router1"`) before restructuring; HD margins
  are 12–18 MHz now.

## Deviations from the photo
- **Whites are grey-ish:** the shell's 13/16 luma scale caps drawn whites at ~777 (the photo's white texture
  ground is Y ~850).
- **Texture is a regular tile.** The 32 × 32 tile (tan pointed lenses, thin wavy cyan bands, dashed green / dark /
  navy stripes, grey outlines) reproduces the photo's vocabulary and periods, not its irregular, camera-blurred
  blobs. The stripes inside the globes run straight: the globe warp (stripes converging toward each globe's
  bottom, as in the photo's middle globe) was cut to fit (it cost a ROM, a late beam and a multiply, ~250 LC).
- **The weave** is 3 bits of the far circle's squared distance: broad cyan / blue bands crossing the arches like
  the photo's, but regular, and its grain follows Arches only in octaves. Arch spacing steps by 4.4 % (5-bit gain
  mantissa); the default is 56.9 px (photo 56).
- **Ring width is fixed** (the 128 px ring / rim / halo band does not scale with Globe Size), so small globes get
  thick rings and giant globes thin ones.
- **Chevrons** have straight legs (the photo's steepen slightly downward) and both apexes sit at y 692 (the
  photo's right chevron has a thin tip ~60 px higher). Inside a chevron the texture follows the chevron field to
  600 px below its edge, then the globe-field texture resumes (a second parallel ^ edge once the row has climbed).
- **Tears** are regular: a 2-line light-cyan line plus 4 rows of ±12 px jitter at each row's wrap. The photo's
  lines are thinner with irregular offsets. At SD the jitter rows look thicker (one SD field line spans 4.45
  virtual lines).
- **Motion:** the clip's slow globe growth (Ron's knob) is P12, not automated; the upper V chevrons of the clip's
  first seconds (alt still t0p5) are not modelled. At 50 Hz the rows move 5/6 as fast.
- **Edges:** crisper than the camera capture (two 9-tap boxes); the left 8–12 px fade in from black (filter
  warm-up plus the 4 px generated-avid offset).
- **Resolutions:** 720p shows the same geometry as 1080p; SD shows the centre 4:3 (1,440 virtual px: one full globe
  and two partial), round globes.
