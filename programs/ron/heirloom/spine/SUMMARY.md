# Spine — a magenta band torn diagonally through a ring lattice, its edges hooked teeth with lit rims

**v0.1.0 built 2026-11-09, not flashed.** Heirloom phase 3 (video clips), from the clip `WIN_20230906_00_33_30`
at 11.0 s (`photos/vid/WIN_20230906_00_33_30_t11p0.png`). The same patch appears at 0.5 s of that clip and in the
clips `WIN_20230906_00_33_43` (7.0 s) and `WIN_20230906_00_34_47` (10.5 s).

The still shows:
- a smooth vertical gradient, magenta at the top to dark plum below the middle and back to violet at the bottom;
- a diagonal band of that gradient cut through a texture of contour rings: a purple "O X O X" lattice along the
  top, green wavy lines below;
- both edges of the band are staircases of hooks (the spine): pointed teeth and round bays, each tooth's left flank
  lit by a bright magenta rim, a slewed dark-violet shadow trailing right of it. Where the band narrows at the lower
  right, the bays close into vertebra holes.

In the clip the whole patch rolls up about 30 lines/s, its oscillators at slightly different rates. Presets are
hand-written placeholders. Working files are in `.heirloom_work/rb_spine/`.

## Reading of the photo (measured)
- **The key is one field.** Texture where K > 0, with
  K = |x − 1.25 y + 350| − 645 + 200 hk(x) + 160 cos(2π (y − 378) / 700)  (virtual px).
  - A folded diagonal ramp (H ramp minus V ramp, full-wave rectified) gives the band: centre line
    x = 325 + 1.25 (y − 540), half-width 645 px. The fold puts texture on both sides, so the band has two spines:
    the main one (teeth pointing down, tips at (700, 545) (935, 630) (1180, 745)) and the bottom-left one (teeth
    pointing up, tips at (30, 848) (245, 895) (465, 975)).
  - An H-rate wave of period 230 px makes the teeth; a V-rate cosine of period 700 lines opens the bays into holes
    along its minima row (the closed oval at (1730, 678..895)).
  - Fitted by logistic regression of the texture mask on these terms, grid-searched over slope, offset and both
    periods (`fit5.py`): 94.0 % pixel agreement. The tooth shape is ¼ cosine + ¾ inverted rectified sine (round
    bays, pointed teeth; 93.8 %).
- **Texture rings.** The texture is a cyclic colouriser of G = cos(2π (x − 26) / 230) + 9 cos(2π (y − 155) / 1250):
  - the O rows sit on the V maxima (O's every 230 px, X saddles between, 2–3 rings per O);
  - between rows the contours are wavy lines, 23 px apart (vertical autocorrelation peak at 23), amplitude about
    ±17 px;
  - purple on the O rows (y 0..330), green between; dark lines between the rings.
  - Texture k-means: purple family 55 % ((166, 0, 242) (122, 1, 214) (102, 4, 168) (83, 13, 115) (48, 13, 65)),
    green 30 % ((8, 99, 0) (7, 66, 1) (46, 62, 6)), dark (29, 28, 18), plum (73, 37, 54).
- **Background.** Independent of x: magenta (255, 0, 246) from y 40 to 420, (195, 0, 249) at 500,
  (123, 2, 179) at 600, plum (90, 25, 90) at 700–800, back to (155, 0, 248) at 960 and (209, 0, 249) at 1040.
- **Rim and shadow.** Scanning right, every bg → texture edge has a magenta (255, 0, 248) rim about 15–25 px wide
  on the bg side; every texture → bg edge has a dark-violet trail ((82, 17, 84) → (97, 4, 210) → bg over 50–75 px).
  Under each tooth tip the trail is a horizontal streak (the hooks' "flat floors"), and its decay edge is serrated
  (8-line period, ±6 px).
- **Line pairs.** Slanted edges step in pairs of lines (an interlaced source captured progressive).

## Motion in the clip
- Cross-correlation between the m-frames (0.5 s apart): everything rolls **up**, with no horizontal motion:
  - the lattice 32–36 px/s, the green lines 28 px/s, the hooks 24 px/s, the bottom spine 16–20 px/s, the
    background gradient 36–40 px/s.
- Over the clip the composition evolves accordingly. The holes row is at y ≈ 210 at 0.5 s and ≈ 800 at 11.0 s,
  and 9 s later (the next clip, 7.0 s) ≈ 500, consistent with 30 px/s. At 0.5 s the holes rolled at about 80 px/s
  (Ron's rate knob was higher at the start of the clip).
- **In the program:** at the default Speed the key's V wave rolls up 26 px/s, the rings' V wave 32 px/s, the
  background 38 px/s and the band 20 px/s. The integer model's default frames move 16 lines per 0.5 s, against the
  clip's 12–18 (`cmp_motion.png`).

## What it does
1. **Key** (h = half virtual px): K = tri(x − xc(y)) − hw + Ah hk(x) + Av cos(φk(y)) + etch.
   - tri is |·| folded every 4096 virtual px, so the band can roll forever.
   - xc = 325 + s (y − 540) + roll.
   - hk is the tooth wave, from a 256-entry ROM.
   - Texture where K > 0 (S9 inverts it).
2. **The key runs 28 pixels ahead of the colour.** The key beam starts at the line start; the colour beam starts 28
   clocks later. A 29-bit shift register of key bits then holds every pixel's future, which gives:
   - **rim:** texture 1–22 virtual px ahead → magenta;
   - **shadow:** distance since the last texture pixel, jittered ±8 px per line pair → a dark-violet blend
     (16-step table).
3. **Rings:** r = nt (cos(φx) + 9 cos(φt(y))) + etch, mod 2 rings, into a 512-entry palette (8 zones × ring A/B ×
   32 phases): purple on the O rows, green between.
4. **Background:** a 256-entry palette over a V wave of 1100 lines, one colour per line.
5. **Line pairs:** all V terms use the virtual line rounded down to even.
6. **Output.** Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes, then luma 13/16, then the blanking gate.
   S8 complements the chroma just before the boxes.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Teeth | the H oscillator: teeth / vertebrae and lattice O's across, ×¼ .. ×4 (512 = the photo's 230 px). 0 = one broad wave, a single curved edge; 1023 = a comb of ~33 fine teeth with fine lattice |
| K2 Band | the band's width: 150 px (0: a chain of separate vertebra holes) .. 645 px (512, the photo) .. 1400 px (1023: the gradient fills the frame, teeth only in the corners) |
| K3 Slant | the band's angle about (325, 540): s = −1.0 (0: leaning back) .. 0 (vertical, ~228) .. 1.25 (512, the photo) .. 3.5 (1023: nearly flat, a horizontal spine) |
| K4 Speed | bipolar roll, ±8 deadband, square law: up (+) / down (−); 700 = the clip's ~30 lines/s; full = ~8× either way |
| K5 Rings | the texture's ring density, ×¼ .. ×4 (512 = the photo's 23-px lines). 0 = a few broad purple / green bands; 1023 = fine moiré |
| K6 Etch | how far the camera (or the Osc) bends the band edges and the rings, linear: 0 .. 437 px and 6.8 rings per full luma (300 = 128 px, 2 rings) |
| S7 Mirror | Off / On: the picture mirrored about the centre (two spines meeting in a V) |
| S8 Palette | Magenta (the photo) / Verdant (chroma complement: a green field, magenta rims, green / purple lattice swapped) |
| S9 Invert | Off / On: the key inverted, texture inside the band and the gradient outside (the rims follow) |
| S10 Tear | Off / On: band tears, 72-virtual-line bands each shifted up to ±128 px, redrawn every 0.75 s |
| S11 Source | Video / Osc |
| P12 Vertebrae | the teeth's depth (both waves of the key), glided: 0 = a clean straight diagonal band with straight rims; 410 = the photo; 1023 = 3×, the band shattered into a field of rimmed holes and islands across the frame |

S7–S10 are whole-frame look flips at the default state of every other control (`sheet.png`, rendered from the
integer model). S11 only swaps the source: with no camera, Osc makes the band edges wobble and notch and warps the
O's, a clear change but not a new picture. Every knob stays visible at P12 = 0 (the rings, the band and the rims
remain) and at 100 %.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 through a 4-pixel box and a one-line buffer, enters
  the patch in two places, as patch cords would:
  - **the key:** added at up to 437 px per full luma, so the spine's edges, holes and rims bend and fray around the
    picture;
  - **the rings:** added at up to 6.8 rings per full luma, so the camera shows as warped contour rings in the
    lattice.
  - At the default Etch the composition still reads clearly (`cmp_final.png`, bottom panel). With no input, Video
    mode is the photo: the RTL is bit-exact with the integer model.
- **Osc mode.** The shell's drifting triangle lattice, at 320 × 270 virtual px here, feeds the same two inputs: the
  band edges wobble and notch and the O's warp. Every source-path control stays live.

## Engine
- **Per-field sequencer** (vblank zone):
  - one serial shift-add multiplier (10 slots);
  - the shared glide on K1 / K2 / K3 / K5 / K6 / P12;
  - a 256-entry exp2 ROM for the exponential controls.

  It computes per field:
  - KX = 36472 · 2^(e/256), the H phase per h (2^24 per period), and the pixel step;
  - the ring density Nt = 246 · 2^(e/256) and its V amplitude 9 Nt;
  - the tooth depth (P12, two-slope) and the key V amplitude 1.25× it;
  - the band half-width (K2, two-slope) and slope (K3);
  - speed² and the four roll steps (key V, ring V, background, band in h = 2 s · lines);
  - the tear seed every 45 fields.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in about 121 clocks (it fits SD's
  858-clock line) on one 16-iteration serial multiplier:
  - three V phases, each y · K + roll, into a 1024 × 16 cosine ROM: the key term Av cos − hw, the ring term
    9 Nt cos and its zone, and the background phase into its ROM with the shadow delta;
  - the band start (xc from s · (y − 540) + roll) and the H phase start, both offset by the tear;
  - the tear band LFSR (one step per 72 virtual lines) and the shadow jitter (one step per line pair).
- **Pixel path** (C_LAT 48 = the colour beam's 28-clock delay + 20 stages):
  - the band DDA (13 bits, mod 8192 h) and two H-phase DDAs (key and colour, 24 bits; both reverse at the centre
    under Mirror);
  - the fold, the tooth and cosine ROMs (fabric copies), and the products Ah·hk, Nt·cos and luma·Etch, each in its
    own stage;
  - the key and ring sums; the key bit into the look-ahead register;
  - the rim priority encoder and the shadow counter → the shadow table → Δ·α;
  - the texture ROM (address issued late, one fabric register) → compose → complement → two boxes (8-back taps from
    block-RAM rings) → luma 13/16 → gate.
- **ROMs and RAMs** (22 EBR): cos 1024 × 16, exp2 256 × 16, background 256 × 32, tooth 256 × 8, ring cos 256 × 8,
  texture palette 512 × 32, the 2048 × 8 line buffer, two 256 × 32 box rings and a 256 × 4 sync ring (`no_rw_check`
  on the rings). All tables come from `spine.py`; colours in the hardware convention, luma pre-scaled 16/13.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 48, in every mode (syncs through the ring).
  - The blanking gate.
  - The HD HDMI output conditioning, with the running-sum resync.
  - Virtual 1080-line coordinates.
  - U = Cr.
- **Verified.**
  - `check_pkg.sh` is clean.
  - GHDL 720p60, decimation 1, black input: the shipped RTL is **bit-exact with `imodel.py`** (100 % of pixels, the
    shell's +4 px offset) after 5 vblank zones of roll. An earlier netlist was bit-exact after 3 zones too. This
    confirms the roll arithmetic, the look-ahead rim, the shadow, the palette indexing and the boxes.
  - Osc and camera frames are shown in `cmp_final.png`.
  - Integer model vs the still: mean |ΔRGB| 25 at 720p. The 4 × 4 region means are within 1–13 levels except the
    bottom row and the lower right (17–32), where the bottom spine and the vertical bays differ. Shares
    magenta / purple / green / dark: still 16 / 44 / 15 / 10 %, RTL 14 / 46 / 22 / 6 %.
  - `dupin.py`: 0 `CARRY I0==I1`. Standalone yosys: one transparency emulation, the line buffer's, as in every
    shell.

## Status
- **v0.1.0 built 2026-11-09 with ROUTER1, all six configs on seed 1.** Post-route Fmax (the builder's routed value
  per config):

  | Config | Fmax |
  |---|---|
  | HD Analog | 83.91 MHz |
  | HD HDMI | 83.20 MHz |
  | HD Dual | 92.05 MHz |
  | SD Analog | 96.72 MHz |
  | SD HDMI | 89.17 MHz |
  | SD Dual | 91.63 MHz |

  6930–6944 / 7680 LC (90 %), 22 / 32 EBR. Every bitstream passed iceunpack, and the packed config carries the new
  labels.
- Output: `out/rev_b/ron/heirloom/spine.vmprog`. **Not flashed.** No pin-skew seed screen was run.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh spine`.
- **Build history.**
  - The first netlist (blended rim, a squared Etch on its own multiplier, 10-bit ring cosine, a 28-bit start delay)
    came to 7402 LC (96 %). Its HD configs passed only on seed 2 (router2); the critical path was the rim/shadow
    select feeding the Δ·α multiplier in one stage.
  - Making the rim a solid one-shot, Etch linear on one shared product, the ring cosine 8-bit, the texture address
    issued late and the start delay a counter brought it to 6.93 k LC.
  - On router2 that netlist passed every HD config on seed 1 (86–87 MHz), but SD Dual stalled the router on seed 1.
    The final build therefore used router1 (dupin: 0 self-add carries), which closed all six on seed 1.

## Deviations
- **The teeth do not lean.** In the still each hook leans right (a long rim descending to a short rise). The fitted
  field has no shear: a 0.2 shear term scored worse against the mask (89.8 % against 93.9 %). The lit-from-the-left
  rim and shadow reproduce most of the lean's look.
- **The lower right differs.** The still has two vertical bays running to the bottom plus a closed oval; the model
  has one bay and a smaller hole there. The field is fitted to the whole frame (94 %), not to that corner.
- **The rings are crisper than the still's.** The still is a camera capture of a monitor: soft, with fine
  horizontal comb dashes in the lattice and a faint line-pair texture in the background. The program draws clean
  rings and the shell's 9-px horizontal blur. The interlace comb and the camera moiré are not drawn; only the
  line-pair stepping is.
- **The rim is a solid one-shot** (1–22 virtual px before the texture), softened only by the output boxes. The still's
  rim has a soft gradient profile; a blended rim cost about 150 LC, so it was cut.
- **The shadow serration is a per-line-pair jitter** of the trail's length (±8 px), redrawn every 0.75 s with the
  tear seed. The still's serration changes every frame.
- **The rings' camera luma is read 28 px ahead** of the ring pixel (the key's look-ahead position). This is
  invisible.
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3 (virtual x 240..1680), which crops
  the left edge of the magenta gradient and the right end of the lattice; the rim and shadow widths are in virtual
  px, so they read the same.
