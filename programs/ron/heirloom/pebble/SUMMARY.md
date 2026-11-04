# Pebble — red ripples dotted by rows, each crest a window onto a mirrored figure

**v0.1.0 built 2026-11-04, not flashed.** Heirloom phase 3 (video clips), from the clip `WIN_20230731_01_19_22`
at 0.5 s (`photos/vid/WIN_20230731_01_19_22_t0p5.png`). The same clip at 4.5 s and 14.0 s shows the far ends of
Ron's knob.

The still shows:
- five red "(" ripples about 337 px apart, each ramping orange to yellow into a window that shows a
  blue / magenta / green layer;
- a 64-line square wave that turns every other band of 32 rows into navy-to-green ramps (the "dotted" look);
- a mirrored two-copy figure with a yellow-outlined crown of tongues, hanging in from the top.

In the clip the ripples' bulge scrolls down, and Ron lowers the figure by hand until it nearly fills the screen.
Presets are hand-written placeholders. Working files are in `.heirloom_work/rb_pebble/`.

## Reading of the photo (measured)
- **The ripples are not circles.** On every row the red centres are equally spaced (std about 1 px).
  - The spacing is 336.4 px at y = 540, 387 at y = 105 and 395 at y = 1067.
  - Each row's offset follows x0 = Xc + c P(y), so every row shares one ramp origin.
- **Field fit.** `fitfield.py` / `m7.py`, 194 rows, rms 1.2 px. It is a horizontal ramp multiplied by DC plus a
  vertical cosine, into a sawtooth colouriser:
  - u = frac((x − Xc) g(y) + φ0), with g(y) = a + b cos(2π (y − yv) / 1750);
  - a = 0.0026148 per px, b / a = 0.144, yv = 558, Xc = −193 px, φ0 = 0.770;
  - u = 0 on the red flats.
- **Rows.** A vertical square wave of 63.75 lines (32 + 32), with the B rows starting at line 38. The transitions
  run 38 → 1058 in 32 half-periods. Two 2-line phase slips in the still are not modelled.
- **Colouriser** (median RGB per 1/256 of a ripple):
  - **A rows:**
    - red (254, 7, 0) from u 0.9 to 0.1;
    - G rising with a soft knee to 168 by u 0.36;
    - the window from 0.387 to 0.656;
    - G at 134, falling to 7 by u 0.95.
  - **B rows:**
    - red keyed off at u ≈ 0.03 (a hard 25 px edge at the sawtooth's wrap);
    - navy (0, 12, 50) ramping linearly to green (0, 254, 33) by u 0.45;
    - yellow from 0.50 to 0.55;
    - a narrow window from 0.555 to 0.605;
    - yellow-orange, then as the A rows from 0.66.
  - **Shares** (k-means, 12): red 29 %, green 13 %, navy 12 %, oranges 22 %, yellow 10 %, blue 7 %, magenta 5 %.
- **Figure layer** (the window contents):
  - It is mirrored about x = 525 + 960 k.
  - Within |x − axis| < 340 it is blue / violet / magenta / red: blue 18 %, magenta 15 %, violet 15 %, azure 13 %,
    red 11 %, purple 10 %, pink 9 %, plum 5 %.
  - Beyond about 360 px it is green / cyan: green 42 %, mid green 16 %, cyan 15 %, blue 13 %.
  - At 4.5 s the hanging figure shows the same palette: magenta 29 %, blue 18 %, violets 35 %, red 5 %.
  - The figure's yellow side outline sits at |x − axis| 250..330 and wobbles ±25 px.
  - The figure is stepped by the 32-line rows.
- **Crown** (top 0..50 lines): rounded tongues hanging down, about 115 px apart, deepest at |x − axis| ≈ 120 and
  235. The cusps between them reach the top. The outline is about 10 px of yellow.
- **Texture.** Hard row edges, crisp window edges (about 8 px), smooth ramps, no tearing or noise.

## Motion in the clip
- **The bulge scrolls down.** The cosine's vertex moves 506 (t 0.0) → 558 (0.5) → 594 (1.0) → 642 (1.5) → 672
  (2.0) → 826 (4.0) → 862 (4.5) → 954 (6.0) → 1292 (14.0).
  - That is 64–100 lines/s, about 75 lines/s (1.25 lines per field) over the first 2 s.
- **The ripples never flow.** Xc and φ0 stay within −177..−193 and 0.77..0.82 through the whole clip.
- **The rows never roll.**
- **The figure descends.** Its bottom edge is at:
  - 16 lines at 0.5 s, 36 at 1.0 s, 168 at 1.5 s, 168 at 2.0 s;
  - about 350 at 3.0 s, 524 at 4.0 s, 676 at 5.0 s;
  - about 820 at 6.5 s, when its top leaves the frame at 76;
  - gone off the bottom by 14.0 s.

  That averages about 135 lines/s, with stalls and jumps: Ron's hand on a knob, not an LFO.
- **The ramp gain changes.** By 13.5 s Ron had also lowered the ramp gain by 14 % (spacing 390 px). The cosine's
  trough was then mid-screen, so the ripples read ")".
- **In the program:**
  - K4's default is the measured 1.25 lines per field.
  - Nothing flows at the defaults, as in the clip.
  - The figure's descent is the headline P12.

## What it does
1. **Ripples.**
   - u = (x − Xc) g(y) + φ0, plus 0.08 ripple × source luma.
   - g = a + b · wave(y), where the wave is the cosine, or a 4× triangle under Zigzag.
   - g is computed once per line; u runs as a DDA across the line.
   - Under Twin the DDA runs back after the centre pixel.
2. **Rows.** A phase accumulator gives each line's type (A or B). Weave adds half a ripple to the B rows.
3. **Colour.** The FIRE palette is read at {row type, u}. It is replaced by the figure layer inside the window:
   |u − 0.5215| < hwA on A rows, |u − 0.58| < hwB on B rows.
4. **Figure layer.**
   - Mirrored distance m = |((x − 525) mod 960) − 480|, with B rows shifted by 24 px.
   - Palette index p = (y − veil − 0.7 m)/600 + 0.2 tri(m/300) + 0.125 tri((y − veil − 0.8 m)/90)
     + 0.75 · luma + flow.
   - COOL palette for m < side + 90, GREEN beyond, with side = 255 + 22 sin(2π (y − veil)/420).
5. **Veil.**
   - The figure layer covers the frame where y < veil − A (1 − scallop(m)) + 64 · luma and m < side + 60.
   - The scallop is a parabola per 115 px (rounded tongues, pointed cusps); A = 45 + 0.15 (veil − 50).
   - Yellow outline within 11 lines of the edge and on the sides (m ≥ side).
6. **Output.**
   - Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes, then luma 13/16, then the blanking gate.
   - Ice swaps Cb and Cr before the boxes.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Ripples | the ripples' count: ring gain ×1/4 .. ×4, exponential. 512 = the photo's 337 px spacing; 0 = one fat ripple; 1023 = about 20 hairline ripples |
| K2 Bend | the ripples' bow: the vertical cosine's depth, b/a = 1.6 s·\|s\|, ±8 deadband. 0 = reversed and folded into horizontal streaks and lenses; 512 = straight vertical bands; 671 = the photo's "(" (0.144); 1023 = folded into nested closed loops |
| K3 Origin | the drop point: the ramp origin, Xc = −193 + 4 (k − 512) px. 0 = far left, deep even arcs; 512 = the photo; 800 = mid-screen ")(" splash; 1023 = right edge, ")" arcs |
| K4 Speed | the bulge's scroll, bipolar (±8 deadband, square law). 682 = the clip's 1.25 lines per field downward; full travel = 12 lines per field either way. Also sets the figure's chevron flow |
| K5 Inlay | width of the window at each crest. 0 = none (pure fire ripples); 512 = the photo (27 % of a ripple on A rows, 5 % on B rows); 1023 = 96 % / 74 % (the figure layer through nearly the whole ripple, thin fire lines left) |
| K6 Rows | period of the dotted rows: 8 .. 63.75 (512, the photo) .. 510 lines (±3 octaves). Also steps the figure |
| S7 Twin | Off / On: the ripples mirrored about the screen centre (two arc systems meeting in a lens) |
| S8 Palette | Fire (the photo) / Ice (Cb ↔ Cr: deep-blue ripples with cyan crests, maroon-green rows, a red figure) |
| S9 Weave | Off / On: the B rows shifted half a ripple (bricks of red and green) |
| S10 Bend | Arc (the photo's cosine) / Zigzag (a triangle at 4× the rate: zigzag ripples, about 2.5 zigs down the frame) |
| S11 Source | Video / Osc |
| P12 Veil | the figure's descent, glided 1/8 per field. 0 = no figure; 314 = the photo's crown peeking in at the top (50 lines); ~512 = the clip at 1.0..1.5 s (~250 lines); ~800 = the clip at 4.5 s (figures to ~700 lines, tongues 140 deep); 1023 = two full-height figures with wavy yellow outlines, ripples only in the gaps |

Every switch is a whole-frame look flip at the default state of every other control. Every knob stays visible at
P12 = 100 %, in the gaps between the figures and in the figures' row stepping (`sheet_p12max.png`).

## Video
- **Video mode (the default).** The camera's luma enters the patch in three places, as patch cords would. It is
  Y>>2 − 16, through a 4-pixel box and a one-line buffer.
  - **Ripple phase:** added at 0.08 ripple per full luma, so the ripple edges bend and fray around the picture.
  - **Figure palette index:** added at 0.75 cycle, so the camera shows in false colour through every crest window,
    and fills the two figures when P12 is up.
  - **Crown edge:** added at 64 lines, so bright parts of the picture drip further down.
  - With no input, Video mode is the photo: the RTL is bit-exact with the integer model.
- **Osc mode.** The shell's 640 × 540 virtual-px triangle lattice, drifting about 75 px/s, feeds the same three
  inputs. The ripples wobble gently, and the windows and crown shift with it. Every source-path control stays
  live.

## Engine
- **Per-field sequencer** (vblank zone):
  - one serial shift-add multiplier (20-bit b operand, 12 slots);
  - the shared glide on K1 / K2 / K3 / K5 / K6 / P12;
  - a 256-entry exp2 ROM for the exponential controls.

  It computes per field:
  - the ring gain a = 21935 · 2^(e/256) per half virtual px (2^24 per ripple), and b = ((a d² >> 16) · 27160) >> 16;
  - the scroll step d4² · 29801 >> 12 (Q28 per field) and the cosine step ys · 599;
  - the rows' krow = 263172 · 2^(−1.5 e/256), with its line step and start phase;
  - veil = (p² · 1174 >> 20) − 60 and the crown depth;
  - the window half-widths, the ramp origin and the figure counter starts.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in about 60 clocks, so it fits SD's
  858-clock line.
  - The cosine comes from a 1024 × 16 ROM, linearly interpolated with an 8-bit fraction. This keeps the bends
    stair-free at full Bend. Under Zigzag it is the triangle instead.
  - On one 16-iteration serial multiplier: g = a ± (b |wave|) >> 15, the DDA step g · xs, and the DDA start
    φ0 ± |g| |x0 − Xc|.
  - It also computes the row type, the side wobble (from the same ROM) and the figure's line terms.
- **Pixel path** (21 stages):
  - the ripple DDA, and the figure counter (mod 1920 half-px) giving m;
  - the m and luma products as two-stage shift-adds, giving the etched phase, the triangles and the crown phase;
  - the window distances and the palette index;
  - the window test, and the crown ROM × depth (9 × 8);
  - the fire and figure ROMs, with their addresses issued late and one fabric register each;
  - the veil flags → compose → Ice swap → two boxes (8-back taps from block-RAM rings) → luma 13/16 → gate.
- **ROMs and RAMs** (21 EBR):
  - fire 512 × 32 and figure 256 × 32: Y, U, V in the hardware convention, luma pre-scaled 16/13, capped at 940;
  - cos 1024 × 16, exp2 256 × 16, crown 256 × 8;
  - the 2048 × 8 line buffer;
  - two 256 × 32 box rings and a 256 × 4 sync ring, all `no_rw_check`.

  All tables come from `pebble.py`.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 20, in every mode (syncs through the ring).
  - The blanking gate.
  - The HD HDMI output conditioning, with the running-sum resync (neutral snap after 8 blank samples).
  - Virtual 1080-line coordinates.
  - U = Cr.
- **Verified.**
  - `check_pkg.sh` is clean.
  - GHDL 720p60, decimation 1, with a black input: **bit-exact with `imodel.py`** (100 % of pixels). The frame
    follows 3 vblank zones of scroll and carries the shell's +4 px output offset.
  - A capture 3 fields later is bit-exact at 6 zones. This confirms the default scroll: the bulge moves down, the
    arcs at the top move right and those at the bottom move left.
  - A frame with Twin + Ice + Weave + Zigzag on, P12 = 800 and K5 = 700 (both mid-glide) is bit-exact with the
    glide modelled.
  - The RTL's palette shares match the still within 1 % per colour.
  - Osc and camera frames are shown in `cmp_final.png`.
  - `dupin.py`: 0 `CARRY I0==I1`.
  - Standalone yosys: one transparency emulation, the line buffer's, as in every shell.

## Status
- **v0.1.0 built 2026-11-04**, builder default router2, **all six configs on seed 1**. Post-route Fmax (the last
  line per config):

  | Config | Fmax |
  |---|---|
  | HD Analog | 93.18 MHz |
  | HD HDMI | 79.85 MHz |
  | HD Dual | 90.50 MHz |
  | SD Analog | 83.00 MHz |
  | SD HDMI | 78.88 MHz |
  | SD Dual | 92.63 MHz |

  6696–6766 / 7680 LC (87–88 %), 21 / 32 EBR. Every bitstream passed iceunpack.
- Output: `out/rev_b/ron/heirloom/pebble.vmprog`. **Not flashed.** No pin-skew seed screen was run.
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh pebble`.
- An earlier netlist, with FF delay lines in the output boxes, came to 7239 LC (94 %). Moving the box taps and the
  syncs into block-RAM rings, and issuing the ROM addresses late, brought it to 6.7 k.

## Deviations
- **The figure layer is a synthetic stand-in.** It is a chevron field through 8-colour cycles fitted to the
  windows' k-means. In the clip it is real material, most likely a mirrored, colourised camera image. Its
  statistics match; its pixels do not. In Video mode the camera drives it.
- **The crown is regular**: a row of rounded tongues, 115 px apart and 45 deep at the photo. The clip's crown is
  irregular and asymmetric. The figure's sides wobble on a single sine.
- **Row edges are exact rows.** The still has 1–2 line blends and two 2-line phase slips.
- **The colouriser is the median over u**, so per-ripple irregularities average out. The window edges are hard and
  get the shell's 9 px blur, where the still's are about 8 px.
- **The brightest yellows clip** at the shell's white cap, about 776 output luma. These are the B-row crests and
  the outline.
- **The picture sits 4 px right of the model** (the shell's convention), and the left 8 px fade in from black.
  The capture borders are not drawn.
- **Video mode frays the ripple edges** at the defaults: the camera's fine texture moves the phase up to 0.04 of a
  ripple around its mean. The picture itself reads through the windows and, with P12 up, inside the figures.
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3 (virtual x 240..1680), so it
  crops the leftmost ripple's outer part and the right figure's outer tongues.
