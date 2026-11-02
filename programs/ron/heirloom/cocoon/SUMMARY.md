# Cocoon — a woven cocoon in a diamond glow, torn video banded across it (Heirloom #2)

**v0.6.0 (2026-10-31, phase 2): the arcs modulate the weave, K4 fades the red box, P12 grows the
band from nothing, S9 Slats. Built, NOT flashed.** (v0.5.0, 2026-10-21: rebuilt from the photo.)
Source photo: `photos/WIN_20220829_23_30_37_Pro.jpg`, a 1920×1080 capture from 2022-08-29.

v0.1/v0.2 were a generic "hexagon of chevrons" sketch, and Ron judged them too abstract. v0.5.0 was
rebuilt in four steps:
- measured from the photo;
- prototyped in float to a near-replica (`cmp_proto.png`);
- modelled in integers (`imodel.py`, the RTL spec);
- written as RTL and checked in GHDL against the model (`cmp_final.png`).

Review fixes after the first build (weave contrast, woven wing arcs, rounded and faded right
corners) were measured against the photo and are included.

Working files are in `.heirloom_work/rb_cocoon/`:
- `notes.md`;
- the `m*.py` measurement scripts;
- `cproto.py`, the float proto, and `sheet.png`, its control sheet;
- `imodel.py`;
- the `cmp_*.png` comparisons.

## Reading of the photo (measured)
- **Flanks.** Light grey (201,202,200) camera video lies at x < 340 and x > 1585 (the 50 % points).
  It is posterized into olive (100,72,9), navy (4,1,121), dark green (25,63,34) and purple blobs.
  The window edge fades through dark:
  - grey 187 at x 306, 84 at 338, 9 at 370;
  - then navy, rising from B 64 at 386 to 112 at 440.
- **Glow.** A diamond, `d = |x − 984| + 1.53 |y − 554|`, fitted to R with an rms error of 8.6.
  Its colours by d:
  - red (218,0,0) up to d 560;
  - maroon (162,0,16) at 760;
  - navy (0,0,100) from 1100.
- **Band.**
  - It covers y 318–812 at full width.
  - Its edges are 3-line dark rules with pale dots every 8 px.
  - The video inside is colourized vividly: green (52,177,4), yellow-green (128,188,7), olive,
    orange (221,97,43), pink (237,102,118) and grey.
  - It is torn sideways in held runs.
- **Cocoon outline.**
  - The shoulder is a diamond edge, `|dy| = 0.785 (x − 538)` about y 562. It matches the measured
    top edge to ±5 px over x 880–1100.
  - It rounds into flat tops at |dy| < 416.
  - The right end is rounded: the crimson ring reaches (1490, dy 162), and the green arc turns
    vertical near x 1560.
  - The cocoon is drawn over the band, and the band's rules stop at the shoulders.
- **Colour field.** A trapezoid, `F = |y − 562| + 0.6·max(0, 1050 − x) + 0.8·max(0, x − 1200)`.
  - It was fitted by bin purity on a weave-free image (a 79×21 box filter). It scores 550,
    against 784 for a kite and 942 for a parabola.
  - The colours along F:
    - pink-red below 100;
    - orange at 250;
    - yellow-orange at 300;
    - the crimson ring at 355–425;
    - olive at 430;
    - a green arc at 460;
    - grey-green from 560.
- **Weave contrast.** The luma was split into the darkest 30 % and the brightest 40 % in x
  760–1240, y 380–700.
  - In the core the dashes are dark crimson, Y 72 (221,3,26), on a bright ground of Y 131
    (245,89,45); the luma std is 26.
  - In the outer weave they are Y 74 (221,12,5) on Y 138 (236,111,20).
  - The 78 px horizontal oscillator gives an autocorrelation of 0.61. Rows of dashes, about
    40 × 12 px, repeat every 22 lines, each row shifted about half a period.
- **Wing arcs.** These are contours of F about 30 units apart. They alternate yellow, red and
  green, and each one is finely zigzagged.

## What it does
1. **Glow.** d·g is looked up in a 256-entry ROM, where g is set by K4. The |x − 984|·g term comes
   from a signed DDA on a late accumulator; the 1.53 |dy|·g term is computed per line.
2. **Field.** The field is `F = |dy|/s + G(1125 + (x + jitter − 1125)/s) + zigzag`.
   - s is the P12 cocoon size.
   - G is a 1024-entry ROM, read through a scaled-x DDA:
     `G = 0.6 soft(1050 − x) + 0.8 soft(x − 1200) + 0.0217 max(0, x − 1462)²`. The last term is
     the rounded right end.
   - The zigzag is `tri(4 × weave phase)/32`: ±4 F-units at a 19.5 px period. It moves with the
     weave.
   - With S11 on, the source luma adds ±48.
3. **Arcs.** `tri((F·inca − φa)/2^16)`, quantised to three levels.
4. **Weave** (band rows only).
   - Rows run along `(y − 562 + φr)/py`.
   - Odd rows shift the 78 px phase by half a period.
   - A slight zigzag follows the x oscillator.
   - A dash is drawn where `tri(x/px + par/2 + 1/4) > −0.16 + F/683`; a gap is drawn 0.63 below
     that threshold.
5. **Colourizer.** A 1024-entry ROM addressed by `bank & F(9:4) & level`.
   - It has five levels (dash, arc+, mid, arc−, gap), each fitted per F bin from the photo's
     dark/mid/bright luma split.
   - The right bank holds the band core, the wing arcs (red / orange / yellow-green), the crimson
     ring, the green arc and grey-green.
   - The left bank, used inside the band left of x 1050, runs yellow → yellow-green.
   - ROM luma is pre-scaled by 16/13, so the output's luma scaling (the bar fix) shows the fitted
     colours.
6. **Key.** A pixel is inside when |dy| < 416 s and x lies past the shoulder. Outside the band the
   cocoon takes the window gain, so it darkens into the flank video and there is no hard cut.
7. **Video.**
   - One line of luma, `(Y − 64)·0.281`, is stored and torn on the band rows.
   - It is posterized to 32 levels: the flank palette outside the band, the vivid palette inside
     it.
   - In the band the video shows where `luma > 0.30 + 0.62·ramp(x)`. The ramp runs 330→630 and
     1420→1580; it mirrors with S7.
8. **Window.** An x-profile ROM gives the flank fade, the window gain (0..16/16) and the band ramps.
9. **Band rules.** They are dimmed to 3/8, with pale-pink dots on the centre lines.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Weave | the dashes: period 36..170 px, square law (78 px = the photo at 56 %); the arcs' zigzag follows it |
| K2 Rows | the dash rows: pitch 48..10 lines (22 lines = the photo at 56 %) |
| K3 Arcs | the wing arcs: spacing 90..12 F-units (30 = the photo at 56 %) |
| K4 Glow | the red box and its glow, and the arcs' hold on the weave. 0 %: the box shows the camera (flank palette); 25 %: black; 50 %: the glow fully in; 67 %: the photo; 100 %: red floods. The glow scale runs 1.6..0.7 over the whole travel (1.0 at 67 %). The arc -> weave modulation is a cube law, none below 40 %, a little at the photo (+-34), full at 100 % (+-127: rows ripple half a row, dashes swell solid / vanish) |
| K5 Tear | the band's torn video (held offsets σ 0..40 px, runs of 2..11 lines, 1 run in 5 ×4) and the texture's line jitter (σ 0..7 px); 50 % = the photo |
| K6 Speed | bipolar, still within ±8 of the centre: dashes and zigzag slide, rows scroll, arcs stream out of the core; the default is +33 % |
| S7 Mirror | the cocoon points right; the camera is not mirrored |
| S8 Palette | Fire (the photo), or Ice: U and V exchanged on the generated colours, giving a cyan-blue cocoon in a dark maroon glow |
| S9 Band | Solid (the photo) / Slats: only the core half of each weave row is band, so the cocoon, glow and flank video show between the slats; the slats follow the rows (K2 pitch, K6 scroll, the arc ripple) |
| S10 Source | Video / Osc |
| S11 Bend | the source luma bends the cocoon's field |
| P12 Cocoon | the cocoon's size s and the band, glided 1/8 per field: 0 % a seed and no band, 50 % the photo, 100 % the cocoon bursts and the band fills the screen. The band's half-height is 0..247 lines over 0..50 % and follows s above (to 543) |

## Video
- **Video** (default). The camera is the flanks and the band ends, as in the photo.
  - Its luma is posterized into the photo's two palettes: grey, olive and navy outside the band;
    green, pink and orange, torn, inside it.
  - In the band it is keyed over the weave by brightness, more strongly toward the ends.
  - With S11 Bend on, it also warps the cocoon.
- **Osc.** Three folded triangle oscillators replace the camera:
  - one horizontal, with a vertical wobble;
  - one vertical, with a horizontal wobble;
  - one diagonal.

  Their blobs have similar size and level distribution, and they run through the same posterize,
  tear and key path as the camera. They drift slowly.

## Engine
- **Sequencer.**
  - One serial shift-add multiplier and one serial divider.
  - The operand selects are one-hot, registered every clock from the state number, with a flat
    AND-OR fetch (the wraith/loupe pattern).
  - **Per field** (states 1–34, in the vblank zone):
    - the raster: `ys = 1080·256 / lines`; xs is square in HD and 2 px per pixel in SD;
    - the width normaliser;
    - the knob laws, s and 1/s;
    - band, key and cut;
    - speed phases;
    - DDA steps and starts.
  - **Per line** (states 100–125, kicked at g_start, computing the next line, about 200 clocks):
    - |dy|/s and the glow line term;
    - band and rule flags;
    - the shoulder;
    - xorshift32 tear and jitter;
    - the row phase;
    - the scaled-x start;
    - the oscillator line terms.
- **Glides.** K1..K5 and P12 share one datapath, three clocks per channel, with a ±4 deadband. The
  rails snap.
- **Pixel path (24 stages).**
  - S1–S11: addresses; key and bank; the weave and zigzag phases; F; dashes and arcs; the glow
    distance; the ROM addresses.
  - S12–S13: ROMs, then fabric copies.
  - S14–S16: the layer and Ice; the window gain; the band rules.
  - S17–S24: output conditioning, then the blanking gate. The conditioning is chroma
    [1 2 2 2 2 2 2 2 1]/16 (its 8-clock tap held in an EBR ring), luma [1 2 1] and luma ×13/16.
- **ROMs.** All come from `cocoon.py` → `cocoon_pkg.vhd`, in the hardware colour convention:
  - G, 1024×10;
  - the x profile, 256×16;
  - the colourizer, 3 × 1024×8;
  - the glow, 3 × 256×8;
  - the video, 3 × 64×8.

  The RAMs are the line buffer (2048×8) and the chroma ring.
- **Contracts.**
  - damier's generated avid.
  - One 24-clock latency in all modes.
  - A blanking gate on the latency-aligned generated avid.
  - Every EBR output is re-registered in fabric.
  - `no_rw_check` only on the line buffer and the chroma ring.
- **Fit history.**
  - The v1 RTL came to 9765 LC (127 %).
  - v2 moved G and the x profile into ROMs, used DDAs instead of per-pixel multiplies, moved the
    oscillator to the beam side and shared the glides.
  - v2c added the one-hot operand fetch, the chroma ring and one oscillator time counter.
  - The texture fixes added about 50 LC.

## Phase 2: v0.6.0 (2026-10-31)
Ron's notes on v0.5.0:
- a little more modulation of the weave by the arcs, its strength on K4 Glow with the glow;
- the red box shouldn't always be on: K4 down fades it out and the diamond glows less, K4 up
  fades it in and the diamond glows more;
- S9 repurposed for something more interesting, the band permanent;
- P12 0 % = no band at all .. 100 % = the old 100 % (band fills the screen).

What changed:
- **Arc -> weave modulation.** The arc triangle (S11) is fed back through a 1024 x 8 ROM
  `C_M` = k/15 x tri (address tri(8:3) & k), so it reaches the weave 6..13 px behind the pixel
  it came from (invisible: the arcs vary over tens of pixels). It is added to the weave's row
  phase at S2 (x 256: up to half a row) and to the dash oscillator before the dash / gap
  thresholds at S9, so the dashes swell on the arcs' dark side and thin on the bright side.
  k = K4^3 / 2^26 (per field, two serial multiplies): 0 below 40 %, 4 at the photo, 15 at 100 %.
- **The red box fade.** Per field from K4: below 256 the box behind the cocoon shows the
  camera through the flank palette at gain beta = 16 - K4/16; 256..511 the glow at gain
  alpha = (K4 - 256)/16 (a dissolve through black at 25 %); from 512 alpha = 16. The gain is
  picked per layer class at S13: band 16; cocoon and flank video max(window, beta), so the
  window's dark seam and the cocoon's edge fade vanish as the camera fills the box; box video
  beta; glow min(window, alpha). At the default (alpha 16, beta 0) every gain is v0.5's.
- **P12 band.** The band's half-height is (P12/2) x 247 / 256 below 50 % (0 .. 247), and
  s x 247 / 256 above (247 .. 543) as before. The band flag (rules, dots, gate) is now per field
  `c_bh > 3`, so at 0 % nothing of the band is drawn. The cocoon's size law is unchanged.
- **S9 Slats.** The band test is per pixel from S5: in Slats only where the row fold
  vyf < 64 (the core half of each weave row), so the slats follow the rows' zigzag, pitch (K2),
  scroll (K6) and the arc ripple (K4). The colourizer bank and the weave gate follow the pixel
  test. The band's torn read and edge rules stay per line.

## Status
- **v0.6.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Built with ROUTER1: router2 missed HD HDMI on seeds 1-5 and stalled on 6. Post-route Fmax: HD Analog s1 84.80, SD Analog s1 87.40, HD HDMI s2 86.03, SD HDMI s2 85.15, HD Dual s1 91.69, SD Dual s1 84.60 MHz; 7364-7410 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.6.0, built 2026-10-31, router2. NOT flashed.**
  - 7356–7397 / 7680 LC (95.8–96.3 %, now FULL), 21 / 32 EBR (+2 for `C_M`), iceunpack OK × 6.
  - Builder seeds and post-route Fmax: HD Analog s2 74.40, SD Analog **s2** 74.09 (s1 hit the
    router timeout), HD HDMI s2 74.63, SD HDMI s1 65.04, HD Dual s1 76.45, SD Dual s1 77.47 MHz.
  - **HD seeds re-picked from a router2 pin-skew screen** (HD HDMI s1–24, HD Analog / HD Dual
    s1–12: `.heirloom_work/rb_cocoon/skew/screen.txt`), hand-installed and repacked. HD timing is
    thin: only 4 of the 24 HD HDMI seeds met 74.25.
    - HD HDMI **s11**: 76.20 MHz, HSYNC +1.22 / data ≤ −0.61 ns (the builder's s2 sat at +0.58).
    - HD Analog **s8**: 79.82 MHz, HDMI HSYNC −1.68 / data ≤ −1.46 ns (s2 had a data bit at +0.17).
    - HD Dual **s2**: 81.14 MHz, −2.44 / ≤ −2.18 ns.
  - A plain rebuild gives the builder seeds. To reproduce the shipped HD bitstreams, run
    `skew/synth.sh <config>` then `RT=router2 skew/route.sh <config> <seed>` (s2 / s1 reproduced
    the builder's bins byte-exact). The builder's vmprog is `skew/cocoon_v060_builder.vmprog`.
  - If a rebuild regresses HD: try `NEXTPNR_ROUTER="--router router1"`, then reclaim LC (the
    p_k16 chain, the line engine's n_ / l_ oscillator terms).
  - `out/rev_b/ron/heirloom/cocoon.vmprog` (484,905 bytes). Presets are placeholders (Photo,
    Seed, Hatch, Ice Mirror, Video Box, Slats, Osc Weave, Bent Video).

**v0.5.0:**
- **v0.5.0, built 2026-10-21, router2 (the builder default).**
  - Build seeds: **HD HDMI = seed 4** (seeds 1–3 missed) and **SD Dual = seed 2** (seed 1
    stalled the router). The others used seed 1.
  - Post-route Fmax:

    | Config | Fmax |
    |---|---|
    | HD Analog | 78.14 MHz |
    | HD HDMI | 74.96 MHz |
    | HD Dual | 74.68 MHz |
    | SD Analog | 76.66 MHz |
    | SD HDMI | 74.95 MHz |
    | SD Dual | 72.09 MHz |

    SD Fmin is 27 MHz.
  - 7232–7258 / 7680 LC (94.2–94.5 %), 19/32 EBR.
  - Every bitstream passes `iceunpack`.
- **Not flashed.** The presets are hand-written.
- **HD margins are thin** (HD Dual +0.4 MHz, HD HDMI on seed 4). If a rebuild regresses:
  - try `NEXTPNR_ROUTER="--router router1"` first;
  - then profile.

  LC to reclaim: the window-gain delay chain (p_k16) and the line engine's n_/l_ double
  registers.

## Deviations from the photo, and why
- **Weave.** The weave is a regular brick lattice: rows of dashes, odd rows shifted half a period,
  a slight zigzag, line jitter.
  - The photo's dashes come from a looser analog interference, with irregular shapes, thin dark
    lines between the rows, and rows that wander.
  - Kept: the period, row pitch, stagger, dash size, colours and contrast. The measured core
    contrast is dark Y 70 / bright Y 128, luma std 25.5, against the photo's 72 / 131 / 26.
- **Field.**
  - The fitted trapezoid makes the wing arcs slightly less round than the photo's.
  - The right crimson lobe is thinner than the photo's.
  - The yellow-green notch at the band's right end comes from the video key, not geometry.
- **Osc blobs.** The Osc substitute is a triangle-oscillator plasma, more geometric than the photo's
  camera blobs. Video mode shows the real thing.
- **Quantisation.**
  - The window and band ramps step every 8 px.
  - The window gain steps in 1/16.
  - The colourizer steps every 16 units of F, the photo's posterized look.
  - Whites read about 776 after the bar fix.
- **P12.** P12 scales the whole cocoon interior about x 1125. The look at 50 % is the photo.
- **Resolution (warn).**
  - The cocoon and glow are sized to the screen height.
  - The window, flanks and band ramps scale with the width.
  - On 4:3 SD the flanks are relatively narrower, and the crimson right lobe is partly cut at the
    window.
  - Interlaced fields render identically.
- **Band dots.** The band dots stay at pixel scale: an 8 px period at every resolution.
