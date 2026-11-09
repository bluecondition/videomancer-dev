# Coral — a red bulb and a chevron eye in fans of red and cyan coral rings

**v0.1.1 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230905_00_44_28_t0p5.png` (clip WIN_20230905_00_44_28, frame 15 at 0.5 s). The 17 s clip, and clip
WIN_20230905_00_44_50, is one patch cycling with an 8.9 s period, with no knob turned. The still shows a solid red
hexagon bulb at the bottom centre, a solid red "v" bar over it whose ends hook down round the bulb, a solid red
triangle above, an eye of red chevrons, and fans of red/cyan arcs sweeping down from the upper corners. The ground is
violet-pink with a cyan sky, and everything is mirrored left-right. Fine diagonal ripples cross the picture, held in
bricks off the red, and the line starts are held. Presets are hand-written placeholders.

## v0.1.1 (review revision)
The three big solid-red shapes now carry the picture. In v0.1.0 they were a red-outlined diamond, thin outlines and
a notched heart.
- **The solid shapes are a table measured on the still** (`shapetab.py`, mirror axis x 1014, strict red
  R > 238, G < 95, B < 60, left and right halves edge-averaged). Each pair of shape rows holds a central run
  [Nc, Wc) and an outer run [Hi, Ho) in virtual px from the centre (512 × 32 ROM). Pixels inside either run are
  solid red.
  - **Triangle:** apex (1010, 385), flat base at y 522, half-width 132.
  - **Hooked bar:** about 40 px thick, top edge (1010, 565) → (590, 520), hooks curling down to about (420, 870).
  - **Hexagon bulb:** top 650–668 (the slight v kept), widest X 348 at y 800, narrowing to 190 at y 1070. Below the
    frame it is extrapolated to a point at y 1356, so the whole bulb shows when the shapes ride up.
- **The shapes ride the rolling sine.** Their rows are translated down by the roll since the still:
  (θ0_still − θ0) × 2215 lines, wrapping every 2215 lines. The bulb sinks off the bottom, comes back from the top
  in the bulb-top phase, and sweeps down through the middle, as in the clip.
- **They widen and narrow** by (K5 − 512) + (g − 639) virtual px, plus the camera term / 8. So K5 = 0 and
  P12 = 0 % still have no solid red, and Swell 100 % is still the red tide.
- **The travelling outer zone** (F > fz) moved up from 1.25 to 2.42 bands at the default, so it no longer draws its
  own heart-shaped bulb at the still's phase. It still floods at Bulb 1023 and Swell 100 %.
- The quarter-sine ROM shrank to 512 entries to make room: 2 EBR, a negligible change to the field.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_coral/`: `notes.md`; `wmap.py` (the palette inversion); `kymo*.npy`,
`outerzone.py`, `fourier.py` (the motion); `runs.py`, `shapetab.py` (the solid shapes); `sw*.png` (the sweeps);
`imodel.py` (the integer model); `sheet.py`.
- **Mirror.** Every frame of both clips is mirror-symmetric, about x 1012–1015 of 1920 (capture px).
- **Colouriser.** One scalar w runs through a 1-D palette: cyan (0, 170, 240) → blue (72, 130, 242) → violet
  (140, 110, 225) → pink (205, 95, 145) → red (255, 45, 0).
  - k-means (k = 12) shares: red 23.5 %, cyan 13.8 %, blue 10.8 %, violet/lavender 20.4 %, pink 10.0 %, the rest
    mixed fringe tones.
  - Luma is nearly constant (hw Y 490–520), except the red (Y 395–460).
  - Every thin red band is a peak of w, with the colours running red → pink → violet → blue → cyan → back between
    bands. That is a triangle wavefolder.
  - The corner fans swing the full red ↔ cyan; the centre stays violet.
- **Solid shapes**, X measured from the centre: the triangle, the hooked "v" bar and the hexagon bulb as listed above.
  The small top spike (|X| < 45, y 0–155) and the chevron (apex (1010, 236)) are fold bands.
- **Corner fans.** Concentric about (50, −200), with pulses about 50 px apart near r 500–700.
- **Textures.**
  - Fine diagonal ripples, 27 px at 45° on the red and 53 px in the ground, held in about 48-px bricks off the red.
  - The left 78 px of every line are held.
  - 2–4 px line jitter on the left-facing red edges.
  - A 1–3 px dark capture border.
- **Fit.** Annealed closed-form fits got stuck. The field kept was hand-tuned against the still and nine phases of
  the cycle:
  `F = 2.5 cos(2π(y − yr)/2215) − 3.5 (1 − cos(π|X|/1000))/2 − 3 max(0, 1 − |X|/420 − |Y|/ry)` bands,
  with ry = 250 / 420 about y 430, and the band phase 0.815.
- **Result.** At 720p the RTL's red share is 28.7 % against the still's 25.6 %; mean |RGB| against the still is
  54.1 (58.3 in v0.1.0).

## Motion in the clip
- **One cycle, no knob.** The clip is one 264-frame cycle, **8.92 s** at 29.61 fps (frame differences and the
  kymographs repeat).
  - The cycle runs: bulb-bottom (the still) → nested diamonds (3.8 s) → bulb-top (7.3 s, the vertical mirror) →
    the bulb sweeping down through the middle (8.2–8.8 s) → bulb-bottom.
- **What moves.** Per pixel, the solid-red run of each cycle is centred at t = 245.8 + 0.1192 (y − 520) frames,
  the same at every x.
  - That is an x-independent vertical sine rolling **down 8.39 lines/frame = 248 lines/s**, wavelength 2215 lines.
  - Band orientation at the sides sweeps from +70° to −70° and back.
- **What the program does with it.** The sine's phase steps (700 − 520) × 174 = 31,320 / 2²⁴ per field at K4 = 700:
  536 fields = 8.94 s, 248 lines/s down.
  - The solid shapes translate with it.
  - The model at −0.5 / +0.5 / +1.0 / +1.5 s moves like the m-frames (`cmp_motion.png`, `cyc_model.png`): the bulb
    and bar sink off the bottom, the chevrons multiply into nested diamonds, the fans straighten, and the bulb comes
    back from the top.
  - The ripples are static (≤ 1 px/frame in the clip).

## What it does
1. **Per line**, the engine computes the NEXT line's values:
   - the diamond row P = D − |y − 430| · ky;
   - the sine phase θ0 + y · 7574, through a 512-entry quarter-sine ROM, gives R = g cos θ;
   - the ground floor 13 + 3·max(cos + 0.3, 0)/1024;
   - a ±2-px tear jitter on about half the lines (an LFSR);
   - the ripple phase at the line start;
   - the shape row yr = y − offset (mod 2215). For 384 ≤ yr < 1408 it reads the shape ROM and latches
     Nc, Wc, Hi and Ho.
2. **Per pixel**:
   - x is held at virtual px 78 and jittered. X = |x − 960|, with an optional Twin fold.
   - Hx = A·HT[X]/256; the diamond key is max(0, P − X·kx).
   - F = R − Hx − diamond + camera, in Q8 bands.
   - G = F × Rings − 0.815. w = 255 − 2·dist(G, nearest band), floored at wf.
   - w = 255 (solid red) where F > fz, or inside a shape run: X − grow < Wc and X ≥ Nc (central), or
     X + grow ≥ Hi and X − grow < Ho (outer). grow = (K5 − 512) + (g − 639) + camera/8.
   - Steps posterises w. A palette ROM (Coral / Lagoon) gives Y / U / V.
   - The ripple is added: a lighter orange on the red, olive / violet bricks elsewhere.
3. **Output.** Two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V with the running-sum resync, luma 13/16 and the
   blanking gate. Palette luma is pre-scaled 16/13 and capped at 768 after the scale.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Rings | The band density, ×0.33 .. ×3 (exponential, glided; 512 = the photo). 0 = a few fat soft bands round the solid shapes; 1023 = dense coral: tight fans and a field of nested chevrons |
| K2 Arcs | The depth of the horizontal cosine (1.2 .. 5.8 bands, photo 3.5): the corner arc fans. 0 = flat horizontal bands; 1023 = steep fans |
| K3 Eye | The depth of the diamond key (0 .. 6 bands, photo 3): the chevron eye above the triangle. 0 = no chevrons; 1023 = a deep stack of nested diamonds |
| K4 Speed | Bipolar roll of the vertical sine, carrying the solid shapes with it (±8 deadband at 512). 700 = the clip (248 lines/s down, 8.9 s cycle); 1023 ≈ 2.8× down; 0 ≈ 2.8× up |
| K5 Bulb | The solid red: shapes narrowed / widened by K5 − 512 px, and the travelling red level (5.6 .. −1.6 bands, photo 2.42). 0 = no solid red, just rings; 1023 = red floods the frame |
| K6 Camera | How far the picture (or the Osc) bends every band and the shapes' edges: 0 .. 3.9 bands per full luma (default 384 = 1.5) |
| S7 Flip | The patch upside down: the bulb hangs from the top, the hooks point up, the tide rolls up |
| S8 Palette | Coral (the photo) / Lagoon (navy, teal, mint, cream-yellow) |
| S9 Twin | Two corals side by side (each half folded about its own centre) |
| S10 Bands | Fold (the photo's soft wavefolder) / Steps (three hard posterised levels) |
| S11 Source | Video / Osc |
| P12 Swell | The rolling sine's amplitude, 0.6 .. 4.5 bands (two-slope; 360 = the photo's 2.5), and the shapes' width with it. 0 % = a calm eye between near-vertical bands, no solid shapes; 100 % = a red tide |

## Video
- **Video mode (the default).** The camera's luma, as (Y >> 2) − 16 from a one-line buffer, times K6, is added to
  F and (/8, in px) to the shapes' widening.
  - Every band bends round the picture, and the solid shapes' edges push out over bright areas.
  - At the default, the bulb, the hooked bar and the triangle read clearly; a busy picture frays the band edges.
  - A black input gives the still's patch: the RTL equals `imodel.py` to 1.42 mean |RGB|.
- **Osc mode.** The shell's drifting 640 × 540 triangle lattice, made zero-mean (−120), feeds the same path. At the
  default the bands bend into lobes and the shapes' edges wobble: an unmistakable flip.

## Engine
- **Vblank-zone sequencer** (zc 30–163).
  - The shared 7-channel glide.
  - Six products on one serial multiplier: swell, arcs, the eye slopes, the ring exponent (and exp2 ROM), speed,
    and the shape offset (Δθ × 2215).
  - The field's sine phase, the outer-zone level and the shapes' widening.
- **Line engine.** Kicked at each g_start (and at zc 1300 for line 0), it finishes the next line in 81 clocks, so it
  fits SD's 858-clock line. Four serial products, a 16-bit LFSR, and the shape-ROM read.
- **Pixel path, 19 stages.** The shape test runs in parallel at stages 5–10 (X − grow and X + grow, four compares)
  and joins the outer-zone flag at stage 11. Then boxes 20–23, luma 24, gate 25. **C_LAT 24.**
- **Block RAM: 23 EBR.**
  - line buffer 2048 × 8 (4);
  - palette 512 × 32 (4);
  - H cosine 1024 × 9 (3);
  - quarter sine 512 × 16 (2);
  - exp2 256 × 16 (1);
  - shapes 512 × 32 (4);
  - output-box rings 4 × 256 × 16 (4, `no_rw_check`);
  - sync ring (1).
- **Contracts.** Generated avid; one latency (24) in every mode; virtual 1080-line coordinates (SD shows the centre
  1440 virtual px); the boxes and 13/16 luma before the gate, with the resync; colours in the hardware convention
  (U = Cr); the engines start per line and per field.
- **Verified** (GHDL, 720p60, decimation 1, default knobs):
  - Black-input Video equals `imodel.py` (1.42 mean |RGB|, with the 12-px output shift aligned).
  - Osc and camera (selfie.jpg) renders are in `cmp_final.png`.
  - v0.1.0's second capture verified the per-field step: the RTL six frames later equals the model six fields on.
  - check_pkg (with `coral_pkg.vhd`) is clean; the TOML pre-check passes; dupin finds 0 `CARRY I0==I1`.

## Status
- **v0.1.1 built 2026-11-09 through the shared queue with ROUTER1.** NOT flashed.
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh coral`.
  - All six configs on seed 1. Post-route Fmax: HD Analog 83.72, HD HDMI 82.56, HD Dual 83.57, SD Analog 81.61,
    SD HDMI 72.43 (SD Fmin 27), SD Dual 83.86 MHz.
  - 6,020–6,035 / 7,680 LC (79 %), 23 / 32 EBR. iceunpack OK ×6; the packed config carries the labels and presets.
  - Output: `out/rev_b/ron/heirloom/coral.vmprog`.
- **v0.1.0 (same day):** router1, seed 1 everywhere, 5,639–5,649 LC, 21 EBR.
- **router2 history:** v0.1.0's first router2 build stalled HD Dual (overused = 2 for 7,000+ iterations), so the
  program was moved to router1. Stay on router1.

## Deviations
- **The field is a hand-tuned three-term patch**, not a contour fit. The corner fans and the chevron eye follow
  the still's composition, not every contour:
  - the top spike is a thin ridge;
  - the cyan sky covers 13 % against 19 %.
- **The solid shapes** follow the still to within a few px (2-line row pairs, the edges averaged left/right). The
  bulb top's v is approximated in 8-px steps.
  - In the clip the shapes are contours that deform over the cycle. Here they keep the still's outline and
    translate with the roll, widening and narrowing only with K5, P12 and the camera.
- **The ripple** is a straight 45° diagonal on a fixed 48-px brick grid; the still's curves gently. The centre's
  pink/olive bricks come out more violet.
- **Not drawn:** the capture border.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3, so the outer ends of the corner
  fans and the hook tips fall near the edges.
- **Cuts:** none for budget.
