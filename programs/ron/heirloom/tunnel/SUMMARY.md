# Tunnel — diamond glow rings streaming out of an eye, over a weave of stripes and chevron bars

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20230723_00_52_24` at 0.5 s
(`photos/vid/WIN_20230723_00_52_24_t0p5.png`). A black-and-white relief: soft white diamond rings around an "eye"
just right of centre, a static weave of horizontal stripes and dark vertical bars (bent into "<" chevrons near the
axis), and two eye corners that throw fans of thin rays to the sides; the rings stop inside the fans and the stripes
stop above the corners. In the clip the weave never moves; the rings sway (their vertical spacing tilts up and down)
and stream out of the eye. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_tunnel/` (`notes.md`, `meas1-3*.py`, `trace*.py`, `ridge.py`, `amp.py`,
`lfft.py`, `prof.py`, `fitT.py`, `proto.py`, `imodel.py`, `sheet.py`, `thincheck.py`).
- **Palette.** A grey ramp (k = 8): black (7,8,7) 16 %, then even steps to white (200,208,216) 11.5 %. Mean luma
  97.6 / 255. The tint follows the luma: neutral blacks, slightly green dark-mids (Cr 503, Cb 511 at Y 250), cool
  highlights (Cr 493, Cb 533 at Y 700).
- **Rings.** Concentric diamonds centred x 1000, y 512. On the centre column the ring crests sit at y 100 / 337
  above and 708 / 961 below: 237 lines apart above, 253 below (a skew of +0.035). The edges slope 0.5-0.6
  (the inner ring steeper near the corners), the top vertex is rounded over ±100 px, and past |dx| ~690 the
  edges turn steep (slope ~1.8: the "///" lines in the top corners, "\\\" in the bottom ones). The crest profile
  (column 980) is a soft glow, FWHM ~0.4 of a period, with a thin bright core; ~50 / 255 between rings, ~0 in the eye.
- **Weave.** Horizontal stripes every 63.5 lines (exactly 17 per frame, bright rows on y = 63.53 n), straight at
  every x. Dark vertical bars every 96 px (20 per line, dark duty 0.42); their x shifts +0.62 px per line of |dy|
  within 250 lines of the axis ("<" chevrons), above and below they stand nearly upright.
- **Eye and fans.** The corners sit at (747, 517) and (1253, 517). From each, thin rays (~6 px) leave at slopes
  ±0.145, ±0.29 and ±0.435. The stripes stop above the outer upper ray (and above the eye's top edge, slope ~0.56
  inside); the bars fade inside the fans (`amp.py`); the rings never cross the fans. The far left and right
  columns carry stripes top to bottom.
- **Texture.** Horizontally soft (camera), ridges read as embossed tubes; no tearing, no noise structure.

## Motion in the clip
- Weave, bars and fans are static: the stripe and bar phases hold to ±1 px over 7 s (`meas3e.py`).
- The rings sway: on the centre column every crest moves UP (142 → 100 → 66 → 34 above, 718 → 706 → 688 → 666
  → 634 below over 0.5 s steps) while the spacing above grows 224 → 286 and below shrinks 274 → 224, with the eye
  corners fixed. That is the V triangle's skew sweeping from +0.10 to −0.12 in 1.5 s (0.13 / s).
- At 7.5 s a new ring is born in the eye and the rings stream outward at ~70 px/s (532 → 490 → 450 → 410 up,
  602 → 680 → 712 down).
- The clip `00_54_55` shows the same patch with the whole field bent by a slow warp (what Etch does with a camera).
- **The program:** the skew rides a triangle LFO (±0.24, 7.4 s period = the clip's 0.13 / s); at the default Speed
  the rings stream out at ~30 px/s at 60 Hz (25 px/s at 50 Hz). `cmp_motion_model.png` shows the integer model
  (bit-exact with the RTL) at +0.5 / +1.0 s next to the m-frames.

## What it does
1. **Rings.** D = a·T(|dx|) + |dy|·(1 ∓ skew), T = r²/256 to 128 px, then r − 64, slope ×3 past 690 px. The ring
   phase is G·D − 180 − flow + etch (G = rings per px, P12), read through a glow profile (cos^2.6 glow + thin core;
   a sharper glow at P12 < 160 and > 768; S8 Line = thin outline only).
2. **Weave.** H(y) = stripe profile per line; V(x) = soft dark-bar key from a per-pixel phase started per line at
   (x0 + 5/8·min(|dy|, 250))·kv; base = V·(g0 + Ah·(H − g0)), g0 = 0.24.
3. **Eye.** e = r − Xe; rays at |dy| = n·rs·e (n = 1..3, a 6-px triangle) outside the eye; the fan = inside the outer
   ray (soft over 40 px); the stripe key Ah opens below the corner V's (slope 3·rs outside, 9/16 inside the eye,
   ×0.375 inside the eye, rising past |dx| 860 for the edge columns).
4. **Screen.** lum = base + lit − base·lit, lit = ring·(1 − fan) + rays / 2; S10 Ink inverts it.
5. **Colour.** A 256-entry ROM maps luma to the grey with its green / cool tint (Y pre-scaled 16/13, capped).
6. **Output.** Y, U, V through one [1 2 2 2 2 2 2 2 1]/16 box (the still's softness and the HD HDMI bar fix), luma
   13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Slope | the diamonds' aspect: flat, wide rings (0, a × 1/4) .. the still (512) .. tall, narrow rings with steep corner lines (1023, × 4) |
| K2 Skew | the rings' vertical balance (bipolar, ±8 deadband): crowded below / tall above (0) .. the still (512) .. crowded above (1023); the clip's sway LFO rides on it |
| K3 Weave | stripe and bar density together: big blocks (0, × 1/3: 190-line stripes, 288-px bars) .. the still (17 stripes, 20 bars) .. a fine mesh (1023, × 3) |
| K4 Speed | ring flow (bipolar, ±8 deadband, square law): rings stream into (−) or out of (+) the eye; 640 = the default ~30 px/s; full = ~2 rings/s |
| K5 Eye | the eye corners' spread and the fans: corners 40 px apart with wide fans (0) .. the still (255 px, slope step 0.145) .. a wide eye 700 px each side with narrow fans (1023) |
| K6 Etch | how far the camera (or the Osc, 4 ×) bends the rings: 0 .. ~15 ring periods at full luma (square law); 160 = 0.36 ring at full luma |
| S7 Twin | one eye / two eyes side by side (every element folded about x = 520 / 1480) |
| S8 Rings | Glow (the still) / Line: thin bright diamond outlines, no glow |
| S9 Weave | On (the still) / Off: rings and rays alone on black |
| S10 Ink | Light (the still) / Dark: the negative, black glows on a pale weave |
| S11 Source | Video / Osc |
| P12 Depth | ring density (glided): 0 % = one giant ring (period 790 px, sharp glow) .. 400 = the still (237 px) .. 100 % = a deep tunnel of fine lined rings (47 px) |

Glides (1/8 of the gap per field) on K1, K2, K3, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, box-filtered over 8 px, is added
  to the ring phase: the rings bend and fray around the picture while the weave and fans stay put. At the default
  Etch the composition still reads on selfie.jpg (`cmp_final.png`, bottom). With no input, Video mode is the still.
- **Osc mode.** A drifting 640 × 540 virtual-px triangle lattice feeds the same path at 4 × the camera's gain: at the
  default Etch the rings warp into big lips and waves (77 % of pixels change vs a black input).

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier, 15 slots (slot decoded two clocks ahead,
  operands pre-muxed), the shared glide, a 256-entry exp2 ROM. Per field: G (P12), a (K1), G·a, the skew (K2 + LFO)
  and G·(1 ± s), speed², the weave exponent and the bar / stripe steps, Xe and the ray step (K5), etch² (K6), the
  flow and LFO steps.
- **Diamond table:** G·a·T(r) mod one ring for r = 0..1023 virtual px, 1024 × 16 block RAM (`no_rw_check`), refilled
  every field by a second-order DDA (Q16.8: Q += D; D += Ga/128 in the parabola, Ga at r = 128, 3 Ga at the knee).
- **Per-line engine** (kicked at each line start, computes the NEXT line in ~46 clocks): |dy|·G(1 ∓ s) − 180 − flow,
  the bar start phase (x0 + 5/8 min(|dy|, 250))·kv − 26 kv, |dy| in quarter px; the stripe level is read from the
  profile ROM once the line's last pixel has left the pixel path.
- **Pixel path (25 stages):** |dx| and the twin fold (2-5), table read (5-7), e and the edge term (7), ray step rs·e
  and the eye key, ring phase = table + line term + etch (8), the key operand (9), ray distances / key difference /
  profile address (10), ray profiles and clamped keys / profile ROM (11), max ray, fan and stripe levels, the bar key
  (12), weights (13), V, ring × (1 − fan), the stripe mix (14-15), weave and lit (16), screen (17-18), Ink and the
  colour address (19), colour ROM (20), compose (21), box (22-23), luma 13/16 (24), gate (25).
- **ROMs:** exp2 256 × 16, profiles 1024 × 8 (glow, sharp glow, line, stripe), colour 256 × 32, the table 1024 × 16
  and the 2048 × 8 line buffer: 13 EBR.
- **Contracts:** the generated avid; one latency (C_LAT 24) in every mode; the output box with the blank resync,
  luma 13/16 before the gate; virtual 1080-line coordinates (SD shows the centre 4:3); colours in the hardware
  convention (U = Cr).
- **Verified:** GHDL 720p60 decimation 1, black input, default knobs: output frames 3 and 4 are each 100 % bit-exact
  (Y and U) with `imodel.py` at fields 3 and 4, and differ from each other on 51 % of pixels (one field of flow), so the
  per-field motion path matches the model too (the RTL picture sits 4 px right of the model's: the shell's pipeline
  offset). Osc and camera frames are in `cmp_final.png`. `thincheck.py`: no single-row
  run ≥ 24 px at the defaults, P12 0 / 100, Line, Line + P12 100, Weave off, Ink, Weave max, Eye 0 / max, Twin,
  at 720p and 1080p, two flow phases each.

## Status
- **v0.1.0 built 2026-11-09 with ROUTER1, not flashed.** All six configs on seed 1. Post-route Fmax: HD Analog 79.02,
  SD Analog 81.99, HD HDMI 84.21, SD HDMI 81.79, HD Dual 82.31, SD Dual 91.73 MHz. 6,755-6,764 / 7,680 LC (88 %),
  13 / 32 EBR. iceunpack OK ×6; the packed config carries the new labels and presets.
- `out/rev_b/ron/heirloom/tunnel.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh tunnel`
  (or `./build_heirloom.sh tunnel` with the same prefix).
- **router2 stalls on this netlist:** the first (router2) build passed HD Analog on seed 1 (79.03 MHz) but SD Analog
  seed 1 sat at overused = 2 for 2,800+ iterations; it was killed and the whole build rerun on router1, which closed
  every config on seed 1. `dupin.py`: 0 `CARRY I0==I1` (router1-safe).
- Fit history: the first RTL (two output boxes per channel, keys in 1/16 px, 28 stages) synthesised to ~8,200 LC;
  one box per channel, narrower key arithmetic, the bar DDA moved next to its use, 2-4-bit blend weights for the fan /
  stripe keys, a 24-bit table fill and a 32-bit line multiplier brought it to 6,760 at the same picture (the integer
  model was updated with it, and the v2 RTL is bit-exact with it).
- Presets are hand-written placeholders (Still, Deep Tunnel, Twin Eyes, Line Work, Negative, Bare Rings, Wide Eye,
  Osc Melt). Capture Ron's looks with `./capture_preset.sh`.

## Deviations from the photo
- The weave is drawn as clean oscillators: the still's left-side stripes wobble slightly and carry ragged notches;
  here they are straight, with the bars at a quarter strength inside the fans.
- The bars' slight perspective tilt above and below the chevron band (±0.2-0.5 px/line) is not modelled.
- The inner ring is a plain diamond: in the still it pinches a little harder into the eye corners than the outer rings.
  The eye corners and fans are fixed elements (as in the clip), so flowing rings cross the corners.
- The relief (bright crest with darker flanks) comes only from the glow profile; there is no directional emboss.
- The brightest crests come out at the shell's ~777 output luma cap (the still's whites reach ~860 for 2 % of pixels).
- The left edge fades in from black over ~8 px (output box warm-up).
- **Resolutions:** 720p shows the same geometry as 1080p (the box filter is per output pixel, so 720p is a little
  softer). SD shows the centre 4:3, so the outer corners of the side fans and the steep corner lines fall outside the
  picture.
