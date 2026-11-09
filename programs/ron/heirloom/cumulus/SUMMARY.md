# Cumulus — mirrored mint clouds in a blue sky over scalloped yellow and pink bands with spikes

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom phase 3 (video clips), from clip WIN_20231009_00_32_55 at 0.5 s
(`photos/vid/WIN_20231009_00_32_55_t0p5.png`). Two mirrored tiles of a saturated blue sky hold mint clouds: an
ellipse on the tile seam, two tilted petals, a heart and a teardrop on the tile centre, each trailed by a dark-blue
shadow. Below it a yellow / green band with green "X" wedges sits in a scalloped bulb, pink around it with white
spikes rising from the bottom and magenta fringes; a band of the same colours runs along the top, crossed by thin
blue tracking lines. In the clip the bands roll up 5 lines per field and the clouds rise 1.4. Presets are
hand-written placeholders.

## Reading of the photo
Working files: `.heirloom_work/rb_cumulus/` (notes.md, meas_key.py, blobtape.py, proto.py, imodel.py, isheet.py,
cmp_exact.py, cmp_ctl.py).
- **Tiles.** Mirror axes at x = 521, 990, 1460 (flip-correlation), horizontal period 939 px (autocorrelation): the
  picture is a function of the folded h = |x − 521| mod 939 (0..470) and the line — an H triangle at 2.045 cycles
  per line, folded.
- **Channels.** R, G and B are separate keys: B = 0 only in the yellow/green bands, G is low only in the blue, R is 0
  in the blue, 0.71 in the clouds and a gradient in the bands. Palette (k-means): blue (0, 47, 255) 52 %, pink
  (243, 197, 228) 12 %, yellow (221, 240, 4) 9 %, mint (181, 238, 228) 7 %, green (48, 247, 5) 5 %, white
  (239, 232, 224) 4 %, plus a magenta fringe (236, 89, 231) and cyan (86, 216, 228).
- **Zones.** A vertical stack of colour zones with scalloped edges: the sky's foot is y = 750 + 84 t^2
  (t = 1 − h/470: a parabola cusp pointing down at each tile centre, measured column by column), the band → forest
  edge 1044 + 145 t^2, the sky's top edge roughly flat at y 36..58.
- **Clouds.** Per row the cloud (and, in the band zone, the pink) is ONE chord |h − c| < w: ellipse rows 262-316
  (c ≥ 470, h 218..470), petals 346-496 (centre 222 → 58, half-width up to 80), heart 526-586 (centre ~0, notch on
  top), teardrop 622-676, and the band's bulb 786-1060 (pink outside h_out(y), saw teeth dipping at 804 / 892, tip at
  (521, 1062)). The cloud centres lie on the parabola y = 277 + 268 t_c^2.
- **Band colour.** G 247, B 5, R = clip(1.8 (|h − hd(y)| − w0)): a dark green line at hd = 150 + 1.8 |y − 905|
  (w0 12 px in the lower band, 50 px in the top one) — the green "X" wedges.
- **Spikes.** White on the pink: (1 − cos θ)/2 < (y − 902)/404, θ = 2π (h − 92)/153 (spikes at h 92, 245, 398, i.e.
  x = 123 / 277 / 429 and 613 / 766 / 919; 22 px wide at y 920, 63 px at 1035). Inside the bulb the same spikes show
  only as thin dark horns near its bottom (a thinner threshold).
- **Textures.** A dark-blue shadow (G 47 → 24) for ~95 px to the right of every cloud, recovering over 25 px (an
  AC-coupling undershoot); a 10 px magenta fringe where the scan leaves the pink for the band; a ±2.5-level G ripple
  in the blue (13-line period, slanted); soft horizontal edges; a cyan torn strip (the next zone) at the bottom.

## Motion in the clip
All 679 frames re-extracted (`fr/`), kymographs at three columns, column-run tracking.
- The first 0.2 s of the clip is frozen (the strip's first three frames are all t = 0).
- **The bands rise 30 lines per 0.1 s = 300 lines/s = 5.0 lines per 60 Hz field** (the band top at x = 990 from 760
  to 374 between 0.5 and 1.8 s, linear).
- **The clouds rise slower: 83 lines/s = 1.4 lines per field** (the ellipse at x = 990 from 264 to 98 between 0.5 and
  2.5 s), and they swell as they go: the petals grow into wings by +1.0 s.
- The zone sequence repeats in ~6.8 s (sky → bulb band → a "forest" of dark-green spikes on white with a cyan comb →
  band with pink hearts → sky), so the zone tape is 2048 lines at 5 lines per field. No knob is turned in the clip;
  the same patch's other clips are other settings (33_43: purple sky, cream clouds, yellow band — the Orchid
  palette).
- **What the program does:** at the default Speed the zone tape rolls up 5.0 lines per field and the cloud tape
  1.33, a rigid scroll (best fit of the m1/m2 cloud masks: rigid 80 lines/s 15 % error; a two-speed tape, 9.8 %,
  gave kinked shapes and was not used). So the bands climb over the clouds — the clouds turn pink inside the band,
  the green spikes and the comb rise from below — as in the clip.

## What it does
1. **Tile.** A 24-bit phase DDA per pixel (2.045 tiles at the default, anchored at the screen centre), folded into
   hq = 0..1023 per half tile.
2. **Clouds.** Per line the engine reads the blob tape (512 entries over 1280 lines: the still's rows plus a 200-line
   tail), interpolates c and w, then c' = 1023 − L (1023 − c) (Lean) and w' = G w + O (Billow). Per pixel
   d = w' − |hq − c'| (Twin: the max with the chord mirrored about the quarter point), + luma × Etch; key = d > 0
   (Invert flips it).
3. **Zones.** Psi = Yz − K t^2 + luma × Etch / 4 (t^2 from a ROM, K per line from a piecewise cusp-depth curve times
   Scallop) against four edges: sky [50, 750), band [750, 1044), forest [1044, 1600), band [1600, 2098).
4. **Colour.** Sky: mint / blue with the shadow (a run counter since the key's last falling edge) and the ripple.
   Band: pink with white spikes / the R gradient with dark-green spikes, magenta on the key's falling edge, blue
   tracking lines on five tape lines. Forest: a cyan comb (116 lines, per-line jitter), pink / white with spikes.
   A 256-entry palette ROM holds both palettes.
5. **Output.** One [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V (the still's softness and the HD HDMI bar fix, with the
   running-sum resync), luma 13/16, the blanking gate. Palette luma is pre-scaled 16/13 × 0.89 so the still's white
   lands on the 940 cap and pink, mint and white keep their order.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tiles | the mirrored tiles across: ×1/4 .. ×4 (exponential; 512 = the still's 2.045). 0 = one huge cloud bank, 1023 = a frieze of eight small cloud motifs |
| K2 Lean | the clouds' centre line: 0 = every cloud slides onto the tile seam (stacked ellipses), 512 = the still, 1023 = ×1.6, the clouds crowd onto the tile centre (small hearts and chevrons) |
| K3 Scallop | the zone edges' cusp depth: 0 = inverted arcs (the bands bulge up at the tile centre), 341 = flat edges, 512 = the still, 1023 = ×4, deep V notches |
| K4 Speed | bipolar (±8 deadband, square law): the bands roll and the clouds rise; 768 = the clip (5.0 / 1.33 lines per field), 1023 = ×4; left of centre rolls down |
| K5 Spikes | the spikes' count ×0.35 .. ×2.8 (512 = the still's 153 px period); right of centre they also reach up to 306 lines higher, into the band |
| K6 Etch | how far the camera (or the Osc) swells the clouds (luma × gain into the chord test) and bends the band edges (a quarter of it into Psi); square law, 384 = the default |
| S7 Twin | Off / On: a second, mirrored set of clouds (seam ↔ centre): the petals cross into X shapes, a heart sits on every seam |
| S8 Palette | Cumulus (the still) / Orchid (purple sky, cream clouds, yellow-orange band, orange spikes: the same patch's 33_43 clip) |
| S9 Invert | Off / On: the cloud key inverted — a mint sky with blue clouds, pink bands with yellow bulbs |
| S10 Slip | Off / On: sync slips — 64-line bands of the picture (virtual lines, on the zone tape) torn sideways by up to half a tile, re-torn about once a second |
| S11 Source | Video / Osc |
| P12 Billow | the clouds' size (gain 0 .. 2.6 plus an offset): 0 % = clear sky (no clouds, the bands all yellow / green), 400 = the still, 100 % = towering clouds merging into broad mint banks, the bands all pink |

## Video
- **Video (default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, times Etch, is added to the cloud chord test
  (bright parts of the picture swell into mint / pink cloud) and a quarter of it to Psi (the band edges bend around
  the picture). At the default Etch the composition still reads; with no input, Video mode is the still.
- **Osc.** The shell's 640 × 540 virtual-px triangle lattice, drifting, at half depth, feeds the same path: the clouds
  swell and dent in a slow lattice. Etch stays live in both modes.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier, 15 slots (registered one-hot operand
  selects, flat AND-OR fetch); a shared glide (1/8 of the gap per field) on K1, K2, K3, K5, K6, P12; a 256 × 16 exp2
  ROM. It computes the tile step and phase, the spike step and tip, speed² → the two tape rates, Lean, the Billow
  gain and offset (the offset read at zc 40, after the glide), Scallop, Etch², the ripple and comb starts, then moves
  the zone tape (mod 2048 lines) and the cloud tape (mod 1280).
- **Per-line engine** (~90 clocks, one serial multiplier with slots at 4 / 21 / 38 / 55 / 72; fits SD's 858-clock
  line): the tape positions, Yb × 205 → the blob entry and fraction, the K segment, two ROM reads and the
  interpolation, Lean, Billow, Scallop × K, the band gradient line, the spike depth, the tracking / comb flags, the
  comb jitter hash, the slip offset.
- **Pixel path** (C_LAT 24): beam at stage 1, DDAs at 5, fold 6, chord / gradient distances and the t^2 ROM address 7,
  d and the spike partials 8, twin max / gradient level / spike sum 9, luma, K × t^2 nibble products and the spike ROM
  10, key and P 11, run counter and Psi 12, zone / spikes / shadow 13, palette index 14, palette ROM 15, fabric copy
  16, compose 17, box 18-19, luma 13/16 20, carry 21-24, gate 25.
- **Memories (16 EBR):** line buffer 2048 × 8 (4), blob tape 512 × 24 (3), t^2 1024 × 12 (3), spike shape 256 × 8 (1),
  palette 256 × 32 (2), exp2 (1), output-box ring 256 × 30 (2, `no_rw_check`).
- **Contracts:** generated avid (parity-locked); one latency (C_LAT 24) in every mode; virtual 1080-line coordinates
  (SD shows the centre 4:3); colours in the hardware convention (U = Cr) from `cumulus.py`; output conditioning with
  the running-sum resync before the blanking gate.
- **Verified (GHDL 720p60, decimation 1, real timing ID).**
  - The default frame (black input, Speed still) is **100 % bit-exact** with the integer model `imodel.py` on Y, U
    and V (capture offset 4 px). So is the default-speed frame three fields in (15 lines of band roll, 4 of cloud
    rise).
  - Two control frames — (K1 600, K2 300, K3 200, K5 800, P12 700, Twin, Orchid, Slip) and (K2 900, K3 60, P12 150,
    Invert), after three glide steps — are bit-exact once the Billow offset is taken a glide step late. That one-field
    lag was then fixed in the RTL (the model has no lag).
  - The same sims found a line-engine bug, fixed before the build: the Lean multiplier slot loaded inside the cusp
    depth's slot, which corrupted K whenever Lean ≠ 512 (invisible at the defaults).
  - The Osc and camera frames were checked by eye (`cmp_final.png`). Model vs still at the defaults: palette shares
    blue 61.9 / 61.6 %, mint 7.2 / 8.0, yellow 8.1 / 9.4, green 6.4 / 5.2, pink 12.9 / 12.2, white 3.4 / 3.6; the
    4 × 4 region means (model luma rescaled) differ by a median of 3.3 levels (max 6.1).
  - `sheet.png` is rendered from the bit-exact model; `cmp_motion.png` compares the model at +0.5 s / +1.0 s with
    the clip's m-frames.

## Status
- **v0.1.0 built 2026-11-09 (router2, the builder default), NOT flashed.** `out/rev_b/ron/heirloom/cumulus.vmprog`.
  - **All six configs on seed 1.** Post-route Fmax: HD Analog 86.13, HD HDMI 87.19, HD Dual 85.57, SD Analog 86.48,
    SD HDMI 74.26 (Fmin 27), SD Dual 83.61 MHz.
  - 6990-7005 / 7680 LC (91 %), 16 / 32 EBR. iceunpack OK ×6; the packed config carries the labels and presets.
  - GHDL analyse / elaborate / synth front end clean (`check_pkg.sh`); TOML pre-check clean; dupin: 0 CARRY I0 == I1
    (safe for router1). The only RAM with transparency emulation is the line buffer (kept, as in the other shells).
  - Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh cumulus` (or `./build_heirloom.sh cumulus`). If a
    later version gets tight, try `NEXTPNR_ROUTER="--router router1"` before restructuring. LC is near the 92 %
    budget: little room for additions.
  - Presets are hand-written placeholders; capture Ron's looks with `./capture_preset.sh`.

## Deviations from the photo
- **The clouds scroll rigidly** at 1.33 lines per field. In the clip they also swell as they rise (the petals become
  wide wings by +1.0 s, the heart splits into a pair); the program's clouds keep the still's shapes while scrolling.
- **The sky's top edge is flat** at y 50; the still's edge bumps between y 36 and 58.
- **Brightness:** drawn whites come out at about 777 (the 13/16 output scale and a 0.89 palette pre-scale that keeps
  white above pink and mint), so the mint, pink and white are dimmer than the camera capture; the blue's luma is 239
  against the still's 260.
- The tracking lines run full width (the still's are partial); the bulb's spike horns are dark green (the still's are
  purple); the still's cyan, G-leading fringe on the cloud edges and its ~10 px rightward key delay are not modelled
  (the shapes are exactly symmetric about each tile axis), nor the right tile's ~30 px spike drift.
- The zones seen only in motion (the forest with its comb and dark-green spikes, the second band) are simplified
  approximations of the clip's.
- With a bright camera picture the default Etch swells the clouds noticeably (the composition still reads).
- **Resolutions:** 720p shows the same picture as 1080p; SD shows the centre 4:3 (virtual x 240-1680, both tile centres
  included) and is softer (the box filter is per output pixel).
