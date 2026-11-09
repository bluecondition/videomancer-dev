# Marionette — a rolling curtain of hanging magenta figures with green gaps and pink heads

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20231003_00_48_14` at 1.5 s
(`photos/vid/WIN_20231003_00_48_14_t1p5.png`). A curtain of 4.7 hanging figures across: tall tapered magenta bodies,
each with a red line down its left edge and a cyan / white line down its right, a violet rim, a black band and a
green gap that deepens downward into a white "arch" line; the hems glow red on the left and white on the right.
Above the bodies a crown band of pink / white heads among bright green "trees", torn into 85-line strips that slip
vertically. In the clip the whole curtain rolls up at about 220 lines / s while Ron lengthens the bodies. Presets
are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_marionette/` (`notes.md`, `contours.py`, `fitk*.py`, `fit4.py`, `tears.py`,
`proto.py` / `mproto.py`, `imodel.py`, `isheet.py`, `thincheck.py`, `rtlcmp.py`).
- **Repeat.** Horizontal period 408 px (row-band autocorrelation 404–408 px everywhere; magenta-run starts 204,
  612, 1024, 1433, 1840): 4.7 figures across. Gap centres at x = 119 + 408 k, body centres at 323 + 408 k.
- **Bodies.** From y 292 (right part; the left part rises to y 228 in an S-shaped shoulder between t = −0.23 and
  +0.12 tile) down past the bottom of the frame. Magenta interior (255,0,255) at the top centre, (200,0,250) near
  the edges. Magenta half-width 109 px at y 350, 52 px at y 700; the black contour 176 px at y 320, 124 px at y 1000.
- **Colouriser.** An RGB colouriser with slope-signed fringes: where the level rises (left edges) B notches out →
  a red line ((212,0,117) → (176,0,19)); where it falls (right edges) G spikes → a cyan / white line
  ((129,197,255)). Inward to outward: magenta → line → violet (B 255 → 0 over 27 px at y 450, 38 px at y 950) →
  black → dark green → green.
- **Field fit.** Per row, the contour levels (line, black, G32, G90, G140, arch) against the distance from the body
  centre fit a near-linear profile A(ν) − K(ν)·f (exponent scan 0.8–2.5: best 1.0–1.2, rms 0.013–0.016), K 0.97 →
  0.75 and A 0.57 → 0.19 down the body, plus a gap dip D 0 → 0.30 (quadratic near the gap centre) for the arches.
  Palette levels: line band −0.06…0, black −0.267, G32 −0.33, G90 −0.40, G140 −0.51, arch −0.59, gap centre at the
  bottom −0.76.
- **Gap arches.** Gap-centre G 65 (y 450), 115 (y 650), 180 (y 950); the white (tile 0) / red-brown (tile 1)
  arch line appears at y ≈ 590 and widens to ±30 px at the bottom.
- **Hem glow.** The lower third of each body glows: a red flame from the left edge (250,0,7), white / lilac on the
  right (237,229,232) / (170,66,255), purple between; it differs per tile (tile 0 red | white, tile 1 lilac across,
  tiles 2–3 maroon).
- **Crown, y 0–290 (+ 1045–1080), 325 lines.** Bright green (5,236,1) trees over the gaps, pink / lilac / white
  figures with dark violet hollows and cyan / red contour lines over the bodies; seams at y 84, 171, 256, 292 (≈ 85
  lines) where the content jumps vertically (row cross-correlation shift 0: vertical slips, not horizontal tears).
- **Palette** (k = 14, whole still): dark green 16 %, magenta (247,4,255) 13 %, (203,1,252) 10.7 %, mid green 9.5 %,
  bright green 9.5 %, violet (93,10,242) 8.8 %, (70,5,166) 7.2 %, dark violet 5.8 %, (159,9,242) 5.2 %, lilac 3.6 %,
  maroon 3.0 %, cyan 2.9 %, white 2.8 %, red 2.0 %.
- **Texture.** Soft horizontal edges, clean vertical ones; an interlace comb in the moving crown (a capture artefact,
  not reproduced).

## Motion in the clip
- **The curtain rolls up.** Body bottom at y 1045 → 940 → 820 → 720 → 640 at t 1.5, 2.0, 2.5, 3.0, 3.5 s (105,
  120, 100, 80 px per 0.5 s); 550 px from 3.5 to 6.0 s = 220 lines / s. The picture wraps with a 1080-line period
  (the crown above plus below the body adds to 320–325 lines at t 1.5–2.5).
- **Ron's knob.** The body / crown split: at t 0 the crown is ~660 lines and the body ~420; by t 1.5 crown 325 /
  body 755; at 6.0 s the crown fills the frame. The sibling clip `00_50_39` is the same patch recoloured (black
  ground, green figures, blue rims, pink heads).
- **Crown flicker.** The strip contents jump every few frames.
- **In the program.** K4 at its default rolls the figures up at 218 lines / s (3.63 virtual lines per field, a
  1080-line wrap; the GHDL frames 3 → 6 moved up 7 rows at 720p = 3.5 lines per field); P12 is Ron's split knob (body
  length 160 … 1000 lines, 724 at the default); each field one strip in four gets a new slip offset (LFSR), so the
  crown seams jump about once a second per strip.

## What it does
1. **Coordinates.** Tile phase u (Q20 per tile, scaled about the screen centre, two tile-count bits on top); figure
   line φ = (virtual line + roll) mod 1080; 85-line strips with a per-strip vertical slip (± 40 lines, Torn 4×) and a
   sideways tear (Torn only).
2. **Profile.** For every line the engine builds a 256-entry profile over the signed tile phase:
   body = A(ν) − K(ν) f − D(ν) dip(f), compressed toward the hem plateau Lp(f) by m(ν) (the glow); minus the rim
   RIM(e), e = distance beyond the body's top / bottom (the left part reaches 62 lines higher, 5/8 of that lower);
   crown = heads profile − 34 lu per line inside 50 / 66 lines of the rim; P = max(body − rim, crown). Each entry
   also carries the crown-texture weight W (fading in over 320 lu past the crossing; W = 1 flags the rim, where the
   gap-arch palette band is stepped over so no thin arch rows appear).
3. **Pixel.** s = P[u] + W × (rib + tex1 + tex2) + hem tilt (tiles 1 / 3 fall / rise across the hem) + source.
4. **Colour.** PAL[light][slope variant][s >> 4]: 256 levels over −1.30 … +1.10 in two slope variants (rising: red
   fringes, falling: cyan / white), Day or Night. The variant compares s with s 8 px back with a ±3 lu hysteresis
   (held on flats, so camera noise and the flat hem don't sparkle).
5. **Output.** Chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16, blanking gate.

## Controls
| Control | Element it drives |
|---|---|
| K1 Figures | how many figures hang across: 1.8 (0) .. 4.7 (512, the clip) .. 12 (1023), exponential, scaled about the centre |
| K2 Waist | the bodies' taper and the gap arches: straight columns, no arches (0) .. the clip (512) .. pinched wedges with tall arches (1023) |
| K3 Heads | the heads and the necks (one CV multed): no heads, flat-topped bodies in a green band (0) .. the clip (512) .. big heads, necks 2.8× (1023) |
| K4 Roll | bipolar (±8 deadband, square law): the curtain rolls up (+) or down (−); 772 = the clip's 218 lines / s, full = 870 lines / s |
| K5 Glow | the hem glow: none, pure magenta bodies (0) .. the clip's red / white hems (512) .. red / white flames up the whole body (1023) |
| K6 Etch | how far the camera (or the Osc) bends the figures: 0 .. strong; 220 = a gentle bend |
| S7 Twin | two rows of figures (the figure period halved; rims kept as thick on screen) |
| S8 Tear | Calm (the clip's 85-line slips) / Torn: 4× slips and sideways tears through the whole curtain |
| S9 Light | Day (the clip) / Night (the sibling clip: black ground, green figure rings, blue rims, pink cores) |
| S10 Sway | the figures swing side to side: a 760-line triangle wave of ±0.17 tile travelling down the curtain |
| S11 Source | Video / Osc |
| P12 Curtain | the bodies' length: a crowd of heads with short bodies (0, the clip's 6 s look) .. the clip (688) .. the curtain closed, tall bodies with glowing hems (100 %) |

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 → 0 … −128, from a half-resolution line buffer, pulls
  the level down by Etch × luma: the bodies narrow and their edges and arches bend around bright parts of the
  picture, the heads take the picture's shapes. A black input adds nothing, so with no camera Video mode is the
  still. At the default Etch (220, up to −0.11 level) the structure stays readable with `photos/selfie.jpg` (the
  bodies narrow and their hems break up); at full Etch the picture shreds into green / black (not pale).
- **Osc mode.** The shell's drifting triangle lattice (640 × 540 virtual px), bipolar, at about 3.5× the camera's span
  (±0.19 level at the default Etch): the bodies bulge into hourglass / vase shapes and the heads shift. Every
  video-path control stays live in Osc mode.

## Engine
- **Per-field sequencer (vblank zone).** A shared glide (K1, K2, K3, K5, K6, P12); slots on the serial multiplier for
  the tile step (exp2 ROM), the first-pixel phase, roll speed², etch², body length, glow onset, neck and heads width;
  a restoring divider for 2^23 / L and the heads index step; the roll and the strip LFSR. The serial multiplier is
  shared with the line engine (the engine never runs during the zone slots).
- **Line engine.** Kicked at each line start (and once in the zone for line 0), computes the NEXT line: φ, the strip
  slip, ν = pos / L, A / K / D from the ν tables × Waist, the glow m, the three texture phases, the sway; then builds
  the 256-entry profile, one entry per clock, into the block-RAM bank the pixel path is not reading (~455 clocks, so
  it fits SD's 858-clock line).
- **Pixel path (16 stages).** Profile read, triangles, texture × W, hem tilt, source, level, slope variant (level
  ring in block RAM), palette ROM, compose, output conditioning, gate.
- **ROMs / RAMs.** Palette 1024 × 32 (8 EBR), profile banks 512 × 16, neck / dip and crown / weight tables, the
  ν / rim / smoothstep table, exp2, strip table, line buffer, chroma, sync and level rings: 21 EBR.
- **Contracts.** Generated avid; one latency (C_LAT 16) in every mode; virtual 1080-line coordinates (SD shows the
  centre 4:3); colours in the hardware convention (U = Cr); luma pre-scaled 16/13 and capped at 940 (drawn whites
  ~777 after the 13/16 scale); the chroma running sums snap back to neutral after 8 blank samples.
- **Verified.** `imodel.py` mirrors the arithmetic; the GHDL default frame (720p60, decimation 1, black input)
  matches it visually (56.5 % of pixels bit-exact, mean |ΔY| 2.0 at the best alignment; the rest are ±1-step
  palette differences in sparse vertical streaks, cause not traced). `thincheck.py`: no single-row luma line ≥ 24 px at 1080p
  for the defaults, P12 0 / 100, Torn, Twin, Heads max, Waist max, Glow max at four roll phases; at 720p only Twin
  shows two (25–33 px).

## Status
- **v0.1.0 built 2026-11-09 through the shared queue with ROUTER1; all six configs on seed 1. Not flashed.**
  - HD Analog 82.82 MHz (7,164 LC), SD Analog 79.68 (7,127), HD HDMI 83.69 (7,128), SD HDMI 80.87 (7,158),
    HD Dual 85.14 (7,159), SD Dual 78.25 MHz (7,165); 21 / 32 EBR. iceunpack OK ×6.
  - Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh marionette`.
  - Output: `out/rev_b/ron/heirloom/marionette.vmprog`.
  - No self-adds (dupin: 0 `CARRY I0==I1`, 0 LUT duplicate inputs); one transparency emulation (the line buffer,
    read-before-write kept as in the shell).
  - Fit history: the first netlist was 7,574 LC (98.6 %, 82.6 MHz). Cut to ~7,150 by a write-address counter in the
    profile build (which also fixed a one-entry write offset), the variant baseline in a block-RAM ring, a late
    crown-ROM read, Lp from the carried tile index, the multiplier shared between sequencer and line engine, and
    combinational texture constants.

## Deviations
- **Crown.** The heads are a designed figure profile (pink / white core high on the left, violet right flank, dark
  outline) with ribs and two diagonal triangles, not the clip's chaotic camera figures; the green trees are plain
  green with contour lines (no dark trunks). The interlace comb is not reproduced.
- **Per-tile variety.** The hem tilts per tile (tile 1 falls, tile 3 rises), but the edge-line colours follow the
  slope everywhere (the clip's tile 2 has a cyan left line; here every left line is red).
- **Bottom of the frame.** The clip's torn 35-line strip at y 1045 shows a slipped strip of the next crown; here the
  bodies' foot rim and the next heads.
- **Hem.** The red / white hem is narrower and smoother than the clip's flame, with small stair steps where the
  plateau changes palette entry.
- **Osc.** A small patch of short red / white stripes can appear where the lattice flattens the hem.
- **Output.** As in the shell, the picture sits 9 px right of the virtual grid (the luma / chroma filters and the
  generated avid's start), so the left ~7 px are dark.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3 (3.5 figures) and, with 4.4
  virtual lines per SD line, the 3-line fringe bands at the bodies' tops and bottoms become one SD line; Twin at 720p
  can show two short single-row lines.
