# SUGARCOAT

**STATUS: v1.2.0 FINAL except presets (Ron signed off 2026-10-07: "Sugarcoat
can be finalized for now").  Presets are placeholders — capture on hardware
with `./capture_preset.sh` when looks are dialled in, then rebuild (HD HDMI
needs SEED 2).**

**v1.2.0 (2026-10-07) — logical THEME maps replace the arithmetic K1
permutations (Ron: v1.1 "feels like a regression in other ways"; wanted the
64 maps separated by density / colour / style so dark dense textures land in
dark areas, mid in grey, bright in white, and hues map to hue-matched
candies).**

Build: 6/6 — HD Analog 79.54 (seed 1) / **HD HDMI 76.52 (SEED 2)** / HD Dual
77.14 (seed 1); SD 78-85; ~4495 LC (58%), 7 EBR.  C_LATENCY = 32.

Design:
- **16 classes**: 0-7 = luma band (Luma map, AND neutral pixels in the Chroma
  map, so grey reads identically in both modes); 8-15 = hue octant (green,
  teal, cyan, blue, red, amber, pink, violet).  P12 Strength: Luma mode masks
  the band to 2/4/8 varieties; Chroma mode sweeps the saturation floor.
- **Theme ROM** (`C_TMAP`, 16 x 16 x 4 bits, LUT ROM): per theme a luma ramp
  of 8 textures SORTED by the texture's mean brightness (verified in the
  generator) and a hue map chosen by colour.  Bank A themes (K1 top 3 bits):
  Classic, Bakery, Liquorice, Sweet Shop, Cocoa, Stripes, Pastel, Bits.  Bank
  B (S9): Classic B, Bakery B, Dark, Bright, Mono, Multi, Bakery Dark,
  Stripes B.  K1 bit 6 = pair SWAP (adjacent luma bands / adjacent hues),
  bit 5 = TONE (luma ramp one step brighter; hues untouched) — both keep the
  ordering.  8 x 2 x 2 x 2 banks = 64 maps, every one logical.
- MAP is 3 stages: class adjust (26) -> ROM read (27) -> register (28);
  PH1 29, PH2 30, CO 31, out 32.  Gloss/outline/shade taps moved +2.
- Texture brightness used for sorting: choc 170, twist 200, nonpareils 250,
  cream 300, allsorts 400, cookie 480, taffy 510, gingerbread 550, waffle 560,
  cane 680, Battenberg 750, beans 750, white choc 780, mint 845, icing 900,
  frosting 950.

Everything else as v1.1.0 (K2 Clean, K4 Contrast, K5 Level, S7 Outline, S8
Shade, S10 Sprinkles, S11 Map, big chips, blanking gate, line-RAM fixes).
K1 is labelled "Theme" (0-63).  Presets still placeholders.

---

**v1.1.0 (2026-10-06) — rework from Ron's first hardware look at v1.0.0
("looking very good"; wanted more texture combinations, no Palette/Warp,
Density + Melt invisible, a contrast knob or cleaner textures, 2-3 more
candies, bigger chips).**  Not yet flashed.

Build: all 6 configs **seed 1, zero retries** — HD Analog 86.16 / HD HDMI
80.66 / HD Dual 76.15 MHz (builder figures; netlists not kept, so not
independently re-routed this round — the v1.0.0 builder figures matched the
independent re-route exactly), SD 75-82 MHz; **~4265 LC (56%)**, 7 EBR.
C_LATENCY = 30.

Design:
- **16-texture bank**: 0 dark chocolate, 1 cookies & cream, 2 choc-chip
  cookie (chips now ~14x11 px of a 16 px cell, density 28%), 3 candy cane
  (red/white diagonal), 4 mint (green/white other diagonal), 5 piped icing,
  6 rainbow taffy (now HORIZONTAL fixed-rainbow stripes so it never reads as
  cane), 7 frosting + confetti, 8 Battenberg (pink/yellow checks), 9 liquorice
  allsorts (black/pink/black/yellow bands), 10 waffle cone (tan + brown grid),
  11 jelly beans (4-colour capsules on white), 12 gingerbread (brown + white
  icing scallops), 13 liquorice twist (black/grey diagonal), 14 nonpareils
  (4-colour dots on dark chocolate), 15 white chocolate (cream bar + grooves).
- **K1 Texture** = 64 class->texture maps: texture = (class*a + r) mod 16 with
  a in {1,3,5,7} (K1 bits 9:8) and r = K1 bits 7:4.  K1 = 0 is the identity
  (the designed assignment).  Each map shows 8 of the 16; **S9 Bank** (A/B)
  xors 8 = the complementary 8.
- **K2 Clean** = run-length speck filter on the class stream: a class is
  accepted after N = 1..16 consecutive pixels (K2 bits 9:6), or immediately
  when it matches the clean class on the line above (class line RAM 2048x4,
  2 EBR); otherwise the last accepted class holds.  The accepted stream is
  read back through a variable tap (17 - N) so the total class delay is fixed
  at 24 and genuine edges do not shift.  0% = raw.
- **K4 Contrast** (steps 0.375 .. 3, 50% = unity) and **K5 Level** (bipolar
  luma offset, centre deadband = exact unity) form a proc-amp on the
  classifier input (G1 two shifted terms -> G2 add -> G3 recentre+clamp).
- **Removed**: K4 Palette (56-entry ROMs -> one fixed 8-stop rainbow), K5
  Warp (W1/W2 stages), K2 Density (stripe pitch fixed at bit 4 of the
  pre-scaled projection), S10 Melt.  **S10 is now Sprinkles** (Off/On).
- Pattern stage is sampled ~26 px ahead of the class stream (a constant,
  invisible texture offset) instead of delaying 4 projections by 16 taps;
  dx / xrun therefore keep counting through blanking and the cell engine
  uses the line count latched at avid rise (s_lrun), so the overrun never
  sees the next line's values.
- Kept from v1.0.0: blanking gate, line-RAM write-behind, vsync-serration
  guard, S7 Outline, S8 Shade, K3 Scale labels + index-trap decode.
- Presets (placeholders, re-capture on HW): Candy Shop, Sweet Shelf (Bank B),
  Big Bites, Bakeshop, Cel Candy.

HW check list: 16 textures all legible at 1080p, chips big enough, Clean
sweep removes speckle without eating thin features, Contrast/Level centre =
unchanged, K1 sweep gives distinct looks, Bank flip, no blanking cast.

---

**v1.0.0 (2026-10-06) — finalization pass on the v3.4 candy-map design.**
Not yet flashed; HW check pending (see below).

Build 2026-10-06: all 6 configs **seed 1, zero retries**; routed Fmax
(independently re-routed, last post-route line) HD Analog 78.19 / HD HDMI
82.05 / HD Dual 82.53 MHz; SD 77-87 MHz; **3748 LC (49%)**, 5 EBR.  Packed to
`out/rev_b/ron/sugarcoat.vmprog`.

Contract fixes:
- **Blanking gate** added: a final registered stage drives Y 64 / U,V 512
  outside latency-aligned avid (`p_out`, `s_avid_sr(C_LATENCY-2)`).  The old
  build ran candy texture through blanking (mix was pinned at 100%).
- **Gloss line RAM** no longer reads and writes the same address in one cycle
  (undefined on iCE40 EBR): the write is registered one clock behind the read.
- **Interlace detect** ignores repeat vsync edges (analog serration): field
  parity / `s_ilace` only update on a vsync that followed real active lines.

Optimisation:
- The SUGAR RUSH dry/wet mixer (3 multipliers, 13-deep dry delay line, 3
  pipeline stages) was dead weight at a fixed 100% and still bled 1/64 of the
  source into every candy colour.  Removed.  **C_LATENCY 16 -> 14** (all
  modes).  The source-luma delay line is kept to depth 9 (Melt tap 8, Shade
  tap 9).

Controls:
- **S7 Outline** (Off/On): dark cocoa contour wherever |gx|+|gy| of the source
  luma exceeds 96, reusing the gloss gradient; drawn over everything, no gloss
  or shade on the line.  Flag computed at L2, delayed in `ol_sr` (read 7).
- **S8 Shade** (Flat/Shaded): source luma shades the candy colour,
  `(y-512)>>1` added into the gloss term at PH2, so the picture reads through
  the texture.
- **K4 Palette** now also colours frosting confetti and the jimmies (stops
  1/3/4/6 of the active ramp) through the same single PH1 palette read,
  index-muxed on the class; **S9 Bakery** therefore recolours the sprinkles to
  cocoa/caramel tones as well as the taffy.
- **K3 Scale** is a labelled param (8/16/32/64 px) with the labelled-default
  index-trap decode (`s_k3 < 4` = label index).
- Presets renamed for the candy-map look (Candy Shop, Fine Print, Big Bites,
  Bakeshop, Melted, Cel Candy) — **placeholder values**, to be re-captured
  from hardware with `./capture_preset.sh`.

HW check list: cookie reads amber (not blue); all 8 hue octants land on the
right candy (hue-sweep source); P12 sweep in both Map modes; gloss visible;
Outline / Shade flips; no green/magenta blanking corruption.

---

**v3 (current): a pure 1:1 colour -> candy-texture map.**  Each pixel is
classified by its chroma (hue octant vs. saturation threshold) or, when
neutral, by its luma; the class picks one of 8 flat screen-space candy
materials.  No generated geometry, nothing animates on its own — the video
alone decides where each candy appears and how it moves.

| class | candy |
|---|---|
| neutral dark | dark chocolate bar (quilted grooves) |
| neutral dim | cookies & cream (cream dot grid on black) |
| neutral light | piped icing (pink scallop piping) |
| neutral bright | frosting + rainbow confetti |
| red | candy cane stripes |
| yellow/orange | chocolate-chip cookie |
| green | mint stripes (opposite diagonal) |
| blue/cyan/magenta | rainbow taffy (palette-ramp stripes, K4) |

Controls: K1 **Texture** (rotates the class -> candy map), K2 **Density**
(stripe pitch), K3 **Scale** (one global texture size, 4 steps: 8/16/32/64 px
cells with matching stripe/groove pitches), K4 **Palette** (taffy ramp), K5
**Warp** (static bend), K6 **Gloss**; S9 Bakery, S10 Melt (luma stripe-shear),
S11 **Map** (Chroma / Luma); P12 **Strength** — how hard the active map is
read (chroma: saturation floor; luma: 2/4/8 varieties).  Sugar Rush is fixed
at 100% (always fully candied).  Gloss emboss + garnish ride on top; jimmies
land only on icing/frosting, and sprinkles stay a fixed size by design.

All 6 configs close at **seed 1 with zero retries** (HD 79.4 / 87.7 / 89.0
MHz, 57% LC) — the CORDIC/polar engine and both shared multipliers of the
earlier motif/texture designs were deleted outright.

**Timing note for future edits:** a per-frame size/scale control must not
widen the pixel-path muxes.  Apply scale ONCE as a pre-shift at the projection
shift-register input (p_geo), keep every downstream slice/shift at its
original narrow width, and register the cell-size mux ahead of the cookie hash
— doing either inline in the MP stage costs 5-8 MHz on HD.

---

## v1/v2 history (superseded)

Video re-rendered as **candy** (a material, not just "candy colored"): the
image is posterized into flat palette bands, then dressed with candy motifs,
gloss and garnish. `SUGAR RUSH` (P12) dips the raw image into the confection.

Structural re-render (per feedback: no field-overlay/mask genre) — the picture
is rebuilt from candy primitives, every control restructures it.

## Status — building incrementally (visual-feedback per stage)

- **Stage 1 ✓** Conditioning + palette map. Posterize luma into N bands
  (K3, 2..16) → map onto a 7-palette candy ramp (K4). SUGAR RUSH crossfades
  raw → confection (inline 3-clock dry/wet lerp). Sim-verified candy colour.
- **Stage 2 ✓** Peppermint pinwheel + the **angle engine**. Pipelined CORDIC
  vectoring (lifted from hypnos, 4 stages/8 iters) turns each pixel's (dx,dy)
  about the measured screen centre into polar (radius, angle).
- **Stage 3 ✓** Domain warp — cross-axis drifting sine displaces (dx,dy) before
  the CORDIC, so the whole motif field swirls/melts like taffy. WARP=K5,
  FREEZE=S11. Sim-verified: WARP=0 clean, WARP up = swirling melt, no lattice.
- **Stage 4 ✓** Gloss — one line RAM gives the previous line's luma; a 2D
  luma-gradient emboss (heightfield normal, light upper-left) adds a specular
  sheen / rim shadow at compose. GLOSS=K6. Sim-verified wet sheen on shoulders.
- **Stage 5a ✓** Motif selector (K1) + polar motifs sharing two multipliers
  (one angle, one radius): Pinwheel, Lollipop Spiral, Jawbreaker Rings.
- **Stage 5b ✓** Candy-cane **stripes** (4th motif, cartesian projection of the
  warped coords reusing the angle multiplier), **BAKERY** (S9, Bakery palette +
  cocoa lights), **OUTLINE** (S7, dark contour on strong band edges), **MELT**
  (S10, luma shears the motif phase), **FLOW** (S8, gradient-octant bend on the
  radial motifs). All sim-verified.
- **Stage 6 ✓** Garnish — scattered jimmie capsules + fine sugar-dust twinkle,
  density riding the SUGAR RUSH ramp (hash-placed, dot-shaped in 8px cells).
- **Stage 7** SUGAR RUSH staged weighting (warp fades in early-mid, gloss mid,
  sprinkles top half; mix floored so P12=0 is a flat poster), 5 presets, and
  the HX4K **timing-close campaign** (see below).

## Timing close (HX4K, no DSP)

First full build: 99% LC, HD Analog **50 MHz** vs the 74.25 required (SD passes
at 27). Closing HD needed removing ~15% of logic AND fixing several ~18-20ns
combinational cones, found by profiling the nextpnr critical path each time
(`--timing-allow-fail -l log`, read "Critical path report"). The sequence:
warp & gloss gain mult->shift; CORDIC 4->3 stages + 20b->17b datapath (angle
kept, radius coarse); split the SUGAR RUSH warp-weight ladder across vblank
cycles (a vblank cone was capping Fmax!); shrink the compose phase-add to 15b
+ move the garnish hash out of the compose cone; **PH pipeline stage** so the
palette-ROM read isn't in series with the phase adder (the big one, +6 MHz);
warp sine-ROM -> folded triangle (removes the ROM from the warp path).
Result so far: HD ~**70 MHz**, LC **84%**. (Sweeping seeds / final trims to
clear 74.25.)  Lesson: when Fmax is flat across big area changes, it's a fixed
LOGIC cone -- PROFILE, don't guess; and vblank/per-frame logic is timed at the
pixel clock too.

### CLOSED (2026-07-27) — all 6 configs pass, genuinely routed
Three targeted fixes cleared HD (C_LATENCY now 16, ~81% LC):
1. **band->pidx LUT** — the per-pixel `band*pscale` E3 multiply was the critical
   carry chain; replaced with a 16-entry lookup filled by accumulation during
   vblank (pixel path = a mux).  +~10 MHz all configs.
2. **Compose retime (+1 stage)** — the output-feeding CO stage stacked tone-mux
   + garnish-mux + gloss-add + clamp.  Split into PH1 (phase add + ring ROM) ->
   PH2 (tone select + garnish) -> CO (2:1 ring mux + gloss add + clamp).  Fixed
   HD Dual.  (First try merged phase-add and tone-mux into one cycle and
   regressed HDMI 80->70 -- keep them in separate cycles.)
3. **Gloss L3 split** — the gloss sheen (barrel-shift + spec-add + clamp) was the
   HDMI limiter; split shift (L3) from add+clamp (L4), absorbed by reading the
   gloss delay SR one tap earlier (no net latency change).  HDMI ~74->80.

Genuine **routed-worst** Fmax (verify the WORST nextpnr freq line, not the
builder's pre-route estimate): HD Analog **76.99** (all seeds pass), HD HDMI
**80.21**, HD Dual **75.15** (tightest, ~1-in-8 seeds); SD all >=69 (need 27).
Image sim-verified intact after the pipeline changes.  Not yet HW-tested.

### Deferred to v0.2
Rock-candy (Voronoi facets), chocolate mould / wafer (rectilinear), caramel
marble; true MOTIF crossfade between neighbours (currently a discrete 4-zone
K1 select); FLOW's full smoothed content-orientation (v0.1 uses a coarse
per-pixel gradient octant, radial motifs only).

## Architecture (current, C_LATENCY = 15, 1 EBR = line RAM)

Streaming, no per-frame random access.
- Raster measurement (interlace-aware) → screen centre cx=W/2, cy=H/2.
- Coordinate accumulators: dx per pixel, dy per active line (interlace-safe).
- Colour path: E1 in → E2 posterize band → E3 palette stop → E4 palette YUV
  (→ delayed to compose).
- Geometry path: W1/W2 warp → R1 |.|,signs → R2 max/min → CORDIC ×4 → RU polar.
- Gloss path: line RAM → gx/gy → lit → g_add (→ delayed to compose).
- MP: two shared multiplies (angle·amul, radius·rmul) per the selected motif.
- CO compose: motif tone (family A white/colour+pinstripe, family B multi-hue
  rings) + gloss sheen.
- SUGAR RUSH inline lerp (M1..M3), then sync-aligned output.
- ~10 multiplies (band, pidx, warp×2, motif×2, gloss, mix×3). Watch at timing.

## Control surface (6 knobs / 5 switches / 1 slider)

- P12 **Sugar Rush** — raw → fully candied (signature performance gesture)
- K1 Motif · K2 Size · K3 Bands · K4 Palette/Hue · K5 Warp · K6 Gloss
- S7 Outline · S8 Flow · S9 Bakery · S10 Melt · S11 Freeze
- Sprinkle density rides the Sugar Rush ramp; Speed is a fixed rate + Freeze.

## Notes

- Palettes authored standard BT.601 (genpal.py). Sim renders standard U/V so
  hues are trustworthy in sim; confirm a possible global U/V swap on hardware.
- The image-tester sim feeds a **decimated 480×270** stream to the VHDL, so the
  screen centre MUST come from measured s_W/s_H — never hardcode pixel coords.
