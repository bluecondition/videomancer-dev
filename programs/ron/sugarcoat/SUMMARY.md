# SUGARCOAT

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
