# Colorfield — soft window keys as paint: breathing fields of colour on a coloured ground

**v0.1.0 built 2026-11-14; v0.1.1 (K6 camera pass) built 2026-11-15; not flashed.** Patchbay #10. Rothko's stacked soft rectangles at the default; at one
end of the slider Ellsworth Kelly's hard-edged bars, at the other one glowing fog of colour. Presets are
hand-written placeholders.

## The patch

    V ramp ─┐
    H ramp ─┼─> + edge noise (slow, low-frequency: four ridge terms) ─┐
    camera luma (slewed) or Osc lattice ─────────────────────────────┤
    dither / scumble (fixed) ────────────────────────────────────────┴─> the two ramps, perturbed
    perturbed V ramp -> four soft window comparators (the fields), screen-mixed       -> paint key K
    perturbed H ramp -> one soft window comparator (the left / right edges)           -> cross key Kc
    4 colour sources (points on the chord's palette path, each wandering on its own LFO)
        -> a crossfader chain switched by the V ramp at the gap midpoints             -> paint colour
    4 size LFOs -> the four window widths (Breath); a 5th LFO -> the cross window
    K x Kc -> keyer: ground colour <-> paint colour   (camera luma + paint luma = the stain)
    keyer out -> canvas grain (fixed noise + the edge noise as a faint mottle) -> output
    Layout switch: the two ramps trade places (Stacked <-> Columns).

Every element of the picture is one line of that sheet: the rectangles are window keys, their brushed edges are
the noise on the ramps, the halo is the key's skirt, the colours are the mixer's inputs, the canvas is the grain.

## What it draws

- **Default (Sunset chord, layout 9):** three stacked rectangles (yellow, a thin orange band, red) on a deep red
  ground, inset from the frame, with feathered irregular edges about 57 lines soft, a faint halo above, below
  and between them, and a mottled canvas surface with fine horizontal grain.
- **Motion:** each rectangle swells and shrinks on its own LFO (periods 7.3, 9.1, 11.7 and 13.9 s at the default
  Speed, the left / right edges on a 16.9 s LFO), each colour wanders along the chord's palette path on its own
  LFO (23 .. 43 s), and the four edge-noise terms drift in different directions (about 15 .. 24 px/s), so the
  edges crawl and the mottle moves like slow cloud. Nothing loops visibly.
- **How it is drawn.** The window keys and the paint colour depend on one coordinate each, so they are two
  512-entry tables redrawn every field (long axis: the keyed paint; cross axis: the cross key). Each pixel
  reads both tables at (coordinate + noise + source + dither) and multiplies. The noise moves the ADDRESS, so
  feather, halo and colour blends wobble together like brushed paint. A 4 x 4 ordered dither on the address
  interpolates between entries; with Grain on, a fixed random offset of about a twentieth of the feather
  scumbles the feather into dashes that the output filter turns into dry-brush streaks.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Fields | window comparators (count and proportions) | 16 layouts, four each of 1, 2, 3 and 4 rectangles (big / thin / equal / bars). The rectangles slide and resize to the new layout; a new one slides in from off-screen. +-8 hysteresis. Default 560 = layout 9 (large, thin band, medium) |
| K2 Feather | window comparators' softness | Feather width, exponential, 3 octaves; adds to P12's 6.4 octaves (1 .. 675 table entries; an entry is about 2 lines). Default 700 |
| K3 Colour | colourizer / palette (moved from K6 in v0.1.1) | 16 chords (ground dark + light, a looped palette path): Maroon, Ember, Saffron, Rust Blue, Blue Green, Hot Pink, Seagram, Ochre, Sunset (default), Violet, Ashes, Coral, Acid, Chapel, Meadow, Spectrum. Colours glide to the new chord. (K3 Breath is fixed at its old default 512: each rectangle breathes +-30 % of its size) |
| K4 Speed | LFO / drift master rate | Bipolar, +-8 deadband: all nine LFOs and the four noise drifts; left runs them backwards. Full = 4.8x the default. Default 680 |
| K5 Drift | colour sources' palette positions | Spacing of the fields along the chord's 16-point path (1 .. 7 steps: tonal .. far apart) and how far each wanders (0 .. +-3 steps). Default 512 = the chord's four anchor colours |
| K6 Camera | camera input gain | camera strength 0 .. 100 %, default 50 %: scales both the edge displacement and the stain. 0 % = no camera (the black-input picture), 25 % = the v0.1.0 gain, 50 % = x2, 100 % = x4 (the stain saturates). Osc keeps its 2x displacement over the camera. Glided, +-4 deadband |
| S7 Layout | ramp routing | Stacked / Columns: the two tables swap axes, so the rectangles stand side by side |
| S8 Grain | canvas | On: fixed fine grain, the mottle (edge noise at 1/4 in luma) and scumbled feathers / Off: clean flat colour, smooth feathers |
| S9 Ground | ground colour source | Dark / Light ground of the chord (glides) |
| S10 Glow | key skirt | Glow: every field's key gets a skirt four feathers wide rising to 1/4, so paint bleeds into the ground above, below and between the fields / Matte: the key ends at the feather |
| S11 Source | camera input | Video / Osc |
| P12 Dissolve | feather, swell, edge noise together | 0 % = crisp straight bars (feather 1 .. 8 entries by K2, no edge noise, no swell); default 42 %; 100 % = the fields swell 60 entries, the feather reaches hundreds of lines, the edge noise +-48 entries: one fog, a continuous glowing gradient through the chord's colours. Glided |

## Video

- **Video (default).** Camera luma goes through two one-pole lags along the line (about 14 px), a vertical lag
  (one state per 8 px, a quarter per line) and a last lag, then is added to the long-axis ramp: full white
  moves an edge 32 entries (about 67 lines), so bright areas pull the fields' edges and the colour boundaries
  toward them in clean curves. The line-lagged luma is also added to the paint luma (the stain, keyed by the
  top two bits of each key), so a soft ghost of the picture sits in the fields and not in the ground. With a
  black input both are exactly zero and the patch draws its full picture.
- **Osc.** A drifting 320 x 270 px triangle lattice through the same lags, bipolar about its mean, at 2x the
  camera's displacement gain: the edges become scallops that march across, and the lattice stains the fields.

## Engine

- **Per field: a microcoded engine** (232 instructions from `colorfield.py`, 256 x 32 register file in block RAM,
  one 28-bit adder, a serial multiplier, serial shifts). The commit block (10 instructions) runs at the vblank
  zone start and makes last field's results live (ground, noise amplitude, scumble size, noise offsets). The
  main program runs after the table fill: knob glides (K2, K3, K5, P12, layout, colours, ground: 1/8 of the gap
  per field), Speed law, nine LFOs (smoothstep of a triangle), feather gain through an exp2 ROM, the layout and
  colour ROM lookups with linear interpolation along the palette path, and the 16 fill constants.
- **Table fill (vblank zone, about 8,200 clocks):** 512 entries, 16 clocks each, one adder pair, one S-curve ROM port,
  one 10 x 11 multiplier. Per entry: four window keys = S(min of two ramps) with the halo skirt, K = 1 -
  prod(1 - key), three crossfades t = S(ramp at twice the gain), paint = C0 + sum t (colour step), then
  E = K (paint - ground) / 2 per channel, stored with K's top two bits; the cross key goes to its own table.
- **Per line:** four noise line-start accumulators and the V table coordinate (virtual 1080-line steps).
- **Per pixel (13 stages to the shell's stage 15):** noise ROMs -> fabric -> sums -> x amplitude -> coordinate
  + noise + source + dither -> clamp -> tables -> fabric -> E x Kc (three multipliers, one stage) -> + ground +
  stain -> grain, clamp. Then the shell: two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V with the per-line
  resync, luma 13/16, blanking gate. The boxes' 8-back taps and the grain delay share one 256 x 64 block-RAM
  ring (chroma 8 bits into box A, 9 into box B, 10 out).
- **Contracts:** generated avid; one latency (C_LAT 20) in every mode; blanking gate; virtual 1080-line
  coordinates from the timing ID; colours in the hardware convention (U = Cr); the engine and the fill are
  started per field (zone start), never free-running; the per-pixel noise is an LFSR reset every field, so the
  grain is a fixed canvas, not static.
- **Memory (24 EBR):** profile 512 x 32 (4), cross 512 x 8 (1), fill constants (2), register file (2), microcode
  (2), colour / layout ROM 1024 x 8 (2), exp2 (1), S-curve + skirt (1), four noise ROMs (4), video lag (1),
  delay ring (4).

## Status

- **v0.1.0 built 2026-11-14 with router1** (`NEXTPNR_ROUTER="--router router1"`), all six configs on seed 1.
  Fmax as the builder reports it (post-route): HD Analog 87.75, SD Analog 82.77, HD HDMI 84.83, SD HDMI 81.06,
  HD Dual 85.39, SD Dual 82.77 MHz. 6,704 .. 6,717 / 7,680 LC (87 %), 24 / 32 EBR.
  `out/rev_b/ron/patchbay/colorfield.vmprog` (484,222 bytes). **Not flashed.** The sources in the program folder
  are byte-identical to the ones built (checksums in the work dir, `built_sources.md5`).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay colorfield`. Router2 is not usable for
  this netlist as built: seed 1 of HD Analog stalled (2 overused wires for 6,000+ iterations) and was killed.
  `dupin.py` on the synth json reported no duplicate-pin carries. An earlier, slower version of the netlist
  did route under router2 (HD Analog only on seed 4, 75.2 MHz) before the two critical paths were pipelined.
- **Model = RTL (GHDL, 720p60, decimation 1, black input, the built RTL):**
  - default knobs with Speed at rest: bit-exact with the integer model, 0 of 918,720 pixels;
  - default knobs, moving (field 4, real glides): bit-exact, 0 of 918,720 pixels; the same frame differs from
    the model's fields 3 and 5, so the patch advances one step per field. No separate later capture was run.
- **Seen, not diffed (GHDL 720p, `cmp_final.png`):** Source Osc (scalloped edges, lattice stain) and Source
  Video with selfie.jpg (edges bend in clean curves, no ragged lines; the stain reads as soft horizontal
  streaks more than as a recognisable ghost of the picture).
- **Not simulated:** SD (ntsc) and 1080i frames, the Columns / Grain off / Matte / Light switches and the other
  chords and layouts on the final RTL. An earlier version of the RTL (before the refit below) was bit-exact
  for Columns + Grain off + Matte at 640 x 360; the final pixel path was only diffed at the defaults. Those
  settings are covered by the integer model only (`sheet.png`, `strip.png`), which runs the same microcode
  and table fill as the RTL.
- No iceunpack check and no pin-skew seed screening were run.
- Work files: `.patchbay_work/rb_colorfield/` (`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`,
  `sheet.png`, `strip.png`).

## Cut / deviations

- **First fit overflowed (8,352 LC).** Rebuilt leaner: the paint is stored pre-keyed (E = K (paint - ground))
  so the pixel path has three multipliers instead of seven, the fill shares one multiplier over 16 clocks per
  entry, and the output boxes' delay lines moved into a block-RAM ring. Costs: chroma enters the output boxes
  at 8 bits (10 out), the cross key is used at 7 bits for luma and 6 for chroma, and the stain is keyed by the
  top two bits of each key instead of the full key.
- **The halo runs along the long axis only** (above, below and between the fields); a halo on the cross key
  too made a tartan of lighter blocks left and right of the fields in the prototype.
- **K6 has 16 labelled chords**, K1 is numeric (1 .. 16): the layouts have no good short names.
- **The camera's displacement gain was fixed in v0.1.0**; v0.1.1 gives it K6 (see the K6 camera pass below). Osc is fed at 2x.
- **SD (by construction, not simulated):** the 4:3 frame shows the WHOLE composition squeezed to 4:3 (the
  table x coordinate spans the frame width in every mode), not a centre crop; the edge-noise and Osc textures
  stay in virtual pixels, so they are slightly finer relative to the rectangles. The fill needs about 8,200
  clocks of vblank plus about 100 for the commit block; CLAUDE.md gives real NTSC about 14,500 and HD about
  99,000. The image tester's fake 8-line vblank is too short for it at SD and at any decimated raster, which
  is why no SD frame was rendered.
- **Resolution of the long axis:** 512 entries per frame height, so a hard edge (P12 = 0) is about 2 lines
  soft at 1080 and positions move in 1/16-entry steps.
- Presets are placeholders: Sunset Stack, Kelly Bars, Fog, Maroon Black, Blue on Green, Pink Columns.

**Watch on hardware (unverified):** SD and 1080i pictures; that the fill finishes inside the real SD vblank
(if it does not, the lower table entries update a field late, with no visible fault expected on a slow
picture); whether the fixed camera gain bends the edges enough or too much on real video; router1 build with
no pin-skew seed screening.

## K6 camera pass (2026-11-15), v0.1.1

- **K6 is now Camera** (0 .. 100 %, default 512 = 50 %), decoded in fabric (the microcode is unchanged): once per
  field from the zone start a 4-clock glide (1/8 of the gap, +-4 deadband) eases `s_kc` toward K6, and the gain
  is g = k / 8 in Q5 (`s_cg`, 0 .. 127; k < 8 = 0). The source side multiplies both uses by g, each in its own
  stage (edge displacement 12 x 8, stain 8 x 7), then slices: displacement x g / 32, stain x g / 32 clamped at
  255. 0 % = exactly the black-input picture, 25 % = x1 (the v0.1.0 fixed gains), 50 % = x2, 100 % = x4.
  The source reaches the pixel path two clocks later than before (it is lagged over ~14 px anyway); latency
  unchanged (C_LAT 20).
- **Osc** keeps its 2x displacement over the camera under the same gain.
- **K6's old function (Colour, 16 chords) moved to K3; K3 Breath is fixed at 512** (its default, rectangles
  breathing +-30 %). The microcode's knob port 2 (Breath) now reads the constant and port 5 (chord) reads K3,
  so the engine and its settled initial state are unchanged. Presets lose their Breath values (Kelly Bars 300,
  Blue on Green 800); the chord values moved from K6 to K3.
- **Seen in GHDL** (720p60, decimation 1 because the table fill needs a longer vblank than any decimated
  raster gives, selfie.jpg; `.patchbay_work/rb_colorfield/k6/k6sheet.png`): 0 % the clean Rothko stack; 25 %
  the edges shift a little and a faint stain streaks the fields; 50 % the stain shows a soft ghost of the
  picture (laptops and skulls) in the fields and the edges ripple with it; 100 % the picture's silhouettes
  carve the field edges, the stain saturates and lightens the fields toward salmon / pale gold (bold, still the
  stacked fields, not washed out). Osc at 50 %: bold scalloped edges that march across.
- **Build v0.1.1 (2026-11-15), router1, all six configs on seed 1**: HD Analog 82.67, SD Analog 75.19, HD HDMI
  79.30, SD HDMI 77.54, HD Dual 80.38, SD Dual 80.48 MHz post-route (worst HD 79.30). 7,083 .. 7,127 / 7,680 LC
  (93 %; the two camera multiplies and the glide cost about 400 LC), 24 / 32 EBR.
  `out/rev_b/ron/patchbay/colorfield.vmprog`. Not flashed.

## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. **Bend / element**: K6 scales how far the camera displaces the field edges and how strongly it stains the
paint. The camera image itself is never shown; it only re-shapes and recolours the Rothko stack.

- **Change:** both uses are x1.25 at every K6 position. The gain register `s_cg` stays 7 bits but is now
  Q4 instead of Q5: `s_cg = (k/8 + k/32) / 2` (0 .. 79; was `k/8` in Q5, 0 .. 127). The two existing
  multiplies are unchanged and their slices drop one bit: displacement `v_vdp(16 downto 4)`, so `v_vdg` widens
  to 13 bits (max |478 x 79 / 16| = 2360; the 16-bit table coordinates and the clamp at stage 8 take it). The
  stain is `v_stp(11 downto 4)`, still clamped at 255. So 25 % = x1.25 v0.1.0, 50 % = x2.5, 100 % = x4.94 (was
  x1 / x2 / x4); k < 8 still gives 0. No new multiplier, latency unchanged. K6's default is 100 % (set by the
  orchestrator).
- **Look check** (720p60 decimation 1, which the table fill needs; bear.png, glide bypassed in scratch copies,
  K6 100 % old vs new; `.patchbay_work/rb_colorfield/k6b/k6b_cmp.png`): the same stacked fields. The bear's
  silhouette carves the field edges further, and the stain is about as saturated as before. No wrap or new
  artifacts.
- **Build (router1, all six configs on seed 1):** HD Analog 82.24, SD Analog 82.72, HD HDMI 78.88, SD HDMI
  85.70, HD Dual 82.20, SD Dual 83.64 MHz (worst HD 78.88); 7,118 .. 7,132 / 7,680 LC, 24 / 32 EBR.
  `out/rev_b/ron/patchbay/colorfield.vmprog`. Not flashed.
