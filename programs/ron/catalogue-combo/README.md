# Catalogue combo batch (2026-09-10 .. 09-11)

Random 2-3 effect combos from EFFECTS_CATALOG.md. Every program: 3 knobs per effect (2 for three-effect combos),
switches + P12 slider = how the effects interact. Flash from `out/rev_b/ron/catalogue-combo/<name>.vmprog`; rebuild with `./build_combo.sh <name>`. NONE hardware-tested.
Build status (2026-09-11): first 45 packed, every HD config a real pass (table: .roulette_work/final_results.txt). Day-2 additions (jigsaw onward, seed 20260920) listed below; see .roulette_work/results.sh for their status.

## Edge fill (2026-09-13)
mirage, halation, tremor, etchwarp, impasto, parallax, flutter, hueflutter and trapquant no longer paint black where a
horizontal displacement reads off the line. Any read that leaves the line, or lands within C_EDGE (12) columns of either
end -- the capture's black blanking columns -- is redirected to the first real column inside that inset, so the fill is
that line's own edge pixel and continues the colour leading up to it with no black seam. (A first version painted a
64-px line average; on hardware the black blanking columns still showed as a seam, magnified by filter windows, cells and
keystone scaling, so it was replaced by this address clamp.) impasto and halation also hold the last active luma through
blanking so their box windows no longer darken the outer columns. Where a switch used to offer Black / Wrap or
Black / Mirror it now reads Fill / Wrap or Fill / Mirror, defaulting to Fill.
Timing rework in the same pass: trapquant (sequencer multiplies pre-registered / split, 69 -> 82 MHz), hueflutter
(output stage split in three, hue-rotation and wobble products split hi/lo, 65 -> 76-82 MHz, latency 24), impasto
(relief product split, sign applied after, 74 -> 90 MHz).

## mirage
Mirage: catalogue combo #4 (seed 20260912) -- Sabattier solarization + displacement mapping (the picture's own luma pushes it sideways).  Three knobs each; the switches and the slider decide how they combine.

- K1 Threshold  solar threshold
- K2 Gain       fold gain 0.125x .. 8x (log); >1x repeats into bands
- K3 Lift       level of the folded zone
- K4 Amount     displacement, bipolar (+-256 px at full map swing)
- K5 Smooth     horizontal low-pass on the map
- K6 Pivot      luma level of zero displacement
- S7  Map       displacement map from the solarized luma / the original
- S8  Order     solarize before the warp (the warp moves the bands) /
- S9  Region    displacement only inside the solar region
- S10 Fold      the solar region also inverts chroma
- S11 Wrap      out-of-range reads wrap instead of edge-filling
- P12 Bands     displacement amount grows with the solar luma

## halation
Halation: catalogue combo #5 (seed 20260912) -- bloom / glow / soft focus + magnetic warp (wobble).  Three knobs each; the switches and the slider decide how they combine.

- K1 Glow      bloom gain
- K2 Threshold luma above which the blur blooms
- K3 Radius    blur width: 9 / 17 / 33 px (two cascaded boxes)
- K4 Wobble    per-line sine displacement amplitude, 0..+-255 px
- K5 Rate      wobble roll speed, bipolar, centre = frozen
- K6 Period    vertical wavelength of the wobble
- S7  Warp     wobble moves both layers / the glow layer only
- S8  Tint     the glow carries a warm chroma cast
- S9  Energy   wobble amplitude scaled by each row's glow energy
- S10 Blend    glow added / screened
- S11 Soft     the base picture is replaced by its blur (soft focus)
- P12 Halo     adds the un-warped blur of the line below on top: a

## infrared
Infrared: catalogue combo #10 (seed 20260912) -- perspective keystone / corner-pin + false-colour (thermal-style LUT).  Three knobs each; the switches and the slider decide how they combine.

- K1 Keystone  bipolar; rows scale linearly with height about the centre
- K2 Skew      bipolar parallelogram shear
- K3 Zoom      overall horizontal scale 0.25x .. 4x (log)
- K4 Palette   thermal / ice / plasma / x-ray (4 zones)
- K5 Range     luma gain into the palette (contrast)
- K6 Offset    luma bias into the palette (sweeps the ramp)
- S7  Rows     palette offset drifts with the row (heat falls with distance)
- S8  Iso      black isotherm lines at palette stop boundaries
- S9  Pins     chroma keystoned with the mirrored trapezoid
- S10 Luma     palette replaces luma too / only chroma comes from the palette
- S11 Invert   palette reversed
- P12 Heat     the palette cycles with time; speed = slider

## stencil
Stencil: catalogue combo #2 (seed 20260912) -- mosaic with per-cell colour averaging + masking by luma / chroma.  Three knobs each; the switches and the slider decide how they combine.

- K1 Cells    cell width 2 .. 64 px (rows follow via K2)
- K2 Aspect   cell height = width / 1, 2, 4, 8
- K3 Grout    dark grid lines between cells
- K4 Level    key threshold
- K5 Soft     key edge softness
- K6 Source   key from luma (0) .. saturation (100), blended
- S7  Invert  mask inverted
- S8  Key     mask computed from the mosaic colour (blocky) / the original
- S9  Reveal  the masked region shows the original / its negative
- S10 Sharp   luma stays original; only chroma is mosaic'd
- S11 Grout   grid lines everywhere / only inside the mask
- P12 Tint    masked cells pushed toward a hue, amount = slider

## porthole
Porthole: catalogue combo #6 (seed 20260912, phosphor slot redrawn) -- polar warp (spherize) + Perlin displacement + picture-in-picture with warping.  Two knobs each; the switches and the slider decide how they combine.

- K1 Curve    spherize: row scale by row radius, bipolar (pinch / bulge)
- K2 Zoom     overall scale 0.25x .. 4x (log)
- K3 Noise    value-noise displacement amount (64x32 lattice, scrolling)
- K4 Speed    lattice scroll speed
- K5 Size     window width (10 .. 100% of the picture; height follows)
- K6 Position window centre column
- S7  Outside original picture / the warped picture outside the window
- S8  Noise   inside the window only / everywhere
- S9  Border  black frame around the window
- S10 Mirror  window content flipped
- S11 Fit     window shows the whole picture compressed / a 1:1 crop
- P12 Drift   the window swings sideways on a sine, speed = slider

## cartridge
Cartridge: catalogue combo #15 (seed 20260914) -- cartoon / cel (edge + flat colour) + palette quantization (Gameboy / CGA / C64 / EGA).  Three knobs each; the switches and the slider decide how they combine.

- K1 Edge      cel edge threshold (lower = more ink)
- K2 Levels    posterize depth 8 / 6 / 4 / 3 / 2 bits
- K3 Width     edge tap spacing 1 / 2 / 4 / 8 px (line weight)
- K4 Palette   Gameboy / CGA / C64 / EGA
- K5 Dither    4x4 Bayer dither amplitude before the quantizer
- K6 Range     luma contrast into the quantizer
- S7  Ink      edges black / the palette's darkest entry
- S8  Order    posterize then quantize / quantize then posterize
- S9  Edges    edges keep the original colour
- S10 Luma     luma from the palette / from the cel (palette chroma only)
- S11 Invert   cel luma inverted before quantizing
- P12 Inking   thickens the ink and lowers the edge threshold together

## outliner
Outliner: catalogue combo #13 (seed 20260914) -- edge detect (Roberts / wide gradient / difference-of-boxes) + split-screen + invert / negative / bit-plane.  Two knobs each; the switches and the slider decide how they combine.

- K1 Threshold edge threshold
- K2 Kernel    1-px gradient / 4-px gradient / difference of 5- and 17-px boxes
- K3 Seam      split position
- K4 Panes     1 / 2 / 4 / 8 vertical strips alternating processed / original
- K5 Plane     negative / bit-planes 8..2
- K6 Mix       negative or plane mixed over the picture
- S7  Edges    white lines on black / lines drawn over the picture
- S8  Right    the "original" side is inverted instead
- S9  Sides    edges on the processed side only / both sides
- S10 Invert   inversion applies to edges only
- S11 Colour   edge lines take the source chroma / are white
- P12 Sweep    the seam swings on a sine, speed = slider

## etchwarp
Etchwarp: catalogue combo #12 (seed 20260914) -- edge detect + Perlin / value-noise displacement + pixelation.  Two knobs each; the switches and the slider decide how they combine.

- K1 Threshold edge threshold
- K2 Ink       edge line strength (mixed over the picture)
- K3 Noise     value-noise displacement amount (64x32 lattice, scrolling)
- K4 Speed     lattice scroll speed
- K5 Cells     pixel cell width 1 .. 64 px
- K6 Rows      cell height = width / 1, 2, 4, 8
- S7  Lines    edges drawn over the picture / edges only on black
- S8  Order    noise moves the cell grid / noise moves the content
- S9  Luma     pixelate luma only (chroma stays sharp)
- S10 Colour   edge lines take the source chroma / are white
- S11 Blocks   edges detected on the pixelated picture / on the smooth one
- P12 Flow     extra noise scroll speed and amount together

## reflector
Reflector: catalogue combo #8 (seed 20260912) -- mirror / kaleidoscope + bloom / glow.  Three knobs each; the switches and the slider decide how they combine.

- K1 Axis      position of the first mirror axis (0..100% of the width)
- K2 Folds     0 / 1 / 2 / 3 nested folds (1, 2, 4, 8 panes)
- K3 Slide     the source scrolls under the mirrors, bipolar
- K4 Glow      bloom gain
- K5 Threshold bloom threshold
- K6 Radius    blur width 9 / 17 / 33 px
- S7  Glow     glow fetched mirrored with the picture / unmirrored
- S8  Panes    alternate panes shown as negative
- S9  Tint     warm glow
- S10 Seams    extra glow along the mirror seams
- S11 Soft     base picture replaced by its blur
- P12 Sweep    the first axis swings on a sine, speed = slider

## tremor
Tremor: catalogue combo #17 (seed 20260915) -- Perlin-style displacement + scanline shear + duotone.  Two knobs each; the switches and the slider decide how the three combine.

- K1 Scale    noise cell size 4 .. 512 px
- K2 Drift    noise field scrolls vertically (0 = still)
- K3 Skew     scanline shear: rows lean by (row in period - P/2) * skew
- K4 Period   shear sawtooth period 2 .. 256 lines
- K5 Hue      highlight tone hue
- K6 Spread   angle between shadow and highlight hues (0 = one tint)
- S7  Noise   warps the picture horizontally / grains the luma instead
- S8  Shear   sawtooth / triangle
- S9  Bands   every other shear band swaps the two tones
- S10 Field   smooth value noise / hard cells
- S11 Tone    duotone keyed from luma / from the noise field itself
- P12 Amount  displacement (or grain) amplitude, up to +-255 px

## vitrail
Vitrail: catalogue combo #20 (seed 20260915) -- bloom / glow + palette quantization + mirror / kaleidoscope.  Two knobs each; the switches and the slider decide how the three combine.

- K1 Strip    kaleidoscope source strip width 64 .. 512 px
- K2 Axis     where the strip is taken from (0 .. 100 % of the width)
- K3 Thresh   bloom threshold
- K4 Radius   bloom blur width 4 .. 64 px
- K5 Levels   luma levels 2 .. 16
- K6 Chroma   chroma levels per axis 1 (grey) .. 8
- S7  Bloom   luma glow / white glow (chroma washes toward neutral)
- S8  Mirror  kaleidoscope fold / plain tiling
- S9  Order   palette after the mirror / palette before the bloom
- S10 Axis    fixed / sweeping across the picture
- S11 Bloom   highlights glow / shadows bloom dark
- P12 Glow    bloom gain, up to blow-out

## impasto
Impasto: catalogue combo #23 (seed 20260916) -- Perlin / simplex displacement + oil paint / Kuwahara smoothing.  Three knobs each; the switches and the slider decide how the two combine.

- K1 Scale    noise cell size 4 .. 512 px
- K2 Drift    noise field scrolls vertically
- K3 Stroke   noise cells stretched horizontally x1 .. x8
- K4 Brush    smoothing window 4 .. 32 px each side
- K5 Edge     0 = plain box blur .. 100 = hard nearest-mean pick
- K6 Relief   brush-stroke relief (difference of the two window means)
- S7  Noise   warps the picture / steers which window is picked
- S8  Field   smooth value noise / hard cells
- S9  Pick    nearest mean / farthest mean (haloed oil)
- S10 Relief  light from the left / from the right
- S11 Knife   plain / luma posterized to 8 steps
- P12 Amount  displacement amplitude, up to +-255 px

## benday
Benday: catalogue combo #31 (seed 20260917) -- chromatic glitch / datamosh bars + ASCII / dither-art / halftone.  Three knobs each; the switches and the slider decide how the two combine.

- K1 Cell     halftone cell 4 .. 32 px
- K2 Ink      dot gain
- K3 Screen   dot / line / diamond / square
- K4 Height   mosh band height 8 .. 64 lines
- K5 Shift    block shift inside a moshed band (px)
- K6 Rate     re-roll rate
- S7  Ink     dark ink on light / light ink on dark
- S8  Colour  mono print / dots take the cell colour
- S9  Mosh    shifts the picture / shifts the screen
- S10 Bands   chroma hold + U/V swap / luma crush
- S11 Bands   halftoned / flat cells
- P12 Density fraction of bands moshed

## blockcut
Blockcut: catalogue combo #32 (seed 20260917) -- edge detect + datamosh / block displacement.  Three knobs each; the switches and the slider decide how the two combine.

- K1 Gain     edge gain
- K2 Thresh   edges below this are dropped
- K3 Blend    edge picture (0) .. the picture with edges over it (100)
- K4 Block    block size 8 .. 64 px
- K5 Shift    block displacement amount
- K6 Rate     re-roll rate
- S7  Edges   white / tinted with the picture's colour
- S8  Blocks  plain / moshed blocks go negative
- S9  Hash    per block / per band (whole rows of blocks share a shift)
- S10 Mosh    shift / hold (block repeats its first column)
- S11 Ink     light edges on dark / dark edges on light
- P12 Density fraction of blocks moshed

## parallax
Parallax: catalogue combo #38 (seed 20260918) -- depth-from-luma parallax + colour channel swap / offset.  Three knobs each; the switches and the slider decide how the two combine.

- K1 Depth    parallax gain (luma above / below focus shifts sideways)
- K2 Sway     the camera sways on its own; speed
- K3 Focus    luma level that does not move
- K4 Swap     channel mode: none, U<>V, Y->U, U->Y, Y<>V, rotate
- K5 Offset   chroma read offset (px, bipolar)
- K6 Split    U and V offset opposite ways (0) .. the same way (100)
- S7  Depth   from luma / from saturation
- S8  Near    bright is near / dark is near
- S9  Chroma  displaced with luma / stays put (colour ghosts)
- S10 Sway    sine / triangle
- S11 Edges   black / wrap around
- P12 Position camera position, bipolar (centre = flat)

## subsample
Subsample: catalogue combo #37 (seed 20260918) -- datamosh / block displacement + chroma subsampling artifacts (4:1:1, composite).  Three knobs each; the switches and the slider decide how the two combine.

- K1 Block    block size 8 .. 64 px
- K2 Shift    block displacement amount
- K3 Rate     re-roll rate
- K4 Hold     chroma sampled every 1 .. 32 px (4:1:1 and worse)
- K5 Bleed    chroma IIR smear length
- K6 Crawl    composite dot-crawl amplitude (luma dots at chroma edges)
- S7  Blocks  shift / hold (block repeats its first column)
- S8  Chroma  subsampled after the mosh / before it (blocks carry the artefacts)
- S9  Crawl   checkerboard / bars
- S10 Moshed  blocks keep chroma / lose chroma (grey blocks)
- S11 Hold    boxes / interpolated (bled)
- P12 Density fraction of blocks moshed

## tonecurve
Tonecurve: catalogue combo #34 (seed 20260918) -- colour bleeding / NTSC dot crawl + picture-in-picture with warping + gamma curves / S-curve contrast. Two knobs each; the switches and the slider decide how the three combine.

- K1 Curve    gamma 0.5, 0.7, 1.5, 2.2, soft S, hard S, inverse S, crush
- K2 Contrast linear gain about mid grey before the curve
- K3 Size     inset 1/2, 1/4, 1/8
- K4 Warp     inset wobble (sine per line, drifting)
- K5 Bleed    chroma IIR smear length
- K6 Lag      chroma delayed 0..31 px
- S7  Curve   main picture / inset only
- S8  Place   inset in the corner / centred
- S9  Bleed   main / inset only
- S10 Inset   picture / negative
- S11 Lag     main / inset only
- P12 Amount  how far the curved luma replaces the original

## crush
Crush: catalogue combo #42 (seed 20260919) -- sync loss / horizontal collapse + mosaic with per-cell colour averaging.  Three knobs each; the switches and the slider decide how the two combine.

- K1 Line     where the picture collapses to (0 .. 100 % of the width)
- K2 Sync     sync loss: hashed per-line offsets
- K3 Roll     the collapse line drifts sideways on its own; speed
- K4 Cells    cell width 2 .. 64 px
- K5 Aspect   cell height = width / 1, 2, 4, 8
- K6 Grout    dark grid lines between cells
- S7  Order   cells then collapse (cells squeeze) / collapse then cells (cells stay square)
- S8  Collapse toward the line from both sides / everything to the left
- S9  Loss    per-line / 8-line bands
- S10 Beyond  black outside the picture / edge cells stretched
- S11 Grout   black / white
- P12 Collapse the picture squeezes to the line (x1 .. x17)

## pressroom
Pressroom: catalogue combo #40 (seed 20260919) -- ASCII / dither-art / halftone + head-switch noise band + chroma / luma / difference key.  Two knobs each; the switches and the slider decide how the three combine.

- K1 Cell     halftone cell 4 .. 32 px
- K2 Screen   dot / line / diamond / square
- K3 Band     head-switch band height (0 .. 63 lines)
- K4 Noise    noise amplitude inside the band
- K5 Level    key threshold
- K6 Soft     key edge dithered by noise
- S7  Key     from luma / from saturation
- S8  Halftone inside the key / outside it (the rest stays photographic)
- S9  Ink     dark on light / light on dark
- S10 Band    at the bottom / rolling up the frame
- S11 Band    noisy picture / halftone screen breaks up (random dots)
- P12 Ink     dot gain

## teletext
Teletext: catalogue combo #47 (seed 20260920) -- chromatic glitch / datamosh bars + ASCII / dither-art + scanline shear.  Two knobs each; the switches and the slider decide how they combine.

- K1 Bars     fraction of glyph bands displaced (datamosh)
- K2 Distance mosh displacement (up to +-512 px)
- K3 Cell     glyph cell 8 / 16 / 32 px
- K4 Contrast density -> glyph mapping (100% = unity, up to 400%)
- K5 Shear    per-line shear amount (+-30 px)
- K6 Rate     shear wave rate (Sine) / tear probability (Tear)
- S7  Ink     white glyphs on black / black on white
- S8  Colour  glyphs mono / glyphs take the cell's chroma
- S9  Mosh    displaces the video / displaces whole glyph cells
- S10 Shear   triangle wave / random per-line tear
- S11 Glyphs  ASCII ramp / block-dither set
- P12 Reveal  video (moshed, sheared) .. glyph rendering

Line buffer read a line late with the mosh offset; on each band's first line the cell luma mean and chroma are written to a two-bank cell RAM (read one band later); glyph = 8x8 ROM row indexed by 16 density levels. Shear is applied on the output side through a 64-deep ring so the glyph raster tears too.  Latency 53 clocks, all modes, blanking-gated.

## gouache
Gouache: catalogue combo #48 (seed 20260920) -- posterization (bit-depth reduction per channel) + hue rotation / saturation crush-boost.  Three knobs each; the switches and the slider decide how they combine.

- K1 Luma     luma levels 1..8 bits
- K2 Chroma   chroma levels 1..8 bits
- K3 Dither   dither amount before quantizing (0..100% of a step)
- K4 Hue      base hue rotation 0..360 degrees
- K5 Sat      saturation 0..400% (100% = unity)
- K6 Spread   extra hue rotation per luma band (rainbow bands)
- S7  Order   posterize then rotate / rotate then posterize
- S8  Sat     gain before the quantizer (crush = fewer bands) / after
- S9  Bands   hue spread keyed by luma band / by chroma (U) band
- S10 Dither  Bayer 4x4 / noise
- S11 Invert  alternate luma bands are negative
- P12 Spin    hue rotation speed, bipolar (centre = still)

Per-pixel chain, no line memory.  Rotation = sine-ROM cos/sin, four 11x10 products.  Latency 18 clocks, all modes, blanking-gated.

## dropout
Dropout: catalogue combo #50 (seed 20260920) -- chromatic glitch / datamosh bars + head-switch noise band.  Three knobs each; the switches and the slider decide how they combine.

- K1 Bars     fraction of bands displaced (datamosh)
- K2 Distance displacement (up to +-512 px, hashed sign)
- K3 Hold     frames a displacement pattern is kept (1..64, P-frame repeat)
- K4 Band     head-switch band height (0..255 lines)
- K5 Noise    noise amplitude in the band
- K6 Skew     head-switch flagging: rows in the band shift progressively
- S7  Roll    band fixed at the bottom / rolling upward
- S8  Dropout displaced bars lose their chroma
- S9  Smear   displaced bars hold 16-px runs (blocky smear) / plain shift
- S10 Burst   bands touching the head-switch band are always displaced
- S11 Noise   white snow / per-line noise lines
- P12 Bands   band height 8 .. 128 lines

Line buffer read a line late; per-band hashed offset (seed refreshed every Hold frames), head-switch skew ramp and noise gate per line.  Latency 16 clocks, all modes, blanking-gated.

## flutter
Flutter: catalogue combo #60 (seed 20260921) -- magnetic warp (wobble) + posterization (bit-depth reduction per channel).  Three knobs each; the switches and the slider decide how they combine.

- K1 Wobble   displacement amplitude (+-255 px)
- K2 Waves    vertical wavelength (phase step per line)
- K3 Speed    wobble speed, bipolar
- K4 Luma     luma levels 1..8 bits
- K5 Chroma   chroma levels 1..8 bits
- K6 Spread   per-luma-band phase (or amplitude) spread
- S7  Bands   bands differ in phase / in amplitude
- S8  Order   posterize the wobbled picture / wobble the posterized picture
- S9  Chroma  chroma wobbles with luma / a quarter wave apart (fringes)
- S10 Edges   outside the line: black / mirrored
- S11 Dither  plain quantizer / 2x2 dithered quantizer
- P12 Twist   horizontal phase term: 0 = pure line wobble, up = 2-D ripple

The input pixel's luma band picks the phase; sine ROM per pixel; line buffer read a line late at x + offset (separate chroma address). Latency 14 clocks, all modes, blanking-gated.

## snowglobe
Snowglobe: catalogue combo #68 (seed 20260922) -- pixelation + RF noise / snow / VHS tracking + polar warp (sphere / tunnel).  Two knobs each; the switches and the slider decide how they combine.

- K1 Cell     cell width 2 / 4 / 8 / 16 / 32 / 64 px
- K2 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 lines
- K3 Snow     snow amount
- K4 Tracking fraction of lines that lose tracking (torn and dark)
- K5 Bulge    lens strength, bipolar (pinch .. bulge)
- K6 Radius   lens radius in lines
- S7  Cells   cells in lens (source) space / in screen space
- S8  Snow    per pixel / held per cell
- S9  Lens    sphere / tunnel
- S10 Edges   outside the line: black / mirrored
- S11 Snow    mono / coloured
- P12 Zoom    overall scale 50 .. 200%

The line buffer is written only on the first line of each row block and read from the other block-parity bank; the lens DDA address is truncated to cells.  Latency 20 clocks, all modes, blanking-gated.

## vhold
Vhold: catalogue combo #73 (seed 20260922) -- rolling shutter / sync roll + sync loss / horizontal collapse.  Three knobs each; the switches and the slider decide how they combine.

- K1 Roll     vertical-hold roll speed, bipolar (centre = locked)
- K2 Bar      blanking bar height (0..255 lines)
- K3 Jello    rows lean by a sine of row + time
- K4 Collapse base squeeze of every row (0 = full width)
- K5 Reach    how far below the seam the extra squeeze reaches
- K6 Beam     collapsed rows brighten (beam concentrates)
- S7  Bar     black / the squeezed picture shows through the bar
- S8  Centre  rows collapse toward the screen centre / toward the jello offset
- S9  Edges   outside the line: black / mirrored
- S10 Tear    rows below the bar tear sideways (hashed per line)
- S11 Pull    seam pull squeezes / stretches (negative pull)
- P12 Pull    extra squeeze at the seam, decaying with Reach

Roll seam per frame (rollover), per-line squeeze factor from the seam distance, DDA line-buffer read.  Latency 15 clocks, all modes, blanking-gated.

## negspin
Negspin: catalogue combo #76 (seed 20260922) -- hue rotation / saturation crush-boost + invert / negative / bit-plane isolation.  Three knobs each; the switches and the slider decide how they combine.

- K1 Hue      base hue rotation 0..360 degrees
- K2 Sat      saturation 0..400% (100% = unity)
- K3 Spread   extra hue rotation per luma band
- K4 Plane    0 = negative, 1..8 = luma bit-plane isolated
- K5 Mix      plane picture mixed over the original (0..100%)
- K6 Chroma   chroma bit-plane depth (0 = off, 1..7 = planes kept)
- S7  Luma    luma as is / negative
- S8  Chroma  chroma as is / negative (hue + 180)
- S9  Flip    the selected plane bit flips the hue by 180 degrees
- S10 Source  plane taken from luma / from saturation
- S11 Order   plane before the rotation / after
- P12 Spin    hue rotation speed, bipolar (centre = still)

Per-pixel chain, sine-ROM rotation (four 11x10 products), no line memory. Latency 18 clocks, all modes, blanking-gated.

## screenprint
Screenprint: catalogue combo #77 (seed 20260922) -- ASCII / dither-art / halftone + multi-source mixing (add, multiply, screen, difference, overlay).  Three knobs each; the switches and the slider decide how they combine.

- K1 Cell     halftone cell 4 / 8 / 16 / 32 px
- K2 Angle    screen angle: axis / 45 degrees / steep
- K3 Dot      dot shape: circle / diamond / square / line
- K4 Blend    add / screen / multiply / difference / overlay
- K5 Ink      ink hue (8 hues)
- K6 Gain     halftone contrast
- S7  Ink     black ink on white paper / coloured ink
- S8  Invert  halftone inverted
- S9  Order   halftone over picture / picture over halftone (operand swap)
- S10 Chroma  picture chroma / ink chroma where there is ink
- S11 Rosette a second screen at another angle (rosette)
- P12 Mix     10 .. 100% of the blend

Per-pixel chain, no line memory: dot screens are cell-local sums of squares / abs values against a gain-scaled luma; blend modes are two 10x10 products.  Latency 17 clocks, all modes, blanking-gated.

## vhsmix
Vhsmix: catalogue combo #87 (seed 20260923) -- posterization + head-switch noise band + multi-source mixing (add, multiply, screen, difference, overlay).  Two knobs each; the switches and the slider decide how they combine.

- K1 Luma     posterize luma 1..8 bits
- K2 Chroma   posterize chroma 1..8 bits
- K3 Band     head-switch band height (0..255 lines)
- K4 Noise    noise amplitude in the band
- K5 Blend    add / screen / multiply / difference / overlay
- K6 Mix      blend amount (the posterized picture mixed onto the original)
- S7  Order   blend(posterized, original) / blend(original, posterized)
- S8  Band    band shows the raw blend / the band is the posterized negative
- S9  Roll    band fixed at the bottom / rolling upward
- S10 Noise   noise on luma / noise on chroma too
- S11 Chroma  blended chroma from the posterized picture / kept original
- P12 Static  noise fills the whole picture (0 = band only)

Per-pixel chain, no line memory.  Latency 15 clocks, all modes, blanking-gated.

## cellcam
Cellcam: catalogue combo #86 (seed 20260923) -- edge detect + chroma subsampling artifacts + pixelation.  Two knobs each; the switches and the slider decide how they combine.

- K1 Cell     cell width 2 / 4 / 8 / 16 / 32 / 64 px
- K2 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 lines
- K3 Gain     edge gain
- K4 Width    edge line width 1 / 2 / 4 / 8 px
- K5 Hold     chroma sample-and-hold 1 / 2 / 4 / 8 / 16 px
- K6 Lag      chroma delayed 0..15 px behind luma
- S7  Edges   drawn over the cells / cells replaced by edges on black
- S8  Source  edges between cells / edges of the raw picture (gradient)
- S9  Chroma  chroma held per cell / held by the Hold knob only
- S10 Ink     edges black / edges white
- S11 Chroma  edges keep chroma / edges mono
- P12 Crawl   NTSC dot crawl on the chroma edges (0 = clean)

Block-held line buffer (written on the first line of each row block, banks by block parity) gives the pixelated picture; cell-to-cell luma steps make the edges.  Latency 16 clocks, all modes, blanking-gated.

## solarcell
Solarcell: catalogue combo #81 (seed 20260923) -- Sabattier solarization + mosaic with per-cell colour averaging.  Three knobs each; the switches and the slider decide how they combine.

- K1 Threshold solarize threshold (fold above it)
- K2 Gain     fold depth
- K3 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 lines
- K4 Cell     cell width 4 / 8 / 16 / 32 / 64 px
- K5 Mix      mosaic mixed over the picture (0..100%)
- K6 Offset   solarize threshold offset per cell row (staircase)
- S7  Chroma  solarize luma only / chroma inverted in the fold
- S8  Order   solarize the mosaic / mosaic of the solarized picture
- S9  Average luma and chroma averaged / luma averaged, chroma sampled
- S10 Grid    seamless / thin dark cell borders
- S11 Iso     black lines where the fold crosses a period
- P12 Fold    threshold sweep: the fold moves through the picture

Cells are averaged horizontally on each row block's first line (running sum reset per cell) into a two-bank cell RAM (read by cell index for the whole block); solar_fold before the write or after the read.  Latency 18 clocks, all modes, blanking-gated.

## hallway
Hallway: catalogue combo #84 (seed 20260923) -- split-screen / quad + lens distortion (barrel, chromatic aberration) + mirror / kaleidoscope. Two knobs each; the switches and the slider decide how they combine.

- K1 Panes    1 / 2 / 3 / 4 panes
- K2 Wipe     pane boundaries slide across the screen
- K3 Barrel   lens strength, bipolar (pincushion .. barrel)
- K4 Fringe   per-channel lens spread (chromatic aberration)
- K5 Segment  mirror segment 1024 / 512 / 256 / 128 / 64 px
- K6 Slide    slide speed of the mirrored strip, bipolar
- S7  Mirror  odd panes mirrored
- S8  Lens    lens on the whole screen / one lens per pane
- S9  Fold    mirror (triangle) / repeat (sawtooth)
- S10 Edges   outside the line: black / mirrored
- S11 Fringe  U wide / V wide
- P12 Depth   nested folds 1..4 (each halves the segment)

Pane position -> nested power-of-two folds -> per-channel lens DDAs (three read addresses).  Latency 25 clocks, all modes, blanking-gated.

## hatchglass
Hatchglass: catalogue combo #93 (seed 20260924) -- mirror / kaleidoscope + crosshatch / stipple + chroma / luma key.  Two knobs each; the switches and the slider decide how they combine.

- K1 Segment  mirror segment width 1024 / 512 / 256 / 128 / 64 px
- K2 Slide    slide speed of the mirrored strip, bipolar
- K3 Weight   hatch line weight (1..8 px)
- K4 Pitch    hatch spacing 4 / 8 / 16 / 32 px
- K5 Level    key threshold
- K6 Soft     key edge softness
- S7  Key     luma / saturation
- S8  Invert  key inverted
- S9  Hatch   hatching where keyed / the mirrored picture where keyed
- S10 Paper   hatch on white / hatch over the mirrored picture
- S11 Fold    mirror (triangle) / repeat (sawtooth tiles)
- P12 Depth   nested folds 1..4 (each halves the segment)

Nested power-of-two folds (registered per-frame constants); the reflected luma picks 0..4 hatch directions; a luma or sat key from the dry pixel chooses hatch or picture.  Latency 25 clocks, all modes, blanking-gated.

## ridgeline
Ridgeline: catalogue combo #98 (seed 20260924) -- Perlin / simplex displacement + edge detect (Roberts / gradient).  Three knobs each; the switches and the slider decide how they combine.

- K1 Amount   noise displacement amplitude (+-255 px)
- K2 Scale    noise frequency across x
- K3 Speed    noise drift speed
- K4 Gain     edge gain
- K5 Edges    Roberts / horizontal / vertical / Sobel-lite
- K6 Ink      edge ink luma (0 = black lines .. 100 = white lines)
- S7  Edges   drawn over the displaced picture / edges only on black
- S8  Order   edges of the displaced picture / displaced edges of the original
- S9  Chroma  chroma follows luma / chroma on its own noise phase
- S10 Edges   edges also push the noise (edge magnitude adds to the offset)
- S11 Octaves one / two octaves
- P12 Rows    noise frequency down the lines (0 = pure column waves)

Sine-ROM noise into the line-buffer read address; Roberts cross against the input line.  Latency 19 clocks, all modes, blanking-gated.

## shearposter
Shearposter: catalogue combo #110 (seed 20260925) -- pixelation + scanline shear / horizontal tearing + posterization.  Two knobs each; the switches and the slider decide how they combine.

- K1 Cell     cell width 2 / 4 / 8 / 16 / 32 / 64 px
- K2 Rows     cell height 1 / 2 / 4 / 8 / 16 / 32 lines
- K3 Shear    per-line shear amplitude (+-255 px)
- K4 Rate     shear wave rate (Sine) / tear probability (Tear)
- K5 Luma     posterize luma 1..8 bits
- K6 Chroma   posterize chroma 1..8 bits
- S7  Shear   sine wave / random tear per line
- S8  Shear   shifts whole cells (cell-aligned) / shifts freely
- S9  Cells   cells shear with the rows / cells stay put, content shears
- S10 Edges   outside the line: black / mirrored
- S11 Chroma  chroma posterized too / clean
- P12 Pitch   posterize dither cell 1 / 2 / 4 / 8 px (2x2 dither scaled)

Block-held line buffer (first line of each row block, banks by block parity) read at x + shear, cell-held address, then a dithered quantizer. Latency 16 clocks, all modes, blanking-gated.

## posterduo
Posterduo: catalogue combo #112 (seed 20260925) -- posterization + duotone / tritone mapping from luma.  Three knobs each; the switches and the slider decide how they combine.

- K1 Luma     posterize luma 1..8 bits
- K2 Dither   2x2 dither amount before quantizing (0..100% of a step)
- K3 Pitch    dither cell 1 / 2 / 4 / 8 px
- K4 Shadow   shadow ink hue (8 hues)
- K5 Light    highlight ink hue (8 hues)
- K6 Contrast tone curve contrast (0..400%)
- S7  Order   posterize then tone (banded inks) / tone then posterize the inks
- S8  Tritone third (neutral) tone in the midtones
- S9  Luma    luma kept / flattened to the tone ramp
- S10 Bands   alternate posterize bands swap the inks
- S11 Chroma  ink chroma posterized too (with Tone-Post)
- P12 Balance midpoint between the two inks sweeps through the picture

Per-pixel chain: dithered quantizer around a duotone entity (7 clocks). Latency 14 clocks, all modes, blanking-gated.

## hueflutter
Hueflutter: catalogue combo #111 (seed 20260925) -- hue rotation / saturation + magnetic warp (wobble) + ordered dithering.  Two knobs each; the switches and the slider decide how they combine.

- K1 Wobble   displacement amplitude (+-255 px)
- K2 Waves    vertical wavelength (phase step per line)
- K3 Hue      base hue rotation 0..360 degrees
- K4 Sat      saturation 0..400% (100% = unity)
- K5 Luma     dither levels 1..8 bits
- K6 Chroma   chroma levels 1..8 bits
- S7  Bands   luma bands wobble with different phases / amplitudes
- S8  Order   dither the wobbled picture / wobble the dithered picture
- S9  Chroma  chroma wobbles with luma / a quarter wave apart
- S10 Hue     hue follows the wobble phase (rainbow ripples) / static
- S11 Dither  plain quantizer / 2x2 dithered quantizer
- P12 Spin    hue rotation speed, bipolar (centre = still)

The input pixel's luma band picks the wobble phase; sine ROM per pixel; the fetched chroma is hue-rotated (sine ROM) and scaled, then dithered. Latency 21 clocks, all modes, blanking-gated.

## lenscreen
Lenscreen: catalogue combo #106 (seed 20260925) -- composite luma/chroma crosstalk + halftone + lens (sphere / tunnel).  Two knobs each; the switches and the slider decide how they combine.

- K1 Bulge    lens strength, bipolar (pinch .. bulge)
- K2 Radius   lens radius in lines
- K3 Cell     halftone cell 4 / 8 / 16 / 32 px
- K4 Dot      dot shape: circle / diamond / square / line
- K5 Cross-Y  chroma leaking into luma (dot crawl) strength
- K6 Cross-C  fine luma detail leaking into chroma strength
- S7  Lens    sphere / tunnel
- S8  Screen  dots on the screen grid / dots warped with the lens
- S9  Ink     black dots on white / dots take the picture colour
- S10 Cross-Y rattles the dots (before the screen) / flickers the print
- S11 Fringe  cross-colour phase alternates per line (rainbow comb)
- P12 Zoom    overall scale 50 .. 200%

Lens DDA fetch from a dual-bank line buffer; the halftone screen is a cell-local distance test against the luma; crosstalk is two small products on the fetched pixel.  Latency 17 clocks, all modes, blanking-gated.

## panemix
Panemix: catalogue combo #105 (seed 20260925) -- split-screen / quad + multi-source mixing (add, screen, multiply, difference, overlay) + false colour (thermal-style LUT).  Two knobs each; the switches and the slider decide how they combine.

- K1 Panes    1 / 2 / 3 / 4 panes
- K2 Wipe     pane boundaries slide across the screen
- K3 Blend    add / screen / multiply / difference / overlay
- K4 Mix      0 .. 100% of the blend between the panes and the picture
- K5 Palette  thermal / ice / plasma / x-ray
- K6 Range    gain into the palette (contrast)
- S7  Mirror  odd panes mirrored
- S8  Luma    palette luma / the blend's own luma kept
- S9  Index   palette indexed by the blend luma / by |pane - picture|
- S10 Invert  palette reversed
- S11 Iso     black isotherm lines at palette stop boundaries
- P12 Heat    the palette cycles with time; speed = slider

Operand A = the pane-mapped picture (line buffer, a line late), operand B = the picture itself; two 10x10 products give all five blend modes; the result indexes a 4 x 64 palette ROM.  Latency 25 clocks, all modes, blanking-gated.

## trapquant
Trapquant: catalogue combo #120 (seed 20260926) -- perspective keystone / corner-pin + posterization (bit-depth reduction per channel).  Three knobs each; the switches and the slider decide how they combine.

- K1 Top      width of the picture at the top (25..225%)
- K2 Bottom   width of the picture at the bottom (25..225%)
- K3 Slide    horizontal slide, bipolar
- K4 Luma     posterize luma 1..8 bits
- K5 Chroma   posterize chroma 1..8 bits
- K6 Dither   dither amount before quantizing (0..100% of a step)
- S7  Edges   outside the picture: black / mirrored
- S8  Order   posterize the source (bands stretch with the keystone) / posterize the screen (bands stay put)
- S9  Ramp    the quantizer offset follows each row's scale: bands slide along the trapezoid
- S10 Dither  2x2 / alternate lines
- S11 Chroma  posterized / kept
- P12 Tilt    rotates the trapezoid: top wide .. bottom wide

Line buffer read a line late; per-line DDA source address (scale interpolated top->bottom with a per-frame serial divider); the dither amounts are per-frame products.  Latency 10 clocks, all modes, blanking-gated.

## foldither
Foldither: catalogue combo #117 (seed 20260926) -- ordered dithering (Bayer) + mirror / kaleidoscope.  Three knobs each; the switches and the slider decide how they combine.

- K1 Segment  mirror segment width 1024 / 512 / 256 / 128 / 64 px
- K2 Slide    slide speed of the mirrored strip, bipolar
- K3 Skew     per-line skew of the fold axis, bipolar (diagonal mirrors)
- K4 Bits     luma levels 1..6 bits
- K5 Dither   dither amount (0..100% of a step)
- K6 Matrix   2x2 / 4x4 / 8x8 Bayer / line screen
- S7  Fold    mirror (triangle) / repeat (sawtooth tiles)
- S8  Dither  in source space (the pattern folds with the picture) / in screen space (the pattern stays put)
- S9  Chroma  chroma dithered with the same matrix / untouched
- S10 Alt     alternate segments negative
- S11 Alt     alternate segments drop two more bits
- P12 Depth   nested folds 1..4 (each halves the segment)

Nested power-of-two folds (registered per-frame constants) on the read address; threshold dither + floor quantizer on the write port (source) or after the fetch (screen).  Latency 20 clocks, all modes, blanking-gated.

## tearcut
Tearcut: catalogue combo #115 (seed 20260926) -- datamosh / block displacement + scanline shear / horizontal tearing + edge detect.  Two knobs each; the switches and the slider decide how they combine.

- K1 Gain     edge gain
- K2 Thresh   edges below this are dropped
- K3 Block    block size 8 .. 64 px
- K4 Density  fraction of blocks moshed
- K5 Tear     tear amplitude (+-255 px)
- K6 Bands    tear band height 2 / 4 / 8 / 16 lines (Random) or the wave period (Wave)
- S7  Edges   white / tinted with the picture's colour
- S8  Tear    random per band / triangle wave down the picture
- S9  Blocks  plain / moshed blocks go negative
- S10 Mosh    shift / hold (block repeats its first column)
- S11 Ink     light edges on dark / dark edges on light
- P12 Blend   edge picture (0) .. the picture with edges over it (100)

Edge = |gx| + |gy| from the input and the Y line buffer, written as an 8-bit edge line and read back one line later at the block-displaced, torn address; the picture itself (dry ring) is never torn, so the edges slide over it.  Latency 18 clocks, all modes, blanking-gated.

## globeswap
Globeswap: catalogue combo #134 (seed 20260927) -- polar warp (spherize / tunnel) + colour channel swap / offset + solarization (Sabattier).  Two knobs each; the switches and the slider decide how they combine.

- K1  Lens     lens strength, bipolar (tunnel .. flat .. sphere)
- K2  Size     lens radius in lines
- K3  Swap     channel mode: none, U<>V, Y->U, U->Y, Y<>V, rotate, -Y, -YUV
- K4  Offset   per-channel column offset, bipolar (+-256 px chromatic split)
- K5  Solar    solarization threshold
- K6  Fold     fold gain 0.125x .. 8x (log); >1x repeats into bands
- S7  Fringe   even split across the picture / split grows toward the lens rim
- S8  Swap     channel swap everywhere / only inside the solar region
- S9  Split    U and V pulled apart, luma still / luma pulled against both chroma
- S10 Lens     row lens across the whole width / lens only inside the disc (the dry picture outside)
- S11 Fold     solar region folds luma only / also inverts its chroma
- P12 Globe    climax: grows the lens (sphere) and its radius, spreads the channels, drives the fold

Edge clamp: reads that leave the line, or land in its C_EDGE blanking columns, take the line's own edge pixel -- the fill continues the colour leading up to it, with no black seam. Per-row lens scale from a 64-entry sphere / tunnel table (per-line sequencer), DDA source column plus a per-channel offset, three edge-clamped addresses into a dual-bank line buffer (Y, U, V read from different columns); a 32-deep dry ring supplies the unlensed pixel outside the disc; solar_fold on the fetched pixel, then the swap.  Latency 18 clocks, all modes, blanking-gated.

## noisetone
Noisetone: catalogue combo #137 (seed 20260927) -- Perlin / value-noise displacement + gamma curves / S-curve contrast.  Three knobs each; the switches and the slider decide how they combine.

- K1  Scale    noise cell size 4 .. 512 px
- K2  Amount   displacement amplitude (up to +-255 px)
- K3  Drift    noise field scroll speed, bipolar (centre = still)
- K4  Curve    gamma 0.5, 0.7, 1.5, 2.2, soft S, hard S, inverse S, crush
- K5  Contrast linear gain about the pivot before the curve (25 .. 420%)
- K6  Pivot    grey level the contrast pivots about (0 .. 100%)
- S7  Patchy   one exposure everywhere / the noise field shifts the pivot (patchy exposure)
- S8  Displace all channels / luma only (colour stays put under the boiling luma)
- S9  Stretch  noise cells tall (vertical streaks) / wide (horizontal streaks)
- S10 Field    smooth value noise / hard cells (chunked tears)
- S11 Chroma   saturation flat / saturation follows the contrast gain
- P12 Storm    amplitude and contrast climb together: 100% is a boiling high-contrast field

Edge clamp: reads that leave the line, or land in its C_EDGE blanking columns, take the line's own edge pixel.  2-D value noise: per-line hashed row keys, four add/xor-shift hashes per pixel, bilinear lerp with Q7 fractions, driving a horizontal read of the dual-bank line buffer; the fetched luma is contrast-stretched about a (noise-shifted) pivot and mapped through an 8 x 128 entry EBR curve ROM. Latency 28 clocks, all modes, blanking-gated.

## stratamix
Stratamix: catalogue combo #132 (seed 20260927) -- depth-from-luma parallax + posterization + multi-source mixing (add, screen, multiply, difference, overlay).  Two knobs each; the switches and the slider decide how they combine.

- K1  Depth    parallax gain, bipolar (luma above the pivot shifts one way, below it the other)
- K2  Pivot    luma level that does not move
- K3  Levels   posterize luma to 1 .. 6 bits
- K4  Tint     chroma tint per luma band (the strata take alternating hues)
- K5  Mode     add / screen / multiply / difference / overlay
- K6  Mix      how far the blend replaces the displaced layer
- S7  Order    posterize before the shift (stepped depth: terraces) / after the mix
- S8  Layers   all strata shift the same way / alternate levels shift opposite ways
- S9  Chroma   colour from the displaced layer / from the still picture (colour ghosts)
- S10 Key      blend everywhere / only above the pivot (the background stays put)
- S11 Contour  plain / black contour lines at every level boundary
- P12 Strata   depth and mix rise together: at 100 % the picture separates into stepped terraces

Operand A = the line above read at x + (luma - pivot) * depth (edge-filled), operand B = the still picture from a dry EBR ring; two 10x10 products give all five modes; floor quantizers with registered mask / half run before the shift or after the mix.  Latency 23 clocks, all modes, blanking-gated.

## syncbleed
Syncbleed: catalogue combo #131 (seed 20260927) -- colour bleeding / NTSC dot-crawl emulation + sync loss / horizontal collapse.  Three knobs each; the switches and the slider decide how they combine.

- K1  Bleed    chroma smear length (IIR, 0 .. 128 px)
- K2  Lag      chroma lags luma by 0 .. 63 px
- K3  Crawl    dot-crawl amount (luma dots, strongest on colour)
- K4  Collapse base squeeze of every row toward the seam
- K5  Seam     collapse centre X, bipolar (centre = screen centre)
- K6  Wave     the squeeze breathes down the picture with a slow rolling wave
- S7  Bleed    before the collapse (smear squeezed too) / after (screen-width smear)
- S8  Length   fixed / scales with the local collapse (squeezed rows bleed more)
- S9  Chroma   collapses with luma / collapses more (colour fringes)
- S10 Crawl    locked checkerboard / drifting (inverts every frame)
- S11 Smear    chroma only / luma too (ghosting)
- P12 Loss     signal loss: collapse, bleed and crawl climb to a total-collapse smear

Per-line squeeze q = base + sine wave; luma and chroma run their own DDA read addresses (chroma with its own gain and a lag), both edge-filled; chroma IIR either on the write port or after the fetch; crawl adds saturation-scaled luma dots.  Latency 14 clocks, all modes, blanking-gated.
