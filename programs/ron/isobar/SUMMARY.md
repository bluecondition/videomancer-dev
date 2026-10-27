# Isobar — kaleidoscope contour colourizer

**v0.1.0 (2026-10-13): built, NOT flashed.** Made from a reference capture of an analog
colourizer: a mirrored psychedelic picture with a glowing blue heart in the middle, bands of
soft magenta/green contour stripes wrapped around it, V-shaped chevrons where the stripes cross
fold lines, smooth colour pools (orange glow, sky blue, deep red) between the bands, and bright
spotlight cones at the top. Presets are hand-written placeholders.

## Reading of the reference
Measured in `~/contour_ref_work/` (ref.jpg; the decomposition in `f_luma.png`, `f_stripe.png`,
`f_perp.png`; numpy protos `proto*.py`; the RTL-faithful integer model `imodel.py`).
- **The stripes are chroma-only.** Along any row, luma is a smooth ramp, and the stripes swing
  U/V along one fixed axis: magenta (U+40, V+45) ↔ yellow-green (U−50, V0) on the 8-bit scale,
  passing near neutral. The loop is a narrow ellipse, and each stripe is a soft square wave
  with ~10 px edges.
- **The midpoint drifts by region** (neutral at the sides, orange at the top centre). Where the
  luma is dark the same loop reads as red/purple.
- **Luma is a separate spotlight:** a linear tent in x peaking on the creases, minus a vertical
  fall-off. L ≈ peak(y) − 100·(1 − p) on a 0..255 scale, with the peak falling from 232 to 99
  down the frame.
- **Kaleidoscope:** a mirror at x = 960 plus creases near x = 520/1400. The chevrons and the
  nested diamonds sit on the fold lines.
- **Pools:** where the field is "inside" a level, the colour is a smooth gradient (the blue
  heart, with a red rim). Outside, the contour stripes are evenly spaced (~45 px), as with a
  distance field.

## What it does
| Control | Function |
|---|---|
| K1 Shape | Heart cross-term α from −2.5 to 5: bowtie, then oval, heart (default ≈ 2.5), tall spade |
| K2 Panes | Fold count K 0.5–4 (square law): one centre mirror up to many narrow panes |
| K3 Pools | Pool share of each ring, 6–85 % (all stripes up to big calm pools with thin bands) |
| K4 Warp | Sine bend of the contours (0 = geometric chevrons) |
| K5 Speed | Bipolar, still in the centre ±24. Right = rings flow outward from the heart, left = inward |
| K6 Palette | Psyche (fitted to the reference) / Ember / Ice / Acid |
| S7 Crease | Crease diamonds (a second field, min'd with the heart) Off / On |
| S8 Mirror | Bottom half mirrored from the top Off / On |
| S9 Pulse | Heartbeat (lub-dub at ~72 bpm) on the density: the rings pump Off / On |
| S10 Glow | Flat luma / Spot (the reference's spotlight tent) |
| S11 Etch | Stripes also cut into the luma (dark gaps on the green phase) Off / On |
| P12 Density | Rings across the picture, g 0.15–4 (square law). 0 = one calm pool filling the frame, 100 % = dense, vibrating contour rings everywhere |

## Engine
- **Panes:** X runs −K..K across the line (a 28-bit DDA, 2^-24 units, started half a step in so
  pixel centres mirror exactly). p = tri(X) comes straight from the fraction bits, with no abs,
  because the triangle is even: `p = X(22:11) xor X(23)`.
- **Field:** `F1 = β|yn − yc| + α(yn − y1)·p + 3p²`, with β 2.06, yc −0.055 and y1 0.09.
  - Per line: B = β|yn − yc| and A = α(yn − y1) come from the serial multiplier.
  - Per pixel: `S = A + 3p/4` (shift-add), then S × p (split lo/hi).
  - A 2048-entry sqrt ROM (fine range for F < 1, coarse up to 16) turns F into an
    even-spaced distance D1.
- **Crease:** `D2 = 2(1 − p) + 2|yn − 0.25|` (shift-add), and D = min(D1, D2).
- **Warp:** `W·sin(π p + 2π(0.7 yn + φ))`, an 8×8 product.
- **Rings:**
  - `m = D·g + φt` (split lo/hi). m(13:12) is the ring (palette row), m(11:0) the place in it.
  - pool = place < pf. One shared multiplier maps the place to 64 positions across the pool
    (× 2^18/pf) or across one of the 6 stripes (× 6·2^18/(1 − pf)); both reciprocals come from
    the serial divider per field.
- **Palette:** `isobar.py` generates 4 palettes × 4 rows × (stripe loop, pool gradient) × 64
  positions. Each entry holds a 4-bit luma offset and 8-bit Cr/Cb (U/V-swapped). Pools blend
  in from the previous ring's stripe colour and out into their own first stripe.
- **Luma:** `peak(y) − 5/16·(1 − p)` (Spot) or `peak(y) − 160` (Flat), with peak 764 at the top
  falling to 400 at the bottom. Then the palette offset and the Etch dip are applied, clamped to
  64..768. Chroma fades in over the first 62 codes above black.
- **Contracts:** damier's generated avid; one 30-stage latency in every mode; [1 2 1] Y/U/V
  smoothing plus 13/16 luma (the HD HDMI bar fix); the blanking gate; glides on K1–K4 and P12;
  the bottom field starts half a field step down; per-field work on the guarded vsync edge;
  sequencer operands fetched by op code (totem's lesson).
- **EBR (17):** sqrt 6, palette 10 (2048 × 4/8/8), sine 1.

## Build notes
- **v0.1.0 (2026-10-13), router2, 6/6 seed 1** (genuine first-seed passes, not retries).
  Fmax: HD Analog 79.8, HD HDMI 82.1, HD Dual 80.9, SD Analog 69.7, SD HDMI 79.2,
  SD Dual 80.5 MHz. ~6340/7680 LC (83 %), 17/32 EBR. All six bitstreams pass iceunpack;
  `out/rev_b/ron/isobar.vmprog` (438 KB, unsigned).
- **Luma, before the first build was finished:** the bottom quarter clamped to pure black, so
  the tent was made 5/16 (was 3/8) and the bottom peak raised to 400 (was 330). Chroma now
  reaches full strength 62 codes above black (was 124).
- **Warp:** the warp phase moves at 4× the ring speed in its own turns. At 1/4 it was
  effectively static.

## Working files
`~/contour_ref_work/`: `ref.jpg`, `proto.py`..`proto7.py` (float previews; proto7 is the
closest), `imodel.py` (an integer model of this RTL's pixel path, which reads the ROMs from
`isobar.py`: `python3 imodel.py out.png k12=700 pal=3 ...`).
