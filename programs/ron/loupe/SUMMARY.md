# Loupe — a glass loupe over torn scallops

**v0.1.0 (2026-10-14): built, NOT flashed.** Made from a reference capture: an analog-synth
frame of white scallops pushing in from the right edge, nested in cyan and grey with concave
spikes between them, mirrored top to bottom. The bands are torn line by line into ragged edges
and yellow lightning, and a glass bubble in the middle holds a diamond of magenta/cyan
chevrons. Presets are hand-written placeholders.

## Reading of the reference
The reading is held up in an integer numpy preview, `~/loupe_work/model.py`, which imports the
palettes from `loupe.py`.
- **Field.** Every band edge is a contour of ONE distance field: the distance to the nearest of a
  column of points just off the right edge, horizontally stretched,
  `d = sqrt((0.30·(2160 − x))² + t²)`. Here `t` is the folded distance to the nearest point row
  (period 520 lines, a point on row 520).
  - Near the points, the contours are the white/cyan scallops.
  - Between neighbours, the outer contours form the concave grey spikes (the cusp of the union of
    circles).
  - Far away, they straighten into the near-vertical bands on the left.
  - The fold in y is the mirror.
  - Fit: the white core (d < 155 px) and the cyan ring are 97–100 % pure per distance bin
    (`fit.py`, `seq.py`).
- **Palette.** In order along d: white → cyan → grey, with a thin black/yellow band inside → dark
  → grey → lavender with magenta and dark-grey fringes → dark green → a cyan fringe → grey → dark
  → lavender/white/cyan → dark grey. The fringes are one or two palette entries.
- **Tear.** The tear is an offset of d per line, not of x. Where the field is nearly horizontal
  (along the spike edges), a small d offset makes a long horizontal streak, which turns the thin
  yellow band into lightning. The white/cyan edges in the reference are clean, so the tear ramps
  on only beyond the cyan/grey boundary.
- **Bubble.** A circle (radius ≈ 0.41 H) holds a 45° diamond (0.85 R, 0.11 R left of centre) of
  "<" chevrons, 30 px apart. Dense rim lines run along the diamond's edges, the field behind is
  washed toward white, and a thin grey ring marks the rim.
- **Ghost lines.** Faint pale ">" lines with their apexes on the cusp rows.

## What it does
1. **Field.** `s = a² + t²` (d-units², 4096 per frame height).
   - `a²` comes from an exact second-order DDA: `A2 += D; D += 2k²`. Here `a = kk·(Xc − x)`,
     `kk` is in d/64 per px, and Xc = 1.125 W.
   - `t²` is per line.
   - Twin: the line starts at the mirror of the right edge (D = K + 2KX, the distance growing).
     At the centre pixel the DDA holds A2 for one clock and turns D into 2K − D, so the right
     half runs forward. The field is mirrored about the centre at the cost of one subtractor.
2. **Square root.** Normalize s (26 bits, five shift steps) to 11 bits, then a 2048 × 11 ROM
   (the address parity holds the odd exponent), then denormalize. The result is d with 13 bits
   of integer.
3. **Tear.** `d' = d + n(y)·ramp(d − d0)`. The ramp is 0 below the onset, then rises in quarters
   over 128 units.
   - `n = je·tg/16`.
   - `je` is held on 7 lines in 8, redrawn as ±8, or as ±8..64 in bursts. Bursts start on 1 line
     in 32 and last 8–39 lines, and a ±2 jitter is added on every line.
   - `tg = p12²/4096`, and `d0 = 1300·(1 − p12²)`.
4. **Index.** `(d'·G16 >> 4) − flow`, 16 bits per palette cycle, mapped to 224 entries. G16 runs
   198..1977 (0.5×..5×, with 1× = one cycle over the fitted 698 px).
5. **Loupe.** The disc is tested against per-line chord half-widths: `sqrt((R² − by²)·ikb²)`,
   inner and outer rim, by the serial square root.
   - Facet: inside the diamond, the palette reads its 32 stripe entries at the chevron phase
     `bxd − |by|·ks + tear + flow`. In the rim band (0.125 R) the lines are blended half to olive.
     The rest of the disc is glass: half to white (Smoke: half to dark), with chroma halved.
   - Window: the disc shows the field UNTORN (the tear ramp is gated off inside it), washed a
     quarter; there are no chevrons or rim lines.
   - The ring blends half to grey.
6. **Ghost.** `g = kg·(Xc − x) + 56·t + c` (mod 2^16). Lines fall where `g < 768` (~1.2 % of a
   560 px period), outside the lens, and are blended a quarter to white.

| Control | Function |
|---|---|
| K1 Scallops | 1–6 points per frame height (square law; 2.08 in the reference) |
| K2 Stretch | horizontal stretch k = 1.0 (round) .. 0.08 (long spikes), square law; 0.30 in the reference |
| K3 Contours | 0.5–5 palette cycles over the field (square law; 1× at the default) |
| K4 Speed | Bipolar, still in the centre ±24. Positive makes the contours stream out of the points; the points drift slowly and the chevrons flow |
| K5 Lens | lens radius 0.12–0.62 frame heights |
| K6 Palette | Loupe (fitted to the reference) / Ember / Abyss / Toxic / Candy / Mono / Neon / Rust |
| S7 Lens | Facet (chevron diamond, the reference) / Window (a clean, untorn view through the glass) |
| S8 Twin | the field mirrored about the centre: two scallop walls meeting in a seam |
| S9 Drift | the lens drifts (W/512 + 1 px, 1 + H/1024 lines per field) and bounces inside the middle half of the frame |
| S10 Glass | Clear (washed out) / Smoke (darkened) |
| S11 Ghost | pale ghost chevron lines (default On) |
| P12 Tear | 0 = pristine scallops; raises the tear and pulls its onset in, so at 100 % even the white cores shred |

## Engine
- **Sequencer:** one serial shift-add multiplier, one serial divider and one serial square root.
  - The multiplier's op code and one-hot operand selects (27 a-sources, 25 b-sources, square
    flag) are registered every clock from the state number, with no enable (wraith's fix), and
    the fetch is a flat AND-OR.
  - Every multi-term constant (points, stretch, Contours gain, tear onset, diamond radius) is a
    constant multiply on the shared multiplier, not a shift-add chain, so no state chains more
    than one or two adds.
  - **Per field** (first vsync edge after active video, states 1–49):
    - 2^24/W and 2^20/H; the points, Pd = 2^20/points, and ky.
    - The stretch, kk and K = kk².
    - The Contours gain, speed², the mirror-row phase, the tear gain and onset.
    - The lens radius and centre (with the drift step and bounce), and the aspect factors.
    - The rim radii² and ikb², the stripe steps, the diamond start and limits, the ghost step and
      start.
    - The DDA start: a0 = kk·X, A2 = a0², D = K ∓ 2KX, with X = Xc, or W/8 + 1 for Twin.
  - **Per line** (states 96–118, kicked at the line's g_go, computing the NEXT line, ~200 clocks,
    so it fits SD's 858-clock line):
    - The fold and t².
    - The xorshift32 tear draw and n.
    - |by|² and the two chord square roots.
    - |by|·ks, the diamond limit, and the chevron and ghost line constants.
  - The pixel path loads the engine's `l_*` registers directly at g_go; there is no hand-over
    layer.
- **Glides:** one shared datapath over a rotating 5-channel bank (K1, K2, K3, K5, P12), run in an
  11-clock burst after each vsync edge.
  - Each channel takes two clocks: the hold (±4 deadband, rails snap), then the 1/8 ease (1/4 for
    P12) and a rotation.
  - The pot mux is registered a clock ahead.
- **Pixel path (23 stages):**
  - S0: the DDA (A2/D, with Twin's centre hold from a pre-registered flag), x, the diamond phase,
    the ghost phase.
  - S1: s = A2/4096 + t² (saturated), |x − Lx|, |bxd|, the chevron phase, the ghost test.
  - S2–S3: normalize (16/8, then 4/2/1), and the lens / diamond / rim tests.
  - S4: ROM address. S5: sqrt ROM. S6: fabric register. S7–S8: denormalize.
  - S9: tear onset. S10: ramp (gated off inside the Window lens). S11: torn d.
  - S12: ×G split hi/lo. S13: sum. S14: index − flow. S15: 224 map.
  - S16: palette or stripe address. S17: palette ROMs. S18: fabric register.
  - S19–S20: blends (ring > rim line > glass/smoke > ghost).
  - S21–S22: [1 2 1] smoothing. S23: luma 13/16 and the gate.
- **Lens flags** ({stripe, glass, rim line, ring, ghost} plus the stripe entry) are written at S3
  into a 256 × 16 block-RAM ring and read back 10 clocks later (S14, re-registered S15). The read
  never meets the write address, so the RAM carries `no_rw_check`. The blend flags then ride three
  FFs to S18.
- **ROMs:** sqrt 2048 × 11 (6 EBR), palettes 3 × 2048 × 8 (12 EBR), flag ring (1 EBR).
- **Contracts:**
  - damier's generated avid.
  - One 23-stage latency in every mode.
  - The [1 2 1] Y/U/V smoothing and 13/16 luma bar fix before the gate (crisp, torn, moving
    edges).
  - Every line DDA starts half a line down on the bottom field.
  - Palettes are U/V-swapped (`loupe.py`).
- **Fit history:**
  - The first cut had four DDAs: right/left columns, outside/inside a ×3 Magnify lens. It also had
    per-channel glides, a 17-stage FF flag pipe and an n_* hand-over layer. It came to 9218 LC
    (120 %).
  - Restructured to one DDA, with Twin by DDA reversal and the Window lens mode replacing
    Magnify. The glides moved to the shared bank and the flags to the EBR ring, and the drift
    became a bounce (no sine ROM): 7114 LC.
  - That first restructured build ran at 44 MHz; the culprit was the glide datapath in one clock.
    It now takes two clocks per channel, with the multi-term constants moved onto the multiplier.

## Build notes
- **v0.1.0 (2026-10-14), router2. Build seeds: HD Analog = seed 4** (seeds 1–3 missed), **HD HDMI =
  seed 2** (seed 1 missed). The other four are seed 1. Fmax (post-route): HD Analog 75.28,
  HD HDMI 76.11, HD Dual 76.91, SD Analog 73.86, SD HDMI 80.61, SD Dual 76.96 MHz (SD Fmin 27).
  ~7105/7680 LC (92.5 %), 19/32 EBR. Every bitstream passed iceunpack. The rebuild was
  deterministic: a second full build gave identical seeds and Fmax.
- **HD margins are thin** (75–77 MHz at 92 % LC). If a later version regresses:
  - Try `NEXTPNR_ROUTER="--router router1"` before restructuring (this engine family closed
    better on router1).
  - Then profile. The S12 ×Contours partial products (15×5 and 15×6 in one stage) are the next
    suspect; split G16 into 3-bit slices over two stages.
  - LC to reclaim if needed: the ×G multiplier as a 7-bit mantissa plus a 0–3 shift (−150), and
    narrower sequencer scratch registers.

## Working files
`~/loupe_work/`:
- `ref.jpg`: the reference.
- `fit.py`, `fit2.py`, `seq.py`, `common.py`: the distance-field fit and band order.
- `proto.py`: the first float sketch.
- `model.py`: the integer preview of the RTL pipeline.
- `mcmp.png`: the reference above, the model below.
