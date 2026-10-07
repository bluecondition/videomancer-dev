# Inferno — PIXELATED procedural flame (domain-warped multi-octave noise fire)

**v2.0.0 (2026-10-04): pixelated only.** Ron keeps Pyre as the realistic
simulator and Inferno as the deliberately fake, pixelated video fire (the two
programs stay completely separate).  Every screen coordinate feeding the
engine is quantized to a block grid, so each block carries one value: block
origin (qx, qy) from position-in-block counters replaces pixel x/y in the
world coordinates, the luma-mod slot read (7-px-delayed quantized column) and
the luma-mod root distance; the per-line gradient is held per block row by
subtracting the line-in-block.  K2 is **Pixels** (block size 4 + K2/32 px,
doubled on streams >= 1280 px wide: 8..70 px at 1080p, never finer), replacing
Width (its zoom multiply is gone).  Palette posterized to 32 steps, smoke to
16; embers are a per-block hash (block index, per-field seed) pipelined to
S15; all per-pixel LFSR dither (heat, overlay boundary) removed and the LFSR
instance deleted.

## What it does
Realistic procedural fire rising from the bottom of the frame in the classic
ramp: yellow at the base through orange and red to dark-red tips fading into
black, plus fine glowing filaments inside the flame. TWO superimposed fire
layers give the flame depth: a fast, detailed orange front layer over a wider,
slower, shorter DEEP-DARK-RED background layer whose settings are a planned
variation of the front one — it shows through the gaps between the orange
tongues (in Blue Flame mode the pair becomes blue gas over violet). Gray smoke rolls off the
flame tips (K3 sets amount/fade height). The
whole field rises with sub-pixel advection, shapes evolving as they climb, with a
slow lateral sway. A Blue Flame variant swaps in a gas-flame palette; Overlay
draws the fire opaquely over incoming video (smoke 50% see-through); Luma Mod
(S8) roots the fire
on the input video instead of the screen bottom — flames pour off the top edges
of bright objects (Canny edge detection), with everything else black.

## How it works
21-stage streaming pipeline at 1 px/clk, no divides:
- **World coordinates** in 1/16-px units; per-frame accumulators advect the flame
  upward (the domain-warp field rises at 3/4 speed and the finest octave 1.5×, so
  shapes morph as they rise). Lateral sway integrates a slow sine into a drift
  offset. K2 scales x around the screen centre (measured line width / 2,
  published once per field, rounded to 4 px so analog line-length wander
  can't jitter it) via one multiply (tongue width). K2's scale tops out at
  31/16 so a 1920-px line never exceeds the 16-bit world x (at the old 39/16
  the right ~240 px of an HD frame repeated the left edge).
- **Domain warp**: a low-frequency (64 px cell) value noise offsets x of all
  subsequent samples — real turbulence in place of the old sinusoidal wobble
  (a second y-warp noise looked marginally better but cost ~500 LCs; dropped
  for timing).
- **Value noise**: nonlinear 16-bit hash (2 adds + rotates/xors, bit-exact with
  the Python prototype `fireproto.py`; the hash is split across two pipeline
  stages, one 16-bit adder each — 1-add hashes showed diagonal weave), bilinear
  with smoothstepped fractions. Three octaves at 32×128 / 16×32 / 8×16 px cells
  (vertical stretch = tall flames); the finest octave is ridged (255 − 2|n−128|)
  for filaments. FBM weights 9/5/2.
  **The TALL cells (octave 1 = 128 px, layer 2 = 256 px) use a 64-step fine
  smoothstep (C_SS6), not the 16-step C_SS.** With 16 steps the interpolated
  value is constant for 8/16 px at a time and those terrace boundaries land on
  the SAME screen row in every column — horizontal contour lines across the
  whole flame (layer 2 was worst: its ridge fold doubles the step contrast and
  its heat had no dither). Layer 2's raw heat now carries the same ±4 LFSR
  dither layer 1 has.
  The y-lerps are split across two stages (S11a product / S11b add): chaining
  subtract→multiply→add in one stage was the design's critical path (~5 ns
  logic + 9 ns routing = 71 MHz). Splitting it bought ~6 MHz.
- **Heat field**: vertical gradient (1 heat unit per line; start value
  servo-locked each vsync so the bottom line hits `gbot = 260 + fuel/2` — no
  divide, adapts to any video mode) minus `(255 − fbm) · heatk/8` with
  raggedness coupled to the same knob (heatk = K1/64 + 10), plus ±4 LFSR dither.
  A soft knee (compress raw heat above 208 by 8) keeps the base textured
  instead of saturating flat at 100% fuel.
- **Smoke** (K3, NOT scaled by Intensify — see below): originates at the flame
  tips and rolls up off-screen,
  dissipating with altitude — `smoke = clip(min(ramp_in(−raw), wisp, 120) −
  att)` with `att = (255−S)/2 − raw/8` and `wisp = clip((fbm−110)·2)`. The
  subtractive attenuation makes weak wisps die low while strong ones survive
  to the frame top at high K3 (knob 0 = no smoke). Combined post-palette: gray
  wins where brighter than the fire pixel (chroma to neutral). Smoke luma is
  64 + smoke·4, i.e. it starts at video black (it used to start at 0, so its
  faint end sat below black). Shift/min ops only, no multiplies.
- **Palette**: 1024×30-bit BRAM ROM, 4×256-entry banks addressed by
  `blue & layer2 & heat` (generated by `genpal.py`, spliced into the VHDL by
  `splice_pal.py`), stored U/V-swapped for hardware — looks chroma-swapped in
  the sim. **Limited-range since v1.1**: luma 64 (black) .. 940 with a /2
  knee above 704 and a 768 cap, chroma ×896. The original full-range mapping
  (Y = y·1023, chroma ×1023; `genpal.py --legacy` reproduces it bit-exactly)
  put the background at Y=0 (below black), crushed the dim end of every ramp
  (24 fire / 40 dark-red / 46 blue entries under 64 — the faint layer-2 tails
  were invisible) and drove the blue core to 1008 (over white). Banks: fire (L1),
  deep dark red (L2), blue-flame (L1 with S9), violet (L2 with S9). Fire ramp
  (hot→cold): yellow → orange → red → dark red → black. The dark-red L2 ramp
  has no yellow at all so it never competes with layer 1's hot core — it reads
  as depth; the violet L2 ramp plays the same role behind the blue gas flame.
- **Luma Mod (S8)**: a streaming Canny detector at quarter horizontal resolution —
  4-px average + horizontal [1,2,1] Gaussian, 3×3 Sobel (its kernel embeds
  the perpendicular [1,2,1] smoothing), |gx|+|gy|/2 magnitude, double threshold
  (K5: THI 24..152, TLO = THI/2) with line-to-line + left-neighbour hysteresis.
  NMS is skipped: only edge presence matters (thickness is irrelevant). Roots
  are stored per 4-px column in up to 3 edge-slot maps (first two spaced
  edges + always the lowest, plus a bright-luma fallback root for edgeless
  bright columns).  Slot maps are DOUBLE-BANKED: display reads the bank
  completed last frame while the detector fills the other (single-banked maps
  were overwritten mid-frame — black bars under vertically stacked elements);
  the write bank is cleared during vblank.  When S8 is on, the
  fire gradient is rooted per pixel on that surface (slope 1/line above it,
  16/line glow skirt below), replacing the bottom-of-screen gradient; smoke
  then rolls off burning object tops automatically. The slot-map read runs
  7 px behind the raster so the 5-stage shading chain lands exactly at the
  gradient mux — delaying the ADDRESS replaced an 8-deep 13-bit result pipe. Line buffers: 2× 1024×8
  (Sobel rows) + 1024×4 hysteresis state + surface map.
- **Two fire layers**: layer 2 is a planned variation of layer 1 that shares
  the expensive parts. It reuses the same domain-warped coordinates but
  samples its noise cells one bit coarser in both axes with its own hash seed
  (64x256 px cells) — free coordinate math that yields wider tongues and, since
  the rise offset is baked into those coordinates, HALF the rise speed: a
  slower, more distant fire. Its gradient sits lower (shorter body), and its field is one octave folded with its own ridge
  (soft body + licking crease). The layers composite by **max in raw-heat
  space** (S14b) — the knee and clamp are monotone, so that equals maxing
  after them: the hotter layer wins (physically right for fire) for one 14-bit
  compare and no second palette read — the compare's result is also the
  layer-2 colour bit. **K5 Depth** sets how far lower:
  `g2off = 3/8·gbot − gbot·depk/256`, depk = K5/16 below centre and K5/8−32
  above (slope doubles where the change reads), so 3/8 (layer 2 hidden — it
  needs fbm2−fbm1 > ~78 to break through) → 1/4 at the default (~17%
  coverage in coherent patches) → 1/8 (~44%) → ~0 at full (level with
  layer 1, about half the flame deep red). The gbot·depk product is a fourth
  pass on the Intensify FSM's serial multiplier. With S8 Luma Mod on, K5 is
  Edge Sens instead and depth stays at the default. A fully
  independent layer would need its own 16 hashes (~1500 LCs); this costs ~700.
- **Overlay (S10)**: fire OVER video — fire is OPAQUE inside the flame envelope
  (raw heat ≥ 16), smoke blends 50/50 with the video (its chroma is neutral 512,
  so the blend is 256 + video/2), and the video passes through untouched
  everywhere else; the smoke/video boundary is LFSR-dithered so it doesn't cut
  a hard contour. Shifts and muxes only. (Was a saturating ADD, which tinted
  the whole picture instead of covering it.)
- **Intensify (P12)**: a shared serial multiplier FSM in vblank computes
  fuel/rise/turbulence x boost (1x..5x in 1/16ths). **Smoke is deliberately
  NOT boosted**: it used to be (smk*boost/64), but at 5x that is 1.23*K3, so
  the smoke hit its 255 ceiling by ~20% of K3's travel and the rest of the knob
  did nothing. K3 now owns the smoke extent alone and spans its full sweep at
  any Intensify setting. The FSM is sequential because combinational knob
  multiplies feeding vsync-latched registers are real STA paths. The
  input-video delay for Overlay is a 32-deep × 32-bit BRAM circular buffer,
  not a FF pipe (32-bit: a 30-bit word split across a 16+14 BRAM pair and
  corrupted the video on hardware).

## Controls
- **K1 Fuel** — flame height AND raggedness (former Heat knob folded in)
- **K2 Pixels** — block size 4 + K2/32 px (x2 on HD-width streams): 8..70 px at 1080p; default 16
- **K3 Smoke** — smoke amount / fadeout height above the flames
- **K4 Turblnce** — domain-warp amount
- **K5 Depth/Edge** — S8 off: Depth, how tall the deep-red background layer burns (hidden → level with the front fire); S8 on: Canny edge sensitivity (up = more edges)
- **K6 Rise** — advection speed (512 ≈ 4 px/frame)
- **T7 Ember** — random hot pops inside the mid-heat band
- **T8 Luma Mod** — fire pours off the top edges of bright input objects (Canny); black elsewhere (blend via Overlay)
- **T9 Blue Flame** — front layer goes blue/gas, background layer goes violet
- **T10 Overlay** — fire drawn over the incoming video: fire opaque, smoke 50% transparent, video elsewhere
- **T11 Bypass** — true passthrough: `data_in` and its OWN syncs go straight to the output (never touches the video-delay BRAM), identical to the SDK passthru program
- **Slider Intensify** — boosts fuel/height, raggedness, rise speed and turbulence together, 1x..5x (smoke is on K3 alone; output gain is fixed at full)

## Hardware contracts (v1.1)
- **Blanking gate**: y/u/v forced to 64/512/512 whenever the latency-aligned
  avid (tap C_LATENCY−2) is low. The fire pipeline free-runs; before this the
  hottest bottom fire was written into the blanking lines below the picture.
- **Serrated vsync**: every non-idempotent per-field action (gradient servo,
  slot-bank swap + clear, Intensify FSM / rise advance, sway, Canny frame tag,
  width-centre publish) fires on `s_vsedge` = the first vsync edge after
  active video. Unguarded, a second edge k lines later re-servoed the gradient
  against a k-line "frame" (top of screen → whole picture on fire) and an
  even edge count cancelled the bank swap (Luma Mod black). Plain resets stay
  on every edge.
- One sync-tap latency (21) in every mode; S11 Bypass uses data_in with its
  own syncs (0 latency, coherent — HW-confirmed clean).

## Status
**v2.0.0 pixelated rework — built, not flashed.**  v1.1.0 was FINAL except presets — limited-range palette chosen on hardware over
the legacy full-range one (2026-10-04). `splice_pal.py --legacy` still
regenerates the old ROM for reference.

## Build (v1.1.0, 2026-10-04)
**Build with router1, seed 7:** `NEXTPNR_ROUTER="--router router1" SEED=7 ./build_programs.sh ron inferno`.
At 7610-7631/7680 LC (99%), 23 EBR, router2 missed on seeds 1-6 and passed 1 of
8 in a sweep; router1 passed 7 of 8 (75.8-77.3 MHz). v1.1.0 results, all seed 7
on the first try: HD Analog 76.76, SD Analog pass, HD HDMI 78.28, SD HDMI 78.97,
HD Dual 76.97, SD Dual 74.71 MHz. (v1.0.0 router2 seeds were analog 4 / hdmi 1 /
dual 6.)

### v2.0.0 (2026-10-04)
`NEXTPNR_ROUTER="--router router1" SEED=7 ./build_programs.sh ron inferno`,
~7590 LC (99%), 23 EBR: HD Analog seed 7 PASS, SD Analog 74.21 (s7), HD HDMI
76.07 (s7), SD HDMI 78.15 (s7), HD Dual 74.56 (**seed 8**; 7 missed), SD Dual
68.54 (s7).  Packed to `out/rev_b/ron/inferno.vmprog`.
