# LIVEWIRE — design proposal (pre-VHDL)

Status: **proposal**, awaiting sign-off on the four open questions at the end.
Target: HX4K, 74.25 MHz, `program_type = "processing"`, HD-first.

---

## 1. The two-plane architecture

The whole program is one streaming pipeline with two co-resident planes that
share every mode:

**SHARP plane — full resolution, in-frame, zero latency beyond the window.**
Line-buffered 3x3/5x5 luma window -> one Sobel -> magnitude, 360-degree
orientation, polarity. This draws everything *crisp*: tube cores, ink lines,
hatch strokes, contour lines, cel outlines, emboss lighting. Its spatial reach
is limited to the +-2..3 px the window gives.

**SOFT plane — decimated cell grid (~15 x 17 px on HD), frame-recursive.**
One BRAM holding an edge-energy field per cell. Each frame the field is
re-injected from the current edge map, diffused one cell outward in all four
directions, and decayed. Bilinear-upsampled back to full res, it is the single
source of *every* "how far the edge material reaches beyond the edge" quantity:
neon halo, Kirlian corona, ink bleed feather, burin hatch density reach, relief
ambient, cel glow suppression — and, with the decay coefficient turned down,
the K8 PERSIST trails. **One memory, one machine, eight uses.**

This split is forced by the platform, and it is also what makes 8 modes
affordable: a mode is not a renderer, it is a parameter set over a shared
primitive bank.

### Why spread must be frame-recursive (the one honest compromise)

Isotropic spread of radius R needs the picture R lines *ahead* of the beam.
Buffering ~4 luma lines is affordable; buffering 60 is not, and the program
must present one fixed latency in every mode. So any halo wider than ~3 px has
to be built across frames. Consequence: a large halo/corona/bleed establishes
at one cell (~15 px) per frame — a 120 px reach is fully lit ~8 frames (130 ms)
after an edge appears, and lags behind fast motion as a comet-tail. On stills it
settles exactly. This is not a bug to be fixed later; it is the platform, and it
is why PERSIST and SPREAD are the same hardware.

Field recursion (per cell, per frame):

    F_new = max( inject,  spread_gain * max(F_old neighbours),  persist * F_old )

`spread_gain` (K5) sets reach; `persist` (K8) sets trail length. They are
separable: short persist + high spread = a halo that hugs a moving edge with a
few frames of wake; high persist = slow ghosts.

---

## 2. Core engine

### 2.1 Scale (K2) = a luma pyramid blended *before* one Sobel
Rather than three detectors blended after the fact (3x the gradient hardware and
an orientation that changes meaning mid-blend), K2 crossfades the *source*:

    L0 raw luma  ->  L1 [1 2 1]^2 window blur  ->  L2 low-res field luma (bilinear)

Two lerps with per-frame weights (shifted-bit-term decomposition, doom v2), then
**one** Sobel. Continuous fine->coarse, one gradient engine, orientation stays
consistent across the whole sweep.

### 2.2 Magnitude, orientation, polarity
- magnitude: octagonal metric `max(|gx|,|gy|) + min(|gx|,|gy|)/2` (rollthebones)
  — more isotropic than L1, adds/shifts only.
- orientation: octant fold + 3 shift-approximated tangent compares (turpentine /
  intaglio) -> 16 bins over 360 degrees. The sign pair (gx,gy) carries polarity,
  so orientation is inherently signed: light->dark and dark->light are opposite
  bins, not the same line. S3 POLARITY simply gates one half of the wheel.
- alpha: soft ramp `clamp((mag - thr) * gain)` — the continuous magnitude is
  what gives genuine anti-aliasing at every scale; no binary edge map anywhere
  in the pixel path.
- real-video hygiene: +-hysteresis on the accept decision and run-length
  qualification (etchplate v3.2), or hand-held camera noise chains micro-edges
  into full-width bars.

### 2.3 Field engine (cell rate, one BRAM port time-multiplexed)
~15 clocks per cell, so one EBR port serves 3 accesses per cell: read
`old(r+1,c)` (upward spread source), the prefetched read of `old(r,c+1)`
(rightward + display), and the write of `new(r,c)`. `new(r-1,c)` comes from a
128-entry row-above buffer (1 EBR); `old(r-1,c)` from a matching display-row
buffer. Bilinear upsample uses `x mod 16` / `y mod 17` as lerp weights (three
4-bit lerps).

BRAM: field 8192 x 5b ~ 10 EBR (4b = 8), plus 2 row buffers, plus 2 luma line
buffers (2048x8) and the dry-video ring. Estimate **20-24 / 32 EBR** — under the
30-EBR congestion cliff, above the 20 "comfortable" line.

Single-bank read-modify-write is the risk here (yosys will not infer BRAM for a
same-cycle RMW). Ours is not same-cycle: the read of a cell and its write are
~10 clocks apart on one port, with registers in between. Tell if it goes wrong:
EBR count collapses and LC explodes; fallback is c64's ping-pong-by-frame-parity
at 2x the EBR, or a 64-wide grid.

---

## 3. Shared primitive bank (what the modes are actually built from)

| Primitive | Source | Used by |
|---|---|---|
| core / body / edge alpha (3 soft thresholds; K4 + EMERGE **move the thresholds**, not a gain) | sharp | NEON, INK, CEL, KIRLIAN, CONTOUR |
| upsampled field + per-mode curve | soft | NEON halo, KIRLIAN corona, INK bleed, BURIN density, PERSIST |
| stripe-field accumulators (`p_k = x*cos_k + y*sin_k`, multiply-free, intaglio) | sharp | BURIN hatch, KIRLIAN filaments, INK paper fibre |
| posterized luma + level-boundary detect (the same Sobel run on the quantized luma) | sharp | CONTOUR iso-lines, CEL flat regions |
| directional derivative `gx*cosL + gy*sinL` (16-angle shift-add case) | sharp | RELIEF |
| orientation -> hue (16-entry palette LUT) | sharp | SPECTRAL, S4 overlay |
| tintable palette through one shared sine path (polka) | per-frame | every mode's ink/ground/halo colours from K6 |

Modes then cost mostly *muxes and constants*, not new datapaths. Notable: CONTOUR
and CEL are the same posterizer with different line weights and fills; SPECTRAL
is the orientation LUT with the field as glow; RELIEF is two products.

---

## 4. Controls

The hardware has **6 rotaries, 5 toggles, 1 slider** — the brief asks for 8
knobs. Proposed fold (see Q1):

- **K1 MODE** — 8 modes, crossfaded at boundaries (the crossfade is a compose-stage
  lerp between two parameter sets, so it costs one lerp, not two renderers).
- **K2 SCALE** — pyramid blend, continuous.
- **K3 THRESHOLD** — edge sensitivity.
- **K4 MATERIAL** — line weight *and* spread reach together (WEIGHT + SPREAD merged;
  EMERGE already sweeps both, so these are base trims).
- **K5 LIGHT / LEVELS** — mode-dependent second parameter: relief light angle,
  contour count, cel levels, burin hatch angle, kirlian branch rate, neon flicker.
- **K6 COLOR** — mode palette / hue-wheel offset.
- **S1 INVERT · S2 INTERIOR · S3 POLARITY · S4 SPECTRAL OVERLAY · S5 PERSIST**
  (GLIDE becomes always-on: pot noise is visible without it, and a
  glide-off switch reads as "broken" more than as a feature).
- **P12 EMERGE** — the signature sweep, glided, 0-100% as specified: clean video ->
  coarse edges over video -> interior fades to the mode's ground -> full weight ->
  overdrive (thresholds collapse, field gain and weight overshoot, orientation
  effects exaggerate) until the frame is edge-material.

Every knob moves the picture at every position (no zone gating), per the
standing rule.

---

## 5. Build order

1. Core: window, pyramid, Sobel, magnitude/orientation/polarity, EMERGE, dry mix,
   blanking gate, latency contract. Render as SPECTRAL + NEON only. **Build, fit,
   flash** — this proves the core and the timing headroom before any mode lands.
2. Field engine + upsample. Prove PERSIST and NEON halo on hardware.
3. Modes in cheapness order: SPECTRAL, RELIEF, CEL, CONTOUR, NEON, INK, KIRLIAN,
   BURIN (BURIN last — intaglio's whole program was 55% LC on its own).
4. Fit checkpoints after each mode; the go/no-go conversation happens at the first
   config that misses, not at the end.

Honest fit estimate: core ~2000 LC, field ~900, primitives ~1400, 8 mode composes
~2000, palette/EMERGE/glue ~900 -> **~7200 of 7680 LC**. That is over the ~94%
routed-closure guideline. Expect to trade something; Q2 decides what.

---

## 6. Decisions taken (2026-09-02)

1. **Knobs**: K4 merges WEIGHT+SPREAD into MATERIAL, freeing K5 for LIGHT/LEVELS.
   PERSIST moves to **S5**; GLIDE is always on.
2. **Spread is frame-recursive** and its motion lag is accepted.
3. **Fit fallback** (only if 8 modes don't fit): merge CONTOUR+CEL into one POSTER
   mode and demote SPECTRAL to the S4 overlay it already is.
4. **Python look-dev first** -- done, see section 7.

## 7. What the look-dev settled (lookdev/livewire_look.py)

Renders: `/home/ron/livewire_work/{modes,emerge_neon,emerge_ink,motion}.png`.
The model is deliberately implementable -- one Sobel, no divides outside a small
reciprocal ROM, no frame buffer. What it changed in the design:

- **Per-scale gain is mandatory.** Blurring the source shrinks its gradient, so
  without a normalising gain (1x fine -> ~4.4x coarse, a per-frame shift) the
  SCALE knob fades the picture out instead of changing what it sees, and the
  field never injects at all. This was the first real bug the model caught.
- **Take the coarse gradient BETWEEN CELLS, then interpolate it.** Taking it from
  the upsampled coarse luma facets every coarse edge into visible stair steps
  (the gradient of a bilinear surface is piecewise constant). Cost: two extra
  lerp chains, and it is the difference between a silhouette and a staircase.
- **Every band of the material is a MULTIPLE of one threshold**: k=0.55 outer
  glow, k=1 body, k=3 core, with a soft AA shoulder. WEIGHT moves that one
  threshold (lagoon). Composing bands as independent additive layers instead
  bleached every mode to white at full resolution.
- **Each mode carries a native SCALE bias** (K2 offsets it): NEON 0.54, BURIN
  0.62, KIRLIAN 0.90, RELIEF 0.46, CONTOUR 0.86, SPECTRAL 0.34, INK 0.58,
  CEL 0.80. Without this the fur-scale detail of real video drowns the print
  modes and every mode converges on "edge detection".
- **Each mode also carries its own field character** (inject threshold multiple,
  spread bonus): a halo needs room, so KIRLIAN injects only from strong coarse
  boundaries while BURIN injects almost everything at a short reach.
- **EMERGE 0 is a true dry lerp** (exact passthrough), the threshold drops fast
  in the first quarter so silhouettes are already drawing at ~12%, and overdrive
  RAISES the core multiple as it thickens -- otherwise 100% is a white frame
  rather than a frame eaten by its own outlines.

### PERSIST: confirmed, with one correction

The naive form `F = max(inject, spread*neighbours, persist*F)` makes PERSIST a
**no-op**: whenever spread > persist the halo term wins everywhere a wake would
be, so a long trail is invisible inside a big glow. The fix is that persistence
does not add to the field, it changes what the field IS -- turning PERSIST up
cuts neighbour coupling (x0.55) and raises self-retention to 0.93, so the glow
tightens and a comet wake appears behind moving subjects. One memory, two
characters, and S5 becomes a genuine look-flip.

Confirmed by `lookdev/motion_test.py` (a 22-frame pan, one field step per frame,
exactly the hardware recursion): the wake is real and reads as motion. It is
**soft and cell-blocky (15x17 px), not a sharp ghost image** -- sharp echoes
would need a full-res frame buffer, which does not exist on this part. INK BLEED
uses the same field at short reach with fibre modulation and an
orientation-biased tap, so it needs no separate persistence machinery.


---

## 8. RTL notes (v0.1, core + NEON/SPECTRAL/RELIEF)

Pipeline, 17 stages, one latency in every mode:

    S0  input latch (chroma swapped in; the dry path is never swapped)
    S1  8-bit luma, [1 2 1] horizontal blur history, coarse column accumulate
    S2  pyramid lerp A (raw -> h-blurred)      one narrow signed product
    S3  pyramid lerp B (-> coarse)             the blended luma is what the
    S4  line-buffer read                       window stores, so ONE Sobel
    S5  land the two rows
    S6  Sobel column sums + histories
    S7  gx, gy
    S8  |gx|, |gy|, octagonal magnitude, signs
    S9  block-float normalise (case over fixed slices) + 16-bin orientation
    S10 material bands (compares against per-frame constants) + injection
    S11 polarity gate; RELIEF directional derivative (2 narrow products)
    S12 pipe
    S13 land the dry video off the ring; orientation -> hue
    S14 interior lerp; material chroma priority select; halo presence
    S15 material luma (additive, capped at 768) + one chroma lerp
    S16 EMERGE dry/wet + Invert (chroma swapped back here)
    S17 blanking gate -> output register

**Block-float is the load-bearing trick.** A per-frame shift puts the
threshold in 16..31, so "multiples of threshold" is just an 8-bit code and
every band (glow 0.55x, body 1x, core 3x) is a compare against a per-frame
constant. The entire edge path -- detection, thickness, anti-aliasing, the
EMERGE sweep -- carries **no per-pixel multiply at all**.

**Field engine** (p_field): one 8192x4 BRAM + one 128x8 aux word per column
carrying {above-row old value, running injection}. Per cell: two prefetch
reads, one recursion write, one aux read/write, all on one port at cell rate.
The four corner values are prefetched a full cell ahead -- reading them
mid-cell would stripe every cell boundary, since the values only settle two
clocks after their address is issued. The write lands on the last line of a
cell row, when that row's injection is complete.

### The multiply audit (6692 -> 4876 LUT4, -27%)

First synthesis came in at 87% LC with only three modes. Nothing structural
was wrong; every one of these was a careless operand width:

| Cost | Cause | Fix |
|---|---|---|
| ~500 LUT | polarity gate written as `a * pol` on 16-bit operands | a shift (>>3) -- the multiply bought nothing visible |
| ~500 LUT | per-frame LUT built as `i * coef` in 8 unrolled slots | build it by ACCUMULATION across sequencer states |
| ~400 LUT | `signed(resize(x,20)) * signed(resize(a,20))` chroma lerps | 12x6 on the delta, not 20x20 on both |
| ~280 LUT | pyramid blends as `a*(32-w) + b*w` | `a + ((b-a)*w >> 5)`, one product |
| ~250 LUT | per-frame threshold products inline | one shared multiplier, operands muxed by state |
| ~170 LUT | emboss on full-width gradient and sine | shift both operands down first |

Standing rule for the rest of the modes: **every product is written as
delta-times-weight at the narrowest width that carries the result**, and any
per-frame arithmetic goes through the shared multiplier or an accumulator.

Resources after the audit: **4876 LUT4, 1213 carry, 20/32 EBR** for the core,
the field and three modes. That leaves roughly 2400 LUT for BURIN (the stripe
engine, shared with KIRLIAN's filaments), KIRLIAN, CONTOUR, CEL and INK.


## 9. The fit wall (measured, 2026-09-03)

`livewire.vhd` v0.1 -- core + field + NEON/SPECTRAL/RELIEF -- routes at
**6586/7680 LC (85%), 20/32 EBR, 72.1 MHz** post-route on hd_analog. Getting
there took four timing iterations, all of them the same lesson at different
places: **one multiply per stage, and never a multiply plus an add plus a
clamp in the same stage.**

| Iteration | LC | Post-route | What moved |
|---|---|---|---|
| v0.1 first route | 99% | 48.6 | field recursion fused into one state (36 logic levels) |
| + recursion pipelined over 4 cell clocks, register-file LUTs dropped | 96% | 55.7 | |
| + material luma to fixed shifts, lerps narrowed, glide trimmed | 92% | 60.3 | |
| + compose split, constant half of each lerp folded per-frame | 87% | 69.8 | |
| + chroma product on its own stage | 88% | 70.0 | several paths now at one depth |
| + dry-delay chain deleted (alphas dissolve instead) | **85%** | **72.1** | |

### Where the logic actually is (LFSR-stubbed ablation)

- field engine: **591 LUT** + 9 EBR
- per-frame sequencer and controls: **781 LUT**
- everything else (input, pyramid, window, Sobel, orientation, bands,
  compose, output): ~2500 LUT

### The ratio that decides the program's shape

**LC is not LUT count: it is LUT + carry + flop, essentially additively.**
This design: 3907 LUT + 1213 carry + 1749 FF = 6869 -> 6790 LC placed.
Intaglio shows the same shape (1721 + 720 + 1316 for ~4230 LC). So a pipeline
register costs exactly what a LUT costs, which is why deleting the 180-flop
dry-video delay chain bought 200 LC and 2 MHz.

**Consequence: eight modes do not fit.** The five remaining modes need roughly
+1700 LUT, +400 carry and +600 flops -- about +2700 LC against ~1100 free.
The pre-agreed fallback (merge CONTOUR+CEL, SPECTRAL to overlay) still needs
about +1950 LC and does not fit either. Realistic savings left in the shared
engine (qsin ROM to EBR, field bilinear as a DDA, narrower sequencer
arithmetic) total ~800 LC -- worth doing, not close to enough.

The natural split is technical, not arbitrary: **the field is what divides
them.** NEON, KIRLIAN, SPECTRAL, RELIEF and INK BLEED all read the field
(halo, corona, bloom, ambient occlusion, bleed) and all want PERSIST. BURIN,
CONTOUR and CEL never touch it -- they are stroke- and level-based, and in a
program without the field they would have ~1500 LC spare for a proper stripe
engine and a real hatch ladder.


## 10. First-render verification (2026-09-03)

The program is five modes now -- NEON, KIRLIAN, SPECTRAL, RELIEF, INK BLEED --
after the fit decision in section 9. It rendered on the first sim, and then
four real bugs fell out of one test: **EMERGE 0 must be bit-exact video.**

1. **`64 - s_intw` where s_intw is 6 bits.** The literal takes the other
   operand's width, so `to_unsigned(64, 6)` = 0 and every interior lerp
   multiplied the video by zero. The picture still looked plausible at high
   EMERGE (the material dominates) and was uniformly wrong at low EMERGE.
   This is the first trap in CLAUDE.md's VHDL list; I walked into it anyway.
   The tell was a compose output that did not vary with the input at all.
2. **Dry chroma order.** With the separate dry tap deleted, passthrough now
   flows through the compose -- so it takes the output U/V swap and needs a
   matching input swap. The ring stores chroma in internal order now; before
   that, a warm brown bear passed through cold blue.
3. **The field halo is not covered by the alpha gate.** The field keeps
   decaying for several frames after EMERGE drops, so an ungated halo still
   tinted chroma at 0. One `s_matoff` gate now covers alphas and halo.
4. **The 768 luma cap was being applied to video.** Capping the composed sum
   crushed a white passthrough to 80% (255 -> 205). The cap belongs to DRAWN
   material: it may never push luma past 768, but it must not pull
   already-brighter video down. Limit = max(768, ground).

Verified with a ten-patch flat-colour card at EMERGE 0: **all ten patches
bit-exact.** Flat patches are the right instrument here -- a photograph can
not distinguish "identity" from "close" through the tester's own resampling,
and three earlier diffs against a photo were inconclusive for exactly that
reason.

5. **The glide accumulator was resized instead of sliced.** `unsigned(resize(
   signed_sum, 14))` keeps the sign bit plus the rightmost 13 bits, so every
   knob past half range folded: the EMERGE accumulator oscillated 5765 ->
   844 -> 6104 -> 1491 instead of converging to 11200, and every per-frame
   constant derived from it thrashed frame to frame. The picture merely
   looked "wrong-ish" -- the interior never faded, the threshold wandered --
   which is why it survived four rounds of staring at renders.

### Instrument choice was the whole lesson here

Three attempts to read internal state by encoding it into output luma and
chroma gave inconsistent answers (a value read as 453 one run and ~0 the
next), because an 8-bit RGB round-trip through the tester's decode carries
+-10 codes of noise -- more than enough to hide the difference between 50 and
60. A `report` statement inside `-- synthesis translate_off` printed the
exact values on the first try and the bug was obvious in one line:

    FIELD 0  s_p12=700  g_p12=5765  intw=0   tbody=24
    FIELD 1  s_p12=700  g_p12=844   intw=30  tbody=31

**For internal state, print it; don't paint it.** Analog readback is only
good for a value you can afford to be wrong about by 5%.

After the fix, the same probe reads `g_p12=11185 intw=63 gndw_y=68 tbody=17`
on every field -- stable, and exactly the designed values (the normalised
threshold landing in 16..31 is the block-float working).


## 11. Timing: the deficit is real, not seed noise

Seven seeds on the five-mode build: **64.7, 69.5, 70.1, 70.7, 70.9, 71.3,
71.7 MHz** -- none reach 74.25. A 7-seed spread that never crosses the line is
a real ~3.6% deficit, and per CLAUDE.md seeds cannot close one; only
restructuring can.

The binding path was not where I assumed. It starts at **`s_intw`** -- a
per-frame register -- and runs through the entire 10x7 interior-lerp multiply
array. The weight is the operand that gates every partial product, so the
whole array sits behind it, while the video operand's path is short. Two
fixes, no extra stage:

- hand the pixel path a **registered complement** (`s_intwc <= 64 - s_intw`)
  so the subtract is not in front of the array;
- **split the weight hi/lo** (`d*w(6:3)` and `d*w(2:0)`), recombining on the
  existing S15 add -- each stage now carries a 3- or 4-bit multiply instead of
  a 7-bit one.

Lesson worth carrying: when a multiply is the critical path, check WHICH
operand the path enters from. A per-frame constant feeding the multiplier
side of an array is still fully timed at the pixel clock, and splitting that
operand is what shortens the path -- pipelining the product does not.


## 12. Margin, not luck (2026-09-05)

Hardware report: the effect worked, then "after changing other parameters the
modes don't look the same any more -- like it is operating under timing."
That read was right, and the build log agreed. Two builds of nearly identical
logic (+31 LC):

| config | build 1 | build 2 |
|---|---|---|
| HD Analog | 77.01 | **69.54 (miss)** |
| HD HDMI | 76.14 | 74.83 |
| HD Dual | **74.39** | 74.99 |

A 7.5 MHz swing from a 31-LC change is not a design difference, it is a
placement lottery -- the design sat close enough to the edge that each seed
was a coin flip, and HD Dual "passed" build 1 by 0.14 MHz, which on real
silicon at temperature is not passing. **Setup violations are data-dependent**:
which paths actually fail depends on the values flowing through them, which
depend on knob positions. That is exactly why the modes degraded only after
other parameters were moved, and why it did not recover on its own.

Note also that build 2's HD Analog "completed (seed 6)" was a best-effort MISS
reported as a tick -- the last-retry case CLAUDE.md warns about. Read the
Fmax, never the tick.

**The fix was logic removal, not seed hunting.** Both lookup tables were being
built out of LUTs: the 16-entry orientation hue wheel (a 16:1 mux of 20 bits
sitting directly in the chroma path) and the 64-entry quarter-wave sine (in
the sequencer, which is timed at the pixel clock like everything else). Moved
both into BRAM -- the part had 12 spare EBR and zero spare LUTs.

yosys will not put a small memory in BRAM on its own: 64x10 and 16x20 are
below the threshold where it thinks a block is worth spending. **Padding each
table to a full 256-deep page makes the inference fire** (EBR 20 -> 23, LUT
4400 -> 4215). Both reads also gained their own pipeline stage in the process,
which is worth as much as the LUT saving.

Result: HD Analog 69.5 -> **75.1 MHz on seed 1** -- an unhunted seed, where
before it took a lucky one.


### Closing the margin properly (the ordered list that worked)

Profile, fix, measure -- four rounds on HD HDMI, the config that was at
+0.05%:

| change | LC | routed |
|---|---|---|
| starting point | 7086 | 74.29 |
| `f_ramp` compared a 16-bit shifted value; the shift amount is a compile-time constant per call site, so compare the INPUT against the constant saturation point instead. Ragged-edge hash subtract hoisted out of the alpha stage. | 7102 | 74.91 |
| pyramid blend delta rewritten as the Laplacian (the [1 2 1] blur already forms those taps, so the subtract is free), blend weights narrowed to 4 bits | 7129 | 74.02 (est. 80.67) |
| **collapsed the pyramid from two blends to one** -- the raw->blurred blend was the whole critical path and buys nothing, because the Sobel that consumes the level already smooths. The [1 2 1] level IS the fine end. | **6900** | **77.89** |

The third row is the instructive one: the placement estimate went to 80.67
while the routed number went DOWN. At 92% LC the design was congestion-bound,
so making logic shallower without making it smaller does nothing -- only
removing it helps. The last change removed a whole pipeline stage and 200 LC,
and bought 3.9 MHz.

Keep the SCALE mapping when collapsing the pyramid: the old knob's lower half
meant "fine" (pure [1 2 1]) and only its upper half blended toward coarse. Map
the whole knob onto the single remaining blend and every mode's native scale
bias silently means something coarser than it was tuned for -- NEON came back
visibly sparse until the two-segment mapping was restored.

## 13. Shipped build (2026-09-06) and the relabel trap

Build5 (24-seed hunt) closed all six: HD Analog 75.63 (s1), HD HDMI 74.79 (s2),
HD Dual 77.01 (s22), SD 68-73.  K1 was then relabelled from "Mode 0-100%" to
five `value_labels` (Neon / Kirlian / Spectral / Relief / Ink Bleed) -- the
platform divides the range evenly, matching the RTL's 205/410/614/819 zones.

Trap: `build_programs.sh` after a TOML-only edit did NOT reuse the cached
bitstreams -- it re-ran all six configs with the default 6 seeds and HD Dual
landed on a last-retry miss (66.93).  For label/preset edits, do not rerun the
builder: rebuild any config you need with `make ... CONFIG=<cfg> SEED=<known
good>` (toolchain env sourced) and repack with `vmprog_pack.py --no-sign`.

## 14. The seed lottery was one fused stage (2026-09-07)

Every saved critical-path report, on every seed and config, ended in the same
place: S4's scale lerp `mix3 <= clamp(mix2p + (d2c * s_w2) >> 4)` -- a 10x5
multiply, add and clamp fused in one stage, fanning out to the line buffer
(4.5 ns logic / 9.6 ns routing).  Fix, zero latency cost: S2 was a bare
register, so the subtract moved down to S2, the product got S3 to itself
(`d2cp`), S4 keeps add+clamp.  A `keep` copy of the per-frame weight (`s_w2p`)
sits in the pixel path so the placer can put it beside the multiplier.

HD HDMI seed sweep, same LC (6833):  before 65/69/70/71/72/75/79 (median 72);
after 84.5/84.2/82.6/82.8/83.5/84.1 (median 83.6).  Build7: all six configs on
seed 1 -- HD Analog 83.06, HD HDMI 84.51, HD Dual 86.87, SD 82-85.

Lesson: 270 LC of trimming and two BRAM experiments moved the median ~0; one
stage split moved it +11 MHz.  Read the critical path FIRST, believe it, and
notice when it is the same stage on every seed -- that is a real path, not
placement noise.  Relief was never on it; removing modes would not have helped.

## 15. Relief -> Cel (2026-09-08)

User: "Relief seems kind of subtle ... Let's try Cel instead."  Mode 3 is now
CEL: posterised flats + a clean black outline.  Relief's two 9x8 emboss
multipliers, the emboss pipeline and the K5 light-angle sine reads are gone.

- Posterizer (S13, own stage): luma keeps its top `s_pkeep` bits and
  replicates them down (0 -> black, full -> white at every depth), clamped to
  64..940; chroma rounds to a grid one bit finer, centred on 512.  keep = 5 is
  the identity, so every other mode -- and CEL at EMERGE 0 -- passes clean.
- `keep = 5 + K5[1:0] - EMERGE/128`, min 1, S8 Video floors it at 3 (8 tones).
  First eighth of EMERGE untouched (outlines emerge before flats), 2 tones at
  the top for every K5.
- Line: dark material, luma minus 4x body alpha (black on any flat, AA edge a
  pixel wide).  K6 tints the field shade that hugs the outline (chroma only).
  Ground = paper 900, lerp weight quartered for CEL (the flats ARE the interior
  replacement).
- Dry ring is now read one clock EARLIER (`ring_w - 9`, was -10) and landed raw
  at S12 so the posterizer gets S13 to itself.  Derivation: d_y(T) = sample(T-12)
  before and after -- one more stage means one LESS address offset.  (First
  draft went to -11: wrong direction.)
- TOML: K1 label Relief -> Cel, K5 "Light" -> "Levels", preset "Struck Metal"
  (K1=448, which had been Spectral since the 5-zone remap) -> "Cel Shade" at
  704; "Oil Slick" moved 704 -> 512 so it is actually Spectral.

Build8, all six on seed 1 except HD Dual (seed 3): HD Analog 81.10, HD HDMI
82.62, HD Dual 83.74, SD 76-83; 6520 LC (-310).  Not yet on hardware.

### 15.1 Cel shade off at K6 0% (2026-09-08)

User asked for the outer glow to switch off at Colour 0%.  In CEL the field
halo only stains chroma (dark material), so the halo GAIN `s_halog` now ramps
0 -> 127 over K6's first eighth (`g_k6(13 downto 11) = 0`), then the normal
120 + overdrive.  Hue still follows K6 everywhere.  Per-frame only, no pixel
cost.  Build9 6/6: HD 79.25 (s2) / 78.21 / 85.37, SD 81-84; 6520 LC.
