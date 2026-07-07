# Spiroscope

A knob-driven phosphor drawing instrument — an advanced etch-a-sketch crossed
with a spirograph. A free-running "pen" plots into a **320×180 × 2-bit**
phosphor **canvas** held in BRAM (v3; was 160×90 × 4-bit — traded phosphor
depth for 2× resolution per user preference); the canvas fills the **whole
screen** — each cell = 4×4 screen px, so 320×180 maps to exactly 1280×720
(`>>2` upscale, shift-only). Spirograph figures are scaled to fill the full
height (round, centred). Two pen engines share the canvas, and a decay sweep
gives oscilloscope-style persistence (3 brightness levels + off; the dithered
fade masks the coarser steps).

## Drawing engines (S7)

- **Spirograph** — `pen = centre + R1·trig(A) + R2·trig(B+φ) [+ R3·trig(C)]`.
  **v4 gears: inc = (zone+1)·128 + 3** — K1/K2 are 16-zone "Gear A/B" knobs.
  The ×128 quantization locks ratios near small integers m:n, so the figure is
  a *recognizable* N-lobed rosette; the +3 detune makes it slowly precess.
  (Design history: v1 pure-integer ratios = static burn-in; v3 fully-fine
  ratios = never locks, reads as scribble/mush. Near-integer + drift is the
  real spirograph behaviour.) 18-bit phases keep per-substep chords ≤ ~2.8° so
  walked segments never look polygonal. One shared 8×10 multiplier sequences
  all six R·trig products (operand@k → product@k+2 schedule). K6 sets
  substeps/frame (8..519): slow watch-it-draw up to ~4 revolutions/frame.
- **Etch-a-sketch** — K1/K2 form a velocity vector, **K4 rotates it, K3 scales
  it** (computed once per frame on the otherwise-idle LUT + shared multiplier),
  integrated per scanline into a **Q10.10** accumulator. All six knobs are
  live in both modes.

## Pen writer — line-walker

Every plot request walks 8-connected from the previous pen cell to the new one
(≤12 cells; farther jumps teleport), emitting 1 or 4 kaleidoscope-mirrored
writes per cell — strokes are **solid**, never beaded, in both engines. The
plotter stalls in its final state until the walker is idle. The teleport
decision is pipelined (latch target → decide next cycle): doing the subtract →
compare → mux cone in one clock was the post-route critical path (73.5 MHz).

## Persistence / decay

A decay sweep fades cells. The slider (hero control):
`0` = permanent (no sweep); **1–49% = dithered fade** — each cell fades by 1
only when `hash(addr)+frame < threshold` (P = thr/1024), so the image melts
smoothly (full fade ~51 s → ~0.1 s) instead of v1's synchronized
every-2^n-frames global step-down pulse; 50%+ = subtract-K every frame (K 1..2);
>98% = K=3 one-frame erase ("shake to clear"). Saturating subtract — never
wraps a cell brighter.

**Sweep scheduling (v3):** 57,600 cells no longer fit in vblank (~49.5k clocks),
so sweep *reads* run whenever `avid` has been low ≥2 clocks (vblank + every
hblank; the 2-clock guard protects the last pixel's trailing canvas fetch), and
each write-back lands one clock later on the write port (pen commits are gated
off while the sweep is active, so the write port is free). Writes are gated on
a "last clock was a sweep read" flag so a paused sweep can't write back a
display-fetched value.

## Display: bilinear upscale (v4)

Each output pixel bilinearly blends the 2×2 neighbouring cells, so strokes
render as smooth glowing vector lines instead of 4×4 blocks. The read port's
idle slots do the work: during span S the arbiter prefetches cells (S+2, row)
and (S+2, row+1) on sub-pixel phases 0/1; a tag pipeline captures them
(TRc/BRc) and commits to the blend registers exactly at the span boundary
(TLb←TRb rotate). Cell columns 0–1 are blanked (8 px, no valid prefetch at
line start). Blend = 2 tiny lerps (2b×3b), t ∈ 0..48, luma = t·21. The
captures double as the "register the 29-EBR read mux" timing fix.

## Colour

K5 picks one of 8 solid colours (rainbow hues + White), stored U/V-swapped.
Blended brightness drives luma; the palette sets chroma. **Rainbow** (S10, v4)
is an angular **pinwheel**: hue = octant of (cell − centre) × 2 + radius ring
+ slow frame rotation, indexing a **16-entry hue wheel** ((Cb,Cr) = 512 +
220·(cos,sin)(k·22.5°)). **Kaleidoscope** (S9) mirrors every pen write into 4
quadrants, write-side. **Background** (S11) composites over live input video
(soft stroke fringes with t<8 key over black only, avoiding halos).

## Control map

| Ctrl | Spirograph | Etch-a-sketch |
|------|------------|---------------|
| K1 | Gear A (1..16, near-integer + drift) | Pen X velocity |
| K2 | Gear B (1..16) | Pen Y velocity |
| K3 | Arm ratio R2:R1 | Speed scale |
| K4 | Phase / twist φ | Rotate drawing direction |
| K5 | Colour (8-palette) | Colour |
| K6 | Draw speed (substeps/frame) | — |
| Slider | Persistence / erase | Persistence / erase |
| S7 | Mode | |
| S8 | Harmonic — adds a 3rd epicycle (freq A+B) | |
| S9 | Kaleidoscope (4-fold) | |
| S10 | Rainbow | |
| S11 | Background Black/Video | |

## BRAM port plan

Single 1W1R EBR canvas (**29 of 32 BRAMs**, 57600×2; addr = `cy*320 + cx` =
two shifts + add, formed in the arbiter stage so it stays off the long readout
chain). Active pixels: read port → upscaled readout, write port → pen plot.
Blanking: the decay sweep issues reads (write-back one clock later). A
registered arbiter mux selects {readout | sweep | pen}.

## Status

v3 (2× resolution). Built; **all 6 configs close timing post-route** at the HD
pixel clock (74.25 MHz): HD Analog 81.7, HD HDMI 84.1, HD Dual 82.8 MHz
(verified at the packaged seeds); SD configs clear 27 MHz easily. ~3850 LCs,
29/32 BRAM. **Not yet hardware-validated.**

Timing notes:
1. `build_programs.sh` prints nextpnr's pessimistic *pre-route* estimate; the
   real routed Fmax is the **second** "Max frequency" line in the nextpnr log.
2. The sin/cos LUT is fed only its **top 8 angle bits** (low 2 tied `"00"`) —
   prunes a quarter of the 1024-entry ROM mux (was the critical path at 160×90).
3. At 29 EBRs the critical path became the **canvas read-data output mux**
   (EBR → 29:1 mux tree → colour logic): builds were a seed lottery around
   74 MHz until a register was added between `s_cv_rdata` and `p_color`
   (C_LATENCY 5→6). That one register moved all HD configs to 76–85 MHz at
   seed 1 — register a wide BRAM cascade's read data before consuming it.
4. The walker's teleport decision is pipelined (latch target → decide next
   cycle); doing subtract → compare → mux in one clock was an earlier
   critical path.

Forked conceptually from the canvas/decay ideas in echotube/howler and the
1W1R BRAM pattern in c64; sin/cos and phase-accumulator primitives from the SDK.
