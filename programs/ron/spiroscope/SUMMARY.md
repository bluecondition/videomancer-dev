# Spiroscope

A knob-driven phosphor drawing instrument — an advanced etch-a-sketch crossed
with a spirograph. A free-running "pen" plots into a 160×90 × 4-bit phosphor
**canvas** held in BRAM; the canvas fills the **whole screen** — each cell =
8×8 screen px, so 160×90 maps to exactly 1280×720 (`>>3` upscale, shift-only).
Spirograph figures are scaled to fill the full height (round, centred). Two pen
engines share the canvas, and a per-frame decay sweep gives oscilloscope-style
persistence.

## Drawing engines (S7)

- **Spirograph** — `pen = centre + R1·trig(A) + R2·trig(B+φ)`. Two 16-bit phase
  accumulators advance per substep; their high 10 bits index a shared
  `sin_cos_full_lut_10x10`. One shared 8×10 multiplier sequences the four
  R·trig products via an 8-state FSM. Phases free-run; a per-frame budget (K6)
  sets how much of the locus is drawn each frame ("draw speed"), so with low
  decay you watch the figure draw itself.
- **Etch-a-sketch** — K1/K2 set pen X/Y velocity (classic), integrated once per
  scanline (≤0.25 cell/line → always a continuous line). With persistence at 0
  the lines are permanent.

## Persistence / decay

A vblank sweep fades every cell. The slider (hero control) sets the rate:
`0` = permanent (true etch-a-sketch); low = long phosphor trails (fade by 1
every 2^p frames); high = fast scope decay; full = one-frame erase
("shake to clear"). Decay uses a saturating subtract-K in a wider temp with an
explicit `K=0` bypass — never wraps a 4-bit cell brighter.

## Colour

K5 picks one of 8 palette colours (Red…Violet, White), stored U/V-swapped for
the Videomancer hardware convention. Brightness (the 4-bit cell value) drives
luma so trails glow and fade; the palette sets chroma. **Rainbow** (S10) instead
derives the hue from `(cellx + celly + frame)` — drifting diagonal spectral
bands. **Kaleidoscope** (S9) mirrors every pen write into 4 quadrants
(`127-v = not v`), done write-side so the per-pixel readout path stays pure.
**Background** (S11) composites the drawing over the live input video.

## Control map

| Ctrl | Spirograph | Etch-a-sketch |
|------|------------|---------------|
| K1 | Freq A (inner ratio) | Pen X velocity |
| K2 | Freq B (outer ratio) | Pen Y velocity |
| K3 | Arm ratio R2:R1 | — |
| K4 | Phase / twist φ | — |
| K5 | Colour (8-palette) | Colour |
| K6 | Draw speed (budget) | — |
| Slider | Persistence / erase | Persistence / erase |
| S7 | Mode | |
| S8 | Harmonic — adds a 3rd epicycle (freq A+B) | |
| S9 | Kaleidoscope (4-fold) | |
| S10 | Rainbow | |
| S11 | Background Black/Video | |

## BRAM port plan

Single 1W1R EBR canvas (15 of 32 BRAMs, 14400×4; addr = `cy*160 + cx` formed in
the arbiter stage so the shift-add stays off the long readout chain). Active
video: read port → upscaled readout, write port → pen plot. Vblank: a decay
sweep owns both ports (read k, write k−1, fade), 14400 cells ≪ ~49k vblank
clocks. A registered arbiter mux selects {readout | pen | decay}.

## Status

Built; **all 6 configs close timing post-route** at the HD pixel clock
(74.25 MHz): HD Analog 83.0, HD HDMI 76.7, HD Dual 80.8 MHz; SD configs clear
27 MHz easily. ~3270 LCs, 15/32 BRAM. **Not yet hardware-validated.**

Timing notes: (1) `build_programs.sh` prints nextpnr's pessimistic *pre-route*
estimate (~64–73 MHz, often FAIL); the real routed Fmax is the **second** "Max
frequency" line in the nextpnr log and passes. (2) The full-screen rebuild was
timing-marginal until the sin/cos LUT was fed only its **top 8 angle bits** (low
2 tied to `"00"`) — that prunes a quarter of the 1024-entry ROM mux (the
critical path) and lifted HD HDMI 74.1→76.7 MHz.

Forked conceptually from the canvas/decay ideas in echotube/howler and the
1W1R BRAM pattern in c64; sin/cos and phase-accumulator primitives from the SDK.
