# RUTT ETRA — classic scan-processor Z-displacement

Incoming video re-rendered as horizontal scanlines vertically displaced by
luma — the classic Rutt/Etra look: bright material rises out of a sagging
dark floor, glowing lines on black, with hidden-line terrain occlusion
(each nearer line drops an opaque skirt hiding the lines behind it).

## Engine — rotating-bank displaced-scanline gather

Displacement is one-sided (down from the source row), so every line that can
cross the current pixel comes from a row **above** it. Every Nth active row
("sampled row") converts its luma — after contrast gain and a horizontal
smoothing IIR — into `{disp[7:0], bright[3:0]}` per column (columns
decimated ×2) and writes it into one of **8 rotating BRAM banks**
(1024×12 = 3× 1024×4 EBR each, 24/32 EBR total). Per display pixel, 8
parallel candidate tests (`age_k − disp_k` vs the stroke window) find hits;
a recency-priority resolve (rotate hit vector by head, priority-encode)
picks the nearest source row — which is exactly the hidden-line rule.
Skirt = any candidate with `age ≥ disp` covering the pixel.

- Max displacement = 8 × spacing − 1 (capped 255): denser line fields have
  proportionally shallower relief — inherent to the 8-bank window.
- Ages are per-bank saturating counters advanced at row start (active lines
  only); banks invalidated at vsync (idempotent under serration). The bank
  being written is masked (`age = 0`) — iCE40 read-during-write is undefined.
- Per-bank registered read address (`attribute keep`) so the placer sits
  each copy beside its own EBR group instead of routing one high-fanout net.
- P12 Relief scale is glided (ease diff/4, ±1 deadband) so pot noise can't
  make the whole line field breathe.
- **Connected strokes (v0.2)**: each display pixel draws the segment from
  the previous half-column's height to its own (midpoint subdivision of the
  x2 columns: phase 0 = prev->mid, phase 1 = mid->this), tested as
  `age-lo >= 0 and age-hi < T` — steep relief stays one continuous line.
  Segments are dimmed by their vertical extent (span > T: 1/2, > 4T: 1/4) —
  a fast beam sweep deposits less light.
- **Relief normalised to the window (v0.2)**: effective scale =
  scale × min(spacing,32)/32, so dense line fields (dmax down to 63) sweep
  the same relative range as sparse ones instead of clipping flat by 10%.
- **Smoothing lag cancelled on READ (v0.2)**: the one-pole IIR delays
  features by 2^s−1 px; the display reads the (already complete) stored
  rows 2^s px ahead, clamped at the measured last column — no latency cost.
- **Roll (v0.2)**: line-grid phase in units of one spacing (16-bit, wraps
  free); first sampled row = phase × spacing at vsync; the field's top line
  has its brightness × phase so new lines fade in instead of popping.
- 15-clk latency, identical across modes; blanking-gated; drawn luma capped
  at 768; generated palette stored U/V-swapped (Video colour passes source
  chroma dry, no swap).

## Controls

P12 **Relief** (headline, glided). K1 Scan Lines (density, CW = denser),
K2 Thickness 1–8, K3 Line Color (White/Green/Amber/Cyan/Magenta/**Video**
= source chroma ×2 at the drawn pixel/Fire/**Terrain** = altitude tint from
the line's luma band — was disp[7:5], which collapsed to 1–2 colours at
dense lines or low relief), K4 Contrast (unity at center), K5 **Roll**
(bipolar, centre still, quadratic, ≤0.23 spacing/field; replaced v0.1's
Backdrop ghost), K6 Smoothing (IIR shift 0–7). S7 Peaks Bright/Dark, S8 Hidden Line, S9 Shading
Banded/Luma, S10 Hard/Glow (halo band ±T), S11 Positive/Negative.

## Status

**v0.2.0 (2026-09-28): FINAL — signed off by Ron ("looks good as a final version").** Connected strokes +
beam-speed dimming, relief normalised to the window, smoothing-lag
compensation, luma-band Terrain, Video line colour (replaced Ice), K5 Roll
(replaced Backdrop). 6/6: HD Analog 80.9 (seed 4; seeds 1-3 stall/miss),
HD HDMI 78.4 (seed 2), HD Dual 75.1 (seed 1), SD 72-77; ~5730 LC (75%),
24/32 EBR. v0.1.1 source + vmprog kept in `.ruttetra_work/`.

**HW-validated** (2026-09-04: "looks great" on first flash). v0.1.1 adds a
thin-stroke shading lift (+3/+2/+1 bf1 steps at thickness 1/2/3, clamped at
the 768 luma cap) — thin lines integrated less light and read dim.
v0.1.0: 6/6 timing-closed (HD 77.2 / 78.2 / 77.6 MHz, seeds 1–2),
~4070 LC (53%), 24/32 EBR.
