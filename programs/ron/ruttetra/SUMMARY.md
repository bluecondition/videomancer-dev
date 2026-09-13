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
- 12-clk latency, identical across modes; blanking-gated; drawn luma capped
  at 768; generated palette stored U/V-swapped.

## Controls

P12 **Relief** (headline, glided). K1 Scan Lines (density, CW = denser),
K2 Thickness 1–8, K3 Line Color (White/Green/Amber/Cyan/Magenta/Ice/Fire/
**Terrain** = altitude tint from disp[7:5]), K4 Contrast (unity at center),
K5 Backdrop (dim grayscale source ghost behind the lines), K6 Smoothing
(IIR shift 0–7). S7 Peaks Bright/Dark, S8 Hidden Line, S9 Shading
Banded/Luma, S10 Hard/Glow (halo band ±T), S11 Positive/Negative.

## Status

**HW-validated** (2026-09-04: "looks great" on first flash). v0.1.1 adds a
thin-stroke shading lift (+3/+2/+1 bf1 steps at thickness 1/2/3, clamped at
the 768 luma cap) — thin lines integrated less light and read dim.
v0.1.0: 6/6 timing-closed (HD 77.2 / 78.2 / 77.6 MHz, seeds 1–2),
~4070 LC (53%), 24/32 EBR.
