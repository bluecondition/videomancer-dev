# Outliner — summary

Catalogue combo #13 (seed 20260914): edge detect + split-screen + invert /
negative / bit-plane. v0.1.0 (2026-09-11) was flashed and liked ("outliner
is good", 2026-09-11). v0.2.0 is the first finalizing batch, built from a
review on 2026-10-12.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Threshold | edge threshold, curved (K1/8 + K1²/4096 = 0..382; fine steps at the low end) |
| K2 Kernel | Roberts 2×2 / Sobel 3×3 / Wide (8-px span, lines thickened 3 rows down) / DoG (5- vs 17-px boxes + vertical Laplacian) |
| K3 Seam | split position, a fraction of the measured width |
| K4 Panes | Split / 2 / 4 / 8 strips alternating edge side and other side |
| K5 Plane | Negative / Bit 8 … Bit 2 |
| K6 Mix | how far the invert side goes to its negative or plane (default 100%) |
| S7 Edges | edge side: white lines on black / lines over the picture |
| S8 Invert Side | the edge side / the other side is inverted |
| S9 Lines | lines on the edge side only / on both sides |
| S10 Invert | the whole area / only edge pixels (lines take the inverted picture) |
| S11 Line Colour | white lines / lines with the source chroma |
| P12 Sweep | seam swings on a sine about K3; swing up to ± the full width, ~1.7 s a cycle at 100% |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- All three kernels were horizontal-only (a difference along the scanline),
  so horizontal edges were never outlined. Two line buffers now give a 3-row
  window and every kernel is 2-D. Roberts is the real 2×2 cross (it was a
  1-px central difference). Sobel is new. Wide is the 8-px span plus a
  2-row vertical difference, with its flags ORed over 3 rows below.
  DoG adds a vertical Laplacian. All four give about 2H for a step of height H, so
  one threshold suits them all.
- DoG was off-centre and lit up flat highlights. The 5-px box was centred 6 px
  away from the 17-px box. The scales were also unbalanced (1.0625 × mean17 −
  1.25 × mean5): a flat field of level L gave −0.19 L, so bright areas filled
  white below ~16% threshold. Now the boxes are concentric and the term is
  (17·sum5 − 5·sum17)/64, which is 0 on any flat field.
- K2 labels were Roberts / Wide / DoG / DoG; now Roberts / Sobel / Wide / DoG.
- K1's top ~15% was dead: the linear 0–1023 threshold exceeded any gradient.
  Now the curve runs 0–382 (strong edges only at 100%).
- P12 swing was a fixed ±127 px (±6.6% of HD) at 8.6 s a cycle. It now
  scales with the measured width: ± the full width at 100%, clamped at the
  edges so the seam dwells there. Speed is 16 + P12/8 + P12²/2048 per frame
  (of 65536): ~1.7 s a cycle at 100%, ~5 s at 50%.
- Dead switch combinations:
  - S8 = Edge Side with S7 = On Black hid all inversion, because the black
    ground was painted over the inverted picture. Now the line drawing
    itself inverts. Negative gives black lines on white, and a bit plane keys
    where lines and ground swap.
  - S10 = Edges Only hid the inversion wherever white lines were drawn. Now
    those lines take the inverted picture.
  - K6 Mix defaulted to 0, which made K5, S8 and S10 inert at power-on. The
    default is now 100%. ~60% would sit near the negative crossfade's
    flat-grey middle.
- The video ran 20 clocks deep against a 12-clock sync delay. The picture sat
  9 px right, with 9 px of blanking data at the left edge, and the
  last 4 stages were a pure delay.

Hardware contracts:
- HD HDMI bar fix (damier v0.4.4): [1 2 1] on Y/U/V plus luma scaled to
  white ~776, before the gate. White-on-black edge lines in motion are the
  bar trigger. Side effect: 1-px Roberts lines read softer and greyer.
- Picture carried in a 256×32 EBR ring (stage 1 → 18) instead of ~20 FF
  stages of raw video (the rollthebones/cascade HD HDMI hazard).
- Input luma is black outside avid. There are no kernel edges in a 12-px
  border or in the first 3 rows of a field, so the frame edges and the
  previous field's bottom rows are never outlined. The field start is "avid
  absent for 4096 clocks", not the vsync edge.
- Latency 26 clocks, the same in all modes; video and syncs are aligned.
- `no_rw_check` on all four RAMs (line buffers, flags and ring write a
  different address from the one read).
- Per-frame products on one serial shift-add multiplier (vblank sequence).

## Build

`./build_combo.sh outliner` → `out/rev_b/ron/catalogue-combo/outliner.vmprog`.

v0.2.0 build (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 81.9, HD HDMI 89.7, HD Dual 81.2 MHz; SD Analog 86.7, SD HDMI 77.7,
SD Dual 81.9 MHz. ~4,290 LC (56%), 13/32 EBR (8 line buffers, 2 Wide
flags, 2 picture ring, 1 sine). Standalone yosys: 0 emulate_transparency.
v0.1.0 was 78.8–84.4 MHz HD, 3,400 LC (44%), 1 EBR.

HW status: v0.2.0 built, not flashed. Not simulated (standing build-first
rule), so a wrong picture is first a pipeline-alignment suspect.

Defaults and presets are hand-written. K1 310 matches v0.1's 120 on
step edges, and the presets are remapped (Blueprint: K1 80 → 240, K2 Wide
300 → 600).
