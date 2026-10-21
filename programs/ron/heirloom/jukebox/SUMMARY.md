# Jukebox — Heirloom #16

**Source photo**: `photos/WIN_20230315_00_28_42_Pro.jpg` (1920×1080 capture, 2023-03-15)

## Breakdown of the photo

- **Composition**: ten tall vertical panels (~192 px wide) across the full height, each a copy of the same
  picture: a rainbow gradient of rounded-square / oval rings centred mid-panel.
- **Rings**: continuous (not posterized) rainbow bands — red, yellow, magenta, blue, cyan — nested; the ring
  shape is a square with rounded corners stretched vertically along the panel.
- **Symmetry**: the pattern mirrors about the horizontal centre line (two rows of ring centres, top and bottom).
- A classic multiplied-oscillator picture: a fast horizontal ramp divided into panels, a slow vertical ramp,
  both folded and summed through a rainbow colorizer.

## Design

| Photo element | Program |
|---|---|
| Panels | N panels across (K1, 1–16) from one serial divide per frame; cell x counter in Q.1 virtual units |
| Rounded-square rings | rs = max(\|dx\|, \|dy\|/2) + min(\|dx\|, \|dy\|/2)·K2/256 — a blend from square (K2 = 0) to diamond |
| Continuous rainbow | 4 cyclic 256-entry palette ROMs (Rainbow / Warm / Cool / Neon, 8 EBR), index = rs × zoom |
| Top/bottom mirror | \|dy\| about a movable line within the row (K5); S7 off repeats the ramp from the row top instead |
| Two rows | S9: row height 1080 or 540 virtual lines |
| Ring density | P12 ZOOM (zk 2–257) |
| The picture | its luma × K6 is added to the ring index (etch) — the picture appears as ring displacement; Source Osc swaps in the oscillator |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Panels | panels across, 1–16 |
| K2 Round | ring shape: square → diamond |
| K3 Hue | palette rotation speed, bipolar |
| K4 Palette | Rainbow / Warm / Cool / Neon |
| K5 Centre | mirror line within the row |
| K6 Etch | video depth into the rings |
| S7 Mirror | rings mirror about the centre line / repeat from the top |
| S8 Rings | Smooth / Hard (8 bands) |
| S9 Rows | One / Two |
| S10 Invert | palette hues complemented |
| S11 Source | Video / Osc |
| **P12 Zoom** | ring density: one huge ring → dozens of fine ones |

Resolution: panel count is a COUNT (panels narrow on 4:3 SD); ring geometry in virtual 1080-line units.

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 78.7–80.1 MHz, 3.7k LC (49 %), 13 EBR. **Not yet flashed.**
