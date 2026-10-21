# Plume — Heirloom #19

**Source photo**: `photos/WIN_20230315_00_58_54_Pro.jpg` (1920×1080 capture)

## Breakdown of the photo

- **Composition**: mirrored left/right. A central plume of chevron ripples in red / magenta / yellow flares out
  of the centre column and fades toward the sides; above and below, green and blue bands flow on near-black.
- **Texture**: rows of chevrons (tri of x plus tri of y), the amplitude shaped by a bright centre envelope.
- **Palette**: red, magenta, yellow, pink-white cores; green and blue flow; black.

## Design

| Photo element | Program |
|---|---|
| Chevron ripples | t = tri(x·K1 + tri(y·K2) + drift) |
| Plume envelope | (W − \|dx\|) >> K4, W from P12; amplitude = t × min(envelope, source luma) |
| Rainbow colouring | 256-entry palette ROM (Fire / Ice, S9) indexed by amplitude + K3 hue + row flow, darkened by K6 |
| Mirror | S7: pattern folded about the centre, video read mirrored per starburst |
| Dark bands | S10: luma halved where the row triangle is low |

Shared plumbing: generated avid, vblank-zone field latch, virtual 1080-line geometry, Source Video/Osc switch
(oscillator muxed at the line-buffer write), output conditioning, blanking gate. Latency 20 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Ripple | chevron pitch across |
| K2 Rows | chevron row pitch |
| K3 Hue | palette offset |
| K4 Edge | plume edge softness |
| K5 Drift | ripple scroll, bipolar |
| K6 Dark | darkens the outer flow |
| S7 Mirror | on/off |
| S8 Weave | Chevron / Diagonal |
| S9 Palette | Fire / Ice |
| S10 Bands | dark bands on/off |
| S11 Source | Video / Osc |
| **P12 Flare** | plume width: a thin column → the whole screen |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 82.5–97.1 MHz, 3.1k LC (41 %), 8 EBR. **Not yet flashed.**
