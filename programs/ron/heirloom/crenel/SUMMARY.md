# Crenel — Heirloom #10

**Source photo**: `photos/WIN_20230223_01_46_41_Pro.jpg` (1920×1080 capture)

## Breakdown of the photo

- **Composition**: a dark navy field. Across the top a row of hot-pink crenellation combs — a square wave of
  teeth standing on a bar, eight across. Below, glossy blue forms with highlight and shadow edges (the video
  differentiated: every edge becomes a rim of light or dark) torn sideways in horizontal bands. Pink V's hang
  under the combs; a green/pink fringe on some edges.
- **Palette**: navy, saturated blue chrome, hot pink.

## Design

| Photo element | Program |
|---|---|
| Navy field | luma below K6 → navy |
| Blue chrome relief | horizontal derivative of the buffered luma over 3 px × P12 gain → luma about 320, chroma from K5 (blue → violet) |
| Pink crenellation combs | per-row-band square-wave teeth (K1 pitch) on a bar (K2 height); K3 sets the row pitch; S7 adds a mirrored comb at the bottom of each row |
| Pink V's | triangle under each comb (S9) |
| Row tears | per-8-line random read offset × K4; S10 still/moving tears |

Shared plumbing: generated avid, vblank-zone field latch, virtual 1080-line geometry, Source Video/Osc switch
(oscillator muxed at the line-buffer write), output conditioning, blanking gate. Latency 20 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Pitch | comb tooth pitch |
| K2 Height | comb height |
| K3 Rows | row pitch |
| K4 Tear | sideways tearing in 8-line bands |
| K5 Hue | relief hue, blue → violet |
| K6 Threshold | video luma below which the field stays navy |
| S7 Combs | Top / Both |
| S8 Relief | Left / Right (sign) |
| S9 Vees | pink V's on/off |
| S10 Tears | Still / Moving |
| S11 Source | Video / Osc |
| **P12 Emboss** | relief depth: flat → chrome |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 81.8–89.9 MHz, 3.7k LC (48 %), 5 EBR. **Not yet flashed.**
