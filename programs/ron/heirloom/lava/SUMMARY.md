# Lava — Heirloom #43

**Source photo**: `photos/WIN_20230605_01_14_56_Pro.jpg` (1920×1080 capture)

## Breakdown of the photo

- **Composition**: red / orange flame lobes with white cores drip down the screen, narrowing as they go; blue-
  violet pods in rows along the bottom; a magenta / violet vertical-stripe curtain at the left; horizontal smear
  everywhere.
- **Palette**: red, orange, yellow-white, olive ground, blue, violet, magenta.

## Design

| Photo element | Program |
|---|---|
| Flame lobes | max of two column triangle waves (K1 pitch, 3:2 ratio) minus a threshold that rises below the P12 reach line → lobe strength → lava palette (K4 glow adds), plus the smeared source luma |
| Pods | diamond-metric cells (K2) along the bottom band (S8) → violet/blue palette |
| Curtain | vertical stripes magenta/violet where x < K3 (S7) |
| Ground | olive/green palette indexed by the smeared source luma |
| Smear | one-pole IIR on the source luma (K5) |
| Mirror | S10: lobe waves folded about the centre |

Shared plumbing: generated avid, vblank-zone field latch, virtual 1080-line geometry, Source Video/Osc switch
(oscillator muxed at the line-buffer write), output conditioning, blanking gate. Latency 20 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Lobes | lobe pitch |
| K2 Pods | pod cell size and band height |
| K3 Curtain | curtain width |
| K4 Glow | lobe core brightness |
| K5 Smear | source smear |
| K6 Drift | lobes drift sideways |
| S7 Curtain | on/off |
| S8 Pods | on/off |
| S9 Palette | Lava / Ice |
| S10 Mirror | on/off |
| S11 Source | Video / Osc |
| **P12 Melt** | how far down the lobes drip |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD HDMI seed 2, rest seed 1), HD 85.0–89.8 MHz, 3.7k LC (48 %), 20 EBR. **Not yet flashed.**
