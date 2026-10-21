# Archway — Heirloom #36

**Source photo**: `photos/WIN_20230523_01_26_43_Pro.jpg` (1920×1080 capture, 2023-05-23)

## Breakdown of the photo

- Black ground; a tall arch of thin cyan / blue lines — nested parabolas rising from the bottom centre to a point at the top.
- Along the outer flanks, stepped blocks of pink / yellow / navy following the same curve.
- A pink chevron (Λ) sits inside at the bottom centre.

## Design

| Photo element | Program |
|---|---|
| Parabola field | F = 16·y + a·dx² (dx² by second differences, a from P12) + drift phase |
| Rays | F mod 2^K1 < width (K2) → cyan / blue by parity (colour pair K4), inside F < reach (K6) |
| Between rays | the source darkened to a quarter |
| Flank blocks | bands of F (every 128 lines) stepped by column block (K3) → pink / yellow / navy, luma averaged with the source (or the picture itself, S8) |
| Chevron | rows below 780 where \|dx\| < (y − 780) → pink |

Generated avid; output conditioning; latency 20; 16 EBR line buffer.

## Controls

| Control | Function |
|---|---|
| K1 Rays | pitch 4 … 512 lines |
| K2 Width | ray line width |
| K3 Blocks | flank block width |
| K4 Hue | ray colour pair |
| K5 Drift | rays flow, bipolar |
| K6 Reach | how far the rays extend |
| S7 Rays | On / Off |
| S8 Flanks | Blocks / Picture |
| S9 Chevron | On / Off |
| S10 Mirror | picture mirrored |
| S11 Source | Video / Osc |
| **P12 Arch** | curvature: a gentle arch → a needle |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 81.6–86.4 MHz, 4.0k LC (52 %), 13 EBR. **Not yet flashed.**
