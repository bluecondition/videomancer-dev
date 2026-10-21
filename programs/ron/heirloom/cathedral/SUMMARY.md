# Cathedral — Heirloom #41

**Source photo**: `photos/WIN_20230602_01_30_58_Pro.jpg` (1920×1080 capture, 2023-06-02)

## Breakdown of the photo

- **Composition**: mirrored left and right, green / teal gothic arches (nested pointed arcs) standing on a
  horizontal rainbow-banded ground (green, yellow, orange, pink, magenta), a pink diamond in the centre.
- **Arches**: thin teal ribs with a filled pointed cap, three across.
- **Palette**: teal / emerald arches, a warm-to-magenta rainbow ground, a pale pink diamond.

## Design

| Photo element | Program |
|---|---|
| Arches | bands of x_i² − ky·(y0 − y) within a width (three parabola accumulators by second differences; one per-line product (y0 − y)·ky); the cap fills where the band is wide near the apex |
| Arch pair | centres cx, cx ± vw/4 (S7 Mirror) or cx, cx + vw/8, cx + vw/4 |
| Rainbow ground | 64-entry continuous rainbow ROM indexed by line·K6 + K3 + drift; S9 Source adds the source luma to the index (the picture through the rainbow) |
| Diamond | \|dx\| + \|dy\| ≤ K4, pink (S8) |
| Arch height | P12 ARCH: ky = 64 + 5·p12 (small = tall spires) |

Source Video / Osc on S11. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Width | arch band width |
| K2 Arches | 1–3 |
| K3 Hue | rainbow offset |
| K4 Diamond | diamond size |
| K5 Drift | rainbow scroll, bipolar |
| K6 Bands | rainbow pitch |
| S7 Mirror | pair mirrored / all to one side |
| S8 Diamond | on / off |
| S9 Ground | Rainbow / Source |
| S10 Arches | Teal / White |
| S11 Source | Video / Osc |
| **P12 Arch** | squat arcs → soaring spires |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Analog, HD Dual seed 2; others seed 1), HD 74.8–76.8 MHz (HD HDMI 74.76 on seed 2), 4.43k LC (58 %), 3 EBR — margins are thin (the 24-bit parabola accumulators), re-verify any config that only closed on a late seed. **Not yet flashed.**
