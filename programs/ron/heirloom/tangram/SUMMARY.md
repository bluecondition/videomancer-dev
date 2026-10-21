# Tangram — Heirloom #9

**Source photo**: `photos/WIN_20230222_00_06_47_Pro.jpg` (1920×1080 capture, 2023-02-22)

## Breakdown of the photo

- Three stripe fields share the frame: red/magenta diagonal stripes on dark blue (top left and right),
  blue vertical stripes (top centre), green/cyan diagonals the other way and a rainbow stripe run
  (red/yellow/green/cyan/blue) across the lower half.
- The fields are keyed into each other by a slow shape (zig-zag seams), and the seams are torn sideways
  in horizontal bands — a keyer fed by several ramp oscillators with a sheared key.

## Design

| Photo element | Program |
|---|---|
| Three stripe fields | three phase accumulators: field 1 = a1·x + b·y, field 2 = a2·x, field 3 = a2·x − b·y; square-wave two-colour stripes |
| Rainbow stripes | field 3 in Rainbow mode: a 256-entry rainbow ROM indexed by its phase |
| Keying between fields | the source luma × P12 gain → a 2-bit selector (field 1 / 2 / 3 / blend) |
| Torn seams | 48-virtual-line bands each add a random phase offset (and shift the source read) scaled by K4 |
| Colour scheme | six rainbow reads per field at hue offsets (K6) give the fields' colour pairs |

The source (video or oscillator) only supplies the key luma, so the line buffer prunes to 6 EBR.
Generated avid; output conditioning before the blanking gate. Latency 16 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Pitch 1 | stripe pitch of field 1 |
| K2 Pitch 2 | stripe pitch of fields 2 and 3 |
| K3 Angle | diagonal slope of fields 1 and 3 |
| K4 Shear | per-band sideways tear of the seams |
| K5 Rate | key drift through the tones |
| K6 Hue | colour scheme rotation |
| S7 Field 3 | Solid / Rainbow |
| S8 Shear | Right / Left |
| S9 Key | Hard / Soft (blend state) |
| S10 Move | stripes scroll |
| S11 Source | Video / Osc |
| **P12 Key** | key gain: field 1 only → all three finely interleaved |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 82.9–84.8 MHz, 3.9k LC (51 %), 6 EBR. **Not yet flashed.**
