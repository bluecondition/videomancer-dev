# Mandala — Heirloom #6

**Source photo**: `photos/WIN_20230128_01_06_08_Pro.jpg` (1920×1080 capture, 2023-01-28)

## Breakdown of the photo

- **Composition**: smooth, continuous rainbow contour rings — nested rounded lobes in magenta, pink, blue,
  green, red and yellow — growing out of several centres and meeting in soft seams; the whole picture mirrors
  left to right. Red / green flares at the far edges, black at the bottom centre.
- Mechanism: several soft radial fields (lens-like oscillator products) combined and run through a rainbow
  colorizer at a high ring rate.

## Design

| Photo element | Program |
|---|---|
| Several centres | three centres orbiting the screen centre (triangle-wave orbit, K3 speed), spread by P12 |
| Lobes meeting in seams | field = the NEAREST centre's r² (min of three) — every centre owns a lobe of rings |
| Left / right mirror | S7: centre 2 is the mirror of centre 1 |
| Rainbow rings | cyclic 256-entry palette (Rainbow, or Magenta / white / blue / pink), index = field × K1 |
| Ring shape | K4 Aspect scales dy² (8 zones, 1/2 → 4×) |
| Third lobe size | K6 Weight scales centre 3's field |
| The picture | luma × K5 added to the ring index (etch); Source Osc swaps in the oscillator |

r² per centre: dx² by second differences per pixel, dy² per line on the serial multiplier (3 jobs per line),
the orbit offsets and (2cx)² per frame (9 jobs in the vblank zone). Generated avid; output conditioning before the
blanking gate. Latency 20 clocks, all modes. Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Rings | ring density |
| K2 Hue | palette rotation |
| K3 Orbit | centre orbit speed, bipolar |
| K4 Aspect | vertical stretch of the rings |
| K5 Etch | video depth into the field |
| K6 Weight | the third centre's lobe shrinks |
| S7 Mirror | centre 2 mirrors centre 1 / orbits on its own |
| S8 Centres | Three / Two |
| S9 Palette | Rainbow / Magenta |
| S10 Rings | Smooth / Hard (8 bands) |
| S11 Source | Video / Osc |
| **P12 Centres** | spread: all centres together (one onion) → far apart (lobes) |

Resolution: all geometry in virtual 1080-line units (round at every mode).

## Status

- v0.1.0 (2026-10-20): first build missed timing on every HD Analog seed (58 MHz, router stalls); the critical
  path was the serial multiplier's job load — a 12-way operand mux fed straight from the triangle-wave functions.
  Operands and the job-start pulse are now registered. Second build: 6/6 (HD Dual seed 4, others seed 1), HD 75.0–78.9 MHz, 5.9k LC (77 %), 9 EBR. **Not yet flashed.** HD Dual at 75.0 is a thin margin.
