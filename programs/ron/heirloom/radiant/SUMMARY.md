# Radiant — Heirloom #52

**Source**: `photos/duci.jpg` (a paper collage, 3196×4176 scan — not synth output)

## Breakdown of the collage

- A maroon, scratched and layered ground (torn paper, scuffs).
- Pink / white / black stripe rays fan out from a point behind the centre, like a sunburst or the spokes of a
  halo; they are paper strips with black edges.
- A dark circular halo (a ring of dark paper) frames the centre, where a man's head has a cup of coffee cut in
  for a hat and the word DUCI pasted over hands holding soil; two figures stand at the sides.

## Design

| Collage element | Program |
|---|---|
| Sunburst rays | N rays (K1, 2–8) as straight lines through the centre; each ray is the double cone between the boundary lines a−w and a+w, even rays pink, odd rays white. Boundary x positions are per-line DDAs (`x = cx − tan(a)·(y − cy)`, 16 Q.4 accumulators; increments and bases from a 256-entry tan ROM on the serial multiplier in vblank), so no per-pixel angle maths |
| Ray width | P12 Rays: w from a hairline to the rays merging into a full pink/white disc |
| Dark halo ring | r² (dx² by second differences, dy² per line) between (R−W)² and (R+W)²; K2 radius, K3 width; S8 makes it a glowing white ring |
| The cut-out at the centre | the video (or oscillator) inside the halo; S9 Cutout puts it outside instead, with the rays inside |
| Maroon textured ground | a slow triangle-wave texture on one of 8 ground hues (K4) |
| Spokes turning | K5 Spin rotates the ray angles per field; K6 moves the centre up/down |

Video on **S11 Source** (Video / Osc). Generated avid; output conditioning before the blanking gate. Latency
20 clocks. Colours in the hardware convention (u = Cr, v = Cb). EBR: 16 line buffer + 1 tan ROM.

## Controls

| Control | Function |
|---|---|
| K1 Rays | number of rays, 2–8 |
| K2 Halo | halo radius |
| K3 Ring | ring width |
| K4 Ground | Maroon / Navy / Forest / Ochre / Plum / Teal / Rust / Slate |
| K5 Spin | ray rotation, bipolar |
| K6 Centre | vertical position of the centre, 25–75 % |
| S7 Rays | On / Off |
| S8 Halo | Dark / Glow |
| S9 Cutout | video Inside / Outside the halo |
| S10 Palette | Pink-white / Cyan-yellow rays |
| S11 Source | Video / Osc |
| **P12 Rays** | ray width: hairlines → a full disc |

Resolution: everything in virtual 1080-line units (the halo stays round at SD).

## Status

- v0.1.0 (2026-10-20): built with **router1** (router2 stalled on every seed), 6/6 seed 1, HD 77.9 / 81.5 / 78.9 MHz (Analog / HDMI / Dual), 5.5k LC (72 %), 16 EBR. **Not yet flashed.** Rebuild with `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh radiant`. A first version with 24 Q.8 ray DDAs (3 lines per ray, pink beside white) hit 6.6k LC and the SD configs could not be placed; the rays now alternate pink / white with 2 lines each.
