# Triplicate — the picture photocopied in red, green and blue, cascading over a wall of prints (Heirloom #53)

**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Source: `photos/selfie.jpg`.
v0.1.0 tinted and cascaded copies of the video without the collage's structure. This version rebuilds the
collage: the measured bear cut-out, sizes, positions, inks and print wall. The camera picture is printed
through every cut-out. Osc mode substitutes a generated skull-faced bear with a computer.
Working files: `.heirloom_work/rb_triplicate/` (`notes.md`, `proto.py`, `imodel.py`, `sheet_video.png`,
`cmp_final.png`).

## Reading of the photo (measured)
The photo is a portrait paper collage, 3527 × 4988 (0.707:1). Measurements below are in page heights
(1600-px working copy).
- **Four copies of one cut-out.** The cut-out is a bear with a skull pasted as its face and a TI-99
  computer on its lap.
  - The back three are one-colour photocopies in red, green and blue. The front one is the full-colour
    original.
  - Measured tops / heights: red 0.057 / 0.759, green 0.233 / 0.647, blue 0.379 / 0.587, front
    0.583 / 0.386.
  - Size ratios between neighbours: green/red 0.852, blue/green 0.907, front/blue 0.658.
- **The cascade.** Steps (dx, dy): red→green 0.110, 0.176; green→blue 0.121, 0.147; blue→front
  0.097, 0.204. The slope is about 54° below horizontal, and each copy covers half of the one behind it.
- **The cut.** The three tinted copies share one outline to within 0.01 h. The bounding box is 0.604 wide
  per unit of height.
  - The dome rises between two rounded ears (±0.17 h at the ears).
  - The neck is ±0.14 h.
  - The bell body flares to −0.244 / +0.36 h, wider on the haunch side.
  - The base is flat. The edges are hard scissor curves with no rim or shadow.
- **Inks.** Each copy is ink over tinted paper with a steep photocopy curve: about 40 % of the head region
  is solid ink, and the ramp between ink and paper is linear. Values are hardware 10-bit, u = Cr:

  | Copy | Ink (Y, U, V) | Paper (Y, U, V) |
  |---|---|---|
  | Red | 350, 818, 424 | 824, 585, 475 |
  | Green | 440, 378, 432 | 872, 479, 474 |
  | Blue | 345, 332, 698 | 833, 449, 564 |

  Light toner specks show in the solid areas.
- **The wall.** Square greyscale prints of skull piles on a regular grid with a period of **0.094** both
  ways.
  - The grey is neutral (mean chroma 3.8/255). Luma mean is 105, with the 2 % / 98 % points at 40 / 214.
  - Per-print exposure varies (std 8.9).
  - Some rows are half-height or shifted. Some edges show thin light seams.

## 16:9 mapping
- The frame height equals the page height. Every element keeps its size and vertical position relative
  to the height.
- At Cascade 50 % the page composition sits centred. The wall of prints simply continues to the frame
  edges.
- The P12 slider widens the cascade's horizontal steps beyond the page (up to 2.7×) to use the wide
  frame.
- Geometry is in virtual 1080-line units, so 720p renders the same picture.

## What it does
1. **Store.** The source (camera, or the Osc generator) is written into a 128 × 128 × 4-bit luma store
   and a 32 × 32 U4V4 chroma store.
   - Luma is box-filtered over power-of-2 windows, with an accumulator RAM doing the vertical sums.
   - Luma is held in four parity banks, so the display can read it bilinearly.
2. **Copies.** Four cut-outs use the measured outline: a 128-row ROM of edge intervals, interpolated per
   line, so the curves have no stair steps.
   - Copy heights are S · (0.759 · H − R · ΔH), giving 0.759 / 0.647 / 0.587 / 0.386 H at the defaults.
   - The copies are anchored at their measured bottoms (0.816 / 0.879 / 0.966 / 0.969 H). The cascade
     scales the offsets between them.
   - The front-most copy wins at each pixel. Each copy shows the whole store, magnified bilinearly.
3. **Photocopy.** Density = 2 (HI − L), clamped, with HI = 115..345 set by K3 Ink.
   - Toner grain: 3/256 of pixels lose 3/8 of their density (light specks); half of the pixels lose 1/16.
   - The density is dithered to a 5-bit level that indexes a 32-entry ink→paper ramp fitted to the photo.
   - The front copy takes a luma ramp plus the chroma store, so it shows in colour.
4. **Wall.** Square prints (101 lines at the default) each show a 64 × 64 window of the store in grey.
   - The window is chosen by a hash and biased towards the top of the picture (the head).
   - Some prints are mirrored. Exposure varies per print in four steps.
   - About a quarter of the left and top edges get a light seam. Each row of prints is offset sideways.
   - The wall is 7/8 of the luma ramp, plus the K5 Wall offset.
5. **Output.** Shell output conditioning, then the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Recede | the size step red → green → blue → front | 0 = all copies the red's size; 50 % = the photo's recession; 150 % = steeper |
| K2 Size | the copies' scale | 60–130 % of the photo's sizes (100 % at 50 %) |
| K3 Ink | the photocopy density | faint prints → the photo's ~40 % solid ink → solid silhouettes |
| K4 Tiles | the wall's print size | 43–323 lines (default 101 = the measured 0.094 H) |
| K5 Wall | the wall's exposure | dark ossuary → bleached prints |
| K6 Speed | (motion) wall and cascade | bipolar, still in the centre (±16): the wall scrolls up or down (7.5 lines/s at the default, about 26 at full) and the cascade breathes ±10 % horizontally (period 17 s at the default, 4.3 s at full) |
| S7 Order | which ink is at the back | RGB (the photo) / BGR |
| S8 Inks | the ink set | RGB / CMY |
| S9 Front | the full-colour original | Colour / Ink (a fourth, black plate) |
| S10 Flip | the copies' facing | Off / On (the 2nd and 4th copies mirrored) |
| S11 Source | the picture being copied | Video (default) / Osc |
| **P12 Cascade** | the diagonal cascade | 0 = all four nested on one spot (matryoshka); 50 % = the photo's steps; 100 % = horizontal steps 2.7× (fanned across the frame). Vertical steps reach the photo's at 50 % and stay there |

P12, K1, K2 and K4 glide (1/8 of the gap per field). The control sheet is
`.heirloom_work/rb_triplicate/sheet_video.png` (from the proto); every control changes the picture.

## Video
- The camera picture is what gets photocopied. It is printed through all four bear cut-outs:
  - three as red / green / blue ink density;
  - the front one in its own colour.
- The same picture, in grey, fills every print on the wall, each print a different window.
- Osc substitutes a generated figure drawn into the store, so all copies and the wall print it:
  - a shaded skull with eye sockets, nose, cross and rows of writing marks;
  - a cream monitor with a blue screen and a scrolling logo strip;
  - a console with dark keys, on dark fur.
- Every control acts in both modes.

## Engine
- **Per-field register machine** (vblank = "no avid for 1024 clocks").
  - Microcode is in LUT logic. The register file is one EBR (16-bit words).
  - One serial shift-right multiplier with arithmetic result shifts, one restoring divider, and one
    post-op stage (add / clamp / min / max).
  - It runs the knob glides, the cascade / size / recession curves, print size and steps, and per copy:
    height, 1/height, row and column steps, top, left and start phases.
- **Per-line engine in hblank** (8 clocks after the generated avid falls, about 36 clocks).
  - It steps the four copies' rows, reads and interpolates the outline ROM (16 reads, then a lerp product
    stage), loads the copies' column DDAs, and steps the wall rows.
- **Pixel path:** column DDAs and edge compares; front-most winner; bilinear store read; density, grain
  and dither; palette ROM; compose; output conditioning; gate.
- **24 EBR:** luma store 16; chroma store 2; accumulator 1; register file 1; outline ROM 1; palette 2;
  sync delay 1. All collision-free RAMs carry `no_rw_check`.
- **Contracts:**
  - generated avid;
  - virtual 1080-line coordinates from the timing ID;
  - one 21-clock latency in every mode (`processing_delay_clks` = 21);
  - output conditioning (luma [1 2 1] and scaled toward ~777; chroma [1 2 2 2 2 2 2 2 1]/16) before
    the blanking gate;
  - colours in the hardware convention (u = Cr).

## Status
- **v0.5.0 built 2026-10-21, router2, 6/6 on seed 1, not flashed.**
  - Post-route Fmax: HD Analog 74.28, HD HDMI 76.84, HD Dual 74.67, SD Analog 71.73, SD HDMI 59.39,
    SD Dual 71.55 MHz.
  - Fit: 7058–7080 / 7680 LC (92 %), 24/32 EBR. All six bitstreams pass iceunpack.
- **HD margins are very thin** (HD Analog +0.03 MHz, HD Dual +0.42). If a rebuild misses, try more seeds
  or `NEXTPNR_ROUTER="--router router1"`. The remaining critical paths are routing in the register
  machine's control fan-out (cycle counter / run flag → multiplier enables, register-file write data).
  Pre-decoded phase flags are the restructure if needed.
- **GHDL renders** (720p60, decimation 1, defaults, exact hardware hues via raw2hw), in both Osc and
  Video (fed a crop of selfie.jpg):
  - copy tops 0.058 / 0.235 / 0.381 H (photo 0.057 / 0.233 / 0.379);
  - visible bottoms 0.815 / 0.879 / 0.965 (photo 0.816 / 0.879 / 0.966).
- **Motion:** 4 frames apart at the default Speed, the wall scrolled up 0.35 px (as designed) and the
  cascade breathes.
- SD and interlace modes were not simulated.

## Deviations from the photo
- **Fixed outline.** Each copy prints the camera picture (or the Osc figure) through the fixed, measured
  bear outline. In the photo the copies are photocopies of one paste-up with a large skull. Here the
  cut-out does not follow the camera's subject.
- **Softer copies.** A 128-row, 4-bit store is magnified to up to 820 lines, so fine detail (the skull's
  Greek writing) becomes texture.
- **The wall's content.** The prints are windows of the same live picture (Osc: the generated skull), not
  separate ossuary photographs. The photo's half-height and shifted rows become per-row offsets.
- **Whites and chroma.** Whites are capped at about 777 by the shell's output scale (the HD HDMI bar fix),
  so the papers are slightly dimmer. Chroma is softened by the 9-tap filter (invisible at HD). The front
  copy's colour comes from a coarse 32 × 32 chroma store.
- **Motion.** The photo is still. The default Speed gives a slow drift and breathing instead.
- **SD 4:3.** The picture is the same, with less wall at the sides. At P12 100 % with Size at maximum, the
  red copy's back edge clips at the 4:3 frame edge.
