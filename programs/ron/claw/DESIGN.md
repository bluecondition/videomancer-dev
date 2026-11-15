# CLAW: design, budget, and what changed from the spec

Proof of concept v0.1.0 (2026-10-31). It covers the spec's phases 0–2 plus a minimal phase 3 (grab, carry, win, drop): the art pipeline, the reference renderer, the static scene, and the claw's motion. The spec's own rule was "Do not silently downgrade the look. Ask." Ron was asleep and told me to keep going, so every compromise is written down here instead. Nothing below is final; each item has its alternative.

## 1. The chip, and how Invaders spent it

| Resource | iCE40 HX4K | Wizard Invaders v0.7.7 |
|---|---|---|
| Logic cells | 7680 | ~6610 (86 %), 94 % at v0.7.3; "chip full" since v0.7.0 |
| Block RAM | 32 EBR × 4 kbit = **128 kbit (16 KB)** | 28 / 32 EBR |
| DSP / multipliers | **none** | 0 (shift-add and serial multipliers only) |
| Clock | 74.25 MHz at HD | 77–87 MHz post-route |

**Where Invaders' memory went:**

- 1-bit sprites: wizard 128², 6 aliens 48² × 2 frames, CRT TV 64² × 2, bunker 128×112, explosion 48² × 4.
- Eye and trail-gap masks.
- A 5-frame TV-screen animation.
- A debris ROM.
- Two bunker-chunk RAMs.

**Where its logic went:** the breadth of the per-pixel object tests (~13 regions × subtract + compare), the 16:1 sprite bit selects, an 8-phase physics FSM, the death and win story timers, the zoom DDA, and 3 debris slots (~200 LC each). Its sequences were cheap because they reuse existing paths:

- the giant TV is the alien sprite path drawn in the wizard's zoom space;
- fades are a Bayer dither against a level;
- the explosion is one sprite with palette rings.

CLAW's sequences should be built the same way.

## 2. CLAW v0.1.0 budget (measured)

| Block | EBR | LC (approx.) |
|---|---|---|
| Prize sprite ROM: 6 types × 48×48 × 4 bpp, split 8k+4k+2k px | 14 | stamper ~250 |
| Line buffer: 2 banks × even/odd × 512×8 | 4 | clear-behind + muxes ~100 |
| Palette: 256 × 30-bit YUV (masters, 2 depth hazes, 7 themes) | 2 | — |
| Font: 32 glyphs, 16×16, 2 bpp | 4 | text path ~200 |
| Corner-inset table (8 radius classes) | 1 | per-line sequencer ~400 |
| Pile slot table (HD + SD lists) | 1 | — |
| Cabinet slot compares (9 slots × 5 compares) | — | ~600 |
| Composite, bulbs, wall, glass, video, output conditioning, gen. avid | — | ~900 |
| **Total (measured, full build)** | **26 / 32** | v0.1.0 ~6330 / 7680 (82 %); **v0.2.0 ~6945 (90 %, router1, HD 79–81 MHz)** incl. the core |

At 26 EBR, the rest of the game can use 4 more EBR before the 30/32 routing cliff. **v0.2.0 is at 90 %:** only ~300 LC remain before the ~94 % practical ceiling. The next sequences (WIN flash and banner, MISS, SO CLOSE) need a trim pass first. The biggest lever measured is the per-slot cost: a full 11-bit slot is ~400 cells, so narrow slots for the prongs and joystick would save ~80 each. Next comes the text path (~350). The grab, carry and win loop (phase 3-lite) is already included in that 82 %. That is roughly two Invaders-style sequences' worth, which is why the priority list in §3H matters.

## 3. Spec items that did not fit, or changed

**A. Prize sprites: the biggest one.**

- **Spec:** 14 prizes, ~96×96, 4 bpp, drawn 2× at 1080p. That is 14 × 36.9 kbit = **516 kbit, four times the entire chip's block RAM.**
- **Built:** **48×48 at 4 bpp, drawn 4×** (192 px at 1080p, the same on-screen size). Halving the size was preferred to halving the depth, as the spec asked.
- **Consequence:** a sprite pixel is 4×4 screen pixels at 1080p, so the prizes are crisp chunky pixel art, not smooth vector-looking toys. This is a visible downgrade from "not visibly low-res", so you should rule on it.
- **PoC roster:** 6 types (TV, VHS, DVD, Spark, Color Bars, Videomancer fallback) = 13.5 EBR.

The full 14-type roster at 48×48 × 4 bpp would need 31.5 EBR, so it cannot all exist. Options:

1. **48×48 × 4 bpp, at most ~8 types** (18 EBR). This is the current look. It needs about 4 EBR found elsewhere, for example a 1 bpp font with a procedural outline.
2. **32×32 × 4 bpp, all 14 types** (14 EBR), drawn 6×. Chunkier still.
3. **48×48 × 2 bpp + a 4-colour palette per type, all 14 types** (16 EBR). Same resolution, but only outline + 3 colours per prize, which loses the shading ramps.
4. **Vector prizes:** each prize as ~10–20 primitives (rounded rects, circles, capsules) rendered into the line buffer by a span engine.
   - Gains: crisp at every resolution, tiny storage, real scaling and rotation.
   - Cost: by far the largest LC and engineering risk; it may not fit next to the game logic.

My recommendation is 1, plus a fifth option: **reuse the line-buffer stamper to scale the hero prize** during the win sequence. The same 48×48 art then zooms to centre stage with a DDA, which the stamper can do because it walks source pixels.

**B. Back rows are not smaller.**

- Every row is drawn at 4×. Depth comes from a darker, hazier palette per row (built into the palette), a higher position, and occlusion.
- **Why:** resampling 48-px art by 0.875× or 0.75× drops whole source columns, and the 1-px outlines break up. True per-row scale needs pre-scaled art (2.3× the ROM) or vector prizes.

**C. Rotation (v0.2.0):** quarter turns are supported in the stamper (transpose/flip addressing, nearly free) and used for the tumble and the pile. Arbitrary-angle rotation for one or two sprites (falling prize, WIN hero) is possible at about 300–500 LC; it looks jaggy at 4× pixels. "Tilt" is a shear, a ±12-unit lean across the sprite's height, not a rotation. Mirroring is real.

**D. Prize door window.** It is 88 units tall and shows the top ~2/3 of a 192-unit prize, a "peek through the door". A smaller sprite would need scaling (see A, option 5).

**E. Font.** 32 glyphs, missing G, Q, V, X, Z (O is drawn as 0). The words the sequences need are fine, except "SO CLOSE" (needs O → 0, acceptable) and anything with G or V. A full 64-glyph font costs 4 more EBR.

**F. Live video on the TV and Videomancer prize screens.** There is no frame store, so the screens can only show the input **as seen through the screen**: the crop of the live picture at that position, not a shrunken picture. This is cheap (a nibble code in the art means "video here"). Not built yet.

**G. Rounded corners.** Every procedural element uses one of 8 radius classes, with a 5-unit outline (4 on the flap, 2 on the cable). That is the same visual language, but not literally a single radius: the marquee and bezel use 30 and the small parts 13–16.

**H0. Cut menu (v0.3.0, chip at 96 % LC / 30 of 32 EBR).** These are measured by standalone-synthesis ablation, in packed cells (approximate):

| Item | Cells | EBR | Visible effect if cut |
|---|---|---|---|
| Synth placeholder prize | 0 | **2** (30→28, off the routing cliff) | one fewer prize type |
| Prong left-highlight stripes | ~85 | | prongs flatter chrome |
| Glass streaks | ~85 | | glass reads less "glassy" |
| Claw depth scaling | ~100 | | claw same size at all depths |
| Marquee bulbs (after the table fix) | ~130 | | no chasing bulbs |
| Depth haze | ~150 | | back rows no darker |
| Output conditioning ([1 2 1] + luma scale) | ~160 | | **not advised:** HD HDMI thick bars |
| Joystick (whole) | ~340 | | no joystick |
| Text (title, scores, DROP) | ~350 | | no text |

The next sequences (WIN flash and banner slam, hero zoom, confetti, MISS, SO CLOSE) need roughly 600–1000 cells.

**H. LC for the rest of the spec.** These are not all going to fit beside v0.1.0 on this chip:

- grab, slip and drop physics with tumbling prizes and refill rain;
- six sequences with confetti, rays, palette cycling, a scanline wipe and zooming heroes;
- attract mode, rigging, and the TILT shake.

Invaders hit the ceiling with a much simpler scene. Suggested priority, cheapest-and-most-important first:

1. Grab, carry and the chute.
2. Win (flash, WIN! slam, hero zoom via the stamper, confetti as memoryless particles).
3. Miss and drop.
4. Attract.
5. Jackpot extras (palette cycle and scanline wipe are cheap; rays less so).
6. TILT.
7. Refill rain.

## 4. Resolution consistency (warnings)

- **720p** draws the 1080p picture exactly, at 2/3 scale.
- **SD** is a 1440-unit 4:3 window, so the cabinet layout spans the narrower width.
  - The pile has **5 prizes per row instead of 6** (20 instead of 24).
  - The marquee text sits closer to the title.
- **SD line budget is tight:** 858 clocks per line.
  - The worst SD line crosses 16 prizes, about 700 clocks with the per-type column trim.
  - That leaves room for only about 2–3 extra moving prizes on one SD line, for example during a refill. At HD and 720p there is 2–3× headroom.
- **Horizontal granularity:** sprites are placed in 2-unit steps at every resolution. That is 2 px at 1080p, 1.3 px at 720p and 1 px at SD. Moving prizes (held, falling) step at that granularity.

## 5. Architecture (for whoever continues)

- **Virtual 1080-line coordinates** from the timing ID, plus a generated avid (cubist/aurora recipe).
- **Procedural cabinet.**
  - 16 rounded-rect elements are mapped onto 9 slots. Elements that share a slot never share a scanline.
  - A per-line sequencer (hblank, ~25 clocks) reads each element's corner insets from one EBR and writes outer, inner and highlight x bounds into slot registers.
  - The pixel path is 5 compares per slot.
  - Prong lean is per line (row × open >> 5), so opening the claw costs nothing per pixel.
- **Pile.**
  - A dual-bank line buffer. One entry is 2 virtual units, and an even/odd RAM pair is one source pixel at 4×. Odd placement shifts by one entry using the previous pixel.
  - At each line start, the stamper walks the slot table (back row first), skips rows that miss, and streams each hit row: one source pixel per clock, trimmed to the type's opaque columns.
  - Entries hold final palette addresses, depth haze included. The displayed bank is cleared one pair behind the beam; both banks are wiped in vblank and line 0 is prestamped.
- **Compositing** is in palette-index space, with one palette EBR. Glass streaks and the acrylic lift Y afterwards, and S10 swaps the wall for dimmed video.
- **Output:** [1 2 1] smoothing on Y, U and V plus the luma scale (HD HDMI bars recipe), then the blanking gate.
- **Art pipeline:** `art/claw_art.py`.
  - Vector parts on an 8× canvas, auto-shaded (top-left highlight band, bottom-right shadow band, part outlines), majority-downsampled to 48×48, with a closed 1-px silhouette outline.
  - Packed with the font, palette, insets, slot table and element table into `claw_pkg.vhd`.
- **Supplied art:** drop `art/supplied/<videomancer|...>.png` (alpha = transparency). It is fitted to 46 px, quantised to the master palette, outlined, and replaces the fallback.
- **Reference renderer:** `art/claw_ref.py` uses the same integer rules (slots, insets, stamper, composite) at 1080p, 720p and SD.

## 6. v0.1.0 controls

| Control | v0.1.0 |
|---|---|
| K1 Gantry X | claw x, glided; travels over the chute |
| K2 Depth | front / middle / back: the claw shrinks 1.0 / 0.82 / 0.66 and goes behind the rows in front of it; the joystick rocks on a change |
| K3 Grip | grab tolerance: how centred the claw must be (20–83 units) |
| K4 Speed | glide speed for gantry and cable |
| K5 Cabinet | 6 themes: Classic, Neon, Jackpot, Vaporwave, Mint, Studio |
| K6 Rig Odds | v0.5.0: slip chance per grab when S11 is Rigged |
| P12 Cable | claw height (push down to lower) |
| S7 Drop | any edge opens / closes the claw. Close low over a prize = grab; open over the chute = it drops through the flap (WIN); elsewhere it falls, tumbles and lands in the pile |
| S8 Mode | v0.4.0: Manual / Auto (autopilot) |
| S9 Grab | v0.5.0: Auto-close (default) / Manual |
| S11 Rigged | v0.5.0: Honest / Rigged (slip on the lift, odds on K6) |
| S10 Video | input video, dimmed, on the back wall |

## 7. Questions for Ron

1. **Prize resolution (A):** 48×48 × 4 bpp chunky pixel art with ~8 types, or one of the other options? Is the pixel-art look acceptable at all?
2. **Back rows (B):** is depth by haze, position and occlusion enough, or is true per-row scale required (only realistic with vector prizes)?
3. **Scope (H):** confirm the priority order, or tell me which sequences matter most.
4. Supplied art for the Videomancer, the Vidiot and the LZX token: PNGs in `art/supplied/` whenever ready (they need to read at 48×48).
