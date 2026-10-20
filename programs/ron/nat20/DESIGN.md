# NAT20 — rendering design

Status 2026-10-18: Ron approved the proposal ("proceed with phase 1"): fixed zoom with the bounding sphere at 95% of the height, and the cut order as written. Phase 1 is built; see "Phase 1 as built" at the end for where the RTL refined this proposal.

Later on 2026-10-18 Ron approved the LC trims (v0.1.1): "the numbers themselves are the most important thing". The engraved numerals now rank right after geometry and AA, above lighting and bevels. He also floated a new direction: several smaller dice of different types on screen at once, "rolled" somehow (§9).

2026-10-19: Ron didn't like v0.1.x's creases and outlines, and asked for a proof of concept of a much smaller die (~10% of the frame height) rendered in fine detail. v0.2.0 is that proof of concept, on a new renderer (§10). §1–§8 describe the full-size design it replaces. Ron then liked the v0.2.0 look and asked for the numbers on each die: v0.3.0 (§11).

Design-stage files:

| File | What it is |
|---|---|
| `gen_dice.py` → `dice.json`, `DICE_CHECK.md` | All five dice as data, every numbering rule checked face by face (ALL PASS) |
| `nat20_model.py`, `sheet.py` | Float look model of the proposed hardware pipeline (the design target for the RTL) |
| `glyph_sdf.py` | Numeral distance fields, stored and sampled exactly as the FPGA would (4-bit, 4 banks, bilinear) |
| `glyph_res_compare.png` | Numerals from the stored fields: 24×32 at 300 px, 16×24 at 300 and 150 px, serif stand-in at 150 px |
| `prev_*.png` | Model renders: `prev_dice` (all dice), `prev_finish` (8 finishes), `prev_ink` (7 inks + sharp edges), `prev_light` (P12 sweep), `prev_check` (every face of every die presented), `prev_numerals_1080` (1:1 HD crops) |

## 1. Rendering approach

Streaming, orthographic, no frame buffer. It builds on CUBIST's proven parts: the microcoded vblank engine (`uasm.py`/`uemu.py`), parallel per-face affine DDAs, a linear-light shading rig with a gamma ROM, and the generated avid.

**Vblank engine (microcode).** Once per field it does knob glides and zone hysteresis, AUTO pose sequencing, the die spin-swap and style crossfades (as parameter morphs). It builds the rotation matrix, transforms the vertices and culls back faces. For each visible face it writes:
- the DDA setup;
- the shading constants: n·L, n·H and the light and half vector in the face frame;
- the highlight centre;
- the glyph ids and silhouette flags.

The worst case is the d20, at about 8–10k clocks, inside NTSC's ~14.5k vblank.

**Face ownership.** Each visible face gets two affine coordinates (a, b), its barycentric weights, which are plain accumulators: +gx per pixel and a wrap once per line, using CUBIST's "park the wrap in gx" trick. Inside means a ≥ 0, b ≥ 0 and a + b ≤ 1. A tiny slack turns crease gaps into harmless overlaps, and a priority encoder picks the owner. Squares (d6) test a, b ≤ 1. Kites (d10) are split into two triangles.
- The d20 needs **10 slots** (20 accumulators). A per-line "lane" scheme would not save enough: up to 8 d20 faces share one scanline, measured over 3,000 random orientations.
- After the owner mux, the face frame gives everything else:
  - λ0 = 1 − a − b, so all three edge distances come out of the inside test for free;
  - the numeral frame is a fixed per-die affine map of (a, b), because numerals point at a vertex (triangles), an edge (squares) or the apex (kites);
  - d4 corner numerals are the same map rotated by 120° (a permutation of λ);
  - the two-digit split is the sign of the numeral-frame x.

**Numerals (engraved).** Each pixel takes **one** distance-field sample: four 2×2-interleaved banks and a bilinear blend. The same four taps give the field's gradient, so one sample yields:
- the lip, anti-aliased at the zero crossing;
- the cut wall, a fixed band of field codes, lit by gradient·light: the wall facing the light catches it and the other falls into shadow;
- the ink floor, matte with no specular, so the face's highlight breaks cleanly at the numeral;
- the lip shadow cast onto the floor.

Unpainted ink keeps all of this but fills the floor with body colour, and still reads (see `prev_ink`, `prev_numerals_1080`). Pips are analytic discs drilled the same way. The 6 and 9 underline on the d10 and d20 is an extra box distance.

**Light.** There is a key light (P12 arc), a fill and ambient. Rounded edges are a normal tilt read from a profile ROM by distance to the nearest edge, so shading stays continuous across creases and internal edges need no AA. Highlights glide in two ways:
- a sharp lobe from the tilted bevel normals (the edge catch-lights);
- a soft "softbox" spot on each flat face, falling off around that face's reflection point, which the engine moves every frame. This closes CUBIST's known gap that flat faces had no gradient.

The spot uses the octagonal metric, so it needs no squarers.

**Materials** are parametric (diffuse, specular strength and width, environment, sheen), tinted by the colour, so a finish crossfade is a parameter glide.

**Backdrop.** A gradient, vignette and soft contact shadow, all from second-difference accumulators with no multipliers. Video comes through a 16-deep EBR delay.

**Resolution.** The engine projects in virtual 1080-line units with the per-mode pixel aspect, so the picture is the same at SD, 720p and 1080p.

## 2. Things that have to bend (need your call)

1. **"Fills at least 75% of screen height at every orientation"** cannot hold at a fixed zoom except on the d20. Measured vertical extent ÷ circumscribed diameter over random orientations:

   | | d4 | d6 | d8 | d10 | d20 |
   |---|---|---|---|---|---|
   | min | 0.58 | 0.58 | 0.58 | 0.45 | 0.80 |
   | max | 0.82 | 1.00 | 1.00 | 1.00 | 1.00 |

   - **Recommended:** a fixed per-die zoom with the bounding sphere at ~95% of the height. Nothing ever clips, and every AUTO presentation pose sits around 75–95%. MANUAL extremes dip lower; the worst is a d10 seen end-on, at ~43%.
   - **Alternative:** an eased auto-framing zoom that holds 75–95%. The die then visibly "breathes" as it turns. CUBIST dropped autozoom for exactly that reason.
2. **Ornate font (S3) does not fit in block RAM.** One font at the quality shown takes 8 of 32 EBR (20×36 texels × 10 glyphs, 4-bit). A second font is +8 EBR, and the budget below is already at the ~30-EBR congestion cliff. Options:
   - (a) drop Ornate and give S3 another job;
   - (b) both fonts at 16×24 in the same 8 EBR. Fine on the d20 (~130 px numerals), visibly polygonal on the d6/d8 (~215–255 px).
3. **Marble, Wood and Glitter will probably not fit alongside everything else.** True die-space textures need the 3D surface position per pixel, about 6 extra multipliers (~400 LC) plus each texture's own logic, so ~600–900 LC for the three. Proposal: build the core first, measure, then add textures in the order you choose while room lasts.
   - A cheap fallback for Glitter (~150 LC) hashes sparkles in face-local 2D. Sparse dots don't show seams, but a sparkle can't straddle an edge.
   - Wood and Marble have no cheap seam-free version.
4. **Gem** is faked, with no refraction: a depth gradient (rich centre, glowing edges by edge distance), sharp facet glints and, over video, the video tinted and darkened through the face. It fits.
5. **Style crossfades are parameter morphs**, not two renders blended. Colour, ink and finish parameters glide over a few frames. Changing to or from a textured finish fades the texture out and back in; font and d6-pips changes dip the numerals out and back in.
6. **Silhouette vertices get a small chamfer, not a true round.** CUBIST measured real round outline corners at hundreds of LC. Interior vertices and edges are shaded round. The d4 tips will look slightly blunted.
7. **SD and resolution warning:** geometry matches at all resolutions, but at 480i the cut walls are only ~1–2 lines wide, so the engraving reads weaker at SD than at HD. Numerals are ~60 lines tall on the d20 at SD.
8. **Future features:**
   - **Set view (all dice at once)** is out of reach for this architecture on the HX4K. It would need ~25 face slots, so it needs a different renderer.
   - **A d12** needs a pentagon slot type: 6 visible faces with 3 extra edge tests each, ~300 LC.
   - **Percentile d10** and **rolling** are data and microcode on what's proposed.

## 3. Resource estimate (HX4K: 7,680 LC, 32 EBR)

CUBIST v0.5.5 for scale: 7,270 LC / 27 EBR. Its 12 face DDAs measure ~700 LC (380 LUT + 334 carry + 583 FF), and its engine ~1,100 LUT.

| Block | LC (est.) | EBR |
|---|---|---|
| Video infra, generated avid, output gate, [1 2 1] HD-HDMI conditioning | 450 | – |
| Microcode engine (ALU, register file, Booth serial multiplier, divider) | 1,300 | 2 RF + 1 sine |
| Microcode ROM (1024 × 32) + die/pose tables | – | 8 + 2 |
| 10-slot face DDAs + wrap sequencer | 1,150 | 2 |
| Ownership, owner mux, face frame, edge distances, AA | 850 | 4–5 descriptor ports |
| Glyph sampler (4 banks, bilinear, gradient) | 350 | 8 |
| Bevels, diffuse, cut walls, lip shadow, specular spot | 700 | 2 (profile, spec) |
| Colour/material (Gloss, Matte, Metal, Pearl, Gem), ink, compose | 600 | 1 (gamma) |
| Backdrop + video delay + compose | 300 | 2 |
| **Core total** | **~5,700–7,300 (75–95%)** | **~31–33** |
| + Marble / Wood / Glitter (3D) | +600–900 | +1–2 |
| + Ornate font | – | +8 |

The EBR line needs trimming to ≤ ~30, for example a narrower microcode word or merging the small ROMs. Treat the LC range as the honest uncertainty: hand estimates have been off before, and only a build proves fit.

## 4. Protect order (your list) and what I'd build

1–4 (geometry and numbering, AA, lighting and bevels, engraved numerals) all fit in the core. From 5–6, the core includes Gloss, Matte, Metal, Pearl, Gem and pips. **Not in the core:** Marble, Wood, Glitter and the Ornate font. Nothing is cut until you say so.

## 5. Choices made in the die tables (confirm or change)

- **d4:** vertex-read, vertex k shows k. Seen from outside at vertex 4, 1→2→3 run counter-clockwise, the same hand as the d6.
- **d6:** Western: 1, 2 and 3 run counter-clockwise round their shared corner, and opposite faces sum to 7. Numerals point up the side faces; on the top and bottom faces they point toward +y.
- **d8:** opposite faces sum to 9. Equal vertex sums (18) are **impossible** on an octahedron: each vertex would need faces {2,3,5,8} or {1,4,6,7}, and the x, y and z vertex sets can't all be such unions. Used the most even spread instead, vertex sums 14/16/18/18/20/22. Numerals point at the nearer pole.
- **d10:** the polar dual of the *uniform* pentagonal antiprism. Kite angles are 36° (apex) and 108° ×3, the long-to-short edge ratio is φ² = 2.618, and the side vertices sit at φ/2 = 0.809 of the way down the axis. Odd numbers ring one apex, even the other, and opposite faces sum to 9. The odd ring, counter-clockwise from above, is 1 7 3 9 5. Numeral tops point at the face's apex.
- **d20:** opposite faces sum to 21. A seeded search gave **vertex sums all 52 or 53** (the ideal is 52.5) and every face's three neighbours sum to 31–32. No face is surrounded only by its own half. 20 neighbours 6, 8, 17; 1 neighbours 4, 13, 15. Numeral tops point at the vertex highest along a fixed axis through the 20.
- **Font:** Lato Bold (SIL OFL, so it can ship with a GPL program). It needs vendoring into the repo with its licence.

## 6. Control map

| Brief | Register | Notes |
|---|---|---|
| K1 Die | `registers_in(0)` | 5 zones with hysteresis; spin-swap < 0.5 s |
| K2 Yaw / Pace | `registers_in(1)` | MANUAL 360°, AUTO speed |
| K3 Pitch / Linger | `registers_in(2)` | MANUAL 360°, AUTO hold time |
| K4 Colour | `registers_in(3)` | 12 sets, crossfade |
| K5 Ink | `registers_in(4)` | 7 inks, crossfade |
| K6 Finish | `registers_in(5)` | crossfade |
| S1 Mode … S5 Background | switches 7–11 = `registers_in(6)` bits 0–4 | |
| P12 Light | `registers_in(7)` | headline control |

In AUTO, poses come from a per-die table of upright presentation angles plus a hero tilt. The engine eases between them in Euler angles, which gives tumbling on varying axes, and shuffles the visiting order like a deck so every face comes up.

## 7. Build phases (build + flash each; sims only where a new artifact can't be reasoned out)

1. Engine + 10-slot DDAs + ownership: flat-shaded dice with AA silhouettes, MANUAL, die switch.
2. Bevels, key/fill lighting, gliding highlights, backdrop.
3. Glyph sampler: engraved numerals, inks, pips.
4. Materials, colours, crossfades, video background, Gem.
5. AUTO showcase, spin-swap.
6. Textures in your order, if room remains.

## 8. Phase 1 as built (v0.1.0)

Files: `nat20.vhd`, `nat20.toml`; the engine's program `nat20_ucode.py` (assembler and emulator `nat20_uasm.py`, die tables `nat20_tables.py`); `nat20_regen.py` writes the microcode, die table and engine ROM into the VHDL; `nat20_rtlmodel.py` runs the microcode in the emulator and the pixel path in numpy (accumulators, ownership, one-pixel lag, AA), so geometry and microcode bugs show in seconds.

Where the RTL refined §1:

- **Edge functions, not barycentrics.** Each face slot has three accumulators holding its edges' signed distances, scaled per edge by 2^13 · m (2^E ≤ L < 2^(E+1) is the projected edge length, m = L / 2^E), so every accumulator steps 2^13..2^14 units a pixel whatever the edge's size.
  - Ownership is three sign bits per slot.
  - An edge shared by two visible faces is computed ONCE, in a canonical direction, and written to both slots with opposite signs. Two's-complement negation is exact through every step and wrap, so neighbouring faces split a crease exactly: no cracks, no padding.
  - AA is one small 1/m multiply per owner edge.
  - Phase 2's bevel distances and phase 3's numeral frame come from the owner's three distances converted to die units: three per-pixel multiplies by per-slot-edge scales.
- **Everything is triangles.** Squares (d6) and kites (d10) are split along a diagonal that both halves share as a FAKE edge (never a silhouette). 10 slots cover the d20 (≤ 10 faces), the d10 (≤ 5 kites = 10 halves) and the d6 (≤ 3 squares = 6 halves).
- **The engine runs during the field, not in vertical blanking.** Its outputs (DDA table, slot descriptors, per-field globals) are double-buffered by a bank bit in the spare depth of the same block RAMs (no extra EBR) and swap in vertical blanking. The budget is a whole field (~225k clocks at NTSC); the d20 needs ~25k. Cost: one field of control latency (~17 ms). This also frees the sim from its fake 8-line vertical blanking.
- **Ownership lags the edge values by one pixel** (CUBIST's arrangement): the owner is decided from the accumulators at x, and its edge values are read one clock later, at x + 1. The silhouette inset (1.5 px) absorbs it.
- **Step window:** the accumulators step only inside the die's horizontal extent [x0, x1), and the line wrap starts as soon as the extent ends. This halves the worst-case drift and lets the 26-bit accumulators cover any visible face.
- **Labelled knobs** (K1 Die, K4 Colour) read a value under N as the label index: at load the firmware writes the label index, not a position.

Phase-1 shading is flat per face: key + fill + sky + ambient through a gamma curve. Faces with nearly equal light can merge; phase 2's bevels and gliding highlight separate them.

### v0.1.1 trims (no intended visible change)

Pair slots (26 accumulators instead of 30), a 3-bit 1/m shift-add in the AA path, live knob reads, and descriptor RAMs read at delayed owner addresses instead of being carried down the pipe. The front-face threshold now scales with the frame height, so an edge-on face and its parallel opposite can't both count as front. Details and measurements: SUMMARY.md.

## 9. Multiple dice on screen (Ron's question, 2026-10-18; awaiting his call)

**Smaller dice alone don't help this renderer.** Its cost is set by how many faces are on screen at once (three edge accumulators per visible triangle, all stepping in parallel), not by how many pixels they cover. A die a quarter the size costs the same. Giving each extra die its own slots costs ~1,700 LC per d20, so two dice would fill the chip with flat shading and nothing else.

**What would fit: reuse the slots across dice on the same scanline.** On each line, the slot accumulators serve the dice one after another, left to right.
- Between dice, the wrap sequencer reloads the next die's edge values and gradients (the same parking trick it uses at the end of every line). A per-die table in block RAM holds each accumulator's value at the die's left edge, advanced once per line by one shared adder as it is read.
- **Rule:** dice that share scanlines must sit side by side with a gap of about 60 px at HD (the reload time; about half that with parallel RAM reads). A die directly above another has no constraint. Dice can't overlap or touch side by side.
- Every die still gets all 10 face slots, so any mix of d4–d20, each tumbling freely. About 5–7 dice fit across a line.
- Estimated cost: ~300 LC and ~6 EBR (per-die tables), on top of the slot hardware. The engine time is roughly the sum over the dice (the d20 is up to ~55k clocks). That's ample at HD; at SD (~225k clocks a field), more than ~4 d20s would update at half rate.
- Smaller dice help indirectly: more fit on a line, and the numerals can use the 16×24 glyph store (4 EBR instead of 8).
- **Resolution warning:** the gap is a fixed clock count, so it is ~3% of the width at HD and ~8% at SD. Layouts must respect the SD figure, or SD gets fewer dice per line. Numerals on small dice get marginal at SD (~20 lines tall).
- The v0.1.1 trims carry over unchanged. If Ron chooses this direction, it should be built before phase 2, because phases 2–4 hang off the per-die data layout.

## 10. v0.2.0: the miniature on a scanline vector renderer (proof of concept)

Ron, 2026-10-19: v0.1.x's creases and outlines did not look good. He asked: what if the die were small, about 10% of the frame height, but very detailed?

**Why a small die changes the architecture.** v0.1's per-face edge accumulators cost the same whatever the die's size, and anti-aliasing them analytically gave the soft, inset outlines and hard creases Ron disliked. A small die instead frees TIME: one line of the die is ~110 px, but a line lasts 2,200 clocks at HD (858 at SD).

**The renderer (FreeType's "cells" idea, in hardware).**
- The engine turns every visible edge into a crossing record: the sub-lines it crosses (16 a line, half-open, so the edges meeting at a vertex share it exactly), its x there, its slope, and the colour step across it (right face minus left face; a silhouette carries its face's colour, and the backdrop is subtracted per line).
- Per line, each record steps once a sub-line and deposits its step at the crossing's pixel as D += step and F += step × frac. The display's running sum of D minus F is the exact box-filtered coverage along each sub-line.
- So every edge, crease and outline alike, gets exact horizontal coverage and 16 vertical levels. The work is per crossing, not per pixel: ~20–250 clocks a line. It does not grow with the die's size, and leaves most of the line free.
- Faces only need to be polygons: the d6/d10 split diagonals have no record. Colours sum to zero across the die, so nothing can leak outside it.

**Resources.** The 26 edge accumulators, the anti-aliasing pipe and the descriptor RAMs are gone. Standalone yosys gives 2,352 LUT4 / 1,978 FF, against 2,816 / 2,440 for v0.1.1. The line buffer is 6 EBR (D/F × Y, U8, V8, 256 px), the crossing records 2, and each record's line-start x 2: 25 EBR in all.

**Checked** (`.nat20_work/v020/ref_check.py`): an independent rasterizer at the same sub-lines with 256 samples a pixel, over every die and mode.
- The largest error is ≤ 11 codes, which is the renderer's 1/16 px crossing position (an edge moved < 1/32 px). With a finer fraction it is ≤ 3 codes.
- Nothing leaks outside the die.
- Render ≤ 244 clocks a line (budget ~714 at SD), engine ≤ 38k clocks a field.

**What it makes possible next** (Ron's priority order: numerals first):
- **Numerals as vector outlines.** Glyph contours become more crossing records, projected by the engine onto each face, so they are exactly anti-aliased and follow the face in 3D.
  - Needs: an outline table, per-face grouping so records off the line are skipped cheaply, and more engine time per field.
  - SD's line budget is the limit: numerals at SD may need 8 sub-lines.
- **Rounded edges and highlights** as thin bevel strips along each crease: two more records a crease, with a highlight colour.
- **Multiple dice:** records from several dice in one table. Their crossings share the line budget, not slots.
  - The line buffer spans 256 px, so dice side by side need more window.

**SD / 720p warning:**
- The die is sized to the frame height, so it is the same picture at every resolution.
- SD uses a 128 px window (the die is ≤ 119 px there).
- An SD field line spans two frame lines, so SD gets 8 sub-lines per frame line against HD's 16 per line: slightly softer edges at SD, as expected.

## 11. v0.3.0: numerals (Ron, 2026-10-19: "I like the way the dice looks, let's add numbers to each one now")

**Why not vector outlines.** Glyph contours as more crossing records would need ~400 records a d20 (~25 segments a digit), a slope divide each (~300k engine clocks, more than an SD field), and 8× the record table. Instead each numeral is a **sprite**: a 4-bit glyph distance field (Lato Bold, 16×20 texels, bilinear) mapped onto its face by an affine map and sampled once a pixel. Its ink is deposited into the same line buffer as a D-only step: per pixel, the change of (coverage × ink step) from the last pixel, so the display's running sum gives the inked colour over the face.

**The engine (numeral pass).**
- The frame comes from the already-projected corners. Projection is linear, so a fixed weighted mix of a face's corners is the projection of the same mix in die space.
  - Per face shape (triangle, square, kite), ROM weights give a centre P, an up vector UPV and a right vector RT from the four corners starting at the face's up corner or up edge.
  - Per die, three constants place and size the numeral: c = P + UPV·TC, texel steps U = RT·SU and V = −UPV·SV.
- Inverting the 2×2 frame (one divide) gives texel steps per pixel. The glyph box's screen bounds give the lines and the span. The AA scale is √det px a texel (floored at 1, the field's range), and the ink step is the ink lit like its face (the face's light rides in its colour word).
- Data needed beyond v0.2.0:
  - 2 bits a face in the triangle words (the up corner/edge);
  - faces stored in number order (d4: vertices in number order);
  - 58 ROM words (corner weights, per-die constants, inks).
- Skipped, as on a real die: numerals squashed more than 4 texels a pixel (faces nearly edge-on), and anything past 12 sprites (a field never needs more than 10).
- To fit the 1024-word ROM, the engine gained a fused multiply (MULSH), native MIN/MAX and signed table reads (SRDS, ROMS). The knob glide became a subroutine. 995 words, peak 122 registers.

**The renderer.**
- After the edge records, each sprite on the line: fetch 8 words, walk the span one pixel a clock (texel coordinates stepping), the last pixel blank, then write back the next line's texel coordinates.
- The glyph pipe (11 stages):
  - texel cell, then 4 bank addresses (2×2 interleave, so a bilinear sample's taps sit in four banks), ROM, taps in fabric;
  - bilinear across and down, then coverage = 8 − k·d/2048 (×16);
  - luma ink step × coverage, with chroma switched whole at half cover (4:2:2 and the output smoothing hide it);
  - the change from the last pixel into the deposit pipe.
- The record table grew to 512 words. It holds the edges, their line-start x (no separate RAM now) and the sprites. The renderer's write-backs share its write port with the engine, renderer first.

**Look.**
- The RTL model renders the same numerals, placements, orientations and sizes as the design-stage look model in every die and pose tried (`.nat20_work/v020/cmp_look.py`).
- White ink, black on Bone, Frost and Sunflower.
- 6 and 9 underlined on the d10 and d20.

**Font.** Lato Bold (SIL OFL) is read from the system at regeneration time (`nat20_glyphs.py`). The program carries only a 16×20 distance-field rendering of ten digits, not the font. Vendor the TTF only together with its OFL text.

**SD.** The same picture. The numerals are ~half the pixel count, so the AA ramp is floored at 1 px a texel more often (very small numerals render a touch sharper). The worst SD line uses 380 of ~714 render clocks.
