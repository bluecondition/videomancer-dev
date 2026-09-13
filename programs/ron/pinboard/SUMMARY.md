# Pinboard

An isometric board of square pins with a ball hovering and skating over it. Pins
near the ball are agitated — they pop up out of the board and sink back at random
phases — so a shimmering patch of raised tiles chases the ball around. With
**Bulge** on, the ball also presses a stepped dome up out of the field.

Port of the Shadertoy raymarcher <https://www.shadertoy.com/view/tssSDN> — a grid
of boxes whose heights are `sin(hash(cell) + t)` attenuated by the distance to a
moving sphere.

## Status

**v0.1 — built and timing-closed on all 6 configs. Not simulated, not HW-tested.**

| Config | Fmax | Target | LCs | BRAM |
|---|---|---|---|---|
| HD Analog | 78.33 | 74.25 | 6614/7680 (86%) | 6/32 |
| SD Analog | 59.63 | 27 | 6620 | 6 |
| HD HDMI | 75.69 | 74.25 | 6616 | 6 |
| SD HDMI | 71.78 | 27 | 6614 | 6 |
| HD Dual | 75.61 | 74.25 | 6616 | 6 |
| SD Dual | 65.07 | 27 | 6620 | 6 |

HD Analog landed on the last retry seed (6), which `build_programs.sh` accepts
best-effort — it was re-run standalone and the **post-route** report confirms a
genuine 78.33 MHz PASS. The other five closed before the last seed.

## What changed from the shader, and why

The original is a 128-step sphere-traced SDF with a low perspective camera. On an
HX4K at pixel rate that is not on the table, so the renderer is an **orthographic
isometric heightfield march** (forked from `ziggurat` v0.4). Consequences:

- **No perspective and no horizon** — the board reads as a tabletop rather than a
  floor running away from the camera. This is the one real look difference.
- **The sphere is a screen-space disc**, shaded analytically rather than traced,
  and depth-sorted against the board with a single compare.
- **The wobble is a triangle wave, not a sine** — indistinguishable at this scale,
  and it costs a fold instead of a LUT.
- **The radial falloff is a 5-ring 2:1 ladder** rather than a smoothstep. It reads
  as a soft falloff; the rings are only really visible with Bulge on, where they
  are the point (a stepped dome).

## Architecture

Reformulated from ziggurat onto **remainders**, which is what makes it cheap.
Writing `i = iv+k, j = ju+k`, ziggurat's two cover half-planes both collapse to

```
ivrem + e >= 0   and   jurem + e >= 0,      e = aH - k*W2
```

so `U`, `V` and the `W2*i` products never have to exist. `ivrem`/`jurem` live in
`[0, W2)` and step by `b` per pixel, `a` per active line — no divide, no multiply.
Marching `k` from KF down to 0 and taking the first cover gives the nearest pin;
`k=0` always covers, so the screen is always filled and there is no background
case. For the winner, `p = ivrem + e` and `q = jurem + e` locate the pixel inside
the pin's silhouette: `p,q < W2` → TOP, else RIGHT if `p>q` else LEFT. Carrying
`p-W2`, `q-W2` and `p-q` out of that stage turns the whole face test into sign
bits.

**March depth is set by the tallest pin.** Cover at `k` needs `aH > (k-1)*W2`, and
one cell of pin height is `W2/2` — so a two-cell pin only ever reaches `k=1`.
`KF=1` is not a compromise, it is the exact depth the height budget can use.

**Height (H1..H8)** is a side pipeline fed only by the cell indices, so no hash
and no product ever lands on the march's critical path — `ivrem`/`jurem` are just
delayed to meet it.

```
aH = (A * tri(hash(i,j) + time) / 32 + B) >> gs
```

`tri` is a 5-bit triangle (a fold); `gs` is a 5-step radial gate (4 compares); `A`
and `B` are per-frame constants, so the "multiply" is a five-term add of shifted
copies of `A` — and since `A` is a register, every shifted copy is free wiring.
The gate is applied to the *finished height* rather than to the amplitude: same
number either way, but it keeps two wide barrel muxes per marched cell out of the
design and off what was the critical path. The KF+1 candidates are the diagonal
`iv+k, ju+k`, so their hash bases are constant increments of one `i*73` / `j*149`.

> The gate and the hash must both be evaluated **per k**. Sharing one across the
> marched cells is tempting (~400 LC) but wrong: a cell's height would then depend
> on which pixel was looking at it, and its top face would break up.

**Ball:** two squared-distance accumulators (`sqx += 2dx+1` per pixel,
`sqy += 2dy+1` per line) give `r2` with no per-pixel multiply. The shade is

```
sh = (R2 - r2)>>ks - (dx + dy)>>kd
```

whose level sets are exact circles offset up-left — so that single expression is
the dome, the light direction *and* (by threshold) a round specular. Occlusion is
one compare, `bi+bj >= i+j`, which lets a popped pin eclipse the ball.

**Per-frame maths** (K1 scaling, `a*b`, amplitude, orbit, radius, `cx²`, `cy²`)
runs on **one 16×16 shift-add sequential multiplier** driven by a vblank
sequencer — 11 ops × 18 clocks, one 18-bit add per cycle. A parallel multiplier
would have been timed at the pixel clock for no benefit.

**Interlace:** the line accumulator steps by `2a` and the bottom field seeds with
`+a`; the ball's vertical accumulator steps by 2 lines and seeds with `(cy-1)²`.

Latency 16 clocks. Chroma is authored in standard BT.601 and swapped once at the
output pins; outputs are forced neutral outside `avid`.

## Controls

- **K1 Pin Size** — cell size, ~24 to ~260 px across at HD, scaled by raster.
- **K2 Rate** — shimmer speed; also drives the ball's drift. At 0 the board
  freezes but the ball keeps a slow crawl.
- **K3 Pop** — how far agitated pins rise (0..2 cells; Bulge halves it).
- **K4 Reach** — radius of the ball's influence, 1..33 cells.
- **K5 Roam** — size of the ball's path (0..16 cells).
- **K6 Hue** — board hue on a 16-step wheel; sides fully saturated, tops tinted
  white. The ball takes the antipode, which on this wheel is just `1024 - u/v`.
- **T7 Edges** — face-seam outlines.
- **T8 Field** — Near (pins near the ball shimmer) / Far (inverted: the ball
  *calms* the board and everything else churns).
- **T9 View** — standard 2:1 iso / shallow.
- **T10 Video** — the live picture supplies the pin colour (sampled and held once
  per pin) while the pin keeps its face shading.
- **T11 Bulge** — the ball also presses a stepped dome up out of the board.
- **P12 Ball Size** — tiny dot to half the screen. The ball rises with its own
  radius so it always clears the board.

## Fit and timing journey

Started at **8061/7680 LC (104%, unplaceable)**, ended at 6614 (86%) and 78 MHz.

1. **March depth is the dominant area lever** — each `k` is a full copy of the
   height pipeline. KF=3 → 104%; KF=2 → 94% and 54 MHz; KF=1 → 86%. The height
   budget made KF=1 exact, so this cost nothing visually.
2. **Six separate `to_unsigned(f(C_HU(i)), 10)` expressions inferred six 16-entry
   ROMs.** Reading the wheel once into a variable and deriving the rest from it
   collapsed them to two.
3. **The critical path was 16.1 ns, of which 11.3 ns was routing** — at 94%
   utilisation the placer had nowhere to put things. Cutting area was worth far
   more than adding pipeline stages (cf. the ice40 note that extra registers can
   *regress* timing above ~90%). The path itself was the radial-gate compares
   feeding two 19-bit barrel muxes; moving the gate past the product deleted the
   muxes outright.
4. **Pre-bias the accumulator wrap constants.** `if ivrem + b >= W2 then ivrem + b
   - W2` is add → compare → subtract, three deep. With `b-W2` precomputed it is
   two parallel adds selected on a sign bit.
5. The remainder delay lines carry `[0, W2)` — 15 bits, not the 19-bit march
   width. The wide type was propagating through 8 stages × 2 signals.
6. Saturating the gate distance at 11 bits (the widest reach threshold is 1317)
   shortened four comparator carry chains per marched cell.
