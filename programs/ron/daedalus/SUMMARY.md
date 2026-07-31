# Daedalus — the picture drawn as a labyrinth

Forked from **VideoSky**, and the fork turns on one observation about its
accumulator.

VideoSky's rules are the **level sets** of a rotatable coordinate field: `P`
steps by `dpx` per pixel and `dpy` per line, and ink lands wherever
`phase < thickness`. Level sets can never break, which is exactly why VideoSky's
Relief can bow them around the image. A maze is not a level set — it is a
**tiling** — so Daedalus keeps the coordinate frame and throws the level set
away.

## The three ideas

**The lattice was already in the accumulator.** `acc` is 24 bits with 8
fractional bits, so `acc(15 downto 8)` is the phase within one period — the
**sub-cell coordinate** — and `acc(23 downto 16)` is the period index — the
**cell id**. VideoSky only ever read the phase byte and discarded the rest.
Reading both bytes turns the same two adds per pixel into a full cell grid that
rotates with Angle and scales with Pitch for no extra cost. `P` indexes columns,
`Q` indexes rows.

**One continuous line — the Truchet weave.** Two tiles, each joining all four
edge-midpoint ports **in pairs**: N↔W + S↔E, or its mirror N↔E + S↔W. Because
every tile uses every port, each port is always live and always has exactly one
path arriving from each side. **Degree 2 everywhere; branching is impossible.**
The ink is therefore a set of unbroken lines that snake around each other at
right angles and fill the plane with no gaps and no sparse patches — a uniform
texture across the entire frame.

**The paths must be staircases.** With ports at the edge midpoints, a
single-turn L from N to W has to turn either at the cell **centre**, where it
collides with the S↔E path, or at the cell **corner**, where the paths of four
cells collide. Physical Truchet tiles dodge this with quarter-circle arcs that
bulge toward the corner without touching it, and there is no right-angle
equivalent of that arc. So:

```
N<->W : (128,0)->(128,64)->(64,64)->(64,128)->(0,128)
S<->E : (128,255)->(128,192)->(192,192)->(192,128)->(255,128)
```

Each hugs a corner diagonal while reaching neither the corner nor the centre, so
the two never touch. **Tile 1 is tile 0 mirrored in `u`**, so the second tile
costs one subtract and a mux rather than a second set of segment tests — and the
eight segment tests never need to know which tile they are drawing.

**Meander cells (v0.4).** The cells are **4× wider than tall**: Q (the column
axis) steps at a quarter of P's rate. Every tile stroke running along the wide
axis is then four times longer in pixels than a vertical jog, so the same two
staircases and the crossing render as *long horizontal runs joined by short
90° drops* — a greek-key meander locked to a grid, not a diagonal-reading
texture. Two consequences handled explicitly: line thickness is measured in
sub-cell units, so vertical strokes take a quarter width (`(w+3)/4`) to come out
the same pixel thickness as horizontal ones (each stroke family also gets its
own crossing gap); and Relief's displacement gets the same /4 on the Q axis so
the warp stays isotropic on screen. Angle rotates the whole anisotropic grid,
so 90° gives the vertical-dominant version.

**Under and over — the crossing tile (v0.3).** A third tile joins the ports
straight through, N↔S and W↔E, one bar crossing the other. The over bar is
unbroken; the under bar stops `2w+4` sub-cell units short of the over bar's
centre line on each side and comes back out — the line visibly ducks under
itself and re-emerges. Which bar is on top alternates checkerboard-fashion with
cell parity (`cx(0) xor cy(0)`), so a straight run through several crossings
goes over, under, over — a woven basket. The cross pairs all four ports like
every other tile, so the degree-2 / one-continuous-line guarantee survives.
Selection reads the hash's next 6-bit field against **half** the Tangle
threshold (the v0.4 density dial-back), so Tangle raises mirrors and crossings
together: at max ~24% of cells are crossings and the rest split between the two
staircases.

**Fractal octaves (v0.3).** An octave of the lattice is a **bit-shift of the
same warped coordinate**: shift the 16-bit integer part left by `k` and the top
byte is the cell id of a lattice 2^k finer, the bottom byte its sub-cell
coordinate already rescaled to 0..255. Up to 3 nested labyrinths (4 in v0.3;
the 16 px octave was the densest and its crossing gaps sat at the aliasing
floor, so v0.4 dropped it) share one
accumulator, one warp, and one Relief (the warp sits *before* the octave split,
so all octaves drape as one surface); each octave pays only its own hash,
mirror, and segment compares. Octaves OR together — fine ink inside coarse ink
is invisible, so the finer weave automatically fills the *gaps* between the big
lines. Octave k fades in via a per-frame width-cap ramp (`(6144 − step·2^k)/64`,
clamped to C_WMAX — no pop) only while its cells are ≥ ~11 px, so at the default
pitch there is one labyrinth, and as Pitch opens out toward big spread-apart
lines, successively finer labyrinths appear between them with proportionally
finer weight (base width doubles per octave in sub-cell units, which halves per
octave in pixels). At minimum pitch all three octaves are fully present: 128,
64, and 32 px cell heights (4× that in width).

**Relief warps the domain, not the pattern (P12, the headline).** VideoSky adds
luma to the phase byte at R5, at the very end. Do that here and each tile
displaces independently and the field turns to gravel. Daedalus adds `rel` to the
**full 16-bit integer coordinate before the split into cell id and sub-cell**, so
the cell boundaries bow along with the strokes and adjacent cells never disagree
about where their shared edge is.

The failure modes are worth stating plainly:

- luma **smooth** → the labyrinth drapes;
- luma **folds** the coordinate → a mirrored crease, still continuous, looks good;
- luma **discontinuous** (a hard edge) → it genuinely tears, and no arithmetic
  fixes that without a line buffer. Hard edges are where the blackwork masses go
  anyway.

Top of the slider folds everywhere at once — the designed climax.

## Rejected: the edge-hashed port network (v0.1)

v0.1 hashed each cell **boundary** for liveness and drew an arm from the cell
centre to each live port. Connectivity was structural and correct — both cells
sharing an edge derived the identical bit — and it looked good. But independent
ports mean cells land at degree 0–4: T-junctions at 3, crosses at 4, dead-end
stubs at 1, blanks at 0. **That is a network, not a labyrinth.** Nothing about it
reads as one continuous line snaking around itself, and the blanks and stubs made
the texture patchy rather than uniform.

The Truchet weave is also *cheaper*: one hash per cell instead of four per edge,
which more than pays for the extra segments (3417 LCs vs 3508).

## Tone

Two paths, and one that was designed out:

- **Blackout** (S7) — above tone 224, solid ink, no maze. This is what makes it
  read as blackwork rather than as a texture filter, and with Width=Fixed it is
  the *only* tone path: a uniform-weight labyrinth over the whole frame with all
  the tone in the blacked-in masses, which is exactly what the reference tattoo
  does. It needs no knob of its own — K1 Ink slides the whole ramp into and out
  of the threshold.
- **Line weight** by luma (S10 = Modulated) — the shadows fatten the line
  instead. Off by default, because varying weight is precisely what stops the
  pattern reading as *consistent across the entire frame*.
- **Cell size by luma — designed out.** Pitch is a per-frame constant feeding the
  accumulator; varying it per pixel destroys the lattice. It turns out not to
  matter, because Relief compresses and expands cells wherever luma has gradient,
  so "finer in the darks" falls out of the warp for free. (A legitimate version
  exists — subdivide dark cells 2×2 by shifting *which* accumulator bits are the
  cell id, a pure mux — but it tears along the subdivision contour. Deferred.)

Half-width is capped at 26 (`C_WMAX`): parallel staircase runs sit 64 apart in
sub-cell units, so anything near 32 merges neighbouring runs into mush.

## The hash

Two rounds of shift-and-add over `cx & cy` (16 bits), split across R6/R7 so the
mix chain never lands on one critical path. Every step is a **bijection** on 16
bits (odd multiplies `*9`, `*33`, `*5`; xor-with-own-high-bits), so distinct cells
never collide and the top bits are the well-mixed ones — which is what the Tangle
threshold reads.

Shift-and-**add**, not shift-and-xor: xor alone is linear over GF(2) and leaves
visible lattice structure (the starfield hash-bit-bias lesson). The carries are
the nonlinearity.

**K5 Tangle** thresholds two independent 6-bit fields of the same hash: bits
15..10 pick the mirror, bits 9..4 pick the crossing. At 0 every cell takes tile
0 and the staircases chain into long ordered diagonal runs. At max ~48% of cells
are over/under crossings and the rest a coin flip of the two staircases —
maximally knotted. (For the mirror bit, past 1/2 the pattern is merely the
mirror of below it, so the knob stops at 1/2 and every position does something.)

## Controls

K1 Ink (ramp bias, ±256; doubles as blackout depth) · K2 Contrast (1.0×–4.9×) ·
K3 Pitch (cell size, linear in *frequency*: ~128 px → ~4 px) · K4 Angle
(continuous 0–360°) · K5 Tangle (ordered runs → knotted labyrinth) · K6 Scheme
(Parchment / Gallery / Night / Blueprint / Cyanotype / Copper, reused from
VideoSky verbatim) · S7 Blackout · S8 Invert · S9 Tint (ink takes the source's
own chroma) · S10 Width (Fixed / Modulated) · S11 Drift · **P12 Relief**.

VideoSky's Waver and Cross are gone. Cross is meaningless when the field is
already 2D; Waver would displace one axis *after* tiling, which is precisely the
tearing mistake.

## Architecture

10-stage streaming pipeline, `C_LATENCY = 10`. **Zero BRAM**, ~4350 LC (57%), and
**two** pixel-rate multiplies (contrast, relief) — one fewer than VideoSky, since
the maze needs no sine tap and a single `rel` drives both axes. The four octaves
replicate only R6–R9 (hash, tile bits, mirror/measure, segment tests); the whole
front end — accumulators, contrast, tone, warp — is shared.

The per-frame angle resolve shares ONE registered 9×8 multiplier, split-operand
across vblank cycles, so no wide combinational product lands on a vsync-latched
register (the ziffern trap: those paths are still STA-timed). Interlace detected
by field-parity toggle; the odd field starts half a line step down. Accumulators
advance on **active lines only** — a blanking-hsync advance makes the phase
field-dependent and the lattice twitters at 30 Hz.

**Gotcha hit during the rewrite:** `'0' & r6_t(7 downto 3)` returns an
**ascending** range (`0 to 5`), so passing it into a function that slices
`v(4 downto 0)` fails with "slice direction doesn't match index direction".
Concatenation always yields an ascending range regardless of its operands; it
only looks fine in VideoSky because every such expression there is immediately
*assigned* to a descending signal, which reindexes by position.

## Status

v0.4: all 6 configs routed on **seed 1**, no retries:

| Config | Fmax | LCs |
|---|---|---|
| HD Analog | 91.10 MHz | 4125 |
| SD Analog | 84.29 MHz | 4133 |
| HD HDMI | 84.58 MHz | 4121 |
| SD HDMI | 88.37 MHz | 4136 |
| HD Dual | 93.81 MHz | 4136 |
| SD Dual | 88.37 MHz | 4145 |

Full clock everywhere (74.25 MHz required for HD). Packaged.

Version history against user feedback: v0.2 Truchet weave HW-viewed = "OK";
v0.3 added fractal octaves + under/over crossings, HW verdict = "too
intricate/dense"; v0.4 = the dial-back (3 octaves, half crossing rate) + the
4:1 meander cells. **v0.4's render is not yet sim-verified.** Open risks: hash
quality (two rounds on 16 bits is thin; bias shows as periodic structure in the
tile field); vertical-stroke quarter-widths at low w (during octave fade-in
`(w+3)/4` rounds to 1 — hairline but present); and whether 512 px-wide base
cells at minimum Pitch read as intended or as too-sparse giant runs.
