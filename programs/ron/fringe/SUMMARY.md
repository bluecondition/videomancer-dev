# FRINGE

**A moiré interference instrument.**  Two independent layers of programmatic
black-and-white line-work are combined by a **boolean blend** into a single
mask and painted against a solid ground; when their pitches, forms or phases
approach one another, large-scale interference fringes bloom across the frame.
No video in — a pure generator (`program_type = "synthesis"`).

**Nothing animates by itself.**  When no control is moving and the glide has
settled, the image is completely static.  Every movement you see was performed.

## Controls

| ctl | name | what it does | |
|---|---|---|---|
| **P12** | **Y Phase B** | Y phase of layer B — the primary gesture, rakes one pattern across the other | **glided** |
| K1 | **Texture A** | 10 textures, equal zones | snap, 8-count deadband |
| K2 | **Scale A** | pattern pitch, 1024 px → 4.0 px | **continuous**, glided |
| K3 | **Phase A** | X phase, ±1024 px (rotation for spokes) | **continuous**, glided |
| K4 | **Texture B** | 10 textures | snap |
| K5 | **Scale B** | pattern pitch | **continuous**, glided |
| K6 | **Phase B** | X phase | **continuous**, glided |
| S7 | **Meet** | with S8, a 4-way blend of the two masks | |
| S8 | **Fringe** | | |
| S9 | **Weight** | Bold (50 % ink) / Fine (half of everything) | |
| S10 | **Ground** | white / black — inverts the composite | |
| S11 | **Colour** | mono / ink coloured by which layer made it | |

### Blend (S7 + S8)

| S7 | S8 | mode | mask | what you see |
|---|---|---|---|---|
| off | off | **Over** | A ∨ B | union — two transparencies stacked |
| on | off | **Meet** | A ∧ B | intersection — sparse jewels at the beats |
| off | on | **Fringe** | A ⊕ B | pure interference — the classic beat field |
| on | on | **Cut** | A ∧ ¬B | A carved by B |

With S10 polarity on top, that reaches NOR, NAND, XNOR and ¬A ∧ B by De
Morgan — ten of the sixteen boolean functions of two masks.

The slider needs no target switch: only the **relative** phase of the two
layers makes fringes, so driving layer B's Y phase alone reaches every
interference state.  The only thing given up is a global vertical shift of the
whole composite.

**Textures**, radial → linear → tiled: Circles · Squares · Spokes · Wavy H ·
Wavy V · Diagonal · Grid · Hex · Checker · Brick.

**The colouriser (S11)** paints ink by *which layer made it*, so the
interference itself carries the colour.  On a white ground that is subtractive
process ink — cyan over magenta going deep blue where they meet, a two-colour
print rosette.  On a black ground it flips to additive light: red plus green
going yellow.  It stays orthogonal to the blend: the blend sets the shape, the
palette sets the hue.  Palettes are authored in standard BT.601 and swapped at
the output pin for the hardware's convention, so sim previews show swapped
hues — judge shape in sim, hue on the device.

**What is deliberately absent: per-layer mask invert.**  For a 50 %-duty
grating, inverting the mask is mathematically identical to shifting the phase
by half a period — which the phase knob already does, more finely.  It only
differed on the three thin-line textures (grid, hex, brick), which is not worth
a switch; Cut covers the useful negations and Ground covers global inversion.
That, plus the fact that black ink on a black ground draws nothing, is why the
original invert/order/background trio felt dead.

Presets: Rings, Riley Weave, Screen Clash, Two Colour, Light Mix, Carved Tile,
Slow Sweep.  **Power-up** is white ground, mono, Over blend, Bold weight, and
*two* circle fields at 64 px and 46.5 px — so the unit wakes up already showing
a strong concentric moiré.  If that is on screen, every part of the engine is
working.

## One phase, one comparison

Every texture reduces to a 10-bit **phase** — 1024 units = one pattern period —
and a comparison.  There are two roads to it, and neither costs what it looks
like it should:

- **Cartesian textures ride DDA accumulators.**  20 bits per axis per layer,
  carrying `(x − cx − offx)·step` in Q8 modulo 2²⁰.  No pixel multiplier at
  all.  The trick that makes them free: the DDAs are clocked by the **avid
  delayed to exactly the stage that consumes them**, so they present the right
  pixel to the window stage without a single delay register of their own.
- **Radial textures ride a CORDIC.**  8 vectoring iterations per layer give r
  and θ; **one** 13×16 multiplier per layer then turns either `r·step_r`
  (circles) or `(θ+rot)·blades` (spokes) into the same 10-bit phase.  The
  CORDIC's 1.6468 gain is undone inside the per-frame constant `step_r`, never
  in the pixel path.

Because the DDAs wrap they carry no sign — **and none is needed**.  Concentric
squares want `max(|px|,|py|)`, and since both axes are scaled by the same
positive step, the CORDIC's own `|dx|` vs `|dy|` comparison has already picked
the winner.  Winner and the two signs ride down as three bits, and the
magnitude fold becomes a ten-bit complement plus one borrow.  That removes
every wide absolute value from the pixel path — worth ~240 LCs.

**Scale** is a mantissa/exponent split of the glided knob: `step = (256+2m) << e`
in Q8 phase units per pixel.  One sweep spans a period of 1024 px down to
4.0 px, continuously and monotonically, at constant octaves per degree of
rotation with no dead zone at either end.

**Honeycomb** is the hex Voronoi of a half-offset (brick) point lattice, so
both edge families fall out of linear bisector tests — `|u| = 512` for the
vertical edges, `|u| + 2|v| = 1280` for the slanted ones.  No squares, no
divides, no cell rounding.  Cells land 1024 × 1280 phase units, within 8 % of
regular.

## Everything per-frame goes through one shift-add engine

A fat combinational cone in per-frame logic caps Fmax exactly like a pixel-path
one does, so **nothing** here is left as a cone.  A single 12-iteration
shift-add engine sequences all eight products a frame needs — the 0.7071
diagonal fold ×2, the 0.6094 CORDIC-gain fold ×2, and the four DDA start
values — in ~110 cycles inside a ≥17000-cycle vertical blank.  The rest of the
frame terms come off a 6-step ladder at one adder level per cycle.

That engine is also the fit story.  The first build came in at **104 % LC**.
Three changes took it to 92 %: twelve 24-bit constant-fold registers collapsed
into the shift-add engine (~350 LCs), the 30-bit absolute-value registers gave
way to the sign/winner-bit trick above (~240 LCs), and the DDAs narrowed from
30 bits to 20 once it was clear nothing downstream needed the un-wrapped
magnitude (~150 LCs).

## Architecture

Streaming, **C_LATENCY = 16**, **no BRAM**, two pixel-path multipliers (one per
layer, shared between the circle and spoke roads) and one vblank shift-add
engine.  Everything else is adders, shifts, compares and slices.

```
acc         dx/dy per layer (pixel domain)      DDA px/py (phase domain,
c1/c2       abs -> max/min + octant flags        clocked at avid tap 10)
c3..c7      CORDIC, 2 iterations per stage
c8          gain strip + octant fold -> r, theta
c9          operand select (r | theta+rot)
c10/c11     two partial products -> 10-bit radial phase
c12         phase windows, sine table, diagonal fold
c13         stripe phase mux + hex/brick cell space
c14         all ten ink rules -> one mask per layer
c15         boolean blend -> source palette -> Y/U/V
```

## Judgment calls

- **Duty cycles.**  The six grating textures run 50 % ink — classic moiré
  gratings, and the ratio that beats hardest against a second layer.  The tiled
  ones use line weights instead, tuned so cells read as cells: grid 18.75 % per
  axis, honeycomb ≈16 %, brick mortar 12.5 %.  Checker is a true 50/50 fill.
  **Weight** (S9) halves every one of those, which is also what turns the
  checkerboard into a windowpane — one threshold drives both.
- **Spokes phase mapping.**  The centre stays put and the phases rotate: X winds
  the fan a full turn, the slider counter-winds it a half turn.  Radial
  breathing was the alternative; counter-rotation won because it keeps the
  slider doing what it does everywhere else — sliding the front pattern against
  the back one — so the same gesture rakes fringes whatever is loaded.  Blade
  count (4–63) comes off the scale knob, so that knob *steps* on this texture
  alone; a blade count is an integer.
- **Alias clamp.**  4.0 px is the deliberate fine limit: 2 px ink, 2 px gap, still
  fully resolvable.  The interference at that end comes from the two layers
  beating against each other, not against the raster.
- **No anti-aliasing.**  Ink is binary.  Coverage estimation would need a
  reciprocal of the per-pixel phase step per layer, and the result would then
  have to survive a binary-transparency composite of two layers under two
  invert modes.  Grey fringing at every edge is a worse failure than a hard
  edge, so edges are hard and the fine clamp does the alias control.
- **Glide.**  One pole per frame at shift 4 — a ≈16-frame constant, ~0.27 s to
  63 %.  Fast enough to feel connected, slow enough that a flick reads as a
  swing.  Scale, X phase and Y phase glide; texture select and all five
  switches snap, including the P12 re-target when draw order flips.
- **Nothing trimmed.**  All ten textures shipped.
- **Switch rework (v1.1).**  The original Invert A / Invert B / Draw Order trio
  was replaced after it proved dead in use: invert is a half-period phase shift
  on seven of the ten textures, draw order is meaningless once the layers
  combine by boolean logic, and the background switch could only ever produce
  black-on-black.  Blend + Weight + Ground + Colour replaced them.

## Verification

**Sim-verified pixel-exact against the model.**  `fringe_model.py` is a
bit-accurate NumPy replica of the same integer pipeline.  Eight GHDL renders —
covering **all ten textures**, both invert modes, both backgrounds, both draw
orders, and a non-zero phase/slider frame — each match it **0 pixels different
out of 129600**.  That pins the DDA/CORDIC alignment, the shared multiplier,
the phase windows, every ink rule and the composite.

The phase test also nails the glide: the frame matched the model exactly at
**three** one-pole steps, which is what a four-frame sim should produce given
that the first vsync still glides toward the previous target.

Two real bugs fell out of it, neither of which a build catches:

1. `s_mbit` was declared `range 0 to 12` while the shift-add loop counts to 13.
2. `resize(-v_u, 10)` on a 12-bit signed folded the honeycomb's `|u| = 512`
   into 256 — a wrong-ink line through the cell edge once per period, wherever
   the phase landed exactly on zero.  Negate at full width and slice.

Worth recording for next time: two of the first test frames were **degenerate**
— black ink over a black ground renders nothing, so the layer under test was
invisible and matched the model trivially.  A composite test is only a test if
both layers are actually on screen; check the white fraction before believing
a 0 % mismatch.

## Timing (HX4K, no DSP)

All 6 configs pass, seed 1–2: HD **77.98 / 77.95 / 75.57**, SD
**74.87 / 76.99 / 73.10** MHz.  **7243–7274 / 7680 LCs (94.7 %)**, 0 EBR, no
PLL on HD analog or dual.  Not yet hardware-tested.

The v1.1 switch rework cost ~190 LCs (92 % → 94.7 %): the ink thresholds became
per-frame signals rather than constants, so five comparators per layer widened,
and the palette added three colour registers plus a chroma output path.

Read those HD numbers as **run-to-run** figures, not a property of the design.
An earlier build of nearly the same netlist landed at 74.64 / 74.56 / 80.79 —
router2 is nondeterministic and this design is **congestion-bound, not
depth-bound**: the critical path on the thin run measured **1.65 ns logic,
11.75 ns routing**.  So the lever for margin here is fewer cells, never more
pipeline stages — adding stages at 92 % fill regresses it, per the HX4K
high-utilisation rule.  The two CORDICs (~1200 LCs together) are the only block
big enough to matter, and cutting them below 8 iterations puts a visible wobble
on the spoke edges (~15 px at the frame edge for 6 iterations).
