# IRIS

A perspective dot tunnel drawn as an eye: a large iris circle nearly filling
the screen, a dark pupil anywhere inside it, and hundreds of round dots on
spiral arms sweeping from the pupil to the rim. Dots are born as pinpoints at
the pupil edge and grow geometrically outward. The field is stretched between
pupil and rim, so an off-centre pupil squeezes the rings on the near side and
fans them on the far side, like a cone seen in perspective. Three transparent
layers (red, green, blue) draw the same field; SPLIT fans them apart with
distance from the pupil.

Status: v0.7, hardware-iterating. v0.6 fixed the artifacts found on HD HDMI
(stray dots down the left edge, arcs at the corners, arcs displaced across the
picture) and sharpened the dots. v0.7 answers the second hardware session:
the pupil knobs stop short of the dark extremes, Pupil Size now darkens the
rings themselves instead of masking them with a black disc, the outermost ring
dissolves as Flow pushes it through the rim, the Full Bleed transition is a
constant-rate dolly. v0.9 cures a seam of coloured hash beside the pupil (a
sampler limit, faded out from the conformal scale). v1.0 answers the third
session: pure white dots at zero split, red and blue edges at green's 2 px
quality, and Switches 7 and 8 repurposed as Fill (Dots/Mesh) and Edge
(Hard/Glow), since the video path they were reserved for does not fit.

## Controls

| Control | Label | What it does |
| --- | --- | --- |
| Knob 1 | Pupil X | Horizontal pupil position; centre of the knob = centre of the iris. Travel stops at 60 % of the iris radius: past that the near side of the ring stack is squeezed into a few pixels and goes dark. |
| Knob 2 | Pupil Y | Vertical pupil position. Knobs 1+2 steer the gaze and tilt the tunnel. |
| Knob 3 | Pupil Size | Pinpoint (the spiral winds into a singularity) to about 60 % of the iris diameter. It does not mask the dots: it puts them out one RING at a time from the centre outward. Every dot in a ring carries the same level -- the distance is measured from the ring's own centre, not from the pixel -- and the fade spans two rings, so only one ring is ever in transition. Because it acts on the dots' coverage rather than on the finished picture, the pupil goes white in Ink exactly as it goes black in Light. |
| Knob 4 | Arms | Number of spiral arms = dots per ring, 6 to 96, always a whole number, with hysteresis so the count never flickers. |
| Knob 5 | Rings | Rings between the (reference-size) pupil and the rim, 3 to 48. Dot size follows the smaller of ring spacing and arm spacing. |
| Knob 6 | Twist | Bipolar with a centre dead zone. Centre = straight radial spokes; right winds clockwise, left counter-clockwise. The knob sets the arm pitch angle, so the spiral keeps its look when Arms or Rings change. |
| Slider | Split | Fans the R/G/B layers apart, growing from the pupil outward (red leads, blue trails, green anchors, plus a small radial offset). Square-law weighted so the lower half gives fine control over subtle fringing. Below 3 % the three layers are exact copies, so the dots are pure white (or pure black in Ink) with no colour at their edges. |
| Switch 7 | Fill: Dots / Mesh | Mesh grows every dot to 1.24x the radius at which neighbours touch, so they overlap and the black between them becomes the figure: a honeycomb of dark interstices in a bright field (bright holes in dark in Ink). With Twist off centre the strands show small notches where they cross ring boundaries, because each pixel tests only its own cell's disc. |
| Switch 8 | Edge: Hard / Glow | Glow makes the coverage ramp span the whole dot instead of one pixel of its edge: every dot becomes a soft ball, brightest at its centre and fading to nothing at its geometric edge, a phosphor-matrix look. Smaller dots are dimmer, as glows are. In Ink they are soft dark blots. |
| Switch 9 | Light / Ink | Light = additive R/G/B on black (the reference). Ink = the same field as cyan/magenta/yellow transparent dots on white paper, multiplying down to black. |
| Switch 10 | Still / Flow | Flow streams the dots outward from the pupil (new pinpoints emerge from the pupil edge) and, through Twist, spins the field. The outermost ring fades out AS A WHOLE -- one level for the entire ring, walked by the flow phase -- over the ~2.4 s it takes to drift out through the rim, to black in Light and to paper in Ink. Straight spokes = pure outward flight. Still freezes where it is; at phase zero the rim is crisp. |
| Switch 11 | Framed / Full Bleed | Framed = iris at 94 % of screen height on a black (or paper) backdrop. Full Bleed enlarges the iris past the screen corners and the pupil travel with it. The change is a dolly of equal ratios per frame, 16 frames end to end. |

All knobs and the slider are glided (about 8 frames, first-order) so movement
is fluid without feeling detached.

## Reference image

Preset **Reference** (the defaults): Pupil X 30 %, Pupil Y 55 %, Pupil Size
31 %, Arms 48, Rings 30, Twist +40 %, Split 40 %, all switches in their first
position (Light, Still, Framed).

Other presets: **Sunburst** (straight spokes, no split), **Misregister** (ink
on paper, big split), **Vortex** (full bleed, flowing, tight twist).

## How it works (short)

The pupil circle and the iris circle are two nested circles, which form a
coaxal pencil with two limiting points, a (inside the pupil) and b (outside
the iris). The Möbius map w = (z-a)/(z-b) sends both circles to circles
about the origin, so in the w-plane the dot field is a plain log-polar lattice
(geometric rings, A dots per ring, twist = per-ring rotation), and because a
Möbius map preserves circles, every ring and every dot is a true circle on
screen at any pupil position.

Because the iris is fixed and centred, the screen is normalised so that the
iris is the unit circle. The two limiting points are then inverse in that
circle, so the map is the disc automorphism w = (z-a)/(1 - conj(a) z) and the
far point never appears: the denominator stays bounded everywhere on screen.
That makes both of the engine's inputs, z-a and 1 - conj(a) z, plain affine
functions of the pixel coordinates, so each is a simple accumulator with
constant per-sample and per-line steps.

The log-polar coordinates are smooth, so a line-rate engine evaluates them
every 8 pixels, one line ahead of the beam: one serial vectoring CORDIC
(three pipeline stages, time-shared between the two lanes) gives both
magnitudes and both angles; a per-frame log table already scaled to ring
units turns the magnitudes into the ring coordinate, and two small per-frame
tables turn the angle into the arm coordinate. There is no multiplier in the
engine. The pixel path runs a DDA between samples and one dot machine
evaluates the three colour layers in turn (each layer every third pixel, held
for three), looking the dot shape up in two per-frame tables; the anti-alias
ramp is scaled by the local conformal factor so dot edges are equally soft
everywhere, and capped for sub-pixel dots.

A blanking-time micro-sequencer (a small accumulator CPU with its program and
registers in block RAM) solves for the limiting point in closed form and
rebuilds the six per-frame tables across the following blanking intervals.
The iris maps to |w| = 1 exactly and the pupil's image radius has a closed
form, so no reference value has to be measured.

### One dot machine, three layers

The dot machine evaluates one layer per pixel on a four-slot schedule --
green, red, green, blue -- so green is refreshed every second pixel and red
and blue every fourth. Two things follow, and both were visible on hardware.
With the split at zero the three layers draw the *same* dots but sample them
at different horizontal phases, so their edges land on different pixels and
the dots wear a thin coloured rim; the sequencer therefore raises a flag when
the glided slider is under 3 % and red and blue become exact copies of green.
And at any split the red and blue crescents used to be chopped into 4 px
treads, because a held value is a staircase. Each colour layer now keeps its
previous evaluation as well, and for the two pixels after a fresh one shows
the midpoint of the two before showing the new value: a 2 px cadence with a
half step between, the same quality as green's own hold. Green is aligned by
taking its own previous evaluation, and the sync tap moves by two pixels so
nothing on screen shifts; at the first evaluation of a line the "previous"
value is seeded fresh so the line above's tail never leaks in. Two other
formulations were built and rejected: a linear ramp between evaluations
(smooth, but ~100 cells the part does not have, and it smears a 3 px crescent
into a gradient), and evaluating red at 2 px on even lines and blue on odd
with a line buffer carrying the other layer across (sharper still, but on
dots this small every boundary is curved enough that the alternating fresh
and stale lines read as a comb along every crescent). Beyond 2 px there is no
path: it would take a second dot machine, ~300 cells.

### The arm coordinate across the angle's cut

The arm coordinate is 16 bits: 16 arms of Q12 fraction. Where the CORDIC's
angle wraps -- its cut, a curve running out of the pupil -- the arm
coordinate jumps by a full turn, which in 16-bit storage is A mod 16 arms.
That is a whole number of arms, so the dot test is right AT the samples; but
the pixel-path DDA interpolates between the two samples straddling the cut
and sweeps linearly through those arms in 8 px, planting one phantom dot per
arm along the cut: a curve of spaced small squares out of the pupil, coloured
because each layer lands on its own pixel phase, indifferent to Split and
Twist, following pupil position and size, and with a dot count that changes
with Arms. It never showed at the default 48 arms or at exactly 32 -- both
multiples of 16 -- and the sim's knob value gives exactly 32; a real pot a
count off gives 31 or 33. The engine now counts cut crossings along each line
(angle quadrant 3 to 0 forward, 0 to 3 back) and adds that many times A mod
16 to the arm coordinate, which keeps consecutive samples continuous and
changes nothing modulo one arm. A mod 16 comes from the sequencer each frame.
Confirmed gone on hardware (2026-09-20).

### The resolution fade

The engine samples the lattice coordinates every 8 pixels and the pixel path
interpolates between samples with a DDA, so there is a limit to how fine a
lattice it can represent. Near the pupil the log-polar lattice crowds without
bound, and past about one ring per 8 pixels the DDA is interpolating across
structure it never sampled: the dots land in the wrong places and the field
breaks up into a thin, broken, many-coloured seam running out of the pupil
along whichever direction is locally radial. It does not respond to any
control, because it is a property of the sampler, not of the picture.

The cure is to fade the lattice out where it stops being resolvable, so it
ends rather than degenerating. The engine already computes the local
conformal scale for the anti-alias ramp, and f_bar's shift pins that number
to log2(ring units per pixel) exactly, so the test is a comparison against a
constant: full strength out to one ring per 8 px (rd 9), gone by one ring per
4 px (rd 10). That span is exactly one octave and lands on a power of two, so
the ramp between them is a plain slice of a subtraction the engine already
performs, and it rides to the pixel path in 8 bits of the sample word that
were already allocated and zero-filled.

Because the Möbius map is conformal, that one number covers both axes: rings
and arms crowd together by the same factor, so the arms need no test of their
own. That is not a convenience but a requirement -- a difference of the arm
coordinate could not supply one, because it wraps every 16 arms and the wrap
reads as a full-scale change, which blacks out a radial spoke: a seam of
exactly the kind being cured. Two earlier attempts, one thresholding the
ring coordinate's horizontal difference and one adding the arm coordinate's,
are worth not repeating: the first misses the plume above and below the pupil
entirely (the rings crowd along the line of centres, but the arms converge on
the pupil in every direction), and the second draws the spoke. The isotropic
form is also the cheapest of the three and the only one that stays off the
pixel path's critical timing.

The ring lattice is anchored at a reference pupil of 6 % of the iris
diameter. Shrinking the pupil below that reveals more inner rings (down to a
singularity); dilating it eats the inner rings, so a wide-open pupil leaves
only the outer band of large dots.

## Deviations from the brief (what was traded, and why)

Worked down the priority list from the bottom, as asked:

- **Video halftone and video backdrop (priority 5) are dropped; switches 7
  and 8 now carry Fill and Edge instead.** The video path was re-attempted in v0.7 and measured
  rather than estimated. The cheapest form that keeps both -- the incoming
  luma attenuating dot coverage for the halftone, and a lighten/darken key
  (not a blend, which would need a multiply per channel per pixel) for the
  backdrop, reading the input live so that no delay line is needed -- costs
  205 logic cells. That is inside the 230 the device has free, but it is not
  inside the routing budget: the same netlist with the video in place routed
  at 61 to 73 MHz across six placement seeds, where the netlist without it
  routes at 70 to 83 and passes on half of them. At 97 % occupancy this
  design is congestion-bound, and 205 cells is worth about 6 MHz. Fitting it
  would mean giving up something else of comparable size -- the sub-pixel
  dot cap, the third colour layer, or the per-ring twist table.
- **Layers are evaluated at one third of the horizontal pixel rate
  (priority 3).** One dot machine serves red, green and blue in turn, so each
  layer's coverage is updated every third pixel and held. Vertical edges are
  exact; horizontal dot edges are quantised to 3 px. This was the cut that
  brought the design from 113 % of the device to something that fits.
- **Geometry is sampled every 8 px** and interpolated linearly in between.
  Within a couple of pixels of the pupil the log-polar field curves faster
  than that, so the innermost one or two rings of a pinpoint pupil can wobble
  slightly; everywhere else the error is far below a pixel.
- **The anti-alias ramp is normalised with about half a stop of error**,
  because the engine approximates the log of the conformal factor linearly in
  the mantissa while the sequencer uses the true log curve. Dot edges are
  therefore very slightly softer in some parts of the field than others.
- **Rings knob semantics.** "Rings between pupil and rim" is exact at the
  reference pupil size (6 %). At other pupil sizes the count of visible rings
  changes, because the brief also asks that dilating the pupil pushes inner
  rings out of existence and a pinpoint pupil winds into a singularity; both
  need the lattice anchored independently of the actual pupil size.
- **Pupil travel** is limited by the larger of the pupil and the reference
  pupil (6 %), and then to 65 % of what is left, which puts the maximum
  offset at about 0.6 of the iris radius. That is where the ring spacing on
  the near side falls to about 10 px; beyond it the near half of the picture
  goes dark, which is what the knobs used to run into at their ends.
- **Twist range.** The maximum useful per-ring rotation is half an arm
  spacing (beyond that the spiral aliases to the opposite hand), so the knob
  maps arm pitch angle and clamps there. Very tight whirlpools therefore need
  many rings relative to arms.
- **Split at maximum.** Three layers of kiss-sized dots cannot be fully
  separated on a lattice; at the top of the slider the layers are offset by
  more than a dot but partially overlap neighbours, giving the confetti of
  primaries with secondary slivers rather than clean isolated primaries.
- **Extreme corner.** With 48 rings and the pupil pushed to the very edge of
  its travel the ring density exceeds the table range (64 rings per octave)
  and saturates, so the outermost rings then sit slightly outside the rim.
- **The pupil ramp is a fixed two rings wide**, not a fraction of the ring
  count: making its width track the Rings knob would need a per-pixel barrel
  shifter. At 48 rings the pupil edge is therefore tight, and at 3 rings the
  ramp covers most of the stack.
- **Both fades are quantised to whole rings, on purpose.** The pupil's level
  is computed from the ring's centre rather than the pixel's own position, and
  the rim's is a single per-frame constant. A smooth spatial ramp was tried
  first and looked wrong: it shaded each ring across itself instead of putting
  rings out one at a time. The rim fade being phase-driven also matters for
  correctness, not just looks -- Flow carries the drawn band up to a ring past
  the rim, and it is the fade reaching full attenuation at the end of the
  phase cycle that stops dots being drawn outside the iris.
- **Green is refreshed twice as often as red and blue.** One dot machine
  serves all three layers, on a four-pixel G, R, G, B cycle: green carries
  most of the luminance and so most of the apparent sharpness, and gets a new
  value every second pixel; red and blue, which only carry the colour
  fringing, get one every fourth. Giving all three a value every pixel would
  need about 400 more logic cells than the device has free.
- **SD update rate.** The sequencer needs about 280 k clocks per pass and only
  runs during blanking. HD has enough blanking to finish in one frame; SD has
  about a fifth of that, so knob response at SD lags by several fields.
  Interlaced modes advance Flow on one field per frame.

## Tunables (top of the sequencer program, useq.py, and iris.vhd)

- `FLOW_RATE` (Q12 ring per frame; 28 is about 2.4 s per ring at 60 Hz).
- `FILL_Q16` (dot radius as a fraction of the kissing radius; 0.92).
- Layer offsets: red leads by +1 sep in angle and +1/2 sep in radius, blue
  trails by -1 sep and -1/4 sep (in `p_lat`, stage P7).

## Build notes

All six configurations meet timing (v1.1, seed base 9):

| Configuration | Required | Achieved | Seed |
| --- | --- | --- | --- |
| HD Analog | 74.25 MHz | 76.52 MHz | 9 |
| HD HDMI | 74.25 MHz | 78.11 MHz | 9 |
| HD Dual | 74.25 MHz | 75.95 MHz | 14 |
| SD Analog | 27 MHz | 73.14 MHz | 9 |
| SD HDMI | 27 MHz | 72.72 MHz | 10 |
| SD Dual | 27 MHz | 75.34 MHz | 9 |

HD Dual landed on the last retry of its seed run; 75.95 is a real pass but the
thinnest margin in the set. 7324-7349 cells, 20 block RAMs.

v1.0 as first flashed had a packing bug: the sample word was re-laid out for
the 6-bit fade but the pixel-side read of the anti-alias shift kept its old
bit positions, so the barrel saw a quarter of the true shift and every dot
went hard-edged (which reads as "sharper"), and the resolution fade, tuned
for anti-aliased dots, left hard 1-2 px dots aliasing beside the pupil as a
twist-dependent seam. Whenever the sample word, the tables or the barrel are
touched, check anti-aliasing DIRECTLY: the mean value of a dot's top/bottom
boundary pixel sits near half intensity with the 1 px ramp (~105-110) and
near 90 without it.

A twelve-seed sweep of HD Analog at this netlist passes 6 of 12 (best
87.97 MHz), the healthiest distribution the design has had. It got there by
paying for the layer smoothing with cells that were doing nothing: the DDA
accumulators carried 26 and 22 bits when only u's 20 and v's low 12 ever
reach the dot test -- the top bits fed only themselves, which synthesis cannot
prove dead -- and the sub-pixel cap had been dead logic since the resolution
fade landed. 7224-7264 cells, 20 block RAMs.

**Read the routed number, never the build script's checkmark.** On this design
the script's six seeds are close to a coin flip for the two hard HD
configurations, and when it runs out of retries it accepts the last attempt and
prints a green tick: HD Dual "completed" at 74.13 MHz, which is a MISS of the
74.25 requirement. HD HDMI "passed" at 74.28 MHz, inside the router's own
run-to-run spread. Both were replaced by sweeping placements against the
already-synthesised netlist (cheap: placement only) and rebuilding that one
configuration against the winning seed. Sixteen placements of HD Dual produced
four passes (68.0 to 79.6 MHz); seven of HD HDMI produced three (69.6 to 78.7).

Two mechanical traps in that workflow, both of which silently produce a
plausible-looking wrong answer:

- The placement estimate printed BEFORE routing is not the result, and on this
  design it is not even correlated with it -- HD Dual seeds 17 to 21 all
  estimated 76 to 81 MHz and every one of them routed at 68 to 72. Take the
  LAST "Max frequency" line, and only once the run has finished.
- Deleting the `.bin` does not force a rebuild. The bitstream is two derived
  steps from the netlist, so make reuses an existing `.asc` and merely re-runs
  `icepack`, yielding a fresh-looking file from the OLD seed. Delete the `.asc`
  too.

The resolution fade cost about 65 logic cells, which at this utilisation was
enough to close HD Analog out entirely: a twelve-seed sweep of the first
version failed on all twelve (67.9 to 73.6 MHz) with a critical path of 3.1 ns
logic against 10.5 ns routing -- congestion, where only removing logic helps.
Two things recovered it. Deriving the anti-alias shift's rounding from a
10-bit increment off the existing subtraction instead of a second 22-bit add
lifted the sweep to five passes in twelve. Computing the fade in the engine
rather than the pixel path kept it off the critical path altogether: the two
earlier per-axis formulations, which needed two 21-bit absolute values and a
compare in the pixel path, ran 165 cells heavier and routed at 55 to 60 MHz.
A cheaper formulation beat every placement-level remedy.

The design occupies about 95 % of the HX4K's logic cells (7256-7269 of 7680)
and 20 of its 32 block RAMs. At that utilisation place-and-route is routing-bound, so the two
things that actually bought speed were removing logic and freeing block RAMs.
The largest single win by far was sizing the microcode ROM to 1280 words
rather than 2048: that is three fewer block RAMs and it moved the routed
result from 62 to 74 MHz at the same seed. Adding pipeline stages made
timing worse, which is expected above about 90 % utilisation.

If more margin is ever wanted, trimming the sequencer program below 1024
words would free one more block RAM at no cost to the picture.
