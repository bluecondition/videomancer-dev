# Hardline: the brief (started 2026-11-15)

A set of black-and-white programs in the family of **Fringe, Hypnos and Cascade**: one geometric idea per
program, drawn full-screen in crisp black and white, and played by hand. The slate and each program's status
are in [README.md](README.md); concept sketches are in [concepts/](concepts/).

Flash from `out/rev_b/ron/hardline/<name>.vmprog`; rebuild one with `./build_set.sh hardline <name>`.

## What the three parents have in common

| | Fringe | Hypnos | Cascade |
|---|---|---|---|
| Picture | two line textures, boolean blend | op-art rings, four shapes, spiral | nested rings that push each other |
| Tone | binary ink, no grey | binary ink + 1-px anti-aliasing | binary, optional stepped / smooth ramp per ring |
| Motion | none: all performed | one bipolar Speed slider, centre = still | none: all performed (Ron's rule) |
| X / Y | phase per layer | Center X / Y on K1 / K2 | Center X / Y on K1 / K2 |
| Glide | S7 | S7 | S7 |

So a Hardline program is:

1. **One geometric idea**, filling the screen, with no layers, overlays or textures on top of it.
2. **Black and white.** Chroma is a constant 512 on every pixel. Grey appears only as edge anti-aliasing, as a
   flat mid-grey where detail is finer than a pixel, and in the Shade option on some programs (below).
3. **Performed, not animated.** A picture with no hands on it and no camera is still. Every movement comes
   from a knob, the slider, a modulator or the camera. Where a program wants travel, it is a bipolar control
   with a dead centre, as Hypnos Speed is.
4. **Razor edges at any scale**: every edge anti-aliased over one pixel, nothing stair-stepped, no sparkle.

## The control scheme (same on all ten)

| Control | Function |
|---|---|
| K1 X | horizontal position of the figure's centre, origin or pan; reaches off-screen where the figure has a centre |
| K2 Y | vertical position, same range |
| K3 Period | line spacing / element size, exponential |
| K4, K5 | the program's own two shape controls |
| K6 Camera | how hard the incoming video bends the linework; **default 100 %**; 0 % = the clean generated figure |
| S7 Glide | Off / On, default On. On: X, Y, Period and the slider ease. **Off = zero easing**, values used the same frame |
| S8, S9 | the program's own look-flips |
| S10 Weight | Bold / Fine: equal bands, or thin lines on an open ground (as Fringe S10) |
| S11 Source | Video / Osc (default Video): the camera, or a slow internal field in its place |
| P12 | the program's headline gesture; 100 % is a designed climax |

Notes on the scheme:

- K3 Period and S10 Weight are on every program, but need not do rigidly the same thing (Ron, 2026-11-15): a
  program whose geometry wants a different reading of them says so in its SUMMARY.
- Glide follows the Fringe / Hypnos v1.1.0 model: the Off branch still tracks the accumulator so switching
  back On glides from where the knob is. Pot jitter is handled with a ±1-count hold, never easing.
- Every control visibly changes the picture at any setting of the others.

## Shade (scallop, blaze, flute, tangle)

Ron allowed gradients on some programs, stepped or smooth as on Cascade, both on a switch. On these four,
S9 is **Shade: Flat / Shade**. Shade replaces the two-tone band with a ramp across each band (a sawtooth for
shells and pleats, a triangle for pipes and glass rods). With Shade on, S10 picks the ramp: **Bold = 4 flat
steps** (black, dark grey, light grey, white), **Fine = a smooth ramp**. With Shade off, S10 is the usual
Bold / Fine line weight, so both switches change the picture in every position.

## The camera (K6)

The camera never appears as a picture. Its luma is box-filtered (at least 8 px, so the bend is smooth and the
lines stay clean) and added to the figure's **phase or coordinates**, so the lines swerve around the image the
way engraving lines follow a form. Each program names what the luma bends (ring phase, stripe phase, thread
position, scale radius, tile turn). Black input = zero bend = the clean figure, so no program is dead with
nothing plugged in. S11 Osc replaces the camera with an internal slow field at the same point.

Strength curve: 0 % none, 25 % a gentle swerve that keeps the figure whole, 100 % (default) a bold bend in
which the image reads as distorted linework but edges stay crisp. The sketches in `concepts/sheet_camera.png`
are near the top of that range and show that the bend strength needs tuning per program: a coordinate bend on
a small motif (Tweed) shreds sooner than a phase bend on rings.

## Rendering and hardware rules (from the parents and CLAUDE.md)

- `program_type = "processing"` (the camera is read). Blanking gate to Y 64, U = V = 512.
- Constant neutral chroma also makes the set immune to the U / V pairing fault class.
- Ink white about 775 after conditioning, ground black 64. Build the HD HDMI output conditioning in from the
  start ([1 2 1] on luma, luma scaled toward black): these are exactly the pictures that bar (sharp,
  full-swing edges in motion).
- Anti-aliasing from the phase gradient (Hypnos's analytic 1-px box filter). Where the local period drops
  under about 2 px the output goes to flat mid-grey instead of aliasing, unless the program offers aliasing
  as a look on a switch (Fresnel).
- Geometry in virtual 1080-line coordinates so SD, 720p and 1080p show the same picture.
- One latency in every mode; engines started per field; no per-pixel multipliers where a DDA will do.

## Chassis

- **Polar chassis = Hypnos v1.1.0** (hardware-validated): log-domain polar engine, analytic anti-aliasing,
  centre glide, S7. Used by blaze, multipole, fault, flute, kinegram, with the camera path grafted on.
- **Shell chassis = Kinetic riley** (the heirloom shell with sharp luma conditioning; not yet flashed): timing,
  virtual coordinates, per-field sequencer, source field, output. Used by fresnel, dazzle, tweed, scallop,
  tangle, with the wave engine, palettes and chroma paths removed.
- Fringe v1.1.0 is the reference for one-phase-one-compare textures on per-axis DDAs.

All ten were launched together on 2026-11-15 at Ron's word, without a pilot flash. If a chassis fault shows on
hardware, the fix goes into every program on that chassis.

## Build pipeline

Agent brief `.hardline_work/agent_prompt.txt`, per-program specs `.hardline_work/prompts/<name>.txt`, work
dirs `.hardline_work/rb_<name>/`. Tools are the Kinetic set tools with the set name `hardline`:
`.kinetic_work/tools/{check,sim,build_queue,accept}.sh hardline <name>`.
