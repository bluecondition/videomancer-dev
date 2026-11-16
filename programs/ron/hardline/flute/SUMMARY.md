# Flute (Hardline)

v0.1.0 built 2026-11-15 (router1, seed 1 on all six configs), not flashed.

## What it draws

Reeded (fluted) architectural glass in front of a ringed or striped scene; the same optics as a lenticular
sheet. The screen is cut into parallel flutes. Each flute is a cylinder lens: with t the offset from the
flute's centre line along the flute normal n, the scene is sampled at

    p' = p + (Power - 1) t n

so Power 1 is flat glass (the plain scene with thin grooves over it), Power > 1 shows a wider slice of the
scene squeezed into every flute (repeating compressed copies), Power 0 gives every flute one tone across its
width (bars), and Power < 0 flips the copies. The scene is concentric rings about X / Y, or 45-degree stripes
through X / Y that stay fixed on the screen while the glass turns over them (parallel squeezed copies, a
sawtooth, or undisturbed stripes, depending on Angle). A 4-virtual-pixel anti-aliased dark groove sits on every
flute boundary and hides the jump. Black and white: U = V = 512 on every pixel; ink about Y 775, ground Y 64.

## Controls

| Control | Function |
|---|---|
| K1 X | the scene's centre behind the glass, +-2 screens, +-8 counts of detent at the middle; moving it shows the lens parallax (default centre) |
| K2 Y | same, vertical |
| K3 Period | the scene's line spacing, 1024 .. 32 virtual px (up = finer), exponential; default 72 px |
| K4 Flutes | flute width 16 .. 430 virtual px, exponential; default 128 px. A flute centre always passes through the screen centre |
| K5 Angle | flute direction, -90 .. +90 degrees; the middle (+-6 counts of detent) = vertical flutes (default) |
| K6 Camera | camera (or Osc) strength, default 100 % |
| S7 Glide | On (default): X / Y ease 1/16 of the gap per field, never slower than 1 px per field, and land exactly; Period, Flutes, Angle, Camera and Power ease 1/16 per field. Off: no easing at all, the value read at this field start draws this field. Every knob is held until it moves more than one count |
| S8 Scene | Rings / Stripes |
| S9 Shade | Flat / Shade: each flute is shaded as a glass rod, the white ink dimming toward the grooves on a triangle |
| S10 Weight | Flat: Bold (equal bands) / Fine (1/6 duty lines). Shade: Bold = 4 flat steps of equal width (black, dark, light, white; step edges anti-aliased), Fine = the smooth triangle (65 levels) |
| S11 Source | Video / Osc |
| P12 POWER | the lens power, bipolar: 50 % (+-8 counts of detent) = flat glass; up to x4.9 at 100 %; down through 0 (about 40 %) to x-3.9 at 0 %. Default x3.2 (78 %) |

## Camera

The camera is never shown. Its luma ((Y >> 2) - 16, a 32-pixel box, then a 1/8 vertical IIR at half
horizontal resolution) is looked up in a table refilled every field, phase = level x K6 x 150 / 256 / 65536
cycles, and added to the scene phase (not to the lens), so rings and stripes swerve around the picture.
At 100 % a white area moves the lines by 2 periods, 50 % 1 period, 25 % half a period; 0 % is exactly the
black-input picture. S11 Osc puts the shell's slow drifting triangle lattice in at the same point with twice
the gain (4 periods at its peak); it is the only thing that moves by itself. The field need not be aligned
to the picture, so the pixel path taps the IIR's own read: the camera field sits one line up and 22 columns
left of the picture, with the right edge held.

## Engine

Polar chassis (Hypnos v1.1.0), 44 clocks of latency in every mode (processing_delay_clks 44).

- Per field: a small microcoded engine, started at every vblank zone (no avid for 1024 clocks) from address 0.
  37-bit accumulator, 64 x 32-bit register file and a 1024-word microprogram in block RAM, one adder, a serial
  18 x 18 multiplier and a serial right shift (16 micro-ops; about 600 words, 3,100 .. 3,400 clocks a field).
  It does the knob holds and glides, the sine / exp lookups with interpolation, three log2s, the flute phase at
  the top-left pixel (a restoring remainder) and all the DDA constants. flute.py holds the microprogram, its
  assembler and its emulator; the integer model runs the emulator, so there is one source for that arithmetic.
  Everything is in x-pixel units; the timing ID gives the line pitch and log2 of the x step, so SD, 720p and
  1080p draw the same picture (SD shows the centre 4:3).
- Per line: three adds (flute offset with wrap, scene coordinate along the flute normal with the flute jump,
  scene coordinate along the flute).
- Per pixel: the flute offset u is a DDA that wraps by the flute width; the scene coordinates (q, v) are DDAs
  in the FLUTE frame, q jumping by (Power - 1) W at every wrap, so the lens costs no multiplier. Then Hypnos's
  log-domain polar engine on (|q|, |v|) with 3 fraction bits: log2 hi, log2 lo -> u = log2(hi / lo) ->
  log2 r = log2 hi + g(u) -> exp2 mantissa -> one shift = phase. The period's fractional octaves ride in the
  exponent, so the phase multiplier is gone. Stripes use the same path with q = the stripe coordinate, v = 0.
- Anti-aliasing: Hypnos's one-pixel box filter of the band square wave. The phase gradient is analytic and
  includes the lens: along the flute Lf + log2|v| - log2 r, across it Lf + log2|Power| + log2|q| - log2 r (both
  fall out of the engine as -g and -g - u), combined as max + 1/2 log2(1 + 2^-2|a-b|). Lines finer than about
  2 px fade to the duty grey.
- Tone: coverage x flute factor (groove coverage / 4 anti-aliased steps / triangle, 0 .. 64) -> luma 64 .. 940
  -> [1 2 1] / 4 with the picture edges replicated -> 13/16 toward black -> blanking gate.
- Shell: generated avid (parity-locked), width latch with hysteresis and a first-line valid flag, 32-px box
  reloaded every line.
- Per-pixel multipliers: 2 x 7x7 and 11x8 and 8x6 (table interpolation), 8x7 (triangle), 9x7 (tone).

## Status

- v0.1.0, built 2026-11-15, not flashed. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline flute`
- router1, seed 1 on every config. Post-route Fmax: HD Analog 89.17, HD HDMI 88.13, HD Dual 92.32,
  SD Analog 88.48, SD HDMI 86.08, SD Dual 85.65 MHz.
- 6,661 .. 6,689 LC (87 %), 24 / 32 block RAMs.
- GHDL render = integer model, 0 differing pixels in every case rendered:
  - 720p full frames (918,720 px): defaults with a black input; bear.png in with Camera at 100 % (the model fed
    the tester's own stimulus luma; with its own resize of the image 15 % of the pixels differ slightly);
    Source = Osc (compared on the 8-bit conversion of the RTL frame, after the model learned that the
    lattice's first line keeps the x phase of the field before: before that, 925 px differed in rows 1 .. 12);
  - 640 x 360: Stripes + Angle + 4-step Shade + negative Power + off-centre X / Y; smooth Shade + Angle +
    camera at 100 %; Fine + Osc + horizontal flutes + Power 0;
  - NTSC 720 x 243, both fields: the same picture as 720p (the centre 4:3).
  All sims with Glide off; the Glide-on path (ease, 1 px per field floor, exact landing, the +-1 count hold)
  was checked in the microprogram emulator only.
- Not rendered: 1080p / 1080i rasters.

## Cut / deviations

- K5 Angle runs -90 .. +90 degrees with vertical at the middle (the spec says 0 .. 180): the default sits at the
  detent and the knob turns both ways. Both ends are horizontal flutes.
- Stripes are at a fixed 45 degrees on the screen (the spec only says "straight stripes"), so K1 and K2 both
  move them and K5 changes the look.
- Flutes reach 430 px (spec 400); Power reaches x4.9 and x-3.9 (spec about x5 and x-4).
- The first RTL had a hard-wired vblank sequencer and came to 9,062 LC (117 %). The per-field maths moved into
  the microcoded engine (6,6xx LC); the flute factor is carried at 0 .. 64 and the camera tap reads the IIR's
  own port (22-column, one-line offset of the camera field). Nothing visible was cut.
- Fine weight is a plain 1/6 duty (at least 5 virtual px un-lensed); lines the lens squeezes thinner fade
  through the box filter instead of being held at 2 px.
- With Glide on, the scene centre slides in from the top-left corner for about a second after the program
  loads (as on Hypnos).
- The anti-aliasing is one x pixel wide in every direction; on interlaced SD a horizontal edge is therefore
  sharper than the field's line pitch (as on Hypnos).
- Found while matching the model: `unsigned(resize(abs(x), 8))` folds 128 .. 255 (a signed narrowing resize
  keeps the sign bit). Hypnos v1.1.0 has the same line in its gradient combine (`s28_ad`), where it only
  matters with the spiral on (slightly soft edges where the radial and angular gradients differ by 2 .. 4
  octaves). Fixed here with a slice.
