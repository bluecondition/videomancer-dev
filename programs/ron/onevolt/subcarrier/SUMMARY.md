# Subcarrier: the screen as the colour plane

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 7. A Swatch loop where hue is a voltage and the camera is the luma:
a constant-luminance rainbow whose geometry is the patch (a DSG3 saltire ramp bent into rings by the figure gradient),
one large Keychain figure drawn in exactly the complementary colours, and the camera's Y as the picture's luma.
Presets are hand-written placeholders.

## The rack

- **LZX Swatch** (YIQ -> RGB): the colour plane. Hue = the angle of (I, Q), saturation = its length; its Clamped output
  is stood in for by a hue-preserving gamut limiter. The hardware U / V are I / Q rotated by a fixed 132.5 deg.
- **Syntonie VU005** dual sine shaper (on a quadrature pair, set slightly past unity: min(1, 1.25 sin)): the phase
  voltage turned into cos / sin for I+ and Q+ (Mapper-style "hue is a voltage").
- **LZX Factors**: the gain on I and Q (saturation), and the keyed inversion of both inside the figure.
- **LZX Angles**: the 30 deg output, (2H + V) / 3, the diagonal hue ramp.
- **LZX DSG3**: shape 1 Absolute of the folded ramps, |tH - tV| (the saltire ramp); shape 2 the figure gradient
  (the circle recipe: parabola per axis, averaged; round in virtual coordinates).
- **LZX Keychain** ch 1: the figure's key (Thresh = Size, plus a 13 s LFO), with a 1-px edge band.
- **LZX Ribbons**: 3-bit flash digitiser + DAC in the Y path (S8).
- **LZX Proc**: the hue offset, the luma constant, and the Y -> I+ mix (S9).
- **LZX TBC2**: the camera's Y, crossfaded into Swatch's Y input.
- **Quadrature LFO** (Syntonie Animate / VU006): "Chroma Orbital Displacement" on I- / Q-, plus the slow hue slide.

## The patch

    Proc bias (K1 Hue) + 31 s LFO (+-45 deg) ------------------------------------------> hue phase sum
    Angles 30 deg out (S10 Angles) or DSG3 Abs |tH - tV| (S10 DSG3 Abs) -> attenuator (P12) -> hue phase sum
    DSG3 figure gradient, clamped at r = 760 virtual px -> attenuverter (K3 Rings) --------> hue phase sum
    hue phase -> VU005 pair -> cos to Swatch I+, sin to Q+                (S7 Out: 0 V on Q+, the I-axis duotone)
    I, Q -> Factors (K2 Saturation) ; Keychain key -> Factors: I and Q inverted inside the figure (S9 Inverted)
      S9 Luma>Chroma: the Factors inversion is unplugged; the Y signal -> Proc -> I+ (bright = orange, dark = cyan)
    DSG3 figure gradient -> Keychain ch 1 (K5 Size + 13 s LFO) -> the key
    Proc ch 2: K4 Luma constant + the gradient as shading + the key (a luma step) -> crossfader A
    TBC2 Y -> crossfader B ; crossfade = K6 Camera            (S11 Osc: the shell's drifting lattice on that jack)
    crossfader -> Ribbons (S8) or straight -> Swatch Y in
    quadrature LFO (19 s) sin / cos -> Swatch I- / Q- (through the Factors gain)
    Swatch -> gamut limiter (table by S7, Y, hue) -> encoder

## What it draws

At the defaults (black input): a kaleidoscope of constant-luminance rainbow bands in an X (the DSG3 saltire ramp,
2.4 turns of the wheel across the screen), bent into curves and rings round the centre by the figure gradient; a
large disc (radius ~390 virtual px, 70 % of the screen height across) in the middle where every hue is replaced by its complement and the luma steps up
a little; a faint luma dome brightening the centre. Saturated magenta, green, blue and orange; soft along the line
(the shell's chroma boxes), every edge a clean curve.

Motion at the defaults: the 31 s hue slide (+-45 deg) makes the bands drift and the rings breathe in and out of the
centre, the 13 s LFO breathes the disc's size by +-6 %, and the 19 s quadrature LFO orbits the whole palette.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Hue | Proc (bias on the hue phase) | rotates every colour round the wheel, 0 .. 360 deg, ends meet |
| K2 Saturation | Factors (gain on I and Q) | 0 = pure greyscale (only the luma, the orbit too) .. 512 = the designed level .. ~1.9x ("Saturation Boost"), every hue then limited at its own gamut edge |
| K3 Rings | attenuverter (figure gradient -> hue) | bipolar, +-8 deadband: centre = no rings; right = rings flowing outward (+3 turns from the centre to r = 540 at full), left = inward; the rings live inside r = 760, the outer screen keeps the ramp |
| K4 Luma | Proc (constant into Swatch Y) | 0.10 .. 0.72 of white: dark, deep colour .. bright pastel |
| K5 Size | Keychain (Thresh) | the complementary figure from a 70-px core to a 970-px disc covering most of the screen (radius linear in the knob) |
| K6 Camera | crossfader (TBC2 Y into Swatch Y) | 0 = the patch's own luma (exactly the black-input picture) .. 100 % (default) = the camera's luma in full, coloured by the field |
| S7 Q cable | VU005 Q output -> Swatch Q+ / 0 V | In: full hue wheel / Out: only the I axis, an orange / blue-cyan duotone through grey |
| S8 Y path | Proc / Ribbons | Proc: smooth luma / Ribbons: Y digitised to eight levels (DAC 0.05 .. 0.75): hard luma terraces, dark outer rings, a bright core, posterised camera |
| S9 Figure | Factors inversion / Proc Y -> I+ | Inverted: the complementary figure / Luma>Chroma: no inversion; luma pushes every colour toward orange (bright) or cyan (dark), all over the screen |
| S10 Ramp | Angles / DSG3 Absolute | Angles: diagonal straight bands bent into off-centre rings / DSG3 Abs (default): the saltire, bands in an X |
| S11 Source | TBC2 / Osc | Video (default) / Osc: the shell's drifting triangle lattice crossfaded in as the luma (dark holes, bright diamonds) |
| P12 SPECTRUM | attenuator (ramp -> hue) | 8 p^2 turns across the screen, glided: 0 % = one flat hue and its complement in the figure, with K3's rings; 50 % = two turns; 100 % = eight turns, dense even chevrons everywhere (the climax) |

All continuous controls glide 1/8 per field; no stepped knobs.

## Video

The camera is patched into Swatch's **Y input** through a crossfader (K6): the true picture, in synthetic colour.
Camera luma, (Y >> 2) - 16, is summed over 4-pixel cells, slewed down the screen (one pole, 1/4 per line, state in a
block RAM) and across (one pole, 1/4 per pixel), and read a line late, so raw input video is never carried in the
pipe. A per-field count of bright cells says whether a camera is present: a camera (or the Osc) crossfades in at once;
a black input fades back (1/16 per field) to the patch's own luma, so the black-input picture is the full patch at any
K6. Osc puts the shell's 640 x 540 virtual-px drifting triangle lattice on the same jack.

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").** 7-channel glide; LFO sine reads on the pixel ROMs; 13 slots
  on one 32 x 16 serial multiplier (P12^2, the ramp slopes and starts, the key radius with its breathing and its
  square, the camera fade, the saturation, the orbit through the saturation); the hue offset and read phases.
- **Per line (hblank, after each active line).** Yh = y8 >> 8 (half virtual px), Yh^2 by shift-add (11 steps) added
  to X0^2 as the beam's start value; the ramp's line term (|B| under DSG3 Abs, B under Angles).
- **Per pixel (14 stages).** DDAs for the ramp's x term and x^2 + y^2; the clamped gradient G = (760^2 - r^2) >> 10
  times |K3| (12 x 10, two partials); key compares (1-px band) and the dome; the patch Y and the 12 x 6 camera
  crossfade; phase = ramp +- rings; the keyed 180 deg flip as a toggle of the phase MSB; Ribbons (seven comparators)
  or a soft clip; two 256 x 16 quarter-wave shaper ROMs; a 2048 x 8 gamut table by {S7, Y >> 5, hue >> 11};
  s = min(gain, limit); s x sin (8 x 8, nibble partials); orbit + Luma>Chroma push; clamp.
- **Output.** As the shell: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16,
  the vertical [1 1]/2 with the line above, the blanking gate. Generated avid; one latency (C_LAT 20) in every mode;
  virtual 1080-line coordinates; colours in the hardware convention (U = Cr); drawn whites ~777.
- **Block RAM (16).** Camera cells 2, shaper ROMs 2, gamut table 4, box delay rings 4, line store 4. Every written RAM
  carries `no_rw_check` (0 transparency emulations in the standalone yosys log).

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI re-placed for the encoder pin skew: router1 seed 7 (HSYNC +1.25 ns, data <= -1.26 ns, 95.62 MHz); the builder's router2 seed 1 was outside the +1.0..+2.3 ns window. A plain rebuild gives the builder's placement again: re-run .heirloom_work/skew_tools/install.sh subcarrier router1 7 hd_hdmi. Screen data: .heirloom_work/rb_subcarrier/skew/.

- **v0.1.0 built 2026-11-25 with router2 (the build default), all six configs on seed 1, not flashed.**
  Rebuild: `./build_set.sh onevolt subcarrier` -> `out/rev_b/ron/onevolt/subcarrier.vmprog` (437,391 bytes).
- Post-route Fmax (the builder's last report per clock; every config passed on its first seed):

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 85.97 | 6,582 |
  | SD Analog | 1 | 93.42 | 6,577 |
  | HD HDMI | 1 | 88.03 | 6,570 |
  | SD HDMI | 1 | 84.18 | 6,570 |
  | HD Dual | 1 | 85.35 | 6,591 |
  | SD Dual | 1 | 96.32 | 6,583 |

  6,570 - 6,591 / 7,680 LC (86 %), 16 / 32 EBR. iceunpack OK x6; the packed config carries the new labels and presets.
  No pin-skew screening.
- **Checks** (working files in `.onevolt_work/rb_subcarrier/`):
  - `check.sh` clean; `scan.py`: 0 integer-minus-signed sites; TOML pre-check clean.
  - GHDL 720p60, decimation 1, black input: **bit-exact with the integer model `imodel.py`** (0 of 917,444 compared
    pixels, rows 1 .. 719; the tester's capture sits 4 px left of the generated avid) at the defaults
    (`rtl_default.png`) and with each of S7 Out, S8 Ribbons, S9 Luma>Chroma, S10 Angles flipped alone, P12 at 0 and at
    1023 (each a separate render, glides emulated from the power-up values).
  - 480i (`--video-mode ntsc`): bit-exact with the model (0 of 173,272 pixels, the second field).
  - K3 at 0 (inward rings, the negative ring path) with K5 at 1023: bit-exact (0 of 917,444).
  - A later default frame (field 8): bit-exact with the model at field 8; 907,728 of 921,600 pixels changed against
    the field-3 frame, so the patch moves exactly as the model moves it.
  - 1080i (`--video-mode 1080i5994`, 1920 x 540): bit-exact with the model (0 of 1,032,724 pixels); the same
    picture as 720p / 1080p.
  - Osc (`rtl_osc.png`) and selfie (`rtl_selfie.png`) frames match the model's renders in structure, not bit for bit:
    the model's camera-path filter does not reproduce the RTL's cell / slew start-up or the lattice phase exactly.
  - `thincheck.py` on the model (720p and 480i, 40 settings x 2 times): no single-row ridges of 24 px or more.
  - `sheet.png` (every knob at 0 / 25 / 50 / 75 / 100 %, every switch flipped, P12 0 / 50 / 100 %, all 16 S7 - S10
    combinations), `strip.png` (8 frames over 3.5 s), `camera.png` (selfie and bear at K6 0 / 25 / 50 / 100 %) and
    `presets.png`, all from the integer model; `cmp_final.png` (proto | model | RTL black | RTL Osc | RTL selfie).
- Fit history: the first RTL (per-pixel K3 x^2 DDA, double-buffered line engine, 18 stages) was bit-exact but packed
  to 7,606 LC (99 %); it was rebuilt as above (the ring term as one 12 x 10 product of the clamped gradient, the line
  engine in hblank, the flip as a phase-MSB toggle, Q20 ramps, a 3-level key edge, a 6-bit crossfade, 14 stages).

## Cut / deviations

- **Default S10 = DSG3 Abs** (the spec lists Angles first): the renders showed the saltire as the stronger default
  frame; Angles is one flip away.
- **The camera is a crossfade held on the patch luma while the input is black** (a presence count). A plain
  crossfade would turn the black-input picture black at K6 100 %; an additive mix left the subject unreadable under
  the busy hue field.
- **Hue slide is +-45 deg** (the spec says about 20) so the motion reads in a few seconds; the orbit rides through the
  Factors gain so K2 at 0 is pure grey.
- **Gamut limiter, not Swatch's per-channel clip**: saturation is limited per hue and luma (11.25 deg x 32-code bins,
  conservative), so boosted saturation shows each hue at its own edge rather than clipped primaries. The Luma>Chroma
  push and the orbit are added after the limiter and can still leave the gamut (the display clips them).
- **The figure gradient is the exact parabola** (round), standing for DSG3's folded, log-curved Average; it is
  clamped at r = 760 so the outer screen keeps straight ramp bands.
- **The key's edge is a 3-level band** (outside / half / inside, about 1 px wide), not a 16-level coverage ramp.
- **K6 crossfade has 64 steps** (glided).
- **SD shows the centre 4:3** of the 1920-wide virtual canvas: the same disc and rings, fewer chevrons across.
- **LFO periods are in fields** (19.2 / 31.2 / 13.0 s at 60 Hz; 20 % longer at 50 Hz).
- **thincheck.py counts a single-row ridge only when the rows beyond it are flat**, so the crest of a smooth hue band
  sampled at 480i is not counted (the first, cruder version flagged those at 480i P12 100 %).
