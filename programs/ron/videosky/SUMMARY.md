# VideoSky — the picture cut into a copper line-screen

Lifted out of **Penrose**, whose "Video Sky" switch engraved the incoming
video into the sky behind the impossible staircase. That was eight lines of
code (`penrose.vhd` stage C2): horizontal rules at a fixed 8-pixel pitch,
each rule's thickness set by the top three bits of the inverted luma, drawn
in ink on paper. Dark video → fat rule → the paper reads dark. It is the
oldest reproduction trick there is — a mezzotint line screen.

VideoSky keeps that kernel exactly (K4=0, P12=0, Waver=0 reproduces it
verbatim — the "Penrose Sky" preset) and builds a whole instrument around it.
Fully streaming and stateless: no line buffer, no frame memory, no warm-up;
every control lands on the next frame.

**P12 "Relief"** (headline): the luma is added to the screen's *phase* before
the thickness compare, so the rules **bow around** the image instead of merely
fattening — bank-note engraving, *Unknown Pleasures*. At 0 the screen is dead
flat (Penrose's sky). At max the luma swings the phase through ~4 line periods
and the field becomes pure topography. Pair with **S10 Width = Fixed** and
Relief is the *only* thing shaping the picture — constant-weight rules, all
form carried by displacement.

## Controls

K1 Ink (bias on the whole tone ramp, ±256) · K2 Contrast (1.0×–4.9× about
mid-grey) · K3 Pitch (continuous, linear in *frequency*: period ~128 px →
~4 px) · K4 Angle (continuous 0–360°) · K5 Waver (the burin wandering in the
hand: sine-of-Q wobble along the rule, ~4 line-spacings per period) ·
K6 Scheme (Parchment / Gallery / Night / Blueprint / Cyanotype / Copper) ·
S7 Cross (second field down the Q axis, biting **only** into the dark half of
the ramp, doubled — how a real engraver builds shadow) · S8 Invert ·
S9 Tint (ink takes the source's own chroma — the rules print in the colour of
what they cut) · S10 Width (Fixed / Modulated) · S11 Drift (rules crawl).

## Architecture

7-stage streaming pipeline, `C_LATENCY = 7`. **Zero BRAM**, ~3130 LC (41%),
three small pixel-rate multiplies.

- **Continuous pitch and angle from an accumulator pair.** Penrose tested
  `v_count mod 8`, so the screen was locked horizontal at a power-of-two
  spacing. Here the screen coordinate is fixed-point (8 fractional bits, phase
  unit 256 = one line period): **P** steps by `dpx` per pixel and `dpy` per
  line, where `(dpx,dpy) = pitch·(sin,cos)` resolved once per frame. Lines are
  the level sets of P, so *any* angle at *any* spacing costs the same two adds
  per pixel and no multiply. **Q** is P rotated 90° (`dqx=-dpy, dqy=dpx`) and
  pays for itself twice: it is the crosshatch field **and** the waver argument.
  cos drives the per-*line* step so angle 0 gives Penrose's horizontal rules.
- **24-bit accumulators, wrap-safe for free.** A 24-bit wrap is a multiple of
  2^16, so the phase byte `acc(15 downto 8)` is continuous across the wrap —
  the width costs nothing and needs no reset logic.
- **Only the low 8 bits matter.** `(phase + wob + rel) mod 256` is all the
  compare needs, so the 8-bit phase byte carries everything the wide
  accumulator had — the displacement adds stay 12-bit.
- **One shared multiplier for the frame resolve.** `step·cos` and `step·sin`
  go through a single registered 9×8 multiply, split-operand
  (`step = sh·256 + sl`) across vblank cycles, so no wide combinational
  product ever lands on a vsync-latched register — *the ziffern trap: those
  paths are still STA-timed even though they only fire once a frame.*
- Interlace detected by field-parity toggle; the odd field starts half a line
  step down so the two fields interleave instead of printing the same rules.

## Status

All 6 configs routed on **seed 1**, no retries, comfortable margin:

| Config | Fmax | LCs |
|---|---|---|
| HD Analog | 98.6 MHz | 3130 |
| SD Analog | 98.6 MHz | 3125 |
| HD HDMI | 88.2 MHz | 3126 |
| SD HDMI | 82.0 MHz | 3126 |
| HD Dual | 95.7 MHz | 3124 |
| SD Dual | 96.8 MHz | 3129 |

Full clock everywhere (74.25 MHz required for HD). Packaged.
**Not yet HW-tested, and the render is not yet sim-verified.**
