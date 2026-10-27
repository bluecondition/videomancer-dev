# GILT v3: Klimt gold paint, swimming in oil

Live video re-rendered as Klimt gold paint covered in ornament that swims around the
picture like oil. Controls: `GUIDE.md`. Why it is built this way: `DECISIONS.md` D14
(and D13 for the gold-painted picture). Status and next steps: `PROGRESS.md`.

**Status: v3.1.2-diag (2026-10-12): v3.1.1 was still blue on hardware, so S11 temporarily flips the U/V output order live, and HD placements are screened for encoder pin skew. Waiting on Ron's A/B (D16).**

## What it is
- **The oil (v0.10.0's warp, HW-approved).** A blurred copy of the picture (~8 px x ~4
  lines, luma and both chroma axes) drags the ornament space: luma sideways, warm/cool
  chroma up and down (S8 Flow swaps them). Two smooth parabolic-sine waves (wavelength
  2 cells) and a large per-line sway bend it further and drift. P12 Swirl scales all of
  it: drag 0..2x v0.10, waves and sway 0..3/4 cell each, drift 0..7 steps per field.
- **The textures (Sugarcoat's mechanism).** Each pixel of the blurred field is
  classified: luma band (Luma map), or hue octant where the picture has colour and
  luma band where it is grey (Chroma map). A 16 x 16 theme ROM turns the class into
  one of 16 textures, evaluated in the warped space: 0 leaf, 1 dots, 2 circles-in-
  circles, 3 spiral, 4 hollow squares, 5 checker, 6 eyes, 7 jewel blobs, 8 tall bars,
  9 mosaic, 10 oil marble, 11 chevrons, 12 rosettes, 13 triangles, 14 gold flakes,
  15 waves. Cell-based ones use a half-pitch row stagger, per-cell hashes for size,
  colour, chirality and presence (K6 Density).
- **The ground** is the gold-painted picture: per-pixel luma through v0.10's 32-step
  gold/brown ramp, with leaf grain keyed to the warped space. K4 Contrast and K2 Poster
  act on it too. Since v3.1 the inks stay on the same ramp: dark = the paint K5 Ink
  steps (+0..3 per cell) toward umber, light = the same distance toward bright gold.
  Only the 8 jewels (S11) leave the gold/brown family. No raw video reaches the output.

## Architecture (C_LATENCY 16, one latency in every mode)
- Field (free-running, consumed a few px stale under the blur): IIR blur through a
  512 x 24 line buffer → drag drives and contrast → luma-band ROM (512 x 3) and hue
  octant → class → theme ROM → texture index; waves → `s_warpx/s_warpy`; marble band;
  outline detector (texture vs 2 previous pixels and the line above, flag stored with
  the texture in a 2048 x 5 line RAM so it is 2 lines thick).
- Pixel path: E1 latch → E2 warp, paint contrast → E3 row stagger, grain hash → E4
  cell slices, paint posterize/invert → E5 cell hashes, folded offsets → E6 squares,
  max/min, 16-sector angle → E7 metrics → E8 all 16 texture rules in parallel → E9
  texture mux, paint ROM → E10 inks, grain → E11-E13 paint + (ink − paint) x Ink → E14
  outline → E15 [1 2 1] → E16 luma scale + blanking gate.
- Hardware contracts: U/V swapped at the field input, STANDARD order at the output (D16); generated avid
  (parity-locked start, field-latched width) drives the output avid and the blanking
  gate; HD HDMI bar conditioning before the gate; serration-safe field event; frame-line
  texture coordinates under interlace; `no_rw_check` on both line RAMs.

## Build v3.1.2-diag (2026-10-12): S11 = live U/V output-order test

HD bitstreams hand-picked for encoder pin skew (`tools/hdmi_pin_skew.py`, HDMI
hs-clk / closest data bit). The builder's own HD placements were in the danger zone:
HD HDMI s2 −0.06 / +0.06, HD Analog s1 +0.36 / +0.74. v3.1.1's HD HDMI s1 was +0.74 / −0.31.

| config | post-route Fmax | seed | HDMI hs-clk / data max |
|---|---|---|---|
| HD Analog | 87.34 MHz | **33** (hand-installed) | −1.26 / −0.91 |
| HD HDMI | 88.49 MHz | **6** (hand-installed) | +1.55 / −0.82 |
| HD Dual | 78.76 MHz | 1 | −2.13 / −1.01 |
| SD Analog / SD HDMI / SD Dual | 83.09 / 80.69 / 92.08 MHz | 1 / 1 / 1 | (27 MHz, skew irrelevant) |

Repacked with `vmprog_pack.py`. A full `build_programs.sh` re-routes HD back to its own
seeds: for TOML-only edits, repack. Screening work: `/home/ron/gilt_v3_work/skew/`.

## Build v3.1.1 (2026-10-12, `./build_programs.sh ron gilt`, router2)

| config | post-route Fmax | seed | LC |
|---|---|---|---|
| HD Analog | 83.93 MHz | **2** (seed 1 stalled the router) | 5579 / 7680 |
| HD HDMI | 89.12 MHz | 1 | 5592 |
| HD Dual | 87.25 MHz | 1 | 5586 |
| SD Analog | 95.73 MHz | 1 | 5608 |
| SD HDMI | 85.08 MHz | 1 | 5592 |
| SD Dual | 87.35 MHz | 1 | 5570 |

73% LC, 7 / 32 EBR. Backups: `versions/gilt_v3.{0.0,1.0,1.1}.{vhd,vmprog}`. Router stalls
on seed 1 move between configs from build to build (v3.0 SD HDMI, v3.1 HD HDMI, v3.1.1
HD Analog): it's router2 noise, not a timing deficit.
