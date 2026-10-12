# Infrared — summary

Catalogue combo #10 (seed 20260912): perspective keystone + thermal-camera
false colour. v0.1.0 (2026-09-10) was flashed and liked ("infrared is
good", 2026-09-11) and marked Ready on 2026-10-12. v1.0.0 is the
finalizing pass, built from a review of v0.1.0.

## Controls (v1.0.0)

| Control | Does |
|---|---|
| K1 Keystone | rows scale linearly with height about the centre |
| K2 Skew | parallelogram shear, up to ±half the measured width |
| K3 Zoom | horizontal scale 0.25×–4×, log, 256 steps |
| K4 Palette | Thermal, Ironbow, Plasma, Rainbow, Ice, Night Vision, White Hot, X-Ray |
| K5 Range | luma gain into the palette, 0–400% (100% = unity) |
| K6 Offset | luma bias into the palette, ±one whole ramp |
| S7 Rows | palette offset rises down the frame |
| S8 Iso | black isotherm lines every eighth of the ramp |
| S9 Cycle | off: P12 = heat position (clamped ramp); on: P12 = cycle speed (wrapped ramp) |
| S10 Luma | palette luma / original luma under palette chroma |
| S11 Ramp | normal / inverted brightness-to-palette mapping |
| P12 Heat | Cycle off: glided push up the ramp (¾ ramp at 100%); Cycle on: speed, squared curve, ~0.5 s a lap at 100% |

## v1.0.0 changes (from the v0.1.0 review)

Bugs:
- S9 Pins was dead: the palette supplies all output chroma, so the
  keystoned-chroma path it steered never reached the screen (yosys had
  already pruned the U/V line buffers). Replaced by Cycle; the chroma path
  is deleted from the source.
- The Heat cycle snapped back every lap: only `heat(9:2)` (a quarter of the
  1024-step ramp) was added. Now the full phase is added, so the cycle is seamless.
- The clamp/wrap choice read the raw slider in the pixel path (pot noise near 0
  flickered clamped highlights). Wrap is now S9 Cycle, latched per frame.
- The palette index was 13-bit and overflowed at high Range + Rows + Offset
  (highlights flipped to the cold end). It is now 14-bit.
- Iso blacked out every clamped-dark pixel (whole shadow areas in X-Ray and
  Plasma). Isotherms now skip clamped pixels.
- K4 labelled-default trap: the firmware loads a labelled default as its label
  index. Values below 8 are honoured as an index only until the knob first reads
  a real position.

Hardware contracts:
- Generated avid (damier/aurora parity-locked start, field-latched width).
  It also drives the keystone DDA. Infrared draws its own chroma, so it falls
  under the aurora rule.
- HD HDMI bar fix (damier v0.4.4): [1 2 1] on Y/U/V plus luma scaled to white
  ~775, before the gate. The cycling wrap drags a full-swing white-to-black edge
  across the picture every frame.
- Black (Y 64, U/V 512) outside the keystoned source, on isotherms and at the
  generated-avid edges. In v0.1 the border was painted through the palette.
- Latency 14 → 16 clocks, the same in all modes.

Controls:
- Skew scales with the measured half-width (was a fixed ±512 px: 27% of HD,
  71% of SD). Offset covers ±one ramp (was ±¼). Range is relabelled 0–400%.
- Palettes are 8 × 256 entries (was 4 × 64; 64-step banding). The four v0.1
  palettes are kept exactly at their old sample points and interpolated in
  between. Ironbow, Rainbow, White Hot and Night Vision are new RGB ramps.
  Generator: `tools/gen_palettes.py` (rewrites the block between the GENERATED
  TABLES markers; `--png` writes a preview).
- Zoom: one-octave 2^(i/64) ROM plus an octave shift. 256 steps of 1.1%
  (was 64 steps of 4.5%).
- Ramp = Invert now mirrors only the luma term, so Offset, Rows and Heat still
  push the scene hotter.

## Build

`./build_combo.sh infrared` → `out/rev_b/ron/catalogue-combo/infrared.vmprog`.

v1.0.0 build (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 85.6, HD HDMI 85.8, HD Dual 84.8 MHz; SD Analog 91.0, SD HDMI 83.1,
SD Dual 79.7 MHz. About 5,000 LC (65%), 22/32 EBR (10 luma line buffer, 12
palette ROM). v0.1.0 was 79–84 MHz HD at 4,212 LC and 12 EBR.

HW status: v1.0.0 verified on hardware by Ron, 12 Oct 2026 (signed off).

Defaults: Ron's hardware capture from earlier on 12 Oct (commit 544a9bf, on
v0.1.0) was carried into v1.0.0 and repacked onto the same bitstreams. Two
values changed meaning: K6 539 → 519 (the Offset range is 4× wider), and
S9 Pins = Mirror → Cycle = On. Pins was dead; the look came from Heat 1%,
which put v0.1 into its frozen wrap mode. Wrap is now Cycle = On, and Heat 10
creeps about one palette step every 30 s. The 7 presets are hand-written;
recapture them with capture_preset.sh.
