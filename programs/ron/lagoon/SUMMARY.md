# Lagoon

Underwater look filter: incoming video is refracted through rippling water,
graded blue-green, and lit with dancing caustic highlights. Forked from
sidewinder's dual-bank line-buffer displacement engine.

## Signal path

1. **Refraction** — each pixel is read back from the previous line at a
   horizontally displaced address:
   - *Swell* (K2): per-line sine wave, up to ±255 px, computed once per line
     in the hblank step sequencer (registered multiplies).
   - *Ripple* (K1): per-pixel interference of two triangle waves at a ~1.6×
     frequency ratio, drifting in opposite directions — the netting/wobble
     of the water surface. One registered per-pixel multiply.
   - Read address clamps to [1, width]. v1.2: LEFT-edge overshoot no longer
     smears the border pixel (often black) — each line's average colour
     (first 64 active pixels, sum >> 6, latched at hsync so it matches the
     line being read back) is painted into the uncovered strip, and
     caustics/murk/tint still apply on top, so the reveal is water-toned
     haze. Right edge keeps the border-pixel smear. The 64-pixel window is
     a single constant in p_ctrl if it needs tuning.
   - v1.2.1: the fill zone extends to reads of source columns < C_LEDGE (6)
     and the average window starts at column 8 — capture sources carry a
     few black blanking-edge columns that showed as a black seam between
     the fill and the picture (and darkened the average).
2. **Caustics** (K6): two more triangle waves interfere; where their sum
   tops a knob-controlled threshold, Y gets up to +384. The knob moves the
   *threshold*, not a gain, so there is no per-pixel gain multiply.
   - v1.1: the ripple folds *phase-modulate* the caustic waves (±1/8
     period) so the diamond lattice bends into writhing curves, and a ±32
     hash dither (x bits ⊕ line ⊕ frame LFSR) roughens the contour edges —
     the v1.0 grid read as too rigid.
   - v1.1.1: brightness halved per HW feedback — 1× gain capped at +192
     (was 2× / +384), glitter cores +256 (was +511).
   - v1.2.2: the dither's pixel/line LSBs (x·bit0 ⊕ line·bit0) showed as a
     checkerboard at caustic contours on HW — now x bits ≥ 1 (2-px minimum
     feature) ⊕ a per-line-stepped LFSR (random vertical phase), amplitude
     ±16, so it reads as soft grain.
   - v1.3.0: the caustic field now SWAYS WITH the swell — each line's
     caustic phase is seeded +base_off×fc (two line-rate multiplies in the
     hblank sequencer; 16-bit phase wrap makes the mod arithmetic exact),
     so the light pattern rides the displaced picture instead of staying
     bolted to screen space. Cost ~570 LCs (72% → 79% util).
   - *Glitter* (S8): caustic peaks above a fixed level become blown-white
     cores; a frame LFSR xor'd with phase bits lights ~1/4 of the cores per
     frame for a twinkle.
3. **Murk** (S9): per-line accumulator subtracts up to 288 from Y toward
   the bottom of the screen. Step is chosen per resolution from the
   measured lines-per-field, so the gradient depth-matches SD/HD.
4. **Tint** (K5): U/V lerp toward the S7 target (Aqua or Abyss) through two
   interpolator_u instances; Y rides a matched 4-deep delay.
5. **Depth** (P12): master dry/wet mix (3 interpolators) — the headline
   fader, from clean video down into the deep.

## Controls

| Ctrl | Name | Function |
|------|------|----------|
| K1 | Ripple | fine refraction amplitude, 0–95 px |
| K2 | Waves | per-line swell amplitude, 0–255 px |
| K3 | Speed | bidirectional, centre detent = frozen, quadratic |
| K4 | Scale | spatial frequency of ripple/caustics/swell, quadratic |
| K5 | Tint | chroma pull toward the water colour |
| K6 | Caustics | highlight coverage (threshold) |
| S7 | Hue | Aqua / Abyss tint target |
| S8 | Glitter | twinkling blown-white caustic cores |
| S9 | Murk | depth-gradient darkening |
| S10 | Sea | Calm / Storm (swell 3rd harmonic, 1.5× amp, 2× rate) |
| S11 | Bypass | |
| P12 | Depth | master dry→wet fader |

## Colour convention

Tint targets stored U/V (Cb/Cr) **swapped** for hardware, matching the
HW-verified mondrian/prism palettes (pre-swap BT.601: Aqua Cb=600 Cr=390,
Abyss Cb=690 Cr=465). The simulator renders standard BT.601, so tints look
warm/olive in sim — correct on hardware.

## Implementation notes

- All SPI knob/switch registers pre-registered every clock before any
  arithmetic (pyro/starfield lesson).
- Per-frame sequencer (vblank): speed/scale quadratic curves, wave
  frequencies, temporal phase advance, murk step select, LFSR.
- Per-line sequencer (hblank): swell sine + harmonic multiplies, per-pixel
  accumulator seeding, murk value.
- Per-pixel: 4 phase accumulators + combinational triangle folds + 1
  registered multiply + clamp. 6 interpolator multiplies total (2 tint,
  3 mix), all with registered ins/outs.
- Pipeline latency 16 clocks; dual-bank Y (2048) + U/V (1024, 2:1) line
  buffers, ~22 EBR.

## Water-effect ideas explored / future

- **Vertical refraction** — needs a multi-line buffer; EBR budget (line
  buffers already ~22/32) rules it out at full width.
- **Surface + reflection split** (waterline with mirrored sky) — vertical
  flip needs a field buffer; not possible on this hardware.
- **Cellular (Voronoi) caustics** — triangle-interference net is the cheap
  stand-in; real cellular noise would need per-pixel hash + min-distance.
- **Bubbles / particles** — could fork fireworks' counter-rasterized
  sprites as a rising-bubble layer (line-buffer sprite engine).
- **Chromatic refraction** (offset U/V read address a few px from Y) —
  cheap to add: second clamp + separate UV pointer; candidate for v1.1.
- **Waterline mode** (top of screen dry, effect ramps in below a K-set
  line) — murk accumulator already gives the vertical ramp hook.

## Timing / build (v1.3.0, 2026-07-12)

All 6 configs routed at full clock, ~6080 LCs (79%), 23 EBR. Build with
`SEED=9 ./build_programs.sh ron lagoon`. v1.3.0 seeds: HD Analog 11 (79.37),
HD HDMI 14 (75.03 routed — completed on the final retry, re-verified
manually), HD Dual 9 (80.22). Utilization is back near the congestion zone;
the next feature should budget for a cut or a seed hunt. Seeds MOVE whenever the netlist changes;
only mid-sweep completions are trustworthy without re-verification (v1.1's
hd_hdmi completed on the final retry and needed a manual re-route to confirm).

Timing-closure history (44 -> pass on HD configs):
1. First cut was 6776 LCs / 88% and congestion-failed everywhere: dropped the
   2 tint interpolators (16-level shift-lerp instead), Bypass via mix t=0
   (video SRs 16 -> 9 deep), storm harmonic as fixed shift, 8-bit ripple amp.
2. Sine-ROM BRAM read fed the swell multiply in one cycle (yosys folds the
   read register into the EBR output port) -- re-register in fabric first.
3. HD Dual still congestion-bound: replaced the K3^2 / K4^2 knob-curve
   multiplies with piecewise-linear shift knees (~-300 LCs), then seed-swept.
4. Builder gotchas hit: "Completed (seed 6)" at the retry cap is a
   best-effort MISS; hd_hdmi's clock is ADV7611_CLK not ADV7181C_CLK, so
   grep for "Max frequency" generically when verifying.

## Status

- **v1.3.1 COMPLETE 2026-07-12** — HW-validated and user-approved through five
  rounds of hardware iteration (caustic brightness, checkerboard dither,
  left-edge fill + seam, swell-tracking caustics).
- Defaults are the user's hardware-tuned settings: Ripple 48px, Waves 113px,
  Speed 59%, Scale 25%, Tint 63%, Caustics 45%, Aqua, Glitter Off, Murk On,
  Calm, Depth 100%.
- Packaged at out/rev_b/ron/lagoon.vmprog.
