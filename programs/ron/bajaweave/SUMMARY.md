# Bajaweave — two swaying line oscillators, woven

**v0.14.0 (2026-10-06): HW-iterating, finalizing.** Look signed off on hardware
at v0.13 ("everything visually is looking perfect"); v0.14 adds knob smoothing
for an occasional stutter in the scroll.

## What it does
Two phase-modulated carriers draw vertical stripes whose sway travels up or down
the screen. They are layered **Woven** (over/under alternating at every crossing,
by stripe parity) or **Stacked** (osc1 on top), in a fully saturated
complementary colour pair that K3 turns around the wheel.

| Control | Function |
|---|---|
| K1 / K4 | Osc1 / Osc2 stripe frequency |
| K2 / K5 | Osc1 / Osc2 sway wavelength (~1–17 cycles per screen) |
| K3 | Colour: osc1 hue; osc2 is always its complement |
| K6 | Sway amount (0 = straight lines … ~4× default) |
| S7 | Layering: Woven / Stacked |
| S8 | Edges: dark full-colour outlines + outer-bend highlight |
| S9 | Depth: Flat / Shadow (the upper ribbon drops a shadow onto the one under it) |
| S10 | Flow: Opposite (equal counter-speed) / Same (osc2 drifts slowly ahead of and behind osc1, up to 3 lines/field) |
| S11 | Shape: Sine / Triangle (glides over ~0.5 s) |
| P12 | Speed, bipolar: centre = frozen, ends = ~23 lines/field each way, the same vertical speed for both waves (eased) |

## Build notes
- **v0.17.0 build (2026-10-07):** all 6 configs on seed 1 except **SD HDMI = seed 2**
  (seed 1 stalled router2; not a timing miss). Post-route Fmax: HD Analog 89.0,
  HD HDMI 93.6, HD Dual 91.6, SD Analog 98.4, SD HDMI 100.0, SD Dual 87.9 MHz.
  6945/7680 LC (90%), 24/32 EBR, pipeline latency 19 clocks in all modes.
- **HD HDMI layout lottery:** this program has twice shown thin, full-width,
  colour-swapped lines on HD HDMI with one placement and none with another of
  the same netlist (a core TX-pin skew matter, see memory `hdmi_tx_placement_skew`).
  Check HD HDMI on hardware after every rebuild; alternates are packed in
  `out/rev_b/ron/_diag/` (seeds 2–4). To ship an alternate without a rebuild
  re-placing it: copy its `hd_hdmi.bin` into `build/.../rev_b/bitstreams/`, then
  `toml_to_config_binary.py` → `rev_b/program_config.bin` and
  `vmprog_pack.py --no-sign --hardware rev_b`.
- Per-field logic (gain, rates, easing) is STA-timed at the pixel clock: every
  fused abs/compare/multi-term add there has at some point been the HD
  critical path. Keep them one operation per register.
