# Bajaweave — two swaying line oscillators, woven

**v1.0.0 (2026-10-07): FINAL — signed off by Ron on hardware ("Bajaweave can be finalized").**
Defaults and the "Default Weave" preset are HW-captured (Opposite flow, Shadow depth,
+17% speed); "Slow Drift" and "Frantic Stripes" are hand-written.

## What it does
Two phase-modulated carriers draw vertical stripes whose sway travels up or down
the screen. They are layered **Woven** (over/under alternating at every crossing,
by stripe parity) or **Stacked** (osc1 on top), in a fully saturated
complementary colour pair that K3 turns around the wheel.

| Control | Function |
|---|---|
| K1 / K4 | Osc1 / Osc2 bars across the screen (~5–125 at HD) |
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
- **v1.0.0 build (2026-10-07, bitstreams = the v0.17.0 build, TOML-only repack):** all 6 configs on seed 1 except **SD HDMI = seed 2**
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

## Community port — fit for the stock SDK (v1.0.2, 2026-10-14)
- The stock LZX SDK builds 8 configs, and its SD Standalone core is ~600 LC
  larger than HD; v1.0.1 packed to 7759/7680 LC there and could not place.
- v1.0.2 replaces the five per-knob smoother instances (K1/K2/K4/K5/K6, ~96 LUT
  + 78 FF each) with ONE shared 3-stage datapath (`p_ksmooth`). A free-running
  sweep keeps each knob's deadband flag current and K1/K4's next eased values
  ready. The guarded field start loads K1/K4 on that clock and runs one apply
  pass over the rest, done 8 clocks later; the gain engine loads at +16. The
  deadband (5 counts), glide (1/8 per field, snap inside half a count) and the
  three snap-to-knob field starts are unchanged.
- Why K1/K4 must land on the field-start clock itself: the carrier
  accumulators free-run through blanking (they reset only at each line's
  avid start), and the first output pixels of the first active line still show
  that history. A version that eased K1/K4 a few clocks late differed in one
  or two pixels at the left end of the first line whenever a knob moved.
- Proven bit-exact against v1.0.1: GHDL side-by-side, 40 frames on a 403 x 201
  raster (a frame length that is not a multiple of 8, so the sweep meets the
  field start at a live slot), knobs and switches changing mid-picture,
  0 mismatches in 3.24 M clocks. A deliberately broken flag freeze was caught
  (159 k mismatches), so the test reaches this logic.
- Packed LC (stock SDK): SD Standalone 7759 → 7455, HD Dual 7176 → 6870.
- **Build seed (stock LZX flow): `[firmware] seed = 6`.** Seeds 1–5 each hung
  nextpnr's router1 in at least one config (a rip-up livelock with no end; LZX's
  build script has no timeout). Seed 6 builds all 8; own HD timing (core
  without the Sync Out generator) is 102.0/103.4/96.4 MHz Analog/HDMI/Dual.
