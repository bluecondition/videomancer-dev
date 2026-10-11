# WAKE — summary

Motion-keyed effects: effects land where the picture moves. Engine = `motion_key.vhd`
(reusable), host = `wake.vhd`. Design notes and phase log: PROGRESS.md. Golden model: model/.

## Build
- **ROUTER1 required**: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=1200 ./build_programs.sh ron wake`
  (router2 misses or stalls at 88% LC).
- v0.3.0 seeds (router1): HD Analog s1 (75.6 MHz), HD Dual s1 (78.3), SD x3 s1. **HD HDMI = seed 36,
  hand-installed** (77.7 MHz, HSYNC +0.62 ns, data −2.28..−0.92). Of 36 screened seeds, only 17
  (+1.03) and 31 (+0.93) clear ±0.7 ns, and both fail timing. The builder's seed 5 sat at +0.31
  with data straddling the clock. Re-screen + reinstall after EVERY rebuild (the builder reinstalls
  its own seed). If HD HDMI shows thin coloured lines, try the next seed rather than RTL changes.

## Status
v0.3.0 (Phase 3: v3 engine, DETAIL, STEADY, Solarize/Crush/Smear, presets 1–7) built 2026-10-11, not flashed.
Golden TB bit-exact (SD, HD-1080i, STEADY+cut). LC ~7,170/7,680 (93.7%), 17 EBR; HD timing margin ~1–2%.
