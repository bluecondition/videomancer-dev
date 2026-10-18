# Sidewinder — per-scanline wave displacement (v1.1.2)

**STATUS: FINAL except presets — signed off by Ron 2026-10-10** ("I think it
looks good") and marked completed in the release audit. v1.1.0 is the
finalization pass of the v1.0 build from 2026-07-03, which never reached
hardware. Remaining: the 8 presets are hand-remapped placeholders; capture
real ones with `./capture_preset.sh` (`--defaults` first), then rebuild.

The picture snakes left and right down the screen: each line is the line
above read back from a dual-bank line buffer at a horizontally shifted
address, the shift being a rolling triangle/sine wave of the line number.

## Controls (v1.1)
| Control | Does |
|---|---|
| P12 Amount | Headline. Amplitude as a fraction of the measured width, quadratic, from a 3% floor (so every other control always shows) up to ±W/2. In Wrap, 100% coils the picture round a tube. |
| K1 Period | Vertical wavelength (freq = K1²>>8 per line), glided. |
| K2 Rate | Roll speed, bipolar with a centre detent (frozen), quadratic. |
| K3 Ripple | 3× harmonic, quadratic, 0..±W/8, independent of Amount. |
| K4 Twist | Bipolar, centre detent: wavelength changes down the screen about the centre row (right = bends bunch towards the bottom). Glided. |
| K5 Split | Chroma dispersion: U bends further than Y, V less (up to U 2× / V 0×), quadratic. |
| K6 Mix | Dry/wet. |
| S7 Shape | Triangle / Sine. |
| S8 Swell | ~6 s LFO on Amount. |
| S9 Steps | 16 px offset quantize (at 1080p; 16 virtual units in every mode). |
| S10 Edge | Mirror / Wrap, both over the inset columns [12, W−13]. |
| S11 Bypass | Mix forced dry (same latency). |

## v1.1.0 changes (from the 2026-10-10 review)
Hardware contracts the v1.0 code predated:
- **Blanking gate**: dry and wet both forced to 64/512/512 outside avid one
  stage before the mixer (the line buffer was read every clock, so picture
  leaked into blanking).
- **Write address**: pixel k was written at address k+1; Wrap's pointer ran
  0..W−1, so never-written address 0 showed as a 1 px black dot on the seam
  of every line. Now pixel k → address k (separate write counter).
- **Black fill** Y=0 (below black) is gone with Black mode itself (→ Mirror).
- **Vsync serration**: the frame sequencer is kicked once per field by the
  vsync edge guarded with a saw-active flag (it re-ran on every serration).
- **Active-line phase**: the wave phase and freq advance at the active-video
  fall, not on every hsync (vblank/equalizing pulses made the field-to-field
  phase source-dependent); this also removed a vstep-4 vs hsync race that
  could kill Rate.
- **Period glide** (and Twist): the phase at the bottom line is ~H·freq, so
  one LSB of K2 noise moved the bottom of an HD screen by ~15 px (~50 px at
  full Amount). Ease by diff>>3 per field with 6 fractional bits.

Design / engine:
- Amount on P12, width-relative with a floor; Mix to K6.
- New K4 Twist (freq accumulator per line, pivoted on the centre row by a
  per-field H/2·twist product) and K5 Split (three run-along pointers: Y, U, V).
- Edge Mirror/Wrap over the inset range so a capture source's black edge
  columns never become a seam (Ron's C_EDGE 12 HW preference).
- Ripple width-relative (±W/8 max, was ±63 px).
- Read address = the pointer register itself (no compare in front of RADDR);
  pointers bounce/wrap per pixel, initialised once per line by a folding
  step sequence (shared reducer: mod S for Wrap, mod 2S + fold for Mirror).
- `no_rw_check` on all six line-buffer RAMs (0 `emulate_transparency`).
- One sine ROM read port shared by both lookups: 23 EBR (was 24).
- Per-field work on one shared multiplier, per-line work on another, both
  with operand b split 6/5 hi/lo (product readable 3 steps after set): the
  unsplit 11×11 per-field multiply was the HD HDMI critical path (seeds
  67–79 MHz, 2/4 failing).
- Dry/wet mix is a program-local split-multiply lerp (shearline's shape),
  not the SDK `interpolator_u`, whose unsplit 11×11 multiply was the HD
  Analog critical path (75.45 MHz, seed 3 only). Same 4 clocks, latency 7.
- Line buffers initialised to black / neutral chroma.
- Field width measured on active line 8 (away from half lines), height
  captured on active lines.

## Build (2026-10-10)
`./build_programs.sh ron sidewinder` (router2), all 6 configs **seed 1**, no
retries: HD Analog 87.75, HD HDMI 77.60, HD Dual 85.32, SD Analog 85.65,
SD HDMI 85.95, SD Dual 86.41 MHz. 4929–4945 LC (64%), 23/32 EBR,
0 `emulate_transparency`. HD HDMI seed sweep on the final netlist: seeds
1–4 = 77.6 / 83.5 / 84.7 / 79.4, all pass, no dominant path.
Not simulated (build-first rule); not flashed.

## Hardware checklist (first flash)
- Blanking: no colour cast / bright edges in the borders.
- Wrap seam: no 1 px dark line snaking down the picture; no dark seam from
  the source's black edge columns (they are skipped).
- Rate on an analog source rolls at the same speed as on HDMI.
- K1 Period / K4 Twist: bottom of the screen steady with hands off.
- P12 sweep 0 → 100%: gentle 3% sway at 0, coiling tube at 100% (Wrap).
- K5 Split: prism fringes on bends, then colour detaching at high values.
- Thin coloured lines on analog sources would be the Drape U/V-parity case
  (each row replays the row above) — not fixed pre-emptively.

## v1.1.2 resolution scaling (2026-10-18)
SD and 720p now show the 1080p picture scaled; 1080p output is bit-identical.
- The wave runs in virtual 1080-line units from the timing ID
  (`registers_in(8)`): `s_ys` = virtual lines per field line (720p 1.5, 480i
  4.44, 576i 3.75, 1080i 2).
- The line sequencer's step 0 (phase += freq, freq += twist) repeats once per
  virtual line the field line covers (`s_rep`, 1 at 1080p; at most 5, so the
  sequence ends by clock 38 of SD's 138-clock blanking). Period and Twist
  therefore give the same bends per screen height everywhere: 720p had 2/3
  of the bends and SD about 1/4.
- The Twist pivot is the virtual centre row (`s_height_m` now counts virtual
  lines).
- S9 Steps quantizes to 16 virtual units: 1080p 16 px, SD 8 px, and 720p
  10.5 px via k = floor(3·off/32), off = 10.5·k (steps 11-13, which were
  multiplier waits). At 720p the Split product uses the unquantized offset,
  a sub-step difference.
- Amount and Ripple were already fractions of the width (unchanged). C_EDGE
  (12 px) is a property of the source's blanking columns, so it stays in
  pixels.
- Build `./build_programs.sh ron sidewinder` (router2), 6/6 on **seed 1**:
  HD Analog 88.00, HD HDMI 80.60, HD Dual 84.10, SD Analog 84.10, SD HDMI
  82.17, SD Dual 78.34 MHz. 5081–5100 LC (66%), 23/32 EBR. Not flashed.
- The community port's `[firmware] seed = 4` line is untouched; it needs
  re-checking when this version is synced (stock router1, 8 configs).

## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

v1.1.1 (2026-10-14): **seed 4**, set as `[firmware] seed = 4` in sidewinder.toml. LZX's stock
build uses one seed for all 8 configs. Why: seed 1 stalls nextpnr's router1 on SD Standalone (25+ min, no progress). Sweep: seeds 2–5 build 8/8; seed 6 also stalls SD Standalone.
With seed 4, the program's own HD logic (scratch core without LZX's Sync Out
generator) is 98.5 / 95.9 / 95.1 MHz (Analog / HDMI / Dual). The dev flow (router2) is unchanged.
