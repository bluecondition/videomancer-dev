# Argyle — Heirloom #8, rebuilt from the photo

**v0.5.0 (2026-10-21): built, NOT flashed.** Rebuilt from `photos/WIN_20230207_23_50_54_Pro.jpg`
after Ron judged v0.1.0 too abstract. It now redraws the photo itself: a lattice of nested diamonds
with magenta lens-shaped cores and fine pink/white, lavender, blue and cyan/green rings, green and cyan
X junctions, and big and small cores alternating along each row. Work files are in
`.heirloom_work/rb_argyle/` (`notes.txt`, `proto.py`, `imodel.py`, `cmp_final.png`).

## Reading of the photo (measured)
- **One repeating tile.** The frame (beyond the 60-px sync edge) is one tile repeated in both
  directions: 524.8 px wide (two diamonds) by 261.0 lines, with a 0.00095 line/px tilt. Folding the
  frame onto it leaves 6.2 RGB rms. That makes 7.32 diamonds across and 4.14 rows.
- **Lattice.**
  - A rows sit at y = 147 + 261k. Their cores alternate: big at x = 227 + 525k (64x57 px) and small
    at x = 481 + 525k (40x26).
  - B rows sit at y = 276 + 261k, with eye-shaped cores of 60x28. A B row lies 128.8 lines below its
    A row, so it is not centred (49 %).
- **The field is a sum, s = G(x) + H(y).**
  - The gradient-ratio test dC/dx : dC/dy = G'(x) : H'(y) holds with a log MAD of 0.11.
  - G and H were fitted from 143 luma ridge contours, each ring a level line of s, to 0.75 px rms.
  - **G (horizontal oscillator).** A rounded triangle, which gives the lens-shaped cores.
    - Extremes: A-big +0.91, B −0.91, A-small +0.74, on a scale where H spans ±1.
    - Harmonics: cell fundamental 0.77; half-frequency 0.098, which is the big/small alternation;
      then 0.031, 0.04 and 0.047.
  - **H (vertical oscillator).** A triangle with sharp extremes; its slope is 1.7x steeper toward
    the B row.
- **Rings.**
  - Each half cell has 7 rings plus the junction ridge, at |s| = 0.24, 0.44, 0.64, 0.85, 1.05, 1.26
    and 1.49. The spacing is about 0.2 and is symmetric on the A and B sides.
  - Below |s| = 0.15 is the dark diagonal boundary; above about 1.6 is the flat magenta core.
- **Colour.**
  - Red and blue are smooth functions of the field read 13–18 px to the right: the capture's
    composite chroma offset. With one shared 15 px offset and 32x32 tables over (shifted G, H), red
    fits to 7.5 rms and blue to 6.9.
  - Green carries the rings: green = base + amp × RING(s), 13.4 rms. RING is the measured 512-entry
    ring waveform; a strictly periodic one fits much worse (22.7 rms).
  - Junctions: a dark-centred green X between vertical neighbours, at (G max, H min); a cyan X
    between horizontal ones, at (G min, H max).
- **Mechanism.** A horizontal and a vertical oscillator are summed into a colourizer. Green folds
  the sum into rings. Red and blue are shaped levels of the same oscillators, shifted by the
  capture's chroma delay.

## What it does
- **Per field, G' table.** One table of G' = (gain × GE + gain × pairs × GO) >> 8 over 1024 phases of
  the two-diamond period. Diamond mode uses a triangle for GE.
- **Per line.**
  - H comes from 256 nodes, linearly interpolated; then H' = gain × H.
  - H' selects a row pair of the 32x32 colour table. The line engine interpolates those rows (4-bit
    fraction) into a 64-entry line table: 32 G nodes plus midpoints, holding red, blue, base and amp.
- **Per pixel.**
  - G' is read at the ring phase p and at the colour phase p + Fringe, plus the source bend.
  - s = G' + H' indexes the ring ROM; the shifted G' indexes the line table.
  - green = base + amp × ring / 64, clipped. Warm swaps red and blue.
  - RGB is converted to 10-bit YUV (luma pre-boosted 16/13), then the output conditioning and the
    blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Columns | diamonds across 16:9, 2–16 (7.32 = photo) |
| K2 Rows | diamond rows down the frame, 1–10 (4.14) |
| K3 Pairs | the big/small core alternation (half-frequency part of the horizontal oscillator), 0–400 % (100 % = photo) |
| K4 Drift | the lattice slides along its diagonal; bipolar, still at centre ±8; default 0.5 px/field up-left, ~8 s per diamond |
| K5 Bend | how far camera luma / Osc bends the rings and colours (default 39 %) |
| K6 Fringe | colour offset against the rings, 0–64 px (15 = photo's chroma offset) |
| S7 Saw | vertical oscillator becomes a sawtooth: diamonds turn into stacked chevrons |
| S8 Hard | hard-clipped rings |
| S9 Shape | Lens (photo) / Diamond: straight-sided cores |
| S10 Palette | Neon (photo) / Warm: red and blue swapped, an amber/orange lattice |
| S11 Source | Video (default) / Osc |
| P12 Cores | oscillator gain 0.55 → 1.0 → 2.2: 0 % = no cores, soft cool rings; 50 % = the photo; 100 % = cores and junctions clip into a flat magenta/green/blue argyle |

Glides: K1, K2, K3, K5 and P12 move 1/8 of the gap per field (deadband 4).

## Video
- **How the camera enters.** Camera luma is added to the horizontal oscillator, in both the ring
  phase and the colour phase. The rings and their colours bend around the picture, and the lattice
  still reads at the default Bend.
- **Osc mode.** Two slow drifting triangles stand in for the camera: about 0.9 cycle across, about
  1 cycle down, amplitude 1.25 tx + 0.75 ty. They feed the same path, and Bend stays live in both
  modes.
- **What it never does.** The video tap is luma into geometry only; it never reaches the output.

## Engine
- **Per field (vblank zone).**
  - Glides, then one serial shift-add multiplier over 10 slots: phase increment, phase per virtual
    px, line-start phase, fringe phase, row step, drift squared, drift x/y, gain × pairs.
  - The gain curve is shift-add, one add per clock.
  - The G' builder runs from zone clock 400, two clocks per phase on one split multiplier, writing
    two identical 1024x13 RAMs (no_rw_check).
- **Per line.** Kicked at the source avid rise, it computes the NEXT line in about 390 clocks, which
  fits SD's 858: H interpolation and gain on a serial multiplier, then the 32-node line table into
  the other bank of a ping-pong RAM (no_rw_check). Line 0 is computed in the vblank zone.
- **Pixel path.**
  - The G' RAM pair, bend, s and table column, ring ROM and line RAM (each with a fabric copy),
    amp × ring, green, the 2-term RGB→YUV matrix, clamp.
  - Then the conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to 13/16. Then the gate
    on the generated avid.
  - Latency is 24 clocks in every mode.
- **Shell.** Heirloom v0.2.0: virtual 1080-line coordinates, generated avid, vblank zone, measured
  width.
- **Memory.** 23 EBR:

  | Block | EBR |
  |---|---|
  | GE 512x12 | 2 |
  | GO 256x12 | 1 |
  | H 256x12 | 1 |
  | ring 512x8 | 1 |
  | colour table 1024x32 | 8 |
  | G' RAMs, 2 × 1024x13 | 8 |
  | line RAM 256x32 | 2 |

- **Fit history.** The first cut, with per-pixel gain multipliers and a 128-entry quarter-step line
  table, came to 8126 LC (105 %). Ablations put the per-pixel multipliers at 1595 LUT, the 4-lane
  table builder at 988 and the colour matrix at 471. Moving to per-field G' RAMs, a 64-entry
  midpoint table and 2-term constants brought it to 6337 LC.

## Status
- v0.5.0 built 2026-10-21 with router2. **HD Dual needed seed 2** (seed 1 missed timing); all other
  configs are seed 1.
- Fmax, post-route: HD Analog 79.69, HD HDMI 80.36, HD Dual 80.91, SD Analog 79.50, SD HDMI 83.94,
  SD Dual 73.90 MHz (SD needs 27).
- 6334–6341/7680 LC (82.5 %), 23/32 EBR. iceunpack OK.
- RTL vs integer model: 7.4 mean absolute difference at 720p (drift phase and edge pixels); motion
  verified. **NOT flashed**; presets are placeholders.

## Deviations from the photo, and why
- **Left edge.** The sync tile (first ~60 px: magenta slivers, teal blobs where the line-start
  oscillator restarts) is not reproduced; it would not fit as a simple restart, so the lattice runs to
  the edge. The 3-px black border and the 0.1 % tilt are dropped.
- **Colour model limit.** The green junction X is a little softer and the outer rings have a little
  less contrast. One smooth colour table plus one ring waveform reaches 13 rms on green.
- **Brightness.** The required HD HDMI conditioning scales luma to 13/16 (whites about 777), so the
  near-white ring peaks are compressed and the picture is a little darker than the capture. The
  2-term colour constants add at most about 2 % chroma error.
- **Osc plateaus.** In Osc mode, where the triangle's slope cancels the oscillator's rounded extreme,
  a junction flattens into a short plateau. Video does this organically.
- **Interlace.** Interlaced modes draw the same picture in both fields, at half vertical resolution.
- **SD.** On 4:3 SD the diamonds keep their shape, so 5.5 fit across instead of 7.3.
