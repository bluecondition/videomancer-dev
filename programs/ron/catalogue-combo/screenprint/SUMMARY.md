# Screenprint — summary

Catalogue combo #77 (seed 20260922): halftone screens + multi-source mixing.
v0.1.0 was flashed and liked ("screenprint is good", 2026-09-12). v0.1.1 moved
the per-pixel barrel shift into its own stage for timing. v0.2.0
(2026-10-12) is the first finalizing pass, built from a review of v0.1.1.
The v0.1.1 sources are in git 3fa438d and `.screenprint_work/v011_backup/`;
its vmprog is `.screenprint_work/screenprint_v011.vmprog`, for A/B.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Angle | screen angle, continuous 0–90° (glided). With Spin on, it sets the spin speed instead |
| K2 Mix | 12–100% of the blend (now reaches a true 100%) |
| K3 Dot | Circle / Diamond / Square / Line |
| K4 Blend | Add / Screen / Multiply / Difference / Hard Light / Overlay, in six equal zones. Add is signed: ink darkens, paper lightens |
| K5 Ink | Black / Red / Orange / Yellow / Green / Cyan / Blue / Magenta. The ink's own brightness goes into the blend, and its colour tints its dots |
| K6 Gain | halftone contrast: how far the dots open with brightness |
| S7 Register | Tight / Loose: the colour layer is printed W/32 px to the right (60 px on HD) |
| S8 Invert | the halftone inverted (ink where the paper was) |
| S9 Spin | the screen turns about the picture centre. K1 sets the speed: ~4.5 min a turn at 0, ~16 s at 50%, ~4 s at 100% |
| S10 Colour | Picture: the picture's colour, and coloured ink tints its dots / Ink: a one-colour print (ink colour on neutral paper) |
| S11 Rosette | a second screen 30° round. Ink prints where either screen inks |
| P12 Size | the headline. The screen cell grows from 4 px to 160 px (exponential, glided), zoomed about the picture centre. At 100% about 12 giant dots span an HD line |

## Review of v0.1.1

Dead or gated controls:
- **S11 Rosette did nothing at the default angle.** Its second screen used the
  same x+y / x−y lattice as the 45° main screen. The "Pop Print" preset's
  rosette was dead too.
- **S9 Order only changed Overlay**, which was 1/8 of K4's travel. Add,
  Screen, Multiply and Difference are symmetric, so swapping the operands
  changes nothing.
- **K5 Ink only showed with S7 = Colour AND S10 = Ink.** S7 did nothing
  unless S10 was Ink (the audit's main finding).
- **K4 Add and Screen gave the same picture with black ink.** In both, ink
  shows the picture and paper goes white. The zones were also uneven:
  Multiply and Overlay got 1/8 of the knob each.

Rendering:
- Dots sat half a pixel off-centre. Cell coordinates ran −h..h−1, so a 4-px
  cell's dot was visibly lopsided.
- Size came only in octaves (4/8/16/32 px) and angle in three steps (0°,
  45°, 63.4°). The steep angle also shrank the cell by √5.
- The blend could output luma 1023 (it clamped at 959 above black), which is
  full-scale Y.
- Mix stopped at 97.7%.

Hardware contracts:
- No HD HDMI bar conditioning. Halftone dots are near-full-swing 1-px edges
  that move whenever the picture moves, which is the known trigger.

Headline:
- P12 was a plain wet/dry Mix that sat at 100% by default. The audit rated it
  weak.

## Tiered list

### Tier 1: built in v0.2.0

1. **New screen engine** (accumulator-pair projection, TECHNIQUES "rotation
   without multiplies"). Per screen, u += cos/cell and v −= sin/cell each
   pixel, and the row starts step by sin/cell and cos/cell each active line.
   Everything is mod one cell. A vblank sequencer (sine ROM, cell ROM, one
   serial multiplier, about 330 clocks) sets the steps. It centres the
   lattice on the measured picture centre, so a dot sits exactly in the
   middle. This gives continuous size and angle and symmetric dots, and the
   shapes are normalised per shape only (the per-cell shift table is gone).
   I checked it with an integer model at the RTL widths: cells 4.00 / 7.99 /
   160.06 px, angles within 0.05°, rosette +29.97°, and the centre pixel on
   a dot centre.
2. **P12 Size** is the headline: 4 → 160 px, exponential, glided, zoomed
   about the centre. 100% is a pop-art climax of giant dots.
3. **K1 Angle** is continuous 0–90°, glided, with ±2-count hysteresis.
4. **Rosette at +30°** (the classic K/C offset), so it shows at every angle.
5. **K4 has six distinct modes in equal zones.** Order folds in as Hard Light
   (the v0.1 "Ink over" overlay) and Overlay (v0.1 "Pic over"). Add is now
   signed, so it no longer duplicates Screen.
6. **K5 Ink has a Black zone plus seven inks with real Y/U/V** (U/V-swapped;
   Red, Orange, Yellow, Green and Blue are the scope-verified rainbow
   values). The ink's luma drives the blend, so Yellow ink in Multiply is a
   light yellow, not black-plus-chroma. The colour tints the dots whatever
   S10 is set to. Both S7 and S9 were freed by this.
7. **S10 Colour Ink** is now a true one-colour print: paper loses the
   picture's colour too.
8. **S7 Register** (new): a misregistered colour layer via a 256-entry EBR
   ring. The offset is W/32, so it is 60 px on HD and 22 on SD.
9. **S9 Spin** (new): the screen turns about the centre, and K1 becomes the
   speed. It is unipolar, so it never stops: at the default K1 it does one
   turn in about 16 s.
10. **Contracts:** [1 2 1] on Y/U/V plus luma scaled to white ~777 (the damier
    bar fix); the blend clamped at 876 above black (white 940 before the
    scale); zone hysteresis plus the label-INDEX default trick on K3/K4/K5;
    a hysteresis hold on K6 Gain, the measured width and the measured
    height (a 1-px width jitter would otherwise shake the centred lattice);
    per-field work on the vsync edge with the saw-active guard; and
    no_rw_check on the colour ring. Latency is 17 → 21 in all modes.
11. **Presets:** v0.1's three looks are remapped (Pop Print, Screen Burn,
    Line Overlay). Four new ones show the new controls: Newsprint, Off
    Register, Pop Art and Spin Rosette. The empty "Default" preset is
    dropped. All are hand-written placeholders.

### Tier 2: recommended next (needs your go-ahead)

- **CMYK separation mode.** Four screens at the classic 15° / 75° / 0° / 45°,
  each keyed from its own colour separation (C/M/Y from RGB, K =
  min(C, M, Y)). The overprint colour comes from a 16-entry table. This
  rebuilds the picture as a full-colour magazine/comic print, and at P12
  100% it becomes Lichtenstein dots. It would take a switch (S7 or S11).
  Estimate +1000 LC; it needs its own timing pass.
- **Soft dot edges.** A ~1-px ramp across the dot boundary instead of a hard
  threshold. It would clean the stair-steps on giant dots and the moiré in
  small, off-axis screens.
- **Two-ink register.** Draw the coloured ink as its own screen offset half a
  cell from the black key (one more DDA pair, no line memory). That is a
  true misregistered two-colour print, rather than the picture colour
  sliding.
- **Ink colour U/V parity**, only if thin coloured-ink lines show on HD HDMI.
  Drawn colour on a line whose hsync→avid parity is off flips hue (aurora's
  red lines). The fix is a per-line parity vote that swaps the ink's U/V,
  about 30 LC.

### Tier 3: optional

- SD scaling: the cell is in pixels, so SD shows the same pixel size, which
  is about 2.7× bigger relative to the picture. It could follow the
  measured width instead.
- Interlace: screens are drawn per field (unchanged from v0.1), so on
  480i/1080i sources the cells are 2× tall. Keying on the frame line would
  square them but adds 30 Hz twitter on fine screens.
- Bipolar Spin (direction), and a paper-stock colour (cream/kraft).
- Raw chroma is still carried ~14 FFs deep, as in v0.1 (the rollthebones
  HD HDMI green/noise lesson). Watch for it; the fix would be an EBR delay.
- Capture presets from the device.

## v0.2.1 (2026-10-13): grey bar on the left

On hardware there was a grey bar down the left side. The probable cause is
S7 Register. The colour layer is read W/32 px back (60 px on HD), so the first
60 px of each line had no colour to read, and v0.2.0 filled them with neutral
chroma: a colourless strip. Now the ring's read-back shift is 0 for the first
12 px (C_EDGE, past the capture's blanking-edge columns), then grows by 1 a
pixel up to W/32. The left edge holds pixel 12's colour, with no seam: the
same idea as the catalogue edge-fill address clamp. At the default settings
(Register Tight) I found no path in the code that makes a left bar. If it
shows there too, the cause is something else.

Built 6/6 with router2: HD 76.3 (seed 1) / 74.9 (**seed 2**, thin margin) /
82.9 (seed 1) MHz (Analog / HDMI / Dual); SD 74.1 / 77.8 / 72.3 MHz, all seed 1.
~6,530 LC (85%), 10 EBR. NOT FLASHED, NOT COMMITTED.

## Hardware checklist (v0.2.0)

1. Defaults: a 45° 8-px circle screen, Multiply, black ink. It should read
   like v0.1, with dots now centred, and slightly dimmer (white ~777). At
   power-on the screen glides in from 4 px and 0° over about half a second.
2. P12 sweep: the dots should zoom smoothly about the centre, from fine grain
   to ~12 giant dots across. Look for steps or jitter at rest.
3. K1: smooth rotation. S9 Spin: it turns, and K1 sets the speed.
4. S11 Rosette shows at every angle.
5. K4: six distinct looks. K5: are the ink hues right? Red should be red. If
   red shows as blue, the U/V convention is flipped on this input; tell me.
6. S7 Register: colour slides right of the dots (60 px on HD).
7. HD HDMI: no thick bars. Note any thin coloured lines with coloured ink
   (that is the Tier 2 parity item).

## Build

`./build_combo.sh screenprint` → `out/rev_b/ron/catalogue-combo/screenprint.vmprog`.
Regenerate the ROM constants with `python3 gen_tables.py` (it splices them
between the GEN markers in screenprint.vhd).
v0.2.0 (2026-10-12), router2, all six configs are real passes (no final-retry
completions):

| Config | Fmax | Seed |
|---|---|---|
| HD Analog | 77.6 MHz | 1 |
| HD HDMI | 77.5 MHz | **2** (seed 1 stalled the router) |
| HD Dual | 76.7 MHz | 1 |
| SD Analog / HDMI / Dual | 78.3 / 81.1 / 82.6 MHz | 1 / **2** / 1 |

6,467–6,492 LC (84%), 10 EBR (4 sine + 4 cell table + 2 colour ring). The
yosys `emulate_transparency` count is 0. All bitstreams pass `iceunpack`.
NOT FLASHED, NOT COMMITTED.
v0.1.1 was 4,446–4,513 LC, 0 EBR, HD 74.1–77.4 MHz. Most of the +2,000 LC is
registers: four screen DDAs, the squarer pipeline and the sequencer.

## Community port — router1 livelock fix (staged v0.2.3, 2026-10-15)

The staged copy (`programs/czartroniks/screenprint`, stock LZX SDK) hung the
stock router1 on 1–7 of the 8 configs on every seed 2–10, at only 6912 LC.
Cause: stage 5's distance sums added operands with SHARED low bits.
- A square's bit 1 is always 0, and every folded distance is odd.
- So yosys merged those FFs, and the adders got the same net on both carry
  inputs. Router1 ping-pongs forever on such a cell.

Fix (bit-exact):
- `qu + qv` is summed from bit 2 up. Bit 1 = 0 means the low two bits never
  carry.
- `bu + bv` is summed from bit 1 up, with the shared bit 0 as a single carry-in.

Results:
- The scan (`~/czartroniks_port/tools/dupin.py`) finds 0 such cells.
- Seed 1 builds all 8 configs; own HD 89.7 / 85.2 / 80.7 MHz.
- GHDL side by side against the staged v0.2.2: 0 mismatches over 24 frames at
  403x201. A mutant that drops the carry-in gave 6065 mismatches.

This dev copy now matches the staged v0.2.3, including the community edits: GPL header, IO-align pad (latency 24), czartroniks program_id and author.
- **HW-validated 2026-10-15** (Ron: "looks good"): the staged v0.2.3 stock-SDK build.
