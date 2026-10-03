# Drape — capture a scanline and drape it like fabric

## What it does
Processing effect. The picture passes through untouched down to the **Split** row. That row is captured, and every row below it replays the captured line: a perspective fan (or back-and-forth pleats) that leans, sways and fades to black. **Freeze** holds the captured line so Split can move while the held line keeps draping.

## How it works
- The Split row is written into three inferred 2048×10 BRAM line buffers (Y/U/V, 15 EBR). The buffers are initialised to neutral black, so they can never show green.
- **Chroma:** the encoder tells Cb from Cr by counting from hsync, but U/V labels follow active video, and the hsync→active-video parity can differ between the captured row and a display row. Each drape pixel keeps its own source U/V pair, and U/V are **swapped on rows whose parity differs from the captured row's**. Two earlier attempts failed on hardware: v1.1 never swapped (green/magenta cast), and v1.2 forced the parity on a single chroma read, which mixed Cb/Cr from different pixels (dark lines at colour edges). A Python model of the core's 4:2:2 chain confirmed the swap scheme is never worse than passthrough at colour edges. **Sim caveat:** the image sim has no 4:2:2 stage and can't show any of this. Judge drape colour on hardware.
- Each drape row reads the buffer back at `src = base + (x − centre)·s`:
  - **Fan:** `s = 1/(1 + e/256)` from a 13-step serial divider that runs in each hblank. Columns become straight lines converging on a vanishing point.
  - **Pleat:** s is a triangle wave in e (+1 → −1 → +1). Columns fan out, pinch, flip and return.
- Depth `e = eased(dy past Stretch) × gain(Spread)`. The gain has a 64 floor, a 1.5× linear slope, and a 13× steeper slope above 50%.
- `base = centre + lean × depth`, where lean = Twist + Sway (a downward-travelling sine), so one multiply serves both. Reads that go past the picture edges reflect back in.
- **Blur** is a one-pole IIR applied when the line is captured (so it is in source space and widens with the fan). Each channel is a one-pole filter; the ~27 px group delay is compensated in `base`.
- Rows count active lines only: a row advances when active video ends, resets at vsync, and the frame height is taken from the active-line count. Per-row values are recomputed in each hblank (~45 clocks) and sampled once per pixel.
- **Hardware contracts:**
  - Neutral blanking gate on every output path.
  - Fixed 10-clock latency in all modes.
  - The sway phase only advances once per field (guarded against serrated vsync).
  - Split is deadbanded and loaded once per field.

## Controls
- **K1 Split:** capture row, spanning the full picture height.
- **K2 Stretch:** where the fan begins, as a fraction of the distance from the split to the bottom. Every knob position does something.
- **K3 Fade:** fade length below Stretch (4 → ~1800 rows). 100% = no fade.
- **K4 Twist:** lean, centred at 50%, up to ±0.5 px per row.
- **K5 Onset:** 0 = the fan starts with a kink at Stretch; turning it up eases the fan in over as many as ~480 rows.
- **K6 Sway:** travelling-wave sway; amplitude and speed both rise with the knob. The bottom 1/16 of travel is still (the default is 0, and only the Wind preset moves).
- **S7 Blur:** soft captured line.
- **S8 Shape:** Fan / Pleat.
- **S9 Freeze:** hold the captured line. This is only visible when the source moves at the split row; otherwise live and held look the same.
- **S10:** unused.
- **S11 Bypass:** latency-matched passthrough.
- **P12 Spread:** fan strength (headline control). 100% runs a Fan drape into its vanishing point (s ≈ 1/64) and gives ~8 pleats in Pleat.

## Presets
Default, Long Drape, Short Tail, Top Clip, Horizon, Pleats, Wind (Sway), Tilt (Twist)

## History
- **v1.0 (2026-08):** built, never flashed.
- **v1.1 (2026-09-29):**
  - Fixes:
    - Added the blanking gate.
    - Twist no longer reads unwritten buffer (was a green wedge).
    - Low Split settings no longer capture a blanking line (was stale/green).
    - The fade now targets black at Y=64 instead of sub-black Y=0.
    - Split and Twist use the full knob resolution (were 9-row / 16-px steps).
  - Controls:
    - Stretch is relative to the split.
    - Fade has a no-fade end.
    - Soft W + Soft merged into Onset.
    - Mix → Sway, Soft → Freeze, Curve+ → Mirror.
    - Curved → Pleat.
    - Spread now reaches the vanishing point.
- **v1.1 on hardware (2026-09-29):**
  - Colours pushed to green/magenta.
  - Occasional thick horizontal glitch lines (a known platform-wide issue, not Drape-specific).
  - 99% LC.
- **v1.2 (2026-09-29):**
  - Fix: chroma is stored as raw 4:2:2 with the hsync-parity-matched read.
  - Mirror removed.
  - Blur is now one-pole (per chroma phase).
  - Sway is folded into Twist's multiply.
  - The source-mux stage is gone, and the fade runs on Y and C only.
  - 4029 LUT standalone (v1.0 was 4551, v1.1 was 4773); 11 EBR.
- **v1.2 on hardware (2026-09-29):**
  - Colours better.
  - Thin dark vertical lines at colour edges, running down the whole drape.
  - Constant waving (Sway defaulted to 250); Ron wants a still drape by default.
- **v1.3 (2026-09-29):**
  - Coherent U/V pairs plus a per-row U/V swap (replacing the parity-forced chroma read).
  - Sway defaults to 0 with a still zone at the bottom of K6.
  - Blur is a one-pole filter on Y/U/V.
- **v1.3 on hardware (2026-09-30):** bugs fixed, **HW-validated, finalized**. Presets are still to be tuned.
