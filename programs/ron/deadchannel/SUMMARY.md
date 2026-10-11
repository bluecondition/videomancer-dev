# Dead Channel — a dying analog signal chain

**Status: v1.5.0 FINAL except presets — signed off by Ron on hardware, 2026-10-09** ("Deadchannel can be finalized"). The remaining job is capturing presets with `./capture_preset.sh`.

## What it does
Processing effect. Three cicero programs (vhstrack, chromaticab, crt) are folded into one signal path, in the order the artifacts really happen:
- **Tape:** a bounded skew pull, random glitch dropout bands, hiss, and chroma drift.
- **Beam:** U and V are read at opposite X positions, and the split grows toward the screen edges.
- **Tube:** a scanline or RGB-triad grille, a vignette, and phosphor glow.

## How it works
- **Line buffers:** Y is buffered full-rate at 10 bits. U and V are buffered half-rate at 8 bits (neutral 128 expands to exactly 512). Each is a dual bank toggled at hsync, 18/32 EBR in total.
  - Every output line replays the previous input line, so all the displacement (skew, tear jog, misconvergence, drift) is just a read address.
- **Skew (P12):** the row offset is a BOUNDED position function, not a cumulative shear:
  - an exponentially decaying top-of-frame flag (`bend - bend>>k`);
  - plus a second pull, re-seeded where the head-switch band starts;
  - amplitude up to ¼ of the width;
  - the result wraps around the frame width.
- **Frame anchor (v1.3):** fires on the first vsync EDGE after active video, guarded by a saw-active flag, with a 1200-active-line ceiling outside any vsync condition. Field parity is sampled during active video.
- **Controls:** every control is decoded free-running every clock, one op per stage; none depends on a frame event.
- **Dropout events (v1.4):** the band is an event, not a roll. Each frame it is idle there is a chance it appears, at a random height (rand8 × height / 256) and a random size (½..15/16 of the K1 maximum). It holds 1–2 frames below half travel, 1–4 to ¾, 1–8 above, scrolls 2–32 lines/frame in a random direction, then vanishes. A second 23-bit LFSR feeds every draw; all draws are free-running and the anchor only samples them.
- **K1 response (v1.5), exponential:**
  - Per-frame hit chance = 2^(12k−12) (32-entry table); none in the bottom 3%.
  - Band half-height = 0.12·10^(2k−2) of the raster (16-entry table × measured height).
  - Per-line slip: ±32 px below 25%, ±64 below 50%, ±128 above (HD).
  - Roughly: 10% ≈ 1 thin (~6-line) event per 30 s; 25% ≈ 1 per 8 s; 50% ≈ 1/s, ~26 lines; 75% ≈ 7/s, ~58 lines; 100% = every frame, ~190 lines on HD.
- **Band glitches (v1.4):** each band line draws its own recipe at hsync:
  - block pixelation of 4–64 px (read-address hold, so chroma follows), on 5/8 of lines;
  - luma posterize to 8 or 4 levels, on half;
  - U/V swap, on half;
  - a negative line, 1 in 8;
  - a pure static line, 1 in 8 (Tear only).

  White dropout comets (~1 per 1024 px, 8–71 px, fading tail) run on every band line, on top of the existing ±128 px per-line slip. Flash runs the same glitches on a bleached, colourless bar.
- **Per-line flags:** band and scanline flags are latched at hsync, so the last pixels of a line still in the pipe keep their own line's flags.
- **Vignette:** distance is `max(|dx|, dy') + min/2`, where dy is scaled by the raster aspect (×1.75 for 16:9, ×1.5 for SD, ×2 again for field lines), so the falloff follows the screen shape.
- **Darkening desaturates:** each 128 codes of vignette or scanline dimming halves the chroma deviation (max 3 halvings, plus one for Faded).
- **Luma:** clamped to 64..940 on the output path.
- **Hardware contracts:**
  - neutral blanking gate;
  - 14-clock sync latency in every mode (13 before v1.4);
  - per-line accumulators advance on active lines only.

## Controls
- **K1 Dropout:** random glitch-band events: how often, how big, how long and how fast they scroll. 0 = no band at all.
- **K2 Hiss:** tape noise on luma.
- **K3 Drift:** scrolling chroma wobble (depth and rate together).
- **K4 Converge:** bipolar beam misconvergence; centre = converged.
- **K5 Glow:** phosphor bleed trail.
- **K6 Tube:** grille depth plus vignette (floored so the Grille switch always reads).
- **S7 Grille:** Scan / Triad. At 1080 lines Scan darkens 1 line in 3; at 720p and SD, every other line.
- **S8 Cast:** Cool / Sick.
- **S9 Chroma:** Rich / Faded.
- **S10 Band:** Tear (glitched video) / Flash (glitched, bleached light bar).
- **S11 Invert:** full video negative (luma 64↔940, chroma about 512).
- **P12 Skew:** bipolar bounded tape pull (headline control).

## Presets
Default, Dead Air, Good Tube, Head Switch (hand-typed; recapture from hardware once approved).

## Build seeds
- v1.5: **HD Analog seed 2** (seed 1 missed timing); the other five pass on seed 1.
- v1.4: **HD Analog seed 2, HD HDMI seed 2** (seed 1 missed timing); the other four pass on seed 1.
- v1.3: **HD Dual seed 2** (seed 1 stalled router2); the other five pass on seed 1.
- v1.2: HD HDMI seed 2.

## History
- **v1.0 (2026-08-24):** HW-REJECTED ("just diagonal streaks").
  - `sat10` folded every value ≥512 through `resize(signed,10)`, so neutral chroma became green.
  - Skew was a cumulative per-line DDA, which shredded ~92% of the slider travel.
- **v1.1:** both fixed; HW-REJECTED. The picture was processed, but every knob was dead and a grey bar scrolled and flashed. Every parameter was latched in the one frame-anchor branch, which never fired on hardware.
- **v1.2 (2026-08-27):** all controls decoded free-running, plus a line-ceiling backup. Built; HD HDMI needed seed 2. Never flashed.
  - Review on 2026-10-09 found the anchor was still level-sensitive (`if vsync_n='0' then clear elsif avid…`), with the ceiling inside the `elsif`.
  - A vsync reading low across active video would still have killed the Skew top flag, band roll and drift.
- **v1.3 (2026-10-09):**
  - edge-triggered, saw-active-guarded frame anchor;
  - legal luma clamp, and Invert as a full negative;
  - darkening desaturates chroma;
  - aspect-scaled round vignette (was a diamond);
  - 1-in-3 scanlines at 1080 lines;
  - band and scanline flags latched per line.
  - Timing-closed on all 6 configs: HD Analog 80.0, HD HDMI 76.4, HD Dual 79.7 (seed 2), SD Analog 74.7, SD HDMI 79.8, SD Dual 77.1 MHz.
  - ~4660 LC (61%), 18/32 EBR.
  - Awaiting hardware. Test order:
    1. S11 Invert (unmissable).
    2. The band rolls smoothly with no flashing (proves the anchor fix).
    3. K1 at 0 removes the band.
    4. Sweep P12 both ways (the top-of-frame bend must appear).
    5. Check on both HD Analog and HD HDMI.
- **v1.4 (2026-10-09):** v1.3 on hardware: the band rolled smoothly, but Ron preferred v1.1's erratic flashing and scrolling bar, and wanted the band to glitch the video rather than be solid static.
  - The band is now random events: it jumps in, holds and scrolls, then vanishes.
  - Band lines carry per-line glitch recipes plus dropout comets.
  - An extra pipeline stage brings the latency to 14 clocks.
  - Timing-closed on all 6 configs: HD Analog 79.6 (seed 2), HD HDMI 76.1 (seed 2), HD Dual 76.7, SD Analog 78.9, SD HDMI 82.5, SD Dual 81.3 MHz.
  - ~5090 LC (66%), 18/32 EBR. Awaiting hardware.
- **v1.5 (2026-10-09):** Ron asked for wider K1 triggering: subtle until ~50%, then frequent and large at the top. The hit rate, band size, hold and per-line slip now follow exponential curves.
  - Timing-closed on all 6 configs: HD Analog 78.4 (seed 2), HD HDMI 79.0, HD Dual 80.7, SD Analog 80.1, SD HDMI 77.2, SD Dual 74.4 MHz.
  - ~5400 LC (70%), 18/32 EBR.
  - HW-validated and signed off on 2026-10-09.
  - The default K1 (29%) now gives a rare event, about 1 per 6 s.
