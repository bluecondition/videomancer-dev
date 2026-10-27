# Haywire — analog glitch-box emulation

**Status: REWORK (Ron, 11 Oct 2026)**, moved to Rework in the Program Shelf report after v0.3.0. The rework direction is undecided; do not iterate without a new brief. v0.3.0 is timing-closed with pin-skew-screened HD seeds; its hardware verdict isn't recorded. v0.1.0 and v0.2.0 were seen on hardware (Ron's notes are in the version log).

## Brief
Ron: a video filter that makes incoming video look like it went through an analog glitch box such as the Tachyons+ DreamWeapon or the Mezkalin boxes. The signature look: a range of instability, snow and overdriven brightness on peaks, and spaced-apart vertical lines that snake, follow edges and radiate from them.

## Research (2026-10-11)
Text sources describe the boxes only as knob counts. The DreamWeapon gear page (tachyonsplus.com/dreamweapon) has ~16 demo stills, which show the look directly:
- rainbow echo bands repeating to the right of a hand's edge, each echo a new hue;
- fine echo lines copying a silhouette;
- wavy, luma-displaced vertical stripes over faces;
- long horizontal smear streaks in the source colour;
- torn and displaced bands, with blanking bars;
- posterized psychedelic colour (green faces, pink highlights);
- bright areas broken into white/cyan dashes.

## How it works
- **Echo lines (the core).** A per-scanline phase oscillator is HARD-SYNCED by the picture.
  - A delta comparator fires when the 4-tap low-passed luma has moved Trigger levels since the last sync, at least `3 + Q/8` px after it. Analog noise and fine texture therefore can't chain syncs.
  - Each sync resets the phase. The wrap points become flat lines about 7 px wide (phase < 7·f0, capped at 0.69 of the period) at fixed distances after every contour, so each echo copies that contour's shape.
  - The envelope starts at 1.5× full scale and falls linearly (Ring), so a train holds full brightness for its first third.
  - **Echo colour = the bright side of its edge**, read from the final picture (Natural or Psyche) at the sync point, with the luma lifted a quarter of the way to white:
    - a falling edge takes the brightest colour since the previous sync (a segment max-hold);
    - a rising edge follows the brightest of its first 8 px.
  - The step is `f0 + fb·luma/256` (Bend = FM), so echoes bunch in bright areas and snake.
  - **Chaos (K6):**
    - the base step is swapped for f0 ± f0/2 (± f0 above half travel) on random pixels;
    - the hit pattern comes from a column-keyed xorshift, restarted from one seed every line, so it is identical on every row and the lines stay coherent;
    - a quarter as many per-row hits add roughness (the column random xor a per-line key, so they are frame-stable too);
    - per echo and per 8-line segment, a random value drops the echo (up to 25%) or picks a ½ / 1.5× / 2× width.
  - Soft sync: the phase free-runs and syncs only relight the envelope.
- **Hold / move (v0.3).** The lines are not a free-running oscillator. They hold dead still, then now and then jump and slide, then freeze again.
  - **Every random that places a line is frame-deterministic:**
    - the column pattern seed;
    - the per-line randoms (`lr`, an xorshift restarted from `line_seed` at every frame anchor and stepped on active lines). These drive the segment dropouts and widths, the per-row roughness, the phase scatter, the picture jitter, the chroma slip, sync drops and the torn-line hue;
    - the wobble phase.
    With no new seed, every frame is identical.
  - **Each frame**, with a Chaos-set chance (16-step exponential table: never at 0, ~1/1000 frames at the bottom, ~1/120 at the default, 1/4 at the top), a MOVE starts:
    - `line_seed` snaps to a new random (dropouts, widths, scatter and tear all change at once);
    - then for 2..17 frames the column seed steps (1 px of pattern slide per frame), a global phase `phg` slips by a random ±1/8 period per frame, and the wobble advances;
    - then everything holds still again.
  - Still unsynced to moves (by design): snow, the hum-bar roll, torn-band events and the Psyche hue drift.
- **Why a delta trigger, not an edge detector.** A numpy proto showed that a 4-px derivative threshold barely fires on soft-focus faces. Level and delta crossing trace soft contours too: echoes radiate from each iso-luma step.
- **Picture path.**
  - invert, hum offset, Drive gain (×0.25..×4), clamp 64..940;
  - the excess over 700 sets snow density, as 4-px dashes with a random phase per line, 3:1 white (faint cyan) to black;
  - Psyche maps luma×2 (+ instability hue drift) through the 256-entry hue ROM, fading chroma in the darks;
  - Smear is a decaying max-hold that carries its colour.
- **Composite.** The lines are lerped over the picture: luma by r (8-bit), chroma by r>>2. Snow goes on top. Picture Dim scales the picture to `y/4 + 48` with chroma halved, under the lines.
- **Instability (P12).** Every term is continuous; s, s² and s³ curves come from the control engine.
  - per-line jitter, up to 63 px HD (s³);
  - phase scatter (s²) and a travelling wobble of the phase seed (s);
  - hum bar, ±127 luma, rolling;
  - torn-band events: chance (s³), random position, size and offset; skew per line, scroll, 1–4 frame hold. Torn lines drop sync, flip hue and add snow;
  - per-line sync drop (s³, up to 50%);
  - chroma slip of 2..20 px;
  - hue drift and a snow floor.
- **Rings (EBR).**
  - Ring A: 8-bit luma.
  - Ring Y: 10-bit luma.
  - Ring UV: 8+8-bit chroma, read at the Y address minus cs.
  - Sync ring.
  - The Y/UV read address is ring A's address delayed KD = 5 clocks, so the displacement stays exact across line changes.
- **Control engine.** One time-shared engine: 8 slots × 16 steps, each with a deadband, a glide on a free-running tick, a 17-point curve from a ROM in EBR and one split multiplier. Slot 5 computes fb = f0·bend/64. It replaced seven parallel interpolators, which overflowed the chip (9374 LC).
- **Hardware shell.**
  - generated, hsync-parity-locked avid; the gate requires both the generated and the source avid;
  - one latency (22 clocks) in every mode;
  - output conditioning: [1 2 1] on Y, [1 3 3 1] pair-averaged on U and V (4:2:2-safe thin coloured lines), luma scaled to white ~775;
  - edge-triggered, serration-guarded frame anchor with a line ceiling;
  - per-line accumulators advance on active lines only.

## Controls
| Control | Function |
|---|---|
| K1 Drive | luma gain 0.25×..4× (50% = unity); peaks blow out into snow |
| K2 Trigger | delta-sync step 160..8 (8-bit luma); up = more contours launch echoes |
| K3 Spacing | echo period 6..160 px |
| K4 Ring | echo train length 24..4096 px |
| K5 Bend | luma FM depth 0..4× (square law) |
| K6 Chaos | how often the lines jump and slide (still in between; never at 0), and how much they wander, break up and vary in thickness |
| S7 Colour | Natural / Psyche |
| S8 Picture | Full / Dim (picture at a quarter; the lines dominate) |
| S9 Sync | Hard / Soft |
| S10 Smear | Off / On |
| S11 Invert | Off / On |
| P12 Instability | 0 = rock steady → 100% = torn, rolling, scattered, snowy |

## Build (v0.3.0)
7122–7151 / 7680 LC (93%), 8/32 EBR, router2. The builder closes all 6 configs on seed 1 (HD Analog 82.6, HD HDMI 85.7, HD Dual 82.0, SD 82.9–86.5 MHz).

**Shipped HD bitstreams are HAND-INSTALLED** after a `tools/hdmi_pin_skew.py` screen (16 seeds for HD HDMI and HD Dual, 40 for HD Analog). Seed 1 failed in all three HD configs: HD HDMI data −0.56; HD Analog HSYNC +0.50 with data up to +0.58; HD Dual data −0.41.

| Config | Seed | Fmax | HDMI hs-clk | closest data | analog-encoder hs / data |
|---|---|---|---|---|---|
| HD HDMI | **2** | 86.9 | +1.64 | −0.86 | −0.39 / −0.06..+1.93 |
| HD Analog | **8** | 89.0 | −1.42 | −1.34 | −0.92 / ≤ −2.72 |
| HD Dual | **9** | 82.5 | −1.69 | −1.15 | +1.59 / −0.45..+0.83 |
| SD ×3 | 1 | 82.9–86.5 | — | — | — |

Earlier shipped seeds: v0.2.0 HD HDMI s14, HD Analog s34, HD Dual s2; v0.1.0 s12, s4, s10.

**Never full-rebuild for a TOML-only edit:** the builder would reinstall seed 1. Instead:
1. `rm build/programs/ron/haywire/{,rev_b/}program_config.bin`;
2. regenerate the config with the converter;
3. repack: `vmprog_pack.py --no-sign --hardware rev_b --toml-path programs/ron/haywire/haywire.toml build/programs/ron/haywire/rev_b out/rev_b/ron/haywire.vmprog`.

After any RTL change, re-screen.

## Version log
- v0.3.0 (2026-10-11), from Ron on v0.2.0: "the lines aren't really like a constant oscillator — they stay still for a while and then might move, then stay still, not in a consistent pattern … make it adjustable with Chaos."
  - Added the hold / move machine; all line-placing randomness is now frame-deterministic.
  - Before this, the column pattern crawled 1 px every frame and the segment dropouts were redrawn every frame (flicker).
  - Fit: 7336 LC (95.5%) at first. Derived-control registers became wires, but that put the fan-out of the Chaos knob into routing. Fixed with per-consumer `keep` copies and by simplifying the output chroma to [1 2 1] (lines are ≥ 3.5 px).
  - Result: 7122–7151 LC (93%), HD HDMI 75–87 MHz over 6 seeds.
- v0.2.0 (2026-10-11), from Ron's look at v0.1.0 on hardware: "not seeing enough prominent vertical lines … they shouldn't be rainbow lines, they should kind of match the color of what they're ringing, and should be a little thicker and more chaotic."
  - Echo colour now comes from the edge's bright side; the rainbow is gone.
  - Lines are flat and ~7 px wide (were sawtooth slivers), and the envelope holds full brightness for the first third of each train.
  - New K6 Chaos replaces Hue; S8 Picture Full/Dim replaces Rainbow/Mono.
  - Defaults: Trigger 700, Spacing 450 (~25 px), Ring 720, Chaos 380.
  - The line ROM port is gone (8 EBR). 7130–7150 LC (93%).
  - Timing fixes on this pass: the echo-luma boost moved a stage later; the control-engine deadband was split over two steps.
- v0.1.0 (2026-10-11): first build.
  - The first cut (seven parallel knob interpolators) came to 9374 LC; it was replaced by the serial control engine.
  - Timing: HD HDMI missed on all 6 seeds (best 73.5). Fixed four routing-bound paths:
    - trigger loop on registered ±Q band edges;
    - one-hot slot enables in the control engine;
    - drive multiply split over 2 stages;
    - smear split into a level loop and a colour stage.
    The two extra picture stages were absorbed with KD 7→5, so latency stays 22.
