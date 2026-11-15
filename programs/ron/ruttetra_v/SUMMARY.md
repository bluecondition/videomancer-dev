# RUTT ETRA VERTICAL — the vertical-raster sibling of ruttetra

Incoming video re-rendered as VERTICAL lines horizontally displaced by luma:
the ruttetra picture turned a quarter turn. Bright material rises (leftward)
out of a dark floor that sags right; glowing lines on black; hidden-line
terrain occlusion (each nearer line drops an opaque skirt to its right).
Same 12-control panel, defaults and presets as ruttetra v0.2.4.

## Engine — same-scanline candidate shift register

Displacement is one-sided (rightward from the source column), so a displaced
line lands LATER IN THE SAME SCANLINE. No picture store is needed at all
(ruttetra needs 24 EBR of rotating row banks; this uses 2):

- Every Nth column is sampled on every row. Luma -> contrast -> a smoothing
  IIR that runs DOWN the column -> displacement + 4-bit brightness.
- **Per-column state RAM** (256 x 32, 2 EBR): the IIR accumulator and the
  displacement the column drew on the previous row. Read address = the next
  column to be sampled (held since the previous sample, so the data sits in
  a fabric register when the sample arrives); write = the column sampled 5
  clocks ago. They never collide (`no_rw_check`).
- **Connected strokes**: each row draws the run from the previous row's
  position to its own (`lo`, `hi`, `span`), dimmed by its length (>T: 1/2,
  >4T: 1/4). The segment is prepared ONCE per sampled column in the write
  path, not per candidate per pixel.
- **Candidates**: the last 8 sampled columns ride an 8-deep shift register
  `{lo, hi, dim, bright, age}`, index 0 = most recent. Ages advance by the
  pixel's virtual step every clock and hold at 512+ (pure skirt, no wrap).
  A pushed candidate appears with age 0 on the clock the display pipeline is
  at that very column (display runs 7 clocks behind capture; minimum
  displacement 1).
- Per display pixel: 8 parallel `age - lo >= 0 and age - hi < T` tests;
  lowest index wins = nearest line = the hidden-line rule (no rotation stage
  needed: the shift register is already in recency order). Skirt = any
  candidate with `age >= lo`.
- Max displacement = 8 x spacing - 1 (capped 255), relief normalised to that
  window (x min(spacing,32)/32), exactly as ruttetra.
- Roll = first sampled column of every row at phase x spacing (phase steps
  once per field); that leftmost line fades in with the phase.
- The column filters and the previous-row displacement restart on the
  field's first active row, armed by any line without video (not the vsync
  edge).
- **Output conditioning built in**: thin vertical lines are near-full-swing
  pixel-rate edges on every scanline (the HD HDMI thick-bar trigger), so Y,
  U and V pass a horizontal [1 2 1]/4 before the blanking gate. Drawn luma
  is already capped at 768 by the palettes. If bars still show, the next
  lever is a wider chroma filter (CLAUDE.md, wrinkle).
- Latency 19 + 1 pad = 20 clocks, identical in every mode; blanking-gated.
  Video line colour reads source chroma 10 FFs deep (same depth as ruttetra).

## Resolution scaling

All horizontal geometry (spacing 8..71, ages, displacement <= 255,
thickness, roll) is in virtual 1920-wide pixels: x step 1 at 1080, 1.5 at
720p (alternating 1/2, restarted every row), 2 at SD (4:3 = 1440-wide
window). Stroke window >= ceil(x step). The smoothing shift is cut by 1
(480p/576p/1080i) or 2 (480i/576i) so its reach down the screen is
comparable.

Known differences / compromises:
- **Relief is shallower relative to its axis than ruttetra**: 255 px is 24%
  of the screen height there but 13% of the width here (the same absolute
  size). Raising it needs wider ages/displacement and more candidates.
- **Smoothing lags downward**: ruttetra cancels its IIR lag by reading
  ahead along the stored row; a filter running down the screen cannot read
  rows that have not arrived, so at high Smoothing the shapes trail down by
  up to ~2^s rows (max ~12% of the height at full CW) and Video colour is
  no longer registered with the relief.
- Interlaced SD: the three lowest Smoothing steps are all "off" (shift cut
  by 2); 1080i / progressive SD lose one step.
- SD lines are 2 virtual px wide minimum, so Thickness 1 and 2 look the
  same there (and at 720p).

## Status

**v0.1.1 (2026-11-13): built, not flashed.** Ron (HW, v0.1.0): "thin lines
could be just a bit brighter". The [1 2 1] output filter halves a 1-pixel
line (3/4 for 2 pixels), so strokes 1 / 2 pixels wide are now drawn x1.5 /
x1.25 ahead of it (`s_tgain`, on the shading factor: 16 -> 24 / 20; keyed
on the stroke's width in PIXELS, so SD T<=2 and 720p T<=2 take x1.5, SD
T3-4 and 720p T3 take x1.25). Pre-filter luma held at 1023, chroma at
+/-448, filtered luma held at the 768 cap. 1080p peaks: T1 416 -> ~543,
T2 592 -> ~724; T>=3 unchanged. Build seeds: HD Analog 75.22 (seed 1),
HD HDMI 74.28 (seed 1, thin), HD Dual 75.10 (**seed 4**; seeds 1-3
missed), SD Analog 67.59 / SD HDMI 75.11 / SD Dual 78.28 (seed 1);
5666-5673 LC (74%), 2/32 EBR. v0.1.0 source + vmprog in
`.ruttetra_v_work/`.

**v0.1.0 (2026-11-13): flashed by Ron (thin lines a bit dim).** 6/6 with the default build
(`./build_programs.sh ron ruttetra_v`, router2): HD Analog 76.35
(**seed 3**; seed 1 missed, seed 2 stalled router2), HD HDMI 76.68
(**seed 2**; seed 1 missed), HD Dual 76.52 (seed 1), SD Analog 76.71 /
SD HDMI 76.88 / SD Dual 69.01 (seed 1, Fmin 27); 5464-5488 LC (71%),
2/32 EBR.
Sanity sims only (480x270 default, NTSC dec 1, 720p dec 2): lines draw,
displace, connect and occlude on all three x-step paths.
