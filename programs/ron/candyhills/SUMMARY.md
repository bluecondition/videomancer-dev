# Candy Hills — side-scrolling striped candy landscape

**v1.1.0 FINAL except presets: signed off by Ron on hardware, 2026-10-11 ("good to finalize this and move to ready").**
Build: all 6 configs at the full 74.25 MHz clock with **router1**, **all
seed 1**. HD Analog 82.5, HD HDMI 87.8, HD Dual 87.6, SD 82.7–84.0 MHz.
6725–6759 LC (88%, down from 98%), 26/32 EBR. vmprog 468837 B. Rebuild with
`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=600
./build_programs.sh ron candyhills`.

**v1.1.3 (2026-10-17): resolution scaling** (not yet flashed). The raster
counters run in virtual 1080-line units, stepped per pixel / per field line
from the video timing ID (`registers_in(8)`): 1.0 at 1080p, 1.5 at 720p, and
at SD 2.0 across (a 1440-wide 4:3 window of the landscape) and 1080/486 per
frame line. Hill amplitude and wavelengths, stripe width and the 45° angle,
round bushes, crest line, lit band, shade fades and the sky gradient now keep
their proportions at 720p and SD (before, SD hills ran off-screen, bushes were
2:1 ellipses and stripes lost their 45°). 1080p output is bit-identical to
v1.1.2 (image-tester pixel diff = 0). Details:
- x: Q.1 accumulator `xa`; crest phase accumulators widened to 21 bits
  (2x units) so the per-pixel step `STEP*xs` is exact at 1.5x. Pre/post-roll
  = 64 virtual px (64/43/32 clocks at 1080/720p/SD; SD now needs 64 blanking
  clocks, not 128).
- y: Q.8 accumulator `pya`; the bottom field starts half a step down. The
  45° diagonal accumulator adds one 1080-line step per virtual line crossed
  (1..5 adds in hblank after each hsync). `act_h` is latched from top fields
  only, so the baselines don't twitter between fields. The old per-height
  sky-gradient classes are gone (py is always 1080-line units).
- Bush anchor write fires when x CROSSES the slot centre (a 1.5/2 step can
  skip the exact value).
- Bush outline: mid/bg rings are widened to at least one field step at 480i
  / 576i / 480p (inner radius² per mode), so an SD field can't skip a dome's
  dark cap. HD modes keep the old constants.
- Build (router1, all seed 1): HD Analog 81.8, HD HDMI 85.4, HD Dual 85.4,
  SD Analog 90.4, SD HDMI 88.1, SD Dual 85.2 MHz. 6832–6875 LC (89.0–89.5%,
  +~110), 26/32 EBR. vmprog 471306 B. Same build line as above.
- Verified with full-raster renders (.scaling_work, real blanking): 1080p 0
  differing pixels vs v1.1.2; 720p = 1080p scaled (r 1.00); SD = the left
  4:3 window of the 1080p frame (r 0.97 at a matched vsync count).
- Sim-only artifact (pre-existing, not changed): the P12 glide's
  `resize(signed(...), 10)` folds 512 to 0 in GHDL simulation (synthesis
  truncates, so hardware holds 512), so every sim restarts the amplitude
  from 0 and climbs about 1/8 per vsync. A 480i capture has had twice the vsyncs
  of a 1080p one at the same warmup and shows taller hills. Compare SD at
  half the warmup.

v1.0.x was iterated on hardware through 2026-09-29 (bush shiver fixed by the
integer world grid, defaults set). v1.1.0 confirmed on hardware, including
S10 Video Sky through its new EBR-ring input path. Defaults come from Ron's
notes; the 4 presets are hand-written, so capture real ones with
./capture_preset.sh.

## What it does
A Tiny-Wings-style landscape that scrolls forever and never repeats. Three
parallax layers of big rolling hills are filled with bold 45° candy stripes.
The stripes are painted onto the hills, so they scroll with them. Round green
bushes sit in random clumps along the crests. The crests are sunlit and the
hills fade into shaded valleys under a graded pastel sky. Back layers are
hazed, finer-striped and slower.

**P12 Hill Amplitude is the headline slider.** At 0% the hills are flat bands.
At 100% the ridges sweep off the top and bottom of the frame, alternating
between all-hill and all-sky views. The gain is 7-bit, about 6 px per step at the tallest peaks, and it glides (eases 1/8 per frame), so
a slider throw rolls the terrain up smoothly and pot noise doesn't shimmer.

## How it works
It streams with no frame buffer, and only the input sync drives the raster
(the picture is used only by S10 Video Sky).

- **Crests:** each layer's crest = baseline − gain·(sinA + sinB/2). sinA is a
  very long primary sine (~5000/6800/9500 px, several screens) and sinB a
  shorter secondary. Both come from per-pixel phase accumulators, so there is
  no x·freq multiply. The crest is not clamped, so it runs off-screen. The
  frontmost layer covering the pixel wins; if none covers it, the pixel is sky.
- **Scroll:** K1 runs 1–64 sub-steps per frame (1/8x–8x). Each sub-step adds
  exact integer constants to every layer's crest phase (7 fraction bits) and
  world-x (1/128 px). Hills, stripes and bushes therefore stay locked at any
  speed. S11 adds the two's-complement constants instead, with one adder
  either way.
- **Stripes:** world-anchored.
  - **x term:** x′·F/16, where F has 4 fraction bits. That's seamless across
    the 16384-px world wrap for any integer F.
  - **y term:** a per-layer per-line accumulator with the same slope, which
    gives exactly 45°.
  - **Crest fan (K2):** warps world-x *before* the stripe multiply:
    x′ = x + cos(winner's primary crest angle)·fan. So dx′/dx = 1 − k·sinA
    follows hill height. At max, k = 0.86 fg / 0.63 mid / 0.45 bg, so the
    stripes swing about 28–82° and never fold.
    - Because it's a warp, the effect is relative to stripe width. The old
      additive phase fan all but vanished on narrow stripes.
  - **Soft edge:** a constant ~8 px at any width.
  - After ~10 hardware rounds, every screen-anchored fan variant slipped
    against the scrolling hills.
  - **Pipeline:** s6b cos·fan, s7 x′ add, s7b x′·Feff split into lo/hi operand
    products (mod 2¹⁴) with the y terms pre-summed, s8 one 3-operand add.
- **Shading:** the top ~15% of frame height below the local crest gets +120
  luma. Past that band the lift fades while a multiplicative shade ramps down
  to 37.5% brightness. Only stripe pixels are shaded.
- **Bushes:** a stateless per-pixel circle test, dx² + dy² ≤ R², with an
  annulus outline, clipped at the crest. dy is measured from the **exact crest
  under the dome centre**, read from a per-layer anchor store:
  - Each scan writes the crest at every slot centre into one 256×16 EBR per
    layer, indexed by world slot.
  - The next line reads it back. The crest depends only on world-x, so the
    previous line's value is exact.
  - The store is banked by scan parity, so a read never collides with a write.
  - Centres just off-screen are covered by running the crest pipeline 64 px
    past both edges in hblank: a post-roll after avid falls and a pre-roll
    from x = −64. The old slope extrapolation lifted or buried bushes near
    screen-left.
  - |dx| and |dy| are carry-free folds (dx about the slot centre at 63.5,
    dy as a ones'-complement absolute value), computed in s5b from a
    dedicated `anchor − py` subtract in s5a.
  - Slots are 128/64/32 px per layer.
  - Groups of 3 or 6 slots are placed by a 4-bit S-box permutation of the
    region index, so K6 density d lights exactly d of every 16 regions.
- **Sky:** a per-line luma gradient (744 at the top, rising ~120–145 toward
  the horizon, scaled by the measured height). K3 picks its hue.

## Controls
- **K1 Scroll Speed** (default 3x): continuous, 1/8x–~12x (1–95 sub-steps per frame). It never stops.
- **K2 Fan Amount**: stripe width and angle follow the terrain, up to about
  28–82° at max at any stripe width.
- **K3 Sky Tint** (default 100% = warm cream): 32-step pastel hue sweep, rose → lavender → periwinkle → sky blue → mint → lime-cream → warm cream, with a ±12 deadband.
- **K4 Palette** (default Bubblegum): Green/Magenta, Mint/Coral, Sunset, Mono, Bubblegum,
  Lemon/Berry, Aqua/Tangerine, Lavender/Lime.
- **K5 Stripe Width**: 64 exponential steps, 1024 px down to 66 px, each
  3–6%. A ±12-count deadband stops pot noise from sliding the stripes.
- **K6 Bush Density** (default 8/16): 0 = no bushes, up to 15/16 of regions.
- **S7 3rd Layer**: off removes the mid layer and its bushes.
- **S8 Distance Haze**
- **S9 Crest Line** (default on): dark outline on the crest in the darker stripe colour.
- **S10 Video Sky**: the input video shows through the sky.
- **S11 Reverse**: scroll direction.
- **P12 Hill Amplitude**: the headline control, default 50%.

## Hardware contracts
- `program_type = "processing"`: required because S10 reads the input picture.
  As a synthesis program the input chroma would read 0, a saturated green sky.
- **Blanking gate:** y/u/v are forced to 64/512/512 outside `avid`, at s10.
- **Vsync:** per-field work runs only on the first vsync edge after active
  video (the `frame_act` guard). Analog serration would otherwise re-scroll
  and latch `act_h` near 0, which gives an all-stripes screen.
- **Raw video stays shallow:** S10 Video Sky reads the input through a 16-deep
  EBR ring (2 EBR, read pointer LATENCY−2 behind the write pointer, plus one
  fabric register), not the FF sync pipe. Deep FF-carried raw video went green
  and noisy on HD HDMI in three other programs.

## Timing notes (v1.1, 98% → 88% LC, HD +5–9 MHz)
- `no_rw_check` on the anchor-store RAMs and the Video Sky ring, both
  collision-free by construction. The yosys log must show 0
  `emulate_transparency`.
- The fan-cosine ROM is addressed from a combinational copy of the winner
  select (`fang_c`), so the EBR absorbs `r5_cos` and `r6_cos` is a fabric
  register before the multiply. Before, the multiply hung off the RDATA pins.
- Bush geometry moved a stage earlier with carry-free absolute values (above).
- Stripe soft-edge compares in s9 (`r9_isB`/`r9_isA`), so s10 is flag muxing
  plus one glow add.
- Shading is one multiply, `y·(64 − shade) >> 6`, with `64 − shade` from s9.
- One kept copy of the amplitude gain per layer multiplier.
- Remaining critical paths are mostly routing: the s11 shade multiply and the
  s10 colour mux. The next big lever, if room is ever needed, is moving the
  48 hazed-colour registers (480 FFs + a 12:1 mux) into EBR (+1 stage).

## Watch on hardware
- **Clipping on bright palettes:** Lemon/Berry reaches luma 983 in the lit band
  (Lavender/Lime 907), above the ~768 comfort cap. If crests blow out to
  white, scale the lift by (1023 − y).
- **Colour lines:** thin coloured lines over coloured areas on an analog source
  would mean U/V line pairing. The fix is cubist's program-generated avid.
- **Sim caveat:** the pre-roll and post-roll need 128 blanking clocks at HD
  (86 at 720p, 64 at SD since v1.1.3), but the image tester's fake hblank is
  64, so default sims shift x. Real SD has 138 and HD 280.
- **SD (fixed in v1.1.3):** interlaced modes used to draw the stripe diagonal
  per field line, so it looked steeper than 45°.
