# Candy Hills — side-scrolling striped candy landscape

**v1.0.0 (2026-09-28): finalization build — timing-closed, awaiting a hardware check.**
Build: all 6 configs at the full 74.25 MHz clock with **router1**, **all
seed 1**. HD Analog 77.9, HD HDMI 79.5, HD Dual 76.4, SD 76.7–78.7.
**7469–7482 LC (97%)**: the design is at its ceiling, so remove logic before
adding any. 24/32 EBR. vmprog 485947 B. Rebuild with
`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=600
./build_programs.sh ron candyhills`.
The last flashed look was v0.18. The v0.19 crest-lit shading, the v0.19b contrast
punch and everything below are new to hardware.

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
  - Slots are 128/64/32 px per layer.
  - Groups of 3 or 6 slots are placed by a 4-bit S-box permutation of the
    region index, so K6 density d lights exactly d of every 16 regions.
- **Sky:** a per-line luma gradient (744 at the top, rising ~120–145 toward
  the horizon, scaled by the measured height). K3 tints it.

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

## Watch on hardware
- **Clipping on bright palettes:** Lemon/Berry reaches luma 983 in the lit band
  (Lavender/Lime 907), above the ~768 comfort cap. If crests blow out to
  white, scale the lift by (1023 − y).
- **Colour lines:** thin coloured lines over coloured areas on an analog source
  would mean U/V line pairing. The fix is cubist's program-generated avid.
- **Sim caveat:** the pre-roll and post-roll need 128 blanking clocks, but the
  image tester's fake hblank is 64, so sims shift x. Real SD has 138 and HD 280.
- **SD only:** interlaced modes draw the stripe diagonal per field line, so it
  looks steeper than 45°.
