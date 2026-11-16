# Tweed (Hardline)

**v0.1.0 built 2026-11-15, not flashed.** Flash `out/rev_b/ron/hardline/tweed.vmprog`.

## What it draws

Woven cloth in black and white, drawn from a real weaving draft. The art reference is the colour-and-weave
effect: houndstooth (2/2 twill with 4 dark and 4 light threads), shepherd's check, glen plaid, log cabin,
pinwheels, hairline stripes and herringbone all come out of one textile rule:

    pixel = weave(i, j) ? colour(i) : colour(j)

i and j are the warp and weft thread numbers, `weave` is one of eight 8 x 8 drafts (1 = the warp thread is on
top) and `colour` is one of eight dark / light thread orders of period 16, used for warp and weft alike.
Nothing else is drawn: 64 cloths from two lists, doubled by the Point draft.

The default frame is a houndstooth, square to the screen, with threads 13.9 virtual px wide (17 motifs across).
Luma only: U = V = 512 on every pixel. Nothing moves by itself.

    (u, v) = R(turn) (x - pivot) / thread size + (D, -D)       threads; D = the camera field

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the cloth's pivot, -512 .. +511 virtual px from the screen centre (1 count = 1 px): pans the cloth, and is the point Turn and Period work about |
| K2 | Y | the same, vertically |
| K3 | Period | thread size, 3 .. 64 virtual px, exponential: 3 x (64/3)^(K3/1023); default 50 % = 13.9 px |
| K4 | Weave | 8 steps: Plain, Basket (2/2), Twill (2/2, default), Warp (3/1 twill), Broken (broken 2/2 twill), Wide (4/4 twill), Weft (1/3 twill), Satin (8-end, warp-faced) |
| K5 | Order | 8 steps, dark : light threads: 1:1, 2:2, 3:1, 4:4 (default), Glen (4:4 then 2:2 twice), Cabin (1:1 for 8 threads, then flipped), 1:3, 8:8 |
| K6 | Camera | how far the picture's luma pushes the cloth, 0 .. 100 %, default 100 % |
| S7 | Glide | Off / On (default On) |
| S8 | Draft | Straight / Point: the thread number used by the DRAFT reflects every 8 threads (the colours do not), so twills turn into diamonds and herringbone and plain weave turns its stripes into log cabin |
| S9 | Threads | Flat / Threads: every float gets a hairline along one side in the opposite tone (black on white threads, white on black), so the cloth reads as woven yarn. 1/6 thread wide, 2 virtual px at least |
| S10 | Weight | Bold = the cloth as drafted / Fine = only the outline of the dark / light figure, white hairlines on black (1/4 thread, 3 virtual px at least). With Threads on, Fine also draws the hairlines of the light floats, so the light areas are hatched |
| S11 | Source | Video / Osc (default Video) |
| P12 | TURN | rotates the cloth about its pivot, bipolar: centre = square to the screen (a +-6-count dead centre), +-90 degrees at the ends |

K4 and K5 take a new step only 8 counts inside it, with the two ends of the travel excepted. Glide On eases X,
Y, Period, Camera and Turn by 1/16 of the gap per field, never slower than one count per field, and lands
exactly. Glide Off uses the value read at this field start, with no easing and no extra field. Both sit behind
a +-1-count hold against pot jitter.

## Camera

The camera is never shown. Its luma becomes a displacement field D that pushes the cloth along its own
diagonal (u + D, v - D), so the cloth drapes over the picture: motifs bend, stretch and crowd where the picture
changes brightness.

- Luma -> x K6 -> box over 96 virtual px (96 / 64 / 48 pixels at 1080 / 720p / SD), reloaded at each line start
  -> one-pole vertical filter in a line store (1/32 per line at 1080p and 720p, 1/16 or 1/8 where the lines are
  taller; the first line of a field loads it).
- The displacement is fixed in PIXELS, not threads: white at K6 100 % moves the cloth 112 virtual px, at every
  Period. That is 8 threads at the default thread size. 25 % = 28 px (2 threads, a gentle swerve), 50 % = 56 px,
  100 % = 112 px. 0 %, or a black input, is exactly the clean cloth.
- The footprint used for anti-aliasing includes the field's own pixel and line differences, so edges stay one
  pixel soft where the cloth is squeezed, and squeezed threads fade to grey instead of aliasing.
- Osc substitutes the shell's drifting 640 x 540 triangle lattice at the same point at twice the level (up to
  227 px): diamond-shaped bulges travel across the cloth.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks", about 520 zone clocks): the +-1-count holds and glides
  (six clocks per channel), then one serial 16-step multiplier in 19 slots: thread size through a 2^(i/256) ROM,
  the Turn's sine and cosine from a 256-point quarter wave with linear interpolation, the four accumulator steps,
  the line-0 start values at the pivot, the camera gain and the hairline width.
- **Per line**: two adds. The cloth coordinates of the line start step once per ACTIVE line.
- **Per pixel** (37 stages): two beam accumulators (Q4.24, modulo 16 threads) -> + / - the camera field ->
  footprint w = |d/dx| + |d/dy| per axis -> which zone the pixel is in and the nearest zone across each axis ->
  two 3-step coverage dividers (17 levels per axis) -> the 3 x 3 cells around the pixel from three weave-ROM
  rows -> the tone of four blend corners -> bilinear blend (one 6 x 6 multiplier) -> fade to mid-grey as the
  footprint goes from 1/2 to 1 thread per pixel (one 10 x 5 multiplier) -> luma -> [1 2 1]/4 -> 13/16 toward
  black (white 776) -> blanking gate. The source path has one 8 x 10 multiplier (luma x gain).
- Chassis: the Kinetic riley shell (timing-ID virtual-1080 steps, generated avid with the parity-locked start,
  source field, luma conditioning), without its wave engine, palettes and chroma paths. One latency (40) in
  every mode. `field_n` passes through one register.

## Status

- Built 2026-11-15 through the shared queue with **router1**, every config on seed 1. Not flashed.
- Post-route Fmax: HD Analog 89.93 MHz, HD HDMI 87.59, HD Dual 84.63 (HD needs 74.25); SD Analog 82.69,
  SD HDMI 88.43, SD Dual 92.64.
- 6,961 .. 6,988 / 7,680 LC (91 %), 10 / 32 EBR. No carry self-adds (dupin: 0).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline tweed`. Router2 was not tried.
- Model = RTL (GHDL), pixel diff 0 on every render compared:
  - 720p60 full frame (921,600 pixels): the defaults with a black input; Source = Osc; bear.png at K6 100 %
    (the model fed the tester's own stimulus luma);
  - NTSC, both fields (720 x 243);
  - 720p60 / 2 with other settings: turn either way and at +-90 degrees, X / Y off centre, Period 0 %, 30 %,
    64 % and 100 %, five of the eight weaves and five of the orders, Point, Threads, Fine, Threads + Fine;
  - Glide On, 720p60 / 4: the frame 2 fields after a knob jump (the 1/16 ease) and 11 fields after a small one
    (the one-count-per-field branch) equal the model with the accumulators stepped that many times. The final
    landing step is the same code path as Glide Off.
- Renders in `.hardline_work/rb_tweed/`: `cmp_final.png`, `sheet.png`, `strip.png`, `camera.png`,
  `all64_straight.png` and `all64_point.png` (every Weave x Order), `presets.png`, `ntsc_cmp.png`.

## Cut / deviations

- **Colour orders have periods that divide 16**, so the cloth wraps on binary accumulators with no seam. The
  spec's 2:1 (period 3) and 1:1:2:2 (period 6) became 1:3 and Cabin, and 4:4:2:2 (period 12) became Glen,
  4:4:2:2:2:2.
- **The camera pushes the cloth 112 px, not 3 threads**, through a 96-px box, not a 32-px one. At 3 threads the
  bear was a faint wobble; a higher gain through the 32-px box shredded the motifs. Shredding depends on
  displacement divided by box width, so the displacement is held in pixels and looks the same at every Period.
- Default thread size 13.9 px (K3 at 50 %), not 12.
- The 1/3 twill is offset two threads against the colour order. At offset 0 every order of period 4 or less
  gave plain stripes.
- Point reflects the draft only. Reflecting the colours too turned 8:8 into a flat black cloth.
- X and Y move the pivot by 512 px either way, not by two screens: a lattice has no centre to lose.
- Osc is heavy at the default Camera 100 %: about half of the picture is stretched into streaks.
- **SD**: the same cloth in the same place (the centre 4:3), but a 480i field line is 4.44 virtual px tall, so
  threads under about 9 px (K3 below about 35 %) fade toward grey and are flat grey under about 4.5 px. At 720p
  the finest threads (K3 near 0) fade partly when the cloth is turned 45 degrees. 1080p holds every setting.
- At the smallest Periods (threads under about 6 px) the Threads and Fine hairlines take most of the thread, and
  the picture becomes an even grey mesh: regular, but low in contrast.
- With a black input K6 does nothing, and X / Y only shift a repeating cloth: their sheet panels look alike.
- Not verified on hardware: anything. The riley chassis it stands on has not been flashed either.
