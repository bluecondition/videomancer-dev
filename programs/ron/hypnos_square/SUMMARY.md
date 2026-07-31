# HYPNOS — concentric-ring / bullseye generator (v0.1)

1960s op-art target (Vasarely / Bridget Riley / the *Vertigo* spiral). For every
pixel: distance from a freely-movable centre → gamma warp → band it. Everything
interesting is in the warp and the banding; the distance is just `max + k·min`.

The single behavioural pillar: **the centre travels ±2 screens off-frame**. On
screen it's a target; nudge the centre to the edge and the field becomes a
shallow arc; push it further and the rings flatten into near-parallel stripes.
That range *is* the instrument, so `dx,dy` are never clamped to the raster.

## Control map (6 knobs + 5 switches + 1 slider — the whole Rev-B surface)

| Phys | Control | Range / detent |
|---|---|---|
| K1 | **Center X** | bipolar ±2 W, detent = frame centre |
| K2 | **Center Y** | bipolar ±2 H, detent = frame centre |
| K3 | **Period** | exp: ~½ ring across the frame → alias limit |
| K4 | **Warp** | bipolar, detent = flat. CCW → infinite tunnel (rings pack to centre), CW → sphere dome (rings pack to edge). `p = K4/512`, gamma on the radius |
| K5 | **Metric** | detent = circle; CCW → diamond (L1), CW → square (L∞). `r = max + k·min`, one coefficient |
| K6 | **Duty** | band-A : band-B width, 5–95 %, applied after the warp |
| P12 | **Zoom** | exp dolly — fly into the rings; period scales so the ring rate stays ~constant |
| S7/S8/S9 | **Glide X / Y / Z** | arm a one-pole slew per axis (applied to the register, so CV steps drift too) |
| S10 | **Output** | Key (hard 1-bit) / Gradient (soft radial ramp) |
| S11 | **Animate** | off (static, default) / on (slow outward ring travel) |

Input video is **always-on**: its luma displaces the radius before banding, so
a live image ripples the rings. Feed black → uniform offset → pure generator.

### What the 12-control ceiling cost (vs. the original 16-control spec)
- No live COLOR — the pattern is **true-black / true-white** (the guardrail:
  contrast is the soul). Colour would need a knob or switch we don't have.
- **Glide time, anim rate, softness baseline, input depth are compile-time
  constants** — there is no spare register to "preset" them into; TOML presets
  can only set the 12 live controls.
- Warp is **linear** in the knob (`p = K4/512 ∈ [0,2]`), not the exponential
  `2^((K4-512)/300)` the spec implied — see timing notes. Full tunnel↔flat↔dome
  range is preserved; dome tops out at p=2 (moderate).
- The "circle" is a `max + 0.5·min` **octagon** — cheap, reads as round enough;
  a true circle would need a second rotated octagon (future polish).

## Renderer
Streaming, **no BRAM, no line buffer**, ~18 pipeline stages, 60 % LC. Core per
pixel: metric `max+k·min` → `log2(r)` (priority-encoder + mantissa LUT) → warp
`pe = p·(log2 r − LREF)` → `exp2` (mantissa LUT + barrel shift) = r′ → phase
`φ = r'·f` → `frac(φ) < duty`. `f` from Period·Zoom folded to one mantissa+shift.
Adaptive softening: local ring frequency `|Δφ/Δx|` drops the edge-hardness shift
so the pattern greys out near the alias limit instead of shimmering.

## Timing closure (the whole story — this fabric hates multiplies)
Target 74.25 MHz on all three HD modes. First routed pass was **58 MHz**; the
climb to 74+ was almost entirely about **multiplies and per-frame vblank cones**,
not the pixel pipeline everyone worries about. Lessons, in the order they bit:

1. **log2 / exp2 each need their own stage.** A priority-encoder+barrel-shift or
   a mantissa-LUT+barrel-shift+clamp in one stage was ~17 ns. Split → +10 MHz.
2. **Every un-split multiply becomes *the* critical path, one at a time.** A
   14×9 or 15×11 array multiply on this DSP-less HX4K is ~15 ns and
   *routing-dominated* (the operand net sprawls). Fix per multiply: split on the
   **wide** operand into two ~7–8-bit partial products + a combine stage, and
   trim the other operand to 8–9 bits. Did this for metric, phase, and warp.
3. **The real hogs were the vblank sequencer cones, not the pixel path.** A
   combinational multiply/exp2 feeding a vsync-latched constant (`s_kmet`,
   `s_p`, `s_fnum`) is STA-timed *every* cycle (the ziffern trap). The metric
   piecewise `×151/×105` was a 16 ns cone → replaced with `k=(1023-metric)>>2`.
   The warp `2^((K4-512)/300)` exp2 was another → replaced with linear `p=K4>>3`.
   These two swaps did more than all the pixel pipelining combined (63 → 82 MHz).
4. **Relay registers beat the placer.** A per-frame constant is *sourced* at the
   sequencer and *consumed* by a pixel-stage multiply far away; the placer
   anchors it near its source and the route eats ~2.7 ns. A 1-cycle registered
   copy (`s_kmet_d`, `s_p_d`, `s_fnum_d`) — harmless for a frame-stable value —
   lets the placer put the operand next to its multiply. **+6 MHz routed.**
5. **Kill a multiply outright where you can.** The key/softening `m·gain` became
   `m << shift` (power-of-2 hardness). Octave-stepped softness, but it removed
   the 4th pixel multiply and its routing.

**Routed result (seed 1):** HD Analog 81.8, HD HDMI 84+, HD Dual ~78 — all clear
74.25 with margin. SD modes ~80–87 (need only 27). 60 % LC, 0 BRAM, 0 EBR.
Build with default seed 1. **Not yet HW-tested.**

## Verify before HW
`hypnos_proto.py` renders a contact sheet of every signature state (target,
tunnel, dome, diamond/square, off-screen arc, stripes, duty, gradient,
alias-grey) from the exact fixed-point model — regenerate it after any change to
the metric / warp / phase math.
