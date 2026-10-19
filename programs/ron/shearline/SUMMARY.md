# SHEARLINE

**Recursive line-feedback smear.** Every scanline redraws the previous
*output* line, read at a sheared and fanned column, blends the live line into
it and writes the result back for the next line. Because each line redraws the
already-moved line above, bright detail streaks into diagonal comet trails
sliding down-screen. Processing program (`program_type = "processing"`).

## Controls

| ctl | name | what it does |
|---|---|---|
| **P12** | **Shear** | trail angle: 0 = straight-down drips → ~15 px/line flat streaks. Piecewise (half the travel below 3 px/line), 1/16 px/line resolution via a per-line Bresenham carry |
| K1 | **Fan** | bipolar, noon = parallel trails (±16 deadband). CW: each line redraws the one above zoomed out about the centre → trails spray outward (warp speed). CCW: converge into a tunnel |
| K2 | **Trail** | blend weight t, exponential: trail length doubles every 1/8 turn, ~4 lines → 1000+ (top 1/8 = t 1023) |
| K3 | **Slope Mod** | per-line random jitter added to the shear (0..31 px) → ragged, shimmering diagonals |
| K4 | **Spectrum** | chroma rotates a little on every recursion (0..0.25 rad/line), plus a luma-scaled +U seed so grey trails pick up colour too: trails age through the hues |
| K5 | **Mix** | dry/wet (100% = exactly wet) |
| S7 | **Direction** | Right / Left |
| S8 | **Blend** | Smear (lerp toward live: the picture dissolves into diagonal blur) / Comet (trail fades to black, brighter of live/trail wins: only highlights streak, picture stays sharp) |
| S11 | **Bypass** | dry at the same latency (Mix forced to 0) |

Presets (designed, not yet HW-captured): Drip, Comet, Warp Speed, Rainbow
Tail, Tunnel, Screamer, Left Sweep.

## v1.1.0 (2026-10-10): finalization rework

Hardware-contract and correctness fixes (v1.0.0 was never flashed):
- **Blanking gate**: output neutral (64/512/512) whenever the latency-aligned
  avid is low.
- **Field-start seeding replaces the clear sweep.** The first active line of
  every field takes t = 0 (pure live). This fixes three things at once: the
  Reset sweep cleared to code 0 (Y below black, chroma saturated green), Free
  Run wrapped the bottom of the frame into the top, and the clear FSM put a
  second source on every write port. The Frame switch is gone.
- **Off-line reads use the measured width**: `rd < 0 or rd >= W`, W = field
  max of the per-line avid count, copied at vsync behind a saw-active guard.
  Left mode used to read never-written columns 1920+ (power-up zeros = a
  green-black band streaking left).
- **Chroma trail follows luma**: U/V are stored per 2-px pair (odd pixel
  wins), so a plain `rd >> 1` read moved chroma by the shift rounded to even.
  At the default 1 px/line, Right-mode colour went straight down and Left ran
  at 2 px/line. The chroma reads pair `(rd - 1 + b) >> 1`, with b toggling on
  every odd-shift line, so it averages the luma rate.
- **Round-toward-zero blend**: `ref ± floor(t·|prev − ref| / 1024)`, sign
  applied after an unsigned multiply. The old floor of a signed product
  biased every recursion by −½ LSB: at long trails that is
  −0.5/(1 − t) ≈ −256 codes, i.e. trails sinking toward green/black. The
  result always lies between ref and prev, so no clamp is needed.
- Buffers power up black/neutral; all six banks carry `no_rw_check` (the read
  bank is never the write bank during active video; collisions only happen in
  blanking, where the data is unused).
- Categories were invalid ("Process", "Feedback") → Temporal, Glitch, Warp.

Engine:
- Read column = `rp − D_int`, registered before the EBR. `rp` is a counter
  loaded with ∓S at hsync. `D` is a 7.14 fixed-point fan DDA
  `k·(x − c) + g`. Its start value `−k·c` comes from a second accumulator
  `k·x` latched at the centre pixel of the previous line (no multiplier).
  `g` is a golden-ratio per-line dither, so small fans move sub-pixel instead
  of having a rounding dead zone ±0.5/k wide around the centre.
- Per-line DDAs (shear phase, dither, chroma toggle) advance on active-video
  fall and reset at vsync, so both interlace fields step identically.
- Spectrum is a Minsky circle step on the write-back path (E10–E15):
  `u' = u + seed − round(a·v)`, `v' = v + round(a·u')`. It is area-preserving,
  so it never runs away at long trails. Applying it at write-back means the
  output path doesn't wait on it.
- Multiplies: blend t×|diff| split 10×5 hi/lo; Spectrum 9×8 on the top 8
  chroma bits. Mix is a program-local copy of the same split-multiply
  shape: the SDK `interpolator_u`'s unsplit 11×11 multiply was the HD
  critical path in the first v1.1 build (HD Analog 74.66 MHz).
- Latency 14 in every mode (was 9).

## Status

**v1.1.0 FINAL except presets**: signed off by Ron 2026-10-10 and marked completed in the release audit, **not yet flashed**. All 6 configs closed on seed 1
(router2): HD Analog 81.0, HD HDMI 85.5, HD Dual 84.3, SD Analog 87.5,
SD HDMI 84.7, SD Dual 83.1 MHz. ~5240 LC (68%), 22/32 EBR, 0
emulate_transparency. Presets are designed, still to be HW-captured.

Watch on hardware:
- Thin coloured lines inside colour trails on analog sources: the stored line
  replays chroma captured at a different hsync→avid parity (drape's per-row
  U/V swap issue).
- HD HDMI green/noise: raw video now runs ~14 FFs deep to the output (the
  depth at which cascade/rollthebones went green). Same depth for bypass.

## v1.1.2 (2026-10-19): resolution scaling

The trails are a per-LINE recursion, so at 720p (each line 1.5 virtual
1080-line units tall) and above all at 480i (each field line ~4.44 units tall,
each pixel 2 units wide) the old per-line constants drew trails that were too
long and too steep: at SD the comets became near-vertical streaks over most of
the picture. Every per-line quantity is now defined per 1080p line and
converted per FIELD line from the timing ID (`registers_in(8)`; steps
free-running, as in mondrian):

- Shear and Slope Mod x ys/xs (px per field line: 720p 1.0, 1080i 2.0,
  480i 2.22, 576i 1.875). The Bresenham phase widens to 8 fraction bits.
- Trail: (1024 - t) x ys, Q.8; the fraction is carried line to line (reset at
  vsync, latched into t at hsync), so the mean decay per screen height is
  exact even where it is 1-2 codes/line (720p at long trails). Clamped at
  t = 0 (a 1080p 4-line trail is under one SD field line).
- Fan k and Spectrum a x ys (zoom / hue rotation per field line); the
  Spectrum seed octave follows the scaled a. The Minsky multiplies widen to a
  12-bit a (a reaches ~1.1 rad/field line at 480i Spectrum 100%).
- One serial shift-add multiplier (11 steps, round-robin over the 5 products)
  forms all of them; at 1080p every product equals the old value.

1080p is bit-identical to v1.1.1 (diff1080 = 0). SD now matches the 1080p
4:3 centre crop (trail angle and length); 720p matches 1080p.

Build (router2, `./build_programs.sh ron shearline`): HD Analog 86.50 (seed 1),
SD Analog 82.17 (seed 2; seed 1 stalled the router under machine load),
HD HDMI 87.84, SD HDMI 80.16, HD Dual 80.48, SD Dual 77.22 MHz (seed 1).
~5772 LC (75%, +530), 22/32 EBR.
