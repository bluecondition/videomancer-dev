# NACRE — concentric circles, each carrying its OWN gradient (v0.1, WIP)

A dedicated fork of **Hypnos Cascade** whose entire subject is the shading.
The cascade motion is unchanged — move the centre and the innermost circle
leads, each outer ring holding until the one inside reaches it, then being
shoved outward, never overlapping. What is new is the **fill**:

> every ring's gradient belongs to **that ring**, measured between its own
> circle and the circle nested inside it — so it **squeezes** where the inner
> circle crowds it and **swells** where the gap opens, with smooth round
> contours morphing between the two circles as the cascade moves.

Formally the field is `f = d_in / (d_in + d_out)`, the true Euclidean
distances to the two bounding circles, triangle-folded to luma (dark at both
circle edges, bright mid-ring).

## Status — **fits, does not render correctly yet**

| | |
|---|---|
| Fit (hd_analog) | **7546 / 7680 LC (98 %)**, 12 EBR |
| Timing | 51.3 MHz routed — **needs 74.25** |
| Render | partial: horizontal streaks, geometry wrong |

This program is **not usable yet** and is committed as work in progress. The
reference look and the validated integer model live in
`../cascade/percircle_int2.py` (+ the `shading_*.png` / `percircle_*.png`
comparison sheets in that directory).

## Why a fork
The per-circle field needs ~4350 LC of knot engine. Cascade v0.9 measured
**11454 / 7680 LC (149 %)** with it — it cannot fit alongside cascade's mark
renderer, outer-field tracker and stepped modes even at 16 rings. Nacre
deletes all three and makes the seed stream the whole renderer.

## How it works
- **E_SPAN** (unchanged from cascade) computes each ring's two scanline edges
  and writes them to a span store.
- **E_HERM** walks the line's runs in ascending x — left edges outside-in,
  the innermost present ring's central run, then right edges inside-out —
  and evaluates `f` at **knots** (every run end plus a one-ring-spacing
  grid). Each knot costs two Euclidean distances (radix-4 sqrt plus a
  sub-pixel remainder correction — integer distances are visibly wrong) and
  two reciprocals, all funnelled through **one shared normalise unit** so the
  die holds a single priority encoder and barrel shifter.
- Between knots the field is **linear**, so the pixel path is `f += d` and a
  triangle fold — one add per pixel, no multiply, no divide, no tracker.
- **C_NC** chained discs (12 HD / 8 SD) plus **rigid outer rings** to C_NT
  (24 / 16) that share the outermost chained offset: fills the screen for
  free — no storage, just engine slots.
- The engine runs **C_NEARLY lines ahead** (2 HD / 3 SD) across four seed
  banks, because a line's knots do not fit in one line period.

**Latency contract** (violating it corrupts the field — three such bugs were
found and fixed): the shared multiply gives `m_p` at T+3; the radix-4 sqrt
needs 7 full stepping cycles and is readable at T+8; `p_norm` gives `hn` at
T+1 and `rrom_q` at T+2.

Because both numerator and denominator are normalised together, **every
post-multiply shift is a constant** (`f = m_p >> 1`, `d0 = m_p >> 2`, the
sub-pixel correction `>> 11`). Do not reintroduce a variable post-shift — the
earlier version did, which was both a ~250 LC barrel and an arithmetic bug.

## Controls
| Phys | Control |
|---|---|
| K1 / K2 | Centre X / Y |
| K3 | Period (ring spacing) |
| K4 | Steps — quantise the gradient to 2..64 levels (S9 = Steps only) |
| S7 | Glide |
| S8 | Cascade (Uniform / Cascade) |
| S9 | Ramp — Steps / **Smooth** (default) |
| S10 | Catch-Up (Stay / Home) |
| K5, K6, P12, S11 | spare |

## Next steps
1. **Debug the render.** Suspects, in order: per-line seed-slot overflow past
   256; the run walker's context roll; bank/`sp_dy` phasing. Probe by
   bit-banding `e_st` / `hm_slot` in `p_out`, or drop `C_NT` to ~6 so walker
   bugs separate from throughput.
2. **Close timing** (51 → 74.25 MHz): split `p_norm` across two cycles — the
   knot frame has idle slots to spare.
3. Only then build all six configs and package.
