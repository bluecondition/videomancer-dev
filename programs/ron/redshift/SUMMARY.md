# Redshift v2 — luma as mass

Pure gravity, no subtlety. Brightness is matter: it bends the picture around
itself, shears it into rotation, splits its light red/blue by infall
direction, rings itself in glowing photon shells, swallows white-hot debris,
and past the horizon collapses into voids (or white holes). **Every control
acts on one always-on system; nothing is zone-gated** — the design verdict
that killed v1 (Kelvin): its temperature zones left most controls doing
nothing at any given slider position, its lens was too polite, and its 100%
endpoint (bleach + black blobs) was a degenerate state instead of a climax.

## Controls

| Ctrl | Name | Does |
|---|---|---|
| P12 | Collapse | master G: deflection (to ±255 px), space darkening, horizon engagement, debris speed. 0 = exact dry, 100% = the designed climax: black space, ringed singularities, spaghettified light |
| K1 | Mass | mass-formation threshold `clamp((l8−(255−K1))<<2)`: low = only highlights are matter, high = everything ×4 — every ring and warp moves instantly |
| K2 | Spin | frame drag: deflection sheared by the field's vertical slope, signed, center detent |
| K3 | Shells | width of 3 concentric photon shells (bands of the mass field at thr−40/−96/−152), hues rotating together |
| K4 | Doppler | signed chroma shift by deflection (V+ / U− ∝ dx: blue toward, red away) + fringe zones |
| K5 | Debris | density of 1×3 white streaks raining into the masses (consumed on arrival) |
| K6 | Horizon | collapse threshold |
| S7 | Polarity | Bright/Dark Mass (inverts the vacc accumulator input) |
| S8 | Core | Void (near-black) / White Hole (inverted warped video) |
| S9 | Force | Attract/Repel — sign lives in the per-frame gain/spin operands, S6 stays a plain add |
| S10 | Turbulence | boiling-spacetime jitter (qsin, luma-zone amp) |
| S11 | Pulse | breathing G |

## Architecture

Single streaming pipeline S0..S13, C_LATENCY = 14 (unchanged from v1).
Single-bank full-color warp buffer (11 EBR); mercurial coarse-grid mass
field ({mb u8, gx s8} in one 2048×16, vacc IIR, hblank updater, dual
bilinear); debris = per-8px-column particle + bright-surface map
(matrix_rain FSM, LAND→CALC→DECIDE). EBR 22/32. LC ~6670 (87%).

**What made v2 dramatic vs v1:** updater gradient pre-amplified ×4
(`<<1` not `>>1`), lens product shift `>>4` not `>>7` (×8), deflection clamp
widened s8→s9 (±255) — combined ~×32 available deflection; the field is
also sharper because K1's threshold curve saturates masses to full scale.

## Multiply ledger (11)
M1–M6 dual bilinear, M7 lens `gx_bil×geff9` (s8×s9, gain carries the Repel
sign), M9 turbulence `qsin×amp` (s10×s9), M10 spin `dv2m×spin8` (s10×s8),
MG1 darken `y8×dk8` (identity form `y − 2dk + (y·dk)>>7` ≡ `y −
((255−y)·dk)>>7`, the fused-subtract lesson), MG2 doppler `dx9×dop8`.

## Timing lessons (v2 additions to the v1 list)
1. K1's mass curve (mux→sub→compare→shift) inline before the vacc
   accumulator add was the v2 critical path — register the curve (`l8_r`),
   accumulate one pixel late (invisible in a 32-px cell average).
2. Shell bands: compares must see PRE-REGISTERED bounds (mercurial strata
   lesson, re-learned) — `c ± w` per pixel cost ~5 MHz.
3. Sign flips (Repel) belong in per-frame operands, not post-sum negate
   muxes.

## Status

v2.0.0 — all 6 configs routed-closed at full clock, ALL seed 1, no retries
(hd_analog independently re-verified 75.73 MHz routed). Built + packaged
with 4 presets (Gravity Well / Ergosphere / White Hole / Total Collapse).
NOT yet HW-tested. Chroma constants (shell hues from qsin, debris 478/540)
are BT.601-standard; if hardware shows swapped hues flip the U/V pairs.
Gotcha hit at packaging: description strings max 127 BYTES — an overlong
one fails validation and silently ships the stale config binary.
