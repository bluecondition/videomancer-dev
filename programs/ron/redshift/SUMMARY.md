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

## Edge fill (v2.1)

Warp reads landing within 6 columns of either border used to smear the
capture source's black blanking columns across the deflected zones. Lagoon's
left-fill pattern, applied to BOTH edges: each line averages 64 pixels
(starting past the black columns), latches at hsync, and the S9 land stage
muxes that colour in for border-zone reads (sidewinder p_wet precedent for
a stable-select mux on BRAM outputs). Gated on dx /= 0 so Collapse = 0
stays exactly dry. The fill enters before the grades, so it inherits the
darkening and the red/blue Doppler drag near masses.

## 1080p bottom-edge fix (v2.2)

The HD hardware is 1080p PROGRESSIVE; v2.1 was written against 540-line
fields. Four faults, all at the bottom of the frame or on analog vsync:
1. `s_aline` was 10 bits, saturating at line 1000, so lines 1000-1079 all read
   as line 1000. The mass field, shells, core and debris hit-test stopped
   changing, and the bottom ~80 lines (about an inch) became vertical
   stripes. Fixed by making it 11 bits.
2. 16-line cell rows on 1080 lines = 68 rows x 60 > 2048, so the mass RAM
   wrapped and the bottom half's field aliased onto the top half, shifted 8
   cells right (ghost lenses). The geometry now also keys on the measured
   field height: 1080p uses 32x32 cells (34 rows, max address 2039), 1080i
   keeps 32x16. On 32-line rows the vacc IIR accumulates even lines only,
   so it still covers half the cell row.
3. The last cell row's lower bilinear corners read the row past the bottom
   (unwritten, or aliased to row 0). They now replicate the last row
   (`s_lastrow` per field, `s_row` counter).
4. Debris runs in half-rows on tall rasters (`s_dline`), so the 10-bit
   particle/surface words, off-screen sky (960+) and streak/speed scale
   are unchanged from the 540-row design.
Also added: a serrated-vsync guard (animation phases and the debris walk
fire once per field; before, each serration edge advanced them again), and
the blanking contract (output neutral 64/512/512 outside avid; it used to
hold the line's last pixel through blanking).

## Resolution scaling (v2.2.2, 2026-10-18)

720p used 32-px cells (1.5x coarse lenses), and SD's deflection was a fixed
+/-255 px (2.2x stronger relative to the screen) with a 128-px turbulence
wavelength. Now the geometry is on a virtual 1080-line canvas (timing ID):
- mass cells are 32 x 32 virtual units everywhere: a half-unit span
  accumulator (`s_cxh/s_cpx/s_ccol`, prefetch events still count pixels
  within the span, `fx5` = virtual position) and a Q.8 row accumulator
  (`s_cyacc`, `fy5`, `s_rowfirst`) replace the 2^xsh / 2^ysh bit slices;
  60 cols x 34 rows at HD (720p too), 45 x 34 on SD's 1440-unit window;
  the span average is per mode (1080 /32, 720p x3/64, SD /16) and the vacc
  write moved one clock later (now a true one-clock pulse -- it used to stay
  high through hblank, so on SD/1080i row-end lines the updater read the
  last column while it was being written: undefined on iCE40 EBR);
  `s_lastrow` is latched from the row count at the field end;
- lens / spin / turbulence gains x 1/xstep in sequencer states 6-5 (720p
  1/2 + 1/8 + 1/32), deflection clamp 255 units (+/-255 / 170 / 127 px),
  Doppler gain x xstep (follows the virtual deflection), turbulence
  wavelength 128 units (`s_xvt`);
- debris rows are half virtual lines (`s_dla`, the 540-row design at every
  mode), `s_nrows` latched at the field end, and the per-column density is
  x xstep to make up for the 8-px columns (pixel scale kept: the prefetch
  needs 8 clocks per column).
1080p bit-identical (sim pixel diff 0). Sims: 720p r 1.00 / edges 0.98 vs
1080p (was 0.89 / 0.83); SD r 0.95 / 0.89 (was 0.93 / 0.85). Cost ~+200
LUT / +110 FF standalone.

Build `./build_programs.sh ron redshift` (router2): 6/6 on seed 1 -- HD
Analog 77.44, HD HDMI 78.92, HD Dual 79.49, SD 73.48 / 82.53 / 73.59 MHz;
7236-7259 LC (**94.5%**, at the congestion ceiling -- no headroom left for
features), 22/32 EBR.

## Status

v2.1.0 — all 6 configs routed-closed at full clock (five seed 1, one seed
2; v2.0 hd_analog independently re-verified 75.73 MHz routed, v2.1 78.24).
Built + packaged with 4 presets (Gravity Well / Ergosphere / White Hole /
Total Collapse). v2.0 HW-approved 2026-07-12; v2.1 edge fill not yet
HW-checked. Chroma constants (shell hues from qsin, debris 478/540) are
BT.601-standard. Gotcha hit at packaging: description strings max 127
BYTES — an overlong one fails validation AFTER bitstreams succeed and
silently ships the stale config binary.
