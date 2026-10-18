# Blacklodge — Twin Peaks Red Room

## What it does
Synthesises the Red Room from Twin Peaks: a crimson velvet curtain with rounded folds hangs over a cream-and-brown chevron floor that recedes in forced perspective. The curtain billows on slow travelling waves. The floor can walk toward or away from the viewer. Flicker adds random Lynch-style light bursts, and Key puts the input video on the floor.

## How it works
Two synthesised layers meet at a horizon.

**Curtain**
- A fold-phase DDA sets the fold density (K1).
- Two travelling triangle waves phase-modulate the folds. Their amplitude is pinned at the top of the frame and grows toward the bottom, which gives the sway.
- Fold shading comes from velvet sin² lerp LUTs, so ridges and creases are rounded.
- Each fold has a parabolic rounded bottom (hem LUT) that follows the swayed phase, with a drop shadow on the floor beneath it.

**Floor**
- A wavelength DDA grows linearly with depth (reciprocal LUT, interpolated per row).
- Rays mode: stripes radiate from the vanishing point and zigzag along their length. Rows mode: zigzag bands run across the floor.
- K5 Walk offsets the row accumulator each frame, which makes the floor flow.

**Shared**
- The vignette is always on. The Flicker gain scales luma (above black 64) and chroma together.
- No per-pixel multipliers on the hot paths except the gain stage. The apex × row-frequency product runs on a serial shift-add multiplier during hblank.
- `processing` program type, with a blanking gate on the output. An active-line counter with a saw-active vsync guard makes the frame height, horizon and vignette robust on analog sources.

## Controls
- K1 Folds — curtain fold density
- K2 Depth — fold shadow level
- K3 Width — chevron stripe width at the horizon (4–36 px at 1080p, scaled with the screen; smooth)
- K4 Apex — vanishing-point x
- K5 Walk — floor flow speed, −100% to +100% (centre = still, right = toward the viewer)
- K6 ViewHt — perspective growth (smooth)
- S7 Sway — Off freezes the drape in place, On billows
- S8 Chevrons — Rays / Rows
- S9 Brown — dark stripes near-black or saturated brown
- S10 Flicker — random light-flicker bursts
- S11 Key — input video replaces the floor
- P12 Horizon — curtain line height (top = curtain raised)

## Presets
Red Room (the original v0.1 defaults), Deep Velvet (walk + flicker), Alcove, Stage Lit (sway frozen), Vertical Weave (Rows, walking), Signal Floor (Key). These are hand-authored, not captured from the hardware.

Defaults were captured from the hardware on 2026-10-08: K1 288, K2 242, K3 632, K4 461, K5 736 (walking forward), K6 0. Switches and P12 are unchanged.

## History
- v0.1.0: original. Its vmprog is kept at out/rev_b/ron/_diag/blacklodge_v010.vmprog.
- v1.0.0 (2026-10-07): finalization, developed and tested as `blacklodge_diag`. Ron approved it for production. The diag vmprog is kept at out/rev_b/ron/_diag/blacklodge_diag_v020.vmprog; its source folder was deleted.
- 2026-10-08: defaults captured from the hardware; **FINAL** (signed off by Ron).
- v1.0.1: community-repo compliance (czartroniks ID, GPL header).
- v1.0.2 (2026-10-17): resolution scaling. The pixel and line counters run in virtual 1080-line units, stepped from the video timing ID (1.0 at 1080p, 1.5 at 720p; at SD 2.0 across and 1080/486 per frame line, so SD is a 1440-wide 4:3 canvas). Fold pitch, sway waves, hem bulge and shadow, horizon, apex, chevron width, the floor's per-row perspective growth and the vignette all follow. The per-row increments (row phase `freq_row × ys`, wavelength growth `step × ys`) run on the existing hblank serial multiplier, and the sway amplitude is line/32 in every mode (720p used to get ~25% more sway). Before this, 720p and SD had wider folds and chevrons, and the 128-line vignette never reached full brightness in a 243-line SD field. 1080p output is bit-identical (pixel diff 0).

## Build (v1.0.2, 2026-10-17, router2, base seed 1)
`./build_programs.sh ron blacklodge` (router2 default, seed 1; HD Dual landed on seed 2 inside the builder's retry loop).

| Config | Fmax (MHz) | Logic cells | Seed |
|---|---|---|---|
| HD Analog | 79.10 | 5270 (69%) | 1 |
| HD HDMI | 83.27 | 5272 | 1 |
| HD Dual | 81.97 | 5292 | 2 |
| SD Analog | 80.30 | 5269 | 1 |
| SD HDMI | 80.79 | 5276 | 1 |
| SD Dual | 86.18 | 5279 | 1 |

2 of 32 EBR used.

## Build (v1.0.0, 2026-10-07, router2, all 6 configs seed 1)
| Config | Fmax (MHz) | Logic cells |
|---|---|---|
| HD Analog | 85.39 | 5086 (66%) |
| HD HDMI | 85.32 | 5085 |
| HD Dual | 80.60 | 5098 |
| SD Analog | 82.75 | 5085 |
| SD HDMI | 86.33 | 5076 |
| SD Dual | 82.38 | 5102 |

2 of 32 EBR used.

## HW status
**FINAL v1.0.0, HW-validated** (signed off 2026-10-08). Defaults were captured from the hardware; presets are hand-authored.
v1.0.2 (resolution scaling) is timing-closed but not yet flashed; its 1080p output is unchanged.
