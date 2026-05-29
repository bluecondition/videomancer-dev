# Blacklodge — Twin Peaks Red Room

## What it does
Synthesises the Red Room from Twin Peaks: a deep crimson velvet curtain with vertical folds hanging above a warm cream-and-brown chevron floor that recedes into the distance with forced perspective. The curtains can sway, the chevron pattern can scroll or rotate, and a vignette frames the scene.

## How it works
Two full-screen synthesis layers separated by a horizon line. The curtain uses summed triangle-wave luma modulation (with per-band coprime sway accumulators and a parabolic hem LUT for rounded fold bottoms). The chevron floor uses a horizon-anchored phase DDA with strictly linear wavelength growth in depth, driven by a reciprocal LUT (2^17 / wavelen) so stripe boundaries trace perfectly straight perspective rays. No per-pixel multipliers on hot paths; small lerp ROMs replace per-pixel multiplies for curtain blends. 8-stage pipeline. The Key switch composites incoming video into the floor region.

## Controls
- K1 Folds — curtain fold density
- K2 Depth — curtain fold shadow depth
- K3 Width — chevron stripe thickness near horizon
- K4 Apex — chevron vanishing-point X (0=left, 512=centre, 1023=right)
- K5 Scroll — horizontal phase offset of chevron pattern
- K6 ViewHt — viewer height (controls perspective compression / tooth frequency)
- T7 Sway — curtain animation on/off
- T8 Chev Dir — Horizontal radiating vs Vertical (90°-rotated) chevrons
- T9 Brown — swap dark chevron stripes from near-black to saturated brown
- T10 Vignette — soft 128px corner fade
- T11 Key — replace floor area with incoming video
- Slider Horizon — horizon line height (top = raised curtain, bottom = lowered)

## Presets
- Red Room
- Deep Velvet
- Alcove
- Stage Lit
