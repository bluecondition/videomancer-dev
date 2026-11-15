# Lenticular — summary

Catalogue combo, 2026-10-27. Ron asked for "another one with similar rules, lesser used effects", after
Lineage, Fissure and Morphogen. v0.1.0 is built (6/6, 68% LC, 8 EBR), not flashed and not simulated.

## Choosing the effects

These counts are over the 49 kept combos.

| Uses | Entries |
|---|---|
| 0 kept | **masking**, tile & shuffle, CRT scanlines / grille (every combo using these was removed after review); all frame-store effects |
| 1 | **lens distortion / chromatic aberration** (hallway), displacement mapping, rolling shutter, crosshatch, Kuwahara, composite crosstalk, RF snow, PiP warp, plus the generative entries just used (CA, hysteresis, kintsugi, mycelium, reaction-diffusion) |
| 2 | **depth-from-luma parallax** (parallax, stratamix), cel, keystone, false colour, duotone, palette quantization, channel swap, bit-plane, gamma, chroma subsampling, colour bleed, ordered dither |

The last three programs all grew down the scanlines, so this one changes the picture's geometry
instead. It is a lenticular 3-D postcard:
- **parallax (2 uses):** the views are the picture shifted by its depth;
- **masking (0 kept uses):** the interleave decides which view shows where;
- **lens (1 use):** the ridges show one view per ridge, with sheen and ghosting.

Everything is horizontal, so it needs only a short picture ring, with no line buffers and no
frame store. The glass and lens programs already on the shelf (Refractor, Dewdrop, Loupe, Porthole)
are single lenses or drops; none is a lens array with multiple views.

The look prototype is `.lent_work/proto.py`. Flip reads as a classic flip card, Holo as a
holographic sticker, and high Spread as the raw interleave.

## Engine (all per pixel, latency 24)

- **Ridge phase.** A Q12 DDA (4096 / pitch × virtual px per pixel), plus a per-line offset for
  Slant (half a pitch per virtual line).
- **View coordinate (Q8 views).** It sums four terms:
  - tilt (P12 × N);
  - N/2 offset ± wobble (cosine, about 4 s a rock);
  - band (N − ¼ views across the virtual width, a Q16 DDA);
  - spread × ridge phase.
- **View index.** The view coordinate's integer part mod N, done as two compare-subtract stages
  (the terms are bounded below 4N).
- **Depth.** A 16-pixel box of 6-bit luma, IIR-smoothed down each column in a 2048 × 8 RAM.
- **Parallax.** Shift = view × (Depth / (N − 1)) × depth. It is one-sided (views only read the
  past), so a 256-word ring needs no base delay. A second ring serves the neighbouring view for
  Ghost.
- **Card transforms.**
  - Flip: odd views negative.
  - Holo: the picture's luma with a prism hue that moves with view, tilt and height. These are
    generated colours, so U/V swap on off-parity lines.
  - Pop: odd views posterized, with chroma pushed.
- **Ghost.** Near each view transition the next view bleeds in at 50%.
- **Sheen.** Cosine across the ridge × K3. It brightens the ridge middle and darkens the grooves,
  about black.
- **Output.** Conditioning (luma [1 2 1], chroma 9-tap, luma x13/16), then the blanking gate.

## Resolution

- Pitch, depth, slant and bands are all in virtual 1080-line units, so the card looks the same at
  1080p, 720p and SD.
- On 4:3 SD the band sweep covers about 0.7 N views across the narrower virtual width (the same
  per-inch rate).

## Status

v0.1.0 (2026-10-27): built and packed (`out/rev_b/ron/catalogue-combo/lenticular.vmprog`),
timing-closed, not flashed, not simulated.

| Config | Fmax (MHz) | Seed | LC | EBR |
|---|---|---|---|---|
| HD Analog | 76.2 | 1 | 5218 (68%) | 8 |
| HD HDMI | 86.5 | 1 | 5232 | 8 |
| HD Dual | 86.7 | 1 | 5224 | 8 |
| SD Analog | 81.8 | 1 | 5225 | 8 |
| SD HDMI | 73.6 | 1 | 5236 | 8 |
| SD Dual | 81.1 | **2** | 5215 | 8 |

Build seeds: SD Dual seed 1 stalled the router (timeout); seed 2 passed. Every other config passed
on seed 1. SD runs at 27 MHz, so the SD numbers have plenty of room. HD Analog's margin is about 3%.

Fixes made while closing timing (alone it went 79.8 -> 82 MHz):
- the depth multiply was narrowed to 6-bit depth;
- the luma x sheen subtract moved one stage earlier;
- the per-field Slant multiply became a mode-selected shift pair.

Start-up: no engine state. Everything is recomputed per pixel from the current line, so nothing can
strand after configuration.

## Things to watch on hardware

- **HD HDMI bars.** Fine ridges and view transitions are sharp, repeating edges that move as the
  card tilts. The conditioning is built in. If bars appear, widen the chroma filter first, then
  soften the view transitions (Ghost on).
- **Depth jitter.** Depth comes from luma, so textured areas can shimmer as they shift between
  views. The 16-pixel box plus the column IIR should hold it. If it crawls, smooth harder.
- **Tilt feel.** P12 sweeps all N views over its travel, so with 8 views each view gets 1/8 of the
  slider. If flips feel too fast, the sweep can be scaled.
