# Oracle — Glitchy Divinatory Portal

## What it does
A three-zone triptych synthesis program. A high-frequency vertical rainbow shimmer runs along the top, slow horizontal ROYGBIV bands run along the bottom, and the middle band is a window of grayscale noise containing a tall ellipse-shaped "portal" that reveals the rainbow beneath. Zone seams can be torn (jittered per-row) for a datamoshed look, and an occasional rainbow speck pokes through the noise field.

## How it works
Pure synthesis (no input dependency). The rainbow palette is an 8-entry ROYGBIV LUT in 10-bit YUV with linear blend between adjacent entries. The portal is an octagonal approximation of a 3:1 ellipse using `max(3*dx, dy) + min(3*dx, dy)/2` — no squared-distance multiplies. Noise is a Fibonacci-hash XOR-shift on a downsampled `(x,y,frame)` triple. Parameters latch at vsync and per-frame derived constants (centre, radius, band frequencies, noise grain shift) are precomputed so the 14-stage pipeline closes at 74.25 MHz on iCE40-HX4K with no DSP blocks.

## Controls
- **K1 Portal Size** — ellipse half-width scale (0 = closed, max ≈ ¼ frame width).
- **K2 Portal X** — horizontal centre position.
- **K3 Portal Y** — vertical centre position.
- **K4 Noise** — grain size for the noise window (low = fine, high = coarse).
- **K5 Hue Speed** — animation rate of the horizontal-band rainbow scroll.
- **K6 Band Freq** — multiplier for vertical-stripe and horizontal-band density.
- **T7 Top Zone** — enable the top vertical-stripe rainbow.
- **T8 Bottom Zone** — enable the bottom horizontal-band rainbow.
- **T9 Portal** — enable the elliptical rainbow portal in the noise zone.
- **T10 Tear Edges** — jitter the zone seams for a torn/datamoshed look.
- **T11 Bypass** — pass input video straight through.
- **S12 Brightness** — overall Y gain plus chroma attenuation toward neutral.

## Presets
- Default Oracle
- Quiet Portal
- Maximal Chaos
- Stopped Clock
- Pure Bars
