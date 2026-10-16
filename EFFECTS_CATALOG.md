# Video Effects Catalog

Reference list of effect families for future Videomancer program ideas (saved 2026-09-06).
Companions: [CLAUDE.md](CLAUDE.md) (platform rules) and [TECHNIQUES.md](TECHNIQUES.md) (proven implementation patterns).

## Spatial / Geometric
- Pixelation (nearest-neighbor downsample, variable cell size)
- Mosaic with per-cell color averaging
- Slit-scan (time-delayed column/row sampling)
- Datamosh / block displacement
- Mirror, kaleidoscope, radial symmetry
- Polar ↔ Cartesian warping (tunnel, fisheye, spherize)
- Lens distortion (barrel, pincushion, chromatic aberration)
- Displacement mapping (use luma of A to offset B)
- Feedback zoom/rotate (video-on-video trails)
- Scanline shear / horizontal tearing
- Tile & shuffle (randomized grid permutation)
- Rolling shutter / sync roll
- Perspective keystone / corner-pin

## Color / Tonal
- Palette quantization (CGA, C64, EGA, Gameboy, custom LUT)
- Posterization (bit-depth reduction per channel)
- Solarization / Sabattier inversion above threshold
- Duotone / tritone mapping from luma
- False-color (thermal-style LUT)
- Hue rotation, saturation crush/boost
- Chroma key, luma key, difference key
- Color channel swap / offset (RGB ↔ YUV tricks)
- Invert, negative, bit-plane isolation
- Gamma curves, S-curve contrast
- Chroma subsampling artifacts (4:1:1, composite simulation)
- Color bleeding / NTSC dot crawl emulation

## Temporal
- Frame blending / echo / trails
- Frame hold / stutter / strobe
- Reverse buffer playback
- Time displacement map (per-pixel time offset by luma)
- Onion skin / ghost compositing
- Datamosh P-frame repeat
- Slow-shutter accumulation
- Motion blur, motion extrapolation
- Frame difference (motion detection visualization)

## Feedback / Recursive
- Classic Rutt-Etra-style escalation
- Rotating feedback with decay
- Colored feedback (channel-split delay lines)
- Feedback with displacement map
- Feedback through a lens/mirror chain

## Stylization
- Edge detect (Sobel, Roberts, difference-of-gaussian)
- Cartoon/cel (edge + flat color)
- ASCII / dither-art / halftone
- Ordered dithering (Bayer), error-diffusion (Floyd-Steinberg)
- Crosshatch, stipple, pointillism
- Oil paint / Kuwahara smoothing
- Bloom, glow, soft-focus
- Chromatic glitch / datamosh bars

## Analog / CRT Emulation
- Composite luma/chroma crosstalk
- RF noise, snow, VHS tracking
- Scanlines, aperture grille, barrel distortion
- Sync loss / horizontal collapse
- Head-switch noise band
- Magnetic warp (wobble)
- Phosphor persistence / decay

## Generative / Hybrid
- Perlin/simplex displacement
- Reaction-diffusion driven by input
- Cellular automata seeded by video
- Particle fields advected by optical flow
- Depth-from-luma → parallax
- Kintsugi-style crack overlays
- Mycelium/growth patterns

## Compositional
- Multi-source mixing (add, multiply, screen, difference, overlay)
- Masking by luma/chroma/alpha
- Picture-in-picture with warping
- Split-screen, quad-mirror

## Experimental / Glitch
- **Hysteresis-edge state glitches** (found 2026-10-15 on hueflutter v0.2.1). A zoned control (bit depth, cell size, palette) whose zone only changes once the knob is N counts inside a new zone remembers HOW it got there. Turned by hand from below, 515 gave 4-bit chroma; from above it would have kept 5 bits; loaded cold it gave the power-up zone, 3-bit chroma, which also triggered the HD HDMI bright bars. Ways to use it deliberately:
  - **Path-dependent knobs:** the same knob position gives different looks depending on the direction you arrived from. Widen the hysteresis band (say ±40 counts) so it reads as an instrument, not a bug.
  - **Schmitt-trigger posterize on the video itself:** per-pixel quantization with hysteresis along the scanline. A level holds until the input moves well past the threshold, so edges smear sideways into streaks and noise turns into stable horizontal glitch runs. Hysteresis that resets per line makes it purely horizontal; carrying state down the column smears vertically.
  - **Modulated band:** drive the hysteresis width from an LFO or from luma, so zones stick and release in bursts.
  - **Sticky-state switch:** "freeze zone" holds the current quantizer state until the knob crosses a far threshold.
  - **Caution:** hard bit-depth flips on moving content can trigger the HD HDMI thick bars. Build in the [1 2 1] output smoothing from the start (see CLAUDE.md, "HD HDMI thick bright bars").
