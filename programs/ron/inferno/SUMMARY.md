# Inferno — Semi-realistic procedural flame with noise-based fire, overlay mode, and multiple flame styles

## What it does
A procedural fire effect: flames rise from the bottom of the frame with white-hot cores, yellow midtones, orange and red shoulders, fading through dark red into black at the edges. Domain-warped multi-octave noise gives the flame organic turbulence and motion. Palette toggles swap between the default fire palette, a glowing Ember style, a more intense Inferno, and a Blue Flame variant; Overlay composites the flame on top of incoming video for fire-over-image effects.

## How it works
8-stage pipeline. A 64-entry signed sin LUT doubles as a nonlinear hash mixer (approximating the shader's `fract(sin(cos(dot(...))) * 83758)`) — two linear combinations of (x, y) get folded through the LUT and XORed, no multipliers needed for hashing. Two noise octaves are bilinearly interpolated and summed for the domain-warp shape, then run through a sharp `1/x^4` brightness curve (yields tiny bright cores with dark falloff) and finally a warm Y→YUV palette. Time evolution comes from per-frame phase offsets driving the hash. Overlay mode mixes the result on top of incoming video by luma.

## Controls
- **K1 Fuel** — fire density / amount of flame
- **K2 Spread** — horizontal flame spread
- **K3 Heat** — overall brightness / saturation balance
- **K4 Turblnce** — noise turbulence intensity
- **K5 Tint** — palette tint shift (−50..+50)
- **K6 Rise** — flame rise speed
- **T7 Ember** — Ember palette variant
- **T8 Inferno** — Inferno palette variant
- **T9 Blue Flame** — Blue Flame palette variant
- **T10 Overlay** — composite flame on incoming video
- **T11 Bypass** — pass video through unchanged
- **Slider Intensity** — master output gain
