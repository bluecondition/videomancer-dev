
## v0.10.3 resolution scaling (2026-10-17, dev repo)

Band, wobble, static-cell, patch and outline geometry now run in virtual 1080-line units stepped from the
timing ID (`f_vstep_x` / `f_vstep_y`, as in mondrian v1.0.2), so 720p and SD show the 1080p picture scaled
by screen height (SD = a 1440-unit 4:3 window; the band zoom stays centred on the measured W/2 or H/2).
- K1 band step per pixel = vstep * xs/2 (`s_hstep`); per field line = vstep * ys/256 (`s_lstep`, vblank serial multiply).
- K2 wobble step per pixel = wstep * xs (Q.1 `wob_ha`); per field line = wstep * ys (Q.8 `wob_va`, same multiplier).
- Medium/Coarse/Chunky static cells (2/4/8 units) and the 128-unit Patchwork tiles key on `vx_a` / `vy_a`.
- Outline width is a per-mode table (`f_outline`): 1080p Thin 2 px x 2 lines / Thick 6 x 6 (unchanged); 720p 1x1 / 4x4;
  1080i 2x1 / 6x3 field lines; 480i/576i 1x1 / 3x2; 480p/576p 1x1 / 3x3.
- Pixel scale kept: Fine static, the 1px band-fuzz/class-dither hash, the per-line/column serration, the line buffer.
- 1080p bit-identical to v0.10.2 (diff1080 = 0).
- Build (dev flow, router2 default): `./build_programs.sh ron cleave`, 6/6 seed 1. HD Analog 78.80, HD HDMI 86.96,
  HD Dual 86.15 MHz; SD 83.13 / 82.69 / 90.38 MHz; ~4030/7680 LC (52%), 4 EBR.

## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

v0.10.2 (2026-10-15): **seed 3**, set as `[firmware] seed = 3` in cleave.toml. LZX's stock build uses one seed for all 8 configs.
- **Why:** v0.10.2 adds the labelled-default decode for K4 Static Size. With it, seed 1 stalls router1 on SD HDMI (overuse stuck at 4147 for 20 min). v0.10.1 had built on seed 1.
- **Sweep:** seed 2 built 7/8 (HD HDMI stalled); seed 3 built 8/8.
- **Own HD timing** on seed 3 (scratch core without LZX's Sync Out generator): 91.4 / 88.8 / 93.5 MHz (Analog / HDMI / Dual).
- The dev flow (router2) is unchanged.
