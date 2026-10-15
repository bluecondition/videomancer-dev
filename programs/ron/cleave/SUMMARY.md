
## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

v0.10.2 (2026-10-15): **seed 3**, set as `[firmware] seed = 3` in cleave.toml. LZX's stock build uses one seed for all 8 configs.
- **Why:** v0.10.2 adds the labelled-default decode for K4 Static Size. With it, seed 1 stalls router1 on SD HDMI (overuse stuck at 4147 for 20 min). v0.10.1 had built on seed 1.
- **Sweep:** seed 2 built 7/8 (HD HDMI stalled); seed 3 built 8/8.
- **Own HD timing** on seed 3 (scratch core without LZX's Sync Out generator): 91.4 / 88.8 / 93.5 MHz (Analog / HDMI / Dual).
- The dev flow (router2) is unchanged.
