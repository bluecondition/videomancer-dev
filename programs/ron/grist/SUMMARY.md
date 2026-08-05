# GRIST — grain-engine exploration (forked from morsel's dough grain)

Status: **exploration only** — no VHDL yet. `explore_grain.py` isolates morsel's
surface-noise engine (bit-exact `f_pack`/`f_mix1`/`f_mix2` hash) and pushes each
axis to an extreme over a synthetic luma ramp + disc. All variant math is
integer shift/add/compare — every look is HX4K-implementable, no DSP/BRAM.

## The engine (from morsel.vhd p_dough)

- **fine grain** — signed nibble of `hash(x, y)` × amplitude (per-pixel tooth)
- **block mottle** — same hash on `(x>>ms, y>>ms)` (morsel: 2–8 px "browning")
- **pores** — hash top nibble < threshold → fixed dark drop (sparse speckles)
- Static function of (x,y): a stable texture, not animated noise. Free-running,
  so it can be computed arbitrarily early in the pipeline (morsel: 14 stages).

## The extremes (ext_*.png, 1920×1080)

| file | axis | verdict |
|---|---|---|
| a_morsel_baseline | as shipped (g=4, ms=3) | subtle cookie tooth |
| b_amp_blizzard | amplitude → max | analog-noise veil, generic |
| c_blocks_plaster | ms=6, 8-bit block noise | becomes a 64 px MOSAIC — block noise dominates, different genre |
| d_pores_corroded | pore threshold ×2.5, drop −140 | pitted sandpaper / corroded metal, gritty and good |
| e_pores_sparkle | inverted pores (+500 on darkened base) | mica sparkle / dust-in-projector |
| f_octaves_concrete | 4 octaves summed (px + 2/8/32 blk) | concrete / handmade paper — richest static texture |
| g_binary_mezzotint | luma vs dithered threshold comparator | **standout** — spray-paint mezzotint, whole image re-rendered as grain density |
| h_film_response | amplitude ∝ min(Y, 1023−Y) | filmic mid-tone grain, clean shadows/highlights |
| i_carve_posterize | grain added before 3-bit posterize | boiling dithered contours between flat zones |

## Notes for the eventual program

- Mezzotint (g) is a pure compare — the threshold field can be built from any
  octave mix, so one engine covers g/f/h continuously.
- Temporal axis not shown in stills: xor a frame counter into the seed →
  animated film grain; per-frame vs static should be a switch.
- ms > 3 needs wider hash input coverage: `f_pack` only uses x(9:0)/y(5:0);
  at big shifts the y-slice thins — fine in these renders, but recheck
  diagonal correlation if block size grows past 64 px.
