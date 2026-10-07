# Liquid Tundra: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| vin | video_in default | full | 35 | 0 | 34 | 0 | 1 | 626.6 |
| ctl_grain | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_key_level | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_key_soft | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| grain | noise animateT | full | 207 | 194 | 73 | 0 | 3 | 110.7 |
| hb | hblur taps5_chromaT | full | 786 | 570 | 559 | 0 | 7 | 109.5 |
| key | threshold channel0_softT | chroma_dead | 183 | 133 | 57 | 0 | 3 | 97.8 |
| osc | shaper opsquare | full | 2 | 0 | 1 | 0 | 1 | 626.6 |
| pal | colorizer entries_bits2 | full | 22 | 3 | 18 | 0 | 2 | 180.4 |
| mix | mix implshift_bits4 | full | 537 | 439 | 432 | 0 | 4 | 129.4 |
| fringe | chroma_shift pixels6_leadluma | full | 155 | 0 | 154 | 0 | 1 | 345.1 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | osc.out × 10 bit, taps 3 | | 30 | 0 | 30 | 0 | | |
| delay | hb.out × 34 bit, taps 3 | | 102 | 0 | 102 | 0 | | |
| delay | pal.out × 34 bit, taps 1 | | 34 | 0 | 34 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **3308** (43.1 %) | 2000 | 2182 | **0** (0 %) | 17 | |

Zone: **green** · controls used: 3/12 · program latency: 17 clocks · worst node Fmax: 97.8 MHz
