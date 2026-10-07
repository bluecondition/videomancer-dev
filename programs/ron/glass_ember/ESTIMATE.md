# Glass Ember: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| vin | video_in default | full | 35 | 0 | 34 | 0 | 1 | 626.6 |
| ctl_bright | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_centre_x | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_contrast | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_key_level | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_key_soft | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_saturation | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| hb | hblur taps3_chromaF | full | 346 | 151 | 279 | 0 | 6 | 111.7 |
| pa | procamp implshift_bits6 | full | 788 | 621 | 473 | 0 | 5 | 90.8 |
| key | threshold channel1_softT | chroma_dead | 183 | 133 | 57 | 0 | 3 | 92.1 |
| rx | hramp frac6_wrapF | full | 63 | 60 | 28 | 0 | 1 | 170.5 |
| ry | vramp frac4_wrapF | full | 40 | 27 | 26 | 0 | 1 | 217.8 |
| rad | radial metricoctagon_scale1 | full | 142 | 130 | 29 | 0 | 2 | 89.8 |
| osc | osc addr_bits8_freq_shift1_rom_stylelogic | full | 243 | 228 | 28 | 0 | 3 | 90.9 |
| pal | colorizer entries_bits3 | full | 39 | 23 | 36 | 0 | 2 | 180.4 |
| sw_lfo_shape | control bit0_latchT | full | 6 | 2 | 2 | 0 | 1 | 228.2 |
| lfo | lfo phase_bits16_rom_stylelogic | full | 291 | 273 | 60 | 0 | 2 | 129.9 |
| mix | mix implshift_bits4 | full | 537 | 439 | 432 | 0 | 4 | 129.4 |
| fringe | chroma_shift pixels8_leadluma | full | 195 | 0 | 194 | 0 | 1 | 345.1 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | osc.wave × 10 bit, taps 5 | | 50 | 0 | 50 | 0 | | |
| delay | pa.out × 34 bit, taps 2 | | 68 | 0 | 68 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **4274** (55.7 %) | 2748 | 2514 | **0** (0 %) | 20 | |

Zone: **green** · controls used: 7/12 · program latency: 20 clocks · worst node Fmax: 89.8 MHz
