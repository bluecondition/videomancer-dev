# Iron Lantern: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| vin | video_in default | full | 35 | 0 | 34 | 0 | 1 | 626.6 |
| ctl_lfo_speed | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_wave_freq | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| neg | invert lumaT_chromaF | full | 48 | 20 | 34 | 0 | 1 | 149.8 |
| p12 | control bit-1_latchT | full | 15 | 2 | 11 | 0 | 1 | 288.9 |
| glide | glide shift2 | full | 59 | 51 | 25 | 0 | 1 | 124.8 |
| sw_lfo_shape | control bit0_latchT | full | 6 | 2 | 2 | 0 | 1 | 228.2 |
| lfo | lfo phase_bits18_rom_stylelogic | full | 293 | 275 | 62 | 0 | 2 | 129.9 |
| ramp | hramp frac7_wrapT | full | 48 | 35 | 28 | 0 | 1 | 159.9 |
| osc | osc addr_bits8_freq_shift0_rom_stylelogic | full | 243 | 228 | 28 | 0 | 3 | 134.7 |
| pal | colorizer entries_bits4 | full | 45 | 30 | 42 | 0 | 2 | 180.4 |
| mix | mix implshift_bits3 | full | 426 | 336 | 425 | 0 | 4 | 153.1 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | neg.out × 34 bit, taps 3/5 | | 170 | 0 | 170 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **2592** (33.8 %) | 1640 | 1539 | **0** (0 %) | 12 | |

Zone: **green** · controls used: 4/12 · program latency: 12 clocks · worst node Fmax: 124.8 MHz
