# Humid Vapor: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| vin | video_in default | full | 35 | 0 | 34 | 0 | 1 | 626.6 |
| ctl_key_level | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_key_soft | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_lfo_speed | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_wave_freq | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| key | threshold channel0_softT | chroma_dead | 183 | 133 | 57 | 0 | 3 | 97.8 |
| p12 | control bit-1_latchT | full | 15 | 2 | 11 | 0 | 1 | 288.9 |
| glide | glide shift3 | full | 60 | 55 | 26 | 0 | 1 | 152.5 |
| ramp | hramp frac7_wrapT | full | 48 | 35 | 28 | 0 | 1 | 159.9 |
| sw_lfo_shape | control bit0_latchT | full | 6 | 2 | 2 | 0 | 1 | 228.2 |
| lfo | lfo phase_bits14_rom_stylelogic | full | 290 | 272 | 58 | 0 | 2 | 138.5 |
| osc | osc addr_bits8_freq_shift0_rom_stylelogic | full | 243 | 228 | 28 | 0 | 3 | 134.7 |
| pal | colorizer entries_bits3 | full | 39 | 23 | 36 | 0 | 2 | 180.4 |
| mix | mix implshift_bits6 | full | 732 | 654 | 446 | 0 | 4 | 98 |
| wet | mix implshift_bits4 | full | 537 | 439 | 432 | 0 | 4 | 129.4 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | vin.out × 34 bit, taps 4/6/10 | | 340 | 0 | 340 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **3754** (48.9 %) | 2504 | 2196 | **0** (0 %) | 16 | |

Zone: **green** · controls used: 6/12 · program latency: 16 clocks · worst node Fmax: 97.8 MHz
