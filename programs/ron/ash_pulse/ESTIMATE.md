# Ash Pulse: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| vin | video_in default | chroma_dead | 36 | 0 | 34 | 0 | 1 | 626.6 |
| ctl_key_level | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_key_soft | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_lfo_speed | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_wave_freq | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| emb | relief span2_gain3 | full | 82 | 35 | 62 | 0 | 3 | 178.1 |
| post | posterize bits3_chromaT | full | 20 | 6 | 14 | 0 | 1 | 309.6 |
| key | threshold channel0_softT | chroma_dead | 183 | 133 | 57 | 0 | 3 | 97.8 |
| sw_lfo_shape | control bit0_latchT | full | 6 | 2 | 2 | 0 | 1 | 228.2 |
| lfo | lfo phase_bits16_rom_stylelogic | full | 291 | 273 | 60 | 0 | 2 | 129.9 |
| ramp | hramp frac8_wrapT | full | 50 | 37 | 29 | 0 | 1 | 159.9 |
| osc | osc addr_bits8_freq_shift1_rom_stylelogic | full | 243 | 228 | 28 | 0 | 3 | 90.9 |
| pal | colorizer entries_bits4_palettehccd0918a | full | 45 | 30 | 42 | 0 | 2 | 180.4 |
| blend | blend modesub | full | 156 | 108 | 74 | 0 | 2 | 177.2 |
| fringe | chroma_shift pixels8_leadluma | full | 195 | 0 | 194 | 0 | 1 | 345.1 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | post.out × 34 bit, taps 2 | | 68 | 0 | 68 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **2601** (33.9 %) | 1513 | 1362 | **0** (0 %) | 11 | |

Zone: **green** · controls used: 5/12 · program latency: 11 clocks · worst node Fmax: 90.9 MHz
