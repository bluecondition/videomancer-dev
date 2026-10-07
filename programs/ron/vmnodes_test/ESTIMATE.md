# VM Nodes Test: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| rate | constant width10_value48 | full | 2 | 0 | 0 | 0 | 0 | - |
| vin | video_in default | full | 35 | 0 | 34 | 0 | 1 | 626.6 |
| k1 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k2 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k3 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k4 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k5 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k6 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| p12 | control bit-1_latchT | full | 15 | 2 | 11 | 0 | 1 | 288.9 |
| glide | glide shift3 | full | 60 | 55 | 26 | 0 | 1 | 152.5 |
| pa | procamp implshift_bits5 | chroma_dead | 269 | 184 | 136 | 0 | 5 | 124 |
| ramp | hramp frac6_wrapT | full | 46 | 33 | 27 | 0 | 1 | 175.8 |
| s7 | control bit0_latchT | full | 6 | 2 | 2 | 0 | 1 | 228.2 |
| lfo | lfo phase_bits14_rom_stylelogic | full | 290 | 272 | 58 | 0 | 2 | 138.5 |
| osc | osc addr_bits8_freq_shift0_rom_stylelogic | full | 243 | 228 | 28 | 0 | 3 | 134.7 |
| vb | vblur taps3_chromaF_depth_bits11 | chroma_dead | 367 | 197 | 224 | 10 | 7 | 107.4 |
| hb | hblur taps3_chromaF | chroma_dead | 226 | 151 | 99 | 0 | 6 | 93.2 |
| th | threshold channel0_softT | full | 183 | 133 | 147 | 0 | 3 | 90.9 |
| col | colorizer entries_bits3 | full | 39 | 23 | 36 | 0 | 2 | 180.4 |
| mix | mix implshift_bits4 | full | 537 | 439 | 432 | 0 | 4 | 129.4 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | osc.wave × 10 bit, taps 17 | | 170 | 0 | 170 | 0 | | |
| delay | th.out × 34 bit, taps 2 | | 68 | 0 | 68 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **3804** (49.5 %) | 2380 | 2216 | **10** (31.3 %) | 29 | |

Zone: **green** · controls used: 8/12 · program latency: 29 clocks · worst node Fmax: 90.9 MHz
