# Wave Key: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| vin | video_in default | chroma_dead | 36 | 0 | 34 | 0 | 1 | 626.6 |
| k1 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k2 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k3 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k4 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| key | threshold channel0_softT | chroma_dead | 183 | 133 | 57 | 0 | 3 | 97.8 |
| p12 | control bit-1_latchT | full | 15 | 2 | 11 | 0 | 1 | 288.9 |
| s7 | control bit0_latchT | full | 6 | 2 | 2 | 0 | 1 | 228.2 |
| s8 | control bit1_latchT | full | 6 | 2 | 2 | 0 | 1 | 228.2 |
| speed | glide shift4 | full | 63 | 56 | 27 | 0 | 1 | 124.8 |
| lfoa | lfo phase_bits16_rom_stylelogic | full | 291 | 273 | 60 | 0 | 2 | 129.9 |
| hramp | hramp frac8_wrapT | full | 50 | 37 | 29 | 0 | 1 | 159.9 |
| hosc | osc addr_bits7_freq_shift0_rom_stylelogic | full | 141 | 125 | 27 | 0 | 3 | 148.8 |
| hcol | colorizer entries_bits4 | full | 45 | 30 | 42 | 0 | 2 | 180.4 |
| lfob | lfo phase_bits16_rom_styleauto | full | 72 | 54 | 51 | 1 | 2 | 129.9 |
| vcnt | vcount width11 | full | 29 | 13 | 23 | 0 | 1 | 249.9 |
| vosc | osc addr_bits10_freq_shift2_rom_styleauto | full | 24 | 11 | 21 | 3 | 3 | 90.9 |
| vcol | colorizer entries_bits3 | full | 39 | 23 | 36 | 0 | 2 | 180.4 |
| mix | mix implshift_bits5 | full | 630 | 546 | 438 | 0 | 4 | 86.6 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | key.out × 34 bit, taps 1 | | 34 | 0 | 34 | 0 | | |
| delay | vosc.wave × 10 bit, taps 1 | | 10 | 0 | 10 | 0 | | |
| delay | vcol.out × 34 bit, taps 1 | | 34 | 0 | 34 | 0 | | |
| delay | key.mask × 10 bit, taps 3 | | 30 | 0 | 30 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **2964** (38.6 %) | 1968 | 1666 | **4** (12.5 %) | 12 | |

Zone: **green** · controls used: 7/12 · program latency: 12 clocks · worst node Fmax: 86.6 MHz
