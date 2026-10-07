# Iron Harbor: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| vin | video_in default | full | 35 | 0 | 34 | 0 | 1 | 626.6 |
| ctl_grain | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_key_level | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_key_soft | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| ctl_smear_level | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| grain | noise animateF | full | 171 | 161 | 65 | 0 | 3 | 127.4 |
| osc | shaper opfold | full | 25 | 23 | 10 | 0 | 1 | 286.9 |
| p12 | control bit-1_latchT | full | 15 | 2 | 11 | 0 | 1 | 288.9 |
| glide | glide shift3 | full | 60 | 55 | 26 | 0 | 1 | 152.5 |
| smkey | threshold channel0_softF | chroma_dead | 32 | 11 | 19 | 0 | 3 | 191.5 |
| smear | smear default | chroma_dead | 17 | 1 | 14 | 0 | 1 | 209.3 |
| vb | vblur taps5_chromaF_depth_bits11 | chroma_dead | 457 | 254 | 296 | 20 | 7 | 99.6 |
| emb | relief span3_gain3 | full | 92 | 35 | 72 | 0 | 3 | 178.1 |
| key | threshold channel0_softT | chroma_dead | 183 | 133 | 57 | 0 | 3 | 97.8 |
| pal | colorizer entries_bits4_paletteh3eb85c93 | full | 45 | 30 | 42 | 0 | 2 | 180.4 |
| mix | mix implshift_bits4 | full | 537 | 439 | 432 | 0 | 4 | 129.4 |
| wet | mix implshift_bits4 | full | 537 | 439 | 432 | 0 | 4 | 129.4 |
| fringe | chroma_shift pixels2_leadchroma | full | 55 | 0 | 54 | 0 | 1 | 345.1 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | vin.out × 34 bit, taps 3/21 | | 714 | 0 | 714 | 0 | | |
| delay | osc.out × 10 bit, taps 10 | | 100 | 0 | 100 | 0 | | |
| delay | emb.out × 34 bit, taps 3 | | 102 | 0 | 102 | 0 | | |
| delay | pal.out × 34 bit, taps 1 | | 34 | 0 | 34 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **4437** (57.8 %) | 2244 | 3212 | **20** (62.5 %) | 28 | |

Zone: **green** · controls used: 5/12 · program latency: 28 clocks · worst node Fmax: 97.8 MHz
