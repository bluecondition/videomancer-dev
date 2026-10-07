# Chroma Wash: resource estimate

| Node | Type / config | Variant | LC | LUT | FF | EBR | Lat | Fmax |
|---|---|---|---|---|---|---|---|---|
| vin | video_in default | full | 35 | 0 | 34 | 0 | 1 | 626.6 |
| k1 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k2 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k3 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k4 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k5 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| k6 | control bit-1_latchF | full | 11 | 0 | 10 | 0 | 1 | 626.6 |
| kglide | glide shift2 | full | 59 | 51 | 25 | 0 | 1 | 124.8 |
| p12 | control bit-1_latchT | full | 15 | 2 | 11 | 0 | 1 | 288.9 |
| glide | glide shift2 | full | 59 | 51 | 25 | 0 | 1 | 124.8 |
| pa | procamp implshift_bits5 | full | 689 | 520 | 465 | 0 | 5 | 118.1 |
| hb | hblur taps5_chromaT | full | 786 | 570 | 559 | 0 | 7 | 109.5 |
| vb | vblur taps2_chromaT_depth_bits11 | full | 424 | 118 | 608 | 15 | 7 | 170.6 |
| key | threshold channel2_softT | chroma_dead | 183 | 133 | 57 | 0 | 3 | 92.1 |
| post | posterize bits4_chromaT | full | 21 | 6 | 15 | 0 | 1 | 309.6 |
| pal | colorizer entries_bits2 | full | 22 | 3 | 18 | 0 | 2 | 180.4 |
| keymix | mix implshift_bits6 | full | 732 | 654 | 446 | 0 | 4 | 98 |
| wetmix | mix implshift_bits4 | full | 537 | 439 | 432 | 0 | 4 | 129.4 |
| vout | video_out default | full | 37 | 1 | 34 | 0 | 1 | 338.9 |
| delay | post.out × 34 bit, taps 2/4 | | 136 | 0 | 136 | 0 | | |
| delay | key.mask × 10 bit, taps 2 | | 20 | 0 | 20 | 0 | | |
| delay | vin.out × 34 bit, taps 28 | | 952 | 0 | 952 | 0 | | |
| shell | passthru baseline | | 1145 | 660 | 624 | 0 | | |
| **total** | | | **5918** (77.1 %) | 3208 | 4521 | **15** (46.9 %) | 34 | |

Zone: **green** · controls used: 7/12 · program latency: 34 clocks · worst node Fmax: 92.1 MHz
