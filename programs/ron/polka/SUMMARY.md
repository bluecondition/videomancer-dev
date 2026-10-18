
## Community port — fit for the stock SDK (v2.2.2, 2026-10-15)
- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. v2.2.1 packed to 8170/7680 LC there (HD Dual 7580).
- v2.2.2, three bit-exact changes:
  1. **Square ROMs.** The two per-pixel 8-bit squares (c4) are 256 x 16 ROMs
     (1 EBR each). Each is addressed by the |dx| that used to load c3, so the
     EBR's address register is the c3 stage.
  2. **Per-frame palette table.** A pixel's dot colour depends only on its
     palette entry (`v_e`, 0..3) and the per-frame `s_pal` / `s_hueoff`. So the
     two per-pixel 10 x 10 saturation multiplies (c14, ~500 LUT) became a
     4-entry u/v table (`pt_u`/`pt_v`), filled in vblank (FSM states 373-507)
     through the same pixel-path hijack the background colour uses, on the
     FSM's serial multiplier. The table keeps the pixel path's rounding: a
     negative product rounds down (floor), so 512 minus the ceiling.
  3. **Booth blends.** The three coverage blends (11-bit signed d x 6-bit
     coverage) are a hand-written radix-4 Booth (`booth_d11_c6`), checked
     exhaustively against d*w.
- Proven bit-exact against v2.2.1: GHDL side-by-side, 40 frames (403 x 201
  raster), knobs and switches changing mid-picture, 0 mismatches in 3.24 M clocks.
- Packed LC (stock SDK): SD Standalone 8170 → 7543, HD Dual 7580 → 6950.
- Stock build: all 8 configs on seed 1; own HD timing (core without the Sync
  Out generator) 79.0/86.3/88.9 MHz Analog/HDMI/Dual.

## Resolution scaling (v2.2.3, 2026-10-17)
- Interlace and the pixel-aspect step now come from the video timing ID
  (`registers_in(8)`): the core ties field_n high, so the old field_n-toggle
  detector never fired and 480i/576i drew 2:1 tall dots.
- Sizes follow screen HEIGHT: dot rmax, max pitch (Wref/16, Wref/6), the
  pitch floor (20/13/10 px), the radius floor (2/1/1) and the sway speed use
  a 16:9 reference width (act_w on HD, 4/3 act_w on SD). SD shows the 1080p
  picture's 4:3 centre with round dots of the same size; 720p is 1080p at 2/3
  (the old fixed 20 px floor made 720p pitch ~10 % coarse at low Spacing).
- 1080p bit-identical (diff1080 0 px). Glide-bypassed sim copies: SD/720p
  dot size and pitch within 2 % of 1080p (relative to height), aspect 1.0.
- Build (this repo, 6 configs): `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron polka`
  — all six on seed 1: HD Analog 87.95, HD HDMI 85.99, HD Dual 83.33 MHz;
  SD 79.99 / 83.73 / 83.35 MHz; 6759-6798 LC (88 %), 10 EBR.
