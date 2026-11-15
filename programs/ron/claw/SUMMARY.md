# CLAW (v0.5.0)

**v0.5.0 (2026-11-14): timing-closed, not flashed.** Three changes Ron asked for, plus one cut he chose:

- **S9 Grab: Auto-close (default) / Manual** (replaces the unused Attract). On Auto-close the claw opens by itself when it starts down (P12 moving down, claw idle and closed). Once it is fully open, it closes by itself when a prize is in reach. "In reach" uses the grab's own test (topmost prize of the claw's row, within K3's tolerance, low enough). The game FSM runs that scan every field as a scan-only pass and grabs nothing: G_START → G_RD0 … G_PICK with `gm_scanonly`, result in `gm_cand`. The real grab then runs on the close edge as before. After an auto-close (or a hand close with S7) the claw won't auto-open again until it rises. S7 still opens and closes by hand in both modes, which is how you let go over the chute. Manual = the old behaviour.
- **S11 Rigged + K6 Rig Odds** (K6 was the unused Sway). At each grab, with S11 on, the LFSR rolls `lfsr(9:0) < K6` (0–100 %). On a rigged grab the prize slips at a random height on the lift: 128–255 virtual lines above where it was grabbed, or at the top of the lift if the grab was too high for that. It falls and tumbles back into the pile like a hand release and never counts as a win. The autopilot (S8 Auto) gets rigged too and simply picks again.
- **Incoming video colours fixed (S10).** The P15 output U/V swap (needed by the generated colours with C_PIX = 16) also hit the camera video, so green came out yellow and red came out blue. The video is now swapped at the input tap as well (p_vid), per CLAUDE.md's "swap at the input latch and the output compose". Pixel depth is unchanged (C_PIX 16, C_LAT 20).
- **Scoreboard cut** (Ron's choice, for room): CREDITS 09 / TRIES / WINS text and the tries/wins BCD counters are gone; the CLAW title and the DROP decal stay. `claw_layout.py` TEXTS keeps ids 0 and 5 only.
- **abc9 topological-order bug, third trigger:** `gm_slipn <= s_cy - rnd - 48` (two chained subtracts, compared against `s_cy` later) killed HD Analog synthesis. Removing either the subtract chain or the compare cleared it (real-config synth-only ablations, `~/claw_work/rabl/rabl.sh`). Fix: one subtract with the offset folded into the operand's bits (`"00000" & '1' & rnd7` = 128 + rnd).
- Build (router1, all seed 1 from the builder, nothing hand-installed): HD Analog 77.23, HD HDMI 74.55 (thin), HD Dual 79.68 MHz; SD 69.9–81.4. ~7370 LC (96 %), 30/32 EBR. `.claw_work/claw_v050.vmprog`.

**v0.4.3 (2026-11-01): timing-closed, not flashed.** The VHS placeholder is redrawn as a real cassette seen from the top (art only; no logic change):

- wide black shell;
- cream label with a thin red rule and two lines of grey handwriting;
- a big smoky window showing a full reel (tape pack cut off by the window edges) and a nearly empty one;
- chrome hubs with three dark spokes, the two reels turned differently (the old matching white hubs with dark centres read as a pair of eyes);
- ribbed grips at both ends;
- the dust door along the front with a sliver of tape.

Preview: `preview/vhs_v043.png`.

- Build (router1, all from the builder, no hand-installed bitstreams): HD Analog seed 1 75.17 (thin), HD HDMI **seed 3** 74.64 (thin), HD Dual seed 1 78.60; SD seed 1 74–80. ~7400 LC (96 %), 30/32 EBR. `.claw_work/claw_v043.vmprog`.

## v0.4.2

**v0.4.2 (2026-11-01): timing-closed, not flashed.**

- **A grabbed prize keeps its pile orientation** (sideways, upside down, mirrored, leaning) while held. The held record now carries the grabbed record's transpose, mirror and tilt bits.
- **Drops tumble every 4 frames, the chute drop included.** The tumble starts from the pickup orientation and turns clockwise: (tr, mir, vflip) → (not tr, not vflip, mir), which covers all 8 flip/rotate states (derived from the stamper's pixel mapping). The prize lands in the pile or the hole in whatever orientation it ended on.
- **Build** (router1, ~7390 LC / 96 %, 30/32 EBR):
  - HD Analog **seed 9, hand-installed** (77.24 MHz). The builder's seeds 1–6 all missed (best 60 MHz) and seeds 7–12 ran 67.6–77.2, so a plain rebuild will NOT reproduce it. To rebuild, run the builder, then `mk.sh hd_analog 9 router1`, copy `bitstreams/hd_analog.bin` into `build/programs/ron/claw/rev_b/bitstreams/` and repack with `vmprog_pack.py --no-sign`.
  - HD Dual seed 4 (81.77), HD HDMI seed 1 (78.21). SD seed 1 (65–78).
  - `.claw_work/claw_v042.vmprog`. The failed builder pack is kept as `claw_v042_HDANALOG_TIMING_FAIL.vmprog`; never flash it.
- **The chip is full and HD timing is seed-luck.** The game FSM's state decode is the routing-bound critical path every time. Before adding anything else: one-hot the game FSM (verify with a grab sim; CLAUDE.md records a yosys FSM-recode miscompile) or split it.

## v0.4.1

**v0.4.1 (2026-10-31): timing-closed, not flashed.** Fixes the colours being flipped on hardware in v0.4.0.

- **Cause:** removing v0.4.0's haze stage made the pixel pipeline one clock shorter (C_PIX 16 → 15), while the syncs stayed at C_LAT = 20. The encoders pair Cb/Cr by counting pixels from hsync, so the avid moving one pixel against hsync swapped U/V. The same thing explains every earlier colour report: v0.1/v0.2 at 15 were swapped, v0.3.0 at 16 with the swap off was swapped, and v0.3.1/v0.3.2 at 16 with the swap on were correct.
- **Fix:** a plain output delay register (r15 → r16 → out) restores C_PIX = 16. A comment at the constant says to keep it. (Putting the pad at P11b instead tripped the abc9 topological-order bug in the HD HDMI netlist only.)
- **Build** (router1, ~7390 LC / 96 %, 30/32 EBR): HD Analog **seed 3** 79.30, HD HDMI 79.68, HD Dual 81.49 MHz; SD 72–81 (seed 1). `.claw_work/claw_v041.vmprog`.

## v0.4.0

**v0.4.0 (2026-10-31): timing-closed (seed-hunted), sim-checked, not flashed.**

- **The WIN animation is the marquee bulbs only.** When a won prize lands in the hole, the bulbs flash for ~3 s: alternate bulbs swap every 2 fields, with all-on bursts every 32 fields. The normal chase resumes after.
- **S8 Auto: the machine plays itself.** The autopilot drives the same glide targets and open/close events a player would:
  - The game logic reads a random pile record (skipping hidden ones) as the target. The autopilot opens the claw, rises, glides over it and sets the depth row to the prize's row (claw scale and joystick rock follow).
  - It lowers to that row's reach, closes (30 fields), lifts, and carries a held prize to the chute and releases. A miss goes straight back to picking. It waits 70 fields, then repeats.
  - "Arrived" reuses the glide's own target distances (f_dx / f_dy < 4 units). S7 is ignored in Auto.
  - S8 labels are now Manual / Auto (bit 1 = Auto).
- **Removed for room (Ron's choice):** the back-row depth darkening (haze), and its pipeline stage.
- **abc9 topological-order bug, hit twice:**
  1. The autopilot loaded `ap_x` from an adder in one arm and `s_vw − 200` in another. Fixed with a registered chute x.
  2. The glide targets had an adder result in one state and the autopilot's target in another. Fixed as one add of gated operands.
- `run_abl.sh` now deletes stale stats; failed syntheses had been printing old numbers.
- **Build** (router1, ~7350 LC / **95.7 %**, 30/32 EBR). The game FSM's state decode is the routing-bound critical path, seed-sensitive at this fill:

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | **2** | 80.50 MHz |
| SD Analog | 1 | 73.60 MHz |
| HD HDMI | **4** | 76.10 MHz |
| SD HDMI | 1 | 69.26 MHz |
| HD Dual | 1 | **74.28 MHz (0.03 MHz margin)** |
| SD Dual | 1 | 69.87 MHz |

  `.claw_work/claw_v040.vmprog`. The chip is full: nothing more fits without a cut or an FSM restructure.

## v0.3.2

**v0.3.2 (2026-10-31): timing-closed, sim-checked, not flashed.**

- **The control panel (centre prize-door window) is static.** It always shows the synth view, the joystick stands on it, and won prizes no longer go there.
- **New prize hole** on the base under the chute: the same size as the panel window (x VW−350..VW−50, y 972..1060).
  - A won prize falls through the chute (flap open) and slides to the chute centre as it falls. It drops behind the base and lands in the hole, where it stays until the next win. Then WINS counts up and its pile slot refills.
  - The hole is a hardwired 17th element in the per-line sequencer (the table is 16 entries), on slot 3 (rows disjoint from the carriage and chute).
- **Cable reversed:** slider down = claw down. The default is now slider up (claw raised).
- S11 Rigged, S8 Mode, S9 Attract and K6 Sway are still not wired (Ron asked; the Rigged plan is in the reply).
- Build: router1, all seed 1. ~7265 LC (**94.6 %**), 30/32 EBR. HD Analog 77.32, HD HDMI 74.57 (thin), HD Dual 77.58 MHz; SD 79–83. `.claw_work/claw_v032.vmprog`.

## v0.3.1

**v0.3.1 (2026-10-31): timing-closed, sim-checked, not flashed.** Corrections after Ron flashed v0.3.0.

- **The U/V swap is back on.** v0.3.0 without it hue-swapped every prize on hardware: the red hat came out blue, purple came out red and yellow came out light cyan. **This hardware needs the output swap, as in v0.2.0.** v0.3.0's reasoning (the generated avid makes it unnecessary) was wrong. Still open: Ron saw the joystick ball blue on v0.2.0 even with the swap on, so check the ball's colour on this build.
- **Joystick restored to v0.2.0.** It is the same element-based stick (64 rows, pivot at the bottom), the same lean and the same rock. The only change is the ball, +15 % (r 18; 14 / 21 in the back / front rock poses). It is drawn on top of the prize-door window (Ron's "control panel"), with the stick's base at the centre of the window's top band, in front of everything. v0.3.0's centred long stick, its boot and its direct-logic version are all gone.
- **The prize door window is back in the centre.**
- **Every prize type is guaranteed to appear in the pile.** The LZX token and the wizard are placed in the front rows. In v0.3.0 the rare token got no slot by chance.
- Build: router1, all seed 1 except HD Dual seed 2. **~7170 LC (93 %)**, 30/32 EBR. HD Analog 75.09 MHz (thin), HD HDMI 76.65, HD Dual 81.40, SD 69–84. `.claw_work/claw_v031.vmprog`.

## v0.3.0

**v0.3.0 (2026-10-31): timing-closed, sim-checked, not flashed.** Built from Ron's feedback on the flashed v0.2.0.

## v0.3.0 changes

- **No U/V swap (WRONG; reverted in v0.3.1).** On Ron's v0.2.0 hardware, red showed as blue: the Classic base panel looked light blue and the red ball blue. CLAW's generated avid already keeps the encoders' Cb/Cr pairing straight, so the "usual" output swap double-swapped. `C_HW_UV_SWAP` is now false, and every cabinet colour should now match the previews.
- **Ron's 4 supplied prizes** (art/supplied/): lzx_token, eyeball, redhat and wizard.
  - They replace the Spark and Colour Bars placeholders: 8 types = TV, VHS, DVD, LZX token, eyeball, synth, red hat, wizard.
  - The sprite ROM is 8k + 8k + 2k px (18 EBR), **30/32 EBR total**.
  - The art is used pixel-exact. Colours are exact: 25 supplied colours went into master slots freed by removing the spark/bars-only colours. The eyeball's 16 colours were merged to 15 (least-used into nearest).
  - Hidden pile records are now y = 2047, because type 7 is a real prize now.
- **Palette:** 64 master colours. The depth haze is arithmetic after the palette (row 1 ×7/8, row 2 ×3/4 toward black / neutral) instead of stored darkened copies. The line buffer holds {depth, colour}. Themes moved to palette base 64.
- **Joystick:**
  - Centred on the control panel, rising over the play field, drawn in front of everything (bezel, glass, pile).
  - The stick is direct logic (narrow item, per-line lean, black boot at the base). The ball element shares slot 2 with the rail, hub and decal (disjoint rows).
  - Ball +15 % (r 18; 15 / 21 in the back / front rock poses).
- **The prize door** moved left of the joystick (C−480..C−180).
- **DROP** replaces PUSH and flashes (white body, red letters) while the claw is descending.
- **Text zones** start on 32/64-unit cells: char and column come from x bits. At SD the title sits 16 units left of centre and the decal text 12 units right.
- **Bulbs** use a 64-entry shape table instead of a squaring multiplier (−90 cells).
- Bug caught by sim: `resize()` narrowing of a signed sum in the haze folded every colour to green (CLAUDE.md trap). It is a slice now.

## Build v0.3.0 (2026-10-31, router1, ~7345 LC / **96 %**, **30/32 EBR**)

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 80.02 MHz | 7343 |
| SD Analog | 1 | 80.68 MHz | 7339 |
| HD HDMI | 1 | 76.91 MHz | 7326 |
| SD HDMI | 1 | 65.23 MHz | 7348 |
| HD Dual | 1 | 76.35 MHz | 7346 |
| SD Dual | 1 | 69.08 MHz | 7351 |

**The chip is full.** Nothing more fits without a trim; see the cut menu in DESIGN.md §3H. `.claw_work/claw_v030.vmprog`.


**Status: v0.2.0 timing-closed and sim-checked, NOT flashed** (2026-10-31). v0.1.0 was flashed by Ron: it runs, but the claw couldn't reach the chute.

## v0.2.0 changes (Ron's v0.1.0 feedback)

- **The claw reaches the chute.** v0.1.0's gantry stopped at x = VW − 330, while the win test wanted x ≥ VW − 300, so the chute was unreachable. The gantry now travels to VW − 200 (the chute centre), and the win zone is the chute opening (x ≥ VW − 340).
- **Real drop.**
  - Opening the claw away from the chute lets the prize fall with gravity (+3 units/frame², capped at 40).
  - It tumbles a quarter turn every 4 frames and bounces once (−1/4 speed).
  - It lands at its depth row's height (plus a little random jitter) under where it was dropped, and becomes a pile prize there, in whatever orientation it landed.
- **Chute drop.** Over the chute, the prize falls behind the acrylic with the flap swung open (hidden). On reaching the chute floor: WINS counts up, the prize goes into the door window, and its slot refills with a random prize.
- **Depth shows on the claw.** The hub, prongs and cable scale 1.0 / 0.82 / 0.66 for Front / Middle / Back, from per-depth offset tables. Reach and hold point scale with them.
- **Joystick on the base panel:** chrome stick, red ball.
  - It leans with the gantry's actual speed (±8 lean steps, gliding one step a frame), so it springs back when the claw stops.
  - A K2 row change rocks it for ~0.4 s: toward the front, the stick foreshortens and the ball grows; toward the back, the ball shrinks.
- **Quarter-turn rotation in the stamper.** A transposed prize walks the sprite ROM down a column (address step ±48 instead of ±1); vertical flip reads row 47 − v. This gives 0/90/180/270°. About 20 % of the pile starts sideways or upside down, and dropped prizes keep their landing orientation.
- **Fit and timing:**
  - The cable is now a narrow direct item: one subtract from a per-frame left edge plus 4-bit tests, with no slot registers. That saved ~300 cells.
  - The flap is the chute element's top band (22 rows, outline below).
  - The joystick base plate was dropped to keep the element table at 16 entries.
  - Game decisions are pre-registered every clock (the state → enable cones were routing-bound).
  - Joystick ball x is one op per state.
  - A real bug, caught by sim: the sequencer's element index was still `range 0 to 15` with 19 elements, which in synthesis would alias the joystick onto the marquee.
  - This netlist routes much better on **router1** (router2: HD 65–77 MHz).

## Build v0.2.0 (2026-10-31, **router1**, full clock, ~6945 LC / 90 %, 26/32 EBR)

`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron claw`

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 80.44 MHz | 6942 |
| SD Analog | 1 | 78.90 MHz | 6945 |
| HD HDMI | 1 | 81.23 MHz | 6942 |
| SD HDMI | 1 | 80.14 MHz | 6960 |
| HD Dual | **2** | 79.25 MHz | 6949 |
| SD Dual | 1 | 74.64 MHz | 6945 |

`out/rev_b/ron/claw.vmprog` (copy in `.claw_work/claw_v020.vmprog`; v0.1.0 in `claw_v010.vmprog`).

## Hardware checklist v0.2.0

1. K1 all the way right puts the claw over the chute.
2. **Grab and carry to the chute, then open:**
   - the prize drops inside the acrylic chute (the flap bar disappears while it falls);
   - WINS counts up;
   - the prize appears in the door window;
   - a new prize fills its pile slot.
3. **Grab, then open anywhere else:** the prize falls, tumbling in quarter turns, bounces once, and settles into the pile under the drop point (possibly sideways or upside down).
4. **K2 Front → Middle → Back:** the claw gets visibly smaller each step, and the joystick rocks forward or back for a moment on each change.
5. **K1 moving:** the joystick ball leans toward the direction of travel and springs back upright when the claw stops.
6. A few pile prizes are sideways or upside down from the start.

---

## v0.1.0 (first build)

Status at the time: timing-closed, sim-checked, NOT flashed (2026-10-31). Built overnight from Ron's CLAW spec. All look and scope compromises are listed in DESIGN.md §3, with the questions for Ron in §7.

An arcade claw machine seen through the glass, filling the screen:

- marquee with a chunky outlined title, chasing bulbs, and CREDITS / TRIES / WINS;
- chrome bezel, back-wall gradient and glass streaks;
- gantry rail and carriage, cable, and a three-prong chrome claw;
- a heaped 4-row pile of 48×48 4bpp video-toy prizes (6 types);
- an acrylic prize chute with a flap;
- the cabinet base, with a prize-door window and a PUSH decal.

## What works in v0.1.0

- **Static scene** at 1080p, 720p and SD (4:3 window), in 6 cabinet themes.
- **Claw motion:**
  - K1 gantry X and P12 cable, both glided at the K4 speed;
  - K2 depth row (the claw passes behind the rows in front of it);
  - S7 (any edge) animates the prongs open or closed.
- **Grab and carry (phase 3-lite):**
  - Closing the claw low over a prize grabs the topmost reachable prize in the claw's depth row, if the claw is centred within K3 Grip's tolerance (20–83 units). TRIES counts every close.
  - The prize then rides with the claw.
  - Opening over the chute (claw x ≥ VW − 300) is a WIN: WINS counts up, the prize goes into the door window, and its pile slot refills with a random prize.
  - Opening anywhere else drops the prize back into its slot (instantly; there is no tumble yet).
- **S10:** live input video, dimmed, on the back wall.
- **Not yet wired** (later phases): K6 sway, S8 mode, S9 attract, S11 rigged, the sequences (win, jackpot, miss, drop, tilt, attract), and tumble physics.

## Build v0.1.0 (2026-10-31, router2, full clock, ~6330 LC / 82 %, 26/32 EBR)

`./build_programs.sh ron claw`. Every config passes at seed 1.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 78.83 MHz | 6331 |
| SD Analog | 1 | 78.32 MHz | 6319 |
| HD HDMI | 1 | 80.94 MHz | 6334 |
| SD HDMI | 1 | 78.74 MHz | 6321 |
| HD Dual | 1 | 79.69 MHz | 6322 |
| SD Dual | 1 | 87.27 MHz | 6320 |

`out/rev_b/ron/claw.vmprog` (copy in `.claw_work/claw_v010.vmprog`).

## Hardware checklist (first flash)

1. **Picture:**
   - The whole cabinet fills the screen at 1080p.
   - No green or magenta in the blanking, no rolling, no thick HDMI bars. Output conditioning is built in.
   - Hues match the preview PNGs (palette is BT.601 internally, swapped for HW at the output).
2. **K5** steps through Classic, Neon, Jackpot, Vaporwave, Mint and Studio. A fresh load should open on Classic.
3. **K1** glides the gantry. **P12** lowers and raises the claw. **K4** sets how snappy both are.
4. **S7** flips open or closed the claw.
5. **K2:** in Back, the claw lowered into the pile goes behind the front and middle rows. In Front it is in front of everything.
6. **Grab and carry:**
   - Lower over a front-row prize with K2 = Front, flip S7 (close). The prize should come up with the claw.
   - Carry it right, over the chute, and flip S7 (open). WINS should count up, the prize should appear in the door window, and a new prize should fill the hole.
   - Opening elsewhere puts the prize back.
7. **S10** puts the input video on the back wall.
8. **720p and SD** should show the same picture. SD is a 4:3 window with 5 prizes per row.
9. **SD check:** the NTSC sim shows thin combing on moving edges and text, from its interlaced stimulus. On hardware the core ties field_n high, so SD should be clean (line-doubled).

## Engineering notes (v0.1.0)

- **First build was 557 % LC:** the four line-buffer RAMs had three write statements each (wipe, clear, stamp) and became 16 k flip-flops. One registered (addr, data, we) mux per bank fixed it (CLAUDE.md BRAM rule).
- **Second build missed HD (62–69 MHz) at 89 % LC.** Three things fixed it:
  1. **Per-line extras:** a `s_vy` → subtract → carry compare → enable cone. They are now free-running subtracts plus bit-logic flags, latched 5 clocks after vy settles.
  2. **About 40 constant compares** as `f_ltu` / `f_lts` bit logic (the invaders v0.7.7 lesson): carry cells 1516 → 1220.
  3. **The vblank glide** (barrel shift → zero-fix → add in one clock) split into one op per state, and the game scan's abs → compare split the same way.

  Result: 89 % → 82 % LC, HD 79–81 MHz at seed 1.
- **Slot compares** are 11-bit unsigned: bounds are clamped to 0..2047 in the sequencer. The left-highlight compare is kept only on the slots that show it (hub, rail, decal, carriage, chute, prongs, cable).
- **Sim artifacts met while testing the grab with forced initial state:**
  - a sed that also hit the Y-target constant (claw drifted down);
  - a static P12 that lifted the claw during the close frame (reach moved above the prize).

  Neither is an RTL bug. Use line-scoped sed (`/signal f_yq/s/...`) for sim-only initial-state edits.

## Files

- `claw.vhd`: the program.
- `claw_pkg.vhd`: generated by `art/claw_art.py`. Do not hand-edit.
- `claw.toml`
- `DESIGN.md`: budget, compromises, roadmap, questions.
- `art/claw_layout.py`: the single source of truth for palette, themes, elements, pile and text.
- `art/claw_art.py`: sprites, font, palette and tables → pkg, plus the contact sheet.
- `art/claw_ref.py`: the Python reference renderer (same integer rules).
- `art/supplied/`: drop PNGs here to replace fallbacks (videomancer.png, etc.).
- `preview/`:
  - `prize_contact_sheet.png`, `font_sheet.png`;
  - `ref_1080p_*.png`, `themes_ref.png`;
  - `sim_*.png` (GHDL renders of the actual RTL).
