# OUBLIETTE (v1.0.0 — top-down turn-based roguelike)

**STATUS: FINAL — signed off by Ron on 2026-10-10** ("I think we can
finalize oubliette"). v1.0.0 is the v0.5.0 bitstreams with the TOML version
bumped (TOML-only repack, no re-route): router1, HD Dual on seed 2 (74.69
MHz, see Build). History: v0.2.0 hardware contracts, v0.3.0 finishable game
(casts, potions, dragon), v0.4.x fog / zoom / map / glide (HW-approved),
v0.5.0 furniture, HP bar, palette themes, title screen and the colour-parity
fix. Presets are four hand-written class presets; capture real ones from the
device with `./capture_preset.sh` if wanted.

## Controls

| Control | Function |
|---|---|
| K1 Class | Warrior / Mage / Rogue / Cleric (labelled; read at game start, the HUD previews its stats) |
| K2 Seed | dungeon seed (with depth) |
| K3 Action | combat action (labelled): Attack / Defend / Cast / Item / Flee |
| K4 Zoom | Far / Wide / Normal / Close = 16 / 32 / 64 / 128 px cells (labelled) |
| K5 Torch | light radius 2-8 cells; the top of the travel = full light (no fog) |
| K6 Palette | Colour / Game Boy / Amber / CGA / C64 (labelled) |
| S7 Step/Act | each flip = one turn (step, attack, confirm), acted on at the next field |
| S8 Map | the whole 64×64 level at 16 px cells (fog shows what you've explored) |
| S9 Potion | each flip drinks a potion (+max HP/2): a turn when exploring, the round's action in combat |
| S10 Peaceful | monsters never move or attack (default Off) |
| S11 New Game | each flip restarts at class select |
| P12 Heading | 8-way heading: the top of the travel is N; pushing up from the bottom turns clockwise NE, E, SE, S, SW, W, NW, N (v0.5.0) |

All twelve controls are in use.

## Rules (v0.3.0)

- **Classes** (HP / ATK / DEF / MP, cast cost): Warrior 24/6/4/6 BASH (3 MP:
  a normal hit and the monster loses its turn), Mage 16/4/2/12 FIRE (4 MP:
  ATK+8 ignoring DEF), Rogue 18/5/3/6 STAB (2 MP: 50% crit for 2×ATK; flee
  always works), Cleric 20/4/3/10 HEAL (4 MP: +max HP/2).
- **Combat round**: Attack = ATK+0..3−DEF (min 1). Defend halves the hit
  taken and gives +2 MP. Cast without the MP, or Item with no potion, wastes
  the round. Flee 50% (Rogue 100%); a successful flee gives two free turns.
- **MP** regenerates +1 every 4th step. **Potions**: start with 1, chests give
  one half the time (plus gold), kills drop one 1 in 4, max 9.
- **Level-up** at level×8 XP: +3 max HP, +1 ATK, +1 DEF on even levels,
  +1 max MP, full HP and MP.
- **Depth**: monster types are drawn from a per-depth table (rats/bats
  shallow, orcs/wraiths deep); monster HP +2, ATK +1, DEF +½ and XP +1 per
  depth below 1. A sword (+2 ATK) on odd depths, a second chest on even ones.
- **Monsters** chase within 6 cells (else wander half the turns), and a
  blocked chase step retries once along the other axis.
- **Depth 7** has no stairs: the dragon (HP 60, ATK 16, DEF 7 with the
  depth bonus) guards the last room and never moves. Killing it wins; gold
  is the score.

## v0.5.0 (2026-10-10) — Tier 3 batch B (look) + colour fix

- **Colour fix (all versions since v0.2.0 showed U/V swapped).** Ron
  described the yellow reticle as "cyan": exactly yellow with U and V
  swapped. Oubliette's generated avid left one clock earlier relative to
  hsync than aurora's (same p_gavid code): opposite chroma pairing at the
  encoder. C_SYNCD 5 → 4 puts it at aurora's distance (avid − hsync = the
  same g offset), where the U/V-swapped palette is HW-correct. Expect the
  designed hues: yellow reticle, blue water, orange lava, red warrior.
- **Heading:** P12 top = N (zone + 1), same rotation order (Ron's request).
- **Room furniture:** each room rolls plain / pillar grid / pool (water, or
  lava from depth 4) / rubble; corridors (carved after rooms) cut through,
  so connectivity holds. Corridor carving now reads each cell (4 states per
  cell): the first room-wall cell of a crossing becomes a door.
- **HP bar:** right of the bottom HUD text, 3 px per HP point (frame =
  max HP), green / amber / red at 1/2 and 1/4. HUD bands are now full width
  (per-line flags + a line counter instead of row compares).
- **K6 palette themes:** new S8 stage, luma quantised to 4 levels → a
  4-colour table per theme (Game Boy, Amber, CGA, C64), active video only.
  In Colour, the tiles take a per-depth tint (crypt, ice, moss, amethyst,
  sea, ember, hellfire) in S7.
- **Title screen:** during class select only the class sprite shows, at the
  Close zoom (128 px), under a 2x "OUBLIETTE" logo band (64 px glyphs from
  the HUD RAM's unused top-line columns 48-56, centred on the measured
  screen; the sequencer now streams all 64 columns).
- **Space:** glide variants from the in-cell offset ± f (8-bit adds; the
  carry/borrow moves the column) instead of four 14-bit adders + decodes;
  5-zone knob decoders as a 64-entry table on the top 6 bits instead of
  eight carry-chain compares; margin tests as bit tests; the colour stage
  is a 128-entry palette ROM (EBR) with dimmed colours precomputed.
- Pipeline +1 (S8): syncs 10 clocks, avid g+10 (same relative distance).

## v0.4.1 (2026-10-10) — monster glide + space reclaim

- **Monsters glide** between cells like the player (Ron's request on
  v0.4.0). All entities share one glide distance s_mgf: one cell at the
  turn's monster phase, counting down 1/8 cell per field. Each monster keeps
  its last step direction and is drawn pulled back toward its old cell by
  s_mgf; the render decodes world+f and world-f once per pixel, and each
  entity compares its cell against the one shifted coordinate its direction
  picks (no per-monster arithmetic). The player is the same kind of entity,
  so the separate player coordinate system is gone.
- **Camera** = target − (player's step) × s_mgf: the 14-bit compare-based
  glide sequencer is gone. A turn starting on a tick zeroes s_mgf first,
  so a glide still running finishes instantly and everything stays in step.
- **Space reclaimed** to pay for it: `no_rw_check` on the tilemap and HUD
  char RAM (emulate_transparency 2 → 0); the border band uses equality-flipped
  flags instead of magnitude compares; the HUD's name glyphs (class,
  monster, action, result) moved into layout ROM rows 28-31 and are copied
  into the field RAM by the sequencer, replacing 13 name registers and their
  lookup tables. Net vs v0.4.0 (standalone yosys): LUT +137, CARRY −74,
  FF +36, i.e. about the same size with monster glide added.
- Ablation (standalone yosys, v0.4.1 before the reclaim, savings if
  removed): glide rendering −438 LUT / −99 carry / −216 FF; whole HUD
  −466 / −52 / −253; camera glide sequencer (since replaced) −239 / −162 /
  −39; border −43 / −66 / −32; fog −79 / −47 / −31.

## v0.4.0 (2026-10-10) — Tier 3 batch A (feel)

- **Fog of war + torchlight.** A 4096×1 seen-cell RAM (2 EBR) is marked
  once per field (a reveal sweep in vblank, every cell within the torch
  radius) and cleared with the level. Per pixel, the cell's distance from
  the player (squares from a 16-entry LUT) classifies it: torch core (fully
  lit), ring (lit on a checkerboard of art pixels: a dithered edge),
  remembered (dimmed and desaturated in a new S7 stage: y/2+32, c/4+384) or
  unexplored (black). Monsters show only in the light. Distance is
  Euclidean on cells, no line of sight (a magic torch).
- **Zoom / Map.** Cell size 16 << s_cl px, picked per field: the cell decode
  is a 4-way choice of bit slices (`cell_dec`). Map view = 16 px cells with
  the camera on the map centre.
- **Smooth walking.** The camera target (player centred) is refreshed
  through vblank; the world scroll glides toward it 1/8 cell per field,
  snapping on jumps over 2 cells. The player sprite is decoded in target
  coordinates, so it stays at the screen centre and walks while the world
  slides; it bobs while gliding.
- **Hit feedback.** The combat target flashes white for 6 fields when hit; a
  24 px screen-edge band flashes red for 6 fields when you are hurt and
  pulses dark red (16 on / 16 off) at HP <= max/4.
- **Facing.** Sprites are drawn facing right; the player mirrors for the
  westward headings, monsters mirror to face the player; monsters bob one
  art pixel on alternating 32-field beats (row 15 of every sprite is blank,
  so the bob's row wrap is invisible).
- Pipeline +1 stage (S7): syncs 10 clocks in every mode.
- TOML: K4 Zoom (labelled), K5 Torch, S8 Map, v0.4.0.

## v0.3.0 (2026-10-10) — finish the game (Tier 2)

- Everything in Rules above, plus the HUD rework: all text now comes from a
  generated layout ROM (`oubliette_hud_pkg`, edit the strings in
  `oubliette.py`), with `{field}` tokens filled from a 64-slot field RAM by
  a once-per-field sequencer (numbers via the BCD ROM, then name glyphs,
  then both 48-column lines streamed into the HUD char RAM). The top line
  shows class/ATK/DEF/XP while exploring, pickup / kill / level-up / depth
  messages, the fight line (monster HP, action, result, damage dealt and
  taken), and the death and victory screens.
- New sprite: the dragon replaces the unused chest sprite (index 10 =
  monster type 6).
- TOML: K3 labelled, S9 "Potion", K4/K5/K6/S8 removed, v0.3.0.

## v0.2.0 (2026-10-10) — hardware contracts

- **Generated avid** (aurora/cubist parity-locked recipe). The program draws
  its own full-colour picture; passing the source avid through gave thin
  U/V-swap lines on aurora v3.0.2 (HD HDMI). The pixel pipeline counts the
  generated avid and carries it to `data_out.avid`.
- **Blanking gate**: outside avid the select stage forces `SRC_BLANK`
  (64/512/512). Previously the held x counter painted tile colour through
  hblank and the last line through vblank.
- **Studio-range palette** (`oubliette.py yuv_swapped`): Y = 64 + y·876 with
  luma above 640 knee-compressed toward 768 (chroma scaled by the same
  factor, which keeps the colour in gamut), chroma ±448. It was 0..1023
  luma and ±511 chroma: bright tiles at Y 871–912, HUD ink at 990
  (superwhite), void at 20 and HUD bar at 40 (below black).
- **Camera one frame late on every step**: the scroll was latched at the
  same vsync that started the turn, so for one frame the player sat a cell
  off-centre and then the map jumped. The scroll now refreshes every clock
  through vertical blank.
- **Field tick with saw-active guard**: every S7 action (turn, combat round,
  game start, continue after death), the HUD rewrite and the message timer
  run once per field, inside vblank. Combat used to resolve mid-frame (sprite
  tear on a kill); the level generator now finishes within vblank.
- **Hysteresis** on P12 heading (16 of 128 codes), K3 action (16 codes) and
  K1 class (24 of 256), so pot noise at a zone edge can't flicker the reticle
  or send a step the wrong way.
- **Labelled K1** with the index fix (a register value below 4 is the label
  index the firmware writes at load).
- **Generator fixes**: stairs placed last (coincident room centres used to
  let the weapon/chest overwrite the only exit, a soft-lock); room x/y draw
  overflowed its integer range (GHDL bound failure, silent wrap in
  synthesis); the stairs-room monster now stands beside the stairs, not on
  them; the last two columns of every line read the game-logic map address
  (the map-port select was the raw avid, 2 clocks early).
- TOML: Peaceful defaults Off; four presets, one per class with its own seed.
- **Timing: the vblank generator FSM was the critical path.** The first
  v0.2.0 build missed HD Analog on all 6 router2 seeds (best 67 MHz): the
  spawn state put `s_gi + 3` in front of the room-table mux, then the new
  `- 1`, then the slot decode, all in one state (10 ns of routing). Fixes:
  spawn split into latch (GN_MON) + one-hot slot write (GN_MONW) with its own
  room counter; room draw split into GN_RPICK → RLIM → RCLAMP → RSET (four
  chained adds were in one state since v0.1); corridor setup latches both
  centres (registered previous-room index `s_gp`, no `s_gi - 1`) then orders
  them (GN_CORR → CORR2). Same rooms, same mapping, one op per state.

## First hardware session — checklist

Video (v0.2.0 contracts):
- No thin coloured lines across the map or HUD (HD HDMI especially); hues
  look right (walls grey-brown, grass green, water blue, warrior red).
- Black void is true black; HUD text legible, not blooming.
- Camera stays centred on the player on every step (no one-frame jump).
- Heading reticle steady at every slider position, including zone edges.

Game (v0.3.0):
- One S7 flip = exactly one turn / one combat round; one S9 flip = one potion.
- Class select previews each class's ATK/DEF/HP/MP; the K3 panel label
  matches the action shown on the fight line.
- Fight line shows the result word and the damage dealt and taken; casts
  spend MP; a kill shows "SLAIN  XP +n"; level-ups show.
- Monsters far away wander; near ones chase and get round corners.
- Every floor 1-6 has reachable stairs; depth 7 shows "THE DRAGON AWAITS";
  killing the dragon shows the victory line.
- Difficulty feel: is depth 4-7 a real fight? Is the dragon beatable with
  potions? (numbers are first guesses: the tables at the top of
  oubliette.vhd)

Feel (v0.4.0):
- Torch: the lit circle follows the player, its edge is dithered, explored
  areas stay dimmed, monsters disappear outside the light; K5 top = no fog.
- K4 zoom levels all look right (no misaligned sprites or tiles); S8 shows
  the whole level and flips back cleanly.
- Steps glide (about 1/8 s per cell) with the player centred and bobbing;
  new levels and zoom/map changes snap, not glide.
- Hit flash on the monster, red border when hit, pulsing border at low HP.
- The player turns to face westward headings; monsters face the player and
  bob.

Batch B (v0.5.0):
- Colours: the reticle is now YELLOW (was cyan); water blue, lava orange,
  grass green, warrior red, mage blue (all were U/V-swapped before).
- Slider top = N.
- Rooms with pillars, pools (lava deeper down) and rubble; doors where
  corridors meet rooms; every room still reachable.
- HP bar right of the status text changes colour at half and quarter HP.
- K6: Game Boy / Amber / CGA / C64 look right and blanking stays black;
  Colour: each depth has its own tint.
- Class select: big class sprite under an OUBLIETTE logo, nothing else.

Glide (v0.4.1):
- Monsters slide cell to cell in step with the player; a monster that
  sidesteps round a corner slides along the axis it actually moved.
- Flipping S7 quickly: no jumps or the player drifting off-centre (each
  new turn finishes the previous glide instantly).
- Zoom / map / new level still jump, not slide.

## Build

**Build with router1:**
`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron oubliette`.
With router2 (the builder default) the routing-bound vblank FSM spreads
65–80 MHz across seeds, and HD HDMI missed all 6 seeds even after the
spawn fix.

v0.5.0 (2026-10-10), router1:

| Config | Seed | Fmax (MHz) | LC |
|---|---|---|---|
| HD Analog | 1 | 77.03 | 7013 |
| SD Analog | 1 | 77.49 | 7014 |
| HD HDMI | 1 | 78.00 | 7014 |
| SD HDMI | 1 | 68.86 | 7004 |
| HD Dual | **2** | 74.69 | 7010 |
| SD Dual | 1 | 76.58 | 7020 |

RAMs 20/32, **~91% LC**. HD Dual passes only on seed 2 with 0.6% margin
(router1 is deterministic: the builder's seed retry lands it). The first
v0.5.0 build hit 94% LC (7233) and failed HD Analog on every seed (68.5 MHz,
routing congestion on the x-band flags). Fixed by the palette ROM (the S6
colour mux + S7 dim adders → one 128-entry EBR pair with dim colours
precomputed: −225 LUT) and comparing the registered pixel counter instead
of the line-start-muxed cur_x in the band flags.
Batch B feature costs (standalone ablation, LUT / carry / FF): furniture
94 / 43 / 35, HP bar 104 / 10 / 34, themes 99 / 76 / 8 (76 carries were
the K6 compare decoder, since replaced by the C_Z5 table), tint 16 / 16 / 13,
doors 19 / −3 / 2. There is no room left for more features without
removing something.

v0.4.1 (2026-10-10), router1, all six configs pass on **seed 1**:

| Config | Fmax (MHz) | LC |
|---|---|---|
| HD Analog | 80.37 | 6858 |
| SD Analog | 84.60 | 6870 |
| HD HDMI | 80.63 | 6842 |
| SD HDMI | 82.45 | 6864 |
| HD Dual | 81.23 | 6864 |
| SD Dual | 85.01 | 6879 |

RAMs 18/32, ~89% LC: about v0.4.0's size with monster glide added. The
first v0.4.1 build closed HD Dual at only 74.46 MHz: vblank-FSM routing
from the turn state into the spawn-mask enables (set inside start_gen,
called from the stairs branch of TS_PMOVE) and the potion state's
"HP full" compare. Fixed by deriving the LFSR seed and spawn mask from
s_depth inside GN_CLEAR (start_gen now only hands over state) and a
registered s_hpfull flag. Side effect: each K2 seed now generates a
different dungeon than in v0.4.0 (the LFSR is seeded after the clear).

v0.4.0 (2026-10-10), router1, all six configs pass on **seed 1**:

| Config | Fmax (MHz) | LC |
|---|---|---|
| HD Analog | 80.82 | 6880 |
| SD Analog | 81.75 | 6893 |
| HD HDMI | 85.05 | 6875 |
| SD HDMI | 79.42 | 6876 |
| HD Dual | 78.95 | 6882 |
| SD Dual | 83.32 | 6875 |

RAMs 18/32 (+1: the seen RAM). **~90% LC**: batch A cost ~1,200 LC (the
estimate was ~500), mostly carry chains (1,100 SB_CARRY standalone: the
second coordinate system's adders, the glide compares, the border band).
Before Tier 3 batch B, reclaim room: the tilemap and HUD char RAM still
emulate read-during-write (standalone yosys: emulate_transparency = 2);
`no_rw_check` on both (collision-free: the one same-address case, the
pickup write, has no consumer for that cycle's read) should save ~230 LC.

v0.3.0 (2026-10-10), router1, all six configs pass on **seed 1**:

| Config | Fmax (MHz) | LC |
|---|---|---|
| HD Analog | 82.88 | 5684 |
| SD Analog | 85.41 | 5686 |
| HD HDMI | 83.08 | 5684 |
| SD HDMI | 82.45 | 5622 |
| HD Dual | 79.83 | 5670 |
| SD Dual | 83.26 | 5652 |

RAMs 17/32 (+4 layout ROM, +1 field RAM), ~74% LC. The first v0.3.0 build
passed HD HDMI at only 75.8: TS_PMOVE decided moves straight off the map
EBR's output. The tile flags (and the monster walkability, new state
TS_MWALK) are now latched one state earlier, worth ~7 MHz.

v0.2.0 (2026-10-10), router1, all six configs pass on **seed 1**:

| Config | Fmax (MHz) | LC |
|---|---|---|
| HD Analog | 83.69 | 5006 |
| SD Analog | 89.61 | 4975 |
| HD HDMI | 86.79 | 5031 |
| SD HDMI | 87.99 | 5009 |
| HD Dual | 87.83 | 4984 |
| SD Dual | 84.68 | 5033 |

RAMs 12/32, ~65% LC.
v0.1.0 (2026-07-03, router2): HD 78.5–81.8, SD 82–85, ~4800 LC.
