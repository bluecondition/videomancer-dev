-- Wizard Invaders
--
-- Videomancer-themed Space Invaders clone using the wizard_invaders_sprites
-- package (WIZARD player, CRT_TV alien with 2-frame walk cycle, VIDEOMANCER
-- bunker, WAND_SHOT bullet).
--
-- Gameplay
--   * 4 rows x 8 cols of 64x64 CRT-TV aliens packed cell-to-sprite
--   * 4 destructible bunkers (128x112 sprite, 8x7 grid of 16x16 chunks)
--     sit just 10 px above the wizard so they really get in the way
--   * Wizard ship (128x128) along the bottom, slid by Fader 12; mirrors
--     to face the direction of travel
--   * Up to 3 wand-shots active at once — fires on each Switch 7 edge
--     into the first free slot (drops if all 3 are in flight)
--   * Aliens drop up to 3 bombs at once on a per-frame cooldown; bombs
--     erode bunker chunks the same way the wand-shot does and end the
--     game on a wizard hit
--   * Switch 11 force-resets the wave
--
-- Designed for a 960x540 effective playfield (HD with hd_clock_divisor = 2
-- or 27 MHz SD).  Auto-measures incoming timing; ship_y / bunker_y scale
-- with v_active so layout is sensible at other resolutions.
--
-- Render pipeline (per pixel):
--   1 clk : video_timing_generator
--   1 clk : pixel_counter
--   1 clk : S1 — register signed coords
--   1 clk : S2 — per-region deltas (live positions read directly: game
--                state changes only during vblank)
--   1 clk : S3 — in-bbox flags + alien_idx + ROM addresses
--   1 clk : S4 — synchronous sprite ROM reads + state lookups
--   1 clk : S5 — bit selects + per-region pixel combine
--   1 clk : S6 — color mux
--   2 clk : IO alignment (wet pixel straight to output; the wet/dry
--           interpolators were removed — mix was hardcoded fully wet)
--   total : 8 render + 2 IO = 10 clocks
--
-- Physics FSM (8 phases, vsync-triggered):
--   Phase 0 : reset on edge; spawn first-free wand-shot on Fire-pending;
--             tick step counter + animation; tick bomb cooldown
--   Phase 1 : per-bullet advance; per-bomb advance; bomb spawn into first
--             free slot at LFSR-picked column; formation x predict
--   Phase 2 : formation x commit (+ drop on bounce); register bullet vs
--             formation deltas; register bomb vs ship deltas
--   Phase 3 : per-bullet alien hit precompute — register in_bbox + idx
--   Phase 4 : per-bullet alien hit commit — clear alien, deactivate bullet
--   Phase 5 : per-projectile bunker hit precompute — find which bunker
--             (if any) the projectile is in and register hit_id + chunk_idx
--   Phase 6 : per-projectile bunker chunk-alive lookup + aliens-clear
--             respawn
--   Phase 7 : per-projectile bunker commit + per-bomb ship hit check

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.wizard_invaders_sprites.all;

architecture invaders of program_top is

    -- ========================================================================
    -- Constants
    -- ========================================================================
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_MAX_VAL    : unsigned(9 downto 0) := to_unsigned(1023, 10);
    constant C_DELAY      : integer := 8;

    -- Alien formation: 3 rows × 6 cols of 64×64 CRT_TV sprites drawn at
    -- 2× nearest-neighbour scale = 128×128 on screen.  Cells are a
    -- power-of-two 128 × 128 so the row/col a pixel falls in is a plain
    -- bit-slice (no comparator cascade).  The sprite fills the cell, so
    -- aliens sit edge-to-edge — the CRT_TV art's own transparent margin
    -- keeps a small visual gap.
    --
    -- The full 768 × 384 grid is taller than the gap above the bunker,
    -- so the formation starts with its top row(s) off-screen above y=0
    -- and marches into view as it drops after each bounce — the classic
    -- Space Invaders entry pattern.
    constant C_COLS         : integer := 6;
    constant C_ROWS         : integer := 3;
    -- Cells are a power of two (128) so the column/row a pixel falls in is a
    -- plain bit-slice of form_dx/dy — no 12-bit comparator cascade or
    -- priority encoder.  The 128-wide sprite fills the cell (no padding), so
    -- aliens sit edge-to-edge; the CRT_TV art's own transparent margin keeps
    -- a small visual gap.
    constant C_CELL_W       : integer := 128;
    constant C_CELL_H       : integer := 128;
    constant C_FORM_W       : integer := C_COLS * C_CELL_W;             -- 768
    constant C_FORM_H       : integer := C_ROWS * C_CELL_H;             -- 384
    constant C_FORM_X_INIT  : integer := 96;                  -- (960-768)/2
    -- Formation starts with its top row off-screen and marches in as it
    -- drops after each bounce — the classic entry pattern.  p_layout
    -- recomputes the live init from the measured resolution.
    constant C_FORM_Y_INIT  : integer := -128;
    constant C_SPRITE_SRC   : integer := 64;
    constant C_SPRITE_W     : integer := 128;
    constant C_SPRITE_H     : integer := 128;
    constant C_SPRITE_OFF   : integer := 0;   -- sprite fills the 128 cell
    -- alien_idx = {row(1..0), col(2..0)} with col stride 8.  Slots 6, 7,
    -- 14, 15, 22..31 unused, but the 5-bit packing is cheaper than
    -- multiplying by 6.
    constant C_ALIEN_IDX_W  : integer := 5;
    constant C_ALIEN_SLOTS  : integer := 2 ** C_ALIEN_IDX_W;  -- 32 (18 used)

    -- Player ship + projectiles
    constant C_SHIP_W       : integer := 128;
    constant C_SHIP_H       : integer := 128;
    constant C_BULLET_W     : integer := 8;
    constant C_BULLET_H     : integer := 16;
    constant C_BOMB_W       : integer := 6;
    constant C_BOMB_H       : integer := 14;
    -- Up to 5 wand-shots in flight (the resource pass freed the LCs); bombs
    -- kept at 3.
    constant C_BULLETS      : integer := 5;
    constant C_BOMBS        : integer := 3;
    -- Wand offset inside the wizard sprite (upper-right of the 128x128
    -- bitmap; mirrored to upper-left when wizard faces left)
    constant C_WAND_OFFSET_X : integer := 88;
    constant C_WAND_OFFSET_Y : integer := 32;

    -- Bunkers.  Chunks are 32 × 32 (16 per bunker, was 32) — even coarser
    -- destruction to free more LCs for the dynamic-init logic.
    constant C_BUNKER_W       : integer := 128;
    constant C_BUNKER_H       : integer := 112;
    constant C_BUNKER_COUNT   : integer := 4;
    constant C_CHUNK_W        : integer := 32;
    constant C_CHUNK_H        : integer := 32;
    constant C_CHUNK_COLS     : integer := C_BUNKER_W / C_CHUNK_W;   -- 4
    constant C_CHUNK_ROWS_PWR : integer := 4;
    constant C_CHUNKS_PER_BNK : integer := C_CHUNK_COLS * C_CHUNK_ROWS_PWR;
                                                -- 16

    constant C_FORM_STEP_X  : integer := 6;     -- faster side-to-side march
    constant C_BORDER       : integer := 8;
    -- Vertical gap (px) between bunker bottom and wizard top.  Bumped from
    -- 10 → 26 (= 10 + one chunk-row) so the bunkers sit one chunk higher.
    constant C_BUNKER_GAP   : integer := 26;
    constant C_BOMB_SPEED   : integer := 2;

    -- ------------------------------------------------------------------
    -- Rainbow ghost trail: instead of dots, leave faded full-sprite
    -- afterimages of the wizard behind it as it slides left/right.  A new
    -- ghost is recorded on a vsync only once the ship has moved at least
    -- C_TRAIL_GAP px since the last one, so a stationary ship leaves no
    -- pile-up.  Each ghost cycles through the 7-colour ROYGBIV phosphor
    -- palette, so the afterimages fan out as a rainbow.  Because the wizard
    -- only moves horizontally, every ghost shares the live wizard's ROM row
    -- (same Y band) — only the per-ghost column index + colour differ.
    constant C_TRAIL_LEN   : integer := 3;  -- number of afterimages held
    -- Spacing ≥ the 128-wide sprite so the ghosts never overlap each other.
    -- That guarantees at most one ghost covers any pixel, so the render needs
    -- a SINGLE wizard-row mux (the winning ghost is picked in S3) no matter
    -- how many ghosts there are — the trail reads as distinct rainbow echoes.
    constant C_TRAIL_GAP   : integer := 144;
    -- Dim luma for the ghost silhouette so it reads as a faded afterimage
    -- rather than a second solid wizard.
    constant C_GHOST_LUMA  : integer := 470;

    -- ROYGBIV phosphor chroma palette, 10-bit U/V in the Videomancer
    -- U/V-swapped convention (hardware-confirmed; see Phosphor program).
    -- Looked up once per ghost at drop time (off the hot path) so the render
    -- path carries plain chroma.  Index 7 duplicates red as an out-of-range
    -- guard for the 3-bit colour counter.
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_TRAIL_PAL_U : t_pal := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10),
        to_unsigned(136, 10), to_unsigned(440, 10), to_unsigned(608, 10),
        to_unsigned(756, 10), to_unsigned(960, 10));
    constant C_TRAIL_PAL_V : t_pal := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10),
        to_unsigned(216, 10), to_unsigned(960, 10), to_unsigned(696, 10),
        to_unsigned(852, 10), to_unsigned(360, 10));

    -- Alien ROM = CRT_TV_F0 followed by CRT_TV_F1 → 128 entries × 64 bits
    -- (frame select is the high bit of the 7-bit address).
    type t_alien_rom is array (0 to 127) of std_logic_vector(63 downto 0);
    function build_alien_rom return t_alien_rom is
        variable v : t_alien_rom;
    begin
        for i in 0 to 63 loop
            v(i)      := CRT_TV_F0(i);
            v(i + 64) := CRT_TV_F1(i);
        end loop;
        return v;
    end function;
    constant C_ALIEN_ROM : t_alien_rom := build_alien_rom;

    -- Explosion ROM = EXPLOSION_F0..F3 → 192 entries × 48 bits.  Address
    -- = {frame(1..0), row(5..0)} (8 bits, only 192/256 used).
    constant C_EXPL_W : integer := 48;
    constant C_EXPL_H : integer := 48;
    constant C_EXPL_ROM_DEPTH : integer := 256;
    type t_expl_rom is array (0 to C_EXPL_ROM_DEPTH - 1)
                        of std_logic_vector(47 downto 0);
    function build_expl_rom return t_expl_rom is
        variable v : t_expl_rom := (others => (others => '0'));
    begin
        for i in 0 to 47 loop
            v(i)        := EXPLOSION_F0(i);
            v(i + 64)   := EXPLOSION_F1(i);
            v(i + 128)  := EXPLOSION_F2(i);
            v(i + 192)  := EXPLOSION_F3(i);
        end loop;
        return v;
    end function;
    constant C_EXPL_ROM : t_expl_rom := build_expl_rom;
    -- 6 vsync ticks per animation frame ≈ 100 ms × 4 frames = 400 ms burst
    constant C_EXPL_FRAME_TIME : integer := 6;

    -- Alien-step-period LUT indexed by top 4 bits of the Alien-Spd pot.
    -- Pot = 0 → 12 frames (the previous default), pot = 1023 → 1 frame
    -- per step (fastest).  Saturates at the bottom so pot values above
    -- the 12th step still hold at 1.
    type t_period_lut is array (0 to 15) of unsigned(5 downto 0);
    constant C_PERIOD_LUT : t_period_lut := (
        to_unsigned(12, 6), to_unsigned(11, 6),
        to_unsigned(10, 6), to_unsigned( 9, 6),
        to_unsigned( 8, 6), to_unsigned( 7, 6),
        to_unsigned( 6, 6), to_unsigned( 5, 6),
        to_unsigned( 4, 6), to_unsigned( 3, 6),
        to_unsigned( 2, 6), to_unsigned( 1, 6),
        to_unsigned( 1, 6), to_unsigned( 1, 6),
        to_unsigned( 1, 6), to_unsigned( 1, 6)
    );

    -- ========================================================================
    -- Parameters
    -- ========================================================================
    signal s_alien_spd_pot  : unsigned(9 downto 0);
    signal s_bullet_spd_pot : unsigned(9 downto 0);
    signal s_bright_pot     : unsigned(9 downto 0);
    signal s_bomb_spd_pot   : unsigned(9 downto 0);
    signal s_hue_pot        : unsigned(9 downto 0);
    signal s_bomb_speed     : unsigned(3 downto 0) := to_unsigned(3, 4);
    signal s_sw_fire        : std_logic;
    signal s_sw_bg_black    : std_logic;
    signal s_sw_color       : std_logic;
    signal s_sw_pause       : std_logic;
    signal s_sw_reset       : std_logic;
    signal s_ship_pos_pot   : unsigned(9 downto 0);

    signal s_sw_fire_prev  : std_logic := '0';
    signal s_sw_reset_prev : std_logic := '0';
    signal s_fire_pending  : std_logic := '0';
    signal s_reset_pending : std_logic := '0';
    -- Round-robin wand-shot slot — every Switch-7 edge spawns at the
    -- next slot in rotation (overwriting if all 5 are in flight), so
    -- every toggle produces a visible shot.
    signal s_fire_idx      : unsigned(2 downto 0) := (others => '0');

    signal s_step_period   : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal s_bullet_speed  : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_bomb_cd_reset : unsigned(7 downto 0) := to_unsigned(60, 8);

    -- ========================================================================
    -- Timing + auto-measured resolution
    -- ========================================================================
    signal s_timing        : t_video_timing_port;
    signal s_h_count       : unsigned(11 downto 0);
    signal s_v_count       : unsigned(11 downto 0);

    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_h_active        : integer range 0 to 4095 := 960;
    signal s_v_active        : integer range 0 to 4095 := 540;

    signal s_ship_y        : signed(11 downto 0) := to_signed(412, 12);
    signal s_bunker_y      : signed(11 downto 0) := to_signed(290, 12);
    -- Dynamic formation init.  At v_active = 540 the formation can only
    -- fit with its top row off-screen (3 rows × 144 = 432 > 274 px above
    -- bunker), so we slot the bottom row just above the bunker.  At
    -- v_active ≥ 720 (HD HDMI / 1080p) all 3 rows fit, so we drop the
    -- whole formation to a small top margin.
    signal s_form_x_init   : signed(11 downto 0) := to_signed(48, 12);
    signal s_form_y_init   : signed(11 downto 0) := to_signed(-160, 12);
    -- One-shot boot init.  s_boot_count counts up to 9 and saturates;
    -- phase 0 of the FSM triggers a single init reset on the vsync where
    -- the counter holds value 8 — short enough that the wizard can
    -- shoot immediately after power-on, but long enough for v_active to
    -- settle and bomb_cooldown to be loaded.
    signal s_boot_count    : unsigned(3 downto 0) := (others => '0');
    type t_bunker_x_arr is array (0 to C_BUNKER_COUNT - 1) of signed(11 downto 0);
    signal s_bunker_x      : t_bunker_x_arr := (others => (others => '0'));

    signal s_ship_x_max    : unsigned(11 downto 0)
                              := to_unsigned(960 - C_SHIP_W, 12);
    signal s_form_x_max    : signed(11 downto 0)
                              := to_signed(960 - C_FORM_W - C_BORDER, 12);
    constant C_FORM_X_MIN  : signed(11 downto 0) := to_signed(C_BORDER, 12);

    -- ========================================================================
    -- Game state
    -- ========================================================================
    signal s_aliens        : std_logic_vector(0 to C_ALIEN_SLOTS - 1)
                              := (others => '1');
    type t_bunker_bits is array (0 to C_BUNKER_COUNT - 1)
                          of std_logic_vector(0 to C_CHUNKS_PER_BNK - 1);
    signal s_bunkers       : t_bunker_bits := (others => (others => '1'));

    signal s_form_x        : signed(11 downto 0) := to_signed(C_FORM_X_INIT, 12);
    signal s_form_y        : signed(11 downto 0) := to_signed(C_FORM_Y_INIT, 12);
    signal s_form_dir      : std_logic := '1';
    signal s_step_counter  : unsigned(5 downto 0) := (others => '0');
    signal s_alien_frame   : std_logic := '0';

    -- Ship X + facing tracker
    signal s_ship_mul      : unsigned(20 downto 0) := (others => '0');
    signal s_ship_x        : signed(11 downto 0) := to_signed(416, 12);
    signal s_ship_x_prev   : signed(11 downto 0) := to_signed(416, 12);
    -- '0' = sprite as authored (faces left visually based on the bitmap);
    -- '1' = mirrored (faces right).  Set by p_facing so the wizard ends
    -- up facing the direction of travel.
    signal s_wizard_facing : std_logic := '0';

    -- Per-bullet state
    type t_pos_arr_b is array (0 to C_BULLETS - 1) of signed(11 downto 0);
    signal s_bullet_x      : t_pos_arr_b := (others => (others => '0'));
    signal s_bullet_y      : t_pos_arr_b := (others => (others => '0'));
    signal s_bullet_active : std_logic_vector(0 to C_BULLETS - 1)
                              := (others => '0');
    signal s_bullet_at_top : std_logic_vector(0 to C_BULLETS - 1)
                              := (others => '0');

    -- Per-bomb state
    type t_pos_arr_m is array (0 to C_BOMBS - 1) of signed(11 downto 0);
    signal s_bomb_x        : t_pos_arr_m := (others => (others => '0'));
    signal s_bomb_y        : t_pos_arr_m := (others => (others => '0'));
    signal s_bomb_active   : std_logic_vector(0 to C_BOMBS - 1)
                              := (others => '0');
    signal s_bomb_at_bot   : std_logic_vector(0 to C_BOMBS - 1)
                              := (others => '0');
    -- Initialise high so no bombs drop during the 8-vsync boot window
    -- (before s_bomb_cd_reset can be loaded by the reset path).
    signal s_bomb_cooldown : unsigned(7 downto 0) := to_unsigned(180, 8);

    -- 8-bit LFSR
    signal s_lfsr          : std_logic_vector(7 downto 0) := "10110011";

    -- Serial alien-hit scan: check one bullet per vsync (round-robin).
    -- Bullets at ≥6 px/frame sit in a 144-tall cell for ~24 frames so a
    -- 5-frame check rotation still catches every hit reliably.
    signal s_bullet_scan   : unsigned(2 downto 0) := (others => '0');

    -- Kill counter for the "fewer aliens → faster march" speed boost.
    -- Increments on each alien kill, resets in any reset path.  Max 18
    -- (= 3×6 grid; 5-bit wide fits 0..31).
    signal s_dead_count    : unsigned(4 downto 0) := (others => '0');

    signal s_form_at_ship  : std_logic := '0';
    signal s_aliens_clear  : std_logic := '0';
    signal s_game_over     : std_logic := '0';
    -- Pre-registered reset trigger — collapses (reset_pending OR
    -- game_over OR form_at_ship OR boot_count=8) into a single
    -- registered bit so phase 0's reset mux selector doesn't fan out
    -- across the chip and stretch the critical path.  Costs 1 cycle of
    -- latency on reset detection (imperceptible).
    signal s_do_reset      : std_logic := '0';

    -- Explosion (single slot — spawned in phase 4 when a wand-shot kills
    -- an alien, animates F0→F3 then disappears).
    signal s_expl_active   : std_logic := '0';
    signal s_expl_x        : signed(11 downto 0) := (others => '0');
    signal s_expl_y        : signed(11 downto 0) := (others => '0');
    signal s_expl_frame    : unsigned(1 downto 0) := (others => '0');
    signal s_expl_timer    : unsigned(2 downto 0) := (others => '0');

    -- Rainbow ghost ring buffer.  s_trail_x holds each afterimage's sprite
    -- top-left X (ghosts start off-screen so none show before the first
    -- move).  Each ghost carries its already-resolved chroma (palette looked
    -- up at drop time) and the wizard facing it had when dropped.
    type t_trail_x_arr is array (0 to C_TRAIL_LEN - 1) of signed(11 downto 0);
    type t_trail_uv_arr is array (0 to C_TRAIL_LEN - 1) of unsigned(9 downto 0);
    signal s_trail_x       : t_trail_x_arr := (others => to_signed(-256, 12));
    signal s_trail_u       : t_trail_uv_arr := (others => C_CHROMA_MID);
    signal s_trail_v       : t_trail_uv_arr := (others => C_CHROMA_MID);
    signal s_trail_face    : std_logic_vector(0 to C_TRAIL_LEN - 1)
                              := (others => '0');
    signal s_trail_valid   : std_logic_vector(0 to C_TRAIL_LEN - 1)
                              := (others => '0');
    signal s_trail_head    : unsigned(2 downto 0) := (others => '0');
    signal s_trail_colctr  : unsigned(2 downto 0) := (others => '0');
    -- Movement tracker, pipelined so the once-per-frame distance test never
    -- forms a long chained carry path: s_trail_anchor = ship X at the last
    -- drop, s_trail_diff = ship X − anchor (registered), s_trail_moved =
    -- |diff| ≥ gap (registered, via two parallel compares — no abs chain).
    signal s_trail_anchor  : signed(11 downto 0) := (others => '0');
    signal s_trail_diff    : signed(11 downto 0) := (others => '0');
    signal s_trail_moved   : std_logic := '0';

    -- ========================================================================
    -- Physics FSM intermediates
    -- ========================================================================
    signal s_ph_phase      : unsigned(2 downto 0) := (others => '0');
    signal s_ph_new_fx     : signed(11 downto 0) := (others => '0');

    -- Bullet-vs-formation hit detect for the SCANNED bullet only (serial,
    -- one bullet per vsync).  With power-of-two cells the hit cell is just a
    -- bit-slice: s_ph_bf_col = scan_dx[9:7], s_ph_bf_row = scan_dy[8:7],
    -- gated by the in-grid range flags.  Single signals (not per-bullet
    -- arrays) keep the LC budget sane.
    signal s_ph_bf_col     : unsigned(2 downto 0) := (others => '0');
    signal s_ph_bf_row     : unsigned(1 downto 0) := (others => '0');
    signal s_ph_bf_dy_in    : std_logic := '0';
    signal s_ph_bf_dx_in    : std_logic := '0';
    -- Bomb vs ship: serialised via the same s_bullet_scan counter so we
    -- check one bomb's ship hit per vsync.  At ~3 px/frame in a 128-tall
    -- wizard, each bomb sits there ~40 frames so the 5-frame rotation
    -- catches every collision easily.
    signal s_ph_bs_dx      : signed(11 downto 0) := (others => '0');
    signal s_ph_bs_dy      : signed(11 downto 0) := (others => '0');
    signal s_ph_bs_act     : std_logic := '0';

    -- Per-bullet alien hit precompute
    -- Scanned-bullet alien hit result (single signal).
    signal s_ph_alien_in   : std_logic := '0';
    signal s_ph_alien_idx  : unsigned(C_ALIEN_IDX_W - 1 downto 0)
                              := (others => '0');
    -- Pre-registered "alien at the hit cell is alive" — read in phase 3 so
    -- phase 4's commit isn't a 32:1 s_aliens read AND'd with the write-enable
    -- to s_bullet_active on one routing-heavy path.
    signal s_ph_alien_alive_pre : std_logic := '0';
    signal s_ph_alien_sel  : unsigned(2 downto 0) := (others => '0');
    -- Pre-computed explosion spawn position (set in phase 3, consumed
    -- in phase 4) so the alien_idx → spawn-coordinate case statement
    -- isn't chained with the alien_alive lookup on the critical path.
    signal s_ph_expl_x_cand : signed(11 downto 0) := (others => '0');
    signal s_ph_expl_y_cand : signed(11 downto 0) := (others => '0');

    -- Per-projectile bunker hit precompute (bullets + bombs treated uniformly)
    -- Index 0..C_BULLETS-1 = bullets; C_BULLETS..C_BULLETS+C_BOMBS-1 = bombs.
    -- Scanned one projectile per vsync to keep LC cost in budget — each
    -- projectile typically spends 30+ frames inside a bunker so the
    -- 1/C_PROJ_COUNT-rate check still catches almost every chunk.
    constant C_PROJ_COUNT  : integer := C_BULLETS + C_BOMBS;
    signal s_proj_scan     : unsigned(3 downto 0) := (others => '0');
    -- Pre-registered position of the scanned projectile (latched in phase 4
    -- so phase 5's subtractors see registered inputs instead of a long
    -- 6:1 mux + 12-bit subtract combinational chain).
    signal s_ph_proj_x     : signed(11 downto 0) := (others => '0');
    signal s_ph_proj_y     : signed(11 downto 0) := (others => '0');
    signal s_ph_proj_a     : std_logic := '0';
    signal s_ph_proj_hit   : std_logic := '0';
    signal s_ph_proj_bid   : unsigned(1 downto 0) := (others => '0');
    signal s_ph_proj_cidx  : unsigned(3 downto 0) := (others => '0');
    signal s_ph_proj_alive : std_logic := '0';
    signal s_ph_proj_sel   : unsigned(3 downto 0) := (others => '0');

    -- ========================================================================
    -- Render pipeline registers
    -- ========================================================================
    -- S1: coords
    signal s_stg1_hx       : signed(11 downto 0) := (others => '0');
    signal s_stg1_vy       : signed(11 downto 0) := (others => '0');

    -- S2: deltas (live game state)
    signal s_stg2_form_dx  : signed(11 downto 0) := (others => '0');
    signal s_stg2_form_dy  : signed(11 downto 0) := (others => '0');
    signal s_stg2_ship_dx  : signed(11 downto 0) := (others => '0');
    signal s_stg2_ship_dy  : signed(11 downto 0) := (others => '0');
    type t_d_arr_b is array (0 to C_BULLETS - 1) of signed(11 downto 0);
    type t_d_arr_m is array (0 to C_BOMBS - 1) of signed(11 downto 0);
    signal s_stg2_bull_dx  : t_d_arr_b := (others => (others => '0'));
    signal s_stg2_bull_dy  : t_d_arr_b := (others => (others => '0'));
    signal s_stg2_bomb_dx  : t_d_arr_m := (others => (others => '0'));
    signal s_stg2_bomb_dy  : t_d_arr_m := (others => (others => '0'));
    type t_bunker_d_arr is array (0 to C_BUNKER_COUNT - 1) of signed(11 downto 0);
    signal s_stg2_bnk_dx   : t_bunker_d_arr := (others => (others => '0'));
    signal s_stg2_bnk_dy   : signed(11 downto 0) := (others => '0');
    signal s_stg2_expl_dx  : signed(11 downto 0) := (others => '0');
    signal s_stg2_expl_dy  : signed(11 downto 0) := (others => '0');
    signal s_stg2_expl_a   : std_logic := '0';
    signal s_stg2_expl_fr  : unsigned(1 downto 0) := (others => '0');
    -- Formation in-grid flags.  Cells are a power of two, so S3 derives
    -- row/col by bit-slicing the registered form deltas — no cascade.
    signal s_stg2_fdy_in    : std_logic := '0';
    signal s_stg2_fdx_in    : std_logic := '0';
    signal s_stg2_frame    : std_logic := '0';
    signal s_stg2_bull_a   : std_logic_vector(0 to C_BULLETS - 1)
                              := (others => '0');
    signal s_stg2_bomb_a   : std_logic_vector(0 to C_BOMBS - 1)
                              := (others => '0');
    -- Ghost trail: per-ghost X delta + validity + facing + resolved chroma
    -- (all registered here from live ring state so S3 reads only S2 flops).
    -- The Y band and ROM row are shared with the live wizard via ship_dy.
    type t_d_arr_t is array (0 to C_TRAIL_LEN - 1) of signed(11 downto 0);
    signal s_stg2_trail_dx : t_d_arr_t := (others => (others => '0'));
    signal s_stg2_trail_v  : std_logic_vector(0 to C_TRAIL_LEN - 1)
                              := (others => '0');
    signal s_stg2_trail_f  : std_logic_vector(0 to C_TRAIL_LEN - 1)
                              := (others => '0');

    -- S3: in-bbox flags + indices + ROM addresses
    signal s_stg3_in_form    : std_logic := '0';
    signal s_stg3_alien_idx  : unsigned(C_ALIEN_IDX_W - 1 downto 0)
                                 := (others => '0');
    signal s_stg3_alien_addr : unsigned(6 downto 0) := (others => '0');
    signal s_stg3_alien_col  : unsigned(5 downto 0) := (others => '0');
    signal s_stg3_in_ship    : std_logic := '0';
    signal s_stg3_ship_row   : unsigned(6 downto 0) := (others => '0');
    signal s_stg3_ship_col   : unsigned(6 downto 0) := (others => '0');
    signal s_stg3_in_bullet  : std_logic_vector(0 to C_BULLETS - 1)
                                := (others => '0');
    signal s_stg3_in_bomb    : std_logic_vector(0 to C_BOMBS - 1)
                                := (others => '0');
    signal s_stg3_in_bunker  : std_logic_vector(0 to C_BUNKER_COUNT - 1)
                                := (others => '0');
    signal s_stg3_bnk_row    : unsigned(6 downto 0) := (others => '0');
    signal s_stg3_bnk_col    : unsigned(6 downto 0) := (others => '0');
    signal s_stg3_chunk_idx  : unsigned(3 downto 0) := (others => '0');
    signal s_stg3_in_expl    : std_logic := '0';
    signal s_stg3_expl_addr  : unsigned(7 downto 0) := (others => '0');
    signal s_stg3_expl_col   : unsigned(5 downto 0) := (others => '0');
    -- Winning ghost (non-overlap → at most one covers a pixel): in-bbox
    -- flag, column, facing, and ring index for the chroma read.  Carried to
    -- S5 where the shared wizard ROM row becomes the silhouette bit.
    signal s_stg3_gh_in      : std_logic := '0';
    signal s_stg3_gh_col     : unsigned(6 downto 0) := (others => '0');
    signal s_stg3_gh_f       : std_logic := '0';
    signal s_stg3_gh_sel     : integer range 0 to C_TRAIL_LEN - 1 := 0;

    -- S4: sync ROM reads + state lookups
    signal s_stg4_alien_row  : std_logic_vector(63 downto 0) := (others => '0');
    signal s_stg4_wizard_row : std_logic_vector(127 downto 0) := (others => '0');
    signal s_stg4_bunker_row : std_logic_vector(127 downto 0) := (others => '0');
    signal s_stg4_in_form    : std_logic := '0';
    signal s_stg4_alien_alive : std_logic := '0';
    signal s_stg4_alien_col  : unsigned(5 downto 0) := (others => '0');
    signal s_stg4_in_ship    : std_logic := '0';
    signal s_stg4_ship_col   : unsigned(6 downto 0) := (others => '0');
    signal s_stg4_in_bullet  : std_logic_vector(0 to C_BULLETS - 1)
                                := (others => '0');
    signal s_stg4_in_bomb    : std_logic_vector(0 to C_BOMBS - 1)
                                := (others => '0');
    signal s_stg4_in_bunker_any : std_logic := '0';
    signal s_stg4_bnk_col    : unsigned(6 downto 0) := (others => '0');
    signal s_stg4_chunk_alive : std_logic := '0';
    signal s_stg4_expl_row   : std_logic_vector(47 downto 0) := (others => '0');
    signal s_stg4_in_expl    : std_logic := '0';
    signal s_stg4_expl_col   : unsigned(5 downto 0) := (others => '0');
    signal s_stg4_gh_in      : std_logic := '0';
    signal s_stg4_gh_col     : unsigned(6 downto 0) := (others => '0');
    signal s_stg4_gh_f       : std_logic := '0';
    signal s_stg4_gh_sel     : integer range 0 to C_TRAIL_LEN - 1 := 0;

    -- S5: combined per-region pixel flags
    signal s_stg5_on_alien   : std_logic := '0';
    signal s_stg5_on_ship    : std_logic := '0';
    signal s_stg5_on_bullet  : std_logic := '0';
    signal s_stg5_on_bomb    : std_logic := '0';
    signal s_stg5_on_bunker  : std_logic := '0';
    signal s_stg5_on_expl    : std_logic := '0';
    signal s_stg5_on_floor   : std_logic := '0';
    signal s_stg5_on_trail   : std_logic := '0';
    signal s_stg5_trail_u    : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_stg5_trail_v    : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Floor line geometry — 4-px stripe at the wizard's feet, full
    -- screen width.  Computed in p_layout based on ship_y.
    constant C_FLOOR_H     : integer := 4;
    signal s_floor_y       : signed(11 downto 0) := to_signed(536, 12);

    -- S6: color mux outputs
    signal s_out_y         : unsigned(9 downto 0) := (others => '0');
    signal s_out_u         : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_out_v         : unsigned(9 downto 0) := C_CHROMA_MID;
    -- Pre-registered foreground chroma (constant per frame).  Keeping the
    -- sw_color choice out of S6 leaves the output mux a single 2:1 select.
    signal s_fg_u          : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_fg_v          : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_out_show      : std_logic := '0';

    -- ========================================================================
    -- Sync delay: 8 render stages, then straight to the 2 IO stages (the
    -- wet/dry interpolators were removed — the mix was hardcoded fully wet,
    -- so they were a no-op; sync now taps at the 8-clock render depth).
    -- ========================================================================
    type t_sync_pipe is array (0 to C_DELAY - 1) of std_logic_vector(3 downto 0);
    signal s_sync_pipe     : t_sync_pipe := (others => (others => '0'));

    type t_data_delay is array (0 to C_DELAY - 1) of std_logic_vector(9 downto 0);
    signal s_y_delay : t_data_delay := (others => (others => '0'));
    signal s_u_delay : t_data_delay := (others => (others => '0'));
    signal s_v_delay : t_data_delay := (others => (others => '0'));

    signal s_wet_y : std_logic_vector(9 downto 0);
    signal s_wet_u : std_logic_vector(9 downto 0);
    signal s_wet_v : std_logic_vector(9 downto 0);

    signal s_io_0 : t_video_stream_yuv444_30b;
    signal s_io_1 : t_video_stream_yuv444_30b;

begin

    -- ========================================================================
    -- Auto-Measure Active Resolution
    -- ========================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then
                    s_measured_h <= s_h_pixel_counter;
                end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then
                    s_measured_v <= s_v_line_counter;
                end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    p_layout : process(clk)
        variable v_h_div_8 : integer range 0 to 511;
        variable v_v       : integer range 0 to 4095;
    begin
        if rising_edge(clk) then
            s_h_active  <= to_integer(s_measured_h);
            s_v_active  <= to_integer(s_measured_v);

            v_v := to_integer(s_measured_v);
            -- Wizard hugs the bottom
            if v_v > C_SHIP_H then
                s_ship_y <= to_signed(v_v - C_SHIP_H, 12);
            else
                s_ship_y <= to_signed(0, 12);
            end if;
            -- Bunker sits C_BUNKER_GAP px above the wizard top (closer than
            -- before so it really crowds the player).
            if v_v > C_SHIP_H + C_BUNKER_H + C_BUNKER_GAP then
                s_bunker_y <= to_signed(v_v - C_SHIP_H - C_BUNKER_GAP
                                        - C_BUNKER_H, 12);
            else
                s_bunker_y <= to_signed(0, 12);
            end if;

            -- Bunker X at fractions 1/8, 3/8, 5/8, 7/8 of h_active
            v_h_div_8 := to_integer(s_measured_h) / 8;
            s_bunker_x(0) <= to_signed(    v_h_div_8 - C_BUNKER_W / 2, 12);
            s_bunker_x(1) <= to_signed(3 * v_h_div_8 - C_BUNKER_W / 2, 12);
            s_bunker_x(2) <= to_signed(5 * v_h_div_8 - C_BUNKER_W / 2, 12);
            s_bunker_x(3) <= to_signed(7 * v_h_div_8 - C_BUNKER_W / 2, 12);

            if to_integer(s_measured_h) > C_SHIP_W then
                s_ship_x_max <= s_measured_h - to_unsigned(C_SHIP_W, 12);
            else
                s_ship_x_max <= (others => '0');
            end if;
            if to_integer(s_measured_h) > C_FORM_W + C_BORDER then
                s_form_x_max <= to_signed(
                    to_integer(s_measured_h) - C_FORM_W - C_BORDER, 12);
            else
                s_form_x_max <= to_signed(C_BORDER, 12);
            end if;

            -- Dynamic formation init derived from the auto-measured
            -- layout.  If C_FORM_H fits comfortably above the bunker,
            -- start the formation with a 16-px top margin (all 3 rows
            -- visible).  Otherwise (cramped 540-line modes) park the
            -- bottom row 2 px above the bunker so it descends from the
            -- top.
            if s_bunker_y >= to_signed(C_FORM_H + 16, 12) then
                s_form_y_init <= to_signed(16, 12);
            else
                -- SIM-TEMP: was `s_bunker_y - to_signed(C_FORM_H + 2, 12)`
                -- — start aliens already on-screen so sim captures them.
                s_form_y_init <= to_signed(0, 12);
            end if;
            -- Centre formation horizontally
            if to_integer(s_measured_h) > C_FORM_W then
                s_form_x_init <= to_signed(
                    (to_integer(s_measured_h) - C_FORM_W) / 2, 12);
            else
                s_form_x_init <= to_signed(0, 12);
            end if;

            -- Floor line just under the wizard (at the very bottom of
            -- the sprite cell), full screen width.
            s_floor_y <= to_signed(v_v - C_FLOOR_H, 12);
        end if;
    end process;

    -- ========================================================================
    -- Register Mapping
    -- ========================================================================
    -- Pots 1..3 are unused (nothing meaningful to tune in this build);
    -- hardcode their effective values so registers_in(0..2) get optimised
    -- away by yosys.
    s_hue_pot        <= to_unsigned(512, 10);
    s_bright_pot     <= to_unsigned(940, 10);
    s_alien_spd_pot  <= unsigned(registers_in(3));   -- Pot 4: Alien Spd
    s_bomb_spd_pot   <= unsigned(registers_in(4));   -- Pot 5: Bomb Drop Spd
    s_bullet_spd_pot <= unsigned(registers_in(5));   -- Pot 6: Shot Spd
    s_sw_fire        <= registers_in(6)(0);
    s_sw_bg_black    <= registers_in(6)(1);
    s_sw_color       <= registers_in(6)(2);
    s_sw_pause       <= registers_in(6)(3);
    s_sw_reset       <= registers_in(6)(4);
    s_ship_pos_pot   <= unsigned(registers_in(7));

    p_speeds : process(clk)
        variable v_period_raw : unsigned(5 downto 0);
        variable v_boost      : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- Alien step period via LUT (pot=0 → 12 = previous default,
            -- pot≥0xC00 → 1 = fastest), with a per-kill boost so the
            -- march speeds up a little with EVERY alien destroyed.
            -- boost = dead_count (saturates the period to 1 once enough
            -- aliens are dead — gives a clear finale-pace feel).
            v_period_raw := C_PERIOD_LUT(
                                to_integer(s_alien_spd_pot(9 downto 6)));
            v_boost := resize(s_dead_count, 6);
            if v_period_raw <= v_boost + to_unsigned(1, 6) then
                s_step_period <= to_unsigned(1, 6);
            else
                s_step_period <= v_period_raw - v_boost;
            end if;
            -- Shot speed: 6 + pot/128 px/frame → 6..13 (6 = previous default)
            s_bullet_speed  <= resize(s_bullet_spd_pot(9 downto 7), 4) +
                               to_unsigned(6, 4);
            -- Bomb drop speed: 3 + pot/128 px/frame → 3..10 (3 = previous
            -- default constant C_BOMB_SPEED)
            s_bomb_speed    <= resize(s_bomb_spd_pot(9 downto 7), 4) +
                               to_unsigned(C_BOMB_SPEED, 4);
            -- Bomb cooldown (fixed, was Pot 4).  30 frames ≈ 0.5 s
            -- between spawn attempts → multiple bombs visible at once.
            s_bomb_cd_reset <= to_unsigned(30, 8);
        end if;
    end process;

    -- ========================================================================
    -- Video Timing Generator + Pixel Counter
    -- ========================================================================
    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    pixel_counter_inst : entity work.pixel_counter
        port map (
            clk     => clk,
            timing  => s_timing,
            h_count => s_h_count,
            v_count => s_v_count
        );

    -- ========================================================================
    -- Ship X (2-stage pipeline)
    -- ========================================================================
    p_ship_mul : process(clk)
    begin
        if rising_edge(clk) then
            s_ship_mul <= s_ship_pos_pot * resize(s_ship_x_max, 11);
        end if;
    end process;

    p_ship_x : process(clk)
    begin
        if rising_edge(clk) then
            s_ship_x <= signed(resize(shift_right(s_ship_mul, 10), 12));
        end if;
    end process;

    -- Wizard facing tracker: face the direction of travel.  With the
    -- authored sprite (un-mirrored) showing the wizard angled one way, we
    -- mirror when moving RIGHT and keep the default when moving LEFT (the
    -- opposite of the previous build, which the player found backwards).
    p_facing : process(clk)
    begin
        if rising_edge(clk) then
            s_ship_x_prev <= s_ship_x;
            if s_ship_x > s_ship_x_prev then
                s_wizard_facing <= '1';   -- moving right → mirrored
            elsif s_ship_x < s_ship_x_prev then
                s_wizard_facing <= '0';   -- moving left → default
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Rainbow ghost recorder
    -- ========================================================================
    -- The "has the wizard moved a ghost-gap since the last drop?" test runs
    -- continuously and pipelined (one 12-bit op per registered stage) so it
    -- never forms a long carry chain on the pixel clock.  At vsync, if the
    -- pre-computed s_trail_moved flag is set, drop a new ghost into the ring
    -- — recording the sprite's top-left X, its facing, and its rainbow
    -- chroma (palette resolved here, during vblank) — and advance the colour
    -- counter.  The ghost's Y is the wizard's live Y, so it shares the row.
    p_trail : process(clk)
    begin
        if rising_edge(clk) then
            -- Stage A: registered signed delta from the last-drop anchor.
            s_trail_diff <= s_ship_x - s_trail_anchor;
            -- Stage B: registered |delta| ≥ gap, via two parallel compares.
            if s_trail_diff >=  to_signed(C_TRAIL_GAP, 12) or
               s_trail_diff <= -to_signed(C_TRAIL_GAP, 12) then
                s_trail_moved <= '1';
            else
                s_trail_moved <= '0';
            end if;

            if s_timing.vsync_start = '1' and s_trail_moved = '1' then
                s_trail_x(to_integer(s_trail_head))     <= s_ship_x;
                s_trail_u(to_integer(s_trail_head))     <=
                    C_TRAIL_PAL_U(to_integer(s_trail_colctr));
                s_trail_v(to_integer(s_trail_head))     <=
                    C_TRAIL_PAL_V(to_integer(s_trail_colctr));
                s_trail_face(to_integer(s_trail_head))  <= s_wizard_facing;
                s_trail_valid(to_integer(s_trail_head)) <= '1';
                s_trail_anchor <= s_ship_x;
                if s_trail_head >= to_unsigned(C_TRAIL_LEN - 1, 3) then
                    s_trail_head <= (others => '0');
                else
                    s_trail_head <= s_trail_head + 1;
                end if;
                if s_trail_colctr >= to_unsigned(6, 3) then
                    s_trail_colctr <= (others => '0');
                else
                    s_trail_colctr <= s_trail_colctr + 1;
                end if;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Continuous predicates
    -- ========================================================================
    p_predicates : process(clk)
        variable v_or : std_logic;
    begin
        if rising_edge(clk) then
            s_sw_fire_prev  <= s_sw_fire;
            s_sw_reset_prev <= s_sw_reset;

            -- Boot-init counter: gives v_active 8 vsyncs (≈130 ms) to
            -- settle.  Saturates at 9.  Phase 0 of the FSM uses
            -- s_boot_count = 8 as a single-shot init trigger (rather
            -- than the previous "boot_done=0 → reset every vsync" path
            -- which locked the player out of firing during boot).
            if s_timing.vsync_start = '1' and s_boot_count < 9 then
                s_boot_count <= s_boot_count + 1;
            end if;

            if s_timing.vsync_start = '1' then
                s_fire_pending  <= '0';
                s_reset_pending <= '0';
            else
                if s_sw_fire /= s_sw_fire_prev then
                    s_fire_pending <= '1';
                end if;
                if s_sw_reset = '1' and s_sw_reset_prev = '0' then
                    s_reset_pending <= '1';
                end if;
            end if;

            for i in 0 to C_BULLETS - 1 loop
                if s_bullet_y(i) <= signed(resize(s_bullet_speed, 12)) then
                    s_bullet_at_top(i) <= '1';
                else
                    s_bullet_at_top(i) <= '0';
                end if;
            end loop;
            for i in 0 to C_BOMBS - 1 loop
                if s_bomb_y(i) >= to_signed(s_v_active, 12) then
                    s_bomb_at_bot(i) <= '1';
                else
                    s_bomb_at_bot(i) <= '0';
                end if;
            end loop;

            if s_form_y + to_signed(C_FORM_H, 12) >= s_ship_y then
                s_form_at_ship <= '1';
            else
                s_form_at_ship <= '0';
            end if;

            -- aliens_clear from the kill counter — 5-bit compare vs the
            -- old 32-bit OR-reduce.
            if s_dead_count >= to_unsigned(C_ROWS * C_COLS, 5) then
                s_aliens_clear <= '1';
            else
                s_aliens_clear <= '0';
            end if;

            -- Pre-register reset trigger so phase 0's mux selector is
            -- a single flop read (was a 4-input OR with high fanout on
            -- the critical path).
            if s_reset_pending = '1' or s_game_over = '1' or
               s_form_at_ship = '1' or
               s_boot_count = to_unsigned(8, 4) then
                s_do_reset <= '1';
            else
                s_do_reset <= '0';
            end if;
        end if;
    end process;

    -- 8-bit Fibonacci LFSR
    p_lfsr : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            v_fb := s_lfsr(7) xor s_lfsr(5) xor s_lfsr(4) xor s_lfsr(3);
            s_lfsr <= s_lfsr(6 downto 0) & v_fb;
        end if;
    end process;

    -- Projectile scan counter — cycles through the C_PROJ_COUNT projectiles,
    -- advancing once per vsync.  Phase 5/6/7 of the physics FSM check that
    -- one projectile's bunker-hit status each frame.
    p_proj_scan : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                if to_integer(s_proj_scan) >= C_PROJ_COUNT - 1 then
                    s_proj_scan <= (others => '0');
                else
                    s_proj_scan <= s_proj_scan + 1;
                end if;
            end if;
        end if;
    end process;

    -- Bullet scan counter — cycles through the C_BULLETS wand-shots so
    -- phase 2/3/4 of the FSM only does alien-hit detection for ONE
    -- bullet per vsync.  Saves the per-bullet cascade-compare
    -- replication (which would otherwise scale linearly with bullet
    -- count and blow our LC budget at 5 bullets).
    p_bullet_scan : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                if to_integer(s_bullet_scan) >= C_BULLETS - 1 then
                    s_bullet_scan <= (others => '0');
                else
                    s_bullet_scan <= s_bullet_scan + 1;
                end if;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Physics FSM (8 phases)
    -- ========================================================================
    p_physics : process(clk)
        variable v_spawned     : std_logic;
        variable v_picked      : std_logic;
        variable v_can_spawn   : std_logic;
        variable v_col         : integer range 0 to 7;
        variable v_row         : integer range 0 to C_ROWS - 1;
        variable v_bs          : integer range 0 to C_BULLETS - 1;
        variable v_ms          : integer range 0 to C_BOMBS - 1;
        variable v_pb_dx       : signed(11 downto 0);
        variable v_pb_dy       : signed(11 downto 0);
        variable v_cidx        : unsigned(3 downto 0);
        variable v_bid         : unsigned(1 downto 0);
        variable v_hit         : std_logic;
        variable v_x           : signed(11 downto 0);
        variable v_y           : signed(11 downto 0);
        -- Used by serial alien-hit precompute in phase 2
        variable v_scan_idx    : integer range 0 to C_BULLETS - 1;
        variable v_scan_bx     : signed(11 downto 0);
        variable v_scan_by     : signed(11 downto 0);
        variable v_scan_act    : std_logic;
        variable v_scan_dx     : signed(11 downto 0);
        variable v_scan_dy     : signed(11 downto 0);
    begin
        if rising_edge(clk) then

            if s_game_over = '1' and s_timing.vsync_start = '1' then
                s_game_over <= '0';
            end if;

            -- Explosion animator: advance once every C_EXPL_FRAME_TIME
            -- vsyncs; clear the slot at end of F3.  Runs before the case
            -- below so a phase-4 spawn in the same vsync still overrides
            -- (it just lands later in the same sequential process).
            if s_timing.vsync_start = '1' and s_expl_active = '1' then
                if s_expl_timer = 0 then
                    if s_expl_frame = "11" then
                        s_expl_active <= '0';
                    else
                        s_expl_frame <= s_expl_frame + 1;
                        s_expl_timer <=
                            to_unsigned(C_EXPL_FRAME_TIME, 3);
                    end if;
                else
                    s_expl_timer <= s_expl_timer - 1;
                end if;
            end if;

            case to_integer(s_ph_phase) is

                -- --------------------------------------------------
                -- Phase 0: vsync trigger.  Reset or spawn wand-shot.
                -- --------------------------------------------------
                when 0 =>
                    if s_timing.vsync_start = '1' then
                        -- Reset triggered by the registered s_do_reset
                        -- bit (set in p_predicates from reset_pending,
                        -- game_over, form_at_ship, or boot_count=8).
                        -- 1-cycle latency vs the combinational OR but
                        -- keeps the critical path much shorter.
                        if s_do_reset = '1' then
                            s_aliens         <= (others => '1');
                            for b in 0 to C_BUNKER_COUNT - 1 loop
                                s_bunkers(b) <= (others => '1');
                            end loop;
                            s_form_x         <= s_form_x_init;
                            s_form_y         <= s_form_y_init;
                            s_form_dir       <= '1';
                            s_step_counter   <= (others => '0');
                            s_bullet_active  <= (others => '0');
                            s_bomb_active    <= (others => '0');
                            s_bomb_cooldown  <= s_bomb_cd_reset;
                            s_alien_frame    <= '0';
                            s_dead_count     <= (others => '0');
                        elsif s_sw_pause = '0' then
                            -- Round-robin wand-shot spawn: every Switch-7
                            -- edge spawns at the slot s_fire_idx points
                            -- to and advances the index, so every toggle
                            -- produces a visible shot even if all slots
                            -- are already in flight (the oldest is
                            -- replaced).
                            if s_fire_pending = '1' then
                                s_bullet_active(to_integer(s_fire_idx)) <= '1';
                                if s_wizard_facing = '0' then
                                    s_bullet_x(to_integer(s_fire_idx)) <=
                                        s_ship_x +
                                        to_signed(C_WAND_OFFSET_X, 12);
                                else
                                    s_bullet_x(to_integer(s_fire_idx)) <=
                                        s_ship_x +
                                        to_signed(C_SHIP_W
                                                  - C_WAND_OFFSET_X
                                                  - C_BULLET_W, 12);
                                end if;
                                s_bullet_y(to_integer(s_fire_idx)) <=
                                    s_ship_y +
                                    to_signed(C_WAND_OFFSET_Y
                                              - C_BULLET_H, 12);
                                if to_integer(s_fire_idx) >= C_BULLETS - 1 then
                                    s_fire_idx <= (others => '0');
                                else
                                    s_fire_idx <= s_fire_idx + 1;
                                end if;
                            end if;

                            if s_step_counter >= s_step_period then
                                s_step_counter <= (others => '0');
                                s_alien_frame  <= not s_alien_frame;
                            else
                                s_step_counter <= s_step_counter + 1;
                            end if;

                            if s_bomb_cooldown > 0 then
                                s_bomb_cooldown <= s_bomb_cooldown - 1;
                            end if;
                        end if;
                        s_ph_phase <= to_unsigned(1, 3);
                    end if;

                -- --------------------------------------------------
                -- Phase 1: projectile advance + bomb spawn + form predict
                -- --------------------------------------------------
                when 1 =>
                    if s_sw_pause = '0' then
                        -- Advance each bullet
                        for i in 0 to C_BULLETS - 1 loop
                            if s_bullet_active(i) = '1' then
                                if s_bullet_at_top(i) = '1' then
                                    s_bullet_active(i) <= '0';
                                else
                                    s_bullet_y(i) <= s_bullet_y(i) -
                                        signed(resize(s_bullet_speed, 12));
                                end if;
                            end if;
                        end loop;

                        -- Advance each bomb
                        for i in 0 to C_BOMBS - 1 loop
                            if s_bomb_active(i) = '1' then
                                if s_bomb_at_bot(i) = '1' then
                                    s_bomb_active(i) <= '0';
                                else
                                    s_bomb_y(i) <= s_bomb_y(i) +
                                        signed(resize(s_bomb_speed, 12));
                                end if;
                            end if;
                        end loop;

                        -- Bomb spawn: LFSR picks col 0..7; only spawn from
                        -- cols 0..5 that have at least one live alien.  We
                        -- drop from the LOWEST live row in that column so
                        -- the bomb appears to leave the actual alien
                        -- sprite, not the empty cell below.  Skip (without
                        -- consuming the cooldown) on dead columns so the
                        -- next pick advances the LFSR and retries.
                        if s_bomb_cooldown = 0 and s_aliens_clear = '0' then
                            v_col := to_integer(unsigned(s_lfsr(2 downto 0)));
                            v_can_spawn := '0';
                            v_row := 0;
                            if v_col < C_COLS then
                                if s_aliens(2 * C_COLS + v_col) = '1' then
                                    v_row := 2; v_can_spawn := '1';
                                elsif s_aliens(1 * C_COLS + v_col) = '1' then
                                    v_row := 1; v_can_spawn := '1';
                                elsif s_aliens(0 * C_COLS + v_col) = '1' then
                                    v_row := 0; v_can_spawn := '1';
                                end if;
                            end if;
                            if v_can_spawn = '1' then
                                v_x := s_form_x + to_signed(
                                    v_col * C_CELL_W + C_CELL_W / 2
                                    - C_BOMB_W / 2, 12);
                                -- sprite bottom = cell top + C_SPRITE_OFF + C_SPRITE_H
                                --               = row*144 + 8 + 128 = row*144 + 136
                                v_y := s_form_y + to_signed(
                                    v_row * C_CELL_H + C_SPRITE_OFF
                                    + C_SPRITE_H, 12);
                                v_spawned := '0';
                                for i in 0 to C_BOMBS - 1 loop
                                    if v_spawned = '0' and
                                       s_bomb_active(i) = '0' then
                                        s_bomb_x(i)      <= v_x;
                                        s_bomb_y(i)      <= v_y;
                                        s_bomb_active(i) <= '1';
                                        v_spawned := '1';
                                    end if;
                                end loop;
                                s_bomb_cooldown <= s_bomb_cd_reset;
                            end if;
                        end if;

                        -- Formation x predict
                        if s_step_counter = 0 then
                            if s_form_dir = '1' then
                                s_ph_new_fx <= s_form_x +
                                               to_signed(C_FORM_STEP_X, 12);
                            else
                                s_ph_new_fx <= s_form_x -
                                               to_signed(C_FORM_STEP_X, 12);
                            end if;
                        else
                            s_ph_new_fx <= s_form_x;
                        end if;
                    else
                        s_ph_new_fx <= s_form_x;
                    end if;
                    s_ph_phase <= to_unsigned(2, 3);

                -- --------------------------------------------------
                -- Phase 2: formation commit + register hit-detect deltas
                -- --------------------------------------------------
                when 2 =>
                    if s_sw_pause = '0' and s_step_counter = 0 then
                        if s_ph_new_fx >= s_form_x_max then
                            s_form_x   <= s_form_x_max;
                            s_form_dir <= '0';
                            s_form_y   <= s_form_y + to_signed(32, 12);
                        elsif s_ph_new_fx <= C_FORM_X_MIN then
                            s_form_x   <= C_FORM_X_MIN;
                            s_form_dir <= '1';
                            s_form_y   <= s_form_y + to_signed(32, 12);
                        else
                            s_form_x <= s_ph_new_fx;
                        end if;
                    end if;

                    -- Pick the scanned bullet (round-robin via
                    -- s_bullet_scan) and pre-decode its row/col cascade
                    -- flags so phase 3 is just a priority encoder.  Only
                    -- one bullet is checked per vsync — see p_bullet_scan.
                    v_scan_idx := to_integer(s_bullet_scan);
                    v_scan_bx  := s_bullet_x(v_scan_idx);
                    v_scan_by  := s_bullet_y(v_scan_idx);
                    v_scan_act := s_bullet_active(v_scan_idx);
                    v_scan_dx  := v_scan_bx - s_form_x;
                    v_scan_dy  := v_scan_by - s_form_y;
                    s_ph_alien_sel <= s_bullet_scan;
                    if v_scan_act = '1' then
                        -- Power-of-two cells → hit cell is a bit-slice.
                        s_ph_bf_col <= unsigned(v_scan_dx(9 downto 7));
                        s_ph_bf_row <= unsigned(v_scan_dy(8 downto 7));
                        if v_scan_dy >= to_signed(0, 12) and
                           v_scan_dy <  to_signed(C_FORM_H, 12) then
                            s_ph_bf_dy_in <= '1';
                        else
                            s_ph_bf_dy_in <= '0';
                        end if;
                        if v_scan_dx >= to_signed(0, 12) and
                           v_scan_dx <  to_signed(C_FORM_W, 12) then
                            s_ph_bf_dx_in <= '1';
                        else
                            s_ph_bf_dx_in <= '0';
                        end if;
                    else
                        s_ph_bf_dy_in <= '0';
                        s_ph_bf_dx_in <= '0';
                    end if;
                    -- Bomb-vs-ship delta for the scanned bomb (shares
                    -- s_bullet_scan since C_BULLETS = C_BOMBS = 5).
                    s_ph_bs_dx  <= s_bomb_x(v_scan_idx) - s_ship_x;
                    s_ph_bs_dy  <= s_bomb_y(v_scan_idx) - s_ship_y;
                    s_ph_bs_act <= s_bomb_active(v_scan_idx);
                    s_ph_phase  <= to_unsigned(3, 3);

                -- --------------------------------------------------
                -- Phase 3: alien hit precompute for the scanned bullet.
                -- Single-bullet encode → alien_idx (5 bits) using the
                -- pre-registered row + col cascade flags from phase 2.
                -- --------------------------------------------------
                when 3 =>
                    s_ph_alien_in <= s_ph_bf_dx_in and s_ph_bf_dy_in;
                    -- alien_idx = {row(1..0), col(2..0)} straight from the
                    -- bit-sliced cell — no priority encoder.
                    s_ph_alien_idx <= s_ph_bf_row & s_ph_bf_col;
                    -- Pre-read alien-alive at the hit cell (s_aliens is stable
                    -- between phases) so phase 4 commits off a single flop.
                    s_ph_alien_alive_pre <=
                        s_aliens(to_integer(s_ph_bf_row & s_ph_bf_col));

                    -- Explosion spawn = cell origin + centring offset.
                    -- cell origin = form + (cell << 7); the 48-wide blast is
                    -- centred in the 128 cell → +40.  Built from the sliced
                    -- col/row (a shift), parallel to the alien_idx path.
                    s_ph_expl_x_cand <= s_form_x +
                        signed(shift_left(resize(s_ph_bf_col, 12), 7)) +
                        to_signed(40, 12);
                    s_ph_expl_y_cand <= s_form_y +
                        signed(shift_left(resize(s_ph_bf_row, 12), 7)) +
                        to_signed(40, 12);

                    s_ph_phase <= to_unsigned(4, 3);

                -- --------------------------------------------------
                -- Phase 4: alien hit commit for the scanned bullet
                -- (s_ph_alien_sel).  On a kill: clear the alien, mark
                -- the bullet inactive, bump the dead-count for the speed
                -- boost, spawn an explosion at the alien's centre.
                -- --------------------------------------------------
                when 4 =>
                    if s_ph_alien_in = '1' and
                       s_ph_alien_alive_pre = '1' then
                        s_aliens(to_integer(s_ph_alien_idx)) <= '0';
                        s_bullet_active(to_integer(s_ph_alien_sel)) <= '0';
                        if s_dead_count < to_unsigned(31, 5) then
                            s_dead_count <= s_dead_count + 1;
                        end if;
                        if s_expl_active = '0' then
                            s_expl_active <= '1';
                            s_expl_frame  <= "00";
                            s_expl_timer  <=
                                to_unsigned(C_EXPL_FRAME_TIME, 3);
                            -- Use pre-computed candidates from phase 3
                            s_expl_x <= s_ph_expl_x_cand;
                            s_expl_y <= s_ph_expl_y_cand;
                        end if;
                    end if;

                    -- Latch the scanned projectile so phase 5 sees a flop,
                    -- not a long 6:1 mux + subtract chain.
                    if to_integer(s_proj_scan) < C_BULLETS then
                        s_ph_proj_x <= s_bullet_x(to_integer(s_proj_scan));
                        s_ph_proj_y <= s_bullet_y(to_integer(s_proj_scan));
                        s_ph_proj_a <= s_bullet_active(to_integer(s_proj_scan));
                    elsif to_integer(s_proj_scan) < C_PROJ_COUNT then
                        s_ph_proj_x <= s_bomb_x(to_integer(s_proj_scan)
                                                - C_BULLETS);
                        s_ph_proj_y <= s_bomb_y(to_integer(s_proj_scan)
                                                - C_BULLETS);
                        s_ph_proj_a <= s_bomb_active(to_integer(s_proj_scan)
                                                     - C_BULLETS);
                    else
                        s_ph_proj_a <= '0';
                    end if;
                    s_ph_phase <= to_unsigned(5, 3);

                -- --------------------------------------------------
                -- Phase 5: bunker hit precompute for s_proj_scan only.
                -- Mux the scanned projectile's position into v_x, v_y,
                -- v_hit; then check all 4 bunkers in parallel and
                -- register the hit-bunker id + chunk index.
                -- --------------------------------------------------
                when 5 =>
                    -- Use the registered scanned projectile from phase 4
                    v_x   := s_ph_proj_x;
                    v_y   := s_ph_proj_y;
                    v_hit := s_ph_proj_a;

                    v_pb_dy := v_y - s_bunker_y;
                    s_ph_proj_hit <= '0';
                    v_cidx := (others => '0');
                    v_bid  := (others => '0');
                    if v_hit = '1' and
                       v_pb_dy >= to_signed(0, 12) and
                       v_pb_dy <  to_signed(C_BUNKER_H, 12) then
                        for b in 0 to C_BUNKER_COUNT - 1 loop
                            v_pb_dx := v_x - s_bunker_x(b);
                            if v_pb_dx >= to_signed(0, 12) and
                               v_pb_dx <  to_signed(C_BUNKER_W, 12) then
                                s_ph_proj_hit <= '1';
                                -- chunk size 32 × 32 → row(1..0)=dy(6..5),
                                -- col(1..0)=dx(6..5)
                                v_cidx := unsigned(v_pb_dy(6 downto 5)) &
                                          unsigned(v_pb_dx(6 downto 5));
                                v_bid  := to_unsigned(b, 2);
                            end if;
                        end loop;
                    end if;
                    s_ph_proj_cidx <= v_cidx;
                    s_ph_proj_bid  <= v_bid;
                    s_ph_proj_sel  <= s_proj_scan;   -- latch for phases 6/7
                    s_ph_phase     <= to_unsigned(6, 3);

                -- --------------------------------------------------
                -- Phase 6: chunk-alive lookup for the scanned projectile
                --          + aliens-clear respawn
                -- --------------------------------------------------
                when 6 =>
                    s_ph_proj_alive <= s_ph_proj_hit and
                        s_bunkers(to_integer(s_ph_proj_bid))
                                 (to_integer(s_ph_proj_cidx));

                    if s_aliens_clear = '1' then
                        s_aliens     <= (others => '1');
                        s_form_x     <= s_form_x_init;
                        s_form_y     <= s_form_y_init;
                        s_form_dir   <= '1';
                        s_dead_count <= (others => '0');
                    end if;

                    s_ph_phase <= to_unsigned(7, 3);

                -- --------------------------------------------------
                -- Phase 7: commit bunker chunk + deactivate scanned
                --          projectile, plus bomb-vs-ship hit check
                -- --------------------------------------------------
                when 7 =>
                    if s_ph_proj_alive = '1' then
                        s_bunkers(to_integer(s_ph_proj_bid))
                                 (to_integer(s_ph_proj_cidx)) <= '0';
                        if to_integer(s_ph_proj_sel) < C_BULLETS then
                            s_bullet_active(to_integer(s_ph_proj_sel)) <= '0';
                        elsif to_integer(s_ph_proj_sel) < C_PROJ_COUNT then
                            s_bomb_active(to_integer(s_ph_proj_sel)
                                          - C_BULLETS) <= '0';
                        end if;
                    end if;

                    -- Bomb-vs-ship for the scanned bomb only (registered
                    -- delta from phase 2 + active flag).
                    if s_ph_bs_act = '1' and
                       s_ph_bs_dx >= to_signed(0, 12) and
                       s_ph_bs_dx <  to_signed(C_SHIP_W, 12) and
                       s_ph_bs_dy >= to_signed(0, 12) and
                       s_ph_bs_dy <  to_signed(C_SHIP_H, 12) then
                        s_game_over <= '1';
                    end if;

                    s_ph_phase <= to_unsigned(0, 3);

                when others =>
                    s_ph_phase <= to_unsigned(0, 3);
            end case;
        end if;
    end process;

    -- ========================================================================
    -- Render — S1: register coords
    -- ========================================================================
    p_stage1 : process(clk)
    begin
        if rising_edge(clk) then
            s_stg1_hx <= signed(resize(s_h_count, 12));
            s_stg1_vy <= signed(resize(s_v_count, 12));
        end if;
    end process;

    -- ========================================================================
    -- Render — S2: per-region deltas (live state reads)
    -- ========================================================================
    p_stage2 : process(clk)
    begin
        if rising_edge(clk) then
            s_stg2_form_dx <= s_stg1_hx - s_form_x;
            s_stg2_form_dy <= s_stg1_vy - s_form_y;
            s_stg2_ship_dx <= s_stg1_hx - s_ship_x;
            s_stg2_ship_dy <= s_stg1_vy - s_ship_y;
            for i in 0 to C_BULLETS - 1 loop
                s_stg2_bull_dx(i) <= s_stg1_hx - s_bullet_x(i);
                s_stg2_bull_dy(i) <= s_stg1_vy - s_bullet_y(i);
            end loop;
            for i in 0 to C_BOMBS - 1 loop
                s_stg2_bomb_dx(i) <= s_stg1_hx - s_bomb_x(i);
                s_stg2_bomb_dy(i) <= s_stg1_vy - s_bomb_y(i);
            end loop;
            for b in 0 to C_BUNKER_COUNT - 1 loop
                s_stg2_bnk_dx(b) <= s_stg1_hx - s_bunker_x(b);
            end loop;
            s_stg2_bnk_dy <= s_stg1_vy - s_bunker_y;
            s_stg2_expl_dx <= s_stg1_hx - s_expl_x;
            s_stg2_expl_dy <= s_stg1_vy - s_expl_y;
            s_stg2_expl_a  <= s_expl_active;
            s_stg2_expl_fr <= s_expl_frame;
            -- Power-of-two (128) cells → the row/col are bit-slices of the
            -- registered form deltas in S3, so S2 only needs the in-grid
            -- range checks (no 12-bit comparator cascade, no priority encoder).
            if (s_stg1_vy - s_form_y) >= to_signed(0, 12)
            and (s_stg1_vy - s_form_y) < to_signed(C_FORM_H, 12) then
                s_stg2_fdy_in <= '1';
            else
                s_stg2_fdy_in <= '0';
            end if;
            if (s_stg1_hx - s_form_x) >= to_signed(0, 12)
            and (s_stg1_hx - s_form_x) < to_signed(C_FORM_W, 12) then
                s_stg2_fdx_in <= '1';
            else
                s_stg2_fdx_in <= '0';
            end if;
            s_stg2_frame  <= s_alien_frame;
            s_stg2_bull_a <= s_bullet_active;
            s_stg2_bomb_a <= s_bomb_active;

            -- Ghost X deltas (live ring state changes only during vblank);
            -- the Y delta is shared with the ship (s_stg2_ship_dy).  Per-ghost
            -- chroma is frame-constant, so it is NOT carried per pixel — only
            -- the winning-ghost selector rides the pipe; colour is read back
            -- from the ring at S5.
            for i in 0 to C_TRAIL_LEN - 1 loop
                s_stg2_trail_dx(i) <= s_stg1_hx - s_trail_x(i);
            end loop;
            s_stg2_trail_v  <= s_trail_valid;
            s_stg2_trail_f  <= s_trail_face;
        end if;
    end process;

    -- ========================================================================
    -- Render — S3: in-bbox flags + indices + ROM addresses
    -- ========================================================================
    p_stage3 : process(clk)
        variable v_ship_in_x   : std_logic;
        variable v_ship_in_y   : std_logic;
        variable v_in_x        : std_logic;
        variable v_in_y        : std_logic;
        variable v_chunk_idx   : unsigned(3 downto 0);
        variable v_bnk_col_use : unsigned(6 downto 0);
        variable v_bnk_in_y    : std_logic;
        variable v_row_bits    : unsigned(1 downto 0);
        variable v_col_bits    : unsigned(2 downto 0);
        variable v_gh_in       : std_logic;
        variable v_gh_col      : unsigned(6 downto 0);
        variable v_gh_f        : std_logic;
        variable v_gh_sel      : integer range 0 to C_TRAIL_LEN - 1;
    begin
        if rising_edge(clk) then
            -- Power-of-two (128) cells: column = form_dx[9:7], row =
            -- form_dy[8:7], and the in-cell coordinate is just the low 7
            -- bits — all plain bit-slices, no compares/subtracts.  The
            -- sprite fills the cell, so "in grid" == "on sprite".
            v_col_bits := unsigned(s_stg2_form_dx(9 downto 7));
            v_row_bits := unsigned(s_stg2_form_dy(8 downto 7));
            s_stg3_in_form <= s_stg2_fdx_in and s_stg2_fdy_in;

            -- alien_idx = {row(1..0), col(2..0)} → 5 bits
            s_stg3_alien_idx <= v_row_bits & v_col_bits;

            -- 2× nearest-neighbour: source coord = in-cell coord >> 1.
            -- ROM address = {frame, source_row(5..0)} → 128-entry ROM.
            s_stg3_alien_addr <= s_stg2_frame &
                                 unsigned(s_stg2_form_dy(6 downto 1));
            s_stg3_alien_col  <= unsigned(s_stg2_form_dx(6 downto 1));

            -- Ship
            if s_stg2_ship_dx >= to_signed(0, 12) and
               s_stg2_ship_dx <  to_signed(C_SHIP_W, 12) then
                v_ship_in_x := '1';
            else
                v_ship_in_x := '0';
            end if;
            if s_stg2_ship_dy >= to_signed(0, 12) and
               s_stg2_ship_dy <  to_signed(C_SHIP_H, 12) then
                v_ship_in_y := '1';
            else
                v_ship_in_y := '0';
            end if;
            s_stg3_in_ship  <= v_ship_in_x and v_ship_in_y;
            s_stg3_ship_row <= unsigned(s_stg2_ship_dy(6 downto 0));
            s_stg3_ship_col <= unsigned(s_stg2_ship_dx(6 downto 0));

            -- Ghost afterimages: same 128-tall Y band as the live wizard
            -- (reuse v_ship_in_y), each at its own X.  Ghosts never overlap
            -- (spacing ≥ sprite width), so pick the single one covering this
            -- pixel and pass its column/facing/ring-index to S5 — one row mux
            -- regardless of ghost count.
            v_gh_in  := '0';
            v_gh_col := (others => '0');
            v_gh_f   := '0';
            v_gh_sel := 0;
            for i in 0 to C_TRAIL_LEN - 1 loop
                if s_stg2_trail_v(i) = '1' and v_ship_in_y = '1' and
                   s_stg2_trail_dx(i) >= to_signed(0, 12) and
                   s_stg2_trail_dx(i) <  to_signed(C_SHIP_W, 12) then
                    v_gh_in  := '1';
                    v_gh_col := unsigned(s_stg2_trail_dx(i)(6 downto 0));
                    v_gh_f   := s_stg2_trail_f(i);
                    v_gh_sel := i;
                end if;
            end loop;
            s_stg3_gh_in  <= v_gh_in;
            s_stg3_gh_col <= v_gh_col;
            s_stg3_gh_f   <= v_gh_f;
            s_stg3_gh_sel <= v_gh_sel;

            -- Bullets (each WAND_SHOT 8x16)
            for i in 0 to C_BULLETS - 1 loop
                if s_stg2_bull_dx(i) >= to_signed(0, 12) and
                   s_stg2_bull_dx(i) <  to_signed(C_BULLET_W, 12) then
                    v_in_x := '1';
                else
                    v_in_x := '0';
                end if;
                if s_stg2_bull_dy(i) >= to_signed(0, 12) and
                   s_stg2_bull_dy(i) <  to_signed(C_BULLET_H, 12) then
                    v_in_y := '1';
                else
                    v_in_y := '0';
                end if;
                s_stg3_in_bullet(i) <= v_in_x and v_in_y and s_stg2_bull_a(i);
            end loop;

            -- Bombs (rectangles)
            for i in 0 to C_BOMBS - 1 loop
                if s_stg2_bomb_dx(i) >= to_signed(0, 12) and
                   s_stg2_bomb_dx(i) <  to_signed(C_BOMB_W, 12) then
                    v_in_x := '1';
                else
                    v_in_x := '0';
                end if;
                if s_stg2_bomb_dy(i) >= to_signed(0, 12) and
                   s_stg2_bomb_dy(i) <  to_signed(C_BOMB_H, 12) then
                    v_in_y := '1';
                else
                    v_in_y := '0';
                end if;
                s_stg3_in_bomb(i) <= v_in_x and v_in_y and s_stg2_bomb_a(i);
            end loop;

            -- Bunkers (4 in parallel, shared dy)
            if s_stg2_bnk_dy >= to_signed(0, 12) and
               s_stg2_bnk_dy <  to_signed(C_BUNKER_H, 12) then
                v_bnk_in_y := '1';
            else
                v_bnk_in_y := '0';
            end if;
            v_chunk_idx   := (others => '0');
            v_bnk_col_use := (others => '0');
            for b in 0 to C_BUNKER_COUNT - 1 loop
                if s_stg2_bnk_dx(b) >= to_signed(0, 12) and
                   s_stg2_bnk_dx(b) <  to_signed(C_BUNKER_W, 12) then
                    v_in_x := '1';
                else
                    v_in_x := '0';
                end if;
                if v_in_x = '1' and v_bnk_in_y = '1' then
                    s_stg3_in_bunker(b) <= '1';
                    -- chunk size 32 × 32 → row(1..0)=dy(6..5),
                    -- col(1..0)=dx(6..5)
                    v_chunk_idx   := unsigned(s_stg2_bnk_dy(6 downto 5)) &
                                     unsigned(s_stg2_bnk_dx(b)(6 downto 5));
                    v_bnk_col_use := unsigned(s_stg2_bnk_dx(b)(6 downto 0));
                else
                    s_stg3_in_bunker(b) <= '0';
                end if;
            end loop;
            s_stg3_chunk_idx <= v_chunk_idx;
            s_stg3_bnk_col   <= v_bnk_col_use;
            s_stg3_bnk_row   <= unsigned(s_stg2_bnk_dy(6 downto 0));

            -- Explosion (48×48 sprite at expl_x/y, animated frame).  ROM
            -- address = {frame(1..0), row(5..0)} (8 bits, 256-entry).
            if s_stg2_expl_dx >= to_signed(0, 12) and
               s_stg2_expl_dx <  to_signed(C_EXPL_W, 12) then
                v_in_x := '1';
            else
                v_in_x := '0';
            end if;
            if s_stg2_expl_dy >= to_signed(0, 12) and
               s_stg2_expl_dy <  to_signed(C_EXPL_H, 12) then
                v_in_y := '1';
            else
                v_in_y := '0';
            end if;
            s_stg3_in_expl   <= v_in_x and v_in_y and s_stg2_expl_a;
            s_stg3_expl_addr <= s_stg2_expl_fr &
                                unsigned(s_stg2_expl_dy(5 downto 0));
            s_stg3_expl_col  <= unsigned(s_stg2_expl_dx(5 downto 0));
        end if;
    end process;

    -- ========================================================================
    -- Render — S4: sync ROM reads + state lookups
    -- ========================================================================
    p_stage4 : process(clk)
        variable v_alien_idx : integer range 0 to C_ALIEN_SLOTS - 1;
        variable v_chunk_hit : std_logic;
        variable v_any_bnk   : std_logic;
    begin
        if rising_edge(clk) then
            -- ROM reads (BRAM-inferred)
            s_stg4_alien_row  <=
                C_ALIEN_ROM(to_integer(s_stg3_alien_addr));
            s_stg4_wizard_row <=
                WIZARD_SPRITE(to_integer(s_stg3_ship_row));
            -- Bunker ROM is 112 entries (0..111) but bnk_row is 7-bit
            -- (0..127); clamp to keep simulation in-bounds.  Synth
            -- usually optimises the guard away when the BRAM is wider.
            if to_integer(s_stg3_bnk_row) < C_BUNKER_H then
                s_stg4_bunker_row <=
                    VIDEOMANCER_BUNKER(to_integer(s_stg3_bnk_row));
            else
                s_stg4_bunker_row <= (others => '0');
            end if;

            -- Alien-alive lookup (32-to-1)
            v_alien_idx := to_integer(s_stg3_alien_idx);
            s_stg4_alien_alive <= s_aliens(v_alien_idx);

            -- Bunker chunk-alive (priority over the in_bunker flags)
            v_chunk_hit := '0';
            v_any_bnk   := '0';
            for b in 0 to C_BUNKER_COUNT - 1 loop
                if s_stg3_in_bunker(b) = '1' then
                    v_any_bnk := '1';
                    if s_bunkers(b)(to_integer(s_stg3_chunk_idx)) = '1' then
                        v_chunk_hit := '1';
                    end if;
                end if;
            end loop;
            s_stg4_chunk_alive   <= v_chunk_hit;
            s_stg4_in_bunker_any <= v_any_bnk;
            s_stg4_bnk_col       <= s_stg3_bnk_col;

            -- Explosion ROM read
            s_stg4_expl_row <=
                C_EXPL_ROM(to_integer(s_stg3_expl_addr));
            s_stg4_in_expl  <= s_stg3_in_expl;
            s_stg4_expl_col <= s_stg3_expl_col;

            s_stg4_in_form   <= s_stg3_in_form;
            s_stg4_alien_col <= s_stg3_alien_col;
            s_stg4_in_ship   <= s_stg3_in_ship;
            s_stg4_ship_col  <= s_stg3_ship_col;
            s_stg4_in_bullet <= s_stg3_in_bullet;
            s_stg4_in_bomb   <= s_stg3_in_bomb;
            s_stg4_gh_in  <= s_stg3_gh_in;
            s_stg4_gh_col <= s_stg3_gh_col;
            s_stg4_gh_f   <= s_stg3_gh_f;
            s_stg4_gh_sel <= s_stg3_gh_sel;
        end if;
    end process;

    -- ========================================================================
    -- Render — S5: bit selects + per-region combine
    -- ========================================================================
    p_stage5 : process(clk)
        variable v_alien_bit  : std_logic;
        variable v_wizard_bit : std_logic;
        variable v_bunker_bit : std_logic;
        variable v_expl_bit   : std_logic;
        variable v_on_bullet  : std_logic;
        variable v_on_bomb    : std_logic;
        variable v_gh_bit     : std_logic;
    begin
        if rising_edge(clk) then
            -- Alien (64-bit row, bit 63 leftmost)
            v_alien_bit := s_stg4_alien_row(63 -
                            to_integer(s_stg4_alien_col));

            -- Wizard (128-bit row, mirrored when facing right).  Reads live
            -- s_wizard_facing (changes only during vblank).
            if s_wizard_facing = '0' then
                v_wizard_bit := s_stg4_wizard_row(127 -
                                 to_integer(s_stg4_ship_col));
            else
                v_wizard_bit := s_stg4_wizard_row(
                                 to_integer(s_stg4_ship_col));
            end if;

            -- Bunker (128-bit row)
            v_bunker_bit := s_stg4_bunker_row(127 -
                             to_integer(s_stg4_bnk_col));

            s_stg5_on_alien  <= s_stg4_in_form
                                and s_stg4_alien_alive and v_alien_bit;
            s_stg5_on_ship   <= s_stg4_in_ship and v_wizard_bit;
            s_stg5_on_bunker <= s_stg4_in_bunker_any and
                                s_stg4_chunk_alive and v_bunker_bit;

            -- Wand-shots render as solid rectangles — dropped per-bullet
            -- sprite lookup to recover LCs for the 3×6 alien grid + 3+3
            -- projectiles + explosion.
            v_on_bullet := '0';
            for i in 0 to C_BULLETS - 1 loop
                if s_stg4_in_bullet(i) = '1' then
                    v_on_bullet := '1';
                end if;
            end loop;
            s_stg5_on_bullet <= v_on_bullet;

            v_on_bomb := '0';
            for i in 0 to C_BOMBS - 1 loop
                if s_stg4_in_bomb(i) = '1' then
                    v_on_bomb := '1';
                end if;
            end loop;
            s_stg5_on_bomb <= v_on_bomb;

            -- Explosion bit (48-bit row, bit 47 leftmost).  expl_col is
            -- a 6-bit value (0..63) but the sprite is only 48 wide, so
            -- gate the index to stay in bounds — out-of-range pixels
            -- aren't inside the sprite anyway (s_stg4_in_expl = 0).
            if to_integer(s_stg4_expl_col) < C_EXPL_W then
                v_expl_bit := s_stg4_expl_row(C_EXPL_W - 1 -
                               to_integer(s_stg4_expl_col));
            else
                v_expl_bit := '0';
            end if;
            s_stg5_on_expl <= s_stg4_in_expl and v_expl_bit;

            -- Floor line: full-width horizontal stripe under the wizard.
            -- Tests live s_floor_y (only changes with v_active).
            if s_stg1_vy >= s_floor_y and
               s_stg1_vy <  s_floor_y + to_signed(C_FLOOR_H, 12) then
                s_stg5_on_floor <= '1';
            else
                s_stg5_on_floor <= '0';
            end if;

            -- Ghost afterimage: the winning ghost shares the live wizard ROM
            -- row (same Y band), so its silhouette bit is that row indexed at
            -- the ghost's column, mirrored per its stored facing.  ONE mux —
            -- the non-overlap guarantee meant S3 already chose the ghost.
            -- Chroma was resolved at drop time, so no palette LUT here.
            if s_stg4_gh_f = '0' then
                v_gh_bit := s_stg4_wizard_row(
                                127 - to_integer(s_stg4_gh_col));
            else
                v_gh_bit := s_stg4_wizard_row(to_integer(s_stg4_gh_col));
            end if;
            s_stg5_on_trail <= s_stg4_gh_in and v_gh_bit;
            s_stg5_trail_u  <= s_trail_u(s_stg4_gh_sel);
            s_stg5_trail_v  <= s_trail_v(s_stg4_gh_sel);
        end if;
    end process;

    -- ========================================================================
    -- Render — S6: color mux
    -- ========================================================================
    -- Foreground chroma, resolved once per clock from the sw_color toggle.
    p_fg_color : process(clk)
    begin
        if rising_edge(clk) then
            if s_sw_color = '1' then
                s_fg_u <= s_hue_pot;
                s_fg_v <= C_MAX_VAL - s_hue_pot;
            else
                s_fg_u <= C_CHROMA_MID;
                s_fg_v <= C_CHROMA_MID;
            end if;
        end if;
    end process;

    p_stage6 : process(clk)
        variable v_fg  : std_logic;
    begin
        if rising_edge(clk) then
            -- Foreground sprites/lines (everything except the ghosts).
            v_fg := s_stg5_on_alien or s_stg5_on_ship or
                    s_stg5_on_bullet or s_stg5_on_bomb or
                    s_stg5_on_bunker or s_stg5_on_expl or
                    s_stg5_on_floor;
            s_out_show <= v_fg or s_stg5_on_trail;

            -- Single 2:1 select per channel: foreground (incl. the live
            -- wizard) wins over the faded ghost afterimages.
            if v_fg = '1' then
                s_out_y <= s_bright_pot;
                s_out_u <= s_fg_u;
                s_out_v <= s_fg_v;
            else
                s_out_y <= to_unsigned(C_GHOST_LUMA, 10);
                s_out_u <= s_stg5_trail_u;
                s_out_v <= s_stg5_trail_v;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Sync + data delay
    -- ========================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_sync_pipe(0) <= data_in.field_n & data_in.avid &
                              data_in.vsync_n & data_in.hsync_n;
            for i in 1 to C_DELAY - 1 loop
                s_sync_pipe(i) <= s_sync_pipe(i - 1);
            end loop;

            s_y_delay(0) <= data_in.y;
            s_u_delay(0) <= data_in.u;
            s_v_delay(0) <= data_in.v;
            for i in 1 to C_DELAY - 1 loop
                s_y_delay(i) <= s_y_delay(i - 1);
                s_u_delay(i) <= s_u_delay(i - 1);
                s_v_delay(i) <= s_v_delay(i - 1);
            end loop;
        end if;
    end process;

    -- ========================================================================
    -- Wet pixel mux
    -- ========================================================================
    p_wet_select : process(s_out_show, s_sw_bg_black,
                           s_out_y, s_out_u, s_out_v,
                           s_y_delay, s_u_delay, s_v_delay)
    begin
        if s_out_show = '1' then
            s_wet_y <= std_logic_vector(s_out_y);
            s_wet_u <= std_logic_vector(s_out_u);
            s_wet_v <= std_logic_vector(s_out_v);
        else
            if s_sw_bg_black = '1' then
                s_wet_y <= std_logic_vector(to_unsigned(0,   10));
                s_wet_u <= std_logic_vector(to_unsigned(512, 10));
                s_wet_v <= std_logic_vector(to_unsigned(512, 10));
            else
                s_wet_y <= s_y_delay(C_DELAY - 1);
                s_wet_u <= s_u_delay(C_DELAY - 1);
                s_wet_v <= s_v_delay(C_DELAY - 1);
            end if;
        end if;
    end process;

    -- ========================================================================
    -- IO alignment
    -- ========================================================================
    -- The wet pixel goes straight to the output (no wet/dry interpolation —
    -- mix was hardcoded fully wet).  Sync is tapped at s_sync_pipe(C_DELAY-1),
    -- matching the wet pixel's 8-clock render depth.  Bit 2 of the sync word
    -- is avid (field_n & avid & vsync_n & hsync_n).
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io_0.y       <= s_wet_y;
            s_io_0.u       <= s_wet_u;
            s_io_0.v       <= s_wet_v;
            s_io_0.hsync_n <= s_sync_pipe(C_DELAY - 1)(0);
            s_io_0.vsync_n <= s_sync_pipe(C_DELAY - 1)(1);
            s_io_0.avid    <= s_sync_pipe(C_DELAY - 1)(2);
            s_io_0.field_n <= s_sync_pipe(C_DELAY - 1)(3);
            s_io_1 <= s_io_0;
        end if;
    end process;

    data_out.y       <= s_io_1.y;
    data_out.u       <= s_io_1.u;
    data_out.v       <= s_io_1.v;
    data_out.hsync_n <= s_io_1.hsync_n;
    data_out.vsync_n <= s_io_1.vsync_n;
    data_out.avid    <= s_io_1.avid;
    data_out.field_n <= s_io_1.field_n;

end architecture invaders;
