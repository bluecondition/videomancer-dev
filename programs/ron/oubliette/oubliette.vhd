-- oubliette.vhd
-- Copyright (C) 2026  bluecondition
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Oubliette - a top-down, turn-based roguelike for the Videomancer.
--
-- Driven entirely by the front panel (6 knobs, 5 switches, 1 slider): pick a
-- class, explore procedurally generated dungeon levels, fight monsters in
-- turn-based combat, loot treasure, and descend.  Pure synthesis - the incoming
-- video is ignored except for its sync, to which the output is genlocked.
--
-- Milestones:
--   M0  skeleton            M1  static tile map      M2  camera + player
--   M3  monsters + combat + HUD                                          <-- HERE (M3a: entity layer)
--   M4  procedural generation   M5  items / overlay / win-lose
--
-- =====================================================================
-- Register / front-panel map
--   K1 Class | K2 Seed | K3 Action | K6 Hue
--   S7 Step/Act (edge=turn) | S8 Status | S9 Use (edge) | S11 New game (edge)
--   P12 Heading (EXPLORE) / target (COMBAT)
-- =====================================================================
--
-- Colour: BT.601 YUV stored U/V-SWAPPED for the HW convention.
--
-- Render pipeline (one pixel/clock; 16 px power-of-two cells):
--   S0 p_acc   world coords (screen + per-frame scroll)
--   S1 p_cell  cell addr / gx / gy / in-bounds / in-player  (compares isolated)
--   S2 p_map   tilemap read     ||  p_ent  16-way entity bbox + priority
--   S3 p_spr   entity sprite ROM read
--   S4 p_trow  tile ROM read
--   S5 p_color void / entity / tile-fg / tile-bg composite
-- 6 stages -> C_SYNCD = 4.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.oubliette_tiles_pkg.all;
use work.oubliette_sprites_pkg.all;
use work.oubliette_font_pkg.all;
use work.oubliette_tables_pkg.all;

architecture oubliette of program_top is

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    constant C_MAP_LOG2  : integer := 6;
    constant C_MAP       : integer := 2 ** C_MAP_LOG2;            -- 64
    -- On-screen cell size = 2**C_CELL_LOG2 px.  The art is 16x16 and is scaled
    -- up nearest-neighbour to fill the cell (pixelated): cell 32 px -> 2x.
    -- Power-of-two so screen<->cell stays a bit-slice (no dividers).
    constant C_CELL_LOG2 : integer := 6;                          -- 64 px (4x)
    constant C_ART_LOG2  : integer := 4;                          -- 16 px art
    constant C_MONS      : integer := 6;                          -- monster slots
    constant C_MON_BASE  : integer := C_S_RAT;                    -- type t -> sprite base+t

    constant C_SYNCD : integer := 5;

    -- which layer wins a pixel (decided in the select stage, coloured in lookup)
    type t_src is (SRC_HUDINK, SRC_HUDBAR, SRC_VOID, SRC_ENT, SRC_TGT, SRC_TILEFG, SRC_TILEBG);

    constant C_SPAWN_X : integer := 16;
    constant C_SPAWN_Y : integer := 16;

    constant C_VOID_Y : unsigned(9 downto 0) := to_unsigned(20, 10);
    constant C_VOID_U : unsigned(9 downto 0) := C_MID;
    constant C_VOID_V : unsigned(9 downto 0) := C_MID;

    -- facing reticle (outline of the tile the player would step into): bright yellow
    constant C_TGT_Y : unsigned(9 downto 0) := to_unsigned(842, 10);
    constant C_TGT_U : unsigned(9 downto 0) := to_unsigned(598, 10);
    constant C_TGT_V : unsigned(9 downto 0) := to_unsigned(218, 10);

    -- HUD status bar: one row of glyphs (8x8 font scaled to 2**C_HUD_LOG2 px)
    -- along the bottom (and a context line at the top).
    constant C_HUD_LOG2 : integer := 5;         -- 32 px glyph cell (8x8 font at 4x)
    constant C_HUDN     : integer := 28;        -- status columns
    constant C_HUD_H    : integer := 2 ** C_HUD_LOG2;  -- bar height (px)
    constant C_HUD_MARG : integer := 4;         -- top/bottom margin (px)
    -- glyph indices into the forked titler font (charset " A..Z0..9.,!?-+:'/()<>=*")
    constant G_SP : integer := 0;
    constant G_H  : integer := 8;   constant G_P : integer := 16;
    constant G_G  : integer := 7;   constant G_L : integer := 12;
    constant G_V  : integer := 22;  constant G_D : integer := 4;
    constant G_SLASH : integer := 45;
    constant G_D0 : integer := 27;  -- '0'; digit d -> G_D0 + d
    -- HUD colours
    constant C_INK_Y : unsigned(9 downto 0) := to_unsigned(990, 10);
    constant C_INK_U : unsigned(9 downto 0) := C_MID;
    constant C_INK_V : unsigned(9 downto 0) := C_MID;
    constant C_BAR_Y : unsigned(9 downto 0) := to_unsigned(40, 10);
    constant C_BAR_U : unsigned(9 downto 0) := C_MID;
    constant C_BAR_V : unsigned(9 downto 0) := C_MID;

    -- Extra glyphs for the top context line.
    constant G_A : integer := 1;   constant G_B : integer := 2;
    constant G_C : integer := 3;   constant G_E : integer := 5;
    constant G_F : integer := 6;   constant G_I : integer := 9;
    constant G_K : integer := 11;  constant G_M : integer := 13;
    constant G_N : integer := 14;  constant G_O : integer := 15;
    constant G_R : integer := 18;  constant G_S : integer := 19;
    constant G_T : integer := 20;  constant G_U : integer := 21;
    constant G_W : integer := 23;  constant G_Y : integer := 25;
    constant G_EQ : integer := 50;

    --------------------------------------------------------------------------
    -- Combat stat tables (small -> kept as constants; all combat math is
    -- adds / compares / saturating subtract + LFSR variance, no multiply).
    --------------------------------------------------------------------------
    type t_cls is array (0 to 3) of integer;
    constant C_CLS_HP  : t_cls := (24, 16, 18, 20);   -- War Mag Rog Clr
    constant C_CLS_ATK : t_cls := (6, 4, 5, 4);
    constant C_CLS_DEF : t_cls := (4, 2, 3, 3);

    type t_mon is array (0 to 5) of integer;
    constant C_MON_HP  : t_mon := (4, 8, 6, 5, 12, 10); -- rat skel slime bat orc wraith
    constant C_MON_ATK : t_mon := (2, 4, 3, 3, 5, 6);
    constant C_MON_DEF : t_mon := (1, 2, 1, 1, 3, 2);
    constant C_MON_XP  : t_mon := (2, 5, 4, 3, 8, 10);

    -- 3-letter names (glyph indices) for the top context line.
    type t_n3 is array (0 to 2) of integer;
    type t_n3x4 is array (0 to 3) of t_n3;
    type t_n3x6 is array (0 to 5) of t_n3;
    constant C_CLSNAME : t_n3x4 :=
        ((G_W, G_A, G_R), (G_M, G_A, G_G), (G_R, G_O, G_G), (G_C, G_L, G_R));
    constant C_MONNAME : t_n3x6 :=
        ((G_R, G_A, G_T), (G_S, G_K, G_E), (G_S, G_L, G_M),
         (G_B, G_A, G_T), (G_O, G_R, G_C), (G_W, G_R, G_A));
    type t_n3x5 is array (0 to 4) of t_n3;
    constant C_ACTNAME : t_n3x5 :=    -- Attack Defend Cast Item Flee
        ((G_A, G_T, G_K), (G_D, G_E, G_F), (G_C, G_S, G_T),
         (G_I, G_T, G_M), (G_F, G_L, G_E));

    --------------------------------------------------------------------------
    -- Timing / resolution
    --------------------------------------------------------------------------
    signal s_timing  : t_video_timing_port;
    signal s_measured_h : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_measured_v : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_firstline : std_logic := '1';
    signal s_vsync_prev  : std_logic := '1';
    signal s_vsync_pulse : std_logic := '0';
    signal s_x : unsigned(11 downto 0) := (others => '0');
    signal s_y : unsigned(11 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Player / camera state
    --------------------------------------------------------------------------
    signal s_px : unsigned(5 downto 0) := to_unsigned(C_SPAWN_X, 6);
    signal s_py : unsigned(5 downto 0) := to_unsigned(C_SPAWN_Y, 6);
    signal s_class : unsigned(1 downto 0) := (others => '0');
    signal s_dir   : unsigned(2 downto 0) := (others => '0');  -- 0=N 1=NE 2=E 3=SE 4=S 5=SW 6=W 7=NW
    signal s_scroll_x : signed(13 downto 0) := (others => '0');
    signal s_scroll_y : signed(13 downto 0) := (others => '0');
    signal s_sw_step : std_logic := '0';

    -- facing reticle: the cell the player would step into (live from heading)
    signal s_tgt_x : unsigned(5 downto 0) := (others => '0');
    signal s_tgt_y : unsigned(5 downto 0) := (others => '0');
    signal s_show_tgt : std_logic := '0';

    -- Player stats (HUD-visible; combat mutates these from M3c).
    signal s_hp    : unsigned(7 downto 0) := to_unsigned(20, 8);
    signal s_maxhp : unsigned(7 downto 0) := to_unsigned(20, 8);
    signal s_gold  : unsigned(11 downto 0) := (others => '0');
    signal s_level : unsigned(4 downto 0) := to_unsigned(1, 5);
    signal s_depth : unsigned(2 downto 0) := to_unsigned(1, 3);

    -- HUD geometry latched per frame (bottom-aligned to measured resolution).
    signal s_hud_y0 : unsigned(11 downto 0) := to_unsigned(1080 - C_HUD_H - C_HUD_MARG, 12);

    --------------------------------------------------------------------------
    -- Master game state + combat
    --------------------------------------------------------------------------
    type t_game is (G_CREATE, G_GEN, G_EXPLORE, G_COMBAT, G_GAMEOVER);
    signal s_game : t_game := G_CREATE;

    -- Procedural dungeon generator (runs into the tilemap BRAM behind a splash).
    constant C_NROOM : integer := 8;
    type t_gen is (GN_CLEAR, GN_RPICK, GN_RCARVE, GN_CORR, GN_CORRH, GN_CORRV,
                   GN_PLACE, GN_MON, GN_DONE);
    signal s_gst   : t_gen := GN_DONE;
    signal s_gensp : std_logic := '0';                       -- splash active
    signal s_glfsr : std_logic_vector(23 downto 0) := x"ACE159";
    signal s_gaddr : unsigned(11 downto 0) := (others => '0');
    type t_rc is array (0 to C_NROOM - 1) of unsigned(5 downto 0);
    signal s_rcx, s_rcy : t_rc := (others => (others => '0'));
    signal s_gi  : integer range 0 to C_NROOM := 0;          -- room/work index
    signal s_rx0, s_ry0, s_rx1, s_ry1 : unsigned(5 downto 0) := (others => '0');
    signal s_cx, s_cy : unsigned(5 downto 0) := (others => '0');  -- carve cursor
    signal s_ca, s_cb : unsigned(5 downto 0) := (others => '0');  -- corridor span
    signal s_rfloor : std_logic_vector(3 downto 0) := (others => '0');  -- room floor type

    -- peaceful mode (S10): monsters don't chase or attack
    signal s_sw_peace : std_logic := '0';

    -- transient pickup message ("GOT SWORD" / "GOT GOLD")
    type t_msg is (MSG_NONE, MSG_SWORD, MSG_GOLD);
    signal s_msg : t_msg := MSG_NONE;
    signal s_msgtmr : unsigned(6 downto 0) := (others => '0');   -- frames remaining

    signal s_ctarget   : integer range 0 to C_MONS - 1 := 0;  -- monster in combat
    signal s_cb_action : integer range 0 to 4 := 0;           -- 0 Atk 1 Def 2 Cast 3 Item 4 Flee
    signal s_defending : std_logic := '0';
    signal s_clfsr     : std_logic_vector(15 downto 0) := x"1234";

    --------------------------------------------------------------------------
    -- Monster table (flip-flops so the render layer can test all 16 in parallel).
    -- M3a: statically populated; AI + combat arrive in M3c.
    --------------------------------------------------------------------------
    type t_m6 is array (0 to C_MONS - 1) of unsigned(5 downto 0);
    type t_m3 is array (0 to C_MONS - 1) of unsigned(2 downto 0);
    type t_m8 is array (0 to C_MONS - 1) of unsigned(7 downto 0);

    signal s_mx : t_m6 := (0 => to_unsigned(18, 6), 1 => to_unsigned(14, 6),
                           2 => to_unsigned(20, 6), 3 => to_unsigned(16, 6),
                           others => (others => '0'));
    signal s_my : t_m6 := (0 => to_unsigned(16, 6), 1 => to_unsigned(18, 6),
                           2 => to_unsigned(20, 6), 3 => to_unsigned(12, 6),
                           others => (others => '0'));
    signal s_mtype : t_m3 := (0 => "000", 1 => "001", 2 => "010", 3 => "100",
                              others => "000");
    signal s_mhp : t_m8 := (others => to_unsigned(10, 8));
    signal s_mactive : std_logic_vector(0 to C_MONS - 1) :=
        (0 => '1', 1 => '1', 2 => '1', 3 => '1', others => '0');

    --------------------------------------------------------------------------
    -- Dungeon tilemap BRAM.  4 bits/cell (tile id in bits 2:0; bit 3 reserved
    -- for a fog/seen flag) - half the block-RAM footprint of an 8-bit cell,
    -- which relieves routing congestion on the HX4K.
    --------------------------------------------------------------------------
    type t_map is array (0 to C_MAP * C_MAP - 1) of std_logic_vector(3 downto 0);

    function build_map return t_map is
        variable m : t_map := (others => (others => '0'));
        variable id : integer;
    begin
        for r in 0 to C_MAP - 1 loop
            for c in 0 to C_MAP - 1 loop
                if r = 0 or r = C_MAP - 1 or c = 0 or c = C_MAP - 1 then
                    id := C_T_WALL;
                elsif (r = 32) and (c > 8) and (c < 55) and (c /= 32) then
                    id := C_T_WALL;
                elsif (c = 32) and (r > 8) and (r < 55)
                      and (r /= 16) and (r /= 32) and (r /= 48) then
                    id := C_T_WALL;
                elsif (r >= 4 and r <= 9) and (c >= 4 and c <= 11) then
                    id := C_T_WATER;
                elsif (r = 20 and c = 44) then
                    id := C_T_STAIRS;
                elsif (r >= 48 and r <= 52) and (c >= 10 and c <= 16) then
                    id := C_T_RUBBLE;
                else
                    id := C_T_FLOOR;
                end if;
                m(r * C_MAP + c) := std_logic_vector(to_unsigned(id, 4));
            end loop;
        end loop;
        return m;
    end function;

    signal s_map : t_map := build_map;
    attribute ram_style : string;
    attribute ram_style of s_map : signal is "block";

    signal s_g_raddr : unsigned(2 * C_MAP_LOG2 - 1 downto 0) := (others => '0');
    -- shared registered map read output (render during active video, game in vblank)
    signal s_map_q   : std_logic_vector(3 downto 0) := (others => '0');
    -- generator write port (driven by p_game, applied in p_gmap)
    signal s_mw_we   : std_logic := '0';
    signal s_mw_addr : unsigned(2 * C_MAP_LOG2 - 1 downto 0) := (others => '0');
    signal s_mw_data : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Tile ROM + Sprite ROM (addr = id*16 + row).
    --------------------------------------------------------------------------
    constant C_TROM_DEPTH : integer := C_NTILES * 16;
    type t_trom is array (0 to C_TROM_DEPTH - 1) of std_logic_vector(15 downto 0);
    function build_trom return t_trom is
        variable r : t_trom;
    begin
        for t in 0 to C_NTILES - 1 loop
            for y in 0 to 15 loop r(t * 16 + y) := C_TILES(t, y); end loop;
        end loop;
        return r;
    end function;
    signal s_trom : t_trom := build_trom;
    attribute ram_style of s_trom : signal is "block";

    constant C_SPROM_DEPTH : integer := C_NSPR * 16;
    type t_sprom is array (0 to C_SPROM_DEPTH - 1) of std_logic_vector(15 downto 0);
    function build_sprom return t_sprom is
        variable r : t_sprom;
    begin
        for s in 0 to C_NSPR - 1 loop
            for y in 0 to 15 loop r(s * 16 + y) := C_SPRITES(s, y); end loop;
        end loop;
        return r;
    end function;
    signal s_sprom : t_sprom := build_sprom;
    attribute ram_style of s_sprom : signal is "block";

    --------------------------------------------------------------------------
    -- Font ROM (8x8, addr = glyph*8 + row) + BCD digit ROM (value -> 3 digits).
    --------------------------------------------------------------------------
    constant C_FROM_DEPTH : integer := C_FONT_CHAR_COUNT * 8;
    type t_from is array (0 to C_FROM_DEPTH - 1) of std_logic_vector(7 downto 0);
    function build_from return t_from is
        variable r : t_from;
    begin
        for g in 0 to C_FONT_CHAR_COUNT - 1 loop
            for y in 0 to 7 loop r(g * 8 + y) := C_FONT_ROM(g, y); end loop;
        end loop;
        return r;
    end function;
    signal s_from : t_from := build_from;
    attribute ram_style of s_from : signal is "block";

    signal s_bcd : t_bcd := C_BCD;
    attribute ram_style of s_bcd : signal is "block";
    signal s_bcd_addr : unsigned(9 downto 0) := (others => '0');
    signal s_bcd_q    : std_logic_vector(11 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- HUD character RAM (block RAM, 128 x 7 - address bit 6 selects the top
    -- context line, bits 5:0 the column).  Replacing the old flip-flop char
    -- arrays kills two 28-way 7-bit read muxes (a recurring critical path)
    -- and their write decoders.  The update FSM rewrites every cell - labels
    -- included - once per frame during vblank, one cell per cycle.
    --------------------------------------------------------------------------
    type t_hudram is array (0 to 127) of std_logic_vector(6 downto 0);
    signal s_hudram : t_hudram := (others => (others => '0'));
    attribute ram_style of s_hudram : signal is "block";
    signal s_hr_we    : std_logic := '0';
    signal s_hr_waddr : unsigned(6 downto 0) := (others => '0');
    signal s_hr_wdata : std_logic_vector(6 downto 0) := (others => '0');

    -- HUD update FSM: latch every displayed number's digits first (BCD ROM
    -- reads), then stream the bottom bar and top line into the char RAM.
    type t_hu is (HU_IDLE, HU_REQ, HU_WAIT, HU_LATCH, HU_BOT, HU_TOP);
    signal s_hu_state : t_hu := HU_IDLE;
    signal s_hu_field : integer range 0 to 5 := 0;
    signal s_hu_col   : integer range 0 to C_HUDN - 1 := 0;
    -- digit registers: 0,1=hp 2,3=maxhp 4,5,6=gold 7,8=level 9=depth 10,11=enemy hp
    type t_dig is array (0 to 11) of integer range 0 to 9;
    signal s_dig : t_dig := (others => 0);

    --------------------------------------------------------------------------
    -- Turn FSM (player move + greedy monster chase) + combat sub-FSM.
    --------------------------------------------------------------------------
    -- Turn FSM is split into short single-purpose states so no one cycle has a
    -- long combinational path (each runs in vblank with whole-frame slack).
    type t_turn is (TS_IDLE, TS_PMON, TS_PSW, TS_PSCAN, TS_PWAIT, TS_PMOVE,
                    TS_MREQ, TS_MCALC, TS_MSTEP, TS_MBUMP, TS_MWAIT, TS_MAPPLY);
    signal s_ts : t_turn := TS_IDLE;
    signal s_mi : integer range 0 to C_MONS - 1 := 0;
    signal s_tx : unsigned(5 downto 0) := (others => '0');
    signal s_ty : unsigned(5 downto 0) := (others => '0');
    signal s_mocc : std_logic := '0';
    -- selected monster's position latched out of the 16:1 table mux (so the
    -- greedy-chase arithmetic doesn't share a cycle with the index mux)
    signal s_sel_mx : unsigned(5 downto 0) := (others => '0');
    signal s_sel_my : unsigned(5 downto 0) := (others => '0');
    signal s_sel_mt : unsigned(2 downto 0) := (others => '0');
    signal s_sel_mhp : unsigned(7 downto 0) := (others => '0');
    -- in-combat monster stats, latched into scalars at encounter start so combat
    -- math and the HUD never re-index the monster table by s_ctarget.
    signal s_cmt  : unsigned(2 downto 0) := (others => '0');
    signal s_cmhp : unsigned(7 downto 0) := (others => '0');
    -- combat stats pre-resolved to scalars (no stat-table lookups on the combat
    -- arithmetic path): player atk/def latched at CREATE, monster at encounter.
    signal s_patk, s_pdef : integer range 0 to 31 := 0;
    signal s_cmatk, s_cmdef : integer range 0 to 31 := 0;
    signal s_cmxp : integer range 0 to 63 := 0;
    -- registered chase deltas (abs + sign/nonzero) so the axis-select + step is
    -- a separate short cycle from the subtract/abs.
    signal s_adx, s_ady : integer range 0 to 64 := 0;
    signal s_ddxp, s_ddxn, s_ddyp, s_ddyn : std_logic := '0';
    -- continuously registered "monster j stands on (s_tx,s_ty)" match vector -
    -- the FSM scan states just OR these bits instead of doing 6 parallel 12-bit
    -- coordinate compares deep inside the state-decode tree (was ~13 levels).
    signal s_mmatch : std_logic_vector(0 to C_MONS - 1) := (others => '0');
    -- per-slot spawn-enable mask, precomputed at generation start so GN_MON
    -- does no depth arithmetic.
    signal s_mon_en : std_logic_vector(0 to C_MONS - 1) := (others => '0');
    signal s_step_prev : std_logic := '0';
    signal s_new_prev  : std_logic := '0';
    signal s_turn_pending : std_logic := '0';
    signal s_sw_new : std_logic := '0';

    -- combat resolves one arithmetic op per cycle (turn-based: latency is free,
    -- and every cycle's combinational path stays short for full-clock timing).
    type t_cs is (CS_INIT, CS_INPUT, CS_PLAYER, CS_PHIT, CS_POUT, CS_LVL,
                  CS_MONSTER, CS_MHIT);
    signal s_cs : t_cs := CS_INPUT;
    signal s_cb_act_l : integer range 0 to 4 := 0;
    signal s_dmg, s_mdmg : integer range 0 to 63 := 0;
    signal s_mon_dead : std_logic := '0';
    -- experience: kills award XP; level-up at level*8 XP (shift, no multiply)
    signal s_xp : unsigned(7 downto 0) := (others => '0');
    -- context name glyphs pre-resolved to registers so the HUD never does a
    -- name-table lookup on the (timing-critical) HUD write path.
    type t_g3 is array (0 to 2) of integer range 0 to 63;
    signal s_actn, s_clsn, s_monn : t_g3 := (others => 0);

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    -- S0
    signal s0_wx, s0_wy : signed(13 downto 0) := (others => '0');
    signal s0_render : std_logic := '0';
    -- S1
    signal s1_addr : unsigned(2 * C_MAP_LOG2 - 1 downto 0) := (others => '0');
    signal s1_gx, s1_gy : unsigned(3 downto 0) := (others => '0');
    signal s1_inb, s1_inplayer, s1_intgt, s1_render : std_logic := '0';
    -- S2
    signal s2_ehit : std_logic := '0';
    signal s2_eid  : integer range 0 to C_NSPR - 1 := 0;
    signal s2_gx, s2_gy : unsigned(3 downto 0) := (others => '0');
    signal s2_inb, s2_intgt, s2_render : std_logic := '0';
    -- S3
    signal s3_sprrow : std_logic_vector(15 downto 0) := (others => '0');
    signal s3_ehit : std_logic := '0';
    signal s3_eid  : integer range 0 to C_NSPR - 1 := 0;
    signal s3_tile : integer range 0 to C_NTILES - 1 := 0;
    signal s3_gx, s3_gy : unsigned(3 downto 0) := (others => '0');
    signal s3_inb, s3_intgt, s3_render : std_logic := '0';
    -- S4
    signal s4_row : std_logic_vector(15 downto 0) := (others => '0');
    signal s4_sprrow : std_logic_vector(15 downto 0) := (others => '0');
    signal s4_ehit : std_logic := '0';
    signal s4_eid  : integer range 0 to C_NSPR - 1 := 0;
    signal s4_tile : integer range 0 to C_NTILES - 1 := 0;
    signal s4_gx, s4_gy : unsigned(3 downto 0) := (others => '0');
    signal s4_inb, s4_intgt, s4_render : std_logic := '0';
    -- S5 (select: layer decided, palette index carried)
    signal s5_src  : t_src := SRC_VOID;
    signal s5_eid  : integer range 0 to C_NSPR - 1 := 0;
    signal s5_tile : integer range 0 to C_NTILES - 1 := 0;
    signal s5_sync : std_logic_vector(3 downto 0) := (others => '0');
    -- S6 (palette lookup -> YUV)
    signal s6_y, s6_u, s6_v : unsigned(9 downto 0) := (others => '0');
    signal s6_sync : std_logic_vector(3 downto 0) := (others => '0');

    -- HUD pipeline (screen-space, parallel to the world pipeline; same depth).
    signal s0_hud_active : std_logic := '0';
    signal s0_hud_top : std_logic := '0';
    signal s0_hud_col : unsigned(5 downto 0) := (others => '0');
    signal s0_frow, s0_fcol : unsigned(2 downto 0) := (others => '0');
    signal s1_glyph : unsigned(6 downto 0) := (others => '0');
    signal s1_hud_active : std_logic := '0';
    signal s1_frow, s1_fcol : unsigned(2 downto 0) := (others => '0');
    signal s2_fontrow : std_logic_vector(7 downto 0) := (others => '0');
    signal s2_hud_active : std_logic := '0';
    signal s2_fcol : unsigned(2 downto 0) := (others => '0');
    signal s3_fontrow : std_logic_vector(7 downto 0) := (others => '0');
    signal s3_hud_active : std_logic := '0';
    signal s3_fcol : unsigned(2 downto 0) := (others => '0');
    signal s4_fontrow : std_logic_vector(7 downto 0) := (others => '0');
    signal s4_hud_active : std_logic := '0';
    signal s4_fcol : unsigned(2 downto 0) := (others => '0');

    type t_syncp is array (0 to C_SYNCD) of std_logic_vector(3 downto 0);
    signal s_syncp : t_syncp := (others => (others => '0'));

    signal s_io : t_video_stream_yuv444_30b;

    function walkable(id : std_logic_vector(3 downto 0)) return boolean is
        variable t : integer;
    begin
        t := to_integer(unsigned(id));
        return (t /= C_T_WALL) and (t /= C_T_WATER) and (t /= C_T_WALL2)
               and (t /= C_T_PILLAR) and (t /= C_T_LAVA);
    end function;

    -- 24-bit Fibonacci LFSR step (taps 24,23,22,17) for dungeon generation.
    function lf24(x : std_logic_vector(23 downto 0)) return std_logic_vector is
        variable fb : std_logic;
    begin
        fb := x(23) xor x(22) xor x(21) xor x(16);
        return x(22 downto 0) & fb;
    end function;

begin

    s_sw_step  <= registers_in(6)(0);
    s_sw_peace <= registers_in(6)(3);   -- S10: monsters passive when on
    s_sw_new   <= registers_in(6)(4);

    --------------------------------------------------------------------------
    -- Timing infrastructure (genlocked to the incoming sync).
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    p_measure_resolution : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then s_measured_h <= s_h_pixel_counter; end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then s_measured_v <= s_v_line_counter; end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    p_firstline : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_firstline <= '1';
            elsif s_timing.avid_start = '1' then
                s_firstline <= '0';
            end if;
        end if;
    end process;

    p_vsync_edge : process(clk)
    begin
        if rising_edge(clk) then
            s_vsync_prev <= s_timing.vsync_n;
            if s_vsync_prev = '0' and s_timing.vsync_n = '1' then
                s_vsync_pulse <= '1';
            else
                s_vsync_pulse <= '0';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame latch: camera scroll (player-centred) + class + heading.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_sx, v_sy : integer;
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                -- centre the player cell: scroll = px*cell + cell/2 - screen/2.
                v_sx := to_integer(s_px) * (2 ** C_CELL_LOG2) + (2 ** (C_CELL_LOG2 - 1))
                        - to_integer(s_measured_h) / 2;
                v_sy := to_integer(s_py) * (2 ** C_CELL_LOG2) + (2 ** (C_CELL_LOG2 - 1))
                        - to_integer(s_measured_v) / 2;
                s_scroll_x <= to_signed(v_sx, 14);
                s_scroll_y <= to_signed(v_sy, 14);
                s_hud_y0   <= s_measured_v - (C_HUD_H + C_HUD_MARG);
            end if;
            s_dir <= unsigned(registers_in(7)(9 downto 7));   -- slider -> 8 headings
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Movement FSM: Switch-7 edge -> step one cell in the heading direction if
    -- the target tile is walkable.
    --------------------------------------------------------------------------
    -- Single-port tilemap controller: 1 write (generator) + 1 read whose address
    -- is time-multiplexed - the render pipeline reads during active video, the
    -- game logic reads during vertical blank.  This keeps the 32 Kbit map in ONE
    -- block-RAM copy (no duplication), which is essential for routing/timing.
    p_mapport : process(clk)
        variable v_raddr : unsigned(2 * C_MAP_LOG2 - 1 downto 0);
    begin
        if rising_edge(clk) then
            if s_mw_we = '1' then
                s_map(to_integer(s_mw_addr)) <= s_mw_data;
            end if;
            if s_timing.avid = '1' then v_raddr := s1_addr;      -- render read
            else                        v_raddr := s_g_raddr;    -- game-logic read
            end if;
            s_map_q <= s_map(to_integer(v_raddr));
        end if;
    end process;

    -- Free-running combat RNG (16-bit Fibonacci LFSR).
    p_clfsr : process(clk)
        variable v_fb : std_logic;
    begin
        if rising_edge(clk) then
            v_fb := s_clfsr(15) xor s_clfsr(13) xor s_clfsr(12) xor s_clfsr(10);
            s_clfsr <= s_clfsr(14 downto 0) & v_fb;
        end if;
    end process;

    -- Continuous monster-at-target matcher: registered every cycle, consumed
    -- by the turn FSM one state later (a wait state guarantees freshness).
    p_mmatch : process(clk)
    begin
        if rising_edge(clk) then
            for j in 0 to C_MONS - 1 loop
                if s_mactive(j) = '1' and s_mx(j) = s_tx and s_my(j) = s_ty then
                    s_mmatch(j) <= '1';
                else
                    s_mmatch(j) <= '0';
                end if;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Master game FSM: CREATE -> EXPLORE <-> COMBAT, EXPLORE/COMBAT -> GAMEOVER.
    -- A turn (player move + greedy monster chase) runs serially during vblank so
    -- the monster table never changes while the render layer is reading it.
    --------------------------------------------------------------------------
    p_game : process(clk)
        -- All ranges constrained so synthesis uses narrow (not 32-bit) arithmetic.
        variable v_edge, v_newedge : std_logic;
        variable v_tx, v_ty : integer range 0 to 63;
        variable mxi, myi, pxi, pyi : integer range 0 to 63;
        variable ddx, ddy : integer range -64 to 64;
        variable adx, ady : integer range 0 to 64;
        variable v_occ : std_logic;
        variable cls : integer range 0 to 3;
        variable mt  : integer range 0 to 5;
        variable v_xp  : integer range 0 to 511;
        variable v_thr : integer range 0 to 255;
        variable monhp : integer range -64 to 255;
        variable dmg, mdmg : integer range -64 to 63;
        variable php, newhp : integer range -64 to 255;
        variable var, var2 : integer range 0 to 3;
        -- generator scratch (constrained widths)
        variable v_rw, v_rh : integer range 0 to 15;
        variable v_rx0, v_ry0 : integer range 0 to 63;
        variable v_type : integer range 0 to 7;
        variable v_dest : integer range 0 to 15;
        variable v_ay, v_by, v_lo, v_hi : integer range 0 to 63;

        -- begin generating a depth-d dungeon level (seeded by K2 + depth)
        procedure start_gen(d : integer) is
        begin
            s_depth <= to_unsigned(d, 3);
            s_glfsr <= x"A5" & std_logic_vector(to_unsigned(d, 6)) & registers_in(1);
            s_gaddr <= (others => '0');
            s_gi    <= 0;
            s_gst   <= GN_CLEAR;
            s_gensp <= '1';
            s_game  <= G_GEN;
            -- spawn-enable mask: slot i gets a monster if i < d+1 (and its room
            -- exists) - resolved here so GN_MON does no depth arithmetic.
            for i in 0 to C_MONS - 1 loop
                if i < d + 1 and i + 3 < C_NROOM then
                    s_mon_en(i) <= '1';
                else
                    s_mon_en(i) <= '0';
                end if;
            end loop;
        end procedure;
    begin
        if rising_edge(clk) then
            s_step_prev <= s_sw_step;
            s_new_prev  <= s_sw_new;
            v_edge    := '0';
            v_newedge := '0';
            if s_sw_step /= s_step_prev then v_edge := '1'; end if;
            if s_sw_new  /= s_new_prev  then v_newedge := '1'; end if;
            s_mw_we <= '0';   -- pulse high only when a generator state writes

            -- live action preview in combat (K3 -> 0..4 via compares, no divide)
            if unsigned(registers_in(2)) < 205 then s_cb_action <= 0;
            elsif unsigned(registers_in(2)) < 410 then s_cb_action <= 1;
            elsif unsigned(registers_in(2)) < 614 then s_cb_action <= 2;
            elsif unsigned(registers_in(2)) < 819 then s_cb_action <= 3;
            else s_cb_action <= 4; end if;

            -- pre-resolve context name glyphs to registers (off the HUD path)
            for k in 0 to 2 loop
                s_actn(k) <= C_ACTNAME(s_cb_action)(k);
                s_clsn(k) <= C_CLSNAME(to_integer(s_class))(k);
                s_monn(k) <= C_MONNAME(to_integer(s_cmt))(k);
            end loop;

            -- facing reticle target cell (live from heading), shown in EXPLORE
            v_tx := to_integer(s_px); v_ty := to_integer(s_py);
            case to_integer(s_dir) is
                when 0 => v_ty := v_ty - 1;                       -- N
                when 1 => v_tx := v_tx + 1; v_ty := v_ty - 1;     -- NE
                when 2 => v_tx := v_tx + 1;                       -- E
                when 3 => v_tx := v_tx + 1; v_ty := v_ty + 1;     -- SE
                when 4 => v_ty := v_ty + 1;                       -- S
                when 5 => v_tx := v_tx - 1; v_ty := v_ty + 1;     -- SW
                when 6 => v_tx := v_tx - 1;                       -- W
                when others => v_tx := v_tx - 1; v_ty := v_ty - 1; -- NW
            end case;
            if v_tx < 0 then v_tx := 0; elsif v_tx > C_MAP - 1 then v_tx := C_MAP - 1; end if;
            if v_ty < 0 then v_ty := 0; elsif v_ty > C_MAP - 1 then v_ty := C_MAP - 1; end if;
            s_tgt_x <= to_unsigned(v_tx, 6);
            s_tgt_y <= to_unsigned(v_ty, 6);
            if s_game = G_EXPLORE then s_show_tgt <= '1'; else s_show_tgt <= '0'; end if;

            -- pickup-message countdown (once per frame)
            if s_vsync_pulse = '1' then
                if s_msgtmr > 0 then s_msgtmr <= s_msgtmr - 1;
                else s_msg <= MSG_NONE; end if;
            end if;

            -- global new-game
            if v_newedge = '1' then
                s_game <= G_CREATE;
                s_ts <= TS_IDLE;
                s_turn_pending <= '0';
            else
                case s_game is

                    ----------------------------------------------------------
                    when G_CREATE =>
                        s_class <= unsigned(registers_in(0)(9 downto 8));  -- live preview
                        if v_edge = '1' then
                            cls := to_integer(unsigned(registers_in(0)(9 downto 8)));
                            s_class <= to_unsigned(cls, 2);
                            s_hp    <= to_unsigned(C_CLS_HP(cls), 8);
                            s_maxhp <= to_unsigned(C_CLS_HP(cls), 8);
                            s_patk  <= C_CLS_ATK(cls);
                            s_pdef  <= C_CLS_DEF(cls);
                            s_gold  <= (others => '0');
                            s_level <= to_unsigned(1, 5);
                            s_xp    <= (others => '0');
                            s_ts <= TS_IDLE;
                            start_gen(1);          -- generate dungeon level 1
                        end if;

                    ----------------------------------------------------------
                    -- Procedural dungeon generation (runs behind the splash;
                    -- one BRAM write per cycle, completes in a fraction of a frame).
                    when G_GEN =>
                        s_glfsr <= lf24(s_glfsr);
                        case s_gst is
                            when GN_CLEAR =>
                                s_mw_addr <= s_gaddr;
                                s_mw_data <= std_logic_vector(to_unsigned(C_T_WALL, 4));
                                s_mw_we   <= '1';
                                if s_gaddr = C_MAP * C_MAP - 1 then
                                    s_gaddr <= (others => '0');
                                    s_gi <= 0;
                                    s_gst <= GN_RPICK;
                                else
                                    s_gaddr <= s_gaddr + 1;
                                end if;
                            when GN_RPICK =>
                                v_rw := 5 + to_integer(unsigned(s_glfsr(2 downto 0)));   -- 5..12
                                v_rh := 5 + to_integer(unsigned(s_glfsr(5 downto 3)));
                                v_rx0 := 2 + to_integer(unsigned(s_glfsr(13 downto 8)));  -- 2..65
                                v_ry0 := 2 + to_integer(unsigned(s_glfsr(21 downto 16)));
                                if v_rx0 + v_rw > C_MAP - 2 then v_rx0 := C_MAP - 2 - v_rw; end if;
                                if v_ry0 + v_rh > C_MAP - 2 then v_ry0 := C_MAP - 2 - v_rh; end if;
                                s_rx1 <= to_unsigned(v_rx0 + v_rw - 1, 6);
                                s_ry1 <= to_unsigned(v_ry0 + v_rh - 1, 6);
                                s_rx0 <= to_unsigned(v_rx0, 6);
                                s_ry0 <= to_unsigned(v_ry0, 6);
                                s_cx  <= to_unsigned(v_rx0, 6);
                                s_cy  <= to_unsigned(v_ry0, 6);
                                s_rcx(s_gi) <= to_unsigned(v_rx0 + v_rw / 2, 6);
                                s_rcy(s_gi) <= to_unsigned(v_ry0 + v_rh / 2, 6);
                                -- pick this room's floor terrain from the LFSR
                                case to_integer(unsigned(s_glfsr(23 downto 21))) is
                                    when 2 => s_rfloor <= std_logic_vector(to_unsigned(C_T_GRASS, 4));
                                    when 3 => s_rfloor <= std_logic_vector(to_unsigned(C_T_SAND, 4));
                                    when 4 => s_rfloor <= std_logic_vector(to_unsigned(C_T_MOSS, 4));
                                    when 5 => s_rfloor <= std_logic_vector(to_unsigned(C_T_CRACKED, 4));
                                    when 6 => s_rfloor <= std_logic_vector(to_unsigned(C_T_BONES, 4));
                                    when others => s_rfloor <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                end case;
                                s_gst <= GN_RCARVE;
                            when GN_RCARVE =>
                                -- room interior = floor terrain; perimeter = outline rock
                                s_mw_addr <= s_cy & s_cx;
                                if s_cx = s_rx0 or s_cx = s_rx1 or s_cy = s_ry0 or s_cy = s_ry1 then
                                    s_mw_data <= std_logic_vector(to_unsigned(C_T_WALL2, 4));
                                else
                                    s_mw_data <= s_rfloor;
                                end if;
                                s_mw_we   <= '1';
                                if s_cx = s_rx1 then
                                    s_cx <= s_rx0;
                                    if s_cy = s_ry1 then
                                        if s_gi = C_NROOM - 1 then s_gi <= 1; s_gst <= GN_CORR;
                                        else s_gi <= s_gi + 1; s_gst <= GN_RPICK; end if;
                                    else
                                        s_cy <= s_cy + 1;
                                    end if;
                                else
                                    s_cx <= s_cx + 1;
                                end if;
                            when GN_CORR =>
                                -- horizontal run at room(s_gi-1)'s row, between centres
                                if s_rcx(s_gi - 1) <= s_rcx(s_gi) then
                                    s_ca <= s_rcx(s_gi - 1); s_cb <= s_rcx(s_gi);
                                    s_cx <= s_rcx(s_gi - 1);
                                else
                                    s_ca <= s_rcx(s_gi); s_cb <= s_rcx(s_gi - 1);
                                    s_cx <= s_rcx(s_gi);
                                end if;
                                s_cy <= s_rcy(s_gi - 1);
                                s_gst <= GN_CORRH;
                            when GN_CORRH =>
                                s_mw_addr <= s_cy & s_cx;
                                s_mw_data <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                s_mw_we   <= '1';
                                if s_cx = s_cb then
                                    -- switch to vertical run at room(s_gi)'s column
                                    v_ay := to_integer(s_rcy(s_gi - 1));
                                    v_by := to_integer(s_rcy(s_gi));
                                    if v_ay <= v_by then v_lo := v_ay; v_hi := v_by;
                                    else v_lo := v_by; v_hi := v_ay; end if;
                                    s_cx <= s_rcx(s_gi);
                                    s_cy <= to_unsigned(v_lo, 6);
                                    s_cb <= to_unsigned(v_hi, 6);
                                    s_gst <= GN_CORRV;
                                else
                                    s_cx <= s_cx + 1;
                                end if;
                            when GN_CORRV =>
                                s_mw_addr <= s_cy & s_cx;
                                s_mw_data <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                s_mw_we   <= '1';
                                if s_cy = s_cb then
                                    if s_gi = C_NROOM - 1 then s_gi <= 0; s_gst <= GN_PLACE;
                                    else s_gi <= s_gi + 1; s_gst <= GN_CORR; end if;
                                else
                                    s_cy <= s_cy + 1;
                                end if;
                            when GN_PLACE =>
                                -- room 0 spawn; room 1 weapon; room 2 chest; last room stairs
                                case s_gi is
                                    when 0 =>
                                        s_px <= s_rcx(0);
                                        s_py <= s_rcy(0);
                                        s_mw_addr <= s_rcy(C_NROOM - 1) & s_rcx(C_NROOM - 1);
                                        s_mw_data <= std_logic_vector(to_unsigned(C_T_STAIRS, 4));
                                        s_mw_we   <= '1';
                                        s_gi <= 1;
                                    when 1 =>
                                        s_mw_addr <= s_rcy(1) & s_rcx(1);
                                        s_mw_data <= std_logic_vector(to_unsigned(C_T_WEAPON, 4));
                                        s_mw_we   <= '1';
                                        s_gi <= 2;
                                    when others =>
                                        s_mw_addr <= s_rcy(2) & s_rcx(2);
                                        s_mw_data <= std_logic_vector(to_unsigned(C_T_CHEST, 4));
                                        s_mw_we   <= '1';
                                        s_gi <= 0;
                                        s_gst <= GN_MON;
                                end case;
                            when GN_MON =>
                                -- monsters in rooms 3..NROOM-1 (spawn/weapon/chest rooms
                                -- 0..2 stay clear), one per room; count via the mask
                                -- precomputed in start_gen (no depth arithmetic here).
                                v_type := to_integer(unsigned(s_glfsr(2 downto 0)));
                                if v_type > 5 then v_type := v_type - 6; end if;
                                v_lo := s_gi + 3;                 -- room index, clamped
                                if v_lo > C_NROOM - 1 then v_lo := C_NROOM - 1; end if;
                                if s_mon_en(s_gi) = '1' then
                                    s_mactive(s_gi) <= '1';
                                    s_mx(s_gi) <= s_rcx(v_lo);
                                    s_my(s_gi) <= s_rcy(v_lo);
                                    s_mtype(s_gi) <= to_unsigned(v_type, 3);
                                    s_mhp(s_gi) <= to_unsigned(C_MON_HP(v_type) + to_integer(s_depth), 8);
                                else
                                    s_mactive(s_gi) <= '0';
                                end if;
                                if s_gi = C_MONS - 1 then s_gst <= GN_DONE;
                                else s_gi <= s_gi + 1; end if;
                            when GN_DONE =>
                                s_gensp <= '0';
                                s_ts <= TS_IDLE;
                                s_game <= G_EXPLORE;
                        end case;

                    ----------------------------------------------------------
                    when G_EXPLORE =>
                        if v_edge = '1' and s_ts = TS_IDLE then
                            s_turn_pending <= '1';
                        end if;
                        case s_ts is
                            when TS_IDLE =>
                                if s_turn_pending = '1' and s_vsync_pulse = '1' then
                                    s_turn_pending <= '0';
                                    s_ts <= TS_PMON;
                                end if;
                            when TS_PMON =>
                                -- compute the player target cell (heading, clamped)
                                v_tx := to_integer(s_px); v_ty := to_integer(s_py);
                                case to_integer(s_dir) is
                                    when 0 => v_ty := v_ty - 1;                       -- N
                                    when 1 => v_tx := v_tx + 1; v_ty := v_ty - 1;     -- NE
                                    when 2 => v_tx := v_tx + 1;                       -- E
                                    when 3 => v_tx := v_tx + 1; v_ty := v_ty + 1;     -- SE
                                    when 4 => v_ty := v_ty + 1;                       -- S
                                    when 5 => v_tx := v_tx - 1; v_ty := v_ty + 1;     -- SW
                                    when 6 => v_tx := v_tx - 1;                       -- W
                                    when others => v_tx := v_tx - 1; v_ty := v_ty - 1; -- NW
                                end case;
                                if v_tx < 1 then v_tx := 1; elsif v_tx > C_MAP - 2 then v_tx := C_MAP - 2; end if;
                                if v_ty < 1 then v_ty := 1; elsif v_ty > C_MAP - 2 then v_ty := C_MAP - 2; end if;
                                s_tx <= to_unsigned(v_tx, 6);
                                s_ty <= to_unsigned(v_ty, 6);
                                s_g_raddr <= to_unsigned(v_ty, 6) & to_unsigned(v_tx, 6);
                                s_ts <= TS_PSW;
                            when TS_PSW =>
                                s_ts <= TS_PSCAN;   -- let p_mmatch see the new target
                            when TS_PSCAN =>
                                -- attack if a monster occupies the target cell
                                v_occ := '0';
                                for j in 0 to C_MONS - 1 loop
                                    if s_mmatch(j) = '1' then
                                        v_occ := '1';
                                        s_ctarget <= j;
                                        s_cmt  <= s_mtype(j);
                                        s_cmhp <= s_mhp(j);
                                    end if;
                                end loop;
                                if v_occ = '1' then
                                    s_game <= G_COMBAT;
                                    s_cs <= CS_INIT;
                                    s_ts <= TS_IDLE;
                                else
                                    s_ts <= TS_PWAIT;
                                end if;
                            when TS_PWAIT =>
                                s_ts <= TS_PMOVE;
                            when TS_PMOVE =>
                                if walkable(s_map_q) then
                                    s_px <= s_tx;
                                    s_py <= s_ty;
                                    v_dest := to_integer(unsigned(s_map_q));
                                    if v_dest = C_T_STAIRS then
                                        -- descend: regenerate a deeper level
                                        if to_integer(s_depth) < 7 then
                                            start_gen(to_integer(s_depth) + 1);
                                        else
                                            start_gen(7);
                                        end if;
                                    else
                                        if v_dest = C_T_WEAPON then
                                            if s_patk < 20 then s_patk <= s_patk + 3; end if;
                                            s_mw_addr <= s_ty & s_tx;   -- pick up: clear cell
                                            s_mw_data <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                            s_mw_we   <= '1';
                                            s_msg <= MSG_SWORD;
                                            s_msgtmr <= to_unsigned(100, 7);
                                        elsif v_dest = C_T_CHEST then
                                            s_gold <= s_gold
                                                + to_unsigned(10 + to_integer(unsigned(s_clfsr(4 downto 0))), 12);
                                            s_mw_addr <= s_ty & s_tx;
                                            s_mw_data <= std_logic_vector(to_unsigned(C_T_FLOOR, 4));
                                            s_mw_we   <= '1';
                                            s_msg <= MSG_GOLD;
                                            s_msgtmr <= to_unsigned(100, 7);
                                        end if;
                                        s_mi <= 0;
                                        s_ts <= TS_MREQ;
                                    end if;
                                else
                                    s_mi <= 0;
                                    s_ts <= TS_MREQ;
                                end if;
                            when TS_MREQ =>
                                -- isolate the 8:1 index mux: just latch this
                                -- monster's position (or skip if inactive).
                                if s_sw_peace = '1' then
                                    s_ts <= TS_IDLE;    -- peaceful: monsters don't act
                                elsif s_mactive(s_mi) = '0' then
                                    if s_mi = C_MONS - 1 then s_ts <= TS_IDLE;
                                    else s_mi <= s_mi + 1; end if;
                                else
                                    s_sel_mx <= s_mx(s_mi);
                                    s_sel_my <= s_my(s_mi);
                                    s_sel_mt <= s_mtype(s_mi);
                                    s_sel_mhp <= s_mhp(s_mi);
                                    s_ts <= TS_MCALC;
                                end if;
                            when TS_MCALC =>
                                -- deltas + abs + sign/nonzero (no compare/step yet)
                                mxi := to_integer(s_sel_mx);
                                myi := to_integer(s_sel_my);
                                pxi := to_integer(s_px); pyi := to_integer(s_py);
                                ddx := pxi - mxi; ddy := pyi - myi;
                                if ddx < 0 then s_adx <= -ddx; else s_adx <= ddx; end if;
                                if ddy < 0 then s_ady <= -ddy; else s_ady <= ddy; end if;
                                if ddx > 0 then s_ddxp <= '1'; else s_ddxp <= '0'; end if;
                                if ddx /= 0 then s_ddxn <= '1'; else s_ddxn <= '0'; end if;
                                if ddy > 0 then s_ddyp <= '1'; else s_ddyp <= '0'; end if;
                                if ddy /= 0 then s_ddyn <= '1'; else s_ddyn <= '0'; end if;
                                s_ts <= TS_MSTEP;
                            when TS_MSTEP =>
                                -- axis-select + one-cell step on registered deltas
                                mxi := to_integer(s_sel_mx);
                                myi := to_integer(s_sel_my);
                                v_tx := mxi; v_ty := myi;
                                if s_adx >= s_ady and s_ddxn = '1' then
                                    if s_ddxp = '1' then v_tx := mxi + 1; else v_tx := mxi - 1; end if;
                                elsif s_ddyn = '1' then
                                    if s_ddyp = '1' then v_ty := myi + 1; else v_ty := myi - 1; end if;
                                elsif s_ddxn = '1' then
                                    if s_ddxp = '1' then v_tx := mxi + 1; else v_tx := mxi - 1; end if;
                                end if;
                                s_tx <= to_unsigned(v_tx, 6);
                                s_ty <= to_unsigned(v_ty, 6);
                                s_ts <= TS_MBUMP;
                            when TS_MBUMP =>
                                -- bump check on the registered target (short path)
                                if s_tx = s_px and s_ty = s_py then
                                    s_ctarget <= s_mi;          -- bump -> combat
                                    s_cmt  <= s_sel_mt;
                                    s_cmhp <= s_sel_mhp;
                                    s_game <= G_COMBAT;
                                    s_cs <= CS_INIT;
                                    s_ts <= TS_IDLE;
                                else
                                    s_g_raddr <= s_ty & s_tx;
                                    s_ts <= TS_MWAIT;
                                end if;
                            when TS_MWAIT =>
                                -- occupancy = any registered match except self
                                v_occ := '0';
                                for j in 0 to C_MONS - 1 loop
                                    if j /= s_mi and s_mmatch(j) = '1' then
                                        v_occ := '1';
                                    end if;
                                end loop;
                                s_mocc <= v_occ;
                                s_ts <= TS_MAPPLY;
                            when TS_MAPPLY =>
                                if walkable(s_map_q) and s_mocc = '0' then
                                    s_mx(s_mi) <= s_tx;
                                    s_my(s_mi) <= s_ty;
                                end if;
                                if s_mi = C_MONS - 1 then s_ts <= TS_IDLE;
                                else s_mi <= s_mi + 1; s_ts <= TS_MREQ; end if;
                        end case;

                    ----------------------------------------------------------
                    when G_COMBAT =>
                        case s_cs is
                            when CS_INIT =>
                                -- pre-resolve monster combat stats to scalars
                                mt := to_integer(s_cmt);
                                s_cmatk <= C_MON_ATK(mt);
                                s_cmdef <= C_MON_DEF(mt);
                                s_cmxp  <= C_MON_XP(mt);
                                s_cs <= CS_INPUT;
                            when CS_INPUT =>
                                if v_edge = '1' then
                                    s_cb_act_l <= s_cb_action;     -- latch action
                                    s_cs <= CS_PLAYER;
                                end if;
                            when CS_PLAYER =>
                                -- player damage value (one cycle of arithmetic)
                                var := to_integer(unsigned(s_clfsr(1 downto 0)));
                                if s_cb_act_l = 2 then dmg := s_patk + 3 + var - s_cmdef;
                                else                   dmg := s_patk + var - s_cmdef; end if;
                                if dmg < 1 then dmg := 1; end if;
                                s_dmg <= dmg;
                                s_cs <= CS_PHIT;
                            when CS_PHIT =>
                                -- apply to monster hp; flag a kill
                                s_mon_dead <= '0';
                                if s_cb_act_l = 0 or s_cb_act_l = 2 then
                                    monhp := to_integer(s_cmhp) - s_dmg;
                                    if monhp < 0 then monhp := 0; end if;
                                    s_cmhp <= to_unsigned(monhp, 8);
                                    s_mhp(s_ctarget) <= to_unsigned(monhp, 8);
                                    if monhp = 0 then s_mon_dead <= '1'; end if;
                                end if;
                                s_cs <= CS_POUT;
                            when CS_POUT =>
                                -- outcome branch (kill / flee / counterattack)
                                if s_mon_dead = '1' then
                                    s_mactive(s_ctarget) <= '0';
                                    v_xp := to_integer(s_xp) + s_cmxp;
                                    if v_xp > 255 then v_xp := 255; end if;
                                    s_xp <= to_unsigned(v_xp, 8);
                                    s_cs <= CS_LVL;
                                elsif s_cb_act_l = 4 and s_clfsr(2) = '1' then
                                    s_game <= G_EXPLORE;             -- fled
                                    s_cs <= CS_INPUT;
                                else
                                    s_cs <= CS_MONSTER;
                                end if;
                            when CS_LVL =>
                                -- level-up when xp >= level*8 (a shift): stats up,
                                -- full heal - then back to exploring.
                                v_thr := to_integer(s_level) * 8;
                                if to_integer(s_xp) >= v_thr and s_level < 30 then
                                    s_xp    <= to_unsigned(to_integer(s_xp) - v_thr, 8);
                                    s_level <= s_level + 1;
                                    s_maxhp <= s_maxhp + 2;
                                    s_hp    <= s_maxhp + 2;
                                    if s_patk < 20 then s_patk <= s_patk + 1; end if;
                                end if;
                                s_game <= G_EXPLORE;
                                s_cs <= CS_INPUT;
                            when CS_MONSTER =>
                                -- monster damage value (one cycle of arithmetic)
                                var2 := to_integer(unsigned(s_clfsr(4 downto 3)));
                                mdmg := s_cmatk + var2 - s_pdef;
                                if s_cb_act_l = 1 then mdmg := mdmg / 2; end if;  -- Defend
                                if mdmg < 1 then mdmg := 1; end if;
                                s_mdmg <= mdmg;
                                s_cs <= CS_MHIT;
                            when CS_MHIT =>
                                php := to_integer(s_hp);
                                newhp := php - s_mdmg;
                                if newhp <= 0 then
                                    s_hp <= (others => '0');
                                    s_game <= G_GAMEOVER;
                                else
                                    s_hp <= to_unsigned(newhp, 8);
                                end if;
                                s_cs <= CS_INPUT;
                        end case;

                    ----------------------------------------------------------
                    when G_GAMEOVER =>
                        if v_edge = '1' then
                            s_game <= G_CREATE;
                        end if;

                end case;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- HUD: BCD digit ROM read port + per-frame status update FSM.
    -- One field at a time: present value -> wait -> write its digit glyphs.
    --------------------------------------------------------------------------
    p_bcd : process(clk)
    begin
        if rising_edge(clk) then
            s_bcd_q <= s_bcd(to_integer(s_bcd_addr));
        end if;
    end process;

    p_hud : process(clk)
        variable v_val : integer range 0 to 4095;
        variable v_g : integer range 0 to 127;
    begin
        if rising_edge(clk) then
            s_hr_we <= '0';
            case s_hu_state is
                when HU_IDLE =>
                    if s_vsync_pulse = '1' then
                        s_hu_field <= 0;
                        s_hu_state <= HU_REQ;
                    end if;
                when HU_REQ =>
                    case s_hu_field is
                        when 0 => v_val := to_integer(s_hp);
                        when 1 => v_val := to_integer(s_maxhp);
                        when 2 => v_val := to_integer(s_gold);
                        when 3 => v_val := to_integer(s_level);
                        when 4 => v_val := to_integer(s_depth);
                        when others => v_val := to_integer(s_cmhp);
                    end case;
                    if v_val > 1023 then v_val := 1023; end if;
                    s_bcd_addr <= to_unsigned(v_val, 10);
                    s_hu_state <= HU_WAIT;
                when HU_WAIT =>
                    s_hu_state <= HU_LATCH;
                when HU_LATCH =>
                    case s_hu_field is
                        when 0 =>
                            s_dig(0) <= to_integer(unsigned(s_bcd_q(7 downto 4)));
                            s_dig(1) <= to_integer(unsigned(s_bcd_q(3 downto 0)));
                        when 1 =>
                            s_dig(2) <= to_integer(unsigned(s_bcd_q(7 downto 4)));
                            s_dig(3) <= to_integer(unsigned(s_bcd_q(3 downto 0)));
                        when 2 =>
                            s_dig(4) <= to_integer(unsigned(s_bcd_q(11 downto 8)));
                            s_dig(5) <= to_integer(unsigned(s_bcd_q(7 downto 4)));
                            s_dig(6) <= to_integer(unsigned(s_bcd_q(3 downto 0)));
                        when 3 =>
                            s_dig(7) <= to_integer(unsigned(s_bcd_q(7 downto 4)));
                            s_dig(8) <= to_integer(unsigned(s_bcd_q(3 downto 0)));
                        when 4 =>
                            s_dig(9) <= to_integer(unsigned(s_bcd_q(3 downto 0)));
                        when others =>
                            s_dig(10) <= to_integer(unsigned(s_bcd_q(7 downto 4)));
                            s_dig(11) <= to_integer(unsigned(s_bcd_q(3 downto 0)));
                    end case;
                    if s_hu_field = 5 then
                        s_hu_col <= 0;
                        s_hu_state <= HU_BOT;
                    else
                        s_hu_field <= s_hu_field + 1;
                        s_hu_state <= HU_REQ;
                    end if;
                when HU_BOT =>
                    -- bottom status bar, one cell/cycle: "HP tt/oo G hhh LV ll D d"
                    case s_hu_col is
                        when 0 => v_g := G_H;  when 1 => v_g := G_P;
                        when 3 => v_g := G_D0 + s_dig(0);
                        when 4 => v_g := G_D0 + s_dig(1);
                        when 5 => v_g := G_SLASH;
                        when 6 => v_g := G_D0 + s_dig(2);
                        when 7 => v_g := G_D0 + s_dig(3);
                        when 9 => v_g := G_G;
                        when 11 => v_g := G_D0 + s_dig(4);
                        when 12 => v_g := G_D0 + s_dig(5);
                        when 13 => v_g := G_D0 + s_dig(6);
                        when 15 => v_g := G_L; when 16 => v_g := G_V;
                        when 18 => v_g := G_D0 + s_dig(7);
                        when 19 => v_g := G_D0 + s_dig(8);
                        when 21 => v_g := G_D;
                        when 23 => v_g := G_D0 + s_dig(9);
                        when others => v_g := G_SP;
                    end case;
                    s_hr_waddr <= to_unsigned(s_hu_col, 7);          -- bit6=0: bottom
                    s_hr_wdata <= std_logic_vector(to_unsigned(v_g, 7));
                    s_hr_we    <= '1';
                    if s_hu_col = C_HUDN - 1 then
                        s_hu_col <= 0;
                        s_hu_state <= HU_TOP;
                    else
                        s_hu_col <= s_hu_col + 1;
                    end if;
                when HU_TOP =>
                    -- top context line, one cell/cycle.
                    v_g := G_SP;
                    case s_game is
                        when G_CREATE =>      -- "CLASS xxx  S7=GO"
                            case s_hu_col is
                                when 0 => v_g := G_C; when 1 => v_g := G_L;
                                when 2 => v_g := G_A; when 3 => v_g := G_S;
                                when 4 => v_g := G_S;
                                when 6 => v_g := s_clsn(0);
                                when 7 => v_g := s_clsn(1);
                                when 8 => v_g := s_clsn(2);
                                when 10 => v_g := G_S; when 11 => v_g := G_D0 + 7;
                                when 12 => v_g := G_EQ; when 13 => v_g := G_G;
                                when 14 => v_g := G_O;
                                when others => v_g := G_SP;
                            end case;
                        when G_COMBAT =>      -- "FIGHT xxx HP nn  yyy"
                            case s_hu_col is
                                when 0 => v_g := G_F; when 1 => v_g := G_I;
                                when 2 => v_g := G_G; when 3 => v_g := G_H;
                                when 4 => v_g := G_T;
                                when 6 => v_g := s_monn(0);
                                when 7 => v_g := s_monn(1);
                                when 8 => v_g := s_monn(2);
                                when 10 => v_g := G_H; when 11 => v_g := G_P;
                                when 13 => v_g := G_D0 + s_dig(10);
                                when 14 => v_g := G_D0 + s_dig(11);
                                when 16 => v_g := s_actn(0);
                                when 17 => v_g := s_actn(1);
                                when 18 => v_g := s_actn(2);
                                when others => v_g := G_SP;
                            end case;
                        when G_GAMEOVER =>    -- "YOU DIED  S7=GO"
                            case s_hu_col is
                                when 0 => v_g := G_Y; when 1 => v_g := G_O;
                                when 2 => v_g := G_U; when 4 => v_g := G_D;
                                when 5 => v_g := G_I; when 6 => v_g := G_E;
                                when 7 => v_g := G_D;
                                when 10 => v_g := G_S; when 11 => v_g := G_D0 + 7;
                                when 12 => v_g := G_EQ; when 13 => v_g := G_G;
                                when 14 => v_g := G_O;
                                when others => v_g := G_SP;
                            end case;
                        when G_GEN =>         -- "DESCENDING"
                            case s_hu_col is
                                when 0 => v_g := G_D; when 1 => v_g := G_E;
                                when 2 => v_g := G_S; when 3 => v_g := G_C;
                                when 4 => v_g := G_E; when 5 => v_g := G_N;
                                when 6 => v_g := G_D; when 7 => v_g := G_I;
                                when 8 => v_g := G_N; when 9 => v_g := G_G;
                                when others => v_g := G_SP;
                            end case;
                        when others =>       -- EXPLORE: pickup message or blank
                            if s_msg = MSG_SWORD then       -- "GOT SWORD"
                                case s_hu_col is
                                    when 0 => v_g := G_G; when 1 => v_g := G_O;
                                    when 2 => v_g := G_T; when 4 => v_g := G_S;
                                    when 5 => v_g := G_W; when 6 => v_g := G_O;
                                    when 7 => v_g := G_R; when 8 => v_g := G_D;
                                    when others => v_g := G_SP;
                                end case;
                            elsif s_msg = MSG_GOLD then     -- "GOT GOLD"
                                case s_hu_col is
                                    when 0 => v_g := G_G; when 1 => v_g := G_O;
                                    when 2 => v_g := G_T; when 4 => v_g := G_G;
                                    when 5 => v_g := G_O; when 6 => v_g := G_L;
                                    when 7 => v_g := G_D;
                                    when others => v_g := G_SP;
                                end case;
                            else
                                v_g := G_SP;
                            end if;
                    end case;
                    s_hr_waddr <= to_unsigned(64 + s_hu_col, 7);     -- bit6=1: top line
                    s_hr_wdata <= std_logic_vector(to_unsigned(v_g, 7));
                    s_hr_we    <= '1';
                    if s_hu_col = C_HUDN - 1 then
                        s_hu_state <= HU_IDLE;
                    else
                        s_hu_col <= s_hu_col + 1;
                    end if;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- HUD char RAM: 1 write port (update FSM, vblank) + 1 registered read port
    -- (render, addr = top-flag & column).  A block RAM, so the render-side
    -- glyph fetch is a clean registered read with no wide mux.
    --------------------------------------------------------------------------
    p_hudram : process(clk)
    begin
        if rising_edge(clk) then
            if s_hr_we = '1' then
                s_hudram(to_integer(s_hr_waddr)) <= s_hr_wdata;
            end if;
            s1_glyph <= unsigned(s_hudram(to_integer(s0_hud_top & s0_hud_col)));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sync passthrough delay line.
    --------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_syncp(0) <= data_in.field_n & data_in.avid &
                          data_in.vsync_n & data_in.hsync_n;
            for k in 1 to C_SYNCD loop
                s_syncp(k) <= s_syncp(k - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S0: world coordinates (screen pixel + per-frame scroll).  Just the add.
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable cur_x, cur_y : unsigned(11 downto 0);
        variable v_yoff : unsigned(11 downto 0);
        variable v_inhud : std_logic;
    begin
        if rising_edge(clk) then
            if s_timing.avid_start = '1' then
                cur_x := (others => '0');
                if s_firstline = '1' then cur_y := (others => '0');
                else                      cur_y := s_y + 1; end if;
                s_x <= to_unsigned(1, 12);
                s_y <= cur_y;
            elsif s_timing.avid = '1' then
                cur_x := s_x; cur_y := s_y;
                s_x <= s_x + 1;
            else
                cur_x := s_x; cur_y := s_y;
            end if;

            s0_wx <= signed(resize(cur_x, 14)) + s_scroll_x;
            s0_wy <= signed(resize(cur_y, 14)) + s_scroll_y;
            s0_render <= s_timing.avid or s_timing.avid_start;

            -- HUD strips (screen space, 16 px glyph cells): top context line at
            -- the top margin, bottom status bar at the bottom.
            if cur_y >= C_HUD_MARG and cur_y < C_HUD_MARG + C_HUD_H
               and cur_x < C_HUDN * (2 ** C_HUD_LOG2) then
                v_inhud := '1';
                v_yoff := cur_y - C_HUD_MARG;
                s0_hud_top <= '1';
            elsif cur_y >= s_hud_y0 and (cur_y - s_hud_y0) < C_HUD_H
                  and cur_x < C_HUDN * (2 ** C_HUD_LOG2) then
                v_inhud := '1';
                v_yoff := cur_y - s_hud_y0;
                s0_hud_top <= '0';
            else
                v_inhud := '0';
                v_yoff := (others => '0');
                s0_hud_top <= '0';
            end if;
            -- cell = 2**C_HUD_LOG2 px; 8x8 glyph scaled by 2**(C_HUD_LOG2-3).
            s0_hud_active <= v_inhud;
            s0_hud_col <= cur_x(C_HUD_LOG2 + 5 downto C_HUD_LOG2);
            s0_frow <= v_yoff(C_HUD_LOG2 - 1 downto C_HUD_LOG2 - 3);
            s0_fcol <= cur_x(C_HUD_LOG2 - 1 downto C_HUD_LOG2 - 3);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1: cell decode + in-bounds / in-player (compares isolated in their own
    -- stage so neither the world add nor the BRAM reads share their path).
    --------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_col, v_row : unsigned(C_MAP_LOG2 - 1 downto 0);
        variable v_inb, v_inpl : std_logic;
    begin
        if rising_edge(clk) then
            v_col := unsigned(s0_wx(C_CELL_LOG2 + C_MAP_LOG2 - 1 downto C_CELL_LOG2));
            v_row := unsigned(s0_wy(C_CELL_LOG2 + C_MAP_LOG2 - 1 downto C_CELL_LOG2));

            if s0_wx(13 downto C_CELL_LOG2 + C_MAP_LOG2) = 0 and
               s0_wy(13 downto C_CELL_LOG2 + C_MAP_LOG2) = 0 then
                v_inb := '1';
            else
                v_inb := '0';
            end if;

            if v_inb = '1' and v_col = s_px and v_row = s_py then
                v_inpl := '1';
            else
                v_inpl := '0';
            end if;

            if s_show_tgt = '1' and v_inb = '1'
               and v_col = s_tgt_x and v_row = s_tgt_y then
                s1_intgt <= '1';
            else
                s1_intgt <= '0';
            end if;

            s1_addr <= v_row & v_col;
            -- nearest-neighbour art pixel within the (larger) cell: middle 4 bits
            s1_gx <= unsigned(s0_wx(C_CELL_LOG2 - 1 downto C_CELL_LOG2 - C_ART_LOG2));
            s1_gy <= unsigned(s0_wy(C_CELL_LOG2 - 1 downto C_CELL_LOG2 - C_ART_LOG2));
            s1_inb <= v_inb;
            s1_inplayer <= v_inpl;
            s1_render <= s0_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2a: pipeline carries (the tile value comes from the shared s_map_q,
    -- which p_mapport reads using s1_addr while avid is high - same latency).
    --------------------------------------------------------------------------
    p_map : process(clk)
    begin
        if rising_edge(clk) then
            s2_gx <= s1_gx; s2_gy <= s1_gy;
            s2_inb <= s1_inb; s2_intgt <= s1_intgt; s2_render <= s1_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2b: entity layer - 16-way parallel cell match + priority encode.
    -- Player has top priority; first active monster at the cell wins otherwise.
    -- eid is the sprite index (player class 0..3, or monster base + type).
    --------------------------------------------------------------------------
    p_ent : process(clk)
        variable v_col, v_row : unsigned(5 downto 0);
        variable v_hit, v_found : std_logic;
        variable v_eid : integer range 0 to C_NSPR - 1;
    begin
        if rising_edge(clk) then
            v_col := s1_addr(5 downto 0);
            v_row := s1_addr(2 * C_MAP_LOG2 - 1 downto C_MAP_LOG2);
            v_hit := '0'; v_found := '0'; v_eid := 0;

            if s1_inb = '1' then
                if s1_inplayer = '1' then
                    v_hit := '1';
                    v_eid := to_integer(s_class);   -- 0..3
                else
                    for i in 0 to C_MONS - 1 loop
                        if v_found = '0' and s_mactive(i) = '1'
                           and s_mx(i) = v_col and s_my(i) = v_row then
                            v_hit := '1';
                            v_found := '1';
                            v_eid := C_MON_BASE + to_integer(s_mtype(i));
                        end if;
                    end loop;
                end if;
            end if;

            s2_ehit <= v_hit;
            s2_eid  <= v_eid;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- HUD read pipeline (parallel to the world pipeline, same depth):
    --   S1 glyph fetch  ->  S2 font ROM read  ->  S3/S4 carry to colour.
    --------------------------------------------------------------------------
    p_hud_pipe : process(clk)
        variable v_faddr : integer range 0 to C_FROM_DEPTH - 1;
    begin
        if rising_edge(clk) then
            -- S1: carries (the glyph itself is fetched by p_hudram in parallel).
            s1_hud_active <= s0_hud_active;
            s1_frow <= s0_frow; s1_fcol <= s0_fcol;

            -- S2: font ROM read (addr = glyph*8 + frow).
            v_faddr := to_integer(s1_glyph) * 8 + to_integer(s1_frow);
            s2_fontrow <= s_from(v_faddr);
            s2_hud_active <= s1_hud_active;
            s2_fcol <= s1_fcol;

            -- S3/S4: carry to align with the world pipeline colour stage.
            s3_fontrow <= s2_fontrow;
            s3_hud_active <= s2_hud_active;
            s3_fcol <= s2_fcol;
            s4_fontrow <= s3_fontrow;
            s4_hud_active <= s3_hud_active;
            s4_fcol <= s3_fcol;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S3: entity sprite ROM read (addr = eid*16 + gy) + carry tile id resolve.
    --------------------------------------------------------------------------
    p_spr : process(clk)
        variable v_saddr : integer range 0 to C_SPROM_DEPTH - 1;
        variable v_id : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            v_saddr := s2_eid * 16 + to_integer(s2_gy);
            s3_sprrow <= s_sprom(v_saddr);
            s3_ehit <= s2_ehit;
            s3_eid  <= s2_eid;

            v_id := to_integer(unsigned(s_map_q));
            if v_id >= C_NTILES then v_id := 0; end if;
            s3_tile <= v_id;

            s3_gx <= s2_gx; s3_gy <= s2_gy;
            s3_inb <= s2_inb; s3_intgt <= s2_intgt; s3_render <= s2_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S4: tile ROM read (addr = tile*16 + gy).
    --------------------------------------------------------------------------
    p_trow : process(clk)
        variable v_addr : integer range 0 to C_TROM_DEPTH - 1;
    begin
        if rising_edge(clk) then
            v_addr := s3_tile * 16 + to_integer(s3_gy);
            s4_row    <= s_trom(v_addr);
            s4_sprrow <= s3_sprrow;
            s4_ehit   <= s3_ehit;
            s4_eid    <= s3_eid;
            s4_tile   <= s3_tile;
            s4_gx     <= s3_gx;
            s4_gy     <= s3_gy;
            s4_inb    <= s3_inb; s4_intgt <= s3_intgt; s4_render <= s3_render;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S5 (select): bit-extract all three layers + decide which one wins.
    -- Only single bits + a short priority chain here - no palette muxes - so
    -- this stage stays short.
    --------------------------------------------------------------------------
    p_sel : process(clk)
        variable v_tbit, v_sbit, v_text, v_ring : std_logic;
        variable p : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            p := to_integer(s4_gx);
            v_tbit := s4_row(15 - p);
            v_sbit := s4_sprrow(15 - p);
            v_text := s4_fontrow(7 - to_integer(s4_fcol));
            -- reticle ring = the target cell's outline (1 art-pixel border)
            if s4_intgt = '1' and (s4_gx = 0 or s4_gx = 15 or
                                   s4_gy = 0 or s4_gy = 15) then
                v_ring := '1';
            else
                v_ring := '0';
            end if;

            if s4_hud_active = '1' then
                if v_text = '1' then s5_src <= SRC_HUDINK;
                else                 s5_src <= SRC_HUDBAR; end if;
            elsif s_gensp = '1' then
                s5_src <= SRC_VOID;          -- generation splash hides the map
            elsif s4_inb = '0' then
                s5_src <= SRC_VOID;
            elsif s4_ehit = '1' and v_sbit = '1' then
                s5_src <= SRC_ENT;
            elsif v_ring = '1' then
                s5_src <= SRC_TGT;
            elsif v_tbit = '1' then
                s5_src <= SRC_TILEFG;
            else
                s5_src <= SRC_TILEBG;
            end if;

            s5_eid  <= s4_eid;
            s5_tile <= s4_tile;
            s5_sync <= s_syncp(C_SYNCD);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S6 (lookup): palette ROM/array -> YUV for the selected layer.
    --------------------------------------------------------------------------
    p_color : process(clk)
        variable e : integer range 0 to C_NSPR - 1;
        variable t : integer range 0 to C_NTILES - 1;
    begin
        if rising_edge(clk) then
            e := s5_eid;
            t := s5_tile;
            case s5_src is
                when SRC_HUDINK =>
                    s6_y <= C_INK_Y; s6_u <= C_INK_U; s6_v <= C_INK_V;
                when SRC_HUDBAR =>
                    s6_y <= C_BAR_Y; s6_u <= C_BAR_U; s6_v <= C_BAR_V;
                when SRC_VOID =>
                    s6_y <= C_VOID_Y; s6_u <= C_VOID_U; s6_v <= C_VOID_V;
                when SRC_TGT =>
                    s6_y <= C_TGT_Y; s6_u <= C_TGT_U; s6_v <= C_TGT_V;
                when SRC_ENT =>
                    s6_y <= to_unsigned(C_SPR_Y(e), 10);
                    s6_u <= to_unsigned(C_SPR_U(e), 10);
                    s6_v <= to_unsigned(C_SPR_V(e), 10);
                when SRC_TILEFG =>
                    s6_y <= to_unsigned(C_TILE_FG_Y(t), 10);
                    s6_u <= to_unsigned(C_TILE_FG_U(t), 10);
                    s6_v <= to_unsigned(C_TILE_FG_V(t), 10);
                when others =>
                    s6_y <= to_unsigned(C_TILE_BG_Y(t), 10);
                    s6_u <= to_unsigned(C_TILE_BG_U(t), 10);
                    s6_v <= to_unsigned(C_TILE_BG_V(t), 10);
            end case;
            s6_sync <= s5_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output registers.
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io.y       <= std_logic_vector(s6_y);
            s_io.u       <= std_logic_vector(s6_u);
            s_io.v       <= std_logic_vector(s6_v);
            s_io.hsync_n <= s6_sync(0);
            s_io.vsync_n <= s6_sync(1);
            s_io.avid    <= s6_sync(2);
            s_io.field_n <= s6_sync(3);
        end if;
    end process;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture oubliette;
