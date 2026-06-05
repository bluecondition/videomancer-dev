-- Bishop v2 — line-buffer wireframe head.
--
-- Architecture:
--   v1 instantiated one DDA + per-pixel hit-tester per edge, in parallel.
--   At ~80 LCs/edge × 50 edges that filled most of HX4K just on edge logic.
--
--   v2 uses a SINGLE shared rasterizer that walks edges sequentially and
--   writes stamps into a 1-bit/pixel dual-bank line buffer (inlined here
--   so we can gate writes — the shared video_line_buffer entity writes
--   every cycle, which kept overwriting fresh stamps with zeros).
--
-- Per-scanline rasterizer timeline (running during the line BEFORE the
-- one being prepared):
--   R_CLEAR  2*HEAD_HALF_W cycles    write 0 across head's x bbox of next-line bank
--                                    (edge update fires once per cycle for first
--                                     N_EDGES cycles, in parallel with clears)
--   R_STAMP  N_EDGES * (2*THICK+1) cycles
--                                    write 1 at h_centre + current_x[i] ± THICK
--                                    for each active edge (with bbox bounds check)
--   R_IDLE                           wait for next hsync (writes gated off)
--
-- Budget @ 96 edges, THICK=1, HEAD_HALF_W=280:
--   CLEAR  560  +  STAMP 96*3 = 288  ≈ 848 cycles/line
--   SD line = 858 cycles, HD = 2200.  Fits.
--
-- Register map:
--   registers_in(0) = K1 Scale      (latched, behavior deferred to a later milestone)
--   registers_in(1) = K2 Thickness  (top 2 bits → THICK ∈ {1, 2, 3, 4})
--   registers_in(2) = K3 Palette    (top 3 bits = palette index)
--   registers_in(3) = K4 Expression (top 3 bits → 1-of-5; ≥5 folds to neutral)
--   registers_in(4) = K5 Grid period (top 2 bits → {16, 32, 64, 128})
--   registers_in(5) = K6 Grid scroll (top 8 bits, biased ±; speed+dir of auto-scroll)
--   registers_in(6) = Switches      (bit0 = T7 Mouth open, bit1 = T8 Eyes open,
--                                    bit2 = T9 grid on/off, bit3 = T10 thick grid,
--                                    bit4 = T11 BG-Video)
--   registers_in(7) = Fader Brightness
--
-- License: GPL-3.0
-- Author: ron

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.bishop_mesh_pkg.all;
use work.all;

architecture bishop of program_top is

    -- ---------------------------------------------------------------
    -- Geometry / rasterizer parameters
    -- ---------------------------------------------------------------
    constant HEAD_HALF_W : natural := 258;  -- 1.25x head (TARGET_H=950); >=max|x|~253 and >=ceil((N+12)/2)
    constant HEAD_HALF_H : natural := 420;  -- half-height for the grid mask
    constant CLEAR_W     : natural := 2 * HEAD_HALF_W;  -- cycles in CLEAR phase
    -- Edge thickness is now a per-frame latched value (`thick_r`), driven
    -- by K2.  build_face_mesh.py sizes the tip-strip math for THICK_BUILD=4
    -- so any K2 setting up to 4 stays parity-correct.
    constant THICK_MAX   : natural := 4;
    -- Cap stamps per edge so a single near-horizontal high-slope edge
    -- can't blow the per-scanline cycle budget.  Edges with
    -- |slope| + 2*THICK > MAX_STAMP_M1 render only the first portion
    -- of their sweep (some pixels at the far end go un-stamped).
    constant MAX_STAMP_M1 : natural := 127;  -- 128 stamps max per edge

    -- Inlined line buffer is 1 BRAM cycle + 1 output flop = 2 cycles.
    -- Plus palette-mul flop = 3.  Match the bypass pipe accordingly.
    constant LATENCY : natural := 3;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_pal_lut is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal_lut := (
        to_unsigned( 720, 10), to_unsigned( 760, 10),
        to_unsigned( 870, 10), to_unsigned( 320, 10),
        to_unsigned(1000, 10), to_unsigned( 540, 10),
        to_unsigned( 940, 10), to_unsigned( 280, 10));
    constant C_PAL_U : t_pal_lut := (
        to_unsigned( 220, 10), to_unsigned( 260, 10),
        to_unsigned( 720, 10), to_unsigned( 400, 10),
        to_unsigned( 512, 10), to_unsigned( 800, 10),
        to_unsigned( 130, 10), to_unsigned( 920, 10));
    constant C_PAL_V : t_pal_lut := (
        to_unsigned( 220, 10), to_unsigned( 720, 10),
        to_unsigned( 220, 10), to_unsigned( 880, 10),
        to_unsigned( 512, 10), to_unsigned( 880, 10),
        to_unsigned( 580, 10), to_unsigned( 380, 10));

    -- ---------------------------------------------------------------
    -- Frame / line timing
    -- ---------------------------------------------------------------
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal hsync_falling_r  : std_logic := '0';
    signal vsync_falling_r  : std_logic := '0';
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal max_y_r      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal h_centre_r   : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal v_centre_r   : unsigned(11 downto 0) := to_unsigned(540, 12);
    -- Registered scanline target row (pixel_y - v_centre + 1).  Pipelined
    -- out of the Stage-1 active-update compares: pixel_y/v_centre_r are
    -- stable across the whole R_CLEAR edge walk, so the 1-cycle delay is
    -- transparent, and it removes the 13-bit subtract from the long
    -- subtract-then-compare carry chain that limited HD HDMI timing.
    signal v_y_target_r : signed(12 downto 0) := (others => '0');

    -- ---------------------------------------------------------------
    -- Per-frame latched user controls (all updated on vsync_falling_r)
    -- ---------------------------------------------------------------
    signal pal_y_r    : unsigned(9 downto 0) := C_PAL_Y(0);
    signal pal_u_r    : unsigned(9 downto 0) := C_PAL_U(0);
    signal pal_v_r    : unsigned(9 downto 0) := C_PAL_V(0);
    signal bright_r   : unsigned(7 downto 0) := to_unsigned(220, 8);
    signal bg_video_r : std_logic := '0';
    -- T9: when '1', the EOR-fill mask paints a grid inside the silhouette.
    signal grid_en_r  : std_logic := '0';
    -- T10: when '1', grid lines are 2px wide instead of 1px.
    signal grid_thick_r : std_logic := '0';
    -- K1 Scale — latched, behavior deferred to a future milestone.
    signal scale_r    : unsigned(9 downto 0) := (others => '0');
    -- K2 Thickness — stored as the actual THICK value (1..4).
    signal thick_r    : unsigned(2 downto 0) := to_unsigned(1, 3);
    -- K4 Expression — selects which row of the C_EDGE_* mesh tables to
    -- index.  Clamped to [0, C_NUM_EXPR-1] in the vsync latch.
    signal expr_idx_r : unsigned(2 downto 0) := (others => '0');
    -- K5 Grid period (00=16, 01=32, 10=64, 11=128).
    signal grid_per_r : unsigned(1 downto 0) := "01";   -- default 32 px (v2.1 spacing)
    -- K6 Grid scroll: a Q7.6 phase accumulator (13 bits = 7 integer +
    -- 6 fractional) advanced by the signed scroll_speed_r once per
    -- vsync.  The integer part (bits 12..6) is subtracted from pixel_x
    -- before masking, so the grid drifts smoothly left/right.  At max
    -- speed it moves ~2px/frame; near K6 centre it's stopped.
    signal grid_phs_acc_r : unsigned(12 downto 0) := (others => '0');
    signal scroll_speed_r : signed(7 downto 0) := (others => '0');
    -- S7 / S8: open mouth / open eyes.  Default 0 = closed (the rest
    -- pose); 1 = open (the expression's tuned mouth/eye).  Selected
    -- per-edge in Stage 0 based on C_EDGE_GROUP.
    signal open_mouth_r : std_logic := '0';
    signal open_eyes_r  : std_logic := '0';

    -- ---------------------------------------------------------------
    -- Mutable edge state.  current_x lives in a small BRAM (1R1W,
    -- 1-cycle read latency) so we don't pay LCs + wide muxes for an
    -- N_EDGES register file.  active stays in flops so it can be
    -- bulk-cleared on vsync.
    -- ---------------------------------------------------------------
    -- Q9.7 fixed-point: bits 15..7 are the integer pixel position
    -- (signed, ±256 range — covers head ±200), bits 6..0 are 1/128-px
    -- fractional.  Slope rounding error is ~1/256 per row, so a 50-row
    -- edge drifts ~0.2 px — small enough that edges visibly meet at
    -- shared vertices.  Same 16-bit storage as Q12.4 so current_x_ram
    -- still fits a single EBR.
    type t_cur_x_ram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal current_x_ram : t_cur_x_ram := (others => (others => '0'));
    signal cx_rd_addr  : unsigned(7 downto 0) := (others => '0');
    signal cx_rd_data  : std_logic_vector(15 downto 0);
    signal cx_wr_addr  : unsigned(7 downto 0) := (others => '0');
    signal cx_wr_data  : std_logic_vector(15 downto 0) := (others => '0');
    signal cx_wr_en    : std_logic := '0';

    signal active    : std_logic_vector(0 to C_NUM_EDGES - 1)
                     := (others => '0');

    -- ---------------------------------------------------------------
    -- Rasterizer FSM
    -- ---------------------------------------------------------------
    -- Active-edge list: during R_CLEAR the edge-update appends the index of
    -- every edge active on this scanline to act_list_ram.  R_STAMP then
    -- iterates ONLY that list (R_STAMP_SCAN issues the list read, SCAN2
    -- consumes it and enters the stamp pipeline).  This replaces the old
    -- inline scan that indexed the full active vector every cycle (a wide
    -- N:1 mux that was the HD critical path) and walked all N edges — now
    -- the stamp phase is O(active-this-row) with no wide mux.
    type t_raster_state is (R_IDLE, R_CLEAR, R_STAMP_SCAN, R_STAMP_SCAN2,
                             R_STAMP_PRELOAD, R_STAMP_PRELOAD2, R_STAMP_PRELOAD3,
                             R_STAMP, R_STAMP_DRAIN);
    signal raster_state    : t_raster_state := R_IDLE;
    signal raster_cycle    : unsigned(10 downto 0) := (others => '0');
    -- Active-edge list storage + pointers (single-driver: all in raster_proc).
    type t_act_list_ram is array (0 to 255) of std_logic_vector(7 downto 0);
    signal act_list_ram : t_act_list_ram := (others => (others => '0'));
    signal al_wr_addr   : unsigned(7 downto 0) := (others => '0');  -- build ptr (R_CLEAR)
    signal al_rd_addr   : unsigned(7 downto 0) := (others => '0');  -- iterate ptr (R_STAMP)
    signal al_rd_data   : std_logic_vector(7 downto 0) := (others => '0');
    signal al_count     : unsigned(7 downto 0) := (others => '0');  -- # active this line
    signal al_wr_en     : std_logic := '0';
    signal al_wr_data   : std_logic_vector(7 downto 0) := (others => '0');
    -- Wide enough for max |slope| + 2*THICK.
    signal stamp_sub       : unsigned(7 downto 0)  := (others => '0');

    -- Per-edge values held across the edge's STAMP iterations.  Latched
    -- once at edge transition.  cur_x doesn't need a "held" register —
    -- cx_rd_addr stays parked on the edge's index for the duration of
    -- its stamps, so cx_rd_data is steady (and registered, so it has
    -- the same per-cycle cost as a flop).
    signal active_held    : std_logic := '0';
    signal bnd_held       : std_logic := '0';   -- 1=route stamps to boundary buffer
    -- bnd_held delayed 1 cycle.  R_STAMP_PRELOAD drains Stage B's
    -- pending stamp from the PREVIOUS edge but bnd_held has already
    -- been latched to the NEXT edge by latch_held, so we'd mis-route
    -- that one stamp into the wrong buffer (visible as extra/missing
    -- EOR crossings on the last row of every bnd<->det edge transition).
    signal bnd_held_d1    : std_logic := '0';
    signal start_rel_held : signed(7 downto 0)  := (others => '0');
    signal count_m1_held  : unsigned(7 downto 0) := (others => '0');
    -- Per-edge x-extent for THIS edge (latched in compute_counts_a from the
    -- x-extent ROMs).  The stamp loop is gated to [xlo_held, xhi_held] so the
    -- thickness pad can't push a near-horizontal edge past its endpoints.
    signal xlo_held       : signed(12 downto 0) := (others => '0');
    signal xhi_held       : signed(12 downto 0) := (others => '0');

    -- Stamp pipeline registers (Stage A → Stage B).  Splits the
    -- (cur_x + start_rel + sub) + h_centre add chain into two cycles.
    -- Stage A registers v_stamp_rel + valid, Stage B does the +h_centre
    -- and writes the LB.  Adds 1 cycle of latency, drained in
    -- R_STAMP_DRAIN.
    signal stamp_rel_r    : signed(12 downto 0) := (others => '0');
    signal stamp_valid_r  : std_logic := '0';
    -- EOR-vs-thickness decoupling.  A boundary edge writes its full
    -- thick sweep to the DETAIL buffer (visible) but only a single 1px
    -- crossing mark to the BOUNDARY buffer (which the EOR fill walker
    -- reads).  center_sub_held is the stamp_sub offset at which the
    -- sweep pixel lands exactly on cur_x (the edge's true crossing);
    -- stamp_is_center_r flags that pixel through the Stage A->B pipe.
    -- Without this, thick/adjacent boundary stamps merge into one run
    -- and corrupt the EOR parity (the "5 sections" banding + the mouth
    -- stub were both this).
    signal center_sub_held  : unsigned(7 downto 0) := (others => '0');
    signal stamp_is_center_r : std_logic := '0';
    -- compute_counts is split across two preload cycles to shorten its
    -- combinational depth (it was the HD critical path).  Stage a
    -- (PRELOAD2) registers the slope's sign / magnitude / padding
    -- decision; stage b (PRELOAD3) does the add + cap + center.
    signal s_int_held  : signed(11 downto 0) := (others => '0');
    signal abs_s_held  : unsigned(7 downto 0) := (others => '0');
    signal pad_held    : std_logic := '0';   -- |slope_int| <= 2*thick
    signal sneg_held   : std_logic := '0';   -- slope is negative

    -- h_centre and bbox-edge constants pre-added once per scanline to
    -- avoid recomputing the (h_centre - HEAD_HALF_W) sum every cycle
    -- of R_CLEAR.
    signal clear_base_r   : unsigned(10 downto 0) := (others => '0');
    signal h_centre_s     : signed(13 downto 0)  := to_signed(960, 14);

    -- R_CLEAR edge-update pipeline stage.  Stage 0 (combinational from
    -- raster_cycle) looks up mesh constants and the current_x value;
    -- those are registered into upd_*_r.  Stage 1 (one cycle later)
    -- uses upd_*_r to decide y_min / y_max / advance and writes back to
    -- current_x / active.  Breaks the big "array read → ALU → array
    -- write" combinational chain that was the dominant critical path.
    signal upd_valid_r  : std_logic := '0';
    signal upd_idx_r    : unsigned(7 downto 0)  := (others => '0');
    signal upd_ymin_r   : signed(12 downto 0)   := (others => '0');
    signal upd_ymax_r   : signed(12 downto 0)   := (others => '0');
    signal upd_xtop_r   : signed(15 downto 0)   := (others => '0');  -- Q9.7
    signal upd_slope_r  : signed(15 downto 0)   := (others => '0');  -- Q9.7
    signal upd_active_r : std_logic := '0';
    signal upd_bnd_r    : std_logic := '0';   -- 1=boundary edge (EOR), 0=detail
    -- One more pipeline stage so Stage 1 fires the cycle AFTER the
    -- BRAM read commits.  Without this, Stage 1's cx_rd_data would be
    -- ram[OLD edge] (the read issued one cycle earlier than the latch
    -- in upd_*_r).
    signal upd_valid_r2  : std_logic := '0';
    signal upd_idx_r2    : unsigned(7 downto 0)  := (others => '0');
    -- EFFECTIVE activation window [ymin, ymax].  For ordinary edges these
    -- equal the raw mesh bounds.  For horizontal edges they are widened by
    -- the thickness pads (ymin - top_pad .. ymax + bot_pad) so the line
    -- spans thick_r scanlines and reads as thick as the steep edges.
    signal upd_ymin_r2   : signed(12 downto 0)   := (others => '0');
    signal upd_ymax_r2   : signed(12 downto 0)   := (others => '0');
    signal upd_xtop_r2   : signed(15 downto 0)   := (others => '0');  -- Q9.7
    signal upd_slope_r2  : signed(15 downto 0)   := (others => '0');  -- Q9.7
    signal upd_active_r2 : std_logic := '0';
    signal upd_bnd_r2    : std_logic := '0';
    -- 1 = this edge is exactly horizontal (ymin==ymax): a single-scanline
    -- line whose `slope` field holds its WIDTH, not a per-row slope.  Such
    -- edges get Y-thickened (active for thick_r rows, see upd_ymin_r2/
    -- upd_ymax_r2 below) and must NOT accumulate slope into cur_x.
    signal upd_is_horiz_r2 : std_logic := '0';
    -- Scanline-vs-edge compare flags, computed at the r->r2 stage (vs the
    -- effective bounds) and registered, so Stage 1 uses 1-bit flags instead
    -- of the 13-bit v_y_target compare (which was the HD critical path).
    -- Valid a stage early because v_y_target_r is constant across R_CLEAR.
    signal upd_is_ymin_r2   : std_logic := '0';   -- v_y_target_r = effective ymin
    signal upd_past_ymax_r2 : std_logic := '0';   -- v_y_target_r > effective ymax

    -- ---------------------------------------------------------------
    -- Active mesh BRAM (the BRAM-prefetch architecture).
    --
    -- The per-edge mesh-read mux (5:1 expr x N-entry LUT-ROM x 3-way
    -- variant) on the render critical path is what was capping HD
    -- timing.  Instead, the vblank copy engine (cp_proc) walks edges
    -- 0..N-1 once per frame and resolves (expr, group, om, oe) into a
    -- single value per (slot, field), written here.  The render then
    -- reads one BRAM word per slot — no LUT-ROM, no expr mux, no
    -- variant select on the per-pixel critical path.  4 EBRs.
    -- ---------------------------------------------------------------
    type t_act_y_ram is array (0 to 255) of std_logic_vector(12 downto 0);
    type t_act_x_ram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal act_ymin_ram  : t_act_y_ram := (others => (others => '0'));
    signal act_ymax_ram  : t_act_y_ram := (others => (others => '0'));
    signal act_xtop_ram  : t_act_x_ram := (others => (others => '0'));
    signal act_slope_ram : t_act_x_ram := (others => (others => '0'));

    signal act_rd_addr   : unsigned(7 downto 0) := (others => '0');
    signal act_ymin_rd   : std_logic_vector(12 downto 0);
    signal act_ymax_rd   : std_logic_vector(12 downto 0);
    signal act_xtop_rd   : std_logic_vector(15 downto 0);
    signal act_slope_rd  : std_logic_vector(15 downto 0);

    -- Static per-edge x-extent ROMs.  Pure geometry (no expression/variant
    -- dependence), so they sit in a plain ROM read at act_rd_addr in lockstep
    -- with the mesh BRAMs above — no cp_proc copy, no wide combinational mux.
    type t_xext_rom is array (0 to 255) of std_logic_vector(12 downto 0);
    function init_xext(src : t_int_array) return t_xext_rom is
        variable r : t_xext_rom := (others => (others => '0'));
    begin
        for i in 0 to C_NUM_EDGES - 1 loop
            r(i) := std_logic_vector(to_signed(src(i), 13));
        end loop;
        return r;
    end function;
    constant C_XLO_ROM   : t_xext_rom := init_xext(C_EDGE_X_LO);
    constant C_XHI_ROM   : t_xext_rom := init_xext(C_EDGE_X_HI);
    signal act_xlo_rd    : std_logic_vector(12 downto 0);
    signal act_xhi_rd    : std_logic_vector(12 downto 0);

    signal act_wr_en     : std_logic := '0';
    signal act_wr_addr   : unsigned(7 downto 0) := (others => '0');
    signal act_ymin_wr   : std_logic_vector(12 downto 0) := (others => '0');
    signal act_ymax_wr   : std_logic_vector(12 downto 0) := (others => '0');
    signal act_xtop_wr   : std_logic_vector(15 downto 0) := (others => '0');
    signal act_slope_wr  : std_logic_vector(15 downto 0) := (others => '0');

    -- ---------------------------------------------------------------
    -- Copy engine — SEQUENTIAL per-edge FSM.  Once per frame in vblank it
    -- walks edges 0..N-1, reads the (static) mesh, optionally jitters both
    -- endpoints (K6 noise) and recomputes the slope with a restoring DIVIDER
    -- (shallow add/compare per cycle — no multiply, so it meets the HD clock),
    -- then writes the active mesh BRAM.  ~22 cycles/edge fits the vblank
    -- window easily.  Only the 'open' mesh variant is read (the static dense
    -- mesh has no per-expression variants), which also trims the LUT-ROM mux.
    -- ---------------------------------------------------------------
    -- Split into many shallow states (free in vblank) so no single cycle has a
    -- deep combinational path — keeps the noise engine off the HD critical path.
    type t_cp_state is (CP_IDLE, CP_LOAD, CP_HASH, CP_PHASE, CP_TRI,
                        CP_MR0, CP_MR1, CP_MR2, CP_MLOAD, CP_MSEL, CP_MDO,
                        CP_OFF, CP_SORT, CP_FLOOR, CP_SPAN, CP_DX, CP_DIV, CP_WR);
    signal cp_state : t_cp_state := CP_IDLE;
    signal cp_addr  : unsigned(8 downto 0) := (others => '0');

    -- K6 NOISE: coherent 2D point wander.  K6 = spread (level 0..7), K5 =
    -- speed.  Each vertex's (dx,dy) is hashed from its (x,y) so a shared point
    -- moves identically and the wireframe stays joined.
    -- spread is a 5-bit LINEAR amplitude (0..31): offset = (tri*spread)>>1 in
    -- Q9.7, so the response is smooth (32 steps) and tops out near +-31px (what
    -- the old coarse level-4 / knob-50% gave) at full knob.
    signal noise_spread_r : unsigned(4 downto 0)  := (others => '0');
    signal noise_speed_r  : unsigned(4 downto 0)  := (others => '0');
    signal noise_t_r      : unsigned(15 downto 0) := (others => '0');
    signal noise_on       : std_logic;
    -- P9 "Noise": '1' = allow the thin-horizontal-streak texture (edges may
    -- collapse flat); '0' = enforce a min vertical span so they don't streak.
    signal noise_streaks_r : std_logic := '0';
    constant NOISE_SPAN_FLOOR : integer := 8;
    -- Latched triangle-wave samples (per endpoint, x/y phases) -> CP_AMP.
    signal t_trxt, t_tryt, t_trxb, t_tryb : signed(9 downto 0) := (others => '0');

    -- Triangle wave: period 1024, amplitude -256..+255.  Low 10 phase bits
    -- (plain truncation; resize() on signed would corrupt the wrap).
    function tri10(ph : signed(16 downto 0)) return signed is
        variable p  : unsigned(9 downto 0);
        variable up : unsigned(8 downto 0);
    begin
        p := unsigned(ph(9 downto 0));
        if p(9) = '0' then up := p(8 downto 0);
        else               up := to_unsigned(511, 9) - p(8 downto 0);
        end if;
        return resize(signed('0' & up), 10) - to_signed(256, 10);
    end function;

    -- Per-edge working registers (one edge in flight at a time).
    signal w_xtop, w_xbot, w_slope : signed(15 downto 0) := (others => '0');
    signal w_ymin, w_ymax          : signed(12 downto 0) := (others => '0');
    signal w_phxt, w_phyt, w_phxb, w_phyb : signed(16 downto 0) := (others => '0');
    signal o_dxt, o_dxb : signed(17 downto 0) := (others => '0');  -- x offsets
    signal o_dyt, o_dyb : signed(13 downto 0) := (others => '0');  -- y offsets
    signal w_nxt, w_nxb : signed(17 downto 0) := (others => '0');
    signal w_nyt, w_nyb : signed(13 downto 0) := (others => '0');
    signal w_xa, w_xb   : signed(17 downto 0) := (others => '0');
    signal w_ya, w_yb   : signed(13 downto 0) := (others => '0');
    signal w_dxsign     : std_logic := '0';
    -- Restoring divider: quot = |dx| / span (slope magnitude, Q9.7).
    signal d_rem      : unsigned(8 downto 0)  := (others => '0');
    signal d_quot     : unsigned(17 downto 0) := (others => '0');
    signal d_dividend : unsigned(17 downto 0) := (others => '0');
    signal d_divisor  : unsigned(7 downto 0)  := (others => '0');
    signal d_cnt      : integer range 0 to 17 := 0;

    -- ---------------------------------------------------------------
    -- MORPHS: mouth open/close (K1 analog), eye blink (P8), expression (K4
    -- blend).  Per-edge endpoint px deltas live in 12 packed ROMs (top/bot for
    -- mouth, eye, and 4 expressions).  In the cp_proc the selected deltas are
    -- scaled (mouth by K1, expression by the K4 blend t) and added to the
    -- endpoints at CP_OFF alongside the noise offset, so the existing
    -- sort->divider->slope pipeline recomputes the slope for free.  All in
    -- vblank; the per-pixel render path is untouched.
    -- ---------------------------------------------------------------
    signal mouth_frac_r : unsigned(4 downto 0) := (others => '0');  -- K1 CLOSE amount (0=open)
    signal expr_base_r  : unsigned(1 downto 0) := (others => '0');  -- K4 lower expr A (0..3)
    signal expr_frac_r  : unsigned(4 downto 0) := (others => '0');  -- K4 blend t (0..31)
    signal morph_active : std_logic;

    -- SPARSE morph storage: per-VERTEX delta ROMs (one per morph, indexed by a
    -- vertex slot 0..NSLOT-1; slot 0 = static = zero) + an edge->slot map.  A
    -- delta is a property of a vertex, so storing per moving-vertex (not per
    -- edge-endpoint) drops 12 edge ROMs to 6 vertex ROMs + 1 map (12->7 BRAM).
    type t_vrom is array (0 to C_MORPH_NSLOT - 1) of std_logic_vector(15 downto 0);
    function init_vrom(src : t_int_array) return t_vrom is
        variable r : t_vrom;
    begin
        for i in 0 to C_MORPH_NSLOT - 1 loop
            r(i) := std_logic_vector(to_unsigned(src(i) mod 65536, 16));
        end loop;
        return r;
    end function;
    constant C_VROM_M : t_vrom := init_vrom(C_VROM_MOUTH);
    constant C_VROM_E : t_vrom := init_vrom(C_VROM_EYE);
    constant C_VROM_H : t_vrom := init_vrom(C_VROM_HAPPY);
    constant C_VROM_S : t_vrom := init_vrom(C_VROM_SAD);
    constant C_VROM_A : t_vrom := init_vrom(C_VROM_ANGRY);
    constant C_VROM_U : t_vrom := init_vrom(C_VROM_SUR);

    type t_emap is array (0 to 255) of std_logic_vector(15 downto 0);
    function init_emap(src : t_int_array) return t_emap is
        variable r : t_emap := (others => (others => '0'));
    begin
        for i in 0 to C_NUM_EDGES - 1 loop
            r(i) := std_logic_vector(to_unsigned(src(i), 16));
        end loop;
        return r;
    end function;
    constant C_EMAP_ROM : t_emap := init_emap(C_MORPH_EMAP);

    -- emap read at cp_addr; the 6 vrom reads share one address (vrom_addr), so
    -- top and bottom endpoints are read in successive cycles (CP_MR0..MR2).
    signal emap_r    : std_logic_vector(15 downto 0);
    signal vrom_addr : unsigned(5 downto 0) := (others => '0');
    signal vr_m, vr_e, vr_h, vr_s, vr_a, vr_u : std_logic_vector(15 downto 0);
    -- latched top/bottom-endpoint deltas for the current edge (fed to CP_MLOAD).
    signal mrd_mt, mrd_mb, mrd_et, mrd_eb : std_logic_vector(15 downto 0);
    signal mrd_ht, mrd_hb, mrd_st, mrd_sb : std_logic_vector(15 downto 0);
    signal mrd_at, mrd_ab, mrd_ut, mrd_ub : std_logic_vector(15 downto 0);

    -- Per-edge morph working registers (px deltas), held for the multiply loop.
    signal m_mxt, m_myt, m_mxb, m_myb : signed(7 downto 0) := (others => '0'); -- mouth
    signal m_dxt, m_dyt, m_dxb, m_dyb : signed(8 downto 0) := (others => '0'); -- exprB - exprA
    -- one shared multiplier sequenced over 8 products (operand select split
    -- into CP_MSEL to keep the mux off the multiply path); results accumulate
    -- straight into mt_* so no per-product intermediate registers are kept.
    signal m_cnt   : integer range 0 to 11 := 0;   -- 0..3 noise, 4..7 expr, 8..11 mouth
    signal mul_a_r : signed(9 downto 0) := (others => '0');
    signal mul_b_r : signed(6 downto 0) := (others => '0');
    -- running total morph delta per endpoint (px): exprA + eye init, then the
    -- expr-blend and mouth products accumulate in CP_MDO.
    signal mt_xt, mt_yt, mt_xb, mt_yb : signed(10 downto 0) := (others => '0');

    -- Render-side Stage 0 pre signals.  The active-mesh BRAM read has
    -- effective 2-cycle latency from when Stage 0a sets act_rd_addr
    -- (register input) to when act_*_rd presents the data (register
    -- output): 1 cycle for the address to be visible, 1 cycle for the
    -- BRAM process to commit ram[addr] -> data register.  So control
    -- signals (valid/idx/active/bnd) need to ride a 2-stage shift
    -- (_pre -> _pre2) before Stage 0b reads them together with the
    -- act_*_rd data.  cx_rd_addr is also driven from _pre2 so
    -- cx_rd_data lines up the same way.
    signal upd_valid_pre  : std_logic := '0';
    signal upd_idx_pre    : unsigned(7 downto 0) := (others => '0');
    signal upd_active_pre : std_logic := '0';
    signal upd_bnd_pre    : std_logic := '0';
    signal upd_valid_pre2  : std_logic := '0';
    signal upd_idx_pre2    : unsigned(7 downto 0) := (others => '0');
    signal upd_active_pre2 : std_logic := '0';
    signal upd_bnd_pre2    : std_logic := '0';

    -- Line-buffer write ports.  Two physical buffers (each ping-pong):
    -- lb_bnd_* receives only boundary-edge stamps and is what the EOR
    -- walker reads.  lb_det_* receives detail-edge stamps (brows,
    -- centerlines) which should be visible but must NOT toggle EOR
    -- parity — otherwise open polylines like the brow arches would
    -- create black holes in the grid fill underneath them.
    signal lb_wr_en_bnd_r : std_logic := '0';
    signal lb_wr_en_det_r : std_logic := '0';
    signal lb_wr_addr_r   : unsigned(10 downto 0) := (others => '0');
    signal lb_wr_data_r   : std_logic := '0';
    signal lb_ab_r        : std_logic := '0';

    -- Inlined dual-bank 1-bit line buffer (4 EBRs total: bnd × 2 + det × 2)
    type t_lb is array (0 to 2047) of std_logic;
    signal lb_bnd_a : t_lb := (others => '0');
    signal lb_bnd_b : t_lb := (others => '0');
    signal lb_det_a : t_lb := (others => '0');
    signal lb_det_b : t_lb := (others => '0');
    signal lb_rd_addr     : unsigned(10 downto 0);
    signal lb_bnd_rd_a_r  : std_logic := '0';
    signal lb_bnd_rd_b_r  : std_logic := '0';
    signal lb_det_rd_a_r  : std_logic := '0';
    signal lb_det_rd_b_r  : std_logic := '0';
    signal lb_ab_d1       : std_logic := '0';  -- aligns mux with BRAM read latency
    signal lb_bnd_rd_data : std_logic;  -- boundary only — drives EOR walker
    signal lb_any_rd_data : std_logic;  -- boundary OR detail — drives display

    -- ---------------------------------------------------------------
    -- Post-line-buffer pipeline registers
    -- ---------------------------------------------------------------
    signal edge_hit_r : std_logic := '0';
    signal pix_y_r    : unsigned(9 downto 0) := (others => '0');
    signal pix_u_r    : unsigned(9 downto 0) := C_CHROMA_MID;
    signal pix_v_r    : unsigned(9 downto 0) := C_CHROMA_MID;

    -- Even-odd polygon fill state.  Walks the LB left-to-right per
    -- scanline; flips on every 0->1 transition, which is the parity
    -- rule for "am I inside the polygon".  When the wireframe contains
    -- nested closed shapes (silhouette, eyes, brows, mouth) the rule
    -- naturally fills the silhouette interior MINUS the feature
    -- interiors — no per-region bbox needed.
    signal eor_inside_r   : std_logic := '0';
    signal eor_prev_lb_r  : std_logic := '0';
    signal eor_inside_d_r : std_logic := '0';  -- aligned with edge_hit_r
    -- Saturating count of consecutive '1' pixels in the LB walk.  Used
    -- to recognise abnormally long solid runs (e.g. the mouth
    -- horizontal centerline, 124 px in one row) as non-crossings — we
    -- apply a cancelling toggle on the trailing 1->0 transition so the
    -- run doesn't break even-odd parity for the rest of the scanline.
    -- Normal edge crossings are 3-4 px and never trip the threshold.
    signal eor_run_len_r  : unsigned(4 downto 0) := (others => '0');

begin

    -- ---------------------------------------------------------------
    -- Input pipe + timing tracking
    -- ---------------------------------------------------------------
    timing_proc : process(clk)
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                hsync_falling_r <= '1';
            else
                hsync_falling_r <= '0';
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                vsync_falling_r <= '1';
            else
                vsync_falling_r <= '0';
            end if;

            if data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_x > to_unsigned(0, 12) then
                    max_x_r    <= pixel_x;
                    h_centre_r <= '0' & pixel_x(11 downto 1);
                end if;
                pixel_x <= (others => '0');
                pixel_y <= pixel_y + 1;
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                if pixel_y > to_unsigned(0, 12) then
                    max_y_r    <= pixel_y;
                    v_centre_r <= '0' & pixel_y(11 downto 1);
                end if;
                pixel_y <= (others => '0');

                -- K3 Palette
                pal_y_r <= C_PAL_Y(to_integer(unsigned(registers_in(2)(9 downto 7))));
                pal_u_r <= C_PAL_U(to_integer(unsigned(registers_in(2)(9 downto 7))));
                pal_v_r <= C_PAL_V(to_integer(unsigned(registers_in(2)(9 downto 7))));
                -- Fader Brightness, T7-T11 switches
                bright_r     <= unsigned(registers_in(7)(9 downto 2));
                bg_video_r   <= registers_in(6)(4);  -- T11
                grid_thick_r <= registers_in(6)(3);  -- T10: 2px grid lines
                grid_en_r    <= registers_in(6)(2);  -- T9 (dead grid)
                noise_streaks_r <= registers_in(6)(2);  -- P9 "Noise": allow streaks
                open_mouth_r <= registers_in(6)(0);  -- T7: mouth open(1)/closed(0)
                open_eyes_r  <= registers_in(6)(1);  -- T8: eyes  open(1)/closed(0)
                -- K1 Scale (latched, behavior deferred)
                scale_r    <= unsigned(registers_in(0));
                -- K2 Thickness: top 2 bits → 0..3, +1 → THICK 1..4
                thick_r    <= ("0" & unsigned(registers_in(1)(9 downto 8)))
                              + to_unsigned(1, 3);
                -- K4 Expression: top 3 bits, clamped to <= C_NUM_EXPR - 1
                if to_integer(unsigned(registers_in(3)(9 downto 7))) > C_NUM_EXPR - 1 then
                    expr_idx_r <= (others => '0');
                else
                    expr_idx_r <= unsigned(registers_in(3)(9 downto 7));
                end if;
                -- ---- Morph controls ----
                -- K1 = MOUTH (analog): 0% closed .. 100% open.  Current mesh is
                -- fully open (= neutral), so the close fraction = 31 - K1top5.
                mouth_frac_r <= to_unsigned(31, 5) - unsigned(registers_in(0)(9 downto 5));
                -- K4 = EXPRESSION blend across 5 (Neutral..Surprised): the knob
                -- spans positions 0..4; base = floor, frac t = fractional part.
                expr_base_r <= unsigned(registers_in(3)(9 downto 8));      -- 0..3 (A)
                expr_frac_r <= unsigned(registers_in(3)(7 downto 3));      -- 0..31 (t)
                -- P8 (open_eyes_r, registers_in(6)(1)) gates the eye-close morph.
                -- K5 = noise SPEED (frame-phase rate), K6 = noise SPREAD
                -- (amplitude level 0..7).  The grid these used to drive is
                -- gone.  noise_t_r advances once per frame by the speed, so the
                -- per-vertex Lissajous phase animates.
                noise_speed_r  <= unsigned(registers_in(4)(9 downto 5));
                noise_spread_r <= unsigned(registers_in(5)(9 downto 5));
                noise_t_r      <= noise_t_r + resize(noise_speed_r, 16);
            end if;
        end if;
    end process;

    -- ---------------------------------------------------------------
    -- Rasterizer FSM — one shared engine, time-multiplexed over edges.
    -- Writes go to bank selected by lb_ab_r and routed to bnd vs det
    -- buffer by lb_wr_en_bnd_r / lb_wr_en_det_r (set from bnd_held).
    -- ---------------------------------------------------------------
    raster_proc : process(clk)
        variable v_idx         : integer range 0 to C_NUM_EDGES - 1;
        variable v_ymin        : signed(12 downto 0);
        variable v_ymax        : signed(12 downto 0);
        variable v_xtop        : signed(11 downto 0);
        variable v_slope       : signed(11 downto 0);
        variable v_next_slope  : signed(11 downto 0);
        variable v_stamp_rel   : signed(12 downto 0);  -- relative to h_centre
        variable v_stamp_abs   : signed(13 downto 0);

        -- Grid masking
        variable v_x_rel       : signed(12 downto 0);
        variable v_abs_x       : unsigned(11 downto 0);
        variable v_abs_y       : unsigned(11 downto 0);
        variable v_in_head     : boolean;
        variable v_in_feature  : boolean;
        variable v_grid_show   : boolean;

        -- Inline helper: kick off the per-edge BRAM reads (current_x and
        -- active mesh slope) plus latch the simple flag/group state.
        -- count_m1_held / start_rel_held are NOT computed here — they
        -- depend on the slope, which arrives in act_slope_rd one cycle
        -- later.  R_STAMP_PRELOAD's wait cycle does the math from the
        -- registered act_slope_rd value, which keeps the heavy LUT-ROM
        -- mux off the per-cycle critical path entirely.
        procedure latch_held(idx : integer) is
        begin
            cx_rd_addr  <= to_unsigned(idx, 8);
            act_rd_addr <= to_unsigned(idx, 8);
            -- Only ever called from R_STAMP_SCAN2 with an edge taken from the
            -- active list, so it is active by construction — no need to index
            -- the (wide) active vector here (that 251:1 mux was the HD path).
            active_held <= '1';
            if C_EDGE_BND(idx) = 1 then
                bnd_held <= '1';
            else
                bnd_held <= '0';
            end if;
        end procedure;

        -- Compute count_m1_held / start_rel_held from a registered slope
        -- value (act_slope_rd, arriving 1 cycle after the BRAM read was
        -- kicked off by latch_held).  Stamp range covers the row-to-row
        -- slope sweep + K2 px padding on each side, except when |slope|
        -- already exceeds 2*K2 — then the K2 padding would just extend
        -- horizontals past their endpoints, so drop it.
        -- compute_counts stage a (PRELOAD2): decode the slope into its
        -- sign / magnitude / "needs K2 padding" form and register them.
        -- Also resolves the horizontal-edge EOR exclusion here (parallel
        -- to the magnitude path).  Splitting the original one-cycle
        -- compute into a+b shortened the HD critical path.
        procedure compute_counts_a is
            variable s_fp   : signed(15 downto 0);
            variable s_int  : signed(11 downto 0);
            variable abs_s  : unsigned(7 downto 0);
        begin
            s_fp   := signed(act_slope_rd);
            s_int  := resize(s_fp(15 downto 7), 12);
            if s_int >= to_signed(0, 12) then
                abs_s := to_unsigned(to_integer(s_int), 8);
                sneg_held <= '0';
            else
                abs_s := to_unsigned(-to_integer(s_int), 8);
                sneg_held <= '1';
            end if;
            s_int_held <= s_int;
            abs_s_held <= abs_s;
            if abs_s > (resize(thick_r, 8) sll 1) then
                pad_held <= '0';
            else
                pad_held <= '1';
            end if;
            -- Horizontal edge -> detail-only for EOR (closed-eye lids,
            -- chin baseline).  Parallel to the magnitude decode.
            if act_ymax_rd = act_ymin_rd then
                bnd_held <= '0';
            end if;
            -- Latch this edge's x-extent (aligned with act_slope_rd) for the
            -- stamp clamp.  Vertical edges carry a wide sentinel -> inert.  With
            -- noise on, the edge has been jittered out of its static extent, so
            -- latch a wide inert range here (keeps the noise_on test OFF the
            -- per-pixel stamp_valid critical path).
            if noise_on = '1' then
                xlo_held <= to_signed(-1024, 13);
                xhi_held <= to_signed(1023, 13);
            else
                xlo_held <= signed(act_xlo_rd);
                xhi_held <= signed(act_xhi_rd);
            end if;
        end procedure;

        -- compute_counts stage b (PRELOAD3): add the K2 padding, cap the
        -- count, derive start_rel and the center offset — all from the
        -- registered stage-a values, so this cycle is just an add + a
        -- compare.
        procedure compute_counts_b is
            variable count_raw   : unsigned(7 downto 0);
            variable v_thick_u   : unsigned(7 downto 0);
            variable v_thick_s   : signed(7 downto 0);
            variable v_start_rel : signed(7 downto 0);
            variable v_count_m1  : unsigned(7 downto 0);
            variable v_wide      : unsigned(10 downto 0);
            variable v_half      : unsigned(7 downto 0);
        begin
            v_thick_u := resize(thick_r, 8);
            v_thick_s := signed(resize(thick_r, 8));
            if pad_held = '1' then
                count_raw := abs_s_held + (v_thick_u sll 1);
                if sneg_held = '1' then
                    v_start_rel := resize(s_int_held, 8) - v_thick_s;
                else
                    v_start_rel := -v_thick_s;
                end if;
            elsif abs_s_held >= 1 and abs_s_held <= 31 then
                -- Near-horizontal (|slope| > 2*thick): a horizontal pad would
                -- run lengthwise and add no perpendicular thickness, so instead
                -- widen the run to ~thick*|slope| (vertical thickness ~thick) and
                -- centre it on the crossing.  Bounded to abs_s<=31 so the wide
                -- run fits the stamp cap; flatter edges (streaks) fall through.
                v_wide := thick_r * abs_s_held;
                v_half := resize(shift_right(v_wide - resize(abs_s_held, 11), 1), 8);
                count_raw := v_wide(7 downto 0);
                if sneg_held = '1' then
                    v_start_rel := resize(s_int_held, 8) - signed('0' & v_half(6 downto 0));
                else
                    v_start_rel := -signed('0' & v_half(6 downto 0));
                end if;
            else
                count_raw := abs_s_held;
                if sneg_held = '1' then
                    v_start_rel := resize(s_int_held, 8);
                else
                    v_start_rel := to_signed(0, 8);
                end if;
            end if;
            if count_raw > to_unsigned(MAX_STAMP_M1, 8) then
                v_count_m1 := to_unsigned(MAX_STAMP_M1, 8);
            else
                v_count_m1 := count_raw(7 downto 0);
            end if;
            -- INACTIVE edges contribute no stamps (stamp_valid is gated by
            -- active_held in R_STAMP), so there's no reason to iterate
            -- their full sweep — collapse them to a single cycle.  This
            -- matters a lot for the closed-eye edges: they're horizontal
            -- with a huge stamp count (~40) yet only active on one row, so
            -- without this they'd burn ~40 cycles/edge on EVERY line and
            -- overrun the per-line rasterizer budget (corrupting the
            -- render — the eye-dependent nose dot).
            if active_held = '0' then
                v_count_m1 := to_unsigned(0, 8);
            end if;
            start_rel_held  <= v_start_rel;
            count_m1_held   <= v_count_m1;
            -- center = sweep pixel at cur_x = -start_rel (start_rel <= 0).
            center_sub_held <= unsigned(resize(-v_start_rel, 8));
        end procedure;
    begin
        if rising_edge(clk) then
            -- Default each cycle: no write to either buffer
            lb_wr_en_bnd_r <= '0';
            lb_wr_en_det_r <= '0';

            v_y_target_r <= signed(resize(pixel_y, 13))
                        - signed(resize(v_centre_r, 13))
                        + to_signed(1, 13);

            -- Pre-add h_centre once per cycle (1-cycle stale, fine
            -- because h_centre_r only updates on hsync).
            clear_base_r <= resize(h_centre_r, 11) - to_unsigned(HEAD_HALF_W, 11);
            h_centre_s   <= signed(resize(h_centre_r, 14));

            -- Stage 0a': 1-cycle delay of pre signals so they reach
            -- Stage 0b at the same cycle that act_*_rd presents the
            -- BRAM-read data for the same edge.
            upd_valid_pre2  <= upd_valid_pre;
            upd_idx_pre2    <= upd_idx_pre;
            upd_active_pre2 <= upd_active_pre;
            upd_bnd_pre2    <= upd_bnd_pre;

            -- Stage 0b: BRAM read data lands here, aligned with the
            -- 2-stage-delayed control signals.  cx_rd_addr is driven
            -- from upd_idx_pre2 so cx_rd_data also lines up with
            -- upd_*_r2 in Stage 1 — but ONLY during R_CLEAR, since
            -- during R_STAMP latch_held owns cx_rd_addr (parked on the
            -- edge being stamped for the whole stamp loop).
            upd_valid_r  <= upd_valid_pre2;
            upd_idx_r    <= upd_idx_pre2;
            upd_active_r <= upd_active_pre2;
            upd_bnd_r    <= upd_bnd_pre2;
            if raster_state = R_CLEAR then
                cx_rd_addr <= upd_idx_pre2;
            end if;
            upd_ymin_r   <= signed(act_ymin_rd);
            upd_ymax_r   <= signed(act_ymax_rd);
            upd_xtop_r   <= signed(act_xtop_rd);
            upd_slope_r  <= signed(act_slope_rd);

            -- Pipeline shift: upd_*_r2 = upd_*_r delayed 1 cycle so
            -- Stage 1's use of upd_*_r2 aligns with cx_rd_data (which
            -- has BRAM's 1-cycle read latency).
            upd_valid_r2  <= upd_valid_r;
            upd_idx_r2    <= upd_idx_r;
            -- Horizontal edges (ymin==ymax) are 1 scanline tall; widen
            -- their activation window by ±thick_r so the line spans
            -- 2*thick+1 rows — matching the 2*thick+1 px HORIZONTAL width
            -- a near-vertical edge gets from its sweep, so both axes read
            -- equally thick.  Centred on the true row (symmetric pads).
            -- Done here (not in Stage 1) to keep the add/sub off the
            -- timing-critical v_y_target compare.  Ordinary edges pass
            -- through unchanged.
            if upd_ymin_r = upd_ymax_r then
                upd_is_horiz_r2 <= '1';
                upd_ymin_r2 <= upd_ymin_r - signed(resize(thick_r, 13));
                upd_ymax_r2 <= upd_ymax_r + signed(resize(thick_r, 13));
                if v_y_target_r = upd_ymin_r - signed(resize(thick_r, 13)) then
                    upd_is_ymin_r2 <= '1'; else upd_is_ymin_r2 <= '0'; end if;
                if v_y_target_r > upd_ymax_r + signed(resize(thick_r, 13)) then
                    upd_past_ymax_r2 <= '1'; else upd_past_ymax_r2 <= '0'; end if;
            else
                upd_is_horiz_r2 <= '0';
                upd_ymin_r2 <= upd_ymin_r;
                upd_ymax_r2 <= upd_ymax_r;
                if v_y_target_r = upd_ymin_r then
                    upd_is_ymin_r2 <= '1'; else upd_is_ymin_r2 <= '0'; end if;
                if v_y_target_r > upd_ymax_r then
                    upd_past_ymax_r2 <= '1'; else upd_past_ymax_r2 <= '0'; end if;
            end if;
            upd_xtop_r2   <= upd_xtop_r;
            upd_slope_r2  <= upd_slope_r;
            upd_active_r2 <= upd_active_r;
            upd_bnd_r2    <= upd_bnd_r;

            -- 1-cycle delayed copy of bnd_held for R_STAMP_PRELOAD's
            -- drain (which writes the previous edge's stamp using the
            -- previous edge's bnd routing).
            bnd_held_d1 <= bnd_held;

            case raster_state is

                when R_IDLE =>
                    if hsync_falling_r = '1' then
                        lb_ab_r       <= not lb_ab_r;
                        raster_state  <= R_CLEAR;
                        raster_cycle  <= (others => '0');
                        al_wr_addr    <= (others => '0');  -- start a fresh active list
                    end if;

                when R_CLEAR =>
                    -- Write 0 across the head's bbox of BOTH buffers.
                    -- Grid fill is computed on the DISPLAY side via an
                    -- even-odd scan of the boundary line buffer.
                    lb_wr_en_bnd_r <= '1';
                    lb_wr_en_det_r <= '1';
                    lb_wr_addr_r   <= clear_base_r + raster_cycle;
                    lb_wr_data_r   <= '0';

                    -- Stage 0a: issue active mesh BRAM read for this
                    -- slot, register the slot's control bits.  Mesh
                    -- data lands in upd_*_r on the next cycle (in
                    -- Stage 0b above), 1 cycle BRAM read latency
                    -- aligned with the BRAM read for current_x and the
                    -- pipeline shift to upd_*_r2.
                    if raster_cycle < to_unsigned(C_NUM_EDGES, 11) then
                        v_idx          := to_integer(raster_cycle(7 downto 0));
                        act_rd_addr    <= raster_cycle(7 downto 0);
                        upd_valid_pre  <= '1';
                        upd_idx_pre    <= raster_cycle(7 downto 0);
                        upd_active_pre <= active(v_idx);
                        if C_EDGE_BND(v_idx) = 1 then
                            upd_bnd_pre <= '1';
                        else
                            upd_bnd_pre <= '0';
                        end if;
                    else
                        upd_valid_pre <= '0';
                    end if;

                    if raster_cycle = to_unsigned(CLEAR_W - 1, 11) then
                        -- Active list is fully built; iterate it.  al_count
                        -- snapshots the build pointer; al_rd_addr starts the
                        -- read so al_rd_data = list[0] arrives in SCAN2.
                        raster_state    <= R_STAMP_SCAN;
                        raster_cycle    <= (others => '0');
                        al_count        <= al_wr_addr;
                        al_rd_addr      <= (others => '0');
                        stamp_sub       <= (others => '0');
                    else
                        raster_cycle <= raster_cycle + 1;
                    end if;

                when R_STAMP_SCAN =>
                    -- Issue the list read for al_rd_addr (al_rd_data lands
                    -- next cycle); drain when the list is exhausted.
                    if al_rd_addr >= al_count then
                        raster_state <= R_STAMP_DRAIN;
                    else
                        raster_state <= R_STAMP_SCAN2;
                    end if;

                when R_STAMP_SCAN2 =>
                    -- al_rd_data holds this active edge's index; latch it
                    -- (issues its mesh/cur_x reads, settling during PRELOAD)
                    -- and enter the stamp pipeline.
                    latch_held(to_integer(unsigned(al_rd_data)));
                    raster_state <= R_STAMP_PRELOAD;

                when R_STAMP_PRELOAD =>
                    -- First wait cycle.  Drain any pending stamp from
                    -- Stage B so the last stamp of the previous edge
                    -- still lands.  Use bnd_held_d1 (= the PREVIOUS
                    -- edge's bnd flag) — bnd_held was already latched
                    -- to the new edge by latch_held() in the prior
                    -- cycle.  BRAM reads for cx_rd_data and act_slope_rd
                    -- complete during this cycle; their outputs become
                    -- visible to PRELOAD2 next cycle.
                    v_stamp_abs := h_centre_s + resize(stamp_rel_r, 14);
                    if stamp_valid_r = '1' then
                        lb_wr_addr_r   <= unsigned(v_stamp_abs(10 downto 0));
                        lb_wr_data_r   <= '1';
                        lb_wr_en_bnd_r <= bnd_held_d1 and stamp_is_center_r;
                        lb_wr_en_det_r <= '1';
                    end if;
                    stamp_valid_r <= '0';
                    raster_state  <= R_STAMP_PRELOAD2;

                when R_STAMP_PRELOAD2 =>
                    -- act_slope_rd / act_ymin_rd / act_ymax_rd are valid
                    -- now; decode slope sign/magnitude/padding (stage a).
                    compute_counts_a;
                    raster_state <= R_STAMP_PRELOAD3;

                when R_STAMP_PRELOAD3 =>
                    -- Finish the count/start_rel/center from the
                    -- registered stage-a values (stage b).  R_STAMP
                    -- starts next cycle with everything ready.
                    compute_counts_b;
                    raster_state <= R_STAMP;

                when R_STAMP =>
                    -- Stage A: take the integer part of cur_x (Q9.7,
                    -- bits 15..7 = 9-bit signed).  Sign-extend to 13.
                    v_stamp_rel := resize(signed(cx_rd_data(15 downto 7)), 13)
                                 + resize(start_rel_held, 13)
                                 + signed(resize(stamp_sub, 13));
                    stamp_rel_r <= v_stamp_rel;
                    if active_held = '1'
                            and v_stamp_rel >= -to_signed(HEAD_HALF_W, 13)
                            and v_stamp_rel <  to_signed(HEAD_HALF_W, 13)
                            and v_stamp_rel >= xlo_held
                            and v_stamp_rel <= xhi_held then
                        stamp_valid_r <= '1';
                    else
                        stamp_valid_r <= '0';
                    end if;
                    -- Flag whether THIS sweep pixel is the edge's true
                    -- crossing (cur_x), carried to Stage B for the 1px
                    -- boundary-buffer write.
                    if stamp_sub = center_sub_held then
                        stamp_is_center_r <= '1';
                    else
                        stamp_is_center_r <= '0';
                    end if;

                    -- Stage B: drain previous cycle's registered values
                    -- into the LB.  Every sweep pixel goes to the DETAIL
                    -- buffer (visible thickness).  The BOUNDARY buffer
                    -- (read by the EOR fill) gets only the single center
                    -- pixel, and only for boundary edges — so thickness
                    -- never corrupts the fill parity.
                    v_stamp_abs := h_centre_s + resize(stamp_rel_r, 14);
                    if stamp_valid_r = '1' then
                        lb_wr_addr_r   <= unsigned(v_stamp_abs(10 downto 0));
                        lb_wr_data_r   <= '1';
                        lb_wr_en_bnd_r <= bnd_held and stamp_is_center_r;
                        lb_wr_en_det_r <= '1';
                    end if;

                    if stamp_sub = count_m1_held then
                        -- Advance to the next list entry; R_STAMP_SCAN reads
                        -- it (or drains when the list is exhausted).
                        stamp_sub <= (others => '0');
                        al_rd_addr <= al_rd_addr + 1;
                        raster_state <= R_STAMP_SCAN;
                    else
                        stamp_sub <= stamp_sub + 1;
                    end if;

                when R_STAMP_DRAIN =>
                    -- One extra cycle to flush the final stamp_rel_r
                    -- through Stage B.
                    v_stamp_abs := h_centre_s + resize(stamp_rel_r, 14);
                    if stamp_valid_r = '1' then
                        lb_wr_addr_r   <= unsigned(v_stamp_abs(10 downto 0));
                        lb_wr_data_r   <= '1';
                        lb_wr_en_bnd_r <= bnd_held and stamp_is_center_r;
                        lb_wr_en_det_r <= '1';
                    end if;
                    stamp_valid_r <= '0';
                    raster_state <= R_IDLE;

                    -- If hsync hits before STAMP completes, restart anyway —
                    -- a partial bank is worse than a fresh one.
                    if hsync_falling_r = '1' then
                        lb_ab_r       <= not lb_ab_r;
                        raster_state  <= R_CLEAR;
                        raster_cycle  <= (others => '0');
                        al_wr_addr    <= (others => '0');  -- fresh active list
                    end if;

            end case;

            -- Stage 1 of R_CLEAR's edge-update pipeline.  Uses the
            -- DOUBLE-registered upd_*_r2 so cx_rd_data (1-cycle delayed
            -- from cx_rd_addr) lines up with the right edge.
            cx_wr_en <= '0';
            al_wr_en <= '0';
            if upd_valid_r2 = '1' then
                if upd_is_ymin_r2 = '1' then
                    -- x_top is already Q9.7 in the mesh package — load
                    -- directly.  Mesh-side fixed-point lets us advance
                    -- x_top by exact 1-row slope steps when stripping
                    -- tips.
                    cx_wr_en   <= '1';
                    cx_wr_addr <= upd_idx_r2;
                    cx_wr_data <= std_logic_vector(resize(upd_xtop_r2, 16));
                    active(to_integer(upd_idx_r2)) <= '1';
                elsif upd_past_ymax_r2 = '1' then
                    active(to_integer(upd_idx_r2)) <= '0';
                elsif upd_active_r2 = '1' and upd_is_horiz_r2 = '0' then
                    -- Q9.7 add: per-row fractional accumulation.  Skipped
                    -- for horizontal edges — their slope field holds the
                    -- line WIDTH, so cur_x must stay parked on the left
                    -- end across all thick_r rows of the widened window.
                    cx_wr_en   <= '1';
                    cx_wr_addr <= upd_idx_r2;
                    cx_wr_data <= std_logic_vector(
                                    signed(cx_rd_data) + upd_slope_r2);
                end if;
                -- Append to the active list iff the edge is active on this
                -- line AFTER this update: it just activated (==ymin), or it
                -- was already active and hasn't passed ymax yet.
                if upd_is_ymin_r2 = '1'
                   or (upd_past_ymax_r2 = '0' and upd_active_r2 = '1') then
                    al_wr_en   <= '1';
                    al_wr_data <= std_logic_vector(upd_idx_r2);
                end if;
            end if;

            -- Global vsync reset of all active flags.  Edges whose
            -- y_max lies past v_target's actual frame range never see
            -- the v_target > y_max trigger and would otherwise stay
            -- active forever (e.g., when the simulator uses a smaller
            -- frame than the mesh was designed for).  Placed after the
            -- case so it overrides any per-edge assignment.
            if vsync_falling_r = '1' then
                active <= (others => '0');
            end if;

            -- Active-list BRAM: append on al_wr_en (advancing the build
            -- pointer), plus a continuous read so al_rd_data trails
            -- al_rd_addr by one cycle for the R_STAMP_SCAN/SCAN2 iterator.
            if al_wr_en = '1' then
                act_list_ram(to_integer(al_wr_addr)) <= al_wr_data;
                al_wr_addr <= al_wr_addr + 1;
            end if;
            al_rd_data <= act_list_ram(to_integer(al_rd_addr));
        end if;
    end process;

    -- Noise active whenever K6 spread is non-zero.
    noise_on <= '1' when noise_spread_r /= "00000" else '0';

    -- Morph active when the mouth is not fully open, eyes are closed, or an
    -- expression other than pure neutral is selected.  Frame-constant; used to
    -- gate the slope recompute in CP_WR.
    morph_active <= '1' when mouth_frac_r /= "00000" or open_eyes_r = '0'
                         or expr_base_r /= "00" or expr_frac_r /= "00000"
                    else '0';

    -- ---------------------------------------------------------------
    -- current_x BRAM — replaces what was an N_EDGES-entry register
    -- file.  iCE40 EBR in 256×16 mode (we use 128 entries × 12 bits).
    -- 1-cycle read latency, lined up with the rest of Stage 0's
    -- registered outputs.
    -- ---------------------------------------------------------------
    cur_x_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if cx_wr_en = '1' then
                current_x_ram(to_integer(cx_wr_addr)) <= cx_wr_data;
            end if;
            cx_rd_data <= current_x_ram(to_integer(cx_rd_addr));
        end if;
    end process;

    -- Active mesh BRAMs (1W1R, 1-cycle read latency).  cp_proc writes
    -- once per slot during vblank; render Stage 0 reads continuously.
    act_ymin_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_ymin_ram(to_integer(act_wr_addr)) <= act_ymin_wr;
            end if;
            act_ymin_rd <= act_ymin_ram(to_integer(act_rd_addr));
        end if;
    end process;

    act_ymax_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_ymax_ram(to_integer(act_wr_addr)) <= act_ymax_wr;
            end if;
            act_ymax_rd <= act_ymax_ram(to_integer(act_rd_addr));
        end if;
    end process;

    act_xtop_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_xtop_ram(to_integer(act_wr_addr)) <= act_xtop_wr;
            end if;
            act_xtop_rd <= act_xtop_ram(to_integer(act_rd_addr));
        end if;
    end process;

    act_slope_bram_proc : process(clk)
    begin
        if rising_edge(clk) then
            if act_wr_en = '1' then
                act_slope_ram(to_integer(act_wr_addr)) <= act_slope_wr;
            end if;
            act_slope_rd <= act_slope_ram(to_integer(act_rd_addr));
        end if;
    end process;

    -- x-extent ROMs read in lockstep with the mesh BRAMs (same address, same
    -- 1-cycle latency), so act_xlo_rd/act_xhi_rd line up with act_slope_rd in
    -- compute_counts_a.
    act_xext_rom_proc : process(clk)
    begin
        if rising_edge(clk) then
            act_xlo_rd <= C_XLO_ROM(to_integer(act_rd_addr));
            act_xhi_rd <= C_XHI_ROM(to_integer(act_rd_addr));
        end if;
    end process;

    -- Morph-delta ROMs read at cp_addr (registered, 1-cycle latency).  cp_addr
    -- is stable across an edge's cp_proc states, so the reads are valid well
    -- before CP_MLOAD consumes them.
    -- emap read tracks the current edge (cp_addr); the 6 vrom reads track
    -- vrom_addr, which cp_proc points at the top then the bottom vertex slot.
    morph_rom_proc : process(clk)
        variable va : integer range 0 to C_MORPH_NSLOT - 1;
    begin
        if rising_edge(clk) then
            emap_r <= C_EMAP_ROM(to_integer(cp_addr(7 downto 0)));
            va := to_integer(vrom_addr);
            vr_m <= C_VROM_M(va);  vr_e <= C_VROM_E(va);
            vr_h <= C_VROM_H(va);  vr_s <= C_VROM_S(va);
            vr_a <= C_VROM_A(va);  vr_u <= C_VROM_U(va);
        end if;
    end process;

    -- Copy engine.  Walks edges 0..N_EDGES-1 in vblank, resolving the
    -- (expr, group, om, oe) variant select once per slot and writing
    -- to the active mesh BRAMs.  3-stage pipeline so the heavy
    -- LUT-ROM mux is isolated from the variant case mux which is
    -- isolated from the BRAM write — each stage shallow.
    cp_proc : process(clk)
        variable ixt, iyt, ixb, iyb : signed(13 downto 0);
        variable hxt, hyt, hxb, hyb : signed(16 downto 0);
        variable tt   : signed(16 downto 0);
        variable dx_v       : signed(18 downto 0);
        variable span_v     : signed(13 downto 0);
        variable spc        : integer range 1 to 255;
        variable newrem     : unsigned(8 downto 0);
        variable v_sl       : signed(18 downto 0);
        variable nslope     : signed(15 downto 0);
        variable xa16       : signed(15 downto 0);
        variable idx        : integer range 0 to 255;
        -- morph
        variable va_t, va_b, vb_t, vb_b : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            act_wr_en <= '0';   -- default: no write this cycle

            case cp_state is
                when CP_IDLE =>
                    if vsync_falling_r = '1' then
                        cp_addr  <= (others => '0');
                        cp_state <= CP_LOAD;
                    end if;

                -- Read the (static, single-variant) mesh for this edge.
                when CP_LOAD =>
                    if cp_addr >= to_unsigned(C_NUM_EDGES, 9) then
                        cp_state <= CP_IDLE;
                    else
                        idx := to_integer(cp_addr(7 downto 0));
                        w_ymin  <= to_signed(f_y_min(to_integer(expr_idx_r), idx), 13);
                        w_ymax  <= to_signed(f_y_max(to_integer(expr_idx_r), idx), 13);
                        w_xtop  <= to_signed(f_x_top(to_integer(expr_idx_r), idx), 16);
                        w_slope <= to_signed(f_slope(to_integer(expr_idx_r), idx), 16);
                        w_xbot  <= to_signed(C_EDGE_X_BOT(idx), 16);
                        cp_state <= CP_HASH;
                    end if;

                -- CP_HASH: per-endpoint coordinate hash (ix = x>>7 px).
                -- hash_x = ix*5 + iy*3 ; hash_y = iy*5 - ix*3 (distinct per
                -- axis -> each point traces a small Lissajous orbit).  Stored
                -- in w_ph*; the frame phase is added in the next state so this
                -- cycle's add chain stays shallow.
                when CP_HASH =>
                    ixt := resize(shift_right(w_xtop, 7), 14);  iyt := resize(w_ymin, 14);
                    ixb := resize(shift_right(w_xbot, 7), 14);  iyb := resize(w_ymax, 14);
                    hxt := resize((shift_left(ixt,2)+ixt) + (shift_left(iyt,1)+iyt), 17);
                    hyt := resize((shift_left(iyt,2)+iyt) - (shift_left(ixt,1)+ixt), 17);
                    hxb := resize((shift_left(ixb,2)+ixb) + (shift_left(iyb,1)+iyb), 17);
                    hyb := resize((shift_left(iyb,2)+iyb) - (shift_left(ixb,1)+ixb), 17);
                    w_phxt <= hxt;  w_phyt <= hyt;  w_phxb <= hxb;  w_phyb <= hyb;
                    cp_state <= CP_PHASE;

                -- CP_PHASE: add the per-frame phase (animation).
                when CP_PHASE =>
                    tt := signed(resize(noise_t_r, 17));
                    w_phxt <= resize(w_phxt + tt, 17);  w_phyt <= resize(w_phyt + tt, 17);
                    w_phxb <= resize(w_phxb + tt, 17);  w_phyb <= resize(w_phyb + tt, 17);
                    cp_state <= CP_TRI;

                -- CP_TRI: sample the triangle waves (kept shallow; the noise
                -- spread multiply is folded into the shared multiply loop).
                -- Also point vrom_addr at this edge's TOP vertex slot (emap_r is
                -- valid by now — emap tracks cp_addr, stable since CP_LOAD).
                when CP_TRI =>
                    t_trxt <= tri10(w_phxt);  t_tryt <= tri10(w_phyt);
                    t_trxb <= tri10(w_phxb);  t_tryb <= tri10(w_phyb);
                    vrom_addr <= unsigned(emap_r(13 downto 8));   -- top slot
                    cp_state  <= CP_MR0;

                -- CP_MR0..MR2: dependent ROM reads have 2-cycle latency (set
                -- addr -> +1 addr visible -> +1 data out).  MR0 switches the
                -- shared vrom_addr to the BOTTOM slot; MR1 latches the TOP-vertex
                -- deltas (now valid from the addr set in CP_TRI); MR2 latches the
                -- BOTTOM-vertex deltas.  mrd_*t/*b then feed CP_MLOAD as before.
                when CP_MR0 =>
                    vrom_addr <= unsigned(emap_r(5 downto 0));    -- bottom slot
                    cp_state  <= CP_MR1;
                when CP_MR1 =>
                    mrd_mt <= vr_m;  mrd_et <= vr_e;  mrd_ht <= vr_h;
                    mrd_st <= vr_s;  mrd_at <= vr_a;  mrd_ut <= vr_u;
                    cp_state <= CP_MR2;
                when CP_MR2 =>
                    mrd_mb <= vr_m;  mrd_eb <= vr_e;  mrd_hb <= vr_h;
                    mrd_sb <= vr_s;  mrd_ab <= vr_a;  mrd_ub <= vr_u;
                    cp_state <= CP_MLOAD;

                -- CP_MLOAD: unpack the per-edge morph deltas.  Mouth + eye are
                -- direct (eye gated by P8 closed); the two K4-adjacent expression
                -- deltas A and (B-A) are selected from the 4 expr ROMs by the
                -- blend base index.  All px (8-bit signed), packed (dy<<8|dx).
                when CP_MLOAD =>
                    m_mxt <= signed(mrd_mt(7 downto 0));  m_myt <= signed(mrd_mt(15 downto 8));
                    m_mxb <= signed(mrd_mb(7 downto 0));  m_myb <= signed(mrd_mb(15 downto 8));
                    case expr_base_r is
                        when "00"   => va_t := (others=>'0'); va_b := (others=>'0');
                                       vb_t := mrd_ht; vb_b := mrd_hb;
                        when "01"   => va_t := mrd_ht; va_b := mrd_hb; vb_t := mrd_st; vb_b := mrd_sb;
                        when "10"   => va_t := mrd_st; va_b := mrd_sb; vb_t := mrd_at; vb_b := mrd_ab;
                        when others => va_t := mrd_at; va_b := mrd_ab; vb_t := mrd_ut; vb_b := mrd_ub;
                    end case;
                    m_dxt <= resize(signed(vb_t(7 downto 0)),9)  - resize(signed(va_t(7 downto 0)),9);
                    m_dyt <= resize(signed(vb_t(15 downto 8)),9) - resize(signed(va_t(15 downto 8)),9);
                    m_dxb <= resize(signed(vb_b(7 downto 0)),9)  - resize(signed(va_b(7 downto 0)),9);
                    m_dyb <= resize(signed(vb_b(15 downto 8)),9) - resize(signed(va_b(15 downto 8)),9);
                    -- init the running total = exprA + eye (eye gated by P8 closed)
                    if open_eyes_r = '0' then
                        mt_xt <= resize(signed(va_t(7 downto 0)),11)  + resize(signed(mrd_et(7 downto 0)),11);
                        mt_yt <= resize(signed(va_t(15 downto 8)),11) + resize(signed(mrd_et(15 downto 8)),11);
                        mt_xb <= resize(signed(va_b(7 downto 0)),11)  + resize(signed(mrd_eb(7 downto 0)),11);
                        mt_yb <= resize(signed(va_b(15 downto 8)),11) + resize(signed(mrd_eb(15 downto 8)),11);
                    else
                        mt_xt <= resize(signed(va_t(7 downto 0)),11);
                        mt_yt <= resize(signed(va_t(15 downto 8)),11);
                        mt_xb <= resize(signed(va_b(7 downto 0)),11);
                        mt_yb <= resize(signed(va_b(15 downto 8)),11);
                    end if;
                    m_cnt <= 0;
                    cp_state <= CP_MSEL;

                -- CP_MSEL: select this product's operands (mux only -> reg, so
                -- the mux stays off the multiply path).  One shared multiplier
                -- serves all three engines: 0..3 = noise tri*spread, 4..7 = expr
                -- blend t*(B-A), 8..11 = mouth frac*mouthΔ.
                when CP_MSEL =>
                    case m_cnt is
                        when 0  => mul_a_r <= resize(t_trxt,10);  when 1  => mul_a_r <= resize(t_tryt,10);
                        when 2  => mul_a_r <= resize(t_trxb,10);  when 3  => mul_a_r <= resize(t_tryb,10);
                        when 4  => mul_a_r <= resize(m_dxt,10);   when 5  => mul_a_r <= resize(m_dyt,10);
                        when 6  => mul_a_r <= resize(m_dxb,10);   when 7  => mul_a_r <= resize(m_dyb,10);
                        when 8  => mul_a_r <= resize(m_mxt,10);   when 9  => mul_a_r <= resize(m_myt,10);
                        when 10 => mul_a_r <= resize(m_mxb,10);   when others => mul_a_r <= resize(m_myb,10);
                    end case;
                    if    m_cnt <= 3 then mul_b_r <= signed(resize(noise_spread_r, 7));
                    elsif m_cnt <= 7 then mul_b_r <= signed(resize(expr_frac_r, 7));
                    else                  mul_b_r <= signed(resize(mouth_frac_r, 7));
                    end if;
                    cp_state <= CP_MDO;

                -- CP_MDO: one multiply, routed by m_cnt.  Noise (0..3) -> the
                -- offset o_d* (direct, shift 1/8); morph (4..11) -> accumulate
                -- into the running total mt_* (shift 5).
                when CP_MDO =>
                    case m_cnt is
                        when 0  => o_dxt <= resize(shift_right(mul_a_r * mul_b_r, 1), 18);
                        when 1  => o_dyt <= resize(shift_right(mul_a_r * mul_b_r, 8), 14);
                        when 2  => o_dxb <= resize(shift_right(mul_a_r * mul_b_r, 1), 18);
                        when 3  => o_dyb <= resize(shift_right(mul_a_r * mul_b_r, 8), 14);
                        when 4 | 8  => mt_xt <= mt_xt + resize(shift_right(mul_a_r * mul_b_r, 5), 11);
                        when 5 | 9  => mt_yt <= mt_yt + resize(shift_right(mul_a_r * mul_b_r, 5), 11);
                        when 6 | 10 => mt_xb <= mt_xb + resize(shift_right(mul_a_r * mul_b_r, 5), 11);
                        when others => mt_yb <= mt_yb + resize(shift_right(mul_a_r * mul_b_r, 5), 11);
                    end case;
                    if m_cnt = 11 then cp_state <= CP_OFF;
                    else m_cnt <= m_cnt + 1; cp_state <= CP_MSEL; end if;

                -- CP_OFF: move the endpoints by the noise offset AND the morph
                -- delta (x px<<7 to Q9.7, y px direct).
                when CP_OFF =>
                    w_nxt <= resize(w_xtop,18) + o_dxt + shift_left(resize(mt_xt,18),7);
                    w_nyt <= resize(w_ymin,14) + o_dyt + resize(mt_yt,14);
                    w_nxb <= resize(w_xbot,18) + o_dxb + shift_left(resize(mt_xb,18),7);
                    w_nyb <= resize(w_ymax,14) + o_dyb + resize(mt_yb,14);
                    cp_state <= CP_SORT;

                -- CP_SORT: re-order endpoints by y (jitter may flip them).
                -- Just the compare + mux; the subtracts that used to chain off
                -- this in the same cycle were the HD critical path, so they now
                -- get their own (registered) states below.
                when CP_SORT =>
                    if w_nyt <= w_nyb then
                        w_ya <= w_nyt;  w_xa <= w_nxt;  w_yb <= w_nyb;  w_xb <= w_nxb;
                    else
                        w_ya <= w_nyb;  w_xa <= w_nxb;  w_yb <= w_nyt;  w_xb <= w_nxt;
                    end if;
                    cp_state <= CP_FLOOR;

                -- CP_FLOOR: when P9 "Noise" is OFF (Smooth), enforce a min
                -- vertical span (extend ymax) so an edge can't collapse flat
                -- and draw a wide horizontal streak — it renders as a gentle
                -- shallow diagonal instead.  Its own state to keep the compare
                -- off the divisor-setup critical path.
                when CP_FLOOR =>
                    if noise_on = '1' and noise_streaks_r = '0'
                            and (w_yb - w_ya) < NOISE_SPAN_FLOOR then
                        w_yb <= w_ya + to_signed(NOISE_SPAN_FLOOR, 14);
                    end if;
                    cp_state <= CP_SPAN;

                -- CP_SPAN: span = yb-ya, clamped to [1,255] -> divisor.
                when CP_SPAN =>
                    span_v := w_yb - w_ya;
                    if    span_v <= 0  then spc := 1;
                    elsif span_v > 255 then spc := 255;
                    else  spc := to_integer(span_v); end if;
                    d_divisor <= to_unsigned(spc, 8);
                    cp_state  <= CP_DX;

                -- CP_DX: dx = xb-xa -> sign + |dx| (dividend); arm the divider.
                when CP_DX =>
                    dx_v := resize(w_xb, 19) - resize(w_xa, 19);
                    if dx_v < 0 then w_dxsign <= '1'; d_dividend <= unsigned(resize(-dx_v, 18));
                    else             w_dxsign <= '0'; d_dividend <= unsigned(resize( dx_v, 18));
                    end if;
                    d_rem  <= (others => '0');
                    d_quot <= (others => '0');
                    d_cnt  <= 17;
                    cp_state <= CP_DIV;

                -- Restoring division, one bit per cycle (shallow: a 9-bit
                -- compare + subtract).  18 steps -> |slope| in Q9.7.
                when CP_DIV =>
                    newrem := d_rem(7 downto 0) & d_dividend(17);
                    if newrem >= ('0' & d_divisor) then
                        d_rem  <= resize(newrem - ('0' & d_divisor), 9);
                        d_quot <= d_quot(16 downto 0) & '1';
                    else
                        d_rem  <= newrem;
                        d_quot <= d_quot(16 downto 0) & '0';
                    end if;
                    d_dividend <= d_dividend(16 downto 0) & '0';
                    if d_cnt = 0 then cp_state <= CP_WR;
                    else d_cnt <= d_cnt - 1; end if;

                -- Apply sign + saturate, pick jittered vs original slope
                -- (noise-off bypass is bit-exact), write the active mesh BRAM.
                when CP_WR =>
                    if w_dxsign = '1' then v_sl := -signed('0' & d_quot);
                    else                   v_sl :=  signed('0' & d_quot);
                    end if;
                    if    v_sl > 32767  then nslope := to_signed(32767, 16);
                    elsif v_sl < -32768 then nslope := to_signed(-32768, 16);
                    else  nslope := resize(v_sl, 16); end if;
                    if    w_xa > 32767  then xa16 := to_signed(32767, 16);
                    elsif w_xa < -32768 then xa16 := to_signed(-32768, 16);
                    else  xa16 := resize(w_xa, 16); end if;
                    act_wr_en   <= '1';
                    act_wr_addr <= cp_addr(7 downto 0);
                    act_ymin_wr <= std_logic_vector(resize(w_ya, 13));
                    act_ymax_wr <= std_logic_vector(resize(w_yb, 13));
                    act_xtop_wr <= std_logic_vector(xa16);
                    if noise_on = '1' or morph_active = '1' then
                        act_slope_wr <= std_logic_vector(nslope);
                    else
                        act_slope_wr <= std_logic_vector(w_slope);
                    end if;
                    cp_addr  <= cp_addr + 1;
                    cp_state <= CP_LOAD;
            end case;
        end if;
    end process;

    -- ---------------------------------------------------------------
    -- Inlined dual-bank 1-bit line buffers (4 iCE40 EBRs total).
    -- bnd buffer carries boundary stamps + clears; the EOR walker
    -- reads it.  det buffer carries detail stamps + clears; the
    -- display ORs in det so brows/centerlines are visible without
    -- breaking EOR parity.
    -- ---------------------------------------------------------------
    lb_rd_addr <= pixel_x(10 downto 0);

    lb_bnd_a_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_bnd_r = '1' and lb_ab_r = '0' then
                lb_bnd_a(to_integer(lb_wr_addr_r)) <= lb_wr_data_r;
            end if;
            lb_bnd_rd_a_r <= lb_bnd_a(to_integer(lb_rd_addr));
        end if;
    end process;

    lb_bnd_b_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_bnd_r = '1' and lb_ab_r = '1' then
                lb_bnd_b(to_integer(lb_wr_addr_r)) <= lb_wr_data_r;
            end if;
            lb_bnd_rd_b_r <= lb_bnd_b(to_integer(lb_rd_addr));
        end if;
    end process;

    lb_det_a_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_det_r = '1' and lb_ab_r = '0' then
                lb_det_a(to_integer(lb_wr_addr_r)) <= lb_wr_data_r;
            end if;
            lb_det_rd_a_r <= lb_det_a(to_integer(lb_rd_addr));
        end if;
    end process;

    lb_det_b_proc : process(clk)
    begin
        if rising_edge(clk) then
            if lb_wr_en_det_r = '1' and lb_ab_r = '1' then
                lb_det_b(to_integer(lb_wr_addr_r)) <= lb_wr_data_r;
            end if;
            lb_det_rd_b_r <= lb_det_b(to_integer(lb_rd_addr));
        end if;
    end process;

    -- ab_d1 captures the bank that was active when the read was issued
    lb_ab_d1_proc : process(clk)
    begin
        if rising_edge(clk) then
            lb_ab_d1 <= lb_ab_r;
        end if;
    end process;

    -- ab='0' means writing A → display reads B (and vice-versa)
    lb_bnd_rd_data <= lb_bnd_rd_a_r when lb_ab_d1 = '1' else lb_bnd_rd_b_r;
    lb_any_rd_data <= (lb_bnd_rd_a_r or lb_det_rd_a_r) when lb_ab_d1 = '1'
                      else (lb_bnd_rd_b_r or lb_det_rd_b_r);

    -- ---------------------------------------------------------------
    -- Post-line-buffer pipeline
    -- ---------------------------------------------------------------
    pix_proc : process(clk)
        variable v_y_scaled : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            edge_hit_r <= lb_any_rd_data;
            v_y_scaled := pal_y_r * bright_r;
            pix_y_r <= v_y_scaled(17 downto 8);
            pix_u_r <= pal_u_r;
            pix_v_r <= pal_v_r;
        end if;
    end process;

    -- Even-odd fill walker.  Operates on lb_bnd_rd_data (the raw bit
    -- coming out of the line buffer, BEFORE the edge_hit_r register).
    -- Reset at hsync; toggle on every rising 0->1 transition.
    eor_proc : process(clk)
    begin
        if rising_edge(clk) then
            if hsync_falling_r = '1' then
                eor_inside_r   <= '0';
                eor_prev_lb_r  <= '0';
                eor_inside_d_r <= '0';
                eor_run_len_r  <= (others => '0');
            else
                if lb_bnd_rd_data = '1' then
                    if eor_prev_lb_r = '0' then
                        -- 0->1 rising: classic EOR toggle.
                        eor_inside_r  <= not eor_inside_r;
                        eor_run_len_r <= to_unsigned(1, 5);
                    elsif eor_run_len_r /= "11111" then
                        eor_run_len_r <= eor_run_len_r + 1;
                    end if;
                else
                    -- lb_bnd_rd_data = '0'
                    if eor_prev_lb_r = '1' and eor_run_len_r = "11111" then
                        -- Trailing edge of a long solid run: cancel the
                        -- earlier toggle so the run nets to 0 crossings.
                        eor_inside_r <= not eor_inside_r;
                    end if;
                    eor_run_len_r <= (others => '0');
                end if;
                eor_prev_lb_r  <= lb_bnd_rd_data;
                -- Delay the "inside" flag by one cycle so it lines up
                -- with edge_hit_r at the output mux.
                eor_inside_d_r <= eor_inside_r;
            end if;
        end if;
    end process;

    -- ---------------------------------------------------------------
    -- Output mux
    -- ---------------------------------------------------------------
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;
    data_out.avid    <= pipe(LATENCY - 1).avid;

    -- Per-pixel output-side counter that tracks active-video pixel
    -- position (resets at hsync).  Used to derive the grid pattern.
    -- We can't reuse pixel_x because that one ticks at data_in time,
    -- not at the output-stream pipeline depth.
    pixel_out_counter : block
        signal grid_hit  : std_logic;
        signal show_pix  : std_logic;
        signal grid_xp   : unsigned(11 downto 0);
        signal grid_mask : unsigned(6 downto 0);
        signal grid_lmask : unsigned(6 downto 0);
    begin
        -- Use the input-side pixel_x / pixel_y counters (already kept
        -- by timing_proc) for the grid pattern.  pixel_x / _y align
        -- naturally with the silhouette + EOR walker, which run on the
        -- same input-side clock.
        -- K5 grid_per_r selects the mask width (power-of-2 periods so
        -- the mod-N is a cheap bit-slice).  K6 drives grid_phs_acc_r,
        -- an auto-scrolling phase; its integer part (bits 12..6) shifts
        -- the vertical-column grid horizontally without affecting the
        -- horizontal-row spacing.
        grid_xp <= pixel_x - resize(grid_phs_acc_r(12 downto 6), 12);
        with grid_per_r select
            grid_mask <= "0001111" when "00",  -- period 16
                         "0011111" when "01",  -- period 32
                         "0111111" when "10",  -- period 64
                         "1111111" when others;-- period 128
        -- A line pixel is where the position within the period is 0
        -- (thin = 1px).  For thick (2px, T10) we clear bit 0 of the
        -- period mask, so positions 0 AND 1 both test as "on the line"
        -- — one comparison per axis instead of two.  Thicker lines
        -- survive chroma subsampling better, so the grid colour reads
        -- closer to the bold outline.
        grid_lmask <= (grid_mask and "1111110") when grid_thick_r = '1'
                      else grid_mask;
        grid_hit <= '1' when (grid_xp(6 downto 0)  and grid_lmask) = "0000000"
                          or (pixel_y(6 downto 0) and grid_lmask) = "0000000"
                    else '0';
        -- Grid fill dropped: the dense wireframe carries the detail itself,
        -- so the output is just the outline (no EOR grid).
        show_pix <= edge_hit_r;

        data_out.y <= std_logic_vector(pix_y_r) when show_pix = '1'
                      else pipe(LATENCY - 1).y when bg_video_r = '1'
                      else (others => '0');
        data_out.u <= std_logic_vector(pix_u_r) when show_pix = '1'
                      else pipe(LATENCY - 1).u when bg_video_r = '1'
                      else std_logic_vector(C_CHROMA_MID);
        data_out.v <= std_logic_vector(pix_v_r) when show_pix = '1'
                      else pipe(LATENCY - 1).v when bg_video_r = '1'
                      else std_logic_vector(C_CHROMA_MID);
    end block;

end architecture bishop;
