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
--   registers_in(5) = K6 Grid phase  (top 7 bits → 0..127 px horizontal shift)
--   registers_in(6) = Switches      (bit0 = T7 Mouth open, bit1 = T8 Eyes open,
--                                    bit2 = T9 grid on/off, bit4 = T11 BG-Video)
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
    constant HEAD_HALF_W : natural := 200;  -- half-width of head bbox in pixels
    constant HEAD_HALF_H : natural := 300;  -- half-height for the grid mask
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
    -- K1 Scale — latched, behavior deferred to a future milestone.
    signal scale_r    : unsigned(9 downto 0) := (others => '0');
    -- K2 Thickness — stored as the actual THICK value (1..4).
    signal thick_r    : unsigned(2 downto 0) := to_unsigned(1, 3);
    -- K4 Expression — selects which row of the C_EDGE_* mesh tables to
    -- index.  Clamped to [0, C_NUM_EXPR-1] in the vsync latch.
    signal expr_idx_r : unsigned(2 downto 0) := (others => '0');
    -- K5 Grid period (00=16, 01=32, 10=64, 11=128).
    signal grid_per_r : unsigned(1 downto 0) := "01";   -- default 32 px (v2.1 spacing)
    -- K6 Grid horizontal phase (subtracted from pixel_x before masking).
    signal grid_phs_r : unsigned(6 downto 0) := to_unsigned(8, 7);  -- default = old hardcoded 8
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
    type t_cur_x_ram is array (0 to 127) of std_logic_vector(15 downto 0);
    signal current_x_ram : t_cur_x_ram := (others => (others => '0'));
    signal cx_rd_addr  : unsigned(6 downto 0) := (others => '0');
    signal cx_rd_data  : std_logic_vector(15 downto 0);
    signal cx_wr_addr  : unsigned(6 downto 0) := (others => '0');
    signal cx_wr_data  : std_logic_vector(15 downto 0) := (others => '0');
    signal cx_wr_en    : std_logic := '0';

    signal active    : std_logic_vector(0 to C_NUM_EDGES - 1)
                     := (others => '0');

    -- ---------------------------------------------------------------
    -- Rasterizer FSM
    -- ---------------------------------------------------------------
    type t_raster_state is (R_IDLE, R_CLEAR, R_STAMP_PRELOAD, R_STAMP, R_STAMP_DRAIN);
    signal raster_state    : t_raster_state := R_IDLE;
    signal raster_cycle    : unsigned(10 downto 0) := (others => '0');
    signal stamp_edge_idx  : unsigned(6 downto 0)  := (others => '0');
    -- Wide enough for max |slope| + 2*THICK.
    signal stamp_sub       : unsigned(6 downto 0)  := (others => '0');

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
    signal start_rel_held : signed(6 downto 0)  := (others => '0');
    signal count_m1_held  : unsigned(6 downto 0) := (others => '0');

    -- Stamp pipeline registers (Stage A → Stage B).  Splits the
    -- (cur_x + start_rel + sub) + h_centre add chain into two cycles.
    -- Stage A registers v_stamp_rel + valid, Stage B does the +h_centre
    -- and writes the LB.  Adds 1 cycle of latency, drained in
    -- R_STAMP_DRAIN.
    signal stamp_rel_r    : signed(12 downto 0) := (others => '0');
    signal stamp_valid_r  : std_logic := '0';

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
    signal upd_idx_r    : unsigned(6 downto 0)  := (others => '0');
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
    signal upd_idx_r2    : unsigned(6 downto 0)  := (others => '0');
    signal upd_ymin_r2   : signed(12 downto 0)   := (others => '0');
    signal upd_ymax_r2   : signed(12 downto 0)   := (others => '0');
    signal upd_xtop_r2   : signed(15 downto 0)   := (others => '0');  -- Q9.7
    signal upd_slope_r2  : signed(15 downto 0)   := (others => '0');  -- Q9.7
    signal upd_active_r2 : std_logic := '0';
    signal upd_bnd_r2    : std_logic := '0';

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
                -- Fader Brightness, T7/T8/T9/T11 switches
                bright_r     <= unsigned(registers_in(7)(9 downto 2));
                bg_video_r   <= registers_in(6)(4);
                grid_en_r    <= registers_in(6)(2);
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
                -- K5 Grid period
                grid_per_r <= unsigned(registers_in(4)(9 downto 8));
                -- K6 Grid horizontal phase (top 7 bits → 0..127)
                grid_phs_r <= unsigned(registers_in(5)(9 downto 3));
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
        variable v_next_idx    : integer range 0 to C_NUM_EDGES - 1;
        variable v_y_target    : signed(12 downto 0);
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

        -- Inline helper: pack edge constants into the held regs.
        -- Held regs are simple lookups (no arithmetic in this latch);
        -- the actual add chain lives in the per-cycle R_STAMP body
        -- so the latch cycle stays short.
        procedure latch_held(idx : integer) is
            variable s_fp      : signed(15 downto 0);   -- Q9.7 slope
            variable s_int     : signed(11 downto 0);   -- floor(s_fp / 128), sign-ext to 12
            variable count_raw : unsigned(7 downto 0);
            variable v_thick_u : unsigned(7 downto 0);  -- thick_r resized
            variable v_thick_s : signed(6 downto 0);    -- thick_r as signed
        begin
            cx_rd_addr  <= to_unsigned(idx, 7);
            active_held <= active(idx);
            if C_EDGE_BND(idx) = 1 then
                bnd_held <= '1';
            else
                bnd_held <= '0';
            end if;
            s_fp      := to_signed(f_slope_sel(to_integer(expr_idx_r), idx,
                                               open_mouth_r, open_eyes_r), 16);
            -- Q9.7: integer part is bits 15..7 (9 bits signed).
            -- Sign-extend to 12 bits to match the existing s_int width.
            s_int     := resize(s_fp(15 downto 7), 12);
            v_thick_u := resize(thick_r, 8);
            v_thick_s := signed(resize(thick_r, 7));
            if s_int >= to_signed(0, 12) then
                count_raw      := to_unsigned(to_integer(s_int), 8)
                                + (v_thick_u sll 1);    -- 2 * thick_r
                start_rel_held <= -v_thick_s;
            else
                count_raw      := to_unsigned(-to_integer(s_int), 8)
                                + (v_thick_u sll 1);
                start_rel_held <= resize(s_int, 7) - v_thick_s;
            end if;
            if count_raw > to_unsigned(MAX_STAMP_M1, 8) then
                count_m1_held <= to_unsigned(MAX_STAMP_M1, 7);
            else
                count_m1_held <= count_raw(6 downto 0);
            end if;
        end procedure;
    begin
        if rising_edge(clk) then
            -- Default each cycle: no write to either buffer
            lb_wr_en_bnd_r <= '0';
            lb_wr_en_det_r <= '0';

            v_y_target := signed(resize(pixel_y, 13))
                        - signed(resize(v_centre_r, 13))
                        + to_signed(1, 13);

            -- Pre-add h_centre once per cycle (1-cycle stale, fine
            -- because h_centre_r only updates on hsync).
            clear_base_r <= resize(h_centre_r, 11) - to_unsigned(HEAD_HALF_W, 11);
            h_centre_s   <= signed(resize(h_centre_r, 14));

            -- Pipeline shift: upd_*_r2 = upd_*_r delayed 1 cycle so
            -- Stage 1's use of upd_*_r2 aligns with cx_rd_data (which
            -- has BRAM's 1-cycle read latency).
            upd_valid_r2  <= upd_valid_r;
            upd_idx_r2    <= upd_idx_r;
            upd_ymin_r2   <= upd_ymin_r;
            upd_ymax_r2   <= upd_ymax_r;
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
                    end if;

                when R_CLEAR =>
                    -- Write 0 across the head's bbox of BOTH buffers.
                    -- Grid fill is computed on the DISPLAY side via an
                    -- even-odd scan of the boundary line buffer.
                    lb_wr_en_bnd_r <= '1';
                    lb_wr_en_det_r <= '1';
                    lb_wr_addr_r   <= clear_base_r + raster_cycle;
                    lb_wr_data_r   <= '0';

                    -- Stage 0: lookup mesh constants + issue BRAM read
                    -- for current_x.  Registered into upd_*_r and
                    -- cx_rd_data for use one cycle later by Stage 1.
                    if raster_cycle < to_unsigned(C_NUM_EDGES, 11) then
                        v_idx        := to_integer(raster_cycle(6 downto 0));
                        upd_valid_r  <= '1';
                        upd_idx_r    <= raster_cycle(6 downto 0);
                        upd_ymin_r   <= to_signed(f_y_min_sel(to_integer(expr_idx_r), v_idx,
                                                              open_mouth_r, open_eyes_r), 13);
                        upd_ymax_r   <= to_signed(f_y_max_sel(to_integer(expr_idx_r), v_idx,
                                                              open_mouth_r, open_eyes_r), 13);
                        upd_xtop_r   <= to_signed(f_x_top_sel(to_integer(expr_idx_r), v_idx,
                                                              open_mouth_r, open_eyes_r), 16);
                        upd_slope_r  <= to_signed(f_slope_sel(to_integer(expr_idx_r), v_idx,
                                                              open_mouth_r, open_eyes_r), 16);
                        upd_active_r <= active(v_idx);
                        if C_EDGE_BND(v_idx) = 1 then
                            upd_bnd_r <= '1';
                        else
                            upd_bnd_r <= '0';
                        end if;
                        cx_rd_addr   <= raster_cycle(6 downto 0);
                    else
                        upd_valid_r  <= '0';
                    end if;

                    if raster_cycle = to_unsigned(CLEAR_W - 1, 11) then
                        -- Transition through R_STAMP_PRELOAD so the
                        -- BRAM read of edge 0's current_x (issued in
                        -- latch_held) has a cycle to settle before
                        -- Stage A reads cx_rd_data.
                        raster_state    <= R_STAMP_PRELOAD;
                        raster_cycle    <= (others => '0');
                        stamp_edge_idx  <= (others => '0');
                        stamp_sub       <= (others => '0');
                        latch_held(0);
                    else
                        raster_cycle <= raster_cycle + 1;
                    end if;

                when R_STAMP_PRELOAD =>
                    -- One wait cycle for cx_rd_data to settle after a
                    -- cx_rd_addr change at the previous edge transition.
                    -- Drain any pending stamp from Stage B too, so the
                    -- last stamp of the previous edge still lands.
                    -- Use bnd_held_d1 (= the PREVIOUS edge's bnd flag) —
                    -- bnd_held was already latched to the new edge by
                    -- latch_held() in the prior cycle, so it would mis-
                    -- route this drained stamp.
                    v_stamp_abs := h_centre_s + resize(stamp_rel_r, 14);
                    if stamp_valid_r = '1' then
                        lb_wr_addr_r   <= unsigned(v_stamp_abs(10 downto 0));
                        lb_wr_data_r   <= '1';
                        lb_wr_en_bnd_r <= bnd_held_d1;
                        lb_wr_en_det_r <= not bnd_held_d1;
                    end if;
                    stamp_valid_r <= '0';
                    raster_state  <= R_STAMP;

                when R_STAMP =>
                    -- Stage A: take the integer part of cur_x (Q9.7,
                    -- bits 15..7 = 9-bit signed).  Sign-extend to 13.
                    v_stamp_rel := resize(signed(cx_rd_data(15 downto 7)), 13)
                                 + resize(start_rel_held, 13)
                                 + signed(resize(stamp_sub, 13));
                    stamp_rel_r <= v_stamp_rel;
                    if active_held = '1'
                            and v_stamp_rel >= -to_signed(HEAD_HALF_W, 13)
                            and v_stamp_rel <  to_signed(HEAD_HALF_W, 13) then
                        stamp_valid_r <= '1';
                    else
                        stamp_valid_r <= '0';
                    end if;

                    -- Stage B: drain previous cycle's registered values
                    -- into the LB.  This is just one 14-bit add per
                    -- cycle, off the wider Stage A path.
                    v_stamp_abs := h_centre_s + resize(stamp_rel_r, 14);
                    if stamp_valid_r = '1' then
                        lb_wr_addr_r   <= unsigned(v_stamp_abs(10 downto 0));
                        lb_wr_data_r   <= '1';
                        lb_wr_en_bnd_r <= bnd_held;
                        lb_wr_en_det_r <= not bnd_held;
                    end if;

                    if stamp_sub = count_m1_held then
                        stamp_sub <= (others => '0');
                        if stamp_edge_idx = to_unsigned(C_NUM_EDGES - 1, 7) then
                            raster_state <= R_STAMP_DRAIN;
                        else
                            stamp_edge_idx <= stamp_edge_idx + 1;
                            -- Latch held values for the NEXT edge and
                            -- transition through PRELOAD so the new
                            -- BRAM read has a cycle to settle.
                            v_next_idx := to_integer(stamp_edge_idx) + 1;
                            latch_held(v_next_idx);
                            raster_state <= R_STAMP_PRELOAD;
                        end if;
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
                        lb_wr_en_bnd_r <= bnd_held;
                        lb_wr_en_det_r <= not bnd_held;
                    end if;
                    stamp_valid_r <= '0';
                    raster_state <= R_IDLE;

                    -- If hsync hits before STAMP completes, restart anyway —
                    -- a partial bank is worse than a fresh one.
                    if hsync_falling_r = '1' then
                        lb_ab_r       <= not lb_ab_r;
                        raster_state  <= R_CLEAR;
                        raster_cycle  <= (others => '0');
                    end if;

            end case;

            -- Stage 1 of R_CLEAR's edge-update pipeline.  Uses the
            -- DOUBLE-registered upd_*_r2 so cx_rd_data (1-cycle delayed
            -- from cx_rd_addr) lines up with the right edge.
            cx_wr_en <= '0';
            if upd_valid_r2 = '1' then
                if v_y_target = upd_ymin_r2 then
                    -- x_top is already Q9.7 in the mesh package — load
                    -- directly.  Mesh-side fixed-point lets us advance
                    -- x_top by exact 1-row slope steps when stripping
                    -- tips.
                    cx_wr_en   <= '1';
                    cx_wr_addr <= upd_idx_r2;
                    cx_wr_data <= std_logic_vector(resize(upd_xtop_r2, 16));
                    active(to_integer(upd_idx_r2)) <= '1';
                elsif v_y_target > upd_ymax_r2 then
                    active(to_integer(upd_idx_r2)) <= '0';
                elsif upd_active_r2 = '1' then
                    -- Q9.7 add: per-row fractional accumulation
                    cx_wr_en   <= '1';
                    cx_wr_addr <= upd_idx_r2;
                    cx_wr_data <= std_logic_vector(
                                    signed(cx_rd_data) + upd_slope_r2);
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
        end if;
    end process;

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
    begin
        -- Use the input-side pixel_x / pixel_y counters (already kept
        -- by timing_proc) for the grid pattern.  pixel_x / _y align
        -- naturally with the silhouette + EOR walker, which run on the
        -- same input-side clock.
        -- K5 grid_per_r selects the mask width (power-of-2 periods so
        -- the mod-N is a cheap bit-slice).  K6 grid_phs_r shifts the
        -- vertical-column grid horizontally without affecting the
        -- horizontal-row spacing.
        grid_xp <= pixel_x - resize(grid_phs_r, 12);
        with grid_per_r select
            grid_mask <= "0001111" when "00",  -- period 16
                         "0011111" when "01",  -- period 32
                         "0111111" when "10",  -- period 64
                         "1111111" when others;-- period 128
        grid_hit <= '1' when (grid_xp(6 downto 0)  and grid_mask) = "0000000"
                          or (pixel_y(6 downto 0) and grid_mask) = "0000000"
                    else '0';
        show_pix <= edge_hit_r
                    or (grid_en_r and eor_inside_d_r and grid_hit);

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
