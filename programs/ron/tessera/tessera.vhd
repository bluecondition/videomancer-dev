-- tessera.vhd  (v3.3 — two tumbling triangles + T10 size-var + T7 multi-color)
--
-- Two triangles, tumbling about Z-axis, falling/rising through the frame.
-- Triangle 1 is a vertical offset of triangle 0 (DELTA_Y = 256 internal px),
-- computed via the "offset trick": edge_init_t1_i = edge_init_t0_i + 256 * dx_i.
-- With 256 a power of 2, the multiply collapses to a shift+add, so triangle 1
-- adds only DDA accumulators + per-pixel test (~840 LC) — no extra multipliers.
-- T10 (Size-Varied) wired: 4-step ±R/8 cycle on frame_count(6:5).
-- T7 (Outline Color = Multi) wired: triangle 1 outline gets +90° hue shift.
--
-- Status of controls (10 of 12 wired):
--   K1 outline hue, K2 fill hue, K3 bg hue, K4 size, Fader 12 video mix,
--   T7 outline multi, T8 filled, T9 outline invert, T10 size varied,
--   T11 direction.
--   STILL DEFERRED: K5 density (single-triangle gate), K6 X/Y rot axes
--                  (only Z implemented).
--
-- Resource usage (iCE40 HX4K, HD timing):
--   ICESTORM_LC : 7576 / 7680  (98%)
--   ICESTORM_RAM:    0 /   32  (0%)
--   Fmax        : 84.15 MHz at 74.25 MHz target  (PASS)
--
-- Iteration history: v3.0 single triangle baseline → v3.1 T10 wired
--                  → v3.2 N=2 via offset trick → v3.3 T7 multi-color outline.
-- Architecture chosen for iCE40 HX4K at HD pixel clock (74.25 MHz):
--   * Rotation by CORDIC — 7 unrolled iteration stages with C_VW=11
--     vertex precision.  Pre-rotation by quadrant brings the residual
--     angle into CORDIC's natural range.  No sin/cos LUT.
--   * Vertex pre-scale uses shift-add chains for fixed-coefficient R*K,
--     R*sin60*K, R*cos60*K — no multipliers.
--   * Edge initial value uses ONE shared inferred multiplier, sequenced
--     by an 8-phase FSM that issues 6 operand pairs and accumulates the
--     products into the 3 edge_init values.
--   * Per-pixel rasterization is DDA (edge-walker): three 29-bit adder
--     accumulators advance by dy_i per pixel and -dx_i per line.  No
--     per-pixel multiplies.
--   * Pipeline split into many short stages; latency does not matter (we
--     only need to align sync correctly).
--
-- Pipeline summary (LATENCY = 11 clocks):
--   1 clk : timing_gen
--   1 clk : pixel_counter + edge_pix DDA accumulator update
--   1 clk : P1 — latch edge_pix into pipeline register (+ frame-side
--            edge_thr passthrough)
--   1 clk : P2 — |edge_i| absolute value computation
--   1 clk : P3 — outline test: |edge_i| < threshold_i + sign extraction
--   1 clk : P4 — combine outline + inside flags
--   1 clk : P5 — color mux (bg / outline / fill / video-fill / invert)
--   1 clk : P6 — register selected YUV
--   1 clk : P7 — register YUV again (extra slack — moves output further
--            from the mux path so chroma/luma routing isn't on the timing
--            critical path)
--   2 clk : IO alignment
--
-- Frame-side path (settles during vblank, no per-pixel dependency):
--   FS0  : at vsync rising edge, latch frame_count, target_angle, radius,
--          y_pos, tri_cx/cy
--   FS1  : pre-scale R by CORDIC K-factor compensation constants to
--          produce initial vertex coordinates (3 pipelined multiplies)
--   FS2  : quadrant pre-rotation (sign flips + axis swaps to bring
--          target angle into [0, 90°])
--   FS3-FS12 : 10 CORDIC iteration stages, each a single shift + 3 adds
--               (one for each of 3 vertices, plus an angle-residual adder)
--   FS13 : translate rotated local vertices to world coords (+cx, +cy)
--   FS14 : compute edge deltas (dx_i, dy_i) — subtractions
--   FS15 : compute |dx_i|, |dy_i| for Manhattan length
--   FS16 : compute Manhattan length and outline threshold (just shifts —
--          C_OUTLINE_PX is 4, so threshold = length << 2)
--   FS17 : pre-compute multiplier operands: -v.x and pass dy, dx, v.y
--   FS18 : two 13-bit multiplies per edge (split into FS18a/FS18b to
--          shorten the combinational multiplier critical path)
--   FS19 : sum the two partial products → edge_init
--   FS20 : snapshot edge_init into edge_row state
--   After FS20, edge_row holds the value of edge_i at (px=0, py=0).
--   Snapshot happens on a single-pulse 'frame_setup_done' signal driven
--   by a small counter chain — see p_frame_setup_done.
--
-- Per-pixel DDA:
--   At vsync_start    : (deferred — actual snapshot when FS20 completes)
--   At avid_start     : edge_pix(i) <= edge_row(i);
--                        edge_row(i) <= edge_row(i) - dx_i;
--   At active pixel   : edge_pix(i) <= edge_pix(i) + dy_i;
--
-- Register map:
--   registers_in(0) = Outline Hue       (10b)
--   registers_in(1) = Fill Hue          (10b)
--   registers_in(2) = BG Hue            (10b)
--   registers_in(3) = Size              (10b)
--   registers_in(4) = Density           (10b, deferred)
--   registers_in(5) = Rot Axis          (10b, top 2 bits select X/Y/Z/Rand;
--                                        v2 only implements Z — TODO)
--   registers_in(6) = Switches          (b0=OutlineMulti b1=Filled b2=Invert
--                                        b3=SizeVaried b4=Rise)
--   registers_in(7) = Video Mix (Fader) (10b)
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture tessera of program_top is

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_MAX_VAL    : unsigned(9 downto 0) := to_unsigned(1023, 10);

    constant C_BG_LUMA      : unsigned(9 downto 0) := to_unsigned(256, 10);
    constant C_FILL_LUMA    : unsigned(9 downto 0) := to_unsigned(700, 10);
    constant C_OUTLINE_LUMA : unsigned(9 downto 0) := to_unsigned(900, 10);

    constant C_OUTLINE_PX   : integer := 2;     -- outline thickness in pixels
    constant C_FALL_PX      : integer := 4;     -- vertical motion per frame

    -- Triangle initial-vertex constants (Q1.10 fixed-point: 1024 = 1.0).
    -- Vertices are equilateral pre-rotation: (0,-1), (-sin60, cos60),
    -- (sin60, cos60).  Pre-scaled by the CORDIC K factor (0.6073) so that
    -- after the 10-iter CORDIC rotation, vertices recover the original
    -- (0,-1)... positions.
    --   sin60          = 0.8660 ; * K = 0.5260 ; * 1024 = 539
    --   cos60          = 0.5000 ; * K = 0.3036 ; * 1024 = 311
    --   1.0            = 1.0000 ; * K = 0.6073 ; * 1024 = 622
    constant C_KR_ONE_Q10   : signed(12 downto 0) := to_signed(622, 13);
    constant C_KR_SIN60_Q10 : signed(12 downto 0) := to_signed(539, 13);
    constant C_KR_COS60_Q10 : signed(12 downto 0) := to_signed(311, 13);

    -- Angle units: full circle = 1024.  Quadrant boundaries 256, 512, 768.
    -- CORDIC atan(2^-i) constants in 1024-units:
    --   atan(1)    = 45.0°  → 128
    --   atan(1/2)  = 26.57° → 76
    --   atan(1/4)  = 14.04° → 40
    --   atan(1/8)  = 7.13°  → 20
    --   atan(1/16) = 3.58°  → 10
    --   atan(1/32) = 1.79°  → 5
    --   atan(1/64) = 0.895° → 3
    --   atan(1/128)= 0.448° → 1
    --   atan(1/256)= 0.224° → 1
    --   atan(1/512)= 0.112° → 0
    -- v3: 7 iterations.  Angular error after 7 iters bounded by
    -- ~2 * atan(2^-7) ≈ 0.9°; at radius 200 that's ~3 px position error.
    -- Acceptable for a tumbling triangle and saves ~3 CORDIC stages worth
    -- of LCs vs the textbook 10-12 iterations.
    constant C_NUM_CORDIC_ITERS : integer := 7;
    type t_atan_lut is array (0 to C_NUM_CORDIC_ITERS - 1) of signed(10 downto 0);
    constant C_ATAN_LUT : t_atan_lut := (
        to_signed(128, 11),
        to_signed( 76, 11),
        to_signed( 40, 11),
        to_signed( 20, 11),
        to_signed( 10, 11),
        to_signed(  5, 11),
        to_signed(  3, 11)
    );

    -- Vertex working width (signed).  v3: reduced to 11 bits.  Max coord
    -- after pre-scale is R*K ≈ 272*0.6073 ≈ 165 < 512 (= 2^9), so signed-11
    -- has ~2x headroom for negation/quadrant pre-rotation overshoot.
    constant C_VW : integer := 11;
    subtype t_vw is signed(C_VW - 1 downto 0);

    -- Total per-pixel render pipeline (timing_gen + counter + P1..P7).
    constant C_RENDER_DELAY : integer := 9;

    --------------------------------------------------------------------------
    -- Parameter signals
    --------------------------------------------------------------------------
    signal s_outline_hue   : unsigned(9 downto 0);
    signal s_fill_hue      : unsigned(9 downto 0);
    signal s_bg_hue        : unsigned(9 downto 0);
    signal s_size_pot      : unsigned(9 downto 0);
    signal s_density_pot   : unsigned(9 downto 0);  -- TODO
    signal s_axis_pot      : unsigned(9 downto 0);  -- TODO
    signal s_outline_multi : std_logic;             -- v3.3: wired (T7 multi-color outline)
    signal s_filled        : std_logic;
    signal s_invert        : std_logic;
    signal s_size_varied   : std_logic;             -- v3.1: wired (radius breath)
    signal s_rise          : std_logic;
    signal s_video_mix     : unsigned(9 downto 0);

    --------------------------------------------------------------------------
    -- Timing & pixel position
    --------------------------------------------------------------------------
    signal s_timing  : t_video_timing_port;
    signal s_h_count : unsigned(11 downto 0);
    signal s_v_count : unsigned(11 downto 0);

    -- Auto-measured active resolution.
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);

    -- Edge detector for vsync rising edge.
    signal s_vsync_prev      : std_logic := '1';
    signal s_vsync_pulse     : std_logic := '0';  -- one-clock pulse at vsync rising edge

    --------------------------------------------------------------------------
    -- Animation state (latched at vsync_pulse, then static for the frame)
    --------------------------------------------------------------------------
    signal s_frame_count : unsigned(15 downto 0) := (others => '0');
    signal s_target_angle: unsigned(9 downto 0)  := (others => '0');
    signal s_radius      : signed(11 downto 0)   := to_signed(80, 12);
    signal s_y_pos       : signed(13 downto 0)   := (others => '0');
    signal s_tri_cx      : signed(11 downto 0)   := to_signed(480, 12);
    signal s_tri_cy      : signed(11 downto 0)   := (others => '0');

    --------------------------------------------------------------------------
    -- FS1: initial vertex coords (R times pre-scaled equilateral constants).
    -- Three signed multiplies (signed(12) × signed(13)) = signed(25),
    -- shifted right by 10 to give Q-scaled local vertex coords.
    --------------------------------------------------------------------------
    signal s_v0_init_x, s_v0_init_y : t_vw := (others => '0');
    signal s_v1_init_x, s_v1_init_y : t_vw := (others => '0');
    signal s_v2_init_x, s_v2_init_y : t_vw := (others => '0');

    --------------------------------------------------------------------------
    -- FS2: quadrant pre-rotation outputs.  Residual angle now in [0, 256].
    --------------------------------------------------------------------------
    signal s_qr_v0_x, s_qr_v0_y : t_vw := (others => '0');
    signal s_qr_v1_x, s_qr_v1_y : t_vw := (others => '0');
    signal s_qr_v2_x, s_qr_v2_y : t_vw := (others => '0');
    signal s_qr_residual        : signed(10 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- CORDIC pipeline state.  Each iteration k uses constant shift k.
    -- We register the entire vertex state at each stage so the CORDIC
    -- engine is a deeply-pipelined feed-forward circuit.
    --------------------------------------------------------------------------
    type t_cordic_state is record
        x0  : t_vw;
        y0  : t_vw;
        x1  : t_vw;
        y1  : t_vw;
        x2  : t_vw;
        y2  : t_vw;
        ang : signed(10 downto 0);
    end record;

    type t_cordic_arr is array (0 to C_NUM_CORDIC_ITERS) of t_cordic_state;
    signal s_cordic : t_cordic_arr := (others =>
        (x0 => (others => '0'), y0 => (others => '0'),
         x1 => (others => '0'), y1 => (others => '0'),
         x2 => (others => '0'), y2 => (others => '0'),
         ang => (others => '0')));

    --------------------------------------------------------------------------
    -- FS13: world coordinates (rotated local + center).  signed(13) is
    -- enough since values stay within screen bounds.
    --------------------------------------------------------------------------
    signal s_v0_x, s_v0_y : signed(12 downto 0) := (others => '0');
    signal s_v1_x, s_v1_y : signed(12 downto 0) := (others => '0');
    signal s_v2_x, s_v2_y : signed(12 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- FS14: edge deltas dx_i = v_{i+1}.x - v_i.x.
    --------------------------------------------------------------------------
    signal s_e0_dx, s_e0_dy : signed(13 downto 0) := (others => '0');
    signal s_e1_dx, s_e1_dy : signed(13 downto 0) := (others => '0');
    signal s_e2_dx, s_e2_dy : signed(13 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- FS15-16: outline thresholds = Manhattan-length * C_OUTLINE_PX.
    -- Length ≈ max(|dx|,|dy|) + min(|dx|,|dy|)/2 — within ~10% of Euclidean.
    -- thickness=4 → threshold = (max + min/2) << 2 = pure shift+add.
    --------------------------------------------------------------------------
    signal s_e0_adx, s_e0_ady : unsigned(13 downto 0) := (others => '0');
    signal s_e1_adx, s_e1_ady : unsigned(13 downto 0) := (others => '0');
    signal s_e2_adx, s_e2_ady : unsigned(13 downto 0) := (others => '0');
    signal s_e0_thr, s_e1_thr, s_e2_thr : unsigned(17 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- FS17: stage edge_init operands.  These hold stable values that the
    -- shared-multiplier FSM reads from over the next 8 cycles.
    --   -v_i.x : negated vertex x
    --   v_i.y, dx_i, dy_i : passthrough
    --------------------------------------------------------------------------
    signal s_e0_neg_vx, s_e1_neg_vx, s_e2_neg_vx : signed(13 downto 0) := (others => '0');
    signal s_e0_vy_d,   s_e1_vy_d,   s_e2_vy_d   : signed(13 downto 0) := (others => '0');
    signal s_e0_dx_d,   s_e1_dx_d,   s_e2_dx_d   : signed(13 downto 0) := (others => '0');
    signal s_e0_dy_d,   s_e1_dy_d,   s_e2_dy_d   : signed(13 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Shared single multiplier (1-cycle registered).  Drives all six
    -- partial-product computes via an 8-phase FSM.
    --   edge_i(0,0) = -v_i.x * dy_i + v_i.y * dx_i
    --
    -- The 6 multiplies are sequenced across phases 0..5.  The 2-cycle
    -- input-to-output latency (operand register + product register) means
    -- products are captured at phases 2..7.
    --------------------------------------------------------------------------
    signal s_mul_a, s_mul_b     : signed(13 downto 0) := (others => '0');
    -- 3-stage split-product pipeline:
    --   Stage 1 : register operands (s_mul_a_r, s_mul_b_r)
    --   Stage 2 : two smaller partial products (14×8 and 14×7)
    --   Stage 3 : combine partials (shift+add)
    -- Adding the register-stage operand isolation alone did not help —
    -- the critical path was inside the 14×14 multiply tree, not at the
    -- multiplier boundary.  Splitting halves each combinational hop and
    -- gives the placer more freedom.  Total latency: 3 cycles from
    -- operand drive to product visible.
    signal s_mul_a_r, s_mul_b_r : signed(13 downto 0) := (others => '0');
    signal s_pp_lo              : signed(21 downto 0) := (others => '0');  -- 14×8 = 22 bits
    signal s_pp_hi              : signed(20 downto 0) := (others => '0');  -- 14×7 = 21 bits
    signal s_mul_p              : signed(27 downto 0) := (others => '0');
    signal s_pp_a, s_pp_b       : signed(27 downto 0) := (others => '0');
    signal s_mul_phase          : unsigned(3 downto 0) := to_unsigned(15, 4);  -- 15 = idle

    -- Edge_init outputs (latched by FSM when sums complete).
    signal s_e0_init, s_e1_init, s_e2_init : signed(28 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Phase 4 — Triangle 1: a second triangle offset vertically by DELTA_Y_T1.
    -- The offset trick: edge_init_t1_i = edge_init_t0_i + DELTA_Y_T1 * dx_i.
    -- With DELTA_Y_T1 = 256 (a power of 2), the multiply is just a shift,
    -- so triangle 1 costs only adders + DDA accumulators + per-pixel test
    -- — no extra multipliers.
    --------------------------------------------------------------------------
    constant C_DELTA_Y_T1_SHIFT : integer := 8;   -- 256-px vertical offset
    signal s_e0_init_t1, s_e1_init_t1, s_e2_init_t1 : signed(28 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Frame-setup-done pulse: indicates edge_init is settled and edge_row
    -- can be snapshotted.  Driven by a delay chain off vsync_pulse.
    --
    -- Timing budget (cycles from vsync_pulse):
    --   FS0..FS1            : 2  (anim, init verts)
    --   FS2 + CORDIC*7      : 8  (quadrant + 7 iterations)
    --   FS13..FS17          : 5  (world, deltas, abs, thr, init ops)
    --   shared-mul FSM      : 8  (phases 0..7 to compute all edge_init)
    --
    -- Total: 23 cycles minimum.  Use 28 for safe margin.
    --------------------------------------------------------------------------
    constant C_FRAME_SETUP_CLKS : integer := 28;
    signal s_setup_chain : std_logic_vector(C_FRAME_SETUP_CLKS - 1 downto 0) := (others => '0');
    signal s_frame_ready : std_logic := '0';

    -- FSM start pulse: kicks off the shared-multiplier sequencer once FS17
    -- outputs are stable.
    constant C_MUL_FSM_START_CLKS : integer := 15;
    signal s_mul_start_chain : std_logic_vector(C_MUL_FSM_START_CLKS - 1 downto 0) := (others => '0');
    signal s_mul_start       : std_logic := '0';

    --------------------------------------------------------------------------
    -- Edge DDA state: edge_pix per-pixel, edge_row per-line.
    -- Width: edge_init can reach ~26 bits; pixel accumulation adds up to
    -- 1920 * dy ≈ 1920*544 ≈ 1M more.  Use 29 bits for slack.
    --------------------------------------------------------------------------
    signal s_e0_row, s_e1_row, s_e2_row : signed(28 downto 0) := (others => '0');
    signal s_e0_pix, s_e1_pix, s_e2_pix : signed(28 downto 0) := (others => '0');
    -- Triangle 1 DDA accumulators (parallel to triangle 0's).
    signal s_e0_row_t1, s_e1_row_t1, s_e2_row_t1 : signed(28 downto 0) := (others => '0');
    signal s_e0_pix_t1, s_e1_pix_t1, s_e2_pix_t1 : signed(28 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-pixel pipeline registers
    --------------------------------------------------------------------------
    -- P1: latch edge_pix and the per-edge outline thresholds.
    signal s_p1_e0, s_p1_e1, s_p1_e2 : signed(28 downto 0) := (others => '0');
    signal s_p1_t0, s_p1_t1, s_p1_t2 : unsigned(17 downto 0) := (others => '0');
    -- Triangle 1 — pipeline mirror (shares thresholds with triangle 0).
    signal s_p1_e0_t1, s_p1_e1_t1, s_p1_e2_t1 : signed(28 downto 0) := (others => '0');

    -- P2: |edge_i|  +  sign of edge_i (registered separately for use at P3).
    signal s_p2_a0, s_p2_a1, s_p2_a2 : unsigned(28 downto 0) := (others => '0');
    signal s_p2_s0, s_p2_s1, s_p2_s2 : std_logic := '0';
    signal s_p2_t0, s_p2_t1, s_p2_t2 : unsigned(17 downto 0) := (others => '0');
    -- Triangle 1
    signal s_p2_a0_t1, s_p2_a1_t1, s_p2_a2_t1 : unsigned(28 downto 0) := (others => '0');
    signal s_p2_s0_t1, s_p2_s1_t1, s_p2_s2_t1 : std_logic := '0';

    -- P3: per-edge outline flag + pass sign through.
    signal s_p3_o0, s_p3_o1, s_p3_o2 : std_logic := '0';
    signal s_p3_s0, s_p3_s1, s_p3_s2 : std_logic := '0';
    -- Triangle 1
    signal s_p3_o0_t1, s_p3_o1_t1, s_p3_o2_t1 : std_logic := '0';
    signal s_p3_s0_t1, s_p3_s1_t1, s_p3_s2_t1 : std_logic := '0';

    -- P4: combined inside flag + on_outline flag.
    signal s_p4_inside     : std_logic := '0';
    signal s_p4_on_outline : std_logic := '0';
    -- Triangle 1
    signal s_p4_inside_t1     : std_logic := '0';
    signal s_p4_on_outline_t1 : std_logic := '0';

    -- P5: color mux output (registered).
    signal s_p5_y : unsigned(9 downto 0) := (others => '0');
    signal s_p5_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_p5_v : unsigned(9 downto 0) := C_CHROMA_MID;

    -- P6, P7: extra pipeline slack for chroma/luma output path.
    signal s_p6_y, s_p7_y : unsigned(9 downto 0) := (others => '0');
    signal s_p6_u, s_p7_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_p6_v, s_p7_v : unsigned(9 downto 0) := C_CHROMA_MID;

    --------------------------------------------------------------------------
    -- Video pass-through delay + sync pipeline.
    -- Video data must align with the color mux (P5) — delay = pipeline
    -- stages before P5 = timing(1) + counter(1) + P1(1) + P2(1) + P3(1)
    -- + P4(1) = 6 stages.
    --------------------------------------------------------------------------
    constant C_VIDEO_DELAY : integer := 6;
    type t_data_delay is array (0 to C_VIDEO_DELAY - 1) of std_logic_vector(9 downto 0);
    signal s_y_delay : t_data_delay := (others => (others => '0'));
    signal s_u_delay : t_data_delay := (others => (others => '0'));
    signal s_v_delay : t_data_delay := (others => (others => '0'));

    type t_sync_pipe is array (0 to C_RENDER_DELAY - 1) of std_logic_vector(3 downto 0);
    signal s_sync_pipe : t_sync_pipe := (others => (others => '0'));

    -- IO alignment (2 stages)
    signal s_io_0 : t_video_stream_yuv444_30b;
    signal s_io_1 : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Register Mapping
    --------------------------------------------------------------------------
    s_outline_hue   <= unsigned(registers_in(0));
    s_fill_hue      <= unsigned(registers_in(1));
    s_bg_hue        <= unsigned(registers_in(2));
    s_size_pot      <= unsigned(registers_in(3));
    s_density_pot   <= unsigned(registers_in(4));
    s_axis_pot      <= unsigned(registers_in(5));
    s_outline_multi <= registers_in(6)(0);
    s_filled        <= registers_in(6)(1);
    s_invert        <= registers_in(6)(2);
    s_size_varied   <= registers_in(6)(3);
    s_rise          <= registers_in(6)(4);
    s_video_mix     <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Auto-Measure Active Resolution
    --------------------------------------------------------------------------
    p_measure_resolution : process(clk)
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

    --------------------------------------------------------------------------
    -- Video Timing Generator
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    --------------------------------------------------------------------------
    -- Pixel Position Counters
    --------------------------------------------------------------------------
    pixel_counter_inst : entity work.pixel_counter
        port map (
            clk     => clk,
            timing  => s_timing,
            h_count => s_h_count,
            v_count => s_v_count
        );

    --------------------------------------------------------------------------
    -- Vsync rising-edge detector (one-clock pulse).
    --------------------------------------------------------------------------
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
    -- FS0: latch frame counter, target angle, radius, y_pos, center.
    --------------------------------------------------------------------------
    p_fs0_anim : process(clk)
        variable v_y_range : signed(13 downto 0);
        variable v_y_next  : signed(13 downto 0);
        variable v_radius  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                s_frame_count <= s_frame_count + 1;

                -- Target rotation angle wraps every 1024 frames (~17 s @60 Hz).
                -- 2x faster rotation: ~8 sec per turn @60fps. Faster than v3.0
                -- (was ~17s/turn) so the tumble reads more clearly.
                s_target_angle <= s_frame_count(8 downto 0) & "0";

                -- Y motion: ±C_FALL_PX per frame.
                v_y_range := signed(resize(s_measured_v, 14))
                           + to_signed(2 * 256, 14);
                if s_rise = '1' then
                    v_y_next := s_y_pos - to_signed(C_FALL_PX, 14);
                    if v_y_next < to_signed(-256, 14) then
                        v_y_next := v_y_next + v_y_range;
                    end if;
                else
                    v_y_next := s_y_pos + to_signed(C_FALL_PX, 14);
                    if v_y_next > signed(resize(s_measured_v, 14)) + to_signed(256, 14) then
                        v_y_next := v_y_next - v_y_range;
                    end if;
                end if;
                s_y_pos <= v_y_next;

                -- Radius: 16..272 from K4 (size_pot >> 2 + 16).
                v_radius := resize(shift_right(s_size_pot, 2), 12)
                            + to_unsigned(16, 12);
                -- T10 (Size-Varied): 4-step ±R/8 cycle on frame_count(6:5)
                -- giving a slow breath ~128 frames period (no multiplier;
                -- just shifts + adders + 4-way mux).
                if s_size_varied = '1' then
                    case s_frame_count(6 downto 5) is
                        when "00" =>
                            s_radius <= signed(v_radius)
                                      - signed(resize(shift_right(v_radius, 3), 12));
                        when "10" =>
                            s_radius <= signed(v_radius)
                                      + signed(resize(shift_right(v_radius, 3), 12));
                        when others =>
                            s_radius <= signed(v_radius);
                    end case;
                else
                    s_radius <= signed(v_radius);
                end if;

                -- Center: horizontally fixed at h_active/2.
                s_tri_cx <= signed(resize(shift_right(s_measured_h, 1), 12));
                s_tri_cy <= v_y_next(11 downto 0);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- FS1: initial pre-rotation vertex coords.
    --   v0 = (0, -R*K)
    --   v1 = (-R*sin60*K, R*cos60*K)
    --   v2 = ( R*sin60*K, R*cos60*K)
    --
    -- R is signed(12), constants are signed(13).  Product signed(25), then
    -- right-shifted by 10 → signed(15) before resize to vertex width.
    -- Three multiplies (R*KR_ONE_Q10, R*KR_SIN60_Q10, R*KR_COS60_Q10) —
    -- yosys infers them as parallel; they're pipelined by registering the
    -- result into s_v*_init_*.
    --------------------------------------------------------------------------
    -- v3: shift-add approximations of the three R-times-constant products.
    -- Saves three full signed-multiply blocks.  Approximations are tuned
    -- so the post-CORDIC triangle visually matches the desired equilateral
    -- shape; small errors are absorbed by the triangle's symmetry.
    --
    --   R * 0.6073   (= R * 622/1024) ≈ R/2 + R/16 + R/32 + R/128 + R/256
    --                                  = R * 0.6093  (err +0.3%)
    --   R * 0.5260   (= R * 539/1024) ≈ R/2 + R/32 - R/256
    --                                  = R * 0.5273  (err +0.2%)
    --   R * 0.3036   (= R * 311/1024) ≈ R/4 + R/16 - R/256
    --                                  = R * 0.3086  (err +1.6%)
    --
    -- All three computations share the radius register input but require
    -- their own adder chain — total: ~9 adders, all 12-bit, ~100 LCs vs
    -- ~750 LCs for three parallel multipliers.
    p_fs1_initverts : process(clk)
        variable v_r       : signed(12 downto 0);
        variable v_r_one   : signed(12 downto 0);
        variable v_r_sin60 : signed(12 downto 0);
        variable v_r_cos60 : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            v_r := resize(s_radius, 13);

            -- R * 0.6093 = R/2 + R/16 + R/32 + R/128 + R/256
            v_r_one := shift_right(v_r, 1)
                     + shift_right(v_r, 4)
                     + shift_right(v_r, 5)
                     + shift_right(v_r, 7)
                     + shift_right(v_r, 8);

            -- R * 0.5273 = R/2 + R/32 - R/256
            v_r_sin60 := shift_right(v_r, 1)
                       + shift_right(v_r, 5)
                       - shift_right(v_r, 8);

            -- R * 0.3086 = R/4 + R/16 - R/256
            v_r_cos60 := shift_right(v_r, 2)
                       + shift_right(v_r, 4)
                       - shift_right(v_r, 8);

            s_v0_init_x <= (others => '0');
            s_v0_init_y <= resize(-v_r_one, C_VW);

            s_v1_init_x <= resize(-v_r_sin60, C_VW);
            s_v1_init_y <= resize( v_r_cos60, C_VW);

            s_v2_init_x <= resize( v_r_sin60, C_VW);
            s_v2_init_y <= resize( v_r_cos60, C_VW);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- FS2: quadrant pre-rotation.
    --   Bring residual into [0, 256] (= [0°, 90°]) via 90°/180°/270°
    --   pre-rotations that are pure sign flips and axis swaps.
    --   Quadrant is the top 2 bits of s_target_angle.
    --------------------------------------------------------------------------
    p_fs2_quadrant : process(clk)
        variable v_q : unsigned(1 downto 0);
        variable v_residual : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            v_q := s_target_angle(9 downto 8);

            -- Residual angle in [0, 256], signed 11-bit.
            v_residual := signed(resize(s_target_angle(7 downto 0), 11));
            s_qr_residual <= v_residual;

            case v_q is
                when "00" =>
                    -- Q1: no swap
                    s_qr_v0_x <= s_v0_init_x;  s_qr_v0_y <= s_v0_init_y;
                    s_qr_v1_x <= s_v1_init_x;  s_qr_v1_y <= s_v1_init_y;
                    s_qr_v2_x <= s_v2_init_x;  s_qr_v2_y <= s_v2_init_y;
                when "01" =>
                    -- Q2 (+90°): x' = -y, y' = x
                    s_qr_v0_x <= -s_v0_init_y; s_qr_v0_y <= s_v0_init_x;
                    s_qr_v1_x <= -s_v1_init_y; s_qr_v1_y <= s_v1_init_x;
                    s_qr_v2_x <= -s_v2_init_y; s_qr_v2_y <= s_v2_init_x;
                when "10" =>
                    -- Q3 (+180°): negate both
                    s_qr_v0_x <= -s_v0_init_x; s_qr_v0_y <= -s_v0_init_y;
                    s_qr_v1_x <= -s_v1_init_x; s_qr_v1_y <= -s_v1_init_y;
                    s_qr_v2_x <= -s_v2_init_x; s_qr_v2_y <= -s_v2_init_y;
                when others =>
                    -- Q4 (+270°): x' = y, y' = -x
                    s_qr_v0_x <= s_v0_init_y;  s_qr_v0_y <= -s_v0_init_x;
                    s_qr_v1_x <= s_v1_init_y;  s_qr_v1_y <= -s_v1_init_x;
                    s_qr_v2_x <= s_v2_init_y;  s_qr_v2_y <= -s_v2_init_x;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- CORDIC pipeline input: load the quadrant-pre-rotated vertices and
    -- residual into stage 0.
    --------------------------------------------------------------------------
    p_cordic_load : process(clk)
    begin
        if rising_edge(clk) then
            s_cordic(0).x0  <= s_qr_v0_x;
            s_cordic(0).y0  <= s_qr_v0_y;
            s_cordic(0).x1  <= s_qr_v1_x;
            s_cordic(0).y1  <= s_qr_v1_y;
            s_cordic(0).x2  <= s_qr_v2_x;
            s_cordic(0).y2  <= s_qr_v2_y;
            s_cordic(0).ang <= s_qr_residual;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- CORDIC pipeline stages 1..C_NUM_CORDIC_ITERS.
    -- Each stage applies one CORDIC iteration k with constant shift k.
    -- Sign of the residual angle selects rotation direction.
    --   x' = x - sign * (y >> k)
    --   y' = y + sign * (x >> k)
    --   ang' = ang - sign * atan(2^-k)
    --------------------------------------------------------------------------
    gen_cordic : for k in 0 to C_NUM_CORDIC_ITERS - 1 generate
        p_cordic_iter : process(clk)
            variable v_sign_pos : boolean;
            variable v_shx0, v_shy0 : t_vw;
            variable v_shx1, v_shy1 : t_vw;
            variable v_shx2, v_shy2 : t_vw;
        begin
            if rising_edge(clk) then
                v_sign_pos := s_cordic(k).ang >= 0;

                v_shx0 := shift_right(s_cordic(k).x0, k);
                v_shy0 := shift_right(s_cordic(k).y0, k);
                v_shx1 := shift_right(s_cordic(k).x1, k);
                v_shy1 := shift_right(s_cordic(k).y1, k);
                v_shx2 := shift_right(s_cordic(k).x2, k);
                v_shy2 := shift_right(s_cordic(k).y2, k);

                if v_sign_pos then
                    -- sign = +1 :  x' = x - (y>>k);  y' = y + (x>>k);  ang' = ang - atan
                    s_cordic(k + 1).x0  <= s_cordic(k).x0 - v_shy0;
                    s_cordic(k + 1).y0  <= s_cordic(k).y0 + v_shx0;
                    s_cordic(k + 1).x1  <= s_cordic(k).x1 - v_shy1;
                    s_cordic(k + 1).y1  <= s_cordic(k).y1 + v_shx1;
                    s_cordic(k + 1).x2  <= s_cordic(k).x2 - v_shy2;
                    s_cordic(k + 1).y2  <= s_cordic(k).y2 + v_shx2;
                    s_cordic(k + 1).ang <= s_cordic(k).ang - C_ATAN_LUT(k);
                else
                    -- sign = -1 :  x' = x + (y>>k);  y' = y - (x>>k);  ang' = ang + atan
                    s_cordic(k + 1).x0  <= s_cordic(k).x0 + v_shy0;
                    s_cordic(k + 1).y0  <= s_cordic(k).y0 - v_shx0;
                    s_cordic(k + 1).x1  <= s_cordic(k).x1 + v_shy1;
                    s_cordic(k + 1).y1  <= s_cordic(k).y1 - v_shx1;
                    s_cordic(k + 1).x2  <= s_cordic(k).x2 + v_shy2;
                    s_cordic(k + 1).y2  <= s_cordic(k).y2 - v_shx2;
                    s_cordic(k + 1).ang <= s_cordic(k).ang + C_ATAN_LUT(k);
                end if;
            end if;
        end process;
    end generate;

    --------------------------------------------------------------------------
    -- FS13: translate rotated local vertices to world coords.
    -- The CORDIC output is at s_cordic(C_NUM_CORDIC_ITERS).
    --------------------------------------------------------------------------
    p_fs13_world : process(clk)
    begin
        if rising_edge(clk) then
            s_v0_x <= resize(s_cordic(C_NUM_CORDIC_ITERS).x0, 13) + resize(s_tri_cx, 13);
            s_v0_y <= resize(s_cordic(C_NUM_CORDIC_ITERS).y0, 13) + resize(s_tri_cy, 13);
            s_v1_x <= resize(s_cordic(C_NUM_CORDIC_ITERS).x1, 13) + resize(s_tri_cx, 13);
            s_v1_y <= resize(s_cordic(C_NUM_CORDIC_ITERS).y1, 13) + resize(s_tri_cy, 13);
            s_v2_x <= resize(s_cordic(C_NUM_CORDIC_ITERS).x2, 13) + resize(s_tri_cx, 13);
            s_v2_y <= resize(s_cordic(C_NUM_CORDIC_ITERS).y2, 13) + resize(s_tri_cy, 13);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- FS14: edge deltas dx_i, dy_i.
    --------------------------------------------------------------------------
    p_fs14_deltas : process(clk)
    begin
        if rising_edge(clk) then
            s_e0_dx <= resize(s_v1_x, 14) - resize(s_v0_x, 14);
            s_e0_dy <= resize(s_v1_y, 14) - resize(s_v0_y, 14);
            s_e1_dx <= resize(s_v2_x, 14) - resize(s_v1_x, 14);
            s_e1_dy <= resize(s_v2_y, 14) - resize(s_v1_y, 14);
            s_e2_dx <= resize(s_v0_x, 14) - resize(s_v2_x, 14);
            s_e2_dy <= resize(s_v0_y, 14) - resize(s_v2_y, 14);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- FS15: |dx_i|, |dy_i| for Manhattan length.
    --------------------------------------------------------------------------
    p_fs15_abs : process(clk)
    begin
        if rising_edge(clk) then
            s_e0_adx <= unsigned(abs(s_e0_dx));
            s_e0_ady <= unsigned(abs(s_e0_dy));
            s_e1_adx <= unsigned(abs(s_e1_dx));
            s_e1_ady <= unsigned(abs(s_e1_dy));
            s_e2_adx <= unsigned(abs(s_e2_dx));
            s_e2_ady <= unsigned(abs(s_e2_dy));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- FS16: outline threshold = (max(|dx|,|dy|) + min(|dx|,|dy|)/2) * 4.
    --------------------------------------------------------------------------
    p_fs16_thr : process(clk)
        variable v_mx, v_mn : unsigned(13 downto 0);
        variable v_len      : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if s_e0_adx > s_e0_ady then
                v_mx := s_e0_adx; v_mn := s_e0_ady;
            else
                v_mx := s_e0_ady; v_mn := s_e0_adx;
            end if;
            v_len := v_mx + shift_right(v_mn, 1);
            s_e0_thr <= resize(v_len, 17) & "0";  -- *2 (thin outline ≈ 1 display px @dec4)

            if s_e1_adx > s_e1_ady then
                v_mx := s_e1_adx; v_mn := s_e1_ady;
            else
                v_mx := s_e1_ady; v_mn := s_e1_adx;
            end if;
            v_len := v_mx + shift_right(v_mn, 1);
            s_e1_thr <= resize(v_len, 17) & "0";

            if s_e2_adx > s_e2_ady then
                v_mx := s_e2_adx; v_mn := s_e2_ady;
            else
                v_mx := s_e2_ady; v_mn := s_e2_adx;
            end if;
            v_len := v_mx + shift_right(v_mn, 1);
            s_e2_thr <= resize(v_len, 17) & "0";
        end if;
    end process;

    --------------------------------------------------------------------------
    -- FS17: stage edge_init operands.
    --   -v.x  (one's-complement +1)
    --   pass v.y, dx, dy
    --------------------------------------------------------------------------
    p_fs17_init_ops : process(clk)
    begin
        if rising_edge(clk) then
            s_e0_neg_vx <= -resize(s_v0_x, 14);
            s_e1_neg_vx <= -resize(s_v1_x, 14);
            s_e2_neg_vx <= -resize(s_v2_x, 14);

            s_e0_vy_d <= resize(s_v0_y, 14);
            s_e1_vy_d <= resize(s_v1_y, 14);
            s_e2_vy_d <= resize(s_v2_y, 14);

            s_e0_dx_d <= s_e0_dx;
            s_e1_dx_d <= s_e1_dx;
            s_e2_dx_d <= s_e2_dx;

            s_e0_dy_d <= s_e0_dy;
            s_e1_dy_d <= s_e1_dy;
            s_e2_dy_d <= s_e2_dy;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Shared multiplier: 3-stage pipelined signed(14) * signed(14).
    --
    -- Splits operand b into low-7 (treated as 8-bit non-negative, prepending
    -- a zero so the value stays positive) and high-7 (signed, captures the
    -- original sign bit at MSB).  Computes two narrower partial products in
    -- stage 2, then combines them with shift+add in stage 3.  This halves
    -- each combinational hop relative to a monolithic 14×14 multiply tree.
    --
    --   b = b_hi * 128 + b_lo  (b_hi signed-7, b_lo unsigned-7)
    --   a * b = a*b_hi*128 + a*b_lo
    --
    -- Latency: 3 cycles operand-drive → product visible.
    --------------------------------------------------------------------------
    p_mul_inst : process(clk)
        variable v_b_slv : std_logic_vector(13 downto 0);
        variable v_b_lo  : signed(7 downto 0);
        variable v_b_hi  : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            -- Stage 1: register operands
            s_mul_a_r <= s_mul_a;
            s_mul_b_r <= s_mul_b;

            -- Stage 2: split b_r and compute partials
            v_b_slv := std_logic_vector(s_mul_b_r);
            v_b_lo  := signed('0' & v_b_slv(6 downto 0));   -- 8-bit non-negative
            v_b_hi  := signed(v_b_slv(13 downto 7));         -- 7-bit signed
            s_pp_lo <= s_mul_a_r * v_b_lo;                   -- 14 × 8 = 22 bits
            s_pp_hi <= s_mul_a_r * v_b_hi;                   -- 14 × 7 = 21 bits

            -- Stage 3: combine
            s_mul_p <= resize(s_pp_lo, 28) + shift_left(resize(s_pp_hi, 28), 7);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Multiplier-start delay chain.  s_mul_start fires C_MUL_FSM_START_CLKS
    -- cycles after vsync_pulse — by which time FS17 outputs are stable.
    --------------------------------------------------------------------------
    p_mul_start : process(clk)
    begin
        if rising_edge(clk) then
            s_mul_start_chain(0) <= s_vsync_pulse;
            for i in 1 to C_MUL_FSM_START_CLKS - 1 loop
                s_mul_start_chain(i) <= s_mul_start_chain(i - 1);
            end loop;
            s_mul_start <= s_mul_start_chain(C_MUL_FSM_START_CLKS - 1);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Multiplier FSM: sequences 6 operand pairs through the shared
    -- multiplier and accumulates results into edge_init_{0,1,2}.
    --
    -- Phase | Drive            | Product available (3-cycle latency)
    --   0   | -v0.x , dy_0     | -- (operand reg propagating)
    --   1   |  v0.y , dx_0     | -- (multiplier propagating)
    --   2   | -v1.x , dy_1     | -- (product reg writing)
    --   3   |  v1.y , dx_1     | product 0 → pp_a
    --   4   | -v2.x , dy_2     | product 1 → pp_b ; edge_init_0 = pp_a + pp_b
    --   5   |  v2.y , dx_2     | product 2 → pp_a
    --   6   | idle             | product 3 → pp_b ; edge_init_1 = pp_a + pp_b
    --   7   | idle             | product 4 → pp_a
    --   8   | idle             | product 5 → pp_b ; edge_init_2 = pp_a + pp_b
    --
    -- After phase 8, FSM returns to idle (phase 15).
    --------------------------------------------------------------------------
    p_mul_fsm : process(clk)
    begin
        if rising_edge(clk) then
            -- Phase counter
            if s_mul_start = '1' then
                s_mul_phase <= to_unsigned(0, 4);
            elsif s_mul_phase < to_unsigned(9, 4) then
                s_mul_phase <= s_mul_phase + 1;
            else
                s_mul_phase <= to_unsigned(15, 4);  -- idle
            end if;

            -- Drive operands based on phase
            case to_integer(s_mul_phase) is
                when 0 => s_mul_a <= s_e0_neg_vx; s_mul_b <= s_e0_dy_d;
                when 1 => s_mul_a <= s_e0_vy_d;   s_mul_b <= s_e0_dx_d;
                when 2 => s_mul_a <= s_e1_neg_vx; s_mul_b <= s_e1_dy_d;
                when 3 => s_mul_a <= s_e1_vy_d;   s_mul_b <= s_e1_dx_d;
                when 4 => s_mul_a <= s_e2_neg_vx; s_mul_b <= s_e2_dy_d;
                when 5 => s_mul_a <= s_e2_vy_d;   s_mul_b <= s_e2_dx_d;
                when others =>
                    s_mul_a <= (others => '0');
                    s_mul_b <= (others => '0');
            end case;

            -- Collect products (3-cycle latency: drive at phase N → product
            -- visible at phase N+3).  So at phase M, we capture the product
            -- whose operands were driven at phase M-3.
            case to_integer(s_mul_phase) is
                when 3 =>
                    s_pp_a <= s_mul_p;
                when 4 =>
                    s_pp_b <= s_mul_p;
                    s_e0_init <= resize(s_pp_a, 29) + resize(s_mul_p, 29);
                when 5 =>
                    s_pp_a <= s_mul_p;
                when 6 =>
                    s_pp_b <= s_mul_p;
                    s_e1_init <= resize(s_pp_a, 29) + resize(s_mul_p, 29);
                when 7 =>
                    s_pp_a <= s_mul_p;
                when 8 =>
                    s_pp_b <= s_mul_p;
                    s_e2_init <= resize(s_pp_a, 29) + resize(s_mul_p, 29);
                when others =>
                    null;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Triangle 1 edge_init: derived from triangle 0's by the vertical-offset
    -- trick.  See declaration: edge_init_t1_i = edge_init_t0_i + DELTA_Y * dx_i.
    -- DELTA_Y is 256 (= shift_left by C_DELTA_Y_T1_SHIFT).
    -- Settles 1 cycle after edge_init; that still beats frame_ready by margin.
    --------------------------------------------------------------------------
    p_edge_init_t1 : process(clk)
    begin
        if rising_edge(clk) then
            s_e0_init_t1 <= s_e0_init + shift_left(resize(s_e0_dx, 29), C_DELTA_Y_T1_SHIFT);
            s_e1_init_t1 <= s_e1_init + shift_left(resize(s_e1_dx, 29), C_DELTA_Y_T1_SHIFT);
            s_e2_init_t1 <= s_e2_init + shift_left(resize(s_e2_dx, 29), C_DELTA_Y_T1_SHIFT);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Frame-setup-done pulse: shifts vsync_pulse through a chain.
    -- After C_FRAME_SETUP_CLKS clocks, the pulse arrives at s_frame_ready,
    -- by which time edge_init is settled.
    --------------------------------------------------------------------------
    p_frame_setup_done : process(clk)
    begin
        if rising_edge(clk) then
            s_setup_chain(0) <= s_vsync_pulse;
            for i in 1 to C_FRAME_SETUP_CLKS - 1 loop
                s_setup_chain(i) <= s_setup_chain(i - 1);
            end loop;
            s_frame_ready <= s_setup_chain(C_FRAME_SETUP_CLKS - 1);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Edge DDA: maintain edge_row (line start) and edge_pix (per pixel).
    --   at frame_ready : edge_row <= edge_init (snapshot)
    --   at avid_start  : edge_pix <= edge_row;
    --                     edge_row <= edge_row - dx
    --   at avid        : edge_pix <= edge_pix + dy
    --------------------------------------------------------------------------
    p_edge_dda : process(clk)
    begin
        if rising_edge(clk) then
            if s_frame_ready = '1' then
                s_e0_row <= s_e0_init;
                s_e1_row <= s_e1_init;
                s_e2_row <= s_e2_init;
                s_e0_row_t1 <= s_e0_init_t1;
                s_e1_row_t1 <= s_e1_init_t1;
                s_e2_row_t1 <= s_e2_init_t1;
            elsif s_timing.avid_start = '1' then
                s_e0_pix <= s_e0_row;
                s_e1_pix <= s_e1_row;
                s_e2_pix <= s_e2_row;
                s_e0_row <= s_e0_row - resize(s_e0_dx, 29);
                s_e1_row <= s_e1_row - resize(s_e1_dx, 29);
                s_e2_row <= s_e2_row - resize(s_e2_dx, 29);
                -- Triangle 1: same dx/dy (same shape), just offset row state
                s_e0_pix_t1 <= s_e0_row_t1;
                s_e1_pix_t1 <= s_e1_row_t1;
                s_e2_pix_t1 <= s_e2_row_t1;
                s_e0_row_t1 <= s_e0_row_t1 - resize(s_e0_dx, 29);
                s_e1_row_t1 <= s_e1_row_t1 - resize(s_e1_dx, 29);
                s_e2_row_t1 <= s_e2_row_t1 - resize(s_e2_dx, 29);
            elsif s_timing.avid = '1' then
                s_e0_pix <= s_e0_pix + resize(s_e0_dy, 29);
                s_e1_pix <= s_e1_pix + resize(s_e1_dy, 29);
                s_e2_pix <= s_e2_pix + resize(s_e2_dy, 29);
                s_e0_pix_t1 <= s_e0_pix_t1 + resize(s_e0_dy, 29);
                s_e1_pix_t1 <= s_e1_pix_t1 + resize(s_e1_dy, 29);
                s_e2_pix_t1 <= s_e2_pix_t1 + resize(s_e2_dy, 29);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P1: latch edge_pix accumulators + outline thresholds into the
    -- per-pixel pipeline (de-couples DDA from downstream comparisons).
    --------------------------------------------------------------------------
    p_p1 : process(clk)
    begin
        if rising_edge(clk) then
            s_p1_e0 <= s_e0_pix;
            s_p1_e1 <= s_e1_pix;
            s_p1_e2 <= s_e2_pix;
            s_p1_t0 <= s_e0_thr;
            s_p1_t1 <= s_e1_thr;
            s_p1_t2 <= s_e2_thr;
            -- Triangle 1 (shares thresholds; only edge values differ)
            s_p1_e0_t1 <= s_e0_pix_t1;
            s_p1_e1_t1 <= s_e1_pix_t1;
            s_p1_e2_t1 <= s_e2_pix_t1;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P2: absolute value + extract sign of each edge.
    --------------------------------------------------------------------------
    p_p2 : process(clk)
    begin
        if rising_edge(clk) then
            s_p2_a0 <= unsigned(abs(s_p1_e0));
            s_p2_a1 <= unsigned(abs(s_p1_e1));
            s_p2_a2 <= unsigned(abs(s_p1_e2));
            s_p2_s0 <= s_p1_e0(s_p1_e0'high);
            s_p2_s1 <= s_p1_e1(s_p1_e1'high);
            s_p2_s2 <= s_p1_e2(s_p1_e2'high);
            s_p2_t0 <= s_p1_t0;
            s_p2_t1 <= s_p1_t1;
            s_p2_t2 <= s_p1_t2;
            -- Triangle 1
            s_p2_a0_t1 <= unsigned(abs(s_p1_e0_t1));
            s_p2_a1_t1 <= unsigned(abs(s_p1_e1_t1));
            s_p2_a2_t1 <= unsigned(abs(s_p1_e2_t1));
            s_p2_s0_t1 <= s_p1_e0_t1(s_p1_e0_t1'high);
            s_p2_s1_t1 <= s_p1_e1_t1(s_p1_e1_t1'high);
            s_p2_s2_t1 <= s_p1_e2_t1(s_p1_e2_t1'high);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P3: per-edge outline test, wedge-clipped to actual segments.
    --
    -- Naive "near edge i" test (|edge_i| < thr) extends along the entire
    -- infinite line containing edge i — which would draw lines into infinity
    -- past each vertex.  Fix: outline_i fires only when the pixel is
    -- ALSO inside the wedge formed by the OTHER two edges.  The wedge for
    -- the segment of edge i is bounded by edges j and k, whose signs must
    -- AGREE with each other (= they both point to the inside).  Specifically:
    --     outline_i = near_i  AND  sign(edge_j) = sign(edge_k)
    -- This clips the outline to the actual edge segment between two vertices.
    --
    -- Since |edge| is 29-bit unsigned and threshold is 18-bit, we check
    -- that upper 11 bits of |edge| are zero, then compare lower 18 bits.
    --------------------------------------------------------------------------
    p_p3 : process(clk)
        variable v_hi0, v_hi1, v_hi2 : unsigned(10 downto 0);
        variable v_lo0, v_lo1, v_lo2 : unsigned(17 downto 0);
        variable v_near0, v_near1, v_near2 : std_logic;
        variable v_hi0_t1, v_hi1_t1, v_hi2_t1 : unsigned(10 downto 0);
        variable v_lo0_t1, v_lo1_t1, v_lo2_t1 : unsigned(17 downto 0);
        variable v_near0_t1, v_near1_t1, v_near2_t1 : std_logic;
    begin
        if rising_edge(clk) then
            v_hi0 := s_p2_a0(28 downto 18);
            v_hi1 := s_p2_a1(28 downto 18);
            v_hi2 := s_p2_a2(28 downto 18);
            v_lo0 := s_p2_a0(17 downto 0);
            v_lo1 := s_p2_a1(17 downto 0);
            v_lo2 := s_p2_a2(17 downto 0);

            if v_hi0 = 0 and v_lo0 < s_p2_t0 then
                v_near0 := '1';
            else
                v_near0 := '0';
            end if;
            if v_hi1 = 0 and v_lo1 < s_p2_t1 then
                v_near1 := '1';
            else
                v_near1 := '0';
            end if;
            if v_hi2 = 0 and v_lo2 < s_p2_t2 then
                v_near2 := '1';
            else
                v_near2 := '0';
            end if;

            -- Wedge-clip: outline_i requires the two OTHER edge signs to agree.
            if v_near0 = '1' and s_p2_s1 = s_p2_s2 then
                s_p3_o0 <= '1';
            else
                s_p3_o0 <= '0';
            end if;
            if v_near1 = '1' and s_p2_s0 = s_p2_s2 then
                s_p3_o1 <= '1';
            else
                s_p3_o1 <= '0';
            end if;
            if v_near2 = '1' and s_p2_s0 = s_p2_s1 then
                s_p3_o2 <= '1';
            else
                s_p3_o2 <= '0';
            end if;

            s_p3_s0 <= s_p2_s0;
            s_p3_s1 <= s_p2_s1;
            s_p3_s2 <= s_p2_s2;

            -- Triangle 1 (same thresholds, separate edge state).
            v_hi0_t1 := s_p2_a0_t1(28 downto 18);
            v_hi1_t1 := s_p2_a1_t1(28 downto 18);
            v_hi2_t1 := s_p2_a2_t1(28 downto 18);
            v_lo0_t1 := s_p2_a0_t1(17 downto 0);
            v_lo1_t1 := s_p2_a1_t1(17 downto 0);
            v_lo2_t1 := s_p2_a2_t1(17 downto 0);

            if v_hi0_t1 = 0 and v_lo0_t1 < s_p2_t0 then
                v_near0_t1 := '1';
            else
                v_near0_t1 := '0';
            end if;
            if v_hi1_t1 = 0 and v_lo1_t1 < s_p2_t1 then
                v_near1_t1 := '1';
            else
                v_near1_t1 := '0';
            end if;
            if v_hi2_t1 = 0 and v_lo2_t1 < s_p2_t2 then
                v_near2_t1 := '1';
            else
                v_near2_t1 := '0';
            end if;

            if v_near0_t1 = '1' and s_p2_s1_t1 = s_p2_s2_t1 then
                s_p3_o0_t1 <= '1';
            else
                s_p3_o0_t1 <= '0';
            end if;
            if v_near1_t1 = '1' and s_p2_s0_t1 = s_p2_s2_t1 then
                s_p3_o1_t1 <= '1';
            else
                s_p3_o1_t1 <= '0';
            end if;
            if v_near2_t1 = '1' and s_p2_s0_t1 = s_p2_s1_t1 then
                s_p3_o2_t1 <= '1';
            else
                s_p3_o2_t1 <= '0';
            end if;

            s_p3_s0_t1 <= s_p2_s0_t1;
            s_p3_s1_t1 <= s_p2_s1_t1;
            s_p3_s2_t1 <= s_p2_s2_t1;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P4: combine flags.  Inside iff all signs match.  Outline iff any
    -- edge is within threshold.
    --------------------------------------------------------------------------
    p_p4 : process(clk)
        variable v_inside_t0, v_inside_t1 : std_logic;
        variable v_outline_t0, v_outline_t1 : std_logic;
    begin
        if rising_edge(clk) then
            -- Triangle 0
            if (s_p3_s0 = s_p3_s1) and (s_p3_s1 = s_p3_s2) then
                v_inside_t0 := '1';
            else
                v_inside_t0 := '0';
            end if;
            v_outline_t0 := s_p3_o0 or s_p3_o1 or s_p3_o2;

            -- Triangle 1
            if (s_p3_s0_t1 = s_p3_s1_t1) and (s_p3_s1_t1 = s_p3_s2_t1) then
                v_inside_t1 := '1';
            else
                v_inside_t1 := '0';
            end if;
            v_outline_t1 := s_p3_o0_t1 or s_p3_o1_t1 or s_p3_o2_t1;

            -- Per-triangle flags kept for future per-triangle coloring.
            s_p4_inside_t1     <= v_inside_t1;
            s_p4_on_outline_t1 <= v_outline_t1;

            -- Combined for P5 mux: outline wins over inside; any triangle wins.
            s_p4_inside     <= v_inside_t0 or v_inside_t1;
            s_p4_on_outline <= v_outline_t0 or v_outline_t1;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P5: color mux.
    --   Outline overrides fill.
    --   Fill (when T8 Filled) shows solid color or delayed video — hard
    --   threshold on Fader 12 (smooth interp is a future iteration).
    --   Outline: solid color or XOR-luma over video (T9 Invert).
    --   Background: bg color.
    --------------------------------------------------------------------------
    p_p5 : process(clk)
        variable v_vy, v_vu, v_vv : unsigned(9 downto 0);
        variable v_eff_hue        : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_vy := unsigned(s_y_delay(C_VIDEO_DELAY - 1));
            v_vu := unsigned(s_u_delay(C_VIDEO_DELAY - 1));
            v_vv := unsigned(s_v_delay(C_VIDEO_DELAY - 1));

            -- T7 Outline Color: Same vs Multi.
            --   Same    : both triangles use outline_hue.
            --   Multi   : triangle 1 uses outline_hue + 90° (hue shift).
            -- 10-bit unsigned + unsigned wraps mod 1024 = "+90° on hue wheel".
            if s_outline_multi = '1' and s_p4_on_outline_t1 = '1' then
                v_eff_hue := s_outline_hue + to_unsigned(256, 10);
            else
                v_eff_hue := s_outline_hue;
            end if;

            if s_p4_on_outline = '1' then
                if s_invert = '1' then
                    s_p5_y <= C_MAX_VAL - v_vy;
                    s_p5_u <= v_vu;
                    s_p5_v <= v_vv;
                else
                    s_p5_y <= C_OUTLINE_LUMA;
                    s_p5_u <= v_eff_hue;
                    s_p5_v <= C_MAX_VAL - v_eff_hue;
                end if;
            elsif s_p4_inside = '1' and s_filled = '1' then
                if s_video_mix > C_CHROMA_MID then
                    s_p5_y <= v_vy;
                    s_p5_u <= v_vu;
                    s_p5_v <= v_vv;
                else
                    s_p5_y <= C_FILL_LUMA;
                    s_p5_u <= s_fill_hue;
                    s_p5_v <= C_MAX_VAL - s_fill_hue;
                end if;
            else
                s_p5_y <= C_BG_LUMA;
                s_p5_u <= s_bg_hue;
                s_p5_v <= C_MAX_VAL - s_bg_hue;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P6, P7: pipeline slack between mux and IO output.
    --------------------------------------------------------------------------
    p_p67 : process(clk)
    begin
        if rising_edge(clk) then
            s_p6_y <= s_p5_y; s_p6_u <= s_p5_u; s_p6_v <= s_p5_v;
            s_p7_y <= s_p6_y; s_p7_u <= s_p6_u; s_p7_v <= s_p6_v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Video pass-through delay + sync pipeline.
    --------------------------------------------------------------------------
    p_delay_pipes : process(clk)
    begin
        if rising_edge(clk) then
            s_sync_pipe(0) <= data_in.field_n & data_in.avid &
                              data_in.vsync_n & data_in.hsync_n;
            for i in 1 to C_RENDER_DELAY - 1 loop
                s_sync_pipe(i) <= s_sync_pipe(i - 1);
            end loop;

            s_y_delay(0) <= data_in.y;
            s_u_delay(0) <= data_in.u;
            s_v_delay(0) <= data_in.v;
            for i in 1 to C_VIDEO_DELAY - 1 loop
                s_y_delay(i) <= s_y_delay(i - 1);
                s_u_delay(i) <= s_u_delay(i - 1);
                s_v_delay(i) <= s_v_delay(i - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- IO Alignment (render(9) + IO(2) = 11 clocks total).
    --------------------------------------------------------------------------
    p_io_align : process(clk)
    begin
        if rising_edge(clk) then
            s_io_0.y       <= std_logic_vector(s_p7_y);
            s_io_0.u       <= std_logic_vector(s_p7_u);
            s_io_0.v       <= std_logic_vector(s_p7_v);
            s_io_0.hsync_n <= s_sync_pipe(C_RENDER_DELAY - 1)(0);
            s_io_0.vsync_n <= s_sync_pipe(C_RENDER_DELAY - 1)(1);
            s_io_0.avid    <= s_sync_pipe(C_RENDER_DELAY - 1)(2);
            s_io_0.field_n <= s_sync_pipe(C_RENDER_DELAY - 1)(3);
            s_io_1 <= s_io_0;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output
    --------------------------------------------------------------------------
    data_out.y       <= s_io_1.y;
    data_out.u       <= s_io_1.u;
    data_out.v       <= s_io_1.v;
    data_out.hsync_n <= s_io_1.hsync_n;
    data_out.vsync_n <= s_io_1.vsync_n;
    data_out.avid    <= s_io_1.avid;
    data_out.field_n <= s_io_1.field_n;

end architecture tessera;
