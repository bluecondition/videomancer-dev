-- Laser Cathedral: ROYGBIV stage-laser parabolic-arc generator (Phase 1).
--
-- Seven horizontal arcs, evenly stacked vertically, with curvature varying
-- linearly from frowning at the top through straight at the middle to
-- smiling at the bottom: c_i = (i - 3) for i in 0..6. Each arc's column-y
-- is y_base_i + (i-3) * (k * (x - cx)^2), so all seven arcs share one
-- parabola family scaled by the curvature index — only one square is
-- evaluated per pixel and the per-arc bend reduces to shift+add+negate.
--
-- Origination dots ride on each arc at fixed horizontal intervals (Phase
-- 1 hardcodes ~15 dots per arc; Phase 2 will add the dot-count knob and
-- the perpendicular-fan beams that originate from each dot).
--
-- Rainbow palette is Y-position indexed via an 8-entry ROYGBIV ROM with
-- 3-bit base + 7-bit fractional interpolation (pattern borrowed from
-- oracle.vhd).
--
-- Optimised for iCE40 HX4K (no DSP blocks): only one fabric multiply per
-- pixel (the squared distance), all per-arc deltas are ±1/±2/±3 of one
-- shared `bend` value (free shift+add), per-frame derived constants
-- absorb the rest of the multiplies via a vsync-paced helper pipeline.
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Curvature   (0..1023, top 3 bits select shift amount)
--   registers_in(1) = Thickness   (0..1023 -> 1..32 px arc half-width)
--   registers_in(2) = Spacing     (0..1023 -> arc vertical pitch fraction)
--   registers_in(3) = Hue Scale   (top 3 bits: 1..8 rainbow cycles)
--   registers_in(4) = Hue Offset  (0..1023 -> rotate palette by phase)
--   registers_in(5) = Center X    (0..1023 -> parabola apex X position)
--   registers_in(6) = Switches    (b0 Anim b1 Xmir b2 Ymir b3 Inv b4 Sat)
--   registers_in(7) = Hue Speed   (0..1023 -> hue DDS rate when anim on)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture laser_cathedral of program_top is

    ----------------------------------------------------------------------
    -- Pipeline depth
    ----------------------------------------------------------------------
    constant LATENCY : natural := 18;  -- +5 for v2 beam-math stages

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    constant NUM_ARCS : natural := 7;

    type t_arc_y     is array (0 to NUM_ARCS - 1) of unsigned(11 downto 0);
    type t_arc_dy    is array (0 to NUM_ARCS - 1) of signed(13 downto 0);
    type t_arc_abs   is array (0 to NUM_ARCS - 1) of unsigned(12 downto 0);
    type t_arc_color is array (0 to NUM_ARCS - 1) of unsigned(9 downto 0);

    -- v2 beam-math types (tight widths — K capped at 2 to stay in budget)
    type t_arc_v4    is array (0 to NUM_ARCS - 1) of signed(3 downto 0);    -- (vx, vy) per arc
    type t_arc_v3u   is array (0 to NUM_ARCS - 1) of unsigned(2 downto 0);  -- k_3mt per arc (max K*3 = 6)
    type t_arc_t3    is array (0 to NUM_ARCS - 1) of signed(2 downto 0);
    type t_arc_dy_s  is array (0 to NUM_ARCS - 1) of signed(13 downto 0);   -- full-precision signed dy
    type t_arc_p17   is array (0 to NUM_ARCS - 1) of signed(16 downto 0);   -- 13x4 mul (dy*vx)
    type t_arc_p12   is array (0 to NUM_ARCS - 1) of signed(11 downto 0);   -- 8x4 mul (dx*vy)
    type t_arc_cross is array (0 to NUM_ARCS - 1) of signed(17 downto 0);   -- cross result, headroom

    ----------------------------------------------------------------------
    -- Rainbow palette (ROYGBIV + wrap) in 10-bit YUV.
    -- Hardware-verified (U/V swapped vs. BT.601), copied from Phosphor.
    ----------------------------------------------------------------------
    type t_pal10 is array (0 to 7) of unsigned(9 downto 0);

    constant C_PAL_Y : t_pal10 := (
        to_unsigned(328, 10),   -- 0 Red
        to_unsigned(584, 10),   -- 1 Orange
        to_unsigned(840, 10),   -- 2 Yellow
        to_unsigned(580, 10),   -- 3 Green
        to_unsigned(164, 10),   -- 4 Blue
        to_unsigned(192, 10),   -- 5 Indigo
        to_unsigned(349, 10),   -- 6 Violet
        to_unsigned(328, 10));  -- 7 Red wrap

    constant C_PAL_U : t_pal10 := (
        to_unsigned(960, 10), to_unsigned(772, 10), to_unsigned(584, 10),
        to_unsigned(136, 10), to_unsigned(440, 10), to_unsigned(608, 10),
        to_unsigned(756, 10), to_unsigned(960, 10));

    constant C_PAL_V : t_pal10 := (
        to_unsigned(360, 10), to_unsigned(212, 10), to_unsigned( 64, 10),
        to_unsigned(216, 10), to_unsigned(960, 10), to_unsigned(696, 10),
        to_unsigned(852, 10), to_unsigned(360, 10));

    constant C_NEUTRAL : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- Position tracking
    ----------------------------------------------------------------------
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal last_x_r     : unsigned(11 downto 0) := (others => '0');
    signal last_y_r     : unsigned(11 downto 0) := (others => '0');
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal max_y_r      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Knob/switch latches (sampled at vsync to keep timing off the
    -- user-control path; identical pattern to oracle.vhd).
    ----------------------------------------------------------------------
    signal curvature_r  : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal thickness_r  : unsigned(9 downto 0) := to_unsigned(200, 10);
    signal spacing_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal hue_scale_r  : unsigned(9 downto 0) := to_unsigned(0, 10);
    signal hue_offset_r : unsigned(9 downto 0) := (others => '0');
    signal center_x_r   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal hue_speed_r  : unsigned(9 downto 0) := (others => '0');
    signal anim_en_r    : std_logic := '0';
    signal xmir_en_r    : std_logic := '0';
    signal ymir_en_r    : std_logic := '0';
    signal invert_en_r  : std_logic := '0';
    signal satur_en_r   : std_logic := '0';

    ----------------------------------------------------------------------
    -- Per-frame derived constants (computed during vblank).
    -- cx = max_x * center_x / 1024, mapped so center_x=512 -> screen midline.
    -- arc_pitch = max_y / pitch_div (pitch_div from spacing knob).
    -- thickness_px = 1 + thickness_r >> 5  (1..32 px half-width).
    -- dot_period_mask, dot_radius derive from screen width.
    -- hue_step_per_line = (1024 * cycles) / max_y, accumulated per scanline.
    ----------------------------------------------------------------------
    signal cx_mul_r       : unsigned(21 downto 0) := (others => '0');
    signal cx_r           : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal cy_r           : unsigned(11 downto 0) := to_unsigned(540, 12);

    -- Arc Y bases (computed at vsync, used at every pixel). The spacing
    -- multiply is split across two cycles via pitch_mul_r so the 11x12
    -- mul stays off the per-pixel critical path.
    signal pitch_mul_r    : unsigned(22 downto 0) := (others => '0');
    signal arc_pitch_r    : unsigned(11 downto 0) := to_unsigned(120, 12);
    signal y_base_r       : t_arc_y := (others => (others => '0'));

    -- Per-arc colors (one ROYGBIV slot per arc, rotated by Color Shift).
    -- Computed once per vsync from C_PAL_*; per-pixel logic just indexes
    -- by arc number when an arc fires.
    signal arc_color_y_r : t_arc_color := (others => to_unsigned(512, 10));
    signal arc_color_u_r : t_arc_color := (others => to_unsigned(512, 10));
    signal arc_color_v_r : t_arc_color := (others => to_unsigned(512, 10));

    -- Color rotation: K4 picks a manual offset (0..6), animation adds a
    -- DDS-paced extra offset that advances when T7 (Animation) is on.
    signal color_shift_r      : unsigned(2 downto 0) := (others => '0');
    signal color_anim_acc_r   : unsigned(15 downto 0) := (others => '0');
    signal color_anim_shift_r : unsigned(2 downto 0) := (others => '0');

    -- Background is fixed black (K5 will become Rotation in Phase 2).

    -- Curvature: smooth multiplier across the full knob range. Stage 3
    -- computes bend = (x_sq_top14 * curvature_r) >> 8 — a small fabric
    -- mul that yields ~1024 distinct bend values (smooth knob feel)
    -- with the same max bend as the old shift-10 setting (~900).

    -- Thickness in pixels (arc half-width); 1..32
    signal thickness_px_r : unsigned(5 downto 0) := to_unsigned(6, 6);

    -- Origination dots: dot_period is power-of-2 spacing (mask = period-1).
    -- For HD this is 128 px (15 dots across); for SD 64 px (11 dots).
    -- dot_period_r = mask + 1 — kept registered for the centered-distance
    -- subtract in stage 3 (need the full period, not just the mask).
    signal dot_period_mask_r : unsigned(11 downto 0) := to_unsigned(127, 12);
    signal dot_period_r      : unsigned(11 downto 0) := to_unsigned(128, 12);
    -- Octagonal dot threshold: max + min/2 < DOT_OCTA_THRESH gives an
    -- 8-px-diameter near-circle (constant per spec).
    constant DOT_OCTA_THRESH : unsigned(13 downto 0) := to_unsigned(5, 14);

    -- Color rotation animation step (per-frame increment when anim_en)
    signal hue_phase_step_r : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- v2 per-arc beam-math constants (computed at vsync).
    --   K = max signed dot column index = max_x / (2*128) = max_x >> 8
    --   t_int(i) = 3 - i (per-arc tilt: +3 top..-3 bottom)
    --   vy_3x_r(i) = -K * t_int(i)
    --   k_3mt_r(i) = K * (3 - |t_int(i)|)  ("edge-fan" term)
    --   beam_thresh_r = 4*K (cross-product threshold ≈ 2 px perpendicular)
    ----------------------------------------------------------------------
    constant C_T_INT : t_arc_t3 := (
        to_signed( 3, 3), to_signed( 2, 3), to_signed( 1, 3),
        to_signed( 0, 3),
        to_signed(-1, 3), to_signed(-2, 3), to_signed(-3, 3));
    constant C_3M_ABS_T : t_arc_v3u := (
        to_unsigned(0, 3), to_unsigned(1, 3), to_unsigned(2, 3),
        to_unsigned(3, 3),
        to_unsigned(2, 3), to_unsigned(1, 3), to_unsigned(0, 3));
    constant C_ABS_T_INT : t_arc_v3u := (
        to_unsigned(3, 3), to_unsigned(2, 3), to_unsigned(1, 3),
        to_unsigned(0, 3),
        to_unsigned(1, 3), to_unsigned(2, 3), to_unsigned(3, 3));
    signal beam_K_r       : unsigned(1 downto 0) := to_unsigned(2, 2);  -- capped at 2
    signal beam_thresh_r  : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal vy_3x_r        : t_arc_v4 := (others => to_signed(0, 4));
    signal k_3mt_r        : t_arc_v3u := (others => to_unsigned(0, 3));

    ----------------------------------------------------------------------
    -- Pipeline stage signals
    ----------------------------------------------------------------------

    -- s1: x effective + signed dx
    signal s1_dx_signed   : signed(13 downto 0) := (others => '0');
    signal s1_abs_dx      : unsigned(12 downto 0) := (others => '0');
    signal s1_y_eff       : unsigned(11 downto 0) := (others => '0');

    -- s2: squared distance + dot column phase
    signal s2_x_sq        : unsigned(25 downto 0) := (others => '0');
    signal s2_dx_mod      : unsigned(11 downto 0) := (others => '0');
    signal s2_y_eff       : unsigned(11 downto 0) := (others => '0');

    -- s3: per-pixel bend value + centered dot distance
    signal s3_bend        : unsigned(13 downto 0) := (others => '0');
    signal s3_dx_mod      : unsigned(11 downto 0) := (others => '0');
    signal s3_y_eff       : unsigned(11 downto 0) := (others => '0');

    -- s4: bend2/bend3 + palette ROM read
    signal s4_bend        : unsigned(13 downto 0) := (others => '0');
    signal s4_bend2       : unsigned(14 downto 0) := (others => '0');
    signal s4_bend3       : unsigned(15 downto 0) := (others => '0');
    signal s4_dx_mod      : unsigned(11 downto 0) := (others => '0');
    signal s4_y_eff       : unsigned(11 downto 0) := (others => '0');

    -- s5: per-arc Y centerline
    signal s5_y_arc       : t_arc_y := (others => (others => '0'));
    signal s5_y_eff       : unsigned(11 downto 0) := (others => '0');
    signal s5_dx_mod      : unsigned(11 downto 0) := (others => '0');

    -- s6: per-arc |dy| (already absolute via compare-and-subtract,
    -- not subtract-then-negate; saves a carry chain in stage 7).
    -- Plus s6_dy_sign(i): '0' if pixel above arc i, '1' if below — used
    -- by Phase 2 beams to pick beam direction relative to arc.
    signal s6_abs_dy      : t_arc_abs := (others => (others => '0'));
    signal s6_dy_sign     : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s6_dx_mod      : unsigned(11 downto 0) := (others => '0');

    -- s7: |dy| per arc + per-arc octagonal-disk-hit pre-compute
    signal s7_abs_dy      : t_arc_abs := (others => (others => '0'));
    signal s7_dy_sign     : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s7_dx_mod      : unsigned(11 downto 0) := (others => '0');
    signal s7_on_dot_pre  : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');

    -- v2 beam-math pipeline (5 new stages between s7 and s8). Cross-
    -- product test per arc per pixel using per-dot direction (vx, vy)
    -- derived from arc tilt and dot column index.

    -- Delay shift register for s1_dx_signed so it aligns with s7 outputs
    -- (s1 -> s7a needs 6 cycles of delay).
    type t_dx_pipe is array (0 to 5) of signed(13 downto 0);
    signal dx_pipe_r : t_dx_pipe := (others => to_signed(0, 14));

    -- s7a: signed dx_to_dot (centered nearest-dot offset, ±64 range, 8-bit)
    --      s_int (signed dot-column index, 3-bit with K_cap=2)
    --      signed_dy per arc — FULL precision (13-bit) so cross-product
    --      math stays consistent in pixel units.
    signal s7a_dx_to_dot  : signed(7 downto 0) := (others => '0');
    signal s7a_s_int      : signed(2 downto 0) := (others => '0');
    signal s7a_signed_dy  : t_arc_dy_s := (others => to_signed(0, 14));
    signal s7a_abs_dy     : t_arc_abs := (others => (others => '0'));
    signal s7a_dy_sign    : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s7a_dx_mod     : unsigned(11 downto 0) := (others => '0');
    signal s7a_on_dot_pre : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');

    -- s7b: per-arc vx_3x (per-pixel from s_int + per-arc consts)
    signal s7b_vx_3x      : t_arc_v4 := (others => to_signed(0, 4));
    signal s7b_signed_dy  : t_arc_dy_s := (others => to_signed(0, 14));
    signal s7b_dx_to_dot  : signed(7 downto 0) := (others => '0');
    signal s7b_abs_dy     : t_arc_abs := (others => (others => '0'));
    signal s7b_dy_sign    : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s7b_dx_mod     : unsigned(11 downto 0) := (others => '0');
    signal s7b_on_dot_pre : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');

    -- s7c: cross-product mults dy*vx (13x4=17-bit) and dx*vy (8x4=12-bit)
    signal s7c_dy_x_vx    : t_arc_p17 := (others => to_signed(0, 17));
    signal s7c_dx_x_vy    : t_arc_p12 := (others => to_signed(0, 12));
    signal s7c_abs_dy     : t_arc_abs := (others => (others => '0'));
    signal s7c_dy_sign    : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s7c_dx_mod     : unsigned(11 downto 0) := (others => '0');
    signal s7c_on_dot_pre : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');

    -- s7d: cross = dy*vx - dx*vy per arc (18-bit signed)
    signal s7d_cross      : t_arc_cross := (others => to_signed(0, 18));
    signal s7d_abs_dy     : t_arc_abs := (others => (others => '0'));
    signal s7d_dy_sign    : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s7d_dx_mod     : unsigned(11 downto 0) := (others => '0');
    signal s7d_on_dot_pre : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');

    -- s7e: |cross| and beam-direction-OK flag per arc
    signal s7e_abs_cross  : t_arc_cross := (others => to_signed(0, 18));
    signal s7e_dir_ok     : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s7e_abs_dy     : t_arc_abs := (others => (others => '0'));
    signal s7e_dx_mod     : unsigned(11 downto 0) := (others => '0');
    signal s7e_on_dot_pre : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');

    -- s8: per-arc on_arc + on_dot + on_beam (v2: cross-product test)
    signal s8_on_arc      : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s8_on_dot      : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s8_on_beam     : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');

    -- s8b (NEW): bisected priority colour pick. Splits the 7-deep colour
    -- mux into a 4-arc top half and a 3-arc bottom half running in
    -- parallel; stage 9 just OR-combines them. Without this split the
    -- HD targets fail timing (the 7-deep chain runs at ~55 MHz).
    signal s8b_top_pal_y  : unsigned(9 downto 0) := (others => '0');
    signal s8b_top_pal_u  : unsigned(9 downto 0) := C_NEUTRAL;
    signal s8b_top_pal_v  : unsigned(9 downto 0) := C_NEUTRAL;
    signal s8b_bot_pal_y  : unsigned(9 downto 0) := (others => '0');
    signal s8b_bot_pal_u  : unsigned(9 downto 0) := C_NEUTRAL;
    signal s8b_bot_pal_v  : unsigned(9 downto 0) := C_NEUTRAL;
    signal s8b_top_fired  : std_logic := '0';
    signal s8b_bot_fired  : std_logic := '0';
    signal s8b_on_arc     : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');
    signal s8b_on_dot     : std_logic_vector(0 to NUM_ARCS - 1) := (others => '0');

    -- s8b also computes beam contribution accumulator. Bisected into
    -- top (arcs 0..3) and bottom (arcs 4..6) halves to keep each
    -- per-half accumulator chain shallow enough for HD timing closure;
    -- stage 9 then sums the halves.
    signal s8b_beam_acc_top_y : unsigned(11 downto 0) := (others => '0');
    signal s8b_beam_acc_top_u : signed(12 downto 0) := (others => '0');
    signal s8b_beam_acc_top_v : signed(12 downto 0) := (others => '0');
    signal s8b_beam_acc_bot_y : unsigned(11 downto 0) := (others => '0');
    signal s8b_beam_acc_bot_u : signed(12 downto 0) := (others => '0');
    signal s8b_beam_acc_bot_v : signed(12 downto 0) := (others => '0');

    -- s9: reduce to any-hit + apply saturate (carries beam_acc forward)
    signal s9_on_arc_any  : std_logic := '0';
    signal s9_on_dot_any  : std_logic := '0';
    signal s9_pal_y       : unsigned(9 downto 0) := (others => '0');
    signal s9_pal_u       : unsigned(9 downto 0) := (others => '0');
    signal s9_pal_v       : unsigned(9 downto 0) := (others => '0');
    signal s9_beam_acc_y  : unsigned(12 downto 0) := (others => '0');
    signal s9_beam_acc_u  : signed(13 downto 0) := (others => '0');
    signal s9_beam_acc_v  : signed(13 downto 0) := (others => '0');

    -- s10: composed final pixel
    signal s10_y          : unsigned(9 downto 0) := (others => '0');
    signal s10_u          : unsigned(9 downto 0) := C_NEUTRAL;
    signal s10_v          : unsigned(9 downto 0) := C_NEUTRAL;

    -- s11: registered output buffer
    signal s11_y          : unsigned(9 downto 0) := (others => '0');
    signal s11_u          : unsigned(9 downto 0) := C_NEUTRAL;
    signal s11_v          : unsigned(9 downto 0) := C_NEUTRAL;

    ----------------------------------------------------------------------
    -- Helper: saturate a centered chroma value away from 512 by ~1.5x.
    -- Used for the T11 "Saturate / neon" boost.
    ----------------------------------------------------------------------
    function chroma_boost(c : unsigned(9 downto 0)) return unsigned is
        variable cs : signed(11 downto 0);
        variable boosted : signed(11 downto 0);
    begin
        cs := signed(resize(c, 12)) - to_signed(512, 12);
        boosted := cs + resize(shift_right(cs, 1), 12);  -- *1.5
        boosted := boosted + to_signed(512, 12);
        if boosted < 0 then
            return to_unsigned(0, 10);
        elsif boosted > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(std_logic_vector(boosted(9 downto 0)));
        end if;
    end function;

begin

    process(clk)
        variable v_pixel_x_eff   : unsigned(11 downto 0);
        variable v_pixel_y_eff   : unsigned(11 downto 0);
        variable v_dx            : signed(13 downto 0);
        variable v_bend_mul      : unsigned(15 downto 0);
        variable v_dx_diff       : unsigned(11 downto 0);
        variable v_arc_color_idx : integer range 0 to 7;
        variable v_color_sum     : unsigned(3 downto 0);
        variable v_max_xy        : unsigned(12 downto 0);
        variable v_min_xy        : unsigned(12 downto 0);
        variable v_octa          : unsigned(13 downto 0);
        variable v_s_int_raw     : signed(5 downto 0);
        variable v_beam_acc_y    : unsigned(12 downto 0);
        variable v_beam_acc_u    : signed(13 downto 0);
        variable v_beam_acc_v    : signed(13 downto 0);
        variable v_arc_u_centered : signed(10 downto 0);
        variable v_arc_v_centered : signed(10 downto 0);
        variable v_y_combined    : unsigned(11 downto 0);
        variable v_u_centered    : signed(11 downto 0);
        variable v_v_centered    : signed(11 downto 0);
        variable v_u_recentered  : signed(12 downto 0);
        variable v_v_recentered  : signed(12 downto 0);
    begin
        if rising_edge(clk) then

            ----------------------------------------------------------------
            -- Delay pipeline (sync signals + avid + original Y/U/V travel
            -- alongside the computed pixel so timing aligns at output).
            ----------------------------------------------------------------
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 0: Position tracking + per-frame parameter latch
            ----------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
                if pixel_x > last_x_r then
                    last_x_r <= pixel_x;
                end if;
            end if;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                pixel_x <= (others => '0');
                pixel_y <= pixel_y + 1;
                if pixel_y > last_y_r then
                    last_y_r <= pixel_y;
                end if;

            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                pixel_y     <= (others => '0');
                frame_count <= frame_count + 1;

                -- Latch screen extents (plus 1 to convert max-index to count)
                max_x_r <= last_x_r + 1;
                max_y_r <= last_y_r + 1;
                last_x_r <= (others => '0');
                last_y_r <= (others => '0');

                -- Sample knobs once per frame
                curvature_r  <= unsigned(registers_in(0));
                thickness_r  <= unsigned(registers_in(1));
                spacing_r    <= unsigned(registers_in(2));
                hue_scale_r  <= unsigned(registers_in(3));
                hue_offset_r <= unsigned(registers_in(4));
                center_x_r   <= unsigned(registers_in(5));
                hue_speed_r  <= unsigned(registers_in(7));
                anim_en_r    <= registers_in(6)(0);
                xmir_en_r    <= registers_in(6)(1);
                ymir_en_r    <= registers_in(6)(2);
                invert_en_r  <= registers_in(6)(3);
                satur_en_r   <= registers_in(6)(4);

                -- Color rotation animation: when T7 is on, advance the
                -- color-shift accumulator at K12-controlled rate. The top
                -- 3 bits become the extra rotation offset (mod 7).
                if anim_en_r = '1' then
                    color_anim_acc_r <= color_anim_acc_r +
                                        resize(hue_phase_step_r, 16);
                end if;
            end if;

            ----------------------------------------------------------------
            -- Per-frame derived constants (computed continuously, sampled
            -- by per-pixel logic). These adders/muxes happen ~once per
            -- vblank; there is no critical-path pressure.
            ----------------------------------------------------------------

            -- cx = max_x * center_x / 1024
            cx_mul_r <= max_x_r * center_x_r;
            cx_r     <= cx_mul_r(21 downto 10);
            cy_r     <= '0' & max_y_r(11 downto 1);  -- exactly half

            -- Curvature is now a smooth multiplier — see stage 3 for the
            -- per-pixel mul. No per-frame derivation needed here.

            -- Thickness fixed (independent of K3 spacing). 6 px half-width
            -- on HD, 3 px on SD, scaling only with screen height — never
            -- with arc pitch.
            if max_y_r >= to_unsigned(720, 12) then
                thickness_px_r <= to_unsigned(6, 6);
            else
                thickness_px_r <= to_unsigned(3, 6);
            end if;

            -- Arc spacing — smooth modifier across the full knob range.
            -- arc_pitch = (spacing_r + 768) * max_y >> 14, giving
            --   knob 0    : ~max_y / 21     (squished)
            --   knob 1023 : ~max_y / 9      (spread)
            -- Pipelined: pitch_mul_r holds the 11x12 product, arc_pitch_r
            -- selects bits next cycle. Two-cycle latency is invisible to
            -- per-pixel logic (arc_pitch_r is only resampled at vsync for
            -- y_base_r).
            pitch_mul_r <= (resize(spacing_r, 11) + to_unsigned(768, 11))
                            * max_y_r;
            arc_pitch_r <= resize(pitch_mul_r(21 downto 14), 12);

            -- Compute the 7 arc Y bases at vsync. Arcs centered around cy_r
            -- with index i giving offset (i - 3) * arc_pitch_r.
            for i in 0 to NUM_ARCS - 1 loop
                if i < 3 then
                    case 3 - i is
                        when 1 => y_base_r(i) <= cy_r - arc_pitch_r;
                        when 2 => y_base_r(i) <= cy_r - shift_left(arc_pitch_r, 1);
                        when others => y_base_r(i) <= cy_r -
                            (shift_left(arc_pitch_r, 1) + arc_pitch_r);
                    end case;
                elsif i = 3 then
                    y_base_r(i) <= cy_r;
                else
                    case i - 3 is
                        when 1 => y_base_r(i) <= cy_r + arc_pitch_r;
                        when 2 => y_base_r(i) <= cy_r + shift_left(arc_pitch_r, 1);
                        when others => y_base_r(i) <= cy_r +
                            (shift_left(arc_pitch_r, 1) + arc_pitch_r);
                    end case;
                end if;
            end loop;

            ----------------------------------------------------------------
            -- Per-arc colours (ROYGBIV one per arc, with K4 + animation
            -- rotation). We use mod-8 (free bit-mask) instead of mod-7;
            -- C_PAL_*(7) is a wrap-around duplicate of entry 0 (Red),
            -- so an 8-position rotation cycles R O Y G B I V R. The mod-7
            -- chain was the critical path on HX4K (~17 ns / 60 MHz).
            ----------------------------------------------------------------
            color_shift_r <= hue_scale_r(9 downto 7);

            for i in 0 to NUM_ARCS - 1 loop
                -- Mask to 3 bits => mod 8, then index into 8-entry palette
                v_color_sum := to_unsigned(i, 4) +
                               resize(color_shift_r, 4) +
                               resize(color_anim_shift_r, 4);
                v_arc_color_idx := to_integer(v_color_sum(2 downto 0));
                arc_color_y_r(i) <= C_PAL_Y(v_arc_color_idx);
                arc_color_u_r(i) <= C_PAL_U(v_arc_color_idx);
                arc_color_v_r(i) <= C_PAL_V(v_arc_color_idx);
            end loop;

            -- Color rotation animation: K12 (Shift Speed) advances a 16-bit
            -- accumulator at vsync; the top 3 bits feed an extra rotation
            -- offset that adds to color_shift_r when T7 (Animation) is on.
            -- No mod-7 clamp needed since per-arc indexing is mod-8.
            hue_phase_step_r <= shift_right(resize(hue_speed_r, 12), 4);
            color_anim_shift_r <= color_anim_acc_r(15 downto 13);

            -- v2 beam math constants (per-frame derived from screen size).
            -- K capped at 2 to fit HX4K LC budget. Outer dots beyond
            -- column ±2 from cx have saturated direction (no per-dot
            -- variation past that point). Inner dots still fan correctly.
            beam_K_r <= to_unsigned(2, 2);
            -- Threshold ~2-3 px perpendicular: 8 * K (= 16 with K=2)
            beam_thresh_r <= resize(shift_left(beam_K_r, 3), 7);
            for i in 0 to NUM_ARCS - 1 loop
                vy_3x_r(i) <= resize(-resize(C_T_INT(i), 4) *
                                     signed(resize(beam_K_r, 4)), 4);
                k_3mt_r(i) <= resize(beam_K_r * C_3M_ABS_T(i), 3);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 1: mirror + dx + per-line hue snapshot
            ----------------------------------------------------------------
            if xmir_en_r = '1' then
                v_pixel_x_eff := (max_x_r - 1) - pixel_x;
            else
                v_pixel_x_eff := pixel_x;
            end if;
            if ymir_en_r = '1' then
                v_pixel_y_eff := (max_y_r - 1) - pixel_y;
            else
                v_pixel_y_eff := pixel_y;
            end if;

            v_dx := signed(resize(v_pixel_x_eff, 14)) -
                    signed(resize(cx_r, 14));
            s1_dx_signed <= v_dx;
            if v_dx < 0 then
                s1_abs_dx <= unsigned(std_logic_vector(resize(-v_dx, 13)));
            else
                s1_abs_dx <= unsigned(std_logic_vector(resize(v_dx, 13)));
            end if;
            s1_y_eff   <= v_pixel_y_eff;

            ----------------------------------------------------------------
            -- STAGE 2: squared distance + raw dot-column modulus
            ----------------------------------------------------------------
            -- Single fabric multiply per pixel (13b * 13b = 26b).
            -- Inputs are registered (s1_abs_dx came from a flop), output is
            -- registered here, so the multiplier sees a clean register-to
            -- -register path. yosys packs this into LUT-based partial
            -- products on the HX4K (no DSPs).
            s2_x_sq   <= s1_abs_dx * s1_abs_dx;
            s2_dx_mod <= s1_abs_dx(11 downto 0) and dot_period_mask_r;
            s2_y_eff  <= s1_y_eff;

            ----------------------------------------------------------------
            -- STAGE 3: smooth bend = (x_sq_top8 * K1_top8) >> 5; also
            -- the centered dot-column distance.
            ----------------------------------------------------------------
            -- 8-bit * 8-bit = 16-bit fabric multiply. K1 top-8 gives 256
            -- knob positions (smooth feel — the bottom 2 K1 bits are
            -- dropped). At HD edge x the max bend lands at ~900, matching
            -- the old K1-100% setting; SD edge ~120. The small mul keeps
            -- stage 3 well under 13.4 ns for HD timing closure.
            v_bend_mul := s2_x_sq(20 downto 13) * curvature_r(9 downto 2);
            s3_bend <= resize(v_bend_mul(15 downto 5), 14);

            -- Centered distance to nearest dot column:
            --   dx_mod  in [0 .. period-1]
            --   dx_diff = period - dx_mod, in [1 .. period]
            --   centered = min(dx_mod, dx_diff)
            -- This makes a dot a true circle (centered) rather than a
            -- one-sided lump as in the previous straight-modulus form.
            v_dx_diff := dot_period_r - s2_dx_mod;
            if s2_dx_mod < v_dx_diff then
                s3_dx_mod <= s2_dx_mod;
            else
                s3_dx_mod <= v_dx_diff;
            end if;
            s3_y_eff <= s2_y_eff;

            ----------------------------------------------------------------
            -- STAGE 4: bend2/bend3 (no palette ROM read — colors are
            -- per-arc and live in arc_color_*_r, indexed in stage 8).
            ----------------------------------------------------------------
            s4_bend   <= s3_bend;
            s4_bend2  <= shift_left(resize(s3_bend, 15), 1);
            s4_bend3  <= resize(shift_left(resize(s3_bend, 16), 1), 16) +
                         resize(s3_bend, 16);
            s4_dx_mod <= s3_dx_mod;
            s4_y_eff  <= s3_y_eff;

            ----------------------------------------------------------------
            -- STAGE 5: per-arc Y centerline
            ----------------------------------------------------------------
            -- delta_0 = -bend3, _1 = -bend2, _2 = -bend, _3 = 0,
            -- delta_4 = +bend,  _5 = +bend2, _6 = +bend3
            s5_y_arc(0) <= y_base_r(0) - resize(s4_bend3(11 downto 0), 12);
            s5_y_arc(1) <= y_base_r(1) - resize(s4_bend2(11 downto 0), 12);
            s5_y_arc(2) <= y_base_r(2) - resize(s4_bend(11 downto 0), 12);
            s5_y_arc(3) <= y_base_r(3);
            s5_y_arc(4) <= y_base_r(4) + resize(s4_bend(11 downto 0), 12);
            s5_y_arc(5) <= y_base_r(5) + resize(s4_bend2(11 downto 0), 12);
            s5_y_arc(6) <= y_base_r(6) + resize(s4_bend3(11 downto 0), 12);
            s5_y_eff    <= s4_y_eff;
            s5_dx_mod   <= s4_dx_mod;

            ----------------------------------------------------------------
            -- STAGE 6: per-arc |dy| + dy sign (above/below arc).
            ----------------------------------------------------------------
            for i in 0 to NUM_ARCS - 1 loop
                if s5_y_eff >= s5_y_arc(i) then
                    s6_abs_dy(i)  <= resize(s5_y_eff - s5_y_arc(i), 13);
                    s6_dy_sign(i) <= '1';  -- pixel below this arc
                else
                    s6_abs_dy(i)  <= resize(s5_y_arc(i) - s5_y_eff, 13);
                    s6_dy_sign(i) <= '0';  -- pixel above this arc
                end if;
            end loop;
            s6_dx_mod <= s5_dx_mod;

            ----------------------------------------------------------------
            -- STAGE 7: pass through |dy| + dy_sign, compute octagonal-disk hit
            ----------------------------------------------------------------
            for i in 0 to NUM_ARCS - 1 loop
                s7_abs_dy(i)  <= s6_abs_dy(i);
                s7_dy_sign(i) <= s6_dy_sign(i);

                -- Octagonal disk: max(dx_dot_dist, |dy|) + min/2 < 5
                if s6_dx_mod >= resize(s6_abs_dy(i), 12) then
                    v_max_xy := resize(s6_dx_mod, 13);
                    v_min_xy := resize(s6_abs_dy(i), 13);
                else
                    v_max_xy := resize(s6_abs_dy(i), 13);
                    v_min_xy := resize(s6_dx_mod, 13);
                end if;
                v_octa := resize(v_max_xy, 14) +
                          resize(shift_right(v_min_xy, 1), 14);
                if v_octa < DOT_OCTA_THRESH then
                    s7_on_dot_pre(i) <= '1';
                else
                    s7_on_dot_pre(i) <= '0';
                end if;
            end loop;
            s7_dx_mod <= s6_dx_mod;

            ----------------------------------------------------------------
            -- v2 BEAM-MATH STAGES (s7a..s7e). Cross-product test per arc:
            --   cross = signed_dy * vx_3x  -  signed_dx_to_dot * vy_3x
            -- Pixel "on beam" iff |cross| < beam_thresh AND beam direction
            -- relative to arc is forward (using existing dy_sign).
            ----------------------------------------------------------------

            -- Delay shift register so signed_dx aligns with s7's outputs
            -- when consumed at s7a. dx_pipe_r(0) is s1_dx_signed delayed
            -- by 1 cycle; (5) is delayed by 6 cycles -> arrives at s7a.
            dx_pipe_r(0) <= s1_dx_signed;
            for i in 1 to 5 loop
                dx_pipe_r(i) <= dx_pipe_r(i - 1);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 7a: extract signed dx_to_dot and s_int from delayed
            -- signed_dx; derive truncated signed_dy per arc from abs_dy.
            ----------------------------------------------------------------
            -- dx_to_dot = bottom 7 bits of signed_dx as 7-bit signed,
            -- sign-extended to 8-bit (range -64..+63).
            s7a_dx_to_dot <= resize(signed(std_logic_vector(dx_pipe_r(5)(6 downto 0))), 8);
            -- s_int = signed_dx >> 7, SATURATED to ±2 (matches K_cap=2,
            -- prevents vx overflow in stage 7b). Without explicit saturation
            -- the 3-bit clip to -4..+3 wraps and produces wrong directions.
            v_s_int_raw := resize(shift_right(dx_pipe_r(5), 7), 6);
            if v_s_int_raw > to_signed(2, 6) then
                s7a_s_int <= to_signed(2, 3);
            elsif v_s_int_raw < to_signed(-2, 6) then
                s7a_s_int <= to_signed(-2, 3);
            else
                s7a_s_int <= resize(v_s_int_raw, 3);
            end if;

            -- signed_dy per arc: derive from abs_dy + dy_sign at FULL
            -- precision (13-bit). Truncating breaks the cross-product
            -- math because dy*vx and dx*vy end up in different units.
            for i in 0 to NUM_ARCS - 1 loop
                if s7_dy_sign(i) = '1' then
                    s7a_signed_dy(i) <= signed(resize(s7_abs_dy(i), 14));
                else
                    s7a_signed_dy(i) <= -signed(resize(s7_abs_dy(i), 14));
                end if;
            end loop;

            -- Plumb the rest forward
            s7a_abs_dy     <= s7_abs_dy;
            s7a_dy_sign    <= s7_dy_sign;
            s7a_dx_mod     <= s7_dx_mod;
            s7a_on_dot_pre <= s7_on_dot_pre;

            ----------------------------------------------------------------
            -- STAGE 7b: compute vx per arc (signed 4-bit, range -8..+7).
            -- vx(i) = s_int * |t_int(i)| + sign(s_int) * k_3mt_r(i)
            ----------------------------------------------------------------
            for i in 0 to NUM_ARCS - 1 loop
                if s7a_s_int(2) = '1' then
                    s7b_vx_3x(i) <= resize(s7a_s_int * signed(resize(C_ABS_T_INT(i), 4)), 4) -
                                    resize(signed(resize(k_3mt_r(i), 4)), 4);
                else
                    s7b_vx_3x(i) <= resize(s7a_s_int * signed(resize(C_ABS_T_INT(i), 4)), 4) +
                                    resize(signed(resize(k_3mt_r(i), 4)), 4);
                end if;
            end loop;

            -- Plumb forward
            s7b_signed_dy  <= s7a_signed_dy;
            s7b_dx_to_dot  <= s7a_dx_to_dot;
            s7b_abs_dy     <= s7a_abs_dy;
            s7b_dy_sign    <= s7a_dy_sign;
            s7b_dx_mod     <= s7a_dx_mod;
            s7b_on_dot_pre <= s7a_on_dot_pre;

            ----------------------------------------------------------------
            -- STAGE 7c: compute mults dy*vx and dx*vy per arc.
            -- 14 multiplies in parallel (7 arcs × 2). Worst case 14x6 mul.
            ----------------------------------------------------------------
            for i in 0 to NUM_ARCS - 1 loop
                s7c_dy_x_vx(i) <= resize(s7b_signed_dy(i) * s7b_vx_3x(i), 17);
                s7c_dx_x_vy(i) <= resize(s7b_dx_to_dot * vy_3x_r(i), 12);
            end loop;
            s7c_abs_dy     <= s7b_abs_dy;
            s7c_dy_sign    <= s7b_dy_sign;
            s7c_dx_mod     <= s7b_dx_mod;
            s7c_on_dot_pre <= s7b_on_dot_pre;

            ----------------------------------------------------------------
            -- STAGE 7d: cross = dy*vx - dx*vy per arc
            ----------------------------------------------------------------
            for i in 0 to NUM_ARCS - 1 loop
                s7d_cross(i) <= resize(s7c_dy_x_vx(i), 18) - resize(s7c_dx_x_vy(i), 18);
            end loop;
            s7d_abs_dy     <= s7c_abs_dy;
            s7d_dy_sign    <= s7c_dy_sign;
            s7d_dx_mod     <= s7c_dx_mod;
            s7d_on_dot_pre <= s7c_on_dot_pre;

            ----------------------------------------------------------------
            -- STAGE 7e: |cross| per arc; direction-OK flag from dy_sign.
            -- Direction rules:
            --   arc i < 3 (top half):  beam goes up, pixel must be above arc
            --                          (dy_sign='0')
            --   arc i > 3 (bot half):  beam goes down, pixel must be below
            --                          (dy_sign='1')
            --   arc i == 3 (center):  always OK (handled separately in s8)
            ----------------------------------------------------------------
            for i in 0 to NUM_ARCS - 1 loop
                if s7d_cross(i)(17) = '1' then
                    s7e_abs_cross(i) <= -s7d_cross(i);
                else
                    s7e_abs_cross(i) <= s7d_cross(i);
                end if;
                if i < 3 then
                    s7e_dir_ok(i) <= NOT s7d_dy_sign(i);
                elsif i > 3 then
                    s7e_dir_ok(i) <= s7d_dy_sign(i);
                else
                    s7e_dir_ok(i) <= '1';  -- center arc handled separately
                end if;
            end loop;
            s7e_abs_dy     <= s7d_abs_dy;
            s7e_dx_mod     <= s7d_dx_mod;
            s7e_on_dot_pre <= s7d_on_dot_pre;

            ----------------------------------------------------------------
            -- STAGE 8: per-arc on_arc + on_dot + on_beam (Phase 2 v2).
            -- on_beam now uses the cross-product test from s7e:
            --   non-center arcs: |cross| < beam_thresh AND direction OK
            --   center arc:      simple horizontal-band test (|dy(3)| < 2)
            ----------------------------------------------------------------
            for i in 0 to NUM_ARCS - 1 loop
                if s7e_abs_dy(i) < resize(thickness_px_r, 13) then
                    s8_on_arc(i) <= '1';
                else
                    s8_on_arc(i) <= '0';
                end if;
                s8_on_dot(i) <= s7e_on_dot_pre(i);

                if i = 3 then
                    -- Center arc: horizontal beams cover the whole arc-3
                    -- band uniformly (matches on_arc(3) so we don't get
                    -- an inner-stripe artifact within the band).
                    if s7e_abs_dy(3) < resize(thickness_px_r, 13) then
                        s8_on_beam(3) <= '1';
                    else
                        s8_on_beam(3) <= '0';
                    end if;
                else
                    -- Non-center arcs: cross-product beam test.
                    if s7e_abs_cross(i) < signed(resize(beam_thresh_r, 18)) and
                       s7e_dir_ok(i) = '1' then
                        s8_on_beam(i) <= '1';
                    else
                        s8_on_beam(i) <= '0';
                    end if;
                end if;
            end loop;

            ----------------------------------------------------------------
            -- STAGE 8b: bisected priority colour pick.
            -- Two parallel 4-deep / 3-deep priority chains; stage 9 then
            -- OR-combines the halves. Keeps either half's depth shallow
            -- enough to close at 74.25 MHz on the HX4K.
            ----------------------------------------------------------------
            -- Top half (arcs 0..3) — top-most fired arc wins
            s8b_top_pal_y <= (others => '0');
            s8b_top_pal_u <= C_NEUTRAL;
            s8b_top_pal_v <= C_NEUTRAL;
            s8b_top_fired <= '0';
            for i in 3 downto 0 loop
                if s8_on_arc(i) = '1' or s8_on_dot(i) = '1' then
                    s8b_top_pal_y <= arc_color_y_r(i);
                    s8b_top_pal_u <= arc_color_u_r(i);
                    s8b_top_pal_v <= arc_color_v_r(i);
                    s8b_top_fired <= '1';
                end if;
            end loop;

            -- Bottom half (arcs 4..6) — top-most fired arc wins
            s8b_bot_pal_y <= (others => '0');
            s8b_bot_pal_u <= C_NEUTRAL;
            s8b_bot_pal_v <= C_NEUTRAL;
            s8b_bot_fired <= '0';
            for i in 6 downto 4 loop
                if s8_on_arc(i) = '1' or s8_on_dot(i) = '1' then
                    s8b_bot_pal_y <= arc_color_y_r(i);
                    s8b_bot_pal_u <= arc_color_u_r(i);
                    s8b_bot_pal_v <= arc_color_v_r(i);
                    s8b_bot_fired <= '1';
                end if;
            end loop;

            -- Plumb on_arc/on_dot vectors forward for stage 9's reduce
            s8b_on_arc <= s8_on_arc;
            s8b_on_dot <= s8_on_dot;

            -- Beam contribution accumulator (bisected into top + bot
            -- halves so neither chain exceeds ~4 cumulative adds).
            -- Top half (arcs 0..3)
            s8b_beam_acc_top_y <= (others => '0');
            s8b_beam_acc_top_u <= (others => '0');
            s8b_beam_acc_top_v <= (others => '0');
            for i in 0 to 3 loop
                if s8_on_beam(i) = '1' then
                    s8b_beam_acc_top_y <= s8b_beam_acc_top_y +
                                          resize(arc_color_y_r(i), 12);
                    s8b_beam_acc_top_u <= s8b_beam_acc_top_u +
                                          (signed(resize(arc_color_u_r(i), 11)) -
                                           to_signed(512, 11));
                    s8b_beam_acc_top_v <= s8b_beam_acc_top_v +
                                          (signed(resize(arc_color_v_r(i), 11)) -
                                           to_signed(512, 11));
                end if;
            end loop;

            -- Bot half (arcs 4..6)
            s8b_beam_acc_bot_y <= (others => '0');
            s8b_beam_acc_bot_u <= (others => '0');
            s8b_beam_acc_bot_v <= (others => '0');
            for i in 4 to 6 loop
                if s8_on_beam(i) = '1' then
                    s8b_beam_acc_bot_y <= s8b_beam_acc_bot_y +
                                          resize(arc_color_y_r(i), 12);
                    s8b_beam_acc_bot_u <= s8b_beam_acc_bot_u +
                                          (signed(resize(arc_color_u_r(i), 11)) -
                                           to_signed(512, 11));
                    s8b_beam_acc_bot_v <= s8b_beam_acc_bot_v +
                                          (signed(resize(arc_color_v_r(i), 11)) -
                                           to_signed(512, 11));
                end if;
            end loop;

            ----------------------------------------------------------------
            -- STAGE 9: combine top/bot halves + reduce + saturate (T11)
            ----------------------------------------------------------------
            s9_on_arc_any <= '0';
            s9_on_dot_any <= '0';
            for i in 0 to NUM_ARCS - 1 loop
                if s8b_on_arc(i) = '1' then
                    s9_on_arc_any <= '1';
                end if;
                if s8b_on_dot(i) = '1' then
                    s9_on_dot_any <= '1';
                end if;
            end loop;

            -- Combine: top half wins over bottom half if it fired, else
            -- bottom half wins, else background.
            if s8b_top_fired = '1' then
                s9_pal_y <= s8b_top_pal_y;
                if satur_en_r = '1' then
                    s9_pal_u <= chroma_boost(s8b_top_pal_u);
                    s9_pal_v <= chroma_boost(s8b_top_pal_v);
                else
                    s9_pal_u <= s8b_top_pal_u;
                    s9_pal_v <= s8b_top_pal_v;
                end if;
            elsif s8b_bot_fired = '1' then
                s9_pal_y <= s8b_bot_pal_y;
                if satur_en_r = '1' then
                    s9_pal_u <= chroma_boost(s8b_bot_pal_u);
                    s9_pal_v <= chroma_boost(s8b_bot_pal_v);
                else
                    s9_pal_u <= s8b_bot_pal_u;
                    s9_pal_v <= s8b_bot_pal_v;
                end if;
            else
                s9_pal_y <= (others => '0');
                s9_pal_u <= C_NEUTRAL;
                s9_pal_v <= C_NEUTRAL;
            end if;

            -- Sum the bisected beam-acc halves (single add per channel
            -- in stage 9; total beam contribution registered for stage 10).
            s9_beam_acc_y <= resize(s8b_beam_acc_top_y, 13) +
                             resize(s8b_beam_acc_bot_y, 13);
            s9_beam_acc_u <= resize(s8b_beam_acc_top_u, 14) +
                             resize(s8b_beam_acc_bot_u, 14);
            s9_beam_acc_v <= resize(s8b_beam_acc_top_v, 14) +
                             resize(s8b_beam_acc_bot_v, 14);

            ----------------------------------------------------------------
            -- STAGE 10: composite final pixel (arc/dot pixel + additive
            -- beam contribution; handles T10 invert).
            ----------------------------------------------------------------
            -- First compute the base arc/dot pixel as v_y_combined etc.
            -- Then ADD the beam_acc (translucent) on top, clamping.

            if invert_en_r = '0' then
                if s9_on_dot_any = '1' then
                    if s9_pal_y > to_unsigned(682, 10) then
                        v_y_combined := to_unsigned(1023, 12);
                    else
                        v_y_combined := resize(s9_pal_y +
                                               shift_right(s9_pal_y, 1), 12);
                    end if;
                    v_arc_u_centered := signed(resize(s9_pal_u, 11)) -
                                        to_signed(512, 11);
                    v_arc_v_centered := signed(resize(s9_pal_v, 11)) -
                                        to_signed(512, 11);
                elsif s9_on_arc_any = '1' then
                    v_y_combined := resize(s9_pal_y, 12);
                    v_arc_u_centered := signed(resize(s9_pal_u, 11)) -
                                        to_signed(512, 11);
                    v_arc_v_centered := signed(resize(s9_pal_v, 11)) -
                                        to_signed(512, 11);
                else
                    -- Background = black, neutral chroma
                    v_y_combined := (others => '0');
                    v_arc_u_centered := (others => '0');
                    v_arc_v_centered := (others => '0');
                end if;
            else
                if s9_on_dot_any = '1' then
                    v_y_combined := (others => '0');
                    v_arc_u_centered := (others => '0');
                    v_arc_v_centered := (others => '0');
                elsif s9_on_arc_any = '1' then
                    v_y_combined := resize(shift_right(s9_pal_y, 2), 12);
                    v_arc_u_centered := signed(resize(s9_pal_u, 11)) -
                                        to_signed(512, 11);
                    v_arc_v_centered := signed(resize(s9_pal_v, 11)) -
                                        to_signed(512, 11);
                else
                    -- Bright background, neutral chroma
                    v_y_combined := to_unsigned(768, 12);
                    v_arc_u_centered := (others => '0');
                    v_arc_v_centered := (others => '0');
                end if;
            end if;

            -- Additive beam mix: alpha = 100% per beam (no shift). Single
            -- beam pixels read at full saturation; multi-beam saturates
            -- and clamps. Diagnostic value to confirm beams render at all.
            v_y_combined := v_y_combined + resize(s9_beam_acc_y, 12);
            if v_y_combined > to_unsigned(1023, 12) then
                s10_y <= (others => '1');
            else
                s10_y <= v_y_combined(9 downto 0);
            end if;

            -- U: centered signed add, recenter, clamp
            v_u_centered := resize(v_arc_u_centered, 12) +
                            resize(s9_beam_acc_u, 12);
            v_u_recentered := resize(v_u_centered, 13) + to_signed(512, 13);
            if v_u_recentered < 0 then
                s10_u <= (others => '0');
            elsif v_u_recentered > 1023 then
                s10_u <= (others => '1');
            else
                s10_u <= unsigned(std_logic_vector(v_u_recentered(9 downto 0)));
            end if;

            -- V: same as U
            v_v_centered := resize(v_arc_v_centered, 12) +
                            resize(s9_beam_acc_v, 12);
            v_v_recentered := resize(v_v_centered, 13) + to_signed(512, 13);
            if v_v_recentered < 0 then
                s10_v <= (others => '0');
            elsif v_v_recentered > 1023 then
                s10_v <= (others => '1');
            else
                s10_v <= unsigned(std_logic_vector(v_v_recentered(9 downto 0)));
            end if;

            ----------------------------------------------------------------
            -- STAGE 11: registered output buffer
            ----------------------------------------------------------------
            s11_y <= s10_y;
            s11_u <= s10_u;
            s11_v <= s10_v;

            ----------------------------------------------------------------
            -- OUTPUT: sync signals and avid travel down pipe(); we override
            -- the Y/U/V fields with the computed pixel.
            ----------------------------------------------------------------
            data_out <= pipe(LATENCY - 1);
            data_out.y <= std_logic_vector(s11_y);
            data_out.u <= std_logic_vector(s11_u);
            data_out.v <= std_logic_vector(s11_v);

        end if;
    end process;

end architecture laser_cathedral;
