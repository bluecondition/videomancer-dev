-- Drape: freezes the scanline at a split point and drapes it downward with
-- a feathered fade to black. Spread stretches the frozen line outward from
-- centre the deeper you go into the fade region.
--
-- Concept:
--   Above the Split row the input passes through untouched. At split_y the
--   line is captured into three inferred BRAMs (Y/U/V). Every row below the
--   split replays that captured line. The lower fade_len rows blend toward
--   pure black (Y=0, U=V=512). When Spread > 0, the captured line is re-read
--   through an "enlarge-from-centre" transform whose strength ramps with
--   fade_alpha: source_x = centre + (display_x - centre) * (1 - ε), where
--   ε = (fade_alpha * spread_knob) >> 10. ε = 0 leaves the line unchanged;
--   ε → 1 collapses every read toward the centre column, stretching the
--   captured content outward.
--
-- Pipeline (LATENCY = 8 clocks from data_in to data_out):
--   T0  data_in                        : combinational bus
--   T1  s1_* : register y/u/v + sync, advance pixel_x/pixel_y. Update
--              per-frame geometry continuously (3-stage pipeline of
--              split_knob × max_y_r → split_y_r, fade_start_r, fade_shift_r,
--              centre_r). Update per-row state at every cycle from pixel_y
--              (is_above, fade_alpha, epsilon for stretch).
--   T2  s2_* : compute dx = pixel_x − centre_r; forward Y/U/V, row flags,
--              fade_alpha.
--   T3  s3_* : compute delta = (dx * epsilon_r) >> 10; rd_addr = pixel_x −
--              delta (natural clamp, see concept). Prepare wr_en/wr_addr.
--   T4  s4_* : inferred BRAM write + synchronous read → s4_lb_y/u/v.
--              Forward control signals. Capture 3 previous lb samples for
--              4-tap horizontal blur.
--   T5  s5_* : mux src = passthrough / frozen-line / black; optional 4-tap
--              box blur on lb before mux.
--   T6  s6_* : fade blend (Y→0, U/V→512). Three parallel 10×11 multiplies.
--   T7  s7_* : brightness multiply on Y (split from s6 to avoid chaining
--              two multiplies in one cycle).
--   T8  data_out : combinational mux of s7 vs latency-matched bypass.
--
-- Register map:
--   registers_in(0)   = K1 Split   (0-1023 → 0..max_y, y where capture happens)
--   registers_in(1)   = K2 Stretch (0-1023 → 0..max_y, y where stretch begins;
--                        clamped so stretch_y is always at least
--                        STRETCH_MIN_MARGIN rows below split_y)
--   registers_in(2)   = K3 Fade    (0-1023 → ramp slope, anchored at Stretch.
--                        alpha = (pixel_y − stretch_y) × slope(knob), sat
--                        1023. Slope from a 32-entry LUT, huge at knob 0
--                        (drape blacks out close to stretch) and small at
--                        knob max (gentle ramp over many rows). Top of
--                        drape at stretch is always alpha = 0; only the
--                        slope length changes.)
--   registers_in(3)   = K4 Twist   (0-1023 → centred at 512: left/right
--                        horizontal lean of the drape. Per-row offset that
--                        grows linearly with dy below split.)
--   registers_in(4)   = K5 Soft W  (0-1023 → Stretch-onset ramp width)
--   registers_in(5)   = K6 Mix (0 = original passthrough, 1023 = full
--                        processed; intermediate values cross-fade the
--                        original video against the drape-processed result)
--   registers_in(6)(0) = Blur switch (4-tap horizontal box on lb output)
--   registers_in(6)(1) = Curve switch (0 = Straight, 1 = Curved).
--                        Straight: factor = eps/(1+eps/256) via an
--                        interpolated LUT. Each column renders as a smooth
--                        diagonal line radiating from a focal point above
--                        the stretch row.
--                        Curved: factor = eps directly (saturated at 255).
--                        Columns fan out hyperbolically with depth — the
--                        deeper you go the faster they pull toward the
--                        edges.
--   registers_in(6)(2) = Soft switch (Off = stretch starts sharply at the
--                        Stretch line, On = eps gain ramps in over Soft W
--                        rows below the Stretch line so the onset of the
--                        spread effect is gradual instead of a visible kink)
--   registers_in(6)(3) = Curve+ (Curved-mode style:
--                        0 = Hard — fast hyperbolic saturation (default)
--                        1 = Soft — slower accumulation, smoother knob feel)
--   registers_in(6)(4) = Bypass (latency-matched passthrough)
--   registers_in(7)   = P12 Spread (0-1023 → horizontal outward stretch
--                        amount — the headline performance control)
--
-- BRAM usage: 3 × 2048×10 inferred single-port BRAMs (one per Y/U/V).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture drape of program_top is

    constant C_DATA_WIDTH : integer := 10;
    constant C_ADDR_BITS  : integer := 11;
    constant LATENCY      : integer := 11;
    constant C_NEUTRAL    : integer := 512;
    constant STRETCH_MIN_MARGIN : natural := 8;  -- rows the Stretch line must
                                                 -- sit below the Split line

    ----------------------------------------------------------------------------
    -- Straight-mode factor LUT. Input is a 10-bit linear eps = (dy * spread)
    -- >> 8; top 5 bits index the 32 entries, bottom 5 bits are a fraction
    -- used for linear interpolation between adjacent entries. Values are
    --   FACTOR_LUT(i) = round((i * 32) * 256 / ((i * 32) + 256))
    -- which implements factor = eps / (1 + eps/256). Applied in the
    -- per-pixel formula
    --   rd_addr = pixel_x - (pixel_x - centre) * factor / 256
    -- this gives display columns that trace straight radial lines from a
    -- focal point above the stretch row. Linear interpolation keeps the
    -- curve smooth per pixel_y instead of stepping between 32 plateaus.
    ----------------------------------------------------------------------------
    type t_factor_lut is array(0 to 63) of unsigned(7 downto 0);
    constant FACTOR_LUT : t_factor_lut := (
        to_unsigned(0,   8), to_unsigned(28,  8),
        to_unsigned(51,  8), to_unsigned(69,  8),
        to_unsigned(85,  8), to_unsigned(98,  8),
        to_unsigned(109, 8), to_unsigned(119, 8),
        to_unsigned(128, 8), to_unsigned(135, 8),
        to_unsigned(142, 8), to_unsigned(148, 8),
        to_unsigned(153, 8), to_unsigned(158, 8),
        to_unsigned(162, 8), to_unsigned(167, 8),
        to_unsigned(170, 8), to_unsigned(174, 8),
        to_unsigned(177, 8), to_unsigned(180, 8),
        to_unsigned(182, 8), to_unsigned(185, 8),
        to_unsigned(187, 8), to_unsigned(189, 8),
        to_unsigned(192, 8), to_unsigned(194, 8),
        to_unsigned(195, 8), to_unsigned(197, 8),
        to_unsigned(199, 8), to_unsigned(200, 8),
        to_unsigned(202, 8), to_unsigned(203, 8),
        -- Extended range (i=32..63) — same eps/(1+eps/256) formula, just
        -- larger eps so the asymptote toward 256 is approached more
        -- closely. Factor max climbs from 203 to 227 (≈89% collapse).
        to_unsigned(205, 8), to_unsigned(206, 8),
        to_unsigned(207, 8), to_unsigned(208, 8),
        to_unsigned(209, 8), to_unsigned(210, 8),
        to_unsigned(211, 8), to_unsigned(212, 8),
        to_unsigned(213, 8), to_unsigned(214, 8),
        to_unsigned(215, 8), to_unsigned(216, 8),
        to_unsigned(216, 8), to_unsigned(217, 8),
        to_unsigned(218, 8), to_unsigned(219, 8),
        to_unsigned(219, 8), to_unsigned(220, 8),
        to_unsigned(221, 8), to_unsigned(221, 8),
        to_unsigned(222, 8), to_unsigned(222, 8),
        to_unsigned(223, 8), to_unsigned(224, 8),
        to_unsigned(224, 8), to_unsigned(225, 8),
        to_unsigned(225, 8), to_unsigned(226, 8),
        to_unsigned(226, 8), to_unsigned(226, 8),
        to_unsigned(227, 8), to_unsigned(227, 8)
    );

    ----------------------------------------------------------------------------
    -- Fade slope LUT. Indexed by the top 5 bits of fade_knob. Values step
    -- down roughly geometrically from 1024 (≈ instant saturation — drape
    -- essentially all black) at knob = 0 down to 2 (≈ 512-row gentle ramp)
    -- at knob = max. The per-row compute just multiplies dy_stretch by the
    -- LUT value so the fade is always a soft ramp; the knob just sets how
    -- long that ramp is.
    ----------------------------------------------------------------------------
    type t_fade_slope_lut is array(0 to 31) of unsigned(9 downto 0);
    constant FADE_SLOPE_LUT : t_fade_slope_lut := (
        to_unsigned(1023, 10), to_unsigned(724, 10),
        to_unsigned( 512, 10), to_unsigned(362, 10),
        to_unsigned( 256, 10), to_unsigned(182, 10),
        to_unsigned( 128, 10), to_unsigned( 91, 10),
        to_unsigned(  64, 10), to_unsigned( 46, 10),
        to_unsigned(  32, 10), to_unsigned( 23, 10),
        to_unsigned(  16, 10), to_unsigned( 12, 10),
        to_unsigned(   9, 10), to_unsigned(  7, 10),
        to_unsigned(   6, 10), to_unsigned(  5, 10),
        to_unsigned(   4, 10), to_unsigned(  4, 10),
        to_unsigned(   3, 10), to_unsigned(  3, 10),
        to_unsigned(   3, 10), to_unsigned(  2, 10),
        to_unsigned(   2, 10), to_unsigned(  2, 10),
        to_unsigned(   2, 10), to_unsigned(  2, 10),
        to_unsigned(   2, 10), to_unsigned(  2, 10),
        to_unsigned(   2, 10), to_unsigned(  2, 10)
    );


    ----------------------------------------------------------------------------
    -- Position tracking and per-frame max capture
    ----------------------------------------------------------------------------
    signal pixel_x       : unsigned(11 downto 0) := (others => '0');
    signal pixel_y       : unsigned(11 downto 0) := (others => '0');
    signal max_x_r       : unsigned(11 downto 0) := to_unsigned(1280, 12);
    signal max_y_r       : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal prev_hsync_n  : std_logic := '1';
    signal prev_vsync_n  : std_logic := '1';

    ----------------------------------------------------------------------------
    -- Per-frame geometry, built continuously from max_y_r and knob values.
    -- 3-stage pipeline: multiply → shift → final clamp. Values settle a few
    -- cycles after any change and are then stable for the whole frame.
    ----------------------------------------------------------------------------
    signal split_mul_p    : unsigned(18 downto 0) := (others => '0');  -- 7×12
    signal stretch_mul_p  : unsigned(18 downto 0) := (others => '0');
    signal centre_p       : unsigned(10 downto 0) := to_unsigned(640, 11);

    signal split_y_q      : unsigned(11 downto 0) := to_unsigned(468, 12);
    signal stretch_y_q    : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal centre_q       : unsigned(10 downto 0) := to_unsigned(640, 11);

    signal split_y_r      : unsigned(11 downto 0) := to_unsigned(468, 12);
    signal stretch_y_r    : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal centre_r       : unsigned(10 downto 0) := to_unsigned(640, 11);

    ----------------------------------------------------------------------------
    -- Per-row state. pixel_y only changes at hsync edges, so these are stable
    -- during the active portion of each scanline even though we latch every
    -- cycle. Fade is a straight linear slope driven by fade_knob:
    --   alpha = (pixel_y - split_y) * fade_knob >> 9   saturated to 1023
    -- Knob = 0 disables the fade entirely; knob = max produces an almost
    -- immediate fade to black one row below split. The intermediate range is
    -- fully continuous (no power-of-2 stepping).
    ----------------------------------------------------------------------------
    signal row_is_above_r : std_logic := '1';
    signal row_dy_r       : unsigned(11 downto 0) := (others => '0');
    signal row_dy_stretch_r : unsigned(11 downto 0) := (others => '0');
    signal row_fade_slope_r : unsigned(9 downto 0) := (others => '0');
    signal row_fade_mul_r : unsigned(21 downto 0) := (others => '0');
    signal row_fade_a_r   : unsigned(9 downto 0) := (others => '0');

    -- Twist (K6): per-row horizontal offset = (knob_centred × dy_split) >> 7.
    -- knob_centred = K6(9..5) − 16, range −16..+15.
    -- dy_split = pixel_y − split_y.
    -- Offset added to rd_addr in Stage 3b so the entire drape leans left or
    -- right, with the lean growing as you go deeper below split.
    signal row_twist_mul_r    : signed(18 downto 0) := (others => '0');
    signal row_twist_offset_r : signed(11 downto 0) := (others => '0');
    signal row_spread_eff_r : unsigned(10 downto 0) := (others => '0');  -- 1.5×knob fits in 11 bits
    signal row_eps_mul    : unsigned(22 downto 0) := (others => '0');
    signal row_eps_raw_r  : unsigned(10 downto 0) := (others => '0');  -- linear eps, 11-bit
    -- Pipelined factor LUT reads + fraction + curved-mode eps
    signal row_factor_lo_r   : unsigned(7 downto 0) := (others => '0');
    signal row_factor_hi_r   : unsigned(7 downto 0) := (others => '0');
    signal row_eps_frac_r    : unsigned(4 downto 0) := (others => '0');
    signal row_eps_curved_r  : unsigned(7 downto 0) := (others => '0');
    signal row_curved_on_r   : std_logic := '0';
    signal row_eps_r      : unsigned(7 downto 0) := (others => '0');  -- final 8-bit factor

    -- Stretch-onset gain. Anchored at stretch_y. 0 above stretch, 1023 below.
    -- When Soft switch is on, the 0→1023 transition ramps over ~16..1024
    -- rows (controlled by Soft W / K5). Multiplied with row_eps_raw to
    -- smooth the onset of the stretch effect — removes the visible kink at
    -- the Stretch line where columns go from vertical to diagonal.
    signal row_stretch_gain_mul_r : unsigned(18 downto 0) := (others => '0');  -- 12×7
    signal row_stretch_gain_r     : unsigned(9 downto 0)  := (others => '0');
    -- Smoothed eps: row_eps_raw_r * row_stretch_gain_r / 1024, fed to LUT
    signal row_eps_smooth_mul_r   : unsigned(20 downto 0) := (others => '0');  -- 11×10
    signal row_eps_smoothed_r     : unsigned(10 downto 0) := (others => '0');


    ----------------------------------------------------------------------------
    -- Stage 1
    ----------------------------------------------------------------------------
    signal s1_y, s1_u, s1_v : unsigned(9 downto 0) := (others => '0');
    signal s1_pixel_x : unsigned(11 downto 0) := (others => '0');
    signal s1_avid    : std_logic := '0';

    ----------------------------------------------------------------------------
    -- Stage 2 (dx computed, carry control signals)
    ----------------------------------------------------------------------------
    signal s2_y, s2_u, s2_v : unsigned(9 downto 0) := (others => '0');
    signal s2_pixel_x  : unsigned(11 downto 0) := (others => '0');
    signal s2_dx       : signed(12 downto 0) := (others => '0');
    signal s2_avid     : std_logic := '0';
    signal s2_is_above : std_logic := '0';
    signal s2_fade_a   : unsigned(9 downto 0) := (others => '0');
    signal s2_eps      : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 3 (delta multiply registered; rd_addr resolved in stage 3b)
    ----------------------------------------------------------------------------
    signal s3_y, s3_u, s3_v : unsigned(9 downto 0) := (others => '0');
    signal s3_is_above : std_logic := '0';
    signal s3_fade_a   : unsigned(9 downto 0) := (others => '0');
    signal s3_avid     : std_logic := '0';
    signal s3_pixel_x  : unsigned(11 downto 0) := (others => '0');
    signal s3_delta_mul : signed(21 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 3b (rd_addr / wr_en / wr_addr settled)
    ----------------------------------------------------------------------------
    signal s3b_y, s3b_u, s3b_v : unsigned(9 downto 0) := (others => '0');
    signal s3b_is_above : std_logic := '0';
    signal s3b_fade_a   : unsigned(9 downto 0) := (others => '0');
    signal s3b_wr_en    : std_logic := '0';
    signal s3b_wr_addr  : unsigned(C_ADDR_BITS-1 downto 0) := (others => '0');
    signal s3b_rd_addr  : unsigned(C_ADDR_BITS-1 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 4 (BRAM output)
    ----------------------------------------------------------------------------
    type t_line_ram is array(0 to 2047) of std_logic_vector(9 downto 0);
    signal s_ram_y : t_line_ram := (others => (others => '0'));
    signal s_ram_u : t_line_ram := (others => (others => '0'));
    signal s_ram_v : t_line_ram := (others => (others => '0'));
    signal s4_lb_y_slv, s4_lb_u_slv, s4_lb_v_slv : std_logic_vector(9 downto 0) := (others => '0');

    signal s4_y, s4_u, s4_v : unsigned(9 downto 0) := (others => '0');
    signal s4_is_above : std_logic := '0';
    signal s4_fade_a   : unsigned(9 downto 0) := (others => '0');
    signal s4_blur_on  : std_logic := '0';

    -- 4-tap blur needs prev_1..prev_3 lb samples.
    signal s4_lb_y_d1, s4_lb_y_d2, s4_lb_y_d3 : unsigned(9 downto 0) := (others => '0');
    signal s4_lb_y_d4, s4_lb_y_d5, s4_lb_y_d6, s4_lb_y_d7 : unsigned(9 downto 0) := (others => '0');
    signal s4_lb_u_d1, s4_lb_u_d2, s4_lb_u_d3 : unsigned(9 downto 0) := (others => '0');
    signal s4_lb_u_d4, s4_lb_u_d5, s4_lb_u_d6, s4_lb_u_d7 : unsigned(9 downto 0) := (others => '0');
    signal s4_lb_v_d1, s4_lb_v_d2, s4_lb_v_d3 : unsigned(9 downto 0) := (others => '0');
    signal s4_lb_v_d4, s4_lb_v_d5, s4_lb_v_d6, s4_lb_v_d7 : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 4b — pre-registered half-sums for the 8-tap blur, so the BRAM
    -- read + 8-way add doesn't all land in one cycle. Each half is a
    -- 4-way add of 10-bit values (max 4092, fits in 12 bits).
    ----------------------------------------------------------------------------
    signal s4b_y, s4b_u, s4b_v : unsigned(9 downto 0) := (others => '0');
    signal s4b_is_above : std_logic := '0';
    signal s4b_fade_a   : unsigned(9 downto 0) := (others => '0');
    signal s4b_half_top_y, s4b_half_bot_y : unsigned(11 downto 0) := (others => '0');
    signal s4b_half_top_u, s4b_half_bot_u : unsigned(11 downto 0) := (others => '0');
    signal s4b_half_top_v, s4b_half_bot_v : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 5 (source mux + blur)
    ----------------------------------------------------------------------------
    signal s5_y, s5_u, s5_v : unsigned(9 downto 0) := (others => '0');
    signal s5_fade_a   : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 6 (fade blend to black)
    ----------------------------------------------------------------------------
    signal s6_y, s6_u, s6_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 7p (mix pre — register delta = wet − dry separately from the
    -- multiply below so the subtract-before-mul path doesn't chain)
    ----------------------------------------------------------------------------
    signal s7p_dry_y, s7p_dry_u, s7p_dry_v     : unsigned(9 downto 0) := (others => '0');
    signal s7p_delta_y, s7p_delta_u, s7p_delta_v : signed(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 7a (mix delta × knob — one signed 11×10 per channel)
    ----------------------------------------------------------------------------
    signal s7a_dry_y, s7a_dry_u, s7a_dry_v : unsigned(9 downto 0) := (others => '0');
    signal s7a_mul_y, s7a_mul_u, s7a_mul_v : signed(21 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Stage 7 (mix add + clamp — final output)
    ----------------------------------------------------------------------------
    signal s7_y, s7_u, s7_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Sync + bypass shift registers (LATENCY deep)
    ----------------------------------------------------------------------------
    type t_bit_sr is array(0 to LATENCY-1) of std_logic;
    signal s_hsync_sr : t_bit_sr := (others => '1');
    signal s_vsync_sr : t_bit_sr := (others => '1');
    signal s_field_sr : t_bit_sr := (others => '1');
    signal s_avid_sr  : t_bit_sr := (others => '0');

    type t_data_sr is array(0 to LATENCY-1) of std_logic_vector(9 downto 0);
    signal s_y_bypass : t_data_sr := (others => (others => '0'));
    signal s_u_bypass : t_data_sr := (others => (others => '0'));
    signal s_v_bypass : t_data_sr := (others => (others => '0'));

    signal s_bypass_on : std_logic := '0';

begin

    s_bypass_on <= registers_in(6)(4);

    ----------------------------------------------------------------------------
    -- Main pipeline
    ----------------------------------------------------------------------------
    main_p : process(clk)
        variable split_knob   : unsigned(9 downto 0);
        variable spread_knob  : unsigned(9 downto 0);
        variable fade_knob    : unsigned(9 downto 0);
        variable stretch_knob : unsigned(9 downto 0);
        variable mix_knob     : unsigned(9 downto 0);

        variable mix_dry_y    : unsigned(9 downto 0);
        variable mix_dry_u    : unsigned(9 downto 0);
        variable mix_dry_v    : unsigned(9 downto 0);
        variable mix_delta_y  : signed(10 downto 0);
        variable mix_delta_u  : signed(10 downto 0);
        variable mix_delta_v  : signed(10 downto 0);
        variable mix_sum_ys   : signed(11 downto 0);
        variable mix_sum_us   : signed(11 downto 0);
        variable mix_sum_vs   : signed(11 downto 0);
        variable soft_knob    : unsigned(9 downto 0);
        variable twist_knob   : unsigned(9 downto 0);
        variable blur_on      : std_logic;
        variable curved_on    : std_logic;
        variable soft_on      : std_logic;
        variable curve_soft   : std_logic;
        variable twist_signed_v : signed(5 downto 0);

        variable soft_width   : unsigned(5 downto 0);  -- 0..63, soft_knob(9:4)
        variable soft_slope   : unsigned(6 downto 0);  -- 1..64, slope per dy row
        variable src_mul_v    : unsigned(18 downto 0);

        variable lut_idx      : integer range 0 to 63;
        variable factor_lo    : unsigned(7 downto 0);
        variable factor_hi    : unsigned(7 downto 0);
        variable factor_delta : unsigned(7 downto 0);
        variable interp_mul   : unsigned(12 downto 0);
        variable interp_v     : unsigned(7 downto 0);
        variable frac_v       : unsigned(4 downto 0);

        variable fade_len_v   : unsigned(11 downto 0);
        variable fade_shift_v : unsigned(3 downto 0);

        variable py_minus_fs  : unsigned(12 downto 0);
        variable alpha_shift  : unsigned(18 downto 0);
        variable alpha_v      : unsigned(9 downto 0);
        variable fs_v         : signed(13 downto 0);

        variable delta_mul    : signed(21 downto 0);
        variable delta_v      : signed(13 downto 0);
        variable rd_addr_v    : signed(13 downto 0);

        variable blur_sum_y   : unsigned(12 downto 0);
        variable blur_sum_u   : unsigned(12 downto 0);
        variable blur_sum_v   : unsigned(12 downto 0);

        variable alpha_inv    : unsigned(9 downto 0);
        variable y_mul        : unsigned(19 downto 0);
        variable u_mul        : unsigned(19 downto 0);
        variable v_mul        : unsigned(19 downto 0);
        variable u_add        : unsigned(19 downto 0);
        variable v_add        : unsigned(19 downto 0);

    begin
        if rising_edge(clk) then
            -------------------------------------------------------------------
            -- Knob / switch reads
            -------------------------------------------------------------------
            split_knob   := unsigned(registers_in(0)(9 downto 0));
            stretch_knob := unsigned(registers_in(1)(9 downto 0));
            fade_knob    := unsigned(registers_in(2)(9 downto 0));
            twist_knob   := unsigned(registers_in(3)(9 downto 0));
            soft_knob    := unsigned(registers_in(4)(9 downto 0));
            mix_knob     := unsigned(registers_in(5)(9 downto 0));
            spread_knob  := unsigned(registers_in(7)(9 downto 0));
            blur_on      := registers_in(6)(0);
            curved_on    := registers_in(6)(1);
            soft_on      := registers_in(6)(2);
            curve_soft   := registers_in(6)(3);

            -------------------------------------------------------------------
            -- Pixel counters
            -------------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_x > to_unsigned(32, 12) then
                    max_x_r <= pixel_x;
                end if;
                pixel_x <= (others => '0');
                pixel_y <= pixel_y + 1;
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                if pixel_y > to_unsigned(32, 12) then
                    max_y_r <= pixel_y;
                end if;
                pixel_y <= (others => '0');
            end if;

            -------------------------------------------------------------------
            -- Per-frame geometry pipeline (P→Q→R, runs continuously). Only
            -- split_y needs scaling against max_y_r; fade is a slope driven
            -- straight off fade_knob so it needs no per-frame pre-compute.
            -------------------------------------------------------------------
            split_mul_p   <= split_knob(9 downto 3) * max_y_r;
            stretch_mul_p <= stretch_knob(9 downto 3) * max_y_r;
            centre_p      <= resize(shift_right(max_x_r, 1), 11);

            split_y_q   <= resize(shift_right(split_mul_p, 7), 12);
            stretch_y_q <= resize(shift_right(stretch_mul_p, 7), 12);
            centre_q    <= centre_p;

            split_y_r   <= split_y_q;
            -- Clamp Stretch: never allow the stretch line above (or on top
            -- of) Split. Always keep it at least STRETCH_MIN_MARGIN rows
            -- below Split so the capture row stays stretch-free.
            if stretch_y_q < (split_y_q + to_unsigned(STRETCH_MIN_MARGIN, 12)) then
                stretch_y_r <= split_y_q + to_unsigned(STRETCH_MIN_MARGIN, 12);
            else
                stretch_y_r <= stretch_y_q;
            end if;
            centre_r    <= centre_q;

            -------------------------------------------------------------------
            -- Per-row state. Fade is now a "reveal" knob, not a slope:
            --   above stretch_y : row_fade_a_r = 0 (passthrough, no fade)
            --   below stretch_y : row_fade_a_r = 1023 − fade_knob (uniform)
            -- So fade = 0 blanks the drape entirely (alpha = 1023 → all
            -- pixels pulled to the fade target of black); fade = 1023
            -- leaves the drape fully visible (alpha = 0).
            -------------------------------------------------------------------
            if pixel_y <= split_y_r then
                row_is_above_r <= '1';
                row_dy_r       <= (others => '0');
            else
                row_is_above_r <= '0';
                row_dy_r       <= pixel_y - split_y_r;
            end if;

            if pixel_y <= stretch_y_r then
                row_dy_stretch_r <= (others => '0');
            else
                row_dy_stretch_r <= pixel_y - stretch_y_r;
            end if;

            -- Fade ramp with knob-controlled slope. Always a soft ramp.
            -- Anchored at STRETCH (row_dy_stretch_r). Three pipeline stages:
            --   Stage A: register slope from LUT
            --   Stage B: dy_stretch × slope  (12 × 10, 22-bit product).
            --            When row_dy_stretch_r = 0 (above stretch) the
            --            product is 0 → alpha = 0 (passthrough region).
            --   Stage C: saturate → row_fade_a_r
            row_fade_slope_r <= FADE_SLOPE_LUT(to_integer(fade_knob(9 downto 5)));
            row_fade_mul_r   <= row_dy_stretch_r * row_fade_slope_r;

            if row_fade_mul_r(21 downto 10) /= to_unsigned(0, 12) then
                row_fade_a_r <= to_unsigned(1023, 10);
            else
                row_fade_a_r <= row_fade_mul_r(9 downto 0);
            end if;

            -- Twist (K6): per-row signed offset that grows with dy_split.
            -- twist_signed = (knob >> 5) − 16, range −16..+15 (knob 512 → 0).
            -- offset = (twist_signed × dy_split) >> 6, max ≈ ±252 pixels at
            -- the bottom of an HD drape with the knob fully turned (2× the
            -- previous range).
            twist_signed_v := signed('0' & std_logic_vector(twist_knob(9 downto 5)))
                            - to_signed(16, 6);
            row_twist_mul_r <= twist_signed_v
                             * signed('0' & std_logic_vector(row_dy_r));
            row_twist_offset_r <= resize(shift_right(row_twist_mul_r, 6), 12);

            -- Soft-edge transition at Split. When Soft switch (S9) is off,
            -- src_a snaps 0 → 1023 at split (hard cut, matches the original
            -- binary is_above/is_drape selection). When on, src_a ramps
            -- Stretch-onset gain (Soft switch S9 + Soft W K5), anchored at
            -- stretch_y. Multiplied into eps so the stretch amount ramps
            -- in smoothly for the first few rows beneath the Stretch line.
            -- Ramp is CAPPED at 256 rows.
            --   S9 off: gain snaps 0 → 1023 at stretch_y.
            --   S9 on : slope = 64 − (15/16) × soft_knob(9:4)  → 64..4
            --             knob 0   → slope 64 → full gain in ~16 rows
            --             knob 512 → slope 34 → ~30-row ramp
            --             knob max → slope 4  → ~256-row ramp (cap)
            --             gain = sat(dy_stretch × slope, 1023).
            if soft_on = '0' then
                row_stretch_gain_mul_r <= (others => '0');
                if row_dy_stretch_r = 0 then
                    row_stretch_gain_r <= (others => '0');
                else
                    row_stretch_gain_r <= to_unsigned(1023, 10);
                end if;
            else
                soft_width := soft_knob(9 downto 4);
                soft_slope := to_unsigned(64, 7) - resize(soft_width, 7)
                            + resize(shift_right(soft_width, 4), 7);
                src_mul_v := row_dy_stretch_r * soft_slope;
                row_stretch_gain_mul_r <= src_mul_v;
                if row_dy_stretch_r = 0 then
                    row_stretch_gain_r <= (others => '0');
                elsif row_stretch_gain_mul_r(18 downto 10) /= "000000000" then
                    row_stretch_gain_r <= to_unsigned(1023, 10);
                else
                    row_stretch_gain_r <= row_stretch_gain_mul_r(9 downto 0);
                end if;
            end if;

            -- Stretch factor is driven by row_dy_stretch_r (rows past the
            -- Stretch knob's start line) scaled by spread_knob. dy is used
            -- directly every row so there's no bucketing or stepping from
            -- a coarse pre-LUT.
            --
            --   eps_mul = row_dy_stretch_r * spread_knob      (22 bits)
            --   eps_raw = saturate(eps_mul >> 8, 1023)        (10 bits)
            --
            -- Curve switch (S8) then selects the final per-row factor:
            --   Straight (S8=0): 5-bit index into FACTOR_LUT plus 5-bit
            --                    linear interpolation between neighbours.
            --                    Gives factor = eps/(1+eps/256), producing
            --                    straight diagonal columns.
            --   Curved   (S8=1): factor = saturate(eps_raw >> 1, 255).
            --                    Equivalent to the original eps = dy*spread
            --                    >> 9 sat 255 — hyperbolic fan-out.
            -- Spread effective scale per Curve mode:
            --   Straight: 1.5 × spread_knob (knob + knob/2)
            --   Curved:   0.5 × spread_knob (knob/2)
            -- Pre-registered so the shift/add doesn't chain in front of
            -- the multiply.
            if curved_on = '1' then
                row_spread_eff_r <= resize(spread_knob(9 downto 1), 11);
            else
                row_spread_eff_r <= resize(spread_knob, 11)
                                  + resize(spread_knob(9 downto 1), 11);
            end if;
            row_eps_mul <= row_dy_stretch_r * row_spread_eff_r;

            -- Stage A: saturate to 10-bit linear eps. Shift is 10 so that at
            -- HD drape heights (dy up to ~720) the full spread_knob range
            -- stays below saturation — avoids the plateau where factor
            -- clamps, every row applies the same transform, and lines drop
            -- perfectly vertical.
            -- eps_raw = eps_mul >> 9, saturated at 2047 (11-bit). Shift was
            -- 10; bumping to 9 doubles the eps for any given dy × spread,
            -- pushing typical settings deeper into the LUT for more stretch
            -- without changing the LUT's curve shape.
            if row_eps_mul(22 downto 20) /= "000" then
                row_eps_raw_r <= to_unsigned(2047, 11);
            else
                row_eps_raw_r <= row_eps_mul(19 downto 9);
            end if;

            -- Smoothing stage: multiply the raw eps by row_stretch_gain_r so
            -- the onset of stretch near stretch_y is gradual (gain ramps
            -- from 0 to 1023 over a few rows when Soft is on). With Soft
            -- off, gain is 0 above stretch and 1023 below, so eps_smoothed
            -- equals eps_raw for all drape rows → no behavioural change.
            row_eps_smooth_mul_r <= row_eps_raw_r * row_stretch_gain_r;
            row_eps_smoothed_r   <= row_eps_smooth_mul_r(20 downto 10);

            -- Stage B: register both FACTOR_LUT neighbours, the fraction,
            -- and the curved-mode equivalent. Indexed by the SMOOTHED eps
            -- top 6 bits (64-entry LUT now).
            lut_idx := to_integer(row_eps_smoothed_r(10 downto 5));
            row_factor_lo_r <= FACTOR_LUT(lut_idx);
            if lut_idx = 63 then
                row_factor_hi_r <= FACTOR_LUT(63);
            else
                row_factor_hi_r <= FACTOR_LUT(lut_idx + 1);
            end if;
            row_eps_frac_r  <= row_eps_smoothed_r(4 downto 0);
            row_curved_on_r <= curved_on;
            -- Curved mode style picked by Curve+ (S10):
            --   Hard (S10=0): factor = sat(eps_smoothed × 2, 255). Reaches
            --                 255 at eps_smoothed = 128 (fast saturation).
            --   Soft (S10=1): factor = sat(eps_smoothed >> 1, 255). Reaches
            --                 255 at eps_smoothed ≈ 510 (4× more knob/dy
            --                 needed). Smoother, finer-grained knob feel.
            if curve_soft = '1' then
                if row_eps_smoothed_r(10 downto 9) /= "00" then
                    row_eps_curved_r <= to_unsigned(255, 8);
                else
                    row_eps_curved_r <= row_eps_smoothed_r(8 downto 1);
                end if;
            else
                if row_eps_smoothed_r(10 downto 7) /= "0000" then
                    row_eps_curved_r <= to_unsigned(255, 8);
                else
                    row_eps_curved_r <= row_eps_smoothed_r(6 downto 0) & '0';
                end if;
            end if;

            -- Stage C: interpolate between neighbours, then mux against
            -- curved-mode result.
            factor_delta := row_factor_hi_r - row_factor_lo_r;
            interp_mul := factor_delta * row_eps_frac_r;
            interp_v := row_factor_lo_r + interp_mul(9 downto 5);
            if row_curved_on_r = '1' then
                row_eps_r <= row_eps_curved_r;
            else
                row_eps_r <= interp_v;
            end if;

            -------------------------------------------------------------------
            -- STAGE 1 : register inputs + forward pixel_x
            -------------------------------------------------------------------
            s1_y       <= unsigned(data_in.y);
            s1_u       <= unsigned(data_in.u);
            s1_v       <= unsigned(data_in.v);
            s1_avid    <= data_in.avid;
            s1_pixel_x <= pixel_x;

            -------------------------------------------------------------------
            -- STAGE 2 : compute dx; carry control + data
            -------------------------------------------------------------------
            s2_y <= s1_y;
            s2_u <= s1_u;
            s2_v <= s1_v;
            s2_pixel_x <= s1_pixel_x;
            s2_avid    <= s1_avid;
            s2_is_above <= row_is_above_r;
            s2_fade_a   <= row_fade_a_r;
            s2_eps      <= row_eps_r;
            -- dx = pixel_x - centre. centre is 11 bits; dx in 13-bit signed.
            s2_dx <= signed(resize(s1_pixel_x, 13))
                   - signed(resize(centre_r, 13));

            -------------------------------------------------------------------
            -- STAGE 3 : delta_mul = s2_dx * s2_eps. Forward everything else.
            -------------------------------------------------------------------
            s3_delta_mul <= s2_dx * signed(resize(s2_eps, 9));
            s3_pixel_x   <= s2_pixel_x;
            s3_avid      <= s2_avid;
            s3_y <= s2_y;
            s3_u <= s2_u;
            s3_v <= s2_v;
            s3_is_above <= s2_is_above;
            s3_fade_a   <= s2_fade_a;

            -------------------------------------------------------------------
            -- STAGE 3b : rd_addr = pixel_x - (delta_mul >> 8), saturated.
            -- delta range ≈ ±|dx|; rd_addr stays in [0, max_x-1] for valid x
            -- so saturation is only a safety net against eps > 256 edges.
            -------------------------------------------------------------------
            delta_v   := resize(shift_right(s3_delta_mul, 8), 14);
            rd_addr_v := signed(resize(s3_pixel_x, 14)) - delta_v
                       + resize(row_twist_offset_r, 14);
            if rd_addr_v < 0 then
                s3b_rd_addr <= (others => '0');
            elsif rd_addr_v(13 downto C_ADDR_BITS) /= 0 then
                s3b_rd_addr <= (others => '1');
            else
                s3b_rd_addr <= unsigned(rd_addr_v(C_ADDR_BITS-1 downto 0));
            end if;

            s3b_wr_en   <= s3_is_above and s3_avid;
            s3b_wr_addr <= s3_pixel_x(C_ADDR_BITS-1 downto 0);
            s3b_y <= s3_y;
            s3b_u <= s3_u;
            s3b_v <= s3_v;
            s3b_is_above <= s3_is_above;
            s3b_fade_a   <= s3_fade_a;

            -------------------------------------------------------------------
            -- STAGE 4 : inferred BRAM write + synchronous read (from s3b)
            -------------------------------------------------------------------
            if s3b_wr_en = '1' then
                s_ram_y(to_integer(s3b_wr_addr)) <= std_logic_vector(s3b_y);
                s_ram_u(to_integer(s3b_wr_addr)) <= std_logic_vector(s3b_u);
                s_ram_v(to_integer(s3b_wr_addr)) <= std_logic_vector(s3b_v);
            end if;
            s4_lb_y_slv <= s_ram_y(to_integer(s3b_rd_addr));
            s4_lb_u_slv <= s_ram_u(to_integer(s3b_rd_addr));
            s4_lb_v_slv <= s_ram_v(to_integer(s3b_rd_addr));

            -- 4-tap blur delay line
            s4_lb_y_d1 <= unsigned(s4_lb_y_slv);
            s4_lb_y_d2 <= s4_lb_y_d1;
            s4_lb_y_d3 <= s4_lb_y_d2;
            s4_lb_y_d4 <= s4_lb_y_d3;
            s4_lb_y_d5 <= s4_lb_y_d4;
            s4_lb_y_d6 <= s4_lb_y_d5;
            s4_lb_y_d7 <= s4_lb_y_d6;
            s4_lb_u_d1 <= unsigned(s4_lb_u_slv);
            s4_lb_u_d2 <= s4_lb_u_d1;
            s4_lb_u_d3 <= s4_lb_u_d2;
            s4_lb_u_d4 <= s4_lb_u_d3;
            s4_lb_u_d5 <= s4_lb_u_d4;
            s4_lb_u_d6 <= s4_lb_u_d5;
            s4_lb_u_d7 <= s4_lb_u_d6;
            s4_lb_v_d1 <= unsigned(s4_lb_v_slv);
            s4_lb_v_d2 <= s4_lb_v_d1;
            s4_lb_v_d3 <= s4_lb_v_d2;
            s4_lb_v_d4 <= s4_lb_v_d3;
            s4_lb_v_d5 <= s4_lb_v_d4;
            s4_lb_v_d6 <= s4_lb_v_d5;
            s4_lb_v_d7 <= s4_lb_v_d6;

            s4_y <= s3b_y;
            s4_u <= s3b_u;
            s4_v <= s3b_v;
            s4_is_above <= s3b_is_above;
            s4_fade_a   <= s3b_fade_a;
            s4_blur_on  <= blur_on;

            -------------------------------------------------------------------
            -- STAGE 4b : pre-register the 8-tap blur as two 4-way half-sums.
            -- Stage 5 then just adds the halves and divides by 8, instead of
            -- chaining BRAM read → 8-way carry → mux in one cycle. When
            -- Blur is off, both halves are just the bare BRAM sample × 4 so
            -- their sum × 8 / 8 reconstructs the original sample.
            -------------------------------------------------------------------
            s4b_y        <= s4_y;
            s4b_u        <= s4_u;
            s4b_v        <= s4_v;
            s4b_is_above <= s4_is_above;
            s4b_fade_a   <= s4_fade_a;

            if s4_blur_on = '1' then
                s4b_half_top_y <= resize(unsigned(s4_lb_y_slv), 12)
                                + resize(s4_lb_y_d1, 12)
                                + resize(s4_lb_y_d2, 12)
                                + resize(s4_lb_y_d3, 12);
                s4b_half_bot_y <= resize(s4_lb_y_d4, 12)
                                + resize(s4_lb_y_d5, 12)
                                + resize(s4_lb_y_d6, 12)
                                + resize(s4_lb_y_d7, 12);
                s4b_half_top_u <= resize(unsigned(s4_lb_u_slv), 12)
                                + resize(s4_lb_u_d1, 12)
                                + resize(s4_lb_u_d2, 12)
                                + resize(s4_lb_u_d3, 12);
                s4b_half_bot_u <= resize(s4_lb_u_d4, 12)
                                + resize(s4_lb_u_d5, 12)
                                + resize(s4_lb_u_d6, 12)
                                + resize(s4_lb_u_d7, 12);
                s4b_half_top_v <= resize(unsigned(s4_lb_v_slv), 12)
                                + resize(s4_lb_v_d1, 12)
                                + resize(s4_lb_v_d2, 12)
                                + resize(s4_lb_v_d3, 12);
                s4b_half_bot_v <= resize(s4_lb_v_d4, 12)
                                + resize(s4_lb_v_d5, 12)
                                + resize(s4_lb_v_d6, 12)
                                + resize(s4_lb_v_d7, 12);
            else
                -- No blur: both halves carry the bare BRAM read × 4 so
                -- their sum × 1/8 reconstructs the original sample.
                s4b_half_top_y <= shift_left(resize(unsigned(s4_lb_y_slv), 12), 2);
                s4b_half_bot_y <= shift_left(resize(unsigned(s4_lb_y_slv), 12), 2);
                s4b_half_top_u <= shift_left(resize(unsigned(s4_lb_u_slv), 12), 2);
                s4b_half_bot_u <= shift_left(resize(unsigned(s4_lb_u_slv), 12), 2);
                s4b_half_top_v <= shift_left(resize(unsigned(s4_lb_v_slv), 12), 2);
                s4b_half_bot_v <= shift_left(resize(unsigned(s4_lb_v_slv), 12), 2);
            end if;

            -------------------------------------------------------------------
            -- STAGE 5 : combine half-sums + source mux
            --   blur_sum = top + bot   (12 + 12 = 13 bits)
            --   blurred sample = blur_sum / 8 = blur_sum(12 downto 3)
            -------------------------------------------------------------------
            blur_sum_y := resize(s4b_half_top_y, 13) + resize(s4b_half_bot_y, 13);
            blur_sum_u := resize(s4b_half_top_u, 13) + resize(s4b_half_bot_u, 13);
            blur_sum_v := resize(s4b_half_top_v, 13) + resize(s4b_half_bot_v, 13);

            if s4b_is_above = '1' then
                s5_y <= s4b_y;
                s5_u <= s4b_u;
                s5_v <= s4b_v;
            else
                s5_y <= blur_sum_y(12 downto 3);
                s5_u <= blur_sum_u(12 downto 3);
                s5_v <= blur_sum_v(12 downto 3);
            end if;
            s5_fade_a <= s4b_fade_a;

            -------------------------------------------------------------------
            -- STAGE 6 : fade blend (Y → 0, U/V → 512)
            --   alpha_inv = NOT(alpha) → 0..1023
            --   Y_out  = (Y_src  * alpha_inv) >> 10
            --   UV_out = (UV_src * alpha_inv + 512 * alpha) >> 10
            -------------------------------------------------------------------
            alpha_inv := not s5_fade_a;
            y_mul := s5_y * alpha_inv;
            u_mul := s5_u * alpha_inv;
            v_mul := s5_v * alpha_inv;
            u_add := u_mul + shift_left(resize(s5_fade_a, 20), 9);
            v_add := v_mul + shift_left(resize(s5_fade_a, 20), 9);
            s6_y <= y_mul(19 downto 10);
            s6_u <= u_add(19 downto 10);
            s6_v <= v_add(19 downto 10);

            -------------------------------------------------------------------
            -- STAGE 7p : dry/wet MIX — pre-stage that registers the delta.
            -- s6 at cycle T represents data_in from 8 cycles ago (s1..s4,
            -- s4b, s5, s6 = 7 register stages plus 1 BRAM read latency =
            -- 8). Aligned dry sample is at s_*_bypass(7) = LATENCY-4.
            -------------------------------------------------------------------
            mix_dry_y := unsigned(s_y_bypass(LATENCY-4));
            mix_dry_u := unsigned(s_u_bypass(LATENCY-4));
            mix_dry_v := unsigned(s_v_bypass(LATENCY-4));

            s7p_delta_y <= signed(resize(s6_y, 11)) - signed(resize(mix_dry_y, 11));
            s7p_delta_u <= signed(resize(s6_u, 11)) - signed(resize(mix_dry_u, 11));
            s7p_delta_v <= signed(resize(s6_v, 11)) - signed(resize(mix_dry_v, 11));
            s7p_dry_y <= mix_dry_y;
            s7p_dry_u <= mix_dry_u;
            s7p_dry_v <= mix_dry_v;

            -------------------------------------------------------------------
            -- STAGE 7a : delta × mix — one signed 11×10 multiply per channel
            -- (register the full 22-bit product so the >> 10 + add + clamp
            -- below lives in its own short stage).
            -------------------------------------------------------------------
            s7a_mul_y <= s7p_delta_y * signed('0' & std_logic_vector(mix_knob));
            s7a_mul_u <= s7p_delta_u * signed('0' & std_logic_vector(mix_knob));
            s7a_mul_v <= s7p_delta_v * signed('0' & std_logic_vector(mix_knob));
            s7a_dry_y <= s7p_dry_y;
            s7a_dry_u <= s7p_dry_u;
            s7a_dry_v <= s7p_dry_v;

            -------------------------------------------------------------------
            -- STAGE 7 : MIX add + clamp — final output
            --   out = dry + (delta * mix) >> 10
            -- At mix = 0 → out = dry (passthrough). At mix ≈ 1023 → out ≈ wet.
            -------------------------------------------------------------------
            mix_sum_ys := signed(resize(s7a_dry_y, 12))
                        + resize(s7a_mul_y(21 downto 10), 12);
            mix_sum_us := signed(resize(s7a_dry_u, 12))
                        + resize(s7a_mul_u(21 downto 10), 12);
            mix_sum_vs := signed(resize(s7a_dry_v, 12))
                        + resize(s7a_mul_v(21 downto 10), 12);

            if mix_sum_ys < 0 then
                s7_y <= (others => '0');
            elsif mix_sum_ys > to_signed(1023, 12) then
                s7_y <= to_unsigned(1023, 10);
            else
                s7_y <= unsigned(mix_sum_ys(9 downto 0));
            end if;
            if mix_sum_us < 0 then
                s7_u <= (others => '0');
            elsif mix_sum_us > to_signed(1023, 12) then
                s7_u <= to_unsigned(1023, 10);
            else
                s7_u <= unsigned(mix_sum_us(9 downto 0));
            end if;
            if mix_sum_vs < 0 then
                s7_v <= (others => '0');
            elsif mix_sum_vs > to_signed(1023, 12) then
                s7_v <= to_unsigned(1023, 10);
            else
                s7_v <= unsigned(mix_sum_vs(9 downto 0));
            end if;

            -------------------------------------------------------------------
            -- Sync / bypass shift registers
            -------------------------------------------------------------------
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            s_y_bypass(0) <= data_in.y;
            s_u_bypass(0) <= data_in.u;
            s_v_bypass(0) <= data_in.v;
            for i in 1 to LATENCY-1 loop
                s_hsync_sr(i) <= s_hsync_sr(i-1);
                s_vsync_sr(i) <= s_vsync_sr(i-1);
                s_field_sr(i) <= s_field_sr(i-1);
                s_avid_sr(i)  <= s_avid_sr(i-1);
                s_y_bypass(i) <= s_y_bypass(i-1);
                s_u_bypass(i) <= s_u_bypass(i-1);
                s_v_bypass(i) <= s_v_bypass(i-1);
            end loop;
        end if;
    end process main_p;

    ----------------------------------------------------------------------------
    -- Output assignment
    ----------------------------------------------------------------------------
    data_out.hsync_n <= s_hsync_sr(LATENCY-1);
    data_out.vsync_n <= s_vsync_sr(LATENCY-1);
    data_out.field_n <= s_field_sr(LATENCY-1);
    data_out.avid    <= s_avid_sr(LATENCY-1);

    data_out.y <= s_y_bypass(LATENCY-1) when s_bypass_on = '1' else std_logic_vector(s7_y);
    data_out.u <= s_u_bypass(LATENCY-1) when s_bypass_on = '1' else std_logic_vector(s7_u);
    data_out.v <= s_v_bypass(LATENCY-1) when s_bypass_on = '1' else std_logic_vector(s7_v);

end architecture drape;
