-- Starfield: shader-inspired glowing parallax starfield generator
--
-- 8 depth layers, cell-based star placement (16x16 cells), improved
-- octagonal distance glow, per-star random colours from four palettes.
-- Luma Mode maps incoming video brightness to star density.
--
-- Optimised for iCE40 HX4K: linear distance (no squaring),
-- shift-only falloff, no per-layer multipliers.
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Layers    (top 3 bits: 000=1 .. 111=8)
--   registers_in(1) = Density   (0-1023, manual density / luma gain)
--   registers_in(2) = Speed     (0-1023, max ~38 px/frame)
--   registers_in(3) = Palette   (top 2 bits: 00=Mixed 01=Cool 10=Warm 11=Rainbow)
--   registers_in(4) = Glow      (full 10-bit smooth glow)
--   registers_in(5) = Depth     (top 3 bits: layer parallax spread)
--   registers_in(6) = Switches  (bit0=Dir bit1=BW bit2=Luma bit3=BG bit4=LumaGate)
--   registers_in(7) = Brightness (0-1023, linear potentiometer)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture starfield of program_top is

    constant LATENCY   : natural := 13;
    constant NUM_LYRS  : natural := 8;

    type t_pipe    is array (natural range <>) of t_video_stream_yuv444_30b;
    type t_u24_arr is array(0 to NUM_LYRS-1) of unsigned(23 downto 0);
    type t_u16_arr is array(0 to NUM_LYRS-1) of unsigned(15 downto 0);
    type t_u4_arr  is array(0 to NUM_LYRS-1) of unsigned(3 downto 0);
    type t_u5_arr  is array(0 to NUM_LYRS-1) of unsigned(4 downto 0);
    type t_u2_arr  is array(0 to NUM_LYRS-1) of unsigned(1 downto 0);
    type t_u10_arr is array(0 to NUM_LYRS-1) of unsigned(9 downto 0);

    constant SEEDS : t_u16_arr := (
        x"A3B1", x"7C2D", x"D917", x"4E6F",
        x"B5C3", x"62E8", x"F147", x"8D9A"
    );

    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Position
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    -- 24-bit scroll accumulators
    signal scroll : t_u24_arr := (others => (others => '0'));

    -- Scroll pre-computation pipeline: always-updating registered helpers so
    -- the once-per-frame scroll() adder sees a single 24-bit add from reg->reg.
    signal speed16_r       : unsigned(15 downto 0) := (others => '0');
    signal decel_1step_r   : unsigned(15 downto 0) := (others => '0');
    signal speed_nonzero_r : std_logic := '0';
    signal accum_decel_r   : t_u16_arr := (others => (others => '0'));
    signal layer_speed_r   : t_u16_arr := (others => (others => '0'));
    signal scroll_delta_r  : t_u24_arr := (others => (others => '0'));
    signal dir_r           : std_logic := '0';

    -- Stage 2
    signal s2_sx : t_u24_arr;
    signal s2_y  : unsigned(11 downto 0);

    -- Registered density-threshold for stage 4a1. Computed one cycle early
    -- from pipe(1).y so that, after this one register stage, the threshold
    -- aligns with the same source pixel pipe(2).y refers to in stage 4a1.
    -- Pre-registering breaks the combinational subtract+scale+clamp chain
    -- that the gated-luma mode otherwise adds to the threshold path.
    signal threshold_r : unsigned(12 downto 0) := (others => '0');

    -- Stage 3a: hash pre-multiply (XOR part)
    signal s3a_h     : t_u16_arr;
    signal s3a_sub_x : t_u4_arr;
    signal s3a_sub_y : unsigned(3 downto 0);

    -- Stage 3: hash finalised (after multiply + xor-shift)
    signal s3_hash  : t_u16_arr;
    signal s3_sub_x : t_u4_arr;
    signal s3_sub_y : unsigned(3 downto 0);

    -- Stage 4a1: per-layer pre-distance properties
    signal s4a1_eff_rad    : t_u5_arr;
    signal s4a1_offset_x   : t_u4_arr;
    signal s4a1_offset_y   : t_u4_arr;
    signal s4a1_sub_x      : t_u4_arr;
    signal s4a1_sub_y      : unsigned(3 downto 0);
    signal s4a1_color_idx  : t_u2_arr;
    -- layer_active AND hash_v < threshold, per layer
    signal s4a1_active_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');

    -- Stage 4ax: absolute 4-bit differences dx, dy — breaks the 4a dist/hit
    -- critical path by registering between abs-diff and distance+compare.
    signal s4ax_dx         : t_u4_arr;
    signal s4ax_dy         : t_u4_arr;
    signal s4ax_eff_rad    : t_u5_arr;
    signal s4ax_active_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4ax_color_idx  : t_u2_arr;

    -- Stage 4a: per-layer distance + final hit test results
    signal s4a_hit       : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4a_dist      : t_u5_arr;
    signal s4a_color_idx : t_u2_arr;

    -- Stage 4b1: per-layer brightness after falloff
    signal s4b1_layer_y   : t_u10_arr;
    signal s4b1_color_idx : t_u2_arr;

    -- Stage 4b: per-layer brightness after depth fade
    signal s4b_layer_y   : t_u10_arr;
    signal s4b_color_idx : t_u2_arr;

    -- Stage 4c1: pairwise sum (8->4) and pairwise max (8->4)
    signal s4c1_sum_01, s4c1_sum_23, s4c1_sum_45, s4c1_sum_67 : unsigned(10 downto 0) := (others => '0');
    signal s4c1_max01_y, s4c1_max23_y, s4c1_max45_y, s4c1_max67_y     : unsigned(9 downto 0) := (others => '0');
    signal s4c1_max01_ci, s4c1_max23_ci, s4c1_max45_ci, s4c1_max67_ci : unsigned(1 downto 0) := (others => '0');

    -- Stage 4c2: quad sum (4->2) and quad max (4->2)
    signal s4c2_sum_0123, s4c2_sum_4567       : unsigned(11 downto 0) := (others => '0');
    signal s4c2_max0123_y, s4c2_max4567_y     : unsigned(9 downto 0) := (others => '0');
    signal s4c2_max0123_ci, s4c2_max4567_ci   : unsigned(1 downto 0) := (others => '0');

    -- Stage 4c: composited star pixel
    signal s4_y : unsigned(9 downto 0);
    signal s4_u : unsigned(9 downto 0);
    signal s4_v : unsigned(9 downto 0);

    -- Stage 5
    signal s5_y : unsigned(9 downto 0);
    signal s5_u : unsigned(9 downto 0);
    signal s5_v : unsigned(9 downto 0);

    ---------------------------------------------------------------------------
    -- Multiplicative hash: good avalanche via Fibonacci constant 0x9E37.
    ---------------------------------------------------------------------------
    function hash_cell(
        cx   : unsigned(15 downto 0);
        cy   : unsigned(15 downto 0);
        seed : unsigned(15 downto 0)
    ) return unsigned is
        variable h    : unsigned(15 downto 0);
        variable cy_r : unsigned(15 downto 0);
        variable prod : unsigned(31 downto 0);
    begin
        cy_r := cy(8 downto 0) & cy(15 downto 9);
        h    := cx xor cy_r xor seed;
        prod := h * to_unsigned(16#9E37#, 16);
        h    := prod(15 downto 0);
        h    := h xor shift_right(h, 5);
        h    := h xor (h(10 downto 0) & "00000");
        return h;
    end function;

begin

    process(clk)
        -- Scroll
        variable speed16       : unsigned(15 downto 0);
        variable depth_3       : unsigned(2 downto 0);

        -- Controls
        variable num_layers_3  : unsigned(2 downto 0);
        variable density       : unsigned(9 downto 0);
        variable palette_sel   : unsigned(1 downto 0);
        variable glow_full     : unsigned(9 downto 0);
        variable bright        : unsigned(9 downto 0);
        variable v_diff_y      : unsigned(9 downto 0);
        variable v_scaled_thr  : unsigned(13 downto 0);

        -- Stage 3
        variable cell_x_v      : unsigned(19 downto 0);
        variable cx16          : unsigned(15 downto 0);
        variable cell_y_v      : unsigned(7 downto 0);
        variable cell_y_16     : unsigned(15 downto 0);
        variable cy_rot        : unsigned(15 downto 0);
        variable hash_prod     : unsigned(31 downto 0);
        variable hash_tmp      : unsigned(15 downto 0);

        -- Stage 4 per-layer
        variable hash_v        : unsigned(15 downto 0);
        variable offset_x      : unsigned(3 downto 0);
        variable offset_y      : unsigned(3 downto 0);
        variable dx            : unsigned(3 downto 0);
        variable dy            : unsigned(3 downto 0);
        variable mn            : unsigned(3 downto 0);
        variable mn_x3         : unsigned(5 downto 0);
        variable dist          : unsigned(4 downto 0);
        variable prop_bits     : unsigned(3 downto 0);
        variable base_rad      : unsigned(2 downto 0);
        variable eff_rad_x256  : unsigned(9 downto 0);
        variable v_luma_boost  : unsigned(1 downto 0);
        variable v_luma_cap    : unsigned(1 downto 0);
        variable v_rad_sum     : unsigned(4 downto 0);
        variable eff_rad       : unsigned(4 downto 0);
        variable color_idx     : unsigned(1 downto 0);
        variable layer_y       : unsigned(9 downto 0);
        variable layer_active  : boolean;

        -- Accumulation
        variable total_y       : unsigned(12 downto 0);
        variable best_u        : unsigned(9 downto 0);
        variable best_v        : unsigned(9 downto 0);
        variable max_layer_y   : unsigned(9 downto 0);
        variable s4_y_var      : unsigned(9 downto 0);

        -- Stage 4c3 final tree-reduction intermediates
        variable best_ci                                  : unsigned(1 downto 0);

        -- Stage 5
        variable bg_y          : unsigned(9 downto 0);
        variable bg_u          : unsigned(9 downto 0);
        variable bg_v          : unsigned(9 downto 0);
        variable out_y_wide    : unsigned(12 downto 0);
        variable out_u         : unsigned(9 downto 0);
        variable out_v         : unsigned(9 downto 0);

    begin
        if rising_edge(clk) then

            ----------------------------------------------------------------
            -- Delay pipeline
            ----------------------------------------------------------------
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 1 : Position tracking & scroll update
            ----------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                pixel_x <= (others => '0');
                pixel_y <= pixel_y + 1;
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                pixel_y     <= (others => '0');
                frame_count <= frame_count + 1;

                -- Scroll update: use the always-pipelined per-layer delta so
                -- the register->register path here is a single 24-bit add.
                if speed_nonzero_r = '1' then
                    for layer in 0 to NUM_LYRS-1 loop
                        if dir_r = '0' then
                            scroll(layer) <= scroll(layer) + scroll_delta_r(layer);
                        else
                            scroll(layer) <= scroll(layer) - scroll_delta_r(layer);
                        end if;
                    end loop;
                end if;
            end if;

            ----------------------------------------------------------------
            -- Scroll speed pre-computation (always updating every clock).
            -- Three registered stages break the formerly-combinational
            -- "decel cascade -> subtract -> scroll DFF" chain so that the
            -- once-per-frame scroll update needs only a single 24-bit add.
            ----------------------------------------------------------------

            -- Stage A: speed16, decel_1step, speed_nonzero, dir
            speed16 := resize(unsigned(registers_in(2)(9 downto 5)), 16);
            speed16 := speed16 + shift_right(speed16, 2);
            speed16_r <= speed16;
            dir_r     <= registers_in(6)(0);
            if speed16 > to_unsigned(0, 16) then
                speed_nonzero_r <= '1';
            else
                speed_nonzero_r <= '0';
            end if;

            depth_3 := unsigned(registers_in(5)(9 downto 7));
            case to_integer(depth_3) is
                when 0 =>
                    decel_1step_r <= to_unsigned(1, 16);
                when 1 =>
                    decel_1step_r <= resize(shift_right(speed16, 5), 16)
                                   + to_unsigned(1, 16);
                when 2 =>
                    decel_1step_r <= resize(shift_right(speed16, 4), 16)
                                   + to_unsigned(1, 16);
                when 3 =>
                    decel_1step_r <= resize(shift_right(speed16, 3), 16)
                                   + to_unsigned(1, 16);
                when 4 =>
                    decel_1step_r <= resize(shift_right(speed16, 3), 16)
                                   + resize(shift_right(speed16, 4), 16)
                                   + to_unsigned(1, 16);
                when 5 =>
                    decel_1step_r <= resize(shift_right(speed16, 2), 16)
                                   + to_unsigned(1, 16);
                when 6 =>
                    decel_1step_r <= resize(shift_right(speed16, 2), 16)
                                   + resize(shift_right(speed16, 3), 16)
                                   + to_unsigned(1, 16);
                when others =>
                    decel_1step_r <= resize(shift_right(speed16, 1), 16)
                                   + to_unsigned(1, 16);
            end case;

            -- Stage B: per-layer accum_decel as a registered shift-add cascade.
            -- accum_decel_r(i) converges to (7 - i) * decel_1step_r after a
            -- handful of clocks; scroll only samples it at vsync so latency is
            -- invisible. Each DFF sees exactly one 16-bit adder.
            accum_decel_r(7) <= (others => '0');
            for i in 0 to 6 loop
                accum_decel_r(i) <= accum_decel_r(i + 1) + decel_1step_r;
            end loop;

            -- Stage C: per-layer layer_speed = speed16_r - accum_decel_r, with
            -- minimum of 1 when the subtraction would underflow.
            for i in 0 to NUM_LYRS-1 loop
                if speed16_r > accum_decel_r(i) then
                    layer_speed_r(i) <= speed16_r - accum_decel_r(i);
                else
                    layer_speed_r(i) <= to_unsigned(1, 16);
                end if;
            end loop;

            -- Stage D: widen to 24 bits for the scroll adder fan-in. Keeps the
            -- vsync-gated scroll() path down to one 24-bit add + mux.
            for i in 0 to NUM_LYRS-1 loop
                scroll_delta_r(i) <= resize(layer_speed_r(i), 24);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 2 : Scrolled X per layer (24-bit)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                s2_sx(i) <= resize(pixel_x, 24) + scroll(i);
            end loop;
            s2_y <= pixel_y;

            ----------------------------------------------------------------
            -- STAGE 3a : hash pre-multiply (XOR of cell_x, rotated cell_y, seed)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                cell_x_v := s2_sx(i)(23 downto 4);
                cx16     := cell_x_v(15 downto 0)
                          xor resize(cell_x_v(19 downto 16), 16);
                cell_y_v := s2_y(11 downto 4);
                cell_y_16 := resize(cell_y_v, 16);
                cy_rot := cell_y_16(8 downto 0) & cell_y_16(15 downto 9);
                s3a_h(i)     <= cx16 xor cy_rot xor SEEDS(i);
                s3a_sub_x(i) <= s2_sx(i)(3 downto 0);
            end loop;
            s3a_sub_y <= s2_y(3 downto 0);

            ----------------------------------------------------------------
            -- STAGE 3 : hash multiply + finalise (consumes registered 3a)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                hash_prod := s3a_h(i) * to_unsigned(16#9E37#, 16);
                hash_tmp  := hash_prod(15 downto 0);
                hash_tmp  := hash_tmp xor shift_right(hash_tmp, 5);
                hash_tmp  := hash_tmp xor (hash_tmp(10 downto 0) & "00000");
                s3_hash(i)  <= hash_tmp;
                s3_sub_x(i) <= s3a_sub_x(i);
            end loop;
            s3_sub_y <= s3a_sub_y;

            ----------------------------------------------------------------
            -- STAGE 4 : Per-layer star detection, glow, colour
            ----------------------------------------------------------------
            num_layers_3 := unsigned(registers_in(0)(9 downto 7));
            density      := unsigned(registers_in(1));
            palette_sel  := unsigned(registers_in(3)(9 downto 8));
            -- Full 10-bit glow knob. Maps linearly to eff_rad growth across
            -- the whole 0..100 % range — see eff_rad_x256 below.
            glow_full    := unsigned(registers_in(4));
            bright       := unsigned(registers_in(7));

            -- Density threshold: manual knob or input video luma.
            -- Luma Mode (switch 2): bright video areas = more stars.
            -- Luma Gate (switch 4, only meaningful when luma mode is on):
            --   off → threshold = luma << 3       (dim areas keep a light dusting)
            --   on  → threshold = clamp((luma - 256) * 11, 0, 8191)
            --         Bottom 25 % luma is truly starless; the ×11 slope
            --         restores peak density at luma = 1023 so high-luma
            --         regions are not dimmed compared to the ungated mode.
            -- Reads pipe(1).y; threshold_r registers the result so the
            -- subtract + scale + clamp does not lengthen stage 4a1's path.
            -- After the register, threshold_r aligns with the same source
            -- pixel that pipe(2).y refers to in stage 4a1.
            if registers_in(6)(2) = '1' then
                if registers_in(6)(4) = '1' then
                    if unsigned(pipe(1).y) > to_unsigned(256, 10) then
                        v_diff_y := unsigned(pipe(1).y) - to_unsigned(256, 10);
                        v_scaled_thr := resize(v_diff_y, 14)
                                      + resize(v_diff_y & "0", 14)
                                      + resize(v_diff_y & "000", 14);
                        if v_scaled_thr > to_unsigned(8191, 14) then
                            threshold_r <= to_unsigned(8191, 13);
                        else
                            threshold_r <= v_scaled_thr(12 downto 0);
                        end if;
                    else
                        threshold_r <= (others => '0');
                    end if;
                else
                    threshold_r <= unsigned(pipe(1).y) & "000";
                end if;
            else
                threshold_r <= density & "000";
            end if;

            ----------------------------------------------------------------
            -- STAGE 4a1 : pre-distance properties (eff_rad, offsets, hash
            -- threshold check, colour index). Splits the big 16-way prop_bits
            -- case + eff_rad add out of the distance compute.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                layer_active := i >= (7 - to_integer(num_layers_3));

                hash_v := s3_hash(i);
                offset_x := resize(hash_v(2 downto 0), 4) + to_unsigned(4, 4);
                offset_y := resize(hash_v(5 downto 3), 4) + to_unsigned(4, 4);
                prop_bits := hash_v(3 downto 0) xor hash_v(7 downto 4);

                case to_integer(prop_bits) is
                    when 0 | 1 | 2 | 3 | 4 | 5 |
                         6 | 7 | 8 | 9 | 10 | 11 =>
                        base_rad := to_unsigned(0, 3);
                    when 12 =>
                        base_rad := to_unsigned(1, 3);
                    when others =>
                        base_rad := to_unsigned(2, 3);
                end case;

                -- Glow is represented in 2.8 fixed-point across the full
                -- 10-bit knob range. For each base size we compute the
                -- target eff_rad as a linear interpolation from base_rad
                -- up to the cap of 2:
                --   base=0: eff_rad_x256 = glow_full / 2    → 0   .. 511
                --   base=1: eff_rad_x256 = 256 + glow_full/4 → 256 .. 511
                --   base>=2: eff_rad_x256 = 512 (saturated at 2.0)
                -- The top 2 bits are the integer part (0..2) and the low
                -- 8 bits are the fractional dither threshold. We compare
                -- that fraction against unused hash bits so each star
                -- independently rounds up or down, producing ~1024 visible
                -- glow levels across the knob and making ALL small stars
                -- (not just a fraction) smoothly converge toward medium at
                -- 100 % glow.
                case to_integer(base_rad) is
                    when 0 =>
                        eff_rad_x256 := '0' & glow_full(9 downto 1);
                    when 1 =>
                        eff_rad_x256 := to_unsigned(256, 10)
                                      + ("00" & glow_full(9 downto 2));
                    when others =>
                        eff_rad_x256 := to_unsigned(512, 10);
                end case;

                if hash_v(15 downto 8) < eff_rad_x256(7 downto 0) then
                    eff_rad := resize(eff_rad_x256(9 downto 8), 5)
                             + to_unsigned(1, 5);
                else
                    eff_rad := resize(eff_rad_x256(9 downto 8), 5);
                end if;

                if eff_rad > to_unsigned(2, 5) then
                    eff_rad := to_unsigned(2, 5);
                end if;

                -- Luma-mod glow bump: when luma mode is on, stars in bright
                -- regions of the incoming video get extra radius so the
                -- starfield tracks the source's highlights.
                --
                -- The bump magnitude is now scaled by the Glow knob via
                -- v_luma_cap (0..2 from glow_full top bits, saturated):
                --   Glow  0..25 % → cap 0  (no luma-driven size bump)
                --   Glow 25..50 % → cap 1
                --   Glow 50..100% → cap 2
                -- v_luma_boost = min(luma_top, v_luma_cap), so high luma can
                -- only enlarge stars as much as Glow permits.
                if registers_in(6)(2) = '1' then
                    if glow_full(9 downto 8) > to_unsigned(2, 2) then
                        v_luma_cap := to_unsigned(2, 2);
                    else
                        v_luma_cap := glow_full(9 downto 8);
                    end if;
                    v_luma_boost := unsigned(pipe(2).y(9 downto 8));
                    if v_luma_boost > v_luma_cap then
                        v_luma_boost := v_luma_cap;
                    end if;
                    v_rad_sum := resize(eff_rad, 5) + resize(v_luma_boost, 5);
                    if v_rad_sum > to_unsigned(7, 5) then
                        eff_rad := to_unsigned(7, 5);
                    else
                        eff_rad := v_rad_sum;
                    end if;
                end if;

                color_idx := hash_v(1 downto 0) xor hash_v(5 downto 4);

                s4a1_eff_rad(i)   <= eff_rad;
                s4a1_offset_x(i)  <= offset_x;
                s4a1_offset_y(i)  <= offset_y;
                s4a1_sub_x(i)     <= s3_sub_x(i);
                s4a1_color_idx(i) <= color_idx;
                if layer_active and hash_v < resize(threshold_r, 16) then
                    s4a1_active_hit(i) <= '1';
                else
                    s4a1_active_hit(i) <= '0';
                end if;
            end loop;
            s4a1_sub_y <= s3_sub_y;

            ----------------------------------------------------------------
            -- STAGE 4ax : absolute 4-bit differences |sub - offset| per axis.
            -- Registered to split the long 4a chain.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                if s4a1_sub_x(i) >= s4a1_offset_x(i) then
                    s4ax_dx(i) <= s4a1_sub_x(i) - s4a1_offset_x(i);
                else
                    s4ax_dx(i) <= s4a1_offset_x(i) - s4a1_sub_x(i);
                end if;
                if s4a1_sub_y >= s4a1_offset_y(i) then
                    s4ax_dy(i) <= s4a1_sub_y - s4a1_offset_y(i);
                else
                    s4ax_dy(i) <= s4a1_offset_y(i) - s4a1_sub_y;
                end if;
                s4ax_eff_rad(i)    <= s4a1_eff_rad(i);
                s4ax_active_hit(i) <= s4a1_active_hit(i);
                s4ax_color_idx(i)  <= s4a1_color_idx(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4a : octagonal distance + final hit test (consumes
            -- registered 4ax outputs).
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                if s4ax_dx(i) >= s4ax_dy(i) then
                    mn := s4ax_dy(i);
                    dist := resize(s4ax_dx(i), 5);
                else
                    mn := s4ax_dx(i);
                    dist := resize(s4ax_dy(i), 5);
                end if;
                mn_x3 := shift_left(resize(mn, 6), 1) + resize(mn, 6);
                dist := dist + resize(shift_right(mn_x3, 3), 5);

                if s4ax_active_hit(i) = '1' and dist <= s4ax_eff_rad(i) then
                    s4a_hit(i) <= '1';
                else
                    s4a_hit(i) <= '0';
                end if;
                s4a_dist(i)      <= dist;
                s4a_color_idx(i) <= s4ax_color_idx(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4b1 : falloff (consumes registered 4a)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                if s4a_hit(i) = '1' then
                    case to_integer(s4a_dist(i)) is
                        when 0      => layer_y := bright;
                        when 1      => layer_y := shift_right(bright, 1)
                                                + shift_right(bright, 2);
                        when 2      => layer_y := shift_right(bright, 1)
                                                + shift_right(bright, 3);
                        when 3      => layer_y := shift_right(bright, 1);
                        when 4      => layer_y := shift_right(bright, 2)
                                                + shift_right(bright, 3);
                        when others => layer_y := shift_right(bright, 2);
                    end case;
                else
                    layer_y := (others => '0');
                end if;

                s4b1_layer_y(i)   <= layer_y;
                s4b1_color_idx(i) <= s4a_color_idx(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4b : depth fade (consumes registered 4b1)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                layer_y := s4b1_layer_y(i);
                case i is
                    when 0 => layer_y := shift_right(layer_y, 2);
                    when 1 => layer_y := shift_right(layer_y, 2)
                                       + shift_right(layer_y, 3);
                    when 2 => layer_y := shift_right(layer_y, 1);
                    when 3 => layer_y := shift_right(layer_y, 1)
                                       + shift_right(layer_y, 3);
                    when 4 => layer_y := layer_y - shift_right(layer_y, 2);
                    when 5 => layer_y := layer_y - shift_right(layer_y, 3);
                    when 6 => layer_y := layer_y - shift_right(layer_y, 4);
                    when 7 => null;
                    when others => null;
                end case;
                s4b_layer_y(i)   <= layer_y;
                s4b_color_idx(i) <= s4b1_color_idx(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4c1 : pairwise (8->4) sum and max — registered
            ----------------------------------------------------------------
            s4c1_sum_01 <= resize(s4b_layer_y(0), 11) + resize(s4b_layer_y(1), 11);
            s4c1_sum_23 <= resize(s4b_layer_y(2), 11) + resize(s4b_layer_y(3), 11);
            s4c1_sum_45 <= resize(s4b_layer_y(4), 11) + resize(s4b_layer_y(5), 11);
            s4c1_sum_67 <= resize(s4b_layer_y(6), 11) + resize(s4b_layer_y(7), 11);

            if s4b_layer_y(0) >= s4b_layer_y(1) then
                s4c1_max01_y <= s4b_layer_y(0); s4c1_max01_ci <= s4b_color_idx(0);
            else
                s4c1_max01_y <= s4b_layer_y(1); s4c1_max01_ci <= s4b_color_idx(1);
            end if;
            if s4b_layer_y(2) >= s4b_layer_y(3) then
                s4c1_max23_y <= s4b_layer_y(2); s4c1_max23_ci <= s4b_color_idx(2);
            else
                s4c1_max23_y <= s4b_layer_y(3); s4c1_max23_ci <= s4b_color_idx(3);
            end if;
            if s4b_layer_y(4) >= s4b_layer_y(5) then
                s4c1_max45_y <= s4b_layer_y(4); s4c1_max45_ci <= s4b_color_idx(4);
            else
                s4c1_max45_y <= s4b_layer_y(5); s4c1_max45_ci <= s4b_color_idx(5);
            end if;
            if s4b_layer_y(6) >= s4b_layer_y(7) then
                s4c1_max67_y <= s4b_layer_y(6); s4c1_max67_ci <= s4b_color_idx(6);
            else
                s4c1_max67_y <= s4b_layer_y(7); s4c1_max67_ci <= s4b_color_idx(7);
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c2 : quad (4->2) sum and max — registered
            ----------------------------------------------------------------
            s4c2_sum_0123 <= resize(s4c1_sum_01, 12) + resize(s4c1_sum_23, 12);
            s4c2_sum_4567 <= resize(s4c1_sum_45, 12) + resize(s4c1_sum_67, 12);

            if s4c1_max01_y >= s4c1_max23_y then
                s4c2_max0123_y <= s4c1_max01_y; s4c2_max0123_ci <= s4c1_max01_ci;
            else
                s4c2_max0123_y <= s4c1_max23_y; s4c2_max0123_ci <= s4c1_max23_ci;
            end if;
            if s4c1_max45_y >= s4c1_max67_y then
                s4c2_max4567_y <= s4c1_max45_y; s4c2_max4567_ci <= s4c1_max45_ci;
            else
                s4c2_max4567_y <= s4c1_max67_y; s4c2_max4567_ci <= s4c1_max67_ci;
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c3 : final (2->1) sum + clamp + final max + palette
            ----------------------------------------------------------------
            total_y := resize(s4c2_sum_0123, 13) + resize(s4c2_sum_4567, 13);

            if s4c2_max0123_y >= s4c2_max4567_y then
                max_layer_y := s4c2_max0123_y; best_ci := s4c2_max0123_ci;
            else
                max_layer_y := s4c2_max4567_y; best_ci := s4c2_max4567_ci;
            end if;

            -- Single palette lookup on the winning colour index
            if registers_in(6)(1) = '1' then
                best_u := to_unsigned(512, 10);
                best_v := to_unsigned(512, 10);
            else
                case palette_sel is
                    when "00" =>
                        case best_ci is
                            when "00"   => best_u := to_unsigned(512, 10);
                                           best_v := to_unsigned(512, 10);
                            when "01"   => best_u := to_unsigned(300, 10);
                                           best_v := to_unsigned(760, 10);
                            when "10"   => best_u := to_unsigned(600, 10);
                                           best_v := to_unsigned(380, 10);
                            when others => best_u := to_unsigned(580, 10);
                                           best_v := to_unsigned(620, 10);
                        end case;
                    when "01" =>
                        case best_ci is
                            when "00"   => best_u := to_unsigned(512, 10);
                                           best_v := to_unsigned(512, 10);
                            when "01"   => best_u := to_unsigned(280, 10);
                                           best_v := to_unsigned(800, 10);
                            when "10"   => best_u := to_unsigned(620, 10);
                                           best_v := to_unsigned(720, 10);
                            when others => best_u := to_unsigned(480, 10);
                                           best_v := to_unsigned(560, 10);
                        end case;
                    when "10" =>
                        case best_ci is
                            when "00"   => best_u := to_unsigned(512, 10);
                                           best_v := to_unsigned(512, 10);
                            when "01"   => best_u := to_unsigned(720, 10);
                                           best_v := to_unsigned(340, 10);
                            when "10"   => best_u := to_unsigned(640, 10);
                                           best_v := to_unsigned(400, 10);
                            when others => best_u := to_unsigned(560, 10);
                                           best_v := to_unsigned(460, 10);
                        end case;
                    when others =>
                        case best_ci is
                            when "00"   => best_u := to_unsigned(512, 10);
                                           best_v := to_unsigned(512, 10);
                            when "01"   => best_u := to_unsigned(800, 10);
                                           best_v := to_unsigned(350, 10);
                            when "10"   => best_u := to_unsigned(180, 10);
                                           best_v := to_unsigned(660, 10);
                            when others => best_u := to_unsigned(780, 10);
                                           best_v := to_unsigned(760, 10);
                        end case;
                end case;
            end if;

            if total_y > to_unsigned(1023, 13) then
                s4_y_var := (others => '1');
            else
                s4_y_var := total_y(9 downto 0);
            end if;
            s4_y <= s4_y_var;
            s4_u <= best_u;
            s4_v <= best_v;

            ----------------------------------------------------------------
            -- STAGE 5 : Background + star composite
            ----------------------------------------------------------------
            if registers_in(6)(3) = '0' then
                bg_y := (others => '0');
                bg_u := to_unsigned(512, 10);
                bg_v := to_unsigned(512, 10);
            else
                bg_y := to_unsigned(32, 10);
                bg_u := to_unsigned(390, 10);
                bg_v := to_unsigned(650, 10);
            end if;

            out_y_wide := resize(bg_y, 13) + resize(s4_y, 13);

            if s4_y > to_unsigned(0, 10) then
                out_u := s4_u;
                out_v := s4_v;
            else
                out_u := bg_u;
                out_v := bg_v;
            end if;

            if out_y_wide > to_unsigned(1023, 13) then
                s5_y <= (others => '1');
            else
                s5_y <= out_y_wide(9 downto 0);
            end if;
            s5_u <= out_u;
            s5_v <= out_v;

            ----------------------------------------------------------------
            -- OUTPUT
            ----------------------------------------------------------------
            data_out    <= pipe(LATENCY - 1);
            data_out.y  <= std_logic_vector(s5_y);
            data_out.u  <= std_logic_vector(s5_u);
            data_out.v  <= std_logic_vector(s5_v);

        end if;
    end process;

end architecture starfield;
