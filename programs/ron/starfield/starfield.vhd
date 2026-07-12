-- Starfield: shader-inspired glowing parallax starfield generator
--
-- 8 depth layers, cell-based star placement (16x16 cells). Each star is
-- rendered from a per-layer glow-kernel ROM in EBR (see gen_kernels.py):
-- address {glow_level(2b), shape(2b), |dy|(3b), |dx|(3b)} -> 8-bit weight.
-- Four star shapes (gaussian glow / soft square / diffraction spike /
-- point+halo), smooth baked falloff, per-layer depth fade baked into the
-- ROM contents. Per-star random colours from four palettes.
-- Luma Mode maps incoming video brightness to star density.
--
-- Optimised for iCE40 HX4K: kernels live in otherwise-unused EBR; the
-- only wide multiplier besides the 8 hash multipliers is one shared
-- 10x10 brightness multiply after the composite trees.
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Layers    (top 3 bits: 000=1 .. 111=8)
--   registers_in(1) = Density   (0-1023, manual density / luma gain)
--   registers_in(2) = Speed     (0-1023, max ~38 px/frame)
--   registers_in(3) = Palette+Mood (bits 8:7 = Mixed/Cool/Warm/Rainbow,
--                     bit 9 = Lively: deeper twinkle, faster churn/shimmer)
--   registers_in(4) = Glow      (top 2 bits kernel level, low 8 dither)
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
use work.starfield_rom_pkg.all;
use work.all;

architecture starfield of program_top is

    constant LATENCY   : natural := 16;
    constant NUM_LYRS  : natural := 8;

    type t_pipe    is array (natural range <>) of t_video_stream_yuv444_30b;
    type t_u24_arr is array(0 to NUM_LYRS-1) of unsigned(23 downto 0);
    type t_u16_arr is array(0 to NUM_LYRS-1) of unsigned(15 downto 0);
    type t_u8_arr  is array(0 to NUM_LYRS-1) of unsigned(7 downto 0);
    type t_u5_arr  is array(0 to NUM_LYRS-1) of unsigned(4 downto 0);
    type t_u4_arr  is array(0 to NUM_LYRS-1) of unsigned(3 downto 0);
    type t_u3_arr  is array(0 to NUM_LYRS-1) of unsigned(2 downto 0);
    type t_u2_arr  is array(0 to NUM_LYRS-1) of unsigned(1 downto 0);

    constant SEEDS : t_u16_arr := (
        x"A3B1", x"7C2D", x"D917", x"4E6F",
        x"B5C3", x"62E8", x"F147", x"8D9A"
    );

    -- Centre-weighted sub-cell position map: 16 hash values -> 12 distinct
    -- positions in [2..13]. Cell-edge positions are deliberately rarer so
    -- large glow kernels seldom sit against a cell border (single-cell
    -- sampling clips the kernel at the border; the kernels' radial window
    -- plus this bias makes the clip invisible).
    type t_offmap is array (0 to 15) of unsigned(3 downto 0);
    constant C_OFFMAP : t_offmap := (
        x"2", x"3", x"4", x"5", x"6", x"6", x"7", x"7",
        x"8", x"8", x"9", x"9", x"A", x"B", x"C", x"D"
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

    -- Registered knob-derived helpers for stage 4a1 (same trick as
    -- threshold_r): the SPI register file sits in one corner of the die,
    -- and routing its outputs straight into the 8-layer property cone was
    -- the critical path. Knob values are quasi-static, so one cycle of
    -- staleness is invisible.
    signal glow_base_r : unsigned(1 downto 0) := (others => '0');
    signal glow_frac_r : unsigned(7 downto 0) := (others => '0');
    signal luma_mode_r : std_logic := '0';
    signal first_act_r : unsigned(2 downto 0) := (others => '0');
    signal pal_sel_r   : unsigned(1 downto 0) := (others => '0');
    signal bw_r        : std_logic := '0';
    signal bg_on_r     : std_logic := '0';

    -- Twinkle / fade animation state. Phases advance once per frame at
    -- vsync; rates are mood-selected (P4 top bit: Calm / Lively) and
    -- pre-registered like the other knob helpers.
    signal tw_phase    : unsigned(7 downto 0)  := (others => '0');
    signal fade_cnt    : unsigned(15 downto 0) := (others => '0');
    signal shim_cnt    : unsigned(15 downto 0) := (others => '0');

    -- Deep-space background: nebula texture scroll (4.12 fixed point,
    -- advances by speed16_r per frame -> drifts at 1/16 of the front
    -- layer's speed) and the short background sampling pipeline. The
    -- background path is only 4 stages, so it reaches the composite ~7
    -- pixels "early" — a fixed horizontal shift of a wrapping texture,
    -- which is invisible and costs no delay-matching registers.
    signal neb_scroll : unsigned(15 downto 0) := (others => '0');
    signal bg1_addr   : unsigned(11 downto 0) := (others => '0');
    signal bg2_neb    : std_logic_vector(7 downto 0) := (others => '0');
    signal bg2_grad   : unsigned(6 downto 0) := (others => '0');
    signal bg3_w      : unsigned(7 downto 0) := (others => '0');
    signal mood_r      : std_logic := '0';
    signal tw_rate_r   : unsigned(2 downto 0) := (others => '0');
    signal fade_rate_r : unsigned(7 downto 0) := (others => '0');
    signal shim_rate_r : unsigned(7 downto 0) := (others => '0');

    -- Stage 3a: hash pre-multiply (XOR part)
    signal s3a_h     : t_u16_arr;
    signal s3a_sub_x : t_u4_arr;
    signal s3a_sub_y : unsigned(3 downto 0);

    -- Stage 3: hash finalised (after multiply + xor-shift)
    signal s3_hash  : t_u16_arr;
    signal s3_sub_x : t_u4_arr;
    signal s3_sub_y : unsigned(3 downto 0);

    -- Stage 3p: per-layer star properties straight off the hash (offsets,
    -- shape, colour, dither verdict, magnitude penalty, threshold check).
    -- Split from the glow-level combine: the 8-bit dither compare plus the
    -- add/clamp chain in one stage was the HD critical path.
    signal s3p_offset_x   : t_u4_arr;
    signal s3p_offset_y   : t_u4_arr;
    signal s3p_sub_x      : t_u4_arr;
    signal s3p_sub_y      : unsigned(3 downto 0);
    signal s3p_shape      : t_u2_arr;
    signal s3p_color_idx  : t_u2_arr;
    signal s3p_mag        : t_u2_arr;
    signal s3p_dith       : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    -- per-star twinkle / fade phase positions and permanent-star flag
    signal s3p_pt         : t_u8_arr;
    signal s3p_pf         : t_u8_arr;
    signal s3p_perm       : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    -- layer_active AND hash_v < threshold, per layer
    signal s3p_active_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');

    -- Stage 4a1: glow-level combine (base + dither - magnitude + luma
    -- bump, clamped) and spike-offset clamp
    signal s4a1_offset_x   : t_u4_arr;
    signal s4a1_offset_y   : t_u4_arr;
    signal s4a1_sub_x      : t_u4_arr;
    signal s4a1_sub_y      : unsigned(3 downto 0);
    signal s4a1_shape      : t_u2_arr;
    signal s4a1_lvl        : t_u2_arr;
    signal s4a1_color_idx  : t_u2_arr;
    signal s4a1_active_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    -- combined twinkle*fade brightness envelope, 0..16 (16 = unity)
    signal s4a1_env        : t_u5_arr;

    -- Stage 4ax: absolute differences |sub - offset| per axis (0..13),
    -- folded to 3 bits with an in-range kill for |d| > 7. These registers
    -- feed the kernel-ROM addresses directly.
    signal s4ax_adx        : t_u3_arr;
    signal s4ax_ady        : t_u3_arr;
    signal s4ax_shape      : t_u2_arr;
    signal s4ax_lvl        : t_u2_arr;
    signal s4ax_color_idx  : t_u2_arr;
    signal s4ax_hit        : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4ax_env        : t_u5_arr;

    -- Stage 4k0: kernel-ROM synchronous read (the EBR output register)
    signal s4k0_w   : t_u8_arr := (others => (others => '0'));
    signal s4k0_hit : std_logic_vector(0 to NUM_LYRS-1) := (others => '0');
    signal s4k0_ci  : t_u2_arr;
    signal s4k0_env : t_u5_arr;

    -- Stage 4k1: registered ROM read data, masked by the hit flag
    signal s4k1_w   : t_u8_arr := (others => (others => '0'));
    signal s4k1_ci  : t_u2_arr;
    signal s4k1_env : t_u5_arr;

    -- Stage 4e: per-layer envelope multiply (8x5, weight*env >> 4)
    signal s4e_w  : t_u8_arr := (others => (others => '0'));
    signal s4e_ci : t_u2_arr;

    -- Stage 4c1: pairwise sum (8->4) and pairwise max (8->4) of weights
    signal s4c1_sum_01, s4c1_sum_23, s4c1_sum_45, s4c1_sum_67 : unsigned(8 downto 0) := (others => '0');
    signal s4c1_max01_w, s4c1_max23_w, s4c1_max45_w, s4c1_max67_w     : unsigned(7 downto 0) := (others => '0');
    signal s4c1_max01_ci, s4c1_max23_ci, s4c1_max45_ci, s4c1_max67_ci : unsigned(1 downto 0) := (others => '0');

    -- Stage 4c2: quad sum (4->2) and quad max (4->2)
    signal s4c2_sum_0123, s4c2_sum_4567       : unsigned(9 downto 0) := (others => '0');
    signal s4c2_max0123_w, s4c2_max4567_w     : unsigned(7 downto 0) := (others => '0');
    signal s4c2_max0123_ci, s4c2_max4567_ci   : unsigned(1 downto 0) := (others => '0');

    -- Stage 4c3: total weight (clamped) + winning star colour index +
    -- star-present flag (star chroma vs background chroma at stage 5)
    signal s4c3_w  : unsigned(9 downto 0) := (others => '0');
    signal s4c3_ci : unsigned(1 downto 0) := (others => '0');
    signal s4c3_sf : std_logic := '0';
    signal s4m1_sf : std_logic := '0';
    signal s4m2_sf : std_logic := '0';

    -- Stage 4m1/4m2: shared brightness multiplier (inputs registered at
    -- 4m1, product registered at 4m2 — product valid two cycles after
    -- the operands, per the registered-multiplier rule). The shimmer
    -- palette EBR read runs in parallel: address registered into the
    -- read at 4m1, data registered at 4m2.
    signal s4m1_w      : unsigned(9 downto 0) := (others => '0');
    signal s4m1_bright : unsigned(9 downto 0) := (others => '0');
    signal s4m1_pal    : std_logic_vector(19 downto 0) := (others => '0');
    signal s4m2_y      : unsigned(11 downto 0) := (others => '0');
    signal s4m2_u      : unsigned(9 downto 0) := (others => '0');
    signal s4m2_v      : unsigned(9 downto 0) := (others => '0');

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
        variable dith8         : unsigned(7 downto 0);
        variable lvl_i         : integer range -3 to 8;
        variable boost_i       : integer range 0 to 3;
        variable cap_i         : integer range 0 to 3;
        variable d4            : unsigned(3 downto 0);
        variable inr_x         : boolean;
        variable inr_y         : boolean;
        variable addr_v        : unsigned(9 downto 0);
        variable layer_active  : boolean;

        -- Twinkle / fade envelope
        variable pt_v          : unsigned(7 downto 0);
        variable pf_v          : unsigned(7 downto 0);
        variable tri7          : unsigned(6 downto 0);
        variable depth4        : unsigned(3 downto 0);
        variable env_t         : unsigned(4 downto 0);
        variable env_f         : unsigned(4 downto 0);
        variable env_v         : unsigned(4 downto 0);
        variable prod13        : unsigned(12 downto 0);

        -- Background path
        variable sum12         : unsigned(11 downto 0);

        -- Accumulation
        variable total_w       : unsigned(10 downto 0);

        -- Stage 4c3 final tree-reduction intermediates
        variable best_ci       : unsigned(1 downto 0);
        variable max_w         : unsigned(7 downto 0);

        -- Stage 4m1 shimmer-palette address / 4m2 brightness product
        variable paddr_v       : unsigned(7 downto 0);
        variable prod_v        : unsigned(19 downto 0);

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

                -- Twinkle / fade / shimmer phase advance (rates pre-registered)
                tw_phase <= tw_phase + resize(tw_rate_r, 8);
                fade_cnt <= fade_cnt + resize(fade_rate_r, 16);
                shim_cnt <= shim_cnt + resize(shim_rate_r, 16);

                -- Nebula drift: 1/16 of the front-layer speed, same direction
                if speed_nonzero_r = '1' then
                    if dir_r = '0' then
                        neb_scroll <= neb_scroll + speed16_r;
                    else
                        neb_scroll <= neb_scroll - speed16_r;
                    end if;
                end if;

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
            -- P4: bits 8:7 = palette, bit 9 = mood (Calm / Lively)
            palette_sel  := unsigned(registers_in(3)(8 downto 7));
            -- Full 10-bit glow knob: top 2 bits pick the kernel-ROM glow
            -- level, low 8 bits dither stars between adjacent levels.
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
            -- Knob-derived helpers, registered every clock (see declaration)
            glow_base_r <= glow_full(9 downto 8);
            glow_frac_r <= glow_full(7 downto 0);
            luma_mode_r <= registers_in(6)(2);
            first_act_r <= to_unsigned(7, 3) - num_layers_3;
            pal_sel_r   <= palette_sel;
            bw_r        <= registers_in(6)(1);
            bg_on_r     <= registers_in(6)(3);

            -- Mood (P4 top bit): Calm = shallow slow twinkle, ~17 s fade
            -- churn; Lively = deep faster twinkle, ~6 s churn.
            mood_r <= registers_in(3)(9);
            if registers_in(3)(9) = '1' then
                tw_rate_r   <= to_unsigned(5, 3);
                fade_rate_r <= to_unsigned(192, 8);
                shim_rate_r <= to_unsigned(128, 8);  -- hue orbit ~8.5 s
            else
                tw_rate_r   <= to_unsigned(2, 3);
                fade_rate_r <= to_unsigned(64, 8);
                shim_rate_r <= to_unsigned(36, 8);   -- hue orbit ~30 s
            end if;

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
            -- STAGE 3p : per-layer star properties straight off the hash.
            --
            -- Star properties draw from the LOW hash bits: the presence
            -- test (hash < threshold) forces the high bits of surviving
            -- stars toward zero, so bits 15:12 carry almost no entropy for
            -- stars that actually exist. Bits 11:0 are used directly and
            -- xor-mixed where more fields are needed.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                layer_active := i >= to_integer(first_act_r);

                hash_v := s3_hash(i);
                s3p_offset_x(i) <= C_OFFMAP(to_integer(hash_v(3 downto 0)));
                s3p_offset_y(i) <= C_OFFMAP(to_integer(hash_v(7 downto 4)));
                s3p_shape(i)    <= hash_v(9 downto 8);
                -- Colour from bits 5:4 (uniform even at low densities,
                -- where the threshold test biases bits 11:10 toward zero),
                -- decorrelated from shape by the xor.
                s3p_color_idx(i) <= hash_v(5 downto 4) xor hash_v(9 downto 8);
                -- Per-star glow dither verdict (the 8-bit compare that
                -- forced this stage split) and magnitude-penalty selector.
                dith8 := hash_v(11 downto 4) xor hash_v(7 downto 0);
                if dith8 < glow_frac_r then
                    s3p_dith(i) <= '1';
                else
                    s3p_dith(i) <= '0';
                end if;
                s3p_mag(i)  <= hash_v(7 downto 6) xor hash_v(1 downto 0);
                s3p_sub_x(i) <= s3_sub_x(i);
                if layer_active and hash_v < resize(threshold_r, 16) then
                    s3p_active_hit(i) <= '1';
                else
                    s3p_active_hit(i) <= '0';
                end if;

                -- Twinkle phase position: per-star offset (rotated dither
                -- byte — uniform even at low density) + global phase, with
                -- a per-star rate select (1x / 2x, the 2x phase being the
                -- global phase shifted left = wrapping twice as fast).
                if (hash_v(10) xor hash_v(0)) = '1' then
                    pt_v := (dith8(3 downto 0) & dith8(7 downto 4))
                          + (tw_phase(6 downto 0) & '0');
                else
                    pt_v := (dith8(3 downto 0) & dith8(7 downto 4))
                          + tw_phase;
                end if;
                s3p_pt(i) <= pt_v;

                -- Fade lifecycle position: per-star offset + slow counter.
                s3p_pf(i) <= (hash_v(3 downto 0) & hash_v(7 downto 4))
                           + fade_cnt(15 downto 8);

                -- ~25% of stars are permanent (never fade out), from
                -- xor-folded bits that stay uniform under the threshold
                -- test's high-bit bias.
                if (hash_v(6) xor hash_v(9)) = '1'
                   and (hash_v(2) xor hash_v(11)) = '1' then
                    s3p_perm(i) <= '1';
                else
                    s3p_perm(i) <= '0';
                end if;
            end loop;
            s3p_sub_y <= s3_sub_y;

            ----------------------------------------------------------------
            -- STAGE 4a1 : glow-level combine + spike offset clamp.
            --
            -- Glow level 0..3 selects the kernel-ROM page:
            --   knob base (glow_full top 2 bits)
            --   + per-star dither verdict from 3p, so the population
            --     blends smoothly between adjacent levels as the knob turns
            --   - per-star magnitude penalty (50% none, 25% -1, 25% -2)
            --     for size variety within the field
            --   + luma-mode bump: bright input video enlarges stars,
            --     capped by the Glow knob's own level.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                offset_x := s3p_offset_x(i);
                offset_y := s3p_offset_y(i);
                -- Diffraction-spike arms reach the kernel edge (|d| = 7);
                -- keep spike stars clear of cell borders so the arms are
                -- never clipped by single-cell sampling.
                if s3p_shape(i) = "10" then
                    if offset_x < to_unsigned(5, 4) then
                        offset_x := to_unsigned(5, 4);
                    elsif offset_x > to_unsigned(10, 4) then
                        offset_x := to_unsigned(10, 4);
                    end if;
                    if offset_y < to_unsigned(5, 4) then
                        offset_y := to_unsigned(5, 4);
                    elsif offset_y > to_unsigned(10, 4) then
                        offset_y := to_unsigned(10, 4);
                    end if;
                end if;

                lvl_i := to_integer(glow_base_r);
                if s3p_dith(i) = '1' then
                    lvl_i := lvl_i + 1;
                end if;
                if s3p_mag(i) = "10" then
                    lvl_i := lvl_i - 1;
                elsif s3p_mag(i) = "11" then
                    lvl_i := lvl_i - 2;
                end if;
                if luma_mode_r = '1' then
                    cap_i   := to_integer(glow_base_r);
                    boost_i := to_integer(unsigned(pipe(3).y(9 downto 8)));
                    if boost_i > cap_i then
                        boost_i := cap_i;
                    end if;
                    lvl_i := lvl_i + boost_i;
                end if;
                if lvl_i < 0 then
                    lvl_i := 0;
                elsif lvl_i > 3 then
                    lvl_i := 3;
                end if;

                -- Twinkle envelope: triangle fold of the phase position.
                -- Calm keeps stars between 9/16 and full; Lively dips to
                -- 4/16. Never zero — twinkle dims, fade extinguishes.
                pt_v := s3p_pt(i);
                if pt_v(7) = '1' then
                    tri7 := not pt_v(6 downto 0);
                else
                    tri7 := pt_v(6 downto 0);
                end if;
                if mood_r = '1' then
                    depth4 := tri7(6 downto 3);
                    if depth4 > to_unsigned(12, 4) then
                        depth4 := to_unsigned(12, 4);
                    end if;
                else
                    depth4 := '0' & tri7(6 downto 4);
                end if;
                env_t := to_unsigned(16, 5) - resize(depth4, 5);

                -- Fade envelope: quarter-cycle fade-in, half-cycle hold,
                -- quarter-cycle fade-out; permanent stars hold forever.
                pf_v := s3p_pf(i);
                case pf_v(7 downto 6) is
                    when "00"   => env_f := '0' & pf_v(5 downto 2);
                    when "11"   => env_f := '0' & not pf_v(5 downto 2);
                    when others => env_f := to_unsigned(16, 5);
                end case;
                if s3p_perm(i) = '1' then
                    env_f := to_unsigned(16, 5);
                end if;

                if env_t < env_f then
                    env_v := env_t;
                else
                    env_v := env_f;
                end if;

                s4a1_offset_x(i)  <= offset_x;
                s4a1_offset_y(i)  <= offset_y;
                s4a1_sub_x(i)     <= s3p_sub_x(i);
                s4a1_shape(i)     <= s3p_shape(i);
                s4a1_lvl(i)       <= to_unsigned(lvl_i, 2);
                s4a1_color_idx(i) <= s3p_color_idx(i);
                s4a1_active_hit(i) <= s3p_active_hit(i);
                s4a1_env(i)       <= env_v;
            end loop;
            s4a1_sub_y <= s3p_sub_y;

            ----------------------------------------------------------------
            -- STAGE 4ax : absolute differences |sub - offset| per axis with
            -- in-range kill (|d| > 7 means the pixel is outside the kernel).
            -- These registers feed the kernel-ROM addresses directly.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                if s4a1_sub_x(i) >= s4a1_offset_x(i) then
                    d4 := s4a1_sub_x(i) - s4a1_offset_x(i);
                else
                    d4 := s4a1_offset_x(i) - s4a1_sub_x(i);
                end if;
                inr_x := d4 <= to_unsigned(7, 4);
                s4ax_adx(i) <= d4(2 downto 0);

                if s4a1_sub_y >= s4a1_offset_y(i) then
                    d4 := s4a1_sub_y - s4a1_offset_y(i);
                else
                    d4 := s4a1_offset_y(i) - s4a1_sub_y;
                end if;
                inr_y := d4 <= to_unsigned(7, 4);
                s4ax_ady(i) <= d4(2 downto 0);

                if s4a1_active_hit(i) = '1' and inr_x and inr_y then
                    s4ax_hit(i) <= '1';
                else
                    s4ax_hit(i) <= '0';
                end if;
                s4ax_shape(i)     <= s4a1_shape(i);
                s4ax_lvl(i)       <= s4a1_lvl(i);
                s4ax_color_idx(i) <= s4a1_color_idx(i);
                s4ax_env(i)       <= s4a1_env(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4k0 : kernel-ROM synchronous read (one ROM per layer,
            -- addressed straight from the 4ax registers; this read register
            -- is the EBR's internal output register). The eight reads are
            -- unrolled onto eight separately-named constants: indexing a
            -- single array-of-arrays constant makes GHDL synthesis merge
            -- them into one 8192-deep ROM and crash (internal error in
            -- netlists-memories.adb).
            ----------------------------------------------------------------
            addr_v := s4ax_lvl(0) & s4ax_shape(0) & s4ax_ady(0) & s4ax_adx(0);
            s4k0_w(0) <= unsigned(C_KERNEL_L0(to_integer(addr_v)));
            addr_v := s4ax_lvl(1) & s4ax_shape(1) & s4ax_ady(1) & s4ax_adx(1);
            s4k0_w(1) <= unsigned(C_KERNEL_L1(to_integer(addr_v)));
            addr_v := s4ax_lvl(2) & s4ax_shape(2) & s4ax_ady(2) & s4ax_adx(2);
            s4k0_w(2) <= unsigned(C_KERNEL_L2(to_integer(addr_v)));
            addr_v := s4ax_lvl(3) & s4ax_shape(3) & s4ax_ady(3) & s4ax_adx(3);
            s4k0_w(3) <= unsigned(C_KERNEL_L3(to_integer(addr_v)));
            addr_v := s4ax_lvl(4) & s4ax_shape(4) & s4ax_ady(4) & s4ax_adx(4);
            s4k0_w(4) <= unsigned(C_KERNEL_L4(to_integer(addr_v)));
            addr_v := s4ax_lvl(5) & s4ax_shape(5) & s4ax_ady(5) & s4ax_adx(5);
            s4k0_w(5) <= unsigned(C_KERNEL_L5(to_integer(addr_v)));
            addr_v := s4ax_lvl(6) & s4ax_shape(6) & s4ax_ady(6) & s4ax_adx(6);
            s4k0_w(6) <= unsigned(C_KERNEL_L6(to_integer(addr_v)));
            addr_v := s4ax_lvl(7) & s4ax_shape(7) & s4ax_ady(7) & s4ax_adx(7);
            s4k0_w(7) <= unsigned(C_KERNEL_L7(to_integer(addr_v)));
            for i in 0 to NUM_LYRS-1 loop
                s4k0_hit(i) <= s4ax_hit(i);
                s4k0_ci(i)  <= s4ax_color_idx(i);
                s4k0_env(i) <= s4ax_env(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4k1 : register the ROM read data (EBR reads need a
            -- register before fabric use) and mask by the hit flag.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                if s4k0_hit(i) = '1' then
                    s4k1_w(i) <= s4k0_w(i);
                else
                    s4k1_w(i) <= (others => '0');
                end if;
                s4k1_ci(i)  <= s4k0_ci(i);
                s4k1_env(i) <= s4k0_env(i);
            end loop;

            ----------------------------------------------------------------
            -- STAGE 4e : per-layer twinkle/fade envelope multiply.
            -- weight(8b) * env(5b) >> 4, env 16 = unity. Small 8x5
            -- multiplier, product registered here and consumed by the
            -- 4c1 trees next cycle.
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                prod13 := s4k1_w(i) * s4k1_env(i);
                s4e_w(i)  <= prod13(11 downto 4);
                s4e_ci(i) <= s4k1_ci(i);
            end loop;

            ----------------------------------------------------------------
            -- BACKGROUND path (3 stages, weight domain): scrolling nebula
            -- texture + vertical gradient, gated by S10. Composites into
            -- the star weight sum at 4c3, so it scales with P12 and sits
            -- under the stars automatically.
            ----------------------------------------------------------------
            -- BG1: texture address (4x stretch, 256-px wrapping tile)
            sum12 := resize(pixel_x, 12) + neb_scroll(15 downto 4);
            bg1_addr <= pixel_y(7 downto 2) & sum12(7 downto 2);

            -- BG2: nebula ROM read (EBR output register) + subtle gradient
            -- (24 -> 0 over the top ~768 lines)
            bg2_neb <= C_NEBULA(to_integer(bg1_addr));
            if pixel_y(11 downto 5) < to_unsigned(24, 7) then
                bg2_grad <= to_unsigned(24, 7) - resize(pixel_y(11 downto 5), 7);
            else
                bg2_grad <= (others => '0');
            end if;

            -- BG3: register the read data and combine (nebula/4 + gradient)
            if bg_on_r = '1' then
                bg3_w <= resize(unsigned(bg2_neb(7 downto 2)), 8)
                       + resize(bg2_grad, 8);
            else
                bg3_w <= (others => '0');
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c1 : pairwise (8->4) sum and max — registered
            ----------------------------------------------------------------
            s4c1_sum_01 <= resize(s4e_w(0), 9) + resize(s4e_w(1), 9);
            s4c1_sum_23 <= resize(s4e_w(2), 9) + resize(s4e_w(3), 9);
            s4c1_sum_45 <= resize(s4e_w(4), 9) + resize(s4e_w(5), 9);
            s4c1_sum_67 <= resize(s4e_w(6), 9) + resize(s4e_w(7), 9);

            if s4e_w(0) >= s4e_w(1) then
                s4c1_max01_w <= s4e_w(0); s4c1_max01_ci <= s4e_ci(0);
            else
                s4c1_max01_w <= s4e_w(1); s4c1_max01_ci <= s4e_ci(1);
            end if;
            if s4e_w(2) >= s4e_w(3) then
                s4c1_max23_w <= s4e_w(2); s4c1_max23_ci <= s4e_ci(2);
            else
                s4c1_max23_w <= s4e_w(3); s4c1_max23_ci <= s4e_ci(3);
            end if;
            if s4e_w(4) >= s4e_w(5) then
                s4c1_max45_w <= s4e_w(4); s4c1_max45_ci <= s4e_ci(4);
            else
                s4c1_max45_w <= s4e_w(5); s4c1_max45_ci <= s4e_ci(5);
            end if;
            if s4e_w(6) >= s4e_w(7) then
                s4c1_max67_w <= s4e_w(6); s4c1_max67_ci <= s4e_ci(6);
            else
                s4c1_max67_w <= s4e_w(7); s4c1_max67_ci <= s4e_ci(7);
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c2 : quad (4->2) sum and max — registered
            ----------------------------------------------------------------
            s4c2_sum_0123 <= resize(s4c1_sum_01, 10) + resize(s4c1_sum_23, 10);
            s4c2_sum_4567 <= resize(s4c1_sum_45, 10) + resize(s4c1_sum_67, 10);

            if s4c1_max01_w >= s4c1_max23_w then
                s4c2_max0123_w <= s4c1_max01_w; s4c2_max0123_ci <= s4c1_max01_ci;
            else
                s4c2_max0123_w <= s4c1_max23_w; s4c2_max0123_ci <= s4c1_max23_ci;
            end if;
            if s4c1_max45_w >= s4c1_max67_w then
                s4c2_max4567_w <= s4c1_max45_w; s4c2_max4567_ci <= s4c1_max45_ci;
            else
                s4c2_max4567_w <= s4c1_max67_w; s4c2_max4567_ci <= s4c1_max67_ci;
            end if;

            ----------------------------------------------------------------
            -- STAGE 4c3 : final (2->1) sum + clamp + final max + palette
            ----------------------------------------------------------------
            total_w := resize(s4c2_sum_0123, 11) + resize(s4c2_sum_4567, 11)
                     + resize(bg3_w, 11);

            if s4c2_max0123_w >= s4c2_max4567_w then
                best_ci := s4c2_max0123_ci;
                max_w   := s4c2_max0123_w;
            else
                best_ci := s4c2_max4567_ci;
                max_w   := s4c2_max4567_w;
            end if;

            if total_w > to_unsigned(1023, 11) then
                s4c3_w <= (others => '1');
            else
                s4c3_w <= total_w(9 downto 0);
            end if;
            s4c3_ci <= best_ci;
            -- Star chroma wins over background tint when a star meaningfully
            -- contributes to this pixel
            if max_w >= to_unsigned(8, 8) then
                s4c3_sf <= '1';
            else
                s4c3_sf <= '0';
            end if;

            ----------------------------------------------------------------
            -- STAGE 4m1 : register the brightness-multiplier operands and
            -- perform the shimmer-palette EBR read (this register is the
            -- EBR output register; all address bits come from registers).
            ----------------------------------------------------------------
            s4m1_w      <= s4c3_w;
            s4m1_bright <= bright;
            s4m1_sf     <= s4c3_sf;
            paddr_v     := shim_cnt(15 downto 12) & pal_sel_r & s4c3_ci;
            s4m1_pal    <= C_SHIMPAL(to_integer(paddr_v));

            ----------------------------------------------------------------
            -- STAGE 4m2 : shared 10x10 brightness multiply, product
            -- registered here (valid two cycles after the operands).
            -- Weight 255 at full brightness maps to Y ~= 1019; stacked
            -- stars overshoot and are clamped in stage 5. Shimmer palette
            -- data is registered off the EBR read, with the B&W override.
            ----------------------------------------------------------------
            prod_v := s4m1_w * s4m1_bright;
            s4m2_y <= prod_v(19 downto 8);
            s4m2_sf <= s4m1_sf;
            if bw_r = '1' then
                s4m2_u <= to_unsigned(512, 10);
                s4m2_v <= to_unsigned(512, 10);
            else
                s4m2_u <= unsigned(s4m1_pal(19 downto 10));
                s4m2_v <= unsigned(s4m1_pal(9 downto 0));
            end if;

            ----------------------------------------------------------------
            -- STAGE 5 : final clamp + chroma select. Background luma
            -- already arrived through the weight sum; here we only pick
            -- star chroma vs the deep-space tint (B&W forces neutral).
            ----------------------------------------------------------------
            if s4m2_y > to_unsigned(1023, 12) then
                s5_y <= (others => '1');
            else
                s5_y <= s4m2_y(9 downto 0);
            end if;

            if s4m2_sf = '1' then
                s5_u <= s4m2_u;
                s5_v <= s4m2_v;
            elsif bg_on_r = '1' and bw_r = '0' then
                s5_u <= to_unsigned(430, 10);
                s5_v <= to_unsigned(620, 10);
            else
                s5_u <= to_unsigned(512, 10);
                s5_v <= to_unsigned(512, 10);
            end if;

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
