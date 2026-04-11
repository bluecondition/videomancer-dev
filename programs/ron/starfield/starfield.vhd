-- Starfield: shader-inspired glowing parallax starfield generator
--
-- 8 depth layers, cell-based star placement (16x16 cells), improved
-- octagonal distance glow, per-star random colours from four palettes.
-- Luma Mode maps incoming video brightness to star density.
--
-- Optimised for iCE40 HX8K: linear distance (no squaring),
-- shift-only falloff, no per-layer multipliers.
--
-- Pipeline latency: 5 clocks
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Layers    (top 3 bits: 000=1 .. 111=8)
--   registers_in(1) = Density   (0-1023, manual density / luma gain)
--   registers_in(2) = Speed     (0-1023, max ~38 px/frame)
--   registers_in(3) = Palette   (top 2 bits: 00=Mixed 01=Cool 10=Warm 11=Rainbow)
--   registers_in(4) = Glow      (top 2 bits: 0-3 radius add, 50% eligible)
--   registers_in(5) = Depth     (top 3 bits: layer parallax spread)
--   registers_in(6) = Switches  (bit0=Dir bit1=BW bit2=Luma bit3=BG bit4=Bypass)
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

    constant LATENCY   : natural := 5;
    constant NUM_LYRS  : natural := 8;

    type t_pipe    is array (natural range <>) of t_video_stream_yuv444_30b;
    type t_u24_arr is array(0 to NUM_LYRS-1) of unsigned(23 downto 0);
    type t_u16_arr is array(0 to NUM_LYRS-1) of unsigned(15 downto 0);
    type t_u4_arr  is array(0 to NUM_LYRS-1) of unsigned(3 downto 0);

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

    -- Stage 2
    signal s2_sx : t_u24_arr;
    signal s2_y  : unsigned(11 downto 0);

    -- Stage 3
    signal s3_hash  : t_u16_arr;
    signal s3_sub_x : t_u4_arr;
    signal s3_sub_y : unsigned(3 downto 0);

    -- Stage 4
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
        variable speed7        : unsigned(6 downto 0);
        variable depth_3       : unsigned(2 downto 0);
        variable decel_1step   : unsigned(15 downto 0);
        variable accum_decel   : unsigned(15 downto 0);
        variable layer_speed   : unsigned(15 downto 0);

        -- Controls
        variable num_layers_3  : unsigned(2 downto 0);
        variable density       : unsigned(9 downto 0);
        variable palette_sel   : unsigned(1 downto 0);
        variable glow_add      : unsigned(1 downto 0);
        variable bright        : unsigned(9 downto 0);
        variable threshold     : unsigned(15 downto 0);

        -- Stage 3
        variable cell_x_v      : unsigned(19 downto 0);
        variable cx16          : unsigned(15 downto 0);
        variable cell_y_v      : unsigned(7 downto 0);

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
        variable glow_elig     : boolean;
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

                -- Speed: ~30 % of old max.
                speed16 := resize(unsigned(registers_in(2)(9 downto 5)), 16);
                speed16 := speed16 + shift_right(speed16, 2);
                speed7  := speed16(6 downto 0);

                -- Depth knob: parallax spread between layers.
                -- Always >= 1 (no flat mode). 7 = extreme parallax.
                depth_3 := unsigned(registers_in(5)(9 downto 7));

                case to_integer(depth_3) is
                    when 0 =>
                        decel_1step := to_unsigned(1, 16);
                    when 1 =>
                        decel_1step := resize(shift_right(speed16, 5), 16)
                                     + to_unsigned(1, 16);
                    when 2 =>
                        decel_1step := resize(shift_right(speed16, 4), 16)
                                     + to_unsigned(1, 16);
                    when 3 =>
                        decel_1step := resize(shift_right(speed16, 3), 16)
                                     + to_unsigned(1, 16);
                    when 4 =>
                        decel_1step := resize(shift_right(speed16, 3), 16)
                                     + resize(shift_right(speed16, 4), 16)
                                     + to_unsigned(1, 16);
                    when 5 =>
                        decel_1step := resize(shift_right(speed16, 2), 16)
                                     + to_unsigned(1, 16);
                    when 6 =>
                        decel_1step := resize(shift_right(speed16, 2), 16)
                                     + resize(shift_right(speed16, 3), 16)
                                     + to_unsigned(1, 16);
                    when others =>
                        decel_1step := resize(shift_right(speed16, 1), 16)
                                     + to_unsigned(1, 16);
                end case;

                -- Scroll: layer 7 (near) full speed, farther layers slower.
                if speed16 > to_unsigned(0, 16) then
                    accum_decel := (others => '0');
                    for layer in 7 downto 0 loop
                        if speed16 > accum_decel then
                            layer_speed := speed16 - accum_decel;
                        else
                            layer_speed := to_unsigned(1, 16);
                        end if;

                        if registers_in(6)(0) = '0' then
                            scroll(layer) <= scroll(layer)
                                           + resize(layer_speed, 24);
                        else
                            scroll(layer) <= scroll(layer)
                                           - resize(layer_speed, 24);
                        end if;

                        accum_decel := accum_decel + decel_1step;
                    end loop;
                end if;
            end if;

            ----------------------------------------------------------------
            -- STAGE 2 : Scrolled X per layer (24-bit)
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                s2_sx(i) <= resize(pixel_x, 24) + scroll(i);
            end loop;
            s2_y <= pixel_y;

            ----------------------------------------------------------------
            -- STAGE 3 : Hash cell coordinates, capture sub positions
            ----------------------------------------------------------------
            for i in 0 to NUM_LYRS-1 loop
                cell_x_v := s2_sx(i)(23 downto 4);
                cx16     := cell_x_v(15 downto 0)
                          xor resize(cell_x_v(19 downto 16), 16);
                cell_y_v := s2_y(11 downto 4);
                s3_hash(i) <= hash_cell(
                    cx16,
                    resize(cell_y_v, 16),
                    SEEDS(i)
                );
                s3_sub_x(i) <= s2_sx(i)(3 downto 0);
            end loop;
            s3_sub_y <= s2_y(3 downto 0);

            ----------------------------------------------------------------
            -- STAGE 4 : Per-layer star detection, glow, colour
            ----------------------------------------------------------------
            num_layers_3 := unsigned(registers_in(0)(9 downto 7));
            density      := unsigned(registers_in(1));
            palette_sel  := unsigned(registers_in(3)(9 downto 8));
            glow_add     := unsigned(registers_in(4)(9 downto 8));
            bright       := unsigned(registers_in(7));

            -- Density threshold: manual knob or input video luma.
            -- Luma Mode (switch 2): bright video areas = more stars.
            -- pipe(2).y is roughly aligned with the pixel being hashed.
            if registers_in(6)(2) = '1' then
                threshold := (others => '0');
                threshold(12 downto 3) := unsigned(pipe(2).y);
            else
                threshold := (others => '0');
                threshold(12 downto 3) := density;
            end if;

            total_y     := (others => '0');
            best_u      := to_unsigned(512, 10);
            best_v      := to_unsigned(512, 10);
            max_layer_y := (others => '0');

            for i in 0 to NUM_LYRS-1 loop

                layer_active := i >= (7 - to_integer(num_layers_3));

                layer_y := (others => '0');

                if layer_active then
                    hash_v := s3_hash(i);

                    offset_x := resize(hash_v(2 downto 0), 4)
                              + to_unsigned(4, 4);
                    offset_y := resize(hash_v(5 downto 3), 4)
                              + to_unsigned(4, 4);

                    -- Independent property bits: XOR-fold lower nibbles
                    -- so properties are uniform regardless of density
                    -- threshold (which constrains upper hash bits).
                    prop_bits := hash_v(3 downto 0)
                               xor hash_v(7 downto 4);

                    -- Star base size: heavily weighted toward 1px.
                    --   75% -> 1px (rad 0), ~6% -> small (rad 1),
                    --   ~19% -> medium (rad 2).
                    case to_integer(prop_bits) is
                        when 0 | 1 | 2 | 3 | 4 | 5 |
                             6 | 7 | 8 | 9 | 10 | 11 =>
                            base_rad := to_unsigned(0, 3);
                        when 12 =>
                            base_rad := to_unsigned(1, 3);
                        when others =>
                            base_rad := to_unsigned(2, 3);
                    end case;

                    -- Glow eligibility: 50% of stars respond to glow knob.
                    glow_elig := prop_bits(3) = '1';

                    -- Effective radius: base + glow addition (for eligible).
                    if glow_elig then
                        eff_rad := resize(base_rad, 5)
                                 + resize(glow_add, 5);
                    else
                        eff_rad := resize(base_rad, 5);
                    end if;

                    -- Improved octagonal distance: max + min*3/8.
                    if s3_sub_x(i) >= offset_x then
                        dx := s3_sub_x(i) - offset_x;
                    else
                        dx := offset_x - s3_sub_x(i);
                    end if;
                    if s3_sub_y >= offset_y then
                        dy := s3_sub_y - offset_y;
                    else
                        dy := offset_y - s3_sub_y;
                    end if;

                    if dx >= dy then
                        mn := dy;
                        dist := resize(dx, 5);
                    else
                        mn := dx;
                        dist := resize(dy, 5);
                    end if;
                    mn_x3 := shift_left(resize(mn, 6), 1)
                           + resize(mn, 6);
                    dist := dist + resize(shift_right(mn_x3, 3), 5);

                    -- Colour from independent folded bits
                    color_idx := hash_v(1 downto 0)
                               xor hash_v(5 downto 4);

                    if hash_v < threshold and dist <= eff_rad then

                        -- Smooth falloff from star center.
                        case to_integer(dist) is
                            when 0 =>
                                layer_y := bright;
                            when 1 =>
                                layer_y := shift_right(bright, 1)
                                         + shift_right(bright, 2);
                            when 2 =>
                                layer_y := shift_right(bright, 1)
                                         + shift_right(bright, 3);
                            when 3 =>
                                layer_y := shift_right(bright, 1);
                            when 4 =>
                                layer_y := shift_right(bright, 2)
                                         + shift_right(bright, 3);
                            when others =>
                                layer_y := shift_right(bright, 2);
                        end case;

                        -- Depth fade: 8 layers, near=100% far=25%
                        case i is
                            when 0 =>
                                layer_y := shift_right(layer_y, 2);
                            when 1 =>
                                layer_y := shift_right(layer_y, 2)
                                         + shift_right(layer_y, 3);
                            when 2 =>
                                layer_y := shift_right(layer_y, 1);
                            when 3 =>
                                layer_y := shift_right(layer_y, 1)
                                         + shift_right(layer_y, 3);
                            when 4 =>
                                layer_y := layer_y
                                         - shift_right(layer_y, 2);
                            when 5 =>
                                layer_y := layer_y
                                         - shift_right(layer_y, 3);
                            when 6 =>
                                layer_y := layer_y
                                         - shift_right(layer_y, 4);
                            when 7 =>
                                null;
                            when others =>
                                null;
                        end case;

                    end if;
                end if;

                total_y := total_y + resize(layer_y, 13);

                if layer_y > max_layer_y then
                    max_layer_y := layer_y;

                    if registers_in(6)(1) = '1' then
                        best_u := to_unsigned(512, 10);
                        best_v := to_unsigned(512, 10);
                    else
                        case palette_sel is

                            -- Mixed: white, blue, gold, pink
                            when "00" =>
                                case color_idx is
                                    when "00"   => best_u := to_unsigned(512, 10);
                                                   best_v := to_unsigned(512, 10);
                                    when "01"   => best_u := to_unsigned(300, 10);
                                                   best_v := to_unsigned(760, 10);
                                    when "10"   => best_u := to_unsigned(600, 10);
                                                   best_v := to_unsigned(380, 10);
                                    when others => best_u := to_unsigned(580, 10);
                                                   best_v := to_unsigned(620, 10);
                                end case;

                            -- Cool: white, blue, purple, gray-blue
                            when "01" =>
                                case color_idx is
                                    when "00"   => best_u := to_unsigned(512, 10);
                                                   best_v := to_unsigned(512, 10);
                                    when "01"   => best_u := to_unsigned(280, 10);
                                                   best_v := to_unsigned(800, 10);
                                    when "10"   => best_u := to_unsigned(620, 10);
                                                   best_v := to_unsigned(720, 10);
                                    when others => best_u := to_unsigned(480, 10);
                                                   best_v := to_unsigned(560, 10);
                                end case;

                            -- Warm: white, orange, amber, pale gold
                            when "10" =>
                                case color_idx is
                                    when "00"   => best_u := to_unsigned(512, 10);
                                                   best_v := to_unsigned(512, 10);
                                    when "01"   => best_u := to_unsigned(720, 10);
                                                   best_v := to_unsigned(340, 10);
                                    when "10"   => best_u := to_unsigned(640, 10);
                                                   best_v := to_unsigned(400, 10);
                                    when others => best_u := to_unsigned(560, 10);
                                                   best_v := to_unsigned(460, 10);
                                end case;

                            -- Rainbow: white, red, cyan, magenta
                            when others =>
                                case color_idx is
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
                end if;

            end loop;

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
            data_out <= pipe(LATENCY - 1);

            if registers_in(6)(4) = '0' then
                data_out.y <= std_logic_vector(s5_y);
                data_out.u <= std_logic_vector(s5_u);
                data_out.v <= std_logic_vector(s5_v);
            end if;

        end if;
    end process;

end architecture starfield;
