-- Solitude: Glitch Cascade on Teal
--
-- A ragged central mass of straight vertical pixel-sort stripes in jewel
-- tones (magenta/red/blue/green) floats on a flat muted teal background.
--
-- The mass's OUTLINE zigzags downward — its horizontal center shifts
-- left-right-left-right as y increases from top to bottom, giving a
-- lightning-bolt-style cascade shape rather than a rectangular block.
-- Per-row hashed noise jitters the left/right edges for additional
-- jaggedness. The stripes THEMSELVES are straight vertical — each screen
-- column has a single colour for its entire height.
--
-- The top edge of the mass is a jagged city-skyline silhouette with tall
-- peaks and deep valleys. The bottom has long "drip" columns that extend
-- downward into the teal.
--
-- Thin white branching lightning caps the mass's top edge, densest right
-- at the cliff line and fading with distance below. Two hash thresholds
-- combine for a "branching" look rather than plain speckle.
--
-- Register map (t_spi_ram, each std_logic_vector(9 downto 0)):
--   registers_in(0) = Edge     (0-1023, top-edge jag amplitude)
--   registers_in(1) = Drip     (0-1023, drip frequency/length)
--   registers_in(2) = Lit Dns  (0-1023, lightning density near top)
--   registers_in(3) = Cascade  (0-1023, horizontal zigzag amplitude)
--   registers_in(4) = Stripe   (0-1023, stripe width 1..8 via top 2 bits)
--   registers_in(5) = Lit Depth(0-1023, how far down lightning extends)
--   registers_in(6) = Switches (bit0=Palette bit1=BG bit2=Cascade
--                               bit3=Lightning bit4=Bypass)
--   registers_in(7) = Brightness (0-1023)
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
use work.all;

architecture solitude of program_top is

    constant LATENCY : natural := 12;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    constant C_TEAL_Y : unsigned(9 downto 0) := to_unsigned(568, 10);
    constant C_TEAL_U : unsigned(9 downto 0) := to_unsigned(608, 10);
    constant C_TEAL_V : unsigned(9 downto 0) := to_unsigned(300, 10);

    constant C_WHITE_Y : unsigned(9 downto 0) := to_unsigned(1000, 10);
    constant C_WHITE_U : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_WHITE_V : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal max_y_r      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    -- Per-frame geometry
    signal mass_top_r    : unsigned(11 downto 0) := to_unsigned(60, 12);
    signal mass_bottom_r : unsigned(11 downto 0) := to_unsigned(1020, 12);
    signal zone1_x_r     : unsigned(11 downto 0) := to_unsigned(600, 12);
    signal zone2_x_r     : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal zone3_x_r     : unsigned(11 downto 0) := to_unsigned(1320, 12);
    signal jag_amount_r     : unsigned(11 downto 0) := to_unsigned(60, 12);
    signal drip_bonus_max_r : unsigned(11 downto 0) := to_unsigned(60, 12);

    -- Cascade (mass-outline snake-path). The mass's horizontal centre is
    -- offset per y-row by a smoothly-varying triangle wave of pixel_y,
    -- plus per-row noise for organic jaggedness. Base mass bounds are
    -- computed at vsync; the per-row offset is added at runtime.
    signal mass_left_r       : unsigned(11 downto 0) := to_unsigned(240, 12);
    signal mass_right_r      : unsigned(11 downto 0) := to_unsigned(1680, 12);
    signal mass_left_edge_r  : unsigned(11 downto 0) := to_unsigned(264, 12);
    signal mass_right_edge_r : unsigned(11 downto 0) := to_unsigned(1656, 12);
    signal cascade_amp_r     : unsigned(10 downto 0) := to_unsigned(120, 11);

    -- Stage 0n signals: register the combined triangle input before the
    -- cascade multiplier. Decoupling the triangle compute from the 9×11
    -- multiply keeps the pixel_y → cascade_offset path within 74.25 MHz.
    signal s0n_x             : unsigned(11 downto 0) := (others => '0');
    signal s0n_y             : unsigned(11 downto 0) := (others => '0');
    signal s0n_noise_hash    : unsigned(11 downto 0) := (others => '0');
    signal s0n_tri_combined  : signed(8 downto 0)    := (others => '0');

    -- Stage 0m signals: registered cascade-offset multiplier output.
    signal s0m_x             : unsigned(11 downto 0) := (others => '0');
    signal s0m_y             : unsigned(11 downto 0) := (others => '0');
    signal s0m_noise_hash    : unsigned(11 downto 0) := (others => '0');
    signal s0m_cascade_offset: signed(12 downto 0)   := (others => '0');

    -- Stage 0h signals: register effective mass bounds for this y,
    -- plus the "rotated y" value y - x/2 that stage 1 uses to pick a
    -- color-band. Slanted bands at 30° slope separate the 4 passes of
    -- the wall — each pass is a slanted strip with its own stripe seed.
    signal s0h_x               : unsigned(11 downto 0) := (others => '0');
    signal s0h_y               : unsigned(11 downto 0) := (others => '0');
    signal s0h_y_rot           : signed(13 downto 0)   := (others => '0');
    signal s0h_mass_left_eff   : unsigned(11 downto 0) := (others => '0');
    signal s0h_mass_right_eff  : unsigned(11 downto 0) := (others => '0');
    signal s0h_mass_left_edge  : unsigned(11 downto 0) := (others => '0');
    signal s0h_mass_right_edge : unsigned(11 downto 0) := (others => '0');

    -- Stage 1 signals
    signal s1_x          : unsigned(11 downto 0) := (others => '0');
    signal s1_y          : unsigned(11 downto 0) := (others => '0');
    signal s1_in_mass_h  : std_logic := '0';
    signal s1_near_left  : std_logic := '0';
    signal s1_near_right : std_logic := '0';
    signal s1_zone       : unsigned(1 downto 0) := (others => '0');
    signal s1_col_hash   : unsigned(15 downto 0) := (others => '0');
    signal s1_lit_hash   : unsigned(15 downto 0) := (others => '0');
    signal s1_is_border  : std_logic := '0';  -- on seam between color bands
    signal s1_is_invert  : std_logic := '0';  -- row marked for color inversion
    signal s1_step_off   : unsigned(7 downto 0) := (others => '0');  -- coarse stepped top offset

    -- Stage 1m
    signal s1m_x          : unsigned(11 downto 0) := (others => '0');
    signal s1m_y          : unsigned(11 downto 0) := (others => '0');
    signal s1m_in_mass_h  : std_logic := '0';
    signal s1m_near_left  : std_logic := '0';
    signal s1m_near_right : std_logic := '0';
    signal s1m_zone       : unsigned(1 downto 0) := (others => '0');
    signal s1m_col_hash   : unsigned(15 downto 0) := (others => '0');
    signal s1m_lit_hash   : unsigned(15 downto 0) := (others => '0');
    signal s1m_jag_raw    : unsigned(19 downto 0) := (others => '0');
    signal s1m_drip_raw   : unsigned(19 downto 0) := (others => '0');
    signal s1m_is_border  : std_logic := '0';
    signal s1m_is_invert  : std_logic := '0';
    signal s1m_step_off   : unsigned(7 downto 0) := (others => '0');

    -- Stage 2a
    signal s2a_y          : unsigned(11 downto 0) := (others => '0');
    signal s2a_top_y      : unsigned(11 downto 0) := (others => '0');
    signal s2a_drip_y     : unsigned(11 downto 0) := (others => '0');
    signal s2a_in_mass_h  : std_logic := '0';
    signal s2a_edge_cut   : std_logic := '0';
    signal s2a_zone       : unsigned(1 downto 0) := (others => '0');
    signal s2a_color_bin  : unsigned(2 downto 0) := (others => '0');
    signal s2a_lit_hash   : unsigned(15 downto 0) := (others => '0');
    signal s2a_is_border  : std_logic := '0';
    signal s2a_is_invert  : std_logic := '0';

    -- Stage 2b
    signal s2b_in_mass    : std_logic := '0';
    signal s2b_zone       : unsigned(1 downto 0) := (others => '0');
    signal s2b_color_bin  : unsigned(2 downto 0) := (others => '0');
    signal s2b_y_from_top : unsigned(11 downto 0) := (others => '0');
    signal s2b_lit_hash   : unsigned(15 downto 0) := (others => '0');
    signal s2b_is_border  : std_logic := '0';
    signal s2b_is_invert  : std_logic := '0';

    -- Stage 2c
    signal s2_in_mass    : std_logic := '0';
    signal s2_zone       : unsigned(1 downto 0) := (others => '0');
    signal s2_color_bin  : unsigned(2 downto 0) := (others => '0');
    signal s2_is_lit     : std_logic := '0';
    signal s2_is_border  : std_logic := '0';
    signal s2_is_invert  : std_logic := '0';
    signal s2_is_speckle : std_logic := '0';
    signal s2_x          : unsigned(11 downto 0) := (others => '0');
    signal s2_y          : unsigned(11 downto 0) := (others => '0');

    -- Stage 3
    signal s3_in_mass    : std_logic := '0';
    signal s3_y_val      : unsigned(9 downto 0) := (others => '0');
    signal s3_u_val      : unsigned(9 downto 0) := (others => '0');
    signal s3_v_val      : unsigned(9 downto 0) := (others => '0');

    -- Stage 4
    signal s4_y          : unsigned(9 downto 0) := (others => '0');
    signal s4_u          : unsigned(9 downto 0) := (others => '0');
    signal s4_v          : unsigned(9 downto 0) := (others => '0');

    -- Stage 5
    signal s5_y          : unsigned(9 downto 0) := (others => '0');
    signal s5_u          : unsigned(9 downto 0) := (others => '0');
    signal s5_v          : unsigned(9 downto 0) := (others => '0');

begin

    process(clk)
        variable x, y          : unsigned(11 downto 0);
        variable mass_width    : unsigned(11 downto 0);

        -- Cascade locals
        variable row_noise        : unsigned(11 downto 0);
        variable noise_hash       : unsigned(11 downto 0);
        variable tri1_unsigned    : unsigned(6 downto 0);
        variable tri1_s           : signed(7 downto 0);
        variable tri2_unsigned    : unsigned(4 downto 0);
        variable tri2_s           : signed(5 downto 0);
        variable tri_combined_s   : signed(8 downto 0);
        variable cascade_product_v: signed(20 downto 0);
        variable ml_signed        : signed(13 downto 0);
        variable mr_signed        : signed(13 downto 0);
        variable mle_signed       : signed(13 downto 0);
        variable mre_signed       : signed(13 downto 0);
        variable mass_left_eff    : unsigned(11 downto 0);
        variable mass_right_eff   : unsigned(11 downto 0);
        variable mass_left_edge_v : unsigned(11 downto 0);
        variable mass_right_edge_v: unsigned(11 downto 0);

        -- Stripe
        variable stripe_idx    : unsigned(10 downto 0);
        variable stripe_shift  : integer range 0 to 3;

        -- Hashes
        variable hash_col      : unsigned(15 downto 0);
        variable hash_lit      : unsigned(15 downto 0);
        variable stripe_seed   : unsigned(15 downto 0);

        -- Chaos effects (stage 1)
        variable y_rot_frac_v  : unsigned(5 downto 0);
        variable border_jit_v  : unsigned(2 downto 0);
        variable row_hash_v    : unsigned(11 downto 0);
        variable step_hash_v   : unsigned(11 downto 0);

        -- Stage 2a
        variable jag_offset    : unsigned(11 downto 0);
        variable top_y_v       : unsigned(11 downto 0);
        variable drip_y_v      : unsigned(11 downto 0);
        variable edge_cut      : boolean;
        variable drip_extend   : boolean;
        variable in_mass_v     : boolean;
        variable color_bin_v   : unsigned(2 downto 0);
        variable y_from_top    : unsigned(11 downto 0);
        variable lit_threshold : unsigned(9 downto 0);
        -- x-based top-edge zigzag
        variable x_tri_uns     : unsigned(6 downto 0);
        variable x_tri_s       : signed(7 downto 0);
        variable top_y_sgn     : signed(13 downto 0);
        variable slope_offset  : unsigned(11 downto 0);

        -- Stage 3 palette
        variable pal_idx       : unsigned(2 downto 0);
        variable pal_y_v       : unsigned(9 downto 0);
        variable pal_u_v       : unsigned(9 downto 0);
        variable pal_v_v       : unsigned(9 downto 0);

        -- Controls
        variable edge_knob     : unsigned(9 downto 0);
        variable drip_knob     : unsigned(9 downto 0);
        variable lit_knob      : unsigned(9 downto 0);
        variable cascade_knob  : unsigned(9 downto 0);
        variable stripe_knob   : unsigned(9 downto 0);
        variable lit_depth     : unsigned(9 downto 0);
        variable bright_knob   : unsigned(9 downto 0);
        variable palette_cool  : boolean;
        variable bg_black      : boolean;
        variable cascade_on    : boolean;
        variable lit_on        : boolean;
        variable bypass_on     : boolean;

        -- Stage 4/5
        variable bg_y_v, bg_u_v, bg_v_v : unsigned(9 downto 0);
        variable comp_y, comp_u, comp_v : unsigned(9 downto 0);
        variable y_scaled      : unsigned(19 downto 0);

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
            -- Read controls
            ----------------------------------------------------------------
            edge_knob    := unsigned(registers_in(0));
            drip_knob    := unsigned(registers_in(1));
            lit_knob     := unsigned(registers_in(2));
            cascade_knob := unsigned(registers_in(3));
            stripe_knob  := unsigned(registers_in(4));
            lit_depth    := unsigned(registers_in(5));
            palette_cool := (registers_in(6)(0) = '1');
            bg_black     := (registers_in(6)(1) = '1');
            cascade_on   := (registers_in(6)(2) = '1');
            lit_on       := (registers_in(6)(3) = '1');
            bypass_on    := (registers_in(6)(4) = '1');
            bright_knob  := unsigned(registers_in(7));

            ----------------------------------------------------------------
            -- STAGE 0 : Position tracking, per-frame geometry at vsync
            ----------------------------------------------------------------
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
                pixel_y     <= (others => '0');
                frame_count <= frame_count + 1;

                mass_top_r    <= shift_right(max_y_r, 4);
                mass_bottom_r <= max_y_r - shift_right(max_y_r, 5);

                -- Base mass horizontal bounds: centered 75 % of width.
                mass_left_r       <= shift_right(max_x_r, 3);
                mass_right_r      <= max_x_r - shift_right(max_x_r, 3);
                mass_left_edge_r  <= shift_right(max_x_r, 3)
                                   + shift_right(max_x_r, 4);
                mass_right_edge_r <= max_x_r - shift_right(max_x_r, 3)
                                   - shift_right(max_x_r, 4);

                -- Cascade amplitude: scales max pixel offset by knob. At
                -- knob=max the offset reaches ≈ max_x/8 either side; the
                -- triangle wave's −64..+63 signed output is multiplied by
                -- cascade_amp_r and then >> 6, so full amplitude ≈ amp.
                -- Side cascade is now OFF by default — the zigzag lives
                -- on the TOP edge instead (see stage 2a). Cascade knob /
                -- switch still wired so the old side-zigzag can be dialled
                -- back in if wanted, but the factory default keeps it at 0.
                if cascade_on then
                    case cascade_knob(9 downto 7) is
                        when "000" =>
                            cascade_amp_r <= to_unsigned(0, 11);
                        when "001" =>
                            cascade_amp_r <= resize(shift_right(max_x_r, 7), 11);
                        when "010" =>
                            cascade_amp_r <= resize(shift_right(max_x_r, 6), 11);
                        when "011" =>
                            cascade_amp_r <= resize(shift_right(max_x_r, 5), 11);
                        when "100" =>
                            cascade_amp_r <= resize(shift_right(max_x_r, 5), 11)
                                          + resize(shift_right(max_x_r, 6), 11);
                        when "101" =>
                            cascade_amp_r <= resize(shift_right(max_x_r, 4), 11);
                        when "110" =>
                            cascade_amp_r <= resize(shift_right(max_x_r, 4), 11)
                                          + resize(shift_right(max_x_r, 5), 11);
                        when others =>
                            cascade_amp_r <= resize(shift_right(max_x_r, 3), 11);
                    end case;
                else
                    cascade_amp_r <= to_unsigned(0, 11);
                end if;

                mass_width := max_x_r - shift_right(max_x_r, 2);
                zone1_x_r <= shift_right(max_x_r, 3)
                           + shift_right(mass_width, 2);
                zone2_x_r <= shift_right(max_x_r, 3)
                           + shift_right(mass_width, 1);
                zone3_x_r <= max_x_r - shift_right(max_x_r, 3)
                                     - shift_right(mass_width, 2);

                case edge_knob(9 downto 7) is
                    when "000" =>
                        jag_amount_r <= shift_right(max_y_r, 5);
                    when "001" =>
                        jag_amount_r <= shift_right(max_y_r, 4);
                    when "010" =>
                        jag_amount_r <= shift_right(max_y_r, 4)
                                      + shift_right(max_y_r, 5);
                    when "011" =>
                        jag_amount_r <= shift_right(max_y_r, 3);
                    when "100" =>
                        jag_amount_r <= shift_right(max_y_r, 3)
                                      + shift_right(max_y_r, 4);
                    when "101" =>
                        jag_amount_r <= shift_right(max_y_r, 2);
                    when "110" =>
                        jag_amount_r <= shift_right(max_y_r, 2)
                                      + shift_right(max_y_r, 4);
                    when others =>
                        jag_amount_r <= shift_right(max_y_r, 1);
                end case;

                drip_bonus_max_r <= max_y_r
                                  - (max_y_r - shift_right(max_y_r, 5));
            end if;

            ----------------------------------------------------------------
            -- STAGE 0n : row-noise hash + combined triangle wave (no
            -- multiply yet — that goes to stage 0m).
            ----------------------------------------------------------------
            noise_hash := pixel_y xor to_unsigned(16#A5E#, 12);

            -- Primary triangle (period 128, ±64). pixel_y(7) flips the
            -- lower-7-bit phase to give a clean saw→fall pattern.
            if pixel_y(7) = '0' then
                tri1_unsigned := pixel_y(6 downto 0);
            else
                tri1_unsigned := not pixel_y(6 downto 0);
            end if;
            tri1_s := signed('0' & tri1_unsigned) - to_signed(64, 8);

            -- Secondary triangle (period 32, ±16) adds a faster wiggle so
            -- the path reads as a curvy road rather than a clean triangle.
            if pixel_y(5) = '0' then
                tri2_unsigned := pixel_y(4 downto 0);
            else
                tri2_unsigned := not pixel_y(4 downto 0);
            end if;
            tri2_s := signed('0' & tri2_unsigned) - to_signed(16, 6);

            s0n_x           <= pixel_x;
            s0n_y           <= pixel_y;
            s0n_noise_hash  <= noise_hash;
            s0n_tri_combined <= resize(tri1_s, 9) + resize(tri2_s, 9);

            ----------------------------------------------------------------
            -- STAGE 0m : cascade-offset multiplier (registered). Isolating
            -- the 9×11 multiplier on its own pipeline stage keeps every
            -- other combinational chain short enough for 74.25 MHz.
            ----------------------------------------------------------------
            cascade_product_v := s0n_tri_combined
                               * signed('0' & cascade_amp_r);
            s0m_cascade_offset <= resize(shift_right(cascade_product_v, 6), 13);
            s0m_x          <= s0n_x;
            s0m_y          <= s0n_y;
            s0m_noise_hash <= s0n_noise_hash;

            ----------------------------------------------------------------
            -- STAGE 0h : apply the registered cascade offset + per-row
            -- noise to the base mass bounds, with saturating clamp to
            -- [0, max_x]. Noise bit-slices differ for left/right so the
            -- two edges wobble independently (more organic).
            ----------------------------------------------------------------
            ml_signed  := signed('0' & mass_left_r)       + resize(s0m_cascade_offset, 14)
                                                           + signed('0' & resize(s0m_noise_hash(2 downto 0), 13));
            mle_signed := signed('0' & mass_left_edge_r)  + resize(s0m_cascade_offset, 14)
                                                           + signed('0' & resize(s0m_noise_hash(2 downto 0), 13));
            mr_signed  := signed('0' & mass_right_r)      + resize(s0m_cascade_offset, 14)
                                                           + signed('0' & resize(s0m_noise_hash(6 downto 4), 13));
            mre_signed := signed('0' & mass_right_edge_r) + resize(s0m_cascade_offset, 14)
                                                           + signed('0' & resize(s0m_noise_hash(6 downto 4), 13));

            if ml_signed < to_signed(0, 14) then
                mass_left_eff := (others => '0');
            elsif ml_signed > signed('0' & '0' & max_x_r) then
                mass_left_eff := max_x_r;
            else
                mass_left_eff := unsigned(ml_signed(11 downto 0));
            end if;

            if mle_signed < to_signed(0, 14) then
                mass_left_edge_v := (others => '0');
            elsif mle_signed > signed('0' & '0' & max_x_r) then
                mass_left_edge_v := max_x_r;
            else
                mass_left_edge_v := unsigned(mle_signed(11 downto 0));
            end if;

            if mr_signed < to_signed(0, 14) then
                mass_right_eff := (others => '0');
            elsif mr_signed > signed('0' & '0' & max_x_r) then
                mass_right_eff := max_x_r;
            else
                mass_right_eff := unsigned(mr_signed(11 downto 0));
            end if;

            if mre_signed < to_signed(0, 14) then
                mass_right_edge_v := (others => '0');
            elsif mre_signed > signed('0' & '0' & max_x_r) then
                mass_right_edge_v := max_x_r;
            else
                mass_right_edge_v := unsigned(mre_signed(11 downto 0));
            end if;

            s0h_x               <= s0m_x;
            s0h_y               <= s0m_y;
            -- Rotated y = y - x/2. Used in stage 1 to select a slanted
            -- color-band. Signed 14-bit so it stays correct across the
            -- canvas (y < x/2 on the right side, giving negative values).
            s0h_y_rot           <= signed(resize(s0m_y, 14))
                                 - signed(resize(shift_right(s0m_x, 1), 14));
            s0h_mass_left_eff   <= mass_left_eff;
            s0h_mass_right_eff  <= mass_right_eff;
            s0h_mass_left_edge  <= mass_left_edge_v;
            s0h_mass_right_edge <= mass_right_edge_v;

            ----------------------------------------------------------------
            -- STAGE 1 : horizontal mass test, zone, hashes.
            -- in_mass_h uses the y-dependent effective bounds registered
            -- in stage 0h so the outline cascades L/R/L/R down the frame.
            -- Stripes are keyed to raw x so each column has a fixed colour.
            ----------------------------------------------------------------
            x := s0h_x;
            y := s0h_y;
            s1_x <= x;
            s1_y <= y;

            if x >= s0h_mass_left_eff and x <= s0h_mass_right_eff then
                s1_in_mass_h <= '1';
            else
                s1_in_mass_h <= '0';
            end if;
            if x < s0h_mass_left_edge then
                s1_near_left <= '1';
            else
                s1_near_left <= '0';
            end if;
            if x > s0h_mass_right_edge then
                s1_near_right <= '1';
            else
                s1_near_right <= '0';
            end if;

            -- Zone based on raw x; each physical column has a stable palette
            -- slot for the whole frame.
            if x < zone1_x_r then
                s1_zone <= "00";
            elsif x < zone2_x_r then
                s1_zone <= "01";
            elsif x < zone3_x_r then
                s1_zone <= "10";
            else
                s1_zone <= "11";
            end if;

            case stripe_knob(9 downto 8) is
                when "00"   => stripe_shift := 0;
                when "01"   => stripe_shift := 1;
                when "10"   => stripe_shift := 2;
                when others => stripe_shift := 3;
            end case;
            stripe_idx := shift_right(resize(x, 11), stripe_shift);

            -- Per-pass stripe hash seed. Each "pass" of the zigzag wall is
            -- a slanted band (30° down-right) 64 px thick in rotated-y
            -- space. Using s0h_y_rot(7:6) for the seed-select makes the
            -- seams between passes slant at the SAME 30° as the top edge
            -- — so as we descend the canvas we cross successive bands,
            -- each with its own colour palette, forming the "extension of
            -- the wall" stacked layers.
            case unsigned(s0h_y_rot(7 downto 6)) is
                when "00"   => stripe_seed := to_unsigned(16#A3B1#, 16);
                when "01"   => stripe_seed := to_unsigned(16#7C2D#, 16);
                when "10"   => stripe_seed := to_unsigned(16#D4A9#, 16);
                when others => stripe_seed := to_unsigned(16#6E3F#, 16);
            end case;

            hash_col := resize(stripe_idx, 16) xor stripe_seed;
            hash_col := hash_col xor shift_left(hash_col, 7);
            hash_col := hash_col xor shift_right(hash_col, 9);
            hash_col := hash_col xor shift_left(hash_col, 8);
            s1_col_hash <= hash_col;

            -- Lightning hash: diagonal continuity in (x, y).
            hash_lit := resize(x(11 downto 1), 16)
                      + shift_left(resize(y, 16), 1)
                      + resize(y, 16)
                      xor shift_left(resize(y, 16), 4)
                      xor to_unsigned(16#7C2D#, 16);
            hash_lit := hash_lit xor shift_left(hash_lit, 5);
            hash_lit := hash_lit xor shift_right(hash_lit, 11);
            hash_lit := hash_lit xor shift_left(hash_lit, 7);
            s1_lit_hash <= hash_lit;

            -- is_border: pixel at seam between y_rot colour bands. Uses
            -- fractional position within a band (low 6 bits of y_rot) and
            -- jitters the border thickness per stripe so the seam looks
            -- jagged, not ruler-straight.
            y_rot_frac_v := unsigned(s0h_y_rot(5 downto 0));
            border_jit_v := stripe_idx(2 downto 0);
            if (y_rot_frac_v < (to_unsigned(4, 6) + resize(border_jit_v, 6)))
               or (y_rot_frac_v > (to_unsigned(58, 6) - resize(border_jit_v, 6)))
            then
                s1_is_border <= '1';
            else
                s1_is_border <= '0';
            end if;

            -- is_invert: ~1/16 rows flagged so pixels there render with
            -- inverted palette colours. Horizontal glitch-band effect.
            row_hash_v := s0h_y xor to_unsigned(16#7C9#, 12);
            row_hash_v := row_hash_v xor shift_left(row_hash_v, 5);
            if row_hash_v(7 downto 4) = "0000" then
                s1_is_invert <= '1';
            else
                s1_is_invert <= '0';
            end if;

            -- step_off: coarse x-block hash (changes every 32 px of x).
            -- Added to top_y in stage 2a → abrupt stepped changes in the
            -- top edge height, so the line isn't a smooth slope-plus-wave.
            step_hash_v := resize(s0h_x(11 downto 5), 12)
                         xor to_unsigned(16#A5E#, 12);
            step_hash_v := step_hash_v xor shift_left(step_hash_v, 3);
            step_hash_v := step_hash_v xor shift_right(step_hash_v, 5);
            s1_step_off <= step_hash_v(7 downto 0);

            ----------------------------------------------------------------
            -- STAGE 1m : registered multiplier outputs.
            ----------------------------------------------------------------
            s1m_x          <= s1_x;
            s1m_y          <= s1_y;
            s1m_in_mass_h  <= s1_in_mass_h;
            s1m_near_left  <= s1_near_left;
            s1m_near_right <= s1_near_right;
            s1m_zone       <= s1_zone;
            s1m_col_hash   <= s1_col_hash;
            s1m_lit_hash   <= s1_lit_hash;
            s1m_jag_raw    <= s1_col_hash(7 downto 0) * jag_amount_r;
            s1m_drip_raw   <= s1_col_hash(7 downto 0) * drip_bonus_max_r;
            s1m_is_border  <= s1_is_border;
            s1m_is_invert  <= s1_is_invert;
            s1m_step_off   <= s1_step_off;

            ----------------------------------------------------------------
            -- STAGE 2a : top_y, drip_y, edge_cut, color_bin.
            ----------------------------------------------------------------
            hash_col := s1m_col_hash;
            hash_lit := s1m_lit_hash;
            x := s1m_x;
            y := s1m_y;

            -- Per-column noise (0..15 px) — OCCASIONAL noisy bumps on the
            -- sloped top line.
            jag_offset := resize(s1m_col_hash(3 downto 0), 12);

            -- Top-edge slope: x >> 2 ≈ 14° baseline on 16:9. Combined
            -- with the ±32 triangle below, the effective slope of the
            -- top edge averages in the 25–35° range and clearly reverses
            -- direction every ~128 px of x.
            slope_offset := shift_right(s1m_x, 2);

            -- Triangle wave of x, period 256 px (4 direction segments
            -- across the canvas at common resolutions), amplitude ±32.
            -- Large enough that the reversals are visible as the "back
            -- to the left / back to the right" behaviour.
            if s1m_x(7) = '0' then
                x_tri_uns := s1m_x(6 downto 0);
            else
                x_tri_uns := not s1m_x(6 downto 0);
            end if;
            x_tri_s := shift_right(signed('0' & x_tri_uns) - to_signed(64, 8), 1);

            -- Combined top-y: baseline + slope + triangle wave + per-column
            -- noise + coarse-x stepped offset. The step_off (0..255 px on
            -- 32-pixel x-blocks) creates abrupt, uneven height jumps so
            -- the top line looks stepped rather than a smooth curve.
            top_y_sgn := signed('0' & mass_top_r)
                       + signed('0' & slope_offset)
                       + resize(x_tri_s, 14)
                       + signed('0' & jag_offset)
                       + signed('0' & resize(s1m_step_off(6 downto 0), 13));
            if top_y_sgn < to_signed(0, 14) then
                top_y_v := (others => '0');
            elsif top_y_sgn >= signed('0' & mass_bottom_r) then
                top_y_v := mass_bottom_r - to_unsigned(8, 12);
            else
                top_y_v := unsigned(top_y_sgn(11 downto 0));
            end if;

            if hash_col(11 downto 4) < drip_knob(9 downto 2) then
                drip_extend := true;
            else
                drip_extend := false;
            end if;

            if drip_extend then
                drip_y_v := mass_bottom_r
                          + resize(shift_right(s1m_drip_raw, 8), 12);
                if drip_y_v > max_y_r then
                    drip_y_v := max_y_r;
                end if;
            else
                if mass_bottom_r > resize(hash_col(5 downto 0), 12) then
                    drip_y_v := mass_bottom_r
                              - resize(hash_col(5 downto 0), 12);
                else
                    drip_y_v := mass_bottom_r;
                end if;
            end if;

            -- Side raggedness: random drops of columns near the cascade
            -- edges.
            edge_cut := false;
            if (s1m_near_left = '1') or (s1m_near_right = '1') then
                if hash_col(15 downto 13) = "000" then
                    edge_cut := true;
                end if;
            end if;

            if hash_col(10 downto 8) = "000" then
                color_bin_v := to_unsigned(6, 3);
            else
                color_bin_v := hash_col(14 downto 12);
            end if;

            s2a_y         <= y;
            s2a_top_y     <= top_y_v;
            s2a_drip_y    <= drip_y_v;
            s2a_in_mass_h <= s1m_in_mass_h;
            if edge_cut then
                s2a_edge_cut <= '1';
            else
                s2a_edge_cut <= '0';
            end if;

            case hash_col(7 downto 6) is
                when "00" =>
                    if s1m_zone = "00" then
                        s2a_zone <= s1m_zone;
                    else
                        s2a_zone <= s1m_zone - 1;
                    end if;
                when "11" =>
                    if s1m_zone = "11" then
                        s2a_zone <= s1m_zone;
                    else
                        s2a_zone <= s1m_zone + 1;
                    end if;
                when others =>
                    s2a_zone <= s1m_zone;
            end case;

            s2a_color_bin <= color_bin_v;
            s2a_lit_hash  <= hash_lit;
            s2a_is_border <= s1m_is_border;
            s2a_is_invert <= s1m_is_invert;

            ----------------------------------------------------------------
            -- STAGE 2b : register y_from_top + final in_mass AND.
            ----------------------------------------------------------------
            in_mass_v := (s2a_in_mass_h = '1')
                       and (s2a_edge_cut = '0')
                       and (s2a_y >= s2a_top_y)
                       and (s2a_y <= s2a_drip_y);
            if in_mass_v then
                s2b_in_mass <= '1';
            else
                s2b_in_mass <= '0';
            end if;
            s2b_zone      <= s2a_zone;
            s2b_color_bin <= s2a_color_bin;
            s2b_lit_hash  <= s2a_lit_hash;
            s2b_is_border <= s2a_is_border;
            s2b_is_invert <= s2a_is_invert;

            if s2a_y >= s2a_top_y then
                s2b_y_from_top <= s2a_y - s2a_top_y;
            else
                s2b_y_from_top <= (others => '0');
            end if;

            ----------------------------------------------------------------
            -- STAGE 2c : depth scaling + threshold mux + hash compare.
            ----------------------------------------------------------------
            y_from_top := s2b_y_from_top;
            case lit_depth(9 downto 8) is
                when "00" =>
                    null;
                when "01" =>
                    y_from_top := shift_right(y_from_top, 1);
                when "10" =>
                    y_from_top := shift_right(y_from_top, 2);
                when others =>
                    y_from_top := shift_right(y_from_top, 3);
            end case;

            case to_integer(y_from_top(7 downto 5)) is
                when 0 =>
                    lit_threshold := resize(lit_knob(9 downto 4), 10);
                when 1 =>
                    lit_threshold := resize(lit_knob(9 downto 5), 10);
                when 2 =>
                    lit_threshold := resize(lit_knob(9 downto 6), 10);
                when 3 =>
                    lit_threshold := resize(lit_knob(9 downto 7), 10);
                when others =>
                    lit_threshold := (others => '0');
            end case;

            if (resize(s2b_lit_hash(7 downto 0), 10) < lit_threshold)
               or (resize(s2b_lit_hash(15 downto 8), 10) < lit_threshold)
            then
                s2_is_lit <= '1';
            else
                s2_is_lit <= '0';
            end if;

            s2_in_mass   <= s2b_in_mass;
            s2_zone      <= s2b_zone;
            s2_color_bin <= s2b_color_bin;
            s2_is_border <= s2b_is_border;
            s2_is_invert <= s2b_is_invert;
            -- Per-pixel speckle: very rare white pixels sprinkled on the
            -- mass. Uses the already-hashed lit_hash bits that are unused
            -- by the lightning threshold.
            if s2b_lit_hash(15 downto 9) = "0000000" then
                s2_is_speckle <= '1';
            else
                s2_is_speckle <= '0';
            end if;

            ----------------------------------------------------------------
            -- STAGE 3 : palette lookup + lightning override.
            ----------------------------------------------------------------
            s3_in_mass <= s2_in_mass;

            if palette_cool then
                case s2_zone is
                    when "00" =>
                        case to_integer(s2_color_bin) is
                            when 0 => pal_idx := to_unsigned(2, 3);
                            when 1 => pal_idx := to_unsigned(3, 3);
                            when 2 => pal_idx := to_unsigned(2, 3);
                            when 3 => pal_idx := to_unsigned(4, 3);
                            when 4 => pal_idx := to_unsigned(3, 3);
                            when 5 => pal_idx := to_unsigned(7, 3);
                            when 6 => pal_idx := to_unsigned(6, 3);
                            when others => pal_idx := to_unsigned(7, 3);
                        end case;
                    when "01" =>
                        case to_integer(s2_color_bin) is
                            when 0 => pal_idx := to_unsigned(2, 3);
                            when 1 => pal_idx := to_unsigned(2, 3);
                            when 2 => pal_idx := to_unsigned(4, 3);
                            when 3 => pal_idx := to_unsigned(3, 3);
                            when 4 => pal_idx := to_unsigned(3, 3);
                            when 5 => pal_idx := to_unsigned(7, 3);
                            when 6 => pal_idx := to_unsigned(6, 3);
                            when others => pal_idx := to_unsigned(4, 3);
                        end case;
                    when "10" =>
                        case to_integer(s2_color_bin) is
                            when 0 => pal_idx := to_unsigned(4, 3);
                            when 1 => pal_idx := to_unsigned(2, 3);
                            when 2 => pal_idx := to_unsigned(7, 3);
                            when 3 => pal_idx := to_unsigned(4, 3);
                            when 4 => pal_idx := to_unsigned(3, 3);
                            when 5 => pal_idx := to_unsigned(2, 3);
                            when 6 => pal_idx := to_unsigned(6, 3);
                            when others => pal_idx := to_unsigned(7, 3);
                        end case;
                    when others =>
                        case to_integer(s2_color_bin) is
                            when 0 => pal_idx := to_unsigned(2, 3);
                            when 1 => pal_idx := to_unsigned(4, 3);
                            when 2 => pal_idx := to_unsigned(4, 3);
                            when 3 => pal_idx := to_unsigned(2, 3);
                            when 4 => pal_idx := to_unsigned(3, 3);
                            when 5 => pal_idx := to_unsigned(7, 3);
                            when 6 => pal_idx := to_unsigned(6, 3);
                            when others => pal_idx := to_unsigned(4, 3);
                        end case;
                end case;
            else
                case s2_zone is
                    when "00" =>
                        case to_integer(s2_color_bin) is
                            when 0 => pal_idx := to_unsigned(0, 3);
                            when 1 => pal_idx := to_unsigned(0, 3);
                            when 2 => pal_idx := to_unsigned(1, 3);
                            when 3 => pal_idx := to_unsigned(1, 3);
                            when 4 => pal_idx := to_unsigned(5, 3);
                            when 5 => pal_idx := to_unsigned(3, 3);
                            when 6 => pal_idx := to_unsigned(6, 3);
                            when others => pal_idx := to_unsigned(7, 3);
                        end case;
                    when "01" =>
                        case to_integer(s2_color_bin) is
                            when 0 => pal_idx := to_unsigned(0, 3);
                            when 1 => pal_idx := to_unsigned(3, 3);
                            when 2 => pal_idx := to_unsigned(1, 3);
                            when 3 => pal_idx := to_unsigned(2, 3);
                            when 4 => pal_idx := to_unsigned(3, 3);
                            when 5 => pal_idx := to_unsigned(3, 3);
                            when 6 => pal_idx := to_unsigned(6, 3);
                            when others => pal_idx := to_unsigned(7, 3);
                        end case;
                    when "10" =>
                        case to_integer(s2_color_bin) is
                            when 0 => pal_idx := to_unsigned(2, 3);
                            when 1 => pal_idx := to_unsigned(3, 3);
                            when 2 => pal_idx := to_unsigned(2, 3);
                            when 3 => pal_idx := to_unsigned(0, 3);
                            when 4 => pal_idx := to_unsigned(3, 3);
                            when 5 => pal_idx := to_unsigned(6, 3);
                            when 6 => pal_idx := to_unsigned(4, 3);
                            when others => pal_idx := to_unsigned(7, 3);
                        end case;
                    when others =>
                        case to_integer(s2_color_bin) is
                            when 0 => pal_idx := to_unsigned(2, 3);
                            when 1 => pal_idx := to_unsigned(2, 3);
                            when 2 => pal_idx := to_unsigned(4, 3);
                            when 3 => pal_idx := to_unsigned(3, 3);
                            when 4 => pal_idx := to_unsigned(5, 3);
                            when 5 => pal_idx := to_unsigned(6, 3);
                            when 6 => pal_idx := to_unsigned(4, 3);
                            when others => pal_idx := to_unsigned(7, 3);
                        end case;
                end case;
            end if;

            case to_integer(pal_idx) is
                when 0 =>
                    pal_y_v := to_unsigned(580, 10);
                    pal_u_v := to_unsigned(640, 10);
                    pal_v_v := to_unsigned(880, 10);
                when 1 =>
                    pal_y_v := to_unsigned(350, 10);
                    pal_u_v := to_unsigned(380, 10);
                    pal_v_v := to_unsigned(880, 10);
                when 2 =>
                    pal_y_v := to_unsigned(300, 10);
                    pal_u_v := to_unsigned(820, 10);
                    pal_v_v := to_unsigned(380, 10);
                when 3 =>
                    pal_y_v := to_unsigned(280, 10);
                    pal_u_v := to_unsigned(780, 10);
                    pal_v_v := to_unsigned(660, 10);
                when 4 =>
                    pal_y_v := to_unsigned(440, 10);
                    pal_u_v := to_unsigned(420, 10);
                    pal_v_v := to_unsigned(290, 10);
                when 5 =>
                    pal_y_v := to_unsigned(560, 10);
                    pal_u_v := to_unsigned(260, 10);
                    pal_v_v := to_unsigned(740, 10);
                when 6 =>
                    pal_y_v := to_unsigned(60, 10);
                    pal_u_v := to_unsigned(500, 10);
                    pal_v_v := to_unsigned(520, 10);
                when others =>
                    pal_y_v := to_unsigned(850, 10);
                    pal_u_v := to_unsigned(580, 10);
                    pal_v_v := to_unsigned(440, 10);
            end case;

            -- Override priority (top = highest):
            --   border  → dark seam line between color bands (visible edge)
            --   speckle → rare pure-white pixel (static noise)
            --   lit     → lightning overlay (pure white)
            --   invert  → row-wide colour-inversion glitch
            --   default → plain palette colour
            if s2_is_border = '1' then
                s3_y_val <= to_unsigned(32, 10);
                s3_u_val <= C_WHITE_U;
                s3_v_val <= C_WHITE_V;
            elsif s2_is_speckle = '1' then
                s3_y_val <= C_WHITE_Y;
                s3_u_val <= C_WHITE_U;
                s3_v_val <= C_WHITE_V;
            elsif s2_is_lit = '1' and lit_on then
                s3_y_val <= C_WHITE_Y;
                s3_u_val <= C_WHITE_U;
                s3_v_val <= C_WHITE_V;
            elsif s2_is_invert = '1' then
                s3_y_val <= to_unsigned(1023, 10) - pal_y_v;
                s3_u_val <= to_unsigned(1023, 10) - pal_u_v;
                s3_v_val <= to_unsigned(1023, 10) - pal_v_v;
            else
                s3_y_val <= pal_y_v;
                s3_u_val <= pal_u_v;
                s3_v_val <= pal_v_v;
            end if;

            ----------------------------------------------------------------
            -- STAGE 4 : composite - background outside mass, s3 inside.
            ----------------------------------------------------------------
            if bg_black then
                bg_y_v := to_unsigned(10, 10);
                bg_u_v := C_WHITE_U;
                bg_v_v := C_WHITE_V;
            else
                bg_y_v := C_TEAL_Y;
                bg_u_v := C_TEAL_U;
                bg_v_v := C_TEAL_V;
            end if;

            if s3_in_mass = '0' then
                comp_y := bg_y_v;
                comp_u := bg_u_v;
                comp_v := bg_v_v;
            else
                comp_y := s3_y_val;
                comp_u := s3_u_val;
                comp_v := s3_v_val;
            end if;

            s4_y <= comp_y;
            s4_u <= comp_u;
            s4_v <= comp_v;

            ----------------------------------------------------------------
            -- STAGE 5 : brightness.
            ----------------------------------------------------------------
            y_scaled := s4_y * bright_knob;
            s5_y <= y_scaled(19 downto 10);
            s5_u <= s4_u;
            s5_v <= s4_v;

            ----------------------------------------------------------------
            -- OUTPUT
            ----------------------------------------------------------------
            data_out <= pipe(LATENCY - 1);

            if not bypass_on then
                data_out.y <= std_logic_vector(s5_y);
                data_out.u <= std_logic_vector(s5_u);
                data_out.v <= std_logic_vector(s5_v);
            end if;

        end if;
    end process;

end architecture solitude;
