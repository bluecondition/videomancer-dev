-- Voronoi: 16-seed full-screen Voronoi cellular pattern.
--
-- Sixteen seed points distributed across the whole HD screen — no
-- repeating tile.  Each seed has its own velocity, half positive and
-- half negative, so the cells drift in MIXED directions instead of all
-- pointing the same way.  A small sin-LUT phase per seed gives the
-- drift a bidirectional wobble.
--
-- The Scale slider selects between four pre-computed seed layouts:
--   XS - 16 seeds spread very tight around screen centre (small cells)
--   S  - 16 seeds packed
--   M  - 16 seeds spread to most of the screen
--   L  - 16 seeds spread fully edge-to-edge (largest cells)
-- (Smaller layout = denser cells visible.)
--
-- Per pixel we compute the distance from the pixel to all 16 seeds in
-- parallel, reduce 16 → 8 → 4 → 2 → 1.  Winner drives an 8-entry
-- palette mod 8.  Edge highlight on cell-ID transitions.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Hue Shift
--   registers_in(1) = Brightness
--   registers_in(2) = Motion       (frame-rate motion phase speed)
--   registers_in(3) = Edge Glow
--   registers_in(4) = Saturation
--   registers_in(5) = Wobble Bias  (signed offset on motion)
--   registers_in(6) = Switches (b0 metric, b1 edge, b2 cycle, b3 palette, b4 invert)
--   registers_in(7) = Scale        (slider — selects seed layout/cell size)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture voronoi of program_top is

    constant LATENCY : natural := 16;
    constant N_SEEDS : natural := 12;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    -- Palette
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y_VIB : t_pal := (
        to_unsigned(328, 10), to_unsigned(584, 10),
        to_unsigned(840, 10), to_unsigned(580, 10),
        to_unsigned(164, 10), to_unsigned(192, 10),
        to_unsigned(349, 10), to_unsigned(580, 10));
    constant C_PAL_U_VIB : t_pal := (
        to_unsigned(960, 10), to_unsigned(772, 10),
        to_unsigned(584, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned(608, 10),
        to_unsigned(756, 10), to_unsigned(440, 10));
    constant C_PAL_V_VIB : t_pal := (
        to_unsigned(360, 10), to_unsigned(212, 10),
        to_unsigned( 64, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned(696, 10),
        to_unsigned(852, 10), to_unsigned(216, 10));
    constant C_PAL_Y_PAS : t_pal := (
        to_unsigned(680, 10), to_unsigned(720, 10),
        to_unsigned(900, 10), to_unsigned(740, 10),
        to_unsigned(560, 10), to_unsigned(580, 10),
        to_unsigned(650, 10), to_unsigned(740, 10));
    constant C_PAL_U_PAS : t_pal := (
        to_unsigned(736, 10), to_unsigned(640, 10),
        to_unsigned(548, 10), to_unsigned(324, 10),
        to_unsigned(476, 10), to_unsigned(560, 10),
        to_unsigned(634, 10), to_unsigned(476, 10));
    constant C_PAL_V_PAS : t_pal := (
        to_unsigned(436, 10), to_unsigned(362, 10),
        to_unsigned(288, 10), to_unsigned(364, 10),
        to_unsigned(736, 10), to_unsigned(604, 10),
        to_unsigned(682, 10), to_unsigned(364, 10));

    -- 4 seed layouts × 14 seeds — pre-computed at vsync.
    type t_base is array (0 to N_SEEDS - 1) of unsigned(11 downto 0);
    -- Layout L (largest cells) — full-screen spread
    constant C_BX_L : t_base := (
        to_unsigned( 160, 12), to_unsigned( 640, 12),
        to_unsigned(1120, 12), to_unsigned(1600, 12),
        to_unsigned( 320, 12), to_unsigned( 800, 12),
        to_unsigned(1280, 12), to_unsigned(1760, 12),
        to_unsigned( 480, 12), to_unsigned( 960, 12),
        to_unsigned(1440, 12), to_unsigned( 600, 12));
    constant C_BY_L : t_base := (
        to_unsigned( 140, 12), to_unsigned( 200, 12),
        to_unsigned( 160, 12), to_unsigned( 220, 12),
        to_unsigned( 440, 12), to_unsigned( 480, 12),
        to_unsigned( 460, 12), to_unsigned( 500, 12),
        to_unsigned( 760, 12), to_unsigned( 800, 12),
        to_unsigned( 740, 12), to_unsigned( 900, 12));
    -- (Earlier alternate layouts removed — slider now controls the
    --  active-seed count instead, since only one layout is used.)

    -- Per-seed velocity components.  HALF positive, HALF negative so the
    -- average motion direction is zero — seeds drift bidirectionally.
    type t_vel is array (0 to N_SEEDS - 1) of signed(7 downto 0);
    constant C_VEL_X : t_vel := (
        to_signed( 5, 8), to_signed(-7, 8), to_signed(11, 8), to_signed(-13, 8),
        to_signed(17, 8), to_signed(-19, 8), to_signed(23, 8), to_signed(-29, 8),
        to_signed(-3, 8), to_signed( 9, 8), to_signed(-15, 8), to_signed(21, 8));
    constant C_VEL_Y : t_vel := (
        to_signed( 7, 8), to_signed(11, 8), to_signed(-9, 8), to_signed(13, 8),
        to_signed(-17, 8), to_signed(19, 8), to_signed(-23, 8), to_signed(27, 8),
        to_signed(-5, 8), to_signed(15, 8), to_signed(-25, 8), to_signed(33, 8));

    -- Position
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    -- Params
    signal hue_shift_r  : unsigned(9 downto 0) := (others => '0');
    signal bright_r     : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal motion_r     : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal edge_glow_r  : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal sat_r        : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal wobble_bias_r : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal metric_r     : std_logic := '0';
    signal edge_on_r    : std_logic := '1';
    signal cycle_r      : std_logic := '0';
    signal pastel_r     : std_logic := '0';
    signal invert_r     : std_logic := '0';
    signal scale_r      : unsigned(9 downto 0) := to_unsigned(768, 10);

    signal cycle_off_r  : unsigned(2 downto 0) := (others => '0');

    -- Signed drift accumulators per seed (12-bit signed wrap).
    type t_drift is array (0 to N_SEEDS - 1) of signed(11 downto 0);
    signal drift_x_r : t_drift := (others => (others => '0'));
    signal drift_y_r : t_drift := (others => (others => '0'));

    -- Live seed positions (= base + drift).  Stored as signed 13-bit.
    type t_seed_pos is array (0 to N_SEEDS - 1) of signed(12 downto 0);
    signal seed_x_r : t_seed_pos := (others => (others => '0'));
    signal seed_y_r : t_seed_pos := (others => (others => '0'));

    -- Active seed count (latched per vsync from Scale slider).
    signal active_n_r : unsigned(3 downto 0) := to_unsigned(12, 4);
    -- Tile-modulo: how many pixel-coord bits we strip when applying mod.
    -- Higher slider → fewer bits kept → more tiles repeating across the
    -- screen → many more visible cells (with deliberate repetition).
    --   00 → 11 bits (no tile), 01 → 10 bits, 10 → 9 bits (4× tile),
    --   11 → 8 bits (8× tile, lots of small cells).
    signal tile_mask_r : unsigned(11 downto 0) := (others => '1');

    -- Seed-update FSM
    signal upd_idx     : unsigned(4 downto 0) := (others => '0');
    signal upd_active  : std_logic := '0';

    -- ------------------------------------------------------------------
    -- Per-pixel pipeline
    -- ------------------------------------------------------------------
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    type t_arr_s14 is array (0 to N_SEEDS - 1) of signed(13 downto 0);
    signal s1_dx : t_arr_s14;
    signal s0_y_p : unsigned(11 downto 0);

    signal s2_dx : t_arr_s14;
    signal s2_dy : t_arr_s14;

    type t_arr_u12 is array (0 to N_SEEDS - 1) of unsigned(11 downto 0);
    signal s3_adx : t_arr_u12;
    signal s3_ady : t_arr_u12;

    type t_arr_u13 is array (0 to N_SEEDS - 1) of unsigned(12 downto 0);
    signal s4_dist : t_arr_u13;

    -- S5: 12 → 6 pair compare.
    type t_arr6_u13 is array (0 to 5) of unsigned(12 downto 0);
    type t_arr6_u4  is array (0 to 5) of unsigned(3 downto 0);
    signal s5_d : t_arr6_u13;
    signal s5_i : t_arr6_u4;

    -- S6: 6 → 3.
    type t_arr3_u13 is array (0 to 2) of unsigned(12 downto 0);
    type t_arr3_u4  is array (0 to 2) of unsigned(3 downto 0);
    signal s6_d : t_arr3_u13;
    signal s6_i : t_arr3_u4;

    -- S7: 3 → 2 (fold first pair, carry third).
    type t_arr2_u13 is array (0 to 1) of unsigned(12 downto 0);
    type t_arr2_u4  is array (0 to 1) of unsigned(3 downto 0);
    signal s7_d : t_arr2_u13;
    signal s7_i : t_arr2_u4;

    -- S8: 2 → 1.
    signal s8_d : unsigned(12 downto 0);
    signal s8_i : unsigned(3 downto 0);

    -- S9: ID history for edge detection.
    signal s9_i     : unsigned(3 downto 0);
    signal s9_id_d1 : unsigned(3 downto 0) := (others => '0');
    signal s9_id_d2 : unsigned(3 downto 0) := (others => '0');

    -- S10: palette + edge.
    signal s10_y    : unsigned(9 downto 0);
    signal s10_u    : unsigned(9 downto 0);
    signal s10_v    : unsigned(9 downto 0);
    signal s10_edge : std_logic;

    -- S11: brightness mul.
    signal s11_y_mul : unsigned(19 downto 0);
    signal s11_u     : unsigned(9 downto 0);
    signal s11_v     : unsigned(9 downto 0);
    signal s11_edge  : std_logic;

    -- S12: sat mul.
    signal s12_y     : unsigned(9 downto 0);
    signal s12_u_mul : signed(22 downto 0);
    signal s12_v_mul : signed(22 downto 0);
    signal s12_edge  : std_logic;

    -- S13: re-center + edge overlay.
    signal s13_y : unsigned(9 downto 0);
    signal s13_u : unsigned(9 downto 0);
    signal s13_v : unsigned(9 downto 0);

    -- S14: final invert.
    signal s14_y : unsigned(9 downto 0);
    signal s14_u : unsigned(9 downto 0);
    signal s14_v : unsigned(9 downto 0);

    function abs_s14(v : signed(13 downto 0)) return unsigned is
        variable t : signed(13 downto 0);
    begin
        if v(13) = '1' then t := -v; else t := v; end if;
        return unsigned(std_logic_vector(t(11 downto 0)));
    end function;

    function sat10_s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

begin

    -- ========================================================================
    -- Position + param latch + signed drift advance + layout select
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge   : std_logic;
        variable v_v_edge   : std_logic;
        variable v_motion_b : unsigned(6 downto 0);
        variable v_prod_x   : signed(15 downto 0);
        variable v_prod_y   : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                hue_shift_r   <= unsigned(registers_in(0));
                bright_r      <= unsigned(registers_in(1));
                motion_r      <= unsigned(registers_in(2));
                edge_glow_r   <= unsigned(registers_in(3));
                sat_r         <= unsigned(registers_in(4));
                wobble_bias_r <= unsigned(registers_in(5));
                metric_r      <= registers_in(6)(0);
                edge_on_r     <= registers_in(6)(1);
                cycle_r       <= registers_in(6)(2);
                pastel_r      <= registers_in(6)(3);
                invert_r      <= registers_in(6)(4);
                scale_r       <= unsigned(registers_in(7));

                -- Scale slider: knob LOW = full screen, few cells.  Knob
                -- HIGH = pixel coordinates wrap modulo a smaller tile so
                -- the 12-seed pattern repeats across the screen, giving
                -- many more visible cells.
                case registers_in(7)(9 downto 8) is
                    when "00"   =>
                        active_n_r  <= to_unsigned( 8, 4);
                        tile_mask_r <= "111111111111";  -- no mask
                    when "01"   =>
                        active_n_r  <= to_unsigned(12, 4);
                        tile_mask_r <= "111111111111";  -- no mask
                    when "10"   =>
                        active_n_r  <= to_unsigned(12, 4);
                        tile_mask_r <= "001111111111";  -- 1024 px tile → 2× rep
                    when others =>
                        active_n_r  <= to_unsigned(12, 4);
                        tile_mask_r <= "000111111111";  -- 512 px tile → 4× rep
                end case;

                if registers_in(6)(2) = '1' then
                    cycle_off_r <= cycle_off_r + 1;
                end if;

                -- Drift advance: signed motion knob × signed velocity.  Top
                -- 4 bits of motion (motion knob /64 → 1/40-ish of original
                -- range) keep wobble modest, so 100% knob equates to the
                -- previous ~2.5% knob.
                v_motion_b := "000" & unsigned(registers_in(2)(9 downto 6));
                for i in 0 to N_SEEDS - 1 loop
                    v_prod_x := signed(resize(v_motion_b, 8)) * C_VEL_X(i);
                    v_prod_y := signed(resize(v_motion_b, 8)) * C_VEL_Y(i);
                    drift_x_r(i) <= drift_x_r(i) +
                                    resize(v_prod_x(11 downto 0), 12);
                    drift_y_r(i) <= drift_y_r(i) +
                                    resize(v_prod_y(11 downto 0), 12);
                end loop;

                upd_idx <= (others => '0');
                upd_active <= '1';
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;

            -- Seed update FSM (one seed/cycle, during/after vblank).
            -- Single layout (L = full-screen spread).
            if upd_active = '1' then
                seed_x_r(to_integer(upd_idx)) <=
                    signed(resize(C_BX_L(to_integer(upd_idx)), 13)) +
                    resize(drift_x_r(to_integer(upd_idx)), 13);
                seed_y_r(to_integer(upd_idx)) <=
                    signed(resize(C_BY_L(to_integer(upd_idx)), 13)) +
                    resize(drift_y_r(to_integer(upd_idx)), 13);
                if upd_idx = to_unsigned(N_SEEDS - 1, 5) then
                    upd_active <= '0';
                else
                    upd_idx <= upd_idx + 1;
                end if;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pidx_int : integer range 0 to 7;
        variable v_pidx     : unsigned(2 downto 0);
        variable v_u_c      : signed(11 downto 0);
        variable v_v_c      : signed(11 downto 0);
        variable v_u_re     : signed(13 downto 0);
        variable v_v_re     : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: per-seed dx.  Inactive seeds (beyond active_n_r) are
            -- forced to a large value so they never win the min reduction.
            -- Pixel + seed coords are masked to tile_mask_r so the pattern
            -- repeats across the screen at high Scale settings.
            for i in 0 to N_SEEDS - 1 loop
                if to_unsigned(i, 4) < active_n_r then
                    s1_dx(i) <= signed(resize(s0_x and tile_mask_r, 14)) -
                                signed(resize(
                                    unsigned(std_logic_vector(seed_x_r(i)))
                                    and ("0" & tile_mask_r), 14));
                else
                    s1_dx(i) <= to_signed(8191, 14);
                end if;
            end loop;
            s0_y_p <= s0_y;

            -- S2: per-seed dy + carry dx.
            for i in 0 to N_SEEDS - 1 loop
                s2_dx(i) <= s1_dx(i);
                if to_unsigned(i, 4) < active_n_r then
                    s2_dy(i) <= signed(resize(s0_y_p and tile_mask_r, 14)) -
                                signed(resize(
                                    unsigned(std_logic_vector(seed_y_r(i)))
                                    and ("0" & tile_mask_r), 14));
                else
                    s2_dy(i) <= to_signed(8191, 14);
                end if;
            end loop;

            -- S3: abs.
            for i in 0 to N_SEEDS - 1 loop
                s3_adx(i) <= abs_s14(s2_dx(i));
                s3_ady(i) <= abs_s14(s2_dy(i));
            end loop;

            -- S4: metric.
            for i in 0 to N_SEEDS - 1 loop
                if metric_r = '0' then
                    if s3_adx(i) > s3_ady(i) then
                        s4_dist(i) <= '0' & s3_adx(i);
                    else
                        s4_dist(i) <= '0' & s3_ady(i);
                    end if;
                else
                    s4_dist(i) <= ('0' & s3_adx(i)) + ('0' & s3_ady(i));
                end if;
            end loop;

            -- S5: pair compare 12 → 6.
            for p in 0 to 5 loop
                if s4_dist(2 * p) <= s4_dist(2 * p + 1) then
                    s5_d(p) <= s4_dist(2 * p);
                    s5_i(p) <= to_unsigned(2 * p, 4);
                else
                    s5_d(p) <= s4_dist(2 * p + 1);
                    s5_i(p) <= to_unsigned(2 * p + 1, 4);
                end if;
            end loop;

            -- S6: 6 → 3.
            for p in 0 to 2 loop
                if s5_d(2 * p) <= s5_d(2 * p + 1) then
                    s6_d(p) <= s5_d(2 * p);
                    s6_i(p) <= s5_i(2 * p);
                else
                    s6_d(p) <= s5_d(2 * p + 1);
                    s6_i(p) <= s5_i(2 * p + 1);
                end if;
            end loop;

            -- S7: 3 → 2 (fold first pair, carry third).
            if s6_d(0) <= s6_d(1) then
                s7_d(0) <= s6_d(0); s7_i(0) <= s6_i(0);
            else
                s7_d(0) <= s6_d(1); s7_i(0) <= s6_i(1);
            end if;
            s7_d(1) <= s6_d(2);
            s7_i(1) <= s6_i(2);

            -- S8: final 2 → 1.
            if s7_d(0) <= s7_d(1) then
                s8_d <= s7_d(0); s8_i <= s7_i(0);
            else
                s8_d <= s7_d(1); s8_i <= s7_i(1);
            end if;

            -- S9: ID history.
            s9_i     <= s8_i;
            s9_id_d1 <= s9_i;
            s9_id_d2 <= s9_id_d1;

            -- S10: palette (use low 3 bits of winner index mod 8).
            v_pidx := s9_i(2 downto 0) + cycle_off_r + hue_shift_r(9 downto 7);
            v_pidx_int := to_integer(v_pidx);
            if pastel_r = '1' then
                s10_y <= C_PAL_Y_PAS(v_pidx_int);
                s10_u <= C_PAL_U_PAS(v_pidx_int);
                s10_v <= C_PAL_V_PAS(v_pidx_int);
            else
                s10_y <= C_PAL_Y_VIB(v_pidx_int);
                s10_u <= C_PAL_U_VIB(v_pidx_int);
                s10_v <= C_PAL_V_VIB(v_pidx_int);
            end if;
            if edge_on_r = '1' and
               (s9_i /= s9_id_d1 or
                (edge_glow_r(9) = '1' and s9_id_d1 /= s9_id_d2)) then
                s10_edge <= '1';
            else
                s10_edge <= '0';
            end if;

            -- S11: brightness multiply.
            s11_y_mul <= s10_y * bright_r;
            s11_u     <= s10_u;
            s11_v     <= s10_v;
            s11_edge  <= s10_edge;

            -- S12: sat mul.
            s12_y     <= s11_y_mul(19 downto 10);
            v_u_c     := signed('0' & std_logic_vector(s11_u)) -
                         to_signed(512, 12);
            v_v_c     := signed('0' & std_logic_vector(s11_v)) -
                         to_signed(512, 12);
            s12_u_mul <= v_u_c * signed('0' & std_logic_vector(sat_r));
            s12_v_mul <= v_v_c * signed('0' & std_logic_vector(sat_r));
            s12_edge  <= s11_edge;

            -- S13: re-center + edge overlay.
            if s12_edge = '1' then
                s13_y <= to_unsigned(1023, 10);
                s13_u <= to_unsigned(512, 10);
                s13_v <= to_unsigned(512, 10);
            else
                s13_y <= s12_y;
                v_u_re := resize(shift_right(s12_u_mul, 10), 14) +
                          to_signed(512, 14);
                v_v_re := resize(shift_right(s12_v_mul, 10), 14) +
                          to_signed(512, 14);
                s13_u <= sat10_s(v_u_re);
                s13_v <= sat10_s(v_v_re);
            end if;

            -- S14: invert.
            if invert_r = '1' then
                s14_y <= to_unsigned(1023, 10) - s13_y;
            else
                s14_y <= s13_y;
            end if;
            s14_u <= s13_u;
            s14_v <= s13_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s14_y);
    data_out.u       <= std_logic_vector(s14_u);
    data_out.v       <= std_logic_vector(s14_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture voronoi;
