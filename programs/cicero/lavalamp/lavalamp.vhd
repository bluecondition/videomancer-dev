-- Lava Lamp: four floating metaball blobs.
--
-- Four blob centres drift through the frame on independent sinusoidal
-- paths.  Each pixel computes Manhattan distance to each blob, treats
-- max(0, blob_size - dist) as that blob's contribution, sums the four
-- contributions and thresholds the result to draw the metaball surface.
-- No multipliers in the per-pixel path — only abs + add chains.
--
-- Register map:
--   registers_in(0) = Blob Size     (rotary 1)
--   registers_in(1) = Drift Speed   (rotary 2 — blob motion rate)
--   registers_in(2) = Drift Range   (rotary 3 — motion amplitude)
--   registers_in(3) = Threshold     (rotary 4 — field cutoff)
--   registers_in(4) = Hue Rotate    (rotary 5)
--   registers_in(5) = Brightness    (rotary 6)
--   registers_in(6) = Switches:
--     b0   Edge     (0=Soft / 1=Hard)
--     b1   Palette  (0=Mono / 1=Plasma)
--     b2   Tracers  (0=Off / 1=Sparkle)
--     b3   Center   (0=Random / 1=Centered)
--     b4   Invert
--   registers_in(7) = Smoothness (slider, KEY — blend gain on field)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture lavalamp of program_top is

    constant LATENCY : natural := 10;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal blobsize_r  : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal speed_r     : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal range_r     : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal thresh_r    : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal hue_r       : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bright_r    : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal smooth_r    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal edge_r      : std_logic := '0';
    signal palette_r   : std_logic := '0';
    signal tracer_r    : std_logic := '0';
    signal center_r    : std_logic := '0';
    signal invert_r    : std_logic := '0';

    signal anim_phase_r : unsigned(11 downto 0) := (others => '0');

    -- LFSR for tracer sparkle
    signal lfsr_r : unsigned(15 downto 0) := to_unsigned(28371, 16);

    -- 4 blob positions (signed centred, range -540..+540 roughly)
    type t_pos4 is array (0 to 3) of signed(12 downto 0);
    signal blob_x_r : t_pos4 := (others => to_signed(0, 13));
    signal blob_y_r : t_pos4 := (others => to_signed(0, 13));

    -- Update FSM (serial, one blob per cycle, 3 pipeline stages each)
    signal upd_idx_r   : unsigned(1 downto 0) := (others => '0');
    signal upd_phase_r : unsigned(1 downto 0) := (others => '0');
    -- Stage A intermediate: anim_mul
    signal upd_mul_x_r : unsigned(15 downto 0) := (others => '0');
    signal upd_mul_y_r : unsigned(15 downto 0) := (others => '0');
    signal upd_idx_a_r : unsigned(1 downto 0) := (others => '0');
    -- Stage B intermediate: angle + sin
    signal upd_sin_x_r : signed(8 downto 0) := (others => '0');
    signal upd_sin_y_r : signed(8 downto 0) := (others => '0');
    signal upd_idx_b_r : unsigned(1 downto 0) := (others => '0');

    -- Phase offsets for each blob (different phases give independent paths)
    type t_phase4 is array (0 to 3) of unsigned(11 downto 0);
    constant C_PHASE_OFF_X : t_phase4 := (
        to_unsigned(0,    12), to_unsigned(1024, 12),
        to_unsigned(2048, 12), to_unsigned(3072, 12));
    constant C_PHASE_OFF_Y : t_phase4 := (
        to_unsigned(512,  12), to_unsigned(1536, 12),
        to_unsigned(2560, 12), to_unsigned(3584, 12));
    -- Per-blob speed multipliers (small, makes paths different)
    type t_speed4 is array (0 to 3) of unsigned(3 downto 0);
    constant C_SPEED_X : t_speed4 := (to_unsigned(3, 4), to_unsigned(5, 4),
                                       to_unsigned(2, 4), to_unsigned(4, 4));
    constant C_SPEED_Y : t_speed4 := (to_unsigned(4, 4), to_unsigned(3, 4),
                                       to_unsigned(5, 4), to_unsigned(2, 4));

    -- 64-entry quarter sine LUT 0..127
    type t_quad is array (0 to 63) of unsigned(7 downto 0);
    constant C_SIN_QUAD : t_quad := (
        to_unsigned(  0, 8), to_unsigned(  3, 8), to_unsigned(  6, 8), to_unsigned( 10, 8),
        to_unsigned( 13, 8), to_unsigned( 16, 8), to_unsigned( 19, 8), to_unsigned( 22, 8),
        to_unsigned( 25, 8), to_unsigned( 28, 8), to_unsigned( 31, 8), to_unsigned( 35, 8),
        to_unsigned( 38, 8), to_unsigned( 41, 8), to_unsigned( 44, 8), to_unsigned( 47, 8),
        to_unsigned( 49, 8), to_unsigned( 52, 8), to_unsigned( 55, 8), to_unsigned( 58, 8),
        to_unsigned( 61, 8), to_unsigned( 63, 8), to_unsigned( 66, 8), to_unsigned( 69, 8),
        to_unsigned( 71, 8), to_unsigned( 74, 8), to_unsigned( 76, 8), to_unsigned( 79, 8),
        to_unsigned( 81, 8), to_unsigned( 83, 8), to_unsigned( 86, 8), to_unsigned( 88, 8),
        to_unsigned( 90, 8), to_unsigned( 92, 8), to_unsigned( 94, 8), to_unsigned( 96, 8),
        to_unsigned( 98, 8), to_unsigned(100, 8), to_unsigned(102, 8), to_unsigned(104, 8),
        to_unsigned(106, 8), to_unsigned(107, 8), to_unsigned(109, 8), to_unsigned(110, 8),
        to_unsigned(112, 8), to_unsigned(113, 8), to_unsigned(114, 8), to_unsigned(116, 8),
        to_unsigned(117, 8), to_unsigned(118, 8), to_unsigned(119, 8), to_unsigned(120, 8),
        to_unsigned(121, 8), to_unsigned(122, 8), to_unsigned(123, 8), to_unsigned(124, 8),
        to_unsigned(124, 8), to_unsigned(125, 8), to_unsigned(125, 8), to_unsigned(126, 8),
        to_unsigned(126, 8), to_unsigned(127, 8), to_unsigned(127, 8), to_unsigned(127, 8));

    -- 8-color palette
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (
        to_unsigned(580, 10), to_unsigned(620, 10),
        to_unsigned(840, 10), to_unsigned(700, 10),
        to_unsigned(540, 10), to_unsigned(420, 10),
        to_unsigned(380, 10), to_unsigned(680, 10));
    constant C_PAL_U : t_pal := (
        to_unsigned(360, 10), to_unsigned(440, 10),
        to_unsigned(280, 10), to_unsigned(640, 10),
        to_unsigned(840, 10), to_unsigned(720, 10),
        to_unsigned(620, 10), to_unsigned(420, 10));
    constant C_PAL_V : t_pal := (
        to_unsigned(820, 10), to_unsigned(720, 10),
        to_unsigned(380, 10), to_unsigned(320, 10),
        to_unsigned(420, 10), to_unsigned(620, 10),
        to_unsigned(780, 10), to_unsigned(680, 10));

    -- Pipeline signals
    signal s0_x : signed(12 downto 0);
    signal s0_y : signed(12 downto 0);

    -- S1: 4 dx, 4 dy
    type t_dx4 is array (0 to 3) of signed(12 downto 0);
    signal s1_dx : t_dx4;
    signal s1_dy : t_dx4;

    -- S2: 4 manhattan distances
    type t_d4 is array (0 to 3) of unsigned(12 downto 0);
    signal s2_dist : t_d4;

    -- S3: 4 contributions (max(0, blob_size - dist))
    type t_c4 is array (0 to 3) of unsigned(10 downto 0);
    signal s3_contrib : t_c4;

    -- S4: pairwise sum 4 → 2
    signal s4_sum01 : unsigned(11 downto 0);
    signal s4_sum23 : unsigned(11 downto 0);

    -- S5: total field
    signal s5_field : unsigned(12 downto 0);

    -- S6: compare to threshold
    signal s6_on    : std_logic;
    signal s6_field : unsigned(12 downto 0);

    -- S7: palette + brightness
    signal s7_y : unsigned(9 downto 0);
    signal s7_u : unsigned(9 downto 0);
    signal s7_v : unsigned(9 downto 0);

    -- S8: bright_r mul (raw)
    signal s8_ymul : unsigned(19 downto 0);
    signal s8_u    : unsigned(9 downto 0);
    signal s8_v    : unsigned(9 downto 0);

    -- S9: extract + invert
    signal s9_y : unsigned(9 downto 0);
    signal s9_u : unsigned(9 downto 0);
    signal s9_v : unsigned(9 downto 0);

    function quad_sin(ang : unsigned(9 downto 0)) return signed is
        variable v_q : unsigned(1 downto 0);
        variable v_idx : integer range 0 to 63;
        variable v_mag : unsigned(7 downto 0);
        variable v_s   : signed(8 downto 0);
    begin
        v_q := ang(9 downto 8);
        v_idx := to_integer(ang(7 downto 2));
        case v_q is
            when "00"   => v_mag := C_SIN_QUAD(v_idx);
            when "01"   => v_mag := C_SIN_QUAD(63 - v_idx);
            when "10"   => v_mag := C_SIN_QUAD(v_idx);
            when others => v_mag := C_SIN_QUAD(63 - v_idx);
        end case;
        if v_q(1) = '0' then
            v_s := signed(resize(v_mag, 9));
        else
            v_s := -signed(resize(v_mag, 9));
        end if;
        return v_s;
    end function;

begin

    -- ========================================================================
    -- Position + parameters + blob motion update at vsync
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_speed : unsigned(9 downto 0);
        variable v_tap : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                v_h_edge := '1';
            end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                v_v_edge := '1';
            end if;

            v_tap := lfsr_r(15) xor lfsr_r(13) xor lfsr_r(12) xor lfsr_r(10);
            lfsr_r <= lfsr_r(14 downto 0) & v_tap;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                blobsize_r <= unsigned(registers_in(0));
                speed_r    <= unsigned(registers_in(1));
                range_r    <= unsigned(registers_in(2));
                thresh_r   <= unsigned(registers_in(3));
                hue_r      <= unsigned(registers_in(4));
                bright_r   <= unsigned(registers_in(5));
                edge_r     <= registers_in(6)(0);
                palette_r  <= registers_in(6)(1);
                tracer_r   <= registers_in(6)(2);
                center_r   <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                smooth_r   <= unsigned(registers_in(7));

                v_speed := "00000" & unsigned(registers_in(1)(9 downto 5));
                anim_phase_r <= anim_phase_r + ("00" & v_speed);

            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Blob update serial FSM (3-stage pipeline, one blob index per cycle).
    -- Runs continuously every clock; result is stable per-frame because
    -- anim_phase_r only changes at vsync.
    -- ========================================================================
    p_update : process(clk)
        variable v_ang_x : unsigned(11 downto 0);
        variable v_ang_y : unsigned(11 downto 0);
        variable v_offset_x : signed(20 downto 0);
        variable v_offset_y : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            -- Stage A: anim_phase * speed for current blob
            upd_mul_x_r <= anim_phase_r * C_SPEED_X(to_integer(upd_idx_r));
            upd_mul_y_r <= anim_phase_r * C_SPEED_Y(to_integer(upd_idx_r));
            upd_idx_a_r <= upd_idx_r;

            -- Stage B: angle + sin lookup
            v_ang_x := C_PHASE_OFF_X(to_integer(upd_idx_a_r)) +
                       upd_mul_x_r(15 downto 4);
            v_ang_y := C_PHASE_OFF_Y(to_integer(upd_idx_a_r)) +
                       upd_mul_y_r(15 downto 4);
            upd_sin_x_r <= quad_sin(v_ang_x(9 downto 0));
            upd_sin_y_r <= quad_sin(v_ang_y(9 downto 0));
            upd_idx_b_r <= upd_idx_a_r;

            -- Stage C: multiply by range, write blob position
            v_offset_x := resize(upd_sin_x_r *
                          signed('0' & std_logic_vector(range_r)), 21);
            v_offset_y := resize(upd_sin_y_r *
                          signed('0' & std_logic_vector(range_r)), 21);
            blob_x_r(to_integer(upd_idx_b_r)) <=
                resize(shift_right(v_offset_x, 7), 13);
            blob_y_r(to_integer(upd_idx_b_r)) <=
                resize(shift_right(v_offset_y, 8), 13);

            -- Advance index (cycle 0..3 repeatedly)
            upd_idx_r <= upd_idx_r + 1;
        end if;
    end process p_update;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_dx : signed(12 downto 0);
        variable v_dy : signed(12 downto 0);
        variable v_dxa : signed(12 downto 0);
        variable v_dya : signed(12 downto 0);
        variable v_dist : unsigned(12 downto 0);
        variable v_size : unsigned(11 downto 0);
        variable v_pidx : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: centered coords
            s0_x <= signed(resize(pixel_x, 13)) - to_signed(960, 13);
            s0_y <= signed(resize(pixel_y, 13)) - to_signed(540, 13);

            -- S1: dx, dy to each of 4 blobs
            for i in 0 to 3 loop
                s1_dx(i) <= s0_x - blob_x_r(i);
                s1_dy(i) <= s0_y - blob_y_r(i);
            end loop;

            -- S2: manhattan distance
            for i in 0 to 3 loop
                v_dxa := s1_dx(i);
                if v_dxa < 0 then v_dxa := -v_dxa; end if;
                v_dya := s1_dy(i);
                if v_dya < 0 then v_dya := -v_dya; end if;
                s2_dist(i) <= resize(unsigned(std_logic_vector(v_dxa(11 downto 0))), 13) +
                              resize(unsigned(std_logic_vector(v_dya(11 downto 0))), 13);
            end loop;

            -- S3: contribution = max(0, blob_size_eff - dist)
            -- blob_size knob 0..1023 → blob radius 0..1023 pixels manhattan
            v_size := resize(blobsize_r, 12);
            for i in 0 to 3 loop
                if s2_dist(i) < resize(v_size, 13) then
                    s3_contrib(i) <= resize(resize(v_size, 13) - s2_dist(i), 11);
                else
                    s3_contrib(i) <= (others => '0');
                end if;
            end loop;

            -- S4: pairwise sum
            s4_sum01 <= resize(s3_contrib(0), 12) + resize(s3_contrib(1), 12);
            s4_sum23 <= resize(s3_contrib(2), 12) + resize(s3_contrib(3), 12);

            -- S5: total field
            s5_field <= resize(s4_sum01, 13) + resize(s4_sum23, 13);

            -- S6: threshold compare with soft edge from smooth_r
            -- on-surface if field >= threshold
            if s5_field >= resize(thresh_r, 13) then
                s6_on <= '1';
            else
                s6_on <= '0';
            end if;
            s6_field <= s5_field;

            -- S7: assign color
            if s6_on = '1' then
                if palette_r = '0' then
                    s7_y <= to_unsigned(900, 10);
                    s7_u <= to_unsigned(380, 10) +
                            resize(hue_r(9 downto 3), 10);
                    s7_v <= to_unsigned(700, 10) -
                            resize(hue_r(9 downto 4), 10);
                else
                    -- Plasma: palette index from field magnitude
                    v_pidx := to_integer(s6_field(8 downto 6) +
                                          hue_r(9 downto 7));
                    s7_y <= C_PAL_Y(v_pidx);
                    s7_u <= C_PAL_U(v_pidx);
                    s7_v <= C_PAL_V(v_pidx);
                end if;
            else
                -- Background: dim gradient (field/8 to give soft halo)
                s7_y <= "0" & s6_field(11 downto 3);
                s7_u <= to_unsigned(512, 10);
                s7_v <= to_unsigned(512, 10);
            end if;

            -- S8: bright mul + tracer sparkle
            s8_ymul <= s7_y * bright_r;
            s8_u    <= s7_u;
            s8_v    <= s7_v;

            -- S9: extract + invert
            if tracer_r = '1' and lfsr_r(0) = '1' and lfsr_r(7) = '1' then
                s9_y <= to_unsigned(1023, 10);
            elsif invert_r = '1' then
                s9_y <= to_unsigned(1023, 10) - s8_ymul(19 downto 10);
            else
                s9_y <= s8_ymul(19 downto 10);
            end if;
            s9_u <= s8_u;
            s9_v <= s8_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s9_y);
    data_out.u       <= std_logic_vector(s9_u);
    data_out.v       <= std_logic_vector(s9_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture lavalamp;
