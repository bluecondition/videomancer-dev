-- Lightning: static lightning-tree branching down from top of screen.
--
-- A central trunk runs vertically from the top of the screen down to the
-- middle.  Six branches fork off at three fork-Y positions (top-third,
-- middle-third, lower-third), alternating left and right.  Each branch
-- has a fixed slope and a fixed end-Y so it stays inside the screen
-- bounds.  Optional LFSR jitter per scanline adds a "crackle" wobble to
-- the otherwise straight paths.
--
-- The tree is essentially static: branches' start/end positions are
-- constants chosen so the tree always fills the screen evenly without
-- clipping.  The Animate toggle either holds the LFSR state (truly
-- static) or re-rolls it each frame (lightning shimmer).
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Branches    (0..6 active branches)
--   registers_in(1) = Jitter      (per-scanline horizontal wobble)
--   registers_in(2) = Glow Width  (line thickness)
--   registers_in(3) = Brightness
--   registers_in(4) = Branch Angle (reserved; unused in this revision)
--   registers_in(5) = Trunk Bias  (horizontal offset of central trunk)
--   registers_in(6) = Switches:
--     b0 Background (0=Dark / 1=Storm cloud noise)
--     b1 Core       (0=Soft / 1=Sharp white core)
--     b2 Animate    (0=Static / 1=Per-frame shimmer)
--     b3 Tint       (0=White / 1=Blue)
--     b4 Invert
--   registers_in(7) = Tree Spread (slider — horizontal branch reach)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture lightning of program_top is

    constant LATENCY  : natural := 14;
    constant N_BOLTS  : natural := 7;  -- trunk + 6 branches

    -- Per-bolt path definition.  Each bolt has start_y, start_x_offset
    -- (relative to centre 960), slope (signed pixels per scanline), and
    -- end_y.  Slope×(end-start) tells us the branch end_x_offset.
    type t_int_arr is array (0 to N_BOLTS - 1) of integer;
    -- Trunk: vertical at centre, full screen, no slope.
    -- Six branches, alternating L/R at three fork heights, all ending
    -- comfortably within the screen.
    constant C_START_Y    : t_int_arr :=
        (   0, 160, 160, 380, 380, 600, 600);
    constant C_END_Y      : t_int_arr :=
        (1080, 540, 540, 760, 760, 980, 980);
    -- start_x_offset is 0 for all branches at fork moment (they start at
    -- the trunk's centre x).  The trunk also has 0 offset.
    -- Slope: signed × pixels per scanline.  Branches alternate −/+, with
    -- the upper pair having steepest slopes (longest reach), going
    -- progressively gentler.
    -- Slope expressed × 16 (fixed-point) for fractional resolution:
    --   −16 = −1.0 px/line, +24 = +1.5 px/line, etc.
    constant C_SLOPE_X16  : t_int_arr :=
        (   0, -28,  28, -22,  22, -16,  16);

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal branches_r   : unsigned(2 downto 0) := to_unsigned(6, 3);
    signal jitter_r     : unsigned(9 downto 0) := to_unsigned(96, 10);
    signal glow_r       : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bright_r     : unsigned(9 downto 0) := to_unsigned(900, 10);
    signal trunk_bias_r : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal storm_r      : std_logic := '0';
    signal sharp_r      : std_logic := '1';
    signal animate_r    : std_logic := '0';
    signal blue_r       : std_logic := '0';
    signal invert_r     : std_logic := '0';
    signal spread_r     : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Bolt position at the CURRENT scanline (updated at each hsync).
    type t_x is array (0 to N_BOLTS - 1) of signed(13 downto 0);
    signal bolt_x_r     : t_x := (others => to_signed(960, 14));
    -- Active flag: per scanline, set if the current pixel_y is within the
    -- bolt's [start_y, end_y) range and the bolt is enabled by branches_r.
    signal bolt_act_r   : std_logic_vector(N_BOLTS - 1 downto 0) :=
                              (others => '0');

    -- Per-bolt fractional slope accumulator (×16 fixed point).
    signal bolt_acc_r : t_x := (others => (others => '0'));

    -- LFSR for jitter (one shared, sampled per bolt with offsets).
    signal lfsr_r : std_logic_vector(15 downto 0) := x"BEEF";

    -- Per-bolt registered jitter (computed every cycle, applied at hsync).
    type t_jit is array (0 to N_BOLTS - 1) of signed(7 downto 0);
    signal bolt_jit_r : t_jit := (others => (others => '0'));

    -- Palettes
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PY_WHITE : t_pal := (others => to_unsigned(960, 10));
    constant C_PU_WHITE : t_pal := (others => to_unsigned(512, 10));
    constant C_PV_WHITE : t_pal := (others => to_unsigned(512, 10));
    constant C_PY_BLUE  : t_pal := (others => to_unsigned(720, 10));
    constant C_PU_BLUE  : t_pal := (others => to_unsigned(820, 10));
    constant C_PV_BLUE  : t_pal := (others => to_unsigned(380, 10));

    -- Per-pixel pipeline
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    type t_arr_s14 is array (0 to N_BOLTS - 1) of signed(13 downto 0);
    signal s1_dx : t_arr_s14;

    type t_arr_u12 is array (0 to N_BOLTS - 1) of unsigned(11 downto 0);
    signal s2_adx : t_arr_u12;

    -- 7-way min reduction: 7 → 4 (pair pad with dummy) → 2 → 1.
    type t_arr4_u12 is array (0 to 3) of unsigned(11 downto 0);
    type t_arr4_u3  is array (0 to 3) of unsigned(2 downto 0);
    signal s3_d : t_arr4_u12;
    signal s3_i : t_arr4_u3;

    type t_arr2_u12 is array (0 to 1) of unsigned(11 downto 0);
    type t_arr2_u3  is array (0 to 1) of unsigned(2 downto 0);
    signal s4_d : t_arr2_u12;
    signal s4_i : t_arr2_u3;

    signal s5_d : unsigned(11 downto 0);
    signal s5_i : unsigned(2 downto 0);

    signal s6_int : unsigned(9 downto 0);
    signal s6_i   : unsigned(2 downto 0);
    signal s6_hit : std_logic;

    signal s7_y    : unsigned(9 downto 0);
    signal s7_u    : unsigned(9 downto 0);
    signal s7_v    : unsigned(9 downto 0);
    signal s7_int  : unsigned(9 downto 0);
    signal s7_hit  : std_logic;

    signal s8_y_mul : unsigned(19 downto 0);
    signal s8_u     : unsigned(9 downto 0);
    signal s8_v     : unsigned(9 downto 0);
    signal s8_hit   : std_logic;

    signal s9_y : unsigned(9 downto 0);
    signal s9_u : unsigned(9 downto 0);
    signal s9_v : unsigned(9 downto 0);

    signal s10_y_mul : unsigned(19 downto 0);
    signal s10_u     : unsigned(9 downto 0);
    signal s10_v     : unsigned(9 downto 0);

    signal s11_y : unsigned(9 downto 0);
    signal s11_u : unsigned(9 downto 0);
    signal s11_v : unsigned(9 downto 0);

    signal s12_y : unsigned(9 downto 0);
    signal s12_u : unsigned(9 downto 0);
    signal s12_v : unsigned(9 downto 0);

    function abs_s14(v : signed(13 downto 0)) return unsigned is
        variable t : signed(13 downto 0);
    begin
        if v(13) = '1' then t := -v; else t := v; end if;
        return unsigned(std_logic_vector(t(11 downto 0)));
    end function;

    function lfsr_step(s : std_logic_vector(15 downto 0))
        return std_logic_vector is
        variable v : std_logic;
    begin
        v := s(15) xor s(14) xor s(12) xor s(3);
        return s(14 downto 0) & v;
    end function;

begin

    -- ========================================================================
    -- Position + parameter latch.
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
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
                branches_r   <= unsigned(registers_in(0)(2 downto 0));
                jitter_r     <= unsigned(registers_in(1));
                glow_r       <= unsigned(registers_in(2));
                bright_r     <= unsigned(registers_in(3));
                trunk_bias_r <= unsigned(registers_in(5));
                storm_r      <= registers_in(6)(0);
                sharp_r      <= registers_in(6)(1);
                animate_r    <= registers_in(6)(2);
                blue_r       <= registers_in(6)(3);
                invert_r     <= registers_in(6)(4);
                spread_r     <= unsigned(registers_in(7));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Bolt path update.  At each hsync we advance each bolt's accumulator
    -- by its slope (×16 fixed point), and the bolt_x_r is acc / 16 added
    -- to the trunk centre × x.
    -- ========================================================================
    p_bolts : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
        variable v_step   : std_logic_vector(15 downto 0);
        variable v_jit    : signed(7 downto 0);
        variable v_trunk_x : signed(13 downto 0);
        variable v_spread_scale : signed(7 downto 0);
        variable v_slope_eff : signed(7 downto 0);
        variable v_acc_next  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- LFSR steps each cycle when animate, else holds.
            if animate_r = '1' then
                lfsr_r <= lfsr_step(lfsr_r);
            end if;

            -- Per-bolt jitter (registered each cycle so the hsync update is
            -- a clean register-to-register add).  Pick different LFSR bytes
            -- so each bolt's wobble is unique.
            for i in 0 to N_BOLTS - 1 loop
                v_jit := signed(lfsr_r(15 downto 8)) -
                         to_signed(128, 8);
                -- Scale jitter by jitter_r/256.
                bolt_jit_r(i) <= resize(shift_right(
                    v_jit * signed('0' & std_logic_vector(jitter_r(9 downto 4))),
                    7), 8);
            end loop;

            -- VSYNC: reset all bolts to the trunk's start_x.  Trunk
            -- (bolt 0) starts at centre + trunk_bias offset.  Branches
            -- start at the same centre — they don't diverge until they
            -- become active (i.e., pixel_y >= start_y).
            if v_v_edge = '1' then
                v_trunk_x := to_signed(960, 14) +
                             signed(resize(trunk_bias_r, 14)) -
                             to_signed(512, 14);
                for i in 0 to N_BOLTS - 1 loop
                    bolt_x_r(i)   <= v_trunk_x;
                    bolt_acc_r(i) <= to_signed(0, 14);
                end loop;
            elsif v_h_edge = '1' then
                -- Compute effective slope per bolt = base_slope × (1 + spread/4).
                for i in 0 to N_BOLTS - 1 loop
                    -- Each bolt becomes active once pixel_y reaches its
                    -- start_y, and stays active until pixel_y reaches end_y.
                    if to_integer(pixel_y) >= C_START_Y(i) and
                       to_integer(pixel_y) < C_END_Y(i) then
                        -- Active: advance accumulator by slope.  Effective
                        -- slope scaled by Tree Spread (slider).
                        v_slope_eff :=
                            to_signed(C_SLOPE_X16(i), 8) +
                            resize(shift_right(
                                to_signed(C_SLOPE_X16(i), 12) *
                                signed('0' &
                                    std_logic_vector(spread_r(9 downto 4))),
                                9), 8);
                        v_acc_next := bolt_acc_r(i) +
                                      resize(v_slope_eff, 14);
                        bolt_acc_r(i) <= v_acc_next;
                        -- bolt_x = trunk_x + acc>>4 + jitter
                        bolt_x_r(i) <= to_signed(960, 14) +
                                       resize(shift_right(v_acc_next, 4), 14) +
                                       resize(bolt_jit_r(i), 14);
                    end if;
                end loop;
            end if;
        end if;
    end process p_bolts;

    -- ========================================================================
    -- Bolt active flag — set if pixel_y is within bolt's Y range AND the
    -- bolt is enabled by the Branches knob.  Trunk (bolt 0) is always on.
    -- ========================================================================
    p_active : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to N_BOLTS - 1 loop
                if i = 0 then
                    bolt_act_r(0) <= '1';  -- trunk always
                elsif to_unsigned(i - 1, 3) < branches_r and
                      to_integer(pixel_y) >= C_START_Y(i) and
                      to_integer(pixel_y) < C_END_Y(i) then
                    bolt_act_r(i) <= '1';
                else
                    bolt_act_r(i) <= '0';
                end if;
            end loop;
        end if;
    end process p_active;

    -- ========================================================================
    -- Per-pixel pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pidx : integer range 0 to 7;
        variable v_int  : unsigned(11 downto 0);
        variable v_min  : unsigned(11 downto 0);
        variable v_idx  : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: per-bolt dx (signed 14-bit).  Inactive bolts forced to
            -- maximum so they never win the min reduction.
            for i in 0 to N_BOLTS - 1 loop
                if bolt_act_r(i) = '1' then
                    s1_dx(i) <= signed(resize(s0_x, 14)) - bolt_x_r(i);
                else
                    s1_dx(i) <= to_signed(8191, 14);
                end if;
            end loop;

            -- S2: abs.
            for i in 0 to N_BOLTS - 1 loop
                s2_adx(i) <= abs_s14(s1_dx(i));
            end loop;

            -- S3: 7 → 4 pair-compare; index 7 padded by max.
            for p in 0 to 2 loop
                if s2_adx(2 * p) <= s2_adx(2 * p + 1) then
                    s3_d(p) <= s2_adx(2 * p);
                    s3_i(p) <= to_unsigned(2 * p, 3);
                else
                    s3_d(p) <= s2_adx(2 * p + 1);
                    s3_i(p) <= to_unsigned(2 * p + 1, 3);
                end if;
            end loop;
            s3_d(3) <= s2_adx(6);
            s3_i(3) <= to_unsigned(6, 3);

            -- S4: 4 → 2.
            for p in 0 to 1 loop
                if s3_d(2 * p) <= s3_d(2 * p + 1) then
                    s4_d(p) <= s3_d(2 * p);
                    s4_i(p) <= s3_i(2 * p);
                else
                    s4_d(p) <= s3_d(2 * p + 1);
                    s4_i(p) <= s3_i(2 * p + 1);
                end if;
            end loop;

            -- S5: 2 → 1.
            if s4_d(0) <= s4_d(1) then
                s5_d <= s4_d(0); s5_i <= s4_i(0);
            else
                s5_d <= s4_d(1); s5_i <= s4_i(1);
            end if;

            -- S6: intensity ramp.
            if sharp_r = '1' then
                if s5_d < ("00" & glow_r(9 downto 2)) then
                    s6_int <= to_unsigned(1023, 10);
                    s6_hit <= '1';
                else
                    s6_int <= (others => '0');
                    s6_hit <= '0';
                end if;
            else
                if s5_d < ("00" & glow_r) then
                    v_int := resize(("00" & glow_r) - s5_d, 12);
                    if v_int >= to_unsigned(1023, 12) then
                        s6_int <= to_unsigned(1023, 10);
                    else
                        s6_int <= v_int(9 downto 0);
                    end if;
                    s6_hit <= '1';
                else
                    s6_int <= (others => '0');
                    s6_hit <= '0';
                end if;
            end if;
            s6_i <= s5_i;

            -- S7: palette lookup.
            v_pidx := to_integer(s6_i);
            if blue_r = '1' then
                s7_y <= C_PY_BLUE(v_pidx);
                s7_u <= C_PU_BLUE(v_pidx);
                s7_v <= C_PV_BLUE(v_pidx);
            else
                s7_y <= C_PY_WHITE(v_pidx);
                s7_u <= C_PU_WHITE(v_pidx);
                s7_v <= C_PV_WHITE(v_pidx);
            end if;
            s7_int <= s6_int;
            s7_hit <= s6_hit;

            -- S8: Y × intensity.
            s8_y_mul <= s7_y * s7_int;
            s8_u     <= s7_u;
            s8_v     <= s7_v;
            s8_hit   <= s7_hit;

            -- S9: extract Y; storm noise on miss.
            if s8_hit = '1' then
                s9_y <= s8_y_mul(19 downto 10);
                s9_u <= s8_u;
                s9_v <= s8_v;
            else
                if storm_r = '1' then
                    s9_y <= to_unsigned(40, 10) +
                            ("00" & unsigned(lfsr_r(7 downto 0)));
                else
                    s9_y <= (others => '0');
                end if;
                s9_u <= to_unsigned(512, 10);
                s9_v <= to_unsigned(512, 10);
            end if;

            -- S10: brightness mul.
            s10_y_mul <= s9_y * bright_r;
            s10_u     <= s9_u;
            s10_v     <= s9_v;

            -- S11: extract.
            s11_y <= s10_y_mul(19 downto 10);
            s11_u <= s10_u;
            s11_v <= s10_v;

            -- S12: invert.
            if invert_r = '1' then
                s12_y <= to_unsigned(1023, 10) - s11_y;
            else
                s12_y <= s11_y;
            end if;
            s12_u <= s11_u;
            s12_v <= s11_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s12_y);
    data_out.u       <= std_logic_vector(s12_u);
    data_out.v       <= std_logic_vector(s12_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture lightning;
