-- Truchet: random-orientation diagonal tile pattern.
--
-- The screen is divided into a grid of square cells.  Each cell has one of
-- two diagonal orientations selected by a hash of (cell_x, cell_y, seed).
-- A line is drawn along the chosen diagonal of each cell; when many cells
-- meet, the lines form curving paths and closed regions across the screen.
-- Seed animation makes the orientations flip over time.
--
-- Designed for 74.25 MHz HD timing on iCE40-HX4K (no DSP). No multipliers
-- needed: distance to diagonal is just abs(lx - ly) or abs(lx + ly - CELL).
--
-- Register map:
--   registers_in(0) = Cell Size      (rotary 1 — power-of-2 cell size)
--   registers_in(1) = Pattern Seed   (rotary 2 — random orientation seed)
--   registers_in(2) = Anim Speed     (rotary 3 — seed-shift rate)
--   registers_in(3) = Line Width     (rotary 4 — diagonal stroke width)
--   registers_in(4) = Color Mix      (rotary 5 — palette index spread)
--   registers_in(5) = Brightness     (rotary 6)
--   registers_in(6) = Switches:
--     b0   Style    (0=Diagonal lines / 1=Solid triangles)
--     b1   Palette  (0=Mono / 1=Duo)
--     b2   Animate  (0=Static / 1=Seed shifts each frame)
--     b3   Pattern  (0=Lines on bg / 1=Inverse)
--     b4   Invert
--   registers_in(7) = Density (slider, KEY — affects cell size mapping)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture truchet of program_top is

    constant LATENCY : natural := 8;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal frame_count  : unsigned(15 downto 0) := (others => '0');

    signal seed_r       : unsigned(9 downto 0) := to_unsigned(170, 10);
    signal anim_r       : unsigned(9 downto 0) := to_unsigned(64,  10);
    signal width_r      : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal colormix_r   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bright_r     : unsigned(9 downto 0) := to_unsigned(800, 10);
    signal cellsel_r    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal density_r    : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal style_r      : std_logic := '0';
    signal palette_r    : std_logic := '1';
    signal animate_r    : std_logic := '1';
    signal pattern_r    : std_logic := '0';
    signal invert_r     : std_logic := '0';

    -- Cell-size selection (power-of-2): cell_pow ∈ {4,5,6,7} = 16/32/64/128
    signal cell_pow_r   : unsigned(2 downto 0) := to_unsigned(5, 3);

    -- Frame-animated seed offset
    signal anim_seed_r  : unsigned(9 downto 0) := (others => '0');

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
    signal s0_x : unsigned(11 downto 0);
    signal s0_y : unsigned(11 downto 0);

    -- S1: cell index + in-cell pos via mask (based on cell_pow_r at vsync)
    signal s1_cx     : unsigned(11 downto 0);  -- cell index
    signal s1_cy     : unsigned(11 downto 0);
    signal s1_lx     : unsigned(6 downto 0);   -- in-cell pos (max 127)
    signal s1_ly     : unsigned(6 downto 0);

    -- S2: hash bit + tile sum/diff for diagonal distance
    signal s2_orient : std_logic;
    signal s2_lx     : unsigned(6 downto 0);
    signal s2_ly     : unsigned(6 downto 0);
    signal s2_cx     : unsigned(11 downto 0);
    signal s2_cy     : unsigned(11 downto 0);

    -- S3: signed diagonal distance components
    signal s3_diag_d : signed(8 downto 0);   -- (lx-ly) for orient 0
    signal s3_diag_o : signed(8 downto 0);   -- (lx+ly - cell_size) for orient 1
    signal s3_orient : std_logic;
    signal s3_cx     : unsigned(11 downto 0);
    signal s3_cy     : unsigned(11 downto 0);

    -- S4: distance from active diagonal (abs)
    signal s4_dist   : unsigned(7 downto 0);
    signal s4_cx     : unsigned(11 downto 0);
    signal s4_cy     : unsigned(11 downto 0);

    -- S5: on-line flag + palette index
    signal s5_on_line : std_logic;
    signal s5_pal_idx : unsigned(2 downto 0);

    -- S6: assemble brightness + raw chroma
    signal s6_y : unsigned(9 downto 0);
    signal s6_u : unsigned(9 downto 0);
    signal s6_v : unsigned(9 downto 0);

    -- S7: brightness mul + invert
    signal s7_ymul : unsigned(19 downto 0);
    signal s7_u    : unsigned(9 downto 0);
    signal s7_v    : unsigned(9 downto 0);

    -- Final
    signal s_out_y : unsigned(9 downto 0);
    signal s_out_u : unsigned(9 downto 0);
    signal s_out_v : unsigned(9 downto 0);

begin

    -- ========================================================================
    -- Position + parameters
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
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                v_h_edge := '1';
            end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                v_v_edge := '1';
            end if;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                frame_count <= frame_count + 1;

                seed_r     <= unsigned(registers_in(1));
                anim_r     <= unsigned(registers_in(2));
                width_r    <= unsigned(registers_in(3));
                colormix_r <= unsigned(registers_in(4));
                bright_r   <= unsigned(registers_in(5));
                style_r    <= registers_in(6)(0);
                palette_r  <= registers_in(6)(1);
                animate_r  <= registers_in(6)(2);
                pattern_r  <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                density_r  <= unsigned(registers_in(7));

                -- Cell size selection from density slider top 2 bits
                case to_integer(unsigned(registers_in(7)(9 downto 8))) is
                    when 0      => cell_pow_r <= to_unsigned(7, 3);  -- 128 px
                    when 1      => cell_pow_r <= to_unsigned(6, 3);  -- 64 px
                    when 2      => cell_pow_r <= to_unsigned(5, 3);  -- 32 px
                    when others => cell_pow_r <= to_unsigned(4, 3);  -- 16 px
                end case;

                -- Animate seed: every N frames, advance anim_seed by anim knob.
                if registers_in(6)(2) = '1' then
                    anim_seed_r <= anim_seed_r +
                                   unsigned(registers_in(2)(9 downto 4));
                end if;
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pow      : integer range 4 to 7;
        variable v_mask     : unsigned(6 downto 0);
        variable v_hash     : unsigned(15 downto 0);
        variable v_cellsize : unsigned(7 downto 0);
        variable v_dist     : signed(8 downto 0);
        variable v_pidx     : integer range 0 to 7;
        variable v_y_mul    : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0
            s0_x <= pixel_x;
            s0_y <= pixel_y;

            -- S1: split pixel coords into cell + in-cell parts.
            -- cell_pow_r values 4..7 → mask = 2^pow - 1.
            v_pow := to_integer(cell_pow_r);
            v_mask := (others => '0');
            for i in 0 to 6 loop
                if i < v_pow then v_mask(i) := '1'; end if;
            end loop;
            s1_cx <= s0_x srl v_pow;  -- cell index
            s1_cy <= s0_y srl v_pow;
            s1_lx <= s0_x(6 downto 0) and v_mask;  -- in-cell x
            s1_ly <= s0_y(6 downto 0) and v_mask;

            -- S2: hash bit = some XOR-based mixing of cx, cy, seed
            v_hash := (s1_cx(7 downto 0) & s1_cy(7 downto 0)) xor
                      ("000000" & seed_r) xor
                      ("000000" & anim_seed_r);
            s2_orient <= v_hash(7) xor v_hash(11) xor v_hash(3);
            s2_lx <= s1_lx;
            s2_ly <= s1_ly;
            s2_cx <= s1_cx;
            s2_cy <= s1_cy;

            -- S3: diagonal distance terms.
            -- orient 0 (TL-BR diagonal): pixel on line if lx == ly (so dist = lx-ly)
            -- orient 1 (TR-BL diagonal): pixel on line if lx + ly == cell_size
            s3_diag_d <= signed(resize(s2_lx, 9)) -
                         signed(resize(s2_ly, 9));
            s3_diag_o <= signed(resize(s2_lx, 9)) +
                         signed(resize(s2_ly, 9)) -
                         signed(resize(v_mask, 9));
            s3_orient <= s2_orient;
            s3_cx <= s2_cx;
            s3_cy <= s2_cy;

            -- S4: pick the active diagonal distance & take absolute value.
            if s3_orient = '0' then
                v_dist := s3_diag_d;
            else
                v_dist := s3_diag_o;
            end if;
            if v_dist < 0 then v_dist := -v_dist; end if;
            s4_dist <= unsigned(std_logic_vector(v_dist(7 downto 0)));
            s4_cx <= s3_cx;
            s4_cy <= s3_cy;

            -- S5: on-line if dist < width/8 (mapping width 0..1023 → 0..127)
            -- Style 0: line if dist < threshold
            -- Style 1: triangle fill if (cells xor orient) gives one of two halves
            if style_r = '0' then
                if s4_dist < ("0" & width_r(9 downto 3)) then
                    s5_on_line <= '1';
                else
                    s5_on_line <= '0';
                end if;
            else
                -- Triangle fill: use sign of diagonal as half-fill
                if s4_dist(7) = '0' then
                    s5_on_line <= '1';
                else
                    s5_on_line <= '0';
                end if;
            end if;
            -- Palette index from cell sum + colormix
            s5_pal_idx <= s4_cx(2 downto 0) xor s4_cy(2 downto 0) xor
                          colormix_r(9 downto 7);

            -- S6: assemble color
            if s5_on_line = (not pattern_r) then
                -- Foreground
                if palette_r = '0' then
                    s6_y <= to_unsigned(900, 10);
                    s6_u <= to_unsigned(512, 10);
                    s6_v <= to_unsigned(512, 10);
                else
                    v_pidx := to_integer(s5_pal_idx);
                    s6_y <= C_PAL_Y(v_pidx);
                    s6_u <= C_PAL_U(v_pidx);
                    s6_v <= C_PAL_V(v_pidx);
                end if;
            else
                -- Background
                s6_y <= to_unsigned(40, 10);
                s6_u <= to_unsigned(512, 10);
                s6_v <= to_unsigned(512, 10);
            end if;

            -- S7: brightness mul (raw)
            s7_ymul <= s6_y * bright_r;
            s7_u <= s6_u;
            s7_v <= s6_v;

            -- Final: extract + invert
            if invert_r = '1' then
                s_out_y <= to_unsigned(1023, 10) - s7_ymul(19 downto 10);
            else
                s_out_y <= s7_ymul(19 downto 10);
            end if;
            s_out_u <= s7_u;
            s_out_v <= s7_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture truchet;
