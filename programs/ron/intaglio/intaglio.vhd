-- intaglio.vhd
--
-- INTAGLIO -- the picture, engraved.
--
-- The frame is redrawn as a copperplate engraving: hatch strokes that
-- follow the image's form (stroke direction = the local luma-gradient
-- direction, bucketed into 8 coherent stroke families 22.5 degrees apart),
-- stroke THICKNESS carrying the tone, crosshatch engaging level by level
-- in the shadows, clean paper in the highlights, and dark contour ink on
-- edges. Everything is streaming and stateless -- no coarse field, no
-- frame memory, no warm-up: every control reacts on the next frame.
--
--   P12 "Bite" (headline): acid depth. Low = a barely-touched plate (only
--        the deepest shadows carry strokes); mid = full tonal engraving;
--        high = over-bitten, crosshatch swallowing the midtones.
--
-- Controls:
--   K1 Pitch    stroke spacing (4 zones, ~8/16/33/66 px)
--   K2 Angle    plate rotation (continuous 0..180, per-frame qsin consts)
--   K3 Ink      stroke width gain + contour ink weight
--   K4 Tint     ink hue (black / sepia / blood / iron-gall blue sweep)
--   K5 Hand     hand tremor: per-line stroke-phase jitter
--   K6 Style    stipple  /  engraving  /  woodcut (parameter presets)
--   S7 Invert   scratchboard (white strokes on inked ground)
--   S8 Wash     half-saturation video colour under-print on the paper
--   S9 Contour  edge ink on/off
--   S10 Cross   crosshatch at 90 (classic) / 45 (freehand) degrees
--   S11 Drift   live plate: stroke phases crawl slowly
--
-- Architecture: single streaming pipeline S0..S12 (C_LATENCY = 13).
-- ZERO per-pixel multiplies, 4 EBR (two 2048x8 luma line buffers).
--
--   * 3x3 Sobel over [1 2 1]-h-blurred luma (prism/mercurial window, 2
--     line buffers). Direction is bucketed into 8 bins over 180 degrees
--     with four shift-approximated tan() boundary compares -- no atan2.
--   * STRIPE-FIELD ACCUMULATORS (the trick that makes it cheap): the 8
--     possible stroke families are global stripe fields p_k = x*cos_k +
--     y*sin_k, maintained as running accumulators (+cos_k per pixel,
--     +sin_k per line) -- zero multiplies, and every pixel that picks
--     family k sees the same coherent field, so strokes are continuous
--     across bucket regions. cos/sin come from 16 borrowed qsin taps
--     rebuilt each frame, which is what makes K2 a continuous plate
--     rotation. Interlace is detected (field parity toggle) and the line
--     step doubled so stroke ANGLES are true in frame space.
--   * Tone -> ink: dark = 255 - luma; the tonal ladder t1/t2/t3 (hatch /
--     cross / third direction at 45) slides down as Bite rises; stroke
--     width = base + (dark - t)>>wsh, distance-to-stripe-center compare
--     with a half-ink antialias band. Contour ink from the Sobel
--     magnitude on top. Stipple swaps the stripe test for a hash-lattice
--     dot test (diamond sub-cell distance); woodcut is a pure per-frame
--     parameter preset (single bold direction, fat strokes, heavy edge).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture intaglio of program_top is

    constant C_LATENCY : integer := 13;

    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    function f_absu(v : signed) return unsigned is
    begin
        if v < 0 then
            return unsigned(-resize(v, v'length + 1));
        else
            return unsigned(resize(v, v'length + 1));
        end if;
    end function;

    -- quarter-wave folded sine, 64-entry logic ROM (fireworks lineage)
    type t_qsin is array(0 to 63) of signed(9 downto 0);
    constant C_QSIN : t_qsin := (
        to_signed(  0, 10), to_signed( 13, 10), to_signed( 25, 10), to_signed( 38, 10), to_signed( 50, 10), to_signed( 63, 10), to_signed( 75, 10), to_signed( 87, 10),
        to_signed(100, 10), to_signed(112, 10), to_signed(124, 10), to_signed(136, 10), to_signed(148, 10), to_signed(160, 10), to_signed(172, 10), to_signed(184, 10),
        to_signed(196, 10), to_signed(207, 10), to_signed(218, 10), to_signed(230, 10), to_signed(241, 10), to_signed(252, 10), to_signed(263, 10), to_signed(273, 10),
        to_signed(284, 10), to_signed(294, 10), to_signed(304, 10), to_signed(314, 10), to_signed(324, 10), to_signed(334, 10), to_signed(343, 10), to_signed(352, 10),
        to_signed(361, 10), to_signed(370, 10), to_signed(379, 10), to_signed(387, 10), to_signed(395, 10), to_signed(403, 10), to_signed(410, 10), to_signed(418, 10),
        to_signed(425, 10), to_signed(432, 10), to_signed(438, 10), to_signed(445, 10), to_signed(451, 10), to_signed(456, 10), to_signed(462, 10), to_signed(467, 10),
        to_signed(472, 10), to_signed(477, 10), to_signed(481, 10), to_signed(485, 10), to_signed(489, 10), to_signed(492, 10), to_signed(496, 10), to_signed(499, 10),
        to_signed(501, 10), to_signed(503, 10), to_signed(505, 10), to_signed(507, 10), to_signed(509, 10), to_signed(510, 10), to_signed(510, 10), to_signed(511, 10));

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then
            v_i := a(7 downto 2);
        else
            v_i := not a(7 downto 2);
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    -- two-stage value-noise hash (inferno lesson: 1-add hashes weave)
    function f_hash_a(cx, cy : unsigned(7 downto 0);
                      seed   : unsigned(15 downto 0)) return unsigned is
        variable h16 : unsigned(15 downto 0);
        variable h   : unsigned(11 downto 0);
    begin
        h16 := ((cx & cy)) xor seed;
        h16 := h16 xor (x"0" & h16(15 downto 4));
        h   := h16(11 downto 0);
        return h + rotate_left(h, 5);
    end function;

    function f_hash_b(hin : unsigned(11 downto 0)) return unsigned is
        variable h : unsigned(11 downto 0);
    begin
        h := hin xor rotate_left(hin, 9);
        h := h + rotate_left(h, 3);
        h := h xor ("00000" & h(11 downto 5));
        return h(9 downto 2);
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms
    ----------------------------------------------------------------------
    signal s_p12   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k2    : unsigned(9 downto 0) := (others => '0');
    signal s_k3    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k4    : unsigned(9 downto 0) := (others => '0');
    signal s_k5    : unsigned(9 downto 0) := (others => '0');
    signal s_inv   : std_logic := '0';                       -- S7
    signal s_wash  : std_logic := '0';                       -- S8
    signal s_edge_on : std_logic := '1';                     -- S9
    signal s_cross45 : std_logic := '0';                     -- S10
    signal s_drift : std_logic := '0';                       -- S11

    -- style decode (K6): 0 stipple, 1 engraving, 2 woodcut
    signal s_style : unsigned(1 downto 0) := "01";

    -- stripe-field constants (per frame, from 16 qsin taps)
    type t_s8arr is array (0 to 7) of signed(7 downto 0);
    signal s_ck, s_sk : t_s8arr := (others => (others => '0'));
    -- line step (doubled when interlaced) + field-parity start offset
    signal s_sk2 : t_s8arr := (others => (others => '0'));
    signal s_pitch_sh : natural range 0 to 3 := 1;
    signal s_bin0  : unsigned(2 downto 0) := (others => '0');  -- flat-area bin
    signal s_xoff  : unsigned(2 downto 0) := "010";            -- cross offset

    -- tonal ladder + widths (per frame)
    signal s_t1, s_t2, s_t3 : unsigned(8 downto 0) := (others => '1');
    signal s_wsh   : natural range 2 to 4 := 3;
    signal s_wbase : unsigned(6 downto 0) := to_unsigned(10, 7);
    signal s_wmax  : unsigned(6 downto 0) := to_unsigned(96, 7);
    signal s_ethr  : unsigned(11 downto 0) := to_unsigned(400, 12);
    signal s_magfl : unsigned(11 downto 0) := to_unsigned(24, 12);
    signal s_jsh   : natural range 0 to 4 := 4;               -- hand tremor
    signal s_stip  : std_logic := '0';

    -- ink / paper palette (per frame)
    signal s_ink_u, s_ink_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ink_y  : unsigned(9 downto 0) := to_unsigned(120, 10);
    signal s_pap_y  : unsigned(9 downto 0) := to_unsigned(930, 10);
    signal s_pap_u  : unsigned(9 downto 0) := to_unsigned(506, 10);
    signal s_pap_v  : unsigned(9 downto 0) := to_unsigned(518, 10);

    signal s_dracc : unsigned(11 downto 0) := (others => '0');  -- drift

    signal s_seq   : unsigned(4 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');
    signal s_qs_r  : signed(9 downto 0) := (others => '0');
    signal s_th0   : unsigned(9 downto 0) := (others => '0');

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';

    -- interlace detect: field parity of this and the previous field
    signal s_fpar, s_fpar_p : std_logic := '0';
    signal s_ilace : std_logic := '1';

    ----------------------------------------------------------------------
    -- pipeline
    ----------------------------------------------------------------------
    signal av : std_logic_vector(1 to 12) := (others => '0');

    signal r0_y, r0_u, r0_v : unsigned(9 downto 0) := (others => '0');
    signal s_x_count : unsigned(10 downto 0) := (others => '0');
    signal s_aline   : unsigned(9 downto 0) := (others => '0');
    signal s_seen    : std_logic := '0';

    -- per-line tremor jitter (lstep sequencer)
    signal s_lstep  : unsigned(1 downto 0) := (others => '0');
    signal s_lh_a   : unsigned(11 downto 0) := (others => '0');
    signal s_ljit8  : unsigned(7 downto 0) := (others => '0');

    -- S1: write-path blur history + read counters
    signal a1_y8 : unsigned(7 downto 0) := (others => '0');
    signal h1, h2 : unsigned(7 downto 0) := (others => '0');
    signal wb8   : unsigned(7 downto 0) := (others => '0');
    signal lrx, lwx : unsigned(10 downto 0) := (others => '0');

    -- line buffers (2 rows above the current)
    type t_lb8 is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb1, lb2 : t_lb8 := (others => (others => '0'));
    signal q1, q2   : std_logic_vector(7 downto 0) := (others => '0');

    -- S2 land, S3 sums, S4 gradient
    signal qr1, qr2 : unsigned(7 downto 0) := (others => '0');
    signal y8d      : unsigned(7 downto 0) := (others => '0');
    signal ss, ss1, ss2 : unsigned(9 downto 0) := (others => '0');
    signal dy, dy1, dy2 : signed(8 downto 0) := (others => '0');
    signal cl3, cl4, cl5, cl6, cl7 : unsigned(7 downto 0) := (others => '0');
    signal s_gx, s_gy : signed(11 downto 0) := (others => '0');

    -- S5 magnitude + |g| pair, S6 bucket
    signal mag5 : unsigned(12 downto 0) := (others => '0');
    signal ax5, ay5 : unsigned(11 downto 0) := (others => '0');
    signal sgn5 : std_logic := '0';
    signal bin6 : unsigned(2 downto 0) := (others => '0');
    signal bin_hold : unsigned(2 downto 0) := (others => '0');
    signal hold_cnt : unsigned(5 downto 0) := (others => '0');
    signal mag6 : unsigned(12 downto 0) := (others => '0');
    signal x_s6 : unsigned(10 downto 0) := (others => '0');
    -- stipple lattice track
    signal st1_8, st2_8 : unsigned(7 downto 0) := (others => '0');
    signal d1_6, d2_6 : unsigned(5 downto 0) := (others => '0');
    signal sh7  : unsigned(11 downto 0) := (others => '0');
    signal sj8  : unsigned(7 downto 0) := (others => '0');
    signal d1_7, d2_7 : unsigned(5 downto 0) := (others => '0');
    signal d1_8, d2_8 : unsigned(5 downto 0) := (others => '0');

    -- stripe-field accumulators (advance under av(6); read same clock)
    type t_p16 is array (0 to 7) of unsigned(15 downto 0);
    signal p_acc : t_p16 := (others => (others => '0'));
    signal b_acc : t_p16 := (others => (others => '0'));

    -- S7 phase picks, S8 distances, S9 ink decisions
    signal ph_h7, ph_x7, ph_t7 : unsigned(7 downto 0) := (others => '0');
    signal mag7 : unsigned(12 downto 0) := (others => '0');
    signal dh8, dx8, dt8 : unsigned(6 downto 0) := (others => '0');
    signal mag8 : unsigned(12 downto 0) := (others => '0');
    signal w1_8, w2_8, w3_8 : unsigned(6 downto 0) := (others => '0');
    signal en1_8, en2_8, en3_8 : std_logic := '0';
    signal ink9  : unsigned(1 downto 0) := (others => '0');
    signal edge9 : std_logic := '0';

    -- chroma/luma delay to compose (S9/S10)
    type t_d10 is array (1 to 10) of unsigned(9 downto 0);
    signal du, dv, dyy : t_d10 := (others => (others => '0'));

    -- S10 lanes, S11 select, S12 out
    signal ln_iy, ln_iu, ln_iv : unsigned(9 downto 0) := (others => '0');
    signal ln_py, ln_pu, ln_pv : unsigned(9 downto 0) := (others => '0');
    signal ln_hy, ln_hu, ln_hv : unsigned(9 downto 0) := (others => '0');
    signal sel11 : unsigned(1 downto 0) := (others => '0');
    signal ink10 : unsigned(1 downto 0) := (others => '0');
    signal edge10 : std_logic := '0';
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared qsin port: the sequencer rebuilds the 8 stripe-field constant
    -- pairs (16 taps) + 2 ink-hue taps each frame; both the angle and ROM
    -- output are registered (mercurial rule). No per-pixel consumer, so
    -- the port is the sequencer's alone.
    ------------------------------------------------------------------------
    s_qs_a <= s_th0 + shift_left(resize(31 - resize(s_seq, 10), 10), 6)
                    + to_unsigned(256, 10)          when s_seq >= 24 else
              s_th0 + shift_left(resize(23 - resize(s_seq, 10), 10), 6)
                                                    when s_seq >= 16 else
              resize(s_k4, 10) + to_unsigned(256, 10) when s_seq = 8 else
              resize(s_k4, 10);

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (one add per step).
    -- Steps 31..24: present cos angles k=0..7 (captured 29..22 into ck)
    -- Steps 23..16: present sin angles      (captured 21..14 into sk)
    -- Steps 13..1 : tonal ladder, style presets, palette
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_t1 : signed(10 downto 0);
        variable v_k  : integer range 0 to 7;
        variable v_c8 : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_p12 <= unsigned(registers_in(7));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_inv     <= registers_in(6)(0);
                s_wash    <= registers_in(6)(1);
                s_edge_on <= registers_in(6)(2);
                s_cross45 <= registers_in(6)(3);
                s_drift   <= registers_in(6)(4);

                s_pitch_sh <= to_integer(unsigned(registers_in(0)(9 downto 8)));
                s_style    <= unsigned(registers_in(5)(9 downto 8));

                -- interlace detect: parity toggled since last field?
                s_fpar_p <= s_fpar;
                s_fpar   <= data_in.field_n;
                if data_in.field_n /= s_fpar then
                    s_ilace <= '1';
                else
                    s_ilace <= '0';
                end if;

                -- plate angle: K2 sweeps 0..180 degrees; drift crawls the
                -- stroke phases when S11 is on
                s_th0 <= resize(unsigned(registers_in(1)(9 downto 1)), 10);
                if registers_in(6)(4) = '1' then
                    s_dracc <= s_dracc + 5;
                end if;

                s_seq <= to_unsigned(31, 5);
            elsif s_seq /= 0 and data_in.avid = '0' then
                -- qsin captures land two steps behind the presented angle
                if s_seq <= 29 and s_seq >= 22 then
                    v_k := to_integer(to_unsigned(29, 5) - s_seq);
                    v_c8 := resize(shift_right(s_qs_r, 4), 8);
                    s_ck(v_k) <= v_c8;
                end if;
                if s_seq <= 21 and s_seq >= 14 then
                    v_k := to_integer(to_unsigned(21, 5) - s_seq);
                    v_c8 := resize(shift_right(s_qs_r, 4), 8);
                    s_sk(v_k) <= v_c8;
                    if s_ilace = '1' then
                        s_sk2(v_k) <= shift_left(v_c8, 1);
                    else
                        s_sk2(v_k) <= v_c8;
                    end if;
                end if;
                case to_integer(s_seq) is
                    when 13 =>
                        -- tonal ladder from Bite: thresholds slide DOWN as
                        -- the plate bites deeper (dark8 must exceed t)
                        v_t1 := to_signed(300, 11)
                                - signed(resize(shift_right(s_p12, 1), 11));
                        if v_t1 < 0 then
                            s_t1 <= (others => '0');
                        else
                            s_t1 <= resize(unsigned(v_t1), 9);
                        end if;
                    when 12 =>
                        if s_t1 > 383 then
                            s_t2 <= (others => '1');
                        else
                            s_t2 <= s_t1 + 128;
                        end if;
                        -- ink width gain
                        case s_k3(9 downto 8) is
                            when "00"   => s_wsh <= 4;
                            when "01"   => s_wsh <= 3;
                            when others => s_wsh <= 2;
                        end case;
                    when 11 =>
                        if s_t2 > 415 then
                            s_t3 <= (others => '1');
                        else
                            s_t3 <= s_t2 + 96;
                        end if;
                        -- contour threshold: Ink opens it, Bite lowers it
                        s_ethr <= to_unsigned(160, 12)
                                  + resize(not s_k3(9 downto 3), 12)
                                  + resize(not s_p12(9 downto 3), 12);
                    when 10 =>
                        -- style presets (parameter-only: woodcut = bold
                        -- single-direction, stipple flag swaps the test)
                        s_stip <= '0';
                        case s_style is
                            when "00" =>            -- stipple
                                s_stip  <= '1';
                                s_wbase <= to_unsigned(8, 7);
                                s_wmax  <= to_unsigned(80, 7);
                            when "11" =>            -- woodcut
                                s_wbase <= to_unsigned(22, 7);
                                s_wmax  <= to_unsigned(116, 7);
                            when others =>          -- engraving
                                s_wbase <= to_unsigned(10, 7);
                                s_wmax  <= to_unsigned(96, 7);
                        end case;
                        if s_cross45 = '1' then
                            s_xoff <= "010";        -- 45 degrees
                        else
                            s_xoff <= "100";        -- 90 degrees
                        end if;
                        -- flat-area default bin follows the plate angle
                        -- (rounded to the even 4-direction set)
                        s_bin0 <= (s_th0(8 downto 6) + 1) and "110";
                        -- tremor depth
                        if s_k5(9 downto 7) = 0 then
                            s_jsh <= 0;
                        else
                            s_jsh <= to_integer(s_k5(9 downto 8)) + 1;
                        end if;
                    when 9 =>
                        -- paper (warm cream; scratchboard swaps at compose)
                        s_pap_y <= to_unsigned(930, 10);
                        s_pap_u <= to_unsigned(506, 10);
                        s_pap_v <= to_unsigned(518, 10);
                        s_ink_y <= to_unsigned(110, 10);
                        -- direction floor: low enough that soft form
                        -- shading steers the strokes, above dither noise
                        s_magfl <= to_unsigned(24, 12);
                    when 6 =>
                        -- capture two-behind: qsin(K4 + 256) -> ink U.
                        -- Subtle tint only (>>4, warm bias): printing ink
                        -- stays near-black, K4 steers the tint direction
                        s_ink_u <= f_cu10(to_signed(504, 11)
                                          + resize(shift_right(s_qs_r, 4), 11));
                    when 5 =>
                        null;
                    when 4 =>
                        -- capture: qsin(K4) -> ink V
                        s_ink_v <= f_cu10(to_signed(524, 11)
                                          + resize(shift_right(s_qs_r, 4), 11));
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a, v_b : unsigned(11 downto 0);
        variable v_t1c, v_t2c, v_t3c, v_t4c : unsigned(14 downto 0);
        variable v_bq : unsigned(2 downto 0);
        variable v_ph : unsigned(7 downto 0);
        variable v_pp : unsigned(15 downto 0);
        variable v_dk : signed(10 downto 0);
        variable v_w  : unsigned(8 downto 0);
        variable v_wc : unsigned(6 downto 0);
        variable v_ink : unsigned(1 downto 0);
        variable v_iy : signed(11 downto 0);
        variable v_dd : unsigned(6 downto 0);
        variable v_hf : signed(6 downto 0);
        variable v_rad, v_cap : unsigned(6 downto 0);
        variable v_sub, v_sb2 : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            av(1) <= data_in.avid;
            for i in 2 to 12 loop
                av(i) <= av(i - 1);
            end loop;
            if data_in.avid = '1' then
                s_seen <= '1';
            end if;

            -- S0: input register
            if data_in.avid = '1' then
                r0_y <= unsigned(data_in.y);
                r0_u <= unsigned(data_in.u);
                r0_v <= unsigned(data_in.v);
                s_x_count <= s_x_count + 1;
            end if;

            -- S1: [1 2 1]/4 write-path blur history; read addr counter
            if av(1) = '1' then
                a1_y8 <= r0_y(9 downto 2);
                h1 <= a1_y8;
                h2 <= h1;
                wb8 <= resize(shift_right(resize(h2, 10)
                              + shift_left(resize(h1, 10), 1)
                              + resize(a1_y8, 10), 2), 8);
                lrx <= lrx + 1;
            end if;

            -- S2: land the line-buffer reads; buffer writes ride this
            -- stage (p_lb1/2)
            if av(2) = '1' then
                qr1 <= unsigned(q1);
                qr2 <= unsigned(q2);
                y8d <= wb8;
                lwx <= lwx + 1;
            end if;

            -- S3: Sobel column sums + two-column histories; center-row
            -- luma tap rides along
            if av(3) = '1' then
                ss  <= resize(qr2, 10) + shift_left(resize(qr1, 10), 1)
                       + resize(y8d, 10);
                dy  <= signed(resize(qr2, 9)) - signed(resize(y8d, 9));
                ss1 <= ss;  ss2 <= ss1;
                dy1 <= dy;  dy2 <= dy1;
                cl3 <= qr1;
            end if;

            -- S4: gradients (gx horizontal, gy vertical)
            if av(4) = '1' then
                s_gx <= signed(resize(ss, 12)) - signed(resize(ss2, 12));
                s_gy <= resize(dy, 12) + shift_left(resize(dy1, 12), 1)
                        + resize(dy2, 12);
                cl4 <= cl3;
            end if;

            -- S5: |gx|, |gy|, magnitude, quadrant sign
            if av(5) = '1' then
                ax5 <= resize(f_absu(s_gx), 12);
                ay5 <= resize(f_absu(s_gy), 12);
                mag5 <= resize(f_absu(s_gx), 13) + resize(f_absu(s_gy), 13);
                if (s_gx < 0) /= (s_gy < 0) then
                    sgn5 <= '1';                     -- 90..180 quadrant
                else
                    sgn5 <= '0';
                end if;
                cl5 <= cl4;
            end if;

            -- S6: direction bucket -- |gy| against four shift-approximated
            -- tan boundaries of |gx| (11.25/33.75/56.25/78.75 deg); flat
            -- areas (below the mag floor) fall to the plate-angle bin.
            -- The stripe accumulators advance this stage; the warm-up
            -- guard blanks the window's first lines/columns.
            if av(6) = '1' then
                v_a := ax5;
                v_b := ay5;
                v_t1c := resize(shift_right(v_a, 3), 15)
                         + resize(shift_right(v_a, 4), 15);
                v_t2c := resize(shift_right(v_a, 1), 15)
                         + resize(shift_right(v_a, 3), 15)
                         + resize(shift_right(v_a, 5), 15);
                v_t3c := resize(v_a, 15) + resize(shift_right(v_a, 1), 15);
                v_t4c := shift_left(resize(v_a, 15), 2) + resize(v_a, 15);
                if resize(v_b, 15) < v_t1c then
                    v_bq := "000";
                elsif resize(v_b, 15) < v_t2c then
                    v_bq := "001";
                elsif resize(v_b, 15) < v_t3c then
                    v_bq := "010";
                elsif resize(v_b, 15) < v_t4c then
                    v_bq := "011";
                else
                    v_bq := "100";
                end if;
                if sgn5 = '1' and v_bq /= 0 then
                    v_bq := to_unsigned(8, 4)(2 downto 0) - v_bq;
                end if;
                -- round to the nearest EVEN family (4 stroke directions,
                -- 45 apart): 22.5-degree selection granularity flips
                -- family on tiny gradient changes and shreds soft shading
                -- into patchwork
                v_bq := (v_bq + 1) and "110";
                if s_aline < 4 or x_s6 < 8 then
                    bin6 <= s_bin0;
                    hold_cnt <= (others => '0');
                elsif mag5 >= s_magfl then
                    bin6 <= v_bq;
                    bin_hold <= v_bq;
                    hold_cnt <= (others => '1');
                elsif hold_cnt /= 0 then
                    -- ridge/valley zones (gradient through zero) inherit
                    -- the last strong direction on this line instead of
                    -- striping with the default family
                    bin6 <= bin_hold;
                    hold_cnt <= hold_cnt - 1;
                else
                    bin6 <= s_bin0;
                end if;
                mag6 <= mag5;
                cl6  <= cl5;
                x_s6 <= x_s6 + 1;

                -- stipple lattice coords + sub-cell diagonal coords
                st1_8 <= resize(shift_right(x_s6 + resize(s_aline, 11),
                                            s_pitch_sh + 3), 8);
                st2_8 <= resize(shift_right(x_s6 - resize(s_aline, 11),
                                            s_pitch_sh + 3), 8);
                v_sub := x_s6 + resize(s_aline, 11);
                v_sb2 := x_s6 - resize(s_aline, 11);
                case s_pitch_sh is
                    when 0 =>
                        d1_6 <= resize(v_sub(2 downto 0), 6);
                        d2_6 <= resize(v_sb2(2 downto 0), 6);
                    when 1 =>
                        d1_6 <= resize(v_sub(3 downto 0), 6);
                        d2_6 <= resize(v_sb2(3 downto 0), 6);
                    when 2 =>
                        d1_6 <= resize(v_sub(4 downto 0), 6);
                        d2_6 <= resize(v_sb2(4 downto 0), 6);
                    when others =>
                        d1_6 <= resize(v_sub(5 downto 0), 6);
                        d2_6 <= resize(v_sb2(5 downto 0), 6);
                end case;

                for k in 0 to 7 loop
                    p_acc(k) <= p_acc(k) + unsigned(resize(s_ck(k), 16));
                end loop;
            end if;

            -- S7: pick the three stroke phases (hatch / cross / third) by
            -- bucket mux (shift by pitch, then a STATIC slice); hand
            -- tremor jitters the phase per line; stipple hash stage A
            if av(7) = '1' then
                v_pp := shift_right(p_acc(to_integer(bin6)), s_pitch_sh);
                v_ph := v_pp(7 downto 0);
                if s_jsh /= 0 then
                    v_ph := v_ph + shift_right(s_ljit8, 4 - s_jsh);
                end if;
                ph_h7 <= v_ph;
                v_pp := shift_right(p_acc(to_integer(bin6 + s_xoff)),
                                    s_pitch_sh);
                v_ph := v_pp(7 downto 0);
                if s_jsh /= 0 then
                    v_ph := v_ph + shift_right(s_ljit8, 4 - s_jsh);
                end if;
                ph_x7 <= v_ph;
                v_pp := shift_right(p_acc(to_integer(bin6 + "010")),
                                    s_pitch_sh);
                ph_t7 <= v_pp(7 downto 0);
                mag7 <= mag6;
                cl7  <= cl6;

                sh7  <= f_hash_a(st1_8, st2_8, x"7A3C");
                d1_7 <= d1_6;
                d2_7 <= d2_6;
            end if;

            -- S8: stripe-center distances + tonal widths; stipple hash B
            if av(8) = '1' then
                if ph_h7 >= 128 then
                    dh8 <= resize(shift_right(ph_h7 - 128, 1), 7);
                else
                    dh8 <= resize(shift_right(128 - ph_h7, 1), 7);
                end if;
                if ph_x7 >= 128 then
                    dx8 <= resize(shift_right(ph_x7 - 128, 1), 7);
                else
                    dx8 <= resize(shift_right(128 - ph_x7, 1), 7);
                end if;
                if ph_t7 >= 128 then
                    dt8 <= resize(shift_right(ph_t7 - 128, 1), 7);
                else
                    dt8 <= resize(shift_right(128 - ph_t7, 1), 7);
                end if;

                -- widths: w = base + (dark - t)>>wsh, clamped; enables
                v_dk := signed(resize(not cl7, 11)) - signed(resize(s_t1, 11));
                en1_8 <= '0';
                if v_dk >= 0 then
                    en1_8 <= '1';
                    v_w := resize(s_wbase, 9)
                           + resize(shift_right(unsigned(resize(v_dk, 11)),
                                                s_wsh), 9);
                    if v_w > resize(s_wmax, 9) then
                        v_wc := s_wmax;
                    else
                        v_wc := resize(v_w, 7);
                    end if;
                    w1_8 <= v_wc;
                end if;
                v_dk := signed(resize(not cl7, 11)) - signed(resize(s_t2, 11));
                en2_8 <= '0';
                if v_dk >= 0 then
                    en2_8 <= '1';
                    v_w := resize(s_wbase, 9)
                           + resize(shift_right(unsigned(resize(v_dk, 11)),
                                                s_wsh), 9);
                    if v_w > resize(s_wmax, 9) then
                        v_wc := s_wmax;
                    else
                        v_wc := resize(v_w, 7);
                    end if;
                    w2_8 <= v_wc;
                end if;
                v_dk := signed(resize(not cl7, 11)) - signed(resize(s_t3, 11));
                en3_8 <= '0';
                if v_dk >= 0 then
                    en3_8 <= '1';
                    v_w := resize(s_wbase, 9)
                           + resize(shift_right(unsigned(resize(v_dk, 11)),
                                                s_wsh), 9);
                    if v_w > resize(s_wmax, 9) then
                        v_wc := s_wmax;
                    else
                        v_wc := resize(v_w, 7);
                    end if;
                    w3_8 <= v_wc;
                end if;
                mag8 <= mag7;

                sj8  <= f_hash_b(sh7);
                d1_8 <= d1_7;
                d2_8 <= d2_7;
            end if;

            -- S9: ink decision. Engraving: union of the three stroke
            -- levels with a half-ink antialias band. Stipple: hash-gated
            -- diamond dots sized by tone. Contour ink on top (S9 switch).
            if av(9) = '1' then
                v_ink := "00";
                if s_stip = '0' then
                    if en1_8 = '1' then
                        if resize(dh8, 8) < resize(w1_8(6 downto 1), 8) then
                            v_ink := "10";
                        elsif resize(dh8, 8) < resize(w1_8(6 downto 1), 8) + 4
                              and v_ink = "00" then
                            v_ink := "01";
                        end if;
                    end if;
                    if en2_8 = '1' and v_ink /= "10" then
                        if resize(dx8, 8) < resize(w2_8(6 downto 1), 8) then
                            v_ink := "10";
                        elsif resize(dx8, 8) < resize(w2_8(6 downto 1), 8) + 4
                              and v_ink = "00" then
                            v_ink := "01";
                        end if;
                    end if;
                    if en3_8 = '1' and v_ink /= "10" then
                        if resize(dt8, 8) < resize(w3_8(6 downto 1), 8) then
                            v_ink := "10";
                        end if;
                    end if;
                else
                    -- stipple: cell fires by hash vs tone, dot radius by
                    -- tone (diamond metric |d1-c| + |d2-c|, c = half cell)
                    -- with the radius CAPPED below the half-cell so dots
                    -- stay separate instead of merging into chunks
                    v_hf := signed(resize(
                        shift_left(to_unsigned(4, 6), s_pitch_sh), 7));
                    v_rad := resize(w1_8(6 downto 2), 7) + 1;
                    v_cap := resize(shift_left(to_unsigned(4, 6),
                                               s_pitch_sh), 7) - 2;
                    if v_rad > v_cap then
                        v_rad := v_cap;
                    end if;
                    if en1_8 = '1'
                       and resize(sj8, 9) < w1_8 & "00" then
                        v_dd := resize(f_absu(signed(resize(d1_8, 7)) - v_hf), 7)
                                + resize(f_absu(signed(resize(d2_8, 7)) - v_hf), 7);
                        if v_dd < v_rad then
                            v_ink := "10";
                        end if;
                    end if;
                end if;
                ink9 <= v_ink;
                if s_edge_on = '1' and mag8 > resize(s_ethr, 13) then
                    edge9 <= '1';
                else
                    edge9 <= '0';
                end if;
            end if;

            -- S10: lanes fully assembled one stage before the mux
            if av(10) = '1' then
                -- ink lane
                ln_iy <= s_ink_y;
                ln_iu <= s_ink_u;
                ln_iv <= s_ink_v;
                -- paper lane (wash = half-sat video under-print + grain)
                if s_wash = '1' then
                    v_iy := signed(resize(to_unsigned(560, 10), 12))
                            + signed(resize(dyy(10)(9 downto 2), 12))
                            + signed(resize(dyy(10)(9 downto 3), 12));
                    ln_py <= f_cu10(v_iy);
                    ln_pu <= resize(shift_right(resize(du(10), 11)
                                    + to_unsigned(512, 11), 1), 10);
                    ln_pv <= resize(shift_right(resize(dv(10), 11)
                                    + to_unsigned(512, 11), 1), 10);
                else
                    ln_py <= s_pap_y - resize(sj8(2 downto 0), 10);
                    ln_pu <= s_pap_u;
                    ln_pv <= s_pap_v;
                end if;
                -- half-ink lane (paper/ink midpoint, cheap antialias)
                ln_hy <= ('0' & s_pap_y(9 downto 1))
                         + ('0' & s_ink_y(9 downto 1));
                ln_hu <= ('0' & s_pap_u(9 downto 1))
                         + ('0' & s_ink_u(9 downto 1));
                ln_hv <= ('0' & s_pap_v(9 downto 1))
                         + ('0' & s_ink_v(9 downto 1));
                ink10  <= ink9;
                edge10 <= edge9;
            end if;

            -- S11: priority select (contour > full ink > half > paper)
            if av(11) = '1' then
                if edge10 = '1' then
                    sel11 <= "11";
                elsif ink10 = "10" then
                    sel11 <= "10";
                elsif ink10 = "01" then
                    sel11 <= "01";
                else
                    sel11 <= "00";
                end if;
            end if;

            -- S12: mux -> output (Invert swaps ink and paper roles:
            -- scratchboard)
            if av(12) = '1' then
                case sel11 is
                    when "11" | "10" =>
                        if s_inv = '1' then
                            s_out_y <= ln_py;
                            s_out_u <= ln_pu;
                            s_out_v <= ln_pv;
                        else
                            s_out_y <= ln_iy;
                            s_out_u <= ln_iu;
                            s_out_v <= ln_iv;
                        end if;
                    when "01" =>
                        s_out_y <= ln_hy;
                        s_out_u <= ln_hu;
                        s_out_v <= ln_hv;
                    when others =>
                        if s_inv = '1' then
                            s_out_y <= ln_iy;
                            s_out_u <= ln_iu;
                            s_out_v <= ln_iv;
                        else
                            s_out_y <= ln_py;
                            s_out_u <= ln_pu;
                            s_out_v <= ln_pv;
                        end if;
                end case;
            end if;

            -- video delay pipes for the wash lane
            dyy(1) <= r0_y;
            du(1)  <= r0_u;
            dv(1)  <= r0_v;
            for i in 2 to 10 loop
                dyy(i) <= dyy(i - 1);
                du(i)  <= du(i - 1);
                dv(i)  <= dv(i - 1);
            end loop;

            -- per-line tremor jitter (2-step lstep sequencer)
            if s_lstep = 1 then
                -- tremor pattern boils at ~10 Hz when the plate is live
                -- (full-rate reseeding strobes at field rate); locked
                -- plate = static hand wobble
                s_lh_a  <= f_hash_a(resize(s_aline, 8),
                                    resize(s_dracc(11 downto 5), 8), x"B33F");
                s_lstep <= "10";
            elsif s_lstep = 2 then
                s_ljit8 <= f_hash_b(s_lh_a);
                s_lstep <= "00";
            end if;

            -- line bookkeeping: reload the stripe accumulators from the
            -- row bases, step the bases (doubled per line when interlaced
            -- so stroke angles are true in frame space)
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_x_count <= (others => '0');
                lrx <= (others => '0');
                lwx <= (others => '0');
                x_s6 <= (others => '0');
                s_lstep <= "01";

                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 1000 then
                        s_aline <= s_aline + 1;
                    end if;
                    for k in 0 to 7 loop
                        b_acc(k) <= b_acc(k) + unsigned(resize(s_sk2(k), 16));
                        p_acc(k) <= b_acc(k) + unsigned(resize(s_sk2(k), 16));
                    end loop;
                else
                    s_aline <= (others => '0');
                    -- field start: base = drift phase + field parity
                    -- half-step (interlace)
                    for k in 0 to 7 loop
                        if s_ilace = '1' and data_in.field_n = '1' then
                            b_acc(k) <= resize(s_dracc, 16)
                                        + unsigned(resize(s_sk(k), 16));
                            p_acc(k) <= resize(s_dracc, 16)
                                        + unsigned(resize(s_sk(k), 16));
                        else
                            b_acc(k) <= resize(s_dracc, 16);
                            p_acc(k) <= resize(s_dracc, 16);
                        end if;
                    end loop;
                end if;
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- luma line buffers: canonical 1W1R, read leads write by one stage
    ------------------------------------------------------------------------
    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            q1 <= lb1(to_integer(lrx));
            if av(2) = '1' then
                lb1(to_integer(lwx)) <= std_logic_vector(wb8);
            end if;
        end if;
    end process p_lb1;

    p_lb2 : process(clk)
    begin
        if rising_edge(clk) then
            q2 <= lb2(to_integer(lrx));
            if av(2) = '1' then
                lb2(to_integer(lwx)) <= q1;
            end if;
        end if;
    end process p_lb2;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture intaglio;
