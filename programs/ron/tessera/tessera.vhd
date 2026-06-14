-- tessera.vhd  (v7.0 — sprite-engine line-buffer rasteriser)
--
-- Up to C_MAX_TRI equilateral-triangle OUTLINES rain down the screen, each with
-- a RANDOM in-plane orientation (unique look), random x + vertical stagger, and
-- tumbling end-over-end at uniform speed, recycling from the top.
--
-- Architecture (the "sprite engine"):
--  * GEOMETRY (vblank): per triangle, rotate the base equilateral verts by a
--    random angle phi (sin/cos LUT) then foreshorten y by a TRIANGLE-WAVE factor
--    f_tau (linear -> uniform apparent tumble speed; cos would dwell/rush).
--    Produces 3 GENERAL edges (a=dy, b=-dx), the edge value at (xmin,0), the
--    bounding box and a threshold.  ALL multiplies go through ONE shared
--    multiplier, sequenced.  Results are pushed into a sprite FIFO.
--  * SPRITE FIFO: a recirculating shift register of per-triangle records.  The
--    fill engine always reads the HEAD (fixed position) -> no runtime-indexed
--    register arrays (those exploded LUT usage in the previous attempt).
--  * FILL (per displayed line): pop each record, and if the line is within its
--    y-band, walk its x bounding span with one shared edge tester, marking
--    outline pixels into the fill half of a ping-pong line buffer; step its
--    edge row for the next line; push it back.
--  * SCANOUT: read the display half of the line buffer (1-line latency; line 0
--    blanked), clear behind, colour mux.  Drawing is bounded to each span so a
--    flat triangle draws a short segment (no full-screen line).
--
-- Outline-only.  Controls: K1 Outline Hue, K3 BG Hue, K4 Fall Speed,
-- K5 Count (1..N), K6 Tumble Speed.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture tessera of program_top is

    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_MAX_VAL    : unsigned(9 downto 0) := to_unsigned(1023, 10);
    constant C_BG_LUMA      : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_OUTLINE_LUMA : unsigned(9 downto 0) := to_unsigned(900, 10);
    constant C_FILL_LUMA    : unsigned(9 downto 0) := to_unsigned(640, 10);

    -- line-buffer pixel codes
    constant LB_BG    : std_logic_vector(1 downto 0) := "00";
    constant LB_LINE  : std_logic_vector(1 downto 0) := "01";  -- outline
    constant LB_SOLID : std_logic_vector(1 downto 0) := "10";  -- solid fill (back side)
    constant LB_VIDEO : std_logic_vector(1 downto 0) := "11";  -- video fill (front side)

    constant C_MAX_TRI       : integer := 10;
    constant C_OUTLINE_SHIFT : integer := 1;
    constant C_LBW           : integer := 2048;
    constant C_LAW           : integer := 11;

    constant C_GW : integer := 14;
    subtype  t_g  is signed(C_GW - 1 downto 0);
    constant C_EW : integer := 21;
    subtype  t_e  is signed(C_EW - 1 downto 0);
    subtype  t_ab is signed(10 downto 0);   -- edge deltas / thresholds (small)
    constant C_FW : integer := 18;           -- fill working edge value (in-band, small)
    subtype  t_fc is signed(C_FW - 1 downto 0);

    --------------------------------------------------------------------------
    -- Per-index hashes (elaboration-time; use HIGH bits).
    --------------------------------------------------------------------------
    type t_hash_arr is array (0 to C_MAX_TRI - 1) of unsigned(15 downto 0);
    function hash16(idx : integer; seed : integer) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := to_unsigned(((idx + 1) * seed) mod 65536, 16);
        h := h xor shift_left(h, 5);
        h := h xor shift_right(h, 11);
        h := h xor shift_left(h, 7);
        return h;
    end function;
    function gen_hashes(seed : integer) return t_hash_arr is
        variable a : t_hash_arr;
    begin
        for i in 0 to C_MAX_TRI - 1 loop a(i) := hash16(i, seed); end loop;
        return a;
    end function;
    constant C_HASH_X   : t_hash_arr := gen_hashes(40503);
    constant C_HASH_PHI : t_hash_arr := gen_hashes(12911);
    constant C_HASH_Y   : t_hash_arr := gen_hashes(53219);
    constant C_HASH_T   : t_hash_arr := gen_hashes(28019);

    --------------------------------------------------------------------------
    -- Sprite record (one per triangle) + recirculating FIFO.
    --------------------------------------------------------------------------
    type t_sprite is record
        a0, a1, a2 : t_ab;         -- dy_e (x step)
        b0, b1, b2 : t_ab;         -- -dx_e (y step)
        e0, e1, e2 : t_e;          -- edge value at (xmin, current fill line)
        xmin, xmax : t_g;
        ymin, ymax : t_g;
        thr0, thr1, thr2 : t_ab;   -- per-edge outline threshold (length-scaled)
        valid      : std_logic;
        front      : std_logic;    -- '1' = front face toward viewer (video side)
    end record;
    constant C_SPRITE_ZERO : t_sprite := (
        a0=>(others=>'0'), a1=>(others=>'0'), a2=>(others=>'0'),
        b0=>(others=>'0'), b1=>(others=>'0'), b2=>(others=>'0'),
        e0=>(others=>'0'), e1=>(others=>'0'), e2=>(others=>'0'),
        xmin=>(others=>'0'), xmax=>(others=>'0'),
        ymin=>(others=>'0'), ymax=>(others=>'0'),
        thr0=>(others=>'0'), thr1=>(others=>'0'), thr2=>(others=>'0'),
        valid=>'0', front=>'0');
    type t_fifo is array (0 to C_MAX_TRI - 1) of t_sprite;
    signal s_fifo  : t_fifo := (others => C_SPRITE_ZERO);
    signal s_stage : t_sprite := C_SPRITE_ZERO;
    signal s_push  : std_logic := '0';

    --------------------------------------------------------------------------
    -- Parameters / timing
    --------------------------------------------------------------------------
    signal s_outline_hue : unsigned(9 downto 0);
    signal s_bg_hue      : unsigned(9 downto 0);
    signal s_fill_hue    : unsigned(9 downto 0);
    signal s_fall_pot    : unsigned(9 downto 0);
    signal s_count_pot   : unsigned(9 downto 0);
    signal s_tumble_pot  : unsigned(9 downto 0);
    signal s_video_en    : std_logic;

    signal s_timing  : t_video_timing_port;
    signal s_h_count : unsigned(11 downto 0);
    signal s_v_count : unsigned(11 downto 0);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_vsync_prev  : std_logic := '1';
    signal s_vsync_pulse : std_logic := '0';

    signal s_initialized : std_logic := '0';
    signal s_tumble_acc  : unsigned(9 downto 0) := (others => '0');
    signal s_fall_acc    : t_g := (others => '0');
    signal s_rangey      : t_g := (others => '0');
    signal s_count       : integer range 1 to C_MAX_TRI := 6;
    signal s_R, s_half, s_third : t_g := (others => '0');

    --------------------------------------------------------------------------
    -- Shared sin/cos LUT + multiplier
    --------------------------------------------------------------------------
    signal s_lut_angle : std_logic_vector(9 downto 0) := (others => '0');
    signal s_lut_sin   : signed(9 downto 0);
    signal s_lut_cos   : signed(9 downto 0);
    signal s_sin_r, s_cos_r : signed(10 downto 0) := (others => '0');

    signal s_mul_a, s_mul_b     : signed(13 downto 0) := (others => '0');
    signal s_mul_a_r, s_mul_b_r : signed(13 downto 0) := (others => '0');
    signal s_pp_lo              : signed(21 downto 0) := (others => '0');
    signal s_pp_hi              : signed(20 downto 0) := (others => '0');
    signal s_mul_p              : signed(27 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Geometry engine
    --------------------------------------------------------------------------
    type t_gstate is (G_IDLE, G_PHI, G_PHIW, G_FTAU, G_FTAU2, G_P, G_PW,
                      G_VCOMB1, G_VCOMB2, G_VCOMB3, G_STORE);
    signal s_gst   : t_gstate := G_IDLE;
    signal s_gtri  : integer range 0 to C_MAX_TRI := 0;
    signal s_pcnt  : integer range 0 to 19 := 0;
    signal s_pwait : integer range 0 to 7 := 0;
    signal s_geng_start : std_logic := '0';
    signal s_ftau  : signed(10 downto 0) := (others => '0');
    signal s_tau_acc : unsigned(9 downto 0) := (others => '0');

    signal s_cx   : t_g := (others => '0');
    signal s_cy   : t_g := (others => '0');
    signal s_cy1, s_cy2, s_cy3, s_cy4, s_cy5 : t_g := (others => '0');  -- pipelined cy + wrap
    signal s_rsin, s_rcos, s_hc, s_tsin, s_hs, s_tcos : t_g := (others => '0');
    signal s_sy0, s_sy1, s_sy2 : t_g := (others => '0');
    signal s_wx0, s_wx1, s_wx2 : t_g := (others => '0');
    signal s_wy0, s_wy1, s_wy2 : t_g := (others => '0');
    signal s_dx0, s_dx1, s_dx2 : t_g := (others => '0');
    signal s_dy0, s_dy1, s_dy2 : t_g := (others => '0');
    signal s_xmin, s_xmax, s_ymin, s_ymax : t_g := (others => '0');
    signal s_bxmn, s_bxmx, s_bymn, s_bymx : t_g := (others => '0');  -- pairwise bbox
    signal s_c0, s_c1, s_c2 : t_e := (others => '0');
    signal s_tmp : t_e := (others => '0');

    --------------------------------------------------------------------------
    -- Fill engine
    --------------------------------------------------------------------------
    type t_fstate is (F_IDLE, F_HEAD, F_WALK, F_DRAIN, F_DONE);
    signal s_fst  : t_fstate := F_IDLE;
    signal s_fn   : integer range 0 to C_MAX_TRI := 0;
    signal s_fx   : signed(C_GW - 1 downto 0) := (others => '0');
    signal s_fc0, s_fc1, s_fc2 : t_fc := (others => '0');
    signal s_fwe  : std_logic := '0';
    signal s_fcode : std_logic_vector(1 downto 0) := "00";
    signal s_fwaddr : unsigned(C_LAW - 1 downto 0) := (others => '0');
    -- fill test pipeline (1 stage: compare/near in cycle N, wedge+write N+1)
    signal s_pn  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_psg : std_logic_vector(2 downto 0) := (others => '0');
    signal s_pwa : unsigned(C_LAW - 1 downto 0) := (others => '0');
    signal s_pv  : std_logic := '0';

    --------------------------------------------------------------------------
    -- Line buffers (ping-pong, 2-bit code, inferred BRAM)
    --------------------------------------------------------------------------
    type t_lb is array (0 to C_LBW - 1) of std_logic_vector(1 downto 0);
    signal s_lb0, s_lb1 : t_lb := (others => "00");
    signal s_rd0, s_rd1 : std_logic_vector(1 downto 0) := "00";
    signal s_hc_d1 : unsigned(11 downto 0) := (others => '0');
    -- vsync buffer wipe (clears stale prev-frame content -> no top-line wrap)
    signal s_clr_addr : unsigned(C_LAW - 1 downto 0) := (others => '0');
    signal s_clr_busy : std_logic := '1';

    signal s_disp_code : std_logic_vector(1 downto 0) := "00";
    signal s_par1 : std_logic := '0';
    signal s_v0_1, s_v0_2 : std_logic := '0';
    signal s_o_y : unsigned(9 downto 0) := (others => '0');
    signal s_o_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_o_v : unsigned(9 downto 0) := C_CHROMA_MID;

    -- incoming-video delay to align with the scanout colour mux (front face fill)
    type t_vdel is array (0 to 3) of std_logic_vector(9 downto 0);
    signal s_vy, s_vu, s_vv : t_vdel := (others => (others => '0'));

    constant C_RENDER_DELAY : integer := 5;
    type t_sync_pipe is array (0 to C_RENDER_DELAY - 1) of std_logic_vector(3 downto 0);
    signal s_sync_pipe : t_sync_pipe := (others => (others => '0'));
    signal s_io_0 : t_video_stream_yuv444_30b;
    signal s_io_1 : t_video_stream_yuv444_30b;

begin

    s_outline_hue <= unsigned(registers_in(0));
    s_fill_hue    <= unsigned(registers_in(1));
    s_bg_hue      <= unsigned(registers_in(2));
    s_fall_pot    <= unsigned(registers_in(3));
    s_count_pot   <= unsigned(registers_in(4));
    s_tumble_pot  <= unsigned(registers_in(5));
    s_video_en    <= registers_in(6)(4);   -- P11 toggle

    p_measure_resolution : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then s_measured_h <= s_h_pixel_counter; end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then s_measured_v <= s_v_line_counter; end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    pixel_counter_inst : entity work.pixel_counter
        port map (clk => clk, timing => s_timing,
                  h_count => s_h_count, v_count => s_v_count);

    p_vsync_edge : process(clk)
    begin
        if rising_edge(clk) then
            s_vsync_prev <= s_timing.vsync_n;
            if s_vsync_prev = '0' and s_timing.vsync_n = '1' then
                s_vsync_pulse <= '1';
            else
                s_vsync_pulse <= '0';
            end if;
        end if;
    end process;

    sincos_inst : entity work.sin_cos_full_lut_10x10
        port map (angle_in => s_lut_angle, sin_out => s_lut_sin, cos_out => s_lut_cos);

    p_lut_reg : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_r <= resize(s_lut_sin, 11);
            s_cos_r <= resize(s_lut_cos, 11);
        end if;
    end process;

    p_mul_inst : process(clk)
        variable v_b_slv : std_logic_vector(13 downto 0);
        variable v_b_lo  : signed(7 downto 0);
        variable v_b_hi  : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            s_mul_a_r <= s_mul_a;
            s_mul_b_r <= s_mul_b;
            v_b_slv := std_logic_vector(s_mul_b_r);
            v_b_lo  := signed('0' & v_b_slv(6 downto 0));
            v_b_hi  := signed(v_b_slv(13 downto 7));
            s_pp_lo <= s_mul_a_r * v_b_lo;
            s_pp_hi <= s_mul_a_r * v_b_hi;
            s_mul_p <= resize(s_pp_lo, 28) + shift_left(resize(s_pp_hi, 28), 7);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Animation: sizes, count, tumble + shared fall accumulators.
    --------------------------------------------------------------------------
    p_anim : process(clk)
        variable v_h, v_r, v_half : t_g;
        variable v_fall, v_next   : t_g;
        variable v_count          : integer;
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                s_initialized <= '1';
                v_h := signed(resize(shift_right(s_measured_v, 4)
                            + shift_right(s_measured_v, 5)
                            + shift_right(s_measured_v, 8), C_GW));
                v_r := shift_right(v_h, 1) + shift_right(v_h, 3)
                     + shift_right(v_h, 5) + shift_right(v_h, 7);
                s_R     <= v_r;
                s_third <= shift_right(v_r, 1);
                v_half := shift_right(v_h, 1) + shift_right(v_h, 4)
                        + shift_right(v_h, 6);
                s_half  <= v_half;
                s_rangey <= signed(resize(s_measured_v, C_GW)) + shift_left(v_h, 1);
                s_tumble_acc <= s_tumble_acc + (shift_right(s_tumble_pot, 6) + 1);
                v_count := to_integer(shift_right(s_count_pot * to_unsigned(C_MAX_TRI, 10), 10)) + 1;
                if v_count > C_MAX_TRI then v_count := C_MAX_TRI; end if;
                s_count <= v_count;
                v_fall := resize(signed('0' & std_logic_vector(shift_right(s_fall_pot, 6))), C_GW)
                        + to_signed(1, C_GW);
                if s_initialized = '0' then
                    s_fall_acc <= (others => '0');
                else
                    v_next := s_fall_acc + v_fall;
                    if v_next >= s_rangey then v_next := v_next - s_rangey; end if;
                    s_fall_acc <= v_next;
                end if;
            end if;
        end if;
    end process;

    p_geng_start : process(clk)
    begin
        if rising_edge(clk) then s_geng_start <= s_vsync_pulse; end if;
    end process;

    --------------------------------------------------------------------------
    -- Geometry engine: per triangle -> sprite record (pushed via s_stage/s_push).
    --------------------------------------------------------------------------
    p_geom : process(clk)
        variable v_a, v_b : signed(13 downto 0);
        variable v_py1, v_py2 : t_g;
        variable v_tau    : unsigned(9 downto 0);
        variable v_frac   : signed(10 downto 0);
        variable v_mnx, v_mxx, v_mny, v_mxy : t_g;
        variable v_adx, v_ady, v_len : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            s_push <= '0';
            case s_gst is
                when G_IDLE =>
                    if s_geng_start = '1' then
                        s_gtri <= 0;
                        s_gst  <= G_PHI;
                    end if;

                when G_PHI =>
                    s_lut_angle <= std_logic_vector(C_HASH_PHI(s_gtri)(15 downto 6));
                    s_pwait <= 0;
                    s_gst   <= G_PHIW;

                when G_PHIW =>
                    -- cy = (shared fall + per-tri stagger) wrapped - margin(R),
                    -- pipelined one op per cycle (was a 5-deep chain at 53 MHz)
                    -- and overlapped with the LUT settle.
                    s_cy1 <= s_fall_acc
                           + signed(resize(C_HASH_Y(s_gtri)(15 downto 6), C_GW));
                    if s_cy1 >= s_rangey then s_cy2 <= s_cy1 - s_rangey;
                    else                      s_cy2 <= s_cy1; end if;
                    if s_cy2 >= s_rangey then s_cy3 <= s_cy2 - s_rangey;
                    else                      s_cy3 <= s_cy2; end if;
                    if s_cy3 >= s_rangey then s_cy4 <= s_cy3 - s_rangey;
                    else                      s_cy4 <= s_cy3; end if;
                    if s_cy4 >= s_rangey then s_cy5 <= s_cy4 - s_rangey;
                    else                      s_cy5 <= s_cy4; end if;
                    s_cy <= s_cy5 - s_R;
                    if s_pwait >= 6 then
                        s_gst <= G_FTAU;
                    else
                        s_pwait <= s_pwait + 1;
                    end if;

                when G_FTAU =>
                    -- pipelined: (1) tumble phase = shared accumulator + hash
                    s_tau_acc <= s_tumble_acc + C_HASH_T(s_gtri)(15 downto 6);
                    s_gst <= G_FTAU2;

                when G_FTAU2 =>
                    -- (2) triangle-wave foreshorten f_tau (Q9, +-512), uniform speed
                    v_frac := signed(resize(s_tau_acc(7 downto 0), 11));
                    case s_tau_acc(9 downto 8) is
                        when "00"   => s_ftau <= to_signed(512, 11) - shift_left(v_frac, 1);
                        when "01"   => s_ftau <= -shift_left(v_frac, 1);
                        when "10"   => s_ftau <= to_signed(-512, 11) + shift_left(v_frac, 1);
                        when others => s_ftau <= shift_left(v_frac, 1);
                    end case;
                    s_pcnt  <= 0;
                    s_pwait <= 0;
                    s_gst   <= G_P;

                when G_P =>
                    case s_pcnt is
                        when 0 => v_a := signed('0' & std_logic_vector(C_HASH_X(s_gtri)(15 downto 3)));
                                  v_b := resize(signed('0' & std_logic_vector(s_measured_h)), 14);
                        when 1 => v_a := resize(s_R, 14);     v_b := resize(s_sin_r, 14);
                        when 2 => v_a := resize(s_R, 14);     v_b := resize(s_cos_r, 14);
                        when 3 => v_a := resize(s_half, 14);  v_b := resize(s_cos_r, 14);
                        when 4 => v_a := resize(s_third, 14); v_b := resize(s_sin_r, 14);
                        when 5 => v_a := resize(s_half, 14);  v_b := resize(s_sin_r, 14);
                        when 6 => v_a := resize(s_third, 14); v_b := resize(s_cos_r, 14);
                        when 7 => v_a := resize(-s_rcos, 14); v_b := resize(s_ftau, 14);
                        when 8 => v_py1 := s_tcos - s_hs; v_a := resize(v_py1, 14); v_b := resize(s_ftau, 14);
                        when 9 => v_py2 := s_tcos + s_hs; v_a := resize(v_py2, 14); v_b := resize(s_ftau, 14);
                        when 10 => v_a := resize(s_wx0, 14); v_b := resize(s_dy0, 14);
                        when 11 => v_a := resize(s_wy0, 14); v_b := resize(s_dx0, 14);
                        when 12 => v_a := resize(s_wx1, 14); v_b := resize(s_dy1, 14);
                        when 13 => v_a := resize(s_wy1, 14); v_b := resize(s_dx1, 14);
                        when 14 => v_a := resize(s_wx2, 14); v_b := resize(s_dy2, 14);
                        when 15 => v_a := resize(s_wy2, 14); v_b := resize(s_dx2, 14);
                        when 16 => v_a := resize(s_dy0, 14); v_b := resize(s_xmin, 14);
                        when 17 => v_a := resize(s_dy1, 14); v_b := resize(s_xmin, 14);
                        when others => v_a := resize(s_dy2, 14); v_b := resize(s_xmin, 14);
                    end case;
                    s_mul_a <= v_a;
                    s_mul_b <= v_b;
                    s_pwait <= 0;
                    s_gst   <= G_PW;

                when G_PW =>
                    if s_pwait >= 3 then
                        case s_pcnt is
                            when 0 => s_cx   <= resize(shift_right(s_mul_p, 13), C_GW);
                            when 1 => s_rsin <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 2 => s_rcos <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 3 => s_hc   <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 4 => s_tsin <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 5 => s_hs   <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 6 => s_tcos <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 7 => s_sy0  <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 8 => s_sy1  <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 9 => s_sy2  <= resize(shift_right(s_mul_p, 9), C_GW);
                            when 10 => s_tmp <= resize(s_mul_p, C_EW);
                            when 11 => s_c0  <= -s_tmp + resize(s_mul_p, C_EW);
                            when 12 => s_tmp <= resize(s_mul_p, C_EW);
                            when 13 => s_c1  <= -s_tmp + resize(s_mul_p, C_EW);
                            when 14 => s_tmp <= resize(s_mul_p, C_EW);
                            when 15 => s_c2  <= -s_tmp + resize(s_mul_p, C_EW);
                            when 16 => s_stage.e0 <= resize(s_mul_p, C_EW) + s_c0;
                            when 17 => s_stage.e1 <= resize(s_mul_p, C_EW) + s_c1;
                            when others => s_stage.e2 <= resize(s_mul_p, C_EW) + s_c2;
                        end case;

                        if s_pcnt = 9 then
                            s_gst <= G_VCOMB1;
                        elsif s_pcnt = 18 then
                            s_gst <= G_STORE;
                        else
                            s_pcnt <= s_pcnt + 1;
                            s_gst  <= G_P;
                        end if;
                    else
                        s_pwait <= s_pwait + 1;
                    end if;

                when G_VCOMB1 =>
                    -- stage 1: register world verts (x not foreshortened)
                    s_wx0 <= s_cx + s_rsin;          s_wy0 <= s_cy + s_sy0;
                    s_wx1 <= s_cx - s_hc - s_tsin;   s_wy1 <= s_cy + s_sy1;
                    s_wx2 <= s_cx + s_hc - s_tsin;   s_wy2 <= s_cy + s_sy2;
                    s_gst <= G_VCOMB2;

                when G_VCOMB2 =>
                    -- stage 2: edges + PAIRWISE bbox (vert0 vs vert1)
                    s_dx0 <= s_wx1 - s_wx0; s_dy0 <= s_wy1 - s_wy0;
                    s_dx1 <= s_wx2 - s_wx1; s_dy1 <= s_wy2 - s_wy1;
                    s_dx2 <= s_wx0 - s_wx2; s_dy2 <= s_wy0 - s_wy2;
                    if s_wx0 <= s_wx1 then s_bxmn <= s_wx0; s_bxmx <= s_wx1;
                    else                   s_bxmn <= s_wx1; s_bxmx <= s_wx0; end if;
                    if s_wy0 <= s_wy1 then s_bymn <= s_wy0; s_bymx <= s_wy1;
                    else                   s_bymn <= s_wy1; s_bymx <= s_wy0; end if;
                    s_gst <= G_VCOMB3;

                when G_VCOMB3 =>
                    -- stage 3: fold in vert2, clamp xmin >= 0
                    if s_wx2 < s_bxmn then v_mnx := s_wx2; else v_mnx := s_bxmn; end if;
                    if s_wx2 > s_bxmx then v_mxx := s_wx2; else v_mxx := s_bxmx; end if;
                    if s_wy2 < s_bymn then v_mny := s_wy2; else v_mny := s_bymn; end if;
                    if s_wy2 > s_bymx then v_mxy := s_wy2; else v_mxy := s_bymx; end if;
                    if v_mnx < to_signed(0, C_GW) then v_mnx := (others => '0'); end if;
                    s_xmin <= v_mnx; s_xmax <= v_mxx;
                    s_ymin <= v_mny; s_ymax <= v_mxy;
                    s_pcnt <= 10;
                    s_pwait <= 0;
                    s_gst  <= G_P;

                when G_STORE =>
                    s_stage.a0 <= resize(s_dy0, 11); s_stage.a1 <= resize(s_dy1, 11); s_stage.a2 <= resize(s_dy2, 11);
                    s_stage.b0 <= resize(-s_dx0, 11); s_stage.b1 <= resize(-s_dx1, 11); s_stage.b2 <= resize(-s_dx2, 11);
                    s_stage.xmin <= s_xmin; s_stage.xmax <= s_xmax;
                    s_stage.ymin <= s_ymin; s_stage.ymax <= s_ymax;
                    -- per-edge threshold = (Manhattan length) * outline width, so
                    -- the outline stays a constant pixel width as edges
                    -- foreshorten differently while tumbling.
                    v_adx := unsigned(abs(resize(s_dx0, 14)));
                    v_ady := unsigned(abs(resize(s_dy0, 14)));
                    if v_adx > v_ady then v_len := v_adx + shift_right(v_ady, 1);
                    else                  v_len := v_ady + shift_right(v_adx, 1); end if;
                    s_stage.thr0 <= signed(resize(shift_left(v_len, C_OUTLINE_SHIFT), 11));
                    v_adx := unsigned(abs(resize(s_dx1, 14)));
                    v_ady := unsigned(abs(resize(s_dy1, 14)));
                    if v_adx > v_ady then v_len := v_adx + shift_right(v_ady, 1);
                    else                  v_len := v_ady + shift_right(v_adx, 1); end if;
                    s_stage.thr1 <= signed(resize(shift_left(v_len, C_OUTLINE_SHIFT), 11));
                    v_adx := unsigned(abs(resize(s_dx2, 14)));
                    v_ady := unsigned(abs(resize(s_dy2, 14)));
                    if v_adx > v_ady then v_len := v_adx + shift_right(v_ady, 1);
                    else                  v_len := v_ady + shift_right(v_adx, 1); end if;
                    s_stage.thr2 <= signed(resize(shift_left(v_len, C_OUTLINE_SHIFT), 11));
                    if s_gtri < s_count then s_stage.valid <= '1'; else s_stage.valid <= '0'; end if;
                    -- front face toward viewer while the tumble factor is >= 0
                    if s_ftau >= 0 then s_stage.front <= '1'; else s_stage.front <= '0'; end if;
                    s_push <= '1';

                    if s_gtri = C_MAX_TRI - 1 then
                        s_gst <= G_IDLE;
                    else
                        s_gtri <= s_gtri + 1;
                        s_gst  <= G_PHI;
                    end if;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sprite FIFO + fill engine (single owner of s_fifo).
    --   vblank: push staged records.
    --   active: each line, pop head, walk span if in band, step row, push back.
    --------------------------------------------------------------------------
    p_sprite : process(clk)
        variable v_hd  : t_sprite;
        variable v_vc  : t_g;
        variable v_n0, v_n1, v_n2 : std_logic;
        variable v_out, v_in : std_logic;
        variable v_t0, v_t1, v_t2 : t_fc;
    begin
        if rising_edge(clk) then
            s_fwe <= '0';

            if s_push = '1' then
                -- shift in a new record at the tail
                for i in 0 to C_MAX_TRI - 2 loop
                    s_fifo(i) <= s_fifo(i + 1);
                end loop;
                s_fifo(C_MAX_TRI - 1) <= s_stage;
            else
                case s_fst is
                    when F_IDLE =>
                        if s_timing.avid_start = '1' then
                            s_fn  <= 0;
                            s_fst <= F_HEAD;
                        end if;

                    when F_HEAD =>
                        v_hd := s_fifo(0);
                        v_vc := signed(resize(s_v_count, C_GW));
                        s_fc0 <= resize(v_hd.e0, C_FW);
                        s_fc1 <= resize(v_hd.e1, C_FW);
                        s_fc2 <= resize(v_hd.e2, C_FW);
                        if v_hd.xmin < to_signed(0, C_GW) then
                            s_fx <= (others => '0');
                        else
                            s_fx <= v_hd.xmin;
                        end if;
                        s_pv <= '0';   -- test pipeline empty
                        if (v_hd.valid = '1') and (v_vc >= v_hd.ymin) and (v_vc <= v_hd.ymax) then
                            s_fst <= F_WALK;
                        else
                            s_fst <= F_DONE;
                        end if;

                    when F_WALK =>
                        v_hd := s_fifo(0);
                        -- (a) emit the write for the pixel tested LAST cycle:
                        --   outline -> LB_LINE; else inside (signs agree) gets
                        --   filled when P11 is on (video on the front face,
                        --   solid on the back); else nothing (background).
                        v_out := '0';
                        if s_pn(0) = '1' and s_psg(1) = s_psg(2) then v_out := '1'; end if;
                        if s_pn(1) = '1' and s_psg(0) = s_psg(2) then v_out := '1'; end if;
                        if s_pn(2) = '1' and s_psg(0) = s_psg(1) then v_out := '1'; end if;
                        if (s_psg(0) = s_psg(1)) and (s_psg(1) = s_psg(2)) then v_in := '1'; else v_in := '0'; end if;
                        s_fwaddr <= s_pwa;
                        if v_out = '1' then
                            s_fcode <= LB_LINE; s_fwe <= s_pv;
                        elsif v_in = '1' and s_video_en = '1' then
                            if v_hd.front = '1' then s_fcode <= LB_VIDEO;
                            else                     s_fcode <= LB_SOLID; end if;
                            s_fwe <= s_pv;
                        else
                            s_fcode <= LB_BG; s_fwe <= '0';
                        end if;
                        -- (b) test THIS pixel (two parallel signed compares per
                        --     edge, using each edge's own threshold), register
                        --     near/sign/addr for next cycle's write
                        v_t0 := resize(v_hd.thr0, C_FW);
                        v_t1 := resize(v_hd.thr1, C_FW);
                        v_t2 := resize(v_hd.thr2, C_FW);
                        if (s_fc0 < v_t0) and (s_fc0 > -v_t0) then v_n0 := '1'; else v_n0 := '0'; end if;
                        if (s_fc1 < v_t1) and (s_fc1 > -v_t1) then v_n1 := '1'; else v_n1 := '0'; end if;
                        if (s_fc2 < v_t2) and (s_fc2 > -v_t2) then v_n2 := '1'; else v_n2 := '0'; end if;
                        s_pn(0) <= v_n0; s_pn(1) <= v_n1; s_pn(2) <= v_n2;
                        s_psg(0) <= s_fc0(C_FW-1); s_psg(1) <= s_fc1(C_FW-1); s_psg(2) <= s_fc2(C_FW-1);
                        s_pwa <= unsigned(s_fx(C_LAW - 1 downto 0));
                        s_pv  <= '1';
                        -- (c) step
                        s_fc0 <= s_fc0 + resize(v_hd.a0, C_FW);
                        s_fc1 <= s_fc1 + resize(v_hd.a1, C_FW);
                        s_fc2 <= s_fc2 + resize(v_hd.a2, C_FW);
                        if s_fx >= v_hd.xmax then
                            s_fst <= F_DRAIN;
                        else
                            s_fx <= s_fx + 1;
                        end if;

                    when F_DRAIN =>
                        -- write the final pixel still in the test pipeline
                        v_hd := s_fifo(0);
                        v_out := '0';
                        if s_pn(0) = '1' and s_psg(1) = s_psg(2) then v_out := '1'; end if;
                        if s_pn(1) = '1' and s_psg(0) = s_psg(2) then v_out := '1'; end if;
                        if s_pn(2) = '1' and s_psg(0) = s_psg(1) then v_out := '1'; end if;
                        if (s_psg(0) = s_psg(1)) and (s_psg(1) = s_psg(2)) then v_in := '1'; else v_in := '0'; end if;
                        s_fwaddr <= s_pwa;
                        if v_out = '1' then
                            s_fcode <= LB_LINE; s_fwe <= s_pv;
                        elsif v_in = '1' and s_video_en = '1' then
                            if v_hd.front = '1' then s_fcode <= LB_VIDEO;
                            else                     s_fcode <= LB_SOLID; end if;
                            s_fwe <= s_pv;
                        else
                            s_fcode <= LB_BG; s_fwe <= '0';
                        end if;
                        s_pv     <= '0';
                        s_fst    <= F_DONE;

                    when F_DONE =>
                        -- recirculate head, stepping its row for the next line
                        v_hd := s_fifo(0);
                        v_hd.e0 := v_hd.e0 + resize(v_hd.b0, C_EW);
                        v_hd.e1 := v_hd.e1 + resize(v_hd.b1, C_EW);
                        v_hd.e2 := v_hd.e2 + resize(v_hd.b2, C_EW);
                        for i in 0 to C_MAX_TRI - 2 loop
                            s_fifo(i) <= s_fifo(i + 1);
                        end loop;
                        s_fifo(C_MAX_TRI - 1) <= v_hd;

                        if s_fn = C_MAX_TRI - 1 then
                            s_fst <= F_IDLE;
                        else
                            s_fn  <= s_fn + 1;
                            s_fst <= F_HEAD;
                        end if;
                end case;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Line buffers.  Fill = buffer[v_count(0)]; display = the other.
    --------------------------------------------------------------------------
    -- Sweep both line buffers to 0 during vsync (fill + clear-behind are idle
    -- then), so no stale previous-frame content survives into the top lines.
    p_clear : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_clr_addr <= (others => '0');
                s_clr_busy <= '1';
            elsif s_clr_busy = '1' then
                if s_clr_addr = to_unsigned(C_LBW - 1, C_LAW) then
                    s_clr_busy <= '0';
                else
                    s_clr_addr <= s_clr_addr + 1;
                end if;
            end if;
        end if;
    end process;

    p_lb0 : process(clk)
        variable v_we : std_logic;
        variable v_wa : unsigned(C_LAW - 1 downto 0);
        variable v_wd : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            if s_clr_busy = '1' then
                v_we := '1'; v_wa := s_clr_addr; v_wd := LB_BG;
            elsif s_v_count(0) = '0' then
                v_we := s_fwe; v_wa := s_fwaddr; v_wd := s_fcode;
            else
                v_we := s_timing.avid; v_wa := s_hc_d1(C_LAW - 1 downto 0); v_wd := LB_BG;
            end if;
            if v_we = '1' then s_lb0(to_integer(v_wa)) <= v_wd; end if;
            s_rd0 <= s_lb0(to_integer(s_h_count(C_LAW - 1 downto 0)));
        end if;
    end process;

    p_lb1 : process(clk)
        variable v_we : std_logic;
        variable v_wa : unsigned(C_LAW - 1 downto 0);
        variable v_wd : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            if s_clr_busy = '1' then
                v_we := '1'; v_wa := s_clr_addr; v_wd := LB_BG;
            elsif s_v_count(0) = '1' then
                v_we := s_fwe; v_wa := s_fwaddr; v_wd := s_fcode;
            else
                v_we := s_timing.avid; v_wa := s_hc_d1(C_LAW - 1 downto 0); v_wd := LB_BG;
            end if;
            if v_we = '1' then s_lb1(to_integer(v_wa)) <= v_wd; end if;
            s_rd1 <= s_lb1(to_integer(s_h_count(C_LAW - 1 downto 0)));
        end if;
    end process;

    -- incoming video delayed to align with the scanout colour mux
    p_vdelay : process(clk)
    begin
        if rising_edge(clk) then
            s_vy(0) <= data_in.y; s_vu(0) <= data_in.u; s_vv(0) <= data_in.v;
            for i in 1 to 3 loop
                s_vy(i) <= s_vy(i - 1);
                s_vu(i) <= s_vu(i - 1);
                s_vv(i) <= s_vv(i - 1);
            end loop;
        end if;
    end process;

    p_hcd : process(clk)
    begin
        if rising_edge(clk) then s_hc_d1 <= s_h_count; end if;
    end process;

    --------------------------------------------------------------------------
    -- Scanout: display buffer = NOT being filled = ~v_count(0); read latency 1.
    --------------------------------------------------------------------------
    p_scanout : process(clk)
        variable v_code : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            s_par1 <= s_v_count(0);
            if s_v_count = 0 then s_v0_1 <= '1'; else s_v0_1 <= '0'; end if;
            s_v0_2 <= s_v0_1;
            if s_par1 = '0' then s_disp_code <= s_rd1; else s_disp_code <= s_rd0; end if;

            if s_v0_2 = '1' then v_code := LB_BG; else v_code := s_disp_code; end if;
            case v_code is
                when LB_LINE =>
                    s_o_y <= C_OUTLINE_LUMA;
                    s_o_u <= s_outline_hue;
                    s_o_v <= C_MAX_VAL - s_outline_hue;
                when LB_VIDEO =>
                    s_o_y <= unsigned(s_vy(3));
                    s_o_u <= unsigned(s_vu(3));
                    s_o_v <= unsigned(s_vv(3));
                when LB_SOLID =>
                    s_o_y <= C_FILL_LUMA;
                    s_o_u <= s_fill_hue;
                    s_o_v <= C_MAX_VAL - s_fill_hue;
                when others =>
                    s_o_y <= C_BG_LUMA;
                    s_o_u <= s_bg_hue;
                    s_o_v <= C_MAX_VAL - s_bg_hue;
            end case;
        end if;
    end process;

    p_sync_pipe : process(clk)
    begin
        if rising_edge(clk) then
            s_sync_pipe(0) <= data_in.field_n & data_in.avid &
                              data_in.vsync_n & data_in.hsync_n;
            for i in 1 to C_RENDER_DELAY - 1 loop
                s_sync_pipe(i) <= s_sync_pipe(i - 1);
            end loop;
        end if;
    end process;

    p_io_align : process(clk)
    begin
        if rising_edge(clk) then
            s_io_0.y       <= std_logic_vector(s_o_y);
            s_io_0.u       <= std_logic_vector(s_o_u);
            s_io_0.v       <= std_logic_vector(s_o_v);
            s_io_0.hsync_n <= s_sync_pipe(C_RENDER_DELAY - 1)(0);
            s_io_0.vsync_n <= s_sync_pipe(C_RENDER_DELAY - 1)(1);
            s_io_0.avid    <= s_sync_pipe(C_RENDER_DELAY - 1)(2);
            s_io_0.field_n <= s_sync_pipe(C_RENDER_DELAY - 1)(3);
            s_io_1 <= s_io_0;
        end if;
    end process;

    data_out.y       <= s_io_1.y;
    data_out.u       <= s_io_1.u;
    data_out.v       <= s_io_1.v;
    data_out.hsync_n <= s_io_1.hsync_n;
    data_out.vsync_n <= s_io_1.vsync_n;
    data_out.avid    <= s_io_1.avid;
    data_out.field_n <= s_io_1.field_n;

end architecture tessera;
