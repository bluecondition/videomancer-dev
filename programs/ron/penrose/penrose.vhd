-- penrose.vhd  (v2.0 -- never-ending staircase, a tribute to M.C. Escher)
--
-- A Penrose staircase: a closed rectangular ring of stairs that climbs forever.
-- The scene is a REAL 3D spiral heightfield -- in orthographic isometric
-- projection, moving one cell in +i and +j while rising one full cell height
-- projects to the IDENTICAL screen position, so a spiral of N = q*D steps with
-- net drift (D,D) closes visually into an impossible loop.
--
-- v2 fills the frame with ONE large, coherent loop instead of one small one,
-- and terraces the courtyard so the whole frame is stepped, structural, and
-- geometrically consistent:
--   * 32-STEP RING.  q=8, D=4, S=10 -> N=32 steps, an 11x11 cell ring at
--     (i0,j0)=(6,-3).  All four sides are now full staircases (v1's 4th side was
--     degenerate at 1 cell, so v1 only coded 3 sides; v2 codes all four).
--   * TERRACED COURTYARD.  The open interior right of the seam wall is filled
--     with concentric rectangular terraces stepping DOWN to its centre -- a
--     single-valued heightfield, so it is perfectly consistent.  P12 sets the
--     tread depth, K3 (Well) sinks the whole courtyard, S10 flattens it.
-- The walking monks of v1 are gone; the impossible loop itself is the
-- "never-ending" -- no figures, no sprite engine.
--
-- Renderer: per-pixel heightfield ray-march (no BRAM, no line buffer, no
-- framebuffer), forked from ziggurat, with the v1 innovations retained:
--
--  * EXACT DDA march.  The ring is mostly EMPTY -- its tall walls live in the
--    off-diagonal columns, and a diagonal-only march (ziggurat) shatters them
--    into a lattice of holes.  The ray also crosses the off-diagonal cells
--    (m,m-1)/(m-1,m); their ray-in-column test reduces to the normal cover
--    compare plus ONE pixel-global rail bit (the sign of ru-rv).  KF=5 needs
--    16 candidates (6 diagonal + 10 rail).
--
--  * NORMALIZED CELL UNITS.  All march arithmetic is in units where one cell is
--    exactly CELL=1024, at every resolution.  The accumulators are fixed-point
--    and step by P = 512/a per pixel, Q = 512/b per line (two vblank divides).
--    So every stair constant (HU, BASEH, KS1..KS4, KT) is compile-time,
--    tvw(m) = (m<<10)+R is free wiring, cover compares against the CONSTANT
--    +/-CELL, and words are 15-bit and resolution-independent.
--
--  * Per-candidate payload is just the face offsets vp = aH - tvw(mi),
--    up = tuw(mj) - aH (plus a terrace/platform bit).  Absolute V/U are never
--    formed.
--
-- Geometry (locked with the Python mock, tiled_mock.py):
--   S=10, D=4, q=8, N=32, ring 11x11 at (i0,j0)=(6,-3), BASEH=1 cell, KF=5
--   (max stair top 4.875 cells).  a = 3*mh/64, b = 5*mv/128 -> self-frames at
--   SD / 720p / 1080i / 1080p (84% x 89% of frame, no clipping).
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

architecture penrose of program_top is

    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Clamp a face tone into the legal luma swing and hand back a 10-bit luma.
    -- Returns UNSIGNED on purpose: unsigned(resize(v,10)) on a signed value
    -- saturates at the SIGNED range (keeps the sign bit, drops the top magnitude
    -- bit), so any tone >= 512 wraps (584 -> 72, i.e. black).
    function clamp_y(v : signed(11 downto 0)) return unsigned is
    begin
        if    v < 16   then return to_unsigned(16, 10);
        elsif v > 1000 then return to_unsigned(1000, 10);
        else                return unsigned(v(9 downto 0));
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Ring geometry (compile-time).
    --------------------------------------------------------------------------
    constant KF   : integer := 5;                -- march depth (stair top 4.875 cells)
    constant NC   : integer := 16;               -- DDA candidates: 6 diag + 10 rail
    constant I0   : integer := 6;                -- ring anchor cell
    constant J0   : integer := -3;
    constant SS   : integer := 10;               -- side length (moves); ring is 11x11
    constant DD   : integer := 4;                -- net drift per loop
    constant A3   : integer := SS - DD;          -- 6
    constant NSTEP: integer := 32;               -- q*D steps around the loop

    -- normalized cell units
    constant CELL : integer := 1024;
    constant HU   : integer := CELL / 8;         -- riser (q = 8) = 128
    constant BASEH: integer := CELL;             -- lowest stair top
    constant KT   : integer := CELL / 2;         -- courtyard rim floor (K3 Well sinks it)
    -- side heights (see header): s1 = KS1 + HU*ci,  s2 = KS2 + HU*cj,
    --                            s3 = KS3 - HU*ci,  s4 = KS4 - HU*cj
    constant KS1  : integer := BASEH - HU * I0;                    -- 256
    constant KS2  : integer := BASEH + HU * (SS - J0);             -- 2688
    constant KS3  : integer := BASEH + HU * (3 * SS + I0);         -- 5632
    constant KS4  : integer := BASEH + HU * (3 * SS + A3 + J0);    -- 5248

    constant C_W  : integer := 15;               -- march word
    subtype  t_w  is signed(C_W - 1 downto 0);
    constant C_IW : integer := 7;                -- cell index width (signed)
    subtype  t_i  is signed(C_IW - 1 downto 0);

    -- candidate list, front (nearest) to back: diag m, then rails (m,m-1)/(m-1,m)
    type t_cint is array(0 to NC - 1) of integer;
    constant CAND_MI   : t_cint := (5,5,4, 4,4,3, 3,3,2, 2,2,1, 1,1,0, 0);
    constant CAND_MJ   : t_cint := (5,4,5, 4,3,4, 3,2,3, 2,1,2, 1,0,1, 0);
    -- 0 = diagonal (both compares), 1 = needs ru<=rv, 2 = needs rv<=ru
    constant CAND_RAIL : t_cint := (0,1,2, 0,1,2, 0,1,2, 0,1,2, 0,1,2, 0);

    --------------------------------------------------------------------------
    -- Parameters (pre-registered before any use).
    --------------------------------------------------------------------------
    signal s_ink, s_shade, s_well, s_scheme, s_hatchk : unsigned(9 downto 0) := (others => '0');
    signal s_depth : unsigned(9 downto 0) := (others => '0');
    signal s_outline_en, s_vidsky, s_shallow, s_terr : std_logic := '0';

    --------------------------------------------------------------------------
    -- Timing / resolution
    --------------------------------------------------------------------------
    signal s_timing  : t_video_timing_port;
    signal s_h_count : unsigned(11 downto 0);
    signal s_v_count : unsigned(11 downto 0);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_vsync_prev  : std_logic := '1';
    signal s_vsync_pulse : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame constants
    --------------------------------------------------------------------------
    signal s_araw, s_braw : unsigned(11 downto 0) := to_unsigned(60, 12);
    signal s_bsh    : unsigned(11 downto 0) := to_unsigned(28, 12);
    signal s_a, s_b : unsigned(6 downto 0) := to_unsigned(60, 7);   -- cell size in pixels
    signal s_P, s_Q : unsigned(17 downto 0) := to_unsigned(34952, 18); -- 512/a, 512/b (5.12)
    signal s_thr    : t_w := to_signed(6, C_W);                     -- outline half-width
    signal s_tstep  : unsigned(8 downto 0) := (others => '0');      -- terrace tread drop
    signal s_kt     : t_w := to_signed(KT, C_W);                    -- courtyard rim (K3 Well lowers)
    -- the three possible terrace-tread heights (r = 0/1/2), precomputed once per
    -- frame so a2a is a 3-way select, not 16 s_kt - r*tstep subtracts (that was
    -- the routing-bound critical path)
    signal s_tAH0, s_tAH1, s_tAH2 : t_w := to_signed(KT, C_W);
    -- print scheme (frame-latched)
    signal s_paper_y, s_ink_y, s_line_y : unsigned(9 downto 0) := (others => '0');
    signal s_paper_u, s_paper_v, s_ink_u, s_ink_v : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_right_y, s_left_y, s_plat_y : unsigned(9 downto 0) := (others => '0');
    signal s_dd, s_dsc : signed(11 downto 0) := (others => '0');  -- ink-paper, K2-scaled
    -- hatch thresholds / masks
    signal s_hmask  : unsigned(4 downto 0) := to_unsigned(7, 5);
    signal s_hmask2 : unsigned(4 downto 0) := to_unsigned(15, 5);
    signal s_tl, s_tr, s_tt : unsigned(4 downto 0) := (others => '0');
    signal s_hatch_off : std_logic := '0';

    signal s_fcnt : unsigned(7 downto 0) := (others => '1');  -- vblank FSM step (idle = 255)

    --------------------------------------------------------------------------
    -- Vblank divider: P = 2^21/a, Q = 2^21/b (restoring, 1 bit/cycle).
    --------------------------------------------------------------------------
    signal s_dv_start : std_logic := '0';
    signal s_dv_busy  : std_logic := '0';
    signal s_dv_i     : integer range 0 to 21 := 0;
    signal s_dv_ra, s_dv_rb : unsigned(7 downto 0) := (others => '0');
    signal s_dv_qa, s_dv_qb : unsigned(17 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Fixed-point iso accumulators: [.:.]=cell . sub-cell . frac
    --------------------------------------------------------------------------
    signal s_Lacc, s_Vs, s_Ns : signed(27 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Stage PIX: cell indices, complement remainders, rail bits
    --------------------------------------------------------------------------
    signal p_iv, p_ju : t_i := (others => '0');
    signal p_R, p_Ru  : unsigned(10 downto 0) := (others => '0');  -- CELL - rv/ru
    signal p_railA, p_railB : std_logic := '0';
    signal p_sync : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Stage A1: per-m shared values (m = 0..KF).
    --------------------------------------------------------------------------
    type t_warrM is array(0 to KF) of t_w;
    type t_farrM is array(0 to KF) of unsigned(1 downto 0);   -- terrace i-distance (0..2)
    type t_garrM is array(0 to KF) of unsigned(2 downto 0);   -- terrace j-distance (0..4)
    signal a1_tvw, a1_tuw : t_warrM := (others => (others => '0'));
    -- HU*iv and HU*ju (one shift each); the four side heights are formed from
    -- these + compile-time constants in a2a.  Computing all 4 x 6 side adds HERE
    -- overloaded p_a1's fanout and made it the congestion-bound critical path.
    signal a1_ish, a1_jsh : t_w := (others => '0');
    -- i-side range bits (from di)
    signal a1_di0S, a1_diS, a1_diD9, a1_diD, a1_di59 : std_logic_vector(0 to KF) := (others => '0');
    -- j-side range bits (from dj)
    signal a1_dj0, a1_dj1S, a1_djS, a1_dj59, a1_dj19 : std_logic_vector(0 to KF) := (others => '0');
    signal a1_fi : t_farrM := (others => (others => '0'));
    signal a1_gj : t_garrM := (others => (others => '0'));
    signal a1_railA, a1_railB : std_logic := '0';
    signal a1_sync : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Stage A2a: per-candidate cell height + region bits (the heavy classify /
    -- 5-way aH mux / terrace lives here, on its own so it is not chained with
    -- the vp/up subtracts -- folded together the cone dropped Fmax to ~55 MHz).
    --------------------------------------------------------------------------
    type t_warrC is array(0 to NC - 1) of t_w;
    -- the stair-side height and the terrace height are computed IN PARALLEL and
    -- registered separately; the 2-way select between them moves to a2b.  Folded
    -- together (gj -> min -> drop -> subtract -> 5-way mux) they were the
    -- critical path at ~55 MHz.
    signal a2a_sAH, a2a_tAH : t_warrC := (others => (others => '0'));
    signal a2a_ex, a2a_tr : std_logic_vector(0 to NC - 1) := (others => '0');
    signal a2a_tvw, a2a_tuw : t_warrM := (others => (others => '0'));
    signal a2a_railA, a2a_railB : std_logic := '0';
    signal a2a_sync : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Stage A2b: per-candidate face offsets + terrace/platform bit
    --------------------------------------------------------------------------
    signal a2_vp, a2_up : t_warrC := (others => (others => '0'));
    signal a2_ex, a2_pl : std_logic_vector(0 to NC - 1) := (others => '0');
    signal a2_railA, a2_railB : std_logic := '0';
    signal a2_sync : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Stage B1: cover + priority within groups; B2: groups -> 1
    --------------------------------------------------------------------------
    constant NG : integer := 4;                  -- 4 groups of 4
    type t_warrG is array(0 to NG - 1) of t_w;
    signal g_cov : std_logic_vector(0 to NG - 1) := (others => '0');
    signal g_vp, g_up : t_warrG := (others => (others => '0'));
    signal g_pl : std_logic_vector(0 to NG - 1) := (others => '0');
    signal g_sync : std_logic_vector(3 downto 0) := (others => '0');

    signal b2_found : std_logic := '0';
    signal b2_vp, b2_up : t_w := (others => '0');
    signal b2_pl : std_logic := '0';
    signal b2_sync : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Colour stages
    --------------------------------------------------------------------------
    signal c1_up, c1_vp, c1_upvp : t_w := (others => '0');
    signal c1_found, c1_pl : std_logic := '0';
    signal c1_sync : std_logic_vector(3 downto 0) := (others => '0');

    signal c2_top, c2_right, c2_line : std_logic := '0';
    signal c2_found, c2_pl : std_logic := '0';
    signal c2_patL, c2_patR, c2_patT, c2_sky : std_logic := '0';
    signal c2_sync : std_logic_vector(3 downto 0) := (others => '0');

    signal c3_y : unsigned(9 downto 0) := (others => '0');
    signal c3_u, c3_v : unsigned(9 downto 0) := C_CHROMA_MID;
    signal c3_sync : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Delay chains (counters + input luma), aligned to the march depth.
    --------------------------------------------------------------------------
    -- one register deeper than v1: the a2 split added a pipeline stage, so p_c2
    -- reads index 6 (delay 7) to stay aligned with the coloured pixel.
    type t_d12 is array(0 to 7) of unsigned(11 downto 0);
    signal hc_d, vc_d : t_d12 := (others => (others => '0'));
    type t_dy is array(0 to 7) of std_logic_vector(9 downto 0);
    signal vy_d : t_dy := (others => (others => '0'));

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Parameter pre-registration.
    --------------------------------------------------------------------------
    p_regs : process(clk)
    begin
        if rising_edge(clk) then
            s_ink        <= unsigned(registers_in(0));    -- K1 Ink
            s_shade      <= unsigned(registers_in(1));    -- K2 Shade
            s_well       <= unsigned(registers_in(2));    -- K3 Well (courtyard depth)
            s_scheme     <= unsigned(registers_in(3));    -- K4 Scheme
            s_hatchk     <= unsigned(registers_in(5));    -- K6 Hatch
            s_outline_en <= registers_in(6)(0);           -- S7 Outline
            s_vidsky     <= registers_in(6)(1);           -- S8 Video Sky
            s_shallow    <= registers_in(6)(2);           -- S9 View
            s_terr       <= registers_in(6)(3);           -- S10 Terraces
            s_depth      <= unsigned(registers_in(7));    -- P12 Terrace Depth
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Timing infrastructure.
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    pixel_counter_inst : entity work.pixel_counter
        port map (clk => clk, timing => s_timing,
                  h_count => s_h_count, v_count => s_v_count);

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

    --------------------------------------------------------------------------
    -- P = 2^21/a, Q = 2^21/b by restoring division (22 cycles, in vblank).
    --------------------------------------------------------------------------
    p_div : process(clk)
        variable v_ra, v_rb : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if s_dv_start = '1' then
                s_dv_ra <= to_unsigned(1, 8);
                s_dv_rb <= to_unsigned(1, 8);
                s_dv_qa <= (others => '0');
                s_dv_qb <= (others => '0');
                s_dv_i  <= 20;
                s_dv_busy <= '1';
            elsif s_dv_busy = '1' then
                v_ra := s_dv_ra(6 downto 0) & '0';
                if v_ra >= ("0" & s_a) then
                    s_dv_ra <= v_ra - ("0" & s_a);
                    s_dv_qa <= s_dv_qa(16 downto 0) & '1';
                else
                    s_dv_ra <= v_ra;
                    s_dv_qa <= s_dv_qa(16 downto 0) & '0';
                end if;
                v_rb := s_dv_rb(6 downto 0) & '0';
                if v_rb >= ("0" & s_b) then
                    s_dv_rb <= v_rb - ("0" & s_b);
                    s_dv_qb <= s_dv_qb(16 downto 0) & '1';
                else
                    s_dv_rb <= v_rb;
                    s_dv_qb <= s_dv_qb(16 downto 0) & '0';
                end if;
                if s_dv_i = 0 then
                    s_dv_busy <= '0';
                else
                    s_dv_i <= s_dv_i - 1;
                end if;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Frame FSM.  Latches a/b, the style constants, the terrace/sweep controls,
    -- and kicks the divider -- ONE SMALL STEP PER CYCLE.  STA has no idea this
    -- only runs in vblank, so any long combinational cone here would cap the
    -- whole design's Fmax; cycles are free in vblank, so every step is tiny.
    -- (v2 has no shared multiplier and no monk FSM -- both are gone with the
    -- walking figures.)
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_y : signed(11 downto 0);
        variable v_t  : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            s_dv_start <= '0';

            if s_vsync_pulse = '1' then
                s_fcnt <= (others => '0');
            elsif s_fcnt /= 255 then
                s_fcnt <= s_fcnt + 1;
            end if;

            case to_integer(s_fcnt) is

                when 0 =>       -- raw cell size: a = 3*mh/64, b = 5*mv/128
                    s_araw <= shift_right(s_measured_h, 5) + shift_right(s_measured_h, 6);
                    s_braw <= shift_right(s_measured_v, 5) + shift_right(s_measured_v, 7);

                when 1 =>       -- shallow view flattens b
                    if s_shallow = '1' then
                        s_bsh <= shift_right(s_braw, 1) + shift_right(s_braw, 2);
                    else
                        s_bsh <= s_braw;
                    end if;

                when 2 =>       -- clamp (keeps the divider quotient inside 18 bits)
                    if    s_araw < 16  then s_a <= to_unsigned(16, 7);
                    elsif s_araw > 127 then s_a <= to_unsigned(127, 7);
                    else                    s_a <= s_araw(6 downto 0);
                    end if;
                    if    s_bsh < 16  then s_b <= to_unsigned(16, 7);
                    elsif s_bsh > 127 then s_b <= to_unsigned(127, 7);
                    else                   s_b <= s_bsh(6 downto 0);
                    end if;
                    s_dv_start <= '1';

                when 3 =>       -- terrace tread depth (P12) and courtyard rim (K3 Well)
                    -- tstep in 0..127 so 2*tstep < CELL/2; center never underflows
                    s_tstep <= resize(s_depth(9 downto 3), 9);
                    -- Well sinks the whole courtyard: KT (shallow dish) .. ~0 (deep pit)
                    s_kt <= to_signed(KT, C_W) - signed(resize(s_well(9 downto 1), C_W));

                when 5 =>       -- the three terrace-tread heights (r = 0/1/2)
                    s_tAH0 <= s_kt;
                    if s_kt - signed(resize(s_tstep, C_W)) < 16 then
                        s_tAH1 <= to_signed(16, C_W);
                    else
                        s_tAH1 <= s_kt - signed(resize(s_tstep, C_W));
                    end if;
                    if s_kt - signed(resize(s_tstep & '0', C_W)) < 16 then
                        s_tAH2 <= to_signed(16, C_W);
                    else
                        s_tAH2 <= s_kt - signed(resize(s_tstep & '0', C_W));
                    end if;

                when 8 =>
                    -- Hatch density.  v_t is K1 in eighths of full coverage; each
                    -- Hatch period scales it to keep the ink weight at any line
                    -- spacing.  Left face full density, right face half.
                    v_t := resize(s_ink(9 downto 7), 5);             -- 0..7
                    s_hatch_off <= '0';
                    if s_hatchk < 256 then                            -- Fine (period 4)
                        s_hmask  <= to_unsigned(3, 5);
                        s_hmask2 <= to_unsigned(7, 5);
                        s_tl <= '0' & v_t(4 downto 1);
                        s_tr <= "00" & v_t(4 downto 2);
                    elsif s_hatchk < 512 then                         -- Medium (period 8)
                        s_hmask  <= to_unsigned(7, 5);
                        s_hmask2 <= to_unsigned(15, 5);
                        s_tl <= v_t;
                        s_tr <= '0' & v_t(4 downto 1);
                    elsif s_hatchk < 768 then                         -- Coarse (period 16)
                        s_hmask  <= to_unsigned(15, 5);
                        s_hmask2 <= to_unsigned(31, 5);
                        s_tl <= v_t(3 downto 0) & '0';
                        s_tr <= v_t;
                    else                                              -- Off
                        s_hmask  <= to_unsigned(7, 5);
                        s_hmask2 <= to_unsigned(15, 5);
                        s_tl <= v_t;
                        s_tr <= '0' & v_t(4 downto 1);
                        s_hatch_off <= '1';
                    end if;
                    s_tt <= "000" & s_ink(9 downto 8);

                when 9 =>       -- print scheme
                    case to_integer(s_scheme(9 downto 8)) is
                        when 0 =>       -- Parchment (warm sepia paper, near-black ink)
                            s_paper_y <= to_unsigned(740, 10);
                            s_paper_u <= to_unsigned(470, 10); s_paper_v <= to_unsigned(540, 10);
                            s_ink_y <= to_unsigned(30, 10);
                            s_ink_u <= to_unsigned(495, 10);   s_ink_v <= to_unsigned(522, 10);
                        when 1 =>       -- Gallery (clean white print)
                            s_paper_y <= to_unsigned(830, 10);
                            s_paper_u <= C_CHROMA_MID;         s_paper_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(40, 10);
                            s_ink_u <= C_CHROMA_MID;           s_ink_v <= C_CHROMA_MID;
                        when 2 =>       -- Night (bright ink on near-black paper)
                            s_paper_y <= to_unsigned(120, 10);
                            s_paper_u <= C_CHROMA_MID;         s_paper_v <= C_CHROMA_MID;
                            s_ink_y <= to_unsigned(750, 10);
                            s_ink_u <= C_CHROMA_MID;           s_ink_v <= C_CHROMA_MID;
                        when others =>  -- Blueprint (white lines on deep blue)
                            s_paper_y <= to_unsigned(300, 10);
                            s_paper_u <= to_unsigned(700, 10); s_paper_v <= to_unsigned(450, 10);
                            s_ink_y <= to_unsigned(850, 10);
                            s_ink_u <= to_unsigned(500, 10);   s_ink_v <= to_unsigned(506, 10);
                    end case;

                -- Face tones shade paper TOWARD INK (not simply downward, which
                -- blacked out dark-paper schemes).  s_dsc is the K2-scaled step.
                when 10 =>
                    s_dd <= signed(resize(s_ink_y, 12)) - signed(resize(s_paper_y, 12));

                when 11 =>      -- (ink-paper)*K2/32 via a dedicated 12x4 multiply
                    s_dsc <= resize(shift_right(s_dd * signed('0' & s_shade(9 downto 6)), 5), 12);

                when 14 =>
                    v_y := signed(resize(s_paper_y, 12)) + s_dsc;
                    s_right_y <= clamp_y(v_y);

                when 15 =>
                    v_y := signed(resize(s_paper_y, 12)) + s_dsc + shift_right(s_dsc, 1);
                    s_left_y <= clamp_y(v_y);

                when 16 =>
                    v_y := signed(resize(s_paper_y, 12)) + shift_right(s_dsc, 2);
                    s_plat_y <= clamp_y(v_y);

                when 17 =>
                    s_line_y <= unsigned(resize(shift_right(resize(s_paper_y, 11)
                                                          + resize(s_ink_y, 11), 1), 10));

                when 30 =>      -- divider done (22 cycles from fcnt 2)
                    s_P <= s_dv_qa;
                    s_Q <= s_dv_qb;

                when 31 =>      -- outline width ~1 pixel, from the per-pixel/line steps
                    s_thr <= resize(signed('0' & shift_right(resize(s_P(17 downto 12), 7)
                                                           + resize(s_Q(17 downto 12), 7), 2)), C_W);

                when others => null;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fixed-point iso accumulators.  Origin at the top-left pixel.
    --------------------------------------------------------------------------
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_Lacc <= (others => '0');
                s_Vs   <= (others => '0');
                s_Ns   <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_Vs   <= s_Lacc;
                s_Ns   <= s_Lacc;
                s_Lacc <= s_Lacc + signed(resize(s_Q, 28));
            elsif s_timing.avid = '1' then
                s_Vs <= s_Vs + signed(resize(s_P, 28));
                s_Ns <= s_Ns - signed(resize(s_P, 28));
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage PIX: cell index + sub-cell remainder.  R = CELL - rv is the only
    -- arithmetic; tvw(m) = (m<<10) + R then costs nothing but wiring.
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_rv, v_ru : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_rv := unsigned(s_Vs(21 downto 12));
            v_ru := unsigned(s_Ns(21 downto 12));
            p_iv <= resize(s_Vs(27 downto 22), C_IW);
            p_ju <= resize(s_Ns(27 downto 22), C_IW);
            p_R  <= to_unsigned(CELL, 11) - resize(v_rv, 11);
            p_Ru <= to_unsigned(CELL, 11) - resize(v_ru, 11);
            if v_ru <= v_rv then p_railA <= '1'; else p_railA <= '0'; end if;
            if v_rv <= v_ru then p_railB <= '1'; else p_railB <= '0'; end if;
            p_sync <= data_in.field_n & data_in.avid &
                      data_in.vsync_n & data_in.hsync_n;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage A1: per-m shared values.  HU = 128, so HU*ci = ci << 7.
    --   s1v = KS1 + HU*ci   s2v = KS2 + HU*cj
    --   s3v = KS3 - HU*ci   s4v = KS4 - HU*cj
    -- Range bits classify di / dj into the four sides + terrace region.
    -- fi / gj are the terrace inset distances (min to the two courtyard edges).
    --------------------------------------------------------------------------
    p_a1 : process(clk)
        variable v_di, v_dj : signed(C_IW downto 0);
        variable v_t1, v_t2 : signed(C_IW downto 0);
    begin
        if rising_edge(clk) then
            a1_ish <= resize(p_iv & "0000000", C_W);      -- HU*iv (HU = 128)
            a1_jsh <= resize(p_ju & "0000000", C_W);
            for m in 0 to KF loop
                a1_tvw(m) <= signed(resize(p_R, C_W)) + to_signed(m * CELL, C_W);
                a1_tuw(m) <= signed(resize(p_Ru, C_W)) + to_signed(m * CELL, C_W);

                v_di := resize(p_iv, C_IW + 1) + to_signed(m - I0, C_IW + 1);
                v_dj := resize(p_ju, C_IW + 1) + to_signed(m - J0, C_IW + 1);
                -- i-side membership
                if v_di >= 0 and v_di <= SS then a1_di0S(m) <= '1'; else a1_di0S(m) <= '0'; end if;
                if v_di = SS then a1_diS(m) <= '1'; else a1_diS(m) <= '0'; end if;
                if v_di >= DD and v_di <= SS - 1 then a1_diD9(m) <= '1'; else a1_diD9(m) <= '0'; end if;
                if v_di = DD then a1_diD(m) <= '1'; else a1_diD(m) <= '0'; end if;
                if v_di >= DD + 1 and v_di <= SS - 1 then a1_di59(m) <= '1'; else a1_di59(m) <= '0'; end if;
                -- j-side membership
                if v_dj = 0 then a1_dj0(m) <= '1'; else a1_dj0(m) <= '0'; end if;
                if v_dj >= 1 and v_dj <= SS then a1_dj1S(m) <= '1'; else a1_dj1S(m) <= '0'; end if;
                if v_dj = SS then a1_djS(m) <= '1'; else a1_djS(m) <= '0'; end if;
                if v_dj >= DD + 1 and v_dj <= SS - 1 then a1_dj59(m) <= '1'; else a1_dj59(m) <= '0'; end if;
                if v_dj >= 1 and v_dj <= SS - 1 then a1_dj19(m) <= '1'; else a1_dj19(m) <= '0'; end if;
                -- terrace inset distances (min to the two opposite courtyard edges)
                v_t1 := v_di - to_signed(DD + 1, C_IW + 1);      -- di - 5
                v_t2 := to_signed(SS - 1, C_IW + 1) - v_di;      -- 9 - di
                if v_t1 < v_t2 then a1_fi(m) <= unsigned(std_logic_vector(v_t1(1 downto 0)));
                else                a1_fi(m) <= unsigned(std_logic_vector(v_t2(1 downto 0))); end if;
                v_t1 := v_dj - to_signed(1, C_IW + 1);           -- dj - 1
                v_t2 := to_signed(SS - 1, C_IW + 1) - v_dj;      -- 9 - dj
                if v_t1 < v_t2 then a1_gj(m) <= unsigned(std_logic_vector(v_t1(2 downto 0)));
                else                a1_gj(m) <= unsigned(std_logic_vector(v_t2(2 downto 0))); end if;
            end loop;
            a1_railA <= p_railA;
            a1_railB <= p_railB;
            a1_sync  <= p_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage A2a: per-candidate region resolve -> aH.  Sides are disjoint by
    -- construction, so the if/elsif is a plain mux.  Terrace tread height is
    -- KT - r*tstep with r in 0..2 (a shift mux, no multiply).
    --------------------------------------------------------------------------
    p_a2a : process(clk)
        variable v_s1, v_s2, v_s3, v_s4, v_st : std_logic;
        variable v_sAH, v_tAH : t_w;
        variable v_r  : unsigned(1 downto 0);
        variable mi, mj : integer;
    begin
        if rising_edge(clk) then
            for c in 0 to NC - 1 loop
                mi := CAND_MI(c);
                mj := CAND_MJ(c);
                v_s1 := a1_dj0(mj)  and a1_di0S(mi);
                v_s2 := a1_diS(mi)  and a1_dj1S(mj);
                v_s3 := a1_djS(mj)  and a1_diD9(mi);
                v_s4 := a1_diD(mi)  and a1_dj59(mj);
                v_st := v_s1 or v_s2 or v_s3 or v_s4;

                -- stair-side height from HU*iv / HU*ju + compile-time constants:
                --   s1 = (KS1 + HU*mi) + HU*iv    s3 = (KS3 - HU*mi) - HU*iv
                --   s2 = (KS2 + HU*mj) + HU*ju    s4 = (KS4 - HU*mj) - HU*ju
                if    v_s1 = '1' then v_sAH := a1_ish + to_signed(KS1 + HU * mi, C_W);
                elsif v_s2 = '1' then v_sAH := a1_jsh + to_signed(KS2 + HU * mj, C_W);
                elsif v_s3 = '1' then v_sAH := to_signed(KS3 - HU * mi, C_W) - a1_ish;
                else                  v_sAH := to_signed(KS4 - HU * mj, C_W) - a1_jsh;
                end if;

                -- terrace height, in PARALLEL: select one of the three
                -- precomputed tread heights by r = min(fi, gj).  Since fi <= 2,
                -- min(fi, gj) <= 2, so r is always in 0..2 with no cap needed.
                if a1_fi(mi) < a1_gj(mj) then v_r := a1_fi(mi);
                else                          v_r := a1_gj(mj)(1 downto 0); end if;
                if s_terr = '0' then v_r := "00"; end if;    -- S10 off: flat floor
                case to_integer(v_r) is
                    when 0      => v_tAH := s_tAH0;
                    when 1      => v_tAH := s_tAH1;
                    when others => v_tAH := s_tAH2;
                end case;

                a2a_sAH(c) <= v_sAH;
                a2a_tAH(c) <= v_tAH;
                a2a_tr(c)  <= a1_di59(mi) and a1_dj19(mj) and not v_st;
                a2a_ex(c)  <= v_st or (a1_di59(mi) and a1_dj19(mj));
            end loop;
            a2a_tvw  <= a1_tvw;
            a2a_tuw  <= a1_tuw;
            a2a_railA <= a1_railA;
            a2a_railB <= a1_railB;
            a2a_sync  <= a1_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage A2b: select stair-vs-terrace height and form the face offsets
    --   vp = aH - tvw(mi),  up = tuw(mj) - aH.
    --------------------------------------------------------------------------
    p_a2b : process(clk)
        variable v_aH : t_w;
        variable mi, mj : integer;
    begin
        if rising_edge(clk) then
            for c in 0 to NC - 1 loop
                mi := CAND_MI(c);
                mj := CAND_MJ(c);
                if a2a_tr(c) = '1' then v_aH := a2a_tAH(c);
                else                    v_aH := a2a_sAH(c);
                end if;
                a2_vp(c) <= v_aH - a2a_tvw(mi);
                a2_up(c) <= a2a_tuw(mj) - v_aH;
                a2_ex(c) <= a2a_ex(c);
                a2_pl(c) <= a2a_tr(c);
            end loop;
            a2_railA <= a2a_railA;
            a2_railB <= a2a_railB;
            a2_sync  <= a2a_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage B1: cover (against the CONSTANT +/-CELL) + priority within groups
    -- of 4, front to back.  A rail candidate's other bound is dominated, so one
    -- compare suffices there.
    --------------------------------------------------------------------------
    p_b1 : process(clk)
        variable v_cov : std_logic_vector(0 to NC - 1);
        variable v_c, v_f, v_pl : std_logic;
        variable v_vp, v_up : t_w;
        variable lo, hi : integer;
    begin
        if rising_edge(clk) then
            for c in 0 to NC - 1 loop
                v_c := a2_ex(c);
                if a2_vp(c) < to_signed(-CELL, C_W) then v_c := '0'; end if;
                if CAND_RAIL(c) = 0 and a2_up(c) > to_signed(CELL, C_W) then v_c := '0'; end if;
                if CAND_RAIL(c) = 1 and a2_railA = '0' then v_c := '0'; end if;
                if CAND_RAIL(c) = 2 and a2_railB = '0' then v_c := '0'; end if;
                v_cov(c) := v_c;
            end loop;
            for g in 0 to NG - 1 loop
                lo := g * 4;
                hi := lo + 3;
                v_f := '0';
                v_vp := a2_vp(lo); v_up := a2_up(lo);
                v_pl := a2_pl(lo);
                for c in lo to hi loop
                    if v_f = '0' and v_cov(c) = '1' then
                        v_f := '1';
                        v_vp := a2_vp(c); v_up := a2_up(c);
                        v_pl := a2_pl(c);
                    end if;
                end loop;
                g_cov(g) <= v_f;
                g_vp(g) <= v_vp; g_up(g) <= v_up;
                g_pl(g) <= v_pl;
            end loop;
            g_sync <= a2_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage B2: groups -> 1 (front group wins).  No cover anywhere = sky.
    --------------------------------------------------------------------------
    p_b2 : process(clk)
        variable v_f, v_pl : std_logic;
        variable v_vp, v_up : t_w;
    begin
        if rising_edge(clk) then
            v_f := '0';
            v_vp := g_vp(NG - 1); v_up := g_up(NG - 1);
            v_pl := '0';
            for g in 0 to NG - 1 loop
                if v_f = '0' and g_cov(g) = '1' then
                    v_f := '1';
                    v_vp := g_vp(g); v_up := g_up(g);
                    v_pl := g_pl(g);
                end if;
            end loop;
            b2_found <= v_f;
            b2_vp <= v_vp; b2_up <= v_up;
            b2_pl <= v_pl;
            b2_sync <= g_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage C1: carry the face offsets and their sum (for face classify).
    --------------------------------------------------------------------------
    p_c1 : process(clk)
    begin
        if rising_edge(clk) then
            c1_up   <= b2_up;
            c1_vp   <= b2_vp;
            c1_upvp <= b2_up + b2_vp;
            c1_found <= b2_found;
            c1_pl    <= b2_pl;
            c1_sync  <= b2_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage C2: face + outline conditions and hatch/sky pattern bits.
    --------------------------------------------------------------------------
    p_c2 : process(clk)
        variable v_line : boolean;
        variable v_pp, v_pm : unsigned(12 downto 0);
        variable v_pt : unsigned(13 downto 0);
        variable v_t  : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            v_line := false;
            if s_outline_en = '1' then
                if ((c1_vp < s_thr) and (c1_vp > -s_thr) and (c1_up > 0)) or
                   ((c1_up < s_thr) and (c1_up > -s_thr) and (c1_vp < 0)) or
                   ((c1_upvp < s_thr) and (c1_upvp > -s_thr) and (c1_vp > 0)) then
                    v_line := true;
                end if;
            end if;
            if v_line then c2_line <= '1'; else c2_line <= '0'; end if;
            if (c1_up > 0) and (c1_vp < 0) then c2_top <= '1'; else c2_top <= '0'; end if;
            if (c1_vp > 0) and (c1_upvp > 0) then c2_right <= '1'; else c2_right <= '0'; end if;

            -- hatch patterns (screen-space; masks are frame constants)
            v_pp := resize(hc_d(6), 13) + resize(vc_d(6), 13);
            v_pm := resize(hc_d(6), 13) - resize(vc_d(6), 13);
            v_pt := resize(hc_d(6), 14) + resize(vc_d(6) & '0', 14);
            if (v_pp(4 downto 0) and s_hmask) < s_tl and s_hatch_off = '0' then
                c2_patL <= '1'; else c2_patL <= '0'; end if;
            if (v_pm(4 downto 0) and s_hmask) < s_tr and s_hatch_off = '0' then
                c2_patR <= '1'; else c2_patR <= '0'; end if;
            if (v_pt(4 downto 0) and s_hmask2) < s_tt and s_hatch_off = '0' then
                c2_patT <= '1'; else c2_patT <= '0'; end if;

            -- sky: engraved horizontal lines; thickness from video luma (S8)
            if s_vidsky = '1' then
                v_t := "00" & (not unsigned(vy_d(6)(9 downto 7)));
                if ("00" & vc_d(6)(2 downto 0)) < v_t then c2_sky <= '1'; else c2_sky <= '0'; end if;
            else
                if vc_d(6)(3 downto 0) = x"0" then c2_sky <= '1'; else c2_sky <= '0'; end if;
            end if;

            c2_found <= c1_found;
            c2_pl    <= c1_pl;
            c2_sync  <= c1_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage C3: final colour resolve.
    --------------------------------------------------------------------------
    p_c3 : process(clk)
    begin
        if rising_edge(clk) then
            if c2_found = '0' then
                if c2_sky = '1' then
                    c3_y <= s_line_y; c3_u <= s_paper_u; c3_v <= s_paper_v;
                else
                    c3_y <= s_paper_y; c3_u <= s_paper_u; c3_v <= s_paper_v;
                end if;
            elsif c2_line = '1' and c2_pl = '0' then
                c3_y <= s_ink_y; c3_u <= s_ink_u; c3_v <= s_ink_v;
            elsif c2_top = '1' then
                if c2_pl = '1' then                 -- terrace top
                    c3_y <= s_plat_y; c3_u <= s_paper_u; c3_v <= s_paper_v;
                elsif c2_patT = '1' then
                    c3_y <= s_line_y; c3_u <= s_paper_u; c3_v <= s_paper_v;
                else
                    c3_y <= s_paper_y; c3_u <= s_paper_u; c3_v <= s_paper_v;
                end if;
            elsif c2_right = '1' then
                if c2_patR = '1' then
                    c3_y <= s_ink_y; c3_u <= s_ink_u; c3_v <= s_ink_v;
                else
                    c3_y <= s_right_y; c3_u <= s_paper_u; c3_v <= s_paper_v;
                end if;
            else
                if c2_patL = '1' then
                    c3_y <= s_ink_y; c3_u <= s_ink_u; c3_v <= s_ink_v;
                else
                    c3_y <= s_left_y; c3_u <= s_paper_u; c3_v <= s_paper_v;
                end if;
            end if;
            c3_sync <= c2_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Delay chains (counters + input luma), aligned to the march depth.
    --------------------------------------------------------------------------
    p_delays : process(clk)
    begin
        if rising_edge(clk) then
            hc_d(0) <= s_h_count;
            vc_d(0) <= s_v_count;
            for i in 1 to 7 loop
                hc_d(i) <= hc_d(i - 1);
                vc_d(i) <= vc_d(i - 1);
            end loop;
            vy_d(0) <= data_in.y;
            for i in 1 to 7 loop
                vy_d(i) <= vy_d(i - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output.
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io.y       <= std_logic_vector(c3_y);
            s_io.u       <= std_logic_vector(c3_u);
            s_io.v       <= std_logic_vector(c3_v);
            s_io.hsync_n <= c3_sync(0);
            s_io.vsync_n <= c3_sync(1);
            s_io.avid    <= c3_sync(2);
            s_io.field_n <= c3_sync(3);
        end if;
    end process;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture penrose;
