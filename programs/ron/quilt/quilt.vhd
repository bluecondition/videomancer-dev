-- quilt.vhd  (v0.2 — per-pixel recursive-subdivision improv quilt)
--
-- Fills the whole screen with a loosely grid-based field of randomly coloured
-- and textured rectangular patches.  Each coarse grid cell is recursively split
-- (horizontally or vertically) into varied sub-rectangles, log-cabin fashion, so
-- patches contain nested sub-patches in a fractal way.  Static image: it only
-- changes when a knob/switch moves.
--
-- Method (no per-pixel multiply, no line buffer, no BRAM — modelled on ziggurat):
--   * Every output pixel marches a fixed-depth pipeline.  Stage 1 finds the
--     coarse cell (x>>g) and seeds a per-cell hash.  Then DMAX=4 subdivision
--     levels, each split across THREE shallow stages so HD timing closes:
--       (hash)    re-mix the running seed, decode split orientation + whether to
--                 subdivide (gated by the P12 "density" slider) + an off-centre
--                 jitter direction;
--       (decide)  compute the split coordinate nx = mid +/- jitter (jitter is
--                 bounded below w/2, so nx is always strictly interior — no
--                 clamp needed);
--       (descend) step the running rectangle bounds [x0,x1)x[y0,y1) into the
--                 half the pixel lies in and fold the choice into the seed so
--                 children diverge.
--     All splits are shift/add only.
--   * The leaf rectangle's seed selects a 16-colour palette entry and one of
--     several procedural textures (stripes / checker / diagonal / dither),
--     rendered from leaf-local coords with shifts/ANDs only.  Optional seam
--     lines border each patch; optional video-fill paints input video into some
--     patches.
--
-- Controls: K1 Grid Scale, K2 Composition Seed, K3 Sub-piece Seed, K4 Threshold
-- (Luma Mod: fully off at 0, then incoming luma drives per-cell piecing density
-- and hides cells whose held luma is below the threshold -> black), K5 Palette
-- Tint, K6 Texture Scale, S7 Seams, S8 Texture, S9 Grid (Square / Not-Square
-- off-grid overlap), S10 Piecing (Crazy = random split axis / Log Cabin =
-- alternating axis, concentric frames), S11 Palette (Fabric/Rainbow), P12
-- Density (KEY).  16 embossed texture types, always on; S8 Relief sets their
-- depth.  K2 Seed re-randomises the whole layout; K3 Evolve is BIPOLAR — centre
-- frozen, right = pieces change in scattered order, left = the change snakes
-- from each piece to one it touches.  Split jitter is fixed at Medium.
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

architecture quilt of program_top is

    constant LATENCY : natural := 22;   -- +1 S1a2 candidate-hash, +1 S10a2 evolve
    constant DMAX    : natural := 4;          -- subdivision levels
    constant MIN_W   : natural := 8;          -- don't split a span narrower than this
    constant SEAM_W  : natural := 2;          -- seam line half-thickness (px)
    -- texture emboss luma deltas (2-bit shade -> highlight / flat / shadow / deep)
    constant TEX_HI  : natural := 70;         -- highlight (valley between threads)
    constant TEX_D1  : natural := 110;        -- shadow
    constant TEX_D2  : natural := 210;        -- deep shadow (thread core)

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- per-level hash mixing constants (compile-time)
    type t_lc is array (0 to DMAX - 1) of unsigned(15 downto 0);
    constant C_LC : t_lc := (x"9E37", x"7C15", x"D2B9", x"3F41");

    --------------------------------------------------------------------------
    -- Pipeline node: running rectangle bounds + seed + freeze + pixel coords.
    --------------------------------------------------------------------------
    type t_node is record
        x0, x1, y0, y1 : unsigned(11 downto 0);
        px, py         : unsigned(11 downto 0);
        seed           : unsigned(15 downto 0);
        dens           : unsigned(7 downto 0);   -- per-cell subdivide threshold base
        hide           : std_logic;              -- Luma Mod: cell luma below K4 threshold
        frozen         : std_logic;
    end record;

    -- decide-stage result: split coordinate + "do it" flag.
    type t_dec is record
        nx   : unsigned(11 downto 0);
        doit : std_logic;
    end record;

    --------------------------------------------------------------------------
    -- Two 16-entry YUV palettes (BT.601: U=Cb, V=Cr).  Constant arrays -> LUTs.
    --------------------------------------------------------------------------
    type t_pal is array (0 to 15) of unsigned(9 downto 0);

    -- Fabric: saturated + muted quilt-fabric colours.
    constant FAB_Y : t_pal := (
        to_unsigned(330,10), to_unsigned(600,10), to_unsigned(820,10), to_unsigned(560,10),
        to_unsigned(470,10), to_unsigned(300,10), to_unsigned(360,10), to_unsigned(520,10),
        to_unsigned(720,10), to_unsigned(250,10), to_unsigned(620,10), to_unsigned(430,10),
        to_unsigned(880,10), to_unsigned(180,10), to_unsigned(540,10), to_unsigned(650,10));
    constant FAB_U : t_pal := (
        to_unsigned(400,10), to_unsigned(360,10), to_unsigned(300,10), to_unsigned(400,10),
        to_unsigned(620,10), to_unsigned(760,10), to_unsigned(700,10), to_unsigned(640,10),
        to_unsigned(470,10), to_unsigned(430,10), to_unsigned(450,10), to_unsigned(660,10),
        to_unsigned(500,10), to_unsigned(512,10), to_unsigned(380,10), to_unsigned(640,10));
    constant FAB_V : t_pal := (
        to_unsigned(720,10), to_unsigned(660,10), to_unsigned(560,10), to_unsigned(380,10),
        to_unsigned(360,10), to_unsigned(440,10), to_unsigned(600,10), to_unsigned(700,10),
        to_unsigned(580,10), to_unsigned(680,10), to_unsigned(470,10), to_unsigned(470,10),
        to_unsigned(540,10), to_unsigned(512,10), to_unsigned(640,10), to_unsigned(380,10));

    -- Rainbow: ROYGBIV.  Colours from the hardware-confirmed Phosphor table,
    -- stored in that table's U/V-swapped-for-hardware convention (Cr in U, Cb in
    -- V) — confirmed on hardware for this program.  Sequence R O Y G B I V x16.
    --                R    O    Y    G    B    I    V
    -- Y:           328  584  840  580  164  192  349
    -- U(=Cr):      960  772  584  136  440  608  756
    -- V(=Cb):      360  212   64  216  960  696  852
    constant RNB_Y : t_pal := (
        to_unsigned(328,10), to_unsigned(584,10), to_unsigned(840,10), to_unsigned(580,10),
        to_unsigned(164,10), to_unsigned(192,10), to_unsigned(349,10), to_unsigned(328,10),
        to_unsigned(584,10), to_unsigned(840,10), to_unsigned(580,10), to_unsigned(164,10),
        to_unsigned(192,10), to_unsigned(349,10), to_unsigned(328,10), to_unsigned(580,10));
    constant RNB_U : t_pal := (
        to_unsigned(960,10), to_unsigned(772,10), to_unsigned(584,10), to_unsigned(136,10),
        to_unsigned(440,10), to_unsigned(608,10), to_unsigned(756,10), to_unsigned(960,10),
        to_unsigned(772,10), to_unsigned(584,10), to_unsigned(136,10), to_unsigned(440,10),
        to_unsigned(608,10), to_unsigned(756,10), to_unsigned(960,10), to_unsigned(136,10));
    constant RNB_V : t_pal := (
        to_unsigned(360,10), to_unsigned(212,10), to_unsigned( 64,10), to_unsigned(216,10),
        to_unsigned(960,10), to_unsigned(696,10), to_unsigned(852,10), to_unsigned(360,10),
        to_unsigned(212,10), to_unsigned( 64,10), to_unsigned(216,10), to_unsigned(960,10),
        to_unsigned(696,10), to_unsigned(852,10), to_unsigned(360,10), to_unsigned(216,10));

    -- 4x4 Bayer matrix for the dither texture.  Index = (y[1:0]<<2)|x[1:0].
    type t_bayer is array (0 to 15) of unsigned(3 downto 0);
    constant C_BAYER : t_bayer := (
        to_unsigned( 0,4), to_unsigned( 8,4), to_unsigned( 2,4), to_unsigned(10,4),
        to_unsigned(12,4), to_unsigned( 4,4), to_unsigned(14,4), to_unsigned( 6,4),
        to_unsigned( 3,4), to_unsigned(11,4), to_unsigned( 1,4), to_unsigned( 9,4),
        to_unsigned(15,4), to_unsigned( 7,4), to_unsigned(13,4), to_unsigned( 5,4));

    --------------------------------------------------------------------------
    -- Functions (all shift/xor/add — no multiply).
    --------------------------------------------------------------------------
    function mix16(v : unsigned(9 downto 0); k : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := ("000000" & v) xor k;
        h := h xor shift_left (h, 7);
        h := h xor shift_right(h, 9);
        h := h xor shift_left (h, 8);
        return h;
    end function;

    function init_seed(comp : unsigned(15 downto 0); x0, y0 : unsigned(11 downto 0))
        return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := comp xor (x0(9 downto 2) & y0(9 downto 2));
        h := h xor shift_left (h, 5);
        h := h xor shift_right(h, 7);
        h := h xor shift_left (h, 3);
        return h;
    end function;

    function hash_lvl(seed, sub : unsigned(15 downto 0); lc : unsigned(15 downto 0))
        return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := seed xor sub xor lc;
        h := h xor shift_left (h, 6);
        h := h xor shift_right(h, 9);
        h := h xor shift_left (h, 2);
        return h;
    end function;

    -- Coarse-cell column index for the per-column luma hold (video density).
    function cell_col(x : unsigned(11 downto 0); g : integer) return integer is
        variable c : integer range 0 to 127;
    begin
        case g is
            when 5      => c := to_integer(x(11 downto 5));
            when 6      => c := to_integer(x(11 downto 6));
            when 7      => c := to_integer(x(11 downto 7));
            when others => c := to_integer(x(11 downto 8));
        end case;
        if c > 63 then c := 63; end if;
        return c;
    end function;

    -- Decide stage: compute the split coordinate for the active axis.  Jitter
    -- magnitude is at most 3w/8 < w/2, so nx is always strictly inside (lo,hi)
    -- and no clamp is required.
    -- w and mid (= midpoint) are precomputed in the hash stage.  Magnitudes are
    -- pure shifts of w (no add) so the only carry chain here is nx = mid +/- mag.
    function calc_split(w        : unsigned(11 downto 0);
                        mid      : unsigned(11 downto 0);
                        dosplit  : std_logic;
                        sgn      : std_logic;
                        half     : std_logic;
                        jit_mode : unsigned(1 downto 0)) return t_dec is
        variable mag : unsigned(11 downto 0);
        variable r : t_dec;
    begin
        case jit_mode is
            when "00"   => mag := (others => '0');
            when "01"   => mag := "0000" & w(11 downto 4);                    -- w/16
            when "10"   => mag := "000"  & w(11 downto 3);                    -- w/8
            when others => mag := "00"   & w(11 downto 2);                    -- w/4
        end case;
        if half = '1' then mag := '0' & mag(11 downto 1); end if;
        if sgn = '1' then r.nx := mid - mag;
        else              r.nx := mid + mag; end if;
        if (dosplit = '1') and (w >= to_unsigned(MIN_W, 12)) then
            r.doit := '1';
        else
            r.doit := '0';
        end if;
        return r;
    end function;

    -- Clamp a signed coord to a 0..4095 unsigned (negative -> 0).
    function clamp0(s : signed(13 downto 0)) return unsigned is
    begin
        if s < 0 then return to_unsigned(0, 12);
        else           return unsigned(s(11 downto 0)); end if;
    end function;

    -- Descend stage: step into the pixel's half and fold the choice into the seed.
    function descend(n : t_node; orient, doit : std_logic; nx : unsigned(11 downto 0))
        return t_node is
        variable r : t_node;
        variable lo, hi, pix : unsigned(11 downto 0);
    begin
        r := n;
        if doit = '1' then
            if orient = '0' then lo := n.x0; hi := n.x1; pix := n.px;
            else                 lo := n.y0; hi := n.y1; pix := n.py; end if;
            if pix < nx then
                hi := nx;
                r.seed := n.seed xor ("00000000" & nx(7 downto 0));
            else
                lo := nx;
                r.seed := (n.seed xor ("00000000" & nx(7 downto 0))) xor x"5A5A";
            end if;
            if orient = '0' then r.x0 := lo; r.x1 := hi;
            else                 r.y0 := lo; r.y1 := hi; end if;
            r.frozen := '0';
        else
            r.frozen := '1';
        end if;
        return r;
    end function;

    --------------------------------------------------------------------------
    -- Position + per-frame parameters
    --------------------------------------------------------------------------
    signal pixel_x, pixel_y : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n     : std_logic := '1';
    signal prev_vsync_n     : std_logic := '1';

    signal s_g          : integer range 5 to 8 := 6;
    signal s_cellmask   : unsigned(11 downto 0) := to_unsigned(63, 12);
    signal s_cellcenter : unsigned(11 downto 0) := to_unsigned(32, 12);
    signal s_density    : unsigned(7 downto 0) := to_unsigned(160, 8);
    signal s_dens_floor : unsigned(7 downto 0) := to_unsigned(95, 8);  -- 255 - density (precomputed)
    signal s_line_top   : std_logic := '0';                 -- on a cell-row's top line
    -- Split jitter is now fixed at Medium (w/8); K4 became the Luma Mod
    -- threshold.  Kept as a constant so calc_split's case folds away.
    constant C_JIT_MODE : unsigned(1 downto 0) := to_unsigned(2, 2);
    signal s_lum_thr    : unsigned(7 downto 0) := to_unsigned(128, 8);  -- K4: Luma Mod hide threshold
    signal s_tint       : unsigned(3 downto 0) := (others => '0');
    signal s_tex_shift  : integer range 3 to 6 := 5;
    -- Both layout seeds now come from the single K2 Seed knob (composition and
    -- sub-piecing used to be K2/K3; K3 is the Evolve Rate).
    signal s_comp_seed  : unsigned(15 downto 0) := x"1234";
    signal s_sub_seed   : unsigned(15 downto 0) := x"ABCD";
    signal s_seam_en    : std_logic := '1';
    signal s_relief     : std_logic := '1';     -- S8: texture emboss depth (0 soft, 1 deep)
    signal s_notsq      : std_logic := '0';     -- S9: 0 = square grid, 1 = off-grid overlap
    signal s_lumamod    : std_logic := '0';     -- derived: K4 above its off deadband
    signal s_logcabin   : std_logic := '0';     -- S10: 0 = crazy (random axis), 1 = log cabin
    signal s_palmode    : std_logic := '0';

    --------------------------------------------------------------------------
    -- Evolve engine (S8).  The quilt is memoryless — every pixel re-derives its
    -- leaf from hashes — so "one piece changed" cannot be stored.  Instead each
    -- leaf owns a slot k in a 65536-slot ring, and a per-frame phase sweeps a
    -- cursor `pos` around that ring:  a leaf's GENERATION is
    --     gen = pass + (k < pos)
    -- where pass/pos are the high/low halves of the phase.  Advancing pos by one
    -- therefore steps the generation of exactly the one slot it just passed, and
    -- the change persists until the cursor laps.  Cumulative, one at a time, no
    -- memory.  gen is split into a colour generation and a texture generation
    -- that alternate (see S10a), so a change moves EITHER the colour or the
    -- texture, never both.
    --
    -- The K3 knob is BIPOLAR and picks what k means, which is what makes the two
    -- evolve modes fall out of one mechanism:
    --   * right of centre (Random) — k is a mix of all 16 leaf seed bits, so the
    --     cursor visits pieces in scattered hash order.
    --   * left of centre (Snake) — k is the leaf's own POSITION on a serpentine
    --     over 8 px bands, so consecutive visits land on pieces that touch and
    --     the change crawls across the quilt.  Cells are visited in a hash-picked
    --     order within each 2x2 block so the walk wanders instead of reading as a
    --     scanline wipe.
    --   * centre — the rate is zero, so nothing evolves.  gen only ever takes the values pass, pass+1 or
    -- pass+2, so the three 4-bit slices it can contribute are precomputed here
    -- once per frame and the pixel path just picks one.
    type t_u4_3 is array (0 to 2) of unsigned(3 downto 0);
    signal s_evo_rate   : unsigned(27 downto 0) := (others => '0');
    signal s_evo_ph     : unsigned(27 downto 0) := (others => '0');
    signal s_evo_pos    : unsigned(15 downto 0) := (others => '0');
    signal s_evo_g      : t_u4_3 := (others => (others => '0'));
    signal s_evo_on     : std_logic := '0';     -- K3 off centre
    signal s_evo_snake  : std_logic := '0';     -- K3 left of centre: spatial walk
    -- Analog vsync serrates (multiple edges per field), so the phase increment
    -- needs the saw-active guard — a plain edge-triggered add would step several
    -- times per frame.  Idempotent latches in the same branch are safe as-is.
    signal s_saw_active : std_logic := '0';

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    -- Per-column luma hold: one sample per coarse-cell column, refreshed on the
    -- top line of each cell-row.  Keeps the subdivide decision constant across a
    -- whole cell so the rectangles stay clean while their density tracks video.
    type t_lumahold is array (0 to 63) of unsigned(7 downto 0);
    signal cell_luma : t_lumahold := (others => to_unsigned(128, 8));

    signal s0_x, s0_y : unsigned(11 downto 0);
    signal s0_luma    : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- Not-Square overlap: each cell gets a random origin jitter so cells overlap.
    -- A pixel is resolved against the 2x2 neighbourhood of jittered cells; the
    -- highest-priority covering cell wins, with the plain square cell as a gap
    -- fallback.  Candidate order k: (gx,gy) (gx-1,gy) (gx,gy-1) (gx-1,gy-1).
    type t_u16_4  is array (0 to 3) of unsigned(15 downto 0);
    type t_s14_4  is array (0 to 3) of signed(13 downto 0);
    type t_u4_4   is array (0 to 3) of unsigned(3 downto 0);
    -- S1c argmax priority: cov & rank(1:0) & z(3:0), packed so the compare
    -- tree is 7-bit and needs no adders at all.
    type t_u7_4   is array (0 to 3) of unsigned(6 downto 0);
    -- candidate bounds + seed only (px/py/dens are shared across all 4)
    type t_cand   is record
        x0, x1, y0, y1 : unsigned(11 downto 0);
        seed           : unsigned(15 downto 0);
    end record;
    type t_cand_4 is array (0 to 3) of t_cand;
    constant CK_X : integer_vector(0 to 3) := (0, 1, 0, 1);
    constant CK_Y : integer_vector(0 to 3) := (0, 0, 1, 1);

    -- S1a holds only the sliced/subtracted ORIGINS; the four init_seed hashes
    -- that used to sit in the same stage were the HD critical path (slice ->
    -- 14-bit subtract -> 16-bit hash, x4, all combinational), so they now get
    -- their own stage S1a2.
    signal s1a_ax, s1a_ay  : t_s14_4;                      -- per-candidate aligned origin
    signal s1a_px, s1a_py  : unsigned(11 downto 0);
    signal s1a_dens        : unsigned(7 downto 0);
    signal s1a_base        : t_node;                       -- plain square cell (seed unset)

    signal s1a2_hk         : t_u16_4;                      -- per-candidate cell hash/seed
    signal s1a2_ax, s1a2_ay: t_s14_4;
    signal s1a2_px, s1a2_py: unsigned(11 downto 0);
    signal s1a2_dens       : unsigned(7 downto 0);
    signal s1a2_base       : t_node;                       -- plain square cell (seed filled)

    -- S1b builds all 4 clamped candidate cells in parallel; S1c just argmaxes.
    signal s1b_cand        : t_cand_4;
    signal s1b_cov         : std_logic_vector(0 to 3);
    signal s1b_z           : t_u4_4;
    signal s1b_rank        : t_u4_4;                       -- size rank (0=1x1, 1, 2=2x2)
    signal s1b_px, s1b_py  : unsigned(11 downto 0);
    signal s1b_dens        : unsigned(7 downto 0);
    signal s1b_base        : t_node;

    signal s1_n  : t_node;                                 -- selected coarse cell node

    -- hash-stage outputs (decoded bits + node carrying new seed)
    signal h0_n, h1_n, h2_n, h3_n     : t_node;
    signal h0_or, h1_or, h2_or, h3_or : std_logic;
    signal h0_ds, h1_ds, h2_ds, h3_ds : std_logic;
    signal h0_sg, h1_sg, h2_sg, h3_sg : std_logic;
    signal h0_hf, h1_hf, h2_hf, h3_hf : std_logic;
    signal h0_w, h1_w, h2_w, h3_w     : unsigned(11 downto 0);   -- active-axis width
    signal h0_mid, h1_mid, h2_mid, h3_mid : unsigned(11 downto 0); -- (lo+hi)/2

    -- decide-stage outputs (node + orient passthrough + split coord + flag)
    signal d0_n, d1_n, d2_n, d3_n         : t_node;
    signal d0_or, d1_or, d2_or, d3_or     : std_logic;
    signal d0_dt, d1_dt, d2_dt, d3_dt     : std_logic;
    signal d0_nx, d1_nx, d2_nx, d3_nx     : unsigned(11 downto 0);

    -- descend-stage output nodes
    signal n0, n1, n2, n3 : t_node;

    -- leaf metrics, split across two stages
    signal s10a_lx, s10a_ly : unsigned(11 downto 0);
    signal s10a_m1, s10a_m2 : unsigned(11 downto 0);
    -- S10a also builds the leaf's evolve ring slot; the compare against the
    -- cursor plus the generation lookup are a stage of their own (S10a2) because
    -- the 16-bit compare will not fit alongside the leaf metrics.
    signal s10a_k           : unsigned(15 downto 0);
    signal s10a_cbase       : unsigned(3 downto 0);   -- leaf base colour number
    signal s10a_tbase       : unsigned(3 downto 0);   -- leaf base texture number
    signal s10a_p           : std_logic;              -- colour/texture alternation phase
    signal s10a_hide        : std_logic;

    -- Evolved leaf indices: the base colour/texture numbers already advanced by
    -- this leaf's colour/texture generation, so the 16-bit leaf seed never has to
    -- reach S10b.
    signal s10a2_lx, s10a2_ly : unsigned(11 downto 0);
    signal s10a2_m1, s10a2_m2 : unsigned(11 downto 0);
    signal s10a2_cidx       : unsigned(3 downto 0);
    signal s10a2_tidx       : unsigned(3 downto 0);
    signal s10a2_hide       : std_logic;

    signal s10_seamd      : unsigned(11 downto 0);
    signal s10_pidx       : unsigned(3 downto 0);
    signal s10_tval       : unsigned(1 downto 0);   -- texture shade (computed in S10b)
    signal s10_hide       : std_logic;

    -- palette + texture
    signal s11_y, s11_u, s11_v : unsigned(9 downto 0);
    signal s11_tval            : unsigned(1 downto 0);   -- 2-bit emboss shade
    signal s11_seamd           : unsigned(11 downto 0);
    signal s11_hide            : std_logic;

    -- assembled colour
    signal s12_y, s12_u, s12_v : unsigned(9 downto 0);
    signal s12_hide            : std_logic;

    -- output
    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Position counters + parameter latch (at vsync edge).
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_ph   : unsigned(27 downto 0);   -- evolve phase, next value
        variable v_rate : unsigned(27 downto 0);
        variable v_k3r  : unsigned(9 downto 0);
        variable v_mag  : unsigned(8 downto 0);     -- distance from centre
        variable v_mlo  : unsigned(6 downto 0);
        variable v_off  : boolean;
        variable v_p0, v_p1, v_p2 : unsigned(7 downto 0);   -- pass, pass+1, pass+2
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            -- precompute "this line is a cell-row top" (pixel_y constant per line)
            if (pixel_y and s_cellmask) = 0 then s_line_top <= '1';
            else                                 s_line_top <= '0'; end if;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if v_h_edge = '1' then        pixel_x <= (others => '0');
            elsif data_in.avid = '1' then pixel_x <= pixel_x + 1; end if;

            -- Saw-active flag for the evolve phase increment below.
            if data_in.avid = '1' then s_saw_active <= '1'; end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');

                -- Evolve phase: ONE step per field, guarded so a serrated analog
                -- vsync (several falling edges per field) cannot advance it more
                -- than once.  pos = ph(19:4) is the ring cursor, pass = ph(27:20)
                -- the lap count; gen is pass or pass+1, and the colour/texture
                -- split needs pass+2, so all three 4-bit slices are cut here.
                if s_saw_active = '1' then
                    s_saw_active <= '0';
                    v_ph := s_evo_ph + s_evo_rate;
                    s_evo_ph  <= v_ph;
                    s_evo_pos <= v_ph(19 downto 4);
                    v_p0 := v_ph(27 downto 20);
                    v_p1 := v_p0 + 1;
                    v_p2 := v_p0 + 2;
                    s_evo_g(0) <= v_p0(4 downto 1);
                    s_evo_g(1) <= v_p1(4 downto 1);
                    s_evo_g(2) <= v_p2(4 downto 1);
                end if;

                -- Grid scale: K1 top 2 bits -> coarse cell 256/128/64/32 px
                case to_integer(unsigned(registers_in(0)(9 downto 8))) is
                    when 0      => s_g <= 8; s_cellmask <= to_unsigned(255, 12);
                                            s_cellcenter <= to_unsigned(128, 12);
                    when 1      => s_g <= 7; s_cellmask <= to_unsigned(127, 12);
                                            s_cellcenter <= to_unsigned(64, 12);
                    when 2      => s_g <= 6; s_cellmask <= to_unsigned(63, 12);
                                            s_cellcenter <= to_unsigned(32, 12);
                    when others => s_g <= 5; s_cellmask <= to_unsigned(31, 12);
                                            s_cellcenter <= to_unsigned(16, 12);
                end case;

                -- K2 Seed drives BOTH layout seeds (composition + sub-piecing).
                s_comp_seed <= mix16(unsigned(registers_in(1)), x"9E37");
                s_sub_seed  <= mix16(unsigned(registers_in(1)), x"C2B5");

                -- K3 Evolve: BIPOLAR about centre.  Centre (within a deadband
                -- wide enough to swallow pot ADC noise) = rate 0 = frozen; right
                -- = Random mode; left = Snake mode.  Both halves ramp the same
                -- way, so the two modes run at matching speeds at matching
                -- distances from centre.
                v_k3r := unsigned(registers_in(2)(9 downto 0));
                v_off := true;
                if v_k3r >= to_unsigned(536, 10) then
                    v_off := false;
                    s_evo_on <= '1'; s_evo_snake <= '0';
                    v_mag := resize(v_k3r - 536, 9);
                elsif v_k3r <= to_unsigned(488, 10) then
                    v_off := false;
                    s_evo_on <= '1'; s_evo_snake <= '1';
                    v_mag := resize(488 - v_k3r, 9);
                else
                    s_evo_on <= '0'; s_evo_snake <= '0';
                    v_mag := (others => '0');
                end if;

                -- Magnitude -> phase step.  Four shift zones with carried offsets
                -- give a monotonic, roughly exponential 16..42945 span.  pos
                -- advances rate/16 slots per frame over a 65536-slot ring, so the
                -- bottom is about one piece a second and the top laps the whole
                -- quilt in ~24 frames.  Resize BEFORE shifting — shift_left keeps
                -- the operand's own width.
                v_mlo := v_mag(6 downto 0);
                case to_integer(v_mag(8 downto 7)) is
                    when 0      => v_rate := shift_left(resize(v_mlo, 28), 1) + 16;
                    when 1      => v_rate := shift_left(resize(v_mlo, 28), 4) + 271;
                    when 2      => v_rate := shift_left(resize(v_mlo, 28), 6) + 2304;
                    when others => v_rate := shift_left(resize(v_mlo, 28), 8) + 10433;
                end case;
                if v_off then s_evo_rate <= (others => '0');
                else          s_evo_rate <= v_rate; end if;
                -- K4 Threshold IS the Luma Mod control: fully anticlockwise turns
                -- luma modulation off entirely, anything above the deadband turns
                -- it on and sets the hide cut point.  The deadband (16/1023) keeps
                -- pot ADC noise from flickering the mode at the bottom of travel.
                s_lum_thr   <= unsigned(registers_in(3)(9 downto 2));
                if registers_in(3)(9 downto 4) = "000000" then s_lumamod <= '0';
                else                                           s_lumamod <= '1'; end if;
                s_tint      <= unsigned(registers_in(4)(9 downto 6));

                -- K6 Texture Scale: Fine / Small / Medium / Coarse.  Periods are
                -- coarse (16/32/64/128 px) so textures read as fabric weave at full
                -- HD, not 1-2 px noise.
                case to_integer(unsigned(registers_in(5)(9 downto 8))) is
                    when 0      => s_tex_shift <= 3;   -- ~16 px
                    when 1      => s_tex_shift <= 4;   -- ~32 px
                    when 2      => s_tex_shift <= 5;   -- ~64 px
                    when others => s_tex_shift <= 6;   -- ~128 px
                end case;

                s_seam_en <= registers_in(6)(0);
                s_relief  <= registers_in(6)(1);   -- S8 Relief: soft / deep
                s_notsq   <= registers_in(6)(2);
                s_logcabin <= registers_in(6)(3);  -- S10 Piecing: Crazy / Log Cabin
                s_palmode <= registers_in(6)(4);

                s_density   <= unsigned(registers_in(7)(9 downto 2));
                s_dens_floor <= 255 - unsigned(registers_in(7)(9 downto 2));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_x0, v_y0 : unsigned(11 downto 0);
        variable v_sz       : integer range 32 to 256;
        variable v_col      : integer range 0 to 63;
        variable v_densv    : unsigned(7 downto 0);
        variable v_hide     : std_logic;
        variable v_axk, v_ayk : signed(13 downto 0);
        variable v_hk       : unsigned(15 downto 0);   -- S1a2 candidate hash
        variable v_bn       : t_node;                  -- S1a2 base node + seed
        variable v_cells    : signed(13 downto 0);
        variable v_pxs, v_pys : signed(13 downto 0);
        variable v_jx, v_jy : unsigned(11 downto 0);
        variable v_x0s, v_y0s : signed(13 downto 0);
        variable v_szc      : integer range 32 to 256; -- cell size in px (= cellmask+1)
        variable v_wcx, v_wcy : integer range 32 to 512; -- candidate extent (1 or 2 cells)
        variable v_rank     : integer range 0 to 2;    -- candidate size rank (#2-cell axes)
        variable vv         : t_u7_4;
        variable v_m01, v_m23 : unsigned(6 downto 0);
        variable v_i01, v_i23, v_kw : integer range 0 to 3;
        variable dtmp       : t_dec;
        variable n          : t_node;
        variable lw, lh     : unsigned(11 downto 0);
        variable lx, ly     : unsigned(11 downto 0);
        variable bb, dd     : unsigned(11 downto 0);
        variable m1, m2     : unsigned(11 downto 0);
        variable seamd      : unsigned(11 downto 0);
        variable bidx       : integer range 0 to 15;
        variable vdiag      : unsigned(11 downto 0);
        variable v_adiag    : unsigned(11 downto 0);
        variable v_pm       : unsigned(11 downto 0);
        variable v_bv       : unsigned(3 downto 0);
        variable v_kmix     : unsigned(11 downto 0);   -- evolve: seed remix
        variable v_k, v_krnd, v_ksnk : unsigned(15 downto 0);  -- evolve ring slot
        variable v_qx, v_qy : unsigned(7 downto 0);    -- leaf corner, 8 px units
        variable v_bx, v_by, v_bxs, v_bsum : unsigned(6 downto 0);
        variable v_h2, v_cell : unsigned(1 downto 0);
        variable v_thi, v_td1, v_td2 : unsigned(9 downto 0);  -- emboss deltas
        variable v_cg, v_tg : unsigned(3 downto 0);    -- colour/texture generation
        variable v_ic, v_it : integer range 0 to 2;    -- which s_evo_g slice
        variable v_nv, v_nh : boolean;
        variable v_t2       : unsigned(1 downto 0);
        variable v_rx, v_ry, v_rd, v_ra : unsigned(1 downto 0);   -- 2-bit ramp fields
        variable v_bxb, v_byb : std_logic;                        -- checker bit (bit s)
        variable v_wxb, v_wyb : std_logic;                        -- block parity (bit s+1)
        variable v_drow, v_dcol : unsigned(1 downto 0);           -- dither cell (scaled)
        variable pidx       : integer range 0 to 15;
        variable ty, tu, tv : unsigned(9 downto 0);

        -- one hash stage: re-hash seed, decode split controls, and precompute the
        -- active-axis width w and midpoint mid (= (lo+hi)/2) so the decide stage's
        -- carry chain is just nx = mid +/- mag (one add) — keeps HD timing closing.
        procedure do_hash(signal q_n  : out t_node;
                          signal q_or : out std_logic;
                          signal q_ds : out std_logic;
                          signal q_sg : out std_logic;
                          signal q_hf : out std_logic;
                          signal q_w  : out unsigned(11 downto 0);
                          signal q_mid : out unsigned(11 downto 0);
                          constant lvl : integer;
                          n_in : t_node) is
            variable h  : unsigned(15 downto 0);
            variable nn : t_node;
            variable o  : std_logic;
            variable t8 : unsigned(7 downto 0);
            variable lo, hi : unsigned(11 downto 0);
            variable sum : unsigned(12 downto 0);
        begin
            h  := hash_lvl(n_in.seed, s_sub_seed, C_LC(lvl));
            nn := n_in; nn.seed := h;
            -- Piecing (S10): Crazy takes the split axis from the hash, Log Cabin
            -- alternates it strictly by depth so every patch nests as concentric
            -- frames.  In Log Cabin the axis is a compile-time constant per level,
            -- so it drops off the lo/hi select path entirely.
            if s_logcabin = '1' then
                if (lvl mod 2) = 1 then o := '1'; else o := '0'; end if;
            else
                o := h(0);
            end if;
            -- density tapers with depth.  Kept in unsigned(8) — an unconstrained
            -- integer here inferred a 32-bit subtract + compare in EVERY level.
            if n_in.dens > to_unsigned(lvl * 32, 8) then
                t8 := n_in.dens - to_unsigned(lvl * 32, 8);
            else
                t8 := (others => '0');
            end if;
            q_n  <= nn;
            q_or <= o;
            if (n_in.frozen = '0') and (h(15 downto 8) < t8) then
                q_ds <= '1';
            else
                q_ds <= '0';
            end if;
            q_sg <= h(2);
            q_hf <= h(3);
            -- active-axis bounds -> w and mid (parallel to the hash above)
            if o = '0' then lo := n_in.x0; hi := n_in.x1;
            else            lo := n_in.y0; hi := n_in.y1; end if;
            sum   := ('0' & lo) + ('0' & hi);
            q_w   <= hi - lo;
            q_mid <= sum(12 downto 1);                   -- (lo+hi)/2
        end procedure;
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- S0 + per-column luma sample/hold (drives video density).  Sample
            -- once per cell, at the cell centre on the cell-row's top line, so a
            -- whole cell shares one luma and its rectangles stay coherent.
            v_col := cell_col(pixel_x, s_g);
            s0_x    <= pixel_x;
            s0_y    <= pixel_y;
            s0_luma <= cell_luma(v_col);
            if s_line_top = '1' and (pixel_x and s_cellmask) = s_cellcenter then
                cell_luma(v_col) <= unsigned(data_in.y(9 downto 2));
            end if;

            -- S1a: square base cell + per-frame density, and the 2x2 candidate
            -- cells' hashes/aligned origins (for Not-Square overlap).
            case s_g is
                when 5      => v_x0 := s0_x(11 downto 5) & "00000";
                               v_y0 := s0_y(11 downto 5) & "00000";   v_sz := 32;
                when 6      => v_x0 := s0_x(11 downto 6) & "000000";
                               v_y0 := s0_y(11 downto 6) & "000000";  v_sz := 64;
                when 7      => v_x0 := s0_x(11 downto 7) & "0000000";
                               v_y0 := s0_y(11 downto 7) & "0000000"; v_sz := 128;
                when others => v_x0 := s0_x(11 downto 8) & "00000000";
                               v_y0 := s0_y(11 downto 8) & "00000000"; v_sz := 256;
            end case;
            -- Luma Mod: when on, the held cell luma both biases the subdivide
            -- density (bright -> more pieces) and hides cells whose held luma
            -- falls below the K4 threshold (0 = hide nothing, 255 = hide all
            -- but pure white).
            if s_lumamod = '1' then
                if s0_luma > s_dens_floor then v_densv := s0_luma - s_dens_floor;
                else                            v_densv := (others => '0'); end if;
                if s0_luma < s_lum_thr then v_hide := '1'; else v_hide := '0'; end if;
            else
                v_densv := s_density;
                v_hide  := '0';
            end if;
            -- square base cell (also the gap fallback in Not-Square).  Its seed
            -- is candidate 0's hash, filled in by S1a2.
            s1a_base.x0     <= v_x0;
            s1a_base.x1     <= v_x0 + v_sz;
            s1a_base.y0     <= v_y0;
            s1a_base.y1     <= v_y0 + v_sz;
            s1a_base.px     <= s0_x;
            s1a_base.py     <= s0_y;
            s1a_base.seed   <= (others => '0');
            s1a_base.dens   <= v_densv;
            s1a_base.hide   <= v_hide;
            s1a_base.frozen <= '0';
            -- 2x2 candidate cell origins (current + three up-left neighbours)
            for k in 0 to 3 loop
                v_axk := signed(resize(v_x0, 14)) - to_signed(CK_X(k) * v_sz, 14);
                v_ayk := signed(resize(v_y0, 14)) - to_signed(CK_Y(k) * v_sz, 14);
                s1a_ax(k) <= v_axk;
                s1a_ay(k) <= v_ayk;
            end loop;
            s1a_px   <= s0_x;
            s1a_py   <= s0_y;
            s1a_dens <= v_densv;

            -- S1a2: the four candidate cell hashes, on their own stage so the
            -- origin subtract and the 16-bit init_seed no longer chain inside one
            -- clock.  Candidate 0 IS the base cell, so its hash doubles as the
            -- base node's seed (one hash saved).
            v_bn := s1a_base;
            for k in 0 to 3 loop
                v_hk := init_seed(s_comp_seed,
                                  unsigned(s1a_ax(k)(11 downto 0)),
                                  unsigned(s1a_ay(k)(11 downto 0)));
                s1a2_hk(k) <= v_hk;
                if k = 0 then v_bn.seed := v_hk; end if;
            end loop;
            s1a2_base <= v_bn;
            s1a2_ax   <= s1a_ax;
            s1a2_ay   <= s1a_ay;
            s1a2_px   <= s1a_px;
            s1a2_py   <= s1a_py;
            s1a2_dens <= s1a_dens;

            -- S1b: size + jitter each candidate, test 2x2 coverage, and build the
            -- clamped candidate node — all 4 in parallel (no cross-candidate
            -- dependency), so S1c only has to argmax.  Each axis spans 1 or 2 grid
            -- cells (hash bits 7/6); a size-2 cell stays grid-aligned (no jitter)
            -- so its reach never exceeds 2 cells and the 2x2 neighbourhood holds.
            v_szc   := to_integer(s_cellmask) + 1;
            v_pxs   := signed(resize(s1a2_px, 14));
            v_pys   := signed(resize(s1a2_py, 14));
            for k in 0 to 3 loop
                v_rank := 0;
                if s1a2_hk(k)(7) = '1' and s1a2_hk(k)(3) = '1' then
                    v_wcx := v_szc + v_szc;  v_jx := (others => '0');  v_rank := v_rank + 1;
                else
                    v_wcx := v_szc;          v_jx := s1a2_hk(k)(11 downto 0) and s_cellmask;
                end if;
                if s1a2_hk(k)(6) = '1' and s1a2_hk(k)(2) = '1' then
                    v_wcy := v_szc + v_szc;  v_jy := (others => '0');  v_rank := v_rank + 1;
                else
                    v_wcy := v_szc;          v_jy := s1a2_hk(k)(15 downto 4) and s_cellmask;
                end if;
                v_x0s := s1a2_ax(k) + signed(resize(v_jx, 14));
                v_y0s := s1a2_ay(k) + signed(resize(v_jy, 14));
                -- Square mode folds in here as "nothing covers", so S1c's select
                -- is a plain zero-test and s_notsq stays off that critical path.
                if s_notsq = '1' and
                   (v_pxs >= v_x0s) and (v_pxs < v_x0s + to_signed(v_wcx, 14)) and
                   (v_pys >= v_y0s) and (v_pys < v_y0s + to_signed(v_wcy, 14)) then
                    s1b_cov(k) <= '1';
                else
                    s1b_cov(k) <= '0';
                end if;
                s1b_z(k)         <= s1a2_hk(k)(15 downto 12);
                s1b_rank(k)      <= to_unsigned(v_rank, 4);
                s1b_cand(k).x0   <= clamp0(v_x0s);
                s1b_cand(k).x1   <= clamp0(v_x0s + to_signed(v_wcx, 14));
                s1b_cand(k).y0   <= clamp0(v_y0s);
                s1b_cand(k).y1   <= clamp0(v_y0s + to_signed(v_wcy, 14));
                s1b_cand(k).seed <= s1a2_hk(k);
            end loop;
            s1b_px   <= s1a2_px;
            s1b_py   <= s1a2_py;
            s1b_dens <= s1a2_dens;
            s1b_base <= s1a2_base;

            -- S1c: tree argmax over covered candidates by priority; no cover
            -- (which includes Square mode, folded into cov above) -> base.  Priority is PACKED, not arithmetic (an
            -- integer_vector here inferred 32-bit adders + comparators and was
            -- the HD critical path): coverage in the top bit so covered always
            -- beats uncovered, then rank so larger cells win (a big cell owns
            -- its whole footprint and the grid line inside it disappears), then
            -- z as the tie-break.  Uncovered candidates zero out so the
            -- "nothing covered" test below is still a plain compare to 0.
            for k in 0 to 3 loop
                vv(k)(6)          := s1b_cov(k);
                vv(k)(5 downto 4) := s1b_rank(k)(1 downto 0);
                vv(k)(3 downto 0) := s1b_z(k);
                if s1b_cov(k) = '0' then vv(k) := (others => '0'); end if;
            end loop;
            if vv(0) >= vv(1) then v_m01 := vv(0); v_i01 := 0;
            else                   v_m01 := vv(1); v_i01 := 1; end if;
            if vv(2) >= vv(3) then v_m23 := vv(2); v_i23 := 2;
            else                   v_m23 := vv(3); v_i23 := 3; end if;
            if v_m01 >= v_m23 then v_kw := v_i01; else v_kw := v_i23; end if;
            if v_m01 = 0 and v_m23 = 0 then
                s1_n <= s1b_base;
            else
                s1_n.x0     <= s1b_cand(v_kw).x0;
                s1_n.x1     <= s1b_cand(v_kw).x1;
                s1_n.y0     <= s1b_cand(v_kw).y0;
                s1_n.y1     <= s1b_cand(v_kw).y1;
                s1_n.seed   <= s1b_cand(v_kw).seed;
                s1_n.px     <= s1b_px;
                s1_n.py     <= s1b_py;
                s1_n.dens   <= s1b_dens;
                s1_n.hide   <= s1b_base.hide;
                s1_n.frozen <= '0';
            end if;

            -- Level 0 : hash -> decide -> descend
            do_hash(h0_n, h0_or, h0_ds, h0_sg, h0_hf, h0_w, h0_mid, 0, s1_n);
            dtmp := calc_split(h0_w, h0_mid, h0_ds, h0_sg, h0_hf, C_JIT_MODE);
            d0_n <= h0_n; d0_or <= h0_or; d0_dt <= dtmp.doit; d0_nx <= dtmp.nx;
            n0 <= descend(d0_n, d0_or, d0_dt, d0_nx);

            -- Level 1
            do_hash(h1_n, h1_or, h1_ds, h1_sg, h1_hf, h1_w, h1_mid, 1, n0);
            dtmp := calc_split(h1_w, h1_mid, h1_ds, h1_sg, h1_hf, C_JIT_MODE);
            d1_n <= h1_n; d1_or <= h1_or; d1_dt <= dtmp.doit; d1_nx <= dtmp.nx;
            n1 <= descend(d1_n, d1_or, d1_dt, d1_nx);

            -- Level 2
            do_hash(h2_n, h2_or, h2_ds, h2_sg, h2_hf, h2_w, h2_mid, 2, n1);
            dtmp := calc_split(h2_w, h2_mid, h2_ds, h2_sg, h2_hf, C_JIT_MODE);
            d2_n <= h2_n; d2_or <= h2_or; d2_dt <= dtmp.doit; d2_nx <= dtmp.nx;
            n2 <= descend(d2_n, d2_or, d2_dt, d2_nx);

            -- Level 3
            do_hash(h3_n, h3_or, h3_ds, h3_sg, h3_hf, h3_w, h3_mid, 3, n2);
            dtmp := calc_split(h3_w, h3_mid, h3_ds, h3_sg, h3_hf, C_JIT_MODE);
            d3_n <= h3_n; d3_or <= h3_or; d3_dt <= dtmp.doit; d3_nx <= dtmp.nx;
            n3 <= descend(d3_n, d3_or, d3_dt, d3_nx);

            -- S10a: leaf-local coords + per-axis edge mins
            n  := n3;
            lw := n.x1 - n.x0;
            lh := n.y1 - n.y0;
            lx := n.px - n.x0;
            ly := n.py - n.y0;
            bb := (lw - 1) - lx;
            dd := (lh - 1) - ly;
            if lx < bb then m1 := lx; else m1 := bb; end if;
            if ly < dd then m2 := ly; else m2 := dd; end if;
            -- Evolve, part 1: the leaf's ring slot k.
            --
            -- RANDOM (K3 right): mix all 16 seed bits so consecutive slots do NOT
            -- share a base colour — k(7:4) becomes colour xor texture.  Feeding
            -- raw seed bits would put the colour field in k's low bits and the
            -- cursor would visibly sweep the palette in order.
            --
            -- SNAKE (K3 left): k is the leaf's own top-left corner, quantised to
            -- 8 px (the minimum split, so distinct leaves land in distinct slots)
            -- and laid out as a serpentine — the column index is inverted on odd
            -- bands, which is just an xor with the band's low bit.  Sweeping k in
            -- order therefore walks the quilt band by band, alternating direction,
            -- and every step lands on a piece touching the last.  Within each 2x2
            -- block the four cells are visited in a hash-picked order (an xor
            -- permutation, keyed off a cheap add of the block coords so it is not
            -- purely linear) so the walk wanders locally instead of reading as a
            -- scanline wipe.
            v_kmix := n.seed(3 downto 0) & n.seed(15 downto 8);
            v_krnd := n.seed(15 downto 12) & (n.seed(11 downto 0) xor v_kmix);
            v_qx   := n.x0(10 downto 3);
            v_qy   := n.y0(10 downto 3);
            v_bx   := v_qx(7 downto 1);
            v_by   := v_qy(7 downto 1);
            v_bsum := v_bx + v_by;
            v_h2   := v_bsum(1 downto 0) xor v_bx(3 downto 2);
            v_bxs  := v_bx xor (6 downto 0 => v_by(0));
            v_cell := (v_qy(0) & v_qx(0)) xor v_h2;
            v_ksnk := v_by & v_bxs & v_cell;
            if s_evo_snake = '1' then v_k := v_ksnk; else v_k := v_krnd; end if;
            s10a_lx    <= lx;
            s10a_ly    <= ly;
            s10a_m1    <= m1;
            s10a_m2    <= m2;
            s10a_k     <= v_k;
            s10a_cbase <= n.seed(7 downto 4);
            s10a_tbase <= n.seed(11 downto 8);
            s10a_p     <= n.seed(12);
            s10a_hide  <= n.hide;

            -- Evolve, part 2 (S10a2): gen = pass + (k < pos).  The colour and
            -- texture generations alternate off the per-leaf phase bit, so each
            -- step of gen moves EXACTLY ONE of them:
            --     p=0: cgen = gen,   tgen = gen+1
            --     p=1: cgen = gen+1, tgen = gen
            -- Each is one of pass/pass+1/pass+2, already sliced per frame into
            -- s_evo_g, so this is a 3:1 mux — no arithmetic on gen at pixel rate.
            -- The indices then just advance the leaf's base colour / texture
            -- number, which wraps in 4 bits for free.
            v_ic := 0;
            v_it := 0;
            if s10a_k < s_evo_pos then v_ic := v_ic + 1; v_it := v_it + 1; end if;
            if s10a_p = '1' then v_ic := v_ic + 1; else v_it := v_it + 1; end if;
            if s_evo_on = '1' then
                v_cg := s_evo_g(v_ic);
                v_tg := s_evo_g(v_it);
            else
                v_cg := (others => '0');
                v_tg := (others => '0');
            end if;
            s10a2_lx   <= s10a_lx;
            s10a2_ly   <= s10a_ly;
            s10a2_m1   <= s10a_m1;
            s10a2_m2   <= s10a_m2;
            s10a2_cidx <= s10a_cbase + v_cg;
            s10a2_tidx <= s10a_tbase + v_tg;
            s10a2_hide <= s10a_hide;

            -- S10b: seam distance + palette index + texture emboss shade.  The
            -- texture is computed HERE (one stage ahead of the palette fetch) so
            -- the 16-way pattern logic and the palette muxes don't share a stage.
            -- Texture -> 2-bit shade: stripe/diagonal types ramp across the period
            -- (rounded "thread" -> depth); the rest are deep binary.  Shift/AND/add.
            if s10a2_m1 < s10a2_m2 then seamd := s10a2_m1; else seamd := s10a2_m2; end if;
            s10_seamd <= seamd;
            s10_pidx  <= s10a2_cidx xor s_tint;
            s10_hide  <= s10a2_hide;

            vdiag   := s10a2_lx + s10a2_ly;
            v_adiag := s10a2_lx + (not s10a2_ly);
            -- All texture coords are read as small fixed bit-slices selected by the
            -- Texture knob (a 4:1 mux), NOT 12-bit barrel shifters — the texture
            -- only needs a 2-bit ramp field (coord>>(s-1))(1:0) and a couple of
            -- single bits.  This both scales every pattern with the knob and keeps
            -- HD timing (barrel shifters here were the congestion bottleneck).
            --   v_r* = 2-bit ramp field at [s:s-1]; v_*b = bit s; v_w*b = bit s+1.
            case s_tex_shift is
                when 3 =>
                    v_rx := s10a2_lx(3 downto 2); v_ry := s10a2_ly(3 downto 2);
                    v_rd := vdiag(3 downto 2);   v_ra := v_adiag(3 downto 2);
                    v_bxb := s10a2_lx(3); v_byb := s10a2_ly(3);
                    v_wxb := s10a2_lx(4); v_wyb := s10a2_ly(4);
                    v_drow := s10a2_ly(2 downto 1); v_dcol := s10a2_lx(2 downto 1);
                    v_pm := to_unsigned(7, 12);
                when 4 =>
                    v_rx := s10a2_lx(4 downto 3); v_ry := s10a2_ly(4 downto 3);
                    v_rd := vdiag(4 downto 3);   v_ra := v_adiag(4 downto 3);
                    v_bxb := s10a2_lx(4); v_byb := s10a2_ly(4);
                    v_wxb := s10a2_lx(5); v_wyb := s10a2_ly(5);
                    v_drow := s10a2_ly(3 downto 2); v_dcol := s10a2_lx(3 downto 2);
                    v_pm := to_unsigned(15, 12);
                when 5 =>
                    v_rx := s10a2_lx(5 downto 4); v_ry := s10a2_ly(5 downto 4);
                    v_rd := vdiag(5 downto 4);   v_ra := v_adiag(5 downto 4);
                    v_bxb := s10a2_lx(5); v_byb := s10a2_ly(5);
                    v_wxb := s10a2_lx(6); v_wyb := s10a2_ly(6);
                    v_drow := s10a2_ly(4 downto 3); v_dcol := s10a2_lx(4 downto 3);
                    v_pm := to_unsigned(31, 12);
                when others =>
                    v_rx := s10a2_lx(6 downto 5); v_ry := s10a2_ly(6 downto 5);
                    v_rd := vdiag(6 downto 5);   v_ra := v_adiag(6 downto 5);
                    v_bxb := s10a2_lx(6); v_byb := s10a2_ly(6);
                    v_wxb := s10a2_lx(7); v_wyb := s10a2_ly(7);
                    v_drow := s10a2_ly(5 downto 4); v_dcol := s10a2_lx(5 downto 4);
                    v_pm := to_unsigned(63, 12);
            end case;
            bidx := to_integer(v_drow & v_dcol);
            v_bv := C_BAYER(bidx);
            v_nv := (s10a2_lx and v_pm) < 2;    -- near a vertical cell line
            v_nh := (s10a2_ly and v_pm) < 2;    -- near a horizontal cell line
            v_t2 := "01";
            case to_integer(s10a2_tidx) is
                when 0  => v_t2 := "01";                                       -- solid
                when 1  => v_t2 := v_rx;                                       -- v ridges
                when 2  => v_t2 := v_ry;                                       -- h ridges
                when 3  => v_t2 := v_rd;                                       -- diagonal
                when 4  => v_t2 := v_ra;                                       -- anti-diagonal
                when 5  =>                                                     -- checker (deep)
                    if v_bxb = v_byb then v_t2 := "00"; else v_t2 := "11"; end if;
                when 6  =>                                                     -- cross-hatch
                    if v_nv or v_nh then v_t2 := "11"; else v_t2 := "00"; end if;
                when 7  => v_t2 := v_bv(3 downto 2);                           -- ordered dither
                when 8  =>                                                     -- dots
                    if v_bv > 11 then v_t2 := "11"; elsif v_bv > 6 then v_t2 := "10";
                    else v_t2 := "00"; end if;
                when 9  =>                                                     -- v pinstripe
                    if v_nv then v_t2 := "11"; else v_t2 := "01"; end if;
                when 10 =>                                                     -- h pinstripe
                    if v_nh then v_t2 := "11"; else v_t2 := "01"; end if;
                when 11 =>                                                     -- basketweave
                    if (v_wxb xor v_wyb) = '0' then v_t2 := v_ry;
                    else v_t2 := v_rx; end if;
                when 12 =>                                                     -- herringbone
                    if v_wyb = '0' then v_t2 := v_rd;
                    else v_t2 := v_ra; end if;
                when 13 =>                                                     -- woven grid
                    if v_nv or v_nh then v_t2 := "11"; else v_t2 := v_rx; end if;
                when 14 => v_t2 := v_rd;                                       -- diagonal (alt)
                when others => v_t2 := v_ry;                                   -- h ridges (alt)
            end case;
            s10_tval <= v_t2;

            -- S11: palette fetch (texture shade just rides through)
            pidx := to_integer(s10_pidx);
            if s_palmode = '0' then
                ty := FAB_Y(pidx); tu := FAB_U(pidx); tv := FAB_V(pidx);
            else
                ty := RNB_Y(pidx); tu := RNB_U(pidx); tv := RNB_V(pidx);
            end if;
            s11_y <= ty; s11_u <= tu; s11_v <= tv;
            s11_tval  <= s10_tval;
            s11_seamd <= s10_seamd;
            s11_hide  <= s10_hide;

            -- S12: assemble colour (seam > texture emboss > flat).  The 2-bit
            -- texture shade maps to a luma delta: 00 highlight, 01 flat,
            -- 10 shadow, 11 deep shadow -> embossed, deeper-looking weave.
            if s_seam_en = '1' and s11_seamd < to_unsigned(SEAM_W, 12) then
                s12_y <= to_unsigned(64, 10);
                s12_u <= C_MID;
                s12_v <= C_MID;
            else
                -- S8 Relief picks the emboss depth: Deep is the full weave, Soft
                -- halves every delta for a flatter, printed-cotton read.  Two
                -- constant sets, so it is a mux ahead of the existing add/sub.
                if s_relief = '1' then
                    v_thi := to_unsigned(TEX_HI, 10);
                    v_td1 := to_unsigned(TEX_D1, 10);
                    v_td2 := to_unsigned(TEX_D2, 10);
                else
                    v_thi := to_unsigned(TEX_HI / 2, 10);
                    v_td1 := to_unsigned(TEX_D1 / 2, 10);
                    v_td2 := to_unsigned(TEX_D2 / 2, 10);
                end if;
                case s11_tval is
                    when "00" =>   -- highlight (valley)
                        if s11_y < (to_unsigned(1023, 10) - v_thi) then
                            s12_y <= s11_y + v_thi;
                        else
                            s12_y <= to_unsigned(1023, 10);
                        end if;
                    when "10" =>   -- shadow
                        if s11_y > v_td1 then
                            s12_y <= s11_y - v_td1;
                        else
                            s12_y <= (others => '0');
                        end if;
                    when "11" =>   -- deep shadow (thread core)
                        if s11_y > v_td2 then
                            s12_y <= s11_y - v_td2;
                        else
                            s12_y <= (others => '0');
                        end if;
                    when others => -- flat
                        s12_y <= s11_y;
                end case;
                s12_u <= s11_u;
                s12_v <= s11_v;
            end if;
            s12_hide <= s11_hide;

            -- S_out: register output.  Luma Mod hides cells over negative luma —
            -- those show black.
            if s12_hide = '1' then
                s_io.y <= (others => '0');
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                s_io.y <= std_logic_vector(s12_y);
                s_io.u <= std_logic_vector(s12_u);
                s_io.v <= std_logic_vector(s12_v);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture quilt;
