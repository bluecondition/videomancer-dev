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
-- Controls: K1 Grid Scale, K2 Composition Seed, K3 Sub-piece Seed, K4 Split
-- Jitter, K5 Palette Tint, K6 Texture Scale, S7 Seams, S8 Texture, S9 Grid
-- (Square / Not-Square off-grid overlap), S10 Luma Mod (incoming luma drives
-- per-cell piecing + hides cells over negative luma -> black), S11 Palette
-- (Fabric/Rainbow), P12 Density (KEY).  16 embossed texture types.
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

    constant LATENCY : natural := 20;
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
        hide           : std_logic;              -- Luma Mod: cell over negative luma
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
        variable c : integer;
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
    signal s_dens_floor : integer range 0 to 255 := 95;     -- 255 - density (precomputed)
    signal s_line_top   : std_logic := '0';                 -- on a cell-row's top line
    signal s_jit_mode   : unsigned(1 downto 0) := to_unsigned(2, 2);
    signal s_tint       : unsigned(3 downto 0) := (others => '0');
    signal s_tex_shift  : integer range 3 to 6 := 5;
    signal s_comp_seed  : unsigned(15 downto 0) := x"1234";
    signal s_sub_seed   : unsigned(15 downto 0) := x"ABCD";
    signal s_seam_en    : std_logic := '1';
    signal s_tex_en     : std_logic := '1';     -- S8: texture on/off
    signal s_notsq      : std_logic := '0';     -- S9: 0 = square grid, 1 = off-grid overlap
    signal s_lumamod    : std_logic := '0';     -- S10: luma masks + drives density
    signal s_palmode    : std_logic := '0';

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
    -- candidate bounds + seed only (px/py/dens are shared across all 4)
    type t_cand   is record
        x0, x1, y0, y1 : unsigned(11 downto 0);
        seed           : unsigned(15 downto 0);
    end record;
    type t_cand_4 is array (0 to 3) of t_cand;
    constant CK_X : integer_vector(0 to 3) := (0, 1, 0, 1);
    constant CK_Y : integer_vector(0 to 3) := (0, 0, 1, 1);

    signal s1a_hk          : t_u16_4;                      -- per-candidate cell hash/seed
    signal s1a_ax, s1a_ay  : t_s14_4;                      -- per-candidate aligned origin
    signal s1a_px, s1a_py  : unsigned(11 downto 0);
    signal s1a_dens        : unsigned(7 downto 0);
    signal s1a_base        : t_node;                       -- plain square cell

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
    signal s10a_seed        : unsigned(15 downto 0);
    signal s10a_hide        : std_logic;

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

            if v_v_edge = '1' then
                pixel_y <= (others => '0');

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

                s_comp_seed <= mix16(unsigned(registers_in(1)), x"9E37");
                s_sub_seed  <= mix16(unsigned(registers_in(2)), x"C2B5");
                s_jit_mode  <= unsigned(registers_in(3)(9 downto 8));
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
                s_tex_en  <= registers_in(6)(1);   -- S8 texture on/off
                s_notsq   <= registers_in(6)(2);
                s_lumamod <= registers_in(6)(3);   -- S10
                s_palmode <= registers_in(6)(4);

                s_density   <= unsigned(registers_in(7)(9 downto 2));
                s_dens_floor <= 255 - to_integer(unsigned(registers_in(7)(9 downto 2)));
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
        variable v_sz       : integer;
        variable v_col      : integer range 0 to 63;
        variable v_dt       : integer;
        variable v_densv    : unsigned(7 downto 0);
        variable v_hide     : std_logic;
        variable v_axk, v_ayk : signed(13 downto 0);
        variable v_cells    : signed(13 downto 0);
        variable v_pxs, v_pys : signed(13 downto 0);
        variable v_jx, v_jy : unsigned(11 downto 0);
        variable v_x0s, v_y0s : signed(13 downto 0);
        variable v_szc      : integer;                 -- cell size in px (= cellmask+1)
        variable v_wcx, v_wcy : integer;               -- candidate extent in px (1 or 2 cells)
        variable v_rank     : integer range 0 to 2;    -- candidate size rank (#2-cell axes)
        variable vv         : integer_vector(0 to 3);
        variable v_m01, v_m23 : integer;
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
        variable v_s, v_sm1 : integer range 0 to 7;
        variable v_pm       : unsigned(11 downto 0);
        variable v_bv       : unsigned(3 downto 0);
        variable v_nv, v_nh : boolean;
        variable v_t2       : unsigned(1 downto 0);
        variable v_vx, v_vy, v_vd, v_va : unsigned(11 downto 0);  -- ramp (>>sm1)
        variable v_bx, v_by : unsigned(11 downto 0);              -- coarse bit (>>s)
        variable v_wx, v_wy : unsigned(11 downto 0);              -- block parity (>>s+1)
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
            variable t8 : integer;
            variable lo, hi : unsigned(11 downto 0);
            variable sum : unsigned(12 downto 0);
        begin
            h  := hash_lvl(n_in.seed, s_sub_seed, C_LC(lvl));
            nn := n_in; nn.seed := h;
            o  := h(0);
            t8 := to_integer(n_in.dens) - lvl * 32;      -- density tapers with depth
            if t8 < 0 then t8 := 0; end if;
            q_n  <= nn;
            q_or <= o;
            if (n_in.frozen = '0') and (to_integer(h(15 downto 8)) < t8) then
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
            -- density (bright -> more pieces) and hides cells over negative luma
            -- (<= mid, i.e. <=128 in the 8-bit hold).
            if s_lumamod = '1' then
                v_dt := to_integer(s0_luma) - s_dens_floor;
                if v_dt < 0 then v_dt := 0; end if;
                v_densv := to_unsigned(v_dt, 8);
                if s0_luma <= to_unsigned(128, 8) then v_hide := '1'; else v_hide := '0'; end if;
            else
                v_densv := s_density;
                v_hide  := '0';
            end if;
            -- square base cell (also the gap fallback in Not-Square)
            s1a_base.x0     <= v_x0;
            s1a_base.x1     <= v_x0 + v_sz;
            s1a_base.y0     <= v_y0;
            s1a_base.y1     <= v_y0 + v_sz;
            s1a_base.px     <= s0_x;
            s1a_base.py     <= s0_y;
            s1a_base.seed   <= init_seed(s_comp_seed, v_x0, v_y0);
            s1a_base.dens   <= v_densv;
            s1a_base.hide   <= v_hide;
            s1a_base.frozen <= '0';
            -- 2x2 candidate cells (current + three up-left neighbours)
            for k in 0 to 3 loop
                v_axk := signed(resize(v_x0, 14)) - to_signed(CK_X(k) * v_sz, 14);
                v_ayk := signed(resize(v_y0, 14)) - to_signed(CK_Y(k) * v_sz, 14);
                s1a_ax(k) <= v_axk;
                s1a_ay(k) <= v_ayk;
                s1a_hk(k) <= init_seed(s_comp_seed,
                                       unsigned(v_axk(11 downto 0)),
                                       unsigned(v_ayk(11 downto 0)));
            end loop;
            s1a_px   <= s0_x;
            s1a_py   <= s0_y;
            s1a_dens <= v_densv;

            -- S1b: size + jitter each candidate, test 2x2 coverage, and build the
            -- clamped candidate node — all 4 in parallel (no cross-candidate
            -- dependency), so S1c only has to argmax.  Each axis spans 1 or 2 grid
            -- cells (hash bits 7/6); a size-2 cell stays grid-aligned (no jitter)
            -- so its reach never exceeds 2 cells and the 2x2 neighbourhood holds.
            v_szc   := to_integer(s_cellmask) + 1;
            v_pxs   := signed(resize(s1a_px, 14));
            v_pys   := signed(resize(s1a_py, 14));
            for k in 0 to 3 loop
                v_rank := 0;
                if s1a_hk(k)(7) = '1' and s1a_hk(k)(3) = '1' then
                    v_wcx := v_szc + v_szc;  v_jx := (others => '0');  v_rank := v_rank + 1;
                else
                    v_wcx := v_szc;          v_jx := s1a_hk(k)(11 downto 0) and s_cellmask;
                end if;
                if s1a_hk(k)(6) = '1' and s1a_hk(k)(2) = '1' then
                    v_wcy := v_szc + v_szc;  v_jy := (others => '0');  v_rank := v_rank + 1;
                else
                    v_wcy := v_szc;          v_jy := s1a_hk(k)(15 downto 4) and s_cellmask;
                end if;
                v_x0s := s1a_ax(k) + signed(resize(v_jx, 14));
                v_y0s := s1a_ay(k) + signed(resize(v_jy, 14));
                if (v_pxs >= v_x0s) and (v_pxs < v_x0s + to_signed(v_wcx, 14)) and
                   (v_pys >= v_y0s) and (v_pys < v_y0s + to_signed(v_wcy, 14)) then
                    s1b_cov(k) <= '1';
                else
                    s1b_cov(k) <= '0';
                end if;
                s1b_z(k)         <= s1a_hk(k)(15 downto 12);
                s1b_rank(k)      <= to_unsigned(v_rank, 4);
                s1b_cand(k).x0   <= clamp0(v_x0s);
                s1b_cand(k).x1   <= clamp0(v_x0s + to_signed(v_wcx, 14));
                s1b_cand(k).y0   <= clamp0(v_y0s);
                s1b_cand(k).y1   <= clamp0(v_y0s + to_signed(v_wcy, 14));
                s1b_cand(k).seed <= s1a_hk(k);
            end loop;
            s1b_px   <= s1a_px;
            s1b_py   <= s1a_py;
            s1b_dens <= s1a_dens;
            s1b_base <= s1a_base;

            -- S1c: tree argmax over covered candidates by priority (covered always
            -- beats uncovered via the +16 bias); Square mode or no cover -> base.
            -- priority: larger cells win first (so a big cell owns its whole
            -- footprint and the grid line inside it disappears); z breaks ties.
            for k in 0 to 3 loop
                if s1b_cov(k) = '1' then
                    vv(k) := 16 + 16 * to_integer(s1b_rank(k)) + to_integer(s1b_z(k));
                else
                    vv(k) := 0;
                end if;
            end loop;
            if vv(0) >= vv(1) then v_m01 := vv(0); v_i01 := 0;
            else                   v_m01 := vv(1); v_i01 := 1; end if;
            if vv(2) >= vv(3) then v_m23 := vv(2); v_i23 := 2;
            else                   v_m23 := vv(3); v_i23 := 3; end if;
            if v_m01 >= v_m23 then v_kw := v_i01; else v_kw := v_i23; end if;
            if s_notsq = '0' or (v_m01 = 0 and v_m23 = 0) then
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
            dtmp := calc_split(h0_w, h0_mid, h0_ds, h0_sg, h0_hf, s_jit_mode);
            d0_n <= h0_n; d0_or <= h0_or; d0_dt <= dtmp.doit; d0_nx <= dtmp.nx;
            n0 <= descend(d0_n, d0_or, d0_dt, d0_nx);

            -- Level 1
            do_hash(h1_n, h1_or, h1_ds, h1_sg, h1_hf, h1_w, h1_mid, 1, n0);
            dtmp := calc_split(h1_w, h1_mid, h1_ds, h1_sg, h1_hf, s_jit_mode);
            d1_n <= h1_n; d1_or <= h1_or; d1_dt <= dtmp.doit; d1_nx <= dtmp.nx;
            n1 <= descend(d1_n, d1_or, d1_dt, d1_nx);

            -- Level 2
            do_hash(h2_n, h2_or, h2_ds, h2_sg, h2_hf, h2_w, h2_mid, 2, n1);
            dtmp := calc_split(h2_w, h2_mid, h2_ds, h2_sg, h2_hf, s_jit_mode);
            d2_n <= h2_n; d2_or <= h2_or; d2_dt <= dtmp.doit; d2_nx <= dtmp.nx;
            n2 <= descend(d2_n, d2_or, d2_dt, d2_nx);

            -- Level 3
            do_hash(h3_n, h3_or, h3_ds, h3_sg, h3_hf, h3_w, h3_mid, 3, n2);
            dtmp := calc_split(h3_w, h3_mid, h3_ds, h3_sg, h3_hf, s_jit_mode);
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
            s10a_lx   <= lx;
            s10a_ly   <= ly;
            s10a_m1   <= m1;
            s10a_m2   <= m2;
            s10a_seed <= n.seed;
            s10a_hide <= n.hide;

            -- S10b: seam distance + palette index + texture emboss shade.  The
            -- texture is computed HERE (one stage ahead of the palette fetch) so
            -- the 16-way pattern logic and the palette muxes don't share a stage.
            -- Texture -> 2-bit shade: stripe/diagonal types ramp across the period
            -- (rounded "thread" -> depth); the rest are deep binary.  Shift/AND/add.
            if s10a_m1 < s10a_m2 then seamd := s10a_m1; else seamd := s10a_m2; end if;
            s10_seamd <= seamd;
            s10_pidx  <= s10a_seed(7 downto 4) xor s_tint;
            s10_hide  <= s10a_hide;

            v_s     := s_tex_shift;
            v_sm1   := s_tex_shift - 1;
            vdiag   := s10a_lx + s10a_ly;
            v_adiag := s10a_lx + (not s10a_ly);
            -- dither/dots cell from a fixed coarse slice (bits 5..3 -> ~8 px cells),
            -- no barrel shifter (keeps timing) and not 1-px noise.
            bidx    := to_integer(s10a_ly(4 downto 3) & s10a_lx(4 downto 3));
            v_bv    := C_BAYER(bidx);
            case s_tex_shift is
                when 3      => v_pm := to_unsigned( 7, 12);
                when 4      => v_pm := to_unsigned(15, 12);
                when 5      => v_pm := to_unsigned(31, 12);
                when others => v_pm := to_unsigned(63, 12);
            end case;
            v_nv := (s10a_lx and v_pm) < 2;    -- near a vertical cell line
            v_nh := (s10a_ly and v_pm) < 2;    -- near a horizontal cell line
            v_vx := s10a_lx  srl v_sm1;        -- ramp coords (slice low 2 bits)
            v_vy := s10a_ly  srl v_sm1;
            v_vd := vdiag    srl v_sm1;
            v_va := v_adiag  srl v_sm1;
            v_bx := s10a_lx  srl v_s;          -- coarse on/off bit
            v_by := s10a_ly  srl v_s;
            v_wx := s10a_lx  srl (v_s + 1);    -- block parity
            v_wy := s10a_ly  srl (v_s + 1);
            v_t2 := "01";
            case to_integer(s10a_seed(11 downto 8)) is
                when 0  => v_t2 := "01";                                       -- solid
                when 1  => v_t2 := v_vx(1 downto 0);                           -- v ridges
                when 2  => v_t2 := v_vy(1 downto 0);                           -- h ridges
                when 3  => v_t2 := v_vd(1 downto 0);                           -- diagonal
                when 4  => v_t2 := v_va(1 downto 0);                           -- anti-diagonal
                when 5  =>                                                     -- checker (deep)
                    if v_bx(0) = v_by(0) then v_t2 := "00"; else v_t2 := "11"; end if;
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
                    if (v_wx(0) xor v_wy(0)) = '0' then v_t2 := v_vy(1 downto 0);
                    else v_t2 := v_vx(1 downto 0); end if;
                when 12 =>                                                     -- herringbone
                    if v_wy(0) = '0' then v_t2 := v_vd(1 downto 0);
                    else v_t2 := v_va(1 downto 0); end if;
                when 13 =>                                                     -- woven grid
                    if v_nv or v_nh then v_t2 := "11"; else v_t2 := v_vx(1 downto 0); end if;
                when 14 => v_t2 := v_vd(1 downto 0);                           -- diagonal (alt)
                when others => v_t2 := v_vy(1 downto 0);                       -- h ridges (alt)
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
            elsif s_tex_en = '1' then
                case s11_tval is
                    when "00" =>   -- highlight (valley)
                        if s11_y < to_unsigned(1023 - TEX_HI, 10) then
                            s12_y <= s11_y + TEX_HI;
                        else
                            s12_y <= to_unsigned(1023, 10);
                        end if;
                    when "10" =>   -- shadow
                        if s11_y > to_unsigned(TEX_D1, 10) then
                            s12_y <= s11_y - TEX_D1;
                        else
                            s12_y <= (others => '0');
                        end if;
                    when "11" =>   -- deep shadow (thread core)
                        if s11_y > to_unsigned(TEX_D2, 10) then
                            s12_y <= s11_y - TEX_D2;
                        else
                            s12_y <= (others => '0');
                        end if;
                    when others => -- flat
                        s12_y <= s11_y;
                end case;
                s12_u <= s11_u;
                s12_v <= s11_v;
            else
                s12_y <= s11_y;
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
