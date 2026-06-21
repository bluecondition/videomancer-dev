-- mondrian.vhd  (v0.1 — De Stijl composition generator)
--
-- Paints the screen in the style of Piet Mondrian: a mostly-white canvas carved
-- into clean axis-aligned rectangles by thick black grid lines, with a scattering
-- of blocks filled in primary red, yellow and blue (and, optionally, black).
--
-- This is a repaint of the "quilt" program: the recursive binary-subdivision
-- pipeline that carves each coarse grid cell into nested sub-rectangles is reused
-- (no per-pixel multiply, no line buffer, no BRAM).  Only the *leaf colouring* and
-- the *seam rendering* differ:
--   * Quilt's uniform 16-colour palette pick becomes a WEIGHTED pick that favours
--     the white background; only a fraction of leaves take a primary colour.
--   * Quilt's thin grey seams become thick PURE-BLACK grid lines (Line Width knob).
--   * Texture is dropped — Mondrian blocks are flat colour.
--
-- S9 Not-Square = cell MERGING: in a base grid, some cells grow to span their
-- right / down / right-down neighbour (2x1, 1x2 or 2x2).  The larger cell wins the
-- overlap and renders FLAT, so the grid line between the fused cells disappears and
-- a larger rectangle forms.  Cells stay grid-aligned, so coverage is pure hash-bit
-- logic (no coordinate compares) — cheap enough to close HD timing.  Square mode
-- (S9=0) keeps every cell 1x1 = the plain grid.
--
-- Controls: K1 Grid Scale, K2 Composition, K3 Sub-Pieces, K4 Split Jitter,
-- K5 Colour Amount, K6 Line Width, S7 Grid Lines, S8 Black Blocks, S9 Grid
-- (Square / Not-Square cell merge), S10 Video Fill, S11 Luma Mod (show the
-- composition only where input luma is positive, else black), P12 Density.
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

architecture mondrian of program_top is

    constant LATENCY : natural := 21;
    constant DMAX    : natural := 4;          -- subdivision levels
    constant MIN_W   : natural := 8;          -- don't split a span narrower than this

    constant C_MID   : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant LINE_Y  : unsigned(9 downto 0) := to_unsigned(48, 10);   -- grid-line luma (deep black)

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
        frozen         : std_logic;
    end record;

    -- decide-stage result: split coordinate + "do it" flag.
    type t_dec is record
        nx   : unsigned(11 downto 0);
        doit : std_logic;
    end record;

    --------------------------------------------------------------------------
    -- Not-Square cell merge: the 2x2 up-left candidate neighbourhood.  A cell can
    -- grow right/down by one cell, so a pixel can be covered by its own cell or one
    -- of the three up-left neighbours grown toward it.  Order: (gx,gy) (gx-1,gy)
    -- (gx,gy-1) (gx-1,gy-1).
    --------------------------------------------------------------------------
    type t_u16_4 is array (0 to 3) of unsigned(15 downto 0);
    type t_u12_4 is array (0 to 3) of unsigned(11 downto 0);
    constant CK_X : integer_vector(0 to 3) := (0, 1, 0, 1);
    constant CK_Y : integer_vector(0 to 3) := (0, 0, 1, 1);

    --------------------------------------------------------------------------
    -- Mondrian palette.  index 0=White 1=Red 2=Yellow 3=Blue 4=Black.  Stored with
    -- U/V (Cb/Cr) SWAPPED to match the Videomancer output convention (matching the
    -- verified phosphor/CGA/C64 palettes).  Pre-swap BT.601 chroma was:
    --   Red  U=417 V=843 | Yellow U=168 V=645 | Blue U=729 V=410
    --------------------------------------------------------------------------
    constant PAL_WHITE  : integer := 0;
    constant PAL_RED    : integer := 1;
    constant PAL_YELLOW : integer := 2;
    constant PAL_BLUE   : integer := 3;
    constant PAL_BLACK  : integer := 4;

    type t_pal is array (0 to 4) of unsigned(9 downto 0);
    constant MON_Y : t_pal := (
        to_unsigned(940,10), to_unsigned(332,10), to_unsigned(729,10),
        to_unsigned(272,10), to_unsigned( 48,10));
    constant MON_U : t_pal := (
        to_unsigned(512,10), to_unsigned(843,10), to_unsigned(645,10),
        to_unsigned(410,10), to_unsigned(512,10));
    constant MON_V : t_pal := (
        to_unsigned(512,10), to_unsigned(417,10), to_unsigned(168,10),
        to_unsigned(729,10), to_unsigned(512,10));

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

    -- Per-cell hash.  Uses one ADD (carry propagation) amid rotates/xors, NOT just
    -- xor/shift: a purely linear (xor/shift) hash of (x0,y0) leaves the grow-bit
    -- field linearly structured, so whole rows / diagonals of cells merge together.
    -- The single add makes it nonlinear, scattering the merges randomly, while
    -- keeping the hash short enough to close HD timing on its own pipeline stage.
    function init_seed(comp : unsigned(15 downto 0); x0, y0 : unsigned(11 downto 0))
        return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := comp xor resize(x0, 16) xor rotate_left(resize(y0, 16), 5);
        h := h + rotate_left(h, 7);          -- single nonlinear add (carry mixing)
        h := h xor rotate_left(h, 11);
        h := h xor shift_right(h, 6);
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

    -- Decide stage: compute the split coordinate for the active axis.  Jitter
    -- magnitude is at most 3w/8 < w/2, so nx is always strictly inside (lo,hi).
    function calc_split(n        : t_node;
                        orient   : std_logic;
                        dosplit  : std_logic;
                        sgn      : std_logic;
                        half     : std_logic;
                        jit_mode : unsigned(1 downto 0)) return t_dec is
        variable lo, hi, w, mag, mid : unsigned(11 downto 0);
        variable r : t_dec;
    begin
        if orient = '0' then lo := n.x0; hi := n.x1;
        else                 lo := n.y0; hi := n.y1; end if;
        w := hi - lo;
        case jit_mode is
            when "00"   => mag := (others => '0');
            when "01"   => mag := "000" & w(11 downto 3);                     -- w/8
            when "10"   => mag := "00"  & w(11 downto 2);                     -- w/4
            when others => mag := ("00" & w(11 downto 2))
                                + ("000" & w(11 downto 3));                   -- 3w/8
        end case;
        if half = '1' then mag := '0' & mag(11 downto 1); end if;
        mid := lo + ('0' & w(11 downto 1));                                  -- lo + w/2
        if sgn = '1' then r.nx := mid - mag;
        else              r.nx := mid + mag; end if;
        if (dosplit = '1') and (n.frozen = '0') and (w >= to_unsigned(MIN_W, 12)) then
            r.doit := '1';
        else
            r.doit := '0';
        end if;
        return r;
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

    -- Coarse-cell column index for the per-column luma hold (per-cell Luma Mod).
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

    --------------------------------------------------------------------------
    -- Position + per-frame parameters
    --------------------------------------------------------------------------
    signal pixel_x, pixel_y : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n     : std_logic := '1';
    signal prev_vsync_n     : std_logic := '1';

    signal s_g          : integer range 5 to 8 := 6;
    signal s_cellsz     : integer range 32 to 256 := 64;   -- coarse cell size (px)
    signal s_cell2sz    : integer range 64 to 512 := 128;  -- 2x cell size (px)
    signal s_cellmask   : unsigned(11 downto 0) := to_unsigned(63, 12);
    signal s_cellcenter : unsigned(11 downto 0) := to_unsigned(32, 12);   -- cell centre (sz/2)
    signal s_density    : unsigned(7 downto 0) := to_unsigned(160, 8);
    signal s_jit_mode   : unsigned(1 downto 0) := to_unsigned(1, 2);
    signal s_color_amt  : unsigned(7 downto 0) := to_unsigned(55, 8);   -- white<->colour threshold
    signal s_line_w     : unsigned(11 downto 0) := to_unsigned(3, 12);  -- grid-line half-width (px)
    signal s_comp_seed  : unsigned(15 downto 0) := x"1234";
    signal s_sub_seed   : unsigned(15 downto 0) := x"ABCD";
    signal s_seam_en    : std_logic := '1';
    signal s_blackblk   : std_logic := '0';
    signal s_notsq      : std_logic := '0';   -- S9: 0 = square grid, 1 = cell merge
    signal s_vid_en     : std_logic := '0';
    signal s_lumamod    : std_logic := '0';   -- S11: per-cell luma gate (black where cell luma <= mid)
    signal s_line_top   : std_logic := '0';   -- this scan line is a cell-row top

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    signal s0_x, s0_y : unsigned(11 downto 0);

    -- Per-cell Luma Mod: one hide bit per base-cell column, sampled at the cell
    -- centre on each cell-row's top line and held across the cell, then delayed to
    -- the output so a whole cell gates on/off coherently (vs the old per-pixel matte).
    signal cell_hide : std_logic_vector(0 to 63) := (others => '0');
    signal hide_pipe : std_logic_vector(0 to LATENCY - 1) := (others => '0');

    -- S1a: base cell origin + 2x2 candidate aligned origins + edge validity.
    signal s1a_ax, s1a_ay    : t_u12_4;
    signal s1a_x0, s1a_y0    : unsigned(11 downto 0);
    signal s1a_px, s1a_py    : unsigned(11 downto 0);
    signal s1a_left_ok       : std_logic;
    signal s1a_up_ok         : std_logic;

    -- S1a2: per-candidate cell hash on its own stage (the nonlinear hash adds would
    -- otherwise stack on the origin-subtract path and miss HD timing).
    signal s1a2_hk           : t_u16_4;
    signal s1a2_x0, s1a2_y0  : unsigned(11 downto 0);
    signal s1a2_px, s1a2_py  : unsigned(11 downto 0);
    signal s1a2_left_ok      : std_logic;
    signal s1a2_up_ok        : std_logic;

    -- S1b: winning candidate, selected (node built in S1c to keep paths short).
    signal s1b_hkw           : unsigned(15 downto 0);
    signal s1b_bigx, s1b_bigy : std_logic;
    signal s1b_ckx, s1b_cky  : std_logic;
    signal s1b_x0, s1b_y0    : unsigned(11 downto 0);
    signal s1b_px, s1b_py    : unsigned(11 downto 0);

    signal s1_n : t_node;                                  -- selected (merged) cell node

    -- hash-stage outputs (decoded bits + node carrying new seed)
    signal h0_n, h1_n, h2_n, h3_n     : t_node;
    signal h0_or, h1_or, h2_or, h3_or : std_logic;
    signal h0_ds, h1_ds, h2_ds, h3_ds : std_logic;
    signal h0_sg, h1_sg, h2_sg, h3_sg : std_logic;
    signal h0_hf, h1_hf, h2_hf, h3_hf : std_logic;

    -- decide-stage outputs (node + orient passthrough + split coord + flag)
    signal d0_n, d1_n, d2_n, d3_n         : t_node;
    signal d0_or, d1_or, d2_or, d3_or     : std_logic;
    signal d0_dt, d1_dt, d2_dt, d3_dt     : std_logic;
    signal d0_nx, d1_nx, d2_nx, d3_nx     : unsigned(11 downto 0);

    -- descend-stage output nodes
    signal n0, n1, n2, n3 : t_node;

    -- leaf metrics, split across two stages
    signal s10a_m1, s10a_m2 : unsigned(11 downto 0);
    signal s10a_seed        : unsigned(15 downto 0);

    signal s10_seamd   : unsigned(11 downto 0);
    signal s10_pidx    : unsigned(2 downto 0);   -- 0..4 palette index
    signal s10_iscolor : std_logic;              -- '1' = primary block (video-fill candidate)

    -- palette fetch
    signal s11_y, s11_u, s11_v : unsigned(9 downto 0);
    signal s11_iscolor         : std_logic;
    signal s11_seamd           : unsigned(11 downto 0);

    -- assembled colour
    signal s12_y, s12_u, s12_v : unsigned(9 downto 0);
    signal s12_vid             : std_logic;      -- this pixel may be video-filled

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

            -- this scan line is the top of a cell-row (pixel_y constant per line)
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
                    when 0      => s_g <= 8; s_cellsz <= 256; s_cell2sz <= 512;
                                   s_cellmask <= to_unsigned(255, 12); s_cellcenter <= to_unsigned(128, 12);
                    when 1      => s_g <= 7; s_cellsz <= 128; s_cell2sz <= 256;
                                   s_cellmask <= to_unsigned(127, 12); s_cellcenter <= to_unsigned(64, 12);
                    when 2      => s_g <= 6; s_cellsz <= 64;  s_cell2sz <= 128;
                                   s_cellmask <= to_unsigned(63, 12);  s_cellcenter <= to_unsigned(32, 12);
                    when others => s_g <= 5; s_cellsz <= 32;  s_cell2sz <= 64;
                                   s_cellmask <= to_unsigned(31, 12);  s_cellcenter <= to_unsigned(16, 12);
                end case;

                s_comp_seed <= mix16(unsigned(registers_in(1)), x"9E37");
                s_sub_seed  <= mix16(unsigned(registers_in(2)), x"C2B5");
                s_jit_mode  <= unsigned(registers_in(3)(9 downto 8));

                -- Colour Amount: 8-bit threshold; higher -> more coloured blocks.
                s_color_amt <= unsigned(registers_in(4)(9 downto 2));

                -- Line Width: K6 top 2 bits -> half-width 1/2/3/4 px.
                case to_integer(unsigned(registers_in(5)(9 downto 8))) is
                    when 0      => s_line_w <= to_unsigned(1, 12);
                    when 1      => s_line_w <= to_unsigned(2, 12);
                    when 2      => s_line_w <= to_unsigned(3, 12);
                    when others => s_line_w <= to_unsigned(4, 12);
                end case;

                s_seam_en  <= registers_in(6)(0);
                s_blackblk <= registers_in(6)(1);
                s_notsq    <= registers_in(6)(2);
                s_vid_en   <= registers_in(6)(3);
                s_lumamod  <= registers_in(6)(4);

                s_density  <= unsigned(registers_in(7)(9 downto 2));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_x0, v_y0   : unsigned(11 downto 0);
        variable v_axk, v_ayk : unsigned(11 downto 0);
        variable v_col        : integer range 0 to 63;
        variable v_bigx, v_bigy, v_cov : std_logic_vector(0 to 3);
        variable v_rnk        : integer range 0 to 2;
        variable vv           : integer_vector(0 to 3);
        variable v_m01, v_m23 : integer;
        variable v_i01, v_i23, v_kw : integer range 0 to 3;
        variable v_wx0, v_wx1, v_wy0, v_wy1 : unsigned(11 downto 0);
        variable dtmp       : t_dec;
        variable n          : t_node;
        variable lw, lh     : unsigned(11 downto 0);
        variable lx, ly     : unsigned(11 downto 0);
        variable bb, dd     : unsigned(11 downto 0);
        variable m1, m2     : unsigned(11 downto 0);
        variable seamd      : unsigned(11 downto 0);
        variable v_colbyte  : unsigned(7 downto 0);
        variable v_sel      : unsigned(1 downto 0);
        variable v_pidx     : integer range 0 to 4;
        variable v_iscol    : std_logic;
        variable pidx       : integer range 0 to 7;
        variable ty, tu, tv : unsigned(9 downto 0);

        -- one hash stage: re-hash seed, decode split controls
        procedure do_hash(signal q_n  : out t_node;
                          signal q_or : out std_logic;
                          signal q_ds : out std_logic;
                          signal q_sg : out std_logic;
                          signal q_hf : out std_logic;
                          constant lvl : integer;
                          n_in : t_node) is
            variable h  : unsigned(15 downto 0);
            variable nn : t_node;
            variable o  : std_logic;
            variable t8 : integer;
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
        end procedure;
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- S0: latch pixel position + per-cell Luma Mod sample/hold/delay.
            s0_x <= pixel_x;
            s0_y <= pixel_y;
            v_col := cell_col(pixel_x, s_g);
            hide_pipe(0) <= cell_hide(v_col);
            for i in 1 to LATENCY - 1 loop hide_pipe(i) <= hide_pipe(i - 1); end loop;
            -- sample one hide bit per cell, at the cell centre on the cell-row's top line
            if s_line_top = '1' and (pixel_x and s_cellmask) = s_cellcenter then
                if unsigned(data_in.y) <= to_unsigned(512, 10) then
                    cell_hide(v_col) <= '1';
                else
                    cell_hide(v_col) <= '0';
                end if;
            end if;

            -- S1a: base grid cell origin + the 2x2 candidate cells' aligned origins.
            -- The up-left candidates are only valid when the base cell isn't on the
            -- top/left edge (so merged origins never go negative).
            v_x0 := s0_x and not s_cellmask;
            v_y0 := s0_y and not s_cellmask;
            s1a_x0 <= v_x0;
            s1a_y0 <= v_y0;
            s1a_px <= s0_x;
            s1a_py <= s0_y;
            if v_x0 >= to_unsigned(s_cellsz, 12) then s1a_left_ok <= '1'; else s1a_left_ok <= '0'; end if;
            if v_y0 >= to_unsigned(s_cellsz, 12) then s1a_up_ok   <= '1'; else s1a_up_ok   <= '0'; end if;
            for k in 0 to 3 loop
                s1a_ax(k) <= v_x0 - to_unsigned(CK_X(k) * s_cellsz, 12);
                s1a_ay(k) <= v_y0 - to_unsigned(CK_Y(k) * s_cellsz, 12);
            end loop;

            -- S1a2: per-candidate cell hash (own stage to isolate the nonlinear adds)
            for k in 0 to 3 loop
                s1a2_hk(k) <= init_seed(s_comp_seed, s1a_ax(k), s1a_ay(k));
            end loop;
            s1a2_x0      <= s1a_x0;
            s1a2_y0      <= s1a_y0;
            s1a2_px      <= s1a_px;
            s1a2_py      <= s1a_py;
            s1a2_left_ok <= s1a_left_ok;
            s1a2_up_ok   <= s1a_up_ok;

            -- S1b: decode each candidate's grow-right / grow-down bits (~25% each,
            -- only in Not-Square mode), test which cover this pixel (pure hash-bit
            -- logic since cells are grid-aligned), and pick the LARGEST covering cell
            -- (size-first, hash priority breaks ties).
            for k in 0 to 3 loop
                v_bigx(k) := s_notsq and s1a2_hk(k)(7) and s1a2_hk(k)(3);
                v_bigy(k) := s_notsq and s1a2_hk(k)(6) and s1a2_hk(k)(2);
            end loop;
            v_cov(0) := '1';
            v_cov(1) := v_bigx(1) and s1a2_left_ok;
            v_cov(2) := v_bigy(2) and s1a2_up_ok;
            v_cov(3) := v_bigx(3) and v_bigy(3) and s1a2_left_ok and s1a2_up_ok;
            for k in 0 to 3 loop
                v_rnk := 0;
                if v_bigx(k) = '1' then v_rnk := v_rnk + 1; end if;
                if v_bigy(k) = '1' then v_rnk := v_rnk + 1; end if;
                if v_cov(k) = '1' then
                    vv(k) := 16 + 16 * v_rnk + to_integer(s1a2_hk(k)(15 downto 12));
                else
                    vv(k) := 0;
                end if;
            end loop;
            -- depth-2 tree argmax (ties -> lower index)
            if vv(0) >= vv(1) then v_m01 := vv(0); v_i01 := 0;
            else                   v_m01 := vv(1); v_i01 := 1; end if;
            if vv(2) >= vv(3) then v_m23 := vv(2); v_i23 := 2;
            else                   v_m23 := vv(3); v_i23 := 3; end if;
            if v_m01 >= v_m23 then v_kw := v_i01; else v_kw := v_i23; end if;

            -- register only the WINNER's data; the node (a couple of adds) is
            -- built next cycle in S1c so this stage's argmax path stays short.
            s1b_hkw  <= s1a2_hk(v_kw);
            s1b_bigx <= v_bigx(v_kw);
            s1b_bigy <= v_bigy(v_kw);
            if CK_X(v_kw) = 1 then s1b_ckx <= '1'; else s1b_ckx <= '0'; end if;
            if CK_Y(v_kw) = 1 then s1b_cky <= '1'; else s1b_cky <= '0'; end if;
            s1b_x0 <= s1a2_x0;
            s1b_y0 <= s1a2_y0;
            s1b_px <= s1a2_px;
            s1b_py <= s1a2_py;

            -- S1c: build the winning (merged) cell node — origin shifted up-left by
            -- the candidate offset, extent 1 or 2 cells.  A grown cell is one root
            -- (one seed) spanning 2 cells, so it subdivides as a unit and the old
            -- grid line between the fused cells is no longer forced.
            if s1b_ckx = '1' then v_wx0 := s1b_x0 - to_unsigned(s_cellsz, 12);
            else                  v_wx0 := s1b_x0; end if;
            if s1b_cky = '1' then v_wy0 := s1b_y0 - to_unsigned(s_cellsz, 12);
            else                  v_wy0 := s1b_y0; end if;
            if s1b_bigx = '1' then v_wx1 := v_wx0 + to_unsigned(s_cell2sz, 12);
            else                   v_wx1 := v_wx0 + to_unsigned(s_cellsz, 12); end if;
            if s1b_bigy = '1' then v_wy1 := v_wy0 + to_unsigned(s_cell2sz, 12);
            else                   v_wy1 := v_wy0 + to_unsigned(s_cellsz, 12); end if;
            s1_n.x0     <= v_wx0;
            s1_n.x1     <= v_wx1;
            s1_n.y0     <= v_wy0;
            s1_n.y1     <= v_wy1;
            s1_n.px     <= s1b_px;
            s1_n.py     <= s1b_py;
            s1_n.seed   <= s1b_hkw;
            s1_n.dens   <= s_density;   -- merged cells subdivide like any other
            s1_n.frozen <= '0';

            -- Level 0 : hash -> decide -> descend
            do_hash(h0_n, h0_or, h0_ds, h0_sg, h0_hf, 0, s1_n);
            dtmp := calc_split(h0_n, h0_or, h0_ds, h0_sg, h0_hf, s_jit_mode);
            d0_n <= h0_n; d0_or <= h0_or; d0_dt <= dtmp.doit; d0_nx <= dtmp.nx;
            n0 <= descend(d0_n, d0_or, d0_dt, d0_nx);

            -- Level 1
            do_hash(h1_n, h1_or, h1_ds, h1_sg, h1_hf, 1, n0);
            dtmp := calc_split(h1_n, h1_or, h1_ds, h1_sg, h1_hf, s_jit_mode);
            d1_n <= h1_n; d1_or <= h1_or; d1_dt <= dtmp.doit; d1_nx <= dtmp.nx;
            n1 <= descend(d1_n, d1_or, d1_dt, d1_nx);

            -- Level 2
            do_hash(h2_n, h2_or, h2_ds, h2_sg, h2_hf, 2, n1);
            dtmp := calc_split(h2_n, h2_or, h2_ds, h2_sg, h2_hf, s_jit_mode);
            d2_n <= h2_n; d2_or <= h2_or; d2_dt <= dtmp.doit; d2_nx <= dtmp.nx;
            n2 <= descend(d2_n, d2_or, d2_dt, d2_nx);

            -- Level 3
            do_hash(h3_n, h3_or, h3_ds, h3_sg, h3_hf, 3, n2);
            dtmp := calc_split(h3_n, h3_or, h3_ds, h3_sg, h3_hf, s_jit_mode);
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
            s10a_m1   <= m1;
            s10a_m2   <= m2;
            s10a_seed <= n.seed;

            -- S10b: grid-line distance + weighted Mondrian colour pick.
            if s10a_m1 < s10a_m2 then seamd := s10a_m1; else seamd := s10a_m2; end if;
            s10_seamd <= seamd;

            v_colbyte := s10a_seed(7 downto 0);
            v_sel     := s10a_seed(9 downto 8);
            if v_colbyte < s_color_amt then
                case v_sel is
                    when "00"   => v_pidx := PAL_RED;
                    when "01"   => v_pidx := PAL_YELLOW;
                    when "10"   => v_pidx := PAL_BLUE;
                    when others => v_pidx := PAL_RED;
                end case;
                v_iscol := '1';
                if s_blackblk = '1' and s10a_seed(13 downto 11) = "111" then
                    v_pidx  := PAL_BLACK;   -- rare solid-black block
                    v_iscol := '0';
                end if;
            else
                v_pidx  := PAL_WHITE;
                v_iscol := '0';
            end if;
            s10_pidx    <= to_unsigned(v_pidx, 3);
            s10_iscolor <= v_iscol;

            -- S11: palette fetch
            pidx := to_integer(s10_pidx);
            ty := MON_Y(pidx); tu := MON_U(pidx); tv := MON_V(pidx);
            s11_y <= ty; s11_u <= tu; s11_v <= tv;
            s11_iscolor <= s10_iscolor;
            s11_seamd   <= s10_seamd;

            -- S12: assemble colour (grid line > flat block)
            if s_seam_en = '1' and s11_seamd < s_line_w then
                s12_y   <= LINE_Y;
                s12_u   <= C_MID;
                s12_v   <= C_MID;
                s12_vid <= '0';
            else
                s12_y   <= s11_y;
                s12_u   <= s11_u;
                s12_v   <= s11_v;
                s12_vid <= s11_iscolor;
            end if;

            -- S_out: per-cell Luma Mod gate (black where the cell's held luma is not
            -- positive) > video-fill override > flat block colour.
            if s_lumamod = '1' and hide_pipe(LATENCY - 1) = '1' then
                s_io.y <= (others => '0');
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            elsif s12_vid = '1' and s_vid_en = '1' then
                s_io.y <= pipe(LATENCY - 1).y;
                s_io.u <= pipe(LATENCY - 1).u;
                s_io.v <= pipe(LATENCY - 1).v;
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

end architecture mondrian;
