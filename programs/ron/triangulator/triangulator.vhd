-- triangulator.vhd  (v0.1 — triangle pixelation)
--
-- A pixelation effect that tiles the screen with a triangular lattice instead of
-- square blocks.  Each cell is Wc wide x H = Wc*7/4 tall (~equilateral; the edge
-- slope is lx*7/4 ~= lx*sqrt(3), giving a ~59.4-degree apex) and is split by ONE
-- diagonal into two triangles; the diagonal orientation alternates per cell COLUMN
-- AND per ROW (half-offset lattice) so the two triangles of adjacent columns merge
-- into the visual upright (▲) and inverted (▽) triangles, which alternate ▲▽▲▽
-- across every row and meet flat-base-to-flat-base between rows.  Each triangle is
-- flat-filled with a colour point-sampled from the incoming live video (sampled
-- once per cell at the cell centre on each cell-row's top line, then held down the
-- cell — the c64/cga hold trick; a point sample so any cell size works, NO divide).
--
-- The cell decomposition uses the mondrian continuous-scale counters (advance as
-- the raster scans), so K1 gives a VERY smooth 8..256 px size with no divide and
-- no power-of-2 quantisation.  The equilateral edge test is lx*7/4 vs ly (shift-add),
-- so there is no per-pixel multiply in the geometry path.
--
-- Controls:
--   K1 Triangle Size (smooth 8..256 px)
--   K2 Luma Gain      (fill contrast around mid; 128 = unity)   [Luma Mod on]
--   K3 Luma Thresh    ("has luma" gate threshold)               [Luma Mod on]
--   K4 Colorize Mode  (Original / Duotone / Posterize / Hue-rotate)
--   K5 Colorize Param + grid/edge colour (8-entry palette)
--   K6 Line Width     (1..4 px)
--   S7 Grid Lines     (triangle edges on/off, normal mode)
--   S8 Luma Mod       (GATE: show only luma-positive triangles)
--   S9 Fill Mode      (Flat = one colour per ▲/▽ | Faceted = per half-cell)
--   S10 Wireframe     (edges only, no fill)
--   S11 Bypass        (pass input through)
--   P12 (reserved)
--
-- Luma Mod / Wireframe matrix:
--   LM on,  WF off -> only lit triangles: fill + their own grid lines, else black
--   LM on,  WF on  -> only lit triangles' grid lines (K5), no fill, else black
--   LM off, WF on  -> full mesh, every edge coloured by the LIVE incoming video
--   LM off, WF off -> normal fill (+ optional K5 grid)
-- Gating is per-triangle (own luma) so only COMPLETE lit triangles draw.
--
-- Fixed colours are stored with U/V (Cb/Cr) SWAPPED to match the Videomancer
-- hardware output convention used by the HW-validated mondrian / c64 palettes
-- (stored U = Cr, stored V = Cb).
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

architecture triangulator of program_top is

    constant LATENCY  : natural := 10;
    constant C_MAXCOL : natural := 256;                       -- max cell columns (small triangles)

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant LINE_Y : unsigned(9 downto 0) := to_unsigned( 48, 10);  -- grid-line luma (deep black)

    --------------------------------------------------------------------------
    -- Colorize tables (U/V stored swapped: stored U = Cr, stored V = Cb).
    --------------------------------------------------------------------------
    -- Posterize: 8-entry rainbow indexed by luma zone (top 3 bits).
    type t_pal8 is array(0 to 7) of unsigned(9 downto 0);
    constant RNB_Y : t_pal8 := (
        to_unsigned(320,10), to_unsigned(460,10), to_unsigned(720,10), to_unsigned(560,10),
        to_unsigned(650,10), to_unsigned(300,10), to_unsigned(360,10), to_unsigned(900,10));
    constant RNB_U : t_pal8 := (   -- = Cr
        to_unsigned(850,10), to_unsigned(740,10), to_unsigned(620,10), to_unsigned(340,10),
        to_unsigned(290,10), to_unsigned(410,10), to_unsigned(620,10), to_unsigned(512,10));
    constant RNB_V : t_pal8 := (   -- = Cb
        to_unsigned(410,10), to_unsigned(360,10), to_unsigned(170,10), to_unsigned(300,10),
        to_unsigned(560,10), to_unsigned(760,10), to_unsigned(760,10), to_unsigned(512,10));

    -- Posterize: 8-step grey ramp.
    constant GRY_Y : t_pal8 := (
        to_unsigned( 64,10), to_unsigned(180,10), to_unsigned(300,10), to_unsigned(430,10),
        to_unsigned(560,10), to_unsigned(690,10), to_unsigned(820,10), to_unsigned(950,10));

    -- Grid-line colour, selected by K5 top-3 bits: 0 black, 1 white, then hues.
    constant GRID_Y : t_pal8 := (
        to_unsigned( 48,10), to_unsigned(940,10), to_unsigned(320,10), to_unsigned(720,10),
        to_unsigned(560,10), to_unsigned(650,10), to_unsigned(300,10), to_unsigned(360,10));
    constant GRID_U : t_pal8 := (   -- = Cr
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(850,10), to_unsigned(620,10),
        to_unsigned(340,10), to_unsigned(290,10), to_unsigned(410,10), to_unsigned(620,10));
    constant GRID_V : t_pal8 := (   -- = Cb
        to_unsigned(512,10), to_unsigned(512,10), to_unsigned(410,10), to_unsigned(170,10),
        to_unsigned(300,10), to_unsigned(560,10), to_unsigned(760,10), to_unsigned(760,10));

    -- Duotone: 4 shadow->highlight pairs.
    type t_duo4 is array(0 to 3) of unsigned(9 downto 0);
    constant DUO_LO_Y : t_duo4 := (to_unsigned( 80,10), to_unsigned(120,10), to_unsigned(120,10), to_unsigned(200,10));
    constant DUO_LO_U : t_duo4 := (to_unsigned(470,10), to_unsigned(470,10), to_unsigned(650,10), to_unsigned(360,10));
    constant DUO_LO_V : t_duo4 := (to_unsigned(590,10), to_unsigned(720,10), to_unsigned(680,10), to_unsigned(320,10));
    constant DUO_HI_Y : t_duo4 := (to_unsigned(900,10), to_unsigned(950,10), to_unsigned(820,10), to_unsigned(850,10));
    constant DUO_HI_U : t_duo4 := (to_unsigned(590,10), to_unsigned(512,10), to_unsigned(600,10), to_unsigned(680,10));
    constant DUO_HI_V : t_duo4 := (to_unsigned(430,10), to_unsigned(512,10), to_unsigned(180,10), to_unsigned(560,10));

    function clamp10(v : signed) return unsigned is
    begin
        if v < 0 then               return to_unsigned(0, 10);
        elsif v > 1023 then         return to_unsigned(1023, 10);
        else                        return unsigned(resize(v, 10));
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Position + per-frame parameters (continuous-scale cell counters: advance
    -- as the raster scans -> arbitrary cell sizes with no divide/multiply).
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';

    signal cellx_loc : unsigned(11 downto 0) := (others => '0');   -- lx within cell
    signal cellx_idx : unsigned(7 downto 0)  := (others => '0');   -- cell column index
    signal celly_loc : unsigned(11 downto 0) := (others => '0');   -- ly within cell
    signal celly_par : std_logic := '0';                           -- cell-row parity (half-offset)
    signal frame_act : std_logic := '0';                           -- seen active video this frame

    signal s_csz     : unsigned(11 downto 0) := to_unsigned(64, 12);  -- cell width Wc
    signal s_cszm1   : unsigned(11 downto 0) := to_unsigned(63, 12);  -- Wc - 1
    signal s_cszc    : unsigned(11 downto 0) := to_unsigned(32, 12);  -- Wc / 2 (cell centre)
    signal s_csz_h   : unsigned(11 downto 0) := to_unsigned(112, 12); -- row height H = Wc*7/4
    signal s_csz_hm1 : unsigned(11 downto 0) := to_unsigned(111, 12); -- H - 1

    signal s_lumamod : std_logic := '0';                           -- S8 luma mod enabled
    signal s_lgain  : unsigned(7 downto 0) := to_unsigned(128, 8); -- luma gain (128 = unity)
    signal s_floor  : unsigned(9 downto 0) := (others => '0');     -- luma black-lift floor
    signal s_cmode  : unsigned(1 downto 0) := (others => '0');     -- colorize mode
    signal s_duosel : unsigned(1 downto 0) := (others => '0');     -- duotone pair
    signal s_palsel : std_logic := '0';                            -- posterize palette
    signal s_hue    : unsigned(1 downto 0) := (others => '0');     -- hue angle (x90 deg)
    signal s_line_w : unsigned(11 downto 0) := to_unsigned(2, 12); -- grid line width
    signal s_grid   : std_logic := '1';
    signal s_fill   : std_logic := '0';                            -- 0 faceted, 1 flat
    signal s_wire   : std_logic := '0';                            -- S10: grid only, no fill
    signal s_bypass : std_logic := '0';                            -- S11
    signal s_gy, s_gu, s_gv : unsigned(9 downto 0) := to_unsigned(48, 10); -- grid colour (K5)

    -- per-frame selected duotone endpoints + deltas
    signal s_dlo_y, s_dlo_u, s_dlo_v : unsigned(9 downto 0) := C_MID;
    signal s_dd_y,  s_dd_u,  s_dd_v  : signed(10 downto 0)  := (others => '0');

    --------------------------------------------------------------------------
    -- Cell colour buffer (1W1R, per cell column).  Sampled at the cell centre on
    -- each cell-row's top line, held for the whole cell.
    --------------------------------------------------------------------------
    type t_cbuf is array(0 to C_MAXCOL - 1) of unsigned(9 downto 0);
    signal buf_y : t_cbuf := (others => C_MID);
    signal buf_u : t_cbuf := (others => C_MID);
    signal buf_v : t_cbuf := (others => C_MID);
    signal s_we    : std_logic := '0';
    signal s_waddr : unsigned(7 downto 0) := (others => '0');
    signal s_wy, s_wu, s_wv : unsigned(9 downto 0) := C_MID;
    signal s_buf_raddr : unsigned(7 downto 0) := (others => '0');
    signal s_rd_y, s_rd_u, s_rd_v : unsigned(9 downto 0) := C_MID;

    signal s_gate_th : unsigned(9 downto 0) := (others => '0');  -- "has luma" threshold (K3)

    -- geo0: pre-registered lx*7/4 (+ aligned ly/cx/parity) so p_comb's diagonal
    -- chain stays short enough to close HD timing (the *7/4 adds would otherwise
    -- sit in the same combinational cone as the diagonal subtract + grid compare).
    signal g0_lxs  : unsigned(10 downto 0) := (others => '0');  -- lx*7/4, <= 446
    signal g0_ly   : unsigned(11 downto 0) := (others => '0');
    signal g0_cx   : unsigned(7 downto 0)  := (others => '0');
    signal g0_even : std_logic := '0';                         -- '1' = "/" column

    -- combinational geometry outputs
    signal c_raddr : unsigned(7 downto 0) := (others => '0');
    signal c_grid  : std_logic := '0';

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    --------------------------------------------------------------------------
    -- Pipeline registers
    --------------------------------------------------------------------------
    signal g_pipe    : std_logic_vector(0 to 5) := (others => '0');
    signal lown_pipe : std_logic_vector(0 to 4) := (others => '0'); -- own-triangle lit (gates fill + own edges)

    signal samp_y, samp_u, samp_v : unsigned(9 downto 0) := C_MID;   -- St1
    signal s2_lp                  : signed(20 downto 0) := (others => '0'); -- St2 luma product
    signal s2_u, s2_v             : unsigned(9 downto 0) := C_MID;
    signal s3_lm                  : unsigned(9 downto 0) := (others => '0'); -- St3 shaped luma
    signal s3_u, s3_v             : unsigned(9 downto 0) := C_MID;
    signal s4_pdy, s4_pdu, s4_pdv : signed(21 downto 0) := (others => '0'); -- St4 duotone products
    signal s4_lm, s4_u, s4_v      : unsigned(9 downto 0) := C_MID;
    signal s5_y, s5_u, s5_v       : unsigned(9 downto 0) := C_MID;          -- St5 colorized
    signal s6_y, s6_u, s6_v       : unsigned(9 downto 0) := C_MID;          -- St6 grid overlay
    signal s6_vid                 : std_logic := '0';                       -- St6: colour this edge from live video

    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Position counters + parameter latch (at vsync edge).
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_csz, v_csh : integer;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            -- X cell counter (origin/local/index) -> continuous cell size, no divide.
            if v_h_edge = '1' then
                cellx_loc <= (others => '0');
                cellx_idx <= (others => '0');
            elsif data_in.avid = '1' then
                if cellx_loc = s_cszm1 then
                    cellx_loc <= (others => '0');
                    cellx_idx <= cellx_idx + 1;
                else
                    cellx_loc <= cellx_loc + 1;
                end if;
            end if;

            -- buffer write: sample the cell centre on each cell-row's top line.
            if data_in.avid = '1' and celly_loc = 0 and cellx_loc = s_cszc then
                s_we    <= '1';
                s_waddr <= cellx_idx;
                s_wy    <= unsigned(data_in.y);
                s_wu    <= unsigned(data_in.u);
                s_wv    <= unsigned(data_in.v);
            else
                s_we <= '0';
            end if;

            if v_v_edge = '1' then
                celly_loc <= (others => '0');
                celly_par <= '0';
                frame_act <= '0';

                -- K1 Triangle Size: continuous 8..256 px (small end goes very fine).
                v_csz := 8 + to_integer(unsigned(registers_in(0)(9 downto 2)));
                if v_csz > 256 then v_csz := 256; end if;
                -- Equilateral row height H = Wc*7/4 (~Wc*sqrt(3); apex ~59.4 deg).
                v_csh := v_csz + v_csz / 2 + v_csz / 4;
                s_csz     <= to_unsigned(v_csz, 12);
                s_cszm1   <= to_unsigned(v_csz - 1, 12);
                s_cszc    <= to_unsigned(v_csz / 2, 12);
                s_csz_h   <= to_unsigned(v_csh, 12);
                s_csz_hm1 <= to_unsigned(v_csh - 1, 12);

                -- Wireframe "has luma" threshold (K3, raw luma).
                s_gate_th <= unsigned(registers_in(2)(9 downto 0));

                -- Luma Mod (S8): a luma GATE (only luma-positive triangles show) plus
                -- fill contrast via K2 gain.  Off -> unity gain, no gate (everything shows).
                s_lumamod <= registers_in(6)(1);
                if registers_in(6)(1) = '1' then
                    s_lgain <= unsigned(registers_in(1)(9 downto 2));
                else
                    s_lgain <= to_unsigned(128, 8);
                end if;
                s_floor <= (others => '0');

                s_cmode  <= unsigned(registers_in(3)(9 downto 8));
                s_duosel <= unsigned(registers_in(4)(9 downto 8));
                s_palsel <= registers_in(4)(9);
                s_hue    <= unsigned(registers_in(4)(9 downto 8));

                -- Line Width (K6 top 2 bits) -> 1/2/3/4 px.  The diagonal uses the
                -- same threshold: its ~2.0 edge gradient cancels the 2x from the
                -- symmetric |d| test, so diagonal and horizontal come out equal.
                case to_integer(unsigned(registers_in(5)(9 downto 8))) is
                    when 0      => s_line_w <= to_unsigned(1, 12);
                    when 1      => s_line_w <= to_unsigned(2, 12);
                    when 2      => s_line_w <= to_unsigned(3, 12);
                    when others => s_line_w <= to_unsigned(4, 12);
                end case;

                s_grid   <= registers_in(6)(0);
                s_fill   <= registers_in(6)(2);
                s_wire   <= registers_in(6)(3);   -- S10 Wireframe (grid only)
                s_bypass <= registers_in(6)(4);   -- S11 Bypass

                -- Grid-line colour from K5 (top 3 bits).
                s_gy <= GRID_Y(to_integer(unsigned(registers_in(4)(9 downto 7))));
                s_gu <= GRID_U(to_integer(unsigned(registers_in(4)(9 downto 7))));
                s_gv <= GRID_V(to_integer(unsigned(registers_in(4)(9 downto 7))));

                -- Pre-select duotone endpoints + deltas once per frame.
                s_dlo_y <= DUO_LO_Y(to_integer(unsigned(registers_in(4)(9 downto 8))));
                s_dlo_u <= DUO_LO_U(to_integer(unsigned(registers_in(4)(9 downto 8))));
                s_dlo_v <= DUO_LO_V(to_integer(unsigned(registers_in(4)(9 downto 8))));
                s_dd_y  <= signed(resize(DUO_HI_Y(to_integer(unsigned(registers_in(4)(9 downto 8)))), 11))
                         - signed(resize(DUO_LO_Y(to_integer(unsigned(registers_in(4)(9 downto 8)))), 11));
                s_dd_u  <= signed(resize(DUO_HI_U(to_integer(unsigned(registers_in(4)(9 downto 8)))), 11))
                         - signed(resize(DUO_LO_U(to_integer(unsigned(registers_in(4)(9 downto 8)))), 11));
                s_dd_v  <= signed(resize(DUO_HI_V(to_integer(unsigned(registers_in(4)(9 downto 8)))), 11))
                         - signed(resize(DUO_LO_V(to_integer(unsigned(registers_in(4)(9 downto 8)))), 11));
            elsif data_in.avid = '1' and frame_act = '0' then
                -- Anchor the vertical cell grid to the FIRST active line of the frame.
                -- celly_loc increments on hsync (incl. the vertical back porch), so
                -- without this the top cell-row's sample line (celly_loc=0) lands in
                -- blanking, never gets written, and the top row shows the previous
                -- frame's bottom row (stale per-column buffer).
                frame_act <= '1';
                celly_loc <= (others => '0');
                celly_par <= '0';
            elsif v_h_edge = '1' then
                if celly_loc = s_csz_hm1 then
                    celly_loc <= (others => '0');
                    celly_par <= not celly_par;   -- flip per row: half-offsets the lattice
                else
                    celly_loc <= celly_loc + 1;
                end if;
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Combinational geometry: triangle classification, read address, grid flag.
    -- 45-degree diagonal => no multiply.
    --------------------------------------------------------------------------
    p_comb : process(g0_lxs, g0_ly, g0_cx, g0_even, s_csz_h, s_csz_hm1, s_line_w, s_fill)
        variable v_d     : signed(12 downto 0);
        variable v_ad    : unsigned(11 downto 0);
        variable v_side  : std_logic;
    begin
        -- g0_lxs is already lx*7/4 (registered): equilateral edge slope.
        if g0_even = '1' then             -- "/" edge (lx*7/4 + ly = H)
            v_d := signed(resize(g0_lxs, 13)) + signed(resize(g0_ly, 13))
                 - signed(resize(s_csz_h, 13));
        else                              -- "\" edge (lx*7/4 = ly)
            v_d := signed(resize(g0_lxs, 13)) - signed(resize(g0_ly, 13));
        end if;

        if v_d >= 0 then v_side := '1'; else v_side := '0'; end if;
        if v_d < 0 then v_ad := unsigned(resize(-v_d, 12)); else v_ad := unsigned(resize(v_d, 12)); end if;

        -- read address: faceted = own cell; flat = the triangle's LEFT cell
        -- (side 1 -> this cell is the left cell; side 0 -> the cell to the left).
        if s_fill = '1' then
            if v_side = '1' then
                c_raddr <= g0_cx;
            elsif g0_cx = 0 then
                c_raddr <= (others => '0');
            else
                c_raddr <= g0_cx - 1;
            end if;
        else
            c_raddr <= g0_cx;
        end if;

        -- grid line: on the active diagonal, or near the TOP (▽ base) or BOTTOM
        -- (▲ base) of the cell row, so every triangle owns all three of its edges.
        -- Vertical cell edges are NOT triangle edges.
        if (v_ad < resize(s_line_w, 12)) or (g0_ly < s_line_w)
           or (g0_ly > s_csz_hm1 - s_line_w) then
            c_grid <= '1';
        else
            c_grid <= '0';
        end if;
    end process p_comb;

    s_buf_raddr <= c_raddr;

    --------------------------------------------------------------------------
    -- Cell colour buffer (1W1R, registered read).
    --------------------------------------------------------------------------
    p_buf : process(clk)
    begin
        if rising_edge(clk) then
            if s_we = '1' then
                buf_y(to_integer(s_waddr)) <= s_wy;
                buf_u(to_integer(s_waddr)) <= s_wu;
                buf_v(to_integer(s_waddr)) <= s_wv;
            end if;
            s_rd_y <= buf_y(to_integer(s_buf_raddr));
            s_rd_u <= buf_u(to_integer(s_buf_raddr));
            s_rd_v <= buf_v(to_integer(s_buf_raddr));
        end if;
    end process p_buf;

    --------------------------------------------------------------------------
    -- Main pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_lm   : signed(12 downto 0);
        variable v_idx  : integer range 0 to 7;
        variable v_cu, v_cv : signed(11 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -- grid flag delay (align with the colour path at St6)
            g_pipe(0) <= c_grid;
            for i in 1 to 5 loop g_pipe(i) <= g_pipe(i - 1); end loop;

            -- geo0: pre-register lx*7/4 with ly/cx/parity aligned, so the diagonal
            -- + grid arithmetic in p_comb is a short combinational chain.
            g0_lxs  <= resize(cellx_loc, 11) + resize(shift_right(cellx_loc, 1), 11)
                     + resize(shift_right(cellx_loc, 2), 11);
            g0_ly   <= celly_loc;
            g0_cx   <= cellx_idx;
            -- orientation alternates per column AND per row, so every other row is
            -- half-offset and flat triangle edges (▲ base <-> ▽ base) meet.
            g0_even <= not (cellx_idx(0) xor celly_par);

            -- St1: capture the sampled cell colour (buffer read now valid).
            samp_y <= s_rd_y;
            samp_u <= s_rd_u;
            samp_v <= s_rd_v;

            -- St1: per-pixel "own triangle has luma" gate, carried alongside the
            -- colour path (St2..St5) to line up with the St6 overlay.  Each pixel is
            -- gated by ITS OWN triangle (flat read), so only complete lit triangles
            -- draw — fill and their own edges; non-lit triangles draw nothing.
            lown_pipe(0) <= '0';
            if s_rd_y >= s_gate_th then
                lown_pipe(0) <= '1';
            end if;
            for i in 1 to 4 loop lown_pipe(i) <= lown_pipe(i - 1); end loop;

            -- St2: luma-mod stage 1 — contrast multiply about mid (isolated).
            s2_lp <= (signed(resize(samp_y, 11)) - to_signed(512, 11))
                   * signed(resize(s_lgain, 10));
            s2_u  <= samp_u;
            s2_v  <= samp_v;

            -- St3: luma-mod stage 2 — recombine, clamp, lift to floor.
            v_lm := resize(shift_right(s2_lp, 7), 13) + to_signed(512, 13);
            if v_lm < 0 then v_lm := (others => '0'); end if;
            if v_lm > 1023 then v_lm := to_signed(1023, 13); end if;
            if v_lm < resize(signed('0' & s_floor), 13) then
                v_lm := resize(signed('0' & s_floor), 13);
            end if;
            s3_lm <= unsigned(v_lm(9 downto 0));
            s3_u  <= s2_u;
            s3_v  <= s2_v;

            -- St4: colorize stage 1 — duotone products (only used in mode 1).
            s4_pdy <= s_dd_y * signed('0' & s3_lm);
            s4_pdu <= s_dd_u * signed('0' & s3_lm);
            s4_pdv <= s_dd_v * signed('0' & s3_lm);
            s4_lm  <= s3_lm;
            s4_u   <= s3_u;
            s4_v   <= s3_v;

            -- St5: colorize stage 2 — assemble per mode.
            case to_integer(s_cmode) is
                when 1 =>      -- Duotone: lerp endpoints by shaped luma
                    v_y := clamp10(signed(resize(s_dlo_y, 12)) + resize(shift_right(s4_pdy, 10), 12));
                    v_u := clamp10(signed(resize(s_dlo_u, 12)) + resize(shift_right(s4_pdu, 10), 12));
                    v_v := clamp10(signed(resize(s_dlo_v, 12)) + resize(shift_right(s4_pdv, 10), 12));
                when 2 =>      -- Posterize: luma zone -> palette
                    v_idx := to_integer(s4_lm(9 downto 7));
                    if s_palsel = '0' then
                        v_y := RNB_Y(v_idx); v_u := RNB_U(v_idx); v_v := RNB_V(v_idx);
                    else
                        v_y := GRY_Y(v_idx); v_u := C_MID;        v_v := C_MID;
                    end if;
                when 3 =>      -- Hue-rotate: spin chroma about mid (90-deg steps)
                    v_cu := signed(resize(s4_u, 12)) - 512;
                    v_cv := signed(resize(s4_v, 12)) - 512;
                    case to_integer(s_hue) is
                        when 1 =>
                            v_u := clamp10(to_signed(512, 13) - resize(v_cv, 13));
                            v_v := clamp10(to_signed(512, 13) + resize(v_cu, 13));
                        when 2 =>
                            v_u := clamp10(to_signed(512, 13) - resize(v_cu, 13));
                            v_v := clamp10(to_signed(512, 13) - resize(v_cv, 13));
                        when 3 =>
                            v_u := clamp10(to_signed(512, 13) + resize(v_cv, 13));
                            v_v := clamp10(to_signed(512, 13) - resize(v_cu, 13));
                        when others =>
                            v_u := s4_u; v_v := s4_v;
                    end case;
                    v_y := s4_lm;
                when others => -- Original
                    v_y := s4_lm; v_u := s4_u; v_v := s4_v;
            end case;
            s5_y <= v_y; s5_u <= v_u; s5_v <= v_v;

            -- St6: grid / wireframe / luma-gate overlay.  Edge = g_pipe(5); each
            -- pixel is gated by its OWN triangle's luma (lown_pipe), so only complete
            -- lit triangles draw (fill + their own edges) — no partial outlines.
            -- Luma Mod is a GATE: only luma-positive triangles show.  s6_vid marks
            -- edges to be coloured from the live video at St7.
            s6_vid <= '0';
            if s_wire = '1' then
                if s_lumamod = '1' then
                    -- only the lit triangles' edges, K5, over black
                    if g_pipe(5) = '1' and lown_pipe(4) = '1' then
                        s6_y <= s_gy; s6_u <= s_gu; s6_v <= s_gv;
                    else
                        s6_y <= (others => '0'); s6_u <= C_MID; s6_v <= C_MID;
                    end if;
                else
                    -- full mesh, every edge coloured by the live incoming video
                    if g_pipe(5) = '1' then
                        s6_vid <= '1';
                        s6_y <= s_gy; s6_u <= s_gu; s6_v <= s_gv;   -- placeholder (St7 -> video)
                    else
                        s6_y <= (others => '0'); s6_u <= C_MID; s6_v <= C_MID;
                    end if;
                end if;
            elsif s_lumamod = '1' then
                -- non-wireframe + luma gate: show only luma-positive triangles
                -- (fill) and their surrounding grid lines; black elsewhere.
                if s_grid = '1' and g_pipe(5) = '1' then
                    if lown_pipe(4) = '1' then
                        s6_y <= s_gy; s6_u <= s_gu; s6_v <= s_gv;
                    else
                        s6_y <= (others => '0'); s6_u <= C_MID; s6_v <= C_MID;
                    end if;
                elsif lown_pipe(4) = '1' then
                    s6_y <= s5_y; s6_u <= s5_u; s6_v <= s5_v;
                else
                    s6_y <= (others => '0'); s6_u <= C_MID; s6_v <= C_MID;
                end if;
            elsif s_grid = '1' and g_pipe(5) = '1' then
                s6_y <= s_gy; s6_u <= s_gu; s6_v <= s_gv;
            else
                s6_y <= s5_y;
                s6_u <= s5_u;
                s6_v <= s5_v;
            end if;

            -- St7: output.  Bypass passes raw input; s6_vid edges take the live video
            -- colour (pipe is aligned with the output sync here).
            if s_bypass = '1' or s6_vid = '1' then
                s_io.y <= pipe(LATENCY - 1).y;
                s_io.u <= pipe(LATENCY - 1).u;
                s_io.v <= pipe(LATENCY - 1).v;
            else
                s_io.y <= std_logic_vector(s6_y);
                s_io.u <= std_logic_vector(s6_u);
                s_io.v <= std_logic_vector(s6_v);
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

end architecture triangulator;
