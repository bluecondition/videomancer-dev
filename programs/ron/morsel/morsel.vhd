-- morsel.vhd
--
-- MORSEL -- the whole screen is one chocolate-chip cookie, and the video is
-- baked into it.
--
-- Forked from the single "choc-chip cookie" swatch inside SUGARCOAT, which
-- was a 4-way-flipped triangle in an 8/16/32/64 px cell.  Here that one
-- material becomes the entire program, and the chip itself is rebuilt from a
-- 3-D model:
--
--   * A real morsel (flat circular base, sides bulging up, tip curling over)
--     is photographed from 8 different 3-D orientations by genchips.py and
--     each SILHOUETTE stored as a polar radius table R(theta), 128 steps.
--     Variant 0 is the classic side-on teardrop -- flat bottom, rounded
--     shoulders, hooked tip; variant 7 is nearly top-down -- a round chip
--     with a small point off one side.  K2 SHAPE slides which part of that
--     range the chips are drawn from.
--   * Every chip is rotated by a free 128-step angle (a subtract on the angle
--     index, so rotation is free and continuous), size-jittered +/-19%, and
--     position-jittered as far inside its cell as it can go without clipping.
--   * A rim light (fixed upper-left) shades each chip: a dome gradient over
--     the body, a tight specular glint on the lit shoulder, a dark contact
--     edge on the far side, and a soft shadow thrown onto the dough beside it.
--   * Chips and dough both carry fine hash grain, so nothing is a flat fill.
--
-- The picture is baked in twice over: the dough TONE follows the video luma
-- pixel-for-pixel (fine detail), and the chip DENSITY and SIZE follow a
-- per-cell luma sample (the coarse halftone that makes a face read across the
-- room).  S8 additionally lets the dough take the video's chroma.
--
-- Anti-lattice: a plain cell grid reads as rows of dots, and per-cell jitter
-- can never fix that -- a chip may only wander as far as the slack left in
-- its own cell, which is nearly nothing once the chips are large.  So every
-- cell COLUMN is slid vertically by its own pseudo-random phase.  The cell
-- boundaries move with the chip, nothing is ever clipped, and the rows are
-- gone.
--
-- Pipeline (streaming, C_LATENCY = 21):
--   1-5   cell lattice: column index -> column hash -> vertical phase -> cell
--         row, in-cell coordinates
--   6-9   cell hash -> rotation / variant / size / jitter / presence, and the
--         per-cell luma sample (ping-pong cell buffer, one cell row of lag)
--   10-12 polar: |cx|,|cy| -> C_RTAB -> exact radius and octant fraction ->
--         128-step screen angle
--   13-15 C_SHAPE silhouette radius, C_COS rim light, size multiply
--   16-17 signed distance inside the silhouette; coverage and shading
--         DECISIONS (compares and muxes only)
--   18-19 one shading add, one dough add, chip and dough colours
--   20-21 alpha scale, then composite
-- Multiplies (HX4K has no DSP): the size scale, the bake ramp, and the small
-- noise gains.  Everything else is compares, shifts and table reads.
--
-- Timing (HD needs 74.25 MHz): the first build ran at 57 MHz and every fix
-- came from profiling the nextpnr critical path, never from guessing.
--   1. The dough stage did two grain multiplies and five adds in one cycle.
--      The grain now finishes 14 stages earlier (it is free-running noise, so
--      it needs no alignment), the emboss is pre-shifted at insertion, and
--      the cast shadow is pre-applied into a second offset so stage 19 picks
--      one and adds once.  57 -> 66 MHz.
--   2. The VBLANK SEQUENCER became the critical path -- per-frame logic is
--      timed at the pixel clock like everything else.  Its chains were split
--      across slots, the jitter-room calculation became C_JMAX, and the slot
--      ranges were aligned to power-of-two boundaries so a table index is
--      just the low bits of s_seq.  66 -> 73 -> 93 MHz (pre-route).
--   3. Barrel-shifting the grain coordinates straight into the hash put
--      s_scl -> shift -> hash in one cone; the shift is now registered.
--
-- Colour convention: authored in STANDARD BT.601 (U = Cb, V = Cr) and swapped
-- at both ends, because the Videomancer carries chroma U<->V swapped (same as
-- sugarcoat/gilt).  Headless BT.601 sims therefore show the cookie blue.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.morsel_rom_pkg.all;

architecture morsel of program_top is

    constant C_LATENCY : integer := 21;
    constant C_LIGHT   : integer := 80;      -- rim light: 225 deg (upper-left)

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
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

    -- 16-bit xor-rotate-add hash, split in two halves so neither cone is deep
    -- NOTE the rotate: without it (a xor b) depends only on x xor y, which
    -- makes every cell on a diagonal share a hash and -- worse -- collapses
    -- the column phase below to a constant.  Measured: the rotate takes the
    -- diagonal correlation from 0.29 to -0.03.
    function f_pack(x, y : unsigned(11 downto 0);
                    seed : unsigned(15 downto 0)) return unsigned is
        variable a, b : unsigned(15 downto 0);
    begin
        a := x(9 downto 0) & y(5 downto 0);
        b := y(9 downto 0) & x(5 downto 0);
        return (a xor rotate_left(b, 5)) xor seed;
    end function;

    function f_mix1(a : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := a;
        h := h xor rotate_left(h, 7);
        h := h + rotate_left(h, 3);
        return h;
    end function;

    function f_mix2(a : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := a;
        h := h xor rotate_left(h, 11);
        h := h + rotate_left(h, 5);
        return h;
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms
    ----------------------------------------------------------------------
    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k4, s_k5, s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_p12 : unsigned(9 downto 0) := to_unsigned(620, 10);

    signal s_lumamod : std_logic := '1';                       -- S7
    signal s_tint    : std_logic := '0';                       -- S8
    signal s_inv     : std_logic := '0';                       -- S9
    signal s_emboss  : std_logic := '1';                       -- S10
    signal s_wchip   : std_logic := '0';                       -- S11

    signal s_scl  : integer range 0 to 4 := 3;      -- cell = 8 << s_scl
    signal s_sh   : integer range 3 to 7 := 6;      -- log2(cell)
    signal s_ms   : integer range 1 to 3 := 3;      -- grain block shift
    signal s_aa   : unsigned(5 downto 0) := to_unsigned(4, 6); -- AA half-width
    signal s_dens : unsigned(7 downto 0) := to_unsigned(155, 8);
    signal s_szb  : unsigned(6 downto 0) := to_unsigned(62, 7);
    signal s_jmax : unsigned(3 downto 0) := (others => '0');

    -- How far a chip may wander inside its cell, indexed by Chip Size >> 2.
    -- This is clamp((126 - 1.73*szb)/4, 0, 15) -- the slack left over once the
    -- biggest silhouette at the biggest size jitter has been placed.  It is a
    -- TABLE because working it out arithmetically took a multiply and three
    -- adds, and a vblank cone is timed at the pixel clock like everything
    -- else: that one chain alone held HD at 73 MHz.
    type t_jm is array (0 to 31) of unsigned(3 downto 0);
    constant C_JMAX : t_jm := (
        to_unsigned(15, 4), to_unsigned(15, 4), to_unsigned(15, 4), to_unsigned(15, 4),
        to_unsigned(15, 4), to_unsigned(15, 4), to_unsigned(15, 4), to_unsigned(15, 4),
        to_unsigned(15, 4), to_unsigned(15, 4), to_unsigned(13, 4), to_unsigned(11, 4),
        to_unsigned(10, 4), to_unsigned(8, 4),  to_unsigned(6, 4),  to_unsigned(4, 4),
        to_unsigned(3, 4),  to_unsigned(1, 4),  to_unsigned(0, 4),  to_unsigned(0, 4),
        to_unsigned(0, 4),  to_unsigned(0, 4),  to_unsigned(0, 4),  to_unsigned(0, 4),
        to_unsigned(0, 4),  to_unsigned(0, 4),  to_unsigned(0, 4),  to_unsigned(0, 4),
        to_unsigned(0, 4),  to_unsigned(0, 4),  to_unsigned(0, 4),  to_unsigned(0, 4));

    -- Shape-window offsets: vtab(j) = min(7, vlo + C_VOFF(j)).  A table, so
    -- the slot index needs no multiply.
    type t_vo is array (0 to 7) of integer range 0 to 4;
    constant C_VOFF : t_vo := (0, 0, 1, 1, 2, 3, 3, 4);
    signal s_blo  : unsigned(9 downto 0) := to_unsigned(373, 10);
    signal s_bspan : unsigned(9 downto 0) := to_unsigned(473, 10);
    signal s_rel  : integer range 0 to 2 := 1;
    signal s_spec : unsigned(8 downto 0) := to_unsigned(190, 9);
    signal s_grain : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_shad : unsigned(8 downto 0) := to_unsigned(60, 9);  -- cast shadow
    signal s_vlo  : unsigned(2 downto 0) := (others => '0');

    type t_sz is array (0 to 7) of unsigned(6 downto 0);
    signal s_sztab : t_sz := (others => to_unsigned(62, 7));
    type t_jt is array (0 to 15) of signed(5 downto 0);
    signal s_jtab : t_jt := (others => (others => '0'));
    type t_vt is array (0 to 7) of unsigned(2 downto 0);
    signal s_vtab : t_vt := (others => (others => '0'));

    signal s_seq : unsigned(5 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- raster measurement
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_avid_q     : std_logic := '0';
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt       : unsigned(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- chip pipeline registers (cN_* is aligned to pipeline stage N)
    ----------------------------------------------------------------------
    signal c1_cellx, c2_cellx, c3_cellx, c4_cellx, c5_cellx
        : unsigned(11 downto 0) := (others => '0');
    signal c1_nx, c2_nx, c3_nx, c4_nx, c5_nx, c6_nx, c7_nx
        : unsigned(5 downto 0) := (others => '0');
    signal c1_mid, c2_mid, c3_mid, c4_mid, c5_mid : std_logic := '0';

    signal c2_hca, c3_hcol : unsigned(15 downto 0) := (others => '0');
    signal c4_yo    : unsigned(11 downto 0) := (others => '0');
    signal c5_celly : unsigned(11 downto 0) := (others => '0');
    signal c5_ny, c6_ny, c7_ny : unsigned(5 downto 0) := (others => '0');
    signal c5_top   : std_logic := '0';
    signal c6_celly0 : std_logic := '0';

    signal c6_ha, c6_hb, c7_h1, c7_h2 : unsigned(15 downto 0) := (others => '0');

    -- cell-luma ping-pong buffers (one cell row of lag, so a chip never
    -- changes its mind about existing part-way down its own body)
    type t_cbuf is array (0 to 255) of std_logic_vector(7 downto 0);
    signal cbuf0, cbuf1 : t_cbuf;
    signal cb0_q, cb1_q : std_logic_vector(7 downto 0) := (others => '0');
    type t_cbd is array (0 to 3) of std_logic_vector(7 downto 0);
    signal cb0_d, cb1_d : t_cbd := (others => (others => '0'));
    signal c7_cluma : unsigned(7 downto 0) := (others => '0');

    signal c8_lm    : unsigned(7 downto 0) := (others => '0');
    signal c8_pres  : unsigned(7 downto 0) := (others => '0');
    signal c8_rot, c9_rot, c10_rot, c11_rot, c12_rot
        : unsigned(6 downto 0) := (others => '0');
    signal c8_var, c9_var, c10_var, c11_var, c12_var
        : unsigned(2 downto 0) := (others => '0');
    signal c8_szi   : unsigned(2 downto 0) := (others => '0');
    signal c8_cx, c8_cy : signed(7 downto 0) := (others => '0');
    signal c8_milk, c9_milk, c10_milk, c11_milk, c12_milk, c13_milk,
           c14_milk, c15_milk, c16_milk : std_logic := '0';
    signal c8_white, c9_white, c10_white, c11_white, c12_white, c13_white,
           c14_white, c15_white, c16_white : std_logic := '0';

    signal c9_present, c10_present, c11_present, c12_present, c13_present,
           c14_present, c15_present, c16_present : std_logic := '0';
    signal c9_szc, c10_szc, c11_szc, c12_szc, c13_szc, c14_szc
        : unsigned(6 downto 0) := (others => '0');
    signal c9_ax, c9_ay : unsigned(5 downto 0) := (others => '0');
    signal c9_px, c9_py : std_logic := '0';
    signal c10_px, c10_py, c10_sw, c10_hi : std_logic := '0';
    signal c11_px, c11_py, c11_sw, c11_hi : std_logic := '0';
    signal c10_addr : unsigned(9 downto 0) := (others => '0');
    signal c11_rq   : unsigned(11 downto 0) := (others => '0');

    -- pixel radius (quarter cell-units) vs the silhouette radius at this angle
    signal c12_pr, c13_pr, c14_pr, c15_pr : unsigned(8 downto 0) := (others => '0');
    signal c12_a7   : unsigned(6 downto 0) := (others => '0');
    signal c13_sadr : unsigned(9 downto 0) := (others => '0');
    signal c13_cadr : unsigned(6 downto 0) := (others => '0');
    signal c14_sq   : unsigned(7 downto 0) := (others => '0');
    signal c14_cq, c15_lit, c16_lit, c17_lit : signed(7 downto 0) := (others => '0');
    signal c15_sr, c16_sr : unsigned(8 downto 0) := (others => '0');
    signal c16_d    : signed(10 downto 0) := (others => '0');

    -- 17 is all compares and muxes; the shading ARITHMETIC is stage 18.  The
    -- band shift and the Relief shift are folded into ONE index (shade =
    -- lit >> (band - relief)) so stage 18 is a single mux plus one add.
    signal c17_alpha, c18_alpha : unsigned(2 downto 0) := (others => '0');
    signal c17_bsh   : integer range 0 to 4 := 2;     -- shift = c17_bsh - 1
    signal c17_ssel  : unsigned(1 downto 0) := (others => '0');
    signal c17_shadow, c18_shadow : std_logic := '0';
    signal c17_milk, c17_white : std_logic := '0';
    signal c18_milk, c18_white : std_logic := '0';
    signal c18_shade : signed(11 downto 0) := (others => '0');

    signal c18_basetex : signed(12 downto 0) := (others => '0');
    signal c18_cu, c18_cv : unsigned(9 downto 0) := (others => '0');
    signal c19_cy, c19_cu, c19_cv : unsigned(9 downto 0) := (others => '0');
    signal c19_dy, c19_du, c19_dv : unsigned(9 downto 0) := (others => '0');
    signal c20_dy, c20_du, c20_dv : unsigned(9 downto 0) := (others => '0');
    signal c20_my, c20_mu, c20_mv : signed(13 downto 0) := (others => '0');
    signal c21_y, c21_u, c21_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- dough: emboss line RAM + surface noise
    ----------------------------------------------------------------------
    type t_lram is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lram  : t_lram;
    signal lb_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal d1_y0, d1_y1 : unsigned(7 downto 0) := (others => '0');
    signal d2_gx, d2_gy : signed(8 downto 0) := (others => '0');
    signal d3_emb  : signed(7 downto 0) := (others => '0');
    -- pre-shifted by Relief at INSERTION, so the shift is not in the cone
    -- that finally adds it to the dough tone
    signal d4_embs : signed(9 downto 0) := (others => '0');
    type t_embsr is array (0 to 12) of signed(9 downto 0);
    signal emb_sr : t_embsr := (others => (others => '0'));

    -- surface noise: only ever has to be a stable function of (x,y), so it is
    -- taken from the live counters and needs no delay line of its own.  The
    -- grain multiplies happen HERE, far from the dough adder.
    signal nz_xs, nz_ys : unsigned(11 downto 0) := (others => '0');
    signal nz_a1, nz_b1, nz_a2, nz_b2 : unsigned(15 downto 0) := (others => '0');
    signal nz_g1, nz_g2 : signed(8 downto 0) := (others => '0');
    signal nz_pore : std_logic := '0';
    signal nz_o    : signed(8 downto 0) := (others => '0');   -- total grain
    signal nz_tex  : signed(5 downto 0) := (others => '0');   -- chip grain
    signal d17_prd : unsigned(17 downto 0) := (others => '0');
    signal d18_base : unsigned(9 downto 0) := (others => '0');
    signal d18_sfc0, d18_sfc1 : signed(11 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- video delay (internal standard BT.601: u = Cb, v = Cr)
    ----------------------------------------------------------------------
    type t_sr is array (0 to 17) of std_logic_vector(9 downto 0);
    signal s_y_sr, s_u_sr, s_v_sr : t_sr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                s_xcnt <= (others => '0');
                s_lcnt <= s_lcnt + 1;
            end if;
            if s_vs_pulse = '1' then
                s_lcnt <= (others => '0');
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer
    --
    -- Everything with a multiply or a ladder in it lives here, one item per
    -- vblank cycle, so the pixel path only ever sees a table read or a mux.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_sj   : integer range -32 to 31;
        variable v_jt   : signed(5 downto 0);
        variable v_c    : integer range -256 to 1023;
        variable v_m20  : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            if s_vs_pulse = '1' then
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_p12 <= unsigned(registers_in(7));
                s_lumamod <= registers_in(6)(0);          -- S7
                s_tint    <= registers_in(6)(1);          -- S8
                s_inv     <= registers_in(6)(2);          -- S9
                s_emboss  <= registers_in(6)(3);          -- S10
                s_wchip   <= registers_in(6)(4);          -- S11
                s_seq <= to_unsigned(63, 6);
            elsif s_seq /= 0 and data_in.avid = '0' then
                -- Slot ranges are aligned to power-of-two boundaries so the
                -- table index IS the low bits of s_seq: no subtract, and the
                -- range decode is bit-matching.  Nothing in here multiplies by
                -- a non-constant either -- a vblank cone is timed at the pixel
                -- clock, and this sequencer was the last thing capping HD.
                case to_integer(s_seq) is
                    when 63 =>
                        -- K1 SCALE: cell size 8/16/32/64/128 px
                        if    s_k1 < 205 then s_scl <= 0;
                        elsif s_k1 < 410 then s_scl <= 1;
                        elsif s_k1 < 640 then s_scl <= 2;
                        elsif s_k1 < 900 then s_scl <= 3;
                        else                  s_scl <= 4; end if;
                        -- K2 SHAPE: start of the 3-D orientation window
                        v_m20 := resize(shift_left(s_k2, 2) + s_k2, 20);
                        s_vlo <= v_m20(12 downto 10);
                    when 62 =>
                        -- matching anti-alias half-width: one screen pixel
                        -- expressed in the normalised 1/4-cell-unit space
                        s_sh <= 3 + s_scl;
                        if s_scl >= 2 then s_ms <= 3; else s_ms <= 1 + s_scl; end if;
                        case s_scl is
                            when 0      => s_aa <= to_unsigned(32, 6);
                            when 1      => s_aa <= to_unsigned(16, 6);
                            when 2      => s_aa <= to_unsigned(8, 6);
                            when 3      => s_aa <= to_unsigned(4, 6);
                            when others => s_aa <= to_unsigned(2, 6);
                        end case;
                    when 61 =>
                        -- K6 CHIP SIZE.  64 = the morsel exactly as modelled;
                        -- capped at 76 so the largest silhouette plus its size
                        -- jitter still fits inside a half cell.  (k6*56)>>10
                        -- written as ((k6<<3)-k6)>>7: one subtract.
                        v_m20 := resize(shift_left(s_k6, 3) - s_k6, 20);
                        s_szb <= to_unsigned(20 + to_integer(v_m20(12 downto 7)), 7);
                    when 60 =>
                        s_jmax <= C_JMAX(to_integer(s_szb(6 downto 2)));
                    when 40 to 47 =>
                        -- size jitter: szb * (1 + (j-4)/32), built from shifts
                        case to_integer(s_seq(2 downto 0)) is
                            when 0 => v_sj := -to_integer(shift_right(s_szb, 3));
                            when 1 => v_sj := -to_integer(shift_right(s_szb, 4))
                                              - to_integer(shift_right(s_szb, 5));
                            when 2 => v_sj := -to_integer(shift_right(s_szb, 4));
                            when 3 => v_sj := -to_integer(shift_right(s_szb, 5));
                            when 5 => v_sj :=  to_integer(shift_right(s_szb, 5));
                            when 6 => v_sj :=  to_integer(shift_right(s_szb, 4));
                            when 7 => v_sj :=  to_integer(shift_right(s_szb, 4))
                                              + to_integer(shift_right(s_szb, 5));
                            when others => v_sj := 0;
                        end case;
                        s_sztab(to_integer(s_seq(2 downto 0))) <=
                            to_unsigned(to_integer(s_szb) + v_sj, 7);
                    when 16 to 31 =>
                        -- position jitter: +/- jmax * {1, 3/4, 1/2, 1/4, 1/8},
                        -- every level one shift-add, and the 16 entries sum to
                        -- zero so the scatter stays centred
                        case to_integer(s_seq(3 downto 0)) is
                            when 0  => v_jt := -signed(resize(s_jmax, 6));
                            when 1  => v_jt := -signed(resize(shift_right(s_jmax, 1)
                                                + shift_right(s_jmax, 2), 6));
                            when 2  => v_jt := -signed(resize(shift_right(s_jmax, 1), 6));
                            when 3  => v_jt := -signed(resize(shift_right(s_jmax, 2), 6));
                            when 4  => v_jt := -signed(resize(shift_right(s_jmax, 3), 6));
                            when 5  => v_jt :=  signed(resize(shift_right(s_jmax, 3), 6));
                            when 6  => v_jt :=  signed(resize(shift_right(s_jmax, 2), 6));
                            when 7  => v_jt :=  signed(resize(shift_right(s_jmax, 1), 6));
                            when 8  => v_jt :=  signed(resize(shift_right(s_jmax, 1)
                                                + shift_right(s_jmax, 2), 6));
                            when 9  => v_jt :=  signed(resize(s_jmax, 6));
                            when 10 => v_jt := -signed(resize(shift_right(s_jmax, 1)
                                                + shift_right(s_jmax, 2), 6));
                            when 11 => v_jt :=  signed(resize(shift_right(s_jmax, 1), 6));
                            when 12 => v_jt := -signed(resize(shift_right(s_jmax, 2), 6));
                            when 13 => v_jt :=  signed(resize(shift_right(s_jmax, 1)
                                                + shift_right(s_jmax, 2), 6));
                            when 14 => v_jt := -signed(resize(shift_right(s_jmax, 1), 6));
                            when others => v_jt := signed(resize(shift_right(s_jmax, 2), 6));
                        end case;
                        s_jtab(to_integer(s_seq(3 downto 0))) <= v_jt;
                    when 8 to 15 =>
                        -- which slice of the 8 modelled 3-D orientations the
                        -- hash may draw from: low = side-on flat-bottomed
                        -- teardrops, high = round chips with a small point
                        v_c := to_integer(s_vlo) + C_VOFF(to_integer(s_seq(2 downto 0)));
                        if v_c > 7 then v_c := 7; end if;
                        s_vtab(to_integer(s_seq(2 downto 0))) <= to_unsigned(v_c, 3);
                    when 7 =>
                        s_dens <= s_p12(9 downto 2);
                    when 6 =>
                        -- K3 BAKE: pale golden dough -> dark baked.  Both ends
                        -- stay cookie-coloured; the chips carry the contrast.
                        s_blo <= to_unsigned(480 - to_integer(s_k3(9 downto 2)), 10);
                    when 5 =>
                        s_bspan <= to_unsigned(420 + to_integer(s_k3(9 downto 3)), 10);
                    when 4 =>
                        if    s_k4 >= 683 then s_rel <= 2;
                        elsif s_k4 >= 341 then s_rel <= 1;
                        else                   s_rel <= 0; end if;
                    when 3 =>
                        -- (k4*320)>>10 = (k4*5)>>4: one add, not a multiply
                        v_m20 := resize(shift_left(s_k4, 2) + s_k4, 20);
                        s_spec <= v_m20(12 downto 4);
                    when 2 =>
                        s_grain <= resize(s_k5(9 downto 7), 4);
                    when 1 =>
                        s_shad <= shift_left(to_unsigned(30, 9), s_rel);
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- CHIP PIPELINE
    ------------------------------------------------------------------------
    p_chip : process(clk)
        variable v_nx, v_ny   : unsigned(5 downto 0);
        variable v_sub        : unsigned(3 downto 0);
        variable v_a          : unsigned(7 downto 0);
        variable v_mx, v_mn   : unsigned(5 downto 0);
        variable v_mxs, v_mns : unsigned(4 downto 0);
        variable v_prod       : unsigned(15 downto 0);
        variable v_lm         : unsigned(7 downto 0);
        variable v_thr, v_sum : unsigned(8 downto 0);
        variable v_sel        : std_logic_vector(2 downto 0);
        variable v_band       : integer range 1 to 3;
        variable v_sh0        : signed(11 downto 0);
        variable v_vo         : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then

            ------------------------------------------------------------
            -- 1: cell column, in-cell x, cell-centre-column flag
            ------------------------------------------------------------
            c1_cellx <= shift_right(s_xcnt, s_sh);
            case s_scl is
                when 0      => v_nx := s_xcnt(2 downto 0) & "000";
                when 1      => v_nx := s_xcnt(3 downto 0) & "00";
                when 2      => v_nx := s_xcnt(4 downto 0) & '0';
                when 3      => v_nx := s_xcnt(5 downto 0);
                when others => v_nx := s_xcnt(6 downto 1);
            end case;
            c1_nx <= v_nx;
            -- centre column of the cell: where the per-cell luma is sampled
            if v_nx = 32 then c1_mid <= '1'; else c1_mid <= '0'; end if;

            ------------------------------------------------------------
            -- 2-3: column hash (this column's vertical phase)
            ------------------------------------------------------------
            c2_hca  <= f_mix1(f_pack(c1_cellx, c1_cellx + 733, x"13D7"));
            c3_hcol <= f_mix2(c2_hca);

            ------------------------------------------------------------
            -- 4: slide this whole column down by its own phase.  The cell
            -- boundaries move with it, so nothing is clipped -- this is what
            -- removes the horizontal rows a plain lattice produces.
            ------------------------------------------------------------
            if s_sh >= 6 then
                v_vo := shift_left(resize(c3_hcol(8 downto 3), 12), s_sh - 6);
            else
                v_vo := shift_right(resize(c3_hcol(8 downto 3), 12), 6 - s_sh);
            end if;
            c4_yo <= s_lcnt + v_vo;

            ------------------------------------------------------------
            -- 5: cell row + in-cell y
            ------------------------------------------------------------
            c5_celly <= shift_right(c4_yo, s_sh);
            case s_scl is
                when 0      => v_ny := c4_yo(2 downto 0) & "000";
                when 1      => v_ny := c4_yo(3 downto 0) & "00";
                when 2      => v_ny := c4_yo(4 downto 0) & '0';
                when 3      => v_ny := c4_yo(5 downto 0);
                when others => v_ny := c4_yo(6 downto 1);
            end case;
            c5_ny <= v_ny;
            if v_ny = 0 then c5_top <= '1'; else c5_top <= '0'; end if;

            ------------------------------------------------------------
            -- 6-7: cell hash (two 16-bit mixes give the 31 bits a chip needs)
            ------------------------------------------------------------
            c6_ha <= f_mix1(f_pack(c5_cellx, c5_celly, x"5A3C"));
            c6_hb <= f_mix1(f_pack(c5_cellx, c5_celly, x"9E37"));
            c7_h1 <= f_mix2(c6_ha);
            c7_h2 <= f_mix2(c6_hb);
            c6_celly0 <= c5_celly(0);

            -- pick the cell-luma bank the PREVIOUS cell row wrote
            if c6_celly0 = '0' then c7_cluma <= unsigned(cb1_d(3));
            else                    c7_cluma <= unsigned(cb0_d(3)); end if;

            ------------------------------------------------------------
            -- 8: unpack the cell's chip
            ------------------------------------------------------------
            if s_inv = '1' then v_lm := c7_cluma;
            else                v_lm := 255 - c7_cluma; end if;
            c8_lm    <= v_lm;
            c8_pres  <= c7_h1(7 downto 0);
            c8_rot   <= c7_h1(14 downto 8);
            c8_var   <= s_vtab(to_integer(c7_h2(2 downto 0)));
            c8_szi   <= c7_h2(5 downto 3);
            c8_cx    <= (signed(resize(c7_nx, 8)) - 32)
                        - resize(s_jtab(to_integer(c7_h2(9 downto 6))), 8);
            c8_cy    <= (signed(resize(c7_ny, 8)) - 32)
                        - resize(s_jtab(to_integer(c7_h2(13 downto 10))), 8);
            c8_milk  <= c7_h2(14);
            c8_white <= c7_h2(15);

            ------------------------------------------------------------
            -- 9: presence + size (both modulated by the cell's luma), and
            --    the first half of the polar conversion
            ------------------------------------------------------------
            if s_lumamod = '1' then
                v_sum := ('0' & s_dens) + ('0' & c8_lm);
                if v_sum < 128 then v_thr := (others => '0');
                else                v_thr := v_sum - 128; end if;
                c9_szc <= s_sztab(to_integer(c8_szi))
                          + resize(c8_lm(7 downto 4), 7);
            else
                v_thr := '0' & s_dens;
                c9_szc <= s_sztab(to_integer(c8_szi));
            end if;
            if ('0' & c8_pres) < v_thr then c9_present <= '1';
            else                            c9_present <= '0'; end if;

            if c8_cx < 0 then c9_ax <= resize(unsigned(-c8_cx), 6); c9_px <= '0';
            else              c9_ax <= resize(unsigned(c8_cx), 6);  c9_px <= '1';
            end if;
            if c8_cy < 0 then c9_ay <= resize(unsigned(-c8_cy), 6); c9_py <= '0';
            else              c9_ay <= resize(unsigned(c8_cy), 6);  c9_py <= '1';
            end if;

            ------------------------------------------------------------
            -- 10: min/max + the >=32 halve, forming the C_RTAB address
            ------------------------------------------------------------
            if c9_ay > c9_ax then
                v_mx := c9_ay; v_mn := c9_ax; c10_sw <= '1';
            else
                v_mx := c9_ax; v_mn := c9_ay; c10_sw <= '0';
            end if;
            if v_mx >= 32 then
                v_mxs := v_mx(5 downto 1); v_mns := v_mn(5 downto 1);
                c10_hi <= '1';
            else
                v_mxs := v_mx(4 downto 0); v_mns := v_mn(4 downto 0);
                c10_hi <= '0';
            end if;
            c10_addr <= v_mns & v_mxs;

            ------------------------------------------------------------
            -- 11: C_RTAB -- exact radius*4 and the octant fraction, one read
            ------------------------------------------------------------
            c11_rq <= C_RTAB(to_integer(c10_addr));

            ------------------------------------------------------------
            -- 12: radius + 128-step screen angle (from +x toward +y)
            ------------------------------------------------------------
            if c11_hi = '1' then
                c12_pr <= c11_rq(11 downto 4) & '0';
            else
                c12_pr <= '0' & c11_rq(11 downto 4);
            end if;
            v_sub := c11_rq(3 downto 0);
            v_sel := c11_px & c11_py & c11_sw;
            case v_sel is
                when "110"  => v_a := to_unsigned(0, 8) + v_sub;
                when "111"  => v_a := to_unsigned(32, 8) - v_sub;
                when "011"  => v_a := to_unsigned(32, 8) + v_sub;
                when "010"  => v_a := to_unsigned(64, 8) - v_sub;
                when "000"  => v_a := to_unsigned(64, 8) + v_sub;
                when "001"  => v_a := to_unsigned(96, 8) - v_sub;
                when "101"  => v_a := to_unsigned(96, 8) + v_sub;
                when others => v_a := to_unsigned(128, 8) - v_sub;
            end case;
            c12_a7 <= v_a(6 downto 0);

            ------------------------------------------------------------
            -- 13: rotate the SHAPE lookup (free: a subtract on the angle
            --     index) and aim the rim light, which is fixed in screen
            --     space and so uses the UNrotated angle
            ------------------------------------------------------------
            c13_sadr <= c12_var & (c12_a7 - c12_rot);
            c13_cadr <= c12_a7 - to_unsigned(C_LIGHT, 7);

            ------------------------------------------------------------
            -- 14: silhouette radius + rim light cosine
            ------------------------------------------------------------
            c14_sq <= C_SHAPE(to_integer(c13_sadr));
            c14_cq <= C_COS(to_integer(c13_cadr));

            ------------------------------------------------------------
            -- 15: apply this chip's size
            ------------------------------------------------------------
            v_prod := resize(c14_sq * c14_szc, 16);
            if v_prod(15 downto 6) > 511 then
                c15_sr <= to_unsigned(511, 9);
            else
                c15_sr <= v_prod(14 downto 6);
            end if;

            ------------------------------------------------------------
            -- 16: signed distance inside the silhouette
            ------------------------------------------------------------
            c16_d  <= resize(signed('0' & c15_sr), 11)
                      - resize(signed('0' & c15_pr), 11);
            c16_sr <= c15_sr;

            ------------------------------------------------------------
            -- 17: coverage (5-step AA) and the shading DECISIONS.  Nothing
            --     arithmetic happens here -- the dome band and the Relief
            --     gain are folded into one shift index so stage 18 is a
            --     single mux plus one add.
            ------------------------------------------------------------
            if c16_present = '0' then
                c17_alpha <= "000";
            elsif c16_d >= signed('0' & resize(s_aa, 10)) then
                c17_alpha <= "100";
            elsif c16_d >= signed('0' & resize(s_aa(5 downto 1), 10)) then
                c17_alpha <= "011";
            elsif c16_d >= 0 then
                c17_alpha <= "010";
            elsif c16_d >= -signed('0' & resize(s_aa(5 downto 1), 10)) then
                c17_alpha <= "001";
            else
                c17_alpha <= "000";
            end if;

            -- dome: the surface normal swings outward toward the rim, so the
            -- lit side brightens and the far side darkens near the edge.
            -- band shift is 1 (rim) / 2 / 3 (core); Relief shifts back up.
            if c16_d < signed('0' & resize(c16_sr(8 downto 3), 10)) then
                v_band := 1;
            elsif c16_d < signed('0' & resize(c16_sr(8 downto 2), 10)) then
                v_band := 2;
            else
                v_band := 3;
            end if;
            if v_band - s_rel < -1 then c17_bsh <= 0;
            elsif v_band - s_rel > 3 then c17_bsh <= 4;
            else                         c17_bsh <= v_band - s_rel + 1;
            end if;

            -- a tight glint on the lit shoulder, a dark contact edge opposite
            if c16_d < signed('0' & resize(c16_sr(8 downto 4), 10)) then
                if    c16_lit > 104 then c17_ssel <= "01";
                elsif c16_lit < -80 then c17_ssel <= "10";
                else                     c17_ssel <= "00"; end if;
            else
                c17_ssel <= "00";
            end if;

            -- the shadow this chip throws onto the dough beside it
            if c16_present = '1' and c16_d < 0
               and c16_d > -signed('0' & resize(s_aa & "00", 10))
               and c16_lit < -50 then
                c17_shadow <= '1';
            else
                c17_shadow <= '0';
            end if;
            c17_milk  <= c16_milk;
            c17_white <= c16_white;

            ------------------------------------------------------------
            -- 18: the one shading add
            ------------------------------------------------------------
            case c17_bsh is
                when 0      => v_sh0 := shift_left(resize(c17_lit, 12), 1);
                when 1      => v_sh0 := resize(c17_lit, 12);
                when 2      => v_sh0 := shift_right(resize(c17_lit, 12), 1);
                when 3      => v_sh0 := shift_right(resize(c17_lit, 12), 2);
                when others => v_sh0 := shift_right(resize(c17_lit, 12), 3);
            end case;
            case c17_ssel is
                when "01"   => c18_shade <= v_sh0 + signed('0' & resize(s_spec, 11));
                when "10"   => c18_shade <= v_sh0 - 60;
                when others => c18_shade <= v_sh0;
            end case;
            c18_alpha  <= c17_alpha;
            c18_shadow <= c17_shadow;
            c18_milk   <= c17_milk;
            c18_white  <= c17_white;

            ------------------------------------------------------------
            -- stage carries
            ------------------------------------------------------------
            c2_cellx <= c1_cellx;  c3_cellx <= c2_cellx;
            c4_cellx <= c3_cellx;  c5_cellx <= c4_cellx;
            c2_nx <= c1_nx; c3_nx <= c2_nx; c4_nx <= c3_nx; c5_nx <= c4_nx;
            c6_nx <= c5_nx; c7_nx <= c6_nx;
            c6_ny <= c5_ny; c7_ny <= c6_ny;
            c2_mid <= c1_mid; c3_mid <= c2_mid; c4_mid <= c3_mid;
            c5_mid <= c4_mid;
            c9_rot <= c8_rot;   c10_rot <= c9_rot;
            c11_rot <= c10_rot; c12_rot <= c11_rot;
            c9_var <= c8_var;   c10_var <= c9_var;
            c11_var <= c10_var; c12_var <= c11_var;
            c9_milk <= c8_milk; c10_milk <= c9_milk; c11_milk <= c10_milk;
            c12_milk <= c11_milk; c13_milk <= c12_milk; c14_milk <= c13_milk;
            c15_milk <= c14_milk; c16_milk <= c15_milk;
            c9_white <= c8_white; c10_white <= c9_white; c11_white <= c10_white;
            c12_white <= c11_white; c13_white <= c12_white;
            c14_white <= c13_white; c15_white <= c14_white;
            c16_white <= c15_white;
            c10_present <= c9_present; c11_present <= c10_present;
            c12_present <= c11_present; c13_present <= c12_present;
            c14_present <= c13_present; c15_present <= c14_present;
            c16_present <= c15_present;
            c10_szc <= c9_szc; c11_szc <= c10_szc; c12_szc <= c11_szc;
            c13_szc <= c12_szc; c14_szc <= c13_szc;
            c10_px <= c9_px; c10_py <= c9_py;
            c11_px <= c10_px; c11_py <= c10_py;
            c11_sw <= c10_sw; c11_hi <= c10_hi;
            c13_pr <= c12_pr; c14_pr <= c13_pr; c15_pr <= c14_pr;
            c15_lit <= c14_cq; c16_lit <= c15_lit; c17_lit <= c16_lit;
        end if;
    end process p_chip;

    ------------------------------------------------------------------------
    -- cell-luma buffers: one 8-bit sample per cell column, taken at the top
    -- line of each cell row and read back through the whole of the NEXT one.
    -- Holding it steady is what stops a chip appearing or growing part-way
    -- down its own body as the video under it changes.
    ------------------------------------------------------------------------
    p_cellbuf : process(clk)
    begin
        if rising_edge(clk) then
            cb0_q <= cbuf0(to_integer(c1_cellx(7 downto 0)));
            cb1_q <= cbuf1(to_integer(c1_cellx(7 downto 0)));
            if c5_top = '1' and c5_mid = '1' then
                if c5_celly(0) = '0' then
                    cbuf0(to_integer(c5_cellx(7 downto 0))) <= s_y_sr(4)(9 downto 2);
                else
                    cbuf1(to_integer(c5_cellx(7 downto 0))) <= s_y_sr(4)(9 downto 2);
                end if;
            end if;
            cb0_d(0) <= cb0_q; cb1_d(0) <= cb1_q;
            for i in 1 to 3 loop
                cb0_d(i) <= cb0_d(i - 1);
                cb1_d(i) <= cb1_d(i - 1);
            end loop;
        end if;
    end process p_cellbuf;

    ------------------------------------------------------------------------
    -- DOUGH: bake ramp from the video luma, hash grain, and a relief emboss
    -- driven by the video's own gradient, so the cookie surface is sculpted
    -- by the picture rather than merely tinted by it.
    ------------------------------------------------------------------------
    p_dough : process(clk)
        variable v_e   : signed(9 downto 0);
        variable v_bs  : unsigned(10 downto 0);
        variable v_g   : signed(5 downto 0);
        variable v_n   : signed(5 downto 0);
        variable v_du, v_dv : signed(11 downto 0);
        variable v_sfc : signed(11 downto 0);
        variable v_y   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- previous line's luma (read before write, same address)
            if data_in.avid = '1' then
                lb_q <= lram(to_integer(s_xcnt(10 downto 0)));
                lram(to_integer(s_xcnt(10 downto 0))) <= data_in.y(9 downto 2);
            end if;
            d1_y0 <= unsigned(data_in.y(9 downto 2));
            d1_y1 <= d1_y0;

            d2_gx <= signed('0' & d1_y0) - signed('0' & d1_y1);
            d2_gy <= signed('0' & d1_y0) - signed('0' & unsigned(lb_q));

            v_e := resize(d2_gx, 10) + resize(d2_gy, 10);
            if    v_e >  80 then d3_emb <= to_signed(80, 8);
            elsif v_e < -80 then d3_emb <= to_signed(-80, 8);
            else                 d3_emb <= resize(v_e, 8); end if;

            -- pre-shift by Relief HERE, not where it meets the dough tone
            d4_embs <= shift_left(resize(d3_emb, 10), s_rel);
            emb_sr(0) <= d4_embs;
            for i in 1 to 12 loop
                emb_sr(i) <= emb_sr(i - 1);
            end loop;

            -- surface noise, from the live counters (a fixed function of x,y;
            -- it is never compared against anything, so its phase is free).
            -- Both grain multiplies and the pore test finish here, 14 stages
            -- before the dough adder that consumes them -- putting them in
            -- that adder's cone was what held HD at 57 MHz.
            -- the block shift is REGISTERED before the hash: barrel-shifting
            -- straight into f_pack put s_scl -> shift -> hash in one cone and
            -- held HD at 72 MHz.  The noise field is free-running, so the
            -- extra cycle costs nothing.
            nz_xs <= shift_right(s_xcnt, s_ms);
            nz_ys <= shift_right(s_lcnt, s_ms);
            nz_a1 <= f_mix1(f_pack(s_xcnt, s_lcnt, x"1234"));
            nz_b1 <= f_mix1(f_pack(nz_xs, nz_ys, x"77A1"));
            nz_a2 <= f_mix2(nz_a1);
            nz_b2 <= f_mix2(nz_b1);

            v_g := signed(resize(s_grain, 6));
            v_n := signed(resize(nz_a2(3 downto 0), 6)) - 8;
            nz_g1 <= resize(shift_right(v_n * v_g, 1), 9);        -- fine grain
            v_n := signed(resize(nz_b2(7 downto 4), 6)) - 8;
            nz_g2 <= resize(shift_right(v_n * v_g, 2), 9);        -- browning
            if nz_a2(15 downto 12) < s_grain then nz_pore <= '1';
            else                                  nz_pore <= '0'; end if;
            -- chocolate is not a flat fill either
            nz_tex <= resize((signed(resize(nz_a2(8 downto 6), 5)) - 4) & '0', 6)
                      + resize(signed(resize(nz_b2(11 downto 9), 5)) - 4, 6);

            if nz_pore = '1' then
                nz_o <= nz_g1 + nz_g2 - 60;
            else
                nz_o <= nz_g1 + nz_g2;
            end if;

            -- 17: bake ramp multiply
            d17_prd <= unsigned(s_y_sr(15)(9 downto 2)) * s_bspan;

            -- 18: dough tone base, and the two surface offsets (with and
            -- without a cast shadow) so stage 19 is one add and a clamp
            v_bs := ('0' & s_blo) + ('0' & d17_prd(17 downto 8));
            if v_bs > 1023 then d18_base <= to_unsigned(1023, 10);
            else                d18_base <= v_bs(9 downto 0); end if;
            d18_sfc0 <= resize(nz_o, 12) + resize(emb_sr(12), 12);
            d18_sfc1 <= resize(nz_o, 12) + resize(emb_sr(12), 12)
                        - signed('0' & resize(s_shad, 11));

            -- 19: dough tone
            if c18_shadow = '1' then v_sfc := d18_sfc1;
            else                     v_sfc := d18_sfc0; end if;
            v_y := signed(resize(d18_base, 13)) + resize(v_sfc, 13);
            c19_dy <= f_cu10(v_y);

            -- dough chroma follows its own tone: paler dough is more yellow,
            -- darker dough redder.  S8 lets the video's chroma bleed in.
            if s_tint = '1' then
                v_du := shift_right(signed(resize(unsigned(s_u_sr(17)), 12))
                                    - 512, 1);
                v_dv := shift_right(signed(resize(unsigned(s_v_sr(17)), 12))
                                    - 512, 1);
            else
                v_du := (others => '0');
                v_dv := (others => '0');
            end if;
            if    d18_base < 256 then
                c19_du <= f_cu10(to_signed(396, 12) + v_du);
                c19_dv <= f_cu10(to_signed(628, 12) + v_dv);
            elsif d18_base < 512 then
                c19_du <= f_cu10(to_signed(392, 12) + v_du);
                c19_dv <= f_cu10(to_signed(618, 12) + v_dv);
            elsif d18_base < 768 then
                c19_du <= f_cu10(to_signed(388, 12) + v_du);
                c19_dv <= f_cu10(to_signed(606, 12) + v_dv);
            else
                c19_du <= f_cu10(to_signed(396, 12) + v_du);
                c19_dv <= f_cu10(to_signed(590, 12) + v_dv);
            end if;
        end if;
    end process p_dough;

    ------------------------------------------------------------------------
    -- CHIP COLOUR + COMPOSITE
    ------------------------------------------------------------------------
    p_compose : process(clk)
        variable v_base : signed(12 downto 0);
        variable v_dif  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            ------------------------------------------------------------
            -- 18: chip base colour, pre-biased with its grain so stage 19
            --     is a single add (see the pulp critical-path lesson)
            ------------------------------------------------------------
            if c17_white = '1' and s_wchip = '1' then
                v_base := to_signed(860, 13);                 -- white chocolate
                c18_cu <= to_unsigned(424, 10);
                c18_cv <= to_unsigned(556, 10);
            elsif c17_milk = '1' then
                v_base := to_signed(196, 13);                 -- milk
                c18_cu <= to_unsigned(470, 10);
                c18_cv <= to_unsigned(566, 10);
            else
                v_base := to_signed(148, 13);                 -- dark
                c18_cu <= to_unsigned(470, 10);
                c18_cv <= to_unsigned(566, 10);
            end if;
            c18_basetex <= v_base + resize(nz_tex, 13);

            ------------------------------------------------------------
            -- 19: the chip's final colour
            ------------------------------------------------------------
            c19_cy <= f_cu10(c18_basetex + resize(c18_shade, 13));
            c19_cu <= c18_cu;
            c19_cv <= c18_cv;

            ------------------------------------------------------------
            -- 20: alpha scale (alpha spans 0..4, so a 3-term shift-add,
            --     never a multiplier)
            ------------------------------------------------------------
            v_dif := resize(signed('0' & c19_cy), 12)
                     - resize(signed('0' & c19_dy), 12);
            case c18_alpha is
                when "000"  => c20_my <= (others => '0');
                when "001"  => c20_my <= resize(v_dif, 14);
                when "010"  => c20_my <= shift_left(resize(v_dif, 14), 1);
                when "011"  => c20_my <= shift_left(resize(v_dif, 14), 1)
                                         + resize(v_dif, 14);
                when others => c20_my <= shift_left(resize(v_dif, 14), 2);
            end case;
            v_dif := resize(signed('0' & c19_cu), 12)
                     - resize(signed('0' & c19_du), 12);
            case c18_alpha is
                when "000"  => c20_mu <= (others => '0');
                when "001"  => c20_mu <= resize(v_dif, 14);
                when "010"  => c20_mu <= shift_left(resize(v_dif, 14), 1);
                when "011"  => c20_mu <= shift_left(resize(v_dif, 14), 1)
                                         + resize(v_dif, 14);
                when others => c20_mu <= shift_left(resize(v_dif, 14), 2);
            end case;
            v_dif := resize(signed('0' & c19_cv), 12)
                     - resize(signed('0' & c19_dv), 12);
            case c18_alpha is
                when "000"  => c20_mv <= (others => '0');
                when "001"  => c20_mv <= resize(v_dif, 14);
                when "010"  => c20_mv <= shift_left(resize(v_dif, 14), 1);
                when "011"  => c20_mv <= shift_left(resize(v_dif, 14), 1)
                                         + resize(v_dif, 14);
                when others => c20_mv <= shift_left(resize(v_dif, 14), 2);
            end case;
            c20_dy <= c19_dy; c20_du <= c19_du; c20_dv <= c19_dv;

            ------------------------------------------------------------
            -- 21: composite
            ------------------------------------------------------------
            c21_y <= f_cu10(signed(resize(c20_dy, 13))
                            + resize(shift_right(c20_my, 2), 13));
            c21_u <= f_cu10(signed(resize(c20_du, 13))
                            + resize(shift_right(c20_mu, 2), 13));
            c21_v <= f_cu10(signed(resize(c20_dv, 13))
                            + resize(shift_right(c20_mv, 2), 13));
        end if;
    end process p_compose;

    ------------------------------------------------------------------------
    -- video delay (with the hardware U/V swap into the internal convention)
    ------------------------------------------------------------------------
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_y_sr(0) <= data_in.y;
            s_u_sr(0) <= data_in.v;                -- HW v carries Cb
            s_v_sr(0) <= data_in.u;                -- HW u carries Cr
            for i in 1 to 17 loop
                s_y_sr(i) <= s_y_sr(i - 1);
                s_u_sr(i) <= s_u_sr(i - 1);
                s_v_sr(i) <= s_v_sr(i - 1);
            end loop;
        end if;
    end process p_delay;

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

    data_out.y       <= std_logic_vector(c21_y);
    data_out.u       <= std_logic_vector(c21_v);   -- swap back out to HW
    data_out.v       <= std_logic_vector(c21_u);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture morsel;
